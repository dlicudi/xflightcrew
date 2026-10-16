//! The loop: `run`, whole.

use super::*;

pub async fn run(
    config: Config,
    live: std::sync::Arc<crate::LiveProfile>,
    voice: Voice,
    status: Option<StatusSink>,
    cabin: std::sync::Arc<std::sync::Mutex<crate::cabin::Cabin>>,
) {
    // Checks are built from the active profile and rebuilt when the aircraft
    // switches (the generation counter changes), so a jet doesn't keep a 172's
    // stall/overspeed limits after a profile swap.
    let advisor_drefs = |p: &AircraftProfile| -> Vec<String> {
        p.advisor.iter().map(|a| a.dataref.clone()).collect()
    };
    let mut gen = live.generation();
    let mut checks = build_checks(&live.get());
    let mut drefs = advisor_drefs(&live.get());
    // The tanks the centre-pump check reads, and their lateral positions
    // (static per airframe, read once per swap).
    let mut pump_tanks = live.get().fuel_tank_pumps_off_when_empty.clone();
    let mut tank_lateral: Option<Vec<f64>> = None;
    // The ice picture (#100), read on its own slow clock in cold air, and
    // since when the air has been clear of icing (the crew then switches off
    // what it switched on).
    let mut icing_read: Option<(Instant, crate::tools::anti_ice::IcingRead)> = None;
    let mut icing_clear_since: Option<Instant> = None;
    // The last icing verdict blackboxed (`icing` `conditions`), so a change
    // is one record with the numbers it was judged on.
    let mut icing_logged: Option<bool> = None;
    let mut avionics_readback = avionics_readback_dref(&live.get());
    // The cockpit duties the copilot performs unasked, per airframe: interior
    // lights flagged `auto_at_night`, and a GPS welcome page to acknowledge.
    // Collected at rebuild so an airframe declaring NEITHER never pays a read.
    let cockpit_duties =
        |p: &AircraftProfile| -> (Vec<crate::aircraft::LightProfile>, Option<String>) {
            (
                p.lights
                    .iter()
                    .filter(|l| l.auto_at_night)
                    .cloned()
                    .collect(),
                p.gps_ack_command.clone(),
            )
        };
    let (mut night_lights, mut gps_ack) = cockpit_duties(&live.get());
    // Whether THIS airframe carries a crew, rebuilt on aircraft switch; the
    // live staffing setting is read at the watch itself (cabin::aboard), so
    // a panel flip applies mid-flight without a profile swap.
    let mut cabin_capable = live.get().cabin_crew;
    tracing::info!(
        checks = checks.len(),
        hz = config.monitor_hz,
        "monitor started"
    );
    // Stamp the record with the airframe so the distiller can key memory by it.
    crate::blackbox::record(
        "flight_meta",
        serde_json::json!({ "aircraft": live.get().id }),
    );

    let xp = XPlaneClient::new(config.xplane_base_url.clone());
    let period = Duration::from_secs_f32(1.0 / config.monitor_hz.max(0.1));
    let mut states: HashMap<&'static str, FireState> =
        checks.iter().map(|c| (c.id(), FireState::new())).collect();

    // IAS history for the rate-of-decay term. `trend` is an EMA (kts/s) so a
    // single noisy sample doesn't read as a sharp washout.
    let mut prev_ias: Option<(f64, Instant)> = None;
    let mut trend = 0.0_f64;
    // The flight model (`flight::Flight`): the one state every consumer reads.
    let mut flight = crate::flight::Flight::new();
    // The duty engine's last answer, logged on change (`duty.rs`).
    let mut duty_trail = crate::duty::DutyTrail::default();
    // The State tab's last text, and its resend countdown.
    let mut last_state_text = String::new();
    let mut state_resend: u8 = 0;
    // Pushback progress is a sequence of *events*, not a sustained condition, so
    // it's called out on transitions here rather than as Checks.
    let mut prev_attached = false;
    let mut prev_pushing = false;
    let mut push_called = false;
    // Last strip pushed to the panel — only re-send on change.
    let mut last_strip = String::new();
    let mut last_card = String::new();
    // Last automation feed pushed, plus a tick countdown that forces a
    // periodic resend: a plugin reload zeroes its buffers, and a purely
    // change-gated publish would leave the Flight tab blank until the next
    // real change (the aircraft-chip heartbeat is the same lesson).
    let mut last_auto = String::new();
    let mut last_sheet = String::new();
    let mut auto_resend = 0u8;
    // When the Settings tab's resolved callsign was last published.
    let mut callsign_at: Option<Instant> = None;
    // The Settings tab's EFB link and QR (2026-10-13): the link it was last
    // built from (`None` = never), and the payload, rebuilt only on a change.
    let mut last_efb: Option<Option<String>> = None;
    let mut efb_payload = String::new();
    // The Checklist tab's change gate (`checklist_panel::ChecklistTab`).
    let mut checklist_tab = crate::checklist_panel::ChecklistTab::default();

    // Talk-down pacing: when the last callout went out, and whether we've
    // already handed over at the threshold (say it once, then shut up).
    let mut guide_last: Option<Instant> = None;
    let mut guide_handed_over = false;
    // Fps-dip watchdog: rolling window of recent fps so a dip is judged
    // against THIS flight's own baseline (60 locked vs 35 in weather), and a
    // rate limit so a sustained slump logs one event, not one per tick.
    let mut fps_window: std::collections::VecDeque<f64> = std::collections::VecDeque::new();
    let mut fps_dip_last: Option<Instant> = None;
    // The world's ticks' cadences and latches (traffic, near miss, party
    // line, ATIS edition, incursion): `world_tick::WorldTick`.
    let mut world = crate::world_tick::WorldTick::default();
    // Gyro auto-sync: cooldown, the readback owed after a fire, and the latch
    // for an airframe whose sync does nothing (reset on every swap below).
    let mut gyro_sync = GyroSync::default();
    // The unasked cockpit duties (night lights, GPS welcome page) and their own
    // slower clock. The monitor ticks at 3 Hz and neither input justifies that:
    // the sun moves a quarter of a degree a minute, and a battery master thrown
    // two seconds ago is still "just now" to the pilot. At 3 Hz this pair would
    // have been 6 reads/s, the same order as the replay/pause flags the load
    // histogram singled out as the obvious thing to slow down.
    let mut cockpit_auto = crate::cockpit_auto::CockpitAuto::default();
    let mut cockpit_auto_last: Option<Instant> = None;
    /// How often the cockpit duties look at the sun and the bus.
    const COCKPIT_AUTO_EVERY: Duration = Duration::from_secs(2);
    // The taxi clearance whose runway the crew last judged for a pilot
    // taxiing (full auto judges its own): said once per clearance.
    let mut cleared_runway_judged: Option<String> = None;
    // When full auto last ran: a clearance answering its request is its own
    // even when it lands just after a cancel (LXGB 2026-10-06: the mission's
    // "request taxi" went out, the cancel handed back, and the reply was
    // judged as the pilot's with the waiver gone).
    let mut full_auto_seen: Option<std::time::Instant> = None;
    // ICAO fuel declarations, each once per flight (reset on the ground).
    let mut fuel_declared = crate::fuel_declaration::Declarations::default();
    let fuel_clock = Instant::now();
    // Landing-clearance coach: when the approach hands us to the tower we owe
    // a "final" report before "cleared to land". `seen` starts the grace so a
    // prompt reporter is never nagged; `spoken` fires the nudge once.
    let mut landing_coach_seen: Option<Instant> = None;
    let mut landing_coach_spoken = false;
    // Region-handoff coach: with the pilot flying, the enroute controller's
    // "squawk seven thousand, contact X Tower on Y" is theirs to obey; after
    // a grace the copilot says once whatever is still undone.
    let mut region_coach: Option<(crate::party_line::RegionHandoff, Instant)> = None;
    // The copilot's progressive taxi guide, rebuilt per published clearance
    // route: the pilot monitoring with the airport diagram out.
    let mut taxi_guide: Option<crate::taxi_guide::Guide> = None;
    // Whether the last tick saw the sim in replay / paused, for transition logging.
    let mut was_replay = false;
    let mut was_paused = false;
    let mut lost_since: Option<Instant> = None;
    let mut seen_sim = false;
    let mut disconnected = false;
    let mut had_flying_control = false;
    // The last tick the pilot's hand was on the yoke under the automation.
    let mut hands_at: Option<Instant> = None;
    let mut control_idle_since: Option<Instant> = None;
    let mut airspeed_recovery = crate::interaction::Recovery::default();
    let mut cabin_recovery = crate::interaction::Recovery::default();
    let mut landing_summary = crate::interaction::Landing::default();
    // The copilot as a person who was there: the flight's memory, the word
    // after the landing, the offers, the asides held for a quiet moment.
    let mut rapport = crate::rapport::Rapport::new();
    let mut rapport_airframe = crate::rapport::Airframe::from_profile(&live.get());
    // The checklist flow's phase-boundary detector: climb, cruise, descent,
    // before landing, after landing, each once per flight.
    let mut flow_tracker = crate::flow::Tracker::default();

    loop {
        // Settings that this tick reads (the taxi guide, the party line) are
        // taken fresh from the live store, so a Settings-screen change applies
        // on the next tick rather than the next boot. Boot-time values used
        // above (the sample rate, the base URL) deliberately stay as they were.
        let config = crate::settings::live();
        // Aircraft switched → rebuild the checks for the new airframe.
        if live.generation() != gen {
            gen = live.generation();
            checks = build_checks(&live.get());
            drefs = advisor_drefs(&live.get());
            pump_tanks = live.get().fuel_tank_pumps_off_when_empty.clone();
            tank_lateral = None;
            icing_read = None;
            icing_clear_since = None;
            icing_logged = None;
            avionics_readback = avionics_readback_dref(&live.get());
            (night_lights, gps_ack) = cockpit_duties(&live.get());
            // A new airframe is a new cockpit: whatever we set on the last one
            // says nothing about this one's switches, and its GPS has its own
            // welcome page. Same rule as the radio-power latch — a process-wide
            // fact that outlives its airframe aims the next aircraft's writes
            // at something it may not have.
            cockpit_auto = crate::cockpit_auto::CockpitAuto::default();
            // "The sync does nothing here" is a fact about THAT airframe.
            gyro_sync = GyroSync::default();
            flow_tracker = crate::flow::Tracker::default();
            landing_summary = crate::interaction::Landing::default();
            rapport = crate::rapport::Rapport::new();
            rapport_airframe = crate::rapport::Airframe::from_profile(&live.get());
            airspeed_recovery = crate::interaction::Recovery::default();
            cabin_recovery = crate::interaction::Recovery::default();
            had_flying_control = false;
            cabin_capable = live.get().cabin_crew;
            states = checks.iter().map(|c| (c.id(), FireState::new())).collect();
            tracing::info!(
                checks = checks.len(),
                "monitor: reconfigured for new aircraft"
            );
            crate::blackbox::record(
                "flight_meta",
                serde_json::json!({ "aircraft": live.get().id }),
            );
        }
        // Replay or pause: neither is the aircraft flying NOW — a replay is a
        // recording, a pause is real but frozen — so nothing here may react.
        // One gate covers the whole reactive tick: checks and callouts, the
        // ATC compliance / incursion / region watches, and crash recording.
        // A replayed crash landing earned "you landed on runway two seven
        // without clearance" and a second crash event (2026-07-14). Fail
        // open: a failed read means live. State (FireStates) simply freezes
        // and resumes on live.
        let in_replay = xp
            .get_float(crate::datarefs::IS_IN_REPLAY)
            .await
            .is_ok_and(|v| v > 0.5);
        let paused = xp
            .get_float(crate::datarefs::PAUSED)
            .await
            .is_ok_and(|v| v > 0.5);
        if in_replay != was_replay {
            was_replay = in_replay;
            tracing::info!(in_replay, "monitor: replay mode changed");
            crate::blackbox::record("replay", serde_json::json!({ "active": in_replay }));
        }
        if paused != was_paused {
            was_paused = paused;
            tracing::info!(paused, "monitor: pause changed");
            crate::blackbox::record("pause", serde_json::json!({ "active": paused }));
        }
        // Published to the arbiter and the command loop's transmit gates.
        // Replay drops station audio (cutting a live transmission — the film
        // has no radio); pause only blocks NEW radio traffic, so a controller
        // caught mid-word finishes the sentence.
        voice.set_replay(in_replay);
        voice.set_paused(paused);
        if in_replay || paused {
            // The world does not wait (our aircraft held where last seen).
            world.traffic_alone(&xp, &voice, &status, true).await;
            tokio::time::sleep(period).await;
            continue;
        }
        if let Some(mut snap) = Snapshot::read(&xp, &drefs).await {
            // The weight the stall floors ride (`aircraft::speed_floor_kts`),
            // every tick: a failed read publishes none.
            crate::aircraft::publish_weight_now(snap.all_up_kg);
            if !pump_tanks.is_empty() {
                snap.fuel_tanks = read_fuel_tanks(&xp, &pump_tanks, &mut tank_lateral).await;
            }
            let now = Instant::now();
            // The altimeter cross-check, published for a follower that wants
            // to know whether the altimeter it flies can be believed (#94).
            let truth = altimeter_truth_of(&snap, live.get().transition_altitude_ft);
            crate::tools::instruments::publish_altimeter_truth(truth);
            // The ice picture (#100): cold air only, every ICING_READ_S.
            if snap.on_ground || snap.oat_c > ICING_READ_OAT_C {
                icing_read = None;
            } else if icing_read
                .as_ref()
                .is_none_or(|(at, _)| now.duration_since(*at) >= Duration::from_secs(ICING_READ_S))
            {
                let systems = crate::tools::anti_ice::resolved_live();
                let r = crate::tools::anti_ice::read_icing(&xp, snap.oat_c, snap.dewpoint_c, &systems)
                    .await;
                icing_read = Some((now, r));
            }
            snap.icing = icing_read.as_ref().map(|(_, r)| r.clone());
            // Clear of icing long enough: the crew switches off only what it
            // switched on itself, and only while an automation flies.
            let icing_now = snap.icing.as_ref().is_some_and(|r| {
                crate::tools::anti_ice::icing_conditions(r) || crate::tools::anti_ice::accreting(r)
            });
            if snap.icing.is_some() && icing_logged != Some(icing_now) {
                icing_logged = Some(icing_now);
                crate::blackbox::record(
                    "icing",
                    serde_json::json!({"event": "conditions", "icing": icing_now,
                        "oat_c": snap.oat_c, "dewpoint_c": snap.dewpoint_c, "alt_ft": snap.alt_ft,
                        "claimed": crate::tools::anti_ice::claimed().len()}),
                );
            }
            if icing_now {
                icing_clear_since = None;
            } else {
                let since = *icing_clear_since.get_or_insert(now);
                let claimed = crate::tools::anti_ice::claimed();
                if !claimed.is_empty()
                    && !snap.on_ground
                    && snap.automation_flying
                    && crate::mode::crew_actuates()
                    && now.duration_since(since) >= Duration::from_secs(ICING_CLEAR_S)
                {
                    let failed = crate::tools::anti_ice::monitor_fix(&xp, &claimed, false).await;
                    crate::blackbox::record(
                        "icing",
                        serde_json::json!({"event": "clear_off", "systems": claimed.len(),
                            "took": failed.is_none(), "oat_c": snap.oat_c, "dewpoint_c": snap.dewpoint_c}),
                    );
                    voice.alert(
                        Priority::Advisory,
                        failed.unwrap_or_else(|| "Clear of icing, anti-ice off.".to_string()),
                    );
                    icing_clear_since = Some(now);
                }
            }
            // The flight model FIRST, every tick whatever is quiet: the
            // consumers below read its phase, memory and this tick's events
            // (`flight.rs`). Its events are blackboxed `flight`, its phase
            // rides the state record as `fphase`.
            let wall_t = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_secs_f64())
                .unwrap_or(0.0);
            // The world's reach: a station whose field we are not at (on the
            // ground) neither hears nor answers, and an uncontrolled
            // departure plans its own taxi.
            crate::party_line::note_aircraft(snap.lat, snap.lon, snap.on_ground);
            let crashed = voice.crashed();
            let flight_events = flight.tick(&flight_sample(&snap, wall_t, crashed));
            snap.phase = coarse_phase(flight.phase(), snap.agl_ft);
            let flight_status = flight.status();
            // The duty engine, in SHADOW: what the flight owes next, logged
            // when it changes beside whether the running automation agrees.
            // It acts on nothing (`duty.rs`).
            if let Some(st) = &flight_status {
                let landed = flight.memory().landings > 0;
                let (d, why) = crate::duty::next(st, landed, wall_t);
                // The State tab, change-gated with a resend every 20 ticks
                // (a plugin reload zeroes its buffer).
                if let Some(sink) = status.as_ref() {
                    let text = crate::flight::state_lines(
                        st,
                        flight.memory(),
                        flight.in_phase_secs(wall_t),
                        (d.name(), why, crate::duty::agrees(d, st.control)),
                    );
                    state_resend = state_resend.saturating_sub(1);
                    if text != last_state_text || state_resend == 0 {
                        sink.state(&text);
                        last_state_text = text;
                        state_resend = 20;
                    }
                }
                // Written once the verdict has settled (`duty::SETTLE_S`): a
                // hand-over between loops is not a disagreement.
                let phase = format!("{:?}", st.phase);
                if let Some(r) = duty_trail.step(d, why, &phase, st.control, wall_t) {
                    crate::blackbox::record(
                        "duty",
                        serde_json::json!({
                            "duty": r.duty.name(), "why": r.why,
                            "phase": r.phase, "began_t": r.began_t,
                            "agrees": r.agrees,
                        }),
                    );
                }
            }
            crate::flight::publish(flight_status);
            // The EFB's view of the aircraft: this tick's reads, handed over
            // as they are (it never asks the sim itself).
            crate::efb::feed::note_tick(crate::efb::feed::Tick {
                lat: snap.lat,
                lon: snap.lon,
                alt_ft: snap.alt_ft,
                agl_ft: snap.agl_ft,
                ias_kts: snap.ias_kts,
                gs_kts: snap.groundspeed_kts,
                on_ground: snap.on_ground,
                track_deg: snap.track_deg,
                heading_deg: snap.heading_deg,
                fuel_kg: snap.fuel_kg,
                endurance_min: snap.endurance_min,
            });
            for ev in &flight_events {
                crate::blackbox::record("flight", ev.to_json());
            }
            seen_sim = true;
            lost_since = None;
            if disconnected {
                disconnected = false;
                landing_summary = crate::interaction::Landing::default();
                landing_summary.note_interruption();
                airspeed_recovery = crate::interaction::Recovery::default();
                cabin_recovery = crate::interaction::Recovery::default();
                if crate::mode::copilot_enabled() {
                    voice.alert(Priority::Caution, "Connected again. I'm monitoring only; you have control. Check the autopilot. Say resume the flight plan when ready.".to_string());
                }
            }

            // Fps-dip watchdog: pure diagnosis, no callout. A dip well below
            // the rolling median lands in the flight log + blackbox with a
            // timestamp, so it correlates against what else was happening
            // (llm answer, tts synth, station transmission — all logged).
            if let Some(fps) = snap.fps {
                let median = {
                    let mut v: Vec<f64> = fps_window.iter().copied().collect();
                    v.sort_by(|a, b| a.total_cmp(b));
                    v.get(v.len() / 2).copied()
                };
                if let Some(median) = median.filter(|m| fps_window.len() >= 30 && *m >= 20.0) {
                    let rate_ok =
                        fps_dip_last.is_none_or(|t| now.duration_since(t).as_secs() >= 10);
                    if fps < median * 0.7 && rate_ok {
                        fps_dip_last = Some(now);
                        tracing::warn!(fps = fps.round(), median = median.round(), "fps dip");
                        crate::blackbox::record(
                            "fps_dip",
                            serde_json::json!({ "fps": r1(fps), "median": r1(median) }),
                        );
                    }
                }
                fps_window.push_back(fps);
                if fps_window.len() > 90 {
                    fps_window.pop_front();
                }
            }

            // Crew-side watches stand down entirely in ATC-only mode: no
            // copilot and no callouts — the pilot kept the world and sent
            // the crew home. World ticks are inert without the station's
            // published state, so most need no gate of their own;
            // region_tick is the exception (its index cache survives).
            let crew_on = crate::mode::copilot_enabled();

            // The sim moved the aircraft (a new flight loaded, a reposition):
            // a list of the flight that was ends with it. Through the door,
            // which knows a jump our own setup or the runner's start made
            // (their doors already ran) and keeps a replay's jump light.
            if flight_events
                .iter()
                .any(|e| matches!(e, crate::flight::Event::Reposition { .. }))
            {
                crate::flight_state::flight_begins(crate::flight_state::NewFlight::Reposition);
            }
            // The checklist at its phase: crossing a boundary runs the
            // matching list as the flying copilot's cross-check when an
            // automation has the aircraft, or proposes it to the pilot
            // flying ("Climb checklist when you're ready?"). The tracker is
            // sustained-condition and one-shot per flight; the flow module
            // decides who is flying and never acts on a proposal by itself.
            if crew_on {
                let sample = crate::flow::Sample {
                    agl_ft: snap.agl_ft,
                    vsi_fpm: snap.vsi_fpm,
                    automation: crate::flow::automation_flying(),
                    landing_config: crate::flow::landing_config_for(
                        // The helicopter's loops land without autoland, so
                        // its landing items are not theirs to wait for.
                        crate::flight_state::crew_flying().is_some_and(|o| {
                            !matches!(
                                o,
                                crate::flight_state::Owner::HeliRoute
                                    | crate::flight_state::Owner::HeliCircuit
                                    | crate::flight_state::Owner::Hover
                            )
                        }),
                        snap.gear_down,
                        snap.flap_ratio,
                        !crate::aircraft::flap_limits().is_empty(),
                        crate::autoland::landing_configured(),
                    ),
                    climb_config: crate::flow::climb_config(
                        snap.flap_ratio,
                        !crate::aircraft::flap_limits().is_empty(),
                    ),
                    on_runway: if snap.on_ground {
                        crate::party_line::on_runway(snap.lat, snap.lon)
                    } else {
                        None
                    },
                };
                if let Some(section) = flow_tracker.tick(&sample, &flight, &flight_events, now) {
                    crate::flow::phase_boundary(section, &config, &voice);
                }
            }

            // The instructor's lesson: every tick feeds the scorer (and the
            // brief's picture of the aircraft); narration comes back only
            // while a lesson flies. Advisory, so the radio is never talked
            // over for a heading nudge.
            // The aircraft against the cleared taxi route (the guide's turn
            // card), for the lesson's centreline and the state record (#88).
            let taxi_line = (snap.on_ground
                && crate::party_line::clearance_state() == Some(crate::tower::State::TaxiCleared))
            .then(|| taxi_guide.as_ref().and_then(|g| g.turn_card(snap.lat, snap.lon)))
            .flatten()
            .map(|c| crate::instructor::TaxiLine::from_card(&c));
            if crew_on {
                let sample = crate::instructor::Sample {
                    t_s: crate::instructor::clock_s(),
                    alt_ft: snap.alt_ft,
                    agl_ft: snap.agl_ft,
                    hdg_deg: snap.heading_deg,
                    ias_kts: snap.ias_kts,
                    bank_deg: snap.bank_deg,
                    vsi_fpm: snap.vsi_fpm,
                    on_ground: snap.on_ground,
                    track_deg: snap.track_deg,
                    gs_kts: snap.groundspeed_kts,
                    engine_running: snap.engine_running,
                    taxi: taxi_line,
                };
                let mut io = crate::instructor::FlowIo {
                    config: &config,
                    voice: &voice,
                };
                for line in crate::instructor::tick_with(sample, &mut io) {
                    voice.alert(Priority::Advisory, line);
                }
            }

            // Pushback callouts (on the ground): tug hooks up → pushing → clear.
            if crew_on && snap.on_ground {
                // Mirror OR action, never both: `voice.alert` already mirrors the
                // spoken line to the panel transcript (2026-07-24), so an extra
                // `s.alert` here wrote every callout twice — a green mirror line
                // AND a blue Advisory line, same timestamp (2026-07-26).
                let say = |line: &str| {
                    voice.alert(Priority::Advisory, line.to_string());
                };
                // Casual ground chatter, not phraseology — so it varies flight to
                // flight (the value is nothing to standardise here, unlike a callout).
                if snap.tug_attached && !prev_attached && !snap.pushing_back {
                    say(crate::readback::one_of(&[
                        "Tug connected, ready for pushback.",
                        "Tug's hooked up, ready to push.",
                        "Ground's connected, ready when you are.",
                    ]));
                }
                // Once per hook-up: `pushing_back` is motion over 0.3 kt with
                // the bar on, and a tug nudging at the edge of it flickered
                // "Pushing back" three times in five seconds (LIPK 2026-09-26).
                if snap.pushing_back && !prev_pushing && !push_called {
                    push_called = true;
                    say(crate::readback::one_of(&[
                        "Pushing back.",
                        "Here we go, pushing back.",
                        "And we're pushing.",
                    ]));
                }
                // Clear is the BAR coming off, not the rolling stopping: now
                // that `pushing_back` is motion-based, a pause mid-push would
                // otherwise announce a tug that is still hooked to the nose.
                // And only after a push: a bar that comes off unmoved is no
                // pushback done (LXGB 2026-10-06: the 737's tug from a
                // cancelled push went with the 737, and the 172 loaded in
                // its place was told "Pushback done, tug's clear").
                let pushed = push_called;
                if !snap.tug_attached {
                    push_called = false;
                }
                if !snap.tug_attached && prev_attached && pushed {
                    say(crate::readback::one_of(&[
                        "Pushback complete, tug clear.",
                        "Pushback done, tug's clear.",
                        "That's us off the gate, tug clear.",
                    ]));
                }
            }
            prev_attached = snap.tug_attached;
            prev_pushing = snap.pushing_back;

            // The wreck flag: X-Plane raises has_crashed the instant the
            // airframe is destroyed; a flight reset drops it, and the world
            // resumes. Published to the arbiter (receive + transmit gates) —
            // the avionics bus stays hot on the crash screen, so the power
            // gate alone let a mayday from the wreck raise the tower
            // (2026-07-13). Fail open: a failed read changes nothing.
            if let Ok(v) = xp.get_float(crate::datarefs::HAS_CRASHED).await {
                let is_crashed = v > 0.5;
                if is_crashed && !voice.crashed() {
                    crate::blackbox::record(
                        "crash",
                        serde_json::json!({
                            "lat": snap.lat, "lon": snap.lon, "alt": r1(snap.alt_ft),
                        }),
                    );
                }
                voice.set_crashed(is_crashed);
            }
            // The cockpit duties a crew member performs unasked: the
            // interior lights once the aircraft is alive and the sun is
            // down, and the GNS430's welcome page. Crew-gated for the same
            // reason as the gyro sync above — this is the copilot's HAND on
            // the panel, and in ATC-only nobody is aboard to reach it.
            // Skipped entirely on an airframe that declares neither duty,
            // so nothing pays for a capability it doesn't have.
            if crew_on
                && crate::mode::crew_actuates()
                && (!night_lights.is_empty() || gps_ack.is_some())
                && cockpit_auto_last.is_none_or(|t| now.duration_since(t) >= COCKPIT_AUTO_EVERY)
            {
                cockpit_auto_last = Some(now);
                // The pilot switched one of these themselves since our last
                // pass: their switch now, not ours.
                if crate::cockpit_auto::take_pilot_moved_lights() {
                    cockpit_auto.pilot_took_the_lights();
                }
                // Both best-effort: a failed read must not act. Defaulting
                // either way would be a guess with a switch on the end of
                // it — a missing sun angle reading as night would light the
                // cockpit at noon.
                let sun = xp.get_float(crate::datarefs::SUN_PITCH_DEG).await;
                let volts = xp
                    .get_float_array(crate::datarefs::BUS_VOLTS)
                    .await
                    .ok()
                    .and_then(|v| v.first().copied());
                if let (Ok(sun), Some(volts)) = (sun, volts) {
                    for action in cockpit_auto.tick(sun, volts) {
                        use crate::cockpit_auto::Action;
                        match action {
                            Action::LightsOn | Action::LightsOff => {
                                let on = action == Action::LightsOn;
                                // Through the SAME writer the `set_lights`
                                // tool uses, so every profile mechanism and
                                // the readback behave identically whether a
                                // pilot asked or the copilot decided.
                                let mut done = Vec::new();
                                for l in &night_lights {
                                    if crate::tools::lights::apply(&xp, l, on)
                                        .await
                                        .is_ok_and(|actual| actual == on)
                                    {
                                        done.push(l.spoken.clone());
                                    }
                                }
                                crate::blackbox::record(
                                    "cockpit_auto",
                                    serde_json::json!({
                                        "event": if on { "lights_on" } else { "lights_off" },
                                        "sun_pitch_deg": r1(sun), "bus_volts": r1(volts),
                                        "lights": done,
                                    }),
                                );
                                // Speak only what actually moved: a switch
                                // that refused is not an announcement, it
                                // is the ground-power lie in a new coat.
                                if let Some(what) = done.first() {
                                    voice.alert(
                                        Priority::Advisory,
                                        if on {
                                            format!("Sun's down, {what} on.")
                                        } else {
                                            format!("Daylight, {what} off.")
                                        },
                                    );
                                }
                            }
                            Action::AckGps => {
                                // Twice: the welcome page and the database
                                // page behind it. Silent — clearing a
                                // startup screen is not worth a word, and
                                // the pilot sees the unit come up.
                                if let Some(cmd) = &gps_ack {
                                    let a = xp.run_command(cmd).await;
                                    let b = xp.run_command(cmd).await;
                                    crate::blackbox::record(
                                        "cockpit_auto",
                                        serde_json::json!({
                                            "event": "gps_ack", "command": cmd,
                                            "ok": a.is_ok() && b.is_ok(),
                                        }),
                                    );
                                }
                            }
                        }
                    }
                }
            }
            // Approach vectors: armed by the ATC station when the pilot requests
            // vectors on the approach frequency. Advisory narration only; muted
            // under the same gates as the visual-approach talk-down. A crashed
            // airframe silences the whole world block: no controller works a
            // wreck, and the frozen flight model reads as garbage anyway.
            let gates = world_gates(
                crate::flight_state::Owner::Recover.active(),
                crate::flight_state::Owner::Autoland.active(),
                PANEL_MUTE.load(std::sync::atomic::Ordering::Relaxed),
                voice.crashed(),
            );
            let quiet = gates.quiet;
            // The CABIN is not part of that. `quiet` mutes the world's
            // narration while the flight deck is busy, and an autoland is
            // exactly when it is - but the cabin is a state machine that has
            // to SEE the flight, not a commentator. Gated with the rest it was
            // never called from the autoland arm to the rollout handoff, so it
            // missed its own landing every time an automation flew the
            // approach (Liege 2026-09-22, "cabin is secure for departure"
            // during the taxi in). A recovery, the panel's mute and a wreck
            // still silence it; a normal landing does not.
            let cabin_quiet = gates.cabin_quiet;
            feed_surface_wind();
            // The crew's oxygen masks: on in the cabin-altitude warning band,
            // off again below it, never over the pilot (crew hands only;
            // #159: they used to follow the master and breathed all flight).
            if crate::mode::crew_actuates() {
                if let Some((urgent, line)) =
                    crate::tools::cabin_air::masks_tick(&xp, snap.on_ground).await
                {
                    let p = if urgent { Priority::Warning } else { Priority::Advisory };
                    voice.alert(p, line);
                }
            }
            // Cabin crew watches (crew-side, airframes with a cabin crew
            // only): the owed "cabin secure" report after an acknowledged
            // prepare call, and the seatbelt-sign interphone call. The
            // sterile-cockpit rule lives in the cabin module — safety
            // reports come through, flavor stays above 10,000 ft.
            if crew_on && !cabin_quiet && crate::cabin::aboard(cabin_capable) {
                let (own, belt_line, tick_line) = {
                    let mut cab = cabin.lock().unwrap_or_else(|e| e.into_inner());
                    (
                        // The cabin's own initiative first: what it
                        // starts here, `tick` reports when due.
                        cab.watch(
                            now,
                            crate::cabin::View {
                                agl_ft: snap.agl_ft,
                                on_ground: snap.on_ground,
                                groundspeed_kts: snap.groundspeed_kts,
                                vsi_fpm: snap.vsi_fpm,
                                engine_running: snap.engine_running,
                                pushing_back: snap.pushing_back || snap.tug_attached,
                                flight: Some(crate::cabin::FlightFacts {
                                    landed: flight.memory().landings > 0
                                        && !flight.phase().airborne(),
                                    repositioned: flight_events.iter().any(|e| {
                                        matches!(e, crate::flight::Event::Reposition { .. })
                                    }),
                                }),
                            },
                        ),
                        snap.seatbelt_sign
                            .and_then(|on| cab.seatbelt(on, snap.agl_ft)),
                        cab.tick(now, snap.agl_ft, snap.on_ground),
                    )
                };
                let own_line = own.and_then(|i| {
                    if i.line.is_none() {
                        crate::blackbox::record(
                            "cabin",
                            serde_json::json!({ "route": "watch", "event": i.event }),
                        );
                    }
                    i.line
                });
                for line in [own_line, belt_line, tick_line].into_iter().flatten() {
                    if let Some(s) = status.as_ref() {
                        s.action(&format!("CAB: {line}"));
                    }
                    crate::blackbox::record(
                        "cabin",
                        serde_json::json!({ "route": "watch", "line": &line }),
                    );
                    voice.alert_cabin(Priority::Advisory, line);
                }
            }
            // The world's compliance watch and the landing's runway watch
            // (`world_tick`), AHEAD of `quiet`: that gate includes
            // `autoland::ACTIVE`, and the runway watch ticks whoever flies
            // the final. One COM1 read serves every world tick below.
            let tick = crate::world_tick::Tick {
                xp: &xp,
                voice: &voice,
                status: &status,
                snap: &snap,
                now,
            };
            let mut heard = world.runway_watch(&tick, gates).await;
            // Our own traffic (stage D): nothing unless XCOPILOT_TRAFFIC=ours.
            world.traffic_link(&tick, &heard).await;
            let com1 = heard.com1;
            crate::efb::feed::note_com1(com1);
            if !quiet {
                // Tell the speech arbiter what COM1 hears (controllers AND
                // the ATIS loop): it cuts a live station transmission the
                // instant we tune off its frequency, and lets a stepped-on
                // tail die with a retune instead of following us to the new
                // frequency.
                voice.set_com1(com1, com1.and_then(crate::party_line::tuned_station));
                // ...and how FAR that station is: the radio's signal-quality
                // model fades a distant tower into hiss instead of playing
                // every transmission studio-clean at 60 nm.
                voice.set_station_range(crate::party_line::active_station_range_nm(
                    snap.lat, snap.lon,
                ));
                // Radio audio needs a powered receiver: with the avionics
                // master OFF the pilot can't hear ATC/ATIS/traffic. Confident
                // reads only — a read failure or an airframe with no avionics
                // readback keeps the radio powered (fail open, never a
                // mysteriously silent controller).
                // Fail open must be RESTORED, not merely defaulted: the flag is
                // process-wide and survives an airframe switch, so "stop
                // publishing" latches whatever the LAST aircraft said. A
                // cold-and-dark 172 published false, the pilot swapped to a Twin
                // Otter (no avionics master — nothing left to publish), and the
                // stale false silently binned every crew radio call for the rest
                // of the session: full auto's clearance and taxi requests never
                // left the aircraft and it blamed the tower ("no taxi clearance
                // came back", Heathrow 2026-07-27). A failed read is the same
                // trap one tick wide.
                let read = match &avionics_readback {
                    Some(dref) => xp.get_float(dref).await.ok(),
                    None => None,
                };
                voice.set_radio_powered(radio_powered_from(avionics_readback.as_deref(), read));
                // Runway incursion: the world's shout (`world_tick`), the crew's
                // when no frequency is tuned.
                world.incursion(&tick, &heard).await;
                // Gyro auto-sync: a drifted DG gets re-aligned by the copilot's
                // own hand, then announced — not a "recommend syncing" nag
                // (pilot feedback, 2026-07-09). Airborne only (on the ground
                // you align at the run-up). Gated on MEASURED drift, not the
                // profile's directional_gyro flag: the sim drifts a DG under
                // "glass" airframes too (the SR22 flew 166° off while the
                // profile swore there was nothing to sync, 2026-07-12) — and
                // a true glass panel reads ~0 here, so it never fires there.
                // Crew-gated: this is the copilot's HAND on the panel, not
                // just its voice — in ATC-only nobody re-aligns your DG.
                //
                // An ACCEPTED command is not a sync: the drift is read back a
                // beat later and the line is said only if it fell. On the stock
                // 737 the command did nothing and the copilot claimed a resync
                // every 120 s with the drift climbing 11.8 to 15.6 (LEBL-LFBD
                // 2026-09-27); a sync that does not take is silent and latches
                // off for the airframe. Not gated on the systems survey's
                // glass_hsi: the readback is the guard.
                if gyro_sync.verify_due(now) {
                    // A failed read is NO measurement (the snapshot's 0.0
                    // default would read as a perfect sync).
                    let after = xp
                        .get_float("sim/cockpit/gyros/dg_drift_vac_deg")
                        .await
                        .ok()
                        .map(fold_drift_deg);
                    match gyro_sync.settle(after, now) {
                        Some(GyroSyncOutcome::Took { before, after }) => {
                            crate::blackbox::record(
                                "gyro_sync",
                                serde_json::json!({
                                    "took": true,
                                    "drift_deg": r1(before),
                                    "after_deg": r1(after),
                                }),
                            );
                            // `voice.alert` mirrors to the panel; an explicit
                            // `s.action(&line)` here too double-wrote the line
                            // to the chat (2026-07-24).
                            voice.alert(Priority::Advisory, gyro_sync_line(before));
                        }
                        Some(GyroSyncOutcome::DidNotTake { before, after }) => {
                            crate::blackbox::record(
                                "gyro_sync",
                                serde_json::json!({
                                    "took": false,
                                    "drift_deg": r1(before),
                                    "after_deg": r1(after),
                                    "latched": true,
                                }),
                            );
                        }
                        Some(GyroSyncOutcome::Unmeasured { before }) => {
                            crate::blackbox::record(
                                "gyro_sync",
                                serde_json::json!({
                                    "took": serde_json::Value::Null,
                                    "drift_deg": r1(before),
                                    "after_deg": serde_json::Value::Null,
                                }),
                            );
                        }
                        None => {}
                    }
                }
                if crew_on && !snap.on_ground && gyro_sync.due(snap.gyro_drift_deg, now) {
                    gyro_sync.last = Some(now);
                    if xp.run_command("sim/instruments/DG_sync_mag").await.is_ok() {
                        gyro_sync.fired(snap.gyro_drift_deg, now);
                    }
                }
                // The ATIS loop, the after-takeoff handoff and the radar watch:
                // the world's (`world_tick`).
                world.radio_services(&tick, &heard).await;
                // The controller's squawk nudge is an INSTRUCTION to the
                // crew: recycle, or dial the code, and say so on both
                // channels (EGPH 2026-09-20: two "recycle transponder" calls,
                // nobody answered). Crew hands only; on its own task, since a
                // stepped knob takes a few hundred milliseconds.
                // An instrument arrival's approach clearance and its handoff
                // between approach and the tower: drained every pass, worked
                // by the crew while full auto flies (`full_auto::arrival_radio`).
                crate::full_auto::arrival_radio_tick(
                    &config.xplane_base_url,
                    crate::full_auto::running() && crate::mode::crew_actuates(),
                );
                // Minimum fuel and mayday fuel: declared by the crew that
                // works the radio, advised to a pilot who works it himself
                // (`fuel_declaration`).
                if snap.on_ground {
                    fuel_declared = Default::default();
                } else if let Some((at_landing, reserve)) = live
                    .get()
                    .fuel
                    .as_ref()
                    .and_then(|f| crate::fuel_declaration::predict(&snap, f))
                    .filter(|_| crew_on)
                {
                    let due = crate::fuel_declaration::due(at_landing, reserve);
                    let t = fuel_clock.elapsed().as_secs_f64();
                    if let Some(d) = fuel_declared.tick(due, t) {
                        let crew_declares =
                            crate::full_auto::running() && crate::mode::crew_actuates();
                        crate::blackbox::record(
                            "fuel_declared",
                            serde_json::json!({
                                "declaration": format!("{d:?}"),
                                "crew_declares": crew_declares,
                                "at_landing_min": r1(at_landing),
                                "final_reserve_min": r1(reserve),
                                "fuel_kg": snap.fuel_kg.map(r1),
                                "dest_nm": snap.dest_route_nm.or(snap.dest_dist_nm).map(r1),
                                "gs_kts": r1(snap.groundspeed_kts),
                            }),
                        );
                        voice.alert(
                            Priority::Caution,
                            crate::fuel_declaration::intercom_line(d, crew_declares).to_string(),
                        );
                        if crew_declares {
                            let base = config.xplane_base_url.clone();
                            tokio::spawn(async move {
                                let xp = XPlaneClient::new(base);
                                let cs = crate::crew_radio::crew_callsign(&xp).await;
                                let words = crate::fuel_declaration::radio_words(d, &cs);
                                crate::crew_radio::transmit(words);
                            });
                        }
                    }
                }
                if let Some(nudge) = crate::party_line::take_squawk_nudge() {
                    if crate::mode::crew_actuates() {
                        let base = config.xplane_base_url.clone();
                        let v = voice.clone();
                        tokio::spawn(async move {
                            let xp = XPlaneClient::new(base);
                            let (check, said, readback) =
                                crate::tools::radio::answer_nudge(&xp, nudge.code, nudge.recycle)
                                    .await;
                            let mut rec = check.json();
                            rec["route"] = serde_json::json!("squawk_answer");
                            rec["recycle"] = serde_json::json!(nudge.recycle);
                            rec["ok"] = serde_json::json!(check.ok());
                            crate::blackbox::record("atc", rec);
                            let prio = if check.ok() {
                                Priority::Advisory
                            } else {
                                Priority::Caution
                            };
                            v.alert(prio, &said);
                            if let Some(words) = readback {
                                let cs = crate::crew_radio::crew_callsign(&xp).await;
                                crate::crew_radio::transmit(crate::crew_radio::radio_call(
                                    &cs, &words,
                                ));
                            }
                        });
                    }
                }
                // A taxi clearance to a runway this airframe cannot use
                // (ETND 2026-09-24: 446 m of grass for an SF50 needing 780):
                // with a pilot taxiing, the copilot says both numbers and the
                // request to make, once per clearance, by the same judgement full
                // auto asks with for itself (`runway::cleared_runway_fit`).
                {
                    let taxi_to = (crate::party_line::clearance_state()
                        == Some(crate::tower::State::TaxiCleared))
                    .then(crate::party_line::clearance_runway)
                    .flatten();
                    if crate::full_auto::running() {
                        full_auto_seen = Some(std::time::Instant::now());
                    }
                    let missions = full_auto_seen
                        .is_some_and(|t| t.elapsed().as_secs_f64() < MISSION_CLEARANCE_SECS);
                    if taxi_to != cleared_runway_judged {
                        cleared_runway_judged = taxi_to.clone();
                        if let Some(rwy) = taxi_to.filter(|_| crew_on && snap.on_ground && !missions)
                        {
                            let (roll, mtow) = crate::aircraft::profile_takeoff_roll(&live.get());
                            let needed =
                                crate::aircraft::takeoff_needed_now_m(&xp, roll, mtow, 500.0).await;
                            if let crate::runway::ClearedFit::Short {
                                cleared,
                                length_m,
                                needed_m,
                                ask,
                            } = crate::runway::cleared_runway_fit(
                                &crate::party_line::field_runways(),
                                &rwy,
                                needed,
                            ) {
                                let line = match &ask {
                                    Some(a) => format!(
                                        "Runway {} is {:.0} metres, we need {:.0}. Ask the tower: {}.",
                                        crate::crew_radio::spell_runway(&cleared),
                                        length_m,
                                        needed_m,
                                        crate::runway::runway_request_call(&cleared, a)
                                    ),
                                    None => crate::runway::short_runway_line(
                                        length_m, needed_m, &cleared, None,
                                    ),
                                };
                                crate::blackbox::record(
                                    "atc",
                                    serde_json::json!({
                                        "route": "cleared_runway_short", "runway": cleared,
                                        "length_m": length_m.round(), "needed_m": needed_m.round(),
                                        "ask": ask, "reply": &line,
                                    }),
                                );
                                voice.alert(Priority::Caution, line);
                            }
                        }
                    }
                }
                // Landing-clearance coach. The approach clears you for the
                // APPROACH and hands you to the tower; the tower must still
                // say "cleared to land". Landing uncleared earns a phone
                // number (the compliance watch above) — so a beat after the
                // handoff, if no landing clearance has come and we're still
                // flying, nudge the pilot to report final. The world clears
                // the handoff on the landing clearance or touchdown, which
                // resets the latch here; a prompt reporter is never nagged.
                match crate::party_line::landing_handoff_runway().filter(|_| crew_on) {
                    Some(rwy) if !snap.on_ground => {
                        let seen = *landing_coach_seen.get_or_insert(now);
                        if !landing_coach_spoken
                            && now.duration_since(seen).as_secs() >= LANDING_COACH_GRACE_SECS
                        {
                            landing_coach_spoken = true;
                            let line = format!(
                                "That's the approach clearance, not a landing clearance. \
                                 Call the tower and report final, runway {rwy} — \
                                 or we touch down uncleared."
                            );
                            voice.alert(Priority::Advisory, line);
                        }
                    }
                    _ => {
                        landing_coach_seen = None;
                        landing_coach_spoken = false;
                    }
                }
                // Region-handoff coach. Full auto consumes the handoff itself
                // (readback, seven thousand, retune); with anyone else flying
                // the copilot takes it and, after a grace, names what the
                // pilot has not done yet — once, never re-asserted.
                if !crate::full_auto::ACTIVE.load(std::sync::atomic::Ordering::Relaxed) {
                    if let Some(h) = crate::party_line::take_region_handoff() {
                        region_coach = Some((h, now));
                    }
                }
                if let Some((h, seen)) = region_coach.as_ref() {
                    if !crew_on || snap.on_ground {
                        region_coach = None;
                    } else if now.duration_since(*seen).as_secs() >= REGION_COACH_GRACE_SECS {
                        let code = if h.squawk_vfr {
                            xp.get_float(crate::datarefs::TRANSPONDER_CODE)
                                .await
                                .ok()
                                .map(|v| v.round() as i64)
                        } else {
                            None
                        };
                        if let Some(line) = region_coach_line(h, com1, code) {
                            crate::blackbox::record(
                                "atc",
                                serde_json::json!({
                                    "route": "region_coach", "khz": h.khz, "com1": com1,
                                    "squawk_vfr": h.squawk_vfr, "code": code, "reply": &line,
                                }),
                            );
                            voice.alert(Priority::Advisory, line);
                        }
                        region_coach = None;
                    }
                }
                // The copilot's progressive taxi guide — the crew half of
                // the taxi clearance. The tower said the route once; the
                // pilot monitoring calls the turns just-in-time ("next
                // left, onto Whiskey"), the runway crossings, and the
                // holding point, in the copilot's own voice at Advisory.
                // Off-route it goes quiet: the compliance watch above owns
                // that conversation, and two voices saying it would nag.
                if crew_on && config.taxi_guide && snap.on_ground {
                    let taxiing = crate::party_line::clearance_state()
                        == Some(crate::tower::State::TaxiCleared);
                    if taxiing {
                        crate::party_line::with_taxi_route(|route| match route {
                            Some(watch)
                                if !taxi_guide.as_ref().is_some_and(|g| g.is_for(&watch.path)) =>
                            {
                                taxi_guide = crate::taxi_guide::Guide::build(watch);
                            }
                            Some(_) => {}
                            None => taxi_guide = None,
                        });
                        if let Some(cue) = taxi_guide
                            .as_mut()
                            .and_then(|g| g.tick(snap.lat, snap.lon, snap.groundspeed_kts, now))
                        {
                            if let Some(s) = status.as_ref() {
                                s.action(&format!("Taxi guide: {cue}"));
                            }
                            crate::blackbox::record(
                                "taxi_guide",
                                serde_json::json!({ "cue": &cue }),
                            );
                            // PERISHABLE: a cue that can't speak within ~8 s
                            // (60 m of taxi) is disinformation — the G11
                            // "next left" queued behind a party-line exchange
                            // and played 17 s late, 130 m past the junction
                            // (2026-07-13). Dropped cues are covered by the
                            // guide's missed-turn nudge and the panel strip.
                            // The copilot driving says only the safety
                            // calls; its own steering is on the panel.
                            let copilot_driving = crate::autotaxi::driving();
                            if !copilot_driving
                                || taxi_guide.as_ref().is_some_and(|g| g.last_cue_is_safety())
                            {
                                voice.alert_stale_by(
                                    Priority::Advisory,
                                    cue,
                                    now + std::time::Duration::from_secs(8),
                                );
                            }
                        }
                    } else {
                        taxi_guide = None;
                    }
                }
                // The live-traffic watches through the traffic advisories: the
                // world's (`world_tick`), after the crew's taxi guide.
                world
                    .traffic(&tick, &mut heard, config.atc_chatter, taxi_guide.is_none())
                    .await;
            } else if snap.on_ground {
                crate::world_tick::WorldTick::vectors_unheard(&snap);
            }
            // The EFB's Airport page draws the same turn card as the panel:
            // geometry over the guide already held, no read.
            crate::efb::feed::note_turn_card(
                taxi_guide
                    .as_ref()
                    .and_then(|g| g.turn_card(snap.lat, snap.lon))
                    .as_ref(),
            );

            // Talk-down: PAR-style callouts down the synthesized approach path,
            // armed by the visual_approach tool. Advisory narration only — the
            // pilot flies. Muted while the copilot itself is flying (recover /
            // autoland) or the panel mute is on, same as every other callout.
            // Crew-gated: the guide IS the copilot talking.
            if let Some(path) = crate::approach_guide::armed().filter(|_| crew_on) {
                if snap.on_ground {
                    // Touchdown completes the guidance — but only if we actually
                    // guided in the air. Armed on the ground (pre-takeoff, or
                    // testing on the ramp) it stays armed for the flight instead
                    // of instantly "completing".
                    if guide_last.is_some() || guide_handed_over {
                        crate::approach_guide::cancel();
                        if !quiet {
                            voice.alert(Priority::Advisory, "Down. Guidance complete.".to_string());
                        }
                        (guide_last, guide_handed_over) = (None, false);
                    }
                } else {
                    let d =
                        crate::approach_guide::deviations(&path, snap.lat, snap.lon, snap.alt_ft);
                    if d.dist_nm > 35.0 {
                        crate::approach_guide::cancel();
                        if !quiet {
                            voice.alert(
                                Priority::Advisory,
                                "We've left the approach area, guidance dropped.".to_string(),
                            );
                        }
                        (guide_last, guide_handed_over) = (None, false);
                    } else if !quiet && !guide_handed_over {
                        if let Some(line) = crate::approach_guide::callout(&d) {
                            let due = guide_last.is_none_or(|t| {
                                t.elapsed().as_secs_f64()
                                    >= crate::approach_guide::cadence_secs(d.dist_nm)
                            });
                            if due {
                                if line.starts_with("Over the threshold") {
                                    guide_handed_over = true; // once — the flare is theirs
                                }
                                crate::blackbox::record(
                                    "talkdown",
                                    serde_json::json!({
                                        "apt": path.icao, "rwy": path.runway_id, "dist": r1(d.dist_nm),
                                        "xtk": r1(d.xtk_nm), "vdev": r1(d.vdev_ft), "msg": &line
                                    }),
                                );
                                voice.alert(Priority::Advisory, line);
                                guide_last = Some(now);
                            }
                        }
                    }
                }
            } else if guide_last.is_some() || guide_handed_over {
                (guide_last, guide_handed_over) = (None, false); // cancelled by voice
            }
            if let Some((last, last_t)) = prev_ias {
                let dt = now.duration_since(last_t).as_secs_f64();
                // Ignore a stretched dt (a skipped/stalled tick) — the rate would
                // be meaningless. Only update inside a sane sampling window.
                if (0.1..=1.0).contains(&dt) {
                    let inst = (snap.ias_kts - last) / dt;
                    trend = 0.6 * trend + 0.4 * inst;
                }
            }
            prev_ias = Some((snap.ias_kts, now));
            snap.ias_trend_kts_per_s = trend;

            // Black box: one state record per tick, all from the snapshot.
            // Bank and track ride along for the offline ACS grade
            // (`instructor::grade_flight`: steep turns, touchdown drift).
            let (lat, lon, alt) = (snap.lat, snap.lon, snap.alt_ft);
            // The hands and the levers, batched in one join: what the pilot
            // and the loops were doing, so a sortie can prove nobody touched
            // it and a complaint ("jerky braking") has a number.
            let pitch_due = pitch_reads_due(snap.on_ground, snap.ap_servos_on, crew_yoke_fresh());
            let inputs = read_inputs(&xp, pitch_due).await;
            if let Some(p) = &inputs.pitch {
                crate::climb_watch::publish_telemetry(p.telemetry());
            }
            crate::climb_watch::publish_sim_speed(inputs.sim_speed);
            if note_interventions(&inputs, flight.control()) {
                hands_at = Some(Instant::now());
            }
            crate::blackbox::record(
                "state",
                serde_json::json!({
                    "lat": lat, "lon": lon, "hdg": r1(snap.heading_deg), "alt": r1(alt),
                    "pos": snap.pos_source.map(|s| s.as_str()),
                    "bank": r1(snap.bank_deg), "track": snap.track_deg.map(r1),
                    "ias": r1(snap.ias_kts), "gs": r1(snap.groundspeed_kts),
                    "vsi": r1(snap.vsi_fpm), "vs": snap.vs_fpm.map(r1), "agl": r1(snap.agl_ft),
                    "ground": snap.on_ground, "flap": snap.flap_ratio,
                    "park": r1(snap.park_brake), "cabin": r1(snap.cabin_altitude_ft),
                    "fuel_kg": snap.fuel_kg, "eng": snap.engine_running,
                    "phase": format!("{:?}", snap.phase),
                    "fphase": format!("{:?}", flight.phase()), "crashed": crashed,
                    "ctl": format!("{:?}", flight.control()),
                    "taxi": taxi_line.map(|l| l.json()),
                    "in": inputs.json(),
                }),
            );

            // Panel settings + the live flight strip. The mute switch comes from
            // the plugin's settings popup; unresolvable (no plugin) = unmuted.
            let mute = xp
                .get_float("xcopilot/opt/monitor_mute")
                .await
                .is_ok_and(|v| v > 0.5);
            PANEL_MUTE.store(mute, std::sync::atomic::Ordering::Relaxed);
            if let Some(s) = status.as_ref() {
                let mut strip = compose_strip(&snap);
                // The glanceable next-turn line: on a busy frequency the
                // spoken cue can be late or dropped (perishable) — the strip
                // is the fallback the pilot can always read.
                if let Some(hint) = taxi_guide
                    .as_ref()
                    .and_then(|g| g.strip_hint(snap.lat, snap.lon))
                {
                    if strip.is_empty() {
                        strip = hint;
                    } else {
                        strip = format!("{strip} | {hint}");
                    }
                }
                if strip != last_strip {
                    s.strip(&strip);
                    last_strip = strip;
                }
                // The turn card behind the panel's arrow: the same geometry
                // as the strip's hint, structured, and empty (card hidden)
                // whenever there is no guide. It rides the periodic resend
                // below so a plugin reload gets it back.
                let card = taxi_guide
                    .as_ref()
                    .and_then(|g| g.turn_card(snap.lat, snap.lon))
                    .map(|c| c.payload())
                    .unwrap_or_default();
                if card != last_card || auto_resend == 0 {
                    s.taxi(&card);
                    last_card = card;
                }
                // The Flight tab's automation feed, published here rather than
                // the 1 Hz radio poller because that thread only exists when
                // ATC is configured — auto status must flow with ATC off.
                let auto = crate::auto_status::compose();
                // The Route tab's load sheet rides the same gate and resend.
                let sheet =
                    crate::simbrief::loadsheet_payload(crate::simbrief::loaded_plan().as_ref());
                if sheet != last_sheet || auto_resend == 0 {
                    s.load_sheet(&sheet);
                    last_sheet = sheet;
                }
                // The Settings tab's resolved callsign under its field
                // ("spoken|source"), through the one resolver: every 5 s
                // (resent, since a plugin reload zeroes it) and at once after
                // a callsign setting changed. A failed read sends nothing.
                let due = CALLSIGN_STALE.swap(false, std::sync::atomic::Ordering::Relaxed)
                    || callsign_at.map_or(true, |t| t.elapsed() >= Duration::from_secs(5));
                if due {
                    callsign_at = Some(Instant::now());
                    if let Some(r) = crate::callsign::live(xp.view()).await {
                        s.callsign(&format!("{}|{}", r.spoken, r.source.as_str()));
                        crate::efb::feed::note_callsign(&r.spoken);
                    }
                }
                // The EFB link under the panel's EFB setting, with its QR:
                // sent on a change (on, off, a new port or token) and on
                // the periodic resend (a plugin reload zeroes it).
                let efb_now = crate::efb::serving_url();
                let efb_changed = last_efb.as_ref() != Some(&efb_now);
                if efb_changed {
                    efb_payload = crate::efb::qr::panel_payload(efb_now.as_deref());
                    last_efb = Some(efb_now);
                }
                if efb_changed || auto_resend == 0 {
                    s.efb_url(&efb_payload);
                }
                // The Checklist tab: once a second, its shown section read
                // live only while the tab is on screen (`checklist_panel`).
                checklist_tab
                    .tick(
                        &xp,
                        s,
                        || live.get().display_name,
                        flight.phase(),
                        Instant::now(),
                    )
                    .await;
                if auto != last_auto || auto_resend == 0 {
                    s.auto_status(&auto);
                    last_auto = auto;
                    auto_resend = 5;
                } else {
                    auto_resend -= 1;
                }
            }

            // How busy the flight deck is, for the voice's pace, the reply's
            // pause and the asides below (`tempo::busy_now`).
            let busy = crate::tempo::busy_now(&crate::tempo::Load {
                on_ground: snap.on_ground,
                groundspeed_kts: snap.groundspeed_kts,
                agl_ft: snap.agl_ft,
                critical_loop: crate::flight_state::crew_in_critical_phase(),
                crew_busy: crate::flow::ACTIVE.load(std::sync::atomic::Ordering::Relaxed)
                    || crate::instructor::active(),
            });
            crate::tempo::set_workload(busy);
            // The rapport SEES every tick, a wreck included (it drops the
            // landing word for one), and speaks only what its tick returns.
            // Its wheels are the flight model's (this tick's events, on the
            // model's clock), never a second touchdown detector (#85).
            if crew_on {
                let view = crate::rapport::View {
                    t_s: wall_t,
                    on_ground: snap.on_ground,
                    groundspeed_kts: snap.groundspeed_kts,
                    agl_ft: snap.agl_ft,
                    vsi_fpm: snap.vsi_fpm,
                    ias_kts: snap.ias_kts,
                    lat: snap.lat,
                    lon: snap.lon,
                    flap_ratio: snap.flap_ratio,
                    gear_down: snap.gear_down,
                    automation_flying: snap.automation_flying,
                    pilot_hands: pilot_hands_recent(hands_at.map(|t| t.elapsed().as_secs_f64())),
                    dest_dist_nm: snap.dest_dist_nm,
                    crashed: voice.crashed(),
                    instructing: crate::mode::instructing(),
                    busy,
                    quiet,
                };
                let persona = crate::persona::Persona::live();
                for line in
                    rapport.tick(&view, &flight, &flight_events, &rapport_airframe, &persona)
                {
                    crate::blackbox::record(
                        "rapport",
                        serde_json::json!({ "event": "said", "line": line.text() }),
                    );
                    voice.alert(Priority::Advisory, line.text().to_string());
                }
            }

            // A wreck gets no advice: the flight model freezes at the impact
            // values (the 2026-07-13 crash read 106 kts and -1,463 fpm
            // forever), so the checks would evaluate a flight that already
            // ended. They stand down until the flight resets. And with no
            // crew aboard (ATC-only mode) there is nobody to speak them.
            if crew_on && !voice.crashed() {
                let flying = snap.automation_flying;
                if flying {
                    had_flying_control = true;
                    control_idle_since = None;
                } else if had_flying_control && !snap.on_ground {
                    let since = *control_idle_since.get_or_insert(now);
                    if now.duration_since(since) >= Duration::from_secs(3) {
                        had_flying_control = false;
                        landing_summary.note_interruption();
                        // The pilot's own "my controls" was answered already;
                        // said again it landed on the next "you have control"
                        // (LEMG 13 1790690743, 2026-09-29, #54).
                        if !pilot_released_recently() {
                            voice.hazard(Severity::Caution, "Flight automation has stopped. I'm monitoring only; you have control. Check the autopilot.");
                        }
                    }
                } else if snap.on_ground {
                    had_flying_control = false;
                }
                let xtk = crate::autoland::last_tracking_error();
                landing_summary.note_flight(&flight_events);
                if let Some(line) = landing_summary.tick(
                    now,
                    snap.on_ground,
                    snap.agl_ft,
                    snap.vsi_fpm,
                    snap.groundspeed_kts,
                    crate::flight_state::Owner::Autoland.active(),
                    xtk,
                ) {
                    // The record for the debrief; the SPOKEN word about the
                    // landing is the rapport's, said the way a person would
                    // ("Nice one, about 180 feet a minute"), not a report.
                    crate::blackbox::record("landing_summary", serde_json::json!({"msg": &line}));
                }
                let mut speed_bad = false;
                let mut speed_spoke = false;
                let mut cabin_bad = false;
                let mut cabin_spoke = false;
                for check in &checks {
                    let state = states.get_mut(check.id()).expect("state per check");
                    match check.eval(&snap) {
                        Some(msg) => {
                            let speed = check.id().starts_with("airspeed_")
                                || check.id().starts_with("overspeed_");
                            let cabin = check.id().starts_with("cabin_alt_");
                            speed_bad |= speed;
                            cabin_bad |= cabin;
                            state.observe_level(check.repeat_level(&snap));
                            let spoke = state.active(
                                check.id(),
                                msg,
                                check.severity(),
                                check.min_duration(),
                                &voice,
                                status.as_ref(),
                                now,
                                check.acts_while_crew_lands(&snap),
                            );
                            speed_spoke |= speed && spoke;
                            cabin_spoke |= cabin && spoke;
                            // The pilot's fuel panel: said, never written.
                            if spoke
                                && check.id() == CENTRE_TANK_DRY
                                && check.fix_write(&snap).is_none()
                            {
                                if let Some(t) = snap.fuel_tanks.as_ref() {
                                    for i in crate::tools::fuel_pumps::pumps_to_switch_off(
                                        &pump_tanks,
                                        Some(t),
                                        snap.engine_running,
                                    ) {
                                        crate::blackbox::record(
                                            "fuel_pumps",
                                            serde_json::json!({
                                                "event": "advised", "source": "monitor",
                                                "tank": i, "kg": t.kg.get(i),
                                                "before": t.pumps.get(i),
                                            }),
                                        );
                                    }
                                }
                            }
                            // On the rising edge, the copilot ACTS on SOP rows that
                            // carry a fix (strobes, landing lights) instead of only
                            // advising — crew business, so it's already behind the
                            // `crew_on` gate above, and the tick never runs in
                            // replay/pause. Fire-and-forget: the switch flips and
                            // the check clears next tick.
                            if spoke && crate::mode::crew_actuates() {
                                if let Some(cmd) = check.fix_action() {
                                    let _ = xp.run_command(cmd).await;
                                }
                                // A cabin climbing through ten thousand: the
                                // crew's own air first (bleed, the oxygen
                                // supply; the masks are `masks_tick`'s), and
                                // past the warning band a follower that is
                                // flying goes DOWN (its terrain floor still
                                // rules). Two "cabin altitude" callouts went by
                                // unanswered on EDLP-LOWI and the pilot blacked
                                // out at FL260 (2026-09-24).
                                if cabin {
                                    let air = crate::tools::cabin_air::ensure_on(&xp).await;
                                    if let Some(line) = air.line() {
                                        voice.alert(Priority::Warning, line);
                                    }
                                    if check.id() == "cabin_alt_warning"
                                        && snap.alt_ft > crate::tools::cabin_air::HYPOXIA_DESCENT_FT + 500.0
                                        && crate::pilot_altitude::take(
                                            crate::tools::cabin_air::HYPOXIA_DESCENT_FT,
                                        )
                                    {
                                        voice.alert(
                                            Priority::Warning,
                                            "Descending to ten thousand for the cabin. \
                                             Ask for a new level once it holds."
                                                .to_string(),
                                        );
                                    }
                                }
                                if let Some((dref, value)) = check.fix_write(&snap) {
                                    // The altimeter goes through the proving
                                    // setter: on the 737 a dial write is a
                                    // silent no-op every 20 s otherwise.
                                    if dref == BARO_KNOB {
                                        // The line above promised a reset; a
                                        // knob that did not get there is said.
                                        if let Ok(now) =
                                            crate::tools::instruments::set_altimeter(&xp, value)
                                                .await
                                        {
                                            let sync = crate::tools::instruments::AltimeterSync {
                                                was: now,
                                                now,
                                                want: value.clamp(28.0, 31.5),
                                            };
                                            crate::blackbox::record(
                                                "callout",
                                                serde_json::json!({"id": "altimeter_reset", "now": now, "want": sync.want, "took": sync.took()}),
                                            );
                                            if !sync.took() {
                                                voice.alert(Priority::Warning, sync.line());
                                            }
                                        }
                                    } else if dref == crate::tools::fuel_pumps::TANK_PUMP_ON {
                                        // An indexed switch per tank, through
                                        // its one writer, read back: the tanks
                                        // are the decision's own.
                                        let tanks = crate::tools::fuel_pumps::pumps_to_switch_off(
                                            &pump_tanks,
                                            snap.fuel_tanks.as_ref(),
                                            snap.engine_running,
                                        );
                                        let done = crate::tools::fuel_pumps::switch_pumps(
                                            &xp,
                                            &tanks,
                                            value > 0.5,
                                        )
                                        .await;
                                        crate::tools::fuel_pumps::record("switched", "monitor", &done);
                                        if let Some(line) = crate::tools::fuel_pumps::failed_line(&done) {
                                            voice.alert(Priority::Caution, line);
                                        }
                                    } else if dref == crate::tools::anti_ice::MONITOR_FIX {
                                        // A set of switches, each read back
                                        // through the tool's one writer.
                                        let wants = snap
                                            .icing
                                            .as_ref()
                                            .map(crate::tools::anti_ice::practice_wants)
                                            .unwrap_or_default();
                                        if let Some(line) = crate::tools::anti_ice::monitor_fix(
                                            &xp,
                                            &wants,
                                            value > 0.5,
                                        )
                                        .await
                                        {
                                            voice.alert(Priority::Caution, line);
                                        }
                                    } else {
                                        let _ = xp.set_float(dref, value).await;
                                    }
                                }
                            }
                        }
                        None => state.clear(),
                    }
                }
                let speed_clear = airspeed_recovery.tick(speed_bad, speed_spoke, now);
                let cabin_clear = cabin_recovery.tick(cabin_bad, cabin_spoke, now);
                if !quiet && !snap.on_ground {
                    if speed_clear {
                        voice.alert(Priority::Advisory, "Airspeed recovered.".to_string());
                    }
                    if cabin_clear {
                        voice.alert(
                            Priority::Advisory,
                            "Cabin altitude back below the caution level.".to_string(),
                        );
                    }
                }
            }
        } else if xp.get_float(crate::datarefs::ON_GROUND_ANY).await.is_err() {
            // No sim: the world runs on alone.
            world.traffic_alone(&xp, &voice, &status, false).await;
            let since = *lost_since.get_or_insert_with(Instant::now);
            if seen_sim && !disconnected && since.elapsed() >= Duration::from_secs(3) {
                disconnected = true;
                had_flying_control = false;
                crate::flight_state::stand_down();
                crate::blackbox::record(
                    "control_ownership",
                    serde_json::json!({"event":"connection_lost"}),
                );
                if crate::mode::copilot_enabled() {
                    voice.hazard(Severity::Warning, "Connection to X-Plane lost. Flight automation is stopped. You'll have control when the sim returns.".to_string());
                }
            }
        } else {
            world.traffic_alone(&xp, &voice, &status, true).await;
            lost_since = None;
        }
        tokio::time::sleep(period).await;
    }
}
