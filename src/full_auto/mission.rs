//! The mission: `fly`, whole.

use super::*;

pub(super) async fn fly(xp: XPlaneClient, config: Config, voice: Option<Voice>, tools: SubTools) {
    // Mutable: the crew re-chooses the departure when the tower's runway is
    // not the plan's (`choose_departure`).
    let mut tools = tools;
    // One mission is one sortie in the blackbox (`logs/sorties/<id>.jsonl`).
    let _sortie = SortieGuard::begin(
        "full_auto",
        match &tools.mission {
            Mission::Flight(r) | Mission::Resume(r) => json!({
                "origin": r.origin_icao, "dest": r.dest_icao,
                "resume": matches!(tools.mission, Mission::Resume(_)),
            }),
            _ => json!({}),
        },
    );
    // A new mission starts owing nothing to the last one's destination: a pin
    // left by the previous arrival would keep the region from ever handing
    // this departure on (see `release_arrival_station`).
    release_arrival_station();
    // The mission has the aircraft: a Crew checklist still running from the
    // last flight (the after-landing list, blocked on its own speech) must
    // not write under it. LEMG (2026-09-28): "Flaps, up." over the takeoff
    // flaps set 45 s earlier, on two missions in a row.
    crate::flow::stand_down(crate::flow::StandDown::Mission("full auto"));
    macro_rules! bail_if_abort {
        () => {
            if ABORT.load(SeqCst) {
                give_up(
                    &xp,
                    tools.taxi.park_brake_valve_set,
                    &voice,
                    "Cancelled. Your aircraft.",
                    "abort",
                )
                .await;
                return;
            }
        };
    }
    // One context, reused for every tool call (all borrows are immutable).
    let ctx = ToolCtx {
        xplane: &xp,
        config: &config,
        voice: voice.as_ref(),
        status: None,
        reminders: None,
    };

    if let Mission::Resume(route) = &tools.mission {
        let callsign = crate::crew_radio::crew_callsign(&xp).await;
        // The normal follower owns speed, route and arrival; no duplicate laws.
        fly_route(&xp, &config, &voice, &ctx, &tools, route, &callsign).await;
        return;
    }

    // "Starting cold" to a running engine (a re-arm after a give-up at the
    // hold, LIME 2026-09-25) is a line the pilot knows is false.
    let already_running = engine_running(&xp).await;
    announce(
        &voice,
        Priority::Advisory,
        // The arming reply has just said "Full auto, X to Y"; a second
        // "Full auto it is" straight after it was the same news twice.
        if already_running {
            crate::readback::one_of(&["Picking up from here.", "Carrying on from here."])
        } else {
            crate::readback::one_of(&["Starting from cold.", "From cold, then.", "Starting her up."])
        },
    );
    event("start", json!({}));
    // Full auto is a GATE-to-gate chain: armed in the air it walks its ground
    // stages against reality. After the missed-approach give-up the pilot hit
    // FULL AUTO at 1,475 ft descending at 900 fpm, and the chain read a stale
    // `LAST_ARRIVED` from the morning's taxi as "at the hold", called ready
    // for departure, and requested a takeoff clearance while the aircraft
    // sank (EGHI, 2026-08-02). Airborne, the honest answer is the tools that
    // fly: refuse and name them.
    let (agl, ground) = tokio::join!(
        xp.get_float(AGL_M),
        xp.get_float(crate::datarefs::ON_GROUND_ANY),
    );
    // Both instruments must affirm the ground gate; an API failure is no
    // clearance (`sim::ground::on_ground_confirmed`, the one definition).
    if !sim::ground::on_ground_confirmed(agl.ok(), ground.ok()) {
        give_up(
            &xp,
            tools.taxi.park_brake_valve_set,
            &voice,
            "I need confirmed ground contact to start the whole flight. Say navigate to follow the plan, fly the ILS, or land for me when airborne.",
            "ground_unconfirmed",
        )
        .await;
        return;
    }
    // The station must be the field we are AT before anything asks it for a
    // thing: a new flight 1,600 km from the last kept Malaga's station, and
    // Gatwick was departed as an uncontrolled field with its stand looked up
    // at LEMG (EGKK 2026-09-28, #151).
    if crate::mode::atc_enabled() {
        if let Err(line) = departure_station(&xp).await {
            give_up(&xp, tools.taxi.park_brake_valve_set, &voice, &line, "departure_station").await;
            return;
        }
    }
    bail_if_abort!();
    // The callsign the tower knows us by, so our calls sound like a real pilot.
    let callsign = crate::crew_radio::crew_callsign(&xp).await;

    // On a Flight mission the route goes into the GPS NOW, at the gate. The
    // write used to sit at the top of stage 4 — fly_route was where the
    // concrete client happened to be — so the whole start-up and taxi ran
    // with an empty flight-plan page, and the pilot read that as "it didn't
    // load" and commanded the load by hand mid-taxi (LXGB to LEMG,
    // 2026-08-02). The map during the taxi is half of what loading the plan
    // is FOR. Best effort, exactly as it was at the runway.
    if let Mission::Flight(route) = &tools.mission {
        if !route.fms_payload.is_empty() {
            let _ = xp
                .set_data("xcopilot/fms/route", route.fms_payload.as_bytes())
                .await;
            let _ = xp.set_int("xcopilot/fms/load", 1).await;
            event(
                "fms_loaded",
                json!({ "waypoints": route.fms_payload.split('|').count() }),
            );
        }
    }

    // --- Stage 1: electrics + engine ------------------------------------------
    if !engine_running(&xp).await {
        // A turbine we have no sequence for is not a piston. Say so and stop,
        // rather than crank a starter this airframe hasn't got — the failure
        // the MD-82 taught us, from the other side (see `StartKind::None`).
        if tools.no_start_sequence {
            give_up(
                &xp,
                tools.taxi.park_brake_valve_set,
                &voice,
                "I don't have an engine start for this aircraft. Get the engines running, then ask me to fly again.",
                "no_start_sequence",
            )
            .await;
            return;
        }
        // Only turbine airframes need the APU/electrical bring-up; a piston's
        // start_engine powers its own bus.
        if tools.power_up_first {
            announce(
                &voice,
                Priority::Advisory,
                crate::readback::one_of(&[
                    "Powering up.",
                    "Getting some power on.",
                    "Bringing her to life.",
                ]),
            );
            // Run the bring-up INLINE, not through the PowerUp tool: the tool
            // spawns the sequence and answers "running the electrical"
            // immediately, so full auto walked straight on to the engine start,
            // which found the APU still down and started a SECOND copy of the
            // same sequence. Two racing copies both read a cross-tie as open and
            // both fire its toggle command, which leaves it shut. Here the
            // orchestrator is already on its own thread, so waiting is free and
            // "power up, then crank" is true again.
            crate::tools::power::run_prep(&xp, &voice, &None).await;
            event("power_up", json!({ "inline": true }));
            bail_if_abort!();
        }

        // Power the DC bus the way THIS airframe does it before cranking: the
        // turboprop start only checks volts, and the Twin Otter's battery is a
        // custom command the stock write can't set — so a cold start read 0 V
        // and never turned a blade (2026-07-26). No-op for pistons (empty).
        if !tools.bus_power_on_cmds.is_empty() {
            for cmd in &tools.bus_power_on_cmds {
                let _ = xp.run_command(cmd).await;
            }
            // The beat is the bus coming alive (it used to be a flat 600
            // ms); then the volts are the readback. The one ramp line said
            // in both modes, and only once the bus reads live.
            crate::crew_pace::pause(crate::crew_pace::beat_ms()).await;
            let live = xp
                .get_float_array(crate::datarefs::BUS_VOLTS)
                .await
                .ok()
                .map(|v| v.iter().any(|b| *b > BUS_LIVE_VOLTS));
            match live {
                Some(true) => crate::crew_pace::milestone(voice.as_ref(), "Battery's on.").await,
                other => {
                    crate::crew_pace::checked(
                        voice.as_ref(),
                        other,
                        "Battery's on.",
                        "The battery didn't come on.",
                    )
                    .await
                }
            }
            event(
                "bus_power",
                json!({ "cmds": tools.bus_power_on_cmds.len(), "live": live }),
            );
            bail_if_abort!();
        }

        // The crew's preflight on the airframe's own switches, where it
        // declares one: the APU and its generators, then the overhead in
        // the checklist's order, before the ramp items, so the lamps they
        // read back are powered (the Zibo's are AC: "Nav lights did not
        // come on" on the battery alone, EGHL 2026-10-06).
        if let Some(pf) = &tools.preflight {
            let apu = tools
                .preflight_apu
                .clone()
                .map(crate::tools::jet_start::BoeingStart::new);
            let mut say = |s: &str| announce(&voice, Priority::Advisory, s);
            let mut panel = |_: &str| {};
            match crate::preflight::run(&xp, pf, apu.as_ref(), &mut say, &mut panel).await {
                Ok(not_taken) => {
                    event("preflight", json!({ "not_taken": not_taken }));
                    if let Some(line) = preflight_line(&not_taken) {
                        announce(&voice, Priority::Advisory, &line);
                    }
                }
                Err(line) => {
                    event("preflight", json!({ "failed": line }));
                    give_up(&xp, tools.taxi.park_brake_valve_set, &voice, &line, "preflight_power")
                        .await;
                    return;
                }
            }
            bail_if_abort!();
        }

        // Beacon BEFORE the propeller moves — that is what it is for, and it
        // is the one light with an actual safety job on a ramp. Nav lights with
        // it, since they belong on whenever the master is.
        // One item at a time, at a person's pace (`crew_pace`): the whole
        // ramp used to be done in the same second. Each is called only once
        // its switch READ BACK on, and named when it did not take.
        // A scripted start resets the ramp to cold and dark as it begins
        // (B738.Z.autostart step 1 turned our beacon, nav lights and seat
        // belts back off, LXGB 2026-10-06) and lights its own beacon before
        // the starters: its ramp is checked once the engines run.
        let beacon = if tools.start_scripted {
            false
        } else {
            ramp_items(&tools, &ctx, &xp, &voice).await
        };
        // Only claim the beacon on an airframe that has one — the Lancair's
        // switch row is NAV / LAND / STRB and nothing else.
        announce(
            &voice,
            Priority::Advisory,
            if beacon && !crate::crew_pace::items_spoken() {
                crate::readback::one_of(&[
                    "Beacon on. Starting the engine.",
                    "Beacon's on, spinning up the engine.",
                    "Beacon on, cranking her over.",
                ])
            } else {
                crate::readback::one_of(&[
                    "Starting the engine.",
                    "Spinning up the engine.",
                    "Cranking her over.",
                ])
            },
        );
        let reply = tools.start_engine.execute(&json!({}), &ctx).await;
        // An inline start has answered with its verdict. Engines not running
        // once it has is a start that stopped and said "Your aircraft." in
        // its reply, and the mission ends there: it used to say so and then
        // wait out the five-minute class deadline holding the aircraft as
        // `Automation("full auto")` (EGHL 2026-10-06, the Zibo's duct
        // pressure refusal).
        if tools.start_inline {
            bail_if_abort!();
            let running = wait_until(START_SETTLE_SECS, 500, || engine_running(&xp)).await;
            if let Some((line, reason)) = inline_start_hand_back(running, reply.as_deref().ok()) {
                event("start_stopped", json!({ "reply": reply.as_ref().ok() }));
                give_up(&xp, tools.taxi.park_brake_valve_set, &voice, &line, reason).await;
                return;
            }
        }
        if let Ok(reply) = reply {
            announce(&voice, Priority::Advisory, &reply);
        }
        // The starter cranks before it catches — for seconds on a piston, for
        // minutes on the jet's two-engine bleed-air sequence.
        waiting("the engine to catch");
        // A scripted start (the 737's) is WATCHED by its tool
        // (`tools::scripted_start`): its verdict ends the wait, an abort at
        // once, where the class deadline waited five minutes on a sequence
        // that had stopped at 25% N2 (2026-10-06 09:11Z). Every other start,
        // and a watch that timed out or stood down, is judged on the engine.
        let started = wait_until(tools.engine_start_secs, 500, || async {
            use crate::tools::scripted_start::{state, WatchState};
            match state() {
                WatchState::Watching => false,
                WatchState::Aborted => true,
                WatchState::Running | WatchState::Idle => engine_running(&xp).await,
            }
        })
        .await;
        if crate::tools::scripted_start::state()
            == crate::tools::scripted_start::WatchState::Aborted
            && !ABORT.load(SeqCst)
        {
            // The watch has said why; this is the hand back.
            give_up(
                &xp,
                tools.taxi.park_brake_valve_set,
                &voice,
                "I've stopped. Your aircraft.",
                "start_aborted",
            )
            .await;
            return;
        }
        if !started {
            give_up(
                &xp,
                tools.taxi.park_brake_valve_set,
                &voice,
                "The engine didn't catch. I've stopped. Your aircraft.",
                "no_engine",
            )
            .await;
            return;
        }
        event("engine_running", json!({}));
        if tools.start_scripted {
            // The aircraft's script has had the ramp; this is the crew's
            // check of it, set where it is not.
            ramp_items(&tools, &ctx, &xp, &voice).await;
            bail_if_abort!();
        }

        // Complete the electrical now the engine's turning: generators AND
        // bleed air on. On the Twin Otter the bleeds feed the autopilot and
        // anti-ice, so a circuit flown without them has no servos. Skipped when
        // PowerUp already ran the bus (jets), and a no-op on airframes with no
        // electrical profile. Battery/master re-fire harmlessly.
        if !(tools.power_up_first
            || tools.all_electrical_on_cmds.is_empty() && tools.all_electrical_toggles.is_empty())
        {
            for cmd in &tools.all_electrical_on_cmds {
                let _ = xp.run_command(cmd).await;
            }
            crate::crew_pace::pause(crate::crew_pace::beat_ms()).await;
            // Toggle-only systems have no ON command, so a bulk power-up that
            // fires command lists skips them SILENTLY. That left the Citation's
            // avionics master off after "electrical set", and the radios dead.
            for t in &tools.all_electrical_toggles {
                if xp.get_float(&t.dataref).await.unwrap_or(0.0) <= 0.5 {
                    let _ = xp.run_command(&t.command).await;
                }
            }
            // What was set is the profile's list; "generators and bleeds"
            // on a 172 was an airliner's line (2026-09-15), and the start
            // line has already named the alternator and avionics.
            // Said where it adds something: a turbine's generators and
            // bleeds. A piston's start has just brought the alternator and
            // avionics on at their own beats, and "Electrical set" after it
            // was a second recap of the same switches.
            if tools.start_engine.name() != "start_engine" {
                announce(&voice, Priority::Advisory, "Electrical set.");
            }
            event(
                "electrical_set",
                json!({
                    "cmds": tools.all_electrical_on_cmds.len(),
                    "toggles": tools.all_electrical_toggles.len(),
                }),
            );
        }
    }
    bail_if_abort!();

    // The after-start flow's last item: the takeoff configuration, set with
    // the whole taxi ahead of it, not during the roll (see `takeoff_config`).
    // The takeoff confirms it before the brakes come off and there selects
    // the level its caller hands down; on the ground and engines running
    // only, so an airborne resume is never reconfigured for a takeoff.
    if !matches!(tools.mission, Mission::Resume(_)) && engine_running(&xp).await {
        let cfg = crate::takeoff_config::set_for_takeoff(
            &xp,
            tools.takeoff.takeoff_flap_ratio,
            None,
            false,
        )
        .await;
        if let Some(line) = crate::takeoff_config::line("Takeoff config", &cfg) {
            announce(&voice, Priority::Advisory, &line);
        }
        event(
            "takeoff_config",
            json!({"set": cfg.set, "not_taken": cfg.not_taken}),
        );
    }
    bail_if_abort!();

    // The cabin's air, on EVERY mission including an airborne resume: the
    // bleed that pressurises and the crew oxygen, where the airframe declares
    // them. It used to ride the engine start alone, so a departure with the
    // engine already running climbed an unpressurised SF50 to FL260 and the
    // pilot blacked out (EDLP-LOWI, 2026-09-24). Read first, left alone in
    // state, read back.
    if engine_running(&xp).await {
        let air = crate::tools::cabin_air::ensure_on(&xp).await;
        if let Some(line) = air.line() {
            announce(&voice, Priority::Advisory, &line);
        }
        event(
            "cabin_air",
            json!({"bleed": air.bleed, "oxygen": air.oxygen, "changed": air.changed()}),
        );
    }
    bail_if_abort!();

    // The IRS the preflight set aligning, waited on before anything moves:
    // the platform is the attitude the autopilot flies on, so no mission
    // taxis out on one still aligning. Begun at the preflight, it has had
    // the whole start to run; the wait is bounded and said.
    if let Some(align) = tools.preflight.as_ref().and_then(|p| p.align.as_ref()) {
        waiting("the I.R.S. to align");
        let mut say = |s: &str| announce(&voice, Priority::Advisory, s);
        let verdict =
            crate::preflight::wait_aligned(&xp, align, &|| ABORT.load(SeqCst), &mut say).await;
        event("irs_align", json!({ "verdict": format!("{verdict:?}") }));
        bail_if_abort!();
        let hand_back = match verdict {
            crate::preflight::AlignVerdict::TimedOut { remaining_s } => {
                Some(crate::preflight::align_timeout_line(remaining_s))
            }
            crate::preflight::AlignVerdict::Off => Some(
                "The I.R.S. is off, so there's no attitude to fly on. Your aircraft.".to_string(),
            ),
            _ => None,
        };
        if let Some(line) = hand_back {
            give_up(&xp, tools.taxi.park_brake_valve_set, &voice, &line, "irs_not_aligned").await;
            return;
        }
        let after = tools.preflight.as_ref().map(|p| p.after_aligned.clone()).unwrap_or_default();
        if !after.is_empty() {
            let mut say = |s: &str| announce(&voice, Priority::Advisory, s);
            let mut panel = |_: &str| {};
            let not_taken = crate::tools::jet_start::run_steps(&xp, &after, &mut say, &mut panel).await;
            event("after_aligned", json!({ "not_taken": not_taken }));
            if !not_taken.is_empty() {
                announce(&voice, Priority::Advisory, &format!("{} didn't take.", not_taken.join(", ")));
            }
        }
    }

    // --- Stage 2: altimeter ---------------------------------------------------
    // NO brake release here, deliberately. The brakes used to come off at this
    // stage — engine just started, taxi clearance not yet requested — which is
    // minutes before anything drives, and an idle jet on a flat apron rolls:
    // the SF50 crept from 0 to 2.8 kt with `park=0.0` while full auto was
    // still WAITING FOR THE TUG, and ended up in the grass (Luton,
    // 2026-07-28). Nobody needs the early release: the taxi governor
    // auto-releases (valve-aware) when a drive arms, the pushback stage
    // handles its own brakes-off at hook-up, and the auto-takeoff releases on
    // the runway. Brakes stay as the pilot left them until something is
    // actually driving.

    // Set the altimeter to field QNH before departure — a misset altimeter is
    // the classic level-bust. Autoland aims at the runway's true elevation so it
    // doesn't depend on this, but a real crew sets it, and everything MSL between
    // here and there reads right once it's set.
    if let Some(sync) =
        crate::tools::instruments::sync_altimeter(&xp, tools.transition_altitude_ft).await
    {
        let priority = if sync.took() {
            Priority::Advisory
        } else {
            Priority::Warning
        };
        announce(&voice, priority, &sync.line());
        event(
            "altimeter_set",
            json!({ "inhg": (sync.now * 100.0).round() / 100.0, "was": (sync.was * 100.0).round() / 100.0, "want": (sync.want * 100.0).round() / 100.0, "took": sync.took() }),
        );
    }

    // ...and align the DG to the compass, the other half of the same two
    // items. The monitor's auto-sync is deliberately AIRBORNE-only, on the
    // rule that on the ground you align at the run-up — which is right, and
    // full auto IS the run-up. Left to the monitor, the sync lands at the
    // first moment the wheels leave the ground: at Southampton a Baron taxied,
    // lined up and started its roll with the heading indicator 159.7 degrees
    // out, and the alignment arrived twenty-six seconds after the line-up
    // (2026-08-01). Nothing the automation flies reads the DG (the AP bug is a
    // magnetic heading, and every ground loop uses true heading), so this is
    // for the pilot watching the panel — which is reason enough.
    let _ = xp.run_command("sim/instruments/DG_sync_mag").await;
    event("gyro_synced", json!({}));
    bail_if_abort!();

    // --- Stage 3: onto the runway ---------------------------------------------
    // A tower that can hear us, or an uncontrolled departure: no
    // simulated ATC, or no station in reach of this field (LILB,
    // 2026-09-25: Gibraltar, 1,000 nm away, cleared the 172 and gave it a
    // taxi to its own runway 09, and full auto waited three minutes for
    // a route that could be ours).
    let towered = crate::mode::atc_enabled() && crate::party_line::station_reaches();
    // An IFR plan asks for its route: the SID, the altitude and the
    // code the rest of the departure flies.
    let ifr_dest = match &tools.mission {
        Mission::Flight(r) if r.ifr => Some(r.dest_icao.clone()),
        _ => None,
    };
    // Already on a runway (the pilot or the sim put us there)? Lined up with
    // the roll ahead: call ready from here. Too little ahead: backtrack. A
    // runway start used to ask for a taxi when the length ahead fell short,
    // and was cleared off the runway to its far hold (EGKK 26L, #128).
    let mut start = runway_start(&xp, &tools).await;
    let mut clearance_asked = false;
    if start != RunwayStart::None && towered {
        // The clearance first, from the strip too (IFR route or zone): it
        // names the runway the departure is flown from. A zone clearance is
        // asked FOR the strip we sit on, and asked again when the one held
        // names another (a clearance the last flight left, LEMG #134).
        match (ifr_dest.as_deref(), start.runway()) {
            (None, Some(rwy)) => {
                let rwy = rwy.to_string();
                zone_clearance_from_the_strip(&xp, &ctx, &voice, &callsign, &rwy).await;
            }
            _ => get_zone_clearance(&xp, &ctx, &voice, &callsign, ifr_dest.as_deref()).await,
        }
        clearance_asked = true;
        bail_if_abort!();
        let cleared = crate::party_line::clearance_runway();
        let on = start.runway().map(str::to_string);
        start = start.under_clearance(cleared.as_deref());
        if start.runway() != on.as_deref() {
            let backtrack = start != RunwayStart::None;
            event(
                "runway_start_other_runway",
                json!({ "on": on, "cleared": cleared, "backtrack": backtrack }),
            );
            match (&cleared, &on) {
                (Some(c), Some(o)) if backtrack => {
                    announce(&voice, Priority::Advisory, &backtrack_line(o, c));
                }
                (Some(c), _) => announce(
                    &voice,
                    Priority::Advisory,
                    &format!(
                        "We're cleared off runway {}, not this one. Asking for the taxi.",
                        crate::atis::spoken_runway(c)
                    ),
                ),
                _ => {}
            }
        }
    }
    if let RunwayStart::Short { runway } = &start {
        if let Err((line, why)) =
            backtrack_from_runway_start(&xp, &ctx, &voice, &tools, &callsign, runway, towered).await
        {
            give_up(&xp, tools.taxi.park_brake_valve_set, &voice, &line, why).await;
            return;
        }
        bail_if_abort!();
        start = RunwayStart::Lined {
            runway: runway.clone(),
        };
    }
    // A runway start calls ready from the strip, and the answer may be for
    // another runway: the field turned round under a clearance for ours (the
    // calm-wind stream), or the tower sequences us elsewhere. Then the start
    // is no start: nothing is read back from here, the crew taxis to the
    // cleared runway like any departure (#178).
    let mut start_cleared = false;
    // At most one backtrack: a reciprocal answer to the ready call is flown
    // down the strip, and the ready call made again from its far end.
    let mut backtracked = false;
    while let Some(on) = start.runway().map(str::to_string) {
        if crate::mode::atc_enabled() && crate::party_line::clearance_state().is_some() {
            if let Some(khz) = crate::party_line::home_tower_khz() {
                let _ = xp.set_float(COM1_FREQ_KHZ, khz as f64).await;
            }
            transponder_before_departure(&xp, &voice, "runway_start").await;
            request_departure(&callsign, Some(&on)).await;
            bail_if_abort!();
            let mut read_back = false;
            waiting("the takeoff clearance");
            match ensure_takeoff_cleared(&voice, &callsign, &mut read_back, false, Some(&on)).await
            {
                ClearanceWait::Cleared => start_cleared = true,
                ClearanceWait::NotCleared => {
                    give_up(
                        &xp,
                        tools.taxi.park_brake_valve_set,
                        &voice,
                        "We're on the runway, but the tower never cleared us for takeoff. \
                         Your aircraft. Say full auto again once we're cleared.",
                        "no_takeoff_clearance",
                    )
                    .await;
                    return;
                }
                ClearanceWait::OtherRunway(cleared) => {
                    let backtrack = !backtracked && is_reciprocal(&on, &cleared);
                    event(
                        "runway_start_other_runway",
                        json!({ "on": on, "cleared": cleared, "at": "ready",
                                "backtrack": backtrack,
                                "state": format!("{:?}", crate::party_line::clearance_state()) }),
                    );
                    if backtrack {
                        announce(&voice, Priority::Advisory, &backtrack_line(&on, &cleared));
                        if let Err((line, why)) = backtrack_from_runway_start(
                            &xp, &ctx, &voice, &tools, &callsign, &cleared, towered,
                        )
                        .await
                        {
                            give_up(&xp, tools.taxi.park_brake_valve_set, &voice, &line, why)
                                .await;
                            return;
                        }
                        bail_if_abort!();
                        backtracked = true;
                        start = RunwayStart::Lined { runway: cleared };
                        continue;
                    }
                    announce(&voice, Priority::Advisory, &other_runway_line(&on, &cleared));
                    start = RunwayStart::None;
                }
            }
            bail_if_abort!();
        }
        break;
    }
    if start == RunwayStart::None {
        event("departure_field", json!({ "towered": towered }));
        // Clearance first — the real order is clearance, THEN taxi. Only with a
        // simulated tower to answer (real/external ATC and copilot-only leave the
        // clearance to the pilot), and only if we don't already hold a squawk.
        if towered && !clearance_asked {
            get_zone_clearance(&xp, &ctx, &voice, &callsign, ifr_dest.as_deref()).await;
        }
        bail_if_abort!();
        // The taxi route is the tower's. Ask for it if we don't already hold
        // one — or if the one we hold WAS PLANNED FROM SOMEWHERE ELSE. A
        // clearance outlives a reposition: the tower planned a route from the
        // aircraft's spot on Stansted's 22C, full auto's first attempt gave
        // up waiting one second before it arrived, the pilot moved to a gate
        // a kilometre away, and the SECOND attempt reused the fossil — read
        // "nose in" against a lane 1 km off, called a tug to push toward it,
        // and the taxi drive then (correctly) refused at 1090 m off route
        // (2026-07-29). A route is only OURS if it starts within reach of
        // the auto-taxi's own join limit; otherwise ask again and the tower
        // re-plans from where we actually are.
        // ...and a route to a STAND is never a departure's, however near it
        // starts. An arrival's clearance ends where we are parked, so the
        // reach test below passes it with room to spare: at Malaga full auto
        // adopted the taxi-in route it had just finished driving, skipped the
        // request entirely, drove 33 s of an already-complete 88 m route back
        // toward the stand from 66 m off it, and announced itself at the
        // holding point — the tower's answer to the departure request was
        // "taxi to holding point Hotel Whiskey one FIRST", and the line-up
        // then failed (2026-08-05). The party line already knows which kind
        // of route it published; nobody had asked it.
        let route_is_ours = taxi_route_is_ours(&xp).await;
        if !route_is_ours && !towered {
            if let Err(why) = self_taxi(&xp, &config, &voice, &tools).await {
                give_up(
                    &xp,
                    tools.taxi.park_brake_valve_set,
                    &voice,
                    &format!("{why} Your aircraft."),
                    "no_self_taxi",
                )
                .await;
                return;
            }
        } else if !route_is_ours {
            if crate::party_line::with_taxi_route(|r| r.is_some()) {
                let arrival = crate::party_line::taxi_to_stand();
                announce(
                    &voice,
                    Priority::Advisory,
                    if arrival {
                        "That was our taxi in to the stand. Asking for a departure clearance."
                    } else {
                        "That taxi clearance was from the old spot. Asking again from here."
                    },
                );
                event("stale_taxi_route", json!({ "arrival": arrival }));
            }
            // Tune COM1 to the tower first — the request goes nowhere if we're
            // off frequency (the crew radio only reaches the tuned station).
            if let Some(khz) = crate::party_line::home_tower_khz() {
                let _ = xp.set_float(COM1_FREQ_KHZ, khz as f64).await;
                event("tuned_tower", json!({ "khz": khz }));
            }
            // The planned departure's runway, asked for (the tower honours it
            // unless the wind really favours another end): a SID exists for
            // one runway, and in calm air the pick could have been the other.
            let cleared_rwy = crate::party_line::assigned_clearance().map(|c| c.runway);
            radio_out(crate::crew_radio::radio_call(
                &callsign,
                &taxi_call(&tools.mission, cleared_rwy.as_deref()),
            ));
            event("request_taxi", json!({ "cleared_rwy": cleared_rwy }));
            // The call queues behind our own intercom lines AND anyone the
            // tower is already working — the clock must outlast a worked
            // arrival. 25 s gave up ONE SECOND before a busy Jerez frequency
            // came back with the clearance (16:45Z, 2026-07-24) — and 90 s
            // then gave up ONE SECOND before a busy Stansted did the same
            // thing (2026-07-29, the tower was working a Ryanair arrival and
            // berating us for sitting on its runway). Whatever this number
            // is, a queued frequency will someday beat it by a second; 180
            // outlasts two worked arrivals.
            waiting("a taxi route");
            // Wait for a route that is OURS, by the same test that just
            // rejected the old one — not merely for "a route exists".
            //
            // The stale route is still published while we ask for a new one
            // (the party line is the world's to write; the crew cannot
            // retire it, and should not be able to). So the weaker condition
            // was satisfied by the very route it had just announced as
            // stale: at Edinburgh the request went out and `at_hold`,
            // `request_departure` and the line-up all fired 0.2 s later, on a
            // route starting 180 m away that the taxi drive then refused
            // (2026-09-20). Six stages in a fifth of a second is not a taxi.
            let got = wait_until(180.0, 1000, || async { taxi_route_is_ours(&xp).await }).await;
            if !got {
                give_up(
                    &xp,
                    tools.taxi.park_brake_valve_set,
                    &voice,
                    "No taxi clearance came back. Tune the tower and request taxi, then ask me to continue the flight.",
                    "no_taxi_clearance",
                )
                .await;
                return;
            }
            // The runway it bought is judged before a wheel turns, and a
            // short one is refused and a longer one asked for.
            if let Err(line) = judge_cleared_runway(&xp, &tools, &voice, &callsign).await {
                give_up(
                    &xp,
                    tools.taxi.park_brake_valve_set,
                    &voice,
                    &line,
                    "cleared_runway_short",
                )
                .await;
                return;
            }
        }
        bail_if_abort!();

        // Off the stand FIRST, when the cleared route's way out runs behind
        // us: at a nose-in gate the taxi drive's opening metres point into
        // the terminal. Read from the route itself, so a tie-down we can taxi
        // straight out of never calls a tug (see `pushback::plan`). Best
        // effort by design — a field or airframe with no tug falls back to
        // the taxi join's own turn-around, which is ugly on an apron but is
        // exactly the behaviour that shipped before this stage existed. The
        // push itself is narrated by the monitor's pushback watch, so the
        // only crew line here is the ask.
        {
            let (lat_r, lon_r, psi_r, mtow_r) = tokio::join!(
                xp.get_float(LAT),
                xp.get_float(LON),
                xp.get_float(crate::autotaxi::PSI_TRUE),
                xp.get_float(crate::datarefs::ACF_M_MAX_KG),
            );
            let mtow_kg = mtow_r.ok();
            let path = crate::party_line::with_taxi_route(|r| r.map(|w| w.path.clone()))
                .unwrap_or_default();
            if let (Ok(lat), Ok(lon), Ok(psi)) = (lat_r, lon_r, psi_r) {
                // A push is owed by the route; where the stand's lane is
                // PAINTED, the push is planned off the paint, unless the
                // paint ends facing away from the route (`choose_plan`, #190).
                let plan = crate::pushback::plan(lat, lon, psi, &path).map(|route_plan| {
                    let lane = crate::party_line::push_lane(lat, lon, psi)
                        .and_then(|l| crate::pushback::lane_plan(&l));
                    let lane_seen = lane.clone();
                    let route_hdg = route_plan.target_hdg;
                    let (chosen, used) = crate::pushback::choose_plan(route_plan, lane);
                    if let Some(lp) = lane_seen {
                        event("push_lane", json!({ "back_m": lp.back_m, "target_hdg": lp.target_hdg,
                                                    "route_target_hdg": route_hdg, "used": used }));
                    }
                    chosen
                });
                let stand = crate::party_line::nearest_stand(lat, lon);
                let at_stand = crate::pushback::on_a_stand(
                    stand.as_ref().map(|s| (s.dist_m, s.heading)),
                    psi,
                    crate::party_line::stands_known(),
                );
                if plan.is_some() && !at_stand {
                    // The way out is behind us but this is not a stand: a
                    // restart from an off-route stop, mid-taxiway. The taxi
                    // join turns the aircraft round; a tug here is theatre.
                    announce(
                        &voice,
                        Priority::Advisory,
                        "Way out's behind us, but we're not on a stand, so no tug. \
                         I'll turn us round under our own steam. Watch the wingtips.",
                    );
                    event(
                        "pushback_skipped",
                        json!({
                            "why": "not_on_stand",
                            "stand": stand.as_ref().map(|s| s.name.clone()),
                            "dist_m": stand.as_ref().map(|s| s.dist_m),
                        }),
                    );
                }
                let tug = crate::pushback::tug_for_weight(mtow_kg);
                if plan.is_some() && at_stand && !tug {
                    // A light aircraft turns itself off the stand: the tug
                    // is minutes of waiting, or never comes (LEMG GA, #170).
                    announce(
                        &voice,
                        Priority::Advisory,
                        "Way out's behind us. No tug for us, I'll turn us round under our \
                         own steam. Watch the wingtips.",
                    );
                    event(
                        "pushback_skipped",
                        json!({
                            "why": "light_aircraft",
                            "mtow_kg": mtow_kg,
                            "stand": stand.as_ref().map(|s| s.name.clone()),
                        }),
                    );
                }
                if let Some(plan) = plan.filter(|_| at_stand && tug) {
                    announce(
                        &voice,
                        Priority::Advisory,
                        crate::readback::one_of(&[
                            "We're nose in. Calling for pushback.",
                            "Way out's behind us. Calling for a push.",
                            "Nose in here, so pushback first.",
                        ]),
                    );
                    event(
                        "pushback",
                        json!({ "back_m": plan.back_m, "target_hdg": plan.target_hdg }),
                    );
                    match crate::pushback::run(
                        &xp,
                        &ctx,
                        &voice,
                        &plan,
                        &ABORT,
                        tools.taxi.park_brake_valve_set,
                    )
                    .await
                    {
                        Ok(back_m) => event("pushback_done", json!({ "back_m": back_m })),
                        Err(why) => {
                            event("pushback_failed", json!({ "why": why }));
                            // Say what actually happened. One line covered
                            // every failure, so a tug that came, hooked on and
                            // pushed us fifty metres was reported as "no tug
                            // came" — the pilot watching it out the window has
                            // no reason to trust anything else the copilot
                            // says after that (2026-07-27). An abort is the
                            // pilot's own and is already spoken for.
                            let line = match why.as_str() {
                                "abort" => "",
                                "no_tug" => {
                                    "No tug came, so I'll turn us around under our own steam. \
                                     Watch the wingtips."
                                }
                                "tug_released" => {
                                    "The tug let go early. I'll bring us round the rest of the \
                                     way myself. Watch the wingtips."
                                }
                                "skipped" => {
                                    "Skipping the push. I'll turn us round under our own steam. \
                                     Watch the wingtips."
                                }
                                "creeping" => {
                                    "The tug never pushed us and we were rolling forward, so \
                                     I've got the brakes. I'll turn us around under our own \
                                     steam. Watch the wingtips."
                                }
                                _ => {
                                    "That's as far as the push got. I'll bring us round the rest \
                                     of the way myself. Watch the wingtips."
                                }
                            };
                            if !line.is_empty() {
                                announce(&voice, Priority::Advisory, line);
                            }
                        }
                    }
                }
            }
        }
        bail_if_abort!();

        // The cleared runway against the planned departure: a SID exists for
        // its runway, and taking off on another would fly its fixes backwards
        // (at LOWI a turn back through the valley). Said and handed back here,
        // on the ramp, with the brakes on, never discovered in the climb.
        // ...but first the crew CHOOSES its own departure from the runway it
        // was given: a SID from that runway that joins the route, or direct
        // to the planned SID's exit (LIME 2026-09-25: the tower gave 10 for
        // a plan departing 28, and full auto handed back on the ramp).
        // ...and an IFR clearance's SID is the departure flown when it
        // serves the cleared runway: the crew's own pick is the fallback,
        // and stays the whole behaviour with no clearance.
        let cleared = crate::party_line::clearance_runway();
        let ifr = crate::party_line::assigned_clearance();
        let cleared_sid = cleared_sid_for(ifr.as_ref(), cleared.as_deref());
        let (planned_rwy, planned_sid) = match &tools.mission {
            Mission::Flight(r) => (r.departure_rwy.clone(), r.sid.clone()),
            _ => (None, None),
        };
        let mismatch = departure_runway_mismatch(planned_rwy.as_deref(), cleared.as_deref());
        let sid_differs = cleared_sid.as_deref().is_some_and(|c| {
            planned_sid
                .as_deref()
                .is_none_or(|p| !p.eq_ignore_ascii_case(c))
        });
        let rwy = cleared.clone().or_else(|| ifr.as_ref().map(|c| c.runway.clone()));
        if mismatch.is_some() || (sid_differs && matches!(tools.mission, Mission::Flight(_))) {
            let why = mismatch.clone().unwrap_or_default();
            let vy = tools.takeoff.vy_kts;
            let only_cleared = mismatch.is_none();
            let replanned = match (&mut tools.mission, rwy.as_deref()) {
                (Mission::Flight(r), Some(c)) => {
                    choose_departure(&config, r, c, vy, cleared_sid.clone(), only_cleared).await
                }
                _ => Err(String::new()),
            };
            // A departure other than the tower's is ASKED for on the radio
            // before it is flown, never flown silently (EGKK 08R, sortie
            // 1790601822: cleared KENE3P, flew LAM5P, #152). Refused or
            // unanswered, the aircraft is handed back here on the ramp.
            let cleared_words =
                cleared_sid.as_deref().map(crate::tower::spoken_sid).unwrap_or_default();
            let mut sid_refused = false;
            let replanned = match replanned {
                Ok(r) => match cleared_sid_plan(r.sid.as_deref(), cleared_sid.as_deref()) {
                    ClearedSidPlan::Fly => Ok(r),
                    ClearedSidPlan::Request(want) => {
                        if request_sid(&callsign, cleared_sid.as_deref(), &want).await {
                            Ok(Rechosen {
                                line: format!(
                                    "The tower gave us the {} departure. {}",
                                    crate::tower::spoken_sid(&want),
                                    r.line
                                ),
                                source: "cleared",
                                sid: r.sid,
                            })
                        } else {
                            sid_refused = true;
                            Err(format!(
                                "The tower cleared us via the {cleared_words} departure, it doesn't join our route, and they didn't give us the {}.",
                                crate::tower::spoken_sid(&want)
                            ))
                        }
                    }
                    ClearedSidPlan::Cannot => {
                        sid_refused = true;
                        Err(format!(
                            "The tower cleared us via the {cleared_words} departure, and no departure from runway {} joins our route.",
                            crate::crew_radio::spell_runway(rwy.as_deref().unwrap_or_default())
                        ))
                    }
                },
                // The planned runway stands and the tower's SID does not fit
                // the route: the plan's own SID is asked for, and flown only
                // once the tower names it.
                Err(no) if only_cleared => {
                    event("cleared_sid_unflyable", json!({ "sid": cleared_sid, "why": no }));
                    let asked = match planned_sid.as_deref() {
                        Some(p) => request_sid(&callsign, cleared_sid.as_deref(), p).await,
                        None => false,
                    };
                    match planned_sid.as_deref() {
                        Some(p) if asked => Ok(Rechosen {
                            line: format!(
                                "The tower gave us the {} departure we planned.",
                                crate::tower::spoken_sid(p)
                            ),
                            source: "cleared",
                            sid: Some(p.to_string()),
                        }),
                        _ => {
                            sid_refused = true;
                            Err(format!(
                                "The tower cleared us via the {cleared_words} departure, it doesn't join our route, and they didn't give us ours."
                            ))
                        }
                    }
                }
                Err(no) => Err(no),
            };
            match replanned {
                Ok(Rechosen { line, source, .. }) => {
                    event(
                        "departure_rechosen",
                        json!({ "line": line, "source": source, "cleared_sid": cleared_sid }),
                    );
                    announce(&voice, Priority::Advisory, &line);
                    // The GPS shows the departure we will fly, not SimBrief's.
                    if let Mission::Flight(r) = &tools.mission {
                        if !r.fms_payload.is_empty() {
                            let _ = xp.set_data("xcopilot/fms/route", r.fms_payload.as_bytes()).await;
                            let _ = xp.set_int("xcopilot/fms/load", 1).await;
                        }
                    }
                }
                // The tower's SID cannot be flown to the route and it gave
                // us no other: handed back on the ramp, the brakes on.
                Err(no) if sid_refused => {
                    event("cleared_sid_not_flown", json!({ "sid": cleared_sid, "why": no }));
                    give_up(
                        &xp,
                        tools.taxi.park_brake_valve_set,
                        &voice,
                        &format!("{no} Your aircraft."),
                        "cleared_sid_not_flown",
                    )
                    .await;
                    return;
                }
                Err(no) => {
                    event("departure_runway_mismatch", json!({ "why": why, "no": no }));
                    give_up(
                        &xp,
                        tools.taxi.park_brake_valve_set,
                        &voice,
                        &format!("{why} {no} Your aircraft."),
                        "departure_runway_mismatch",
                    )
                    .await;
                    return;
                }
            }
        }

        // A running turboprop taxis at the profile's run idle, whatever the
        // lever was left at (a re-arm skips the start that sets it).
        condition_to_run(&xp).await;
        // Drive the cleared route to the holding point. The taxi light, or
        // on an airframe that has none the landing light: the SR22 carries
        // only nav, strobe and landing, and the taxi ran dark because the
        // "taxi" key was declared nowhere (2026-09-12). That is how the real
        // aircraft is taxied at night; it goes off at the stand with the rest.
        let dark = crate::tools::lights::dark_now(&xp).await;
        let (taxi_key, taxi_lit) = match crate::tools::lights::taxi_light(dark) {
            Some(l) => {
                let ok = crate::tools::lights::apply(&xp, &l, true).await.is_ok();
                (l.key.clone(), ok)
            }
            None => ("none".to_string(), false),
        };
        event(
            "lights_taxi",
            json!({ "on": taxi_lit, "light": taxi_key, "dark": dark }),
        );
        // The taxi tool's own reply ("My taxi. Holding point Delta via ...")
        // is the line; "Rolling out to the runway" ahead of it said it twice.
        if let Ok(reply) = tools.taxi.execute(&json!({}), &ctx).await {
            announce(&voice, Priority::Advisory, &reply);
        }
        // Wait for the drive to finish. `driving()`, not ACTIVE: a backtrack
        // drops ACTIVE in the gap between its two legs, and polling the bare
        // flag reads that gap as "finished".
        waiting("the taxi to arrive");
        if crate::autotaxi::driving()
            && !wait_for_drive().await
        {
            give_up(
                &xp,
                tools.taxi.park_brake_valve_set,
                &voice,
                "The taxi didn't finish. Your aircraft.",
                "taxi_stalled",
            )
            .await;
            return;
        }
        bail_if_abort!();
        // ACTIVE clearing is NOT proof of arrival: a pedal yield, an off-route
        // stop, or the governor evaporating all clear it too. A 30-second
        // stand-down (pedals) had full auto call "ready for departure" from
        // the PARKING STAND, wait out a traffic hold, and read back a takeoff
        // clearance 1.8 km from the runway (16:48Z, 2026-07-24). The drive's
        // own verdict is `LAST_ARRIVED` (the backtrack supervisor's signal);
        // when it says no, accept a position within 60 m of the route's end
        // instead (the pilot may have taxied us there by hand) — otherwise
        // stand down honestly.
        if !crate::autotaxi::LAST_ARRIVED.load(SeqCst) {
            // Near the hold means near it ALONG the route and ON its line
            // (`autotaxi::at_the_hold`): a straight 60 m from the route's end
            // also takes an aircraft stopped 27 m off the line beside it, as
            // the LSZS 03 line-up drive did (2026-09-24).
            let cleared = crate::party_line::with_taxi_route(|r| r.and_then(crate::autotaxi::Route::build));
            let near_hold = match cleared {
                Some(route) => {
                    let (Ok(lat), Ok(lon)) = (xp.get_float(LAT).await, xp.get_float(LON).await)
                    else {
                        give_up(
                            &xp,
                            tools.taxi.park_brake_valve_set,
                            &voice,
                            "I can't read our position. Your aircraft.",
                            "no_position",
                        )
                        .await;
                        return;
                    };
                    let fix = crate::autotaxi::route_end_fix(&route, lat, lon);
                    event(
                        "taxi_end_check",
                        json!({ "to_go_m": fix.map(|f| f.0), "lateral_m": fix.map(|f| f.1) }),
                    );
                    fix.is_some_and(|(to_go, lateral)| crate::autotaxi::at_the_hold(to_go, lateral))
                }
                None => false,
            };
            if !near_hold {
                give_up(
                    &xp,
                    tools.taxi.park_brake_valve_set,
                    &voice,
                    "The taxi ended before the holding point, so I'm standing down. \
                     Say fly the flight plan or full auto again and I'll take it from here.",
                    "taxi_incomplete",
                )
                .await;
                return;
            }
        }
        event("at_hold", json!({}));
        // A squawk that came back after the clearance wait gave up is set
        // here, before we talk to the tower again, and the box goes to ALT.
        transponder_before_departure(&xp, &voice, "hold").await;

        // ATC: get the departure clearance BEFORE lining up — the tower gates
        // both the line-up and the takeoff on it. No station running means
        // clearance stays None and the tools don't gate; the calls fall on dead
        // air, harmless. The calls are voiced + transcribed by the main loop's
        // crew-radio drain (with the tower's reply), so we only transmit here.
        let atc_running = crate::party_line::clearance_state().is_some();
        if atc_running {
            request_departure(&callsign, None).await;
            bail_if_abort!();
        }

        // Onto the runway, CLEARANCE-AWARE, and VERIFIED. The line-up tool
        // rightly refuses to enter the runway on a bare taxi clearance — and
        // full_auto used to sail past that refusal string, wait out the
        // traffic hold at the hold-short, read back "cleared for takeoff"
        // from the WRONG SIDE of the line, and arm a circuit that refused:
        // stranded at the hold with a valid clearance (12:44Z, 2026-07-24).
        // Now: try the line-up when the tower's state allows entering
        // (line-up-and-wait or cleared; no tower = pilot's judgment); when
        // the tower is HOLDING us, wait the clearance out FIRST, then drive
        // on; and never proceed to the circuit without the runway_fit check
        // actually confirming we're on the strip.
        use crate::tower::State::{LinedUp, TakeoffCleared};
        let can_enter = || {
            matches!(
                crate::party_line::clearance_state(),
                None | Some(LinedUp) | Some(TakeoffCleared)
            )
        };
        let mut read_back = false;
        for attempt in 0..2 {
            if atc_running && !can_enter() {
                // Held short (traffic, wake): the clearance must come first.
                // Held SHORT: a line-up-and-wait is a good enough answer to
                // get us moving — the clearance is asked for again from the
                // runway, below.
                if ensure_takeoff_cleared(&voice, &callsign, &mut read_back, true, None).await
                    != ClearanceWait::Cleared
                {
                    give_up(
                        &xp,
                        tools.taxi.park_brake_valve_set,
                        &voice,
                        "The tower never cleared us for takeoff. Your aircraft. \
                         Say full auto again once we're cleared.",
                        "no_takeoff_clearance",
                    )
                    .await;
                    return;
                }
                bail_if_abort!();
            }
            // A CONDITIONAL line-up ("behind the landing X, line up behind")
            // is a clearance to enter AFTER X, not now. The Baron at Malaga
            // rolled five seconds after the words with the lander at 2.4 nm
            // and reached the pavement 15 s after it touched down — the
            // margin was the 85 s crawl, not any logic (12:21Z, 2026-09-11).
            // The world publishes the condition every tick from its traffic
            // picture; wait for it to clear, or for the hold to become a
            // release (the condition goes away with it).
            if atc_running && crate::party_line::line_up_condition_clear() == Some(false) {
                announce(
                    &voice,
                    Priority::Advisory,
                    "Behind the landing traffic. Waiting for it to pass.",
                );
                event("line_up_behind_wait", json!({}));
                waiting("the landing traffic to pass");
                if !wait_until(LINE_UP_BEHIND_SECS, 500, || async {
                    crate::party_line::line_up_condition_clear() != Some(false)
                })
                .await
                {
                    give_up(
                        &xp,
                        tools.taxi.park_brake_valve_set,
                        &voice,
                        "The landing traffic never cleared. Holding short. Your aircraft.",
                        "line_up_behind_timeout",
                    )
                    .await;
                    return;
                }
                bail_if_abort!();
                event("line_up_behind_clear", json!({}));
            }
            // Once: a backtrack is two executions (the drive to the far end,
            // then the turn onto the line), and the tool's own reply names
            // each leg. "Taking the runway" twice in ninety seconds, LXGB 27,
            // 2026-09-15.
            // The line-up tool's own reply names the runway; a "Taking the
            // runway" ahead of it said it twice.
            let gen0 = crate::autotaxi::drive_gen();
            if let Ok(reply) = tools.line_up.execute(&json!({}), &ctx).await {
                announce(&voice, Priority::Advisory, &reply);
            }
            // A full-length backtrack is a long, slow drive (LXGB's 09 backtrack
            // is ~1 km) — the 120 s that fit a plain line-up timed out mid-taxi
            // (2026-07-26). Give it the taxi's budget.
            // `driving()` covers the gap between a backtrack's two legs —
            // ACTIVE alone drops there, and full auto called a running line-up
            // a failure one second after leg two engaged (2026-07-28).
            waiting("the taxi to arrive");
            if crate::autotaxi::driving()
                && !wait_for_drive().await
            {
                give_up(
                    &xp,
                    tools.taxi.park_brake_valve_set,
                    &voice,
                    "The line-up didn't finish. Your aircraft.",
                    "line_up_stalled",
                )
                .await;
                return;
            }
            bail_if_abort!();
            // The drive's own verdict and where it left us, not the fit alone
            // ([`judge_lined_up`]): a failed line-up never rolls. Once more
            // from where it stopped (on the strip: a settle onto the line,
            // `autotaxi::on_strip_settle_path`), then handed back parked.
            let drove = crate::autotaxi::drive_gen() != gen0;
            let why = match lined_up_check(&xp, &tools, drove).await {
                Ok(Ok(())) => break,
                Ok(Err(miss)) => {
                    let words = line_up_miss_words(&miss);
                    event(
                        "line_up_missed",
                        json!({ "attempt": attempt, "miss": format!("{miss:?}") }),
                    );
                    if attempt == 1 {
                        give_up(
                            &xp,
                            tools.taxi.park_brake_valve_set,
                            &voice,
                            &format!("{words} I couldn't line us up. Your aircraft."),
                            "line_up_failed",
                        )
                        .await;
                        return;
                    }
                    announce(&voice, Priority::Caution, &format!("{words} Lining up again."));
                    continue;
                }
                Err(why) => why,
            };
            // A runway that is too short stays too short: no second line-up.
            // ...and neither does a runway with too little of it AHEAD: the
            // second attempt is a forward line-up, and at LILB 03
            // (2026-09-25) it drove 110 m further down from 508 m left to
            // 398.
            let hopeless = matches!(
                why,
                Some(
                    crate::runway::NoFit::RunwayTooShort { .. }
                        | crate::runway::NoFit::TooLittleLeft { .. }
                )
            );
            if attempt == 1 || hopeless {
                // Say which gate failed. "I couldn't get us onto the runway"
                // was said to an A330 on Edinburgh 24's centreline, aligned
                // within a degree, the tower logging `lined_up=true`: the fit
                // had failed on LENGTH (2026-09-20).
                event("line_up_no_fit", json!({ "why": format!("{why:?}") }));
                // ...and `NoneNear` is a gate like the rest when it knows
                // which strip it looked for: at Le Bourget that was the runway
                // the tower had cleared, sitting 169 m away, and the vague
                // line was all the pilot got (2026-09-22).
                use crate::runway::NoFit;
                let named = match &why {
                    Some(NoFit::NoneNear { id, .. }) => !id.is_empty(),
                    Some(
                        NoFit::RunwayTooShort { .. }
                        | NoFit::TooLittleLeft { .. }
                        | NoFit::NotAligned { .. },
                    ) => true,
                    _ => false,
                };
                // Only the MARGIN short: the pilot's to waive, as at the arm.
                // The arm measures the whole runway and the line-up only the
                // runway ahead, so this is where a short field is found.
                let waivable = match (&why, &tools.mission) {
                    (
                        Some(NoFit::TooLittleLeft { remaining_m, needed_m, .. }),
                        Mission::Flight(r) | Mission::Resume(r),
                    ) => margin_only_short(
                        *remaining_m,
                        *needed_m,
                        crate::aircraft::takeoff_margin_waived(),
                    )
                    .map(|bare| (bare, r.origin_icao.clone(), r.dest_icao.clone())),
                    _ => None,
                };
                let line = match (&why, named) {
                    (Some(w), true) => match &waivable {
                        Some((bare, origin, dest)) => {
                            note_refusal_for(Gate::OriginRunway, origin, dest);
                            format!(
                                "{} Without the margin we need {:.0}. If you've checked it yourself, say fly it anyway.",
                                crate::autotakeoff::no_fit_line(w, false).0,
                                bare
                            )
                        }
                        None => format!(
                            "{} Your aircraft.",
                            crate::autotakeoff::no_fit_line(w, false).0
                        ),
                    },
                    _ => "I couldn't get us onto the runway. Your aircraft.".to_string(),
                };
                give_up(
                    &xp,
                    tools.taxi.park_brake_valve_set,
                    &voice,
                    &line,
                    "line_up_failed",
                )
                .await;
                return;
            }
        }
        event("lined_up", json!({}));
        // A SID with minimums: the takeoff's own climb-out at best rate.
        DEPARTURE_MINIMUMS.store(
            matches!(&tools.mission, Mission::Flight(r) if !r.climb_minimums.is_empty()),
            SeqCst,
        );
        ensure_squawk(&xp, &ctx, &voice).await;
        // On the runway: strobes and landing lights. These are the two that
        // say "this aeroplane is about to move fast", which is why they go on
        // here and not at the holding point.
        let strobe = light(&tools, &ctx, "strobe", true).await;
        let landing = light(&tools, &ctx, "landing", true).await;
        event(
            "lights_takeoff",
            json!({ "strobe": strobe, "landing": landing }),
        );

        // The takeoff clearance, if the line-up went on under line-up-and-wait
        // (already satisfied when the held-short path above got it first).
        if atc_running
            && ensure_takeoff_cleared(&voice, &callsign, &mut read_back, false, None).await
                != ClearanceWait::Cleared
        {
            give_up(
                &xp,
                tools.taxi.park_brake_valve_set,
                &voice,
                "We're lined up and ready, but the tower never cleared us for takeoff. \
                 Your aircraft. Say full auto again once we're cleared.",
                "no_takeoff_clearance",
            )
            .await;
            return;
        }
        bail_if_abort!();
    } else {
        // A RUNWAY START: the sim put us on the numbers and nobody taxied.
        // "Skip straight to the circuit" used to mean skipping the tower too:
        // no call, no clearance, no strobes, and the A330 flew Glasgow's zone
        // to "you are inside controlled airspace without clearance" (EGPF,
        // 2026-09-21). The tower now takes a "ready" from the strip as a
        // lined-up aircraft (`Ctx::own_on_runway`), so the same call and the
        // same wait as the taxied departure apply, made above: only a
        // clearance for THIS runway gets here (`cleared_elsewhere`, #178).
        // A SID with minimums: the takeoff's own climb-out at best rate, as
        // from a taxied line-up.
        DEPARTURE_MINIMUMS.store(
            matches!(&tools.mission, Mission::Flight(r) if !r.climb_minimums.is_empty()),
            SeqCst,
        );
        let strobe = light(&tools, &ctx, "strobe", true).await;
        let landing = light(&tools, &ctx, "landing", true).await;
        event(
            "lights_takeoff",
            json!({ "strobe": strobe, "landing": landing, "runway_start": true }),
        );
        if start_cleared {
            ensure_squawk(&xp, &ctx, &voice).await;
            bail_if_abort!();
        }
    }

    // --- Stage 4: the mission — a circuit, or the SimBrief flight -------------
    match &tools.mission {
        Mission::Circuit { right } => {
            announce(
                &voice,
                Priority::Advisory,
                crate::readback::one_of(&[
                    "Lined up. Flying the circuit.",
                    "Lined up and ready. Flying the circuit.",
                    "On the runway. Off we go, flying the circuit.",
                ]),
            );
            let args = json!({ "right": right });
            if let Ok(reply) = tools.circuit.execute(&args, &ctx).await {
                announce(&voice, Priority::Advisory, &reply);
            }
            // A refusal comes back Ok with the reason as the reply, so the
            // reply proves nothing — the FLAG is the arm, exactly as
            // fly_route checks the takeoff's. Without this, a refused
            // circuit fell through to a 900 s wait for an autoland that was
            // never coming, with every re-arm answered "I'm already running
            // the full sequence" (audit, 2026-08-02). The refusal line has
            // already been spoken; this only stands the sequence down.
            if !crate::circuit::ACTIVE.load(SeqCst) {
                give_up(
                    &xp,
                    tools.taxi.park_brake_valve_set,
                    &voice,
                    "The circuit didn't arm, so I'm standing down. Your aircraft.",
                    "circuit_refused",
                )
                .await;
                return;
            }
            event("circuit_armed", json!({}));
            drop_departure_handoff("circuit_armed");
            // The circuit owns the FLYING from here, but not the last stage:
            // it hands the final to autoland, which stops on the runway and
            // lets go. Somebody has to vacate. See `taxi_in_after_landing` —
            // ninety seconds holding Southampton's 02 with an arrival at half
            // a mile is what "our job is done" used to mean here.
            taxi_in_after_landing(&xp, &config, &voice, &tools, true).await;
            drop_departure_handoff("circuit_ended");
            // A circuit whose takeoff never completed ends the MISSION as the
            // takeoff's give-up, the runner's `expect_give_up` included, not
            // a silent "ended" (LEMG 13, 2026-09-28: `no_autoland`).
            if circuit_takeoff_failed(crate::circuit::stood_down(), ABORT.load(SeqCst)) {
                // A recovery holding the aircraft is never "your aircraft"
                // (#181: the autopilot with no pitch).
                let line = if crate::tools::recover::ACTIVE.load(SeqCst) {
                    "The flight is cancelled. I have the aircraft until you say my controls."
                } else {
                    "The flight is cancelled. Your aircraft."
                };
                let reason = match crate::circuit::stood_down() {
                    Some(crate::circuit::TERRAIN_REFUSED) => crate::circuit::TERRAIN_REFUSED,
                    _ => crate::circuit::TAKEOFF_NOT_COMPLETED,
                };
                give_up(&xp, tools.taxi.park_brake_valve_set, &voice, line, reason).await;
                return;
            }
            stop_active();
        }
        Mission::Flight(route) | Mission::Resume(route) => {
            fly_route(&xp, &config, &voice, &ctx, &tools, route, &callsign).await;
        }
    }
}
