//! The world's ticks the crew's monitor drives (P2.6). The station is the
//! world, the monitor its scheduler: every tick `monitor::run` calls these at
//! the points it always has, in the same order, under the same gates. Moved
//! whole from `monitor::run`; the crew work interleaved between them (the
//! speech arbiter's COM1 and power, the gyro sync, the squawk answer, the
//! coaches, the taxi guide) stays in the monitor, which is why there are
//! five calls and not one.
//!
//! [`WorldTick::runway_watch`] runs AHEAD of the monitor's `quiet` block
//! (which closes under `autoland::ACTIVE`): the runway watch ticks whoever
//! flies the final. The rest run inside it, the unheard vectors in its
//! `else`.

use std::time::Instant;

use crate::monitor::{r1, region_note_said, Snapshot, WorldGates};
use crate::plugin_status::StatusSink;
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

/// What one monitor tick lends the world's ticks.
#[derive(Clone, Copy)]
pub(crate) struct Tick<'a> {
    pub(crate) xp: &'a XPlaneClient,
    pub(crate) voice: &'a Voice,
    pub(crate) status: &'a Option<StatusSink>,
    pub(crate) snap: &'a Snapshot,
    pub(crate) now: Instant,
}

/// What the runway watch read and decided this tick, for the ticks after it.
pub(crate) struct Heard {
    /// One COM1 read for the compliance watch, the incursion shout, the ATIS
    /// loop and the traffic advisories (None when the watch does not tick).
    pub(crate) com1: Option<i64>,
    /// The controller the radio actually hears.
    pub(crate) tuned_speaker: Option<crate::voice::StationSpeaker>,
    pub(crate) landing_held: bool,
    pub(crate) cleared_to_land: bool,
    /// The runway watch's TCAS read, shared with the traffic watches.
    pub(crate) targets: Option<Vec<crate::world::TcasTarget>>,
}

/// The cadences and latches the world's ticks keep between monitor ticks.
#[derive(Default)]
pub(crate) struct WorldTick {
    /// Approach traffic advisories while being vectored, rate-limited: a real
    /// controller calls a factor once and moves on, not every sweep.
    traffic_last: Option<Instant>,
    /// Near-miss cadence: repeats every 15 s while the conflict persists,
    /// independent of the advisory clock above; a safety alert must not wait
    /// out a 45 s narration window.
    near_miss_last: Option<Instant>,
    /// Party-line cadence.
    chatter_last: Option<Instant>,
    /// The last ATIS edition logged to the panel ("London Heathrow information
    /// Alpha"): the loop speaks every repeat, the transcript records once.
    atis_logged: Option<String>,
    /// Whether an advisory is waiting on its "clear of traffic" closure.
    traffic_open: bool,
    traffic_called: Option<crate::world::TrafficFactor>,
    /// Runway-incursion shout cadence: it repeats while the conflict persists
    /// (a safety shout may nag), but not every tick.
    incursion_last: Option<Instant>,
    /// Our own traffic (stage D, `XCOPILOT_TRAFFIC=ours`): the fleet at the
    /// installed field, ticked by [`WorldTick::traffic_link`].
    traffic: crate::traffic_link::TrafficWorld,
}

impl WorldTick {
    /// The compliance watch and the landing's runway watch, ahead of `quiet`.
    pub(crate) async fn runway_watch(&mut self, t: &Tick<'_>, gates: WorldGates) -> Heard {
        let Tick {
            xp,
            voice,
            status,
            snap,
            now,
        } = *t;
        // One COM1 read serves the compliance watch, the incursion
        // shout, the ATIS loop and the traffic advisories below, read
        // whenever the compliance watch ticks (a wreck reads nothing).
        let com1: Option<i64> = if gates.compliance_ticks {
            xp.get_float(crate::datarefs::COM1_FREQ_KHZ)
                .await
                .ok()
                .map(|v| v.round() as i64)
        } else {
            None
        };
        // Which CONTROLLER the radio actually hears, resolved once per
        // tick: every controller call below speaks in THIS voice (or
        // falls to the crew when None). A berate hardcoded to the
        // Tower while the pilot sat on the approach frequency put two
        // controllers on one radio (2026-07-12).
        let tuned_speaker = com1.and_then(crate::party_line::tuned_controller);
        // Compliance watch: the tower holds the pilot to the clearances
        // it actually issued (uncleared runway entry, takeoff, landing),
        // and challenges zone traffic it has never talked to. Tuned to
        // the station the controller speaks; otherwise the copilot flags
        // it as crew: a controller can't berate a radio that isn't
        // listening.
        //
        // NOT under `quiet`: that gate includes `autoland::ACTIVE`, and
        // the landing clearance is exactly what this watch guards. Gated
        // with the chatter it was never ticked from the autoland arm to
        // the rollout handoff, so its air/ground edge went stale and an
        // uncleared landing was called on the first tick after the
        // handoff, 26.6 s after the touchdown, at a stopped aircraft
        // (2026-09-26, issue #63). It TICKS on every live tick but a
        // wreck, so its edges are always current, and SPEAKS unless a
        // recovery, the panel's mute or a wreck silences the world (the
        // cabin's gate). A muted call is consumed, never said late.
        if gates.compliance_ticks {
            if let Some(call) = crate::world::compliance_tick_facing(
                snap.lat,
                snap.lon,
                snap.alt_ft,
                crate::hover::tower_on_ground(snap.on_ground, snap.agl_ft),
                snap.groundspeed_kts,
                tuned_speaker.is_some(),
                now,
                Some(snap.heading_deg).filter(|h| *h != 0.0),
            ) {
                crate::blackbox::record(
                    "atc",
                    serde_json::json!({
                        "route": "compliance", "crew": call.crew, "reply": &call.line,
                        "autoland": gates.autoland, "said": gates.compliance_speaks,
                    }),
                );
                if !gates.compliance_speaks {
                    // A recovery, the panel's mute: consumed, unsaid.
                } else if call.crew {
                    voice.alert(Priority::Caution, call.line);
                } else {
                    // The controller you're actually TUNED to delivers
                    // it (approach relays a deviation and the phone
                    // number exactly like the real thing), never the
                    // tower's voice on someone else's frequency.
                    let who = tuned_speaker.unwrap_or(crate::voice::StationSpeaker::Tower);
                    if let Some(s) = status.as_ref() {
                        s.action(&format!("{}: {}", who.tag(), call.line));
                    }
                    voice.alert_station(Priority::Warning, who, call.line);
                }
            }
        }
        // The landing's runway watch: the go-around order (a ghost on the
        // runway that will still be there when we arrive) and the release
        // of a withheld landing clearance. NOT under `quiet`, for the
        // compliance watch's reason: `quiet` includes `autoland::ACTIVE`,
        // and an autoland final is exactly the approach this watch guards.
        // Gated with the chatter, "continue approach, expect late landing
        // clearance" at 7 nm was never released and never judged into a
        // go-around, and autoland landed uncleared (EGKK-EGBB 15, sortie
        // 1790601822-full_auto, 2026-09-28, issue #155). Ticks on every
        // live tick but a wreck; speaks unless a recovery, the panel's
        // mute or a wreck silences the world (a muted call is consumed).
        // The TCAS read is shared with the traffic watches below.
        let landing_held = crate::party_line::landing_hold_armed();
        let cleared_to_land =
            crate::party_line::clearance_state() == Some(crate::tower::State::ClearedToLand);
        let mut runway_watch_targets: Option<Vec<crate::world::TcasTarget>> = None;
        if gates.compliance_ticks
            && (landing_held || cleared_to_land)
            && crate::party_line::active_field().is_some()
        {
            let targets = crate::world::targets(xp.view()).await;
            let tower_tuned = matches!(tuned_speaker, Some(crate::voice::StationSpeaker::Tower));
            // Safety outranks flavor: the go-around order first.
            if let Some(call) = crate::world::go_around_tick(
                &targets,
                snap.agl_ft,
                snap.on_ground,
                tuned_speaker.is_some(),
                now,
            ) {
                crate::blackbox::record(
                    "atc",
                    serde_json::json!({
                        "route": "go_around_order", "crew": call.crew,
                        "reply": &call.line, "autoland": gates.autoland,
                        "said": gates.compliance_speaks,
                    }),
                );
                if !gates.compliance_speaks {
                    // A recovery, the panel's mute: consumed, unsaid.
                } else if call.crew {
                    voice.alert(Priority::Warning, call.line);
                } else {
                    // Whoever you're tuned to gives the order: a
                    // go-around shout is any controller's to deliver,
                    // in their own voice.
                    let who = tuned_speaker.unwrap_or(crate::voice::StationSpeaker::Tower);
                    if let Some(s) = status.as_ref() {
                        s.action(&format!("{}: {}", who.tag(), call.line));
                    }
                    voice.alert_station(Priority::Warning, who, call.line);
                }
            } else if landing_held {
                if let Some(line) = crate::world::landing_hold_tick(&targets, snap.on_ground, now)
                {
                    crate::blackbox::record(
                        "atc",
                        serde_json::json!({
                            "route": "landing_release", "reply": &line,
                            "autoland": gates.autoland, "said": gates.compliance_speaks,
                        }),
                    );
                    // The tower's clearance on the tower frequency only,
                    // like the takeoff release.
                    if tower_tuned && gates.compliance_speaks {
                        if let Some(s) = status.as_ref() {
                            s.action(&format!("TWR: {line}"));
                        }
                        voice.alert_station(
                            Priority::Advisory,
                            crate::voice::StationSpeaker::Tower,
                            line,
                        );
                    }
                }
            }
            runway_watch_targets = Some(targets);
        }
        Heard {
            com1,
            tuned_speaker,
            landing_held,
            cleared_to_land,
            targets: runway_watch_targets,
        }
    }

    /// Runway incursion, inside `!quiet`.
    pub(crate) async fn incursion(&mut self, t: &Tick<'_>, heard: &Heard) {
        let Tick {
            xp,
            voice,
            status,
            snap,
            now,
        } = *t;
        let tuned_speaker = heard.tuned_speaker;
        // Runway incursion: sitting (or slow-taxiing) ON a runway with
        // live traffic in THAT strip's final corridor — the one
        // conflict geometry where seconds matter. Repeats every 15 s
        // while it persists; escalates inside 1.5 nm. The tower shouts
        // when the pilot is on its frequency, the copilot calls it as
        // crew otherwise — a safety net must not depend on the radio
        // being tuned.
        // Rolling above ~30 kt is a departing aircraft, not an
        // incursion. LiveTraffic / AI aircraft feed the TCAS picture.
        // ...unless the tower ITSELF put us there: a standing takeoff
        // clearance or "line up and wait" means the runway is ours,
        // and "exit the runway, expedite" 42 seconds after "cleared
        // for takeoff" is the tower contradicting its own clearance
        // (Malaga runway 12, 2026-07-12). A genuine conflict against
        // a cleared departure is the release/cancel machinery's
        // problem, never this shout.
        // So too the LANDING it cleared, until the vacated report, and
        // a runway start or a moving cleared crossing: EGKK 26L
        // (2026-09-27) heard "exit the runway immediately" from the
        // touchdown to the vacate, eight times, cleared to land.
        let runway_is_ours = crate::party_line::runway_occupancy_authorised(
            snap.lat,
            snap.lon,
            snap.groundspeed_kts,
        );
        if snap.on_ground && snap.groundspeed_kts < 30.0 && !runway_is_ours {
            // Named by the end we point down, as the compliance
            // watch names it: rolling out on 15 past midfield is
            // not "runway three three" (EGKK-EGBB, #155).
            if let Some(rwy) = crate::world::on_active_runway_facing(
                snap.lat,
                snap.lon,
                Some(snap.heading_deg).filter(|h| *h != 0.0),
            ) {
                let due = self
                    .incursion_last
                    .is_none_or(|t| now.duration_since(t).as_secs() >= 15);
                if due {
                    if let Some(nm) =
                        crate::world::inbound_traffic_for_strip(xp.view(), &rwy).await
                    {
                        self.incursion_last = Some(now);
                        let urgent = nm < 1.5;
                        crate::blackbox::record(
                            "runway_incursion",
                            serde_json::json!({ "runway": &rwy, "traffic_nm": r1(nm) }),
                        );
                        if let Some(who) = tuned_speaker {
                            // Addressed by callsign, like every other
                            // controller call — an unaddressed "exit
                            // the runway" reads as talking to nobody
                            // (pilot feedback, 2026-07-10).
                            let callsign = crate::crew_radio::crew_callsign(&xp)
                                .await
                                .unwrap_or_else(|| "aircraft on the runway".to_string());
                            let line = if urgent {
                                format!(
                                    "{callsign}, traffic on short final, exit the runway immediately."
                                )
                            } else {
                                format!(
                                    "{callsign}, traffic on {nm:.0} mile final, exit the runway, expedite."
                                )
                            };
                            if let Some(s) = status.as_ref() {
                                s.action(&format!("{}: {line}", who.tag()));
                            }
                            voice.alert_station(Priority::Warning, who, line);
                        } else {
                            let line = if urgent {
                                "Traffic short final and we're on the runway. Clear the runway NOW."
                                    .to_string()
                            } else {
                                format!(
                                    "Traffic on {nm:.0} mile final and we're still on the runway. Expedite."
                                )
                            };
                            voice.alert(Priority::Warning, line);
                        }
                    }
                }
            }
        }
    }

    /// The ATIS loop, the after-takeoff handoff and the radar watch, inside `!quiet`.
    pub(crate) async fn radio_services(&mut self, t: &Tick<'_>, heard: &Heard) {
        let Tick {
            xp,
            voice,
            status,
            snap,
            now,
        } = *t;
        let (com1, tuned_speaker) = (heard.com1, heard.tuned_speaker);
        // The ATIS loop: while COM1 sits on the ATIS frequency the
        // broadcast repeats, cutting a new information letter when the
        // weather genuinely moves. Cheap gate first — tuned AND due —
        // so the weather set is read once per broadcast, not per tick.
        if let Some(com1) = com1 {
            // The ATC tab tunes ANY in-range field's frequencies:
            // follow COM1 to that field's ATIS instead of staying
            // silent because it wasn't the active station's.
            crate::world::atis_retune(com1, snap.lat, snap.lon, snap.alt_ft);
            if crate::world::atis_due(com1, std::time::Instant::now()) {
                let wx = crate::world::AtisWx {
                    zulu_sec: xp
                        .get_float(crate::datarefs::ZULU_TIME_SEC)
                        .await
                        .unwrap_or(0.0),
                    wind_from_deg: xp
                        .get_float(crate::datarefs::WIND_DIR_DEG_TRUE)
                        .await
                        .unwrap_or(0.0),
                    wind_kts: xp
                        .get_float(crate::datarefs::WIND_SPEED_MSC)
                        .await
                        .unwrap_or(0.0)
                        * 1.943844,
                    visibility_sm: xp
                        .get_float(crate::datarefs::VISIBILITY_SM)
                        .await
                        .unwrap_or(10.0),
                    qnh_hpa: xp
                        .get_float(crate::datarefs::QNH_PASCALS)
                        .await
                        .unwrap_or(101300.0)
                        / 100.0,
                    oat_c: xp
                        .get_float(crate::datarefs::OAT_DEG_C)
                        .await
                        .unwrap_or(15.0),
                    // The FAA broadcast's dew point: the air at the
                    // aircraft, as the temperature is; a failed read says
                    // nothing (the snapshot's -30 is an unread default,
                    // `monitor`, never a measurement to broadcast).
                    dewpoint_c: {
                        let (dew, levels) = tokio::join!(
                            xp.get_float_array(crate::tools::anti_ice::DEWPOINT_LEVELS),
                            xp.get_float_array(crate::tools::anti_ice::ATMOSPHERE_LEVELS_M),
                        );
                        match (dew, levels) {
                            (Ok(d), Ok(l)) => crate::tools::anti_ice::dewpoint_at(
                                &l,
                                &d,
                                snap.alt_ft / 3.28084,
                            ),
                            _ => None,
                        }
                    },
                };
                if let Some(report) =
                    crate::world::atis_tick(com1, &wx, std::time::Instant::now())
                {
                    // The broadcast LOOPS on the radio, but the log
                    // is a transcript: one entry per edition (field
                    // or letter change), not one per repetition —
                    // the loop was flooding the panel (2026-07-10).
                    let edition = report.split('.').next().unwrap_or("").to_string();
                    if self.atis_logged.as_deref() != Some(edition.as_str()) {
                        self.atis_logged = Some(edition);
                        if let Some(s) = status.as_ref() {
                            s.action(&format!("ATIS: {report}"));
                        }
                        crate::blackbox::record("atis", serde_json::json!({ "msg": &report }));
                    }
                    voice.alert_station(
                        Priority::Advisory,
                        crate::voice::StationSpeaker::Atis,
                        report,
                    );
                }
            }
        }
        // After-takeoff handoff: armed by the tower's takeoff
        // clearance, spoken once airborne and climbing ("contact
        // Gibraltar Departure on one three zero decimal four"). It is
        // the TOWER talking on the tower frequency: tuned anywhere
        // else it goes into the void — missed radio is gone, never
        // replayed on another frequency (two controllers on one
        // radio, 2026-07-12).
        if let Some(line) = crate::world::handoff_tick(snap.alt_ft, snap.on_ground) {
            crate::blackbox::record(
                "atc_handoff",
                serde_json::json!({ "alt": r1(snap.alt_ft), "msg": &line }),
            );
            if matches!(tuned_speaker, Some(crate::voice::StationSpeaker::Tower)) {
                if let Some(s) = status.as_ref() {
                    s.action(&format!("TWR: {line}"));
                }
                voice.alert_station(
                    Priority::Advisory,
                    crate::voice::StationSpeaker::Tower,
                    line,
                );
            }
        }
        // Ground's handoff at the holding point ("hold short runway two
        // six left, contact Gatwick Tower on ..."): GROUND talking, heard
        // only on Ground's frequency; the crew that works the radio takes
        // it from the party line either way (`take_position_handoff`).
        if let Some(line) = crate::world::ground_handoff_tick(snap.lat, snap.lon, snap.on_ground) {
            if matches!(tuned_speaker, Some(crate::voice::StationSpeaker::Ground)) {
                if let Some(s) = status.as_ref() {
                    s.action(&format!("GND: {line}"));
                }
                voice.alert_station(
                    Priority::Advisory,
                    crate::voice::StationSpeaker::Ground,
                    line,
                );
            }
        }
        // Radar watch on an assigned squawk: "radar contact" once the
        // sim shows the code replying, "confirm squawk" nudges while
        // it doesn't. Transponder reads gated on an armed watch, so
        // flights without a radar service cost nothing. AIRBORNE
        // only: a departure code is set on the ground with the
        // transponder correctly in standby — nudging there is wrong,
        // and "radar contact" belongs after liftoff.
        // The watch belongs to the ASSIGNER (approach), but radio
        // physics belong to COM1: nothing is heard off frequency,
        // and on someone else's frequency the message arrives in
        // THAT controller's voice — a tower relaying radar's
        // "confirm squawk", exactly like the real thing (approach
        // nagged the pilot ACROSS the tower frequency, 2026-07-12).
        // Untuned, the watch just waits; the transponder reads cost
        // nothing while nobody could speak anyway.
        let squawk_ears = com1.and_then(crate::party_line::tuned_controller);
        if !snap.on_ground && crate::world::squawk_watch_active() {
            if let Some(speaker) = squawk_ears {
                let code = xp
                    .get_float(crate::datarefs::TRANSPONDER_CODE)
                    .await
                    .ok()
                    .map(|v| v.round() as i64);
                let replying = xp
                    .get_float(crate::datarefs::TRANSPONDER_MODE)
                    .await
                    .is_ok_and(|v| v >= 2.0);
                if let Some((line, _assigner)) =
                    crate::world::squawk_tick(code, replying, speaker, now)
                {
                    if let Some(s) = status.as_ref() {
                        s.action(&format!("{}: {line}", speaker.tag()));
                    }
                    crate::blackbox::record(
                        "atc",
                        serde_json::json!({
                            "route": "squawk_watch", "code": code, "replying": replying,
                            "reply": &line,
                        }),
                    );
                    voice.alert_station(Priority::Advisory, speaker, line);
                }
            }
        }
    }

    /// The live-traffic watches, the region switch, the vectors, the near
    /// miss and the traffic advisories, inside `!quiet`. `taxi_guide_idle`
    /// is the crew's taxi guide as this tick left it.
    pub(crate) async fn traffic(
        &mut self,
        t: &Tick<'_>,
        heard: &mut Heard,
        atc_chatter: bool,
        taxi_guide_idle: bool,
    ) {
        let Tick {
            xp,
            voice,
            status,
            snap,
            now,
        } = *t;
        let (com1, tuned_speaker) = (heard.com1, heard.tuned_speaker);
        let (landing_held, cleared_to_land) = (heard.landing_held, heard.cleared_to_land);
        // Live-traffic watches off one TCAS target read: the
        // go-around order (cancel the landing clearance a ghost on
        // the runway is about to contradict), the held takeoff
        // clearance release, and the party line (clearances narrated
        // for the ghosts themselves — LiveTraffic replays real
        // flights, so the tower "clears" what reality is about to do
        // anyway). Reads gated: nothing armed and chatter off means
        // no dataref traffic.
        {
            let tower_tuned = matches!(tuned_speaker, Some(crate::voice::StationSpeaker::Tower));
            // A held release is spent once we ROLL on the runway: the
            // tower saw the takeoff it was holding. Left armed it
            // outlived a reject and replayed "traffic clear, cleared
            // for takeoff" into the next mission (EGKK 2026-09-27).
            if snap.on_ground && crate::world::void_takeoff_hold_by_roll(snap.groundspeed_kts) {
                crate::blackbox::record(
                    "atc",
                    serde_json::json!({
                        "event": "hold_void", "why": "rolled",
                        "gs_kts": snap.groundspeed_kts,
                    }),
                );
            }
            let hold_armed = crate::party_line::takeoff_hold_armed();
            // A landing clearance withheld for traffic on the runway
            // ("continue approach") or given: the runway watch above
            // judged it this tick; the flavour below waits for it.
            // Party-line flavor stands down while the taxi guide is
            // working AND the aircraft is rolling: its cues are
            // perishable, and a three-line Ryanair landing exchange
            // cost the pilot the G11 turn (cue spoken 17 s late,
            // 2026-07-13). But guide-active is NOT rolling — the
            // guide rides the whole taxi clearance, and gating on it
            // alone killed the party line for a 20-minute Gatwick
            // taxi, most of it stationary (same day). Stopped, no cue
            // is imminent; chatter is welcome. Calls addressed to US
            // (compliance, hold releases) are unaffected either way.
            // Our own traffic speaks for itself: under `ours` the TCAS slots
            // hold OUR aircraft, and narrating them as ghosts said every
            // arrival twice, on the station's runway 12 while it flew 13
            // (LEMG, 2026-10-08). The ghost party line is `real` traffic's.
            let chatter_on = atc_chatter
                && tower_tuned
                && (taxi_guide_idle || snap.groundspeed_kts < 3.0)
                && crate::traffic_link::TrafficMode::now() != crate::traffic_link::TrafficMode::Ours;
            // The wake watch needs the TCAS picture while the pilot
            // is on the tower frequency on the ground — a heavy
            // rotating ahead of you starts a clock whether or not
            // the party line is narrating it.
            let wake_watch = tower_tuned && snap.on_ground;
            if let Some((_, flat, flon, felev)) = crate::party_line::active_field() {
                if chatter_on || hold_armed || landing_held || cleared_to_land || wake_watch {
                    // One read per tick: the runway watch's, when it
                    // made one.
                    let targets = match heard.targets.take() {
                        Some(t) => t,
                        None => crate::world::targets(xp.view()).await,
                    };
                    crate::world::wake_note_targets(&targets, now);
                    // A withheld landing clearance is the runway
                    // watch's (above, whoever flies); the flavour
                    // waits for it.
                    if landing_held {
                    } else if hold_armed {
                        if let Some(line) = crate::world::takeoff_hold_tick_from(
                            &targets,
                            felev,
                            snap.on_ground,
                            Some((snap.lat, snap.lon)),
                            now,
                        ) {
                            crate::blackbox::record(
                                "atc",
                                serde_json::json!({
                                    "route": "hold_release", "reply": &line,
                                }),
                            );
                            // The tower's clearance on the tower
                            // frequency: tuned elsewhere it's said
                            // into the void (ask again and the tower
                            // re-states it), never replayed on another
                            // controller's frequency.
                            if tower_tuned {
                                if let Some(s) = status.as_ref() {
                                    s.action(&format!("TWR: {line}"));
                                }
                                voice.alert_station(
                                    Priority::Advisory,
                                    crate::voice::StationSpeaker::Tower,
                                    line,
                                );
                            }
                        }
                    } else if chatter_on
                        && self
                            .chatter_last
                            .is_none_or(|t| now.duration_since(t).as_secs() >= 45)
                    {
                        let fname = crate::party_line::active_field()
                            .map(|(n, _, _, _)| n)
                            .unwrap_or_default();
                        // The strip the PLAYER holds (stands on, or
                        // landed on and not yet reported vacated): a
                        // ghost landing it gets "continue", not a
                        // narrated clearance over the player's head.
                        let blocked = crate::world::player_blocked_strip(
                            snap.on_ground,
                            snap.lat,
                            snap.lon,
                        );
                        if let Some(exchange) = crate::world::chatter_tick(
                            &targets,
                            &fname,
                            flat,
                            flon,
                            felev,
                            blocked.as_deref(),
                            now,
                        ) {
                            self.chatter_last = Some(now);
                            let transcript: Vec<&str> =
                                exchange.lines.iter().map(|l| l.text.as_str()).collect();
                            crate::blackbox::record(
                                "atc",
                                serde_json::json!({
                                    "route": "party_line",
                                    "reply": transcript.join(" / "),
                                }),
                            );
                            // Pilot and tower lines queue in order
                            // through the one arbiter — the exchange
                            // plays as a dialogue, two voices. The
                            // '~' marks AMBIENCE for the panel: an
                            // overheard exchange, not a call to this
                            // aircraft — the log dims it so calls
                            // with our name on them stay bright.
                            for l in exchange.lines {
                                let who = if l.ghost {
                                    crate::voice::StationSpeaker::Ghost {
                                        seed: exchange.ghost_seed,
                                    }
                                } else {
                                    crate::voice::StationSpeaker::Tower
                                };
                                if let Some(s) = status.as_ref() {
                                    s.action(&format!("~{}: {}", who.tag(), l.text));
                                }
                                voice.alert_station_overheard(Priority::Advisory, who, l.text);
                            }
                        }
                    }
                }
            }
        }
        // Region switch: airborne it's a controller handoff; on the
        // ground (loaded the flight at a new field) it's the copilot
        // noting the local tower. Either way the main loop rebuilds
        // the station around the new airport. Gated by mode — the
        // region index survives an ATC-off toggle (cheap re-enable),
        // so this is the one world tick that would otherwise keep
        // running against a torn-down station.
        if let Some(sw) = (crate::mode::atc_enabled())
            .then(|| crate::world::region_tick(snap.lat, snap.lon, snap.on_ground, now))
            .flatten()
        {
            let abnormal = crate::flight::published()
                .is_some_and(|f| f.abnormal != crate::flight::Abnormal::Normal);
            let said =
                !sw.crew || region_note_said(snap.on_ground, abnormal, crate::persona::chatter());
            crate::blackbox::record(
                "atc_region",
                serde_json::json!({
                    "lat": r1(snap.lat), "lon": r1(snap.lon),
                    "crew": sw.crew, "said": said, "msg": &sw.line
                }),
            );
            if sw.crew && said {
                // `voice.alert` mirrors to the panel; an explicit
                // `s.action(&sw.line)` here too double-wrote the line
                // to the chat (the Jerez relocation note, 2026-07-24).
                voice.alert(Priority::Advisory, sw.line);
            } else if sw.crew {
                // Unsaid, still on the panel for a glance: mirror OR
                // speak, never both.
                if let Some(s) = status.as_ref() {
                    s.action(&sw.line);
                }
            } else {
                if let Some(s) = status.as_ref() {
                    s.action(&format!("DEP: {}", sw.line));
                }
                voice.alert_station(
                    Priority::Advisory,
                    crate::voice::StationSpeaker::Departure,
                    sw.line,
                );
            }
        }
        if let Some(line) = crate::world::vector_tick(
            snap.lat,
            snap.lon,
            snap.alt_ft,
            snap.track_deg,
            snap.on_ground,
            std::time::Instant::now(),
        ) {
            // Radio physics: the vectors are heard ONLY on the
            // approach frequency. Off it, the controller still talks
            // (the plan advances, the blackbox records the missed
            // call) but the cockpit hears nothing — tuning away
            // mid-vector loses the service, it doesn't mute-proof it
            // ("ATC still speaking after I tuned another station",
            // 2026-07-12).
            let on_freq = com1.and_then(crate::party_line::tuned_controller)
                == Some(crate::voice::StationSpeaker::Approach);
            if on_freq {
                if let Some(s) = status.as_ref() {
                    // The prefix names the speaking position on the panel.
                    s.action(&format!("APP: {line}"));
                }
            }
            crate::blackbox::record(
                "vectors",
                serde_json::json!({
                    "lat": r1(snap.lat), "lon": r1(snap.lon),
                    "alt": r1(snap.alt_ft), "heard": on_freq, "msg": &line
                }),
            );
            if on_freq {
                voice.alert_station(
                    Priority::Advisory,
                    crate::voice::StationSpeaker::Approach,
                    line,
                );
            }
        }
        // Near miss: inside half a mile and 500 ft — the one air
        // picture where calm narration stops being enough. The tuned
        // controller issues the safety alert with an avoidance
        // suggestion; the CREW shouts it when no frequency is tuned
        // (a safety net must not depend on the radio, like the
        // incursion watch). Above 500 ft AGL only: real TCAS inhibits
        // alerts low, where the ghost lifting off behind a departure
        // would false-fire. Words only — the copilot never maneuvers
        // uncommanded.
        if !snap.on_ground && snap.agl_ft > 500.0 {
            let due = self
                .near_miss_last
                .is_none_or(|t| now.duration_since(t).as_secs() >= 15);
            if due {
                if let Some(f) = crate::world::nearest_traffic_factor(xp.view())
                    .await
                    .filter(crate::world::is_near_miss)
                {
                    self.near_miss_last = Some(now);
                    crate::blackbox::record(
                        "near_miss",
                        serde_json::json!({
                            "nm": r1(f.nm), "clock": f.clock, "ft": f.ft
                        }),
                    );
                    let speaker = com1.and_then(crate::party_line::tuned_controller);
                    if let Some(who) = speaker {
                        let callsign = crate::crew_radio::crew_callsign(&xp)
                            .await
                            .unwrap_or_else(|| "traffic alert".to_string());
                        let line =
                            format!("{callsign}, {}", crate::world::near_miss_line(&f, false));
                        if let Some(s) = status.as_ref() {
                            s.action(&format!("{}: {line}", who.tag()));
                        }
                        voice.alert_station(Priority::Warning, who, line);
                    } else {
                        let line = crate::world::near_miss_line(&f, true);
                        voice.alert(Priority::Warning, line);
                    }
                }
            }
        }
        // Traffic advisories: the controller narrates the nearest
        // factor off the TCAS picture (LiveTraffic or AI aircraft)
        // whenever one is watching — vectored to final, or simply on
        // any of the station's frequencies while airborne. At most
        // one call per 45 s. Narration only — separation stays the
        // pilot's job.
        let vectored = crate::party_line::vectored_callsign();
        let controller = if vectored.is_some() {
            Some(crate::voice::StationSpeaker::Approach)
        } else if !snap.on_ground {
            com1.and_then(crate::party_line::tuned_controller)
        } else {
            None
        };
        if let Some(speaker) = controller {
            // Never two voices on one conflict: while the near-miss
            // alert is speaking (15 s cadence), the calm narration of
            // the same target stands down.
            let due = self
                .traffic_last
                .is_none_or(|t| now.duration_since(t).as_secs() >= 45)
                && self
                    .near_miss_last
                    .is_none_or(|t| now.duration_since(t).as_secs() >= 45);
            if due {
                // Called once, and again only when it is more of a
                // factor (`traffic_worth_recalling`).
                let factor = crate::world::nearest_traffic_factor(xp.view()).await;
                let fresh = factor.as_ref().filter(|f| {
                    self.traffic_called
                        .as_ref()
                        .is_none_or(|c| crate::world::traffic_worth_recalling(c, f))
                });
                let advisory = fresh.map(crate::world::advisory_line);
                if let Some(f) = fresh {
                    self.traffic_called = Some(f.clone());
                }
                // Close the loop: the last factor is gone.
                if factor.is_none() && self.traffic_open {
                    self.traffic_open = false;
                    self.traffic_called = None;
                    self.traffic_last = Some(now);
                    let line = "clear of traffic.".to_string();
                    if let Some(s) = status.as_ref() {
                        s.action(&format!("{}: {line}", speaker.tag()));
                    }
                    voice.alert_station(Priority::Advisory, speaker, line);
                }
                if let Some(call) = advisory {
                    self.traffic_open = true;
                    self.traffic_last = Some(now);
                    // Address the flight when vectoring gave us the
                    // callsign; otherwise read the tail number.
                    let callsign = match &vectored {
                        Some(c) => c.clone(),
                        None => crate::crew_radio::crew_callsign(&xp)
                            .await
                            .unwrap_or_else(|| "traffic alert".to_string()),
                    };
                    let line = format!("{callsign}, {call}");
                    if let Some(s) = status.as_ref() {
                        s.action(&format!("{}: {line}", speaker.tag()));
                    }
                    crate::blackbox::record(
                        "traffic_advisory",
                        serde_json::json!({ "msg": &line }),
                    );
                    voice.alert_station(Priority::Advisory, speaker, line);
                }
            }
        }
    }

    /// Our own traffic: the fleet at the installed field stepped on the
    /// sim's clock and traded with the station (`traffic_link`), right after
    /// the runway watch. Nothing at all unless `XCOPILOT_TRAFFIC=ours`.
    pub(crate) async fn traffic_link(&mut self, t: &Tick<'_>, _heard: &Heard) {
        let sim_speed = crate::climb_watch::sim_speed_now();
        let _ = self
            .traffic
            .tick(t.xp, t.voice, t.snap, sim_speed, t.now)
            .await;
        // The frequency the fleet and the tower made, aired as it may be:
        // the tower tuned, never over our PTT or a pilot's line waiting,
        // never later than 8 s. Nothing queued, nothing done.
        let _ = t
            .voice
            .pump_overheard(t.now, crate::pollers::command_waiting());
    }

    /// Under `quiet` on the ground the vectors still advance, unheard.
    pub(crate) fn vectors_unheard(snap: &Snapshot) {
        let _ = crate::world::vector_tick(
            snap.lat,
            snap.lon,
            snap.alt_ft,
            snap.track_deg,
            snap.on_ground,
            std::time::Instant::now(),
        );
    }
}

#[cfg(test)]
mod seam_tests {
    /// The crew reaches the station ONLY through the seam: `party_line`
    /// (inputs, reads, the types it names) and `world` (the ticks this file
    /// drives). The station is crate-private in `atc`, so a path into it
    /// does not compile; this scan names the rule where it would be broken
    /// (an alias of the module, a path in a string a source scan matches).
    /// Comments may still name the world's own items.
    #[test]
    fn the_station_is_reached_only_through_the_seam() {
        let module = "station";
        let src = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        let mut hits = Vec::new();
        let mut files = 0;
        for f in crate::test_support::rust_files(&src) {
            files += 1;
            let text = std::fs::read_to_string(&f).expect("read source");
            for (n, line) in text.lines().enumerate() {
                let code = line.split("//").next().unwrap_or("");
                let path_into = code.match_indices(&format!("{module}::")).any(|(i, _)| {
                    !code[..i]
                        .chars()
                        .next_back()
                        .is_some_and(|c| c.is_ascii_alphanumeric() || c == '_')
                });
                let alias = code.contains("atc::{")
                    && code
                        .split(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
                        .any(|w| w == module);
                if path_into || alias {
                    hits.push(format!("{}:{}: {}", f.display(), n + 1, line.trim()));
                }
            }
        }
        assert!(files > 100, "the scan found only {files} files");
        assert!(
            hits.is_empty(),
            "the crew names the station directly; go through party_line or world: {hits:#?}"
        );
    }
}
