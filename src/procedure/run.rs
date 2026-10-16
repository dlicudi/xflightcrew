//! The loop: the launch, `fly`, whole, and the terrain probe's verdict.

use super::*;

/// Spawn the follower on its own thread + runtime + client, the circuit's shape.
pub fn start_scoped(
    base_url: String,
    voice: Option<Voice>,
    plan: Plan,
    parents: Vec<std::sync::Arc<AtomicBool>>,
) -> bool {
    let Some(lease) = RUN.begin() else {
        return false;
    };
    if ACTIVE.swap(true, Ordering::SeqCst) {
        return false;
    }
    ABORT.store(false, Ordering::SeqCst);
    // A new procedure never inherits the last one's missed approach: a
    // normal landing leaves it stashed for the next steep go-around.
    take_missed_legs();
    std::thread::spawn(move || {
        let _lease = lease.with_cleanup(&ACTIVE, || {
            publish_leg(None);
            publish_tod(None);
            publish_fix_dist(None);
            clear_pilot_altitude();
            crate::terrain::publish_floor_ahead(None);
            publish_route_ahead(None);
            take_splice();
            crate::tools::speedhold::release();
            hold_at_cleared(None, "follower_ended");
        });
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            ACTIVE.store(false, Ordering::SeqCst);
            return;
        };
        rt.block_on(async {
            // A second client for the one thing owed on EVERY exit: a
            // speedbrake this follower put out goes back in, whoever ended
            // the flight and however (uncancelled on purpose).
            let stow = XPlaneClient::new(base_url.clone());
            fly(
                XPlaneClient::new(base_url.clone())
                    .with_cancel(_lease.cancel.clone())
                    .with_parents(parents),
                base_url,
                voice,
                plan,
            )
            .await;
            if DRAG_OURS.swap(false, Ordering::SeqCst) {
                let _ = crate::tools::controls::speedbrake_set(&stow, 0.0).await;
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({"event": "speedbrake", "out": false, "why": "follower_ended"}),
                );
            }
        });
        ACTIVE.store(false, Ordering::SeqCst);
        publish_leg(None);
        publish_tod(None);
        publish_fix_dist(None);
    });
    true
}

pub(super) fn alert(voice: &Option<Voice>, priority: Priority, line: &str) {
    tracing::info!(target: "xflightcrew::procedure", spoke = %line, "copilot");
    if let Some(v) = voice {
        v.alert(priority, line.to_string());
    }
}

pub(super) fn stand_down(voice: &Option<Voice>, line: &str, reason: &str) {
    RUN.record(if reason == "abort" {
        sim::run::Outcome::Cancelled
    } else {
        sim::run::Outcome::Failed(reason.into())
    });
    ACTIVE.store(false, Ordering::SeqCst);
    clear_pilot_altitude();
    hold_at_cleared(None, "stand_down");
    crate::terrain::publish_floor_ahead(None);
    // Drop any cruise-power hold so nothing keeps working the throttle after we
    // hand back (no-op if it was never armed).
    crate::tools::speedhold::release();
    crate::blackbox::record(
        "procedure",
        serde_json::json!({"event": "stand_down", "reason": reason}),
    );
    alert(voice, Priority::Caution, line);
}

pub(super) async fn fly(xp: XPlaneClient, base_url: String, voice: Option<Voice>, plan: Plan) {
    // The flight this run was launched in: its stash is that flight's.
    let flight = MISSED_LEGS.now();
    let Plan {
        name,
        mut legs,
        end,
        vy_kts,
        cruise_ias_kts,
        transition_altitude_ft,
        climb_minimums,
        mut arrival_constraints,
        visual_deg,
        missed_legs,
        cruise_schedule,
        arrival_handoff_kts,
    } = plan;
    // Never judge the aircraft against a minimum above the level it is
    // flying to (`minimums_within_cruise`); the arm says so, and this
    // covers every other way into the follower.
    let cruise_ft = legs.iter().filter_map(|l| l.alt1_ft).fold(f64::MIN, f64::max);
    let (climb_minimums, above) = if cruise_ft > 0.0 {
        minimums_within_cruise(climb_minimums, cruise_ft)
    } else {
        (climb_minimums, Vec::new())
    };
    for m in &above {
        crate::blackbox::record(
            "procedure",
            serde_json::json!({"event": "minimum_above_cruise", "fix": m.fix, "min_ft": m.min_ft, "cruise_ft": cruise_ft}),
        );
    }
    let mut handoff = handoff_index(&legs);
    let _detail = crate::auto_status::DetailGuard::hold(&DETAIL);
    // Process-wide facts must never outlive the flight that set them.
    clear_pilot_altitude();
    crate::terrain::publish_floor_ahead(None);
    // The altimeter first: every altitude below is read off it. Said only
    // when the knob actually moved (a correct setting earns no chatter).
    if let Some(ta) = transition_altitude_ft {
        if let Some(sync) = crate::tools::instruments::sync_altimeter(&xp, ta).await {
            crate::blackbox::record(
                "procedure",
                serde_json::json!({"event": "altimeter_set", "was": sync.was, "now": sync.now, "want": sync.want, "took": sync.took()}),
            );
            // A set that did not take is never quiet: every altitude below
            // is wrong by the difference (EGSS 2026-09-22, 12,500 ft).
            if !sync.took() {
                alert(&voice, Priority::Warning, &sync.line());
            } else if sync.changed() {
                alert(&voice, Priority::Advisory, &sync.line());
            }
        }
    }
    crate::blackbox::record(
        "procedure",
        serde_json::json!({"event": "armed", "name": name, "legs": legs.len(), "approach": matches!(end, EndAction::Approach{..})}),
    );
    alert(&voice, Priority::Advisory, &format!("Flying {name}."));

    // Seed from real instruments BEFORE engaging anything. An unconstrained
    // first leg holds indicated altitude, just like subsequent legs.
    let (var, psi, alt) = tokio::join!(
        xp.get_float(MAG_VAR),
        xp.get_float(PSI_TRUE),
        xp.get_float(ALT_IND_FT),
    );
    let Some([mut mag_var, psi_seed, mut carry_alt]) = measurements("procedure", [var, psi, alt])
    else {
        stand_down(
            &voice,
            "Flight instruments unavailable. Your aircraft.",
            "seed_unread",
        );
        return;
    };
    let mut bug_mag = true_to_mag(psi_seed, mag_var);
    let mut heading_hold = HeadingHold::default();
    let started = std::time::Instant::now();
    if !autoflight::set_dial_confirmed("procedure", &xp, Dial::HeadingBug, bug_mag, &mut None, 0.0).await {
        stand_down(
            &voice,
            "Heading selection failed. Your aircraft.",
            "seed_unconfirmed",
        );
        return;
    }
    // The servos in through the one engage door (`ap_engage`: FD first, on
    // the tape); the tick's lateral watch below confirms them.
    // The follower carries no `ApConfig`: the live airframe's FD list.
    let ap_fd = crate::ap_engage::fd_commands();
    crate::ap_engage::press_logged(&xp, &ap_fd, "procedure", "seed", 1).await;
    let hdg_seed = crate::autoflight::read_status(&xp, HDG_STATUS).await.ok();
    autoflight::bring_heading(&xp, hdg_seed, "procedure_seed").await;
    let mut servo_watch = crate::ap_engage::ServoWatch::default();
    // What we COMMANDED last tick, a different thing from the leg constraint
    // carried across unconstrained fixes, and the pacing `descent_target_ft`
    // actually needs. They were one variable until 2026-09-13, and since
    // `carry_alt` is overwritten with the new leg's own constraint one line
    // after it is read, the top-of-descent test compared the constraint with
    // itself from the second tick of every leg onward: the leg to JRZ armed at
    // 55.9 nm and began its 5,300 ft descent immediately, into the Sierra. The
    // deferral worked for exactly one tick per leg, which is why the logs
    // always showed one honest 8,000 followed by the dive.
    let mut prev_cmd_alt = carry_alt;
    let mut last_vs_cmd: Option<f64> = None;
    let mut prev_ias: Option<(f64, std::time::Instant)> = None;
    let mut ias_trend_f = 0.0_f64;
    // A descent flown at idle, the speed on the pitch (`IdleDescent`).
    let mut idle_descent = IdleDescent::default();
    // The altitude PRESELECT is driven to the leg target too: a stale dial
    // with capture armed (1,122 ft on the Jerez departure, from nobody's
    // loop) would otherwise level the climb wherever it happened to sit —
    // and driven to the real target, the capture works FOR us: it owns the
    // level-off at the constraint more cleanly than the VS chase.
    let mut last_alt_dial: Option<f64> = None;

    // Cruise power: on an enroute flight, arm the flight speed-hold at the
    // planned indicated cruise speed. The throttle law then FINDS the power —
    // full while slow (the climb still commands full throttle), settling to
    // cruise power once at speed, robust to weight/altitude/temp. Pitch (the VS
    // law below) owns altitude; the governor owns speed. Released at every exit
    // so autoland/the arrival own the throttle alone.
    if let Some(kts) = power_hold_target(&end, cruise_ias_kts) {
        // No altitude here to cap against — the first tick below has one and
        // corrects it within a fifth of a second, which is why the cap lives
        // there and not in this arm.
        crate::tools::speedhold::arm_flight(base_url.clone(), voice.clone(), kts);
        let event = if matches!(end, EndAction::Enroute) {
            "cruise_power"
        } else {
            "approach_power"
        };
        crate::blackbox::record(
            "procedure",
            serde_json::json!({"event": event, "target_ias": kts}),
        );
    }

    let mut said_star_speed: std::collections::HashSet<String> = std::collections::HashSet::new();
    // The OFP schedule's last logged pick (leg, source, why rejected), and
    // the planned steps already said.
    let mut schedule_logged: Option<(usize, &'static str, Option<&'static str>)> = None;
    let mut said_steps: std::collections::HashSet<String> = std::collections::HashSet::new();
    // The STAR speed limits whose slow-down has begun: held from then on
    // (`arrival_speed_cap`).
    let mut star_speed_begun: std::collections::HashSet<usize> = std::collections::HashSet::new();
    // The arrival's hand-over speed, an enroute plan's only (#199), whether
    // its slow-down has begun (held from then on) and was said.
    let arrival_handoff_kts = arrival_handoff_for(arrival_handoff_kts, &end);
    let mut handoff_begun = false;
    let mut said_handoff = false;
    // An instrument approach's slow-down to its FAF (`faf_speed_cap`),
    // planned on the airframe's energy model (`approach_energy`; `None`
    // where it declares none: the clean approach speed flies as before),
    // whether it has begun, the flap out when it began, and the FAF speed
    // this tick for the log.
    let faf_model = match &end {
        EndAction::Approach { approach_ias, .. } => crate::aircraft::live_profile()
            .and_then(|p| crate::approach_energy::EnergyModel::from_profile(&p, 3.0))
            .map(|m| (m, crate::approach_energy::gate_kts(*approach_ias))),
        _ => None,
    };
    let mut faf_begun = false;
    let mut faf_said = false;
    let mut faf_flap_out: Option<f64> = None;
    let mut faf_now: Option<FafEnergy> = None;
    // The speedbrake's last change and which changes are said (`descent_drag`,
    // `DragVoice`).
    let mut drag_changed: Option<std::time::Instant> = None;
    let mut drag_voice = DragVoice::default();
    let drag_clock = std::time::Instant::now();
    let mut star_ceiling_logged: Option<usize> = None;
    // The lowest ceiling seen on the leg flying (`ceilinged_leg_alt`).
    let mut ceiling_latch: Option<(usize, f64)> = None;
    // The one descent a pilot-held altitude is reminded about: (the pilot's
    // altitude, the fix). The plan's later descents under the same held
    // altitude are silent; a new altitude from the pilot is a new hold.
    let mut held_leg: Option<(f64, String)> = None;
    // The plan's profile, for the rule that a descent never climbs back to a
    // navlog altitude above it (`DescentProfile`). Off on an approach: its
    // own procedure owns the heights.
    let descent_cruise_ft = (cruise_ft > 0.0 && !matches!(end, EndAction::Approach { .. }))
        .then_some(cruise_ft);
    let mut plan_top = descent_cruise_ft.and_then(|c| plan_top_index(&legs, c));
    let mut plan_peak_ft = prev_cmd_alt;
    let mut descent_held_logged: Option<usize> = None;
    let mut climb_held_logged: Option<usize> = None;
    if !arrival_constraints.is_empty() {
        crate::blackbox::record(
            "procedure",
            serde_json::json!({"event": "star_constraints", "constraints": arrival_constraints.iter().map(|c|
                serde_json::json!({"fix": c.fix, "min_ft": c.min_ft, "max_ft": c.max_ft, "max_kts": c.max_kts})).collect::<Vec<_>>()}),
        );
    }
    // The STAR's windows the plan does not make, said once each: over the
    // cruise at the arm (and a splice), and any the airframe cannot reach
    // from where it is as it flies (#32, #156).
    let mut said_windows: std::collections::HashSet<String> = std::collections::HashSet::new();
    let mut said_unable_windows: std::collections::HashSet<String> = std::collections::HashSet::new();
    let star_cruise_ft = (cruise_ft > 0.0).then_some(cruise_ft);
    let say_above_cruise = |legs: &[ProcLeg], cons: &[crate::nav::StarConstraint], said: &mut std::collections::HashSet<String>| {
        let Some(cruise) = star_cruise_ft else { return };
        for (fix, min_ft, cruise_ft) in windows_above_cruise(legs, cons, cruise) {
            if said.insert(fix.to_ascii_uppercase()) {
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({"event": "star_window_above_cruise", "fix": fix, "min_ft": min_ft, "cruise_ft": cruise_ft}),
                );
                alert(&voice, Priority::Advisory, &above_cruise_line(&fix, min_ft, cruise_ft));
            }
        }
    };
    say_above_cruise(&legs, &arrival_constraints, &mut said_windows);
    let mut idx = 0usize;
    // The join onto an approach's final from its IF, and how the last tick
    // steered (`crate::course_leg`, #106).
    let mut final_join = crate::course_leg::FinalJoin::default();
    // The hold being flown, by leg index, and the last hold leg seen (#101).
    let mut hold: Option<(usize, crate::course_leg::HoldFlight)> = None;
    let mut hold_seen: Option<usize> = None;
    let mut steer_was = crate::course_leg::Steer::Direct;
    let mut closure: Option<(usize, crate::circuit::Closure)> = None;
    let mut stalls = 0u32;
    let mut progress_s = 0.0;
    let mut tick_clock = std::time::Instant::now();
    // The FMS is sequenced once per leg, when the follower first steers at it.
    let mut fms_leg: Option<usize> = None;
    // The aircraft's own LNAV flies the lateral of the plain legs where the
    // airframe declares it, on the full-auto enroute plan only; the heading
    // bug is the watched fallback (#227, `crate::lnav`).
    let lnav_on = matches!(end, EndAction::Enroute) && autoflight::panel().lnav_declared();
    let mut lnav = crate::lnav::LnavLateral::default();
    // The last `lnav` `on_bug` reason published, with its leg.
    let mut bug_why_said: Option<(usize, &'static str)> = None;
    // The leg index last published with the route (`route_ahead`).
    let mut route_published: Option<usize> = None;
    // Terrain look-ahead: probed on its own slow cadence (the ground does not
    // move), disabled for the flight the first time the plugin has no answer so
    // an old build degrades to the previous behaviour instead of stalling the
    // tick for two seconds every ten.
    let mut last_probe = std::time::Instant::now() - std::time::Duration::from_secs(60);
    let mut terrain_available = true;
    let mut terrain_misses: u32 = 0;
    let mut said_floor: Option<f64> = None;
    let mut said_alt_hold = false;
    let mut said_short: std::collections::HashSet<String> = std::collections::HashSet::new();
    let mut gear_set = false;
    let mut last_flap: Option<f64> = None;
    let mut prev_alt_sample: Option<(f64, std::time::Instant)> = None;
    let mut climb_rate_fpm = 0.0_f64;
    // The autopilot's answer to the VS wheel, watched (#181).
    let mut climb_watch = crate::climb_watch::ClimbWatch::default();
    let mut last_verdict = crate::climb_watch::Verdict::Flying;
    let watch_t0 = std::time::Instant::now();
    let mut rate_samples = 0u32;
    let mut said_unable: std::collections::HashSet<String> = std::collections::HashSet::new();
    let mut said_warned_unable = 0usize;
    // The departure minimums met so far and the floors ahead
    // (`DepartureMinimums`), the minimums said as held under the clearance,
    // and the clearance's altitude while it was holding the target down.
    let mut minimums_state = DepartureMinimums::default();
    // The 250 below 10,000 cap, once begun held through a level near the line.
    let mut airspace = crate::tools::speedhold::AirspaceLimit::default();
    // The ceiling where we fly on an airframe whose Vmo rises with altitude
    // (`vmo_high:`), with the vertical speed it projects from: the
    // altimeter's change across ticks.
    let mut vmo = crate::aircraft::VmoCeiling::default();
    let mut vmo_prev_alt: Option<f64> = None;
    let mut said_min_capped: std::collections::HashSet<String> = std::collections::HashSet::new();
    let mut cap_holding: Option<f64> = None;
    // The VS wheel's last action logged (`vs_action`), so a change is one record.
    let mut vs_action_logged: Option<VsAction> = None;
    while idx < legs.len() {
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        if xp.cancelled() || ABORT.load(Ordering::SeqCst) {
            stand_down(&voice, "Cancelled. Your aircraft.", "abort");
            return;
        }
        // A frozen sim (pause/replay) suspends the sequence; the AP holds.
        if crate::xplane::sim_frozen(&xp).await {
            tick_clock = std::time::Instant::now();
            continue;
        }
        // An arrival re-picked in flight (#79): the new STAR's legs from its
        // entry, and its windows, taken whole between two ticks.
        if let Some(s) = take_splice() {
            match apply_splice(&legs, idx, &s) {
                Ok(new) => {
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "star_spliced", "idx": idx, "from": s.from, "entry": s.entry,
                            "why": s.why, "fixes": new[s.from..].iter().map(|l| l.fix_ident.clone()).collect::<Vec<_>>(),
                            "alts": new[s.from..].iter().map(|l| l.alt1_ft.map(f64::round)).collect::<Vec<_>>(),
                            "constraints": s.constraints.len()}),
                    );
                    legs = new;
                    arrival_constraints = s.constraints;
                    say_above_cruise(&legs, &arrival_constraints, &mut said_windows);
                    handoff = handoff_index(&legs);
                    plan_top = descent_cruise_ft.and_then(|c| plan_top_index(&legs, c));
                    star_speed_begun.clear();
                    said_star_speed.clear();
                    star_ceiling_logged = None;
                    ceiling_latch = None;
                    fms_leg = None;
                    route_published = None;
                }
                Err(why) => crate::blackbox::record(
                    "procedure",
                    serde_json::json!({"event": "splice_stale", "idx": idx, "from": s.from, "entry": s.entry, "why": why}),
                ),
            }
        }
        if route_published != Some(idx) {
            route_published = Some(idx);
            publish_route_ahead(Some((idx, legs.clone())));
        }

        let (
            lat,
            lon,
            elev_m,
            alt_ind,
            psi_r,
            hpath_r,
            agl_m,
            vs_st,
            hdg_st,
            ias_r,
            gs_r,
            phi_r,
            var_r,
            servos_r,
        ) = tokio::join!(
            xp.get_float(LAT),
            xp.get_float(LON),
            xp.get_float(ELEV_M),
            xp.get_float(ALT_IND_FT),
            xp.get_float(PSI_TRUE),
            xp.get_float(HPATH),
            xp.get_float(AGL_M),
            crate::autoflight::read_status(&xp, VS_STATUS),
            crate::autoflight::read_status(&xp, HDG_STATUS),
            xp.get_float(IAS),
            xp.get_float(GS_MS),
            xp.get_float(crate::datarefs::BANK_DEG),
            xp.get_float(MAG_VAR),
            crate::autoflight::read_status(&xp, crate::datarefs::SERVOS_ON),
        );
        let now = std::time::Instant::now();
        let dt = now.duration_since(tick_clock).as_secs_f64();
        tick_clock = now;
        let Some([lat, lon, elev_m, alt_ft, psi, agl_m, vs_st, hdg_st, ias, gs, phi]) =
            measurements(
                "procedure",
                [
                    lat, lon, elev_m, alt_ind, psi_r, agl_m, vs_st, hdg_st, ias_r, gs_r, phi_r,
                ],
            )
        else {
            continue;
        };
        progress_s += dt;
        let true_alt_ft = elev_m * M_TO_FT;
        // The ceiling this tick: the speed owed and the governor's target
        // are held under it, so the Citation X slows for its 270 below
        // 8,000 ft before the descent gets there. Without a `vmo_high:` it
        // is the published ceiling.
        let vmo_vs = match vmo_prev_alt {
            Some(prev) if (0.2..=5.0).contains(&dt) => (true_alt_ft - prev) / dt * 60.0,
            _ => 0.0,
        };
        vmo_prev_alt = Some(true_alt_ft);
        let speed_ceiling = vmo.ceiling(alt_ft, vmo_vs);
        let agl_ft = agl_m * M_TO_FT;
        // The magnetic variation HERE, not at the departure airport. It is a
        // fact of the PLACE, and this loop crosses enough of the world to
        // change it: read once at Keflavik and carried to Scotland, it was
        // 11.4 degrees stale, so every heading the follower wrote was 11.4
        // degrees off the nose. The roll-out capture that is supposed to end
        // a turn ("bug on the nose") instead commanded a standing 11 degree
        // error, the AP held 25 degrees of bank for 95 seconds and went round
        // 160 degrees, and the arrival's intercept could not close either
        // (BIKF-EGPH, 2026-09-20). A stale reading kept, rather than the
        // tick skipped: a lost read is no reason to throw away the last good
        // variation, which is right to a fraction of a degree a minute later.
        if let Ok(v) = var_r {
            if v.is_finite() {
                mag_var = v;
            }
        }
        // The 250 below 10,000 limit, applied as we CLIMB THROUGH it rather
        // than once at the arm. The cruise target is armed at a thousand feet
        // on departure and would otherwise be held all the way up — which is
        // exactly how a 255 kt cruise speed became a speed bust for the whole
        // climb (2026-09-20). Re-armed only when the cap actually changes;
        // `arm_flight` is idempotent and a running hold just takes the new
        // number, so this costs nothing per tick.
        let unmet = minimums_state.unmet(&climb_minimums, &legs, idx, alt_ft);
        // An unmet minimum above an IFR clearance's altitude is held under
        // the clearance until departure clears higher: not a climb to judge
        // (a watch that sees no climb would hand the aircraft back).
        let under_clearance = match (unmet, cleared_ceiling()) {
            (Some((_, m)), Some(c)) => {
                crate::pilot_altitude::held().is_none() && m > c + MINIMUM_MET_TOL_FT
            }
            _ => false,
        };
        // The climb rate, from the altimeter itself (smoothed): what the
        // minimum watch predicts with.
        if let Some((a0, t0)) = prev_alt_sample {
            let span = now.duration_since(t0).as_secs_f64();
            if (0.3..=5.0).contains(&span) {
                let inst = (alt_ft - a0) / span * 60.0;
                // Seeded from the first real sample, never from zero: the
                // watch judged its first tick at a zero climb rate, predicted
                // arriving at WI505 at the present altitude and handed the
                // aircraft back one second after the takeoff did, at 3,000 ft
                // in the valley (LOWI, 2026-09-24).
                climb_rate_fpm = if rate_samples == 0 { inst } else { 0.7 * climb_rate_fpm + 0.3 * inst };
                rate_samples += 1;
            }
        }
        prev_alt_sample = Some((alt_ft, now));
        // No judgement until the rate is a measurement (several seconds).
        let unmet_judged = unmet.filter(|_| rate_samples >= MIN_RATE_SAMPLES && !under_clearance);
        if let Some((at, min_ft)) = unmet_judged {
            let fix = legs[at].fix_ident.clone();
            let dist = distance_to_leg_nm(lat, lon, &legs, idx, at);
            match watch_minimum(alt_ft, climb_rate_fpm, gs * MS_TO_KTS, dist, min_ft) {
                MinimumWatch::OnTrack => {}
                MinimumWatch::Short { predicted_ft } => {
                    if said_short.insert(fix.clone()) {
                        crate::blackbox::record(
                            "procedure",
                            serde_json::json!({"event": "minimum_short", "fix": fix,
                                "min_ft": min_ft, "predicted_ft": predicted_ft, "dist_nm": dist}),
                        );
                        alert(
                            &voice,
                            Priority::Caution,
                            &format!(
                                "We'll cross {fix} about {:.0} feet under its {} minimum. Climbing at best rate.",
                                ((min_ft - predicted_ft) / 50.0).round() * 50.0,
                                crate::flightplan::spoken_feet(min_ft)
                            ),
                        );
                    }
                }
                MinimumWatch::Unable { predicted_ft } => {
                    // A miss predicted while CLIMBING is flown on, at best
                    // rate: that is the most anyone could do, and a hand-over
                    // near the ground mid-departure is the worst moment for
                    // it (LOWI, 2026-09-24). Said, once per fix. Only an
                    // aircraft that is not climbing at all is handed back.
                    let climbing = climb_rate_fpm >= NOT_CLIMBING_FPM;
                    if said_unable.insert(fix.clone()) || !climbing {
                        crate::blackbox::record(
                            "procedure",
                            serde_json::json!({"event": "minimum_unable", "fix": fix,
                                "min_ft": min_ft, "predicted_ft": predicted_ft, "alt_ft": alt_ft,
                                "dist_nm": dist, "rate_fpm": climb_rate_fpm, "climbing": climbing}),
                        );
                    }
                    if !climbing {
                        stand_down(
                            &voice,
                            &format!(
                                "We're not climbing and {fix} needs {}. Your aircraft. Climb now.",
                                crate::flightplan::spoken_feet(min_ft),
                            ),
                            "minimum_unable",
                        );
                        return;
                    }
                    if said_unable.len() > said_warned_unable {
                        said_warned_unable = said_unable.len();
                        alert(
                            &voice,
                            Priority::Warning,
                            &format!(
                                "We won't make {} at {fix}, about {:.0} feet under. Climbing at best rate.",
                                crate::flightplan::spoken_feet(min_ft),
                                ((min_ft - predicted_ft) / 50.0).round() * 50.0,
                            ),
                        );
                    }
                }
            }
        }
        // The speed owed this tick (the governor's target outside an idle
        // descent), for the vertical's speed protection below.
        let mut speed_owed: Option<f64> = None;
        if let Some(planned) = power_hold_target(&end, cruise_ias_kts) {
            // The OFP's cruise schedule on a cruise leg of an enroute flight:
            // the speed its cost index planned, held as IAS or Mach,
            // whichever is lower at the level flown (#101).
            let planned = if matches!(end, EndAction::Enroute) {
                let sched = scheduled_cruise_kts(&cruise_schedule, &legs[idx], alt_ft);
                let ceiling = speed_ceiling;
                let pick = cruise_pick(
                    planned,
                    sched.as_ref(),
                    crate::aircraft::speed_floor_kts(),
                    ceiling,
                    // The published ceiling IS the cruise cap (one rule).
                    ceiling,
                );
                let key = (idx, pick.source, pick.rejected);
                if sched.is_some() && schedule_logged != Some(key) {
                    schedule_logged = Some(key);
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({
                            "event": "cruise_schedule", "idx": idx, "fix": legs[idx].fix_ident,
                            "source": pick.source, "target": pick.kts, "rejected": pick.rejected,
                            "capped_from": pick.capped_from,
                            "plan_ias": sched.as_ref().and_then(|s| s.plan_ias_kts),
                            "mach": sched.as_ref().and_then(|s| s.mach),
                            "ofp_target": sched.as_ref().map(|s| s.target_kts),
                            "profile_kts": planned, "alt_ft": alt_ft, "ias": ias,
                        }),
                    );
                }
                pick.kts
            } else {
                planned
            };
            // A departure minimum ahead is climbed at Vy, the steepest the
            // airframe goes up: at cruise speed the SF50 made 375 ft a mile
            // where MOGT3H asks 600 (LOWI, 2026-09-24).
            let planned = if unmet.is_some() && vy_kts > 0.0 { vy_kts } else { planned };
            // The pilot's speed outranks the plan's, as their altitude does.
            let planned = pilot_speed().unwrap_or(planned);
            // The STAR's speed limits: slowed in time for each, held after.
            let star_cap = arrival_speed_cap(&legs, &arrival_constraints, idx, lat, lon, ias, gs * MS_TO_KTS, &mut star_speed_begun);
            // Flown a little under the limit (`STAR_SPEED_MARGIN_KTS`): a
            // maximum is not a target.
            let planned = match &star_cap {
                Some((kts, fix)) if *kts - STAR_SPEED_MARGIN_KTS < planned => {
                    if said_star_speed.insert(fix.clone()) {
                        // The slow-down BEGINS here (`ias` is now, not at the
                        // fix; `star_speed_crossed` is the crossing), miles
                        // along the route to the fix.
                        let dist_nm = (idx..legs.len())
                            .find(|&k| legs[k].fix_ident == *fix)
                            .map(|k| distance_to_leg_nm(lat, lon, &legs, idx, k));
                        crate::blackbox::record(
                            "procedure",
                            serde_json::json!({"event": "star_speed", "fix": fix, "kts": kts, "ias": ias,
                                "at": "slowing", "dist_nm": dist_nm}),
                        );
                        alert(
                            &voice,
                            Priority::Advisory,
                            &format!("Slowing to {kts:.0} knots for {fix}."),
                        );
                    }
                    *kts - STAR_SPEED_MARGIN_KTS
                }
                _ => planned,
            };
            // The speed the arrival hands to autoland at the end of the
            // route, slowed for in time like a STAR limit (#199): EGBB-EGNX
            // (2026-09-30) held its 250 to the last fix and gave autoland a
            // clean 737 at 252 kt 8.4 nm out on the path.
            let planned = match arrival_handoff_kts {
                Some(kts) if kts < planned && !legs.is_empty() => {
                    let to_end = distance_to_leg_nm(lat, lon, &legs, idx, legs.len() - 1);
                    match arrival_handoff_cap(to_end, ias, gs * MS_TO_KTS, kts, &mut handoff_begun) {
                        Some(cap) => {
                            if !said_handoff {
                                said_handoff = true;
                                crate::blackbox::record(
                                    "procedure",
                                    serde_json::json!({"event": "arrival_slowdown", "kts": cap,
                                        "to_end_nm": to_end, "ias": ias, "planned": planned}),
                                );
                                alert(
                                    &voice,
                                    Priority::Advisory,
                                    &format!("Slowing to {cap:.0} knots for the approach."),
                                );
                            }
                            cap
                        }
                        None => planned,
                    }
                }
                _ => planned,
            };
            // An instrument approach crosses its FAF at a speed and flap the
            // final can absorb (`faf_speed_cap`): planned back from the
            // 1,000 ft gate on the airframe's own deceleration, slowed for in
            // time (the descent owed first, then idle level), configured as
            // the speed allows. Autoland, handed a clean 737 at 210 a mile
            // before FF26L, could not make its gate (EGKK, 2026-10-14).
            let on_map_final = legs[idx].is_map;
            faf_now = None;
            let planned = match (&faf_model, handoff) {
                (Some((m, gate_kts)), Some(hi)) if idx <= hi && !on_map_final && !map_short_of_runway(&legs[hi]) => {
                    match faf_to_threshold_nm(&legs, hi) {
                        Some(faf_thr_nm) => {
                            let faf = faf_energy(
                                m,
                                *gate_kts,
                                faf_thr_nm,
                                crate::stable_approach::GateKind::Instrument.gate_ft(),
                            );
                            faf_now = Some(faf);
                            // Round a pattern to the IF, its miles (as the
                            // descent is paced), then the legs on to the FAF.
                            let to_faf = final_join
                                .pattern_to_go_nm(&legs, idx, lat, lon, gs * MS_TO_KTS, mag_var)
                                .zip(legs[idx].fix.as_ref())
                                .map(|(p, f)| p + distance_to_leg_nm(f.lat, f.lon, &legs, idx + 1, hi))
                                .unwrap_or_else(|| distance_to_leg_nm(lat, lon, &legs, idx, hi));
                            let above_faf = legs[hi].alt1_ft.map_or(0.0, |a| alt_ft - a);
                            match faf_speed_cap(m, faf, to_faf, ias, gs * MS_TO_KTS, above_faf, descent_fpm(), &mut faf_begun)
                                .filter(|cap| *cap < planned)
                            {
                                Some(cap) => {
                                    if !faf_said {
                                        faf_said = true;
                                        // The flap already out stays out.
                                        faf_flap_out = crate::tools::flaps::read_handle(&xp).await.ok();
                                        crate::blackbox::record(
                                            "procedure",
                                            serde_json::json!({"event": "faf_slowdown", "kts": cap,
                                                "fix": legs[hi].fix_ident, "to_faf_nm": to_faf,
                                                "faf_path_nm": faf.path_nm, "gate_kts": gate_kts,
                                                "above_faf_ft": above_faf, "ias": ias, "planned": planned,
                                                "flap_out": faf_flap_out,
                                                "need_nm": faf_slowdown_nm(m, ias, cap, gs * MS_TO_KTS, above_faf, descent_fpm())}),
                                        );
                                        alert(
                                            &voice,
                                            Priority::Advisory,
                                            &format!("Slowing to {cap:.0} knots for the final approach fix."),
                                        );
                                    }
                                    // The configuration for the speed, extended as
                                    // it comes off, the gear with it (the plan's
                                    // own rule and its gear, `approach_energy`).
                                    // Never on an unread lever: a flap out it did
                                    // not see could be taken back in.
                                    let out = last_flap.or(faf_flap_out);
                                    let more = out.map(|o| faf_flap(m, ias, cap, o)).filter(|f| out.is_some_and(|o| *f > o + 0.01));
                                    if let Some(flap) = more {
                                        if crate::tools::flaps::set_handle(&xp, flap).await.is_ok() {
                                            last_flap = Some(flap);
                                            crate::blackbox::record(
                                                "procedure",
                                                serde_json::json!({"event": "faf_config", "flap": flap, "ias": ias,
                                                    "kts": cap, "to_faf_nm": to_faf, "gear": gear_set}),
                                            );
                                        }
                                    }
                                    let flap_now = last_flap.or(faf_flap_out).unwrap_or(0.0);
                                    if !gear_set
                                        && crate::approach_energy::gear_with_flap(flap_now)
                                        && crate::autoland::gear_may_extend(ias, crate::aircraft::gear_limit_kts())
                                    {
                                        gear_set = crate::tools::gear::set_handle(&xp, false).await.is_ok();
                                        crate::blackbox::record(
                                            "procedure",
                                            serde_json::json!({"event": "faf_config", "flap": flap_now, "ias": ias,
                                                "kts": cap, "to_faf_nm": to_faf, "gear": gear_set}),
                                        );
                                    }
                                    cap
                                }
                                None => planned,
                            }
                        }
                        None => planned,
                    }
                }
                _ => planned,
            };
            // On the final to a MAP: landing speed and configuration, so the
            // visual segment is flown stabilised (a jet cannot slow down and
            // go down 4.7 degrees in two miles).
            let planned = match (&end, on_map_final) {
                (EndAction::Approach { approach_ias, .. }, true) => *approach_ias,
                _ => planned,
            };
            // Approach's assigned speed is flown, not just read back, until
            // the handoff voids it; the final keeps the landing's (EGPH
            // 2026-10-16: "maintain 180" read back, 188 flown).
            let planned = match crate::party_line::assigned_speed() {
                Some(kts) if !on_map_final => kts,
                _ => planned,
            };
            if on_map_final {
                if !gear_set && crate::autoland::gear_may_extend(ias, crate::aircraft::gear_limit_kts()) {
                    gear_set = crate::tools::gear::set_handle(&xp, false).await.is_ok();
                }
                // A MAP ON the runway is an ordinary final: flap as the
                // approach goes (half inside 6 nm, full a mile before the 1,000 ft gate on the
                // path, `autoland::flap_schedule`), as a pilot flies an ILS.
                // Full flap from the FAF flew LIPR's 9 nm final at 65 kt for
                // eight minutes (2026-09-26). A MAP short of the runway keeps
                // the whole landing configuration for its visual segment.
                let wanted = if map_short_of_runway(&legs[idx]) {
                    1.0
                } else {
                    let d = legs[idx]
                        .fix
                        .as_ref()
                        .map(|f| crate::nav::haversine_nm(lat, lon, f.lat, f.lon));
                    let err = map_path(&legs[idx], d, gs * MS_TO_KTS).map_or(0.0, |(p, _)| alt_ft - p);
                    let by_distance = crate::autoland::flap_schedule(d.unwrap_or(9.9), err);
                    // And FLAPS ON SPEED: slowing toward the approach speed
                    // under the clean floor with the flaps up is a stall on
                    // a jet. By distance alone the 737 went from 217 to 150
                    // kt clean from 12 nm out and earned "Airspeed! Lower the
                    // nose" at LEMG 13 (2026-09-26). Near the clean floor,
                    // every detent the placards allow (`flap_within_limits`
                    // below steps them out as the speed comes down).
                    let near_clean_floor = crate::aircraft::speed_floor_kts()
                        .is_some_and(|f| ias < f + FLAPS_ON_SPEED_MARGIN_KTS);
                    if near_clean_floor { 1.0 } else { by_distance }
                };
                let flap = crate::autoland::flap_within_limits(wanted, ias, &crate::aircraft::flap_limits());
                if last_flap.is_none_or(|f: f64| (f - flap).abs() > 0.01)
                    && crate::tools::flaps::set_handle(&xp, flap).await.is_ok()
                {
                    last_flap = Some(flap);
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "map_config", "flap": flap, "gear": gear_set, "ias": ias}),
                    );
                }
            }
            // Floored HERE, against the floor for the configuration flown
            // (landing on the final to a MAP, clean otherwise), and compared
            // floored: comparing the raw 87 with the governor's floored 96
            // re-armed, and re-said the floor, every tick (LSZS, 2026-09-24).
            // The slow-down to the FAF flies the floor of the flap it put out.
            let floor = if on_map_final {
                crate::aircraft::approach_floor_kts()
            } else if faf_begun && faf_now.is_some() {
                last_flap
                    .or(faf_flap_out)
                    .and_then(crate::aircraft::config_floor_kts)
                    .or_else(crate::aircraft::speed_floor_kts)
            } else {
                crate::aircraft::speed_floor_kts()
            };
            // With memory (`AirspaceLimit`): a level at or near 10,000 ft
            // must not flip the cap on the altimeter's wander (#154).
            // And under the airframe's ceiling where we fly.
            let limited = airspace.limit(alt_ft, planned);
            let limited = speed_ceiling.map_or(limited, |c| limited.min(c));
            let want = crate::aircraft::floored_target(limited, floor);
            speed_owed = Some(want);
            // In a descent the governor sits under the speed owed and the
            // pitch holds it (`IdleDescent`); on a MAP's final the speed is
            // the landing's, flown on power.
            let governed = if on_map_final { want } else { idle_descent.governor_kts(want, floor) };
            if crate::tools::speedhold::flight_target().is_some_and(|t| (t - governed).abs() > 0.5) {
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({
                        "event": "speed_limit", "alt_ft": alt_ft,
                        "planned": planned, "target": governed, "owed": want,
                        "idle_descent": idle_descent.on(),
                        "airspace_capped": airspace.capped(),
                    }),
                );
                crate::tools::speedhold::arm_flight_with_floor(base_url.clone(), voice.clone(), governed, floor);
            }
        }

        let leg = &legs[idx];

        // Skip a leg whose fix never resolved — flying at a phantom (0,0) fix
        // would drag the aircraft off the planet. Log it and move on.
        if leg.fix.is_none() && !leg.fix_ident.is_empty() {
            crate::blackbox::record(
                "procedure",
                serde_json::json!({"event": "skip_unresolved", "idx": idx, "fix": leg.fix_ident}),
            );
            idx += 1;
            continue;
        }
        publish_leg(leg.fix.as_ref().map(|f| ActiveLeg {
            fix: leg.fix_ident.clone(),
            lat: f.lat,
            lon: f.lon,
        }));
        if fms_leg != Some(idx) {
            fms_leg = Some(idx);
            sequence_fms(&xp, idx, leg).await;
        }

        // --- Lateral: steer the great-circle track to the fix, wind-crabbed so
        // the AP holds the GROUND track (the circuit's downwind lesson). A
        // to-altitude leg holds its own magnetic course.
        let psi_v = Some(psi);
        let crab = match (psi_v, hpath_r.as_ref().ok().copied().filter(|v| v.is_finite())) {
            (Some(psi), Some(hp)) => {
                crate::angles::signed_diff_deg(psi, hp).clamp(-25.0, 25.0)
            }
            _ => 0.0,
        };
        let dist_nm = bearing_to_fix(lat, lon, leg).map(|_| {
            let f = leg.fix.as_ref().unwrap();
            crate::nav::haversine_nm(lat, lon, f.lat, f.lon)
        });
        publish_fix_dist(dist_nm);
        // How far out this fix counts as reached: the turn onto the NEXT leg
        // has to begin there. Sized to the radius this airframe actually
        // turns at, because a fix inside that circle can never be closed on
        // — the 737's whole EIDW departure was inside it (2026-09-20).
        let gs_kts = gs * MS_TO_KTS;
        let capture_nm = capture_radius_nm(
            gs_kts,
            leg_turn_deg(lat, lon, leg, legs.get(idx + 1)).unwrap_or(0.0),
        );
        // A CF leg is flown ON its course, and an approach's IF behind us is
        // joined by a pattern, never homed on: EGKK 26L from WILLO turned
        // 150 degrees at CF26L and was handed to autoland 1,965 m off the
        // course tracking 044 (2026-09-27, `crate::course_leg`). Both give a
        // TRACK; the crab is added here as for every other leg.
        use crate::course_leg::{CourseLine, Join, Steer};
        let track_true = hpath_r.as_ref().ok().copied().filter(|v| v.is_finite()).unwrap_or(psi);
        let mut steer = Steer::Direct;
        let mut steer_track: Option<f64> = None;
        // The course line this tick is judged against, for the log.
        let mut steer_line: Option<CourseLine> = None;
        let join = if matches!(end, EndAction::Approach { .. }) {
            final_join.step(&legs, idx, lat, lon, track_true, gs_kts, mag_var, alt_ft, descent_fpm())
        } else {
            None
        };
        if matches!(join, Some(Join::Skip { .. }) | Some(Join::Pattern { .. })) {
            steer_line = legs.get(idx + 1).and_then(|l| CourseLine::of_leg(l, mag_var));
        }
        let line_pos = |line: &Option<CourseLine>| {
            line.as_ref().map(|l| (l.along_nm(lat, lon), l.xtk_nm(lat, lon)))
        };
        match join {
            Some(Join::Skip { why }) => {
                let (along, xtk) = line_pos(&steer_line).unzip();
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({"event": "final_join", "how": why, "idx": idx, "fix": leg.fix_ident,
                        "along_nm": along, "xtk_nm": xtk, "track": track_true}),
                );
                alert(&voice, Priority::Advisory, "Intercepting the final approach course.");
                idx += 1;
                heading_hold.reset();
                closure = None;
                continue;
            }
            Some(Join::Pattern { track_true: t, leg: pattern_leg, started, side }) => {
                if started {
                    let (along, xtk) = line_pos(&steer_line).unzip();
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "final_join", "how": "pattern", "idx": idx, "fix": leg.fix_ident,
                            "side": side, "along_nm": along, "xtk_nm": xtk, "track": track_true}),
                    );
                    alert(
                        &voice,
                        Priority::Advisory,
                        "We're on the far side of the final. A pattern to join it.",
                    );
                }
                steer = Steer::Pattern(pattern_leg);
                steer_track = Some(t);
            }
            _ => {}
        }
        // An RF leg round its arc, a hold as a racetrack (#101).
        let arc = if steer_track.is_none() { leg.arc() } else { None };
        let mut holding = false;
        let mut hold_end: Option<crate::course_leg::HoldEnd> = None;
        if let Some(a) = arc.as_ref() {
            steer_track = Some(crate::course_leg::arc_track(a, lat, lon));
            steer = Steer::Arc;
        } else if steer_track.is_none() && leg.is_hold() {
            if hold_seen != Some(idx) {
                hold_seen = Some(idx);
                hold = crate::course_leg::HoldFlight::for_leg(leg, mag_var).map(|h| (idx, h));
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({"event": if hold.is_some() { "hold_armed" } else { "hold_not_flown" },
                        "idx": idx, "fix": leg.fix_ident, "leg": leg.leg_type, "course": leg.course_mag,
                        "turn": leg.turn.map(String::from), "leg_min": leg.path.hold_leg_min,
                        "leg_nm": leg.dist_nm, "alt": leg.alt1_ft}),
                );
            }
            if let Some((_, h)) = hold.as_mut().filter(|(i, _)| *i == idx) {
                let s = h.step(leg, lat, lon, track_true, gs_kts, alt_ft, CAPTURE_NM, progress_s);
                if s.moved {
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "hold", "idx": idx, "fix": leg.fix_ident,
                            "phase": h.phase.word(), "circuits": h.circuits, "alt_ft": alt_ft}),
                    );
                    if h.phase == crate::course_leg::HoldPhase::TurnOut && h.circuits == 0 {
                        alert(
                            &voice,
                            Priority::Advisory,
                            &format!("Entering the hold at {}.", crate::tools::procedure::spell_ident(&leg.fix_ident)),
                        );
                    }
                }
                if let Some(e) = s.end {
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "hold_end", "idx": idx, "fix": leg.fix_ident,
                            "how": if e == crate::course_leg::HoldEnd::Flown { "flown" } else { "gave_up" },
                            "circuits": h.circuits, "alt_ft": alt_ft, "alt": leg.alt1_ft}),
                    );
                    if e == crate::course_leg::HoldEnd::GaveUp {
                        alert(
                            &voice,
                            Priority::Advisory,
                            &format!(
                                "We did not make {} feet in the hold at {}. Leaving it.",
                                leg.alt1_ft.map(|a| a.round() as i64).unwrap_or(0),
                                crate::tools::procedure::spell_ident(&leg.fix_ident)
                            ),
                        );
                    }
                }
                holding = true;
                hold_end = s.end;
                steer_track = Some(s.track_true);
                steer = s.steer;
            }
        }
        if steer_track.is_none() {
            let line = CourseLine::of_leg(leg, mag_var);
            if let Some((t, s)) = line.as_ref().and_then(|l| {
                crate::course_leg::course_track(l, lat, lon, track_true, gs_kts, steer_was == Steer::Track)
            }) {
                steer_track = Some(t);
                steer = s;
            }
            if line.is_some() {
                steer_line = line;
            }
        }
        let steer_pos = line_pos(&steer_line);
        if steer != steer_was {
            crate::blackbox::record(
                "procedure",
                serde_json::json!({"event": "steer", "idx": idx, "fix": leg.fix_ident, "steer": steer.word(),
                    "along_nm": steer_pos.map(|p| p.0), "xtk_nm": steer_pos.map(|p| p.1), "track": track_true}),
            );
        }
        steer_was = steer;
        let target_mag = if let Some(t) = steer_track {
            Some(true_to_mag(t + crab, mag_var))
        } else if let Some(brg_true) = bearing_to_fix(lat, lon, leg) {
            Some(true_to_mag(brg_true + crab, mag_var))
        } else {
            // course is already magnetic; add crab to hold it as a track.
            leg.course_mag.map(|c| (c + crab).rem_euclid(360.0))
        };
        // --- LNAV (#227): the FMS flies a plain leg to a fix once its active
        // leg IS this one; the bug flies everything else, and is the
        // fallback, said once, retried on the next leg. Read only where it
        // can matter (an eligible leg, or LNAV in play to hand back).
        let lnav_eligible = lnav_on
            && steer == Steer::Direct
            && steer_track.is_none()
            && steer_line.is_none()
            && arc.is_none()
            && !holding
            && !final_join.flying_pattern()
            && crate::lnav::leg_flyable(leg);
        // The EFB's "who has the lateral": a tap on the follower's own
        // decision, never a read.
        crate::efb::feed::note_lateral(lnav.owns_lateral());
        let mut fms_seen: Option<String> = None;
        if lnav_on && !lnav.retired() && (lnav_eligible || lnav.in_play()) {
            let now_s = started.elapsed().as_secs_f64();
            let (fms_r, xtk_r, engaged) = tokio::join!(
                xp.get_string(crate::datarefs::GPS_NAV_ID),
                xp.get_float(crate::datarefs::GPS_XTK),
                autoflight::read_lnav(&xp),
            );
            let fms = fms_r.ok();
            fms_seen = fms.clone();
            let xtk = xtk_r.ok().filter(|v| v.is_finite());
            // The capture's best, before a drop resets it (for the tape).
            let best_xtk = lnav.best_xtk();
            let d = lnav.decide(crate::lnav::Tick {
                now_s,
                idx,
                eligible: lnav_eligible,
                leg_ident: &leg.fix_ident,
                next_ident: legs.get(idx + 1).map(|l| l.fix_ident.as_str()),
                fms_ident: fms.as_deref(),
                engaged,
                xtk_nm: xtk,
                gs_kts,
            });
            match d.note {
                Some(crate::lnav::Note::Engaged {
                    say,
                    reengage,
                    presses,
                }) => {
                    crate::blackbox::record(
                        "lnav",
                        serde_json::json!({"event": if reengage { "reengaged" } else { "engaged" },
                            "idx": idx, "fix": leg.fix_ident, "fms": fms, "xtk": xtk, "presses": presses}),
                    );
                    if say {
                        alert(&voice, Priority::Advisory, crate::lnav::ENGAGE_LINE);
                    }
                }
                Some(crate::lnav::Note::Captured { secs }) => crate::blackbox::record(
                    "lnav",
                    serde_json::json!({"event": "captured", "idx": idx, "fix": leg.fix_ident,
                        "xtk": xtk, "secs": secs}),
                ),
                None => {}
            }
            match d.act {
                crate::lnav::Act::Nothing => {}
                crate::lnav::Act::Press => {
                    let sent = autoflight::press_lnav(&xp, "procedure_lnav").await;
                    crate::blackbox::record(
                        "lnav",
                        serde_json::json!({"event": "engage", "idx": idx, "fix": leg.fix_ident,
                            "fms": fms, "xtk": xtk, "press": lnav.presses(), "sent": sent}),
                    );
                }
                crate::lnav::Act::ReGoto => {
                    crate::blackbox::record(
                        "lnav",
                        serde_json::json!({"event": "regoto", "idx": idx, "fix": leg.fix_ident,
                            "fms": fms, "owned": lnav.owns_lateral()}),
                    );
                    sequence_fms(&xp, idx, leg).await;
                }
                crate::lnav::Act::DirectTo => {
                    let r =
                        crate::tools::direct_to::execute(&xp, &leg.fix_ident, true, "procedure")
                            .await;
                    crate::blackbox::record(
                        "lnav",
                        serde_json::json!({"event": "direct_to", "idx": idx, "fix": leg.fix_ident,
                            "fms": fms, "xtk": xtk, "executed": r.is_ok(), "refused": r.err()}),
                    );
                }
                crate::lnav::Act::Drop(why) => {
                    crate::blackbox::record(
                        "lnav",
                        serde_json::json!({"event": "dropped", "why": why.word(), "idx": idx,
                            "fix": leg.fix_ident, "fms": fms, "xtk": xtk, "status": engaged,
                            "best_xtk": best_xtk, "fallbacks": lnav.fallbacks()}),
                    );
                    let (hdg, taken) = lnav_hand_back(&xp, psi, mag_var).await;
                    bug_mag = hdg;
                    heading_hold.reset();
                    crate::blackbox::record(
                        "lnav",
                        serde_json::json!({"event": "fallback", "idx": idx, "fix": leg.fix_ident,
                            "hdg": hdg, "heading_on": taken}),
                    );
                    alert(&voice, Priority::Caution, crate::lnav::FALLBACK_LINE);
                    if lnav.retired() {
                        crate::blackbox::record(
                            "lnav",
                            serde_json::json!({"event": "retired", "fallbacks": lnav.fallbacks()}),
                        );
                    }
                }
                crate::lnav::Act::HandBack(why) => {
                    let (hdg, taken) = lnav_hand_back(&xp, psi, mag_var).await;
                    bug_mag = hdg;
                    heading_hold.reset();
                    crate::blackbox::record(
                        "lnav",
                        serde_json::json!({"event": "handback", "why": why, "idx": idx,
                            "fix": leg.fix_ident, "fms": fms, "hdg": hdg, "heading_on": taken}),
                    );
                }
            }
        }
        // Why the bug and not LNAV flies this leg, on each change.
        let steering = if steer != Steer::Direct || steer_track.is_some() || steer_line.is_some() {
            Some("course_leg")
        } else if arc.is_some() {
            Some("arc")
        } else if holding {
            Some("holding")
        } else if final_join.flying_pattern() {
            Some("pattern")
        } else {
            None
        };
        let why = crate::lnav::bug_reason(&crate::lnav::BugWhy {
            owned: lnav.owns_lateral(),
            declared: autoflight::panel().lnav_declared(),
            enroute: matches!(end, EndAction::Enroute),
            retired: lnav.retired(),
            steering,
            leg,
            fms_ident: fms_seen.as_deref(),
        });
        if let Some(w) = why.filter(|w| bug_why_said != Some((idx, *w))) {
            bug_why_said = Some((idx, w));
            crate::blackbox::record(
                "lnav",
                serde_json::json!({"event": "on_bug", "why": w, "idx": idx,
                    "fix": leg.fix_ident, "type": leg.leg_type, "fms": fms_seen}),
            );
        } else if why.is_none() {
            bug_why_said = None;
        }
        // A course leg flown on its course is done abeam its fix, as a
        // fly-by is: the intercept may still be closing the line.
        // An arc is done at its end radial; a hold when it says so.
        let done = if let Some(a) = arc.as_ref() {
            crate::course_leg::arc_done(a, lat, lon, CAPTURE_NM)
        } else if holding {
            hold_end.is_some()
        } else {
            leg_done(leg, dist_nm, alt_ft, capture_nm, lat, lon)
                || (matches!(steer, Steer::Intercept | Steer::Track)
                    && steer_line
                        .as_ref()
                        .is_some_and(|l| crate::course_leg::course_passed(l, lat, lon, capture_nm)))
        };
        // The closure watchdog judges a leg flown AT its fix; a pattern flies
        // away from the IF on purpose, and so does a hold's outbound leg.
        if let Some(d) = dist_nm.filter(|_| !done && !final_join.flying_pattern() && !holding) {
            if closure.as_ref().is_none_or(|(i, _)| *i != idx) {
                // A new leg is judged on its own: stalls on an earlier leg
                // that was later flown are not this one's (LIPK 2026-09-25:
                // two at the start of the 51 nm leg to ESFOK, thirty minutes
                // later a third on FI30 stood the approach down). And the
                // turn onto it is given its time before any verdict.
                if closure.is_some() {
                    stalls = 0;
                }
                let turn = bearing_to_fix(lat, lon, leg)
                    .map(|b| crate::angles::abs_diff_deg(b, psi))
                    .unwrap_or(0.0);
                let grace = leg_turn_grace_s(turn);
                closure = Some((
                    idx,
                    crate::circuit::Closure::with_grace(d * 1852.0, progress_s, grace),
                ));
            }
            if closure.as_mut().unwrap().1.stalled(d * 1852.0, progress_s) {
                stalls += 1;
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({
                        "event": "leg_stalled", "idx": idx, "dist_nm": d, "stalls": stalls,
                        "cap_nm": capture_nm,
                    }),
                );
                // LNAV out first: the bug and HDG SEL are the rejoin's.
                if lnav.drop_now(idx, started.elapsed().as_secs_f64()) {
                    let (_, taken) = lnav_hand_back(&xp, psi, mag_var).await;
                    crate::blackbox::record(
                        "lnav",
                        serde_json::json!({"event": "dropped", "why": crate::lnav::Why::LegStalled.word(),
                            "idx": idx, "fix": leg.fix_ident, "heading_on": taken,
                            "fallbacks": lnav.fallbacks()}),
                    );
                    alert(&voice, Priority::Caution, crate::lnav::FALLBACK_LINE);
                }
                let heading = true_to_mag(psi, mag_var);
                if autoflight::set_dial(&xp, Dial::HeadingBug, heading).await.is_ok() {
                    bug_mag = heading;
                }
                heading_hold.reset();
                if stalls < 3 {
                    alert(
                        &voice,
                        Priority::Caution,
                        "Not closing on the next fix. Rejoining.",
                    );
                    continue;
                }
                let _ = autoflight::set_dial(&xp, Dial::Vs, 0.0).await;
                stand_down(
                    &voice,
                    "Not closing on the next fix. Your aircraft.",
                    "leg_stalled",
                );
                return;
            }
        }
        // One writer: never the bug while LNAV flies the lateral.
        if let Some(t) = target_mag.filter(|_| lnav.bug_write_allowed()) {
            let now_s = started.elapsed().as_secs_f64();
            // Keyed by the phase too: a new intercept, course or pattern leg
            // is set at once, corrections inside one wait (`HeadingHold`).
            if let Some(h) = heading_hold.decide(steer.key(idx), t, true_to_mag(psi, mag_var), phi, now_s) {
                if autoflight::set_dial(&xp, Dial::HeadingBug, h).await.is_ok() {
                    bug_mag = h;
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "heading_set", "idx": idx, "hdg": h, "crab": crab}),
                    );
                } else {
                    heading_hold.reset();
                }
            }
        }
        // The lateral mode must actually be ON for the bug to mean anything —
        // the circuit's guard, which this follower LACKED: an AP disconnect
        // mid-climb left the servos off with only the VS re-engage watching,
        // and the aircraft spiralled at 20°/s while the loop slewed a bug
        // nobody was following (LEJR enroute, 2026-07-24).
        // The servos through the one engage (`ap_engage::ServoWatch`: FD
        // first, re-pressed until they read in, `ap_engage` / `ap_engaged`).
        let st = servo_watch.step(
            started.elapsed().as_secs_f64(),
            hdg_st <= 0.5,
            servos_r.as_ref().ok().copied(),
            ias,
        );
        if st.fire {
            crate::ap_engage::press_logged(&xp, &ap_fd, "procedure", "lateral", st.try_n)
                .await;
        }
        if st.confirmed_now {
            crate::ap_engage::record_engaged("procedure", "lateral", st.try_n);
        }
        // Not over LNAV, its engage, or a hand-back's HDG SEL settling.
        if lnav.heading_mode_wanted(started.elapsed().as_secs_f64())
            && autoflight::bring_heading(&xp, Some(hdg_st), "procedure_lateral")
                .await
                .pressed(Button::Heading)
        {
            crate::blackbox::record(
                "procedure",
                serde_json::json!({"event": "reengage_lateral", "idx": idx}),
            );
        }

        // --- Vertical: fly toward the leg's constraint (carried across
        // unconstrained legs) with the VS law — proportional rate, clamped,
        // mode re-engaged when dropped (the takeoff's recipe) — and the
        // preselect dialled to the same target so the sim's armed capture
        // levels off AT the constraint instead of a stale value.
        // A DESCENT waits for its top-of-descent instead of starting the
        // moment the leg arms — the constraint is at the FIX, not here.

        // --- Terrain: probe the ground AHEAD along the track and never command
        // an altitude into it. Skipped on an approach, whose own procedure owns
        // the descent below the enroute floor (and whose platform altitudes
        // would otherwise be lifted out of the glideslope by this).
        let on_approach = matches!(end, EndAction::Approach { .. });
        if terrain_available
            && !on_approach
            && last_probe.elapsed().as_secs_f64() >= TERRAIN_PROBE_SECS
        {
            last_probe = std::time::Instant::now();
            // Look along the track we are STEERING, out to the fix at most: near
            // a field the answer should be about the field, not the hills past it.
            let brg = bearing_to_fix(lat, lon, leg)
                .or(psi_v)
                .unwrap_or_else(|| mag_to_true(bug_mag, mag_var));
            let look = dist_nm
                .map(|d| (d + 2.0).min(TERRAIN_LOOK_NM))
                .unwrap_or(TERRAIN_LOOK_NM);
            let pts = crate::terrain::look_ahead_points(lat, lon, brg, look, TERRAIN_POINTS);
            match crate::terrain::max_elevation_ft(&xp, &pts).await {
                Some(ground) => {
                    terrain_misses = 0;
                    crate::terrain::publish_floor_ahead(Some(ground + crate::terrain::CLEARANCE_FT))
                }
                None if !terrain_probe_fatal(
                    &mut terrain_misses,
                    crate::xplane::sim_frozen(&xp).await,
                ) =>
                {
                    // A probe the sim never answered because it was FROZEN is
                    // no verdict on the plugin: the pilot paused 0.6 s after
                    // the query went out, the two-second wait ran into the
                    // pause, and the floor was switched off for the whole
                    // LXGB-LEAX flight (2026-09-15). Deferred; only two live
                    // misses in a row retire it.
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "terrain_probe_deferred", "misses": terrain_misses}),
                    );
                }
                None => {
                    // Said aloud: a floor that retires in silence is a
                    // safety feature the pilot believes is still on (two
                    // days of flights without one, 2026-09-16).
                    terrain_available = false;
                    crate::terrain::publish_floor_ahead(None);
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "terrain_probe_unavailable"}),
                    );
                    alert(
                        &voice,
                        Priority::Caution,
                        "The terrain probe isn't answering. No terrain floor for the rest of this flight; I'll hold the plan's altitudes. Check the plugin build.",
                    );
                }
            }
        }
        // --- The pilot's altitude outranks the plan's, the ground outranks
        // both. The autopilot tool hands a commanded altitude to the follower
        // rather than writing the dial, so the follower stays the one writer
        // and an override cannot be undone by the next tick.
        // The STAR's "at or below"s ahead, kept reachable: this fix's target
        // comes down to whatever still makes the tightest one further on.
        let star_leg;
        let (leg_for_vertical, arrival_ceiling) = match ceilinged_leg_alt(
            &legs,
            &arrival_constraints,
            idx,
            gs_kts,
            descent_fpm(),
            &mut ceiling_latch,
        ) {
            Some((alt, c)) => {
                if star_ceiling_logged != Some(idx) {
                    star_ceiling_logged = Some(idx);
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "star_ceiling", "idx": idx, "fix": leg.fix_ident,
                                           "plan_ft": leg.alt1_ft, "ceiling_ft": c}),
                    );
                }
                let mut l = leg.clone();
                l.alt1_ft = Some(alt);
                star_leg = l;
                (&star_leg, true)
            }
            None => (leg, false),
        };
        // Round a pattern the descent is paced on the pattern's miles, not
        // the straight line to an IF it is flying away from (#156); an IF
        // skipped for the intercept but still ahead along the final keeps
        // its altitude on the miles to it (#161).
        let pattern_nm = final_join.pattern_to_go_nm(&legs, idx, lat, lon, gs_kts, mag_var);
        let owed_if = final_join.skipped_if_ahead_nm(&legs, idx, lat, lon, mag_var);
        let leg_for_vertical = owed_if.map_or(leg_for_vertical, |(i, _)| &legs[i]);
        let v = vertical_tick(
            leg_for_vertical,
            carry_alt,
            prev_cmd_alt,
            alt_ft,
            owed_if.map(|(_, d)| d).or(pattern_nm).or(dist_nm),
            gs_kts,
            crate::pilot_altitude::held(),
            crate::terrain::floor_ahead_ft().map(|f| floor_indicated(f, true_alt_ft, alt_ft)),
            descent_fpm(),
            DescentProfile {
                cruise_ft: descent_cruise_ft,
                peak_ft: plan_peak_ft,
                past_top: plan_top.is_some_and(|t| idx > t),
                arrival_ceiling,
            },
        );
        plan_peak_ft = v.peak_ft;
        // A planned step climb begins on this leg: said once, as a crew
        // says it, and only when it is the plan's level being flown (not a
        // pilot's, not the terrain floor's).
        if let Some((fix, from_ft, to_ft)) = step_climb_at(&legs, idx, &cruise_schedule) {
            if !v.pilot && v.floored.is_none() && said_steps.insert(fix.clone()) {
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({"event": "step_climb", "idx": idx, "fix": fix,
                                       "from_ft": from_ft, "to_ft": to_ft, "alt_ft": alt_ft}),
                );
                alert(
                    &voice,
                    Priority::Advisory,
                    &format!("Step climb, {}.", atc::atis::spoken_flight_level(to_ft)),
                );
            }
        }
        if let Some((plan_ft, held_ft)) = v.climb_held {
            if climb_held_logged != Some(idx) {
                climb_held_logged = Some(idx);
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({"event": "climb_no_descent", "idx": idx, "fix": leg.fix_ident,
                                       "plan_ft": plan_ft, "held_ft": held_ft, "alt_ft": alt_ft}),
                );
            }
        }
        if let Some((plan_ft, held_ft)) = v.descent_held {
            if descent_held_logged != Some(idx) {
                descent_held_logged = Some(idx);
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({"event": "descent_no_climb", "idx": idx, "fix": leg.fix_ident,
                                       "plan_ft": plan_ft, "held_ft": held_ft, "alt_ft": alt_ft}),
                );
            }
        }
        // The leg to a MAP flies the published path, rate fed forward
        // (`map_path`); every other leg the constraint law above.
        let path = map_path(leg, dist_nm, gs_kts);
        let want_alt = path.map_or(v.want_alt, |(a, _)| a);
        // A departure minimum ahead is a FLOOR on the target until its fix
        // is flown, not only a reason to climb faster: a plan predicting
        // 3,000 ft at RIM against the chart's 4,000 levelled at 3,000 and the
        // minimum watch then called "not climbing" and handed the aircraft
        // back; a minimum dropped the moment it was met hunted round it
        // (EGKK D156J, #153). Minimums above the cruise were dropped at the
        // start (`minimums_within_cruise`). Then an IFR clearance's altitude
        // caps the climb, the minimum's included, until departure clears
        // higher (`departure_target_ft`).
        let ceiling_now = cleared_ceiling();
        let minimum = DepartureMinimums::floor(&climb_minimums, &legs, idx).filter(|_| path.is_none());
        let dep = departure_target_ft(want_alt, minimum.map(|m| m.1), ceiling_now, v.pilot, v.floored);
        let want_alt = dep.want_ft;
        if dep.minimum_capped {
            if let (Some((at, min_ft)), Some(c)) = (minimum, ceiling_now) {
                let fix = legs[at].fix_ident.clone();
                if said_min_capped.insert(fix.to_ascii_uppercase()) {
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "minimum_capped", "fix": fix, "min_ft": min_ft,
                                           "ceiling_ft": c, "alt_ft": alt_ft}),
                    );
                    alert(
                        &voice,
                        Priority::Advisory,
                        &format!(
                            "{fix} needs {}, we're cleared to {}. Holding it until departure clears us higher.",
                            crate::flightplan::spoken_feet(min_ft),
                            crate::flightplan::spoken_feet(c),
                        ),
                    );
                }
            }
        }
        // The clearance let go of a target it was holding down: the climb
        // on is said, once, as the dial moves.
        if let Some(c) = cap_holding {
            if ceiling_now.is_none() && want_alt > c + STEP_FT {
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({"event": "cleared_climb", "from_ft": c, "to_ft": want_alt, "alt_ft": alt_ft}),
                );
                alert(
                    &voice,
                    Priority::Advisory,
                    &format!("Cleared higher. Climbing to {}.", crate::flightplan::spoken_feet(want_alt)),
                );
            }
        }
        cap_holding = if dep.cap_limited { ceiling_now } else { None };
        carry_alt = v.carry;
        // A minimum's level is COMMANDED: the climb's hold on a plan
        // altitude below the level commanded (`climb_no_descent`) keeps it
        // after its fix, instead of the plan's navlog 5,000 pulling the
        // aircraft back down from 6,000.
        prev_cmd_alt = if dep.raised_by_minimum { v.prev_cmd.max(want_alt) } else { v.prev_cmd };
        // The follower's own top of descent, for the advisor and the spoken
        // answer: the same point the law above paces against. `carry` is the
        // leg's constraint (carried across unconstrained fixes).
        // No top of descent on an approach: its descent is the procedure's
        // platform altitudes and path, and "descent coming up, down to 200
        // feet" was said four times down the LIPE 12 ILS (2026-09-25).
        let tod = dist_nm
            .filter(|_| !on_approach && path.is_none())
            .and_then(|d| {
                descent_point_nm(v.carry, alt_ft, gs_kts, descent_fpm()).map(|tod| PlannedTod {
                    fix: leg.fix_ident.clone(),
                    dist_nm: d,
                    to_tod_nm: d - tod,
                    target_ft: v.carry,
                    started: want_alt < alt_ft - 100.0,
                    held_ft: None,
                })
            });
        let pilot_holds = v.pilot && want_alt > v.carry + 100.0;
        publish_tod(held_tod(tod, pilot_holds.then_some(want_alt), &mut held_leg));
        match v.floored {
            Some(floor) => {
                if said_floor.is_none_or(|p| floor > p + TERRAIN_RESAY_FT) {
                    said_floor = Some(floor);
                    alert(
                        &voice,
                        Priority::Caution,
                        &format!(
                            "Terrain ahead, holding {} feet.",
                            crate::auto_status::commas((floor / 100.0).ceil() as i64 * 100)
                        ),
                    );
                }
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({"event": "terrain_floor", "idx": idx, "floor": floor,
                                       "alt_ft": alt_ft, "agl_ft": agl_ft, "dist_nm": dist_nm}),
                );
            }
            // Ground fell away: a later rise is news again.
            None => {
                if said_floor.is_some_and(|p| {
                    crate::terrain::floor_ahead_ft().is_none_or(|f| f < p - TERRAIN_RESAY_FT)
                }) {
                    said_floor = None;
                }
            }
        }
        // The dial is the level this movement ends at, set once
        // (`dial_target_ft`); the path itself is flown on the VS dial below.
        // A terrain floor or a pilot's level is a level the dial must not
        // go under.
        let held = if v.pilot || v.floored.is_some() { Some(want_alt) } else { None };
        // The target's own rate only while the plan's path IS the target
        // (the VS law's feed-forward rule below).
        let path_fpm = if (want_alt - v.want_alt).abs() < 1.0 { v.target_fpm } else { 0.0 };
        let dial = dial_target_ft(
            want_alt,
            alt_ft,
            v.carry,
            held,
            path.and(leg.alt1_ft),
            path_fpm,
        );
        if !autoflight::set_dial_confirmed(
            "procedure",
            &xp,
            Dial::Altitude,
            dial,
            &mut last_alt_dial,
            50.0,
        )
        .await
        {
            continue;
        }
        // Speed-protected: VS mode holds RATE whatever the airspeed, so the
        // climb rate backs off as IAS sags below Vy (the takeoff's own law,
        // floored positive) — the unprotected 700 fpm stalled the LEJR climb.
        // Both limbs are the AIRFRAME's now. The shared 700 was a GA figure
        // and a 1.1 degree path at jet speeds: the BIKF arrival needed about
        // 2,000 fpm on every leg inbound of BASLU and would have arrived
        // 17,000 ft high, while the same cap on the climb made FL340 a
        // 37-minute job burning low-altitude fuel (2026-09-20). The descent
        // rate is the SAME one that placed the top of descent, so the point
        // and the rate can never disagree.
        let now_ias = std::time::Instant::now();
        let ias_trend = prev_ias.map_or(0.0, |(p, t0): (f64, std::time::Instant)| {
            let dt = now_ias.duration_since(t0).as_secs_f64();
            if (0.2..=5.0).contains(&dt) {
                (ias - p) / dt
            } else {
                0.0
            }
        });
        prev_ias = Some((ias, now_ias));
        // Smoothed for the descent's speed protection (the tick-to-tick
        // difference of a half-second tick is mostly the ASI's noise).
        ias_trend_f += (ias_trend - ias_trend_f) * (dt / (IAS_TREND_TAU_S + dt)).clamp(0.0, 1.0);
        let climb_cap = if minimums_state.unmet(&climb_minimums, &legs, idx, alt_ft).is_some() {
            best_climb_fpm()
        } else {
            climb_fpm()
        };
        let mut vs_cmd = match path {
            Some((path_ft, ff_fpm)) => (ff_fpm + (path_ft - alt_ft) * 2.0)
                .clamp(-descent_fpm().max(ff_fpm.abs() * 1.4), 300.0),
            None => {
                // The feed-forward only while the plan's path IS the target
                // (a departure minimum or a MAP path is not it).
                let ff = if (want_alt - v.want_alt).abs() < 1.0 { v.target_fpm } else { 0.0 };
                vs_law_fpm(want_alt, alt_ft, ff, descent_fpm(), climb_cap)
            }
        };
        if vs_cmd > 0.0 && vy_kts > 0.0 {
            vs_cmd = vs_cmd.min(crate::autotakeoff::climb_vs_fpm(ias, vy_kts, vs_cmd));
            // ...and the circuit's own fade to NOTHING at the airframe's speed
            // floor. The takeoff law above takes 60 fpm a knot off its cap and
            // never goes under 150, so against the SF50's 1,500 fpm cap it
            // still asked about 1,000 fpm at 104 kt and FL260, 16 kt under Vy
            // and falling (EDLP-LOWI, 2026-09-24). A climb is never worth more
            // than the airspeed left.
            if let Some(floor) = crate::aircraft::speed_floor_kts() {
                vs_cmd = crate::aircraft::climb_limit_fpm(vs_cmd, ias, ias_trend, vy_kts, floor);
            }
        }
        if vs_cmd < -DRAG_MIN_DESCENT_FPM {
            // Down from cruise: the destination's transition level rules.
            crate::tools::instruments::note_descending();
        }
        // The descent flies the SPEED first (`speed_protected_vs`): a descent
        // that would outrun the governor gives way before the speed does,
        // and the height it leaves behind is the speedbrake's
        // (`descent_drag`). A published MAP path keeps its own rate.
        let vs_want = vs_cmd;
        let target_kts = speed_owed.or_else(crate::tools::speedhold::flight_target);
        if path.is_none() {
            vs_cmd = speed_protected_vs(vs_cmd, target_kts.map(|t| ias - t), ias_trend_f);
            idle_descent.step(vs_want);
        } else {
            idle_descent.step(0.0);
        }
        // The next STAR window, judged from here: an "at or below" at the
        // airframe's best descent, an "at or above" on the climb this loop is
        // FLYING (`window_climb_fpm`). One that cannot be made is said once
        // (#32). Not while the pilot or a clearance holds the altitude
        // (their level, their restrictions).
        let window_climb = window_climb_fpm(vs_cmd, dep.cap_limited);
        if let (false, true, Some(climb)) = (
            arrival_constraints.is_empty(),
            crate::pilot_altitude::held().is_none(),
            window_climb,
        ) {
            if let Some(miss) = star_window_miss(
                &legs,
                &arrival_constraints,
                idx,
                lat,
                lon,
                alt_ft,
                gs_kts,
                climb,
                descent_fpm(),
                star_cruise_ft,
            ) {
                if said_unable_windows.insert(miss.fix.to_ascii_uppercase()) {
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "star_window_unable", "fix": miss.fix,
                            "low": miss.low, "limit_ft": miss.limit_ft,
                            "predicted_ft": miss.predicted_ft, "by_ft": miss.by_ft,
                            "dist_nm": miss.dist_nm, "alt_ft": alt_ft, "climb_fpm": climb}),
                    );
                    alert(&voice, Priority::Advisory, &miss.line());
                }
            }
        }
        {
            let ours = DRAG_OURS.load(Ordering::SeqCst);
            let held_s = drag_changed.map_or(f64::INFINITY, |c: std::time::Instant| c.elapsed().as_secs_f64());
            let want_out = descent_drag(ours, held_s, ias, target_kts, vs_want, alt_ft - want_alt);
            let has_speedbrake = crate::takeoff_config::has_speedbrake();
            // Never with flap or gear (`drag_barred`), read only while the
            // drag is out or wanted. A failed read is no measurement: a brake
            // of ours stays as the speed rule says, one not out stays in.
            let barred = if has_speedbrake && (ours || want_out) {
                let (flap_r, gear_r) =
                    tokio::join!(xp.get_float(FLAP_RATIO), xp.get_float(GEAR_HANDLE));
                match (flap_r, gear_r) {
                    (Ok(f), Ok(g)) => drag_barred(f, g > 0.5, &crate::aircraft::flap_limits()),
                    _ if ours => None,
                    _ => Some("unread"),
                }
            } else {
                None
            };
            let want_out = want_out && barred.is_none();
            if want_out != ours && has_speedbrake {
                // Read first: a lever already out is the pilot's, and an
                // ARMED one (negative) is stowed-and-armed, ours to use.
                let at = crate::tools::controls::speedbrake_read(&xp).await.ok();
                let write = match (want_out, at) {
                    (true, Some(v)) if v <= 0.05 => Some(SPEEDBRAKE_FLIGHT),
                    (false, Some(_)) => Some(0.0),
                    _ => None,
                };
                if let Some(v) = write {
                    if crate::tools::controls::speedbrake_set(&xp, v).await.is_ok() {
                        DRAG_OURS.store(want_out, Ordering::SeqCst);
                        drag_changed = Some(std::time::Instant::now());
                        let said = drag_voice.say(want_out, drag_clock.elapsed().as_secs_f64());
                        crate::blackbox::record(
                            "procedure",
                            serde_json::json!({"event": "speedbrake", "out": want_out,
                                "ias": ias, "target": target_kts,
                                "vs_cmd": vs_cmd, "vs_want": vs_want, "high_ft": alt_ft - want_alt,
                                "held_s": held_s.is_finite().then_some(held_s),
                                "said": said,
                                "why": if want_out { "speed" } else { barred.unwrap_or("speed") }}),
                        );
                        if said {
                            alert(
                                &voice,
                                Priority::Advisory,
                                if want_out {
                                    "Speedbrake."
                                } else {
                                    "Speedbrake in."
                                },
                            );
                        }
                    }
                }
            }
        }
        // The modes are read before the VS wheel is touched (`vs_action`):
        // VS mode gone NEAR the target, or an ALT hold closing on OUR dial,
        // is the autopilot's own level-off, left flying and its wheel left
        // alone (a VS dial written under ALT HOLD reads back 0 and threw the
        // tick away, EGKK 2026-09-28). Far from the target it's a dropped
        // mode: re-engage, read the mode again, and only then the wheel.
        let mut action = if vs_st > 0.5 {
            VsAction::Write
        } else {
            // A failed read of the hold is no measurement: not holding.
            let alt_hold_on = crate::autoflight::read_status(&xp, ALT_HOLD_STATUS).await.is_ok_and(|v| v > 1.5);
            vs_action(false, alt_hold_on, alt_ft, want_alt, dial, vs_cmd, climb_rate_fpm)
        };
        if action == VsAction::Engage {
            last_vs_cmd = None; // engage can sync the dial even if the response is lost
            if !autoflight::bring_vs(&xp, Some(vs_st), "procedure_vs")
                .await
                .sent
            {
                continue;
            }
            if crate::autoflight::read_status(&xp, VS_STATUS).await.is_ok_and(|v| v > 0.5) {
                action = VsAction::Write;
                // A vertical mode engaged: its pitch authority is checked
                // afresh (#181).
                if let Some(vs) = climb_watch.vs_fpm() {
                    crate::climb_watch::arm_authority("procedure", vs, vs_cmd);
                }
            }
        }
        if vs_action_logged != Some(action) {
            vs_action_logged = Some(action);
            crate::blackbox::record(
                "procedure",
                serde_json::json!({"event": "vs_mode", "action": format!("{action:?}").to_lowercase(),
                                   "alt_ft": alt_ft, "want_alt": want_alt, "dial": dial, "vs_cmd": vs_cmd}),
            );
        }
        // The autopilot's answer, WATCHED (#181): the climb far over the
        // wheel with the speed going, or one still on under the floor, is
        // sent down and taken if it does not come down. The flap is read
        // only under the clean floor (its configuration's floor then).
        let clean_floor = crate::aircraft::speed_floor_kts();
        let watch_floor = match clean_floor {
            Some(f) if ias < f => match xp.get_float(FLAP_RATIO).await {
                Ok(flap) => crate::aircraft::config_floor_kts(flap).or(Some(f)),
                Err(_) => Some(f),
            },
            other => other,
        };
        // The rate is the aircraft's, so the TRUE altitude: the altimeter
        // steps when its setting changes (30.32 to 29.92 at EGKK's
        // transition read as -1,681 fpm in a 2,100 fpm climb and handed a
        // climbing A330 to the recovery, 2026-10-14).
        let verdict = climb_watch.step(
            watch_t0.elapsed().as_secs_f64(),
            true_alt_ft,
            ias,
            watch_floor,
            (action == VsAction::Write).then_some(vs_cmd),
        );
        if verdict != last_verdict {
            last_verdict = verdict;
            crate::blackbox::record(
                "procedure",
                serde_json::json!({"event": "climb_watch", "verdict": format!("{verdict:?}"),
                                   "ias": ias, "vs": climb_watch.vs_fpm(), "vs_cmd": vs_cmd,
                                   "floor": watch_floor, "alt_ft": alt_ft}),
            );
        }
        // The autopilot's pitch AUTHORITY after an engage (#181): a
        // takeoff's carried into the follower, or the follower's own.
        let authority = crate::climb_watch::feed_authority(
            climb_watch.vs_fpm(),
            (action == VsAction::Write).then_some(vs_cmd),
        );
        let take = match (verdict, authority) {
            (_, Some(crate::climb_watch::Authority::NoAuthority(_))) => {
                Some(crate::climb_watch::Why::NoAuthority)
            }
            (crate::climb_watch::Verdict::TakeOver(why), _) => Some(why),
            _ => None,
        };
        if let Some(why) = take {
            let hold_kts = target_kts;
            let no_pitch = why.ap_has_no_pitch();
            // The recovery says the missing pitch itself, first thing.
            let line = if no_pitch {
                "I have control.".to_string()
            } else {
                format!("{} I have control, recovering.", why.line())
            };
            stand_down(
                &voice,
                &line,
                if no_pitch { "no_pitch_authority" } else { "climb_not_held" },
            );
            crate::tools::recover::take_over(
                base_url.clone(),
                voice.clone(),
                crate::tools::recover::TakeOver {
                    why: format!("procedure_{}", why.word()),
                    min_safe_ias: watch_floor
                        .map(|f| f + crate::circuit::RECOVERY_MARGIN_KTS),
                    hold_kts,
                    ap_no_pitch: no_pitch,
                    hold_ft: None,
                },
            );
            return;
        }
        let vs_cmd = crate::climb_watch::dial_for(verdict, vs_cmd);
        // The VS wheel in the 100 fpm clicks an autopilot has, moved only
        // when the click changes: the law's fine value every tick was the
        // same twitching the heading bug did (2026-09-26).
        let vs_cmd = vs_dial_fpm(vs_cmd);
        if action == VsAction::Write {
            if !autoflight::set_dial_confirmed("procedure", &xp, Dial::Vs, vs_cmd, &mut last_vs_cmd, 50.0)
                .await
            {
                continue;
            }
        } else {
            // Unwritten: the next VS engage writes it afresh.
            last_vs_cmd = None;
        }
        let owner = if v.floored.is_some() {
            AltOwner::Terrain
        } else if v.pilot {
            AltOwner::Pilot
        } else {
            AltOwner::Plan
        };
        DETAIL.set(leg_detail(
            &leg.fix_ident,
            dist_nm,
            gs_kts,
            alt_ft,
            want_alt,
            owner,
        ));
        crate::auto_status::set_ap_targets(Some(bug_mag), Some(want_alt), Some(vs_cmd));

        crate::blackbox::record(
            "procedure",
            serde_json::json!({
                "event": "leg", "idx": idx, "type": leg.leg_type, "fix": leg.fix_ident,
                "dist_nm": dist_nm, "alt_ft": alt_ft, "alt_true": true_alt_ft, "want_alt": want_alt,
                // The vertical as commanded: the law's want, the speed's
                // hold on it (`speed_protected_vs`), the wheel.
                "vs_want": vs_want, "vs_cmd": vs_cmd, "ias": ias, "spd_tgt": target_kts,
                // The FAF speed the approach slows to (`faf_speed_cap`).
                "faf_join": faf_now.map(|f| f.join_kts), "faf_begun": faf_begun,
                "agl_ft": agl_ft, "crab": crab, "psi": psi_v, "bug": bug_mag, "tgt": target_mag,
                // How the lateral is flown and where on the course it is
                // (`course_leg`): the FAF's line during a final's join.
                "steer": steer.word(), "along_nm": steer_pos.map(|p| p.0), "xtk_nm": steer_pos.map(|p| p.1), "pattern_nm": pattern_nm,
                // A skipped IF still ahead, its miles (#161).
                "if_owed_nm": owed_if.map(|o| o.1),
                // The capture radius in force, so a fix that will not close
                // can be told from one the aircraft simply has not reached.
                "cap_nm": capture_nm,
            }),
        );

        // --- Approach handoff: at the FAF leg, once captured, autoland owns it.
        if let (
            EndAction::Approach {
                apt_dat,
                cfg,
                approach_ias,
                rwy_id,
            },
            Some(hi),
        ) = (&end, handoff)
        {
            if idx >= hi && done {
                crate::blackbox::record(
                    "procedure",
                    // The FAF speed planned for, and the gate the energy
                    // model predicts from the speed handed over (autoland's
                    // `e_gate`): over the gate speed here, the final is lost
                    // before autoland has it.
                    serde_json::json!({"event": "handoff_autoland", "idx": idx, "agl_ft": agl_ft,
                        "ias": ias, "faf_join": faf_now.map(|f| f.join_kts),
                        "e_gate": faf_model.as_ref().zip(faf_now).map(|((m, gate_kts), f)|
                            crate::approach_energy::predicted_gate_kts(m, *gate_kts, ias, f.path_nm, f.path_nm)),
                        "gate_kts": faf_model.as_ref().map(|(_, g)| *g)}),
                );
                if map_short_of_runway(leg) && !MAP_AUTOLAND {
                    // The visual segment is the pilot's, talked down: the
                    // aircraft configured and on speed, the autopilot still
                    // holding the descent, the talk-down armed on THIS path
                    // (from here to 50 ft over the threshold, not 3 degrees).
                    let rwy = match rwy_id.clone() {
                        Some(r) => {
                            let apt = apt_dat.clone();
                            tokio::task::spawn_blocking(move || {
                                crate::runway::runway_by_id(
                                    &apt,
                                    lat,
                                    lon,
                                    &r.id,
                                    10.0,
                                    r.icao.as_deref(),
                                )
                                .ok()
                                .flatten()
                            })
                            .await
                            .ok()
                            .flatten()
                        }
                        None => None,
                    };
                    let path = rwy.as_ref().map(|r| map_handover_path(r, lat, lon, alt_ft));
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "map_handover", "agl_ft": agl_ft,
                            "ft_per_nm": path.as_ref().map(|p| p.ft_per_nm),
                            "dist_nm": rwy.as_ref().map(|r| crate::nav::haversine_nm(lat, lon, r.thr_lat, r.thr_lon))}),
                    );
                    let line = match (&path, &rwy) {
                        (Some(p), Some(r)) => {
                            crate::approach_guide::arm(p.clone());
                            format!(
                                "Missed approach point. Runway {} ahead, {:.1} miles. Your aircraft. \
                                 The autopilot is holding the descent; I'll talk you down.",
                                crate::crew_radio::spell_runway(&r.id),
                                crate::nav::haversine_nm(lat, lon, r.thr_lat, r.thr_lon)
                            )
                        }
                        _ => "Missed approach point, runway ahead. Your aircraft.".to_string(),
                    };
                    stand_down(&voice, &line, "map_handover");
                    return;
                }
                let mut hand_down = crate::autoland::HandDown::default();
                if map_short_of_runway(leg) {
                    // The visual segment: its own angle to the landing, and
                    // the published missed approach for its go-around.
                    hand_down = crate::autoland::HandDown::visual_segment(visual_deg);
                    stash_missed_legs(flight, missed_legs.clone());
                    alert(&voice, Priority::Advisory, "Missed approach point. Runway ahead, landing us.");
                } else {
                    alert(&voice, Priority::Advisory, "At the fix. Landing us.");
                }
                // The governor lets go BEFORE autoland's throttle law starts,
                // so no tick has two writers on the lever.
                crate::tools::speedhold::release();
                crate::autoland::run_autoland_scoped(
                    base_url.clone(),
                    apt_dat.clone(),
                    cfg.clone(),
                    *approach_ias,
                    voice.clone(),
                    None,
                    rwy_id.clone(),
                    xp.cancellation(),
                    hand_down,
                );
                clear_pilot_altitude();
                crate::terrain::publish_floor_ahead(None);
                RUN.record(sim::run::Outcome::HandedOff);
                ACTIVE.store(false, Ordering::SeqCst);
                return;
            }
        }

        // A to-altitude leg is satisfied by ALTITUDE, so one cannot sequence
        // while the pilot owns that altitude. Say so once rather than orbit a
        // heading in silence waiting for a climb nobody is flying.
        if dist_nm.is_none() && crate::pilot_altitude::held().is_some() && !said_alt_hold {
            said_alt_hold = true;
            if let Some(t) = leg.alt1_ft {
                alert(
                    &voice,
                    Priority::Caution,
                    &format!(
                        "This leg wants {} feet and you have the altitude. Say an altitude to \
                         carry on, or cancel.",
                        crate::auto_status::commas(t.round() as i64)
                    ),
                );
            }
        }

        if done {
            if let Some(c) = star_speed_crossing(&arrival_constraints, leg, ias) {
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({"event": "star_speed_crossed", "fix": c.fix, "kts": c.kts,
                        "target": c.target_kts, "ias": c.ias, "met": c.met,
                        "over_target_kts": c.over_target_kts, "dist_nm": dist_nm, "alt_ft": alt_ft}),
                );
            }
            idx += 1;
            if idx < legs.len() {
                let next = &legs[idx];
                // A course leg is named by its course, the way it is flown.
                let label = match (next.fix_ident.is_empty(), next.course_mag) {
                    (false, _) => next.fix_ident.clone(),
                    (true, Some(c)) => format!("course {}", crate::flightplan::spell_code(&format!("{:03}", c.round().rem_euclid(360.0) as i64))),
                    (true, None) => "the next leg".to_string(),
                };
                alert(&voice, Priority::Advisory, &format!("Next, {label}."));
            }
        }
    }

    // The route flown: LNAV goes out through the owner before the arrival
    // takes the aircraft (it seeds HDG SEL on its own bug).
    if lnav.in_play() {
        if let Ok(psi) = xp.get_float(PSI_TRUE).await {
            let (hdg, taken) = lnav_hand_back(&xp, psi, mag_var).await;
            crate::blackbox::record(
                "lnav",
                serde_json::json!({"event": "handback", "why": "route_complete", "hdg": hdg, "heading_on": taken}),
            );
        }
    }
    RUN.record(if matches!(end, EndAction::Approach { .. }) {
        sim::run::Outcome::Failed("approach ended without handoff".into())
    } else {
        sim::run::Outcome::Completed
    });
    // Ran off the end of the legs (a departure, or an approach with no handoff).
    // Drop any cruise-power hold before handing back (no-op if never armed).
    crate::tools::speedhold::release();
    match end {
        EndAction::Level => {
            crate::blackbox::record("procedure", serde_json::json!({"event": "complete_level"}));
            alert(
                &voice,
                Priority::Readback,
                "Procedure complete. Level, the autopilot is holding. Your aircraft.",
            );
        }
        EndAction::Enroute => {
            // The flight orchestrator is polling ACTIVE; it announces the
            // arrival itself. The AP keeps holding heading + altitude.
            crate::blackbox::record(
                "procedure",
                serde_json::json!({"event": "complete_enroute"}),
            );
        }
        EndAction::Approach { .. } => {
            crate::blackbox::record(
                "procedure",
                serde_json::json!({"event": "complete_no_handoff"}),
            );
            alert(
                &voice,
                Priority::Caution,
                "Ran out of approach legs without a handoff. Your aircraft.",
            );
        }
    }
    clear_pilot_altitude();
    crate::terrain::publish_floor_ahead(None);
    ACTIVE.store(false, Ordering::SeqCst);
}

/// Live misses of the terrain probe before the floor is retired for the
/// flight. One is a race with a pause or a slow frame; two in a row is a
/// plugin that does not answer.
pub const TERRAIN_MISSES_TO_RETIRE: u32 = 2;

/// Count an unanswered terrain probe. A miss while the sim is frozen is not
/// counted at all (the plugin's flight loop was not running to answer);
/// returns true when the probe should be retired for the flight.
pub fn terrain_probe_fatal(misses: &mut u32, frozen: bool) -> bool {
    if frozen {
        return false;
    }
    *misses += 1;
    *misses >= TERRAIN_MISSES_TO_RETIRE
}
