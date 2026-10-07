//! The loop: `fly`, whole, and the published arrival it tries.

use super::*;

pub(super) async fn fly(xp: XPlaneClient, base_url: String, voice: Option<Voice>, mut p: Params) {
    // The approach speed for what the aircraft weighs now, where the
    // airframe declares its Vref table (`aircraft::approach_kts_now`): base,
    // final and the hand-down to autoland are built on it, not on a fixed
    // mid-weight figure.
    if let Some((kts, kg)) = crate::aircraft::approach_kts_now(&xp).await {
        // The floors ride the same weight (autoland's lesson, #232).
        crate::aircraft::publish_weight_now(Some(kg));
        crate::blackbox::record(
            "circuit",
            serde_json::json!({"event": "approach_ias_weighed", "declared": p.approach_ias,
                "kts": kts, "weight_kg": kg.round()}),
        );
        p.approach_ias = kts;
    }
    // `base_url` is kept owned so each sub-loop gets its own client for its own
    // thread (the takeoff loop and the autoland loop each run on one).
    // The departure runway — its course drives every leg heading. Fit it from
    // where we're pointing right now (lined up before the roll), the same way
    // the takeoff loop does.
    let (Ok(lat), Ok(lon), Ok(psi)) = (
        xp.get_float(LAT).await,
        xp.get_float(LON).await,
        xp.get_float(PSI_TRUE).await,
    ) else {
        stand_down(
            &xp,
            &voice,
            "I can't read our position. The circuit is off, your aircraft.",
            "no_position",
        );
        return;
    };
    let cands = nearby_runways(&p.apt_dat, lat, lon, 3.0).unwrap_or_default();
    let Some((rwy, _)) = crate::runway::runway_fit(&cands, lat, lon, psi, p.takeoff.needed_m)
    else {
        stand_down(
            &xp,
            &voice,
            "We're not on a runway yet. I can only fly the circuit from the runway itself. \
             Taxi us over and line us up first, then say do a circuit.",
            "not_lined_up",
        );
        return;
    };
    let rwy_course = rwy.course_deg;
    let rwy_id = rwy.id.clone();
    let callsign = crew_callsign(&xp).await;
    let field_elev_ft = rwy.elev_ft;
    // The field's own circuit data (#182): apt.dat's `1302 allows_circuits`
    // and the runway's `1101` pattern side. LEMG says no circuits and 13
    // left, and the 737 has flown circuits there nightly on whatever side
    // the terrain picked.
    let field = {
        let (path, icao) = (p.apt_dat.clone(), rwy.icao.clone());
        tokio::task::spawn_blocking(move || crate::nav::find_airport(&path, &icao).ok().flatten())
            .await
            .ok()
            .flatten()
    };
    let published_left = field.as_ref().and_then(|a| a.pattern_left(&rwy.id));
    let allows_circuits = field.as_ref().and_then(|a| a.allows_circuits);
    let want_left = pattern_side_wanted(p.left_pattern, published_left);

    // Geometry scaled to the AP's real turn radius at pattern speed (~Vy plus
    // a bit) — fixed constants smaller than the turn diameter degenerated the
    // rectangle into loops (LXGB, 2026-07-24).
    let gs_est_mps = p.takeoff.vy_kts.max(60.0) * 0.514_444 * 1.1;
    // The pattern SPEED the throttle holds in the loop (~Vy), so the flown turn
    // radius matches the geometry above — a pattern flown at climb power runs
    // away to Vne and balloons ~3x wider (LXGB, 2026-07-26). Captured here,
    // before `p.takeoff` is moved into the takeoff loop.
    let pattern_kts = pattern_speed_kts(p.takeoff.vy_kts, p.approach_ias, p.vs_clean_kts);
    // ...and the speed the approach-setup legs are flown at.
    let approach_kts = approach_leg_kts(pattern_kts, p.approach_ias);
    let p_approach_ias = p.approach_ias;
    // Two radii, because the base-to-final turn is flown slower than the
    // pattern and is the one dimension where an over-estimate rolls us out
    // short of the centreline rather than merely wide.
    let mut geom =
        Geometry::for_radii(turn_radius_m(gs_est_mps), final_turn_radius_m(approach_kts));
    // The floor under the climb's speed protection, captured before `p` moves.
    let p_climb_floor_kts = p.climb_floor_kts;
    let p_vs_clean_kts = p.vs_clean_kts;
    // ...and the speed the climb is flown at, for the limiter's full-rate
    // threshold — NOT the leg speed the throttle holds (see
    // `climb_full_rate_kts` for the four knots that cost a circuit).
    let p_vy_kts = p.takeoff.vy_kts;

    // The world under BOTH candidate patterns: terrain probed from the sim
    // itself over the corridor the autopilot really flies (`corridor_frame`),
    // and the no-go airspace polygons the G1000 draws (OpenAir file). One
    // generic rule picks the side, height and descent; missing data degrades
    // to the old behavior, never to a guess.
    let poly_l = pattern_polyline(&rwy, &geom, rwy.length_m, true);
    let poly_r = pattern_polyline(&rwy, &geom, rwy.length_m, false);
    let spaces = {
        let path = p.airspace_path.clone();
        tokio::task::spawn_blocking(move || crate::airspace::load_near(&path, lat, lon, 0.5))
            .await
            .unwrap_or_default()
    };
    let tpa_probe_ft = field_elev_ft + p.tpa_agl_ft;
    let l_air =
        crate::airspace::first_conflict(&spaces, &poly_l, tpa_probe_ft).map(|a| a.spoken_name());
    let r_air =
        crate::airspace::first_conflict(&spaces, &poly_r, tpa_probe_ft).map(|a| a.spoken_name());
    let probe = |g: Geometry| {
        let (xp, rwy, l_air, r_air) = (&xp, &rwy, l_air.clone(), r_air.clone());
        async move {
            let l = SideFacts {
                airspace: l_air,
                ..probe_side(xp, rwy, &g, true).await
            };
            let r = SideFacts {
                airspace: r_air,
                ..probe_side(xp, rwy, &g, false).await
            };
            (l, r)
        }
    };
    let (mut l_facts, mut r_facts) = probe(geom).await;
    let mut pick = choose_pattern(
        want_left,
        &l_facts,
        &r_facts,
        field_elev_ft,
        p.tpa_agl_ft,
    );
    let lxgb = rwy.icao == "LXGB";
    // No side's descent clears the ground: a shorter downwind brings the base
    // in over other ground, where the geometry still leaves autoland a final
    // (`shortest_downwind_past_m`; a jet's drawn one is already the least).
    if pick.refused.is_some() && !lxgb {
        let least = shortest_downwind_past_m(geom.final_capture_m / 1.3, p_approach_ias);
        for dn in shorter_downwinds(geom.downwind_past_m, least) {
            let g = Geometry {
                downwind_past_m: dn,
                ..geom
            };
            let (l, r) = probe(g).await;
            let again = choose_pattern(want_left, &l, &r, field_elev_ft, p.tpa_agl_ft);
            crate::blackbox::record(
                "circuit",
                serde_json::json!({"event": "downwind_shortened", "downwind_past_m": dn,
                                   "refused": again.refused.is_some()}),
            );
            if again.refused.is_none() {
                (geom, l_facts, r_facts, pick) = (g, l, r, again);
                break;
            }
        }
    }
    let tpa_msl_ft = pick.tpa_msl_ft;
    // Gibraltar: the ONLY safe side is SOUTH — over the sea, looping around the
    // Rock. NORTH is Spain (restricted, foreign soil). choose_pattern picks the
    // LOWER-TERRAIN side, which at LXGB is the flat isthmus to the north, and it
    // flew the pattern straight into Spanish airspace (2026-07-26). Override to
    // the sea side; the runway sets the turn direction — 09 loops right
    // (clockwise), 27 loops left (anticlockwise), both staying over the water.
    // The authored route checks its own legs (`raise_for_terrain`); the
    // rectangle it may fall back to flies the forced side's own descent.
    let (left, side_pick, descent_shape, refused) = if lxgb {
        let left = sea_side_is_left(rwy_course);
        let forced = check_descent(
            &if left { &l_facts } else { &r_facts }.descent,
            field_elev_ft,
            tpa_msl_ft,
        );
        let refused = forced.shape.is_none().then(|| {
            format!(
                "The standard pattern here does not clear the ground: {}.",
                forced.problem.clone().unwrap_or_default()
            )
        });
        (
            left,
            "lxgb_south_over_the_sea",
            forced.shape.unwrap_or_default(),
            refused,
        )
    } else {
        (pick.left, pick.why, pick.shape, pick.refused.clone())
    };
    let check_json = |c: &DescentCheck| {
        serde_json::json!({
            "default_min_ft": c.default_min_ft, "flown_min_ft": c.flown_min_ft,
            "platform_agl_ft": c.shape.map(|s| s.platform_agl_ft),
            "glidepath_deg": c.shape.map(|s| s.glidepath_deg()),
            "problem": c.problem,
        })
    };
    crate::blackbox::record(
        "circuit",
        serde_json::json!({
            "event": "armed", "runway": rwy.id, "icao": rwy.icao,
            "course": rwy_course, "field_elev_ft": field_elev_ft,
            "tpa_msl_ft": tpa_msl_ft, "left": left, "side_pick": side_pick,
            "left_max_ft": l_facts.line_max_ft, "right_max_ft": r_facts.line_max_ft,
            "left_corridor_max_ft": l_facts.terrain_max_ft,
            "right_corridor_max_ft": r_facts.terrain_max_ft,
            "left_descent": check_json(&pick.descent_left),
            "right_descent": check_json(&pick.descent_right),
            "descent_platform_agl_ft": descent_shape.platform_agl_ft,
            "glidepath_deg": descent_shape.glidepath_deg(),
            "refused": refused,
            "left_airspace": l_facts.airspace, "right_airspace": r_facts.airspace,
            "airspaces_near": spaces.len(),
            "radius_m": geom.final_capture_m, "width_m": geom.width_m,
            "downwind_past_m": geom.downwind_past_m,
            "asked_left": p.left_pattern, "published_left": published_left,
            "allows_circuits": allows_circuits,
        }),
    );
    // Never flown into the ground (#182): refused on the runway, with the
    // numbers. The Rock's authored route is checked after the takeoff, and
    // its own refusal hands back there.
    if let (Some(line), false) = (refused.as_deref(), lxgb) {
        stand_down(&xp, &voice, &format!("{line} Your aircraft."), TERRAIN_REFUSED);
        return;
    }
    // ...and at the Rock, where the rectangle is refused, the authored route
    // is judged HERE too, on the runway, before anything flies: its legs are
    // fixed to the runway, so the answer after the takeoff is the same one.
    // Refused both ways in the air, the circuit handed an airborne aircraft
    // to nobody with "Your aircraft" (#182 review of 528b7d1, 2026-09-29).
    if let (Some(line), true) = (refused.as_deref(), lxgb) {
        let route = authored_circuit(&rwy, &geom, field_elev_ft, tpa_msl_ft, left);
        let why = match &route {
            Some(route) => {
                check_authored_route(&xp, &rwy, route, tpa_msl_ft, field_elev_ft, pattern_kts)
                    .await
                    .1
                    .err()
            }
            None => Some("no authored route".to_string()),
        };
        if let Some(why) = why {
            crate::blackbox::record(
                "circuit",
                serde_json::json!({"event": "authored_route_unsafe", "why": why, "on": "runway"}),
            );
            stand_down(
                &xp,
                &voice,
                &format!("{} Your aircraft.", rock_refused_line(line)),
                TERRAIN_REFUSED,
            );
            return;
        }
    }
    let side_word = |l: bool| if l { "left" } else { "right" };
    match side_pick {
        "switched_side_for_terrain" => alert(
            &voice,
            Priority::Caution,
            &format!(
                "Terrain on the {} side, flying a {}-hand circuit instead.",
                side_word(want_left),
                side_word(left)
            ),
        ),
        "switched_side_for_airspace" => {
            let name = if want_left {
                &l_facts.airspace
            } else {
                &r_facts.airspace
            };
            alert(
                &voice,
                Priority::Caution,
                &format!(
                    "The {} side runs into {}, flying a {}-hand circuit instead.",
                    side_word(want_left),
                    name.as_deref().unwrap_or("restricted airspace"),
                    side_word(left)
                ),
            );
        }
        "raised_for_terrain" if tpa_msl_ft - tpa_probe_ft >= RAISE_SAID_FT => alert(
            &voice,
            Priority::Caution,
            &format!("Terrain in the pattern, flying it at {tpa_msl_ft:.0} feet."),
        ),
        "airspace_unavoidable" => {
            let name = if left {
                &l_facts.airspace
            } else {
                &r_facts.airspace
            };
            alert(
                &voice,
                Priority::Caution,
                &format!(
                    "No pattern side stays clear of {} here, flying the {} side on terrain alone.",
                    name.as_deref().unwrap_or("restricted airspace"),
                    side_word(left)
                ),
            );
        }
        _ => {}
    }
    // The field's data, said before the takeoff (#182). Not at the Rock,
    // whose side is the sea's.
    if let Some(line) = field
        .as_ref()
        .filter(|_| !lxgb)
        .and_then(|a| {
            field_circuit_line(&a.name, &rwy.id, p.left_pattern, published_left, allows_circuits)
        })
    {
        alert(&voice, Priority::Advisory, &line);
    }
    if !lxgb && descent_shape != DescentShape::default() {
        alert(
            &voice,
            Priority::Caution,
            &descent_shape_line(descent_shape, field_elev_ft),
        );
    }
    alert(
        &voice,
        Priority::Advisory,
        crate::readback::one_of(&[
            "Flying a full circuit. My takeoff first.",
            "Full circuit coming up. I'll take us off first.",
            "Let's fly the circuit. My takeoff to start.",
        ]),
    );

    // --- Phase 1: the takeoff (the proven loop). Arm it, then wait for it to
    // hand off. It clears its own ACTIVE at the thousand-foot handoff — or on a
    // reject, which we tell apart by how high we ended up.
    // Read before the move: `p.takeoff` goes into the takeoff loop below, and
    // the base leg needs to know whether this airframe HAS gear to lower.
    let retractable_gear = p.takeoff.retract_gear;
    // ...and the retraction schedule, for a go-around the tower orders.
    let flap_schedule = p.takeoff.flap_schedule.clone();
    // Where the airframe declares it, the pattern altitude goes on the
    // altitude dial before the roll and the autopilot's own capture holds it
    // (see `ap_holds_level`). Otherwise the takeoff only lifts a dial the
    // climb would bust through, well clear of the pattern.
    let p_ap_capture = ap_altitude_capture();
    let mut takeoff = p.takeoff;
    if p_ap_capture {
        takeoff.level_ft = Some(tpa_msl_ft);
    }
    // The rectangle walks the flap schedule after the handoff (#150); an
    // authored route does not, so its takeoff cleans up as it always did.
    if authored_circuit(&rwy, &geom, field_elev_ft, tpa_msl_ft, left).is_some() {
        takeoff.flaps_to_caller = false;
    }
    if !autotakeoff::start_scoped(base_url.clone(), voice.clone(), takeoff, xp.cancellation()) {
        stand_down(
            &xp,
            &voice,
            "The previous takeoff is still stopping. Your aircraft.",
            "takeoff_busy",
        );
        return;
    }
    loop {
        tokio::time::sleep(std::time::Duration::from_millis(300)).await;
        if xp.cancelled() {
            stand_down(
                &xp,
                &voice,
                "Circuit cancelled. Your aircraft.",
                "abort_takeoff_phase",
            );
            return;
        }
        if !autotakeoff::running() {
            break;
        }
    }
    let outcome = autotakeoff::outcome();
    if outcome != Some(sim::run::Outcome::Completed) {
        // A reject has already stopped, parked and said "your aircraft".
        let line = match outcome {
            // The recovery has the aircraft and has said why (#181).
            Some(sim::run::Outcome::Failed(r)) if r == autotakeoff::AP_NO_PITCH => {
                "The circuit is off. I have the aircraft until you say my controls."
            }
            Some(sim::run::Outcome::Failed(_)) => {
                "The takeoff was rejected, so the circuit is off."
            }
            _ => "The takeoff handed control back. Your aircraft.",
        };
        stand_down(&xp, &voice, line, TAKEOFF_NOT_COMPLETED);
        return;
    }
    let agl_ft = xp.get_float(AGL_M).await.unwrap_or(0.0) * M_TO_FT;
    if agl_ft < AIRBORNE_MIN_AGL_FT {
        stand_down(
            &xp,
            &voice,
            "The takeoff didn't get us airborne, so the circuit is off.",
            "takeoff_rejected",
        );
        return;
    }
    // What the takeoff left us besides the aircraft: the climb it was
    // commanding and the flaps it left out for this loop's schedule (#150).
    let handover = autotakeoff::take_climb_handover().unwrap_or_default();
    crate::blackbox::record(
        "circuit",
        serde_json::json!({"event": "takeoff_handoff", "agl_ft": agl_ft,
                           "vs_cmd": handover.vs_cmd, "flaps_left": handover.flaps_left}),
    );

    // --- Phase 2a: an AUTHORED track, where the parametric rectangle can't be
    // flown clear (LXGB — R-164 blankets both sides, so the rectangle flies
    // through Spanish airspace AND the Rock). The airspace is unavoidable here,
    // but the route keeps the ground track over the bay and around the Rock's
    // terrain. Validated against the live terrain probe first; if any waypoint
    // isn't clear (a wrong hand-authored point, an off-nominal start), fall
    // back to the standard pattern rather than fly it into a hill.
    // The authored around-the-Rock loop, once the sea-side fix pointed it the
    // right way (clockwise on 09), flew a clean circuit south around the Rock and
    // handed to autoland — confirmed in the sim (2026-07-26). It runs whenever
    // it's available and terrain-clear.
    if let Some(route) = authored_circuit(&rwy, &geom, field_elev_ft, tpa_msl_ft, left) {
        // Probed along the LEGS, not at the waypoints: the six points sat on
        // sea either side of the Rock while the downwind between them crossed
        // the ridge 523 ft up (2026-09-14). Terrain under a pattern-altitude
        // leg raises the pattern altitude; terrain under a descending leg,
        // or a raise past MAX_TERRAIN_RAISE_FT, refuses the route.
        let (leg_max, verdict) =
            check_authored_route(&xp, &rwy, &route, tpa_msl_ft, field_elev_ft, pattern_kts).await;
        let authored = match verdict {
            Ok(v) => Some(v),
            Err(why) => {
                crate::blackbox::record(
                    "circuit",
                    serde_json::json!({"event": "authored_route_unsafe", "why": why,
                                       "on": "airborne"}),
                );
                None
            }
        };
        if let Some((route, tpa_used)) = authored {
            crate::blackbox::record(
                "circuit",
                serde_json::json!({
                    "event": "authored_route", "wpts": route.len(),
                    "tpa_msl_ft": tpa_used, "raised_ft": tpa_used - tpa_msl_ft,
                    "leg_max_ft": leg_max,
                }),
            );
            if tpa_used > tpa_msl_ft + 1.0 {
                alert(
                    &voice,
                    Priority::Advisory,
                    &format!(
                        "Pattern altitude {} feet, to clear the Rock.",
                        (tpa_used / 50.0).round() * 50.0
                    ),
                );
            }
            alert(
                &voice,
                Priority::Advisory,
                crate::readback::one_of(&[
                    "Airborne. Flying the bay circuit, around the Rock.",
                    "Off the ground. Taking the bay circuit around the Rock.",
                    "Airborne. Around the Rock we go, over the bay.",
                ]),
            );
            let mag_var = xp.get_float(MAG_VAR).await.unwrap_or(0.0);
            fly_authored_route(
                XPlaneClient::new(base_url.clone()),
                base_url.clone(),
                voice.clone(),
                route,
                rwy.clone(),
                mag_var,
                pattern_kts,
                climb_full_rate_kts(pattern_kts, p_vy_kts, p_climb_floor_kts),
                p_climb_floor_kts,
                callsign.clone(),
                p.apt_dat.clone(),
                p.cfg.clone(),
                p.approach_ias,
            )
            .await;
            return;
        }
        // ...and the standard pattern is flown only where it clears the
        // ground itself (#182). Refused both ways in the AIR (the runway's
        // judgement passed, the probe answered otherwise after the takeoff):
        // never "your aircraft" at the handoff height, the reversal's hold
        // instead, climbing clear of the ground ahead on the autopilot.
        if let Some(line) = refused.as_deref() {
            let corridor = [l_facts.terrain_max_ft, r_facts.terrain_max_ft]
                .into_iter()
                .chain(leg_max.iter().copied())
                .flatten()
                .reduce(f64::max);
            let mag_var = xp.get_float(MAG_VAR).await.unwrap_or(0.0);
            let hold_kts = crate::autoland::hand_back_speed_kts(
                p_approach_ias,
                handover.flaps_left.unwrap_or(0.0),
                &crate::aircraft::flap_limits(),
            );
            let floor = crate::aircraft::config_floor_kts(handover.flaps_left.unwrap_or(0.0));
            hold_clear_of_terrain(
                &xp,
                &base_url,
                &voice,
                &rock_refused_line(line),
                corridor,
                tpa_msl_ft,
                rwy_course,
                mag_var,
                hold_kts,
                floor,
            )
            .await;
            return;
        }
        alert(
            &voice,
            Priority::Caution,
            "The Rock circuit isn't clear from here. Flying the standard pattern.",
        );
    }
    alert(
        &voice,
        Priority::Advisory,
        crate::readback::one_of(&[
            "Airborne. Flying the pattern.",
            "Off the ground. Into the pattern.",
            "Airborne. Taking us round the pattern.",
        ]),
    );

    // --- Phase 2: the pattern rides the AP EXACTLY as the takeoff left it —
    // servos + HDG + VS engaged, VS dial at the climb rate. NOTHING is
    // re-pressed wholesale here: the old pattern-start recipe (servos_on +
    // HDG press + ALT_HOLD) stomped those working modes — X-Plane's
    // `altitude_hold` holds the CURRENT altitude, which locked every pattern
    // at the ~520 ft handoff height instead of climbing to TPA, left the
    // terrain guard's "climb" a NO-OP, and the aircraft flew straight through
    // its commanded crosswind turn into terrain SW of LXGB (2026-07-24,
    // blackboxed: flat AGL all flights, 6 s of no-turn on crosswind).
    // Vertical is VS-mode + dial (the takeoff's proven recipe); modes are
    // re-engaged ONLY when their status reads off, like the takeoff's guard.
    let mag_var = xp.get_float(MAG_VAR).await.unwrap_or(0.0);

    let mut leg = Leg::Upwind;
    let _detail = crate::auto_status::DetailGuard::hold(&DETAIL);
    // The dial's CURRENT value seeds the slew (the takeoff left it near the
    // runway heading); every subsequent write moves it the short way only.
    LAST_BUG.store(u64::MAX, std::sync::atomic::Ordering::Relaxed);
    let mut bug_mag = match autoflight::read_dial(&xp, Dial::HeadingBug).await {
        Ok(v) => v,
        Err(_) => true_to_mag(rwy_course, mag_var),
    };
    bug_mag = slew_bug(
        bug_mag,
        true_to_mag(leg_heading(rwy_course, Leg::Upwind, left), mag_var),
        BUG_SLEW_PER_TICK,
    );
    write_bug(&xp, bug_mag).await;
    // The pattern's knob from here on (`BugHold`, issue #93).
    let mut bug_hold = BugHold::default();
    let loop_t0 = std::time::Instant::now();
    // The lateral re-engage (`ap_engage::ServoWatch`): FD first, confirmed.
    let mut servo_watch = crate::ap_engage::ServoWatch::default();
    // The pattern's own give-up (#160).
    let mut pattern_guard = PatternGuard::default();
    // The autopilot's answer to the dial (#181).
    let mut climb_watch = crate::climb_watch::ClimbWatch::default();
    let mut last_verdict = crate::climb_watch::Verdict::Flying;
    // Forward terrain guard cadence + throttle; a fired guard raises the
    // vertical target floor for a while instead of pressing dead buttons.
    let mut tick: u32 = 0;
    let mut guard_warned: Option<std::time::Instant> = None;
    let mut guard_floor_ft: Option<(f64, std::time::Instant)> = None;
    // VS-dial bookkeeping (the takeoff's recipe: engage syncs the dial away,
    // so rewrite after any re-engage; skip no-op writes otherwise).
    let mut last_vs_cmd: Option<f64> = None;
    let mut vs_refused: Option<std::time::Instant> = None;
    let mut vs_refused_said = false;
    let mut lights_done = false;
    // Speed-hold bookkeeping: power works the throttle to hold `pattern_kts`,
    // leading on the airspeed trend EMA'd tick to tick (the flight law's input).
    let mut prev_ias: Option<(f64, std::time::Instant)> = None;
    let mut ias_trend = 0.0_f64;
    // Vertical speed, derived the same way, for the altitude capture's lead.
    let mut prev_alt: Option<(f64, std::time::Instant)> = None;
    let mut vs_est = 0.0_f64;
    // ...and the rate we are closing the threshold at, derived the same way
    // again, for the glidepath's feed-forward. GROUND closure, not airspeed:
    // the path descends at 318 ft for every mile actually covered, so a
    // headwind genuinely slows the target down and an IAS proxy would
    // over-command by the whole wind (see `path_descent_fpm`).
    let mut prev_along: Option<(f64, std::time::Instant)> = None;
    let mut closure_kts = 0.0_f64;
    // The ratcheted base/final descent target — see the vertical law below.
    let mut descent_target: Option<f64> = None;
    // Gear goes down ONCE, at the 180, and the latch is what makes it once.
    let mut gear_lowered = !retractable_gear;
    // The flap handle as this loop last commanded it; `None` until base.
    let mut last_flap: Option<f64> = None;
    // Set when the base turn onto final may BEGIN. The handover waits for the
    // geometry to become flyable, not merely for the leg to change.
    let mut turning_final = false;
    // The tower's go-around orders, watched from the pattern on: a landing
    // clearance only stands on the approach legs, and there the order
    // outranks everything (see `flies_go_around_order`).
    // The pilot's order rides the same watch and is flown the same way
    // (`go_around`, #77).
    let mut orders = crate::go_around::GoAroundWatch::arm();
    // An order seen while handing over, flown on the next tick.
    let mut order_owed: Option<crate::go_around::GoAroundBy> = None;
    // A clean-up in progress, the takeoff's (#150) or a go-around's: the flap
    // it has commanded, until clean.
    let mut cleaning_up: Option<f64> = takeoff_flaps_to_clean(handover.flaps_left, &flap_schedule);
    if cleaning_up.is_some() {
        last_flap = cleaning_up;
    }
    // Whose clean-up it is, for the record.
    let mut cleanup_event = "climb_flap_step";
    // The flaps' own pace for the clean-up (#169), from the detent above the
    // one handed over: the takeoff may have stepped it on the way out.
    let mut flap_pace = autotakeoff::FlapPace::default();
    let handed_from = cleaning_up.and_then(|f| detent_above(f, &crate::aircraft::flap_limits()));
    if let Some(above) = handed_from {
        flap_pace.stepped(above);
    }
    let pace_t0 = std::time::Instant::now();
    // The takeoff's climb, carried into the pattern's law (#150).
    let mut climb_carry = ClimbCarry::new(handover.vs_cmd);
    let mut last_tick_at: Option<std::time::Instant> = None;
    // The gear the go-around still owes raising.
    let mut gear_up_owed = false;
    let mut go_arounds: u32 = 0;
    // The circuit's own go-arounds for a final too high to hand over (#188).
    let mut high_go_arounds: u32 = 0;
    // The steepest the pattern descends at, this airframe (#188).
    let descent_cap = pattern_descent_cap_fpm(crate::procedure::descent_fpm());
    // The downwind held past its base turn for height, and where it began.
    let mut extending: Option<f64> = None;
    // The bank flown through the turn onto base.
    let mut base_bank = BankWatch::default();

    loop {
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        if xp.cancelled() {
            stand_down(
                &xp,
                &voice,
                "Circuit cancelled. Your aircraft.",
                "abort_pattern_phase",
            );
            return;
        }
        // A frozen sim (pause/replay) suspends the sequence; the AP holds.
        if crate::xplane::sim_frozen(&xp).await {
            continue;
        }

        // Position, height and heading as ONE frame saw them (#157).
        let (pos, hpath_r, hdg_st, vs_st) = tokio::join!(
            sim::position::read(&xp),
            xp.get_float(HPATH),
            crate::autoflight::read_status(&xp, HDG_STATUS),
            crate::autoflight::read_status(&xp, VS_STATUS),
        );
        let (lat, lon) = sim::position::lat_lon(&pos);
        let agl_m = sim::position::field(&pos, |p| p.agl_m);
        let elev_m = sim::position::field(&pos, |p| p.elev_m);
        let psi_r = sim::position::field(&pos, |p| p.psi_true_deg);
        let thr_dref = throttle_dataref();
        let (ias_r, thr_r, dial_r, alt_hold_r, phi_r, servos_r) = tokio::join!(
            xp.get_float(IAS),
            xp.get_float(&thr_dref),
            autoflight::read_dial(&xp, Dial::Altitude),
            crate::autoflight::read_status(&xp, ALT_HOLD_STATUS),
            xp.get_float(PHI),
            crate::autoflight::read_status(&xp, SERVOS_ON),
        );
        let (Ok(lat), Ok(lon)) = (lat, lon) else {
            continue;
        };
        let agl_ft = agl_m.unwrap_or(0.0) * M_TO_FT;
        let alt_msl_ft = elev_m.map(|m| m * M_TO_FT).unwrap_or(f64::NAN);
        let (hdg_read, vs_read) = (hdg_st.ok(), vs_st.ok());
        let (hdg_on, vs_on) = (hdg_read.unwrap_or(1.0) > 0.5, vs_read.unwrap_or(1.0) > 0.5);
        // Position in the departure-runway frame: along the takeoff direction
        // (negative on the approach side) and cross-track (right positive).
        let a_m = -along_track_nm(lat, lon, &rwy) * M_PER_NM;
        let x_m = cross_track_nm(lat, lon, &rwy) * M_PER_NM;
        // Height over the circuit's own path here (#188): the handoff waits
        // for it, and a final still high a mile out is flown around.
        let above_path = above_path_ft(alt_msl_ft, field_elev_ft, a_m, descent_shape.gp_ft_per_nm);
        // Once only: a second lap as high as the first is the same geometry,
        // and autoland's own late go-around and reversal take it from there
        // (blackbox `high_handoff`).
        let high_final = leg == Leg::Final
            && turning_final
            && high_go_arounds == 0
            && final_high_give_up(above_path, a_m);
        if high_final {
            high_go_arounds += 1;
        }

        // --- The tower's go-around order, before anything else: a landing
        // clearance voided while the circuit flies its own final is flown
        // HERE, not left to autoland (which arms at 2 nm and only watches
        // from then on). LEMG 31 (sortie 1790584715-full_auto, 2026-09-28):
        // the order came at 736 ft, 5.7 km out, and the circuit flew on,
        // handed over at 2 nm and landed "without clearance".
        let fresh = orders.take();
        let crew = high_final.then_some(crate::go_around::GoAroundBy::Crew);
        if let Some(by) = order_owed.take().or(fresh).or(crew) {
            let ias_now = ias_r.as_ref().ok().copied();
            // The crew's own go-around answers no order.
            if by != crate::go_around::GoAroundBy::Crew {
                crate::go_around::answer(if flies_go_around_order(leg) {
                    crate::go_around::Answer::Flying
                } else {
                    crate::go_around::Answer::NotOnApproach
                });
            }
            if flies_go_around_order(leg) {
                go_arounds += 1;
                let limits = crate::aircraft::flap_limits();
                let flap = go_around_flap(last_flap, &limits);
                let (thr_ok, flap_ok) = tokio::join!(xp.set_float(&thr_dref, 1.0), async {
                    if limits.is_empty() || last_flap.is_none_or(|f| f <= flap + 1e-6) {
                        None
                    } else {
                        Some(crate::tools::flaps::set_handle(&xp, flap).await.is_ok())
                    }
                },);
                crate::blackbox::record(
                    "circuit",
                    serde_json::json!({
                        "event": "ordered_go_around", "leg": format!("{leg:?}"),
                        "agl_ft": agl_ft, "a_m": a_m, "x_m": x_m, "ias": ias_now,
                        "flap_was": last_flap, "flap": flap, "flap_ok": flap_ok,
                        "thr_ok": thr_ok.is_ok(), "n": go_arounds, "by": by.label(),
                        "above_path_ft": above_path,
                    }),
                );
                if let Some(was) = last_flap {
                    flap_pace.stepped(was);
                }
                if !limits.is_empty() {
                    last_flap = Some(flap);
                }
                cleaning_up = (!limits.is_empty() && flap > 1e-6).then_some(flap);
                cleanup_event = "go_around_flap_step";
                gear_up_owed = retractable_gear && gear_lowered;
                leg = Leg::Upwind;
                turning_final = false;
                descent_target = None;
                last_vs_cmd = None;
                alert(&voice, Priority::Warning, go_around_line(by));
                // The tower voided the clearance, or the pilot did: it is owed
                // the call, and the downwind report comes round with the
                // pattern.
                transmit(radio_call(&callsign, "going around"));
            } else {
                crate::blackbox::record(
                    "circuit",
                    serde_json::json!({"event": "go_around_order_ignored",
                                       "leg": format!("{leg:?}"), "agl_ft": agl_ft,
                                       "by": by.label()}),
                );
            }
        }

        // --- Hand over only once our own turn onto final has rolled out. ---
        // `turning_final` means the turn may BEGIN; the aircraft is still up
        // to 1.3 radii abeam at that instant, pointing across the centreline.
        // See `ready_for_autoland` for why "autoland could close this" is not
        // the same question as "we are lined up".
        // Rolled out is judged on the ground TRACK, not the nose: the leg
        // commands intercept-plus-crab, so on the centreline in a crosswind
        // the heading carries the whole wind-correction angle and a
        // heading-based gate never passes (see `ready_for_autoland`). A
        // failed track read reads as "not rolled out yet": it can only delay
        // the handoff by a tick, where treating it as aligned would hand over
        // the very thing this guard exists to prevent.
        let hdg_err = hpath_r
            .as_ref()
            .ok()
            .map(|hp| crate::angles::signed_diff_deg(*hp, rwy_course))
            .unwrap_or(180.0);
        // Established by autoland's own measure: its pursuit's residual at
        // the threshold inside the runway's landing limit. The IAS stands in
        // for the groundspeed (a failed read: the approach speed).
        let handoff_gs = ias_r
            .as_ref()
            .ok()
            .copied()
            .filter(|v| v.is_finite() && *v > 40.0)
            .unwrap_or(p_approach_ias);
        let rwy_width_m = crate::runway::runway_width_m(&rwy);
        let landing_limit_m = crate::autoland::landing_xtk_limit_m(rwy_width_m);
        // On the path too (#188), until the one lap flown around for height
        // comes back as high: then autoland's own go-around takes it.
        let high_handoff = high_go_arounds > 0 && final_high_give_up(above_path, a_m);
        if turning_final
            && ready_for_autoland(
                x_m,
                a_m.abs(),
                hdg_err,
                handoff_gs,
                rwy_width_m.map(|_| landing_limit_m),
                if high_handoff { None } else { above_path },
            )
        {
            let alt_msl = field_elev_ft + agl_ft;
            let arr = ArrivalCtx {
                xp: &xp,
                apt_dat: &p.apt_dat,
                cifp_dir: &p.cifp_dir,
                fix_data_path: &p.fix_data_path,
                nav_data_path: &p.nav_data_path,
                cfg: &p.cfg,
                approach_ias: p.approach_ias,
            };
            if try_published_arrival(
                arr, &rwy.icao, &rwy_id, &base_url, &voice, lat, lon, alt_msl,
            )
            .await
            {
                // The procedure flies the arrival, its parent our token.
                handed_off(&xp);
                return;
            }
            // The arrival lookup takes a while off the reactor: an order
            // given meanwhile is still this leg's, never autoland's to miss.
            if let Some(by) = orders.take() {
                order_owed = Some(by);
                continue;
            }
            // A stand-down (swap, instructor, a new flight) that landed during
            // that await set our ABORT and autoland's: clearing autoland's
            // latch below would launch an autoland nobody can cancel.
            if xp.cancelled() {
                stand_down(
                    &xp,
                    &voice,
                    "Circuit cancelled. Your aircraft.",
                    "abort_pattern_phase",
                );
                return;
            }
            crate::blackbox::record(
                "circuit",
                serde_json::json!({
                    "event": "handoff_autoland",
                    "agl_ft": agl_ft,
                    "icao": rwy.icao,
                    "rwy": rwy_id,
                    "xtk_m": x_m,
                    "along_m": a_m,
                    "trk_err": hdg_err,
                    "residual_m": handoff_residual_m(x_m, a_m, handoff_gs),
                    "limit_m": landing_limit_m,
                    "glidepath_deg": descent_shape.glidepath_deg(),
                    "above_path_ft": above_path,
                    "high_handoff": high_handoff,
                }),
            );
            // A path steepened over the ground is autoland's path too (#182),
            // else it would take the aircraft handed over and dive for 3 degrees.
            // A circuit final is judged at the visual gate, 500 ft (#199).
            let hand_down = crate::autoland::HandDown::circuit(
                descent_shape.steep().then(|| descent_shape.glidepath_deg()),
            );
            // The circuit's token is its parent: a stand-down that lands
            // after the recheck above still cancels this autoland.
            crate::autoland::run_autoland_scoped(
                base_url.clone(),
                p.apt_dat.clone(),
                p.cfg.clone(),
                p.approach_ias,
                voice.clone(),
                None,
                Some(crate::runway::PlannedRunway::anywhere(&rwy_id)),
                xp.cancellation(),
                hand_down,
            );
            handed_off(&xp);
            return;
        }

        // Fly each leg as a TRACK, not a bare heading: crab into the wind so the
        // AP holds the ground track. Without this a 12 kt crosswind blew the
        // downwind 900 m back into the field (the teardrop, 2026-07-23).
        // crab = heading - track; command heading = leg heading + crab.
        let (psi_v, hp_v) = (psi_r.ok(), hpath_r.ok());
        let crab = match (psi_v, hp_v) {
            (Some(psi), Some(hp)) => {
                crate::angles::signed_diff_deg(psi, hp).clamp(-25.0, 25.0)
            }
            _ => 0.0,
        };
        // Every leg but the last flies its own heading. FINAL flies an
        // intercept: it is the leg whose whole job is to be ON the centreline
        // by the handoff, and commanding the bare runway course makes it a leg
        // that flies parallel to the line it is supposed to join.
        let leg_hdg = match leg {
            Leg::Final => final_intercept_hdg(
                rwy_course,
                final_led_xtk_m(x_m, hp_v, rwy_course, handoff_gs, final_lead_s()),
            ),
            _ => leg_heading(rwy_course, leg, left),
        };
        // The bank the autopilot really flies, from the turn onto base, sizes
        // the turn onto final (see `final_capture_on_bank`).
        if leg == Leg::Base {
            if let (Some(psi), Ok(phi)) = (psi_v, phi_r.as_ref()) {
                base_bank.note(*phi, psi - leg_hdg);
            }
            if let (Some(bank), Ok(ias)) = (base_bank.steady_bank_deg(), ias_r.as_ref()) {
                if let Some(c) = final_capture_on_bank(*ias, bank) {
                    geom.final_capture_m = c;
                }
            }
        }
        let want_hdg = leg_hdg + crab;
        let target_mag = true_to_mag(want_hdg, mag_var);
        // Through the one door every pursuit steer takes: the bug leads the
        // nose gently until the autopilot is banked the right way. A bare
        // slew put it 90 degrees off from wings level in 1.5 s, and the SF50
        // flew 13 of its 30 circuit turns the long way round (ETND 26,
        // 2026-09-23: crosswind, downwind and base each a 270 right).
        bug_mag = match (psi_v, phi_r.as_ref().ok()) {
            (Some(psi), Some(phi)) => bug_toward(
                bug_mag,
                target_mag,
                true_to_mag(psi, mag_var),
                *phi,
                BUG_SLEW_PER_TICK,
            ),
            _ => slew_bug(bug_mag, target_mag, BUG_SLEW_PER_TICK),
        };
        // Set, not rewritten: per leg, a correction past the leg's deadband,
        // the final's finer (`BugHold`, issue #93).
        hold_bug(
            &xp,
            &mut bug_hold,
            leg as u32,
            &mut bug_mag,
            loop_t0.elapsed().as_secs_f64(),
            if leg == Leg::Final {
                CIRCUIT_FINAL_DEADBAND
            } else {
                CIRCUIT_LEG_DEADBAND
            },
        )
        .await;
        // The lateral mode must actually be ON for the bug to mean anything —
        // re-engage like the takeoff's VS guard, never press blind: the
        // servos through the one engage (`ap_engage::ServoWatch`, FD first,
        // confirmed), the mode pressed while it reads off.
        reengage_lateral(
            &xp,
            &mut servo_watch,
            &p.cfg.fd_commands,
            loop_t0.elapsed().as_secs_f64(),
            !hdg_on,
            servos_r.as_ref().ok().copied(),
            ias_r.as_ref().ok().copied().unwrap_or(f64::NAN),
            "rectangle",
        )
        .await;
        autoflight::bring_heading(&xp, hdg_read, "circuit_rectangle").await;

        // Forward terrain floor, every ~2 s: the ground under us and along
        // the ground track ahead (the follower's own probe geometry,
        // `terrain::look_ahead_points`). Inside LIVE_FLOOR_FT of it on the
        // upwind, crosswind, downwind or base, the circuit stops descending
        // and climbs to its floor (`live_floor_ft`), said once a spell,
        // blackbox `terrain_floor` (#182: LEMG 13's base at 305 ft AGL).
        // Silently inactive without the plugin probe (the old behavior).
        tick = tick.wrapping_add(1);
        if tick.is_multiple_of(4) && alt_msl_ft.is_finite() && leg != Leg::Final {
            if let Some(hp) = hp_v {
                let mut pts = vec![(lat, lon)];
                pts.extend(crate::terrain::look_ahead_points(
                    lat,
                    lon,
                    hp,
                    LIVE_LOOK_NM,
                    LIVE_LOOK_POINTS,
                ));
                if let Some(e) = crate::terrain::max_elevation_ft(&xp, &pts).await {
                    if let Some(floor) = live_floor_ft(leg, alt_msl_ft, e) {
                        guard_floor_ft = Some((floor, std::time::Instant::now()));
                        if guard_warned.is_none() {
                            guard_warned = Some(std::time::Instant::now());
                            crate::blackbox::record(
                                "circuit",
                                serde_json::json!({"event": "terrain_floor", "leg": format!("{leg:?}"),
                                                   "ground_ft": e, "alt_msl": alt_msl_ft,
                                                   "agl_ft": agl_ft, "floor_ft": floor}),
                            );
                            alert(
                                &voice,
                                Priority::Warning,
                                &format!(
                                    "Terrain ahead. Holding {:.0} feet.",
                                    (floor / 50.0).ceil() * 50.0
                                ),
                            );
                        }
                    }
                }
            }
        }

        // --- Vertical: the takeoff's proven VS recipe, never ALT_HOLD. The
        // target is the leg's altitude (TPA; the handoff height from base),
        // floored by an active terrain-guard demand. The dial is written on
        // change; a dropped VS mode is re-engaged and the dial rewritten
        // (engage syncs it away — the takeoff's lesson).
        guard_floor_ft = guard_floor_ft.filter(|(_, t)| t.elapsed().as_secs() < 60);
        // A spell ends with its floor: the next one is said again.
        if guard_floor_ft.is_none() {
            guard_warned = None;
        }
        // GEAR DOWN abeam the touchdown point — the same moment the descent
        // starts, which is how a real pattern does it: power back, gear down,
        // start down. Putting it a leg later (on base) while the descent had
        // already moved to the 180 left the aircraft descending, slowing and
        // gear-up for the whole second half of downwind, which is exactly the
        // configuration an airframe's gear warning exists to shout about — and
        // it did, all the way round (SF50 at EGHI, twice).
        //
        // Base is kept as the backstop for any path that reaches it without
        // passing the abeam test; the latch means it can only fire once.
        if !gear_lowered
            && (matches!(leg, Leg::Base) || (matches!(leg, Leg::Downwind) && a_m < 0.0))
        {
            let _ = crate::tools::gear::set_handle(&xp, false).await;
            alert(&voice, Priority::Advisory, "Gear down.");
            gear_lowered = true;
        }
        let leg_target = leg_target_shaped(
            leg,
            a_m,
            field_elev_ft,
            tpa_msl_ft,
            geom.downwind_past_m,
            descent_shape,
            &mut descent_target,
        );
        // The floor never applies on final: descending toward the runway is
        // the POINT there, and autoland brings its own terrain floor. A
        // ridge-probe floor that lingered into base handed autoland the
        // approach 190 ft high and it balked over the numbers (2026-07-24),
        // so on base it is only the ground's own LIVE_FLOOR_FT (`live_floor_ft`)
        // and only while that ground is there: a balk is better than the
        // hill LEMG 13's base passed 305 ft over (#182).
        let target_alt = if leg == Leg::Final {
            leg_target
        } else {
            guard_floor_ft.map_or(leg_target, |(f, _)| leg_target.max(f))
        };
        // Still a pattern (#160): not far above it, no leg without end, an
        // autopilot in. Else the aircraft goes back, flyable and said loudly.
        let pattern_ft = tpa_msl_ft.max(target_alt);
        let servos = servos_r
            .as_ref()
            .ok()
            .filter(|v| v.is_finite())
            .map(|v| *v > 0.5);
        if let Some(trip) = pattern_guard.step(
            loop_t0.elapsed().as_secs_f64(),
            leg,
            alt_msl_ft,
            pattern_ft,
            leg_limit_s(leg, &geom, rwy.length_m, pattern_kts),
            servos,
        ) {
            let hold_kts = crate::autoland::hand_back_speed_kts(
                p_approach_ias,
                last_flap.unwrap_or(0.0),
                &crate::aircraft::flap_limits(),
            );
            let floor = crate::aircraft::config_floor_kts(last_flap.unwrap_or(0.0));
            give_up_pattern(
                &xp, &base_url, &voice, trip, leg, alt_msl_ft, pattern_ft, mag_var, hold_kts,
                floor,
            )
            .await;
            return;
        }
        // Vertical speed, derived from successive altitude reads (the same
        // trick the speed law uses on IAS) — the lead term the capture needs.
        if alt_msl_ft.is_finite() {
            let t_now = std::time::Instant::now();
            if let Some((prev, t_prev)) = prev_alt {
                let dt = t_now.duration_since(t_prev).as_secs_f64();
                if (0.1..=2.0).contains(&dt) {
                    vs_est = 0.6 * vs_est + 0.4 * (alt_msl_ft - prev) / dt * 60.0;
                }
            }
            prev_alt = Some((alt_msl_ft, t_now));
        }
        // The target MOVES on final: it is the glidepath at our distance to
        // run, so it walks down as we close, and the capture has to be told
        // that or it flies a permanent lag (see `alt_capture_vs`). Only on
        // final — base aims at the fixed roll-out point, and downwind at a
        // level altitude, so for those the target really is standing still.
        if a_m.is_finite() {
            let t_now = std::time::Instant::now();
            if let Some((prev, t_prev)) = prev_along {
                let dt = t_now.duration_since(t_prev).as_secs_f64();
                if (0.1..=2.0).contains(&dt) {
                    let closing = (prev.abs() - a_m.abs()) / dt * 1.943_844;
                    closure_kts = (0.6 * closure_kts + 0.4 * closing).clamp(0.0, 400.0);
                }
            }
            prev_along = Some((a_m, t_now));
        }
        let target_rate = if leg == Leg::Final {
            path_descent_fpm_on(closure_kts, descent_shape.gp_ft_per_nm)
        } else {
            0.0
        };
        // The descent sized on the height to lose over the track left, within
        // the airframe's (#188): a flat 500 fpm from a pattern raised for the
        // ground handed autoland the final 545 ft high. A terrain floor above
        // the leg's target is still the target: never below it.
        let descent_fpm = leg_descent_authority_fpm(
            leg,
            a_m,
            x_m,
            alt_msl_ft,
            field_elev_ft,
            tpa_msl_ft,
            &geom,
            left,
            descent_shape,
            handoff_gs,
            descent_cap,
            target_rate,
        );
        let vs_want = if alt_msl_ft.is_finite() {
            alt_capture_vs_on(target_alt, target_rate, alt_msl_ft, vs_est, descent_fpm)
        } else {
            0.0
        };
        // ...and only as much of it as the airspeed will pay for. The AP buys
        // rate with knots, and on a piston twin in a climbing turn there is no
        // power left to buy them back — see `climb_limit_fpm`.
        let ias_read = match &ias_r {
            Ok(v) => *v,
            Err(_) => f64::NAN,
        };
        // The ONE speed this leg holds: the throttle below holds it and the
        // limiter grants full rate at it (#169).
        let want_kts = leg_want_kts(
            leg,
            LegSpeeds {
                pattern_kts,
                approach_kts,
                approach_ias: p_approach_ias,
            },
            last_flap,
            cleaning_up,
            &flap_schedule,
            &crate::aircraft::flap_limits(),
            crate::aircraft::config_floor_kts,
        );
        // ...and the floor it fades to, the kept flap's while cleaning up.
        let climb_floor = leg_climb_floor_kts(
            leg,
            p_climb_floor_kts,
            p_vs_clean_kts,
            cleaning_up,
            crate::aircraft::config_floor_kts,
        );
        let full_kts = climb_full_rate_kts(want_kts, p_vy_kts, climb_floor);
        let law_vs = climb_limit_fpm(vs_want, ias_read, ias_trend, full_kts, climb_floor);
        // ...walked to from the climb the takeoff handed over (#150).
        let tick_dt = last_tick_at.map_or(0.5, |t| t.elapsed().as_secs_f64());
        last_tick_at = Some(std::time::Instant::now());
        let mut vs_cmd = climb_carry.step(
            law_vs,
            tick_dt,
            matches!(leg, Leg::Base | Leg::Final),
            ias_read.is_finite() && ias_read <= climb_floor,
        );
        // Landing and taxi lights off through 1,000 ft. The authored route's
        // comment said "as the rectangle does" and the rectangle never did:
        // the A330's climb list named "taxi light, off" (EGPF, 2026-09-21).
        if !lights_done && alt_msl_ft.is_finite() && alt_msl_ft - field_elev_ft > LIGHTS_OFF_AGL_FT
        {
            lights_done = true;
            let did = crate::tools::lights::after_climb(&xp).await;
            crate::blackbox::record(
                "circuit",
                serde_json::json!({"event": "lights_climb", "off": did}),
            );
        }

        // The autopilot's own capture flies the level legs where the airframe
        // declares it. Failed reads are no measurement: no dial write, no
        // hold believed, and the VS law below carries on as it always has.
        let mut ap_level = false;
        if p_ap_capture && alt_msl_ft.is_finite() {
            if let (Ok(dial), Ok(hold)) = (&dial_r, &alt_hold_r) {
                match circuit_dial_ft(target_rate, target_alt, *dial, alt_msl_ft) {
                    Some(want) => {
                        let ok = autoflight::set_dial(&xp, Dial::Altitude, want).await.is_ok();
                        crate::blackbox::record(
                            "circuit",
                            serde_json::json!({"event": "alt_dial", "was": dial, "now": want,
                                               "leg": format!("{leg:?}"), "ok": ok}),
                        );
                    }
                    None => ap_level = ap_holds_level(target_rate, target_alt, *dial, *hold),
                }
            }
        }
        // The autopilot's answer, WATCHED (#181): a climb far over the dial
        // with the speed going, or one still on under the floor, is sent
        // down, and taken if it does not come down. The floor is the kept
        // flap's while cleaning up (`climb_floor`), the landing flap's on
        // base and final.
        let watch_floor = match leg {
            Leg::Base | Leg::Final => {
                crate::aircraft::config_floor_kts(last_flap.unwrap_or(0.0)).or(Some(climb_floor))
            }
            _ => Some(climb_floor),
        };
        let verdict = climb_watch.step(
            loop_t0.elapsed().as_secs_f64(),
            alt_msl_ft,
            ias_read,
            watch_floor,
            (!ap_level).then_some(vs_cmd),
        );
        if verdict != last_verdict {
            last_verdict = verdict;
            crate::blackbox::record(
                "circuit",
                serde_json::json!({"event": "climb_watch", "verdict": format!("{verdict:?}"),
                                   "leg": format!("{leg:?}"), "ias": ias_read,
                                   "vs": climb_watch.vs_fpm(), "cmd_vs": vs_cmd,
                                   "floor": watch_floor, "agl_ft": agl_ft}),
            );
        }
        if let crate::climb_watch::Verdict::TakeOver(why) = verdict {
            let hold_kts = crate::autoland::hand_back_speed_kts(
                p_approach_ias,
                last_flap.unwrap_or(0.0),
                &crate::aircraft::flap_limits(),
            );
            let trip = PatternTrip::ClimbNotHeld {
                why,
                ias: ias_read,
                vs: climb_watch.vs_fpm().unwrap_or(f64::NAN),
            };
            give_up_pattern(
                &xp, &base_url, &voice, trip, leg, alt_msl_ft, pattern_ft, mag_var, hold_kts,
                watch_floor,
            )
            .await;
            return;
        }
        vs_cmd = crate::climb_watch::dial_for(verdict, vs_cmd);
        // The autopilot's pitch AUTHORITY after an engage (#181), the
        // takeoff's carried here: the dial this loop flies, 0 while the
        // autopilot's capture holds the level.
        let auth_dial = if ap_level { 0.0 } else { vs_cmd };
        if let Some(crate::climb_watch::Authority::NoAuthority(_)) =
            crate::climb_watch::feed_authority(climb_watch.vs_fpm(), Some(auth_dial))
        {
            let hold_kts = crate::autoland::hand_back_speed_kts(
                p_approach_ias,
                last_flap.unwrap_or(0.0),
                &crate::aircraft::flap_limits(),
            );
            let trip = PatternTrip::ClimbNotHeld {
                why: crate::climb_watch::Why::NoAuthority,
                ias: ias_read,
                vs: climb_watch.vs_fpm().unwrap_or(f64::NAN),
            };
            give_up_pattern(
                &xp, &base_url, &voice, trip, leg, alt_msl_ft, pattern_ft, mag_var, hold_kts,
                watch_floor,
            )
            .await;
            return;
        }
        if ap_level {
            // Whatever the VS dial says now is stale the moment VS re-engages.
            last_vs_cmd = None;
            vs_refused = None;
        } else {
            if autoflight::bring_vs(&xp, vs_read, "circuit_rectangle")
                .await
                .pressed(Button::VerticalSpeed)
            {
                // The engage synced the dial: a rewrite is forced.
                last_vs_cmd = None;
                // A vertical mode engaged: its authority is checked afresh.
                if let Some(vs) = climb_watch.vs_fpm() {
                    crate::climb_watch::arm_authority("circuit", vs, vs_cmd);
                }
            }
            if last_vs_cmd.is_none_or(|v| (v - vs_cmd).abs() > 40.0) {
                let _ = autoflight::set_dial(&xp, Dial::Vs, vs_cmd).await;
                last_vs_cmd = Some(vs_cmd);
            }
            // The one thing the capture rests on: VS taking the aircraft back
            // OUT of the hold. An autopilot that will not is said, once, while
            // there is still a downwind's worth of height to act on.
            if p_ap_capture && !vs_on {
                let since = *vs_refused.get_or_insert_with(std::time::Instant::now);
                if vs_stuck(since.elapsed().as_secs_f64()) && !vs_refused_said {
                    vs_refused_said = true;
                    crate::blackbox::record(
                        "circuit",
                        serde_json::json!({"event": "vs_refused", "leg": format!("{leg:?}"),
                                           "cmd_vs": vs_cmd}),
                    );
                    alert(
                        &voice,
                        Priority::Warning,
                        "The autopilot won't take vertical speed. Check the vertical mode.",
                    );
                }
            } else {
                vs_refused = None;
            }
        }

        // --- Power for speed: the throttle holds the pattern speed (~Vy) while
        // the VS law above flies altitude. Left at climb power the Twin Otter
        // ran the whole pattern up to Vne (202 KIAS), ~3x the planned turn
        // radius, straight through the Spanish airspace choose_pattern had tried
        // to avoid (LXGB, 2026-07-26). The flight speed-hold law, folded into
        // this tick; autoland re-takes the throttle at the handoff below.
        let mut ias_now = f64::NAN;
        if let (Ok(ias), Ok(thr)) = (ias_r, thr_r) {
            ias_now = ias;
            let t_now = std::time::Instant::now();
            if let Some((prev, t_prev)) = prev_ias {
                let dt = t_now.duration_since(t_prev).as_secs_f64();
                if (0.1..=2.0).contains(&dt) {
                    ias_trend = 0.6 * ias_trend + 0.4 * (ias - prev) / dt;
                }
            }
            prev_ias = Some((ias, t_now));
            // Configure for landing: base toward the approach flap, final to
            // full, each notch only once the speed is under its placard
            // (`flap_within_limits`, autoland's own law). Airframes with no
            // flap profile are left alone.
            if matches!(leg, Leg::Base | Leg::Final) {
                // A go-around's clean-up not finished by base: base owns the
                // flaps from here.
                cleaning_up = None;
                let limits = crate::aircraft::flap_limits();
                if !limits.is_empty() {
                    let wanted = if matches!(leg, Leg::Final) {
                        1.0
                    } else {
                        BASE_FLAP_RATIO
                    };
                    let flap = crate::autoland::flap_within_limits(wanted, ias, &limits);
                    if last_flap != Some(flap) {
                        last_flap = Some(flap);
                        let _ = crate::tools::flaps::set_handle(&xp, flap).await;
                        crate::blackbox::record(
                            "circuit",
                            serde_json::json!({"event": "flaps", "leg": format!("{leg:?}"), "ratio": flap, "ias": ias}),
                        );
                    }
                }
            }
            // A go-around cleans up on the airframe's retraction schedule,
            // one detent at a time, paced on the flaps arriving.
            if let Some(cmd) = cleaning_up {
                let clean_ok = crate::aircraft::config_floor_kts(0.0).unwrap_or(pattern_kts * 0.9);
                // The step's speed inside the placard of what is out (#169).
                let flap_schedule =
                    cleanup_schedule(&flap_schedule, &crate::aircraft::flap_limits(), cmd);
                let due = if flap_schedule.is_empty() {
                    go_around_flap_step(&flap_schedule, cmd, None, ias, clean_ok)
                } else {
                    autotakeoff::flap_retract_step(&flap_schedule, cmd, ias)
                };
                if due.is_some() {
                    // Read only while a step is due; a failed read is no
                    // measurement, and no step is taken on it.
                    let deployed = xp.get_float(crate::takeoff_config::FLAP_DEPLOY).await.ok();
                    let t_s = pace_t0.elapsed().as_secs_f64();
                    let judged = flap_pace.judged(t_s, cmd, deployed);
                    let step = go_around_flap_step(&flap_schedule, cmd, judged, ias, clean_ok);
                    if step.is_none() && flap_pace.withheld_log_due(t_s) {
                        crate::blackbox::record(
                            "circuit",
                            serde_json::json!({"event": "flap_step_withheld", "at": cmd,
                                               "ias": ias, "agl_ft": agl_ft,
                                               "deployed": deployed}),
                        );
                    }
                    if let Some(next) = step {
                        flap_pace.stepped(cmd);
                        let ok = crate::tools::flaps::set_handle(&xp, next).await.is_ok();
                        last_flap = Some(next);
                        cleaning_up = (next > 1e-6).then_some(next);
                        let detent = crate::aircraft::live_profile().and_then(|lp| {
                            crate::aircraft::flap_detent_spoken(lp.flaps.as_ref(), next)
                        });
                        crate::blackbox::record(
                            "circuit",
                            serde_json::json!({"event": cleanup_event, "to": next,
                                               "ias": ias, "agl_ft": agl_ft, "deployed": deployed,
                                               "ok": ok}),
                        );
                        alert(
                            &voice,
                            Priority::Advisory,
                            &autotakeoff::flap_step_line(next, detent.as_deref()),
                        );
                    }
                }
            }
            // The go-around's gear, once climbing and clear of its own
            // warning (the takeoff's rule). The downwind lowers it again.
            if gear_up_owed
                && vs_est > 0.0
                && autotakeoff::gear_up_due(
                    agl_ft,
                    ias,
                    last_flap.unwrap_or(0.0),
                    autotakeoff::gear_warning(),
                    crate::aircraft::gear_limit_kts(),
                )
            {
                gear_up_owed = false;
                let ok = crate::tools::gear::set_handle(&xp, true).await.is_ok();
                if ok {
                    gear_lowered = false;
                }
                crate::blackbox::record(
                    "circuit",
                    serde_json::json!({"event": "go_around_gear_up", "ias": ias,
                                       "agl_ft": agl_ft, "ok": ok}),
                );
                alert(&voice, Priority::Advisory, "Gear up.");
            }
            // The leg's speed, the same number the climb limiter was fed.
            let delta = crate::tools::speedhold::throttle_adjust_live(want_kts - ias, ias_trend);
            if delta != 0.0 {
                let _ = xp
                    .set_float(&throttle_dataref(), (thr + delta).clamp(0.0, 1.0))
                    .await;
            }
        }

        DETAIL.set(circuit_detail(
            leg, &rwy_id, alt_msl_ft, target_alt, ias_now,
        ));
        crate::auto_status::set_ap_targets(Some(bug_mag), Some(target_alt), Some(vs_cmd));
        crate::blackbox::record(
            "circuit",
            serde_json::json!({
                "event": "leg", "leg": format!("{leg:?}"), "agl_ft": agl_ft,
                "a_m": a_m, "x_m": x_m, "crab": crab,
                "bug": bug_mag, "tgt": target_mag, "psi": psi_v,
                "hdg_on": hdg_on, "vs_on": vs_on, "ap_level": ap_level, "cmd_vs": vs_cmd,
                "law_vs": law_vs, "carry": climb_carry.carrying(), "flap": last_flap,
                "alt_msl": alt_msl_ft, "target_alt": target_alt,
                "ias": ias_now, "pat_kts": pattern_kts, "want_kts": want_kts,
                "climb_floor": climb_floor,
                // The climb the watch judges on (#181).
                "vs": climb_watch.vs_fpm(),
                // The descent authority and the height over the path (#188).
                "descent_fpm": descent_fpm, "above_path": above_path,
            }),
        );

        if !pattern_advance(leg, a_m, x_m, &geom, rwy.length_m, left) {
            continue;
        }
        // Too high for the rest of the pattern to meet the path: the
        // downwind carries on (#188), said once, bounded by
        // MAX_DOWNWIND_EXTEND_M.
        if leg == Leg::Downwind
            && extend_downwind(
                alt_msl_ft,
                field_elev_ft,
                tpa_msl_ft,
                a_m,
                x_m,
                &geom,
                left,
                descent_shape,
                handoff_gs,
                descent_cap,
            )
        {
            if extending.is_none() {
                extending = Some(a_m);
                crate::blackbox::record(
                    "circuit",
                    serde_json::json!({"event": "downwind_extended", "a_m": a_m, "x_m": x_m,
                                       "alt_msl": alt_msl_ft, "above_path": above_path,
                                       "descent_cap_fpm": descent_cap,
                                       "max_extend_m": MAX_DOWNWIND_EXTEND_M}),
                );
                alert(
                    &voice,
                    Priority::Advisory,
                    "High for the approach. Extending the downwind.",
                );
            }
            continue;
        }
        if leg == Leg::Downwind {
            if let Some(from) = extending.take() {
                crate::blackbox::record(
                    "circuit",
                    serde_json::json!({"event": "downwind_extension_end",
                                       "extended_m": (a_m.abs() - from.abs()).max(0.0),
                                       "alt_msl": alt_msl_ft, "above_path": above_path}),
                );
            }
        }
        let Some(nxt) = next_leg(leg) else { continue };
        leg = nxt;

        // The wind-corrected bug is written every tick above; the transitions
        // just announce, report, and change altitude / hand off.
        match leg {
            Leg::Crosswind => {
                alert(&voice, Priority::Advisory, "Crosswind.");
            }
            Leg::Downwind => {
                alert(&voice, Priority::Advisory, "Downwind.");
                // The copilot reports the pattern position to the tower.
                transmit(radio_call(
                    &callsign,
                    &format!("downwind runway {}", spell_runway(&rwy_id)),
                ));
            }
            Leg::Base => {
                // Turn base and start down — the VS law above flies the
                // descent once the leg target drops to the handoff height.
                alert(&voice, Priority::Advisory, "Base. Descending.");
                transmit(radio_call(&callsign, "turning base"));
            }
            Leg::Final => {
                // ARM the turn onto final; do NOT hand over here. Reaching
                // `final_capture_m` means the base turn may BEGIN, not that
                // the aircraft is on the centreline — it is still 1.3 radii
                // abeam at that instant. Handing autoland an aircraft 2,815 m
                // off with 2,421 m to run gave it a 49° angle it could not
                // close, and it crossed the threshold 188 m off the paint
                // (EGHI, 2026-07-30). The circuit flies its own turn now and
                // lets go when the geometry is actually flyable.
                alert(
                    &voice,
                    Priority::Advisory,
                    crate::readback::one_of(&[
                        "Turning final. Landing us.",
                        "On final now. I'll put her down.",
                        "Turning final. Bringing us in.",
                    ]),
                );
                crate::blackbox::record(
                    "circuit",
                    serde_json::json!({
                        "event": "final_turn", "capture_m": geom.final_capture_m,
                        "bank_measured": base_bank.steady_bank_deg(),
                        "x_m": x_m, "a_m": a_m,
                    }),
                );
                // The final report — this is what earns the landing clearance.
                transmit(radio_call(
                    &callsign,
                    &format!("final runway {}", spell_runway(&rwy_id)),
                ));
                turning_final = true;
            }
            Leg::Upwind => {}
        }
    }
}

/// The arrival-relevant slice of [`Params`] — passed as fields because by the
/// time the arrival runs, `p.takeoff` has been moved into the takeoff loop and
/// `&p` can no longer be borrowed whole.
pub(super) struct ArrivalCtx<'a> {
    pub(super) xp: &'a XPlaneClient,
    pub(super) apt_dat: &'a str,
    pub(super) cifp_dir: &'a str,
    pub(super) fix_data_path: &'a str,
    pub(super) nav_data_path: &'a str,
    pub(super) cfg: &'a ApConfig,
    pub(super) approach_ias: f64,
}

/// Try to hand the arrival to the airport's PUBLISHED approach (the terrain-safe
/// path) instead of a raw autoland. Builds the approach plan off the realtime
/// loop, and only takes it when [`crate::procedure::approach_flyable_from`] says
/// we're positioned to fly it — outside and above the FAF. Returns whether the
/// procedure follower took over. A tight circuit (inside the FAF, at pattern
/// height) always returns false, so autoland flies the short final unchanged.
#[allow(clippy::too_many_arguments)]
pub(super) async fn try_published_arrival(
    arr: ArrivalCtx<'_>,
    icao: &str,
    rwy_id: &str,
    base_url: &str,
    voice: &Option<Voice>,
    lat: f64,
    lon: f64,
    alt_msl: f64,
) -> bool {
    if arr.cifp_dir.is_empty() {
        return false;
    }
    let (apt_dat, cifp, fixp, navp) = (
        arr.apt_dat.to_string(),
        arr.cifp_dir.to_string(),
        arr.fix_data_path.to_string(),
        arr.nav_data_path.to_string(),
    );
    let (icao, rwy_id, cfg, ias) = (
        icao.to_string(),
        rwy_id.to_string(),
        arr.cfg.clone(),
        arr.approach_ias,
    );
    // find_airport + procedure load/resolve read big files — off the reactor.
    let plan = tokio::task::spawn_blocking(move || {
        let airport = crate::nav::find_airport(&apt_dat, &icao).ok().flatten()?;
        let end = crate::procedure::EndAction::Approach {
            apt_dat: apt_dat.clone(),
            cfg,
            approach_ias: ias,
            rwy_id: Some(crate::runway::PlannedRunway::at(&icao, &rwy_id)),
        };
        crate::procedure::approach_plan_for(
            &cifp, &fixp, &navp, &airport, &icao, &rwy_id, lat, lon, end,
        )
        .ok()
    })
    .await
    .ok()
    .flatten();
    let Some(plan) = plan else {
        return false;
    };
    if !crate::procedure::approach_flyable_from(&plan, lat, lon, alt_msl) {
        crate::blackbox::record(
            "circuit",
            serde_json::json!({"event": "approach_declined", "reason": "inside_or_below_faf"}),
        );
        return false;
    }
    crate::blackbox::record(
        "circuit",
        serde_json::json!({"event": "handoff_procedure", "name": plan.name}),
    );
    alert(
        voice,
        Priority::Advisory,
        &format!("Joining {}.", plan.name),
    );
    crate::procedure::start_scoped(
        base_url.to_string(),
        voice.clone(),
        plan,
        arr.xp.cancellation(),
    )
}
