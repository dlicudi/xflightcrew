//! The route flown: `fly_route`, whole, and its numbers.

use super::*;

pub(super) const LAT: &str = "sim/flightmodel/position/latitude";
pub(super) const LON: &str = "sim/flightmodel/position/longitude";
pub(super) const AGL_M: &str = "sim/flightmodel/position/y_agl";
/// The altimeter, not true elevation: the check-in's "passing" and the arrival's
/// platform altitudes are what the controller and the chart mean.
pub(super) const ALT_IND_FT: &str = crate::datarefs::ALTITUDE_FT;
pub(super) const M_TO_FT: f64 = 3.280_84;
pub(super) const MS_TO_KTS: f64 = 1.943_844;
/// Below this surface wind the arrival picks its runway by geometry (the end
/// most aligned with our bearing to the field), not wind: a light or variable
/// wind isn't worth a big turn onto final. At or above it the wind decides —
/// landing INTO it — because a real tailwind is what put the arrival on
/// Gibraltar's 09 into 17 kt while the tower worked 27 (2026-07-24).
pub(crate) const WIND_DECISIVE_KTS: f64 = 4.0;

/// An airborne AGL below this after the takeoff loop exits means it REJECTED
/// (never flew) — same reading the circuit uses.
pub(super) const AIRBORNE_MIN_AGL_FT: f64 = 300.0;

/// A resume starts only this far off the ground: a lower aircraft is
/// landing or has landed, never one the planner flies from.
pub(super) const RESUME_MIN_AGL_FT: f64 = 100.0;

/// How long the circuit's taxi-in stage waits for autoland to ARM before
/// concluding the pattern never got that far. Generous: a takeoff, a climb to
/// pattern altitude, a full rectangle and a base leg, at trainer speeds, with
/// a go-around's worth of slack on top.
pub(super) const ARM_WAIT_SECS: f64 = 900.0;

/// Stage 4 of a Flight mission: the takeoff, the enroute fixes at the plan's
/// altitudes, then the destination's published approach with autoland as the
/// fallback — every phase one of the proven loops, watched in sequence.
pub(super) async fn fly_route(
    xp: &XPlaneClient,
    config: &Config,
    voice: &Option<Voice>,
    ctx: &ToolCtx<'_>,
    tools: &SubTools,
    route: &Route,
    callsign: &Option<String>,
) {
    use std::sync::atomic::Ordering::Relaxed;

    // (The GPS already has the route: fly() writes the FMS mailbox at arm
    // time, at the gate, so the taxi happens with the plan on the map.)

    // Checked in with departure during the takeoff's climb: the clearance's
    // ceiling has done its job and is not put back after the takeoff.
    let mut checked_in = false;
    if !matches!(tools.mission, Mission::Resume(_)) {
        // --- Takeoff: the proven loop, armed through its tool so every arm-time
        // gate applies (the pitch-channel probe included).
        announce(
            voice,
            Priority::Advisory,
            &format!("Cleared to go. Flying your plan to {}.", route.dest_name),
        );
        // An IFR clearance's altitude is the level the takeoff selects on
        // the dial before the roll, and the climb-out's target.
        let level_ft = crate::party_line::assigned_clearance()
            .filter(|_| route.ifr)
            .map(|c| c.initial_alt_ft as f64);
        if let Ok(reply) = tools
            .takeoff
            .execute(&json!({ "level_ft": level_ft }), ctx)
            .await
        {
            announce(voice, Priority::Advisory, &reply);
        }
        if !crate::autotakeoff::ACTIVE.load(Relaxed) {
            give_up(
                xp,
                tools.taxi.park_brake_valve_set,
                voice,
                "The takeoff didn't arm, so the flight is off. Your aircraft.",
                "takeoff_refused",
            )
            .await;
            return;
        }
        waiting("the takeoff");
        // The takeoff's climb works the radio too: the tower hands us to
        // departure at field + 700 ft, long before the takeoff hands over
        // (#176, EGKK 08R: read back 37 s late at 2,760 ft AGL, after the
        // takeoff's handoff).
        let climb_to_ft = check_in_climb_ft(&route.legs, route.cruise_ft);
        let obey = move || async move {
            let Some((to, khz)) = crate::party_line::take_departure_handoff() else {
                return false;
            };
            event("departure_handoff", json!({ "during": "takeoff", "to": &to, "khz": khz }));
            obey_departure_handoff(xp, callsign, to, khz, climb_to_ft).await;
            true
        };
        let wait = wait_takeoff_working_the_radio(
            360.0,
            TAKEOFF_POLL_MS,
            || async { crate::autotakeoff::running() },
            obey,
        )
        .await;
        checked_in = wait.checked_in;
        if !wait.finished {
            if !ABORT.load(SeqCst) {
                give_up(
                    xp,
                    tools.taxi.park_brake_valve_set,
                    voice,
                    "The takeoff never finished. Your aircraft.",
                    "takeoff_stalled",
                )
                .await;
            } else {
                give_up(
                    xp,
                    tools.taxi.park_brake_valve_set,
                    voice,
                    "Cancelled. Your aircraft.",
                    "abort",
                )
                .await;
            }
            return;
        }
        let outcome = crate::autotakeoff::outcome();
        if outcome != Some(sim::run::Outcome::Completed) {
            // A reject has stopped, parked and said "your aircraft" itself.
            let line = match outcome {
                // The recovery has the aircraft and has said why (#181).
                Some(sim::run::Outcome::Failed(r)) if r == crate::autotakeoff::AP_NO_PITCH => {
                    "The flight is off. I have the aircraft until you say my controls."
                }
                Some(sim::run::Outcome::Failed(_)) => {
                    "The takeoff was rejected, so the flight is off."
                }
                _ => "The takeoff handed control back. Your aircraft.",
            };
            give_up(
                xp,
                tools.taxi.park_brake_valve_set,
                voice,
                line,
                crate::circuit::TAKEOFF_NOT_COMPLETED,
            )
            .await;
            return;
        }
        let agl_ft = xp.get_float(AGL_M).await.unwrap_or(0.0) * M_TO_FT;
        if agl_ft < AIRBORNE_MIN_AGL_FT {
            give_up(
                xp,
                tools.taxi.park_brake_valve_set,
                voice,
                "The takeoff didn't get us airborne, so the flight is off.",
                "takeoff_rejected",
            )
            .await;
            return;
        }
        event("airborne", json!({ "agl_ft": agl_ft }));
        DEPARTURE_MINIMUMS.store(false, SeqCst);
    } else {
        let (ground, agl) = tokio::join!(
            xp.get_float(crate::datarefs::ON_GROUND_ANY),
            xp.get_float(AGL_M)
        );
        // Still in the air, the state the planner judged: low is no reason
        // (its first leg climbs to the way's floor, LEMG 2026-10-05); on the
        // wheels or an unread flag is.
        if !matches!((ground,agl), (Ok(g),Ok(h)) if g.is_finite() && h.is_finite() && g < 0.5 && h*M_TO_FT >= RESUME_MIN_AGL_FT)
        {
            give_up(
                xp,
                tools.taxi.park_brake_valve_set,
                voice,
                "Aircraft state changed before resume. I'm monitoring only; you have control.",
                "resume_state_changed",
            )
            .await;
            return;
        }
        event(
            "resumed",
            json!({ "fix": route.legs.first().map(|l| &l.fix_ident) }),
        );
        // Reload only after the explicit resume; fms_goto selects the joined fix.
        let _ = xp
            .set_data("xcopilot/fms/route", route.fms_payload.as_bytes())
            .await;
        let _ = xp.set_int("xcopilot/fms/load", 1).await;
    }

    // --- Enroute: the procedure follower over the plan's own fixes. It climbs
    // to cruise on the first constraint and flies the plan's descent numbers
    // after TOD; quiet completion (EndAction::Enroute) hands back here.
    // Under an IFR clearance the climb stops at its altitude until the
    // departure controller's check-in clears higher (the ceiling is let go
    // at the check-in, `obey_departure_handoff`).
    let ceiling = crate::party_line::assigned_clearance()
        .filter(|_| route.ifr && !matches!(tools.mission, Mission::Resume(_)))
        .filter(|_| !checked_in)
        .map(|c| c.initial_alt_ft as f64);
    crate::procedure::hold_at_cleared(ceiling, "ifr_clearance");
    if !crate::procedure::start_scoped(
        config.xplane_base_url.clone(),
        voice.clone(),
        crate::procedure::Plan {
            name: format!("the flight plan to {}", route.dest_name),
            legs: route.legs.clone(),
            end: crate::procedure::EndAction::Enroute,
            // The climb to cruise runs on the speed-protected law — the
            // unprotected rate stalled the first LEJR climb at 53 knots.
            vy_kts: tools.takeoff.vy_kts,
            cruise_ias_kts: tools.cruise_ias_kts,
            transition_altitude_ft: Some(tools.transition_altitude_ft),
            climb_minimums: route.climb_minimums.clone(),
            arrival_constraints: route.arrival_constraints.clone(),
            visual_deg: None,
            missed_legs: Vec::new(),
            cruise_schedule: route.cruise_schedule.clone(),
            // The arrival starts at autoland's clean approach speed, slowed
            // for by the end of the route (#199: EGNX 09 was handed a clean
            // 737 at 252 kt).
            arrival_handoff_kts: Some(crate::autoland::clean_approach_ias(
                tools.circuit.approach_ias,
            )),
        },
        xp.cancellation(),
    ) {
        crate::procedure::hold_at_cleared(None, "procedure_busy");
        give_up(
            xp,
            tools.taxi.park_brake_valve_set,
            voice,
            "The previous procedure is still stopping. Your aircraft.",
            "procedure_busy",
        )
        .await;
        return;
    }
    // The CLIMB and CRUISE checklists' mixture items, flown rather than read:
    // the continuous manager keeps it full rich below the leaning floor and
    // leans to peak EGT above it, re-trimming as the climb and the power
    // settle. The pilot's hand on the lever, or "my mixture", releases it.
    // Armed silently: the manager says what it did when it first leans
    // (`lean_seek`'s own line), and this moment already carries the
    // thousand-foot callout, "flying the plan" and the handoff readback
    // (four lines in one second at LXGB, 2026-09-15).
    if tools.piston_mixture {
        crate::tools::engine::arm_mixture_manager(config.xplane_base_url.clone());
        event("mixture_managed", json!({}));
    }
    // Generous ceiling — ABORT bails out of the wait immediately. The wait
    // works the radio: the tower's after-takeoff handoff is owed a readback,
    // a retune and a check-in.
    let climb_to_ft = check_in_climb_ft(&route.legs, route.cruise_ft);
    // The destination's own position and tower, for the inbound call at the
    // zone approach: the apt.dat row, never `home_tower_khz`, which is
    // whoever the region last handed us to.
    let dest_tower = crate::nav::find_airport(&crate::config::apt_dat_path(), &route.dest_icao)
        .ok()
        .flatten()
        .and_then(|a| {
            let (lat, lon) = a.position()?;
            let khz = a.staffed_tower()?.khz;
            Some((lat, lon, khz))
        });
    let mut arrival = ArrivalRepick::of(route, &crate::config::apt_dat_path());
    if !wait_enroute_working_the_radio(xp, ctx, voice, callsign, climb_to_ft, dest_tower, &mut arrival).await {
        if !ABORT.load(SeqCst) {
            give_up(
                xp,
                tools.taxi.park_brake_valve_set,
                voice,
                "The enroute leg never completed. Your aircraft.",
                "enroute_stalled",
            )
            .await;
        } else {
            give_up(
                xp,
                tools.taxi.park_brake_valve_set,
                voice,
                "Cancelled. Your aircraft.",
                "abort",
            )
            .await;
        }
        return;
    }
    if ABORT.load(SeqCst) {
        give_up(
            xp,
            tools.taxi.park_brake_valve_set,
            voice,
            "Cancelled. Your aircraft.",
            "abort",
        )
        .await;
        return;
    }
    event("enroute_complete", json!({}));
    // The installed airport data has no staffed tower at the destination.
    // End the last radar service HERE, at the enroute/arrival seam: leaving
    // its station live made Cardiff answer Swansea's inbound call, assign its
    // own runway 30, and block taxi-in waiting for a clearance that could not
    // exist (EGTF-EGFH, 2026-10-01, #210).
    if dest_tower.is_none() && crate::mode::atc_enabled() {
        crate::party_line::begin_uncontrolled_arrival(&route.dest_icao);
        event(
            "uncontrolled_arrival",
            json!({ "destination": route.dest_icao, "why": "no_staffed_tower" }),
        );
        announce(
            voice,
            Priority::Advisory,
            &format!(
                "{} has no tower in the scenery data. Radar service terminated; continuing as an uncontrolled arrival.",
                route.dest_icao
            ),
        );
    }
    // Whatever the cruise did with them, the signs are on for the arrival (a
    // short sector never crossed ten thousand; nothing pressed when already on).
    let _ = crate::tools::signs::set(xp, true).await;
    // The BEFORE LANDING item: full rich for the arrival, whatever the field
    // elevation (the manager's own floor would leave a leaned lever over a
    // high field), before the approach or the visual landing is built.
    if tools.piston_mixture {
        match crate::tools::engine::mixture_rich_for_landing(xp).await {
            Some(moved) => {
                event("mixture_rich", json!({ "moved": moved }));
                if moved {
                    announce(voice, Priority::Advisory, "Mixture full rich.");
                }
            }
            None => event("mixture_rich", json!({ "moved": false, "unread": true })),
        }
    }

    // --- Arrival: prefer the destination's PUBLISHED approach (the terrain-
    // safe corridor, the circuit's arrival rule); a visual autoland otherwise.
    let (Ok(lat), Ok(lon)) = (xp.get_float(LAT).await, xp.get_float(LON).await) else {
        give_up(
            xp,
            tools.taxi.park_brake_valve_set,
            voice,
            "I can't read our position for the arrival. Your aircraft.",
            "no_position",
        )
        .await;
        return;
    };
    let alt_msl_ft = xp.get_float(ALT_IND_FT).await.unwrap_or(0.0);
    // The surface wind AT THE FIELD chooses the landing runway (see
    // build_arrival): the destination's METAR first, the aircraft-wind
    // datarefs only without one. The datarefs are the wind HERE, and here is
    // 30 nm out at 7,000 ft: LEJR-LXGB read a light westerly aloft, set up
    // the straight-in to 27, and the tower's "negative, runway zero niner,
    // wind zero niner zero degrees, two one knots" came with the landing
    // clearance (2026-09-14).
    let aloft = match (
        xp.get_float(crate::datarefs::WIND_DIR_DEG_TRUE).await,
        xp.get_float(crate::datarefs::WIND_SPEED_MSC).await,
    ) {
        (Ok(dir), Ok(spd_ms)) => Some((dir, spd_ms * MS_TO_KTS)),
        _ => None,
    };
    let metar = crate::weather::surface_wind(&crate::config::xplane_root(), &route.dest_icao);
    let (wind, wind_source) = arrival_wind(metar, aloft);
    let wind_detail = json!({
        "source": wind_source,
        "dir": wind.map(|(d, _)| d.round()),
        "kts": wind.map(|(_, k)| k.round()),
    });
    // The runway chosen at the arm, flown unless the arrival's own safety
    // rules overrule it (said) or the tower names another (said).
    let planned = arrival_runway();
    let build = |forced: Option<String>| {
        let planned = planned.clone();
        let (apt_dat, cifp, fixp, navp) = (
            crate::config::apt_dat_path(),
            config.cifp_dir.clone(),
            config.fix_data_path.clone(),
            config.nav_data_path.clone(),
        );
        let dest = route.dest_icao.clone();
        let cfg = tools.circuit.cfg.clone();
        let ias = tools.circuit.approach_ias;
        async move {
            tokio::task::spawn_blocking(move || {
                build_arrival(
                    &apt_dat, &cifp, &fixp, &navp, &dest, lat, lon, cfg, ias, wind,
                    forced.as_deref(),
                    planned.as_deref(),
                )
            })
            .await
            .unwrap_or(Arrival {
                plan: None,
                rwy_id: None,
                tower_khz: None,
                mountain_msa_ft: None,
                overruled: None,
            })
        }
    };
    let mut built = {
        build(None).await
    };
    if let (Some(was), Some(why)) = (planned.as_deref(), built.overruled) {
        event(
            "arrival_runway_overruled",
            json!({ "planned": was, "now": built.rwy_id, "why": why, "wind": wind_detail }),
        );
        if let Some(line) = overruled_line(was, built.rwy_id.as_deref(), why) {
            announce(voice, Priority::Advisory, &line);
        }
    }
    set_arrival_runway(built.rwy_id.clone(), "arrival");
    if route
        .local_runway
        .as_ref()
        .is_some_and(|r| built.rwy_id.as_ref() != Some(r))
    {
        give_up(
            xp,
            tools.taxi.park_brake_valve_set,
            voice,
            "The arrival runway changed. The local route needs replanning. Your aircraft.",
            "local_runway_changed",
        )
        .await;
        return;
    }
    // The radio: tune the DESTINATION'S tower from its own apt.dat comm row —
    // never `home_tower_khz`, which is whoever the enroute region last handed
    // us to. At LINTO that was Tangier (genuinely the nearest field,
    // mid-strait), and the inbound call earned "join final runway two eight"
    // from Ibn Battouta Tower while we set up for Gibraltar's zero nine
    // (17:32Z, 2026-07-24). The region hands the STATION over as we close;
    // with COM1 already on the destination's frequency the handoff finds us
    // listening.
    let mut inbound_owed: Option<i64> = None;
    if let Some(khz) = built.tower_khz {
        let _ = xp.set_float(COM1_FREQ_KHZ, khz as f64).await;
        // You call it, you get it - for the crew's own retune as much as the
        // pilot's. The enroute region weighs a field by its published
        // services, so a tower-only strip beside a primary never wins the
        // sky over its own runway (LEAX 123.5 against Malaga's five rows:
        // both the inbound and the final went into the void on a frequency
        // no station held, 2026-09-15). Tuning the destination's tower arms
        // that field the way a keyed transmission on its frequency does; the
        // swap wait below then finds it on the next loop pass.
        crate::party_line::arm_station_for_freq(khz, lat, lon, alt_msl_ft, Instant::now());
        // And HOLD it there for the rest of the arrival: arming only wins the
        // next swap, where the region's tick keeps running all the way to the
        // flare and will hand a nearer field the station we are talking to.
        crate::party_line::pin_arrival_station(Some(khz));
        // The call is worth nothing until the station on that frequency IS
        // the destination: at PIMOS the region swapped to Gibraltar three
        // seconds AFTER "inbound for landing" went out, the call went into
        // the void, and Gibraltar's zone watch challenged the arrival as an
        // unknown aircraft (12:36Z, 2026-09-11). The swap is distance-driven
        // and due about now; wait for it, bounded, and call either way.
        // Already made by the enroute leg (the region handed us to this
        // tower inside INBOUND_CALL_NM): nothing owed, the tower knows us.
        let swapped = INBOUND_MADE.load(SeqCst) || {
            waiting("the destination's frequency");
            wait_until(INBOUND_SWAP_WAIT_SECS, 500, || async {
                crate::party_line::home_tower_khz() == Some(khz)
            })
            .await
        };
        if INBOUND_MADE.load(SeqCst) {
            event("inbound_already_made", json!({ "khz": khz }));
        } else if swapped {
            call_inbound(xp, ctx, voice, callsign, khz, true, built.rwy_id.as_deref()).await;
        } else {
            // Not yet the destination's frequency: the call would go into
            // the void (LEJR-LXGB, 2026-09-14: made 10 minutes before the
            // swap from Ibn Battouta, the nearest field for most of the
            // strait, so Gibraltar never heard an inbound and the final
            // report was its first word from us). Owed at the swap, made
            // below while the arrival flies; before the final report at the
            // latest.
            inbound_owed = Some(khz);
            event("inbound_deferred", json!({ "khz": khz }));
        }
    }
    // The TOWER's runway, once the inbound call has one. LOWI (2026-09-24):
    // "join final runway two six" came back two seconds after the crew had
    // armed the 08 approach, and nothing reconciled them. The tower's end is
    // flown when there is an approach to it we can fly; otherwise ours stands
    // and the crew says why (autoland still watches the landing clearance).
    if let Some(tower_rwy) = tower_arrival_runway(built.tower_khz).await {
        if built.rwy_id.as_deref() != Some(tower_rwy.as_str()) {
            let theirs = build(Some(tower_rwy.clone())).await;
            let spoken = crate::crew_radio::spell_runway(&tower_rwy);
            if theirs.plan.is_some() {
                event(
                    "arrival_runway_from_tower",
                    json!({ "was": built.rwy_id, "now": tower_rwy }),
                );
                announce(
                    voice,
                    Priority::Advisory,
                    &format!("Tower wants runway {spoken}. Setting up for it."),
                );
                set_arrival_runway(Some(tower_rwy.clone()), "tower");
                built = theirs;
            } else {
                event(
                    "tower_runway_unflyable",
                    json!({ "ours": built.rwy_id, "tower": tower_rwy }),
                );
                announce(
                    voice,
                    Priority::Caution,
                    &format!(
                        "Tower wants runway {spoken}, but there's no approach I can fly to it. \
                         Staying with runway {}.",
                        built
                            .rwy_id
                            .as_deref()
                            .map(crate::crew_radio::spell_runway)
                            .unwrap_or_default()
                    ),
                );
            }
        }
    }
    // The wind-chosen end (build_arrival) drives BOTH the published-approach
    // branch and the visual fallback — the visual autoland is forced onto the
    // same runway so a tailwind can't be re-picked back in by track.
    // The runway AND the field it belongs to. A bare ident is resolved by
    // proximity, and on the EGPH arrival that found EGPJ's 24 nine miles
    // away across the Forth (2026-09-20) — a plan that names its destination
    // must never let the airport be a search result.
    let landing_rwy = built
        .rwy_id
        .as_deref()
        .map(|id| crate::runway::PlannedRunway::at(&route.dest_icao, id));
    match built.plan {
        Some(plan)
            if route.local_runway.is_none()
                && crate::procedure::approach_flyable_from(&plan, lat, lon, alt_msl_ft) =>
        {
            event(
                "arrival_procedure",
                json!({ "name": plan.name, "runway": built.rwy_id, "wind": wind_detail }),
            );
            announce(
                voice,
                Priority::Advisory,
                &format!("Setting up {}.", plan.name),
            );
            // The ILS on NAV1 now, so the needles are alive for the whole
            // approach, not only autoland's final (`ils::tune`).
            if let Some(id) = built.rwy_id.as_deref() {
                let _ = crate::ils::tune(xp, &route.dest_icao, id, lat, lon).await;
            }
            if !crate::procedure::start_scoped(
                config.xplane_base_url.clone(),
                voice.clone(),
                plan,
                xp.cancellation(),
            ) {
                give_up(
                    xp,
                    tools.taxi.park_brake_valve_set,
                    voice,
                    "The previous procedure is still stopping. Your aircraft.",
                    "procedure_busy",
                )
                .await;
                return;
            }
        }
        _ if route.local_runway.is_none() && built.mountain_msa_ft.is_some() => {
            // A mountain field with no approach we can fly: never a visual
            // autoland into the valley. The follower has finished the route;
            // hold at or above the sector altitude and hand over, saying why.
            // (LSZS, 2026-09-24: its RNAV approaches end at a missed-approach
            // point before the runway; the old fallback was a visual autoland
            // into the Engadin with 15,500 ft sectors.)
            let msa = built.mountain_msa_ft.unwrap_or(0.0);
            let hold_ft = (alt_msl_ft.max(msa + 500.0) / 500.0).ceil() * 500.0;
            let thr = crate::aircraft::throttle_dataref();
            let _ = tokio::join!(
                autoflight::set_dial(xp, Dial::Altitude, hold_ft),
                autoflight::set_dial(xp, Dial::Vs, if alt_msl_ft < hold_ft { 1000.0 } else { 0.0 }),
                xp.set_float(&thr, GIVE_UP_THROTTLE),
            );
            event(
                "arrival_handover",
                json!({ "runway": built.rwy_id, "msa_ft": msa, "hold_ft": hold_ft }),
            );
            give_up(
                xp,
                tools.taxi.park_brake_valve_set,
                voice,
                &mountain_handover_line(&route.dest_name, msa, hold_ft),
                "arrival_unflyable",
            )
            .await;
            return;
        }
        _ => {
            event(
                "arrival_autoland",
                json!({ "runway": built.rwy_id, "wind": wind_detail }),
            );
            announce(
                voice,
                Priority::Advisory,
                if route.local_runway.is_some() {
                    "On the local final. Landing us visually."
                } else {
                    "No published approach lines up from here, so I'll land us visually."
                },
            );
            crate::autoland::run_autoland_scoped(
                config.xplane_base_url.clone(),
                crate::config::apt_dat_path(),
                tools.circuit.cfg.clone(),
                tools.circuit.approach_ias,
                voice.clone(),
                None,
                landing_rwy.clone(),
                xp.cancellation(),
                crate::autoland::HandDown::default(),
            );
        }
    };

    // --- The final report is what earns the landing clearance: call it the
    // moment autoland takes the approach (the FAF handoff, or straight away on
    // the visual). Nothing left to actuate here — the arrival loops own the
    // aircraft — so this watch is the orchestrator's last duty.
    waiting("the arrival to hand to autoland");
    let started = Instant::now();
    let mut watch = ArrivalWatch::default();
    let called = loop {
        if ABORT.load(SeqCst) {
            break false;
        }
        // The deferred inbound call goes out the moment the destination's
        // station is the one on the frequency, or the moment a tower on it
        // asks who we are (the challenge is drained either way).
        let challenged = crate::party_line::take_zone_challenge();
        if let Some(khz) = inbound_owed {
            let swapped = crate::party_line::home_tower_khz() == Some(khz);
            if swapped || challenged {
                inbound_owed = None;
                call_inbound(xp, ctx, voice, callsign, khz, swapped, built.rwy_id.as_deref()).await;
            }
        }
        if crate::autoland::ACTIVE.load(Relaxed) || !crate::procedure::ACTIVE.load(Relaxed) {
            break true;
        }
        let elapsed = started.elapsed().as_secs_f64();
        let stalled = watch.stalled(
            crate::procedure::active_leg().map(|l| l.fix).as_deref(),
            crate::procedure::fix_distance_nm(),
            elapsed,
        );
        if stalled || elapsed >= ARRIVAL_CAP_SECS {
            event(
                "arrival_stalled",
                json!({ "elapsed_s": elapsed.round(), "stalled": stalled }),
            );
            break false;
        }
        tokio::time::sleep(Duration::from_millis(1000)).await;
    };
    // Out of patience with an approach still being flown: the follower dies
    // with this scope whatever we do here (its client refuses writes the
    // moment the lease drops), so stand it down while we can still speak and
    // still write, and hand back a FLYABLE aircraft — the rule autoland's own
    // give-ups follow. Silence here is what killed the EGPH arrival.
    if !called && !ABORT.load(SeqCst) && crate::procedure::ACTIVE.load(Relaxed) {
        crate::procedure::abort();
        // The follower's stand-down releases the speed governor; the throttle
        // write has to come after it, or the governor re-commands the power we
        // just set.
        wait_until(5.0, 250, || async {
            !crate::procedure::ACTIVE.load(Relaxed)
        })
        .await;
        let thr = crate::aircraft::throttle_dataref();
        let _ = tokio::join!(
            xp.set_float(&thr, GIVE_UP_THROTTLE),
            autoflight::set_dial(xp, Dial::Vs, 0.0),
        );
        give_up(
            xp,
            tools.taxi.park_brake_valve_set,
            voice,
            "The arrival isn't getting us there. Your aircraft. The autopilot is holding \
             heading and altitude, climb power set.",
            "arrival_stalled",
        )
        .await;
        return;
    }
    // A procedure→autoland handoff has a brief both-inactive window while the
    // autoland thread spins up; give it a beat before concluding nobody flies.
    if called && !crate::autoland::ACTIVE.load(Relaxed) {
        tokio::time::sleep(Duration::from_secs(3)).await;
    }
    if crate::autoland::ACTIVE.load(Relaxed) && !ABORT.load(SeqCst) {
        // Never swapped (a field with no region of its own): the inbound
        // still precedes the final, on whatever tower is listening.
        if let Some(khz) = inbound_owed.take() {
            call_inbound(xp, ctx, voice, callsign, khz, false, built.rwy_id.as_deref()).await;
        }
        // "Final" is reported established on it, never when autoland takes
        // the arrival: a reposition round the field is not a final, and the
        // tower clears what it is told (`autoland::final_report_due`).
        let waited = Instant::now();
        while !ABORT.load(SeqCst)
            && crate::autoland::ACTIVE.load(Relaxed)
            && !crate::autoland::established_on_final()
        {
            tokio::time::sleep(Duration::from_millis(500)).await;
        }
        let waited_s = waited.elapsed().as_secs_f64();
        if waited_s >= 1.0 {
            event(
                "final_call_waited",
                json!({ "waited_s": waited_s.round(),
                        "established": crate::autoland::established_on_final() }),
            );
        }
    }
    if crate::autoland::established_on_final() && !ABORT.load(SeqCst) {
        let call = match &landing_rwy {
            Some(rwy) => format!("final runway {}", crate::crew_radio::spell_runway(&rwy.id)),
            None => "final".to_string(),
        };
        radio_out(crate::crew_radio::radio_call(callsign, &call));
        event(
            "final_call",
            json!({ "runway": landing_rwy.as_ref().map(|r| &r.id),
                    "icao": landing_rwy.as_ref().and_then(|r| r.icao.as_ref()) }),
        );
        // The tower's answer (the clearance, and any correction of the
        // runway) is autoland's to read back and obey: it watches the party
        // line every tick and re-arms itself on the cleared end.
    }

    // Gate to gate: once autoland is down and stopped (it goes inactive after
    // "down and stopped, your aircraft"), taxi to PARKING on our own — request
    // the taxi-in and drive to the stand, the arrival mirror of the departure
    // taxi we ran at the start. Bounded so a balk/abort can't hang here, and
    // skipped unless we actually got the aircraft down on the ground (autoland
    // also goes inactive on a go-around, climbing away).
    if crate::autoland::ACTIVE.load(Relaxed) {
        taxi_in_after_landing(xp, config, voice, tools, false).await;
    }
    event("arrival_owns", json!({}));
    stop_active();
}
