//! The loop: the launches, the overspeed watch, the backtrack, the alignment
//! on the runway and `drive`, whole.

use super::*;

/// True when the plugin's steering channel is live — a zero write is a
/// no-op steer, so probing costs nothing.
pub async fn steer_channel_ok(xp: &dyn crate::xplane::XPlane) -> bool {
    xp.set_float(TAXI_STEER, 0.0).await.is_ok()
}

/// Spawn the steering loop. The caller (the `auto_taxi` tool) has already
/// validated the route, armed the taxi governor, and proved the plugin's
/// steer channel writable.
pub fn start(base_url: String, voice: Option<Voice>, route: Route, base_kts: f64) {
    let Some(gen) = RUN.claim() else {
        return; // already driving — the tool retargets instead
    };
    DRIVES_STARTED.fetch_add(1, Ordering::SeqCst);
    LAST_ARRIVED.store(false, Ordering::Relaxed);
    RELOCATED.store(false, Ordering::Relaxed);
    if let Ok(mut w) = PROGRESS.lock() {
        *w = ProgressWatch::start(clock_s(), route_deadline_s(route.total, base_kts));
    }
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            return;
        };
        rt.block_on(drive(XPlaneClient::new(base_url), voice, route, base_kts, &gen));
    });
}

/// Past this groundspeed a drive is no taxi at all, whatever the target.
pub const OVERSPEED_ABS_KTS: f64 = 25.0;
/// This far over the target, held [`OVERSPEED_HOLD_S`], stops the drive.
pub const OVERSPEED_OVER_KTS: f64 = 8.0;
pub const OVERSPEED_HOLD_S: f64 = 3.0;

/// Whether the taxi is running away: `Some("absolute")` past
/// [`OVERSPEED_ABS_KTS`] (stop now), `Some("over_target")` past the target
/// by [`OVERSPEED_OVER_KTS`] (stop once held). Pure.
pub fn taxi_overspeed(gs_kts: f64, target_kts: f64) -> Option<&'static str> {
    if gs_kts > OVERSPEED_ABS_KTS {
        Some("absolute")
    } else if target_kts >= 1.0 && gs_kts > target_kts + OVERSPEED_OVER_KTS {
        Some("over_target")
    } else {
        None
    }
}

/// How many drives have started ([`drive_gen`]); ownership of the flag is
/// the run's ([`RUN`]).
pub(super) static DRIVES_STARTED: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Drive a backtrack: leg one (down the strip to the turn pocket) now, and
/// when it ARRIVES, leg two — a plain line-up from wherever the aircraft
/// actually stopped, which the join machinery turns into the 180. Any other
/// ending of leg one (pedals, "my controls", park handle, off-route) stands
/// the whole maneuver down: those paths already spoke, and the pilot who
/// took over mid-backtrack does not want a second drive starting itself.
pub fn start_backtrack(
    base_url: String,
    voice: Option<Voice>,
    route: Route,
    rwy: crate::runway::Runway,
    base_kts: f64,
    need_m: f64,
) {
    // Leg two is built here, far from the tool that knows the airframe — carry
    // the pace across rather than letting the fresh route fall back to GA.
    let pace = route.pace;
    let turn = TurnRound {
        pivots: pace.brake_steer || crate::aircraft::nose_castors(),
        width_m: crate::runway::runway_width_m(&rwy),
    };
    BACKTRACKING.store(true, Ordering::Relaxed);
    start(base_url.clone(), voice.clone(), route, base_kts);
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            return;
        };
        rt.block_on(async move {
            // Wait out leg one — bounded, so a stuck drive can't leak this
            // thread forever.
            for _ in 0..2400 {
                if !ACTIVE.load(Ordering::Relaxed) {
                    break;
                }
                tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            }
            if ACTIVE.load(Ordering::Relaxed) || !LAST_ARRIVED.load(Ordering::Relaxed) {
                BACKTRACKING.store(false, Ordering::Relaxed);
                return; // taken over, arrested, or still (impossibly) driving
            }
            if !crate::tools::speedhold::taxi_hold_active() {
                BACKTRACKING.store(false, Ordering::Relaxed);
                return; // speed authority gone with leg one — nobody drives
            }
            let xp = XPlaneClient::new(base_url.clone());
            let (Ok(lat), Ok(lon)) = (xp.get_float(LAT).await, xp.get_float(LON).await) else {
                BACKTRACKING.store(false, Ordering::Relaxed);
                return;
            };
            // Leg two of a backtrack: the aircraft has turned around and is
            // already on the strip pointing down it; settle and straighten
            // in the least runway that does it.
            let Some(path) = backtrack_leg_two_path(&rwy, lat, lon, turn) else {
                BACKTRACKING.store(false, Ordering::Relaxed);
                alert(
                    &voice,
                    Priority::Caution,
                    "I couldn't set up the turn. The line-up is yours.",
                );
                return;
            };
            let watch = crate::party_line::TaxiWatch {
                path,
                via: String::new(),
                seg: Vec::new(),
                holding_point: String::new(),
                lead_on: Vec::new(),
            };
            let Some(mut route2) = Route::build(&watch) else {
                BACKTRACKING.store(false, Ordering::Relaxed);
                alert(
                    &voice,
                    Priority::Caution,
                    "I couldn't set up the turn. The line-up is yours.",
                );
                return;
            };
            let spoken = crate::atis::spoken_runway(&rwy.id);
            route2.pace = pace;
            route2.align_course_deg = Some(rwy.course_deg);
            route2.align_runway = Some((rwy.clone(), need_m));
            route2.turn_hint = turn_round_hint(&rwy, lat, lon);
            route2.done_line = Some(format!("Lined up runway {spoken}. Holding position."));
            crate::blackbox::record(
                "autotaxi",
                serde_json::json!({"event": "backtrack_turn", "rwy": rwy.id,
                                   "pivots": turn.pivots, "width_m": turn.width_m,
                                   "turn_hint": route2.turn_hint,
                                   "from_along_m": -crate::runway::along_track_nm(lat, lon, &rwy) * M_PER_NM,
                                   "from_ct_m": crate::runway::cross_track_nm(lat, lon, &rwy) * M_PER_NM}),
            );
            crate::tools::speedhold::set_taxi_target(base_kts);
            start(base_url, voice, route2, base_kts);
            // Leg two owns ACTIVE from here.
            BACKTRACKING.store(false, Ordering::Relaxed);
        });
    });
}

/// How a line-up's alignment ended ([`align_on_the_runway`]).
pub(super) enum AlignEnd {
    Aligned,
    /// Stopped this many degrees off the course (signed).
    NotAligned(f64),
    /// Stood down under it (pedals, "my controls", the hold gone, a newer
    /// drive): no arrival.
    Interrupted,
}

/// What a line-up that could not straighten says instead of "lined up".
pub fn not_aligned_line(err_deg: f64) -> String {
    format!(
        "On the runway, but {:.0} degrees off its heading and I couldn't straighten us. \
         Holding position.",
        err_deg.abs()
    )
}

/// Bring a castoring nose round onto the runway's course after a line-up's
/// arrival ([`LineUpAlign`]): the differential brakes on the drive's own
/// channels, the governor creeping or stopping. Every tick is blackboxed
/// `line_up_align`, the verdict `line_up_aligned`.
pub(super) async fn align_on_the_runway(
    xp: &XPlaneClient,
    voice: &Option<Voice>,
    course: f64,
    fit: Option<&(crate::runway::Runway, f64)>,
    gen: &sim::run::Lease<'_>,
) -> AlignEnd {
    // The runway it may roll is judged from where the drive stopped.
    let mut judge: Option<LineUpAlign> = None;
    let t0 = std::time::Instant::now();
    let mut frozen_s = 0.0_f64;
    let mut start: Option<(f64, f64)> = None;
    let mut last_creep: Option<bool> = None;
    let mut last_err = 0.0_f64;
    let mut last_lat: Option<f64> = None;
    let end = loop {
        // The tick runs to its period, not sleep + work (`DRIVE_TICK_MS`).
        let pacer = sim::state::Pacer::start();
        if !gen.live()
            || !speedhold::taxi_hold_active()
        {
            break AlignEnd::Interrupted;
        }
        // The freeze flags, the pedals, the position and the yaw rate in
        // ONE read (`sim::state`): a round of requests is a sim frame, and
        // the freeze check alone was one or two more (iMac 2026-09-30).
        let st = sim::state::read(xp, sim::state::Want(ALIGN_WANT)).await;
        if st.frozen() {
            progress_hold();
            frozen_s += 0.5;
            tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            continue;
        }
        let (yield_r, pos, yaw_r) = (
            st.get(sim::state::F::SteerYield),
            st.position(),
            st.get(sim::state::F::R),
        );
        if yield_r.is_ok_and(|v| v > 0.5) {
            crate::blackbox::record("autotaxi", serde_json::json!({"event": "pedal_yield"}));
            crate::flight::note_pilot_takeover("taxi: pedals");
            alert(
                voice,
                Priority::Advisory,
                "Your steering. I still have the throttle and brakes.",
            );
            gen.lower_flag();
            break AlignEnd::Interrupted;
        }
        let (lat_r, lon_r) = sim::position::lat_lon(&pos);
        let psi_r = sim::position::field(&pos, |p| p.psi_true_deg);
        let gs_r = sim::position::field(&pos, |p| p.gs_ms);
        // A failed read is no measurement: nothing written, the channels
        // expire, the next tick asks again.
        let (Ok(lat), Ok(lon), Ok(psi), Ok(gs_ms), Ok(yaw)) = (lat_r, lon_r, psi_r, gs_r, yaw_r)
        else {
            tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            continue;
        };
        progress_hold();
        let gs_kts = gs_ms * MS_TO_KTS;
        let from = *start.get_or_insert((lat, lon));
        let judge = judge.get_or_insert_with(|| {
            let spare = fit.map(|(rwy, need)| align_spare_m(rwy, *need, lat, lon));
            let j = LineUpAlign::with_spare(course, spare);
            let j = if crate::aircraft::nose_free() {
                j.with_ground(crate::ground_steer::GroundPlant::for_airframe())
            } else {
                j
            };
            crate::blackbox::record(
                "autotaxi",
                serde_json::json!({"event": "line_up_align_budget", "spare_m": spare,
                                   "roll_limit_m": j.roll_limit_m, "creep_kts": j.creep_kts}),
            );
            j
        });
        let rolled_m = crate::autoland::haversine_nm(from.0, from.1, lat, lon) * M_PER_NM;
        let t_s = t0.elapsed().as_secs_f64() - frozen_s;
        last_err = judge.err_deg(psi);
        // The lateral off the runway's centreline, when the runway is known.
        let lat_m = fit.map(|(rwy, _)| crate::runway::cross_track_nm(lat, lon, rwy) * M_PER_NM);
        last_lat = lat_m;
        let step = judge.step_on_line(t_s, psi, gs_kts, yaw, rolled_m, lat_m);
        let (bl, br, creep) = match step {
            AlignStep::Align { bl, br, creep } => (bl, br, creep),
            _ => (0.0, 0.0, false),
        };
        crate::blackbox::record(
            "autotaxi",
            serde_json::json!({"event": "line_up_align", "err": last_err, "yaw_dps": yaw,
                               "gs": gs_kts, "bl": bl, "br": br, "creep": creep,
                               "trim": judge.trim, "rolled_m": rolled_m, "t_s": t_s,
                               "lat_m": lat_m, "closing": judge.closing}),
        );
        match step {
            AlignStep::Done { .. } => break AlignEnd::Aligned,
            AlignStep::GiveUp { err_deg, .. } => break AlignEnd::NotAligned(err_deg),
            AlignStep::Align { .. } => {}
        }
        if last_creep != Some(creep) {
            speedhold::set_taxi_target(if creep { judge.creep_kts } else { 0.0 });
            last_creep = Some(creep);
        }
        // The rudder helps what it can (a steerable castoring nose, the
        // SF50's); the brakes do the turning.
        // On a free nose the rudder is the one ground law's.
        let steer = if judge.free() {
            judge.rudder
        } else {
            ((last_err - yaw * ALIGN_LEAD_S) / ALIGN_FULL_DEG).clamp(-1.0, 1.0)
        };
        let _ = tokio::join!(
            xp.set_float(TAXI_STEER, steer),
            xp.set_float(LEFT_BRAKE, bl),
            xp.set_float(RIGHT_BRAKE, br),
        );
        pacer.wait(std::time::Duration::from_millis(DRIVE_TICK_MS)).await;
    };
    speedhold::set_taxi_target(0.0);
    let _ = tokio::join!(
        xp.set_float(TAXI_STEER, 0.0),
        xp.set_float(LEFT_BRAKE, 0.0),
        xp.set_float(RIGHT_BRAKE, 0.0),
    );
    let ok = match &end {
        AlignEnd::Aligned => Some(true),
        AlignEnd::NotAligned(_) => Some(false),
        AlignEnd::Interrupted => None,
    };
    let (creeps, rolled) = match (&judge, start) {
        (Some(j), Some(from)) => {
            let here = sim::position::read(xp).await;
            let (la, lo) = sim::position::lat_lon(&here);
            let rolled = la.ok().zip(lo.ok()).map(|(la, lo)| {
                crate::autoland::haversine_nm(from.0, from.1, la, lo) * M_PER_NM
            });
            (j.creeps, rolled)
        }
        _ => (0, None),
    };
    crate::blackbox::record(
        "autotaxi",
        serde_json::json!({"event": "line_up_aligned", "ok": ok, "err_deg": last_err,
                           "course": course, "creeps": creeps, "rolled_m": rolled,
                           "lat_m": last_lat,
                           "closed_lateral": judge.as_ref().is_some_and(|j| j.closed_lateral),
                           "secs": t0.elapsed().as_secs_f64() - frozen_s}),
    );
    end
}

/// Say a line without blocking the loop (queued; gated off in ATC-only mode
/// like every crew alert).
/// How long a drive may command motion without moving before it stands down.
pub(super) const STAGNANT_SECS: f64 = 30.0;

/// What a stuck drive says. Each clause is what the log can actually show:
/// "the nose is pointing away" used to be said for every stall with a big
/// heading error, including a 172 sat on the grass at breakaway power that
/// no heading could have moved (LIPK 2026-09-26).
pub(super) fn stagnant_line(off_pavement: bool, facing_away: bool) -> &'static str {
    match (off_pavement, facing_away) {
        (true, _) => {
            "I can't get us moving, we're off the pavement and on the grass. \
             Your aircraft. Get us back onto the taxiway, then ask me to taxi again."
        }
        (false, true) => {
            "I can't get us moving with the nosewheel hard over. Your aircraft. \
             Turn us toward the taxiway, then ask me to taxi again."
        }
        (false, false) => "I can't get us moving. Standing down. Your aircraft.",
    }
}

/// The steering loop. Steering writes ride the plugin's expiring channel, so
/// every exit path — including a panic or kill -9 — releases the override
/// within a second by silence alone; the explicit zero write on the way out
/// is just politeness.
pub(super) async fn drive(
    xp: XPlaneClient,
    voice: Option<Voice>,
    mut route: Route,
    base_kts: f64,
    gen: &sim::run::Lease<'_>,
) {
    let _detail = crate::auto_status::DetailGuard::hold(&DETAIL);
    // The landing light over runway spans, decided once for the drive: none
    // when the airframe's landing lamp is already its taxi light tonight.
    let crossing_light = if route.runway_spans.is_empty() {
        None
    } else {
        crate::tools::lights::crossing_light(crate::tools::lights::dark_now(&xp).await)
    };
    let mut crossing_lit = false;
    tracing::info!("auto taxi started");
    crate::blackbox::record(
        "autotaxi",
        serde_json::json!({"event": "engaged", "total_m": route.total, "base_kts": base_kts}),
    );
    // The pilot's pace: adopted from a spoken "taxi at N" mid-drive.
    let mut base = base_kts;
    // What the schedule last wrote into the governor — any OTHER value read
    // back is an external retarget (spoken stop/go/speed) to fold in.
    let mut last_sched = base_kts;
    // External stop ("stop" / both pedals park the target at 0): the
    // schedule stands aside until a "go" restores a rolling target.
    let mut ext_stopped = false;
    // A tug on the bar: held since, and whether the hold has been announced.
    let mut tug_since: Option<std::time::Instant> = None;
    let mut tug_held = false;
    // Seeded at the first fix: >0 means we engaged off the route or facing
    // against it and are JOINING — widened guard, bend-speed cap — until
    // captured, when it drops to 0 and the established guard takes over.
    let mut join_allow: Option<f64> = None;
    let mut join_capture = JoinCapture::default();
    let t_join = std::time::Instant::now();
    // Yaw rate for the steering lead, EMA-smoothed: raw tick-to-tick heading
    // differences of an HTTP read are noise the lead would amplify.
    let mut prev_psi: Option<(f64, std::time::Instant)> = None;
    let mut yaw_rate = 0.0f64;
    // Last steering command, so the slew limiter has something to move FROM.
    let mut prev_steer = 0.0f64;
    let mut straight_trim = StraightTrim::default();
    // A turn-round the geometry knows the way of is turned that way from
    // the first tick (`Route.turn_hint`).
    let mut turn_dir: Option<f64> = route.turn_hint;
    // Whether the brake-steer assist currently has a wheel braked, so every
    // exit path can release it exactly once — and the last commanded pair,
    // so the assist's own slew has something to move from.
    let mut bs_on = false;
    let mut brakes = BrakeSteer::default();
    // An airliner's pedals steer the nose a few degrees: where the airframe
    // declares a tiller the same command goes to it too (`crate::tiller`).
    let mut tiller = crate::tiller::Tiller::for_airframe();
    tiller.arm(&xp).await;
    // A nose with no steering of its own flies the one ground law on the
    // route (`ground_steer`), on the sim's own yaw rate.
    let free_nose = crate::aircraft::nose_free();
    let mut ground = crate::ground_steer::GroundSteer::for_airframe();
    let mut ref_path = route.ref_path();
    let mut ref_dir: Option<f64> = route.turn_hint;
    let max_yaw_dps = route.pace.max_yaw_dps.unwrap_or(crate::ground_steer::DEFAULT_MAX_YAW_DPS);
    // Stagnation watch: commanding motion but not moving (see below).
    let mut stalled_since: Option<std::time::Instant> = None;
    // Over the taxi pace since (see [`taxi_overspeed`]).
    let mut overspeed_since: Option<std::time::Instant> = None;
    // The last good fix, for the relocation guard ([`relocation_jump_m`]).
    let mut last_fix: Option<((f64, f64), std::time::Instant)> = None;
    // The last tick's lateral, for the join's jump test (`join_fix`).
    let mut last_lateral: Option<f64> = None;
    let mut last_jumped = false;
    // A join the nose cannot point at is judged once, at the first fix
    // ([`JoinTurn`]).
    let mut join_turn_judged = false;
    // The traffic guard (`traffic_guard`): our span from the mass the join
    // turn reads too, our nose from the `.acf`, the picture read once a
    // second (the sim's TCAS table is five Web API reads), the hold's
    // state. No picture, no guard.
    let own_size = OwnSize::from_mtow(
        xp.get_float(crate::datarefs::ACF_M_MAX_KG)
            .await
            .ok()
            .map(f64::from),
    )
    .with_nose(crate::aircraft::learned::acf_model().and_then(|m| m.geometry.nose_m));
    let mut traffic_track = TrafficTrack::default();
    let mut traffic_seen: Vec<Seen> = Vec::new();
    let mut traffic_read_at: Option<std::time::Instant> = None;
    let mut traffic_hold = TrafficHold::default();
    loop {
        // The tick runs to its period, not sleep + work (`DRIVE_TICK_MS`).
        let pacer = sim::state::Pacer::start();
        if !gen.live() {
            crate::blackbox::record(
                "autotaxi",
                serde_json::json!({"event": "stopped_from_outside", "superseded": !gen.current()}),
            );
            break;
        }
        // Speed authority gone — steering alone must not keep driving. This
        // is NOT only the deliberate paths (park handle, spoken release):
        // the governor's throttle-override check can't tell a pilot's hand
        // from a hardware-axis bump the sim re-drives, and mid-drive that
        // left an aircraft rolling at taxi power with NOBODY on the
        // throttle — the 2026-07-20 Gatwick runaway ran 10 to 50 knots
        // through the grass in silence. On the ground the safe reading is
        // stopping: throttle to idle plus ONE expiring brake pulse (the
        // pilot's own brake key, frees itself within a second, can't
        // stick), and say it out loud — a pilot who truly wants the power
        // pushes again with everything already released. Airborne (a
        // runaway that got that far), touch nothing.
        if !speedhold::taxi_hold_active() {
            let on_ground = xp.get_float(crate::datarefs::ON_GROUND_ANY).await.unwrap_or(0.0) > 0.5;
            tracing::info!(
                on_ground,
                "auto taxi: taxi hold gone, arresting and releasing"
            );
            if on_ground {
                let _ = xp.set_float(&throttle_dataref(), 0.0).await;
                if xp
                    .run_command_for(crate::tools::BRAKES_REGULAR, crate::tools::BRAKE_HOLD_SECS)
                    .await
                    .is_ok()
                {
                    crate::tools::note_brake_cmd();
                }
                alert(
                    &voice,
                    Priority::Caution,
                    "I've lost the speed hold. Throttle's idle. Your aircraft.",
                );
            } else {
                alert(
                    &voice,
                    Priority::Caution,
                    "I've lost the speed hold. Your aircraft.",
                );
            }
            crate::blackbox::record(
                "autotaxi",
                serde_json::json!({"event": "hold_gone", "arrested": on_ground}),
            );
            break;
        }
        // Paused or replay: stop writing — the steer channel expires and the
        // pedals are the pilot's until the sim is live again.
        //
        // One read, not a chain: sequential HTTP awaits stretched the
        // takeoff tick to half a second and bought a slew-limit weave
        // (sortie four); the same medicine here keeps the steer tick honest.
        // Position, heading and groundspeed as ONE frame saw them (#157),
        // and since 2026-09-30 the freeze flags, the pedals and the tug in
        // the same request (`sim::state`): every round of requests is a sim
        // frame (50 ms on the iMac at 20 fps), and the freeze check was one
        // or two rounds of its own ahead of the read.
        let st = sim::state::read(&xp, sim::state::Want(if free_nose { DRIVE_WANT_FREE } else { DRIVE_WANT })).await;
        if st.frozen() {
            stalled_since = None; // a pause is not a stall
            progress_hold();
            tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            continue;
        }
        let (yield_r, pos, tug_r) = (
            st.get(sim::state::F::SteerYield),
            st.position(),
            st.get(sim::state::F::Tug),
        );
        // A free nose reads the sim's yaw rate too (its law's measurement).
        let yaw_now = if free_nose { st.opt(sim::state::F::R) } else { None };
        let (lat_r, lon_r) = sim::position::lat_lon(&pos);
        let psi_r = sim::position::field(&pos, |p| p.psi_true_deg);
        let gs_r = sim::position::field(&pos, |p| p.gs_ms);
        let tug_on = tug_r.is_ok_and(|v| v > 0.5);
        if tug_on {
            let t0 = *tug_since.get_or_insert_with(std::time::Instant::now);
            if !tug_held {
                tug_held = true;
                tracing::info!("auto taxi: tug on the bar, holding until it lets go");
                crate::blackbox::record("autotaxi", serde_json::json!({"event": "tug_hold"}));
                speedhold::set_taxi_target(0.0);
                last_sched = 0.0;
                // Once per tug: the push may have just said it.
                if !crate::pushback::take_tug_warned() {
                    alert(
                        &voice,
                        Priority::Advisory,
                        "The tug's still on the nose. Holding until it lets go.",
                    );
                }
            }
            if t0.elapsed().as_secs_f64() > TUG_WAIT_SECS {
                tracing::warn!("auto taxi: the tug never let go, standing down");
                crate::blackbox::record(
                    "autotaxi",
                    serde_json::json!({"event": "tug_never_released"}),
                );
                alert(
                    &voice,
                    Priority::Caution,
                    "The tug hasn't let go. Standing down, brakes set. Your aircraft.",
                );
                gen.lower_flag();
                break;
            }
            // No steer heartbeat: the nosewheel is the tug's, and the
            // channel expires on its own. Nothing here is a stall.
            stalled_since = None;
            progress_hold();
            tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            continue;
        } else if tug_held {
            tug_held = false;
            tug_since = None;
            // Not said: the monitor's "Pushback done, tug's clear" is the
            // one voice for the bar coming off (both were heard, LIPK
            // 2026-09-26).
            crate::blackbox::record("autotaxi", serde_json::json!({"event": "tug_released"}));
        }
        // The pilot's feet moved the yaw axis: the plugin already dropped
        // the override — stand the loop down and keep the speed hold, which
        // is the pre-existing "you steer, I have the throttle" division.
        if yield_r.is_ok_and(|v| v > 0.5) {
            tracing::info!("auto taxi: pilot on the pedals, steering released");
            crate::blackbox::record("autotaxi", serde_json::json!({"event": "pedal_yield"}));
            crate::flight::note_pilot_takeover("taxi: pedals");
            alert(
                &voice,
                Priority::Advisory,
                "Your steering. I still have the throttle and brakes.",
            );
            gen.lower_flag();
            break;
        }
        let (Ok(lat), Ok(lon), Ok(psi), Ok(gs_ms)) = (lat_r, lon_r, psi_r, gs_r) else {
            // Sim hiccup: no writes, the channel expires, retry.
            tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
            continue;
        };
        let gs_kts = gs_ms * MS_TO_KTS;
        // Moved under us: whatever this route was, it is for a place the
        // aircraft no longer is. Stand down BEFORE the join or the off-route
        // guard reads the jump as a drive gone wrong (LEMG 31, 2026-09-28).
        let now = std::time::Instant::now();
        if let Some((from, t0)) = last_fix {
            if let Some(jump_m) =
                relocation_jump_m(from, (lat, lon), now.duration_since(t0).as_secs_f64())
            {
                tracing::warn!(jump_m, "auto taxi: the aircraft was moved, standing down");
                crate::blackbox::record(
                    "autotaxi",
                    serde_json::json!({"event": "relocated", "jump_m": jump_m,
                                       "dt_s": now.duration_since(t0).as_secs_f64()}),
                );
                speedhold::set_taxi_target(0.0);
                let _ = xp.set_float(TAXI_STEER, 0.0).await;
                RELOCATED.store(true, Ordering::Relaxed);
                alert(
                    &voice,
                    Priority::Caution,
                    "We've been moved. Taxi stopped. Your aircraft.",
                );
                gen.lower_flag();
                break;
            }
        }
        let moved_m = last_fix.map_or(0.0, |(from, _)| {
            crate::autoland::haversine_nm(from.0, from.1, lat, lon) * M_PER_NM
        });
        last_fix = Some(((lat, lon), now));
        // Signed here, magnitude everywhere below: the gates ask how far off
        // the route we are, and the record wants to know which side. While
        // still JOINING, project along the nose: the nearest paint can be a
        // leg already behind us (the pushback that let go 60 m short of its
        // node, Malaga 2026-09-11 — the join drove a loop back to the node).
        // ...never onto a leg the aircraft did not move to (`join_fix`).
        let joining = join_allow.is_none_or(|a| a > 0.0);
        let fixed = if joining {
            let (fix, jumped) = join_fix(
                route.fix_signed_facing(lat, lon, psi),
                route.fix_signed(lat, lon),
                last_lateral,
                moved_m,
            );
            if jumped && !last_jumped {
                crate::blackbox::record(
                    "autotaxi",
                    serde_json::json!({"event": "projection_jump", "prev_lat_m": last_lateral,
                                       "moved_m": moved_m,
                                       "facing": route.fix_signed_facing(lat, lon, psi)}),
                );
            }
            last_jumped = jumped;
            fix
        } else {
            route.fix_signed(lat, lon)
        };
        let Some((progress, offset)) = fixed else {
            tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
            continue;
        };
        let lateral = offset.abs();
        last_lateral = Some(lateral);
        let remaining = route.remaining(progress);
        progress_note(progress);
        if ext_stopped {
            // Held by a spoken "stop" (or both pedals): a stop somebody
            // asked for is not a stall.
            progress_hold();
        }
        // Traffic on the route ahead: stop the taxi gap behind it and wait,
        // never drive into it, never steer round it. The drive drove into
        // an AI airliner standing at a holding point, the two aircraft
        // drawn inside each other (pilot's screenshot, 2026-10-10): it
        // never read the traffic picture.
        if traffic_read_at.is_none_or(|t| t.elapsed().as_secs_f64() >= TRAFFIC_READ_S) {
            traffic_read_at = Some(std::time::Instant::now());
            let picture = crate::world::targets(xp.view()).await;
            traffic_seen = traffic_track.update(&picture, clock_s(), |cs| {
                target_tail_m(crate::world::traffic_type(cs).as_deref())
            });
        }
        let traffic_ahead = route.traffic_ahead(progress, gs_ms, &own_size, &traffic_seen);
        let (traffic_cap, traffic_ev) = traffic_hold.step(traffic_ahead.as_ref(), clock_s());
        if let Some(ev) = &traffic_ev {
            let rec = match ev {
                HoldEvent::Began {
                    callsign,
                    gap_m,
                    along_m,
                    crossing,
                } => serde_json::json!({
                    "event": "traffic_hold", "callsign": callsign, "gap_m": gap_m,
                    "along_m": along_m, "crossing": crossing, "progress_m": progress,
                    "remaining_m": remaining, "gs": gs_ms * MS_TO_KTS, "nose_m": own_size.nose_m}),
                HoldEvent::Long { callsign, held_s } => serde_json::json!({
                    "event": "traffic_hold_long", "callsign": callsign, "held_s": held_s}),
                HoldEvent::Cleared { callsign, held_s } => serde_json::json!({
                    "event": "traffic_clear", "callsign": callsign, "held_s": held_s,
                    "progress_m": progress}),
            };
            tracing::info!(?ev, "auto taxi: traffic");
            crate::blackbox::record("autotaxi", rec);
            if let Some(line) = hold_line(ev) {
                alert(&voice, Priority::Advisory, line);
            }
        }
        // Slowed or stopped by traffic is no stall: the progress clock and
        // the stagnation watch both stand aside, however long it takes.
        let traffic_slowed = traffic_cap.is_some_and(|c| c < TRAFFIC_EXCUSES_BELOW_KTS);
        if traffic_slowed {
            progress_hold();
        }
        DETAIL.set(format!(
            "{} m to go",
            (remaining.max(0.0) / 10.0).round() as i64 * 10
        ));
        if let Some(l) = crossing_light.as_ref() {
            let want = on_runway_span(&route.runway_spans, progress);
            if want != crossing_lit {
                crossing_lit = want;
                let ok = crate::tools::lights::apply(&xp, l, want).await.is_ok();
                crate::blackbox::record(
                    "autotaxi",
                    serde_json::json!({"event": "crossing_lights", "on": want, "ok": ok, "progress_m": progress}),
                );
            }
        }
        // Steering error first: the join state and the guard both need it.
        // The aim: the airframe's, capped in a bend and near the stop
        // (see [`steer_look_m`]).
        let look = steer_look_m(
            &route.pace,
            gs_kts,
            route.route_turn_rate_dps(progress, BEND_PROBE_M, gs_kts),
            remaining,
            free_nose,
        );
        // Inside the stop zone, fly the route's FINAL BEARING instead of
        // pursuing an aim point that has stopped moving. See
        // `final_bearing_deg`: past the end the pursuit error runs to 160°+
        // and steers the aircraft into a turn toward a waypoint behind it.
        // Holding the last segment's direction is what a pilot does — nose
        // straight down the paint, roll to a halt.
        let err = if remaining <= STOP_HEADING_HOLD_M {
            route.stop_heading_err_deg(lat, lon, psi, progress, look)
        } else {
            route.heading_err_deg(lat, lon, psi, progress, look)
        };
        // The first fix: a join the nose cannot point at is flown as a
        // planned turn onto the route ahead, or refused and said (#190).
        if !join_turn_judged {
            join_turn_judged = true;
            let pivots = route.pace.brake_steer || crate::aircraft::nose_castors();
            let own_geometry = route.align_course_deg.is_some() || route.turn_hint.is_some();
            let quick = err.abs() > JOIN_TURN_MIN_DEG && !pivots && !own_geometry;
            let mtow_kg = if quick {
                xp.get_float(crate::datarefs::ACF_M_MAX_KG).await.ok().map(f64::from)
            } else {
                None
            };
            let on_rwy = crate::party_line::on_runway(lat, lon);
            if joining && join_turn_applies(err, pivots, mtow_kg, on_rwy, own_geometry) {
                let radius_m = join_turn_radius_m(mtow_kg);
                let turn = join_turn(&route.path, lat, lon, psi, radius_m, |a, b| {
                    crate::party_line::on_pavement(a, b)
                });
                match turn.and_then(|t| route.with_lead(&t.lead, t.join_m).map(|r| (t, r))) {
                    Some((t, rebuilt)) => {
                        tracing::info!(err, turn = t.turn_deg, "auto taxi: planned turn onto the route");
                        crate::blackbox::record(
                            "autotaxi",
                            serde_json::json!({"event": "join_turn", "err_deg": err, "lateral_m": offset,
                                               "dir": t.dir, "turn_deg": t.turn_deg, "radius_m": radius_m,
                                               "join_m": t.join_m, "lead_m": t.lead_m,
                                               "intercept_deg": t.intercept_deg,
                                               "total_m": rebuilt.total, "mtow_kg": mtow_kg}),
                        );
                        route = rebuilt;
                        ref_path = route.ref_path();
                        turn_dir = Some(t.dir);
                        ref_dir = Some(t.dir);
                        last_lateral = None;
                        progress_rebase();
                        continue;
                    }
                    None => {
                        tracing::warn!(err, "auto taxi: no turn onto the route fits, refusing");
                        crate::blackbox::record(
                            "autotaxi",
                            serde_json::json!({"event": "join_refused", "err_deg": err, "lateral_m": offset,
                                               "radius_m": radius_m, "mtow_kg": mtow_kg}),
                        );
                        speedhold::set_taxi_target(0.0);
                        let _ = xp.set_float(TAXI_STEER, 0.0).await;
                        alert(&voice, Priority::Caution, JOIN_REFUSED_LINE);
                        gen.lower_flag();
                        break;
                    }
                }
            }
        }
        let err = committed_err(err, &mut turn_dir);
        // The turn the route asks for over that same aim span, for the rate
        // lead to credit. Zero inside the stop zone: there the error is held
        // against a FIXED final bearing, which is a straight line by
        // construction, so any yaw there really is a swing to be checked.
        let route_turn_dps = if remaining <= STOP_HEADING_HOLD_M {
            0.0
        } else {
            route.route_turn_rate_dps(progress, look, gs_kts)
        };
        let allow = *join_allow.get_or_insert_with(|| {
            let a = join_allowance_m(lateral, err);
            if a > 0.0 {
                tracing::info!(lateral, err, "auto taxi: joining the route");
                crate::blackbox::record(
                    "autotaxi",
                    serde_json::json!({"event": "join", "lateral_m": lateral, "err_deg": err}),
                );
            }
            a
        });
        if allow > 0.0 && join_capture.step(lateral, err, t_join.elapsed().as_secs_f64()) {
            tracing::info!(lateral, "auto taxi: joined the route");
            crate::blackbox::record("autotaxi", serde_json::json!({"event": "joined"}));
            join_allow = Some(0.0);
        }
        // Too far off the pavement we were cleared on: stop, hand steering
        // back, and say so. Established that's the tight guard; a join gets
        // its engage distance plus turning room, never more — the one drive
        // this loop makes is TOWARD the route.
        if lateral > OFF_ROUTE_STOP_M + join_allow.unwrap_or(0.0) {
            tracing::warn!(lateral, "auto taxi: off route, stopping");
            crate::blackbox::record(
                "autotaxi",
                serde_json::json!({"event": "off_route", "lateral_m": lateral}),
            );
            speedhold::set_taxi_target(0.0);
            let _ = xp.set_float(TAXI_STEER, 0.0).await;
            alert(
                &voice,
                Priority::Caution,
                "We're off the cleared route. Stopping here. Your steering.",
            );
            gen.lower_flag();
            break;
        }
        // Stopped at the end of the route but NOT on it: that is no arrival.
        // A projection runs along the route however far off it the aircraft
        // is, so the end can be "reached" from the grass beside it: LSZS 03
        // (SR22, 2026-09-24) the second line-up joined 27 m off, never
        // captured, and declared the holding point 27.4 m off the line; full
        // auto then found the runway 34 degrees off. Hold it where it is and
        // hand the steering back, with the number.
        if remaining <= 1.0 && gs_kts < 1.5 && !arrived_on_line(lateral) {
            tracing::warn!(lateral, "auto taxi: route end reached off the line");
            crate::blackbox::record(
                "autotaxi",
                serde_json::json!({"event": "stopped_off_line", "lateral_m": lateral,
                                   "joining": join_allow.is_none_or(|a| a > 0.0)}),
            );
            speedhold::set_taxi_target(0.0);
            let _ = xp.set_float(TAXI_STEER, 0.0).await;
            alert(&voice, Priority::Caution, &off_line_stop_line(lateral));
            gen.lower_flag();
            break;
        }
        // Arrived: stopped at the hold-short line, ON it. The governor stays,
        // holding position on the brakes, exactly as a spoken "stop" —
        // "go" past this point is the pilot's call (a crossing clearance).
        if remaining <= 1.0 && gs_kts < 1.5 {
            // A line-up on a castoring nose ends POINTING down the runway,
            // not merely on it ([`LineUpAlign`]).
            let mut crooked: Option<f64> = None;
            if let Some(course) = route.align_course_deg.filter(|_| {
                route.pace.brake_steer || crate::aircraft::nose_castors()
            }) {
                match align_on_the_runway(&xp, &voice, course, route.align_runway.as_ref(), gen)
                    .await
                {
                    AlignEnd::Aligned => {}
                    AlignEnd::NotAligned(err) => crooked = Some(err),
                    AlignEnd::Interrupted => break,
                }
            }
            tracing::info!("auto taxi: holding point, done");
            LAST_ARRIVED.store(true, Ordering::Relaxed);
            crate::blackbox::record("autotaxi", serde_json::json!({"event": "holding_point"}));
            speedhold::set_taxi_target(0.0);
            let _ = xp.set_float(TAXI_STEER, 0.0).await;
            let line = if let Some(err) = crooked {
                not_aligned_line(err)
            } else if let Some(done) = &route.done_line {
                done.clone()
            } else if route.holding_point.is_empty() {
                "Holding point. Holding position, brakes set.".to_string()
            } else {
                format!(
                    "Holding point {}. Holding position, brakes set.",
                    route.holding_point
                )
            };
            alert(&voice, Priority::Advisory, &line);
            gen.lower_flag();
            break;
        }
        // A drive that commands motion but never MOVES is stuck. A backwards
        // join's near-full nosewheel lock is a chock the governor's gentle
        // taxi throttle can't break away against: a 169°-off engage at a
        // Jerez stand crept at millimetres per second for half a minute, no
        // voice line, until the pilot's pedals rescued it (16:48Z,
        // 2026-07-24). Twenty seconds of commanded-but-stationary says so out
        // loud and hands back, instead of pirouetting in place.
        // The window covers the governor's ramp to its BREAKAWAY ceiling
        // (`speedhold::taxi_throttle_cap`), about 20 s from idle at its
        // usual step: judged at 20 s the 172 at LIPK was still at 0.35.
        if !ext_stopped && !traffic_slowed && gs_kts < 0.7 {
            let t0 = *stalled_since.get_or_insert_with(std::time::Instant::now);
            if t0.elapsed().as_secs_f64() > STAGNANT_SECS {
                let thr_dref = throttle_dataref();
                let (thr, la, lo) = tokio::join!(
                    xp.get_float(&thr_dref),
                    xp.get_float(LAT),
                    xp.get_float(LON),
                );
                let (thr, here) = (thr.ok(), la.ok().zip(lo.ok()));
                let off_pavement = here
                    .and_then(|(la, lo)| crate::party_line::on_pavement(la, lo))
                    == Some(false);
                tracing::warn!(err, ?thr, off_pavement, "auto taxi: stagnant, standing down");
                crate::blackbox::record(
                    "autotaxi",
                    serde_json::json!({"event": "stagnant", "err_deg": err, "remaining_m": remaining,
                                       "throttle": thr, "off_pavement": off_pavement}),
                );
                speedhold::set_taxi_target(0.0);
                let _ = xp.set_float(TAXI_STEER, 0.0).await;
                let line = stagnant_line(off_pavement, err.abs() > JOIN_ERR_SLOW_DEG);
                alert(&voice, Priority::Caution, line);
                gen.lower_flag();
                break;
            }
        } else {
            stalled_since = None;
        }
        // Too fast to be a taxi: stop, never press on into a corner. The
        // Evolution taxied at 17 to 23 kt against a 10 kt target, took a
        // corner at that pace, ground-looped off the pavement and hit a wall
        // (LIME 2026-09-25). Sustained over the target by
        // [`OVERSPEED_OVER_KTS`], or ever past [`OVERSPEED_ABS_KTS`].
        // Against the drive's PACE, never a bend's momentary target: slowing
        // from 14 kt toward a 6 kt bend is the governor working, and judged
        // against the 6 it stopped a good taxi at LIME (2026-09-25).
        let target_now = speedhold::taxi_target().unwrap_or(base_kts).max(base);
        let over = taxi_overspeed(gs_kts, target_now);
        let over_held = match over {
            Some("absolute") => true,
            Some(_) => overspeed_since
                .get_or_insert_with(std::time::Instant::now)
                .elapsed()
                .as_secs_f64()
                >= OVERSPEED_HOLD_S,
            None => {
                overspeed_since = None;
                false
            }
        };
        if over_held {
            tracing::warn!(gs_kts, target_now, "auto taxi: too fast, stopping");
            crate::blackbox::record(
                "autotaxi",
                serde_json::json!({"event": "overspeed_stop", "gs": gs_kts, "target": target_now, "why": over}),
            );
            speedhold::set_taxi_target(0.0);
            let _ = xp.set_float(TAXI_STEER, 0.0).await;
            alert(
                &voice,
                Priority::Warning,
                &format!("Too fast for the taxi, {gs_kts:.0} knots. Stopping. Your aircraft."),
            );
            gen.lower_flag();
            break;
        }
        // Steering: aim at the route ahead, command the error down — led by
        // the yaw rate so a laggy plant (castoring nosewheel steering by
        // rudder) can't snake — with the lock tapered by groundspeed so a
        // standstill can break away. From a backwards or off-route start
        // this is the whole join — the deflection tightens as the roll
        // builds. Approaching the stop (route end, or a spoken "stop"
        // bleeding speed off) the wheel straightens so the brakes never
        // bite onto a deflected nosewheel.
        let now = std::time::Instant::now();
        let dt = prev_psi.map_or(0.2, |(_, t0)| {
            now.duration_since(t0).as_secs_f64().clamp(0.05, 1.0)
        });
        let raw_rate = prev_psi.map_or(0.0, |(p, t0)| {
            let dt = now.duration_since(t0).as_secs_f64();
            if (0.05..=1.0).contains(&dt) {
                crate::angles::signed_diff_deg(psi, p) / dt
            } else {
                0.0
            }
        });
        prev_psi = Some((psi, now));
        // Low-passed on TIME (`YAW_RATE_TAU_S`), not half per tick.
        yaw_rate += (raw_rate.clamp(-90.0, 90.0) - yaw_rate) * (1.0 - (-dt / YAW_RATE_TAU_S).exp());
        // A free castoring nose: the one ground law on the route
        // (`ground_steer`), the rudder and the brakes allocated together.
        let mut ground_rec: Option<serde_json::Value> = None;
        let (steer, bl, br, trim) = if free_nose {
            let v = gs_ms;
            let (e, ff) = if remaining <= STOP_HEADING_HOLD_M {
                // The stop zone: the route's final bearing, as every nose.
                (err, 0.0)
            } else {
                crate::ground_steer::path_demand(&ref_path, progress, route.local_m(lat, lon), psi, v, &ground, max_yaw_dps)
            };
            let e = committed_err(e, &mut ref_dir);
            let s = ground.step(crate::ground_steer::Demand {
                dt,
                v_ms: v,
                yaw: yaw_now.map_or(crate::ground_steer::Yaw::Mean(raw_rate), crate::ground_steer::Yaw::Now),
                err_deg: e,
                ff_dps: ff,
                rudder: crate::ground_steer::Rudder::Own {
                    slew_per_s: route.pace.steer_slew_per_s.unwrap_or(0.8),
                },
                u_max: PIVOT_BRAKE_MAX,
                r_max_dps: ground.taxi_rate_limit(v, PIVOT_BRAKE_MAX, max_yaw_dps),
                io_delay_s: crate::ground_steer::IO_DELAY_S,
            });
            let (bl, br) = s.wheels();
            bs_on = bl > 0.0 || br > 0.0;
            prev_steer = s.rudder;
            let mut rec = s.json();
            rec["e_ref"] = serde_json::json!(e);
            rec["ff"] = serde_json::json!(ff);
            rec["r"] = serde_json::json!(yaw_now);
            ground_rec = Some(rec);
            (s.rudder, bl, br, 0.0)
        } else {
            let trim = straight_trim.step(
                offset,
                route_turn_dps,
                gs_kts,
                join_allow.unwrap_or(0.0) > 0.0,
                remaining,
                dt,
            );
            let want = stop_straighten(
                (steer_cmd_with(
                    damped_err_deg(err, yaw_rate, route_turn_dps, route.pace.steer_lead_s),
                    route.pace.steer_full_deg,
                ) + trim)
                    .clamp(-1.0, 1.0)
                    * steer_authority(gs_kts),
                remaining,
                gs_kts,
            );
            let mut steer = slew_steer(prev_steer, want, route.pace.steer_slew_per_s, dt);
            prev_steer = steer;
            if ext_stopped {
                steer *= (gs_kts / 3.0).clamp(0.0, 1.0);
            }
            // The brake half of the steering, where the airframe needs it. Only
            // ever one side, slewed on like a foot (BRAKE_STEER_SLEW_PER_S),
            // released the moment the command shrinks back into the deadband.
            //
            // Written EVERY tick, never edge-gated on change. The plugin channel
            // is a heartbeat that expires a second after writes stop, so skipping
            // a steady command doesn't leave the last one standing, it drops the
            // brake mid-turn, and a turn is exactly where the command sits still
            // for seconds. (The old gate existed to keep HTTP off the tick, which
            // the join! below already buys: all three control writes go out
            // together, so the whole steering output costs ONE round trip, not
            // three. The 2026-07-28 lesson was about sequential awaits dragging
            // the loop to a second, not about the number of writes.)
            let (bl, br) = if route.pace.brake_steer || crate::aircraft::nose_castors() {
                let (bl, br) = brakes.step(
                    steer,
                    err,
                    gs_kts,
                    yaw_rate,
                    join_allow.unwrap_or(0.0) > 0.0,
                    dt,
                );
                bs_on = bl > 0.0 || br > 0.0;
                (bl, br)
            } else {
                (0.0, 0.0)
            };
            (steer, bl, br, trim)
        };
        let mut drive_rec = serde_json::json!({
            // `lat_m` is SIGNED: right of the route is positive. It was
            // a distance until 2026-08-05, which made every taxi trail in
            // the log unreadable against the paint: "two metres off" says
            // nothing about which side, and the side is the whole
            // question when the pilot can see the line.
            // `turn_dps` is the bend's OWN rate, the credit the lead
            // gives back: `yaw_dps` minus it is what the law actually
            // damps, and the pair is how a wide corner gets told apart
            // from a swing in the trace.
            "event": "drive", "gs": gs_kts, "err": err, "yaw_dps": yaw_rate,
            "turn_dps": route_turn_dps, "look_m": look,
            "steer": steer, "lat_m": offset, "remaining_m": remaining, "trim": trim,
            "bs": bs_on, "bl": bl, "br": br, "tug": tug_on,
            // "plugin": one frame's position; "split": separate reads (#157).
            "pos": sim::position::source_word(&pos),
        });
        // The free nose's law, tick by tick: the yaw rate asked and read,
        // the standing yaw learnt, the heading error flown and the route's
        // own rate fed forward.
        if let Some(g) = ground_rec {
            drive_rec["gsteer"] = g;
        }
        // The traffic the guard is keeping behind, while there is any.
        if let Some(a) = &traffic_ahead {
            drive_rec["traffic"] = serde_json::json!({
                "callsign": a.callsign, "along_m": a.along_m, "gap_m": a.gap_m,
                "stop_in_m": a.stop_in_m, "crossing": a.crossing, "cap": traffic_cap,
                "held": traffic_hold.holding()});
        }
        // The tiller as written, where the airframe has one.
        if let Some(c) = tiller.command(steer, gs_kts) {
            drive_rec["tiller"] = serde_json::json!(c);
        }
        crate::blackbox::record("autotaxi", drive_rec);
        let steer_write = if route.pace.brake_steer || crate::aircraft::nose_castors() {
            let (s, a, b, ()) = tokio::join!(
                xp.set_float(TAXI_STEER, steer),
                xp.set_float(LEFT_BRAKE, bl),
                xp.set_float(RIGHT_BRAKE, br),
                tiller.write(&xp, steer, gs_kts),
            );
            let _ = (a, b);
            s
        } else if tiller.declared() {
            let (s, ()) = tokio::join!(
                xp.set_float(TAXI_STEER, steer),
                tiller.write(&xp, steer, gs_kts),
            );
            s
        } else {
            xp.set_float(TAXI_STEER, steer).await
        };
        if steer_write.is_err() {
            // The channel vanished (plugin reload): no steering, no drive.
            tracing::warn!("auto taxi: steer channel lost, stopping");
            speedhold::set_taxi_target(0.0);
            alert(
                &voice,
                Priority::Caution,
                "I've lost the steering. Stopping. Your aircraft.",
            );
            gen.lower_flag();
            break;
        }
        // Speed: fold in any external retarget, then write the schedule.
        match speedhold::taxi_target() {
            Some(t) if (t - last_sched).abs() > 0.01 => {
                // Someone else moved the target: a spoken "stop" parks the
                // schedule; a spoken pace ("go", "taxi at 12") becomes the
                // new base.
                if t < 1.0 {
                    ext_stopped = true;
                } else {
                    base = t;
                    ext_stopped = false;
                }
                last_sched = t;
            }
            _ => {}
        }
        if !ext_stopped {
            let mut sched = schedule_kts(base, &route, progress, gs_kts);
            if join_allow.unwrap_or(0.0) > 0.0 {
                sched = sched.min(route.pace.join_cap_kts(err));
            }
            // Applied every tick, joined or not: the route's bends know nothing
            // about where the aircraft actually is, and the join cap lets go
            // too early to cover it.
            sched = sched.min(route.pace.track_cap_kts(lateral, err));
            // The traffic ahead, through the same one writer: held on
            // TRAFFIC_HOLD_KTS, idle and brakes like every other wait.
            if let Some(c) = traffic_cap {
                sched = sched.min(c);
            }
            if (sched - last_sched).abs() > 0.01 {
                speedhold::set_taxi_target(sched);
                last_sched = sched;
            }
        }
        pacer.wait(std::time::Duration::from_millis(DRIVE_TICK_MS)).await;
    }
    // Politeness on the way out — expiry would do it within the second.
    let _ = xp.set_float(TAXI_STEER, 0.0).await;
    tiller.release(&xp).await;
    // The brake-steer datarefs do NOT expire like the plugin channel does, so
    // a braked wheel left behind would drag forever. Every exit is a `break`
    // to here (checked — no returns inside the loop), so one release covers
    // them all.
    if bs_on {
        let _ = xp.set_float(LEFT_BRAKE, 0.0).await;
        let _ = xp.set_float(RIGHT_BRAKE, 0.0).await;
    }
    // Only the newest drive may clear the flag (the run's rule, [`RUN`]).
    gen.lower_flag();
    tracing::info!(gen = gen.id, "auto taxi stopped");
}
