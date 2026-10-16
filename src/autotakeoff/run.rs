//! The loop: the inherited autopilot cleared, the climb engaged, `fly`, whole.

use super::*;

/// An autopilot found on at the roll: its servos in, or its heading or
/// vertical-speed mode lit. Pure.
pub fn ap_inherited(servos: f64, heading: f64, vertical: f64) -> bool {
    servos > 0.5 || heading > 0.5 || vertical > 0.5
}

/// A takeoff starts with the autopilot OFF, whatever the last flight left
/// on. LEMG 13 SF50 (sortie 1790607325, issue #160): the runner loaded the
/// SF50 onto the runway while the SR22 before it was on autoland, and the
/// roll began with `autopilot_on` 1 and HDG and VS lit (`ap_state` 50; the 23
/// other recorded takeoffs began with neither lit). At 300 ft the engage found both
/// modes lit, pressed nothing, and nothing flew the aircraft: VS commanded
/// 1,932 then 1,500 and -500 while it climbed at 2,000 to 3,400 fpm, the bug
/// on 132 while the heading drifted to 111 and the bank grew from 1 to 45
/// degrees left in 100 s, to 21,900 ft in circles. Servos and flight
/// director off on the ground (the takeoff's own channels fly the roll and the
/// rotation), read back, blackboxed `ap_inherited`; the engage at 300 ft is
/// then the fresh engage every good takeoff made. A failed read is no
/// measurement: nothing is written. `None` = nothing was on (or no reading).
pub(super) async fn clear_inherited_autopilot(xp: &dyn crate::xplane::XPlane) -> Option<bool> {
    let (servos, heading, vertical) = tokio::join!(
        crate::autoflight::read_status(&*xp, SERVOS_ON),
        crate::autoflight::read_status(&*xp, HDG_STATUS),
        crate::autoflight::read_status(&*xp, VS_STATUS)
    );
    let [servos, heading, vertical] =
        crate::control::measurements("autotakeoff", [servos, heading, vertical])?;
    if !ap_inherited(servos, heading, vertical) {
        return None;
    }
    let fired = autoflight::press(xp, Button::DisconnectFd).await.is_ok();
    let after = crate::autoflight::read_status(&*xp, SERVOS_ON).await.ok();
    let cleared = fired && after.is_some_and(|v| v < 0.5);
    crate::blackbox::record(
        "autotakeoff",
        serde_json::json!({
            "event": "ap_inherited", "servos": servos, "hdg": heading, "vs": vertical,
            "servos_after": after, "cleared": cleared,
        }),
    );
    Some(cleared)
}

/// The takeoff's outcome when the autopilot's pitch moved nothing (#181):
/// the callers end without "rejected" and without "your aircraft".
pub const AP_NO_PITCH: &str = "ap_no_pitch";

/// The autopilot took the climb and its pitch moves nothing (#181): the
/// recovery flies the pitch on our column to the selected level (a climb
/// still ahead) and never gives it back to the autopilot; the run ends
/// [`AP_NO_PITCH`] for the caller.
pub(super) async fn no_pitch_take_over(xp: &XPlaneClient, p: &Params, voice: &Option<Voice>, why: &str) {
    let (dial, alt) = tokio::join!(autoflight::read_dial(xp, Dial::Altitude), xp.get_float(ALT_IND_FT));
    let hold_ft = crate::climb_watch::takeover_hold_ft(dial.ok(), alt.ok());
    let started = crate::tools::recover::take_over(
        p.base_url.clone(),
        voice.clone(),
        crate::tools::recover::TakeOver {
            why: format!("autotakeoff_{why}"),
            min_safe_ias: None,
            hold_kts: None,
            ap_no_pitch: true,
            hold_ft,
        },
    );
    crate::blackbox::record(
        "autotakeoff",
        serde_json::json!({"event": "ap_no_pitch", "why": why, "hold_ft": hold_ft,
                           "recovery_started": started}),
    );
    RUN.record(sim::run::Outcome::Failed(AP_NO_PITCH.into()));
}

/// An engage still not read back after this long is given to the recovery:
/// the 737's took three presses in 3 s when it worked (EGKK 08R, 1792149156).
pub(super) const ENGAGE_GIVE_UP_S: f64 = 20.0;

/// A mode command is only a request; handoff needs live mode and dial readbacks.
/// The SERVOS are read too: lit modes are not an autopilot flying (#160).
/// The servos go in through the one engage door (`ap_engage::press_logged`:
/// the flight director first, which this press skipped until the refactor
/// audit's #5, each press `ap_engage` on the tape, `tries` counting them);
/// the caller re-calls this every tick until everything reads back, so the
/// confirmation is this function's own read-back, and the VS dial is still
/// written AFTER the modes (an engage syncs it away).
/// How long the engage leaves the panel after a press before judging the
/// modes again. The 737's annunciators lag the press: the climb engage
/// pressed servos and VS six times in 0.7 s (LEGR 2026-10-06, sortie
/// 1791274391-full_auto, #226), each a toggle.
pub(super) const ENGAGE_SETTLE: std::time::Duration = std::time::Duration::from_millis(1500);

/// Whether the engage may press again: nothing pressed yet, or the last
/// press has had [`ENGAGE_SETTLE`]. Pure.
pub fn engage_may_press(pressed_at: Option<std::time::Instant>, now: std::time::Instant) -> bool {
    pressed_at.is_none_or(|t| now.duration_since(t) >= ENGAGE_SETTLE)
}

pub(super) async fn engage_climb(
    xp: &dyn crate::xplane::XPlane,
    fd: &[String],
    tries: &mut u32,
    vs: f64,
    last: &mut Option<f64>,
    pressed_at: &mut Option<std::time::Instant>,
) -> bool {
    if crate::xplane::sim_frozen(xp).await {
        return false;
    }
    let (heading, vertical, servos) = tokio::join!(
        crate::autoflight::read_status(&*xp, HDG_STATUS),
        crate::autoflight::read_status(&*xp, VS_STATUS),
        crate::autoflight::read_status(&*xp, SERVOS_ON)
    );
    let Some([heading, vertical, servos]) =
        crate::control::measurements("autotakeoff", [heading, vertical, servos])
    else {
        return false;
    };
    if heading <= 0.5 || vertical <= 0.5 || servos <= 0.5 {
        let now = std::time::Instant::now();
        if !engage_may_press(*pressed_at, now) {
            return false;
        }
        *pressed_at = Some(now);
        *tries += 1;
        // Which read kept it pressing: none was on the tape (1792152599).
        crate::blackbox::record(
            "autotakeoff",
            serde_json::json!({"event": "engage_status", "try": *tries,
                               "hdg": heading, "vs": vertical, "servos": servos}),
        );
        if !crate::ap_engage::press_logged(xp, fd, "autotakeoff", "climb", *tries).await {
            return false;
        }
    }
    if !autoflight::bring_heading(xp, Some(heading), "autotakeoff_climb")
        .await
        .sent
    {
        return false;
    }
    let vs_mode = autoflight::bring_vs(xp, Some(vertical), "autotakeoff_climb").await;
    if vs_mode.pressed(Button::VerticalSpeed) {
        *last = None; // a mode change synchronises the dial
        if !vs_mode.sent {
            return false;
        }
    }
    if !autoflight::set_dial_confirmed("autotakeoff", xp, Dial::Vs, vs, last, 40.0).await {
        return false;
    }
    let (heading, vertical, servos) = tokio::join!(
        crate::autoflight::read_status(&*xp, HDG_STATUS),
        crate::autoflight::read_status(&*xp, VS_STATUS),
        crate::autoflight::read_status(&*xp, SERVOS_ON)
    );
    matches!((heading, vertical, servos), (Ok(h), Ok(v), Ok(s))
        if h.is_finite() && v.is_finite() && s.is_finite() && h > 0.5 && v > 0.5 && s > 0.5)
}

pub(super) async fn fly(xp: XPlaneClient, voice: Option<Voice>, mut p: Params) {
    // Find the runway under the nose. A full apt.dat pass, seconds — done
    // here, off the tool's reply path.
    let (lat, lon, psi, var) = tokio::join!(
        xp.get_float(LAT),
        xp.get_float(LON),
        xp.get_float(PSI_TRUE),
        xp.get_float(MAG_VAR)
    );
    let Some([lat, lon, psi, var]) =
        crate::control::measurements("autotakeoff", [lat, lon, psi, var])
    else {
        alert(
            &voice,
            Priority::Caution,
            "I can't read our position. Your takeoff.",
        );
        return;
    };
    let cands = match crate::runway::nearby_runways(&p.apt_dat, lat, lon, 2.0) {
        Ok(c) => c,
        Err(e) => {
            tracing::warn!(?e, "auto takeoff: apt.dat unreadable");
            alert(
                &voice,
                Priority::Caution,
                "I can't read the runway data. Your takeoff.",
            );
            return;
        }
    };
    let Some((rwy, remaining_m)) = runway_fit(&cands, lat, lon, psi, p.needed_m) else {
        // Say WHICH of the four things is wrong. They want four different
        // actions, and "not on the runway yet" was said to an aircraft
        // sitting on the runway with the tower telling it to vacate.
        let why = why_no_fit_cleared(
            &cands,
            lat,
            lon,
            psi,
            p.needed_m,
            p.cleared_runway.as_deref(),
        );
        let can_line_up =
            crate::autotaxi::pick_line_up(&cands, lat, lon, p.cleared_runway.as_deref(), None)
                .is_some();
        crate::blackbox::record(
            "autotakeoff",
            serde_json::json!({"event": "no_runway_fit", "why": format!("{why:?}")}),
        );
        let (line, urgent) = no_fit_line(&why, can_line_up);
        alert(
            &voice,
            if urgent {
                Priority::Warning
            } else {
                Priority::Caution
            },
            &line,
        );
        return;
    };
    if let Some(clr) = &p.cleared_runway {
        if !runway_matches(&rwy.id, clr) && !cleared_fits_here(&cands, clr, lat, lon, psi) {
            alert(
                &voice,
                Priority::Warning,
                &format!(
                    "Hold on. We're lined up on runway {}, but the clearance is runway {}. Your aircraft.",
                    crate::atis::spoken_runway(&rwy.id),
                    crate::atis::spoken_runway(clr)
                ),
            );
            crate::blackbox::record(
                "autotakeoff",
                serde_json::json!({"event": "wrong_runway", "fitted": rwy.id, "cleared": clr}),
            );
            return;
        }
    }
    // The elevator is the autopilot's to take at 300 ft (#181): a failed
    // servo or pitch trim, or another plugin on the autopilot's brains, and
    // no automation takes off. Weight, balance and who holds the column are
    // logged either way, for the next climb that does not follow its dial.
    let authority = crate::tools::recover::read_pitch_authority(&xp).await;
    let refusal = authority.refusal();
    crate::blackbox::record(
        "autotakeoff",
        serde_json::json!({"event": "pitch_authority", "refused": refusal,
                           "state": authority.json()}),
    );
    if let Some(why) = refusal {
        alert(
            &voice,
            Priority::Warning,
            &format!("{why} I'm not taking off. Your aircraft."),
        );
        return;
    }
    // Every refusal gate has passed — NOW stand the taxi governor down (a
    // line-up ends with it holding position at target zero; it kept the
    // aircraft parked through the scan above). Doing this in the tool stood
    // it down before a refusal and left the aircraft coasting uncontrolled
    // (first sortie it braked against the roll, second it let a refused
    // takeoff roll away — both LXGB, 2026-07-20). Its loop exits within a
    // tick; the moment of free-wheeling is the throttle ramp's first beat.
    // The heading bug is preset HERE, while the governor still holds the
    // aircraft on its brakes: the knob slews by presses, one round trip a
    // degree, and preset after "Brakes released" the Evolution stood 4.7 s
    // with its brakes off and nothing steering, idling from 10.6 to 0.6
    // true before the roll's first tick (LILB 03, 1790712974, 2026-09-29).
    let _ = crate::tools::autopilot::drive_heading_bug_step(
        &xp,
        true_to_mag(rwy.course_deg, var),
        360.0,
    )
    .await;
    crate::tools::speedhold::release();
    tokio::time::sleep(std::time::Duration::from_millis(500)).await;
    // The line-up ends "brakes set" — the parking-brake ratio latched. It
    // is OURS to release: sortie five's first engage sat at full power,
    // groundspeed zero, for twelve seconds against the parked brake and
    // rejected as stagnant. The VALVE goes with it (2026-08-03): left in the
    // set position it traps the roll's own brake applications — the reject
    // path's stopping pulse included — back into the parking brake.
    // Trim for takeoff BEFORE the brakes come off (see `trim_for_takeoff`:
    // a nose-down stop trim is what the AP's climb loses to at the engage).
    // The WHOLE configuration, and the brakes wait for the flaps to arrive:
    // the handle used to be written after "Rolling", and an A330's CONF 2
    // was still running out at sixty knots (EGPF, 2026-09-21). After full
    // auto's after-start call this finds everything set and says nothing.
    // Nothing the last flight left engaged flies this one (#160); before the
    // configuration, which sets the flight directors an airframe declares.
    clear_inherited_autopilot(&xp).await;
    // The takeoff thrust a crew sets for this weight, field and runway
    // (`takeoff_thrust:`): the rating, its N1, the climb N1 at the
    // reduction. Nothing read, nothing declared: the levers to full.
    let thrust_law = crate::aircraft::takeoff_thrust();
    let thrust_plan = match &thrust_law {
        Some(t) => {
            let fit = crate::takeoff_thrust::RunwayFit {
                needed_full_m: p.needed_m,
                available_m: remaining_m,
            };
            let reduction_agl =
                crate::takeoff_config::thrust_reduction_agl_ft().unwrap_or(THRUST_REDUCTION_DEFAULT_FT);
            let plan = crate::takeoff_thrust::plan_now(&xp, t, reduction_agl, Some(fit)).await;
            crate::blackbox::record(
                "autotakeoff",
                serde_json::json!({"event": "thrust_plan", "rating": plan.as_ref().map(|x| &x.rating),
                    "assumed_c": plan.as_ref().and_then(|x| x.assumed_c),
                    "to_n1": plan.as_ref().map(|x| x.to_n1), "climb_n1": plan.as_ref().map(|x| x.climb_n1),
                    "weight_t": plan.as_ref().map(|x| x.weight_t), "oat_c": plan.as_ref().map(|x| x.oat_c),
                    "field_pa_ft": plan.as_ref().map(|x| x.field_pa_ft),
                    "needed_full_m": p.needed_m, "available_m": remaining_m}),
            );
            plan
        }
        None => None,
    };
    let config = crate::takeoff_config::set_for_takeoff_rated(
        &xp,
        p.takeoff_flap_ratio,
        p.level_ft,
        true,
        thrust_plan.as_ref().map(|x| x.rating.as_str()),
    )
    .await;
    if let Some(line) = crate::takeoff_config::line("Takeoff config", &config) {
        alert(&voice, Priority::Advisory, &line);
    }
    if let Some(plan) = &thrust_plan {
        alert(&voice, Priority::Advisory, &crate::takeoff_thrust::line(plan));
    }
    // An abort said during the flap wait: the brakes are still set.
    if xp.cancelled() || !ACTIVE.load(Ordering::Relaxed) {
        return;
    }
    // The rotation and the climb-out are flown on the plugin's yoke
    // channels: tell it which input this airframe's yoke is.
    crate::yoke::arm(&xp, "autotakeoff").await;
    // The takeoff speeds for what the aircraft weighs, off the type's card
    // for the flap it set, where the airframe declares one
    // (`weight_speeds: takeoff`): Vr, and the initial climb at V2 + 20 (the
    // climb hold flies V2 + 15 under it). The Zibo at 45.7 t rotated on a
    // fixed 145 where its FMC's Vr was 111 (LEMG 30, 2026-10-07).
    if let Some((v, kg)) = crate::aircraft::takeoff_speeds_now(&xp, p.takeoff_flap_ratio).await {
        crate::blackbox::record(
            "autotakeoff",
            serde_json::json!({"event": "speeds_weighed", "weight_kg": kg.round(),
                "v1": v.v1, "vr": v.vr, "v2": v.v2,
                "declared_vr": p.vr_kts, "declared_climb": p.initial_climb_kts}),
        );
        p.vr_kts = v.vr;
        p.initial_climb_kts = v.v2 + V2_CLIMB_ADD_KTS;
    }
    crate::tools::controls::release_park_brake_for_roll(&xp, p.park_brake_valve_set).await;
    tracing::info!(
        icao = %rwy.icao, id = %rwy.id, remaining_m, vr = p.vr_kts, "auto takeoff: rolling"
    );
    crate::blackbox::record(
        "autotakeoff",
        serde_json::json!({
            "event": "engaged", "rwy": format!("{} {}", rwy.icao, rwy.id),
            "remaining_m": remaining_m, "vr": p.vr_kts,
        }),
    );
    // Transponder to ALT as the roll starts, the way a crew sets it lining up:
    // set at liftoff alone, a takeoff handed back on the roll left it on
    // standby and the tower called "negative radar contact" with the pilot
    // climbing out (EDLP 24, 2026-09-24). Liftoff asks again (a no-op when
    // it took).
    squawk_altitude(p.base_url.clone(), voice.clone(), true);
    alert(
        &voice,
        Priority::Advisory,
        &format!(
            "Brakes released. Rolling, runway {}.",
            crate::atis::spoken_runway(&rwy.id)
        ),
    );

    // Pre-set the flight director NOW (the heading bug was preset before
    // the brakes came off, above): doing it later would stall the loop
    // while the steer channel expires mid-roll. No VS dial preset: VS-mode
    // engage syncs the dial to the current rate anyway, so it's written
    // after the engage instead.
    for c in &p.cfg.fd_commands {
        let _ = xp.run_command(c).await;
    }

    // Takeoff flaps from the profile, set before speed builds. 0.0 on a
    // clean-takeoff type still runs, guaranteeing the config off the gate.
    if let Some(ratio) = p.takeoff_flap_ratio {
        let _ = crate::tools::flaps::set_handle(&xp, ratio).await;
    }
    // Nothing to retract if we took off clean (<= a whisker of flap).
    let mut flaps_retracted = p.takeoff_flap_ratio.unwrap_or(0.0) <= 0.01;
    // The setting the law believes is selected, for the schedule to walk down.
    let mut flap_cmd = p.takeoff_flap_ratio.unwrap_or(0.0);
    // The flaps' own pace for the clean-up walk (#169).
    let mut flap_pace = FlapPace::default();

    let mut airborne = false;
    let mut rotated = false;
    let mut ap_engaged = false;
    let climb_out = climb_out_for(p.handoff_agl_ft);
    if let Some(c) = climb_out {
        crate::blackbox::record(
            "autotakeoff",
            serde_json::json!({"event": "climb_out", "accel_agl_ft": c.accel_agl_ft,
                               "max_fpm": c.max_fpm, "hold_kts": p.initial_climb_kts}),
        );
    }
    let mut handoff_wait: Option<std::time::Instant> = None;
    let _detail = crate::auto_status::DetailGuard::hold(&DETAIL);
    let mut gear_raised = !p.retract_gear;
    let mut lights_off = false;
    let mut positive_called = false;
    let mut thr_challenged = false;
    // The hardware lever where it sat as the roll began, on an airframe that
    // can snap its levers to it (`throttle_hardware_axis`).
    let hw_axis = crate::aircraft::throttle_hardware_axis();
    let hw_baseline = match &hw_axis {
        Some(name) => xp.get_float(name).await.ok().filter(|v| v.is_finite()),
        None => None,
    };
    // The initial climb's speed hold by pitch, on an airframe that declares
    // its climb attitude (`climb_pitch_max_deg`); the fixed rotation
    // attitude elsewhere.
    let climb_hold = ClimbHold::new(
        p.initial_climb_kts,
        crate::aircraft::live_profile().and_then(|lp| lp.climb_pitch_max_deg),
    );
    if let Some(h) = climb_hold {
        crate::blackbox::record(
            "autotakeoff",
            serde_json::json!({"event": "climb_hold", "hold_kts": h.hold_kts,
                               "pitch_max_deg": h.pitch_max_deg}),
        );
    }
    let mut pitch_law = RotatePitch::with_authority(
        p.tail_strike_deg,
        p.pitch_authority_down.map_or(PITCH_MIN, |a| -a.abs()),
    )
    .with_climb_hold(climb_hold)
    .with_rotate_rate(crate::aircraft::live_profile().and_then(|lp| lp.reference.as_ref().and_then(|r| r.rotate_rate_deg_s)))
    // The airframe's fitted rotation elevator (#184), fed forward.
    .with_rotate_model(crate::aircraft::live_profile().and_then(|lp| lp.rotate_elevator));
    // Thrust reduction at the declared height (`takeoff_config:`).
    let reduction = crate::takeoff_config::thrust_reduction();
    let mut reduced = false;
    // The levers flown to the plan's N1 where the airframe declares it: a
    // ceiling on every other law's lever, led on the line and trimmed on
    // the measured N1 (`takeoff_thrust::N1Loop`).
    let mut n1_loop = match (&thrust_law, &thrust_plan) {
        (Some(t), Some(_)) => Some(crate::takeoff_thrust::N1Loop::new(t)),
        _ => None,
    };
    let n1_reduction_agl =
        crate::takeoff_config::thrust_reduction_agl_ft().unwrap_or(THRUST_REDUCTION_DEFAULT_FT);
    let mut n1_climb = false;
    let mut wings = WingsLevel::default();
    let mut last_vs_cmd: Option<f64> = None;
    // The servo presses of the 300 ft engage (`engage_climb`).
    let mut engage_tries: u32 = 0;
    let mut engage_pressed_at: Option<std::time::Instant> = None;
    // When the engage began failing to read back above the engage height.
    let mut engage_since: Option<std::time::Instant> = None;
    // The autopilot's climb from the engage on: the climb flown, carried
    // into the law without a step (#150).
    let mut climb_vs = ClimbVs::default();
    // The MCP window leaves V2 once the takeoff accelerates, on an airframe
    // whose window the takeoff set (`takeoff_config: v2_kts`); once.
    let mut window_owed = crate::aircraft::live_profile()
        .and_then(|lp| lp.takeoff_config.as_ref().and_then(|t| t.v2_kts))
        .is_some_and(|v| v.is_finite() && v > 0.0);
    let window_kts = clean_up_speed_kts(&p.flap_schedule, p.vy_kts);
    publish_climb_handover(None);
    let mut last_thr_cmd = 0.3_f64;
    let mut full_thr_at: Option<f64> = None;
    let spool_s = crate::aircraft::lever_profile().and_then(|a| a.takeoff_spool_s);
    // The manual's takeoff torque, for this airframe, read once per roll.
    let power_limit = takeoff_power();
    // The roll section's heading error, one tick stale, for the throttle's
    // swinging-nose hold (throttle is commanded before position is read).
    let mut last_err = 0.0_f64;
    let mut steer_law = RollSteer::for_airframe();
    let brake_steer = brake_steers();
    // The centreline cut: the closure-time cut on every airframe, fed the
    // drift and held harder on the scheduled law only (see [`RollCut`]).
    let mut roll_cut = RollCut::for_law(&steer_law);
    // The heading error's last value, for the yaw rate an aligning pivot
    // is capped on (the roll reads no yaw rate of its own).
    let mut prev_align_err: Option<f64> = None;
    // The power waits for the nose on a brake-steered airframe
    // ([`RollAlign`]); held from the first tick until the first read says.
    let mut roll_align = RollAlign::default();
    // A free castoring nose's roll brakes: the one ground law.
    let mut roll_ground =
        crate::aircraft::nose_free().then(crate::ground_steer::GroundSteer::for_airframe);
    let mut align_hold = brake_steer;
    let mut last_tick: Option<std::time::Instant> = None;
    let mut thrust_watch = autoflight::ThrustWatch::default();
    let thrust_t0 = std::time::Instant::now();
    let roll_start = std::time::Instant::now();
    let mut frozen_secs = 0.0_f64;
    // The tick's writes, sent together (`sim::state::Writes`).
    let mut writes = sim::state::Writes::new();
    loop {
        // The tick runs to its period, not sleep + work (`ROLL_TICK_MS`).
        let pacer = sim::state::Pacer::start();
        if xp.cancelled() || !ACTIVE.load(Ordering::Relaxed) {
            RUN.record(sim::run::Outcome::PilotOverride);
            // Released externally ("my controls"): the release path already
            // spoke. Steering expires by silence; power stays where it is —
            // a pilot taking over mid-roll wants it set.
            let _ = xp.set_float(TAXI_STEER, 0.0).await;
            crate::blackbox::record("autotakeoff", serde_json::json!({"event": "released"}));
            return;
        }
        if REJECT.swap(false, Ordering::Relaxed) {
            if airborne {
                alert(
                    &voice,
                    Priority::Caution,
                    "We're already flying. Your aircraft. The autopilot is holding the climb.",
                );
                crate::blackbox::record(
                    "autotakeoff",
                    serde_json::json!({"event": "handoff_airborne_abort"}),
                );
            } else {
                // The pilot's "reject the takeoff" flies the same stop as
                // the loop's own rejects, to a standstill and the parking
                // brake, before "your aircraft" (LEMG 13, 2026-09-28).
                reject_rolling(
                    &xp,
                    &voice,
                    "Rejecting takeoff.",
                    "pilot_abort",
                    &rwy,
                    &mut steer_law,
                    p.park_brake_valve_set,
                )
                .await;
            }
            return;
        }
        // ONE read a tick (`sim::state`): the freeze flags, position,
        // heading, groundspeed, height and vertical speed as ONE frame saw
        // them (#157: read separately, latitude and longitude paired two
        // frames, up to 2.9 m of cross-track for a tick on LEMG 13), and the
        // speeds, attitude, rates, yields and torque with them, the
        // profile's throttle beside it in the same round. Every round of
        // requests is a sim frame: the freeze check, the read and four
        // rounds of writes made the roll tick 0.34 s (dt median, iMac at 20
        // fps, 2026-09-30) against its 150 ms sleep. An older plugin falls
        // back to the separate reads, still one round, and the tick says
        // which (`pos`).
        let thr_dref = throttle_dataref();
        // The panel's reads beside the frame (`autoflight::Panel::tick_names`):
        // the stock autothrottle, or a declared MCP's lamps.
        let af_panel = autoflight::panel();
        // The column as the sim holds it, beside our command: on the Zibo
        // the pitch channel rides its own input (`crate::yoke`), and the
        // LEMG climb-out held -0.5 while the nose rose to 23 degrees with
        // nothing to say whether the column moved (2026-10-07).
        let mut extra_names: Vec<String> = vec![thr_dref.clone(), YOKE_PITCH_READ.to_string()];
        extra_names.extend(af_panel.tick_names());
        let extra_refs: Vec<&str> = extra_names.iter().map(String::as_str).collect();
        // The N1 beside the frame, in the same round, where the levers are
        // flown to it.
        let n1_read = async {
            if n1_loop.is_some() {
                xp.get_float_array(ENGN_N1).await.ok()
            } else {
                None
            }
        };
        let ((st, extra), n1_arr) = tokio::join!(
            sim::state::read_with(&xp, sim::state::Want(ROLL_WANT), &extra_refs),
            n1_read
        );
        let n1_now = n1_arr.as_deref().and_then(crate::takeoff_thrust::mean_n1);
        // Pause or replay: freeze — no writes, and the stagnation clock
        // slides instead of burning (the recover watcher's rule).
        if st.frozen() {
            tokio::time::sleep(std::time::Duration::from_millis(400)).await;
            frozen_secs += 0.4;
            continue;
        }
        use sim::state::F;
        let pos = st.position();
        let (ias, on_ground, steer_yield, theta, phi, pitch_q) = (
            st.get(F::Ias),
            st.get(F::OnGround),
            st.get(F::SteerYield),
            st.get(F::Theta),
            st.get(F::Phi),
            st.get(F::Q),
        );
        let (pitch_yield, roll_yield, torque_nm, yaw_r) = (
            st.get(F::PitchYield),
            st.get(F::RollYield),
            st.get(F::Trq0),
            st.get(F::R),
        );
        let mut extra = extra.into_iter();
        let thr_read = extra.next().unwrap_or_else(|| Err(anyhow::anyhow!("unread")));
        let thr_seen = thr_read.as_ref().ok().copied();
        let yoke_read = extra.next().and_then(|r| r.ok()).filter(|v| v.is_finite());
        // The levers are the takeoff's: an ENGAGED autothrottle (armed is
        // the configuration's, never touched) is taken out and said once.
        {
            let panel_reads: Vec<Option<f64>> = extra.map(|r| r.ok()).collect();
            let engaged = takeoff_watch_reads(
                af_panel.tick_from(&panel_reads).at_engaged,
                reduced,
                crate::aircraft::athr_cl_detent(),
            );
            let step = thrust_watch.step(thrust_t0.elapsed().as_secs_f64(), engaged);
            if let Some(line) = autoflight::act_thrust(&xp, step, "autotakeoff").await {
                alert(&voice, Priority::Caution, line);
            }
        }
        let (lat, lon) = sim::position::lat_lon(&pos);
        let psi = sim::position::field(&pos, |p| p.psi_true_deg);
        let gs_ms = sim::position::field(&pos, |p| p.gs_ms);
        let agl_m = sim::position::field(&pos, |p| p.agl_m);
        // The flight model's vertical speed (`vh_ind_fpm`): the climb the
        // pitch is flying when the autopilot takes it (`ClimbVs`).
        let vs_read = sim::position::field(&pos, |p| p.vs_fpm);
        let pos_source = sim::position::source_word(&pos);
        // A failed read is no measurement: the engage then writes the law.
        let vs_flown = vs_read.ok().filter(|v| v.is_finite());
        let torque_ftlb = torque_nm
            .ok()
            .filter(|v| v.is_finite())
            .map(|v| v * NM_TO_FTLB);
        let Some([lat, lon, psi, gs_ms, ias, agl_m, on_ground, theta_deg, phi_deg, q]) =
            crate::control::measurements(
                "autotakeoff",
                [
                    lat, lon, psi, gs_ms, ias, agl_m, on_ground, theta, phi, pitch_q,
                ],
            )
        else {
            tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            continue;
        };
        let gs_kts = gs_ms * MS_TO_KTS;
        let agl_ft = agl_m * M_TO_FT;
        let pitch_rate = Some(q);
        let now_t = std::time::Instant::now();
        let dt = last_tick
            .map_or(0.15, |t| (now_t - t).as_secs_f64())
            .clamp(0.05, 1.0);
        last_tick = Some(now_t);

        // The pilot's feet took the pedals (the plugin dropped the override
        // already): the takeoff continues as a manual one, power set.
        if !airborne && steer_yield.is_ok_and(|v| v > 0.5) {
            RUN.record(sim::run::Outcome::PilotOverride);
            crate::flight::note_pilot_takeover("takeoff: pedals");
            alert(&voice, Priority::Caution, "Your aircraft. Power is set.");
            crate::blackbox::record("autotakeoff", serde_json::json!({"event": "pedal_yield"}));
            return;
        }
        // The pilot's hands took the yoke during rotation or the hand-flown
        // climb (the plugin dropped the override already): theirs — either
        // axis is the whole yoke.
        if rotated
            && !ap_engaged
            && (pitch_yield.is_ok_and(|v| v > 0.5) || roll_yield.is_ok_and(|v| v > 0.5))
        {
            RUN.record(sim::run::Outcome::PilotOverride);
            crate::flight::note_pilot_takeover("takeoff: yoke");
            alert(&voice, Priority::Caution, "Your aircraft. Power is set.");
            crate::blackbox::record("autotakeoff", serde_json::json!({"event": "yoke_yield"}));
            return;
        }

        // Throttle: ramped to full, held at half power while the nose is
        // still swinging at low speed (full power on a swinging nose is
        // the maximum yaw kick at the worst moment), advances always
        // slewed, with the pilot's hand honored. A read well below the
        // last COMMAND is challenged ONCE (a hardware axis the sim
        // re-drives jitters; a hand keeps winning), then it's a pilot
        // reject — hands off everything.
        let elapsed = roll_start.elapsed().as_secs_f64() - frozen_secs - roll_align.held_s();
        // A hold holds the aircraft: the creep's lever and brake.
        let (creep_cap, hold_brake) = if align_hold && !airborne {
            roll_align.creep(gs_kts)
        } else {
            (1.0, 0.0)
        };
        let thr_want = aligning_throttle_capped(
            torque_limited(
                tick_throttle_want(airborne, elapsed, last_err, gs_kts, ias, p.vne_kts),
                last_thr_cmd,
                torque_ftlb,
                ias,
                power_limit.as_ref(),
            ),
            align_hold && !airborne,
            creep_cap,
        );
        let (thr_want, reducing, n1_cap, n1_step) = match (n1_loop.as_mut(), &thrust_plan) {
            (Some(lp), Some(plan)) => {
                n1_climb = n1_climb || (airborne && agl_ft >= n1_reduction_agl);
                let target = if n1_climb { plan.climb_n1 } else { plan.to_n1 };
                let (cap, step) = lp.step(target, n1_now, last_thr_cmd, dt);
                (thr_want.min(cap), n1_climb && thr_want >= cap, Some(cap), Some(step))
            }
            _ => {
                let (w, r) = thrust_reduced(thr_want, airborne, agl_ft, reduction);
                (w, r, None, None)
            }
        };
        let thr_cmd = throttle_slew(last_thr_cmd, thr_want, dt);
        let thr_cmd = if reducing {
            reduction_slew(thr_cmd, last_thr_cmd, dt)
        } else {
            thr_cmd
        };
        if reducing && !reduced {
            reduced = true;
            alert(&voice, Priority::Advisory, "Climb thrust.");
            let n1 = xp.get_float_array(ENGN_N1).await.ok();
            crate::blackbox::record(
                "autotakeoff",
                serde_json::json!({"event": "thrust_reduction", "agl": agl_ft, "ias": ias,
                                   "from": last_thr_cmd, "to": thr_want, "n1": n1,
                                   "n1_target": thrust_plan.as_ref().map(|x| x.climb_n1)}),
            );
        }
        // FAIL OPEN, deliberately: a failed throttle read impersonates our own
        // command, so an API blink can never register as a pilot pull mid-roll.
        // The cost is one tick of blindness to a real hand; the alternative is
        // a takeoff handed away by a hiccup.
        let thr_now = thr_read.unwrap_or(thr_cmd);
        let lever_pulled = thr_now < last_thr_cmd - 0.25 && elapsed > 2.0;
        let hand = if lever_pulled {
            let now = match &hw_axis {
                Some(name) => xp.get_float(name).await.ok().filter(|v| v.is_finite()),
                None => None,
            };
            let hand = pilot_moved_throttle(hw_axis.is_some(), hw_baseline, now);
            if !hand {
                crate::blackbox::record(
                    "autotakeoff",
                    serde_json::json!({"event": "throttle_snap_ignored", "read": thr_now,
                                       "cmd": last_thr_cmd, "axis": now, "axis_base": hw_baseline}),
                );
            }
            hand
        } else {
            false
        };
        if hand {
            if thr_challenged {
                RUN.record(sim::run::Outcome::PilotOverride);
                alert(&voice, Priority::Warning, "Your throttle. Your aircraft.");
                crate::blackbox::record(
                    "autotakeoff",
                    serde_json::json!({"event": "throttle_yield"}),
                );
                let _ = xp.set_float(TAXI_STEER, 0.0).await;
                return;
            }
            thr_challenged = true;
        } else {
            thr_challenged = false;
        }
        // Queued, and sent with the tick's other writes in ONE round.
        writes.set(&thr_dref, thr_cmd);
        last_thr_cmd = thr_cmd;
        if full_thr_at.is_none() && lever_at_takeoff_power(thr_cmd, n1_cap) {
            full_thr_at = Some(elapsed);
        }

        if !airborne {
            // Not accelerating: something is holding the aircraft.
            if roll_stagnant(elapsed, full_thr_at, spool_s, gs_kts) {
                reject_rolling(
                    &xp,
                    &voice,
                    "We're not accelerating. Rejecting takeoff.",
                    "stagnant",
                    &rwy,
                    &mut steer_law,
                    p.park_brake_valve_set,
                )
                .await;
                return;
            }
            let along_m = along_track_nm(lat, lon, &rwy) * M_PER_NM;
            let remaining = rwy.length_m - (-along_m).max(0.0);
            if ground_reject_allowed(rotated, on_ground) && must_reject(remaining, ias, p.vr_kts) {
                reject_rolling(
                    &xp,
                    &voice,
                    "Runway is running out. Rejecting takeoff.",
                    "runway_end",
                    &rwy,
                    &mut steer_law,
                    p.park_brake_valve_set,
                )
                .await;
                return;
            }
            // Steer the centreline: PD plus the learned P-factor trim,
            // slew-limited (no tick slams the wheel).
            let xtk_m = cross_track_nm(lat, lon, &rwy) * M_PER_NM;
            let psi_off = crate::angles::signed_diff_deg(psi, rwy.course_deg);
            let cut = roll_cut.update(xtk_m, psi_off, gs_kts, dt);
            let (xtk_rate, drift) = (cut.rate_m_s, cut.drift_deg);
            let target = (rwy.course_deg + cut.target_off_deg).rem_euclid(360.0);
            let err = crate::angles::signed_diff_deg(target, psi);
            last_err = err;
            if ground_reject_allowed(rotated, on_ground) {
                let off_course = crate::angles::signed_diff_deg(psi, rwy.course_deg);
                if let Some(why) = lateral_excursion(xtk_m, off_course, gs_kts) {
                    crate::blackbox::record(
                        "autotakeoff",
                        serde_json::json!({"event": "excursion", "why": why,
                            "xtk_m": xtk_m, "off_course": off_course, "gs": gs_kts}),
                    );
                    reject_rolling(
                        &xp,
                        &voice,
                        "We're leaving the centreline. Rejecting takeoff.",
                        "lateral_excursion",
                        &rwy,
                        &mut steer_law,
                        p.park_brake_valve_set,
                    )
                    .await;
                    return;
                }
            }
            // A failed read is no measurement: logged null, the law falls back.
            let yaw_dps = yaw_r.ok().filter(|v| v.is_finite());
            steer_law.set_torque(torque_ftlb);
            let steer = steer_law.step_with_yaw(err, dt, gs_kts, thr_cmd, yaw_dps);
            // The brake half, on an airframe that has no other low-speed
            // steering; zero once rotated (the channel expires anyway).
            // The sim's yaw rate where read; else the error differenced.
            let align_yaw = yaw_dps
                .unwrap_or_else(|| prev_align_err.map_or(0.0, |p| -(err - p) / dt.max(0.05)));
            prev_align_err = Some(err);
            if brake_steer && !rotated && align_hold {
                match roll_align.step(err, align_yaw, gs_kts, dt) {
                    RollAlignState::Hold => {}
                    RollAlignState::Go => {
                        align_hold = false;
                        crate::blackbox::record(
                            "autotakeoff",
                            serde_json::json!({"event": "roll_aligned", "err": err,
                                               "yaw_dps": align_yaw, "gs": gs_kts,
                                               "held_s": roll_align.held_s()}),
                        );
                    }
                    RollAlignState::Failed => {
                        crate::blackbox::record(
                            "autotakeoff",
                            serde_json::json!({"event": "not_aligned", "err": err,
                                               "yaw_dps": align_yaw, "gs": gs_kts,
                                               "held_s": roll_align.held_s()}),
                        );
                        reject_rolling(
                            &xp,
                            &voice,
                            &format!(
                                "I can't straighten us on the runway, {:.0} degrees off. \
                                 Rejecting takeoff.",
                                err.abs()
                            ),
                            "not_aligned",
                            &rwy,
                            &mut steer_law,
                            p.park_brake_valve_set,
                        )
                        .await;
                        return;
                    }
                }
            }
            let (bl, br) = if brake_steer && !rotated && roll_ground.is_some() {
                let law = roll_ground.as_mut().expect("checked");
                let yaw = match yaw_dps {
                    Some(r) => crate::ground_steer::Yaw::Now(r),
                    None => crate::ground_steer::Yaw::Mean(align_yaw),
                };
                let split = free_roll_brakes(law, err, yaw, steer, gs_kts, dt, align_hold);
                if align_hold {
                    with_hold_brake(split, hold_brake)
                } else {
                    split
                }
            } else if brake_steer && !rotated {
                let split = roll_brake_split_aligning(steer, gs_kts, err, align_yaw);
                if align_hold {
                    let split = roll_align.trimmed(split, err, align_yaw, gs_kts, dt);
                    with_hold_brake(split, hold_brake)
                } else {
                    split
                }
            } else {
                (0.0, 0.0)
            };
            if brake_steer {
                writes
                    .set(crate::autotaxi::LEFT_BRAKE, bl)
                    .set(crate::autotaxi::RIGHT_BRAKE, br);
            }
            // Rotated with the wheels off (under the 8 ft liftoff): the
            // rudder lets go, as it does at the liftoff, and the track is the
            // wings' (below). The roll law is still stepped, so a wheel that
            // touches again gets its rudder back on the next tick.
            // 1790699819 (LEMG 13, 2026-09-29, #184): 3.4 s floating at 81 to
            // 95 KIAS, the rudder steering a heading target 2 to 2.7 degrees
            // off while the cross-track ran 2.7 to 7.1 m.
            let floating = rotated && on_ground < 0.5;
            let rudder = if floating { 0.0 } else { steer };
            writes.set(TAXI_STEER, rudder);
            if !rotated {
                DETAIL.set(format!(
                    "rolling, {} kt, rotate at {} kt",
                    (ias / 5.0).round() * 5.0,
                    p.vr_kts.round()
                ));
            }
            crate::blackbox::record(
                "autotakeoff",
                serde_json::json!({
                    "event": "roll", "gs": gs_kts, "ias": ias, "err": err, "yoke": yoke_read,
                    "thr_read": thr_seen,
                    // Attitude and the commanded ramp: the Luton tail strike
                    // happened during the roll and the log carried no theta at
                    // all, so the peak that struck is simply not in the record.
                    "theta": theta_deg, "pitch_tgt": pitch_law.target_deg(),
                    "q": pitch_rate,
                    "steer": steer, "trim": steer_law.trim(), "xtk_m": xtk_m,
                    // The wheels (`onground_any`): a rotated tick without them
                    // is the float, rudder released and banked for the track;
                    // the scorer judges the centreline on the wheels.
                    "wheels": on_ground >= 0.5,
                    "bl": bl, "br": br,
                    "xtk_rate": xtk_rate,
                    // The heading and the target it was steered to: without
                    // them the rudder's yaw per unit of steer cannot be read
                    // off a roll (LIBP 04 held 3 degrees of error for six
                    // seconds at 86 to 105 kt on 0.15 to 0.25 of steer).
                    "psi": psi, "tgt": target, "drift": drift,
                    // The cross-track the cut was taken on (the median of
                    // three reads on the scheduled law), its proportional
                    // cut and the cross-track trim's (see [`RollCut`]).
                    "xtk_used": cut.xtk_m, "cut": cut.cut_deg, "cut_i": cut.cut_i_deg,
                    // The rate the cut led with: `xtk_rate`, or on an
                    // airframe declaring its track lag the heading's rate.
                    "xtk_lead": cut.lead_m_s,
                    // Which read the position came from: "plugin" is one
                    // frame (xcopilot/position), "split" the separate stock
                    // reads of an older plugin (#157).
                    "pos": pos_source,
                    // The sim's yaw rate the scheduled law damps on (null on a
                    // failed read), and whether it flew (the 737's
                    // `roll_rudder:`). Until 2026-09-28 `r` carried the
                    // brake split's differenced error instead, spikes and all.
                    "r": yaw_dps, "law": if steer_law.scheduled() { "rudder" } else { "default" },
                    "thr": thr_cmd, "remaining_m": remaining, "dt": dt, "trq_ftlb": torque_ftlb,
                    // The N1 flown to (`takeoff_thrust:`; null elsewhere):
                    // the engines' mean, the lever the law allows, its step.
                    "n1": n1_now, "n1_cap": n1_cap, "n1_step": n1_step.map(|s| format!("{s:?}")),
                    // The power waiting for the nose ([`RollAlign`]).
                    "align_hold": align_hold, "hold_brake": hold_brake,
                }),
            );
            // Rotate: ELEVATOR, through the plugin's pitch channel — the AP
            // can't do this (its VS mode syncs to the current rate at
            // engage: zero, nose held down, sortie five levitated off at 98
            // knots with 250 m left). Only with the nose roughly straight
            // (or the runway running out): rotating yawed launches the
            // aircraft sideways off the strip.
            if !rotated && ias >= p.vr_kts && rotate_ok(err, remaining) {
                rotated = true;
                DETAIL.set("rotating");
                alert(&voice, Priority::Advisory, "Rotate.");
                crate::blackbox::record(
                    "autotakeoff",
                    serde_json::json!({"event": "rotate", "ias": ias, "remaining_m": remaining}),
                );
            }
            if rotated {
                // Wings held level through the pull — the rudder steering
                // still owns the heading until liftoff.
                pitch_law.on_wheels(on_ground >= 0.5);
                pitch_law.climb_fpm(vs_flown);
                let pitch = pitch_law.step_climb(theta_deg, pitch_rate, dt, Some((ias, agl_ft)));
                if pitch_law.model.is_some() {
                    // The column the rotation wrote and the model's share of
                    // it (#184: the roll record carries no column, and the
                    // SR22's lag was the column's).
                    crate::blackbox::record(
                        "autotakeoff",
                        serde_json::json!({"event": "rotate_tick", "ias": ias, "theta": theta_deg,
                                           "q": pitch_rate, "dt": dt, "pitch": pitch,
                                           "pitch_ff": pitch_law.model_pull(),
                                           "pitch_tgt": pitch_law.target_deg(),
                                           "wheels": on_ground >= 0.5,
                                           "agl": agl_ft, "vs": vs_flown}),
                    );
                }
                // Wings level on the wheels; floating, a small bank toward the
                // roll law's target heading (the runway course plus its
                // cross-track cut), as the climb banks toward the course.
                let bank = if floating {
                    climb_bank_target(err).clamp(-FLOAT_BANK_MAX_DEG, FLOAT_BANK_MAX_DEG)
                } else {
                    0.0
                };
                let ail = wings.step(phi_deg, bank, dt);
                crate::monitor::note_crew_yoke();
                writes.set(TAKEOFF_PITCH, pitch).set(TAKEOFF_ROLL, ail);
            }
            // Airborne only once clearly clear of the surface: a ground-
            // effect bounce (on_ground false at 2 ft, back down a tick
            // later — second sortie) must not stop the steering while the
            // wheels are about to touch again at speed.
            // The roll's writes, all at once: the lever, the brakes, the
            // rudder and the column in ONE round (they were four, one after
            // another, 2026-09-30). A lost channel is judged on the result.
            let done = writes.flush(&xp).await;
            if done.ok(TAXI_STEER) == Some(false) {
                reject_rolling(
                    &xp,
                    &voice,
                    "I've lost the steering. Rejecting takeoff.",
                    "steer_lost",
                    &rwy,
                    &mut steer_law,
                    p.park_brake_valve_set,
                )
                .await;
                return;
            }
            if done.ok(TAKEOFF_PITCH) == Some(false) || done.ok(TAKEOFF_ROLL) == Some(false) {
                reject_rolling(
                    &xp,
                    &voice,
                    "I've lost the elevator. Rejecting takeoff.",
                    "pitch_lost",
                    &rwy,
                    &mut steer_law,
                    p.park_brake_valve_set,
                )
                .await;
                return;
            }
            if on_ground < 0.5 && ias > p.vr_kts * 0.8 && agl_ft > 8.0 {
                airborne = true;
                let _ = xp.set_float(TAXI_STEER, 0.0).await;
                // Transponder to ALT (mode C) now we're flying — real ops squawk
                // altitude on departure. Through the shared writer, so an
                // airframe whose box is an add-on's own (the Twin Otter's KT73,
                // which rewrites the stock dataref from its knob every frame)
                // gets its KNOB turned rather than a write nothing reads. A
                // no-op on an airframe with no transponder, so no profile gate.
                // On its own task: a knob stepped by commands takes a few
                // hundred milliseconds, and this loop is about to rotate. And
                // the result is KEPT: it was discarded, and the A330 flew out
                // in standby to two "recycle transponder" calls with nothing
                // said on the flight deck (EGPH, 2026-09-20).
                squawk_altitude(p.base_url.clone(), voice.clone(), false);
                crate::blackbox::record(
                    "autotakeoff",
                    serde_json::json!({"event": "liftoff", "ias": ias}),
                );
            }
            pacer.wait(std::time::Duration::from_millis(ROLL_TICK_MS)).await;
            continue;
        }

        // Airborne: the lever goes now, as it always has, except on the
        // hand-flown climb, where it rides the column's round.
        if ap_engaged || agl_ft >= AP_ENGAGE_AGL_FT {
            writes.flush(&xp).await;
        }
        // Airborne: pitch attitude by hand until established, then the AP.
        if !positive_called && agl_ft > 50.0 {
            positive_called = true;
            alert(&voice, Priority::Advisory, "Positive rate.");
        }
        if !gear_raised
            && gear_up_due(
                agl_ft,
                ias,
                flap_cmd,
                gear_warning(),
                crate::aircraft::gear_limit_kts(),
            )
        {
            gear_raised = true;
            let _ = crate::tools::gear::set_handle(&xp, true).await;
            alert(&voice, Priority::Advisory, "Gear up.");
            crate::blackbox::record(
                "autotakeoff",
                serde_json::json!({"event": "gear_up", "agl": agl_ft, "ias": ias,
                                   "flap": flap_cmd, "warning": gear_warning().is_some()}),
            );
            crate::takeoff_config::after_liftoff(&xp).await;
        }
        // Clean up the takeoff flaps once safely climbing — after the gear,
        // the way a crew does it. On an airframe that declares manoeuvring
        // speeds this walks the schedule a detent at a time and each step
        // waits for its own speed; without one it is the old single shot.
        if !flaps_retracted && agl_ft > climb_out.map_or(FLAP_RETRACT_AGL_FT, |c| c.accel_agl_ft) {
            if p.flap_schedule.is_empty() {
                if ias > p.initial_climb_kts {
                    flaps_retracted = true;
                    let _ = crate::tools::flaps::set_handle(&xp, 0.0).await;
                    alert(&voice, Priority::Advisory, "Flaps up.");
                    move_speed_window(&xp, &mut window_owed, window_kts, "flap_step", agl_ft, ias)
                        .await;
                }
            } else if flap_retract_step(&p.flap_schedule, flap_cmd, ias).is_some() {
                // Each step waits for the flaps to ARRIVE at the last one:
                // held to the acceleration height, a jet is past both
                // detents' speeds at once, and the schedule commanded 5 to
                // 2 and 2 to 1 on consecutive ticks, half a second apart
                // (Alghero, 2026-09-27). Read only while a step is due, not
                // every tick of the roll. A failed read is no measurement:
                // no step on it.
                let deployed = xp.get_float(crate::takeoff_config::FLAP_DEPLOY).await.ok();
                let t_s = roll_start.elapsed().as_secs_f64();
                let judged = flap_pace.judged(t_s, flap_cmd, deployed);
                let step = paced_flap_step(&p.flap_schedule, flap_cmd, judged, ias);
                if step.is_none() && flap_pace.withheld_log_due(t_s) {
                    crate::blackbox::record(
                        "autotakeoff",
                        serde_json::json!({
                            "event": "flap_step_withheld", "at": flap_cmd, "ias": ias,
                            "agl_ft": agl_ft, "deployed": deployed,
                        }),
                    );
                }
                if let Some(next) = step {
                    flap_pace.stepped(flap_cmd);
                    flap_cmd = next;
                    flaps_retracted = next <= 1e-6;
                    let _ = crate::tools::flaps::set_handle(&xp, next).await;
                    let detent = crate::aircraft::live_profile()
                        .and_then(|lp| crate::aircraft::flap_detent_spoken(lp.flaps.as_ref(), next));
                    crate::blackbox::record(
                        "autotakeoff",
                        serde_json::json!({
                            "event": "flap_step", "to": next, "ias": ias, "agl_ft": agl_ft,
                            "detent": detent, "deployed": deployed,
                        }),
                    );
                    alert(
                        &voice,
                        Priority::Advisory,
                        &flap_step_line(next, detent.as_deref()),
                    );
                    move_speed_window(&xp, &mut window_owed, window_kts, "flap_step", agl_ft, ias)
                        .await;
                }
            }
        }
        // Landing and taxi lights off passing 1,000 ft, the practice: the
        // takeoff owns the aircraft to its handoff, and a 737 handing off at
        // 2,765 ft carried them into the climb list (EGKK 08R, #174).
        if !lights_off && agl_ft >= LIGHTS_OFF_AGL_FT {
            lights_off = true;
            let did = crate::tools::lights::after_climb(&xp).await;
            crate::blackbox::record(
                "autotakeoff",
                serde_json::json!({"event": "lights_climb", "off": did, "agl_ft": agl_ft.round()}),
            );
        }
        if !ap_engaged {
            if agl_ft < AP_ENGAGE_AGL_FT {
                // Still hand-flying: hold the rotation attitude while speed
                // builds, and the wings against full-power torque — banked
                // gently toward runway heading, since the rudder steering
                // ended at liftoff.
                let hdg_err = crate::angles::signed_diff_deg(rwy.course_deg, psi);
                pitch_law.on_wheels(false);
                pitch_law.climb_fpm(vs_flown);
                let pitch = pitch_law.step_climb(theta_deg, pitch_rate, dt, Some((ias, agl_ft)));
                let ail = wings.step(phi_deg, climb_bank_target(hdg_err), dt);
                crate::monitor::note_crew_yoke();
                writes.set(TAKEOFF_PITCH, pitch).set(TAKEOFF_ROLL, ail);
                let done = writes.flush(&xp).await;
                if done.ok(TAKEOFF_PITCH) == Some(false) || done.ok(TAKEOFF_ROLL) == Some(false) {
                    alert(
                        &voice,
                        Priority::Warning,
                        "I've lost the elevator. Your aircraft.",
                    );
                    crate::blackbox::record(
                        "autotakeoff",
                        serde_json::json!({"event": "pitch_lost_airborne"}),
                    );
                    return;
                }
                crate::blackbox::record(
                    "autotakeoff",
                    serde_json::json!({
                        "event": "climb_manual", "ias": ias, "agl": agl_ft,
                        // `q` is the measured pitch rate the damping runs on
                        // (a missing read now skips the tick), and `dt` the
                        // interval over which the law was sampled. The 2026-08-02 diagnosis needed both and the
                        // record carried neither: the ring was in the DAMPING
                        // path, and nothing logged what it damped on.
                        "theta": theta_deg, "pitch": pitch, "q": pitch_rate, "dt": dt,
                        "yoke": yoke_read,
                        "pitch_tgt": pitch_law.target_deg(),
                        // The airspeed trend the climb hold walks the target
                        // on, knots per second (`ClimbHold`).
                        "accel": pitch_law.accel_kts_s(),
                        // The column the ceiling guard holds, null while it
                        // holds none (#78, EGKK 08R flew 22.4 under 18).
                        "pitch_lim": pitch_law.ceiling_limit(),
                        "phi": phi_deg, "ail": ail, "hdg_err": hdg_err,
                        // The climb the engage will take (#150).
                        "vs": vs_flown,
                    }),
                );
                pacer.wait(std::time::Duration::from_millis(CLIMB_MANUAL_TICK_MS)).await;
                continue;
            }
            // Established in the climb: the AP takes it. Servos + HDG + VS,
            // and the VS dial written AFTER the mode engages — engage syncs
            // the dial to the CURRENT rate, stomping any preset (the sync
            // that made VS-mode rotation impossible in the first place).
            // The pitch channel simply expires; the servo owns the elevator.
            // Each try takes the climb flown NOW (#150).
            climb_vs = ClimbVs::default();
        }
        // With departure minimums ahead (full auto's SID), the climb-out is
        // flown at the airframe's BEST rate, holding its initial climb speed
        // by climbing harder: capped at the enroute 1,500 fpm the SF50 made
        // 750 ft a mile where MOGT3H's WI505 wanted 816 (LOWI, 2026-09-24).
        let (base_fpm, climb_out) = if crate::full_auto::departure_minimums() {
            let best = crate::procedure::best_climb_fpm().max(p.climb_fpm);
            (best, Some(ClimbOut { accel_agl_ft: f64::INFINITY, max_fpm: best }))
        } else {
            (p.climb_fpm, climb_out)
        };
        // The climb flown at the engage, carried into the law at a crew's
        // pace; below a declared acceleration height the law holds the speed
        // (`ClimbVs`).
        let (law_vs, vs) = climb_vs.step(
            ias,
            p.initial_climb_kts,
            base_fpm,
            climb_guard_vne(p.vne_kts),
            agl_ft,
            climb_out,
            vs_flown,
            dt,
            accelerating(climb_out, agl_ft, flaps_retracted),
        );
        if !engage_climb(
            &xp,
            &p.cfg.fd_commands,
            &mut engage_tries,
            vs,
            &mut last_vs_cmd,
            &mut engage_pressed_at,
        )
        .await {
            // An engage that never reads back is no autopilot: nobody flies
            // the thrust and the modes are pressed on and off. The 737 left
            // EGKK 26L at climb thrust, pressed 167 times over four minutes,
            // levelled at 5,000 ft and reached 360 kt (1792152599). The
            // recovery takes it: our column, the governor on the thrust.
            let since = engage_since.get_or_insert_with(std::time::Instant::now);
            if !ap_engaged && since.elapsed().as_secs_f64() >= ENGAGE_GIVE_UP_S {
                no_pitch_take_over(&xp, &p, &voice, "ap_wont_engage").await;
                return;
            }
            continue;
        }
        if !ap_engaged {
            ap_engaged = true;
            let trim = xp.get_float(ELEV_TRIM).await.ok();
            crate::blackbox::record(
                "autotakeoff",
                serde_json::json!({
                    "event": "ap_engaged", "ias": ias, "agl": agl_ft, "cmd_vs": vs, "trim": trim,
                    // The climb the pitch was flying and the law's own rate:
                    // `cmd_vs` starts at the first and walks to the second.
                    "vs": vs_flown, "law_vs": law_vs,
                }),
            );
            // The engage is a request: is the elevator the autopilot's
            // (#181)? Fed here, then by whoever takes the climb.
            if let Some(flown) = vs_flown {
                crate::climb_watch::arm_authority("autotakeoff", flown, vs);
            }
        } else if let Some(crate::climb_watch::Authority::NoAuthority(why)) =
            crate::climb_watch::feed_authority(vs_flown, Some(vs))
        {
            no_pitch_take_over(&xp, &p, &voice, why).await;
            return;
        }
        DETAIL.set(format!(
            "climbing, {} ft, handoff at {} ft",
            (agl_ft / 50.0).round() * 50.0,
            p.handoff_agl_ft.max(AP_ENGAGE_AGL_FT + 50.0).round()
        ));
        crate::auto_status::set_ap_targets(None, None, Some(vs));
        crate::blackbox::record(
            "autotakeoff",
            serde_json::json!({"event": "climb", "ias": ias, "agl": agl_ft, "cmd_vs": vs,
                               "law_vs": law_vs, "vs": vs_flown, "thr": last_thr_cmd,
                               "n1": n1_now, "n1_cap": n1_cap}),
        );
        if agl_ft >= p.handoff_agl_ft.max(AP_ENGAGE_AGL_FT + 50.0) {
            // At the height, dirty, and a few knots short of the clean-up:
            // keep flying the climb until the schedule above has done it.
            let floor = crate::aircraft::speed_floor_kts();
            let waited = handoff_wait.get_or_insert_with(std::time::Instant::now);
            let waited_s = waited.elapsed().as_secs_f64();
            // A caller that walks the schedule itself takes the flaps as
            // they are (#150): no wait, no one-shot.
            let flaps_left = leaves_flaps_to_caller(
                flaps_retracted,
                p.flaps_to_caller,
                !p.flap_schedule.is_empty(),
            );
            if !flaps_left
                && (handoff_waits_for_clean_up(flaps_retracted, ias, floor, waited_s)
                    || (climb_out.is_some()
                        && handoff_waits_for_schedule(
                            flaps_retracted,
                            !p.flap_schedule.is_empty(),
                            waited_s,
                        )))
            {
                tokio::time::sleep(std::time::Duration::from_millis(400)).await;
                continue;
            }
            if flaps_left {
                crate::blackbox::record(
                    "autotakeoff",
                    serde_json::json!({"event": "flaps_handoff", "ias": ias,
                                       "floor": floor, "up": false, "to_caller": true,
                                       "flap": flap_cmd}),
                );
            }
            // The takeoff accelerates on from here in the caller's hands: the
            // window leaves V2 now if no flap step has moved it.
            move_speed_window(&xp, &mut window_owed, window_kts, "handoff", agl_ft, ias).await;
            // Never hand over dirty. The schedule above waits for Vy, which a
            // trainer passes in the first few hundred feet and a jet does not
            // reach before this loop ends: a 737 lifting at V2+20 hands off at
            // 1,000 ft and 181 kt against a Vy of 210, so nothing ever fired
            // and the flaps stayed at 5 from Lyon to FL360 and all the way to
            // Heathrow's top of descent, the whole climb held against their
            // 250 kt placard (2026-09-17). Whoever takes the aircraft next
            // flies it clean, or is TOLD it is not — the floor is the
            // airframe's clean-configuration minimum, so a retraction can
            // never be one into the stall it was meant to avoid.
            if let Some(up) =
                clean_up_at_handoff(flaps_retracted, ias, floor).filter(|_| !flaps_left)
            {
                if up {
                    let _ = crate::tools::flaps::set_handle(&xp, 0.0).await;
                    alert(&voice, Priority::Advisory, "Flaps up.");
                } else {
                    alert(
                        &voice,
                        Priority::Caution,
                        "Flaps are still out, we're too slow to clean up. Watch the placard.",
                    );
                }
                crate::blackbox::record(
                    "autotakeoff",
                    serde_json::json!({"event": "flaps_handoff", "ias": ias,
                                       "floor": floor, "up": up}),
                );
            }
            // Nobody takes an aircraft with the gear still hanging: a gear
            // held for its warning speed comes up at the handoff at the latest.
            if !gear_raised {
                let _ = crate::tools::gear::set_handle(&xp, true).await;
                alert(&voice, Priority::Advisory, "Gear up.");
                crate::blackbox::record(
                    "autotakeoff",
                    serde_json::json!({"event": "gear_up", "agl": agl_ft, "ias": ias,
                                       "flap": flap_cmd, "at_handoff": true}),
                );
                crate::takeoff_config::after_liftoff(&xp).await;
            }
            // Say the height we ACTUALLY reached, not a hardcoded thousand: a
            // circuit hands off at 500 ft, and it announced "one thousand
            // feet" there — a number the pilot can see is wrong on the
            // altimeter, from the same sentence that tells them who is flying.
            let height = spoken_height_ft(agl_ft);
            // A PILOT handoff leaves a flyable aircraft, the rule autoland's
            // give-ups follow: a level the climb will capture and a speed
            // somebody holds. This loop's own overspeed guard dies with it,
            // and a jet at takeoff thrust in a 1,500 fpm VS climb was at Vmo
            // by 10,000 ft with the dial still at 100 (EGPH, 2026-09-20).
            let mut held = String::new();
            if !p.handoff_to_caller {
                let (dial, alt) =
                    tokio::join!(autoflight::read_dial(&xp, Dial::Altitude), xp.get_float(ALT_IND_FT));
                if let (Ok(dial), Ok(alt)) = (dial, alt) {
                    let level = match handoff_altitude_dial_ft(dial, alt) {
                        Some(set) => {
                            let ok = autoflight::set_dial(&xp, Dial::Altitude, set).await.is_ok();
                            crate::blackbox::record(
                                "autotakeoff",
                                serde_json::json!({"event": "handoff_alt_dial", "was": dial,
                                                   "now": set, "alt_ft": alt, "ok": ok}),
                            );
                            ok.then_some(set)
                        }
                        None => Some(dial),
                    };
                    if let Some(level) = level {
                        held.push_str(&format!(" to {}", crate::flightplan::spoken_feet(level)));
                    }
                    if let Some(kts) = p.handoff_speed_kts {
                        let kts = crate::tools::speedhold::airspace_limited(alt, kts);
                        crate::tools::speedhold::arm_flight(p.base_url.clone(), voice.clone(), kts);
                        crate::blackbox::record(
                            "autotakeoff",
                            serde_json::json!({"event": "handoff_speed", "target": kts}),
                        );
                        held.push_str(&format!(", and I'm holding {kts:.0} knots"));
                    }
                }
            }
            alert(
                &voice,
                Priority::Caution,
                &format!(
                    "{height}, {} knots. {}{held}.",
                    ias.round() as i64,
                    if p.handoff_to_caller {
                        // The circuit, full auto or a procedure is taking it.
                        // Saying "your aircraft" here is a lie about the one
                        // thing that must never be wrong.
                        "The autopilot is holding runway heading and the climb"
                    } else {
                        "Your aircraft. The autopilot is holding runway heading and the climb"
                    }
                ),
            );
            crate::blackbox::record(
                "autotakeoff",
                serde_json::json!({"event": "handoff", "agl": agl_ft, "ias": ias,
                                   "to_caller": p.handoff_to_caller, "cmd_vs": vs,
                                   "flaps_left": flaps_left.then_some(flap_cmd)}),
            );
            publish_climb_handover(Some(ClimbHandover {
                vs_cmd: Some(vs),
                flaps_left: flaps_left.then_some(flap_cmd),
            }));
            RUN.record(sim::run::Outcome::Completed);
            return;
        }
        tokio::time::sleep(std::time::Duration::from_millis(400)).await;
    }
}

/// A height called the way a pilot calls it, rounded to the nearest hundred:
/// "five hundred feet", "one thousand feet", "one thousand five hundred feet".
/// Spelled out rather than left as digits because this goes to TTS, where a
/// bare "1500" is read as a quantity, not an altitude.
pub fn spoken_height_ft(agl_ft: f64) -> String {
    const ONES: [&str; 10] = [
        "zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine",
    ];
    let h = (agl_ft.max(0.0) / 100.0).round() as i64 * 100;
    let thousands = h / 1000;
    let hundreds = (h % 1000) / 100;
    let mut out = String::new();
    if thousands > 0 {
        let t = ONES.get(thousands as usize).copied().unwrap_or("");
        if t.is_empty() {
            // Above nine thousand this is not a takeoff handoff any more;
            // fall back to the number rather than inventing words for it.
            out.push_str(&thousands.to_string());
        } else {
            out.push_str(t);
        }
        out.push_str(" thousand");
    }
    if hundreds > 0 {
        if thousands > 0 {
            out.push(' ');
        }
        out.push_str(ONES[hundreds as usize]);
        out.push_str(" hundred");
    }
    if out.is_empty() {
        out.push_str("zero");
    }
    out.push_str(" feet");
    out
}
