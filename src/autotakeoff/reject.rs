//! The rejected takeoff: flown to a stop, parked and handed back, and its call.

use super::*;

/// Under this a rejected roll has STOPPED and the parking brake goes on; the
/// hand-back's own stopped line (`full_auto::hand_back_hold`).
pub(super) const REJECT_STOPPED_KTS: f64 = sim::ground::STOPPED_KTS;
/// The reject's park: a hand-back's own gates (a pilot on the toe brakes
/// keeps a fast stop).
pub(super) const REJECT_HOLD: crate::full_auto::HoldFor = crate::full_auto::HoldFor::by("autotakeoff");
/// Below this the stop lets go of the steering: a walk on the centreline.
pub(super) const REJECT_STEER_MIN_KTS: f64 = 3.0;
/// The longest the stop is flown, in live seconds (a pause slides it).
pub(super) const REJECT_STOP_MAX_S: f64 = 90.0;
/// Maximum braking is renewed this often, inside the held command's second.
pub(super) const REJECT_BRAKE_EVERY_S: f64 = 0.8;
/// The autothrottle is looked at this often through the stop.
pub(super) const REJECT_AT_EVERY_S: f64 = 1.0;
/// The stop's tick.
pub(super) const REJECT_TICK_MS: u64 = 200;

/// What the stop did, for the blackbox and the hand-back line.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct RejectStop {
    /// Under [`REJECT_STOPPED_KTS`] before [`REJECT_STOP_MAX_S`].
    pub stopped: bool,
    /// Groundspeed at the first and last reads, and the most seen, knots.
    pub from_kts: Option<f64>,
    pub end_kts: Option<f64>,
    pub max_kts: Option<f64>,
    /// Live seconds flown.
    pub secs: f64,
    pub brake_pulses: u32,
    /// An ENGAGED autothrottle found and taken out: whether it read back out.
    pub at_off: Option<bool>,
}

/// A reject is FLOWN to a full stop: the throttles idle every tick (the
/// levers are ours, an engaged autothrottle is taken out and read back),
/// maximum braking renewed inside its own expiry, and the centreline held
/// with the roll's own law, until the aircraft is under
/// [`REJECT_STOPPED_KTS`]. The reject used to brake for one second and say
/// "your aircraft" at 127 kt (LIBP 04, 2026-09-26); then it stopped to 20 kt
/// on the EGKK rejects; and the pilot's own "reject the takeoff" never flew
/// the stop at all: LEMG 13 (sortie 1790627884-full_auto, 2026-09-28) said
/// "Throttle is idle, brakes on. Your aircraft." at 63 kt, the engines
/// spooling down from takeoff thrust pushed it to 73 kt with one second of
/// brakes, and it rolled 70 to 52 kt for two minutes with nobody braking.
pub(crate) async fn stop_on_centreline(
    xp: &dyn XPlane,
    rwy: &Runway,
    law: &mut RollSteer,
) -> RejectStop {
    let mut out = RejectStop::default();
    let thr = throttle_dataref();
    let mut live_s = 0.0_f64;
    let mut last = tokio::time::Instant::now();
    let mut last_brake: Option<f64> = None;
    let mut last_at: Option<f64> = None;
    let mut prev_xtk: Option<f64> = None;
    let mut rate_filter = XtkRate::default();
    while live_s < REJECT_STOP_MAX_S {
        let now = tokio::time::Instant::now();
        let dt = (now - last).as_secs_f64().clamp(0.0, 1.0);
        last = now;
        // Pause or replay: nothing is written and the clock slides.
        if crate::xplane::sim_frozen(xp).await {
            tokio::time::sleep(std::time::Duration::from_millis(REJECT_TICK_MS)).await;
            continue;
        }
        live_s += dt;
        // The levers are the reject's: idle, every tick.
        let _ = xp.set_float(&thr, 0.0).await;
        if last_at.is_none_or(|t| live_s - t >= REJECT_AT_EVERY_S) {
            last_at = Some(live_s);
            if let Some(ok) = crate::takeoff_config::autothrottle_off(xp).await {
                out.at_off = Some(ok);
            }
        }
        // One frame's position (#157), as the roll reads it.
        let (pos, yaw_r) = tokio::join!(sim::position::read(xp), xp.get_float(YAW_RATE));
        let (lat, lon) = sim::position::lat_lon(&pos);
        let psi = sim::position::field(&pos, |p| p.psi_true_deg);
        let gs = sim::position::field(&pos, |p| p.gs_ms);
        // A failed read is no measurement: the brakes stay on, nothing else.
        let meas = crate::control::measurements("autotakeoff", [lat, lon, psi, gs]);
        let gs_kts = meas.map(|[_, _, _, g]| g.abs() * MS_TO_KTS);
        if let Some(k) = gs_kts {
            out.from_kts.get_or_insert(k);
            out.end_kts = Some(k);
            out.max_kts = Some(out.max_kts.map_or(k, |m: f64| m.max(k)));
            if k < REJECT_STOPPED_KTS {
                out.stopped = true;
                break;
            }
        }
        if last_brake.is_none_or(|t| live_s - t >= REJECT_BRAKE_EVERY_S) {
            // Noted BEFORE the command too: the plugin latches its brake
            // yield at the command's begin, before the Web API answers, and a
            // monitor tick in between would read our brake as a hand.
            crate::tools::note_brake_cmd();
            if xp.run_command_for(BRAKES_MAX, crate::tools::BRAKE_HOLD_SECS).await.is_ok() {
                crate::tools::note_brake_cmd();
                out.brake_pulses += 1;
            }
            last_brake = Some(live_s);
        }
        if let (Some([lat, lon, psi, _]), Some(gs_kts)) = (meas, gs_kts) {
            let dt = dt.max(0.05);
            let xtk_m = cross_track_nm(lat, lon, rwy) * M_PER_NM;
            let rate = rate_filter.update(prev_xtk.map_or(0.0, |p| (xtk_m - p) / dt), dt);
            prev_xtk = Some(xtk_m);
            let target = roll_target_heading(rwy.course_deg, xtk_m, rate, gs_kts);
            let err = crate::angles::signed_diff_deg(target, psi);
            let yaw_dps = yaw_r.ok().filter(|v| v.is_finite());
            let steer = if gs_kts >= REJECT_STEER_MIN_KTS {
                law.step_with_yaw(err, dt, gs_kts, 0.0, yaw_dps)
            } else {
                0.0
            };
            let _ = xp.set_float(TAXI_STEER, steer).await;
            crate::blackbox::record(
                "autotakeoff",
                serde_json::json!({"event": "reject_rollout", "gs": gs_kts, "xtk_m": xtk_m,
                                   "err": err, "steer": steer, "r": yaw_dps,
                                   "pos": sim::position::source_word(&pos)}),
            );
        }
        tokio::time::sleep(std::time::Duration::from_millis(REJECT_TICK_MS)).await;
    }
    let _ = xp.set_float(TAXI_STEER, 0.0).await;
    out.secs = live_s;
    out
}

/// The line the reject ends on, from what the stop and the hand-back READ
/// BACK: "your aircraft" only over an aircraft that is stopped and held, or
/// said with what is not. Pure.
pub fn reject_end_line(stop: &RejectStop, held: Option<bool>) -> String {
    match (held, stop.stopped) {
        (Some(true), _) => "Stopped. Parking brake set. Your aircraft.".into(),
        (Some(false), true) => {
            "Stopped. The parking brake didn't set, hold the brakes. Your aircraft.".into()
        }
        (Some(false), false) => "The parking brake didn't set, hold the brakes. Your aircraft."
            .into(),
        (None, true) => "Stopped. Your aircraft.".into(),
        (None, false) => "I couldn't stop us. Your aircraft, brakes.".into(),
    }
}

/// A reject from ANY speed on the ground: the reject, the stop flown to a
/// full stop on the centreline, the parking brake set and read back
/// (`full_auto::hand_back_hold`, after the held brake lets go of the ratio),
/// and only then the hand-back. The run's outcome is the reject's
/// (`Failed(reason)`), so a caller ends as `takeoff_not_completed`.
pub(super) async fn reject_rolling(
    xp: &XPlaneClient,
    voice: &Option<Voice>,
    line: &str,
    reason: &str,
    rwy: &Runway,
    law: &mut RollSteer,
    valve_set: Option<f64>,
) {
    // Unscoped: a reject is how the run ENDS, and a cancelled scope must
    // never refuse the throttles and brakes that stop it.
    let xp = &xp.unscoped();
    reject(xp, voice, line, reason).await;
    let stop = stop_on_centreline(xp, rwy, law).await;
    let held = crate::full_auto::hand_back_hold(xp, valve_set, REJECT_HOLD).await;
    let end = reject_end_line(&stop, held);
    alert(voice, Priority::Warning, &end);
    crate::blackbox::record(
        "autotakeoff",
        serde_json::json!({"event": "reject_stopped", "reason": reason,
                           "stopped": stop.stopped, "from_kts": stop.from_kts,
                           "max_kts": stop.max_kts, "end_kts": stop.end_kts,
                           "secs": (stop.secs * 10.0).round() / 10.0,
                           "brake_pulses": stop.brake_pulses, "at_off": stop.at_off,
                           "park_held": held}),
    );
}

/// The reject's hands in its first second: throttles idle, an engaged
/// autothrottle out (read back, `Some` when there was one), maximum braking.
pub(crate) async fn reject_hands(xp: &dyn XPlane) -> Option<bool> {
    let _ = xp.set_float(&throttle_dataref(), 0.0).await;
    let at_off = crate::takeoff_config::autothrottle_off(xp).await;
    // Noted before as well as after: see `stop_on_centreline`.
    crate::tools::note_brake_cmd();
    if xp
        .run_command_for(BRAKES_MAX, crate::tools::BRAKE_HOLD_SECS)
        .await
        .is_ok()
    {
        crate::tools::note_brake_cmd();
    }
    at_off
}

/// The reject's first second: the run's outcome, throttles idle, the
/// autothrottle out, maximum braking, the lever up on a down-detent
/// airframe, the line, and the tower told.
pub(super) async fn reject(xp: &XPlaneClient, voice: &Option<Voice>, line: &str, reason: &str) {
    // The throttle and brakes of a reject are never refused by the scope
    // the abort itself just cancelled.
    let xp = &xp.unscoped();
    RUN.record(sim::run::Outcome::Failed(reason.into()));
    let at_off = reject_hands(xp).await;
    crate::takeoff_config::on_reject(xp).await;
    alert(voice, Priority::Warning, line);
    // The tower is told. A reject is a radio event: the clearance it ends is
    // the TOWER's, and left standing it outlived the stop (EGKK 2026-09-27:
    // the resume heard a stale "cleared for takeoff" and was asked to read
    // it back instead of getting its taxi). Only under a tower's clearance
    // onto this runway; an uncontrolled field has nobody to tell.
    let call = reject_call_due(crate::party_line::clearance_state());
    if call {
        let callsign = crate::crew_radio::crew_callsign(xp).await;
        crate::crew_radio::transmit(crate::crew_radio::radio_call(&callsign, REJECT_CALL));
    }
    crate::blackbox::record(
        "autotakeoff",
        serde_json::json!({"event": "reject", "reason": reason, "radio_call": call,
                           "at_off": at_off}),
    );
}

/// The crew's reject report on the tower frequency. ICAO's word is
/// "stopping"; the crew says what it is doing, which no mishear turns into a
/// request (`tower::parse_intent` reads both as `RejectingTakeoff`).
pub const REJECT_CALL: &str = "rejecting takeoff";

/// Whether a reject is reported: only when the tower has us on its runway
/// (cleared for takeoff, or lined up without a clearance).
pub fn reject_call_due(state: Option<crate::tower::State>) -> bool {
    matches!(
        state,
        Some(crate::tower::State::TakeoffCleared | crate::tower::State::LinedUp)
    )
}

/// The MCP window off V2 when it is `owed`, once, blackboxed `speed_window`
/// with where it went (`at`: the first flap step or the handoff).
pub(super) async fn move_speed_window(
    xp: &XPlaneClient,
    owed: &mut bool,
    kts: f64,
    at: &str,
    agl_ft: f64,
    ias: f64,
) {
    if !std::mem::take(owed) {
        return;
    }
    let ok = speed_window_after_acceleration(xp, kts).await;
    crate::blackbox::record(
        "autotakeoff",
        serde_json::json!({"event": "speed_window", "to": kts, "ok": ok, "at": at,
                           "agl": agl_ft, "ias": ias}),
    );
}
