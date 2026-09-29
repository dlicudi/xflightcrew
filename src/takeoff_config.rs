//! The configuration a crew sets BEFORE the aircraft moves, not during the roll.
//!
//! EGPF, 2026-09-21, the A330 round the circuit under full auto: the flap
//! handle was written after "Brakes released. Rolling" (fine on a 172, whose
//! flaps travel in three seconds; CONF 2 was still running out at sixty
//! knots), the altitude dial sat at the 100 ft the aircraft loads with and the
//! monitor called "through the selected altitude, 100 set" twice on the climb,
//! and the ground spoilers and the autobrake were never touched, so the
//! Monitor checklist named them on every flight. Full auto flies the checklist
//! items it owns: these are the takeoff's and the landing's.
//!
//! One page, three callers: full auto after the engine start (the after-start
//! flow), the auto takeoff before the brakes come off (every path to a roll
//! goes through it, and there it WAITS for the flaps), and autoland at the
//! gear. Each item is read first and left alone when it is already in state,
//! read back after the write, and named when it did not take.

use crate::aircraft::TakeoffConfigProfile;
use crate::xplane::XPlane;
use std::sync::Mutex;

const FLAP_HANDLE: &str = crate::datarefs::FLAP_RATIO;
/// Where the flaps ARE, following the handle at the airframe's own rate.
/// The climb-out's retraction reads it too: each step waits for the last.
pub(crate) const FLAP_DEPLOY: &str = "sim/cockpit2/controls/flap_handle_deploy_ratio";
const SPEEDBRAKE: &str = TakeoffConfigProfile::SPEEDBRAKE;
/// 0 RTO, 1 off, 2..5 the landing levels.
const AUTOBRAKE: &str = TakeoffConfigProfile::AUTOBRAKE;
const ALT_DIAL: &str = "sim/cockpit2/autopilot/altitude_dial_ft";
const GROUNDSPEED_MS: &str = "sim/flightmodel/position/groundspeed";
/// The stock autoflight names: one list, the profile probe's
/// ([`TakeoffConfigProfile::references`]).
const FD_MODE: &str = TakeoffConfigProfile::FD_MODE;
const CMD_FD_ON: &str = TakeoffConfigProfile::CMD_FD_ON;
const AT_MODE: &str = TakeoffConfigProfile::AT_MODE;
const CMD_AT_ARM: &str = TakeoffConfigProfile::CMD_AT_ARM;
const CMD_AT_OFF: &str = TakeoffConfigProfile::CMD_AT_OFF;
const SPEED_WINDOW: &str = TakeoffConfigProfile::SPEED_WINDOW;
/// A speed window this close to V2 already shows it.
const V2_TOLERANCE_KTS: f64 = 1.0;

const SPEEDBRAKE_ARMED: f64 = -0.5;
const AUTOBRAKE_RTO: i64 = 0;
/// Below this a reject is a stop from a walk and the lever stays down.
const REJECT_SPOILER_KTS: f64 = 40.0;
/// The longest the brakes wait on the flaps. An A330 runs CONF 0 to 2 in
/// about twenty seconds; a flap that has not arrived by now is said, and the
/// roll goes (the takeoff's own flap law still holds the handle).
const FLAP_WAIT_SECS: f64 = 40.0;
/// How close the flaps must be to the handle to count as there.
pub(crate) const FLAP_ARRIVED: f64 = 0.03;

/// Whether flaps reading `deployed` have run to the `commanded` handle
/// ratio. Pure; the one test behind the takeoff's wait and the climb-out's
/// pacing of its retraction steps.
pub fn flaps_arrived(deployed: f64, commanded: f64) -> bool {
    deployed.is_finite() && (deployed - commanded).abs() <= FLAP_ARRIVED
}

/// A dial this far above the field is the pilot's own and is left alone.
const DIAL_CLEAR_FT: f64 = 500.0;

/// What the airframe's profile declares (`takeoff_config:`). Silent profiles
/// get flaps, trim and the dial only: a 172 has no spoilers to arm.
static AIRFRAME: Mutex<Option<crate::aircraft::TakeoffConfigProfile>> = Mutex::new(None);

/// Republished on every swap, including to `None`
/// ([`crate::flight_state::publish_airframe`]).
pub fn publish(p: Option<crate::aircraft::TakeoffConfigProfile>) {
    *AIRFRAME.lock().unwrap_or_else(|e| e.into_inner()) = p;
}

/// Does this airframe have a speedbrake lever the crew may use in flight?
/// The same declaration that arms the ground spoilers: one lever.
pub fn has_speedbrake() -> bool {
    airframe().is_some_and(|af| af.arm_spoilers)
}

fn airframe() -> Option<crate::aircraft::TakeoffConfigProfile> {
    AIRFRAME.lock().unwrap_or_else(|e| e.into_inner()).clone()
}

/// Thrust reduction, declared: pull the levers back to climb thrust above
/// this height.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThrustReduction {
    pub agl_ft: f64,
    pub lever: f64,
}

/// The live airframe's thrust reduction, when it declares both numbers (and
/// sane ones: a lever inside (0, 1), a height above the runway).
pub fn thrust_reduction() -> Option<ThrustReduction> {
    let af = airframe()?;
    let agl_ft = af
        .thrust_reduction_agl_ft
        .filter(|h| h.is_finite() && *h > 0.0)?;
    let lever = af
        .climb_thrust_ratio
        .filter(|r| r.is_finite() && *r > 0.0 && *r < 1.0)?;
    Some(ThrustReduction { agl_ft, lever })
}

/// What was set, and what would not go.
#[derive(Debug, Default, PartialEq)]
pub struct Outcome {
    pub set: Vec<String>,
    pub not_taken: Vec<String>,
    /// An item could not be judged yet (a speedbrake in use, a failed read):
    /// ask again next tick.
    pub deferred: bool,
}

/// The altitude to select before the roll. The caller's level (the circuit's
/// pattern altitude, the plan's cruise) to the nearest hundred; with none, a
/// dial the climb could never capture is set 3,000 ft above the field,
/// rounded up to the thousand, and a dial the pilot set above that is the
/// pilot's. `None` = leave the dial alone.
pub fn initial_altitude_ft(dial_ft: f64, field_elev_ft: f64, level_ft: Option<f64>) -> Option<f64> {
    match level_ft {
        Some(level) => {
            let want = (level / 100.0).round() * 100.0;
            ((dial_ft - want).abs() > 50.0).then_some(want)
        }
        None if dial_ft >= field_elev_ft + DIAL_CLEAR_FT => None,
        None => Some(((field_elev_ft + 3000.0) / 1000.0).ceil() * 1000.0),
    }
}

/// Set the takeoff configuration. `flap_ratio` is the profile's takeoff
/// handle position (`None` leaves the flaps to the pilot), `level_ft` the
/// altitude the caller will level at, `wait_flaps` holds until the flaps have
/// ARRIVED (the pre-roll call; the after-start call has the whole taxi).
pub async fn set_for_takeoff(
    xp: &dyn XPlane,
    flap_ratio: Option<f64>,
    level_ft: Option<f64>,
    wait_flaps: bool,
) -> Outcome {
    let mut out = Outcome::default();
    let af = airframe().unwrap_or_default();

    // Thrust against the parking brake is idled first: the levers are the
    // takeoff's to open, after the brakes (#181, the 737 sat at thrust 1.0
    // against its brake for 67 s, t 1790649044 to 1790649111).
    match levers_to_idle(xp).await {
        Some(true) => out.set.push("thrust idle".into()),
        Some(false) => out.not_taken.push("the thrust levers".into()),
        None => {}
    }

    if let Some(want) = flap_ratio {
        match xp.get_float(FLAP_HANDLE).await {
            Ok(at) if (at - want).abs() > 0.01 => {
                let ok = xp.set_float(FLAP_HANDLE, want).await.is_ok()
                    && xp
                        .get_float(FLAP_HANDLE)
                        .await
                        .is_ok_and(|v| (v - want).abs() <= 0.01);
                if ok {
                    out.set.push("flaps".into());
                } else {
                    out.not_taken.push("the flaps".into());
                }
            }
            Ok(_) => {}
            Err(_) => out.not_taken.push("the flaps".into()),
        }
        if wait_flaps && !flaps_arrive(xp, want).await {
            out.not_taken.push("the flaps, still travelling".into());
        }
    }

    if crate::tools::controls::trim_for_takeoff(xp).await.is_some() {
        out.set.push("trim".into());
    }

    if af.takeoff_speedbrake_down {
        match speedbrake_down(xp, false).await {
            Some(true) => out.set.push("speedbrake down".into()),
            Some(false) => out.not_taken.push("the speedbrake lever".into()),
            None => {}
        }
    } else if af.arm_spoilers {
        match xp.get_float(SPEEDBRAKE).await {
            Ok(at) if at > -0.25 => {
                let ok = xp.set_float(SPEEDBRAKE, SPEEDBRAKE_ARMED).await.is_ok()
                    && xp.get_float(SPEEDBRAKE).await.is_ok_and(|v| v < -0.25);
                if ok {
                    out.set.push("spoilers armed".into());
                } else {
                    out.not_taken.push("the ground spoilers".into());
                }
            }
            Ok(_) => {}
            Err(_) => out.not_taken.push("the ground spoilers".into()),
        }
    }

    if af.autobrake_rto {
        set_autobrake(xp, AUTOBRAKE_RTO, "autobrake max", &mut out).await;
    }

    // The autoflight a crew sets before the roll (EGPH 2026-09-21: the MCP
    // speed window at 125, no flight directors, no autothrottle arm).
    if af.flight_directors {
        flight_directors_on(xp, &mut out).await;
    }
    if af.autothrottle_armed {
        autothrottle_armed(xp, &mut out).await;
    }
    if let Some(v2) = af.v2_kts.filter(|v| v.is_finite() && *v > 0.0) {
        speed_window_v2(xp, v2, &mut out).await;
    }

    let elev_ft = xp
        .get_float(crate::datarefs::ELEVATION_M)
        .await
        .ok()
        .map(|m| m * 3.280_84);
    if let (Ok(dial), Some(elev)) = (xp.get_float(ALT_DIAL).await, elev_ft) {
        if let Some(want) = initial_altitude_ft(dial, elev, level_ft) {
            let ok = xp.set_float(ALT_DIAL, want).await.is_ok()
                && xp
                    .get_float(ALT_DIAL)
                    .await
                    .is_ok_and(|v| (v - want).abs() <= 50.0);
            if ok {
                out.set
                    .push(format!("{} selected", crate::flightplan::spoken_feet(want)));
            } else {
                out.not_taken.push("the altitude selector".into());
            }
        }
    }

    crate::blackbox::record(
        "autotakeoff",
        serde_json::json!({"event": "takeoff_config", "set": out.set,
                           "not_taken": out.not_taken, "waited_flaps": wait_flaps}),
    );
    out
}

/// Levers past this with the parking brake set are thrust against it.
const LEVERS_OPEN: f64 = 0.05;
/// The parking brake reads set from here.
const PARK_SET: f64 = 0.5;

/// Thrust against the parking brake on the ground: the levers are open and
/// the brake is set. A failed read is no measurement (`false`). Pure.
pub fn thrust_against_brake(
    levers: Option<f64>,
    park: Option<f64>,
    on_ground: Option<f64>,
) -> bool {
    let ok = |v: Option<f64>| v.filter(|x| x.is_finite());
    ok(levers).is_some_and(|l| l > LEVERS_OPEN)
        && ok(park).is_some_and(|p| p >= PARK_SET)
        && ok(on_ground).is_some_and(|g| g > 0.5)
}

/// The levers to idle when they hold thrust against the parking brake,
/// read back. `None` = nothing to do (or nothing read).
async fn levers_to_idle(xp: &dyn XPlane) -> Option<bool> {
    let lever = crate::aircraft::throttle_dataref();
    let (levers, park, ground) = tokio::join!(
        xp.get_float(&lever),
        xp.get_float(crate::tools::PARK_BRAKE),
        xp.get_float(crate::datarefs::ON_GROUND_ANY)
    );
    let was = levers.ok();
    if !thrust_against_brake(was, park.ok(), ground.ok()) {
        return None;
    }
    let took = xp.set_float(&lever, 0.0).await.is_ok()
        && xp.get_float(&lever).await.is_ok_and(|v| v <= LEVERS_OPEN);
    crate::blackbox::record(
        "autotakeoff",
        serde_json::json!({"event": "thrust_idled", "was": was, "took": took}),
    );
    Some(took)
}

/// The lever to the down detent. `None` when it is already there,
/// `Some(took)` when it was moved (a failed read is a lever that did not
/// take). `armed_only` leaves a lever that is OUT alone: in the air that one
/// is in somebody's hand.
async fn speedbrake_down(xp: &dyn XPlane, armed_only: bool) -> Option<bool> {
    match xp.get_float(SPEEDBRAKE).await {
        Ok(at) if at < -0.25 || (!armed_only && at > 0.05) => Some(
            xp.set_float(SPEEDBRAKE, 0.0).await.is_ok()
                && xp
                    .get_float(SPEEDBRAKE)
                    .await
                    .is_ok_and(|v| v.abs() <= 0.05),
        ),
        Ok(_) => None,
        Err(_) => Some(false),
    }
}

/// After the gear: ground spoilers armed for the roll are disarmed for the
/// flight (EGPH 2026-09-21: the 737 climbed through 7,000 ft with SPEED BRAKE
/// ARMED lit). The landing arms them again at the gear.
pub async fn after_liftoff(xp: &dyn XPlane) {
    if !airframe().is_some_and(|af| af.arm_spoilers) {
        return;
    }
    if let Some(ok) = speedbrake_down(xp, true).await {
        crate::blackbox::record(
            "autotakeoff",
            serde_json::json!({"event": "spoilers_disarmed", "ok": ok}),
        );
    }
}

/// A rejected takeoff on a down-detent airframe: nothing is armed to deploy
/// the ground spoilers, so the crew raises the lever, the way the real one
/// rises with the reversers. Rolling only; an armed airframe's sim does it.
pub async fn on_reject(xp: &dyn XPlane) {
    if !airframe().is_some_and(|af| af.takeoff_speedbrake_down) {
        return;
    }
    let rolling = xp
        .get_float(GROUNDSPEED_MS)
        .await
        .is_ok_and(|ms| ms.abs() * 1.943_84 > REJECT_SPOILER_KTS);
    if rolling {
        let ok = xp.set_float(SPEEDBRAKE, 1.0).await.is_ok();
        crate::blackbox::record(
            "autotakeoff",
            serde_json::json!({"event": "reject_speedbrake", "ok": ok}),
        );
    }
}

/// The landing's half, at the gear: ground spoilers armed and the landing
/// autobrake. Nothing on an airframe that declares neither.
pub async fn set_for_landing(xp: &dyn XPlane) -> Outcome {
    let mut out = Outcome::default();
    let Some(af) = airframe() else {
        return out;
    };
    if af.arm_spoilers {
        match xp.get_float(SPEEDBRAKE).await {
            // Only from stowed: a speedbrake the descent is USING is not ours
            // to pull in.
            Ok(at) if at >= 0.05 => out.deferred = true,
            Ok(at) if at > -0.25 => {
                let ok = xp.set_float(SPEEDBRAKE, SPEEDBRAKE_ARMED).await.is_ok()
                    && xp.get_float(SPEEDBRAKE).await.is_ok_and(|v| v < -0.25);
                if ok {
                    out.set.push("spoilers armed".into());
                } else {
                    out.not_taken.push("the ground spoilers".into());
                }
            }
            Ok(_) => {}
            Err(_) => out.deferred = true,
        }
    }
    if let Some(level) = af.landing_autobrake_level {
        set_autobrake(xp, level, "autobrake set", &mut out).await;
    }
    if !(out.set.is_empty() && out.not_taken.is_empty()) {
        crate::blackbox::record(
            "autoland",
            serde_json::json!({"event": "landing_config", "set": out.set,
                               "not_taken": out.not_taken}),
        );
    }
    out
}

/// Flight directors on: read, `fdir_on` only when off, read back.
async fn flight_directors_on(xp: &dyn XPlane, out: &mut Outcome) {
    match xp.get_float(FD_MODE).await {
        Ok(at) if at < 0.5 => {
            let ok = xp.run_command(CMD_FD_ON).await.is_ok()
                && xp.get_float(FD_MODE).await.is_ok_and(|v| v >= 0.5);
            if ok {
                out.set.push("flight directors on".into());
            } else {
                out.not_taken.push("the flight directors".into());
            }
        }
        Ok(_) => {}
        Err(_) => out.not_taken.push("the flight directors".into()),
    }
}

/// What the autothrottle arm owes: `None` in state (armed, no mode),
/// `Some(command)` to fire. An ENGAGED mode is taken out to armed, since the
/// takeoff owns the thrust levers (one writer per dataref); hard off is
/// armed. Pure.
pub fn autothrottle_step(mode: f64) -> Option<&'static str> {
    if !mode.is_finite() {
        return None;
    }
    match mode.round() as i64 {
        0 => None,
        m if m < 0 => Some(CMD_AT_ARM),
        _ => Some(CMD_AT_OFF),
    }
}

/// An autothrottle mode ENGAGED (`autothrottle_enabled` above zero): it
/// moves the levers. Armed (0) and hard off (-1) do not. Pure.
pub fn autothrottle_engaged(mode: f64) -> bool {
    mode.is_finite() && mode.round() as i64 > 0
}

/// A reject owns the levers: an ENGAGED autothrottle is taken out and read
/// back. `None` when none was engaged or it could not be read (nothing to
/// take out, nothing claimed); `Some(out)` when it was, with the read-back.
pub async fn autothrottle_off(xp: &dyn XPlane) -> Option<bool> {
    let mode = xp.get_float(AT_MODE).await.ok()?;
    if !autothrottle_engaged(mode) {
        return None;
    }
    let _ = xp.run_command(CMD_AT_OFF).await;
    let out = xp
        .get_float(AT_MODE)
        .await
        .is_ok_and(|v| v.is_finite() && !autothrottle_engaged(v));
    crate::blackbox::record(
        "autotakeoff",
        serde_json::json!({"event": "reject_autothrottle_off", "was": mode, "out": out}),
    );
    Some(out)
}

async fn autothrottle_armed(xp: &dyn XPlane, out: &mut Outcome) {
    match xp.get_float(AT_MODE).await {
        Ok(at) => {
            if let Some(cmd) = autothrottle_step(at) {
                let ok = xp.run_command(cmd).await.is_ok()
                    && xp
                        .get_float(AT_MODE)
                        .await
                        .is_ok_and(|v| v.is_finite() && v.round() as i64 == 0);
                if ok {
                    out.set.push("autothrottle armed".into());
                } else {
                    out.not_taken.push("the autothrottle arm".into());
                }
            }
        }
        Err(_) => out.not_taken.push("the autothrottle arm".into()),
    }
}

/// V2 in the MCP speed window: read, written only when it shows something
/// else, read back against V2.
async fn speed_window_v2(xp: &dyn XPlane, v2: f64, out: &mut Outcome) {
    match xp.get_float(SPEED_WINDOW).await {
        Ok(at) if (at - v2).abs() > V2_TOLERANCE_KTS => {
            let ok = xp.set_float(SPEED_WINDOW, v2).await.is_ok()
                && xp
                    .get_float(SPEED_WINDOW)
                    .await
                    .is_ok_and(|v| (v - v2).abs() <= V2_TOLERANCE_KTS);
            if ok {
                out.set.push(format!("V2 {v2:.0} set"));
            } else {
                out.not_taken.push("the speed window".into());
            }
        }
        Ok(_) => {}
        Err(_) => out.not_taken.push("the speed window".into()),
    }
}

async fn set_autobrake(xp: &dyn XPlane, level: i64, label: &str, out: &mut Outcome) {
    match xp.get_float(AUTOBRAKE).await {
        Ok(at) if at.round() as i64 != level => {
            let ok = xp.set_int(AUTOBRAKE, level).await.is_ok()
                && xp
                    .get_float(AUTOBRAKE)
                    .await
                    .is_ok_and(|v| v.round() as i64 == level);
            if ok {
                out.set.push(label.into());
            } else {
                out.not_taken.push("the autobrake".into());
            }
        }
        Ok(_) => {}
        Err(_) => out.not_taken.push("the autobrake".into()),
    }
}

/// Hold until the flaps have run to `want`. A failed read is no measurement
/// and keeps waiting; the clock ends it.
async fn flaps_arrive(xp: &dyn XPlane, want: f64) -> bool {
    let deadline = tokio::time::Instant::now() + std::time::Duration::from_secs_f64(FLAP_WAIT_SECS);
    loop {
        if xp
            .get_float(FLAP_DEPLOY)
            .await
            .is_ok_and(|v| flaps_arrived(v, want))
        {
            return true;
        }
        if tokio::time::Instant::now() >= deadline {
            return false;
        }
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
    }
}

/// The BEFORE TAKEOFF list, audited as a Monitor flow once the roll has
/// begun: the brakes are off and the transponder at ALT by then, the takeoff's
/// last two items, so the airframe's own list (XChecklist) cross-checks what
/// the takeoff set, read-only, ONE line at its end (`flow::summary_line`).
/// EGPH 2026-09-21: the phase tracker's flight begins at the roll, so no list
/// ever audited the takeoff configuration and an armed speedbrake reached
/// 7,000 ft. Nothing without a crew voice, or a file without the section.
/// Blackbox `autotakeoff` `before_takeoff_audit`.
pub fn before_takeoff_audit(voice: Option<crate::voice::Voice>) {
    let Some(voice) = voice else {
        return;
    };
    let section = crate::flow::find_section("before takeoff");
    let started = section.is_some_and(|idx| {
        crate::flow::start(
            idx,
            crate::flow::Flavour::Monitor,
            (*crate::settings::live()).clone(),
            voice,
        )
    });
    crate::blackbox::record(
        "autotakeoff",
        serde_json::json!({"event": "before_takeoff_audit",
                           "section": section.is_some(), "started": started}),
    );
}

/// The one line for what was done: a milestone, not the switches. `None` when
/// everything was already in state (the after-start call set it; the pre-roll
/// call has nothing to add).
pub fn line(what: &str, o: &Outcome) -> Option<String> {
    let mut s = String::new();
    if !o.set.is_empty() {
        s = format!("{what}: {}.", o.set.join(", "));
    }
    if !o.not_taken.is_empty() {
        if !s.is_empty() {
            s.push(' ');
        }
        s.push_str(&format!("Check {}.", o.not_taken.join(" and ")));
    }
    (!s.is_empty()).then_some(s)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::await_holding_lock)]
    use super::*;
    use crate::xplane::FakeXPlane;

    /// The published airframe is process-wide; these tests take turns.
    static LOCK: Mutex<()> = Mutex::new(());

    fn a330() -> crate::aircraft::TakeoffConfigProfile {
        crate::aircraft::load("airbus_a330_300")
            .expect("A330 profile loads")
            .takeoff_config
            .expect("the A330 declares its takeoff config")
    }

    #[test]
    fn the_dial_the_a330_loads_with_is_never_left() {
        // EGPF: 100 ft on the dial, field at 20 ft, pattern 1,796 ft.
        assert_eq!(initial_altitude_ft(100.0, 20.0, Some(1795.8)), Some(1800.0));
        // No level handed down: 3,000 above the field, up to the thousand.
        assert_eq!(initial_altitude_ft(100.0, 20.0, None), Some(4000.0));
        assert_eq!(initial_altitude_ft(100.0, 2300.0, None), Some(6000.0));
        // The pilot's own dial is the pilot's...
        assert_eq!(initial_altitude_ft(6000.0, 20.0, None), None);
        // ...unless the caller knows where it will level.
        assert_eq!(
            initial_altitude_ft(6000.0, 20.0, Some(1800.0)),
            Some(1800.0)
        );
        assert_eq!(initial_altitude_ft(1800.0, 20.0, Some(1795.8)), None);
    }

    /// As found on the EGPF ramp: clean, spoilers stowed, autobrake off, 100 ft.
    #[tokio::test(start_paused = true)]
    async fn the_a330_is_configured_before_it_moves() {
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        publish(Some(a330()));
        let xp = FakeXPlane::with_reads(&[
            (FLAP_HANDLE, 0.0),
            (FLAP_DEPLOY, 0.5),
            (SPEEDBRAKE, 0.0),
            (AUTOBRAKE, 1.0),
            (ALT_DIAL, 100.0),
            (crate::datarefs::ELEVATION_M, 6.0),
            ("sim/cockpit2/controls/elevator_trim", 0.0),
            (FD_MODE, 0.0),
            (SPEED_WINDOW, 100.0),
        ]);
        let o = set_for_takeoff(&xp, Some(0.5), Some(1795.8), true).await;
        assert_eq!(xp.last_write(FLAP_HANDLE), Some(0.5));
        assert_eq!(xp.last_write(SPEEDBRAKE), Some(SPEEDBRAKE_ARMED));
        assert_eq!(xp.last_write(AUTOBRAKE), Some(0.0), "RTO");
        assert_eq!(xp.last_write(ALT_DIAL), Some(1800.0));
        assert!(xp.ran(CMD_FD_ON), "flight directors");
        assert_eq!(xp.last_write(SPEED_WINDOW), Some(153.0), "V2");
        assert!(!xp.ran(CMD_AT_ARM), "the A330's autothrust is its levers'");
        // The fake never moves, so every readback disagrees: each item is
        // NAMED, never assumed.
        assert_eq!(o.not_taken.len(), 6, "{o:?}");
        assert!(line("Takeoff config", &o)
            .unwrap()
            .starts_with("Check the flaps"));
        publish(None);
    }

    fn b738() -> crate::aircraft::TakeoffConfigProfile {
        crate::aircraft::load("boeing_737_800")
            .expect("737 profile loads")
            .takeoff_config
            .expect("the 737 declares its takeoff config")
    }

    /// EGPH 2026-09-21: armed after start against the 737's own BEFORE
    /// TAKEOFF list (SPEED BRAKE, DOWN DETENT). A Boeing's lever is never
    /// armed for the roll, and one found armed is put down.
    #[tokio::test(start_paused = true)]
    async fn the_737_takes_off_with_the_speedbrake_down() {
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        publish(Some(b738()));
        let reads = |lever: f64| {
            FakeXPlane::with_reads(&[
                (FLAP_HANDLE, 0.375),
                (FLAP_DEPLOY, 0.375),
                (SPEEDBRAKE, lever),
                (AUTOBRAKE, 0.0),
                (ALT_DIAL, 25000.0),
                (crate::datarefs::ELEVATION_M, 41.0),
                ("sim/cockpit2/controls/elevator_trim", 0.0),
            ])
        };
        let xp = reads(0.0);
        set_for_takeoff(&xp, Some(0.375), None, false).await;
        assert!(!xp.wrote(SPEEDBRAKE), "down is where it belongs");
        let xp = reads(-0.5);
        set_for_takeoff(&xp, Some(0.375), None, false).await;
        assert_eq!(xp.last_write(SPEEDBRAKE), Some(0.0));
        publish(None);
    }

    /// Armed for the roll is disarmed after the gear; a lever that is OUT in
    /// the air is somebody's and stays.
    #[tokio::test(start_paused = true)]
    async fn spoilers_armed_for_the_roll_are_disarmed_after_liftoff() {
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        publish(Some(a330()));
        let xp = FakeXPlane::with_reads(&[(SPEEDBRAKE, -0.5)]);
        after_liftoff(&xp).await;
        assert_eq!(xp.last_write(SPEEDBRAKE), Some(0.0));
        let xp = FakeXPlane::with_reads(&[(SPEEDBRAKE, 0.5)]);
        after_liftoff(&xp).await;
        assert!(!xp.wrote(SPEEDBRAKE));
        publish(None);
        let xp = FakeXPlane::with_reads(&[(SPEEDBRAKE, -0.5)]);
        after_liftoff(&xp).await;
        assert!(
            !xp.wrote(SPEEDBRAKE),
            "a silent airframe has no lever of ours"
        );
    }

    /// With nothing armed, a reject at speed raises the lever; a stop from a
    /// walk does not, and an armed Airbus is the sim's.
    #[tokio::test(start_paused = true)]
    async fn a_reject_raises_the_lever_the_takeoff_left_down() {
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        publish(Some(b738()));
        let xp = FakeXPlane::with_reads(&[(GROUNDSPEED_MS, 50.0)]);
        on_reject(&xp).await;
        assert_eq!(xp.last_write(SPEEDBRAKE), Some(1.0));
        let xp = FakeXPlane::with_reads(&[(GROUNDSPEED_MS, 5.0)]);
        on_reject(&xp).await;
        assert!(!xp.wrote(SPEEDBRAKE));
        publish(Some(a330()));
        let xp = FakeXPlane::with_reads(&[(GROUNDSPEED_MS, 50.0)]);
        on_reject(&xp).await;
        assert!(!xp.wrote(SPEEDBRAKE));
        publish(None);
    }

    /// #181: the 737 stood at thrust 1.0 against its parking brake for 67 s
    /// while full auto worked the radio. Open levers with the brake set are
    /// idled first and read back; open levers with the brake off (a roll in
    /// progress) or in the air, and a failed read, are left alone.
    #[tokio::test(start_paused = true)]
    async fn thrust_against_the_parking_brake_is_idled() {
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        publish(None);
        assert!(thrust_against_brake(Some(1.0), Some(1.0), Some(1.0)));
        assert!(!thrust_against_brake(Some(0.02), Some(1.0), Some(1.0)), "idle");
        assert!(!thrust_against_brake(Some(1.0), Some(0.0), Some(1.0)), "brake off");
        assert!(!thrust_against_brake(Some(1.0), Some(1.0), Some(0.0)), "airborne");
        assert!(!thrust_against_brake(None, Some(1.0), Some(1.0)), "no reading");
        let lever = crate::aircraft::throttle_dataref();
        let xp = FakeXPlane::with_reads(&[
            (lever.as_str(), 1.0),
            (crate::tools::PARK_BRAKE, 1.0),
            (crate::datarefs::ON_GROUND_ANY, 1.0),
            (ALT_DIAL, 5000.0),
            (crate::datarefs::ELEVATION_M, 41.0),
        ]);
        let o = set_for_takeoff(&xp, None, None, false).await;
        assert_eq!(xp.last_write(&lever), Some(0.0));
        // The fake never moves: the levers are NAMED, never assumed idle.
        assert!(o.not_taken.contains(&"the thrust levers".to_string()), "{o:?}");
        let xp = FakeXPlane::with_reads(&[
            (lever.as_str(), 1.0),
            (crate::tools::PARK_BRAKE, 0.0),
            (crate::datarefs::ON_GROUND_ANY, 1.0),
        ]);
        set_for_takeoff(&xp, None, None, false).await;
        assert!(!xp.wrote(&lever), "no brake, no thrust against it");
    }

    /// Already set by the after-start call: nothing written, nothing said.
    #[tokio::test(start_paused = true)]
    async fn a_configured_aircraft_is_left_alone() {
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        publish(Some(a330()));
        let xp = FakeXPlane::with_reads(&[
            (FLAP_HANDLE, 0.5),
            (FLAP_DEPLOY, 0.5),
            (SPEEDBRAKE, -0.5),
            (AUTOBRAKE, 0.0),
            (ALT_DIAL, 1800.0),
            (crate::datarefs::ELEVATION_M, 6.0),
            ("sim/cockpit2/controls/elevator_trim", 0.0),
            (FD_MODE, 1.0),
            (SPEED_WINDOW, 153.0),
        ]);
        let o = set_for_takeoff(&xp, Some(0.5), Some(1795.8), true).await;
        assert_eq!(o, Outcome::default());
        assert!(!xp.wrote(FLAP_HANDLE) && !xp.wrote(SPEEDBRAKE) && !xp.wrote(ALT_DIAL));
        assert_eq!(line("Takeoff config", &o), None);
        publish(None);
    }

    /// Flaps that never arrive hold the brakes for the wait and no longer,
    /// and are said.
    #[tokio::test(start_paused = true)]
    async fn the_brakes_wait_for_the_flaps_but_not_for_ever() {
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        publish(None);
        let xp = FakeXPlane::with_reads(&[
            (FLAP_HANDLE, 0.5),
            (FLAP_DEPLOY, 0.1),
            (ALT_DIAL, 5000.0),
            (crate::datarefs::ELEVATION_M, 6.0),
            ("sim/cockpit2/controls/elevator_trim", 0.0),
        ]);
        let t = tokio::time::Instant::now();
        let o = set_for_takeoff(&xp, Some(0.5), None, true).await;
        assert!(t.elapsed().as_secs_f64() >= FLAP_WAIT_SECS);
        assert_eq!(o.not_taken, vec!["the flaps, still travelling".to_string()]);
        // A 172: no spoilers, no autobrake, never touched.
        assert!(!xp.wrote(SPEEDBRAKE) && !xp.wrote(AUTOBRAKE));
    }

    /// EGPH 2026-09-21, the 737 lined up: the MCP speed window at 125, no
    /// flight directors, the autothrottle hard off. The takeoff config sets
    /// all three before the brakes come off, each read back against intent.
    #[tokio::test(start_paused = true)]
    async fn the_737_sets_its_autoflight_before_the_roll() {
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        publish(Some(b738()));
        let xp = FakeXPlane::with_reads(&[
            (FLAP_HANDLE, 0.375),
            (FLAP_DEPLOY, 0.375),
            (SPEEDBRAKE, 0.0),
            (AUTOBRAKE, 0.0),
            (ALT_DIAL, 25000.0),
            (crate::datarefs::ELEVATION_M, 41.0),
            ("sim/cockpit2/controls/elevator_trim", 0.0),
            (FD_MODE, 0.0),
            (AT_MODE, -1.0),
            (SPEED_WINDOW, 125.0),
        ]);
        xp.step_on_command(CMD_FD_ON, FD_MODE, 1.0);
        xp.step_on_command(CMD_AT_ARM, AT_MODE, 1.0);
        let o = set_for_takeoff(&xp, Some(0.375), None, false).await;
        assert!(xp.ran(CMD_FD_ON) && xp.ran(CMD_AT_ARM), "{o:?}");
        assert!(!xp.ran(CMD_AT_OFF));
        assert_eq!(xp.last_write(SPEED_WINDOW), Some(150.0), "V2 on the card");
        assert!(o.set.contains(&"flight directors on".to_string()), "{o:?}");
        assert!(o.set.contains(&"autothrottle armed".to_string()), "{o:?}");
        // The fake's window never moves: a write that did not take is named.
        assert_eq!(o.not_taken, vec!["the speed window".to_string()]);
        publish(None);
    }

    /// Already set: nothing fired, nothing written, nothing said. A silent
    /// profile touches none of it.
    #[tokio::test(start_paused = true)]
    async fn set_autoflight_is_left_alone() {
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let reads = [
            (FLAP_HANDLE, 0.375),
            (FLAP_DEPLOY, 0.375),
            (SPEEDBRAKE, 0.0),
            (AUTOBRAKE, 0.0),
            (ALT_DIAL, 25000.0),
            (crate::datarefs::ELEVATION_M, 41.0),
            ("sim/cockpit2/controls/elevator_trim", 0.0),
            (FD_MODE, 1.0),
            (AT_MODE, 0.0),
            (SPEED_WINDOW, 150.4),
        ];
        publish(Some(b738()));
        let xp = FakeXPlane::with_reads(&reads);
        let o = set_for_takeoff(&xp, Some(0.375), None, false).await;
        assert_eq!(o, Outcome::default());
        assert!(!xp.ran(CMD_FD_ON) && !xp.ran(CMD_AT_ARM) && !xp.wrote(SPEED_WINDOW));
        publish(None);
        let xp = FakeXPlane::with_reads(&reads);
        xp.set_read(FD_MODE, 0.0);
        xp.set_read(AT_MODE, -1.0);
        set_for_takeoff(&xp, Some(0.375), None, false).await;
        assert!(!xp.ran(CMD_FD_ON) && !xp.ran(CMD_AT_ARM) && !xp.wrote(SPEED_WINDOW));
    }

    /// The autothrottle's one question: armed and not engaged. Hard off is
    /// armed; an engaged mode is taken out, because the takeoff owns the
    /// levers (one writer per dataref); a failed read fires nothing.
    #[test]
    fn the_autothrottle_is_armed_never_engaged() {
        assert_eq!(autothrottle_step(0.0), None);
        assert_eq!(autothrottle_step(-1.0), Some(CMD_AT_ARM));
        assert_eq!(autothrottle_step(1.0), Some(CMD_AT_OFF));
        assert_eq!(autothrottle_step(2.0), Some(CMD_AT_OFF));
        assert_eq!(autothrottle_step(f64::NAN), None);
    }

    /// The declarations carry no names; the probe gets the stock ones behind
    /// them, and the per-airframe choices are the ones the profiles make.
    #[test]
    fn declared_autoflight_is_probed_and_per_airframe() {
        let b = b738();
        let refs = b.references();
        for n in [
            FD_MODE,
            CMD_FD_ON,
            AT_MODE,
            CMD_AT_ARM,
            CMD_AT_OFF,
            SPEED_WINDOW,
        ] {
            assert!(refs.contains(&n), "{n}");
        }
        let full = crate::aircraft::load("boeing_737_800").unwrap();
        let probed = crate::aircraft::profile_references(&full);
        assert!(probed.iter().any(|p| p == SPEED_WINDOW), "{probed:?}");
        let a = a330();
        assert!(a.flight_directors && a.v2_kts == Some(153.0));
        assert!(
            !a.autothrottle_armed,
            "the A330's script arms it from the levers"
        );
        assert!(a.thrust_reduction_agl_ft.is_none());
        assert!(!a.references().contains(&AT_MODE));
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        publish(Some(b));
        assert_eq!(
            thrust_reduction(),
            Some(ThrustReduction {
                agl_ft: 1000.0,
                lever: 0.9
            })
        );
        publish(Some(a));
        assert_eq!(thrust_reduction(), None);
        publish(None);
        assert_eq!(thrust_reduction(), None);
    }

    /// The Monitor's audit: the 737's own BEFORE TAKEOFF list, read-only,
    /// finds EGPH's state (lever armed, no FD, A/T hard off, 125 in the
    /// window) as four discrepancies, and a configured aircraft as none.
    #[test]
    fn the_before_takeoff_list_audits_the_autoflight() {
        use crate::checklist::Item;
        let profile = crate::aircraft::load("boeing_737_800").unwrap();
        let lists = crate::checklist::load(&profile).expect("the 737 checklist parses");
        let idx = crate::flow::find_titled(&lists, "before takeoff").expect("section");
        let judge = |state: &[(&str, f64)]| -> Vec<String> {
            lists[idx]
                .items
                .iter()
                .filter_map(|item| {
                    let (dref, cond) = match item {
                        Item::Act { cond, dataref, .. } | Item::Verify { cond, dataref, .. } => {
                            (dataref.as_str(), cond)
                        }
                        _ => return None,
                    };
                    let v = state.iter().find(|(d, _)| *d == dref)?.1;
                    (!cond.eval(v)).then(|| dref.to_string())
                })
                .collect()
        };
        let egph = [
            (SPEEDBRAKE, -0.5),
            (FD_MODE, 0.0),
            (AT_MODE, -1.0),
            (SPEED_WINDOW, 125.0),
        ];
        assert_eq!(judge(&egph).len(), 4, "{:?}", judge(&egph));
        let set = [
            (SPEEDBRAKE, 0.0),
            (FD_MODE, 1.0),
            (AT_MODE, 0.0),
            (SPEED_WINDOW, 150.0),
        ];
        assert!(judge(&set).is_empty(), "{:?}", judge(&set));
    }

    /// A speedbrake the descent is using is not pulled in to arm it.
    #[tokio::test(start_paused = true)]
    async fn the_landing_arms_only_a_stowed_lever() {
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        publish(Some(a330()));
        let out = FakeXPlane::with_reads(&[(SPEEDBRAKE, 0.5), (AUTOBRAKE, 4.0)]);
        let o = set_for_landing(&out).await;
        assert!(!out.wrote(SPEEDBRAKE) && !out.wrote(AUTOBRAKE), "{o:?}");
        assert!(o.deferred, "asked again once the lever is stowed");
        let stowed = FakeXPlane::with_reads(&[(SPEEDBRAKE, 0.0), (AUTOBRAKE, 1.0)]);
        set_for_landing(&stowed).await;
        assert_eq!(stowed.last_write(SPEEDBRAKE), Some(SPEEDBRAKE_ARMED));
        assert_eq!(stowed.last_write(AUTOBRAKE), Some(4.0), "MED on the A330");
        publish(None);
    }

    #[test]
    fn the_line_is_a_milestone() {
        let o = Outcome {
            set: vec!["flaps".into(), "spoilers armed".into()],
            ..Default::default()
        };
        assert_eq!(
            line("Takeoff config", &o).as_deref(),
            Some("Takeoff config: flaps, spoilers armed.")
        );
    }
}
