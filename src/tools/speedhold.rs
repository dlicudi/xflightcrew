//! `hold_speed` — the copilot works the throttle to hold a target airspeed.
//!
//! For airframes with no autothrottle (the 172): "hold 70 knots" arms a
//! background governor that nudges `throttle_ratio_all` toward the target a
//! few times a second-and-a-half. Deliberately gentle: small bounded steps
//! with a deadband, so it trims toward the speed instead of chasing gusts —
//! and it never touches pitch, so the pilot still owns the attitude (speed
//! hold with power is the stable GA division of labour on approach).
//!
//! Release: "stop holding speed" / "my throttle" (`off`), and it stands down
//! by itself the moment a recovery or autoland takes the aircraft, or on the
//! ground below taxi speed.
//!
//! Taxiing, the pilot's BRAKES are a pause, not a release: both pedals down —
//! or the keyboard/joystick brake TOGGLE, which never moves the pedals and
//! instead latches the parking-brake ratio at regular effort (~0.36, measured
//! live 2026-07-14) — brings the throttle to idle and the governor sits on
//! its hands; brakes fully off rolls back up to the held target ("resume when
//! I release the brakes", 2026-07-13; "ease off the throttle when I toggle
//! the brakes", 2026-07-14). The full stops stay explicit — the park handle
//! (ratio past 0.5) or a throttle push releases the hold entirely.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx, BRAKES_REGULAR, BRAKE_HOLD_SECS};
use crate::xplane::XPlaneClient;

const IAS: &str = "sim/cockpit2/gauges/indicators/airspeed_kts_pilot";
/// The thrust lever THIS airframe obeys — not always the stock one (the
/// Citation X tracks `throttle_jet_rev_ratio`, so every write to
/// `throttle_ratio_all` went nowhere and it flew a whole pattern at takeoff
/// thrust). Resolved per call, so a profile swap takes effect immediately.
fn throttle_dref() -> String {
    crate::aircraft::throttle_dataref()
}
/// The combined lever, -1..0 beta and 0..1 alpha: the taxi hold's lever on
/// an airframe that taxis in beta, read and written as one, so our own beta
/// can never read as the pilot's hand on the throttle.
const BETA_LEVER: &str = "sim/cockpit2/engine/actuators/throttle_beta_rev_ratio_all";

/// The taxi hold's lever for the live airframe.
fn taxi_lever() -> String {
    if crate::aircraft::taxi_beta_max().is_some() {
        BETA_LEVER.to_string()
    } else {
        throttle_dref()
    }
}
const GS_MS: &str = "sim/flightmodel/position/groundspeed";
const ON_GROUND: &str = "sim/flightmodel/failures/onground_any";
const LEFT_BRAKE: &str = "sim/cockpit2/controls/left_brake_ratio";
const RIGHT_BRAKE: &str = "sim/cockpit2/controls/right_brake_ratio";
// The held-brake command and its expiry (`BRAKES_REGULAR`/`BRAKE_HOLD_SECS`)
// live in `super` now: `hold_brakes` shares the same brake key. The history
// of WHY it's a command, not ratio writes (the override that never existed,
// the re-drive fight REST-rate writes lost) is pinned on the consts there.
/// Proportional taxi brake, served by the bundled plugin: it drives the
/// toe-brake ratios from this value every FRAME (override_toe_brakes held
/// only while braking — per-frame writes win the re-drive fight that
/// REST-rate writes lost), and expires it within a second of writes
/// stopping, ratios zeroed and override dropped — the same can't-be-left-on
/// property as the held command, but it keeps the law's magnitude. Speed
/// TRIMS go here (regular effort is right for "stop", wrong for shaving two
/// knots); stops keep the firm held command. Missing (plugin absent or old)
/// falls back to the held command — harsher but braking, never silent
/// nothing.
const TAXI_BRAKE: &str = "xcopilot/taxi_brake";
const PARK_BRAKE: &str = "sim/cockpit2/controls/parking_brake_ratio";
const MS_TO_KTS: f64 = 1.943_844;
/// Toe-brake ratio above which the pilot is deliberately braking — enough to
/// clear pedal rest/noise, well below a firm press. Only read once our own
/// held-brake command has expired, so the ratio IS the pilot's pedal.
const PILOT_BRAKE_YIELD: f64 = 0.1;
/// Both pedals back below this = fully released, the brake-pause is over and
/// the taxi hold rolls again. Lower than the yield threshold on purpose —
/// hysteresis, so a pedal hovering at the yield point can't flap the hold
/// between paused and rolling.
const PILOT_BRAKE_RESUME: f64 = 0.05;

/// The pilot has taken the brakes to SLOW, on either input path, while the
/// governor isn't braking (so the reads reflect hardware, not our own
/// command). The copilot must ease the throttle off rather than power up
/// against the brakes — the same yield it gives a throttle-lever nudge.
///
/// Two paths because X-Plane has two (both measured live, 2026-07-14):
/// - TOE PEDALS drive the left/right ratios. Deliberately `min`, not `max`:
///   a single pedal is DIFFERENTIAL braking, which is how you steer a
///   taxiing aircraft — and on a castoring-nosewheel type (the SR22) it's
///   the ONLY low-speed steering. Standing down on one pedal made the taxi
///   hold cut throttle to 0 every time the pilot turned (2026-07-12). Only
///   both pedals together reads as "I'm slowing us," not "I'm turning."
/// - The brake TOGGLE (keyboard "b" / a joystick button) never moves the
///   pedals: it latches the PARKING-BRAKE ratio at regular effort (~0.36).
///   That sat below the park-handle stand-down (> 0.5), above nothing — the
///   governor throttled against toggled brakes and never saw them
///   (2026-07-14). Anything past pedal-noise on that ratio is the pilot
///   braking; past 0.5 is the park handle, which the caller turns into a
///   full stand-down before this is ever consulted.
///
/// The caller must only trust these reads once `tools::brake_cmd_quiet()` —
/// our own held stop command rides the same parking-brake ratio while active.
///
/// Both of those guards are CLOCKS, and a clock cannot see a latch. If
/// something makes one of our own brake applications permanent, it outlives
/// `governor_braking` and it outlives `brake_cmd_quiet`, and this then reports
/// the governor's own squeeze as the pilot's foot — forever, since nothing will
/// ever release a brake nobody knows is on. That is not hypothetical: it is
/// what stranded a Baron at Malaga through the park-brake valve (2026-08-03).
/// The lesson is upstream of here. Anything that can turn a momentary brake
/// into a latched one must be disarmed before the drive starts
/// ([`crate::tools::controls::ensure_park_brake_valve`]), because at this end
/// our own brake and the pilot's are genuinely indistinguishable.
fn pilot_took_the_brakes(left: f64, right: f64, parkbrake: f64, governor_braking: bool) -> bool {
    !governor_braking && (left.min(right) > PILOT_BRAKE_YIELD || parkbrake > PILOT_BRAKE_YIELD)
}

/// True while the governor loop runs. Public so other control loops could
/// check it, mirroring `recover::ACTIVE` / `autoland::ACTIVE`.
pub static ACTIVE: AtomicBool = AtomicBool::new(false);

/// True while the governor is actively TAXIING (not a flight hold) — the
/// parking-brake tool uses this to tell the pilot the throttle is coming to
/// idle with the brake.
/// The readback for a speed handed to the follower: the number flown, and
/// why it is not the one asked when a limit moved it. Pure.
pub fn follower_speed_line(asked: f64, flown: f64) -> String {
    if (asked - flown).abs() < 0.5 {
        format!("{flown:.0} knots, on the flight plan.")
    } else if flown > asked {
        format!("{flown:.0} knots, the floor for this aircraft. {asked:.0} is too slow.")
    } else {
        format!("{flown:.0} knots, the limit for this aircraft. {asked:.0} is too fast.")
    }
}

pub fn taxi_hold_active() -> bool {
    ACTIVE.load(Ordering::Relaxed) && *MODE.lock().unwrap_or_else(|e| e.into_inner()) == Mode::Taxi
}

/// The running taxi hold's target, or `None` when the governor isn't taxiing.
/// The auto-taxi loop reads this to spot an EXTERNAL retarget (a spoken
/// "stop" / "go" / "taxi at 12") and fold it into its own speed schedule.
/// The airspeed the governor is holding in flight, when it is.
pub fn flight_target() -> Option<f64> {
    (ACTIVE.load(Ordering::Relaxed)
        && *MODE.lock().unwrap_or_else(|e| e.into_inner()) == Mode::Flight)
        .then(|| *TARGET_KTS.lock().unwrap_or_else(|e| e.into_inner()))
}

pub fn taxi_target() -> Option<f64> {
    taxi_hold_active().then(|| *TARGET_KTS.lock().unwrap_or_else(|e| e.into_inner()))
}

/// Retarget a RUNNING taxi hold — the auto-taxi speed schedule's knob (slow
/// for a bend, ramp down to the holding point). No-op unless the governor is
/// actively taxiing: this must never convert a flight hold into a taxi.
/// Deliberately does NOT touch `LAST_TAXI_KTS`, so a schedule write of 0 at
/// the hold-short line doesn't eat the pace a spoken "go" would resume.
pub fn set_taxi_target(kts: f64) {
    if taxi_hold_active() {
        *TARGET_KTS.lock().unwrap_or_else(|e| e.into_inner()) = clamp_taxi_target(kts);
    }
}

/// The airspace speed limit: 250 KIAS below 10,000 feet.
///
/// A rule of the air, not an airframe fact, so it binds everything — and it
/// is simply inert on anything that cannot reach it, which is every piston
/// here. The 737 could: its profile holds 255 KIAS in the cruise (correctly,
/// that is M0.78 up high) and the follower armed that at 1,031 ft AGL on
/// departure and held it, busting the limit by five knots for the whole climb
/// (BIKF-EGPH, 2026-09-20). The profile's own comment said "250 below 10,000
/// is the cruise climb"; nothing enforced it.
///
/// INDICATED altitude, because that is what the rule and the altimeter both
/// mean — the same reason the follower flies the altimeter.
pub const BELOW_10K_LIMIT_KTS: f64 = 250.0;
pub const SPEED_LIMIT_ALT_FT: f64 = 10_000.0;

/// `wanted`, capped to [`BELOW_10K_LIMIT_KTS`] while below
/// [`SPEED_LIMIT_ALT_FT`]. Pure, so the rule is one testable line.
pub fn airspace_limited(alt_ft: f64, wanted: f64) -> f64 {
    if alt_ft.is_finite() && alt_ft < SPEED_LIMIT_ALT_FT {
        wanted.min(BELOW_10K_LIMIT_KTS)
    } else {
        wanted
    }
}

/// How far above [`SPEED_LIMIT_ALT_FT`] a capped aircraft must climb before
/// the cap lifts. Once the cap has begun it holds through a level at or
/// near 10,000 ft (an altimeter wandering 50 ft either side of the line);
/// once lifted it comes back only below the line itself. Both edges are on
/// the legal side: the cap is never lifted below 10,000 nor missing below it.
pub const SPEED_LIMIT_LIFT_BAND_FT: f64 = 300.0;

/// [`airspace_limited`] with memory, for a loop that applies the rule every
/// tick. The stateless test flips at the line: a plan level at exactly
/// 10,000 ft, or a level-off a few feet under it, would hunt the speed target
/// 250/300 on every wander of the altimeter, the same shape as D156J's 6,000
/// ft minimum (EGKK LAM5P, 2026-09-28, #153; #154). One rule for the loop:
/// capped once below the line, lifted only past the band above it.
#[derive(Debug, Default, Clone, Copy)]
pub struct AirspaceLimit {
    /// `None` until the first finite altitude.
    capped: Option<bool>,
}

impl AirspaceLimit {
    /// Advance the latch on `alt_ft` (indicated) and return `wanted`, capped
    /// while the latch holds. A failed read (non-finite) is no measurement:
    /// the latch keeps what it knew, and with nothing known the stateless
    /// rule's answer (no invented cap) stands.
    pub fn limit(&mut self, alt_ft: f64, wanted: f64) -> f64 {
        if alt_ft.is_finite() {
            let capped = match self.capped {
                None => alt_ft < SPEED_LIMIT_ALT_FT,
                Some(true) => alt_ft < SPEED_LIMIT_ALT_FT + SPEED_LIMIT_LIFT_BAND_FT,
                Some(false) => alt_ft < SPEED_LIMIT_ALT_FT,
            };
            self.capped = Some(capped);
        }
        if self.capped == Some(true) {
            wanted.min(BELOW_10K_LIMIT_KTS)
        } else {
            wanted
        }
    }

    /// Whether the cap currently holds (`None` before any measurement).
    pub fn capped(&self) -> Option<bool> {
        self.capped
    }
}

/// Arm (or retarget) the FLIGHT speed-hold governor from code — the autoflight
/// cruise-power hold. Same Flight-mode arm the `hold_speed` tool does: sets the
/// target and, if the governor isn't already running, starts it on its own
/// thread. Idempotent; a running hold just picks up the new target.
pub fn arm_flight(base_url: String, voice: Option<crate::voice::Voice>, target_kts: f64) {
    arm_flight_with_floor(base_url, voice, target_kts, crate::aircraft::speed_floor_kts())
}

/// [`arm_flight`] against a given floor: the LANDING configuration's
/// (`aircraft::approach_floor_kts`) for a caller flying configured, the clean
/// one otherwise. LSZS final (2026-09-24): 87 kt with the flaps out floored
/// to the clean 96, said on every tick.
pub fn arm_flight_with_floor(
    base_url: String,
    voice: Option<crate::voice::Voice>,
    target_kts: f64,
    floor: Option<f64>,
) {
    // The airframe's stall floor outranks whoever is asking: a wrong number
    // handed down from a profile, a plan or a pilot is still a wrong number
    // (the RV-10's 65 kt "approach speed", 2026-09-11).
    let asked = clamp_target(target_kts);
    let target = crate::aircraft::floored_target(asked, floor);
    if target > asked {
        tracing::warn!(
            asked,
            floor = target,
            "speed governor: target raised to the airframe's floor"
        );
        crate::blackbox::record(
            "speedhold",
            serde_json::json!({"event": "target_floored", "asked": asked, "floor": target}),
        );
        if let Some(v) = &voice {
            v.alert(
                crate::voice::Priority::Caution,
                format!(
                    "{asked:.0} knots is below the {target:.0} knot floor for this airframe. Holding {target:.0}."
                ),
            );
        }
    }
    *TARGET_KTS.lock().unwrap_or_else(|e| e.into_inner()) = target;
    *MODE.lock().unwrap_or_else(|e| e.into_inner()) = Mode::Flight;
    if !ACTIVE.swap(true, Ordering::Relaxed) {
        std::thread::spawn(move || {
            let Ok(rt) = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
            else {
                ACTIVE.store(false, Ordering::Relaxed);
                return;
            };
            rt.block_on(governor(XPlaneClient::new(base_url), voice));
        });
    }
}

/// Stand the governor down (no-op if idle) — the follower calls this before
/// handing off to autoland, which runs its own speed law and must own the
/// throttle alone.
pub fn release() {
    ACTIVE.store(false, Ordering::Relaxed);
}
/// Target knots — updated live by a new `hold_speed` call, no restart.
static TARGET_KTS: Mutex<f64> = Mutex::new(0.0);
/// The last rolling taxi target, so "go" after a "stop" resumes the same
/// pace. 0.0 = never taxied this session.
static LAST_TAXI_KTS: Mutex<f64> = Mutex::new(0.0);

/// Which speed the governor chases. Flight = indicated airspeed with throttle
/// only; Taxi = groundspeed with throttle *and brakes* (idle thrust still
/// accelerates a 172 downhill — on the ground "less power" isn't enough).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Mode {
    Flight,
    Taxi,
}
static MODE: Mutex<Mode> = Mutex::new(Mode::Flight);
/// The speed the governor last saw (IAS in flight, groundspeed on the
/// ground), for the panel's "speed 95 kt (at 93)". NaN until the first tick.
static ACTUAL: Mutex<f64> = Mutex::new(f64::NAN);

/// One-line status for the dashboard's "managing" panel: what the governor is
/// chasing, or `None` when it isn't running.
pub fn status() -> Option<String> {
    if !ACTIVE.load(std::sync::atomic::Ordering::Relaxed) {
        return None;
    }
    let kts = *TARGET_KTS.lock().unwrap_or_else(|e| e.into_inner());
    let mode = *MODE.lock().unwrap_or_else(|e| e.into_inner());
    let actual = *ACTUAL.lock().unwrap_or_else(|e| e.into_inner());
    let at = if actual.is_finite() {
        format!(" (at {actual:.0})")
    } else {
        String::new()
    };
    Some(match mode {
        Mode::Taxi if kts < 1.0 => "holding position".to_string(),
        Mode::Taxi => format!("taxi {kts:.0} kt{at}"),
        Mode::Flight => format!("speed {kts:.0} kt{at}"),
    })
}

/// One governor step: throttle change for a speed error and the airspeed
/// trend, knots and knots/second → ratio. Human-like: many SMALL corrections
/// on a fast tick (500 ms) instead of big slow shoves — and the derivative
/// term reacts to a *decaying* airspeed before the error grows, which is the
/// anti-stall reflex ("the needle's unwinding, power NOW" — not "we are now
/// 10 knots slow"). The slow side gets extra authority: overspeed is a
/// comfort problem, stall is not.
pub fn throttle_adjust(err_kts: f64, trend_kts_s: f64) -> f64 {
    throttle_adjust_with(err_kts, trend_kts_s, None)
}

/// [`throttle_adjust`] for the live airframe: its declared governor
/// (`speed_governor:`), else the shared law.
pub fn throttle_adjust_live(err_kts: f64, trend_kts_s: f64) -> f64 {
    throttle_adjust_with(err_kts, trend_kts_s, crate::aircraft::speed_governor())
}

/// [`throttle_adjust`] on an airframe's own governor: the projection's lead,
/// the gain and the urgent step are the profile's, the gentle step scaled
/// with the urgent one. A big jet's engines lag seconds behind a lever that
/// moves a few knots a second per unit: the shared 5 s lead and 0.04 a tick
/// left the A330 with 0.93 of lever at its level-off and 265 kt on the
/// downwind (LEMG 30, sortie 1790633832-full_auto, #169). `None` is the
/// shared law, exactly. Pure.
pub fn throttle_adjust_with(
    err_kts: f64,
    trend_kts_s: f64,
    gov: Option<crate::aircraft::SpeedGovernorProfile>,
) -> f64 {
    let (lead_s, gain, urgent, gentle) = match gov {
        Some(g) => (g.lead_s, g.gain, g.max_step, 0.015 * g.max_step / 0.04),
        None => (THROTTLE_LEAD_S, 0.003, 0.04, 0.015),
    };
    if err_kts.abs() < 1.5 && trend_kts_s.abs() < 1.0 {
        return 0.0; // in the groove and steady: hands off
    }
    // Act on the error we are HEADED FOR, not the one on the dial. The old
    // P+D was algebraically this same projection at 2.7 s of lead, which is
    // shorter than the lever's own slew: the circuit took the Baron at 131 kt
    // off a takeoff handoff, pulled hard for a 109 kt target, and was STILL
    // pulling at 124 with the speed already falling 2.5 kt/s toward it — the
    // sag ran through the target to 98.7, one knot below blue line, and full
    // recovery authority only arrived at the old gate's raw err > 10, i.e.
    // once the aircraft was ALREADY 10 kt slow. The climb limiter then cut
    // the climb to zero and the aircraft porpoised at 700 ft AGL (EGHI,
    // 2026-08-02). Five seconds of lead stops the pull while the excess is
    // still burning off and brings the power back before the undershoot, and
    // the escalation gates watch the SAME projection — urgency about where
    // the speed will be, not where it was.
    let proj = err_kts - trend_kts_s * lead_s;
    let max_up = if proj > 10.0 { urgent } else { gentle };
    // The fast side scales with urgency too. A flat 0.015 was sized on GA,
    // where nothing accelerates hard at pattern power; a jet off the runway
    // accelerated 125 to 241 knots across one upwind-to-downwind while the
    // circuit's law pulled the lever at the polite rate the whole way (SF50,
    // Luton 2026-07-28). Well past the target, or building fast: pull like
    // you mean it. Overspeed is a comfort problem right up until it's Vmo.
    let mut max_down = if proj < -10.0 { urgent } else { gentle };
    // A declared governor pulls up to twice its urgent step as the
    // projection runs 10 to 20 kt past: the A330's lever slews 0.08 a tick
    // from 1.0 against a 4.6 s spool, and a climb given up at the level-off
    // with the lever still at 1.0 ran 12 kt over the clean-up's 212.5
    // (LEMG 30, sortie 1790698132-full_auto, 2026-09-29, #169). The shared
    // law is unchanged.
    if gov.is_some() && proj < -10.0 {
        max_down = urgent * (-proj / 10.0).min(DEEP_PULL_MAX);
    }
    (proj * gain).clamp(-max_down, max_up)
}

/// The most a declared governor's pull grows over its urgent step, as a
/// multiple, for a projection far past the target (see
/// [`throttle_adjust_with`]).
const DEEP_PULL_MAX: f64 = 2.0;

/// How far ahead [`throttle_adjust`] projects the airspeed, seconds. Longer
/// than the lever slew it has to lead — 2.7 s (the old implicit lead) let a
/// hard pull overshoot the target by ten knots — and the same order as the
/// climb limiter's and altitude capture's leads.
const THROTTLE_LEAD_S: f64 = 5.0;

/// Sanity bounds for a spoken flight target — outside this it's a mishear.
///
/// The top is the AIRFRAME's (`aircraft::speed_ceiling_kts`, Vne less a
/// margin, republished per swap); 250 only when the profile declares no Vne.
/// The flat 250 was a GA number that silently held an A330's cruise governor
/// forty knots under its profile's 290 (2026-09-20).
pub fn clamp_target(kts: f64) -> f64 {
    let ceiling = crate::aircraft::speed_ceiling_kts()
        .unwrap_or(250.0)
        .max(55.0);
    kts.clamp(55.0, ceiling)
}

/// A SPOKEN flight target, judged before anything is armed: `Err` is the
/// line to say instead. A number past the airframe's ceiling or under its
/// landing floor is a MISHEAR and is refused, never clamped: "set speed 85"
/// heard as "885 knots" on an SR22's final was clamped to its 189 kt ceiling
/// and the governor took the throttle to go and get it, and "17 knots" became
/// a 55 kt hold the airspeed warning called nine seconds later (LSZS-LILO,
/// 2026-09-24). Between the landing floor and the floor for the flaps
/// actually out, the target is raised to that floor and the line says so.
pub fn spoken_flight_target(
    asked: f64,
    ceiling: f64,
    config_floor: Option<f64>,
    landing_floor: Option<f64>,
) -> std::result::Result<(f64, Option<String>), String> {
    let ceiling = ceiling.max(55.0);
    if asked > ceiling {
        return Err(format!(
            "{asked:.0} knots is past this airframe's {ceiling:.0} knot limit. Say the speed again."
        ));
    }
    let bottom = landing_floor.unwrap_or(55.0).max(55.0);
    if asked < bottom {
        return Err(format!(
            "{asked:.0} knots is below the stall margin, {bottom:.0} at the least. Say the speed again."
        ));
    }
    match config_floor {
        Some(f) if asked < f => Ok((
            f,
            Some(format!("{asked:.0} is below the {f:.0} knot floor with these flaps. ")),
        )),
        _ => Ok((asked, None)),
    }
}

/// A flight hold that has had its wheels down this long has landed.
const TOUCHDOWN_HOLD_S: f64 = 1.0;

/// Sanity bounds for a taxi target. 0 is legitimate ("hold short" = stop).
pub fn clamp_taxi_target(kts: f64) -> f64 {
    kts.clamp(0.0, 25.0)
}

/// Idle and still fast: the flight hold at the bottom of the lever while the
/// speed does not come down. A throttle cannot slow a descending jet, and the
/// hold used to sit at idle saying nothing: 234 kt against a 180 kt target at
/// -1,400 fpm into the Inn valley, the pilot pulling a lever that was already
/// closed (LOWI, 2026-09-24). Said once per episode, after it has lasted.
#[derive(Debug, Default)]
pub struct IdleStillFast {
    since: Option<f64>,
    said: bool,
}

/// Knots over the target that count as fast at idle.
const IDLE_FAST_KTS: f64 = 15.0;
/// ...for this long, not slowing, before it is said.
const IDLE_FAST_SECS: f64 = 15.0;

impl IdleStillFast {
    /// `now_s` any monotonic clock. Returns the line to say, once.
    pub fn tick(&mut self, now_s: f64, ias: f64, target: f64, thr: f64, trend_kts_s: f64) -> Option<String> {
        let fast = thr <= 0.02 && ias > target + IDLE_FAST_KTS && trend_kts_s > -0.3;
        if ias < target + 5.0 || thr > 0.1 {
            *self = Self::default();
            return None;
        }
        if !fast {
            self.since = None;
            return None;
        }
        let since = *self.since.get_or_insert(now_s);
        if self.said || now_s - since < IDLE_FAST_SECS {
            return None;
        }
        self.said = true;
        Some(format!(
            "Throttle's at idle and we're still {:.0} knots fast. Shallow the descent to slow down.",
            ias - target
        ))
    }
}

/// Above this spoken target on the ground, the pilot means a flight hold, not a
/// taxi — and below it airborne, they don't (55-kt floor applies).
const TAXI_MAX_KTS: f64 = 30.0;

/// One taxi-governor step: `(new_throttle, brake_ratio)` for a target,
/// current groundspeed, and the speed TREND (knots/second). Throttle capped
/// well below takeoff power; brakes are for deep overspeed and stops only,
/// never speed trims; near-zero targets hold the aircraft on the brakes so
/// idle thrust can't creep it forward.
///
/// The trend term is the same anticipation the flight law has, and it's the
/// fix for the taxi limit cycle: a 10-knot taxi sawtoothed 6.6 to 11.9 knots
/// forever (blackbox, 2026-07-14) because power only came back once the
/// error was already big, seconds into the idle-throttle decel, and braking
/// at one knot over turned every crest into a stab. Leading the error by the
/// trend acts on where the speed is GOING. The 2.5 s lead matches the engine
/// spool the correction has to ride through; the caller must hand in a
/// SMOOTHED trend (the governor EMAs it) — raw tick-to-tick differences of a
/// noisy groundspeed read turn the lead into phantom brake stabs.
/// The taxi governor's throttle ceiling: 0.35 rolling (never takeoff power),
/// more to BREAK AWAY from a standstill. At the old flat 0.35 a 172 on the
/// grass with the nosewheel at full lock sat at 0.0 kt for 20 s, twice
/// (LIPK 2026-09-26), as a Jerez stand join did on pavement (2026-07-24).
/// A person adds power until it moves, then takes it off: the ceiling drops
/// back the moment the wheels turn, and the ramp to it is the governor's
/// usual 0.02 a tick, so pavement breaks away long before it gets there.
pub const TAXI_ROLLING_CAP: f64 = 0.35;
pub const TAXI_BREAKAWAY_CAP: f64 = 0.55;
const BREAKAWAY_BELOW_KTS: f64 = 0.7;

pub fn taxi_throttle_cap(gs_kts: f64) -> f64 {
    if gs_kts < BREAKAWAY_BELOW_KTS {
        TAXI_BREAKAWAY_CAP
    } else {
        TAXI_ROLLING_CAP
    }
}

pub fn taxi_control(
    target_kts: f64,
    gs_kts: f64,
    throttle: f64,
    trend_kts_s: f64,
) -> (Option<f64>, f64) {
    if target_kts < 1.0 && gs_kts < 1.5 {
        // Finish the roll on light pressure; parked effort only once the
        // wheels are still — half brakes arriving at walking pace snapped
        // the aircraft to a halt nose-first ("jerks forward as it brakes",
        // 2026-07-22). The slew carries the squeeze between the two.
        let brake = if gs_kts > 0.3 { 0.18 } else { 0.5 };
        return (Some(0.0), brake); // holding position: idle + brakes
    }
    // Where we'll be once a lever change has worked through the spool.
    let raw = target_kts - gs_kts;
    let eff = raw - trend_kts_s * 2.5;
    // A build of speed that only the PROJECTION calls deep: at a slow target
    // the ordinary breakaway acceleration is enough to do it. The Baron's
    // 6 kt line-up pace read 6.9 kt building 0.9 kt/s, the projection put
    // it 3 kt over, idle-and-brake followed, prop drag stopped it dead, and
    // the rebuild from idle repeated the cycle three times in 205 m (Malaga,
    // 2026-09-11). Idle-and-brake needs the speed to actually BE over by a
    // couple of knots, unless the build is a genuine surge (downhill).
    let surge = trend_kts_s >= SURGE_KTS_S;
    if eff > 1.0 {
        // Too slow, or decaying toward it: a touch more power (never takeoff
        // power), brakes off. Small steps on a fast tick, human feet, not a
        // bang-bang thermostat; well behind the pace the step doubles so the
        // rebuild from idle doesn't undershoot for ten seconds.
        let step = (eff * 0.003).min(if eff > 3.0 { 0.02 } else { 0.01 });
        (Some((throttle + step).min(taxi_throttle_cap(gs_kts))), 0.0)
    } else if eff < -3.0 && (target_kts < 1.0 || raw < -2.0 || surge) {
        // WELL past the target: idle now and brake in proportion, together —
        // the polite step-down let an SR22 run a 10-knot taxi through 20
        // (2026-07-11). Ramped from the boundary: the old flat 0.2 minimum
        // stabbed the pedals the instant this branch was crossed. Rolling
        // TRIMS (a bend slow-down, target still moving) cap well below a
        // stop's effort: a 12→6 kt bend target braked at 0.45 and threw
        // everyone forward ("still very abrupt", SR22 2026-07-22).
        // Rolling caps came down AGAIN for the SF50 ("braking / slowing down
        // is a bit jerky", 2026-07-28) — the third airframe to say so, and
        // every complaint has moved these numbers the same direction. A stop
        // keeps its 0.6 authority; a rolling trim is passengers' comfort.
        // ...but comfort is for TRIMS. A runaway (the Evolution at 23 kt for
        // 10, into a wall at LIME 2026-09-25) gets a stop's authority.
        let cap = if target_kts < 1.0 || raw < -RUNAWAY_OVER_KTS { 0.6 } else { 0.22 };
        (Some(0.0), ((-eff - 3.0) * 0.09 + 0.08).clamp(0.08, cap))
    } else if eff < -1.0 {
        // A little fast: power off is the whole correction for a rolling
        // taxi — idle thrust won't hold it fast on the flat, and braking to
        // shave one knot is how the ride turns into a rodeo. Gently: the
        // old 0.04 dump slammed the lever shut in half a second and bought
        // the deep undershoot on the other side. A STOP keeps braking to
        // the halt once the throttle's closed: idle creep never finishes a
        // stop on its own.
        // Once the speed is already falling, the lever is where it should
        // be: stepping it further toward idle is what turned a one-knot
        // trim into a full stop on the Baron's line-up.
        if throttle > 0.03 && trend_kts_s > SETTLING_KTS_S {
            (Some((throttle - 0.01).max(0.0)), 0.0)
        } else if throttle > 0.03 {
            (None, 0.0)
        } else if target_kts < 1.0 {
            (Some(0.0), ((-eff) * 0.08).clamp(0.1, 0.6))
        } else {
            (Some(0.0), 0.0)
        }
    } else {
        (None, 0.0) // in the groove: hands off, brakes released
    }
}

/// The taxi law with a BETA range below idle ([`crate::aircraft::TaxiProfile::beta_max`]):
/// `lever` is X-Plane's combined lever (-1..0 beta, 0..1 alpha). A rolling
/// overspeed goes into beta, never the brakes; the brakes stay for a stop,
/// for an overspeed past [`BETA_BRAKE_KTS`], and for one beta cannot take
/// once it is spent ([`beta_spent_brake`]). Out of beta, the lever comes
/// back to idle quickly when the pace is wanted again. Pure.
pub fn taxi_control_beta(
    target_kts: f64,
    gs_kts: f64,
    lever: f64,
    trend_kts_s: f64,
    beta_max: f64,
) -> (Option<f64>, f64) {
    let beta_max = beta_max.clamp(0.0, 0.9);
    let (thr, brake) = taxi_control(target_kts, gs_kts, lever.max(0.0), trend_kts_s);
    if target_kts < 1.0 {
        // A stop: the brakes, as ever, and idle once nearly still (beta
        // cannot hold it still). Still rolling, the beta already in stays
        // in: out to idle at 7 kt the Evolution's PT6 pushed on against the
        // stop's brakes (LILB 03, 1790740292, 2026-09-30).
        if lever < 0.0 && gs_kts > 1.5 {
            let brake = if beta_spent(lever, beta_max) {
                brake.max(beta_spent_brake(-gs_kts))
            } else {
                brake
            };
            return (Some(lever), brake);
        }
        return (thr.map(|t| t.max(0.0)).or(Some(0.0)), brake);
    }
    let raw = target_kts - gs_kts;
    let eff = raw - trend_kts_s * 2.5;
    if beta_spent(lever, beta_max) && raw < 0.0 {
        // BETA SPENT and still over the pace: the lever stays at its
        // deepest and the brakes take the rest, in proportion.
        let brake = if -eff >= BETA_BRAKE_KTS {
            brake
        } else {
            beta_spent_brake(raw - BETA_SPENT_LEAD_S * trend_kts_s)
        };
        return (Some(-beta_max), brake);
    }
    if eff > 1.0 && lever < 0.0 {
        // Wanted the pace again: out of beta toward idle, briskly.
        return (Some((lever + BETA_STEP_OUT).min(0.0)), 0.0);
    }
    let fast_at_idle = thr.is_some_and(|t| t <= 0.0) && eff < -1.0;
    if brake > 0.0 || fast_at_idle {
        // At least the depth the overspeed asks for, and DEEPER each tick
        // the speed is not falling: a fixed depth per knot held the
        // Evolution at 17.7 kt for a 10 kt taxi with beta engaged at -0.43
        // (LIME 2026-09-25). A foot on a lever, walked back until it bites.
        let floor = ((-eff - 1.0) * BETA_PER_KT).clamp(BETA_MIN, beta_max);
        let falling = trend_kts_s < BETA_BITING_KTS_S;
        let next = if lever > -floor {
            (lever - BETA_STEP_IN).max(-floor)
        } else if falling {
            lever
        } else {
            lever - BETA_STEP_IN
        };
        let emergency = -eff >= BETA_BRAKE_KTS;
        return (Some(next.max(-beta_max)), if emergency { brake } else { 0.0 });
    }
    if lever < 0.0 && eff >= -1.0 {
        // On pace again while in beta: ease back toward idle.
        return (Some((lever + BETA_STEP_IN).min(0.0)), 0.0);
    }
    (thr, brake)
}
/// Beta is SPENT: the lever within one step out ([`BETA_STEP_OUT`]) of the
/// profile's deepest, so the law's own ease out and back is still spent.
fn beta_spent(lever: f64, beta_max: f64) -> bool {
    beta_max > 0.0 && lever <= -beta_max + BETA_STEP_OUT + 1e-9
}

/// The brake once BETA IS SPENT, for a speed `under_kts` under the target
/// (negative: over it) led by the trend: past [`BETA_SPENT_OVER_KTS`] over,
/// from [`BETA_SPENT_MIN`] in proportion, never past a rolling trim's
/// [`BETA_SPENT_MAX`]; 0 inside the band. The Evolution at idle in full beta
/// (0.8) rolls on at 13 to 14 kt by itself (LILB 03, 2026-09-30), so every
/// target below that was a target the hold could not fly: the backtrack's
/// leg one (1790740292) held 13.2 kt into its stop ramp's 6.4 with no brake
/// until 7 m to go, 9.6 kt at the end and about 9.5 m past it. Pure.
pub fn beta_spent_brake(under_kts: f64) -> f64 {
    let over = -under_kts - BETA_SPENT_OVER_KTS;
    if over <= 0.0 {
        0.0
    } else {
        (BETA_SPENT_MIN + over * BETA_SPENT_PER_KT).min(BETA_SPENT_MAX)
    }
}
const BETA_SPENT_OVER_KTS: f64 = 0.5;
const BETA_SPENT_MIN: f64 = 0.04;
const BETA_SPENT_PER_KT: f64 = 0.08;
const BETA_SPENT_MAX: f64 = 0.22;
/// The spent brake's lead on the speed trend, s (the overspeed law's 2.5
/// held the brake off a stop ramp it was 3 kt behind).
const BETA_SPENT_LEAD_S: f64 = 0.5;
/// Beta depth per knot of overspeed, and the shallowest worth commanding.
const BETA_PER_KT: f64 = 0.06;
const BETA_MIN: f64 = 0.08;
/// Per tick into beta, and out of it when the pace is wanted back.
const BETA_STEP_IN: f64 = 0.03;
const BETA_STEP_OUT: f64 = 0.06;
/// Slowing at least this fast (kt/s): the beta depth has bitten, hold it.
const BETA_BITING_KTS_S: f64 = -0.2;
/// Overspeed (kt, projected) that still earns the brakes with beta in.
pub const BETA_BRAKE_KTS: f64 = 8.0;

/// Over the target by this much is a runaway, not a trim: the brake gets a
/// stop's authority.
const RUNAWAY_OVER_KTS: f64 = 8.0;

/// A build of speed this fast (kt/s) is a surge — downhill, a gust — and
/// earns idle-and-brake on the projection alone, before the speed gets there.
const SURGE_KTS_S: f64 = 1.2;
/// Falling faster than this (kt/s) means a trim has already taken: leave the
/// lever where it is rather than chase the speed down to idle.
const SETTLING_KTS_S: f64 = -0.3;

/// Smooth the proportional brake toward the law's demand: pedals SQUEEZE
/// on, they don't stab ("braking is TOO jerky", 2026-07-20 — every full
/// stop was the binary brake key at regular effort in one frame, and trim
/// steps hit the channel raw). Asymmetric on purpose: releasing quickly is
/// not a jerk, applying quickly is.
pub fn brake_slew(prev: f64, want: f64, dt_s: f64) -> f64 {
    let dt = dt_s.clamp(0.02, 1.0);
    // 0.35/s apply: the 0.5/s squeeze still read as a stab on the SR22's
    // effective brakes ("still very abrupt", 2026-07-22).
    // 0.28/s apply: 0.35 still drew "a bit jerky" from the SF50's cabin
    // (2026-07-28). Releases stay quick — that is never the jerk.
    let step = if want > prev { 0.28 * dt } else { 1.5 * dt };
    prev + (want - prev).clamp(-step, step)
}

/// Must this quiet tick WRITE a zero to the proportional brake channel?
///
/// Two reasons. We were braking and have stopped, so the squeeze is released
/// promptly rather than left to the channel's expiry. Or this governor has not
/// zeroed the channel even once, which makes the arm-time state explicit
/// instead of inherited — a fresh governor starts with `braking` false, so
/// without this the release branch never runs at all on a drive that never
/// brakes.
///
/// The second half was written to fix the Malaga stall on the theory that the
/// toe RATIOS latch across a standdown, and that theory is WRONG — worth
/// keeping here, because a fix with a plausible story attached is how the
/// `tow_in_progress` dead code survived a fortnight. Probed live the next day
/// (2026-08-03) with the aircraft still stuck: `left_brake_ratio` and
/// `right_brake_ratio` both read 0.0 and `override_toe_brakes` 0, so the
/// plugin's expiry cleans up after itself exactly as documented. What was
/// actually holding the Baron was `parking_brake_ratio`, latched at the value
/// of the governor's own squeeze by the park-brake VALVE — see
/// [`crate::tools::controls::ensure_park_brake_valve`], which is the real fix.
/// This one stays because zeroing a channel you have not zeroed is honest and
/// costs one write, not because it was ever the bug.
pub fn should_release_brake_channel(braking: bool, already_released: bool) -> bool {
    braking || !already_released
}

pub struct HoldSpeed {
    /// The profile's final-approach speed, for "hold approach speed".
    pub approach_kts: Option<f64>,
    /// The airframe's park-brake valve SET sense, for the taxi auto-release.
    pub park_brake_valve_set: Option<f64>,
}

#[derive(Deserialize)]
struct Args {
    /// Target IAS, knots. Absent = hold the current speed.
    #[serde(default)]
    knots: Option<f64>,
    /// Named target: "approach" = the profile's final-approach speed.
    #[serde(default)]
    preset: Option<String>,
    /// true = release the throttle back to the pilot.
    #[serde(default)]
    off: bool,
    /// true = "go": resume the last rolling taxi speed after a stop.
    #[serde(default)]
    resume: bool,
}

#[async_trait]
impl Tool for HoldSpeed {
    fn name(&self) -> &str {
        "hold_speed"
    }

    fn description(&self) -> &str {
        "Hold a target speed by managing the throttle. Airborne: airspeed hold \
         ('hold 70 knots', 'maintain approach speed') — power only, the pilot \
         flies pitch. On the ground: taxi speed hold with throttle AND brakes \
         ('taxi at 10 knots'; 'stop' / 'hold short' / 'hold position' = knots 0, \
         brake to a halt and hold; 'go' / 'continue taxi' = resume the last taxi \
         speed) — the pilot steers. Release with 'stop holding speed' or 'my \
         throttle'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "knots": {"type": "number", "description": "target IAS, knots; absent = hold current speed"},
                "preset": {"type": "string", "enum": ["approach"], "description": "named target: the profile's final-approach speed"},
                "off": {"type": "boolean", "description": "true = release the throttle"},
                "resume": {"type": "boolean", "description": "true = resume the last taxi speed after a stop ('go')"}
            }
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let mut a: Args = serde_json::from_value(args.clone())?;
        if a.off {
            return Ok(if ACTIVE.swap(false, Ordering::Relaxed) {
                "Speed hold off — your throttle.".to_string()
            } else {
                "I wasn't holding the speed.".to_string()
            });
        }
        let on_ground = ctx.xplane.get_float(ON_GROUND).await.unwrap_or(0.0) > 0.5;
        // "Go" — resume the pace we were taxiing at before the stop.
        if a.resume {
            if !on_ground {
                return Ok("We're airborne — nothing to resume down there.".to_string());
            }
            let last = *LAST_TAXI_KTS.lock().unwrap_or_else(|e| e.into_inner());
            if last < 1.0 {
                return Ok("We weren't taxiing — say a speed, like taxi at ten knots.".to_string());
            }
            a.knots = Some(last);
        }
        // A spoken "stop" in the air is a mishear or a panic; either way the
        // flight hold's 55-knot floor must not be the answer to it.
        if !on_ground && a.knots.is_some_and(|k| k < 1.0) {
            return Ok(
                "We're airborne — I can't stop the aircraft. Say a speed to hold.".to_string(),
            );
        }
        // ONE owner for the lever. The circuit, a procedure and autoland each
        // manage speed themselves, and arming a second governor on top is a
        // tug-of-war, not a handover: "reduce airspeed to 160" spoken on a
        // circuit's downwind armed this hold against the circuit's own power
        // law, each side re-commanding over the other, and the lever never
        // came down — the aircraft rode X-Plane's overspeed protection at
        // 270 KIAS for three minutes while the circuit commanded a descent it
        // never got (SF50, Luton 2026-07-28). Refuse with the real answer.
        // `off: true` still passes: releasing a hold is always safe.
        if !on_ground && !a.off {
            if crate::circuit::ACTIVE.load(Ordering::Relaxed) {
                return Ok(
                    "The circuit has the throttle — it flies the pattern speed. \
                           Say cancel circuit if you want the speed."
                        .to_string(),
                );
            }
            if crate::autoland::ACTIVE.load(Ordering::Relaxed) {
                return Ok(
                    "I'm landing and have the throttle. Say my controls if you want the speed."
                        .to_string(),
                );
            }
            // The follower owns the lever, so a number goes TO it (as a
            // spoken altitude does), never onto a second governor. It said
            // "the approach has the throttle" in an enroute cruise and would
            // not take 110 knots (LIPE-LIPK, 2026-09-25).
            if crate::procedure::ACTIVE.load(Ordering::Relaxed) {
                return Ok(match a.knots {
                    Some(k) => {
                        // The floor for the configuration flown: the clean
                        // 167 refused 120 and 140 on a 737 with the flaps
                        // out (LEMG, 2026-09-26).
                        let flaps = ctx
                            .xplane
                            .get_float(crate::datarefs::FLAP_RATIO)
                            .await
                            .unwrap_or(0.0);
                        let floor = crate::aircraft::config_floor_kts(flaps)
                            .or(crate::aircraft::speed_floor_kts());
                        let ceiling = crate::aircraft::speed_ceiling_kts();
                        let want = crate::aircraft::floored_target(k, floor);
                        let want = ceiling.map_or(want, |c| want.min(c));
                        crate::procedure::take_speed(want);
                        follower_speed_line(k, want)
                    }
                    None => "The flight plan has the throttle. Give me a number in knots and I'll fly it."
                        .to_string(),
                });
            }
        }
        // Mode by context: a small number on the ground is a taxi; airborne (or
        // a big number) is a flight hold. A big number on the ground is neither.
        let taxi = on_ground && a.knots.map_or(a.preset.is_none(), |k| k <= TAXI_MAX_KTS);
        if on_ground && !taxi {
            return Ok("We're on the ground — say a taxi speed like 'taxi at 10 knots', or ask again airborne.".to_string());
        }
        // A crew-held "brakes on" yields to a commanded taxi the same way the
        // parking brake does: rolling was just asked for. A stop target hands
        // over silently, the governor parks on its own boot before the held
        // command expires. Released FIRST: the held command rides the
        // parking-brake ratio, so with the hold still on the check below
        // would read our own braking as the pilot's brake toggle.
        let crew_brakes_released = taxi && super::controls::release_held_brakes();
        // A commanded taxi carries the intent to ROLL: release the brakes
        // ourselves and say so — "release it first" sent the pilot to do the
        // copilot's job. Past 0.5 the ratio is the park HANDLE; the band
        // between pedal-noise and 0.5 is the keyboard/joystick brake TOGGLE's
        // regular-effort latch (~0.36, measured 2026-07-14) — unless a crew
        // hold was JUST dropped above, whose decaying command sits in that
        // same band and needs no release. Through the ONE valve-aware routine
        // the parking-brake tool uses: the ratio-only shortcut said "parking
        // brake off" on the SR22 while the valve held the wheels
        // (2026-07-11). A stop target with the brake set needs no governor.
        let mut brake_released = false;
        let pb = ctx.xplane.get_float(PARK_BRAKE).await.unwrap_or(0.0);
        if taxi && (pb > 0.5 || (pb > PILOT_BRAKE_YIELD && !crew_brakes_released)) {
            if a.knots.is_some_and(|k| k < 1.0) {
                return Ok("Parking brake's set — we're staying put.".to_string());
            }
            brake_released =
                super::controls::drive_park_brake(ctx.xplane, false, self.park_brake_valve_set)
                    .await
                    .unwrap_or(false);
            if !brake_released {
                return Ok(
                    "Parking brake's set and it won't release from here — check the handle."
                        .to_string(),
                );
            }
        }
        // The valve is ISOLATED for the whole drive, not merely as a side
        // effect of releasing a brake that happened to be set. It has to be
        // unconditional, because the failure needs no parking brake to start
        // it: left in the SET position the valve traps brake pressure, and the
        // governor's own proportional squeezes — whose entire contract is that
        // they expire within a second — latch into `parking_brake_ratio` at
        // whatever value was squeezed. The tick after that reads its own brake
        // as the pilot's foot and pauses, so the drive stops itself and waits
        // forever for a pilot who is not braking (Malaga, BE58, 2026-08-03:
        // one 0.10 anticipatory squeeze at 9 kt, stopped four seconds later,
        // stagnant with 602 of 631 m to run). An airframe declaring no valve
        // is untouched.
        if taxi {
            super::controls::ensure_park_brake_valve(ctx.xplane, false, self.park_brake_valve_set)
                .await;
        }
        let mut floor_note: Option<String> = None;
        let target = if taxi {
            match a.knots {
                Some(k) => clamp_taxi_target(k),
                // "Hold this speed" while rolling = keep the current pace.
                None => clamp_taxi_target(ctx.xplane.get_float(GS_MS).await? * MS_TO_KTS),
            }
        } else if let Some(k) = a.knots {
            let flaps = ctx
                .xplane
                .get_float(crate::datarefs::FLAP_RATIO)
                .await
                .unwrap_or(0.0);
            match spoken_flight_target(
                k,
                crate::aircraft::speed_ceiling_kts().unwrap_or(250.0),
                crate::aircraft::config_floor_kts(flaps),
                crate::aircraft::approach_floor_kts().or(crate::aircraft::speed_floor_kts()),
            ) {
                Err(line) => {
                    crate::blackbox::record(
                        "speedhold",
                        json!({"event": "spoken_target_refused", "asked": k}),
                    );
                    return Ok(line);
                }
                Ok((t, note)) => {
                    floor_note = note;
                    t
                }
            }
        } else if a.preset.as_deref() == Some("approach") {
            match self.approach_kts {
                Some(k) => clamp_target(k),
                None => {
                    return Ok(
                        "No approach speed on file for this airframe — give me a number."
                            .to_string(),
                    )
                }
            }
        } else {
            // "Maintain this speed" — hold whatever we're doing right now.
            clamp_target(ctx.xplane.get_float(IAS).await?)
        };
        if taxi && target >= 1.0 {
            *LAST_TAXI_KTS.lock().unwrap_or_else(|e| e.into_inner()) = target;
        }
        *TARGET_KTS.lock().unwrap_or_else(|e| e.into_inner()) = target;
        *MODE.lock().unwrap_or_else(|e| e.into_inner()) =
            if taxi { Mode::Taxi } else { Mode::Flight };
        // Already governing → just the new target/mode; otherwise start the
        // loop — on its OWN thread with its own runtime, like recover and the
        // monitor. The main runtime is current-thread and blocks on the mic
        // between commands, so a tokio::spawn there starves: the first live
        // governor sat unpolled for 23 seconds until the next voice command.
        if !ACTIVE.swap(true, Ordering::Relaxed) {
            let base = ctx.config.xplane_base_url.clone();
            let voice = ctx.voice.cloned();
            std::thread::spawn(move || {
                let Ok(rt) = tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                else {
                    ACTIVE.store(false, Ordering::Relaxed);
                    return;
                };
                rt.block_on(governor(XPlaneClient::new(base), voice));
            });
        }
        Ok(if taxi {
            let brake_note = match (brake_released, crew_brakes_released && target >= 1.0) {
                (true, true) => "Parking brake off, my brakes too. ",
                (true, false) => "Parking brake off. ",
                (false, true) => "My brakes are off. ",
                (false, false) => "",
            };
            if target < 1.0 {
                "Stopping — brakes on. Say go to roll again.".to_string()
            } else if a.resume {
                format!("{brake_note}Rolling — back to {target:.0} knots.")
            } else {
                format!(
                    "{brake_note}Taxiing at {target:.0} knots — I have throttle and brakes, you steer. Say my throttle to take it back."
                )
            }
        } else {
            format!(
                "{}I have the throttle, holding {target:.0} knots. Say stop holding speed to take it back.",
                floor_note.unwrap_or_default()
            )
        })
    }
}

/// Glide a lever dataref from wherever it is to `to` in 50 ms micro-steps —
/// continuous motion like a hand, not a stepped jump (per-tick 1.5% snaps
/// still read as jerky on the quadrant; this is the smoothing between ticks).
async fn glide(xp: &XPlaneClient, dataref: &str, to: f64, over_ms: u64) {
    let Ok(from) = xp.get_float(dataref).await else {
        let _ = xp.set_float(dataref, to).await;
        return;
    };
    let steps = (over_ms / 50).max(1);
    for i in 1..=steps {
        let v = from + (to - from) * (i as f64 / steps as f64);
        let _ = xp.set_float(dataref, v).await;
        if i < steps {
            tokio::time::sleep(std::time::Duration::from_millis(50)).await;
        }
    }
}

/// The governor loop: gentle bounded nudges toward the target until released.
/// Flight mode chases IAS with throttle only; taxi mode chases groundspeed with
/// throttle and brakes on a faster cadence. Brakes are always released on the
/// way out — never hand the pilot an aircraft with a hidden boot on the pedals.
async fn governor(xp: XPlaneClient, voice: Option<crate::voice::Voice>) {
    tracing::info!("speed governor started");
    // On file both ends: whether a governor held the speed after the
    // enroute air start could only be inferred from the throttle trace
    // (#177, 2026-09-29).
    let life = |event: &str| {
        crate::blackbox::record(
            "speedhold",
            serde_json::json!({
                "event": event,
                "mode": format!("{:?}", *MODE.lock().unwrap_or_else(|e| e.into_inner())),
                "target": *TARGET_KTS.lock().unwrap_or_else(|e| e.into_inner()),
            }),
        );
    };
    life("governor_started");
    let mut braking = false;
    // Our own brake pollution, by channel: the proportional channel drives
    // the PEDAL ratios (gated by `braking`), and the held stop command rides
    // the PARKING-BRAKE ratio (gated by the shared `brake_cmd_quiet` clock,
    // which also covers the crew's `hold_brakes` holder). Both gates must
    // pass before a brake read is believed to be the pilot's.
    // The proportional channel failed once (plugin absent or old): warned,
    // trims fall back to the held command for the rest of this governor.
    let mut trim_fallback = false;
    // Whether the proportional channel has been zeroed since this governor
    // armed. It has NOT, and that is the whole of this flag: the channel's
    // plugin override expires in a second, but the toe ratios KEEP the last
    // value written under it, and on an airframe with no toe-brake axis
    // assigned nothing re-drives them back to zero. So a governor that stopped
    // while holding a stop leaves the brakes on at 0.5 — invisibly, since it
    // is not the parking brake, lights no annunciator, and the pilot has no
    // pedal to clear it with. The NEXT drive then inherits them: at Malaga a
    // Baron made 0.66 kt in eighty-five seconds, spun 73 degrees on the spot
    // and was called stagnant, and both of that day's taxis peaked at 3.9 and
    // 0.7 kt against 11-14 on every day before it (2026-08-03; probed live,
    // the ratios still read 0.5 minutes after standdown and a plain write of
    // zero stuck, so nothing at all was re-driving them). Cleared once on arm,
    // so no drive can ever start against the last one's brakes.
    let mut brake_released = false;
    // The slewed brake actually written (the squeeze), and its clock.
    let mut brake_out = 0.0_f64;
    let mut last_brake_t: Option<std::time::Instant> = None;
    // Brake-paused: the pilot pressed both pedals mid-taxi, the throttle went
    // to idle, and the hold is waiting for the pedals to come fully off.
    let mut paused = false;
    // The throttle position this loop last commanded. If the lever is later
    // found far from where we put it, that's the PILOT's hand — yield
    // rather than fight them for the lever (a taxi hold once yanked the
    // throttle back from a takeoff push; never again).
    let mut last_cmd: Option<f64> = None;
    // One deviated read is NOT yet a hand: a hardware-axis micro-bump
    // re-drives the dataref to wherever the physical lever RESTS (the jump
    // size says nothing about the movement size), and that killed an auto
    // taxi into a silent runaway (Gatwick, 2026-07-20). So the first
    // deviation is challenged — re-command ours once and look again next
    // tick. A hand keeps re-driving over the re-command and wins on the
    // second read (~a tick of contest, imperceptible); a bump doesn't and
    // the hold rides on.
    let mut override_pending = false;
    // Last IAS sample, for the flight governor's trend (anticipation) term.
    let mut prev_ias: Option<(f64, std::time::Instant)> = None;
    // Last groundspeed sample and the EMA-smoothed trend for the taxi law's
    // anticipation term. Smoothed on purpose: the 2.5 s lead amplifies any
    // read noise 25x if fed raw tick-to-tick differences.
    let mut prev_gs: Option<(f64, std::time::Instant)> = None;
    let mut idle_fast = IdleStillFast::default();
    // When a FLIGHT hold first found the wheels on the ground (a landing).
    let mut ground_since: Option<std::time::Instant> = None;
    let clock = std::time::Instant::now();
    let mut gs_trend = 0.0_f64;
    loop {
        if !ACTIVE.load(Ordering::Relaxed) {
            break;
        }
        // Yield instantly to anything that takes the whole aircraft.
        if super::recover::ACTIVE.load(Ordering::Relaxed)
            || crate::autoland::ACTIVE.load(Ordering::Relaxed)
        {
            ACTIVE.store(false, Ordering::Relaxed);
            break;
        }
        // Frozen sim (paused or replay): both laws are P+D against a state
        // that isn't moving — the error never changes, so the throttle (or
        // brake) command walks to its clamp and springs on unfreeze. Hold
        // everything exactly as it is and wait for live. The trend windows
        // (0.2–2 s) discard the stale sample on resume.
        if crate::xplane::sim_frozen(&xp).await {
            tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            continue;
        }
        let target = *TARGET_KTS.lock().unwrap_or_else(|e| e.into_inner());
        let mode = *MODE.lock().unwrap_or_else(|e| e.into_inner());
        // Three-valued on purpose: a FAILED read is a sim hiccup, not an
        // answer — it must never impersonate "airborne" (the Web API blinked
        // during a plugin reload and the taxi governor dropped a braking
        // stop as 'airborne, released' at 10 knots, 2026-07-13).
        let on_ground = xp.get_float(ON_GROUND).await.ok().map(|v| v > 0.5);
        match mode {
            Mode::Flight => {
                if let (Ok(ias), Ok(thr)) = (
                    xp.get_float(IAS).await,
                    xp.get_float(&throttle_dref()).await,
                ) {
                    if let Some(lc) = last_cmd.filter(|lc| (thr - lc).abs() > 0.15) {
                        if override_pending {
                            tracing::info!(thr, lc, "speed governor: pilot override, releasing");
                            ACTIVE.store(false, Ordering::Relaxed);
                            break;
                        }
                        override_pending = true;
                        tracing::info!(
                            thr,
                            lc,
                            "speed governor: throttle deviated, challenging once"
                        );
                        let _ = xp.set_float(&throttle_dref(), lc).await;
                        tokio::time::sleep(std::time::Duration::from_millis(250)).await;
                        continue;
                    }
                    override_pending = false;
                    // Stand down when parked — holding 70 knots against the
                    // brakes is not a thing anyone wants. (Never silent: this
                    // exact path once ate three arm requests invisibly.)
                    if on_ground == Some(true) && xp.get_float(GS_MS).await.unwrap_or(0.0) < 5.0 {
                        tracing::info!("speed governor: parked, standing down");
                        ACTIVE.store(false, Ordering::Relaxed);
                        break;
                    }
                    // A flight hold on the runway is a landing: idle and hand
                    // back. LIPE 12 (2026-09-25): an 80 kt hold asked for in
                    // the flare stayed on through the touchdown, the roll went
                    // 64 to 75 kt with a bounce to 5 ft, and the pilot's
                    // "taxi at 6" was judged against the stall margin.
                    ground_since = match on_ground {
                        Some(true) => ground_since.or(Some(std::time::Instant::now())),
                        Some(false) => None,
                        None => ground_since,
                    };
                    if ground_since.is_some_and(|t| t.elapsed().as_secs_f64() >= TOUCHDOWN_HOLD_S) {
                        let _ = xp.set_float(&throttle_dref(), 0.0).await;
                        tracing::info!(ias, "speed governor: touchdown, idle and standing down");
                        crate::blackbox::record(
                            "speedhold",
                            serde_json::json!({"event": "touchdown_idle", "ias": ias, "target": target}),
                        );
                        if let Some(v) = &voice {
                            v.alert(
                                crate::voice::Priority::Caution,
                                "Touchdown. Throttle idle, speed hold off.",
                            );
                        }
                        ACTIVE.store(false, Ordering::Relaxed);
                        break;
                    }
                    // IAS trend across ticks feeds the anticipation term.
                    let now_t = std::time::Instant::now();
                    let trend = match prev_ias {
                        Some((last_ias, last_t)) => {
                            let dt = now_t.duration_since(last_t).as_secs_f64();
                            if (0.2..=2.0).contains(&dt) {
                                (ias - last_ias) / dt
                            } else {
                                0.0
                            }
                        }
                        None => 0.0,
                    };
                    prev_ias = Some((ias, now_t));
                    *ACTUAL.lock().unwrap_or_else(|e| e.into_inner()) = ias;
                    if let Some(line) =
                        idle_fast.tick(clock.elapsed().as_secs_f64(), ias, target, thr, trend)
                    {
                        crate::blackbox::record(
                            "speedhold",
                            serde_json::json!({"event": "idle_still_fast", "ias": ias, "target": target}),
                        );
                        if let Some(v) = &voice {
                            v.say(crate::voice::Priority::Caution, line);
                        }
                    }
                    let adj = throttle_adjust_live(target - ias, trend);
                    if adj != 0.0 {
                        let cmd = (thr + adj).clamp(0.0, 1.0);
                        glide(&xp, &throttle_dref(), cmd, 200).await; // ~most of the tick
                        tokio::time::sleep(std::time::Duration::from_millis(50)).await;
                    } else {
                        // In the groove: don't spin the API at 20 Hz doing nothing.
                        tokio::time::sleep(std::time::Duration::from_millis(250)).await;
                    }
                    if adj != 0.0 {
                        last_cmd = Some((thr + adj).clamp(0.0, 1.0));
                    }
                } else {
                    // Reads failed (sim hiccup): back off, don't spin.
                    tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
                }
            }
            Mode::Taxi => {
                // Can't read the squat switch: hold everything exactly as it
                // is (the brake ratios persist in the sim under the override)
                // and retry — the one wrong move is releasing a stop we may
                // be mid-way through.
                if on_ground.is_none() {
                    tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
                    continue;
                }
                // Airborne means the taxi is over — release everything, done.
                if on_ground == Some(false) {
                    tracing::info!("speed governor: airborne, taxi hold released");
                    ACTIVE.store(false, Ordering::Relaxed);
                    break;
                }
                // The parking-brake ratio, read every tick: the park HANDLE
                // (past 0.5) set mid-roll is the pilot STOPPING — close the
                // throttle and hand the aircraft back, never chase the target
                // against the brake at taxi power. Below that, the ratio can
                // be the pilot's brake TOGGLE (regular effort ~0.36), which
                // feeds the pause logic further down.
                let pb = xp.get_float(PARK_BRAKE).await.unwrap_or(0.0);
                if pb > 0.5 {
                    tracing::info!(
                        "speed governor: parking brake set, closing throttle, standing down"
                    );
                    glide(&xp, &taxi_lever(), 0.0, 400).await;
                    ACTIVE.store(false, Ordering::Relaxed);
                    break;
                }
                // Brake-paused: idle throttle, hands off, waiting on the
                // brakes — pedals AND the toggle latch, whichever the pilot
                // used. Fully released rolls the taxi again; a hand advancing
                // the throttle meanwhile is the pilot taking the aircraft —
                // release for good.
                if paused {
                    let lb = xp.get_float(LEFT_BRAKE).await.unwrap_or(0.0);
                    let rb = xp.get_float(RIGHT_BRAKE).await.unwrap_or(0.0);
                    if lb.max(rb).max(pb) > PILOT_BRAKE_RESUME {
                        if xp.get_float(&taxi_lever()).await.is_ok_and(|t| t > 0.15) {
                            tracing::info!(
                                "speed governor: throttle advanced while brake-paused, releasing"
                            );
                            ACTIVE.store(false, Ordering::Relaxed);
                            break;
                        }
                        tokio::time::sleep(std::time::Duration::from_millis(200)).await;
                        continue;
                    }
                    paused = false;
                    last_cmd = None;
                    tracing::info!("speed governor: brakes released, resuming taxi");
                    if let Some(v) = &voice {
                        v.say(
                            crate::voice::Priority::Advisory,
                            format!("Rolling again, {target:.0} knots."),
                        );
                    }
                }
                // The pilot on the brakes — BOTH toe pedals, or the brake
                // toggle's parking-brake latch — is a PAUSE, not a hand-back:
                // throttle to idle and wait for the brakes (it used to stand
                // down for good, and every brake press cost a "go" to get the
                // taxi back — resume-on-release asked for 2026-07-13; the
                // toggle went entirely unseen and got throttled against,
                // 2026-07-14). A single pedal is differential steering
                // (turning the aircraft), which the hold must ride through.
                // Only believed once every crew brake command is quiet — the
                // held stop rides the same parking-brake ratio while active.
                if !braking && super::brake_cmd_quiet() {
                    let lb = xp.get_float(LEFT_BRAKE).await.unwrap_or(0.0);
                    let rb = xp.get_float(RIGHT_BRAKE).await.unwrap_or(0.0);
                    if pilot_took_the_brakes(lb, rb, pb, braking) {
                        tracing::info!(
                            brake = lb.max(rb).max(pb),
                            "speed governor: pilot braking, throttle idle, paused"
                        );
                        glide(&xp, &taxi_lever(), 0.0, 300).await;
                        last_cmd = Some(0.0);
                        paused = true;
                        if let Some(v) = &voice {
                            v.say(
                                crate::voice::Priority::Advisory,
                                "Brakes yours, throttle's idle. I'll roll again when you're off them.",
                            );
                        }
                        continue;
                    }
                }
                if let (Ok(gs_ms), Ok(thr)) = (
                    xp.get_float(GS_MS).await,
                    xp.get_float(&taxi_lever()).await,
                ) {
                    if let Some(lc) = last_cmd.filter(|lc| (thr - lc).abs() > 0.15) {
                        if override_pending {
                            tracing::info!(thr, lc, "speed governor: pilot override, releasing");
                            ACTIVE.store(false, Ordering::Relaxed);
                            break;
                        }
                        override_pending = true;
                        tracing::info!(
                            thr,
                            lc,
                            "speed governor: throttle deviated, challenging once"
                        );
                        let _ = xp.set_float(&taxi_lever(), lc).await;
                        tokio::time::sleep(std::time::Duration::from_millis(250)).await;
                        continue;
                    }
                    override_pending = false;
                    let gs_kts = gs_ms * MS_TO_KTS;
                    *ACTUAL.lock().unwrap_or_else(|e| e.into_inner()) = gs_kts;
                    let now_t = std::time::Instant::now();
                    let raw_trend = match prev_gs {
                        Some((last_gs, last_t)) => {
                            let dt = now_t.duration_since(last_t).as_secs_f64();
                            if (0.05..=2.0).contains(&dt) {
                                (gs_kts - last_gs) / dt
                            } else {
                                0.0
                            }
                        }
                        None => 0.0,
                    };
                    prev_gs = Some((gs_kts, now_t));
                    gs_trend = gs_trend * 0.7 + raw_trend * 0.3;
                    let (new_thr, brake) = match crate::aircraft::taxi_beta_max() {
                        Some(b) => taxi_control_beta(target, gs_kts, thr, gs_trend, b),
                        None => taxi_control(target, gs_kts, thr, gs_trend),
                    };
                    let wrote = new_thr.is_some() || brake > 0.0 || braking;
                    if let Some(t) = new_thr {
                        glide(&xp, &taxi_lever(), t, 200).await;
                        last_cmd = Some(t);
                    }
                    // Brakes go through the plugin's proportional channel —
                    // BOTH toe ratios, equally, every frame — with the demand
                    // slewed so pedals squeeze instead of stab ("braking is
                    // TOO jerky", 2026-07-20: every stop was the binary key
                    // at regular effort in one frame). Stops and trims share
                    // the smooth channel now; the binary brake key survives
                    // only as the fallback for a missing/old plugin. Never
                    // silent on failure: a swallowed brake error hid the
                    // broken override for days of flights.
                    let now_bt = std::time::Instant::now();
                    let brake_dt = last_brake_t.map_or(0.1, |t: std::time::Instant| {
                        now_bt.duration_since(t).as_secs_f64()
                    });
                    last_brake_t = Some(now_bt);
                    brake_out = brake_slew(brake_out, brake, brake_dt);
                    // The brake trace was invisible: "braking is jerky" has
                    // been reported three times and diagnosed by feel every
                    // time. Log only while anything is happening.
                    if brake_out > 0.005 || brake > 0.0 {
                        crate::blackbox::record(
                            "speedhold",
                            serde_json::json!({
                                "event": "brake", "cmd": brake, "out": brake_out,
                                "gs": gs_kts, "target": target, "trend": gs_trend,
                                "lever": new_thr.unwrap_or(thr),
                            }),
                        );
                    }
                    if brake_out > 0.005 {
                        let mut smoothed = false;
                        brake_released = false;
                        if !trim_fallback {
                            match xp.set_float(TAXI_BRAKE, brake_out).await {
                                Ok(()) => smoothed = true,
                                Err(e) => {
                                    trim_fallback = true;
                                    tracing::warn!(
                                        "speed governor: proportional brake unavailable \
                                         ({e:#}), falling back to the brake key"
                                    );
                                }
                            }
                        }
                        if !smoothed && brake > 0.0 {
                            match xp.run_command_for(BRAKES_REGULAR, BRAKE_HOLD_SECS).await {
                                Ok(()) => super::note_brake_cmd(),
                                Err(e) => {
                                    tracing::warn!("speed governor: brake command failed: {e:#}")
                                }
                            }
                        }
                    } else if should_release_brake_channel(braking, brake_released)
                        && !trim_fallback
                    {
                        // Release the trim channel promptly instead of riding
                        // out its expiry; a failure here is covered by that
                        // same expiry, so it's only worth a debug line.
                        //
                        // The `!brake_released` half is the ARM-TIME clear (see
                        // the flag): expiry frees the override, never the
                        // ratios, so the first quiet tick of every governor
                        // must write the zero that the last one's stop left
                        // behind. Without it a fresh governor's `braking`
                        // starts false, this branch never fires, and the drive
                        // rolls against brakes it did not set and cannot see.
                        if let Err(e) = xp.set_float(TAXI_BRAKE, 0.0).await {
                            tracing::debug!("speed governor: brake release write failed: {e:#}");
                        }
                        brake_released = true;
                        brake_out = 0.0;
                    }
                    braking = brake_out > 0.005;
                    tokio::time::sleep(std::time::Duration::from_millis(if wrote {
                        100
                    } else {
                        300
                    }))
                    .await;
                } else {
                    tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
                }
            }
        }
    }
    tracing::info!("speed governor stopped");
    life("governor_stopped");
    // A lever left in beta goes back to idle: the next thing on this lever
    // may be a takeoff, and a roll must never start with the prop in beta.
    if crate::aircraft::taxi_beta_max().is_some() && last_cmd.is_some_and(|c| c < 0.0) {
        let _ = xp.set_float(BETA_LEVER, 0.0).await;
    }
    // The held brake COMMAND expires by itself within BRAKE_HOLD_SECS, and
    // that is all this used to say. The proportional CHANNEL is the other
    // half and it does not clean up after itself: its override expires, its
    // ratios do not (see `brake_released`). So a governor that stops while
    // squeezing hands the pilot a boot on the pedals they cannot find or
    // release.
    //
    // Released here only when something ELSE is holding the aircraft — the
    // parking brake, which is exactly what every caller sets at the end of a
    // drive ("Holding position, brakes set"). Zeroing it unconditionally
    // would be the opposite failure, an aircraft rolling off a stand at idle
    // with nobody on the brakes, which this project has already paid for
    // twice. Still squeezing with no park brake set, the squeeze stays and is
    // logged: the arm-time clear above means it can no longer poison the next
    // drive, and a warning beats a silent latch.
    if braking && !trim_fallback {
        let held = xp.get_float(PARK_BRAKE).await.unwrap_or(0.0) > 0.5;
        if held {
            if let Err(e) = xp.set_float(TAXI_BRAKE, 0.0).await {
                tracing::debug!("speed governor: exit brake release failed: {e:#}");
            }
        } else {
            tracing::warn!(
                brake_out,
                "speed governor: stopped mid-squeeze with no parking brake — \
                 leaving the brakes on rather than letting it roll"
            );
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn idle_and_still_fast_is_said_once_after_it_lasts() {
        // LOWI: 234 kt against 180 at idle, speed not coming down.
        let mut w = super::IdleStillFast::default();
        assert_eq!(w.tick(0.0, 234.0, 180.0, 0.0, 0.5), None);
        assert_eq!(w.tick(10.0, 234.0, 180.0, 0.0, 0.5), None, "not yet 15 s");
        let line = w.tick(16.0, 234.0, 180.0, 0.0, 0.5).expect("said");
        assert!(line.contains("54 knots fast"), "{line}");
        assert_eq!(w.tick(30.0, 234.0, 180.0, 0.0, 0.5), None, "once per episode");
        // Back near the target: the episode ends, a new one can be said.
        assert_eq!(w.tick(40.0, 182.0, 180.0, 0.0, -1.0), None);
        // Slowing at idle is the throttle working: nothing to say.
        let mut s = super::IdleStillFast::default();
        for t in 0..30 {
            assert_eq!(s.tick(t as f64, 220.0, 180.0, 0.0, -1.5), None);
        }
        // Not at idle: the hold still has lever to use.
        let mut p = super::IdleStillFast::default();
        for t in 0..30 {
            assert_eq!(p.tick(t as f64, 220.0, 180.0, 0.3, 0.5), None);
        }
    }


    /// 255 KIAS is the 737's cruise hold and correct at altitude — M0.78 at
    /// FL360. Armed at 1,031 ft on departure and held, it was a five knot
    /// speed bust for the whole climb (BIKF-EGPH, 2026-09-20).
    #[test]
    fn the_cruise_speed_is_capped_below_ten_thousand() {
        assert_eq!(airspace_limited(1_031.0, 255.0), 250.0, "the departure");
        assert_eq!(airspace_limited(9_999.0, 255.0), 250.0, "right up to it");
        assert_eq!(airspace_limited(10_000.0, 255.0), 255.0, "and free above");
        assert_eq!(airspace_limited(35_000.0, 255.0), 255.0, "the cruise");
        // Inert on anything that cannot reach the limit — every piston here.
        assert_eq!(airspace_limited(1_000.0, 110.0), 110.0);
        // A failed altitude read is no measurement: do not invent a cap.
        assert_eq!(airspace_limited(f64::NAN, 255.0), 255.0);
    }

    /// An altimeter 50 ft either side of `level`, the wander of a level
    /// held by an autopilot: deterministic, both extremes reached.
    fn wander(level: f64, n: usize) -> impl Iterator<Item = f64> {
        (0..n).map(move |i| level + 50.0 * ((i as f64) * 1.7).sin())
    }

    fn changes(v: &[f64]) -> usize {
        v.windows(2).filter(|w| (w[0] - w[1]).abs() > 0.5).count()
    }

    /// A climb that levels AT 10,000 ft (a plan level on the line) with the
    /// altimeter wandering +-50 ft: the stateless rule flips the target
    /// 250/300 on every wander; the latch holds the cap, and a level just
    /// past the lift band changes it once, never back. #154, the shape of
    /// D156J's minimum (#153).
    #[test]
    fn a_level_at_ten_thousand_never_flips_the_cap() {
        let climb: Vec<f64> = (0..200)
            .map(|i| 8_000.0 + 10.0 * i as f64)
            .chain(wander(10_000.0, 600))
            .collect();
        let old: Vec<f64> = climb.iter().map(|a| airspace_limited(*a, 300.0)).collect();
        assert!(changes(&old) > 50, "the stateless rule hunts: {}", changes(&old));
        let mut latch = AirspaceLimit::default();
        let new: Vec<f64> = climb.iter().map(|a| latch.limit(*a, 300.0)).collect();
        assert_eq!(changes(&new), 0, "capped from the climb, held at the level");
        assert!(new.iter().all(|k| *k == 250.0));

        // Levelled on the lift band's edge: one change, never back.
        let mut latch = AirspaceLimit::default();
        let over: Vec<f64> = (0..230)
            .map(|i| 8_000.0 + 10.0 * i as f64)
            .chain(wander(10_300.0, 600))
            .map(|a| latch.limit(a, 300.0))
            .collect();
        assert_eq!(changes(&over), 1, "lifted once at the band");
        assert_eq!(*over.last().unwrap(), 300.0);
    }

    /// A descent from the cruise through 10,000 ft with the same +-50 ft
    /// noise, levelling on the line, and one carried on down: the cap comes
    /// on once, at the first reading under 10,000 (never above, never
    /// late), and never alternates. Then back up through it: lifted once.
    #[test]
    fn a_descent_through_ten_thousand_caps_once() {
        let noise = |i: usize| 50.0 * ((i as f64) * 1.7).sin();
        let mut latch = AirspaceLimit::default();
        let alts: Vec<f64> = (0..200)
            .map(|i| 12_000.0 - 10.0 * i as f64)
            .chain(wander(10_000.0, 600))
            .collect();
        let got: Vec<f64> = alts.iter().map(|a| latch.limit(*a, 300.0)).collect();
        assert_eq!(changes(&got), 1, "capped once at the first reading under the line");
        let first = got.iter().position(|k| *k == 250.0).unwrap();
        assert!(alts[first] < SPEED_LIMIT_ALT_FT);
        assert!(alts[..first].iter().all(|a| *a >= SPEED_LIMIT_ALT_FT));

        let mut latch = AirspaceLimit::default();
        let alts: Vec<f64> = (0..700)
            .map(|i| 12_000.0 - 10.0 * i as f64 + noise(i))
            .collect();
        let got: Vec<f64> = alts.iter().map(|a| latch.limit(*a, 300.0)).collect();
        assert_eq!(changes(&got), 1);
        let first = got.iter().position(|k| *k == 250.0).unwrap();
        assert!(alts[..first].iter().all(|a| *a >= SPEED_LIMIT_ALT_FT));

        let alts: Vec<f64> = (0..700)
            .map(|i| 5_000.0 + 10.0 * i as f64 + noise(i))
            .collect();
        let got: Vec<f64> = alts.iter().map(|a| latch.limit(*a, 300.0)).collect();
        assert_eq!(changes(&got), 1);
        let lifted = got.iter().position(|k| *k == 300.0).unwrap();
        assert!(alts[lifted] >= SPEED_LIMIT_ALT_FT + SPEED_LIMIT_LIFT_BAND_FT);
    }

    /// The latch's edges: seeded by the first reading on the stateless rule,
    /// a failed read keeps what it knew, inert below 250 kt.
    #[test]
    fn the_airspace_latch_seeds_and_survives_a_failed_read() {
        let mut l = AirspaceLimit::default();
        assert_eq!(l.limit(f64::NAN, 300.0), 300.0, "nothing known: no invented cap");
        assert_eq!(l.capped(), None);
        assert_eq!(l.limit(10_100.0, 300.0), 300.0, "seeded above the line: free");
        assert_eq!(l.limit(f64::NAN, 300.0), 300.0, "a lost read keeps it");
        assert_eq!(l.limit(9_990.0, 300.0), 250.0);
        assert_eq!(l.limit(f64::NAN, 300.0), 250.0, "and keeps the cap too");
        assert_eq!(l.limit(10_200.0, 300.0), 250.0, "inside the band: still capped");
        assert_eq!(l.limit(9_000.0, 110.0), 110.0, "inert on a piston");
    }
    use super::*;

    /// LIPE-LIPK, 2026-09-25: "set speed to 110 knots" in the cruise was
    /// refused with "the approach has the throttle". A number goes to the
    /// follower, and the readback says the number flown and why.
    #[test]
    fn a_speed_for_the_follower_is_read_back_as_flown() {
        assert_eq!(follower_speed_line(110.0, 110.0), "110 knots, on the flight plan.");
        assert!(follower_speed_line(50.0, 58.0).contains("floor"));
        assert!(follower_speed_line(200.0, 148.0).contains("too fast"));
    }

    #[test]
    fn brakes_squeeze_on_and_let_go_quickly() {
        // A demanded 0.5 park from nothing arrives over ~a second of
        // squeeze, never in one frame (the head-nod at every stop).
        let first = brake_slew(0.0, 0.5, 0.1);
        assert!(first <= 0.06, "first tick is a touch, got {first}");
        let mut b = 0.0;
        let mut ticks = 0;
        while b < 0.49 && ticks < 50 {
            b = brake_slew(b, 0.5, 0.1);
            ticks += 1;
        }
        assert!(
            (10..=18).contains(&ticks),
            "~1.5 s to full park, took {ticks} ticks"
        );
        // Release is quick — letting go is not a jerk.
        let released = brake_slew(0.5, 0.0, 0.1);
        assert!(released < 0.4, "{released}");
        // Small trims pass through unslewed.
        assert_eq!(brake_slew(0.2, 0.22, 0.1), 0.22);
    }

    /// #169, LEMG 30 (sortie 1790633832-full_auto, 2026-09-28): the clean
    /// A330 circuit ran 227.5 to 265 kt from the crosswind's level-off, the
    /// lever at 1.0 through the climb the limiter had cut and still 0.93
    /// when the autopilot's own capture took the level, pulled 0.04 a tick.
    /// Replayed on the recorded climb (vertical speed, s from the sortie
    /// start, every other state sample) through a thrust plant of the
    /// A330's order: A kt/s per unit of lever over a trim, through a spool
    /// lag, less gravity's share of the climb (the fit on the clean legs:
    /// 2.5 kt/s, 0.125, 4.6 s; the idle decay after the level-off says
    /// nearer 4). On every plant from 2.5 to 5 kt/s and 2 to 7 s the
    /// A330's governor (`speed_governor:`) peaks lower and settles tighter
    /// than the shared law, and never 10 kt over the pattern speed.
    #[test]
    fn the_a330_governor_catches_the_level_off() {
        const VS: [(f64, f64); 42] = [
            (177.1, 787.0), (180.2, 366.0), (183.6, 146.0), (186.9, 117.0), (190.5, 346.0),
            (194.1, 706.0), (197.8, 944.0), (201.4, 699.0), (204.9, 218.0), (208.6, -248.0),
            (211.8, -494.0), (214.7, -526.0), (218.1, -363.0), (221.4, -177.0), (225.0, -40.0),
            (228.7, 56.0), (232.0, 124.0), (235.4, 112.0), (239.1, 136.0), (242.5, 347.0),
            (245.5, 534.0), (248.6, 607.0), (252.1, 700.0), (255.6, 778.0), (259.3, 825.0),
            (263.1, 791.0), (266.3, 816.0), (269.8, 828.0), (273.3, 789.0), (276.9, 695.0),
            (280.5, 477.0), (284.1, 345.0), (287.4, 284.0), (290.8, 298.0), (294.2, 344.0),
            (297.5, 325.0), (300.6, 275.0), (303.9, 233.0), (307.2, 191.0), (310.9, 155.0),
            (314.2, -110.0), (317.8, -376.0),
        ];
        let gov = crate::aircraft::load("airbus_a330_300")
            .and_then(|p| p.speed_governor)
            .and_then(crate::aircraft::SpeedGovernorProfile::valid)
            .expect("the A330 declares its governor");
        let vs_at = |t: f64| {
            let i = VS.iter().position(|p| p.0 > t).unwrap_or(VS.len() - 1).max(1);
            let (a, b) = (VS[i - 1], VS[i]);
            a.1 + (b.1 - a.1) * ((t - a.0) / (b.0 - a.0)).clamp(0.0, 1.0)
        };
        let fly = |g: Option<crate::aircraft::SpeedGovernorProfile>, a: f64, trim: f64, lag: f64| {
            let (want, tick, h) = (227.5, 0.65, 0.05);
            let (mut ias, mut lever, mut eff) = (223.2_f64, 0.66_f64, 0.66_f64);
            let (mut t, mut next, mut trend, mut prev) = (177.1, 177.1, 0.0_f64, 223.2_f64);
            let (mut peak, mut sq, mut n) = (0.0_f64, 0.0, 0);
            while t < 317.8 {
                if t >= next {
                    next += tick;
                    trend = 0.6 * trend + 0.4 * (ias - prev) / tick;
                    prev = ias;
                    lever = (lever + throttle_adjust_with(want - ias, trend, g)).clamp(0.0, 1.0);
                }
                eff += (lever - eff) * h / lag;
                let gamma = vs_at(t) / (ias * 101.27);
                ias += (a * (eff - trim) - 0.545 * (ias - 220.0) / 100.0 - 19.07 * gamma) * h;
                t += h;
                peak = peak.max(ias);
                if t > 225.0 {
                    sq += (ias - want).powi(2);
                    n += 1;
                }
            }
            (peak, (sq / n as f64).sqrt())
        };
        for (a, trim) in [(2.5, 0.125), (4.0, 0.18), (5.0, 0.2)] {
            for lag in [2.0, 4.6, 7.0] {
                let (old_peak, old_rms) = fly(None, a, trim, lag);
                let (peak, rms) = fly(Some(gov), a, trim, lag);
                assert!(peak < old_peak, "{a} kt/s, {lag} s: {peak:.1} vs {old_peak:.1}");
                assert!(rms <= old_rms, "{a} kt/s, {lag} s: {rms:.1} vs {old_rms:.1}");
                assert!(peak < 227.5 + 12.0, "{a} kt/s, {lag} s: {peak:.1}");
            }
        }
        // No declared governor is the shared law, exactly.
        assert_eq!(throttle_adjust_with(-5.0, 4.0, None), throttle_adjust(-5.0, 4.0));
        // At the capture (+190.5 s: 222.3 kt, 2.6 kt/s up) the shared law
        // pulled the polite 0.015; the A330's pulls harder.
        assert_eq!(throttle_adjust(5.2, 2.6), -0.015);
        assert!(throttle_adjust_with(5.2, 2.6, Some(gov)) < -0.03);
    }

    /// #169, LEMG 30 (sortie 1790698132-full_auto, 2026-09-29): with CONF 1
    /// out and 212.5 wanted, the climb collapsed from 2,468 fpm to level in
    /// ten seconds (the recorded `vs`, +143.8 to +168.6 s) with the lever at
    /// 1.0, and the crosswind peaked 224.4 kt at +160.4 s. Replayed through
    /// the same thrust plants as the clean level-off, the deeper pull peaks
    /// lower on every plant with the fast engines and never higher; the old
    /// law is this one with the pull capped at the urgent step. On the plant
    /// that reproduces the flight (4 kt/s, 4.6 s: 221.2 at +164.3) it peaks
    /// under 219.
    #[test]
    fn the_a330_governor_pulls_harder_far_past_the_target() {
        const VS: [(f64, f64); 16] = [
            (143.8, 2468.0), (145.5, 1931.0), (147.1, 1601.0), (148.6, 1059.0),
            (150.5, 651.0), (151.9, 323.0), (153.6, 74.0), (155.3, -18.0), (157.1, 8.0),
            (158.7, 70.0), (160.4, 146.0), (162.0, 165.0), (163.7, 188.0), (165.3, 182.0),
            (167.1, 208.0), (168.6, 231.0),
        ];
        let gov = crate::aircraft::load("airbus_a330_300")
            .and_then(|p| p.speed_governor)
            .and_then(crate::aircraft::SpeedGovernorProfile::valid)
            .expect("the A330 declares its governor");
        let vs_at = |t: f64| {
            let i = VS.iter().position(|p| p.0 > t).unwrap_or(VS.len() - 1).max(1);
            let (a, b) = (VS[i - 1], VS[i]);
            a.1 + (b.1 - a.1) * ((t - a.0) / (b.0 - a.0)).clamp(0.0, 1.0)
        };
        let fly = |deep: bool, a: f64, trim: f64, lag: f64| {
            let (want, tick, h) = (212.5, 0.75, 0.05);
            let (mut ias, mut lever, mut eff) = (180.9_f64, 1.0_f64, 1.0_f64);
            let (mut t, mut next, mut trend, mut prev) = (143.8, 143.8, 2.5_f64, 180.9_f64);
            let mut peak = 0.0_f64;
            while t < 168.6 {
                if t >= next {
                    next += tick;
                    trend = 0.6 * trend + 0.4 * (ias - prev) / tick;
                    prev = ias;
                    let mut d = throttle_adjust_with(want - ias, trend, Some(gov));
                    if !deep {
                        d = d.max(-gov.max_step);
                    }
                    lever = (lever + d).clamp(0.0, 1.0);
                }
                eff += (lever - eff) * h / lag;
                let gamma = vs_at(t) / (ias * 101.27);
                ias += (a * (eff - trim) - 0.545 * (ias - 220.0) / 100.0 - 19.07 * gamma) * h;
                t += h;
                peak = peak.max(ias);
            }
            peak
        };
        for (a, trim) in [(2.5, 0.125), (4.0, 0.18), (5.0, 0.2)] {
            for lag in [2.0, 4.6, 7.0] {
                let (old, new) = (fly(false, a, trim, lag), fly(true, a, trim, lag));
                assert!(new <= old + 1e-9, "{a} kt/s, {lag} s: {new:.1} vs {old:.1}");
                if a >= 4.0 {
                    assert!(new < old - 1.0, "{a} kt/s, {lag} s: {new:.1} vs {old:.1}");
                }
            }
        }
        assert!(fly(true, 4.0, 0.18, 4.6) < 219.0);
        // Twice the urgent step at most, and only past 10 kt of projection.
        assert_eq!(throttle_adjust_with(-40.0, 0.0, Some(gov)), -2.0 * gov.max_step);
        assert!((throttle_adjust_with(-11.0, 0.0, Some(gov)) + 0.066).abs() < 1e-9);
        // The shared law never pulls past its urgent step.
        assert_eq!(throttle_adjust(-40.0, 0.0), -0.04);
    }

    #[test]
    fn governor_law_is_gentle_and_deadbanded() {
        assert_eq!(
            throttle_adjust(1.0, 0.0),
            0.0,
            "in the groove, steady → hands off"
        );
        assert_eq!(throttle_adjust(-1.2, 0.5), 0.0);
        assert!(throttle_adjust(3.0, 0.0) > 0.0, "slow → more power");
        assert!(throttle_adjust(-3.0, 0.0) < 0.0, "fast → less power");
        // The anti-stall reflex: on speed but DECAYING fast → power comes in
        // before the error exists at all.
        assert!(
            throttle_adjust(0.0, -2.5) > 0.0,
            "unwinding needle → power now"
        );
        // And rising speed damps the correction rather than piling on.
        assert!(throttle_adjust(3.0, 3.0) < throttle_adjust(3.0, 0.0));
        // Authority is bounded, asymmetrically: overspeed is a comfort issue,
        // stall is not — the slow side gets the bigger clamp.
        assert_eq!(throttle_adjust(100.0, 0.0), 0.04);
        // The fast side pulls with the same urgency it pushes: a flat -0.015
        // let a jet run 125 to 270 knots through a whole circuit (2026-07-28).
        assert_eq!(throttle_adjust(-100.0, 0.0), -0.04);
        // ...but a small overspeed still gets the polite step.
        assert_eq!(throttle_adjust(-5.0, 0.0), -0.015);
        // Building fast counts as urgent even before the error is big.
        assert_eq!(throttle_adjust(-5.0, 4.0), -0.04);
        assert_eq!(
            throttle_adjust(0.0, -10.0),
            0.04,
            "fast decay unlocks urgency"
        );
    }

    /// The takeoff-handoff transient (EGHI Baron, 2026-08-02): the circuit
    /// inherits 131 kt at full power against a 109 kt target, and the old law
    /// was still pulling at 124 with the speed already falling 2.5 kt/s — the
    /// sag ran to 98.7, below blue line, the climb limiter cut to zero, and
    /// the aircraft porpoised at 700 ft AGL. The law must ease off while the
    /// excess is burning and bring power back BEFORE the undershoot.
    #[test]
    fn the_pull_eases_before_the_target_and_power_returns_before_the_sag() {
        // 15 kt fast but plunging toward the target: no more than a polite
        // pull — the plunge is doing the work.
        assert!(
            throttle_adjust(-15.0, -2.5) > -0.015,
            "still pulling hard with the speed already falling"
        );
        // 6 kt fast and plunging: power is already coming BACK.
        assert!(
            throttle_adjust(-5.8, -2.5) > 0.01,
            "the sag is five seconds out — the lever must be moving up now"
        );
        // On target and falling 2 kt/s: full recovery authority, not the
        // polite step the old raw-error gate allowed until 10 kt slow.
        assert!(
            throttle_adjust(1.2, -2.0) > 0.03,
            "recovery urgency must key off the projection"
        );
        // The mirror case stays urgent: near target but accelerating hard is
        // the SF50 runaway, and the projection pulls at the big clamp.
        assert_eq!(throttle_adjust(-5.0, 4.0), -0.04);
    }

    #[test]
    fn spoken_targets_are_sanity_bounded() {
        assert_eq!(clamp_target(70.0), 70.0);
        // LSZS-LILO: the SR22's 189 ceiling, floors 84 clean / 74 landing.
        assert!(spoken_flight_target(885.0, 189.0, Some(84.0), Some(74.0))
            .unwrap_err()
            .contains("189"));
        assert!(spoken_flight_target(17.0, 189.0, Some(74.0), Some(74.0))
            .unwrap_err()
            .contains("74"));
        assert_eq!(
            spoken_flight_target(85.0, 189.0, Some(74.0), Some(74.0)),
            Ok((85.0, None))
        );
        let (t, note) = spoken_flight_target(80.0, 189.0, Some(84.0), Some(74.0)).unwrap();
        assert_eq!(t, 84.0);
        assert!(note.unwrap().contains("84"));
        assert_eq!(
            clamp_target(7.0),
            55.0,
            "mishear can't idle us into a stall"
        );
        assert_eq!(clamp_target(700.0), 250.0, "no Vne on file: the GA cap");
        // A jet's cap is its own Vne less the margin, and the airspace rule
        // below 10,000 ft is applied separately by `flight_target_for_alt`.
        crate::aircraft::publish_speed_ceiling(Some(330.0));
        assert_eq!(
            clamp_target(290.0),
            290.0,
            "an A330's cruise target survives"
        );
        assert_eq!(clamp_target(700.0), 315.0);
        crate::aircraft::publish_speed_ceiling(None);
        assert_eq!(clamp_target(290.0), 250.0);
        assert_eq!(clamp_taxi_target(10.0), 10.0);
        assert_eq!(clamp_taxi_target(60.0), 25.0, "no takeoff-roll 'taxi'");
        assert_eq!(clamp_taxi_target(0.0), 0.0, "hold short is a real target");
    }

    #[test]
    fn taxi_law_breaks_away_then_takes_the_power_off() {
        // Stuck at full lock: past the rolling cap, up to the breakaway one.
        let (thr, _) = taxi_control(8.0, 0.0, 0.35, 0.0);
        assert!(thr.unwrap() > 0.35 && thr.unwrap() <= TAXI_BREAKAWAY_CAP);
        let (thr, _) = taxi_control(8.0, 0.0, TAXI_BREAKAWAY_CAP, 0.0);
        assert_eq!(thr, Some(TAXI_BREAKAWAY_CAP), "never beyond the breakaway cap");
        // Rolling: back under the rolling cap at once.
        let (thr, _) = taxi_control(8.0, 2.0, 0.5, 0.0);
        assert!(thr.unwrap() <= TAXI_ROLLING_CAP);
    }

    #[test]
    fn taxi_law_throttle_then_brakes() {
        // Too slow: power comes up, capped below anything takeoff-like.
        let (thr, brake) = taxi_control(10.0, 4.0, 0.10, 0.0);
        assert!(thr.unwrap() > 0.10 && thr.unwrap() <= 0.35);
        assert_eq!(brake, 0.0);
        // A little fast with power in: close the throttle before the brakes.
        let (thr, brake) = taxi_control(10.0, 12.0, 0.20, 0.0);
        assert!(thr.unwrap() < 0.20);
        assert_eq!(brake, 0.0, "no braking against thrust");
        // WELL past the target: idle now AND brake — the polite step-down
        // let an SR22 run a 10-knot taxi through 20 (2026-07-11).
        let (thr, brake) = taxi_control(10.0, 16.0, 0.20, 0.0);
        assert_eq!(thr, Some(0.0), "deep overspeed cuts to idle immediately");
        assert!(brake > 0.0 && brake <= 0.6, "{brake}");
        // Too fast at idle, deep: brakes, proportionally, bounded.
        let (thr, brake) = taxi_control(10.0, 18.0, 0.02, 0.0);
        assert_eq!(thr, Some(0.0));
        assert!(brake > 0.0 && brake <= 0.6, "{brake}");
        // In the groove: hands off everything, brakes released.
        assert_eq!(taxi_control(10.0, 10.5, 0.15, 0.0), (None, 0.0));
        // Hold short: idle + parked on the brakes even when stopped.
        assert_eq!(taxi_control(0.0, 0.2, 0.0, 0.0), (Some(0.0), 0.5));
        // Still rolling into the stop: light pressure finishes the roll —
        // parked effort at walking pace is the nose-first jerk.
        let (thr, brake) = taxi_control(0.0, 1.0, 0.0, 0.0);
        assert_eq!(thr, Some(0.0));
        assert!(brake > 0.0 && brake < 0.25, "gentle finish: {brake}");
    }

    /// The Evolution idles too fast to taxi (POH: "Don't ride the brakes in
    /// taxi. Ease the throttle into beta instead"): a rolling overspeed goes
    /// into beta with no brake; a stop still brakes; a runaway still brakes;
    /// wanting the pace back comes out of beta.
    #[test]
    fn a_beta_taxi_slows_on_the_propeller_not_the_brakes() {
        // 14 kt at idle for a 10 kt pace: beta, no brake.
        let (lever, brake) = taxi_control_beta(10.0, 14.0, 0.0, 0.0, 0.5);
        assert!(lever.is_some_and(|l| l < 0.0 && l >= -0.5), "{lever:?}");
        assert_eq!(brake, 0.0);
        // A steady overspeed that beta has not bitten walks deeper each tick
        // (LIME: -0.43 held 17.7 kt for a 10 kt taxi).
        let mut l = -0.43;
        for _ in 0..5 {
            l = taxi_control_beta(10.0, 17.7, l, 0.15, 0.8).0.unwrap();
        }
        assert!(l < -0.5, "went deeper: {l}");
        // Once it bites (slowing), it holds the depth.
        let held = taxi_control_beta(10.0, 15.0, -0.6, -0.5, 0.8).0.unwrap();
        assert_eq!(held, -0.6);
        // Deeper each tick, never past the profile's depth.
        let mut l = 0.0;
        for _ in 0..40 {
            l = taxi_control_beta(10.0, 16.0, l, 0.0, 0.5).0.unwrap();
        }
        assert!((-0.5..0.0).contains(&l), "{l}");
        // A stop brakes as before.
        let (_, brake) = taxi_control_beta(0.0, 3.0, 0.0, 0.0, 0.5);
        assert!(brake > 0.0);
        // A runaway, 10 kt over, still earns the brakes on top of beta.
        let (_, brake) = taxi_control_beta(10.0, 21.0, -0.3, 0.0, 0.5);
        assert!(brake > 0.0, "the emergency keeps its brakes");
        // Slow again in beta: out toward idle, no brake.
        let (lever, brake) = taxi_control_beta(10.0, 7.0, -0.3, 0.0, 0.5);
        assert!(lever.unwrap() > -0.3 && lever.unwrap() <= 0.0);
        assert_eq!(brake, 0.0);
        // In the groove at idle: the plain law's answer.
        assert_eq!(taxi_control_beta(10.0, 10.5, 0.15, 0.0, 0.5), (None, 0.0));
    }

    /// The beta law as flown on 2026-09-30 (b6c24d5), for the replays.
    fn beta_law_as_flown(target_kts: f64, gs_kts: f64, lever: f64, trend_kts_s: f64, beta_max: f64) -> (Option<f64>, f64) {
        let beta_max = beta_max.clamp(0.0, 0.9);
        let (thr, brake) = taxi_control(target_kts, gs_kts, lever.max(0.0), trend_kts_s);
        if target_kts < 1.0 {
            return (thr.map(|t| t.max(0.0)).or(Some(0.0)), brake);
        }
        let eff = (target_kts - gs_kts) - trend_kts_s * 2.5;
        if eff > 1.0 && lever < 0.0 {
            return (Some((lever + BETA_STEP_OUT).min(0.0)), 0.0);
        }
        let fast_at_idle = thr.is_some_and(|t| t <= 0.0) && eff < -1.0;
        if brake > 0.0 || fast_at_idle {
            let floor = ((-eff - 1.0) * BETA_PER_KT).clamp(BETA_MIN, beta_max);
            let falling = trend_kts_s < BETA_BITING_KTS_S;
            let next = if lever > -floor {
                (lever - BETA_STEP_IN).max(-floor)
            } else if falling {
                lever
            } else {
                lever - BETA_STEP_IN
            };
            let emergency = -eff >= BETA_BRAKE_KTS;
            return (Some(next.max(-beta_max)), if emergency { brake } else { 0.0 });
        }
        if lever < 0.0 && eff >= -1.0 {
            return (Some((lever + BETA_STEP_IN).min(0.0)), 0.0);
        }
        (thr, brake)
    }

    /// Beta spent, the brakes: the Evolution at idle in full beta rolls on
    /// at 13 to 14 kt. The backtrack's leg one at LILB 03 (1790740292,
    /// 2026-09-30) held 13.2 kt into its stop ramp (6.4 kt asked at 22 m to
    /// go) with no brake, braked first at 7 m to go (the 8 kt emergency),
    /// let go again at 9.6 kt with 3.8 m left, and was still at 9.9 kt past
    /// the end: about 9.5 m past its stop. On a plant fitted to that leg
    /// (idle in full beta settling at 13 kt, 0.04 per second; the brake 12 to
    /// 25 kt/s per unit, the flown 0.19 took 3.7 kt off in 0.8 s) the law as
    /// flown overruns (19 to 27 m on the plant, about 9.5 flown); spent beta
    /// braking stops within 1.5 m of the end, and a 10 kt pace is held
    /// within 1 kt.
    #[test]
    fn spent_beta_brakes_the_stop_ramp_where_it_should() {
        // The logged ticks: 22 m to go, and the first brake at 8.5 m.
        let (lever, brake) = taxi_control_beta(6.4, 13.2, -0.8, -0.05, 0.8);
        assert_eq!(lever, Some(-0.8));
        assert!(brake > 0.15, "{brake}");
        let (_, brake) = taxi_control_beta(5.22, 13.26, -0.8, 0.0, 0.8);
        assert!(brake > 0.15, "{brake}");
        // Not spent (shallow beta): the propeller first, as before.
        assert_eq!(taxi_control_beta(10.0, 14.0, -0.3, 0.0, 0.8).1, 0.0);
        // Inside the band: no brake.
        assert_eq!(taxi_control_beta(10.0, 10.4, -0.8, 0.0, 0.8).1, 0.0);
        assert_eq!(beta_spent_brake(-0.4), 0.0);
        assert!((beta_spent_brake(-20.0) - BETA_SPENT_MAX).abs() < 1e-12);

        // The plant: idle in beta settling at 13 kt (18 out of beta), the
        // lever's power on top; the brake through the governor's slew.
        let run = |k_brake: f64, flown: bool, base: f64, from_m: f64, v0: f64| {
            let beta = 0.8;
            let (mut v, mut lever, mut trend, mut b_out) = (v0, -0.8_f64, 0.0_f64, 0.0_f64);
            let (mut along, mut t, mut last_gov) = (0.0_f64, 0.0_f64, -1.0_f64);
            let (mut worst_over, mut arrived) = (0.0_f64, false);
            let mut prev_v = v;
            let (mut want_lever, mut want_brake) = (lever, 0.0);
            while t < 120.0 {
                let rem = from_m - along;
                if rem <= 1.0 && v < 1.5 {
                    arrived = true;
                }
                let target = if arrived {
                    0.0
                } else {
                    base.min(crate::autotaxi::stop_ramp_kts(rem))
                };
                if rem > 30.0 && t > 20.0 {
                    worst_over = worst_over.max(v - base);
                }
                if t - last_gov >= 0.55 {
                    let dt = if last_gov < 0.0 { 0.55 } else { t - last_gov };
                    trend = trend * 0.7 + 0.3 * (v - prev_v) / dt;
                    prev_v = v;
                    last_gov = t;
                    let (l, b) = if flown {
                        beta_law_as_flown(target, v, lever, trend, beta)
                    } else {
                        taxi_control_beta(target, v, lever, trend, beta)
                    };
                    if let Some(l) = l {
                        want_lever = l;
                    }
                    want_brake = b;
                }
                let dt = 0.05;
                lever += (want_lever - lever).clamp(-dt / 0.2, dt / 0.2);
                b_out = brake_slew(b_out, want_brake, dt);
                let v_eq = if lever < 0.0 { 18.0 + 6.25 * lever } else { 18.0 + 40.0 * lever };
                v = (v - (0.04 * (v - v_eq) + k_brake * b_out) * dt).max(0.0);
                along += v / MS_TO_KTS * dt;
                t += dt;
                if arrived && v == 0.0 {
                    break;
                }
            }
            (along - from_m, worst_over)
        };
        for k in [12.0, 18.0, 25.0] {
            // The leg's last 100 m from 13.9 kt, a 14 kt pace, as flown.
            let (flown_past, _) = run(k, true, 14.0, 100.0, 13.9);
            let (past, _) = run(k, false, 14.0, 100.0, 13.9);
            // The Evolution's 10 kt backtrack, 520 m from 13.5 kt.
            let (past10, over) = run(k, false, 10.0, 520.0, 13.5);
            eprintln!("k {k}: as flown {flown_past:.1} m past; now {past:.1}, at 10 kt {past10:.1} ({over:.1} kt over)");
            assert!(flown_past > 5.0, "k {k}: the law as flown stopped {flown_past:.1} m past");
            assert!(past.abs() < 1.5, "k {k}: {past:.1} m past");
            assert!(past10.abs() < 1.5, "k {k}: {past10:.1} m past");
            assert!(over < 1.0, "k {k}: {over:.1} kt over the pace");
        }
    }

    #[test]
    fn taxi_law_never_brakes_a_speed_trim() {
        // The limit-cycle fix (6.6 ↔ 11.9 kt sawtooth, 2026-07-14): one or
        // two knots over at idle is NOT a braking problem — coast it off.
        let (thr, brake) = taxi_control(10.0, 11.8, 0.02, 0.0);
        assert_eq!(brake, 0.0, "no brake stab to shave two knots");
        assert_eq!(thr, Some(0.0));
        // But a STOP still brakes all the way to the halt once the
        // throttle's closed — idle creep never finishes a stop.
        let (thr, brake) = taxi_control(0.0, 2.5, 0.0, 0.0);
        assert_eq!(thr, Some(0.0));
        assert!(brake > 0.0, "a stop stays on the brakes to the halt");
        // And the deep brake ramps from the boundary instead of stabbing at
        // 0.2 the instant the branch is crossed.
        let (_, just_over) = taxi_control(10.0, 13.2, 0.0, 0.0);
        assert!(just_over < 0.2, "gentle at the boundary: {just_over}");
        let (_, way_over) = taxi_control(10.0, 18.0, 0.0, 0.0);
        assert!(way_over > just_over, "deeper overspeed brakes harder");
    }

    // The 6 kt line-up limit cycle (Malaga, 2026-09-11): 6.9 kt building
    // 0.9 kt/s at a 6 kt target is a trim, not a stop.
    #[test]
    fn taxi_law_holds_a_slow_pace_without_stopping() {
        let (thr, brake) = taxi_control(6.0, 6.87, 0.18, 0.87);
        assert_eq!(brake, 0.0, "a one-knot build never earns the brakes");
        let thr = thr.expect("power comes off a touch");
        assert!(thr < 0.18 && thr > 0.1, "trimmed, not idled: {thr}");
        // The trim has taken (speed falling): hands off, the lever stays.
        assert_eq!(taxi_control(6.0, 6.9, 0.14, -0.6), (None, 0.0));
        // Actually two knots over and still building: the old answer stands.
        let (thr, brake) = taxi_control(6.0, 8.3, 0.14, 0.9);
        assert_eq!(thr, Some(0.0));
        assert!(brake > 0.0);
    }

    #[test]
    fn taxi_law_anticipates_the_trend() {
        // Decaying toward the target at idle: power comes back BEFORE the
        // speed falls through it — the other half of the limit-cycle fix.
        let steady = taxi_control(10.0, 9.7, 0.05, 0.0);
        assert_eq!(steady, (None, 0.0), "on pace and steady: hands off");
        let (thr, brake) = taxi_control(10.0, 9.7, 0.05, -1.5);
        assert!(thr.unwrap() > 0.05, "decaying through target: power early");
        assert_eq!(brake, 0.0);
        // Accelerating gently through the target with power in: trim it
        // early too, still no brakes.
        let (thr, brake) = taxi_control(10.0, 10.5, 0.20, 0.8);
        assert!(
            thr.unwrap() < 0.20,
            "building through target: power off early"
        );
        assert_eq!(brake, 0.0);
        // A genuine SURGE (downhill, +1.5 kt/s) reads as deep overspeed
        // before the speed gets there: idle and brakes, by design.
        let (thr, brake) = taxi_control(10.0, 10.5, 0.20, 1.5);
        assert_eq!(thr, Some(0.0));
        assert!(brake > 0.0, "a surge earns the brakes early");
    }

    #[test]
    fn manual_toe_brakes_make_the_governor_yield() {
        // BOTH pedals down while we're NOT braking (override off → the ratios
        // are the pilot's pedals): the pilot is slowing us, yield.
        assert!(
            pilot_took_the_brakes(0.2, 0.2, 0.0, false),
            "both pedals = slowing"
        );
        assert!(
            pilot_took_the_brakes(0.5, 0.2, 0.0, false),
            "both engaged, uneven, still slowing"
        );
        // A SINGLE pedal is differential braking = steering a turn (the only
        // low-speed steering on a castoring nosewheel). The hold must ride
        // through it, not cut throttle to 0 the moment the pilot turns.
        assert!(
            !pilot_took_the_brakes(0.4, 0.0, 0.0, false),
            "left pedal = steering, ride through"
        );
        assert!(
            !pilot_took_the_brakes(0.0, 0.4, 0.0, false),
            "right pedal = steering, ride through"
        );
        // Pedal rest / sensor noise below the threshold is NOT a takeover.
        assert!(!pilot_took_the_brakes(0.05, 0.03, 0.0, false));
        assert!(!pilot_took_the_brakes(0.0, 0.0, 0.02, false));
        // While the governor is braking (override on) the ratios are OUR own
        // command, not the pilot's — never read them as a takeover, or we'd
        // stand ourselves down the instant we touched the brakes.
        assert!(
            !pilot_took_the_brakes(0.6, 0.6, 0.0, true),
            "that's our brake, not theirs"
        );
    }

    #[test]
    fn brake_toggle_latch_makes_the_governor_yield() {
        // The keyboard/joystick brake TOGGLE never moves the pedals: it
        // latches the parking-brake ratio at regular effort (~0.36 measured
        // live, 2026-07-14). That must read as the pilot braking — the
        // governor throttled straight against it before.
        assert!(
            pilot_took_the_brakes(0.0, 0.0, 0.36, false),
            "toggle latch = slowing"
        );
        // A partially-set park handle below the stand-down threshold is
        // braking all the same.
        assert!(pilot_took_the_brakes(0.0, 0.0, 0.3, false));
        // The decay tail after our own held command expires (~0.02 within a
        // second) is noise, not a takeover — and the fresher, higher tail is
        // excluded by the shared brake_cmd_quiet clock, not this threshold.
        assert!(
            !pilot_took_the_brakes(0.0, 0.0, 0.02, false),
            "decay tail is not the pilot"
        );
        // While the governor itself is braking, nothing reads as the pilot.
        assert!(
            !pilot_took_the_brakes(0.0, 0.0, 0.36, true),
            "our stop, not their toggle"
        );
    }

    /// No drive may start against the last drive's brakes. The channel's
    /// override expires, its toe ratios do not, and nothing re-drives them on
    /// an airframe with no toe-brake axis — so the release has to be WRITTEN,
    /// and a fresh governor is exactly the case that used to skip it.
    #[test]
    fn a_fresh_governor_clears_the_last_ones_brakes() {
        // The bug, in one line: `braking` starts false on arm, so gating the
        // release on it alone means the very first tick writes nothing and the
        // taxi rolls against an inherited 0.5 (Malaga, 2026-08-03).
        assert!(
            should_release_brake_channel(false, false),
            "arm-time clear: never inherit the last governor's squeeze"
        );
        // Once cleared, a quiet tick stays quiet — this is not an every-tick
        // write, and the API load report is the reason to care.
        assert!(!should_release_brake_channel(false, true));
        // Coming off a squeeze always releases promptly, cleared or not.
        assert!(should_release_brake_channel(true, true));
        assert!(should_release_brake_channel(true, false));
    }
}
