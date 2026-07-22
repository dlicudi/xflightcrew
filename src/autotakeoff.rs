//! Auto takeoff — the copilot flies the takeoff roll and initial climb.
//!
//! The departure-side sibling of [`crate::autoland`] and [`crate::autotaxi`],
//! composed from the same parts: the runway comes from apt.dat (autoland's
//! parser, fitted to where the aircraft is actually pointing), the roll is
//! steered down the centreline through the plugin's `xcopilot/taxi_steer`
//! channel (rudder + nosewheel under `override_joystick_heading`, expiring
//! within a second of writes stopping), rotation and the initial climb are
//! ELEVATOR through the plugin's `xcopilot/pitch` channel (the AP's VS mode
//! syncs to the current rate at engage — zero on the runway — so it can
//! never rotate), and the AP takes the ESTABLISHED climb: heading bug on
//! runway heading, VS mode holding a speed-protected climb, full throttle
//! throughout. At the handoff height the copilot says "your aircraft" and
//! leaves the autopilot flying, exactly the state a real crew hands over in.
//!
//! Handback, every path instant:
//! - "Abort takeoff" / "reject" is a REJECT: throttle to idle, one expiring
//!   brake pulse, steering released. The tool arms [`REJECT`], the loop acts.
//! - "My controls" (the release path in main) just clears [`ACTIVE`]: the
//!   loop stands down silently, steering expires, and the throttle stays
//!   where it is — the pilot who takes over mid-roll wants the power set.
//! - The pilot's FEET win in-plugin (the `steer_yield` latch): the copilot
//!   calls "your aircraft, power is set" and the takeoff continues as a
//!   manual one.
//! - The pilot pulling the THROTTLE back is a pilot reject: challenged once
//!   (hardware-axis jitter re-drives a lever the hand never touched), then
//!   honored — hands off everything, no re-command.
//!
//! Safety gates in the loop, not just at arm time: not accelerating means
//! reject; runway running out below rotate speed means reject; the sim
//! frozen (pause/replay) freezes the loop with it. The laws are pure
//! functions and battle-test as plain `cargo test`.

use std::sync::atomic::{AtomicBool, Ordering};

use crate::autoland::{along_track_nm, cross_track_nm, heading_diff, true_to_mag, Runway};
use crate::tools::autopilot::ApConfig;
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

const LAT: &str = "sim/flightmodel/position/latitude";
const LON: &str = "sim/flightmodel/position/longitude";
const PSI_TRUE: &str = "sim/flightmodel/position/psi";
const GS_MS: &str = "sim/flightmodel/position/groundspeed";
const IAS: &str = "sim/flightmodel/position/indicated_airspeed";
const AGL_M: &str = "sim/flightmodel/position/y_agl";
const MAG_VAR: &str = "sim/flightmodel/position/magnetic_variation";
const ON_GROUND: &str = "sim/flightmodel/failures/onground_any";
const THROTTLE_ALL: &str = "sim/cockpit2/engine/actuators/throttle_ratio_all";
const GEAR_HANDLE: &str = "sim/cockpit2/controls/gear_handle_down";
const CMD_SERVOS_ON: &str = "sim/autopilot/servos_on";
const CMD_HDG_MODE: &str = "sim/autopilot/heading";
const CMD_VS_MODE: &str = "sim/autopilot/vertical_speed";
const VS_DIAL: &str = "sim/cockpit/autopilot/vertical_velocity";
const HDG_STATUS: &str = "sim/cockpit2/autopilot/heading_status";
const VS_STATUS: &str = "sim/cockpit2/autopilot/vvi_status";
const TAXI_STEER: &str = "xcopilot/taxi_steer";
const STEER_YIELD: &str = "xcopilot/steer_yield";
const TAKEOFF_PITCH: &str = "xcopilot/pitch";
const PITCH_YIELD: &str = "xcopilot/pitch_yield";
const TAKEOFF_ROLL: &str = "xcopilot/roll";
const ROLL_YIELD: &str = "xcopilot/roll_yield";
const THETA: &str = "sim/flightmodel/position/true_theta";
const PHI: &str = "sim/flightmodel/position/true_phi";
const PARK_BRAKE: &str = "sim/cockpit2/controls/parking_brake_ratio";

const M_TO_FT: f64 = 3.280_84;
const MS_TO_KTS: f64 = 1.943_844;
const M_PER_NM: f64 = 1852.0;

/// Lined up means aligned within this many degrees of a runway's course.
const ALIGN_MAX_DEG: f64 = 20.0;
/// ... and within this many meters of its centreline.
const LATERAL_MAX_M: f64 = 40.0;
/// ... and no more than this far short of the threshold (a lead-on stub
/// aligned with the strip still counts; the far side of the airport doesn't).
const BEFORE_THRESHOLD_MAX_M: f64 = 100.0;
/// Runway remaining below this at arm time is not a takeoff, whatever the
/// POH says — floor for airframes with no published takeoff distance.
const MIN_REMAINING_M: f64 = 500.0;
/// Reject once the remaining runway is inside this with rotation still short:
/// enough pavement to stop a light single from below Vr.
const REJECT_REMAINING_M: f64 = 250.0;
/// No meaningful acceleration this long after brakes-off is a reject —
/// something is holding the aircraft (brakes, mixture, a dead engine).
/// Long enough that the throttle ramp isn't counted against the roll.
/// GROUNDSPEED, not IAS: the 172's airspeed indicator reads zero until
/// ~20 knots, and sortie three's reject fired against an instrument that
/// hadn't started reading yet.
const STAGNANT_SECS: f64 = 12.0;
const STAGNANT_GS_KTS: f64 = 18.0;
/// Full P deflection at this much heading error on the roll — much tighter
/// than the taxi law's 30°: at roll speeds a runway is held with degrees.
const ROLL_FULL_DEG: f64 = 12.0;
/// Hard cap on roll deflection: a takeoff never needs full lock, and in
/// X-Plane full lock under power is a scrubbing swerve.
const MAX_ROLL_STEER: f64 = 0.85;
/// Lead time constant of the damping term: the command answers where the
/// error is GOING, which is what kills the counter-steer oscillation when
/// each observation arrives a network round trip stale.
const STEER_LEAD_S: f64 = 0.4;
/// Max steering sweep rate, PER SECOND — dt-scaled, so the real tick rate
/// never sets the actuator speed. The old per-tick limit (0.25) at the
/// measured half-second tick swept 0.5/s: the rudder lagged the error by
/// over a second, and that lag WAS the left-right weave — sortie four's
/// telemetry shows the steer chasing in exact 0.25 steps the whole roll,
/// a textbook slew-induced limit cycle. 1.5/s crosses the full range in
/// just over a second; the low-speed cap still owns the parked case.
const STEER_SLEW_PER_S: f64 = 1.5;
/// Integrator gain (deflection per degree-second) and cap: the standing
/// rudder the yaw moment demands. Second sortie: pure P+D settled at a
/// STANDING 18 degrees of error against full-power P-factor — the error a
/// proportional law must keep to hold any rudder at all — then the
/// authority taper starved it and the aircraft left the pavement 39
/// degrees off. The integrator learns that trim so the error can go to
/// zero with the rudder still held.
const STEER_TRIM_RATE: f64 = 0.01;
const STEER_TRIM_MAX: f64 = 0.3;
/// The error is clamped to this in the trim update: the integrator learns
/// a STANDING offset, slowly — sortie four's 39 degree transient wound the
/// trim at 0.4/s to its cap in two seconds, and the stale trim then fueled
/// the opposite overshoot. Clamped, the learn rate tops out at 0.06/s.
const STEER_TRIM_ERR_CLAMP_DEG: f64 = 6.0;
/// Speed-authority floor. Sortie two's 0.3 floor starved the STANDING
/// P-factor rudder — but that job belongs to the trim integrator now,
/// which adds OUTSIDE this taper. So the P term can shrink properly with
/// speed again: at 60-80 knots the rudder is powerful, and sortie four's
/// 0.65 floor swung 0.6 of deflection over 8 degree errors — the muscle
/// behind the weave.
const STEER_AUTH_FLOOR: f64 = 0.35;
/// Deflection cap at a standstill, ramping to [`MAX_ROLL_STEER`] by
/// [`LOW_SPEED_FULL_KTS`]. Under takeoff power the prop wash makes the
/// rudder and nosewheel HYPER-effective at low speed — the third sortie
/// commanded 0.85 at brake release to fix a 14 degree error and the
/// aircraft spot-turned 70 degrees through the target at 25 deg/s,
/// pirouetting on the runway until the stagnation guard rejected. Small
/// corrections parked; the full cap arrives with the roll.
const LOW_SPEED_STEER_CAP: f64 = 0.12;
const LOW_SPEED_FULL_KTS: f64 = 15.0;
/// The trim integrator only learns above this groundspeed: it exists for
/// the P-factor moment on the ROLL, and letting it wind up during a
/// low-speed swing doubles every overshoot (sortie three hit its cap in
/// both directions inside the pirouette).
const TRIM_MIN_GS_KTS: f64 = 10.0;
/// Power feed-forward: right rudder led IN WITH the throttle, the way a
/// pilot's foot moves with the hand — the yaw moment is proportional to
/// power, so the compensation rides the ramp instead of waiting for an
/// error to react to. Fades partially with speed (the moment weakens as
/// airflow straightens); the trim integrator learns the residual.
/// Sign assumes a clockwise prop (yaw left, lead right) — every current
/// profile; a counter-clockwise airframe will need this in the profile.
const POWER_LEAD_MAX: f64 = 0.25;
const POWER_LEAD_FADE_KTS: f64 = 40.0;
/// Throttle advances to full over this long — slamming it at a standstill
/// is the maximum yaw moment at the exact moment steering authority is
/// highest and airflow over the rudder is zero.
const THROTTLE_RAMP_S: f64 = 2.5;
/// Centreline correction: degrees of cut per meter of offset, clamped.
const XTK_GAIN_DEG_PER_M: f64 = 0.4;
const XTK_MAX_DEG: f64 = 8.0;
/// Rotation pitch attitude, degrees nose up — the elevator flies the nose
/// here and HOLDS it through the initial climb; at full power in a light
/// single that settles near Vy. Rotation is ELEVATOR, not the autopilot:
/// the AP's VS mode syncs to the CURRENT vertical speed at engage — zero
/// on the runway, whatever the dial was pre-set to — so sortie five called
/// rotate at 55 knots, the AP held the nose down, and the aircraft
/// levitated off at 98 with 250 m of runway left.
const ROTATE_PITCH_DEG: f64 = 8.0;
/// Integrating pitch law: deflection per degree-second of attitude error,
/// a lead on the pitch rate for damping, and caps. Integrating, so it
/// holds whatever standing elevator the trim state demands with the
/// attitude ON target — the same reason the roll law grew its trim.
const PITCH_GAIN_PER_DEG_S: f64 = 0.05;
const PITCH_LEAD_S: f64 = 0.5;
const PITCH_MAX: f64 = 0.65;
const PITCH_MIN: f64 = -0.10;
/// Hand the established climb to the autopilot this high — clear of ground
/// effect, wings level, speed building; the VS dial is written AFTER the
/// mode engages, because engage syncs the dial to the current rate.
const AP_ENGAGE_AGL_FT: f64 = 300.0;
/// The hand-flown climb holds the WINGS too — between liftoff and the AP
/// engage, full-power torque rolls a light single left with nobody on the
/// ailerons. Bank leans toward runway heading, capped shallow.
const CLIMB_BANK_MAX_DEG: f64 = 8.0;
const BANK_PER_HDG_ERR: f64 = 1.0;
/// Aileron law: P on bank error (aileron commands roll RATE, so
/// proportional converges without an integrator), rate-lead damped.
const AIL_FULL_BANK_DEG: f64 = 15.0;
const AIL_LEAD_S: f64 = 0.3;
const AIL_MAX: f64 = 0.5;
const AIL_SLEW_PER_S: f64 = 2.0;
/// Hand the aircraft back this high above the field.
const HANDOFF_AGL_FT: f64 = 1000.0;
/// Raise the gear (when there is one) above this, climbing.
const GEAR_UP_AGL_FT: f64 = 200.0;

/// True while the takeoff loop runs. Cleared by the loop, the release path
/// in main ("my controls" — silent stand-down), or the tool's off switch.
pub static ACTIVE: AtomicBool = AtomicBool::new(false);
/// Set by the `auto_takeoff` tool's off switch: the loop REJECTS (throttle
/// idle, brake pulse) rather than standing down silently.
pub static REJECT: AtomicBool = AtomicBool::new(false);

/// One-line status for the dashboard's "managing" panel.
pub fn status() -> Option<String> {
    ACTIVE.load(Ordering::Relaxed).then(|| "auto takeoff".to_string())
}

/// Everything the loop needs, resolved by the arming tool from the profile.
pub struct Params {
    pub apt_dat: String,
    pub cfg: ApConfig,
    /// Rotate speed, KIAS.
    pub vr_kts: f64,
    /// Initial climb speed, KIAS — the climb law protects it.
    pub vy_kts: f64,
    /// Ceiling on the commanded climb, fpm.
    pub climb_fpm: f64,
    pub retract_gear: bool,
    /// The runway END the tower cleared ("09R"), for the line-up crosscheck.
    /// `None` when no ATC is running or no runway is on the clearance.
    pub cleared_runway: Option<String>,
    /// Runway needed for the roll, meters (POH distance with margin).
    pub needed_m: f64,
}

/// Fit the aircraft's position and heading to a runway end: which strip are
/// we lined up on, and how much of it is left ahead of the nose. `None`
/// when nothing fits — not on a runway, not aligned, or not enough left.
pub fn runway_fit(cands: &[Runway], lat: f64, lon: f64, psi_true: f64, needed_m: f64) -> Option<(Runway, f64)> {
    let mut best: Option<(Runway, f64, f64)> = None;
    for r in cands {
        let err = heading_diff(psi_true, r.course_deg);
        if err > ALIGN_MAX_DEG {
            continue;
        }
        let lateral_m = (cross_track_nm(lat, lon, r) * M_PER_NM).abs();
        if lateral_m > LATERAL_MAX_M {
            continue;
        }
        // along_track is positive SHORT of the threshold, negative down the
        // runway; rolled distance is how far past the numbers we sit.
        let along_m = along_track_nm(lat, lon, r) * M_PER_NM;
        if along_m > BEFORE_THRESHOLD_MAX_M {
            continue;
        }
        let remaining_m = r.length_m - (-along_m).max(0.0);
        if remaining_m < needed_m.max(MIN_REMAINING_M) {
            continue;
        }
        if best.as_ref().is_none_or(|(_, e, _)| err < *e) {
            best = Some((r.clone(), err, remaining_m));
        }
    }
    best.map(|(r, _, rem)| (r, rem))
}

/// Two runway idents name the same end when they match ignoring case and a
/// stray leading zero ("9" vs "09"). Empty on either side is a pass — no
/// data is not a mismatch.
pub fn runway_matches(fitted: &str, cleared: &str) -> bool {
    if fitted.is_empty() || cleared.is_empty() {
        return true;
    }
    let norm = |s: &str| s.trim_start_matches('0').to_ascii_uppercase();
    norm(fitted) == norm(cleared)
}

/// The heading to hold on the roll: runway course with a bounded cut back
/// toward the centreline (`xtk_m` right-positive, same convention as
/// autoland's final).
pub fn roll_target_heading(course_deg: f64, xtk_m: f64) -> f64 {
    let corr = (xtk_m * XTK_GAIN_DEG_PER_M).clamp(-XTK_MAX_DEG, XTK_MAX_DEG);
    (course_deg - corr).rem_euclid(360.0)
}

/// Slew limit: move from `prev` toward `want` by at most `max_step`.
pub fn slew(prev: f64, want: f64, max_step: f64) -> f64 {
    prev + (want - prev).clamp(-max_step, max_step)
}

/// The roll steering law: P on the error, a lead term on its rate (damps
/// the counter-steer oscillation of stale observations), and a slow
/// integrator that LEARNS the standing rudder the yaw moment demands —
/// P-factor at full power needs rudder held with the nose already
/// straight, which no proportional term can do without a standing error.
/// Output is capped, speed-tapered (gently — the taper's old floor
/// starved the law at rotate speed), and slew-limited.
#[derive(Default)]
pub struct RollSteer {
    trim: f64,
    prev_err: Option<f64>,
    out: f64,
}

/// The rudder led in WITH the power: proportional to the throttle
/// actually commanded, fading partially as airflow builds.
pub fn power_lead(thr: f64, gs_kts: f64) -> f64 {
    POWER_LEAD_MAX * thr * (1.0 - 0.6 * (gs_kts / POWER_LEAD_FADE_KTS).clamp(0.0, 1.0))
}

impl RollSteer {
    /// One control tick: heading error (deg, right positive), the time
    /// since the last tick, groundspeed, and the throttle being
    /// commanded (for the power feed-forward). Returns the deflection.
    pub fn step(&mut self, err_deg: f64, dt_s: f64, gs_kts: f64, thr: f64) -> f64 {
        let dt = dt_s.clamp(0.05, 1.0);
        let rate = self.prev_err.map_or(0.0, |p| (err_deg - p) / dt);
        self.prev_err = Some(err_deg);
        let auth = 1.0 - (1.0 - STEER_AUTH_FLOOR) * ((gs_kts - 15.0) / 35.0).clamp(0.0, 1.0);
        let pd = ((err_deg + STEER_LEAD_S * rate) / ROLL_FULL_DEG).clamp(-1.0, 1.0) * auth;
        // Prop wash makes the tail HYPER-effective under power at low
        // speed: the cap is tiny parked and grows with the roll, so an
        // engage-time misalignment is walked out over the first hundred
        // meters instead of spot-turned through (the sortie-three pirouette).
        let cap = LOW_SPEED_STEER_CAP
            + (MAX_ROLL_STEER - LOW_SPEED_STEER_CAP)
                * (gs_kts / LOW_SPEED_FULL_KTS).clamp(0.0, 1.0);
        let want = (pd + self.trim + power_lead(thr, gs_kts)).clamp(-cap, cap);
        // The trim integrates unconditionally ON THE ROLL — its own small
        // cap bounds wind-up, and a saturated command with a persisting
        // error is exactly when the standing rudder must build. Below
        // taxi pace it stays frozen: it exists for P-factor at speed, not
        // for doubling every low-speed overshoot.
        if gs_kts > TRIM_MIN_GS_KTS {
            let e = err_deg.clamp(-STEER_TRIM_ERR_CLAMP_DEG, STEER_TRIM_ERR_CLAMP_DEG);
            self.trim =
                (self.trim + e * dt * STEER_TRIM_RATE).clamp(-STEER_TRIM_MAX, STEER_TRIM_MAX);
        }
        self.out = slew(self.out, want, STEER_SLEW_PER_S * dt);
        self.out
    }

    /// The learned standing-rudder trim, for telemetry.
    pub fn trim(&self) -> f64 {
        self.trim
    }
}

/// Rotation waits for a roughly straight nose: pulling the aircraft off
/// yawed launches it sideways off the strip (the second sortie rotated 39
/// degrees off heading, from the grass). Unless the runway is running out,
/// where flying beats the overrun.
pub fn rotate_ok(err_deg: f64, remaining_m: f64) -> bool {
    err_deg.abs() < 15.0 || remaining_m < 400.0
}

/// The rotation elevator: an integrating law on pitch attitude, damped on
/// the pitch rate. Pulls the nose to [`ROTATE_PITCH_DEG`] and holds it
/// there through the initial climb, until the AP takes the established
/// climb at [`AP_ENGAGE_AGL_FT`].
#[derive(Default)]
pub struct RotatePitch {
    out: f64,
    prev_theta: Option<f64>,
}

impl RotatePitch {
    pub fn step(&mut self, theta_deg: f64, dt_s: f64) -> f64 {
        let dt = dt_s.clamp(0.05, 1.0);
        let rate = self.prev_theta.map_or(0.0, |p| (theta_deg - p) / dt);
        self.prev_theta = Some(theta_deg);
        let err = ROTATE_PITCH_DEG - theta_deg - PITCH_LEAD_S * rate;
        self.out = (self.out + err * PITCH_GAIN_PER_DEG_S * dt).clamp(PITCH_MIN, PITCH_MAX);
        self.out
    }
}

/// The bank the climb-out flies: toward runway heading, capped shallow —
/// this is track-keeping on departure, not a turn.
pub fn climb_bank_target(hdg_err_deg: f64) -> f64 {
    (hdg_err_deg * BANK_PER_HDG_ERR).clamp(-CLIMB_BANK_MAX_DEG, CLIMB_BANK_MAX_DEG)
}

/// The climb-out ailerons: P on bank error with a rate lead, slew-limited.
/// Bank positive = right wing down; aileron positive = roll right — the
/// same convention, so the error maps straight through.
#[derive(Default)]
pub struct WingsLevel {
    out: f64,
    prev_bank: Option<f64>,
}

impl WingsLevel {
    pub fn step(&mut self, bank_deg: f64, target_deg: f64, dt_s: f64) -> f64 {
        let dt = dt_s.clamp(0.05, 1.0);
        let rate = self.prev_bank.map_or(0.0, |p| (bank_deg - p) / dt);
        self.prev_bank = Some(bank_deg);
        let err = target_deg - bank_deg - AIL_LEAD_S * rate;
        let want = (err / AIL_FULL_BANK_DEG).clamp(-1.0, 1.0) * AIL_MAX;
        self.out = slew(self.out, want, AIL_SLEW_PER_S * dt);
        self.out
    }
}

/// Throttle schedule on the roll: 30% immediately (the engine spools, the
/// roll starts, the yaw stays small), full power over the ramp.
pub fn throttle_ramp(elapsed_s: f64) -> f64 {
    (0.3 + 0.7 * (elapsed_s / THROTTLE_RAMP_S)).clamp(0.3, 1.0)
}

/// Nose this far off at low speed holds the power back — full power on a
/// swinging nose is the maximum P-factor kick at the worst moment (the
/// time-only ramp reached 100% while sortie three was still pirouetting
/// nearly stationary). The hold releases with the nose straight, and it
/// only applies below [`THROTTLE_HOLD_MAX_GS_KTS`]: chopping power over a
/// gust-yaw at 40 knots mid-roll would be its own emergency.
const THROTTLE_HOLD_ERR_DEG: f64 = 10.0;
const THROTTLE_HOLD_RATIO: f64 = 0.5;
const THROTTLE_HOLD_MAX_GS_KTS: f64 = 20.0;

/// The commanded throttle this tick: the time ramp, held at half power
/// while the nose is still swinging at low speed.
pub fn throttle_cmd(elapsed_s: f64, err_deg: f64, gs_kts: f64) -> f64 {
    let ramp = throttle_ramp(elapsed_s);
    if gs_kts < THROTTLE_HOLD_MAX_GS_KTS && err_deg.abs() > THROTTLE_HOLD_ERR_DEG {
        ramp.min(THROTTLE_HOLD_RATIO)
    } else {
        ramp
    }
}

/// Throttle advances are ALWAYS gradual, releases instant: when the
/// swinging-nose hold lets go, the lever walks up at the ramp's own rate
/// instead of jumping half-to-full in one tick — sortie four's release
/// kicked 19 degrees of yaw at 12 knots, exactly the swing the hold
/// existed to prevent. The power feed-forward rides the commanded value,
/// so the rudder walks up with it.
const THROTTLE_UP_PER_S: f64 = 0.28;
pub fn throttle_slew(prev: f64, want: f64, dt_s: f64) -> f64 {
    if want <= prev {
        want
    } else {
        (prev + THROTTLE_UP_PER_S * dt_s.clamp(0.02, 1.0)).min(want)
    }
}

/// The commanded climb rate: the profile's best rate, backed off while the
/// airspeed is below the climb speed — speed first, rate second, the same
/// energy rule autoland's go-around flies. Never below a positive floor:
/// once airborne the aircraft keeps climbing away from the ground.
pub fn climb_vs_fpm(ias_kts: f64, vy_kts: f64, max_fpm: f64) -> f64 {
    (max_fpm - (vy_kts - ias_kts).max(0.0) * 60.0).clamp(150.0, max_fpm)
}

/// Reject when the runway is running out and rotation speed is still short:
/// inside the margin with less than 90% of Vr there is no takeoff today.
pub fn must_reject(remaining_m: f64, ias_kts: f64, vr_kts: f64) -> bool {
    remaining_m < REJECT_REMAINING_M && ias_kts < vr_kts * 0.9
}

/// Spawn the takeoff loop. The tool has already gated replay, ground,
/// clearance state, and the steer channel; the loop re-validates the runway
/// itself (the apt.dat scan is too slow for the tool's reply).
pub fn start(base_url: String, voice: Option<Voice>, p: Params) {
    if ACTIVE.swap(true, Ordering::Relaxed) {
        return;
    }
    REJECT.store(false, Ordering::Relaxed);
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread().enable_all().build() else {
            ACTIVE.store(false, Ordering::Relaxed);
            return;
        };
        rt.block_on(fly(XPlaneClient::new(base_url), voice, p));
        ACTIVE.store(false, Ordering::Relaxed);
    });
}

fn alert(voice: &Option<Voice>, priority: Priority, line: &str) {
    if let Some(v) = voice {
        v.alert(priority, line);
    }
}

/// Throttle idle, one expiring brake pulse, steering released, and say why.
async fn reject(xp: &XPlaneClient, voice: &Option<Voice>, line: &str, reason: &str) {
    let _ = xp.set_float(THROTTLE_ALL, 0.0).await;
    if xp
        .run_command_for(crate::tools::BRAKES_REGULAR, crate::tools::BRAKE_HOLD_SECS)
        .await
        .is_ok()
    {
        crate::tools::note_brake_cmd();
    }
    let _ = xp.set_float(TAXI_STEER, 0.0).await;
    alert(voice, Priority::Warning, line);
    crate::blackbox::record("autotakeoff", serde_json::json!({"event": "reject", "reason": reason}));
}

async fn fly(xp: XPlaneClient, voice: Option<Voice>, p: Params) {
    // Find the runway under the nose. A full apt.dat pass, seconds — done
    // here, off the tool's reply path.
    let (Ok(lat), Ok(lon), Ok(psi)) = (
        xp.get_float(LAT).await,
        xp.get_float(LON).await,
        xp.get_float(PSI_TRUE).await,
    ) else {
        alert(&voice, Priority::Caution, "I can't read our position. Your takeoff.");
        return;
    };
    let cands = match crate::autoland::nearby_runways(&p.apt_dat, lat, lon, 2.0) {
        Ok(c) => c,
        Err(e) => {
            tracing::warn!(?e, "auto takeoff: apt.dat unreadable");
            alert(&voice, Priority::Caution, "I can't read the runway data. Your takeoff.");
            return;
        }
    };
    let Some((rwy, remaining_m)) = runway_fit(&cands, lat, lon, psi, p.needed_m) else {
        // At the hold with a line-up in reach, point at the maneuver that
        // exists for exactly this — otherwise the honest refusal.
        let line = if crate::autotaxi::pick_line_up(&cands, lat, lon, p.cleared_runway.as_deref())
            .is_some()
        {
            "We're not on the runway yet. Say line us up, then give me the takeoff."
        } else {
            "We're not lined up on a runway I can use. Line us up and ask me again."
        };
        alert(&voice, Priority::Caution, line);
        return;
    };
    if let Some(clr) = &p.cleared_runway {
        if !runway_matches(&rwy.id, clr) {
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
    // Every refusal gate has passed — NOW stand the taxi governor down (a
    // line-up ends with it holding position at target zero; it kept the
    // aircraft parked through the scan above). Doing this in the tool stood
    // it down before a refusal and left the aircraft coasting uncontrolled
    // (first sortie it braked against the roll, second it let a refused
    // takeoff roll away — both LXGB, 2026-07-20). Its loop exits within a
    // tick; the moment of free-wheeling is the throttle ramp's first beat.
    crate::tools::speedhold::ACTIVE.store(false, Ordering::Relaxed);
    tokio::time::sleep(std::time::Duration::from_millis(500)).await;
    // The line-up ends "brakes set" — the parking-brake ratio latched. It
    // is OURS to release: sortie five's first engage sat at full power,
    // groundspeed zero, for twelve seconds against the parked brake and
    // rejected as stagnant.
    let _ = xp.set_float(PARK_BRAKE, 0.0).await;
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
    alert(
        &voice,
        Priority::Advisory,
        &format!("Brakes released. Rolling, runway {}.", crate::atis::spoken_runway(&rwy.id)),
    );

    // Pre-set the flight director and heading bug NOW, parked: the bug
    // slews by knob presses (one HTTP round trip per degree), and doing
    // that later would stall the loop while the steer channel expires
    // mid-roll. No VS dial preset — VS-mode engage syncs the dial to the
    // current rate anyway, so it's written after the engage instead.
    for c in &p.cfg.fd_commands {
        let _ = xp.run_command(c).await;
    }
    let var = xp.get_float(MAG_VAR).await.unwrap_or(0.0);
    let _ =
        crate::tools::autopilot::drive_heading_bug_step(&xp, true_to_mag(rwy.course_deg, var), 360.0)
            .await;

    let mut airborne = false;
    let mut rotated = false;
    let mut ap_engaged = false;
    let mut gear_raised = !p.retract_gear;
    let mut positive_called = false;
    let mut thr_challenged = false;
    let mut pitch_law = RotatePitch::default();
    let mut wings = WingsLevel::default();
    let mut last_vs_cmd: Option<f64> = None;
    let mut last_thr_cmd = 0.3_f64;
    // The roll section's heading error, one tick stale, for the throttle's
    // swinging-nose hold (throttle is commanded before position is read).
    let mut last_err = 0.0_f64;
    let mut steer_law = RollSteer::default();
    let mut last_tick: Option<std::time::Instant> = None;
    let roll_start = std::time::Instant::now();
    let mut frozen_secs = 0.0_f64;
    loop {
        if !ACTIVE.load(Ordering::Relaxed) {
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
                crate::blackbox::record("autotakeoff", serde_json::json!({"event": "handoff_airborne_abort"}));
            } else {
                reject(&xp, &voice, "Rejecting takeoff. Throttle is idle, brakes on. Your aircraft.", "pilot_abort")
                    .await;
            }
            return;
        }
        // Pause or replay: freeze — no writes, and the stagnation clock
        // slides instead of burning (the recover watcher's rule).
        if crate::xplane::sim_frozen(&xp).await {
            tokio::time::sleep(std::time::Duration::from_millis(400)).await;
            frozen_secs += 0.4;
            continue;
        }
        // One concurrent read burst: sequentially these eleven round trips
        // took ~350 ms and dragged the control tick to half a second — the
        // lag behind the fourth sortie's weave. Joined, the tick runs at
        // the sleep rate.
        let (lat, lon, psi, gs_ms, ias, agl_m, on_ground, steer_yield, thr_read, theta, phi, pitch_yield, roll_yield) = tokio::join!(
            xp.get_float(LAT),
            xp.get_float(LON),
            xp.get_float(PSI_TRUE),
            xp.get_float(GS_MS),
            xp.get_float(IAS),
            xp.get_float(AGL_M),
            xp.get_float(ON_GROUND),
            xp.get_float(STEER_YIELD),
            xp.get_float(THROTTLE_ALL),
            xp.get_float(THETA),
            xp.get_float(PHI),
            xp.get_float(PITCH_YIELD),
            xp.get_float(ROLL_YIELD),
        );
        let (Ok(lat), Ok(lon), Ok(psi), Ok(gs_ms), Ok(ias), Ok(agl_m), Ok(on_ground)) =
            (lat, lon, psi, gs_ms, ias, agl_m, on_ground)
        else {
            tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            continue;
        };
        let gs_kts = gs_ms * MS_TO_KTS;
        let agl_ft = agl_m * M_TO_FT;
        let theta_deg = theta.unwrap_or(0.0);
        let phi_deg = phi.unwrap_or(0.0);
        let now_t = std::time::Instant::now();
        let dt = last_tick.map_or(0.15, |t| (now_t - t).as_secs_f64()).clamp(0.05, 1.0);
        last_tick = Some(now_t);

        // The pilot's feet took the pedals (the plugin dropped the override
        // already): the takeoff continues as a manual one, power set.
        if !airborne && steer_yield.is_ok_and(|v| v > 0.5) {
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
        let elapsed = roll_start.elapsed().as_secs_f64() - frozen_secs;
        let thr_want = if airborne { 1.0 } else { throttle_cmd(elapsed, last_err, gs_kts) };
        let thr_cmd = throttle_slew(last_thr_cmd, thr_want, dt);
        let thr_now = thr_read.unwrap_or(thr_cmd);
        if thr_now < last_thr_cmd - 0.25 && elapsed > 2.0 {
            if thr_challenged {
                alert(&voice, Priority::Warning, "Your throttle. Your aircraft.");
                crate::blackbox::record("autotakeoff", serde_json::json!({"event": "throttle_yield"}));
                let _ = xp.set_float(TAXI_STEER, 0.0).await;
                return;
            }
            thr_challenged = true;
        } else {
            thr_challenged = false;
        }
        let _ = xp.set_float(THROTTLE_ALL, thr_cmd).await;
        last_thr_cmd = thr_cmd;

        if !airborne {
            // Not accelerating: something is holding the aircraft.
            if elapsed > STAGNANT_SECS && gs_kts < STAGNANT_GS_KTS {
                reject(&xp, &voice, "We're not accelerating. Rejecting takeoff. Your aircraft.", "stagnant")
                    .await;
                return;
            }
            let along_m = along_track_nm(lat, lon, &rwy) * M_PER_NM;
            let remaining = rwy.length_m - (-along_m).max(0.0);
            if !rotated && must_reject(remaining, ias, p.vr_kts) {
                reject(&xp, &voice, "Runway is running out. Rejecting takeoff. Your aircraft.", "runway_end")
                    .await;
                return;
            }
            // Steer the centreline: PD plus the learned P-factor trim,
            // slew-limited (no tick slams the wheel).
            let xtk_m = cross_track_nm(lat, lon, &rwy) * M_PER_NM;
            let target = roll_target_heading(rwy.course_deg, xtk_m);
            let err = ((target - psi + 540.0).rem_euclid(360.0)) - 180.0;
            last_err = err;
            let steer = steer_law.step(err, dt, gs_kts, thr_cmd);
            if xp.set_float(TAXI_STEER, steer).await.is_err() {
                reject(&xp, &voice, "I've lost the steering. Rejecting takeoff. Your aircraft.", "steer_lost")
                    .await;
                return;
            }
            crate::blackbox::record(
                "autotakeoff",
                serde_json::json!({
                    "event": "roll", "gs": gs_kts, "ias": ias, "err": err,
                    "steer": steer, "trim": steer_law.trim(), "xtk_m": xtk_m,
                    "thr": thr_cmd, "remaining_m": remaining, "dt": dt,
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
                alert(&voice, Priority::Advisory, "Rotate.");
                crate::blackbox::record(
                    "autotakeoff",
                    serde_json::json!({"event": "rotate", "ias": ias, "remaining_m": remaining}),
                );
            }
            if rotated {
                // Wings held level through the pull — the rudder steering
                // still owns the heading until liftoff.
                let pitch = pitch_law.step(theta_deg, dt);
                let ail = wings.step(phi_deg, 0.0, dt);
                let (wp, wr) =
                    tokio::join!(xp.set_float(TAKEOFF_PITCH, pitch), xp.set_float(TAKEOFF_ROLL, ail));
                if wp.is_err() || wr.is_err() {
                    reject(&xp, &voice, "I've lost the elevator. Rejecting takeoff. Your aircraft.", "pitch_lost")
                        .await;
                    return;
                }
            }
            // Airborne only once clearly clear of the surface: a ground-
            // effect bounce (on_ground false at 2 ft, back down a tick
            // later — second sortie) must not stop the steering while the
            // wheels are about to touch again at speed.
            if on_ground < 0.5 && ias > p.vr_kts * 0.8 && agl_ft > 8.0 {
                airborne = true;
                let _ = xp.set_float(TAXI_STEER, 0.0).await;
                crate::blackbox::record(
                    "autotakeoff",
                    serde_json::json!({"event": "liftoff", "ias": ias}),
                );
            }
            tokio::time::sleep(std::time::Duration::from_millis(150)).await;
            continue;
        }

        // Airborne: pitch attitude by hand until established, then the AP.
        if !positive_called && agl_ft > 50.0 {
            positive_called = true;
            alert(&voice, Priority::Advisory, "Positive rate.");
        }
        if !gear_raised && agl_ft > GEAR_UP_AGL_FT {
            gear_raised = true;
            let _ = xp.set_float(GEAR_HANDLE, 0.0).await;
            alert(&voice, Priority::Advisory, "Gear up.");
        }
        if !ap_engaged {
            if agl_ft < AP_ENGAGE_AGL_FT {
                // Still hand-flying: hold the rotation attitude while speed
                // builds, and the wings against full-power torque — banked
                // gently toward runway heading, since the rudder steering
                // ended at liftoff.
                let hdg_err = ((rwy.course_deg - psi + 540.0).rem_euclid(360.0)) - 180.0;
                let pitch = pitch_law.step(theta_deg, dt);
                let ail = wings.step(phi_deg, climb_bank_target(hdg_err), dt);
                let (wp, wr) =
                    tokio::join!(xp.set_float(TAKEOFF_PITCH, pitch), xp.set_float(TAKEOFF_ROLL, ail));
                if wp.is_err() || wr.is_err() {
                    alert(&voice, Priority::Warning, "I've lost the elevator. Your aircraft.");
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
                        "theta": theta_deg, "pitch": pitch,
                        "phi": phi_deg, "ail": ail, "hdg_err": hdg_err,
                    }),
                );
                tokio::time::sleep(std::time::Duration::from_millis(150)).await;
                continue;
            }
            // Established in the climb: the AP takes it. Servos + HDG + VS,
            // and the VS dial written AFTER the mode engages — engage syncs
            // the dial to the CURRENT rate, stomping any preset (the sync
            // that made VS-mode rotation impossible in the first place).
            // The pitch channel simply expires; the servo owns the elevator.
            ap_engaged = true;
            let _ = xp.run_command(CMD_SERVOS_ON).await;
            if xp.get_float(HDG_STATUS).await.unwrap_or(0.0) <= 0.5 {
                let _ = xp.run_command(CMD_HDG_MODE).await;
            }
            if xp.get_float(VS_STATUS).await.unwrap_or(0.0) <= 0.5 {
                let _ = xp.run_command(CMD_VS_MODE).await;
            }
            let vs = climb_vs_fpm(ias, p.vy_kts, p.climb_fpm);
            let _ = xp.set_float(VS_DIAL, vs).await;
            last_vs_cmd = Some(vs);
            crate::blackbox::record(
                "autotakeoff",
                serde_json::json!({"event": "ap_engaged", "ias": ias, "agl": agl_ft, "cmd_vs": vs}),
            );
        }
        let vs = climb_vs_fpm(ias, p.vy_kts, p.climb_fpm);
        if last_vs_cmd.is_none_or(|v| (v - vs).abs() > 40.0) {
            let _ = xp.set_float(VS_DIAL, vs).await;
            last_vs_cmd = Some(vs);
        }
        if xp.get_float(VS_STATUS).await.unwrap_or(1.0) <= 0.5 {
            // Someone (or a mode change) dropped VS: re-engage, and force a
            // dial rewrite next tick — the engage just synced it away.
            let _ = xp.run_command(CMD_VS_MODE).await;
            last_vs_cmd = None;
        }
        crate::blackbox::record(
            "autotakeoff",
            serde_json::json!({"event": "climb", "ias": ias, "agl": agl_ft, "cmd_vs": vs}),
        );
        if agl_ft >= HANDOFF_AGL_FT {
            alert(
                &voice,
                Priority::Caution,
                &format!(
                    "One thousand feet, {} knots. Your aircraft. The autopilot is holding runway heading and the climb.",
                    ias.round() as i64
                ),
            );
            crate::blackbox::record(
                "autotakeoff",
                serde_json::json!({"event": "handoff", "agl": agl_ft, "ias": ias}),
            );
            return;
        }
        tokio::time::sleep(std::time::Duration::from_millis(400)).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autoland::project_latlon;

    /// A 2,000 m runway at the equator, taking off due north from (0,0).
    fn rwy_north() -> Runway {
        Runway {
            icao: "TEST".into(),
            id: "36".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 2000.0,
            surface: 1,
            dist_nm: 0.0,
            elev_ft: 0.0,
        }
    }

    #[test]
    fn fits_the_runway_under_an_aligned_nose() {
        let cands = vec![rwy_north()];
        // On the numbers, pointing down the strip: fits, full length ahead.
        let (r, rem) = runway_fit(&cands, 0.0, 0.0, 2.0, 500.0).expect("lined up");
        assert_eq!(r.id, "36");
        assert!((rem - 2000.0).abs() < 30.0, "{rem}");
        // 300 m down the roll: fits with less left.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 300.0 / 1852.0);
        let (_, rem) = runway_fit(&cands, lat, lon, 358.0, 500.0).expect("mid-runway");
        assert!((rem - 1700.0).abs() < 40.0, "{rem}");
    }

    #[test]
    fn refuses_misaligned_offset_or_short() {
        let cands = vec![rwy_north()];
        // Facing 40 degrees off: not lined up.
        assert!(runway_fit(&cands, 0.0, 0.0, 40.0, 500.0).is_none());
        // At the hold-short point, 60 m right of the centreline: not on it.
        let (lat, lon) = project_latlon(0.0, 0.0, 90.0, 60.0 / 1852.0);
        assert!(runway_fit(&cands, lat, lon, 0.0, 500.0).is_none());
        // 1,700 m down a 2,000 m strip: not enough left to roll.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 1700.0 / 1852.0);
        assert!(runway_fit(&cands, lat, lon, 0.0, 500.0).is_none());
        // Backwards on the strip fits the RECIPROCAL end, never this one.
        assert!(runway_fit(&cands, 0.0, 0.0, 180.0, 500.0).is_none());
    }

    #[test]
    fn cleared_runway_crosscheck_is_forgiving_of_format_only() {
        assert!(runway_matches("09R", "09R"));
        assert!(runway_matches("9", "09"));
        assert!(runway_matches("27l", "27L"));
        assert!(runway_matches("", "27"), "no fitted ident is not a mismatch");
        assert!(runway_matches("27", ""), "no cleared ident is not a mismatch");
        assert!(!runway_matches("27", "09"), "the backwards line-up must be caught");
        assert!(!runway_matches("27L", "27R"));
    }

    #[test]
    fn roll_target_cuts_toward_the_centreline_bounded() {
        // 10 m right of the line: cut left of runway heading, bounded.
        let tgt = roll_target_heading(0.0, 10.0);
        assert!(tgt > 350.0 && tgt < 360.0, "cut left, got {tgt}");
        // 100 m off (should never happen): still clamped to the max cut.
        let tgt = roll_target_heading(0.0, 100.0);
        assert!((360.0 - tgt - XTK_MAX_DEG).abs() < 0.01, "{tgt}");
    }

    #[test]
    fn the_integrator_trims_out_a_standing_yaw_moment() {
        // The second sortie's failure shape: a constant-ish 18 degree error
        // that pure P needed to hold ANY rudder. Feed the law a standing
        // error; the trim must grow so the total command keeps rising even
        // though P alone has plateaued — the rudder that finally straightens
        // the nose.
        let mut law = RollSteer::default();
        let first = law.step(18.0, 0.15, 25.0, 0.0);
        let mut last = first;
        for _ in 0..40 {
            last = law.step(18.0, 0.15, 25.0, 0.0);
        }
        assert!(last > first, "trim accumulates: {first} -> {last}");
        assert!(law.trim() > 0.2, "learned standing rudder, got {}", law.trim());
        // And with the error finally at zero the trim STILL holds rudder —
        // the whole point: straight nose, rudder in.
        let centered = law.step(0.0, 0.15, 40.0, 0.0);
        assert!(centered > 0.15, "rudder held with the nose straight: {centered}");
    }

    #[test]
    fn a_transient_swing_cannot_wind_the_trim() {
        // Sortie four: a 39 degree recovery overshoot wound the trim to its
        // cap in two seconds, and the stale trim fueled the NEXT overshoot.
        // The error is clamped in the update: a huge swing learns no faster
        // than a modest standing error.
        let mut big = RollSteer::default();
        let mut small = RollSteer::default();
        for _ in 0..10 {
            big.step(39.0, 0.15, 30.0, 0.0);
            small.step(STEER_TRIM_ERR_CLAMP_DEG, 0.15, 30.0, 0.0);
        }
        assert!((big.trim() - small.trim()).abs() < 1e-9, "{} vs {}", big.trim(), small.trim());
        assert!(big.trim() < 0.1, "two seconds of swing stays a nudge: {}", big.trim());
    }

    #[test]
    fn throttle_advances_gradually_and_releases_instantly() {
        // The swinging-nose hold letting go must WALK the lever up, not jump
        // half-to-full in a tick (sortie four's release kicked 19 degrees of
        // yaw at 12 knots).
        let stepped = throttle_slew(0.5, 1.0, 0.15);
        assert!(stepped < 0.55, "one tick walks, never jumps: {stepped}");
        let mut thr = 0.5;
        for _ in 0..20 {
            thr = throttle_slew(thr, 1.0, 0.15);
        }
        assert!((thr - 1.0).abs() < 1e-9, "full power arrives: {thr}");
        // A pull-back (the hold re-engaging, a pilot hand) is honored NOW.
        assert_eq!(throttle_slew(1.0, 0.5, 0.15), 0.5);
    }

    #[test]
    fn roll_law_damps_and_never_slams() {
        let mut law = RollSteer::default();
        // A swing home (error collapsing fast) commands LESS than the same
        // error steady — the lead term kills the counter-steer oscillation.
        let mut steady = RollSteer::default();
        steady.step(12.0, 0.15, 20.0, 0.0);
        let steady_out = steady.step(12.0, 0.15, 20.0, 0.0);
        law.step(20.0, 0.15, 20.0, 0.0);
        let collapsing = law.step(12.0, 0.15, 20.0, 0.0);
        assert!(collapsing < steady_out, "{collapsing} vs {steady_out}");
        // From rest, a huge demanded deflection arrives in slew steps —
        // dt-scaled, so a slow tick sweeps proportionally more, never a
        // fixed per-tick crumb (the fixed 0.25 at half-second ticks WAS
        // sortie four's weave).
        let mut fresh = RollSteer::default();
        assert!(fresh.step(90.0, 0.15, 5.0, 0.0).abs() <= STEER_SLEW_PER_S * 0.15 + 1e-9);
        let mut slow_tick = RollSteer::default();
        let one = slow_tick.step(90.0, 0.5, 20.0, 0.0).abs();
        assert!(one > 0.5, "a half-second tick sweeps half the range: {one}");
        // Output is always inside the cap.
        let mut sat = RollSteer::default();
        for _ in 0..30 {
            assert!(sat.step(90.0, 0.15, 5.0, 0.0).abs() <= MAX_ROLL_STEER + 1e-9);
        }
        // Authority at speed keeps a real floor: the same error at 50 kt
        // still commands most of what it does at 15 kt (the 0.21 starvation
        // was the second sortie's runway excursion).
        let mut slow = RollSteer::default();
        let mut fast = RollSteer::default();
        let s = (0..3).map(|_| slow.step(10.0, 0.15, 10.0, 0.0)).last().unwrap();
        let f = (0..3).map(|_| fast.step(10.0, 0.15, 50.0, 0.0)).last().unwrap();
        assert!(f >= s * STEER_AUTH_FLOOR - 1e-9, "floor holds: {f} vs {s}");
    }

    #[test]
    fn parked_under_power_the_wheel_barely_moves() {
        // Sortie three: 0.85 of deflection at brake release spot-turned
        // the aircraft at 25 deg/s (prop wash makes the tail hyper-
        // effective under power at low speed). At a standstill even a
        // huge error commands only the low-speed cap, and the trim stays
        // frozen so the swing can't wind it up.
        let mut law = RollSteer::default();
        let mut out = 0.0;
        for _ in 0..10 {
            out = law.step(-60.0, 0.15, 0.0, 0.0);
        }
        assert!(out.abs() <= LOW_SPEED_STEER_CAP + 1e-9, "{out}");
        assert_eq!(law.trim(), 0.0, "no trim learning parked");
        // The cap grows with the roll: by 15 knots the full range is back.
        let mut rolling = RollSteer::default();
        let mut r = 0.0;
        for _ in 0..10 {
            r = rolling.step(-60.0, 0.15, 20.0, 0.0);
        }
        assert!(r.abs() > 0.5, "full authority once rolling, got {r}");
    }

    #[test]
    fn rudder_leads_in_with_the_power_and_throttle_waits_for_a_straight_nose() {
        // Zero error, power coming up: the foot moves with the hand — the
        // law holds right rudder proportional to throttle, no error needed.
        let mut law = RollSteer::default();
        let with_power = (0..5).map(|_| law.step(0.0, 0.15, 5.0, 1.0)).last().unwrap();
        assert!(with_power > 0.1, "lead rides the throttle, got {with_power}");
        let mut idle = RollSteer::default();
        let no_power = (0..5).map(|_| idle.step(0.0, 0.15, 5.0, 0.0)).last().unwrap();
        assert!(no_power.abs() < 0.01, "no power, no lead: {no_power}");
        // The lead fades as airflow straightens the moment out.
        assert!(power_lead(1.0, 40.0) < power_lead(1.0, 0.0));
        assert!(power_lead(1.0, 40.0) > 0.05, "never fades to nothing on the roll");
        // Throttle: swinging nose at low speed holds half power...
        assert_eq!(throttle_cmd(10.0, 14.0, 8.0), THROTTLE_HOLD_RATIO);
        // ...straight nose rides the ramp to full...
        assert_eq!(throttle_cmd(10.0, 2.0, 8.0), 1.0);
        // ...and at speed a gust yaw never chops the power.
        assert_eq!(throttle_cmd(10.0, 14.0, 40.0), 1.0);
        // The ramp itself still applies early with the nose straight.
        assert!(throttle_cmd(0.0, 0.0, 0.0) < 0.4);
    }

    #[test]
    fn rotation_waits_for_a_straight_nose_unless_the_runway_ends() {
        assert!(rotate_ok(3.0, 900.0), "straight and long: rotate");
        assert!(!rotate_ok(25.0, 900.0), "yawed with room: keep straightening");
        assert!(rotate_ok(25.0, 300.0), "yawed but running out: fly");
    }

    #[test]
    fn rotation_is_an_elevator_pull_that_holds_the_attitude() {
        // Sortie five: rotate called at 55 knots, the AP's VS mode held the
        // nose down (engage syncs to the current rate: zero), and the
        // aircraft levitated off at 98 with 250 m left. The rotation law is
        // an integrating elevator pull to the target attitude.
        let mut law = RotatePitch::default();
        let first = law.step(0.0, 0.15);
        assert!(first > 0.0, "nose on the ground, pull back: {first}");
        let mut out = first;
        for _ in 0..40 {
            out = law.step(0.0, 0.15);
        }
        assert!(out > 0.3, "an unresponsive nose earns more pull: {out}");
        assert!(out <= PITCH_MAX + 1e-9, "capped: {out}");
        // Integrating: on target and steady, the standing pull HOLDS — the
        // elevator the trim state demands at this speed.
        let mut settled = RotatePitch::default();
        for _ in 0..10 {
            settled.step(ROTATE_PITCH_DEG - 1.0, 0.15);
        }
        settled.step(ROTATE_PITCH_DEG, 0.15);
        let a = settled.step(ROTATE_PITCH_DEG, 0.15);
        let b = settled.step(ROTATE_PITCH_DEG, 0.15);
        assert!(a > 0.0 && (a - b).abs() < 1e-6, "holds at target: {a} vs {b}");
        // A nose rising fast toward the target backs the pull off (the
        // rate lead), so the rotation doesn't slingshot past the attitude.
        let mut rising = RotatePitch::default();
        let before = rising.step(2.0, 0.15);
        let after = rising.step(6.0, 0.15);
        assert!(after < before, "fast rise backs off: {before} -> {after}");
        // And the nose-down authority is a fraction of the pull: this law
        // rotates, it never bunts.
        let mut over = RotatePitch::default();
        for _ in 0..100 {
            over.step(30.0, 0.15);
        }
        assert!(over.out >= PITCH_MIN - 1e-9, "floor holds: {}", over.out);
    }

    #[test]
    fn slew_limits_and_throttle_ramps() {
        // A demanded hard-over arrives in steps, not a slam.
        assert_eq!(slew(0.0, 0.7, 0.25), 0.25);
        assert_eq!(slew(0.25, 0.7, 0.25), 0.5);
        assert_eq!(slew(0.5, 0.55, 0.25), 0.55, "small changes pass through");
        assert!((slew(0.2, -0.7, 0.25) + 0.05).abs() < 1e-9, "both directions");
        // Throttle: rolling at 30% immediately, full by the ramp's end,
        // never over.
        assert_eq!(throttle_ramp(0.0), 0.3);
        let mid = throttle_ramp(1.25);
        assert!(mid > 0.5 && mid < 0.9, "{mid}");
        assert_eq!(throttle_ramp(2.5), 1.0);
        assert_eq!(throttle_ramp(60.0), 1.0);
    }

    #[test]
    fn climb_law_trades_rate_for_speed_never_descends() {
        // On speed: full rate.
        assert_eq!(climb_vs_fpm(74.0, 74.0, 700.0), 700.0);
        // 10 kt slow: backed off, still climbing.
        let slow = climb_vs_fpm(64.0, 74.0, 700.0);
        assert!(slow < 700.0 && slow >= 150.0, "{slow}");
        // Absurdly slow: pinned at the floor, never a descent.
        assert_eq!(climb_vs_fpm(40.0, 74.0, 700.0), 150.0);
        // Fast: capped at the profile's rate, no zoom climb.
        assert_eq!(climb_vs_fpm(95.0, 74.0, 700.0), 700.0);
    }

    #[test]
    fn runway_end_reject_fires_only_slow_and_short() {
        // 200 m left at 30 kt against a 55 kt Vr: no takeoff today.
        assert!(must_reject(200.0, 30.0, 55.0));
        // 200 m left but nearly at Vr: keep going, rotation is imminent.
        assert!(!must_reject(200.0, 52.0, 55.0));
        // Plenty of runway, any speed: no reject.
        assert!(!must_reject(900.0, 20.0, 55.0));
    }
}
