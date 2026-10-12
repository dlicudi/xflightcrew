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

use crate::aircraft::throttle_dataref;
use crate::autoflight::{self, Button, Dial};
use std::sync::atomic::{AtomicBool, Ordering};

use crate::runway::{along_track_nm, cross_track_nm, true_to_mag, Runway};
use crate::tools::autopilot::ApConfig;
use crate::voice::{Priority, Voice};
use crate::xplane::{XPlane, XPlaneClient};

const LAT: &str = "sim/flightmodel/position/latitude";
const LON: &str = "sim/flightmodel/position/longitude";
const PSI_TRUE: &str = "sim/flightmodel/position/psi";
#[cfg(test)]
const GS_MS: &str = "sim/flightmodel/position/groundspeed";
#[cfg(test)]
const IAS: &str = "sim/flightmodel/position/indicated_airspeed";
#[cfg(test)]
const AGL_M: &str = "sim/flightmodel/position/y_agl";
const MAG_VAR: &str = "sim/flightmodel/position/magnetic_variation";
/// Clean up the takeoff flaps this high AGL, once safely above Vy — a
/// conservative "positive climb, gear up, then flaps" point.
const FLAP_RETRACT_AGL_FT: f64 = 400.0;
#[cfg(test)]
const CMD_SERVOS_ON: &str = autoflight::cmd::SERVOS_ON;
/// Servos and flight director off: an autopilot left on by the last flight.
#[cfg(test)]
const CMD_AP_OFF: &str = autoflight::cmd::SERVOS_FDIR_OFF;
const SERVOS_ON: &str = sim::datarefs::SERVOS_ON;
#[cfg(test)]
const CMD_VS_MODE: &str = autoflight::cmd::VERTICAL_SPEED;
#[cfg(test)]
const VS_DIAL: &str = Dial::Vs.read_name();
const HDG_STATUS: &str = autoflight::names::HDG_STATUS;
const VS_STATUS: &str = autoflight::names::VS_STATUS;
/// Elevator trim, -1 (nose down) .. +1, recorded at the AP engage: the one
/// number that says whether the climb the servo is handed is flyable.
const ELEV_TRIM: &str = "sim/cockpit2/controls/elevator_trim";
const ALT_IND_FT: &str = sim::datarefs::ALTITUDE_FT;
const TAXI_STEER: &str = "xcopilot/taxi_steer";
const TAKEOFF_PITCH: &str = "xcopilot/pitch";
/// The yoke's pitch ratio as the sim holds it (read back, never written here).
const YOKE_PITCH_READ: &str = "sim/cockpit2/controls/yoke_pitch_ratio";
const TAKEOFF_ROLL: &str = "xcopilot/roll";
/// N1 per engine, percent: recorded at the thrust reduction, the evidence
/// for the declared climb lever.
const ENGN_N1: &str = "sim/flightmodel/engine/ENGN_N1_";
#[cfg(test)]
const THETA: &str = "sim/flightmodel/position/true_theta";
/// Yaw rate, deg/s, nose right positive: the roll law's damping.
const YAW_RATE: &str = "sim/flightmodel/position/R";
/// What the roll's tick reads past the position (`sim::state`, which names
/// each dataref): airspeed, the wheels (`onground_any`), the plugin's
/// pedal and yoke yields, true pitch and bank, the sim's own pitch rate
/// (`Q`, deg/s: what the rotation law damps on, measured, so it needs
/// neither differencing nor a filter; see [`RotatePitch::step`]), engine
/// 1's torque (`ENGN_TRQ`, N m) and the yaw rate.
/// The roll's tick period, ms (`sim::state::Pacer`): its old 150 ms sleep
/// plus the MacBook's rounds, the `roll` dt the laws were tuned on (bench
/// takeoffs, median 0.25 s; the iMac ran 0.34, 2026-09-30).
const ROLL_TICK_MS: u64 = 250;
/// The hand-flown climb's, ms (bench `climb_manual` dt median 0.23 s).
const CLIMB_MANUAL_TICK_MS: u64 = 230;
const ROLL_WANT: &[sim::state::F] = &[
    sim::state::F::Ias,
    sim::state::F::OnGround,
    sim::state::F::SteerYield,
    sim::state::F::Theta,
    sim::state::F::Phi,
    sim::state::F::Q,
    sim::state::F::PitchYield,
    sim::state::F::RollYield,
    sim::state::F::Trq0,
    sim::state::F::R,
];

const M_TO_FT: f64 = 3.280_84;
const MS_TO_KTS: f64 = 1.943_844;
const M_PER_NM: f64 = 1852.0;

// The runway fit moved to `runway.rs` (pillars audit 2026-10-07, stage A):
// the takeoff's gates read it here, the copilot's answers there.
#[cfg(test)]
use crate::runway::why_no_fit;
use crate::runway::{
    runway_fit, why_no_fit_cleared, NoFit, ALIGN_MAX_DEG, BEFORE_THRESHOLD_MAX_M, LATERAL_MAX_M,
};

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
/// The lever counts as at takeoff power from here, for a spooling engine.
const STAGNANT_FULL_THR: f64 = 0.95;

/// The thrust reduction height an N1-flown takeoff (`takeoff_thrust:`)
/// takes when its `takeoff_config:` declares none: the usual 1,000 ft.
const THRUST_REDUCTION_DEFAULT_FT: f64 = 1000.0;

/// Whether the lever has reached takeoff power, for the stagnation clock:
/// [`STAGNANT_FULL_THR`], or on an airframe flying its takeoff N1
/// (`takeoff_thrust:`) the lever that N1 law allows, which a derate holds
/// well under 0.95 (the Zibo's 22K, about 0.86). Pure.
pub fn lever_at_takeoff_power(thr_cmd: f64, n1_cap: Option<f64>) -> bool {
    thr_cmd >= STAGNANT_FULL_THR || n1_cap.is_some_and(|c| thr_cmd >= c - 0.01)
}

/// Is the roll stagnant? Without a declared spool (every airframe until
/// 2026-10-06) the clock runs from brake release, unchanged. A turbofan
/// declares `takeoff_spool_s`: its clock starts once the lever has reached
/// takeoff power and the spool is added, because the Zibo 737 at EGHL was
/// rejected 6 s after the lever got there, engines still spooling, and
/// surged to 20 kt after the reject. A lever that never gets there (a held
/// nose) still rejects at twice the whole allowance from brake release.
pub fn roll_stagnant(
    elapsed_s: f64,
    full_thr_at_s: Option<f64>,
    spool_s: Option<f64>,
    gs_kts: f64,
) -> bool {
    if gs_kts >= STAGNANT_GS_KTS {
        return false;
    }
    match spool_s {
        None => elapsed_s > STAGNANT_SECS,
        Some(s) => {
            full_thr_at_s.is_some_and(|t0| elapsed_s - t0 > STAGNANT_SECS + s)
                || elapsed_s > 2.0 * (STAGNANT_SECS + s)
        }
    }
}
/// Full P deflection at this much heading error on the roll — much tighter
/// than the taxi law's 30°: at roll speeds a runway is held with degrees.
pub(crate) const ROLL_FULL_DEG: f64 = 12.0;
/// Hard cap on roll deflection: a takeoff never needs full lock, and in
/// X-Plane full lock under power is a scrubbing swerve.
pub(crate) const MAX_ROLL_STEER: f64 = 0.85;
/// Lead time constant of the damping term: the command answers where the
/// error is GOING, which is what kills the counter-steer oscillation when
/// each observation arrives a network round trip stale.
pub(crate) const STEER_LEAD_S: f64 = 0.4;
/// Max steering sweep rate, PER SECOND — dt-scaled, so the real tick rate
/// never sets the actuator speed. The old per-tick limit (0.25) at the
/// measured half-second tick swept 0.5/s: the rudder lagged the error by
/// over a second, and that lag WAS the left-right weave — sortie four's
/// telemetry shows the steer chasing in exact 0.25 steps the whole roll,
/// a textbook slew-induced limit cycle. 1.5/s crosses the full range in
/// just over a second; the low-speed cap still owns the parked case.
pub(crate) const STEER_SLEW_PER_S: f64 = 1.5;
/// Integrator gain (deflection per degree-second) and cap: the standing
/// rudder the yaw moment demands. Second sortie: pure P+D settled at a
/// STANDING 18 degrees of error against full-power P-factor — the error a
/// proportional law must keep to hold any rudder at all — then the
/// authority taper starved it and the aircraft left the pavement 39
/// degrees off. The integrator learns that trim so the error can go to
/// zero with the rudder still held.
pub(crate) const STEER_TRIM_RATE: f64 = 0.01;
pub(crate) const STEER_TRIM_MAX: f64 = 0.3;
/// The error is clamped to this in the trim update: the integrator learns
/// a STANDING offset, slowly — sortie four's 39 degree transient wound the
/// trim at 0.4/s to its cap in two seconds, and the stale trim then fueled
/// the opposite overshoot. Clamped, the learn rate tops out at 0.06/s.
pub(crate) const STEER_TRIM_ERR_CLAMP_DEG: f64 = 6.0;
/// Above this the rudder is aerodynamic and the trim scales with 1/v².
pub(crate) const TRIM_SCALE_FROM_KTS: f64 = 40.0;
/// The roll law SCHEDULED on a declared rudder
/// ([`crate::aircraft::RollRudderProfile`]): the gain per degree is this
/// closure rate divided by the yaw rate a unit of steer holds at the speed,
/// so the heading closes at the same pace at 40 kt and at Vr (never more per
/// degree than [`ROLL_FULL_DEG`] allows). Damped by [`STEER_LEAD_S`] of the
/// sim's MEASURED yaw rate. Chosen over the EGKK-fitted plant with its rudder
/// at half, as declared and at two and a half times: 1.3 rang at 3.5 times.
pub(crate) const RUDDER_CLOSE_PER_S: f64 = 1.0;
/// The trim's corner, per second of the same gain: it takes a steady
/// crosswind in about five seconds (EGKK 26L: 0.2 of steer from 50 kt).
/// 0.45 and up wound up on a recovery from an offset with a weak rudder.
pub(crate) const RUDDER_TRIM_PER_S: f64 = 0.3;
/// ...and its cap: about 0.23 held an 8 kt crosswind at 60 kt.
pub(crate) const RUDDER_TRIM_MAX: f64 = 0.4;
/// Speed-authority floor. Sortie two's 0.3 floor starved the STANDING
/// P-factor rudder — but that job belongs to the trim integrator now,
/// which adds OUTSIDE this taper. So the P term can shrink properly with
/// speed again: at 60-80 knots the rudder is powerful, and sortie four's
/// 0.65 floor swung 0.6 of deflection over 8 degree errors — the muscle
/// behind the weave.
pub(crate) const STEER_AUTH_FLOOR: f64 = 0.35;
/// Deflection cap at a standstill, ramping to [`MAX_ROLL_STEER`] by
/// [`LOW_SPEED_FULL_KTS`]. Under takeoff power the prop wash makes the
/// rudder and nosewheel HYPER-effective at low speed — the third sortie
/// commanded 0.85 at brake release to fix a 14 degree error and the
/// aircraft spot-turned 70 degrees through the target at 25 deg/s,
/// pirouetting on the runway until the stagnation guard rejected. Small
/// corrections parked; the full cap arrives with the roll.
pub(crate) const LOW_SPEED_STEER_CAP: f64 = 0.12;
pub(crate) const LOW_SPEED_FULL_KTS: f64 = 15.0;
/// The trim integrator only learns above this groundspeed: it exists for
/// the P-factor moment on the ROLL, and letting it wind up during a
/// low-speed swing doubles every overshoot (sortie three hit its cap in
/// both directions inside the pirouette).
pub(crate) const TRIM_MIN_GS_KTS: f64 = 10.0;
/// Power feed-forward: right rudder led IN WITH the throttle, the way a
/// pilot's foot moves with the hand — the yaw moment is proportional to
/// power, so the compensation rides the ramp instead of waiting for an
/// error to react to. Fades partially with speed (the moment weakens as
/// airflow straightens); the trim integrator learns the residual.
/// Sign assumes a clockwise prop (yaw left, lead right) — every current
/// profile; a counter-clockwise airframe will need this in the profile.
pub(crate) const POWER_LEAD_MAX: f64 = 0.25;
pub(crate) const POWER_LEAD_FADE_KTS: f64 = 40.0;
/// Throttle advances to full over this long — slamming it at a standstill
/// is the maximum yaw moment at the exact moment steering authority is
/// highest and airflow over the rudder is zero.
/// A pilot advances the lever over seconds, never in one movement: 2.5 s
/// from 30% to full was the "0 to 100" the pilot called out (2026-09-25).
const THROTTLE_RAMP_S: f64 = 6.0;
/// Centreline correction: degrees of cut per meter of offset, clamped. The
/// figure is quoted AT [`XTK_GAIN_REF_KTS`] and scaled down above it - see
/// [`roll_target_heading`].
pub(crate) const XTK_GAIN_DEG_PER_M: f64 = 0.4;
pub(crate) const XTK_MAX_DEG: f64 = 8.0;
/// The pace the cut's gain was fitted at. Below it nothing changes; above it
/// the gain is scaled by ref/gs, which holds the lateral CLOSURE TIME
/// constant instead of the angle.
pub(crate) const XTK_GAIN_REF_KTS: f64 = 40.0;
/// How far ahead the cut looks, in seconds of cross-track closure. The lead
/// the outer loop needs to cover the inner loop's lag: the nose takes a
/// second or two to reach a commanded cut and as long again to give it back.
pub(crate) const XTK_LEAD_S: f64 = 2.0;
/// Rotation pitch attitude, degrees nose up — the elevator flies the nose
/// here and HOLDS it through the initial climb; at full power in a light
/// single that settles near Vy. Rotation is ELEVATOR, not the autopilot:
/// the AP's VS mode syncs to the CURRENT vertical speed at engage — zero
/// on the runway, whatever the dial was pre-set to — so sortie five called
/// rotate at 55 knots, the AP held the nose down, and the aircraft
/// levitated off at 98 with 250 m of runway left.
const ROTATE_PITCH_DEG: f64 = 8.0;
/// ...reached at this RATE, degrees per second. A rotation is a rate, not a
/// step, and treating it as a step is what struck an MD-82's tail at Luton
/// (2026-07-28): the target appeared 8° away in one tick, the integrator wound
/// to its cap pulling for it, and the nose arrived with enough momentum to
/// sail through the target to 10.6° at 21 ft — on a fuselage whose geometry
/// runs out around 11. Ramping the commanded attitude from wherever the nose
/// was keeps the error small, so the elevator never winds up in the first
/// place. 2.5°/s is the rate airline procedure asks a pilot to rotate at.
const ROTATE_RATE_DEG_S: f64 = 2.5;
/// Integrating pitch law: deflection per degree-second of attitude error,
/// a lead on the pitch rate for damping, and caps. Integrating, so it
/// holds whatever standing elevator the trim state demands with the
/// attitude ON target — the same reason the roll law grew its trim.
/// Proportional elevator per degree of (lead-compensated) attitude error —
/// the path that answers immediately and adds no phase lag.
const PITCH_P_PER_DEG: f64 = 0.03;
/// Integral gain, per degree per second. Only has to find the STANDING pull,
/// so it is a fifth of the old pure-integrator gain: at that speed it trims
/// rather than flies.
const PITCH_TRIM_PER_DEG_S: f64 = 0.01;
/// The damping lead, seconds. Applied to the rate ERROR (how far off the
/// COMMANDED rotation rate the nose is), not the raw pitch rate: damping the
/// raw pitch rate would fight the rotation itself, since a nose coming up at
/// exactly the commanded 2.5°/s is doing precisely what was asked.
///
/// It was 1.0 s, and that is the far side of a cliff. Swept against a
/// second-order pitch plant with the loop's real transport delay
/// (`the_rotation_settles_instead_of_ringing`), the worst-case attitude spread
/// across the plant uncertainty runs 2.3-2.4° anywhere in 0.3-0.5 s, then
/// breaks to 9.7° at 0.8 and 10.2° at 1.0 — the lead term stops damping the
/// motion and starts driving it. 0.3 rather than 0.5 because it is the only
/// value that also survives the FALLBACK rate (see `RotatePitch::step`): with
/// a differenced attitude the same sweep gives 4.5° at 0.3 and 14.6° at 0.5.
/// A failed read must cost accuracy, not stability.
const PITCH_LEAD_S: f64 = 0.3;
/// Margin below an airframe's tail-strike attitude that the rotation will not
/// command past. The target (8°) normally sits below this anyway — the clamp
/// exists for the OVERSHOOT, which is what actually strikes.
const TAIL_STRIKE_MARGIN_DEG: f64 = 2.0;
const PITCH_MAX: f64 = 0.65;
/// Nose-DOWN authority the rotation law may command. The GA figure, and the
/// default for anything whose profile declares nothing.
///
/// A tenth of the travel is plenty on a light single, where the rotation only
/// ever needs holding back, and deliberately small so a law that is about to
/// ring cannot shove the nose at the runway. On a jet at takeoff thrust with
/// nose-up trim it is not authority at all: BIKF-EGPH (2026-09-20) commanded
/// 8 degrees and watched the nose go to 22.4, the law pinned at this limit
/// for the whole divergence with the pitch RATE still building — it wanted
/// -0.38 and could ask for -0.10. Only the autopilot engaging at 1,000 ft
/// ended it. See [`RotatePitch::with_authority`].
const PITCH_MIN: f64 = -0.10;
/// Hand the established climb to the autopilot this high — clear of ground
/// effect, wings level, speed building; the VS dial is written AFTER the
/// mode engages, because engage syncs the dial to the current rate.
const AP_ENGAGE_AGL_FT: f64 = 300.0;
/// The hand-flown climb holds the WINGS too — between liftoff and the AP
/// engage, full-power torque rolls a light single left with nobody on the
/// ailerons. Bank leans toward runway heading, capped shallow.
const CLIMB_BANK_MAX_DEG: f64 = 8.0;
/// The bank the float under the 8 ft liftoff may use toward the track,
/// degrees: a wingtip 19 ft out drops a foot at 3 (the SR22's half span).
const FLOAT_BANK_MAX_DEG: f64 = 3.0;
const BANK_PER_HDG_ERR: f64 = 1.0;
/// Aileron law: P on bank error (aileron commands roll RATE, so
/// proportional converges without an integrator), rate-lead damped.
pub(crate) const AIL_FULL_BANK_DEG: f64 = 15.0;
pub(crate) const AIL_LEAD_S: f64 = 0.3;
pub(crate) const AIL_MAX: f64 = 0.5;
pub(crate) const AIL_SLEW_PER_S: f64 = 2.0;
/// Default hand-back height above the field, feet (a normal departure); a
/// circuit overrides it lower via `Params::handoff_agl_ft`.
pub const DEFAULT_HANDOFF_AGL_FT: f64 = 1000.0;
/// The next flap setting to select, retracting: the detent ONE step cleaner
/// than `current`, taken only once the speed allows it.
///
/// `schedule` is `(handle ratio, manoeuvring speed)` from the profile's
/// detents. One step at a time, the way a crew does it and the way the flaps
/// physically travel — never "the cleanest this speed allows", which on a
/// fast clean-up would skip a detent the aircraft is still passing through.
///
/// `None` when there is nothing cleaner, or when the next step is still too
/// fast for us. An empty schedule means the airframe declares no manoeuvring
/// speeds and keeps the old one-shot clean-up.
pub fn flap_retract_step(schedule: &[(f64, f64)], current: f64, ias: f64) -> Option<f64> {
    schedule
        .iter()
        .filter(|(ratio, _)| *ratio < current - 1e-6)
        .max_by(|a, b| a.0.total_cmp(&b.0))
        .filter(|(_, min_kts)| ias >= *min_kts)
        .map(|(ratio, _)| *ratio)
}

/// Height above the field at which the takeoff switches the landing and
/// taxi lights off (`lights::after_climb`; the circuit and full auto's
/// follower do the same from their own loops for a flight they joined).
pub const LIGHTS_OFF_AGL_FT: f64 = 1000.0;

/// [`flap_retract_step`], paced on the flaps themselves: the step is taken
/// only once the flaps `deployed` have arrived at the `commanded` detent.
/// `None` for a failed read (no measurement, no step). Pure.
pub fn paced_flap_step(
    schedule: &[(f64, f64)],
    commanded: f64,
    deployed: Option<f64>,
    ias: f64,
) -> Option<f64> {
    flap_retract_step(schedule, commanded, ias)
        .filter(|_| deployed.is_some_and(|d| crate::takeoff_config::flaps_arrived(d, commanded)))
}

/// A deploy reading this still, over [`FLAP_SETTLE_S`] of reads, has stopped.
pub const FLAP_SETTLE_BAND: f64 = 0.01;
/// How long a deploy reading must hold still to count as settled, seconds.
pub const FLAP_SETTLE_S: f64 = 4.0;
/// A gap between reads longer than this starts the settle clock again.
const FLAP_SETTLE_GAP_S: f64 = 2.0;
/// A withheld step is blackboxed at most this often, seconds.
const FLAP_WITHHELD_LOG_S: f64 = 10.0;

/// The flaps' own pace for a retraction walk (#169). The flaps have arrived
/// at the commanded detent when the deploy reading is within
/// [`crate::takeoff_config::flaps_arrived`] of it, OR when it has moved off
/// the detent the last step left and stopped ([`FLAP_SETTLE_BAND`] over
/// [`FLAP_SETTLE_S`] of reads). The stock A330 (LEMG 13, sortie
/// 1790621184-full_auto, 2026-09-28) selected CONF 1 at +360 s and never
/// stepped again, the speed past the 210 retract speed for most of a
/// minute: its deploy ratio never came within 0.03 of the handle. A
/// reading that never left the old detent is not settled (a jammed flap is
/// not an arrival). A retraction reading already past the commanded detent
/// has arrived at once (the A330's CONF 1, 2026-09-29). A failed read is no
/// measurement.
#[derive(Debug, Clone, Default)]
pub struct FlapPace {
    from: Option<f64>,
    reads: Vec<(f64, f64)>,
    withheld_at: Option<f64>,
}

impl FlapPace {
    /// A step was taken from `from`: the settle clock starts again.
    pub fn stepped(&mut self, from: f64) {
        *self = Self {
            from: Some(from),
            ..Self::default()
        };
    }

    /// Whether the flaps count as arrived at `commanded`, fed the reading
    /// taken at `t_s` (seconds on any monotonic clock).
    pub fn arrived(&mut self, t_s: f64, commanded: f64, deployed: Option<f64>) -> bool {
        let Some(d) = deployed.filter(|d| d.is_finite()) else {
            return false;
        };
        if crate::takeoff_config::flaps_arrived(d, commanded) {
            return true;
        }
        // A retraction reading already PAST the commanded detent, cleaner
        // than it, has arrived: nothing is left out that the next step's
        // speed must respect. The A330's CONF 1 is slats: its reading ran
        // 0.106 then 0.011 under the 0.25 handle, still falling, and the
        // walk waited 15 s for it to stop while the crosswind ran to 224 kt
        // over the 215 placard (LEMG 30, sortie 1790698132-full_auto,
        // 2026-09-29).
        if self.from.is_some_and(|f| f > commanded + 1e-6)
            && d <= commanded + crate::takeoff_config::FLAP_ARRIVED
        {
            return true;
        }
        if self
            .reads
            .last()
            .is_some_and(|(t, _)| t_s < *t || t_s - t > FLAP_SETTLE_GAP_S)
        {
            self.reads.clear();
        }
        self.reads.push((t_s, d));
        while self.reads.len() >= 2 && t_s - self.reads[1].0 >= FLAP_SETTLE_S {
            self.reads.remove(0);
        }
        let span = t_s - self.reads[0].0;
        let (lo, hi) = self
            .reads
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), (_, r)| {
                (lo.min(*r), hi.max(*r))
            });
        let moved = self
            .from
            .is_some_and(|f| !crate::takeoff_config::flaps_arrived(d, f));
        moved && span >= FLAP_SETTLE_S && hi - lo < FLAP_SETTLE_BAND
    }

    /// The deploy reading as [`paced_flap_step`] should judge it: the
    /// commanded detent itself once [`Self::arrived`], else the reading.
    pub fn judged(&mut self, t_s: f64, commanded: f64, deployed: Option<f64>) -> Option<f64> {
        if self.arrived(t_s, commanded, deployed) {
            Some(commanded)
        } else {
            deployed
        }
    }

    /// Whether a step the speed allows but the pace withheld is due a
    /// blackbox line: the first, then every [`FLAP_WITHHELD_LOG_S`].
    pub fn withheld_log_due(&mut self, t_s: f64) -> bool {
        let due = self
            .withheld_at
            .is_none_or(|t| t_s < t || t_s - t >= FLAP_WITHHELD_LOG_S);
        if due {
            self.withheld_at = Some(t_s);
        }
        due
    }
}

/// The call for a retraction step to `to`, named by its detent the way a
/// crew calls it: "Flaps one.", "Flaps up.". Without a name it is the old
/// vague line. Two steps 0.5 s apart both said "Flaps coming up a stage."
/// (Alghero, 2026-09-27). Pure.
pub fn flap_step_line(to: f64, spoken: Option<&str>) -> String {
    if to <= 1e-6 {
        return "Flaps up.".into();
    }
    match spoken.map(str::trim).filter(|s| !s.is_empty()) {
        Some(name) => format!("Flaps {name}."),
        None => "Flaps coming up a stage.".into(),
    }
}

/// Raise the gear (when there is one) above this, climbing.
const GEAR_UP_AGL_FT: f64 = 200.0;
/// Above this the gear comes up whatever its warning says: drag and the
/// gear placard matter more than a horn. The handoff raises it too.
const GEAR_UP_LATEST_AGL_FT: f64 = 1000.0;
/// Knots over the warning speed before the gear comes up, so it does not
/// sound on the first gust.
const GEAR_WARN_MARGIN_KTS: f64 = 3.0;
/// Knots under the gear placard at which the gear comes up regardless.
const GEAR_VLO_MARGIN_KTS: f64 = 10.0;

/// The airframe's gear-warning envelope, from its profile or its `.acf`:
/// republished on every swap, including to `None`
/// ([`crate::flight_state::publish_airframe`]); the detector publishes the
/// `.acf`'s envelope over a silent profile once per airframe.
pub fn publish_gear_warning(w: Option<crate::aircraft::GearWarning>) {
    crate::aircraft::update_airframe_facts(|a| a.gear_warning = w);
}

pub(crate) fn gear_warning() -> Option<crate::aircraft::GearWarning> {
    crate::aircraft::airframe_facts().gear_warning
}

/// Is it time to raise the gear after takeoff? Above [`GEAR_UP_AGL_FT`]
/// climbing, once raising it will not sound the airframe's own gear warning
/// (`warning`, at takeoff thrust so only the speed and flap clauses apply),
/// and in any case by [`GEAR_UP_LATEST_AGL_FT`] or near the gear placard.
/// The SF50's warning fires below 125 kt with its 0.5 takeoff flap: the flat
/// 200 ft rule raised the gear at 122 kt and the horn ran to 335 ft on every
/// departure (ETND 26, 2026-09-23). Pure.
pub fn gear_up_due(
    agl_ft: f64,
    ias_kts: f64,
    flap_ratio: f64,
    warning: Option<crate::aircraft::GearWarning>,
    vlo_kts: Option<f64>,
) -> bool {
    if agl_ft <= GEAR_UP_AGL_FT {
        return false;
    }
    if agl_ft >= GEAR_UP_LATEST_AGL_FT
        || vlo_kts.is_some_and(|v| ias_kts >= v - GEAR_VLO_MARGIN_KTS)
    {
        return true;
    }
    match warning {
        None => true,
        Some(w) => !w.would_sound(ias_kts - GEAR_WARN_MARGIN_KTS, flap_ratio, 1.0),
    }
}

/// True while the takeoff loop runs. Cleared by the loop, the release path
/// in main ("my controls" — silent stand-down), or the tool's off switch.
pub(crate) static RUN: sim::run::Slot = sim::run::Slot::new();
pub fn running() -> bool {
    RUN.running()
}
pub fn outcome() -> Option<sim::run::Outcome> {
    RUN.outcome()
}
pub fn cancel() {
    RUN.cancel();
    ACTIVE.store(false, Ordering::SeqCst);
}

pub static ACTIVE: AtomicBool = AtomicBool::new(false);
/// Set by the `auto_takeoff` tool's off switch: the loop REJECTS (throttle
/// idle, brake pulse) rather than standing down silently.
pub static REJECT: AtomicBool = AtomicBool::new(false);

/// One-line status for the dashboard's "managing" panel.
/// The takeoff's live line for the panel: the roll with its speed against
/// Vr, the rotation, the climb against the handoff height.
pub static DETAIL: crate::auto_status::Detail = crate::auto_status::Detail::new();

pub fn status() -> Option<String> {
    ACTIVE
        .load(Ordering::Relaxed)
        .then(|| DETAIL.line("auto takeoff"))
}

/// Everything the loop needs, resolved by the arming tool from the profile.
pub struct Params {
    pub apt_dat: String,
    pub cfg: ApConfig,
    /// Rotate speed, KIAS.
    pub vr_kts: f64,
    /// Enroute climb speed, KIAS. The CIRCUIT sizes its pattern off this
    /// (`aircraft::pattern_speed_kts` reads it straight off these params);
    /// the takeoff's own climb law flies `initial_climb_kts`.
    pub vy_kts: f64,
    /// Initial climb speed, KIAS — what the climb law protects between
    /// liftoff and the handoff, and the speed the takeoff flaps come up at.
    /// Equal to `vy_kts` on everything that does not declare its own (see
    /// `ReferenceProfile::initial_climb_kts`).
    pub initial_climb_kts: f64,
    /// Ceiling on the commanded climb, fpm.
    pub climb_fpm: f64,
    pub retract_gear: bool,
    /// Flap-handle ratio to set for takeoff, retracted once safely climbing.
    /// `None` leaves the flaps as the pilot left them.
    pub takeoff_flap_ratio: Option<f64>,
    /// `(handle ratio, manoeuvring speed)` for every detent that declares one,
    /// the climb-out retraction schedule. Empty on an airframe with no
    /// declared speeds, which keeps the one-shot clean-up.
    pub flap_schedule: Vec<(f64, f64)>,
    /// The runway END the tower cleared ("09R"), for the line-up crosscheck.
    /// `None` when no ATC is running or no runway is on the clearance.
    pub cleared_runway: Option<String>,
    /// Runway needed for the roll, meters (POH distance with margin).
    pub needed_m: f64,
    /// The altitude the CALLER will level at, feet, selected on the altitude
    /// dial before the roll (the circuit's pattern altitude). `None` leaves a
    /// dial the pilot set and lifts one the climb could never capture
    /// (`takeoff_config::initial_altitude_ft`).
    pub level_ft: Option<f64>,
    /// Height AGL to hand back at, feet. 1000 for a normal departure; a circuit
    /// hands off LOWER (~500) so it turns crosswind while still climbing,
    /// instead of running four kilometres straight ahead to reach 1000 first.
    pub handoff_agl_ft: f64,
    /// The attitude at which this airframe's tail touches the runway, degrees.
    /// The rotation will not command within [`TAIL_STRIKE_MARGIN_DEG`] of it.
    /// `None` on airframes short enough not to have one worth naming.
    pub tail_strike_deg: Option<f64>,
    /// This airframe's nose-down elevator limit, 0..1. `None` = the GA tenth.
    pub pitch_authority_down: Option<f64>,
    /// Never-exceed / Vmo, KIAS — the climb's OVERSPEED protection. `None`
    /// keeps the full-power climb every airframe flew before this existed.
    pub vne_kts: Option<f64>,
    /// True when an ORCHESTRATOR armed this takeoff and will keep flying after
    /// the handoff (the circuit, full auto, a procedure) — as opposed to the
    /// pilot asking for a takeoff on its own.
    ///
    /// It decides one thing, and that one thing is the most important sentence
    /// this module speaks. The handoff line said "your aircraft"
    /// UNCONDITIONALLY, so a circuit's takeoff told the pilot they had control
    /// at the exact moment the circuit was taking it. A pilot who believes
    /// that either fights the automation for the yoke or, worse, sits back
    /// while nobody flies — and the whole trust model here rests on the
    /// copilot being right about who has the aircraft.
    pub handoff_to_caller: bool,
    /// The caller walks the flap schedule after the handoff (the circuit,
    /// which takes the aircraft at 500 ft, under a jet's acceleration
    /// height): the handoff leaves the flaps where they are and hands them
    /// over ([`take_climb_handover`]) instead of cleaning up in one shot.
    pub flaps_to_caller: bool,
    /// The speed the governor is armed at when the PILOT takes the handoff
    /// (the profile's cruise IAS, capped to 250 below 10,000 ft), and the
    /// base URL it is armed over. `None` leaves the throttle where the climb
    /// left it. Never used when an orchestrator takes the aircraft: it owns
    /// the throttle from there.
    pub handoff_speed_kts: Option<f64>,
    pub base_url: String,
    /// The airframe's park-brake valve SET sense, for the brakes-off before the
    /// roll. `None` = the airframe models no valve, and the ratio is the whole
    /// of the parking brake.
    pub park_brake_valve_set: Option<f64>,
}

/// What to SAY about a failed fit, and whether it is urgent (a Warning: we
/// are occupying a runway we cannot depart from). One wording for the spoken
/// takeoff and for full auto's line-up check, so neither can call an aircraft
/// on the centreline "not on the runway".
pub fn no_fit_line(why: &NoFit, can_line_up: bool) -> (String, bool) {
    match why {
        // The runway itself is too short. Say so plainly, with both
        // numbers, and do NOT send the pilot to line up or backtrack —
        // neither can add pavement.
        NoFit::RunwayTooShort {
            id,
            length_m,
            needed_m,
        } => (
            format!(
                "Runway {} is {:.0} metres and I need {:.0} for this aircraft. It's too \
                 short — lining up or backtracking won't change that. Your takeoff if \
                 you want it.",
                crate::crew_radio::spell_runway(id),
                length_m,
                needed_m
            ),
            true,
        ),
        // ON the runway with too little ahead: we are occupying a strip
        // we cannot depart from, which is a vacate, not a line-up.
        NoFit::TooLittleLeft {
            id,
            remaining_m,
            needed_m,
        } => (
            format!(
                "We're lined up on {} but at the far end — {:.0} metres left and I need \
                 {:.0}. Backtrack or vacate; I can't take off from here.",
                crate::crew_radio::spell_runway(id),
                remaining_m,
                needed_m
            ),
            true,
        ),
        // Across the pavement: the worst place to be and the one the old
        // message actively denied.
        NoFit::NotAligned { id, err_deg } if *err_deg < 90.0 => (
            format!(
                "We're on {} but {:.0} degrees off the centreline. Line us up, or vacate \
                 if there's traffic.",
                crate::crew_radio::spell_runway(id),
                err_deg
            ),
            true,
        ),
        NoFit::ShortOfThreshold { .. } | NoFit::NotAligned { .. } | NoFit::Offset { .. }
            if can_line_up =>
        {
            (
                "We're not on the runway yet. Say line us up, then give me the takeoff."
                    .to_string(),
                false,
            )
        }
        // Not near any usable pavement. Name the strip we looked for and how
        // far off it is: "I couldn't get us onto the runway" told the pilot
        // nothing while the aircraft sat exactly where the tower sent it.
        NoFit::NoneNear { id, lateral_m } if !id.is_empty() => (
            format!(
                "Runway {} is {:.0} metres away from us, too far for me to line up on.",
                crate::crew_radio::spell_runway(id),
                lateral_m
            ),
            false,
        ),
        _ => (
            "We're not lined up on a runway I can use. Line us up and ask me again.".to_string(),
            false,
        ),
    }
}

/// The string crosscheck's escape hatch for duplicated pavement: apt.dat can
/// carry ONE physical strip as two rows under different designators —
/// Stansted's 3,052 m 04/22 and its 1,048 m 04C/22C are collinear to under a
/// metre of cross-track, the author's model of the field's center
/// designators. Lined up there, [`runway_fit`] names the LONG row (the short
/// one fails the length gate) while the tower cleared "two two center", and
/// the string compare refused a takeoff from exactly the cleared position —
/// three times in a row, EGSS 2026-07-29. If the CLEARED designator itself
/// fits our position and heading (the fit's own gates, minus length — an
/// administrative sub-segment's declared distance is not the pavement), we
/// are standing on the cleared runway whatever the fitted row is called. A
/// reciprocal still fails on heading and a true parallel on cross-track, so
/// the backwards-line-up trap this check exists for is intact.
pub fn cleared_fits_here(
    cands: &[Runway],
    cleared: &str,
    lat: f64,
    lon: f64,
    psi_true: f64,
) -> bool {
    cands.iter().any(|r| {
        runway_matches(&r.id, cleared)
            && crate::angles::abs_diff_deg(psi_true, r.course_deg) <= ALIGN_MAX_DEG
            && (cross_track_nm(lat, lon, r) * M_PER_NM).abs() <= LATERAL_MAX_M
            && along_track_nm(lat, lon, r) * M_PER_NM <= BEFORE_THRESHOLD_MAX_M
    })
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

// The ground-track laws moved to `roll_steer.rs` (phase 2); re-exported so
// every `autotakeoff::` path keeps compiling.
pub use crate::roll_steer::*;

/// A jet's climb-out (the type is the airframe snapshot's, in `aircraft`).
pub use crate::aircraft::ClimbOut;

/// Spawn the takeoff loop. The tool has already gated replay, ground,
/// clearance state, and the steer channel; the loop re-validates the runway
/// itself (the apt.dat scan is too slow for the tool's reply).
pub fn start_scoped(
    base_url: String,
    voice: Option<Voice>,
    p: Params,
    parents: Vec<std::sync::Arc<AtomicBool>>,
) -> bool {
    let Some(lease) = RUN.begin() else {
        return false;
    };
    if ACTIVE.swap(true, Ordering::Relaxed) {
        return false;
    }
    REJECT.store(false, Ordering::Relaxed);
    // A cancel racing the arm lands between the claim and the clear above:
    // it flips ACTIVE off after our swap, and the line above just erased its
    // flag. The cancel wins; verify the claim survived before anything flies
    // (the same claim-clear-verify order every automation start uses).
    if !ACTIVE.load(Ordering::Relaxed) {
        return false;
    }
    std::thread::spawn(move || {
        let _lease = lease.with_cleanup(&ACTIVE, || {});
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            ACTIVE.store(false, Ordering::Relaxed);
            return;
        };
        rt.block_on(fly(
            XPlaneClient::new(base_url)
                .with_cancel(_lease.cancel.clone())
                .with_parents(parents),
            voice,
            p,
        ));
        ACTIVE.store(false, Ordering::Relaxed);
    });
    true
}

fn alert(voice: &Option<Voice>, priority: Priority, line: &str) {
    tracing::info!(target: "xvoicelayer::autotakeoff", spoke = %line, "copilot");
    if let Some(v) = voice {
        v.alert(priority, line);
    }
}

// The takeoff's laws and its loop in their own files, whole and unchanged
// (refactor phase 4); every path stays `crate::autotakeoff::...` through
// these re-exports.
mod roll;
pub use roll::*;
mod rotate;
pub use rotate::*;
mod climb;
pub use climb::*;
mod reject;
pub use reject::*;
mod run;
pub use run::*;

// Maximum braking moved beside the regular brakes (`tools::BRAKES_MAX`,
// pillars audit 2026-10-07, stage A).
use crate::tools::BRAKES_MAX;

/// The recorded LEMG 13 circuit climb-outs and the 737's VS mode, shared by
/// this module's tests and the circuit's (#150).
#[path = "autotakeoff/climb_out_bench_tests.rs"]
#[cfg(test)]
pub(crate) mod climb_out_bench;

#[cfg(test)]
mod tests;

/// The Zibo 737's rotation, fitted to LEMG 30 (2026-10-07) and flown.
#[cfg(test)]
mod zibo_rotation_tests;

/// The Zibo 737 from the rotation to the liftoff: the track's lag behind the
/// heading fitted to LEMG 30 (2026-10-07), the rolls replayed and the plant
/// flown through the rotation.
#[cfg(test)]
mod zibo_liftoff_tests;

/// The pilot's reject at LEMG 13 (2026-09-28, sortie 1790627884-full_auto,
/// stock 737), replayed through a plant: the stop is flown, the throttles
/// held at idle, the parking brake set once stopped, and only then "your
/// aircraft".
#[cfg(test)]
mod reject_stop_tests;

#[cfg(test)]
mod stagnant_tests {
    use super::roll_stagnant;

    /// Every airframe without a declared spool keeps the old clock.
    #[test]
    fn no_spool_keeps_the_brake_release_clock() {
        assert!(!roll_stagnant(11.9, None, None, 0.0));
        assert!(roll_stagnant(12.1, None, None, 0.0));
        assert!(!roll_stagnant(30.0, None, None, 18.0));
    }

    /// The EGHL Zibo reject: lever up at 6.7 s, rejected at 12.6 s with the
    /// engines still spooling. With the spool declared it waits.
    #[test]
    fn a_spooling_jet_is_judged_from_the_lever_at_takeoff_power() {
        let spool = Some(8.0);
        assert!(!roll_stagnant(12.6, Some(6.7), spool, 0.1));
        assert!(!roll_stagnant(26.0, Some(6.7), spool, 3.0));
        assert!(roll_stagnant(27.0, Some(6.7), spool, 3.0));
        assert!(!roll_stagnant(27.0, Some(6.7), spool, 25.0));
        // A lever that never got there still rejects, later.
        assert!(!roll_stagnant(39.0, None, spool, 0.0));
        assert!(roll_stagnant(40.5, None, spool, 0.0));
    }

    /// A derate's lever stops short of 0.95: its takeoff power is the lever
    /// its N1 law allows, so the spool clock still starts there. Without
    /// the law, 0.95 as before.
    #[test]
    fn a_derated_lever_at_its_n1_cap_is_takeoff_power() {
        use super::lever_at_takeoff_power as at;
        assert!(at(0.95, None));
        assert!(!at(0.86, None));
        assert!(at(0.855, Some(0.86)));
        assert!(!at(0.80, Some(0.86)));
        assert!(at(0.96, Some(0.86)));
    }
}
