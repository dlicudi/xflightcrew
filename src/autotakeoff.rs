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

use crate::autoflight::{self, Button, Dial};
use std::sync::atomic::{AtomicBool, Ordering};

use crate::autoland::{along_track_nm, cross_track_nm, true_to_mag, Runway};
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
/// The thrust lever THIS airframe obeys — not always the stock one (the
/// Citation X tracks `throttle_jet_rev_ratio`, so every write to
/// `throttle_ratio_all` went nowhere and it flew a whole pattern at takeoff
/// thrust). Resolved per call, so a profile swap takes effect immediately.
fn throttle_dref() -> String {
    crate::aircraft::throttle_dataref()
}
const GEAR_HANDLE: &str = "sim/cockpit2/controls/gear_handle_down";
const FLAP_HANDLE: &str = "sim/cockpit2/controls/flap_ratio";
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

/// Lined up means aligned within this many degrees of a runway's course.
const ALIGN_MAX_DEG: f64 = 20.0;
/// ... and within this many meters of its centreline.
const LATERAL_MAX_M: f64 = 40.0;
/// ... and no more than this far short of the threshold (a lead-on stub
/// aligned with the strip still counts; the far side of the airport doesn't).
const BEFORE_THRESHOLD_MAX_M: f64 = 100.0;
/// Runway remaining below this at arm time is not a takeoff, whatever the
/// POH says — floor for airframes with no published takeoff distance.
pub const MIN_REMAINING_M: f64 = 500.0;
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
/// The tick the stiff wings-level gain was set on, seconds (see
/// `WingsLevel::step`).
pub(crate) const STIFF_TUNED_TICK_S: f64 = 0.3;
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

/// The airframe's gear-warning envelope, from its profile or its `.acf`.
static GEAR_WARNING: std::sync::Mutex<Option<crate::aircraft::GearWarning>> =
    std::sync::Mutex::new(None);

/// Republished on every swap, including to `None`
/// ([`crate::flight_state::publish_airframe`]); the detector publishes the
/// `.acf`'s envelope over a silent profile once per airframe.
pub fn publish_gear_warning(w: Option<crate::aircraft::GearWarning>) {
    *GEAR_WARNING.lock().unwrap_or_else(|e| e.into_inner()) = w;
}

pub(crate) fn gear_warning() -> Option<crate::aircraft::GearWarning> {
    *GEAR_WARNING.lock().unwrap_or_else(|e| e.into_inner())
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
    /// (`circuit::pattern_speed_kts` reads it straight off these params);
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

/// Fit the aircraft's position and heading to a runway end: which strip are
/// we lined up on, and how much of it is left ahead of the nose. `None`
/// when nothing fits — not on a runway, not aligned, or not enough left.
pub fn runway_fit(
    cands: &[Runway],
    lat: f64,
    lon: f64,
    psi_true: f64,
    needed_m: f64,
) -> Option<(Runway, f64)> {
    let mut best: Option<(Runway, f64, f64)> = None;
    for r in cands {
        let err = crate::angles::abs_diff_deg(psi_true, r.course_deg);
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

/// Why [`runway_fit`] found nothing — the nearest near-miss, so a refusal can
/// name the problem instead of guessing at it.
///
/// The four failures want four different actions from the pilot, and the
/// caller used to collapse them into one line. At Edinburgh (2026-09-20) the
/// aircraft sat on runway 24, aligned to within a degree and dead on the
/// centreline, 2,408 m along a 2,549 m strip — 141 m of runway ahead. The
/// refusal said "We're not on the runway yet. Say line us up", while the
/// tower was simultaneously saying "you are on runway two four without
/// clearance, traffic on short final, exit immediately". The fit was right
/// and the words were a lie.
#[derive(Debug, Clone, PartialEq)]
pub enum NoFit {
    /// Nothing within reach on this field at all: no candidate runway, or the
    /// best-aligned one lies further off than a line-up could ever be. `id`
    /// and `lateral_m` name that strip when there was one, so the refusal can
    /// say WHICH runway it looked for and how far away it is — the vague "I
    /// couldn't get us onto the runway" is what sent the pilot to the logs
    /// (LFPB 2026-09-22). Empty id = no pavement near us at all.
    NoneNear { id: String, lateral_m: f64 },
    /// On the pavement, aligned, but not enough runway ahead of the nose —
    /// and there WOULD be from the threshold. Backtracking fixes this one.
    TooLittleLeft {
        id: String,
        remaining_m: f64,
        needed_m: f64,
    },
    /// The whole runway is shorter than this aircraft needs. Position is
    /// irrelevant: no line-up and no backtrack can help.
    ///
    /// Edinburgh 24 is 2,549 m; the A330 asks 2,560 (a 6,000 ft roll times
    /// the 1.4 factor). Eleven metres. The refusal said "we're not on the
    /// runway yet", so the pilot lined up, was told the same thing, backtracked
    /// the full length, and was told it a third time (2026-09-20).
    RunwayTooShort {
        id: String,
        length_m: f64,
        needed_m: f64,
    },
    /// On (or near) a strip, but lying across it.
    NotAligned { id: String, err_deg: f64 },
    /// Aligned with a runway but sitting beside it.
    Offset { id: String, lateral_m: f64 },
    /// Aligned and on the centreline, but still short of the threshold.
    ShortOfThreshold { id: String, m: f64 },
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
                crate::circuit::spell_runway(id),
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
                crate::circuit::spell_runway(id),
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
                crate::circuit::spell_runway(id),
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
                crate::circuit::spell_runway(id),
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

/// [`why_no_fit`] told about the CLEARED runway: nowhere near any pavement,
/// the refusal names the end the tower cleared and how far it is, never the
/// strip that merely lay best along the nose. At ETND (2026-09-24) the SF50
/// was cleared 26R, faced 016 at the hold, and heard "08L is 152 metres
/// away": the reciprocal end of the runway it was cleared for, which reads
/// as a crew that does not know its clearance. Every other gate stands as
/// diagnosed (an aircraft aligned on another end is judged on that end).
pub fn why_no_fit_cleared(
    cands: &[Runway],
    lat: f64,
    lon: f64,
    psi_true: f64,
    needed_m: f64,
    cleared: Option<&str>,
) -> NoFit {
    let why = why_no_fit(cands, lat, lon, psi_true, needed_m);
    let NoFit::NoneNear { .. } = why else {
        return why;
    };
    match cleared.and_then(|c| cands.iter().find(|r| r.id.eq_ignore_ascii_case(c))) {
        Some(r) => NoFit::NoneNear {
            id: r.id.clone(),
            lateral_m: (cross_track_nm(lat, lon, r) * M_PER_NM).abs(),
        },
        None => why,
    }
}

/// Diagnose a failed [`runway_fit`]: the best-aligned candidate, and the FIRST
/// gate it fails — in the same order the fit applies them, so the answer is
/// always the thing that actually stopped it.
pub fn why_no_fit(cands: &[Runway], lat: f64, lon: f64, psi_true: f64, needed_m: f64) -> NoFit {
    // Of the ends we are aligned with, the one we are ON (least lateral):
    // parallels share a heading to thousandths of a degree, so the best-
    // aligned pick was a coin toss between them. EGKK 2026-09-27 (sortie
    // 1790527340): on 26L's centreline at 256.3 true, 26R (257.624) beat
    // 26L (257.629) by 0.005 degrees, 200 m away, and the diagnosis was
    // "none near" instead of "too little left". With nothing aligned, the
    // best-aligned end, as before.
    let lateral = |r: &Runway| (cross_track_nm(lat, lon, r) * M_PER_NM).abs();
    let aligned = cands
        .iter()
        .filter(|r| crate::angles::abs_diff_deg(psi_true, r.course_deg) <= ALIGN_MAX_DEG)
        .min_by(|a, b| lateral(a).total_cmp(&lateral(b)));
    let best = aligned.or_else(|| {
        cands.iter().min_by(|a, b| {
            crate::angles::abs_diff_deg(psi_true, a.course_deg).total_cmp(&crate::angles::abs_diff_deg(psi_true, b.course_deg))
        })
    });
    let Some(r) =
        best.filter(|r| (cross_track_nm(lat, lon, r) * M_PER_NM).abs() < 2.0 * LATERAL_MAX_M)
    else {
        return NoFit::NoneNear {
            id: best.map(|r| r.id.clone()).unwrap_or_default(),
            lateral_m: best
                .map(|r| (cross_track_nm(lat, lon, r) * M_PER_NM).abs())
                .unwrap_or(0.0),
        };
    };
    let id = r.id.clone();
    let err = crate::angles::abs_diff_deg(psi_true, r.course_deg);
    if err > ALIGN_MAX_DEG {
        return NoFit::NotAligned { id, err_deg: err };
    }
    let lateral_m = (cross_track_nm(lat, lon, r) * M_PER_NM).abs();
    if lateral_m > LATERAL_MAX_M {
        return NoFit::Offset { id, lateral_m };
    }
    let along_m = along_track_nm(lat, lon, r) * M_PER_NM;
    if along_m > BEFORE_THRESHOLD_MAX_M {
        return NoFit::ShortOfThreshold { id, m: along_m };
    }
    let need = needed_m.max(MIN_REMAINING_M);
    // Would the FULL strip do? If not, where we stand is beside the point.
    if r.length_m < need {
        return NoFit::RunwayTooShort {
            id,
            length_m: r.length_m,
            needed_m: need,
        };
    }
    NoFit::TooLittleLeft {
        id,
        remaining_m: r.length_m - (-along_m).max(0.0),
        needed_m: need,
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


/// The transponder to ALT (mode C) through the shared writer, so an add-on's
/// own box (the Twin Otter's KT73) has its KNOB turned; on its own task, since
/// a stepped knob takes a few hundred milliseconds. The result is kept and a
/// failure said: the A330 flew out in standby to two "recycle transponder"
/// calls with nothing said on the flight deck (EGPH, 2026-09-20). It is the
/// ONE departure check (`radio::departure_check`): the tower's assigned code
/// is dialled and read back too, so the spoken takeoff and the circuit leave
/// with the squawk full auto would have set.
fn squawk_altitude(base_url: String, v: Option<Voice>, audit: bool) {
    tokio::spawn(async move {
        let xp = XPlaneClient::new(base_url);
        let check =
            crate::tools::radio::departure_check(&xp, crate::party_line::assigned_squawk()).await;
        let mut rec = check.json();
        rec["event"] = serde_json::json!("transponder_alt");
        rec["ok"] = serde_json::json!(check.ok());
        crate::blackbox::record("autotakeoff", rec);
        if let Some(line) = check.failure_line() {
            alert(&v, Priority::Caution, &line);
        }
        // The roll's first call: brakes off and the transponder done, the
        // list audits the rest (`takeoff_config::before_takeoff_audit`).
        if audit {
            crate::takeoff_config::before_takeoff_audit(v);
        }
    });
}

/// Differential brake on the takeoff roll, for an airframe that steers on its
/// brakes ([`crate::aircraft::TaxiProfile::brake_steer`]): the taxi's own
/// split of the steer command, faded out between [`ROLL_BRAKE_FULL_KTS`] and
/// [`ROLL_BRAKE_OFF_KTS`] as the rudder comes alive. EDLP 24 (2026-09-24): the
/// SF50's castoring nosewheel and a rudder with no airflow gave the roll no
/// steering at all; the heading error grew from 8 to 27 degrees with the
/// channel at full deflection until the pilot took the pedals at 21 kt.
pub fn roll_brake_split(steer: f64, gs_kts: f64) -> (f64, f64) {
    let fade = roll_brake_fade(gs_kts);
    let (l, r) = crate::autotaxi::brake_steer_split(steer, gs_kts);
    (l * fade, r * fade)
}

/// The share of the roll's brake steering left at `gs_kts`: whole below
/// [`ROLL_BRAKE_FULL_KTS`], none above [`ROLL_BRAKE_OFF_KTS`] (a brake on
/// the roll costs the acceleration; the rudder steers there). Pure.
pub fn roll_brake_fade(gs_kts: f64) -> f64 {
    ((ROLL_BRAKE_OFF_KTS - gs_kts) / (ROLL_BRAKE_OFF_KTS - ROLL_BRAKE_FULL_KTS)).clamp(0.0, 1.0)
}

/// The roll's brakes on a FREE castoring nose (`aircraft::nose_free`): the
/// one ground law (`ground_steer`) on the roll's heading target, the roll's
/// rudder a known input, its wheel difference faded out with the speed
/// ([`roll_brake_fade`]) and no faster a turn than the aligning pivot's
/// while the power waits ([`RollAlign`]). It replaces the aligning PD and
/// its trim ([`roll_brake_split_aligning`], [`RollAlign::trimmed`]) and the
/// taxi split below 40 kt on these noses: the SR22 at LEMG 12 and the
/// Evolution at LILB 03 (2026-09-29/30) pirouetted +/-12 degrees and held
/// 4 to 5 degrees off on them. Pure.
pub fn free_roll_brakes(
    law: &mut crate::ground_steer::GroundSteer,
    err_deg: f64,
    yaw: crate::ground_steer::Yaw,
    rudder: f64,
    gs_kts: f64,
    dt: f64,
    holding: bool,
) -> (f64, f64) {
    let r_max = if holding {
        crate::autotaxi::ALIGN_RATE_MAX_DPS
    } else {
        crate::ground_steer::DEFAULT_MAX_YAW_DPS
    };
    law.step(crate::ground_steer::heading_demand(
        dt,
        gs_kts / MS_TO_KTS,
        yaw,
        err_deg,
        crate::ground_steer::Rudder::Given(rudder),
        FREE_ROLL_DIFF_MAX * roll_brake_fade(gs_kts),
        r_max,
    ))
    .wheels()
}
/// The most wheel difference the roll's brakes use on a free nose (the
/// pivot's own).
const FREE_ROLL_DIFF_MAX: f64 = 0.45;

/// [`roll_brake_split`], plus the start of a roll that begins crooked: below
/// [`ROLL_ALIGN_KTS`] a heading error past [`ROLL_ALIGN_ERR_DEG`], or a swing
/// past [`crate::autotaxi::ALIGN_RATE_MAX_DPS`], is the aligning pivot's
/// ([`crate::autotaxi::align_pivot_brakes`]): the roll law's steer is capped
/// tiny at walking pace (prop wash) and its brake split has no authority
/// under 2 kt. LIME 28 (2026-09-25): the Evolution started its roll 21
/// degrees off the runway, the steer sat at 0.17 with no brake at all, the
/// error grew to 30 at 6 kt and the pilot's pedals ended it.
///
/// The pivot was full brake from 15 degrees on the error alone, capped only
/// past 15 deg/s, and at the iMac's 0.3 to 0.45 s tick it pirouetted: the
/// SR22 at LEMG 12 (1790690995, 2026-09-29) from 10.8 off to -11.6 at 17.5
/// deg/s, then +/-12 degrees from 4 to 18 kt; the Evolution at LILB 03
/// (1790712974) from 24.6 off through -4.4 at 21.6 deg/s. Now the PD led on
/// `yaw_dps` (the sim's `R` where read), which catches the swing with the
/// opposite brake before it overshoots.
pub fn roll_brake_split_aligning(steer: f64, gs_kts: f64, err_deg: f64, yaw_dps: f64) -> (f64, f64) {
    let (l, r) = roll_brake_split(steer, gs_kts);
    if gs_kts >= ROLL_ALIGN_KTS
        || (err_deg.abs() <= ROLL_ALIGN_ERR_DEG
            && yaw_dps.abs() <= crate::autotaxi::ALIGN_RATE_MAX_DPS)
    {
        return (l, r);
    }
    crate::autotaxi::align_pivot_brakes(err_deg, yaw_dps)
}
/// The aligning pivot works below this groundspeed...
const ROLL_ALIGN_KTS: f64 = 15.0;
/// ...on a heading error past this: the aligned band itself, since the
/// power waits until the nose is inside it ([`RollAlign`]); at 3 a nose
/// held 2 to 3 off had no brake and no power to come round.
const ROLL_ALIGN_ERR_DEG: f64 = crate::autotaxi::ALIGNED_DEG;

/// The roll's power waits for the nose on a brake-steered airframe
/// ([`RollAlign`]): a roll that STARTS at this groundspeed or more is the
/// roll law's from its first tick...
const ROLL_ALIGN_HOLD_KTS: f64 = 6.0;
/// ...a held one creeps at this, the line-up's own creep...
const ROLL_ALIGN_CREEP_KTS: f64 = crate::autotaxi::ALIGN_CREEP_KTS;
/// ...stopped on both brakes when this far over it, until back under it...
const ROLL_ALIGN_OVER_KTS: f64 = crate::autotaxi::ALIGN_CREEP_OVER_KTS;
/// ...at this much brake past the creep, and this more per knot over it...
const ROLL_ALIGN_BRAKE_MIN: f64 = 0.15;
const ROLL_ALIGN_BRAKE_PER_KT: f64 = 0.1;
/// ...to at most this...
const ROLL_ALIGN_BRAKE_MAX: f64 = 0.45;
/// ...and held past this groundspeed all the same, it is a reject on the
/// ground, never a roll.
const ROLL_ALIGN_RUNAWAY_KTS: f64 = 10.0;
/// ...while the nose is more than [`crate::autotaxi::ALIGNED_DEG`] off, or
/// swinging faster than this, the lever stays at [`ROLL_PIVOT_THR`] and the
/// ramp and the stagnation clock wait...
const ROLL_ALIGNED_DPS: f64 = 3.0;
/// ...the pivot's power (the ramp's own first step)...
const ROLL_PIVOT_THR: f64 = 0.3;
/// ...and for no longer than this: a nose that will not come round is a
/// reject at walking pace, said.
pub const ROLL_ALIGN_MAX_S: f64 = 20.0;

/// Whether the roll's power may come up: a castoring nose that starts its
/// roll crooked is pivoted straight at the pivot's power FIRST, then rolled.
/// The time ramp ran the lever from 0.3 to 0.5 while the nose was still
/// coming round, and the power drove the pirouette (LEMG 12 SR22 and LILB 03
/// Evolution, 2026-09-29: 10.8 and 24.6 degrees off at the first tick,
/// swinging +/-12 and through 29 degrees at 0.41 to 0.59 of lever). Once
/// straight (or rolling past [`ROLL_ALIGN_HOLD_KTS`] at its first tick) it
/// never holds again: a gust-yaw at speed is the roll law's.
///
/// A hold HOLDS the aircraft ([`RollAlign::creep`]): a creep over its speed
/// is stopped on both brakes with the lever at idle, and speed never lets
/// the power up. LILB 03 Evolution (1790732094, 2026-09-30): held at 0.3 of
/// lever, the PT6 at 300 to 330 ft-lb ran the aircraft from 0.2 to 6.6 kt
/// in 5 s while the pivot took the nose from 16.6 to 4.4 degrees off its
/// target; the 6 kt gate then let the power up (`roll_aligned` held_s
/// 5.06, err 4.4), the roll diverged to 7.7 degrees and rejected at 68 kt,
/// 9.4 m off. Pure.
#[derive(Debug, Default, Clone)]
pub struct RollAlign {
    held_s: f64,
    done: bool,
    /// Over the creep: stopped until back under it.
    over: bool,
    /// The standing-yaw trim, signed brake (+ right), the line-up's own
    /// ([`crate::autotaxi::LineUpAlign`]).
    trim: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RollAlignState {
    /// Power at the pivot's, the ramp waiting.
    Hold,
    /// Straight (or rolling): the roll's own power.
    Go,
    /// [`ROLL_ALIGN_MAX_S`] held and still crooked.
    Failed,
}

impl RollAlign {
    pub fn step(&mut self, err_deg: f64, yaw_dps: f64, gs_kts: f64, dt: f64) -> RollAlignState {
        if self.done {
            return RollAlignState::Go;
        }
        let rolling_start = self.held_s == 0.0 && gs_kts >= ROLL_ALIGN_HOLD_KTS;
        if rolling_start
            || (err_deg.abs() <= crate::autotaxi::ALIGNED_DEG && yaw_dps.abs() <= ROLL_ALIGNED_DPS)
        {
            self.done = true;
            return RollAlignState::Go;
        }
        self.held_s += dt.max(0.0);
        if self.held_s > ROLL_ALIGN_MAX_S || gs_kts > ROLL_ALIGN_RUNAWAY_KTS {
            return RollAlignState::Failed;
        }
        RollAlignState::Hold
    }

    /// While it holds: the most lever the creep allows and the brake on
    /// BOTH wheels (added to the pivot's split). Under the creep the pivot's
    /// power and no brake; [`ROLL_ALIGN_OVER_KTS`] over it the lever at idle
    /// and the brakes on in proportion, until back under the creep. Pure.
    pub fn creep(&mut self, gs_kts: f64) -> (f64, f64) {
        if gs_kts > ROLL_ALIGN_CREEP_KTS + ROLL_ALIGN_OVER_KTS {
            self.over = true;
        } else if gs_kts <= ROLL_ALIGN_CREEP_KTS {
            self.over = false;
        }
        if !self.over {
            return (ROLL_PIVOT_THR, 0.0);
        }
        let brake = (ROLL_ALIGN_BRAKE_MIN
            + (gs_kts - ROLL_ALIGN_CREEP_KTS).max(0.0) * ROLL_ALIGN_BRAKE_PER_KT)
            .min(ROLL_ALIGN_BRAKE_MAX);
        (0.0, brake)
    }

    /// The pivot's split while it holds, with the standing yaw trimmed out
    /// as the line-up's alignment does: a creep held 4 to 5 degrees off by
    /// the propeller's pull against the pivot's 0.03 of brake (1790732094,
    /// 2026-09-30, the err 6.1 to 4.4 with the yaw dying 1.5 to 0.4 deg/s)
    /// is otherwise a creep that never straightens. Pure.
    pub fn trimmed(
        &mut self,
        (l, r): (f64, f64),
        err_deg: f64,
        yaw_dps: f64,
        gs_kts: f64,
        dt: f64,
    ) -> (f64, f64) {
        self.trim = crate::autotaxi::align_trim_step(self.trim, err_deg, yaw_dps, gs_kts, dt);
        crate::autotaxi::with_align_trim((l, r), self.trim, yaw_dps)
    }

    /// Seconds the power has waited for the nose.
    pub fn held_s(&self) -> f64 {
        self.held_s
    }

    /// Whether it has let the power come up.
    #[cfg(test)]
    pub fn done(&self) -> bool {
        self.done
    }
}

/// The lever while [`RollAlign`] holds: never over the pivot's.
#[cfg(test)]
pub fn aligning_throttle(want: f64, holding: bool) -> f64 {
    aligning_throttle_capped(want, holding, ROLL_PIVOT_THR)
}

/// [`aligning_throttle`] under the creep's own cap ([`RollAlign::creep`]).
pub fn aligning_throttle_capped(want: f64, holding: bool, cap: f64) -> f64 {
    if holding {
        want.min(ROLL_PIVOT_THR).min(cap)
    } else {
        want
    }
}

/// The pivot's split with the creep's brake on both wheels. Pure.
pub fn with_hold_brake((l, r): (f64, f64), both: f64) -> (f64, f64) {
    ((l + both).min(1.0), (r + both).min(1.0))
}

/// Full brake-steering authority on the roll below this groundspeed...
const ROLL_BRAKE_FULL_KTS: f64 = 20.0;
/// ...and none above this, where the rudder steers.
const ROLL_BRAKE_OFF_KTS: f64 = 40.0;

/// Whether the live airframe steers on its brakes.
fn brake_steers() -> bool {
    crate::aircraft::nose_castors()
        || crate::aircraft::live_profile()
            .and_then(|p| p.taxi.as_ref().map(|t| t.brake_steer))
            .unwrap_or(false)
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
    /// The standing pull the trim state demands — the INTEGRAL part, kept
    /// separate from `out` so the proportional path can move without
    /// disturbing it.
    ///
    /// This law was a PURE integrator and rang from liftoff to the 300 ft AP
    /// handover, where it stopped dead because the autopilot flies it from
    /// there: 5.6° to 13.2° about an 8° target, hard against `PITCH_MIN` on
    /// every down-swing (SF50 at EGHI, 2026-07-30). Elevator to pitch rate to
    /// attitude is already about a double integration; a third in the
    /// controller cannot be stabilised by one lead term at any gain.
    ///
    /// Going P+I fixed the structure and did NOT stop the ringing, which is
    /// the part worth remembering: across 49 takeoffs in the blackbox the
    /// same-airframe pair either side of that change (SF50, EGHI 20, one
    /// evening) went 1.26 -> 1.89 direction reversals per second and 20% ->
    /// 48% of ticks pinned against `PITCH_MIN`. The peak-to-peak halved and
    /// the oscillation got FASTER. What was actually driving it was the
    /// damping term's rate estimate and its 1 s lead — see
    /// [`RotatePitch::step`] and [`PITCH_LEAD_S`] (2026-08-02).
    trim: f64,
    /// The commanded attitude, ramped at [`ROTATE_RATE_DEG_S`] from wherever
    /// the nose was on the first tick. `None` until then.
    target: Option<f64>,
    /// Highest attitude this airframe may be commanded to, degrees — its
    /// tail-strike angle less [`TAIL_STRIKE_MARGIN_DEG`]. `None` on airframes
    /// with no geometry limit worth naming (the GA singles: a 172 runs out of
    /// elevator long before it runs out of tail clearance).
    ceiling: Option<f64>,
    /// This airframe's nose-down limit — see [`PITCH_MIN`].
    pitch_min: f64,
    /// The speed hold by pitch after liftoff, on an airframe that declares
    /// one (see [`ClimbHold`]); `None` holds the rotation attitude, as ever.
    hold: Option<ClimbHold>,
    /// The last airspeed read and its low-passed trend, knots per second.
    prev_ias: Option<f64>,
    accel_kts_s: f64,
    /// The column the climb hold's ceiling guard allows, while it holds one
    /// ([`RotatePitch::ceiling_guard`]); `None` when the nose is inside its
    /// approach to `pitch_max_deg`.
    ceiling_limit: Option<f64>,
    /// Tests only: the law as it flew EGKK 08R, without the guard.
    unguarded: bool,
    /// The rotation's rate, degrees a second ([`ROTATE_RATE_DEG_S`] unless
    /// the airframe declares its own, `rotate_rate_deg_s`).
    rotate_rate: Option<f64>,
    /// The airframe's fitted rotation elevator (`rotate_elevator:`), fed
    /// forward by [`RotatePitch::model_pull`]; `None` is the P+I alone.
    model: Option<crate::aircraft::RotateElevatorProfile>,
    /// On the wheels, as the caller last said ([`RotatePitch::on_wheels`]).
    wheels: bool,
    /// The attitude the rotation started from (the roll attitude).
    rest: Option<f64>,
    /// The model's column, as last fed forward.
    model_out: f64,
    /// The vertical speed, fpm, as last read (`vh_ind_fpm`;
    /// [`RotatePitch::climb_fpm`]): with the airspeed, the flight path.
    vs_fpm: Option<f64>,
    /// Tests only: see [`RotatePitch::as_flown_b2b1481`].
    #[cfg(test)]
    as_flown_b2b1481: Option<f64>,
}

/// The share of the fitted rotation model fed forward; the P+I finds the
/// rest. Under one on purpose: a column short of the need is closed from
/// below by the loop, one past it is an overshoot at liftoff. Against the
/// SR22 plant fitted to LEMG (#184) the whole model lifted off 82 kt and
/// overshot to 10.7 degrees; 0.8 lifted off 83 and held under 9.6.
pub const ROTATE_MODEL_SHARE: f64 = 0.8;
/// The flight path the model believes, degrees: a vertical speed read in a
/// bounce or a zoom is not a path to take the spring off by.
const ROTATE_MODEL_PATH_MAX_DEG: f64 = 12.0;
/// The slowest airspeed the model's `(ref / ias)^2` is taken at, knots: a
/// rotation is never below it, and a dead ASI must not ask for full column.
const ROTATE_MODEL_MIN_KTS: f64 = 40.0;

/// The hand-flown initial climb's speed hold: from liftoff to the autopilot's
/// engage the rotation's attitude target is walked by the airspeed TREND, so
/// the aircraft climbs at `hold_kts` instead of accelerating away at a fixed
/// 8 degrees. The rotation attitude is its floor (a slow aircraft flies the old
/// law) and `pitch_max_deg` its ceiling, reached only as the tail clears the
/// runway ([`tail_clear_ceiling_deg`]).
///
/// LEMG 13, the 737, 2026-09-28 (five full-auto circuits, every one the same):
/// rotate 145, liftoff 162, then 185 kt by the 300 ft engage at 11 degrees,
/// the law pushing -0.37 to hold 8. EGPH 2026-09-21 the same, 157 to 185. A
/// crew rotates to about 15 degrees and holds V2 + 15 by pitch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClimbHold {
    pub hold_kts: f64,
    pub pitch_max_deg: f64,
}

impl ClimbHold {
    /// From the airframe's declaration and its initial climb speed (V2 + 20
    /// on the jets): hold [`CLIMB_HOLD_UNDER_KTS`] under it, the middle of the
    /// V2 to V2 + 20 band a crew flies. `None` without a sane ceiling above
    /// the rotation attitude.
    pub fn new(initial_climb_kts: f64, pitch_max_deg: Option<f64>) -> Option<Self> {
        let pitch_max_deg = pitch_max_deg.filter(|d| d.is_finite() && *d > ROTATE_PITCH_DEG)?;
        (initial_climb_kts.is_finite() && initial_climb_kts > CLIMB_HOLD_UNDER_KTS).then(|| {
            ClimbHold {
                hold_kts: initial_climb_kts - CLIMB_HOLD_UNDER_KTS,
                pitch_max_deg,
            }
        })
    }
}

/// The hold sits this far under the initial climb speed (V2 + 20): V2 + 15.
pub const CLIMB_HOLD_UNDER_KTS: f64 = 5.0;
/// The tail's arm behind the main wheels, feet, for the clearance a height
/// buys: a long one on purpose (the A330-300's is about 100, the 737-800's
/// about 60), so every airframe's ceiling grows no faster than its tail
/// clears. The sim's `y_agl` reads 0.1 ft on the 737's wheels (LEMG
/// 2026-09-28), so the height IS the wheels'.
pub const TAIL_ARM_FT: f64 = 100.0;

/// The attitude the tail allows at `agl_ft` wheel height: the rotation's own
/// ceiling on the runway, plus the angle the height buys over the tail's arm
/// (5.7 degrees at 10 ft, 11.3 at 20, 19.3 at 35). A crew keeps rotating
/// through liftoff; the tail strike is an attitude ON the runway. Pure.
pub fn tail_clear_ceiling_deg(on_runway_deg: f64, agl_ft: f64) -> f64 {
    on_runway_deg + (agl_ft.max(0.0) / TAIL_ARM_FT).atan().to_degrees()
}
/// The airspeed trend's low-pass, seconds. A two-sample difference over a
/// 0.2 s tick turns a tenth of a knot of read noise into half a knot a second.
const CLIMB_ACCEL_TAU_S: f64 = 1.0;
/// The acceleration the hold asks for: the speed error closed over this
/// time constant, capped both ways.
const CLIMB_SPEED_TAU_S: f64 = 3.0;
const CLIMB_ACCEL_MAX_KTS_S: f64 = 1.5;
/// How fast the attitude target walks per knot-per-second of acceleration
/// it should not have, degrees per second. At takeoff thrust a degree of
/// flight path is about a third of a knot a second, so this closes the
/// attitude on the speed in about three seconds.
const CLIMB_DEG_S_PER_KT_S: f64 = 1.5;
/// The target walks down no faster than this, degrees per second (up is the
/// rotation's own rate).
const CLIMB_PITCH_DOWN_DEG_S: f64 = 1.5;

impl RotatePitch {
    /// `tail_strike_deg` is the airframe's tail-clearance attitude, from the
    /// profile; `None` leaves the law with only its own 8° target.
    ///
    /// There is deliberately no constructor that leaves the authority
    /// implicit. One existed and every caller but the tests moved off it
    /// within the hour — an argument nobody passes is an argument nobody
    /// thinks about, and this one is the difference between a ceiling and a
    /// suggestion. Pass `PITCH_MIN` to mean the GA figure and say so.
    ///
    /// A ceiling the law
    /// cannot push against is not a ceiling: the 737 declares an 11° tail
    /// strike, was commanded 8°, and flew 22.4° because the push it wanted
    /// was four times what it was allowed to ask for (2026-09-20).
    pub fn with_authority(tail_strike_deg: Option<f64>, pitch_min: f64) -> Self {
        RotatePitch {
            out: 0.0,
            prev_theta: None,
            trim: 0.0,
            target: None,
            ceiling: tail_strike_deg.map(|d| d - TAIL_STRIKE_MARGIN_DEG),
            // Never POSITIVE — a "minimum" above zero would be a standing
            // pull the law could not release — and never past full travel.
            pitch_min: pitch_min.clamp(-1.0, 0.0),
            hold: None,
            prev_ias: None,
            accel_kts_s: 0.0,
            ceiling_limit: None,
            unguarded: false,
            rotate_rate: None,
            model: None,
            wheels: true,
            rest: None,
            model_out: 0.0,
            vs_fpm: None,
            #[cfg(test)]
            as_flown_b2b1481: None,
        }
    }

    /// The airframe's fitted rotation elevator (`rotate_elevator:`); `None`
    /// (or an insane declaration) keeps the P+I alone, unchanged.
    pub fn with_rotate_model(
        mut self,
        model: Option<crate::aircraft::RotateElevatorProfile>,
    ) -> Self {
        self.model = model.and_then(crate::aircraft::RotateElevatorProfile::valid);
        self
    }

    /// Whether the wheels are on the runway this tick (`onground_any`).
    /// Read only by a declared rotation model.
    pub fn on_wheels(&mut self, on: bool) {
        self.wheels = on;
    }

    /// The flight model's vertical speed this tick (`vh_ind_fpm`), for a
    /// declared rotation model's flight path; `None` (a failed read) keeps
    /// the last.
    pub fn climb_fpm(&mut self, vs_fpm: Option<f64>) {
        if let Some(v) = vs_fpm.filter(|v| v.is_finite()) {
            self.vs_fpm = Some(v);
        }
    }

    /// The model's column right now, for the blackbox (`roll` `pitch_ff`).
    pub fn model_pull(&self) -> f64 {
        self.model_out
    }

    /// The fitted rotation column, fed forward, one law on the wheels and
    /// off them: the column that holds the TARGET attitude's angle of attack
    /// against the tail's spring (the attitude less the flight path; on the
    /// wheels the path is level), plus the breakout, plus the column that
    /// turns the nose at the ramp's rate. The breakout walks to `air_base`
    /// through the ground effect's height. A share of it
    /// ([`ROTATE_MODEL_SHARE`]).
    ///
    /// 1790699819 (LEMG 13, 2026-09-29, #184): the first model took the
    /// spring and the breakout away AT wheels-off, 0.40 to 0.18 of feed at
    /// 81.4 KIAS, as if a lifting wheel were a climbing aircraft. The nose
    /// fell from 5.0 to 3.0 degrees, the wing floated in ground effect, and
    /// the 8 ft liftoff came at 94.7 KIAS. A path is flown before the spring
    /// lets go, and ground effect holds the nose down until it is climbed out
    /// of.
    fn model_step(
        &mut self,
        air: Option<(f64, f64)>,
        target: f64,
        want_rate: f64,
        dt: f64,
    ) -> f64 {
        let _ = dt; // read only by the b2b1481 replay (tests)
        let Some(m) = self.model else {
            return 0.0;
        };
        let Some((ias, agl_ft)) = air.filter(|(v, h)| v.is_finite() && h.is_finite()) else {
            // No airspeed: the last column stands (a failed read is no
            // measurement).
            return self.model_out;
        };
        let q = (ias.max(ROTATE_MODEL_MIN_KTS) / m.ref_kts).powi(2);
        let rest = self.rest.unwrap_or(target);
        let (path_deg, lifted) = if self.wheels {
            (0.0, 0.0)
        } else {
            let tas_fpm = ias.max(ROTATE_MODEL_MIN_KTS) * 101.269;
            let path = self
                .vs_fpm
                .map_or(0.0, |vs| vs.atan2(tas_fpm).to_degrees())
                .clamp(-ROTATE_MODEL_PATH_MAX_DEG, ROTATE_MODEL_PATH_MAX_DEG);
            (path, (agl_ft / m.ground_effect_ft).clamp(0.0, 1.0))
        };
        let base = m.breakout + (m.air_base - m.breakout) * lifted;
        #[cfg(test)]
        if let Some(hold) = self.as_flown_b2b1481.filter(|_| !self.wheels) {
            let goal = ROTATE_MODEL_SHARE * hold / q;
            self.model_out = goal + (self.model_out - goal) * (-dt / 0.2).exp();
            return self.model_out;
        }
        self.model_out = ROTATE_MODEL_SHARE
            * (m.per_deg * (target - path_deg - rest).max(0.0)
                + (base + want_rate.max(0.0) / m.deg_s_per_unit) / q);
        self.model_out
    }

    /// Tests only: the model's flying half as b2b1481 flew 1790699819, a flat
    /// 0.164 hold reached over 0.2 s at wheels-off.
    #[cfg(test)]
    fn as_flown_b2b1481(mut self) -> Self {
        self.as_flown_b2b1481 = Some(0.164);
        self
    }

    /// The law without its ceiling guard: what flew EGKK 08R (#78), for the
    /// replay that pins the guard.
    #[cfg(test)]
    fn without_ceiling_guard(mut self) -> Self {
        self.unguarded = true;
        self
    }

    /// The column limit the ceiling guard holds right now, for the blackbox
    /// (`climb_manual` `pitch_lim`); `None` while it holds none.
    pub fn ceiling_limit(&self) -> Option<f64> {
        self.ceiling_limit
    }

    /// Hold the initial climb speed by pitch after liftoff (see
    /// [`ClimbHold`]). `None` is the fixed rotation attitude.
    pub fn with_climb_hold(mut self, hold: Option<ClimbHold>) -> Self {
        self.hold = hold;
        self
    }

    /// The airframe's own rotation rate (`rotate_rate_deg_s`), bounded to
    /// 1.5 to 6 degrees a second; `None` keeps [`ROTATE_RATE_DEG_S`].
    pub fn with_rotate_rate(mut self, deg_s: Option<f64>) -> Self {
        if let Some(r) = deg_s.filter(|r| r.is_finite()) {
            self.rotate_rate = Some(r.clamp(1.5, 6.0));
        }
        self
    }

    fn rotate_rate(&self) -> f64 {
        self.rotate_rate.unwrap_or(ROTATE_RATE_DEG_S)
    }

    /// The low-passed airspeed trend, knots per second, for the blackbox.
    pub fn accel_kts_s(&self) -> f64 {
        self.accel_kts_s
    }

    /// The attitude being commanded right now — the ramp, for the blackbox.
    pub fn target_deg(&self) -> f64 {
        self.target.unwrap_or(0.0)
    }

    /// `rate_deg_s` is the sim's MEASURED pitch rate (`sim/flightmodel/
    /// position/Q`). `None` falls back to differencing the attitude, which is
    /// what this law did for its whole life and is the reason it rang.
    ///
    /// A two-sample difference over the loop's 0.25 s tick is a bad estimator
    /// twice over: it amplifies every wobble in the attitude read by 1/dt (a
    /// 1° tick-to-tick movement becomes 4°/s, and at the old 1 s lead that is
    /// 4° of phantom error against a real error of 1-2°), and it carries an
    /// inherent half-sample delay — so the DAMPING term arrives about 50° out
    /// of phase at the ~0.9 s period the aircraft actually rang at, which
    /// makes it drive the motion rather than oppose it. Q has neither problem:
    /// it is measured, not differentiated, so it needs no filter (and a filter
    /// was tried and measurably made things worse, EGHI 2026-07-30 — it put
    /// lag back exactly where the phase margin is thinnest).
    ///
    /// Q is the BODY-axis rate where the law wants the Euler pitch rate; they
    /// differ by cos(bank), and this climb holds the wings inside
    /// `CLIMB_BANK_MAX_DEG`, so the error is under 1%.
    #[cfg(test)]
    pub fn step(&mut self, theta_deg: f64, rate_deg_s: Option<f64>, dt_s: f64) -> f64 {
        self.step_climb(theta_deg, rate_deg_s, dt_s, None)
    }

    /// [`RotatePitch::step`] with the airspeed and height, for the climb
    /// hold: `air` is `(ias_kts, agl_ft)`. Without a declared hold, or with
    /// `air` `None`, exactly the fixed-attitude law.
    pub fn step_climb(
        &mut self,
        theta_deg: f64,
        rate_deg_s: Option<f64>,
        dt_s: f64,
        air: Option<(f64, f64)>,
    ) -> f64 {
        let dt = dt_s.clamp(0.05, 1.0);
        let rate = rate_deg_s
            .unwrap_or_else(|| self.prev_theta.map_or(0.0, |p| (theta_deg - p) / dt))
            .clamp(-30.0, 30.0);
        self.prev_theta = Some(theta_deg);
        // The commanded attitude walks up from where the nose actually was,
        // never further than the airframe's geometry allows.
        let cap = self
            .ceiling
            .map_or(ROTATE_PITCH_DEG, |c| ROTATE_PITCH_DEG.min(c));
        let prev_target = self.target;
        if prev_target.is_none() {
            self.rest = Some(theta_deg);
        }
        let ramped = match prev_target {
            None => theta_deg.min(cap),
            Some(t) if t < cap => (t + self.rotate_rate() * dt).min(cap),
            // Past the ramp: the hold's target stands (the ramp never pulls
            // it back to the rotation attitude).
            Some(t) if self.hold.is_some() => t,
            Some(t) => (t + self.rotate_rate() * dt).min(cap),
        };
        // The airspeed trend, whenever the caller has the airspeed.
        if let Some((ias, _)) = air.filter(|(v, _)| v.is_finite()) {
            let raw = self.prev_ias.map_or(0.0, |p| ((ias - p) / dt).clamp(-10.0, 10.0));
            if self.prev_ias.is_some() {
                self.accel_kts_s += (raw - self.accel_kts_s) * dt / (CLIMB_ACCEL_TAU_S + dt);
            }
            self.prev_ias = Some(ias);
        }
        let held = match (self.hold, air) {
            (Some(h), Some((ias, agl_ft))) if ramped >= cap - 1e-9 && ias.is_finite() => {
                // The climb attitude, as far as the tail has cleared.
                let tail = self
                    .ceiling
                    .map_or(f64::INFINITY, |c| tail_clear_ceiling_deg(c, agl_ft));
                // The target stops CEILING_MARGIN_DEG under the ceiling, so
                // a settled attitude is under it too (#78); the guard keeps
                // the nose from passing the ceiling on the way there.
                let margin = if self.unguarded {
                    0.0
                } else {
                    CEILING_MARGIN_DEG
                };
                let top = (h.pitch_max_deg - margin).min(tail).max(cap);
                let want_accel = ((h.hold_kts - ias) / CLIMB_SPEED_TAU_S)
                    .clamp(-CLIMB_ACCEL_MAX_KTS_S, CLIMB_ACCEL_MAX_KTS_S);
                let walk = (CLIMB_DEG_S_PER_KT_S * (self.accel_kts_s - want_accel) * dt)
                    .clamp(-CLIMB_PITCH_DOWN_DEG_S * dt, ROTATE_RATE_DEG_S * dt);
                Some((ramped + walk).clamp(cap, top))
            }
            _ => None,
        };
        let target = held.unwrap_or(ramped);
        self.target = Some(target);
        // Damp the rate ERROR: while the ramp is still climbing, the nose is
        // SUPPOSED to be moving at ROTATE_RATE_DEG_S, and subtracting that
        // motion as if it were an overshoot is what stalls a rotation. The
        // hold's walk is a commanded rate too.
        let want_rate = match (held, prev_target) {
            (Some(t), Some(p)) => (t - p) / dt,
            _ if target < cap => self.rotate_rate(),
            _ => 0.0,
        };
        let err = target - theta_deg - PITCH_LEAD_S * (rate - want_rate);
        // PROPORTIONAL + integral, not integral alone.
        //
        // Elevator -> pitch rate -> attitude is already about a double
        // integration. A pure integrator on top makes three, and one lead term
        // cannot supply the phase to stabilise that at ANY gain — it was never
        // going to settle, only ring wider or narrower. It rang 5.6-13.2 deg
        // about an 8 deg target, hard against PITCH_MIN on every down-swing.
        //
        // The proportional path answers immediately and adds no lag; the
        // integrator is left to do the one job the original comment actually
        // wanted from it — hold the standing pull the trim state demands, so
        // that on target and steady the elevator does not fall away.
        self.trim = (self.trim + err * PITCH_TRIM_PER_DEG_S * dt).clamp(self.pitch_min, PITCH_MAX);
        // A declared rotation model's column, fed forward (zero without one).
        let fed = self.model_step(air, target, want_rate, dt);
        let out = (err * PITCH_P_PER_DEG + self.trim + fed).clamp(self.pitch_min, PITCH_MAX);
        self.out = match self.hold {
            Some(h) if !self.unguarded => {
                self.ceiling_guard(h.pitch_max_deg, theta_deg, rate, dt, out, err)
            }
            _ => out,
        };
        self.out
    }

    /// The climb attitude is a CEILING, not a target the law may sail
    /// through: the nose may approach `pitch_max_deg` no faster than
    /// [`CEILING_APPROACH_S`] closes the gap, and while it comes faster the
    /// column is walked forward by the excess rate (an integral, so it finds
    /// whatever push this airframe's nose-up moment needs without knowing
    /// it) plus a proportional part for the first tick. The walk lets go only
    /// once the attitude law asks for less than the limit; while it holds,
    /// the attitude law's integrator is held to the limit (it cannot wind
    /// pull the guard then has to undo).
    ///
    /// EGKK 08R, the 737, 2026-09-28 (the first ClimbHold flight): the target
    /// stopped at 18 degrees and the nose flew to 22.4. The law arrived with
    /// the nose rising 2.8 degrees a second and had 0.03 of column per degree
    /// to stop it with; four degrees over it was pushing 0.15 where the 737
    /// needs 0.14 of push just to hold its attitude (the fit in
    /// `the_egkk_climb_stays_under_its_ceiling`).
    fn ceiling_guard(
        &mut self,
        pitch_max_deg: f64,
        theta_deg: f64,
        rate: f64,
        dt: f64,
        out: f64,
        err: f64,
    ) -> f64 {
        let excess = rate - (pitch_max_deg - theta_deg) / CEILING_APPROACH_S;
        let walked = match self.ceiling_limit {
            Some(lim) => Some(lim - CEILING_WALK_GAIN * excess * dt),
            None if excess > 0.0 => Some(self.out.min(out) - CEILING_WALK_GAIN * excess * dt),
            None => None,
        };
        // Released once the attitude law itself asks for no more than the
        // limit and the nose is inside its approach.
        self.ceiling_limit = walked
            .filter(|lim| excess > 0.0 || *lim < out)
            .map(|lim| lim.clamp(self.pitch_min, PITCH_MAX));
        let Some(lim) = self.ceiling_limit else {
            return out;
        };
        let u = out
            .min(lim - CEILING_RATE_GAIN * excess.max(0.0))
            .clamp(self.pitch_min, PITCH_MAX);
        self.trim = self.trim.min(u - err * PITCH_P_PER_DEG).max(self.pitch_min);
        u
    }
}

/// The time constant the nose may close on the climb ceiling with, seconds:
/// the rate allowed is the gap over this. Fitted with the two gains below
/// against the EGKK 08R recording across its plant uncertainty
/// (`the_egkk_climb_stays_under_its_ceiling`): 2.0 grazes the ceiling on
/// the slow plants, 3.0 holds the nose a further half degree down for
/// nothing.
const CEILING_APPROACH_S: f64 = 2.5;
/// The climb hold's target stops this far under `climb_pitch_max_deg`: a P+I
/// law settles ON its target, grazing a ceiling the target sits on.
const CEILING_MARGIN_DEG: f64 = 0.5;
/// The guard's integral gain: column walked forward per second per degree a
/// second of rate over the approach. A gain on the attitude law, not an
/// airframe number: it finds the airframe's push itself, and holds the
/// ceiling from 8 to 32 degrees a second per unit of column (the fit is 12.9).
const CEILING_WALK_GAIN: f64 = 0.15;
/// The guard's proportional gain: column pushed at once per degree a second
/// over the approach.
const CEILING_RATE_GAIN: f64 = 0.04;

/// The bank the climb-out flies: toward runway heading, capped shallow —
/// this is track-keeping on departure, not a turn.
pub fn climb_bank_target(hdg_err_deg: f64) -> f64 {
    (hdg_err_deg * BANK_PER_HDG_ERR).clamp(-CLIMB_BANK_MAX_DEG, CLIMB_BANK_MAX_DEG)
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

/// The ONE throttle command per tick, ground or air. Airborne, the climb's
/// overspeed guard IS the command: the guard used to live in a second,
/// change-gated write further down the loop while this path commanded an
/// unconditional 1.0 every airborne tick, so the guard's reduced lever
/// survived exactly one tick before full power was re-driven over it (the
/// SF50's Vmo overspeed, back again), and with a deep cut the pilot-override
/// detector then read the guard's own write as a hand pulling the lever and
/// handed the takeoff away mid-climb. Two writers on one lever is the whole
/// bug; this seam is where the loop and the test agree on the number.
pub fn tick_throttle_want(
    airborne: bool,
    elapsed_s: f64,
    err_deg: f64,
    gs_kts: f64,
    ias_kts: f64,
    vne_kts: Option<f64>,
) -> f64 {
    if airborne {
        climb_throttle(ias_kts, climb_guard_vne(vne_kts))
    } else {
        throttle_cmd(elapsed_s, err_deg, gs_kts)
    }
}

/// The thrust lever after the declared thrust reduction: above its height,
/// airborne, no more than the climb lever, and whether the reduction BINDS
/// (the lever it leaves is under the takeoff's own want). Takeoff thrust held
/// to the handoff and past it was the follower's to take back (EGPH
/// 2026-09-21); a crew sets climb thrust at the reduction height. Pure.
pub fn thrust_reduced(
    want: f64,
    airborne: bool,
    agl_ft: f64,
    reduction: Option<crate::takeoff_config::ThrustReduction>,
) -> (f64, bool) {
    match reduction {
        Some(r) if airborne && agl_ft >= r.agl_ft && want > r.lever => (r.lever, true),
        _ => (want, false),
    }
}

/// A reduction comes back at this rate, lever per second: the way a crew
/// pulls to climb thrust, not a step the pitch has to answer at once.
pub const THRUST_REDUCE_PER_S: f64 = 0.1;

/// The lever while the reduction binds: `slewed` (the takeoff's own slew,
/// whose releases are instant) no further below `last` than
/// [`THRUST_REDUCE_PER_S`] a second. Every other release stays instant. Pure.
pub fn reduction_slew(slewed: f64, last: f64, dt_s: f64) -> f64 {
    slewed.max((last - THRUST_REDUCE_PER_S * dt_s.max(0.0)).min(last))
}

/// The commanded climb rate: the profile's best rate, backed off while the
/// airspeed is below the climb speed — speed first, rate second, the same
/// energy rule autoland's go-around flies. Never below a positive floor:
/// once airborne the aircraft keeps climbing away from the ground.
pub fn climb_vs_fpm(ias_kts: f64, vy_kts: f64, max_fpm: f64) -> f64 {
    // The floor must never EXCEED the request: the procedure follower passes
    // its own shrinking level-off command as the cap, and f64::clamp PANICS
    // on min > max — clamp(150, 145.9) killed the enroute follower thread
    // dead at 7,950 ft, fifty feet under its 8,000 ft level-off, and the
    // flight continued on whatever the AP dials last held (LXGB to LEMG,
    // 2026-08-02). A request already under the floor is granted as asked:
    // near level-off, "keep climbing away from the ground" is not the
    // question any more.
    let cap = max_fpm.max(0.0);
    (cap - (vy_kts - ias_kts).max(0.0) * 60.0).clamp(150.0_f64.min(cap), cap)
}

/// What the handoff owes the takeoff flaps: `None` when they are already up,
/// `Some(true)` to retract them now, `Some(false)` when the aircraft is too
/// slow to clean up and the next pair of hands must be TOLD so.
///
/// The in-climb schedule waits for Vy, and Vy is not a speed every airframe
/// sees before this loop ends. A 737 lifts at V2+20 and hands over at 1,000 ft
/// doing 181 kt against a profile Vy of 210, so its one-shot never fired: the
/// flaps stayed at 5 from Lyon to FL360 and on to Heathrow's top of descent,
/// the entire climb sitting on their 250 kt placard (2026-09-17). A takeoff
/// that ends with the aircraft still configured for takeoff has not finished.
///
/// `clean_floor_kts` is the airframe's clean-configuration minimum (1.2 x
/// clean Vs): below it, retracting is the stall the schedule exists to avoid,
/// so the flaps stay out and the copilot says why. An airframe that declares
/// no floor has nothing to judge against and cleans up, as it always did.
pub fn clean_up_at_handoff(
    retracted: bool,
    ias_kts: f64,
    clean_floor_kts: Option<f64>,
) -> Option<bool> {
    if retracted {
        return None;
    }
    Some(clean_floor_kts.is_none_or(|floor| ias_kts > floor))
}

/// How long the handoff waits, still flying the climb, for the speed that
/// lets the flaps come up. The A330 reached its handoff height at 203 kt
/// against a 210 kt clean floor, accelerating at 2.5 kt a second: the loop
/// said "too slow to clean up", left, and the aircraft passed the 215 kt
/// placard twelve seconds later with nobody on the flap lever (EGPH,
/// 2026-09-20). Three seconds more would have cleaned it up.
pub const CLEAN_UP_WAIT_S: f64 = 30.0;

/// Whether the handoff holds off, still climbing, for the clean-up: dirty,
/// under the clean floor, and inside [`CLEAN_UP_WAIT_S`].
pub fn handoff_waits_for_clean_up(
    retracted: bool,
    ias_kts: f64,
    clean_floor_kts: Option<f64>,
    waited_s: f64,
) -> bool {
    clean_up_at_handoff(retracted, ias_kts, clean_floor_kts) == Some(false)
        && waited_s < CLEAN_UP_WAIT_S
}

/// The altitude dial a PILOT handoff leaves behind. A dial the climb can
/// never capture (at or under us: the A330's sat at 100 ft, and the VS climb
/// ran through 11,000 with "check altitude capture" said three times) is set
/// 2,000 ft up, rounded up to the thousand. A dial the pilot set above us is
/// the pilot's: `None`.
pub const HANDOFF_DIAL_CLEAR_FT: f64 = 500.0;
pub fn handoff_altitude_dial_ft(dial_ft: f64, alt_ft: f64) -> Option<f64> {
    if dial_ft >= alt_ft + HANDOFF_DIAL_CLEAR_FT {
        return None;
    }
    Some(((alt_ft + 2000.0) / 1000.0).ceil() * 1000.0)
}

/// How far below Vne the overspeed protection starts working, and how much
/// climb rate it trades in per knot of excess.
const OVERSPEED_GUARD_KTS: f64 = 20.0;
const OVERSPEED_VS_PER_KT: f64 = 60.0;
const OVERSPEED_VS_MAX_FPM: f64 = 1500.0;
/// Thrust is pulled no lower than this in the climb — enough to keep climbing.
const CLIMB_THROTTLE_MIN: f64 = 0.55;
const OVERSPEED_THR_PER_KT: f64 = 0.02;

/// The speed the climb-out guards against: the airframe's Vne, or the 250 kt
/// rule below 10,000 ft plus the guard band, whichever is lower.
///
/// The guard was Vne alone. A 172's Vne is 163 and it cannot get there in a
/// climb; an A330's is 330, and at TOGA with a 1,500 fpm dial it reached 273
/// kt by the 1,000 ft handoff, then Vmo, and the aircraft's own high-speed
/// protection pitched it into a 5,400 fpm climb (EGPH, 2026-09-20). Every
/// takeoff is below 10,000 ft, so the band starts at 250 on any airframe
/// that could exceed it; airframes with a lower Vne are untouched.
pub fn climb_guard_vne(vne_kts: Option<f64>) -> Option<f64> {
    vne_kts.map(|v| v.min(crate::tools::speedhold::BELOW_10K_LIMIT_KTS + OVERSPEED_GUARD_KTS))
}

/// Climb rate with speed protection on BOTH sides.
///
/// [`climb_vs_fpm`] only ever protected the slow side: below Vy it backs the
/// rate off, above Vy it commands the maximum and nothing else. That is right
/// for a trainer, which cannot outrun itself at full power, and wrong for a
/// jet, which can. The SF50 left the runway at full thrust and accelerated
/// through the climb to 270 KIAS against a Vmo of 250 — measured live in the
/// climb, 2026-07-28 — with the copilot commanding a placid 1500 fpm the
/// whole way, because nothing in the law was watching the top end.
///
/// Inside the guard band the excess speed is traded for altitude: climb
/// HARDER, which is what a pilot does with thrust they cannot reduce. The
/// trade is capped so the protection can never become a pitch-up of its own.
pub fn climb_vs_protected(ias_kts: f64, vy_kts: f64, max_fpm: f64, vne_kts: Option<f64>) -> f64 {
    let base = climb_vs_fpm(ias_kts, vy_kts, max_fpm);
    let Some(vne) = vne_kts else { return base };
    let over = ias_kts - (vne - OVERSPEED_GUARD_KTS);
    if over <= 0.0 {
        return base;
    }
    base + (over * OVERSPEED_VS_PER_KT).min(OVERSPEED_VS_MAX_FPM)
}

/// Climb thrust with the same guard. Trading speed for rate is only half an
/// answer — on an airframe with this much excess thrust the aircraft simply
/// re-accelerates — so the lever comes back too, which is the other half of
/// what a pilot does. Never below [`CLIMB_THROTTLE_MIN`]: the point is to stop
/// overspeeding, not to stop climbing. Airframes with no declared Vne are
/// untouched and keep the full-power climb they have always flown.
pub fn climb_throttle(ias_kts: f64, vne_kts: Option<f64>) -> f64 {
    let Some(vne) = vne_kts else { return 1.0 };
    let over = ias_kts - (vne - OVERSPEED_GUARD_KTS);
    if over <= 0.0 {
        return 1.0;
    }
    (1.0 - over * OVERSPEED_THR_PER_KT).max(CLIMB_THROTTLE_MIN)
}

/// A jet's climb-out, declared by the profile (`acceleration_height_agl_ft`
/// and `best_climb_fpm`) and republished on every swap.
///
/// EGPH 24, 2026-09-21, the 737: every airframe climbed out on one capped
/// 1,500 fpm dial at takeoff thrust. That is a 172's whole climb and a fifth
/// of a 737's excess power, so the rest went into speed: 185 kt at 340 ft,
/// the flaps running up on speed alone at 400, 500 and 740 ft, 240 kt through
/// 1,000 ft. A crew holds V2 + 20 to the acceleration height by PITCH, then
/// lowers the nose and cleans up on the schedule.
/// Full auto has a departure's minimum altitudes ahead: the takeoff climbs at
/// best rate. Set by full auto for its takeoff, cleared after.
pub static DEPARTURE_MINIMUMS: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClimbOut {
    pub accel_agl_ft: f64,
    /// The hardest the speed hold may climb, fpm.
    pub max_fpm: f64,
}

impl ClimbOut {
    pub fn from_profile(p: &crate::aircraft::AircraftProfile) -> Option<Self> {
        let r = p.reference.as_ref()?;
        let accel_agl_ft = r
            .acceleration_height_agl_ft
            .filter(|h| h.is_finite() && *h > 0.0)?;
        Some(ClimbOut {
            accel_agl_ft,
            max_fpm: r.best_climb_fpm.filter(|f| f.is_finite()).unwrap_or(0.0),
        })
    }
}

static CLIMB_OUT: std::sync::Mutex<Option<ClimbOut>> = std::sync::Mutex::new(None);

/// The live airframe's manual takeoff power limit
/// ([`crate::aircraft::TakeoffPowerProfile`]), republished on every swap.
static TAKEOFF_POWER: std::sync::Mutex<Option<crate::aircraft::TakeoffPowerProfile>> =
    std::sync::Mutex::new(None);

pub fn publish_takeoff_power(p: Option<crate::aircraft::TakeoffPowerProfile>) {
    *TAKEOFF_POWER.lock().unwrap_or_else(|e| e.into_inner()) = p;
}

fn takeoff_power() -> Option<crate::aircraft::TakeoffPowerProfile> {
    TAKEOFF_POWER.lock().unwrap_or_else(|e| e.into_inner()).clone()
}

/// Newton-metres to foot-pounds (`ENGN_TRQ`, `sim::state::F::Trq0`, is in N m).
const NM_TO_FTLB: f64 = 0.737_562;
/// From this share of the limit the lever stops advancing: a turboprop
/// spooling up and gathering ram air keeps adding torque at a fixed lever.
const TORQUE_HOLD_FRAC: f64 = 0.95;
/// Past the limit the lever comes back at least this much per tick, and in
/// proportion to the excess (torque is near proportional to the lever at a
/// given speed), aiming this share under the limit.
const TORQUE_BACKOFF_STEP: f64 = 0.02;
const TORQUE_AIM_FRAC: f64 = 0.98;

/// The lever under the manual's torque limit: below [`TORQUE_HOLD_FRAC`] of
/// it the law's own want; from there to the limit the lever holds; past the
/// limit it comes back in proportion to the excess. Past `until_kias`, or
/// with no torque read or no limit, the want as asked. Pure.
///
/// It stepped back 0.02 a tick only once 5% over, and a PT6 at a held lever
/// kept climbing: LILB 03 Evolution (1790724857, 2026-09-30) ran 1,735
/// ft-lb against the manual's 1,250, 39% over, by 60 kt.
pub fn torque_limited(
    want: f64,
    last: f64,
    torque_ftlb: Option<f64>,
    ias_kts: f64,
    limit: Option<&crate::aircraft::TakeoffPowerProfile>,
) -> f64 {
    let (Some(l), Some(tq)) = (limit, torque_ftlb) else {
        return want;
    };
    if ias_kts >= l.until_kias || !tq.is_finite() {
        return want;
    }
    if tq > l.torque_ftlb {
        let proportional = last * l.torque_ftlb / tq * TORQUE_AIM_FRAC;
        proportional.min(last - TORQUE_BACKOFF_STEP).max(0.0).min(want)
    } else if tq >= l.torque_ftlb * TORQUE_HOLD_FRAC {
        want.min(last)
    } else {
        want
    }
}

/// Republished on every swap, including to `None`
/// ([`crate::flight_state::publish_airframe`]).
pub fn publish_climb_out(c: Option<ClimbOut>) {
    *CLIMB_OUT.lock().unwrap_or_else(|e| e.into_inner()) = c;
}

/// The declaration, for a takeoff that hands off at or above its height. A
/// circuit hands off at 500 ft into its own climb law and keeps the old
/// climb-out whole: half a profile is worse than either.
fn climb_out_for(handoff_agl_ft: f64) -> Option<ClimbOut> {
    CLIMB_OUT
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .filter(|c| handoff_agl_ft >= c.accel_agl_ft)
}

/// Climb rate bought per knot above the initial climb speed, below the
/// acceleration height. A 737 loses about a knot a second per 1,000 fpm, so
/// this closes the speed loop at about 0.1 rad/s: slow against the five
/// seconds of lag and dead time in its VS mode (sixty degrees of margin).
const SPEED_HOLD_FPM_PER_KT: f64 = 100.0;

/// The climb-out's commanded rate. Below the acceleration height the speed is
/// held by climbing: every knot over `target_kts` buys rate, up to the
/// airframe's best. At and above it, and on any airframe that declares none,
/// the protected climb every takeoff has always flown.
pub fn climb_out_vs_fpm(
    ias_kts: f64,
    target_kts: f64,
    base_fpm: f64,
    vne_kts: Option<f64>,
    agl_ft: f64,
    climb_out: Option<ClimbOut>,
) -> f64 {
    let base = climb_vs_protected(ias_kts, target_kts, base_fpm, vne_kts);
    match climb_out {
        Some(c) if agl_ft < c.accel_agl_ft => {
            let hold = base + (ias_kts - target_kts).max(0.0) * SPEED_HOLD_FPM_PER_KT;
            hold.min(c.max_fpm.max(base))
        }
        _ => base,
    }
}

/// How long the handoff waits, still climbing, for the flap SCHEDULE to
/// finish on an airframe that declares a climb-out. The handoff's own
/// clean-up is one shot to zero on the clean floor: flaps 5 to up at 195 kt,
/// fifteen under the 737's flaps-up manoeuvring speed.
pub const SCHEDULE_WAIT_S: f64 = 45.0;
pub fn handoff_waits_for_schedule(retracted: bool, has_schedule: bool, waited_s: f64) -> bool {
    !retracted && has_schedule && waited_s < SCHEDULE_WAIT_S
}

/// The fastest the commanded climb moves once the autopilot has it, fpm per
/// second: about a tenth of a g of pitch change (0.1 x 9.81 m/s/s is 193 fpm
/// a second), a crew's hand lowering the nose, not a step.
///
/// LEMG 13 (the five circuits of 2026-09-28, e.g. 1790596335-full_auto): the
/// pitch was climbing 2,300 to 2,500 fpm through 300 ft and the engage wrote
/// 1,500; the circuit took it at 500 ft and wrote 456, a thousand fpm off the
/// dial in 0.8 s. After #78 the pitch climbs about 3,100 fpm at V2 + 15, so
/// the old engage would have been a 1,900 fpm push.
pub const CLIMB_VS_SLEW_FPM_S: f64 = 200.0;

/// `prev` toward `want` at [`CLIMB_VS_SLEW_FPM_S`] over `dt_s` (bounded to a
/// second, so a stalled tick cannot turn into a step). Pure.
pub fn climb_vs_slewed(prev: f64, want: f64, dt_s: f64) -> f64 {
    slew(prev, want, CLIMB_VS_SLEW_FPM_S * dt_s.clamp(0.0, 1.0))
}

/// The speed hold's integral, fpm per second per knot off the speed: the
/// proportional part ([`climb_out_vs_fpm`], 100 fpm a knot) closes near 0.1
/// rad/s against the 737's VS mode, and this puts the integral's corner a
/// quarter of that down (0.025 rad/s), about fourteen degrees of the margin.
const CLIMB_SPEED_TRIM_FPM_S_PER_KT: f64 = 2.5;
/// A speed-holding climb never commands less than this: airborne, it climbs.
const CLIMB_HOLD_FLOOR_FPM: f64 = 150.0;

/// The autopilot's climb from the 300 ft engage to the handoff (#150).
///
/// The engage TAKES the climb the pitch was flying (`flown_fpm`, the flight
/// model's vertical speed) and the command moves from there toward the law
/// at [`CLIMB_VS_SLEW_FPM_S`], never a step. Below a declared acceleration
/// height ([`ClimbOut`]) the law is a speed-holding climb: the proportional
/// speed hold plus an integral trim, seeded at the engage so that the law
/// itself starts at the climb flown. A proportional hold alone settles where
/// the excess thrust is spent, 100 fpm a knot over the speed: 190 kt at the
/// 737's 3,900 fpm of excess, where the pitch had held 165.
#[derive(Debug, Clone, Copy, Default)]
pub struct ClimbVs {
    /// The last command; `None` until the first step (the engage).
    cmd: Option<f64>,
    /// The speed hold's integral trim, fpm; `None` outside the hold.
    trim: Option<f64>,
}

impl ClimbVs {
    /// One tick: `(law, command)`, fpm. `flown_fpm` is read only at the
    /// first step; a failed read (`None`) is no measurement, and the engage
    /// then writes the law as it always did. Pure.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        ias_kts: f64,
        target_kts: f64,
        base_fpm: f64,
        vne_kts: Option<f64>,
        agl_ft: f64,
        climb_out: Option<ClimbOut>,
        flown_fpm: Option<f64>,
        dt_s: f64,
    ) -> (f64, f64) {
        let flown = flown_fpm.filter(|v| v.is_finite()).map(|v| v.max(0.0));
        let p = climb_out_vs_fpm(ias_kts, target_kts, base_fpm, vne_kts, agl_ft, climb_out);
        let law = match climb_out {
            Some(c) if agl_ft < c.accel_agl_ft => {
                let hi = c
                    .max_fpm
                    .max(climb_vs_protected(ias_kts, target_kts, base_fpm, vne_kts));
                let lo = CLIMB_HOLD_FLOOR_FPM.min(hi);
                let trim = match (self.trim, self.cmd) {
                    (Some(t), Some(_)) => {
                        t + CLIMB_SPEED_TRIM_FPM_S_PER_KT
                            * (ias_kts - target_kts)
                            * dt_s.clamp(0.0, 1.0)
                    }
                    // The engage: the law starts where the pitch is.
                    (_, None) => flown.map_or(0.0, |f| f - p),
                    // Back under a height the climb had left: from zero.
                    (None, Some(_)) => 0.0,
                };
                let trim = trim.clamp(lo - p, hi - p);
                self.trim = Some(trim);
                p + trim
            }
            _ => {
                self.trim = None;
                p
            }
        };
        let cmd = match self.cmd.or(flown) {
            Some(prev) => climb_vs_slewed(prev, law, dt_s),
            None => law,
        };
        self.cmd = Some(cmd);
        (law, cmd)
    }
}

/// Whether the handoff leaves the flaps where they are for the CALLER to
/// clean up on the schedule: an orchestrator that says it will
/// ([`Params::flaps_to_caller`], the circuit), an airframe with a schedule
/// to walk, and flaps still out. The one-shot clean-up at the circuit's 500
/// ft took the 737 from 1 to up one second after 5 to 1, at 191 kt against a
/// 210 kt flaps-up speed (LEMG 13, 2026-09-28). Pure.
pub fn leaves_flaps_to_caller(retracted: bool, flaps_to_caller: bool, has_schedule: bool) -> bool {
    !retracted && flaps_to_caller && has_schedule
}

/// What the takeoff hands its caller besides the aircraft: the climb it was
/// commanding and the flaps it left out (`None` when clean or cleaned up).
/// Set at the handoff, cleared at every start, taken once.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ClimbHandover {
    pub vs_cmd: Option<f64>,
    pub flaps_left: Option<f64>,
}

static CLIMB_HANDOVER: std::sync::Mutex<Option<ClimbHandover>> = std::sync::Mutex::new(None);

fn publish_climb_handover(h: Option<ClimbHandover>) {
    *CLIMB_HANDOVER.lock().unwrap_or_else(|e| e.into_inner()) = h;
}

/// The last completed takeoff's [`ClimbHandover`], once.
pub fn take_climb_handover() -> Option<ClimbHandover> {
    CLIMB_HANDOVER
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .take()
}

/// The speed the MCP window goes to once the takeoff accelerates: the flaps-up
/// manoeuvring speed a crew sets at the acceleration height (the schedule's
/// top retraction speed; the 737's 210), else the climb speed. Pure.
pub fn clean_up_speed_kts(schedule: &[(f64, f64)], vy_kts: f64) -> f64 {
    schedule
        .iter()
        .map(|(_, kts)| *kts)
        .filter(|k| k.is_finite() && *k > 0.0)
        .fold(None, |m: Option<f64>, k| Some(m.map_or(k, |m| m.max(k))))
        .unwrap_or(vy_kts)
}

/// The MCP speed window off V2 to `kts` (#150): written and read back
/// (within a knot). Only an airframe whose window the takeoff set to V2
/// (`takeoff_config: v2_kts`) is touched; the caller gates on that.
pub async fn speed_window_after_acceleration(xp: &dyn crate::xplane::XPlane, kts: f64) -> bool {
    use autoflight::Dial;
    autoflight::set_dial(xp, Dial::Airspeed, kts).await.is_ok()
        && autoflight::read_dial(xp, Dial::Airspeed)
            .await
            .is_ok_and(|v| (v - kts).abs() <= 1.0)
}

/// A roll leaving the centreline is rejected while it is still on the
/// pavement. LILB 03 (2026-09-25): the Evolution swung left at full power
/// with the steering pinned right, and the only reject in the roll was the
/// runway's END: it came at 68 kt, 129 m left of the centreline. Above
/// [`EXCURSION_MIN_KTS`], more than [`EXCURSION_XTK_M`] off the centreline
/// or [`EXCURSION_HDG_DEG`] off the runway's course is no takeoff. Pure.
pub fn lateral_excursion(xtk_m: f64, heading_off_deg: f64, gs_kts: f64) -> Option<&'static str> {
    if gs_kts < EXCURSION_MIN_KTS {
        return None;
    }
    if xtk_m.abs() > EXCURSION_XTK_M {
        Some("off_centreline")
    } else if heading_off_deg.abs() > EXCURSION_HDG_DEG {
        Some("off_heading")
    } else {
        None
    }
}
/// Below this the line-up's own straightening owns the error.
pub const EXCURSION_MIN_KTS: f64 = 10.0;
/// Half the narrowest runway anything here departs from (18 m), less a
/// wingtip's worth of margin.
pub const EXCURSION_XTK_M: f64 = 10.0;
pub const EXCURSION_HDG_DEG: f64 = 20.0;

/// Reject when the runway is running out and rotation speed is still short:
/// inside the margin with less than 90% of Vr there is no takeoff today.
pub fn must_reject(remaining_m: f64, ias_kts: f64, vr_kts: f64) -> bool {
    remaining_m < REJECT_REMAINING_M && ias_kts < vr_kts * 0.9
}

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

/// Maximum braking, held: a rejected takeoff is a maximum-effort stop.
pub(crate) const BRAKES_MAX: &str = "sim/flight_controls/brakes_max";
/// Under this a rejected roll has STOPPED and the parking brake goes on; the
/// hand-back's own stopped line (`full_auto::hand_back_hold`).
const REJECT_STOPPED_KTS: f64 = sim::ground::STOPPED_KTS;
/// The reject's park: a hand-back's own gates (a pilot on the toe brakes
/// keeps a fast stop).
const REJECT_HOLD: crate::full_auto::HoldFor = crate::full_auto::HoldFor::by("autotakeoff");
/// Below this the stop lets go of the steering: a walk on the centreline.
const REJECT_STEER_MIN_KTS: f64 = 3.0;
/// The longest the stop is flown, in live seconds (a pause slides it).
const REJECT_STOP_MAX_S: f64 = 90.0;
/// Maximum braking is renewed this often, inside the held command's second.
const REJECT_BRAKE_EVERY_S: f64 = 0.8;
/// The autothrottle is looked at this often through the stop.
const REJECT_AT_EVERY_S: f64 = 1.0;
/// The stop's tick.
const REJECT_TICK_MS: u64 = 200;

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
    let thr = throttle_dref();
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
async fn reject_rolling(
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
    let _ = xp.set_float(&throttle_dref(), 0.0).await;
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
async fn reject(xp: &XPlaneClient, voice: &Option<Voice>, line: &str, reason: &str) {
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
        let callsign = crate::circuit::crew_callsign(xp).await;
        crate::circuit::transmit(crate::circuit::radio_call(&callsign, REJECT_CALL));
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
async fn move_speed_window(
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
async fn clear_inherited_autopilot(xp: &dyn crate::xplane::XPlane) -> Option<bool> {
    let (servos, heading, vertical) = tokio::join!(
        xp.get_float(SERVOS_ON),
        xp.get_float(HDG_STATUS),
        xp.get_float(VS_STATUS)
    );
    let [servos, heading, vertical] =
        crate::control::measurements("autotakeoff", [servos, heading, vertical])?;
    if !ap_inherited(servos, heading, vertical) {
        return None;
    }
    let fired = autoflight::press(xp, Button::DisconnectFd).await.is_ok();
    let after = xp.get_float(SERVOS_ON).await.ok();
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
async fn no_pitch_take_over(xp: &XPlaneClient, p: &Params, voice: &Option<Voice>, why: &str) {
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

/// A mode command is only a request; handoff needs live mode and dial readbacks.
/// The SERVOS are read too: lit modes are not an autopilot flying (#160).
/// The servos go in through the one engage door (`ap_engage::press_logged`:
/// the flight director first, which this press skipped until the refactor
/// audit's #5, each press `ap_engage` on the tape, `tries` counting them);
/// the caller re-calls this every tick until everything reads back, so the
/// confirmation is this function's own read-back, and the VS dial is still
/// written AFTER the modes (an engage syncs it away).
async fn engage_climb(
    xp: &dyn crate::xplane::XPlane,
    fd: &[String],
    tries: &mut u32,
    vs: f64,
    last: &mut Option<f64>,
) -> bool {
    if crate::xplane::sim_frozen(xp).await {
        return false;
    }
    let (heading, vertical, servos) = tokio::join!(
        xp.get_float(HDG_STATUS),
        xp.get_float(VS_STATUS),
        xp.get_float(SERVOS_ON)
    );
    let Some([heading, vertical, servos]) =
        crate::control::measurements("autotakeoff", [heading, vertical, servos])
    else {
        return false;
    };
    if heading <= 0.5 || vertical <= 0.5 || servos <= 0.5 {
        *tries += 1;
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
        xp.get_float(HDG_STATUS),
        xp.get_float(VS_STATUS),
        xp.get_float(SERVOS_ON)
    );
    matches!((heading, vertical, servos), (Ok(h), Ok(v), Ok(s))
        if h.is_finite() && v.is_finite() && s.is_finite() && h > 0.5 && v > 0.5 && s > 0.5)
}

async fn fly(xp: XPlaneClient, voice: Option<Voice>, p: Params) {
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
    let cands = match crate::autoland::nearby_runways(&p.apt_dat, lat, lon, 2.0) {
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
    let config =
        crate::takeoff_config::set_for_takeoff(&xp, p.takeoff_flap_ratio, p.level_ft, true).await;
    if let Some(line) = crate::takeoff_config::line("Takeoff config", &config) {
        alert(&voice, Priority::Advisory, &line);
    }
    // An abort said during the flap wait: the brakes are still set.
    if xp.cancelled() || !ACTIVE.load(Ordering::Relaxed) {
        return;
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
        let _ = xp.set_float(FLAP_HANDLE, ratio).await;
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
    let mut wings = WingsLevel::default();
    let mut last_vs_cmd: Option<f64> = None;
    // The servo presses of the 300 ft engage (`engage_climb`).
    let mut engage_tries: u32 = 0;
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
        let thr_dref = throttle_dref();
        let (st, extra) = sim::state::read_with(
            &xp,
            sim::state::Want(ROLL_WANT),
            &[thr_dref.as_str(), autoflight::names::AT_ON, autoflight::names::AT_MODE],
        )
        .await;
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
        // The levers are the takeoff's: an ENGAGED autothrottle (armed is
        // the configuration's, never touched) is taken out and said once.
        {
            let (on, mode) = (extra.next(), extra.next());
            let engaged = autoflight::autothrottle_engaged(
                on.and_then(|r| r.ok()),
                mode.and_then(|r| r.ok()),
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
        let (thr_want, reducing) = thrust_reduced(thr_want, airborne, agl_ft, reduction);
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
                                   "from": last_thr_cmd, "to": thr_want, "n1": n1}),
            );
        }
        // FAIL OPEN, deliberately: a failed throttle read impersonates our own
        // command, so an API blink can never register as a pilot pull mid-roll.
        // The cost is one tick of blindness to a real hand; the alternative is
        // a takeoff handed away by a hiccup.
        let thr_now = thr_read.unwrap_or(thr_cmd);
        if thr_now < last_thr_cmd - 0.25 && elapsed > 2.0 {
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

        if !airborne {
            // Not accelerating: something is holding the aircraft.
            if elapsed > STAGNANT_SECS && gs_kts < STAGNANT_GS_KTS {
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
            if !rotated && must_reject(remaining, ias, p.vr_kts) {
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
            if !rotated {
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
                    "event": "roll", "gs": gs_kts, "ias": ias, "err": err,
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
            let _ = xp.set_float(GEAR_HANDLE, 0.0).await;
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
                    let _ = xp.set_float(FLAP_HANDLE, 0.0).await;
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
                    let _ = xp.set_float(FLAP_HANDLE, next).await;
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
        let (base_fpm, climb_out) = if DEPARTURE_MINIMUMS.load(Ordering::Relaxed) {
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
        );
        if !engage_climb(&xp, &p.cfg.fd_commands, &mut engage_tries, vs, &mut last_vs_cmd).await {
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
                               "law_vs": law_vs, "vs": vs_flown}),
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
                    let _ = xp.set_float(FLAP_HANDLE, 0.0).await;
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
                let _ = xp.set_float(GEAR_HANDLE, 0.0).await;
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

/// The recorded LEMG 13 circuit climb-outs and the 737's VS mode, shared by
/// this module's tests and the circuit's (#150).
#[path = "autotakeoff/climb_out_bench_tests.rs"]
#[cfg(test)]
pub(crate) mod climb_out_bench;

#[cfg(test)]
mod tests;

/// The pilot's reject at LEMG 13 (2026-09-28, sortie 1790627884-full_auto,
/// stock 737), replayed through a plant: the stop is flown, the throttles
/// held at idle, the parking brake set once stopped, and only then "your
/// aircraft".
#[cfg(test)]
mod reject_stop_tests;
