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
use crate::xplane::{XPlane, XPlaneClient};

const LAT: &str = "sim/flightmodel/position/latitude";
const LON: &str = "sim/flightmodel/position/longitude";
const PSI_TRUE: &str = "sim/flightmodel/position/psi";
#[cfg(test)]
const GS_MS: &str = "sim/flightmodel/position/groundspeed";
const IAS: &str = "sim/flightmodel/position/indicated_airspeed";
#[cfg(test)]
const AGL_M: &str = "sim/flightmodel/position/y_agl";
const MAG_VAR: &str = "sim/flightmodel/position/magnetic_variation";
const ON_GROUND: &str = "sim/flightmodel/failures/onground_any";
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
const CMD_SERVOS_ON: &str = "sim/autopilot/servos_on";
/// Servos and flight director off: an autopilot left on by the last flight.
const CMD_AP_OFF: &str = "sim/autopilot/servos_fdir_off";
const SERVOS_ON: &str = sim::datarefs::SERVOS_ON;
const CMD_HDG_MODE: &str = "sim/autopilot/heading";
const CMD_VS_MODE: &str = "sim/autopilot/vertical_speed";
const VS_DIAL: &str = "sim/cockpit/autopilot/vertical_velocity";
const HDG_STATUS: &str = "sim/cockpit2/autopilot/heading_status";
const VS_STATUS: &str = "sim/cockpit2/autopilot/vvi_status";
/// Elevator trim, -1 (nose down) .. +1, recorded at the AP engage: the one
/// number that says whether the climb the servo is handed is flyable.
const ELEV_TRIM: &str = "sim/cockpit2/controls/elevator_trim";
const ALT_DIAL: &str = "sim/cockpit2/autopilot/altitude_dial_ft";
const ALT_IND_FT: &str = sim::datarefs::ALTITUDE_FT;
const TAXI_STEER: &str = "xcopilot/taxi_steer";
const STEER_YIELD: &str = "xcopilot/steer_yield";
const TAKEOFF_PITCH: &str = "xcopilot/pitch";
const PITCH_YIELD: &str = "xcopilot/pitch_yield";
const TAKEOFF_ROLL: &str = "xcopilot/roll";
const ROLL_YIELD: &str = "xcopilot/roll_yield";
/// Engine torque, N m per engine.
const ENGN_TRQ: &str = "sim/flightmodel/engine/ENGN_TRQ";
/// N1 per engine, percent: recorded at the thrust reduction, the evidence
/// for the declared climb lever.
const ENGN_N1: &str = "sim/flightmodel/engine/ENGN_N1_";
const THETA: &str = "sim/flightmodel/position/true_theta";
const PHI: &str = "sim/flightmodel/position/true_phi";
/// The sim's own pitch rate, deg/s — what the rotation law damps on. Measured,
/// so it needs neither differencing nor a filter; see [`RotatePitch::step`].
const PITCH_RATE: &str = "sim/flightmodel/position/Q";
/// Yaw rate, deg/s, nose right positive: the roll law's damping.
const YAW_RATE: &str = "sim/flightmodel/position/R";

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
/// Above this the rudder is aerodynamic and the trim scales with 1/v².
const TRIM_SCALE_FROM_KTS: f64 = 40.0;
/// The roll law SCHEDULED on a declared rudder
/// ([`crate::aircraft::RollRudderProfile`]): the gain per degree is this
/// closure rate divided by the yaw rate a unit of steer holds at the speed,
/// so the heading closes at the same pace at 40 kt and at Vr (never more per
/// degree than [`ROLL_FULL_DEG`] allows). Damped by [`STEER_LEAD_S`] of the
/// sim's MEASURED yaw rate. Chosen over the EGKK-fitted plant with its rudder
/// at half, as declared and at two and a half times: 1.3 rang at 3.5 times.
const RUDDER_CLOSE_PER_S: f64 = 1.0;
/// The trim's corner, per second of the same gain: it takes a steady
/// crosswind in about five seconds (EGKK 26L: 0.2 of steer from 50 kt).
/// 0.45 and up wound up on a recovery from an offset with a weak rudder.
const RUDDER_TRIM_PER_S: f64 = 0.3;
/// ...and its cap: about 0.23 held an 8 kt crosswind at 60 kt.
const RUDDER_TRIM_MAX: f64 = 0.4;
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
/// A pilot advances the lever over seconds, never in one movement: 2.5 s
/// from 30% to full was the "0 to 100" the pilot called out (2026-09-25).
const THROTTLE_RAMP_S: f64 = 6.0;
/// Centreline correction: degrees of cut per meter of offset, clamped. The
/// figure is quoted AT [`XTK_GAIN_REF_KTS`] and scaled down above it - see
/// [`roll_target_heading`].
const XTK_GAIN_DEG_PER_M: f64 = 0.4;
const XTK_MAX_DEG: f64 = 8.0;
/// The pace the cut's gain was fitted at. Below it nothing changes; above it
/// the gain is scaled by ref/gs, which holds the lateral CLOSURE TIME
/// constant instead of the angle.
const XTK_GAIN_REF_KTS: f64 = 40.0;
/// How far ahead the cut looks, in seconds of cross-track closure. The lead
/// the outer loop needs to cover the inner loop's lag: the nose takes a
/// second or two to reach a commanded cut and as long again to give it back.
const XTK_LEAD_S: f64 = 2.0;
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
const AIL_FULL_BANK_DEG: f64 = 15.0;
const AIL_LEAD_S: f64 = 0.3;
const AIL_MAX: f64 = 0.5;
const AIL_SLEW_PER_S: f64 = 2.0;
/// The tick the stiff wings-level gain was set on, seconds (see
/// `WingsLevel::step`).
const STIFF_TUNED_TICK_S: f64 = 0.3;
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
static RUN: sim::run::Slot = sim::run::Slot::new();
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
        .filter(|r| heading_diff(psi_true, r.course_deg) <= ALIGN_MAX_DEG)
        .min_by(|a, b| lateral(a).total_cmp(&lateral(b)));
    let best = aligned.or_else(|| {
        cands.iter().min_by(|a, b| {
            heading_diff(psi_true, a.course_deg).total_cmp(&heading_diff(psi_true, b.course_deg))
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
    let err = heading_diff(psi_true, r.course_deg);
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
            && heading_diff(psi_true, r.course_deg) <= ALIGN_MAX_DEG
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

/// The heading to hold on the roll: runway course with a bounded cut back
/// toward the centreline (`xtk_m` right-positive, same convention as
/// autoland's final; `xtk_rate_m_s` right-positive too).
///
/// THE CUT IS SIZED FOR A CLOSURE TIME, NOT AN ANGLE PER METRE. A fixed
/// 0.4 deg/m closes an offset in `1 / (V * gain)` seconds, so the faster the
/// aircraft the tighter the outer loop winds - at 40 kt that is 7 seconds and
/// the loop is placid, at 150 kt it is under 2 and sits on top of the inner
/// loop's own lag. The pair then oscillates, and it did at both ends of the
/// same sortie (EGLF 24 / LFPB 03, 2026-09-22): the takeoff wandered
/// +1.7, -4.6, +10.1 m and LIFTED OFF ten metres right of the centreline,
/// and the landing rollout wove +3.1 / -5.7 m through four degrees of heading
/// before it damped. Scaling the gain by `ref/gs` above [`XTK_GAIN_REF_KTS`]
/// holds the closure time at the placid figure for every speed; below that
/// pace nothing changes, so the low-speed behaviour the takeoff was tuned
/// around (the sortie-three pirouette) is untouched.
///
/// [`XTK_LEAD_S`] is the other half: the cut is taken against where the
/// offset WILL be, not where it is. Autoland's final learned this years
/// earlier ("P alone converged asymptotically and S-ed under wind") and the
/// roll never got it.
pub fn roll_target_heading(course_deg: f64, xtk_m: f64, xtk_rate_m_s: f64, gs_kts: f64) -> f64 {
    (course_deg - roll_cut_deg(xtk_m, xtk_rate_m_s, gs_kts)).rem_euclid(360.0)
}

/// The cut [`roll_target_heading`] takes off the course, degrees (right of
/// the centreline positive, so the target moves left).
fn roll_cut_deg(xtk_m: f64, xtk_rate_m_s: f64, gs_kts: f64) -> f64 {
    let offset = if xtk_m.is_finite() { xtk_m } else { 0.0 };
    let rate = if xtk_rate_m_s.is_finite() {
        xtk_rate_m_s
    } else {
        0.0
    };
    // A missing or absurd groundspeed reads as the reference pace, which is
    // the gain this has always had.
    let pace = if gs_kts.is_finite() {
        gs_kts.abs().max(XTK_GAIN_REF_KTS)
    } else {
        XTK_GAIN_REF_KTS
    };
    let gain = XTK_GAIN_DEG_PER_M * (XTK_GAIN_REF_KTS / pace);
    ((offset + rate * XTK_LEAD_S) * gain).clamp(-XTK_MAX_DEG, XTK_MAX_DEG)
}

/// The cross-track RATE the centreline law leads with, smoothed. Raw, it is
/// a position differenced across a jittery tick: 0.8, 4.1, 5.4 and 7.4 m/s in
/// neighbouring ticks of a 737 roll whose real drift was about 1 m/s (LEMG,
/// 2026-09-26), and `XTK_LEAD_S` of lead turns each spike into degrees of
/// target heading, so the rudder flicked side to side down the runway. A
/// reading past [`XTK_RATE_MAX_M_S`] is noise, not a movement; the rest is
/// averaged over [`XTK_RATE_TAU_S`].
#[derive(Debug, Default, Clone, Copy)]
pub struct XtkRate {
    value: Option<f64>,
}

impl XtkRate {
    pub fn update(&mut self, raw_m_s: f64, dt_s: f64) -> f64 {
        if !raw_m_s.is_finite() || !dt_s.is_finite() || dt_s <= 0.0 {
            return self.value.unwrap_or(0.0);
        }
        let raw = raw_m_s.clamp(-XTK_RATE_MAX_M_S, XTK_RATE_MAX_M_S);
        let a = (dt_s / (XTK_RATE_TAU_S + dt_s)).clamp(0.0, 1.0);
        let v = match self.value {
            Some(v) => v + (raw - v) * a,
            None => raw * a,
        };
        self.value = Some(v);
        v
    }
}

/// The aircraft's own DRIFT on the roll: its track across the runway less
/// its heading, the angle a heading held dead on course still carries it
/// sideways by. The cut in [`roll_target_heading`] is proportional, and its
/// gain falls as 1/v to hold the closure time, so a steady drift is held off
/// only by a standing offset that grows with speed: drift / gain, 1.6 m at
/// 40 kt and 6 m at 150 for two thirds of a degree.
///
/// EGBB 33 (1790534902, 2026-09-27): about 3 kt of crosswind from the left,
/// the scheduled rudder law holding the heading within 0.3 degrees of its
/// target the whole roll, and the aircraft drifting right at 0.35 to 0.45
/// m/s from 95 kt: track less heading a steady 0.6 to 0.7 degrees, 0.9
/// once rotated. The cut had reached 0.5 degrees at 4 m; the wheels left
/// the runway 5.7 m right of the centreline.
///
/// Fed forward, like the final's wind crab (`autoland::CrabFilter`): the
/// heading target less this puts the TRACK where the cut asks. Read from
/// the cross-track rate the cut already uses (`asin(rate / v)` is the track
/// across the runway) less the heading off the runway smoothed by the SAME
/// [`XTK_RATE_TAU_S`], so a nose swing reaches both sides together and is
/// not read as drift, then low-passed over [`DRIFT_TAU_S`] and bounded.
/// Nothing below [`DRIFT_FROM_KTS`], where a centimetre of position jitter
/// is a degree of track.
#[derive(Debug, Default, Clone, Copy)]
pub struct DriftFilter {
    /// Heading less course, smoothed like the cross-track rate.
    heading: Option<f64>,
    est: Option<f64>,
}

/// The drift is read from this groundspeed (the cut's own reference pace).
pub const DRIFT_FROM_KTS: f64 = 40.0;
/// Its smoothing, seconds: three times the rate's, so the rate's spikes
/// (1.3 m of position in one EGBB tick at 50 kt) come through at a tenth.
pub const DRIFT_TAU_S: f64 = 4.0;
/// A raw reading past this is a swerve, not a drift.
const DRIFT_RAW_MAX_DEG: f64 = 3.0;
/// The most drift the target may carry.
pub const DRIFT_MAX_DEG: f64 = 2.0;

impl DriftFilter {
    /// One tick: heading less runway course (degrees, right positive), the
    /// smoothed cross-track rate ([`XtkRate`], m/s right positive),
    /// groundspeed and the tick. Returns the drift to take off the target,
    /// degrees (track right of heading positive).
    pub fn update(&mut self, psi_off_deg: f64, rate_m_s: f64, gs_kts: f64, dt_s: f64) -> f64 {
        if !psi_off_deg.is_finite() || !rate_m_s.is_finite() || !gs_kts.is_finite() {
            return self.value();
        }
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return self.value();
        }
        let a = (dt_s / (XTK_RATE_TAU_S + dt_s)).clamp(0.0, 1.0);
        let heading = match self.heading {
            Some(h) => h + (psi_off_deg - h) * a,
            None => psi_off_deg,
        };
        self.heading = Some(heading);
        if gs_kts < DRIFT_FROM_KTS {
            return self.value();
        }
        let v = gs_kts * 0.514_444;
        let track = (rate_m_s / v).clamp(-1.0, 1.0).asin().to_degrees();
        let raw = (track - heading).clamp(-DRIFT_RAW_MAX_DEG, DRIFT_RAW_MAX_DEG);
        let b = (dt_s / (DRIFT_TAU_S + dt_s)).clamp(0.0, 1.0);
        let e = self.est.unwrap_or(0.0);
        self.est = Some((e + (raw - e) * b).clamp(-DRIFT_MAX_DEG, DRIFT_MAX_DEG));
        self.value()
    }

    /// The current estimate; zero before it has read anything.
    pub fn value(&self) -> f64 {
        self.est.unwrap_or(0.0)
    }
}

/// The fastest a cross-track reading may move and still be believed, m/s.
pub const XTK_RATE_MAX_M_S: f64 = 5.0;
/// The smoothing of the cross-track rate, seconds.
pub const XTK_RATE_TAU_S: f64 = 1.0;

/// On a scheduled roll (`roll_rudder:`) at the declared rudder's speed and
/// over, the cut is this many times [`roll_target_heading`]'s, blended in
/// from [`XTK_GAIN_REF_KTS`] (see [`roll_fast_blend`]): a lateral closure of
/// about 3.5 s where the cut alone holds 7.
pub const ROLL_CUT_FAST_MULT: f64 = 2.0;
/// The cross-track trim: degrees of cut per metre-second of offset at
/// [`XTK_GAIN_REF_KTS`], scaled by ref/gs like the cut, from
/// [`DRIFT_FROM_KTS`]...
pub const XTK_TRIM_DEG_PER_M_S: f64 = 0.06;
/// ...and the most it may cut at that pace, degrees (less as the speed rises).
pub const XTK_TRIM_MAX_DEG: f64 = 1.0;
/// Ceiling on the scheduled law's gain from the declared rudder's speed:
/// never more than full steer per this many degrees (the default law's
/// [`ROLL_FULL_DEG`] below [`XTK_GAIN_REF_KTS`], blended between).
const RUDDER_FULL_DEG_FAST: f64 = 8.0;

/// How far a scheduled roll is into holding the line harder: 0 below
/// [`XTK_GAIN_REF_KTS`] (the pace the cut is quoted at, and nothing below it
/// changes), 1 from the speed the airframe's rudder was measured at
/// (`roll_rudder:` `at_kts`, 80 on the 737), linear between.
fn roll_fast_blend(gs_kts: f64, rudder_at_kts: f64) -> f64 {
    if !gs_kts.is_finite() {
        return 0.0;
    }
    let span = rudder_at_kts - XTK_GAIN_REF_KTS;
    if span.is_nan() || span <= 1.0 {
        return if gs_kts >= XTK_GAIN_REF_KTS { 1.0 } else { 0.0 };
    }
    ((gs_kts - XTK_GAIN_REF_KTS) / span).clamp(0.0, 1.0)
}

/// One tick of the roll's centreline cut, as [`RollCut::update`] computes it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CutTick {
    /// The cross-track the cut was taken on, metres right (the read, or on a
    /// scheduled roll the median of the last three reads).
    pub xtk_m: f64,
    /// The smoothed cross-track rate it leads with, m/s right.
    pub rate_m_s: f64,
    /// The drift fed forward ([`DriftFilter`]), degrees; zero on the default law.
    pub drift_deg: f64,
    /// The proportional cut, degrees (right of the line positive).
    pub cut_deg: f64,
    /// The cross-track trim's cut, degrees; zero on the default law.
    pub cut_i_deg: f64,
    /// The heading target less the runway course, degrees.
    pub target_off_deg: f64,
}

/// The centreline half of the takeoff roll: from the cross-track read and
/// the heading off the runway to the heading target, the one path the roll
/// loop and its replays share.
///
/// Every airframe gets the cut it flew ([`roll_target_heading`], plus the
/// [`DriftFilter`] on a `roll_rudder:` airframe). A SCHEDULED roll adds
/// three things, each read off the 737 rolls of 2026-09-28 (EGKK 08R lifted
/// off 5.4 m left, LEMG 13 1790591256 9.0 m left at 150 kt):
///
/// - The cross-track is the MEDIAN of the last three reads. Latitude and
///   longitude arrive in separate requests, so a read can pair two frames:
///   on LEMG's diagonal runway that is up to 2.9 m of cross-track for one
///   tick above 120 kt (0.7 at EGKK's east-west one), and each spike is
///   metres of target and a flick of rudder. A single-tick spike never
///   passes a median; a real movement is late by one tick. The source is
///   fixed since (#157: the plugin's `xcopilot/position`, one frame in one
///   request); the median stays as the defence for an older plugin's split
///   reads and any other single-tick glitch.
/// - From [`XTK_GAIN_REF_KTS`] the cut grows to [`ROLL_CUT_FAST_MULT`] times
///   the closure-time cut: the heading loop under it now holds the line, and
///   a 7 s lateral closure let a yaw gust at 100 kt carry the aircraft 4 m.
/// - A slow trim on the cross-track itself ([`XTK_TRIM_DEG_PER_M_S`], capped
///   at [`XTK_TRIM_MAX_DEG`]): whatever the drift feed and the heading trim
///   leave standing, it takes out by rotation.
#[derive(Debug, Default, Clone)]
pub struct RollCut {
    /// The declared rudder's speed on a scheduled roll; `None` on the default law.
    rudder_at_kts: Option<f64>,
    /// The cut as flown on 2026-09-28, for the replays that pin the change.
    #[cfg(test)]
    as_flown: bool,
    recent: std::collections::VecDeque<f64>,
    prev_xtk: Option<f64>,
    rate: XtkRate,
    drift: DriftFilter,
    /// Metre-seconds of cross-track, for the trim.
    integral: f64,
}

impl RollCut {
    /// The cut for a roll on the law scheduled on a rudder measured at
    /// `rudder_at_kts`, or (`None`) on the default law.
    pub fn new(rudder_at_kts: Option<f64>) -> Self {
        Self {
            rudder_at_kts,
            ..Self::default()
        }
    }

    /// The cut for the roll this steering law flies.
    pub fn for_law(law: &RollSteer) -> Self {
        Self::new(law.rudder.map(|r| r.at_kts))
    }

    #[cfg(test)]
    fn as_flown(mut self) -> Self {
        self.as_flown = true;
        self
    }

    fn improved(&self) -> bool {
        #[cfg(test)]
        if self.as_flown {
            return false;
        }
        self.rudder_at_kts.is_some()
    }

    /// One tick: the cross-track read (metres right), the heading less the
    /// runway course (degrees right), groundspeed and the tick.
    pub fn update(&mut self, xtk_read_m: f64, psi_off_deg: f64, gs_kts: f64, dt_s: f64) -> CutTick {
        let improved = self.improved();
        let xtk = if improved && xtk_read_m.is_finite() {
            self.recent.push_back(xtk_read_m);
            while self.recent.len() > 3 {
                self.recent.pop_front();
            }
            if self.recent.len() == 3 {
                let mut v: Vec<f64> = self.recent.iter().copied().collect();
                v.sort_by(|a, b| a.total_cmp(b));
                v[1]
            } else {
                xtk_read_m
            }
        } else {
            xtk_read_m
        };
        let raw = match self.prev_xtk {
            Some(p) if dt_s > 0.0 => (xtk - p) / dt_s,
            _ => 0.0,
        };
        self.prev_xtk = Some(xtk);
        let rate = self.rate.update(raw, dt_s);
        let drift = if self.rudder_at_kts.is_some() {
            self.drift.update(psi_off_deg, rate, gs_kts, dt_s)
        } else {
            0.0
        };
        let mut cut = roll_cut_deg(xtk, rate, gs_kts);
        let mut cut_i = 0.0;
        if improved {
            let at = self.rudder_at_kts.unwrap_or(XTK_GAIN_REF_KTS);
            let mult = 1.0 + (ROLL_CUT_FAST_MULT - 1.0) * roll_fast_blend(gs_kts, at);
            cut = (cut * mult).clamp(-XTK_MAX_DEG, XTK_MAX_DEG);
            if gs_kts >= DRIFT_FROM_KTS && xtk.is_finite() && dt_s.is_finite() && dt_s > 0.0 {
                let lim = XTK_TRIM_MAX_DEG / XTK_TRIM_DEG_PER_M_S;
                self.integral = (self.integral + xtk * dt_s).clamp(-lim, lim);
            }
            let pace = if gs_kts.is_finite() {
                gs_kts.abs().max(XTK_GAIN_REF_KTS)
            } else {
                XTK_GAIN_REF_KTS
            };
            cut_i = XTK_TRIM_DEG_PER_M_S * self.integral * (XTK_GAIN_REF_KTS / pace);
        }
        CutTick {
            xtk_m: xtk,
            rate_m_s: rate,
            drift_deg: drift,
            cut_deg: cut,
            cut_i_deg: cut_i,
            target_off_deg: -(cut + cut_i) - drift,
        }
    }
}

/// Slew limit: move from `prev` toward `want` by at most `max_step`.
pub fn slew(prev: f64, want: f64, max_step: f64) -> f64 {
    prev + (want - prev).clamp(-max_step, max_step)
}

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
    /// Groundspeed at the last tick, for the trim's speed scaling.
    prev_gs: Option<f64>,
    out: f64,
    /// No propeller, no P-factor to lead: a jet takes no power lead
    /// (`for_airframe`). The default is the propeller law.
    jet: bool,
    /// The airframe's measured rudder, when its profile declares one: the
    /// law is then scheduled on it (see [`RollSteer::step_with_yaw`]).
    rudder: Option<crate::aircraft::RollRudderProfile>,
    /// The engine torque last read, ft-lb, for a lead declared on it
    /// (`roll_rudder:` `power_steer_per_kftlb`); `None` leads on the lever.
    torque_ftlb: Option<f64>,
    /// The schedule as flown on 2026-09-28, for the replays that pin the change.
    #[cfg(test)]
    as_flown: bool,
}

/// The rudder led in WITH the power: proportional to the throttle
/// actually commanded, fading partially as airflow builds.
pub fn power_lead(thr: f64, gs_kts: f64) -> f64 {
    POWER_LEAD_MAX * thr * (1.0 - 0.6 * (gs_kts / POWER_LEAD_FADE_KTS).clamp(0.0, 1.0))
}

impl RollSteer {
    /// The law for the live airframe: a JET gets no power lead. The lead is
    /// right rudder against a propeller's left-turning tendency, and fed
    /// into the SF50's roll it WAS the swing: 0.1 to 0.25 of right rudder at
    /// full power, the heading error building to 18 degrees against a
    /// saturated command and the aircraft 60 m right of the centreline by
    /// 60 kt (LOWI 26, 2026-09-24).
    pub fn for_airframe() -> Self {
        let jet = crate::aircraft::live_profile().is_some_and(|p| {
            p.reference
                .as_ref()
                .and_then(|r| r.kind.as_deref())
                .is_some_and(|k| k.to_ascii_lowercase().contains("jet"))
        });
        Self {
            jet,
            rudder: crate::aircraft::roll_rudder(),
            ..Self::default()
        }
    }

    /// The law scheduled on a measured rudder (tests and replays).
    pub fn with_rudder(jet: bool, rudder: crate::aircraft::RollRudderProfile) -> Self {
        Self {
            jet,
            rudder: rudder.valid(),
            ..Self::default()
        }
    }

    /// The torque read this tick (ft-lb; `None` on a failed read), for the
    /// standing rudder a turboprop declares on it. A failed read leads on
    /// the lever, as `power_steer` does.
    pub fn set_torque(&mut self, torque_ftlb: Option<f64>) {
        self.torque_ftlb = torque_ftlb.filter(|t| t.is_finite());
    }

    /// Whether this law is scheduled on a declared rudder.
    pub fn scheduled(&self) -> bool {
        self.rudder.is_some()
    }

    /// The schedule as flown on 2026-09-28 (the declared v² extrapolated to
    /// Vr, [`ROLL_FULL_DEG`] at every speed).
    #[cfg(test)]
    fn as_flown(mut self) -> Self {
        self.as_flown = true;
        self
    }

    /// The groundspeed the declared rudder is read at, and the degrees of
    /// error full steer is never asked for under, at this groundspeed.
    fn schedule(&self, rudder: crate::aircraft::RollRudderProfile, gs_kts: f64) -> (f64, f64) {
        #[cfg(test)]
        if self.as_flown {
            return (gs_kts, ROLL_FULL_DEG);
        }
        let blend = roll_fast_blend(gs_kts, rudder.at_kts);
        let full = ROLL_FULL_DEG + (RUDDER_FULL_DEG_FAST - ROLL_FULL_DEG) * blend;
        (gs_kts.min(rudder.at_kts), full)
    }

    /// One control tick: heading error (deg, right positive), the time
    /// since the last tick, groundspeed, and the throttle being
    /// commanded (for the power feed-forward). Returns the deflection.
    pub fn step(&mut self, err_deg: f64, dt_s: f64, gs_kts: f64, thr: f64) -> f64 {
        self.step_with_yaw(err_deg, dt_s, gs_kts, thr, None)
    }

    /// [`RollSteer::step`] with the sim's yaw rate (deg/s, nose right
    /// positive, `sim/flightmodel/position/R`) for the damping. The default
    /// law ignores it and keeps its differenced error, as flown.
    ///
    /// With a declared rudder the law is SCHEDULED on it. EGKK 26L
    /// (2026-09-27, twice): an 8 kt crosswind from the left needed about 0.23
    /// of right rudder from 40 kt on, and the default law gave it 0.03 of
    /// steer per degree of error over 50 kt (its authority floor, set for the
    /// 172's lively rudder) plus a trim learning 0.01 per degree-second that
    /// the 1/v² scaling bled away nearly as fast as it learned. The nose sat
    /// 5 degrees left for ten seconds and both rolls were rejected 10 m left
    /// of the centreline at 82 and 87 kt. Here every gain is a yaw rate
    /// divided by the yaw rate a unit of steer holds at this speed, so the
    /// heading closes at [`RUDDER_CLOSE_PER_S`] whatever the speed; the
    /// same lead is taken on the MEASURED yaw rate, not a differenced heading
    /// (the error's own difference carries every step of the target); the
    /// trim learns at [`RUDDER_TRIM_PER_S`] of the same gain and shrinks as 1/v
    /// with the speed, which is how a crosswind's steer falls (its moment
    /// grows as v, the rudder's as v²). Never more per degree than the
    /// default law's full authority, so the slow roll is the one it was.
    pub fn step_with_yaw(
        &mut self,
        err_deg: f64,
        dt_s: f64,
        gs_kts: f64,
        thr: f64,
        yaw_dps: Option<f64>,
    ) -> f64 {
        let dt = dt_s.clamp(0.05, 1.0);
        let rate = self.prev_err.map_or(0.0, |p| (err_deg - p) / dt);
        self.prev_err = Some(err_deg);
        if let Some(rudder) = self.rudder {
            // A failed read is no measurement: the error's own rate stands in.
            let yaw = yaw_dps.filter(|v| v.is_finite()).unwrap_or(-rate);
            return self.step_scheduled(rudder, err_deg, dt, gs_kts, thr, yaw);
        }
        let auth = 1.0 - (1.0 - STEER_AUTH_FLOOR) * ((gs_kts - 15.0) / 35.0).clamp(0.0, 1.0);
        let pd = ((err_deg + STEER_LEAD_S * rate) / ROLL_FULL_DEG).clamp(-1.0, 1.0) * auth;
        // Prop wash makes the tail HYPER-effective under power at low
        // speed: the cap is tiny parked and grows with the roll, so an
        // engage-time misalignment is walked out over the first hundred
        // meters instead of spot-turned through (the sortie-three pirouette).
        let cap = LOW_SPEED_STEER_CAP
            + (MAX_ROLL_STEER - LOW_SPEED_STEER_CAP)
                * (gs_kts / LOW_SPEED_FULL_KTS).clamp(0.0, 1.0);
        let lead = if self.jet { 0.0 } else { power_lead(thr, gs_kts) };
        let want = (pd + self.trim + lead).clamp(-cap, cap);
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
        // The trim is a yawing MOMENT held as a deflection, and the rudder's
        // moment grows with the square of the airspeed: the same deflection
        // that just held the line at 70 kt swings the aircraft at 110.
        // LIBP 04 (2026-09-26, 737): the trim built to 0.2 of right rudder
        // drifting left through 50-90 kt, the rudder came alive at 95 and
        // swung it through the centreline to 9.5 m right at 124 kt with the
        // heading error asking LEFT and the trim still outvoting it; the
        // excursion reject followed. So the trim shrinks as the speed rises.
        // (A fast bleed when the error turns against it was tried against
        // the plants in the tests and made every strong-disturbance roll
        // worse: the scaling alone is the fix.)
        if let Some(pg) = self.prev_gs.filter(|pg| *pg > TRIM_SCALE_FROM_KTS && gs_kts > *pg) {
            self.trim *= (pg / gs_kts).powi(2);
        }
        self.prev_gs = Some(gs_kts);
        self.out = slew(self.out, want, STEER_SLEW_PER_S * dt);
        self.out
    }

    fn step_scheduled(
        &mut self,
        rudder: crate::aircraft::RollRudderProfile,
        err_deg: f64,
        dt: f64,
        gs_kts: f64,
        thr: f64,
        yaw_dps: f64,
    ) -> f64 {
        // The declared rudder is believed up to the speed it was measured at
        // and not past it. It was fitted from 30 to 86 kt (EGKK 26L) as
        // growing with v²; the 737 rolls of 2026-09-28, fitted to Vr the same
        // way (three-second heading predictions from the logged steer, 0.3 s
        // dead time, 0.47 s lag), hold 4 to 6 deg/s per unit from 45 to
        // 110 kt and 8.5 at 132, and the 27th's rolls refitted to Vr 7 to
        // 12.6: about v^1.25, never v². Extrapolated, the schedule read 31
        // at 132 kt and cut the gain to a third: the nose sat a degree off
        // its target for thirty seconds from 100 kt at EGKK 08R, steer
        // creeping 0.03 to 0.09, and the aircraft lifted off 5.4 m left.
        // Held at the declared figure above its speed the heading closes in
        // about a second at Vr on either day's rudder.
        let (at_kts, full_deg) = self.schedule(rudder, gs_kts);
        let authority = rudder.yaw_dps_at(at_kts).max(1e-3);
        // Where the rudder is still dead the gain would be enormous: never
        // more per degree than the default law's full authority at a walk,
        // [`RUDDER_FULL_DEG_FAST`] once the roll is under way (from 60 kt the
        // declared rudder is already stronger than either day's measured one,
        // so the old ceiling was the gain there).
        //
        // The closure and the trim's corner are the airframe's where it
        // declares them (`roll_rudder:` `close_per_s`, `trim_per_s`): a need
        // the lead and the trim did not predict is held off at need / kp
        // degrees. The SR22's second LEMG 13 roll (1790682305, 2026-09-29)
        // needed 0.5 at 45 to 75 kt in 3 kt of crosswind and 0.2 by 88 kt;
        // at 0.032 per degree the nose sat 4 degrees left, then swung 5
        // right with the steer still right, 4.1 m left to 7.5 m right on
        // the wheels. Its lag is shorter than the 737's (0.3 s fitted), and
        // 1.5 per second with a corner of 0.45 holds both of its rolls.
        let close = rudder.close_per_s.unwrap_or(RUDDER_CLOSE_PER_S);
        let kp = (close / authority).min(1.0 / full_deg);
        let pd = (kp * (err_deg - STEER_LEAD_S * yaw_dps)).clamp(-1.0, 1.0);
        let cap = LOW_SPEED_STEER_CAP
            + (MAX_ROLL_STEER - LOW_SPEED_STEER_CAP)
                * (gs_kts / LOW_SPEED_FULL_KTS).clamp(0.0, 1.0);
        let lead = if self.jet {
            0.0
        } else if let (Some(per), Some(tq)) = (rudder.power_steer_per_kftlb, self.torque_ftlb) {
            // On the torque, not the lever: the Evolution's torque limiter
            // cycled the lever 0.55 to 0.76 while the need followed the
            // torque, 0.51 per 1,000 ft-lb, flat to 78 kt; led on the lever
            // and faded past 60 kt it gave 0.21 at 78 kt against 0.65 needed
            // and the nose sat 5 to 8 degrees left (LILB 03 1790736651).
            (per * tq.max(0.0) / 1000.0 * rudder.power_steer_share(gs_kts)).min(MAX_ROLL_STEER)
        } else if let Some(ps) = rudder.power_steer {
            // Believed to the speed it was measured at, then falling as v^2:
            // held flat, 0.35 met a need of 0.2 at 88 kt.
            ps * thr.clamp(0.0, 1.0) * rudder.power_steer_share(gs_kts)
        } else {
            power_lead(thr, gs_kts)
        };
        let want = (pd + self.trim + lead).clamp(-cap, cap);
        if gs_kts > TRIM_MIN_GS_KTS {
            let e = err_deg.clamp(-STEER_TRIM_ERR_CLAMP_DEG, STEER_TRIM_ERR_CLAMP_DEG);
            let corner = rudder.trim_per_s.unwrap_or(RUDDER_TRIM_PER_S);
            self.trim =
                (self.trim + corner * kp * e * dt).clamp(-RUDDER_TRIM_MAX, RUDDER_TRIM_MAX);
        }
        if let Some(pg) = self
            .prev_gs
            .filter(|pg| *pg > TRIM_SCALE_FROM_KTS && gs_kts > *pg)
        {
            self.trim *= pg / gs_kts;
        }
        self.prev_gs = Some(gs_kts);
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

/// The climb-out ailerons: P on bank error with a rate lead, slew-limited.
/// Bank positive = right wing down; aileron positive = roll right — the
/// same convention, so the error maps straight through.
#[derive(Default)]
pub struct WingsLevel {
    out: f64,
    prev_bank: Option<f64>,
    /// Bank error for full aileron, degrees; `None` is [`AIL_FULL_BANK_DEG`].
    full_bank_deg: Option<f64>,
    /// A declared roll-rate plant (`arrest_roll:`): the law flies the bank
    /// its written ailerons will make, not the one read.
    plant: Option<crate::aircraft::ArrestRollProfile>,
    /// The ailerons written inside the plant's dead time: (age s, aileron,
    /// seconds it was held).
    pipe: std::collections::VecDeque<(f64, f64, f64)>,
}

impl WingsLevel {
    /// A stiffer loop: full aileron at `full_bank_deg` of bank error. The
    /// climb-out's 15 degrees is a gentle track-keeper; over the runway the
    /// landing arrest needs the bank to arrive in about a second, and at 15
    /// the 737 rolled at 2 deg/s on 0.17 of aileron (LEMG 13, 2026-09-26).
    pub fn stiff(full_bank_deg: f64) -> Self {
        Self {
            full_bank_deg: Some(full_bank_deg.clamp(2.0, AIL_FULL_BANK_DEG)),
            ..Self::default()
        }
    }

    /// On a ROLL-RATE plant (an Airbus's sidestick, `arrest_roll:`): the
    /// aileron that closes the bank the ailerons already written WILL make
    /// on the target over the profile's `close_s`. The stiff law, led only
    /// by the bank's measured rate, rang on the A330 through its dead time
    /// (+/-2 to 4 degrees at 1.5 to 2 s, LEMG 30, 2026-09-28).
    pub fn rate_plant(p: crate::aircraft::ArrestRollProfile) -> Self {
        Self {
            plant: Some(p),
            ..Self::default()
        }
    }

    /// The bank the ailerons already written will have made once they have
    /// all arrived, degrees: the reading on a stiff law.
    pub fn predicted(&self, bank_deg: f64) -> f64 {
        match self.plant {
            Some(p) => {
                bank_deg
                    + self.pipe.iter().map(|(_, a, held)| p.deg_s_per_unit * a * held).sum::<f64>()
            }
            None => bank_deg,
        }
    }

    pub fn step(&mut self, bank_deg: f64, target_deg: f64, dt_s: f64) -> f64 {
        let dt = dt_s.clamp(0.05, 1.0);
        if let Some(p) = self.plant {
            // The last write has been held this tick; what has aged past
            // the dead time is in the reading already.
            if let Some(last) = self.pipe.back_mut() {
                last.2 = dt;
            }
            for e in self.pipe.iter_mut() {
                e.0 += dt;
            }
            self.pipe.retain(|e| e.0 < p.dead_s + 1e-9);
            let pred = self.predicted(bank_deg);
            let want =
                ((target_deg - pred) / (p.deg_s_per_unit * p.close_s)).clamp(-AIL_MAX, AIL_MAX);
            self.out = slew(self.out, want, AIL_SLEW_PER_S * dt);
            self.prev_bank = Some(bank_deg);
            self.pipe.push_back((0.0, self.out, 0.0));
            return self.out;
        }
        let rate = self.prev_bank.map_or(0.0, |p| (bank_deg - p) / dt);
        self.prev_bank = Some(bank_deg);
        let err = target_deg - bank_deg - AIL_LEAD_S * rate;
        // The stiff law's gain was set on a 0.25 to 0.3 s tick, and the
        // tick is the loop's delay: at the iMac's 0.4 to 0.44 s (19 fps) the
        // 737's arrest rang +/-5 degrees on the aileron's stops through the
        // flare and touched down banked 9.9 (EGKK 08R 1790687142,
        // 2026-09-29). A slower tick softens it in proportion.
        let full = match self.full_bank_deg {
            Some(f) => f * (dt / STIFF_TUNED_TICK_S).max(1.0),
            None => AIL_FULL_BANK_DEG,
        };
        let want = (err / full).clamp(-1.0, 1.0) * AIL_MAX;
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

/// Newton-metres to foot-pounds (`ENGN_TRQ` is in N m).
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
    const WINDOW: &str = crate::aircraft::TakeoffConfigProfile::SPEED_WINDOW;
    xp.set_float(WINDOW, kts).await.is_ok()
        && xp
            .get_float(WINDOW)
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
const REJECT_STOPPED_KTS: f64 = 1.0;
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
            let err = ((target - psi + 540.0).rem_euclid(360.0)) - 180.0;
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
    let held = crate::full_auto::hand_back_hold(xp, valve_set).await;
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
    let fired = xp.run_command(CMD_AP_OFF).await.is_ok();
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
    let (dial, alt) = tokio::join!(xp.get_float(ALT_DIAL), xp.get_float(ALT_IND_FT));
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
async fn engage_climb(xp: &dyn crate::xplane::XPlane, vs: f64, last: &mut Option<f64>) -> bool {
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
    if (heading <= 0.5 || vertical <= 0.5 || servos <= 0.5)
        && xp.run_command(CMD_SERVOS_ON).await.is_err()
    {
        return false;
    }
    if heading <= 0.5 && xp.run_command(CMD_HDG_MODE).await.is_err() {
        return false;
    }
    if vertical <= 0.5 {
        *last = None; // a mode change synchronises the dial
        if xp.run_command(CMD_VS_MODE).await.is_err() {
            return false;
        }
    }
    if !crate::control::set_confirmed("autotakeoff", xp, VS_DIAL, vs, last, 40.0).await {
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
    crate::tools::speedhold::ACTIVE.store(false, Ordering::Relaxed);
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
    let roll_start = std::time::Instant::now();
    let mut frozen_secs = 0.0_f64;
    loop {
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
        let thr_dref = throttle_dref();
        let (
            pos,
            ias,
            on_ground,
            steer_yield,
            thr_read,
            theta,
            phi,
            pitch_q,
            pitch_yield,
            roll_yield,
            torque_nm,
            yaw_r,
        ) = tokio::join!(
            // Position, heading, groundspeed, height and vertical speed as
            // ONE frame saw them (#157): read separately, latitude and
            // longitude paired two frames, up to 2.9 m of cross-track for a
            // tick on LEMG 13. An older plugin falls back to the separate
            // reads, and the tick says which (`pos`).
            sim::position::read(&xp),
            xp.get_float(IAS),
            xp.get_float(ON_GROUND),
            xp.get_float(STEER_YIELD),
            xp.get_float(&thr_dref),
            xp.get_float(THETA),
            xp.get_float(PHI),
            xp.get_float(PITCH_RATE),
            xp.get_float(PITCH_YIELD),
            xp.get_float(ROLL_YIELD),
            xp.get_float_array(ENGN_TRQ),
            xp.get_float(YAW_RATE),
        );
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
            .and_then(|v| v.first().copied())
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
        let _ = xp.set_float(&throttle_dref(), thr_cmd).await;
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
            let psi_off = ((psi - rwy.course_deg + 540.0).rem_euclid(360.0)) - 180.0;
            let cut = roll_cut.update(xtk_m, psi_off, gs_kts, dt);
            let (xtk_rate, drift) = (cut.rate_m_s, cut.drift_deg);
            let target = (rwy.course_deg + cut.target_off_deg).rem_euclid(360.0);
            let err = ((target - psi + 540.0).rem_euclid(360.0)) - 180.0;
            last_err = err;
            if !rotated {
                let off_course = ((psi - rwy.course_deg + 540.0).rem_euclid(360.0)) - 180.0;
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
                let _ = tokio::join!(
                    xp.set_float(crate::autotaxi::LEFT_BRAKE, bl),
                    xp.set_float(crate::autotaxi::RIGHT_BRAKE, br),
                );
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
            if xp.set_float(TAXI_STEER, rudder).await.is_err() {
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
                let (wp, wr) = tokio::join!(
                    xp.set_float(TAKEOFF_PITCH, pitch),
                    xp.set_float(TAKEOFF_ROLL, ail)
                );
                if wp.is_err() || wr.is_err() {
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
            }
            // Airborne only once clearly clear of the surface: a ground-
            // effect bounce (on_ground false at 2 ft, back down a tick
            // later — second sortie) must not stop the steering while the
            // wheels are about to touch again at speed.
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
            tokio::time::sleep(std::time::Duration::from_millis(150)).await;
            continue;
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
                let hdg_err = ((rwy.course_deg - psi + 540.0).rem_euclid(360.0)) - 180.0;
                pitch_law.on_wheels(false);
                pitch_law.climb_fpm(vs_flown);
                let pitch = pitch_law.step_climb(theta_deg, pitch_rate, dt, Some((ias, agl_ft)));
                let ail = wings.step(phi_deg, climb_bank_target(hdg_err), dt);
                crate::monitor::note_crew_yoke();
                let (wp, wr) = tokio::join!(
                    xp.set_float(TAKEOFF_PITCH, pitch),
                    xp.set_float(TAKEOFF_ROLL, ail)
                );
                if wp.is_err() || wr.is_err() {
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
                tokio::time::sleep(std::time::Duration::from_millis(150)).await;
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
        if !engage_climb(&xp, vs, &mut last_vs_cmd).await {
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
                let (dial, alt) = tokio::join!(xp.get_float(ALT_DIAL), xp.get_float(ALT_IND_FT));
                if let (Ok(dial), Ok(alt)) = (dial, alt) {
                    let level = match handoff_altitude_dial_ft(dial, alt) {
                        Some(set) => {
                            let ok = xp.set_float(ALT_DIAL, set).await.is_ok();
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
#[cfg(test)]
pub(crate) mod climb_out_bench {
    const LEMG13: &str = include_str!("../bench/takeoffs/lemg13_circuit_climb_outs.jsonl");

    /// One climb-out as the blackbox drew it: `autotakeoff` `climb_manual`
    /// (t, agl, ias), the `ap_engaged` tick (t, agl, ias, cmd_vs), every VS
    /// command after it (`climb`, then the circuit's `leg`, as (t, cmd_vs)),
    /// the handoff time, and every flap handle move (t, ratio), the
    /// one-shot `flaps_handoff` as 0.
    pub struct Recorded {
        pub sortie: String,
        pub manual: Vec<(f64, f64, f64)>,
        pub engage: (f64, f64, f64, f64),
        pub commands: Vec<(f64, f64)>,
        pub handoff_t: f64,
        pub flaps: Vec<(f64, f64)>,
    }

    pub fn load() -> Vec<Recorded> {
        let recs: Vec<serde_json::Value> = LEMG13
            .lines()
            .filter_map(|l| serde_json::from_str(l).ok())
            .collect();
        let mut sorties: Vec<String> = recs
            .iter()
            .filter_map(|r| r["sortie"].as_str().map(str::to_string))
            .collect();
        sorties.dedup();
        let f = |r: &serde_json::Value, k: &str| r[k].as_f64().unwrap_or(f64::NAN);
        sorties
            .into_iter()
            .map(|sortie| {
                let mine: Vec<&serde_json::Value> = recs
                    .iter()
                    .filter(|r| r["sortie"] == sortie.as_str())
                    .collect();
                let ev =
                    |r: &serde_json::Value, k: &str, e: &str| r["kind"] == k && r["event"] == e;
                let manual = mine
                    .iter()
                    .filter(|r| ev(r, "autotakeoff", "climb_manual"))
                    .map(|r| (f(r, "t"), f(r, "agl"), f(r, "ias")))
                    .collect();
                let e = mine
                    .iter()
                    .find(|r| ev(r, "autotakeoff", "ap_engaged"))
                    .expect("an engage");
                let commands = mine
                    .iter()
                    .filter(|r| ev(r, "autotakeoff", "climb") || ev(r, "circuit", "leg"))
                    .map(|r| (f(r, "t"), f(r, "cmd_vs")))
                    .collect();
                let handoff_t = mine
                    .iter()
                    .find(|r| ev(r, "autotakeoff", "handoff"))
                    .map(|r| f(r, "t"))
                    .expect("a handoff");
                let flaps = mine
                    .iter()
                    .filter_map(|r| {
                        if ev(r, "autotakeoff", "flap_step") {
                            Some((f(r, "t"), f(r, "to")))
                        } else if ev(r, "autotakeoff", "flaps_handoff") && r["up"] == true {
                            Some((f(r, "t"), 0.0))
                        } else {
                            None
                        }
                    })
                    .collect();
                Recorded {
                    sortie,
                    manual,
                    engage: (f(e, "t"), f(e, "agl"), f(e, "ias"), f(e, "cmd_vs")),
                    commands,
                    handoff_t,
                    flaps,
                }
            })
            .collect()
    }

    /// The climb the pitch was flying as the autopilot took it: the slope
    /// of the last second of hand-flown heights, fpm (over the runway the
    /// height above it is the altitude less a constant).
    pub fn flown_fpm(manual: &[(f64, f64, f64)]) -> f64 {
        let t_end = manual.last().expect("hand-flown ticks").0;
        let last: Vec<&(f64, f64, f64)> = manual.iter().filter(|m| m.0 >= t_end - 1.0).collect();
        let n = last.len() as f64;
        let (mt, mh) = (
            last.iter().map(|m| m.0).sum::<f64>() / n,
            last.iter().map(|m| m.1).sum::<f64>() / n,
        );
        let num: f64 = last.iter().map(|m| (m.0 - mt) * (m.1 - mh)).sum();
        let den: f64 = last.iter().map(|m| (m.0 - mt).powi(2)).sum();
        num / den * 60.0
    }

    /// The 737's VS mode as EGPH 24 drew it (2026-09-21): 2 s of dead time,
    /// then two 1.5 s lags. The LEMG 13 engages agree: the dial read 1,500
    /// and the aircraft held 2,150 to 2,250 for the four seconds to 500 ft.
    pub struct VsMode {
        delay: std::collections::VecDeque<f64>,
        vs1: f64,
        pub vs: f64,
    }

    impl VsMode {
        pub fn new(vs: f64, dt: f64) -> Self {
            let n = (2.0 / dt).round() as usize;
            VsMode {
                delay: std::collections::VecDeque::from(vec![vs; n]),
                vs1: vs,
                vs,
            }
        }
        pub fn step(&mut self, cmd: f64, dt: f64) -> f64 {
            self.delay.push_back(cmd);
            let seen = self.delay.pop_front().unwrap_or(cmd);
            self.vs1 += (seen - self.vs1) * dt / 1.5;
            self.vs += (self.vs1 - self.vs) * dt / 1.5;
            self.vs
        }
    }

    /// Knots a second at `ias` climbing `vs_fpm` with a thrust-to-weight
    /// excess `tw` (#78's fit to LEMG 13: 0.185).
    pub fn accel_kts_s(tw: f64, ias: f64, vs_fpm: f64) -> f64 {
        let sin_gamma = (vs_fpm / (ias * 101.27)).clamp(-1.0, 1.0);
        (tw - sin_gamma) * 9.81 / 0.514_444
    }

    /// The climb the #78 pitch hold flies at V2 + 15 on that fit: every bit of
    /// the excess into the path, about 3,090 fpm at 165 kt.
    pub fn held_climb_fpm(tw: f64, ias: f64) -> f64 {
        ias * 101.27 * tw
    }
}

#[cfg(test)]
mod tests {

    /// EGKK 2026-09-27: the reject left the tower's takeoff clearance live.
    /// The crew now reports it, in words the tower reads as the reject and
    /// never as a request, and only when the tower has us on its runway.
    #[test]
    fn a_reject_is_reported_to_the_tower_that_cleared_it() {
        use crate::tower::{parse_intent, Intent, State};
        let call = crate::circuit::radio_call(
            &Some("Whiskey Alpha Tango one seven three".into()),
            super::REJECT_CALL,
        );
        assert_eq!(
            parse_intent(&call.to_ascii_lowercase()),
            Intent::RejectingTakeoff,
            "{call}"
        );
        assert!(super::reject_call_due(Some(State::TakeoffCleared)));
        assert!(super::reject_call_due(Some(State::LinedUp)));
        for quiet in [
            None,
            Some(State::Parked),
            Some(State::TaxiCleared),
            Some(State::Inbound),
        ] {
            assert!(!super::reject_call_due(quiet), "{quiet:?}");
        }
    }

    /// The Edinburgh refusal, from the measured geometry (2026-09-20): on
    /// runway 24, aligned to 0.9 degrees, dead on the centreline, 2,408 m
    /// along a 2,549 m strip. The fit correctly refused — 141 m of runway
    /// ahead against a 500 m minimum — and then said "we're not on the
    /// runway yet" while the tower said "you are on runway two four".
    /// ETND 2026-09-24, the recorded numbers: cleared 26R (the grass strip
    /// beside 08/26), stopped at 52.585512 N 8.349835 E facing 016 true, and
    /// the refusal named 08L, the reciprocal, "152 metres away". It names
    /// the cleared end now, at the same distance; without a clearance the
    /// best-aligned strip is still what it names.
    #[test]
    fn the_line_up_refusal_names_the_cleared_runway() {
        let dir = std::env::temp_dir().join(format!("xvl_etnd_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let apt = dir.join("apt.dat");
        std::fs::write(
            &apt,
            "1    126 0 0 ETND Diepholz AB\n\
             100 50.00 3 0 0.00 0 0 0 08L 52.5862935 8.3393953 0 0 1 0 0 0 26R 52.5866670 8.3459906 0 0 1 0 0 0\n\
             100 45.11 1 0 0.25 0 2 0  08 52.5850000 8.3318780 12 0 1 0 0 0 26 52.5859870 8.3501930 0 0 1 3 0 0\n",
        )
        .unwrap();
        let (lat, lon, psi) = (52.585511622928735, 8.349835096044318, 16.0);
        let cands = crate::autoland::nearby_runways(apt.to_str().unwrap(), lat, lon, 3.0).unwrap();
        assert!(runway_fit(&cands, lat, lon, psi, 780.0).is_none());
        let old = why_no_fit(&cands, lat, lon, psi, 780.0);
        let NoFit::NoneNear { id, lateral_m } = &old else {
            panic!("{old:?}");
        };
        assert_eq!(id, "08L", "the recorded refusal");
        assert!((lateral_m - 152.0).abs() < 1.0, "{lateral_m}");

        let why = why_no_fit_cleared(&cands, lat, lon, psi, 780.0, Some("26R"));
        let NoFit::NoneNear { id, lateral_m } = &why else {
            panic!("{why:?}");
        };
        assert_eq!(id, "26R");
        assert!((lateral_m - 152.0).abs() < 1.0, "{lateral_m}");
        let (line, _) = no_fit_line(&why, false);
        assert!(line.starts_with("Runway two six right is 152 metres away"), "{line}");
        assert!(!line.contains("zero eight"), "{line}");

        // No clearance, or one the data does not know: as before.
        assert_eq!(why_no_fit_cleared(&cands, lat, lon, psi, 780.0, None), old);
        assert_eq!(why_no_fit_cleared(&cands, lat, lon, psi, 780.0, Some("33")), old);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_refused_takeoff_says_which_of_the_four_things_is_wrong() {
        // The 2,000 m north strip, and a position 1,900 m down it: aligned,
        // on the centreline, 100 m of runway left — Edinburgh's shape.
        let cands = vec![rwy_north()];
        let far = (1900.0 / 111_320.0, 0.0);
        match why_no_fit(&cands, far.0, far.1, 0.0, 500.0) {
            NoFit::TooLittleLeft {
                id, remaining_m, ..
            } => {
                assert_eq!(id, "36");
                assert!(
                    remaining_m < 200.0,
                    "a hundred metres of runway left, not a line-up problem: {remaining_m:.0}"
                );
            }
            other => panic!("expected TooLittleLeft, got {other:?}"),
        }
        // Lying ACROSS the runway — on it, and the worst place to be.
        assert!(matches!(
            why_no_fit(&cands, 0.0, 0.0, 90.0, 500.0),
            NoFit::NotAligned { .. }
        ));
        // Aligned at the right end: this one fits, so there is no diagnosis
        // to give.
        assert!(runway_fit(&cands, 0.0, 0.0, 0.0, 500.0).is_some());
        // Nothing anywhere near.
        assert_eq!(
            why_no_fit(&[], 0.0, 0.0, 0.0, 500.0),
            NoFit::NoneNear {
                id: String::new(),
                lateral_m: 0.0
            }
        );
        // EGPH 2026-09-20: at the handoff height at 203 kt against a 210 kt
        // clean floor. The handoff waits, flying, and goes once clean or fast
        // enough, and never waits past its bound or for a clean wing.
        assert!(handoff_waits_for_clean_up(false, 203.0, Some(210.0), 0.0));
        assert!(!handoff_waits_for_clean_up(false, 211.0, Some(210.0), 3.0));
        assert!(!handoff_waits_for_clean_up(true, 203.0, Some(210.0), 0.0));
        assert!(!handoff_waits_for_clean_up(false, 203.0, None, 0.0));
        assert!(!handoff_waits_for_clean_up(
            false,
            203.0,
            Some(210.0),
            CLEAN_UP_WAIT_S
        ));
        // The dial at 100 ft passing 1,150: set 2,000 up, to the thousand.
        assert_eq!(handoff_altitude_dial_ft(100.0, 1150.0), Some(4000.0));
        assert_eq!(handoff_altitude_dial_ft(1150.0, 1150.0), Some(4000.0));
        // The pilot's own level above us stands.
        assert_eq!(handoff_altitude_dial_ft(6000.0, 1150.0), None);
        assert_eq!(handoff_altitude_dial_ft(1650.0, 1150.0), None);
        // The words never deny a runway we are sitting on.
        for why in [
            NoFit::RunwayTooShort {
                id: "24".into(),
                length_m: 2549.0,
                needed_m: 2560.0,
            },
            NoFit::TooLittleLeft {
                id: "24".into(),
                remaining_m: 141.0,
                needed_m: 2560.0,
            },
            NoFit::NotAligned {
                id: "24".into(),
                err_deg: 35.0,
            },
        ] {
            for can_line_up in [true, false] {
                let (line, urgent) = no_fit_line(&why, can_line_up);
                assert!(urgent, "{line}");
                assert!(!line.contains("not on the runway"), "{line}");
            }
        }

        // THE RUNWAY ITSELF TOO SHORT. Edinburgh's shape: on the numbers,
        // aligned, full length ahead — and the aircraft needs more than the
        // strip has. No position fixes this, and the old answer sent the
        // pilot to line up and then to backtrack, twice, for eleven metres.
        match why_no_fit(&cands, 0.0, 0.0, 0.0, 2560.0) {
            NoFit::RunwayTooShort {
                length_m, needed_m, ..
            } => {
                assert!(length_m < needed_m, "{length_m} vs {needed_m}");
            }
            other => panic!("a short runway is not a position problem: {other:?}"),
        }
        // ...and it is reported even from the threshold, where "not enough
        // LEFT" would be technically true and completely useless.
        assert!(matches!(
            why_no_fit(&cands, 0.0, 0.0, 0.0, 2560.0),
            NoFit::RunwayTooShort { .. }
        ));

        // NOTHING NEAR, but we know WHAT we looked for. LFPB 2026-09-22: the
        // cleared runway 07 lay 169 m off, and both the tool and full auto
        // said only "I couldn't get us onto the runway". A strip 400 m abeam
        // the north runway is that shape.
        let abeam = why_no_fit(&cands, 0.0, 400.0 / (111_320.0 * 0.0_f64.cos()), 0.0, 500.0);
        match &abeam {
            NoFit::NoneNear { id, lateral_m } => {
                assert_eq!(id, "36");
                assert!((*lateral_m - 400.0).abs() < 5.0, "{lateral_m}");
            }
            other => panic!("expected NoneNear with the strip named, got {other:?}"),
        }
        let (line, urgent) = no_fit_line(&abeam, false);
        assert!(
            line.contains("three six") && line.contains("400 metres"),
            "{line}"
        );
        assert!(
            !urgent,
            "beside the field is not an occupied runway: {line}"
        );
        // ...and with no pavement at all in the data, the old wording stands —
        // there is nothing truthful to name.
        let (bare, _) = no_fit_line(
            &NoFit::NoneNear {
                id: String::new(),
                lateral_m: 0.0,
            },
            false,
        );
        assert!(bare.contains("not lined up on a runway"), "{bare}");
    }

    /// The BIKF departure (2026-09-20): commanded 8 degrees, flew 22.4. The
    /// law was pinned at its nose-down limit the whole way with the pitch
    /// RATE still building — it wanted about -0.38 and could ask for -0.10.
    #[test]
    fn the_rotation_can_push_hard_enough_to_hold_its_own_ceiling() {
        // The trace, replayed: 20.1 degrees against an 8 degree target with
        // the nose still coming up at 2 deg/s.
        let (theta, rate) = (20.1, 2.05);
        let mut ga = RotatePitch::with_authority(None, PITCH_MIN);
        let mut jet = RotatePitch::with_authority(Some(11.0), -0.5);
        // Walk both to the cap so the ramp is finished and the target is 8.
        for _ in 0..40 {
            ga.step(8.0, Some(0.0), 0.2);
            jet.step(8.0, Some(0.0), 0.2);
        }
        let ga_out = ga.step(theta, Some(rate), 0.2);
        let jet_out = jet.step(theta, Some(rate), 0.2);
        assert!(
            (ga_out - PITCH_MIN).abs() < 1e-6,
            "the GA limit saturates and stays there: {ga_out}"
        );
        assert!(
            jet_out < ga_out - 0.2,
            "the jet can actually push: {jet_out} vs {ga_out}"
        );
        // ...and the limit is still a limit: never past full travel, never a
        // standing pull dressed up as a minimum.
        let silly = RotatePitch::with_authority(None, 4.0);
        assert!(silly.pitch_min <= 0.0);
        let deep = RotatePitch::with_authority(None, -9.0);
        assert!(deep.pitch_min >= -1.0);
    }

    /// Flaps come up a detent at a time, each step waiting for its own
    /// manoeuvring speed. The old law took flaps 5 straight to 0 at 170 kt.
    #[test]
    fn the_flaps_walk_their_schedule_instead_of_going_straight_to_zero() {
        // The 737's declared schedule: up at 210, flaps 1 at 190, 2 at 180.
        let sched = [(0.0, 210.0), (0.125, 190.0), (0.25, 180.0)];
        // Flaps 5 at 170 kt, which is what the old rule cleaned up at.
        assert_eq!(flap_retract_step(&sched, 0.375, 170.0), None, "too slow");
        // 190 earns the first step, and ONLY the first.
        assert_eq!(flap_retract_step(&sched, 0.375, 190.0), Some(0.25));
        // From flaps 1, going clean waits for 210.
        assert_eq!(flap_retract_step(&sched, 0.125, 200.0), None);
        assert_eq!(flap_retract_step(&sched, 0.125, 210.0), Some(0.0));
        // Already clean: nothing left.
        assert_eq!(flap_retract_step(&sched, 0.0, 250.0), None);
        // Fast enough for everything still only takes ONE step: the flaps
        // have to travel through the detents either way.
        assert_eq!(flap_retract_step(&sched, 0.375, 300.0), Some(0.25));
        // An airframe with no declared speeds keeps the one-shot clean-up.
        assert_eq!(flap_retract_step(&[], 0.375, 300.0), None);
    }

    /// Held to the acceleration height, a jet is past every detent's speed
    /// at once: the schedule stepped 5 to 2 and 2 to 1 half a second apart
    /// (Alghero, 2026-09-27). The next step waits for the flaps to arrive.
    #[test]
    fn a_flap_step_waits_for_the_flaps_to_arrive() {
        let sched = [(0.0, 210.0), (0.125, 190.0), (0.25, 180.0)];
        // First step: the flaps sit at the takeoff 5.
        assert_eq!(paced_flap_step(&sched, 0.375, Some(0.375), 215.0), Some(0.25));
        // Next tick: commanded 2, still travelling from 5. No second step.
        assert_eq!(paced_flap_step(&sched, 0.25, Some(0.36), 215.0), None);
        // A failed read is no measurement.
        assert_eq!(paced_flap_step(&sched, 0.25, None, 215.0), None);
        assert_eq!(paced_flap_step(&sched, 0.25, Some(f64::NAN), 215.0), None);
        // Arrived (inside the tolerance): the next step.
        assert_eq!(paced_flap_step(&sched, 0.25, Some(0.26), 215.0), Some(0.125));
        // Arrived but too slow is still too slow.
        assert_eq!(paced_flap_step(&sched, 0.125, Some(0.125), 200.0), None);
    }

    /// #169: the stock A330's deploy ratio never came within 0.03 of CONF 1's
    /// handle, and the walk never stepped again. A reading that left the old
    /// detent and stopped counts as arrived; one still moving, one that never
    /// left the old detent (a jammed flap), or a failed read does not.
    #[test]
    fn a_stalled_deploy_reading_still_steps() {
        // The A330's schedule: CONF 1 (0.25) cleans up above 210.
        let sched = [(0.0, 210.0), (0.25, 180.0)];
        let mut pace = FlapPace::default();
        pace.stepped(0.5);
        let step = |pace: &mut FlapPace, t: f64, d: Option<f64>| {
            let judged = pace.judged(t, 0.25, d);
            paced_flap_step(&sched, 0.25, judged, 212.0)
        };
        // Travelling from CONF 2: no step.
        assert_eq!(step(&mut pace, 0.0, Some(0.45)), None);
        assert_eq!(step(&mut pace, 1.0, Some(0.40)), None);
        // Stopped short at 0.33, 0.005 of wander: not yet four seconds.
        for i in 0..8 {
            let t = 2.0 + i as f64 * 0.5;
            let d = 0.33 + if i % 2 == 0 { 0.0 } else { 0.004 };
            assert_eq!(step(&mut pace, t, Some(d)), None, "t {t}");
        }
        // Four seconds still: the step.
        assert_eq!(step(&mut pace, 6.0, Some(0.332)), Some(0.0));
        // A failed read is no measurement.
        assert_eq!(step(&mut pace, 6.5, None), None);
        // A gap in the reads starts the clock again.
        assert_eq!(step(&mut pace, 20.0, Some(0.332)), None);

        // Jammed at the detent the step left: never an arrival.
        let mut jam = FlapPace::default();
        jam.stepped(0.5);
        for i in 0..40 {
            assert_eq!(step(&mut jam, i as f64 * 0.5, Some(0.5)), None);
        }
        // Still creeping (0.02 in 4 s): not settled.
        let mut creep = FlapPace::default();
        creep.stepped(0.5);
        for i in 0..20 {
            let t = i as f64 * 0.5;
            assert_eq!(step(&mut creep, t, Some(0.40 - t * 0.005)), None, "t {t}");
        }
        // Arrived inside the tolerance needs no settling at all.
        let mut near = FlapPace::default();
        assert_eq!(step(&mut near, 0.0, Some(0.26)), Some(0.0));
        // #169, LEMG 30 (sortie 1790698132-full_auto, 2026-09-29): the
        // A330's CONF 1 reading falls past the handle (0.106, then 0.011,
        // still moving): past the detent on the way in is arrived at once.
        let mut slats = FlapPace::default();
        slats.stepped(0.5);
        assert_eq!(step(&mut slats, 0.0, Some(0.106)), Some(0.0));
        assert_eq!(step(&mut slats, 0.7, Some(0.011)), Some(0.0));
        // ...but not with no step to judge the direction by, and never a
        // reading still above the detent.
        let mut blind = FlapPace::default();
        assert_eq!(step(&mut blind, 0.0, Some(0.106)), None);
        let mut above = FlapPace::default();
        above.stepped(0.5);
        assert_eq!(step(&mut above, 0.0, Some(0.30)), None);
        // The withheld line: the first, then every ten seconds.
        let mut log = FlapPace::default();
        assert!(log.withheld_log_due(0.0));
        assert!(!log.withheld_log_due(5.0));
        assert!(log.withheld_log_due(10.0));
    }

    #[test]
    fn a_flap_step_is_called_by_its_detent() {
        assert_eq!(flap_step_line(0.125, Some("one")), "Flaps one.");
        assert_eq!(flap_step_line(0.25, Some("ten degrees")), "Flaps ten degrees.");
        assert_eq!(flap_step_line(0.0, Some("up")), "Flaps up.");
        assert_eq!(flap_step_line(0.0, None), "Flaps up.");
        // No name: the old line.
        assert_eq!(flap_step_line(0.125, None), "Flaps coming up a stage.");
        assert_eq!(flap_step_line(0.125, Some("  ")), "Flaps coming up a stage.");
    }

    /// The 737 walks 5, 1, up from its real profile, each named.
    #[test]
    fn the_737_walks_five_one_up() {
        let flaps = crate::aircraft::load("boeing_737_800").unwrap().flaps;
        let sched = crate::aircraft::flap_schedule(flaps.as_ref());
        let (mut at, mut calls) = (0.375_f64, Vec::new());
        while let Some(next) = paced_flap_step(&sched, at, Some(at), 250.0) {
            at = next;
            let name = crate::aircraft::flap_detent_spoken(flaps.as_ref(), next);
            calls.push((next, flap_step_line(next, name.as_deref())));
        }
        assert_eq!(
            calls,
            vec![(0.125, "Flaps one.".to_string()), (0.0, "Flaps up.".to_string())]
        );
    }

    #[test]
    fn the_gear_waits_for_its_own_warning_to_clear() {
        // The SF50's .acf: below 125 kt, flaps past 0.5 or thrust under 50%.
        let sf50 = crate::aircraft::gear_warning_from_acf(
            "P acf/_gear_warn_above_flap_rat 0.500000000\n\
             P acf/_gear_warn_below_kias 125.000000000\n\
             P acf/_gear_warn_below_thro_pct 50.000000000\n",
        );
        assert_eq!(
            sf50,
            Some(crate::aircraft::GearWarning {
                below_kias: 125.0,
                below_throttle_pct: Some(50.0),
                above_flap_ratio: Some(0.5),
            })
        );
        let vlo = Some(160.0);
        // ETND 26: 200 ft and 122 kt with the takeoff flap, the horn.
        assert!(!gear_up_due(203.0, 122.0, 0.5, sf50, vlo));
        assert!(!gear_up_due(340.0, 125.6, 0.5, sf50, vlo), "inside the margin");
        assert!(gear_up_due(380.0, 128.7, 0.5, sf50, vlo));
        // Clean, the flap clause is gone and full thrust is not the power one.
        assert!(gear_up_due(203.0, 110.0, 0.0, sf50, vlo));
        // Never below the floor, always by the latest height or the placard.
        assert!(!gear_up_due(150.0, 150.0, 0.0, sf50, vlo));
        assert!(gear_up_due(1000.0, 110.0, 0.5, sf50, vlo));
        assert!(gear_up_due(400.0, 151.0, 0.5, sf50, vlo));
        // An airframe with no declared warning keeps the old 200 ft rule.
        assert!(gear_up_due(201.0, 80.0, 0.5, None, None));
        assert_eq!(crate::aircraft::gear_warning_from_acf("P acf/_Vle_kts 150\n"), None);
    }
    #[tokio::test]
    async fn paused_missing_or_refused_modes_cannot_complete_the_takeoff_handoff() {
        let xp = crate::xplane::FakeXPlane::with_reads(&[
            (crate::datarefs::PAUSED, 1.0),
            (crate::datarefs::IS_IN_REPLAY, 0.0),
            (super::HDG_STATUS, 0.0),
            (super::VS_STATUS, 0.0),
            (super::SERVOS_ON, 0.0),
            (super::VS_DIAL, 700.0),
        ]);
        let mut last = None;
        assert!(!super::engage_climb(&xp, 700.0, &mut last).await);
        assert!(!xp.ran(super::CMD_SERVOS_ON));
        assert!(!xp.wrote(super::VS_DIAL));
        xp.set_read(crate::datarefs::PAUSED, 0.0);
        assert!(!super::engage_climb(&xp, 700.0, &mut last).await); // commands ignored
        assert!(xp.ran(super::CMD_VS_MODE));
        xp.set_read(super::HDG_STATUS, 1.0);
        xp.set_read(super::VS_STATUS, 1.0);
        // Lit modes with the servos out are not an autopilot flying (#160).
        assert!(!super::engage_climb(&xp, 700.0, &mut last).await);
        xp.set_read(super::SERVOS_ON, 1.0);
        assert!(super::engage_climb(&xp, 700.0, &mut last).await);
        xp.set_read(super::VS_STATUS, f64::NAN);
        assert!(!super::engage_climb(&xp, 700.0, &mut last).await);
        xp.set_read(super::VS_STATUS, 1.0);
        xp.set_read(super::SERVOS_ON, f64::NAN);
        assert!(!super::engage_climb(&xp, 700.0, &mut last).await);
    }

    /// LEMG 13 SF50 (1790607325, #160): the roll began with HDG and VS lit
    /// from the SR22 before it, and the 300 ft engage pressed nothing, so
    /// nothing flew the climb. Lit modes with the servos out now press the
    /// servos in; lit modes on the ground are taken off before the roll.
    #[tokio::test]
    async fn an_autopilot_left_on_is_never_taken_for_an_engage() {
        // The engage: HDG and VS lit, servos out. The old gate read the two
        // modes, found them on, and pressed nothing.
        let xp = crate::xplane::FakeXPlane::with_reads(&[
            (crate::datarefs::PAUSED, 0.0),
            (crate::datarefs::IS_IN_REPLAY, 0.0),
            (super::HDG_STATUS, 1.0),
            (super::VS_STATUS, 1.0),
            (super::SERVOS_ON, 0.0),
            (super::VS_DIAL, 1500.0),
        ]);
        let mut last = None;
        let _ = super::engage_climb(&xp, 1500.0, &mut last).await;
        assert!(xp.ran(super::CMD_SERVOS_ON), "the servos are pressed in");
        // A clean start (every other recorded takeoff: HDG and VS off) is
        // left alone on the ground.
        let clean = crate::xplane::FakeXPlane::with_reads(&[
            (super::SERVOS_ON, 0.0),
            (super::HDG_STATUS, 0.0),
            (super::VS_STATUS, 0.0),
        ]);
        assert_eq!(super::clear_inherited_autopilot(&clean).await, None);
        assert!(!clean.ran(super::CMD_AP_OFF));
        // The recorded roll: ap on, HDG and VS lit.
        let left_on = crate::xplane::FakeXPlane::with_reads(&[
            (super::SERVOS_ON, 1.0),
            (super::HDG_STATUS, 1.0),
            (super::VS_STATUS, 1.0),
        ]);
        left_on.set_on_command(super::CMD_AP_OFF, super::SERVOS_ON, 0.0);
        assert_eq!(super::clear_inherited_autopilot(&left_on).await, Some(true));
        assert!(left_on.ran(super::CMD_AP_OFF), "taken off before the roll");
        // Lit modes alone (a flight director left with modes) count too.
        assert!(super::ap_inherited(0.0, 1.0, 0.0));
        assert!(super::ap_inherited(0.0, 0.0, 1.0));
        assert!(!super::ap_inherited(0.0, 0.0, 0.0));
        // A failed read is no measurement: nothing is pressed.
        let unread = crate::xplane::FakeXPlane::with_reads(&[
            (super::HDG_STATUS, 1.0),
            (super::VS_STATUS, 1.0),
        ]);
        assert_eq!(super::clear_inherited_autopilot(&unread).await, None);
        assert!(!unread.ran(super::CMD_AP_OFF));
    }

    use super::*;
    use crate::autoland::project_latlon;

    /// The handoff line is the most important sentence this module speaks,
    /// and it was wrong in two ways at once on every circuit (2026-08-01,
    /// caught live in the cockpit): it said "your aircraft" while the circuit
    /// was taking control, and it announced "one thousand feet" at the
    /// circuit's 500 ft handoff — a number the pilot could see was wrong on
    /// the altimeter, in the same breath as the claim about who was flying.
    #[test]
    fn the_handoff_names_the_real_height() {
        assert_eq!(spoken_height_ft(500.0), "five hundred feet");
        assert_eq!(spoken_height_ft(1000.0), "one thousand feet");
        assert_eq!(spoken_height_ft(1500.0), "one thousand five hundred feet");
        // It is called on the tick that CROSSES the height, so it is never
        // exactly round: round to the hundred a pilot would say.
        assert_eq!(spoken_height_ft(1013.0), "one thousand feet");
        assert_eq!(spoken_height_ft(517.0), "five hundred feet");
        assert_eq!(spoken_height_ft(2050.0), "two thousand one hundred feet");
    }

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

    /// Stansted's real geometry (Global Airports apt.dat): the 1,048 m
    /// 04C/22C row is a collinear sub-segment of the 3,052 m 04/22 pavement.
    /// Cleared "22C" and lined up mid-strip, the fit names "22" (the short
    /// row fails the length gate for a jet's factored roll) and the string
    /// compare refused the takeoff — EGSS, 2026-07-29, three times.
    #[test]
    fn collinear_duplicate_designators_pass_the_crosscheck() {
        // Course 222.8 for the 22 ends: threshold at the NE, rolling SW.
        let cands = vec![
            Runway {
                icao: "EGSS".into(),
                id: "22C".into(),
                thr_lat: 51.8895556,
                thr_lon: 0.2416333,
                course_deg: 222.8,
                length_m: 1048.0,
                surface: 1,
                dist_nm: 0.0,
                elev_ft: 348.0,
            },
            Runway {
                icao: "EGSS".into(),
                id: "22".into(),
                thr_lat: 51.8951583,
                thr_lon: 0.2500444,
                course_deg: 222.8,
                length_m: 3052.0,
                surface: 1,
                dist_nm: 0.0,
                elev_ft: 348.0,
            },
        ];
        // The stop the line-up delivered: on the shared centreline, 190 m
        // down the 22C row, nose roughly down the strip.
        let (lat, lon) = (51.88799, 0.23930);
        // A factored roll longer than the 22C row's remainder: the fit
        // skips 22C and names the long row.
        let (rwy, rem) = runway_fit(&cands, lat, lon, 223.0, 900.0).expect("on the pavement");
        assert_eq!(rwy.id, "22", "the short row fails the length gate");
        assert!(rem > 1500.0, "{rem}");
        assert!(!runway_matches(&rwy.id, "22C"), "the strings alone refuse");
        // The escape hatch: the cleared designator fits where we stand.
        assert!(cleared_fits_here(&cands, "22C", lat, lon, 223.0));
        // The traps stay armed: the reciprocal fails on heading, a name
        // not under the wheels fails outright.
        assert!(!cleared_fits_here(&cands, "04C", lat, lon, 223.0));
        assert!(!cleared_fits_here(&cands, "27", lat, lon, 223.0));
        // A true parallel 200 m abeam fails on cross-track.
        let mut parallel = cands[0].clone();
        parallel.id = "22R".into();
        let (plat, plon) =
            project_latlon(parallel.thr_lat, parallel.thr_lon, 132.8, 200.0 / 1852.0);
        parallel.thr_lat = plat;
        parallel.thr_lon = plon;
        assert!(!cleared_fits_here(&[parallel], "22R", lat, lon, 223.0));
    }

    #[test]
    fn cleared_runway_crosscheck_is_forgiving_of_format_only() {
        assert!(runway_matches("09R", "09R"));
        assert!(runway_matches("9", "09"));
        assert!(runway_matches("27l", "27L"));
        assert!(
            runway_matches("", "27"),
            "no fitted ident is not a mismatch"
        );
        assert!(
            runway_matches("27", ""),
            "no cleared ident is not a mismatch"
        );
        assert!(
            !runway_matches("27", "09"),
            "the backwards line-up must be caught"
        );
        assert!(!runway_matches("27L", "27R"));
    }

    #[test]
    fn roll_target_cuts_toward_the_centreline_bounded() {
        // 10 m right of the line at taxi pace: cut left of runway heading,
        // bounded, and exactly as it always was at or below the reference.
        let tgt = roll_target_heading(0.0, 10.0, 0.0, XTK_GAIN_REF_KTS);
        assert!(tgt > 350.0 && tgt < 360.0, "cut left, got {tgt}");
        assert!(
            (360.0 - tgt - 10.0 * XTK_GAIN_DEG_PER_M).abs() < 1e-9,
            "the reference pace keeps the gain it was fitted with: {tgt}"
        );
        assert_eq!(
            roll_target_heading(0.0, 10.0, 0.0, 5.0),
            tgt,
            "below the reference nothing changes - the low-speed roll is tuned"
        );
        // 100 m off (should never happen): still clamped to the max cut.
        let tgt = roll_target_heading(0.0, 100.0, 0.0, XTK_GAIN_REF_KTS);
        assert!((360.0 - tgt - XTK_MAX_DEG).abs() < 0.01, "{tgt}");
    }

    /// The cut is sized for a CLOSURE TIME, so the lateral closure a given
    /// offset earns is the same at every speed. That is the whole fix: a
    /// fixed angle per metre winds the outer loop tighter the faster you go,
    /// until it sits on the inner loop's lag and the pair weaves.
    #[test]
    fn the_roll_cut_closes_in_the_same_time_at_any_speed() {
        let closure_m_s = |gs_kts: f64| {
            let tgt = roll_target_heading(0.0, 10.0, 0.0, gs_kts);
            let cut = (360.0 - tgt).to_radians();
            gs_kts * MS_TO_KTS.recip() * cut.sin()
        };
        let reference = closure_m_s(XTK_GAIN_REF_KTS);
        for gs in [60.0, 100.0, 140.0, 160.0] {
            let got = closure_m_s(gs);
            assert!(
                (got - reference).abs() < reference * 0.02,
                "at {gs} kt a 10 m offset must still close at {reference:.2} m/s, got {got:.2}"
            );
        }
        // And the cut itself shrinks, which is the same statement said the
        // other way round: 4 degrees at 150 kt was the weave.
        let fast = 360.0 - roll_target_heading(0.0, 10.0, 0.0, 150.0);
        assert!(fast < 1.5, "a 10 m offset at 150 kt is a small cut: {fast}");
    }

    /// LFPB 03 and EGLF 24, 2026-09-22: the same law at both ends of one
    /// sortie, weaving. The rollout wandered +3.1 / -5.7 m through four
    /// degrees of heading; the takeoff went +1.7, -4.6, +10.1 and LIFTED OFF
    /// ten metres right of the centreline.
    ///
    /// The plant is the cascade that did it, fitted to the LFPB tick log: a
    /// rudder that commands a yaw RATE through a 0.7 s lag, the loop's own
    /// half-second of transport delay, and the ground geometry closing the
    /// cross-track. Both laws fly it, because a test that only shows the fix
    /// working proves nothing about the bug.
    /// LIBP 04 (2026-09-26, 737): a steady yaw needing right rudder through
    /// the low-speed roll, a rudder whose bite grows with the square of the
    /// airspeed. The trim built up drifting left, the rudder came alive at
    /// 95 kt and the old trim swung the roll 9.5 m right by 124 kt. The roll
    /// now stays inside a few metres all the way to rotation.
    /// LEMG (2026-09-26): a 1 m/s drift read through ±0.4 m of position
    /// noise on a jittery tick. Raw, the rate swung the target heading by
    /// degrees tick to tick; smoothed, it is steady.
    #[test]
    fn the_cross_track_rate_is_smoothed_before_it_is_led() {
        let (mut raw_t, mut filt_t) = (Vec::new(), Vec::new());
        let mut f = XtkRate::default();
        let (mut x, mut prev) = (0.0_f64, None::<f64>);
        for i in 0..80 {
            let dt = if i % 3 == 0 { 0.15 } else { 0.25 };
            x += 1.0 * dt;
            let noise = if i % 2 == 0 { 0.4 } else { -0.4 };
            let seen = x + noise;
            let raw = prev.map_or(0.0, |p| (seen - p) / dt);
            prev = Some(seen);
            let smooth = f.update(raw, dt);
            if i > 20 {
                raw_t.push(roll_target_heading(40.0, seen, raw, 150.0));
                filt_t.push(roll_target_heading(40.0, seen, smooth, 150.0));
            }
        }
        let swing = |v: &[f64]| v.windows(2).map(|w| (w[1] - w[0]).abs()).fold(0.0, f64::max);
        assert!(swing(&filt_t) < swing(&raw_t) * 0.25, "{:.2} vs {:.2}", swing(&filt_t), swing(&raw_t));
        assert!(swing(&filt_t) < 0.3, "{:.2} degrees tick to tick", swing(&filt_t));
    }

    #[test]
    fn a_trim_learned_slow_does_not_swing_the_roll_fast() {
        const COURSE: f64 = 40.0;
        const TICK: f64 = 0.2;
        let mut law = RollSteer { jet: true, ..RollSteer::default() };
        let (mut psi, mut yaw, mut xtk, mut gs) = (COURSE, 0.0_f64, 0.5_f64, 0.0_f64);
        let mut prev: Option<f64> = None;
        let mut pipe = std::collections::VecDeque::from(vec![0.0; 1]);
        let (mut worst, mut worst_fast) = (0.0_f64, 0.0_f64);
        while gs < 135.0 {
            let rate = prev.map_or(0.0, |p| (xtk - p) / TICK);
            prev = Some(xtk);
            let target = roll_target_heading(COURSE, xtk, rate, gs);
            let err = ((target - psi + 540.0).rem_euclid(360.0)) - 180.0;
            pipe.push_back(law.step(err, TICK, gs, 1.0));
            let steer = pipe.pop_front().unwrap_or(0.0);
            for _ in 0..10 {
                let h = TICK / 10.0;
                // Nosewheel steering fades by 45 kt, BEFORE the rudder has
                // come alive: the dip the trim winds up in.
                let nose = 6.0 * (1.0 - gs / 45.0).max(0.0);
                let aero = 22.0 * (gs / 110.0).powi(2);
                // A steady left-turning disturbance (crosswind, thrust).
                let want = (nose + aero) * steer - 0.9;
                yaw += (want - yaw) * (h / 0.8).min(1.0);
                psi += yaw * h;
                xtk += gs / MS_TO_KTS * (psi - COURSE).to_radians().sin() * h;
                gs += 3.2 * h;
            }
            worst = worst.max(xtk.abs());
            if gs > 90.0 {
                worst_fast = worst_fast.max(xtk.abs());
            }
        }
        // The law before the scaling: 7.8 m, 6.0 m above 90 kt.
        assert!(worst < 5.0, "the roll wandered {worst:.1} m");
        assert!(worst_fast < 2.0, "above 90 kt it swung {worst_fast:.1} m");
    }

    /// The 737's rudder as the profile declares it (`roll_rudder:`).
    fn rudder_737() -> crate::aircraft::RollRudderProfile {
        let text = std::fs::read_to_string("profiles/boeing_737_800.yaml").unwrap();
        let p: crate::aircraft::AircraftProfile = serde_yaml::from_str(&text).unwrap();
        p.roll_rudder
            .and_then(|r| r.valid())
            .expect("the 737 declares its rudder")
    }

    /// One tick of a flown roll, as the blackbox's `roll` event carries it.
    struct RollTick {
        gs: f64,
        err: f64,
        xtk: f64,
        steer: f64,
        trim: f64,
    }

    /// A 737 takeoff roll through the plant fitted to both EGKK 26L rolls of
    /// 2026-09-27 (30 to 86 kt, three-second predictions of the heading
    /// 0.07 degrees rms): a unit of steer holds 6.45 deg/s of yaw at 60 kt,
    /// growing with v² to the 86 kt that fit reached and as v^1.25 past it
    /// (every 737 roll of the 27th and 28th refitted to Vr: never v², 12.6
    /// at 132 kt where v² reads 31), reached through a 0.47 s lag after 0.3 s of dead
    /// time (second order from steer to heading), plus the nosewheel at a
    /// walk; the crosswind from the left yaws it into the wind by
    /// 2.48 deg/s per 60 kt less 0.99 at the recorded 7.4 kt, in proportion.
    /// The loop ticks at the recorded 0.27 s with the real target law and
    /// cross-track filter. `kscale` makes the real rudder weaker or stronger
    /// than the one declared.
    fn fly_737_roll(
        law: RollSteer,
        xw_kt: f64,
        kscale: f64,
        xtk0: f64,
        psi_off: f64,
        accel_kts_s: f64,
        gs0: f64,
        vr_kts: f64,
    ) -> Vec<RollTick> {
        fly_737_roll_drifting(
            law,
            xw_kt,
            kscale,
            xtk0,
            psi_off,
            accel_kts_s,
            gs0,
            vr_kts,
            0.0,
            false,
        )
    }

    /// [`fly_737_roll`] with the tyres letting the track run `slip_deg`
    /// right of the heading (EGBB 33: 0.6 to 0.7 degrees), and the loop's
    /// [`DriftFilter`] fed into the target when `feed`.
    #[allow(clippy::too_many_arguments)]
    fn fly_737_roll_drifting(
        mut law: RollSteer,
        xw_kt: f64,
        kscale: f64,
        xtk0: f64,
        psi_off: f64,
        accel_kts_s: f64,
        gs0: f64,
        vr_kts: f64,
        slip_deg: f64,
        feed: bool,
    ) -> Vec<RollTick> {
        const COURSE: f64 = 257.6;
        const TICK: f64 = 0.27;
        const DEAD_S: f64 = 0.3;
        const LAG_S: f64 = 0.47;
        const H: f64 = 0.01;
        let (mut psi, mut yaw, mut xtk, mut gs) = (COURSE + psi_off, 0.0_f64, xtk0, gs0);
        let mut queue: std::collections::VecDeque<(f64, f64)> = Default::default();
        let mut applied = 0.0_f64;
        let mut prev_xtk: Option<f64> = None;
        let mut rate_filter = XtkRate::default();
        let mut drift_filter = DriftFilter::default();
        let mut t = 0.0_f64;
        let mut ticks = Vec::new();
        while gs < vr_kts {
            let raw = prev_xtk.map_or(0.0, |p| (xtk - p) / TICK);
            prev_xtk = Some(xtk);
            let rate = rate_filter.update(raw, TICK);
            let drift = if feed {
                drift_filter.update(psi - COURSE, rate, gs, TICK)
            } else {
                0.0
            };
            let target = roll_target_heading(COURSE, xtk, rate, gs) - drift;
            let err = ((target - psi + 540.0).rem_euclid(360.0)) - 180.0;
            let steer = law.step_with_yaw(err, TICK, gs, 1.0, Some(yaw));
            queue.push_back((t + DEAD_S, steer));
            ticks.push(RollTick {
                gs,
                err,
                xtk,
                steer,
                trim: law.trim(),
            });
            let next = t + TICK;
            while t < next - 1e-9 {
                while queue.front().is_some_and(|(at, _)| *at <= t) {
                    applied = queue.pop_front().map_or(applied, |(_, s)| s);
                }
                let nose = 0.23 * gs * (1.0 - gs / 40.0).max(0.0);
                let fitted = gs.min(86.0);
                let past = (gs.max(86.0) / 86.0).powf(1.25);
                let rudder = kscale * 6.45 * (fitted / 60.0).powi(2) * past + nose;
                let weathervane = xw_kt / 7.4 * (2.48 * gs / 60.0 - 0.99).max(0.0);
                yaw += (rudder * applied - weathervane - yaw) * H / LAG_S;
                psi += yaw * H;
                xtk += gs / MS_TO_KTS * (psi - COURSE + slip_deg).to_radians().sin() * H;
                gs += accel_kts_s * H;
                t += H;
            }
        }
        ticks
    }

    fn worst_xtk(ticks: &[RollTick], upto_kts: f64) -> f64 {
        ticks
            .iter()
            .filter(|k| k.gs >= 30.0 && k.gs <= upto_kts)
            .map(|k| k.xtk.abs())
            .fold(0.0, f64::max)
    }

    /// Tick-to-tick steer jumps past 0.08: the weave's signature.
    fn steer_swings(ticks: &[RollTick]) -> usize {
        ticks
            .windows(2)
            .filter(|w| w[0].gs >= 30.0 && (w[1].steer - w[0].steer).abs() > 0.08)
            .count()
    }

    fn default_737_law() -> RollSteer {
        RollSteer {
            jet: true,
            ..RollSteer::default()
        }
    }

    /// EGKK 26L, 2026-09-27, both rolls rejected `lateral_excursion`: an
    /// 8 kt crosswind from the left, the nose 5 degrees left of the target
    /// for ten seconds on 0.28 of steer, 10.3 m left at 87 kt. The default
    /// law must still do that through the fitted plant, or this test is not
    /// about EGKK any more; the law scheduled on the 737's declared rudder
    /// holds the centreline through rotation, its trim taking the crosswind
    /// within seconds of 40 kt.
    #[test]
    fn a_steady_crosswind_is_held_on_the_centreline() {
        // First roll: 7.5 kt of crosswind, on the centreline at the start.
        let old = fly_737_roll(default_737_law(), 7.5, 1.0, 0.08, 0.19, 6.2, 0.0, 145.0);
        let old_err = old
            .iter()
            .filter(|k| k.gs <= 87.0)
            .map(|k| k.err.abs())
            .fold(0.0, f64::max);
        assert!(
            old_err > 5.0,
            "the default law held only {old_err:.1} degrees off"
        );
        assert!(
            worst_xtk(&old, 87.0) > 10.0,
            "the default law reached only {:.1} m by 87 kt",
            worst_xtk(&old, 87.0)
        );
        let new = fly_737_roll(
            RollSteer::with_rudder(true, rudder_737()),
            7.5,
            1.0,
            0.08,
            0.19,
            6.2,
            0.0,
            145.0,
        );
        assert!(
            worst_xtk(&new, 145.0) < 5.0,
            "held within 5 m through rotation: {:.1} m",
            worst_xtk(&new, 145.0)
        );
        let new_err = new.iter().map(|k| k.err.abs()).fold(0.0, f64::max);
        assert!(
            new_err < 3.0,
            "the nose stays within 3 degrees: {new_err:.1}"
        );
        let trim_60 = new.iter().find(|k| k.gs >= 60.0).map_or(0.0, |k| k.trim);
        assert!(
            trim_60 > 0.1,
            "the crosswind's steady rudder is in the trim by 60 kt: {trim_60:.2}"
        );
        assert_eq!(
            steer_swings(&new),
            0,
            "and the rudder does not weave for it"
        );

        // Second roll: 6.4 kt, starting 3.1 m left with the nose 1.5 right.
        let old = fly_737_roll(default_737_law(), 6.4, 1.0, -3.1, 1.49, 6.1, 0.0, 145.0);
        let new = fly_737_roll(
            RollSteer::with_rudder(true, rudder_737()),
            6.4,
            1.0,
            -3.1,
            1.49,
            6.1,
            0.0,
            145.0,
        );
        assert!(worst_xtk(&old, 87.0) > 8.0, "{:.1}", worst_xtk(&old, 87.0));
        assert!(
            worst_xtk(&new, 145.0) < 5.0,
            "the second roll within 5 m too: {:.1}",
            worst_xtk(&new, 145.0)
        );
    }

    /// No wind (the LIBP case the trim scaling was for): the scheduled law
    /// recovers an offset without weaving and overshoots no more than the
    /// default law, with the real rudder as the plant has it, half of it or
    /// 1.75 times it; and a crosswind with a rudder not as declared is still
    /// held in half the default law's excursion or less. (Until 2026-09-28
    /// the strong case was 2.5 times a plant growing as v² to Vr: 100 deg/s
    /// per unit at 150 kt, eight times what either day's rolls measured. On
    /// the refitted plant 1.75 is still 3.4 to 5 times the measured rudder
    /// at Vr; at 2 an engage 2 m off and 1.5 degrees crooked at 80 kt swings
    /// twice, at 2.5 the loop rings near Vr.)
    #[test]
    fn the_scheduled_roll_law_does_not_weave() {
        // The far side of the centreline from where it started.
        let overshoot = |ticks: &[RollTick], xtk0: f64| {
            ticks
                .iter()
                .map(|k| -k.xtk * xtk0.signum())
                .fold(0.0, f64::max)
        };
        for kscale in [0.5, 1.0, 1.75] {
            for (gs0, xtk0, psi_off) in [(0.0, 3.0, 1.0), (40.0, 3.0, 2.0), (80.0, -2.0, -1.5)] {
                let old = fly_737_roll(
                    default_737_law(),
                    0.0,
                    kscale,
                    xtk0,
                    psi_off,
                    6.2,
                    gs0,
                    150.0,
                );
                let new = fly_737_roll(
                    RollSteer::with_rudder(true, rudder_737()),
                    0.0,
                    kscale,
                    xtk0,
                    psi_off,
                    6.2,
                    gs0,
                    150.0,
                );
                let (o, n) = (overshoot(&old, xtk0), overshoot(&new, xtk0));
                assert!(
                    n < 2.5 && n <= o + 1.0,
                    "x{kscale} from {gs0} kt: {n:.1} m through the centreline, default {o:.1}"
                );
                assert!(
                    steer_swings(&new) <= 1,
                    "x{kscale} from {gs0} kt: the rudder swung {} times",
                    steer_swings(&new)
                );
                assert!(
                    new.last().is_some_and(|k| k.xtk.abs() < 2.5),
                    "x{kscale} from {gs0} kt: rotates near the centreline"
                );
            }
            let old = fly_737_roll(default_737_law(), 7.4, kscale, 0.08, 0.2, 6.2, 0.0, 145.0);
            let new = fly_737_roll(
                RollSteer::with_rudder(true, rudder_737()),
                7.4,
                kscale,
                0.08,
                0.2,
                6.2,
                0.0,
                145.0,
            );
            assert!(
                worst_xtk(&new, 145.0) < worst_xtk(&old, 145.0) * 0.5,
                "x{kscale} in the crosswind: {:.1} against {:.1}",
                worst_xtk(&new, 145.0),
                worst_xtk(&old, 145.0)
            );
            assert_eq!(
                steer_swings(&new),
                0,
                "x{kscale} in the crosswind: the rudder weaves"
            );
        }
    }

    /// Every recorded 737 roll in the sorties (2026-09-26 and 27), replayed
    /// through the same plant with its own mean crosswind (the `state`
    /// wind across the runway), acceleration and start: the scheduled law
    /// is never worse than the default law it replaces, and every one stays
    /// within 5 m. The plant under-reads the gustier days (LIBP, LEMG flew
    /// 7 to 10 m against the plant's 2 to 6 under the default law), so this
    /// ranks the laws; it does not reproduce those rolls metre for metre.
    #[test]
    fn recorded_737_rolls_replay_no_worse() {
        // (sortie, crosswind from the left kt, accel kt/s, xtk0 m, psi0 deg, vr kt)
        const ROLLS: [(&str, f64, f64, f64, f64, f64); 10] = [
            ("1790426786 LIBP 04", 3.6, 6.2, 0.5, 0.0, 125.0),
            ("1790429680 LIBP 04", 3.9, 6.3, 0.6, 0.0, 150.0),
            ("1790441293 LEMG 13", 1.1, 6.6, 0.0, 0.0, 150.0),
            ("1790457940 LIBP 22", -0.7, 6.0, 0.0, -0.02, 150.0),
            ("1790466443 LIEA 02", 0.1, 6.0, 0.0, -0.40, 150.0),
            ("1790494983 LIEA 20", -0.1, 5.9, -0.1, -0.87, 150.0),
            ("1790504948 LEBL 24R", 0.1, 6.1, 0.0, 0.55, 150.0),
            ("1790510400 LFBD 29", 1.4, 5.9, 0.1, 0.09, 150.0),
            ("1790524628 EGKK 26L", 7.5, 6.2, 0.1, 0.19, 145.0),
            ("1790525414 EGKK 26L", 6.4, 6.1, -3.1, 1.49, 145.0),
        ];
        for (id, xw, accel, xtk0, psi0, vr) in ROLLS {
            let old = fly_737_roll(default_737_law(), xw, 1.0, xtk0, psi0, accel, 0.0, vr);
            let new = fly_737_roll(
                RollSteer::with_rudder(true, rudder_737()),
                xw,
                1.0,
                xtk0,
                psi0,
                accel,
                0.0,
                vr,
            );
            let (o, n) = (worst_xtk(&old, vr), worst_xtk(&new, vr));
            assert!(
                n <= o + 0.3,
                "{id}: {n:.1} m against the default law's {o:.1}"
            );
            assert!(n < 5.0, "{id}: {n:.1} m");
            assert_eq!(steer_swings(&new), 0, "{id}: the rudder weaves");
        }
    }

    /// EGBB 33 (1790534902, 2026-09-27): about 3 kt of crosswind from the
    /// left, the heading held within 0.3 degrees of its target, and the
    /// track 0.6 to 0.7 degrees right of the heading from 95 kt. Through the
    /// fitted plant with that drift, the cut alone lifts off metres right
    /// (the recorded roll: 5.7 m at 161 kt); fed the drift it holds the
    /// line, with no weave bought for it.
    #[test]
    fn a_steady_drift_is_fed_forward_not_held_off() {
        let fly = |feed: bool| {
            fly_737_roll_drifting(
                RollSteer::with_rudder(true, rudder_737()),
                3.1,
                1.0,
                0.13,
                0.0,
                5.85,
                0.0,
                161.0,
                0.65,
                feed,
            )
        };
        let (old, new) = (fly(false), fly(true));
        // At Vr (145 kt IAS, 147 over the ground): the recorded roll was
        // 3.7 m right there, before rotation raised the drift to 0.9.
        let at_vr = |t: &[RollTick]| t.iter().find(|k| k.gs >= 147.0).map_or(0.0, |k| k.xtk);
        println!(
            "cut alone: {:.1} m at Vr, {:.1} at liftoff; fed: worst {:.1} m",
            at_vr(&old),
            old.last().map_or(0.0, |k| k.xtk),
            worst_xtk(&new, 161.0)
        );
        assert!(
            at_vr(&old) > 3.0,
            "the cut alone must still lose the line, or this is not EGBB: {:.1} m",
            at_vr(&old)
        );
        assert!(
            worst_xtk(&new, 161.0) < 1.5,
            "fed the drift, within 1.5 m to liftoff: {:.1} m (cut alone {:.1} at Vr)",
            worst_xtk(&new, 161.0),
            at_vr(&old)
        );
        assert!(new.iter().map(|k| k.err.abs()).fold(0.0, f64::max) < 3.0);
        assert_eq!(steer_swings(&new), 0, "and the rudder does not weave for it");
    }

    /// With no drift to read, the feed changes nothing that matters: the
    /// offset recoveries, the crosswinds and every recorded 737 roll fly
    /// within a few tenths of the law without it, and never weave.
    #[test]
    fn the_drift_feed_is_inert_without_a_drift() {
        let lift = |t: &[RollTick]| t.last().map_or(0.0, |k| k.xtk);
        for kscale in [0.5, 1.0, 2.5] {
            for (xw, gs0, xtk0, psi_off) in [
                (0.0, 0.0, 3.0, 1.0),
                (0.0, 40.0, 3.0, 2.0),
                (0.0, 80.0, -2.0, -1.5),
                (7.4, 0.0, 0.08, 0.2),
            ] {
                let run = |feed: bool| {
                    fly_737_roll_drifting(
                        RollSteer::with_rudder(true, rudder_737()),
                        xw,
                        kscale,
                        xtk0,
                        psi_off,
                        6.2,
                        gs0,
                        150.0,
                        0.0,
                        feed,
                    )
                };
                let (a, b) = (run(false), run(true));
                let (wa, wb) = (worst_xtk(&a, 150.0), worst_xtk(&b, 150.0));
                assert!(
                    wb <= wa + 0.3,
                    "x{kscale} {xw} kt from {gs0} kt: {wb:.1} m against {wa:.1}"
                );
                assert!(
                    steer_swings(&b) <= steer_swings(&a).max(1),
                    "x{kscale} {xw} kt from {gs0} kt: the rudder swung {} times",
                    steer_swings(&b)
                );
                let (la, lb) = (lift(&a).abs(), lift(&b).abs());
                assert!(
                    lb < 2.5 || lb <= la + 0.3,
                    "x{kscale} {xw} kt from {gs0} kt: rotates {lb:.1} m off against {la:.1}"
                );
            }
        }
    }

    /// The filter over the recorded EGBB roll ticks (bench/sorties): the
    /// course is the logged target plus the cut it was built from. It reads
    /// the drift the roll carried (0.6 to 0.7 degrees from 95 kt) by
    /// rotation, and the position spikes in the ticks (1.3 m in one tick at
    /// 50 kt) never throw it past a degree.
    #[test]
    fn the_drift_is_read_off_the_recorded_egbb_roll() {
        const EGBB: &str = include_str!("../bench/sorties/1790534902-full_auto.trimmed.jsonl");
        let ticks: Vec<serde_json::Value> = EGBB
            .lines()
            .filter_map(|l| serde_json::from_str::<serde_json::Value>(l).ok())
            .filter(|r| r["kind"] == "autotakeoff" && r["event"] == "roll")
            .collect();
        assert!(ticks.len() > 100, "{} roll ticks", ticks.len());
        let mut filter = DriftFilter::default();
        let (mut worst, mut at_vr) = (0.0_f64, None);
        for r in &ticks {
            let g = |k: &str| r[k].as_f64().unwrap_or(0.0);
            let (gs, xtk, rate) = (g("gs"), g("xtk_m"), g("xtk_rate"));
            let course = g("tgt") - roll_target_heading(0.0, xtk, rate, gs);
            let psi_off = ((g("psi") - course + 540.0).rem_euclid(360.0)) - 180.0;
            let d = filter.update(psi_off, rate, gs, g("dt"));
            worst = worst.max(d.abs());
            if at_vr.is_none() && g("ias") >= 145.0 {
                at_vr = Some(d);
            }
        }
        let at_vr = at_vr.expect("the roll reached Vr");
        println!("drift at Vr {at_vr:.2}, worst {worst:.2} degrees");
        assert!((0.45..=0.9).contains(&at_vr), "drift at Vr {at_vr:.2} degrees");
        assert!(worst < 1.0, "never past a degree: {worst:.2}");
    }

    /// Without a declared rudder nothing changes: the yaw rate is ignored
    /// and every other airframe flies the law it flew.
    #[test]
    fn an_undeclared_rudder_keeps_the_default_law() {
        let mut plain = default_737_law();
        let mut with_yaw = default_737_law();
        for (err, gs) in [(2.0, 30.0), (3.0, 45.0), (1.0, 70.0), (-2.0, 100.0)] {
            let a = plain.step(err, 0.27, gs, 1.0);
            let b = with_yaw.step_with_yaw(err, 0.27, gs, 1.0, Some(5.0));
            assert_eq!(a, b);
        }
        assert!(!RollSteer::default().scheduled());
        // A nonsense declaration is no declaration.
        let silly = crate::aircraft::RollRudderProfile {
            yaw_dps: 0.0,
            at_kts: 80.0,
            power_steer: None,
            power_steer_to_kts: None,
            power_steer_per_kftlb: None,
            yaw_power: None,
            close_per_s: None,
            trim_per_s: None,
        };
        assert!(!RollSteer::with_rudder(true, silly).scheduled());
    }

    /// The 737 rolls of 2026-09-28 (bench/takeoffs, every `roll` tick of
    /// EGKK 08R 1790601822 and the seven LEMG 13 circuits), one Vec per roll.
    fn rolls_0928() -> Vec<(String, Vec<serde_json::Value>)> {
        const TEXT: &str = include_str!("../bench/takeoffs/b738_rolls_2026-09-28.jsonl");
        let mut rolls: Vec<(String, Vec<serde_json::Value>)> = Vec::new();
        for r in TEXT
            .lines()
            .filter_map(|l| serde_json::from_str::<serde_json::Value>(l).ok())
        {
            let id = r["sortie"].as_str().unwrap_or("").to_string();
            match rolls.last_mut() {
                Some((last, ticks)) if *last == id => ticks.push(r),
                _ => rolls.push((id, vec![r])),
            }
        }
        rolls
    }

    /// The 737's rudder on 2026-09-28, deg/s of yaw per unit of steer: the
    /// recorded steer through 0.3 s of dead time and a 0.47 s lag, fitted
    /// per speed band to three-second heading predictions of all eight
    /// rolls with a disturbance per roll (0.29 degrees rms).
    fn rudder_0928(gs: f64) -> f64 {
        const PTS: [(f64, f64); 7] = [
            (30.0, 3.5),
            (45.0, 3.5),
            (70.0, 4.0),
            (90.0, 5.4),
            (110.0, 5.8),
            (132.0, 8.5),
            (170.0, 10.5),
        ];
        if gs <= PTS[0].0 {
            return PTS[0].1;
        }
        for w in PTS.windows(2) {
            if gs <= w[1].0 {
                let f = (gs - w[0].0) / (w[1].0 - w[0].0);
                return w[0].1 + (w[1].1 - w[0].1) * f;
            }
        }
        PTS[PTS.len() - 1].1
    }

    /// The same fit over the 737 rolls of 2026-09-27 (EGKK 26L twice, EGBB,
    /// LEMG twice) to Vr: about v^1.25, 7.2 at 80 kt.
    fn rudder_0927(gs: f64) -> f64 {
        7.2 * (gs.max(0.0) / 80.0).powf(1.25)
    }

    /// One recorded roll replayed under a law: the aircraft is the recorded
    /// one plus the response to the DIFFERENCE between the law's steer and
    /// the recorded steer, through the fitted rudder, the loop's 0.3 s of
    /// dead time and the 0.47 s lag, and the ground geometry. Everything the
    /// recorded roll met (gusts, the tyres' slip, the cross-track spikes of
    /// the position read) is met again as it came. Returns per tick
    /// (gs, cross-track the law read, the TRUE cross-track (the median of
    /// five reads, which no single-tick spike passes), steer), and the
    /// largest difference between the replayed steer and the recorded one.
    fn replay_roll(
        ticks: &[serde_json::Value],
        law: RollSteer,
        cut: RollCut,
        rudder: &dyn Fn(f64) -> f64,
    ) -> (Vec<(f64, f64, f64, f64)>, f64) {
        replay_roll_braked(ticks, law, cut, rudder, None)
    }

    /// [`replay_roll`] on an airframe that brake-steers its roll
    /// ([`roll_brake_split_aligning`]): the replayed split less the split of
    /// the recorded steer yaws the aircraft through `brake` (deg/s per unit
    /// of right brake less left), on the same dead time and lag as the
    /// rudder. `None` is the rudder alone, as every earlier replay flew.
    fn replay_roll_braked(
        ticks: &[serde_json::Value],
        mut law: RollSteer,
        mut cut: RollCut,
        rudder: &dyn Fn(f64) -> f64,
        brake: Option<&dyn Fn(f64) -> f64>,
    ) -> (Vec<(f64, f64, f64, f64)>, f64) {
        const DEAD_S: f64 = 0.3;
        const LAG_S: f64 = 0.47;
        const H: f64 = 0.01;
        let g = |r: &serde_json::Value, k: &str| r[k].as_f64().unwrap_or(0.0);
        let n = ticks.len();
        let mut ts = vec![0.0; n];
        for k in 1..n {
            ts[k] = ts[k - 1] + g(&ticks[k], "dt");
        }
        let psi: Vec<f64> = ticks.iter().map(|r| g(r, "psi")).collect();
        // The sim's yaw rate the law damped on: the recorded `r` where the
        // tick carries it (the SR22's of 2026-09-29), else the heading's own
        // rate (the 737 bench carries none; before 2026-09-28 `r` was not
        // it, see the roll tick's comment).
        let yaw_rec: Vec<f64> = (0..n)
            .map(|k| {
                if let Some(r) = ticks[k]["r"].as_f64().filter(|r| r.is_finite()) {
                    return r;
                }
                let (a, b) = (k.saturating_sub(1), (k + 1).min(n - 1));
                let d = ((psi[b] - psi[a] + 540.0).rem_euclid(360.0)) - 180.0;
                if ts[b] > ts[a] {
                    d / (ts[b] - ts[a])
                } else {
                    0.0
                }
            })
            .collect();
        let xtk_rec: Vec<f64> = ticks.iter().map(|r| g(r, "xtk_m")).collect();
        let truth: Vec<f64> = (0..n)
            .map(|k| {
                let mut w: Vec<f64> = xtk_rec[k.saturating_sub(2)..(k + 3).min(n)].to_vec();
                w.sort_by(|a, b| a.total_cmp(b));
                w[w.len() / 2]
            })
            .collect();
        // The runway course, from a tick of the law as flown.
        let r5 = &ticks[5];
        let course = g(r5, "tgt")
            + g(r5, "drift")
            + roll_cut_deg(g(r5, "xtk_m"), g(r5, "xtk_rate"), g(r5, "gs"));
        let (mut dyaw, mut dpsi, mut dx) = (0.0_f64, 0.0_f64, 0.0_f64);
        let mut queue: std::collections::VecDeque<(f64, f64, f64)> = Default::default();
        let mut applied = 0.0_f64;
        let mut applied_brake = 0.0_f64;
        let mut t = 0.0_f64;
        let mut out = Vec::with_capacity(n);
        let mut dev = 0.0_f64;
        for k in 0..n {
            let r = &ticks[k];
            let dt = if k == 0 { 0.2 } else { g(r, "dt") };
            if k > 0 {
                let (gs0, gs1) = (g(&ticks[k - 1], "gs"), g(r, "gs"));
                let end = ts[k];
                while t < end - 1e-9 {
                    while queue.front().is_some_and(|(at, _, _)| *at <= t) {
                        if let Some((_, s, b)) = queue.pop_front() {
                            applied = s;
                            applied_brake = b;
                        }
                    }
                    let gs = gs0 + (gs1 - gs0) * (1.0 - (end - t) / dt);
                    let braked = brake.map_or(0.0, |b| b(gs) * applied_brake);
                    dyaw += (rudder(gs) * applied + braked - dyaw) * H / LAG_S;
                    dpsi += dyaw * H;
                    dx += gs / MS_TO_KTS * dpsi.to_radians() * H;
                    t += H;
                }
                t = end;
            }
            let gs = g(r, "gs");
            let psi_off = ((psi[k] + dpsi - course + 540.0).rem_euclid(360.0)) - 180.0;
            let c = cut.update(xtk_rec[k] + dx, psi_off, gs, dt);
            let err = c.target_off_deg - psi_off;
            // The throttle as recorded (a propeller's power lead rides it);
            // the 737's ticks carry none and a jet takes no lead.
            let thr = r["thr"].as_f64().unwrap_or(1.0);
            let yaw = yaw_rec[k] + dyaw;
            // The torque as recorded, for a lead declared on it (the
            // Evolution's); a law without one ignores it.
            law.set_torque(r["trq_ftlb"].as_f64());
            let steer = law.step_with_yaw(err, dt, gs, thr, Some(yaw));
            dev = dev.max((steer - g(r, "steer")).abs());
            let dbrake = if brake.is_some() {
                let (bl, br) = roll_brake_split_aligning(steer, gs, err, yaw);
                let (bl0, br0) =
                    roll_brake_split_aligning(g(r, "steer"), gs, g(r, "err"), yaw_rec[k]);
                (br - bl) - (br0 - bl0)
            } else {
                0.0
            };
            queue.push_back((t + DEAD_S, steer - g(r, "steer"), dbrake));
            out.push((gs, c.xtk_m, truth[k] + dx, steer));
        }
        (out, dev)
    }

    /// Worst true offset from 30 kt, the true offset at liftoff (the last
    /// roll tick), and tick-to-tick steer jumps past 0.08 (the weave).
    fn roll_verdict(out: &[(f64, f64, f64, f64)]) -> (f64, f64, usize) {
        let worst = out
            .iter()
            .filter(|o| o.0 >= 30.0)
            .map(|o| o.2.abs())
            .fold(0.0, f64::max);
        let lift = out.last().map_or(0.0, |o| o.2);
        let swings = out
            .windows(2)
            .filter(|w| w[0].0 >= 30.0 && (w[1].3 - w[0].3).abs() > 0.08)
            .count();
        (worst, lift, swings)
    }

    /// The replay is the flight: the law as flown on 2026-09-28, replayed
    /// through it, writes the recorded steer tick for tick (so the replay
    /// carries everything the aircraft met), and the recorded offsets are
    /// the failures being fixed: EGKK 08R lifted off 5.4 m left (5.8 m worst,
    /// the takeoff FAIL), LEMG 1790591256 9.0 m worst and 5.9 at liftoff.
    #[test]
    fn the_recorded_737_rolls_replay_as_flown() {
        let rolls = rolls_0928();
        assert_eq!(rolls.len(), 8, "eight rolls on the bench");
        let mut failed = 0;
        for (id, ticks) in &rolls {
            let law = RollSteer::with_rudder(true, rudder_737()).as_flown();
            let (_, dev) = replay_roll(
                ticks,
                law,
                RollCut::for_law(&RollSteer::with_rudder(true, rudder_737())).as_flown(),
                &rudder_0928,
            );
            assert!(
                dev < 0.01,
                "{id}: the replayed law strays {dev:.4} from the recorded steer"
            );
            let xs: Vec<f64> = ticks
                .iter()
                .filter(|r| r["gs"].as_f64().unwrap_or(0.0) >= 30.0)
                .map(|r| r["xtk_m"].as_f64().unwrap_or(0.0))
                .collect();
            let worst = xs.iter().map(|x| x.abs()).fold(0.0, f64::max);
            let lift = xs.last().copied().unwrap_or(0.0);
            if worst > 3.0 || lift.abs() > 2.0 {
                failed += 1;
            }
            if id.starts_with("1790601822") {
                assert!(
                    (5.7..=5.9).contains(&worst) && (-5.5..=-5.3).contains(&lift),
                    "{worst} {lift}"
                );
            }
        }
        assert!(
            failed >= 4,
            "the rolls as flown: {failed} outside 3 m worst or 2 m at liftoff"
        );
    }

    /// The fix, replayed through every recorded roll of 2026-09-28: the
    /// heading loop scheduled on the declared rudder only up to the speed it
    /// was measured at, the cut held harder from 40 kt with a cross-track
    /// trim, and the cross-track read through a median of three. On the
    /// rudder fitted to these rolls and to the 27th's, each lifts off within
    /// 2 m of the centreline and never strays 3 m (the scorer's roll limit
    /// is 5), with no weave; on a rudder 0.6 or 3 times that day's it stays
    /// within 3.5 m and does not weave. And no roll is worse than it flew.
    #[test]
    fn the_737_roll_holds_the_centreline_through_liftoff() {
        let rolls = rolls_0928();
        let fly = |ticks: &[serde_json::Value], rudder: &dyn Fn(f64) -> f64| {
            let law = RollSteer::with_rudder(true, rudder_737());
            roll_verdict(
                &replay_roll(
                    ticks,
                    law,
                    RollCut::for_law(&RollSteer::with_rudder(true, rudder_737())),
                    rudder,
                )
                .0,
            )
        };
        for (id, ticks) in &rolls {
            let flown = {
                let law = RollSteer::with_rudder(true, rudder_737()).as_flown();
                roll_verdict(
                    &replay_roll(
                        ticks,
                        law,
                        RollCut::for_law(&RollSteer::with_rudder(true, rudder_737())).as_flown(),
                        &rudder_0928,
                    )
                    .0,
                )
            };
            for (name, rudder) in [
                ("09-28", &rudder_0928 as &dyn Fn(f64) -> f64),
                ("09-27", &rudder_0927),
            ] {
                let (worst, lift, swings) = fly(ticks, rudder);
                println!(
                    "{id} {name}: worst {worst:.1} m, liftoff {lift:+.1} m (flown {:.1}, {:+.1})",
                    flown.0, flown.1
                );
                assert!(
                    worst <= 3.0,
                    "{id} on the {name} rudder: {worst:.1} m worst"
                );
                assert!(
                    lift.abs() <= 2.0,
                    "{id} on the {name} rudder: {lift:+.1} m at liftoff"
                );
                assert_eq!(swings, 0, "{id} on the {name} rudder: the steer weaves");
                assert!(
                    worst <= flown.0 + 0.3,
                    "{id}: {worst:.1} m against {:.1} flown",
                    flown.0
                );
            }
            for scale in [0.6, 3.0] {
                let rudder = move |gs: f64| scale * rudder_0928(gs);
                let (worst, lift, swings) = fly(ticks, &rudder);
                assert!(
                    worst <= 3.5 && lift.abs() <= 2.5,
                    "{id} on x{scale} the rudder: {worst:.1} m worst, {lift:+.1} at liftoff"
                );
                assert_eq!(swings, 0, "{id} on x{scale} the rudder: the steer weaves");
            }
        }
    }

    /// The SR22's LEMG 13 roll of 2026-09-29 (sortie 1790677606-full_auto),
    /// every `roll` tick from brake release to the reject.
    fn sr22_roll_0929() -> Vec<serde_json::Value> {
        const TEXT: &str = include_str!("../bench/takeoffs/sr22_roll_lemg13_2026-09-29.jsonl");
        TEXT.lines()
            .filter_map(|l| serde_json::from_str::<serde_json::Value>(l).ok())
            .collect()
    }

    /// The SR22's rudder fitted to that roll, deg/s of yaw per unit of steer:
    /// the sim's yaw rate `r` against the steer through the same 0.3 s of
    /// dead time and 0.47 s lag, over 15 kt to the reject (73 kt), with a
    /// standing rudder of its own (the propeller): 31 deg/s at 60 kt growing
    /// as v², 0.35 to 0.37 of steer standing from 30 kt (0.39 deg/s rms;
    /// v^1.5 fits 0.40).
    fn rudder_sr22_0929(gs: f64) -> f64 {
        31.0 * (gs.max(0.0) / 60.0).powi(2)
    }

    /// The SR22's rudder as the profile declares it (`roll_rudder:`).
    fn rudder_sr22() -> crate::aircraft::RollRudderProfile {
        let text = std::fs::read_to_string("profiles/cirrus_sr22.yaml").unwrap();
        let p: crate::aircraft::AircraftProfile = serde_yaml::from_str(&text).unwrap();
        p.roll_rudder
            .and_then(|r| r.valid())
            .expect("the SR22 declares its rudder")
    }

    /// The SR22's roll as flown: the default law replayed through the
    /// recorded roll writes the recorded steer tick for tick, and the offset
    /// is the failure being fixed (9.8 m left at the reject, 66 kt).
    #[test]
    fn the_sr22_roll_replays_as_flown() {
        let ticks = sr22_roll_0929();
        assert_eq!(ticks.len(), 63);
        let (out, dev) = replay_roll(
            &ticks,
            RollSteer::default(),
            RollCut::new(None),
            &rudder_sr22_0929,
        );
        assert!(
            dev < 0.01,
            "the replayed law strays {dev:.4} from the recorded steer"
        );
        let (worst, end, _) = roll_verdict(&out);
        assert!(worst > 9.0 && end < -9.0, "{worst:.1} {end:+.1}");
    }

    /// The fix: the SR22 declares its rudder and its propeller's standing
    /// rudder (`roll_rudder:` `power_steer`), led in with the throttle for
    /// the whole roll. Replayed through the recorded roll on the fitted
    /// rudder at half to three times, and on an aircraft needing 0.08 of
    /// steer less or more than it did that day, it never strays 3 m and
    /// does not weave; on the fitted rudder and the day's need, 1.5 m. The
    /// declared rudder alone (the generic power lead, on the closure and trim
    /// of that day) still strays over 4 m: the propeller's standing rudder is
    /// the part the default law lacked.
    #[test]
    fn the_sr22_roll_holds_the_centreline_on_its_propeller() {
        let ticks = sr22_roll_0929();
        let declared = rudder_sr22();
        assert_eq!(declared.power_steer, Some(0.35));
        let fly =
            |ticks: &[serde_json::Value], rd: crate::aircraft::RollRudderProfile, scale: f64| {
                let law = RollSteer::with_rudder(false, rd);
                let cut = RollCut::for_law(&law);
                let plant = move |gs: f64| scale * rudder_sr22_0929(gs);
                roll_verdict(&replay_roll(ticks, law, cut, &plant).0)
            };
        for need in [-0.08, 0.0, 0.08] {
            // The recorded steer relabelled: the aircraft needed `need` more.
            let shifted: Vec<serde_json::Value> = ticks
                .iter()
                .map(|r| {
                    let mut r = r.clone();
                    r["steer"] = serde_json::json!(r["steer"].as_f64().unwrap_or(0.0) - need);
                    r
                })
                .collect();
            for scale in [0.5, 1.0, 2.0, 3.0] {
                let (worst, end, swings) = fly(&shifted, declared, scale);
                println!("need {need:+} x{scale}: worst {worst:.2} m, end {end:+.2} m");
                assert!(worst <= 3.0, "need {need:+}, rudder x{scale}: {worst:.1} m");
                assert_eq!(
                    swings, 0,
                    "need {need:+}, rudder x{scale}: the steer weaves"
                );
            }
        }
        let (worst, _, _) = fly(&ticks, declared, 1.0);
        assert!(worst <= 1.5, "{worst:.2} m on the fitted rudder");
        // Against the declared rudder's law as it stood that day (the 737's
        // closure and trim): the faster closure since holds even the generic
        // lead's roll nearer.
        let generic = crate::aircraft::RollRudderProfile {
            power_steer: None,
            ..rudder_sr22_as_flown()
        };
        let (worst, _, _) = fly(&ticks, generic, 1.0);
        assert!(worst > 4.0, "{worst:.2} m without the propeller's rudder");
    }

    /// The SR22's second LEMG 13 roll of 2026-09-29 (sortie
    /// 1790682305-full_auto), flown on the declared rudder: every `roll`
    /// tick from brake release to wheels-off. The wheels left between the
    /// state records at 1790682400.9 (on the ground, 0.2 ft) and .401.9
    /// (0.5 ft); the ticks after that are the ~1.6 s the roll law kept
    /// steering in ground effect, and a replay of the tyres is no replay of
    /// them.
    fn sr22_rudder_roll_0929() -> Vec<serde_json::Value> {
        const TEXT: &str =
            include_str!("../bench/takeoffs/sr22_roll_lemg13_rudder_2026-09-29.jsonl");
        TEXT.lines()
            .filter_map(|l| serde_json::from_str::<serde_json::Value>(l).ok())
            .filter(|r| r["t"].as_f64().is_some_and(|t| t <= SR22_RUDDER_WHEELS_OFF_T))
            .collect()
    }
    const SR22_RUDDER_WHEELS_OFF_T: f64 = 1790682401.0;

    /// The SR22's rudder refitted over BOTH rolls of 2026-09-29 (three-second
    /// heading predictions from the sim's `r`, 0.3 s of dead time, a standing
    /// need per roll and speed): 50 deg/s per unit at 60 kt growing as v
    /// (0.08 and 0.23 degrees rms against 0.11 and 0.28 for the declared 31
    /// as v^2; a v^2 rudder strong enough for the fast end misses the slow).
    fn rudder_sr22_0929_linear(gs: f64) -> f64 {
        50.0 * gs.max(0.0) / 60.0
    }

    /// The SR22's rudder as the profile declared it on 2026-09-29, the second
    /// roll's law: the standing rudder held to Vr, the 737's closure and trim.
    fn rudder_sr22_as_flown() -> crate::aircraft::RollRudderProfile {
        crate::aircraft::RollRudderProfile {
            power_steer_to_kts: Some(250.0),
            close_per_s: None,
            trim_per_s: None,
            ..rudder_sr22()
        }
    }

    /// The recorded steer relabelled: the aircraft needed `need` more.
    fn needing(ticks: &[serde_json::Value], need: f64) -> Vec<serde_json::Value> {
        ticks
            .iter()
            .map(|r| {
                let mut r = r.clone();
                r["steer"] = serde_json::json!(r["steer"].as_f64().unwrap_or(0.0) - need);
                r
            })
            .collect()
    }

    /// Worst true offset from 30 kt up to `to_kts` of groundspeed.
    fn worst_to(out: &[(f64, f64, f64, f64)], to_kts: f64) -> f64 {
        out.iter()
            .filter(|o| o.0 >= 30.0 && o.0 <= to_kts)
            .map(|o| o.2.abs())
            .fold(0.0, f64::max)
    }

    /// Rotation on the second roll: 73 kt indicated at 77.7 kt over the ground.
    const SR22_RUDDER_ROTATE_GS: f64 = 78.0;

    /// The second roll as flown: the declared rudder's law of 2026-09-29,
    /// replayed through it, writes the recorded steer tick for tick, and the
    /// offsets are the failure being fixed: 4.1 m left by rotation (the
    /// recorded 4.1 at 68 kt), then the swing right to 6.5 m (the median of
    /// the reads; 7.5 raw) at wheels-off, 93 kt.
    #[test]
    fn the_sr22_declared_roll_replays_as_flown() {
        let ticks = sr22_rudder_roll_0929();
        assert_eq!(ticks.len(), 84);
        let law = RollSteer::with_rudder(false, rudder_sr22_as_flown());
        let cut = RollCut::for_law(&law);
        let (out, dev) = replay_roll(&ticks, law, cut, &rudder_sr22_0929);
        assert!(
            dev < 0.01,
            "the replayed law strays {dev:.4} from the recorded steer"
        );
        let (worst, end, _) = roll_verdict(&out);
        let to_rotate = worst_to(&out, SR22_RUDDER_ROTATE_GS);
        println!(
            "as flown: {to_rotate:.2} m to rotation, {worst:.2} m to wheels-off, {end:+.2} m there"
        );
        assert!((3.9..=4.3).contains(&to_rotate), "{to_rotate:.2}");
        assert!(worst > 6.0 && end > 6.0, "{worst:.2} {end:+.2}");
    }

    /// The fix, replayed through both SR22 rolls of 2026-09-29 to wheels-off:
    /// the propeller's standing rudder believed only to the 73 kt it was
    /// measured at and falling as v^2 past it, the heading closed at 1.5 per
    /// second with the trim's corner at 0.45 (`roll_rudder:`
    /// `power_steer_to_kts`, `close_per_s`, `trim_per_s`). On either fit of the
    /// rudder at half to twice (the declared 31 as v^2, the refit 50 as v, to
    /// 1.6 times), and on an aircraft needing 0.08 of steer less or more than
    /// it did either day, no roll strays 3 m; on the fits and the day's need
    /// the second roll holds 2.5 m to wheels-off where it flew 6.5, the first
    /// 1.5 m. The standing rudder believed to Vr, or the 737's closure, still
    /// strays past 3.5 m on the second roll.
    #[test]
    fn the_sr22_roll_holds_the_centreline_past_its_measured_speed() {
        let declared = rudder_sr22();
        assert_eq!(declared.power_steer_to_kts, Some(73.0));
        let fly = |ticks: &[serde_json::Value],
                   rd: crate::aircraft::RollRudderProfile,
                   plant: &dyn Fn(f64) -> f64| {
            let law = RollSteer::with_rudder(false, rd);
            let cut = RollCut::for_law(&law);
            replay_roll(ticks, law, cut, plant).0
        };
        type Plant = Box<dyn Fn(f64) -> f64>;
        let plants: Vec<(String, Plant)> = [0.5, 1.0, 2.0]
            .into_iter()
            .map(|k| {
                let p: Plant = Box::new(move |gs: f64| k * rudder_sr22_0929(gs));
                (format!("31 v^2 x{k}"), p)
            })
            .chain([0.5, 1.0, 1.6].into_iter().map(|k| {
                let p: Plant = Box::new(move |gs: f64| k * rudder_sr22_0929_linear(gs));
                (format!("50 v x{k}"), p)
            }))
            .collect();
        for (name, ticks) in [("first", sr22_roll_0929()), ("second", sr22_rudder_roll_0929())] {
            for need in [-0.08, 0.0, 0.08] {
                let shifted = needing(&ticks, need);
                for (plant_name, plant) in &plants {
                    let out = fly(&shifted, declared, plant.as_ref());
                    let (worst, end, swings) = roll_verdict(&out);
                    println!(
                        "{name} need {need:+} {plant_name}: worst {worst:.2} m, {end:+.2} m at the end, {swings} swings"
                    );
                    assert!(
                        worst <= 3.0,
                        "{name}, need {need:+}, {plant_name}: {worst:.2} m"
                    );
                    assert!(
                        swings <= 1,
                        "{name}, need {need:+}, {plant_name}: the steer weaves"
                    );
                }
            }
        }
        let (worst, _, _) = roll_verdict(&fly(&sr22_roll_0929(), declared, &rudder_sr22_0929));
        assert!(worst <= 1.5, "the first roll: {worst:.2} m");
        for plant in [
            &rudder_sr22_0929 as &dyn Fn(f64) -> f64,
            &rudder_sr22_0929_linear,
        ] {
            let out = fly(&sr22_rudder_roll_0929(), declared, plant);
            let (worst, _, swings) = roll_verdict(&out);
            let to_rotate = worst_to(&out, SR22_RUDDER_ROTATE_GS);
            println!("the second roll: {to_rotate:.2} m to rotation, {worst:.2} m to wheels-off");
            assert!(worst <= 2.5, "the second roll: {worst:.2} m");
            assert!(to_rotate <= 2.5, "the second roll to rotation: {to_rotate:.2} m");
            assert_eq!(swings, 0);
        }
        for (why, rd) in [
            (
                "believed to Vr",
                crate::aircraft::RollRudderProfile {
                    power_steer_to_kts: Some(250.0),
                    ..declared
                },
            ),
            (
                "the 737's closure",
                crate::aircraft::RollRudderProfile {
                    close_per_s: None,
                    trim_per_s: None,
                    ..declared
                },
            ),
        ] {
            let (worst, _, _) =
                roll_verdict(&fly(&sr22_rudder_roll_0929(), rd, &rudder_sr22_0929));
            println!("{why}: {worst:.2} m");
            assert!(worst > 3.5, "{why}: {worst:.2} m");
        }
    }

    /// The Evolution's LILB 03 rolls, every `roll` tick from brake release
    /// to the reject, each with the law it flew:
    /// - 1790712974 (2026-09-29): started 24.6 degrees off before the
    ///   aligning pivot was fixed; the default law; 9 m left at 49 kt.
    /// - 1790724857 (2026-09-30): aligned at 1.9 degrees; the default law;
    ///   the torque ran to 1,735 ft-lb; 10 m left at 60 kt.
    /// - 1790736651 (2026-09-30, b6c24d5): the declared rudder of eb511f4 and
    ///   the torque limiter holding 1,093 to 1,513 ft-lb with the lever
    ///   cycling 0.55 to 0.76; from 50 kt the nose sat 4.4 to 7.9 degrees
    ///   left with the steer at 0.63 to 0.71; 9.5 m left at 78 kt.
    fn evolution_rolls() -> Vec<(&'static str, Vec<serde_json::Value>)> {
        let ld = |t: &str| -> Vec<serde_json::Value> {
            t.lines()
                .filter_map(|l| serde_json::from_str::<serde_json::Value>(l).ok())
                .collect()
        };
        vec![
            (
                "1790712974",
                ld(include_str!("../bench/takeoffs/evolution_roll_lilb03_1790712974.jsonl")),
            ),
            (
                "1790724857",
                ld(include_str!("../bench/takeoffs/evolution_roll_lilb03_1790724857.jsonl")),
            ),
            (
                "1790736651",
                ld(include_str!("../bench/takeoffs/evolution_roll_lilb03_1790736651.jsonl")),
            ),
        ]
    }

    /// The Evolution's rudder refitted over all three rolls (three-second
    /// heading predictions from the sim's `r` through 0.3 s of dead time and
    /// the 0.47 s lag, from 12 kt, calm air, the brake split an input):
    /// 30 deg/s per unit at 60 kt as v^0.35 (1.47 degrees rms), or 33 as
    /// v^0.5 (1.52; 45 as v fits 2.19, 59 as v^2 3.72), the brake 12 deg/s
    /// per unit of split, and the standing need 0.51 of steer per 1,000
    /// ft-lb of torque (a constant need fits 3.54).
    fn rudder_evolution_flat(gs: f64) -> f64 {
        30.0 * (gs.max(0.0) / 60.0).powf(0.35)
    }
    fn rudder_evolution_sqrt(gs: f64) -> f64 {
        33.0 * (gs.max(0.0) / 60.0).sqrt()
    }
    const BRAKE_EVOLUTION: f64 = 12.0;

    /// The Evolution's rudder as the profile declares it (`roll_rudder:`).
    fn rudder_evolution() -> crate::aircraft::RollRudderProfile {
        let text = std::fs::read_to_string("profiles/lancair_evolution.yaml").unwrap();
        let p: crate::aircraft::AircraftProfile = serde_yaml::from_str(&text).unwrap();
        p.roll_rudder
            .and_then(|r| r.valid())
            .expect("the Evolution declares its rudder")
    }

    /// The Evolution's rudder as declared by eb511f4 (1790736651's law).
    fn rudder_evolution_eb511f4() -> crate::aircraft::RollRudderProfile {
        crate::aircraft::RollRudderProfile {
            yaw_dps: 30.0,
            at_kts: 60.0,
            yaw_power: None,
            power_steer: Some(0.6),
            power_steer_to_kts: Some(60.0),
            power_steer_per_kftlb: None,
            close_per_s: Some(1.0),
            trim_per_s: Some(0.3),
        }
    }

    /// Reversals of the steer from 30 kt: a jump past 0.08 against the last
    /// such jump. A steer walking up to its cap is not a weave.
    fn steer_reversals(out: &[(f64, f64, f64, f64)]) -> usize {
        let mut last = 0.0_f64;
        let mut n = 0;
        for w in out.windows(2).filter(|w| w[0].0 >= 30.0) {
            let d = w[1].3 - w[0].3;
            if d.abs() > 0.08 {
                if d * last < 0.0 {
                    n += 1;
                }
                last = d;
            }
        }
        n
    }

    /// One Evolution roll replayed under a law on a fitted rudder (scaled by
    /// `scale`) and brake.
    fn fly_evolution(
        ticks: &[serde_json::Value],
        law: RollSteer,
        cut: RollCut,
        rudder: fn(f64) -> f64,
        scale: f64,
    ) -> (Vec<(f64, f64, f64, f64)>, f64) {
        let plant = move |gs: f64| scale * rudder(gs);
        let braked = |_gs: f64| BRAKE_EVOLUTION;
        replay_roll_braked(ticks, law, cut, &plant, Some(&braked))
    }

    /// The law each roll flew.
    fn evolution_law_as_flown(name: &str) -> (RollSteer, RollCut) {
        if name == "1790736651" {
            let law = RollSteer::with_rudder(false, rudder_evolution_eb511f4());
            let cut = RollCut::for_law(&law);
            (law, cut)
        } else {
            (RollSteer::default(), RollCut::new(None))
        }
    }

    /// The Evolution's rolls as flown: each law replayed through its roll
    /// writes the recorded steer (within 0.02: the first tick's slew is
    /// replayed on the harness's 0.2 s, not the logged 0.15), and the
    /// offsets are the failures being fixed, 8.0 to 8.6 m left (the median
    /// of five reads). On the default law the steer at the reject was the
    /// authority floor, not the cap; on eb511f4's law it was 0.65 with the
    /// nose 7.6 degrees left, the lead on the LEVER faded past 60 kt.
    #[test]
    fn the_evolution_roll_replays_as_flown() {
        let lens = [32, 29, 38];
        for ((name, ticks), len) in evolution_rolls().into_iter().zip(lens) {
            assert_eq!(ticks.len(), len, "{name}");
            let (law, cut) = evolution_law_as_flown(name);
            let (out, dev) = fly_evolution(&ticks, law, cut, rudder_evolution_flat, 1.0);
            assert!(dev < 0.025, "{name}: the replayed law strays {dev:.4}");
            let (worst, end, _) = roll_verdict(&out);
            println!(
                "{name} as flown: {worst:.2} m, {end:+.2} m at the reject, {} reversals",
                steer_reversals(&out)
            );
            assert!(worst > 7.5 && end < -7.5, "{name}: {worst:.2} {end:+.2}");
            let last = ticks.last().unwrap();
            let steer = last["steer"].as_f64().unwrap();
            assert!(last["err"].as_f64().unwrap() > 7.0 && steer < 0.7, "{name}: {steer}");
        }
    }

    /// The torque lead: on the torque read, 0.51 per 1,000 ft-lb believed to
    /// 80 kt, where the lever lead gave 0.21 at 78 kt against 0.65 needed.
    #[test]
    fn the_evolution_leads_its_rudder_on_the_torque() {
        let declared = rudder_evolution();
        assert_eq!(declared.power_steer_per_kftlb, Some(0.51));
        let lead = |tq: Option<f64>, thr: f64, gs: f64| {
            let mut law = RollSteer::with_rudder(false, declared);
            law.set_torque(tq);
            // No error, no yaw: the first tick's want is the lead, slewed.
            let mut out = 0.0;
            for _ in 0..20 {
                out = law.step_with_yaw(0.0, 0.25, gs, thr, Some(0.0));
            }
            out
        };
        // 1,250 ft-lb at 78 kt: 0.64 whatever the lever.
        let a = lead(Some(1250.0), 0.55, 78.0);
        let b = lead(Some(1250.0), 0.76, 78.0);
        assert!((a - b).abs() < 1e-9, "{a} {b}");
        assert!((0.62..=0.66).contains(&a), "{a}");
        // A failed torque read leads on the lever, as power_steer does.
        let c = lead(None, 0.55, 50.0);
        assert!((c - 0.6 * 0.55).abs() < 0.02, "{c}");
        // Other airframes: no torque field, the torque is ignored.
        let mut sr22 = RollSteer::with_rudder(false, rudder_sr22());
        let mut sr22b = RollSteer::with_rudder(false, rudder_sr22());
        sr22.set_torque(Some(1500.0));
        for (err, gs) in [(2.0, 20.0), (1.0, 50.0), (-1.0, 80.0)] {
            assert_eq!(
                sr22.step_with_yaw(err, 0.25, gs, 1.0, Some(0.3)),
                sr22b.step_with_yaw(err, 0.25, gs, 1.0, Some(0.3))
            );
        }
        // The declared rudder grows as v^0.5, not v^2.
        let y = |gs| declared.yaw_dps_at(gs);
        assert!((y(20.0) / y(80.0) - 0.5).abs() < 1e-9);
    }

    /// The refit, replayed through all three rolls to the reject: the rudder
    /// declared at 32 deg/s at 80 kt growing as v^0.5 (`yaw_power`), the
    /// PT6's standing rudder led on the measured torque (0.51 per 1,000
    /// ft-lb, `power_steer_per_kftlb`) believed to 80 kt, closed at 1.5 per
    /// second with the trim's corner at 0.1 (the torque lead carries what
    /// the trim learnt slowly). On either fit, the rudder at two thirds to
    /// one and a half times (the three-roll fit bounds it tighter than the
    /// SR22's one roll did) and 0.05 of need either way:
    /// - 1790736651: 3.0 m on the fits (1.9 to 3.9 m on the neighbours),
    ///   ending within 0.9 m of the line, where it flew 8.6;
    /// - 1790724857: 3.7 to 3.8 m (4.8), where it flew 8.0, at torques the
    ///   limiter now holds off;
    /// - 1790712974: 3.6 to 3.7 m (4.9), already 2.7 m left by 30 kt from
    ///   its crooked start, where it flew 8.4.
    /// The steer reverses at most three times. eb511f4's law (the lever
    /// lead) replays 1790736651 as flown, and the torque lead on its v^2
    /// schedule and closure strays further on the fits.
    #[test]
    fn the_evolution_roll_holds_the_line_on_its_torque() {
        let declared = rudder_evolution();
        assert_eq!(declared.yaw_power, Some(0.5));
        assert_eq!(declared.power_steer_to_kts, Some(80.0));
        let fits: [(&str, fn(f64) -> f64); 2] = [
            ("30 v^0.35", rudder_evolution_flat),
            ("33 v^0.5", rudder_evolution_sqrt),
        ];
        let fly = |ticks: &[serde_json::Value],
                   rd: crate::aircraft::RollRudderProfile,
                   rudder: fn(f64) -> f64,
                   scale: f64| {
            let law = RollSteer::with_rudder(false, rd);
            let cut = RollCut::for_law(&law);
            fly_evolution(ticks, law, cut, rudder, scale).0
        };
        for ((name, ticks), (fit_max, all_max)) in
            evolution_rolls().into_iter().zip([(3.8, 5.0), (3.9, 4.9), (3.1, 4.0)])
        {
            let (law, cut) = evolution_law_as_flown(name);
            let flown = roll_verdict(&fly_evolution(&ticks, law, cut, rudder_evolution_flat, 1.0).0);
            for need in [-0.05, 0.0, 0.05] {
                let shifted = needing(&ticks, need);
                for (plant_name, rudder) in fits {
                    for scale in [0.67, 1.0, 1.5] {
                        let out = fly(&shifted, declared, rudder, scale);
                        let (worst, end, _) = roll_verdict(&out);
                        let rev = steer_reversals(&out);
                        println!(
                            "{name} need {need:+} {plant_name} x{scale}: worst {worst:.2} m, {end:+.2} m at the end, {rev} reversals"
                        );
                        assert!(
                            worst <= all_max,
                            "{name}, need {need:+}, {plant_name} x{scale}: {worst:.2} m"
                        );
                        assert!(
                            worst < flown.0 - 3.0,
                            "{name}: {worst:.2} m against {:.2} flown",
                            flown.0
                        );
                        assert!(rev <= 3, "{name}, {plant_name} x{scale}: the steer weaves");
                        if need == 0.0 && scale == 1.0 {
                            assert!(worst <= fit_max, "{name} on {plant_name}: {worst:.2} m");
                        }
                    }
                }
            }
        }
        // The torque lead alone, on eb511f4's schedule and closure.
        let torque_only = crate::aircraft::RollRudderProfile {
            power_steer_per_kftlb: Some(0.51),
            power_steer_to_kts: Some(80.0),
            ..rudder_evolution_eb511f4()
        };
        let today = &evolution_rolls()[2].1;
        let worst = |rd| {
            fits.iter()
                .map(|(_, r)| roll_verdict(&fly(today, rd, *r, 1.0)).0)
                .fold(0.0, f64::max)
        };
        let (with, without) = (worst(declared), worst(torque_only));
        println!("1790736651: {with:.2} m declared, {without:.2} m on eb511f4's schedule");
        assert!(without > with + 0.5, "{without:.2} against {with:.2}");
    }

    /// The cross-track read pairs two frames now and then (latitude and
    /// longitude are separate requests): on LEMG 13 up to 2.9 m for one tick
    /// above 120 kt. The median of three never passes a single-tick spike,
    /// follows a real movement a tick late, and the default law reads the
    /// raw value as it always has.
    #[test]
    fn a_single_tick_position_spike_never_reaches_the_cut() {
        let mut cut = RollCut::for_law(&RollSteer::with_rudder(true, rudder_737()));
        let mut plain = RollCut::new(None);
        let mut seen = Vec::new();
        for (k, x) in [0.5, 0.52, 0.54, 3.4, 0.58, 0.60, 0.62]
            .into_iter()
            .enumerate()
        {
            let c = cut.update(x, 0.0, 120.0, 0.22);
            let p = plain.update(x, 0.0, 120.0, 0.22);
            if k == 3 {
                assert_eq!(p.xtk_m, 3.4, "the default law reads the raw value");
            }
            seen.push(c.xtk_m);
        }
        assert!(
            seen.iter().all(|x| *x < 0.7),
            "a spike reached the cut: {seen:?}"
        );
        // A real step is followed, one tick late.
        let mut cut = RollCut::for_law(&RollSteer::with_rudder(true, rudder_737()));
        let steps: Vec<f64> = [0.0, 0.0, 0.0, 2.0, 2.0, 2.0]
            .into_iter()
            .map(|x| cut.update(x, 0.0, 120.0, 0.22).xtk_m)
            .collect();
        assert_eq!(steps[3], 0.0);
        assert_eq!(steps[4], 2.0);
    }

    /// Below 40 kt the scheduled roll's cut is the one it flew on what it
    /// reads: no trim, no extra gain; the heading law is unchanged below
    /// 40 kt, and past the declared speed its gain holds.
    #[test]
    fn the_slow_roll_is_the_one_it_was() {
        let mut new = RollCut::for_law(&RollSteer::with_rudder(true, rudder_737()));
        for (x, gs) in [
            (0.3, 5.0),
            (0.6, 12.0),
            (1.0, 20.0),
            (1.4, 30.0),
            (1.6, 39.0),
        ] {
            for _ in 0..3 {
                let c = new.update(x, 0.4, gs, 0.25);
                assert_eq!(
                    c.cut_deg,
                    roll_cut_deg(c.xtk_m, c.rate_m_s, gs),
                    "{x} m at {gs} kt"
                );
                assert_eq!(c.cut_i_deg, 0.0);
            }
        }
        // From 80 kt the cut is twice the closure-time cut on the same reads.
        let mut fast = RollCut::for_law(&RollSteer::with_rudder(true, rudder_737()));
        for _ in 0..3 {
            let c = fast.update(1.0, 0.0, 120.0, 0.25);
            assert!((c.cut_deg - 2.0 * roll_cut_deg(c.xtk_m, c.rate_m_s, 120.0)).abs() < 1e-12);
        }
        let mut a = RollSteer::with_rudder(true, rudder_737());
        let mut b = RollSteer::with_rudder(true, rudder_737()).as_flown();
        for (err, gs) in [(2.0, 10.0), (1.0, 25.0), (-1.5, 35.0), (0.5, 39.0)] {
            assert_eq!(
                a.step_with_yaw(err, 0.25, gs, 1.0, Some(0.2)),
                b.step_with_yaw(err, 0.25, gs, 1.0, Some(0.2))
            );
        }
        // Past the declared speed the gain holds instead of falling as 1/v².
        let kp = |gs: f64| {
            let mut l = RollSteer::with_rudder(true, rudder_737());
            l.step_with_yaw(1.0, 0.25, gs, 1.0, Some(0.0))
        };
        assert!(
            (kp(140.0) - kp(80.0)).abs() < 1e-9,
            "{} {}",
            kp(140.0),
            kp(80.0)
        );
    }

    #[test]
    fn the_roll_does_not_weave_down_the_centreline() {
        /// One run. `scaled` picks the law: false is the old fixed angle per
        /// metre, true is [`roll_target_heading`] as it now stands.
        fn fly_roll(
            scaled: bool,
            gs0: f64,
            xtk0: f64,
            psi0: f64,
            decel: f64,
            secs: f64,
        ) -> Vec<f64> {
            const COURSE: f64 = 24.76;
            const TICK: f64 = 0.42;
            const DELAY_TICKS: usize = 1;
            let mut law = RollSteer::default();
            let (mut psi, mut yaw, mut xtk, mut gs) = (psi0, 0.0_f64, xtk0, gs0);
            let mut pipe = std::collections::VecDeque::from(vec![0.0; DELAY_TICKS]);
            let mut prev: Option<f64> = None;
            let mut track = Vec::new();
            let mut t = 0.0;
            while t < secs {
                let rate = prev.map_or(0.0, |p| (xtk - p) / TICK);
                prev = Some(xtk);
                let target = if scaled {
                    roll_target_heading(COURSE, xtk, rate, gs)
                } else {
                    // The law as it was: a fixed cut per metre, no lead.
                    let corr = (xtk * XTK_GAIN_DEG_PER_M).clamp(-XTK_MAX_DEG, XTK_MAX_DEG);
                    (COURSE - corr).rem_euclid(360.0)
                };
                let err = ((target - psi + 540.0).rem_euclid(360.0)) - 180.0;
                pipe.push_back(law.step(err, TICK, gs, 0.0));
                let steer = pipe.pop_front().unwrap_or(0.0);
                // Sub-steps: the airframe does not tick at the loop's rate.
                for _ in 0..12 {
                    let h = TICK / 12.0;
                    // Yaw rate follows the rudder through a 0.7 s lag, with
                    // the surface biting harder the faster the air over it.
                    let want = 18.0 * steer * (gs / 120.0).min(1.0);
                    yaw += (want - yaw) * (h / 0.7).min(1.0);
                    psi += yaw * h;
                    xtk += gs / MS_TO_KTS * (psi - COURSE).to_radians().sin() * h;
                    gs = (gs - decel * h).max(8.0);
                }
                track.push(xtk);
                t += TICK;
            }
            track
        }
        let span = |t: &[f64]| {
            t.iter().cloned().fold(f64::MIN, f64::max) - t.iter().cloned().fold(f64::MAX, f64::min)
        };

        // The landing rollout: 140 kt, four metres left, decelerating hard.
        let old = fly_roll(false, 140.0, -4.2, 24.3, 4.0, 20.0);
        let new = fly_roll(true, 140.0, -4.2, 24.3, 4.0, 20.0);
        assert!(
            span(&old) > 9.0,
            "the plant must still weave under the old law, or this test is \
             not about LFPB any more: span {:.1} m",
            span(&old)
        );
        assert!(
            span(&new) < span(&old) * 0.6,
            "the scaled cut takes the weave out: {:.1} m against {:.1}",
            span(&new),
            span(&old)
        );
        assert!(
            new.last().is_some_and(|x| x.abs() < 2.0),
            "and it ends ON the centreline, not merely swinging through it: {:?}",
            new.last()
        );

        // The takeoff roll: accelerating, so the gain the old law held all
        // the way to Vr is the one that grew a 10 m offset by liftoff.
        let old = fly_roll(false, 41.0, 1.74, 24.63, -2.9, 21.0);
        let new = fly_roll(true, 41.0, 1.74, 24.63, -2.9, 21.0);
        assert!(
            new.last().unwrap().abs() < old.last().unwrap().abs(),
            "the wheels leave the ground nearer the centreline: {:.1} against {:.1}",
            new.last().unwrap(),
            old.last().unwrap()
        );
        assert!(
            span(&new) <= span(&old),
            "and no new excursion is bought for it: {:.1} against {:.1}",
            span(&new),
            span(&old)
        );
    }

    /// The lead looks at where the offset WILL be. Drifting AWAY earns more
    /// cut than the offset alone; already closing fast earns less, which is
    /// what stops the overshoot.
    #[test]
    fn the_roll_cut_leads_the_cross_track() {
        let gs = 140.0;
        let held = 360.0 - roll_target_heading(0.0, 5.0, 0.0, gs);
        let diverging = 360.0 - roll_target_heading(0.0, 5.0, 1.0, gs);
        let closing = roll_target_heading(0.0, 5.0, -3.0, gs);
        assert!(diverging > held, "drifting off earns more cut: {diverging}");
        assert!(
            closing < 1.0 || closing > 359.0,
            "closing fast enough, the cut is already given back: {closing}"
        );
        // Nothing the sim can hand us turns the target into a NaN.
        for bad in [f64::NAN, f64::INFINITY] {
            for tgt in [
                roll_target_heading(0.0, bad, 0.0, gs),
                roll_target_heading(0.0, 5.0, bad, gs),
                roll_target_heading(0.0, 5.0, 0.0, bad),
            ] {
                assert!(tgt.is_finite(), "a bad read must not steer by NaN");
            }
        }
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
        assert!(
            law.trim() > 0.2,
            "learned standing rudder, got {}",
            law.trim()
        );
        // And with the error finally at zero the trim STILL holds rudder —
        // the whole point: straight nose, rudder in.
        let centered = law.step(0.0, 0.15, 40.0, 0.0);
        assert!(
            centered > 0.15,
            "rudder held with the nose straight: {centered}"
        );
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
        assert!(
            (big.trim() - small.trim()).abs() < 1e-9,
            "{} vs {}",
            big.trim(),
            small.trim()
        );
        assert!(
            big.trim() < 0.1,
            "two seconds of swing stays a nudge: {}",
            big.trim()
        );
    }

    /// The overspeed guard must OWN the airborne lever. It used to share it
    /// with an unconditional 1.0 commanded every airborne tick, so the
    /// guard's reduced write survived one tick at a time (the SF50's Vmo
    /// overspeed came straight back), and the pilot-override detector read
    /// the guard's own deep cut as a hand on the lever. March the loop's
    /// real per-tick arithmetic through one writer: the lever settles ON the
    /// guard value and stays, and the readback of our own write can never
    /// trip the override threshold.
    #[test]
    fn the_overspeed_guard_owns_the_airborne_lever() {
        let vne = Some(250.0); // SF50 Vmo, the measured live case
        let mut last_thr_cmd = 1.0_f64; // full power off the runway
        for _ in 0..40 {
            let want = tick_throttle_want(true, 60.0, 0.0, 140.0, 270.0, vne);
            let thr_cmd = throttle_slew(last_thr_cmd, want, 0.4);
            // The sim reads back what this loop wrote last tick; the
            // override detector (thr_now < last_cmd - 0.25) must never see
            // the guard's own command as a pilot pull.
            let thr_now = last_thr_cmd;
            assert!(
                thr_now >= last_thr_cmd - 0.25,
                "the guard's own write tripped the override detector"
            );
            last_thr_cmd = thr_cmd;
        }
        let guard = climb_throttle(270.0, vne);
        assert!(guard < 0.75, "the live case cuts deep: {guard}");
        assert!(
            (last_thr_cmd - guard).abs() < 1e-9,
            "the lever settles ON the guard and stays: {last_thr_cmd} vs {guard}"
        );
        // Ground ticks are untouched: the roll's ramp and swinging-nose hold
        // still own the lever there.
        assert_eq!(tick_throttle_want(false, 10.0, 2.0, 8.0, 0.0, vne), 1.0);
        assert_eq!(
            tick_throttle_want(false, 10.0, 14.0, 8.0, 0.0, vne),
            THROTTLE_HOLD_RATIO
        );
        // No declared Vne keeps the full-power climb every airframe flew.
        assert_eq!(tick_throttle_want(true, 60.0, 0.0, 140.0, 200.0, None), 1.0);
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
        let s = (0..3)
            .map(|_| slow.step(10.0, 0.15, 10.0, 0.0))
            .last()
            .unwrap();
        let f = (0..3)
            .map(|_| fast.step(10.0, 0.15, 50.0, 0.0))
            .last()
            .unwrap();
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
        let with_power = (0..5)
            .map(|_| law.step(0.0, 0.15, 5.0, 1.0))
            .last()
            .unwrap();
        assert!(
            with_power > 0.1,
            "lead rides the throttle, got {with_power}"
        );
        let mut idle = RollSteer::default();
        let no_power = (0..5)
            .map(|_| idle.step(0.0, 0.15, 5.0, 0.0))
            .last()
            .unwrap();
        assert!(no_power.abs() < 0.01, "no power, no lead: {no_power}");
        // The lead fades as airflow straightens the moment out.
        assert!(power_lead(1.0, 40.0) < power_lead(1.0, 0.0));
        assert!(
            power_lead(1.0, 40.0) > 0.05,
            "never fades to nothing on the roll"
        );
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
        assert!(
            !rotate_ok(25.0, 900.0),
            "yawed with room: keep straightening"
        );
        assert!(rotate_ok(25.0, 300.0), "yawed but running out: fly");
    }

    /// A rotation is a RATE. Luton 2026-07-28: the old law saw the whole 8° as
    /// error on the first tick, wound the integrator to its cap pulling for it,
    /// and the nose arrived with enough momentum to overshoot to 10.6° at 21 ft
    /// — a tail strike on a fuselage that runs out around 11, then a ±2.5°
    /// oscillation for the rest of the initial climb.
    #[test]
    fn the_rotation_walks_the_nose_up_instead_of_snatching_it() {
        let mut law = RotatePitch::with_authority(None, PITCH_MIN);
        // First tick from a nose-on-the-ground attitude: the commanded target
        // starts THERE, so the error is small and the elevator stays gentle.
        let first = law.step(0.0, None, 0.15);
        // "Gentle" is a small FRACTION of the available pull, not a number a
        // pure integrator produced for free by starting at zero. The law is
        // P+I now, so the first tick is deliberately non-zero — that is the
        // proportional path doing its job. What must never happen is the
        // snatch: jumping toward the cap.
        assert!(first > 0.0, "it must start pulling: {first}");
        assert!(
            first < 0.2 * PITCH_MAX,
            "gentle to start, not a snatch: {first} of {PITCH_MAX}"
        );
        // The real anti-snatch guarantee, unchanged: the COMMANDED attitude
        // starts at the nose and is walked up, so the law never sees the whole
        // 8 degrees as error on tick one.
        assert!(law.target_deg() < 1.0, "target starts at the nose");
        // Two seconds later the commanded attitude has walked up about
        // 2.5°/s, and no further than the 8° target.
        let mut ramp = RotatePitch::with_authority(None, PITCH_MIN);
        for _ in 0..14 {
            ramp.step(0.0, None, 0.15);
        }
        let t = ramp.target_deg();
        assert!((t - 2.1 * ROTATE_RATE_DEG_S).abs() < 1.5, "ramping: {t}");
        for _ in 0..60 {
            ramp.step(0.0, None, 0.15);
        }
        assert!(
            (ramp.target_deg() - ROTATE_PITCH_DEG).abs() < 1e-6,
            "and stops at the target: {}",
            ramp.target_deg()
        );
        // A nose FOLLOWING the ramp is doing what was asked, so the elevator
        // must not keep winding: the rate damping measures the rate ERROR, not
        // the rotation itself.
        let mut good = RotatePitch::with_authority(None, PITCH_MIN);
        let mut theta = 0.0;
        let mut peak: f64 = 0.0;
        for _ in 0..60 {
            let out = good.step(theta, None, 0.15);
            peak = peak.max(out);
            theta = (theta + ROTATE_RATE_DEG_S * 0.15).min(ROTATE_PITCH_DEG);
        }
        assert!(
            peak < 0.25,
            "a rotation on rate needs no big pull: peak {peak}"
        );
    }

    /// A long fuselage gets a ceiling: the profile's tail-strike attitude, less
    /// a margin, bounds what the rotation may ever ask for.
    #[test]
    fn a_tail_strike_limit_caps_the_commanded_attitude() {
        // 9.5° geometry, 2° margin → 7.5°, below the law's own 8° target.
        let mut md = RotatePitch::with_authority(Some(9.5), PITCH_MIN);
        for _ in 0..80 {
            md.step(0.0, None, 0.15);
        }
        assert!(
            (md.target_deg() - 7.5).abs() < 1e-6,
            "capped by the airframe: {}",
            md.target_deg()
        );
        // Past the ceiling, the law pushes back rather than holding the pull.
        let mut over = RotatePitch::with_authority(Some(9.5), PITCH_MIN);
        for _ in 0..40 {
            over.step(0.0, None, 0.15);
        }
        let pulling = over.out;
        for _ in 0..20 {
            over.step(11.0, None, 0.15);
        }
        assert!(
            over.out < pulling,
            "above the tail limit the elevator must relax: {} !< {pulling}",
            over.out
        );
        // No limit declared → the plain 8° target, unchanged.
        let mut ga = RotatePitch::with_authority(None, PITCH_MIN);
        for _ in 0..80 {
            ga.step(0.0, None, 0.15);
        }
        assert!((ga.target_deg() - ROTATE_PITCH_DEG).abs() < 1e-6);
    }

    /// A declared rotation rate walks the target up that fast; none keeps
    /// the airline 2.5 (the SR22 declares 5: LEMG 12 1790690995).
    #[test]
    fn a_declared_rotation_rate_walks_the_target_faster() {
        let target_after = |rate: Option<f64>| {
            let mut law = RotatePitch::with_authority(None, PITCH_MIN).with_rotate_rate(rate);
            for _ in 0..8 {
                law.step(-1.75, None, 0.25);
            }
            law.target_deg()
        };
        let airline = target_after(None);
        let single = target_after(Some(5.0));
        assert!((airline - (-1.75 + 2.5 * 1.75)).abs() < 0.3, "{airline}");
        assert!((single - (-1.75 + 5.0 * 1.75)).abs() < 1e-6, "{single}");
        assert!((target_after(Some(50.0)) - ROTATE_PITCH_DEG).abs() < 1e-6, "bounded at 6");
    }

    /// One rotation tick as the blackbox has it (a `roll` record from the one
    /// the rotation began on, then `climb_manual`).
    #[derive(Debug, Clone, Copy)]
    struct RotTick {
        t: f64,
        dt: f64,
        ias: f64,
        theta: f64,
        q: f64,
        /// Height, feet; zero on a roll record (it carries none).
        agl: f64,
        /// The column the law logged (climb ticks only).
        pitch: Option<f64>,
    }

    /// The three SR22 rotations of LEMG 13, 2026-09-29 (#184), by sortie,
    /// with the rotation rate each flew: 1790682305 (MacBook, 0.25 s tick)
    /// and 1790690995 at the airline 2.5, 1790694884 at the declared 5.
    fn sr22_rotations() -> Vec<(&'static str, Option<f64>, Vec<RotTick>)> {
        let recs: Vec<serde_json::Value> =
            include_str!("../bench/takeoffs/sr22_rotation_lemg_2026-09-29.jsonl")
                .lines()
                .filter_map(|l| serde_json::from_str(l).ok())
                .collect();
        let f = |r: &serde_json::Value, k: &str| r[k].as_f64();
        [
            ("1790682305-full_auto", None),
            ("1790690995-full_auto", None),
            ("1790694884-full_auto", Some(5.0)),
        ]
        .into_iter()
        .map(|(sid, rate)| {
            let ticks = recs
                .iter()
                .filter(|r| r["sortie"] == sid)
                .filter(|r| r["event"] == "roll" || r["event"] == "climb_manual")
                .map(|r| RotTick {
                    t: f(r, "t").unwrap(),
                    dt: f(r, "dt").unwrap(),
                    ias: f(r, "ias").unwrap(),
                    theta: f(r, "theta").unwrap(),
                    q: f(r, "q").unwrap(),
                    agl: f(r, "agl").unwrap_or(0.0),
                    pitch: f(r, "pitch").filter(|_| r["event"] == "climb_manual"),
                })
                .collect();
            (sid, rate, ticks)
        })
        .collect()
    }

    /// The SR22's rotation law as it flew: GA authority, no ceiling, no
    /// hold, the sortie's rate.
    fn sr22_law(rate: Option<f64>) -> RotatePitch {
        RotatePitch::with_authority(None, PITCH_MIN).with_rotate_rate(rate)
    }

    /// The SR22's fitted rotation, as the profile declares it.
    fn rotate_model_sr22() -> crate::aircraft::RotateElevatorProfile {
        let text = std::fs::read_to_string("profiles/cirrus_sr22.yaml").unwrap();
        let p: crate::aircraft::AircraftProfile = serde_yaml::from_str(&text).unwrap();
        p.rotate_elevator
            .and_then(|m| m.valid())
            .expect("the SR22 declares its rotation")
    }

    /// Open loop over the recorded ticks: the column the law writes on each.
    fn replay_rotation(law: &mut RotatePitch, ticks: &[RotTick]) -> Vec<f64> {
        ticks
            .iter()
            .map(|k| {
                law.on_wheels(k.agl <= 0.0);
                law.step_climb(k.theta, Some(k.q), k.dt, Some((k.ias, k.agl)))
            })
            .collect()
    }

    /// The SR22's pitch through the rotation, fitted to those three (#184):
    /// a rate plant (the column sets a pitch rate through a lag `tau`, after
    /// the write's delay). The nose turns
    /// `k x ((v/ref)^2 x (u - per_deg x (alpha - rest)) - base)` deg/s, alpha
    /// the attitude less the flight path (level on the wheels), and never
    /// below the roll attitude on the wheels: the `per_deg` is the tail's
    /// spring, the base on the wheels the breakout (the nose's weight about the
    /// mains). Fitted by least squares on the attitude to 5 degrees on the
    /// wheels, the column replayed from the ticks: 0.35 degree rms.
    ///
    /// Flying (1790699819, #184): the base stays the breakout in ground effect
    /// and walks to `AIR_BASE` by `GROUND_EFFECT_FT` (0.31 held 4.6 degrees at
    /// 88 KIAS a few feet up; the 100 kt climbs hold 10 degrees on a 7 degree
    /// path with 0.1). The wing lifts as `v^2 x (alpha - LIFT_ZERO_DEG)`, ten
    /// percent more in ground effect, the weight balanced at 104 kt and 3.2
    /// degrees of alpha (the steady climb); the height is flown on the excess.
    /// With the elevator 1.4 and the spring 1.3 of the fit (the 1790699819
    /// airframe, whose nose came up at 4.7 deg/s on 0.38 of column), the law
    /// b2b1481 flew leaves the wheels at 80.7 KIAS and 5.0 degrees (flown
    /// 81.4, 5.0) and reaches 8 ft at 94.7 (flown 94.7).
    struct Sr22Rotation {
        theta: f64,
        q: f64,
        v: f64,
        /// Height, feet, and vertical speed, ft/s.
        h: f64,
        vs: f64,
        wheels: bool,
        /// Multipliers on the fitted elevator and spring (robustness).
        k_mult: f64,
        spring_mult: f64,
    }

    impl Sr22Rotation {
        const REF_KTS: f64 = 90.0;
        const K: f64 = 29.2;
        const BREAKOUT: f64 = 0.104;
        const PER_DEG: f64 = 0.0328;
        const TAU_S: f64 = 0.131;
        const DELAY_S: f64 = 0.26;
        const AIR_BASE: f64 = -0.059;
        const GROUND_EFFECT_FT: f64 = 20.0;
        const GROUND_EFFECT_LIFT: f64 = 0.10;
        const REST_DEG: f64 = -1.65;
        const LIFT_ZERO_DEG: f64 = -0.33;
        const CLIMB_KTS: f64 = 104.0;
        const CLIMB_ALPHA_DEG: f64 = 3.2;
        /// Airspeed gained per second on the roll (74 to 98 KIAS in 5.6 s),
        /// less half a knot a second per degree of path.
        const ACCEL_KTS_S: f64 = 4.3;

        fn new(theta: f64, v: f64) -> Self {
            Sr22Rotation {
                theta,
                q: 0.0,
                v,
                h: 0.0,
                vs: 0.0,
                wheels: true,
                k_mult: 1.0,
                spring_mult: 1.0,
            }
        }

        /// The airframe of 1790699819 (see above).
        fn flown_1790699819(v: f64) -> Self {
            Sr22Rotation {
                k_mult: 1.4,
                spring_mult: 1.3,
                ..Self::new(Self::REST_DEG, v)
            }
        }

        fn path_deg(&self) -> f64 {
            self.vs.atan2(self.v * 1.687_81).to_degrees()
        }

        fn advance(&mut self, u: f64, dur_s: f64) {
            let step_s = 0.005_f64;
            let mut left = dur_s;
            let weight = Self::CLIMB_KTS.powi(2) * (Self::CLIMB_ALPHA_DEG - Self::LIFT_ZERO_DEG);
            while left > 1e-9 {
                let step = step_s.min(left);
                let dq = (self.v / Self::REF_KTS).powi(2);
                let path = if self.wheels { 0.0 } else { self.path_deg() };
                let alpha = self.theta - path;
                let in_effect = (1.0 - self.h / Self::GROUND_EFFECT_FT).max(0.0);
                let base = if self.wheels {
                    Self::BREAKOUT
                } else {
                    Self::AIR_BASE + (Self::BREAKOUT - Self::AIR_BASE) * in_effect
                };
                let want = Self::K
                    * self.k_mult
                    * (dq * (u - Self::PER_DEG * self.spring_mult * (alpha - Self::REST_DEG))
                        - base);
                self.q += (want - self.q) / Self::TAU_S * step;
                self.theta += self.q * step;
                if self.wheels && self.theta < Self::REST_DEG {
                    self.theta = Self::REST_DEG;
                    self.q = self.q.max(0.0);
                }
                let lift = self.v.powi(2)
                    * (alpha - Self::LIFT_ZERO_DEG)
                    * (1.0 + Self::GROUND_EFFECT_LIFT * in_effect)
                    / weight;
                let accel = 32.2 * (lift - 1.0);
                if self.wheels && accel > 0.0 {
                    self.wheels = false;
                }
                if !self.wheels {
                    self.vs += accel * step;
                    self.h += self.vs * step;
                    if self.h <= 0.0 && self.vs < 0.0 {
                        self.h = 0.0;
                        self.vs = 0.0;
                        self.wheels = true;
                    }
                }
                self.v += (Self::ACCEL_KTS_S - 0.5 * path) * step;
                left -= step;
            }
        }
    }

    /// What a closed-loop rotation did, as the blackbox would say it.
    #[derive(Debug, Clone, Copy)]
    struct RotationFlown {
        /// Airspeed where the wheels left, KIAS, and the attitude there.
        wheels_off_kts: f64,
        wheels_off_deg: f64,
        /// Airspeed at 8 ft (the takeoff's `liftoff`), KIAS.
        liftoff_kts: f64,
        /// Seconds from Vr to the nose at 7 degrees.
        to_7_deg_s: f64,
        /// Highest attitude; the lowest between the wheels leaving and 8 ft
        /// (the float's sag); the lowest from a second after 8 ft on.
        max_theta: f64,
        float_min: f64,
        min_after: f64,
    }

    /// Fly `law` against the fitted SR22 from Vr, through the write's delay,
    /// at `tick` seconds with a jitter and the runway's rate noise; the law
    /// reads the wheels, the height and the vertical speed as the loop does.
    fn fly_sr22_rotation(
        mut law: RotatePitch,
        mut plant: Sr22Rotation,
        tick: f64,
        seed: u32,
    ) -> RotationFlown {
        let mut seed = seed.wrapping_mul(2_654_435_761).wrapping_add(1);
        let mut noise = move || {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            f64::from(seed >> 8) / f64::from(1_u32 << 23) - 1.0
        };
        let mut pending: Vec<(f64, f64)> = Vec::new();
        let mut applied = 0.0;
        let mut t = 0.0_f64;
        let (mut off_kts, mut off_deg) = (f64::NAN, f64::NAN);
        let (mut liftoff, mut lift_t) = (f64::NAN, f64::NAN);
        let mut to_7 = f64::NAN;
        let (mut max_theta, mut float_min, mut min_after) = (f64::MIN, f64::MAX, f64::MAX);
        while t < 14.0 {
            let dt = tick + 0.2 * tick * noise();
            let on = plant.wheels;
            let q_read = plant.q + noise() * if on { 1.0 } else { 0.1 };
            law.on_wheels(on);
            law.climb_fpm(Some(plant.vs * 60.0));
            let theta_read = plant.theta + 0.1 * noise();
            let cmd = law.step_climb(theta_read, Some(q_read), dt, Some((plant.v, plant.h)));
            pending.push((t + Sr22Rotation::DELAY_S, cmd));
            let end = t + dt;
            while t < end - 1e-9 {
                let next = pending
                    .iter()
                    .map(|&(at, _)| at)
                    .filter(|&at| at > t && at < end)
                    .fold(end, f64::min)
                    .min(t + 0.01);
                plant.advance(applied, next - t);
                t = next;
                while let Some(i) = pending.iter().position(|&(at, _)| at <= t) {
                    applied = pending.remove(i).1;
                }
                if !plant.wheels && off_kts.is_nan() {
                    off_kts = plant.v;
                    off_deg = plant.theta;
                }
                if plant.h > 8.0 && liftoff.is_nan() {
                    liftoff = plant.v;
                    lift_t = t;
                }
                if to_7.is_nan() && plant.theta >= 7.0 {
                    to_7 = t;
                }
                max_theta = max_theta.max(plant.theta);
                if off_kts.is_finite() && liftoff.is_nan() {
                    float_min = float_min.min(plant.theta);
                }
                if lift_t.is_finite() && t > lift_t + 1.0 {
                    min_after = min_after.min(plant.theta);
                }
            }
        }
        RotationFlown {
            wheels_off_kts: off_kts,
            wheels_off_deg: off_deg,
            liftoff_kts: liftoff,
            to_7_deg_s: to_7,
            max_theta,
            float_min,
            min_after,
        }
    }

    /// The replay IS the law as flown: stepped over the recorded ticks, the
    /// rotation law writes the column every climb tick logged, on all three
    /// flights. The roll records carry no column; this is how the SR22's was
    /// read: 0.03 to 0.36 through the whole rotation (PITCH_MAX 0.65).
    #[test]
    fn the_sr22_rotation_replays_as_flown() {
        for (sid, rate, ticks) in sr22_rotations() {
            let out = replay_rotation(&mut sr22_law(rate), &ticks);
            let mut climbs = 0;
            for (k, u) in ticks.iter().zip(&out) {
                if let Some(logged) = k.pitch {
                    assert!((u - logged).abs() < 1e-9, "{sid} at {}: {u} vs {logged}", k.t);
                    climbs += 1;
                }
            }
            assert!(climbs >= 10, "{sid}: {climbs} climb ticks");
            let ground_max = ticks
                .iter()
                .zip(&out)
                .filter(|(k, _)| k.agl <= 0.0)
                .map(|(_, u)| *u)
                .fold(f64::MIN, f64::max);
            assert!(ground_max < 0.37, "{sid}: the rotation never pulled past {ground_max}");
        }
    }

    /// The plant is the flown rotation: driven by the column each flight
    /// wrote (replayed), from the attitude at Vr, it keeps the recorded
    /// attitude on the wheels to 5 degrees within half a degree rms and a
    /// degree on every tick: the nose held back near 3 degrees at 88 to 94
    /// kt on the two iMac flights, 0.3 to 0.4 of column against the spring.
    #[test]
    fn the_sr22_plant_is_the_flown_rotation() {
        let mut sq = 0.0;
        let mut n = 0.0;
        for (sid, rate, ticks) in sr22_rotations() {
            let cols = replay_rotation(&mut sr22_law(rate), &ticks);
            let ground: Vec<_> = ticks
                .iter()
                .zip(&cols)
                .take_while(|(k, _)| k.agl <= 0.0 && k.theta <= 5.0)
                .collect();
            let mut plant = Sr22Rotation::new(ground[0].0.theta, ground[0].0.ias);
            let mut pending: Vec<(f64, f64)> = ground
                .iter()
                .map(|(k, u)| (k.t + Sr22Rotation::DELAY_S, **u))
                .collect();
            let mut applied = 0.0;
            let mut t = ground[0].0.t;
            for (k, _) in &ground {
                while t < k.t - 1e-9 {
                    while pending.first().is_some_and(|&(at, _)| at <= t) {
                        applied = pending.remove(0).1;
                    }
                    let step = 0.01_f64.min(k.t - t);
                    // The airspeed as recorded, not the plant's own.
                    plant.v = k.ias;
                    plant.advance(applied, step);
                    // Recorded on the wheels: the ground half of the plant.
                    (plant.wheels, plant.h, plant.vs) = (true, 0.0, 0.0);
                    t += step;
                }
                let e = plant.theta - k.theta;
                sq += e * e;
                n += 1.0;
                assert!(
                    e.abs() < 1.0,
                    "{sid} at {} kt: plant {} vs flown {}",
                    k.ias,
                    plant.theta,
                    k.theta
                );
            }
        }
        let rms = (sq / n).sqrt();
        assert!(rms < 0.5, "the fit holds: {rms:.2} degree rms over {n} ticks");
    }

    /// #184: the SR22 rotated at Vr 73 and lifted off at 98 to 100 KIAS on
    /// every flight, the nose 5 degrees behind the ramp. Against the fitted
    /// plant, at the iMac's 0.4 s tick and the MacBook's 0.25, the law as it
    /// flew then reaches 8 ft past 92 kt, the nose at 7 degrees 5 s after Vr;
    /// fed the declared model, by 88 kt, the nose at 7 inside 2.6 s, never
    /// higher than 10, no sag between the wheels and 8 ft and no drop under 7
    /// once climbing. The model is a hypothesis about the plant, so the
    /// elevator and the spring at 0.7 to 1.4 of the fit must still beat the
    /// old law, without the float's sag, and overshoot it by little.
    #[test]
    fn the_sr22_rotates_near_vr() {
        let model = rotate_model_sr22();
        let vr = 73.5;
        for tick in [0.25, 0.4] {
            for seed in 0..6 {
                let fly = |law: RotatePitch| {
                    let plant = Sr22Rotation::new(Sr22Rotation::REST_DEG, vr);
                    fly_sr22_rotation(law, plant, tick, seed)
                };
                let old = fly(sr22_law(Some(5.0)));
                assert!(old.liftoff_kts > 92.0, "the flown law, tick {tick}: {old:?}");
                assert!(old.to_7_deg_s > 5.0, "the flown law, tick {tick}: {old:?}");
                let new = fly(sr22_law(Some(5.0)).with_rotate_model(Some(model)));
                assert!(new.liftoff_kts < 88.0, "tick {tick} seed {seed}: {new:?}");
                assert!(new.to_7_deg_s < 2.6, "tick {tick} seed {seed}: {new:?}");
                assert!(new.max_theta < 10.0, "tick {tick} seed {seed}: {new:?}");
                assert!(new.float_min > new.wheels_off_deg - 0.3, "sagged: {new:?}");
                assert!(new.min_after > 7.0, "tick {tick} seed {seed}: {new:?}");
            }
        }
        for k_mult in [0.7, 1.0, 1.4] {
            for spring_mult in [0.7, 1.3] {
                let plant = || Sr22Rotation {
                    k_mult,
                    spring_mult,
                    ..Sr22Rotation::new(Sr22Rotation::REST_DEG, vr)
                };
                let old = fly_sr22_rotation(sr22_law(Some(5.0)), plant(), 0.4, 1);
                let law = sr22_law(Some(5.0)).with_rotate_model(Some(model));
                let new = fly_sr22_rotation(law, plant(), 0.4, 1);
                let case = format!("elevator x{k_mult}, spring x{spring_mult}");
                assert!(new.liftoff_kts < old.liftoff_kts - 3.0, "{case}: {new:?} vs {old:?}");
                // A softer spring than declared is an over-pull: 11.2 and 12.4
                // degrees at 0.7 of it (the old law 11.3 and 11.2).
                assert!(new.max_theta < 12.5, "{case}: {new:?}");
                assert!(new.max_theta < old.max_theta + 1.3, "{case}: {new:?} vs {old:?}");
                assert!(new.float_min > new.wheels_off_deg - 0.5, "{case}: {new:?}");
            }
        }
    }

    /// 1790699819 (LEMG 13, the iMac, 2026-09-29): the first model, flown.
    /// On the wheels it worked (Vr 74.6, wheels off 81.4 KIAS 1.4 s later at
    /// 5.0 degrees); then its flat flying hold took the column from 0.39 to
    /// 0.24, the nose sagged to 3.0, and the aircraft floated in ground
    /// effect to the 8 ft liftoff at 94.7 KIAS. Against the plant of that
    /// airframe, the law as b2b1481 flew it does the same (wheels off near
    /// 81 at 5 degrees, 8 ft past 92); the path-and-ground-effect model
    /// climbs out of the float by 89 without a sag, under 10 degrees.
    #[test]
    fn the_sr22_climbs_out_of_ground_effect() {
        let model = rotate_model_sr22();
        let mut flown_liftoffs = Vec::new();
        for tick in [0.25, 0.4] {
            for seed in 0..6 {
                let fly = |law: RotatePitch| {
                    fly_sr22_rotation(law, Sr22Rotation::flown_1790699819(74.6), tick, seed)
                };
                let law = || sr22_law(Some(5.0)).with_rotate_model(Some(model));
                let flown = fly(law().as_flown_b2b1481());
                assert!(
                    (79.0..=83.0).contains(&flown.wheels_off_kts)
                        && (4.0..=6.0).contains(&flown.wheels_off_deg),
                    "as flown, tick {tick} seed {seed}: {flown:?}"
                );
                // The float is chaotic (88.5 to 97.8 over the seeds); the
                // flight's 94.7 is its middle.
                assert!(flown.liftoff_kts > 88.0, "as flown, tick {tick} seed {seed}: {flown:?}");
                flown_liftoffs.push(flown.liftoff_kts);
                let new = fly(law());
                assert!(new.liftoff_kts < flown.liftoff_kts, "{new:?} vs {flown:?}");
                assert!(new.liftoff_kts < 89.0, "tick {tick} seed {seed}: {new:?}");
                assert!(new.float_min > new.wheels_off_deg - 0.3, "sagged: {new:?}");
                assert!(new.max_theta < 10.0, "tick {tick} seed {seed}: {new:?}");
                assert!(new.min_after > 7.0, "tick {tick} seed {seed}: {new:?}");
            }
        }
        let mean = flown_liftoffs.iter().sum::<f64>() / flown_liftoffs.len() as f64;
        assert!(mean > 92.0, "the float as flown: {mean:.1} kt at 8 ft");
    }

    /// The rotation records of 1790699819 (`rotate_tick`: the column and the
    /// model's share, the wheels).
    fn sr22_float_ticks() -> Vec<serde_json::Value> {
        include_str!("../bench/takeoffs/sr22_float_lemg13_1790699819.jsonl")
            .lines()
            .filter_map(|l| serde_json::from_str::<serde_json::Value>(l).ok())
            .filter(|r| r["event"] == "rotate_tick")
            .collect()
    }

    /// Open loop over 1790699819's rotation: the law as flown (b2b1481)
    /// writes every logged column, wheels and float alike, so the replay is
    /// the flight. The model now: identical on the wheels, and at wheels-off
    /// (81.4 KIAS, 5.0 degrees) it keeps the pull, 0.35 or more where the
    /// flight dropped to 0.24; through the float it never feeds forward less
    /// than the 0.3 that held 4.6 degrees at 88 KIAS.
    #[test]
    fn the_float_of_1790699819_replays_and_keeps_its_pull() {
        let ticks = sr22_float_ticks();
        assert_eq!(ticks.len(), 12);
        let f = |r: &serde_json::Value, k: &str| r[k].as_f64().unwrap();
        let model = rotate_model_sr22();
        let mut flown = sr22_law(Some(5.0)).with_rotate_model(Some(model)).as_flown_b2b1481();
        let mut now = sr22_law(Some(5.0)).with_rotate_model(Some(model));
        let mut first_float = true;
        for r in &ticks {
            let wheels = r["wheels"] == true;
            // The float's height was not logged: a few feet, under the 8 ft.
            let air = Some((f(r, "ias"), if wheels { 0.0 } else { 3.0 }));
            let (theta, q, dt) = (f(r, "theta"), Some(f(r, "q")), f(r, "dt"));
            flown.on_wheels(wheels);
            now.on_wheels(wheels);
            let a = flown.step_climb(theta, q, dt, air);
            let b = now.step_climb(theta, q, dt, air);
            assert!((a - f(r, "pitch")).abs() < 1e-9, "replay at {}: {a}", f(r, "t"));
            assert!((flown.model_pull() - f(r, "pitch_ff")).abs() < 1e-9);
            if wheels {
                assert_eq!(a.to_bits(), b.to_bits(), "the wheels are unchanged");
            } else {
                if first_float {
                    assert!(a < 0.25 && b > 0.35, "wheels-off: flown {a}, now {b}");
                    first_float = false;
                }
                assert!(now.model_pull() > 0.3, "fed {} at {}", now.model_pull(), f(r, "ias"));
            }
        }
        assert!(!first_float, "the float is in the record");
    }

    /// No declaration, no change: the law without a model writes the same
    /// column tick for tick whether the wheels are said to be down or not,
    /// and a model that is not sane is no model.
    #[test]
    fn an_undeclared_rotation_model_changes_nothing() {
        for (_, rate, ticks) in sr22_rotations() {
            let mut a = sr22_law(rate);
            let mut b = sr22_law(rate).with_rotate_model(None);
            let insane = crate::aircraft::RotateElevatorProfile {
                breakout: 5.0,
                ..rotate_model_sr22()
            };
            let mut c = sr22_law(rate).with_rotate_model(Some(insane));
            for k in &ticks {
                b.on_wheels(true);
                c.on_wheels(k.agl <= 0.0);
                let air = Some((k.ias, k.agl));
                let x = a.step_climb(k.theta, Some(k.q), k.dt, air);
                assert_eq!(x.to_bits(), b.step_climb(k.theta, Some(k.q), k.dt, air).to_bits());
                assert_eq!(x.to_bits(), c.step_climb(k.theta, Some(k.q), k.dt, air).to_bits());
            }
        }
    }

    #[test]
    fn rotation_is_an_elevator_pull_that_holds_the_attitude() {
        // Sortie five: rotate called at 55 knots, the AP's VS mode held the
        // nose down (engage syncs to the current rate: zero), and the
        // aircraft levitated off at 98 with 250 m left. The rotation law is
        // an integrating elevator pull to the target attitude.
        let mut law = RotatePitch::default();
        let first = law.step(0.0, None, 0.15);
        assert!(first > 0.0, "nose on the ground, pull back: {first}");
        let mut out = first;
        for _ in 0..40 {
            out = law.step(0.0, None, 0.15);
        }
        assert!(out > 0.3, "an unresponsive nose earns more pull: {out}");
        assert!(out <= PITCH_MAX + 1e-9, "capped: {out}");
        // Integrating: on target and steady, the standing pull HOLDS — the
        // elevator the trim state demands at this speed.
        let mut settled = RotatePitch::default();
        for _ in 0..10 {
            settled.step(ROTATE_PITCH_DEG - 1.0, None, 0.15);
        }
        // Arriving at the target leaves a rate residue for a tick or two
        // before the pull is truly standing. Let that wash out, then it must
        // hold dead still — and it must CONVERGE on the way, never wander.
        let mut prev_gap = f64::MAX;
        for _ in 0..20 {
            let x = settled.step(ROTATE_PITCH_DEG, None, 0.15);
            let y = settled.step(ROTATE_PITCH_DEG, None, 0.15);
            let gap = (x - y).abs();
            assert!(gap <= prev_gap + 1e-9, "must converge, not ring: {gap}");
            prev_gap = gap;
        }
        let a = settled.step(ROTATE_PITCH_DEG, None, 0.15);
        let b = settled.step(ROTATE_PITCH_DEG, None, 0.15);
        assert!(
            a > 0.0 && (a - b).abs() < 1e-6,
            "holds at target: {a} vs {b}"
        );
        // A nose rising fast toward the target backs the pull off (the
        // rate lead), so the rotation doesn't slingshot past the attitude.
        let mut rising = RotatePitch::default();
        let before = rising.step(2.0, None, 0.15);
        let after = rising.step(6.0, None, 0.15);
        assert!(after < before, "fast rise backs off: {before} -> {after}");
        // And the nose-down authority is a fraction of the pull: this law
        // rotates, it never bunts.
        let mut over = RotatePitch::default();
        for _ in 0..100 {
            over.step(30.0, None, 0.15);
        }
        assert!(over.out >= PITCH_MIN - 1e-9, "floor holds: {}", over.out);
    }

    /// A second-order pitch plant: elevator commands pitch ACCELERATION,
    /// aero damping opposes the rate, static stability opposes the attitude.
    /// `m0` is a standing nose-up moment — climb power plus a nose-up
    /// stabiliser, which is what makes holding the target need nose-DOWN
    /// elevator against a floor of only [`PITCH_MIN`].
    ///
    /// This is the order the real aircraft has, and it is the whole point: the
    /// sortie test's pitch model is a first-order lag, and a single lag cannot
    /// ring at any gain, which is why the law rang through three gain changes
    /// with every test passing.
    struct PitchPlant {
        theta: f64,
        q: f64,
        wn: f64,
        zeta: f64,
        m_delta: f64,
        m0: f64,
    }

    /// A pitch plant the takeoff harnesses can fly: the LEMG-fitted
    /// [`PitchPlant`] or the EGKK-fitted [`RatePlant`].
    trait PitchModel {
        fn advance(&mut self, elevator: f64, dur_s: f64);
        fn theta(&self) -> f64;
        fn q(&self) -> f64;
        /// On the wheels: the attitude stops at the runway.
        fn hold_on_runway(&mut self);
    }

    impl PitchModel for PitchPlant {
        fn advance(&mut self, elevator: f64, dur_s: f64) {
            PitchPlant::advance(self, elevator, dur_s);
        }
        fn theta(&self) -> f64 {
            self.theta
        }
        fn q(&self) -> f64 {
            self.q
        }
        fn hold_on_runway(&mut self) {
            self.theta = 0.0;
            self.q = self.q.max(0.0);
        }
    }

    /// The 737's pitch as EGKK 08R flew it (sortie 1790601822-full_auto,
    /// 2026-09-28, #78): the column sets a pitch RATE, `k` degrees a second
    /// per unit of column past `u0`, the column that holds the attitude,
    /// through a lag `tau` (the harness adds the loop's transport delay).
    /// Second order in the attitude. Fitted to the 27 airborne ticks from
    /// liftoff to the 300 ft engage, driven by the column the law logged:
    /// `k` 12.9, `u0` -0.139, `tau` 0.1 s, delay 0.4 s, 0.23 degree RMS on
    /// the attitude. The -0.139 is the finding: the 737 pitched up at 3
    /// degrees a second on a column of 0.10 and needed a PUSH of 0.14 to stop,
    /// the nose-up trim and the thrust line the rotation rode on (it rotated
    /// at 2.9 degrees a second on a column of about zero).
    struct RatePlant {
        theta: f64,
        q: f64,
        k: f64,
        u0: f64,
        tau: f64,
    }

    impl RatePlant {
        /// As fitted.
        const K: f64 = 12.9;
        const U0: f64 = -0.139;
        const TAU_S: f64 = 0.1;
        const DELAY_S: f64 = 0.4;
    }

    impl PitchModel for RatePlant {
        fn advance(&mut self, elevator: f64, dur_s: f64) {
            let h = 0.005_f64;
            let mut left = dur_s;
            while left > 1e-9 {
                let step = h.min(left);
                self.q += (self.k * (elevator - self.u0) - self.q) / self.tau * step;
                self.theta += self.q * step;
                left -= step;
            }
        }
        fn theta(&self) -> f64 {
            self.theta
        }
        fn q(&self) -> f64 {
            self.q
        }
        fn hold_on_runway(&mut self) {
            self.theta = 0.0;
            self.q = self.q.max(0.0);
        }
    }

    impl PitchPlant {
        fn new(zeta: f64, m0: f64) -> Self {
            PitchPlant {
                theta: 0.0,
                q: 0.0,
                wn: 2.5,
                zeta,
                m_delta: 167.0,
                m0,
            }
        }
        fn advance(&mut self, elevator: f64, dur_s: f64) {
            let h = 0.005_f64;
            let mut left = dur_s;
            while left > 1e-9 {
                let step = h.min(left);
                let qd = self.m_delta * elevator + self.m0
                    - 2.0 * self.zeta * self.wn * self.q
                    - self.wn * self.wn * self.theta;
                self.q += qd * step;
                self.theta += self.q * step;
                left -= step;
            }
        }
    }

    /// Fly the rotation law against that plant through the loop's real
    /// transport delay (a REST write does not reach the aircraft the instant
    /// it is computed) and measure what the blackbox measures: the attitude
    /// spread once the ramp has arrived, and how much of the time the elevator
    /// is pinned against a stop.
    ///
    /// Returns (spread_deg, saturated_fraction).
    fn fly_rotation(zeta: f64, m0: f64, lag_s: f64, measured_rate: bool) -> (f64, f64) {
        let mut plant = PitchPlant::new(zeta, m0);
        let mut law = RotatePitch::default();
        // A deterministic stand-in for the wobble in a real attitude read —
        // the differencing estimator's whole problem is what it does to this.
        // A deterministic LCG, because the wobble must actually wobble: this
        // is the ±0.05° the differencing estimator turns into ±0.2°/s of
        // phantom rate at a 0.25 s tick, which is the whole mechanism under
        // test. (Written first as a float recurrence that silently collapsed
        // to a constant zero — a noise source stuck at zero makes the fallback
        // path look better than it is.)
        let mut seed = 1_u32;
        let mut wobble = move || {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (f64::from(seed >> 8) / f64::from(1_u32 << 23) - 1.0) * 0.05
        };
        let mut pending: Vec<(f64, f64)> = Vec::new();
        let mut applied = 0.0;
        let (mut t, dt) = (0.0_f64, 0.25_f64);
        let (mut lo, mut hi) = (f64::MAX, f64::MIN);
        let (mut ticks, mut saturated) = (0.0, 0.0);
        while t < 18.0 {
            let rate = measured_rate.then_some(plant.q);
            let cmd = law.step(plant.theta + wobble(), rate, dt);
            pending.push((t + lag_s, cmd));
            // Advance the plant across the tick, applying commands as they land.
            let (mut now, end) = (t, t + dt);
            while now < end - 1e-9 {
                let next = pending
                    .iter()
                    .map(|&(at, _)| at)
                    .filter(|&at| at > now && at < end)
                    .fold(end, f64::min);
                plant.advance(applied, next - now);
                now = next;
                while let Some(i) = pending.iter().position(|&(at, _)| at <= now) {
                    applied = pending.remove(i).1;
                }
            }
            t = end;
            // Measure once the ramp has had time to deliver the nose.
            if t > 5.0 {
                lo = lo.min(plant.theta);
                hi = hi.max(plant.theta);
                ticks += 1.0;
                if cmd <= PITCH_MIN + 1e-9 || cmd >= PITCH_MAX - 1e-9 {
                    saturated += 1.0;
                }
            }
        }
        (hi - lo, saturated / ticks)
    }

    /// The rotation must SETTLE on the target, not ring about it.
    ///
    /// Four takeoffs at EGHI on one evening, same airframe either side of the
    /// P+I change: 1.26 -> 1.89 direction reversals per second, 20% -> 48% of
    /// ticks hard against `PITCH_MIN`. Every sortie after it was the same
    /// shape (08-01: 57% and 2.42/s; 08-02: 48% and 2.27/s, ~0.9 s period,
    /// holding 7.0-10.8 about an 8 degree target). The structure was not the
    /// fault — the DAMPING term was, on both counts: a rate differenced from
    /// the attitude over a 0.25 s tick, and a 1 s lead on it.
    ///
    /// This is the test that could have caught it. The plant is second order
    /// with the loop's transport delay, so it CAN ring; the assertions are the
    /// two numbers the blackbox reports.
    #[test]
    fn the_rotation_settles_instead_of_ringing() {
        // Across the plant uncertainty — damping, standing nose-up moment, and
        // how long a REST write takes to reach the aircraft.
        for zeta in [0.35, 0.45, 0.55] {
            for m0 in [0.0, 20.0, 40.0] {
                for lag in [0.10, 0.20, 0.30] {
                    let (spread, sat) = fly_rotation(zeta, m0, lag, true);
                    assert!(
                        spread < 4.0,
                        "rings: {spread:.1} deg spread (zeta {zeta}, m0 {m0}, lag {lag})"
                    );
                    assert!(
                        sat < 0.05,
                        "camps on a stop: {:.0}% of ticks (zeta {zeta}, m0 {m0}, lag {lag})",
                        sat * 100.0
                    );
                }
            }
        }
        // The fallback — no measured rate, the law differences the attitude —
        // must cost accuracy, not stability. This is why PITCH_LEAD_S is 0.3
        // and not the 0.5 the measured path alone would allow: at 0.5 this
        // same sweep reaches a 14.6 degree spread with 31% saturation.
        for lag in [0.10, 0.20, 0.30] {
            let (spread, sat) = fly_rotation(0.45, 40.0, lag, false);
            assert!(spread < 6.0, "fallback rings: {spread:.1} deg (lag {lag})");
            assert!(sat < 0.05, "fallback camps: {:.0}%", sat * 100.0);
        }
        // And the shipped lead is on the flat part of the curve, not the edge
        // of a cliff: the whole failure was a constant sitting past one.
        let at_ship = fly_rotation(0.45, 40.0, 0.20, true).0;
        assert!(
            at_ship < 2.5,
            "the tuned point must be comfortable: {at_ship:.1} deg"
        );
    }

    /// The 737's rotation and hand-flown climb, rotate to the 300 ft engage,
    /// against `plant` (through the loop's transport delay) and a flight path
    /// with the aircraft's own excess thrust. The path and the [`PitchPlant`]
    /// were fitted to LEMG 13 (sortie 1790596335-full_auto, 2026-09-28; the
    /// hold is judged on the [`RatePlant`] refitted to EGKK 08R): the law pushing
    /// -0.37 to hold 11 degrees is a standing nose-up moment of about 130
    /// (6.25 x 11 + 167 x 0.37); 2,280 fpm at 184 kt is a 7 degree path at
    /// 11 degrees of pitch, so about 4 degrees of alpha there; 1.2 kt/s on
    /// that path is a thrust-to-weight excess of 0.185. Returns what the
    /// blackbox reports per `climb_manual` tick: the airspeed at 300 ft, the
    /// highest airspeed and attitude from liftoff to 300 ft, the highest
    /// tail margin (the attitude less the 11 degree strike plus the angle the
    /// height buys over the -800's 60 ft arm; positive is a strike), and the
    /// share of ticks the elevator
    /// sat on a stop.
    fn fly_initial_climb(
        hold: Option<ClimbHold>,
        mut plant: impl PitchModel,
        lag_s: f64,
    ) -> (f64, f64, f64, f64, f64) {
        const TW: f64 = 0.185;
        const KTS_S_PER_G: f64 = 9.81 / 0.514_444;
        let mut law = RotatePitch::with_authority(Some(11.0), -0.5).with_climb_hold(hold);
        let (mut ias, mut gamma, mut h_ft) = (145.5_f64, 0.0_f64, 0.0_f64);
        let mut airborne = false;
        let mut pending: Vec<(f64, f64)> = Vec::new();
        let mut applied = 0.0;
        let (mut t, dt) = (0.0_f64, 0.2_f64);
        let (mut max_ias, mut max_theta, mut max_theta_low) = (0.0_f64, 0.0_f64, f64::MIN);
        let (mut ticks, mut saturated) = (0.0, 0.0);
        let mut at_300 = None;
        // A tenth of a knot of read noise, deterministic.
        let mut seed = 7_u32;
        let mut noise = move || {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (f64::from(seed >> 8) / f64::from(1_u32 << 23) - 1.0) * 0.1
        };
        while t < 60.0 && at_300.is_none() {
            let cmd = law.step_climb(
                plant.theta(),
                Some(plant.q()),
                dt,
                Some((ias + noise(), h_ft)),
            );
            pending.push((t + lag_s, cmd));
            let (mut now, end) = (t, t + dt);
            while now < end - 1e-9 {
                let next = pending
                    .iter()
                    .map(|&(at, _)| at)
                    .filter(|&at| at > now && at < end)
                    .fold(end, f64::min);
                let span = next - now;
                plant.advance(applied, span);
                // On the wheels the attitude cannot go below the runway.
                if !airborne && plant.theta() < 0.0 {
                    plant.hold_on_runway();
                }
                let alpha = 4.0 * (184.0 / ias).powi(2);
                if !airborne && ias >= 160.0 && plant.theta() >= alpha {
                    airborne = true;
                }
                if airborne {
                    gamma += ((plant.theta() - alpha) - gamma) / 1.5 * span;
                    ias += (TW - gamma.to_radians().sin()) * KTS_S_PER_G * span;
                    h_ft += ias * 1.6878 * gamma.to_radians().sin() * span;
                } else {
                    // The recorded ground run: 145 to 162 in 4.4 s.
                    ias += 3.9 * span;
                }
                now = next;
                while let Some(i) = pending.iter().position(|&(at, _)| at <= now) {
                    applied = pending.remove(i).1;
                }
            }
            t = end;
            if airborne && h_ft > 8.0 {
                max_ias = max_ias.max(ias);
                max_theta = max_theta.max(plant.theta());
                let strike = 11.0 + (h_ft / 60.0).atan().to_degrees();
                max_theta_low = max_theta_low.max(plant.theta() - strike);
                ticks += 1.0;
                if cmd <= -0.5 + 1e-9 || cmd >= PITCH_MAX - 1e-9 {
                    saturated += 1.0;
                }
                if h_ft >= AP_ENGAGE_AGL_FT {
                    at_300 = Some(ias);
                }
            }
        }
        (
            at_300.expect("reaches the engage height"),
            max_ias,
            max_theta,
            max_theta_low,
            saturated / ticks,
        )
    }

    /// #78: the hand-flown segment holds V2 to V2 + 20, not the rotation
    /// attitude. The fixed 8 degrees reproduces LEMG 13 (162 at liftoff, 185
    /// at 300 ft on all five circuits); the hold keeps the 737 inside its
    /// band with the tail clear, across the plant uncertainty.
    #[test]
    fn the_initial_climb_holds_v2_plus_fifteen_by_pitch() {
        let b738 = crate::aircraft::load("boeing_737_800").expect("737 profile loads");
        let initial = b738
            .reference
            .as_ref()
            .and_then(|r| r.initial_climb_kts)
            .unwrap();
        let hold = ClimbHold::new(initial, b738.climb_pitch_max_deg);
        let h = hold.expect("the 737 declares its climb attitude");
        assert_eq!(h.hold_kts, 165.0, "V2 150 + 15");
        let (v2, top) = (initial - 20.0, initial);

        // The recorded flight, reproduced: the plant is the aircraft.
        let (at_300, ..) = fly_initial_climb(None, PitchPlant::new(0.45, 130.0), 0.2);
        assert!(at_300 > 178.0, "the old law runs away as the 737 did: {at_300:.1}");
        let fitted = RatePlant {
            theta: 0.0,
            q: 0.0,
            k: RatePlant::K,
            u0: RatePlant::U0,
            tau: RatePlant::TAU_S,
        };
        let (at_300, ..) = fly_initial_climb(None, fitted, RatePlant::DELAY_S);
        assert!(at_300 > 178.0, "and on the EGKK plant: {at_300:.1}");

        // The hold is flown on the plant refitted to its first real flight
        // (EGKK 08R, #78), across its authority, holding push and lag. The
        // LEMG plant above was fitted to a standing push alone: driven by
        // the column EGKK logged it flies 25 to 28 degrees against the
        // recorded 22.4 (6 degrees RMS), a pitch six times too quick for the
        // 737 it stands for, so it no longer judges the climb.
        let mut plants = Vec::new();
        for k in [10.0, RatePlant::K, 16.0] {
            for u0 in [-0.10, RatePlant::U0, -0.20] {
                for tau in [RatePlant::TAU_S, 0.25] {
                    plants.push((k, u0, tau));
                }
            }
        }
        for &(k, u0, tau) in &plants {
            for lag in [0.2, RatePlant::DELAY_S, 0.5] {
                let plant = RatePlant {
                    theta: 0.0,
                    q: 0.0,
                    k,
                    u0,
                    tau,
                };
                let (at_300, max_ias, max_theta, low, sat) = fly_initial_climb(hold, plant, lag);
                let tag = format!("k {k}, u0 {u0}, tau {tau}, lag {lag}");
                assert!(
                    (v2..=top).contains(&max_ias),
                    "peak {max_ias:.1} outside V2 {v2} to V2 + 20 ({tag})"
                );
                assert!(
                    (h.hold_kts - 5.0..=h.hold_kts + 5.0).contains(&at_300),
                    "at the engage {at_300:.1} vs hold {} ({tag})",
                    h.hold_kts
                );
                assert!(
                    max_theta <= h.pitch_max_deg,
                    "theta {max_theta:.2} over the ceiling ({tag})"
                );
                assert!(low < 0.0, "tail strike: {low:.1} deg past it ({tag})");
                assert!(sat < 0.05, "camps on a stop: {:.0}% ({tag})", sat * 100.0);
            }
        }
    }

    /// EGKK 08R, the first ClimbHold flight (#78), as the blackbox drew it:
    /// every `roll` tick from the rotation and every `climb_manual` tick to
    /// the 300 ft engage.
    struct EgkkTick {
        climb: bool,
        t: f64,
        theta: f64,
        q: f64,
        dt: f64,
        ias: f64,
        /// `climb_manual` only: the height and the column the law wrote.
        agl: f64,
        pitch: f64,
        /// The target as logged (a `roll` record logs the PREVIOUS tick's).
        pitch_tgt: f64,
    }

    fn egkk_ticks() -> (ClimbHold, Vec<EgkkTick>) {
        let recs: Vec<serde_json::Value> =
            include_str!("../bench/takeoffs/egkk08r_climb_hold.jsonl")
                .lines()
                .filter_map(|l| serde_json::from_str(l).ok())
                .collect();
        let f = |r: &serde_json::Value, k: &str| r[k].as_f64().unwrap_or(f64::NAN);
        let h = recs
            .iter()
            .find(|r| r["event"] == "climb_hold")
            .expect("climb_hold");
        let hold = ClimbHold {
            hold_kts: f(h, "hold_kts"),
            pitch_max_deg: f(h, "pitch_max_deg"),
        };
        let rotate_t = recs
            .iter()
            .find(|r| r["event"] == "rotate")
            .map(|r| f(r, "t"))
            .unwrap();
        let ticks = recs
            .iter()
            .filter(|r| {
                (r["event"] == "roll" && f(r, "t") >= rotate_t) || r["event"] == "climb_manual"
            })
            .map(|r| EgkkTick {
                climb: r["event"] == "climb_manual",
                t: f(r, "t"),
                theta: f(r, "theta"),
                q: f(r, "q"),
                dt: f(r, "dt"),
                ias: f(r, "ias"),
                agl: f(r, "agl"),
                pitch: f(r, "pitch"),
                pitch_tgt: f(r, "pitch_tgt"),
            })
            .collect();
        (hold, ticks)
    }

    /// The law the 737 flew (tail strike 11, push authority 0.5), with the
    /// EGKK hold.
    fn egkk_law(hold: ClimbHold, guarded: bool) -> RotatePitch {
        let law = RotatePitch::with_authority(Some(11.0), -0.5).with_climb_hold(Some(hold));
        if guarded {
            law
        } else {
            law.without_ceiling_guard()
        }
    }

    /// Steps the law over the recorded rotation (the roll ticks, open loop:
    /// they carry the attitude the aircraft had) and returns the column each
    /// wrote, (t, column). A roll record carries no height, so the one the
    /// law read is given back by the tail ceiling the NEXT record's target
    /// was held under; the last, by the first climb tick's.
    fn egkk_rotation(law: &mut RotatePitch, ticks: &[EgkkTick]) -> Vec<(f64, f64)> {
        let first_climb = ticks.iter().position(|k| k.climb).unwrap();
        (0..first_climb)
            .map(|i| {
                let k = &ticks[i];
                let agl = if ticks[i + 1].climb {
                    ticks[i + 1].agl
                } else {
                    let over = (ticks[i + 1].pitch_tgt - (11.0 - TAIL_STRIKE_MARGIN_DEG)).max(0.0);
                    TAIL_ARM_FT * over.to_radians().tan()
                };
                (
                    k.t,
                    law.step_climb(k.theta, Some(k.q), k.dt, Some((k.ias, agl))),
                )
            })
            .collect()
    }

    /// Flies the law from EGKK's liftoff against a [`RatePlant`] (`k`, `u0`,
    /// `tau`) through `delay_s`: the rotation replayed as recorded, then
    /// closed loop over the recorded airspeed and height to the 300 ft
    /// engage, then `extra_s` more on the air as last read. Returns the
    /// highest attitude and (t, theta) per tick.
    fn fly_egkk(
        guarded: bool,
        plant: (f64, f64, f64),
        delay_s: f64,
        extra_s: f64,
    ) -> (f64, Vec<(f64, f64)>) {
        let (hold, ticks) = egkk_ticks();
        let mut law = egkk_law(hold, guarded);
        let mut pending: Vec<(f64, f64)> = egkk_rotation(&mut law, &ticks)
            .into_iter()
            .map(|(t, u)| (t + delay_s, u))
            .collect();
        let air: Vec<&EgkkTick> = ticks.iter().filter(|k| k.climb).collect();
        let (k, u0, tau) = plant;
        let mut p = RatePlant {
            theta: air[0].theta,
            q: air[0].q,
            k,
            u0,
            tau,
        };
        let mut t = air[0].t;
        let mut applied = pending
            .iter()
            .filter(|(at, _)| *at <= t)
            .last()
            .map_or(0.0, |x| x.1);
        pending.retain(|(at, _)| *at > t);
        let end = air.last().unwrap().t + extra_s;
        let (mut max_theta, mut log) = (p.theta, Vec::new());
        let mut i = 0;
        while t < end {
            let r = air[i.min(air.len() - 1)];
            let dt = if i < air.len() { r.dt } else { 0.22 };
            let cmd = law.step_climb(p.theta, Some(p.q), dt, Some((r.ias, r.agl)));
            log.push((t, p.theta));
            pending.push((t + delay_s, cmd));
            let stop = t + dt;
            while t < stop - 1e-9 {
                let next = pending
                    .iter()
                    .map(|&(at, _)| at)
                    .filter(|&at| at > t && at < stop)
                    .fold(stop, f64::min);
                p.advance(applied, next - t);
                t = next;
                while let Some(j) = pending.iter().position(|&(at, _)| at <= t + 1e-9) {
                    applied = pending.remove(j).1;
                }
                max_theta = max_theta.max(p.theta);
            }
            i += 1;
        }
        (max_theta, log)
    }

    /// #78, EGKK 08R (sortie 1790601822-full_auto, 2026-09-28): the hold held
    /// its speed (162.6 to 170.3 kt to the engage) and the nose flew to 22.4
    /// degrees under an 18 degree ceiling. The TARGET never passed 18 (it
    /// stopped there at 77 ft); the rotation tracked its ramp within 0.6
    /// degrees. The attitude loop overshot: the nose arrived at 18 rising
    /// 2.6 degrees a second, and four degrees past it the law was pushing
    /// 0.18 on a 737 that needs 0.14 of push just to stop rotating.
    #[test]
    fn the_egkk_climb_stays_under_its_ceiling() {
        let (hold, ticks) = egkk_ticks();
        assert_eq!((hold.hold_kts, hold.pitch_max_deg), (165.0, 18.0));
        let recorded_max = ticks.iter().map(|k| k.theta).fold(f64::MIN, f64::max);
        assert!((recorded_max - 22.38).abs() < 0.01, "{recorded_max}");

        // The replay: the law without the guard, stepped over the recorded
        // ticks, wrote the column the blackbox logged, every climb tick.
        let mut law = egkk_law(hold, false);
        let mut column = egkk_rotation(&mut law, &ticks);
        for k in ticks.iter().filter(|k| k.climb) {
            let u = law.step_climb(k.theta, Some(k.q), k.dt, Some((k.ias, k.agl)));
            assert!(
                (u - k.pitch).abs() < 0.002,
                "t {}: replayed {u:.3} vs logged {:.3}",
                k.t,
                k.pitch
            );
            assert!(
                (law.target_deg() - k.pitch_tgt).abs() < 0.01,
                "target at t {}",
                k.t
            );
            assert!(
                law.target_deg() <= hold.pitch_max_deg,
                "the target never passed the ceiling"
            );
            column.push((k.t, k.pitch));
        }

        // The plant IS the aircraft: driven by that column through the
        // fitted delay from the liftoff tick, it flies the recorded attitude.
        let air: Vec<&EgkkTick> = ticks.iter().filter(|k| k.climb).collect();
        let mut p = RatePlant {
            theta: air[0].theta,
            q: air[0].q,
            k: RatePlant::K,
            u0: RatePlant::U0,
            tau: RatePlant::TAU_S,
        };
        let applied_at = |t: f64| {
            column
                .iter()
                .filter(|(at, _)| at + RatePlant::DELAY_S <= t + 1e-9)
                .last()
                .map_or(0.0, |x| x.1)
        };
        let (mut sq, mut worst, mut peak) = (0.0, 0.0_f64, 0.0_f64);
        for w in air.windows(2) {
            let mut t = w[0].t;
            while t < w[1].t - 1e-9 {
                let step = 0.005_f64.min(w[1].t - t);
                p.advance(applied_at(t), step);
                t += step;
            }
            let e = p.theta - w[1].theta;
            sq += e * e;
            worst = worst.max(e.abs());
            peak = peak.max(p.theta);
        }
        let rms = (sq / (air.len() - 1) as f64).sqrt();
        assert!(rms < 0.3, "fit RMS {rms:.2}");
        assert!(worst < 0.6, "fit worst {worst:.2}");
        assert!(
            (peak - recorded_max).abs() < 0.5,
            "fitted peak {peak:.2} vs {recorded_max}"
        );

        // Closed loop on the fitted plant: the old law reproduces the
        // overshoot, the guarded law keeps the nose under the ceiling.
        let fitted = (RatePlant::K, RatePlant::U0, RatePlant::TAU_S);
        let (old, _) = fly_egkk(false, fitted, RatePlant::DELAY_S, 0.0);
        assert!(
            old > 21.0,
            "the old law flies past the ceiling as the 737 did: {old:.2}"
        );
        let (new, log) = fly_egkk(true, fitted, RatePlant::DELAY_S, 10.0);
        assert!(new <= hold.pitch_max_deg, "guarded: {new:.2}");
        // Still a climb attitude, not a flattened one, by the engage.
        let engage_t = air.last().unwrap().t;
        let by_engage = log
            .iter()
            .filter(|(t, _)| *t <= engage_t)
            .map(|x| x.1)
            .fold(f64::MIN, f64::max);
        assert!(by_engage > 15.0, "climbs to its attitude: {by_engage:.2}");
        // And across the plant uncertainty: authority, the holding push, the
        // lag and the transport delay.
        for k in [10.0, 12.9, 16.0] {
            for u0 in [-0.10, -0.14, -0.20] {
                for tau in [0.1, 0.25, 0.4] {
                    for delay in [0.2, 0.35, 0.5] {
                        let (max, log) = fly_egkk(true, (k, u0, tau), delay, 10.0);
                        let tag = format!("k {k}, u0 {u0}, tau {tau}, delay {delay}");
                        assert!(
                            max <= hold.pitch_max_deg,
                            "{max:.2} over the ceiling ({tag})"
                        );
                        // No ring: the last four seconds move less than a degree.
                        let end = log.last().unwrap().0;
                        let tail: Vec<f64> = log
                            .iter()
                            .filter(|(t, _)| *t > end - 4.0)
                            .map(|x| x.1)
                            .collect();
                        let spread = tail.iter().cloned().fold(f64::MIN, f64::max)
                            - tail.iter().cloned().fold(f64::MAX, f64::min);
                        assert!(spread < 1.0, "rings: {spread:.2} ({tag})");
                    }
                }
            }
        }
    }

    /// Silent profiles keep the fixed attitude: no ceiling, no hold, and the
    /// law's target never passes the rotation attitude whatever the speed.
    #[test]
    fn an_airframe_without_a_climb_attitude_keeps_the_rotation_attitude() {
        assert_eq!(ClimbHold::new(74.0, None), None);
        assert_eq!(
            ClimbHold::new(74.0, Some(ROTATE_PITCH_DEG)),
            None,
            "no room above the rotation attitude"
        );
        let c172 = crate::aircraft::builtin("cessna_172sp").unwrap();
        assert_eq!(c172.climb_pitch_max_deg, None);
        let mut law = RotatePitch::with_authority(None, PITCH_MIN);
        let mut ias = 60.0;
        for _ in 0..100 {
            ias += 1.0;
            law.step_climb(8.0, Some(0.0), 0.2, Some((ias, 200.0)));
        }
        assert_eq!(law.target_deg(), ROTATE_PITCH_DEG);
    }

    /// #91: thrust comes back to the declared climb lever at the reduction
    /// height, airborne only, at a crew's pace; every other release (the
    /// swinging-nose hold, the overspeed guard) stays instant, and a silent
    /// airframe keeps takeoff thrust as it always did.
    #[test]
    fn thrust_reduction_at_the_declared_height() {
        let r = Some(crate::takeoff_config::ThrustReduction { agl_ft: 1000.0, lever: 0.9 });
        assert_eq!(thrust_reduced(1.0, true, 999.0, r), (1.0, false));
        assert_eq!(thrust_reduced(1.0, false, 1200.0, r), (1.0, false), "on the ground never");
        assert_eq!(thrust_reduced(1.0, true, 1000.0, r), (0.9, true));
        assert_eq!(thrust_reduced(0.7, true, 1500.0, r), (0.7, false), "the guard's lower lever stands");
        assert_eq!(thrust_reduced(1.0, true, 1500.0, None), (1.0, false));
        // Paced: from takeoff thrust to 0.9 over a second, never past it.
        let mut lever = 1.0;
        let mut steps = 0;
        while lever > 0.9 + 1e-9 {
            lever = reduction_slew(throttle_slew(lever, 0.9, 0.2), lever, 0.2);
            steps += 1;
            assert!(lever >= 0.9 - 1e-9);
        }
        assert_eq!(steps, 5, "0.1 of lever a second at 0.2 s ticks");
        // The takeoff's own releases are untouched by it.
        assert_eq!(throttle_slew(1.0, 0.5, 0.2), 0.5);
    }

    /// LFLL-EGLL, 2026-09-17. The 737 lifted at V2+20 and climbed out at
    /// 174 kt. Handed the profile's enroute `vy_kts` of 210 as its climb
    /// reference, the law read that as thirty-six knots slow, took 2,150 fpm
    /// off the rate and commanded 150 fpm — a jet at takeoff thrust told to
    /// stop climbing at 300 ft. It levelled at 1,030 ft and accelerated to
    /// 220 before the follower picked it up. The reference for the INITIAL
    /// climb is the initial climb speed.
    #[test]
    fn the_initial_climb_reference_is_the_speed_the_aircraft_actually_has() {
        // The 737-800's card: V2 150, initial climb 170, enroute Vy 210, and
        // the rate the arming tool hands down (80% of best, clamped: 1500).
        let cap = 1500.0;
        let at_liftoff = climb_vs_protected(174.0, 170.0, cap, Some(340.0));
        assert_eq!(at_liftoff, cap, "at the initial climb speed, full rate");
        // Against the enroute figure the SAME aircraft is floored — the bug,
        // kept here so the two numbers can never be conflated again.
        assert_eq!(
            climb_vs_protected(174.0, 210.0, cap, Some(340.0)),
            150.0,
            "the enroute climb speed floors a jet that is climbing perfectly well"
        );
        // Genuinely slow still trades rate for speed, jet or not.
        let slow = climb_vs_protected(150.0, 170.0, cap, Some(340.0));
        assert!(slow < cap && slow >= 150.0, "{slow}");
    }

    /// A takeoff that ends with the aircraft still configured for takeoff has
    /// not finished. The in-climb schedule waits for the climb speed, which a
    /// jet does not reach before the handoff: flaps 5 rode from Lyon to
    /// FL360 and on to Heathrow's top of descent, the whole climb sitting on
    /// their 250 kt placard (2026-09-17).
    #[test]
    fn the_handoff_never_leaves_the_takeoff_flaps_out() {
        // Already clean: nothing owed, nothing said.
        assert_eq!(clean_up_at_handoff(true, 181.0, Some(167.0)), None);
        // The 737 at its handoff: 181 kt against a 167 kt clean floor.
        assert_eq!(clean_up_at_handoff(false, 181.0, Some(167.0)), Some(true));
        // Too slow to clean up: the flaps STAY, and the caller says so — a
        // retraction here is the stall the schedule exists to avoid.
        assert_eq!(clean_up_at_handoff(false, 160.0, Some(167.0)), Some(false));
        // An airframe with no declared floor has nothing to judge against and
        // cleans up, exactly as it always did.
        assert_eq!(clean_up_at_handoff(false, 70.0, None), Some(true));
    }

    #[test]
    fn slew_limits_and_throttle_ramps() {
        // A demanded hard-over arrives in steps, not a slam.
        assert_eq!(slew(0.0, 0.7, 0.25), 0.25);
        assert_eq!(slew(0.25, 0.7, 0.25), 0.5);
        assert_eq!(slew(0.5, 0.55, 0.25), 0.55, "small changes pass through");
        assert!(
            (slew(0.2, -0.7, 0.25) + 0.05).abs() < 1e-9,
            "both directions"
        );
        // Throttle: rolling at 30% immediately, full by the ramp's end,
        // never over.
        assert_eq!(throttle_ramp(0.0), 0.3);
        let mid = throttle_ramp(THROTTLE_RAMP_S / 2.0);
        assert!(mid > 0.5 && mid < 0.9, "{mid}");
        // Gradual: no more than half the travel in the first two seconds.
        assert!(throttle_ramp(2.0) < 0.6, "{}", throttle_ramp(2.0));
        assert_eq!(throttle_ramp(THROTTLE_RAMP_S), 1.0);
        assert_eq!(throttle_ramp(60.0), 1.0);
    }

    #[test]
    fn climb_law_trades_rate_for_speed_never_descends() {
        // On speed: full rate.
        assert_eq!(climb_vs_fpm(74.0, 74.0, 700.0), 700.0);
        // 10 kt slow: backed off, still climbing.
        let slow = climb_vs_fpm(64.0, 74.0, 700.0);
        assert!((150.0..700.0).contains(&slow), "{slow}");
        // Absurdly slow: pinned at the floor, never a descent.
        assert_eq!(climb_vs_fpm(40.0, 74.0, 700.0), 150.0);
        // A request already UNDER the floor is granted, never a panic:
        // f64::clamp aborts the thread on min > max, and the follower's
        // level-off asked for 145.9 (LXGB to LEMG, 2026-08-02).
        assert_eq!(climb_vs_fpm(155.0, 105.0, 145.9), 145.9);
        assert_eq!(climb_vs_fpm(100.0, 105.0, 100.0), 100.0);
        assert_eq!(climb_vs_fpm(120.0, 105.0, 0.0), 0.0);
        // Fast: capped at the profile's rate, no zoom climb.
        assert_eq!(climb_vs_fpm(95.0, 74.0, 700.0), 700.0);
    }

    /// The climb protected only the SLOW side, which is right for a trainer
    /// (it cannot outrun itself at full power) and wrong for a jet, which can.
    /// The SF50 left Luton at full thrust and accelerated to 270 KIAS against
    /// a Vmo of 250 while the copilot placidly commanded 1500 fpm and never
    /// touched the lever (measured live in the climb, 2026-07-28).
    #[test]
    fn the_climb_protects_the_fast_side_too() {
        const VNE: f64 = 250.0;
        // Well below the guard band: untouched, both rate and thrust.
        assert_eq!(climb_vs_protected(200.0, 120.0, 2000.0, Some(VNE)), 2000.0);
        assert_eq!(climb_throttle(200.0, Some(VNE)), 1.0);
        // Inside the band the aircraft climbs HARDER — excess speed traded
        // for altitude — and the lever comes back with it.
        let fast = climb_vs_protected(245.0, 120.0, 2000.0, Some(VNE));
        assert!(fast > 2000.0, "trade the speed for rate: {fast}");
        let thr = climb_throttle(245.0, Some(VNE));
        assert!(thr < 1.0, "and pull the lever: {thr}");
        // Harder the faster it gets, monotonically, on both.
        assert!(climb_vs_protected(260.0, 120.0, 2000.0, Some(VNE)) > fast);
        assert!(climb_throttle(260.0, Some(VNE)) < thr);
        // The protection can never become a pitch-up or a shutdown of its own.
        let extreme = climb_vs_protected(400.0, 120.0, 2000.0, Some(VNE));
        assert!(
            extreme <= 2000.0 + OVERSPEED_VS_MAX_FPM,
            "capped: {extreme}"
        );
        assert!(climb_throttle(400.0, Some(VNE)) >= CLIMB_THROTTLE_MIN);
        // An airframe that declares no Vne keeps exactly the climb it had.
        assert_eq!(
            climb_vs_protected(260.0, 120.0, 2000.0, None),
            climb_vs_fpm(260.0, 120.0, 2000.0)
        );
        assert_eq!(climb_throttle(260.0, None), 1.0);
        // The 172 is nowhere near its Vne in a climb, so nothing changes for it.
        assert_eq!(
            climb_vs_protected(74.0, 74.0, 700.0, Some(163.0)),
            climb_vs_fpm(74.0, 74.0, 700.0)
        );
        assert_eq!(climb_throttle(74.0, Some(163.0)), 1.0);
    }

    #[test]
    fn the_climb_guard_is_the_250_kt_rule_on_a_jet_and_vne_on_a_trainer() {
        // A330: Vne 330, the band starts at 250 (270 - 20) not 310.
        assert_eq!(climb_guard_vne(Some(330.0)), Some(270.0));
        assert!(
            climb_throttle(273.0, climb_guard_vne(Some(330.0))) < 1.0,
            "273 kt at 1,000 ft is over"
        );
        assert_eq!(climb_throttle(240.0, climb_guard_vne(Some(330.0))), 1.0);
        // 172: Vne 163 stays the reference.
        assert_eq!(climb_guard_vne(Some(163.0)), Some(163.0));
        assert_eq!(climb_guard_vne(None), None);
    }

    #[test]
    fn runway_end_reject_fires_only_slow_and_short() {
        // 200 m left at 30 kt against a 55 kt Vr: no takeoff today.
        assert!(must_reject(200.0, 30.0, 55.0));
        // The Evolution's manual torque: below it the ramp, at it the lever
        // holds, past it the lever comes back; past 150 kt, the ramp again.
        let evo = crate::aircraft::TakeoffPowerProfile { torque_ftlb: 1250.0, until_kias: 150.0 };
        assert_eq!(torque_limited(0.8, 0.5, Some(900.0), 40.0, Some(&evo)), 0.8);
        assert_eq!(torque_limited(0.8, 0.5, Some(1200.0), 40.0, Some(&evo)), 0.5, "held from 95%");
        assert!((torque_limited(0.8, 0.5, Some(1260.0), 40.0, Some(&evo)) - 0.48).abs() < 1e-9);
        let back = torque_limited(0.8, 0.5, Some(1735.0), 40.0, Some(&evo));
        assert!((back - 0.5 * 1250.0 / 1735.0 * 0.98).abs() < 1e-9, "in proportion: {back}");
        assert_eq!(torque_limited(1.0, 0.5, Some(1400.0), 155.0, Some(&evo)), 1.0);
        assert_eq!(torque_limited(1.0, 0.5, None, 40.0, Some(&evo)), 1.0, "no read: the ramp");
        assert_eq!(torque_limited(1.0, 0.5, Some(2000.0), 40.0, None), 1.0, "no limit declared");
        // LILB 03 (2026-09-25): 11 m left at 30 kt is a reject there, not
        // 129 m left at 68 kt at the far end.
        assert_eq!(lateral_excursion(-11.1, -22.3, 29.9), Some("off_centreline"));
        assert_eq!(lateral_excursion(-4.0, -21.0, 21.4), Some("off_heading"));
        assert_eq!(lateral_excursion(-1.5, 14.1, 1.2), None, "the line-up's crawl");
        assert_eq!(lateral_excursion(-3.0, 5.0, 60.0), None);
        // 200 m left but nearly at Vr: keep going, rotation is imminent.
        assert!(!must_reject(200.0, 52.0, 55.0));
        // Plenty of runway, any speed: no reject.
        assert!(!must_reject(900.0, 20.0, 55.0));
    }

    // --- End-to-end sortie ---------------------------------------------------
    //
    // The law tests above each pin one function. This one CLOSES THE LOOP: a
    // coarse airframe model, driven tick-by-tick by the SAME control laws the
    // `fly` loop calls, with reads and control writes routed through a
    // `FakeXPlane` under the real dataref names. It is the automated stand-in
    // for a hand-flown takeoff — nothing else proves the laws COMPOSE into a
    // takeoff that tracks the centreline, rotates, and reaches the AP handoff.
    //
    // What it does NOT cover (by construction): the `fly` orchestration itself
    // (its gates, yields, and reject paths have their own unit tests above),
    // the lat/lon -> cross-track geodesy (autoland's tests own that — the model
    // holds cross-track directly), and real X-Plane physics (the model is a
    // deliberately crude Newtonian sketch). Its ONE job is to fail loudly if a
    // change to the laws stops the aircraft holding the runway — e.g. flipping
    // the `power_lead` sign, or breaking the trim integrator, lets the modelled
    // P-factor walk it off the strip and the cross-track assertion fires.

    /// A deliberately crude single-engine airframe, in a runway-local frame:
    /// heading and cross-track are relative to a course of 0 degrees, right
    /// positive. Not X-Plane — just enough coupling (thrust, a speed-tapered
    /// yaw authority, a left-yawing P-factor moment under power, a first-order
    /// pitch response, and lift past Vr) for the real laws to fly against.
    struct Airframe {
        gs_ms: f64,   // groundspeed
        along_m: f64, // distance rolled from the threshold
        xtk_m: f64,   // cross-track, right positive
        psi_deg: f64, // heading, right of course positive
        theta_deg: f64,
        agl_ft: f64,
    }

    impl Airframe {
        fn lined_up_nose_right() -> Self {
            // A realistic line-up: on the centreline but the nose 6 deg right,
            // the kind of small misalignment the roll law walks out.
            Self {
                gs_ms: 0.0,
                along_m: 0.0,
                xtk_m: 0.0,
                psi_deg: 6.0,
                theta_deg: 0.0,
                agl_ft: 0.0,
            }
        }
        fn ias_kts(&self) -> f64 {
            self.gs_ms * MS_TO_KTS // calm air: IAS == groundspeed
        }
        fn on_ground(&self) -> bool {
            self.agl_ft < 1.0
        }
        /// Advance one tick from the commanded controls (all as written to the
        /// plugin channels): throttle 0..1, steer -1..1 (right positive), pitch
        /// -1..1 (nose-up positive).
        fn step(&mut self, dt: f64, thr: f64, steer: f64, pitch: f64) {
            let gs_kts = self.gs_ms * MS_TO_KTS;
            // Thrust minus a little drag; never below a standstill.
            self.gs_ms = (self.gs_ms + (3.0 * thr - 0.01 * self.gs_ms) * dt).max(0.0);
            // Yaw: steering authority grows with airflow; P-factor yaws left
            // under power and fades as the airflow straightens. The rudder
            // feed-forward and trim integrator in the roll law exist to cancel
            // exactly this — get their sign wrong and the model diverges.
            let steer_yaw = steer * 30.0;
            let pfac = -7.0 * thr * (1.0 - 0.5 * (gs_kts / 60.0).clamp(0.0, 1.0));
            self.psi_deg += (steer_yaw + pfac) * dt;
            let psi_rad = self.psi_deg.to_radians();
            self.xtk_m += self.gs_ms * psi_rad.sin() * dt;
            self.along_m += self.gs_ms * psi_rad.cos() * dt;
            // Pitch: first-order lag toward the attitude the elevator commands.
            let cmd_theta = pitch * 15.0;
            self.theta_deg += (cmd_theta - self.theta_deg) * 2.0 * dt;
            // Lift builds once the nose is up and we're at flying speed — the
            // aircraft leaves the ground on its own, which is what lets the
            // liftoff latch (AGL > 8 ft) ever fire.
            let vs_fpm = if self.theta_deg > 2.0 && self.ias_kts() >= 52.0 {
                (self.theta_deg - 2.0) * 120.0 * (self.ias_kts() / 55.0).min(1.5)
            } else {
                0.0
            };
            self.agl_ft = (self.agl_ft + vs_fpm / 60.0 * dt).max(0.0);
        }
    }

    #[tokio::test]
    async fn sortie_flies_the_roll_rotates_and_reaches_the_ap_handoff() {
        use crate::xplane::FakeXPlane;
        use sim::xplane::XPlane;

        let sim = FakeXPlane::new();
        let course_deg = 0.0;
        let length_m = 2000.0;
        let vr_kts = 55.0;
        let dt = 0.15; // fly's nominal tick

        let mut af = Airframe::lined_up_nose_right();
        let mut steer_law = RollSteer::default();
        let mut sortie_prev_xtk: Option<f64> = None;
        let mut pitch_law = RotatePitch::default();

        // Loop state mirrored from `fly`.
        let mut rotated = false;
        let mut airborne = false;
        let mut last_thr_cmd = 0.0_f64;
        let mut last_err = 0.0_f64;

        // What we assert on.
        let mut max_abs_xtk = 0.0_f64;
        let mut max_abs_xtk_after_capture = 0.0_f64;
        let mut rotate_snapshot: Option<(f64, f64, f64)> = None; // (ias, remaining, err)
        let mut hit_full_throttle = false;
        let mut ticks = 0;

        loop {
            ticks += 1;
            assert!(ticks < 1200, "takeoff never converged (ran {ticks} ticks)");
            let elapsed = ticks as f64 * dt;

            // Publish the world exactly where `fly` reads it, under the real
            // dataref names, then read the flight state back through the trait.
            sim.set_read(GS_MS, af.gs_ms);
            sim.set_read(IAS, af.ias_kts());
            sim.set_read(AGL_M, af.agl_ft / M_TO_FT);
            sim.set_read(PSI_TRUE, af.psi_deg.rem_euclid(360.0));
            sim.set_read(THETA, af.theta_deg);
            sim.set_read(ON_GROUND, if af.on_ground() { 1.0 } else { 0.0 });
            let (gs_ms, ias, agl_m, psi) = tokio::join!(
                sim.get_float(GS_MS),
                sim.get_float(IAS),
                sim.get_float(AGL_M),
                sim.get_float(PSI_TRUE),
            );
            let (gs_ms, ias, agl_ft, psi) = (
                gs_ms.unwrap(),
                ias.unwrap(),
                agl_m.unwrap() * M_TO_FT,
                psi.unwrap(),
            );
            let gs_kts = gs_ms * MS_TO_KTS;

            // Throttle (mirrors `fly`: computed from the PREVIOUS tick's error).
            let thr_want = if airborne {
                1.0
            } else {
                throttle_cmd(elapsed, last_err, gs_kts)
            };
            let thr_cmd = throttle_slew(last_thr_cmd, thr_want, dt);
            sim.set_float(&throttle_dref(), thr_cmd).await.unwrap();
            last_thr_cmd = thr_cmd;
            hit_full_throttle |= thr_cmd >= 0.99;

            let remaining = length_m - af.along_m;
            if !airborne {
                assert!(remaining > 0.0, "rolled off the end without flying");
                assert!(
                    !must_reject(remaining, ias, vr_kts),
                    "the loop would have rejected here"
                );
            }

            // Steer the centreline (cross-track held by the model, not geodesy).
            let xtk_rate = sortie_prev_xtk.map_or(0.0, |p: f64| (af.xtk_m - p) / dt);
            sortie_prev_xtk = Some(af.xtk_m);
            let target = roll_target_heading(course_deg, af.xtk_m, xtk_rate, gs_kts);
            let err = ((target - psi + 540.0).rem_euclid(360.0)) - 180.0;
            last_err = err;
            let steer = steer_law.step(err, dt, gs_kts, thr_cmd);
            assert!(
                steer.abs() <= MAX_ROLL_STEER + 1e-9,
                "steer out of bounds: {steer}"
            );
            sim.set_float(TAXI_STEER, steer).await.unwrap();

            // Rotate, then hold the attitude with the elevator channel.
            if !rotated && ias >= vr_kts && rotate_ok(err, remaining) {
                rotated = true;
                rotate_snapshot = Some((ias, remaining, err));
            }
            let mut pitch = 0.0;
            if rotated {
                pitch = pitch_law.step(af.theta_deg, None, dt);
                sim.set_float(TAKEOFF_PITCH, pitch).await.unwrap();
            }

            // Liftoff latch, same gate as `fly`.
            if !airborne && !af.on_ground() && ias > vr_kts * 0.8 && agl_ft > 8.0 {
                airborne = true;
            }

            max_abs_xtk = max_abs_xtk.max(af.xtk_m.abs());
            if af.along_m > 200.0 {
                max_abs_xtk_after_capture = max_abs_xtk_after_capture.max(af.xtk_m.abs());
            }

            // The AP would take the established climb here; the sortie is done.
            if agl_ft >= AP_ENGAGE_AGL_FT {
                break;
            }

            // Advance the world from what we actually wrote to the channels.
            let steer_a = sim.last_write(TAXI_STEER).unwrap();
            let thr_a = sim.last_write(&throttle_dref()).unwrap();
            let pitch_a = if rotated {
                sim.last_write(TAKEOFF_PITCH).unwrap()
            } else {
                0.0
            };
            let _ = pitch;
            af.step(dt, thr_a, steer_a, pitch_a);
        }

        // It rotated, at or past Vr, with runway to spare and a straight nose.
        let (r_ias, r_rem, r_err) = rotate_snapshot.expect("never rotated");
        assert!(r_ias >= vr_kts, "rotated below Vr: {r_ias}");
        assert!(
            r_rem > 300.0,
            "rotated with the runway almost gone: {r_rem} m"
        );
        assert!(r_err.abs() < 15.0, "rotated with the nose {r_err} deg off");

        // It held the centreline the whole roll and tracked runway heading to
        // the handoff — no weave, no departure off the strip.
        assert!(
            max_abs_xtk < 15.0,
            "wandered {max_abs_xtk:.1} m off the centreline"
        );
        assert!(
            max_abs_xtk_after_capture < 6.0,
            "did not settle on the centreline: {max_abs_xtk_after_capture:.1} m after capture"
        );
        assert!(
            (af.psi_deg - course_deg).abs() < 8.0,
            "handed off {:.1} deg off runway heading",
            af.psi_deg - course_deg
        );
        assert!(hit_full_throttle, "never reached full power on the roll");

        // The trim integrator learned a right-rudder standing offset — the
        // proof it recognised and cancelled the modelled P-factor rather than
        // just chasing it. Flip `power_lead`'s sign and this (and the
        // cross-track bound) go red.
        assert!(
            steer_law.trim() > 0.0,
            "trim never learned the P-factor: {}",
            steer_law.trim()
        );
    }

    /// The 737's climb-out as the blackbox drew it at EGPH 24 (2026-09-21):
    /// 4.1 kt/s of excess thrust at takeoff power, a knot a second given back
    /// per 1,000 fpm, and a VS mode that is two 1.5 s lags behind 2 s of dead
    /// time. Returns (ias at the acceleration height, flap steps as (agl, ias,
    /// ratio), handoff (agl, ias), VS command reversals below the height).
    fn fly_737_climb_out(
        climb_out: Option<ClimbOut>,
    ) -> (f64, Vec<(f64, f64, f64)>, (f64, f64), usize) {
        let schedule = crate::aircraft::flap_schedule(
            crate::aircraft::load("boeing_737_800")
                .unwrap()
                .flaps
                .as_ref(),
        );
        let (hold_kts, base_fpm, handoff_agl, dt) = (170.0, 1500.0, 1000.0, 0.4);
        let vne = climb_guard_vne(Some(340.0));
        // As the AP took it: 340 ft, 185 kt, climbing 1,600 fpm.
        let (mut agl, mut ias, mut vs1, mut vs) = (340.0_f64, 185.0_f64, 1600.0_f64, 1600.0_f64);
        let mut delay = std::collections::VecDeque::from(vec![1600.0_f64; 5]);
        let (mut flap, mut steps, mut at_accel) = (0.375_f64, Vec::new(), None);
        let mut deployed = flap;
        let (mut reversals, mut last_cmd, mut last_dir) = (0usize, None::<f64>, 0.0_f64);
        let mut waited = None::<f64>;
        for tick in 0..400 {
            let t = tick as f64 * dt;
            let gate = climb_out.map_or(FLAP_RETRACT_AGL_FT, |c| c.accel_agl_ft);
            if flap > 1e-6 && agl > gate {
                if let Some(next) = paced_flap_step(&schedule, flap, Some(deployed), ias) {
                    flap = next;
                    steps.push((agl, ias, next));
                }
            }
            // The flaps follow the handle at about a detent in four seconds.
            deployed += (flap - deployed).clamp(-0.03 * dt, 0.03 * dt);
            let cmd = climb_out_vs_fpm(ias, hold_kts, base_fpm, vne, agl, climb_out);
            if climb_out.is_some_and(|c| agl < c.accel_agl_ft) {
                if let Some(prev) = last_cmd {
                    let dir = (cmd - prev).signum();
                    if (cmd - prev).abs() > 50.0 {
                        if dir == -last_dir {
                            reversals += 1;
                        }
                        last_dir = dir;
                    }
                }
                last_cmd = Some(cmd);
            }
            if at_accel.is_none() && agl >= 1000.0 {
                at_accel = Some(ias);
            }
            if agl >= handoff_agl {
                let w = t - *waited.get_or_insert(t);
                let retracted = flap <= 1e-6;
                if !(climb_out.is_some() && handoff_waits_for_schedule(retracted, true, w)) {
                    return (at_accel.unwrap(), steps, (agl, ias), reversals);
                }
            }
            delay.push_back(cmd);
            let seen = delay.pop_front().unwrap();
            vs1 += (seen - vs1) * dt / 1.5;
            vs += (vs1 - vs) * dt / 1.5;
            ias += (4.1 - 1.06 * vs / 1000.0) * dt;
            agl += vs / 60.0 * dt;
        }
        panic!("never handed off");
    }

    #[test]
    fn the_plant_flies_the_egph_climb_out_under_the_old_law() {
        // 240 kt through 1,000 ft and the flaps gone by 740: the flight.
        let (at_accel, steps, _, _) = fly_737_climb_out(None);
        assert!((230.0..255.0).contains(&at_accel), "{at_accel}");
        assert!(steps.iter().all(|(agl, _, _)| *agl < 1000.0), "{steps:?}");
    }

    #[test]
    fn a_jet_holds_its_climb_speed_to_the_acceleration_height() {
        let c = ClimbOut {
            accel_agl_ft: 1000.0,
            max_fpm: 3000.0,
        };
        let (at_accel, steps, (h_agl, h_ias), reversals) = fly_737_climb_out(Some(c));
        // Handed 185 kt at 340 ft with 3,000 fpm to spend against 3,900 of
        // excess: not V2 + 20, but thirty-five knots under what it was, and
        // still under the flaps-up speed, so the schedule has steps to take.
        assert!(at_accel < 210.0, "{at_accel}");
        assert!(reversals <= 1, "the speed hold hunted: {reversals}");
        // Flaps 5 stays to the height, then 1 and up, each at its own speed
        // and after the flaps have run to the last.
        assert_eq!(steps.len(), 2, "{steps:?}");
        assert!(steps.iter().all(|(agl, _, _)| *agl > 1000.0), "{steps:?}");
        assert!(steps[1].1 >= 210.0 && steps[1].2 <= 1e-6, "{steps:?}");
        // The handoff is clean, inside the wait, and nowhere near 250.
        assert!(h_ias < 235.0 && h_agl < 2500.0, "{h_agl} ft {h_ias} kt");
    }

    #[test]
    fn the_speed_hold_is_a_jets_below_its_height_and_nobody_elses() {
        let c = Some(ClimbOut {
            accel_agl_ft: 1000.0,
            max_fpm: 3000.0,
        });
        // On speed: the base rate. Ten over: a thousand more. Capped at best.
        assert_eq!(
            climb_out_vs_fpm(170.0, 170.0, 1500.0, None, 500.0, c),
            1500.0
        );
        assert_eq!(
            climb_out_vs_fpm(180.0, 170.0, 1500.0, None, 500.0, c),
            2500.0
        );
        assert_eq!(
            climb_out_vs_fpm(200.0, 170.0, 1500.0, None, 500.0, c),
            3000.0
        );
        // Slow is still slow: the rate is backed off, never added to.
        assert_eq!(
            climb_out_vs_fpm(160.0, 170.0, 1500.0, None, 500.0, c),
            900.0
        );
        // At the height, and on a trainer at any height, the old climb.
        assert_eq!(
            climb_out_vs_fpm(200.0, 170.0, 1500.0, None, 1000.0, c),
            1500.0
        );
        assert_eq!(
            climb_out_vs_fpm(90.0, 74.0, 560.0, None, 500.0, None),
            560.0
        );
        // A best rate under the base never LOWERS the climb.
        let low = Some(ClimbOut {
            accel_agl_ft: 1000.0,
            max_fpm: 0.0,
        });
        assert_eq!(
            climb_out_vs_fpm(200.0, 170.0, 1500.0, None, 500.0, low),
            1500.0
        );
    }

    #[test]
    fn a_circuit_keeps_the_old_climb_out_whole() {
        let _g = CLIMB_OUT_TEST.lock().unwrap_or_else(|e| e.into_inner());
        publish_climb_out(Some(ClimbOut {
            accel_agl_ft: 1000.0,
            max_fpm: 3000.0,
        }));
        assert!(climb_out_for(DEFAULT_HANDOFF_AGL_FT).is_some());
        assert!(climb_out_for(500.0).is_none());
        publish_climb_out(None);
        assert!(climb_out_for(DEFAULT_HANDOFF_AGL_FT).is_none());
    }

    static CLIMB_OUT_TEST: std::sync::Mutex<()> = std::sync::Mutex::new(());

    #[test]
    fn the_jets_declare_a_climb_out_and_the_trainer_does_not() {
        for (id, want) in [
            ("boeing_737_800", true),
            ("airbus_a330_300", true),
            ("cessna_172sp", false),
        ] {
            let Some(p) = crate::aircraft::load(id) else {
                continue;
            };
            assert_eq!(ClimbOut::from_profile(&p).is_some(), want, "{id}");
        }
    }

    /// #150, replayed: on both recorded LEMG 13 circuits the pitch was
    /// climbing 2,300 to 2,500 fpm through 300 ft and the engage wrote 1,500.
    /// The engage now writes the climb flown, a slew step from it at most,
    /// whether the law it walks to is the circuit's or a jet's speed hold.
    #[test]
    fn the_engage_takes_the_climb_the_pitch_was_flying() {
        let recorded = climb_out_bench::load();
        assert_eq!(recorded.len(), 2);
        let jet = Some(ClimbOut {
            accel_agl_ft: 1000.0,
            max_fpm: 3000.0,
        });
        let vne = climb_guard_vne(Some(340.0));
        let dt = 0.4;
        for r in &recorded {
            let flown = climb_out_bench::flown_fpm(&r.manual);
            let (_, agl, ias, cmd) = r.engage;
            assert!(
                flown - cmd > 700.0,
                "{}: flown {flown:.0}, the engage wrote {cmd:.0}",
                r.sortie
            );
            // The circuit's takeoff (no climb-out below 500 ft): the law is
            // the old rate, the command starts at the climb flown.
            let (law, first) =
                ClimbVs::default().step(ias, 170.0, 1500.0, vne, agl, None, Some(flown), dt);
            assert_eq!(law, 1500.0, "{}", r.sortie);
            assert!(
                (first - flown).abs() <= CLIMB_VS_SLEW_FPM_S * dt + 1e-6,
                "{}: flown {flown:.0}, first {first:.0}",
                r.sortie
            );
            // A departure's speed hold starts where the pitch is.
            let (law, first) =
                ClimbVs::default().step(ias, 170.0, 1500.0, vne, agl, jet, Some(flown), dt);
            assert!((law - flown).abs() < 1.0, "{}: law {law:.0}", r.sortie);
            assert!((first - flown).abs() < 1.0, "{}", r.sortie);
            // A failed read is no measurement: the law, as it always was.
            let (law, first) =
                ClimbVs::default().step(ias, 170.0, 1500.0, vne, agl, None, None, dt);
            assert_eq!((law, first), (1500.0, 1500.0));
        }
    }

    /// #150 with #78: the pitch holds V2 + 15 to 300 ft (165 kt, all the
    /// excess in the path: about 3,090 fpm on #78's LEMG 13 fit). Engaged on
    /// the old proportional law the dial read 1,200 (five knots slow) and the
    /// 737 ran to the acceleration height well past V2 + 20; the speed hold
    /// seeded at the climb flown keeps the band through the 737's VS mode
    /// (2 s dead, two 1.5 s lags), then lowers the nose at the height at a
    /// crew's pace.
    #[test]
    fn a_jet_takes_the_held_climb_and_keeps_its_speed_to_the_height() {
        let (tw, dt) = (0.185, 0.4);
        let c = Some(ClimbOut {
            accel_agl_ft: 1000.0,
            max_fpm: 3000.0,
        });
        let vne = climb_guard_vne(Some(340.0));
        let fly = |new: bool| {
            let (mut agl, mut ias) = (300.0_f64, 165.0_f64);
            let flown = climb_out_bench::held_climb_fpm(tw, ias);
            let mut ap = climb_out_bench::VsMode::new(flown, dt);
            let mut law = ClimbVs::default();
            let (mut prev, mut max_step) = (flown, 0.0_f64);
            let (mut lo, mut hi, mut at_height) = (ias, ias, None);
            let mut reversals = (0usize, 0.0_f64);
            while agl < 1300.0 {
                let cmd = if new {
                    law.step(ias, 170.0, 1500.0, vne, agl, c, Some(ap.vs), dt).1
                } else {
                    climb_out_vs_fpm(ias, 170.0, 1500.0, vne, agl, c)
                };
                let step = cmd - prev;
                max_step = max_step.max(step.abs());
                if agl < 1000.0 && step.abs() > 20.0 {
                    if step.signum() == -reversals.1 {
                        reversals.0 += 1;
                    }
                    reversals.1 = step.signum();
                }
                prev = cmd;
                let vs = ap.step(cmd, dt);
                ias += climb_out_bench::accel_kts_s(tw, ias, vs) * dt;
                agl += vs / 60.0 * dt;
                if agl < 1000.0 {
                    lo = lo.min(ias);
                    hi = hi.max(ias);
                } else if at_height.is_none() {
                    at_height = Some(ias);
                }
            }
            (max_step, lo, hi, at_height.unwrap(), reversals.0)
        };
        let (old_step, _, old_hi, old_at, _) = fly(false);
        assert!(old_step > 1500.0, "the old engage pushes: {old_step:.0}");
        assert!(
            old_hi > 185.0 && old_at > 185.0,
            "the old law runs away: {old_hi:.1}"
        );
        let (step, lo, hi, at, reversals) = fly(true);
        assert!(
            step <= CLIMB_VS_SLEW_FPM_S * dt + 1e-6,
            "the command stepped {step:.0} fpm in a tick"
        );
        assert!(
            lo >= 160.0 && hi <= 180.0,
            "{lo:.1} to {hi:.1} below the height"
        );
        assert!((160.0..=180.0).contains(&at), "at the height {at:.1}");
        assert!(reversals <= 1, "the speed hold hunted: {reversals}");
    }

    /// #150: the circuit takes the aircraft at 500 ft and walks the schedule
    /// itself; a departure (or a clean airframe, or one with no schedule)
    /// still cleans up at the handoff as it always did. The window leaves V2
    /// for the flaps-up speed a crew bugs, only where the takeoff set V2.
    #[tokio::test]
    async fn the_circuit_takes_the_flaps_and_the_window_leaves_v2() {
        assert!(leaves_flaps_to_caller(false, true, true));
        assert!(!leaves_flaps_to_caller(true, true, true), "clean already");
        assert!(
            !leaves_flaps_to_caller(false, false, true),
            "a departure cleans up"
        );
        assert!(
            !leaves_flaps_to_caller(false, true, false),
            "no schedule to walk"
        );

        let b738 = crate::aircraft::load("boeing_737_800").expect("737 profile loads");
        let sched = crate::aircraft::flap_schedule(b738.flaps.as_ref());
        assert_eq!(
            clean_up_speed_kts(&sched, 210.0),
            210.0,
            "the 737's flaps-up speed"
        );
        assert_eq!(
            clean_up_speed_kts(&[], 74.0),
            74.0,
            "no schedule: the climb speed"
        );
        assert!(
            b738.takeoff_config
                .as_ref()
                .and_then(|t| t.v2_kts)
                .is_some(),
            "the 737's takeoff sets V2, so its window is ours to move"
        );

        use crate::xplane::FakeXPlane;
        const WINDOW: &str = crate::aircraft::TakeoffConfigProfile::SPEED_WINDOW;
        let xp = FakeXPlane::with_reads(&[(WINDOW, 150.0)]);
        // The fake never takes a write: written, and read back as not taken.
        assert!(!speed_window_after_acceleration(&xp, 210.0).await);
        assert_eq!(xp.last_write(WINDOW), Some(210.0));
        xp.set_read(WINDOW, 210.0);
        assert!(speed_window_after_acceleration(&xp, 210.0).await);
    }

    #[test]
    fn a_brake_steered_roll_uses_the_brakes_until_the_rudder_bites() {
        // EDLP 24: full left steer at 10-21 kt with nothing answering.
        let (l, r) = super::roll_brake_split(-0.85, 12.0);
        assert!(l > 0.2 && r == 0.0, "inside (left) brake: {l} {r}");
        let (l30, _) = super::roll_brake_split(-0.85, 30.0);
        assert!((l30 - 0.5 * l).abs() < 1e-9, "half by 30 kt: {l30}");
        assert_eq!(super::roll_brake_split(-0.85, 45.0), (0.0, 0.0), "the rudder's above 40");
        assert_eq!(super::roll_brake_split(0.02, 12.0), (0.0, 0.0), "no brake for a breath of steer");
    }

    #[test]
    fn a_crooked_roll_start_pivots_on_the_inside_brake() {
        // LIME 28: 21 degrees right of the nose at 1 kt, steer 0.17, no brake.
        let (l, r) = super::roll_brake_split_aligning(0.17, 0.9, 21.0, 0.0);
        // The soft aligning PD (#46): 21 of 40 degrees of led error.
        assert!(r > 0.2 && l == 0.0, "right brake, from a standstill: {l} {r}");
        // Proportional: small errors get less.
        let (_, r6) = super::roll_brake_split_aligning(0.17, 0.9, 6.0, 0.0);
        assert!(r6 < r, "{r6} < {r}");
        // Aligned: the ordinary split, nothing added.
        assert_eq!(
            super::roll_brake_split_aligning(0.17, 0.9, 2.0, 0.0),
            super::roll_brake_split(0.17, 0.9)
        );
        // Already turning right fast: the rate ceiling lets go.
        assert_eq!(super::roll_brake_split_aligning(0.17, 0.9, 21.0, 30.0).1, 0.0);
        // Rolling fast: the rudder's, not a pivot.
        assert_eq!(
            super::roll_brake_split_aligning(0.17, 16.0, 21.0, 0.0),
            super::roll_brake_split(0.17, 16.0)
        );
    }

    /// The roll's power waits for the nose on a brake-steered airframe. The
    /// first roll ticks of 2026-09-29: the SR22 at LEMG 12 10.8 degrees off
    /// at 0.07 kt (1790690995, t 1790692260.5), the lever then walked 0.3 to
    /// 0.5 through a +/-12 degree pirouette; the Evolution at LILB 03 24.6 off
    /// at 2.6 kt (1790712974, t 1790713247.3), 0.32 to 0.59 through a swing
    /// to -4.4 at 21.6 deg/s. Runway starts roll 0.3 to 0.5 off: no wait.
    #[test]
    fn a_crooked_roll_start_holds_the_power_until_the_nose_is_straight() {
        use super::{aligning_throttle, RollAlign, RollAlignState};
        let mut a = RollAlign::default();
        assert_eq!(a.step(10.8, 0.1, 0.07, 0.15), RollAlignState::Hold);
        assert_eq!(aligning_throttle(0.41, true), 0.3, "the pivot's power, not the ramp's");
        let mut b = RollAlign::default();
        assert_eq!(b.step(24.6, -4.8, 2.6, 0.15), RollAlignState::Hold);
        // Straight and steady: the power comes up, and never waits again.
        assert_eq!(a.step(1.2, 0.5, 0.3, 0.4), RollAlignState::Go);
        assert_eq!(a.step(12.0, 20.0, 9.0, 0.4), RollAlignState::Go);
        assert_eq!(aligning_throttle(0.8, false), 0.8);
        // A runway start is straight at once.
        let mut c = RollAlign::default();
        assert_eq!(c.step(0.4, 0.0, 0.0, 0.15), RollAlignState::Go);
        assert!(c.held_s() == 0.0 && c.done());
        // Rolling already (past a walking pace): the roll law's.
        let mut d = RollAlign::default();
        assert_eq!(d.step(8.0, 0.0, 7.0, 0.15), RollAlignState::Go);
        // A nose that will not come round: a reject, said, after the bound.
        let mut e = RollAlign::default();
        let mut t = 0.0;
        let end = loop {
            let s = e.step(10.8, 0.0, 0.1, 0.42);
            if s != RollAlignState::Hold {
                break s;
            }
            t += 0.42;
        };
        assert_eq!(end, RollAlignState::Failed);
        assert!((t - super::ROLL_ALIGN_MAX_S).abs() < 0.5, "{t}");
    }

    /// The roll start closed on the fitted plant ([`crate::autotaxi::yaw_plant`]):
    /// the aligning split at the pivot's power from the SR22's 10.8 and the
    /// Evolution's 24.6 degrees lets the power up with the nose straight and
    /// swings no more than 2 degrees through. As flown the SR22 swung to
    /// -11.6 in 3 s and the Evolution to -4.4.
    #[test]
    fn a_crooked_roll_start_comes_straight_before_the_power() {
        use crate::autotaxi::yaw_plant::{signed, Plant};
        use super::{RollAlign, RollAlignState};
        for e0 in [10.8, 24.6] {
            for tick in [0.3, 0.42, 0.5] {
                let mut p = Plant::new(140.0, 3.0, e0, 0.0);
                let mut a = RollAlign::default();
                let mut through = 0.0_f64;
                let mut t = 0.0;
                loop {
                    match a.step(p.err, p.yaw, 1.0, tick) {
                        RollAlignState::Hold => {}
                        RollAlignState::Go => break,
                        RollAlignState::Failed => panic!("{e0} at {tick}: never straight"),
                    }
                    let lr = super::roll_brake_split_aligning(0.1, 1.0, p.err, p.yaw);
                    p.run(signed(lr), 1.0, tick);
                    through = through.max(-p.err * e0.signum());
                    t += tick;
                }
                assert!(p.err.abs() <= crate::autotaxi::ALIGNED_DEG, "{e0} at {tick}: {}", p.err);
                assert!(through <= 2.0, "{e0} at {tick}: {through:.1} through");
                assert!(t < 12.0, "{e0} at {tick}: {t:.1} s");
            }
        }
    }

    /// LILB 03 Evolution (1790732094, 2026-09-30), the roll's first 18 ticks:
    /// (dt, err to the target, yaw deg/s, gs kt, lever, bl, br) as logged.
    /// Held at 0.3 of lever the PT6 ran it from 0.2 to 6.6 kt; the 6 kt gate
    /// let the power up at err 4.39 (`roll_aligned` held_s 5.06), and the
    /// roll rejected at 68 kt, 9.4 m off.
    #[rustfmt::skip]
    const LILB_ROLL_HOLD: &[(f64, f64, f64, f64, f64, f64, f64)] = &[
        (0.150, 16.59, -0.31, 0.16, 0.30, 0.000, 0.194), (0.329, 16.36, 1.64, 0.04, 0.30, 0.000, 0.147),
        (0.433, 15.58, 1.87, 0.08, 0.30, 0.000, 0.133), (0.338, 14.83, 2.53, 0.16, 0.30, 0.000, 0.110),
        (0.456, 13.40, 3.80, 0.29, 0.30, 0.000, 0.065), (0.323, 11.97, 4.99, 0.40, 0.30, 0.000, 0.022),
        (0.427, 9.60, 5.89, 0.72, 0.30, 0.025, 0.000), (0.301, 8.02, 4.54, 1.11, 0.30, 0.012, 0.000),
        (0.442, 6.59, 2.22, 1.86, 0.30, 0.000, 0.024), (0.327, 6.06, 1.47, 2.46, 0.30, 0.000, 0.035),
        (0.411, 5.52, 1.32, 3.23, 0.30, 0.000, 0.032), (0.315, 5.16, 1.12, 3.85, 0.30, 0.000, 0.033),
        (0.425, 4.78, 0.81, 4.74, 0.30, 0.000, 0.035), (0.383, 4.55, 0.52, 5.59, 0.30, 0.000, 0.039),
        (0.429, 4.39, 0.36, 6.56, 0.30, 0.000, 0.041), (0.313, 4.32, 0.35, 7.28, 0.38, 0.000, 0.041),
        (0.403, 4.18, 0.44, 8.24, 0.43, 0.000, 0.037), (0.326, 4.10, 0.20, 9.10, 0.47, 0.000, 0.042),
    ];

    /// The hold holds the aircraft: through the logged ticks it never lets
    /// the power up on speed; over the creep the lever is idle and both
    /// brakes on; held past a runaway it is a reject, never a roll.
    #[test]
    fn a_roll_hold_holds_the_aircraft_and_never_lets_go_on_speed() {
        use super::{RollAlign, RollAlignState};
        let mut a = RollAlign::default();
        let mut braked_from = None;
        for &(dt, err, yaw, gs, thr, _, _) in LILB_ROLL_HOLD {
            let (cap, brake) = a.creep(gs);
            let s = a.step(err, yaw, gs, dt);
            assert_eq!(s, RollAlignState::Hold, "at {gs} kt, err {err}: the flown law said Go at 6.56");
            if gs > 3.0 {
                assert_eq!(cap, 0.0, "over the creep: idle, not {thr}");
                assert!(brake >= super::ROLL_ALIGN_BRAKE_MIN, "{gs} kt: {brake}");
                braked_from.get_or_insert(gs);
            } else if gs <= 2.0 {
                assert_eq!(brake, 0.0);
            }
        }
        assert!(braked_from.unwrap() < 3.3, "{braked_from:?}");
        // Back under the creep: the pivot's power, brakes off.
        assert_eq!(a.creep(1.9), (super::ROLL_PIVOT_THR, 0.0));
        // Held past a runaway all the same: a reject on the ground.
        let mut r = RollAlign::default();
        assert_eq!(r.step(10.0, 0.0, 1.0, 0.3), RollAlignState::Hold);
        assert_eq!(r.step(10.0, 0.0, super::ROLL_ALIGN_RUNAWAY_KTS + 0.5, 0.3), RollAlignState::Failed);
        // The brake rides on the pivot's split, both wheels.
        let (l, rr) = super::with_hold_brake((0.0, 0.1), 0.2);
        assert!((l - 0.2).abs() < 1e-9 && (rr - 0.3).abs() < 1e-9);
        assert_eq!(super::aligning_throttle_capped(0.9, true, 0.0), 0.0);
        assert_eq!(super::aligning_throttle_capped(0.9, false, 0.0), 0.9);
    }

    /// Closed on the yaw plant ([`crate::autotaxi::yaw_plant`]) from the
    /// logged roll start (16.6 off, 0.16 kt), with the Evolution's speed as
    /// flown (0.3 of lever built 1.2 kt/s, idle thrust 0.2 kt/s) and the
    /// propeller's standing yaw (0.03 of brake left, what held the logged
    /// creep 4 to 5 off): the hold creeps under 3.5 kt, trims the standing
    /// yaw out and lets the power up straight inside the bound. The flown
    /// law on the same plant let it up at 6 kt still off.
    #[test]
    fn a_roll_hold_creeps_the_nose_straight_before_the_power() {
        use crate::autotaxi::yaw_plant::{signed, Plant};
        use super::{RollAlign, RollAlignState};
        const BIAS: f64 = -0.03;
        let speed = |gs: f64, lever: f64, both: f64, dt: f64| {
            (gs + (lever * 4.0 + 0.2 - both * 6.0) * dt).max(0.0)
        };
        for tick in [0.3, 0.42] {
            let mut p = Plant::new(140.0, 3.0, 16.59, -0.31);
            let mut a = RollAlign::default();
            let (mut gs, mut lever, mut t, mut peak) = (0.16_f64, 0.3_f64, 0.0_f64, 0.0_f64);
            let end = loop {
                let (cap, both) = a.creep(gs);
                match a.step(p.err, p.yaw, gs, tick) {
                    RollAlignState::Hold => {}
                    other => break other,
                }
                lever = super::throttle_slew(lever, cap.min(super::ROLL_PIVOT_THR), tick);
                let split = super::roll_brake_split_aligning(0.1, gs, p.err, p.yaw);
                let split = a.trimmed(split, p.err, p.yaw, gs, tick);
                let (l, r) = super::with_hold_brake(split, both);
                p.run(signed((l, r)) + BIAS, (gs / 2.0).clamp(0.3, 1.0), tick);
                gs = speed(gs, lever, both, tick);
                peak = peak.max(gs);
                t += tick;
            };
            assert_eq!(end, RollAlignState::Go, "{tick}: {t:.1} s, err {:.1}", p.err);
            assert!(p.err.abs() <= crate::autotaxi::ALIGNED_DEG, "{tick}: {:.1}", p.err);
            assert!(peak < 3.5, "{tick}: crept to {peak:.1} kt");
            // The flown law: 0.3 of lever to 6 kt, the pivot alone.
            let mut p = Plant::new(140.0, 3.0, 16.59, -0.31);
            let (mut gs, mut t) = (0.16_f64, 0.0_f64);
            while gs < 6.0 && t < 20.0 {
                let split = super::roll_brake_split_aligning(0.1, gs, p.err, p.yaw);
                p.run(signed(split) + BIAS, (gs / 2.0).clamp(0.3, 1.0), tick);
                gs = speed(gs, 0.3, 0.0, tick);
                t += tick;
            }
            assert!(p.err.abs() > crate::autotaxi::ALIGNED_DEG, "{tick}: flown law {:.1}", p.err);
        }
    }

    #[test]
    fn a_jet_takes_no_propeller_power_lead() {
        // LOWI 26 (2026-09-24): full power, straight, parked: the propeller
        // law leads right rudder, the jet law nothing.
        let mut prop = super::RollSteer::default();
        let mut jet = super::RollSteer { jet: true, ..Default::default() };
        let (mut p, mut j) = (0.0, 0.0);
        for _ in 0..20 {
            p = prop.step(0.0, 0.2, 5.0, 1.0);
            j = jet.step(0.0, 0.2, 5.0, 1.0);
        }
        assert!(p > 0.05, "prop leads right: {p}");
        assert!(j.abs() < 1e-9, "jet leads nothing: {j}");
    }

    /// A sim whose frames run on while the Web API answers: every request is
    /// served `frames_per_request` frames after the one before it, as the
    /// API serves each request from whichever flight-loop frame it lands in.
    /// The aircraft flies `truth` (time s, along m, cross-track m right of
    /// `course`) from `thr`. `plugin` is a plugin that publishes
    /// `xcopilot/position`, copied in the frame its request lands in.
    struct FrameSim {
        thr: (f64, f64),
        course: f64,
        truth: Vec<(f64, f64, f64)>,
        t0: std::sync::Mutex<f64>,
        frame: std::sync::atomic::AtomicU64,
        frames_per_request: u64,
        plugin: bool,
    }

    const SIM_FPS: f64 = 30.0;

    impl FrameSim {
        /// The aircraft at time `t`: (lat, lon), groundspeed m/s.
        fn at(&self, t: f64) -> ((f64, f64), f64) {
            let k = self
                .truth
                .windows(2)
                .position(|w| t < w[1].0)
                .unwrap_or(self.truth.len() - 2);
            let (a, b) = (self.truth[k], self.truth[k + 1]);
            let f = ((t - a.0) / (b.0 - a.0)).clamp(0.0, 1.5);
            let along = a.1 + f * (b.1 - a.1);
            let xtk = a.2 + f * (b.2 - a.2);
            let gs = (b.1 - a.1) / (b.0 - a.0);
            let on_line = project_latlon(self.thr.0, self.thr.1, self.course, along / M_PER_NM);
            let right = (self.course + 90.0).rem_euclid(360.0);
            (project_latlon(on_line.0, on_line.1, right, xtk / M_PER_NM), gs)
        }
        /// The time the next request is served at.
        fn next_t(&self) -> f64 {
            let f = self
                .frame
                .fetch_add(self.frames_per_request, std::sync::atomic::Ordering::SeqCst);
            *self.t0.lock().unwrap() + f as f64 / SIM_FPS
        }
        /// Start a tick's reads at `t`.
        fn tick_at(&self, t: f64) {
            *self.t0.lock().unwrap() = t;
            self.frame.store(0, std::sync::atomic::Ordering::SeqCst);
        }
    }

    #[async_trait::async_trait]
    impl crate::xplane::XPlane for FrameSim {
        async fn get_float(&self, name: &str) -> anyhow::Result<f64> {
            let ((lat, lon), gs) = self.at(self.next_t());
            Ok(match name {
                LAT => lat,
                LON => lon,
                PSI_TRUE => self.course,
                GS_MS => gs,
                _ => 0.0,
            })
        }
        async fn get_bytes(&self, name: &str) -> anyhow::Result<Vec<u8>> {
            anyhow::ensure!(
                self.plugin && name == sim::position::POSITION,
                "dataref not found: {name}"
            );
            let t = self.next_t();
            let ((lat, lon), gs) = self.at(t);
            Ok(sim::position::encode(&sim::position::Position {
                lat,
                lon,
                elev_m: 0.0,
                psi_true_deg: self.course,
                gs_ms: gs,
                vs_fpm: 0.0,
                agl_m: 0.0,
                frame: Some((t * SIM_FPS).round() as u64),
                sim_time_s: Some(t),
                source: sim::position::Source::Plugin,
            }))
        }
        async fn set_float(&self, _: &str, _: f64) -> anyhow::Result<()> {
            Ok(())
        }
        async fn set_int(&self, _: &str, _: i64) -> anyhow::Result<()> {
            Ok(())
        }
        async fn run_command(&self, _: &str) -> anyhow::Result<()> {
            Ok(())
        }
        async fn run_command_for(&self, _: &str, _: f64) -> anyhow::Result<()> {
            Ok(())
        }
        async fn get_float_array(&self, _: &str) -> anyhow::Result<Vec<f64>> {
            Ok(vec![])
        }
        async fn set_int_array_elem(&self, _: &str, _: usize, _: i64) -> anyhow::Result<()> {
            Ok(())
        }
        async fn set_float_array_elem(&self, _: &str, _: usize, _: f64) -> anyhow::Result<()> {
            Ok(())
        }
    }

    /// #157: latitude and longitude read as two requests pair two frames. The
    /// LEMG 13 roll of 1790591256 (every `roll` tick, its groundspeed and
    /// cross-track as the truth, on 13's diagonal) is flown through a sim
    /// that serves each request two frames after the last (30 fps), and each
    /// tick is read the way the roll reads it. Split, the cross-track the law
    /// is handed is off the aircraft's own by up to 3.0 m above 120 kt (the
    /// 2.9 m flown), a different error every tick; from the plugin's
    /// one-frame snapshot it is the aircraft's own, every tick. On EGKK 08R's
    /// east-west strip the same lag is under half of it.
    #[tokio::test]
    async fn one_frames_position_never_pairs_two_frames() {
        async fn worst(
            ticks: &[serde_json::Value],
            course: f64,
            plugin: bool,
        ) -> (f64, sim::position::Source) {
            // Along-track from the logged groundspeed, the logged
            // cross-track as the aircraft's.
            let mut truth = Vec::new();
            let (mut along, mut last_t) = (0.0, None::<f64>);
            for r in ticks {
                let t = r["t"].as_f64().unwrap();
                let gs_ms = r["gs"].as_f64().unwrap() / MS_TO_KTS;
                if let Some(t0) = last_t {
                    along += gs_ms * (t - t0);
                }
                last_t = Some(t);
                truth.push((t, along, r["xtk_m"].as_f64().unwrap()));
            }
            let sim = FrameSim {
                thr: (36.6893, -4.5147),
                course,
                truth: truth.clone(),
                t0: std::sync::Mutex::new(0.0),
                frame: std::sync::atomic::AtomicU64::new(0),
                frames_per_request: 2,
                plugin,
            };
            let rwy = Runway {
                icao: "TEST".into(),
                id: "13".into(),
                thr_lat: sim.thr.0,
                thr_lon: sim.thr.1,
                course_deg: course,
                length_m: 3200.0,
                surface: 1,
                dist_nm: 0.0,
                elev_ft: 0.0,
            };
            let (mut worst, mut source) = (0.0_f64, sim::position::Source::Split);
            for (r, (t, _, _)) in ticks.iter().zip(&truth) {
                if r["gs"].as_f64().unwrap() < 120.0 {
                    continue;
                }
                sim.tick_at(*t);
                let pos = sim::position::read(&sim).await.unwrap();
                source = pos.source;
                let read_m = cross_track_nm(pos.lat, pos.lon, &rwy) * M_PER_NM;
                // The aircraft's own cross-track in the frame the reading
                // began in.
                let ((lat, lon), _) = sim.at(*t);
                let own_m = cross_track_nm(lat, lon, &rwy) * M_PER_NM;
                worst = worst.max((read_m - own_m).abs());
            }
            (worst, source)
        }
        let rolls = rolls_0928();
        let (_, lemg) = rolls
            .iter()
            .find(|(id, _)| id.starts_with("1790591256"))
            .expect("the LEMG 13 roll");
        // LEMG 13 true course by the rolls' heading on the line.
        let (split, src) = worst(lemg, 132.2, false).await;
        assert_eq!(src, sim::position::Source::Split);
        assert!(
            split > 1.5,
            "split reads should pair two frames on the diagonal: {split:.2} m"
        );
        let (snap, src) = worst(lemg, 132.2, true).await;
        assert_eq!(src, sim::position::Source::Plugin);
        assert!(
            snap < 0.005,
            "one frame's position is the aircraft's: {snap:.4} m"
        );
        // The same lag on EGKK 08R's course: under half the diagonal's
        // (1.2 m against 3.0 here).
        let (east_west, _) = worst(lemg, 77.6, false).await;
        assert!(
            east_west < 0.5 * split,
            "{east_west:.2} m east-west vs {split:.2} diagonal"
        );
    }
}

/// The pilot's reject at LEMG 13 (2026-09-28, sortie 1790627884-full_auto,
/// stock 737), replayed through a plant: the stop is flown, the throttles
/// held at idle, the parking brake set once stopped, and only then "your
/// aircraft".
#[cfg(test)]
mod reject_stop_tests {
    use super::*;
    use crate::xplane::FakeXPlane;
    use std::sync::Mutex;
    use tokio::time::Instant;

    const AT_MODE: &str = crate::aircraft::TakeoffConfigProfile::AT_MODE;
    const CMD_AT_OFF: &str = crate::aircraft::TakeoffConfigProfile::CMD_AT_OFF;
    const GS: &str = "sim/flightmodel/position/groundspeed";
    const PARK: &str = "sim/cockpit2/controls/parking_brake_ratio";
    const SET: &str = "sim/flight_controls/park_brake_set";

    /// Takeoff thrust's acceleration at 60 to 70 kt (the roll records:
    /// 60.4 to 66.7 kt in 1.2 s) and its decay once the levers close (a
    /// CFM56 spooling down from TOGA, fitted so one second of regular
    /// brakes peaks where the sortie did, 73 kt).
    const THRUST_KTS_S: f64 = 5.0;
    const SPOOL_TAU_S: f64 = 1.5;
    /// Coasting at idle, nobody braking: 73.3 kt at +76.8 to 52.7 at +116.
    const COAST_KTS_S: f64 = 0.5;
    /// Maximum braking, from the EGKK rejects (1790524628, 1790525414,
    /// 1790525500): about 0.3 s before the brakes bite, a second to build,
    /// then 8 to 15 kt/s tick to tick, 10.5 on the average.
    const MAX_BRAKE_KTS_S: f64 = 10.5;
    const REGULAR_BRAKE_KTS_S: f64 = 3.5;
    const BRAKE_DEAD_S: f64 = 0.3;
    const BRAKE_BUILD_S: f64 = 1.0;

    #[derive(Default)]
    struct State {
        last: Option<Instant>,
        gs_kts: f64,
        idle_at: Option<Instant>,
        /// The held brake command: effort, since (continuous), until.
        brake: Option<(f64, Instant, Instant)>,
        max_kts: f64,
        gs_at_park: Option<f64>,
        thrust_back: bool,
    }

    /// The 737 as the reject found it: rolling at `kts` on takeoff thrust,
    /// the autothrottle `at_mode`.
    struct RejectPlant {
        sim: FakeXPlane,
        st: Mutex<State>,
    }

    impl RejectPlant {
        fn rolling_at_kts(kts: f64, at_mode: f64) -> Self {
            let sim = FakeXPlane::with_reads(&[
                (crate::datarefs::ON_GROUND_ANY, 1.0),
                (crate::autotaxi::TUG_ATTACHED, 0.0),
                (PARK, 0.0),
                (AT_MODE, at_mode),
                (sim::datarefs::LATITUDE, 0.0),
                (sim::datarefs::LONGITUDE, 0.0),
                (sim::datarefs::ELEVATION_M, 15.0),
                ("sim/flightmodel/position/psi", 0.0),
                ("sim/flightmodel/position/vh_ind_fpm", 0.0),
                (sim::datarefs::AGL_METERS, 0.0),
                (YAW_RATE, 0.0),
            ]);
            sim.step_on_command(SET, PARK, 1.0);
            sim.set_on_command(CMD_AT_OFF, AT_MODE, 0.0);
            let st = State { gs_kts: kts, max_kts: kts, ..Default::default() };
            RejectPlant { sim, st: Mutex::new(st) }
        }

        /// Integrate to now in 10 ms steps.
        fn advance(&self) {
            let now = Instant::now();
            let at_engaged = self.sim.read_value(AT_MODE).is_some_and(|m| m > 0.5);
            let parked = self.sim.read_value(PARK).is_some_and(|p| p > 0.5);
            let mut s = self.st.lock().unwrap();
            let Some(mut t) = s.last else {
                s.last = Some(now);
                return;
            };
            let step = std::time::Duration::from_millis(10);
            while t + step <= now {
                t += step;
                let thrust = match s.idle_at {
                    _ if at_engaged => THRUST_KTS_S,
                    None => THRUST_KTS_S,
                    Some(i) => {
                        THRUST_KTS_S * (-(t - i).as_secs_f64() / SPOOL_TAU_S).exp()
                    }
                };
                let brake = match s.brake {
                    _ if parked => MAX_BRAKE_KTS_S,
                    Some((effort, since, until)) if t <= until => {
                        let on = (t - since).as_secs_f64() - BRAKE_DEAD_S;
                        effort * (on / BRAKE_BUILD_S).clamp(0.0, 1.0)
                    }
                    _ => 0.0,
                };
                let coast = if s.gs_kts > 0.0 { COAST_KTS_S } else { 0.0 };
                s.gs_kts = (s.gs_kts + (thrust - brake - coast) * 0.01).max(0.0);
                s.max_kts = s.max_kts.max(s.gs_kts);
            }
            s.last = Some(t);
        }

        fn gs_kts(&self) -> f64 {
            self.advance();
            self.st.lock().unwrap().gs_kts
        }
    }

    #[async_trait::async_trait]
    impl crate::xplane::XPlane for RejectPlant {
        async fn get_float(&self, name: &str) -> anyhow::Result<f64> {
            if name == GS {
                return Ok(self.gs_kts() / MS_TO_KTS);
            }
            self.advance();
            self.sim.get_float(name).await
        }
        async fn set_float(&self, name: &str, value: f64) -> anyhow::Result<()> {
            self.advance();
            if name.contains("throttle") {
                let mut s = self.st.lock().unwrap();
                if value > 0.05 {
                    s.thrust_back = s.idle_at.is_some();
                    s.idle_at = None;
                } else if s.idle_at.is_none() {
                    s.idle_at = Some(Instant::now());
                }
            }
            self.sim.set_float(name, value).await
        }
        async fn set_int(&self, name: &str, value: i64) -> anyhow::Result<()> {
            self.sim.set_int(name, value).await
        }
        async fn run_command(&self, name: &str) -> anyhow::Result<()> {
            self.advance();
            if name == SET {
                let gs = self.st.lock().unwrap().gs_kts;
                self.st.lock().unwrap().gs_at_park.get_or_insert(gs);
            }
            self.sim.run_command(name).await
        }
        async fn run_command_for(&self, name: &str, secs: f64) -> anyhow::Result<()> {
            self.advance();
            let effort = match name {
                BRAKES_MAX => MAX_BRAKE_KTS_S,
                crate::tools::BRAKES_REGULAR => REGULAR_BRAKE_KTS_S,
                _ => 0.0,
            };
            if effort > 0.0 {
                let now = Instant::now();
                let until = now + std::time::Duration::from_secs_f64(secs);
                let mut s = self.st.lock().unwrap();
                let since = match s.brake {
                    Some((_, since, prev_until)) if now <= prev_until => since,
                    _ => now,
                };
                s.brake = Some((effort, since, until));
            }
            self.sim.run_command_for(name, secs).await
        }
        async fn get_float_array(&self, name: &str) -> anyhow::Result<Vec<f64>> {
            self.sim.get_float_array(name).await
        }
        async fn set_int_array_elem(&self, name: &str, i: usize, v: i64) -> anyhow::Result<()> {
            self.sim.set_int_array_elem(name, i, v).await
        }
        async fn set_float_array_elem(&self, name: &str, i: usize, v: f64) -> anyhow::Result<()> {
            self.sim.set_float_array_elem(name, i, v).await
        }
    }

    fn rwy_north() -> Runway {
        Runway {
            icao: "LEMG".into(),
            id: "13".into(),
            thr_lat: -0.001,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 3200.0,
            surface: 1,
            dist_nm: 0.0,
            elev_ft: 52.0,
        }
    }

    /// The plant is the sortie: the old reject (idle and one second of the
    /// regular brakes, then nobody) pushed the 737 from 69 to about 73 kt
    /// on the spool-down and left it above 50 kt forty seconds on.
    #[tokio::test(start_paused = true)]
    async fn the_plant_reproduces_the_lemg_reject() {
        let jet = RejectPlant::rolling_at_kts(69.2, 0.0);
        jet.set_float(&throttle_dref(), 0.0).await.unwrap();
        jet.run_command_for(crate::tools::BRAKES_REGULAR, 1.0).await.unwrap();
        tokio::time::sleep(std::time::Duration::from_secs(6)).await;
        jet.gs_kts();
        let peak = jet.st.lock().unwrap().max_kts;
        assert!((72.0..75.0).contains(&peak), "peak {peak:.1} kt, the sortie's 73.3");
        tokio::time::sleep(std::time::Duration::from_secs(34)).await;
        let gs = jet.gs_kts();
        assert!(gs > 50.0, "{gs:.1} kt forty seconds on, the sortie's 52.7");
    }

    /// The fix: the reject owns the levers (idle every tick, an ENGAGED
    /// autothrottle taken out and read back), brakes at maximum inside the
    /// held command's expiry to a FULL stop on the centreline, and sets the
    /// parking brake only once stopped (#112's caution never sees its own
    /// brake rolling), read back, before "your aircraft".
    #[tokio::test(start_paused = true)]
    async fn a_rejected_takeoff_is_flown_to_a_stop_and_parked() {
        let jet = RejectPlant::rolling_at_kts(69.2, 1.0);
        let at_off = reject_hands(&jet).await;
        assert_eq!(at_off, Some(true), "the engaged autothrottle is out, read back");
        let mut law = RollSteer::for_airframe();
        let stop = stop_on_centreline(&jet, &rwy_north(), &mut law).await;
        let held = crate::full_auto::hand_back_hold(&jet, Some(1.0)).await;
        let s = jet.st.lock().unwrap();
        assert!(stop.stopped, "{stop:?}");
        assert!(!s.thrust_back, "the levers never came back up");
        assert!(
            jet.sim.writes_to(&throttle_dref()).iter().all(|v| *v == 0.0),
            "idle held: {:?}",
            jet.sim.writes_to(&throttle_dref())
        );
        assert!(s.max_kts < 71.0, "peak {:.1} kt, never the sortie's 73.3", s.max_kts);
        assert!(stop.secs < 12.0, "stopped in {:.1} s", stop.secs);
        assert!(stop.brake_pulses >= 8, "maximum braking renewed: {stop:?}");
        assert_eq!(held, Some(true), "the parking brake reads set");
        let at_park = s.gs_at_park.expect("the parking brake was set");
        assert!(at_park < 1.0, "parked at {at_park:.1} kt, not rolling");
        assert!(s.gs_kts < 0.1, "held at {:.2} kt", s.gs_kts);
        assert_eq!(
            reject_end_line(&stop, held),
            "Stopped. Parking brake set. Your aircraft."
        );
    }

    /// Nothing claimed that did not read back.
    #[test]
    fn the_reject_says_only_what_holds_the_aircraft() {
        let stopped = RejectStop { stopped: true, ..Default::default() };
        let rolling = RejectStop::default();
        assert!(reject_end_line(&stopped, Some(false)).contains("didn't set"));
        assert!(!reject_end_line(&rolling, Some(false)).starts_with("Stopped"));
        assert!(reject_end_line(&rolling, None).contains("couldn't stop"));
    }

    /// An engaged autothrottle moves the levers; armed and hard off do not.
    #[test]
    fn only_an_engaged_autothrottle_is_taken_out() {
        use crate::takeoff_config::autothrottle_engaged;
        assert!(!autothrottle_engaged(-1.0) && !autothrottle_engaged(0.0));
        assert!(autothrottle_engaged(1.0) && autothrottle_engaged(2.0));
        assert!(!autothrottle_engaged(f64::NAN));
    }
}
