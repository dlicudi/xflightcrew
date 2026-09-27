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
const THETA: &str = "sim/flightmodel/position/true_theta";
const PHI: &str = "sim/flightmodel/position/true_phi";
/// The sim's own pitch rate, deg/s — what the rotation law damps on. Measured,
/// so it needs neither differencing nor a filter; see [`RotatePitch::step`].
const PITCH_RATE: &str = "sim/flightmodel/position/Q";

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
/// Above this the rudder is aerodynamic and the trim scales with 1/v².
const TRIM_SCALE_FROM_KTS: f64 = 40.0;
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
const BANK_PER_HDG_ERR: f64 = 1.0;
/// Aileron law: P on bank error (aileron commands roll RATE, so
/// proportional converges without an integrator), rate-lead damped.
const AIL_FULL_BANK_DEG: f64 = 15.0;
const AIL_LEAD_S: f64 = 0.3;
const AIL_MAX: f64 = 0.5;
const AIL_SLEW_PER_S: f64 = 2.0;
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

fn gear_warning() -> Option<crate::aircraft::GearWarning> {
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
    let best = cands.iter().min_by(|a, b| {
        heading_diff(psi_true, a.course_deg).total_cmp(&heading_diff(psi_true, b.course_deg))
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
    let corr = ((offset + rate * XTK_LEAD_S) * gain).clamp(-XTK_MAX_DEG, XTK_MAX_DEG);
    (course_deg - corr).rem_euclid(360.0)
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

/// The fastest a cross-track reading may move and still be believed, m/s.
pub const XTK_RATE_MAX_M_S: f64 = 5.0;
/// The smoothing of the cross-track rate, seconds.
pub const XTK_RATE_TAU_S: f64 = 1.0;

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
fn squawk_altitude(base_url: String, v: Option<Voice>) {
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
    let fade = ((ROLL_BRAKE_OFF_KTS - gs_kts) / (ROLL_BRAKE_OFF_KTS - ROLL_BRAKE_FULL_KTS))
        .clamp(0.0, 1.0);
    let (l, r) = crate::autotaxi::brake_steer_split(steer, gs_kts);
    (l * fade, r * fade)
}

/// [`roll_brake_split`], plus the start of a roll that begins crooked: below
/// [`ROLL_ALIGN_KTS`] a heading error past [`ROLL_ALIGN_ERR_DEG`] pivots on
/// the inside brake in proportion to the ERROR, not the roll law's steer,
/// whose deflection is capped tiny at walking pace (prop wash) and whose brake
/// split has no authority under 2 kt. LIME 28 (2026-09-25): the Evolution
/// started its roll 21 degrees off the runway, the steer sat at 0.17 with no
/// brake at all, the error grew to 30 at 6 kt and the pilot's pedals ended
/// it. The pivot's own yaw-rate ceiling applies, so it cannot spin.
pub fn roll_brake_split_aligning(steer: f64, gs_kts: f64, err_deg: f64, yaw_dps: f64) -> (f64, f64) {
    let (l, r) = roll_brake_split(steer, gs_kts);
    if gs_kts >= ROLL_ALIGN_KTS || err_deg.abs() <= ROLL_ALIGN_ERR_DEG {
        return (l, r);
    }
    let want = (err_deg / ROLL_ALIGN_FULL_DEG).clamp(-1.0, 1.0);
    // Full pivot authority from a standstill: power is coming up behind it.
    let (pl, pr) = crate::autotaxi::pivot_brake_split(want, gs_kts.max(2.0));
    let k = crate::autotaxi::pivot_rate_scale(want, yaw_dps);
    (l.max(pl * k), r.max(pr * k))
}
/// The aligning pivot works below this groundspeed...
const ROLL_ALIGN_KTS: f64 = 15.0;
/// ...on a heading error past this...
const ROLL_ALIGN_ERR_DEG: f64 = 3.0;
/// ...at full pivot brake from this error.
const ROLL_ALIGN_FULL_DEG: f64 = 15.0;

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
            ..Self::default()
        }
    }

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
}

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
        }
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
        let target = match self.target {
            None => theta_deg.min(cap),
            Some(t) => (t + ROTATE_RATE_DEG_S * dt).min(cap),
        };
        self.target = Some(target);
        // Damp the rate ERROR: while the ramp is still climbing, the nose is
        // SUPPOSED to be moving at ROTATE_RATE_DEG_S, and subtracting that
        // motion as if it were an overshoot is what stalls a rotation.
        let want_rate = if target < cap { ROTATE_RATE_DEG_S } else { 0.0 };
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
        self.out = (err * PITCH_P_PER_DEG + self.trim).clamp(self.pitch_min, PITCH_MAX);
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
    /// Bank error for full aileron, degrees; `None` is [`AIL_FULL_BANK_DEG`].
    full_bank_deg: Option<f64>,
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

    pub fn step(&mut self, bank_deg: f64, target_deg: f64, dt_s: f64) -> f64 {
        let dt = dt_s.clamp(0.05, 1.0);
        let rate = self.prev_bank.map_or(0.0, |p| (bank_deg - p) / dt);
        self.prev_bank = Some(bank_deg);
        let err = target_deg - bank_deg - AIL_LEAD_S * rate;
        let full = self.full_bank_deg.unwrap_or(AIL_FULL_BANK_DEG);
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
/// Over the limit by this share, the lever comes back a step per tick.
const TORQUE_OVER_FRAC: f64 = 0.05;
const TORQUE_BACKOFF_STEP: f64 = 0.02;

/// The lever under the manual's torque limit: below the limit the law's own
/// want; at it the lever holds; past it by [`TORQUE_OVER_FRAC`] it comes
/// back a step. Past `until_kias`, or with no torque read or no limit, the
/// want as asked. Pure.
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
    if tq > l.torque_ftlb * (1.0 + TORQUE_OVER_FRAC) {
        (last - TORQUE_BACKOFF_STEP).max(0.0).min(want)
    } else if tq >= l.torque_ftlb {
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

/// Throttle idle, one expiring brake pulse, steering released, and say why.
/// Maximum braking, held: a rejected takeoff is a maximum-effort stop.
const BRAKES_MAX: &str = "sim/flight_controls/brakes_max";
/// Below this a rejected roll is slow enough to hand over.
const REJECT_HANDOVER_KTS: f64 = 20.0;
/// The longest the stop is flown before handing over regardless.
const REJECT_STOP_MAX_S: f64 = 90.0;

/// A reject at speed is FLOWN to a stop: idle, maximum braking renewed every
/// second, and the centreline held with the roll's own law until the
/// aircraft is under [`REJECT_HANDOVER_KTS`]. The reject used to brake for
/// one second, zero the steering and say "your aircraft" at 127 kt; nobody
/// was on the pedals and the 737 left the side of LIBP 04 (2026-09-26).
async fn stop_on_centreline(xp: &XPlaneClient, rwy: &Runway, law: &mut RollSteer) {
    let t0 = std::time::Instant::now();
    let mut last = std::time::Instant::now();
    let mut last_brake: Option<std::time::Instant> = None;
    let mut prev_xtk: Option<f64> = None;
    let mut rate_filter = XtkRate::default();
    while t0.elapsed().as_secs_f64() < REJECT_STOP_MAX_S {
        let (lat, lon, psi, gs) = tokio::join!(
            xp.get_float(LAT),
            xp.get_float(LON),
            xp.get_float(PSI_TRUE),
            xp.get_float(GS_MS)
        );
        let Some([lat, lon, psi, gs]) = crate::control::measurements("autotakeoff", [lat, lon, psi, gs])
        else {
            tokio::time::sleep(std::time::Duration::from_millis(200)).await;
            continue;
        };
        let gs_kts = gs * MS_TO_KTS;
        if gs_kts < REJECT_HANDOVER_KTS {
            break;
        }
        if last_brake.is_none_or(|t| t.elapsed().as_secs_f64() >= 0.8) {
            if xp.run_command_for(BRAKES_MAX, crate::tools::BRAKE_HOLD_SECS).await.is_ok() {
                crate::tools::note_brake_cmd();
            }
            last_brake = Some(std::time::Instant::now());
        }
        let dt = last.elapsed().as_secs_f64().clamp(0.05, 1.0);
        last = std::time::Instant::now();
        let xtk_m = cross_track_nm(lat, lon, rwy) * M_PER_NM;
        let rate = rate_filter.update(prev_xtk.map_or(0.0, |p| (xtk_m - p) / dt), dt);
        prev_xtk = Some(xtk_m);
        let target = roll_target_heading(rwy.course_deg, xtk_m, rate, gs_kts);
        let err = ((target - psi + 540.0).rem_euclid(360.0)) - 180.0;
        let steer = law.step(err, dt, gs_kts, 0.0);
        let _ = xp.set_float(TAXI_STEER, steer).await;
        crate::blackbox::record(
            "autotakeoff",
            serde_json::json!({"event": "reject_rollout", "gs": gs_kts, "xtk_m": xtk_m,
                               "err": err, "steer": steer}),
        );
        tokio::time::sleep(std::time::Duration::from_millis(200)).await;
    }
    let _ = xp.set_float(TAXI_STEER, 0.0).await;
    if xp.run_command_for(BRAKES_MAX, crate::tools::BRAKE_HOLD_SECS).await.is_ok() {
        crate::tools::note_brake_cmd();
    }
}

/// A reject while ROLLING: the reject, then the stop flown on the
/// centreline, then the hand-over, said once stopped.
async fn reject_rolling(
    xp: &XPlaneClient,
    voice: &Option<Voice>,
    why: &str,
    reason: &str,
    rwy: &Runway,
    law: &mut RollSteer,
) {
    // Unscoped: a reject is how the run ENDS, and a cancelled scope must
    // never refuse the brakes that stop it.
    let xp = &xp.unscoped();
    reject(xp, voice, &format!("{why} Rejecting takeoff."), reason).await;
    stop_on_centreline(xp, rwy, law).await;
    alert(voice, Priority::Warning, "Stopped. Your aircraft.");
    crate::blackbox::record("autotakeoff", serde_json::json!({"event": "reject_stopped", "reason": reason}));
}

async fn reject(xp: &XPlaneClient, voice: &Option<Voice>, line: &str, reason: &str) {
    // The throttle and brakes of a reject are never refused by the scope
    // the abort itself just cancelled.
    let xp = &xp.unscoped();
    RUN.record(sim::run::Outcome::Failed(reason.into()));
    let _ = xp.set_float(&throttle_dref(), 0.0).await;
    if xp
        .run_command_for(crate::tools::BRAKES_REGULAR, crate::tools::BRAKE_HOLD_SECS)
        .await
        .is_ok()
    {
        crate::tools::note_brake_cmd();
    }
    let _ = xp.set_float(TAXI_STEER, 0.0).await;
    crate::takeoff_config::on_reject(xp).await;
    alert(voice, Priority::Warning, line);
    crate::blackbox::record(
        "autotakeoff",
        serde_json::json!({"event": "reject", "reason": reason}),
    );
}

/// A mode command is only a request; handoff needs live mode and dial readbacks.
async fn engage_climb(xp: &dyn crate::xplane::XPlane, vs: f64, last: &mut Option<f64>) -> bool {
    if crate::xplane::sim_frozen(xp).await {
        return false;
    }
    let (heading, vertical) = tokio::join!(xp.get_float(HDG_STATUS), xp.get_float(VS_STATUS));
    let Some([heading, vertical]) =
        crate::control::measurements("autotakeoff", [heading, vertical])
    else {
        return false;
    };
    if (heading <= 0.5 || vertical <= 0.5) && xp.run_command(CMD_SERVOS_ON).await.is_err() {
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
    let (heading, vertical) = tokio::join!(xp.get_float(HDG_STATUS), xp.get_float(VS_STATUS));
    matches!((heading, vertical), (Ok(h), Ok(v)) if h.is_finite() && v.is_finite() && h > 0.5 && v > 0.5)
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
    // rejected as stagnant. The VALVE goes with it (2026-08-03): left in the
    // set position it traps the roll's own brake applications — the reject
    // path's stopping pulse included — back into the parking brake.
    // Trim for takeoff BEFORE the brakes come off (see `trim_for_takeoff`:
    // a nose-down stop trim is what the AP's climb loses to at the engage).
    // The WHOLE configuration, and the brakes wait for the flaps to arrive:
    // the handle used to be written after "Rolling", and an A330's CONF 2
    // was still running out at sixty knots (EGPF, 2026-09-21). After full
    // auto's after-start call this finds everything set and says nothing.
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
    squawk_altitude(p.base_url.clone(), voice.clone());
    alert(
        &voice,
        Priority::Advisory,
        &format!(
            "Brakes released. Rolling, runway {}.",
            crate::atis::spoken_runway(&rwy.id)
        ),
    );

    // Pre-set the flight director and heading bug NOW, parked: the bug
    // slews by knob presses (one HTTP round trip per degree), and doing
    // that later would stall the loop while the steer channel expires
    // mid-roll. No VS dial preset — VS-mode engage syncs the dial to the
    // current rate anyway, so it's written after the engage instead.
    for c in &p.cfg.fd_commands {
        let _ = xp.run_command(c).await;
    }
    let _ = crate::tools::autopilot::drive_heading_bug_step(
        &xp,
        true_to_mag(rwy.course_deg, var),
        360.0,
    )
    .await;

    // Takeoff flaps from the profile, set before speed builds. 0.0 on a
    // clean-takeoff type still runs, guaranteeing the config off the gate.
    if let Some(ratio) = p.takeoff_flap_ratio {
        let _ = xp.set_float(FLAP_HANDLE, ratio).await;
    }
    // Nothing to retract if we took off clean (<= a whisker of flap).
    let mut flaps_retracted = p.takeoff_flap_ratio.unwrap_or(0.0) <= 0.01;
    // The setting the law believes is selected, for the schedule to walk down.
    let mut flap_cmd = p.takeoff_flap_ratio.unwrap_or(0.0);

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
    let mut positive_called = false;
    let mut thr_challenged = false;
    let mut pitch_law = RotatePitch::with_authority(
        p.tail_strike_deg,
        p.pitch_authority_down.map_or(PITCH_MIN, |a| -a.abs()),
    );
    let mut wings = WingsLevel::default();
    let mut last_vs_cmd: Option<f64> = None;
    let mut last_thr_cmd = 0.3_f64;
    // The manual's takeoff torque, for this airframe, read once per roll.
    let power_limit = takeoff_power();
    // The roll section's heading error, one tick stale, for the throttle's
    // swinging-nose hold (throttle is commanded before position is read).
    let mut last_err = 0.0_f64;
    let mut steer_law = RollSteer::for_airframe();
    let brake_steer = brake_steers();
    // Previous cross-track, for the roll's own closure rate (see
    // [`roll_target_heading`]). Metres, right-positive.
    let mut prev_xtk_m: Option<f64> = None;
    let mut rate_filter = XtkRate::default();
    // The heading error's last value, for the yaw rate an aligning pivot
    // is capped on (the roll reads no yaw rate of its own).
    let mut prev_align_err: Option<f64> = None;
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
                reject(
                    &xp,
                    &voice,
                    "Rejecting takeoff. Throttle is idle, brakes on. Your aircraft.",
                    "pilot_abort",
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
            lat,
            lon,
            psi,
            gs_ms,
            ias,
            agl_m,
            on_ground,
            steer_yield,
            thr_read,
            theta,
            phi,
            pitch_q,
            pitch_yield,
            roll_yield,
            torque_nm,
        ) = tokio::join!(
            xp.get_float(LAT),
            xp.get_float(LON),
            xp.get_float(PSI_TRUE),
            xp.get_float(GS_MS),
            xp.get_float(IAS),
            xp.get_float(AGL_M),
            xp.get_float(ON_GROUND),
            xp.get_float(STEER_YIELD),
            xp.get_float(&thr_dref),
            xp.get_float(THETA),
            xp.get_float(PHI),
            xp.get_float(PITCH_RATE),
            xp.get_float(PITCH_YIELD),
            xp.get_float(ROLL_YIELD),
            xp.get_float_array(ENGN_TRQ),
        );
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
        let elapsed = roll_start.elapsed().as_secs_f64() - frozen_secs;
        let thr_want = torque_limited(
            tick_throttle_want(airborne, elapsed, last_err, gs_kts, ias, p.vne_kts),
            last_thr_cmd,
            torque_ftlb,
            ias,
            power_limit.as_ref(),
        );
        let thr_cmd = throttle_slew(last_thr_cmd, thr_want, dt);
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
                reject(
                    &xp,
                    &voice,
                    "We're not accelerating. Rejecting takeoff. Your aircraft.",
                    "stagnant",
                )
                .await;
                return;
            }
            let along_m = along_track_nm(lat, lon, &rwy) * M_PER_NM;
            let remaining = rwy.length_m - (-along_m).max(0.0);
            if !rotated && must_reject(remaining, ias, p.vr_kts) {
                reject_rolling(&xp, &voice, "Runway is running out.", "runway_end", &rwy, &mut steer_law).await;
                return;
            }
            // Steer the centreline: PD plus the learned P-factor trim,
            // slew-limited (no tick slams the wheel).
            let xtk_m = cross_track_nm(lat, lon, &rwy) * M_PER_NM;
            let xtk_rate = rate_filter.update(prev_xtk_m.map_or(0.0, |p| (xtk_m - p) / dt), dt);
            prev_xtk_m = Some(xtk_m);
            let target = roll_target_heading(rwy.course_deg, xtk_m, xtk_rate, gs_kts);
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
                    reject_rolling(&xp, &voice, "We're leaving the centreline.", "lateral_excursion", &rwy, &mut steer_law).await;
                    return;
                }
            }
            let steer = steer_law.step(err, dt, gs_kts, thr_cmd);
            // The brake half, on an airframe that has no other low-speed
            // steering; zero once rotated (the channel expires anyway).
            let yaw_dps = prev_align_err.map_or(0.0, |p| -(err - p) / dt.max(0.05));
            prev_align_err = Some(err);
            let (bl, br) = if brake_steer && !rotated {
                roll_brake_split_aligning(steer, gs_kts, err, yaw_dps)
            } else {
                (0.0, 0.0)
            };
            if brake_steer {
                let _ = tokio::join!(
                    xp.set_float(crate::autotaxi::LEFT_BRAKE, bl),
                    xp.set_float(crate::autotaxi::RIGHT_BRAKE, br),
                );
            }
            if xp.set_float(TAXI_STEER, steer).await.is_err() {
                reject(
                    &xp,
                    &voice,
                    "I've lost the steering. Rejecting takeoff. Your aircraft.",
                    "steer_lost",
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
                    "bl": bl, "br": br,
                    "xtk_rate": xtk_rate,
                    // The heading and the target it was steered to: without
                    // them the rudder's yaw per unit of steer cannot be read
                    // off a roll (LIBP 04 held 3 degrees of error for six
                    // seconds at 86 to 105 kt on 0.15 to 0.25 of steer).
                    "psi": psi, "tgt": target,
                    "thr": thr_cmd, "remaining_m": remaining, "dt": dt, "trq_ftlb": torque_ftlb,
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
                let pitch = pitch_law.step(theta_deg, pitch_rate, dt);
                let ail = wings.step(phi_deg, 0.0, dt);
                crate::monitor::note_crew_yoke();
                let (wp, wr) = tokio::join!(
                    xp.set_float(TAKEOFF_PITCH, pitch),
                    xp.set_float(TAKEOFF_ROLL, ail)
                );
                if wp.is_err() || wr.is_err() {
                    reject(
                        &xp,
                        &voice,
                        "I've lost the elevator. Rejecting takeoff. Your aircraft.",
                        "pitch_lost",
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
                squawk_altitude(p.base_url.clone(), voice.clone());
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
                }
            } else if let Some(next) = flap_retract_step(&p.flap_schedule, flap_cmd, ias) {
                flap_cmd = next;
                flaps_retracted = next <= 1e-6;
                let _ = xp.set_float(FLAP_HANDLE, next).await;
                crate::blackbox::record(
                    "autotakeoff",
                    serde_json::json!({
                        "event": "flap_step", "to": next, "ias": ias, "agl_ft": agl_ft,
                    }),
                );
                alert(
                    &voice,
                    Priority::Advisory,
                    if flaps_retracted {
                        "Flaps up."
                    } else {
                        "Flaps coming up a stage."
                    },
                );
            }
        }
        if !ap_engaged {
            if agl_ft < AP_ENGAGE_AGL_FT {
                // Still hand-flying: hold the rotation attitude while speed
                // builds, and the wings against full-power torque — banked
                // gently toward runway heading, since the rudder steering
                // ended at liftoff.
                let hdg_err = ((rwy.course_deg - psi + 540.0).rem_euclid(360.0)) - 180.0;
                let pitch = pitch_law.step(theta_deg, pitch_rate, dt);
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
            let vs = climb_out_vs_fpm(
                ias,
                p.initial_climb_kts,
                p.climb_fpm,
                climb_guard_vne(p.vne_kts),
                agl_ft,
                climb_out,
            );
            if !engage_climb(&xp, vs, &mut last_vs_cmd).await {
                continue;
            }
            ap_engaged = true;
            let trim = xp.get_float(ELEV_TRIM).await.ok();
            crate::blackbox::record(
                "autotakeoff",
                serde_json::json!({
                    "event": "ap_engaged", "ias": ias, "agl": agl_ft, "cmd_vs": vs, "trim": trim,
                }),
            );
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
        let vs = climb_out_vs_fpm(
            ias,
            p.initial_climb_kts,
            base_fpm,
            climb_guard_vne(p.vne_kts),
            agl_ft,
            climb_out,
        );
        if !engage_climb(&xp, vs, &mut last_vs_cmd).await {
            continue;
        }
        DETAIL.set(format!(
            "climbing, {} ft, handoff at {} ft",
            (agl_ft / 50.0).round() * 50.0,
            p.handoff_agl_ft.max(AP_ENGAGE_AGL_FT + 50.0).round()
        ));
        crate::auto_status::set_ap_targets(None, None, Some(vs));
        crate::blackbox::record(
            "autotakeoff",
            serde_json::json!({"event": "climb", "ias": ias, "agl": agl_ft, "cmd_vs": vs}),
        );
        if agl_ft >= p.handoff_agl_ft.max(AP_ENGAGE_AGL_FT + 50.0) {
            // At the height, dirty, and a few knots short of the clean-up:
            // keep flying the climb until the schedule above has done it.
            let floor = crate::aircraft::speed_floor_kts();
            let waited = handoff_wait.get_or_insert_with(std::time::Instant::now);
            let waited_s = waited.elapsed().as_secs_f64();
            if handoff_waits_for_clean_up(flaps_retracted, ias, floor, waited_s)
                || (climb_out.is_some()
                    && handoff_waits_for_schedule(
                        flaps_retracted,
                        !p.flap_schedule.is_empty(),
                        waited_s,
                    ))
            {
                tokio::time::sleep(std::time::Duration::from_millis(400)).await;
                continue;
            }
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
            if let Some(up) = clean_up_at_handoff(flaps_retracted, ias, floor) {
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
                                   "to_caller": p.handoff_to_caller}),
            );
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

#[cfg(test)]
mod tests {

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
        assert!(super::engage_climb(&xp, 700.0, &mut last).await);
        xp.set_read(super::VS_STATUS, f64::NAN);
        assert!(!super::engage_climb(&xp, 700.0, &mut last).await);
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
        assert_eq!(torque_limited(0.8, 0.5, Some(1260.0), 40.0, Some(&evo)), 0.5);
        assert!((torque_limited(0.8, 0.5, Some(1400.0), 40.0, Some(&evo)) - 0.48).abs() < 1e-9);
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
        let (mut reversals, mut last_cmd, mut last_dir) = (0usize, None::<f64>, 0.0_f64);
        let mut waited = None::<f64>;
        for tick in 0..400 {
            let t = tick as f64 * dt;
            let gate = climb_out.map_or(FLAP_RETRACT_AGL_FT, |c| c.accel_agl_ft);
            if flap > 1e-6 && agl > gate {
                if let Some(next) = flap_retract_step(&schedule, flap, ias) {
                    flap = next;
                    steps.push((agl, ias, next));
                }
            }
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
        // Flaps 5 stays to the height, then every detent at its own speed.
        assert_eq!(steps.len(), 3, "{steps:?}");
        assert!(steps.iter().all(|(agl, _, _)| *agl > 1000.0), "{steps:?}");
        assert!(steps[2].1 >= 210.0 && steps[2].2 <= 1e-6, "{steps:?}");
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
        assert!(r > 0.3 && l == 0.0, "right brake, from a standstill: {l} {r}");
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
}
