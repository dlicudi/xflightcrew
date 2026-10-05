//! The helicopter's hover: the autopilot holds the POSITION, this loop holds
//! the HEIGHT on the collective.
//!
//! Measured on the S-76C (2026-09-22) before a line of this was written: the
//! autopilot engages (once the stick trim and inverter are on), its hover
//! mode captures and works the cyclic, and it never moves the collective -
//! vertical speed pitches the nose like an aeroplane's, and the autothrottle,
//! which drives the collective on some helicopters, is not fitted
//! (`autothrottle_enabled` -1). The collective is `prop_ratio[0]` and takes a
//! write. So the split is the classic one: cyclic to the autopilot, collective
//! to us.
//!
//! [`CollectiveLaw`] is P on the vertical-speed error plus a slow trim that
//! finds the hover collective, rate-limited both ways, with a lift-off ramp
//! on the ground (the error alone would take a minute to find the hover from
//! a flat pitch). [`target_vs_fpm`] turns a height into a climb rate and a
//! landing into a gentle descent. The tests fly both through a simulated
//! helicopter over a range of responses, because the S-76's real hover
//! collective and gain are only known after the first lift.
//!
//! Every lever figure the loop owns is scaled by the POWER BUDGET
//! ([`PowerBudget`], [`LoadingEstimate`], [`PowerFeed`]): weight over density
//! ratio from `m_total`, the pressure altitude and the OAT, corrected by the
//! lever the rotor actually lifts at. A hover the lever cannot give is refused
//! on the ground, or held in the ground cushion, with its numbers (#84).
//!
//! The loop hands back a helicopter the pilot can hold: the collective stays
//! where it was (a write is not an expiring override here, so it does not
//! drop), the autopilot is disengaged with the command that actually does it
//! on this aircraft (`servos_fdir_off`; plain `servos_off` left it engaged).

use crate::autoflight::{self, Button, Dial};
use crate::datarefs::ON_GROUND_ANY;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

/// The hover loop is flying. Written only by the run ([`RUN`]).
pub static ACTIVE: AtomicBool = AtomicBool::new(false);
/// The hover loop's run: claimed by the first [`request`] (later ones only
/// retarget), cancelled by [`abort`] and [`release`]. A loop winding down
/// cannot clear the flag, or the goal, of one started after it.
pub(crate) static RUN: sim::run::Slot = sim::run::Slot::flagged(&ACTIVE);

/// What the loop is flying towards. Changed live by a new request.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Goal {
    Hover {
        height_ft: f64,
    },
    Land,
    /// Out of the hover into forward flight: accelerate along the track at
    /// the height the hover was at, climb once the rotor is flying (past
    /// translational lift), level at `height_ft` at `speed_mps`.
    Depart {
        height_ft: f64,
        speed_mps: f64,
        track_deg: f64,
    },
    /// A flown vector steered by an orchestrator (the circuit): the same
    /// speed and track as a departure, retargeted as it goes, with no climb
    /// gate and nothing announced; it descends too (a final's glidepath).
    Fly {
        height_ft: f64,
        speed_mps: f64,
        track_deg: f64,
    },
}

impl Goal {
    /// Speed and true track of a flown vector; `None` for a hover or a
    /// landing.
    pub fn vector(&self) -> Option<(f64, f64)> {
        match *self {
            Goal::Depart {
                speed_mps,
                track_deg,
                ..
            }
            | Goal::Fly {
                speed_mps,
                track_deg,
                ..
            } => Some((speed_mps, track_deg)),
            _ => None,
        }
    }
}

static GOAL: Mutex<Option<Goal>> = Mutex::new(None);

fn lock_goal() -> std::sync::MutexGuard<'static, Option<Goal>> {
    GOAL.lock().unwrap_or_else(|e| e.into_inner())
}

pub fn goal() -> Option<Goal> {
    *lock_goal()
}

/// The groundspeed the hover is asked to move at: m/s along a TRUE track.
/// Zero is a hover over the spot. The hover mode is a groundspeed hold, so
/// a moving target is an air taxi: the autopilot flies it on the cyclic
/// while the collective keeps the height.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Motion {
    pub speed_mps: f64,
    pub track_deg: f64,
}

static MOTION: Mutex<Motion> = Mutex::new(Motion {
    speed_mps: 0.0,
    track_deg: 0.0,
});

fn lock_motion() -> std::sync::MutexGuard<'static, Motion> {
    MOTION.lock().unwrap_or_else(|e| e.into_inner())
}

pub fn motion() -> Motion {
    *lock_motion()
}

/// Move along `track_deg` at `speed_mps` (0 = hold the spot). The loop
/// ramps to it; a landing always ramps to zero first.
pub fn set_motion(m: Motion) {
    *lock_motion() = m;
}

/// The fastest air taxi: the hover mode has been flown to 0.4 m/s of error
/// hands off, never above a walking pace, so this grows by measurement.
pub const MAX_AIR_TAXI_MPS: f64 = 5.2;

/// East and south components of a speed along a true track (the frame
/// `hover_vector` is in: x east, z south).
pub fn track_vector(m: Motion) -> [f64; 2] {
    let t = m.track_deg.to_radians();
    [m.speed_mps * t.sin(), -m.speed_mps * t.cos()]
}

/// The commanded groundspeed, slewed toward the asked one: a step in the
/// hover mode's target is a lurch of the cyclic.
#[derive(Debug, Clone, Copy, Default)]
pub struct VelocityRamp {
    v: [f64; 2],
}

const RAMP_ACCEL_MPS2: f64 = 0.4;

impl VelocityRamp {
    pub fn now(&self) -> [f64; 2] {
        self.v
    }
    /// Take over a command flown by other means (a departure's own speed
    /// and track), so the next goal ramps from where it really is.
    pub fn set(&mut self, v: [f64; 2]) {
        self.v = v;
    }
    pub fn step(&mut self, want: [f64; 2], dt_s: f64) -> [f64; 2] {
        let dt = dt_s.clamp(0.02, 0.5);
        let (dx, dz) = (want[0] - self.v[0], want[1] - self.v[1]);
        let d = dx.hypot(dz);
        let max = RAMP_ACCEL_MPS2 * dt;
        if d <= max {
            self.v = want;
        } else {
            self.v[0] += dx / d * max;
            self.v[1] += dz / d * max;
        }
        self.v
    }
}

/// A heading asked during a departure (MAGNETIC, as spoken), for the loop to
/// turn the track to: the loop knows the variation of the moment.
static TURN_MAG: Mutex<Option<f64>> = Mutex::new(None);

/// "Turn heading two seven zero" during a departure turns the TRACK; the
/// autopilot's heading mode would replace the hover mode flying it. `None`
/// when no departure runs (the heading tool's ordinary refusal stands).
pub fn depart_turn(heading_mag: f64) -> Option<String> {
    if !ACTIVE.load(Ordering::SeqCst) || !matches!(goal(), Some(Goal::Depart { .. })) {
        return None;
    }
    let h = heading_mag.rem_euclid(360.0);
    *TURN_MAG.lock().unwrap_or_else(|e| e.into_inner()) = Some(h);
    Some(format!("Turning to heading {:03.0}.", h))
}

/// "Speed fifty knots" during a departure changes the flown speed; the
/// autopilot's airspeed mode would replace the hover mode (and the S-76's
/// pitched to 30 degrees below 30 kt, 2026-09-23). `None` when no departure
/// runs. Out of range is refused with the range.
pub fn depart_speed(kts: f64) -> Option<String> {
    if !ACTIVE.load(Ordering::SeqCst) {
        return None;
    }
    let Some(Goal::Depart {
        height_ft,
        track_deg,
        ..
    }) = goal()
    else {
        return None;
    };
    let mps = kts / 1.94384;
    if !(DEPART_CLIMB_MPS..=DEPART_MAX_MPS + 1e-6).contains(&mps) {
        return Some(format!(
            "I'll fly {:.0} to {:.0} knots.",
            DEPART_CLIMB_MPS * 1.94384,
            DEPART_MAX_MPS * 1.94384
        ));
    }
    retarget(Goal::Depart {
        height_ft,
        speed_mps: mps,
        track_deg,
    });
    Some(format!("{kts:.0} knots."))
}

/// The track of the moment slewed toward the wanted one, the short way, at
/// a standard-rate 3 degrees a second: the hover mode followed a rotating
/// target at 1.5 deg/s with 0.1 m/s of error (EDDV, 2026-09-23), and at the
/// ramp's rate the circuit's turns were 600 m wide. Slower at a walk, where
/// a 3 deg/s swing of a small vector is a sideways lurch.
pub fn slew_track(now_deg: f64, want_deg: f64, speed_mps: f64, dt_s: f64) -> f64 {
    let dt = dt_s.clamp(0.02, 0.5);
    let _ = speed_mps;
    let rate = TURN_RATE_DPS;
    let err = crate::angles::signed_diff_deg(want_deg, now_deg);
    (now_deg + err.clamp(-rate * dt, rate * dt)).rem_euclid(360.0)
}

const TURN_RATE_DPS: f64 = 3.0;

/// A scalar speed slewed at the ramp's acceleration.
fn slew_speed(now: f64, want: f64, dt_s: f64) -> f64 {
    let step = RAMP_ACCEL_MPS2 * dt_s.clamp(0.02, 0.5);
    now + (want - now).clamp(-step, step)
}

/// The pedals in FORWARD flight. In the hover the aircraft's own yaw hold
/// kept the heading to a tenth of a degree for minutes; flying 30 kt through
/// a turn it did not: the track swung 70 degrees and the nose 25, the S-76
/// flying 45 degrees sideways, then backwards (EDDV, 2026-09-23). Above
/// [`PEDALS_ON_MPS`] the nose is held on the track.
///
/// The S-76's pedal is a yaw-RATE command with a heading hold behind it,
/// measured in a 30 ft hover the same day: override at neutral held the
/// heading to 0.01 degrees for 8 s, 0.1 of pedal yawed at a steady 4.7 deg/s
/// within half a second, and on release the yaw stopped in half a second and
/// the new heading held. So the law asks for a RATE proportional to the
/// heading error and turns it into pedal; the aircraft does the damping. The
/// first law damped on the measured `R` at 0.05 a deg/s - a loop gain of
/// 2.4 through a quarter-second tick - and rang the tail at a couple of
/// hertz until the fault latch stood it down. A slow trim takes out a
/// standing need (a weathervane in forward flight).
#[derive(Debug, Clone, Copy)]
pub struct HeadingHold {
    trim: f64,
}

/// Yaw rate per unit of pedal, measured in the hover.
const YAW_DPS_PER_PEDAL: f64 = 47.0;
/// The yaw rate asked per degree of heading error, and its cap.
const YAW_RATE_PER_DEG: f64 = 0.5;
const YAW_RATE_MAX_DPS: f64 = 6.0;
const PEDAL_KI_PER_DEG_S: f64 = 0.002;
const PEDAL_TRIM_MAX: f64 = 0.1;
const PEDAL_MAX: f64 = 0.25;
/// The pedals are ours above this commanded speed, the aircraft's again
/// below the lower figure.
pub const PEDALS_ON_MPS: f64 = 8.0;
const PEDALS_OFF_MPS: f64 = 5.0;
/// A yaw rate past this with the pedals ours stands them down.
const PEDAL_FAULT_DPS: f64 = 25.0;

impl HeadingHold {
    pub fn new() -> Self {
        Self { trim: 0.0 }
    }
    pub fn trim(&self) -> f64 {
        self.trim
    }
    pub fn step(&mut self, target_deg: f64, heading_deg: f64, dt_s: f64) -> f64 {
        let dt = dt_s.clamp(0.02, 0.5);
        let err = crate::angles::signed_diff_deg(target_deg, heading_deg);
        if err.abs() < 10.0 {
            self.trim =
                (self.trim + err * PEDAL_KI_PER_DEG_S * dt).clamp(-PEDAL_TRIM_MAX, PEDAL_TRIM_MAX);
        }
        let rate = (err * YAW_RATE_PER_DEG).clamp(-YAW_RATE_MAX_DPS, YAW_RATE_MAX_DPS);
        (rate / YAW_DPS_PER_PEDAL + self.trim).clamp(-PEDAL_MAX, PEDAL_MAX)
    }
}

/// Change what a RUNNING loop flies (a departure, a stop out of one). A loop
/// that is not running is started by [`request`], never here.
pub fn retarget(g: Goal) {
    if ACTIVE.load(Ordering::SeqCst) {
        *lock_goal() = Some(g);
    }
}

/// The panel's line: what the hover loop is flying.
pub fn status() -> Option<String> {
    if !ACTIVE.load(Ordering::SeqCst) {
        return None;
    }
    Some(match goal()? {
        Goal::Hover { height_ft } => {
            let m = motion();
            if m.speed_mps > 0.0 {
                format!(
                    "air taxi {:.0} kt, {height_ft:.0} ft",
                    m.speed_mps * 1.94384
                )
            } else {
                format!("hover {height_ft:.0} ft")
            }
        }
        Goal::Land => "setting down".to_string(),
        Goal::Depart {
            height_ft,
            speed_mps,
            ..
        }
        | Goal::Fly {
            height_ft,
            speed_mps,
            ..
        } => format!(
            "flying {:.0} kt, {:.0} ft",
            speed_mps * 1.94384,
            (height_ft / 10.0).round() * 10.0
        ),
    })
}

/// Stand the hover down: the loop's next tick hands back the aircraft.
/// The rotor's rpm dataref (element 0) and governed speed, from the airframe
/// profile, for the droop guard, and the collective's dataref for callers
/// outside the hover tool (the taxi-in). `None` off a helicopter.
static ROTOR: Mutex<Option<(String, f64, String)>> = Mutex::new(None);

pub fn publish_rotor(r: Option<(String, f64, String)>) {
    *ROTOR.lock().unwrap_or_else(|e| e.into_inner()) = r;
}

/// The rotor facts [`publish_rotor`] takes, from a profile: `None` on
/// anything that is not a helicopter, so a swap to an aeroplane clears them
/// (`flight_state::publish_airframe`).
pub fn rotor_facts(p: &crate::aircraft::AircraftProfile) -> Option<(String, f64, String)> {
    p.heli_start.as_ref().map(|h| {
        (
            h.rotor_rpm.clone(),
            h.rotor_rpm_governed,
            h.collective.clone(),
        )
    })
}

fn rotor() -> Option<(String, f64)> {
    ROTOR
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|(d, g, _)| (d.clone(), *g))
}

/// The collective dataref of the helicopter loaded, `None` on anything else.
pub fn collective_dref() -> Option<String> {
    ROTOR
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|(_, _, c)| c.clone())
}

/// The rotor guard. Below [`DROOP_HOLD`] of governed the collective may not
/// rise; below [`DROOP_LOWER`] it comes down. The law had pulled 0.9 on a
/// heavy S-76 with one engine short of FLY and held the rotor at 83 % at five
/// feet (EDDV, 2026-09-23): a real rotor there is a helicopter falling out of
/// the sky, and more collective only makes it worse.
const DROOP_HOLD: f64 = 0.97;
const DROOP_LOWER: f64 = 0.94;
/// Drooped this long low down while lifting or hovering: not enough power,
/// set it down.
const DROOP_SET_DOWN_S: f64 = 5.0;

/// The collective ceiling for a rotor at `nr` (fraction of governed), given
/// the command of the moment and the tick.
pub fn droop_ceiling(nr: f64, cmd: f64, dt_s: f64) -> f64 {
    if nr >= DROOP_HOLD {
        COLLECTIVE_MAX
    } else if nr >= DROOP_LOWER {
        cmd
    } else {
        (cmd - RATE_DOWN * dt_s.clamp(0.02, 0.5)).max(0.0)
    }
}

/// A droop is SAID once it has lasted this long, or at once below
/// [`DROOP_LOWER`]. Every droop recorded with both engines at FLY was the
/// governor catching a load change: three in the hover taxis of 2026-09-23
/// (nr 0.970 to 0.974 at 0.69 collective, one of them at hilltop EDVW), each
/// back above 97 % inside 1.4 s, each announced as "Rotor's drooping" while
/// nothing was wrong. The ceiling still holds the lever from the first tick.
const DROOP_SPEAK_S: f64 = 2.0;

/// What a droop of `since_s` seconds earns this tick.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct DroopCall {
    pub speak: bool,
    /// Airborne and low: set it down.
    pub set_down: bool,
    /// Still on the ground, lifting: the rotor droops before the wheels are
    /// light, so the lift is given up and the lever comes down. Before, the
    /// ramp sat at the guard's ceiling on a drooped rotor for as long as the
    /// pilot let it: the set-down was only ever for an aircraft in the air.
    pub abandon_lift: bool,
}

/// `lifting`: on the ground with the ramp asking for a climb.
pub fn droop_call(
    nr: f64,
    since_s: f64,
    on_ground: bool,
    lifting: bool,
    h_ft: f64,
    landing: bool,
) -> DroopCall {
    if nr >= DROOP_HOLD || !nr.is_finite() {
        return DroopCall::default();
    }
    let long = since_s > DROOP_SET_DOWN_S && !landing;
    DroopCall {
        speak: since_s >= DROOP_SPEAK_S || nr < DROOP_LOWER,
        set_down: long && !on_ground && h_ft < MAX_HOVER_FT,
        abandon_lift: long && on_ground && lifting,
    }
}

/// The line for a droop the power cannot answer: what the rotor fell to, on
/// how much lever, in what air.
pub fn power_short_line(on_ground: bool, nr_min: f64, collective: f64, air: Option<Air>) -> String {
    let air = air
        .filter(Air::valid)
        .map(|a| {
            format!(
                ", density altitude {} feet at {} kilograms",
                crate::answers::with_commas(round_to(a.density_alt_ft(), 100.0)),
                crate::answers::with_commas(round_to(a.weight_kg, 10.0)),
            )
        })
        .unwrap_or_default();
    let what = format!(
        "rotor at {:.0} percent on {:.0} percent collective{air}",
        nr_min * 100.0,
        collective * 100.0
    );
    if on_ground {
        format!("Not enough power to lift: {what}. Collective down.")
    } else {
        format!("Not enough power: {what}. Setting it down.")
    }
}

/// Set by [`release`]: the loop's next tick ends WITHOUT turning the
/// autopilot off, for a caller about to put it in its cruise modes.
static RELEASED: AtomicBool = AtomicBool::new(false);

/// End the loop and leave the autopilot and the collective as they are (the
/// route's hand-over to the cruise modes).
pub fn release() {
    if RUN.live() {
        RELEASED.store(true, Ordering::SeqCst);
        RUN.cancel();
    }
}

pub fn abort() {
    RUN.cancel();
}

/// The autopilot tools that select a MODE, and so would replace the hover
/// mode the loop is flying on: "turn heading two hundred" at 50 ft engaged
/// heading mode, the hover mode dropped, and the S-76 flew forward out of
/// the hover at 15 kt (2026-09-22).
const MODE_TOOLS: &[&str] = &[
    "set_heading",
    "navigate",
    "set_altitude",
    "level_off",
    "set_vertical_speed",
    "set_climb",
    "set_speed",
    "autothrottle",
    "autopilot_off",
];

/// The refusal for a tool the running hover cannot share the autopilot
/// with; `None` when no hover is running or the tool is not a mode.
pub fn blocks(tool: &str) -> Option<&'static str> {
    (ACTIVE.load(Ordering::SeqCst) && MODE_TOOLS.contains(&tool)).then_some(
        "I'm holding the hover on the autopilot. Say a hover height, set it down, \
         or my controls.",
    )
}

// ---------------------------------------------------------------------------
// The law
// ---------------------------------------------------------------------------

/// Collective per 1,000 fpm of vertical-speed error.
const KP_PER_1000FPM: f64 = 0.3;
/// The trim's rate: the standing collective the hover needs. It has to CHASE
/// that figure, which rises as the rotor climbs out of its ground cushion
/// (0.555 at lift-off, 0.582 at nine feet on the S-76): at 0.04 it trailed,
/// and the first live lift crept from three feet to nine in 45 s.
const KI_PER_1000FPM_S: f64 = 0.12;
/// Collective slew limits, per second. A collective jerked is a yaw kick and
/// a balloon; these are a pilot's hand.
const RATE_UP: f64 = 0.08;
const RATE_DOWN: f64 = 0.12;
/// The trim's band round the power curve's level figure, the lever's floor
/// under it, and how far from the curve a hand-back may leave the lever.
const TRIM_BELOW_LEVEL: f64 = 0.15;
const TRIM_ABOVE_LEVEL: f64 = 0.20;
const LEVER_BELOW_BAND: f64 = 0.05;
const STEADY_BAND: f64 = 0.08;
/// The ramp that finds lift-off from a flat pitch.
const LIFT_RAMP: f64 = 0.04;
/// Groundspeed under which a landing's parking brake is set, m/s (2 kt).
/// Not `sim::ground::STOPPED_KTS` on purpose: a rotor still carrying weight
/// creeps on its wheels (a knot and a half at LILO, see [`LOWER_RATE`]), and
/// the brake is what stops that creep.
const TOUCHDOWN_BRAKE_MAX_MPS: f64 = 1.0;

/// Whether a landing on the ground at `gs` m/s may set the parking brake.
pub fn touchdown_brake_due(gs: f64) -> bool {
    gs.is_finite() && gs < TOUCHDOWN_BRAKE_MAX_MPS
}

/// Once on the ground after a landing, the collective comes down at this:
/// about 3 s from a hover's 0.6. At 0.10 it took six, and the S-76 sat light
/// on its wheels for all of them, rolling at a knot and a half on the second
/// set-down at LILO ("not entirely firmly on the ground", 2026-09-24).
const LOWER_RATE: f64 = 0.20;
pub const COLLECTIVE_MAX: f64 = 0.9;
/// A climb this fast, or leaving the ground, ends the lift-off ramp.
const LIFTED_FPM: f64 = 60.0;

#[derive(Debug, Clone, Copy, Default)]
pub struct CollectiveLaw {
    cmd: f64,
    trim: f64,
    airborne: bool,
    /// The rotor guard's ceiling this tick (see [`droop_ceiling`]).
    ceiling: Option<f64>,
    /// Airborne, the power curve's figure for the speed of the moment: the
    /// trim is held to a band around it and the lever never far under it.
    level: Option<f64>,
    /// On the ground, lifting: the lever goes here at the up-rate before the
    /// lift-off ramp takes over (see [`PowerBudget::prelift`]).
    prelift: Option<f64>,
}

impl CollectiveLaw {
    /// Start from the collective the aircraft has now.
    pub fn from(collective: f64) -> Self {
        Self {
            cmd: collective,
            trim: collective,
            airborne: false,
            ceiling: None,
            level: None,
            prelift: None,
        }
    }

    /// Where a lift-off from the ground starts its ramp.
    pub fn set_prelift(&mut self, p: Option<f64>) {
        self.prelift = p;
    }

    pub fn airborne(&self) -> bool {
        self.airborne
    }
    pub fn trim(&self) -> f64 {
        self.trim
    }

    /// Move the trim by a known change in the power needed (a change of
    /// speed along the power curve), ahead of the error that would find it.
    /// The power curve's level figure for this tick (`None` on the ground).
    pub fn set_level(&mut self, l: Option<f64>) {
        self.level = l;
    }
    /// The collective a hand-back leaves: the trim, but never far from the
    /// power curve.
    pub fn steady(&self) -> f64 {
        match self.level {
            Some(l) => self.trim.clamp(l - STEADY_BAND, l + STEADY_BAND),
            None => self.trim,
        }
    }

    pub fn set_ceiling(&mut self, c: Option<f64>) {
        self.ceiling = c;
    }
    pub fn cmd(&self) -> f64 {
        self.cmd
    }

    pub fn shift_trim(&mut self, by: f64) {
        if self.airborne {
            self.trim = (self.trim + by).clamp(0.0, COLLECTIVE_MAX);
        }
    }

    pub fn step(&mut self, target_vs_fpm: f64, vs_fpm: f64, on_ground: bool, dt_s: f64) -> f64 {
        let dt = dt_s.clamp(0.02, 0.5);
        if on_ground && !self.airborne {
            if target_vs_fpm > 0.0 {
                // Finding lift-off: up to the budget's start at a pilot's
                // hand, then a steady ramp, the trim riding with it.
                let rate = if self.prelift.is_some_and(|p| self.cmd < p) {
                    PRELIFT_RATE
                } else {
                    LIFT_RAMP
                };
                let top = self
                    .prelift
                    .filter(|p| self.cmd < *p)
                    .unwrap_or(COLLECTIVE_MAX);
                self.cmd = (self.cmd + rate * dt)
                    .min(top)
                    .min(COLLECTIVE_MAX)
                    .min(self.ceiling.unwrap_or(COLLECTIVE_MAX));
                self.trim = self.cmd;
                if vs_fpm > LIFTED_FPM {
                    self.airborne = true;
                }
            } else {
                // Down and staying down: lower the lever.
                self.cmd = (self.cmd - LOWER_RATE * dt).max(0.0);
                self.trim = self.cmd;
            }
            return self.cmd;
        }
        if !on_ground {
            self.airborne = true;
        } else if target_vs_fpm <= 0.0 {
            // Touched down on a descent: the landing is over.
            self.airborne = false;
            self.cmd = (self.cmd - LOWER_RATE * dt).max(0.0);
            self.trim = self.cmd;
            return self.cmd;
        }
        let err = (target_vs_fpm - vs_fpm) / 1000.0;
        self.trim = (self.trim + err * KI_PER_1000FPM_S * dt).clamp(0.0, COLLECTIVE_MAX);
        // The trim is the level figure, found: it may wander round the power
        // curve, never across it. At ETHC (2026-09-23) a landing asked in a
        // 1,700 fpm climb integrated it to 0.0, the lever followed to 0.17,
        // the sink built to 3,100 fpm and the up-rate could not catch it;
        // the takeover before it had wound it to 0.9 in two seconds.
        let floor = self.level.map(|l| l - TRIM_BELOW_LEVEL);
        if let Some(l) = self.level {
            self.trim = self.trim.clamp(l - TRIM_BELOW_LEVEL, l + TRIM_ABOVE_LEVEL);
        }
        let want = (self.trim + err * KP_PER_1000FPM)
            .clamp(floor.map_or(0.0, |f| f - LEVER_BELOW_BAND), COLLECTIVE_MAX);
        let step = (want - self.cmd).clamp(-RATE_DOWN * dt, RATE_UP * dt);
        self.cmd += step;
        if let Some(c) = self.ceiling {
            // The rotor guard: the lever no higher, and a trim that does not
            // wind up against it.
            self.cmd = self.cmd.min(c);
            self.trim = self.trim.min(c);
        }
        self.cmd
    }
}

/// The collective level flight needs at a groundspeed: the S-76's power
/// curve, measured on the flights of 2026-09-23 at EDDV (0.66 in a hover
/// out of ground effect at 300 ft, 0.46 at 30 kt, 0.44 at 40, 0.42 at 72),
/// the knee between them the translational lift. Only its SLOPE is used,
/// to move the trim as the commanded speed changes: a takeover decelerating
/// from 35 kt at a cruise 0.30 left the trim to find 0.66 through the error,
/// the sink built to 300 fpm, the law overshot to 0.74 and the collective
/// swings pitched the nose from 16 degrees to -5 (EDDV, same day). Per
/// airframe when a second helicopter flies.
const POWER_CURVE: &[(f64, f64)] = &[
    (0.0, 0.66),
    (5.0, 0.62),
    (8.0, 0.55),
    (12.0, 0.49),
    (15.4, 0.46),
    (21.0, 0.44),
    (37.0, 0.42),
];

pub fn level_collective(speed_mps: f64) -> f64 {
    let s = speed_mps.max(0.0);
    for w in POWER_CURVE.windows(2) {
        let ((s0, c0), (s1, c1)) = (w[0], w[1]);
        if s <= s1 {
            return c0 + (c1 - c0) * (s - s0) / (s1 - s0);
        }
    }
    POWER_CURVE.last().map_or(0.5, |p| p.1)
}

// ---------------------------------------------------------------------------
// The power budget: density altitude and weight
// ---------------------------------------------------------------------------
//
// The curve and the lift-off figures were measured light and low (EDDV,
// ~160-300 ft). At hilltop EDVW (1,298 ft, heavy, 2026-09-23) the same S-76
// lifted off at 0.644 against 0.555 and needed 0.683 at ten feet against
// 0.580: the ramp spent 16 s finding the lift-off from a flat pitch, the hover
// crept from five feet to nine in twelve seconds and never made its twelve,
// and the pilot took it ("bring us down"). Every figure the loop owns (the
// lift-off, the trim band, a takeover's start, a hand-back's lever) was the
// light aircraft's.
//
// Thrust at a governed rotor speed goes with the density and the pitch, so
// the lever a hover needs goes with weight over density ratio (W/sigma). The
// 30 lift-offs recorded 2026-09-22..25 carry it: 0.711 at LSZS (5,605 ft)
// against 0.604 at the same fuel low down, 1.18 where ISA density says 1.18;
// the lever, not the acf's blade pitch (-2.2 to 10 degrees), is what scales.
// No OAT or payload was logged, so the fit is at ISA with a 170 kg crew.

/// The lift-off and the nine-foot hover, found by the lift-off ramp at
/// 163 ft with 325 kg of fuel (2026-09-22): 3,435 kg empty + fuel + an
/// assumed 170 kg crew.
const LIFTOFF_REF: f64 = 0.555;
const IGE_REF: f64 = 0.582;
const IGE_REF_WEIGHT_KG: f64 = 3930.0;
const IGE_REF_SIGMA: f64 = 0.995;
/// The power curve's own loading: the flights of 2026-09-23 at ~300 ft with
/// ~575 kg of fuel. An ESTIMATE (the payload was never logged); the lift-off
/// corrects it within the hover ([`LoadingEstimate`]) and the `liftoff`
/// record calibrates it for good.
const CURVE_WEIGHT_KG: f64 = 4180.0;
const CURVE_SIGMA: f64 = 0.991;
/// The lift-off figures' loading on the curve's scale.
const IGE_REF_LOADING: f64 = (IGE_REF_WEIGHT_KG / IGE_REF_SIGMA) / (CURVE_WEIGHT_KG / CURVE_SIGMA);
/// The most lever the hover may plan on: the law's own ceiling less room for
/// the height loop to correct with.
const HOVER_HEADROOM: f64 = 0.05;
/// A hover this close to the most lever is tight (blackboxed, not said).
const TIGHT_MARGIN: f64 = 0.05;
/// A hover higher than this is out of the ground cushion: at a loading whose
/// out-of-ground-effect hover is beyond the lever, a "hover at N" is held here.
pub const GROUND_CUSHION_FT: f64 = 20.0;
/// The ramp starts this far under the predicted lift-off, reached at the
/// law's up-rate, so a heavy rotor does not spend 16 s at 0.04/s finding it
/// and a light one is not thrown off the ground by a prediction too high (the
/// recorded lift-offs scatter 0.03 round the fit).
const PRELIFT_BELOW: f64 = 0.08;
/// The lever's rate up to the prelift: on the wheels, under the lift-off, the
/// lever moves nothing but the rotor's pitch; the down-rate's figure.
const PRELIFT_RATE: f64 = RATE_DOWN;
/// A loading outside this is a bad read, never a helicopter.
const LOADING_MIN: f64 = 0.6;
const LOADING_MAX: f64 = 1.8;

/// The air and the weight the hover flies in.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Air {
    pub pressure_alt_ft: f64,
    pub oat_c: f64,
    pub weight_kg: f64,
}

impl Air {
    /// Readings that could be an aircraft in the atmosphere. Anything else is
    /// no measurement.
    pub fn valid(&self) -> bool {
        self.pressure_alt_ft.is_finite()
            && self.oat_c.is_finite()
            && self.weight_kg.is_finite()
            && (-2_000.0..40_000.0).contains(&self.pressure_alt_ft)
            && (-70.0..60.0).contains(&self.oat_c)
            && self.weight_kg > 300.0
    }
    /// Density ratio from the pressure altitude (ISA pressure) and the
    /// outside air temperature.
    pub fn sigma(&self) -> f64 {
        let delta = (1.0 - 6.875_6e-6 * self.pressure_alt_ft)
            .max(0.01)
            .powf(5.2559);
        let theta = (self.oat_c + 273.15) / 288.15;
        delta / theta
    }
    /// The ISA altitude of that density, the figure a helicopter pilot plans
    /// a hover by.
    pub fn density_alt_ft(&self) -> f64 {
        (1.0 - self.sigma().max(0.01).powf(1.0 / 4.2559)) / 6.875_6e-6
    }
    /// Weight over density ratio, as a ratio to the power curve's own.
    pub fn loading(&self) -> f64 {
        (self.weight_kg / self.sigma()) / (CURVE_WEIGHT_KG / CURVE_SIGMA)
    }
}

/// The lever a curve figure needs at `loading` (1.0 is the curve's own).
pub fn scale_lever(lever_ref: f64, loading: f64) -> f64 {
    lever_ref * loading
}

/// The loading a measured lever says, against the figure the curve had for
/// the same state. `None` when the figures cannot be a loading.
pub fn loading_from(lever: f64, lever_ref: f64) -> Option<f64> {
    (lever.is_finite() && lever_ref > 0.0)
        .then(|| lever / lever_ref)
        .filter(|l| (LOADING_MIN..=LOADING_MAX).contains(l))
}

/// [`level_collective`] at a loading.
pub fn level_collective_at(speed_mps: f64, loading: f64) -> f64 {
    scale_lever(level_collective(speed_mps), loading).clamp(0.0, COLLECTIVE_MAX)
}

/// What this hover's loading is best known as: predicted from the air and
/// the weight, and once the rotor has lifted, MEASURED by the lever it lifted
/// at. The measurement then carries the prediction's changes (fuel burnt, a
/// climb) as a ratio, so a wrong reference weight is corrected by the first
/// lift-off and never again leans on the estimate.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct LoadingEstimate {
    predicted: Option<f64>,
    /// (measured at lift-off, the prediction of that moment)
    lifted: Option<(f64, Option<f64>)>,
}

impl LoadingEstimate {
    pub fn predict(&mut self, air: Option<Air>) {
        if let Some(l) = air
            .filter(Air::valid)
            .map(|a| a.loading())
            .filter(|l| (LOADING_MIN..=LOADING_MAX).contains(l))
        {
            self.predicted = Some(l);
        }
    }
    /// The rotor lifted at `lever`: the loading it says, if it can be one.
    pub fn lifted_at(&mut self, lever: f64) -> Option<f64> {
        let m = loading_from(lever, LIFTOFF_REF / IGE_REF_LOADING)?;
        self.lifted = Some((m, self.predicted));
        Some(m)
    }
    pub fn predicted(&self) -> Option<f64> {
        self.predicted
    }
    /// The loading of the moment; the curve's own (1.0) when nothing is known,
    /// which is the loop as it was.
    pub fn now(&self) -> f64 {
        let l = match (self.lifted, self.predicted) {
            (Some((m, Some(then))), Some(now)) => m * now / then,
            (Some((m, _)), _) => m,
            (None, Some(p)) => p,
            (None, None) => 1.0,
        };
        l.clamp(LOADING_MIN, LOADING_MAX)
    }
}

/// The height the ground cushion is gone by (about the S-76's rotor
/// diameter, 44 ft).
const GE_FADE_FT: f64 = 40.0;
/// The cushion's feed-forward works only this slow: faster, the power
/// curve's own knee is the change that matters.
const GE_FF_MAX_MPS: f64 = 2.0;

/// The lever a hover over the spot needs at `h_ft` and `loading`: the
/// lift-off on the wheels, the nine-foot figure, the curve's out-of-ground-
/// effect hover by `GE_FADE_FT`. At EDVW the need rose 0.039 between the
/// lift-off and ten feet, which the trim found at its integral rate: five
/// feet to nine took twelve seconds.
pub fn hover_need(h_ft: f64, loading: f64) -> f64 {
    let lift = LIFTOFF_REF / IGE_REF_LOADING;
    let ige = IGE_REF / IGE_REF_LOADING;
    let oge = level_collective(0.0);
    let h = h_ft.max(0.0);
    let need = if h <= 9.0 {
        lift + (ige - lift) * h / 9.0
    } else if h < GE_FADE_FT {
        ige + (oge - ige) * (h - 9.0) / (GE_FADE_FT - 9.0)
    } else {
        oge
    };
    need * loading
}

/// The power figures of one hover, tick by tick: the loading it believes,
/// the power curve's and the ground cushion's feed-forward into the trim,
/// the band, and the lift-off that measures the loading. The loop and the
/// tests fly the same one.
#[derive(Debug, Clone, Copy, Default)]
pub struct PowerFeed {
    pub loading: LoadingEstimate,
    prev_speed: Option<f64>,
    prev_h: Option<f64>,
    was_airborne: bool,
    was_lifting: bool,
}

/// A lift-off, measured.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LiftOff {
    pub lever: f64,
    pub predicted: Option<f64>,
    pub measured: Option<f64>,
}

impl PowerFeed {
    pub fn new(loading: LoadingEstimate, law: &CollectiveLaw) -> Self {
        Self {
            loading,
            was_airborne: law.airborne(),
            ..Self::default()
        }
    }

    /// Before the law steps: move the trim by the known changes in the need
    /// (speed along the curve; height through the cushion, over the spot)
    /// and set the band. Returns the band's level figure.
    pub fn before(
        &mut self,
        law: &mut CollectiveLaw,
        cmd_speed: f64,
        speed_h: Option<f64>,
        h_ft: f64,
        on_ground: bool,
    ) -> Option<f64> {
        let l = self.loading.now();
        if let Some(prev) = self.prev_speed {
            law.shift_trim(level_collective_at(cmd_speed, l) - level_collective_at(prev, l));
        }
        self.prev_speed = Some(cmd_speed);
        let hovering = !on_ground && law.airborne() && cmd_speed < GE_FF_MAX_MPS;
        if let (true, Some(prev)) = (hovering, self.prev_h) {
            law.shift_trim(hover_need(h_ft, l) - hover_need(prev, l));
        }
        self.prev_h = hovering.then_some(h_ft);
        // The band must surround the same equilibrium that feeds the trim.
        // Over the spot that is the ground-effect curve, not the OGE end of
        // the forward-flight curve.  Keeping the band on OGE while shifting
        // the trim down the cushion made the two protections fight: a heavy
        // S-76 reached 2.8 ft with -100 fpm commanded, but the OGE floor held
        // enough collective to hover there indefinitely (EGBJ, 2026-10-01).
        // In forward flight the translational-lift curve remains the level
        // reference, including while the aircraft is still decelerating.
        let level = (!on_ground && law.airborne()).then(|| {
            if hovering {
                hover_need(h_ft, l)
            } else {
                level_collective_at(cmd_speed.max(speed_h.unwrap_or(0.0)), l)
            }
        });
        law.set_level(level);
        level
    }

    /// After the law stepped from `lever_before`: a lift-off this tick is
    /// measured (only a lift the ramp made: a bounce is not one).
    pub fn after(
        &mut self,
        law: &CollectiveLaw,
        lever_before: f64,
        lifting: bool,
    ) -> Option<LiftOff> {
        let lifted = law.airborne() && !self.was_airborne && (lifting || self.was_lifting);
        self.was_airborne = law.airborne();
        self.was_lifting = lifting;
        lifted.then(|| {
            let predicted = self.loading.predicted();
            LiftOff {
                lever: lever_before,
                predicted,
                measured: self.loading.lifted_at(lever_before),
            }
        })
    }
}

/// What the hover needs at a loading, against what the lever has.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PowerBudget {
    pub loading: f64,
    pub liftoff: f64,
    /// In ground effect: a hover taxi's height.
    pub ige: f64,
    /// Out of ground effect.
    pub oge: f64,
    pub available: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PowerVerdict {
    Fine,
    /// The out-of-ground-effect hover is within `TIGHT_MARGIN` of the lever.
    Tight,
    /// A hover taxi is flyable; a hover out of the ground cushion is not.
    NoOge,
    /// Not even a hover in the ground cushion.
    NoHover,
}

impl PowerBudget {
    pub fn at(loading: f64) -> Self {
        Self {
            loading,
            liftoff: scale_lever(LIFTOFF_REF / IGE_REF_LOADING, loading),
            ige: scale_lever(IGE_REF / IGE_REF_LOADING, loading),
            oge: scale_lever(level_collective(0.0), loading),
            available: COLLECTIVE_MAX - HOVER_HEADROOM,
        }
    }
    pub fn verdict(&self) -> PowerVerdict {
        if self.ige > self.available {
            PowerVerdict::NoHover
        } else if self.oge > self.available {
            PowerVerdict::NoOge
        } else if self.oge > self.available - TIGHT_MARGIN {
            PowerVerdict::Tight
        } else {
            PowerVerdict::Fine
        }
    }
    /// Where the lift-off ramp starts.
    pub fn prelift(&self) -> f64 {
        (self.liftoff - PRELIFT_BELOW).clamp(0.0, self.available)
    }
    /// The spoken line for a hover the lever cannot give, with its numbers.
    pub fn line(&self, air: Option<Air>) -> Option<String> {
        let pct = |c: f64| (c * 100.0).round();
        let why = match air.filter(Air::valid) {
            Some(a) => format!(
                "density altitude {} feet at {} kilograms",
                crate::answers::with_commas(round_to(a.density_alt_ft(), 100.0)),
                crate::answers::with_commas(round_to(a.weight_kg, 10.0)),
            ),
            None => "at this weight".to_string(),
        };
        match self.verdict() {
            PowerVerdict::NoHover => Some(format!(
                "Not enough power to hover here: {why} needs {:.0} percent collective, and {:.0} is all we have.",
                pct(self.ige),
                pct(self.available)
            )),
            PowerVerdict::NoOge => Some(format!(
                "No hover out of ground effect here: {why} needs {:.0} percent collective, and {:.0} is all we have. Keeping it low.",
                pct(self.oge),
                pct(self.available)
            )),
            _ => None,
        }
    }
    pub fn record(&self, air: Option<Air>, event: &str) -> serde_json::Value {
        let air = air.filter(Air::valid);
        serde_json::json!({
            "event": event,
            "pa_ft": air.map(|a| a.pressure_alt_ft),
            "oat_c": air.map(|a| a.oat_c),
            "weight_kg": air.map(|a| a.weight_kg),
            "sigma": air.map(|a| a.sigma()),
            "da_ft": air.map(|a| a.density_alt_ft()),
            "loading": self.loading,
            "need_liftoff": self.liftoff,
            "need_ige": self.ige,
            "need_oge": self.oge,
            "available": self.available,
            "verdict": format!("{:?}", self.verdict()),
        })
    }
}

fn round_to(v: f64, step: f64) -> f64 {
    (v / step).round() * step
}

/// Climb rate per foot of height error, and the limits either way.
pub const HEIGHT_GAIN_FPM_PER_FT: f64 = 10.0;
const MAX_CLIMB_FPM: f64 = 300.0;
const MAX_DESCENT_FPM: f64 = 250.0;
/// The last few feet of a landing, and the rate they are flown at.
const TOUCHDOWN_BAND_FT: f64 = 4.0;
const TOUCHDOWN_FPM: f64 = 100.0;

/// A departure climbs only above this groundspeed (translational lift: a
/// climb from a hover low down is the part of the height-velocity diagram a
/// helicopter pilot avoids), at this rate.
pub const DEPART_CLIMB_MPS: f64 = 8.0;
const DEPART_CLIMB_FPM: f64 = 500.0;
/// A flown vector's steepest descent: a six-degree final at 30 kt is 310 fpm.
const FLY_DESCENT_FPM: f64 = 500.0;
/// A landing asked while moving holds its height until slowed to this.
const LAND_SLOWED_MPS: f64 = 1.5;

/// `speed_mps`: the COMMANDED groundspeed of the moment (the ramp's).
pub fn target_vs_fpm(goal: Goal, height_ft: f64, speed_mps: f64) -> f64 {
    match goal {
        Goal::Hover { height_ft: want } => {
            ((want - height_ft) * HEIGHT_GAIN_FPM_PER_FT).clamp(-MAX_DESCENT_FPM, MAX_CLIMB_FPM)
        }
        Goal::Land if speed_mps > LAND_SLOWED_MPS => 0.0,
        Goal::Land if height_ft <= TOUCHDOWN_BAND_FT => -TOUCHDOWN_FPM,
        Goal::Land => -(height_ft * HEIGHT_GAIN_FPM_PER_FT).clamp(TOUCHDOWN_FPM, MAX_DESCENT_FPM),
        Goal::Depart { .. } if speed_mps < DEPART_CLIMB_MPS => 0.0,
        Goal::Depart {
            height_ft: want, ..
        } => {
            ((want - height_ft) * HEIGHT_GAIN_FPM_PER_FT).clamp(-MAX_DESCENT_FPM, DEPART_CLIMB_FPM)
        }
        Goal::Fly {
            height_ft: want, ..
        } => {
            ((want - height_ft) * HEIGHT_GAIN_FPM_PER_FT).clamp(-FLY_DESCENT_FPM, DEPART_CLIMB_FPM)
        }
    }
}

/// The hover mode's standing error, taken out of its target. It holds a
/// groundspeed VECTOR, and asked for zero it let the S-76 run forward at
/// 4 to 8 kt, nose low, with no wind (EDDV, 2026-09-23). The target is
/// biased against the drift actually measured (east and south, the frame
/// `hover_vector` is in), slowly, so the mode stays the inner loop.
#[derive(Debug, Clone, Copy, Default)]
pub struct DriftTrim {
    x: f64,
    z: f64,
}

const DRIFT_KI_PER_S: f64 = 0.05;
const DRIFT_TRIM_MAX_MPS: f64 = 3.0;

impl DriftTrim {
    pub fn target(&self) -> [f64; 2] {
        [self.x, self.z]
    }
    /// `vx`, `vz`: east and south velocity, m/s. On the ground the trim
    /// resets: nothing is drifting and the next lift starts clean.
    pub fn step(&mut self, vx: f64, vz: f64, on_ground: bool, dt_s: f64) -> [f64; 2] {
        if on_ground {
            *self = Self::default();
            return self.target();
        }
        let dt = dt_s.clamp(0.02, 0.5);
        self.x -= vx * DRIFT_KI_PER_S * dt;
        self.z -= vz * DRIFT_KI_PER_S * dt;
        let m = self.x.hypot(self.z);
        if m > DRIFT_TRIM_MAX_MPS {
            self.x *= DRIFT_TRIM_MAX_MPS / m;
            self.z *= DRIFT_TRIM_MAX_MPS / m;
        }
        self.target()
    }
}

// ---------------------------------------------------------------------------
// The loop
// ---------------------------------------------------------------------------

/// Pressure altitude (a double): with the OAT it is the density the power
/// budget is flown by.
const PRESSURE_ALT: &str = "sim/flightmodel2/position/pressure_altitude";
/// How often the air and the weight are read again in flight (fuel burns, a
/// departure climbs).
const AIR_REFRESH_S: f64 = 10.0;
const Y_AGL: &str = "sim/flightmodel/position/y_agl";
const VS_FPM: &str = "sim/flightmodel/position/vh_ind_fpm";
const GROUNDSPEED: &str = "sim/flightmodel/position/groundspeed";
const THETA: &str = "sim/flightmodel/position/theta";
const PHI: &str = "sim/flightmodel/position/phi";
const AP_ON: &str = autoflight::names::AP_ON;
const HOVER_STATUS: &str = autoflight::names::HOVER_STATUS;
const VX: &str = "sim/flightmodel/position/local_vx";
const VZ: &str = "sim/flightmodel/position/local_vz";
const PSI: &str = "sim/flightmodel/position/psi";
const YAW_RATE: &str = "sim/flightmodel/position/R";
/// The plugin's rudder channel (the taxi's steering): an expiring heartbeat
/// written every tick, dropped the frame the pilot's yaw axis moves.
const PEDALS: &str = "xcopilot/taxi_steer";
const PEDAL_YIELD: &str = "xcopilot/steer_yield";
const HEADING_STATUS: &str = autoflight::names::HDG_STATUS;
const ALT_STATUS: &str = autoflight::names::ALT_HOLD_STATUS;
/// The disconnect that works on the S-76 (plain `servos_off` did not).
const M_TO_FT: f64 = 3.28084;
/// The radio height the gear reads at rest (0.027 m on the S-76).
const REST_AGL_FT: f64 = 0.1;
/// Past this the hover is not a hover: hand back.
const MAX_ATTITUDE_DEG: f64 = 20.0;
/// Drifting past this, a hover becomes a set-down; past twice it, even a
/// set-down is not flying and the aircraft is handed back.
const MAX_DRIFT_MPS: f64 = 8.0;
/// A departure is flying its command within this, or it is stopped.
const DEPART_MAX_ERR_MPS: f64 = 5.0;
/// The fastest departure and the highest. The hover mode flew a 40 kt
/// target at 44 kt and let go at 47 (hover_status 0, and it would not
/// re-engage at that speed): EDDV, 2026-09-23. Faster is the cruise modes'.
pub const DEPART_MAX_MPS: f64 = 20.6;
pub const DEPART_MAX_FT: f64 = 1000.0;
/// The hover mode is asked to capture again at most this often.
const RECAPTURE_EVERY_S: f64 = 4.0;
/// The hover mode's groundspeed target further than this from the trim's
/// is not ours, and is put back.
const VECTOR_SLOP_MPS: f64 = 0.05;
/// An airborne request takes the aircraft over only this slow: the hover mode
/// captures the velocity it is engaged at and the loop ramps from there. The
/// mode flew 44 kt and let go at 47 (EDDV, 2026-09-23).
pub const TAKEOVER_MAX_MPS: f64 = 20.0;
/// Hover and landing are both low-level manoeuvres: a hover asked above
/// this is refused rather than flown on a law tested this low.
pub const MAX_HOVER_FT: f64 = 50.0;
const TICK: Duration = Duration::from_millis(100);
/// Stopped on the ground this long after a landing = done.
const LANDED_SECS: f64 = 2.0;

/// Ask for a hover or a landing. Starts the loop if it is not running (the
/// caller has checked the airframe and the ground), else retargets it.
pub fn request(base_url: String, voice: Option<Voice>, collective: String, g: Goal) {
    *lock_goal() = Some(g);
    let Some(run) = RUN.claim() else {
        return; // running: the next tick flies the new goal
    };
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            return;
        };
        rt.block_on(fly(XPlaneClient::new(base_url), voice, collective, &run));
        // The goal is this loop's to drop only while it owns the run: a
        // loop started after a cancel flies its own.
        if run.current() {
            *lock_goal() = None;
        }
    });
}

fn say(voice: &Option<Voice>, p: Priority, text: impl Into<String>) {
    if let Some(v) = voice {
        v.alert(p, text);
    }
}

/// What a drift earns. The hover mode is a groundspeed HOLD
/// (`hover_status`: "hold 2D groundspeed") and captures the velocity it is
/// engaged at: the recapture at 6.8 m/s told it to hold 6.8 m/s, and the
/// hand-back that followed left an unstabilised S-76 at 15 ft with nobody
/// able to hold it (EDDV, 2026-09-23). Low down the answer to a drift is the
/// ground, with the autopilot still on the cyclic; the pilot gets it only
/// when even the set-down is not flying.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Drift {
    Fine,
    Stop,
    SetDown,
    HandBack,
}

/// `err_mps`: how far the groundspeed is from the one commanded (the plain
/// groundspeed in a hover over the spot).
/// How far a flown vector may be off its command before it is stopped. A
/// slow one (a hover taxi) gets more: turning a taxiway corner at 10 kt the
/// aircraft lags the turned command by more than 5 m/s and the route stopped
/// itself into a hover at every corner (ETNW, 2026-09-23).
fn stop_err_mps(goal: Goal) -> f64 {
    match goal.vector() {
        Some((speed, _)) if speed < SLOW_VECTOR_MPS => SLOW_STOP_ERR_MPS,
        _ => DEPART_MAX_ERR_MPS,
    }
}
const SLOW_VECTOR_MPS: f64 = 8.0;
const SLOW_STOP_ERR_MPS: f64 = 9.0;

/// For the TOWER, a helicopter under this height is on the ground: a hover
/// taxi on a taxi clearance lifted the wheels, and the tower's compliance
/// watch said "you were not cleared for takeoff" (ETNW, 2026-09-23). A climb
/// through it is the takeoff.
pub const TOWER_GROUND_AGL_FT: f64 = 100.0;

pub fn tower_on_ground(on_ground: bool, agl_ft: f64) -> bool {
    on_ground || (collective_dref().is_some() && agl_ft < TOWER_GROUND_AGL_FT)
}

pub fn drift_action(goal: Goal, err_mps: f64) -> Drift {
    match goal {
        _ if err_mps > 2.0 * MAX_DRIFT_MPS => Drift::HandBack,
        Goal::Hover { .. } if err_mps > MAX_DRIFT_MPS => Drift::SetDown,
        // A departure that stops following its command is stopped: the
        // hover at the height it reached, never a descent at speed.
        Goal::Depart { .. } | Goal::Fly { .. } if err_mps > stop_err_mps(goal) => Drift::Stop,
        _ => Drift::Fine,
    }
}

/// A new KIND of goal settles before "stopped following" may judge it. The
/// route handed its final to the loop at 35 kt, the hover mode was still
/// capturing the vector 3.5 s later (5.2 m/s off), the Stop fired, and the
/// route read the loop's own hover as someone else's and ended 1.3 km short
/// of LIME's pad (2026-09-25). The set-down and hand-back limits hold
/// throughout. Keyed on the goal's kind: the route re-aims `Fly` every tick.
pub const GOAL_SETTLE_S: f64 = 8.0;

/// `action` once the goal has had `since_s` to settle. Pure.
pub fn settled(action: Drift, since_s: f64) -> Drift {
    if action == Drift::Stop && since_s < GOAL_SETTLE_S {
        Drift::Fine
    } else {
        action
    }
}

/// The hover mode's target is not the one we set: put it back.
fn vector_moved(v: &[f64], want: [f64; 2]) -> bool {
    v.iter()
        .take(2)
        .zip(want)
        .any(|(x, w)| (x - w).abs() > VECTOR_SLOP_MPS)
}

async fn set_vector(xp: &XPlaneClient, v: [f64; 2]) {
    let _ = autoflight::set_dial_elem(xp, Dial::HoverVector, 0, v[0]).await;
    let _ = autoflight::set_dial_elem(xp, Dial::HoverVector, 1, v[1]).await;
}

/// Hand the aircraft back STEADY. In the air the collective goes to the
/// trim the law found (the hover figure), never the command of the moment:
/// the drift hand-back left 0.66 against a 0.57 hover, and with the
/// autopilot off and nobody on the lever the S-76 climbed away at 900 fpm
/// (2026-09-22). `collective` is `None` on the ground.
///
/// MOVING, the autopilot is not turned off but put in the cruise modes, the
/// heading and the altitude of the moment: the hover mode let go at 47 kt,
/// the drift hand-back turned the autopilot off at 500 ft with nobody
/// flying, and the S-76 was rolling into a bank at 54 kt when it was caught
/// by hand with exactly this (EDDV, 2026-09-23). Heading and altitude hold on
/// this autopilot fly the cyclic like any airframe's, and the collective the
/// law leaves is a cruise power.
async fn hand_back(
    xp: &XPlaneClient,
    voice: &Option<Voice>,
    why: &str,
    collective: Option<(&str, f64)>,
    speed_mps: Option<f64>,
) {
    if let Some((dref, trim)) = collective {
        let _ = xp.set_float_index(dref, 0, trim).await;
    }
    // In the air the autopilot is NEVER simply turned off: the second time
    // it was, at 400 ft and 7 m/s, the S-76 came down at 1,600 fpm with
    // nobody flying it and spun on the ground (EDDV, 2026-09-23). Moving,
    // the cruise modes; slow, the hover mode asked for zero. The pilot's own
    // disconnect is one press away either way.
    let airborne = collective.is_some();
    let moving = airborne && speed_mps.is_some_and(|s| s > CRUISE_HANDBACK_MPS);
    let cruise = moving && cruise_modes(xp).await;
    let hovering = airborne && !cruise && hover_mode(xp).await;
    if !cruise && !hovering {
        let _ = autoflight::press(xp, Button::DisconnectFd).await;
    }
    crate::blackbox::record(
        "hover",
        serde_json::json!({ "event": "hand_back", "why": why, "collective": collective.map(|c| c.1), "speed": speed_mps, "cruise_modes": cruise, "hover_mode": hovering }),
    );
    let line = if cruise {
        format!("{why} Autopilot's holding heading and altitude. Your controls.")
    } else if hovering {
        format!("{why} Autopilot's holding the hover. Your controls.")
    } else {
        format!("{why} Your controls.")
    };
    say(voice, Priority::Warning, line);
}

/// The autopilot on and in its hover mode, asked for zero groundspeed. True
/// when the mode reads captured.
async fn hover_mode(xp: &XPlaneClient) -> bool {
    let _ = autoflight::press(xp, Button::Engage).await;
    tokio::time::sleep(Duration::from_millis(300)).await;
    if xp.get_float(HOVER_STATUS).await.is_ok_and(|v| v < 1.5) {
        let _ = autoflight::press(xp, Button::Hover).await;
        tokio::time::sleep(Duration::from_millis(500)).await;
    }
    set_vector(xp, [0.0, 0.0]).await;
    xp.get_float(HOVER_STATUS).await.is_ok_and(|v| v >= 1.5)
}

/// Above this a hand-back leaves the autopilot in its cruise modes; below
/// it, the hover mode. Altitude hold at 35 kt let the S-76 drift backwards
/// and sink 1,200 fpm (ETHC, 2026-09-23); the hover mode has held 40.
const CRUISE_HANDBACK_MPS: f64 = 22.0;

/// The autopilot on the heading and altitude of the moment. The mode
/// commands toggle, so each fires only when its mode reads off. True when
/// both read engaged.
pub(crate) async fn cruise_modes(xp: &XPlaneClient) -> bool {
    let (hdg, alt) = tokio::join!(
        xp.get_float(crate::datarefs::MAG_PSI),
        xp.get_float("sim/cockpit2/gauges/indicators/altitude_ft_pilot"),
    );
    let (Ok(hdg), Ok(alt)) = (hdg, alt) else {
        return false;
    };
    let _ = autoflight::set_dial(xp, Dial::HeadingBug, hdg.round()).await;
    let _ = autoflight::set_dial(xp, Dial::Altitude, (alt / 10.0).round() * 10.0).await;
    let _ = autoflight::press(xp, Button::Engage).await;
    tokio::time::sleep(Duration::from_millis(300)).await;
    if xp.get_float(HEADING_STATUS).await.is_ok_and(|v| v < 0.5) {
        let _ = autoflight::press(xp, Button::Heading).await;
    }
    if xp.get_float(ALT_STATUS).await.is_ok_and(|v| v < 0.5) {
        let _ = autoflight::press(xp, Button::AltitudeHold).await;
    }
    tokio::time::sleep(Duration::from_millis(500)).await;
    let (h, a) = tokio::join!(xp.get_float(HEADING_STATUS), xp.get_float(ALT_STATUS));
    h.is_ok_and(|v| v > 0.5) && a.is_ok_and(|v| v > 0.5)
}

/// The air and the weight, read together; `None` unless all three read and
/// could be an aircraft (a failed read is no measurement).
async fn read_air(xp: &XPlaneClient) -> Option<Air> {
    let (pa, oat, w) = tokio::join!(
        xp.get_float(PRESSURE_ALT),
        xp.get_float(crate::datarefs::OAT_DEG_C),
        xp.get_float(crate::datarefs::TOTAL_WEIGHT_KG),
    );
    Some(Air {
        pressure_alt_ft: pa.ok()?,
        oat_c: oat.ok()?,
        weight_kg: w.ok()?,
    })
    .filter(Air::valid)
}

async fn fly(
    xp: XPlaneClient,
    voice: Option<Voice>,
    collective: String,
    run: &sim::run::Lease<'_>,
) {
    // The power budget first, before the autopilot takes anything: a hover
    // the lever cannot give is refused on the ground with its numbers.
    let mut air = read_air(&xp).await;
    let mut air_at = Instant::now();
    let mut loading = LoadingEstimate::default();
    loading.predict(air);
    let budget0 = PowerBudget::at(loading.now());
    let mut rec = budget0.record(air, "power_budget");
    rec["known"] = serde_json::json!(loading.predicted().is_some());
    crate::blackbox::record("hover", rec);
    let on_ground0 = xp.get_float(ON_GROUND_ANY).await.is_ok_and(|g| g > 0.5);
    if loading.predicted().is_some() && budget0.verdict() == PowerVerdict::NoHover {
        if on_ground0 {
            crate::blackbox::record(
                "hover",
                serde_json::json!({ "event": "refused", "why": "power", "need_ige": budget0.ige, "available": budget0.available }),
            );
            if let Some(line) = budget0.line(air) {
                say(&voice, Priority::Caution, line);
            }
            return;
        }
        // Already flying: nothing to refuse, but the pilot hears the numbers.
        if let Some(line) = budget0.line(air) {
            say(&voice, Priority::Warning, line);
        }
    }
    let drift0 = DriftTrim::default();
    // The autopilot holds the position; its hover mode captured on the ground
    // in the live test, so it is engaged before anything lifts. It captures
    // the groundspeed of the moment, so the target is zeroed AFTER it.
    let _ = autoflight::press(&xp, Button::Engage).await;
    tokio::time::sleep(Duration::from_millis(500)).await;
    if xp.get_float(HOVER_STATUS).await.is_ok_and(|v| v < 1.5) {
        let _ = autoflight::press(&xp, Button::Hover).await;
        tokio::time::sleep(Duration::from_millis(500)).await;
    }
    // A takeover in flight: the mode captured the velocity it was engaged
    // at, and the loop ramps from there, never from a zero that would ask
    // the cyclic for a stop in one step.
    let (gnd0, vx0, vz0) = tokio::join!(
        xp.get_float(ON_GROUND_ANY),
        xp.get_float(VX),
        xp.get_float(VZ)
    );
    let v0 = match (gnd0, vx0, vz0) {
        (Ok(g), Ok(x), Ok(z)) if g < 0.5 => [x, z],
        _ => drift0.target(),
    };
    set_vector(&xp, v0).await;
    let (ap, hs) = tokio::join!(xp.get_float(AP_ON), xp.get_float(HOVER_STATUS));
    let (ap, hs) = (ap.ok(), hs.ok());
    if !(ap.is_some_and(|v| v > 0.5) && hs.is_some_and(|v| v >= 1.5)) {
        crate::blackbox::record(
            "hover",
            serde_json::json!({ "event": "refused", "ap": ap, "hover_status": hs }),
        );
        let _ = autoflight::press(&xp, Button::DisconnectFd).await;
        say(
            &voice,
            Priority::Caution,
            "The autopilot won't take the hover. Check the stick trim and avionics are on.",
        );
        return;
    }
    let start = xp
        .get_float_array(&collective)
        .await
        .ok()
        .and_then(|v| v.first().copied())
        .unwrap_or(0.0);
    let mut law = CollectiveLaw::from(start);
    // Taken over in the air: the trim starts at the power curve's level
    // figure for the speed of the moment, not at a lever the pilot (or a
    // deceleration) left somewhere else; the command slews there at the
    // law's own rate.
    if v0 != drift0.target() {
        law.airborne = true;
        law.trim = level_collective_at(v0[0].hypot(v0[1]), loading.now());
    } else if loading.predicted().is_some() {
        // On the wheels with the air and the weight known: the lever goes
        // to just under the predicted lift-off at once. Unknown, the ramp
        // finds it from where the lever is, as it always did.
        law.set_prelift(Some(budget0.prelift()));
    }
    let mut drift = DriftTrim::default();
    let mut ramp = VelocityRamp::default();
    ramp.set(v0);
    // A new hover starts over the spot, whatever the last one was doing.
    set_motion(Motion::default());
    crate::blackbox::record(
        "hover",
        serde_json::json!({ "event": "engaged", "goal": format!("{:?}", goal()), "collective": start }),
    );

    let mut last = Instant::now();
    let mut announced_hover = false;
    let mut announced_depart = false;
    // The track a departure is flying this moment (it turns, never jumps).
    let mut track_now: Option<f64> = None;
    let mut speed_now: Option<f64> = None;
    let mut pedals = HeadingHold::new();
    let mut pedals_on = false;
    let mut pedals_yielded = false;
    let mut grounded_since: Option<Instant> = None;
    let mut touchdown_braked = false;
    let mut tick_n: u64 = 0;
    let mut last_recapture: Option<Instant> = None;
    let rotor_cfg = rotor();
    let mut droop_since: Option<Instant> = None;
    let mut droop_said = false;
    let mut nr_min: f64 = 1.0;
    let mut goal_kind: Option<std::mem::Discriminant<Goal>> = None;
    let mut goal_since = Instant::now();
    let mut capped_said = false;
    let mut feed = PowerFeed::new(loading, &law);
    // Only a hover this loop lifted from the ground is held in the cushion: a
    // takeover in the air is already holding its height, and the droop guard
    // judges what it really has.
    let lifted_here = !law.airborne();
    while run.live() {
        tokio::time::sleep(TICK).await;
        if crate::xplane::sim_frozen(&xp).await {
            last = Instant::now();
            continue;
        }
        let Some(mut g) = goal() else { break };
        if air_at.elapsed().as_secs_f64() > AIR_REFRESH_S {
            air_at = Instant::now();
            if let Some(a) = read_air(&xp).await {
                air = Some(a);
                feed.loading.predict(air);
            }
        }
        // A hover out of the ground cushion that the lever cannot hold is
        // held in it, and said once with the numbers.
        let budget = PowerBudget::at(feed.loading.now());
        if let Goal::Hover { height_ft } = g {
            if lifted_here
                && height_ft > GROUND_CUSHION_FT
                && budget.verdict() == PowerVerdict::NoOge
            {
                g = Goal::Hover {
                    height_ft: GROUND_CUSHION_FT,
                };
                retarget(g);
                let mut rec = budget.record(air, "hover_capped");
                rec["asked_ft"] = serde_json::json!(height_ft);
                crate::blackbox::record("hover", rec);
                if !capped_said {
                    capped_said = true;
                    if let Some(line) = budget.line(air) {
                        say(&voice, Priority::Caution, line);
                    }
                }
            }
        }
        let nr = match &rotor_cfg {
            Some((dref, governed)) if *governed > 0.0 => xp
                .get_float_array(dref)
                .await
                .ok()
                .and_then(|v| v.first().copied())
                .map(|r| r / governed),
            _ => None,
        };
        let (vx, vz, psi, yaw_rate, mag_psi, yielded) = tokio::join!(
            xp.get_float(VX),
            xp.get_float(VZ),
            xp.get_float(PSI),
            xp.get_float(YAW_RATE),
            xp.get_float(crate::datarefs::MAG_PSI),
            xp.get_float(PEDAL_YIELD),
        );
        // A heading asked during a departure: its true track, through the
        // variation of the moment (true minus magnetic heading).
        let mut g = g;
        if let (Ok(t), Ok(m)) = (&psi, &mag_psi) {
            let pending = TURN_MAG.lock().unwrap_or_else(|e| e.into_inner()).take();
            if let (
                Some(h),
                Goal::Depart {
                    height_ft,
                    speed_mps,
                    ..
                },
            ) = (pending, g)
            {
                g = Goal::Depart {
                    height_ft,
                    speed_mps,
                    track_deg: (h + crate::angles::signed_diff_deg(*t, *m)).rem_euclid(360.0),
                };
                retarget(g);
                crate::blackbox::record(
                    "hover",
                    serde_json::json!({ "event": "turn", "heading_mag": h, "track_true": (h + crate::angles::signed_diff_deg(*t, *m)).rem_euclid(360.0) }),
                );
            }
        }
        let (agl, vs, gnd, gs, th, ph, ap, hs, hv) = tokio::join!(
            xp.get_float(Y_AGL),
            xp.get_float(VS_FPM),
            xp.get_float(ON_GROUND_ANY),
            xp.get_float(GROUNDSPEED),
            xp.get_float(THETA),
            xp.get_float(PHI),
            xp.get_float(AP_ON),
            xp.get_float(HOVER_STATUS),
            autoflight::read_dial_array(&xp, Dial::HoverVector),
        );
        let (gs, th, ph, ap, hs, hv) = (gs.ok(), th.ok(), ph.ok(), ap.ok(), hs.ok(), hv.ok());
        // A failed read is no measurement: skip the tick, never guess.
        let (Ok(agl), Ok(vs), Ok(gnd)) = (agl, vs, gnd) else {
            continue;
        };
        let now = Instant::now();
        let dt = (now - last).as_secs_f64();
        last = now;
        let h_ft = (agl * M_TO_FT - REST_AGL_FT).max(0.0);
        let on_ground = gnd > 0.5;

        let steady = (!on_ground).then_some((collective.as_str(), law.steady()));
        // The horizontal speed (the `groundspeed` dataref carries the climb).
        let speed_h = match (&vx, &vz) {
            (Ok(x), Ok(z)) => Some(x.hypot(*z)),
            _ => None,
        };
        if ap.is_some_and(|v| v < 0.5) {
            hand_back(
                &xp,
                &voice,
                "The autopilot dropped out of the hover.",
                steady,
                speed_h,
            )
            .await;
            return;
        }
        if th.is_some_and(|t| t.abs() > MAX_ATTITUDE_DEG)
            || ph.is_some_and(|p| p.abs() > MAX_ATTITUDE_DEG)
        {
            hand_back(&xp, &voice, "The hover's gone unstable.", steady, speed_h).await;
            return;
        }
        // The hover mode let go (something replaced it): engage it again, at
        // most every few seconds. Only when it reads NOT captured - the
        // command may toggle a captured mode off.
        if hs.is_some_and(|v| v < 1.5)
            && last_recapture.is_none_or(|t: Instant| t.elapsed().as_secs_f64() > RECAPTURE_EVERY_S)
        {
            let _ = autoflight::press(&xp, Button::Hover).await;
            let (c, t) = (ramp.now(), drift.target());
            set_vector(&xp, [c[0] + t[0], c[1] + t[1]]).await;
            last_recapture = Some(Instant::now());
            crate::blackbox::record(
                "hover",
                serde_json::json!({ "event": "recapture", "gs": gs, "hover_status": hs }),
            );
        }
        // The commanded groundspeed: the asked motion, ramped; zero on the
        // ground and for a landing (a set-down is over the spot).
        let asked = match g {
            Goal::Hover { .. } if !on_ground => track_vector(motion()),
            _ if g.vector().is_some() && !on_ground => {
                let (speed_mps, track_deg) = g.vector().unwrap_or_default();
                // Speed and direction slew apart: the speed at the ramp's
                // acceleration, the track at a turn rate. The start is where
                // the command already is (a hover's zero, a turn's track).
                let v0 = ramp.now();
                let s0 = *speed_now.get_or_insert(v0[0].hypot(v0[1]));
                // A turn starts from where the command already points (a
                // takeover's captured velocity), never from the goal's track:
                // the arrival took over flying away from the pad, the command
                // pointed at it, and the error handed back in 1.4 s (ETHC,
                // 2026-09-23).
                let t0 = *track_now.get_or_insert_with(|| {
                    if v0[0].hypot(v0[1]) > 1.0 {
                        v0[0].atan2(-v0[1]).to_degrees().rem_euclid(360.0)
                    } else {
                        track_deg
                    }
                });
                let s1 = slew_speed(s0, speed_mps, dt);
                let t1 = slew_track(t0, track_deg, s1, dt);
                speed_now = Some(s1);
                track_now = Some(t1);
                let v = track_vector(Motion {
                    speed_mps: s1,
                    track_deg: t1,
                });
                ramp.set(v);
                v
            }
            _ => [0.0, 0.0],
        };
        let cmd_v = if g.vector().is_some() && !on_ground {
            asked
        } else {
            // Out of a departure the next goal ramps from its command.
            speed_now = None;
            track_now = None;
            ramp.step(asked, dt)
        };
        // The pedals: ours in forward flight, the aircraft's in the hover,
        // the pilot's the moment the pilot's feet move.
        let cmd_speed = cmd_v[0].hypot(cmd_v[1]);
        if yielded.as_ref().is_ok_and(|v| *v > 0.5) && pedals_on && !pedals_yielded {
            pedals_yielded = true;
            crate::blackbox::record("hover", serde_json::json!({ "event": "pedal_yield" }));
            say(&voice, Priority::Readback, "Your pedals.");
        }
        // A pedal law that yaws the aircraft this fast is not flying it:
        // the pedals go back to the aircraft for the rest of the flight.
        if pedals_on
            && !pedals_yielded
            && yaw_rate.as_ref().is_ok_and(|r| r.abs() > PEDAL_FAULT_DPS)
        {
            pedals_yielded = true;
            crate::blackbox::record(
                "hover",
                serde_json::json!({ "event": "pedal_fault", "yaw_rate": yaw_rate.as_ref().ok() }),
            );
            say(
                &voice,
                Priority::Caution,
                "Yawing hard. Pedals back to the aircraft.",
            );
        }
        let was_on = pedals_on;
        pedals_on = !on_ground
            && !pedals_yielded
            && if pedals_on {
                cmd_speed > PEDALS_OFF_MPS
            } else {
                cmd_speed > PEDALS_ON_MPS
            };
        if pedals_on != was_on {
            crate::blackbox::record(
                "hover",
                serde_json::json!({ "event": if pedals_on { "pedals_on" } else { "pedals_off" }, "speed": cmd_speed }),
            );
        }
        let pedal = match (&psi, &yaw_rate) {
            (Ok(h), Ok(_)) if pedals_on => {
                let want = cmd_v[0].atan2(-cmd_v[1]).to_degrees().rem_euclid(360.0);
                let p = pedals.step(want, *h, dt);
                let _ = xp.set_float(PEDALS, p).await;
                Some(p)
            }
            _ => None,
        };
        // The trim works on the ERROR from the command (a read that failed
        // is no measurement, and the trim holds still on it), and the mode's
        // target is the command plus the trim.
        let err = match (&vx, &vz) {
            (Ok(vx), Ok(vz)) => {
                drift.step(vx - cmd_v[0], vz - cmd_v[1], on_ground, dt);
                Some((vx - cmd_v[0]).hypot(vz - cmd_v[1]))
            }
            _ => None,
        };
        let target = [cmd_v[0] + drift.target()[0], cmd_v[1] + drift.target()[1]];
        if hs.is_some_and(|v| v >= 1.5) && hv.as_deref().is_some_and(|v| vector_moved(v, target)) {
            set_vector(&xp, target).await;
        }
        let kind = std::mem::discriminant(&g);
        if goal_kind != Some(kind) {
            goal_kind = Some(kind);
            goal_since = Instant::now();
        }
        let action = err.or(gs).map_or(Drift::Fine, |e| drift_action(g, e));
        match settled(action, goal_since.elapsed().as_secs_f64()) {
            Drift::Fine => {}
            Drift::Stop => {
                let hold = h_ft.round().max(10.0);
                *lock_goal() = Some(Goal::Hover { height_ft: hold });
                set_motion(Motion::default());
                crate::blackbox::record(
                    "hover",
                    serde_json::json!({ "event": "depart_stopped", "err_v": err, "h_ft": h_ft }),
                );
                say(
                    &voice,
                    Priority::Warning,
                    format!("Not holding the speed. Stopping to a hover at {hold:.0} feet."),
                );
                continue;
            }
            Drift::SetDown => {
                *lock_goal() = Some(Goal::Land);
                set_motion(Motion::default());
                crate::blackbox::record(
                    "hover",
                    serde_json::json!({ "event": "drift_set_down", "gs": gs, "h_ft": h_ft }),
                );
                say(
                    &voice,
                    Priority::Warning,
                    "We're drifting. Setting it down.",
                );
                continue;
            }
            Drift::HandBack => {
                hand_back(
                    &xp,
                    &voice,
                    "The hover mode let go of the speed.",
                    steady,
                    speed_h,
                )
                .await;
                return;
            }
        }

        let tvs = target_vs_fpm(g, h_ft, cmd_v[0].hypot(cmd_v[1]));
        // The power curve and the ground cushion at this hover's loading
        // (the curve's own when the air and the weight are unknown).
        let load_now = feed.loading.now();
        let level = feed.before(&mut law, cmd_v[0].hypot(cmd_v[1]), speed_h, h_ft, on_ground);
        // The rotor guard: the lever held on a drooping rotor from the first
        // tick, said once it lasts, and a lift the power cannot hold is set
        // down, or never leaves the ground.
        let lifting = on_ground && !law.airborne() && tvs > 0.0;
        match nr {
            Some(n) if n < DROOP_HOLD => {
                law.set_ceiling(Some(droop_ceiling(n, law.cmd(), dt)));
                nr_min = nr_min.min(n);
                let since = *droop_since.get_or_insert_with(|| {
                    crate::blackbox::record(
                        "hover",
                        serde_json::json!({
                            "event": "rotor_droop", "nr": n, "collective": law.cmd(),
                            "loading": load_now, "need_ige": budget.ige, "need_oge": budget.oge,
                            "da_ft": air.map(|a| a.density_alt_ft()), "weight_kg": air.map(|a| a.weight_kg),
                            "ground": on_ground, "h_ft": h_ft,
                        }),
                    );
                    now
                });
                let call = droop_call(
                    n,
                    (now - since).as_secs_f64(),
                    on_ground,
                    lifting,
                    h_ft,
                    matches!(g, Goal::Land),
                );
                if call.speak && !droop_said {
                    droop_said = true;
                    say(
                        &voice,
                        Priority::Warning,
                        format!(
                            "Rotor's drooping, {:.0} percent. Easing the collective.",
                            n * 100.0
                        ),
                    );
                }
                if call.set_down || call.abandon_lift {
                    *lock_goal() = Some(Goal::Land);
                    set_motion(Motion::default());
                    let mut rec = budget.record(
                        air,
                        if call.abandon_lift {
                            "lift_abandoned"
                        } else {
                            "droop_set_down"
                        },
                    );
                    rec["nr"] = serde_json::json!(n);
                    rec["nr_min"] = serde_json::json!(nr_min);
                    rec["collective"] = serde_json::json!(law.cmd());
                    rec["h_ft"] = serde_json::json!(h_ft);
                    crate::blackbox::record("hover", rec);
                    say(
                        &voice,
                        Priority::Warning,
                        power_short_line(call.abandon_lift, nr_min, law.cmd(), air),
                    );
                }
            }
            _ => {
                law.set_ceiling(None);
                if let Some(since) = droop_since.take() {
                    crate::blackbox::record(
                        "hover",
                        serde_json::json!({ "event": "droop_cleared", "secs": (now - since).as_secs_f64(), "nr_min": nr_min, "said": droop_said }),
                    );
                }
                droop_said = false;
                nr_min = 1.0;
            }
        }
        // The lever the rotor stood on before this tick: at a lift-off it is
        // the lift-off's (the step that notices it adds the height loop's).
        let lever_before = law.cmd();
        let cmd = law.step(tvs, vs, on_ground, dt);
        let _ = xp.set_float_index(&collective, 0, cmd).await;
        // The lift-off is a MEASUREMENT of the loading: the lever the rotor
        // lifted at against the reference's 0.555. It corrects the prediction
        // (and the reference weight behind it) for the rest of this hover.
        if let Some(lift) = feed.after(&law, lever_before, lifting) {
            let mut rec = PowerBudget::at(feed.loading.now()).record(air, "liftoff");
            rec["collective"] = serde_json::json!(lift.lever);
            rec["predicted_loading"] = serde_json::json!(lift.predicted);
            rec["predicted_liftoff"] =
                serde_json::json!(lift.predicted.map(|l| PowerBudget::at(l).liftoff));
            rec["measured_loading"] = serde_json::json!(lift.measured);
            rec["prelift"] = serde_json::json!(lifted_here.then(|| budget0.prelift()));
            crate::blackbox::record("hover", rec);
        }

        tick_n += 1;
        if tick_n % 2 == 0 {
            crate::blackbox::record(
                "hover",
                serde_json::json!({
                    "event": "tick", "goal": format!("{g:?}"), "h_ft": h_ft, "vs": vs,
                    "target_vs": tvs, "collective": cmd, "trim": law.trim(),
                    "ground": on_ground, "gs": gs, "theta": th, "phi": ph, "dt": dt,
                    "ap": ap, "hover_status": hs, "vector": hv,
                    "vx": vx.as_ref().ok(), "vz": vz.as_ref().ok(), "drift_trim": drift.target(),
                    "psi": psi.as_ref().ok(), "yaw_rate": yaw_rate.as_ref().ok(),
                    "cmd_v": cmd_v, "err_v": err, "track_now": track_now, "nr": nr,
                    "pedal": pedal, "pedal_trim": pedals.trim(),
                    "level": level, "loading": load_now,
                }),
            );
        }

        match g {
            Goal::Hover { height_ft } => {
                if !announced_hover
                    && law.airborne()
                    && (h_ft - height_ft).abs() < 1.5
                    && vs.abs() < 100.0
                {
                    announced_hover = true;
                    say(
                        &voice,
                        Priority::Readback,
                        format!("Hovering at {height_ft:.0} feet."),
                    );
                }
            }
            Goal::Depart {
                height_ft,
                speed_mps,
                ..
            } => {
                if !announced_depart
                    && (h_ft - height_ft).abs() < 20.0
                    && gs.is_some_and(|s| (s - speed_mps).abs() < 1.5)
                {
                    announced_depart = true;
                    crate::blackbox::record(
                        "hover",
                        serde_json::json!({ "event": "departed", "h_ft": h_ft, "gs": gs }),
                    );
                    say(
                        &voice,
                        Priority::Readback,
                        format!(
                            "Level at {height_ft:.0} feet, {:.0} knots.",
                            speed_mps * 1.94384
                        ),
                    );
                }
            }
            Goal::Fly { .. } => {}
            Goal::Land => {
                if on_ground && !law.airborne() {
                    // Down and nearly stopped: the wheels are held from
                    // here, not rolled on while the lever comes down. Never
                    // at speed: a drift set-down on the LILO-LILB ridge
                    // touched at 19 kt and the first cut of this set the
                    // parking brake there (2026-09-24).
                    if !touchdown_braked && gs.is_some_and(touchdown_brake_due) {
                        touchdown_braked = true;
                        let valve = crate::aircraft::live_profile().and_then(|p| p.park_brake_valve_set);
                        let set = crate::tools::controls::drive_park_brake(&xp, true, valve).await;
                        crate::blackbox::record(
                            "hover",
                            serde_json::json!({ "event": "touchdown_brake", "set": set.as_ref().ok(), "gs": gs }),
                        );
                    }
                    let since = *grounded_since.get_or_insert(now);
                    if (now - since).as_secs_f64() >= LANDED_SECS && cmd <= 0.001 {
                        let _ = autoflight::press(&xp, Button::DisconnectFd).await;
                        crate::blackbox::record("hover", serde_json::json!({ "event": "landed" }));
                        say(&voice, Priority::Readback, "We're down. Collective's down.");
                        return;
                    }
                } else {
                    grounded_since = None;
                }
            }
        }
    }
    // Released to the cruise modes by the route: the autopilot and the
    // collective are the route's now, nothing here touches them.
    if RELEASED.swap(false, Ordering::SeqCst) {
        crate::blackbox::record("hover", serde_json::json!({ "event": "released" }));
        return;
    }
    // Stood down from outside (a cancel, "my controls", a swap): the
    // collective stays where it is and the autopilot comes off. The caller
    // said the hand-back; saying it again here doubles the line.
    let _ = autoflight::press(&xp, Button::DisconnectFd).await;
    crate::blackbox::record("hover", serde_json::json!({ "event": "stood_down" }));
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A helicopter's vertical axis: collective above the hover figure is a
    /// climb acceleration, through a rotor lag and a tick of transport delay,
    /// with the ground underneath. `gain` is fpm per second per unit of
    /// collective, and nobody knows the S-76's yet: the tests fly a range.
    /// Collective saved in the ground cushion, and the height it fades by.
    const GROUND_EFFECT: f64 = 0.035;
    const GROUND_EFFECT_FT: f64 = 12.0;

    struct Plant {
        h_ft: f64,
        vs: f64,
        accel: f64,
        hover_c: f64,
        gain: f64,
        /// `Some(tau)`: a DAMPED helicopter, whose climb rate settles at
        /// `gain` fpm per unit of collective above the hover over `tau`
        /// seconds - the S-76 measured live (+0.017 held ~65 fpm). `None`:
        /// collective is pure acceleration.
        damping_s: Option<f64>,
        queue: std::collections::VecDeque<f64>,
    }

    impl Plant {
        fn new(hover_c: f64, gain: f64, damping_s: Option<f64>) -> Self {
            Self {
                h_ft: 0.0,
                vs: 0.0,
                accel: 0.0,
                hover_c,
                gain,
                damping_s,
                queue: std::collections::VecDeque::from(vec![0.0; 2]),
            }
        }
        fn on_ground(&self) -> bool {
            self.h_ft <= 0.0 && self.vs <= 0.0
        }
        fn step(&mut self, c: f64, dt: f64) {
            self.queue.push_back(c);
            let c = self.queue.pop_front().unwrap();
            // GROUND EFFECT: the hover needs less collective near the
            // ground, and the requirement climbs as the rotor leaves it -
            // the S-76 lifted at 0.555 and needed 0.582 at nine feet, and
            // the trim chasing that rise was the 45 s creep (2026-09-22).
            let hover_c =
                self.hover_c - GROUND_EFFECT * (1.0 - self.h_ft / GROUND_EFFECT_FT).max(0.0);
            let want = match self.damping_s {
                Some(tau) => (self.gain * (c - hover_c) - self.vs) / tau,
                None => self.gain * (c - hover_c),
            };
            self.accel += (want - self.accel) * (dt / 0.4).min(1.0);
            self.vs += self.accel * dt;
            self.h_ft += self.vs / 60.0 * dt;
            if self.h_ft <= 0.0 {
                self.h_ft = 0.0;
                if self.vs < 0.0 {
                    self.vs = 0.0;
                }
                if self.accel < 0.0 {
                    self.accel = 0.0;
                }
            }
        }
    }

    /// (max height error after settling, worst touchdown fpm, final collective)
    /// (height error after settling, touchdown fpm, final collective, peak
    /// height, seconds to reach nine feet)
    fn sortie(hover_c: f64, gain: f64, damping_s: Option<f64>) -> (f64, f64, f64, f64, f64) {
        let mut p = Plant::new(hover_c, gain, damping_s);
        let mut reached = f64::INFINITY;
        let mut lifted: Option<f64> = None;
        let mut law = CollectiveLaw::from(0.0);
        let dt = 0.1;
        let mut c = 0.0;
        let mut worst_err: f64 = 0.0;
        let mut peak: f64 = 0.0;
        // Lift to ten feet and hold for a minute.
        for i in 0..900 {
            let g = Goal::Hover { height_ft: 10.0 };
            c = law.step(target_vs_fpm(g, p.h_ft, 0.0), p.vs, p.on_ground(), dt);
            p.step(c, dt);
            peak = peak.max(p.h_ft);
            if p.h_ft > 0.0 && lifted.is_none() {
                lifted = Some(i as f64 * dt);
            }
            if p.h_ft >= 9.0 && reached.is_infinite() {
                reached = i as f64 * dt - lifted.unwrap_or(0.0);
            }
            if i > 600 {
                worst_err = worst_err.max((p.h_ft - 10.0).abs());
            }
        }
        // Set it down.
        let mut touchdown: f64 = 0.0;
        let mut was_air = true;
        for _ in 0..900 {
            let before = p.vs;
            c = law.step(
                target_vs_fpm(Goal::Land, p.h_ft, 0.0),
                p.vs,
                p.on_ground(),
                dt,
            );
            p.step(c, dt);
            if was_air && p.on_ground() {
                touchdown = touchdown.min(before);
                was_air = false;
            }
        }
        (worst_err, touchdown, c, peak, reached)
    }

    #[test]
    fn it_lifts_holds_ten_feet_and_sets_down_across_unknown_helicopters() {
        let mut bad = Vec::new();
        // Pure acceleration over a range of gains, and a DAMPED rotor around
        // the S-76's live figure (~3,800 fpm per unit collective): the first
        // live lift took 45 s to creep from 3 to 9 ft behind its target,
        // which a pure-acceleration plant cannot show.
        let plants = [
            (1500.0, None),
            (3000.0, None),
            (6000.0, None),
            (2000.0, Some(2.5)),
            (3800.0, Some(2.5)),
            (8000.0, Some(2.5)),
        ];
        for hover_c in [0.35, 0.5, 0.65] {
            for (gain, damping) in plants {
                let (err, td, end_c, peak, reached) = sortie(hover_c, gain, damping);
                let line = format!(
                    "hover {hover_c} gain {gain} damping {damping:?}: err {err:.1} ft, \
                     peak {peak:.1} ft, 9 ft {reached:.0} s after lift-off, touchdown {td:.0} fpm, \
                     collective {end_c:.2}"
                );
                if err >= 1.5 || peak >= 16.0 || td <= -250.0 || end_c >= 0.01 || reached > 20.0 {
                    bad.push(line);
                }
            }
        }
        assert!(bad.is_empty(), "{bad:#?}");
    }

    /// A mode selected mid-hover is refused, never flown: it replaced the
    /// hover mode once. Nothing is refused when no hover runs.
    #[test]
    fn the_hover_keeps_the_autopilot_modes() {
        assert!(blocks("set_heading").is_none(), "no hover running");
        let run = RUN.take_over();
        let heading = blocks("set_heading");
        let lights = blocks("set_lights");
        drop(run);
        assert!(heading.is_some_and(|l| l.contains("set it down")));
        assert!(lights.is_none(), "a switch is not a mode");
    }

    /// A drift low down is answered with the ground, not the pilot; a
    /// set-down that keeps drifting is flown on, and only a runaway is
    /// handed back.
    #[test]
    fn a_drift_sets_it_down_before_it_hands_back() {
        let hover = Goal::Hover { height_ft: 20.0 };
        assert_eq!(drift_action(hover, 2.0), Drift::Fine);
        assert_eq!(drift_action(hover, 6.8), Drift::Fine);
        assert_eq!(drift_action(hover, 9.0), Drift::SetDown);
        assert_eq!(drift_action(Goal::Land, 9.0), Drift::Fine);
        assert_eq!(drift_action(Goal::Land, 17.0), Drift::HandBack);
        // The vector the S-76 was left holding (EDDV 2026-09-23) is moved.
        assert!(vector_moved(&[-4.91, -4.74], [0.0, 0.0]));
        assert!(!vector_moved(&[0.0, 0.01], [0.0, 0.0]));
        assert!(!vector_moved(&[-1.0, 0.5], [-1.0, 0.5]));
    }

    /// The hover mode as a proportional groundspeed hold with a standing
    /// error (the forward run at EDDV): the trim takes it out without
    /// ringing, across inner-loop gains, and never asks past its cap.
    #[test]
    fn the_drift_trim_takes_out_the_modes_standing_error() {
        let mut bad = Vec::new();
        for k in [0.2, 0.5, 1.5] {
            for (dx, dz) in [(0.1, 0.0), (0.0, -0.2), (0.15, 0.15)] {
                let mut trim = DriftTrim::default();
                let (mut vx, mut vz) = (0.0_f64, 0.0_f64);
                let dt = 0.1;
                let mut peak: f64 = 0.0;
                let mut end: f64 = 0.0;
                for i in 0..3000 {
                    let [tx, tz] = trim.step(vx, vz, false, dt);
                    assert!(tx.hypot(tz) <= DRIFT_TRIM_MAX_MPS + 1e-9);
                    vx += (k * (tx - vx) + dx) * dt;
                    vz += (k * (tz - vz) + dz) * dt;
                    peak = peak.max(vx.hypot(vz));
                    if i > 2400 {
                        end = end.max(vx.hypot(vz));
                    }
                }
                let untrimmed = (dx as f64).hypot(dz) / k;
                if end > 0.1 || peak > untrimmed * 1.5 + 0.05 {
                    bad.push(format!(
                        "k {k} d ({dx},{dz}): end {end:.2} m/s, peak {peak:.2}, untrimmed {untrimmed:.2}"
                    ));
                }
            }
        }
        assert!(bad.is_empty(), "{bad:#?}");
        let mut t = DriftTrim { x: 1.0, z: 1.0 };
        assert_eq!(
            t.step(1.0, 1.0, true, 0.1),
            [0.0, 0.0],
            "the ground resets it"
        );
    }

    /// An air taxi ramps its speed in and out, a landing stops first, and
    /// the drift limits judge the error from the command, not the speed.
    #[test]
    fn an_air_taxi_ramps_and_is_judged_on_its_error() {
        // South is +z: a track of 180 at 5 m/s.
        let v = track_vector(Motion {
            speed_mps: 5.0,
            track_deg: 180.0,
        });
        assert!(v[0].abs() < 1e-9 && (v[1] - 5.0).abs() < 1e-9);
        let v = track_vector(Motion {
            speed_mps: 2.0,
            track_deg: 90.0,
        });
        assert!((v[0] - 2.0).abs() < 1e-9 && v[1].abs() < 1e-9);
        let mut r = VelocityRamp::default();
        let mut t = 0.0;
        while r.now() != [0.0, 5.0] {
            r.step([0.0, 5.0], 0.1);
            t += 0.1;
            assert!(t < 60.0);
        }
        assert!((t - 5.0 / RAMP_ACCEL_MPS2).abs() < 0.3, "{t}");
        // Eight m/s AT a commanded eight is on the command.
        assert_eq!(
            drift_action(Goal::Hover { height_ft: 30.0 }, 0.3),
            Drift::Fine
        );
        // The command's vector plus the trim is what the mode is given: a
        // trimmed air taxi reads as "not moved".
        assert!(!vector_moved(&[0.1, 5.2], [0.1, 5.2]));
    }

    /// A departure accelerates level and climbs only once flying; a landing
    /// asked at speed slows before it descends; a departure that stops
    /// following is stopped, not set down.
    #[test]
    fn a_departure_climbs_only_once_flying() {
        let d = Goal::Depart {
            height_ft: 300.0,
            speed_mps: 15.0,
            track_deg: 180.0,
        };
        assert_eq!(target_vs_fpm(d, 30.0, 3.0), 0.0);
        assert_eq!(target_vs_fpm(d, 30.0, 9.0), DEPART_CLIMB_FPM);
        assert_eq!(target_vs_fpm(d, 300.0, 15.0), 0.0);
        assert_eq!(target_vs_fpm(Goal::Land, 200.0, 10.0), 0.0);
        assert!(target_vs_fpm(Goal::Land, 200.0, 1.0) < 0.0);
        assert_eq!(drift_action(d, 6.0), Drift::Stop);
        assert_eq!(drift_action(d, 3.0), Drift::Fine);
        // LIME 2026-09-25: 5.2 m/s off 3.5 s after the route's handover is
        // a capture, not a departure that stopped following.
        assert_eq!(settled(Drift::Stop, 3.5), Drift::Fine);
        assert_eq!(settled(Drift::Stop, 9.0), Drift::Stop);
        assert_eq!(settled(Drift::SetDown, 1.0), Drift::SetDown, "the hard limits hold");
        assert_eq!(settled(Drift::HandBack, 1.0), Drift::HandBack);
        // A hover taxi at 10 kt rounds a corner with more lag than that.
        let taxi = Goal::Fly {
            height_ft: 12.0,
            speed_mps: 5.0,
            track_deg: 90.0,
        };
        assert_eq!(drift_action(taxi, 6.0), Drift::Fine);
        assert_eq!(drift_action(taxi, 10.0), Drift::Stop);
    }

    /// A departure's turn: the short way, at a rate the ramp can fly, and
    /// it arrives.
    #[test]
    fn a_turn_slews_the_track_the_short_way() {
        let mut t = 350.0;
        let mut secs = 0.0;
        while crate::angles::abs_diff_deg(20.0, t) > 0.01 {
            let before = t;
            t = slew_track(t, 20.0, 15.0, 0.1);
            assert!(
                crate::angles::signed_diff_deg(t, before) > 0.0,
                "turns right"
            );
            secs += 0.1;
            assert!(secs < 60.0);
        }
        // 30 degrees at the standard rate.
        assert!((9.5..10.5).contains(&secs), "{secs}");
        assert!((slew_track(0.0, 90.0, 2.0, 0.5) - 1.5).abs() < 1e-9);
    }

    /// A rate-command yaw with a heading hold (the S-76 as measured: pedal
    /// is a yaw rate, neutral holds), a lag and a tick of transport delay,
    /// and a standing weathervane rate the pedal must cancel. The hold keeps
    /// the nose on a track turning at the standard rate, across rate gains
    /// either side of the measured 47, and brings a 60-degree offset back
    /// without going round.
    #[test]
    fn the_pedals_keep_the_nose_on_a_turning_track() {
        let mut bad = Vec::new();
        for gain in [25.0, 47.0, 90.0] {
            for vane in [-1.5, 0.0, 1.5] {
                for tau in [0.3, 0.8] {
                    let mut hold = HeadingHold::new();
                    let (mut hdg, mut r) = (240.0_f64, 0.0_f64);
                    let mut queue = std::collections::VecDeque::from(vec![0.0; 3]);
                    let mut track = 180.0;
                    let dt = 0.25;
                    let (mut worst_turn, mut worst_end, mut over): (f64, f64, f64) =
                        (0.0, 0.0, 0.0);
                    for i in 0..1200 {
                        if (400..520).contains(&i) {
                            track = (track + TURN_RATE_DPS * dt) % 360.0;
                        }
                        queue.push_back(hold.step(track, hdg, dt));
                        let p = queue.pop_front().unwrap();
                        r += (gain * p + vane - r) * (dt / tau).min(1.0);
                        hdg = (hdg + r * dt).rem_euclid(360.0);
                        let err = crate::angles::signed_diff_deg(hdg, track);
                        if i < 200 {
                            over = over.min(err);
                        }
                        if (440..520).contains(&i) {
                            worst_turn = worst_turn.max(err.abs());
                        }
                        if i > 1000 {
                            worst_end = worst_end.max(err.abs());
                        }
                    }
                    if worst_turn > 12.0 || worst_end > 1.5 || over < -15.0 {
                        bad.push(format!(
                            "gain {gain} vane {vane} tau {tau}: turn err {worst_turn:.1}, \
                             end err {worst_end:.1}, overshoot {over:.1}"
                        ));
                    }
                }
            }
        }
        assert!(bad.is_empty(), "{bad:#?}");
    }

    /// A deceleration from 35 kt to a hover while holding height, through a
    /// damped plant whose level collective follows the power curve (the knee
    /// of translational lift). With the curve's slope fed into the trim the
    /// height holds; the error alone is the takeover that sank 300 fpm.
    #[test]
    fn a_deceleration_holds_height_through_the_power_knee() {
        let fly = |ff: bool| {
            let mut law = CollectiveLaw::from(level_collective(18.0));
            law.airborne = true;
            let (mut h, mut vs, mut acc) = (400.0_f64, 0.0_f64, 0.0_f64);
            let mut speed = 18.0_f64;
            let mut prev = speed;
            let mut worst: f64 = 0.0;
            let mut queue = std::collections::VecDeque::from(vec![law.cmd; 2]);
            let dt = 0.1;
            for _ in 0..900 {
                speed = (speed - RAMP_ACCEL_MPS2 * dt).max(0.0);
                if ff {
                    law.shift_trim(level_collective(speed) - level_collective(prev));
                }
                prev = speed;
                let tvs = target_vs_fpm(Goal::Hover { height_ft: 400.0 }, h, speed);
                queue.push_back(law.step(tvs, vs, false, dt));
                let c = queue.pop_front().unwrap();
                // 3,800 fpm per unit of collective above the level figure,
                // settling over 2.5 s (the S-76's hover, measured).
                let want = (3800.0 * (c - level_collective(speed)) - vs) / 2.5;
                acc += (want - acc) * (dt / 0.4_f64).min(1.0);
                vs += acc * dt;
                h += vs / 60.0 * dt;
                worst = worst.max((h - 400.0).abs());
            }
            worst
        };
        let (with, without) = (fly(true), fly(false));
        assert!(with < 15.0, "height error {with:.1} ft with the curve");
        assert!(
            without > with * 1.5,
            "the curve earns its place: {without:.1} vs {with:.1}"
        );
    }

    /// The rotor guard: the collective may not rise on a drooping rotor, comes
    /// down on a badly drooped one, and the law's trim does not wind up
    /// against the ceiling.
    #[test]
    fn a_drooping_rotor_caps_the_collective() {
        assert_eq!(droop_ceiling(1.0, 0.6, 0.1), COLLECTIVE_MAX);
        assert_eq!(droop_ceiling(0.95, 0.6, 0.1), 0.6);
        assert!(droop_ceiling(0.83, 0.9, 0.1) < 0.9);
        let mut law = CollectiveLaw::from(0.6);
        law.airborne = true;
        law.set_ceiling(Some(0.6));
        for _ in 0..50 {
            // Asked to climb hard, sinking: the lever still stays put.
            let c = law.step(300.0, -200.0, false, 0.1);
            assert!(c <= 0.6 + 1e-9, "{c}");
        }
        assert!(law.trim() <= 0.6 + 1e-9);
    }

    /// A landing asked in a hard climb (ETHC, 2026-09-23: 1,700 fpm up at a
    /// wound-up 0.9) must not trade it for a hard sink: the trim stays in
    /// its band round the power curve and the lever never falls far under
    /// it. The same plant, the same law, with and without the band.
    #[test]
    fn a_landing_from_a_climb_never_becomes_a_plunge() {
        let fly = |band: bool| {
            let level = 0.66;
            let mut law = CollectiveLaw::from(0.9);
            law.airborne = true;
            law.trim = 0.57;
            let (mut h, mut vs, mut acc) = (600.0_f64, 1700.0_f64, 0.0_f64);
            let mut worst: f64 = 0.0;
            let mut queue = std::collections::VecDeque::from(vec![0.9; 2]);
            let dt = 0.2;
            for _ in 0..600 {
                law.set_level(band.then_some(level));
                queue.push_back(law.step(target_vs_fpm(Goal::Land, h, 0.0), vs, false, dt));
                let c = queue.pop_front().unwrap();
                let want = (3800.0 * (c - level) - vs) / 2.5;
                acc += (want - acc) * (dt / 0.4_f64).min(1.0);
                vs += acc * dt;
                h += vs / 60.0 * dt;
                if h <= 0.0 {
                    break;
                }
                worst = worst.min(vs);
                if band {
                    assert!(law.trim() >= level - TRIM_BELOW_LEVEL - 1e-9);
                }
            }
            worst
        };
        let (with, without) = (fly(true), fly(false));
        assert!(with > -800.0, "worst sink {with:.0} fpm with the band");
        assert!(
            without < with,
            "the band earns its place: {without:.0} vs {with:.0}"
        );
    }

    #[test]
    fn a_landing_comes_down_gently_and_a_hover_asks_for_its_height() {
        assert_eq!(target_vs_fpm(Goal::Land, 2.0, 0.0), -TOUCHDOWN_FPM);
        assert_eq!(target_vs_fpm(Goal::Land, 40.0, 0.0), -MAX_DESCENT_FPM);
        assert_eq!(
            target_vs_fpm(Goal::Hover { height_ft: 10.0 }, 0.0, 0.0),
            10.0 * HEIGHT_GAIN_FPM_PER_FT
        );
        assert_eq!(
            target_vs_fpm(Goal::Hover { height_ft: 10.0 }, 10.0, 0.0),
            0.0
        );
        // On the ground asked to stay down, the lever only comes down.
        let mut law = CollectiveLaw::from(0.3);
        let c = law.step(-100.0, 0.0, true, 0.1);
        assert!(c < 0.3 && c >= 0.0);
    }
    // -----------------------------------------------------------------------
    // The power budget (#84)
    // -----------------------------------------------------------------------

    /// Every lift-off the S-76 made 2026-09-22..25 (blackbox `hover` ticks:
    /// the last lever on the wheels), with the field (state `alt`, ft) and
    /// the fuel (kg): (alt, fuel, lever).
    const RECORDED_LIFTOFFS: &[(f64, f64, f64)] = &[
        (162.7, 325.3, 0.555),
        (216.9, 412.4, 0.560),
        (216.3, 412.5, 0.572),
        (210.8, 394.8, 0.565),
        (217.1, 411.5, 0.575),
        (217.0, 386.8, 0.561),
        (215.9, 350.4, 0.558),
        (237.7, 313.0, 0.561),
        (230.0, 260.7, 0.546),
        (229.9, 228.0, 0.556),
        (229.9, 186.7, 0.547),
        (232.8, 412.1, 0.578),
        (230.4, 368.3, 0.567),
        (246.0, 305.8, 0.563),
        (257.7, 411.8, 0.583),
        (271.5, 620.4, 0.617),
        (265.6, 581.2, 0.609),
        (299.5, 604.4, 0.613),
        (307.9, 518.2, 0.594),
        (320.5, 626.5, 0.642),
        (377.5, 545.6, 0.604),
        (385.8, 671.9, 0.629),
        (1297.8, 569.4, 0.644), // EDVW, the hilltop of #84
        (1371.3, 704.6, 0.665),
        (384.0, 463.3, 0.606),
        (5605.1, 543.5, 0.711), // LSZS
        (1094.6, 365.4, 0.622),
        (1096.7, 773.1, 0.676),
        (1062.5, 773.7, 0.656),
        (1252.5, 503.4, 0.656),
    ];

    /// No OAT or payload was logged: the standard day and a 170 kg crew.
    fn isa_air(alt_ft: f64, fuel_kg: f64) -> Air {
        Air {
            pressure_alt_ft: alt_ft,
            oat_c: 15.0 - 1.98 * alt_ft / 1000.0,
            weight_kg: 3435.0 + fuel_kg + 170.0,
        }
    }

    /// The prediction the lift-off starts from is the recorded lift-offs',
    /// and it errs low, never past the prelift margin: a high one would put
    /// the lever over the lift-off at the fast rate.
    #[test]
    fn the_lift_off_goes_with_weight_over_density() {
        let (mut new_err, mut old_err) = (0.0, 0.0);
        for &(alt, fuel, lever) in RECORDED_LIFTOFFS {
            let air = isa_air(alt, fuel);
            assert!(air.valid());
            let b = PowerBudget::at(air.loading());
            new_err += (b.liftoff - lever).abs();
            old_err += (LIFTOFF_REF - lever).abs();
            assert!(
                b.prelift() < lever,
                "prelift {:.3} over the lift-off {lever} at {alt} ft, {fuel} kg",
                b.prelift()
            );
            assert!(
                (b.liftoff - lever).abs() < 0.06,
                "{alt} ft {fuel} kg: {:.3} vs {lever}",
                b.liftoff
            );
        }
        let n = RECORDED_LIFTOFFS.len() as f64;
        let (new_err, old_err) = (new_err / n, old_err / n);
        assert!(new_err < 0.02, "mean error {new_err:.3}");
        assert!(
            old_err > 2.0 * new_err,
            "the budget earns its place: {old_err:.3} vs {new_err:.3}"
        );
        // LSZS, 5,605 ft: 0.711 measured; the light figure was 0.156 short.
        let lszs = PowerBudget::at(isa_air(5605.1, 543.5).loading()).liftoff;
        assert!((lszs - 0.711).abs() < 0.03, "LSZS {lszs:.3}");
    }

    #[test]
    fn the_air_is_the_standard_atmosphere() {
        let sl = Air {
            pressure_alt_ft: 0.0,
            oat_c: 15.0,
            weight_kg: 4000.0,
        };
        assert!((sl.sigma() - 1.0).abs() < 1e-3);
        assert!(sl.density_alt_ft().abs() < 30.0);
        let isa5 = Air {
            pressure_alt_ft: 5000.0,
            oat_c: 5.1,
            weight_kg: 4000.0,
        };
        assert!((isa5.sigma() - 0.8617).abs() < 0.002, "{}", isa5.sigma());
        assert!((isa5.density_alt_ft() - 5000.0).abs() < 60.0);
        // A hot hilltop: 1,300 ft at 30 C flies like 3,300 ft.
        let hot = Air {
            pressure_alt_ft: 1300.0,
            oat_c: 30.0,
            weight_kg: 4000.0,
        };
        let da = hot.density_alt_ft();
        assert!((3000.0..3600.0).contains(&da), "{da}");
        // Heavier, higher, hotter: each needs more.
        let base = isa_air(300.0, 500.0).loading();
        assert!(isa_air(300.0, 800.0).loading() > base);
        assert!(isa_air(3000.0, 500.0).loading() > base);
        assert!(hot.loading() > Air { oat_c: 0.0, ..hot }.loading());
        // A failed or absurd read is no measurement.
        assert!(!Air {
            weight_kg: f64::NAN,
            ..sl
        }
        .valid());
        assert!(!Air {
            weight_kg: 0.0,
            ..sl
        }
        .valid());
        assert!(!Air { oat_c: 300.0, ..sl }.valid());
        let mut est = LoadingEstimate::default();
        est.predict(Some(Air {
            weight_kg: f64::NAN,
            ..sl
        }));
        assert_eq!(est.predicted(), None);
        assert_eq!(
            est.now(),
            1.0,
            "unknown is the curve's own: the loop as it was"
        );
    }

    #[test]
    fn the_budget_refuses_and_warns_with_its_numbers() {
        let at = |pa: f64, w: f64| {
            let air = Air {
                pressure_alt_ft: pa,
                oat_c: 15.0 - 1.98 * pa / 1000.0,
                weight_kg: w,
            };
            (PowerBudget::at(air.loading()), air)
        };
        // The curve's own flights: fine.
        let (b, _) = at(300.0, 4180.0);
        assert_eq!(b.verdict(), PowerVerdict::Fine);
        assert!((b.oge - 0.66).abs() < 0.01);
        // Maximum gross at 2,500 ft: a hover taxi, no hover out of the cushion.
        let (b, air) = at(2500.0, 5307.0);
        assert_eq!(b.verdict(), PowerVerdict::NoOge, "{b:?}");
        let line = b.line(Some(air)).unwrap();
        assert!(
            line.contains("out of ground effect") && line.contains("percent collective"),
            "{line}"
        );
        assert!(
            line.contains("density altitude 2,500 feet") && line.contains("5,310 kilograms"),
            "{line}"
        );
        // Maximum gross at 8,000 ft on a hot day: nothing.
        let hot = Air {
            pressure_alt_ft: 8000.0,
            oat_c: 25.0,
            weight_kg: 5307.0,
        };
        let b = PowerBudget::at(hot.loading());
        assert_eq!(b.verdict(), PowerVerdict::NoHover, "{b:?}");
        let line = b.line(Some(hot)).unwrap();
        assert!(line.starts_with("Not enough power to hover here"), "{line}");
        assert!(
            line.contains(&format!("{:.0} is all we have", b.available * 100.0)),
            "{line}"
        );
        // Between them: tight, and said by nobody.
        let (b, _) = at(2000.0, 5000.0);
        assert_eq!(b.verdict(), PowerVerdict::Tight, "{b:?}");
        assert!(b.line(None).is_none());
    }

    #[test]
    fn a_droop_is_held_at_once_said_when_it_lasts_and_a_lift_that_droops_is_given_up() {
        // The governor catching a turn (nr 0.970, back inside 1.4 s): held,
        // not said.
        let c = droop_call(0.9696, 0.5, false, false, 9.0, false);
        assert!(!c.speak && !c.set_down && !c.abandon_lift);
        assert!(droop_call(0.9696, 2.1, false, false, 9.0, false).speak);
        assert!(
            droop_call(0.93, 0.0, false, false, 9.0, false).speak,
            "deep: at once"
        );
        // Lasting, low, in the air: set down.
        let c = droop_call(0.95, 5.5, false, false, 9.0, false);
        assert!(c.set_down && !c.abandon_lift);
        // Lasting on the wheels while lifting: the lift is given up.
        let c = droop_call(0.95, 5.5, true, true, 0.0, false);
        assert!(c.abandon_lift && !c.set_down);
        assert_eq!(
            droop_call(0.95, 5.5, true, false, 0.0, false),
            DroopCall {
                speak: true,
                ..DroopCall::default()
            }
        );
        assert!(
            !droop_call(0.95, 5.5, false, false, 9.0, true).set_down,
            "already landing"
        );
        assert_eq!(
            droop_call(0.99, 9.0, false, false, 9.0, false),
            DroopCall::default()
        );
        let hill = isa_air(1297.8, 569.4);
        let line = power_short_line(true, 0.931, 0.873, Some(hill));
        assert!(
            line.starts_with(
                "Not enough power to lift: rotor at 93 percent on 87 percent collective"
            ),
            "{line}"
        );
        assert!(
            line.contains("density altitude 1,300 feet") && line.ends_with("Collective down."),
            "{line}"
        );
        assert!(power_short_line(false, 0.93, 0.8, None).ends_with("Setting it down."));
    }

    #[test]
    fn the_loading_is_measured_at_lift_off_and_carries_the_prediction() {
        let hill = isa_air(1297.8, 569.4);
        let mut est = LoadingEstimate::default();
        est.predict(Some(hill));
        let p = est.predicted().unwrap();
        // EDVW lifted at 0.644: heavier than the fuel and the field say.
        let m = est.lifted_at(0.644).unwrap();
        assert!(m > p, "{m} vs {p}");
        assert!((PowerBudget::at(est.now()).liftoff - 0.644).abs() < 1e-9);
        // 100 kg of fuel later the measurement carries the change.
        est.predict(Some(Air {
            weight_kg: hill.weight_kg - 100.0,
            ..hill
        }));
        let ratio = (hill.weight_kg - 100.0) / hill.weight_kg;
        assert!((est.now() - m * ratio).abs() < 1e-9);
        // A lift-off with nothing predicted is the measurement alone.
        let mut bare = LoadingEstimate::default();
        let m = bare.lifted_at(0.644).unwrap();
        assert_eq!(bare.now(), m);
        // A lever that cannot be a loading is no measurement.
        assert_eq!(LoadingEstimate::default().lifted_at(0.05), None);
    }

    /// A vertical axis through the S-76's damped response (3,800 fpm per unit
    /// of collective, settling over 2.5 s) and a TRANSPORT DELAY (the lever
    /// written reaches the rotor two ticks later), on a cushion that fades
    /// EXPONENTIALLY (the budget's model is piecewise linear: the plant is
    /// not the model), sized by the hover of EDVW: 0.644 on the wheels,
    /// 0.683 at ten feet.
    struct Rotor {
        h_ft: f64,
        vs: f64,
        accel: f64,
        lift: f64,
        oge: f64,
        queue: std::collections::VecDeque<f64>,
    }

    impl Rotor {
        fn new(lift: f64, oge: f64, c0: f64) -> Self {
            Self {
                h_ft: 0.0,
                vs: 0.0,
                accel: 0.0,
                lift,
                oge,
                queue: vec![c0; 2].into(),
            }
        }
        fn need(&self) -> f64 {
            self.oge - (self.oge - self.lift) * (-self.h_ft / 13.0).exp()
        }
        fn on_ground(&self) -> bool {
            self.h_ft <= 0.0 && self.vs <= 0.0
        }
        fn step(&mut self, c: f64, dt: f64) {
            self.queue.push_back(c);
            let c = self.queue.pop_front().unwrap();
            let want = (3800.0 * (c - self.need()) - self.vs) / 2.5;
            self.accel += (want - self.accel) * (dt / 0.4).min(1.0);
            self.vs += self.accel * dt;
            self.h_ft += self.vs / 60.0 * dt;
            if self.h_ft <= 0.0 {
                self.h_ft = 0.0;
                self.vs = self.vs.max(0.0);
                self.accel = self.accel.max(0.0);
            }
        }
    }

    /// (seconds to leave the ground, seconds to 11 ft of a 12 ft hover, peak
    /// height, worst error over the last minute), from the request. `budget`
    /// false is the loop as it was: the ramp from a flat pitch and the light
    /// curve. The tick is the live loop's (~0.37 s in the EDVW records).
    fn lift_to_twelve(lift: f64, oge: f64, air: Option<Air>, budget: bool) -> (f64, f64, f64, f64) {
        let dt = 0.37;
        let mut p = Rotor::new(lift, oge, 0.0);
        let mut law = CollectiveLaw::from(0.0);
        let mut est = LoadingEstimate::default();
        if budget {
            est.predict(air);
            law.set_prelift(Some(PowerBudget::at(est.now()).prelift()));
        }
        let mut feed = PowerFeed::new(est, &law);
        let (mut t_lift, mut t_11, mut peak, mut worst) =
            (f64::INFINITY, f64::INFINITY, 0.0_f64, 0.0_f64);
        let g = Goal::Hover { height_ft: 12.0 };
        for i in 0..((120.0 / dt) as usize) {
            let t = i as f64 * dt;
            let on_ground = p.on_ground();
            let tvs = target_vs_fpm(g, p.h_ft, 0.0);
            let lifting = on_ground && !law.airborne() && tvs > 0.0;
            if budget {
                feed.before(&mut law, 0.0, Some(0.0), p.h_ft, on_ground);
            } else {
                law.set_level((!on_ground && law.airborne()).then(|| level_collective(0.0)));
            }
            let before = law.cmd();
            let c = law.step(tvs, p.vs, on_ground, dt);
            if budget {
                feed.after(&law, before, lifting);
            }
            p.step(c, dt);
            if p.h_ft > 0.0 && t_lift.is_infinite() {
                t_lift = t;
            }
            if p.h_ft >= 11.0 && t_11.is_infinite() {
                t_11 = t;
            }
            peak = peak.max(p.h_ft);
            if t > 60.0 {
                worst = worst.max((p.h_ft - 12.0).abs());
            }
        }
        (t_lift, t_11, peak, worst)
    }

    /// EDVW (#84): the heavy hilltop hover. The loop as it was spent the
    /// recorded 16 s finding the lift-off and crept up to its height; with
    /// the budget the lever starts under the predicted lift-off, the cushion
    /// is fed forward, and it neither leaps nor creeps.
    #[test]
    fn a_heavy_hilltop_hover_starts_at_its_lever_and_makes_its_height() {
        let loading = 0.644 / (LIFTOFF_REF / IGE_REF_LOADING);
        let (lift, oge) = (0.644, level_collective(0.0) * loading);
        let hill = isa_air(1297.8, 569.4);
        let (old_lift, old_11, _, _) = lift_to_twelve(lift, oge, Some(hill), false);
        let (new_lift, new_11, peak, worst) = lift_to_twelve(lift, oge, Some(hill), true);
        assert!(old_lift > 14.0, "the recorded slow ramp: {old_lift:.1} s");
        assert!(new_lift < 9.0, "lifted at {new_lift:.1} s");
        assert!(
            new_11 + 8.0 < old_11,
            "11 ft at {new_11:.1} s vs {old_11:.1} s"
        );
        assert!(peak < 14.5, "peak {peak:.1} ft");
        assert!(worst < 1.0, "held to {worst:.1} ft");
    }

    /// EGBJ, 2026-10-01: at 5,155 kg the S-76 descended to 2.8 ft, then
    /// hovered for 50 s although the landing still asked for -100 fpm.  The
    /// trim followed the ground-effect feed-forward, but its safety band was
    /// anchored to the OGE collective and would not let the lever follow it.
    /// One physical equilibrium must drive both feed-forward and the band.
    #[test]
    fn a_heavy_landing_follows_the_ground_cushion_to_contact() {
        let dt = 0.37;
        let loading = 1.205;
        let lift = LIFTOFF_REF / IGE_REF_LOADING * loading;
        let oge = level_collective(0.0) * loading;
        let mut p = Rotor::new(lift, oge, oge);
        p.h_ft = GE_FADE_FT;
        let mut law = CollectiveLaw::from(oge);
        law.airborne = true;
        law.trim = oge;
        let estimate = LoadingEstimate {
            predicted: Some(loading),
            lifted: None,
        };
        let mut feed = PowerFeed::new(estimate, &law);
        let mut touchdown = None;
        for i in 0..((90.0 / dt) as usize) {
            let on_ground = p.on_ground();
            feed.before(&mut law, 0.0, Some(0.0), p.h_ft, on_ground);
            let before = p.vs;
            let c = law.step(target_vs_fpm(Goal::Land, p.h_ft, 0.0), p.vs, on_ground, dt);
            p.step(c, dt);
            if p.on_ground() {
                touchdown = Some((i as f64 * dt, before));
                break;
            }
        }
        let (seconds, sink_fpm) = touchdown.expect("the landing reached the ground");
        assert!(seconds < 60.0, "touchdown took {seconds:.1} s");
        assert!(
            sink_fpm > -300.0,
            "touchdown remained controlled at {sink_fpm:.0} fpm"
        );
    }

    /// Across loadings and predictions that are wrong both ways (a payload
    /// nobody loaded, a hot day read as standard): never a leap off the
    /// ground, always its height, never slower than the loop was.
    #[test]
    fn a_wrong_prediction_neither_leaps_nor_creeps() {
        let mut bad = Vec::new();
        for true_loading in [0.95, 1.1, 1.3] {
            let lift = LIFTOFF_REF / IGE_REF_LOADING * true_loading;
            let oge = level_collective(0.0) * true_loading;
            for wrong in [0.88, 1.0, 1.08] {
                // An air whose prediction is `wrong` times the truth.
                let w = true_loading * wrong * CURVE_WEIGHT_KG / CURVE_SIGMA * 1.0;
                let air = Air {
                    pressure_alt_ft: 0.0,
                    oat_c: 15.0,
                    weight_kg: w,
                };
                let (_, old_11, _, _) = lift_to_twelve(lift, oge, Some(air), false);
                let (_, new_11, peak, worst) = lift_to_twelve(lift, oge, Some(air), true);
                let line = format!(
                    "loading {true_loading} predicted x{wrong}: 11 ft {new_11:.1} s (was {old_11:.1}), peak {peak:.1}, held {worst:.1}"
                );
                if peak > 15.0 || worst > 1.0 || new_11 > old_11 || new_11 > 25.0 {
                    bad.push(line);
                }
            }
        }
        assert!(bad.is_empty(), "{bad:#?}");
    }

    /// A takeover of a heavy helicopter in the air: the trim starts at the
    /// light curve's 0.66 and the height goes while the integral finds the
    /// rest; at the budget's figure it holds. A hand-back's lever likewise.
    #[test]
    fn a_heavy_takeover_starts_its_trim_at_the_scaled_curve() {
        let true_loading = 1.25;
        let need = level_collective(0.0) * true_loading;
        let fly = |trim0: f64, level: f64| {
            let dt = 0.37;
            let mut law = CollectiveLaw::from(trim0);
            law.airborne = true;
            law.trim = trim0;
            let (mut h, mut vs, mut acc) = (400.0_f64, 0.0_f64, 0.0_f64);
            let mut queue = std::collections::VecDeque::from(vec![trim0; 2]);
            let mut worst: f64 = 0.0;
            for _ in 0..((60.0 / dt) as usize) {
                law.set_level(Some(level));
                queue.push_back(law.step(
                    target_vs_fpm(Goal::Hover { height_ft: 400.0 }, h, 0.0),
                    vs,
                    false,
                    dt,
                ));
                let c = queue.pop_front().unwrap();
                let want = (3800.0 * (c - need) - vs) / 2.5;
                acc += (want - acc) * (dt / 0.4_f64).min(1.0);
                vs += acc * dt;
                h += vs / 60.0 * dt;
                worst = worst.max((h - 400.0).abs());
            }
            (worst, law.steady())
        };
        // The prediction 4 % light, as the recorded lift-offs err.
        let predicted = level_collective_at(0.0, true_loading * 0.96);
        let (old, old_steady) = fly(level_collective(0.0), level_collective(0.0));
        let (new, new_steady) = fly(predicted, predicted);
        assert!(new < 20.0, "height held to {new:.1} ft");
        assert!(
            old > 1.5 * new,
            "the scaled curve earns its place: {old:.1} vs {new:.1} ft"
        );
        assert!(
            (new_steady - need).abs() < 0.04,
            "hand-back lever {new_steady:.3} vs {need:.3}"
        );
        assert!(
            need - old_steady > 0.07,
            "the light band left {old_steady:.3} vs {need:.3}"
        );
    }

    #[test]
    fn the_touchdown_brake_waits_for_a_stop() {
        assert!(super::touchdown_brake_due(0.3));
        assert!(
            !super::touchdown_brake_due(9.7),
            "the ridge set-down at 19 kt"
        );
        assert!(!super::touchdown_brake_due(f64::NAN));
    }
}
