//! Aircraft profiles.
//!
//! A profile is the per-airframe data that shapes the copilot: which systems
//! exist (and therefore which tools are exposed) and their limits.
//!
//! Profiles load from `profiles/<id>.yaml` at startup ([`load`]), falling back
//! to a built-in Rust definition ([`builtin`]) if the file is missing — so the
//! app works out of the box, and new airframes can be added as YAML without
//! recompiling. Datarefs/commands stay in the Rust tools (they're standard
//! X-Plane); the YAML carries the per-aircraft *config* (flap detents, lights,
//! capability flags).

use serde::{Deserialize, Serialize};

/// Simulator tuning, not a POH power setting. Throttle ratios per second.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct FlarePowerProfile {
    /// The fastest the retard walks the lever, per second, where
    /// `retard_max_per_s` is not declared.
    pub reduction_per_second: f64,
    /// The retard HEIGHT (radio ft): nothing comes off above it, and below
    /// it the lever walks to idle timed to the wheels (`autoland::
    /// FlarePower`). `None` retards from the flare gate. A Boeing retards
    /// at 20 to 30 ft and lands at idle; keyed on the sink, the law fired in
    /// the float the arrest had made (LEMG 13, 2026-09-26: idle at 20 ft
    /// level, -859 fpm and 2.4 g at the wheels) or held until the float was
    /// already spent (LEMG 13, 2026-09-28: inside 185 at 15.8 ft, then held
    /// again for the sink it had rebuilt).
    #[serde(default)]
    pub retard_below_ft: Option<f64>,
    /// The fastest the retard walks the lever, per second (absent:
    /// `reduction_per_second`). Slower, the thrust leaving does not rebuild
    /// the sink in the last second.
    #[serde(default)]
    pub retard_max_per_s: Option<f64>,
    /// The SPEED key: the retard waits while the airspeed is more than this
    /// under the approach speed (kt; absent: `autoland::RETARD_SLOW_KTS`).
    /// The thrust is what holds a slow flare's speed, and a knot the retard
    /// takes comes back as sink at the wheels (the 737's plant: 63 fpm a
    /// knot). It defers only a retard not yet begun on a flare still
    /// sinking: a float takes the thrust off whatever the speed
    /// (`autoland::RETARD_FLOAT_FPM`; EGBB 15, 2026-09-28, issue #146).
    #[serde(default)]
    pub retard_slow_kts: Option<f64>,
    /// The TIME key: below the retard height the lever starts off only once
    /// the time to the wheels (radio over sink) is inside this (s), and
    /// then walks to idle timed to them. `None`: at the retard height.
    #[serde(default)]
    pub retard_at_s: Option<f64>,
    /// ...or once the airspeed is more than this over the approach speed
    /// (kt): a fast flare floats, and the float is the speed.
    #[serde(default)]
    pub retard_fast_kts: Option<f64>,
    /// The short final's power HOLD: from this many seconds of the path's
    /// sink above the runway (at the groundspeed), the speed law only ever
    /// takes power off, as the roundout's does, unless the airspeed projects
    /// under the flare's protected speed (`autoland::short_final_power_held`).
    /// A lever the speed law adds in the seconds before the servos are
    /// released keeps spooling into the roundout with nothing trimming
    /// against it. `None`: held only in the roundout.
    #[serde(default)]
    pub hold_from_s: Option<f64>,
}

/// How the landing arrest's vertical-speed dial leads THIS autopilot's lag
/// (`autoland::vs_dial_fpm`): dial = target + gain x (target - predicted),
/// where predicted = actual + trend_seconds x the sink's rate of change,
/// capped at `max_climb_fpm`. The default is the 172's, fitted to a
/// first-order 4.5 s lag (LXGB 27, 2026-09-14). A jet's vertical-speed
/// mode is second-order with two seconds of dead time (the 737 at EGCC,
/// 2026-09-19: the same gain and cap pinned the dial at +300 for three
/// seconds and ballooned it to +160 fpm at ten feet), so a slower airframe
/// declares a lower gain, a lower cap and a trend term for damping.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ArrestLeadProfile {
    pub gain: f64,
    pub max_climb_fpm: f64,
    #[serde(default)]
    pub trend_seconds: f64,
}

/// The cabin's air: how THIS airframe's bleed (pressurisation) and crew
/// oxygen are switched and read. Declared, never guessed: the SF50's bleed is
/// an xlua slider behind `laminar/SF50/fresh_air_on` (the stock pack dataref
/// is its output), its oxygen master is a toggle command in front of the
/// SUPPLY (`demand_flow_setting`), and the crew MASKS are `o2_valve_on`
/// (worn: the breathing sound plays while it reads 1, `sf50_oxygen.lua`).
/// EDLP-LOWI (2026-09-24): full auto resumed with the engine already running,
/// the bleed was never switched, the cabin followed the aircraft to 24,000 ft
/// and the pilot blacked out; the master read on and the masks were off.
/// The masks were then worn for every mission (#159): now only in the
/// cabin-altitude warning band (`tools::cabin_air::MaskWatch`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CabinAirProfile {
    /// Fired to put the bleed ON.
    pub bleed_on_command: String,
    /// Reads at or above 0.5 with the bleed on.
    pub bleed_read: String,
    /// The crew oxygen SUPPLY (the master): a TOGGLE (read first, fired only
    /// when off). On for every mission.
    #[serde(default)]
    pub oxygen_toggle: Option<String>,
    /// Reads at or above 1 with the supply on.
    #[serde(default)]
    pub oxygen_read: Option<String>,
    /// The crew MASKS, 1 = worn. Never the mission's: donned only in the
    /// cabin-altitude warning band and taken off below it.
    #[serde(default)]
    pub oxygen_masks: Option<String>,
}

/// The altimeter knobs' own fast paths (`tools::instruments::set_altimeter`,
/// issue #94). On a stock airframe the dial write stands and none of this is
/// used. The 737's xlua owns its baro MEMORY and copies it into the stock dial
/// every frame, so the only way in is its knob, a hundredth a press: EGSS
/// (2026-09-22) loaded with the memory at 0.0 and 0 to 30.40 is 3,040 API
/// commands. Its STD button pressed twice puts the memory at 29.92 from
/// anywhere (`B738_baro_memory` resets it while STD shows and nobody has
/// preselected), so a set far from the ask goes to 29.92 first and steps from
/// there. Each name is a hypothesis until the probe and the readback agree.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct AltimeterProfile {
    /// Pressed `std_presses` times to put the captain's setting at 29.92.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub std_command: Option<String>,
    #[serde(default = "one_press")]
    pub std_presses: usize,
    /// The first officer's STD button, pressed the same number of times.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub copilot_std_command: Option<String>,
}

fn one_press() -> usize {
    1
}

/// What an anti-ice or de-ice system protects (`tools::anti_ice`, issue #100).
/// The wing is four different machines, so it is four kinds: a crew switches
/// boots on once ice has formed, and bleed or electric heat before it does.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IceKind {
    Pitot,
    Window,
    /// Engine inlet / cowl anti-ice.
    Engine,
    Prop,
    WingBoots,
    WingBleed,
    WingHeat,
    WingTks,
}

/// One anti-ice or de-ice system. The commands and readback default to the
/// stock `sim/ice/*` pair for the kind (`tools::anti_ice::stock`); a profile
/// overrides them where an add-on owns the switch.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AntiIceSystem {
    pub kind: IceKind,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub on_commands: Vec<String>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub off_commands: Vec<String>,
    /// Read back after the switch; `name[i]` reads one element. At or above
    /// 0.5 is on. Per-engine kinds without an index read every engine.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub readback: Option<String>,
}

/// The anti-ice systems this airframe has. Declared, else read from the
/// `.acf`'s electrical loads (`systems::IceKit`), which can say THAT a wing is
/// protected but not how: an undeclared wing is left alone and said.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct AntiIceProfile {
    #[serde(default)]
    pub systems: Vec<AntiIceSystem>,
}

/// The autopilot's own modes this airframe flies (issue #92): declared after
/// a probe; otherwise the systems survey's autopilot type decides, and a
/// readback that confirms a mode is remembered for the airframe
/// (`systems::ap_modes`). `None` fields say nothing.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct AutoflightProfile {
    /// The stock autothrottle (`sim/autopilot/autothrottle_on`) flies this
    /// airframe's thrust.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub autothrust: Option<bool>,
    /// Level change / FLC (`sim/autopilot/level_change`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub level_change: Option<bool>,
    /// GPS steering / LNAV (`sim/autopilot/gpss`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lnav: Option<bool>,
}

/// The flare's ELEVATOR law for this airframe (`autoland::FlareGains`): the
/// default was fitted to the 172 and the 737. The SF50 answers the column
/// about twice as hard and needs a standing pull at a flare's speed: on the
/// default 0.18 took -572 fpm to +36 in two seconds, 0.35 ballooned it, and
/// the release to neutral sank it at -590 onto the runway (EDLP 24,
/// 2026-09-24).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FlareElevatorProfile {
    /// Scale on the law's P and D terms.
    pub gain: f64,
    /// Scale on its integrator (the standing pull it finds).
    #[serde(default = "one")]
    pub trim_gain: f64,
    /// Most pull the flare may command.
    pub max: f64,
    /// Most push, as a positive number.
    #[serde(default = "default_max_push")]
    pub max_push: f64,
    /// A balloon eases the held pull at most this fast, elevator per second.
    #[serde(default)]
    pub balloon_ease_per_s: Option<f64>,
    /// An EXPONENTIAL flare's time constant, seconds (`autoland::flare_target_fpm`):
    /// the sink target falls with the height instead of holding flat from
    /// the gate. Only for an airframe whose flare is flown on the elevator.
    #[serde(default)]
    pub tau_s: Option<f64>,
    /// How THIS airframe's sink answers the elevator, fitted to its own
    /// flares (`autoland::FlareModel`). Declared, the whole arrest is flown
    /// on the model (the elevator that HOLDS the sink wanted, against the
    /// sink the elevator does not explain) instead of the P+D tracker.
    #[serde(default)]
    pub model: Option<FlareModelProfile>,
}

/// The elevator-to-sink response of an airframe with the servos out, as its
/// own flares recorded it, and what the flare law does with it
/// (`autoland::FlareModel`). A standing pull holds a standing change of sink
/// (`fpm_per_elevator`) arriving through two lags (`lag_s`, `lag2_s`) after a
/// dead time (`dead_s`); near the runway the aircraft pitches down by itself
/// (the tail's ground effect), worth `ground_fpm` of sink at the wheels and
/// growing linearly from `ground_ft`. Fitted to the 737's seven LEMG flares
/// of 2026-09-26 (six on 13, one on 12; 35 fpm rms): 4,970 fpm, 0.84 and
/// 3.1 s, 1.35 s of dead time, 405 fpm of ground effect from 43 ft. The
/// first fit (2,600 fpm, two 1 s lags, 1.8 s, no ground term) landed every
/// recorded gate soft on paper and the LEMG 12 flare at 407.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FlareModelProfile {
    /// Standing change of sink per unit of elevator, fpm.
    pub fpm_per_elevator: f64,
    /// The first lag, seconds.
    pub lag_s: f64,
    /// The second lag, seconds (absent: the first again).
    #[serde(default)]
    pub lag2_s: Option<f64>,
    /// The dead time, seconds.
    pub dead_s: f64,
    /// The sink the ground adds at the wheels, fpm (0: none modelled).
    #[serde(default)]
    pub ground_fpm: f64,
    /// ...growing linearly from this height, ft.
    #[serde(default = "default_model_ground_ft")]
    pub ground_ft: f64,
    /// The law asks the MODEL's sink this far ahead to be the flare's target
    /// at the height the aircraft will have then, seconds (absent: the dead
    /// time plus one).
    #[serde(default)]
    pub horizon_s: Option<f64>,
    /// The sink the flare's target arrives at on the runway, fpm (positive;
    /// absent: the arrest's own quarter of the path sink).
    #[serde(default)]
    pub touchdown_fpm: Option<f64>,
    /// Smoothing on the unexplained sink, seconds.
    #[serde(default = "default_model_observer_s")]
    pub observer_s: f64,
    /// In rough air the flare's pull only comes back (absent: yes, the
    /// 737's LEMG 13 downdraft). `false` flies the prediction in any air: a
    /// strong elevator whose held pull becomes a float bleeds the speed of
    /// a slow wing into the ground (the SF50, issue #163).
    #[serde(default)]
    pub rough_hold: Option<bool>,
    /// A RATE plant: the elevator asks a rate of change of sink, and a
    /// neutral column HOLDS the sink it has (an Airbus's fly-by-wire normal
    /// law: the A330's sidestick). `fpm_per_elevator` is then fpm per
    /// SECOND per unit, `ground_fpm` fpm per second at the wheels, and the
    /// flare may push (to `max_push`): a float or a balloon is ended forward,
    /// since neutral holds it. Absent: the standing-sink plant.
    #[serde(default)]
    pub rate: bool,
    /// A light single's HOLD-OFF flare (the SR22, issue #166), ft: the
    /// exponential flare levels at this height over the runway instead of
    /// flying to it, then settles at `touchdown_fpm` while the idle wing
    /// bleeds its speed, so the wheels come down on the SPEED, near the
    /// stall, not on a sink; and from the first contact the column holds the
    /// pull it touched with, never more and never less, the lever closed
    /// (`autoland::TouchHold`). Absent: the jets' firm-touchdown flare. The
    /// SR22 flew that to the wheels at its approach speed (80 kt, 215 fpm)
    /// and every landing hopped 4 to 10 ft (LEMG 13, 2026-09-29).
    #[serde(default)]
    pub hold_off_ft: Option<f64>,
    /// fpm of sink per knot the speed loses (positive): the prediction
    /// carries the airspeed's trend ahead as sink, so a hold-off at idle
    /// pulls for the knots still to bleed instead of a tick late for each
    /// (`autoland::FlareModel::note_speed`). Absent: the speed is only what
    /// the model does not explain. The SR22: 115 fpm a knot at idle with
    /// full flap (LEMG 13 sortie 1790619465, 2026-09-28).
    #[serde(default)]
    pub fpm_per_kt: Option<f64>,
}

fn default_model_ground_ft() -> f64 {
    40.0
}
fn default_model_observer_s() -> f64 {
    0.3
}

impl FlareModelProfile {
    pub fn valid(self) -> Option<Self> {
        ((500.0..=10_000.0).contains(&self.fpm_per_elevator)
            && (0.1..=5.0).contains(&self.lag_s)
            && self.lag2_s.is_none_or(|l| (0.1..=8.0).contains(&l))
            && (0.0..=4.0).contains(&self.dead_s)
            && (0.0..=1500.0).contains(&self.ground_fpm)
            && (5.0..=150.0).contains(&self.ground_ft)
            && self.horizon_s.is_none_or(|h| (0.3..=8.0).contains(&h))
            && self
                .touchdown_fpm
                .is_none_or(|t| (20.0..=400.0).contains(&t))
            && self.hold_off_ft.is_none_or(|h| (0.5..=5.0).contains(&h))
            && self.fpm_per_kt.is_none_or(|k| (0.0..=400.0).contains(&k))
            && (0.05..=3.0).contains(&self.observer_s))
        .then_some(self)
    }
}

fn default_max_push() -> f64 {
    0.10
}

/// How the airframe's bank answers the roll channel with the servos out, for
/// the landing arrest's wings (`autotakeoff::WingsLevel::rate_plant`). An
/// Airbus's sidestick asks a ROLL RATE and a neutral stick holds the bank, so
/// the stiff wings-level law (full aileron at 6 degrees of error) led only by
/// the bank's measured rate rang through the loop's own delay: the A330 at
/// LEMG 30 (sorties 1790633832 and 1790634412, 2026-09-28) swung +/-2 to 4
/// degrees at a 1.5 to 2 s period with 0.3 to 0.5 of aileron, fitted 11 to 13
/// deg/s per unit after 0.25 to 0.37 s. Declared, the arrest flies the bank
/// the ailerons already written WILL make (a predictor over the dead time)
/// toward its target over `close_s`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ArrestRollProfile {
    /// Roll rate per unit of aileron, deg/s.
    pub deg_s_per_unit: f64,
    /// Before the aileron is felt, seconds (the loop's write included).
    pub dead_s: f64,
    /// The bank error is closed over this, seconds.
    pub close_s: f64,
}

impl ArrestRollProfile {
    /// Used only when every number is a sane one.
    pub fn valid(self) -> Option<Self> {
        ((2.0..=60.0).contains(&self.deg_s_per_unit)
            && (0.0..=1.5).contains(&self.dead_s)
            && (0.2..=3.0).contains(&self.close_s))
        .then_some(self)
    }
}

/// The yaw plant of a free castoring nose (`ground_steer::GroundPlant`),
/// identified from the airframe's own ground ticks: the one ground law's
/// model for the taxi, the line-up, the roll's start and the rollout
/// (`ground_steer`). `rdot = Kb(v) u + rudder v^2 d - D(v) r - Kv(v) slip + w`,
/// with `Kb(v) = brake_dps2 v / (v + brake_from_ms) max(1 + brake_per_ms v,
/// 0.2)`, `D(v) = damp + damp_per_ms v` and `Kv(v) = vane_dps2 (v / 30)^2`,
/// the track following the nose at `side_ms2 / v` per second (v in m/s,
/// degrees). Undeclared, the SR22's.
///
/// It replaces the SR22's `rollout_brakes:` (the split's gain schedule, lead
/// and trim fitted to LEMG 13 1790689843 and 1790694457, 2026-09-29): the
/// rollout's brake difference is now the plant inverted, the trim the
/// disturbance observer.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GroundPlantProfile {
    /// Yaw acceleration per unit of wheel difference, deg/s^2.
    pub brake_dps2: f64,
    /// The brake's authority is half at this groundspeed, m/s.
    pub brake_from_ms: f64,
    /// ...and changes by this share per m/s above it (never under a fifth).
    pub brake_per_ms: f64,
    /// Yaw damping at a standstill, 1/s...
    pub damp: f64,
    /// ...plus this per m/s.
    pub damp_per_ms: f64,
    /// The rudder, deg/s^2 per unit per (m/s)^2.
    pub rudder_per_ms2: f64,
    /// The weathervane, deg/s^2 per degree of slip at 30 m/s.
    pub vane_dps2: f64,
    /// The side force the track follows the nose on, m/s^2 per radian.
    pub side_ms2: f64,
}

impl GroundPlantProfile {
    /// Used only when every number is a sane one.
    pub fn valid(self) -> Option<Self> {
        ((10.0..=600.0).contains(&self.brake_dps2)
            && (0.0..=2.0).contains(&self.brake_from_ms)
            && (-0.5..=0.5).contains(&self.brake_per_ms)
            && (0.0..=3.0).contains(&self.damp)
            && (0.0..=1.0).contains(&self.damp_per_ms)
            && (0.0..=5.0).contains(&self.rudder_per_ms2)
            && (0.0..=100.0).contains(&self.vane_dps2)
            && (1.0..=200.0).contains(&self.side_ms2))
        .then_some(self)
    }
}

/// The rollout's steer on a STEERED nosewheel, when the takeoff roll's law
/// does not fit it (`autoland::RolloutSteer`, issue #197).
///
/// LEMG 13 on the iMac (2026-09-30, the King Air C90B, sorties
/// 1790757702, 1790758429 and 1790758604): each contact yawed the nose left
/// at 8 to 15 deg/s with the steer at zero, the takeoff roll's default law
/// answered with its whole 0.35 of authority at 80 kt on a differenced
/// heading, and the nose swung 30 to 80 degrees right with the steer full
/// left: fitted over the three rollouts (three to four and a half seconds
/// of heading and track, 1.2 to 2.1 degrees rms), 45 to 65 deg/s of yaw per
/// unit at 80 kt after 0.3 s, and a yaw the slip itself feeds (1.5 to 2.4
/// deg/s per degree between the nose and the track) that no heading error
/// sees coming. So the law damps on the MEASURED yaw rate
/// (`sim/flightmodel/position/R`), and its gain rises as the speed falls.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RolloutSteerProfile {
    /// Steer per degree of heading error at `ref_kts` and above.
    pub per_deg: f64,
    /// The groundspeed `per_deg` is the gain at, knots; under it the gain
    /// grows as `ref_kts / gs`...
    pub ref_kts: f64,
    /// ...down to this groundspeed, knots, and no further (the nosewheel's
    /// own low-speed steering takes over under it).
    pub min_kts: f64,
    /// Seconds of the measured yaw rate damped against the error.
    pub damp_s: f64,
    /// The most steer at and above `fast_kts` (the rudder's authority grows
    /// as v^2); the full 0.85 at `min_kts` and under.
    pub max_fast: f64,
    /// The groundspeed `max_fast` applies from, knots.
    pub fast_kts: f64,
}

impl RolloutSteerProfile {
    /// Used only when every number is a sane one.
    pub fn valid(self) -> Option<Self> {
        ((0.002..=0.1).contains(&self.per_deg)
            && (20.0..=200.0).contains(&self.ref_kts)
            && (5.0..=self.ref_kts).contains(&self.min_kts)
            && (0.0..=3.0).contains(&self.damp_s)
            && (0.1..=0.85).contains(&self.max_fast)
            && self.fast_kts > self.min_kts
            && self.fast_kts <= 200.0)
            .then_some(self)
    }
}

/// The live airframe's rollout steer law, when declared.
pub fn rollout_steer() -> Option<RolloutSteerProfile> {
    live_profile()
        .and_then(|p| p.rollout_steer)
        .and_then(RolloutSteerProfile::valid)
}

/// The live airframe's ground plant, when declared and sane.
pub fn ground_plant() -> Option<GroundPlantProfile> {
    live_profile()
        .and_then(|p| p.ground_plant)
        .and_then(GroundPlantProfile::valid)
}

/// How the speed governor leads this airframe's engines
/// (`tools::speedhold::throttle_adjust_with`). The shared law projects the
/// speed 5 s ahead and moves the lever at most 0.015 a tick near its target
/// and 0.04 when urgent: GA numbers. The A330's engines lag about 4.6 s and a
/// unit of lever is about 2.5 kt/s, so the lever came off 0.93 at the
/// autopilot's capture only at 0.06 a second and the clean circuit ran 227.5
/// to 265 kt on the downwind (LEMG 30, sortie 1790633832-full_auto, #169).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SpeedGovernorProfile {
    /// The speed is projected this far ahead, seconds.
    pub lead_s: f64,
    /// Lever per knot of projected error, per tick.
    pub gain: f64,
    /// The most the lever moves in a tick when the projection is urgent.
    pub max_step: f64,
}

impl SpeedGovernorProfile {
    /// Used only when every number is a sane one.
    pub fn valid(self) -> Option<Self> {
        ((2.0..=20.0).contains(&self.lead_s)
            && (0.001..=0.02).contains(&self.gain)
            && (0.02..=0.2).contains(&self.max_step))
        .then_some(self)
    }
}

/// The live airframe's arrest roll plant, when declared and sane.
pub fn arrest_roll() -> Option<ArrestRollProfile> {
    live_profile().and_then(|p| p.arrest_roll).and_then(ArrestRollProfile::valid)
}

/// The live airframe's speed governor, when declared and sane.
pub fn speed_governor() -> Option<SpeedGovernorProfile> {
    live_profile().and_then(|p| p.speed_governor).and_then(SpeedGovernorProfile::valid)
}

/// The rotation's elevator on the wheels (`autotakeoff::RotatePitch`),
/// fitted: the column the nose needs to come up at a rate and to be held up
/// on the main wheels, and how that column carries into the air.
/// Declared, the rotation law feeds that column forward (a share of it,
/// `autotakeoff::ROTATE_MODEL_SHARE`) and its own P+I only corrects; `None`
/// is the P+I alone, as ever.
///
/// Why it exists (LEMG, the SR22, 2026-09-29, #184): on the wheels the tail
/// holds the nose down with a SPRING (its angle of attack is the attitude),
/// 0.033 of column per degree, as stiff as the law's own 0.03 per degree of
/// error. The proportional pull met the spring near 3 degrees and the
/// integrator, 0.01 per degree-second, took five more seconds to find the
/// rest: the nose sat at 3 degrees through 88 to 94 KIAS and the aircraft
/// lifted off at 98 to 100 KIAS on every flight, rotate 73 (1790682305 at a
/// 0.25 s tick, 1790690995 and 1790694884 at 0.4 s).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RotateElevatorProfile {
    /// The airspeed the column figures are given at, knots; the column
    /// figures scale as `(ref_kts / ias)^2`.
    pub ref_kts: f64,
    /// Pitch rate on the wheels, deg/s, per unit of column past what holds
    /// the nose where it is, at `ref_kts`.
    pub deg_s_per_unit: f64,
    /// The column that just lifts the nosewheel at `ref_kts`.
    pub breakout: f64,
    /// More column per degree of the wing's angle of attack past the roll
    /// attitude (the tail's spring; the same at every speed). On the wheels
    /// the angle is the attitude; flying, the attitude less the flight path,
    /// so the spring does not leave with the wheels: it leaves as the path
    /// comes up.
    pub per_deg: f64,
    /// Flying clear of ground effect, the column the breakout becomes, at
    /// `ref_kts` (the spring apart). In ground effect the breakout stands:
    /// 1790699819 needed about 0.31 to hold 4.6 degrees at 88 KIAS a few
    /// feet up, the breakout plus the spring, and the 100 kt climbs hold on
    /// 0.1 at 10 degrees on a 7 degree path, the spring less 0.06.
    pub air_base: f64,
    /// The height ground effect is gone by, feet; `air_base` is reached
    /// linearly from the wheels to it (about half the span).
    pub ground_effect_ft: f64,
}

impl RotateElevatorProfile {
    /// Used only when every number is a sane one.
    pub fn valid(self) -> Option<Self> {
        ((30.0..=300.0).contains(&self.ref_kts)
            && (1.0..=200.0).contains(&self.deg_s_per_unit)
            && (0.0..=0.6).contains(&self.breakout)
            && (0.0..=0.1).contains(&self.per_deg)
            && (-0.5..=0.6).contains(&self.air_base)
            && (3.0..=150.0).contains(&self.ground_effect_ft))
            .then_some(self)
    }
}

/// How hard this airframe's rudder yaws it on the takeoff roll
/// (`autotakeoff::RollSteer`): the yaw rate, deg/s, that a whole unit of the
/// steer channel holds at `at_kts`, growing with the square of the speed (or
/// as `yaw_power` declares).
/// Declared, the roll law's gains are scheduled on it and its trim takes a
/// crosswind in seconds; `None` is the default law. The default's gain per
/// degree at 50 to 80 kt was a quarter of what the 737 needs to hold an 8 kt
/// crosswind, and its trim bled away with the speed faster than it learned:
/// held 5 degrees off, 10 m left, rejected twice (EGKK 26L, 2026-09-27).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RollRudderProfile {
    /// Yaw rate, deg/s, per unit of steer at `at_kts`.
    pub yaw_dps: f64,
    /// The groundspeed that figure was measured at, knots.
    pub at_kts: f64,
    /// How the rudder grows with the speed below `at_kts`: yaw per unit as
    /// (v / at_kts) to this power (2 when `None`, the free stream's v^2). A
    /// rudder in a propeller's wash is nearly flat: the Evolution's fits as
    /// v^0.35 to v^0.5 (LILB 03, 2026-09-29/30), and a v^2 schedule below
    /// 80 kt read a third of it at 45 kt and tripled the gain there.
    #[serde(default)]
    pub yaw_power: Option<f64>,
    /// A propeller's standing rudder at full power, units of steer: led in
    /// with the throttle for the whole roll in place of the generic power
    /// lead (0.25 at a standstill, fading to 0.1 by 40 kt). The SR22 holds
    /// about 0.35 from 30 kt to Vr in calm air (LEMG 13, 2026-09-29).
    #[serde(default)]
    pub power_steer: Option<f64>,
    /// The fastest groundspeed that standing rudder was measured at, knots:
    /// past it the lead falls as the square of the speed (the propeller's
    /// moment holds, the rudder's grows as v^2). `None` believes it to
    /// `at_kts`. Held flat to Vr, the SR22's 0.35 plus the learned trim met a
    /// need of 0.2 at 88 kt and swung the nose 5 degrees right on the wheels
    /// (LEMG 13 1790682305, 2026-09-29).
    #[serde(default)]
    pub power_steer_to_kts: Option<f64>,
    /// A turboprop's standing rudder per 1,000 ft-lb of MEASURED torque,
    /// units of steer: led in on the torque read each tick in place of
    /// `power_steer` times the lever, wherever a torque is read (with
    /// [`Self::power_steer_share`] past `power_steer_to_kts`). A torque
    /// limiter moves the lever, and a PT6 at a held lever keeps making
    /// torque: the Evolution needs 0.51 per 1,000 ft-lb from 12 to 78 kt
    /// (LILB 03, three rolls, 2026-09-29/30), while its lever cycled 0.55 to
    /// 0.76 under the limiter at 1,093 to 1,402 ft-lb (1790736651).
    #[serde(default)]
    pub power_steer_per_kftlb: Option<f64>,
    /// The heading closure the roll law's gain is sized for, per second
    /// (`autotakeoff::RUDDER_CLOSE_PER_S` when `None`, the 737's). A standing
    /// need the lead and trim did not predict is held off at `need / gain`
    /// degrees, so a faster closure is a smaller standing error.
    #[serde(default)]
    pub close_per_s: Option<f64>,
    /// The trim's corner, per second of the same gain
    /// (`autotakeoff::RUDDER_TRIM_PER_S` when `None`).
    #[serde(default)]
    pub trim_per_s: Option<f64>,
}

impl RollRudderProfile {
    /// Used only when both numbers are sane ones (a power steer out of
    /// range is dropped).
    pub fn valid(self) -> Option<Self> {
        let power_steer = self.power_steer.filter(|s| (0.0..=0.6).contains(s));
        let power_steer_to_kts = self.power_steer_to_kts.filter(|k| (20.0..=250.0).contains(k));
        let power_steer_per_kftlb = self.power_steer_per_kftlb.filter(|s| (0.0..=1.0).contains(s));
        let yaw_power = self.yaw_power.filter(|p| (0.0..=3.0).contains(p));
        let close_per_s = self.close_per_s.filter(|c| (0.3..=3.0).contains(c));
        let trim_per_s = self.trim_per_s.filter(|c| (0.05..=1.0).contains(c));
        ((0.5..=100.0).contains(&self.yaw_dps) && (20.0..=200.0).contains(&self.at_kts))
            .then_some(Self {
                power_steer,
                power_steer_to_kts,
                power_steer_per_kftlb,
                yaw_power,
                close_per_s,
                trim_per_s,
                ..self
            })
    }

    /// The share of `power_steer` believed at this groundspeed: whole up to
    /// the speed it was measured at ([`Self::power_steer_to_kts`], else
    /// `at_kts`), falling as the square of the speed past it.
    pub fn power_steer_share(&self, gs_kts: f64) -> f64 {
        let to = self.power_steer_to_kts.unwrap_or(self.at_kts);
        if gs_kts.is_finite() && gs_kts > to {
            (to / gs_kts).powi(2)
        } else {
            1.0
        }
    }

    /// The yaw rate a unit of steer holds at this groundspeed, deg/s.
    pub fn yaw_dps_at(&self, gs_kts: f64) -> f64 {
        let v = if gs_kts.is_finite() {
            gs_kts.max(0.0)
        } else {
            0.0
        };
        match self.yaw_power {
            Some(p) => self.yaw_dps * (v / self.at_kts).powf(p),
            None => self.yaw_dps * (v / self.at_kts).powi(2),
        }
    }
}

impl FlareElevatorProfile {
    /// Used only when every number is a sane one.
    pub fn valid(self) -> Option<Self> {
        ((0.1..=3.0).contains(&self.gain)
            && (0.0..=10.0).contains(&self.trim_gain)
            && (0.05..=0.6).contains(&self.max)
            && (0.0..=0.3).contains(&self.max_push)
            && self
                .balloon_ease_per_s
                .is_none_or(|r| r.is_finite() && r > 0.0))
        .then_some(Self {
            model: self.model.and_then(FlareModelProfile::valid),
            ..self
        })
    }
}

impl Default for ArrestLeadProfile {
    fn default() -> Self {
        Self {
            gain: 2.0,
            max_climb_fpm: 300.0,
            trend_seconds: 0.0,
        }
    }
}

impl ArrestLeadProfile {
    /// A declared lead is only used when every number is a number.
    pub fn valid(self) -> Option<Self> {
        (self.gain.is_finite()
            && (0.0..=5.0).contains(&self.gain)
            && self.max_climb_fpm.is_finite()
            && (0.0..=1000.0).contains(&self.max_climb_fpm)
            && self.trend_seconds.is_finite()
            && (0.0..=5.0).contains(&self.trend_seconds))
        .then_some(self)
    }
}

/// Everything the tool layer needs to know about one airframe.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AircraftProfile {
    pub id: String,
    pub display_name: String,
    /// Other ICAO types a SimBrief plan for THIS airframe may be filed as,
    /// when SimBrief has no base type of its own for it (the Lancair
    /// Evolution, acf_ICAO EVOT, is planned as a custom airframe on the
    /// Piper M500, P46T). The plan audit's type check accepts them.
    #[serde(default)]
    pub simbrief_types: Vec<String>,
    /// Which loaded aircraft this profile is FOR: its `acf_ICAO` and the
    /// model (`acf_relative_path`, optionally author and description). The
    /// stock Laminar 737 and the Zibo both say B738, and one ICAO cannot
    /// pick between them (2026-09-27). `None` is a profile detection never
    /// picks (a pinned `XCOPILOT_AIRCRAFT` still can).
    #[serde(default, rename = "match")]
    pub matcher: Option<ProfileMatch>,
    /// Flap system, if any. `None` means no `set_flaps` tool.
    #[serde(default)]
    pub flaps: Option<FlapProfile>,
    /// Fuel endurance monitoring thresholds. `None` means no fuel callouts.
    #[serde(default)]
    pub fuel: Option<FuelProfile>,
    /// Clean (flaps-up) stall speed, KIAS — drives the monitor's airspeed
    /// callouts. `None` means no stall caution.
    #[serde(default)]
    pub vs_clean_kts: Option<f64>,
    /// Never-exceed speed (Vne), KIAS — drives the monitor's overspeed callout.
    /// `None` means no overspeed caution.
    #[serde(default)]
    pub vne_kts: Option<f64>,
    /// A HIGHER Vmo above an altitude (`vmo_high: { above_ft: 8000, kts:
    /// 350 }`), for an airframe whose limit rises with height: `vne_kts` is
    /// then the LOW limit, the one below `above_ft` and the one a caller
    /// without an altitude gets. The Citation X is 270 KIAS below 8,000 ft
    /// and 350 above; one number capped its cruise at 233 and would have
    /// nagged "overspeed" over 243 at FL350. `None` = one Vne at every
    /// altitude ([`vne_at`]).
    #[serde(default)]
    pub vmo_high: Option<VmoHigh>,
    /// Landing-gear operating/extended limit speed (VLO/VLE), KIAS — the `gear`
    /// tool refuses cycling the gear above it. `None` = no gear speed gate.
    #[serde(default)]
    pub vlo_kts: Option<f64>,
    /// The airframe's own landing-gear warning envelope (the horn and the
    /// warning annunciator), from its `.acf` at detection when the profile
    /// is silent. The takeoff raises the gear only once doing so will not
    /// sound it: the SF50's fires below 125 kt with the takeoff flap set,
    /// and the gear came up at 200 ft and 122 kt (ETND, 2026-09-23).
    #[serde(default)]
    pub gear_warning: Option<GearWarning>,
    /// Nominal DC bus voltage (12 or 24). The engine-start "battery's low"
    /// gate scales with it: a healthy 12 volt RV-10 read "only 12 volts"
    /// against the 172's 24 volt threshold (2026-09-11). `None` = 24.
    #[serde(default)]
    pub bus_volts_nominal: Option<f64>,
    /// Optional progressive autoland power reduction; absent retains idle flare.
    #[serde(default)]
    pub flare_power: Option<FlarePowerProfile>,
    /// The least thrust (lever ratio) the autoland holds on a stabilised
    /// final, when not more than a few knots fast. Jets fly the approach on
    /// thrust; `None` lets the speed law idle the engines (the pistons).
    #[serde(default)]
    pub approach_thrust_min: Option<f64>,
    /// How the landing arrest leads this autopilot's lag. `None` = the
    /// 172's fitted numbers (see [`ArrestLeadProfile`]).
    #[serde(default)]
    pub arrest_lead: Option<ArrestLeadProfile>,
    /// The flare's elevator law, when the default does not fit.
    #[serde(default)]
    pub flare_elevator: Option<FlareElevatorProfile>,
    /// The rudder's authority on the takeoff roll, when measured.
    #[serde(default)]
    pub roll_rudder: Option<RollRudderProfile>,
    /// The rotation's elevator on the wheels, when fitted.
    #[serde(default)]
    pub rotate_elevator: Option<RotateElevatorProfile>,
    /// The bank's answer to the roll channel in the arrest, when fitted.
    #[serde(default)]
    pub arrest_roll: Option<ArrestRollProfile>,
    /// The yaw plant of a free castoring nose (`ground_steer`), when fitted.
    #[serde(default)]
    pub ground_plant: Option<GroundPlantProfile>,
    /// The rollout's steer on a steered nosewheel, when fitted.
    #[serde(default)]
    pub rollout_steer: Option<RolloutSteerProfile>,
    /// The speed governor's lead on this airframe's engines, when fitted.
    #[serde(default)]
    pub speed_governor: Option<SpeedGovernorProfile>,
    /// Bleed air and crew oxygen, where the airframe has them.
    #[serde(default)]
    pub cabin_air: Option<CabinAirProfile>,
    /// None = unknown; do not invent a pressurisation system in warnings.
    #[serde(default)]
    pub pressurised: Option<bool>,
    /// Static reference figures (V-speeds, weights, limits) the copilot recites
    /// when asked. Per-airframe, from the POH/POM. `None` = no reference answers.
    #[serde(default)]
    pub reference: Option<ReferenceProfile>,
    /// How briskly the copilot taxis this airframe. `None` = the built-in GA
    /// pace (see [`TaxiProfile`]).
    #[serde(default)]
    pub taxi: Option<TaxiProfile>,
    /// The thrust-lever dataref this airframe obeys, when it is not the stock
    /// one. `None` = `throttle_ratio_all` (see [`throttle_dataref`]).
    #[serde(default)]
    pub throttle_dataref: Option<String>,
    /// Pitch attitude at which this airframe's tail touches the runway,
    /// degrees. The auto-takeoff rotation will not command within a margin of
    /// it. `None` on airframes short enough that elevator authority runs out
    /// long before tail clearance does — every GA single so far.
    #[serde(default)]
    pub tail_strike_deg: Option<f64>,
    /// Nose-DOWN elevator the rotation law may command, 0..1. `None` = the GA
    /// tenth, which is right for a light single and is not authority at all
    /// on a jet — see `autotakeoff::PITCH_MIN`.
    #[serde(default)]
    pub pitch_authority_down: Option<f64>,
    /// The highest pitch attitude the hand-flown initial climb may hold,
    /// degrees, once clear of the runway (see `autotakeoff::ClimbHold`). With
    /// it the rotation holds the initial climb speed by PITCH from liftoff to
    /// the autopilot's engage, the way a crew flies V2 + 15; without it the
    /// law holds its fixed rotation attitude, as every GA single always has.
    /// EGPH 2026-09-21 and LEMG 2026-09-28: at 8 degrees a 737 at takeoff
    /// thrust went 162 to 185 kt between liftoff and 300 ft.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub climb_pitch_max_deg: Option<f64>,
    /// How far ahead the circuit's final leg leads the offset it pursues,
    /// seconds (see `circuit::final_led_xtk_m`): an autopilot whose heading
    /// answers its bug slowly runs through the centreline on the bare
    /// pursuit. Silent: no lead, the pursuit every airframe has flown. The
    /// A330 ran 66 to 67 m through at LEMG 30 (2026-09-29, #189).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub circuit_final_lead_s: Option<f64>,
    /// Path to an XChecklist file to interpret (relative to the working dir).
    /// `None` means no checklists for this airframe. The file is read-only input;
    /// the copilot interprets it (see [`crate::checklist`]).
    #[serde(default)]
    pub checklist: Option<String>,
    /// Maps the checklist file's datarefs to curated actions, per-airframe. Empty
    /// means the copilot only verifies/calls out checklist items (never acts).
    #[serde(default)]
    pub checklist_map: Vec<ChecklistMapEntry>,
    /// Which file section the checklist flow ([`crate::flow`]) reads at each
    /// phase boundary, for files whose titles the section words do not find:
    /// `climb`, `cruise`, `descent`, `before_landing`, `after_landing` -> the
    /// file's own section title. Empty (the default) means the section words
    /// match the titles themselves, as they do in the 172 and SR22 files.
    #[serde(default)]
    pub checklist_flow: std::collections::BTreeMap<String, String>,
    /// Phase-expectation advisor: switch states the airframe *should* be in per
    /// flight phase; the monitor advises on the delta ("yaw damper's still off"
    /// at climb). Declarative SOP knowledge — the deterministic version of a
    /// human copilot's "aren't you forgetting something?".
    #[serde(default)]
    pub advisor: Vec<AdvisorItem>,
    /// Exterior/interior lights this airframe has. Empty means no `set_lights` tool.
    #[serde(default)]
    pub lights: Vec<LightProfile>,
    /// Heading/altitude/VS autopilot. `false` means no autopilot tools.
    #[serde(default)]
    pub autopilot: bool,
    /// Extra prerequisites for airframes whose AP needs more than servos-on to
    /// engage (the C90 needs its flight director up and AC power from an inverter).
    /// `None` → plain servos-on (the GA singles). See [`AutopilotPrep`].
    #[serde(default)]
    pub autopilot_prep: Option<AutopilotPrep>,
    /// Piston engine controls (throttle, mixture).
    #[serde(default)]
    pub engine: bool,
    /// Move the gear lever with the stock gear COMMANDS, not a handle write
    /// (see `tools::gear::Gear::by_command`).
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub gear_by_command: bool,
    /// Thrust levers (`set_throttle`) without the piston controls — for turbine
    /// aircraft that have a throttle but no mixture/magnetos/piston start. A piston
    /// `engine` already implies a throttle, so it's only set for jets/turboprops.
    #[serde(default)]
    pub throttle: bool,
    /// Constant-speed propeller lever (`set_prop`) — the blue levers that set the
    /// governed prop RPM. `None` (fixed-pitch, e.g. the 172) → no prop tool.
    #[serde(default)]
    pub prop: Option<PropProfile>,
    /// Turbine start sequence (`start_engines`) — bleed/fuel/ignition/crank with N2
    /// monitoring, as opposed to the piston `engine` group's one-shot start.
    #[serde(default)]
    pub turbine_start: bool,
    /// Procedural power-up (`power_up`) — "all electrical systems on" runs the
    /// electrical bring-up step by step (battery, APU, bleed, buses, generators),
    /// calling out each, rather than a one-shot shortcut.
    #[serde(default)]
    pub auto_prep: bool,
    /// The aircraft scripts its OWN start behind the stock `auto_start`
    /// command (an xlua `replace_command`): one press runs the whole cold and
    /// dark bring-up and the engine start with the aircraft's real switches.
    /// The stock 737-800: battery, standby power, fire tests, APU, packs, then
    /// starter 2 and 1 on N2, generators, probe heat, bleeds, APU off, 124 s
    /// from cold to both running (probed 2026-09-17). `StartKind::Scripted`:
    /// full auto fires the command and waits on the engines.
    #[serde(default)]
    pub scripted_auto_start: bool,
    /// Per-engine FUEL CUTOFF lever, when the stock `mixture_ratio` is not the
    /// control the aircraft is actually cut with. Empty = write
    /// `sim/cockpit2/engine/actuators/mixture_ratio[i]`, which is right on
    /// every airframe whose levers the sim itself owns (the MD-82).
    ///
    /// The stock 737-800 is the case that found this (2026-09-20): its
    /// `B738.systems.lua` rewrites `mixture_ratio[0..1]` EVERY FRAME from
    /// `laminar/B738/engine/slider/condition_lever{1,2}`, which is itself an
    /// animation output driven toward `..._target`. So the stock write is
    /// accepted (`is_writable: true`), overwritten on the next frame, and does
    /// nothing: the copilot said "fuel levers cut" at a Dublin stand and the
    /// engines ran on until the pilot cut them by hand. The writable control is
    /// the `_target` pair, and only a readback can tell the two apart.
    #[serde(default)]
    pub fuel_cutoff_levers: Vec<String>,
    /// The value those levers take at CUTOFF. Defaults to 0.0, which is cutoff
    /// on every lever seen so far; a lever whose sense is inverted says so here.
    #[serde(default)]
    pub fuel_cutoff_value: f64,
    /// Airliner autoflight extras: `set_speed` (AP airspeed select) and
    /// `autothrottle` (engage/disengage). For jets with an autothrottle.
    #[serde(default)]
    pub autothrottle: bool,
    /// Speedbrakes/spoilers (`speedbrake` extend/retract/arm). For aircraft that
    /// have them (the GA singles don't).
    #[serde(default)]
    pub speedbrake: bool,
    /// Reverse thrust (`reverse_thrust`) — rollout-only, self-stowing below
    /// taxi speed. For turboprops/jets whose model has a usable beta/reverse
    /// range on `throttle_beta_rev_ratio` (the Twin Otter).
    #[serde(default)]
    pub reverse_thrust: bool,
    /// Adjustable altimeter (`set_barometer`).
    #[serde(default)]
    pub barometer: bool,
    /// Vacuum directional gyro that needs syncing (`align_gyro`).
    #[serde(default)]
    pub directional_gyro: bool,
    /// COM1 radio (`set_com1`).
    #[serde(default)]
    pub com_radio: bool,
    /// Transponder (`set_transponder` / squawk).
    #[serde(default)]
    pub transponder: bool,
    /// Override for airframes whose transponder MODE selector is an add-on's
    /// own knob, where the stock `transponder_mode` actuator is not an input
    /// at all but an OUTPUT the add-on rewrites every frame from that knob
    /// (the RWDesigns Twin Otter's KT73 — see
    /// [`TransponderModeKnob`]). `None` = the stock actuator, the universal case.
    #[serde(default)]
    pub transponder_mode_knob: Option<TransponderModeKnob>,
    /// What this airframe's crew sets before the roll and at the gear beyond
    /// flaps, trim and the altitude selector (see [`crate::takeoff_config`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub takeoff_config: Option<TakeoffConfigProfile>,
    /// The manual's takeoff power limit, where it has one (see
    /// [`TakeoffPowerProfile`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub takeoff_power: Option<TakeoffPowerProfile>,
    /// The seat belt sign switch, where it is the add-on's own (see
    /// [`crate::tools::signs`]). `None` with `cabin_crew` is the stock switch.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub seat_belt_signs: Option<SeatBeltSigns>,
    /// The circuit selects its pattern altitude on the altitude dial and lets
    /// the autopilot's capture hold the level legs, instead of flying altitude
    /// through the VS dial (`circuit::ap_holds_level`). Per airframe because
    /// it rests on the stock VS command taking the aircraft OUT of an altitude
    /// hold, which is still unproven on the 737's MCP (history, 2026-09-19).
    #[serde(default)]
    pub ap_altitude_capture: bool,
    /// Autoland parks the altitude dial at the FIELD (0 ft) instead of above
    /// the aircraft. The stock 737's MCP will not fly a V/S descent away from
    /// a window above it: parked at 5,600 over a 3,400 ft inbound, the VS
    /// dial's -570 held level and both LEMG 13 finals arrived high
    /// (2026-10-05); with the window at 0 the descent and the go-around
    /// climb both followed the dial. Off: parked above (the LEAX rule).
    #[serde(default)]
    pub alt_dial_park_field: bool,
    /// How far ahead of the stabilised gate the landing flap is selected,
    /// nm (`autoland::landing_flap_nm`); `None` = the 1 nm default. The 737's
    /// full flap pitched it up and its V/S gave -100 to -400 fpm against
    /// -1,200 for 12 s at 3.5 nm (LEMG 13, 2026-10-05): the flap must be out
    /// before the path is held.
    #[serde(default)]
    pub landing_flap_lead_nm: Option<f64>,
    /// ILS approach setup (`prepare_ils`) — needs a NAV radio + approach mode.
    #[serde(default)]
    pub ils: bool,
    /// Parking brake (`set_parking_brake`).
    #[serde(default)]
    pub parking_brake: bool,
    /// Cabin crew aboard (the interphone persona — see [`crate::cabin`]).
    /// Airliner-class airframes only; `XCOPILOT_CABIN_CREW` overrides for
    /// testing on anything.
    #[serde(default)]
    pub cabin_crew: bool,
    /// Whether full auto may load this airframe from the plan at the arm
    /// (`full_auto::arm_load`, issue #95). `stock` (the default) writes the
    /// stock `weight/m_fuel` and `weight/m_fixed`, read back; `unprobed` and
    /// `own_fuel_system` are named and left to the pilot, never forced. The
    /// spoken "load the aircraft" is the pilot's and ignores it.
    #[serde(default)]
    pub load_sheet: LoadSheet,
    /// Fly an ILS COUPLED on this airframe's own approach mode (see
    /// `ils::Coupling`): armed on final, the bug and VS dial handed over as
    /// the localizer and glideslope capture. Off until a capture has been
    /// flown on the type; the copilot's own path otherwise.
    #[serde(default)]
    pub ils_coupled: bool,
    /// The autopilot's own approach button and its status, where the add-on
    /// has its own (the Laminar 737's APP is `laminar/B738/autopilot/app_press`;
    /// the stock command armed nothing and the coupling was abandoned after
    /// six seconds, LEMG 13 2026-09-26). `None`: the stock command/dataref.
    #[serde(default)]
    pub approach_command: Option<String>,
    #[serde(default)]
    pub approach_status: Option<String>,
    /// The `park_brake_valve` value that means SET on this airframe, where the
    /// valve is modeled. The sense is PER-AIRCRAFT (C90B: 0 = set, verified live;
    /// SR22: 1 = set — the opposite, seen as an "inverted" handle). `None` = don't
    /// touch the valve (ratio only), the safe default for unknown airframes.
    #[serde(default)]
    pub park_brake_valve_set: Option<f64>,
    /// Electrical systems the `set_electrical` tool exposes (per-airframe wiring).
    /// Empty means no electrical tool.
    #[serde(default)]
    pub electrical: Vec<ElectricalSystem>,
    /// Cockpit flood lights (`set_flood_lights`). Separate from `lights` because
    /// the floods use a mix of write mechanisms (a scalar plus an array element)
    /// that the scalar-per-light `set_lights` can't express.
    #[serde(default)]
    pub flood_lights: bool,
    /// Override dataref for the flood lights when the airframe's flood isn't the
    /// universal pilot/copilot pair — a single on/off switch, optionally an array
    /// element written `name[i]` (the King Air's overhead flood is
    /// `sim/cockpit2/switches/generic_lights_switch[0]`). `None` → universal floods.
    #[serde(default)]
    pub flood_lights_dataref: Option<String>,
    /// Override for airframes whose flood switch is a TOGGLE command with a
    /// read-only position dataref, which no write can drive. Read first, fired
    /// only when the position differs — the Citation X rule: a toggle fired as
    /// "on" turns the system off whenever it was already on.
    ///
    /// The Baron 58 is the case that found this. Its profile declared
    /// `flood_lights_dataref: laminar/b58/lighting/flood_light_switch`, and
    /// probing the live sim showed that name is a COMMAND, not a dataref — the
    /// airframe has no dataref by that name at all, so every `set_flood_lights`
    /// on the Baron was a write into nothing that then read back nothing
    /// (2026-08-02). Its real position lives at
    /// `laminar/b58/flood_light_toggle/switch_pos`, read-only.
    #[serde(default)]
    pub flood_lights_toggle: Option<ToggleSwitch>,
    /// A command fired TWICE when the aircraft comes alive, to acknowledge the
    /// GNS430's welcome/database page — the screen that otherwise sits there
    /// until somebody presses ENT. `None` on airframes with no such unit.
    /// Fired once per power-up (see [`crate::cockpit_auto`]), never repeated
    /// while the bus stays live, so a pilot navigating the unit is never
    /// interrupted by a stray ENT.
    #[serde(default)]
    pub gps_ack_command: Option<String>,
    /// Override switch for pitot heat when the stock command is a no-op — the
    /// airframe's script drives the heat from its own control and overwrites the
    /// stock datarefs every frame (MD-82: `laminar/md82/ice/heatknob`, 0 = off,
    /// 1–9 = on; the tool writes 1/0). `None` → the stock pitot-heat command.
    #[serde(default)]
    pub pitot_heat_dataref: Option<String>,
    /// Electric/auxiliary fuel pump (`set_fuel_pump`).
    #[serde(default)]
    pub fuel_pump: bool,
    /// Override for airframes whose boost pumps are custom commands rather than
    /// the universal `fuel_pump_on` array (the RWDesigns Twin Otter's FWD/AFT
    /// pumps live behind SASL commands; writing the stock array is a no-op).
    /// When set, `set_fuel_pump` fires these and confirms from `readback`
    /// (entries may be `name[i]`). Exposes the tool even when `fuel_pump` is
    /// false.
    #[serde(default)]
    pub fuel_pump_commands: Option<PumpCommands>,
    /// Tank indices of the stock `sim/cockpit2/fuel/fuel_tank_pump_on` whose
    /// pumps come OFF when that tank is empty with an engine running (see
    /// [`crate::tools::fuel_pumps`]): the 737's CTR pumps were left running
    /// dry for a whole flight after the scripted start switched them on over
    /// an empty centre (2026-09-27). Only a CENTRELINE tank with wing tanks
    /// beside it is ever honoured (`acf_tank_X`); a main tank is never a
    /// switch to tidy. Declare only where the add-on's scripts leave the
    /// stock array as the cockpit switch. Empty (silent) = off.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub fuel_tank_pumps_off_when_empty: Vec<usize>,
    /// Magneto/ignition selector (`set_magnetos`) — for the run-up mag check.
    #[serde(default)]
    pub magnetos: bool,
    /// Carburetor / alternate-air heat (`set_carb_heat`).
    #[serde(default)]
    pub carb_heat: bool,
    /// Number of engines — how many array elements the per-engine accessory
    /// tools (carb heat, fuel pump, magnetos) write. Defaults to 1 (the GA
    /// singles); a twin like the DC-3 sets 2 so both engines switch together.
    #[serde(default = "default_engines")]
    pub engines: usize,
    /// Drive the magnetos via the per-engine `ignition_on` selector (off/R/L/
    /// both) instead of the single `ignition_key` — for radials like the DC-3.
    #[serde(default)]
    pub magnetos_per_engine: bool,
    /// Fuel tank selector (`set_fuel_selector`), per-airframe positions.
    /// `None` → no selector tool.
    #[serde(default)]
    pub fuel_selector: Option<FuelSelectorProfile>,
    /// DC power switch driven by up/down cycle commands (battery / off / ground
    /// power) — the AWX DC-3 pattern. Gates `set_power`. `None` → no power tool.
    #[serde(default)]
    pub power_switch: Option<PowerSwitchProfile>,
    /// Fuel selectors driven by up/down cycle commands, one per side (the DC-3
    /// has two). Empty → no cycle fuel-selector tool. Distinct from
    /// `fuel_selector` (the single direct-command selector on the GA singles).
    #[serde(default)]
    pub fuel_selectors: Vec<CycleSelectorProfile>,
    /// Twin piston start/shutdown for cycle-switch airframes (mixture/fuel are
    /// xlua-overridden). Gates `start_engines` / `stop_engines`. `None` → use the
    /// generic `engine` group instead.
    #[serde(default)]
    pub piston_start: Option<PistonStartProfile>,
    /// Notched mixture lever (xlua-overridden). Gates `set_mixture` for airframes
    /// whose mixture is a detented lever rather than a continuous 0–100%. `None`
    /// → no notched mixture tool.
    #[serde(default)]
    pub mixture_notch: Option<MixtureNotchProfile>,
    /// Switch/condition-lever turboprop start (the Laminar King Air pattern).
    /// Gates `start_engines` / `stop_engines`. Distinct from `turbine_start`
    /// (the MD-82 jet sequence) and the piston `engine`/`piston_start` groups.
    #[serde(default)]
    pub turboprop_start: Option<TurbineStartProfile>,
    /// Mode-selector + master-switch jet start: the AIRLINER pattern, and the
    /// one the existing blocks could not express.
    ///
    /// `turbine_start` is the MD-82's, hardcoded to `laminar/md82/*`;
    /// `turboprop_start` is condition levers and starter commands. An Airbus
    /// or a 737 has neither — it has an ENG MODE knob and two guarded master
    /// switches, so every airliner here but the MD-82 had `turbine_start:
    /// false` and no way to start its own engines at all (found in the A330,
    /// 2026-09-20).
    #[serde(default)]
    pub airliner_start: Option<AirlinerStartProfile>,
    /// Pitot heat anti-ice (`set_pitot_heat`).
    #[serde(default)]
    pub pitot_heat: bool,
    /// Elevator pitch trim (`set_pitch_trim`).
    #[serde(default)]
    pub pitch_trim: bool,
    /// NAV1 radio + OBS course (`set_nav1`, `set_obs`).
    #[serde(default)]
    pub nav_radio: bool,
    /// Second comm radio + active/standby swap (`set_com2`, `swap_radio`).
    #[serde(default)]
    pub com2_radio: bool,
    /// Whether the gear retracts. `false` (e.g. the 172) means no `gear` tool.
    #[serde(default)]
    #[allow(dead_code)]
    pub retractable_gear: bool,
    /// A helicopter. Every flying and ground law here is fixed-wing (a
    /// rotation through the pitch channel, a flare through the VS dial, a
    /// throttle governor, a pattern sized to a bank angle), so the registry
    /// withholds them all ([`crate::tools::ROTORCRAFT_WITHHELD`]) whatever
    /// the other flags say; the crew keeps the switches, the radios and the
    /// autopilot dials. The S-76C, 2026-09-16.
    #[serde(default)]
    pub rotorcraft: bool,
    /// A helicopter's engine start and shutdown ([`crate::tools::heli`]):
    /// held starters, throttles as the fuel control (0 is OFF), the rotor
    /// read back at its governed speed.
    #[serde(default)]
    pub heli_start: Option<HeliStartProfile>,
    /// Transition altitude (ft MSL): at or above it, "set the altimeter" dials in
    /// standard pressure (29.92 inHg / 1013 hPa) instead of local QNH — that's the
    /// rule above the transition altitude. Defaults to 18,000 ft (US FL180); lower
    /// it per-profile for regions with a lower transition altitude.
    #[serde(default = "default_transition_altitude")]
    pub transition_altitude_ft: f64,
    /// How far the snap-to-panel camera moves toward the saved pose: 1.0 = all the
    /// way (the captured close-up), 0.5 = a half glance from wherever you're
    /// looking — position and angles interpolated, so the view turns toward the
    /// panel without zooming into it.
    #[serde(default = "default_view_blend")]
    pub view_blend: f64,
    /// Cockpit geography: lowercase item name → spoken location ("on the left
    /// subpanel, next to the avionics master"). Answers "where is the X?"
    /// deterministically — the one thing PDF manuals can't carry (their location
    /// diagrams don't survive text extraction). Grows per-airframe as verified.
    #[serde(default)]
    pub locations: std::collections::HashMap<String, String>,
    /// Snap-to-panel: tool name → a saved 3D-cockpit camera position to jump to
    /// just before that action runs, so you watch the switch move. Empty (default)
    /// = no snapping. Positions are captured live from the pilot's own framing (the
    /// copilot reads the head datarefs and stores them here), so no fiddly X-Plane
    /// quick-look saving. E.g. `gear: {x: -0.35, y: 0.83, z: -0.44, psi: 12, the: -20}`.
    #[serde(default)]
    pub view_map: std::collections::HashMap<String, PanelView>,
    /// The altimeter knobs' fast paths ([`AltimeterProfile`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub altimeter: Option<AltimeterProfile>,
    /// Anti-ice and de-ice systems ([`AntiIceProfile`]); `None` reads them
    /// from the `.acf`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub anti_ice: Option<AntiIceProfile>,
    /// The autopilot's own modes ([`AutoflightProfile`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub autoflight: Option<AutoflightProfile>,
}

/// A saved 3D-cockpit camera position — the pilot's-head datarefs
/// (`sim/graphics/view/pilots_head_*`): position relative to the CG in metres and
/// the view angles in degrees. Restored by writing them back after selecting the
/// 3D cockpit view. `phi` (roll) defaults to 0 — rarely needed.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PanelView {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    /// Heading (yaw), degrees.
    pub psi: f64,
    /// Pitch, degrees.
    pub the: f64,
    #[serde(default)]
    pub phi: f64,
}

/// US transition altitude (FL180). See [`AircraftProfile::transition_altitude_ft`].
fn default_transition_altitude() -> f64 {
    18_000.0
}

/// Full snap by default — profiles opt into a softer glance. See `view_blend`.
fn default_view_blend() -> f64 {
    1.0
}

/// Engine count for the per-engine accessory tools — 1 (the GA singles) unless
/// a profile says otherwise. See [`AircraftProfile::engines`].
fn default_engines() -> usize {
    1
}

/// Autopilot engagement prerequisites beyond servos-on, for airframes that need
/// them. The C90's AP won't hold without its **flight director** switch up (its
/// `fd_monitor` drops the servos otherwise) and is **AC-powered**, so with the
/// inverter off it has no power to engage at all — which lets the tool name the
/// real cause ("no AC power, turn the inverter on") instead of a generic hint.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct AutopilotPrep {
    /// Commands to raise the flight director before engaging the servos. Empty =
    /// none. C90: `laminar/c90/avionics/switch/fd_up`.
    #[serde(default)]
    pub flight_director: Vec<String>,
    /// True if the AP is AC-powered (needs an inverter). Drives the honest
    /// "no AC power" readback when it fails to engage.
    #[serde(default)]
    pub ac_powered: bool,
}

/// Constant-speed propeller control, per airframe. The blue levers command a
/// *governed RPM*, not a raw position; X-Plane's actuator dataref carries it in
/// radians/second (all engines), so the tool works in RPM (what the pilot says and
/// the tach shows) and converts. Named detents map to the airframe's book settings;
/// an explicit RPM is clamped to [`min_rpm`, `max_rpm`]. Verified live on the C90B
/// with `scripts/xp_watch.py` (the lever moves `..._rad_sec_all`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PropProfile {
    /// Actuator dataref (rad/sec, all engines). C90B:
    /// `sim/cockpit2/engine/actuators/prop_rotation_speed_rad_sec_all`.
    pub dataref: String,
    /// Full-forward / takeoff governed RPM ("props full", "props up").
    pub max_rpm: f64,
    /// Cruise detent RPM ("props to cruise").
    pub cruise_rpm: f64,
    /// Aft governing limit / low RPM ("props back", "props low").
    pub min_rpm: f64,
}

/// Fuel-endurance alert thresholds (minutes of fuel remaining at the current
/// burn). The monitor warns as the figure drops through these — caution first,
/// then warning. Defaults are the usual VFR reserves (30 min day / 45 min night
/// → caution at 45, warning at 30), per-airframe so a thirstier type can raise
/// them.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FuelProfile {
    /// Endurance (minutes) at or below which to caution.
    pub reserve_caution_min: f64,
    /// Endurance (minutes) at or below which to warn (more urgent).
    pub reserve_warning_min: f64,
    /// Nominal cruise fuel burn (kg/hr). Used by the *ground* check to estimate
    /// endurance before takeoff (current idle burn would read as near-infinite),
    /// so "you're low for a flight" can fire while parked. The 172 burns ~24 kg/hr.
    pub cruise_burn_kg_hr: f64,
    /// Density of what this airframe actually burns, kg per US gallon.
    /// `None` = 100LL (2.72), which is every piston here.
    ///
    /// Jet A-1 is 3.04. Briefing a 737's kerosene at avgas density inflated
    /// every fuel figure it spoke by 11.8% (BIKF-EGPH, 2026-09-20: 3,126 kg
    /// in the tanks read back as "1,149 gallons" — 1,028 is the truth).
    #[serde(default)]
    pub kg_per_gal: Option<f64>,
    /// The unit the CREW of this airframe works in: "gallons", "kg" or "lb".
    /// `None` = gallons.
    ///
    /// An airliner is never fuelled in gallons, and the app briefed one in
    /// them because gallons were the GA unit it was built around — while full
    /// auto's own preflight quoted the SAME fuel in pounds. Two parts of one
    /// cockpit, two units, one of them not the airframe's.
    #[serde(default)]
    pub spoken_unit: Option<String>,
}

/// How a given airframe's fuel is spoken.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FuelSpoken {
    Gallons,
    Kilos,
    Pounds,
}

/// The live airframe's fuel density and spoken unit, republished on every
/// profile swap — a process-wide fact must never outlive its airframe.
static FUEL_UNIT: std::sync::Mutex<(f64, FuelSpoken)> =
    std::sync::Mutex::new((AVGAS_KG_PER_GAL, FuelSpoken::Gallons));

/// 100LL, the default for anything that declares nothing.
pub const AVGAS_KG_PER_GAL: f64 = 2.72;
const LB_PER_KG: f64 = 2.204_62;

/// Publish the airframe's fuel density and spoken unit. Called on EVERY swap.
pub fn publish_fuel_unit(f: Option<&FuelProfile>) {
    let density = f
        .and_then(|f| f.kg_per_gal)
        .filter(|d| d.is_finite() && *d > 0.0)
        .unwrap_or(AVGAS_KG_PER_GAL);
    let unit = match f
        .and_then(|f| f.spoken_unit.as_deref())
        .unwrap_or("gallons")
        .trim()
        .to_ascii_lowercase()
        .as_str()
    {
        "kg" | "kgs" | "kilos" | "kilograms" => FuelSpoken::Kilos,
        "lb" | "lbs" | "pounds" => FuelSpoken::Pounds,
        _ => FuelSpoken::Gallons,
    };
    *FUEL_UNIT.lock().unwrap_or_else(|e| e.into_inner()) = (density, unit);
}

/// The live airframe's (kg per gallon, spoken unit).
pub fn fuel_unit() -> (f64, FuelSpoken) {
    *FUEL_UNIT.lock().unwrap_or_else(|e| e.into_inner())
}

/// Thousands separators. A local copy on purpose: this module compiles into
/// the LIB as well as the bin, and `auto_status` is the bin's.
fn commas(n: i64) -> String {
    let (sign, mut d) = (if n < 0 { "-" } else { "" }, n.abs().to_string());
    let mut out = String::new();
    while d.len() > 3 {
        out.insert_str(0, &format!(",{}", &d[d.len() - 3..]));
        d.truncate(d.len() - 3);
    }
    format!("{sign}{d}{out}")
}

/// `kg` of fuel as this airframe's crew would say it, with its unit word.
pub fn say_fuel_kg(kg: f64) -> String {
    say_fuel_figure(fuel_in_unit_kg(kg))
}

/// `kg` as a NUMBER in the airframe's unit — for a caller that has to round
/// before it subtracts. Three independently rounded figures do not add up:
/// "You have 1149 aboard, but the plan needs 3002; short about 1852" was off
/// by one because each was rounded from its own float (2026-09-20).
pub fn fuel_in_unit_kg(kg: f64) -> f64 {
    let (density, unit) = fuel_unit();
    match unit {
        FuelSpoken::Gallons => kg / density,
        FuelSpoken::Kilos => kg,
        FuelSpoken::Pounds => kg * LB_PER_KG,
    }
}

/// The same figure with no unit word — for the second and third numbers in a
/// list that has already named the unit once.
pub fn fuel_figure_kg(kg: f64) -> String {
    commas(fuel_in_unit_kg(kg).round() as i64)
}

/// A fuel FLOW in kg/hr, as this airframe's crew would say it. Same rule as
/// the quantity: an airliner's burn is tonnes an hour, not gallons an hour at
/// the density of petrol.
pub fn say_fuel_flow_kg_hr(kg_hr: f64) -> String {
    let (density, unit) = fuel_unit();
    match unit {
        FuelSpoken::Gallons => format!("{:.1} gallons an hour", kg_hr / density),
        FuelSpoken::Kilos => format!("{} kilos an hour", commas(kg_hr.round() as i64)),
        FuelSpoken::Pounds => format!(
            "{} pounds an hour",
            commas((kg_hr * LB_PER_KG).round() as i64)
        ),
    }
}

/// A number already in the airframe's unit, said with its unit word.
pub fn say_fuel_figure(n: f64) -> String {
    let word = match fuel_unit().1 {
        FuelSpoken::Gallons => "gallons",
        FuelSpoken::Kilos => "kilos",
        FuelSpoken::Pounds => "pounds",
    };
    format!("{} {word}", commas(n.round() as i64))
}

/// Fuel tank selector positions, per airframe — the tool's whole surface is
/// config, because airframes differ: the 172 selector is LEFT/BOTH/RIGHT (OFF is
/// a separate shutoff valve, not a selector position); the SR22 is OFF/LEFT/RIGHT
/// with no "both". Each position fires a `command` (commands animate the 3D knob;
/// a raw dataref write may not), then — when `readback` is set and the position
/// carries a known `value` — the dataref is read to confirm; otherwise the
/// command is trusted.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FuelSelectorProfile {
    /// Dataref read after switching to confirm the position (an enum int).
    /// `None` → trust the command (no confirmation read).
    #[serde(default)]
    pub readback: Option<String>,
    /// Selectable positions, in the order offered to the LLM.
    pub positions: Vec<FuelPosition>,
}

/// One fuel-selector position.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FuelPosition {
    /// Token the LLM selects by and returns (e.g. `"left"`, `"both"`, `"off"`).
    pub key: String,
    /// Spoken form for the readback (e.g. `"left"`, `"both"`, `"off"`).
    pub spoken: String,
    /// Command that selects this position (preferred — animates the knob).
    pub command: String,
    /// Expected `readback` value for this position. `None` → can't confirm by
    /// value, so the command is trusted.
    #[serde(default)]
    pub value: Option<f64>,
}

/// A multi-position switch driven by up/down CYCLE commands rather than a
/// direct "set to N" — the AWX DC-3 pattern (power switch, fuel selectors). To
/// reach a target the tool reads `position`, then fires `up`/`down` the right
/// number of times. (Writing the position dataref doesn't work on the DC-3: the
/// add-on's dataref handlers are no-ops, so only the commands move the system.)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CycleSwitch {
    /// Integer-position dataref read to find the current detent.
    pub position: String,
    /// Command that increments the position by one.
    pub up: String,
    /// Command that decrements the position by one.
    pub down: String,
    /// Lowest detent value.
    pub min: i64,
    /// Highest detent value.
    pub max: i64,
    /// Whether the switch wraps `max`→`min` (the fuel selectors do; the power
    /// switch is clamped at the ends).
    #[serde(default)]
    pub wrap: bool,
}

/// One named target position of a [`CycleSwitch`] (the token the LLM selects by,
/// its spoken form, and the detent value to cycle to).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SwitchState {
    pub key: String,
    pub spoken: String,
    pub value: i64,
}

/// The DC power switch (battery / off / ground power) on a cycle switch. Gates
/// the `set_power` tool. `None` = no power tool.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PowerSwitchProfile {
    pub switch: CycleSwitch,
    /// Selectable named positions, in the order offered to the LLM.
    pub states: Vec<SwitchState>,
    /// Dataref read to confirm the battery is live (any element > 0). `None` =
    /// trust the switch position.
    #[serde(default)]
    pub battery_readback: Option<String>,
}

/// One fuel selector on a cycle switch (the DC-3 has one per side). Gates the
/// `set_fuel_selector` tool when any are present; also drives the start
/// sequence's "fuel to a main tank" step.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CycleSelectorProfile {
    /// Token the LLM selects this selector by (e.g. `left`, `right`).
    pub key: String,
    /// Spoken form (e.g. `left fuel selector`).
    pub spoken: String,
    pub switch: CycleSwitch,
    /// Engine index (0-based) this selector feeds — for the start sequence.
    #[serde(default)]
    pub engine: usize,
    /// Position value to select for engine start (a main tank). `None` = leave
    /// the selector where it is during start.
    #[serde(default)]
    pub start_value: Option<i64>,
    /// Selectable positions, in the order offered to the LLM.
    pub positions: Vec<SwitchState>,
}

/// Twin (or more) turboprop start, switch/condition-lever driven — the Laminar
/// King Air pattern. Gates `start_engines` / `stop_engines` for turbines. The
/// sequence per the verified C90B start: condition levers to low idle, then for
/// each engine fire its start switch (crank), wait for N1 to reach idle, release
/// the starter, bring that engine's generator online (so the bus can power the
/// next start), and finally condition levers to the run position.
/// The APU bring-up a jet's engine start needs first.
///
/// A turbofan is motored by BLEED AIR, and on the Citation X the APU also
/// powers a bus the engine's own start gate reads. Verified live 2026-07-28:
/// with batteries on but the APU off, the starter command is accepted, the
/// annunciator lights and `ignition_key` goes to 4 — and the engine does not
/// turn at all. With the APU generator and bleed on, the same command spooled
/// N2 from 0 to 30% in fifteen seconds. A turboprop needs none of this, so the
/// whole block is optional.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApuStartProfile {
    /// Toggle for the APU master, fired only if `master_readback` says it's off.
    pub master_toggle: String,
    pub master_readback: String,
    /// The starter command, HELD until the APU lights (see `light_n1`).
    ///
    /// It is a spring-loaded momentary on the Citation X, and the script only
    /// engages the starter after the command has been held longer than a
    /// second: `phase == 1 and duration > 1.0`, springing back to ON the
    /// instant it is released. Firing it twice for a second and a half each —
    /// which is what "press it, then press it again" looks like in code — gives
    /// the APU about half a second of starter and it never lights.
    pub starter_commands: Vec<String>,
    /// APU speed at which it is lit and self-sustaining, so the starter can be
    /// let go and it spools to `ready_n1` on its own. The aircraft's own
    /// checklist says "HOLD TILL START" against N1 > 10.
    #[serde(default = "default_apu_light_n1")]
    pub light_n1: f64,
    /// APU speed dataref and the value that counts as running.
    pub n1_dataref: String,
    #[serde(default = "default_apu_ready_n1")]
    pub ready_n1: f64,
    /// How long to wait for it to come up (it took ~25 s live).
    #[serde(default = "default_apu_ready_secs")]
    pub ready_secs: f64,
    /// Fired once it's running — generator, then bleed air.
    #[serde(default)]
    pub after_ready_commands: Vec<String>,
}

fn default_apu_ready_n1() -> f64 {
    95.0
}
fn default_apu_light_n1() -> f64 {
    15.0
}
fn default_apu_ready_secs() -> f64 {
    90.0
}

/// One switch on the way to a running APU: fire `command`, but only when the
/// switch is not already where we want it.
///
/// Read-before-fire, because these are TOGGLES: firing one blind turns a
/// switch that was already on back off. The same rule the transponder knob
/// and the light pairs learned, and the reason `position` is a separate name
/// from `command` — on this aircraft they are frequently the same string
/// meaning two different things.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StartStep {
    pub command: String,
    /// The switch POSITION to read first. `None` fires unconditionally, which
    /// is right for a momentary button and wrong for anything that latches.
    #[serde(default)]
    pub position: Option<String>,
    #[serde(default = "one")]
    pub want: f64,
    /// A momentary press-and-hold (the APU START button is one), seconds.
    /// `None` is a single press.
    #[serde(default)]
    pub held_secs: Option<f64>,
    /// For a KNOB rather than a toggle: the command that steps the other way,
    /// used when the position sits ABOVE `want`.
    ///
    /// The ADIRS selectors are three-position (OFF / NAV / ATT) and the
    /// aircraft's own script picks a direction from where the knob is. A
    /// toggle model cannot say that: told only "press right until it reads
    /// NAV", a knob left in ATT walks the wrong way.
    #[serde(default)]
    pub down_command: Option<String>,
    /// A milestone SPOKEN once this step has confirmed ("Fuel pumps on, eight
    /// of eight."). Most steps carry none: a crew's flow is silent, and the
    /// voice says the state of the aircraft, not the name of every switch.
    /// Every step is still blackboxed and shown on the panel transcript.
    #[serde(default)]
    pub say: Option<String>,
}

/// The airliner start: bleed air up, ENG MODE to IGN/START, each master
/// switch lifted past its guard and set, then the knob back to NORM.
///
/// Every field is a name the aircraft answers to, so every one is a
/// hypothesis until the swap-time probe confirms it — `profile_references`
/// walks this like the rest of the profile.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AirlinerStartProfile {
    /// ENG MODE selector, and the values for IGN/START and NORM.
    pub mode_selector: String,
    pub mode_start: f64,
    pub mode_norm: f64,
    /// Engine master switch ARRAY — one element per engine.
    pub masters: String,
    #[serde(default = "one")]
    pub master_on: f64,
    #[serde(default)]
    pub master_off: f64,
    /// Per-engine guard the master sits under, lifted to `guard_lift` while
    /// the switch is set and dropped `guard_secs` later. Empty when the
    /// switch has no guard. The A330's masters do: its own autostart lifts,
    /// sets, and drops after a quarter second — a bare switch write with the
    /// guard down is the kind of silent no-op a study-level add-on is full of.
    #[serde(default)]
    pub master_guards: Vec<String>,
    #[serde(default = "guard_lift_default")]
    pub guard_lift: f64,
    #[serde(default = "guard_secs_default")]
    pub guard_secs: f64,
    /// The order the engines are started in, as ENGINE INDICES (0 = number
    /// one). Empty = 0, 1, 2... The A330's POM starts number two first, the
    /// way Airbus crews do; the MD-82's and the 737's start number one.
    #[serde(default)]
    pub start_order: Vec<usize>,
    /// Bringing the APU up: an ORDERED list of switch presses, because that
    /// is what the aircraft has. Batteries, APU battery, APU master, APU
    /// START, bleed.
    ///
    /// Not a list of datarefs. `laminar/A333/buttons/APU_start` exists as
    /// BOTH a dataref and a command — the dataref is the button's position,
    /// which the aircraft rewrites every frame, so setting it is the silent
    /// no-op a study-level add-on is full of. It was declared as a dataref
    /// here first and the APU never moved (2026-09-20).
    #[serde(default)]
    pub apu_steps: Vec<StartStep>,
    /// The N1 gauge that says the bleed source is actually there.
    pub apu_n1: String,
    /// Everything between a running APU and the first master switch: bleed
    /// air, the beacon, and the FUEL PUMPS. Same read-before-fire rule.
    ///
    /// The pumps are the reason this list exists. The first version of this
    /// block went straight from the APU to the master switches, which is a
    /// start with no fuel at the engine — and the aircraft's own sequence
    /// spends eight of its twenty-four steps on them (2026-09-20).
    #[serde(default)]
    pub pre_start_steps: Vec<StartStep>,
    /// ...and everything after both engines are running: packs, APU bleed
    /// off, APU off. Leaving the APU running on its bleed is not a failed
    /// start, but it is not a finished one either.
    #[serde(default)]
    pub post_start_steps: Vec<StartStep>,
    /// The APU N1 at which it can supply start air.
    #[serde(default = "apu_ready_default")]
    pub apu_ready_n1: f64,
    /// Spool gauge and the value that means this engine is away. N2 on most,
    /// N3 on a three-spool Trent — the profile names which dataref.
    pub spool: String,
    #[serde(default = "spool_idle_default")]
    pub spool_idle: f64,
}

fn one() -> f64 {
    1.0
}
fn guard_lift_default() -> f64 {
    0.002
}
fn guard_secs_default() -> f64 {
    0.25
}
fn apu_ready_default() -> f64 {
    95.0
}
fn spool_idle_default() -> f64 {
    50.0
}

/// A helicopter's engine start, measured live on the S-76C (2026-09-22):
/// throttle 0 is OFF and anything above it lets fuel in (the mixture follows
/// the throttle by itself), the starter is held, each engine settles at about
/// 62 % N1 on the idle throttle, and both throttles to FLY bring the rotor to
/// its governed speed.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HeliStartProfile {
    /// Per-engine starter command, HELD while the engine cranks.
    pub starters: Vec<String>,
    /// Per-engine throttle array: 0 off, `idle` to start, `fly` to run.
    pub throttle: String,
    pub idle: f64,
    pub fly: f64,
    /// N1 at which an engine is lit and self-sustaining on the idle throttle.
    pub idle_n1: f64,
    /// How long one engine may crank before it is given up on.
    #[serde(default = "heli_crank_secs")]
    pub crank_secs: f64,
    /// Collective array (element 0 is the lever). A start with it up is refused.
    pub collective: String,
    /// Rotor brake ratio, released before the start.
    #[serde(default)]
    pub rotor_brake: Option<String>,
    /// Rotor speed (element 0) and the governed figure it should reach.
    pub rotor_rpm: String,
    pub rotor_rpm_governed: f64,
    /// Fired once both engines are at FLY.
    #[serde(default)]
    pub generators: Vec<String>,
    /// Fired after the generators: what the rest of the start leaves on so
    /// the autopilot can engage (the S-76's autopilot refused every mode
    /// until the stick trim / yaw damper and the inverter were on).
    #[serde(default)]
    pub after_start: Vec<String>,
    /// The tail wheel lock's TOGGLE command (the engage command did nothing on
    /// the S-76), fired only when the lock reads off. Unlocked, rotor torque
    /// pivoted the S-76 on its main wheels at up to 20 deg/s with the brake
    /// set and the pedals neutral, until the engines were cut (ETNW,
    /// 2026-09-23).
    #[serde(default)]
    pub tail_wheel_lock: Option<String>,
}

fn heli_crank_secs() -> f64 {
    40.0
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TurbineStartProfile {
    /// Condition-lever (fuel) array dataref. Each engine's element is written.
    pub condition_lever: String,
    /// Condition value for starting (low idle) and for normal run (high idle).
    pub condition_start: f64,
    pub condition_run: f64,
    /// Per-engine command that engages the starter (the ign/start switch UP).
    pub start_up: Vec<String>,
    /// Per-engine command that releases the starter (the switch back to centre).
    /// May be empty when `starter_held` is set (nothing to release).
    #[serde(default)]
    pub start_dn: Vec<String>,
    /// The starter is a momentary press-and-hold control, not a latching switch:
    /// `start_up` is *held* for up to `crank_secs` (the Web API's command
    /// duration) instead of fired once, and `start_dn` is skipped. The Lancair
    /// Evolution's ENGINE START button (`sim/starters/engage_starter_1`) works
    /// this way; the C90's ign/start switch latches and stays `false`.
    #[serde(default)]
    pub starter_held: bool,
    /// Per-engine generator-ON command, fired once that engine reaches idle.
    #[serde(default)]
    pub generators: Vec<String>,
    /// The airframe's OWN securing control, fired by `stop_engines` after the
    /// condition levers go to cutoff. Empty = the levers are the whole story,
    /// which is the case on every free turbine here.
    ///
    /// Same shape as [`AircraftProfile::fuel_pump_commands`] and
    /// [`TransponderModeKnob`]: the universal write is not wrong, it is simply
    /// not the control the aircraft is secured with. The SF50 is the case that
    /// found this — its POM's Parking checklist says "ENGINE START / STOP -
    /// STOP", which is the ignition knob to OFF, and sf50_ignition.lua's
    /// handler is what actually fires `sim/starters/shut_down_1` *and* drops
    /// the boost pump and igniter with it. Cutting the mixture alone winds the
    /// engine down but leaves the knob at RUN with the pump and igniter live,
    /// which is not a secured aircraft.
    ///
    /// Fired AFTER the cutoff, never instead of it: the lever write is the
    /// backstop if the command is a no-op, and every airframe that declares
    /// nothing here behaves exactly as it did before this existed.
    #[serde(default)]
    pub shutdown_commands: Vec<String>,
    /// N1 (%) at which the engine is lit and stable enough to cut the starter.
    #[serde(default = "default_n1_idle")]
    pub n1_idle: f64,
    /// Introduce fuel at this N2 (%) instead of the default N1 threshold.
    ///
    /// A free turbine and a turbofan spool differently, and the sequence was
    /// written against the first: fuel goes in at 12% N1, which on a PT6 is a
    /// well-lit starter motor. On the Citation X's AE3007 the aircraft's own
    /// procedure introduces fuel at 10% N2, where N1 is barely off the stop —
    /// wait for 12% N1 there and the starter has already auto-cut (its script
    /// stops it at 40% N2) with no fuel ever introduced. `None` keeps the N1
    /// behaviour every turboprop profile was tuned on.
    #[serde(default)]
    pub fuel_intro_n2: Option<f64>,
    /// Max seconds to crank one engine before giving up.
    #[serde(default = "default_turbine_crank_secs")]
    pub crank_secs: f64,
    /// Turn each engine's boost pump on before cranking it (the C90 POM's fuel
    /// panel: the boost pump feeds the engine "before it starts" — the
    /// engine-driven pump takes over once running). Written to the universal
    /// `fuel_pump_on[i]`.
    #[serde(default)]
    pub boost_pumps: bool,
    /// Per-engine auto-ignition ARM command, fired before cranking (POM: "Engine
    /// Auto Ignition Arm – ON"). Empty = airframe has none.
    #[serde(default)]
    pub ignition_arm: Vec<String>,
    /// The APU bring-up this airframe needs before it can crank at all.
    /// `None` on every free turbine (see [`ApuStartProfile`]).
    #[serde(default)]
    pub apu: Option<ApuStartProfile>,
}

fn default_n1_idle() -> f64 {
    50.0
}
fn default_turbine_crank_secs() -> f64 {
    30.0
}

/// A notched mixture lever (the AWX DC-3's xlua-overridden control: idle
/// cutoff / auto-lean / auto-reach / max, written as 0..3). Gates `set_mixture`.
/// Distinct from the GA singles' continuous 0–100% mixture (the `engine` group).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MixtureNotchProfile {
    /// Lever dataref written to select a notch (drives `ENGN_mixt` via override).
    pub lever: String,
    /// Readback dataref to confirm the notch settled. `None` = trust the write.
    #[serde(default)]
    pub readback: Option<String>,
    /// Selectable notches, lean → rich, in the order offered to the LLM.
    pub notches: Vec<SwitchState>,
}

/// Twin piston start/shutdown for cycle-switch airframes whose mixture and fuel
/// are xlua-overridden (so the generic piston `engine` group can't drive them).
/// Gates `start_engines` / `stop_engines`. Universal datarefs (throttle, engine
/// running, bus volts) stay in the tool; the per-airframe commands live here.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PistonStartProfile {
    /// Per-engine starter commands, held to crank.
    pub starters: Vec<String>,
    /// Command setting all magnetos to BOTH (start) and OFF (shutdown).
    pub mags_both: String,
    pub mags_off: String,
    /// Notched mixture lever dataref (xlua-overridden); the rich notch used for
    /// start and the cutoff notch used for shutdown.
    pub mixture_lever: String,
    pub mixture_start_notch: f64,
    pub mixture_cutoff_notch: f64,
    /// Seconds to hold each starter.
    #[serde(default = "default_crank_secs")]
    pub crank_secs: f64,
    /// Throttle crack fraction (0..1) for the start.
    #[serde(default = "default_throttle_crack")]
    pub throttle_crack: f64,
    /// Per-engine generator/alternator ON commands, fired once running.
    #[serde(default)]
    pub generators: Vec<String>,
}

fn default_crank_secs() -> f64 {
    6.0
}
fn default_throttle_crack() -> f64 {
    0.12
}

/// One phase-expectation rule: in the named phases, the dataref should read on
/// (or off); otherwise the copilot speaks `say` once, as an Advisory.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdvisorItem {
    /// Switch/annunciator to read (scalar float; > 0.5 = on).
    pub dataref: String,
    /// Expected state in the listed phases (default: on).
    #[serde(default = "advisor_expect_on_default")]
    pub expect_on: bool,
    /// Phases where the expectation applies, lowercase (`ground`, `takeoff`,
    /// `climb`, `cruise`, `approach`, `landing`).
    pub phases: Vec<String>,
    /// What the copilot says when the state doesn't match.
    pub say: String,
    /// A command the copilot FIRES to fix the state itself, rather than only
    /// advising — the strobe/landing-light SOP rows the pilot wants actioned,
    /// not asked about. `None` = advisory only (the pilot's switch to flip).
    #[serde(default)]
    pub command: Option<String>,
}

fn advisor_expect_on_default() -> bool {
    true
}

/// A flap system as an ordered list of detents, retracted → fully extended.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlapProfile {
    pub detents: Vec<FlapDetent>,
    /// The flap-handle ratio set for takeoff (0.0 = clean). `None` means the
    /// auto-takeoff leaves the flaps where they are. 0.0 on a clean-takeoff type
    /// (the 172) still matters — it ensures the config before the roll.
    #[serde(default)]
    pub takeoff_ratio: Option<f64>,
}

/// The climb-out retraction schedule from a flap profile: `(handle ratio,
/// manoeuvring speed)` for every detent that declares one, cleanest first.
/// Empty when none do, which keeps the old single-shot clean-up.
pub fn flap_schedule(f: Option<&FlapProfile>) -> Vec<(f64, f64)> {
    let mut v: Vec<(f64, f64)> = f
        .map(|f| f.detents.as_slice())
        .unwrap_or(&[])
        .iter()
        .filter_map(|d| Some((d.ratio, d.retract_above_kts?)))
        .filter(|(r, k)| r.is_finite() && k.is_finite() && *k > 0.0)
        .collect();
    v.sort_by(|a, b| a.0.total_cmp(&b.0));
    v
}

/// The spoken name of the detent at `ratio` ("one", "ten degrees"), for the
/// call a crew makes as it selects it. `None` when no detent sits there.
pub fn flap_detent_spoken(f: Option<&FlapProfile>, ratio: f64) -> Option<String> {
    f?.detents
        .iter()
        .find(|d| (d.ratio - ratio).abs() < 1e-3)
        .map(|d| d.spoken.trim().to_string())
        .filter(|s| !s.is_empty())
}

/// One flap handle position.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlapDetent {
    /// Canonical token the LLM selects by and returns (e.g. `"10"`, `"full"`).
    pub label: String,
    /// Spoken form for the readback (e.g. `"ten degrees"`).
    pub spoken: String,
    /// Value written to the flap-handle dataref (0.0 up .. 1.0 full).
    pub ratio: f64,
    /// Max indicated airspeed (kts) at which it's safe to *extend* to this
    /// detent — the placard Vfe. `None` for retraction (always allowed).
    #[serde(default)]
    pub vfe_kts: Option<f64>,
    /// MINIMUM indicated airspeed (kts) at which this configuration is safe to
    /// fly — the detent's manoeuvring speed, and therefore the gate on
    /// RETRACTING to it. `None` leaves the detent out of the climb-out
    /// retraction schedule.
    ///
    /// Retraction had no speed gate at all: the takeoff cleaned up in ONE
    /// step, flaps to zero, on height plus `initial_climb_kts`. On a light
    /// single that is right — one flap setting, and Vy is the speed. On a
    /// 737 it took flaps 5 straight to 0 at 657 ft and 170 kt, some forty
    /// knots below flaps-up manoeuvring speed (BIKF-EGPH, 2026-09-20).
    #[serde(default)]
    pub retract_above_kts: Option<f64>,
    /// The stall speed IN this configuration, kts, at the same weight as
    /// `vs_clean_kts` and `vs0_kts`. `None` = the straight line between
    /// them. A slatted wing takes most of its reduction at the first
    /// detent: the A330's CONF 2 is 139 by its own .acf, the line said 150,
    /// and the monitor called "getting slow" at 175 kt (EGPF 2026-09-21).
    #[serde(default)]
    pub vs_kts: Option<f64>,
}

/// Flaps lower the stall speed by up to this fraction at full extension
/// when the profile declares no landing-configuration stall (172: clean Vs
/// 48, full-flap Vs0 40, about 17%).
pub const FLAP_STALL_REDUCTION: f64 = 0.17;

/// The declared per-detent stall speeds as `(handle ratio, kts)`, cleanest
/// first. Empty when no detent declares one. Pure.
pub fn detent_stall_points(f: Option<&FlapProfile>) -> Vec<(f64, f64)> {
    let mut v: Vec<(f64, f64)> = f
        .map(|f| f.detents.as_slice())
        .unwrap_or(&[])
        .iter()
        .filter_map(|d| Some((d.ratio, d.vs_kts?)))
        .filter(|(r, k)| r.is_finite() && k.is_finite() && *r > 0.0 && *r <= 1.0 && *k > 0.0)
        .collect();
    v.sort_by(|a, b| a.0.total_cmp(&b.0));
    v
}

/// The stall speed at a flap handle ratio: through the airframe's declared
/// per-detent stall speeds when it has them (clean at 0, Vso at 1 unless a
/// detent says otherwise, straight between neighbours), else the straight
/// line from the clean stall to Vso, else the trainer's fixed reduction.
/// Never above clean. Pure.
pub fn stall_speed_in_config(
    vs_clean_kts: f64,
    vs0_kts: Option<f64>,
    points: &[(f64, f64)],
    flap_ratio: f64,
) -> f64 {
    let f = if flap_ratio.is_finite() {
        flap_ratio.clamp(0.0, 1.0)
    } else {
        0.0
    };
    let vs0 = vs0_kts.filter(|v| *v > 0.0 && *v < vs_clean_kts);
    let mut knots: Vec<(f64, f64)> = points
        .iter()
        .copied()
        .filter(|(r, k)| r.is_finite() && k.is_finite() && *r > 0.0 && *r <= 1.0 && *k > 0.0)
        .map(|(r, k)| (r, k.min(vs_clean_kts)))
        .collect();
    if knots.is_empty() {
        return match vs0 {
            Some(vs0) => vs_clean_kts - (vs_clean_kts - vs0) * f,
            None => vs_clean_kts * (1.0 - FLAP_STALL_REDUCTION * f),
        };
    }
    knots.sort_by(|a, b| a.0.total_cmp(&b.0));
    knots.insert(0, (0.0, vs_clean_kts));
    if let (Some(v0), true) = (vs0, knots.last().is_some_and(|k| k.0 < 1.0)) {
        knots.push((1.0, v0));
    }
    for w in knots.windows(2) {
        let ((r0, k0), (r1, k1)) = (w[0], w[1]);
        if f <= r1 {
            let t = if r1 > r0 { (f - r0) / (r1 - r0) } else { 1.0 };
            return k0 + (k1 - k0) * t;
        }
    }
    knots.last().map(|k| k.1).unwrap_or(vs_clean_kts)
}

/// Lift-curve slope the .acf model assumes, per degree of angle of attack a
/// slat adds (thin-aerofoil order; the per-detent result moves under 1 kt
/// between 0.08 and 0.12 on the A330).
const ACF_LIFT_SLOPE_PER_DEG: f64 = 0.1;

/// The per-detent stall speeds (detents 1..n, whole kts) an `.acf` implies,
/// or `None` unless it declares SLATS (`acf/_slat1_inc` over 0 with a
/// non-zero `_slat1_dn`): a flap-only wing keeps the straight line. X-Plane
/// flies the wing from these numbers: each detent adds the slat's stall-angle
/// increase (`_slat1_inc` x its `_slat1_dn/i` fraction) and the flap's lift
/// (`_flap1_cl` x `_flap1_dn/i` over the last notch); the two ends are
/// calibrated to the file's own `_Vs_kts` and `_Vso_kts` (lift goes as
/// 1/V^2). The A330: 147 / 139 / 131 for CONF 1 / 2 / 3. Pure.
pub fn detent_stall_from_acf(acf_text: &str) -> Option<Vec<f64>> {
    let mut vals: std::collections::HashMap<&str, f64> = std::collections::HashMap::new();
    for l in acf_text.lines() {
        let mut it = l.split_whitespace();
        if it.next() != Some("P") {
            continue;
        }
        if let (Some(k), Some(v)) = (it.next(), it.next().and_then(|v| v.parse::<f64>().ok())) {
            if k.starts_with("acf/") {
                vals.insert(k, v);
            }
        }
    }
    let get = |k: &str| vals.get(k).copied().filter(|v| v.is_finite());
    let n = get("acf/_flap_detents").map(|d| d.round() as usize).filter(|d| *d > 0)?;
    let vs = get("acf/_Vs_kts").filter(|v| *v > 0.0)?;
    let vso = get("acf/_Vso_kts").filter(|v| *v > 0.0 && *v < vs)?;
    let slat_inc = get("acf/_slat1_inc").unwrap_or(0.0);
    let flap: Vec<f64> = (0..=n)
        .map(|i| get(&format!("acf/_flap1_dn/{i}")).unwrap_or(0.0).max(0.0))
        .collect();
    let slat: Vec<f64> = (0..=n)
        .map(|i| get(&format!("acf/_slat1_dn/{i}")).unwrap_or(0.0).clamp(0.0, 1.0))
        .collect();
    if slat_inc <= 0.0 || slat.iter().all(|s| *s <= 0.0) {
        return None;
    }
    let flap_max = flap.iter().copied().fold(0.0, f64::max);
    let s = ACF_LIFT_SLOPE_PER_DEG * slat_inc;
    let fcl = get("acf/_flap1_cl").unwrap_or(0.0).max(0.0);
    let lift = |i: usize| {
        s * slat[i] + if flap_max > 0.0 { fcl * flap[i] / flap_max } else { 0.0 }
    };
    let full = lift(n);
    if full <= 0.0 {
        return None;
    }
    let (a, b) = (1.0 / (vs * vs), 1.0 / (vso * vso));
    Some(
        (1..=n)
            .map(|i| {
                let phi = (lift(i) / full).clamp(0.0, 1.0);
                (a + phi * (b - a)).powf(-0.5).round()
            })
            .collect(),
    )
}

/// Static reference figures recited on request (V-speeds, weights, limits), from
/// the airframe's POH/POM. Every field is optional so a profile fills in only what
/// it knows; a missing figure is simply not offered. Vne and clean-stall live at
/// the top level (the monitor uses them) and aren't duplicated here.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ReferenceProfile {
    #[serde(default)]
    pub kind: Option<String>, // e.g. "single-engine aircraft", "twin-engine jet"
    #[serde(default)]
    pub vno_kts: Option<f64>, // max structural cruise
    #[serde(default)]
    pub va_kts: Option<f64>, // maneuvering
    #[serde(default)]
    pub vx_kts: Option<f64>, // best angle of climb
    #[serde(default)]
    pub vy_kts: Option<f64>, // best rate of climb
    /// The speed the INITIAL climb is flown at, KIAS — what the auto takeoff
    /// protects between liftoff and the handoff, as opposed to the enroute
    /// climb speed `vy_kts` holds afterwards.
    ///
    /// On a trainer they are the same number and this stays `None`. On a jet
    /// they are not: the 737 lifts at V2+20 and climbs out around 170 while
    /// its `vy_kts` of 210 is the speed the pattern and the cruise climb use.
    /// Fed the enroute figure, the takeoff's climb law read 174 kt as
    /// "36 knots slow", stripped 2,150 fpm off the commanded rate and
    /// levelled a 737 at full thrust at 1,030 ft (LFLL, 2026-09-17).
    /// Defaults to `vy_kts`, so every profile that does not set it is
    /// unchanged.
    #[serde(default)]
    pub initial_climb_kts: Option<f64>,
    /// Height AGL where the climb-out stops holding `initial_climb_kts` and
    /// starts accelerating and cleaning up. A jet's declaration: below it the
    /// takeoff climbs as hard as it must (to `best_climb_fpm`) to keep the
    /// speed, and the flaps stay where the takeoff set them. `None` keeps the
    /// trainer's climb-out (see `autotakeoff::ClimbOut`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub acceleration_height_agl_ft: Option<f64>,
    #[serde(default)]
    pub vr_kts: Option<f64>, // rotate
    /// How fast the rotation raises the nose, degrees a second; `None` is
    /// the airline 2.5 (`autotakeoff::ROTATE_RATE_DEG_S`). A light single
    /// rotates faster: the SR22 at 2.5 from its nose-down roll attitude
    /// took 6.2 s from Vr 73 to lift off at 99.5 KIAS (LEMG 12 1790690995,
    /// 2026-09-29), the drift growing 5.9 m in those seconds.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rotate_rate_deg_s: Option<f64>,
    /// Blue line: single-engine best rate of climb. A TWIN's number — the one
    /// speed a multi pilot flies the whole engine-failure drill against — and
    /// meaningless on a single, so it stays `None` there.
    #[serde(default)]
    pub vyse_kts: Option<f64>,
    /// Red line: single-engine minimum control speed. The other half of the
    /// twin pair above.
    #[serde(default)]
    pub vmca_kts: Option<f64>,
    #[serde(default)]
    pub best_glide_kts: Option<f64>,
    #[serde(default)]
    pub vs0_kts: Option<f64>, // stall, full flaps
    #[serde(default)]
    pub approach_kts: Option<f64>,
    #[serde(default)]
    pub max_crosswind_kts: Option<f64>, // demonstrated
    #[serde(default)]
    pub cruise_kts: Option<f64>,
    /// Target INDICATED airspeed to hold in cruise (the autoflight cruise-power
    /// governor chases this). Distinct from `cruise_kts`, which is the spoken POH
    /// figure (usually a TAS). `None` = no automatic cruise-power reduction.
    #[serde(default)]
    pub cruise_ias_kts: Option<f64>,
    /// Optimum cruise altitude at [`Reference::optimum_alt_at_kg`], feet, and
    /// how many feet it rises per 1,000 kg burned off. Jets only: the altitude
    /// that flies best moves UP as the aircraft gets lighter, by roughly a
    /// level an hour on a narrowbody, and nothing in the tree ever looked
    /// (EIDW-BIKF, 2026-09-20: planned FL340 and held it for the whole
    /// crossing, 4,600 ft below optimum by mid-Atlantic). `None` on anything
    /// that does not step-climb, which is every piston here — the copilot then
    /// says nothing rather than inventing a number.
    #[serde(default)]
    pub optimum_alt_ft: Option<f64>,
    #[serde(default)]
    pub optimum_alt_at_kg: Option<f64>,
    #[serde(default)]
    pub optimum_alt_ft_per_1000kg: Option<f64>,
    /// The rates the ENROUTE vertical law climbs and descends at, fpm. Both
    /// default to the GA figure when a profile is silent.
    ///
    /// Distinct from `best_climb_fpm`, which is the takeoff's initial climb.
    /// These pace the follower between fixes, and the descent one also places
    /// the top of descent, so the two stay consistent by construction: the
    /// TOD is put exactly where this rate can make the constraint.
    ///
    /// 700 fpm is a 1.1 degree path at jet speeds. The EIDW-BIKF arrival
    /// (2026-09-20) needed about 2,000 fpm on every leg from BASLU inbound
    /// and the law could offer a third of it — the aircraft would have
    /// reached Keflavík some 17,000 ft high. The same constant capped the
    /// CLIMB, which is why FL340 took 37 minutes instead of 20, all of it
    /// burning low-altitude fuel.
    #[serde(default)]
    pub enroute_climb_fpm: Option<f64>,
    #[serde(default)]
    pub enroute_descent_fpm: Option<f64>,
    #[serde(default)]
    pub best_climb_fpm: Option<f64>,
    #[serde(default)]
    pub max_gross_lb: Option<f64>,
    #[serde(default)]
    pub empty_weight_lb: Option<f64>,
    #[serde(default)]
    pub useful_load_lb: Option<f64>,
    #[serde(default)]
    pub service_ceiling_ft: Option<f64>,
    #[serde(default)]
    pub fuel_capacity_gal: Option<f64>,
    #[serde(default)]
    pub fuel_useable_gal: Option<f64>,
    #[serde(default)]
    pub endurance_hours: Option<f64>,
    #[serde(default)]
    pub takeoff_distance_ft: Option<f64>,
    /// GROUND ROLL, feet — the runway the wheels actually need. The automation
    /// flies this one (the line-up's full-length check, `runway_fit`, full
    /// auto's lined-up test), all of which add their own ×1.4 margin on top.
    ///
    /// Distinct from `takeoff_distance_ft`, which is the figure the copilot
    /// RECITES: on a GA single that is the distance over a 50 ft obstacle, but
    /// on an airliner it is the factored takeoff FIELD LENGTH at max weight,
    /// which already carries its own margins. Multiplying that again demanded
    /// 3179 m for an MD-82 — more than the whole of Luton's runway — so the
    /// line-up backtracked for length it did not need and full auto then judged
    /// a perfectly good line-up a failure and quit, twice (2026-07-28).
    /// `None` falls back to `takeoff_distance_ft`, which is what every GA
    /// profile has always used.
    #[serde(default)]
    pub takeoff_roll_ft: Option<f64>,
    #[serde(default)]
    pub landing_distance_ft: Option<f64>,
    #[serde(default)]
    pub max_rpm: Option<f64>,
    #[serde(default)]
    pub oil_temp_min_f: Option<f64>,
    #[serde(default)]
    pub oil_temp_max_f: Option<f64>,
    #[serde(default)]
    pub oil_press_min_psi: Option<f64>,
    #[serde(default)]
    pub oil_press_max_psi: Option<f64>,
}

/// Taxi pacing, per airframe: the pace on a straight taxiway, and the caps the
/// schedule falls to for an ordinary bend and a sharp one.
///
/// These were one set of numbers for every aircraft (10/6/4), tuned on the 172,
/// where 10 knots is a THROTTLE setting. On a jet it is a brake setting: an
/// MD-82 at idle thrust rolls past 10 knots on flat pavement on its own, so the
/// governor rides the brakes the whole taxi, and airline practice is nearer
/// 15-20 knots down a long taxiway with 10 through the turns. Anything omitted
/// falls back to the GA figures, so an airframe that says nothing taxis exactly
/// as it did before this existed.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TaxiProfile {
    /// Pace on a straight leg, knots — the target when the pilot names none.
    #[serde(default)]
    pub kts: Option<f64>,
    /// Cap through an ordinary bend (>= 30 degrees).
    #[serde(default)]
    pub turn_kts: Option<f64>,
    /// Cap through a sharp corner (>= 100 degrees) and a turn-around join.
    #[serde(default)]
    pub sharp_kts: Option<f64>,
    /// Heading error (degrees) at which the steering commands FULL deflection.
    /// Bigger = gentler. `None` keeps the shared default, which was tuned on the
    /// 172.
    #[serde(default)]
    pub steer_full_deg: Option<f64>,
    /// Seconds of yaw-rate LEAD mixed into the steering error — the damping
    /// term. `None` keeps the shared default.
    ///
    /// This is the knob a CASTORING nosewheel needs, and the reason it is
    /// per-airframe at all. A linked nosewheel (172, MD-82) is self-damped and
    /// its taxi yaw rates are small, so the term is nearly inert there; a
    /// castoring one steers through the rudder and differential braking, which
    /// is a laggier plant that the same gain drives into a weave. Measured over
    /// the blackbox at 4-12 kt, yaw response per unit of steer runs 44 deg/s on
    /// the Twin Otter and 12.8 on the 172, but only 6.9 on the SF50 and 2.3 on
    /// the SR22 — the two castoring airframes are not weaker loops by accident,
    /// they are laggier ones, and lag is fixed with phase lead, not gain.
    #[serde(default)]
    pub steer_lead_s: Option<f64>,
    /// Cap on how fast the steering command may travel, per SECOND. `None` = no
    /// limit, which is every airframe that has never needed one.
    ///
    /// Deliberately generous where it is set: a slew limit is phase lag, and a
    /// TIGHT one causes the very oscillation it looks like it should cure (the
    /// auto-takeoff roll's 0.25-per-tick limiter swept only 0.5/s and produced a
    /// textbook slew-induced limit cycle, 2026-07-21). Set it above the rate the
    /// airframe actually uses so it only ever clips the violent transient.
    #[serde(default)]
    pub steer_slew_per_s: Option<f64>,
    /// Pure-pursuit aim time for the taxi steering, seconds. `None` keeps the
    /// shared 5 s. A slow-turning plant needs a LONG aim — see the autotaxi
    /// lookahead law; the SF50 carries 9.
    #[serde(default)]
    pub lookahead_s: Option<f64>,
    /// Steer with DIFFERENTIAL BRAKES as well as the yoke-heading channel.
    ///
    /// For airframes where that channel has nothing behind it at taxi speed:
    /// a castoring nosewheel takes no steering input, and on a JET there is no
    /// prop wash over the rudder either, so between walking pace and
    /// rudder-effective speed the aircraft simply does not answer. The SF50 is
    /// the proving case: thirty seconds of full-scale steer command while the
    /// cross-track error GREW to 13.8 m and the aircraft taxied down the grass
    /// beside taxiway Alpha (Luton, 2026-07-28). Real castoring-wheel aircraft
    /// are steered exactly this way — a touch of inside brake.
    #[serde(default)]
    pub brake_steer: bool,
    /// The taxi hold's speed control below idle: the deepest share of the
    /// BETA range (0..1, of X-Plane's -1..0) the lever may go, before any
    /// brake is used on a rolling taxi. For a turboprop whose idle is a
    /// taxi too fast: "Don't ride the brakes in taxi. Ease the throttle into
    /// beta instead" (Lancair Evolution POH). `None`: the brakes, as before.
    /// A hypothesis until a blackboxed taxi measures it.
    #[serde(default)]
    pub beta_max: Option<f64>,
    /// The yaw rate a taxi corner may ask of this airframe, degrees per
    /// second: a corner of radius R is taken no faster than this times R.
    /// `None` keeps the angle-only bend caps. The 737 turns at about 20 deg/s
    /// at full lock under 7 kt (LEMG and LIBP, 2026-09-26), and entered an
    /// 80-degree corner drawn at a 9 m radius at 11.5 kt, where the route asked
    /// 38 deg/s: it swung 5.8 m wide.
    #[serde(default)]
    pub max_yaw_dps: Option<f64>,
    /// A full-length backtrack's pace down the strip, knots. `None` keeps the
    /// straight pace floored at 14 (a GA backtrack at 6 kt timed full auto
    /// out at LXGB). The Evolution's 14 ran it to 16 kt on LILB's 15 m strip,
    /// 3.6 m off, and 9.5 m past its stop (2026-09-30, #196).
    #[serde(default)]
    pub backtrack_kts: Option<f64>,
}

/// A switch whose only control is a TOGGLE command, paired with the dataref
/// that reads its state.
///
/// Firing a toggle as "on" turns the switch OFF whenever it was already on, so
/// a toggle can only be driven safely by reading first and acting on the
/// difference. Whole airframes are built this way: every light and the avionics
/// master on the Citation X are toggle-only, and its xlua datarefs are
/// READ-ONLY over the Web API (`PATCH … → 403 dataref_is_readonly`, verified
/// live 2026-07-28), so commands are not merely preferred there — they are the
/// only way to actuate anything at all.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToggleSwitch {
    /// The toggle command to fire when the state doesn't match.
    pub command: String,
    /// The 0/1 dataref that reads this switch's actual position.
    pub dataref: String,
}

/// One switchable light.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LightProfile {
    /// Canonical token the LLM selects by (e.g. `"landing"`).
    pub key: String,
    /// Spoken form for the readback (e.g. `"landing light"`).
    pub spoken: String,
    /// The 0/1 switch dataref — read back to confirm the actual state.
    pub dataref: String,
    /// Commands that drive the panel switch on/off. Preferred over a raw
    /// `dataref` write: a write can set the value without animating the 3D
    /// switch (as the avionics master did). When present the tool fires these;
    /// `dataref` is still used for the readback. Falls back to a write if unset.
    #[serde(default)]
    pub command_on: Option<String>,
    #[serde(default)]
    pub command_off: Option<String>,
    /// Further switch datarefs driven to the SAME state as `dataref`. One
    /// "light" on the panel is sometimes two switches: the Citation X has
    /// independent left and right landing lights, and switching only one leaves
    /// the aircraft half lit while the readback happily confirms the half it
    /// looked at. Written, never commanded — the extras exist precisely because
    /// the airframe has no single command for the pair.
    #[serde(default)]
    pub also_datarefs: Vec<String>,
    /// Toggle-only switches making up this light. When present these are
    /// driven INSTEAD of the command/dataref above: each is read and toggled
    /// only if it doesn't already match. One "light" can be several — the
    /// Citation X's landing lights are an independent left and right.
    #[serde(default)]
    pub toggles: Vec<ToggleSwitch>,
    /// The copilot switches this light on by its own hand once the aircraft is
    /// powered and the sun is down, and off again at daybreak (see
    /// [`crate::cockpit_auto`]). For the cockpit lighting a crew member would
    /// simply reach up and set — the Baron's roof floods — never for an
    /// EXTERIOR light, where switching on the pilot's behalf changes what other
    /// traffic sees.
    ///
    /// One-shot per spell, never re-asserted: the pilot switching it back off
    /// is the pilot's decision and the copilot does not argue. Off is only ever
    /// commanded for a light the copilot itself turned on.
    #[serde(default)]
    pub auto_at_night: bool,
}

/// One electrical system the `set_electrical` tool can switch. Driven by the
/// profile so airframes with different wiring (the 172's single battery + split
/// master; the SR22's dual battery/alternator) need no code — just config.
/// Switching fires `on_commands`/`off_commands` (commands animate the 3D switch;
/// a raw dataref write may not); `readback` datarefs (any may be `name[i]`) are
/// read after to confirm — all must match the target, else the mixed state is
/// reported.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ElectricalSystem {
    /// Token the LLM selects by (e.g. `"battery"`, `"battery1"`, `"avionics"`).
    pub key: String,
    /// Spoken form for the readback (e.g. `"battery one"`, `"both batteries"`).
    pub spoken: String,
    pub on_commands: Vec<String>,
    pub off_commands: Vec<String>,
    /// Switch datarefs written to 1/0 directly, for systems the sim lets us
    /// write but that have no on/off command. Fired after any commands above.
    /// NOT a way to drive an xlua add-on: those datarefs are read-only over the
    /// Web API (403), which is what `toggles` exists for.
    #[serde(default)]
    pub write_datarefs: Vec<String>,
    /// Toggle-only switches in this system, each read first and toggled only if
    /// it doesn't already match the target (see [`ToggleSwitch`]). The Citation
    /// X's avionics master is toggle-only AND its dataref is unwritable, so
    /// this is the only way to switch it.
    #[serde(default)]
    pub toggles: Vec<ToggleSwitch>,
    /// Guard covers over this system's switch, each a TOGGLE command and the
    /// dataref that reads its position (0 closed, 1 open). A guarded switch
    /// is held ON by its closed guard: the stock 737's xlua forces
    /// `battery_on[0]` to 1 every frame while `guard_cover_pos[0]` is under
    /// 0.5, so `battery_1_off` with the guard shut is undone the next frame
    /// and the mission's shutdown said "The battery didn't move" (17:07Z,
    /// 2026-09-26). Switching OFF opens each guard first and waits for it to
    /// read open; switching ON closes it after the switch, as a crew does.
    #[serde(default)]
    pub guards: Vec<ToggleSwitch>,
    #[serde(default)]
    pub readback: Vec<String>,
}

/// The add-on's own transponder mode SELECTOR, for airframes where the stock
/// `transponder_mode` actuator is downstream of the knob rather than upstream
/// of the box.
///
/// The RWDesigns Twin Otter is the case that found this: its KT73 is a SASL
/// instrument, and `master.lua`'s `KT73()` runs every frame setting
/// `sim/cockpit/radios/transponder_mode` FROM `TO/radios/KT73SelKnob`. Writing
/// the stock actuator therefore does nothing that survives a frame — the
/// cockpit knob never moves and the mode snaps back — so the copilot's
/// standby/altitude switching was silently a no-op on that airframe
/// (2026-07-27). Same shape as [`AircraftProfile::fuel_pump_commands`]: the
/// universal write is not wrong, it is simply not connected to anything here.
///
/// The values are the KNOB's positions, which need not match the stock
/// encoding at all (the Twotter's ALT is knob 5 and drives stock mode 2, not
/// 3), which is exactly why the readback re-reads the KNOB.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransponderModeKnob {
    /// The add-on's selector dataref ("TO/radios/KT73SelKnob").
    pub dataref: String,
    pub off: i64,
    pub standby: i64,
    pub on: i64,
    pub altitude: i64,
    /// Commands that turn the knob one position up / down, for a knob whose
    /// dataref is READ-ONLY. The A330's `auto_on_knob_pos` is: probed live it
    /// reports `is_writable: false`, so every mode write on that airframe was
    /// a silent no-op and the flight went out in standby to "negative radar
    /// contact, recycle transponder" (EGPH, 2026-09-20). With both declared
    /// the knob is STEPPED, read before each step, and the dataref is only
    /// ever read.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub up_command: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub down_command: Option<String>,
    /// Other knobs the box needs before ON or ALTITUDE transmits anything.
    /// The A330 has three: with the mode knob in AUTO and the TCAS knob left
    /// in STBY its script writes stock mode 1, so the takeoff blackboxed
    /// `transponder_alt ok: true` from the one knob it looked at and the
    /// flight squawked standby round Glasgow's zone (2026-09-21). Turned
    /// after the mode knob, never for OFF or STANDBY, and the answer is then
    /// the STOCK mode, which the add-on computes from all of them.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub also: Vec<CompanionKnob>,
}

/// Whether the stock load sheet writes (`weight/m_fuel`, `weight/m_fixed`)
/// are known to take on an airframe (see [`AircraftProfile::load_sheet`]).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LoadSheet {
    /// Stock tanks and payload: written and read back (the 737, probed live
    /// on the EGPH ramp 2026-09-21; the 172).
    #[default]
    Stock,
    /// Never written and read back on this airframe.
    Unprobed,
    /// The aircraft's scripts own the tanks: a stock write is undone.
    OwnFuelSystem,
}

/// An add-on's own seat belt sign switch: a read-only position stepped by
/// its commands. `off` is where the crew leaves it in cruise (AUTO on an
/// Airbus: the signs come back with the flaps or the gear by themselves).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SeatBeltSigns {
    pub position: String,
    pub on: i64,
    pub off: i64,
    pub up_command: String,
    pub down_command: String,
}

/// The configuration items an airliner's crew owns and a trainer has not got.
/// Stock datarefs on purpose (`speedbrake_ratio`, `auto_brake_level`): the
/// profile declares THAT the airframe has them, never a name.
/// The takeoff power the MANUAL allows, as engine torque, until a speed:
/// the Lancair Evolution takes off at 1,250 ft-lb of its 2,230 "since more
/// power would over-power the rudders' and ailerons' ability to counter the
/// engine torque" (N844X POH), more only as the speed builds past 150 kt
/// (POM). The takeoff put in full power 2.5 s into the roll and the aircraft
/// left LILB 03's centreline by 129 m with the steering pinned against it
/// (2026-09-25). Held on the MEASURED torque (`ENGN_TRQ`), never a guessed
/// lever position.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TakeoffPowerProfile {
    pub torque_ftlb: f64,
    pub until_kias: f64,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TakeoffConfigProfile {
    /// Ground spoilers armed for landing, and for takeoff unless
    /// `takeoff_speedbrake_down`. Also declares the speedbrake lever itself.
    #[serde(default)]
    pub arm_spoilers: bool,
    /// Boeing practice: the lever sits in the DOWN detent for takeoff (armed
    /// is a takeoff configuration warning on the 737, and its own Before
    /// Takeoff list says so); a reject raises it by hand. Armed only at the
    /// gear. An Airbus leaves this false and arms for the roll.
    #[serde(default)]
    pub takeoff_speedbrake_down: bool,
    /// Autobrake to RTO / MAX before the roll (stock level 0).
    #[serde(default)]
    pub autobrake_rto: bool,
    /// The stock `auto_brake_level` autoland selects at the gear. The A330's
    /// script lights LO at 3 and MED at 4. `None` leaves it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub landing_autobrake_level: Option<i64>,
    /// Flight directors ON before the roll (stock `flight_director_mode`,
    /// raised by `sim/autopilot/fdir_on`, read back).
    #[serde(default)]
    pub flight_directors: bool,
    /// Autothrottle ARMED and not engaged before the roll (stock
    /// `autothrottle_enabled` 0: armed by `autothrottle_arm` from hard off,
    /// an engaged mode taken out by `autothrottle_off`, because the takeoff
    /// owns the thrust levers). An airframe whose script arms it from the
    /// lever detents (the A330) leaves this false.
    #[serde(default)]
    pub autothrottle_armed: bool,
    /// V2, KIAS, set in the MCP speed window before the roll (stock
    /// `airspeed_dial_kts_mach`, read back). The reference card's figure.
    /// `None` leaves the window.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub v2_kts: Option<f64>,
    /// Thrust reduction: above this height AGL the takeoff pulls the levers
    /// back to `climb_thrust_ratio`. Both or neither; `None` keeps takeoff
    /// thrust until whoever takes the aircraft next sets it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub thrust_reduction_agl_ft: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub climb_thrust_ratio: Option<f64>,
}

impl TakeoffConfigProfile {
    pub const SPEEDBRAKE: &'static str = "sim/cockpit2/controls/speedbrake_ratio";
    /// 0 RTO, 1 off, 2..5 the landing levels.
    pub const AUTOBRAKE: &'static str = "sim/cockpit2/switches/auto_brake_level";
    /// 0 off, 1 on, 2 on with the servos.
    pub const FD_MODE: &'static str = "sim/cockpit2/autopilot/flight_director_mode";
    pub const CMD_FD_ON: &'static str = "sim/autopilot/fdir_on";
    /// -1 hard off, 0 ARMED (servos declutched; the PFD's "ARM"), 1 speed,
    /// 2 N1, 3 retard.
    pub const AT_MODE: &'static str = "sim/cockpit2/autopilot/autothrottle_enabled";
    pub const CMD_AT_ARM: &'static str = "sim/autopilot/autothrottle_arm";
    /// "All modes off, remain armed."
    pub const CMD_AT_OFF: &'static str = "sim/autopilot/autothrottle_off";
    /// The MCP speed window (the 737's glareshield script reads this one).
    pub const SPEED_WINDOW: &'static str = "sim/cockpit2/autopilot/airspeed_dial_kts_mach";

    /// The stock names the declarations rely on, for the profile probe
    /// ([`profile_references`]): the profile says THAT the airframe has an
    /// item, the names live here and in `takeoff_config`, which uses these.
    pub fn references(&self) -> Vec<&'static str> {
        let mut out = Vec::new();
        if self.arm_spoilers || self.takeoff_speedbrake_down {
            out.push(Self::SPEEDBRAKE);
        }
        if self.autobrake_rto || self.landing_autobrake_level.is_some() {
            out.push(Self::AUTOBRAKE);
        }
        if self.flight_directors {
            out.extend([Self::FD_MODE, Self::CMD_FD_ON]);
        }
        if self.autothrottle_armed {
            out.extend([Self::AT_MODE, Self::CMD_AT_ARM, Self::CMD_AT_OFF]);
        }
        if self.v2_kts.is_some() {
            out.push(Self::SPEED_WINDOW);
        }
        out
    }
}

/// One more knob a transponder needs set (see [`TransponderModeKnob::also`]):
/// read-only, stepped by its commands to `want`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CompanionKnob {
    pub dataref: String,
    pub want: i64,
    pub up_command: String,
    pub down_command: String,
}

/// Custom on/off commands for the electric boost pumps, for airframes where the
/// universal `fuel_pump_on` array is a no-op (see
/// [`AircraftProfile::fuel_pump_commands`]). All `on_commands` fire for "pump
/// on" and all `off_commands` for "pump off" — a multi-pump aircraft (the Twin
/// Otter's FWD + AFT) switches them together, matching how the checklist says
/// "boost pumps ON". `readback` datarefs (each may be `name[i]`) confirm.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PumpCommands {
    pub on_commands: Vec<String>,
    pub off_commands: Vec<String>,
    /// Switch datarefs written to 1/0 directly, for systems whose only control
    /// is a TOGGLE command. Firing a toggle as "on" turns the system off
    /// whenever it was already on, which is worse than having no tool at all —
    /// the Citation X's avionics master is the case (`cmd_avionics_toggle` is
    /// the only command its scripts create). Written after any commands above.
    #[serde(default)]
    pub write_datarefs: Vec<String>,
    #[serde(default)]
    pub readback: Vec<String>,
}

/// One entry of the checklist action map: a dataref *substring* and the curated
/// action it drives. When the interpreter ([`crate::checklist`]) meets a file
/// item whose dataref contains `match_`, it runs that action — so *which*
/// dataref maps to *which* system is per-airframe config here, while *how* a
/// condition becomes the tool's argument stays shared logic in Rust. First match
/// wins, so order more specific entries first.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChecklistMapEntry {
    /// Substring matched against the file's dataref (e.g. `"parkbrake"`,
    /// `"battery_array_on[0]"`).
    #[serde(rename = "match")]
    pub match_: String,
    /// The action this dataref drives.
    pub action: ChecklistAction,
}

/// A curated action a checklist item can drive. The variant names *what* system;
/// the condition supplies the value (handled in [`crate::checklist`]).
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ChecklistAction {
    ParkingBrake,
    PitotHeat,
    FuelPump,
    AutopilotOff,
    Transponder,
    FuelSelector,
    Flaps,
    Mixture,
    Throttle,
    /// Ignition key: start (crank) or magneto position, from the value.
    Ignition,
    PitchTrim,
    /// Electrical system by name (`battery`, `alternator`, `avionics`, `master`).
    Electrical(String),
    /// A switchable light by its profile key (`nav`, `landing`, …).
    Light(String),
}

/// Load a profile by id: prefer `profiles/<id>.yaml`, else the built-in.
pub fn load(id: &str) -> Option<AircraftProfile> {
    read_profile(id, true)
}

/// [`load`], with or without the "loaded from YAML" log line (the matcher
/// sweep reads every profile and would print one per file).
fn read_profile(id: &str, log: bool) -> Option<AircraftProfile> {
    let path = format!("profiles/{id}.yaml");
    match std::fs::read_to_string(&path) {
        Ok(text) => match serde_yaml::from_str::<AircraftProfile>(&text) {
            Ok(profile) => {
                if log {
                    tracing::info!(path = %path, "loaded aircraft profile from YAML");
                }
                Some(profile)
            }
            Err(e) => {
                tracing::error!("failed to parse {path}: {e:#}; falling back to built-in");
                builtin(id)
            }
        },
        Err(_) => builtin(id), // no YAML file — use the built-in definition
    }
}

/// What a profile says it is FOR (the `match:` block). Every key given must
/// hit; `icao` is required, and a profile with nothing else is a type-wide
/// catch-all. Each list hits when ANY of its entries does, compared
/// case-insensitively, the path with backslashes as slashes.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ProfileMatch {
    /// `sim/aircraft/view/acf_ICAO` values.
    #[serde(default)]
    pub icao: Vec<String>,
    /// Substrings of `sim/aircraft/view/acf_relative_path` (the model's
    /// folder and file, relative to the install).
    #[serde(default)]
    pub acf: Vec<String>,
    /// Substrings of `sim/aircraft/view/acf_author`.
    #[serde(default)]
    pub author: Vec<String>,
    /// Substrings of `sim/aircraft/view/acf_descrip`.
    #[serde(default)]
    pub description: Vec<String>,
}

impl ProfileMatch {
    /// Does this profile claim the ICAO type at all?
    pub fn claims_type(&self, icao: &str) -> bool {
        let icao = icao.trim();
        !icao.is_empty()
            && self
                .icao
                .iter()
                .any(|t| t.trim().eq_ignore_ascii_case(icao))
    }
}

/// Every profile's `match:` block, as `(id, matcher)` sorted by id: the
/// YAMLs in `profiles/` (read the way [`load`] reads them), and a built-in
/// whose YAML is missing. A profile without a block is left out.
pub fn profile_matchers() -> Vec<(String, ProfileMatch)> {
    let mut ids: Vec<String> = std::fs::read_dir("profiles")
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|e| {
            let path = e.path();
            (path.extension().is_some_and(|x| x == "yaml"))
                .then(|| path.file_stem()?.to_str().map(str::to_string))?
        })
        .collect();
    for b in ["cessna_172sp", "cirrus_sr22"] {
        if !ids.iter().any(|i| i == b) {
            ids.push(b.to_string());
        }
    }
    ids.sort();
    ids.into_iter()
        .filter_map(|id| {
            let p = read_profile(&id, false)?;
            Some((p.id, p.matcher?))
        })
        .collect()
}

/// The profile for a TYPE, not a model: a flight plan's ICAO (SimBrief
/// `aircraft/icaocode`) names no model, so any profile whose `match.icao`
/// holds it answers (the first by id). For type-level questions only (jet
/// or helicopter, from a plan); the loaded aircraft is resolved by
/// `detect::resolve`, never by this.
pub fn profile_for_type(icao: &str) -> Option<AircraftProfile> {
    profile_matchers()
        .into_iter()
        .find(|(_, m)| m.claims_type(icao))
        .and_then(|(id, _)| read_profile(&id, true))
}

/// The universal thrust-lever dataref: what every airframe used until one
/// didn't.
pub const STOCK_THROTTLE: &str = "sim/cockpit2/engine/actuators/throttle_ratio_all";

/// The thrust lever THIS airframe actually obeys, republished on every profile
/// swap (including back to `None`) — a stale override would aim the next
/// aircraft's power at a dataref it doesn't use, the same shape as the
/// transponder knob and the radio-power latch.
static THROTTLE_DREF: std::sync::Mutex<Option<String>> = std::sync::Mutex::new(None);

/// Publish the airframe's thrust-lever dataref. Called on EVERY profile swap.
pub fn publish_throttle_dataref(dref: Option<String>) {
    *THROTTLE_DREF.lock().unwrap_or_else(|e| e.into_inner()) = dref;
}

/// X-Plane's landing-gear warning, as Plane Maker models it: with the gear
/// not down and the speed below `below_kias`, the horn sounds when the
/// throttle is under `below_throttle_pct` or the flaps are past
/// `above_flap_ratio`. The `.acf` keys are `acf/_gear_warn_below_kias`,
/// `acf/_gear_warn_below_thro_pct`, `acf/_gear_warn_above_flap_rat`.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize, Serialize)]
pub struct GearWarning {
    pub below_kias: f64,
    #[serde(default)]
    pub below_throttle_pct: Option<f64>,
    #[serde(default)]
    pub above_flap_ratio: Option<f64>,
}

impl GearWarning {
    /// Would the warning sound with the gear up in this state? A flap AT
    /// the threshold counts (a handle written to 0.5 reads back a whisker
    /// either side of it). Pure.
    pub fn would_sound(&self, ias_kts: f64, flap_ratio: f64, throttle_ratio: f64) -> bool {
        if ias_kts >= self.below_kias {
            return false;
        }
        let flaps = self
            .above_flap_ratio
            .is_some_and(|f| flap_ratio >= f - 0.005);
        let power = self
            .below_throttle_pct
            .is_some_and(|t| throttle_ratio * 100.0 < t);
        flaps || power
    }
}

/// The live airframe's nose gear CASTORS (its `.acf`): it turns on the
/// ground by differential brake only, whatever the profile declares. Read
/// at detection, reset per swap (`flight_state::publish_airframe`). The
/// Evolution's profile never said `brake_steer`, and full rudder at 6 kt
/// turned it 3.7 degrees a second, off LILB's route three times
/// (2026-09-25); the RV-10's never did either.
static NOSE_CASTORS: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

pub fn publish_nose_castors(on: bool) {
    NOSE_CASTORS.store(on, std::sync::atomic::Ordering::Relaxed);
}

pub fn nose_castors() -> bool {
    NOSE_CASTORS.load(std::sync::atomic::Ordering::Relaxed)
}

/// The castoring nose has NO steering of its own either (`_steerdeg_lospeed`
/// 0: the Evolution, the SR22, the RV-10; the SF50's castors with 40 degrees
/// of it): at walking pace the brakes are the only thing that turns it.
static NOSE_FREE: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

pub fn publish_nose_free(on: bool) {
    NOSE_FREE.store(on, std::sync::atomic::Ordering::Relaxed);
}

pub fn nose_free() -> bool {
    NOSE_FREE.load(std::sync::atomic::Ordering::Relaxed)
}

/// The live airframe's taxi beta depth (`taxi.beta_max`), when it taxis in
/// beta.
pub fn taxi_beta_max() -> Option<f64> {
    live_profile()
        .and_then(|p| p.taxi.as_ref().and_then(|t| t.beta_max))
        .filter(|b| *b > 0.0)
}

/// Whether an `.acf`'s nose castors with no steering angle at all. Pure.
pub fn nose_free_from_acf(acf_text: &str) -> bool {
    let steer_lo = acf_text.lines().find_map(|l| {
        let mut it = l.split_whitespace();
        (it.next() == Some("P") && it.next() == Some("_gear/0/_steerdeg_lospeed"))
            .then(|| it.next().and_then(|v| v.parse::<f64>().ok()))
            .flatten()
    });
    nose_castors_from_acf(acf_text) && steer_lo.is_none_or(|d| d.abs() < 1.0)
}

/// Whether an `.acf`'s first gear (the nose of a tricycle) castors. Pure.
pub fn nose_castors_from_acf(acf_text: &str) -> bool {
    acf_text.lines().any(|l| {
        let mut it = l.split_whitespace();
        it.next() == Some("P")
            && it.next() == Some("_gear/0/_gear_castors")
            && it.next().and_then(|v| v.parse::<f64>().ok()).is_some_and(|v| v > 0.5)
    })
}

/// The gear-warning envelope from an `.acf`'s text, or `None` when it
/// declares no warning speed. Pure.
pub fn gear_warning_from_acf(acf_text: &str) -> Option<GearWarning> {
    let mut kias = None;
    let mut thro = None;
    let mut flap = None;
    for l in acf_text.lines() {
        let mut it = l.split_whitespace();
        if it.next() != Some("P") {
            continue;
        }
        let (Some(k), Some(v)) = (it.next(), it.next()) else {
            continue;
        };
        let v = v.parse::<f64>().ok();
        match k {
            "acf/_gear_warn_below_kias" => kias = v,
            "acf/_gear_warn_below_thro_pct" => thro = v,
            "acf/_gear_warn_above_flap_rat" => flap = v,
            _ => {}
        }
    }
    let below_kias = kias.filter(|k| *k > 0.0)?;
    Some(GearWarning {
        below_kias,
        below_throttle_pct: thro.filter(|t| *t > 0.0),
        above_flap_ratio: flap.filter(|f| *f > 0.0 && *f <= 1.0),
    })
}

/// The configuration placards autoland extends the gear and flaps against:
/// VLO/VLE, and each flap detent's `(ratio, Vfe)`. Republished on every
/// profile swap like the thrust lever, so a fast handoff on the next airframe
/// is judged by ITS limits. Empty/`None` = the profile is silent, no gate.
static GEAR_LIMIT_KTS: std::sync::Mutex<Option<f64>> = std::sync::Mutex::new(None);
static FLAP_LIMITS: std::sync::Mutex<Vec<(f64, Option<f64>)>> = std::sync::Mutex::new(Vec::new());

/// Publish the airframe's gear and flap speed limits. Called on EVERY profile swap.
pub fn publish_config_limits(vlo_kts: Option<f64>, flaps: Option<&FlapProfile>) {
    *GEAR_LIMIT_KTS.lock().unwrap_or_else(|e| e.into_inner()) = vlo_kts;
    *FLAP_LIMITS.lock().unwrap_or_else(|e| e.into_inner()) = flaps
        .map(|f| f.detents.iter().map(|d| (d.ratio, d.vfe_kts)).collect())
        .unwrap_or_default();
}

/// The live airframe's nominal bus voltage, published per swap like the
/// other selectors. `None` = the 24 volt default.
static BUS_VOLTS_NOMINAL: std::sync::Mutex<Option<f64>> = std::sync::Mutex::new(None);

/// Publish the airframe's nominal bus voltage. Called on EVERY profile swap.
pub fn publish_bus_volts(nominal: Option<f64>) {
    *BUS_VOLTS_NOMINAL.lock().unwrap_or_else(|e| e.into_inner()) = nominal;
}

/// Below this bus voltage the battery is too flat to reliably crank: 23 on a
/// 24 volt system (a healthy pre-start bus sits ~24-25, the low 22s after
/// accessories with the engine off), and the same 4% margin under whatever
/// the live airframe's profile says its system is.
pub fn min_start_volts() -> f64 {
    let nominal = BUS_VOLTS_NOMINAL
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .unwrap_or(24.0);
    nominal * 23.0 / 24.0
}

/// The airspeed FLOOR under every automation's target, from the live profile's
/// clean stall speed: no governor, autoland or follower may ask for less than
/// [`SPEED_FLOOR_RATIO`] times Vs, whatever number it was handed. The RV-10
/// was flown to 63 kt by an autoland whose approach speed was a Cessna's 65,
/// its own clean stall (2026-09-11): the target was wrong and nothing below
/// it asked whether it could be. `None` = the profile declares no Vs.
static SPEED_FLOOR_KTS: std::sync::Mutex<Option<f64>> = std::sync::Mutex::new(None);
/// 1.2 Vs: below the 1.3 an approach is flown at, above anything a wing can
/// hold level with a bank on. A target under this is a wrong number.
pub const SPEED_FLOOR_RATIO: f64 = 1.2;

/// Publish the airframe's speed floor from its clean stall speed. Called on
/// EVERY profile swap.
pub fn publish_speed_floor(vs_clean_kts: Option<f64>) {
    *SPEED_FLOOR_KTS.lock().unwrap_or_else(|e| e.into_inner()) = vs_clean_kts
        .filter(|v| *v > 0.0)
        .map(|v| (v * SPEED_FLOOR_RATIO).round());
}

/// The live airframe's speed floor, kts, if its profile declares a clean stall.
pub fn speed_floor_kts() -> Option<f64> {
    *SPEED_FLOOR_KTS.lock().unwrap_or_else(|e| e.into_inner())
}

// The "too slow" ratios over the clean stall, in one place so a change to
// one is made against the others (refactor audit 2026-10-04, finding #13:
// the climb limiter's floor was moved at 1.15 by the #169 fix and sat under
// the 1.2 automation floor with the 1.13 caution between them). Their order
// is pinned by `slow_ratios_are_ordered`.

/// The monitor's stall WARNING, x the stall in the configuration flown.
pub const STALL_WARNING_RATIO: f64 = 1.10;

/// The monitor's low-speed caution with a PILOT flying, x the stall in the
/// configuration flown: the band a hand-flown approach should stay over.
pub const CAUTION_HAND_RATIO: f64 = 1.20;

/// The monitor's low-speed caution with an automation flying, x the stall in
/// the configuration flown:
/// under [`SPEED_FLOOR_RATIO`], so an automation holding its floor in a
/// gust is not called slow, and well over the stall.
pub const CAUTION_AUTO_RATIO: f64 = 1.13;

/// The climb limiter's floor, x the stall in the configuration flown
/// (`circuit::climb_floor_kts`): the speed a climb bleeds to before the
/// limiter has taken all of it away. It IS the automation floor: it was
/// 1.15, under [`SPEED_FLOOR_RATIO`], so a climb could bleed to a speed no
/// automation may target (CLAUDE.md "No automation target below the speed
/// floor"; refactor audit 2026-10-04, finding #13).
pub const CLIMB_FLOOR_RATIO: f64 = SPEED_FLOOR_RATIO;

/// The least pattern speed, x clean Vs (`circuit::PATTERN_MIN_STALL_MULT`),
/// and the speed an upset recovery holds ([`recovery_hold_kts`]).
pub const PATTERN_MIN_RATIO: f64 = 1.3;

/// The approach speed an airframe that declares none flies, x Vso.
pub const APPROACH_VSO_RATIO: f64 = 1.3;

/// The same, x clean Vs: the automation floor, so the fallback approach is
/// never under it.
pub const APPROACH_VS_RATIO: f64 = SPEED_FLOOR_RATIO;

/// The approach speed an airframe that declares none flies: max(1.3 Vso,
/// 1.2 Vs). ONE rule for the tool registry (autoland), `generic_from_acf`,
/// the instructor's brief and scorer, and the tests that build airframes
/// (refactor audit 2026-10-04, finding #2: the instructor still briefed 1.3
/// x the CLEAN stall, the rule the registry dropped after the SR22 floated
/// and bounced at 90 kt, LEMG 13 2026-09-29).
///
/// No flat 70 kt floor, deliberately. The registry had one and
/// `generic_from_acf` did not, so the two paths disagreed; every shipped
/// profile declares `approach_kts` and a generic airframe always carried the
/// unfloored figure, so the floor only ever reached an airframe with a
/// declared stall and no approach speed, and there it is the SR22 float by
/// another door (a 40 kt-stall wing flown at 70). The 1.2 Vs term already
/// keeps it over the automation floor. Pure.
pub fn fallback_approach_kts(vs0_kts: Option<f64>, vs_clean_kts: f64) -> f64 {
    let by_vso = vs0_kts
        .filter(|v| v.is_finite() && *v > 0.0)
        .map_or(0.0, |v| v * APPROACH_VSO_RATIO);
    by_vso.max(vs_clean_kts * APPROACH_VS_RATIO).round()
}

/// The profile's approach speed, else [`fallback_approach_kts`] from its
/// stalls; `None` when it declares neither an approach speed nor a clean
/// stall.
pub fn approach_kts(p: &AircraftProfile) -> Option<f64> {
    let r = p.reference.as_ref();
    r.and_then(|r| r.approach_kts).or_else(|| {
        p.vs_clean_kts
            .filter(|v| v.is_finite() && *v > 0.0)
            .map(|vs| fallback_approach_kts(r.and_then(|r| r.vs0_kts), vs))
    })
}

/// Rotation speed for an airframe that declares none, x clean Vs.
///
/// 1.1, not the registry's old 1.15 (min 55). The shipped profiles that
/// declare a Vr sit at 0.85 to 1.15 of their clean stall (SR22 1.04, 737
/// 1.04, SF50 1.13, 172 1.15, A330 0.85): takeoff flap lowers the stall, so
/// 1.1 x the clean figure is still over 1.2 x the takeoff stall on every one
/// of them, and 1.15 rotated late and long. A generic airframe already
/// rotated at 1.1 (`generic_from_acf`, whose figure the registry reads);
/// now an airframe with a profile and no Vr agrees with it (refactor audit
/// 2026-10-04, finding #2).
pub const VR_STALL_RATIO: f64 = 1.1;

/// [`VR_STALL_RATIO`] x the clean stall, rounded. Pure.
pub fn fallback_vr_kts(vs_clean_kts: f64) -> f64 {
    (vs_clean_kts * VR_STALL_RATIO).round()
}

/// The speed an upset recovery holds: [`PATTERN_MIN_RATIO`] x the clean
/// stall, never under 70 kt. The floor stays here though not on the
/// approach: a recovery wants margin over an unknown or optimistic stall
/// and has no runway to float down, and slow is the one way it can still
/// go wrong. Pure.
pub fn recovery_hold_kts(vs_clean_kts: f64) -> f64 {
    (vs_clean_kts * PATTERN_MIN_RATIO).max(RECOVERY_HOLD_MIN_KTS)
}

/// The least speed [`recovery_hold_kts`] asks for.
pub const RECOVERY_HOLD_MIN_KTS: f64 = 70.0;

/// The highest speed the flight governor may be asked to hold, from the
/// airframe's Vne/Vmo. `None` = the profile declares no Vne.
///
/// The governor's sanity cap was a flat 250 kt, which is a GA number: the
/// A330's cruise governor was armed at its profile's 290 and held 250 with
/// nothing said, and a spoken "hold two eighty" in any jet was quietly 250
/// (found on the profile review, 2026-09-20). The cap is now the airframe's
/// own, [`ceiling_from_vne`], republished per swap like the floor. Below 10,000 ft the 250 kt rule still applies on top of it
/// (`speedhold::BELOW_10K_LIMIT_KTS`); that is airspace, not airframe.
static SPEED_LIMITS: std::sync::Mutex<SpeedLimits> =
    std::sync::Mutex::new(SpeedLimits { vne_kts: None, vmo_high: None });

/// The live airframe's declared speed limits, the ceilings' one source.
#[derive(Debug, Clone, Copy)]
struct SpeedLimits {
    vne_kts: Option<f64>,
    vmo_high: Option<VmoHigh>,
}
/// The monitor's overspeed caution floor, a fraction of Vne (the monitor's
/// `OVERSPEED_CAUTION_FACTOR` is this one).
pub const OVERSPEED_CAUTION_FACTOR: f64 = 0.90;

/// Knots under the monitor's overspeed caution the ceiling sits: the
/// governor's own wander (296 to 303 kt around a 299 target at FL180) and a
/// descent's first seconds must not reach the caution band.
pub const SPEED_CEILING_MARGIN_KTS: f64 = 10.0;

/// The speed ceiling for a Vne: the overspeed caution's floor
/// ([`OVERSPEED_CAUTION_FACTOR`] of Vne) less [`SPEED_CEILING_MARGIN_KTS`]. It was Vne less 15, which sits INSIDE the
/// caution band for any Vne over 150 kt: a 737 (Vne 340, caution from 306)
/// accepted "hold three twenty" and then called "Overspeed!" on it (refactor
/// audit, 2026-10-04). One rule for the governor and the follower's cruise
/// (`procedure::cruise_cap_kts`). Pure.
pub fn ceiling_from_vne(vne_kts: f64) -> f64 {
    (vne_kts * OVERSPEED_CAUTION_FACTOR - SPEED_CEILING_MARGIN_KTS).floor()
}

/// Publish the airframe's speed ceiling from its Vne and its high-altitude
/// Vmo, if any. Called on EVERY profile swap, `None` included.
pub fn publish_speed_ceiling(vne_kts: Option<f64>, vmo_high: Option<VmoHigh>) {
    *SPEED_LIMITS.lock().unwrap_or_else(|e| e.into_inner()) = SpeedLimits {
        vne_kts: vne_kts.filter(|v| *v > 0.0),
        vmo_high: vmo_high.and_then(VmoHigh::valid),
    };
}

/// The live airframe's speed ceiling, kts, if its profile declares a Vne:
/// the LOW one on an airframe with a `vmo_high:`, for a caller without an
/// altitude (conservative). With one in hand, [`speed_ceiling_kts_at`] or
/// [`VmoCeiling`].
pub fn speed_ceiling_kts() -> Option<f64> {
    SPEED_LIMITS.lock().unwrap_or_else(|e| e.into_inner()).vne_kts.map(ceiling_from_vne)
}

/// A Vmo that applies at and above an INDICATED altitude (the profile's
/// `vmo_high:`); below it the profile's `vne_kts` is the limit.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct VmoHigh {
    /// Indicated altitude, feet, at and above which `kts` is the limit.
    pub above_ft: f64,
    /// The limit up there, KIAS.
    pub kts: f64,
    /// Mmo, when declared: up high the limit is the LOWER of `kts` and the
    /// airspeed this Mach is at the altitude ([`cas_for_mach`]).
    /// The Citation X's 350 is M0.935 by FL290 and M0.94 is 271 KIAS at its
    /// FL430 optimum: a raised Vmo without its Mmo would cruise past Mach.
    #[serde(default)]
    pub mmo: Option<f64>,
}

impl VmoHigh {
    /// Used only when the numbers are sane ones (an insane Mmo is dropped).
    pub fn valid(self) -> Option<Self> {
        (self.above_ft.is_finite() && self.above_ft > 0.0 && self.kts.is_finite() && self.kts > 0.0)
            .then_some(VmoHigh {
                mmo: self.mmo.filter(|m| m.is_finite() && (0.3..1.5).contains(m)),
                ..self
            })
    }

    /// The limit at an indicated altitude in the high band, KIAS: `kts`,
    /// or the Mmo's airspeed there when that is lower.
    fn kts_at(self, alt_ft: f64) -> f64 {
        self.mmo
            .map_or(self.kts, |m| self.kts.min(cas_for_mach(m, alt_ft)))
    }
}

/// Calibrated airspeed, kt, of `mach` at a pressure altitude (ISA,
/// troposphere and the isothermal layer above 36,089 ft). The governor holds
/// INDICATED speed, so a planned Mach is held as the airspeed it is at the
/// level flown: M0.628 at FL180 is 300 kt, M0.78 at FL360 is 260. Pure.
pub fn cas_for_mach(mach: f64, pressure_alt_ft: f64) -> f64 {
    const A0_KTS: f64 = 661.4786;
    let h = pressure_alt_ft.clamp(-2_000.0, 65_000.0);
    let delta = if h <= 36_089.0 {
        (1.0 - 6.875_59e-6 * h).powf(5.255_88)
    } else {
        0.223_36 * (-4.806_34e-5 * (h - 36_089.0)).exp()
    };
    let qc_over_p = (1.0 + 0.2 * mach * mach).powf(3.5) - 1.0;
    let qc_over_p0 = qc_over_p * delta;
    A0_KTS * (5.0 * ((qc_over_p0 + 1.0).powf(2.0 / 7.0) - 1.0)).sqrt()
}

/// The airframe's Vne/Vmo at an INDICATED altitude: `vmo_high`'s figure at
/// and above its altitude (its Mmo's airspeed where lower), `vne_kts` below
/// it, and `vne_kts` too for an altitude not read (the low limit is the
/// conservative one). `None` = no limit declared there. Pure.
pub fn vne_at(vne_kts: Option<f64>, vmo_high: Option<VmoHigh>, alt_ft: f64) -> Option<f64> {
    match vmo_high.and_then(VmoHigh::valid) {
        Some(h) if alt_ft.is_finite() && alt_ft >= h.above_ft => Some(h.kts_at(alt_ft)),
        _ => vne_kts.filter(|v| *v > 0.0),
    }
}

impl AircraftProfile {
    /// This airframe's Vne/Vmo at an indicated altitude ([`vne_at`]).
    #[cfg(test)]
    pub fn vne_at(&self, alt_ft: f64) -> Option<f64> {
        vne_at(self.vne_kts, self.vmo_high, alt_ft)
    }
}

/// The live airframe's Vne/Vmo at an indicated altitude ([`vne_at`] over
/// the published limits).
pub fn published_vne_at(alt_ft: f64) -> Option<f64> {
    let l = *SPEED_LIMITS.lock().unwrap_or_else(|e| e.into_inner());
    vne_at(l.vne_kts, l.vmo_high, alt_ft)
}

/// The live airframe's high-altitude Vmo, when its profile declares a sane one.
pub fn published_vmo_high() -> Option<VmoHigh> {
    SPEED_LIMITS.lock().unwrap_or_else(|e| e.into_inner()).vmo_high.and_then(VmoHigh::valid)
}

/// The live airframe's speed ceiling at an indicated altitude: the clean
/// threshold, for a judgement made once (a spoken number). A loop that
/// holds a speed through a descent takes [`VmoCeiling`], which slows
/// BEFORE the limit drops.
pub fn speed_ceiling_kts_at(alt_ft: f64) -> Option<f64> {
    published_vne_at(alt_ft).map(ceiling_from_vne)
}

/// The highest speed ceiling the live airframe has at ANY altitude: the
/// sanity bound on a target armed without an altitude, which the governor
/// then holds under the ceiling where it flies. Equal to
/// [`speed_ceiling_kts`] without a `vmo_high:`.
pub fn speed_ceiling_kts_max() -> Option<f64> {
    let high = published_vmo_high().map(|h| ceiling_from_vne(h.kts));
    match (speed_ceiling_kts(), high) {
        (Some(a), Some(b)) => Some(a.max(b)),
        (a, b) => a.or(b),
    }
}

/// How far ahead a descent is projected, seconds, for the ceiling it will
/// meet: the low Vmo is flown BEFORE crossing its altitude, not after. A
/// descent at idle bleeds about a knot a second with the pitch shallowed to
/// slow, and 305 to 233 is 72 kt.
pub const VMO_DESCENT_LEAD_S: f64 = 90.0;

/// The climb, fpm, that lifts a held low ceiling once over the altitude.
pub const VMO_RELEASE_CLIMB_FPM: f64 = 300.0;

/// The speed ceiling with memory, for a loop that holds a speed every tick
/// on an airframe with a `vmo_high:`. Low while the descent projected
/// [`VMO_DESCENT_LEAD_S`] ahead is under the altitude; once low it HOLDS
/// (the pitch shallowing to slow would otherwise lift the projection, the
/// ceiling and the speed again, round and round) until the aircraft climbs
/// over the altitude. A level held over it after a descent stays slow,
/// which is legal. Without a `vmo_high:` it is [`speed_ceiling_kts`]
/// exactly, and with nothing measured the low ceiling stands.
#[derive(Debug, Default, Clone, Copy)]
pub struct VmoCeiling {
    /// `None` until the first finite altitude.
    low: Option<bool>,
    /// The last finite altitude, for the Mmo's airspeed on a lost read.
    alt_ft: Option<f64>,
}

impl VmoCeiling {
    /// Advance on the live airframe's published limits.
    pub fn ceiling(&mut self, alt_ft: f64, vs_fpm: f64) -> Option<f64> {
        let l = *SPEED_LIMITS.lock().unwrap_or_else(|e| e.into_inner());
        self.ceiling_for(l.vne_kts, l.vmo_high, alt_ft, vs_fpm)
    }

    /// Advance on indicated `alt_ft` and `vs_fpm` (a vertical speed not read
    /// is level) and return the ceiling this tick. A failed altitude read
    /// keeps what was known. Pure.
    pub fn ceiling_for(
        &mut self,
        vne_kts: Option<f64>,
        vmo_high: Option<VmoHigh>,
        alt_ft: f64,
        vs_fpm: f64,
    ) -> Option<f64> {
        let low_ceiling = vne_kts.filter(|v| *v > 0.0).map(ceiling_from_vne);
        let Some(h) = vmo_high.and_then(VmoHigh::valid) else {
            return low_ceiling;
        };
        if alt_ft.is_finite() {
            let vs = if vs_fpm.is_finite() { vs_fpm } else { 0.0 };
            let ahead = alt_ft + vs.min(0.0) * VMO_DESCENT_LEAD_S / 60.0;
            let low = match self.low {
                Some(true) => !(alt_ft >= h.above_ft && vs > VMO_RELEASE_CLIMB_FPM),
                _ => ahead < h.above_ft,
            };
            self.low = Some(low);
            self.alt_ft = Some(alt_ft);
        }
        match (self.low, self.alt_ft) {
            (Some(false), Some(alt)) => Some(ceiling_from_vne(h.kts_at(alt.max(h.above_ft)))),
            _ => low_ceiling,
        }
    }

    /// Whether the low ceiling holds (`None` before any measurement).
    #[cfg(test)]
    pub fn low(&self) -> Option<bool> {
        self.low
    }
}

/// The ground roll to check a runway against, scaled by ACTUAL weight.
///
/// `roll_at_mtow_m` is the profile's figure, which is a MAX-TAKEOFF-WEIGHT
/// number; applying it at every weight is why an A330 on a light sector was
/// refused Edinburgh's 2,549 m runway by eleven metres — and refused from
/// every position on it, having been sent to line up and then to backtrack
/// (2026-09-20). Real A330s operate there.
///
/// LINEAR in weight, and that choice is the safety argument. Ground roll
/// physically scales nearer W², but measured against the published field
/// lengths (about 2,500 m at 242 t, about 1,800 m at 180 t) the real
/// exponent is about 1.1. Linear therefore asks for slightly MORE runway
/// than the aircraft needs, and the square law would ask for far less —
/// 1,416 m against a true 1,900 at that weight. When a margin has to be
/// wrong, it is wrong upward.
///
/// Clamped to half the MTOW roll at the bottom: nothing here models density
/// altitude, contamination or a rejected takeoff, and the factor the caller
/// applies is not a substitute for a performance calculation. Unknown weight
/// or unknown MTOW keeps the profile's figure exactly as before.
pub fn takeoff_roll_m(roll_at_mtow_m: f64, weight_kg: Option<f64>, mtow_kg: Option<f64>) -> f64 {
    let (Some(w), Some(mtow)) = (weight_kg, mtow_kg) else {
        return roll_at_mtow_m;
    };
    if !w.is_finite() || !mtow.is_finite() || w <= 0.0 || mtow <= 0.0 {
        return roll_at_mtow_m;
    }
    // Never LESS demanding than half, and never less demanding than the
    // profile figure would be above max weight.
    roll_at_mtow_m * (w / mtow).clamp(0.5, 1.0)
}

/// The ground roll a profile checks runways against, metres, and the weight
/// it is quoted at, kg. Prefer the declared GROUND ROLL; the recited takeoff
/// distance is the fallback the GA profiles have always used (on an airliner
/// that figure is a factored field length, see
/// `ReferenceProfile::takeoff_roll_ft`). One derivation for the tool registry
/// and the spoken "do we have enough runway".
pub fn profile_takeoff_roll(profile: &AircraftProfile) -> (Option<f64>, Option<f64>) {
    let r = profile.reference.as_ref();
    (
        r.and_then(|r| r.takeoff_roll_ft.or(r.takeoff_distance_ft))
            .map(|f| f * 0.3048),
        r.and_then(|r| r.max_gross_lb).map(|lb| lb * 0.453_592),
    )
}

/// Pavement the takeoff needs, metres: the weight-scaled ground roll with a
/// 1.4 factor (the book figure is a test pilot's on a dry day), never under
/// `floor_m`. No roll on file = the floor.
///
/// ONE rule for the line-up's full-length check, full auto's "are we lined
/// up" fit, the circuit's and the takeoff's runway fit and the arm-time field
/// check. It was five copies, and the weight scaling reached one of them: the
/// spoken takeoff accepted Edinburgh 24 while full auto, asking the same
/// question of the same aircraft on the same centreline, gave up with "I
/// couldn't get us onto the runway" (2026-09-20, the tower logging
/// `lined_up=true` throughout).
pub fn takeoff_needed_m(
    roll_at_mtow_m: Option<f64>,
    weight_kg: Option<f64>,
    mtow_kg: Option<f64>,
    floor_m: f64,
) -> f64 {
    takeoff_needed_factored_m(roll_at_mtow_m, weight_kg, mtow_kg, floor_m, takeoff_factor())
}

/// The margin on the book roll every runway gate applies.
pub const TAKEOFF_FACTOR: f64 = 1.4;

/// The pilot waived the runway margin for the full-auto mission flying the
/// plan it was waived for (`full_auto::Gate::OriginRunway`): set at that
/// arm, cleared at every other arm, wherever full auto stops, and by the
/// airframe swap, so no other takeoff ever inherits it.
pub static TAKEOFF_MARGIN_WAIVED: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);

/// [`TAKEOFF_FACTOR`], or none while a waived full-auto mission flies.
fn takeoff_factor() -> f64 {
    use std::sync::atomic::Ordering::Relaxed;
    if TAKEOFF_MARGIN_WAIVED.load(Relaxed) {
        1.0
    } else {
        TAKEOFF_FACTOR
    }
}

/// [`takeoff_needed_m`] with the factor given (the preflight asks both
/// ways to say what a waiver gives up).
pub fn takeoff_needed_factored_m(
    roll_at_mtow_m: Option<f64>,
    weight_kg: Option<f64>,
    mtow_kg: Option<f64>,
    floor_m: f64,
    factor: f64,
) -> f64 {
    roll_at_mtow_m.map_or(floor_m, |m| {
        (takeoff_roll_m(m, weight_kg, mtow_kg) * factor).max(floor_m)
    })
}

/// [`takeoff_needed_m`] at what the aircraft weighs NOW. A failed weight read
/// keeps the max-weight figure, which is the conservative end.
pub async fn takeoff_needed_now_m(
    xp: &dyn sim::xplane::XPlane,
    roll_at_mtow_m: Option<f64>,
    mtow_kg: Option<f64>,
    floor_m: f64,
) -> f64 {
    let weight_now = xp
        .get_float(sim::datarefs::TOTAL_WEIGHT_KG)
        .await
        .ok()
        .filter(|w| w.is_finite() && *w > 0.0);
    takeoff_needed_m(roll_at_mtow_m, weight_now, mtow_kg, floor_m)
}

/// The margin on the book LANDING distance every landing-runway gate
/// applies (see [`landing_needed_m`]): none.
pub const LANDING_FACTOR: f64 = 1.0;

/// Pavement a landing needs, metres: the book landing distance, else the
/// book takeoff distance (what every GA POH gives, and the longer of the
/// two there), times [`LANDING_FACTOR`], never under `floor_m`. No figure on
/// file = the floor.
///
/// ONE rule beside [`takeoff_needed_m`], for the planner's destination
/// refusal, full auto's preflight, the plan audit and autoland's self-pick
/// (refactor audit 2026-10-04, finding #10). They disagreed: the planned
/// path took a runway equal to the bare book figure while the emergency
/// path demanded 1.3 times it, so a destination the preflight passed could
/// be one the self-pick would refuse on a go-around. The BOOK figure is the
/// one kept: the jets' POM landing distances are already factored field
/// lengths (the 737's 5,360 ft), so another 1.3 refused a 737 into LXGB's
/// 5,449 ft, which it serves daily, and an emergency pick must never ask
/// more of a runway than the plan did. The floor is the caller's: the
/// self-pick keeps its 1,200 m (it chooses among strips nobody planned), a
/// plan has none (a 172's 530 m grass field is a destination). Pure.
pub fn landing_needed_m(landing_m: Option<f64>, takeoff_m: Option<f64>, floor_m: f64) -> f64 {
    let book = landing_m
        .filter(|d| d.is_finite() && *d > 0.0)
        .or_else(|| takeoff_m.filter(|d| d.is_finite() && *d > 0.0));
    book.map_or(floor_m, |d| (d * LANDING_FACTOR).max(floor_m))
}

/// [`landing_needed_m`] for a PLAN, in feet from the profile's feet: no
/// floor, and `None` (no check) when the profile recites neither figure.
pub fn planned_landing_needed_ft(landing_ft: Option<f64>, takeoff_ft: Option<f64>) -> Option<f64> {
    let need_m = landing_needed_m(
        landing_ft.map(|f| f * 0.3048),
        takeoff_ft.map(|f| f * 0.3048),
        0.0,
    );
    (need_m > 0.0).then(|| (need_m / 0.3048).round())
}

/// The altitude this airframe cruises best at, at `weight_kg`.
///
/// A straight line through the profile's one reference point, because that is
/// what the real curve is over the weights an airliner actually cruises at:
/// the 737-800 sits near FL340 at max takeoff weight and near FL400 empty,
/// which is about 290 ft per tonne burned off. `None` unless the profile
/// declares all three numbers, so nothing invents an optimum for an airframe
/// that does not have one.
///
/// This is an ADVISORY number, never a target: a level belongs to ATC. See
/// [`step_climb_ft`].
pub fn optimum_altitude_ft(r: &ReferenceProfile, weight_kg: f64) -> Option<f64> {
    let (at, ft, slope) = (
        r.optimum_alt_at_kg?,
        r.optimum_alt_ft?,
        r.optimum_alt_ft_per_1000kg?,
    );
    (weight_kg.is_finite() && weight_kg > 0.0).then(|| ft + (at - weight_kg) / 1000.0 * slope)
}

/// How far below optimum is worth a word, and the step it would take.
///
/// The penalty for cruising low is real but gentle, so this does NOT fire for
/// being a few hundred feet off: only when a WHOLE standard step (2,000 ft,
/// the spacing of same-direction levels) sits between us and the optimum is
/// there anything a pilot could act on.
const STEP_CLIMB_FT: f64 = 2000.0;

/// The widest gap a step climb answers: three levels. A step climb is how a
/// jet follows an optimum that drifts UP as fuel burns (EIDW-BIKF: 4,600 ft
/// by mid-Atlantic); a cruise many levels under optimum was PLANNED low, for
/// a reason the copilot cannot see (a short sector, the ceiling, the wind),
/// and one step does not address it. EGBB-EGKK (2026-09-27, 100 nm at
/// 15,000 ft) said "23,300 feet below optimum ... flight level one seven
/// zero would burn less": the gap was true (the profile's line at 64 t) and
/// the level was one step by design, and together they said nothing a pilot
/// could use.
const STEP_CLIMB_MAX_GAP_FT: f64 = 3.0 * STEP_CLIMB_FT;

/// The next same-direction level up, if cruising `cruise_ft` is at least a
/// full step below `optimum_ft` and no more than [`STEP_CLIMB_MAX_GAP_FT`]
/// below it. `None` when we are at or near optimum, above it (a jet above
/// optimum is a different problem and not this one's to solve), or so far
/// below it that the level was planned, not drifted into. The step is
/// always exactly one level: a climb is a request to ATC, and asking for
/// two at once is how you get neither.
pub fn step_climb_ft(cruise_ft: f64, optimum_ft: f64) -> Option<f64> {
    (STEP_CLIMB_FT..=STEP_CLIMB_MAX_GAP_FT)
        .contains(&(optimum_ft - cruise_ft))
        .then_some(cruise_ft + STEP_CLIMB_FT)
}

/// The floor for the LANDING configuration: 1.2 x Vso when the profile
/// declares one, else the clean floor. The clean floor is right for a plan's
/// cruise target and the pattern, wrong for a flapped approach: the 737's
/// clean Vs is 139, its Vref with flaps 40 is 145, and flooring the
/// approach at 167 flew it to the threshold 22 kt fast, over a flap gate
/// (Vfe40 162) the schedule could never pass, and into two go-arounds
/// (LFLL 35L, 2026-09-17). On a 172 (Vso 40, Vs 48) the two floors are
/// 48 and 58 under a 65 kt approach: nothing changes.
static APPROACH_FLOOR_KTS: std::sync::Mutex<Option<f64>> = std::sync::Mutex::new(None);

/// Publish the landing-configuration floor. Called on EVERY profile swap,
/// after [`publish_speed_floor`].
pub fn publish_approach_floor(vs0_kts: Option<f64>, vs_clean_kts: Option<f64>) {
    let by_vso = vs0_kts
        .filter(|v| *v > 0.0)
        .map(|v| (v * SPEED_FLOOR_RATIO).round());
    let clean = vs_clean_kts
        .filter(|v| *v > 0.0)
        .map(|v| (v * SPEED_FLOOR_RATIO).round());
    *APPROACH_FLOOR_KTS.lock().unwrap_or_else(|e| e.into_inner()) = by_vso.or(clean);
}

/// The live airframe's stall table: its clean and landing stall and the
/// detents that declare their own (`FlapDetent::vs_kts`).
#[derive(Debug, Clone, PartialEq)]
pub struct StallCurve {
    pub vs_clean_kts: f64,
    pub vs0_kts: Option<f64>,
    pub points: Vec<(f64, f64)>,
}

static STALL_CURVE: std::sync::Mutex<Option<StallCurve>> = std::sync::Mutex::new(None);

/// Publish the stall table. Called on EVERY profile swap; `None` (the
/// straight line) unless the profile has a clean stall AND a detent that
/// declares its own.
pub fn publish_stall_curve(
    vs_clean_kts: Option<f64>,
    vs0_kts: Option<f64>,
    flaps: Option<&FlapProfile>,
) {
    let points = detent_stall_points(flaps);
    let curve = vs_clean_kts
        .filter(|v| v.is_finite() && *v > 0.0)
        .filter(|_| !points.is_empty())
        .map(|vs| StallCurve {
            vs_clean_kts: vs,
            vs0_kts,
            points,
        });
    *STALL_CURVE.lock().unwrap_or_else(|e| e.into_inner()) = curve;
}

/// The live airframe's stall table, when its profile declares one.
pub fn stall_curve() -> Option<StallCurve> {
    STALL_CURVE.lock().unwrap_or_else(|e| e.into_inner()).clone()
}

/// The live airframe's landing-configuration floor, kts.
pub fn approach_floor_kts() -> Option<f64> {
    *APPROACH_FLOOR_KTS.lock().unwrap_or_else(|e| e.into_inner())
}

/// The floor for the configuration the aircraft is IN: the clean floor with
/// the flaps up, the landing floor with them fully out, interpolated by the
/// flap ratio between. A speed target is only as safe as the wing under it:
/// the 737's Vref 145 is 1.27 Vso with flaps 40 and 1.04 Vs with flaps 5,
/// and the circuit flew its final at flaps 5 toward exactly that number
/// (LFLL, 2026-09-17). Every leg that slows toward the approach speed
/// floors against this with the flaps it has actually commanded.
pub fn config_floor_kts(flap_ratio: f64) -> Option<f64> {
    let r = if flap_ratio.is_finite() {
        flap_ratio.clamp(0.0, 1.0)
    } else {
        0.0
    };
    // Declared per-detent stall speeds: the floor is 1.2 x the stall IN this
    // configuration, not a line between the clean and landing floors (the
    // A330's CONF 2 floor is 167 by its stall table, 180 by the line).
    if let Some(c) = stall_curve() {
        return Some(
            (SPEED_FLOOR_RATIO * stall_speed_in_config(c.vs_clean_kts, c.vs0_kts, &c.points, r))
                .round(),
        );
    }
    match (speed_floor_kts(), approach_floor_kts()) {
        (Some(clean), Some(landing)) => Some((clean + (landing - clean) * r).round()),
        (Some(clean), None) => Some(clean),
        (None, Some(landing)) => Some(landing),
        (None, None) => None,
    }
}

/// The least thrust the autoland holds on a stabilised final, as a lever
/// ratio (`None` = idle is allowed). A jet's approach is flown ON thrust:
/// the 737 idled for the last mile to lose the circuit's excess and arrived
/// over the numbers 4 kt under Vref still spooling up (LFLL, 2026-09-17).
static APPROACH_THRUST_MIN: std::sync::Mutex<Option<f64>> = std::sync::Mutex::new(None);

/// Publish the airframe's approach thrust floor. Called on EVERY profile swap.
pub fn publish_approach_thrust_min(min: Option<f64>) {
    *APPROACH_THRUST_MIN
        .lock()
        .unwrap_or_else(|e| e.into_inner()) =
        min.filter(|v| v.is_finite() && (0.0..=1.0).contains(v));
}

/// The live airframe's approach thrust floor, if its profile declares one.
pub fn approach_thrust_min() -> Option<f64> {
    *APPROACH_THRUST_MIN
        .lock()
        .unwrap_or_else(|e| e.into_inner())
}

/// The landing arrest's lead for the live airframe (see [`ArrestLeadProfile`]).
static ARREST_LEAD: std::sync::Mutex<ArrestLeadProfile> =
    std::sync::Mutex::new(ArrestLeadProfile {
        gain: 2.0,
        max_climb_fpm: 300.0,
        trend_seconds: 0.0,
    });

/// Publish the airframe's arrest lead. Called on EVERY profile swap; an
/// absent or malformed block is the default, never the previous airframe's.
pub fn publish_arrest_lead(lead: Option<ArrestLeadProfile>) {
    *ARREST_LEAD.lock().unwrap_or_else(|e| e.into_inner()) =
        lead.and_then(ArrestLeadProfile::valid).unwrap_or_default();
}

/// The live airframe's arrest lead.
pub fn arrest_lead() -> ArrestLeadProfile {
    *ARREST_LEAD.lock().unwrap_or_else(|e| e.into_inner())
}

/// The live airframe's whole profile, for the few consumers that need the
/// tool surface itself away from the command loop (the stand shutdown builds
/// the registry from it). Republished on EVERY swap by `publish_airframe`.
static LIVE_PROFILE: std::sync::Mutex<Option<std::sync::Arc<AircraftProfile>>> =
    std::sync::Mutex::new(None);

pub fn publish_live_profile(p: &AircraftProfile) {
    *LIVE_PROFILE.lock().unwrap_or_else(|e| e.into_inner()) = Some(std::sync::Arc::new(p.clone()));
}

pub fn live_profile() -> Option<std::sync::Arc<AircraftProfile>> {
    LIVE_PROFILE.lock().unwrap_or_else(|e| e.into_inner()).clone()
}

/// The flare's elevator law for the live airframe (see
/// [`FlareElevatorProfile`]); `None` is the default law.
static FLARE_ELEVATOR: std::sync::Mutex<Option<FlareElevatorProfile>> =
    std::sync::Mutex::new(None);

/// Publish it. Called on EVERY profile swap; absent is the default law,
/// never the previous airframe's.
pub fn publish_flare_elevator(p: Option<FlareElevatorProfile>) {
    *FLARE_ELEVATOR.lock().unwrap_or_else(|e| e.into_inner()) =
        p.and_then(FlareElevatorProfile::valid);
}

pub fn flare_elevator() -> Option<FlareElevatorProfile> {
    *FLARE_ELEVATOR.lock().unwrap_or_else(|e| e.into_inner())
}

/// The live airframe's rudder authority on the roll (see
/// [`RollRudderProfile`]); `None` is the default roll law.
static ROLL_RUDDER: std::sync::Mutex<Option<RollRudderProfile>> = std::sync::Mutex::new(None);

/// Publish it. Called on EVERY profile swap; absent is the default law,
/// never the previous airframe's.
pub fn publish_roll_rudder(p: Option<RollRudderProfile>) {
    *ROLL_RUDDER.lock().unwrap_or_else(|e| e.into_inner()) = p.and_then(RollRudderProfile::valid);
}

pub fn roll_rudder() -> Option<RollRudderProfile> {
    *ROLL_RUDDER.lock().unwrap_or_else(|e| e.into_inner())
}

/// The live airframe's landing distance in metres, if its profile recites
/// one. The autoland's float rule asks it how much pavement a landing from
/// here still needs; `None` is the old GA figure.
static LANDING_DISTANCE_M: std::sync::Mutex<Option<f64>> = std::sync::Mutex::new(None);

/// Publish the airframe's landing distance. Called on EVERY profile swap.
pub fn publish_landing_distance(feet: Option<f64>) {
    *LANDING_DISTANCE_M.lock().unwrap_or_else(|e| e.into_inner()) = feet
        .filter(|v| v.is_finite() && *v > 0.0)
        .map(|v| v * 0.3048);
}

/// The live airframe's landing distance, metres.
pub fn landing_distance_m() -> Option<f64> {
    *LANDING_DISTANCE_M.lock().unwrap_or_else(|e| e.into_inner())
}

/// `asked`, raised to the floor when it is under it. Pure; the callers say so
/// aloud and record it.
pub fn floored_target(asked: f64, floor: Option<f64>) -> f64 {
    match floor {
        Some(f) if asked < f => f,
        _ => asked,
    }
}

/// The live airframe's VLO/VLE, if its profile declares one.
pub fn gear_limit_kts() -> Option<f64> {
    *GEAR_LIMIT_KTS.lock().unwrap_or_else(|e| e.into_inner())
}

/// How far a flap reading may sit from a detent and still be AT it: the
/// flaps tool's "already there" test, under the closest detent spacing any
/// profile declares (the 737's eighths, 0.125).
pub const FLAP_DETENT_TOLERANCE: f64 = 0.02;

/// The placard of the flap that is out: the lowest Vfe of every detent at or
/// inside `flap_ratio` (within [`FLAP_DETENT_TOLERANCE`]). `None` clean, or
/// with no Vfe declared. `limits` is `(ratio, Vfe)` as [`flap_limits`] gives.
///
/// ONE lookup for the circuit's clean-up, autoland's target and missed
/// approach, the monitor's overspeed and the flaps tool (refactor audit
/// 2026-10-04, finding #12). They disagreed on two things, decided here:
/// - an entry at ratio 0 never carries a placard. Clean is limited by Vne,
///   not a Vfe, and a generic `.acf` read can put a Vfe on the retracted
///   row; the circuit counted it and capped a clean pattern with it while
///   autoland did not.
/// - the detent tolerance is 0.02. The circuit and autoland used 1e-6
///   (their ratios are commanded, exact); the monitor used 0.05 on a READ
///   position, which would take the next detent's limit on any profile with
///   detents closer than that. 0.02 counts a reading at its detent and no
///   other. Pure.
pub fn flap_placard_kts(flap_ratio: f64, limits: &[(f64, Option<f64>)]) -> Option<f64> {
    if !flap_ratio.is_finite() {
        return None;
    }
    limits
        .iter()
        .filter(|(ratio, _)| *ratio > 1e-6 && *ratio <= flap_ratio + FLAP_DETENT_TOLERANCE)
        .filter_map(|(_, vfe)| *vfe)
        .reduce(f64::min)
}

/// The live airframe's flap detents as `(ratio, Vfe)`, retracted first.
pub fn flap_limits() -> Vec<(f64, Option<f64>)> {
    FLAP_LIMITS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()
}

/// The dataref every throttle write and read should use.
///
/// The Citation X is why this exists. Its `CitX_throttle.lua` tracks
/// `throttle_jet_rev_ratio`, never `throttle_ratio_all`, and re-drives its own
/// levers from that every frame — so every reduction the speed governor
/// commanded went to a dataref the aircraft ignores. Flown live 2026-07-28 it
/// held takeoff thrust through the whole pattern: 215 knots on upwind, 309 on
/// crosswind, 373 on downwind against a 198 kt target, past both Vmo figures,
/// still climbing with a descent commanded. The copilot could not slow it down
/// and had no way to know.
pub fn throttle_dataref() -> String {
    THROTTLE_DREF
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()
        .unwrap_or_else(|| STOCK_THROTTLE.to_string())
}

/// A profile for an airframe nobody has written one for, built from the
/// placards in its own `.acf` file: the speeds, the flap detents and their
/// limits, whether the gear retracts, whether the engine is carburetted. The
/// tool surface is the stock 172's (stock datarefs are the best guess for a
/// stock-systems airframe); only the NUMBERS change, and those are the ones
/// that hurt. The RV-10 flew on the 172's 65 kt approach speed — its own clean
/// stall speed — because the fallback was silent and total (2026-09-11).
///
/// `acf_text` is the file's contents (`P key value` lines). Pure, so it
/// battle-tests on a fixture. Approach speed is the larger of 1.3 Vso and
/// 1.2 Vs: the acf's Vso is what the author typed, and the model can stall
/// above it with partial flap (the RV-10 did, at 63 kt against a Vso of 50).
pub fn generic_from_acf(icao: &str, acf_text: &str) -> AircraftProfile {
    let vals: std::collections::HashMap<&str, &str> = acf_text
        .lines()
        .filter_map(|l| {
            let mut it = l.split_whitespace();
            (it.next()? == "P").then(|| (it.next(), it.next()))?.into()
        })
        .filter_map(|(k, v)| Some((k?, v?)))
        .collect();
    let num = |k: &str| vals.get(k).and_then(|v| v.parse::<f64>().ok());
    let kts = |k: &str| num(k).filter(|v| *v > 0.0 && *v < 9000.0);
    let mut p = cessna_172sp();
    p.id = format!("generic_{}", icao.to_lowercase());
    // Built on the 172's bones, but never detected as one.
    p.matcher = None;
    p.pressurised = None; // Unknown systems must use neutral cabin-altitude wording.
    let name = vals
        .get("acf/_name")
        .map(|s| s.to_string())
        .unwrap_or_else(|| icao.to_string());
    // `_name` is one token here (split on whitespace); good enough for a label.
    p.display_name = format!("{name} (no profile)");
    p.checklist = None;
    p.checklist_map = Vec::new();
    p.checklist_flow = Default::default();
    p.bus_volts_nominal = None;

    let vs = kts("acf/_Vs_kts");
    let vso = kts("acf/_Vso_kts");
    p.vs_clean_kts = vs.or(p.vs_clean_kts);
    p.vne_kts = kts("acf/_Vne_kts").or(p.vne_kts);
    let vle = kts("acf/_Vle_kts");
    p.retractable_gear = vle.is_some();
    p.vlo_kts = vle;
    p.gear_warning = gear_warning_from_acf(acf_text);
    let injected = vals
        .get("_engn/0/_type")
        .is_some_and(|t| t.contains("INJ") || !t.starts_with("RCP"));
    p.carb_heat = !injected;
    if vals
        .get("_engn/0/_type")
        .is_some_and(|t| !t.starts_with("RCP"))
    {
        // Not a piston: the magneto and mixture-side tools would be lies.
        p.magnetos = false;
        p.fuel_pump = false;
    }

    // Flap detents: `_flap_detents` notches at `_flap1_dn/i` degrees; ratio
    // is degrees over the last notch; Vfe1 for the first, Vfem for the last,
    // a straight line between.
    let detents = num("acf/_flap_detents").map(|d| d as usize).unwrap_or(0);
    let degs: Vec<f64> = (1..=detents)
        .filter_map(|i| num(&format!("acf/_flap1_dn/{i}")))
        .filter(|d| *d > 0.0)
        .collect();
    if let (Some(last), true) = (degs.last().copied(), !degs.is_empty()) {
        let vfe1 = kts("acf/_Vfe1_kts");
        let vfem = kts("acf/_Vfem_kts").or(vfe1);
        let n = degs.len();
        let mut list = vec![FlapDetent {
            label: "up".into(),
            spoken: "up".into(),
            ratio: 0.0,
            vfe_kts: None,
            // A generic airframe declares no manoeuvring speeds, so it keeps
            // the one-step clean-up on `initial_climb_kts`.
            retract_above_kts: None,
            vs_kts: None,
        }];
        // A slatted wing's per-detent stall, from the same file; only when
        // every notch maps one to one (a slats-only notch at 0 degrees is
        // dropped from `degs`, and the stall line is then kept).
        let acf_stall = detent_stall_from_acf(acf_text).filter(|v| v.len() == n);
        for (i, d) in degs.iter().enumerate() {
            let full = i + 1 == n;
            let vfe = match (vfe1, vfem) {
                (Some(a), Some(b)) if n > 1 => Some(a + (b - a) * i as f64 / (n - 1) as f64),
                (a, b) => b.or(a),
            };
            list.push(FlapDetent {
                label: if full {
                    "full".into()
                } else {
                    format!("{d:.0}")
                },
                spoken: if full {
                    "full".into()
                } else {
                    format!("{d:.0} degrees")
                },
                ratio: (d / last * 10_000.0).round() / 10_000.0,
                vfe_kts: vfe.map(f64::round),
                retract_above_kts: None,
                vs_kts: acf_stall.as_ref().and_then(|v| v.get(i).copied()),
            });
        }
        p.flaps = Some(FlapProfile {
            detents: list,
            takeoff_ratio: Some(0.0),
        });
    }

    let r = p.reference.get_or_insert_with(Default::default);
    r.vno_kts = kts("acf/_Vno_kts").or(r.vno_kts);
    r.va_kts = kts("acf/_Va_kts").or(r.va_kts);
    r.vx_kts = kts("acf/_Vx_kts").or(r.vx_kts);
    r.vy_kts = kts("acf/_Vy_kts").or(r.vy_kts);
    r.best_glide_kts = kts("acf/_Vbg_kts").or(r.best_glide_kts);
    r.vs0_kts = vso.or(r.vs0_kts);
    if let Some(vs) = vs {
        r.vr_kts = Some(fallback_vr_kts(vs));
        r.approach_kts = Some(fallback_approach_kts(vso, vs));
    }
    if let Some(vno) = r.vno_kts {
        r.cruise_ias_kts = Some((vno * 0.85).round());
    }
    r.cruise_kts = None;
    r.best_climb_fpm = None;
    r.max_gross_lb = num("acf/_m_max").or(r.max_gross_lb);
    r.empty_weight_lb = num("acf/_m_empty").or(r.empty_weight_lb);
    r.useful_load_lb = match (r.max_gross_lb, r.empty_weight_lb) {
        (Some(g), Some(e)) if g > e => Some(g - e),
        _ => None,
    };
    for f in [
        &mut r.service_ceiling_ft,
        &mut r.fuel_capacity_gal,
        &mut r.fuel_useable_gal,
        &mut r.endurance_hours,
        &mut r.takeoff_distance_ft,
        &mut r.landing_distance_ft,
    ] {
        *f = None; // not in the acf: not recited
    }
    p
}

/// Every dataref and command name a profile relies on, as strings, for the
/// probe that asks the sim whether each one exists. A walk over the profile's
/// own serialisation rather than a hand list of fields: a new field with a
/// dataref in it is probed the day it is added, not the day someone remembers.
/// Names are `a/b/...` paths without whitespace; keys that carry prose or
/// file paths are skipped, an `[index]` suffix is dropped.
pub fn profile_references(p: &AircraftProfile) -> Vec<String> {
    const SKIP_KEYS: [&str; 12] = [
        "match",
        "say",
        "spoken",
        "label",
        "key",
        "kind",
        "checklist",
        "id",
        "display_name",
        "voice",
        "phases",
        "positions_spoken",
    ];
    fn walk(v: &serde_yaml::Value, key: &str, out: &mut Vec<String>) {
        match v {
            serde_yaml::Value::String(s) => {
                if SKIP_KEYS.contains(&key) {
                    return;
                }
                let name = s.split('[').next().unwrap_or(s).trim();
                let last = name.rsplit('/').next().unwrap_or("");
                if name.contains('/')
                    && !name.contains(char::is_whitespace)
                    && !last.contains('.')
                    && !name.starts_with('/')
                    && !out.iter().any(|o| o == name)
                {
                    out.push(name.to_string());
                }
            }
            serde_yaml::Value::Sequence(items) => items.iter().for_each(|i| walk(i, key, out)),
            serde_yaml::Value::Mapping(m) => {
                for (k, v) in m {
                    let k = k.as_str().unwrap_or("");
                    walk(v, k, out);
                }
            }
            _ => {}
        }
    }
    let mut out = Vec::new();
    if let Ok(v) = serde_yaml::to_value(p) {
        walk(&v, "", &mut out);
    }
    // The takeoff configuration's declarations carry no names either: the
    // stock ones behind each ([`TakeoffConfigProfile::references`]).
    if let Some(tc) = &p.takeoff_config {
        for name in tc.references() {
            if !out.iter().any(|o| o == name) {
                out.push(name.to_string());
            }
        }
    }
    // Fields that carry no name of their own, only indices into stock
    // arrays the tool reads and writes: the names they rely on.
    if !p.fuel_tank_pumps_off_when_empty.is_empty() {
        for name in [
            crate::datarefs::FUEL_TANK_PUMP_ON,
            crate::datarefs::FUEL_PER_TANK_KG,
            crate::datarefs::FUEL_TANK_LATERAL_M,
        ] {
            if !out.iter().any(|o| o == name) {
                out.push(name.to_string());
            }
        }
    }
    out
}

/// Which of a profile's names the sim does not have: neither a dataref nor
/// a command. Pure over the two indexes, so the probe's verdict tests
/// without a sim.
pub fn missing_references(
    names: &[String],
    datarefs: &std::collections::HashSet<String>,
    commands: &std::collections::HashSet<String>,
) -> Vec<String> {
    names
        .iter()
        .filter(|n| !datarefs.contains(*n) && !commands.contains(*n))
        .cloned()
        .collect()
}

/// The spoken result of a probe: nothing when every name exists, otherwise
/// the count and the first few, so the pilot knows which tools will fail
/// before asking for them.
pub fn probe_summary(missing: &[String], total: usize) -> Option<String> {
    if missing.is_empty() {
        return None;
    }
    let shown: Vec<String> = missing
        .iter()
        .take(3)
        .map(|m| m.rsplit('/').next().unwrap_or(m).replace('_', " "))
        .collect();
    let more = if missing.len() > 3 {
        format!(" and {} more", missing.len() - 3)
    } else {
        String::new()
    };
    Some(format!(
        "Profile check: {} of {total} names this profile relies on are not in this aircraft: {}{more}. The tools that use them will fail.",
        missing.len(),
        shown.join(", ")
    ))
}

/// Built-in profile by id (fallback when no YAML file exists).
pub fn builtin(id: &str) -> Option<AircraftProfile> {
    match id {
        "cessna_172sp" => Some(cessna_172sp()),
        "cirrus_sr22" => Some(cirrus_sr22()),
        _ => None,
    }
}

#[cfg(test)]
#[allow(clippy::items_after_test_module)]
mod tests;

/// The X-Plane stock Cessna 172SP Skyhawk.
///
/// Flap detents 0°/10°/20°/30°; Vfe 110 KIAS for the first 10° and 85 KIAS
/// beyond. Fixed gear. Full piston panel + a heading/altitude/VS autopilot.
fn cessna_172sp() -> AircraftProfile {
    let detent = |label: &str, spoken: &str, ratio: f64, vfe: Option<f64>| FlapDetent {
        retract_above_kts: None,
        vs_kts: None,
        label: label.into(),
        spoken: spoken.into(),
        ratio,
        vfe_kts: vfe,
    };
    let light =
        |key: &str, spoken: &str, dataref: &str, cmd_on: &str, cmd_off: &str| LightProfile {
            also_datarefs: Vec::new(),
            toggles: Vec::new(),
            auto_at_night: false,
            key: key.into(),
            spoken: spoken.into(),
            dataref: dataref.into(),
            command_on: Some(cmd_on.into()),
            command_off: Some(cmd_off.into()),
        };
    let fuel_pos = |key: &str, spoken: &str, command: &str, value: Option<f64>| FuelPosition {
        key: key.into(),
        spoken: spoken.into(),
        command: command.into(),
        value,
    };
    AircraftProfile {
        id: "cessna_172sp".into(),
        display_name: "Cessna 172SP Skyhawk".into(),
        simbrief_types: Vec::new(),
        matcher: Some(ProfileMatch {
            icao: vec!["C172".into()],
            acf: vec!["Laminar Research/Cessna 172 SP/".into()],
            ..Default::default()
        }),
        engines: 1,
        magnetos_per_engine: false,
        flood_lights_dataref: None,
        flood_lights_toggle: None,
        gps_ack_command: None,
        // Cycle-switch systems (the DC-3 pattern) — not used by the GA singles.
        power_switch: None,
        fuel_selectors: Vec::new(),
        piston_start: None,
        mixture_notch: None,
        turboprop_start: None,
        // A piston has no ENG MODE knob and no master switches.
        airliner_start: None,
        retractable_gear: false,
        gear_by_command: false,
        rotorcraft: false,
        heli_start: None,
        transition_altitude_ft: 18_000.0,
        view_blend: 1.0,
        locations: std::collections::HashMap::new(),
        view_map: std::collections::HashMap::new(),
        autopilot: true,
        autopilot_prep: None,
        engine: true,
        throttle: false,
        prop: None,
        turbine_start: false,
        auto_prep: false,
        scripted_auto_start: false,
        fuel_cutoff_levers: Vec::new(),
        fuel_cutoff_value: 0.0,
        autothrottle: false,
        speedbrake: false,
        barometer: true,
        directional_gyro: true,
        com_radio: true,
        transponder: true,
        // Stock actuator: both builtins model a sim-native transponder.
        transponder_mode_knob: None,
        takeoff_config: None,
        takeoff_power: None,
        seat_belt_signs: None,
        ap_altitude_capture: false,
        alt_dial_park_field: false,
        landing_flap_lead_nm: None,
        ils: true,
        parking_brake: true,
        cabin_crew: false,
        load_sheet: LoadSheet::Stock,
        ils_coupled: false,
        approach_command: None,
        approach_status: None,
        bus_volts_nominal: None,
        // The stock profile carries the same continuous flare laws. Keep the
        // builtin sound when profiles/ is unavailable.
        flare_power: Some(FlarePowerProfile {
            reduction_per_second: 0.10,
            retard_below_ft: None,
            retard_max_per_s: Some(0.10),
            retard_slow_kts: None,
            retard_at_s: None,
            retard_fast_kts: None,
            hold_from_s: None,
        }),
        approach_thrust_min: None,
        arrest_lead: None,
        flare_elevator: Some(FlareElevatorProfile {
            gain: 1.0,
            trim_gain: 1.0,
            max: 0.35,
            max_push: 0.10,
            balloon_ease_per_s: Some(0.15),
            tau_s: None,
            model: None,
        }),
        roll_rudder: None,
        rotate_elevator: None,
        arrest_roll: None,
        ground_plant: None,
        rollout_steer: None,
        speed_governor: None,
        cabin_air: None,
        pressurised: Some(false),
        park_brake_valve_set: None,
        electrical: {
            let e = |key: &str, spoken: &str, on: &[&str], off: &[&str], rb: &[&str]| {
                ElectricalSystem {
                    write_datarefs: Vec::new(),
                    toggles: Vec::new(),
                    guards: Vec::new(),
                    key: key.into(),
                    spoken: spoken.into(),
                    on_commands: on.iter().map(|s| s.to_string()).collect(),
                    off_commands: off.iter().map(|s| s.to_string()).collect(),
                    readback: rb.iter().map(|s| s.to_string()).collect(),
                }
            };
            vec![
                e(
                    "battery",
                    "battery",
                    &["sim/electrical/battery_1_on"],
                    &["sim/electrical/battery_1_off"],
                    &["sim/cockpit/electrical/battery_on"],
                ),
                e(
                    "alternator",
                    "alternator",
                    &["sim/electrical/generator_1_on"],
                    &["sim/electrical/generator_1_off"],
                    &["sim/cockpit/electrical/generator_on[0]"],
                ),
                // The two AVIONICS switches: BUS 1 (avionics master) + BUS 2 (cross-tie).
                e(
                    "avionics",
                    "avionics master",
                    &["sim/systems/avionics_on", "sim/electrical/cross_tie_on"],
                    &["sim/systems/avionics_off", "sim/electrical/cross_tie_off"],
                    &[
                        "sim/cockpit/electrical/avionics_on",
                        "sim/cockpit2/electrical/cross_tie",
                    ],
                ),
                // Split BAT/ALT rocker — battery + alternator together, NOT avionics.
                e(
                    "master",
                    "master",
                    &[
                        "sim/electrical/battery_1_on",
                        "sim/electrical/generator_1_on",
                    ],
                    &[
                        "sim/electrical/battery_1_off",
                        "sim/electrical/generator_1_off",
                    ],
                    &[
                        "sim/cockpit/electrical/battery_on",
                        "sim/cockpit/electrical/generator_on[0]",
                    ],
                ),
            ]
        },
        flood_lights: true,
        fuel_pump: true,
        fuel_pump_commands: None,
        fuel_tank_pumps_off_when_empty: Vec::new(),
        reverse_thrust: false,
        magnetos: true,
        carb_heat: true,
        // The 172 selector is LEFT/BOTH/RIGHT (OFF is a separate shutoff valve,
        // not a selector position). Commands animate the 3D knob; the logical
        // position dataref confirms (1 left, 3 right, 4 both).
        fuel_selector: Some(FuelSelectorProfile {
            readback: Some("sim/cockpit2/fuel/fuel_tank_selector".into()),
            positions: vec![
                fuel_pos("left", "left", "sim/fuel/fuel_selector_lft", Some(1.0)),
                fuel_pos("both", "both", "sim/fuel/fuel_selector_all", Some(4.0)),
                fuel_pos("right", "right", "sim/fuel/fuel_selector_rgt", Some(3.0)),
            ],
        }),
        pitot_heat: true,
        pitot_heat_dataref: None,
        advisor: vec![],
        pitch_trim: true,
        nav_radio: true,
        com2_radio: true,
        flaps: Some(FlapProfile {
            detents: vec![
                detent("up", "up", 0.0, None),
                detent("10", "ten degrees", 1.0 / 3.0, Some(110.0)),
                detent("20", "twenty degrees", 2.0 / 3.0, Some(85.0)),
                detent("full", "full", 1.0, Some(85.0)),
            ],
            takeoff_ratio: Some(0.0), // 172 takes off clean
        }),
        fuel: Some(FuelProfile {
            reserve_caution_min: 45.0,
            reserve_warning_min: 30.0,
            kg_per_gal: None,
            spoken_unit: None,
            cruise_burn_kg_hr: 24.0,
        }),
        // 172SP clean stall ~48 KIAS (power-off, flaps up).
        vs_clean_kts: Some(48.0),
        // 172SP Vne (never-exceed) 163 KIAS.
        vne_kts: Some(163.0),
        vmo_high: None,
        vlo_kts: None,
        gear_warning: None,
        // The GA pace is the built-in default — nothing to declare.
        taxi: None,
        throttle_dataref: None,
        tail_strike_deg: None,
        // A light single runs out of elevator before it runs out of tail
        // clearance, and only ever needs the rotation held BACK.
        pitch_authority_down: None,
        climb_pitch_max_deg: None,
        circuit_final_lead_s: None,
        // Reference figures from the X-Plane C172 Pilot's Operating Manual.
        reference: Some(ReferenceProfile {
            kind: Some("single-engine aircraft".into()),
            // A piston does not step-climb: it goes as high as it can and
            // stays. No optimum to advise, so nothing is said.
            optimum_alt_ft: None,
            // ...and the GA vertical rates are the shared default already.
            enroute_climb_fpm: None,
            enroute_descent_fpm: None,
            optimum_alt_at_kg: None,
            optimum_alt_ft_per_1000kg: None,
            vno_kts: Some(129.0),
            va_kts: Some(99.0),
            vx_kts: Some(62.0),
            vy_kts: Some(74.0),
            // A trainer climbs out at Vy; only airframes that do not say so.
            initial_climb_kts: None,
            acceleration_height_agl_ft: None,
            vyse_kts: None,
            vmca_kts: None,
            vr_kts: Some(55.0),
            rotate_rate_deg_s: None,
            best_glide_kts: Some(68.0),
            vs0_kts: Some(40.0),
            approach_kts: Some(65.0),
            max_crosswind_kts: Some(15.0),
            cruise_kts: Some(124.0),
            cruise_ias_kts: Some(108.0), // KIAS to hold in cruise (124 kt TAS)
            best_climb_fpm: Some(730.0),
            max_gross_lb: Some(2558.0),
            empty_weight_lb: Some(1640.0),
            useful_load_lb: Some(918.0),
            service_ceiling_ft: Some(14000.0),
            fuel_capacity_gal: Some(56.0),
            fuel_useable_gal: Some(53.0),
            endurance_hours: Some(4.0),
            takeoff_distance_ft: Some(1630.0),
            // No separate ground roll: on a GA single the over-50-ft figure is
            // already a sane "runway needed", and it is what has been flown.
            takeoff_roll_ft: None,
            landing_distance_ft: Some(1335.0),
            max_rpm: Some(2700.0),
            oil_temp_min_f: Some(100.0),
            oil_temp_max_f: Some(245.0),
            oil_press_min_psi: Some(50.0),
            oil_press_max_psi: Some(90.0),
        }),
        checklist: Some("checklists/cessna_172sp.txt".into()),
        checklist_map: {
            let m = |s: &str, a: ChecklistAction| ChecklistMapEntry {
                match_: s.into(),
                action: a,
            };
            use ChecklistAction::*;
            vec![
                m("parkbrake", ParkingBrake),
                m("autopilot/autopilot_on", AutopilotOff),
                m("electrical/battery_on", Electrical("battery".into())),
                m("electrical/avionics_on", Electrical("avionics".into())),
                m("electrical/generator_on", Electrical("alternator".into())),
                m("pitot_heat_on", PitotHeat),
                m("fuel_pump_on", FuelPump),
                m("transponder_mode", Transponder),
                m("fuel_tank_selector", FuelSelector),
                m("flap_ratio", Flaps),
                m("mixture_ratio", Mixture),
                m("throttle_ratio", Throttle),
                m("ignition_key", Ignition),
                m("elevator_trim", PitchTrim),
                m("taxi_light", Light("taxi".into())),
                m("landing_lights", Light("landing".into())),
                m("strobe_lights", Light("strobe".into())),
                m("beacon_lights", Light("beacon".into())),
                m("navigation_lights", Light("nav".into())),
                m("nav_lights", Light("nav".into())),
            ]
        },
        checklist_flow: Default::default(),
        altimeter: None,
        anti_ice: None,
        autoflight: None,
        lights: vec![
            light(
                "nav",
                "navigation lights",
                "sim/cockpit2/switches/navigation_lights_on",
                "sim/lights/nav_lights_on",
                "sim/lights/nav_lights_off",
            ),
            light(
                "beacon",
                "beacon",
                "sim/cockpit2/switches/beacon_on",
                "sim/lights/beacon_lights_on",
                "sim/lights/beacon_lights_off",
            ),
            light(
                "strobe",
                "strobe lights",
                "sim/cockpit2/switches/strobe_lights_on",
                "sim/lights/strobe_lights_on",
                "sim/lights/strobe_lights_off",
            ),
            light(
                "landing",
                "landing light",
                "sim/cockpit2/switches/landing_lights_on",
                "sim/lights/landing_lights_on",
                "sim/lights/landing_lights_off",
            ),
            light(
                "taxi",
                "taxi light",
                "sim/cockpit2/switches/taxi_light_on",
                "sim/lights/taxi_lights_on",
                "sim/lights/taxi_lights_off",
            ),
        ],
    }
}

/// The X-Plane stock Cirrus SR22 (Laminar). Built from the GoofyJP XChecklist
/// file, the POH, and the rustdecks SR22 deck (the verified dataref/command
/// source). Enabled systems write universal X-Plane datarefs or rustdecks-
/// verified commands: the dual battery/alternator, mixture, throttle, boost
/// pump, key-start ignition, pitch trim, and the off/left/right fuel selector.
/// Still off: carb heat (fuel-injected) and the vacuum DG (G1000 glass).
fn cirrus_sr22() -> AircraftProfile {
    let detent = |label: &str, spoken: &str, ratio: f64, vfe: Option<f64>| FlapDetent {
        retract_above_kts: None,
        vs_kts: None,
        label: label.into(),
        spoken: spoken.into(),
        ratio,
        vfe_kts: vfe,
    };
    let light =
        |key: &str, spoken: &str, dataref: &str, cmd_on: &str, cmd_off: &str| LightProfile {
            also_datarefs: Vec::new(),
            toggles: Vec::new(),
            auto_at_night: false,
            key: key.into(),
            spoken: spoken.into(),
            dataref: dataref.into(),
            command_on: Some(cmd_on.into()),
            command_off: Some(cmd_off.into()),
        };
    let fuel_pos = |key: &str, spoken: &str, command: &str, value: Option<f64>| FuelPosition {
        key: key.into(),
        spoken: spoken.into(),
        command: command.into(),
        value,
    };
    AircraftProfile {
        id: "cirrus_sr22".into(),
        display_name: "Cirrus SR22".into(),
        simbrief_types: Vec::new(),
        matcher: Some(ProfileMatch {
            icao: vec!["SR22".into()],
            acf: vec!["Laminar Research/Cirrus SR22/".into()],
            ..Default::default()
        }),
        engines: 1,
        magnetos_per_engine: false,
        flood_lights_dataref: None,
        flood_lights_toggle: None,
        gps_ack_command: None,
        // Cycle-switch systems (the DC-3 pattern) — not used by the GA singles.
        power_switch: None,
        fuel_selectors: Vec::new(),
        piston_start: None,
        mixture_notch: None,
        turboprop_start: None,
        // A piston has no ENG MODE knob and no master switches.
        airliner_start: None,
        retractable_gear: false,
        gear_by_command: false,
        rotorcraft: false,
        heli_start: None,
        transition_altitude_ft: 18_000.0,
        view_blend: 1.0,
        locations: std::collections::HashMap::new(),
        view_map: std::collections::HashMap::new(),
        // Universal-dataref tools — safe across airframes.
        autopilot: true,
        autopilot_prep: None,
        barometer: true,
        com_radio: true,
        com2_radio: true,
        nav_radio: true,
        transponder: true,
        // Stock actuator: both builtins model a sim-native transponder.
        transponder_mode_knob: None,
        takeoff_config: None,
        takeoff_power: None,
        seat_belt_signs: None,
        ap_altitude_capture: false,
        alt_dial_park_field: false,
        landing_flap_lead_nm: None,
        ils: true,
        parking_brake: true,
        cabin_crew: false,
        load_sheet: LoadSheet::Stock,
        ils_coupled: false,
        approach_command: None,
        approach_status: None,
        bus_volts_nominal: None,
        flare_power: None,
        approach_thrust_min: None,
        arrest_lead: None,
        flare_elevator: None,
        roll_rudder: None,
        rotate_elevator: None,
        arrest_roll: None,
        ground_plant: None,
        rollout_steer: None,
        speed_governor: None,
        cabin_air: None,
        pressurised: Some(false),
        park_brake_valve_set: None,
        pitot_heat: true,
        pitot_heat_dataref: None,
        advisor: vec![],
        // Throttle/mixture/start write universal datarefs (throttle_ratio_all,
        // mixture_ratio_all, the starter command) — confirmed against the
        // rustdecks SR22 deck. (start_engine sets battery 1 + alternator 1 only;
        // the SR22's second battery/alternator need the dual-electrical tool.)
        engine: true,
        throttle: false,
        prop: None,
        turbine_start: false,
        auto_prep: false,
        scripted_auto_start: false,
        fuel_cutoff_levers: Vec::new(),
        fuel_cutoff_value: 0.0,
        autothrottle: false,
        speedbrake: false,
        // Dual battery/alternator + avionics — datarefs/commands verified against
        // the rustdecks SR22 deck.
        electrical: {
            let e = |key: &str, spoken: &str, on: &[&str], off: &[&str], rb: &[&str]| {
                ElectricalSystem {
                    write_datarefs: Vec::new(),
                    toggles: Vec::new(),
                    guards: Vec::new(),
                    key: key.into(),
                    spoken: spoken.into(),
                    on_commands: on.iter().map(|s| s.to_string()).collect(),
                    off_commands: off.iter().map(|s| s.to_string()).collect(),
                    readback: rb.iter().map(|s| s.to_string()).collect(),
                }
            };
            vec![
                e(
                    "battery1",
                    "battery one",
                    &["sim/electrical/battery_1_on"],
                    &["sim/electrical/battery_1_off"],
                    &["sim/cockpit2/electrical/battery_on[0]"],
                ),
                e(
                    "battery2",
                    "battery two",
                    &["sim/electrical/battery_2_on"],
                    &["sim/electrical/battery_2_off"],
                    &["sim/cockpit2/electrical/battery_on[1]"],
                ),
                e(
                    "alternator1",
                    "alternator one",
                    &["sim/electrical/generator_1_on"],
                    &["sim/electrical/generator_1_off"],
                    &["sim/cockpit2/electrical/generator_on[0]"],
                ),
                e(
                    "alternator2",
                    "alternator two",
                    &["sim/electrical/generator_2_on"],
                    &["sim/electrical/generator_2_off"],
                    &["sim/cockpit2/electrical/generator_on[1]"],
                ),
                e(
                    "batteries",
                    "both batteries",
                    &["sim/electrical/battery_1_on", "sim/electrical/battery_2_on"],
                    &[
                        "sim/electrical/battery_1_off",
                        "sim/electrical/battery_2_off",
                    ],
                    &[
                        "sim/cockpit2/electrical/battery_on[0]",
                        "sim/cockpit2/electrical/battery_on[1]",
                    ],
                ),
                e(
                    "alternators",
                    "both alternators",
                    &[
                        "sim/electrical/generator_1_on",
                        "sim/electrical/generator_2_on",
                    ],
                    &[
                        "sim/electrical/generator_1_off",
                        "sim/electrical/generator_2_off",
                    ],
                    &[
                        "sim/cockpit2/electrical/generator_on[0]",
                        "sim/cockpit2/electrical/generator_on[1]",
                    ],
                ),
                e(
                    "avionics",
                    "avionics master",
                    &["sim/systems/avionics_on"],
                    &["sim/systems/avionics_off"],
                    &["sim/cockpit/electrical/avionics_on"],
                ),
            ]
        },
        // Key-start ignition: set_magnetos writes the universal ignition_key
        // (0 off/1 R/2 L/3 both/4 start) and start_engine engages starter 1
        // (sim/starters/engage_starter_1, the command the rustdecks deck uses).
        magnetos: true,
        carb_heat: false, // fuel-injected — no carb heat
        // Boost pump: set_fuel_pump writes sim/cockpit2/engine/actuators/
        // fuel_pump_on[0] — the dataref the rustdecks deck drives for BOOST
        // (not laminar/sr22/switch/boost_pump, which an earlier scaffold guessed).
        fuel_pump: true,
        fuel_pump_commands: None,
        fuel_tank_pumps_off_when_empty: Vec::new(),
        reverse_thrust: false,
        // OFF / LEFT / RIGHT — no "both" (low-wing twin tanks, fed one at a
        // time). Selector commands verified against the rustdecks deck. The
        // standard sim/cockpit2/fuel/fuel_tank_selector (0=off/1=left/2=right) is
        // the readback the GoofyJP checklist trusts, so it both confirms a write
        // and lets the checklist decode the file's `fuel_tank_selector:N` items.
        fuel_selector: Some(FuelSelectorProfile {
            readback: Some("sim/cockpit2/fuel/fuel_tank_selector".into()),
            positions: vec![
                fuel_pos("off", "off", "laminar/sr22/fuel_sel_left_off", Some(0.0)),
                fuel_pos(
                    "left",
                    "left",
                    "laminar/sr22/fuel_sel_left_select",
                    Some(1.0),
                ),
                fuel_pos(
                    "right",
                    "right",
                    "laminar/sr22/fuel_sel_right_select",
                    Some(2.0),
                ),
            ],
        }),
        directional_gyro: false, // G1000 glass — no vacuum DG
        flood_lights: false,
        // set_pitch_trim writes the universal elevator_trim; "takeoff" centres it
        // (within the file's T/O band).
        pitch_trim: true,
        // Flaps 0 / 50% / 100% (Vfe 150 / 110 KIAS); set_flaps writes the
        // universal flap_ratio. Detent ratios: verify.
        flaps: Some(FlapProfile {
            detents: vec![
                detent("up", "up", 0.0, None),
                detent("50", "fifty percent", 0.5, Some(150.0)),
                detent("full", "full", 1.0, Some(110.0)),
            ],
            takeoff_ratio: None,
        }),
        // SR22 ~92 gal usable, ~17 gal/hr cruise (~46 kg/hr). Verify.
        fuel: Some(FuelProfile {
            reserve_caution_min: 45.0,
            reserve_warning_min: 30.0,
            kg_per_gal: None,
            spoken_unit: None,
            cruise_burn_kg_hr: 46.0,
        }),
        // VS1 (clean) ~76 KIAS; VS0 (landing) ~61.
        // SR22 clean stall (Vs) 70 KIAS, per the X-Plane SR22 POM.
        vs_clean_kts: Some(70.0),
        // SR22 Vne (never-exceed) 204 KIAS.
        vne_kts: Some(204.0),
        vmo_high: None,
        vlo_kts: None,
        gear_warning: None,
        taxi: None,
        throttle_dataref: None,
        tail_strike_deg: None,
        // A light single runs out of elevator before it runs out of tail
        // clearance, and only ever needs the rotation held BACK.
        pitch_authority_down: None,
        climb_pitch_max_deg: None,
        circuit_final_lead_s: None,
        // Reference figures from the X-Plane Cirrus SR22 Pilot's Operating Manual
        // (the modeled aircraft is the turbo TSIO-550-K, ceiling 25,000 ft).
        reference: Some(ReferenceProfile {
            kind: Some("single-engine aircraft".into()),
            // A piston does not step-climb: it goes as high as it can and
            // stays. No optimum to advise, so nothing is said.
            optimum_alt_ft: None,
            // ...and the GA vertical rates are the shared default already.
            enroute_climb_fpm: None,
            enroute_descent_fpm: None,
            optimum_alt_at_kg: None,
            optimum_alt_ft_per_1000kg: None,
            vno_kts: Some(178.0),
            va_kts: Some(133.0),
            vx_kts: Some(88.0),
            vy_kts: Some(101.0),
            initial_climb_kts: None,
            acceleration_height_agl_ft: None,
            vyse_kts: None,
            vmca_kts: None,
            vr_kts: Some(73.0),
            rotate_rate_deg_s: None,
            best_glide_kts: Some(88.0),
            vs0_kts: Some(60.0),
            approach_kts: None,
            max_crosswind_kts: Some(21.0),
            cruise_kts: Some(180.0), // normal cruise (manual lists 213 KTAS max)
            cruise_ias_kts: None,    // no full-auto for the SR22 yet
            best_climb_fpm: Some(1200.0),
            max_gross_lb: Some(3600.0),
            empty_weight_lb: Some(2354.0),
            useful_load_lb: Some(1260.0),
            service_ceiling_ft: Some(25000.0),
            fuel_capacity_gal: Some(92.0),
            fuel_useable_gal: None,
            endurance_hours: Some(5.0), // 92 gal / ~18 gph
            takeoff_distance_ft: None,
            takeoff_roll_ft: None,
            landing_distance_ft: None,
            max_rpm: Some(2700.0),
            oil_temp_min_f: None,
            oil_temp_max_f: None,
            oil_press_min_psi: None,
            oil_press_max_psi: None,
        }),
        checklist: Some("checklists/cirrus_sr22.txt".into()),
        // Lights — datarefs verified against the rustdecks SR22 deck; universal
        // on/off commands.
        lights: vec![
            light(
                "nav",
                "nav lights",
                "sim/cockpit2/switches/navigation_lights_on",
                "sim/lights/nav_lights_on",
                "sim/lights/nav_lights_off",
            ),
            light(
                "strobe",
                "strobe lights",
                "sim/cockpit2/switches/strobe_lights_on",
                "sim/lights/strobe_lights_on",
                "sim/lights/strobe_lights_off",
            ),
            light(
                "landing",
                "landing light",
                "sim/cockpit2/switches/landing_lights_switch[0]",
                "sim/lights/landing_lights_on",
                "sim/lights/landing_lights_off",
            ),
        ],
        // Acts on universal-dataref systems plus the now-wired boost pump,
        // key-start ignition, pitch trim and the off/left/right fuel selector
        // (decoded from the profile's position values). The dual battery/
        // alternator stay call-out — their file conditions are compound `&&`.
        checklist_map: {
            let m = |s: &str, a: ChecklistAction| ChecklistMapEntry {
                match_: s.into(),
                action: a,
            };
            use ChecklistAction::*;
            vec![
                m("parkbrake", ParkingBrake),
                m("transponder_mode", Transponder),
                m("pitot_heat_on", PitotHeat),
                m("throttle_ratio", Throttle),
                m("ENGN_mixt", Mixture),
                m("flap_handle", Flaps),
                m("avionics_on", Electrical("avionics".into())), // batteries are compound → call-out
                m("nav_lights", Light("nav".into())),
                m("strobe_lights", Light("strobe".into())),
                m("landing_lights", Light("landing".into())),
                m("boost_pump", FuelPump), // laminar/sr22/switch/boost_pump → set_fuel_pump
                m("ignition_key", Ignition), // actuators/ignition_key: 4=start, 0=off
                m("ignition_on", Ignition), // mag test: 1=R, 2=L, 3=both
                m("elevator_trim", PitchTrim), // "set T/O" → centre (not ail_trim)
                m("fuel_tank_selector", FuelSelector), // 0=off/1=left/2=right
            ]
        },
        checklist_flow: Default::default(),
        altimeter: None,
        anti_ice: None,
        autoflight: None,
    }
}

#[cfg(test)]
mod generic_profile_tests;

/// Every hardcoded `sim/...` name in the workspace, checked against the
/// running install's own `DataRefs.txt` and `Commands.txt`. The live probe
/// walks the PROFILE's names on every airframe (`profile_references`); the
/// Rust tools' constants were never probed, and two of them were misspelt for
/// months (`vvi_dial_fpm` in autoland, `compass_heading_deg_magnetic` in the
/// stall recovery, found 2026-09-15), each read failing silently into a
/// fallback that impersonated a measurement. Needs the sim on disk:
/// `cargo test -- --ignored real_install_names`.
#[path = "aircraft/real_install_names_tests.rs"]
#[cfg(test)]
mod real_install_names;

/// Does the write LAND? The companion to `real_install_names`, which only
/// asks whether a name resolves.
///
/// A study-level add-on owns much of its own cockpit: it REPLACES stock
/// commands (our fire reaches its handler — fine), and it WRITES stock
/// datarefs every frame (our write is overwritten a frame later — a silent
/// no-op). Both resolve. Both probe clean. One of them does nothing.
///
/// Three failures in one evening on the A330 (2026-09-20) were this shape:
/// a name declared as a dataref that was really a command, an array element
/// written by subscript, and an `_on` command the aircraft leaves stock while
/// replacing only its `_toggle`. Every one passed the existence probe.
///
/// Needs the aircraft on disk:
/// `cargo test -- --ignored real_install_writes`.
#[path = "aircraft/real_install_writes_tests.rs"]
#[cfg(test)]
mod real_install_writes;

#[cfg(test)]
mod vmo_tests;

/// The rules unified by the refactor audit (2026-10-04, findings #2, #10,
/// #12, #13): one home each, pinned here.
#[cfg(test)]
mod one_rule_tests;
