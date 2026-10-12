//! Profile schema, the flight-law sections: flare power and arrest lead, cabin
//! air, the altimeter, anti-ice, autoflight and its MCP, the autopilot's
//! landing and FMA, the flare elevator and model, the arrest roll, the ground
//! plant, the rollout steer, the speed governor, the final track, the rotate
//! elevator and the roll rudder.

use super::*;

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
    /// The FLOAT's retard, lever per second (absent: `retard_max_per_s`).
    /// A float has no sink to rebuild too hard: the thrust holding it up is
    /// what lands it. The timed retard's cap (the 737's 0.04, for the sink
    /// it rebuilds in the last second) walked 0.33 to 0.25 through LXGB's
    /// 2 s float at 10 to 4.5 ft (2026-10-06), where the touchdown came
    /// 631 m past the threshold. The roundout's float takes the same rate.
    #[serde(default)]
    pub retard_float_per_s: Option<f64>,
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
    /// The knob's own commands, where the add-on ignores the stock
    /// `sim/instruments/barometer_up/down` (the Zibo: one press of its
    /// `laminar/B738/pilot/barometer_up` is a hundredth, the stock one does
    /// nothing, and both dials are read-only, probed 2026-10-06). `None`
    /// keeps the stock knob.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub knob: Option<BaroKnob>,
}

/// Each side's altimeter knob, up and down ([`AltimeterProfile::knob`]).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct BaroKnob {
    pub up: String,
    pub down: String,
    pub copilot_up: String,
    pub copilot_down: String,
}

pub(super) fn one_press() -> usize {
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
    /// The panel's own buttons, where the airframe has them
    /// (`autoflight::Panel`): CMD A, HDG SEL, V/S and ALT HLD. `None` is the
    /// stock command. The 737's xlua buttons fire the stock modes and keep
    /// its MCP lamps and FMA honest; CMD B is never named here (it dropped CMD A
    /// live, LEMG 2026-10-05): only as `autopilot_lands.second_cmd`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub engage_command: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub heading_command: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vs_command: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub altitude_hold_command: Option<String>,
    /// What takes an ENGAGED autothrottle out to ARM when a loop flies the
    /// levers; `None` is the stock `autothrottle_off` ("all modes off,
    /// remain armed").
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub autothrottle_off_command: Option<String>,
    /// The panel's own LNAV button (a toggle) and its lamp, 1 engaged: the
    /// enroute follower hands its lateral to the FMS on an airframe that
    /// declares them, its heading bug the watched fallback (#227). The 737's
    /// `lnav_press` fires the stock GPSS (only with CMD in); `autopilot_state`
    /// carries it as bit 524288 (`autoflight::Modes::LNAV`). Both or neither;
    /// `None` keeps the follower on its heading bug.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lnav_command: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lnav_status: Option<String>,
    /// This MCP flies a V/S only TOWARD its altitude window (Boeing logic):
    /// autoland parks the window at the field (`autoflight::WindowRule::
    /// AtField`), toward every descent of the approach. Off: parked above
    /// (the LEAX rule). Replaces `alt_dial_park_field` (89fe7a6).
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub vs_toward_window: bool,
    /// The panel's own CMD lamp (1 engaged): the servos' read where the
    /// MCP does not keep the stock `servos_on` (the Zibo's zibomod flies
    /// its own autopilot; probed 2026-10-07, `autopilot_state` held 132
    /// through every MCP press). `None` reads `servos_on`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub engage_status: Option<String>,
    /// The panel's own disconnect (the yoke button, never a latching bar:
    /// a bar left down blocks every later engage). `None` is the stock
    /// `servos_off_any`; on a declared panel it also stands in for the
    /// servos-and-FD disconnect.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub disconnect_command: Option<String>,
    /// The panel's own LVL CHG button (a toggle). `None` is the stock.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub level_change_command: Option<String>,
    /// The mode lamps, 1 lit: declared, the owner builds `autoflight::Modes`
    /// from them instead of `autopilot_state`, and every mode status a loop
    /// reads is the lamp's (`autoflight::read_status`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lamps: Option<ModeLamps>,
    /// The MCP windows, where they are not the stock dials
    /// (`autoflight::Dial`): written, or stepped on their knob commands,
    /// and read back from the window itself.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub windows: Option<McpWindows>,
    /// The A/T arm switch and the flight director switches: each a toggle,
    /// read on its position before it is fired, read back after.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub autothrottle_arm: Option<PanelSwitch>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub flight_director: Option<PanelSwitch>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub flight_director_fo: Option<PanelSwitch>,
    /// The panel's TO/GA: with the A/T armed it sets the takeoff thrust
    /// itself. The takeoff still flies its own levers (owed:
    /// `autoflight::Panel::toga`); a go-around from a landing the autopilot
    /// flies (`autopilot_lands`) presses it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub toga_command: Option<String>,
    /// The A/T SPEED button (a toggle, read on the `lamps.speed` lamp
    /// before it is pressed): the autothrottle holds the MCP speed window.
    /// Pressed only by a landing the autopilot flies (`autopilot_lands`
    /// `autothrottle`). `None` is the stock `autothrottle_on`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub speed_command: Option<String>,
}

/// The mode lamps of a panel whose MCP is not the stock autopilot's
/// (`AutoflightProfile::lamps`): each the lamp's dataref, 1 lit. `None`
/// fields are modes this panel's lamps do not show.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ModeLamps {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub heading: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vs: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub altitude_hold: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub level_change: Option<String>,
    /// APP: lit armed OR captured (the lamp does not tell them apart).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub approach: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vorloc: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vnav: Option<String>,
    /// The autothrottle's SPEED and N1 lamps: either lit is an ENGAGED
    /// autothrottle for the thrust's one owner.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub speed: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub n1: Option<String>,
}

/// One MCP window (`AutoflightProfile::windows`): `write` is the dataref a
/// value is written to; or `up`/`down` are the knob's commands, each press
/// `step`. `readback` is where the window is read (defaults to `write`).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct McpWindow {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub write: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub up: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub down: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub step: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub readback: Option<String>,
}

/// The MCP windows a panel declares (`AutoflightProfile::windows`).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct McpWindows {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub heading: Option<McpWindow>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub altitude: Option<McpWindow>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub airspeed: Option<McpWindow>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vs: Option<McpWindow>,
    /// The MCP course windows, captain's and first officer's: the ILS course
    /// where the MCP drives the OBS (the Zibo's `course_pilot` is the NAV1
    /// OBS's source until the localiser captures). `None`: the stock OBS.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub course: Option<McpWindow>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub course_fo: Option<McpWindow>,
}

/// The autopilot LANDS this airframe on an ILS (`autopilot_lands`, the
/// `ap_lands` module): APP armed, the airframe's own capture, the second
/// channel engaged after the glideslope, its own flare, retard and rollout,
/// the crew monitoring and taking it back loudly at the declared gates.
/// Absent: autoland flies its own law to the ground, as it always has.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct AutopilotLands {
    /// The second channel's button and lamp (CMD B on a 737): pressed after
    /// the glideslope captures, read lit with CMD A for a dual approach.
    pub second_cmd: PanelSwitch,
    /// The flight mode annunciations, read every tick on this path and
    /// blackboxed raw (`autoland` `coupled_tick`).
    pub fma: FmaNames,
    /// What the FMA numbers mean ([`FmaCodes`]).
    #[serde(default)]
    pub codes: FmaCodes,
    /// Tune NAV2 and set the first officer's course too: a dual-channel
    /// approach reads both receivers.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub nav2: bool,
    /// Engage the A/T in SPEED on the MCP window for the approach (the arm
    /// switch, the window, the SPEED button, each read back): the airframe's
    /// own retard at the flare. Off, or not taken: our thrust law flies.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub autothrottle: bool,
    /// Radio height by which both halves are captured and both channels
    /// engaged, else our own law flies the approach (said).
    #[serde(default = "default_dual_by_agl_ft")]
    pub dual_by_agl_ft: f64,
    /// Radio height by which the landing is armed (FLARE armed, LAND 2/3),
    /// else a go-around with height in hand (said).
    #[serde(default = "default_land_by_agl_ft")]
    pub land_by_agl_ft: f64,
    /// Below this radio height a lost autoland is OUR flare's, never a
    /// go-around from the wheels; above it, a go-around.
    #[serde(default = "default_own_flare_below_agl_ft")]
    pub own_flare_below_agl_ft: f64,
    /// The rollout is the autopilot's down to this groundspeed, then it is
    /// disconnected and our rollout steers.
    #[serde(default = "default_ap_disconnect_kts")]
    pub disconnect_kts: f64,
    /// ...or sooner, past this much cross-track on the rollout.
    #[serde(default = "default_rollout_xtk_limit_m")]
    pub rollout_xtk_limit_m: f64,
}

pub(super) fn default_dual_by_agl_ft() -> f64 {
    1500.0
}
pub(super) fn default_land_by_agl_ft() -> f64 {
    1000.0
}
pub(super) fn default_own_flare_below_agl_ft() -> f64 {
    200.0
}
pub(super) fn default_ap_disconnect_kts() -> f64 {
    30.0
}
pub(super) fn default_rollout_xtk_limit_m() -> f64 {
    10.0
}

/// The FMA datarefs an `autopilot_lands` path reads (an element as
/// `name[i]`). `None`: that column is not read, and nothing is decided on it.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct FmaNames {
    /// The roll mode engaged (LOC, ROLLOUT).
    pub roll: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub roll_armed: Option<String>,
    /// The pitch mode engaged (G/S, FLARE).
    pub pitch: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pitch_armed: Option<String>,
    /// The autothrottle mode (MCP SPD, RETARD).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub thrust: Option<String>,
    /// The landing annunciation (LAND 2, LAND 3).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub land: Option<String>,
    /// Read and blackboxed only, for the probe flight to decode.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub log: Vec<String>,
}

/// The FMA values that mean each mode, per column ([`FmaNames`]). Empty:
/// that mode is never read as shown.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct FmaCodes {
    /// `roll`: the localiser captured; the rollout.
    #[serde(default)]
    pub loc: Vec<f64>,
    #[serde(default)]
    pub rollout: Vec<f64>,
    /// `pitch`: the glideslope captured; the flare; TO/GA.
    #[serde(default)]
    pub gs: Vec<f64>,
    #[serde(default)]
    pub flare: Vec<f64>,
    #[serde(default)]
    pub toga: Vec<f64>,
    /// `pitch_armed`: the flare armed.
    #[serde(default)]
    pub flare_armed: Vec<f64>,
    /// `thrust`: the A/T holding the MCP speed; retarding.
    #[serde(default)]
    pub speed: Vec<f64>,
    #[serde(default)]
    pub retard: Vec<f64>,
    /// `land`: a landing annunciated (LAND 2 or LAND 3).
    #[serde(default)]
    pub land: Vec<f64>,
}

/// A toggle switch on the panel and the position it reads (1 on).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PanelSwitch {
    pub command: String,
    pub position: String,
}

/// The attitude flare's elevator limits for this airframe
/// (`autoland::attitude_flare`), and the pull its ground effect takes. An
/// airframe that declares none flies the defaults.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FlareElevatorProfile {
    /// Most pull the flare may command.
    #[serde(default = "default_flare_max")]
    pub max: f64,
    /// Most push, as a positive number.
    #[serde(default = "default_max_push")]
    pub max_push: f64,
    /// The extra pull this airframe's ground effect takes at the wheels,
    /// MEASURED on its own flares (`autoland::attitude_flare` feeds it
    /// forward along the ground effect's shape); 0 for an airframe whose
    /// flare finds it in time (the 172). The SR22's low wing took 0.33 at
    /// 5 ft to 0.61 at the wheels with the nose held flat (LEMG 30,
    /// 1791793670, 1791793963).
    #[serde(default)]
    pub ground_pull: f64,
}

impl Default for FlareElevatorProfile {
    fn default() -> Self {
        Self {
            max: default_flare_max(),
            max_push: default_max_push(),
            ground_pull: 0.0,
        }
    }
}

pub(super) fn default_flare_max() -> f64 {
    0.35
}

pub(super) fn default_max_push() -> f64 {
    0.10
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
    /// Flying clear of ground effect, the spring per degree of the wing's
    /// angle of attack, reached from `per_deg` as `air_base` is; `None` keeps
    /// `per_deg` flying too (the SR22). On the wheels the spring is the main
    /// gear's as much as the tail's; flying it is the wing's own stiffness,
    /// and on a jet the two differ threefold (the Zibo 737: 0.077 on the
    /// wheels, 0.225 flying, LEMG 30 2026-10-07).
    #[serde(default)]
    pub air_per_deg: Option<f64>,
    /// The flight path the spring is taken off by, from the height's own
    /// rate instead of the flight model's vertical speed (`vh_ind_fpm`).
    /// That vertical speed trails the height by about 1.3 s on the Zibo
    /// (432 fpm read at 13 ft with the height climbing 1,145 fpm, LEMG 30
    /// 2026-10-07): against a stiff spring flying, 3 degrees of path behind
    /// is 0.6 of column too much pull. `false` (the default) reads the
    /// vertical speed, as ever.
    #[serde(default)]
    pub path_from_height: bool,
}

impl RotateElevatorProfile {
    /// Used only when every number is a sane one. A stiff spring flying is
    /// held against a large nose-up base (the Zibo's -1.5 at 0.225 per
    /// degree), so the base may go to -2 where `air_per_deg` is declared.
    pub fn valid(self) -> Option<Self> {
        let base_min = if self.air_per_deg.is_some() { -2.0 } else { -0.5 };
        ((30.0..=300.0).contains(&self.ref_kts)
            && (1.0..=200.0).contains(&self.deg_s_per_unit)
            && (0.0..=0.6).contains(&self.breakout)
            && (0.0..=0.1).contains(&self.per_deg)
            && self.air_per_deg.is_none_or(|a| (0.0..=0.4).contains(&a))
            && (base_min..=0.6).contains(&self.air_base)
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
    /// The dead time from the steer to the yaw, seconds, where it is long
    /// enough to fly round: the law then steers on the heading the steer
    /// already written WILL make (`RollSteer`'s predictor), not on the one
    /// read. `None` steers on the reading, as every airframe did before it
    /// (their 0.3 s is inside the lead). The Zibo's pedals reach the yaw
    /// through 1.0 s (LEMG 30, three rolls of 2026-10-07): steered on the
    /// reading, the scheduled law rang 14.6 m off the centreline through the
    /// fitted plant.
    #[serde(default)]
    pub dead_s: Option<f64>,
    /// The first-order lag after that dead time, seconds (0 when `None`):
    /// how long the yaw read takes to answer a steer that has arrived.
    #[serde(default)]
    pub lag_s: Option<f64>,
    /// How far the ground TRACK lags the heading on the wheels, seconds:
    /// the tyres turn the aircraft's path after its nose. Where declared, the
    /// centreline cut (`roll_steer::RollCut`) leads on the track the heading
    /// is making (heading plus the drift) instead of the cross-track rate
    /// read, which is that heading this much late, and the drift is read
    /// against the heading delayed the same way. `None` leads on the rate
    /// read, as every airframe did before it. The Zibo's track follows its
    /// heading through 0.5 to 1.0 s at 70 to 110 KIAS and 1 to 2 s toward
    /// Vr (LEMG 30, five rolls of 2026-10-07); led on the rate read, the cut
    /// saw its own correction a second or two late, crossed the centreline
    /// at the rotation still moving 1 to 2 m/s, and the track carried that
    /// to the liftoff 5 to 11 m off.
    #[serde(default)]
    pub track_lag_s: Option<f64>,
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
        let dead_s = self.dead_s.filter(|d| (0.05..=3.0).contains(d));
        let lag_s = self.lag_s.filter(|l| (0.0..=2.0).contains(l));
        let track_lag_s = self.track_lag_s.filter(|l| (0.05..=3.0).contains(l));
        ((0.5..=100.0).contains(&self.yaw_dps) && (20.0..=200.0).contains(&self.at_kts))
            .then_some(Self {
                power_steer,
                power_steer_to_kts,
                power_steer_per_kftlb,
                yaw_power,
                close_per_s,
                trim_per_s,
                dead_s,
                lag_s,
                track_lag_s,
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
        // Up to the stop: the attitude flare holds the nose angle, so
        // its limit is the elevator's travel, not a balloon guard. At
        // 0.6 the SR22's 0.8 dropped the whole block and the old law
        // flew it in silence (452 fpm, LEMG 13, 1791792579).
        ((0.05..=1.0).contains(&self.max) && (0.0..=0.3).contains(&self.max_push)).then_some(self)
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
