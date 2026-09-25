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
    pub reduction_per_second: f64,
    /// Pause the reduction when measured descent exceeds this positive rate.
    pub hold_sink_fpm: f64,
    /// Include the recent sink trend to avoid cutting into a renewed descent.
    pub lead_seconds: f64,
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
/// is its output), and its oxygen button is a toggle command in front of a
/// stock valve that can sit shut behind a set button. EDLP-LOWI
/// (2026-09-24): full auto resumed with the engine already running, the bleed
/// was never switched, the cabin followed the aircraft to 24,000 ft and the
/// pilot blacked out; the oxygen button read on and the valve was shut.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CabinAirProfile {
    /// Fired to put the bleed ON.
    pub bleed_on_command: String,
    /// Reads at or above 0.5 with the bleed on.
    pub bleed_read: String,
    /// The crew oxygen: a TOGGLE (read first, fired only when off).
    #[serde(default)]
    pub oxygen_toggle: Option<String>,
    /// Reads at or above 1 with the oxygen on.
    #[serde(default)]
    pub oxygen_read: Option<String>,
    /// A valve behind the button, written 1.
    #[serde(default)]
    pub oxygen_valve: Option<String>,
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
}

fn default_max_push() -> f64 {
    0.10
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
        .then_some(self)
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
    let path = format!("profiles/{id}.yaml");
    match std::fs::read_to_string(&path) {
        Ok(text) => match serde_yaml::from_str::<AircraftProfile>(&text) {
            Ok(profile) => {
                tracing::info!(path = %path, "loaded aircraft profile from YAML");
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

/// The highest speed the flight governor may be asked to hold, from the
/// airframe's Vne/Vmo. `None` = the profile declares no Vne.
///
/// The governor's sanity cap was a flat 250 kt, which is a GA number: the
/// A330's cruise governor was armed at its profile's 290 and held 250 with
/// nothing said, and a spoken "hold two eighty" in any jet was quietly 250
/// (found on the profile review, 2026-09-20). The cap is now the airframe's
/// own, `SPEED_CEILING_MARGIN_KTS` under Vne, republished per swap like the
/// floor. Below 10,000 ft the 250 kt rule still applies on top of it
/// (`speedhold::BELOW_10K_LIMIT_KTS`); that is airspace, not airframe.
static SPEED_CEILING_KTS: std::sync::Mutex<Option<f64>> = std::sync::Mutex::new(None);
pub const SPEED_CEILING_MARGIN_KTS: f64 = 15.0;

/// Publish the airframe's speed ceiling from its Vne. Called on EVERY profile swap.
pub fn publish_speed_ceiling(vne_kts: Option<f64>) {
    *SPEED_CEILING_KTS.lock().unwrap_or_else(|e| e.into_inner()) = vne_kts
        .filter(|v| *v > 0.0)
        .map(|v| (v - SPEED_CEILING_MARGIN_KTS).round());
}

/// The live airframe's speed ceiling, kts, if its profile declares a Vne.
pub fn speed_ceiling_kts() -> Option<f64> {
    *SPEED_CEILING_KTS.lock().unwrap_or_else(|e| e.into_inner())
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

/// The next same-direction level up, if cruising `cruise_ft` is at least a
/// full step below `optimum_ft`. `None` when we are at or near optimum, or
/// above it — a jet above optimum is a different problem and not this one's
/// to solve. The step is always exactly one level: a climb is a request to
/// ATC, and asking for two at once is how you get neither.
pub fn step_climb_ft(cruise_ft: f64, optimum_ft: f64) -> Option<f64> {
    (optimum_ft - cruise_ft >= STEP_CLIMB_FT).then(|| cruise_ft + STEP_CLIMB_FT)
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
        }];
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
        r.vr_kts = Some((vs * 1.1).round());
        let by_vso = vso.map(|v| v * 1.3).unwrap_or(0.0);
        r.approach_kts = Some(by_vso.max(vs * 1.2).round());
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
mod tests {
    use super::*;

    /// Every test that publishes the fuel unit holds this. `FUEL_UNIT` is
    /// process-wide, cargo runs these threads in parallel, and without it
    /// they overwrite each other's airframe mid-assertion — the same trap
    /// the station's `SIDE_STATE_LOCK` exists for.
    static FUEL_TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    /// An array element is never written by name-with-a-subscript.
    ///
    /// `set_float("sim/.../eng_master[0]", 1.0)` names a dataref that does not
    /// exist — the API answers "not found" and the caller, using `let _ =`,
    /// hears nothing. On the A330 the APU came up, the bleed came on, all
    /// eight fuel pumps came on, and both master switches sat at zero while
    /// the start waited two minutes for an idle that was never coming
    /// (2026-09-20). `set_float_array_elem` reads, patches the index and
    /// writes the whole array back.
    ///
    /// Cheap to write by accident and invisible when you do, which is the
    /// definition of a thing that belongs in a test rather than a comment.
    #[test]
    fn no_array_element_is_written_by_subscripted_name() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        let mut files = Vec::new();
        fn walk(dir: &std::path::Path, out: &mut Vec<std::path::PathBuf>) {
            for e in std::fs::read_dir(dir).unwrap().flatten() {
                let p = e.path();
                if p.is_dir() {
                    walk(&p, out);
                } else if p.extension().is_some_and(|x| x == "rs") {
                    out.push(p);
                }
            }
        }
        walk(&root, &mut files);
        let mut offenders = Vec::new();
        for f in &files {
            let text = std::fs::read_to_string(f).unwrap();
            for (i, line) in text.lines().enumerate() {
                let t = line.trim_start();
                // A set_* whose name argument is a format! building a
                // subscript. The array writers take an index instead.
                let writes_scalar = t.contains("set_float(") || t.contains("set_int(");
                if writes_scalar && t.contains("format!(") && t.contains('[') && t.contains("{idx")
                {
                    offenders.push(format!("{}:{}", f.display(), i + 1));
                }
            }
        }
        assert!(
            offenders.is_empty(),
            "use set_float_array_elem / set_int_array_elem for an element, not \
             a subscripted name — the API does not know that dataref and the \
             write is lost silently: {offenders:?}"
        );
        assert!(files.len() > 10, "the walk found the tree: {}", files.len());
    }

    /// EVERY speed and EVERY rate in the flight laws, classified once.
    ///
    /// This is the rule CLAUDE.md already states — "all automation numbers
    /// are per-airframe, defaulting to the old GA figures when a profile is
    /// silent" — with something behind it at last. The rule was written
    /// before 2026-09-20 and on that date a 737 still flew a 0.8 nm capture
    /// radius, a 700 fpm descent, a 0.10 nose-down limit, avgas density and
    /// an 800 fpm go-around, because prose does not fail a build.
    ///
    /// A constant named in knots, feet per minute or degrees per second is
    /// a number the AIRCRAFT achieves, and those are the ones that were
    /// wrong. Each is listed here with what it is, and anything new or
    /// renamed fails until someone decides which it is. That decision is
    /// cheap at the moment of writing and expensive at 34,000 feet.
    #[test]
    fn every_speed_and_rate_in_the_laws_is_classified() {
        // (module, NAME) -> why it may be a shared constant.
        const REVIEWED: &[(&str, &str, &str)] = &[
            ("procedure", "NOT_CLIMBING_FPM", "a rate that is not a climb, any airframe"),
            ("procedure", "ARRIVAL_DECEL_KTS_S", "planning slow-down for a STAR speed limit: a jet's descent rate, so a slower airframe only starts early"),
            ("autotakeoff", "ROLL_BRAKE_FULL_KTS", "brake-steer fade band, groundspeed; only brake_steer airframes"),
            ("autotakeoff", "ROLL_BRAKE_OFF_KTS", "brake-steer fade band, groundspeed; only brake_steer airframes"),
            ("speedhold", "IDLE_FAST_KTS", "margin over the pilot's own target, not an airframe speed"),
            // --- Properties of the SIM or the SENSORS, not the airframe ---
            (
                "autotakeoff",
                "STAGNANT_GS_KTS",
                "the ASI is dead below ~20 kt on any airframe",
            ),
            (
                "autotakeoff",
                "TRIM_MIN_GS_KTS",
                "integrator arming: below this nothing steers yet",
            ),
            (
                "autotaxi",
                "STRAIGHTEN_HOLD_KTS",
                "walking pace; a stop is a stop",
            ),
            (
                "autotaxi",
                "CREEP_KTS",
                "the speed below which the sim reads as stopped",
            ),
            (
                "speedhold",
                "SURGE_KTS_S",
                "acceleration that means a surge, not a schedule",
            ),
            (
                "speedhold",
                "SETTLING_KTS_S",
                "deceleration that means settling, likewise",
            ),
            // --- Rules and ceilings that bind every aircraft ---
            (
                "speedhold",
                "TAXI_MAX_KTS",
                "a safety ceiling on taxi, never a target",
            ),
            (
                "autoland",
                "VACATE_HANDOFF_KTS",
                "taxi speed off the runway, not a flight speed",
            ),
            // --- Aerodynamic ramps: airflow over a surface, not performance ---
            (
                "autotakeoff",
                "LOW_SPEED_FULL_KTS",
                "rudder authority builds with airflow",
            ),
            (
                "autotakeoff",
                "XTK_GAIN_REF_KTS",
                "not a speed the aircraft flies: the pace the centreline gain \
                 is QUOTED at, above which it scales by ref/gs to hold the \
                 closure time - the same geometry on every airframe",
            ),
            (
                "autotakeoff",
                "POWER_LEAD_FADE_KTS",
                "throttle lead fades as the surface bites",
            ),
            (
                "autotakeoff",
                "THROTTLE_HOLD_MAX_GS_KTS",
                "holds power while the nose swings",
            ),
            (
                "autotaxi",
                "BRAKE_STEER_MIN_KTS",
                "differential brake bites above walking pace",
            ),
            (
                "autotaxi",
                "BRAKE_STEER_FULL_KTS",
                "...and is fully effective above this",
            ),
            (
                "autotaxi",
                "FREE_NOSE_MIN_KTS",
                "a nose with no steering of its own (from the .acf) is brake-steered from walking pace",
            ),
            (
                "autotaxi",
                "FREE_NOSE_FULL_KTS",
                "...and fully by this",
            ),
            // --- Handling conventions, flown the same in anything ---
            (
                "autotakeoff",
                "ROTATE_RATE_DEG_S",
                "2-3 deg/s is how a rotation is flown",
            ),
            // --- GA DEFAULTS with a per-airframe override already in place ---
            (
                "autotaxi",
                "STRAIGHT_KTS",
                "default: Pace::from_profile overrides",
            ),
            (
                "autotaxi",
                "TURN_KTS",
                "default: Pace::from_profile overrides",
            ),
            (
                "autotaxi",
                "SHARP_KTS",
                "default: Pace::from_profile overrides",
            ),
            (
                "procedure",
                "GA_VS_FPM",
                "default: publish_vs_limits overrides",
            ),
            (
                "autoland",
                "ROUNDOUT_VS_FPM",
                "default: arrest_targets_fpm overrides",
            ),
            (
                "autoland",
                "FLARE_VS_FPM",
                "default: arrest_targets_fpm overrides",
            ),
            (
                "autoland",
                "VS_LEAD_MAX_CLIMB_FPM",
                "default: ArrestLeadProfile overrides",
            ),
            (
                "autoland",
                "MAX_CLIMB_FPM",
                "floor only: max_climb_fpm() takes the airframe's",
            ),
            (
                "circuit",
                "AUTHORED_CLIMB_FPM",
                "floor only: authored_climb_fpm() takes the airframe's",
            ),
            // --- Scaling REFERENCES, not targets: the law divides by them ---
            (
                "autoland",
                "GLIDEPATH_GAIN_REF_KTS",
                "gain scales by actual/this, so it self-corrects",
            ),
            (
                "autotakeoff",
                "PITCH_TRIM_PER_DEG_S",
                "integrator rate; authority is per-airframe",
            ),
            // --- MARGINS in knots. Honest debt: a 10 kt margin is a sixth of
            // a 172's approach speed and a fourteenth of a 737's. None has
            // been caught misbehaving, and each is reviewed here rather than
            // silently shared.
            (
                "autoland",
                "SPEED_FLOOR_MARGIN_KTS",
                "MARGIN: above the airframe's own floor",
            ),
            (
                "autoland",
                "APPROACH_FAST_KTS",
                "MARGIN: over the approach speed",
            ),
            (
                "autoland",
                "CLIMB_SPEED_MARGIN_KTS",
                "MARGIN: surplus before a climb engages",
            ),
            (
                "autoland",
                "CLIMB_ENERGY_BONUS_KTS",
                "MARGIN: speed asked for under the floor",
            ),
            (
                "autotakeoff",
                "OVERSPEED_GUARD_KTS",
                "MARGIN: below Vne, which is per-airframe",
            ),
            (
                "circuit",
                "APPROACH_LEG_MARGIN_KTS",
                "MARGIN: over the approach speed",
            ),
            // --- Known debt, named. Suspect on a jet, not yet measured wrong.
            (
                "autotakeoff",
                "OVERSPEED_VS_MAX_FPM",
                "DEBT: 1500 is a jet's ordinary climb",
            ),
            (
                "circuit",
                "ALT_MAX_EXTRA_DESCENT_FPM",
                "DEBT: 500 extra is little for a jet",
            ),
            (
                "autoland",
                "FLAP_EXTEND_MARGIN_KTS",
                "MARGIN: under the airframe's own per-detent Vfe",
            ),
            (
                "autoland",
                "FLAP_RETRACT_MARGIN_KTS",
                "MARGIN: over the airframe's own per-detent Vfe",
            ),
            (
                "procedure",
                "DRAG_OUT_KTS",
                "MARGIN: over the governor's own target",
            ),
            (
                "procedure",
                "DRAG_IN_KTS",
                "MARGIN: over the governor's own target",
            ),
            (
                "autotakeoff",
                "GEAR_WARN_MARGIN_KTS",
                "MARGIN: over the airframe's own .acf gear-warning speed",
            ),
            (
                "autotakeoff",
                "GEAR_VLO_MARGIN_KTS",
                "MARGIN: under the airframe's own gear placard",
            ),
            (
                "procedure",
                "DRAG_MIN_DESCENT_FPM",
                "a gate on the COMMANDED rate, which is already the airframe's descent_fpm",
            ),
        ];

        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        let files = [
            ("autotakeoff", root.join("autotakeoff.rs")),
            ("autotaxi", root.join("autotaxi.rs")),
            ("autoland", root.join("autoland.rs")),
            ("circuit", root.join("circuit.rs")),
            ("procedure", root.join("procedure.rs")),
            ("pushback", root.join("pushback.rs")),
            ("taxi_in", root.join("taxi_in.rs")),
            ("speedhold", root.join("tools").join("speedhold.rs")),
        ];
        let mut found = Vec::new();
        for (module, path) in &files {
            let text = std::fs::read_to_string(path).unwrap();
            for line in text.lines() {
                // Laws only, and a law is declared at the top level. A
                // constant indented inside a function is that test's or that
                // helper's own fixture and binds no aircraft. (Truncating at
                // `#[cfg(test)]` instead cut several real laws out of the
                // scan, because these files gate items throughout.)
                let Some(rest) = line.strip_prefix("const ") else {
                    continue;
                };
                let Some((name, _)) = rest.split_once(": f64 = ") else {
                    continue;
                };
                if name == "MS_TO_KTS" {
                    continue; // a unit, not a speed
                }
                if name.ends_with("_KTS")
                    || name.ends_with("_FPM")
                    || name.ends_with("_DEG_S")
                    || name.ends_with("_KTS_S")
                {
                    found.push((*module, name.to_string()));
                }
            }
        }
        assert!(found.len() > 25, "the scan found the laws: {}", found.len());
        let unreviewed: Vec<String> = found
            .iter()
            .filter(|(m, n)| !REVIEWED.iter().any(|(rm, rn, _)| rm == m && rn == n))
            .map(|(m, n)| format!("{m}::{n}"))
            .collect();
        assert!(
            unreviewed.is_empty(),
            "new speed/rate constants must be per-airframe or reviewed here \
             (see CLAUDE.md: all automation numbers are per-airframe): {unreviewed:?}"
        );
        // ...and the list must not rot: an entry for a constant that no
        // longer exists is a reason nobody has read in a while.
        let stale: Vec<&str> = REVIEWED
            .iter()
            .filter(|(m, n, _)| !found.iter().any(|(fm, fn_)| fm == m && fn_ == n))
            .map(|(_, n, _)| *n)
            .collect();
        assert!(stale.is_empty(), "reviewed but gone: {stale:?}");
    }

    /// The A330 can start its own engines, and the block PARSES — every name
    /// and value out of the aircraft's own autostart script.
    ///
    /// It could not until 2026-09-20: neither existing start block could
    /// express an ENG MODE knob and guarded master switches, so the profile
    /// said "engine start is the pilot's" and the copilot had no route to it
    /// at all. Found sitting on the ramp unable to start.
    #[test]
    fn the_a330_declares_an_airliner_start() {
        let p = load("airbus_a330_300").expect("the A330 loads");
        let a = p.airliner_start.as_ref().expect("an airliner start block");
        assert!(a.mode_selector.ends_with("eng_mode_selector"));
        assert_eq!((a.mode_start, a.mode_norm), (1.0, 0.0), "IGN/START, NORM");
        assert!(a.masters.ends_with("eng_master"));
        // One guard per engine, or none at all — a guard list that covers
        // only some of the switches would silently no-op the rest.
        assert!(
            a.master_guards.is_empty() || a.master_guards.len() == p.engines,
            "{} guards for {} engines",
            a.master_guards.len(),
            p.engines
        );
        assert_eq!(a.guard_lift, 0.002, "the script's own lift value");
        assert!(a.guard_secs > 0.0, "and it must be dropped again");
        // The APU is a sequence of COMMANDS, and the momentary START button
        // is the one step with no position to read — it does not latch.
        assert!(
            a.apu_steps.len() >= 4,
            "batteries, APU battery, master, START"
        );
        assert!(
            a.apu_steps.iter().any(|s| s.held_secs.is_some()),
            "the START button is a press-and-hold"
        );
        assert!(
            a.apu_steps
                .iter()
                .filter(|s| s.held_secs.is_none())
                .all(|s| s.position.is_some()),
            "every LATCHING step reads before it fires: a toggle at a switch \
             already on turns it off"
        );
        // The FUEL PUMPS. The first version of this block went APU straight
        // to master switches — a start with no fuel at the engine. The
        // aircraft's own sequence spends eight of its twenty-four steps here,
        // so a pre-start list without them is not a start sequence.
        let pumps = a
            .pre_start_steps
            .iter()
            .filter(|s| s.command.contains("pump"))
            .count();
        assert_eq!(pumps, 8, "all eight fuel pumps, as the aircraft does it");
        assert!(
            a.pre_start_steps
                .iter()
                .any(|s| s.command.contains("apu_toggle")),
            "start air has to be selected"
        );
        // THE ADIRS. Without them the engines run and the aircraft has no
        // attitude, heading or air data to fly on: every PFD tape flags red
        // and the ND reads MAP NOT AVAIL. Three IRs, three ADRs, three
        // selectors to NAV.
        let count = |frag: &str| {
            a.pre_start_steps
                .iter()
                .filter(|s| s.command.contains(frag))
                .count()
        };
        assert_eq!(
            count("/ir1_toggle") + count("/ir2_toggle") + count("/ir3_toggle"),
            3
        );
        assert_eq!(
            count("adr1_toggle") + count("adr2_toggle") + count("adr3_toggle"),
            3
        );
        let knobs: Vec<_> = a
            .pre_start_steps
            .iter()
            .filter(|s| s.command.contains("knob"))
            .collect();
        assert_eq!(knobs.len(), 3, "one selector per ADIRS");
        // A three-position knob needs a direction. Told only "press right
        // until it reads NAV", one left in ATT walks away from it.
        assert!(
            knobs.iter().all(|s| s.down_command.is_some()),
            "every knob can step both ways"
        );
        // ...and the ADIRS come BEFORE the engines are cranked, so alignment
        // runs while they start rather than after.
        let first_adirs = a
            .pre_start_steps
            .iter()
            .position(|s| s.command.contains("adirs"))
            .expect("adirs present");
        let beacon = a
            .pre_start_steps
            .iter()
            .position(|s| s.command.contains("beacon"))
            .expect("beacon present");
        assert!(first_adirs < beacon, "alignment starts first");

        // ...and the copilot WATCHES them for the rest of the flight, not
        // just at the start. A start can be missed, and an aircraft can be
        // loaded with them off — and the failure is silent until every tape
        // on the PFD is red. These rows fix rather than ask: there is no
        // judgement in "the inertial reference should be on".
        let watched: Vec<_> = p
            .advisor
            .iter()
            .filter(|r| r.dataref.contains("adirs"))
            .collect();
        assert_eq!(watched.len(), 9, "three IRs, three ADRs, three selectors");
        assert!(
            watched.iter().all(|r| r.command.is_some()),
            "every ADIRS row is actioned, not merely advised"
        );
        assert!(
            watched.iter().all(|r| r.expect_on),
            "off is always the failure"
        );
        // Watched on the GROUND too — before it matters, not once airborne
        // with nothing to fly on.
        assert!(
            watched
                .iter()
                .all(|r| r.phases.iter().any(|p| p == "ground")),
            "caught before we go"
        );
        // ...and the APU stops carrying the bleed once the engines have it.
        assert!(
            a.post_start_steps.iter().any(|s| s.want == 0.0),
            "the APU is shut down, not left running on its bleed"
        );
        assert!(
            a.post_start_steps
                .iter()
                .filter(|s| s.command.contains("pack"))
                .count()
                == 2,
            "both packs come back on"
        );
        // The START button is a command, and its name is ALSO a dataref on
        // this aircraft — the button's position, rewritten every frame.
        // Declaring it as a dataref is what made the APU never move.
        assert!(a
            .apu_steps
            .iter()
            .any(|s| s.command.ends_with("APU_start") && s.position.is_none()));
        assert!(a.spool_idle > 0.0 && a.apu_ready_n1 > 0.0);
        // ...and it must not also claim a start block it cannot fly: two
        // tools called `start_engines` is a registry that answers twice.
        assert!(
            !p.turbine_start,
            "the MD-82 sequence is not this aircraft's"
        );
        assert!(p.turboprop_start.is_none(), "nor are condition levers");
    }

    /// The ground roll is scaled to what the aircraft WEIGHS, and the
    /// scaling errs upward.
    ///
    /// Edinburgh 24 is 2,549 m; the A330's profile roll of 6,000 ft times the
    /// caller's 1.4 factor asks 2,560. Eleven metres, at MAX takeoff weight,
    /// on a light sector — and the refusal came from every position on the
    /// runway, having sent the pilot to line up and then to backtrack
    /// (2026-09-20). Real A330s operate into Edinburgh.
    #[test]
    fn the_takeoff_roll_follows_the_weight_and_errs_upward() {
        let mtow = 533_519.0 * 0.453_592; // the A330's, kg
        let roll = 6000.0 * 0.3048; // its MTOW ground roll, m
                                    // At max weight nothing changes — the profile figure IS that case.
        assert_eq!(takeoff_roll_m(roll, Some(mtow), Some(mtow)), roll);
        // On a light sector it asks for less, and EGPH becomes flyable.
        let light = takeoff_roll_m(roll, Some(180_000.0), Some(mtow));
        assert!(
            light * 1.4 < 2549.0,
            "EGPH fits at 180 t: {:.0}",
            light * 1.4
        );
        // ...but NOT as little as the square law would allow. Measured
        // against published field lengths the real exponent is about 1.1, so
        // linear asks slightly MORE than the aircraft needs and the square
        // law asks far less. When a margin is wrong it is wrong upward.
        let square = roll * (180_000.0 / mtow).powi(2);
        assert!(
            light > square * 1.2,
            "linear must be well above the square law: {light:.0} vs {square:.0}"
        );
        // A floor, because nothing here models density altitude, a wet
        // runway or a rejected takeoff.
        assert_eq!(takeoff_roll_m(roll, Some(1.0), Some(mtow)), roll * 0.5);
        // Over max weight asks no less than the figure itself.
        assert_eq!(takeoff_roll_m(roll, Some(mtow * 2.0), Some(mtow)), roll);
        // An unknown weight or MTOW keeps the old behaviour exactly.
        assert_eq!(takeoff_roll_m(roll, None, Some(mtow)), roll);
        assert_eq!(takeoff_roll_m(roll, Some(180_000.0), None), roll);
        assert_eq!(takeoff_roll_m(roll, Some(f64::NAN), Some(mtow)), roll);
    }

    /// A retraction schedule has to be FLYABLE: the speed that lets a detent
    /// be selected must sit under the placard of the detent being left. The
    /// A330 cleaned up "above 230" out of a CONF 1 whose Vfe is 215, so the
    /// clean-up could never legally happen and never did: flap 1 rode the
    /// climb to 330 kt (EGPH, 2026-09-20).
    #[test]
    fn every_profiles_flap_schedule_can_be_flown_inside_its_placards() {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("profiles");
        let mut checked = 0;
        for e in std::fs::read_dir(dir).unwrap().flatten() {
            let path = e.path();
            if path.extension().is_none_or(|x| x != "yaml") {
                continue;
            }
            let text = std::fs::read_to_string(&path).unwrap();
            let Ok(profile) = serde_yaml::from_str::<AircraftProfile>(&text) else {
                continue;
            };
            let Some(flaps) = &profile.flaps else {
                continue;
            };
            let mut detents = flaps.detents.clone();
            detents.sort_by(|a, b| a.ratio.total_cmp(&b.ratio));
            for pair in detents.windows(2) {
                let (cleaner, dirtier) = (&pair[0], &pair[1]);
                if let (Some(gate), Some(vfe)) = (cleaner.retract_above_kts, dirtier.vfe_kts) {
                    checked += 1;
                    assert!(
                        gate < vfe,
                        "{}: retracting to {} waits for {gate} kt, past the {vfe} kt placard of {}",
                        path.display(),
                        cleaner.label,
                        dirtier.label
                    );
                }
            }
        }
        assert!(checked > 0, "no schedule was checked");
    }

    /// The pavement rule is ONE function. Full auto kept its own copy without
    /// the weight scaling and said "I couldn't get us onto the runway" to an
    /// A330 on Edinburgh 24's centreline, which the spoken takeoff accepted
    /// (2026-09-20, about 200 t on the wheels).
    #[test]
    fn one_pavement_rule_and_edinburgh_fits_it_at_the_weight_flown() {
        let mtow = 533_519.0 * 0.453_592;
        let roll = Some(6000.0 * 0.3048);
        // As flown: fits the 2,549 m strip, and the 2,338 m left from the
        // displaced threshold the lead-on joins at.
        let flown = takeoff_needed_m(roll, Some(200_000.0), Some(mtow), 500.0);
        assert!(flown < 2338.0, "{flown:.0}");
        // At max weight, or with the weight unread, the old refusal stands.
        assert!(takeoff_needed_m(roll, Some(mtow), Some(mtow), 500.0) > 2549.0);
        assert!(takeoff_needed_m(roll, None, Some(mtow), 500.0) > 2549.0);
        // No roll on file is the caller's floor; a tiny roll never goes under it.
        assert_eq!(
            takeoff_needed_m(None, Some(1000.0), Some(1100.0), 400.0),
            400.0
        );
        assert_eq!(takeoff_needed_m(Some(100.0), None, None, 500.0), 500.0);
    }

    /// EVERY jet and turboprop declares the numbers the laws would otherwise
    /// take from a Cessna 172, and they PARSE.
    ///
    /// The 737 was fixed on 2026-09-20 and five more turbine profiles were
    /// still silent that evening — each one would have repeated the whole
    /// sortie: a 0.8 nm capture radius, a 700 fpm descent, a 0.10 nose-down
    /// limit, avgas density and an 800 fpm go-around. Silence is a legal
    /// default (it has to be, so nothing GA moves), which is exactly why the
    /// turbines have to be named here instead.
    #[test]
    fn every_turbine_profile_declares_its_own_numbers() {
        const TURBINES: &[&str] = &[
            "boeing_737_800",
            "airbus_a330_300",
            "boeing_747_400",
            "md82",
            "citation_x",
            "cirrus_vision_sf50",
            "dhc6_twin_otter",
            "king_air_c90b",
            "lancair_evolution",
        ];
        for id in TURBINES {
            let p = load(id).unwrap_or_else(|| panic!("{id} loads"));
            let kind = p
                .reference
                .as_ref()
                .and_then(|r| r.kind.as_deref())
                .unwrap_or("");
            assert!(
                kind.contains("jet") || kind.contains("turboprop"),
                "{id} is listed as a turbine but says {kind:?}"
            );
            assert!(
                p.pitch_authority_down.is_some_and(|a| a > 0.10),
                "{id}: the rotation needs more push than a 172's tenth"
            );
            let f = p
                .fuel
                .as_ref()
                .unwrap_or_else(|| panic!("{id} has no fuel block, so it briefs in avgas"));
            assert_eq!(f.kg_per_gal, Some(3.04), "{id} burns Jet A-1, not 100LL");
            assert!(f.spoken_unit.is_some(), "{id} must name its crew's unit");
            let r = p.reference.as_ref().expect("reference");
            let climb = r.enroute_climb_fpm.unwrap_or(0.0);
            let descent = r.enroute_descent_fpm.unwrap_or(0.0);
            assert!(
                climb > 700.0 && descent > 700.0,
                "{id}: {climb}/{descent} is the GA figure or worse"
            );
            // The optimum-altitude advisory is all-or-nothing: a partial
            // declaration would silently produce no advice at all.
            let opt = [
                r.optimum_alt_ft,
                r.optimum_alt_at_kg,
                r.optimum_alt_ft_per_1000kg,
            ];
            assert!(
                opt.iter().all(Option::is_some) || opt.iter().all(Option::is_none),
                "{id}: optimum altitude is half-declared, which advises nothing"
            );
        }
    }

    /// Everything this airframe was given today, read back out of its YAML —
    /// a profile field that does not parse is a fix that never shipped.
    #[test]
    fn the_737_profile_declares_its_jet_numbers() {
        let p = load("boeing_737_800").expect("the 737 profile loads");
        assert_eq!(p.pitch_authority_down, Some(0.5), "nose-down authority");
        assert_eq!(p.tail_strike_deg, Some(11.0));
        let f = p.fuel.as_ref().expect("fuel block");
        assert_eq!(f.kg_per_gal, Some(3.04), "Jet A-1, not 100LL");
        assert_eq!(f.spoken_unit.as_deref(), Some("kg"));
        let r = p.reference.as_ref().expect("reference block");
        assert_eq!(r.enroute_climb_fpm, Some(2000.0));
        assert_eq!(r.enroute_descent_fpm, Some(2000.0));
        assert_eq!(r.optimum_alt_ft, Some(38_000.0));
        // The retraction schedule the climb-out walks: up at 210, 1 at 190,
        // 2 at 180 — cleanest first.
        assert_eq!(
            flap_schedule(p.flaps.as_ref()),
            vec![(0.0, 210.0), (0.125, 190.0), (0.25, 180.0)]
        );
    }

    /// The BIKF-EGPH brief, verbatim from the panel (2026-09-20): 3,126 kg in
    /// a 737's tanks read back as "1,149 gallons". That figure is 3126/2.72 —
    /// AVGAS density, on an airframe that burns Jet A-1.
    #[test]
    fn a_jets_fuel_is_not_spoken_in_avgas_gallons() {
        let _g = FUEL_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let jet = FuelProfile {
            reserve_caution_min: 60.0,
            reserve_warning_min: 30.0,
            cruise_burn_kg_hr: 2500.0,
            kg_per_gal: Some(3.04),
            spoken_unit: Some("kg".into()),
        };
        publish_fuel_unit(Some(&jet));
        assert_eq!(say_fuel_kg(3126.0), "3,126 kilos", "the crew's own unit");
        // Had it stayed in gallons, it would at least be the right ones.
        let jet_gal = FuelProfile {
            spoken_unit: Some("gallons".into()),
            ..jet.clone()
        };
        publish_fuel_unit(Some(&jet_gal));
        assert_eq!(say_fuel_kg(3126.0), "1,028 gallons", "Jet A, not 100LL");
        // A piston declares nothing and is unchanged: 100LL, gallons — the
        // 1,149 that started all this is CORRECT for a light aircraft.
        publish_fuel_unit(None);
        assert_eq!(say_fuel_kg(3126.0), "1,149 gallons");
        assert_eq!(fuel_unit().0, AVGAS_KG_PER_GAL);
        // And a swap off the jet must not leave its density behind.
        publish_fuel_unit(Some(&jet));
        publish_fuel_unit(None);
        assert_eq!(
            say_fuel_kg(3126.0),
            "1,149 gallons",
            "never outlives its airframe"
        );
    }

    /// The figures in the brief are rounded into the airframe's unit BEFORE
    /// they are subtracted, so they add up on screen. The panel said "You
    /// have 1149 aboard, but the plan needs 3002; short about 1852" — and
    /// 3002 minus 1149 is 1853.
    #[test]
    fn the_spoken_fuel_figures_agree_with_each_other() {
        let _g = FUEL_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        publish_fuel_unit(None);
        let aboard = fuel_in_unit_kg(3126.0).round();
        let need = fuel_in_unit_kg(8165.0).round();
        assert_eq!(say_fuel_figure(aboard), "1,149 gallons");
        assert_eq!(say_fuel_figure(need), "3,002 gallons");
        assert_eq!(
            say_fuel_figure(need - aboard),
            "1,853 gallons",
            "a difference of rounded figures, never a rounded difference"
        );
    }

    #[test]
    fn a_bad_fuel_declaration_is_refused_rather_than_flown() {
        let _g = FUEL_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let bad = FuelProfile {
            reserve_caution_min: 60.0,
            reserve_warning_min: 30.0,
            cruise_burn_kg_hr: 2500.0,
            kg_per_gal: Some(0.0),
            spoken_unit: Some("furlongs".into()),
        };
        publish_fuel_unit(Some(&bad));
        assert_eq!(fuel_unit().0, AVGAS_KG_PER_GAL, "a zero density is refused");
        assert_eq!(
            fuel_unit().1,
            FuelSpoken::Gallons,
            "an unknown unit falls back"
        );
        publish_fuel_unit(None);
    }

    #[test]
    fn cessna_yaml_loads_and_matches() {
        // load() prefers profiles/cessna_172sp.yaml (cwd is the crate root in tests).
        let p = load("cessna_172sp").expect("profile loads");
        assert_eq!(p.id, "cessna_172sp");
        assert!(p.autopilot && p.engine && p.barometer && p.com_radio);
        assert!(!p.retractable_gear);
        assert_eq!(p.lights.len(), 5);
        // SOP advisor rows: taxi light on the ground, strobes airborne, landing
        // light for takeoff/arrival.
        assert_eq!(p.advisor.len(), 3);
        assert!(p.advisor.iter().all(|a| a.expect_on));
        // The taxi light auto-comes-on for taxi (the ground phase)...
        let taxi = p
            .advisor
            .iter()
            .find(|a| a.command.as_deref() == Some("sim/lights/taxi_lights_on"))
            .expect("a taxi-light SOP row");
        assert_eq!(taxi.phases, vec!["ground".to_string()]);
        // ...and the landing light is never expected on the ground (it dazzles
        // other crews — that's the taxi light's job).
        let landing = p
            .advisor
            .iter()
            .find(|a| a.command.as_deref() == Some("sim/lights/landing_lights_on"))
            .expect("a landing-light SOP row");
        assert!(!landing.phases.iter().any(|ph| ph == "ground"));
        let flaps = p.flaps.expect("has flaps");
        assert_eq!(flaps.detents.len(), 4);
        assert_eq!(flaps.detents[0].label, "up");
        assert_eq!(flaps.detents[3].label, "full");
        assert_eq!(flaps.detents[1].vfe_kts, Some(110.0));
        let fuel = p.fuel.expect("has fuel thresholds");
        assert_eq!(fuel.reserve_caution_min, 45.0);
        assert_eq!(fuel.reserve_warning_min, 30.0);
        // Reference figures load from YAML (and match the built-in), so the
        // copilot can recite V-speeds/limits without hardcoded literals.
        let r = p.reference.as_ref().expect("has reference figures");
        assert_eq!(r.best_glide_kts, Some(68.0));
        assert_eq!(r.va_kts, Some(99.0));
        assert_eq!(r.vx_kts, cessna_172sp().reference.unwrap().vx_kts);
        // The checklist action map loads, and the YAML matches the built-in (same
        // matches, in the same order) — so behaviour is identical either source.
        let yaml_map = &p.checklist_map;
        let builtin_map = &cessna_172sp().checklist_map;
        assert!(!yaml_map.is_empty());
        assert_eq!(yaml_map.len(), builtin_map.len());
        for (a, b) in yaml_map.iter().zip(builtin_map) {
            assert_eq!(a.match_, b.match_, "map entry mismatch (YAML vs built-in)");
        }
        // Electrical loads from YAML (not a silent fall-back to the built-in) and
        // matches it key-for-key.
        let builtin = cessna_172sp();
        let yaml_elec: Vec<&str> = p.electrical.iter().map(|e| e.key.as_str()).collect();
        let builtin_elec: Vec<&str> = builtin.electrical.iter().map(|e| e.key.as_str()).collect();
        assert_eq!(
            yaml_elec, builtin_elec,
            "electrical mismatch (YAML vs built-in)"
        );
    }

    #[test]
    fn yaml_profiles_parse_strictly() {
        // load() falls back to the built-in on a parse error, which silently
        // masks a malformed profile (a bad dataref, an un-tagged action enum, a
        // bare `off` collapsing to YAML 1.1 boolean false). Deserialize each file
        // directly so any such breakage fails loudly here instead of vanishing
        // behind the fallback.
        for id in ["cessna_172sp", "cirrus_sr22"] {
            let text = std::fs::read_to_string(format!("profiles/{id}.yaml")).unwrap();
            let p: AircraftProfile = serde_yaml::from_str(&text)
                .unwrap_or_else(|e| panic!("{id}.yaml failed to parse: {e:#}"));
            assert_eq!(p.id, id);
            let fs = p.fuel_selector.expect("fuel selector present");
            assert!(fs
                .positions
                .iter()
                .all(|p| !p.key.is_empty() && !p.command.is_empty()));
            assert!(!p.checklist_map.is_empty());
        }
    }

    /// An airframe whose own lua OWNS the stock mixture handle must declare the
    /// lever it is really cut with (2026-09-20). The 737's B738.systems.lua
    /// rewrites `mixture_ratio[0..1]` every frame from `condition_lever{1,2}`,
    /// so the stock write is accepted and then erased, and "fuel levers cut"
    /// was a lie at the EIDW stand. Probed live: `condition_lever1` is_writable
    /// FALSE, `_target` TRUE — the TARGET is the control.
    #[test]
    fn the_737_declares_the_lever_it_is_actually_cut_with() {
        let text = std::fs::read_to_string("profiles/boeing_737_800.yaml").unwrap();
        let p: AircraftProfile = serde_yaml::from_str(&text).unwrap();
        assert_eq!(
            p.fuel_cutoff_levers.len(),
            p.engines,
            "one cutoff lever per engine, or the second one runs on"
        );
        for lever in &p.fuel_cutoff_levers {
            assert!(
                !lever.contains("mixture_ratio"),
                "{lever}: the lua rewrites the stock handle every frame"
            );
            assert!(
                lever.ends_with("_target"),
                "{lever}: condition_lever itself is an animation output, read-only"
            );
        }
        assert_eq!(
            p.fuel_cutoff_value, 0.0,
            "the lua lights the engine at >= 0.75"
        );
        // And the probe covers them, so a renamed lever is said at boot rather
        // than found at a stand.
        let refs = profile_references(&p);
        for lever in &p.fuel_cutoff_levers {
            assert!(refs.contains(lever), "{lever} is not probed");
        }
    }

    /// The stock airliners (2026-09-16): parsed strictly, and the numbers the
    /// automation flies are the airframe's own, never the GA defaults (the
    /// RV-10 lesson at 174,000 lb).
    #[test]
    fn stock_airliner_yamls_parse_strictly_and_carry_their_own_numbers() {
        for (id, vr, roll) in [
            ("boeing_737_800", 145.0, 5500.0),
            ("airbus_a330_300", 148.0, 6000.0),
        ] {
            let text = std::fs::read_to_string(format!("profiles/{id}.yaml")).unwrap();
            let p: AircraftProfile = serde_yaml::from_str(&text)
                .unwrap_or_else(|e| panic!("{id}.yaml failed to parse: {e:#}"));
            assert_eq!(p.id, id);
            assert_eq!(p.engines, 2);
            assert_eq!(
                p.pressurised,
                Some(true),
                "{id}: cabin alerts need the real answer"
            );
            assert!(p.cabin_crew && p.retractable_gear && p.throttle && !p.engine);
            assert!(
                !p.autothrottle,
                "{id}: the md82 autothrottle tool would withhold hold_speed"
            );
            assert!(
                !p.turbine_start && !p.auto_prep,
                "{id}: the MD-82's APU/bleed sequence is not this aircraft's"
            );
            assert_eq!(
                p.scripted_auto_start,
                id == "boeing_737_800",
                "{id}: the 737 scripts its own start, the A330 does not (yet)"
            );
            let r = p.reference.as_ref().expect("reference card");
            assert_eq!(r.vr_kts, Some(vr));
            assert_eq!(
                r.takeoff_roll_ft,
                Some(roll),
                "{id}: the roll the runway check uses"
            );
            assert!(r.landing_distance_ft.unwrap() > 4000.0);
            assert!(r.approach_kts.unwrap() >= 140.0 && p.vs_clean_kts.unwrap() > 130.0);
            let f = p.flaps.as_ref().expect("flap detents");
            let to = f.takeoff_ratio.unwrap();
            assert!(
                f.detents.iter().any(|d| (d.ratio - to).abs() < 1e-9),
                "{id}: takeoff flap is a gate"
            );
            assert!(f.detents.last().unwrap().vfe_kts.is_some());
            assert!(p.fuel.as_ref().unwrap().cruise_burn_kg_hr > 1000.0);
            assert!(p.tail_strike_deg.is_some());
            // A jet's INITIAL climb is not its enroute climb speed, and the
            // takeoff's climb law protects the number it is given: fed the
            // enroute 210, it read the 737's real 174 kt as thirty-six knots
            // slow and commanded 150 fpm at takeoff thrust (2026-09-17).
            let ic = r
                .initial_climb_kts
                .expect("an airliner declares its initial climb");
            assert!(
                ic >= vr && ic < r.vy_kts.unwrap(),
                "{id}: initial climb {ic} must sit between Vr and the enroute climb"
            );
        }
    }

    #[test]
    fn dc3_yaml_loads_and_is_conservative() {
        // YAML-only profile (no built-in). Deserialize the file directly so a
        // malformed field fails loudly here rather than vanishing behind load()'s
        // built-in fallback (which would return None for "dc3").
        let text = std::fs::read_to_string("profiles/dc3.yaml").unwrap();
        let p: AircraftProfile = serde_yaml::from_str(&text)
            .unwrap_or_else(|e| panic!("dc3.yaml failed to parse: {e:#}"));
        assert_eq!(p.id, "dc3");
        // Twin radial: retractable gear with a VLO gate, throttle-only (the piston
        // group stays off — mixture is xlua-overridden, start/ignition are custom).
        assert!(p.retractable_gear);
        assert_eq!(p.vlo_kts, Some(140.0));
        assert!(p.throttle && !p.engine);
        assert!(p.fuel_selector.is_none()); // uses the dual cycle selectors instead
                                            // Twin per-engine accessories, verified live: magnetos (via ignition_on),
                                            // carb heat, and the boost pumps, all written across both engines.
        assert_eq!(p.engines, 2);
        assert!(p.magnetos && p.magnetos_per_engine && p.carb_heat && p.fuel_pump);
        // No functional X-Plane autopilot on this airframe (servos never latch,
        // verified live), so autopilot + ILS are off; the rest of the universal
        // tools stay on.
        assert!(!p.autopilot && !p.ils);
        assert!(p.barometer && p.transponder && p.pitch_trim);
        // Flaps: up + 1/4, 1/2, 3/4, full with descending Vfe.
        let flaps = p.flaps.expect("has flaps");
        assert_eq!(flaps.detents.len(), 5);
        assert_eq!(flaps.detents[0].label, "up");
        assert_eq!(flaps.detents[4].label, "full");
        assert_eq!(flaps.detents[1].vfe_kts, Some(130.0));
        assert_eq!(flaps.detents[4].vfe_kts, Some(97.0));
        assert_eq!(p.vne_kts, Some(178.0));
        assert_eq!(p.vs_clean_kts, Some(65.0));
        let r = p.reference.as_ref().expect("dc3 reference");
        assert_eq!(r.vr_kts, Some(84.0));
        assert_eq!(r.max_gross_lb, Some(25200.0));
        // Custom cycle-switch systems wired from config: power switch, dual fuel
        // selectors, and the twin piston start.
        let power = p.power_switch.as_ref().expect("dc3 power switch");
        assert_eq!(power.switch.up, "awx/c47/powersw_up");
        assert!(!power.switch.wrap); // clamped at the ends
        assert_eq!(power.states.first().map(|s| s.value), Some(0)); // battery first
        assert_eq!(p.fuel_selectors.len(), 2);
        let left = &p.fuel_selectors[0];
        assert_eq!(left.key, "left");
        assert_eq!(left.engine, 0);
        assert_eq!(left.start_value, Some(2)); // left main
        assert!(left.switch.wrap); // selectors wrap 0<->4
        assert_eq!(p.fuel_selectors[1].start_value, Some(3)); // right main
        let start = p.piston_start.as_ref().expect("dc3 piston start");
        assert_eq!(start.starters.len(), 2);
        assert_eq!(start.mixture_start_notch, 2.0);
        assert_eq!(start.mixture_cutoff_notch, 0.0);
        let mix = p.mixture_notch.as_ref().expect("dc3 mixture notch");
        assert_eq!(mix.notches.len(), 4);
        assert_eq!(mix.notches.first().map(|n| n.value), Some(0)); // cutoff
        assert_eq!(mix.notches.last().map(|n| n.value), Some(3)); // full rich
    }

    /// The Baron is the first STOCK piston twin: unlike the DC-3 (every system
    /// xlua-overridden) the generic tools drive it directly, so what this test
    /// guards is that the twin-ness actually reaches them.
    #[test]
    fn beechcraft_baron_58_yaml_loads() {
        let text = std::fs::read_to_string("profiles/beechcraft_baron_58.yaml").unwrap();
        let p: AircraftProfile = serde_yaml::from_str(&text)
            .unwrap_or_else(|e| panic!("beechcraft_baron_58.yaml failed to parse: {e:#}"));
        assert_eq!(p.id, "beechcraft_baron_58");

        // Twin, generic piston group. `engines` is what makes the magneto and
        // start tools write both engines rather than only number one.
        assert_eq!(p.engines, 2);
        assert!(p.engine && p.magnetos && p.fuel_pump);
        assert!(!p.magnetos_per_engine); // ignition_key, not the DC-3's ignition_on
        assert!(!p.carb_heat); // IO-550, fuel injected
        assert!(!p.turbine_start && p.piston_start.is_none());

        // Per-side fuel selectors, cycle-driven. Without these the start
        // sequence cranks two dry engines: they sit at OFF cold, and their
        // position dataref is read-only, so only the command can open them.
        assert_eq!(p.fuel_selectors.len(), 2);
        let (l, r) = (&p.fuel_selectors[0], &p.fuel_selectors[1]);
        assert_eq!((l.engine, r.engine), (0, 1));
        assert_eq!((l.start_value, r.start_value), (Some(1), Some(1)));
        assert!(l.switch.up.ends_with("_L_up") && r.switch.up.ends_with("_R_up"));
        // Never step onto a position we have not proven exists in the sim.
        assert_eq!((l.switch.min, l.switch.max), (0, 1));
        assert!(!l.switch.wrap, "off must not wrap round from on");

        // Retractable gear with a limit — the 172's whole reason for having no
        // gear tool is that it has none.
        assert!(p.retractable_gear);
        assert_eq!(p.vlo_kts, Some(152.0));

        // Per-notch Vfe is the point of a flap profile: approach flap is good to
        // 152 KIAS and full only to 122, and one figure for both would put full
        // flap out thirty knots early.
        let flaps = p.flaps.as_ref().expect("baron flaps");
        assert_eq!(flaps.detents.len(), 3);
        assert_eq!(flaps.takeoff_ratio, Some(0.0)); // clean takeoff
        let vfe: Vec<Option<f64>> = flaps.detents.iter().map(|d| d.vfe_kts).collect();
        assert_eq!(vfe, vec![None, Some(152.0), Some(122.0)]);

        // The twin speeds a multi pilot actually asks for.
        let r = p.reference.as_ref().expect("baron reference");
        assert_eq!(r.vyse_kts, Some(100.0)); // blue line
        assert_eq!(r.vmca_kts, Some(84.0));
        assert_eq!(r.kind.as_deref(), Some("twin-engine aircraft"));
        // The ground roll, not a factored field length (the MD-82 lesson).
        assert_eq!(r.takeoff_roll_ft, Some(1373.0));

        // Two independent landing-light switches, written together: switching
        // one leaves the aircraft half lit with a readback that confirms the
        // half it looked at (the Citation X trap).
        let landing = p
            .lights
            .iter()
            .find(|l| l.key == "landing")
            .expect("baron landing light");
        assert_eq!(landing.also_datarefs.len(), 1);
        assert!(landing.command_on.is_none(), "written, never commanded");

        // Roof floods: a left and a right pod, written together for the same
        // reason, and the one light on this airframe the copilot sets itself
        // after dark.
        let roof = p
            .lights
            .iter()
            .find(|l| l.key == "roof")
            .expect("baron roof lights");
        assert_eq!(roof.dataref, "laminar/b58/lighting/roof_L_on");
        assert_eq!(
            roof.also_datarefs,
            vec!["laminar/b58/lighting/roof_R_on".to_string()],
            "one pod lit is a readback confirming the half it looked at"
        );
        assert!(roof.auto_at_night);
        // Interior only. Switching an EXTERIOR light on the pilot's behalf
        // changes what other traffic sees, so nothing out there is ever
        // flagged: this assertion is the guard on that rule, not a restatement
        // of the profile.
        for l in p.lights.iter().filter(|l| l.auto_at_night) {
            assert_eq!(l.key, "roof", "{} is not an interior light", l.key);
        }

        // The flood switch is a COMMAND with a read-only position dataref —
        // probing the live sim showed no `flood_light_switch` DATAREF exists,
        // so the profile's old `flood_lights_dataref` wrote into nothing and
        // read back nothing (2026-08-02).
        assert!(p.flood_lights);
        assert!(
            p.flood_lights_dataref.is_none(),
            "there is no such dataref on this airframe"
        );
        let flood = p.flood_lights_toggle.as_ref().expect("baron flood toggle");
        assert_eq!(flood.command, "laminar/b58/lighting/flood_light_switch");
        assert_eq!(flood.dataref, "laminar/b58/flood_light_toggle/switch_pos");

        // The GNS430's welcome page, cleared once per power-up.
        assert_eq!(p.gps_ack_command.as_deref(), Some("sim/GPS/g430n1_ent"));

        // One battery, one alternator per engine, plus the pair and avionics.
        let keys: Vec<&str> = p.electrical.iter().map(|e| e.key.as_str()).collect();
        assert_eq!(
            keys,
            vec![
                "battery",
                "alternator1",
                "alternator2",
                "alternators",
                "avionics"
            ]
        );
    }

    #[test]
    fn king_air_c90b_yaml_loads() {
        // YAML-only profile. Deserialize directly so a malformed field fails here
        // rather than silently falling back.
        let text = std::fs::read_to_string("profiles/king_air_c90b.yaml").unwrap();
        let p: AircraftProfile = serde_yaml::from_str(&text)
            .unwrap_or_else(|e| panic!("king_air_c90b.yaml failed to parse: {e:#}"));
        assert_eq!(p.id, "king_air_c90b");
        // Twin turboprop: power-lever throttle (not the piston group), retractable
        // gear, no piston/turbine-start/AP tools for v1.
        assert!(p.throttle && !p.engine);
        assert!(p.retractable_gear);
        // Real AP, verified live (needs engines running); no piston/jet-start tools.
        assert!(p.autopilot && p.ils);
        assert!(!p.turbine_start && !p.magnetos && !p.carb_heat);
        // PT6 turboprop start, verified live: two engines, condition lever +
        // ign/start switch commands + per-engine generator.
        // Electrical wired to parity with the other profiles: battery, dual
        // generators (+ both), avionics.
        let elec: Vec<&str> = p.electrical.iter().map(|e| e.key.as_str()).collect();
        for k in [
            "battery",
            "generator1",
            "generator2",
            "generators",
            "avionics",
        ] {
            assert!(elec.contains(&k), "c90b electrical missing {k}");
        }
        assert!(p.flood_lights); // cockpit floods wired (generic_lights_switch[0])
        assert!(p.fuel_pump); // standby boost pumps (verified)
        let ts = p.turboprop_start.as_ref().expect("c90b turboprop start");
        assert_eq!(ts.start_up.len(), 2);
        assert_eq!(ts.start_dn.len(), 2);
        // dn+up per generator (the 3-pos GEN switch only latches ON on the
        // dn→center transition), so 4 commands for the pair.
        assert_eq!(ts.generators.len(), 4);
        assert_eq!(ts.condition_start, 0.5);
        assert_eq!(ts.condition_run, 1.0);
        // Modeled limits from the .acf.
        assert_eq!(p.vne_kts, Some(226.0));
        assert_eq!(p.vs_clean_kts, Some(100.0));
        // Flaps up/approach/down.
        let flaps = p.flaps.expect("has flaps");
        assert_eq!(flaps.detents.len(), 3);
        assert_eq!(flaps.detents[1].label, "approach");
        assert_eq!(flaps.detents[2].label, "down");
        let r = p.reference.as_ref().expect("c90b reference");
        assert_eq!(r.max_gross_lb, Some(9650.0));
        assert_eq!(r.vr_kts, Some(90.0));
    }

    /// Every installed airframe whose nose castors declares brake steering
    /// (the Evolution and the RV-10 did not, 2026-09-25). Needs the install:
    /// `cargo test -- --ignored real_install_castoring_noses_brake_steer`.
    #[test]
    #[ignore]
    fn real_install_castoring_noses_brake_steer() {
        let root = std::env::var("XCOPILOT_XPLANE_ROOT")
            .unwrap_or_else(|_| "/Users/duanelicudi/X-Plane 12".to_string());
        for (id, dir, acf) in [
            ("lancair_evolution", "Laminar Research/Lancair Evolution", "N844X.acf"),
            ("vans_rv10", "Laminar Research/Van's Aircraft RV-10", "RV-10.acf"),
            ("cirrus_sr22", "Laminar Research/Cirrus SR22", "Cirrus SR22.acf"),
            ("cirrus_vision_sf50", "Laminar Research/Cirrus Vision SF50", "CirrusSF50.acf"),
            ("cessna_172sp", "Laminar Research/Cessna 172 SP", "Cessna_172SP.acf"),
            ("king_air_c90b", "Laminar Research/Beechcraft King Air C90B", "C90B.acf"),
        ] {
            let path = std::path::Path::new(&root).join("Aircraft").join(dir).join(acf);
            let text = String::from_utf8_lossy(&std::fs::read(&path).expect("acf on disk")).to_string();
            let p = load(id).unwrap_or_else(|| panic!("{id} loads"));
            let declared = p.taxi.as_ref().is_some_and(|t| t.brake_steer);
            assert_eq!(nose_castors_from_acf(&text), declared, "{id}");
        }
    }

    #[test]
    fn a_castoring_nose_is_read_from_the_acf() {
        assert!(nose_castors_from_acf("P _gear/0/_gear_castors 1\nP _gear/1/_gear_castors 0"));
        assert!(!nose_castors_from_acf("P _gear/0/_gear_castors 0\nP _gear/1/_gear_castors 1"));
        assert!(!nose_castors_from_acf(""));
        let evo = "P _gear/0/_gear_castors 1\nP _gear/0/_steerdeg_lospeed 0.000000000";
        let sf50 = "P _gear/0/_gear_castors 1\nP _gear/0/_steerdeg_lospeed 40.000000000";
        assert!(nose_free_from_acf(evo));
        assert!(!nose_free_from_acf(sf50));
        assert!(!nose_free_from_acf("P _gear/0/_gear_castors 0\nP _gear/0/_steerdeg_lospeed 0.0"));
    }

    #[test]
    fn lancair_evolution_yaml_loads() {
        // YAML-only profile. Deserialize directly so a malformed field fails here
        // rather than silently falling back.
        let text = std::fs::read_to_string("profiles/lancair_evolution.yaml").unwrap();
        let p: AircraftProfile = serde_yaml::from_str(&text)
            .unwrap_or_else(|e| panic!("lancair_evolution.yaml failed to parse: {e:#}"));
        assert_eq!(p.id, "lancair_evolution");
        // Single turboprop: power lever (not the piston group), retractable gear
        // with the POM's 150 KIAS gear speed, standard autopilot.
        assert!(p.throttle && !p.engine);
        assert!(p.retractable_gear);
        assert_eq!(p.vlo_kts, Some(150.0));
        assert!(p.autopilot && p.ils);
        assert!(!p.turbine_start && !p.magnetos && !p.carb_heat);
        // PT6 start with the press-and-hold starter: one engine, no start_dn
        // (the held button releases itself), igniter armed, boost pump fed.
        let ts = p
            .turboprop_start
            .as_ref()
            .expect("evolution turboprop start");
        assert!(ts.starter_held);
        assert_eq!(ts.start_up.len(), 1);
        assert!(ts.start_dn.is_empty());
        assert!(ts.boost_pumps);
        assert_eq!(ts.ignition_arm.len(), 1);
        assert_eq!(ts.condition_start, 0.5);
        assert_eq!(ts.condition_run, 1.0);
        // Dual battery + generator + standby alternator + DC power. No "both
        // generators" pair (POM: GEN and ALT are not normally used together) and
        // no avionics switch (the xlua forces avionics on at flight start).
        let elec: Vec<&str> = p.electrical.iter().map(|e| e.key.as_str()).collect();
        for k in [
            "battery1",
            "battery2",
            "batteries",
            "generator",
            "alternator",
            "dc_power",
        ] {
            assert!(elec.contains(&k), "evolution electrical missing {k}");
        }
        assert!(!elec.contains(&"avionics") && !elec.contains(&"generators"));
        // After-start brings the GENERATOR + DC POWER online, not the alternator.
        assert_eq!(ts.generators.len(), 2);
        // The AP panel's own power switch is engaged before the servos.
        let prep = p.autopilot_prep.as_ref().expect("evolution autopilot prep");
        assert_eq!(prep.flight_director.len(), 1);
        assert!(!prep.ac_powered);
        // No beacon or taxi light on this airframe (POM) — only nav/strobe/landing.
        let lights: Vec<&str> = p.lights.iter().map(|l| l.key.as_str()).collect();
        assert_eq!(lights, ["nav", "strobe", "landing"]);
        // POM / .acf limits.
        assert_eq!(p.vne_kts, Some(256.0));
        assert_eq!(p.vs_clean_kts, Some(76.0));
        let flaps = p.flaps.expect("has flaps");
        assert_eq!(flaps.detents.len(), 3);
        assert_eq!(flaps.detents[1].vfe_kts, Some(160.0));
        assert_eq!(flaps.detents[2].vfe_kts, Some(140.0));
        // Fuel selector: off/left/right (the POH's engine-failure drills call
        // for OFF; no "both" — the wing tanks don't interconnect).
        let fsel = p.fuel_selector.as_ref().expect("evolution fuel selector");
        let pos: Vec<&str> = fsel.positions.iter().map(|p| p.key.as_str()).collect();
        assert_eq!(pos, ["off", "left", "right"]);
        let r = p.reference.as_ref().expect("evolution reference");
        assert_eq!(r.vr_kts, Some(77.0));
        assert_eq!(r.max_gross_lb, Some(4300.0));
        assert_eq!(r.cruise_kts, Some(300.0));
        let prop = p.prop.as_ref().expect("evolution prop");
        assert_eq!(prop.max_rpm, 2000.0);
    }

    #[test]
    fn dhc6_twin_otter_yaml_loads() {
        // YAML-only profile (RWDesigns add-on). Deserialize directly so a
        // malformed field fails loudly here rather than silently falling back.
        let text = std::fs::read_to_string("profiles/dhc6_twin_otter.yaml").unwrap();
        let p: AircraftProfile = serde_yaml::from_str(&text)
            .unwrap_or_else(|e| panic!("dhc6_twin_otter.yaml failed to parse: {e:#}"));
        assert_eq!(p.id, "dhc6_twin_otter");
        // Twin turboprop: power levers (not the piston group), FIXED gear.
        assert!(p.throttle && !p.engine);
        assert!(!p.retractable_gear);
        assert_eq!(p.engines, 2);
        assert!(!p.magnetos && !p.carb_heat && !p.turbine_start);
        // Stock-driven AP (the panel buttons fire sim/autopilot commands).
        assert!(p.autopilot && p.ils);
        assert!(p.autopilot_prep.is_none());
        // Boost pumps are COMMAND-driven (the universal array is a no-op on
        // this add-on): fuel_pump off, the override wired for both pumps.
        assert!(!p.fuel_pump);
        let pc = p
            .fuel_pump_commands
            .as_ref()
            .expect("twotter pump commands");
        assert_eq!(pc.on_commands.len(), 2);
        assert_eq!(pc.off_commands.len(), 2);
        assert_eq!(pc.readback.len(), 2);
        // Electrical: battery + DC master + generators (each/both) + bleed air.
        let elec: Vec<&str> = p.electrical.iter().map(|e| e.key.as_str()).collect();
        for k in [
            "battery",
            "master",
            "generator1",
            "generator2",
            "generators",
            "bleed",
        ] {
            assert!(elec.contains(&k), "twotter electrical missing {k}");
        }
        // PT6 start with the fire-once starter: no start_dn (the model latches
        // its own 16 s starter run), boost pumps fed per engine via ignition_arm.
        let ts = p.turboprop_start.as_ref().expect("twotter turboprop start");
        assert_eq!(ts.start_up.len(), 2);
        assert!(ts.start_dn.is_empty());
        assert!(!ts.starter_held);
        assert!(!ts.boost_pumps);
        assert_eq!(ts.ignition_arm.len(), 2);
        assert_eq!(ts.generators.len(), 2);
        // Reverse thrust: rollout tool, gated + self-stowing (STOL bushplane —
        // reverse is core to its short landings).
        assert!(p.reverse_thrust);
        // Modeled .acf limits: flaps 0/10/20/30/40, Vfe 105/95, Vne 202, Vs 60.
        let flaps = p.flaps.expect("has flaps");
        assert_eq!(flaps.detents.len(), 5);
        assert_eq!(flaps.detents[0].label, "up");
        assert_eq!(flaps.detents[4].label, "full");
        assert_eq!(flaps.detents[1].vfe_kts, Some(105.0));
        assert_eq!(flaps.detents[4].vfe_kts, Some(95.0));
        assert_eq!(p.vne_kts, Some(202.0));
        assert_eq!(p.vs_clean_kts, Some(60.0));
        let prop = p.prop.as_ref().expect("twotter prop");
        assert_eq!(prop.max_rpm, 2200.0);
        let r = p.reference.as_ref().expect("twotter reference");
        assert_eq!(r.max_gross_lb, Some(12500.0));
        assert_eq!(r.vr_kts, Some(75.0));
        assert_eq!(p.lights.len(), 5);
    }

    #[test]
    fn unknown_profile_is_none() {
        // No YAML, no built-in → None (caller surfaces a clear error).
        assert!(load("does_not_exist").is_none());
    }

    #[test]
    fn sr22_yaml_loads_and_is_conservative() {
        let p = load("cirrus_sr22").expect("SR22 profile loads");
        assert_eq!(p.id, "cirrus_sr22");
        // Universal-dataref tools on (incl. engine — throttle/mixture/start), plus
        // the rustdecks-verified boost pump, key-start ignition and pitch trim.
        assert!(p.autopilot && p.parking_brake && p.pitot_heat && p.transponder && p.engine);
        assert!(p.magnetos && p.fuel_pump && p.pitch_trim);
        // Still off: carb heat (fuel-injected) and the vacuum DG (G1000 glass).
        assert!(!p.carb_heat && !p.directional_gyro);
        // Fuel selector is wired (verified deck commands): OFF / LEFT / RIGHT,
        // no "both".
        let fsel = p.fuel_selector.as_ref().expect("SR22 has a fuel selector");
        let pos: Vec<&str> = fsel.positions.iter().map(|p| p.key.as_str()).collect();
        assert_eq!(pos, ["off", "left", "right"]);
        // Dual electrical: BAT1/BAT2/ALT1/ALT2 + the pair switches + avionics.
        let elec: Vec<&str> = p.electrical.iter().map(|e| e.key.as_str()).collect();
        for k in [
            "battery1",
            "battery2",
            "alternator1",
            "alternator2",
            "batteries",
            "alternators",
            "avionics",
        ] {
            assert!(elec.contains(&k), "SR22 electrical missing {k}");
        }
        // Flaps 0/50/100.
        let flaps = p.flaps.expect("has flaps");
        assert_eq!(flaps.detents.len(), 3);
        assert_eq!(flaps.detents[2].label, "full");
        assert_eq!(flaps.detents[1].vfe_kts, Some(150.0));
        assert_eq!(p.vs_clean_kts, Some(70.0));
        assert_eq!(p.vne_kts, Some(204.0));
        // Reference card loads from YAML — the SR22's own numbers.
        let r = p.reference.as_ref().expect("sr22 reference");
        assert_eq!(r.best_glide_kts, Some(88.0));
        assert_eq!(r.max_gross_lb, Some(3600.0));
        assert_eq!(p.lights.len(), 3);
        // Conservative map: only the universal-safe entries, and the YAML matches
        // the built-in.
        let yaml_map = &p.checklist_map;
        let builtin_map = &cirrus_sr22().checklist_map;
        assert_eq!(yaml_map.len(), builtin_map.len());
        for (a, b) in yaml_map.iter().zip(builtin_map) {
            assert_eq!(a.match_, b.match_);
        }
        // Still call-out, not mapped: the dual battery/alternator (their file
        // conditions are compound `&&`).
        for e in yaml_map {
            assert!(!e.match_.contains("battery") && !e.match_.contains("generator"));
        }
        // The boost pump, key-start ignition, pitch trim and fuel selector ARE
        // now mapped.
        let matches: Vec<&str> = yaml_map.iter().map(|e| e.match_.as_str()).collect();
        for m in [
            "boost_pump",
            "ignition_key",
            "ignition_on",
            "elevator_trim",
            "fuel_tank_selector",
        ] {
            assert!(matches.contains(&m), "SR22 map missing {m}");
        }
        // Fuel-selector positions carry their dataref values so the checklist can
        // decode `fuel_tank_selector:N` (0=off/1=left/2=right).
        let fsel = p.fuel_selector.as_ref().unwrap();
        assert_eq!(
            fsel.readback.as_deref(),
            Some("sim/cockpit2/fuel/fuel_tank_selector")
        );
        assert_eq!(fsel.positions.iter().filter_map(|p| p.value).count(), 3);
    }
}

/// The X-Plane stock Cessna 172SP Skyhawk.
///
/// Flap detents 0°/10°/20°/30°; Vfe 110 KIAS for the first 10° and 85 KIAS
/// beyond. Fixed gear. Full piston panel + a heading/altitude/VS autopilot.
fn cessna_172sp() -> AircraftProfile {
    let detent = |label: &str, spoken: &str, ratio: f64, vfe: Option<f64>| FlapDetent {
        retract_above_kts: None,
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
        ils: true,
        parking_brake: true,
        cabin_crew: false,
        bus_volts_nominal: None,
        flare_power: None,
        approach_thrust_min: None,
        arrest_lead: None,
        flare_elevator: None,
        cabin_air: None,
        pressurised: Some(false),
        park_brake_valve_set: None,
        electrical: {
            let e = |key: &str, spoken: &str, on: &[&str], off: &[&str], rb: &[&str]| {
                ElectricalSystem {
                    write_datarefs: Vec::new(),
                    toggles: Vec::new(),
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
        vlo_kts: None,
        gear_warning: None,
        // The GA pace is the built-in default — nothing to declare.
        taxi: None,
        throttle_dataref: None,
        tail_strike_deg: None,
        // A light single runs out of elevator before it runs out of tail
        // clearance, and only ever needs the rotation held BACK.
        pitch_authority_down: None,
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
        ils: true,
        parking_brake: true,
        cabin_crew: false,
        bus_volts_nominal: None,
        flare_power: None,
        approach_thrust_min: None,
        arrest_lead: None,
        flare_elevator: None,
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
        vlo_kts: None,
        gear_warning: None,
        taxi: None,
        throttle_dataref: None,
        tail_strike_deg: None,
        // A light single runs out of elevator before it runs out of tail
        // clearance, and only ever needs the rotation held BACK.
        pitch_authority_down: None,
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
    }
}

#[cfg(test)]
mod generic_profile_tests {
    // An unknown airframe flies on its own placards, never the 172's. The
    // fixture is the RV-10's acf, the airframe that showed why (2026-09-11).
    #[test]
    fn an_unknown_airframe_gets_a_profile_from_its_own_acf() {
        let acf = "\
A
1100
ACF
P acf/_ICAO RV10
P acf/_name Van's RV-10
P acf/_Vs_kts 65.000000000
P acf/_Vso_kts 50.000000000
P acf/_Vfe1_kts 96.000000000
P acf/_Vfem_kts 87.000000000
P acf/_Vno_kts 156.000000000
P acf/_Vne_kts 200.000000000
P acf/_Vle_kts 9.9999E4
P acf/_Vx_kts 78.000000000
P acf/_Vy_kts 90.000000000
P acf/_Vbg_kts 85.000000000
P acf/_Va_kts 0.000000000
P acf/_flap_detents 2
P acf/_flap1_dn/0 0.000000000
P acf/_flap1_dn/1 10.000000000
P acf/_flap1_dn/2 30.000000000
P acf/_flap1_dn/3 0.000000000
P acf/_m_empty 1700.000000000
P acf/_m_max 2700.000000000
P _engn/0/_type RCP_INJ
";
        let p = super::generic_from_acf("RV10", acf);
        assert_eq!(p.id, "generic_rv10");
        assert!(
            p.display_name.contains("(no profile)"),
            "{}",
            p.display_name
        );
        assert_eq!(p.vs_clean_kts, Some(65.0));
        assert_eq!(p.vne_kts, Some(200.0));
        assert!(!p.retractable_gear, "Vle 99999 is fixed gear");
        assert!(!p.carb_heat, "injected");
        assert!(p.magnetos, "still a piston");
        let r = p.reference.as_ref().unwrap();
        // 1.3 Vso is 65 — the clean stall. 1.2 Vs is 78, and that is the floor.
        assert_eq!(r.approach_kts, Some(78.0));
        assert_eq!(r.vr_kts, Some(72.0));
        assert_eq!(
            (r.vno_kts, r.vx_kts, r.vy_kts, r.vs0_kts),
            (Some(156.0), Some(78.0), Some(90.0), Some(50.0))
        );
        assert_eq!(
            r.va_kts,
            Some(99.0),
            "a zero placard keeps the base's figure"
        );
        assert_eq!(r.cruise_ias_kts, Some(133.0));
        assert_eq!(r.max_gross_lb, Some(2700.0));
        assert_eq!(r.useful_load_lb, Some(1000.0));
        assert_eq!(r.takeoff_distance_ft, None, "not in the acf: not recited");
        let flaps = p.flaps.as_ref().unwrap();
        let d: Vec<(String, f64, Option<f64>)> = flaps
            .detents
            .iter()
            .map(|d| (d.label.clone(), d.ratio, d.vfe_kts))
            .collect();
        assert_eq!(
            d,
            vec![
                ("up".into(), 0.0, None),
                ("10".into(), 0.3333, Some(96.0)),
                ("full".into(), 1.0, Some(87.0)),
            ]
        );
        assert!(
            p.checklist.is_none(),
            "the 172's checklist is not this aircraft's"
        );
    }

    /// The floor statics are process-wide; the tests that publish them take
    /// this so they never read each other's airframe.
    static FLOOR_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    #[test]
    fn no_target_goes_below_the_stall_floor() {
        let _g = FLOOR_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // The RV-10: Vs 65, floor 78. The 172's 65 kt approach is under it.
        super::publish_speed_floor(Some(65.0));
        assert_eq!(super::speed_floor_kts(), Some(78.0));
        assert_eq!(super::floored_target(65.0, super::speed_floor_kts()), 78.0);
        assert_eq!(
            super::floored_target(100.0, super::speed_floor_kts()),
            100.0
        );
        // A profile with no Vs floors nothing (the old behaviour, honestly).
        super::publish_speed_floor(None);
        assert_eq!(super::floored_target(65.0, super::speed_floor_kts()), 65.0);
    }

    /// The 737: clean Vs 139, Vso 114, Vref 145. The approach is floored
    /// against the LANDING configuration (137), never the clean floor (167)
    /// that flew it to the threshold 22 kt fast (LFLL, 2026-09-17). Without
    /// a Vso the clean floor stands in; without either, nothing floors.
    #[test]
    fn the_approach_floor_is_the_landing_configuration() {
        let _g = FLOOR_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        super::publish_approach_floor(Some(114.0), Some(139.0));
        assert_eq!(super::approach_floor_kts(), Some(137.0));
        assert_eq!(
            super::floored_target(145.0, super::approach_floor_kts()),
            145.0
        );
        super::publish_approach_floor(None, Some(139.0));
        assert_eq!(super::approach_floor_kts(), Some(167.0));
        super::publish_approach_floor(Some(0.0), None);
        assert_eq!(super::approach_floor_kts(), None);
    }

    /// The 737 (Vs 139, Vso 114): flaps up floors at 167, flaps 40 at 137,
    /// flaps 5 (0.375) at 156. A wing with no Vso floors flat at the clean
    /// number; no stall speeds at all floors nothing.
    #[test]
    fn the_floor_follows_the_flaps() {
        let _g = FLOOR_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        super::publish_speed_floor(Some(139.0));
        super::publish_approach_floor(Some(114.0), Some(139.0));
        assert_eq!(super::config_floor_kts(0.0), Some(167.0));
        assert_eq!(super::config_floor_kts(1.0), Some(137.0));
        assert_eq!(super::config_floor_kts(0.375), Some(156.0));
        assert_eq!(
            super::config_floor_kts(f64::NAN),
            Some(167.0),
            "an unreadable flap is the clean floor"
        );
        super::publish_approach_floor(None, Some(139.0));
        assert_eq!(super::config_floor_kts(1.0), Some(167.0));
        super::publish_speed_floor(None);
        super::publish_approach_floor(None, None);
        assert_eq!(super::config_floor_kts(0.5), None);
        super::publish_approach_thrust_min(Some(0.3));
        assert_eq!(super::approach_thrust_min(), Some(0.3));
        super::publish_approach_thrust_min(Some(7.0));
        assert_eq!(
            super::approach_thrust_min(),
            None,
            "a lever ratio outside 0..1 is refused"
        );
    }

    // The probe asks the sim about every name a profile relies on. The RV-10's
    // xlua fuel selector is the case: stock names would have probed fine and
    // the real ones were never asked (2026-09-11).
    #[test]
    fn a_profiles_datarefs_and_commands_are_all_found_by_the_walk() {
        let p = super::load("vans_rv10").expect("RV-10 profile");
        let refs = super::profile_references(&p);
        for must in [
            "laminar/rv10/switch/fuel_tank_pos",
            "laminar/rv10/tank_sel_left",
            "sim/electrical/battery_1_on",
            "sim/cockpit2/switches/taxi_light_on",
            "sim/cockpit2/engine/actuators/prop_rotation_speed_rad_sec_all",
        ] {
            assert!(refs.iter().any(|r| r == must), "missing {must} in {refs:?}");
        }
        assert!(
            !refs.iter().any(|r| r.contains("checklists/")),
            "file paths are not datarefs"
        );
        assert!(
            !refs.iter().any(|r| r.contains('[')),
            "indices dropped: {refs:?}"
        );
        assert!(
            !refs.iter().any(|r| r.contains(' ')),
            "prose is not a dataref: {refs:?}"
        );
        let n = refs.len();
        assert_eq!(
            refs.iter().collect::<std::collections::BTreeSet<_>>().len(),
            n,
            "deduplicated"
        );

        let have: std::collections::HashSet<String> = refs
            .iter()
            .filter(|r| r.starts_with("sim/"))
            .cloned()
            .collect();
        let cmds: std::collections::HashSet<String> = ["laminar/rv10/tank_sel_left".to_string()]
            .into_iter()
            .collect();
        let missing = super::missing_references(&refs, &have, &cmds);
        assert!(
            missing.iter().any(|m| m == "laminar/rv10/tank_sel_right"),
            "{missing:?}"
        );
        assert!(
            !missing.iter().any(|m| m == "laminar/rv10/tank_sel_left"),
            "a command counts"
        );
        assert!(
            !missing.iter().any(|m| m.starts_with("sim/")),
            "{missing:?}"
        );

        assert_eq!(super::probe_summary(&[], 40), None);
        let s = super::probe_summary(&["laminar/rv10/tank_sel_left".into()], 40).unwrap();
        assert!(s.contains("1 of 40") && s.contains("tank sel left"), "{s}");
        let many: Vec<String> = (0..5).map(|i| format!("sim/x/name_{i}")).collect();
        assert!(super::probe_summary(&many, 40)
            .unwrap()
            .contains("and 2 more"));
    }

    #[test]
    fn the_start_gate_scales_with_the_system_voltage() {
        super::publish_bus_volts(None);
        assert!(
            (super::min_start_volts() - 23.0).abs() < 1e-9,
            "24 volt default"
        );
        super::publish_bus_volts(Some(12.0));
        assert!(
            (super::min_start_volts() - 11.5).abs() < 1e-9,
            "a healthy 12 is not low"
        );
        super::publish_bus_volts(None);
    }
}

/// Every hardcoded `sim/...` name in the workspace, checked against the
/// running install's own `DataRefs.txt` and `Commands.txt`. The live probe
/// walks the PROFILE's names on every airframe (`profile_references`); the
/// Rust tools' constants were never probed, and two of them were misspelt for
/// months (`vvi_dial_fpm` in autoland, `compass_heading_deg_magnetic` in the
/// stall recovery, found 2026-09-15), each read failing silently into a
/// fallback that impersonated a measurement. Needs the sim on disk:
/// `cargo test -- --ignored real_install_names`.
#[cfg(test)]
mod real_install_names {
    use std::collections::HashSet;
    use std::path::{Path, PathBuf};

    /// Literals that are deliberately not names: prefixes the code lists by,
    /// and a test fixture that must NOT exist.
    const NOT_NAMES: &[&str] = &["sim/operation/failures/rel_whatsit_thing"];

    fn rust_files(dir: &Path, out: &mut Vec<PathBuf>) {
        let Ok(rd) = std::fs::read_dir(dir) else {
            return;
        };
        for e in rd.flatten() {
            let p = e.path();
            if p.is_dir() {
                if p.file_name().is_some_and(|n| n == "target") {
                    continue;
                }
                rust_files(&p, out);
            } else if p.extension().is_some_and(|x| x == "rs") {
                out.push(p);
            }
        }
    }

    /// `"sim/..."` string literals in one source file, array suffix stripped.
    fn sim_literals(src: &str) -> Vec<String> {
        let mut out = Vec::new();
        let mut rest = src;
        while let Some(i) = rest.find("\"sim/") {
            let body = &rest[i + 1..];
            let Some(end) = body.find('"') else { break };
            let lit = &body[..end];
            let name = match lit.find('[') {
                Some(b) => &lit[..b],
                None => lit,
            };
            if !name.ends_with('_')
                && !name.ends_with('/')
                && name
                    .chars()
                    .all(|c| c.is_ascii_alphanumeric() || c == '/' || c == '_')
            {
                out.push(name.to_string());
            }
            rest = &body[end + 1..];
        }
        out
    }

    fn first_column(path: &Path) -> HashSet<String> {
        std::fs::read_to_string(path)
            .unwrap_or_else(|e| panic!("{}: {e}", path.display()))
            .lines()
            .filter_map(|l| l.split(['\t', ' ']).next())
            .filter(|n| n.starts_with("sim/"))
            .map(str::to_string)
            .collect()
    }

    #[test]
    #[ignore]
    fn every_hardcoded_sim_name_exists_in_the_install() {
        // Same resolution as `tools::autoland::xplane_root` (a bin-side
        // function; this module also compiles into the lib).
        let root = PathBuf::from(std::env::var("XCOPILOT_XPLANE_ROOT").unwrap_or_else(|_| {
            format!("{}/X-Plane 12", std::env::var("HOME").unwrap_or_default())
        }));
        let plugins = root.join("Resources").join("plugins");
        let mut known = first_column(&plugins.join("DataRefs.txt"));
        known.extend(first_column(&plugins.join("Commands.txt")));
        assert!(
            known.len() > 1000,
            "DataRefs.txt looks empty: {}",
            known.len()
        );

        let repo = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let mut files = Vec::new();
        rust_files(&repo.join("src"), &mut files);
        rust_files(&repo.join("crates"), &mut files);
        assert!(files.len() > 20, "found only {} source files", files.len());

        let mut missing = Vec::new();
        for f in &files {
            let src = std::fs::read_to_string(f).unwrap();
            for name in sim_literals(&src) {
                if !known.contains(&name) && !NOT_NAMES.contains(&name.as_str()) {
                    missing.push(format!(
                        "{} <- {}",
                        name,
                        f.strip_prefix(&repo).unwrap().display()
                    ));
                }
            }
        }
        missing.sort();
        missing.dedup();
        assert!(
            missing.is_empty(),
            "names the sim does not have:\n{}",
            missing.join("\n")
        );
    }

    #[test]
    fn literal_scan_strips_indexes_and_skips_prefixes() {
        // Built from pieces so the fixture's names are not themselves
        // literals the install test would scan.
        let q = '"';
        let src = format!("const A: &str = {q}sim/a/b{q}; x({q}sim/c/d[3]{q}); list({q}sim/e/rel_{q}); t({q}sim/f/g/{q});");
        let want: Vec<String> = ["a/b", "c/d"].iter().map(|t| format!("sim/{t}")).collect();
        assert_eq!(sim_literals(&src), want);
    }
}

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
#[cfg(test)]
mod real_install_writes {
    use std::collections::HashSet;
    use std::path::{Path, PathBuf};

    /// Profiles to audit, and the aircraft folder each is written against.
    const AUDIT: &[(&str, &str)] = &[
        ("airbus_a330_300", "Laminar Research/Airbus A330-300"),
        ("boeing_737_800", "Laminar Research/Boeing 737-800"),
        ("md82", "Laminar Research/McDonnell Douglas MD-82"),
        ("citation_x", "Laminar Research/Cessna Citation X"),
        ("cirrus_vision_sf50", "Laminar Research/Cirrus Vision SF50"),
        ("king_air_c90b", "Laminar Research/Beechcraft King Air C90B"),
        ("dhc6_twin_otter", "Laminar Research/DHC-6 Twin Otter"),
        ("lancair_evolution", "Laminar Research/Lancair Evolution"),
    ];

    fn lua_files(dir: &Path, out: &mut Vec<PathBuf>) {
        let Ok(rd) = std::fs::read_dir(dir) else {
            return;
        };
        for e in rd.flatten() {
            let p = e.path();
            if p.is_dir() {
                lua_files(&p, out);
            } else if p.extension().is_some_and(|x| x == "lua") {
                out.push(p);
            }
        }
    }

    /// Stock datarefs the aircraft ASSIGNS TO — anything it writes each frame
    /// is its output, and ours is lost.
    fn aircraft_owned(src: &str) -> HashSet<String> {
        let mut handles = Vec::new();
        let mut rest = src;
        while let Some(i) = rest.find("= find_dataref(\"sim/") {
            let before = &rest[..i];
            let handle = before
                .rsplit(|c: char| c.is_whitespace())
                .find(|t| !t.is_empty())
                .unwrap_or("")
                .to_string();
            let body = &rest[i + "= find_dataref(\"".len()..];
            if let Some(end) = body.find('"') {
                handles.push((handle, body[..end].to_string()));
            }
            rest = &rest[i + 4..];
        }
        handles
            .into_iter()
            .filter(|(h, _)| {
                !h.is_empty() && (src.contains(&format!("{h} =")) || src.contains(&format!("{h}[")))
            })
            .map(|(_, n)| n)
            .collect()
    }

    fn replaced(src: &str) -> HashSet<String> {
        let mut out = HashSet::new();
        let mut rest = src;
        while let Some(i) = rest.find("replace_command(\"") {
            let body = &rest[i + "replace_command(\"".len()..];
            if let Some(end) = body.find('"') {
                out.insert(body[..end].to_string());
            }
            rest = &rest[i + 4..];
        }
        out
    }

    #[test]
    #[ignore]
    fn profile_writes_are_not_silently_owned_by_the_aircraft() {
        // aircraft.rs compiles into the LIB too, where `tools` is the bin's.
        let root = std::env::var("XCOPILOT_XPLANE_ROOT").unwrap_or_else(|_| {
            format!("{}/X-Plane 12", std::env::var("HOME").unwrap_or_default())
        });
        let mut findings = Vec::new();
        let mut audited = 0usize;
        for (id, folder) in AUDIT {
            let dir = Path::new(&root).join("Aircraft").join(folder);
            if !dir.exists() {
                continue;
            }
            let Some(p) = super::load(id) else { continue };
            let mut files = Vec::new();
            lua_files(&dir, &mut files);
            if files.is_empty() {
                continue; // no xlua: nothing can own anything
            }
            audited += 1;
            let src: String = files
                .iter()
                .filter_map(|f| std::fs::read_to_string(f).ok())
                .collect::<Vec<_>>()
                .join("\n");
            // OWNERSHIP is the systems model rewriting a dataref every frame.
            // The aircraft's own AUTOSTART script also assigns to some of
            // them — but that is a script pressing switches, exactly as we
            // do, and it proves the write lands rather than that it is lost.
            // Scanning it flagged the A330's engine masters and ENG MODE
            // knob, both of which demonstrably work (2026-09-20).
            let systems: String = files
                .iter()
                .filter(|f| {
                    !f.to_string_lossy()
                        .to_ascii_lowercase()
                        .contains("autostart")
                })
                .filter_map(|f| std::fs::read_to_string(f).ok())
                .collect::<Vec<_>>()
                .join("\n");
            let owned = aircraft_owned(&systems);
            let repl = replaced(&src);

            // Commands the profile FIRES. A stock command the aircraft
            // neither replaces nor needs is fine; one whose SIBLING is
            // replaced while this one is not is the trap.
            for e in &p.electrical {
                for c in e.on_commands.iter().chain(e.off_commands.iter()) {
                    if !c.starts_with("sim/") || repl.contains(c) {
                        continue;
                    }
                    let sibling = c.rsplit_once('_').map(|(stem, _)| format!("{stem}_toggle"));
                    if sibling.is_some_and(|s| repl.contains(&s)) {
                        findings.push(format!(
                            "{id}: fires {c}, which the aircraft leaves stock while REPLACING \
                             its _toggle — use the toggle with a read-before-fire"
                        ));
                    }
                }
            }
            // Datarefs the profile WRITES that the aircraft owns.
            if let Some(d) = &p.pitot_heat_dataref {
                if owned.contains(d) {
                    findings.push(format!("{id}: writes {d}, which the aircraft rewrites"));
                }
            }
            // Written directly and OBSERVED to work: the A330's engines
            // started on 2026-09-20. The switches script assigns these only
            // inside its own knob handlers, which this scan cannot tell from
            // a per-frame rewrite. Evidence beats the heuristic; anything NEW
            // still has to be justified here.
            const PROVEN: &[&str] = &["sim/cockpit2/engine/actuators/eng_mode_selector"];
            if let Some(a) = &p.airliner_start {
                for n in [&a.mode_selector, &a.masters] {
                    if PROVEN.contains(&n.as_str()) {
                        continue;
                    }
                    if owned.contains(n) {
                        findings.push(format!("{id}: writes {n}, which the aircraft rewrites"));
                    }
                }
            }
        }
        assert!(audited > 0, "no audited aircraft found under {root}");
        assert!(
            findings.is_empty(),
            "silent writes:\n  {}",
            findings.join("\n  ")
        );
    }
}
