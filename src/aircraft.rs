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

/// Everything the tool layer needs to know about one airframe.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AircraftProfile {
    pub id: String,
    pub display_name: String,
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
    /// Static reference figures (V-speeds, weights, limits) the copilot recites
    /// when asked. Per-airframe, from the POH/POM. `None` = no reference answers.
    #[serde(default)]
    pub reference: Option<ReferenceProfile>,
    /// Path to an XChecklist file to interpret (relative to the working dir).
    /// `None` means no checklists for this airframe. The file is read-only input;
    /// the copilot interprets it (see [`crate::checklist`]).
    #[serde(default)]
    pub checklist: Option<String>,
    /// Maps the checklist file's datarefs to curated actions, per-airframe. Empty
    /// means the copilot only verifies/calls out checklist items (never acts).
    #[serde(default)]
    pub checklist_map: Vec<ChecklistMapEntry>,
    /// Exterior/interior lights this airframe has. Empty means no `set_lights` tool.
    #[serde(default)]
    pub lights: Vec<LightProfile>,
    /// Heading/altitude/VS autopilot. `false` means no autopilot tools.
    #[serde(default)]
    pub autopilot: bool,
    /// Piston engine controls (throttle, mixture).
    #[serde(default)]
    pub engine: bool,
    /// Thrust levers (`set_throttle`) without the piston controls — for turbine
    /// aircraft that have a throttle but no mixture/magnetos/piston start. A piston
    /// `engine` already implies a throttle, so it's only set for jets/turboprops.
    #[serde(default)]
    pub throttle: bool,
    /// Turbine start sequence (`start_engines`) — bleed/fuel/ignition/crank with N2
    /// monitoring, as opposed to the piston `engine` group's one-shot start.
    #[serde(default)]
    pub turbine_start: bool,
    /// Procedural power-up (`power_up`) — "all electrical systems on" runs the
    /// electrical bring-up step by step (battery, APU, bleed, buses, generators),
    /// calling out each, rather than a one-shot shortcut.
    #[serde(default)]
    pub auto_prep: bool,
    /// Airliner autoflight extras: `set_speed` (AP airspeed select) and
    /// `autothrottle` (engage/disengage). For jets with an autothrottle.
    #[serde(default)]
    pub autothrottle: bool,
    /// Speedbrakes/spoilers (`speedbrake` extend/retract/arm). For aircraft that
    /// have them (the GA singles don't).
    #[serde(default)]
    pub speedbrake: bool,
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
    /// ILS approach setup (`prepare_ils`) — needs a NAV radio + approach mode.
    #[serde(default)]
    pub ils: bool,
    /// Parking brake (`set_parking_brake`).
    #[serde(default)]
    pub parking_brake: bool,
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
    /// Electric/auxiliary fuel pump (`set_fuel_pump`).
    #[serde(default)]
    pub fuel_pump: bool,
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
    /// Transition altitude (ft MSL): at or above it, "set the altimeter" dials in
    /// standard pressure (29.92 inHg / 1013 hPa) instead of local QNH — that's the
    /// rule above the transition altitude. Defaults to 18,000 ft (US FL180); lower
    /// it per-profile for regions with a lower transition altitude.
    #[serde(default = "default_transition_altitude")]
    pub transition_altitude_ft: f64,
}

/// US transition altitude (FL180). See [`AircraftProfile::transition_altitude_ft`].
fn default_transition_altitude() -> f64 {
    18_000.0
}

/// Engine count for the per-engine accessory tools — 1 (the GA singles) unless
/// a profile says otherwise. See [`AircraftProfile::engines`].
fn default_engines() -> usize {
    1
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
    pub start_dn: Vec<String>,
    /// Per-engine generator-ON command, fired once that engine reaches idle.
    #[serde(default)]
    pub generators: Vec<String>,
    /// N1 (%) at which the engine is lit and stable enough to cut the starter.
    #[serde(default = "default_n1_idle")]
    pub n1_idle: f64,
    /// Max seconds to crank one engine before giving up.
    #[serde(default = "default_turbine_crank_secs")]
    pub crank_secs: f64,
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

/// A flap system as an ordered list of detents, retracted → fully extended.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlapProfile {
    pub detents: Vec<FlapDetent>,
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
}

/// Static reference figures recited on request (V-speeds, weights, limits), from
/// the airframe's POH/POM. Every field is optional so a profile fills in only what
/// it knows; a missing figure is simply not offered. Vne and clean-stall live at
/// the top level (the monitor uses them) and aren't duplicated here.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ReferenceProfile {
    #[serde(default)] pub kind: Option<String>,            // e.g. "single-engine aircraft", "twin-engine jet"
    #[serde(default)] pub vno_kts: Option<f64>,            // max structural cruise
    #[serde(default)] pub va_kts: Option<f64>,             // maneuvering
    #[serde(default)] pub vx_kts: Option<f64>,             // best angle of climb
    #[serde(default)] pub vy_kts: Option<f64>,             // best rate of climb
    #[serde(default)] pub vr_kts: Option<f64>,             // rotate
    #[serde(default)] pub best_glide_kts: Option<f64>,
    #[serde(default)] pub vs0_kts: Option<f64>,            // stall, full flaps
    #[serde(default)] pub approach_kts: Option<f64>,
    #[serde(default)] pub max_crosswind_kts: Option<f64>,  // demonstrated
    #[serde(default)] pub cruise_kts: Option<f64>,
    #[serde(default)] pub best_climb_fpm: Option<f64>,
    #[serde(default)] pub max_gross_lb: Option<f64>,
    #[serde(default)] pub empty_weight_lb: Option<f64>,
    #[serde(default)] pub useful_load_lb: Option<f64>,
    #[serde(default)] pub service_ceiling_ft: Option<f64>,
    #[serde(default)] pub fuel_capacity_gal: Option<f64>,
    #[serde(default)] pub fuel_useable_gal: Option<f64>,
    #[serde(default)] pub endurance_hours: Option<f64>,
    #[serde(default)] pub takeoff_distance_ft: Option<f64>,
    #[serde(default)] pub landing_distance_ft: Option<f64>,
    #[serde(default)] pub max_rpm: Option<f64>,
    #[serde(default)] pub oil_temp_min_f: Option<f64>,
    #[serde(default)] pub oil_temp_max_f: Option<f64>,
    #[serde(default)] pub oil_press_min_psi: Option<f64>,
    #[serde(default)] pub oil_press_max_psi: Option<f64>,
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

/// Built-in profile by id (fallback when no YAML file exists).
pub fn builtin(id: &str) -> Option<AircraftProfile> {
    match id {
        "cessna_172sp" => Some(cessna_172sp()),
        "cirrus_sr22" => Some(cirrus_sr22()),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cessna_yaml_loads_and_matches() {
        // load() prefers profiles/cessna_172sp.yaml (cwd is the crate root in tests).
        let p = load("cessna_172sp").expect("profile loads");
        assert_eq!(p.id, "cessna_172sp");
        assert!(p.autopilot && p.engine && p.barometer && p.com_radio);
        assert!(!p.retractable_gear);
        assert_eq!(p.lights.len(), 5);
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
        assert_eq!(yaml_elec, builtin_elec, "electrical mismatch (YAML vs built-in)");
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
            assert!(fs.positions.iter().all(|p| !p.key.is_empty() && !p.command.is_empty()));
            assert!(!p.checklist_map.is_empty());
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
        for k in ["battery", "generator1", "generator2", "generators", "avionics"] {
            assert!(elec.contains(&k), "c90b electrical missing {k}");
        }
        assert!(p.flood_lights); // cockpit floods wired (generic_lights_switch[0])
        assert!(p.fuel_pump); // standby boost pumps (verified)
        let ts = p.turboprop_start.as_ref().expect("c90b turboprop start");
        assert_eq!(ts.start_up.len(), 2);
        assert_eq!(ts.start_dn.len(), 2);
        assert_eq!(ts.generators.len(), 2);
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
        for k in ["battery1", "battery2", "alternator1", "alternator2", "batteries", "alternators", "avionics"] {
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
        for m in ["boost_pump", "ignition_key", "ignition_on", "elevator_trim", "fuel_tank_selector"] {
            assert!(matches.contains(&m), "SR22 map missing {m}");
        }
        // Fuel-selector positions carry their dataref values so the checklist can
        // decode `fuel_tank_selector:N` (0=off/1=left/2=right).
        let fsel = p.fuel_selector.as_ref().unwrap();
        assert_eq!(fsel.readback.as_deref(), Some("sim/cockpit2/fuel/fuel_tank_selector"));
        assert_eq!(fsel.positions.iter().filter_map(|p| p.value).count(), 3);
    }
}

/// The X-Plane stock Cessna 172SP Skyhawk.
///
/// Flap detents 0°/10°/20°/30°; Vfe 110 KIAS for the first 10° and 85 KIAS
/// beyond. Fixed gear. Full piston panel + a heading/altitude/VS autopilot.
fn cessna_172sp() -> AircraftProfile {
    let detent = |label: &str, spoken: &str, ratio: f64, vfe: Option<f64>| FlapDetent {
        label: label.into(),
        spoken: spoken.into(),
        ratio,
        vfe_kts: vfe,
    };
    let light = |key: &str, spoken: &str, dataref: &str, cmd_on: &str, cmd_off: &str| {
        LightProfile {
            key: key.into(),
            spoken: spoken.into(),
            dataref: dataref.into(),
            command_on: Some(cmd_on.into()),
            command_off: Some(cmd_off.into()),
        }
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
        engines: 1,
        magnetos_per_engine: false,
        flood_lights_dataref: None,
        // Cycle-switch systems (the DC-3 pattern) — not used by the GA singles.
        power_switch: None,
        fuel_selectors: Vec::new(),
        piston_start: None,
        mixture_notch: None,
        turboprop_start: None,
        retractable_gear: false,
        transition_altitude_ft: 18_000.0,
        autopilot: true,
        engine: true,
        throttle: false,
        turbine_start: false,
        auto_prep: false,
        autothrottle: false,
        speedbrake: false,
        barometer: true,
        directional_gyro: true,
        com_radio: true,
        transponder: true,
        ils: true,
        parking_brake: true,
        electrical: {
            let e = |key: &str, spoken: &str, on: &[&str], off: &[&str], rb: &[&str]| ElectricalSystem {
                key: key.into(),
                spoken: spoken.into(),
                on_commands: on.iter().map(|s| s.to_string()).collect(),
                off_commands: off.iter().map(|s| s.to_string()).collect(),
                readback: rb.iter().map(|s| s.to_string()).collect(),
            };
            vec![
                e("battery", "battery", &["sim/electrical/battery_1_on"], &["sim/electrical/battery_1_off"],
                  &["sim/cockpit/electrical/battery_on"]),
                e("alternator", "alternator", &["sim/electrical/generator_1_on"], &["sim/electrical/generator_1_off"],
                  &["sim/cockpit/electrical/generator_on[0]"]),
                // The two AVIONICS switches: BUS 1 (avionics master) + BUS 2 (cross-tie).
                e("avionics", "avionics master",
                  &["sim/systems/avionics_on", "sim/electrical/cross_tie_on"],
                  &["sim/systems/avionics_off", "sim/electrical/cross_tie_off"],
                  &["sim/cockpit/electrical/avionics_on", "sim/cockpit2/electrical/cross_tie"]),
                // Split BAT/ALT rocker — battery + alternator together, NOT avionics.
                e("master", "master",
                  &["sim/electrical/battery_1_on", "sim/electrical/generator_1_on"],
                  &["sim/electrical/battery_1_off", "sim/electrical/generator_1_off"],
                  &["sim/cockpit/electrical/battery_on", "sim/cockpit/electrical/generator_on[0]"]),
            ]
        },
        flood_lights: true,
        fuel_pump: true,
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
        }),
        fuel: Some(FuelProfile {
            reserve_caution_min: 45.0,
            reserve_warning_min: 30.0,
            cruise_burn_kg_hr: 24.0,
        }),
        // 172SP clean stall ~48 KIAS (power-off, flaps up).
        vs_clean_kts: Some(48.0),
        // 172SP Vne (never-exceed) 163 KIAS.
        vne_kts: Some(163.0),
        vlo_kts: None,
        // Reference figures from the X-Plane C172 Pilot's Operating Manual.
        reference: Some(ReferenceProfile {
            kind: Some("single-engine aircraft".into()),
            vno_kts: Some(129.0),
            va_kts: Some(99.0),
            vx_kts: Some(62.0),
            vy_kts: Some(74.0),
            vr_kts: Some(55.0),
            best_glide_kts: Some(68.0),
            vs0_kts: Some(40.0),
            approach_kts: Some(65.0),
            max_crosswind_kts: Some(15.0),
            cruise_kts: Some(124.0),
            best_climb_fpm: Some(730.0),
            max_gross_lb: Some(2558.0),
            empty_weight_lb: Some(1640.0),
            useful_load_lb: Some(918.0),
            service_ceiling_ft: Some(14000.0),
            fuel_capacity_gal: Some(56.0),
            fuel_useable_gal: Some(53.0),
            endurance_hours: Some(4.0),
            takeoff_distance_ft: Some(1630.0),
            landing_distance_ft: Some(1335.0),
            max_rpm: Some(2700.0),
            oil_temp_min_f: Some(100.0),
            oil_temp_max_f: Some(245.0),
            oil_press_min_psi: Some(50.0),
            oil_press_max_psi: Some(90.0),
        }),
        checklist: Some("checklists/cessna_172sp.txt".into()),
        checklist_map: {
            let m = |s: &str, a: ChecklistAction| ChecklistMapEntry { match_: s.into(), action: a };
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
        lights: vec![
            light("nav", "navigation lights", "sim/cockpit2/switches/navigation_lights_on",
                  "sim/lights/nav_lights_on", "sim/lights/nav_lights_off"),
            light("beacon", "beacon", "sim/cockpit2/switches/beacon_on",
                  "sim/lights/beacon_lights_on", "sim/lights/beacon_lights_off"),
            light("strobe", "strobe lights", "sim/cockpit2/switches/strobe_lights_on",
                  "sim/lights/strobe_lights_on", "sim/lights/strobe_lights_off"),
            light("landing", "landing light", "sim/cockpit2/switches/landing_lights_on",
                  "sim/lights/landing_lights_on", "sim/lights/landing_lights_off"),
            light("taxi", "taxi light", "sim/cockpit2/switches/taxi_light_on",
                  "sim/lights/taxi_lights_on", "sim/lights/taxi_lights_off"),
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
        label: label.into(),
        spoken: spoken.into(),
        ratio,
        vfe_kts: vfe,
    };
    let light = |key: &str, spoken: &str, dataref: &str, cmd_on: &str, cmd_off: &str| LightProfile {
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
        engines: 1,
        magnetos_per_engine: false,
        flood_lights_dataref: None,
        // Cycle-switch systems (the DC-3 pattern) — not used by the GA singles.
        power_switch: None,
        fuel_selectors: Vec::new(),
        piston_start: None,
        mixture_notch: None,
        turboprop_start: None,
        retractable_gear: false,
        transition_altitude_ft: 18_000.0,
        // Universal-dataref tools — safe across airframes.
        autopilot: true,
        barometer: true,
        com_radio: true,
        com2_radio: true,
        nav_radio: true,
        transponder: true,
        ils: true,
        parking_brake: true,
        pitot_heat: true,
        // Throttle/mixture/start write universal datarefs (throttle_ratio_all,
        // mixture_ratio_all, the starter command) — confirmed against the
        // rustdecks SR22 deck. (start_engine sets battery 1 + alternator 1 only;
        // the SR22's second battery/alternator need the dual-electrical tool.)
        engine: true,
        throttle: false,
        turbine_start: false,
        auto_prep: false,
        autothrottle: false,
        speedbrake: false,
        // Dual battery/alternator + avionics — datarefs/commands verified against
        // the rustdecks SR22 deck.
        electrical: {
            let e = |key: &str, spoken: &str, on: &[&str], off: &[&str], rb: &[&str]| ElectricalSystem {
                key: key.into(),
                spoken: spoken.into(),
                on_commands: on.iter().map(|s| s.to_string()).collect(),
                off_commands: off.iter().map(|s| s.to_string()).collect(),
                readback: rb.iter().map(|s| s.to_string()).collect(),
            };
            vec![
                e("battery1", "battery one", &["sim/electrical/battery_1_on"], &["sim/electrical/battery_1_off"],
                  &["sim/cockpit2/electrical/battery_on[0]"]),
                e("battery2", "battery two", &["sim/electrical/battery_2_on"], &["sim/electrical/battery_2_off"],
                  &["sim/cockpit2/electrical/battery_on[1]"]),
                e("alternator1", "alternator one", &["sim/electrical/generator_1_on"], &["sim/electrical/generator_1_off"],
                  &["sim/cockpit2/electrical/generator_on[0]"]),
                e("alternator2", "alternator two", &["sim/electrical/generator_2_on"], &["sim/electrical/generator_2_off"],
                  &["sim/cockpit2/electrical/generator_on[1]"]),
                e("batteries", "both batteries",
                  &["sim/electrical/battery_1_on", "sim/electrical/battery_2_on"],
                  &["sim/electrical/battery_1_off", "sim/electrical/battery_2_off"],
                  &["sim/cockpit2/electrical/battery_on[0]", "sim/cockpit2/electrical/battery_on[1]"]),
                e("alternators", "both alternators",
                  &["sim/electrical/generator_1_on", "sim/electrical/generator_2_on"],
                  &["sim/electrical/generator_1_off", "sim/electrical/generator_2_off"],
                  &["sim/cockpit2/electrical/generator_on[0]", "sim/cockpit2/electrical/generator_on[1]"]),
                e("avionics", "avionics master", &["sim/systems/avionics_on"], &["sim/systems/avionics_off"],
                  &["sim/cockpit/electrical/avionics_on"]),
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
        // OFF / LEFT / RIGHT — no "both" (low-wing twin tanks, fed one at a
        // time). Selector commands verified against the rustdecks deck. The
        // standard sim/cockpit2/fuel/fuel_tank_selector (0=off/1=left/2=right) is
        // the readback the GoofyJP checklist trusts, so it both confirms a write
        // and lets the checklist decode the file's `fuel_tank_selector:N` items.
        fuel_selector: Some(FuelSelectorProfile {
            readback: Some("sim/cockpit2/fuel/fuel_tank_selector".into()),
            positions: vec![
                fuel_pos("off", "off", "laminar/sr22/fuel_sel_left_off", Some(0.0)),
                fuel_pos("left", "left", "laminar/sr22/fuel_sel_left_select", Some(1.0)),
                fuel_pos("right", "right", "laminar/sr22/fuel_sel_right_select", Some(2.0)),
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
        }),
        // SR22 ~92 gal usable, ~17 gal/hr cruise (~46 kg/hr). Verify.
        fuel: Some(FuelProfile {
            reserve_caution_min: 45.0,
            reserve_warning_min: 30.0,
            cruise_burn_kg_hr: 46.0,
        }),
        // VS1 (clean) ~76 KIAS; VS0 (landing) ~61.
        // SR22 clean stall (Vs) 70 KIAS, per the X-Plane SR22 POM.
        vs_clean_kts: Some(70.0),
        // SR22 Vne (never-exceed) 204 KIAS.
        vne_kts: Some(204.0),
        vlo_kts: None,
        // Reference figures from the X-Plane Cirrus SR22 Pilot's Operating Manual
        // (the modeled aircraft is the turbo TSIO-550-K, ceiling 25,000 ft).
        reference: Some(ReferenceProfile {
            kind: Some("single-engine aircraft".into()),
            vno_kts: Some(178.0),
            va_kts: Some(133.0),
            vx_kts: Some(88.0),
            vy_kts: Some(101.0),
            vr_kts: Some(73.0),
            best_glide_kts: Some(88.0),
            vs0_kts: Some(60.0),
            approach_kts: None,
            max_crosswind_kts: Some(21.0),
            cruise_kts: Some(180.0), // normal cruise (manual lists 213 KTAS max)
            best_climb_fpm: Some(1200.0),
            max_gross_lb: Some(3600.0),
            empty_weight_lb: Some(2354.0),
            useful_load_lb: Some(1260.0),
            service_ceiling_ft: Some(25000.0),
            fuel_capacity_gal: Some(92.0),
            fuel_useable_gal: None,
            endurance_hours: Some(5.0), // 92 gal / ~18 gph
            takeoff_distance_ft: None,
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
            light("nav", "nav lights", "sim/cockpit2/switches/navigation_lights_on",
                  "sim/lights/nav_lights_on", "sim/lights/nav_lights_off"),
            light("strobe", "strobe lights", "sim/cockpit2/switches/strobe_lights_on",
                  "sim/lights/strobe_lights_on", "sim/lights/strobe_lights_off"),
            light("landing", "landing light", "sim/cockpit2/switches/landing_lights_switch[0]",
                  "sim/lights/landing_lights_on", "sim/lights/landing_lights_off"),
        ],
        // Acts on universal-dataref systems plus the now-wired boost pump,
        // key-start ignition, pitch trim and the off/left/right fuel selector
        // (decoded from the profile's position values). The dual battery/
        // alternator stay call-out — their file conditions are compound `&&`.
        checklist_map: {
            let m = |s: &str, a: ChecklistAction| ChecklistMapEntry { match_: s.into(), action: a };
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
                m("boost_pump", FuelPump),    // laminar/sr22/switch/boost_pump → set_fuel_pump
                m("ignition_key", Ignition),  // actuators/ignition_key: 4=start, 0=off
                m("ignition_on", Ignition),   // mag test: 1=R, 2=L, 3=both
                m("elevator_trim", PitchTrim), // "set T/O" → centre (not ail_trim)
                m("fuel_tank_selector", FuelSelector), // 0=off/1=left/2=right
            ]
        },
    }
}
