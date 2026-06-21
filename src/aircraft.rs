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
    /// Electric/auxiliary fuel pump (`set_fuel_pump`).
    #[serde(default)]
    pub fuel_pump: bool,
    /// Magneto/ignition selector (`set_magnetos`) — for the run-up mag check.
    #[serde(default)]
    pub magnetos: bool,
    /// Carburetor / alternate-air heat (`set_carb_heat`).
    #[serde(default)]
    pub carb_heat: bool,
    /// Fuel tank selector (`set_fuel_selector`): off/left/right/both.
    #[serde(default)]
    pub fuel_selector: bool,
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
    fn unknown_profile_is_none() {
        // No YAML, no built-in → None (caller surfaces a clear error).
        assert!(load("does_not_exist").is_none());
    }

    #[test]
    fn sr22_yaml_loads_and_is_conservative() {
        let p = load("cirrus_sr22").expect("SR22 profile loads");
        assert_eq!(p.id, "cirrus_sr22");
        // Universal-dataref tools on (incl. engine — throttle/mixture/start);
        // SR22-specific systems still off until their tools exist.
        assert!(p.autopilot && p.parking_brake && p.pitot_heat && p.transponder && p.engine);
        assert!(!p.magnetos && !p.fuel_pump && !p.fuel_selector && !p.directional_gyro);
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
        assert_eq!(p.vs_clean_kts, Some(76.0));
        assert_eq!(p.lights.len(), 3);
        // Conservative map: only the universal-safe entries, and the YAML matches
        // the built-in.
        let yaml_map = &p.checklist_map;
        let builtin_map = &cirrus_sr22().checklist_map;
        assert_eq!(yaml_map.len(), builtin_map.len());
        for (a, b) in yaml_map.iter().zip(builtin_map) {
            assert_eq!(a.match_, b.match_);
        }
        // The SR22-specific systems (dual electrical, fuel) are NOT mapped to act
        // yet — they need their own tools. (Mixture maps via ENGN_mixt, but
        // set_mixture writes the universal mixture_ratio_all, so that's safe.)
        for e in yaml_map {
            assert!(!e.match_.contains("battery") && !e.match_.contains("generator")
                && !e.match_.contains("fuel_tank") && !e.match_.contains("boost"));
        }
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
    AircraftProfile {
        id: "cessna_172sp".into(),
        display_name: "Cessna 172SP Skyhawk".into(),
        retractable_gear: false,
        autopilot: true,
        engine: true,
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
        fuel_selector: true,
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

/// The X-Plane stock Cirrus SR22 (Laminar). **Scaffold** — built from the
/// GoofyJP XChecklist file and the POH; datarefs marked "verify" below are
/// unconfirmed against the sim. Conservative by design: only systems on
/// universal X-Plane datarefs are enabled/actioned. The SR22's own systems —
/// dual battery/alternator, the `ENGN_mixt` mixture, the `boost_pump`, the
/// off/left/right fuel selector, key-start ignition — are left off until their
/// datarefs/tools are built and verified in the cockpit.
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
    AircraftProfile {
        id: "cirrus_sr22".into(),
        display_name: "Cirrus SR22".into(),
        retractable_gear: false,
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
        magnetos: false,
        carb_heat: false, // fuel-injected — no carb heat
        fuel_pump: false,
        fuel_selector: false,
        directional_gyro: false, // G1000 glass — no vacuum DG
        flood_lights: false,
        pitch_trim: false,
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
        vs_clean_kts: Some(76.0),
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
        // Acts on universal-dataref systems (parking brake, transponder, pitot,
        // lights, throttle, mixture, flaps). The SR22's own systems — dual
        // battery/alternator, boost pump, off/left/right fuel selector — still
        // verify/call-out until their tools are built (verified datarefs are now
        // known from the rustdecks deck).
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
            ]
        },
    }
}
