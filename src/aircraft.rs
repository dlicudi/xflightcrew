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
    /// Electrical systems — battery, avionics master (`set_electrical`).
    #[serde(default)]
    pub electrical: bool,
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
    }

    #[test]
    fn unknown_profile_is_none() {
        // No YAML, no built-in → None (caller surfaces a clear error).
        assert!(load("does_not_exist").is_none());
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
        electrical: true,
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
