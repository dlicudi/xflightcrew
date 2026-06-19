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
    /// Whether the gear retracts. `false` (e.g. the 172) means no `gear` tool.
    #[serde(default)]
    #[allow(dead_code)]
    pub retractable_gear: bool,
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
    /// The 0/1 switch dataref controlling it.
    pub dataref: String,
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
    let light = |key: &str, spoken: &str, dataref: &str| LightProfile {
        key: key.into(),
        spoken: spoken.into(),
        dataref: dataref.into(),
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
        flaps: Some(FlapProfile {
            detents: vec![
                detent("up", "up", 0.0, None),
                detent("10", "ten degrees", 1.0 / 3.0, Some(110.0)),
                detent("20", "twenty degrees", 2.0 / 3.0, Some(85.0)),
                detent("full", "full", 1.0, Some(85.0)),
            ],
        }),
        lights: vec![
            light("nav", "navigation lights", "sim/cockpit2/switches/navigation_lights_on"),
            light("beacon", "beacon", "sim/cockpit2/switches/beacon_on"),
            light("strobe", "strobe lights", "sim/cockpit2/switches/strobe_lights_on"),
            light("landing", "landing light", "sim/cockpit2/switches/landing_lights_on"),
            light("taxi", "taxi light", "sim/cockpit2/switches/taxi_light_on"),
        ],
    }
}
