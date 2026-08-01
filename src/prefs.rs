//! Tiny persistent pilot-preferences store — the settings the in-sim panel can
//! change at runtime and that must survive a restart (env vars can't carry a
//! choice made mid-flight). One JSON file (`prefs.json`) in the working
//! directory, read at startup and rewritten on change. Precedence is always
//! explicit env var > saved preference > built-in default, so a launch script
//! that sets `PIPER_VOICE` still wins.
//!
//! Self-contained on purpose (fs + serde only): it compiles in both the main
//! binary's module tree and the `src/bin/` library surface.

use std::path::Path;

const PREFS_PATH: &str = "prefs.json";

#[derive(Default, serde::Serialize, serde::Deserialize)]
struct Prefs {
    /// The copilot's own voice — a piper model path (`models/<name>.onnx`),
    /// picked from the panel's Settings tab. `None` = never chosen.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pilot_voice: Option<String>,
    /// The app mode ("both" | "copilot" | "atc") — the panel's last pick,
    /// honored at the next boot unless `XCOPILOT_MODE` overrides it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    mode: Option<String>,
    /// Cabin crew staffing ("auto" | "aboard" | "off") — the panel's last
    /// pick, honored at the next boot unless `XCOPILOT_CABIN_CREW` overrides.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    cabin_crew: Option<String>,
    /// The panel's UI scale index (0 = 100%, 1 = 125%, 2 = 150%). Purely a
    /// panel concern — the app only remembers it and republishes across
    /// plugin reloads and boots.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    ui_scale: Option<i64>,
    /// Everything the settings registry owns ([`crate::settings`]), stored as
    /// `key -> rendered value`. A map rather than more typed fields because the
    /// registry is the schema: a knob is declared once there, and this file
    /// simply remembers what was chosen. Unknown keys are preserved on
    /// rewrite, so a pref saved by a newer build survives an older one.
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    settings: std::collections::BTreeMap<String, String>,
}

fn load() -> Prefs {
    std::fs::read_to_string(PREFS_PATH)
        .ok()
        .and_then(|s| serde_json::from_str(&s).ok())
        .unwrap_or_default()
}

/// The saved pilot-voice pick, if it still points at a model on disk — a
/// deleted model file quietly falls back to the default rather than failing
/// the TTS load at startup.
pub fn pilot_voice() -> Option<String> {
    load().pilot_voice.filter(|p| Path::new(p).exists())
}

/// Persist a new pilot-voice pick. Best-effort: a read-only working directory
/// costs the persistence, never the swap itself.
pub fn save_pilot_voice(spec: &str) {
    let mut prefs = load();
    prefs.pilot_voice = Some(spec.to_string());
    store(prefs);
}

/// The saved app mode, if any ("both" | "copilot" | "atc").
pub fn mode() -> Option<String> {
    load().mode
}

/// Persist a new app-mode pick (the panel toggle). Best-effort, like the
/// voice: a failed write costs the persistence, never the live switch.
pub fn save_mode(mode: &str) {
    let mut prefs = load();
    prefs.mode = Some(mode.to_string());
    store(prefs);
}

/// The saved cabin-crew staffing, if any ("auto" | "aboard" | "off").
pub fn cabin_crew() -> Option<String> {
    load().cabin_crew
}

/// Persist a new cabin-crew staffing pick (the panel toggle). Best-effort,
/// like the others: a failed write costs the persistence, never the switch.
pub fn save_cabin_crew(staffing: &str) {
    let mut prefs = load();
    prefs.cabin_crew = Some(staffing.to_string());
    store(prefs);
}

/// The saved panel UI scale index (0/1/2), if any.
pub fn ui_scale() -> Option<i64> {
    load().ui_scale.filter(|v| (0..=2).contains(v))
}

/// Persist a new panel UI scale pick. Best-effort, like the others.
pub fn save_ui_scale(v: i64) {
    let mut prefs = load();
    prefs.ui_scale = Some(v);
    store(prefs);
}

/// A saved registry setting, as it was rendered when stored.
pub fn setting(key: &str) -> Option<String> {
    load().settings.get(key).cloned()
}

/// Persist one registry setting. Best-effort like the rest: a read-only
/// working directory costs the persistence, never the live change.
pub fn save_setting(key: &str, rendered: &str) {
    let mut prefs = load();
    prefs
        .settings
        .insert(key.to_string(), rendered.to_string());
    store(prefs);
}

fn store(prefs: Prefs) {
    match serde_json::to_string_pretty(&prefs) {
        Ok(json) => {
            if let Err(e) = std::fs::write(PREFS_PATH, json) {
                tracing::warn!("prefs: saving {PREFS_PATH} failed: {e}");
            }
        }
        Err(e) => tracing::warn!("prefs: serializing failed: {e}"),
    }
}
