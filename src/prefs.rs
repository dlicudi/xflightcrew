//! Tiny persistent pilot-preferences store — the settings the in-sim panel can
//! change at runtime and that must survive a restart (env vars can't carry a
//! choice made mid-flight). One JSON file (`prefs.json`) in the data folder
//! (`sim::paths::prefs_file`), read at startup and rewritten on change. Precedence is always
//! explicit env var > saved preference > built-in default, so a launch script
//! that sets `PIPER_VOICE` still wins.
//!
//! Self-contained on purpose (fs + serde only): it compiles in both the main
//! binary's module tree and the `src/bin/` library surface.

use std::path::Path;

/// `prefs.json` in the data folder (`sim::paths`).
fn prefs_path() -> std::path::PathBuf {
    sim::paths::prefs_file()
}

#[derive(Default, serde::Serialize, serde::Deserialize)]
struct Prefs {
    /// The copilot's own voice — a piper model path (`models/<name>.onnx`),
    /// picked from the panel's Settings tab. `None` = never chosen.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pilot_voice: Option<String>,
    /// The cabin crew's voice, the panel's second picker. `None` = never chosen.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    cabin_voice: Option<String>,
    /// The app mode ("both" | "copilot" | "atc" | "instructor") — the panel's last pick,
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
    std::fs::read_to_string(prefs_path())
        .ok()
        .and_then(|s| serde_json::from_str(&s).ok())
        .unwrap_or_default()
}

/// The saved pilot-voice pick, if it still points at a model on disk — a
/// deleted model file quietly falls back to the default rather than failing
/// the TTS load at startup.
pub fn pilot_voice() -> Option<String> {
    load().pilot_voice.filter(|p| model_on_disk(p))
}

/// A saved voice spec's model file is on disk: the `#N` speaker suffix of a
/// multi-speaker model stripped first (a `vctk#12` pick is its `.onnx`).
fn model_on_disk(spec: &str) -> bool {
    Path::new(spec.split('#').next().unwrap_or(spec)).exists()
}

/// Persist a new pilot-voice pick. Best-effort: a read-only working directory
/// costs the persistence, never the swap itself.
pub fn save_pilot_voice(spec: &str) {
    let mut prefs = load();
    prefs.pilot_voice = Some(spec.to_string());
    store(prefs);
}

/// The saved cabin-voice pick, if its model is still on disk.
pub fn cabin_voice() -> Option<String> {
    load().cabin_voice.filter(|p| model_on_disk(p))
}

/// Persist a new cabin-voice pick. Best-effort, like the pilot's.
pub fn save_cabin_voice(spec: &str) {
    let mut prefs = load();
    prefs.cabin_voice = Some(spec.to_string());
    store(prefs);
}

/// The saved app mode, if any ("both" | "copilot" | "atc" | "instructor").
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
    load().ui_scale.filter(|v| (0..=4).contains(v))
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
    prefs.settings.insert(key.to_string(), rendered.to_string());
    store(prefs);
}

/// Forget one registry setting: its default is in force from the next read.
pub fn remove_setting(key: &str) {
    let mut prefs = load();
    if prefs.settings.remove(key).is_some() {
        store(prefs);
    }
}

/// Persist several registry settings in ONE rewrite (the `.env` import):
/// all of them land or, on a failed write, none do. `Err` names the file.
pub fn save_settings(pairs: &[(String, String)]) -> Result<(), String> {
    let mut prefs = load();
    for (k, v) in pairs {
        prefs.settings.insert(k.clone(), v.clone());
    }
    let json = serde_json::to_string_pretty(&prefs).map_err(|e| format!("prefs: {e}"))?;
    let path = prefs_path();
    std::fs::write(&path, json).map_err(|e| format!("saving {}: {e}", path.display()))
}

fn store(prefs: Prefs) {
    match serde_json::to_string_pretty(&prefs) {
        Ok(json) => {
            let path = prefs_path();
            if let Err(e) = std::fs::write(&path, json) {
                tracing::warn!("prefs: saving {} failed: {e}", path.display());
            }
        }
        Err(e) => tracing::warn!("prefs: serializing failed: {e}"),
    }
}
