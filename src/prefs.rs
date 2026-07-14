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
    match serde_json::to_string_pretty(&prefs) {
        Ok(json) => {
            if let Err(e) = std::fs::write(PREFS_PATH, json) {
                tracing::warn!("prefs: saving {PREFS_PATH} failed: {e}");
            }
        }
        Err(e) => tracing::warn!("prefs: serializing failed: {e}"),
    }
}
