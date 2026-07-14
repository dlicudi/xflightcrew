//! Black-box flight recorder — structured JSONL for post-flight analysis.
//!
//! One line per event in `logs/blackbox.jsonl`: `{"t": <unix secs>, "kind":
//! "state" | "autoland" | "recover" | ..., ...fields}`. The monitor records the
//! aircraft state ~1/s; the autoland/recover watchers record their per-tick
//! guidance and control commands. Unlike the tracing log this is machine-parsable
//! (jq/pandas) and keyed by wall-clock, so an incident can be reconstructed
//! tick-by-tick. Append-only, best-effort: recording must never affect the loop.
//! `XCOPILOT_BLACKBOX=off` disables it.

use std::io::Write;
use std::sync::Mutex;

static FILE: Mutex<Option<Option<std::fs::File>>> = Mutex::new(None);

/// Append one record. `fields` must be a JSON object; `t` and `kind` are added.
pub fn record(kind: &str, fields: serde_json::Value) {
    let ts = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0);
    let mut line = serde_json::json!({ "t": (ts * 10.0).round() / 10.0, "kind": kind });
    if let (Some(out), Some(add)) = (line.as_object_mut(), fields.as_object()) {
        for (k, v) in add {
            out.insert(k.clone(), v.clone());
        }
    }
    let mut guard = FILE.lock().unwrap_or_else(|e| e.into_inner());
    // First use: open once (or record the decision not to — env off / no logs dir).
    if guard.is_none() {
        let enabled = !matches!(
            std::env::var("XCOPILOT_BLACKBOX").unwrap_or_default().to_lowercase().as_str(),
            "off" | "0" | "false"
        );
        *guard = Some(enabled.then(|| {
            let _ = std::fs::create_dir_all("logs");
            std::fs::OpenOptions::new().create(true).append(true).open("logs/blackbox.jsonl").ok()
        }).flatten());
    }
    if let Some(Some(f)) = guard.as_mut() {
        let _ = writeln!(f, "{line}");
    }
}
