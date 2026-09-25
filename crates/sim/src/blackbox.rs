//! Black-box flight recorder — structured JSONL for post-flight analysis.
//!
//! One line per event in `logs/blackbox.jsonl`: `{"t": <unix secs>, "kind":
//! "state" | "autoland" | "recover" | ..., ...fields}`. The monitor records the
//! aircraft state ~1/s; the autoland/recover watchers record their per-tick
//! guidance and control commands. Unlike the tracing log this is machine-parsable
//! (jq/pandas) and keyed by wall-clock, so an incident can be reconstructed
//! tick-by-tick. Append-only, best-effort: recording must never affect the loop.
//! `XCOPILOT_BLACKBOX=off` disables it.
//!
//! **Sorties** (2026-09-25, for unattended testing): between `begin_sortie`
//! and `end_sortie` every record carries `"sortie": <id>` and is ALSO written
//! to `logs/sorties/<id>.jsonl`, so one run is one file a runner can grade.
//! The main file is rotated at first open past [`ROTATE_BYTES`] (it had grown
//! to 311 MB and every debrief read all of it). A TAP sees each record after
//! it is written and may return more records (the live scorer's
//! `stage_result`); those are written untapped, so a tap never recurses.

use std::io::Write;
use std::sync::Mutex;

/// Past this the main file is renamed aside at first open.
pub const ROTATE_BYTES: u64 = 200 * 1024 * 1024;

type Tap = fn(&serde_json::Value) -> Vec<(String, serde_json::Value)>;

struct Sortie {
    id: String,
    file: Option<std::fs::File>,
}

static FILE: Mutex<Option<Option<std::fs::File>>> = Mutex::new(None);
static SORTIE: Mutex<Option<Sortie>> = Mutex::new(None);
static TAP: Mutex<Option<Tap>> = Mutex::new(None);

fn enabled() -> bool {
    !matches!(
        std::env::var("XCOPILOT_BLACKBOX")
            .unwrap_or_default()
            .to_lowercase()
            .as_str(),
        "off" | "0" | "false"
    )
}

fn now_s() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

/// Install the tap (the app's live scorer). One tap; a second replaces it.
pub fn set_tap(tap: Tap) {
    *TAP.lock().unwrap_or_else(|e| e.into_inner()) = Some(tap);
}

/// The running sortie's id, if any.
pub fn sortie() -> Option<String> {
    SORTIE.lock().unwrap_or_else(|e| e.into_inner()).as_ref().map(|s| s.id.clone())
}

/// Start a sortie: `label` names the mission ("full_auto", "heli_route").
/// A sortie already running is ended first ("superseded"). Returns the id.
pub fn begin_sortie(label: &str, detail: serde_json::Value) -> String {
    if sortie().is_some() {
        end_sortie("superseded", serde_json::json!({}));
    }
    let secs = now_s() as u64;
    let id = format!("{secs}-{label}");
    let file = enabled()
        .then(|| {
            let _ = std::fs::create_dir_all("logs/sorties");
            std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(format!("logs/sorties/{id}.jsonl"))
                .ok()
        })
        .flatten();
    *SORTIE.lock().unwrap_or_else(|e| e.into_inner()) = Some(Sortie { id: id.clone(), file });
    let mut d = detail;
    if let Some(o) = d.as_object_mut() {
        o.insert("event".into(), "begin".into());
        o.insert("label".into(), label.into());
    }
    record("sortie", d);
    id
}

/// End the running sortie with its outcome ("completed", "gave_up", ...).
/// The end record is the last line of its file. No sortie: nothing.
pub fn end_sortie(outcome: &str, detail: serde_json::Value) {
    if sortie().is_none() {
        return;
    }
    let mut d = detail;
    if let Some(o) = d.as_object_mut() {
        o.insert("event".into(), "end".into());
        o.insert("outcome".into(), outcome.into());
    }
    record("sortie", d);
    *SORTIE.lock().unwrap_or_else(|e| e.into_inner()) = None;
}

fn open_main() -> Option<std::fs::File> {
    if !enabled() {
        return None;
    }
    let _ = std::fs::create_dir_all("logs");
    let path = std::path::Path::new("logs/blackbox.jsonl");
    if std::fs::metadata(path).is_ok_and(|m| m.len() > ROTATE_BYTES) {
        let _ = std::fs::rename(path, format!("logs/blackbox.{}.jsonl", now_s() as u64));
    }
    std::fs::OpenOptions::new().create(true).append(true).open(path).ok()
}

fn write(kind: &str, fields: &serde_json::Value) -> serde_json::Value {
    let ts = now_s();
    let mut line = serde_json::json!({ "t": (ts * 10.0).round() / 10.0, "kind": kind });
    let mut sortie = SORTIE.lock().unwrap_or_else(|e| e.into_inner());
    if let Some(out) = line.as_object_mut() {
        if let Some(add) = fields.as_object() {
            for (k, v) in add {
                out.insert(k.clone(), v.clone());
            }
        }
        if let Some(s) = sortie.as_ref() {
            out.insert("sortie".into(), s.id.clone().into());
        }
    }
    {
        let mut guard = FILE.lock().unwrap_or_else(|e| e.into_inner());
        if guard.is_none() {
            *guard = Some(open_main());
        }
        if let Some(Some(f)) = guard.as_mut() {
            let _ = writeln!(f, "{line}");
        }
    }
    if let Some(Sortie { file: Some(f), .. }) = sortie.as_mut() {
        let _ = writeln!(f, "{line}");
    }
    line
}

/// Append one record. `fields` must be a JSON object; `t` and `kind` are added
/// (and `sortie` while one runs).
pub fn record(kind: &str, fields: serde_json::Value) {
    let line = write(kind, &fields);
    let tap = *TAP.lock().unwrap_or_else(|e| e.into_inner());
    if let Some(tap) = tap {
        for (k, f) in tap(&line) {
            write(&k, &f);
        }
    }
}
