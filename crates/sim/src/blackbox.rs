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
//!
//! **One record is one write** (#144, 2026-09-28): the line is formatted
//! whole, newline included, and goes out in a single `write_all` under an
//! advisory file lock ([`append_line`]). A `writeln!` of a `Value` is one
//! `write(2)` per token, and a second PROCESS appending the same file (the
//! sortie runner's own `api_load`) interleaved with it byte by byte: 70
//! corrupt lines in one day's log, each pair losing two records.

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

/// Append one whole line to an append-mode file: one `write_all` of the
/// formatted bytes under an exclusive advisory lock, so neither another
/// thread's handle nor another process can land inside it. Best-effort: a
/// lock that fails still writes.
pub fn append_line(f: &mut std::fs::File, line: &[u8]) {
    let locked = f.lock().is_ok();
    let _ = f.write_all(line);
    if locked {
        let _ = f.unlock();
    }
}

/// A record as the bytes of its line, newline included.
fn line_bytes(line: &serde_json::Value) -> Vec<u8> {
    let mut buf = serde_json::to_vec(line).unwrap_or_default();
    buf.push(b'\n');
    buf
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
    let bytes = line_bytes(&line);
    {
        let mut guard = FILE.lock().unwrap_or_else(|e| e.into_inner());
        if guard.is_none() {
            *guard = Some(open_main());
        }
        if let Some(Some(f)) = guard.as_mut() {
            append_line(f, &bytes);
        }
    }
    if let Some(Sortie { file: Some(f), .. }) = sortie.as_mut() {
        append_line(f, &bytes);
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

#[cfg(test)]
mod tests {
    use super::*;

    /// #144: many writers, each with its OWN append handle to one file (as
    /// two processes are), each record long enough to be many tokens. Every
    /// line must parse and every record must arrive whole, exactly once.
    #[test]
    fn concurrent_appenders_never_interleave_a_line() {
        let dir = std::env::temp_dir().join(format!("bb-hammer-{}", std::process::id()));
        let _ = std::fs::create_dir_all(&dir);
        let path = dir.join("blackbox.jsonl");
        let _ = std::fs::remove_file(&path);
        const THREADS: u64 = 16;
        const PER: u64 = 400;
        let handles: Vec<_> = (0..THREADS)
            .map(|w| {
                let path = path.clone();
                std::thread::spawn(move || {
                    let mut f = std::fs::OpenOptions::new()
                        .create(true)
                        .append(true)
                        .open(&path)
                        .unwrap();
                    for i in 0..PER {
                        let rec = serde_json::json!({
                            "t": 1790580993.8, "kind": "autoland", "writer": w, "i": i,
                            "event": "power", "throttle_before": 0.27267703,
                            "throttle_command": 0.28063357596207755, "vvi": -42.181896,
                            "top": vec!["sim/flightmodel2/misc/has_crashed x58"; 8],
                            "write_ok": true,
                        });
                        append_line(&mut f, &line_bytes(&rec));
                    }
                })
            })
            .collect();
        for h in handles {
            h.join().unwrap();
        }
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_dir_all(&dir);
        let mut seen = std::collections::HashSet::new();
        for line in text.lines() {
            let v: serde_json::Value = serde_json::from_str(line)
                .unwrap_or_else(|e| panic!("unparsable line ({e}): {line}"));
            assert!(seen.insert((v["writer"].as_u64().unwrap(), v["i"].as_u64().unwrap())));
        }
        assert_eq!(seen.len() as u64, THREADS * PER);
    }

    #[test]
    fn a_line_is_formatted_whole_with_its_newline() {
        let rec = serde_json::json!({"kind": "api_load", "top": ["a", "b"]});
        let b = line_bytes(&rec);
        assert_eq!(*b.last().unwrap(), b'\n');
        assert_eq!(b.iter().filter(|&&c| c == b'\n').count(), 1);
        let back: serde_json::Value = serde_json::from_slice(&b[..b.len() - 1]).unwrap();
        assert_eq!(back, rec);
    }
}
