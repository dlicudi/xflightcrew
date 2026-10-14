//! "Move these into xfc settings": the `.env` lines that pin registry knobs,
//! written to prefs.json and commented out of the file, ONE action the
//! terminal's Settings screen offers after showing the list.
//!
//! Nothing here runs by itself. At boot the environment still pins (env >
//! pref > default, `settings`); this module only reads the `.env` files the
//! boot loaded (`Config::from_env`: the working directory's or a parent's,
//! then the data folder's) to say which pins came from them. The shell's own
//! environment is never touched: a knob whose value in force is not the
//! file's (the shell set another) is not the file's to move.

use std::path::{Path, PathBuf};
use std::sync::Mutex;

use crate::settings::{self, Persist, Setting};

/// One `.env` line that pins a registry knob.
#[derive(Clone, Debug, PartialEq)]
pub struct Move {
    pub key: &'static str,
    pub env: &'static str,
    pub label: &'static str,
    /// The value as the file has it.
    pub value: String,
    /// The `.env` it is in.
    pub file: PathBuf,
    /// What prefs.json would store, or why it cannot be moved (a value the
    /// registry would refuse stays in the file, said).
    pub stored: Result<String, String>,
}

/// The `.env` files the boot loads, in its order: the first `.env` from the
/// working directory up (dotenvy's search), then the data folder's.
pub fn env_files() -> Vec<PathBuf> {
    let mut out = Vec::new();
    if let Ok(mut dir) = std::env::current_dir() {
        loop {
            let candidate = dir.join(sim::paths::ENV_FILE_NAME);
            if candidate.is_file() {
                out.push(candidate);
                break;
            }
            if !dir.pop() {
                break;
            }
        }
    }
    let data = sim::paths::env_file();
    if data.is_file() && !out.iter().any(|p| same_file(p, &data)) {
        out.push(data);
    }
    out
}

fn same_file(a: &Path, b: &Path) -> bool {
    match (a.canonicalize(), b.canonicalize()) {
        (Ok(x), Ok(y)) => x == y,
        _ => a == b,
    }
}

/// The `KEY=value` pairs of a `.env` text, as dotenvy reads them.
pub fn pairs(text: &str) -> Vec<(String, String)> {
    dotenvy::from_read_iter(text.as_bytes())
        .filter_map(Result::ok)
        .collect()
}

/// What prefs.json would store for `raw`, judged as [`settings::set`] judges
/// an edit: the kind reads it, the kind and the knob's own check accept it.
pub fn judge(s: &Setting, raw: &str) -> Result<String, String> {
    let v = s
        .kind
        .parse(raw)
        .ok_or_else(|| format!("{} cannot be {raw:?}", s.label))?;
    s.kind.validate(&v)?;
    (s.check)(&v)?;
    let stored = s.kind.store(&v);
    if matches!(s.kind, settings::Kind::Voice | settings::Kind::Path { .. })
        && stored.trim().is_empty()
    {
        return Err("blank in .env: left there".into());
    }
    Ok(stored)
}

/// The moves for these files' texts (pure): every line naming a registry
/// knob's variable whose value in force (`in_force`) IS the line's, the
/// first file winning as dotenvy's first load does. A knob whose pick
/// outranks its variable (the copilot's and the cabin's voices) is never
/// pinned and never moved: its line is a default the pick already beats.
pub fn plan_for(
    files: &[(PathBuf, String)],
    in_force: impl Fn(&str) -> Option<String>,
) -> Vec<Move> {
    let mut out: Vec<Move> = Vec::new();
    for (file, text) in files {
        for (k, v) in pairs(text) {
            let Some(s) = settings::ALL
                .iter()
                .find(|s| s.env == k && !s.pick_beats_env && !s.env.is_empty())
            else {
                continue;
            };
            if out.iter().any(|m| m.env == s.env) || in_force(&k).as_deref() != Some(v.as_str()) {
                continue;
            }
            out.push(Move {
                key: s.key,
                env: s.env,
                label: s.label,
                stored: judge(s, &v),
                value: v,
                file: file.clone(),
            });
        }
    }
    out
}

/// The plan for this process: its `.env` files against its environment.
/// Cached: the files only change through [`apply`], which clears it.
pub fn pending() -> Vec<Move> {
    static CACHE: Mutex<Option<Vec<Move>>> = Mutex::new(None);
    let mut c = CACHE.lock().unwrap_or_else(|e| e.into_inner());
    if let Some(m) = c.as_ref().filter(|_| !stale()) {
        return m.clone();
    }
    let files: Vec<(PathBuf, String)> = env_files()
        .into_iter()
        .filter_map(|p| std::fs::read_to_string(&p).ok().map(|t| (p, t)))
        .collect();
    let moves = plan_for(&files, |k| std::env::var(k).ok());
    *c = Some(moves.clone());
    CLEAR.store(false, std::sync::atomic::Ordering::Relaxed);
    moves
}

/// Set by [`apply`]: the next [`pending`] reads the files again.
static CLEAR: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// The moves that can be made (the refused stay listed, not moved).
pub fn movable(moves: &[Move]) -> Vec<&Move> {
    moves.iter().filter(|m| m.stored.is_ok()).collect()
}

/// A `.env` text with these variables' lines commented out (pure), each
/// prefixed `# moved to xfc settings <date>: `; every other line, comment
/// and blank kept as it was.
pub fn comment_out(text: &str, envs: &[&str], date: &str) -> String {
    let mut out = String::with_capacity(text.len() + 64);
    for line in text.split_inclusive('\n') {
        let body = line.trim_end_matches(['\n', '\r']);
        let t = body.trim_start();
        let t = t.strip_prefix("export ").unwrap_or(t).trim_start();
        let named = t
            .split_once('=')
            .map(|(k, _)| k.trim())
            .is_some_and(|k| envs.contains(&k));
        if named && !t.starts_with('#') {
            out.push_str(&format!("# moved to xfc settings {date}: {body}"));
            out.push_str(&line[body.len()..]);
        } else {
            out.push_str(line);
        }
    }
    out
}

/// A backup path beside `file` that does not exist yet:
/// `.env.bak-2026-10-14`, then `-2`, `-3`...
fn backup_path(file: &Path, date: &str) -> PathBuf {
    let name = file
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| sim::paths::ENV_FILE_NAME.into());
    let first = file.with_file_name(format!("{name}.bak-{date}"));
    if !first.exists() {
        return first;
    }
    (2..)
        .map(|n| file.with_file_name(format!("{name}.bak-{date}-{n}")))
        .find(|p| !p.exists())
        .unwrap_or(first)
}

/// Make the moves: back each `.env` up, write the values to prefs.json in
/// one rewrite (the app mode and cabin staffing through their own fields),
/// then comment their lines out. Stops at the first failure with nothing
/// half-done that matters: a failed backup writes nothing; a failed prefs
/// write leaves every `.env` as it was. `Ok` says what happened, `Err` why
/// not, both for the screen.
pub fn apply(moves: &[Move]) -> Result<String, String> {
    let ok: Vec<&Move> = movable(moves);
    if ok.is_empty() {
        return Err("nothing in .env can be moved".into());
    }
    let date = crate::identify::today();
    let mut files: Vec<&Path> = Vec::new();
    for m in &ok {
        if !files.contains(&m.file.as_path()) {
            files.push(&m.file);
        }
    }
    let mut backups = Vec::new();
    for f in &files {
        let to = backup_path(f, &date);
        std::fs::copy(f, &to).map_err(|e| format!("backing up {}: {e}", f.display()))?;
        backups.push(to);
    }
    let mut map: Vec<(String, String)> = Vec::new();
    for m in &ok {
        let stored = m.stored.clone().unwrap_or_default();
        let persist = settings::ALL
            .iter()
            .find(|s| s.key == m.key)
            .map(|s| s.persist);
        match (persist, m.key) {
            (Some(Persist::Map), _) => map.push((m.key.to_string(), stored)),
            (Some(Persist::Own), "mode") => crate::prefs::save_mode(&stored),
            (Some(Persist::Own), "cabin_crew") => crate::prefs::save_cabin_crew(&stored),
            _ => {}
        }
    }
    crate::prefs::save_settings(&map)?;
    for f in &files {
        let envs: Vec<&str> = ok.iter().filter(|m| m.file == *f).map(|m| m.env).collect();
        let text =
            std::fs::read_to_string(f).map_err(|e| format!("reading {}: {e}", f.display()))?;
        std::fs::write(f, comment_out(&text, &envs, &date))
            .map_err(|e| format!("rewriting {}: {e}", f.display()))?;
    }
    CLEAR.store(true, std::sync::atomic::Ordering::Relaxed);
    tracing::info!(moved = ok.len(), backups = ?backups, "settings moved out of .env");
    Ok(format!(
        "moved {} into xfc settings, in force from the next start; backup {}",
        ok.len(),
        backups
            .iter()
            .map(|p| crate::home::tilde(p))
            .collect::<Vec<_>>()
            .join(", ")
    ))
}

/// Whether [`apply`] ran since the last [`pending`]: the caller re-reads.
pub fn stale() -> bool {
    CLEAR.load(std::sync::atomic::Ordering::Relaxed)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn files(text: &str) -> Vec<(PathBuf, String)> {
        vec![(PathBuf::from("/tmp/x/.env"), text.to_string())]
    }

    /// The pins a `.env` holds are listed with what prefs.json would store;
    /// a line the shell overrode, a variable the registry does not know, and
    /// the copilot's voice (its pick beats the line) are not.
    #[test]
    fn the_plan_lists_the_pins_the_file_made() {
        let text = "# a comment\nXCOPILOT_EFB=true\nXCOPILOT_TRAFFIC=ours\n\
                    PIPER_VOICE=models/en_US-ryan-high.onnx\nXCOPILOT_LISTEN=ptt\n\
                    XCOPILOT_TAXI_GUIDE=false\nXCOPILOT_VAD_RMS=0.5\n";
        let env = |k: &str| match k {
            "XCOPILOT_EFB" => Some("true".to_string()),
            "XCOPILOT_TRAFFIC" => Some("ours".to_string()),
            "PIPER_VOICE" => Some("models/en_US-ryan-high.onnx".to_string()),
            "XCOPILOT_LISTEN" => Some("ptt".to_string()),
            // The shell said otherwise: dotenvy kept the shell's.
            "XCOPILOT_TAXI_GUIDE" => Some("true".to_string()),
            "XCOPILOT_VAD_RMS" => Some("0.5".to_string()),
            _ => None,
        };
        let moves = plan_for(&files(text), env);
        let keys: Vec<&str> = moves.iter().map(|m| m.key).collect();
        assert_eq!(keys, ["efb", "traffic", "vad_threshold"]);
        assert_eq!(moves[0].stored, Ok("on".to_string()));
        assert_eq!(moves[1].stored, Ok("ours".to_string()));
        // Out of the registry's range: listed, refused, left in the file.
        assert!(moves[2].stored.is_err(), "{:?}", moves[2].stored);
        assert_eq!(movable(&moves).len(), 2);
    }

    /// Only the named lines are commented, dated; comments, blanks, other
    /// variables, an `export` form and the file's last newline stay.
    #[test]
    fn commenting_out_touches_only_the_moved_lines() {
        let text = "# keep\nXCOPILOT_EFB=true\n\nexport XCOPILOT_TRAFFIC=ours\nWHISPER_MODEL=x.bin\n# XCOPILOT_EFB=old";
        let out = comment_out(text, &["XCOPILOT_EFB", "XCOPILOT_TRAFFIC"], "2026-10-14");
        assert_eq!(
            out,
            "# keep\n# moved to xfc settings 2026-10-14: XCOPILOT_EFB=true\n\n\
             # moved to xfc settings 2026-10-14: export XCOPILOT_TRAFFIC=ours\nWHISPER_MODEL=x.bin\n# XCOPILOT_EFB=old"
        );
        // Read back, the moved knobs are gone from the file's pairs.
        let left: Vec<String> = pairs(&out).into_iter().map(|(k, _)| k).collect();
        assert_eq!(left, ["WHISPER_MODEL"]);
    }

    /// The whole action on a real file: backed up, the values in prefs.json's
    /// shape, the lines commented; a refused value stays live in the file.
    /// Writes only under a temp folder (the prefs write is not exercised
    /// here: it is the data folder's, covered by the restart round trip).
    #[test]
    fn a_backup_is_made_beside_the_file_before_it_is_rewritten() {
        let dir = std::env::temp_dir().join(format!("xfc-env-import-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let f = dir.join(".env");
        std::fs::write(&f, "A=1\n").unwrap();
        let b1 = backup_path(&f, "2026-10-14");
        assert_eq!(b1, dir.join(".env.bak-2026-10-14"));
        std::fs::write(&b1, "x").unwrap();
        assert_eq!(
            backup_path(&f, "2026-10-14"),
            dir.join(".env.bak-2026-10-14-2")
        );
        let _ = std::fs::remove_dir_all(&dir);
    }
}
