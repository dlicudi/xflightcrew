//! `xfc voices`: install, preview, use and remove the Piper voices and the
//! whisper speech models, never by hand-editing `.env` or fetching files.
//!
//! ```text
//! xfc voices                    the screen
//! xfc voices list [--installed] [--filter en_GB]
//! xfc voices install <key>...   e.g. en_GB-alan-medium, or ggml-base.en
//! xfc voices install --starter  what the config names and is missing
//! xfc voices remove <key>...
//! ```
//!
//! The third outbound call after SimBrief and the METAR file, on demand only:
//! the catalogue is fetched (and cached a day in the models folder) when this
//! command runs, never at boot or from the command loop. Pure core in
//! [`catalogue`], files and network in [`download`], the screen in [`screen`].

pub mod catalogue;
pub mod download;
mod screen;

use std::path::{Path, PathBuf};

use anyhow::{anyhow, Result};

use crate::config::Config;
use catalogue::{Configured, Entry, Kind, Row, StarterItem, StarterStatus, Use};

/// What the preview says.
pub const PREVIEW_LINE: &str = "Tower, Cessna one two three, ready for departure.";

/// Said wherever a pick is made: the running copilot reads prefs at boot.
pub const RESTART_NOTE: &str = "a running xfc picks the change up on restart";

/// Everything the command works from: the folder, both catalogues, what the
/// live config uses, and a note when the catalogue is not fresh.
pub struct Store {
    pub dir: PathBuf,
    pub piper: Vec<Entry>,
    pub whisper: Vec<Entry>,
    pub uses: Vec<(String, Use)>,
    /// The starter set the live config names ([`starter`]).
    pub starter: Vec<StarterItem>,
    /// "offline: ..." when the catalogue came from the cache or not at all.
    pub note: Option<String>,
}

impl Store {
    pub fn load(config: &Config) -> Self {
        let dir = crate::config::models_dir();
        download::clean_stale_parts(&dir);
        let (piper, note) = piper_catalogue(&dir);
        Store {
            dir,
            piper,
            whisper: catalogue::whisper_catalogue(),
            uses: catalogue::uses(&configured(config)),
            starter: starter(config),
            note,
        }
    }

    /// The starter set as it stands: what is missing (file checks where the
    /// config names each), priced against the catalogue.
    pub fn starter_status(&self) -> StarterStatus {
        let missing = self
            .starter
            .iter()
            .filter(|i| !item_installed(i))
            .cloned()
            .collect();
        let all: Vec<Entry> = self.piper.iter().chain(&self.whisper).cloned().collect();
        catalogue::starter_status(missing, &all)
    }

    /// The rows the missing starter items install from (the catalogued ones).
    pub fn starter_rows(&self) -> Vec<Row> {
        self.starter_status()
            .missing
            .iter()
            .filter_map(|i| self.row(&i.key))
            .filter(|r| !r.installed && !r.entry.is_local())
            .collect()
    }

    /// The rows of one tab, joined against the folder as it is now.
    pub fn rows(&self, kind: Kind) -> Vec<Row> {
        let disk = download::list_dir(&self.dir);
        let cat = match kind {
            Kind::Piper => &self.piper,
            Kind::Whisper => &self.whisper,
        };
        catalogue::join(kind, cat, &disk, &self.uses)
    }

    /// The roles as the registry now has them, after a pick on the screen.
    pub fn refresh_uses(&mut self) {
        self.uses = catalogue::uses(&configured(&crate::settings::live()));
    }

    /// Find a key on either tab.
    pub fn row(&self, key: &str) -> Option<Row> {
        [Kind::Piper, Kind::Whisper]
            .into_iter()
            .flat_map(|k| self.rows(k))
            .find(|r| r.entry.key == key)
    }
}

/// The roles' voices as the live config has them.
fn configured(c: &Config) -> Configured {
    Configured {
        copilot: c.piper_voice.clone(),
        cabin: c.cabin_piper_voice.clone(),
        tower: c.atc_piper_voice.clone(),
        approach: c.atc_approach_piper_voice.clone(),
        departure: c.atc_departure_piper_voice.clone(),
        atis: c.atc_atis_piper_voice.clone(),
        pool: c.atc_voice_pool.clone(),
        whisper: c.whisper_model_path.clone(),
    }
}

/// The starter set the live config names: its speech model and every single
/// voice (`catalogue::starter_items`); no voices when speech is off.
pub fn starter(config: &Config) -> Vec<StarterItem> {
    catalogue::starter_items(&configured(config), config.tts_backend != "off")
}

/// Whether a starter item is on disk where the config names it: a voice's
/// `.onnx` and `.onnx.json`, the speech model's file.
pub fn item_installed(i: &StarterItem) -> bool {
    match i.kind {
        Kind::Piper => crate::voice::voice_installed(&i.spec),
        Kind::Whisper => Path::new(&i.spec).is_file(),
    }
}

/// What of the starter set is missing: file checks only, offline and cheap
/// (the boot's decision to show the home screen asks this).
pub fn starter_missing(config: &Config) -> Vec<StarterItem> {
    starter(config)
        .into_iter()
        .filter(|i| !item_installed(i))
        .collect()
}

/// The Piper catalogue: the day-old cache, else a fetch (cached atomically),
/// else the stale cache said as offline, else nothing said as offline.
fn piper_catalogue(dir: &Path) -> (Vec<Entry>, Option<String>) {
    let cache = dir.join(".catalogue").join("voices.json");
    let age = std::fs::metadata(&cache)
        .and_then(|m| m.modified())
        .ok()
        .and_then(|t| t.elapsed().ok());
    let cached = || {
        std::fs::read_to_string(&cache)
            .ok()
            .and_then(|s| catalogue::parse_piper(&s).ok())
    };
    if !catalogue::catalogue_stale(age) {
        if let Some(c) = cached() {
            return (c, None);
        }
    }
    match download::fetch_text(catalogue::PIPER_CATALOGUE_URL)
        .and_then(|s| catalogue::parse_piper(&s).map(|c| (c, s)))
    {
        Ok((c, text)) => {
            let _ = std::fs::create_dir_all(cache.parent().unwrap_or(dir));
            let part = cache.with_extension("json.part");
            if std::fs::write(&part, text).is_ok() {
                let _ = std::fs::rename(&part, &cache);
            }
            (c, None)
        }
        Err(e) => match cached() {
            Some(c) => {
                let hours = age.map(|a| a.as_secs() / 3600).unwrap_or(0);
                (
                    c,
                    Some(format!(
                        "offline ({e:#}): using the catalogue cached {hours} h ago"
                    )),
                )
            }
            None => (
                Vec::new(),
                Some(format!(
                    "offline ({e:#}) and no cached catalogue: installed voices only"
                )),
            ),
        },
    }
}

/// The spec a role is set to for a key: its model in the models folder.
pub fn voice_spec(key: &str) -> String {
    crate::config::model_path(&format!("{key}.onnx"))
}

/// The registry setting a role's voice is picked through.
pub fn role_setting(role: &str) -> Option<&'static str> {
    Some(match role {
        "copilot" => "copilot_voice",
        "cabin" => "cabin_voice",
        "tower" => "atc_tower_voice",
        "approach" => "atc_approach_voice",
        "departure" => "atc_departure_voice",
        "ATIS" => "atc_atis_voice",
        "speech recognition" => "whisper_model",
        _ => return None,
    })
}

/// Pick a voice (or the speech model) for a role through the settings
/// registry, the ONE door: pinning, checks, persistence and a live swap are
/// its. Ok says what happened and when it applies.
pub fn set_role(role: &str, spec: &str) -> std::result::Result<String, String> {
    let key = role_setting(role).ok_or_else(|| format!("no voice setting for {role}"))?;
    set_key(key, spec)
}

/// Set a voice setting by its registry key (the Cast view's rows carry it).
pub fn set_key(key: &str, spec: &str) -> std::result::Result<String, String> {
    let s = crate::settings::ALL
        .iter()
        .find(|s| s.key == key)
        .ok_or_else(|| format!("no setting {key}"))?;
    crate::settings::set(key, &crate::settings::Value::Text(spec.to_string()))?;
    let when = match s.tier {
        crate::settings::Tier::Live => "in force now",
        crate::settings::Tier::Restart => RESTART_NOTE,
    };
    Ok(format!(
        "{}: {}; {when}",
        s.label,
        crate::settings::voice_name(spec)
    ))
}

/// Add a voice to the away-field pool, or take one out, through the
/// registry. Ok says the pool's new size.
pub fn edit_pool(spec: &str, add: bool) -> std::result::Result<String, String> {
    let live = crate::settings::live();
    let mut pool: Vec<String> = live
        .atc_voice_pool
        .split(',')
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .map(str::to_string)
        .collect();
    let had = pool.iter().any(|p| p == spec);
    match (add, had) {
        (true, true) => return Err(format!("{} is in the pool", crate::settings::voice_name(spec))),
        (false, false) => {
            return Err(format!("{} is not in the pool", crate::settings::voice_name(spec)))
        }
        (true, false) => pool.push(spec.to_string()),
        (false, true) => pool.retain(|p| p != spec),
    }
    crate::settings::set(
        "atc_voice_pool",
        &crate::settings::Value::Text(pool.join(",")),
    )?;
    Ok(format!(
        "away-field pool: {} voices; {RESTART_NOTE}",
        pool.len()
    ))
}

/// `u` on a list row: use this voice or model for a role, through
/// [`set_role`]. A voice must be installed first.
pub fn use_as(row: &Row, role: &str) -> Result<String> {
    if !row.installed {
        return Err(anyhow!("install {} first", row.entry.key));
    }
    let spec = match row.entry.kind {
        Kind::Piper => voice_spec(&row.entry.key),
        Kind::Whisper => crate::config::model_path(&format!("{}.bin", row.entry.key)),
    };
    let role = match row.entry.kind {
        Kind::Whisper => "speech recognition",
        Kind::Piper if role == "speech recognition" => {
            return Err(anyhow!("{} is a voice, not a speech model", row.entry.key))
        }
        Kind::Piper => role,
    };
    if role == "pool" {
        return edit_pool(&spec, true).map_err(|e| anyhow!(e));
    }
    set_role(role, &spec).map_err(|e| anyhow!(e))
}

/// Speak the preview line in a voice (speaker 0 of a multi-speaker one),
/// through the in-process ONNX path `xfc tts` uses. Blocks until played.
#[cfg(feature = "onnx-tts")]
pub fn preview(row: &Row) -> Result<()> {
    if row.entry.kind != Kind::Piper {
        return Err(anyhow!("only voices can be previewed"));
    }
    if !row.installed {
        return Err(anyhow!("install {} to preview it", row.entry.key));
    }
    let mut v = crate::tts_onnx::OnnxTts::load(&voice_spec(&row.entry.key))?;
    let samples = v.render(PREVIEW_LINE)?;
    let path = std::env::temp_dir().join("xflightcrew-voice-preview.wav");
    crate::tts_onnx::write_wav(&path, &samples, v.sample_rate())?;
    crate::tts::play(&path)
}

#[cfg(not(feature = "onnx-tts"))]
pub fn preview(_row: &Row) -> Result<()> {
    Err(anyhow!("preview needs the onnx-tts build"))
}

/// `xfc voices ...`. Runs on its own thread: the downloads and the catalogue
/// fetch are blocking HTTP, which must never run inside the async runtime.
/// Ok(true) when every asked action succeeded.
pub fn main_cmd(args: Vec<String>, config: Config) -> Result<bool> {
    std::thread::spawn(move || run(&args, &config))
        .join()
        .map_err(|_| anyhow!("xfc voices panicked"))?
}

const USAGE: &str = "usage:
  xfc voices                    the screen
  xfc voices list [--installed] [--filter en_GB]
  xfc voices install <key>...   e.g. en_GB-alan-medium, or ggml-base.en
  xfc voices install --starter  what the config names and is missing
  xfc voices remove <key>...";

fn run(args: &[String], config: &Config) -> Result<bool> {
    // The registry as boot has it, so a pick here is judged and shown
    // against the same values (the screen runs before the crew's install).
    crate::settings::install(config.clone());
    let store = Store::load(config);
    match args.first().map(String::as_str) {
        None => {
            screen::run(store)?;
            Ok(true)
        }
        Some("list") => {
            let mut installed = false;
            let mut query = String::new();
            let mut it = args[1..].iter();
            while let Some(a) = it.next() {
                match a.as_str() {
                    "--installed" => installed = true,
                    "--filter" => {
                        query = it
                            .next()
                            .ok_or_else(|| anyhow!("--filter needs a term"))?
                            .clone()
                    }
                    other => return Err(anyhow!("unknown option {other}\n{USAGE}")),
                }
            }
            print_list(&store, &query, installed);
            Ok(true)
        }
        Some("install") if args.get(1).map(String::as_str) == Some("--starter") => {
            Ok(install_starter_cli(&store))
        }
        Some("install") if args.len() > 1 => Ok(args[1..]
            .iter()
            .map(|k| install_cli(&store, k))
            .fold(true, |ok, one| ok & one)),
        Some("remove") if args.len() > 1 => Ok(args[1..]
            .iter()
            .map(|k| remove_cli(&store, k))
            .fold(true, |ok, one| ok & one)),
        Some("help" | "--help" | "-h") => {
            println!("{USAGE}");
            Ok(true)
        }
        Some(_) => Err(anyhow!("{USAGE}")),
    }
}

/// The voices screen on the calling thread (the home screen's Voices row):
/// returns when it is quit. Blocking HTTP: never call it inside the runtime.
pub fn open_screen(config: &Config) -> Result<()> {
    screen::run(Store::load(config))?;
    Ok(())
}

/// One list line, shared by the CLI and the screen.
pub fn row_columns(r: &Row) -> [String; 5] {
    let e = &r.entry;
    let place = match e.kind {
        Kind::Piper => e.country.clone(),
        Kind::Whisper => e.lang.clone(),
    };
    let speakers = match (e.kind, e.speakers) {
        (Kind::Piper, n) if n > 0 => format!("{n} spk"),
        _ => String::new(),
    };
    let size = if r.installed {
        catalogue::human_size(r.disk_bytes)
    } else {
        catalogue::human_size(e.download_size())
    };
    [e.key.clone(), place, e.quality.clone(), speakers, size]
}

fn print_list(store: &Store, query: &str, installed_only: bool) {
    if let Some(note) = &store.note {
        println!("{note}");
    }
    for (kind, title) in [
        (Kind::Piper, "Voices"),
        (Kind::Whisper, "Speech recognition"),
    ] {
        let rows = store.rows(kind);
        let shown = catalogue::filter(&rows, query, installed_only);
        let inst: Vec<&Row> = rows.iter().filter(|r| r.installed).collect();
        let bytes: u64 = inst.iter().map(|r| r.disk_bytes).sum();
        println!(
            "{title}: {} shown, {} installed ({}) in {}",
            shown.len(),
            inst.len(),
            catalogue::human_size(bytes),
            store.dir.display()
        );
        for r in shown {
            let [key, place, quality, spk, size] = row_columns(r);
            let mark = if r.installed { "✓" } else { " " };
            println!(
                " {mark} {key:<42} {place:<16} {quality:<8} {spk:>7} {size:>8}   {}",
                r.tag()
            );
        }
        println!();
    }
}

fn install_cli(store: &Store, key: &str) -> bool {
    let Some(row) = store.row(key) else {
        eprintln!("{key}: not in the catalogue (try `xfc voices list --filter {key}`)");
        return false;
    };
    if row.installed {
        eprintln!("{key}: already installed");
        return true;
    }
    if row.entry.is_local() {
        eprintln!("{key}: a local model, nothing to download");
        return false;
    }
    let mut last = u8::MAX;
    let result = download::install(&row.entry, &store.dir, &mut |p| {
        let pct = p.percent();
        if pct != last {
            last = pct;
            eprint!(
                "\r{key}: file {}/{} {pct:>3}% of {}",
                p.file + 1,
                p.files,
                catalogue::human_size(p.total)
            );
        }
    });
    eprintln!();
    match result {
        Ok(()) => {
            let digest = row
                .entry
                .files
                .first()
                .map(|f| f.digest.name())
                .unwrap_or("");
            println!(
                "{key}: installed in {}, {} file(s) size and {digest} verified",
                store.dir.display(),
                row.entry.files.len()
            );
            true
        }
        Err(e) => {
            eprintln!("{key}: install failed, nothing installed: {e:#}");
            false
        }
    }
}

/// `xfc voices install --starter`: the total first, then each missing item
/// through the same atomic, verified path as a single install.
fn install_starter_cli(store: &Store) -> bool {
    let s = store.starter_status();
    if s.complete() {
        println!(
            "starter set: complete ({} items) in {}",
            store.starter.len(),
            store.dir.display()
        );
        return true;
    }
    let rows = store.starter_rows();
    println!(
        "starter set: {} missing, {} to download into {}",
        s.missing.len(),
        catalogue::human_size(s.bytes),
        store.dir.display()
    );
    for r in &rows {
        println!(
            "  {:<42} {:>8}",
            r.entry.key,
            catalogue::human_size(r.entry.download_size())
        );
    }
    let mut ok = true;
    for key in &s.unknown {
        eprintln!("{key}: not in the catalogue, nothing to download (is the catalogue offline?)");
        ok = false;
    }
    for r in &rows {
        ok &= install_cli(store, &r.entry.key);
    }
    ok
}

fn remove_cli(store: &Store, key: &str) -> bool {
    let Some(row) = store.row(key) else {
        eprintln!("{key}: not installed");
        return false;
    };
    if let Err(e) = catalogue::remove_gate(&row) {
        eprintln!("{e}");
        return false;
    }
    match download::remove(row.entry.kind, key, &store.dir) {
        Ok(()) => {
            println!("{key}: removed from {}", store.dir.display());
            true
        }
        Err(e) => {
            eprintln!("{key}: {e:#}");
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn row(kind: Kind, installed: bool) -> Row {
        Row {
            entry: Entry {
                key: "en_GB-alan-medium".into(),
                kind,
                country: String::new(),
                lang: "en_GB".into(),
                quality: "medium".into(),
                speakers: 1,
                files: Vec::new(),
            },
            installed,
            disk_bytes: 0,
            uses: Vec::new(),
        }
    }

    /// Only the refusals (a pick that takes writes prefs.json): a voice not
    /// installed, a voice offered as the speech model, a role no setting
    /// picks, and a role its variable pins, refused by the registry itself.
    #[test]
    fn use_as_goes_through_the_registry_and_says_its_refusals() {
        let err = use_as(&row(Kind::Piper, false), "copilot").unwrap_err();
        assert!(err.to_string().contains("install"), "{err}");
        let err = use_as(&row(Kind::Piper, true), "speech recognition").unwrap_err();
        assert!(err.to_string().contains("not a speech model"), "{err}");
        let err = use_as(&row(Kind::Piper, true), "pilot").unwrap_err();
        assert!(err.to_string().contains("no voice setting"), "{err}");
        let _guard = crate::settings::TEST_LOCK
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        // SAFETY: the lock above serialises every env-mutating test.
        unsafe { std::env::set_var("XCOPILOT_ATC_PIPER_VOICE", "x.onnx") };
        let err = use_as(&row(Kind::Piper, true), "tower").unwrap_err();
        unsafe { std::env::remove_var("XCOPILOT_ATC_PIPER_VOICE") };
        assert!(
            err.to_string().contains("pinned by XCOPILOT_ATC_PIPER_VOICE"),
            "{err}"
        );
        assert_eq!(role_setting("ATIS"), Some("atc_atis_voice"));
    }
}
