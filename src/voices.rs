//! `xfc voices`: install, preview, use and remove the Piper voices and the
//! whisper speech models, never by hand-editing `.env` or fetching files.
//!
//! ```text
//! xfc voices                    the screen
//! xfc voices list [--installed] [--filter en_GB]
//! xfc voices install <key>...   e.g. en_GB-alan-medium, or ggml-base.en
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
use catalogue::{Configured, Entry, Kind, Row, Use};

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
            note,
        }
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

/// Set the copilot's or the cabin's voice through the SAME prefs the panel's
/// voice picker writes. ATC roles stay env-configured.
pub fn use_as(row: &Row, role: &str) -> Result<String> {
    if row.entry.kind != Kind::Piper {
        return Err(anyhow!(
            "a speech model is set with WHISPER_MODEL={} in .env",
            crate::config::model_path(&format!("{}.bin", row.entry.key))
        ));
    }
    if !row.installed {
        return Err(anyhow!("install {} first", row.entry.key));
    }
    let spec = voice_spec(&row.entry.key);
    match role {
        "copilot" => crate::prefs::save_pilot_voice(&spec),
        "cabin" => crate::prefs::save_cabin_voice(&spec),
        other => return Err(anyhow!("{other} is set in .env, not here")),
    }
    Ok(format!(
        "{role} voice set to {}; {RESTART_NOTE}",
        row.entry.key
    ))
}

/// How an ATC role is pointed at a voice: the variable to set.
pub fn atc_hint(key: &str) -> String {
    format!(
        "ATC voices are set in .env: XCOPILOT_ATC_PIPER_VOICE={} for the tower; \
         _APPROACH_, _DEPARTURE_, _ATIS_PIPER_VOICE and XCOPILOT_ATC_VOICE_POOL alike",
        voice_spec(key)
    )
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
  xfc voices remove <key>...";

fn run(args: &[String], config: &Config) -> Result<bool> {
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

    /// Only the refusals: a successful pick writes the panel's prefs.json.
    #[test]
    fn use_as_takes_an_installed_voice_for_the_copilot_or_cabin_only() {
        let err = use_as(&row(Kind::Piper, false), "copilot").unwrap_err();
        assert!(err.to_string().contains("install"), "{err}");
        let err = use_as(&row(Kind::Whisper, true), "copilot").unwrap_err();
        assert!(err.to_string().contains("WHISPER_MODEL"), "{err}");
        let err = use_as(&row(Kind::Piper, true), "tower").unwrap_err();
        assert!(err.to_string().contains(".env"), "{err}");
    }
}
