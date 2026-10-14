//! The pure core of `xfc voices`: what can be installed (the Piper catalogue
//! and our whisper table), what is on disk, who uses what, the filter and the
//! remove gate. No network and no filesystem here: the shell lists the
//! folder and fetches the catalogue, these functions decide.

use std::collections::BTreeMap;

use anyhow::{anyhow, Context, Result};

/// Where a Piper voice's files are fetched from (`<base>/<path>`).
pub const PIPER_BASE: &str = "https://huggingface.co/rhasspy/piper-voices/resolve/main";
/// The Piper catalogue itself.
pub const PIPER_CATALOGUE_URL: &str =
    "https://huggingface.co/rhasspy/piper-voices/resolve/main/voices.json";
/// Where a whisper ggml model is fetched from (`<base>/<file>`).
pub const WHISPER_BASE: &str = "https://huggingface.co/ggerganov/whisper.cpp/resolve/main";

/// The two tabs: text to speech and speech recognition.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Piper,
    Whisper,
}

/// How a downloaded file is checked before it is renamed in.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Digest {
    Md5(String),
    Sha256(String),
}

impl Digest {
    pub fn name(&self) -> &'static str {
        match self {
            Digest::Md5(_) => "md5",
            Digest::Sha256(_) => "sha256",
        }
    }
    pub fn hex(&self) -> &str {
        match self {
            Digest::Md5(h) | Digest::Sha256(h) => h,
        }
    }
}

/// One file of an installable entry.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RemoteFile {
    /// The full download URL.
    pub url: String,
    /// The name it lands under, flat in the models folder.
    pub local_name: String,
    pub size: u64,
    pub digest: Digest,
}

/// One voice or speech model, from the catalogue or found on disk.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Entry {
    pub key: String,
    pub kind: Kind,
    /// "Great Britain"; empty for whisper and local entries.
    pub country: String,
    /// "en_GB"; the language tag a filter can match.
    pub lang: String,
    /// "medium", "high"; for whisper the model's size class.
    pub quality: String,
    pub speakers: u32,
    /// Nothing to download for a local entry (not in any catalogue).
    pub files: Vec<RemoteFile>,
}

impl Entry {
    pub fn download_size(&self) -> u64 {
        self.files.iter().map(|f| f.size).sum()
    }
    pub fn is_local(&self) -> bool {
        self.files.is_empty()
    }
}

/// The names an entry occupies in the models folder: what makes it installed
/// and what a remove deletes.
pub fn local_names(kind: Kind, key: &str) -> Vec<String> {
    match kind {
        Kind::Piper => vec![format!("{key}.onnx"), format!("{key}.onnx.json")],
        Kind::Whisper => vec![format!("{key}.bin")],
    }
}

#[derive(serde::Deserialize)]
struct PiperLanguage {
    code: String,
    #[serde(default)]
    country_english: String,
}

#[derive(serde::Deserialize)]
struct PiperFile {
    size_bytes: u64,
    md5_digest: String,
}

#[derive(serde::Deserialize)]
struct PiperVoice {
    language: PiperLanguage,
    quality: String,
    num_speakers: u32,
    files: BTreeMap<String, PiperFile>,
}

/// Parse Piper's `voices.json`. Only the `.onnx` and its `.onnx.json` are
/// kept (the MODEL_CARD is not installed), and both land flat as
/// `<key>.onnx` / `<key>.onnx.json`. A voice missing either is skipped:
/// installing half of one is never offered.
pub fn parse_piper(json: &str) -> Result<Vec<Entry>> {
    let raw: BTreeMap<String, PiperVoice> =
        serde_json::from_str(json).context("parsing the Piper catalogue")?;
    let mut out = Vec::with_capacity(raw.len());
    for (key, v) in raw {
        let pick = |suffix: &str| {
            v.files
                .iter()
                .find(|(path, _)| path.ends_with(suffix))
                .map(|(path, f)| RemoteFile {
                    url: format!("{PIPER_BASE}/{path}"),
                    local_name: format!("{key}{suffix}"),
                    size: f.size_bytes,
                    digest: Digest::Md5(f.md5_digest.to_ascii_lowercase()),
                })
        };
        let (Some(model), Some(config)) = (pick(".onnx"), pick(".onnx.json")) else {
            continue;
        };
        out.push(Entry {
            key,
            kind: Kind::Piper,
            country: v.language.country_english,
            lang: v.language.code,
            quality: v.quality,
            speakers: v.num_speakers,
            files: vec![model, config],
        });
    }
    Ok(out)
}

/// Our whisper table: there is no upstream manifest. Sizes and sha256 are
/// the Hugging Face tree's own (`/api/models/ggerganov/whisper.cpp/tree/main`,
/// `lfs.oid`, read 2026-10-13), never invented.
const WHISPER_TABLE: &[(&str, &str, u64, &str)] = &[
    (
        "ggml-base.en",
        "base",
        147_964_211,
        "a03779c86df3323075f5e796cb2ce5029f00ec8869eee3fdfb897afe36c6d002",
    ),
    (
        "ggml-small.en",
        "small",
        487_614_201,
        "c6138d6d58ecc8322097e0f987c32f1be8bb0a18532a3f88f734d1bbf9c41e5d",
    ),
    (
        "ggml-medium.en",
        "medium",
        1_533_774_781,
        "cc37e93478338ec7700281a7ac30a10128929eb8f427dda2e865faa8f6da4356",
    ),
    (
        "ggml-large-v3-turbo",
        "large",
        1_624_555_275,
        "1fc70f774d38eb169993ac391eea357ef47c88757ef72ee5943879b7e8e2bc69",
    ),
];

/// The whisper models we offer, from [`WHISPER_TABLE`].
pub fn whisper_catalogue() -> Vec<Entry> {
    WHISPER_TABLE
        .iter()
        .map(|(key, class, size, sha)| Entry {
            key: (*key).to_string(),
            kind: Kind::Whisper,
            country: String::new(),
            lang: if key.ends_with(".en") {
                "en"
            } else {
                "multilingual"
            }
            .to_string(),
            quality: (*class).to_string(),
            speakers: 0,
            files: vec![RemoteFile {
                url: format!("{WHISPER_BASE}/{key}.bin"),
                local_name: format!("{key}.bin"),
                size: *size,
                digest: Digest::Sha256((*sha).to_string()),
            }],
        })
        .collect()
}

/// The entry key a configured path names: the file's own name less its
/// extension, any `#n` speaker suffix stripped. `models/en_GB-vctk-medium.onnx#3`
/// is `en_GB-vctk-medium`, `models/ggml-base.en.bin` is `ggml-base.en`. Empty
/// specs (an ATIS voice inheriting the tower's) name nothing.
pub fn key_of_spec(spec: &str) -> Option<String> {
    let spec = spec.trim();
    let path = match spec.rsplit_once('#') {
        Some((p, n)) if n.parse::<u32>().is_ok() => p,
        _ => spec,
    };
    let name = path.rsplit(['/', '\\']).next().unwrap_or(path);
    let key = name
        .strip_suffix(".onnx")
        .or_else(|| name.strip_suffix(".bin"))
        .unwrap_or(name);
    (!key.is_empty()).then(|| key.to_string())
}

/// One role that uses a model, as the live config says.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Use {
    pub role: &'static str,
    /// The variable that sets it, said when a remove is refused.
    pub env: &'static str,
    /// Whether `u` (use as) can set it: copilot and cabin only, through prefs.
    pub via_prefs: bool,
}

/// The configured voices and model, copied out of the live config so the
/// join stays pure and testable.
#[derive(Clone, Debug, Default)]
pub struct Configured {
    pub copilot: String,
    pub cabin: String,
    pub tower: String,
    pub approach: String,
    pub departure: String,
    pub atis: String,
    /// Comma-separated, entries may carry `#n`.
    pub pool: String,
    pub whisper: String,
}

/// Every (key, use) pair the configuration holds, deduplicated per role.
pub fn uses(c: &Configured) -> Vec<(String, Use)> {
    let one = |role, env, via_prefs| Use {
        role,
        env,
        via_prefs,
    };
    let mut out: Vec<(String, Use)> = Vec::new();
    let mut push = |spec: &str, u: Use| {
        if let Some(k) = key_of_spec(spec) {
            if !out.iter().any(|(ek, eu)| *ek == k && eu.role == u.role) {
                out.push((k, u));
            }
        }
    };
    push(&c.copilot, one("copilot", "PIPER_VOICE", true));
    push(&c.cabin, one("cabin", "XCOPILOT_CABIN_VOICE", true));
    push(&c.tower, one("tower", "XCOPILOT_ATC_PIPER_VOICE", false));
    push(
        &c.approach,
        one("approach", "XCOPILOT_ATC_APPROACH_PIPER_VOICE", false),
    );
    push(
        &c.departure,
        one("departure", "XCOPILOT_ATC_DEPARTURE_PIPER_VOICE", false),
    );
    push(&c.atis, one("ATIS", "XCOPILOT_ATC_ATIS_PIPER_VOICE", false));
    for spec in c.pool.split(',') {
        push(spec, one("ATC pool", "XCOPILOT_ATC_VOICE_POOL", false));
    }
    push(
        &c.whisper,
        one("speech recognition", "WHISPER_MODEL", false),
    );
    out
}

/// One member of the starter set: the spec the config names and the entry
/// key that installs it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StarterItem {
    pub kind: Kind,
    pub key: String,
    /// The path the config names, what the crew will load.
    pub spec: String,
}

/// The starter set: the speech model the config names plus every SINGLE
/// voice it names (copilot, cabin, tower, approach, departure, ATIS when
/// set), each once. Derived from the config, never a list of its own, so a
/// changed default changes the starter. The away-field pool is not in it:
/// those voices are optional and the caster draws only the installed ones.
/// `voices` false (the copilot's speech off) leaves the voices out.
pub fn starter_items(c: &Configured, voices: bool) -> Vec<StarterItem> {
    let mut out: Vec<StarterItem> = Vec::new();
    let mut push = |kind: Kind, spec: &str| {
        if let Some(key) = key_of_spec(spec) {
            if !out.iter().any(|i| i.kind == kind && i.key == key) {
                out.push(StarterItem {
                    kind,
                    key,
                    spec: spec.trim().to_string(),
                });
            }
        }
    };
    push(Kind::Whisper, &c.whisper);
    if voices {
        for spec in [
            &c.copilot,
            &c.cabin,
            &c.tower,
            &c.approach,
            &c.departure,
            &c.atis,
        ] {
            push(Kind::Piper, spec);
        }
    }
    out
}

/// The starter's state against the catalogue: what is missing, its download
/// size, and what no catalogue offers (a local model that went missing, or
/// no catalogue to hand).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct StarterStatus {
    pub missing: Vec<StarterItem>,
    pub bytes: u64,
    pub unknown: Vec<String>,
}

impl StarterStatus {
    pub fn complete(&self) -> bool {
        self.missing.is_empty()
    }
}

/// Price the missing items against the catalogue entries.
pub fn starter_status(missing: Vec<StarterItem>, catalogue: &[Entry]) -> StarterStatus {
    let mut bytes = 0;
    let mut unknown = Vec::new();
    for item in &missing {
        match catalogue
            .iter()
            .find(|e| e.kind == item.kind && e.key == item.key && !e.is_local())
        {
            Some(e) => bytes += e.download_size(),
            None => unknown.push(item.key.clone()),
        }
    }
    StarterStatus {
        missing,
        bytes,
        unknown,
    }
}

/// The starter's one line: "starter set: complete", or how many are missing,
/// their size and the key that gets them.
pub fn starter_line(s: &StarterStatus, key_hint: &str) -> String {
    if s.complete() {
        return "starter set: complete".into();
    }
    let n = s.missing.len();
    let size = if s.unknown.len() == n {
        String::new()
    } else if s.unknown.is_empty() {
        format!(", {}", human_size(s.bytes))
    } else {
        format!(
            ", {} plus {} not in the catalogue",
            human_size(s.bytes),
            s.unknown.len()
        )
    };
    let hint = if key_hint.is_empty() {
        String::new()
    } else {
        format!(", {key_hint}")
    };
    format!("starter set: {n} missing{size}{hint}")
}

/// One line of the list: an entry, whether it is installed, who uses it.
#[derive(Clone, Debug)]
pub struct Row {
    pub entry: Entry,
    pub installed: bool,
    /// Bytes on disk when installed (the shell's listing), else 0.
    pub disk_bytes: u64,
    pub uses: Vec<Use>,
}

impl Row {
    /// The tag column: who uses it ("copilot, tower"), or "local".
    pub fn tag(&self) -> String {
        let mut roles: Vec<&str> = self.uses.iter().map(|u| u.role).collect();
        roles.dedup();
        let mut tag = roles.join(", ");
        if self.entry.is_local() {
            tag = if tag.is_empty() {
                "local".into()
            } else {
                format!("local, {tag}")
            };
        }
        tag
    }
}

/// Join a catalogue with the folder's listing (`name -> bytes`) and the
/// configured uses. Installed = every local name present. Models on disk that
/// no catalogue knows are listed too, as local entries: a Piper voice by its
/// `.onnx` with the `.onnx.json` beside it, a whisper model by its
/// `ggml-*.bin`. Sorted by key.
pub fn join(
    kind: Kind,
    catalogue: &[Entry],
    disk: &BTreeMap<String, u64>,
    uses: &[(String, Use)],
) -> Vec<Row> {
    let uses_of = |key: &str| -> Vec<Use> {
        uses.iter()
            .filter(|(k, _)| k == key)
            .map(|(_, u)| u.clone())
            .collect()
    };
    let installed_bytes = |key: &str| -> Option<u64> {
        local_names(kind, key)
            .iter()
            .map(|n| disk.get(n).copied())
            .sum::<Option<u64>>()
    };
    let mut rows: Vec<Row> = catalogue
        .iter()
        .filter(|e| e.kind == kind)
        .map(|e| {
            let bytes = installed_bytes(&e.key);
            Row {
                entry: e.clone(),
                installed: bytes.is_some(),
                disk_bytes: bytes.unwrap_or(0),
                uses: uses_of(&e.key),
            }
        })
        .collect();
    for name in disk.keys() {
        let key = match kind {
            Kind::Piper => name.strip_suffix(".onnx"),
            Kind::Whisper => name.strip_suffix(".bin").filter(|k| k.starts_with("ggml-")),
        };
        let Some(key) = key else { continue };
        if rows.iter().any(|r| r.entry.key == key) {
            continue;
        }
        let Some(bytes) = installed_bytes(key) else {
            continue;
        };
        rows.push(Row {
            entry: Entry {
                key: key.to_string(),
                kind,
                country: String::new(),
                lang: key.split('-').next().unwrap_or("").to_string(),
                quality: String::new(),
                speakers: 0,
                files: Vec::new(),
            },
            installed: true,
            disk_bytes: bytes,
            uses: uses_of(key),
        });
    }
    rows.sort_by(|a, b| a.entry.key.cmp(&b.entry.key));
    rows
}

/// Rows matching a filter: every whitespace-separated term must appear,
/// case-insensitively, in the key, the language tag, the country, the
/// quality or the tag column. `--installed` keeps installed rows only.
pub fn filter<'a>(rows: &'a [Row], query: &str, installed_only: bool) -> Vec<&'a Row> {
    let terms: Vec<String> = query.split_whitespace().map(|t| t.to_lowercase()).collect();
    rows.iter()
        .filter(|r| !installed_only || r.installed)
        .filter(|r| {
            let hay = format!(
                "{} {} {} {} {}",
                r.entry.key,
                r.entry.lang,
                r.entry.country,
                r.entry.quality,
                r.tag()
            )
            .to_lowercase();
            terms.iter().all(|t| hay.contains(t.as_str()))
        })
        .collect()
}

/// May this model be removed? Refused while anything uses it, naming who
/// and how to move them off it.
pub fn remove_gate(row: &Row) -> Result<()> {
    if !row.installed {
        return Err(anyhow!("{} is not installed", row.entry.key));
    }
    if row.uses.is_empty() {
        return Ok(());
    }
    let who: Vec<String> = row
        .uses
        .iter()
        .map(|u| {
            if u.via_prefs {
                format!("{} (pick another with u)", u.role)
            } else {
                format!("{} (set by {})", u.role, u.env)
            }
        })
        .collect();
    Err(anyhow!(
        "{} is in use: {}. Point it at another model first",
        row.entry.key,
        who.join(", ")
    ))
}

/// Leftover partial downloads in a folder listing: removed on the next run.
pub fn stale_parts(disk: &BTreeMap<String, u64>) -> Vec<String> {
    disk.keys()
        .filter(|n| n.ends_with(".part"))
        .cloned()
        .collect()
}

/// Should the cached catalogue be refetched? A missing or day-old cache is.
pub fn catalogue_stale(age: Option<std::time::Duration>) -> bool {
    age.is_none_or(|a| a >= std::time::Duration::from_secs(24 * 3600))
}

/// Bytes as the list shows them: decimal, "63 MB", "1.6 GB".
pub fn human_size(bytes: u64) -> String {
    const MB: f64 = 1_000_000.0;
    let b = bytes as f64;
    if b >= 1000.0 * MB {
        format!("{:.1} GB", b / (1000.0 * MB))
    } else if b >= MB {
        format!("{:.0} MB", b / MB)
    } else if bytes > 0 {
        format!("{:.0} kB", (b / 1000.0).max(1.0))
    } else {
        "0 MB".into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!("../../bench/voices_catalogue.json");

    fn disk(names: &[(&str, u64)]) -> BTreeMap<String, u64> {
        names.iter().map(|(n, b)| ((*n).to_string(), *b)).collect()
    }

    fn defaults() -> Configured {
        Configured {
            copilot: "models/en_US-amy-medium.onnx".into(),
            cabin: "models/en_US-hfc_female-medium.onnx".into(),
            tower: "models/en_GB-alba-medium.onnx".into(),
            approach: "models/en_US-ryan-high.onnx".into(),
            departure: "models/en_GB-northern_english_male-medium.onnx".into(),
            atis: String::new(),
            pool: "models/en_GB-vctk-medium.onnx#0,models/en_GB-vctk-medium.onnx#14".into(),
            whisper: "models/ggml-base.en.bin".into(),
        }
    }

    #[test]
    fn the_catalogue_keeps_the_model_and_its_config_flat_and_skips_the_card() {
        let cat = parse_piper(FIXTURE).unwrap();
        assert_eq!(cat.len(), 5);
        let alan = cat.iter().find(|e| e.key == "en_GB-alan-medium").unwrap();
        assert_eq!(alan.country, "Great Britain");
        assert_eq!(alan.lang, "en_GB");
        assert_eq!(alan.quality, "medium");
        assert_eq!(alan.speakers, 1);
        let names: Vec<&str> = alan.files.iter().map(|f| f.local_name.as_str()).collect();
        assert_eq!(
            names,
            ["en_GB-alan-medium.onnx", "en_GB-alan-medium.onnx.json"]
        );
        assert_eq!(
            alan.files[0].url,
            format!("{PIPER_BASE}/en/en_GB/alan/medium/en_GB-alan-medium.onnx")
        );
        assert_eq!(alan.files[0].size, 63_201_294);
        assert_eq!(
            alan.files[0].digest,
            Digest::Md5("8f6b35eeb8ef6269021c6cb6d2414c9b".into())
        );
        assert!(cat
            .iter()
            .all(|e| e.files.iter().all(|f| !f.url.ends_with("MODEL_CARD"))));
    }

    #[test]
    fn the_starter_is_the_configs_single_voices_and_speech_model() {
        let items = starter_items(&defaults(), true);
        let keys: Vec<(Kind, &str)> = items.iter().map(|i| (i.kind, i.key.as_str())).collect();
        assert_eq!(
            keys,
            [
                (Kind::Whisper, "ggml-base.en"),
                (Kind::Piper, "en_US-amy-medium"),
                (Kind::Piper, "en_US-hfc_female-medium"),
                (Kind::Piper, "en_GB-alba-medium"),
                (Kind::Piper, "en_US-ryan-high"),
                (Kind::Piper, "en_GB-northern_english_male-medium"),
            ]
        );
        // The pool is never in it; a shared voice counts once; the ATIS
        // when set; speech off leaves only the speech model.
        let c = Configured {
            atis: "models/en_GB-alan-medium.onnx".into(),
            cabin: "models/en_US-amy-medium.onnx".into(),
            ..defaults()
        };
        let keys: Vec<String> = starter_items(&c, true).into_iter().map(|i| i.key).collect();
        assert!(!keys.iter().any(|k| k.contains("vctk")), "{keys:?}");
        assert_eq!(keys.iter().filter(|k| *k == "en_US-amy-medium").count(), 1);
        assert!(keys.contains(&"en_GB-alan-medium".to_string()));
        assert_eq!(starter_items(&c, false).len(), 1);
    }

    #[test]
    fn the_starter_line_prices_what_is_missing() {
        let cat = parse_piper(FIXTURE).unwrap();
        let alan = cat.iter().find(|e| e.key == "en_GB-alan-medium").unwrap();
        let item = |kind, key: &str| StarterItem {
            kind,
            key: key.into(),
            spec: String::new(),
        };
        let mut cat = cat.clone();
        cat.extend(whisper_catalogue());
        let s = starter_status(Vec::new(), &cat);
        assert_eq!(starter_line(&s, "press s"), "starter set: complete");
        let s = starter_status(
            vec![
                item(Kind::Piper, "en_GB-alan-medium"),
                item(Kind::Whisper, "ggml-base.en"),
            ],
            &cat,
        );
        let base = whisper_catalogue()
            .into_iter()
            .find(|e| e.key == "ggml-base.en")
            .unwrap()
            .download_size();
        assert_eq!(s.bytes, alan.download_size() + base);
        assert_eq!(
            starter_line(&s, "press s"),
            format!("starter set: 2 missing, {}, press s", human_size(s.bytes))
        );
        // Nothing to price it against: still said, without a size.
        let s = starter_status(vec![item(Kind::Piper, "en_GB-alan-medium")], &[]);
        assert_eq!(starter_line(&s, ""), "starter set: 1 missing");
    }

    #[test]
    fn a_multi_speaker_voice_says_how_many() {
        let cat = parse_piper(FIXTURE).unwrap();
        let vctk = cat.iter().find(|e| e.key == "en_GB-vctk-medium").unwrap();
        assert_eq!(vctk.speakers, 109);
    }

    #[test]
    fn the_whisper_table_is_real_checksums_and_flat_names() {
        let table = whisper_catalogue();
        assert!(table.iter().any(|e| e.key == "ggml-base.en"));
        assert!(table.iter().any(|e| e.key == "ggml-small.en"));
        assert!(table.iter().any(|e| e.key == "ggml-large-v3-turbo"));
        for e in &table {
            assert_eq!(e.files.len(), 1, "{}", e.key);
            let f = &e.files[0];
            assert_eq!(f.local_name, format!("{}.bin", e.key));
            assert_eq!(f.url, format!("{WHISPER_BASE}/{}.bin", e.key));
            assert!(f.size > 100_000_000, "{}", e.key);
            let Digest::Sha256(h) = &f.digest else {
                panic!("{} must be sha256", e.key)
            };
            assert_eq!(h.len(), 64, "{}", e.key);
            assert!(
                h.chars()
                    .all(|c| c.is_ascii_hexdigit() && !c.is_ascii_uppercase()),
                "{}",
                e.key
            );
        }
    }

    #[test]
    fn a_spec_names_its_key_without_folder_extension_or_speaker() {
        assert_eq!(
            key_of_spec("models/en_GB-vctk-medium.onnx#3").as_deref(),
            Some("en_GB-vctk-medium")
        );
        assert_eq!(
            key_of_spec("/abs/ggml-base.en.bin").as_deref(),
            Some("ggml-base.en")
        );
        assert_eq!(
            key_of_spec(" en_US-amy-medium.onnx ").as_deref(),
            Some("en_US-amy-medium")
        );
        assert_eq!(key_of_spec(""), None);
    }

    #[test]
    fn installed_needs_both_files_and_unknown_voices_are_local() {
        let cat = parse_piper(FIXTURE).unwrap();
        let d = disk(&[
            ("en_GB-alan-medium.onnx", 63_201_294),
            ("en_GB-alan-medium.onnx.json", 4888),
            ("en_GB-alba-medium.onnx", 63_000_000),
            ("my_custom-voice.onnx", 10),
            ("my_custom-voice.onnx.json", 1),
            ("orphan-model.onnx", 10),
            ("ggml-base.en.bin", 5),
        ]);
        let rows = join(Kind::Piper, &cat, &d, &uses(&defaults()));
        let get = |k: &str| rows.iter().find(|r| r.entry.key == k);
        assert!(get("en_GB-alan-medium").unwrap().installed);
        assert_eq!(get("en_GB-alan-medium").unwrap().disk_bytes, 63_206_182);
        assert!(
            !get("en_GB-alba-medium").unwrap().installed,
            "no .onnx.json"
        );
        let custom = get("my_custom-voice").unwrap();
        assert!(custom.installed && custom.entry.is_local());
        assert_eq!(custom.tag(), "local");
        assert!(get("orphan-model").is_none(), "half a voice is not a voice");
        assert!(get("ggml-base.en").is_none(), "whisper is the other tab");
    }

    #[test]
    fn in_use_comes_from_every_configured_role_pool_included() {
        let cat = parse_piper(FIXTURE).unwrap();
        let d = disk(&[
            ("en_GB-alba-medium.onnx", 1),
            ("en_GB-alba-medium.onnx.json", 1),
            ("en_GB-vctk-medium.onnx", 1),
            ("en_GB-vctk-medium.onnx.json", 1),
            ("en_US-amy-medium.onnx", 1),
            ("en_US-amy-medium.onnx.json", 1),
        ]);
        let rows = join(Kind::Piper, &cat, &d, &uses(&defaults()));
        let tag = |k: &str| rows.iter().find(|r| r.entry.key == k).unwrap().tag();
        assert_eq!(tag("en_GB-alba-medium"), "tower");
        assert_eq!(tag("en_US-amy-medium"), "copilot");
        assert_eq!(
            tag("en_GB-vctk-medium"),
            "ATC pool",
            "one tag for many speakers"
        );
        assert_eq!(tag("en_GB-alan-medium"), "");

        let w = join(
            Kind::Whisper,
            &whisper_catalogue(),
            &disk(&[("ggml-base.en.bin", 1), ("ggml-tiny.bin", 1)]),
            &uses(&defaults()),
        );
        let base = w.iter().find(|r| r.entry.key == "ggml-base.en").unwrap();
        assert!(base.installed);
        assert_eq!(base.tag(), "speech recognition");
        assert!(w
            .iter()
            .any(|r| r.entry.key == "ggml-tiny" && r.entry.is_local()));
    }

    #[test]
    fn remove_refuses_a_voice_in_use_and_names_who() {
        let cat = parse_piper(FIXTURE).unwrap();
        let d = disk(&[
            ("en_GB-alba-medium.onnx", 1),
            ("en_GB-alba-medium.onnx.json", 1),
            ("en_US-amy-medium.onnx", 1),
            ("en_US-amy-medium.onnx.json", 1),
            ("en_GB-alan-medium.onnx", 1),
            ("en_GB-alan-medium.onnx.json", 1),
        ]);
        let rows = join(Kind::Piper, &cat, &d, &uses(&defaults()));
        let get = |k: &str| rows.iter().find(|r| r.entry.key == k).unwrap();
        let err = remove_gate(get("en_GB-alba-medium"))
            .unwrap_err()
            .to_string();
        assert!(
            err.contains("tower") && err.contains("XCOPILOT_ATC_PIPER_VOICE"),
            "{err}"
        );
        let err = remove_gate(get("en_US-amy-medium"))
            .unwrap_err()
            .to_string();
        assert!(err.contains("copilot"), "{err}");
        assert!(remove_gate(get("en_GB-alan-medium")).is_ok());
        let err = remove_gate(get("en_GB-vctk-medium"))
            .unwrap_err()
            .to_string();
        assert!(err.contains("not installed"), "{err}");
    }

    #[test]
    fn the_filter_matches_every_term_anywhere_and_can_keep_installed_only() {
        let cat = parse_piper(FIXTURE).unwrap();
        let d = disk(&[
            ("en_GB-alan-medium.onnx", 1),
            ("en_GB-alan-medium.onnx.json", 1),
        ]);
        let rows = join(Kind::Piper, &cat, &d, &uses(&defaults()));
        let keys = |q: &str, inst: bool| -> Vec<String> {
            filter(&rows, q, inst)
                .iter()
                .map(|r| r.entry.key.clone())
                .collect()
        };
        assert_eq!(keys("en_gb", false).len(), 3);
        assert_eq!(keys("britain medium", false).len(), 3);
        assert_eq!(keys("de_DE high", false), ["de_DE-thorsten-high"]);
        assert_eq!(keys("", true), ["en_GB-alan-medium"]);
        assert_eq!(keys("tower", false), ["en_GB-alba-medium"]);
        assert!(keys("nothing-like-it", false).is_empty());
    }

    #[test]
    fn partial_downloads_are_found_and_the_cache_lasts_a_day() {
        let d = disk(&[("a.onnx.part", 1), ("a.onnx", 1), ("ggml-x.bin.part", 2)]);
        assert_eq!(stale_parts(&d), ["a.onnx.part", "ggml-x.bin.part"]);
        use std::time::Duration;
        assert!(catalogue_stale(None));
        assert!(!catalogue_stale(Some(Duration::from_secs(3600))));
        assert!(catalogue_stale(Some(Duration::from_secs(25 * 3600))));
    }

    #[test]
    fn sizes_read_as_the_list_shows_them() {
        assert_eq!(human_size(63_201_294), "63 MB");
        assert_eq!(human_size(1_624_555_275), "1.6 GB");
        assert_eq!(human_size(4888), "5 kB");
        assert_eq!(human_size(0), "0 MB");
    }
}
