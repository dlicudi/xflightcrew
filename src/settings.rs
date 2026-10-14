//! The live settings store and the registry that describes it.
//!
//! Two problems, one module.
//!
//! **The store.** [`Config`] used to be built once by `Config::from_env` and
//! cloned into each thread, so every knob was a boot-time constant even when
//! its consumer re-read it on every tick. The value lives here in an
//! `Arc<Config>` instead: consumers take a fresh [`live`] handle per tick (one
//! line at the top of a loop), which makes every already-per-tick field
//! settable while the sim runs, at the cost of one rwlock read a tick.
//!
//! **The registry.** A setting is declared ONCE, here: what it is called, which
//! environment variable pins it, how to render and validate it, how to apply it
//! live, and where it persists. The terminal's Settings screen is a view of
//! this list rather than a second copy of the schema, and the in-sim panel's
//! own pickers can move onto it without inventing a third precedence chain.
//!
//! Precedence is the project's existing rule and is not negotiable here:
//! **explicit env var > saved preference > built-in default**. A knob whose env
//! var is set is PINNED: [`set`] refuses it and the screen says so. Silently
//! accepting an edit that a `.env` line overrides at the next boot is the
//! "always end up back with ryan" bug (see `config::Config::from_env`), and a
//! settings screen is exactly where it would happen again.

use std::sync::{Arc, OnceLock, RwLock};

use crate::config::Config;

/// The process-wide live configuration.
fn cell() -> &'static RwLock<Arc<Config>> {
    static LIVE: OnceLock<RwLock<Arc<Config>>> = OnceLock::new();
    LIVE.get_or_init(|| RwLock::new(Arc::new(Config::default())))
}

/// The configuration as it stands right now. Cheap enough to call per tick;
/// hold the returned handle for the duration of one tick, never longer, or a
/// setting change won't be seen.
pub fn live() -> Arc<Config> {
    cell().read().unwrap_or_else(|e| e.into_inner()).clone()
}

/// Publish the boot configuration, then apply any saved preferences for
/// registry knobs whose environment variable is silent. Returns the resolved
/// configuration, which is what the rest of boot should use.
pub fn install(cfg: Config) -> Arc<Config> {
    *cell().write().unwrap_or_else(|e| e.into_inner()) = Arc::new(cfg);
    for s in ALL {
        // `Own` settings carry their own boot path in `main` (the panel
        // protocol's mode/cabin pickers); re-applying them here would be a
        // second, competing precedence chain for the same value.
        // A `Restart` knob's pref was read where boot reads it
        // (`config::knob`); applying it again here would re-run its checks
        // (an apt.dat scan for the home field) for nothing.
        if !matches!(s.persist, Persist::Map) || s.tier == Tier::Restart || s.env_pinned() {
            continue;
        }
        apply_saved(s, crate::prefs::setting(s.key).as_deref());
    }
    live()
}

/// The middle rung of env > pref > default for one knob: apply a saved value
/// unless the environment pins the knob. `true` = the saved value is now in
/// force. An unreadable or rejected value keeps the default, said in the log.
fn apply_saved(s: &Setting, saved: Option<&str>) -> bool {
    if s.env_pinned() {
        return false;
    }
    let Some(raw) = saved else {
        return false;
    };
    match s.kind.parse(raw) {
        Some(v) => {
            if let Err(why) = s.kind.validate(&v).and_then(|()| (s.check)(&v)) {
                tracing::warn!(key = s.key, value = %raw, %why, "saved setting refused; keeping the default");
                return false;
            }
            if (s.apply)(&v) {
                return true;
            }
            tracing::warn!(key = s.key, value = %raw, "saved setting rejected; keeping the default");
        }
        None => {
            tracing::warn!(key = s.key, value = %raw, "saved setting unreadable; keeping the default")
        }
    }
    false
}

/// Replace the live configuration with a modified copy. Copy-on-write, so a
/// consumer holding a handle finishes its tick on a consistent snapshot rather
/// than seeing a field change mid-decision.
pub fn update(f: impl FnOnce(&mut Config)) {
    let mut guard = cell().write().unwrap_or_else(|e| e.into_inner());
    let mut next = (**guard).clone();
    f(&mut next);
    *guard = Arc::new(next);
}

/// A setting's value, in the four shapes the registry supports.
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Bool(bool),
    Num(f64),
    Choice(String),
    /// Free text: a callsign, a telephony table. Empty = none.
    Text(String),
}

impl Value {
    fn as_bool(&self) -> Option<bool> {
        match self {
            Value::Bool(b) => Some(*b),
            _ => None,
        }
    }
    fn as_num(&self) -> Option<f64> {
        match self {
            Value::Num(n) => Some(*n),
            _ => None,
        }
    }
    fn as_choice(&self) -> Option<&str> {
        match self {
            Value::Choice(s) => Some(s.as_str()),
            _ => None,
        }
    }
    pub fn as_text(&self) -> Option<&str> {
        match self {
            Value::Text(s) => Some(s.as_str()),
            _ => None,
        }
    }
}

/// What a setting accepts, and how the screen renders and steps it.
#[derive(Clone, Copy)]
pub enum Kind {
    Bool,
    Num {
        min: f64,
        max: f64,
        step: f64,
        decimals: usize,
        unit: &'static str,
    },
    Choice(&'static [&'static str]),
    /// One line of free text, at most `max_len` characters. The screen
    /// cannot step it; the panel and the terminal edit it as a field.
    Text {
        max_len: usize,
    },
    /// A file (`dir: false`) or folder (`dir: true`) on this machine. Empty
    /// = the default location. A path that is not there is refused on
    /// [`set`], named.
    Path {
        dir: bool,
    },
    /// A Piper voice spec, `path/to/model.onnx` with an optional `#N`
    /// speaker of a multi-speaker model. Accepted on [`set`] only when
    /// installed (`voice::voice_installed`), else refused naming `xfc
    /// voices`. Empty = the slot's default (an ATC position inherits the
    /// tower's); whether a slot may be empty is its own `apply`'s call.
    Voice,
}

/// The longest path or voice spec a [`Kind::Path`] or [`Kind::Voice`]
/// value may be.
const PATH_MAX: usize = 1024;

/// One line of text, dashes made plain, at most `max_len` characters: what
/// a [`Kind::Text`], [`Kind::Path`] or [`Kind::Voice`] accepts.
fn one_line(raw: &str, max_len: usize) -> Option<String> {
    let text: String = raw
        .chars()
        .map(|c| {
            if matches!(c, '\u{2013}' | '\u{2014}') {
                '-'
            } else {
                c
            }
        })
        .collect();
    (text.chars().count() <= max_len && !text.chars().any(char::is_control)).then_some(text)
}

impl Kind {
    /// Read a stored (or typed) value back. `None` = unreadable or out of
    /// range, which is always treated as "keep what we have".
    pub fn parse(&self, raw: &str) -> Option<Value> {
        let raw = raw.trim();
        match self {
            Kind::Bool => match raw.to_ascii_lowercase().as_str() {
                "true" | "on" | "1" | "yes" => Some(Value::Bool(true)),
                "false" | "off" | "0" | "no" => Some(Value::Bool(false)),
                _ => None,
            },
            Kind::Num {
                min, max, decimals, ..
            } => {
                let n: f64 = raw.parse().ok()?;
                (n >= *min && n <= *max).then(|| Value::Num(round_to(n, *decimals)))
            }
            Kind::Choice(options) => options
                .iter()
                .find(|o| o.eq_ignore_ascii_case(raw))
                .map(|o| Value::Choice((*o).to_string())),
            // One line, no control characters, and no em or en dash (the
            // panel font and the TTS choke on them): a pasted one becomes a
            // hyphen rather than a refusal.
            Kind::Text { max_len } => one_line(raw, *max_len).map(Value::Text),
            Kind::Path { .. } | Kind::Voice => one_line(raw, PATH_MAX).map(Value::Text),
        }
    }

    /// What the KIND alone refuses, checked by [`set`] before the setting's
    /// own `apply`: a path that is not there, a voice not installed. The
    /// error is written for the pilot.
    pub fn validate(&self, v: &Value) -> Result<(), String> {
        let Some(t) = v.as_text().map(str::trim) else {
            return Ok(());
        };
        if t.is_empty() {
            return Ok(());
        }
        match self {
            Kind::Path { dir } => {
                let p = std::path::Path::new(t);
                match (dir, p.is_dir(), p.is_file()) {
                    (true, true, _) | (false, _, true) => Ok(()),
                    (true, false, true) => Err(format!("{t} is a file, not a folder")),
                    (false, true, _) => Err(format!("{t} is a folder, not a file")),
                    (true, ..) => Err(format!("no folder at {t}")),
                    (false, ..) => Err(format!("no file at {t}")),
                }
            }
            Kind::Voice => {
                if crate::voice::voice_installed(t) {
                    Ok(())
                } else {
                    Err(format!(
                        "{} is not installed: `xfc voices` installs it",
                        voice_name(t)
                    ))
                }
            }
            _ => Ok(()),
        }
    }

    /// How a value is written to prefs.json and shown on screen.
    pub fn render(&self, v: &Value) -> String {
        match (self, v) {
            (Kind::Bool, Value::Bool(b)) => if *b { "on" } else { "off" }.to_string(),
            (Kind::Num { decimals, unit, .. }, Value::Num(n)) => {
                let num = format!("{n:.*}", *decimals);
                if unit.is_empty() {
                    num
                } else {
                    format!("{num} {unit}")
                }
            }
            (_, Value::Choice(s)) => s.clone(),
            (Kind::Text { .. }, Value::Text(s)) if s.is_empty() => "(none)".to_string(),
            (Kind::Text { .. }, Value::Text(s)) => s.clone(),
            (Kind::Path { .. } | Kind::Voice, Value::Text(s)) if s.is_empty() => {
                "(default)".to_string()
            }
            (Kind::Voice, Value::Text(s)) => voice_name(s),
            (Kind::Path { .. }, Value::Text(s)) => s.clone(),
            // A mismatched pair can only come from a bad registry entry.
            (_, other) => format!("{other:?}"),
        }
    }

    /// The value as prefs.json stores it: no unit suffix, so it round-trips
    /// through [`Kind::parse`].
    pub fn store(&self, v: &Value) -> String {
        match (self, v) {
            (Kind::Num { decimals, .. }, Value::Num(n)) => format!("{n:.*}", *decimals),
            // Raw, never the "(none)" the screen shows for an empty one,
            // and a voice as its whole spec, never the short name.
            (Kind::Text { .. } | Kind::Path { .. } | Kind::Voice, Value::Text(s)) => s.clone(),
            _ => self.render(v),
        }
    }

    /// The next value when the screen steps this setting up or down.
    pub fn step(&self, v: &Value, up: bool) -> Value {
        match (self, v) {
            (Kind::Bool, Value::Bool(b)) => Value::Bool(!b),
            (
                Kind::Num {
                    min,
                    max,
                    step,
                    decimals,
                    ..
                },
                Value::Num(n),
            ) => {
                let next = if up { n + step } else { n - step };
                Value::Num(round_to(next.clamp(*min, *max), *decimals))
            }
            (Kind::Choice(options), Value::Choice(s)) => {
                let i = options.iter().position(|o| o == s).unwrap_or(0);
                let n = options.len();
                let next = if up { i + 1 } else { i + n - 1 };
                Value::Choice(options[next % n].to_string())
            }
            (_, other) => other.clone(),
        }
    }
}

/// A traffic density as the registry shows it.
fn density_word(d: traffic::Density) -> &'static str {
    match d {
        traffic::Density::Off => "off",
        traffic::Density::Light => "light",
        traffic::Density::Normal => "normal",
    }
}

fn round_to(v: f64, decimals: usize) -> f64 {
    let f = 10f64.powi(decimals as i32);
    (v * f).round() / f
}

/// Where a setting's chosen value is remembered.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Persist {
    /// The registry owns it: prefs.json's `settings` map, and the boot-time
    /// re-apply in [`install`].
    Map,
    /// The setting already has its own prefs field and boot path (the panel
    /// protocol's mode and cabin-crew pickers, which the plugin also writes).
    /// `apply` persists it; [`install`] leaves it alone.
    Own,
}

/// When a change takes effect, which is the honest thing to show next to a
/// knob that cannot apply until the subsystem restarts.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Tier {
    /// Applies to the next tick, utterance, or transmission.
    Live,
    /// Read once, at boot, and in force from the NEXT start: its pref is
    /// resolved where boot reads it (`config::BOOT_KNOBS`, `config::knob`),
    /// never by [`install`], so the screens that run before install (the
    /// voices screen, the home screen) and the threads that capture it at
    /// boot see the same value. A change is saved and shown at once.
    Restart,
}

impl Tier {
    pub fn describe(self) -> &'static str {
        match self {
            Tier::Live => "applies immediately",
            Tier::Restart => "applies on restart",
        }
    }
}

/// Where a setting is shown: the terminal groups by it, and part 2's tabs
/// are these.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Section {
    Crew,
    Voices,
    Atc,
    AudioPtt,
    Identity,
    Traffic,
    Paths,
    Engine,
    Efb,
}

impl Section {
    /// Every section, in the order the screens show them.
    pub const ALL: [Section; 9] = [
        Section::Crew,
        Section::Voices,
        Section::Atc,
        Section::AudioPtt,
        Section::Identity,
        Section::Traffic,
        Section::Paths,
        Section::Engine,
        Section::Efb,
    ];

    pub fn title(self) -> &'static str {
        match self {
            Section::Crew => "Crew",
            Section::Voices => "Voices",
            Section::Atc => "ATC",
            Section::AudioPtt => "Audio and PTT",
            Section::Identity => "Identity and SimBrief",
            Section::Traffic => "Traffic",
            Section::Paths => "Paths",
            Section::Engine => "Engine",
            Section::Efb => "EFB",
        }
    }
}

/// The registry in screen order: grouped by [`Section`], declaration order
/// inside each. The terminal's Settings view walks this.
pub fn by_section() -> Vec<&'static Setting> {
    Section::ALL
        .iter()
        .flat_map(|sec| ALL.iter().filter(move |s| s.section == *sec))
        .collect()
}

/// A voice spec's short name: the model's file stem, its `#N` speaker kept
/// (`models/en_GB-vctk-medium.onnx#4` is `en_GB-vctk-medium#4`).
pub fn voice_name(spec: &str) -> String {
    let (path, speaker) = match spec.rsplit_once('#') {
        Some((p, n)) => (p, Some(n)),
        None => (spec, None),
    };
    let file = path.rsplit('/').next().unwrap_or(path);
    let stem = file.strip_suffix(".onnx").unwrap_or(file);
    match speaker {
        Some(n) => format!("{stem}#{n}"),
        None => stem.to_string(),
    }
}

/// One knob.
pub struct Setting {
    /// prefs.json key, and the identifier the screen and tests use.
    pub key: &'static str,
    /// The environment variable that pins it. Empty = none can.
    pub env: &'static str,
    pub label: &'static str,
    /// One line, written for someone deciding whether to change it.
    pub help: &'static str,
    pub kind: Kind,
    pub tier: Tier,
    pub persist: Persist,
    pub section: Section,
    /// The pilot's pick outranks the environment variable, which is only a
    /// DEFAULT here, never a pin: the copilot's and the cabin's voices,
    /// whose `PIPER_VOICE` line in a `.env` reverted every panel pick at the
    /// next boot ("always end up back with ryan", 2026-07-23, kept in
    /// `Config::from_env`). Such a knob is never refused as pinned.
    pub pick_beats_env: bool,
    /// The value in force right now, read from wherever the truth lives. A
    /// `Restart` knob shows what the NEXT start will use.
    pub get: fn() -> Value,
    /// What the consumer would refuse, said: an ICAO the nav data does not
    /// know, a frequency off the COM band. Run by [`set`] after the kind's
    /// own check and before `apply`.
    pub check: fn(&Value) -> Result<(), String>,
    /// Apply it. `false` = rejected (wrong shape or out of range), in which
    /// case nothing has changed and nothing is persisted.
    pub apply: fn(&Value) -> bool,
}

/// The defaults every entry of [`ALL`] fills in with `..BASE`: a live,
/// registry-persisted knob with no check of its own. Never itself a setting.
const BASE: Setting = Setting {
    key: "",
    env: "",
    label: "",
    help: "",
    kind: Kind::Bool,
    tier: Tier::Live,
    persist: Persist::Map,
    section: Section::Crew,
    pick_beats_env: false,
    get: || Value::Bool(false),
    check: |_| Ok(()),
    apply: |_| false,
};

impl Setting {
    /// Is this knob pinned by an explicit environment variable (including a
    /// line in `.env`, which dotenvy has already loaded into the environment)?
    /// A knob whose pick beats its variable is never pinned.
    pub fn env_pinned(&self) -> bool {
        !self.pick_beats_env && !self.env.is_empty() && std::env::var(self.env).is_ok()
    }
    pub fn value(&self) -> Value {
        (self.get)()
    }
    pub fn rendered(&self) -> String {
        self.kind.render(&self.value())
    }
}

/// Change a setting: validate, apply live, persist. The error is written to be
/// shown to the pilot as-is.
pub fn set(key: &str, v: &Value) -> Result<(), String> {
    let s = ALL
        .iter()
        .find(|s| s.key == key)
        .ok_or_else(|| format!("no such setting: {key}"))?;
    if s.env_pinned() {
        return Err(format!("pinned by {}, edit your .env", s.env));
    }
    s.kind.validate(v)?;
    (s.check)(v)?;
    if !(s.apply)(v) {
        return Err(format!("{} did not accept that value", s.label));
    }
    if s.persist == Persist::Map {
        // Live text is stored as it was READ BACK: a telephony table keeps
        // only the entries that parsed, so prefs.json and the panel show
        // what is in force, not what was typed. A boot knob's readback is
        // the pref itself, not yet written: it is stored as given.
        let stored = match s.kind {
            Kind::Text { .. } if s.tier == Tier::Live => s.kind.store(&s.value()),
            _ => s.kind.store(v),
        };
        // An empty path or voice means "the default": forgotten, so the
        // default (which a later build may move) is what the next read gets.
        if matches!(s.kind, Kind::Path { .. } | Kind::Voice) && stored.trim().is_empty() {
            crate::prefs::remove_setting(s.key);
        } else {
            crate::prefs::save_setting(s.key, &stored);
        }
    }
    tracing::info!(key = s.key, value = %s.kind.render(v), "setting changed");
    Ok(())
}

/// Every declared setting, in the order the screen shows them.
pub const ALL: &[Setting] = &[
    Setting {
        key: "mode",
        section: Section::Crew,
        env: "XCOPILOT_MODE",
        label: "app mode",
        help: "both, copilot only (real ATC has the radio), ATC only, instructor (hands off)",
        kind: Kind::Choice(&["both", "copilot", "atc", "instructor"]),
        tier: Tier::Live,
        persist: Persist::Own,
        get: || Value::Choice(crate::mode::get().as_str().to_string()),
        apply: |v| {
            let Some(m) = v.as_choice().and_then(crate::mode::Mode::parse) else {
                return false;
            };
            crate::mode::set(m);
            crate::prefs::save_mode(m.as_str());
            true
        },
        ..BASE
    },
    Setting {
        key: "cabin_crew",
        section: Section::Crew,
        env: "XCOPILOT_CABIN_CREW",
        label: "cabin crew",
        help: "auto follows the airframe profile; aboard/off override it",
        kind: Kind::Choice(&["auto", "aboard", "off"]),
        tier: Tier::Live,
        persist: Persist::Own,
        get: || Value::Choice(crate::cabin::staffing().as_str().to_string()),
        apply: |v| {
            let Some(s) = v.as_choice().and_then(crate::cabin::Staffing::parse) else {
                return false;
            };
            crate::cabin::set_staffing(s);
            crate::prefs::save_cabin_crew(s.as_str());
            true
        },
        ..BASE
    },
    Setting {
        key: "voice_volume",
        section: Section::AudioPtt,
        env: "XCOPILOT_VOICE_VOL",
        label: "intercom volume",
        help: "the copilot and cabin crew, 1.0 = unity",
        kind: Kind::Num {
            min: 0.0,
            max: 2.0,
            step: 0.05,
            decimals: 2,
            unit: "",
        },
        tier: Tier::Live,
        persist: Persist::Map,
        get: || Value::Num(live().voice_volume as f64),
        apply: |v| apply_volumes(v.as_num(), None),
        ..BASE
    },
    Setting {
        key: "atc_volume",
        section: Section::AudioPtt,
        env: "XCOPILOT_ATC_VOL",
        label: "radio volume",
        help: "controllers, ATIS and party-line traffic, like a COM knob",
        kind: Kind::Num {
            min: 0.0,
            max: 2.0,
            step: 0.05,
            decimals: 2,
            unit: "",
        },
        tier: Tier::Live,
        persist: Persist::Map,
        get: || Value::Num(live().atc_volume as f64),
        apply: |v| apply_volumes(None, v.as_num()),
        ..BASE
    },
    Setting {
        key: "vad_threshold",
        section: Section::AudioPtt,
        env: "XCOPILOT_VAD_RMS",
        label: "mic gate (RMS)",
        help: "raise in a noisy room, lower if quiet speech is missed",
        kind: Kind::Num {
            min: 0.001,
            max: 0.1,
            step: 0.001,
            decimals: 3,
            unit: "",
        },
        tier: Tier::Live,
        persist: Persist::Map,
        get: || Value::Num(live().vad_threshold as f64),
        apply: |v| {
            let Some(n) = v.as_num() else { return false };
            update(|c| c.vad_threshold = n as f32);
            true
        },
        ..BASE
    },
    Setting {
        key: "end_silence_ms",
        section: Section::AudioPtt,
        env: "XCOPILOT_END_SILENCE_MS",
        label: "end of speech",
        help: "trailing silence that ends an utterance; raise if a pause cuts you off",
        kind: Kind::Num {
            min: 200.0,
            max: 3000.0,
            step: 50.0,
            decimals: 0,
            unit: "ms",
        },
        tier: Tier::Live,
        persist: Persist::Map,
        get: || Value::Num(live().end_silence_ms as f64),
        apply: |v| {
            let Some(n) = v.as_num() else { return false };
            update(|c| c.end_silence_ms = n as u32);
            true
        },
        ..BASE
    },
    Setting {
        key: "mic_gate_tail_ms",
        section: Section::AudioPtt,
        env: "XCOPILOT_MIC_GATE_TAIL_MS",
        label: "echo guard",
        help: "mic stays muted this long after the copilot speaks; raise on speakers",
        kind: Kind::Num {
            min: 0.0,
            max: 3000.0,
            step: 50.0,
            decimals: 0,
            unit: "ms",
        },
        tier: Tier::Live,
        persist: Persist::Map,
        get: || Value::Num(live().mic_gate_tail_ms as f64),
        apply: |v| {
            let Some(n) = v.as_num() else { return false };
            update(|c| c.mic_gate_tail_ms = n as u64);
            true
        },
        ..BASE
    },
    Setting {
        key: "stt_min_confidence",
        section: Section::AudioPtt,
        env: "XCOPILOT_STT_MIN_CONFIDENCE",
        label: "act above",
        help: "STT confidence needed to act outright",
        kind: Kind::Num {
            min: 0.0,
            max: 1.0,
            step: 0.01,
            decimals: 2,
            unit: "",
        },
        tier: Tier::Live,
        persist: Persist::Map,
        get: || Value::Num(live().stt_min_confidence as f64),
        apply: |v| {
            let Some(n) = v.as_num() else { return false };
            update(|c| c.stt_min_confidence = n as f32);
            true
        },
        ..BASE
    },
    Setting {
        key: "stt_confirm_floor",
        section: Section::AudioPtt,
        env: "XCOPILOT_STT_CONFIRM_FLOOR",
        label: "confirm above",
        help: "below the act threshold but above this, the copilot reads it back for a yes",
        kind: Kind::Num {
            min: 0.0,
            max: 1.0,
            step: 0.01,
            decimals: 2,
            unit: "",
        },
        tier: Tier::Live,
        persist: Persist::Map,
        get: || Value::Num(live().stt_confirm_floor as f64),
        apply: |v| {
            let Some(n) = v.as_num() else { return false };
            update(|c| c.stt_confirm_floor = n as f32);
            true
        },
        ..BASE
    },
    Setting {
        key: "confirm_critical",
        section: Section::Crew,
        env: "XCOPILOT_CONFIRM",
        label: "confirm critical",
        help: "require a spoken affirm before autopilot flight-path changes",
        kind: Kind::Bool,
        tier: Tier::Live,
        persist: Persist::Map,
        get: || Value::Bool(live().confirm_critical),
        apply: |v| {
            let Some(b) = v.as_bool() else { return false };
            update(|c| c.confirm_critical = b);
            true
        },
        ..BASE
    },
    Setting {
        key: "taxi_guide",
        section: Section::Crew,
        env: "XCOPILOT_TAXI_GUIDE",
        label: "taxi guide",
        help: "the copilot's just-in-time turn and crossing calls along the cleared route",
        kind: Kind::Bool,
        tier: Tier::Live,
        persist: Persist::Map,
        get: || Value::Bool(live().taxi_guide),
        apply: |v| {
            let Some(b) = v.as_bool() else { return false };
            update(|c| c.taxi_guide = b);
            true
        },
        ..BASE
    },
    Setting {
        key: "atc_readback",
        section: Section::Atc,
        env: "XCOPILOT_ATC_READBACK",
        label: "readback challenges",
        help: "the tower challenges a missing or wrong-runway readback; coach stays either way",
        kind: Kind::Bool,
        tier: Tier::Live,
        persist: Persist::Map,
        get: || Value::Bool(live().atc_readback),
        apply: |v| {
            let Some(b) = v.as_bool() else { return false };
            update(|c| c.atc_readback = b);
            true
        },
        ..BASE
    },
    Setting {
        key: "atc_chatter",
        section: Section::Atc,
        env: "XCOPILOT_ATC_CHATTER",
        label: "party line",
        help: "the tower narrates clearances for live traffic; safety watches stay on",
        kind: Kind::Bool,
        tier: Tier::Live,
        persist: Persist::Map,
        get: || Value::Bool(live().atc_chatter),
        apply: |v| {
            let Some(b) = v.as_bool() else { return false };
            update(|c| c.atc_chatter = b);
            true
        },
        ..BASE
    },
    Setting {
        key: "traffic",
        section: Section::Traffic,
        env: "XCOPILOT_TRAFFIC",
        label: "traffic",
        help: "ours: our own traffic flies the field, controlled by the tower; real or off: the sim's traffic, narrated",
        kind: Kind::Choice(&["off", "real", "ours"]),
        tier: Tier::Live,
        persist: Persist::Map,
        get: || {
            Value::Choice(
                crate::traffic_link::TrafficMode::parse(&live().traffic)
                    .as_str()
                    .to_string(),
            )
        },
        apply: |v| {
            let Some(m) = v.as_choice() else { return false };
            let m = crate::traffic_link::TrafficMode::parse(m).as_str().to_string();
            update(|c| c.traffic = m);
            true
        },
        ..BASE
    },
    Setting {
        key: "traffic_density",
        section: Section::Traffic,
        env: "XCOPILOT_TRAFFIC_DENSITY",
        label: "traffic density",
        help: "how busy our own generated traffic is; a new field's schedule takes it",
        kind: Kind::Choice(&["off", "light", "normal"]),
        tier: Tier::Live,
        persist: Persist::Map,
        get: || {
            Value::Choice(
                traffic::Density::parse(&live().traffic_density)
                    .map_or("normal", density_word)
                    .to_string(),
            )
        },
        apply: |v| {
            let Some(d) = v.as_choice().and_then(traffic::Density::parse) else {
                return false;
            };
            update(|c| c.traffic_density = density_word(d).to_string());
            true
        },
        ..BASE
    },
    Setting {
        key: "traffic_file",
        section: Section::Traffic,
        env: "XCOPILOT_TRAFFIC_FILE",
        label: "traffic file",
        help: "blank = generated; a YAML schedule of your own (t_s, callsign, type, kind), read when a field's traffic starts",
        kind: Kind::Text { max_len: 400 },
        tier: Tier::Live,
        persist: Persist::Map,
        get: || Value::Text(live().traffic_file.clone()),
        apply: |v| {
            let Some(t) = v.as_text() else { return false };
            let t = t.trim().to_string();
            update(|c| c.traffic_file = t);
            true
        },
        ..BASE
    },
    Setting {
        key: "crew_chatter",
        section: Section::Crew,
        env: "XCOPILOT_CREW_CHATTER",
        label: "copilot chat",
        help: "what the copilot volunteers: landing word, offers, remarks; answers and safety calls always",
        kind: Kind::Choice(&["quiet", "normal", "chatty"]),
        tier: Tier::Live,
        persist: Persist::Map,
        get: || {
            Value::Choice(
                crate::persona::Chatter::parse(&live().crew_chatter)
                    .as_str()
                    .to_string(),
            )
        },
        apply: |v| {
            let Some(c) = v.as_choice() else { return false };
            let c = crate::persona::Chatter::parse(c).as_str().to_string();
            update(|cfg| cfg.crew_chatter = c);
            true
        },
        ..BASE
    },
    Setting {
        key: "start_calls",
        section: Section::Crew,
        env: "XCOPILOT_START_CALLS",
        label: "start-up calls",
        help: "milestones: the start callouts a crew makes (default); every item: each switch called as it is confirmed",
        kind: Kind::Choice(&["milestones", "every item"]),
        tier: Tier::Live,
        persist: Persist::Map,
        get: || {
            Value::Choice(
                crate::crew_pace::StartCalls::parse(&live().start_calls)
                    .as_str()
                    .to_string(),
            )
        },
        apply: |v| {
            let Some(c) = v.as_choice() else { return false };
            let c = crate::crew_pace::StartCalls::parse(c).as_str().to_string();
            update(|cfg| cfg.start_calls = c);
            true
        },
        ..BASE
    },
    Setting {
        key: "crew_dialect",
        section: Section::Crew,
        env: "XCOPILOT_CREW_DIALECT",
        label: "copilot idiom",
        help: "auto follows the voice's accent (\"cheers\" or \"you bet\"); british or american picks one",
        kind: Kind::Choice(&["auto", "british", "american"]),
        tier: Tier::Live,
        persist: Persist::Map,
        get: || {
            let d = live().crew_dialect.trim().to_ascii_lowercase();
            let d = if d == "british" || d == "american" {
                d
            } else {
                "auto".to_string()
            };
            Value::Choice(d)
        },
        apply: |v| {
            let Some(d) = v.as_choice() else { return false };
            let d = d.to_string();
            update(|cfg| cfg.crew_dialect = d);
            true
        },
        ..BASE
    },
    Setting {
        key: "callsign",
        section: Section::Identity,
        env: "XCOPILOT_CALLSIGN",
        label: "callsign",
        help: "blank = the SimBrief plan's, then the tail; an id (BAW306, N172SP) or the words (Speedbird 306)",
        kind: Kind::Text { max_len: 32 },
        tier: Tier::Live,
        persist: Persist::Map,
        get: || Value::Text(live().callsign.clone()),
        apply: |v| {
            let Some(t) = v.as_text() else { return false };
            let t = t.split_whitespace().collect::<Vec<_>>().join(" ");
            // Letters, digits, spaces and hyphens ("G-ABCD"): anything else
            // is not a callsign, and a '|' would break the panel's rows.
            let ok = t
                .chars()
                .all(|c| c.is_ascii_alphanumeric() || c == ' ' || c == '-');
            if !ok || (!t.is_empty() && !t.chars().any(|c| c.is_ascii_alphanumeric())) {
                return false;
            }
            update(|c| c.callsign = t);
            crate::monitor::callsign_changed();
            true
        },
        ..BASE
    },
    Setting {
        key: "airline_telephony",
        section: Section::Identity,
        env: "XCOPILOT_AIRLINE_TELEPHONY",
        label: "airline telephony",
        help: "your airlines' radio names, asked before the built-in ones: WAT=Walker; XYZ=Zulu Air",
        kind: Kind::Text { max_len: 400 },
        tier: Tier::Live,
        persist: Persist::Map,
        get: || {
            Value::Text(crate::callsign::telephony_text(
                &crate::callsign::parse_telephony(&live().airline_telephony),
            ))
        },
        apply: |v| {
            let Some(t) = v.as_text() else { return false };
            let entries = crate::callsign::parse_telephony(t);
            // Something typed and nothing readable is a refusal, not a
            // silent wipe of the table.
            if entries.is_empty() && !t.trim().is_empty() {
                return false;
            }
            let text = crate::callsign::telephony_text(&entries);
            update(|c| c.airline_telephony = text);
            crate::monitor::callsign_changed();
            true
        },
        ..BASE
    },
    Setting {
        key: "audio_bus",
        section: Section::AudioPtt,
        env: "XCOPILOT_AUDIO_BUS",
        label: "voices through X-Plane audio",
        help: "play the crew and ATC through the sim's own audio (the plugin's bus) instead of the Mac's output",
        kind: Kind::Bool,
        tier: Tier::Live,
        persist: Persist::Map,
        get: || Value::Bool(crate::tts::AUDIO_BUS.load(std::sync::atomic::Ordering::Relaxed)),
        apply: |v| {
            let Some(b) = v.as_bool() else { return false };
            update(|c| c.audio_bus = b);
            crate::tts::set_audio_bus(b)
        },
        ..BASE
    },
    Setting {
        key: "efb",
        section: Section::Efb,
        env: "XCOPILOT_EFB",
        label: "EFB",
        help: "serve the EFB page on the LAN (iPad, iPhone, a second screen); the URL is in the log",
        kind: Kind::Bool,
        tier: Tier::Live,
        persist: Persist::Map,
        get: || Value::Bool(live().efb),
        apply: |v| {
            let Some(b) = v.as_bool() else { return false };
            update(|c| c.efb = b);
            true
        },
        ..BASE
    },
    Setting {
        key: "efb_port",
        section: Section::Efb,
        env: "XCOPILOT_EFB_PORT",
        label: "EFB port",
        help: "the TCP port the EFB listens on",
        kind: Kind::Num {
            min: 1024.0,
            max: 65535.0,
            step: 1.0,
            decimals: 0,
            unit: "",
        },
        tier: Tier::Live,
        persist: Persist::Map,
        get: || Value::Num(live().efb_port as f64),
        apply: |v| {
            let Some(n) = v.as_num().filter(|n| (1024.0..=65535.0).contains(n)) else {
                return false;
            };
            update(|c| c.efb_port = n as u16);
            true
        },
        ..BASE
    },
    Setting {
        key: "efb_token",
        section: Section::Efb,
        env: "XCOPILOT_EFB_TOKEN",
        label: "EFB token",
        help: "the EFB link's key (letters and digits, 8 or more); blank = a new one at the next enable",
        kind: Kind::Text { max_len: 64 },
        tier: Tier::Live,
        persist: Persist::Map,
        get: || Value::Text(live().efb_token.clone()),
        apply: |v| {
            let Some(t) = v.as_text() else { return false };
            let t = t.trim();
            if !crate::efb::token_ok(t) && !t.is_empty() {
                return false;
            }
            let t = t.to_string();
            update(|c| c.efb_token = t);
            true
        },
        ..BASE
    },
    // ---- Voices: who speaks with what (`voice::cast` reads these) ----
    Setting {
        key: "copilot_voice",
        env: "PIPER_VOICE",
        label: "copilot voice",
        help: "the first officer's voice, an installed Piper model; your pick outranks a PIPER_VOICE line",
        kind: Kind::Voice,
        persist: Persist::Own,
        section: Section::Voices,
        pick_beats_env: true,
        get: || Value::Text(live().piper_voice.clone()),
        apply: apply_copilot_voice,
        ..BASE
    },
    Setting {
        key: "cabin_voice",
        env: "XCOPILOT_CABIN_VOICE",
        label: "cabin voice",
        help: "the cabin crew's voice; never the copilot's",
        kind: Kind::Voice,
        persist: Persist::Own,
        section: Section::Voices,
        pick_beats_env: true,
        get: || Value::Text(live().cabin_piper_voice.clone()),
        check: |v| {
            let t = text(v).unwrap_or_default();
            if !t.is_empty() && t == live().piper_voice {
                return Err("that is the copilot's voice: pick another for the cabin".into());
            }
            Ok(())
        },
        apply: apply_cabin_voice,
        ..BASE
    },
    Setting {
        key: "atc_tower_voice",
        env: "XCOPILOT_ATC_PIPER_VOICE",
        label: "home tower voice",
        help: "the home field's tower (and its ground and delivery); blank = the built-in voice",
        kind: Kind::Voice,
        tier: Tier::Restart,
        section: Section::Voices,
        get: || Value::Text(live().atc_piper_voice.clone()),
        apply: |v| {
            let Some(t) = text(v) else { return false };
            let t = voice_or_default(t, "XCOPILOT_ATC_PIPER_VOICE");
            update(|c| c.atc_piper_voice = t);
            true
        },
        ..BASE
    },
    Setting {
        key: "atc_approach_voice",
        env: "XCOPILOT_ATC_APPROACH_PIPER_VOICE",
        label: "home approach voice",
        help: "the home field's approach controller; blank = the built-in voice",
        kind: Kind::Voice,
        tier: Tier::Restart,
        section: Section::Voices,
        get: || Value::Text(live().atc_approach_piper_voice.clone()),
        apply: |v| {
            let Some(t) = text(v) else { return false };
            let t = voice_or_default(t, "XCOPILOT_ATC_APPROACH_PIPER_VOICE");
            update(|c| c.atc_approach_piper_voice = t);
            true
        },
        ..BASE
    },
    Setting {
        key: "atc_departure_voice",
        env: "XCOPILOT_ATC_DEPARTURE_PIPER_VOICE",
        label: "home departure voice",
        help: "the home field's departure controller; blank = the built-in voice",
        kind: Kind::Voice,
        tier: Tier::Restart,
        section: Section::Voices,
        get: || Value::Text(live().atc_departure_piper_voice.clone()),
        apply: |v| {
            let Some(t) = text(v) else { return false };
            let t = voice_or_default(t, "XCOPILOT_ATC_DEPARTURE_PIPER_VOICE");
            update(|c| c.atc_departure_piper_voice = t);
            true
        },
        ..BASE
    },
    Setting {
        key: "atc_atis_voice",
        env: "XCOPILOT_ATC_ATIS_PIPER_VOICE",
        label: "home ATIS voice",
        help: "the home field's ATIS reader; blank = the tower's voice",
        kind: Kind::Voice,
        tier: Tier::Restart,
        section: Section::Voices,
        get: || Value::Text(live().atc_atis_piper_voice.clone()),
        apply: |v| {
            let Some(t) = text(v) else { return false };
            update(|c| c.atc_atis_piper_voice = t);
            true
        },
        ..BASE
    },
    Setting {
        key: "atc_voice_pool",
        env: "XCOPILOT_ATC_VOICE_POOL",
        label: "away-field voices",
        help: "the voices every other field's controllers are drawn from, comma-separated; blank = the home voices everywhere",
        kind: Kind::Text { max_len: 16_000 },
        tier: Tier::Restart,
        section: Section::Voices,
        get: || Value::Text(live().atc_voice_pool.clone()),
        check: check_pool,
        apply: |v| {
            let Some(t) = text(v) else { return false };
            let pool = t
                .split(',')
                .map(str::trim)
                .filter(|s| !s.is_empty())
                .collect::<Vec<_>>()
                .join(",");
            update(|c| c.atc_voice_pool = pool);
            true
        },
        ..BASE
    },
    Setting {
        key: "atc_voice_shift",
        env: "XCOPILOT_ATC_VOICE_SHIFT",
        label: "controller pitch shift",
        help: "a small per-controller pitch and pace offset, so a shared model is never the same person twice",
        tier: Tier::Restart,
        section: Section::Voices,
        get: || Value::Bool(live().atc_voice_shift),
        apply: |v| {
            let Some(b) = v.as_bool() else { return false };
            update(|c| c.atc_voice_shift = b);
            true
        },
        ..BASE
    },
    Setting {
        key: "atc_tts_length",
        env: "XCOPILOT_ATC_TTS_LENGTH",
        label: "controller pace",
        help: "blank = each controller's measured pace; a number overrides it for all (lower is faster)",
        kind: Kind::Text { max_len: 8 },
        tier: Tier::Restart,
        section: Section::Voices,
        get: || knob_text("XCOPILOT_ATC_TTS_LENGTH"),
        check: |v| check_scale(v, 0.3, 3.0),
        apply: |v| v.as_text().is_some(),
        ..BASE
    },
    Setting {
        key: "atc_tts_noise_w",
        env: "XCOPILOT_ATC_TTS_NOISE_W",
        label: "controller rhythm",
        help: "the controllers' rhythm variation; blank = 1.0",
        kind: Kind::Text { max_len: 8 },
        tier: Tier::Restart,
        section: Section::Voices,
        get: || knob_text("XCOPILOT_ATC_TTS_NOISE_W"),
        check: |v| check_scale(v, 0.0, 2.0),
        apply: |v| v.as_text().is_some(),
        ..BASE
    },
    // ---- ATC ----
    Setting {
        key: "atc_airport",
        env: "XCOPILOT_ATC_AIRPORT",
        label: "home field",
        help: "the ICAO whose tower the crew hosts at start, cast with the home voices; blank = no ATC",
        kind: Kind::Text { max_len: 4 },
        tier: Tier::Restart,
        section: Section::Atc,
        get: || Value::Text(live().atc_airport.clone()),
        check: check_icao,
        apply: |v| {
            let Some(t) = text(v) else { return false };
            let t = t.to_ascii_uppercase();
            update(|c| c.atc_airport = t);
            true
        },
        ..BASE
    },
    Setting {
        key: "atc_region",
        env: "XCOPILOT_ATC_REGION",
        label: "ATC region",
        help: "auto: the flight is handed to whichever tower is nearest; off: the home field only",
        kind: Kind::Choice(&["auto", "off"]),
        tier: Tier::Restart,
        section: Section::Atc,
        get: || {
            let off = live().atc_region.trim() == "off";
            Value::Choice(if off { "off" } else { "auto" }.into())
        },
        apply: |v| {
            let Some(c) = v.as_choice() else { return false };
            let c = c.to_string();
            update(|cfg| cfg.atc_region = c);
            true
        },
        ..BASE
    },
    // ---- Audio and PTT ----
    Setting {
        key: "ptt_dataref",
        env: "XCOPILOT_PTT_DATAREF",
        label: "intercom PTT dataref",
        help: "the plugin's push-to-talk, bound to a button in X-Plane; blank = the PTT key below",
        kind: Kind::Text { max_len: 128 },
        tier: Tier::Restart,
        section: Section::AudioPtt,
        get: || Value::Text(live().ptt_dataref.clone()),
        check: check_dataref,
        apply: |v| {
            let Some(t) = text(v) else { return false };
            update(|c| c.ptt_dataref = t);
            true
        },
        ..BASE
    },
    Setting {
        key: "radio_ptt_dataref",
        env: "XCOPILOT_RADIO_PTT_DATAREF",
        label: "radio PTT dataref",
        help: "the transmit switch: held, a call goes to the tuned station; blank = routed by what you say",
        kind: Kind::Text { max_len: 128 },
        tier: Tier::Restart,
        section: Section::AudioPtt,
        get: || Value::Text(live().radio_ptt_dataref.clone()),
        check: check_dataref,
        apply: |v| {
            let Some(t) = text(v) else { return false };
            update(|c| c.radio_ptt_dataref = t);
            true
        },
        ..BASE
    },
    Setting {
        key: "ptt_key",
        env: "XCOPILOT_PTT_KEY",
        label: "PTT key",
        help: "a keyboard key held to talk, used when the PTT dataref is blank (`xfc key-probe` names yours)",
        kind: Kind::Text { max_len: 32 },
        tier: Tier::Restart,
        section: Section::AudioPtt,
        get: || Value::Text(live().ptt_key.clone()),
        check: check_ptt_key,
        apply: |v| {
            let Some(t) = text(v) else { return false };
            update(|c| c.ptt_key = t);
            true
        },
        ..BASE
    },
    Setting {
        key: "mute_xp_radio",
        env: "XCOPILOT_MUTE_XP_RADIO",
        label: "mute X-Plane's radio",
        help: "silence the sim's own ATC and ATIS voice while ours is on (also the nav-aid idents)",
        tier: Tier::Restart,
        section: Section::AudioPtt,
        get: || Value::Bool(live().mute_xp_radio),
        apply: |v| {
            let Some(b) = v.as_bool() else { return false };
            update(|c| c.mute_xp_radio = b);
            true
        },
        ..BASE
    },
    Setting {
        key: "chime",
        env: "XCOPILOT_CHIME",
        label: "reminder chime",
        help: "a WAV or AIFF played before a reminder; blank = the system's Glass",
        kind: Kind::Path { dir: false },
        tier: Tier::Restart,
        section: Section::AudioPtt,
        get: || Value::Text(live().chime_path.clone()),
        apply: |v| {
            let Some(t) = text(v) else { return false };
            update(|c| c.chime_path = t);
            true
        },
        ..BASE
    },
    Setting {
        key: "negative_tone",
        env: "XCOPILOT_NEGATIVE_TONE",
        label: "not-understood tone",
        help: "a WAV or AIFF played when a clear command was not recognised; blank = the system's Basso",
        kind: Kind::Path { dir: false },
        tier: Tier::Restart,
        section: Section::AudioPtt,
        get: || Value::Text(live().negative_tone_path.clone()),
        apply: |v| {
            let Some(t) = text(v) else { return false };
            update(|c| c.negative_tone_path = t);
            true
        },
        ..BASE
    },
    Setting {
        key: "squelch_wav",
        env: "XCOPILOT_SQUELCH_WAV",
        label: "squelch tail",
        help: "the recorded click at the end of each transmission; blank = the built-in one",
        kind: Kind::Path { dir: false },
        tier: Tier::Restart,
        section: Section::AudioPtt,
        get: || Value::Text(live().atc_squelch_wav.clone()),
        apply: |v| {
            let Some(t) = text(v) else { return false };
            let t = if t.is_empty() {
                sim::paths::asset("afv/Click_f32.wav").to_string_lossy().into_owned()
            } else {
                t
            };
            update(|c| c.atc_squelch_wav = t);
            true
        },
        ..BASE
    },
    // ---- Identity and SimBrief ----
    Setting {
        key: "simbrief_id",
        env: "XCOPILOT_SIMBRIEF_ID",
        label: "SimBrief id",
        help: "your SimBrief user number, for \"fetch the flight plan\"; blank = no SimBrief",
        kind: Kind::Text { max_len: 12 },
        section: Section::Identity,
        get: || Value::Text(live().simbrief_id.clone()),
        check: check_simbrief,
        apply: |v| {
            let Some(t) = text(v) else { return false };
            update(|c| c.simbrief_id = t);
            true
        },
        ..BASE
    },
    Setting {
        key: "pilot_name",
        env: "XCOPILOT_PILOT_NAME",
        label: "your name",
        help: "what the copilot calls you now and then; blank = never by name",
        kind: Kind::Text { max_len: 32 },
        section: Section::Identity,
        get: || Value::Text(live().pilot_name.clone()),
        check: check_name,
        apply: |v| {
            let Some(t) = text(v) else { return false };
            update(|c| c.pilot_name = t);
            true
        },
        ..BASE
    },
    Setting {
        key: "copilot_name",
        env: "XCOPILOT_COPILOT_NAME",
        label: "copilot's name",
        help: "the copilot's own name, for \"what's your name\"; blank = none",
        kind: Kind::Text { max_len: 32 },
        section: Section::Identity,
        get: || Value::Text(live().copilot_name.clone()),
        check: check_name,
        apply: |v| {
            let Some(t) = text(v) else { return false };
            update(|c| c.copilot_name = t);
            true
        },
        ..BASE
    },
    // ---- Crew ----
    Setting {
        key: "aircraft",
        env: "XCOPILOT_AIRCRAFT",
        label: "aircraft profile",
        help: "auto detects the loaded aircraft; a profile id forces that one",
        kind: Kind::Text { max_len: 64 },
        tier: Tier::Restart,
        section: Section::Crew,
        get: || Value::Text(live().aircraft.clone()),
        check: check_aircraft,
        apply: |v| {
            let Some(t) = text(v) else { return false };
            let t = if t.eq_ignore_ascii_case("auto") || t.is_empty() {
                "auto".to_string()
            } else {
                t
            };
            update(|c| c.aircraft = t);
            true
        },
        ..BASE
    },
    Setting {
        key: "monitor",
        env: "XCOPILOT_MONITOR",
        label: "monitor",
        help: "the background watch that calls out problems; off silences it",
        tier: Tier::Restart,
        section: Section::Crew,
        get: || Value::Bool(live().monitor_enabled),
        apply: |v| {
            let Some(b) = v.as_bool() else { return false };
            update(|c| c.monitor_enabled = b);
            true
        },
        ..BASE
    },
    // ---- Paths ----
    Setting {
        key: "xplane_root",
        env: "XCOPILOT_XPLANE_ROOT",
        label: "X-Plane folder",
        help: "the X-Plane 12 install: nav data, airports and the plugin folder; blank = ~/X-Plane 12",
        kind: Kind::Path { dir: true },
        tier: Tier::Restart,
        section: Section::Paths,
        get: || Value::Text(crate::config::xplane_root_next()),
        apply: |v| v.as_text().is_some(),
        ..BASE
    },
    Setting {
        key: "apt_dat",
        env: "XCOPILOT_APT_DAT",
        label: "airports file",
        help: "the Global Airports apt.dat the traffic and the EFB read; blank = the X-Plane folder's",
        kind: Kind::Path { dir: false },
        tier: Tier::Restart,
        section: Section::Paths,
        get: || Value::Text(crate::config::apt_dat_path_next()),
        apply: |v| v.as_text().is_some(),
        ..BASE
    },
    Setting {
        key: "models_dir",
        env: "XCOPILOT_MODELS_DIR",
        label: "voices folder",
        help: "where the voices and speech models live (`xfc voices` installs there); blank = the data folder's models/",
        kind: Kind::Path { dir: true },
        tier: Tier::Restart,
        section: Section::Paths,
        get: || Value::Text(crate::config::models_dir_next().to_string_lossy().into_owned()),
        apply: |v| v.as_text().is_some(),
        ..BASE
    },
    // ---- Engine ----
    Setting {
        key: "whisper_model",
        env: "WHISPER_MODEL",
        label: "speech model",
        help: "the whisper model that hears you (`xfc voices` installs them); blank = ggml-base.en",
        kind: Kind::Path { dir: false },
        tier: Tier::Restart,
        section: Section::Engine,
        get: || Value::Text(live().whisper_model_path.clone()),
        apply: |v| {
            let Some(t) = text(v) else { return false };
            let t = if t.is_empty() {
                crate::config::model_path("ggml-base.en.bin")
            } else {
                t
            };
            update(|c| c.whisper_model_path = t);
            true
        },
        ..BASE
    },
    Setting {
        key: "whisper_gpu",
        env: "XCOPILOT_WHISPER_GPU",
        label: "speech on the GPU",
        help: "whisper on the Metal GPU (fast); off keeps it on the CPU, away from a GPU-bound sim",
        tier: Tier::Restart,
        section: Section::Engine,
        get: || Value::Bool(live().whisper_gpu),
        apply: |v| {
            let Some(b) = v.as_bool() else { return false };
            update(|c| c.whisper_gpu = b);
            true
        },
        ..BASE
    },
    Setting {
        key: "tts",
        env: "XCOPILOT_TTS",
        label: "speech engine",
        help: "onnx: the Piper voices in process; say: macOS speech; piper: the piper program; off: text only",
        kind: Kind::Choice(&["onnx", "say", "piper", "off"]),
        tier: Tier::Restart,
        section: Section::Engine,
        get: || Value::Choice(live().tts_backend.clone()),
        apply: |v| {
            let Some(c) = v.as_choice() else { return false };
            let c = c.to_string();
            update(|cfg| cfg.tts_backend = c);
            true
        },
        ..BASE
    },
    Setting {
        key: "tts_threads",
        env: "XCOPILOT_TTS_THREADS",
        label: "speech threads",
        help: "CPU threads each voice may use while it speaks",
        kind: Kind::Num {
            min: 1.0,
            max: 16.0,
            step: 1.0,
            decimals: 0,
            unit: "",
        },
        tier: Tier::Restart,
        section: Section::Engine,
        get: || {
            let n = crate::config::knob_next("XCOPILOT_TTS_THREADS")
                .and_then(|v| v.trim().parse::<f64>().ok())
                .filter(|n| *n >= 1.0)
                .unwrap_or(4.0);
            Value::Num(n)
        },
        apply: |v| v.as_num().is_some(),
        ..BASE
    },
    Setting {
        key: "tts_length",
        env: "XCOPILOT_TTS_LENGTH",
        label: "voice pace",
        help: "the crew voices' pace; blank = each model's own (lower is faster)",
        kind: Kind::Text { max_len: 8 },
        tier: Tier::Restart,
        section: Section::Engine,
        get: || knob_text("XCOPILOT_TTS_LENGTH"),
        check: |v| check_scale(v, 0.3, 3.0),
        apply: |v| v.as_text().is_some(),
        ..BASE
    },
    Setting {
        key: "tts_noise",
        env: "XCOPILOT_TTS_NOISE",
        label: "voice timbre",
        help: "the crew voices' timbre variation; blank = each model's own",
        kind: Kind::Text { max_len: 8 },
        tier: Tier::Restart,
        section: Section::Engine,
        get: || knob_text("XCOPILOT_TTS_NOISE"),
        check: |v| check_scale(v, 0.0, 2.0),
        apply: |v| v.as_text().is_some(),
        ..BASE
    },
    Setting {
        key: "tts_noise_w",
        env: "XCOPILOT_TTS_NOISE_W",
        label: "voice rhythm",
        help: "the crew voices' rhythm variation; blank = each model's own",
        kind: Kind::Text { max_len: 8 },
        tier: Tier::Restart,
        section: Section::Engine,
        get: || knob_text("XCOPILOT_TTS_NOISE_W"),
        check: |v| check_scale(v, 0.0, 2.0),
        apply: |v| v.as_text().is_some(),
        ..BASE
    },
    Setting {
        key: "say_voice",
        env: "XCOPILOT_SAY_VOICE",
        label: "macOS voice",
        help: "the `say` engine's voice (`say -v '?'` lists them); blank = the system's",
        kind: Kind::Text { max_len: 40 },
        tier: Tier::Restart,
        section: Section::Engine,
        get: || Value::Text(live().say_voice.clone()),
        apply: |v| {
            let Some(t) = text(v) else { return false };
            update(|c| c.say_voice = t);
            true
        },
        ..BASE
    },
    Setting {
        key: "say_rate",
        env: "XCOPILOT_SAY_RATE",
        label: "macOS voice rate",
        help: "the `say` engine's words a minute; blank = its own",
        kind: Kind::Text { max_len: 3 },
        tier: Tier::Restart,
        section: Section::Engine,
        get: || Value::Text(live().say_rate.clone()),
        check: check_rate,
        apply: |v| {
            let Some(t) = text(v) else { return false };
            update(|c| c.say_rate = t);
            true
        },
        ..BASE
    },
    Setting {
        key: "atc_say_voice",
        env: "XCOPILOT_ATC_VOICE",
        label: "ATC fallback voice",
        help: "the macOS voice a controller speaks in when its Piper voice cannot load",
        kind: Kind::Text { max_len: 40 },
        tier: Tier::Restart,
        section: Section::Engine,
        get: || Value::Text(live().atc_voice.clone()),
        apply: |v| {
            let Some(t) = text(v) else { return false };
            update(|c| c.atc_voice = t);
            true
        },
        ..BASE
    },
    Setting {
        key: "atc_say_rate",
        env: "XCOPILOT_ATC_RATE",
        label: "ATC fallback rate",
        help: "that fallback voice's words a minute",
        kind: Kind::Text { max_len: 3 },
        tier: Tier::Restart,
        section: Section::Engine,
        get: || Value::Text(live().atc_rate.clone()),
        check: check_rate,
        apply: |v| {
            let Some(t) = text(v) else { return false };
            update(|c| c.atc_rate = t);
            true
        },
        ..BASE
    },
];

/// The live airline telephony table, as the atc crate's provider hands it
/// over (`callsign::provide_telephony`, registered in `main`).
pub fn airline_telephony() -> String {
    live().airline_telephony.clone()
}

/// Both channel volumes go to the audio layer together (it takes a pair), so a
/// change to either re-sends both from the live configuration.
fn apply_volumes(voice: Option<f64>, atc: Option<f64>) -> bool {
    if voice.is_none() && atc.is_none() {
        return false;
    }
    update(|c| {
        if let Some(v) = voice {
            c.voice_volume = v as f32;
        }
        if let Some(a) = atc {
            c.atc_volume = a as f32;
        }
    });
    let c = live();
    crate::tts::set_volumes(c.voice_volume, c.atc_volume);
    true
}

/// A text value, trimmed. `None` = not text (a mismatched registry entry).
fn text(v: &Value) -> Option<String> {
    v.as_text().map(|t| t.trim().to_string())
}

/// A boot knob's saved or pinned value as text, for a knob with no
/// `Config` field of its own (the TTS scales, the thread count).
fn knob_text(env: &str) -> Value {
    Value::Text(
        crate::config::knob_next(env)
            .unwrap_or_default()
            .trim()
            .to_string(),
    )
}

/// A voice slot's value: empty is the slot's built-in voice.
fn voice_or_default(t: String, env: &str) -> String {
    if t.is_empty() {
        crate::config::voice_default(env)
    } else {
        t
    }
}

/// Blank, or a number from `min` to `max`: a VITS scale override (blank =
/// the model's own).
fn check_scale(v: &Value, min: f64, max: f64) -> Result<(), String> {
    let t = text(v).unwrap_or_default();
    if t.is_empty() {
        return Ok(());
    }
    match t.parse::<f64>() {
        Ok(n) if (min..=max).contains(&n) => Ok(()),
        _ => Err(format!("blank, or a number from {min} to {max}")),
    }
}

/// Blank, or words per minute the `say` command takes.
fn check_rate(v: &Value) -> Result<(), String> {
    let t = text(v).unwrap_or_default();
    if t.is_empty() {
        return Ok(());
    }
    match t.parse::<u32>() {
        Ok(n) if (80..=400).contains(&n) => Ok(()),
        _ => Err("blank, or words a minute from 80 to 400".into()),
    }
}

/// The home field, as the station would build it: blank (no ATC), or an
/// ICAO code that X-Plane's apt.dat knows. An apt.dat that cannot be read
/// is no measurement: the code's shape is all that is checked then.
fn check_icao(v: &Value) -> Result<(), String> {
    let t = text(v).unwrap_or_default().to_ascii_uppercase();
    if t.is_empty() {
        return Ok(());
    }
    if !(3..=4).contains(&t.len()) || !t.chars().all(|c| c.is_ascii_alphanumeric()) {
        return Err("an ICAO code is 3 or 4 letters and digits (LXGB)".into());
    }
    let live_apt = live().apt_data_path.clone();
    let apt = if live_apt.is_empty() {
        crate::config::apt_dat_path()
    } else {
        live_apt
    };
    match crate::nav::find_airport(&apt, &t) {
        Ok(None) => Err(format!("{t} is not in X-Plane's airports ({apt})")),
        Ok(Some(_)) | Err(_) => Ok(()),
    }
}

/// A SimBrief id as `simbrief::fetch` asks for it: the numeric user id
/// (the API's `userid=`; a username is a different query it does not make).
fn check_simbrief(v: &Value) -> Result<(), String> {
    let t = text(v).unwrap_or_default();
    if t.chars().all(|c| c.is_ascii_digit()) {
        Ok(())
    } else {
        Err("the SimBrief id is the number on your SimBrief account page".into())
    }
}

/// A name the copilot can say: letters, spaces, hyphens, apostrophes.
fn check_name(v: &Value) -> Result<(), String> {
    let t = text(v).unwrap_or_default();
    if t.chars()
        .all(|c| c.is_alphabetic() || c == ' ' || c == '-' || c == '\'')
    {
        Ok(())
    } else {
        Err("a name is letters, spaces, hyphens and apostrophes".into())
    }
}

/// Blank, or a dataref name (`xcopilot/ptt_active`): no spaces.
fn check_dataref(v: &Value) -> Result<(), String> {
    let t = text(v).unwrap_or_default();
    if t.chars()
        .all(|c| c.is_ascii_alphanumeric() || matches!(c, '/' | '_' | '[' | ']' | '.' | '-'))
    {
        Ok(())
    } else {
        Err("a dataref name has no spaces (xcopilot/ptt_active)".into())
    }
}

/// Blank, or a key the push-to-talk listener knows (`xfc key-probe` names
/// the one you press).
fn check_ptt_key(v: &Value) -> Result<(), String> {
    let t = text(v).unwrap_or_default();
    if t.is_empty() || crate::audio::parse_ptt_key(&t).is_some() {
        Ok(())
    } else {
        Err(format!(
            "{t} is not a key name (`xfc key-probe` shows yours)"
        ))
    }
}

/// `auto` (detected from the loaded aircraft), or a profile id.
fn check_aircraft(v: &Value) -> Result<(), String> {
    let t = text(v).unwrap_or_default();
    if t.eq_ignore_ascii_case("auto") || crate::aircraft::load(&t).is_some() {
        Ok(())
    } else {
        Err(format!("{t} is not a profile: auto, or an id in profiles/"))
    }
}

/// The away-field pool: comma-separated voice specs (`x.onnx`, `x.onnx#N`),
/// or blank for the curated voices everywhere.
fn check_pool(v: &Value) -> Result<(), String> {
    let t = text(v).unwrap_or_default();
    for spec in t.split(',').map(str::trim).filter(|s| !s.is_empty()) {
        let (path, speaker) = match spec.rsplit_once('#') {
            Some((p, n)) => (p, Some(n)),
            None => (spec, None),
        };
        if !path.ends_with(".onnx") || speaker.is_some_and(|n| n.parse::<u32>().is_err()) {
            return Err(format!(
                "{spec} is not a voice (model.onnx or model.onnx#N)"
            ));
        }
    }
    Ok(())
}

/// The copilot's voice, live: the running arbiter swaps it before its next
/// line, as the panel's picker does. No crew running (the voices screen),
/// the pick is only saved.
fn apply_copilot_voice(v: &Value) -> bool {
    let Some(t) = text(v).filter(|t| !t.is_empty()) else {
        return false;
    };
    update(|c| c.piper_voice = t.clone());
    crate::prefs::save_pilot_voice(&t);
    if let Some(voice) = crate::voice::running() {
        voice.set_pilot_voice(t);
    }
    true
}

/// The cabin's voice, live, like the copilot's.
fn apply_cabin_voice(v: &Value) -> bool {
    let Some(t) = text(v).filter(|t| !t.is_empty()) else {
        return false;
    };
    update(|c| c.cabin_piper_voice = t.clone());
    crate::prefs::save_cabin_voice(&t);
    if let Some(voice) = crate::voice::running() {
        voice.set_cabin_voice(t);
    }
    true
}

/// Held by any test that touches process-wide state here: the live store
/// ([`install`], [`update`]) or an environment variable used to exercise the
/// pinning rule. Both are shared by every test thread — and `Config::from_env`
/// reads the environment — so an unguarded mutation races every other test in
/// the binary, the same lesson as the station's `SIDE_STATE_LOCK`. Tests must
/// also avoid [`set`] on an UNPINNED knob: it persists, and the test binary's
/// working directory is the repo, holding the real prefs.json.
#[cfg(test)]
pub static TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

#[cfg(test)]
mod tests {
    use super::*;

    /// Sample values a setting must be able to carry: the ends and the middle
    /// of a range, both bools, every choice.
    fn samples(kind: &Kind) -> Vec<Value> {
        match kind {
            Kind::Bool => vec![Value::Bool(true), Value::Bool(false)],
            Kind::Num {
                min, max, decimals, ..
            } => vec![
                Value::Num(round_to(*min, *decimals)),
                Value::Num(round_to((min + max) / 2.0, *decimals)),
                Value::Num(round_to(*max, *decimals)),
            ],
            Kind::Choice(options) => options
                .iter()
                .map(|o| Value::Choice((*o).to_string()))
                .collect(),
            Kind::Text { max_len } => vec![
                Value::Text(String::new()),
                Value::Text("WAT=Walker".chars().take(*max_len).collect()),
            ],
            Kind::Path { .. } => vec![Value::Text(String::new()), Value::Text("/tmp/x.wav".into())],
            Kind::Voice => vec![
                Value::Text(String::new()),
                Value::Text("models/en_GB-vctk-medium.onnx#4".into()),
            ],
        }
    }

    /// The registry is the schema, so its entries have to be self-consistent:
    /// every key unique, and every value the setting can legally hold surviving
    /// the trip out to prefs.json and back. A value that renders but does not
    /// re-parse is a setting that silently forgets itself at the next boot,
    /// which is worse than not being settable at all.
    #[test]
    fn every_setting_round_trips_through_its_own_kind() {
        let mut seen = std::collections::HashSet::new();
        for s in ALL {
            assert!(seen.insert(s.key), "duplicate setting key: {}", s.key);
            assert!(!s.label.is_empty() && !s.help.is_empty(), "{}", s.key);
            for v in samples(&s.kind) {
                let stored = s.kind.store(&v);
                assert_eq!(
                    s.kind.parse(&stored),
                    Some(v.clone()),
                    "{} does not round-trip through {stored:?}",
                    s.key
                );
                // Stepping from any legal value must land on another one.
                for up in [true, false] {
                    let next = s.kind.step(&v, up);
                    let stored = s.kind.store(&next);
                    assert_eq!(s.kind.parse(&stored), Some(next), "{} step", s.key);
                }
            }
        }
    }

    /// Whatever the live configuration holds, the screen can render it and
    /// stepping it lands back inside the declared range. This is the case the
    /// round-trip test cannot cover: an environment variable is parsed by
    /// `config.rs`, not by the registry, so it can legally sit outside the
    /// range the screen offers (`XCOPILOT_VAD_RMS=0.5`), and the first
    /// keypress must pull it in rather than reject or preserve it.
    #[test]
    fn a_value_from_outside_the_declared_range_is_pulled_in_by_a_step() {
        for s in ALL {
            let _ = s.rendered();
        }
        let kind = Kind::Num {
            min: 0.001,
            max: 0.1,
            step: 0.001,
            decimals: 3,
            unit: "",
        };
        let wild = Value::Num(0.5);
        assert_eq!(kind.step(&wild, true), Value::Num(0.1));
        assert_eq!(kind.step(&wild, false), Value::Num(0.1));
        assert_eq!(kind.step(&Value::Num(0.0), true), Value::Num(0.001));
    }

    /// Numbers clamp at their declared ends rather than walking past them.
    #[test]
    fn stepping_a_number_stops_at_its_limits() {
        let kind = Kind::Num {
            min: 0.0,
            max: 1.0,
            step: 0.25,
            decimals: 2,
            unit: "",
        };
        let mut v = Value::Num(0.9);
        for _ in 0..5 {
            v = kind.step(&v, true);
        }
        assert_eq!(v, Value::Num(1.0));
        for _ in 0..10 {
            v = kind.step(&v, false);
        }
        assert_eq!(v, Value::Num(0.0));
        // Out of range never parses, so a hand-edited prefs.json can't push a
        // value past what the code expects.
        assert_eq!(kind.parse("1.5"), None);
        assert_eq!(kind.parse("banana"), None);
    }

    /// A choice cycles both ways and refuses anything not on the list.
    #[test]
    fn a_choice_cycles_and_rejects_strangers() {
        let kind = Kind::Choice(&["both", "copilot", "atc"]);
        let v = Value::Choice("atc".into());
        assert_eq!(kind.step(&v, true), Value::Choice("both".into()));
        assert_eq!(kind.step(&v, false), Value::Choice("copilot".into()));
        assert_eq!(kind.parse("COPILOT"), Some(Value::Choice("copilot".into())));
        assert_eq!(kind.parse("tower"), None);
    }

    /// Bools accept every spelling the environment variables use, because a
    /// value saved from one surface is read back by the other.
    #[test]
    fn bools_read_the_same_spellings_the_env_vars_use() {
        for on in ["true", "on", "1", "yes", "TRUE"] {
            assert_eq!(Kind::Bool.parse(on), Some(Value::Bool(true)), "{on}");
        }
        for off in ["false", "off", "0", "no", "OFF"] {
            assert_eq!(Kind::Bool.parse(off), Some(Value::Bool(false)), "{off}");
        }
        assert_eq!(Kind::Bool.parse("maybe"), None);
    }

    /// The store must never carry a unit into prefs.json: a rendered "900 ms"
    /// that its own parser rejects would silently drop the saved value at the
    /// next boot.
    #[test]
    fn a_unit_is_shown_but_never_stored() {
        let kind = Kind::Num {
            min: 0.0,
            max: 3000.0,
            step: 50.0,
            decimals: 0,
            unit: "ms",
        };
        let v = Value::Num(900.0);
        assert_eq!(kind.render(&v), "900 ms");
        assert_eq!(kind.store(&v), "900");
        assert_eq!(kind.parse(&kind.store(&v)), Some(v));
    }

    /// An env-pinned setting is refused rather than silently overwritten at the
    /// next boot — the "always end up back with ryan" rule.
    #[test]
    fn an_env_pinned_setting_refuses_the_edit() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let s = ALL
            .iter()
            .find(|s| s.key == "taxi_guide")
            .expect("declared");
        // SAFETY: the lock above serialises every env-mutating test.
        unsafe { std::env::set_var(s.env, "true") };
        assert!(s.env_pinned());
        let err = set("taxi_guide", &Value::Bool(false)).expect_err("pinned");
        assert!(err.contains("XCOPILOT_TAXI_GUIDE"), "{err}");
        unsafe { std::env::remove_var(s.env) };
        assert!(!s.env_pinned());
    }

    /// #228: the "voices through X-Plane audio" pick was applied from the
    /// panel and never saved. It is a registry setting now: a saved pick is
    /// applied at startup, and the environment still pins it.
    #[test]
    fn the_audio_bus_pick_survives_a_restart() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let s = ALL.iter().find(|s| s.key == "audio_bus").expect("declared");
        unsafe { std::env::remove_var(s.env) };
        crate::tts::configure_audio_bus(false, false, "http://127.0.0.1:8086");
        assert!(apply_saved(s, Some("on")) || apply_saved(s, Some("true")), "a saved pick applies");
        assert_eq!(s.value(), Value::Bool(true));
        assert!(live().audio_bus);
        // Pinned by the environment: the saved pick and a new one are refused.
        // SAFETY: the lock above serialises every env-mutating test.
        unsafe { std::env::set_var(s.env, "false") };
        crate::tts::configure_audio_bus(false, true, "http://127.0.0.1:8086");
        assert!(!apply_saved(s, Some("on")));
        assert!(set("audio_bus", &Value::Bool(true)).is_err());
        assert_eq!(s.value(), Value::Bool(false));
        unsafe { std::env::remove_var(s.env) };
        crate::tts::configure_audio_bus(false, false, "http://127.0.0.1:8086");
    }

    fn text_setting(key: &str) -> &'static Setting {
        ALL.iter().find(|s| s.key == key).expect("declared")
    }

    /// A text knob is env > pref > default like every other: a saved value
    /// fills in only while its variable is silent, and with nothing saved
    /// the default (blank) stands.
    #[test]
    fn a_text_setting_is_env_then_pref_then_default() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let s = text_setting("callsign");
        // Default: nothing in the environment, nothing saved.
        unsafe { std::env::remove_var(s.env) };
        update(|c| c.callsign = String::new());
        assert!(!apply_saved(s, None));
        assert_eq!(s.value(), Value::Text(String::new()));
        assert_eq!(s.rendered(), "(none)");
        // A saved value fills the default in.
        assert!(apply_saved(s, Some("WAT2516")));
        assert_eq!(live().callsign, "WAT2516");
        // The environment (as `Config::from_env` read it) beats the saved one.
        // SAFETY: the lock above serialises every env-mutating test.
        unsafe { std::env::set_var(s.env, "BAW306") };
        update(|c| c.callsign = "BAW306".into());
        assert!(!apply_saved(s, Some("WAT2516")));
        assert_eq!(live().callsign, "BAW306");
        unsafe { std::env::remove_var(s.env) };

        let t = text_setting("airline_telephony");
        unsafe { std::env::remove_var(t.env) };
        update(|c| c.airline_telephony = String::new());
        // Saved junk keeps the default; a saved table is read back canonical.
        assert!(!apply_saved(t, Some("nonsense")));
        assert_eq!(live().airline_telephony, "");
        assert!(apply_saved(t, Some("wat=Walker, junk")));
        assert_eq!(t.value(), Value::Text("WAT=Walker".into()));
        unsafe { std::env::set_var(t.env, "XYZ=Zulu") };
        update(|c| c.airline_telephony = "XYZ=Zulu".into());
        assert!(!apply_saved(t, Some("WAT=Walker")));
        assert_eq!(t.value(), Value::Text("XYZ=Zulu".into()));
        unsafe { std::env::remove_var(t.env) };
        update(|c| {
            c.callsign = String::new();
            c.airline_telephony = String::new();
        });
    }

    /// An env-pinned text knob refuses the edit naming its variable, and
    /// the live value is untouched.
    #[test]
    fn an_env_pinned_text_setting_refuses_the_edit() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        for key in ["callsign", "airline_telephony"] {
            let s = text_setting(key);
            // SAFETY: the lock above serialises every env-mutating test.
            unsafe { std::env::set_var(s.env, "") };
            let before = s.value();
            let err = set(key, &Value::Text("WAT2516".into())).expect_err("pinned");
            assert!(err.contains(s.env), "{err}");
            assert!(
                !err.contains('\u{2014}') && !err.contains('\u{2013}'),
                "{err}"
            );
            assert_eq!(s.value(), before);
            unsafe { std::env::remove_var(s.env) };
        }
    }

    /// What a text knob accepts: one line within its length, dashes made
    /// plain; a callsign of letters, digits, spaces and hyphens; a telephony
    /// table with at least one readable entry (or blank, to clear it).
    #[test]
    fn text_values_are_checked_before_they_apply() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let kind = Kind::Text { max_len: 8 };
        assert_eq!(
            kind.parse("  G\u{2014}ABCD "),
            Some(Value::Text("G-ABCD".into()))
        );
        assert_eq!(kind.parse("123456789"), None);
        assert_eq!(kind.parse("a\tb"), None);
        assert_eq!(kind.store(&Value::Text(String::new())), "");

        let cs = text_setting("callsign");
        assert!((cs.apply)(&Value::Text("Walker  2516".into())));
        assert_eq!(live().callsign, "Walker 2516");
        assert!(!(cs.apply)(&Value::Text("WAT|2516".into())));
        assert!(!(cs.apply)(&Value::Text("--".into())));
        assert!(!(cs.apply)(&Value::Bool(true)));
        assert!((cs.apply)(&Value::Text(String::new())));
        assert_eq!(live().callsign, "");

        let tel = text_setting("airline_telephony");
        assert!(
            !(tel.apply)(&Value::Text("Walker".into())),
            "nothing readable"
        );
        assert!((tel.apply)(&Value::Text("wat=Walker;XYZ=Zulu Air".into())));
        assert_eq!(live().airline_telephony, "WAT=Walker; XYZ=Zulu Air");
        assert!((tel.apply)(&Value::Text(" ".into())), "blank clears");
        assert_eq!(live().airline_telephony, "");
    }

    /// Applying a value of the wrong shape changes nothing.
    #[test]
    fn a_mismatched_value_is_rejected_not_coerced() {
        let s = ALL
            .iter()
            .find(|s| s.key == "taxi_guide")
            .expect("declared");
        assert!(!(s.apply)(&Value::Num(1.0)));
        let s = ALL
            .iter()
            .find(|s| s.key == "voice_volume")
            .expect("declared");
        assert!(!(s.apply)(&Value::Bool(true)));
    }

    /// The live store is copy-on-write: a handle taken before a change keeps
    /// reading the value it started with, so one tick can't see half an edit.
    #[test]
    fn a_held_handle_finishes_its_tick_on_one_snapshot() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        update(|c| c.taxi_guide = true);
        let held = live();
        update(|c| c.taxi_guide = false);
        assert!(held.taxi_guide, "the held snapshot must not mutate");
        assert!(!live().taxi_guide, "a fresh handle sees the change");
    }

    /// Boot publishes the configuration where every consumer reads it, and
    /// hands back the same thing it published: a loop that re-reads the store
    /// per tick and a boot path that kept its own copy must not disagree.
    ///
    /// Asserted on fields the registry does NOT manage, deliberately. A
    /// registry-managed field is exactly the one `install` is entitled to
    /// overwrite from prefs.json, so asserting the value we passed in would
    /// make this test pass or fail on whatever the developer last clicked.
    #[test]
    fn install_publishes_the_configuration_it_returns() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let cfg = Config {
            atc_airport: "EGHI".into(),
            whisper_model_path: "models/ggml-tiny.en.bin".into(),
            ..Config::default()
        };
        let resolved = install(cfg);
        assert_eq!(resolved.atc_airport, "EGHI");
        let seen_by_a_loop = live();
        assert_eq!(seen_by_a_loop.atc_airport, "EGHI");
        assert_eq!(seen_by_a_loop.whisper_model_path, "models/ggml-tiny.en.bin");
        // ...and a subsequent change is visible to the next reader, which is
        // the whole point of the store.
        update(|c| c.atc_airport = "LXGB".into());
        assert_eq!(live().atc_airport, "LXGB");
        assert_eq!(
            resolved.atc_airport, "EGHI",
            "the boot snapshot is still its own"
        );
    }

    /// Environment variables the registry deliberately does NOT carry, each
    /// with why: developer and build knobs, and the two that decide where
    /// prefs.json itself is. Anything else the code reads is a setting.
    const ENV_ONLY: &[(&str, &str)] = &[
        ("XCOPILOT_BUILD", "a build stamp (env!), not a runtime knob"),
        (
            "XCOPILOT_BLACKBOX",
            "developer: tests and runs that must not record",
        ),
        (
            "XCOPILOT_DREF_CACHE_MS",
            "developer: the Web API read cache",
        ),
        ("XCOPILOT_AUDITION_DIR", "developer: voice audition output"),
        ("XCOPILOT_PLUGIN_STATUS", "developer: the panel mirror"),
        (
            "XCOPILOT_HOME",
            "decides the data folder, so where prefs.json itself lives",
        ),
        (
            "XCOPILOT_MONITOR_HZ",
            "developer: the monitor's sample rate",
        ),
        (
            "XCOPILOT_UI",
            "decided by the terminal (tui on a TTY, else plain)",
        ),
        (
            "XCOPILOT_LISTEN",
            "developer: continuous capture or Enter-to-talk",
        ),
        (
            "XCOPILOT_ATIS_FREQ",
            "read only by the old standalone `atc` dev binary; xfc takes ATIS from apt.dat",
        ),
    ];

    /// Every `"XCOPILOT_..."` literal in the workspace's Rust is a registered
    /// setting's variable or on [`ENV_ONLY`] with its reason: a new variable
    /// cannot slip in that the settings screens never show.
    #[test]
    fn every_environment_variable_is_a_setting_or_named_env_only() {
        let repo = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
        let mut roots = vec![repo.join("src")];
        for e in std::fs::read_dir(repo.join("crates")).unwrap().flatten() {
            roots.push(e.path().join("src"));
        }
        fn walk(dir: &std::path::Path, out: &mut Vec<std::path::PathBuf>) {
            for e in std::fs::read_dir(dir).into_iter().flatten().flatten() {
                let p = e.path();
                if p.is_dir() {
                    walk(&p, out);
                } else if p.extension().is_some_and(|x| x == "rs") {
                    out.push(p);
                }
            }
        }
        let mut files = Vec::new();
        for r in &roots {
            walk(r, &mut files);
        }
        let mut found = std::collections::BTreeSet::new();
        for f in &files {
            let text = std::fs::read_to_string(f).unwrap();
            for (i, _) in text.match_indices("\"XCOPILOT_") {
                let name: String = text[i + 1..]
                    .chars()
                    .take_while(|c| c.is_ascii_uppercase() || c.is_ascii_digit() || *c == '_')
                    .collect();
                // A whole literal only: `"XCOPILOT_ATC_"` prose or a format
                // prefix is not a variable.
                if text[i + 1 + name.len()..].starts_with('"') && !name.ends_with('_') {
                    found.insert(name);
                }
            }
        }
        assert!(found.len() > 40, "the scan found only {}", found.len());
        let stray: Vec<&String> = found
            .iter()
            .filter(|n| {
                !ALL.iter().any(|s| s.env == n.as_str()) && !ENV_ONLY.iter().any(|(e, _)| e == n)
            })
            .collect();
        assert!(
            stray.is_empty(),
            "variables neither registered (settings::ALL) nor named in ENV_ONLY: {stray:?}"
        );
        for (e, _) in ENV_ONLY {
            assert!(
                !ALL.iter().any(|s| s.env == *e),
                "{e} is both a setting and env-only"
            );
        }
    }

    /// The boot knobs and the registry's `Restart` entries are one list:
    /// every `Restart` knob's pref is read where boot reads it
    /// (`config::BOOT_KNOBS`, `config::knob`), and nothing there is a knob
    /// the screens cannot show. The order this pins, per knob class:
    /// `Restart` in `Config::from_env` (before the voices and home screens),
    /// `Live` + `Map` by `install`, `Own` on its own field's boot path.
    #[test]
    fn every_restart_knob_is_resolved_where_boot_reads_it() {
        for s in ALL.iter().filter(|s| s.tier == Tier::Restart) {
            assert_eq!(s.persist, Persist::Map, "{}", s.key);
            assert!(
                crate::config::BOOT_KNOBS.contains(&(s.env, s.key)),
                "{} ({}) is Restart but boot never reads its pref",
                s.key,
                s.env
            );
        }
        for (env, key) in crate::config::BOOT_KNOBS {
            assert!(
                ALL.iter()
                    .any(|s| s.key == *key && s.env == *env && s.tier == Tier::Restart),
                "{env} -> {key} is read at boot but is no Restart setting"
            );
        }
        // Every section has something in it, and a knob is in exactly one.
        for sec in Section::ALL {
            assert!(
                ALL.iter().any(|s| s.section == sec),
                "{} is empty",
                sec.title()
            );
        }
        assert_eq!(by_section().len(), ALL.len());
    }

    /// A path is refused when it is not there (said, named) and accepted
    /// blank (the default); a voice only when installed, naming `xfc voices`.
    #[test]
    fn a_path_or_voice_is_checked_against_the_disk() {
        let dir = std::env::temp_dir().join(format!("xfc-kind-check-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("c.wav");
        std::fs::write(&file, b"x").unwrap();
        let t = |p: &std::path::Path| Value::Text(p.to_string_lossy().into_owned());
        let f = Kind::Path { dir: false };
        let d = Kind::Path { dir: true };
        assert!(f.validate(&t(&file)).is_ok());
        assert!(f.validate(&Value::Text(String::new())).is_ok());
        assert!(f
            .validate(&t(&dir))
            .unwrap_err()
            .contains("a folder, not a file"));
        assert!(d.validate(&t(&dir)).is_ok());
        let gone = dir.join("nope");
        assert!(d.validate(&t(&gone)).unwrap_err().contains("no folder at"));
        let voice = dir.join("en_GB-test-medium.onnx");
        std::fs::write(&voice, b"x").unwrap();
        let spec = format!("{}#3", voice.display());
        let err = Kind::Voice
            .validate(&Value::Text(spec.clone()))
            .unwrap_err();
        assert!(
            err.contains("en_GB-test-medium#3") && err.contains("xfc voices"),
            "{err}"
        );
        std::fs::write(dir.join("en_GB-test-medium.onnx.json"), b"{}").unwrap();
        assert!(Kind::Voice.validate(&Value::Text(spec)).is_ok());
        assert_eq!(
            Kind::Voice.render(&Value::Text(format!("{}#3", voice.display()))),
            "en_GB-test-medium#3"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// Which phase of the restart round trip a child run is (`set` or
    /// `check`); unset in a normal test run.
    const RESTART_PHASE: &str = "XFC_SETTINGS_RESTART_PHASE";

    /// The value the round trip sets for each knob: a legal value that is
    /// NOT what a fresh data folder holds, so surviving the restart proves
    /// the pref was read. Bools flip, numbers step, choices move on; text,
    /// paths and voices from the table (every such knob must be in it).
    fn restart_sample(s: &Setting, home: &std::path::Path) -> Value {
        let p = |rel: &str| home.join(rel).to_string_lossy().into_owned();
        let current = s.value();
        match s.kind {
            Kind::Bool => Value::Bool(!current.as_bool().unwrap_or(false)),
            Kind::Num {
                min,
                max,
                step,
                decimals,
                ..
            } => {
                let cur = current.as_num().unwrap_or(min);
                let a = round_to(min + step, decimals);
                let b = round_to((min + 2.0 * step).min(max), decimals);
                Value::Num(if (cur - a).abs() < 1e-9 { b } else { a })
            }
            Kind::Choice(options) => {
                let cur = current.as_choice().unwrap_or("").to_string();
                let other = options.iter().find(|o| **o != cur).expect("two options");
                Value::Choice((*other).to_string())
            }
            Kind::Text { .. } | Kind::Path { .. } | Kind::Voice => Value::Text(match s.key {
                "traffic_file" => "my-traffic.yaml".into(),
                "callsign" => "BAW306".into(),
                "airline_telephony" => "WAT=Walker".into(),
                "efb_token" => "abcd1234efgh".into(),
                "copilot_voice" => p("models/v_copilot.onnx"),
                "cabin_voice" => p("models/v_cabin.onnx"),
                "atc_tower_voice" => p("models/v_tower.onnx"),
                "atc_approach_voice" => p("models/v_approach.onnx"),
                "atc_departure_voice" => p("models/v_departure.onnx"),
                "atc_atis_voice" => p("models/v_atis.onnx#2"),
                "atc_voice_pool" => format!(
                    "{},{}",
                    p("models/v_pool.onnx#1"),
                    p("models/v_pool.onnx#5")
                ),
                "atc_tts_length" => "1.2".into(),
                "atc_tts_noise_w" => "0.9".into(),
                "atc_airport" => "EGHI".into(),
                "ptt_dataref" => "myplugin/ptt".into(),
                "radio_ptt_dataref" => "myplugin/radio_ptt".into(),
                "ptt_key" => "f8".into(),
                "chime" => p("c.wav"),
                "negative_tone" => p("n.wav"),
                "squelch_wav" => p("s.wav"),
                "simbrief_id" => "123456".into(),
                "pilot_name" => "Sam".into(),
                "copilot_name" => "Alex".into(),
                "aircraft" => "cessna_172sp".into(),
                "xplane_root" => p("xp"),
                "apt_dat" => p("apt.dat"),
                "models_dir" => p("models2"),
                "whisper_model" => p("w.bin"),
                "tts_length" => "0.9".into(),
                "tts_noise" => "0.5".into(),
                "tts_noise_w" => "0.7".into(),
                "say_voice" => "Karen".into(),
                "say_rate" => "180".into(),
                "atc_say_voice" => "Moira".into(),
                "atc_say_rate" => "170".into(),
                other => panic!("{other}: add a restart sample for this knob"),
            }),
        }
    }

    /// The files the round trip's values point at, in a fresh data folder.
    fn restart_home() -> std::path::PathBuf {
        let home =
            std::env::temp_dir().join(format!("xfc-settings-restart-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&home);
        for d in ["models", "models2", "xp"] {
            std::fs::create_dir_all(home.join(d)).unwrap();
        }
        for v in [
            "copilot",
            "cabin",
            "tower",
            "approach",
            "departure",
            "atis",
            "pool",
        ] {
            std::fs::write(home.join(format!("models/v_{v}.onnx")), b"x").unwrap();
            std::fs::write(home.join(format!("models/v_{v}.onnx.json")), b"{}").unwrap();
        }
        for f in ["c.wav", "n.wav", "s.wav", "w.bin"] {
            std::fs::write(home.join(f), b"x").unwrap();
        }
        std::fs::write(
            home.join("apt.dat"),
            "I\n1000 Version\n\n1 44 0 0 EGHI Southampton\n99\n",
        )
        .unwrap();
        home
    }

    /// Run this test binary's `restart_child` in `phase`, in a process whose
    /// data folder is `home` and whose environment pins nothing.
    fn run_restart_child(phase: &str, home: &std::path::Path) {
        let mut cmd = std::process::Command::new(std::env::current_exe().unwrap());
        cmd.args([
            "--exact",
            "settings::tests::restart_child",
            "--ignored",
            "--test-threads=1",
            "--nocapture",
        ])
        .current_dir(home)
        .env(RESTART_PHASE, phase)
        .env("XCOPILOT_HOME", home)
        .env("XCOPILOT_BLACKBOX", "off")
        // The station's airports file, so the home field's check reads ours
        // (a non-registry variable: `Config::apt_data_path`).
        .env("XPLANE_APT_DATA", home.join("apt.dat"));
        for s in ALL {
            cmd.env_remove(s.env);
        }
        for (env, _) in crate::config::BOOT_KNOBS {
            cmd.env_remove(env);
        }
        let out = cmd.output().expect("the child runs");
        let text = format!(
            "{}{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        );
        assert!(
            out.status.success() && text.contains("1 passed"),
            "restart child ({phase}) failed:\n{text}"
        );
    }

    /// Every registered setting round-trips through a restart: set through
    /// the registry in one process, then read back by a NEW process's
    /// `Config::from_env` + `settings::install` (+ the mode and staffing
    /// boot), in a temp `XCOPILOT_HOME` with nothing in the environment.
    /// Two child processes because `sim::paths` decides the data folder
    /// once per process.
    #[test]
    fn every_setting_survives_a_restart() {
        if std::env::var(RESTART_PHASE).is_ok() {
            return; // inside a child: only `restart_child` runs there
        }
        let home = restart_home();
        run_restart_child("set", &home);
        // The first process wrote the temp folder's prefs.json, nobody else's.
        let prefs = std::fs::read_to_string(home.join("prefs.json")).expect("prefs written");
        assert!(prefs.contains("\"atc_airport\": \"EGHI\""), "{prefs}");
        assert!(prefs.contains("v_copilot.onnx"), "{prefs}");
        run_restart_child("check", &home);
        let _ = std::fs::remove_dir_all(&home);
    }

    /// One phase of [`every_setting_survives_a_restart`], run only as its
    /// child (ignored otherwise: it writes the data folder's prefs.json).
    #[test]
    #[ignore = "a child process of every_setting_survives_a_restart"]
    fn restart_child() {
        let Ok(phase) = std::env::var(RESTART_PHASE) else {
            return;
        };
        let home = std::path::PathBuf::from(std::env::var("XCOPILOT_HOME").unwrap());
        assert_eq!(
            sim::paths::data(),
            home.as_path(),
            "the data folder is the temp home"
        );
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let config = install(crate::config::Config::from_env().expect("config"));
        crate::boot_mode_and_staffing(&config);
        let expected: Vec<(&str, Value)> = {
            // The samples are taken against a FRESH folder's values, so the
            // set phase computes them and the check phase reads them back.
            let file = home.join("expected.json");
            if phase == "set" {
                let pairs: Vec<(&str, Value)> = ALL
                    .iter()
                    .map(|s| (s.key, restart_sample(s, &home)))
                    .collect();
                let stored: std::collections::BTreeMap<&str, String> = pairs
                    .iter()
                    .map(|(k, v)| {
                        let s = ALL.iter().find(|s| s.key == *k).unwrap();
                        (*k, s.kind.store(v))
                    })
                    .collect();
                std::fs::write(&file, serde_json::to_string(&stored).unwrap()).unwrap();
                pairs
            } else {
                let stored: std::collections::BTreeMap<String, String> =
                    serde_json::from_str(&std::fs::read_to_string(&file).unwrap()).unwrap();
                ALL.iter()
                    .map(|s| (s.key, s.kind.parse(&stored[s.key]).expect("stored parses")))
                    .collect()
            }
        };
        let mut wrong = Vec::new();
        for (key, v) in &expected {
            let s = ALL.iter().find(|s| s.key == *key).unwrap();
            if phase == "set" {
                if let Err(e) = set(key, v) {
                    wrong.push(format!("{key}: set refused {v:?}: {e}"));
                }
            } else if s.rendered() != s.kind.render(v) {
                wrong.push(format!(
                    "{key}: after a restart {}, set {}",
                    s.rendered(),
                    s.kind.render(v)
                ));
            }
        }
        assert!(wrong.is_empty(), "{phase}:\n{}", wrong.join("\n"));
    }
}
