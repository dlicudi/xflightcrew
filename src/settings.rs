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
        if !matches!(s.persist, Persist::Map) || s.env_pinned() {
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
            Kind::Text { max_len } => {
                // One line, no control characters, and no em or en dash (the
                // panel font and the TTS choke on them): a pasted one becomes
                // a hyphen rather than a refusal.
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
                (text.chars().count() <= *max_len && !text.chars().any(char::is_control))
                    .then_some(Value::Text(text))
            }
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
            // A mismatched pair can only come from a bad registry entry.
            (_, other) => format!("{other:?}"),
        }
    }

    /// The value as prefs.json stores it: no unit suffix, so it round-trips
    /// through [`Kind::parse`].
    fn store(&self, v: &Value) -> String {
        match (self, v) {
            (Kind::Num { decimals, .. }, Value::Num(n)) => format!("{n:.*}", *decimals),
            // Raw, never the "(none)" the screen shows for an empty one.
            (Kind::Text { .. }, Value::Text(s)) => s.clone(),
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

fn round_to(v: f64, decimals: usize) -> f64 {
    let f = 10f64.powi(decimals as i32);
    (v * f).round() / f
}

/// Where a setting's chosen value is remembered.
#[derive(Clone, Copy, PartialEq)]
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
#[derive(Clone, Copy, PartialEq)]
pub enum Tier {
    /// Applies to the next tick, utterance, or transmission.
    Live,
}

impl Tier {
    pub fn describe(self) -> &'static str {
        match self {
            Tier::Live => "applies immediately",
        }
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
    /// The value in force right now, read from wherever the truth lives.
    pub get: fn() -> Value,
    /// Apply it. `false` = rejected (wrong shape or out of range), in which
    /// case nothing has changed and nothing is persisted.
    pub apply: fn(&Value) -> bool,
}

impl Setting {
    /// Is this knob pinned by an explicit environment variable (including a
    /// line in `.env`, which dotenvy has already loaded into the environment)?
    pub fn env_pinned(&self) -> bool {
        !self.env.is_empty() && std::env::var(self.env).is_ok()
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
    if !(s.apply)(v) {
        return Err(format!("{} did not accept that value", s.label));
    }
    if s.persist == Persist::Map {
        // Text is stored as it was READ BACK: a telephony table keeps only
        // the entries that parsed, so prefs.json and the panel show what is
        // in force, not what was typed.
        let stored = match s.kind {
            Kind::Text { .. } => s.kind.store(&s.value()),
            _ => s.kind.store(v),
        };
        crate::prefs::save_setting(s.key, &stored);
    }
    tracing::info!(key = s.key, value = %s.kind.render(v), "setting changed");
    Ok(())
}

/// Every declared setting, in the order the screen shows them.
pub const ALL: &[Setting] = &[
    Setting {
        key: "mode",
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
    },
    Setting {
        key: "cabin_crew",
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
    },
    Setting {
        key: "voice_volume",
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
    },
    Setting {
        key: "atc_volume",
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
    },
    Setting {
        key: "vad_threshold",
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
    },
    Setting {
        key: "end_silence_ms",
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
    },
    Setting {
        key: "mic_gate_tail_ms",
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
    },
    Setting {
        key: "stt_min_confidence",
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
    },
    Setting {
        key: "stt_confirm_floor",
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
    },
    Setting {
        key: "confirm_critical",
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
    },
    Setting {
        key: "taxi_guide",
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
    },
    Setting {
        key: "atc_readback",
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
    },
    Setting {
        key: "atc_chatter",
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
    },
    Setting {
        key: "crew_chatter",
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
    },
    Setting {
        key: "start_calls",
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
    },
    Setting {
        key: "crew_dialect",
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
    },
    Setting {
        key: "callsign",
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
    },
    Setting {
        key: "airline_telephony",
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
    },
    Setting {
        key: "audio_bus",
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
    },
    Setting {
        key: "efb",
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
    },
    Setting {
        key: "efb_port",
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
    },
    Setting {
        key: "efb_token",
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
            Kind::Text { .. } => vec![Value::Text(String::new()), Value::Text("WAT=Walker".into())],
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
}
