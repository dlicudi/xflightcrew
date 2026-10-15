//! The setup checks: what the one app (`src/app.rs`) shows on its Checks
//! tab and waits on before the crew boots, and `xfc home --print`.
//!
//! ```text
//! xfc                      the app; the crew boots at once when every
//!                          REQUIRED check passes (X-Plane folder, plugin
//!                          missing, starter set incomplete), else the app
//!                          opens on Checks and boots the moment they pass
//! xfc home | xfc setup     the app, opened on Checks (hidden aliases)
//! xfc home --print         the checks as text (for support, and no TTY)
//! XCOPILOT_UI=plain xfc    the crew, headless (also plain `xfc` with no TTY)
//! ```
//!
//! The boot's decision is offline and cheap ([`offline`]: file checks only).
//! The Checks tab adds the two live facts when it opens or on `c`, never
//! polled: ONE Web API GET ([`ask_web_api`]) and the default microphone's
//! NAME ([`mic_name`], nothing recorded). Every verdict is pure ([`checks`]);
//! the tab is [`screen`].

pub mod screen;

use std::path::{Path, PathBuf};
use std::time::Duration;

use anyhow::{anyhow, Result};

use crate::config::Config;
use crate::plugin_install::{self, Verdict};
use crate::voices::catalogue::{self, Kind, StarterStatus};

/// What the checks are judged from: gathered by the shell, judged pure.
#[derive(Clone, Debug)]
pub struct Facts {
    pub xplane: PathBuf,
    /// The folder holds `Resources/plugins`.
    pub xplane_found: bool,
    pub plugin: Verdict,
    /// The installed plugin's version, when one could be read.
    pub plugin_installed: Option<String>,
    /// The version this build carries, or `None` (no plugin carried).
    pub plugin_carried: Option<String>,
    /// How many starter items are not on disk (file checks).
    pub starter_missing: usize,
    /// The same, priced against the catalogue once it is to hand.
    pub starter: Option<StarterStatus>,
    /// The Web API's answer: `None` until asked.
    pub web_api: Option<Result<(), String>>,
    /// The port the Web API is asked on.
    pub api_port: String,
    /// The default input device: `None` until looked, `Some(None)` none.
    pub mic: Option<Option<String>>,
}

/// A check's mark.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mark {
    Ok,
    Bad,
    Warn,
    /// Not asked yet.
    Wait,
}

impl Mark {
    pub fn symbol(self) -> &'static str {
        match self {
            Mark::Ok => "✓",
            Mark::Bad => "✗",
            Mark::Warn => "!",
            Mark::Wait => "·",
        }
    }
}

/// What enter does on a check.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fix {
    None,
    /// Say how to point xfc at the X-Plane folder.
    XPlaneHint,
    /// Ask the Web API again.
    Recheck,
    /// Install the plugin this build carries.
    InstallPlugin,
    /// Say how to build and install a plugin (this build carries none).
    PluginHint,
    /// Say how to watch the mic level.
    MicHint,
    /// Install the starter set, progress on the screen.
    InstallStarter,
}

/// One check's line.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Check {
    pub mark: Mark,
    pub text: String,
    pub fix: Fix,
    /// What the fix key says on the right ("enter: update"); empty for none.
    pub hint: &'static str,
    /// Red AND one the crew cannot start well without: plain `xfc` shows
    /// the screen for it.
    pub required: bool,
}

/// `~/X-Plane 12` for `$HOME/X-Plane 12`.
pub fn tilde(p: &Path) -> String {
    let shown = sim::paths::shown(p);
    match std::env::var("HOME") {
        Ok(home) if !home.is_empty() && shown.starts_with(&home) => {
            format!("~{}", &shown[home.len()..])
        }
        _ => shown,
    }
}

fn xplane_check(f: &Facts) -> Check {
    if f.xplane_found {
        Check {
            mark: Mark::Ok,
            text: format!("X-Plane found  {}", tilde(&f.xplane)),
            fix: Fix::None,
            hint: "",
            required: false,
        }
    } else {
        Check {
            mark: Mark::Bad,
            text: format!("X-Plane not found at {}", tilde(&f.xplane)),
            fix: Fix::XPlaneHint,
            hint: "enter: how to set it",
            required: true,
        }
    }
}

fn web_api_check(f: &Facts) -> Check {
    match &f.web_api {
        None => Check {
            mark: Mark::Wait,
            text: format!("Web API on {}: asking", f.api_port),
            fix: Fix::Recheck,
            hint: "",
            required: false,
        },
        Some(Ok(())) => Check {
            mark: Mark::Ok,
            text: format!("Web API on {}", f.api_port),
            fix: Fix::Recheck,
            hint: "",
            required: false,
        },
        // Not required: the crew waits for the sim on its own.
        Some(Err(_)) => Check {
            mark: Mark::Bad,
            text: "X-Plane not running, or the Web API is off (Settings, Network)".into(),
            fix: Fix::Recheck,
            hint: "enter: ask again",
            required: false,
        },
    }
}

fn plugin_check(f: &Facts) -> Check {
    let carried = f.plugin_carried.as_deref().unwrap_or("?");
    let installed = f.plugin_installed.as_deref().unwrap_or("?");
    let can_install = f.plugin_carried.is_some();
    let (install_fix, install_hint) = if can_install {
        (Fix::InstallPlugin, "enter: install")
    } else {
        (Fix::PluginHint, "enter: how")
    };
    let (mark, text, fix, hint, required) = if !f.xplane_found {
        (
            Mark::Bad,
            "Plugin: no X-Plane folder to look in".to_string(),
            Fix::None,
            "",
            false,
        )
    } else {
        match f.plugin {
            Verdict::Missing => (
                Mark::Bad,
                "Plugin not installed".into(),
                install_fix,
                install_hint,
                true,
            ),
            Verdict::UpToDate => (
                Mark::Ok,
                format!("Plugin {installed}"),
                Fix::None,
                "",
                false,
            ),
            Verdict::NoReference => (
                Mark::Ok,
                format!("Plugin {installed} (this build carries none to compare)"),
                Fix::None,
                "",
                false,
            ),
            Verdict::Older => (
                Mark::Bad,
                format!("Plugin older than this build ({installed})"),
                Fix::InstallPlugin,
                "enter: update",
                false,
            ),
            Verdict::Newer => (
                Mark::Warn,
                format!("Plugin newer than this build ({carried}): {installed}"),
                Fix::None,
                "",
                false,
            ),
            Verdict::OtherBuild => (
                Mark::Warn,
                format!("Plugin another build of {installed}"),
                Fix::InstallPlugin,
                "enter: replace",
                false,
            ),
            Verdict::Unknown => (
                Mark::Warn,
                "Plugin installed, its version unreadable".into(),
                Fix::InstallPlugin,
                "enter: replace",
                false,
            ),
        }
    };
    Check {
        mark,
        text,
        fix,
        hint,
        required,
    }
}

fn mic_check(f: &Facts) -> Check {
    match &f.mic {
        None => Check {
            mark: Mark::Wait,
            text: "Microphone: looking".into(),
            fix: Fix::MicHint,
            hint: "",
            required: false,
        },
        Some(Some(name)) => Check {
            mark: Mark::Ok,
            text: format!("Microphone  {name}"),
            fix: Fix::MicHint,
            hint: "",
            required: false,
        },
        Some(None) => Check {
            mark: Mark::Bad,
            text: "Microphone: no input device".into(),
            fix: Fix::MicHint,
            hint: "enter: how to check",
            required: false,
        },
    }
}

fn starter_check(f: &Facts) -> Check {
    if f.starter_missing == 0 {
        return Check {
            mark: Mark::Ok,
            text: "Starter voices: complete".into(),
            fix: Fix::None,
            hint: "",
            required: false,
        };
    }
    let size = match &f.starter {
        Some(s) if s.unknown.len() < s.missing.len() => {
            format!(", {}", catalogue::human_size(s.bytes))
        }
        _ => String::new(),
    };
    Check {
        mark: Mark::Warn,
        text: format!("Starter voices: {} missing{size}", f.starter_missing),
        fix: Fix::InstallStarter,
        hint: "enter: get them",
        required: true,
    }
}

/// The setup checks, in the screen's order.
pub fn checks(f: &Facts) -> Vec<Check> {
    vec![
        xplane_check(f),
        web_api_check(f),
        plugin_check(f),
        mic_check(f),
        starter_check(f),
    ]
}

/// The crew waits for these (the app opens on Checks for them).
pub fn needs_home(f: &Facts) -> bool {
    checks(f).iter().any(|c| c.required)
}

/// The port the Web API listens on, from the base URL.
pub fn api_port(base_url: &str) -> String {
    let rest = base_url.split("://").nth(1).unwrap_or(base_url);
    let host = rest.split('/').next().unwrap_or(rest);
    match host.rsplit_once(':') {
        Some((_, port)) if !port.is_empty() => port.to_string(),
        _ => "80".into(),
    }
}

/// The offline facts: file checks only, no network, no sim, no mic.
pub fn offline(config: &Config) -> Facts {
    let xplane = PathBuf::from(crate::config::xplane_root());
    let carried = plugin_install::embedded();
    let status = plugin_install::status(&xplane, carried);
    Facts {
        xplane_found: status.is_xplane,
        plugin: status.verdict,
        plugin_installed: status
            .installed
            .as_ref()
            .and_then(|i| i.as_ref().map(|id| id.version.clone())),
        plugin_carried: status.carried.as_ref().map(|id| id.version.clone()),
        xplane,
        starter_missing: crate::voices::starter_missing(config).len(),
        starter: None,
        web_api: None,
        api_port: api_port(&config.xplane_base_url),
        mic: None,
    }
}

/// ONE GET to the Web API, a short timeout: any HTTP answer means it is up.
pub fn ask_web_api(base_url: &str) -> Result<(), String> {
    let url = format!("{}/api/capabilities", base_url.trim_end_matches('/'));
    let client = reqwest::blocking::Client::builder()
        .timeout(Duration::from_millis(1500))
        .build()
        .map_err(|e| e.to_string())?;
    client
        .get(&url)
        .send()
        .map(|_| ())
        .map_err(|e| e.to_string())
}

/// The default input device's name: no stream is opened, nothing recorded.
pub fn mic_name() -> Option<String> {
    use cpal::traits::{DeviceTrait, HostTrait};
    cpal::default_host()
        .default_input_device()
        .and_then(|d| d.name().ok())
}

/// The lines under the checks.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Summary {
    pub version: String,
    pub voices: String,
    pub settings: String,
    pub files: String,
}

/// The models on disk: how many, how big (the folder only, offline).
pub fn voices_line() -> String {
    let disk = crate::voices::download::list_dir(&crate::config::models_dir());
    let rows: Vec<_> = [Kind::Piper, Kind::Whisper]
        .into_iter()
        .flat_map(|k| catalogue::join(k, &[], &disk, &[]))
        .collect();
    let bytes: u64 = rows.iter().map(|r| r.disk_bytes).sum();
    format!(
        "{} installed · {}",
        rows.len(),
        catalogue::human_size(bytes)
    )
}

/// The settings a person looks for first.
pub fn settings_line() -> String {
    let live = crate::settings::live();
    format!(
        "mode {} · listen {} · EFB {}",
        crate::mode::get().as_str(),
        live.listen_mode,
        if live.efb { "on" } else { "off" }
    )
}

pub fn summary() -> Summary {
    Summary {
        version: format!("{} · {}", env!("XVL_VERSION"), env!("XCOPILOT_BUILD")),
        voices: voices_line(),
        settings: settings_line(),
        files: format!("data {}", tilde(&sim::paths::roots().data)),
    }
}

/// What enter says for the hint fixes.
pub fn hint_text(fix: Fix) -> String {
    match fix {
        Fix::XPlaneHint => format!(
            "set the X-Plane folder on the Paths tab, or XCOPILOT_XPLANE_ROOT=<your X-Plane 12 folder> in {}; then c to check again",
            tilde(&sim::paths::env_file())
        ),
        Fix::PluginHint => "this build carries no plugin: from a source checkout, make -C plugin install; or a release build (scripts/build-release.sh) installs it from here".into(),
        Fix::MicHint => "xfc mic-check shows the live level (Ctrl-C ends it); macOS asks for microphone access the first time".into(),
        _ => String::new(),
    }
}

/// A fix as `--print` says it: the command, or the hint.
pub fn print_fix(fix: Fix) -> Option<String> {
    match fix {
        Fix::None | Fix::Recheck => None,
        Fix::InstallPlugin => Some("xfc plugin install".into()),
        Fix::InstallStarter => Some("xfc voices install --starter".into()),
        Fix::XPlaneHint | Fix::PluginHint | Fix::MicHint => Some(hint_text(fix)),
    }
}

/// Settings and the mode as boot has them: the registry with its saved
/// picks, the mode and the cabin staffing (env > prefs > default). Run
/// before the app opens so its settings tabs show what the crew will use.
pub fn prepare_settings(config: &Config) {
    let config = (*crate::settings::install(config.clone())).clone();
    crate::boot_mode_and_staffing(&config);
}

/// `xfc home --print`: the checks as text, the live two included.
pub fn print_report(config: &Config) -> String {
    prepare_settings(config);
    let mut f = offline(config);
    f.web_api = Some(ask_web_api(&config.xplane_base_url));
    f.mic = Some(mic_name());
    let store = crate::voices::Store::load(config);
    f.starter = Some(store.starter_status());
    let s = summary();
    let mut out = format!("XFlightCrew {}\n", s.version);
    out += "Setup\n";
    for c in checks(&f) {
        out += &format!("  {} {}\n", c.mark.symbol(), c.text);
        if c.mark != Mark::Ok {
            if let Some(fix) = print_fix(c.fix) {
                out += &format!("      fix: {fix}\n");
            }
        }
    }
    if let Some(note) = &store.note {
        out += &format!("  ({note})\n");
    }
    out += &format!("Voices    {}\n", s.voices);
    out += &format!("Settings  {}\n", s.settings);
    out += "Files\n";
    for line in crate::paths_report().lines() {
        out += &format!("  {line}\n");
    }
    out
}

/// `xfc home --print`: the checks as text, on its own thread (the
/// catalogue is blocking HTTP, never inside the async runtime).
pub fn print_cmd(config: &Config) -> Result<()> {
    let config = config.clone();
    let text = std::thread::spawn(move || print_report(&config))
        .join()
        .map_err(|_| anyhow!("xfc home --print panicked"))?;
    print!("{text}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    pub(super) fn green() -> Facts {
        Facts {
            xplane: PathBuf::from("/Users/pilot/X-Plane 12"),
            xplane_found: true,
            plugin: Verdict::UpToDate,
            plugin_installed: Some("2026.10.14".into()),
            plugin_carried: Some("2026.10.14".into()),
            starter_missing: 0,
            starter: None,
            web_api: Some(Ok(())),
            api_port: "8086".into(),
            mic: Some(Some("MacBook Pro Microphone".into())),
        }
    }

    #[test]
    fn all_green_needs_no_screen() {
        let f = green();
        assert!(
            checks(&f).iter().all(|c| c.mark == Mark::Ok),
            "{:?}",
            checks(&f)
        );
        assert!(!needs_home(&f));
    }

    #[test]
    fn the_required_checks_are_the_folder_the_plugin_and_the_starter() {
        let f = Facts {
            xplane_found: false,
            ..green()
        };
        assert!(needs_home(&f));
        assert_eq!(checks(&f)[0].fix, Fix::XPlaneHint);
        let f = Facts {
            plugin: Verdict::Missing,
            ..green()
        };
        assert!(needs_home(&f));
        assert_eq!(checks(&f)[2].fix, Fix::InstallPlugin);
        // Without a carried plugin, the fix says how.
        let f = Facts {
            plugin: Verdict::Missing,
            plugin_carried: None,
            ..green()
        };
        assert_eq!(checks(&f)[2].fix, Fix::PluginHint);
        let f = Facts {
            starter_missing: 2,
            ..green()
        };
        assert!(needs_home(&f));
        assert_eq!(checks(&f)[4].text, "Starter voices: 2 missing");
        // An older plugin is red but not required: the boot says it.
        let f = Facts {
            plugin: Verdict::Older,
            plugin_installed: Some("2026.10.13".into()),
            ..green()
        };
        assert_eq!(checks(&f)[2].mark, Mark::Bad);
        assert!(!needs_home(&f));
        // The Web API down is never required: the crew waits for the sim.
        let f = Facts {
            web_api: Some(Err("refused".into())),
            mic: Some(None),
            ..green()
        };
        assert!(checks(&f)[1].text.contains("Web API is off"));
        assert!(!needs_home(&f));
        // Not asked yet is not red.
        let f = Facts {
            web_api: None,
            mic: None,
            ..green()
        };
        assert_eq!(checks(&f)[1].mark, Mark::Wait);
        assert!(!needs_home(&f));
    }

    #[test]
    fn the_starter_line_is_priced_once_the_catalogue_is_here() {
        let f = Facts {
            starter_missing: 2,
            starter: Some(StarterStatus {
                missing: vec![
                    catalogue::StarterItem {
                        kind: Kind::Piper,
                        key: "a".into(),
                        spec: String::new(),
                    },
                    catalogue::StarterItem {
                        kind: Kind::Piper,
                        key: "b".into(),
                        spec: String::new(),
                    },
                ],
                bytes: 180_000_000,
                unknown: Vec::new(),
            }),
            ..green()
        };
        assert_eq!(checks(&f)[4].text, "Starter voices: 2 missing, 180 MB");
    }

    #[test]
    fn the_port_comes_from_the_base_url() {
        assert_eq!(api_port("http://127.0.0.1:8086"), "8086");
        assert_eq!(api_port("http://127.0.0.1:8086/"), "8086");
        assert_eq!(api_port("http://localhost"), "80");
    }
}
