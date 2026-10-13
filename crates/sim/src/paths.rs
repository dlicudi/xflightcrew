//! Where every file lives: the ONE owner of the two roots (2026-10-13,
//! docs/tasks/data-folder/BRIEF.md). Nothing else names a top-level folder;
//! pinned by `every_file_root_goes_through_the_owner` below.
//!
//! - [`resources`]: what ships, read only: `profiles/` (the YAMLs),
//!   `checklists/`, `assets/`.
//! - [`data`]: the user's, writable: `models/`, `logs/` (`sorties/`,
//!   `blackbox.jsonl`), `prefs.json`, `.env`, `profiles/learned/`.
//!
//! Decided ONCE, first match wins ([`resolve`]):
//! 1. **Override**: `XCOPILOT_HOME=<dir>`, both roots that dir.
//! 2. **Dev checkout**: the working directory holds `profiles/` and
//!    `Cargo.toml` (both roots the working directory, kept RELATIVE so every
//!    path is the same string it always was), or the binary's location
//!    walks up to the `xflightcrew` package's checkout (both roots it). A
//!    cargo test binary keeps its working directory, as tests always did.
//! 3. **Installed**: data in `~/Library/Application Support/XFlightCrew`
//!    (macOS) or `$XDG_DATA_HOME/xflightcrew` (default
//!    `~/.local/share/xflightcrew`), created on first use; resources the
//!    first of `<exe dir>/` and `<exe dir>/../share/xflightcrew/` holding
//!    `profiles/`.

use std::path::{Path, PathBuf};
use std::sync::OnceLock;

/// How the roots were found.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// `XCOPILOT_HOME`.
    Override,
    /// A source checkout (the working directory, or the binary's own).
    DevCheckout,
    /// A copied or packaged binary.
    Installed,
}

impl Mode {
    pub fn as_str(self) -> &'static str {
        match self {
            Mode::Override => "override",
            Mode::DevCheckout => "dev checkout",
            Mode::Installed => "installed",
        }
    }
}

/// The two roots, as decided.
#[derive(Debug, Clone, PartialEq)]
pub struct Roots {
    pub mode: Mode,
    /// What ships (read only). Empty means the working directory.
    pub resources: PathBuf,
    /// The user's (writable). Empty means the working directory.
    pub data: PathBuf,
    /// No `profiles/` in the resources (an override without them, or no
    /// installed candidate, `resources` then the binary's folder): the
    /// profiles are the built-ins. Said at startup ([`missing_line`]).
    pub resources_missing: bool,
}

/// What [`resolve`] decides from: the process's inputs, so it is testable.
#[derive(Debug, Clone, Default)]
pub struct Inputs {
    /// `XCOPILOT_HOME`, when set and not blank.
    pub home_override: Option<PathBuf>,
    /// The working directory.
    pub cwd: PathBuf,
    /// The running binary.
    pub exe: Option<PathBuf>,
    /// `HOME`.
    pub home: Option<PathBuf>,
    /// `XDG_DATA_HOME` (Linux).
    pub xdg_data_home: Option<PathBuf>,
    /// macOS's layout, else XDG.
    pub macos: bool,
}

impl Inputs {
    /// This process's.
    pub fn from_process() -> Inputs {
        let nonblank = |k: &str| {
            std::env::var_os(k)
                .filter(|v| !v.to_string_lossy().trim().is_empty())
                .map(PathBuf::from)
        };
        Inputs {
            home_override: nonblank("XCOPILOT_HOME"),
            cwd: std::env::current_dir().unwrap_or_default(),
            exe: std::env::current_exe().ok(),
            home: nonblank("HOME"),
            xdg_data_home: nonblank("XDG_DATA_HOME"),
            macos: cfg!(target_os = "macos"),
        }
    }
}

/// A source checkout: `profiles/` and `Cargo.toml` side by side.
fn is_checkout(dir: &Path) -> bool {
    dir.join("profiles").is_dir() && dir.join("Cargo.toml").is_file()
}

/// The `xflightcrew` package's own manifest (its `[package]` name).
fn is_our_manifest(dir: &Path) -> bool {
    std::fs::read_to_string(dir.join("Cargo.toml")).is_ok_and(|t| {
        t.lines()
            .map(str::trim)
            .any(|l| l.replace(' ', "") == "name=\"xflightcrew\"")
    })
}

/// A cargo test harness: `target/<profile>/deps/<name>-<hash>`.
fn is_test_harness(exe: &Path) -> bool {
    exe.parent()
        .and_then(Path::file_name)
        .is_some_and(|n| n == "deps")
}

/// The installed data folder for this platform.
fn installed_data(i: &Inputs) -> PathBuf {
    let home = i.home.clone().unwrap_or_default();
    if i.macos {
        home.join("Library/Application Support/XFlightCrew")
    } else {
        i.xdg_data_home
            .clone()
            .unwrap_or_else(|| home.join(".local/share"))
            .join("xflightcrew")
    }
}

/// The roots for these inputs. Pure but for the folder checks (it reads,
/// never creates).
pub fn resolve(i: &Inputs) -> Roots {
    if let Some(dir) = &i.home_override {
        return Roots {
            mode: Mode::Override,
            resources: dir.clone(),
            data: dir.clone(),
            resources_missing: !dir.join("profiles").is_dir(),
        };
    }
    if is_checkout(&i.cwd) {
        return Roots {
            mode: Mode::DevCheckout,
            resources: PathBuf::new(),
            data: PathBuf::new(),
            resources_missing: false,
        };
    }
    if let Some(exe) = &i.exe {
        if let Some(root) = exe
            .ancestors()
            .skip(1)
            .find(|d| is_checkout(d) && is_our_manifest(d))
        {
            let root = if is_test_harness(exe) {
                PathBuf::new()
            } else {
                root.to_path_buf()
            };
            return Roots {
                mode: Mode::DevCheckout,
                resources: root.clone(),
                data: root,
                resources_missing: false,
            };
        }
    }
    let exe_dir = i
        .exe
        .as_deref()
        .and_then(Path::parent)
        .map(Path::to_path_buf)
        .unwrap_or_default();
    let candidates = [exe_dir.clone(), exe_dir.join("../share/xflightcrew")];
    let found = candidates.iter().find(|d| d.join("profiles").is_dir());
    Roots {
        mode: Mode::Installed,
        resources: found.cloned().unwrap_or(exe_dir),
        data: installed_data(i),
        resources_missing: found.is_none(),
    }
}

static ROOTS: OnceLock<Roots> = OnceLock::new();

/// The process's roots, resolved on first use (after the working
/// directory's `.env` is loaded: `config::Config::from_env`). The installed
/// data folder is created then.
pub fn roots() -> &'static Roots {
    ROOTS.get_or_init(|| {
        let r = resolve(&Inputs::from_process());
        if r.mode != Mode::DevCheckout {
            let _ = std::fs::create_dir_all(&r.data);
        }
        r
    })
}

/// What ships (read only).
pub fn resources() -> &'static Path {
    &roots().resources
}

/// The user's (writable).
pub fn data() -> &'static Path {
    &roots().data
}

/// The aircraft profiles' folder (the YAMLs).
pub fn profiles_dir() -> PathBuf {
    resources().join("profiles")
}

/// A shipped file: `resource("checklists/md82.txt")`. A relative path a
/// profile carries resolves here; an absolute one stays itself.
pub fn resource(rel: impl AsRef<Path>) -> PathBuf {
    resources().join(rel)
}

/// A shipped asset: `asset("afv/Click_f32.wav")`.
pub fn asset(rel: &str) -> PathBuf {
    resources().join("assets").join(rel)
}

/// The default voices and speech-models folder (`XCOPILOT_MODELS_DIR`
/// wins, in `config::models_dir`).
pub fn models_default() -> PathBuf {
    data().join("models")
}

/// The logs folder: the tracing log, the blackbox, the sorties.
pub fn logs_dir() -> PathBuf {
    data().join("logs")
}

/// The blackbox.
pub fn blackbox_file() -> PathBuf {
    logs_dir().join("blackbox.jsonl")
}

/// One file per sortie.
pub fn sorties_dir() -> PathBuf {
    logs_dir().join("sorties")
}

/// The tracing log's base name, in [`logs_dir`] (rolled daily).
pub const LOG_FILE_NAME: &str = "xflightcrew.log";

/// The saved settings.
pub fn prefs_file() -> PathBuf {
    data().join("prefs.json")
}

/// The user's `.env` in the data folder.
pub fn env_file() -> PathBuf {
    data().join(".env")
}

/// Where `xfc identify` WRITES a learned model.
pub fn learned_write_dir() -> PathBuf {
    data().join("profiles/learned")
}

/// Where a learned model is READ, in order: the user's, then the shipped
/// ones (one folder in a checkout).
pub fn learned_read_dirs() -> Vec<PathBuf> {
    let mut dirs = vec![learned_write_dir()];
    let shipped = resources().join("profiles/learned");
    if !dirs.contains(&shipped) {
        dirs.push(shipped);
    }
    dirs
}

/// A path as a person reads it: absolute against the working directory.
pub fn shown(p: &Path) -> String {
    let abs = if p.is_absolute() {
        p.to_path_buf()
    } else {
        std::env::current_dir().unwrap_or_default().join(p)
    };
    let s = abs.to_string_lossy().into_owned();
    s.strip_suffix('/').map(str::to_string).unwrap_or(s)
}

/// One line: the mode and both roots (the startup log, `xfc paths`).
pub fn summary() -> String {
    let r = roots();
    let mut line = format!(
        "{} (resources {}, data {})",
        r.mode.as_str(),
        shown(&r.resources),
        shown(&r.data)
    );
    if let Some(missing) = missing_line() {
        line.push_str("; ");
        line.push_str(missing);
    }
    line
}

/// Said when the resources hold no `profiles/`; `None` when they do.
pub fn missing_line() -> Option<&'static str> {
    let r = roots();
    r.resources_missing.then_some(match r.mode {
        Mode::Override => "no profiles/ in XCOPILOT_HOME: built-in profiles only, no checklists",
        _ => "no profiles/ beside the binary or in ../share/xflightcrew: built-in profiles only, no checklists",
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch(tag: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("xfc-paths-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    fn checkout(at: &Path) {
        std::fs::create_dir_all(at.join("profiles")).unwrap();
        std::fs::write(at.join("Cargo.toml"), "[package]\nname = \"xflightcrew\"\n").unwrap();
    }

    #[test]
    fn the_override_is_both_roots() {
        let i = Inputs {
            home_override: Some("/x/home".into()),
            cwd: "/".into(),
            ..Default::default()
        };
        let r = resolve(&i);
        assert_eq!(r.mode, Mode::Override);
        assert_eq!(r.resources, PathBuf::from("/x/home"));
        assert_eq!(r.data, PathBuf::from("/x/home"));
        assert!(r.resources_missing, "no profiles/ there");
    }

    /// The pilot's setup: every path the same relative string it was.
    #[test]
    fn a_checkout_as_the_working_directory_stays_relative() {
        let d = scratch("cwd");
        checkout(&d);
        let r = resolve(&Inputs {
            cwd: d.clone(),
            ..Default::default()
        });
        assert_eq!(r.mode, Mode::DevCheckout);
        assert_eq!(
            r.data.join("logs/blackbox.jsonl"),
            PathBuf::from("logs/blackbox.jsonl")
        );
        assert_eq!(r.resources.join("profiles"), PathBuf::from("profiles"));
    }

    #[test]
    fn a_checkout_binary_run_from_elsewhere_finds_its_checkout() {
        let d = scratch("exe");
        checkout(&d);
        let away = scratch("away");
        let r = resolve(&Inputs {
            cwd: away.clone(),
            exe: Some(d.join("target/release/xfc")),
            ..Default::default()
        });
        assert_eq!(r.mode, Mode::DevCheckout);
        assert_eq!(r.data, d);
        // A test harness in the same tree keeps its working directory.
        let t = resolve(&Inputs {
            cwd: away,
            exe: Some(d.join("target/debug/deps/atc-0123")),
            ..Default::default()
        });
        assert_eq!(t.mode, Mode::DevCheckout);
        assert_eq!(t.data, PathBuf::new());
    }

    #[test]
    fn another_package_is_no_checkout() {
        let d = scratch("other");
        std::fs::create_dir_all(d.join("profiles")).unwrap();
        std::fs::write(d.join("Cargo.toml"), "[package]\nname = \"other\"\n").unwrap();
        let away = scratch("away2");
        let r = resolve(&Inputs {
            cwd: away,
            exe: Some(d.join("bin/xfc")),
            home: Some("/h".into()),
            macos: true,
            ..Default::default()
        });
        assert_eq!(r.mode, Mode::Installed);
    }

    #[test]
    fn installed_data_is_the_platforms_and_resources_sit_by_the_binary() {
        let d = scratch("inst");
        std::fs::create_dir_all(d.join("bin")).unwrap();
        std::fs::create_dir_all(d.join("share/xflightcrew/profiles")).unwrap();
        let mac = resolve(&Inputs {
            cwd: "/".into(),
            exe: Some(d.join("bin/xfc")),
            home: Some("/h".into()),
            macos: true,
            ..Default::default()
        });
        assert_eq!(mac.mode, Mode::Installed);
        assert_eq!(
            mac.data,
            PathBuf::from("/h/Library/Application Support/XFlightCrew")
        );
        assert_eq!(mac.resources, d.join("bin/../share/xflightcrew"));
        assert!(!mac.resources_missing);
        let linux = resolve(&Inputs {
            cwd: "/".into(),
            exe: Some(d.join("bin/xfc")),
            home: Some("/h".into()),
            ..Default::default()
        });
        assert_eq!(linux.data, PathBuf::from("/h/.local/share/xflightcrew"));
        let xdg = resolve(&Inputs {
            cwd: "/".into(),
            exe: Some(d.join("bin/xfc")),
            home: Some("/h".into()),
            xdg_data_home: Some("/x".into()),
            ..Default::default()
        });
        assert_eq!(xdg.data, PathBuf::from("/x/xflightcrew"));
        // Beside the binary wins; none at all is said.
        std::fs::create_dir_all(d.join("bin/profiles")).unwrap();
        let beside = resolve(&Inputs {
            cwd: "/".into(),
            exe: Some(d.join("bin/xfc")),
            ..Default::default()
        });
        assert_eq!(beside.resources, d.join("bin"));
        let bare = scratch("bare");
        let none = resolve(&Inputs {
            cwd: "/".into(),
            exe: Some(bare.join("xfc")),
            ..Default::default()
        });
        assert!(none.resources_missing);
        assert_eq!(none.resources, bare);
    }

    /// The lines that may name a root, each with its reason.
    const ALLOWED: &[(&str, &str, &str)] = &[
        // A profile's own value, as the YAMLs carry it: a shipped file the
        // checklist loader resolves through `resource`.
        (
            "src/aircraft/builtin_profiles.rs",
            "checklist: Some(\"checklists/",
            "profile data",
        ),
        // The spoken word, not a folder.
        ("src/flow.rs", "\"checklists\"", "a word"),
        ("src/router.rs", "\"checklists\"", "a word"),
        ("src/checklist.rs", "\"checklists\"", "a word"),
    ];

    /// ONE owner: outside this file no production code names a top-level
    /// folder or file of either root (`logs`, `models`, `profiles`,
    /// `checklists`, `assets`, `prefs.json`, `.env`) as a literal. Test code
    /// runs in the checkout and keeps its relative paths.
    #[test]
    fn every_file_root_goes_through_the_owner() {
        let repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let mut roots = vec![repo.join("src")];
        for e in std::fs::read_dir(repo.join("crates")).unwrap().flatten() {
            roots.push(e.path().join("src"));
        }
        fn walk(dir: &Path, out: &mut Vec<PathBuf>) {
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
        files.sort();
        let names = [
            "logs",
            "models",
            "profiles",
            "checklists",
            "assets",
            "prefs.json",
            ".env",
        ];
        let mut needles = Vec::new();
        for n in names {
            for pre in ["\"", "\"./"] {
                needles.push(format!("{pre}{n}/"));
                needles.push(format!("{pre}{n}\""));
            }
        }
        let mut bad = Vec::new();
        let mut scanned = 0;
        for f in &files {
            let rel = f
                .strip_prefix(&repo)
                .unwrap()
                .to_string_lossy()
                .replace('\\', "/");
            let name = f.file_name().unwrap().to_string_lossy();
            if name == "tests.rs"
                || name.ends_with("_tests.rs")
                || rel.contains("/tests/")
                || rel == "crates/sim/src/paths.rs"
            {
                continue;
            }
            let text = std::fs::read_to_string(f).unwrap();
            if text.trim_start().starts_with("#![cfg(test)]") {
                continue;
            }
            scanned += 1;
            let production = text.split("#[cfg(test)]\nmod ").next().unwrap();
            let mut skip_next = false;
            for (i, l) in production.lines().enumerate() {
                if skip_next {
                    skip_next = false;
                    continue;
                }
                if l.trim() == "#[cfg(test)]" {
                    skip_next = true;
                    continue;
                }
                let code = l.split("//").next().unwrap_or("");
                if code.contains("include_str!") || code.contains("CARGO_MANIFEST_DIR") {
                    continue;
                }
                if !needles.iter().any(|n| code.contains(n.as_str())) {
                    continue;
                }
                if ALLOWED
                    .iter()
                    .any(|(file, snippet, _)| *file == rel && code.contains(snippet))
                {
                    continue;
                }
                bad.push(format!("{rel}:{}: {}", i + 1, l.trim()));
            }
        }
        assert!(scanned > 100, "only {scanned} files scanned");
        assert!(
            bad.is_empty(),
            "a file root named outside sim::paths (route it through the owner):\n{}",
            bad.join("\n")
        );
    }
}
