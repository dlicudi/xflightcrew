//! `xfc plugin status|install`: the X-Plane plugin carried inside `xfc`
//! (the `embed-plugin` feature, scripts/build-release.sh), installed the
//! way `make -C plugin install` does (docs/tasks/embedded-resources/BRIEF.md).
//!
//! ```text
//! xfc plugin status  [--xplane <dir>]
//! xfc plugin install [--xplane <dir>] [--dry-run] [--while-running]
//! ```
//!
//! The old `plugins/xcopilot/` goes FIRST (two copies would both load and
//! register every dataref twice, CLAUDE.md "Wire contracts"); a replaced
//! plugin is backed up to the data folder, never beside it (X-Plane loads
//! every folder under `plugins/`); the `.xpl` is written atomically and read
//! back. It NEVER reloads plugins: a reload under a live panel broke it.
//! Pure core ([`marker`], [`verdict`], [`plan`]), thin shell ([`apply`],
//! [`main_cmd`]).

use std::path::{Path, PathBuf};

use anyhow::{anyhow, bail, Context, Result};

/// The installed plugin, under the X-Plane folder.
pub const XPL_REL: &str = "Resources/plugins/xflightcrew/mac_x64/xflightcrew.xpl";
/// Its folder: what a backup copies.
pub const DIR_REL: &str = "Resources/plugins/xflightcrew";
/// The pre-rename folder, removed first.
pub const OLD_REL: &str = "Resources/plugins/xcopilot";
/// X-Plane's plugins folder: its absence means this is no X-Plane folder.
pub const PLUGINS_REL: &str = "Resources/plugins";

/// The plugin this build carries, or `None` (the default features).
pub fn embedded() -> Option<&'static [u8]> {
    #[cfg(feature = "embed-plugin")]
    {
        Some(include_bytes!(env!("XFC_PLUGIN_XPL_FILE")))
    }
    #[cfg(not(feature = "embed-plugin"))]
    {
        None
    }
}

/// A plugin build's identity: the Makefile's `VERSION` (the commit's date,
/// `2026.10.13`) and `BUILD` (short SHA, maybe `-dirty.HHMMSS`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Ident {
    pub version: String,
    pub build: Option<String>,
}

impl Ident {
    fn shown(&self) -> String {
        match &self.build {
            Some(b) => format!("{} (build {b})", self.version),
            None => format!("{} (build unknown)", self.version),
        }
    }
}

fn find(hay: &[u8], needle: &[u8]) -> Option<usize> {
    hay.windows(needle.len()).position(|w| w == needle)
}

/// The text after `start` up to the first byte in `stops` (at most 64
/// bytes, printable ASCII only).
fn field(bytes: &[u8], start: usize, stops: &[u8]) -> Option<String> {
    let rest = bytes.get(start..)?;
    let end = rest.iter().take(64).position(|b| stops.contains(b))?;
    let s = std::str::from_utf8(&rest[..end]).ok()?;
    (!s.is_empty() && s.bytes().all(|b| b.is_ascii_graphic())).then(|| s.to_string())
}

/// Read a plugin's identity from its bytes: the marker the plugin carries
/// (`XFC_PLUGIN_VERSION=<v>;XFC_PLUGIN_BUILD=<b>;`, plugin/src/xflightcrew.cpp),
/// else, for a plugin built before it, the panel's "after a rebuild · v<v>
/// · <b>" line, else the description's "XFlightCrew <v>,".
pub fn marker(bytes: &[u8]) -> Option<Ident> {
    const V: &[u8] = b"XFC_PLUGIN_VERSION=";
    const B: &[u8] = b";XFC_PLUGIN_BUILD=";
    if let Some(at) = find(bytes, V) {
        let start = at + V.len();
        if let Some(version) = field(bytes, start, b";") {
            let after = start + version.len();
            let build = bytes
                .get(after..)
                .filter(|r| r.starts_with(B))
                .and_then(|_| field(bytes, after + B.len(), b";"));
            return Some(Ident { version, build });
        }
    }
    const PANEL: &[u8] = "after a rebuild \u{b7} v".as_bytes();
    const DOT: &[u8] = " \u{b7} ".as_bytes();
    if let Some(at) = find(bytes, PANEL) {
        let start = at + PANEL.len();
        if let Some(version) = field(bytes, start, b" ") {
            let after = start + version.len();
            let build = bytes
                .get(after..)
                .filter(|r| r.starts_with(DOT))
                .and_then(|_| field(bytes, after + DOT.len(), b"\0"));
            return Some(Ident { version, build });
        }
    }
    const DESC: &[u8] = b"XFlightCrew ";
    let mut from = 0;
    while let Some(at) = find(&bytes[from..], DESC) {
        let start = from + at + DESC.len();
        if let Some(version) = field(bytes, start, b",") {
            if parse_version(&version).is_some() {
                return Some(Ident {
                    version,
                    build: None,
                });
            }
        }
        from = start;
    }
    None
}

/// `2026.10.13` as numbers; `dev` or anything else is no version.
fn parse_version(v: &str) -> Option<Vec<u32>> {
    v.split('.').map(|p| p.parse().ok()).collect()
}

/// How the installed plugin stands against the one this build carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    /// No `.xpl` installed.
    Missing,
    /// The same version and build.
    UpToDate,
    /// An earlier date than the carried one.
    Older,
    /// A later date than the carried one.
    Newer,
    /// The same date, another build: which is newer cannot be told.
    OtherBuild,
    /// Installed, but one of the versions could not be read.
    Unknown,
    /// Installed; this build carries no plugin to compare with.
    NoReference,
}

impl Verdict {
    pub fn line(self) -> &'static str {
        match self {
            Verdict::Missing => "missing: not installed",
            Verdict::UpToDate => "up to date",
            Verdict::Older => "older than this build's: xfc plugin install updates it",
            Verdict::Newer => "newer than this build's: install would downgrade it",
            Verdict::OtherBuild => {
                "another build of the same date: xfc plugin install replaces it with this build's"
            }
            Verdict::Unknown => "installed, but its version cannot be compared",
            Verdict::NoReference => "installed; this build carries no plugin to compare with",
        }
    }
}

/// The verdict, pure.
pub fn verdict(installed: Option<Option<&Ident>>, carried: Option<&Ident>) -> Verdict {
    let Some(installed) = installed else {
        return Verdict::Missing;
    };
    let Some(carried) = carried else {
        return Verdict::NoReference;
    };
    let Some(installed) = installed else {
        return Verdict::Unknown;
    };
    let (Some(a), Some(b)) = (
        parse_version(&installed.version),
        parse_version(&carried.version),
    ) else {
        return if installed == carried {
            Verdict::UpToDate
        } else {
            Verdict::Unknown
        };
    };
    match a.cmp(&b) {
        std::cmp::Ordering::Less => Verdict::Older,
        std::cmp::Ordering::Greater => Verdict::Newer,
        std::cmp::Ordering::Equal => match (&installed.build, &carried.build) {
            (Some(x), Some(y)) if x == y => Verdict::UpToDate,
            _ => Verdict::OtherBuild,
        },
    }
}

/// What `status` found: reads only.
#[derive(Debug, Clone)]
pub struct Status {
    pub xplane: PathBuf,
    pub is_xplane: bool,
    pub xpl: PathBuf,
    /// `None` not installed; `Some(None)` installed, version unreadable.
    pub installed: Option<Option<Ident>>,
    pub old_present: bool,
    pub carried: Option<Ident>,
    pub verdict: Verdict,
}

/// Look at an X-Plane folder (reads, never writes).
pub fn status(xplane: &Path, carried_bytes: Option<&[u8]>) -> Status {
    let xpl = xplane.join(XPL_REL);
    let installed = std::fs::read(&xpl).ok().map(|b| marker(&b));
    let carried = carried_bytes.and_then(marker);
    let verdict = match (&installed, carried_bytes, &carried) {
        (None, ..) => Verdict::Missing,
        (Some(_), None, _) => Verdict::NoReference,
        // A carried plugin whose marker cannot be read compares as unknown.
        (Some(_), Some(_), None) => Verdict::Unknown,
        (Some(i), Some(_), Some(c)) => verdict(Some(i.as_ref()), Some(c)),
    };
    Status {
        is_xplane: xplane.join(PLUGINS_REL).is_dir(),
        old_present: xplane.join(OLD_REL).exists(),
        xplane: xplane.to_path_buf(),
        xpl,
        installed,
        carried,
        verdict,
    }
}

/// The status as printed.
pub fn render_status(s: &Status, carries: bool) -> String {
    let mut out = format!("X-Plane:     {}", s.xplane.display());
    if !s.is_xplane {
        out += "  (no Resources/plugins here: not an X-Plane folder)";
    }
    out += "\n";
    out += &format!("plugin:      {}\n", s.xpl.display());
    out += &format!(
        "installed:   {}\n",
        match &s.installed {
            None => "no".to_string(),
            Some(None) => "yes, version unreadable".to_string(),
            Some(Some(id)) => format!("yes, {}", id.shown()),
        }
    );
    out += &format!(
        "old xcopilot/: {}\n",
        if s.old_present {
            "PRESENT (two copies would both load: xfc plugin install removes it)"
        } else {
            "none"
        }
    );
    out += &format!(
        "this build:  {}\n",
        match (&s.carried, carries) {
            (_, false) => "this build carries no plugin".to_string(),
            (None, true) => "carries a plugin, version unreadable".to_string(),
            (Some(id), true) => format!("carries {}", id.shown()),
        }
    );
    out += &format!("verdict:     {}\n", s.verdict.line());
    out
}

/// One step of an install, in order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Step {
    /// The pre-rename folder, removed FIRST.
    RemoveOld(PathBuf),
    /// The current plugin folder copied out of `plugins/`.
    Backup { from: PathBuf, to: PathBuf },
    /// The `mac_x64/` folder.
    CreateDir(PathBuf),
    /// The `.xpl`, through a temp file in the same folder and a rename.
    WriteAtomic { path: PathBuf, tmp: PathBuf },
    /// The written file read back against the carried bytes.
    ReadBack(PathBuf),
}

impl Step {
    pub fn describe(&self) -> String {
        match self {
            Step::RemoveOld(p) => format!(
                "remove the old plugin folder {} (two copies would both load)",
                p.display()
            ),
            Step::Backup { from, to } => {
                format!("back up {} to {}", from.display(), to.display())
            }
            Step::CreateDir(p) => format!("create {}", p.display()),
            Step::WriteAtomic { path, tmp } => format!(
                "write {} (through {}, then a rename)",
                path.display(),
                tmp.display()
            ),
            Step::ReadBack(p) => format!("read {} back", p.display()),
        }
    }
}

/// What is on disk before an install.
#[derive(Debug, Clone, Copy, Default)]
pub struct Present {
    pub old: bool,
    pub current: bool,
}

impl Present {
    pub fn look(xplane: &Path) -> Present {
        Present {
            old: xplane.join(OLD_REL).exists(),
            current: xplane.join(DIR_REL).exists(),
        }
    }
}

/// The install, as steps: pure.
pub fn plan(xplane: &Path, backup_dir: &Path, present: Present) -> Vec<Step> {
    let mut steps = Vec::new();
    if present.old {
        steps.push(Step::RemoveOld(xplane.join(OLD_REL)));
    }
    if present.current {
        steps.push(Step::Backup {
            from: xplane.join(DIR_REL),
            to: backup_dir.join("xflightcrew"),
        });
    }
    let path = xplane.join(XPL_REL);
    let dir = path.parent().unwrap().to_path_buf();
    steps.push(Step::CreateDir(dir.clone()));
    steps.push(Step::WriteAtomic {
        tmp: dir.join(format!(".xflightcrew.xpl.partial-{}", std::process::id())),
        path: path.clone(),
    });
    steps.push(Step::ReadBack(path));
    steps
}

fn copy_tree(from: &Path, to: &Path) -> std::io::Result<()> {
    std::fs::create_dir_all(to)?;
    for e in std::fs::read_dir(from)? {
        let e = e?;
        let target = to.join(e.file_name());
        let ty = e.file_type()?;
        if ty.is_dir() {
            copy_tree(&e.path(), &target)?;
        } else if ty.is_symlink() {
            #[cfg(unix)]
            std::os::unix::fs::symlink(std::fs::read_link(e.path())?, &target)?;
        } else {
            std::fs::copy(e.path(), &target)?;
        }
    }
    Ok(())
}

/// Carry the steps out with these bytes.
pub fn apply(steps: &[Step], bytes: &[u8]) -> Result<()> {
    for step in steps {
        match step {
            Step::RemoveOld(p) => std::fs::remove_dir_all(p)
                .or_else(|e| {
                    if p.is_file() {
                        std::fs::remove_file(p)
                    } else {
                        Err(e)
                    }
                })
                .with_context(|| format!("removing {}", p.display()))?,
            Step::Backup { from, to } => {
                copy_tree(from, to).with_context(|| format!("backing up to {}", to.display()))?
            }
            Step::CreateDir(p) => {
                std::fs::create_dir_all(p).with_context(|| format!("creating {}", p.display()))?
            }
            Step::WriteAtomic { path, tmp } => {
                let write = || -> std::io::Result<()> {
                    std::fs::write(tmp, bytes)?;
                    #[cfg(unix)]
                    {
                        use std::os::unix::fs::PermissionsExt;
                        std::fs::set_permissions(tmp, std::fs::Permissions::from_mode(0o755))?;
                    }
                    std::fs::rename(tmp, path)
                };
                if let Err(e) = write() {
                    let _ = std::fs::remove_file(tmp);
                    return Err(anyhow!(e)).with_context(|| format!("writing {}", path.display()));
                }
            }
            Step::ReadBack(p) => {
                let back = std::fs::read(p).with_context(|| format!("reading {}", p.display()))?;
                if back != bytes {
                    bail!("{} does not read back as written", p.display());
                }
            }
        }
    }
    Ok(())
}

/// A UTC stamp for a backup folder: `20261013-235959`.
pub fn stamp(unix_secs: u64) -> String {
    let days = (unix_secs / 86_400) as i64;
    let rem = unix_secs % 86_400;
    // Howard Hinnant's civil_from_days.
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!(
        "{y:04}{m:02}{d:02}-{:02}{:02}{:02}",
        rem / 3600,
        rem / 60 % 60,
        rem % 60
    )
}

/// Is X-Plane running (a process named `X-Plane`)? `None` when it cannot
/// be told.
fn xplane_running() -> Option<bool> {
    let out = std::process::Command::new("pgrep")
        .args(["-x", "X-Plane"])
        .output()
        .ok()?;
    match out.status.code() {
        Some(0) => Some(true),
        Some(1) => Some(false),
        _ => None,
    }
}

const USAGE: &str = "usage:
  xfc plugin status  [--xplane <dir>]
  xfc plugin install [--xplane <dir>] [--dry-run] [--while-running]";

/// `xfc plugin ...`: true on success.
pub fn main_cmd(args: &[String]) -> Result<bool> {
    let mut xplane: Option<PathBuf> = None;
    let (mut dry_run, mut while_running) = (false, false);
    let mut it = args.iter().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--xplane" => {
                xplane = Some(
                    it.next()
                        .ok_or_else(|| anyhow!("--xplane needs a folder\n{USAGE}"))?
                        .into(),
                )
            }
            "--dry-run" => dry_run = true,
            "--while-running" => while_running = true,
            other => bail!("unknown option {other}\n{USAGE}"),
        }
    }
    let xplane = xplane.unwrap_or_else(|| crate::config::xplane_root().into());
    let carried = embedded();
    match args.first().map(String::as_str) {
        Some("status") => {
            print!(
                "{}",
                render_status(&status(&xplane, carried), carried.is_some())
            );
            Ok(true)
        }
        Some("install") => install(&xplane, carried, dry_run, while_running),
        _ => {
            println!("{USAGE}");
            Ok(args.is_empty())
        }
    }
}

fn install(
    xplane: &Path,
    carried: Option<&[u8]>,
    dry_run: bool,
    while_running: bool,
) -> Result<bool> {
    let mut out = String::new();
    let result = install_to(xplane, carried, dry_run, while_running, &mut out);
    print!("{out}");
    result
}

/// The install, its report written to `out` (the CLI prints it, the home
/// screen shows it): the same refusals, the same steps.
pub fn install_to(
    xplane: &Path,
    carried: Option<&[u8]>,
    dry_run: bool,
    while_running: bool,
    out: &mut String,
) -> Result<bool> {
    use std::fmt::Write as _;
    let Some(bytes) = carried else {
        let _ = writeln!(
            out,
            "this build carries no plugin (built without the embed-plugin feature).\n\
             From a source checkout: make -C plugin install\n\
             Or build a release that carries it: scripts/build-release.sh"
        );
        return Ok(false);
    };
    if !xplane.join(PLUGINS_REL).is_dir() {
        let _ = writeln!(
            out,
            "{} has no Resources/plugins: not an X-Plane folder (pass --xplane <dir> or set XCOPILOT_XPLANE_ROOT)",
            xplane.display()
        );
        return Ok(false);
    }
    let running = xplane_running();
    if running != Some(false) && !while_running {
        let why = if running == Some(true) {
            "X-Plane is running"
        } else {
            "cannot tell whether X-Plane is running"
        };
        if !dry_run {
            let _ = writeln!(
                out,
                "{why}: refusing to install. Quit X-Plane first, or pass --while-running and then reload plugins or restart X-Plane yourself."
            );
            return Ok(false);
        }
        let _ = writeln!(
            out,
            "{why}: a real install would refuse without --while-running."
        );
    }
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    let backup_dir = sim::paths::plugin_backups_dir().join(stamp(now));
    let backup_dir = PathBuf::from(sim::paths::shown(&backup_dir));
    let steps = plan(xplane, &backup_dir, Present::look(xplane));
    if let Some(id) = marker(bytes) {
        let _ = writeln!(
            out,
            "installing the plugin this build carries: {}",
            id.shown()
        );
    }
    if dry_run {
        let _ = writeln!(out, "dry run, the steps an install would take:");
    }
    for (n, s) in steps.iter().enumerate() {
        let _ = writeln!(out, "{}. {}", n + 1, s.describe());
    }
    if dry_run {
        let _ = writeln!(out, "dry run: nothing changed");
        return Ok(true);
    }
    apply(&steps, bytes)?;
    let _ = writeln!(out, "installed to {}", xplane.join(XPL_REL).display());
    if running == Some(true) {
        let _ = writeln!(
            out,
            "X-Plane is running: the new plugin loads only once you reload plugins or restart X-Plane (xfc never reloads them)."
        );
    } else {
        let _ = writeln!(out, "it loads the next time X-Plane starts.");
    }
    Ok(true)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch(tag: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("xfc-plugin-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(d.join(PLUGINS_REL)).unwrap();
        d
    }

    fn xpl(version: &str, build: &str) -> Vec<u8> {
        let mut b = b"\xcf\xfa\xed\xfe junk \0".to_vec();
        b.extend_from_slice(
            format!("XFC_PLUGIN_VERSION={version};XFC_PLUGIN_BUILD={build};\0").as_bytes(),
        );
        b.extend_from_slice(b"\0more");
        b
    }

    #[test]
    fn the_marker_is_read_from_the_bytes() {
        assert_eq!(
            marker(&xpl("2026.10.13", "0e3abe1-dirty.235631")),
            Some(Ident {
                version: "2026.10.13".into(),
                build: Some("0e3abe1-dirty.235631".into())
            })
        );
        // A plugin from before the marker: the panel's line.
        let old = "x\0after a rebuild \u{b7} v2026.10.12 \u{b7} abc1234\0y".as_bytes();
        assert_eq!(
            marker(old),
            Some(Ident {
                version: "2026.10.12".into(),
                build: Some("abc1234".into())
            })
        );
        // Only the description: a version, no build.
        let desc = b"\0XFlightCrew plugin: started\0XFlightCrew 2026.9.1, push-to-talk\0";
        assert_eq!(
            marker(desc),
            Some(Ident {
                version: "2026.9.1".into(),
                build: None
            })
        );
        assert_eq!(marker(b"nothing here"), None);
    }

    fn id(v: &str, b: &str) -> Ident {
        Ident {
            version: v.into(),
            build: Some(b.into()),
        }
    }

    #[test]
    fn the_verdict_compares_dates_then_builds() {
        let c = id("2026.10.13", "aaa");
        assert_eq!(verdict(None, Some(&c)), Verdict::Missing);
        assert_eq!(verdict(Some(Some(&c)), Some(&c)), Verdict::UpToDate);
        // Numbers, not strings: 9 < 10.
        assert_eq!(
            verdict(Some(Some(&id("2026.9.30", "x"))), Some(&c)),
            Verdict::Older
        );
        assert_eq!(
            verdict(Some(Some(&id("2026.11.1", "x"))), Some(&c)),
            Verdict::Newer
        );
        assert_eq!(
            verdict(Some(Some(&id("2026.10.13", "bbb"))), Some(&c)),
            Verdict::OtherBuild
        );
        assert_eq!(verdict(Some(None), Some(&c)), Verdict::Unknown);
        assert_eq!(verdict(Some(Some(&c)), None), Verdict::NoReference);
        assert_eq!(verdict(None, None), Verdict::Missing);
        assert_eq!(
            verdict(Some(Some(&id("dev", "x"))), Some(&c)),
            Verdict::Unknown
        );
    }

    #[test]
    fn status_reads_a_scratch_folder() {
        let x = scratch("status");
        let carried = xpl("2026.10.13", "aaa");
        let s = status(&x, Some(&carried));
        assert!(s.is_xplane);
        assert_eq!(s.verdict, Verdict::Missing);
        assert!(!s.old_present);
        std::fs::create_dir_all(x.join(OLD_REL)).unwrap();
        std::fs::create_dir_all(x.join(XPL_REL).parent().unwrap()).unwrap();
        std::fs::write(x.join(XPL_REL), xpl("2026.10.12", "old")).unwrap();
        let s = status(&x, Some(&carried));
        assert!(s.old_present);
        assert_eq!(s.verdict, Verdict::Older);
        let text = render_status(&s, true);
        assert!(text.contains("PRESENT"), "{text}");
        assert!(text.contains("2026.10.12 (build old)"), "{text}");
        let s = status(&x, None);
        assert_eq!(s.verdict, Verdict::NoReference);
        assert!(render_status(&s, false).contains("this build carries no plugin"));
    }

    #[test]
    fn the_plan_removes_the_old_first_and_backs_up_out_of_plugins() {
        let x = PathBuf::from("/xp");
        let b = PathBuf::from("/data/plugin-backups/20261013-000000");
        let steps = plan(
            &x,
            &b,
            Present {
                old: true,
                current: true,
            },
        );
        assert_eq!(steps[0], Step::RemoveOld(x.join(OLD_REL)));
        assert_eq!(
            steps[1],
            Step::Backup {
                from: x.join(DIR_REL),
                to: b.join("xflightcrew")
            }
        );
        for s in &steps {
            if let Step::Backup { to, .. } = s {
                assert!(!to.starts_with(x.join(PLUGINS_REL)));
            }
            if let Step::WriteAtomic { path, tmp } = s {
                assert_eq!(path.parent(), tmp.parent(), "the temp file sits beside it");
                assert!(!tmp.to_string_lossy().ends_with(".xpl"));
            }
        }
        assert!(matches!(steps.last(), Some(Step::ReadBack(_))));
        let fresh = plan(&x, &b, Present::default());
        assert!(matches!(fresh[0], Step::CreateDir(_)));
        assert_eq!(fresh.len(), 3);
    }

    #[test]
    fn an_install_on_a_scratch_folder() {
        let x = scratch("install");
        let data = std::env::temp_dir().join(format!("xfc-plugin-data-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&data);
        std::fs::create_dir_all(x.join(OLD_REL).join("mac_x64")).unwrap();
        std::fs::write(x.join(OLD_REL).join("mac_x64/xcopilot.xpl"), b"old").unwrap();
        std::fs::create_dir_all(x.join(XPL_REL).parent().unwrap()).unwrap();
        std::fs::write(x.join(XPL_REL), xpl("2026.10.1", "prev")).unwrap();
        let backup = data.join("plugin-backups/20261013-120000");
        let bytes = xpl("2026.10.13", "new");
        let steps = plan(&x, &backup, Present::look(&x));
        apply(&steps, &bytes).unwrap();
        assert!(!x.join(OLD_REL).exists(), "the old folder is gone");
        assert_eq!(std::fs::read(x.join(XPL_REL)).unwrap(), bytes);
        assert_eq!(
            marker(&std::fs::read(backup.join("xflightcrew/mac_x64/xflightcrew.xpl")).unwrap())
                .unwrap()
                .version,
            "2026.10.1"
        );
        let names: Vec<_> = std::fs::read_dir(x.join(PLUGINS_REL))
            .unwrap()
            .flatten()
            .map(|e| e.file_name())
            .collect();
        assert_eq!(names, vec![std::ffi::OsString::from("xflightcrew")]);
        let in_mac: Vec<_> = std::fs::read_dir(x.join(XPL_REL).parent().unwrap())
            .unwrap()
            .flatten()
            .collect();
        assert_eq!(in_mac.len(), 1, "no temp file left");
        assert_eq!(status(&x, Some(&bytes)).verdict, Verdict::UpToDate);
    }

    #[test]
    fn a_backup_stamp_is_utc_civil_time() {
        assert_eq!(stamp(0), "19700101-000000");
        // 2026-10-13 23:59:59 UTC.
        assert_eq!(stamp(1_791_935_999), "20261013-235959");
        assert_eq!(stamp(951_782_400), "20000229-000000");
    }

    /// The carried plugin, in a release build, reads its own marker.
    #[cfg(feature = "embed-plugin")]
    #[test]
    fn the_carried_plugin_has_its_marker() {
        let id = marker(embedded().unwrap()).expect("marker");
        assert!(parse_version(&id.version).is_some(), "{id:?}");
        assert!(id.build.is_some());
    }
}
