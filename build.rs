//! Build script: stamp the version and the git build id into the binary, so the
//! app's startup log and status panel show exactly which build is running - no
//! doubt about whether a rebuilt copilot was actually picked up.
//!
//! The version is the DATE of the commit the build sits on (`2026.9.22`),
//! never a number typed into a file: a hand-kept `0.1.0` stood unchanged
//! through five hundred commits. Several commits on one day share a version;
//! the build id tells them apart. The plugin's Makefile computes the same
//! string the same way, so the pair can be compared at a glance.

use std::process::Command;

fn git(args: &[&str]) -> Option<String> {
    Command::new("git")
        .args(args)
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
}

fn main() {
    let sha = git(&["rev-parse", "--short", "HEAD"])
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| "unknown".to_string());
    let version = git(&["log", "-1", "--format=%cd", "--date=format:%Y.%-m.%-d"])
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| "dev".to_string());
    // Mark a dirty tree so a not-yet-committed build is distinguishable from
    // the commit it is based on. Staged changes count too: `git diff --quiet`
    // alone looked only at the unstaged ones.
    let dirty = git(&["status", "--porcelain", "--untracked-files=no"])
        .is_some_and(|s| !s.is_empty());
    let build = if dirty { format!("{sha}-dirty") } else { sha };
    // "debug" or "release": an unoptimised build runs every control tick and
    // every voice slower, and a log from one must say so.
    let profile = std::env::var("PROFILE").unwrap_or_else(|_| "unknown".to_string());
    println!("cargo:rustc-env=XVL_VERSION={version}");
    println!("cargo:rustc-env=XCOPILOT_BUILD={build}");
    println!("cargo:rustc-env=XVL_PROFILE={profile}");
    // Re-run when the commit or the working tree changes so the stamp stays
    // current: a commit moves the index and the branch ref, an edit moves the
    // sources (without these an edited tree kept its clean stamp).
    println!("cargo:rerun-if-changed=.git/HEAD");
    println!("cargo:rerun-if-changed=.git/index");
    if let Some(r) = git(&["symbolic-ref", "-q", "HEAD"]) {
        println!("cargo:rerun-if-changed=.git/{r}");
    }
    for dir in ["src", "crates", "profiles", "Cargo.toml"] {
        println!("cargo:rerun-if-changed={dir}");
    }
    embed_plugin();
}

/// The `embed-plugin` feature: the built `.xpl` (`XFC_PLUGIN_XPL`, set by
/// scripts/build-release.sh) carried inside `xfc` for `xfc plugin install`
/// (src/plugin_install.rs). Cargo cannot build the plugin (it needs the
/// X-Plane SDK and clang), so a release builds it first and points here.
fn embed_plugin() {
    println!("cargo:rerun-if-env-changed=XFC_PLUGIN_XPL");
    if std::env::var_os("CARGO_FEATURE_EMBED_PLUGIN").is_none() {
        return;
    }
    let Some(path) = std::env::var_os("XFC_PLUGIN_XPL").filter(|p| !p.is_empty()) else {
        panic!(
            "the embed-plugin feature needs XFC_PLUGIN_XPL, the path of a built xflightcrew.xpl \
             (make -C plugin; or run scripts/build-release.sh)"
        );
    };
    let path = std::path::PathBuf::from(path);
    let bytes = std::fs::read(&path).unwrap_or_else(|e| {
        panic!(
            "embed-plugin: XFC_PLUGIN_XPL={} cannot be read: {e}",
            path.display()
        )
    });
    // The marker `xfc plugin status` compares by (plugin/src/xflightcrew.cpp).
    let needle = b"XFC_PLUGIN_VERSION=";
    assert!(
        bytes.windows(needle.len()).any(|w| w == needle),
        "embed-plugin: {} carries no XFC_PLUGIN_VERSION marker (built before it; rebuild with make -C plugin)",
        path.display()
    );
    let abs = path.canonicalize().unwrap();
    println!("cargo:rerun-if-changed={}", abs.display());
    println!("cargo:rustc-env=XFC_PLUGIN_XPL_FILE={}", abs.display());
}
