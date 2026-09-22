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
}
