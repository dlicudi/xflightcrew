//! Build script: stamp the git build id into the binary as the `XCOPILOT_BUILD`
//! env var, so the app's startup log and status panel show exactly which build is
//! running — no doubt about whether a rebuilt copilot was actually picked up.

use std::process::Command;

fn main() {
    let sha = Command::new("git")
        .args(["rev-parse", "--short", "HEAD"])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| "unknown".to_string());
    // Mark a dirty tree (uncommitted changes) so a not-yet-committed build is
    // distinguishable from the committed one it's based on.
    let dirty = Command::new("git")
        .args(["diff", "--quiet"])
        .status()
        .map(|s| !s.success())
        .unwrap_or(false);
    let build = if dirty { format!("{sha}-dirty") } else { sha };
    println!("cargo:rustc-env=XCOPILOT_BUILD={build}");
    // Re-run when the commit or the working tree changes so the stamp stays current.
    println!("cargo:rerun-if-changed=.git/HEAD");
    println!("cargo:rerun-if-changed=.git/index");
}
