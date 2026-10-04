//! Helpers for the tests that pin a rule to the SOURCE (a call before
//! another, a name written once). Such a scan must survive its module
//! becoming a directory (`x.rs` plus `x/*.rs`): read through
//! [`module_source`], never `include_str!("x.rs")`, which goes silently
//! vacuous the day the code it pins moves to a submodule.

#![cfg(test)]
#![allow(dead_code)] // shared by the bin and the lib; each uses a part

use std::path::{Path, PathBuf};

/// The checkout's root (the crew package).
pub fn root() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR"))
}

/// A test-only source file: `tests.rs`, `*_tests.rs` or anything under a
/// `tests/` directory. A test module moved to its own file carries no
/// `#[cfg(test)]` of its own, so a scan of the live code skips it by name.
pub fn is_test_file(p: &Path) -> bool {
    let name = p.file_name().and_then(|n| n.to_str()).unwrap_or("");
    name == "tests.rs"
        || name.ends_with("_tests.rs")
        || p.components().any(|c| c.as_os_str() == "tests")
}

/// Every `.rs` file under `dir`, recursively, in path order.
pub fn rust_files(dir: &Path) -> Vec<PathBuf> {
    fn walk(dir: &Path, out: &mut Vec<PathBuf>) {
        let Ok(rd) = std::fs::read_dir(dir) else {
            return;
        };
        for e in rd.flatten() {
            let p = e.path();
            if p.is_dir() {
                walk(&p, out);
            } else if p.extension().is_some_and(|x| x == "rs") {
                out.push(p);
            }
        }
    }
    let mut out = Vec::new();
    walk(dir, &mut out);
    out.sort();
    out
}

/// The files of the module at `src/<name>` (`"full_auto"`, `"tools/setup"`,
/// `"main"`): `src/<name>.rs` first, then every `.rs` under `src/<name>/` in
/// path order (its submodules and moved-out tests).
pub fn module_files(name: &str) -> Vec<PathBuf> {
    let src = root().join("src");
    let mut out = Vec::new();
    let file = src.join(format!("{name}.rs"));
    if file.is_file() {
        out.push(file);
    }
    out.extend(rust_files(&src.join(name)));
    assert!(!out.is_empty(), "no source for the module {name}");
    out
}

/// The whole text of a module, as [`module_files`] lists it, joined. The
/// parent comes first, so its own text keeps its order and the first match
/// of a pattern in the live code is still the first.
pub fn module_source(name: &str) -> &'static str {
    let text = module_files(name)
        .iter()
        .map(|p| std::fs::read_to_string(p).unwrap())
        .collect::<Vec<_>>()
        .join("\n");
    Box::leak(text.into_boxed_str())
}
