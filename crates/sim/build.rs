//! Build script: the shipped resources (`profiles/`, `checklists/`,
//! `assets/`) built into the binary, so a bare `xfc` copied anywhere needs
//! nothing beside it (docs/tasks/embedded-resources/BRIEF.md). `sim::paths`
//! unpacks them when no resources are on disk.
//!
//! A generated table of `include_bytes!` rather than a crate such as
//! `include_dir`: no new dependency, the build id is a hash of exactly the
//! bytes embedded, and a missing or empty folder FAILS the build here, where
//! a macro would embed an empty tree without a word.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};

/// The folders that ship, relative to the workspace root.
const FOLDERS: &[&str] = &["profiles", "checklists", "assets"];

/// Files a build must carry: a clean checkout without them is a broken
/// release, never a quiet one (the AFV clicks were once gitignored).
const REQUIRED: &[&str] = &[
    "assets/afv/Click_f32.wav",
    "assets/afv/Crackle_f32.wav",
    "assets/afv/AFV_COPYING.md",
];

fn walk(dir: &Path, out: &mut Vec<PathBuf>) {
    let entries = std::fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("embedded resources: cannot read {}: {e}", dir.display()));
    for e in entries.flatten() {
        let p = e.path();
        let name = e.file_name();
        if name.to_string_lossy().starts_with('.') {
            continue;
        }
        if p.is_dir() {
            println!("cargo:rerun-if-changed={}", p.display());
            walk(&p, out);
        } else if p.is_file() {
            out.push(p);
        }
    }
}

/// FNV-1a, 64 bit: enough to tell two embedded trees apart.
fn fnv(h: &mut u64, bytes: &[u8]) {
    for b in bytes {
        *h ^= u64::from(*b);
        *h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
}

fn main() {
    let manifest = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let root = manifest.join("../..").canonicalize().unwrap();
    let mut files = Vec::new();
    for f in FOLDERS {
        let dir = root.join(f);
        println!("cargo:rerun-if-changed={}", dir.display());
        assert!(
            dir.is_dir(),
            "embedded resources: {} is missing; the build will not embed an empty tree",
            dir.display()
        );
        let before = files.len();
        walk(&dir, &mut files);
        assert!(
            files.len() > before,
            "embedded resources: {} is empty; the build will not embed an empty tree",
            dir.display()
        );
    }
    for r in REQUIRED {
        assert!(
            root.join(r).is_file(),
            "embedded resources: {r} is missing (commit it: git add -f {r})"
        );
    }
    files.sort();
    let mut hash = 0xcbf2_9ce4_8422_2325u64;
    let mut table = String::from("pub static FILES: &[(&str, &[u8])] = &[\n");
    for f in &files {
        println!("cargo:rerun-if-changed={}", f.display());
        let rel = f
            .strip_prefix(&root)
            .unwrap()
            .to_string_lossy()
            .replace('\\', "/");
        let bytes = std::fs::read(f).unwrap();
        fnv(&mut hash, rel.as_bytes());
        fnv(&mut hash, &(bytes.len() as u64).to_le_bytes());
        fnv(&mut hash, &bytes);
        writeln!(
            table,
            "    ({rel:?}, include_bytes!({:?})),",
            f.display().to_string()
        )
        .unwrap();
    }
    table.push_str("];\n");
    writeln!(table, "pub const ID: &str = \"{hash:016x}\";").unwrap();
    let out = PathBuf::from(std::env::var("OUT_DIR").unwrap()).join("embedded_resources.rs");
    std::fs::write(out, table).unwrap();
}
