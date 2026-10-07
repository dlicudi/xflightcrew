    use std::collections::HashSet;
    use std::path::{Path, PathBuf};

    /// Literals that are deliberately not names: prefixes the code lists by,
    /// and a test fixture that must NOT exist.
    const NOT_NAMES: &[&str] = &["sim/operation/failures/rel_whatsit_thing"];

    fn rust_files(dir: &Path, out: &mut Vec<PathBuf>) {
        let Ok(rd) = std::fs::read_dir(dir) else {
            return;
        };
        for e in rd.flatten() {
            let p = e.path();
            if p.is_dir() {
                if p.file_name().is_some_and(|n| n == "target") {
                    continue;
                }
                rust_files(&p, out);
            } else if p.extension().is_some_and(|x| x == "rs") {
                out.push(p);
            }
        }
    }

    /// `"sim/..."` string literals in one source file, array suffix stripped.
    fn sim_literals(src: &str) -> Vec<String> {
        let mut out = Vec::new();
        let mut rest = src;
        while let Some(i) = rest.find("\"sim/") {
            let body = &rest[i + 1..];
            let Some(end) = body.find('"') else { break };
            let lit = &body[..end];
            let name = match lit.find('[') {
                Some(b) => &lit[..b],
                None => lit,
            };
            if !name.ends_with('_')
                && !name.ends_with('/')
                && name
                    .chars()
                    .all(|c| c.is_ascii_alphanumeric() || c == '/' || c == '_')
            {
                out.push(name.to_string());
            }
            rest = &body[end + 1..];
        }
        out
    }

    fn first_column(path: &Path) -> HashSet<String> {
        std::fs::read_to_string(path)
            .unwrap_or_else(|e| panic!("{}: {e}", path.display()))
            .lines()
            .filter_map(|l| l.split(['\t', ' ']).next())
            .filter(|n| n.starts_with("sim/"))
            .map(str::to_string)
            .collect()
    }

    #[test]
    #[ignore]
    fn every_hardcoded_sim_name_exists_in_the_install() {
        // Same resolution as `crate::config::xplane_root` (a bin-side
        // function; this module also compiles into the lib).
        let root = PathBuf::from(std::env::var("XCOPILOT_XPLANE_ROOT").unwrap_or_else(|_| {
            format!("{}/X-Plane 12", std::env::var("HOME").unwrap_or_default())
        }));
        let plugins = root.join("Resources").join("plugins");
        let mut known = first_column(&plugins.join("DataRefs.txt"));
        known.extend(first_column(&plugins.join("Commands.txt")));
        assert!(
            known.len() > 1000,
            "DataRefs.txt looks empty: {}",
            known.len()
        );

        let repo = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let mut files = Vec::new();
        rust_files(&repo.join("src"), &mut files);
        rust_files(&repo.join("crates"), &mut files);
        assert!(files.len() > 20, "found only {} source files", files.len());

        let mut missing = Vec::new();
        for f in &files {
            let src = std::fs::read_to_string(f).unwrap();
            for name in sim_literals(&src) {
                if !known.contains(&name) && !NOT_NAMES.contains(&name.as_str()) {
                    missing.push(format!(
                        "{} <- {}",
                        name,
                        f.strip_prefix(&repo).unwrap().display()
                    ));
                }
            }
        }
        missing.sort();
        missing.dedup();
        assert!(
            missing.is_empty(),
            "names the sim does not have:\n{}",
            missing.join("\n")
        );
    }

    #[test]
    fn literal_scan_strips_indexes_and_skips_prefixes() {
        // Built from pieces so the fixture's names are not themselves
        // literals the install test would scan.
        let q = '"';
        let src = format!("const A: &str = {q}sim/a/b{q}; x({q}sim/c/d[3]{q}); list({q}sim/e/rel_{q}); t({q}sim/f/g/{q});");
        let want: Vec<String> = ["a/b", "c/d"].iter().map(|t| format!("sim/{t}")).collect();
        assert_eq!(sim_literals(&src), want);
    }
