    use std::collections::HashSet;
    use std::path::{Path, PathBuf};

    /// Profiles to audit, and the aircraft folder each is written against.
    const AUDIT: &[(&str, &str)] = &[
        ("airbus_a330_300", "Laminar Research/Airbus A330-300"),
        ("boeing_737_800", "Laminar Research/Boeing 737-800"),
        ("md82", "Laminar Research/McDonnell Douglas MD-82"),
        ("citation_x", "Laminar Research/Cessna Citation X"),
        ("cirrus_vision_sf50", "Laminar Research/Cirrus Vision SF50"),
        ("king_air_c90b", "Laminar Research/Beechcraft King Air C90B"),
        ("dhc6_twin_otter", "Laminar Research/DHC-6 Twin Otter"),
        ("lancair_evolution", "Laminar Research/Lancair Evolution"),
    ];

    fn lua_files(dir: &Path, out: &mut Vec<PathBuf>) {
        let Ok(rd) = std::fs::read_dir(dir) else {
            return;
        };
        for e in rd.flatten() {
            let p = e.path();
            if p.is_dir() {
                lua_files(&p, out);
            } else if p.extension().is_some_and(|x| x == "lua") {
                out.push(p);
            }
        }
    }

    /// Stock datarefs the aircraft ASSIGNS TO — anything it writes each frame
    /// is its output, and ours is lost.
    fn aircraft_owned(src: &str) -> HashSet<String> {
        let mut handles = Vec::new();
        let mut rest = src;
        while let Some(i) = rest.find("= find_dataref(\"sim/") {
            let before = &rest[..i];
            let handle = before
                .rsplit(|c: char| c.is_whitespace())
                .find(|t| !t.is_empty())
                .unwrap_or("")
                .to_string();
            let body = &rest[i + "= find_dataref(\"".len()..];
            if let Some(end) = body.find('"') {
                handles.push((handle, body[..end].to_string()));
            }
            rest = &rest[i + 4..];
        }
        handles
            .into_iter()
            .filter(|(h, _)| {
                !h.is_empty() && (src.contains(&format!("{h} =")) || src.contains(&format!("{h}[")))
            })
            .map(|(_, n)| n)
            .collect()
    }

    fn replaced(src: &str) -> HashSet<String> {
        let mut out = HashSet::new();
        let mut rest = src;
        while let Some(i) = rest.find("replace_command(\"") {
            let body = &rest[i + "replace_command(\"".len()..];
            if let Some(end) = body.find('"') {
                out.insert(body[..end].to_string());
            }
            rest = &rest[i + 4..];
        }
        out
    }

    #[test]
    #[ignore]
    fn profile_writes_are_not_silently_owned_by_the_aircraft() {
        // aircraft.rs compiles into the LIB too, where `tools` is the bin's.
        let root = std::env::var("XCOPILOT_XPLANE_ROOT").unwrap_or_else(|_| {
            format!("{}/X-Plane 12", std::env::var("HOME").unwrap_or_default())
        });
        let mut findings = Vec::new();
        let mut audited = 0usize;
        for (id, folder) in AUDIT {
            let dir = Path::new(&root).join("Aircraft").join(folder);
            if !dir.exists() {
                continue;
            }
            let Some(p) = super::load(id) else { continue };
            let mut files = Vec::new();
            lua_files(&dir, &mut files);
            if files.is_empty() {
                continue; // no xlua: nothing can own anything
            }
            audited += 1;
            let src: String = files
                .iter()
                .filter_map(|f| std::fs::read_to_string(f).ok())
                .collect::<Vec<_>>()
                .join("\n");
            // OWNERSHIP is the systems model rewriting a dataref every frame.
            // The aircraft's own AUTOSTART script also assigns to some of
            // them — but that is a script pressing switches, exactly as we
            // do, and it proves the write lands rather than that it is lost.
            // Scanning it flagged the A330's engine masters and ENG MODE
            // knob, both of which demonstrably work (2026-09-20).
            let systems: String = files
                .iter()
                .filter(|f| {
                    !f.to_string_lossy()
                        .to_ascii_lowercase()
                        .contains("autostart")
                })
                .filter_map(|f| std::fs::read_to_string(f).ok())
                .collect::<Vec<_>>()
                .join("\n");
            let owned = aircraft_owned(&systems);
            let repl = replaced(&src);

            // Commands the profile FIRES. A stock command the aircraft
            // neither replaces nor needs is fine; one whose SIBLING is
            // replaced while this one is not is the trap.
            for e in &p.electrical {
                for c in e.on_commands.iter().chain(e.off_commands.iter()) {
                    if !c.starts_with("sim/") || repl.contains(c) {
                        continue;
                    }
                    let sibling = c.rsplit_once('_').map(|(stem, _)| format!("{stem}_toggle"));
                    if sibling.is_some_and(|s| repl.contains(&s)) {
                        findings.push(format!(
                            "{id}: fires {c}, which the aircraft leaves stock while REPLACING \
                             its _toggle — use the toggle with a read-before-fire"
                        ));
                    }
                }
            }
            // Datarefs the profile WRITES that the aircraft owns.
            if let Some(d) = &p.pitot_heat_dataref {
                if owned.contains(d) {
                    findings.push(format!("{id}: writes {d}, which the aircraft rewrites"));
                }
            }
            // Written directly and OBSERVED to work: the A330's engines
            // started on 2026-09-20. The switches script assigns these only
            // inside its own knob handlers, which this scan cannot tell from
            // a per-frame rewrite. Evidence beats the heuristic; anything NEW
            // still has to be justified here.
            const PROVEN: &[&str] = &["sim/cockpit2/engine/actuators/eng_mode_selector"];
            if let Some(a) = &p.airliner_start {
                for n in [&a.mode_selector, &a.masters] {
                    if PROVEN.contains(&n.as_str()) {
                        continue;
                    }
                    if owned.contains(n) {
                        findings.push(format!("{id}: writes {n}, which the aircraft rewrites"));
                    }
                }
            }
        }
        assert!(audited > 0, "no audited aircraft found under {root}");
        assert!(
            findings.is_empty(),
            "silent writes:\n  {}",
            findings.join("\n  ")
        );
    }

