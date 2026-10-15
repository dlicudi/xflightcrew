//! Who actually flies where from a field, and how often: small route tables
//! for the fields we fly first (Gibraltar and Malaga, the pilot's approval
//! of 2026-10-13), from which [`crate::rotation`] builds the day's
//! ROTATIONS. A field with no table keeps the hour generator
//! ([`crate::schedule::generate`]).
//!
//! The timetables are DATA (2026-10-14): one YAML file per field in
//! `routes/` (`LXGB.yaml`, `LEMG.yaml`, `EGKK.yaml`), each with its sources
//! as comments, shipped like the profiles (`sim::paths::routes_dir`, built
//! into the binary and unpacked when nothing is on disk) and parsed once
//! ([`fields`]): a new field is a file dropped in, never a rebuild. A file
//! that does not parse is logged and skipped. They were Rust tables until then; Gatwick
//! and the fields after it would have tripled them. A row names its carrier
//! by its constant's name in [`crate::airlines`] (`carrier: EZY_GATWICK`),
//! resolved by [`airlines::carrier_by_name`]: the carriers, their telephony
//! and number ranges stay Rust, pinned to the station by the bin. The
//! compile-time check the tables had is now this module's tests: every
//! file parses, every carrier resolves, every type is one the performance
//! tables know (never a silent 172), the numbers, windows and frequencies
//! are sane, and a route between two table fields is written once. The
//! pilot's own file (`XCOPILOT_TRAFFIC_FILE`) stays the other YAML way in.
//!
//! Gatwick (`EGKK.yaml`, 2026-10-14) is the third table and the first that
//! shares routes: Malaga's and Gibraltar's Gatwick rows are written in
//! their files only, and Gatwick's rows for them are DERIVED ([`derive`]),
//! the same flights seen from the other end ([`crate::rotation`]). Its
//! table holds the busiest routes; the hour generator flies the rest of its
//! rate (`top_up`, [`crate::midflow::top_up`]).
//!
//! A weekly frequency is the route's passengers over a full narrowbody each
//! way (about 150 seated, 85% full) and the 52 weeks, rounded, split
//! between the carriers the article names on it. Flight numbers are
//! illustrative inside each carrier's own range (only BA's are the real
//! ones). Malaga's times are the carriers' LINES OF FLYING ([`LINES`]):
//! each aircraft's day built from the route's block time and turnarounds
//! ([`crate::rotation`]); Gibraltar's rows keep the published slots.

use std::sync::OnceLock;

use serde::Deserialize;

use crate::airlines::{self, Carrier};

/// One route at a field: a carrier's rotation to and from `other`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RouteRow {
    pub carrier: Carrier,
    /// The INBOUND flight number of the day's first rotation (from
    /// `other` to the field); its outbound is the next number, the day's
    /// later rotations count on in twos (BA492 in, BA493 out).
    pub number: u32,
    /// The other end, its ICAO code and the name a board shows.
    pub other: &'static str,
    pub city: &'static str,
    /// Rotations a week in the IATA summer season, and in the winter.
    pub summer: u8,
    pub winter: u8,
    /// The local windows the arrivals fall in, hours: the day's rotations
    /// are spread evenly across them ([`crate::rotation`]). Empty
    /// ([`LINES`]): the carrier's lines of flying decide.
    pub windows: &'static [(f64, f64)],
    /// The first rotation of the day NIGHT-STOPS: its aircraft came in the
    /// evening before and leaves in the morning wave (a based carrier).
    pub based: bool,
    /// The types flown on the route, by weight; the carrier's own when
    /// empty.
    pub types: &'static [(&'static str, u32)],
    /// A SHARED route's far end (2026-10-14): the route is written once,
    /// in `other`'s file, and this row is derived from it ([`derive`]): the
    /// same carrier, numbers and frequencies, its flights the source's seen
    /// from here ([`crate::rotation`] mirrors the source's day: our arrival
    /// is its departure a block time later). `None`: written in this
    /// field's own file.
    pub mirror_of: Option<&'static str>,
}

/// A field's table.
#[derive(Debug, Clone, Copy)]
pub struct FieldRoutes {
    pub icao: &'static str,
    /// Standard time's offset from UTC, hours (Spain and Gibraltar keep
    /// Central European Time, with the EU's summer time on top).
    pub utc_offset_h: f64,
    /// The field's name as a board shows it (another field's derived rows
    /// name it as their city).
    pub city: &'static str,
    /// The hour generator fills what the timetable does not fly
    /// ([`crate::midflow::top_up`]): a table that covers only the busiest
    /// routes opts in, so the field still flies its own rate. Off, the
    /// timetable is the whole day (Gibraltar's and Malaga's are).
    pub top_up: bool,
    pub rows: &'static [RouteRow],
}

/// No windows: the route is flown on its carrier's LINES OF FLYING
/// ([`crate::rotation`]), each aircraft's day from its base's morning wave
/// on, one rotation a block time, a turnaround and a block time back after
/// the last. Until 2026-10-13 Malaga's rows named three windows (the
/// low-cost banks 09:00 to 12:00, 13:30 to 17:00 and 19:00 to 23:00, a hub
/// day 08:30 to 22:30, a shuttle 07:30 to 23:00): live at 08:00 the board
/// showed no arrival from 08:06 to 09:03 and no departure from 08:47 to
/// 09:45, the banks' edges, where the real field is busy all morning. In a
/// route file: a row with no `windows`.
pub const LINES: &[(f64, f64)] = &[];

/// The timetables' folder. This crate's own tests run from its folder, so
/// they read the checkout's.
fn routes_dir() -> std::path::PathBuf {
    if cfg!(test) {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../routes")
    } else {
        sim::paths::routes_dir()
    }
}

/// Every `*.yaml` in `dir`, by file name: (name, text). None for a folder
/// that does not read.
fn route_files(dir: &std::path::Path) -> Vec<(String, String)> {
    let mut files: Vec<(String, String)> = std::fs::read_dir(dir)
        .into_iter()
        .flatten()
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "yaml"))
        .filter_map(|p| {
            let name = p.file_name()?.to_string_lossy().into_owned();
            Some((name, std::fs::read_to_string(&p).ok()?))
        })
        .collect();
    files.sort();
    files
}

/// A route file as written.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FileTable {
    icao: String,
    city: String,
    utc_offset_h: f64,
    #[serde(default)]
    top_up: bool,
    routes: Vec<FileRow>,
}

/// A row as written: the carrier by its constant's name, `windows`,
/// `based` and `types` optional.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FileRow {
    carrier: String,
    number: u32,
    other: String,
    city: String,
    summer: u8,
    winter: u8,
    #[serde(default)]
    windows: Vec<(f64, f64)>,
    #[serde(default)]
    based: bool,
    #[serde(default)]
    types: Vec<(String, u32)>,
}

fn leak_str(s: String) -> &'static str {
    Box::leak(s.into_boxed_str())
}

/// Parses route file `file` into a table. An error names the file and,
/// for a carrier no constant carries, the row. The table's strings and
/// slices are leaked: called once per shipped file ([`fields`]) and by
/// tests, never per tick.
fn parse_table(file: &str, text: &str) -> Result<FieldRoutes, String> {
    let t: FileTable = serde_yaml::from_str(text).map_err(|e| format!("{file}: {e}"))?;
    let mut rows = Vec::with_capacity(t.routes.len());
    for (i, r) in t.routes.into_iter().enumerate() {
        let carrier = airlines::carrier_by_name(&r.carrier).ok_or_else(|| {
            format!(
                "{file}: routes[{i}] ({} {} to {}): no carrier constant is called {}",
                r.carrier, r.number, r.other, r.carrier
            )
        })?;
        let windows: &'static [(f64, f64)] = if r.windows.is_empty() {
            LINES
        } else {
            Box::leak(r.windows.into_boxed_slice())
        };
        let types: &'static [(&'static str, u32)] = if r.types.is_empty() {
            &[]
        } else {
            Box::leak(
                r.types
                    .into_iter()
                    .map(|(t, w)| (leak_str(t), w))
                    .collect::<Vec<_>>()
                    .into_boxed_slice(),
            )
        };
        rows.push(RouteRow {
            carrier,
            number: r.number,
            other: leak_str(r.other),
            city: leak_str(r.city),
            summer: r.summer,
            winter: r.winter,
            windows,
            based: r.based,
            types,
            mirror_of: None,
        });
    }
    Ok(FieldRoutes {
        icao: leak_str(t.icao),
        utc_offset_h: t.utc_offset_h,
        city: leak_str(t.city),
        top_up: t.top_up,
        rows: Box::leak(rows.into_boxed_slice()),
    })
}

/// Every table with the rows other tables share with it (2026-10-14): a
/// route between two table fields is ONE thing, written once in one
/// field's file; the other field's row is DERIVED here from it, after its
/// own rows (so a table no other names keeps its rows and their order,
/// and its day, exactly). A derived row keeps the source's carrier,
/// numbers, frequencies and types; its `other` is the source field, its
/// city the source's name, and its times are the source's flights a block
/// time away ([`crate::rotation`]), so `based` and `windows` are the
/// source's business and unused here. Malaga's EZY8061 row is Gatwick's
/// too: Malaga's departure EZY8062 is Gatwick's arrival EZY8062.
fn derive(tables: Vec<FieldRoutes>) -> Vec<FieldRoutes> {
    let shared: Vec<Vec<RouteRow>> = tables
        .iter()
        .map(|here| {
            tables
                .iter()
                .filter(|src| src.icao != here.icao)
                .flat_map(|src| {
                    src.rows
                        .iter()
                        .filter(|r| r.mirror_of.is_none() && r.other == here.icao)
                        .map(|r| RouteRow {
                            other: src.icao,
                            city: src.city,
                            windows: LINES,
                            based: false,
                            mirror_of: Some(src.icao),
                            ..*r
                        })
                })
                .collect()
        })
        .collect();
    tables
        .into_iter()
        .zip(shared)
        .map(|(t, more)| {
            if more.is_empty() {
                return t;
            }
            let mut rows = t.rows.to_vec();
            rows.extend(more);
            FieldRoutes {
                rows: Box::leak(rows.into_boxed_slice()),
                ..t
            }
        })
        .collect()
}

/// Every field with a route table. The tables are parsed from the
/// timetables' folder on first use and never change after: immutable
/// shipped data, like a const table, never state (the crate's "no
/// statics" is about facts; this holds none).
pub fn fields() -> &'static [FieldRoutes] {
    static TABLES: OnceLock<Vec<FieldRoutes>> = OnceLock::new();
    TABLES.get_or_init(|| {
        derive(
            route_files(&routes_dir())
                .iter()
                .filter_map(|(file, text)| {
                    parse_table(file, text)
                        .inspect_err(|e| tracing::error!("route file skipped: {e}"))
                        .ok()
                })
                .collect(),
        )
    })
}

/// The route table of `icao`, if we fly one there.
pub fn field_routes(icao: &str) -> Option<&'static FieldRoutes> {
    fields().iter().find(|f| f.icao.eq_ignore_ascii_case(icao))
}

/// Every carrier a route table names, for the telephony check
/// ([`crate::airlines::all_carriers`]).
pub fn all_carriers() -> Vec<Carrier> {
    fields()
        .iter()
        .flat_map(|f| f.rows.iter().map(|r| r.carrier))
        .collect()
}

/// An airline's name as a board shows it, and its IATA designator (the
/// board's flight numbers: "U2 8901"). The ICAO code itself for one not
/// listed.
pub fn airline_name(code: &str) -> (&'static str, &'static str) {
    match code {
        "BAW" => ("British Airways", "BA"),
        "EZY" => ("easyJet", "U2"),
        "RYR" => ("Ryanair", "FR"),
        "VLG" => ("Vueling", "VY"),
        "EXS" => ("Jet2", "LS"),
        "TOM" => ("TUI", "BY"),
        "NAX" => ("Norwegian", "DY"),
        "KLM" => ("KLM", "KL"),
        "TRA" => ("Transavia", "HV"),
        "EIN" => ("Aer Lingus", "EI"),
        "BEL" => ("Brussels Airlines", "SN"),
        "SAS" => ("SAS", "SK"),
        "FIN" => ("Finnair", "AY"),
        "AUA" => ("Austrian", "OS"),
        "DLH" => ("Lufthansa", "LH"),
        "EWG" => ("Eurowings", "EW"),
        "SWR" => ("Swiss", "LX"),
        "AFR" => ("Air France", "AF"),
        "IBE" => ("Iberia", "IB"),
        "IBS" => ("Iberia Express", "I2"),
        "AEA" => ("Air Europa", "UX"),
        "ANE" => ("Air Nostrum", "YW"),
        "TAP" => ("TAP Air Portugal", "TP"),
        "THY" => ("Turkish Airlines", "TK"),
        "QTR" => ("Qatar Airways", "QR"),
        "UAL" => ("United", "UA"),
        "ETD" => ("Etihad", "EY"),
        "LOT" => ("LOT", "LO"),
        "ICE" => ("Icelandair", "FI"),
        "AEE" => ("Aegean", "A3"),
        "WZZ" => ("Wizz Air", "W6"),
        "ITY" => ("ITA Airways", "AZ"),
        "SXS" => ("SunExpress", "XQ"),
        "CAI" => ("Corendon", "XC"),
        "KMM" => ("KM Malta Airlines", "KM"),
        "SEH" => ("Sky Express", "GQ"),
        "UBT" => ("Norse Atlantic", "Z0"),
        "CES" => ("China Eastern", "MU"),
        "CCA" => ("Air China", "CA"),
        "AUR" => ("Aurigny", "GR"),
        "UAE" => ("Emirates", "EK"),
        _ => ("", ""),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every shipped file parses, under its own field's name, and no two
    /// files are one field.
    #[test]
    fn every_route_file_parses() {
        let files = route_files(&routes_dir());
        assert!(files.len() >= 3, "{}", routes_dir().display());
        for (file, text) in &files {
            let t = parse_table(file, text).unwrap_or_else(|e| panic!("{e}"));
            assert_eq!(format!("{}.yaml", t.icao), *file);
            assert!(!t.rows.is_empty(), "{file}");
            assert!((-12.0..=14.0).contains(&t.utc_offset_h), "{file}");
        }
        assert_eq!(fields().len(), files.len());
        let mut seen = std::collections::HashSet::new();
        assert!(fields().iter().all(|f| seen.insert(f.icao)));
    }

    /// A file that names a carrier no constant carries, or misspells a
    /// field, fails naming the file and the row.
    #[test]
    fn a_bad_route_file_names_its_file_and_row() {
        let bad = "icao: XXXX\ncity: X\nutc_offset_h: 0.0\nroutes:\n  \
                   - {carrier: EZY, number: 11, other: EGKK, city: A, summer: 7, winter: 7}\n  \
                   - {carrier: EZY_GATWIK, number: 8001, other: EGKK, city: A, summer: 7, winter: 7}\n";
        let e = parse_table("XXXX.yaml", bad).unwrap_err();
        assert!(e.contains("XXXX.yaml") && e.contains("routes[1]"), "{e}");
        assert!(e.contains("EZY_GATWIK"), "{e}");
        let typo = "icao: XXXX\ncity: X\nutc_offset_h: 0.0\nroutes:\n  \
                    - {carrier: EZY, number: 11, other: EGKK, city: A, sumer: 7, winter: 7}\n";
        let e = parse_table("XXXX.yaml", typo).unwrap_err();
        assert!(e.contains("XXXX.yaml") && e.contains("routes[0]"), "{e}");
    }

    /// Every row flies types the performance tables know, inside its
    /// carrier's own number range, to a four-letter ICAO, in windows that
    /// are a day's hours (or on its lines of flying, to a place we know),
    /// and never a frequency of more than seven a day.
    #[test]
    fn every_route_row_is_well_formed() {
        for f in fields() {
            for row in f.rows {
                let at = format!("{} {}{}", f.icao, row.carrier.code, row.number);
                for types in [row.types, row.carrier.types] {
                    for (t, _) in types {
                        assert_ne!(
                            crate::perf::class_for_type(t).0.name,
                            crate::perf::PISTON_SINGLE.name,
                            "{at}: {t}"
                        );
                    }
                }
                let types = if row.types.is_empty() {
                    row.carrier.types
                } else {
                    row.types
                };
                assert!(types.iter().any(|(_, w)| *w > 0), "{at}");
                let (lo, hi) = row.carrier.numbers;
                assert!((lo..=hi).contains(&row.number), "{at} outside {lo}..{hi}");
                assert!(
                    row.other.len() == 4 && row.other.bytes().all(|b| b.is_ascii_uppercase()),
                    "{at}: {}",
                    row.other
                );
                assert_ne!(row.other, f.icao, "{at}");
                assert!(
                    !row.windows.is_empty() || crate::places::position(row.other).is_some(),
                    "{at}: lines of flying need the other end's place"
                );
                assert!(
                    row.windows
                        .iter()
                        .all(|(a, b)| 0.0 <= *a && a < b && *b <= 24.0),
                    "{at}: {:?}",
                    row.windows
                );
                assert!(row.summer <= 42 + 7 && row.winter <= 42 + 7, "{at}");
                assert!(row.summer > 0 || row.winter > 0, "{at}: never flown");
                assert!(!row.city.is_empty(), "{at}");
                assert!(
                    !airline_name(row.carrier.code).0.is_empty(),
                    "{}",
                    row.carrier.code
                );
            }
        }
    }

    /// No two rows of a field share a flight number: two rotations would
    /// fly under one callsign.
    #[test]
    fn flight_numbers_never_collide_at_a_field() {
        for f in fields() {
            let mut seen = std::collections::HashSet::new();
            for row in f.rows {
                let daily = (row.summer.max(row.winter) as u32).div_ceil(7);
                for k in 0..daily {
                    for n in [row.number + 2 * k, row.number + 2 * k + 1] {
                        assert!(
                            seen.insert(format!("{}{n}", row.carrier.code)),
                            "{} {}{n} twice",
                            f.icao,
                            row.carrier.code
                        );
                    }
                }
            }
        }
    }

    /// A route between two table fields is written ONCE (2026-10-14): a
    /// file's own row to another table field whose own file also has a row
    /// back with the same carrier is the route twice, and the two could
    /// disagree. One file writes it; the other end derives it ([`derive`]).
    fn written_twice(tables: &[FieldRoutes]) -> Vec<String> {
        let mut out = Vec::new();
        for a in tables {
            for b in tables.iter().filter(|b| b.icao < a.icao) {
                for ra in a
                    .rows
                    .iter()
                    .filter(|r| r.mirror_of.is_none() && r.other == b.icao)
                {
                    if b.rows.iter().any(|rb| {
                        rb.mirror_of.is_none()
                            && rb.other == a.icao
                            && rb.carrier.code == ra.carrier.code
                    }) {
                        out.push(format!(
                            "{} {}{} to {}: written in both files; write it in one",
                            a.icao, ra.carrier.code, ra.number, b.icao
                        ));
                    }
                }
            }
        }
        out
    }

    /// Every derived row is its source's: the source's own row back to this
    /// field, the same carrier, numbers, frequencies and types.
    fn derived_disagree(tables: &[FieldRoutes]) -> Vec<String> {
        let mut out = Vec::new();
        for t in tables {
            for r in t.rows {
                let Some(src) = r.mirror_of else {
                    continue;
                };
                let ok = tables.iter().filter(|s| s.icao == src).any(|s| {
                    s.rows.iter().any(|sr| {
                        sr.mirror_of.is_none()
                            && sr.other == t.icao
                            && sr.carrier == r.carrier
                            && sr.number == r.number
                            && sr.summer == r.summer
                            && sr.winter == r.winter
                            && sr.types == r.types
                            && r.other == src
                    })
                });
                if !ok {
                    out.push(format!(
                        "{} {}{} from {src}",
                        t.icao, r.carrier.code, r.number
                    ));
                }
            }
        }
        out
    }

    #[test]
    fn a_shared_route_is_written_once_and_derived_at_the_other_end() {
        let bad = written_twice(fields());
        assert!(bad.is_empty(), "{bad:#?}");
        let bad = derived_disagree(fields());
        assert!(bad.is_empty(), "{bad:#?}");
        // Every own row to a table field has its derived row there.
        for a in fields() {
            for r in a.rows.iter().filter(|r| r.mirror_of.is_none()) {
                if let Some(b) = field_routes(r.other) {
                    assert!(
                        b.rows.iter().any(|d| d.mirror_of == Some(a.icao)
                            && d.carrier.code == r.carrier.code
                            && d.number == r.number),
                        "{} {}{} has no row at {}",
                        a.icao,
                        r.carrier.code,
                        r.number,
                        b.icao
                    );
                }
            }
        }
    }

    /// The rule bites: a route written at both ends is refused, and the
    /// derived row is the source's, its far end the source field.
    #[test]
    fn the_cross_field_rule_catches_a_route_written_twice() {
        let lxgb = *field_routes("LXGB").unwrap();
        let ezy = lxgb.rows.iter().find(|r| r.other == "EGKK").unwrap();
        let table = |rows: Vec<RouteRow>| FieldRoutes {
            icao: "EGKK",
            utc_offset_h: 0.0,
            city: "London Gatwick",
            top_up: false,
            rows: Box::leak(rows.into_boxed_slice()),
        };
        let twice = RouteRow {
            other: "LXGB",
            city: "Gibraltar",
            number: ezy.number + 1,
            ..*ezy
        };
        assert!(!written_twice(&[lxgb, table(vec![twice])]).is_empty());
        let derived = derive(vec![lxgb, table(vec![])]);
        let d = derived[1].rows;
        assert_eq!(
            d.len(),
            lxgb.rows.iter().filter(|r| r.other == "EGKK").count()
        );
        assert_eq!(d[0].mirror_of, Some("LXGB"));
        assert_eq!((d[0].other, d[0].city), ("LXGB", "Gibraltar"));
        assert_eq!(
            (d[0].number, d[0].summer, d[0].winter),
            (ezy.number, ezy.summer, ezy.winter)
        );
        assert_eq!(derived[0].rows, lxgb.rows, "the source keeps its rows");
        assert!(written_twice(&derived).is_empty());
        assert!(derived_disagree(&derived).is_empty());
        // A derived row that strays from its source is caught.
        let mut strayed = d.to_vec();
        strayed[0].summer += 1;
        assert!(!derived_disagree(&[lxgb, table(strayed)]).is_empty());
    }

    /// The generated day of every field, pinned: an edit to a route file
    /// (or to the rotations) that moves a flight changes these, so it is
    /// made on purpose and the pin updated with it. Pinned 2026-10-14 from
    /// the day the Rust tables generated, proved equal to the files' in
    /// the commit that moved them (`the_route_files_equal_the_rust_tables`,
    /// `the_route_files_fly_the_same_day_as_the_rust_tables`). Gatwick's
    /// pinned 2026-10-14 with its table, Gibraltar's and Malaga's unchanged
    /// by it.
    #[test]
    fn the_generated_days_are_pinned() {
        use crate::rotation::{day_rotations, Date};
        use crate::schedule::Density;
        let fnv = |s: &str| {
            s.bytes().fold(0xCBF2_9CE4_8422_2325u64, |h, b| {
                (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01B3)
            })
        };
        let mut got = Vec::new();
        for f in fields() {
            for (y, m, d) in [(2026, 7, 15), (2026, 7, 18), (2026, 1, 14), (2026, 1, 17)] {
                let day = day_rotations(f, Date::ymd(y, m, d), 3, Density::Normal);
                let first = day
                    .first()
                    .map(|r| r.arr.callsign.clone())
                    .unwrap_or_default();
                got.push(format!(
                    "{} {y}-{m:02}-{d:02} {} {first} {:016x}",
                    f.icao,
                    day.len(),
                    fnv(&format!("{day:?}"))
                ));
            }
        }
        let want = [
            "EGKK 2026-07-15 220 EZY8902 d615124dfa988254",
            "EGKK 2026-07-18 227 EZY8904 40265b14ebaf1785",
            "EGKK 2026-01-14 125 EXS618 d4774891337865b5",
            "EGKK 2026-01-17 130 EZY8066 021370be12c51c71",
            "LEMG 2026-07-15 224 NAX3586 ba3692f54e0ab81f",
            "LEMG 2026-07-18 227 NAX3586 82b1c932f8bd0438",
            "LEMG 2026-01-14 154 NAX3586 6eeac15b260dfe52",
            "LEMG 2026-01-17 170 NAX3586 d2bf3149f7b602af",
            "LXGB 2026-07-15 7 BAW490 0d97bda92a9614ee",
            "LXGB 2026-07-18 4 BAW490 6fae55dc3f7a571e",
            "LXGB 2026-01-14 4 BAW490 e6bd1df68761b9a1",
            "LXGB 2026-01-17 3 EZY8901 3bb8c9c69d5e1275",
        ];
        assert_eq!(got, want, "{got:#?}");
    }

    #[test]
    fn gibraltar_is_british_airways_and_easyjet() {
        let t = field_routes("lxgb").unwrap();
        assert!(t
            .rows
            .iter()
            .all(|r| ["BAW", "EZY"].contains(&r.carrier.code)));
        assert!(field_routes("EGLL").is_none());
    }
}
