//! Who actually flies where from a field, and how often: small route tables
//! for the fields we fly first (Gibraltar and Malaga, the pilot's approval
//! of 2026-10-13), from which [`crate::rotation`] builds the day's
//! ROTATIONS. A field with no table keeps the hour generator
//! ([`crate::schedule::generate`]).
//!
//! The timetables are DATA (2026-10-14): one YAML file per field in
//! `crates/traffic/routes/` (`LXGB.yaml`, `LEMG.yaml`), each with its
//! sources as comments, built into the crate by `include_str!` and parsed
//! once ([`fields`]), so nothing is read from disk or the network and the
//! binary needs nothing beside it. They were Rust tables until then; Gatwick
//! and the fields after it would have tripled them. A row names its carrier
//! by its constant's name in [`crate::airlines`] (`carrier: EZY_GATWICK`),
//! resolved by [`airlines::carrier_by_name`]: the carriers, their telephony
//! and number ranges stay Rust, pinned to the station by the bin. The
//! compile-time check the tables had is now this module's tests: every
//! file parses, every carrier resolves, every type is one the performance
//! tables know (never a silent 172), the numbers, windows and frequencies
//! are sane, and two fields that list each other agree. The pilot's own
//! file (`XCOPILOT_TRAFFIC_FILE`) stays the other YAML way in.
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
}

/// A field's table.
#[derive(Debug, Clone, Copy)]
pub struct FieldRoutes {
    pub icao: &'static str,
    /// Standard time's offset from UTC, hours (Spain and Gibraltar keep
    /// Central European Time, with the EU's summer time on top).
    pub utc_offset_h: f64,
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

/// The route files built into the crate, by file name, in [`fields`]'s
/// order.
const FILES: &[(&str, &str)] = &[
    ("LXGB.yaml", include_str!("../routes/LXGB.yaml")),
    ("LEMG.yaml", include_str!("../routes/LEMG.yaml")),
];

/// A route file as written.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FileTable {
    icao: String,
    utc_offset_h: f64,
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
        });
    }
    Ok(FieldRoutes {
        icao: leak_str(t.icao),
        utc_offset_h: t.utc_offset_h,
        rows: Box::leak(rows.into_boxed_slice()),
    })
}

/// Every field with a route table. The tables are parsed from the files
/// built into the crate on first use and never change after: immutable
/// shipped data, like a const table, never state (the crate's "no
/// statics" is about facts; this holds none).
pub fn fields() -> &'static [FieldRoutes] {
    static TABLES: OnceLock<Vec<FieldRoutes>> = OnceLock::new();
    TABLES.get_or_init(|| {
        FILES
            .iter()
            .map(|(file, text)| {
                parse_table(file, text)
                    .unwrap_or_else(|e| panic!("a shipped route file: {e} (see routes.rs tests)"))
            })
            .collect()
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
        for (file, text) in FILES {
            let t = parse_table(file, text).unwrap_or_else(|e| panic!("{e}"));
            assert_eq!(format!("{}.yaml", t.icao), *file);
            assert!(!t.rows.is_empty(), "{file}");
            assert!((-12.0..=14.0).contains(&t.utc_offset_h), "{file}");
        }
        assert_eq!(fields().len(), FILES.len());
        let mut seen = std::collections::HashSet::new();
        assert!(fields().iter().all(|f| seen.insert(f.icao)));
    }

    /// A file that names a carrier no constant carries, or misspells a
    /// field, fails naming the file and the row.
    #[test]
    fn a_bad_route_file_names_its_file_and_row() {
        let bad = "icao: XXXX\nutc_offset_h: 0.0\nroutes:\n  \
                   - {carrier: EZY, number: 11, other: EGKK, city: A, summer: 7, winter: 7}\n  \
                   - {carrier: EZY_GATWIK, number: 8001, other: EGKK, city: A, summer: 7, winter: 7}\n";
        let e = parse_table("XXXX.yaml", bad).unwrap_err();
        assert!(e.contains("XXXX.yaml") && e.contains("routes[1]"), "{e}");
        assert!(e.contains("EZY_GATWIK"), "{e}");
        let typo = "icao: XXXX\nutc_offset_h: 0.0\nroutes:\n  \
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

    /// Two fields with tables that list each other fly ONE route: the
    /// rows at each end name the same carrier and the same frequencies,
    /// and one end's first outbound is the other's first inbound (a row's
    /// `number` is its inbound and `number + 1` its outbound, so the two
    /// rows' numbers are one apart, either way round). Vacuous until two
    /// table fields list each other (Gatwick and Malaga, the first).
    fn ends_disagree(tables: &[FieldRoutes]) -> Vec<String> {
        let mut out = Vec::new();
        for a in tables {
            for b in tables.iter().filter(|b| b.icao != a.icao) {
                for ra in a.rows.iter().filter(|r| r.other == b.icao) {
                    let agrees = b.rows.iter().any(|rb| {
                        rb.other == a.icao
                            && rb.carrier.code == ra.carrier.code
                            && (rb.number == ra.number + 1 || ra.number == rb.number + 1)
                            && rb.summer == ra.summer
                            && rb.winter == ra.winter
                    });
                    if !agrees {
                        out.push(format!(
                            "{} {}{} from {}: no matching row at {} ({}{} in, {} and {} a week)",
                            a.icao,
                            ra.carrier.code,
                            ra.number,
                            b.icao,
                            b.icao,
                            ra.carrier.code,
                            ra.number + 1,
                            ra.summer,
                            ra.winter
                        ));
                    }
                }
            }
        }
        out
    }

    #[test]
    fn two_table_fields_agree_on_the_route_between_them() {
        let bad = ends_disagree(fields());
        assert!(bad.is_empty(), "{bad:#?}");
    }

    /// The cross-field rule bites: a route listed at one end only, or with
    /// another frequency or number at the other, is caught.
    #[test]
    fn the_cross_field_rule_catches_a_one_sided_route() {
        let lxgb = field_routes("LXGB").unwrap();
        let ezy = lxgb.rows.iter().find(|r| r.other == "EGKK").unwrap();
        let gatwick_row = |number: u32, summer: u8| RouteRow {
            carrier: airlines::carrier_by_name("EZY_GATWICK").unwrap(),
            number,
            other: "LXGB",
            city: "Gibraltar",
            summer,
            winter: ezy.winter,
            windows: LINES,
            based: false,
            types: &[],
        };
        let table = |rows: Vec<RouteRow>| FieldRoutes {
            icao: "EGKK",
            utc_offset_h: 0.0,
            rows: Box::leak(rows.into_boxed_slice()),
        };
        assert!(!ends_disagree(&[*lxgb, table(vec![])]).is_empty());
        assert!(
            !ends_disagree(&[*lxgb, table(vec![gatwick_row(ezy.number, ezy.summer)])]).is_empty()
        );
        assert!(!ends_disagree(&[
            *lxgb,
            table(vec![gatwick_row(ezy.number + 1, ezy.summer + 1)])
        ])
        .is_empty());
        let both = ends_disagree(&[*lxgb, table(vec![gatwick_row(ezy.number + 1, ezy.summer)])]);
        assert!(both.is_empty(), "{both:#?}");
    }

    /// The generated day of every field, pinned: an edit to a route file
    /// (or to the rotations) that moves a flight changes these, so it is
    /// made on purpose and the pin updated with it. Pinned 2026-10-14 from
    /// the day the Rust tables generated, proved equal to the files' in
    /// the commit that moved them (`the_route_files_equal_the_rust_tables`,
    /// `the_route_files_fly_the_same_day_as_the_rust_tables`).
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
            "LXGB 2026-07-15 7 BAW490 0d97bda92a9614ee",
            "LXGB 2026-07-18 4 BAW490 6fae55dc3f7a571e",
            "LXGB 2026-01-14 4 BAW490 e6bd1df68761b9a1",
            "LXGB 2026-01-17 3 EZY8901 3bb8c9c69d5e1275",
            "LEMG 2026-07-15 224 NAX3586 ba3692f54e0ab81f",
            "LEMG 2026-07-18 227 NAX3586 82b1c932f8bd0438",
            "LEMG 2026-01-14 154 NAX3586 6eeac15b260dfe52",
            "LEMG 2026-01-17 170 NAX3586 d2bf3149f7b602af",
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
        assert!(field_routes("EGKK").is_none());
    }
}
