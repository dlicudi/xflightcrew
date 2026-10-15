//! Who actually flies where from a field, and how often: route tables from
//! which [`crate::rotation`] builds the day's ROTATIONS. A field with no
//! table keeps the hour generator ([`crate::schedule::generate`]).
//!
//! The timetables are DATA, shipped like the profiles
//! (`sim::paths::routes_dir`, built into the binary and unpacked when
//! nothing is on disk) and parsed once ([`fields`]): `routes/fields.yaml`
//! names every field a route names (its board name, its place, its clock,
//! [`crate::places`]), and `routes/carriers/<CODE>.yaml` holds one operator
//! each (its name, IATA designator, telephony, types, numbers and routes).
//! Until 2026-10-15 they were one file per field with the carriers, places
//! and clocks as Rust tables; none of those scaled past a handful of fields
//! (docs/tasks/global-routes). A carrier file that does not parse is logged
//! and skipped. The tests check what the compiler once did: every file
//! parses, every type is one the performance tables know (never a silent
//! 172), and the numbers, windows and frequencies are sane. The pilot's own
//! file (`XCOPILOT_TRAFFIC_FILE`) stays the other YAML way in.
//!
//! A route row is written for one field, its `at`, and a carrier writes a
//! pair from one end only (a row of the same carrier back from `other` to
//! `at` is a load error naming both; more rows the same way are one route
//! flown as several, BA's two Heathrow slots at Gibraltar). A field that is some row's `at` has a table; a row
//! whose `other` has a table too is that table's as well, DERIVED
//! ([`derive`]): the same flights seen from the other end
//! ([`crate::rotation`]). Gatwick's table holds its busiest routes; the
//! hour generator flies the rest of its rate (`top_up`,
//! [`crate::midflow::top_up`]).
//!
//! A weekly frequency is the route's passengers over a full narrowbody each
//! way (about 150 seated, 85% full) and the 52 weeks, rounded, split
//! between the carriers the article names on it. Flight numbers are
//! illustrative inside each carrier's own range (only BA's are the real
//! ones). Malaga's times are the carriers' LINES OF FLYING ([`LINES`]):
//! each aircraft's day built from the route's block time and turnarounds
//! ([`crate::rotation`]); Gibraltar's rows keep the published slots.

use std::collections::{BTreeMap, HashMap};
use std::sync::OnceLock;

use serde::Deserialize;

use crate::airlines::Carrier;

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
    /// A SHARED route's far end (2026-10-14): the route is written for
    /// `other`, and this row is derived from it ([`derive`]): the same
    /// carrier, numbers and frequencies, its flights the source's seen
    /// from here ([`crate::rotation`] mirrors the source's day: our arrival
    /// is its departure a block time later). `None`: written for this
    /// field.
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

/// A field a route names, as `routes/fields.yaml` writes it: its board
/// name, its airport reference point (degrees) and its clock (standard
/// time's offset from UTC, hours, and whether it keeps summer time).
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Place {
    pub icao: String,
    pub city: String,
    pub lat: f64,
    pub lon: f64,
    pub utc_offset_h: f64,
    pub dst: bool,
    #[serde(default)]
    pub top_up: bool,
}

/// An operator's file, `routes/carriers/<CODE>.yaml`.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FileCarrier {
    code: String,
    name: String,
    iata: String,
    telephony: String,
    types: Vec<(String, u32)>,
    numbers: (u32, u32),
    routes: Vec<FileRow>,
}

/// A row as written: `windows`, `based` and `types` optional.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FileRow {
    at: String,
    other: String,
    number: u32,
    summer: u8,
    winter: u8,
    #[serde(default)]
    windows: Vec<(f64, f64)>,
    #[serde(default)]
    based: bool,
    #[serde(default)]
    types: Vec<(String, u32)>,
}

/// An operator as a board shows it: "easyJet", "U2".
struct Airline {
    carrier: Carrier,
    name: &'static str,
    iata: &'static str,
}

/// Everything the route data holds, parsed once ([`data`]).
#[derive(Default)]
struct Data {
    places: Vec<Place>,
    airlines: Vec<Airline>,
    telephony: Vec<(&'static str, &'static str)>,
    tables: Vec<FieldRoutes>,
}

/// The timetables' folder. This crate's own tests run from its folder, so
/// they read the checkout's.
fn routes_dir() -> std::path::PathBuf {
    if cfg!(test) {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../routes")
    } else {
        sim::paths::routes_dir()
    }
}

/// Every `*.yaml` in `dir`, by file name: (name, text).
fn yaml_files(dir: &std::path::Path) -> Vec<(String, String)> {
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

fn leak_str(s: String) -> &'static str {
    Box::leak(s.into_boxed_str())
}

fn leak_types(types: Vec<(String, u32)>) -> &'static [(&'static str, u32)] {
    if types.is_empty() {
        return &[];
    }
    let v: Vec<_> = types.into_iter().map(|(t, w)| (leak_str(t), w)).collect();
    Box::leak(v.into_boxed_slice())
}

/// The route data from `fields.yaml`'s text and the carrier files (name,
/// text), and every error: a file that does not parse is skipped, a row
/// whose `at` or `other` is no field, or that writes its carrier's pair
/// from the other end too, is dropped, each named by its file and row.
/// Strings and slices are leaked: called once ([`data`]) and by tests,
/// never per tick.
fn load(fields: &str, carriers: &[(String, String)]) -> (Data, Vec<String>) {
    let mut errors = Vec::new();
    let places: Vec<Place> = match serde_yaml::from_str(fields) {
        Ok(p) => p,
        Err(e) => return (Data::default(), vec![format!("fields.yaml: {e}")]),
    };
    let by_icao: HashMap<&str, &Place> = places.iter().map(|p| (p.icao.as_str(), p)).collect();
    let place = |icao: &str| by_icao.get(icao).copied();
    let mut airlines = Vec::new();
    let mut rows: BTreeMap<String, Vec<RouteRow>> = BTreeMap::new();
    let mut seen: HashMap<(String, String, String), String> = HashMap::new();
    for (file, text) in carriers {
        let c: FileCarrier = match serde_yaml::from_str(text) {
            Ok(c) => c,
            Err(e) => {
                errors.push(format!("{file}: {e}"));
                continue;
            }
        };
        let carrier = Carrier {
            code: leak_str(c.code),
            telephony: leak_str(c.telephony),
            types: leak_types(c.types),
            numbers: c.numbers,
        };
        for (i, r) in c.routes.into_iter().enumerate() {
            let here = format!(
                "{file} routes[{i}] ({} {} to {})",
                carrier.code, r.at, r.other
            );
            let (Some(_), Some(other)) = (place(&r.at), place(&r.other)) else {
                errors.push(format!("{here}: not in fields.yaml"));
                continue;
            };
            let back = (carrier.code.to_string(), r.other.clone(), r.at.clone());
            if let Some(first) = seen.get(&back) {
                errors.push(format!("{here}: written at both ends, first at {first}"));
                continue;
            }
            seen.entry((back.0, back.2, back.1)).or_insert(here);
            let windows: &'static [(f64, f64)] = if r.windows.is_empty() {
                LINES
            } else {
                Box::leak(r.windows.into_boxed_slice())
            };
            rows.entry(r.at).or_default().push(RouteRow {
                carrier,
                number: r.number,
                other: leak_str(r.other),
                city: leak_str(other.city.clone()),
                summer: r.summer,
                winter: r.winter,
                windows,
                based: r.based,
                types: leak_types(r.types),
                mirror_of: None,
            });
        }
        airlines.push(Airline {
            carrier,
            name: leak_str(c.name),
            iata: leak_str(c.iata),
        });
    }
    let tables = rows
        .into_iter()
        .filter_map(|(at, rows)| {
            let p = place(&at)?;
            Some(FieldRoutes {
                icao: leak_str(at),
                utc_offset_h: p.utc_offset_h,
                city: leak_str(p.city.clone()),
                top_up: p.top_up,
                rows: Box::leak(rows.into_boxed_slice()),
            })
        })
        .collect();
    let telephony = airlines
        .iter()
        .map(|a| (a.carrier.code, a.carrier.telephony))
        .collect();
    let data = Data {
        tables: derive(tables),
        places,
        airlines,
        telephony,
    };
    (data, errors)
}

/// Every table with the rows other tables share with it (2026-10-14): a
/// route between two table fields is ONE thing, written for one field; the
/// other field's row is DERIVED here from it, after its own rows. A
/// derived row keeps the source's carrier, numbers, frequencies and types;
/// its `other` is the source field, its city the source's name, and its
/// times are the source's flights a block time away ([`crate::rotation`]),
/// so `based` and `windows` are the source's business and unused here.
/// Malaga's EZY8061 row is Gatwick's too: Malaga's departure EZY8062 is
/// Gatwick's arrival EZY8062.
fn derive(tables: Vec<FieldRoutes>) -> Vec<FieldRoutes> {
    let mut shared: HashMap<&str, Vec<RouteRow>> = HashMap::new();
    for src in &tables {
        for r in src
            .rows
            .iter()
            .filter(|r| r.mirror_of.is_none() && r.other != src.icao)
        {
            shared.entry(r.other).or_default().push(RouteRow {
                other: src.icao,
                city: src.city,
                windows: LINES,
                based: false,
                mirror_of: Some(src.icao),
                ..*r
            });
        }
    }
    tables
        .into_iter()
        .map(|t| match shared.remove(t.icao) {
            None => t,
            Some(more) => FieldRoutes {
                rows: Box::leak([t.rows, &more].concat().into_boxed_slice()),
                ..t
            },
        })
        .collect()
}

/// The route data, read from the timetables' folder on first use and
/// never changed after: immutable shipped data, like a const table, never
/// state (the crate's "no statics" is about facts; this holds none).
fn data() -> &'static Data {
    static DATA: OnceLock<Data> = OnceLock::new();
    DATA.get_or_init(|| {
        let dir = routes_dir();
        let fields = std::fs::read_to_string(dir.join("fields.yaml")).unwrap_or_default();
        let (data, errors) = load(&fields, &yaml_files(&dir.join("carriers")));
        for e in errors {
            tracing::error!("route data: {e}");
        }
        data
    })
}

/// Every field with a route table.
pub fn fields() -> &'static [FieldRoutes] {
    &data().tables
}

/// The route table of `icao`, if we fly one there.
pub fn field_routes(icao: &str) -> Option<&'static FieldRoutes> {
    fields().iter().find(|f| f.icao.eq_ignore_ascii_case(icao))
}

/// A field a route names ([`crate::places`]).
pub fn place(icao: &str) -> Option<&'static Place> {
    data()
        .places
        .iter()
        .find(|p| p.icao.eq_ignore_ascii_case(icao))
}

/// Every carrier the route data holds, for the telephony check
/// ([`crate::airlines::all_carriers`]).
pub fn all_carriers() -> Vec<Carrier> {
    data().airlines.iter().map(|a| a.carrier).collect()
}

/// Every carrier's (code, telephony): what the station learns through the
/// party line's provider (`atc::callsign::provide_carrier_telephony`).
pub fn telephony() -> &'static [(&'static str, &'static str)] {
    &data().telephony
}

/// An airline's name as a board shows it, and its IATA designator (the
/// board's flight numbers: "U2 8901"). Empty for one the data does not
/// hold.
pub fn airline_name(code: &str) -> (&'static str, &'static str) {
    data()
        .airlines
        .iter()
        .find(|a| a.carrier.code == code)
        .map_or(("", ""), |a| (a.name, a.iata))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn shipped() -> (String, Vec<(String, String)>) {
        let dir = routes_dir();
        let fields = std::fs::read_to_string(dir.join("fields.yaml")).unwrap();
        (fields, yaml_files(&dir.join("carriers")))
    }

    /// The shipped data loads without a word, each carrier file named by
    /// its code, each field once, every clock a real one.
    #[test]
    fn the_route_data_loads_clean() {
        let (fields, carriers) = shipped();
        let (data, errors) = load(&fields, &carriers);
        assert!(errors.is_empty(), "{errors:#?}");
        assert!(data.tables.len() >= 3);
        assert_eq!(data.airlines.len(), carriers.len());
        for (a, (file, _)) in data.airlines.iter().zip(&carriers) {
            assert_eq!(format!("{}.yaml", a.carrier.code), *file);
        }
        let mut seen = std::collections::HashSet::new();
        for p in &data.places {
            assert!(seen.insert(&p.icao), "{} twice", p.icao);
            assert!((-12.0..=14.0).contains(&p.utc_offset_h), "{}", p.icao);
            assert!(!p.city.is_empty(), "{}", p.icao);
        }
    }

    /// A file that misspells a key fails naming the file and the row; a
    /// pair written at both ends names both rows; a row to no field names
    /// it.
    #[test]
    fn a_bad_route_file_names_its_file_and_row() {
        let fields = "- {icao: EGKK, city: A, lat: 0, lon: 0, utc_offset_h: 0, dst: true}\n\
                      - {icao: LEMG, city: B, lat: 1, lon: 1, utc_offset_h: 1, dst: true}\n";
        let head = "code: EZY\nname: E\niata: U2\ntelephony: Easy\ntypes: [[A320, 1]]\n\
                    numbers: [11, 8999]\nroutes:\n";
        let file = |rows: &str| vec![("EZY.yaml".to_string(), format!("{head}{rows}"))];
        let row = "  - {at: LEMG, other: EGKK, number: 11, summer: 7, winter: 7}\n";
        let (_, e) = load(fields, &file(&row.replace("summer", "sumer")));
        assert!(
            e[0].contains("EZY.yaml") && e[0].contains("routes[0]"),
            "{e:?}"
        );
        let back = "  - {at: EGKK, other: LEMG, number: 13, summer: 7, winter: 7}\n";
        let (data, e) = load(fields, &file(&format!("{row}{row}{back}")));
        assert!(
            e.len() == 1 && e[0].contains("routes[2]") && e[0].contains("routes[0]"),
            "{e:?}"
        );
        assert_eq!(data.tables.len(), 1, "EGKK's one row was dropped");
        assert_eq!(data.tables[0].rows.len(), 2);
        let (_, e) = load(fields, &file(&row.replace("EGKK", "EGLL")));
        assert!(e[0].contains("routes[0]") && e[0].contains("EGLL"), "{e:?}");
    }

    /// The parse cost of a world's worth of routes: 50,000 rows over 500
    /// carriers and 2,000 fields, printed (`--nocapture`), never asserted.
    #[test]
    fn a_world_of_routes_parses() {
        let l = |n: usize| char::from(b'A' + (n % 26) as u8);
        let icao = |i: usize| {
            let i = i % 2000;
            format!("K{}{}{}", l(i / 676), l(i / 26), l(i))
        };
        let mut fields = String::new();
        for i in 0..2000 {
            fields += &format!(
                "- {{icao: {}, city: \"City {i}\", lat: 10.5, lon: 20.25, utc_offset_h: 1.0, dst: true}}\n",
                icao(i)
            );
        }
        let carriers: Vec<(String, String)> = (0..500)
            .map(|c| {
                let mut t = format!(
                    "code: C{c:02}\nname: \"Carrier {c}\"\niata: X{c}\ntelephony: \"Tel {c}\"\n\
                     types: [[A320, 60], [A321, 40]]\nnumbers: [1, 9999]\nroutes:\n"
                );
                for r in 0..100 {
                    t += &format!(
                        "  - {{at: {}, other: {}, number: {}, summer: 7, winter: 5, windows: [[8.0, 9.5]]}}\n",
                        icao(c * 3 + r),
                        icao(c * 7 + r * 13 + 1000),
                        2 * r + 1
                    );
                }
                (format!("C{c:02}.yaml"), t)
            })
            .collect();
        let t0 = std::time::Instant::now();
        let (data, errors) = load(&fields, &carriers);
        let rows: usize = data.tables.iter().map(|t| t.rows.len()).sum();
        println!(
            "50k routes: {:.0} ms, {rows} rows with derived, {} errors",
            t0.elapsed().as_secs_f64() * 1000.0,
            errors.len()
        );
        assert!(rows >= 50_000);
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

    /// The derived row is the source's, its far end the source field (a
    /// route written at both ends is the loader's error,
    /// [`a_bad_route_file_names_its_file_and_row`]).
    #[test]
    fn a_derived_row_is_its_sources() {
        let lxgb = *field_routes("LXGB").unwrap();
        let ezy = lxgb.rows.iter().find(|r| r.other == "EGKK").unwrap();
        let table = |rows: Vec<RouteRow>| FieldRoutes {
            icao: "EGKK",
            utc_offset_h: 0.0,
            city: "London Gatwick",
            top_up: false,
            rows: Box::leak(rows.into_boxed_slice()),
        };
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
        assert!(derived_disagree(&derived).is_empty());
        // A derived row that strays from its source is caught.
        let mut strayed = d.to_vec();
        strayed[0].summer += 1;
        assert!(!derived_disagree(&[lxgb, table(strayed)]).is_empty());
    }

    /// The generated days of Gibraltar, Malaga and Gatwick, every movement
    /// written out (2026-10-15): pinned before the timetables became
    /// per-carrier data, so the move is proved to fly the same day. A
    /// change that moves a flight rewrites the fixture on purpose
    /// (`UPDATE_GOLDEN=write`).
    #[test]
    fn the_golden_days_are_unchanged() {
        use crate::rotation::{day_rotations, hhmm, Date};
        use crate::schedule::Density;
        let mut got = String::new();
        for icao in ["LXGB", "LEMG", "EGKK"] {
            for (y, m, d) in [(2026, 7, 15), (2026, 7, 18), (2026, 1, 14), (2026, 1, 17)] {
                let f = field_routes(icao).unwrap();
                for r in day_rotations(f, Date::ymd(y, m, d), 3, Density::Normal) {
                    for (kind, l) in [("arr", &r.arr), ("dep", &r.dep)] {
                        got += &format!(
                            "{icao} {y}-{m:02}-{d:02} {kind} {} {} {} {} {} {:.0} {:.0} {} {:016x} {}\n",
                            l.callsign,
                            r.icao_type,
                            l.other,
                            l.city,
                            hhmm(l.sched_s),
                            l.sched_s,
                            l.est_s,
                            r.night_stop,
                            r.seed,
                            l.spoken
                        );
                    }
                }
            }
        }
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/src/routes_golden.txt");
        if std::env::var("UPDATE_GOLDEN").as_deref() == Ok("write") {
            std::fs::write(path, &got).unwrap();
        }
        let want = std::fs::read_to_string(path).unwrap();
        for (i, (g, w)) in got.lines().zip(want.lines()).enumerate() {
            assert_eq!(g, w, "line {}", i + 1);
        }
        assert_eq!(got.lines().count(), want.lines().count());
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
