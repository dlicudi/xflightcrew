//! Navigation-data lookup from X-Plane's `earth_nav.dat`.
//!
//! Parses ILS localizer records (row code 4) to resolve "the ILS for runway N"
//! into a frequency + magnetic course, choosing the one nearest the aircraft.
//!
//! Field encoding (decoded empirically against KSEA, then verified): a row-4
//! line is `4 lat lon elev FREQ range BEARING ident AIRPORT region RUNWAY name`,
//! where FREQ is in 10 kHz units (11075 = 110.75 MHz) and the localizer
//! magnetic course is BEARING / 360 (123840.337 → 344°).

use std::io::{BufRead, BufReader};

use anyhow::{Context, Result};

#[derive(Debug, Clone)]
pub struct Ils {
    pub freq_10khz: i64,
    pub course_mag: f64,
    pub airport: String,
    pub runway: String,
    pub distance_nm: f64,
}

impl Ils {
    pub fn freq_mhz(&self) -> f64 {
        self.freq_10khz as f64 / 100.0
    }
}

/// Find the ILS localizer for `runway` (e.g. "28", "16L") nearest `(lat, lon)`,
/// within `max_nm`. Returns `None` if there's no match nearby.
pub fn find_ils(path: &str, runway: &str, lat: f64, lon: f64, max_nm: f64) -> Result<Option<Ils>> {
    let want = match runway_number(runway) {
        Some(n) => n,
        None => return Ok(None),
    };

    let file = std::fs::File::open(path).with_context(|| format!("opening nav data: {path}"))?;
    let reader = BufReader::new(file);

    let mut best: Option<Ils> = None;
    for line in reader.lines() {
        let line = line?;
        let f: Vec<&str> = line.split_whitespace().collect();
        // Row code 4 = ILS localizer; need through the runway field.
        if f.len() < 11 || f[0] != "4" {
            continue;
        }
        if runway_number(f[10]) != Some(want) {
            continue;
        }
        let (rlat, rlon) = match (f[1].parse::<f64>(), f[2].parse::<f64>()) {
            (Ok(a), Ok(b)) => (a, b),
            _ => continue,
        };
        let d = haversine_nm(lat, lon, rlat, rlon);
        if d > max_nm {
            continue;
        }
        if best.as_ref().is_none_or(|b| d < b.distance_nm) {
            best = Some(Ils {
                freq_10khz: f[4].parse().unwrap_or(0),
                course_mag: (f[6].parse::<f64>().unwrap_or(0.0) / 360.0).rem_euclid(360.0),
                airport: f[8].to_string(),
                runway: f[10].to_string(),
                distance_nm: d,
            });
        }
    }
    Ok(best)
}

/// The airport (ICAO) of the ILS record nearest `(lat, lon)`, with its distance
/// in nm — i.e. "which ILS-equipped field am I at/closest to". Used to catch
/// "runway 13" when you're parked at a field that has no runway 13: the matched
/// ILS would be at a *different*, farther airport.
pub fn nearest_ils_airport(path: &str, lat: f64, lon: f64) -> Result<Option<(String, f64)>> {
    let file = std::fs::File::open(path).with_context(|| format!("opening nav data: {path}"))?;
    let reader = BufReader::new(file);

    let mut best: Option<(String, f64)> = None;
    for line in reader.lines() {
        let line = line?;
        let f: Vec<&str> = line.split_whitespace().collect();
        if f.len() < 11 || f[0] != "4" {
            continue;
        }
        let (rlat, rlon) = match (f[1].parse::<f64>(), f[2].parse::<f64>()) {
            (Ok(a), Ok(b)) => (a, b),
            _ => continue,
        };
        let d = haversine_nm(lat, lon, rlat, rlon);
        if best.as_ref().is_none_or(|(_, bd)| d < *bd) {
            best = Some((f[8].to_string(), d));
        }
    }
    Ok(best)
}

/// One ILS/LOC facility at a known airport — the approach controller's chart
/// data. Course keeps the same `bearing / 360` magnetic decode as [`find_ils`];
/// the glideslope angle comes from the matching row-6 record (its bearing
/// field is `angle × 100_000 + true course`: 300089.709 → 3.00°).
#[derive(Debug, Clone)]
pub struct AirportIls {
    pub runway: String,
    /// Frequency and course ride along for the copilot's "set up the ILS"
    /// composition (auto-tuning NAV1 off the clearance) — the approach
    /// controller itself only needs to know the ILS exists.
    #[allow(dead_code)]
    pub freq_10khz: i64,
    #[allow(dead_code)]
    pub course_mag: f64,
    /// Glideslope angle, degrees — sets how far out the vectors join final
    /// (the platform must sit under the slope there).
    pub gs_deg: Option<f64>,
}

/// Every ILS (row 4) and standalone localizer (row 5) at `icao`, glideslope
/// angles merged in. One linear scan; call it from a background load, not the
/// reply path — earth_nav.dat is tens of MB.
pub fn airport_ils(path: &str, icao: &str) -> Result<Vec<AirportIls>> {
    let file = std::fs::File::open(path).with_context(|| format!("opening nav data: {path}"))?;
    let reader = BufReader::new(file);
    let mut out: Vec<AirportIls> = Vec::new();
    let mut slopes: Vec<(String, f64)> = Vec::new(); // (runway, gs_deg)
    for line in reader.lines() {
        let line = line?;
        let f: Vec<&str> = line.split_whitespace().collect();
        if f.len() < 11 || f[8] != icao {
            continue;
        }
        match f[0] {
            "4" | "5" => out.push(AirportIls {
                runway: f[10].to_string(),
                freq_10khz: f[4].parse().unwrap_or(0),
                course_mag: (f[6].parse::<f64>().unwrap_or(0.0) / 360.0).rem_euclid(360.0),
                gs_deg: None,
            }),
            "6" => {
                if let Ok(enc) = f[6].parse::<f64>() {
                    slopes.push((f[10].to_string(), (enc / 1000.0).floor() / 100.0));
                }
            }
            _ => {}
        }
    }
    for ils in &mut out {
        ils.gs_deg = slopes.iter().find(|(r, _)| *r == ils.runway).map(|(_, a)| *a);
    }
    Ok(out)
}

/// The ILS platform altitude for `runway` (ft MSL) from the airport's CIFP
/// approach — the "descend to X until established" number, read off the real
/// procedure instead of invented. Final-segment legs only (route type `I` of
/// approach `I<runway>`); the FAF leg's crossing altitude (desc-code 4th char
/// `F`) wins, then the IF leg's glideslope-intercept altitude (alt-desc `I`,
/// second altitude), then the highest altitude on the segment. `None` = no
/// CIFP file, no ILS approach in it, or no parsable altitude — the caller
/// falls back to a geometric platform.
pub fn ils_platform_alt_ft(cifp_dir: &str, icao: &str, runway: &str) -> Option<f64> {
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let file = std::fs::File::open(path).ok()?;
    let reader = BufReader::new(file);
    // Approach idents may carry a variant suffix ("I20-Y"/"I20-Z" at Jerez):
    // accept them all — the first FAF found wins, and sibling variants of
    // the same ILS share the platform in practice.
    let want = format!("I{runway}");
    let want_pfx = format!("{want}-");
    let mut faf: Option<f64> = None;
    let mut gs_intercept: Option<f64> = None;
    let mut highest: Option<f64> = None;
    let alt_ft = |s: &str| -> Option<f64> {
        let s = s.trim();
        if let Some(fl) = s.strip_prefix("FL") {
            return fl.parse::<f64>().ok().map(|f| f * 100.0);
        }
        s.parse::<f64>().ok().filter(|a| *a > 0.0)
    };
    for line in reader.lines() {
        let Ok(line) = line else { continue };
        if !line.starts_with("APPCH:") {
            continue;
        }
        let f: Vec<&str> = line.split(',').collect();
        if f.len() < 25 || f[1] != "I" || (f[2] != want && !f[2].starts_with(&want_pfx)) {
            continue;
        }
        let a1 = alt_ft(f[23]);
        if f[8].chars().nth(3) == Some('F') {
            faf = faf.or(a1);
        }
        if f[22].trim() == "I" {
            gs_intercept = gs_intercept.or_else(|| alt_ft(f[24]));
        }
        if let Some(a) = a1 {
            // Missed-approach legs (after the runway fix) climb ABOVE the
            // platform; guard by only keeping altitudes at or below the
            // glideslope-intercept when one is known.
            if highest.is_none_or(|h| a > h) && gs_intercept.is_none_or(|g| a <= g) {
                highest = Some(a);
            }
        }
    }
    faf.or(gs_intercept).or(highest)
}

/// Leading digits of a runway designator ("28L" → 28, "07" → 7).
pub fn runway_number(s: &str) -> Option<u32> {
    let digits: String = s.chars().take_while(|c| c.is_ascii_digit()).collect();
    digits.parse().ok()
}

/// Great-circle distance in nautical miles.
/// One physical runway from apt.dat: both ends, geometry only. Headings are
/// computed from the end coordinates (degrees true).
#[derive(Debug, Clone)]
pub struct Runway {
    pub ident_a: String,
    pub ident_b: String,
    pub lat_a: f64,
    pub lon_a: f64,
    pub lat_b: f64,
    pub lon_b: f64,
    /// Pavement width from the 100-row, metres; 0.0 = unknown.
    pub width_m: f64,
}

impl Runway {
    /// Landing/takeoff heading for end A (the direction you fly when using it).
    pub fn heading_a_deg(&self) -> f64 {
        bearing_deg(self.lat_a, self.lon_a, self.lat_b, self.lon_b)
    }
    pub fn heading_b_deg(&self) -> f64 {
        bearing_deg(self.lat_b, self.lon_b, self.lat_a, self.lon_a)
    }
}

/// One communication frequency row from apt.dat (legacy row codes 50-56, or
/// 1050-1056 for 8.33 kHz channels). `kind` is the normalized service code
/// (row code mod 1000): see the `FREQ_*` constants. `khz` is the frequency in
/// kHz (118325 = 118.325 MHz); legacy rows carry 10 kHz units in the file and
/// are converted on parse.
#[derive(Debug, Clone, PartialEq)]
pub struct AtcFreq {
    pub kind: u32,
    pub khz: i64,
    /// The row's label, e.g. "Tower Freq" — display only, not phraseology.
    pub name: String,
}

// The comm-frequency surface is consumed by the atc binary's tree; the copilot
// binary compiles this module too but doesn't reach it (yet), hence the allows.
#[allow(dead_code)]
pub const FREQ_ATIS: u32 = 50;
#[allow(dead_code)]
pub const FREQ_UNICOM: u32 = 51;
#[allow(dead_code)]
pub const FREQ_CLEARANCE: u32 = 52;
#[allow(dead_code)]
pub const FREQ_GROUND: u32 = 53;
#[allow(dead_code)]
pub const FREQ_TOWER: u32 = 54;
#[allow(dead_code)]
pub const FREQ_APPROACH: u32 = 55;
#[allow(dead_code)]
pub const FREQ_DEPARTURE: u32 = 56;

/// An airport as apt.dat knows it: the header's field elevation plus runways.
#[derive(Debug, Clone)]
pub struct AirportInfo {
    /// Name from the header row (e.g. "Gibraltar") — the spoken station name.
    #[allow(dead_code)] // atc-binary tree only (yet)
    pub name: String,
    /// Field elevation, feet MSL — the authoritative figure (a flight plan's
    /// destination-waypoint altitude is a *crossing* altitude, seen live 2,500 ft
    /// high over LERL).
    pub elev_ft: f64,
    pub runways: Vec<Runway>,
    /// Communication frequencies, in file order (empty for untowered fields).
    #[allow(dead_code)] // atc-binary tree only (yet)
    pub freqs: Vec<AtcFreq>,
}

impl AirportInfo {
    /// The field's reference position — the first runway threshold (apt.dat
    /// has no single airport lat/lon row; a threshold is within a mile of any
    /// sane answer). `None` for a field with no runway rows.
    pub fn position(&self) -> Option<(f64, f64)> {
        self.runways.first().map(|r| (r.lat_a, r.lon_a))
    }
}

impl AirportInfo {
    /// The first frequency of the given service `kind` (`FREQ_TOWER`, ...).
    #[allow(dead_code)] // atc-binary tree only (yet)
    pub fn freq(&self, kind: u32) -> Option<&AtcFreq> {
        self.freqs.iter().find(|f| f.kind == kind)
    }

    /// A tower row that names a real facility. The scenery Gateway fills some
    /// fields' comm rows with the literal "unnamed entity" — seen on LEMY,
    /// an ultralight strip whose air-to-air frequency is typed as TWR — and a
    /// row like that is a frequency, not a controller. Nobody should be
    /// handed off to it or told it's the nearest station.
    #[allow(dead_code)] // atc-binary tree only (yet)
    pub fn staffed_tower(&self) -> Option<&AtcFreq> {
        self.freq(FREQ_TOWER)
            .filter(|f| !f.name.trim().eq_ignore_ascii_case("unnamed entity"))
    }
}

/// The runways `icao` actually has, from X-Plane's Global Airports `apt.dat`.
pub fn find_runways(path: &str, icao: &str) -> Result<Vec<Runway>> {
    Ok(find_airport(path, icao)?.map(|a| a.runways).unwrap_or_default())
}

/// Find `icao` in X-Plane's Global Airports `apt.dat` (row 1 = airport header
/// `1 <elev_ft> x x <ICAO> <name>`, rows 100 = land runways: 8 header fields
/// then a 9-field block per end, ident/lat/lon leading each block — verified
/// against LEAL). A streaming scan of the ~400 MB file takes a moment; callers
/// answer a question with it, not a realtime loop. `None` = not found.
pub fn find_airport(path: &str, icao: &str) -> Result<Option<AirportInfo>> {
    let file = std::fs::File::open(path).with_context(|| format!("open {path}"))?;
    let mut reader = BufReader::with_capacity(1 << 20, file);
    let mut runways = Vec::new();
    let mut freqs = Vec::new();
    let mut name = String::new();
    let mut in_airport = false;
    let mut elev_ft = 0.0_f64;
    // Reused byte buffer + a cheap prefix test before any splitting: the file is
    // ~400 MB and 13M lines, and a String-per-line scan is unusable in a debug
    // build. Only `1 `/`16 `/`17 ` (headers), `100 ` (runways), and the comm
    // rows (`1050`-`1056`, legacy `50`-`56`) matter.
    let mut buf = Vec::with_capacity(256);
    loop {
        buf.clear();
        if reader.read_until(b'\n', &mut buf).unwrap_or(0) == 0 {
            break;
        }
        // b'1' covers 1, 16, 17, 100, 1050-1056; b'5' covers legacy 50-56.
        let interesting = matches!(buf.first(), Some(b'1') | Some(b'5'));
        if !interesting {
            continue;
        }
        let Ok(line) = std::str::from_utf8(&buf) else { continue };
        let mut f = line.split_whitespace();
        match f.next() {
            // Airport headers (land/sea/heli) start a block and end the previous.
            Some("1") | Some("16") | Some("17") => {
                if in_airport {
                    break; // ran past our airport's block
                }
                let elev: Option<f64> = f.next().and_then(|e| e.parse().ok());
                in_airport = f.nth(2) == Some(icao); // skip the two deprecated fields
                if in_airport {
                    elev_ft = elev.unwrap_or(0.0);
                    name = f.collect::<Vec<_>>().join(" ");
                }
            }
            Some("100") if in_airport => {
                if let Some(r) = parse_runway_row(f) {
                    runways.push(r);
                }
            }
            Some(code) if in_airport => {
                if let Some(fr) = parse_comm_row(code, f) {
                    freqs.push(fr);
                }
            }
            _ => {}
        }
    }
    // `in_airport` stays true when the file ends inside our block, or flips off
    // never having found it — runways empty either way distinguishes only via
    // the elevation default, so gate on having *seen* the airport.
    if !in_airport && runways.is_empty() {
        return Ok(None);
    }
    Ok(Some(AirportInfo { name, elev_ft, runways, freqs }))
}

/// A land-runway row (code 100): 8 header fields, then a 9-field block per
/// end, ident/lat/lon leading each block. `fields` starts after the row code.
fn parse_runway_row<'a>(fields: impl Iterator<Item = &'a str>) -> Option<Runway> {
    let f: Vec<&str> = fields.collect();
    if f.len() < 19 {
        return None;
    }
    match (f[8].parse(), f[9].parse(), f[17].parse(), f[18].parse()) {
        (Ok(lat_a), Ok(lon_a), Ok(lat_b), Ok(lon_b)) => Some(Runway {
            ident_a: f[7].to_string(),
            ident_b: f[16].to_string(),
            lat_a,
            lon_a,
            lat_b,
            lon_b,
            width_m: f[0].parse().unwrap_or(0.0),
        }),
        _ => None,
    }
}

/// A comm row: `<code> <freq> <label...>`. Legacy codes (50-56) store 10 kHz
/// units (12210 = 122.10 MHz); 1050+ codes store kHz (118325).
fn parse_comm_row<'a>(code: &str, mut fields: impl Iterator<Item = &'a str>) -> Option<AtcFreq> {
    let c: u32 = code.parse().ok()?;
    if !matches!(c, 50..=56 | 1050..=1056) {
        return None;
    }
    let raw: i64 = fields.next()?.parse().ok()?;
    Some(AtcFreq {
        kind: c % 1000,
        khz: if c >= 1000 { raw } else { raw * 10 },
        name: fields.collect::<Vec<_>>().join(" "),
    })
}

/// Stream EVERY airport in apt.dat through `f` (icao, parsed info) — one pass
/// over the ~400 MB file, the same row parsing as [`find_airport`]. Built for
/// offline sweeps (the ATC cross-airport battle test), where a per-airport
/// [`find_airport`] rescan would be quadratic.
pub fn scan_airports(path: &str, mut f: impl FnMut(&str, AirportInfo)) -> Result<()> {
    let file = std::fs::File::open(path).with_context(|| format!("open {path}"))?;
    let mut reader = BufReader::with_capacity(1 << 20, file);
    let mut cur: Option<(String, AirportInfo)> = None;
    let mut buf = Vec::with_capacity(256);
    loop {
        buf.clear();
        if reader.read_until(b'\n', &mut buf).unwrap_or(0) == 0 {
            break;
        }
        // Same cheap prefix filter as find_airport: only headers, runways and
        // comm rows matter, and a String per line is unusable at 13M lines.
        if !matches!(buf.first(), Some(b'1') | Some(b'5')) {
            continue;
        }
        let Ok(line) = std::str::from_utf8(&buf) else { continue };
        let mut fields = line.split_whitespace();
        match fields.next() {
            Some("1") | Some("16") | Some("17") => {
                if let Some((icao, apt)) = cur.take() {
                    f(&icao, apt);
                }
                let elev: Option<f64> = fields.next().and_then(|e| e.parse().ok());
                if let Some(icao) = fields.nth(2) {
                    let icao = icao.to_string();
                    let name = fields.collect::<Vec<_>>().join(" ");
                    cur = Some((
                        icao,
                        AirportInfo {
                            name,
                            elev_ft: elev.unwrap_or(0.0),
                            runways: Vec::new(),
                            freqs: Vec::new(),
                        },
                    ));
                }
            }
            Some("100") => {
                if let Some((_, apt)) = cur.as_mut() {
                    if let Some(r) = parse_runway_row(fields) {
                        apt.runways.push(r);
                    }
                }
            }
            Some(code) => {
                if let Some((_, apt)) = cur.as_mut() {
                    if let Some(fr) = parse_comm_row(code, fields) {
                        apt.freqs.push(fr);
                    }
                }
            }
            _ => {}
        }
    }
    if let Some((icao, apt)) = cur.take() {
        f(&icao, apt);
    }
    Ok(())
}

// --- Taxiway network (apt.dat 1201/1202 rows) --------------------------------

/// One taxi-network edge. Runway edges are routable — crossing a runway is a
/// real taxi instruction — but heavily penalized so a route never taxis
/// ALONG one when a taxiway exists.
#[derive(Debug, Clone)]
pub struct TaxiEdge {
    pub a: i64,
    pub b: i64,
    /// apt.dat "oneway": traversable a→b only.
    pub oneway: bool,
    pub runway: bool,
    /// Taxiway name ("A", "B2"); runway edges carry the designator pair
    /// ("09L/27R"). May be empty (unnamed links).
    pub name: String,
}

/// The taxi routing network of one airport.
#[derive(Debug, Clone, Default)]
pub struct TaxiNet {
    pub nodes: std::collections::HashMap<i64, (f64, f64)>,
    pub edges: Vec<TaxiEdge>,
}

/// A planned taxi route, ready to phrase: ordered taxiway names, runways
/// crossed, the holding-point name, and the node polyline for the
/// off-route compliance watch. Names are RAW apt.dat strings — the station
/// speaks them.
#[derive(Debug, Clone)]
pub struct TaxiRoutePlan {
    pub via: Vec<String>,
    pub crossings: Vec<String>,
    pub holding_point: String,
    pub path: Vec<(f64, f64)>,
    /// Per-segment (name, is_runway) aligned with `path`: `seg[i]` describes
    /// the leg `path[i] -> path[i+1]` (so `seg.len() == path.len() - 1`).
    /// This is what lets the copilot's taxi guide say "left onto Bravo"
    /// instead of just "left": the planner knew the names all along.
    pub seg: Vec<(String, bool)>,
}

/// Parse `icao`'s taxi network from apt.dat — same streaming block scan as
/// [`find_airport`], collecting 1201 (node) and 1202 (edge) rows. `None`
/// when the field has no network (small strips mostly don't).
pub fn find_taxi_net(path: &str, icao: &str) -> Result<Option<TaxiNet>> {
    let file = std::fs::File::open(path).with_context(|| format!("open {path}"))?;
    let mut reader = BufReader::with_capacity(1 << 20, file);
    let mut net = TaxiNet::default();
    let mut in_airport = false;
    let mut buf = Vec::with_capacity(256);
    loop {
        buf.clear();
        if reader.read_until(b'\n', &mut buf).unwrap_or(0) == 0 {
            break;
        }
        if buf.first() != Some(&b'1') {
            continue;
        }
        let Ok(line) = std::str::from_utf8(&buf) else { continue };
        let mut f = line.split_whitespace();
        match f.next() {
            Some("1") | Some("16") | Some("17") => {
                if in_airport {
                    break;
                }
                in_airport = f.nth(3) == Some(icao);
            }
            Some("1201") if in_airport => {
                // lat lon usage id [name...]
                let (lat, lon) = match (
                    f.next().and_then(|s| s.parse::<f64>().ok()),
                    f.next().and_then(|s| s.parse::<f64>().ok()),
                ) {
                    (Some(a), Some(b)) => (a, b),
                    _ => continue,
                };
                let _usage = f.next();
                if let Some(id) = f.next().and_then(|s| s.parse::<i64>().ok()) {
                    net.nodes.insert(id, (lat, lon));
                }
            }
            Some("1202") if in_airport => {
                // a b twoway|oneway runway|taxiway [name...]
                let (a, b) = match (
                    f.next().and_then(|s| s.parse::<i64>().ok()),
                    f.next().and_then(|s| s.parse::<i64>().ok()),
                ) {
                    (Some(a), Some(b)) => (a, b),
                    _ => continue,
                };
                let oneway = f.next() == Some("oneway");
                let kind = f.next().unwrap_or("");
                net.edges.push(TaxiEdge {
                    a,
                    b,
                    oneway,
                    runway: kind == "runway",
                    name: f.collect::<Vec<_>>().join(" ").trim().to_string(),
                });
            }
            _ => {}
        }
    }
    Ok((!net.edges.is_empty()).then_some(net))
}

impl TaxiNet {
    /// Route from a position to a position (a stand to a runway threshold)
    /// and extract the phrasable plan. `None` when either end doesn't snap
    /// to the network (within 3 km) or no path exists.
    pub fn plan(
        &self,
        from_lat: f64,
        from_lon: f64,
        to_lat: f64,
        to_lon: f64,
    ) -> Option<TaxiRoutePlan> {
        let m_per_deg_lat = 111_320.0;
        let m_per_deg_lon = 111_320.0 * from_lat.to_radians().cos();
        let dist_m = |a: (f64, f64), b: (f64, f64)| -> f64 {
            let dx = (a.1 - b.1) * m_per_deg_lon;
            let dy = (a.0 - b.0) * m_per_deg_lat;
            (dx * dx + dy * dy).sqrt()
        };
        // Snap both ends to the nearest node that touches a TAXIWAY edge —
        // never a runway-only node (the aircraft taxis on taxiways).
        let mut taxiway_nodes: std::collections::HashSet<i64> = std::collections::HashSet::new();
        for e in self.edges.iter().filter(|e| !e.runway) {
            taxiway_nodes.insert(e.a);
            taxiway_nodes.insert(e.b);
        }
        let snap = |lat: f64, lon: f64| -> Option<i64> {
            taxiway_nodes
                .iter()
                .filter_map(|id| self.nodes.get(id).map(|&p| (*id, dist_m((lat, lon), p))))
                .min_by(|x, y| x.1.total_cmp(&y.1))
                .filter(|(_, d)| *d < 3000.0)
                .map(|(id, _)| id)
        };
        let start = snap(from_lat, from_lon)?;
        let goal = snap(to_lat, to_lon)?;
        if start == goal {
            return None;
        }
        // Dijkstra; runway edges cost ×25 so a route only touches one to
        // CROSS it, never to taxi along it.
        let mut adj: std::collections::HashMap<i64, Vec<usize>> = std::collections::HashMap::new();
        for (i, e) in self.edges.iter().enumerate() {
            adj.entry(e.a).or_default().push(i);
            if !e.oneway {
                adj.entry(e.b).or_default().push(i);
            }
        }
        use std::cmp::Reverse;
        use std::collections::BinaryHeap;
        let mut best: std::collections::HashMap<i64, (f64, Option<usize>)> =
            std::collections::HashMap::new();
        let mut heap: BinaryHeap<(Reverse<u64>, i64)> = BinaryHeap::new();
        best.insert(start, (0.0, None));
        heap.push((Reverse(0), start));
        while let Some((Reverse(cost_mm), node)) = heap.pop() {
            let cost = cost_mm as f64 / 1000.0;
            if node == goal {
                break;
            }
            if best.get(&node).is_none_or(|(c, _)| cost > *c + 0.5) {
                continue; // stale heap entry
            }
            for &ei in adj.get(&node).map(|v| v.as_slice()).unwrap_or(&[]) {
                let e = &self.edges[ei];
                let next = if e.a == node { e.b } else { e.a };
                let (Some(&pa), Some(&pb)) = (self.nodes.get(&e.a), self.nodes.get(&e.b)) else {
                    continue;
                };
                let mut w = dist_m(pa, pb);
                if e.runway {
                    w *= 25.0;
                }
                let nc = cost + w;
                if best.get(&next).is_none_or(|(c, _)| nc < *c) {
                    best.insert(next, (nc, Some(ei)));
                    heap.push((Reverse((nc * 1000.0) as u64), next));
                }
            }
        }
        // Walk back goal→start collecting edges, then flip.
        let mut edges_rev: Vec<usize> = Vec::new();
        let mut node = goal;
        let mut hops = 0;
        while node != start {
            let (_, Some(ei)) = *best.get(&node)? else { return None };
            edges_rev.push(ei);
            let e = &self.edges[ei];
            node = if e.a == node { e.b } else { e.a };
            hops += 1;
            if hops > 10_000 {
                return None; // cycle guard
            }
        }
        edges_rev.reverse();
        let mut via: Vec<String> = Vec::new();
        let mut crossings: Vec<String> = Vec::new();
        let mut holding_point = String::new();
        let mut path: Vec<(f64, f64)> = Vec::new();
        let mut seg: Vec<(String, bool)> = Vec::new();
        let mut walk = start;
        path.extend(self.nodes.get(&walk).copied());
        for &ei in &edges_rev {
            let e = &self.edges[ei];
            walk = if e.a == walk { e.b } else { e.a };
            // Keep `seg` aligned with `path` segments: a seg entry only when
            // this edge actually appends a point after an existing one.
            if let Some(&p) = self.nodes.get(&walk) {
                if !path.is_empty() {
                    seg.push((e.name.clone(), e.runway));
                }
                path.push(p);
            }
            if e.name.is_empty() {
                continue;
            }
            if e.runway {
                if !crossings.contains(&e.name) {
                    crossings.push(e.name.clone());
                }
            } else {
                if via.last() != Some(&e.name) {
                    via.push(e.name.clone());
                }
                holding_point = e.name.clone();
            }
        }
        via.dedup();
        via.truncate(6);
        Some(TaxiRoutePlan { via, crossings, holding_point, path, seg })
    }
}

/// Pick the runway end best aligned into the wind. Returns the end's ident,
/// its true heading, and the headwind component (negative = tailwind).
/// Deterministic by construction: it can only name a runway the field has.
pub fn best_runway_for_wind(
    runways: &[Runway],
    wind_from_deg: f64,
    wind_kts: f64,
) -> Option<(String, f64, f64)> {
    let mut best: Option<(String, f64, f64)> = None;
    for r in runways {
        for (ident, hdg) in
            [(&r.ident_a, r.heading_a_deg()), (&r.ident_b, r.heading_b_deg())]
        {
            let diff = angle_diff_deg(wind_from_deg, hdg);
            let headwind = wind_kts * diff.to_radians().cos();
            if best.as_ref().is_none_or(|(_, _, hw)| headwind > *hw) {
                best = Some((ident.clone(), hdg, headwind));
            }
        }
    }
    best
}

/// Smallest signed angle a−b, degrees, in −180..180.
fn angle_diff_deg(a: f64, b: f64) -> f64 {
    (a - b + 540.0).rem_euclid(360.0) - 180.0
}

/// Initial great-circle bearing from point 1 to point 2, degrees true 0..360.
fn bearing_deg(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let (p1, p2) = (lat1.to_radians(), lat2.to_radians());
    let dl = (lon2 - lon1).to_radians();
    let y = dl.sin() * p2.cos();
    let x = p1.cos() * p2.sin() - p1.sin() * p2.cos() * dl.cos();
    y.atan2(x).to_degrees().rem_euclid(360.0)
}

fn haversine_nm(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    const R_NM: f64 = 3440.065;
    let (p1, p2) = (lat1.to_radians(), lat2.to_radians());
    let dphi = (lat2 - lat1).to_radians();
    let dlam = (lon2 - lon1).to_radians();
    let a = (dphi / 2.0).sin().powi(2) + p1.cos() * p2.cos() * (dlam / 2.0).sin().powi(2);
    2.0 * R_NM * a.sqrt().asin()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    #[test]
    fn runway_number_parses_designators() {
        assert_eq!(runway_number("28"), Some(28));
        assert_eq!(runway_number("16L"), Some(16));
        assert_eq!(runway_number("07R"), Some(7));
        assert_eq!(runway_number("K1"), None);
    }

    #[test]
    fn finds_nearest_matching_runway_and_decodes_fields() {
        // Two runway-28 ILS records; one ~0nm away, one far. Expect the near one.
        let dir = std::env::temp_dir();
        let path = dir.join("xc-nav-test.dat");
        let mut f = std::fs::File::create(&path).unwrap();
        writeln!(f, "I").unwrap();
        writeln!(f, "1200 Version - test").unwrap();
        // near (lat 36.71, lon -4.55): freq 10950 -> 109.50, bearing 281.785*360=101442.6
        writeln!(f, " 4  36.710000000   -4.550000000  100 10950 18 101442.600 ITST LXXX LX 28 ILS-cat-I").unwrap();
        // far (KSEA-ish)
        writeln!(f, " 4  47.466600000 -122.317800000  359 11075 18 123840.337 IFAR KSEA K1 28 ILS-cat-I").unwrap();
        f.flush().unwrap();

        let ils = find_ils(path.to_str().unwrap(), "28", 36.71, -4.55, 60.0)
            .unwrap()
            .expect("should find the near runway-28 ILS");
        assert_eq!(ils.airport, "LXXX");
        assert!((ils.freq_mhz() - 109.50).abs() < 0.001, "freq {}", ils.freq_mhz());
        assert!((ils.course_mag - 281.785).abs() < 0.5, "course {}", ils.course_mag);
    }

    #[test]
    fn parses_leal_runways_and_picks_into_the_wind() {
        // Verbatim LEAL rows from the Global Airports apt.dat, plus a following
        // airport to prove the block scan stops.
        let path = std::env::temp_dir().join("xc-apt-test.dat");
        let mut f = std::fs::File::create(&path).unwrap();
        writeln!(f, "1    142 0 0 LEAL Alicante").unwrap();
        writeln!(f, "100 45.33 27 727 0.10 1 3 0  10 38.2845188 -0.5750054 0 0 3 3 0 2 28 38.2797989 -0.5412058 0 0 3 3 0 2").unwrap();
        // Comm rows: an 8.33-style row (kHz) and a legacy row (10 kHz units).
        writeln!(f, "1054 118250 Tower Freq").unwrap();
        writeln!(f, "50 12210 ATIS").unwrap();
        writeln!(f, "1   1027 0 0 0MU1 Sunderland").unwrap();
        writeln!(f, "100 22.86 3 0 0.25 0 0 0  09 37.2471001 -94.1770610 0 0 1 0 0 0 27 37.2469784 -94.1700162 0 0 1 0 0 0").unwrap();
        writeln!(f, "1054 122800 Sunderland Tower").unwrap();
        f.flush().unwrap();

        // Field elevation and name come off the header row (LEAL sits at 142 ft).
        let apt = find_airport(path.to_str().unwrap(), "LEAL").unwrap().expect("found");
        assert_eq!(apt.elev_ft, 142.0);
        assert_eq!(apt.name, "Alicante");
        // Frequencies: only LEAL's, decoded to kHz whatever the row era.
        assert_eq!(apt.freqs.len(), 2, "LEAL's two comm rows, not Sunderland's");
        let twr = apt.freq(FREQ_TOWER).expect("tower row");
        assert_eq!((twr.khz, twr.name.as_str()), (118250, "Tower Freq"));
        assert_eq!(apt.freq(FREQ_ATIS).expect("atis row").khz, 122100, "legacy 10 kHz units");
        assert!(apt.freq(FREQ_GROUND).is_none());
        let rw = find_runways(path.to_str().unwrap(), "LEAL").unwrap();
        assert_eq!(rw.len(), 1, "one physical runway, not Sunderland's");
        assert_eq!((rw[0].ident_a.as_str(), rw[0].ident_b.as_str()), ("10", "28"));
        // Geometry sanity: the 10 end points roughly east-southeast.
        assert!((rw[0].heading_a_deg() - 100.0).abs() < 8.0, "{}", rw[0].heading_a_deg());

        // The hallucination case: wind 157 at 6 (today's LEAL METAR). The LLM
        // said "runway 18"; the field has 10 and 28, and 10 is more into wind.
        let (ident, _, headwind) = best_runway_for_wind(&rw, 157.0, 6.0).expect("a pick");
        assert_eq!(ident, "10");
        assert!(headwind > 0.0, "into wind, not tailwind: {headwind}");
        // Westerly wind flips it.
        assert_eq!(best_runway_for_wind(&rw, 280.0, 10.0).unwrap().0, "28");
        // Unknown airport → empty, and the chooser refuses to invent.
        let none = find_runways(path.to_str().unwrap(), "LEMD").unwrap();
        assert!(none.is_empty());
        assert!(best_runway_for_wind(&none, 157.0, 6.0).is_none());
    }

    #[test]
    fn taxi_net_parses_routes_and_names_the_crossings() {
        // A toy field: runway 09/27 east-west, a stand north of it, taxiway
        // A south from the stand, a short runway-flagged segment (the 14/32
        // crossing), then taxiway B west to the 09 holding point.
        //
        //   stand(1)
        //     | A
        //    (2)
        //     | 14/32  <- crossing
        //    (3)
        //     \___ B ___ (4)   [near the 09 threshold]
        //
        let path = std::env::temp_dir().join("xc-taxi-test.dat");
        let mut f = std::fs::File::create(&path).unwrap();
        writeln!(f, "1    50 0 0 LTST Testfield").unwrap();
        writeln!(f, "100 45.33 27 727 0.10 1 3 0  09 36.0000000 -5.0100000 0 0 3 3 0 2 27 36.0000000 -5.0000000 0 0 3 3 0 2").unwrap();
        writeln!(f, "1054 118250 Testfield Tower").unwrap();
        writeln!(f, "1201 36.0030000 -5.0050000 both 1 stand").unwrap();
        writeln!(f, "1201 36.0020000 -5.0050000 both 2 A").unwrap();
        writeln!(f, "1201 36.0010000 -5.0050000 both 3 junction").unwrap();
        writeln!(f, "1201 36.0004000 -5.0095000 both 4 B").unwrap();
        writeln!(f, "1202 1 2 twoway taxiway A").unwrap();
        writeln!(f, "1202 2 3 twoway runway 14/32").unwrap();
        writeln!(f, "1202 3 4 twoway taxiway B").unwrap();
        writeln!(f, "1 1027 0 0 0MU1 Sunderland").unwrap();
        writeln!(f, "1202 5 6 twoway taxiway Z").unwrap();
        f.flush().unwrap();

        let net = find_taxi_net(path.to_str().unwrap(), "LTST")
            .unwrap()
            .expect("network found");
        assert_eq!(net.nodes.len(), 4);
        assert_eq!(net.edges.len(), 3, "only LTST's edges, not Sunderland's");

        // From the stand to the 09 threshold: via A and B, crossing 14/32,
        // holding point named for the last taxiway before the runway.
        let plan = net
            .plan(36.0031, -5.0050, 36.0000, -5.0100)
            .expect("a route exists");
        assert_eq!(plan.via, vec!["A".to_string(), "B".to_string()]);
        assert_eq!(plan.crossings, vec!["14/32".to_string()]);
        assert_eq!(plan.holding_point, "B");
        assert!(plan.path.len() >= 4, "polyline for the compliance watch");

        // Nowhere near the network: no snap, no plan — never a wild route.
        assert!(net.plan(37.0, -6.0, 36.0, -5.01).is_none());
        // A field without 1201/1202 rows has no network at all.
        assert!(find_taxi_net(path.to_str().unwrap(), "0MU1").unwrap().is_some());
        assert!(find_taxi_net(path.to_str().unwrap(), "LXXX").unwrap().is_none());
    }

    #[test]
    #[ignore] // needs the real X-Plane install; run with --ignored
    fn live_heathrow_taxi_route_plans() {
        let path = "/Users/duanelicudi/X-Plane 12/Global Scenery/Global Airports/Earth nav data/apt.dat";
        let t = std::time::Instant::now();
        let net = find_taxi_net(path, "EGLL").unwrap().expect("EGLL has a network");
        eprintln!("EGLL taxi net: {} nodes, {} edges in {:?}", net.nodes.len(), net.edges.len(), t.elapsed());
        assert!(net.nodes.len() > 500 && net.edges.len() > 300);
        // From T2 stands (51.4700, -0.4520) to the 27R threshold
        // (51.4775, -0.4325): a real route with named taxiways.
        let t = std::time::Instant::now();
        let plan = net.plan(51.4700, -0.4520, 51.4775, -0.4325).expect("a route");
        eprintln!("plan in {:?}: via {:?}, cross {:?}, hp {:?}", t.elapsed(), plan.via, plan.crossings, plan.holding_point);
        assert!(!plan.via.is_empty(), "named taxiways expected at Heathrow");
        assert!(t.elapsed().as_millis() < 200, "planning must not stall a reply");
    }

    #[test]
    #[ignore] // needs the real X-Plane install; run with --ignored
    fn live_apt_dat_scan_is_fast_enough_to_speak() {
        let path = "/Users/duanelicudi/X-Plane 12/Global Scenery/Global Airports/Earth nav data/apt.dat";
        let t = std::time::Instant::now();
        let rw = find_runways(path, "LEAL").unwrap();
        let elapsed = t.elapsed();
        assert_eq!(rw.len(), 1);
        assert_eq!((rw[0].ident_a.as_str(), rw[0].ident_b.as_str()), ("10", "28"));
        // A voice answer stalls past a few seconds — even a debug build must fit.
        assert!(elapsed.as_secs_f64() < 5.0, "scan took {elapsed:?}");
        eprintln!("LEAL scan: {elapsed:?}");
    }

    #[test]
    fn airport_ils_reads_locs_and_merges_glideslopes() {
        // Real EGLL rows (Custom Data cycle): loc row 4, GS row 6, plus a
        // different airport's loc that must not leak in.
        let path = std::env::temp_dir().join("xc-nav-ils.dat");
        let mut f = std::fs::File::create(&path).unwrap();
        writeln!(f, "I").unwrap();
        writeln!(f, "1150 Version - data cycle").unwrap();
        writeln!(f, " 4  51.477697222   -0.426961111       79    11030    18  31769.709  IAA EGLL EG 09L ILS-cat-III").unwrap();
        writeln!(f, " 6  51.478750000   -0.480675000       79    11030    18 300089.709  IAA EGLL EG 09L GS").unwrap();
        writeln!(f, " 4  51.464969444   -0.428230556       75    10950    18  31769.717  IBB EGLL EG 09R ILS-cat-III").unwrap();
        writeln!(f, " 4  40.472222222   -3.560833333     2000    11010    18  50400.000  MAA LEMD LE 32L ILS-cat-I").unwrap();
        writeln!(f, "99").unwrap();
        f.flush().unwrap();
        let ils = airport_ils(path.to_str().unwrap(), "EGLL").unwrap();
        assert_eq!(ils.len(), 2, "{ils:?}");
        let l = ils.iter().find(|i| i.runway == "09L").expect("09L");
        assert_eq!(l.freq_10khz, 11030);
        assert!((l.course_mag - 31769.709 / 360.0).abs() < 0.01, "{}", l.course_mag);
        assert_eq!(l.gs_deg, Some(3.0), "row-6 encoding: angle×100000 + course");
        let r = ils.iter().find(|i| i.runway == "09R").expect("09R");
        assert_eq!(r.gs_deg, None, "no GS row for 09R in this sample");
    }

    #[test]
    fn cifp_platform_altitude_reads_the_faf_crossing() {
        // Real EGLL ILS 27R final legs: IF at 3000 (GS intercept 2500), FAF
        // crossing 2500, runway fix, and a missed-approach climb that must
        // NOT win. Expected platform: the FAF's 2500.
        let dir = std::env::temp_dir().join("xc-cifp-test");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(
            dir.join("EGLL.dat"),
            "APPCH:010,I,I27R, ,CF27R,EG,P,C,E  I, ,   ,IF, ,IRR,EG,P,I,      ,0887,0123,    ,    ,I,03000,02500,     , ,   ,    ,   , , , , , ,0,D,S;\n\
             APPCH:020,I,I27R, ,FF27R,EG,P,C,E  F, ,   ,CF, ,IRR,EG,P,I,      ,0887,0098,2680,0025,G,02500,02500,     , ,   ,-300,   ,EGLL,EG,P,A, ,0,D,S;\n\
             APPCH:030,I,I27R, ,RW27R,EG,P,G,G  M, ,   ,CF, ,IRR,EG,P,I,      ,0887,0023,2680,0075, ,00136,     ,     , ,   ,-300,   , , , , , ,0,D,S;\n\
             APPCH:050,I,I27R, , , , , , E  , ,   ,CA, , , , , ,      ,    ,    ,3150,    ,+,03000,     ,     , ,   ,    ,   , , , , , ,0,D,S;\n\
             APPCH:010,A,I27R,BIG,BIG,EG,D, ,V  C, ,   ,IF, , , , , ,      ,    ,    ,    ,    ,-,220,    ,   , , , , , ,0,D,S;\n",
        )
        .unwrap();
        let dir = dir.to_str().unwrap().to_string();
        assert_eq!(ils_platform_alt_ft(&dir, "EGLL", "27R"), Some(2500.0));
        // No such approach / airport: honest None, callers fall back.
        assert_eq!(ils_platform_alt_ft(&dir, "EGLL", "09L"), None);
        assert_eq!(ils_platform_alt_ft(&dir, "EGKK", "26L"), None);
        // Variant-suffixed idents (Jerez has I20-Y/I20-Z, no bare I20): the
        // suffixed approach's FAF must still resolve. Real LEJR rows.
        std::fs::write(
            std::path::Path::new(&dir).join("LEJR.dat"),
            "APPCH:010,I,I20-Y, ,CF20Y,LE,P,C,E  I, ,   ,IF, ,IJR,LE,P,I,      ,0224,0104,    ,    ,J,02600,01800,     , ,   ,    ,   , , , , , ,0,D,S;\n\
             APPCH:020,I,I20-Y, ,JER,LE,D,B,E  F, ,   ,CF, ,IJR,LE,P,I,      ,0224,0066,2020,0038,H,01800,01800,     ,-,160,-300,   ,JER,LE,D,B, ,0,D,S;\n\
             APPCH:030,I,I20-Y, ,RW20,LE,P,G,G  M, ,   ,CF, ,IJR,LE,P,I,      ,0224,0014,2020,0052, ,00145,     ,     , ,   ,-300,   , , , , , ,0,D,S;\n",
        )
        .unwrap();
        assert_eq!(ils_platform_alt_ft(&dir, "LEJR", "20"), Some(1800.0));
    }

    /// Real-install smoke test (needs the sim's nav data + CIFP on disk):
    /// `cargo test -- --ignored real_install_ils`. Heathrow must resolve all
    /// four ILS runways and the 27R platform from the actual AIRAC files.
    #[test]
    #[ignore]
    fn real_install_ils_chart_data_reads() {
        let config = crate::config::Config::from_env().expect("config");
        let ils = airport_ils(&config.nav_data_path, "EGLL").expect("nav scan");
        for rwy in ["09L", "09R", "27L", "27R"] {
            let hit = ils.iter().find(|i| i.runway == rwy);
            assert!(hit.is_some(), "EGLL {rwy} ILS missing: {ils:?}");
            assert_eq!(hit.unwrap().gs_deg, Some(3.0), "{rwy}");
        }
        assert_eq!(
            ils_platform_alt_ft(&config.cifp_dir, "EGLL", "27R"),
            Some(2500.0),
            "27R platform from the CIFP FAF"
        );
    }

    #[test]
    fn none_when_no_runway_nearby() {
        let path = std::env::temp_dir().join("xc-nav-test2.dat");
        let mut f = std::fs::File::create(&path).unwrap();
        writeln!(f, " 4  47.466600000 -122.317800000  359 11075 18 123840.337 IFAR KSEA K1 28 ILS-cat-I").unwrap();
        f.flush().unwrap();
        // Aircraft far from the only runway-28 → none within 60nm.
        let r = find_ils(path.to_str().unwrap(), "28", 36.71, -4.55, 60.0).unwrap();
        assert!(r.is_none());
    }
}
