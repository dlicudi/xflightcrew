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
        ils.gs_deg = slopes
            .iter()
            .find(|(r, _)| *r == ils.runway)
            .map(|(_, a)| *a);
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

    /// How far (nm) a point is from the field's PAVEMENT: the nearest point on
    /// any runway centreline. The question "which field am I standing at" is
    /// about footprint, and [`position`](Self::position) is a poor proxy for
    /// it on a big airport: Anchorage's first runway row is 7R/25L, whose
    /// threshold lies 2.7 nm from the GA ramp north of 15/33, while Lake Hood
    /// Strip's single threshold is 0.9 nm from that same ramp. Measured by
    /// footprint the ramp is 0.56 nm from Anchorage's 15/33 and 0.91 nm from
    /// Lake Hood's strip, which is the answer a pilot would give. A field
    /// with no runway rows has no footprint and answers `None`.
    pub fn ground_distance_nm(&self, lat: f64, lon: f64) -> Option<f64> {
        let m_per_deg_lat = 111_320.0;
        let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
        let best = self
            .runways
            .iter()
            .map(|r| {
                // Flat-earth in metres around the query point: the distances
                // here are a few km at most.
                let ax = (r.lon_a - lon) * m_per_deg_lon;
                let ay = (r.lat_a - lat) * m_per_deg_lat;
                let bx = (r.lon_b - lon) * m_per_deg_lon;
                let by = (r.lat_b - lat) * m_per_deg_lat;
                let (dx, dy) = (bx - ax, by - ay);
                let len2 = dx * dx + dy * dy;
                let t = if len2 > 0.0 {
                    (-(ax * dx + ay * dy) / len2).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let (px, py) = (ax + t * dx, ay + t * dy);
                (px * px + py * py).sqrt()
            })
            .min_by(|a, b| a.total_cmp(b))?;
        Some(best / 1852.0)
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
    Ok(find_airport(path, icao)?
        .map(|a| a.runways)
        .unwrap_or_default())
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
        let Ok(line) = std::str::from_utf8(&buf) else {
            continue;
        };
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
    Ok(Some(AirportInfo {
        name,
        elev_ft,
        runways,
        freqs,
    }))
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
        let Ok(line) = std::str::from_utf8(&buf) else {
            continue;
        };
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
    /// Runway ends whose ACTIVE ZONE this edge lies inside (apt.dat 1204
    /// arrival/departure rows) — the authored equivalent of the hold-short
    /// lines: the zone boundary is where an aircraft holds. `ils` rows
    /// (the CAT/LVP holds, further out) are deliberately not collected
    /// until low-visibility ops exist.
    pub zone_of: Vec<String>,
}

/// A parking position — apt.dat row 1300 (ramp start location). The endpoint
/// an arrival taxi routes to.
#[derive(Debug, Clone)]
pub struct Stand {
    pub lat: f64,
    pub lon: f64,
    /// True heading the aircraft parks on (nose direction).
    pub heading: f64,
    /// The stand name as painted ("Stand 12", "A7", "GA North").
    pub name: String,
    /// apt.dat location type: gate / hangar / tie_down / misc. A hangar is not
    /// a suitable parking stand to send an arrival to.
    pub kind: String,
    /// Aircraft categories the stand accepts (apt.dat field: heavy / jets /
    /// turboprops / props / helos / fighters), lowercased. Empty = unspecified.
    pub accepts: Vec<String>,
}

impl Stand {
    /// A stand an arrival can actually park at — anything but a hangar.
    pub fn is_suitable(&self) -> bool {
        !self.kind.eq_ignore_ascii_case("hangar")
    }
}

/// The taxi routing network of one airport.
#[derive(Debug, Clone, Default)]
pub struct TaxiNet {
    pub nodes: std::collections::HashMap<i64, (f64, f64)>,
    pub edges: Vec<TaxiEdge>,
    /// Parking stands (apt.dat 1300), for arrival taxi-to-parking.
    pub stands: Vec<Stand>,
    /// The PAINT: every drawn taxi centreline at the field, as polylines with
    /// their beziers already sampled (apt.dat 120 linear features carrying
    /// line type 1 or 51). Empty at a field that draws none, or when the
    /// caller built the net without them.
    ///
    /// The network and the paint answer two different questions and the
    /// network is only authoritative for one of them. 1201/1202 give
    /// TOPOLOGY — what connects to what, and what it is called — and they are
    /// authored by hand as an approximation of the pavement, so their
    /// geometry can sit metres from the line the pilot is looking at. At
    /// Malaga the Baron tracked its planned route to within 0.12 m for a
    /// kilometre and taxied the whole way 1.7 to 2.8 m to the side of the
    /// drawn line, up to 6 m through the taxiway E junction (2026-08-05).
    /// It is the same lesson the runway lead-on already carries: the entry
    /// onto the strip is DRAWN, and inventing it is how an aircraft ends up
    /// in the grass. See [`TaxiNet::ride_paint`].
    pub paint: Vec<Vec<(f64, f64)>>,
}

/// How far a drawn centreline may be from the planned route and still be the
/// line that route MEANT. Sized off the error being corrected: Malaga's
/// network sits 1.7 to 2.8 m off its paint on the straights and about 6 m
/// through a junction, and the next parallel line at a normal field is a
/// taxiway width away. Wider than this is not a mis-drawn network, it is a
/// different taxiway, and moving a route there would be inventing one.
const PAINT_SNAP_M: f64 = 8.0;

/// How far off the leg's own direction a drawn line may run and still be the
/// same line. This is what stops a route being dragged onto the taxiway it is
/// crossing, or onto a hold bar drawn square across it: those run at an angle
/// no continuation of this leg does.
const PAINT_PARALLEL_DEG: f64 = 25.0;

/// Spacing of the samples a leg is broken into before snapping. The drawn
/// line curves where the network draws a corner, so a leg has to be sampled
/// to follow it at all — and the samples ARE the new route, so this is also
/// how finely the drive can be asked to trace a fillet.
const PAINT_SAMPLE_M: f64 = 5.0;

/// How far past the network's last node a stand may sit and still be driven
/// to. The routing network stops at the taxiway; the apron lead-in into a
/// stand is usually not modelled at all, so without this the drive finishes
/// at the nearest node and parks ON THE TAXIWAY — 66 m from stand W5 at
/// Malaga, across the lane the tower had just cleared somebody else down
/// (2026-08-05). Bounded because past this the "stand" is not the one this
/// route serves and driving blind across an apron is worse than stopping.
const STAND_LEAD_IN_MAX_M: f64 = 120.0;

/// ...and inside this the route already ends at the stand; anything shorter
/// is inside the tolerance of where the aircraft parks anyway.
const STAND_LEAD_IN_MIN_M: f64 = 6.0;

/// The default hold-short standoff: how far back from the runway-junction
/// node a planned route ends. Clears the painted line at GA fields
/// (30-50 m from the centreline); callers pass more for instrument
/// runways, whose holding positions sit ~100 m out.
pub const HOLD_SHORT_STANDOFF_M: f64 = 40.0;

/// How far past the runway's zone boundary a vacate drives before it is
/// finished. The mirror of [`HOLD_SHORT_STANDOFF_M`]: a departure stops that
/// far SHORT of the line, an arrival ends that far PAST it, and for the same
/// reason — the aircraft has length, and the nose being over the line is not
/// the tail being over it.
pub const VACATE_CLEAR_M: f64 = 60.0;

/// An exit more than this off the rollout heading is BEHIND us: reaching it
/// would mean taxiing back down the runway, which is worse than the longer
/// exit ahead. Abeam counts as ahead — we are stopped, so the drive's own
/// join owns the turn.
const AHEAD_OF_ABEAM_DEG: f64 = 100.0;

/// Hard stop on the vacate walk. A field whose active zones swallow half the
/// apron would otherwise send us to the terminal: this is the first hundred
/// metres off the strip, not the taxi in — the taxi-in clearance owns the
/// rest, and it is requested from wherever this ends.
const MAX_VACATE_STEPS: usize = 8;

/// How far from an authored runway edge still counts as ON the strip. A
/// vacate planned from anywhere else is a drive TOWARD a runway, not off one:
/// the caller's gate is the standing landing clearance, which holds from
/// touchdown until the taxi-in request — so a "taxi to parking" retried from
/// a taxiway after a give-up re-entered here and aimed the aircraft back at
/// the runway entrance (audit, 2026-08-02). Only judgeable where runway
/// edges are authored; thin data keeps the old behaviour, as everywhere else
/// in the vacate.
const VACATE_ON_RUNWAY_M: f64 = 60.0;

/// The most a vacate may turn off the rollout heading to leave the runway.
/// A right-angle turn-off is standard and a rapid exit is 30 to 45 degrees;
/// past this the link is one built for the OTHER end of the strip and taking
/// it means reversing on the runway (EGLL's N4W, 169 degrees off a 27L
/// rollout, 2026-09-17). Deliberately the same generosity as
/// [`AHEAD_OF_ABEAM_DEG`]: abeam is a turn, backwards is not.
const VACATE_TURN_MAX_DEG: f64 = 100.0;

/// The exit snap's distance bound — the same 3 km every other snap in this
/// file caps at (`snap_within`, `runway_entrance_near`). Without it a stale
/// clearance at one field planned a "vacate" toward a junction kilometres
/// away at another.
const MAX_EXIT_SNAP_M: f64 = 3_000.0;

/// A planned exit from the runway we have just landed on: the polyline to
/// drive, and the taxiway it puts us on.
///
/// Deliberately NOT a [`TaxiRoutePlan`]. There is no clearance here, no
/// holding point and no via list to read back — vacating is the completion of
/// the landing clearance we already hold, not a new instruction, and dressing
/// it as a cleared route would invite a readback for something nobody issued.
#[derive(Debug, Clone)]
pub struct VacatePlan {
    /// Starts AT THE AIRCRAFT, ends clear of the runway's protected zone.
    pub path: Vec<(f64, f64)>,
    /// Per-segment `(name, is_runway)`, aligned with `path` exactly as
    /// [`TaxiRoutePlan::seg`] is — the drive and the guide read both the same
    /// way.
    pub seg: Vec<(String, bool)>,
    /// The taxiway we end up on, for the crew line and the "runway vacated"
    /// report. May be empty on an unnamed link.
    pub exit_name: String,
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
    /// The tail the hold-short trim REMOVED: the authored lead-on from the
    /// route's end (the holding point) onto the runway-centreline junction —
    /// the painted yellow line from the hold onto the strip. Empty when the
    /// route wasn't trimmed (it ends at a genuine off-runway node). The
    /// line-up drives THIS instead of inventing a diagonal (the aircraft cut
    /// the Echo stub's curve and stopped crooked, LXGB 2026-07-24).
    pub lead_on: Vec<(f64, f64)>,
}

/// apt.dat painted-line types that mean "taxiway centreline": solid yellow,
/// and the same line drawn with a black border (which is what most modern
/// scenery uses). Deliberately only these two — an edge line, a hold-short
/// bar or a roadway marking is not a line to drive down, and one wrong type
/// here snaps a route sideways onto a marking that merely runs nearby.
const PAINT_CENTRELINE_TYPES: [&str; 2] = ["1", "51"];

/// Points sampled along one bezier segment of a drawn line. The curves are
/// junction fillets a few tens of metres long, so eight is smooth to well
/// under the snap tolerance and keeps the polyline cheap to project onto.
const BEZIER_STEPS: usize = 8;

/// One node of a drawn linear feature while it is being parsed.
#[derive(Clone, Copy)]
struct PaintNode {
    p: (f64, f64),
    /// The bezier control point, when the row carried one (112/114/116). The
    /// control on the far side of the node is its mirror image, which is what
    /// makes the drawn line smooth through the node.
    ctrl: Option<(f64, f64)>,
    /// Does the segment STARTING here carry a centreline line type? apt.dat
    /// attaches the type to the node the segment leaves, and a string's
    /// terminating row (115/116) carries no type at all — so this is a
    /// property of the segment, read off its first node.
    centre: bool,
}

/// Cubic bezier point at `t`.
fn bezier(p0: (f64, f64), p1: (f64, f64), p2: (f64, f64), p3: (f64, f64), t: f64) -> (f64, f64) {
    let u = 1.0 - t;
    let (a, b, c, d) = (u * u * u, 3.0 * u * u * t, 3.0 * u * t * t, t * t * t);
    (
        a * p0.0 + b * p1.0 + c * p2.0 + d * p3.0,
        a * p0.1 + b * p1.1 + c * p2.1 + d * p3.1,
    )
}

/// Turn one parsed linear feature into the centreline polylines it contains,
/// beziers sampled. A feature can mix types (a taxiway line that becomes a
/// hold bar), so this emits a run per unbroken stretch of centreline rather
/// than one polyline per feature.
fn flush_paint_feature(nodes: &mut Vec<PaintNode>, out: &mut Vec<Vec<(f64, f64)>>) {
    let mut run: Vec<(f64, f64)> = Vec::new();
    for i in 0..nodes.len().saturating_sub(1) {
        let (a, b) = (nodes[i], nodes[i + 1]);
        if !a.centre {
            if run.len() >= 2 {
                out.push(std::mem::take(&mut run));
            } else {
                run.clear();
            }
            continue;
        }
        if run.is_empty() {
            run.push(a.p);
        }
        match (a.ctrl, b.ctrl) {
            (None, None) => run.push(b.p),
            (ca, cb) => {
                let p1 = ca.unwrap_or(a.p);
                // The incoming control is the outgoing one mirrored through
                // the node — that mirror is the whole reason apt.dat stores
                // only one per node.
                let p2 = cb.map_or(b.p, |c| (2.0 * b.p.0 - c.0, 2.0 * b.p.1 - c.1));
                for k in 1..=BEZIER_STEPS {
                    run.push(bezier(a.p, p1, p2, b.p, k as f64 / BEZIER_STEPS as f64));
                }
            }
        }
    }
    if run.len() >= 2 {
        out.push(run);
    }
    nodes.clear();
}

/// A route polyline with its per-leg `(name, is_runway)`, aligned the way
/// [`TaxiRoutePlan::seg`] is: `seg[i]` describes `path[i] -> path[i+1]`.
pub type RoutePolyline = (Vec<(f64, f64)>, Vec<(String, bool)>);

/// The drawn centreline nearest `p` that could be a continuation of a leg
/// running on bearing `brg`: its index in `paint` and the foot of the
/// perpendicular. `prefer` is the line the previous sample snapped to, which
/// wins whenever it is still a candidate — without that, two lines drawn a
/// metre apart (a taxiway and the stand lead-in leaving it) swap the route
/// between them sample by sample and saw-tooth a straight leg.
fn nearest_paint(
    paint: &[Vec<(f64, f64)>],
    p: (f64, f64),
    brg: f64,
    prefer: Option<usize>,
    m_per_deg_lon: f64,
) -> Option<(usize, (f64, f64))> {
    let mut best: Option<(f64, usize, (f64, f64))> = None;
    let mut best_prefer: Option<(f64, usize, (f64, f64))> = None;
    for (idx, line) in paint.iter().enumerate() {
        for w in line.windows(2) {
            let (ax, ay) = ((w[0].1 - p.1) * m_per_deg_lon, (w[0].0 - p.0) * 111_320.0);
            let (bx, by) = ((w[1].1 - p.1) * m_per_deg_lon, (w[1].0 - p.0) * 111_320.0);
            let (dx, dy) = (bx - ax, by - ay);
            let len2 = dx * dx + dy * dy;
            if len2 < 1e-6 {
                continue;
            }
            let t = (-(ax * dx + ay * dy) / len2).clamp(0.0, 1.0);
            let (fx, fy) = (ax + t * dx, ay + t * dy);
            let d = (fx * fx + fy * fy).sqrt();
            if d > PAINT_SNAP_M {
                continue;
            }
            // Undirected: a line drawn from the other end is the same line.
            let gap = (dx.atan2(dy).to_degrees() - brg).rem_euclid(180.0);
            if gap.min(180.0 - gap) > PAINT_PARALLEL_DEG {
                continue;
            }
            let foot = (p.0 + fy / 111_320.0, p.1 + fx / m_per_deg_lon);
            if best.is_none_or(|(bd, _, _)| d < bd) {
                best = Some((d, idx, foot));
            }
            if prefer == Some(idx) && best_prefer.is_none_or(|(bd, _, _)| d < bd) {
                best_prefer = Some((d, idx, foot));
            }
        }
    }
    best_prefer.or(best).map(|(_, idx, foot)| (idx, foot))
}

/// Move a planned route onto the paint it is an approximation of, returning
/// the resampled `(path, seg)`.
///
/// The route out of the planner is a chain of 1201 nodes: the right taxiways
/// in the right order, drawn by hand to within a few metres of the yellow
/// line. Following it exactly — which the drive does, to 0.12 m — puts the
/// aircraft a couple of metres to one side of the line for the whole taxi,
/// which is what the pilot sees and what a ground crew would fail you for.
/// So each leg is sampled and each sample slid sideways onto the drawn
/// centreline, when there is one running the same way close enough to be the
/// same line.
///
/// It can only ever move a point `PAINT_SNAP_M` sideways onto a line the leg
/// was already following, never re-route: the topology, the names, the
/// crossings and the holding point are the network's and are untouched. A
/// field that draws no centrelines (or a net built without them) gets its
/// route back unchanged, not a refusal.
///
/// Runway legs are left alone. A crossing runs square across every taxi line
/// near it, the strip's own markings are not 120 features anyway, and the
/// runway geometry is authoritative where the taxi network is not.
pub fn ride_paint(
    paint: &[Vec<(f64, f64)>],
    path: &[(f64, f64)],
    seg: &[(String, bool)],
) -> RoutePolyline {
    if paint.is_empty() || path.len() < 2 {
        return (path.to_vec(), seg.to_vec());
    }
    let m_per_deg_lon = 111_320.0 * path[0].0.to_radians().cos();
    let dist_m = |a: (f64, f64), b: (f64, f64)| {
        let (dx, dy) = ((a.1 - b.1) * m_per_deg_lon, (a.0 - b.0) * 111_320.0);
        (dx * dx + dy * dy).sqrt()
    };
    let mut out_path: Vec<(f64, f64)> = Vec::with_capacity(path.len() * 4);
    let mut out_seg: Vec<(String, bool)> = Vec::with_capacity(path.len() * 4);
    let mut prefer: Option<usize> = None;
    for i in 0..path.len() - 1 {
        let (a, b) = (path[i], path[i + 1]);
        let s = seg.get(i).cloned().unwrap_or_default();
        let len = dist_m(a, b);
        if s.1 || len < 0.5 {
            out_path.push(a);
            out_seg.push(s);
            prefer = None;
            continue;
        }
        let brg = bearing_deg(a.0, a.1, b.0, b.1);
        let n = ((len / PAINT_SAMPLE_M).round() as usize).max(1);
        for k in 0..n {
            let t = k as f64 / n as f64;
            let raw = (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t);
            let snapped = match nearest_paint(paint, raw, brg, prefer, m_per_deg_lon) {
                Some((idx, q)) => {
                    prefer = Some(idx);
                    q
                }
                None => raw,
            };
            // Progress along the route must stay monotonic: every consumer
            // projects onto this polyline and reads how far along it is, and
            // a sample slid BACKWARDS past the one before it would read as
            // the aircraft losing ground. Nothing in the gates above forbids
            // it outright, so the cheap answer is to keep the raw sample —
            // one unsnapped point in the middle of a snapped run is invisible.
            let keeps_going = out_path
                .last()
                .is_none_or(|&prev| dist_m(prev, snapped) > 0.25 && {
                    let step = bearing_deg(prev.0, prev.1, snapped.0, snapped.1);
                    let gap = (step - brg).rem_euclid(360.0);
                    gap.min(360.0 - gap) < 90.0
                });
            out_path.push(if keeps_going { snapped } else { raw });
            out_seg.push(s.clone());
        }
    }
    // The end point rides the last leg's bearing, so a route that finishes at
    // a holding point finishes ON the line, which is where it will stop.
    let last = path[path.len() - 1];
    let prev = path[path.len() - 2];
    let end = if seg.last().is_some_and(|s| s.1) {
        last
    } else {
        let brg = bearing_deg(prev.0, prev.1, last.0, last.1);
        nearest_paint(paint, last, brg, prefer, m_per_deg_lon).map_or(last, |(_, q)| q)
    };
    out_path.push(end);
    (out_path, out_seg)
}

/// Parse `icao`'s taxi network from apt.dat — same streaming block scan as
/// [`find_airport`], collecting 1201 (node) and 1202 (edge) rows, plus the
/// drawn taxi centrelines (120 linear features) the routing geometry is only
/// an approximation of. `None` when the field has no network (small strips
/// mostly don't) — the paint alone is not a network, since it carries no
/// connectivity and no names.
pub fn find_taxi_net(path: &str, icao: &str) -> Result<Option<TaxiNet>> {
    let file = std::fs::File::open(path).with_context(|| format!("open {path}"))?;
    let mut reader = BufReader::with_capacity(1 << 20, file);
    let mut net = TaxiNet::default();
    let mut in_airport = false;
    let mut buf = Vec::with_capacity(256);
    // The linear feature being read. Nodes only count as PAINT inside a 120
    // block: the same 111-116 rows draw pavement boundaries (110) and the
    // airport perimeter (130), and a boundary is not a line to taxi down.
    let mut in_feature = false;
    let mut feature: Vec<PaintNode> = Vec::new();
    loop {
        buf.clear();
        if reader.read_until(b'\n', &mut buf).unwrap_or(0) == 0 {
            break;
        }
        if buf.first() != Some(&b'1') {
            continue;
        }
        let Ok(line) = std::str::from_utf8(&buf) else {
            continue;
        };
        let mut f = line.split_whitespace();
        let row = f.next();
        // Any row that opens a new block ends the feature being read.
        if !matches!(row, Some("111" | "112" | "113" | "114" | "115" | "116")) {
            flush_paint_feature(&mut feature, &mut net.paint);
            in_feature = in_airport && row == Some("120");
        }
        match row {
            Some("111" | "112" | "113" | "114" | "115" | "116") if in_feature => {
                let (Some(lat), Some(lon)) = (
                    f.next().and_then(|s| s.parse::<f64>().ok()),
                    f.next().and_then(|s| s.parse::<f64>().ok()),
                ) else {
                    continue;
                };
                // 112/114/116 carry the bezier control point before the
                // attributes; the plain rows go straight to them.
                let ctrl = if matches!(row, Some("112" | "114" | "116")) {
                    match (
                        f.next().and_then(|s| s.parse::<f64>().ok()),
                        f.next().and_then(|s| s.parse::<f64>().ok()),
                    ) {
                        (Some(a), Some(b)) => Some((a, b)),
                        _ => None,
                    }
                } else {
                    None
                };
                let centre = f.any(|a| PAINT_CENTRELINE_TYPES.contains(&a));
                feature.push(PaintNode {
                    p: (lat, lon),
                    ctrl,
                    centre,
                });
                // 113/114 close a loop: the drawn line returns to its start.
                if matches!(row, Some("113" | "114")) {
                    if let Some(&first) = feature.first() {
                        feature.push(first);
                    }
                    flush_paint_feature(&mut feature, &mut net.paint);
                    in_feature = false;
                } else if matches!(row, Some("115" | "116")) {
                    flush_paint_feature(&mut feature, &mut net.paint);
                    in_feature = false;
                }
            }
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
                    zone_of: Vec::new(),
                });
            }
            Some("1300") if in_airport => {
                // lat lon heading loc_type plane_types name...
                // e.g. `1300 52.30 -1.20 90.0 gate jets|turboprops Stand 12`
                let (lat, lon, heading) = match (
                    f.next().and_then(|s| s.parse::<f64>().ok()),
                    f.next().and_then(|s| s.parse::<f64>().ok()),
                    f.next().and_then(|s| s.parse::<f64>().ok()),
                ) {
                    (Some(a), Some(b), Some(h)) => (a, b, h),
                    _ => continue,
                };
                let kind = f.next().unwrap_or("misc").to_string();
                let accepts = f
                    .next()
                    .unwrap_or("")
                    .split('|')
                    .map(|s| s.trim().to_ascii_lowercase())
                    .filter(|s| !s.is_empty())
                    .collect();
                let name = f.collect::<Vec<_>>().join(" ").trim().to_string();
                net.stands.push(Stand {
                    lat,
                    lon,
                    heading,
                    name,
                    kind,
                    accepts,
                });
            }
            Some("1204") if in_airport => {
                // classification runway-list — attaches to the 1202 row
                // above it. arrival/departure/both zones bound the runway
                // holding points; `ils` zones are the LVP holds, skipped.
                let class = f.next().unwrap_or("");
                if class == "arrival" || class == "departure" || class == "both" {
                    if let (Some(list), Some(e)) = (f.next(), net.edges.last_mut()) {
                        for ident in list.split(',').map(str::trim).filter(|s| !s.is_empty()) {
                            if !e.zone_of.iter().any(|z| z == ident) {
                                e.zone_of.push(ident.to_string());
                            }
                        }
                    }
                }
            }
            _ => {}
        }
    }
    Ok((!net.edges.is_empty()).then_some(net))
}

impl TaxiNet {
    /// The nearest SUITABLE parking stand (excludes hangars) by straight-line
    /// distance — the destination an arrival taxi routes to. `None` when the
    /// field publishes no stands.
    pub fn nearest_stand(&self, lat: f64, lon: f64) -> Option<&Stand> {
        let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
        let d2 = |s: &Stand| {
            let dx = (s.lon - lon) * m_per_deg_lon;
            let dy = (s.lat - lat) * 111_320.0;
            dx * dx + dy * dy
        };
        self.stands
            .iter()
            .filter(|s| s.is_suitable())
            .min_by(|a, b| d2(a).total_cmp(&d2(b)))
    }

    /// [`plan`](Self::plan) to a STAND — the arrival mirror of
    /// [`plan_to_runway`](Self::plan_to_runway), and it exists for the same
    /// reason: the network's nearest node is not the destination.
    ///
    /// A departure's route ends short of the runway ON PURPOSE and the drawn
    /// lead-on carries it the rest of the way. An arrival's ended short by
    /// ACCIDENT, because apt.dat routes taxiways and mostly does not model
    /// the apron: the plain snap put the last point at whatever node lay
    /// nearest the stand and the drive stopped there, which at Malaga was
    /// 66 m short of stand W5 and parked across a live taxiway — twice, the
    /// tower having named the stand both times (2026-08-05).
    ///
    /// So the route is continued from that node to the stand itself, and the
    /// continuation rides the paint like everything else: the lead-in to a
    /// stand IS a drawn line at any field that paints one, and where none is
    /// drawn the straight leg is what the pavement is for. Bounded by
    /// [`STAND_LEAD_IN_MAX_M`] — a stand further out than that is not the one
    /// this route serves, and the old stop-at-the-node behaviour is better
    /// than inventing a drive across an unseen apron.
    pub fn plan_to_stand(&self, from_lat: f64, from_lon: f64, stand: &Stand) -> Option<TaxiRoutePlan> {
        let mut plan = self.plan(from_lat, from_lon, stand.lat, stand.lon, 0.0, &[])?;
        let end = *plan.path.last()?;
        let m_per_deg_lon = 111_320.0 * stand.lat.to_radians().cos();
        let gap = {
            let (dx, dy) = (
                (stand.lon - end.1) * m_per_deg_lon,
                (stand.lat - end.0) * 111_320.0,
            );
            (dx * dx + dy * dy).sqrt()
        };
        if !(STAND_LEAD_IN_MIN_M..=STAND_LEAD_IN_MAX_M).contains(&gap) {
            return Some(plan);
        }
        // Sampled and snapped exactly like a planned leg, so a drawn lead-in
        // is followed where there is one. The leg carries no NAME: an apron
        // lead-in is not a taxiway, and the guide would otherwise invent a
        // turn "onto" something the pilot cannot see on the chart.
        let (lead, lead_seg) = ride_paint(
            &self.paint,
            &[end, (stand.lat, stand.lon)],
            &[(String::new(), false)],
        );
        plan.path.extend(lead.into_iter().skip(1));
        plan.seg.extend(lead_seg);
        Some(plan)
    }

    /// Route from a position to a position (a stand to a runway threshold)
    /// and extract the phrasable plan. `None` when either end doesn't snap
    /// to the network (within 3 km) or no path exists.
    ///
    /// The tail is ended at the destination runway's HOLD-SHORT position,
    /// best data first: `hold_zone` names the runway ends whose authored
    /// active zone (apt.dat 1204 rows) the route must stop at the edge of —
    /// the per-stub truth when the airport carries it. Without zone data,
    /// `hold_short_m` is the geometric fallback: how far back from a
    /// runway-junction tail the route ends ([`HOLD_SHORT_STANDOFF_M`]
    /// unless the caller knows the runway's class says otherwise —
    /// instrument holds sit further out).
    pub fn plan(
        &self,
        from_lat: f64,
        from_lon: f64,
        to_lat: f64,
        to_lon: f64,
        hold_short_m: f64,
        hold_zone: &[String],
    ) -> Option<TaxiRoutePlan> {
        self.plan_inner(
            from_lat,
            from_lon,
            to_lat,
            to_lon,
            hold_short_m,
            hold_zone,
            false,
        )
    }

    /// [`plan`](Self::plan) to a RUNWAY — the destination is a threshold, and
    /// the route must end somewhere the aircraft can get onto the strip from.
    ///
    /// Split from `plan` rather than inferred from the geometry because the
    /// callers genuinely differ and only they know: the departure planner is
    /// handed a threshold, taxi-in is handed a stand, and the off-route replan
    /// is handed the standing route's already-trimmed holding point — which
    /// sits a hold-short standoff from the runway and must NOT be dragged back
    /// onto the junction the trim just removed.
    pub fn plan_to_runway(
        &self,
        from_lat: f64,
        from_lon: f64,
        to_lat: f64,
        to_lon: f64,
        hold_short_m: f64,
        hold_zone: &[String],
    ) -> Option<TaxiRoutePlan> {
        self.plan_inner(
            from_lat,
            from_lon,
            to_lat,
            to_lon,
            hold_short_m,
            hold_zone,
            true,
        )
    }

    /// Plan the drive OFF the runway we have just landed on.
    ///
    /// The arrival mirror of `plan_to_runway`, and it reuses that work rather
    /// than inventing a second idea of where a runway meets a taxiway: an
    /// ENTRANCE and an EXIT are the same node, decided by the same
    /// connectivity relation (on a runway edge, or one hop from one) — only
    /// the direction of travel differs. And the zone walk below is the
    /// hold-short trim run backwards: the trim DROPS the trailing edges inside
    /// the runway's protected zone so a departure stops short of the line;
    /// this KEEPS going until it is `clear_m` past the same boundary, because
    /// the tail has to be over it too.
    ///
    /// Why this exists at all: the taxi-in used to transmit its request from
    /// wherever autoland stopped and wait up to sixty seconds for a route,
    /// sitting on the strip the whole time — and on the give-up path it handed
    /// back "your aircraft" still on the runway. Vacating needs no clearance
    /// from anybody. It is the completion of the landing clearance we already
    /// hold, so it happens first and the request comes from the taxiway, which
    /// is the order a real crew flies.
    ///
    /// `psi_true` is the rollout direction: exits BEHIND us are refused, since
    /// reaching one means taxiing back down the runway, which is worse than
    /// the longer exit ahead. `zone` names the runway ends whose active zone
    /// we want to be clear of.
    ///
    /// Never refuses for want of data — a field with no authored runway edges
    /// (776 ends worldwide) falls back to the nearest taxiway node ahead, and
    /// only a field with no network at all returns `None`.
    pub fn plan_vacate(
        &self,
        lat: f64,
        lon: f64,
        psi_true: f64,
        zone: &[String],
        clear_m: f64,
    ) -> Option<VacatePlan> {
        let m_per_deg_lat = 111_320.0;
        let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
        let dist_m = |a: (f64, f64), b: (f64, f64)| -> f64 {
            let dx = (a.1 - b.1) * m_per_deg_lon;
            let dy = (a.0 - b.0) * m_per_deg_lat;
            (dx * dx + dy * dy).sqrt()
        };
        let here = (lat, lon);
        // Ahead of abeam, measured from the rollout heading. We are stopped, so
        // the drive's own join owns the turn; this only rules out the exits
        // that would send us back down the strip.
        let ahead = |p: (f64, f64)| -> bool {
            let brg = bearing_deg(lat, lon, p.0, p.1);
            angle_diff_deg(brg, psi_true).abs() <= AHEAD_OF_ABEAM_DEG
        };

        let mut taxiway_nodes: std::collections::HashSet<i64> = std::collections::HashSet::new();
        for e in self.edges.iter().filter(|e| !e.runway) {
            taxiway_nodes.insert(e.a);
            taxiway_nodes.insert(e.b);
        }
        if taxiway_nodes.is_empty() {
            return None;
        }
        // Only plan a vacate FROM the strip. The caller's gate is the standing
        // landing clearance, which holds from touchdown to the taxi-in
        // request — position is what says the vacate already happened. Where
        // runway edges are authored, an aircraft nowhere near one has nothing
        // to vacate, and the nearest-junction-ahead snap below would aim it
        // BACK at the runway entrance. Thin data (no runway edges) cannot
        // judge this and keeps the old behaviour.
        let seg_dist_m = |p: (f64, f64), a: (f64, f64), b: (f64, f64)| -> f64 {
            let (px, py) = ((p.1 - a.1) * m_per_deg_lon, (p.0 - a.0) * m_per_deg_lat);
            let (bx, by) = ((b.1 - a.1) * m_per_deg_lon, (b.0 - a.0) * m_per_deg_lat);
            let len2 = bx * bx + by * by;
            let t = if len2 > 0.0 {
                ((px * bx + py * by) / len2).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let (dx, dy) = (px - t * bx, py - t * by);
            (dx * dx + dy * dy).sqrt()
        };
        let mut has_runway_edges = false;
        let mut on_strip = false;
        for e in self.edges.iter().filter(|e| e.runway) {
            has_runway_edges = true;
            if let (Some(&a), Some(&b)) = (self.nodes.get(&e.a), self.nodes.get(&e.b)) {
                if seg_dist_m(here, a, b) <= VACATE_ON_RUNWAY_M {
                    on_strip = true;
                    break;
                }
            }
        }
        if has_runway_edges && !on_strip {
            return None;
        }
        // Candidate exits: runway nodes that a taxiway also touches — the
        // JUNCTION where the two meet.
        //
        // This is where an exit and an entrance stop being the same thing.
        // `runway_entrance_near` accepts a node one hop OFF the runway,
        // because a departure holds short there and the drawn lead-on carries
        // it the rest of the way. An arrival cannot use that node as its
        // decision point: measuring "is this exit ahead of me" at the far end
        // of a link says nothing about where the link MEETS the runway, and a
        // junction 22 m behind the aircraft reads as an exit 150 m off to the
        // side. Southbound on the test net that picked the turn-off we had
        // just rolled past. The junction is the exit; the walk below carries
        // us onto the taxiway from there.
        //
        // When the field authors no runway edges at all there is nothing to
        // filter on, so every taxiway node is a candidate and the nearest one
        // ahead wins — the same degradation `plan_to_runway` makes.
        let mut runway_nodes: std::collections::HashSet<i64> = std::collections::HashSet::new();
        for e in self.edges.iter().filter(|e| e.runway) {
            runway_nodes.insert(e.a);
            runway_nodes.insert(e.b);
        }
        // How a link is ranked against the zones it lies in. Shared by the
        // turn-off test and the walk so both read the pavement the same way.
        let zone_rank = |e: &TaxiEdge| -> u8 {
            if e.zone_of.is_empty() {
                0
            } else if e.zone_of.iter().all(|z| zone.contains(z)) {
                1
            } else {
                2
            }
        };
        // The turn-off a junction actually offers this rollout: the best
        // traversable taxiway leaving it that we can TURN ONTO, or `None`.
        //
        // A junction being ahead is not enough. A rapid-exit taxiway is
        // angled toward the traffic it serves, so the ones built for the
        // OTHER end of the strip leave the same junction pointing backwards.
        // EGLL's N4W meets 27L eighty-four metres ahead and runs back
        // east-north-east, 169 degrees off the landing direction: the vacate
        // took it, drove the 737 up the runway, turned it around ON the strip
        // and drove it back past its own touchdown point — 624 m of route and
        // three minutes of occupancy with traffic inside a mile (2026-09-17).
        // The turn-off that works is 220 m further on (N5W, 54 degrees off).
        // More distance down a runway we are already rolling along is cheap;
        // a reversal on it is not.
        let turn_off = |id: i64, p: (f64, f64)| -> Option<(i64, (f64, f64), String, u8)> {
            self.edges
                .iter()
                .filter(|e| !e.runway)
                .filter_map(|e| {
                    let nxt = if e.a == id {
                        e.b
                    } else if e.b == id && !e.oneway {
                        e.a
                    } else {
                        return None;
                    };
                    let np = *self.nodes.get(&nxt)?;
                    let brg = bearing_deg(p.0, p.1, np.0, np.1);
                    if angle_diff_deg(brg, psi_true).abs() > VACATE_TURN_MAX_DEG {
                        return None;
                    }
                    Some((nxt, np, e.name.clone(), zone_rank(e)))
                })
                .min_by(|x, y| {
                    x.3.cmp(&y.3)
                        .then_with(|| dist_m(here, y.1).total_cmp(&dist_m(here, x.1)))
                })
        };
        let (exit, first) = taxiway_nodes
            .iter()
            .filter(|id| runway_nodes.is_empty() || runway_nodes.contains(id))
            .filter_map(|id| self.nodes.get(id).map(|&p| (*id, p)))
            .filter(|(_, p)| ahead(*p))
            // Bounded like every other snap in this file: a junction
            // kilometres away is another airport's, not our exit.
            .filter(|(_, p)| dist_m(here, *p) <= MAX_EXIT_SNAP_M)
            // Only a junction we can leave. A field whose every turn-off is
            // authored backwards yields no plan at all, and the caller falls
            // through to the taxi-in from the runway — the same degradation
            // the dead-end case makes, and better than the U-turn.
            .filter_map(|(id, p)| turn_off(id, p).map(|s| (id, p, s)))
            .min_by(|x, y| dist_m(here, x.1).total_cmp(&dist_m(here, y.1)))
            .map(|(id, _, step)| (id, step))?;

        // Walk outward until `clear_m` past the exit, never along a runway
        // edge and never back through a node we have used. "Outward" is
        // measured from the AIRCRAFT, which is on the runway — so maximising
        // it is literally "get away from the runway", which is the whole job.
        let mut path_ids = vec![exit];
        let mut seen: std::collections::HashSet<i64> = std::collections::HashSet::new();
        seen.insert(exit);
        let mut names: Vec<String> = Vec::new();
        // Distance travelled OUTSIDE the zone. Metres covered while still
        // inside it earn no credit, which is the whole point: the link from
        // the runway onto the parallel taxiway is itself a zone edge, so
        // counting its length would finish the drive at the far end of it —
        // and by the hold-short trim's own logic that far node IS the holding
        // point, i.e. exactly the line we are trying to get the tail over.
        let mut clear_gone = 0.0;
        // A field whose zones swallow the apron would otherwise walk us to the
        // terminal; the vacate is the first hundred metres, not the taxi in.
        let mut left_zone = zone.is_empty();
        // The first step is the turn-off this exit was CHOSEN for, not
        // whatever lies furthest from the aircraft: at a junction offering
        // both, "furthest" is the link angled back down the strip.
        {
            let (nxt, p, name, rank) = first;
            let cur_p = *self.nodes.get(&exit)?;
            if rank == 0 {
                left_zone = true;
            }
            if left_zone {
                clear_gone += dist_m(cur_p, p);
            }
            path_ids.push(nxt);
            seen.insert(nxt);
            names.push(name);
        }
        while clear_gone < clear_m && names.len() < MAX_VACATE_STEPS {
            let cur = *path_ids.last()?;
            let cur_p = *self.nodes.get(&cur)?;
            let step = self
                .edges
                .iter()
                .filter(|e| !e.runway)
                .filter_map(|e| {
                    let nxt = if e.a == cur {
                        e.b
                    } else if e.b == cur && !e.oneway {
                        e.a
                    } else {
                        return None;
                    };
                    if seen.contains(&nxt) {
                        return None;
                    }
                    // Leaving protected pavement is the point: rank clean
                    // edges first, then our own runway's zone, then — last of
                    // all — an edge inside ANOTHER runway's zone. `zone` only
                    // carries the runway we landed on, so a plain
                    // "in-our-zone" test read a link crossing an intersecting
                    // runway as a way OUT and actively preferred driving
                    // across the other strip, with the metres counted as
                    // clearance (audit, 2026-08-02).
                    self.nodes
                        .get(&nxt)
                        .map(|&p| (nxt, p, e.name.clone(), zone_rank(e)))
                })
                .min_by(|x, y| {
                    // Cleanest pavement first, then whichever gets us furthest
                    // from the runway we are standing on.
                    x.3.cmp(&y.3)
                        .then_with(|| dist_m(here, y.1).total_cmp(&dist_m(here, x.1)))
                });
            let Some((nxt, p, name, zone_rank)) = step else {
                break;
            };
            if zone_rank == 0 {
                left_zone = true;
            }
            if left_zone {
                clear_gone += dist_m(cur_p, p);
            }
            path_ids.push(nxt);
            seen.insert(nxt);
            names.push(name);
        }

        // A walk that never moved found no traversable taxiway edge at the
        // junction — a one-way stub authored toward the runway is enough. The
        // "plan" would be a single leg from the aircraft to a node ON the
        // centreline: a drive down the runway reported as a vacate, with
        // "runway vacated" already owed from the strip. No plan is strictly
        // better — the caller falls through to the taxi-in from the runway,
        // exactly as it behaved before the vacate existed (audit, 2026-08-02,
        // demonstrated by the dead-end test below).
        if names.is_empty() {
            return None;
        }
        // The path starts AT THE AIRCRAFT — the same rule the stand route
        // learned the hard way, because a drive whose route begins 200 m away
        // reads as already-arrived on its first tick.
        let mut path = vec![here];
        for id in &path_ids {
            path.push(*self.nodes.get(id)?);
        }
        // The first leg is the one off the runway itself; it has no authored
        // name, and calling it by the runway's would be a lie.
        let mut seg = vec![(String::new(), true)];
        for n in &names {
            seg.push((n.clone(), false));
        }
        seg.truncate(path.len().saturating_sub(1));
        // Where we END UP, not the link we turned onto. The walk usually
        // crosses a short stub inside the zone before reaching the parallel
        // taxiway, and it is the latter the crew reports being clear on and
        // the taxi-in plans from.
        let exit_name = names
            .iter()
            .rev()
            .find(|n| !n.is_empty())
            .cloned()
            .unwrap_or_default();
        // Onto the paint, like the departure route. The first leg is flagged
        // as the runway one, so the snap leaves it — and with it the start
        // point, which is the aircraft's own position and the one point in
        // any route that must never be moved.
        let (path, seg) = ride_paint(&self.paint, &path, &seg);
        (path.len() >= 2).then_some(VacatePlan {
            path,
            seg,
            exit_name,
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn plan_inner(
        &self,
        from_lat: f64,
        from_lon: f64,
        to_lat: f64,
        to_lon: f64,
        hold_short_m: f64,
        hold_zone: &[String],
        to_runway: bool,
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
        let snap_within = |lat: f64, lon: f64, from: &std::collections::HashSet<i64>| {
            from.iter()
                .filter_map(|id| self.nodes.get(id).map(|&p| (*id, dist_m((lat, lon), p))))
                .min_by(|x, y| x.1.total_cmp(&y.1))
                .filter(|(_, d)| *d < 3000.0)
                .map(|(id, _)| id)
        };
        let start = snap_within(from_lat, from_lon, &taxiway_nodes)?;
        // A runway destination must snap to a node that can actually REACH the
        // runway, not merely to the taxiway node that happens to lie nearest
        // the threshold. Nearest-anything is how EGHI's runway 20 became a
        // clearance to nowhere (2026-07-30): the north end of the parallel
        // taxiway A5 is a dead-end stub 172 m abeam the strip, and being the
        // closest taxiway node to the threshold (631 m, against 1047 m for the
        // nearest real entrance) it won the snap. The tower cleared us to
        // "holding point Alpha five runway two zero", the drive stopped at a
        // stub connected to nothing, and because the goal touched no runway
        // edge the hold-short trim never ran and `lead_on` came out empty —
        // leaving the line-up to invent an entrance, which is the trap the
        // drawn lead-on exists to close. Across the whole apt.dat this is a
        // class, not a field: 3,498 of 21,656 runway ends snap to a
        // non-entrance, 551 of them badly (EFHK 04R, ENBO 07, ESNN 34).
        let goal = to_runway
            .then(|| self.runway_entrance_near(to_lat, to_lon, &taxiway_nodes))
            .flatten()
            .or_else(|| snap_within(to_lat, to_lon, &taxiway_nodes))?;
        if start == goal {
            // For a RUNWAY destination this is genuinely nothing to do: we are
            // already at the entrance the clearance would send us to.
            if to_runway {
                return None;
            }
            // For a STAND it is not. A sparse network puts both ends on one
            // node whenever the parking is close, and returning None there
            // means the tower answers "taxi to parking, this frequency" with
            // no route attached — the crew's taxi-in then waits out its whole
            // 60 s for a route that was never coming and gives up with "no
            // taxi-in clearance came back", which is a lie: the clearance
            // came, the ROUTE did not (EGHI 2026-07-30, aircraft 224 m from
            // node 44 and Stand 14 another 69 m past it, on a 24-edge
            // network).
            //
            // The honest answer is the short taxi it actually is: out to the
            // node, then in to the stand. No named legs, because none were
            // traversed.
            // The path must START AT THE AIRCRAFT. A normal route begins at
            // its snapped node and the drive JOINS to it, which is fine when
            // the aircraft is beside that node — but here the whole point is
            // that one node serves both ends, and the aircraft can be a long
            // way from it. First cut began at the node: at EGHI that put the
            // route's start 224 m from an aircraft that was stopped, so the
            // drive's arrival test ("within a metre and under 1.5 kt") was
            // true on the first tick and taxi_in went from "driving" to
            // "secured" in two seconds without moving.
            let node = *self.nodes.get(&start)?;
            let here = (from_lat, from_lon);
            let dest = (to_lat, to_lon);
            // Route VIA the node when it is roughly on the way — it sits on
            // the taxiway, which is where we would rather be — but not when
            // doubling back to it would be the long way round.
            let via_node =
                dist_m(here, node) + dist_m(node, dest) <= 1.6 * dist_m(here, dest).max(1.0);
            let path = if via_node {
                vec![here, node, dest]
            } else {
                vec![here, dest]
            };
            let seg = vec![(String::new(), false); path.len() - 1];
            return Some(TaxiRoutePlan {
                via: Vec::new(),
                crossings: Vec::new(),
                holding_point: String::new(),
                seg,
                path,
                lead_on: Vec::new(),
            });
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
            let (_, Some(ei)) = *best.get(&node)? else {
                return None;
            };
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
        // End the route at the destination's hold-short position — the goal
        // node is usually the taxiway/runway JUNCTION (apt.dat networks
        // join taxiways to the runway centreline), and a route ending there
        // put the auto-taxi stop (and the guide's "holding point ahead,
        // that's us") across the hold line (2026-07-20).
        //
        // Best data first: when the airport authors ACTIVE ZONES (1204
        // rows) for the destination runway, drop the trailing edges inside
        // its zone — the boundary node IS the authored holding point,
        // per stub (EGLL carries these for every entry; the geometric 100 m
        // guess was still wrong stub-to-stub).
        // The untrimmed path, kept so the trimmed tail — the authored lead-on
        // from the holding point onto the runway centreline — survives on the
        // plan for the line-up to drive.
        let full = path.clone();
        let zone_edges = if hold_zone.is_empty() {
            0
        } else {
            edges_rev
                .iter()
                .rev()
                .take_while(|&&ei| self.edges[ei].zone_of.iter().any(|z| hold_zone.contains(z)))
                .count()
                .min(path.len().saturating_sub(2))
        };
        if zone_edges > 0 {
            path.truncate(path.len() - zone_edges);
            seg.truncate(seg.len() - zone_edges);
        }
        // Geometric fallback: when the last node touches a runway edge,
        // pull the tail back by a hold-short standoff so the route ends
        // where an aircraft should STOP. A network whose tail ends at a
        // genuine holding-point node off the runway keeps its geometry.
        // The distance is the caller's to judge (instrument runways paint
        // their lines much further out than GA fields) — an approximation
        // either way, which is why authored zones win above.
        let ends_on_runway = self
            .edges
            .iter()
            .any(|e| e.runway && (e.a == goal || e.b == goal));
        let total: f64 = path.windows(2).map(|w| dist_m(w[0], w[1])).sum();
        if zone_edges == 0 && ends_on_runway && total > hold_short_m + 30.0 {
            let mut cut = 0.0;
            while path.len() >= 3 {
                let n = path.len();
                let leg = dist_m(path[n - 2], path[n - 1]);
                if cut + leg > hold_short_m {
                    // The stop point lands inside this leg: move the end
                    // node back along it (or drop it when only a sliver
                    // would remain).
                    let keep = 1.0 - (hold_short_m - cut) / leg;
                    if keep > 0.05 {
                        let (a, b) = (path[n - 2], path[n - 1]);
                        path[n - 1] = (a.0 + (b.0 - a.0) * keep, a.1 + (b.1 - a.1) * keep);
                    } else {
                        path.pop();
                        seg.pop();
                    }
                    break;
                }
                cut += leg;
                path.pop();
                seg.pop();
            }
        }
        // The lead-on: whatever the trims removed, from the (possibly moved)
        // hold point out to the runway-junction node. It STARTS at the
        // trimmed route's own end, always: the zone trim and the sliver-pop
        // truncate at a node, which leaves the trimmed path a strict prefix
        // of the full one — the old longest-common-prefix arithmetic then
        // skipped the holding point itself, and a one-edge zone tail (the
        // per-stub authoring EGLL uses) degenerated the lead-on to a single
        // point. Then the original tail follows from the first point the
        // trims touched. Empty when nothing was trimmed.
        let trimmed = full.len() != path.len() || full.last() != path.last();
        let mut lead_on: Vec<(f64, f64)> = Vec::new();
        if trimmed {
            lead_on.extend(path.last().copied());
            let prefix = full
                .iter()
                .zip(path.iter())
                .take_while(|(a, b)| a == b)
                .count();
            lead_on.extend_from_slice(&full[prefix..]);
        }
        // Onto the paint, now the route is trimmed and the lead-on split off:
        // the network chose the taxiways, the drawn line says where they run.
        // Both halves ride it — the lead-on IS a painted line (the yellow one
        // from the holding point onto the strip), and the line-up drives it.
        (path, seg) = ride_paint(&self.paint, &path, &seg);
        let lead_seg = vec![(String::new(), false); lead_on.len().saturating_sub(1)];
        lead_on = ride_paint(&self.paint, &lead_on, &lead_seg).0;
        // The route starts where the AIRCRAFT is, not at the node it snapped
        // to: the apron leg from the stand to the network is part of the
        // cleared route, so the off-route watch and the taxi guide measure
        // from brake release (a Malaga stand exit was berated as off-route
        // 2 s after rolling, 100 m from a route it had no way to be on yet,
        // 2026-07-13).
        if let Some(&first) = path.first() {
            if dist_m((from_lat, from_lon), first) > 1.0 {
                path.insert(0, (from_lat, from_lon));
                seg.insert(0, (String::new(), false));
            }
        }
        Some(TaxiRoutePlan {
            via,
            crossings,
            holding_point,
            path,
            seg,
            lead_on,
        })
    }

    /// The taxiway node nearest `(lat, lon)` that can actually get onto a
    /// runway from there. `None` when the field's network models no entrance at
    /// all, which leaves [`plan_to_runway`](Self::plan_to_runway) on the plain
    /// nearest-node snap rather than refusing a route the old code would fly.
    ///
    /// "Entrance" is decided by CONNECTIVITY, never by edge names: apt.dat
    /// authors leave runway edges unnamed often enough that
    /// [`mark_runway_crossings`](crate::station) already refuses to trust the
    /// flags for the same reason. A node qualifies if it sits on a runway edge,
    /// or one hop from a node that does — which is exactly how a network draws
    /// the stub from a holding point out to the centreline.
    ///
    /// Nearest wins among entrances, with no check that the entrance belongs to
    /// the runway asked for. At a parallel-runway field the entrances of the
    /// other strip sit a runway separation away, so the right one wins on
    /// distance anyway; a field that breaks that assumption would need the
    /// destination runway's own geometry passed in.
    fn runway_entrance_near(
        &self,
        lat: f64,
        lon: f64,
        taxiway_nodes: &std::collections::HashSet<i64>,
    ) -> Option<i64> {
        let m_per_deg_lat = 111_320.0;
        let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
        let mut runway_nodes: std::collections::HashSet<i64> = std::collections::HashSet::new();
        for e in self.edges.iter().filter(|e| e.runway) {
            runway_nodes.insert(e.a);
            runway_nodes.insert(e.b);
        }
        if runway_nodes.is_empty() {
            return None;
        }
        let mut adjacent: std::collections::HashSet<i64> = std::collections::HashSet::new();
        for e in &self.edges {
            if runway_nodes.contains(&e.a) {
                adjacent.insert(e.b);
            }
            if runway_nodes.contains(&e.b) {
                adjacent.insert(e.a);
            }
        }
        taxiway_nodes
            .iter()
            .filter(|id| runway_nodes.contains(id) || adjacent.contains(id))
            .filter_map(|id| {
                self.nodes.get(id).map(|&p| {
                    let dx = (p.1 - lon) * m_per_deg_lon;
                    let dy = (p.0 - lat) * m_per_deg_lat;
                    (*id, dx.hypot(dy))
                })
            })
            .min_by(|x, y| x.1.total_cmp(&y.1))
            .filter(|(_, d)| *d < 3000.0)
            .map(|(id, _)| id)
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
        for (ident, hdg) in [
            (&r.ident_a, r.heading_a_deg()),
            (&r.ident_b, r.heading_b_deg()),
        ] {
            let diff = angle_diff_deg(wind_from_deg, hdg);
            let headwind = wind_kts * diff.to_radians().cos();
            if best.as_ref().is_none_or(|(_, _, hw)| headwind > *hw) {
                best = Some((ident.clone(), hdg, headwind));
            }
        }
    }
    best
}

/// The headwind component on one named runway END, for comparing a candidate
/// against the runway already in use. `None` if the field has no such end.
pub fn headwind_for(
    runways: &[Runway],
    ident: &str,
    wind_from_deg: f64,
    wind_kts: f64,
) -> Option<f64> {
    for r in runways {
        for (id, hdg) in [
            (&r.ident_a, r.heading_a_deg()),
            (&r.ident_b, r.heading_b_deg()),
        ] {
            if id == ident {
                return Some(wind_kts * angle_diff_deg(wind_from_deg, hdg).to_radians().cos());
            }
        }
    }
    None
}

/// Smallest signed angle a−b, degrees, in −180..180.
fn angle_diff_deg(a: f64, b: f64) -> f64 {
    (a - b + 540.0).rem_euclid(360.0) - 180.0
}

/// Initial great-circle bearing from point 1 to point 2, degrees true 0..360.
pub fn bearing_deg(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let (p1, p2) = (lat1.to_radians(), lat2.to_radians());
    let dl = (lon2 - lon1).to_radians();
    let y = dl.sin() * p2.cos();
    let x = p1.cos() * p2.sin() - p1.sin() * p2.cos() * dl.cos();
    y.atan2(x).to_degrees().rem_euclid(360.0)
}

/// Great-circle distance in nautical miles between two lat/lon points (haversine).
pub fn haversine_nm(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    const R_NM: f64 = 3440.065;
    let (p1, p2) = (lat1.to_radians(), lat2.to_radians());
    let dphi = (lat2 - lat1).to_radians();
    let dlam = (lon2 - lon1).to_radians();
    let a = (dphi / 2.0).sin().powi(2) + p1.cos() * p2.cos() * (dlam / 2.0).sin().powi(2);
    2.0 * R_NM * a.sqrt().asin()
}

/// A point `dist_nm` from (lat, lon) along true `bearing` (flat-earth good enough
/// at approach ranges).
pub fn project_latlon(lat: f64, lon: f64, bearing: f64, dist_nm: f64) -> (f64, f64) {
    let d = dist_nm / 60.0; // nm → degrees of latitude
    let b = bearing.to_radians();
    let dlat = d * b.cos();
    let dlon = d * b.sin() / lat.to_radians().cos().max(1e-6);
    (lat + dlat, lon + dlon)
}

// ===========================================================================
// Terminal procedures (SID / STAR / approach) from X-Plane's CIFP.
//
// Each `CIFP/<ICAO>.dat` holds ARINC-424 procedure records, one leg per line:
//   SID:seq,rtype,IDENT,TRANS,FIX,rgn,sec,sub,desc,...,PT,turn,recnav,...,
//        theta,rho,COURSE,DIST, altdesc,alt1,alt2, ...
// Comma-split field indices, verified against real EGLL/LEMG records (and the
// same 22/23/24 altitude columns `ils_platform_alt_ft` already relies on):
//   [0]  record type + sequence ("APPCH:020")
//   [2]  procedure ident       (approach "I27R"/"R27R", SID/STAR name "BLN4L")
//   [3]  transition ident      ("RW30" runway transition, a fix for an enroute
//                               transition, or BLANK for the common/final part)
//   [4]  fix ident             ("MG930", "RW27R", "" for the to-altitude legs)
//   [5]  fix ICAO region       ("LE", "EG") — disambiguates duplicate idents
//   [8]  description code      (4th char 'F' = final approach fix)
//   [11] path & terminator     (IF/TF/CF/DF/RF fix legs; CA/VA/FA to-altitude;
//                               FM/VM manual; HM/HA/HF holds)
//   [12] turn direction        (L/R)
//   [20] magnetic course ×10   (2680 → 268.0°) — set on course/heading legs
//   [21] distance ×10, nm      (0025 → 2.5)
//   [22] altitude descriptor   (+ at-or-above, - at-or-below, blank/@ at, B/G/I ..)
//   [23] altitude 1, ft (or FLxxx)
//   [24] altitude 2, ft
// Speed limits live further right at an index that shifts between record shapes,
// so they are deliberately NOT parsed — the follower flies a fixed profile speed.
// ===========================================================================

/// Which family of terminal procedure — the CIFP record prefix.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProcKind {
    /// Standard Instrument Departure.
    Sid,
    /// Standard Terminal Arrival Route.
    Star,
    /// Instrument approach.
    Appch,
}

impl ProcKind {
    fn prefix(self) -> &'static str {
        match self {
            ProcKind::Sid => "SID:",
            ProcKind::Star => "STAR:",
            ProcKind::Appch => "APPCH:",
        }
    }
}

/// A navigation fix resolved to a position.
#[derive(Debug, Clone, PartialEq)]
pub struct Fix {
    pub ident: String,
    pub lat: f64,
    pub lon: f64,
}

/// One leg of a terminal procedure, parsed from a CIFP record. `fix` is `None`
/// until [`resolve_proc_fixes`] fills it (and stays `None` for to-altitude legs
/// that have no fix at all).
#[derive(Debug, Clone)]
pub struct ProcLeg {
    pub seq: u32,
    /// Transition/runway this leg belongs to (field 3): "RW30", a transition
    /// fix name, or empty for the common/final portion.
    pub transition: String,
    /// Path & terminator (field 11): IF, TF, CF, DF, CA, VA, FM, RF, HM, ...
    pub leg_type: String,
    /// Fix ident (field 4): a waypoint/navaid, "RW27R" for the threshold, or
    /// empty on a to-altitude leg.
    pub fix_ident: String,
    /// Fix ICAO region (field 5) — narrows duplicate idents at resolve time.
    pub fix_region: String,
    pub fix: Option<Fix>,
    /// Magnetic course for course/heading legs (field 20), degrees; `None` on
    /// pure track/direct legs.
    pub course_mag: Option<f64>,
    /// Leg distance in nm (field 21), where the record carries one.
    pub dist_nm: Option<f64>,
    pub turn: Option<char>,
    /// Altitude descriptor (field 22): '+' at-or-above, '-' at-or-below, else at.
    pub alt_desc: char,
    pub alt1_ft: Option<f64>,
    pub alt2_ft: Option<f64>,
    /// True when this leg's fix is the final approach fix.
    pub is_faf: bool,
}

impl ProcLeg {
    /// Does this leg terminate at a fixed point (vs. climb/descend to an
    /// altitude on a heading)? Fix legs are the ones the follower can sequence
    /// laterally; to-altitude legs (CA/VA/FA) it flies as heading-holds.
    pub fn has_fix_target(&self) -> bool {
        self.fix.is_some()
    }
}

fn cifp_field<'a>(f: &[&'a str], i: usize) -> &'a str {
    f.get(i).map(|s| s.trim()).unwrap_or("")
}

/// ARINC altitude cell → feet MSL. Handles "FLxxx" and zero-padded numbers;
/// blank / zero → `None`.
fn arinc_alt_ft(s: &str) -> Option<f64> {
    let s = s.trim();
    if let Some(fl) = s.strip_prefix("FL") {
        return fl.parse::<f64>().ok().map(|f| f * 100.0);
    }
    s.parse::<f64>().ok().filter(|a| *a > 0.0)
}

/// A CIFP "×10" cell (course, distance) → its real value; blank → `None`.
fn arinc_tenths(s: &str) -> Option<f64> {
    let s = s.trim();
    (!s.is_empty())
        .then(|| s.parse::<f64>().ok())
        .flatten()
        .map(|v| v / 10.0)
}

/// Parse the legs of one procedure from CIFP lines. `transition` selects which
/// legs to keep: `None` keeps only the common/final portion (blank field 3) —
/// the right choice for an approach's final segment; `Some(t)` also keeps legs
/// whose transition is `t` — e.g. `Some("RW30")` for a SID off runway 30.
/// Legs come back transition-first, then common, each ordered by sequence.
pub fn parse_proc_legs<'a>(
    lines: impl Iterator<Item = &'a str>,
    kind: ProcKind,
    ident: &str,
    transition: Option<&str>,
) -> Vec<ProcLeg> {
    let prefix = kind.prefix();
    let mut out: Vec<ProcLeg> = Vec::new();
    for line in lines {
        if !line.starts_with(prefix) {
            continue;
        }
        let f: Vec<&str> = line.split(',').collect();
        if f.len() < 12 || cifp_field(&f, 2) != ident {
            continue;
        }
        let tr = cifp_field(&f, 3).to_string();
        match transition {
            None if !tr.is_empty() => continue,
            Some(want) if !tr.is_empty() && tr != want => continue,
            _ => {}
        }
        let seq = f[0]
            .split(':')
            .nth(1)
            .and_then(|s| s.trim().parse().ok())
            .unwrap_or(0);
        out.push(ProcLeg {
            seq,
            transition: tr,
            leg_type: cifp_field(&f, 11).to_string(),
            fix_ident: cifp_field(&f, 4).to_string(),
            fix_region: cifp_field(&f, 5).to_string(),
            fix: None,
            course_mag: arinc_tenths(cifp_field(&f, 20)),
            dist_nm: arinc_tenths(cifp_field(&f, 21)),
            turn: cifp_field(&f, 12)
                .chars()
                .next()
                .filter(|c| *c == 'L' || *c == 'R'),
            alt_desc: cifp_field(&f, 22).chars().next().unwrap_or(' '),
            alt1_ft: arinc_alt_ft(cifp_field(&f, 23)),
            alt2_ft: arinc_alt_ft(cifp_field(&f, 24)),
            is_faf: cifp_field(&f, 8).chars().nth(3) == Some('F'),
        });
    }
    // Transition (runway) legs first, then the common portion, each by sequence.
    out.sort_by_key(|l| (l.transition.is_empty(), l.seq));
    out
}

/// Read one procedure's legs from `CIFP/<ICAO>.dat`. See [`parse_proc_legs`]
/// for `transition` semantics. Empty vec = no such file/procedure.
pub fn load_proc_legs(
    cifp_dir: &str,
    icao: &str,
    kind: ProcKind,
    ident: &str,
    transition: Option<&str>,
) -> Vec<ProcLeg> {
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let Ok(text) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    parse_proc_legs(text.lines(), kind, ident, transition)
}

/// Every distinct procedure ident of `kind` at `icao`, in first-seen order.
pub fn list_procedures(cifp_dir: &str, icao: &str, kind: ProcKind) -> Vec<String> {
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let Ok(text) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    let prefix = kind.prefix();
    let mut out: Vec<String> = Vec::new();
    for line in text.lines() {
        if !line.starts_with(prefix) {
            continue;
        }
        let f: Vec<&str> = line.split(',').collect();
        let id = cifp_field(&f, 2);
        if !id.is_empty() && !out.iter().any(|s| s == id) {
            out.push(id.to_string());
        }
    }
    out
}

/// The runway an approach ident serves: strip the leading type letter(s) and any
/// "-Y"/"-Z" variant suffix. "I27R" → "27R", "R27R-Z" → "27R", "I20-Y" → "20".
/// `None` if there are no runway digits.
pub fn approach_runway(ident: &str) -> Option<String> {
    let body: String = ident
        .chars()
        .skip_while(|c| c.is_ascii_alphabetic())
        .collect();
    let body = body.split('-').next().unwrap_or("");
    let digits: String = body.chars().take_while(|c| c.is_ascii_digit()).collect();
    if digits.is_empty() {
        return None;
    }
    let suffix: String = body
        .chars()
        .skip_while(|c| c.is_ascii_digit())
        .take(1)
        .filter(|c| matches!(c, 'L' | 'R' | 'C'))
        .collect();
    Some(format!("{digits}{suffix}"))
}

/// Whether an approach/runway designator matches `runway` (e.g. "27R"): same
/// leading number, and same L/R/C side when both name one.
fn runway_matches(a: &str, b: &str) -> bool {
    if runway_number(a) != runway_number(b) {
        return false;
    }
    let side = |s: &str| {
        s.chars()
            .find(|c| matches!(c, 'L' | 'R' | 'C' | 'l' | 'r' | 'c'))
    };
    match (side(a), side(b)) {
        (Some(x), Some(y)) => x.eq_ignore_ascii_case(&y),
        _ => true, // one side unspecified → the number match is enough
    }
}

/// The best instrument approach ident for `runway` at `icao`: prefer ILS ('I'),
/// then RNAV/GPS ('R'/'X'/'Q'), then LOC ('L'), then anything else. `None` if the
/// field publishes no approach to that runway.
pub fn best_approach(cifp_dir: &str, icao: &str, runway: &str) -> Option<String> {
    let rank = |id: &str| match id.chars().next() {
        Some('I') => 0,
        Some('X') | Some('R') | Some('Q') => 1,
        Some('L') => 2,
        Some('D') | Some('S') | Some('V') => 3,
        _ => 4,
    };
    list_procedures(cifp_dir, icao, ProcKind::Appch)
        .into_iter()
        .filter(|id| approach_runway(id).is_some_and(|r| runway_matches(&r, runway)))
        .min_by_key(|id| rank(id))
}

/// A SID that departs `runway` at `icao` — the first ident with a leg whose
/// transition is `RW<runway>`. `None` if the field publishes no SID for it.
pub fn sid_for_runway(cifp_dir: &str, icao: &str, runway: &str) -> Option<String> {
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let text = std::fs::read_to_string(path).ok()?;
    let want = format!("RW{runway}");
    for line in text.lines() {
        if !line.starts_with("SID:") {
            continue;
        }
        let f: Vec<&str> = line.split(',').collect();
        let tr = cifp_field(&f, 3);
        if tr == want
            || (runway_matches(tr.strip_prefix("RW").unwrap_or(tr), runway) && tr.starts_with("RW"))
        {
            let id = cifp_field(&f, 2);
            if !id.is_empty() {
                return Some(id.to_string());
            }
        }
    }
    None
}

/// Resolve every fix in `legs` to a position, in place. Runway ("RW..") and the
/// airport ident come from `airport`; all other idents are looked up in
/// `earth_fix.dat` (waypoints) then `earth_nav.dat` (VOR/NDB/DME), matching
/// ident + ICAO region, choosing the candidate nearest `(near_lat, near_lon)`.
/// One scan of each data file regardless of leg count — call it off the realtime
/// loop. Legs whose fix can't be resolved keep `fix: None`.
pub fn resolve_proc_fixes(
    fix_path: &str,
    nav_path: &str,
    airport: &AirportInfo,
    icao: &str,
    legs: &mut [ProcLeg],
    near_lat: f64,
    near_lon: f64,
) {
    use std::collections::HashSet;

    // 1. Runway thresholds + the airport reference point resolve locally.
    for leg in legs.iter_mut() {
        if leg.fix_ident.is_empty() {
            continue;
        }
        if let Some(rw) = leg.fix_ident.strip_prefix("RW") {
            if let Some((lat, lon)) = runway_threshold(airport, rw) {
                leg.fix = Some(Fix {
                    ident: leg.fix_ident.clone(),
                    lat,
                    lon,
                });
            }
        } else if leg.fix_ident == icao {
            if let Some((lat, lon)) = airport.position() {
                leg.fix = Some(Fix {
                    ident: leg.fix_ident.clone(),
                    lat,
                    lon,
                });
            }
        }
    }

    // 2. Everything still unresolved: one scan each of the two nav files.
    let wanted: HashSet<(String, String)> = legs
        .iter()
        .filter(|l| l.fix.is_none() && !l.fix_ident.is_empty())
        .map(|l| (l.fix_ident.clone(), l.fix_region.clone()))
        .collect();
    if wanted.is_empty() {
        return;
    }
    let mut found: std::collections::HashMap<(String, String), (f64, f64, f64)> =
        Default::default();
    let mut consider = |ident: &str, region: &str, lat: f64, lon: f64| {
        let key = (ident.to_string(), region.to_string());
        if !wanted.contains(&key) {
            return;
        }
        let d = haversine_nm(near_lat, near_lon, lat, lon);
        found
            .entry(key)
            .and_modify(|best| {
                if d < best.2 {
                    *best = (lat, lon, d);
                }
            })
            .or_insert((lat, lon, d));
    };

    // earth_fix.dat: `lat lon IDENT area REGION ...`
    if let Ok(file) = std::fs::File::open(fix_path) {
        for line in BufReader::new(file).lines().map_while(Result::ok) {
            let f: Vec<&str> = line.split_whitespace().collect();
            if f.len() < 5 {
                continue;
            }
            if let (Ok(lat), Ok(lon)) = (f[0].parse::<f64>(), f[1].parse::<f64>()) {
                consider(f[2], f[4], lat, lon);
            }
        }
    }
    // earth_nav.dat: `code lat lon elev freq range var IDENT area REGION name`
    if let Ok(file) = std::fs::File::open(nav_path) {
        for line in BufReader::new(file).lines().map_while(Result::ok) {
            let f: Vec<&str> = line.split_whitespace().collect();
            if f.len() < 10 {
                continue;
            }
            // Row codes 2 (NDB), 3 (VOR), 12/13 (DME) carry a named ident.
            if !matches!(f[0], "2" | "3" | "12" | "13") {
                continue;
            }
            if let (Ok(lat), Ok(lon)) = (f[1].parse::<f64>(), f[2].parse::<f64>()) {
                consider(f[7], f[9], lat, lon);
            }
        }
    }

    for leg in legs.iter_mut() {
        if leg.fix.is_some() || leg.fix_ident.is_empty() {
            continue;
        }
        if let Some(&(lat, lon, _)) = found.get(&(leg.fix_ident.clone(), leg.fix_region.clone())) {
            leg.fix = Some(Fix {
                ident: leg.fix_ident.clone(),
                lat,
                lon,
            });
        }
    }
}

/// Reconstruct a straight-in approach for `runway` from the airport's published
/// DME fixes when the CIFP carries no APPCH records — the way a pilot flies the
/// DME/TACAN approach off the chart. X-Plane ships terminal fixes named
/// `<runway>D<dme>` (e.g. `27D10`, `27D06`, `27D05` at Gibraltar: the 10/6/5 DME
/// points on the runway-27 final, stepping in over the water) even where the
/// procedure records are missing. Ordered farthest-DME first, a 3° descent
/// profile synthesised from each fix's distance to the threshold, the innermost
/// flagged as the FAF, then the threshold itself. Fixes are read straight from
/// `earth_fix.dat` (already positioned). Empty = no such fixes to chain.
///
/// Terrain safety comes from the fixes: they trace the real published final,
/// which the authority routed clear of high ground. Getting TO the first fix is
/// the follower's job — do it at approach altitude (the outer fix is ~3,000 ft),
/// which clears terrain a low pattern would hit.
pub fn reconstruct_approach(
    fix_path: &str,
    airport: &AirportInfo,
    icao: &str,
    runway: &str,
) -> Vec<ProcLeg> {
    /// 3° glidepath, feet per nm (tan 3° × 6076).
    const GLIDE_FT_PER_NM: f64 = 318.0;
    let Some((thr_lat, thr_lon)) = runway_threshold(airport, runway) else {
        return Vec::new();
    };
    // Collect the `<runway>D<dme>` fixes for this field: (dme, ident, lat, lon).
    let mut dfix: Vec<(u32, String, f64, f64)> = Vec::new();
    if let Ok(file) = std::fs::File::open(fix_path) {
        for line in BufReader::new(file).lines().map_while(Result::ok) {
            let f: Vec<&str> = line.split_whitespace().collect();
            if f.len() < 5 || f[3] != icao {
                continue;
            }
            // ident = "<runway>D<digits>", e.g. "27D05".
            let Some(rest) = f[2].strip_prefix(runway).and_then(|s| s.strip_prefix('D')) else {
                continue;
            };
            let Ok(dme) = rest.parse::<u32>() else {
                continue;
            };
            if let (Ok(lat), Ok(lon)) = (f[0].parse::<f64>(), f[1].parse::<f64>()) {
                dfix.push((dme, f[2].to_string(), lat, lon));
            }
        }
    }
    if dfix.is_empty() {
        return Vec::new();
    }
    // Only fixes ON the final approach course reconstruct a straight-in. The
    // ident says "runway 09, 6 DME", not "on the 09 centreline": Gibraltar's
    // 09D05/09D06 lie on the VOR/DME approach's offset track, 50 degrees
    // south of the runway's extended centreline. Flown as a straight-in they
    // handed autoland the aircraft 3.6 km abeam with 5.5 km to run, which its
    // 35 degree cut cannot close: a saturated final, a go-around at 95 ft
    // still 300 m off the centreline, and a crash into the North Front
    // (12:46Z, 2026-09-11). A fix off the course is not a straight-in fix;
    // with none left there is no straight-in, and the caller lands visually.
    let Some(course) = runway_course_deg(airport, runway) else {
        return Vec::new();
    };
    let inbound_from = (course + 180.0).rem_euclid(360.0);
    dfix.retain(|(_, ident, lat, lon)| {
        let brg = bearing_deg(thr_lat, thr_lon, *lat, *lon);
        let off = ((brg - inbound_from + 540.0).rem_euclid(360.0) - 180.0).abs();
        let aligned = off <= RECON_ALIGN_DEG;
        if !aligned {
            tracing::info!(
                ident,
                off_deg = off,
                "DME fix is off the final approach course, not a straight-in fix"
            );
        }
        aligned
    });
    if dfix.is_empty() {
        return Vec::new();
    }
    // Farthest DME first (fly inbound); the innermost is the FAF.
    dfix.sort_by_key(|item| std::cmp::Reverse(item.0));
    let elev = airport.elev_ft;
    let mut legs: Vec<ProcLeg> = Vec::new();
    let last = dfix.len() - 1;
    for (i, (_dme, ident, lat, lon)) in dfix.iter().enumerate() {
        let dist_nm = haversine_nm(*lat, *lon, thr_lat, thr_lon);
        legs.push(ProcLeg {
            seq: (i as u32 + 1) * 10,
            transition: String::new(),
            leg_type: "TF".to_string(),
            fix_ident: ident.clone(),
            fix_region: String::new(),
            fix: Some(Fix {
                ident: ident.clone(),
                lat: *lat,
                lon: *lon,
            }),
            course_mag: None,
            dist_nm: Some(dist_nm),
            turn: None,
            alt_desc: ' ',
            alt1_ft: Some((elev + dist_nm * GLIDE_FT_PER_NM).round()),
            alt2_ft: None,
            is_faf: i == last, // the innermost DME fix
        });
    }
    // The threshold itself, so the follower has a fix to fly to after the FAF
    // (it hands to autoland at the FAF regardless).
    legs.push(ProcLeg {
        seq: (dfix.len() as u32 + 1) * 10,
        transition: String::new(),
        leg_type: "TF".to_string(),
        fix_ident: format!("RW{runway}"),
        fix_region: String::new(),
        fix: Some(Fix {
            ident: format!("RW{runway}"),
            lat: thr_lat,
            lon: thr_lon,
        }),
        course_mag: None,
        dist_nm: Some(0.0),
        turn: None,
        alt_desc: ' ',
        alt1_ft: Some(elev),
        alt2_ft: None,
        is_faf: false,
    });
    legs
}

/// The threshold position of runway `id` at `airport`, matching either end.
pub fn runway_threshold(airport: &AirportInfo, id: &str) -> Option<(f64, f64)> {
    airport.runways.iter().find_map(|r| {
        if runway_matches(&r.ident_a, id) {
            Some((r.lat_a, r.lon_a))
        } else if runway_matches(&r.ident_b, id) {
            Some((r.lat_b, r.lon_b))
        } else {
            None
        }
    })
}

/// The landing course of runway `id` at `airport`, degrees true.
pub fn runway_course_deg(airport: &AirportInfo, id: &str) -> Option<f64> {
    airport.runways.iter().find_map(|r| {
        if runway_matches(&r.ident_a, id) {
            Some(r.heading_a_deg())
        } else if runway_matches(&r.ident_b, id) {
            Some(r.heading_b_deg())
        } else {
            None
        }
    })
}

/// A point in the runway frame: `(cross-track m, along-track m)` relative to
/// the threshold, where along-track is positive on the APPROACH side (short of
/// the threshold, still to fly) and cross-track is signed right of the inbound
/// course. Where an approach fix sits relative to the final approach course.
pub fn runway_frame_m(
    thr_lat: f64,
    thr_lon: f64,
    course_deg: f64,
    lat: f64,
    lon: f64,
) -> (f64, f64) {
    let d_m = haversine_nm(thr_lat, thr_lon, lat, lon) * 1852.0;
    let brg = bearing_deg(thr_lat, thr_lon, lat, lon);
    let rel = (brg - (course_deg + 180.0)).to_radians();
    (d_m * rel.sin(), d_m * rel.cos())
}

/// How far a DME fix's bearing from the threshold may stray from the runway's
/// reciprocal course and still count as ON the final approach course.
const RECON_ALIGN_DEG: f64 = 10.0;

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
        writeln!(
            f,
            " 4  36.710000000   -4.550000000  100 10950 18 101442.600 ITST LXXX LX 28 ILS-cat-I"
        )
        .unwrap();
        // far (KSEA-ish)
        writeln!(
            f,
            " 4  47.466600000 -122.317800000  359 11075 18 123840.337 IFAR KSEA K1 28 ILS-cat-I"
        )
        .unwrap();
        f.flush().unwrap();

        let ils = find_ils(path.to_str().unwrap(), "28", 36.71, -4.55, 60.0)
            .unwrap()
            .expect("should find the near runway-28 ILS");
        assert_eq!(ils.airport, "LXXX");
        assert!(
            (ils.freq_mhz() - 109.50).abs() < 0.001,
            "freq {}",
            ils.freq_mhz()
        );
        assert!(
            (ils.course_mag - 281.785).abs() < 0.5,
            "course {}",
            ils.course_mag
        );
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
        let apt = find_airport(path.to_str().unwrap(), "LEAL")
            .unwrap()
            .expect("found");
        assert_eq!(apt.elev_ft, 142.0);
        assert_eq!(apt.name, "Alicante");
        // Frequencies: only LEAL's, decoded to kHz whatever the row era.
        assert_eq!(apt.freqs.len(), 2, "LEAL's two comm rows, not Sunderland's");
        let twr = apt.freq(FREQ_TOWER).expect("tower row");
        assert_eq!((twr.khz, twr.name.as_str()), (118250, "Tower Freq"));
        assert_eq!(
            apt.freq(FREQ_ATIS).expect("atis row").khz,
            122100,
            "legacy 10 kHz units"
        );
        assert!(apt.freq(FREQ_GROUND).is_none());
        let rw = find_runways(path.to_str().unwrap(), "LEAL").unwrap();
        assert_eq!(rw.len(), 1, "one physical runway, not Sunderland's");
        assert_eq!(
            (rw[0].ident_a.as_str(), rw[0].ident_b.as_str()),
            ("10", "28")
        );
        // Geometry sanity: the 10 end points roughly east-southeast.
        assert!(
            (rw[0].heading_a_deg() - 100.0).abs() < 8.0,
            "{}",
            rw[0].heading_a_deg()
        );

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
        // A tail the way real Gateway networks build it: taxiway C on to the
        // node where it meets the runway CENTRELINE (7), which also anchors
        // a runway edge — the hold-short trim case.
        writeln!(f, "1201 36.0004000 -5.0140000 both 7 hp").unwrap();
        writeln!(f, "1201 36.0010000 -5.0140000 both 8 rwy").unwrap();
        writeln!(f, "1202 1 2 twoway taxiway A").unwrap();
        writeln!(f, "1202 2 3 twoway runway 14/32").unwrap();
        writeln!(f, "1202 3 4 twoway taxiway B").unwrap();
        // An `ils` zone row on B: the LVP hold, which the parser must SKIP —
        // were it collected, the zone tail below would swallow B too.
        writeln!(f, "1204 ils 09,27").unwrap();
        writeln!(f, "1202 4 7 twoway taxiway C").unwrap();
        // C lies inside 09/27's active zone: node 4 is the authored holding
        // point, the way EGLL marks every runway entry.
        writeln!(f, "1204 departure 09,27").unwrap();
        writeln!(f, "1204 arrival 09,27").unwrap();
        writeln!(f, "1202 7 8 twoway runway 09/27").unwrap();
        // Parking: a gate right by the stand node, and a hangar further out
        // that an arrival must NOT be sent to.
        writeln!(
            f,
            "1300 36.0031000 -5.0050000 180.0 gate turboprops|props Stand 1"
        )
        .unwrap();
        writeln!(f, "1300 36.0060000 -5.0050000 90.0 hangar jets Maintenance").unwrap();
        writeln!(f, "1 1027 0 0 0MU1 Sunderland").unwrap();
        writeln!(f, "1202 5 6 twoway taxiway Z").unwrap();
        f.flush().unwrap();

        let net = find_taxi_net(path.to_str().unwrap(), "LTST")
            .unwrap()
            .expect("network found");
        assert_eq!(net.nodes.len(), 6);
        assert_eq!(net.edges.len(), 5, "only LTST's edges, not Sunderland's");

        // Parking: both stands parse; the nearest SUITABLE one (from a point
        // near the gate) is the gate, never the hangar.
        assert_eq!(net.stands.len(), 2, "gate + hangar");
        let near = net.nearest_stand(36.0031, -5.0050).expect("a stand");
        assert_eq!(near.name, "Stand 1");
        assert!(near.is_suitable());
        assert!(net
            .stands
            .iter()
            .any(|s| s.kind == "hangar" && !s.is_suitable()));
        assert_eq!(
            near.accepts,
            vec!["turboprops".to_string(), "props".to_string()]
        );

        // From the stand to the 09 threshold: via A and B, crossing 14/32,
        // holding point named for the last taxiway before the runway.
        let plan = net
            .plan(
                36.0031,
                -5.0050,
                36.0000,
                -5.0100,
                HOLD_SHORT_STANDOFF_M,
                &[],
            )
            .expect("a route exists");
        assert_eq!(plan.via, vec!["A".to_string(), "B".to_string()]);
        assert_eq!(plan.crossings, vec!["14/32".to_string()]);
        assert_eq!(plan.holding_point, "B");
        assert!(plan.path.len() >= 4, "polyline for the compliance watch");
        // The polyline starts AT the aircraft — the apron leg from the stand
        // to the snapped node is part of the cleared route.
        assert_eq!(plan.path[0], (36.0031, -5.0050));
        assert_eq!(plan.seg.len(), plan.path.len() - 1, "seg stays aligned");
        assert_eq!(plan.seg[0], (String::new(), false));

        // To a threshold whose route ends at the runway-junction node (7):
        // the tail is pulled back by the hold-short standoff, so the route
        // ends where an aircraft STOPS, never on the runway (an untrimmed
        // route parked the auto taxi's nose across the hold line,
        // 2026-07-20). Node 4's route above stays untrimmed — it never
        // touches a runway edge.
        let plan2 = net
            .plan(
                36.0031,
                -5.0050,
                36.0004,
                -5.0141,
                HOLD_SHORT_STANDOFF_M,
                &[],
            )
            .expect("a route exists");
        assert_eq!(plan2.holding_point, "C");
        let &(llat, llon) = plan2.path.last().unwrap();
        let dx = (llon - (-5.0140)) * 111_320.0 * 36.0_f64.to_radians().cos();
        let dy = (llat - 36.0004) * 111_320.0;
        let d = (dx * dx + dy * dy).sqrt();
        assert!(
            (35.0..=45.0).contains(&d),
            "stops ~40 m short of the junction: {d}"
        );
        assert_eq!(
            plan2.seg.len(),
            plan2.path.len() - 1,
            "seg stays aligned after the trim"
        );
        // The trimmed tail SURVIVES as the lead-on: from the hold-short point
        // to the runway-junction node — the painted line the line-up drives.
        assert!(!plan2.lead_on.is_empty(), "geometric trim keeps its tail");
        assert_eq!(
            *plan2.lead_on.first().unwrap(),
            *plan2.path.last().unwrap(),
            "starts at the hold"
        );
        assert_eq!(
            *plan2.lead_on.last().unwrap(),
            (36.0004, -5.0140),
            "ends at the junction"
        );
        // An untrimmed route (node 4, never touches a runway edge) has none.
        assert!(plan.lead_on.is_empty(), "no trim, no lead-on");
        // An instrument-scale standoff trims further back along the same
        // route (Heathrow's CAT holds sit ~100 m out; 40 m parked a taxi on
        // the entry stub past the line, 2026-07-20).
        let cat = net
            .plan(36.0031, -5.0050, 36.0004, -5.0141, 100.0, &[])
            .expect("a route exists");
        let &(clat, clon) = cat.path.last().unwrap();
        let dx = (clon - (-5.0140)) * 111_320.0 * 36.0_f64.to_radians().cos();
        let dy = (clat - 36.0004) * 111_320.0;
        let d = (dx * dx + dy * dy).sqrt();
        assert!(
            (95.0..=105.0).contains(&d),
            "instrument standoff honoured: {d}"
        );
        // AUTHORED active zones beat both guesses: with the destination
        // runway named, the route ends at the zone boundary node — the
        // per-stub holding point as the airport author placed it. The `ils`
        // row on B must NOT extend the zone (LVP holds are future work).
        let zone: Vec<String> = vec!["09".into(), "27".into()];
        let zoned = net
            .plan(
                36.0031,
                -5.0050,
                36.0004,
                -5.0141,
                HOLD_SHORT_STANDOFF_M,
                &zone,
            )
            .expect("a route exists");
        assert_eq!(
            *zoned.path.last().unwrap(),
            (36.0004, -5.0095),
            "ends at node 4, the zone edge"
        );
        assert_eq!(
            zoned.seg.len(),
            zoned.path.len() - 1,
            "seg aligned after the zone trim"
        );

        // Nowhere near the network: no snap, no plan — never a wild route.
        assert!(net
            .plan(37.0, -6.0, 36.0, -5.01, HOLD_SHORT_STANDOFF_M, &[])
            .is_none());
        // A field without 1201/1202 rows has no network at all.
        assert!(find_taxi_net(path.to_str().unwrap(), "0MU1")
            .unwrap()
            .is_some());
        assert!(find_taxi_net(path.to_str().unwrap(), "LXXX")
            .unwrap()
            .is_none());
    }

    /// Southampton's shape, which made a clearance to nowhere (2026-07-30).
    ///
    /// EGHI's parallel taxiway runs 170 m abeam the strip and its north end —
    /// the A5 stub — is a DEAD END, connected to no runway edge. Being the
    /// taxiway node nearest the runway 20 threshold (631 m, against 1047 m for
    /// the nearest real entrance) it won the plain snap, so the tower cleared
    /// us to "holding point Alpha five runway two zero" and the drive stopped
    /// 172 m abeam a runway it could not reach. A runway destination has to
    /// snap to a node the aircraft can get ONTO the runway from.
    #[test]
    fn a_runway_route_ends_at_an_entrance_not_the_nearest_dead_end() {
        // Runway 09/27 along lat 36.0000; the 27 threshold is the east end.
        // A parallel taxiway 111 m south, its EAST end (22) a dead-end stub
        // that is much the nearest taxiway node to that threshold. The only
        // real entrance is the stub 20->10, out at the west end.
        //
        //         09 thr                                      27 thr
        //   ---------(10)=========(11)------------------------- runway
        //             |                                (10,11 centreline)
        //            (20)-------(21)-------(22)   <- dead end, nearest
        //             |
        //            (30) stand
        let path = std::env::temp_dir().join("xc-taxi-deadend.dat");
        let mut f = std::fs::File::create(&path).unwrap();
        writeln!(f, "1    50 0 0 LTDE Deadend").unwrap();
        writeln!(f, "100 45.33 27 727 0.10 1 3 0  09 36.0000000 -5.0100000 0 0 3 3 0 2 27 36.0000000 -5.0000000 0 0 3 3 0 2").unwrap();
        writeln!(f, "1201 36.0000000 -5.0080000 both 10 rwy_w").unwrap();
        writeln!(f, "1201 36.0000000 -5.0030000 both 11 rwy_e").unwrap();
        writeln!(f, "1201 35.9990000 -5.0080000 both 20 hp").unwrap();
        writeln!(f, "1201 35.9990000 -5.0040000 both 21 par").unwrap();
        writeln!(f, "1201 35.9990000 -5.0015000 both 22 stub").unwrap();
        writeln!(f, "1201 35.9985000 -5.0075000 both 30 stand").unwrap();
        writeln!(f, "1202 10 11 twoway runway 09/27").unwrap();
        writeln!(f, "1202 20 10 twoway taxiway A1").unwrap();
        writeln!(f, "1202 20 21 twoway taxiway A").unwrap();
        writeln!(f, "1202 21 22 twoway taxiway A5").unwrap();
        writeln!(f, "1202 30 20 twoway taxiway A1").unwrap();
        drop(f);
        let net = find_taxi_net(path.to_str().unwrap(), "LTDE")
            .unwrap()
            .expect("a network");

        let m = |a: (f64, f64), b: (f64, f64)| {
            let dx = (a.1 - b.1) * 111_320.0 * 36.0_f64.to_radians().cos();
            let dy = (a.0 - b.0) * 111_320.0;
            (dx * dx + dy * dy).sqrt()
        };
        let thr27 = (36.0000, -5.0000);
        // The trap: the dead-end stub really is nearest, by a wide margin.
        assert!(
            m((35.9990, -5.0015), thr27) < m((36.0000, -5.0080), thr27),
            "the stub is the nearest taxiway node — that is the whole trap"
        );

        // Routing to the runway ends at the entrance, hold-short trimmed off
        // the junction, with the drawn lead-on kept for the line-up.
        let plan = net
            .plan_to_runway(35.9985, -5.0075, thr27.0, thr27.1, 40.0, &[])
            .expect("a route exists");
        let end = *plan.path.last().unwrap();
        assert!(
            m(end, (36.0000, -5.0080)) < 50.0,
            "ends near the entrance stub, not the dead end: {end:?}"
        );
        assert!(
            m(end, (35.9990, -5.0015)) > 300.0,
            "and nowhere near the A5 dead end"
        );
        assert_eq!(plan.holding_point, "A1");
        assert!(
            !plan.lead_on.is_empty(),
            "the entrance touches a runway edge, so the trim keeps a lead-on \
             for the line-up to drive — the dead end had none to keep"
        );
        assert_eq!(plan.seg.len(), plan.path.len() - 1, "seg stays aligned");

        // A POINT destination keeps the plain nearest-node snap: the off-route
        // replan aims at the standing route's already-trimmed holding point,
        // and must not be dragged back onto the junction the trim removed.
        let point = net
            .plan(35.9985, -5.0075, 35.9990, -5.0016, 40.0, &[])
            .expect("a route exists");
        assert!(
            m(*point.path.last().unwrap(), (35.9990, -5.0015)) < 20.0,
            "a point goal still snaps to the node nearest it"
        );

        // A network that models no runway at all falls back rather than
        // refusing a route it would have flown before.
        let path2 = std::env::temp_dir().join("xc-taxi-norwy.dat");
        let mut f2 = std::fs::File::create(&path2).unwrap();
        writeln!(f2, "1    50 0 0 LTNR Norunway").unwrap();
        writeln!(f2, "1201 35.9990000 -5.0080000 both 20 hp").unwrap();
        writeln!(f2, "1201 35.9990000 -5.0015000 both 22 stub").unwrap();
        writeln!(f2, "1202 20 22 twoway taxiway A").unwrap();
        drop(f2);
        let bare = find_taxi_net(path2.to_str().unwrap(), "LTNR")
            .unwrap()
            .expect("a network");
        assert!(
            bare.plan_to_runway(35.9990, -5.0081, thr27.0, thr27.1, 40.0, &[])
                .is_some(),
            "no entrance in the data is not a reason to refuse to taxi"
        );
    }

    /// A field near 36°N, so the paint tests read as a taxi diagram in
    /// metres: `ll(east, north)` from the airport reference point.
    const LAT0: f64 = 36.0;
    const LON0: f64 = -5.0;
    fn ll(east_m: f64, north_m: f64) -> (f64, f64) {
        (
            LAT0 + north_m / 111_320.0,
            LON0 + east_m / (111_320.0 * LAT0.to_radians().cos()),
        )
    }

    /// The whole point, in one test: a network drawn beside its paint, and a
    /// route that comes back ON the paint.
    ///
    /// The geometry is Malaga's, reduced (2026-08-05). The drawn line runs
    /// due east; the 1201 nodes that name it were authored 2.5 m north of it,
    /// which is inside the tolerance of anyone eyeballing a WED window and
    /// twenty times what the drive's own tracking error is.
    #[test]
    fn a_route_rides_the_drawn_line_the_network_only_approximates() {
        let paint = vec![vec![ll(0.0, 0.0), ll(400.0, 0.0)]];
        let path = vec![ll(0.0, 2.5), ll(200.0, 2.5), ll(400.0, 2.5)];
        let seg = vec![("A".to_string(), false); 2];
        let (rode, rode_seg) = ride_paint(&paint, &path, &seg);
        assert_eq!(rode_seg.len(), rode.len() - 1, "seg stays aligned with path");
        assert!(rode_seg.iter().all(|s| s.0 == "A"), "names are untouched");
        let off = |p: (f64, f64)| (p.0 - LAT0) * 111_320.0;
        for p in &rode {
            assert!(
                off(*p).abs() < 0.05,
                "every point should sit on the drawn line, not 2.5 m north of it: {:.2} m",
                off(*p)
            );
        }
        // Along-track is preserved: this moves a route sideways, never
        // shortens or re-routes it.
        let east = |p: (f64, f64)| (p.1 - LON0) * 111_320.0 * LAT0.to_radians().cos();
        assert!(east(rode[0]).abs() < 0.5 && (east(*rode.last().unwrap()) - 400.0).abs() < 0.5);
        assert!(
            rode.windows(2).all(|w| east(w[1]) > east(w[0])),
            "progress along the route must stay monotonic"
        );

        // A line that merely passes nearby is not this leg's line. Same
        // geometry, but the paint crosses at right angles: nothing snaps,
        // and the route is handed back as it was rather than dragged onto
        // the taxiway it is crossing.
        let across = vec![vec![ll(200.0, -50.0), ll(200.0, 50.0)]];
        let (unmoved, _) = ride_paint(&across, &path, &seg);
        assert!(unmoved.iter().all(|p| (off(*p) - 2.5).abs() < 0.05));

        // Too far away to be the same line: a taxiway width out is a
        // different taxiway, and a route is never invented onto one.
        let far = vec![vec![ll(0.0, -20.0), ll(400.0, -20.0)]];
        let (kept, _) = ride_paint(&far, &path, &seg);
        assert!(kept.iter().all(|p| (off(*p) - 2.5).abs() < 0.05));

        // No paint at all (thin data, or a net built without it) is not a
        // refusal — the route comes back exactly as planned.
        let (bare, bare_seg) = ride_paint(&[], &path, &seg);
        assert_eq!(bare, path);
        assert_eq!(bare_seg, seg);
    }

    /// An arrival is driven to the STAND, not to the taxiway node nearest it.
    ///
    /// The network models the taxiway and stops; the apron lead-in usually
    /// isn't in it at all. At Malaga that left the aircraft parked 66 m short
    /// of the stand the tower had named, across a live taxiway, twice
    /// (2026-08-05).
    #[test]
    fn an_arrival_is_driven_onto_the_stand_the_tower_named() {
        let dir = std::env::temp_dir().join(format!("xvl_stand_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("apt.dat");
        let mut f = std::fs::File::create(&path).unwrap();
        writeln!(f, "I").unwrap();
        writeln!(f, "1000 Generated").unwrap();
        writeln!(f, "1 10 0 0 TEST Test Field").unwrap();
        // A taxiway east-west; the stand sits 60 m north of its east end,
        // with nothing in the network to reach it.
        let a = ll(0.0, 0.0);
        let b = ll(200.0, 0.0);
        let stand_p = ll(200.0, 60.0);
        writeln!(f, "1201 {:.7} {:.7} both 1 A", a.0, a.1).unwrap();
        writeln!(f, "1201 {:.7} {:.7} both 2 A", b.0, b.1).unwrap();
        writeln!(f, "1202 1 2 twoway taxiway A").unwrap();
        writeln!(
            f,
            "1300 {:.7} {:.7} 0.0 gate jets|props Stand 1",
            stand_p.0, stand_p.1
        )
        .unwrap();
        drop(f);
        let net = find_taxi_net(path.to_str().unwrap(), "TEST")
            .unwrap()
            .expect("a network");
        let stand = net.stands[0].clone();
        let plan = net.plan_to_stand(a.0, a.1, &stand).expect("a route");
        let end = *plan.path.last().unwrap();
        let off = |p: (f64, f64), q: (f64, f64)| {
            let dx = (p.1 - q.1) * 111_320.0 * LAT0.to_radians().cos();
            let dy = (p.0 - q.0) * 111_320.0;
            (dx * dx + dy * dy).sqrt()
        };
        assert!(
            off(end, stand_p) < 1.0,
            "the drive must finish at the stand, not {:.0} m short on the taxiway",
            off(end, stand_p)
        );
        assert_eq!(
            plan.seg.len(),
            plan.path.len() - 1,
            "the lead-in keeps seg aligned"
        );
        assert!(
            plan.seg.last().is_some_and(|s| s.0.is_empty() && !s.1),
            "an apron lead-in is not a named taxiway: {:?}",
            plan.seg.last()
        );

        // Too far to be this route's stand: the old stop-at-the-node
        // behaviour, because driving blind across an apron is worse than
        // stopping short of it.
        let far = Stand {
            lat: ll(200.0, 300.0).0,
            lon: ll(200.0, 300.0).1,
            ..stand.clone()
        };
        let plan = net.plan_to_stand(a.0, a.1, &far).expect("a route");
        assert!(
            off(*plan.path.last().unwrap(), (far.lat, far.lon)) > 100.0,
            "a stand out of lead-in range is not driven to"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A runway leg is left alone, and with it the point a vacate starts
    /// from. The crossing runs square across every taxi line near it, the
    /// strip's own markings are not linear features, and the aircraft's own
    /// position is the one point in any route that must never move.
    #[test]
    fn the_snap_never_moves_a_runway_leg_or_the_start_of_a_vacate() {
        let paint = vec![vec![ll(-30.0, 0.0), ll(400.0, 0.0)]];
        let path = vec![ll(0.0, 4.0), ll(60.0, 4.0), ll(200.0, 4.0)];
        let seg = vec![("".to_string(), true), ("B".to_string(), false)];
        let (rode, _) = ride_paint(&paint, &path, &seg);
        assert_eq!(rode[0], path[0], "the runway leg keeps its start exactly");
        let off = |p: (f64, f64)| (p.0 - LAT0) * 111_320.0;
        assert!(
            off(*rode.last().unwrap()).abs() < 0.05,
            "the taxiway leg after it still rides the line"
        );
    }

    /// The parse: line type on the node the segment LEAVES, beziers sampled,
    /// and anything that is not a centreline left out of the paint.
    #[test]
    fn drawn_centrelines_parse_out_of_a_linear_feature() {
        let dir = std::env::temp_dir().join(format!("xvl_paint_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("apt.dat");
        let mut f = std::fs::File::create(&path).unwrap();
        writeln!(f, "I").unwrap();
        writeln!(f, "1000 Generated").unwrap();
        writeln!(f, "1 10 0 0 TEST Test Field").unwrap();
        writeln!(f, "1201 36.0000000 -5.0000000 both 1 A").unwrap();
        writeln!(f, "1201 36.0000000 -5.0010000 both 2 A").unwrap();
        writeln!(f, "1202 1 2 twoway taxiway A").unwrap();
        // A centreline with a bezier in the middle, then a stretch of some
        // other marking, then the terminator.
        writeln!(f, "120 taxiway A centre").unwrap();
        writeln!(f, "111 36.0000000 -5.0000000 51").unwrap();
        writeln!(f, "112 36.0000000 -5.0005000 36.0000500 -5.0005000 51").unwrap();
        writeln!(f, "111 36.0000000 -5.0010000 4").unwrap();
        writeln!(f, "115 36.0000000 -5.0015000").unwrap();
        // A pavement boundary drawn with the same rows is NOT paint to drive.
        writeln!(f, "110 1 0.0 0.0 apron").unwrap();
        writeln!(f, "111 36.0100000 -5.0000000 51").unwrap();
        writeln!(f, "113 36.0100000 -5.0010000 51").unwrap();
        drop(f);
        let net = find_taxi_net(path.to_str().unwrap(), "TEST")
            .unwrap()
            .expect("a network");
        assert_eq!(net.paint.len(), 1, "one centreline run: {:?}", net.paint);
        let line = &net.paint[0];
        assert!(
            line.len() > BEZIER_STEPS,
            "the curve is sampled, not a straight chord: {}",
            line.len()
        );
        // The bezier bulges north of the chord, which is the whole reason to
        // sample it rather than join the nodes.
        let bulge = line
            .iter()
            .map(|p| (p.0 - 36.0) * 111_320.0)
            .fold(0.0_f64, f64::max);
        assert!(bulge > 0.5, "bezier ignored: {bulge:.2} m");
        // The type-4 leg (a hold bar) is not a line to taxi down, so the run
        // ends at the node where the centreline type stops — the segment
        // beyond it is drawn, but it is not a line to drive.
        assert!(
            line.iter().all(|p| p.1 >= -5.0010 - 1e-9),
            "the run must stop where the centreline type stops: {:?}",
            line.last()
        );
        assert!(
            (line.last().unwrap().1 + 5.0010).abs() < 1e-9,
            "and it must reach it: {:?}",
            line.last()
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The sortie this came from, measured against the real file.
    /// `cargo test -p atc -- --ignored live_lemg_route_rides --nocapture`
    ///
    /// Malaga, 2026-08-05: the Baron taxied 1,214 m from its GA stand to
    /// runway 13, tracking the planned route to 0.12 m and the drawn line to
    /// nothing better than 1.7 m. The assertion is on the route, not on the
    /// aircraft — where the route runs is the only part of that we own.
    #[test]
    #[ignore] // needs the real X-Plane install; run with --ignored
    fn live_lemg_route_rides_the_paint() {
        let path =
            "/Users/duanelicudi/X-Plane 12/Global Scenery/Global Airports/Earth nav data/apt.dat";
        let Ok(Some(net)) = find_taxi_net(path, "LEMG") else {
            return; // no install, no verdict
        };
        assert!(!net.paint.is_empty(), "LEMG draws its taxi centrelines");
        let (slat, slon) = (36.682214, -4.502304); // the stand it pushed off
        let (tlat, tlon) = (36.6620, -4.5100); // runway 13 threshold, near enough
        let zone: Vec<String> = vec!["13".into(), "31".into()];
        let plan = net
            .plan_to_runway(slat, slon, tlat, tlon, 100.0, &zone)
            .expect("a route");
        // Distance from each route point to the nearest drawn centreline,
        // whatever direction it runs — a plain "how far off the paint is it".
        let m_per_deg_lon = 111_320.0 * slat.to_radians().cos();
        let to_paint = |p: (f64, f64)| -> f64 {
            net.paint
                .iter()
                .flat_map(|l| l.windows(2))
                .map(|w| {
                    let (ax, ay) = ((w[0].1 - p.1) * m_per_deg_lon, (w[0].0 - p.0) * 111_320.0);
                    let (bx, by) = ((w[1].1 - p.1) * m_per_deg_lon, (w[1].0 - p.0) * 111_320.0);
                    let (dx, dy) = (bx - ax, by - ay);
                    let l2 = dx * dx + dy * dy;
                    let t = if l2 > 1e-6 {
                        (-(ax * dx + ay * dy) / l2).clamp(0.0, 1.0)
                    } else {
                        0.0
                    };
                    ((ax + t * dx).powi(2) + (ay + t * dy).powi(2)).sqrt()
                })
                .fold(f64::MAX, f64::min)
        };
        let mut offs: Vec<f64> = plan.path.iter().skip(1).map(|&p| to_paint(p)).collect();
        offs.sort_by(f64::total_cmp);
        let median = offs[offs.len() / 2];
        let p90 = offs[offs.len() * 9 / 10];
        println!(
            "{} points ({} riding the line) over {:.0} m: median {:.2} m off the paint, \
             p90 {:.2} m, worst {:.2} m",
            offs.len(),
            offs.iter().filter(|d| **d < 0.5).count(),
            plan.path
                .windows(2)
                .map(|w| {
                    let (dx, dy) = (
                        (w[0].1 - w[1].1) * m_per_deg_lon,
                        (w[0].0 - w[1].0) * 111_320.0,
                    );
                    (dx * dx + dy * dy).sqrt()
                })
                .sum::<f64>(),
            median,
            p90,
            offs.last().unwrap()
        );
        assert!(
            median < 0.5,
            "the route should ride the drawn line, median {median:.2} m off"
        );

        // And the arrival mirror, from where that sortie's taxi-in stopped:
        // the route must finish ON the stand, not at the taxiway node the
        // aircraft actually parked on (66 m short of W5, twice).
        let (plat, plon) = (36.680758, -4.502952);
        let stand = net.nearest_stand(plat, plon).expect("LEMG has stands").clone();
        let plan = net
            .plan_to_stand(plat, plon, &stand)
            .expect("a route to the stand");
        let end = *plan.path.last().unwrap();
        let short = {
            let (dx, dy) = (
                (stand.lon - end.1) * m_per_deg_lon,
                (stand.lat - end.0) * 111_320.0,
            );
            (dx * dx + dy * dy).sqrt()
        };
        println!("stand {} : route finishes {short:.1} m from it", stand.name);
        assert!(short < 5.0, "parks {short:.0} m short of stand {}", stand.name);
    }

    /// The real Southampton network behind the fix above.
    /// `cargo test -p atc -- --ignored live_eghi_runway_20 --nocapture`
    #[test]
    #[ignore] // needs the real X-Plane install; run with --ignored
    fn live_eghi_runway_20_holds_at_an_entrance() {
        let path =
            "/Users/duanelicudi/X-Plane 12/Global Scenery/Global Airports/Earth nav data/apt.dat";
        let Ok(Some(net)) = find_taxi_net(path, "EGHI") else {
            return; // no install, no verdict
        };
        // The GA apron stand the SF50 departed from, and the 20 threshold.
        let (slat, slon) = (50.953095, -1.358671);
        let (tlat, tlon) = (50.9589489, -1.3519495);
        let zone: Vec<String> = vec!["02".into(), "20".into()];
        let plan = net
            .plan_to_runway(slat, slon, tlat, tlon, 100.0, &zone)
            .expect("a route");
        let end = *plan.path.last().unwrap();
        let m = |a: (f64, f64), b: (f64, f64)| {
            let dx = (a.1 - b.1) * 111_320.0 * 50.95_f64.to_radians().cos();
            let dy = (a.0 - b.0) * 111_320.0;
            (dx * dx + dy * dy).sqrt()
        };
        println!(
            "holding point {} via {:?}, {:.0} m of route, ends {:.0} m from thr20",
            plan.holding_point,
            plan.via,
            plan.path.windows(2).map(|w| m(w[0], w[1])).sum::<f64>(),
            m(end, (tlat, tlon))
        );
        // The A5 dead end is at 50.954136,-1.357265 — 172 m abeam the strip
        // with no connection to it. Anywhere but there.
        assert!(
            m(end, (50.954136, -1.357265)) > 100.0,
            "must not end at the A5 dead-end stub: {end:?}"
        );
        assert!(
            !plan.lead_on.is_empty(),
            "a real entrance leaves the line-up a drawn lead-on"
        );
    }

    #[test]
    #[ignore] // needs the real X-Plane install; run with --ignored
    fn live_heathrow_taxi_route_plans() {
        let path =
            "/Users/duanelicudi/X-Plane 12/Global Scenery/Global Airports/Earth nav data/apt.dat";
        let t = std::time::Instant::now();
        let net = find_taxi_net(path, "EGLL")
            .unwrap()
            .expect("EGLL has a network");
        eprintln!(
            "EGLL taxi net: {} nodes, {} edges in {:?}",
            net.nodes.len(),
            net.edges.len(),
            t.elapsed()
        );
        assert!(net.nodes.len() > 500 && net.edges.len() > 300);
        // From T2 stands (51.4700, -0.4520) to the 27R threshold
        // (51.4775, -0.4325): a real route with named taxiways.
        let t = std::time::Instant::now();
        let plan = net
            .plan(
                51.4700,
                -0.4520,
                51.4775,
                -0.4325,
                HOLD_SHORT_STANDOFF_M,
                &[],
            )
            .expect("a route");
        eprintln!(
            "plan in {:?}: via {:?}, cross {:?}, hp {:?}",
            t.elapsed(),
            plan.via,
            plan.crossings,
            plan.holding_point
        );
        assert!(!plan.via.is_empty(), "named taxiways expected at Heathrow");
        assert!(
            t.elapsed().as_millis() < 200,
            "planning must not stall a reply"
        );
    }

    #[test]
    #[ignore] // needs the real X-Plane install; run with --ignored
    fn live_apt_dat_scan_is_fast_enough_to_speak() {
        let path =
            "/Users/duanelicudi/X-Plane 12/Global Scenery/Global Airports/Earth nav data/apt.dat";
        let t = std::time::Instant::now();
        let rw = find_runways(path, "LEAL").unwrap();
        let elapsed = t.elapsed();
        assert_eq!(rw.len(), 1);
        assert_eq!(
            (rw[0].ident_a.as_str(), rw[0].ident_b.as_str()),
            ("10", "28")
        );
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
        assert!(
            (l.course_mag - 31769.709 / 360.0).abs() < 0.01,
            "{}",
            l.course_mag
        );
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
        // Same Custom-Data-first rule as Config::from_env (which lives with
        // the crew crate): a Navigraph cycle outranks the stock files.
        fn env_or_first_existing(var: &str, custom: &str, stock: &str) -> String {
            std::env::var(var).unwrap_or_else(|_| {
                if std::path::Path::new(custom).exists() {
                    custom
                } else {
                    stock
                }
                .to_string()
            })
        }
        let nav_data_path = env_or_first_existing(
            "XPLANE_NAV_DATA",
            "/Users/duanelicudi/X-Plane 12/Custom Data/earth_nav.dat",
            "/Users/duanelicudi/X-Plane 12/Resources/default data/earth_nav.dat",
        );
        let cifp_dir = env_or_first_existing(
            "XPLANE_CIFP_DIR",
            "/Users/duanelicudi/X-Plane 12/Custom Data/CIFP",
            "/Users/duanelicudi/X-Plane 12/Resources/default data/CIFP",
        );
        let ils = airport_ils(&nav_data_path, "EGLL").expect("nav scan");
        for rwy in ["09L", "09R", "27L", "27R"] {
            let hit = ils.iter().find(|i| i.runway == rwy);
            assert!(hit.is_some(), "EGLL {rwy} ILS missing: {ils:?}");
            assert_eq!(hit.unwrap().gs_deg, Some(3.0), "{rwy}");
        }
        assert_eq!(
            ils_platform_alt_ft(&cifp_dir, "EGLL", "27R"),
            Some(2500.0),
            "27R platform from the CIFP FAF"
        );
    }

    // --- Terminal procedure parsing (real EGLL/LEMG CIFP records) ---

    // The ILS 27R final segment at Heathrow: initial fix, FAF, runway, missed.
    const EGLL_I27R: &str = "\
APPCH:010,I,I27R, ,CF27R,EG,P,C,E  I, ,   ,IF, ,IRR,EG,P,I,      ,0887,0123,    ,    ,I,03000,02500,     , ,   ,    ,   , , , , , ,0,D,S;
APPCH:020,I,I27R, ,FF27R,EG,P,C,E  F, ,   ,CF, ,IRR,EG,P,I,      ,0887,0098,2680,0025,G,02500,02500,     , ,   ,-300,   ,EGLL,EG,P,A, ,0,D,S;
APPCH:030,I,I27R, ,RW27R,EG,P,G,G  M, ,   ,CF, ,IRR,EG,P,I,      ,0887,0023,2680,0075, ,00136,     ,     , ,   ,-300,   , , , , , ,0,D,S;
APPCH:050,I,I27R, , , , , , E  , ,   ,CA, , , , , ,      ,    ,    ,3150,    ,+,03000,     ,     , ,   ,    ,   , , , , , ,0,D,S;
APPCH:010,A,I27R,BIG,BIG,EG,D, ,V  C, ,   ,IF, , , , , ,      ,    ,    ,    ,    ,-,220,    ,   , , , , , ,0,D,S;";

    #[test]
    fn appch_final_segment_parses_the_legs_in_order() {
        // transition=None keeps only the common/final portion (blank field 3),
        // NOT the BIG transition.
        let legs = parse_proc_legs(EGLL_I27R.lines(), ProcKind::Appch, "I27R", None);
        let types: Vec<&str> = legs.iter().map(|l| l.leg_type.as_str()).collect();
        assert_eq!(
            types,
            vec!["IF", "CF", "CF", "CA"],
            "final segment, BIG excluded"
        );

        // The FAF leg: FF27R, flagged, magnetic course 268, platform 2500.
        let faf = &legs[1];
        assert_eq!(faf.fix_ident, "FF27R");
        assert!(faf.is_faf);
        assert_eq!(faf.course_mag, Some(268.0));
        assert_eq!(faf.alt1_ft, Some(2500.0));

        // The runway leg: threshold at 136 ft.
        assert_eq!(legs[2].fix_ident, "RW27R");
        assert_eq!(legs[2].alt1_ft, Some(136.0));

        // The missed leg climbs on a heading to 3000 with no fix.
        let miss = &legs[3];
        assert_eq!(miss.leg_type, "CA");
        assert!(miss.fix_ident.is_empty());
        assert_eq!(miss.alt_desc, '+');
        assert_eq!(miss.alt1_ft, Some(3000.0));
        assert_eq!(miss.course_mag, Some(315.0));
    }

    #[test]
    fn appch_transition_is_included_when_named() {
        let legs = parse_proc_legs(EGLL_I27R.lines(), ProcKind::Appch, "I27R", Some("BIG"));
        assert!(
            legs.iter().any(|l| l.fix_ident == "BIG"),
            "BIG transition kept"
        );
        assert!(
            legs.iter().any(|l| l.fix_ident == "RW27R"),
            "common part still kept"
        );
    }

    // A Malaga SID off runway 30 (BLN4L): course-to-fix then track-to-fix legs.
    const LEMG_BLN4L: &str = "\
SID:005,5,BLN4L,RW30,MG930,LE,P,C,E   , ,   ,CF, ,MLG,LE,D, ,      ,3172,0061,3120,0050, ,     ,     ,13000,-,210,    ,   , , , , , , , , ;
SID:010,5,BLN4L,RW30,MG901,LE,P,C,E   , ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,     ,     ,     ,-,210,    ,   , , , , , , , , ;
SID:020,5,BLN4L,RW30,MG902,LE,P,C,E   , ,   ,TF, , , , , ,      ,    ,    ,    ,    ,+,07500,     ,     ,-,230,    ,   , , , , , , , , ;
SID:030,5,BLN4L,RW30,BLN,LE,D, ,VE  , ,   ,TF, , , , , ,      ,    ,    ,    ,    ,+,10000,     ,     , ,   ,    ,   , , , , , , , , ;";

    #[test]
    fn sid_runway_transition_parses() {
        let legs = parse_proc_legs(LEMG_BLN4L.lines(), ProcKind::Sid, "BLN4L", Some("RW30"));
        let fixes: Vec<&str> = legs.iter().map(|l| l.fix_ident.as_str()).collect();
        assert_eq!(fixes, vec!["MG930", "MG901", "MG902", "BLN"]);
        // First leg is course-to-fix 312 heading; last leg climbs to at-or-above
        // 10,000.
        assert_eq!(legs[0].leg_type, "CF");
        assert_eq!(legs[0].course_mag, Some(312.0));
        assert_eq!(legs[3].alt_desc, '+');
        assert_eq!(legs[3].alt1_ft, Some(10000.0));
    }

    #[test]
    fn approach_runway_strips_type_and_variant() {
        assert_eq!(approach_runway("I27R").as_deref(), Some("27R"));
        assert_eq!(approach_runway("R27R-Z").as_deref(), Some("27R"));
        assert_eq!(approach_runway("I20-Y").as_deref(), Some("20"));
        assert_eq!(approach_runway("D07").as_deref(), Some("07"));
        assert_eq!(approach_runway("RNAV"), None); // no runway digits at all
    }

    #[test]
    fn reconstructs_the_gibraltar_dme_approach_from_fixes() {
        // LXGB: no CIFP approach, but earth_fix.dat carries the runway-27 DME
        // fixes. Reconstruct the straight-in from them (the real Gibraltar case).
        let apt = std::env::temp_dir().join("xc-lxgb-apt.dat");
        let mut f = std::fs::File::create(&apt).unwrap();
        writeln!(f, "1 11 0 0 LXGB Gibraltar").unwrap();
        // Runway 09/27: 27 threshold at the east end (~-5.341), 09 at the west.
        writeln!(f, "100 46 0 0 0 0 0 0 09 36.1509000 -5.3581000 0 0 0 0 0 0 27 36.1515000 -5.3412000 0 0 0 0 0 0").unwrap();
        f.flush().unwrap();
        let fix = std::env::temp_dir().join("xc-lxgb-fix.dat");
        let mut f = std::fs::File::create(&fix).unwrap();
        // The real LXGB fixes (earth_fix.dat), plus an unrelated field's fix.
        writeln!(
            f,
            "36.157361111 -5.240083333 27D05 LXGB LX 5906505 RWY27 D05"
        )
        .unwrap();
        writeln!(
            f,
            "36.158305556 -5.219500000 27D06 LXGB LX 5906505 RWY27 D06"
        )
        .unwrap();
        writeln!(
            f,
            "36.162027778 -5.137250000 27D10 LXGB LX 5906505 RWY27 D10"
        )
        .unwrap();
        writeln!(
            f,
            "36.104666667 -5.405333333 09D05 LXGB LX 5906505 RWY09 D05"
        )
        .unwrap();
        writeln!(
            f,
            "36.088000000 -5.404666667 09D06 LXGB LX 5906505 RWY09 D06"
        )
        .unwrap();
        // A synthetic 09 fix that IS on the 09 centreline, 4 nm out.
        writeln!(
            f,
            "36.148200000 -5.440600000 09D04 LXGB LX 5906505 RWY09 D04"
        )
        .unwrap();
        writeln!(f, "40.000000000 -3.000000000 27D05 LEMD LE 0 other field").unwrap();
        f.flush().unwrap();

        let airport = find_airport(apt.to_str().unwrap(), "LXGB")
            .unwrap()
            .unwrap();
        let legs = reconstruct_approach(fix.to_str().unwrap(), &airport, "LXGB", "27");
        // Three DME fixes (27D10/06/05) + the threshold, farthest first.
        let idents: Vec<&str> = legs.iter().map(|l| l.fix_ident.as_str()).collect();
        assert_eq!(idents, vec!["27D10", "27D06", "27D05", "RW27"]);
        // Descending altitude on a 3° path — outer fix highest.
        assert!(legs[0].alt1_ft.unwrap() > legs[2].alt1_ft.unwrap());
        assert!(
            legs[0].alt1_ft.unwrap() > 2500.0,
            "10 DME ~ 3000 ft: {:?}",
            legs[0].alt1_ft
        );
        // The innermost DME fix (27D05) is the FAF; the threshold is not.
        assert!(legs[2].is_faf, "27D05 is the FAF");
        assert!(!legs[3].is_faf);
        // The 09 fixes stay out of the 27 reconstruction.
        assert!(!idents.contains(&"09D05"));
        // A runway with no DME fixes → nothing to reconstruct.
        assert!(reconstruct_approach(fix.to_str().unwrap(), &airport, "LXGB", "18").is_empty());

        // Runway 09: the real 09D05/09D06 sit on the VOR/DME approach's offset
        // track, ~50 degrees south of the centreline. They are NOT straight-in
        // fixes (flying them as such crashed a Baron into the North Front,
        // 2026-09-11); only the on-course 09D04 survives, and it is the FAF.
        let legs = reconstruct_approach(fix.to_str().unwrap(), &airport, "LXGB", "09");
        let idents: Vec<&str> = legs.iter().map(|l| l.fix_ident.as_str()).collect();
        assert_eq!(idents, vec!["09D04", "RW09"], "off-course DME fixes dropped");
        assert!(legs[0].is_faf);
        let (xtk_m, along_m) = runway_frame_m(
            36.1509, -5.3581, runway_course_deg(&airport, "09").unwrap(), 36.1482, -5.4406,
        );
        assert!(xtk_m.abs() < 100.0, "on the centreline: xtk {xtk_m:.0} m");
        assert!((along_m - 4.0 * 1852.0).abs() < 200.0, "4 nm out: {along_m:.0} m");
        // And the offset fix, in the same frame: abeam, not ahead.
        let (xtk_m, along_m) = runway_frame_m(
            36.1509, -5.3581, runway_course_deg(&airport, "09").unwrap(), 36.104666667, -5.405333333,
        );
        assert!(xtk_m.abs() > 4000.0, "09D05 is {xtk_m:.0} m off the centreline");
        assert!(along_m < 5000.0, "with only {along_m:.0} m to run");
    }

    #[test]
    fn no_on_course_dme_fixes_means_no_straight_in() {
        let apt = std::env::temp_dir().join("xc-lxgb-apt-nofix.dat");
        let mut f = std::fs::File::create(&apt).unwrap();
        writeln!(f, "1 11 0 0 LXGB Gibraltar").unwrap();
        writeln!(f, "100 46 0 0 0 0 0 0 09 36.1509000 -5.3581000 0 0 0 0 0 0 27 36.1515000 -5.3412000 0 0 0 0 0 0").unwrap();
        f.flush().unwrap();
        let fix = std::env::temp_dir().join("xc-lxgb-fix-nofix.dat");
        let mut f = std::fs::File::create(&fix).unwrap();
        writeln!(f, "36.104666667 -5.405333333 09D05 LXGB LX 5906505 RWY09 D05").unwrap();
        writeln!(f, "36.088000000 -5.404666667 09D06 LXGB LX 5906505 RWY09 D06").unwrap();
        f.flush().unwrap();
        let airport = find_airport(apt.to_str().unwrap(), "LXGB").unwrap().unwrap();
        // The live Gibraltar case: every 09 fix is off course, so there is no
        // reconstruction at all and the arrival falls back to a visual landing.
        assert!(reconstruct_approach(fix.to_str().unwrap(), &airport, "LXGB", "09").is_empty());
    }

    #[test]
    fn resolver_fills_runway_and_navaid_fixes() {
        // A tiny airport with runway 30/12, plus a nav-data file holding the
        // MG930 waypoint and the BLN VOR — the resolver must place both.
        let apt = std::env::temp_dir().join("xc-proc-apt.dat");
        let mut f = std::fs::File::create(&apt).unwrap();
        writeln!(f, "1 16 0 0 LEMG Malaga").unwrap();
        // Runway 30/12: end A "30" near (36.67,-4.50), end B "12".
        writeln!(f, "100 45 0 0 0 0 0 0 30 36.6700000 -4.5000000 0 0 0 0 0 0 12 36.6600000 -4.4800000 0 0 0 0 0 0").unwrap();
        f.flush().unwrap();
        let fix = std::env::temp_dir().join("xc-proc-fix.dat");
        let mut f = std::fs::File::create(&fix).unwrap();
        writeln!(f, "36.7535555 -4.5927222 MG930 LEMG LE 4464727 MG930").unwrap();
        f.flush().unwrap();
        let nav = std::env::temp_dir().join("xc-proc-nav.dat");
        let mut f = std::fs::File::create(&nav).unwrap();
        writeln!(
            f,
            "3 38.1525555 -3.6247222 1772 11620 130 1.000 BLN ENRT LE BAILEN VOR/DME"
        )
        .unwrap();
        f.flush().unwrap();

        let airport = find_airport(apt.to_str().unwrap(), "LEMG")
            .unwrap()
            .unwrap();
        let mut legs = parse_proc_legs(
            "SID:005,5,X,RW30,MG930,LE,P,C,E   , ,   ,CF, ,MLG,LE,D, ,      ,0,0,3120,0050, ,     ,     ,     , ,   ,    ,   ;\n\
             SID:010,5,X,RW30,BLN,LE,D, ,VE  , ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,     ,     ,     ;\n\
             SID:020,5,X,RW30,RW30,LE,P,G,G  M, ,   ,CF, , , , , ,      ,0,0,3120,0050, ,     ,     ,     ;"
                .lines(),
            ProcKind::Sid,
            "X",
            Some("RW30"),
        );
        resolve_proc_fixes(
            fix.to_str().unwrap(),
            nav.to_str().unwrap(),
            &airport,
            "LEMG",
            &mut legs,
            36.67,
            -4.5,
        );
        // MG930 waypoint from earth_fix.dat.
        assert_eq!(legs[0].fix.as_ref().unwrap().ident, "MG930");
        assert!((legs[0].fix.as_ref().unwrap().lat - 36.7535555).abs() < 1e-4);
        // BLN VOR from earth_nav.dat.
        assert!((legs[1].fix.as_ref().unwrap().lat - 38.1525555).abs() < 1e-4);
        // RW30 threshold from apt.dat.
        assert!((legs[2].fix.as_ref().unwrap().lat - 36.67).abs() < 1e-6);
    }

    /// Real-install smoke test: Malaga publishes procedures; parse + resolve the
    /// ILS to a runway and one of its SIDs end to end.
    /// `cargo test -p atc -- --ignored real_install_procedures`
    #[test]
    #[ignore]
    fn real_install_procedures() {
        fn first_existing(custom: &str, stock: &str) -> String {
            if std::path::Path::new(custom).exists() {
                custom
            } else {
                stock
            }
            .to_string()
        }
        let cifp = first_existing(
            "/Users/duanelicudi/X-Plane 12/Custom Data/CIFP",
            "/Users/duanelicudi/X-Plane 12/Resources/default data/CIFP",
        );
        let fix_path = first_existing(
            "/Users/duanelicudi/X-Plane 12/Custom Data/earth_fix.dat",
            "/Users/duanelicudi/X-Plane 12/Resources/default data/earth_fix.dat",
        );
        let nav_path = first_existing(
            "/Users/duanelicudi/X-Plane 12/Custom Data/earth_nav.dat",
            "/Users/duanelicudi/X-Plane 12/Resources/default data/earth_nav.dat",
        );
        let apt =
            "/Users/duanelicudi/X-Plane 12/Global Scenery/Global Airports/Earth nav data/apt.dat";
        let airport = find_airport(apt, "LEMG").unwrap().expect("LEMG in apt.dat");
        let (near_lat, near_lon) = airport.position().unwrap();

        // Pick the ILS approach for 12 or 30 and resolve every leg's fix.
        for rwy in ["12", "30", "13", "31"] {
            if let Some(appch) = best_approach(&cifp, "LEMG", rwy) {
                let mut legs = load_proc_legs(&cifp, "LEMG", ProcKind::Appch, &appch, None);
                assert!(!legs.is_empty(), "{appch} has legs");
                resolve_proc_fixes(
                    &fix_path, &nav_path, &airport, "LEMG", &mut legs, near_lat, near_lon,
                );
                let named: Vec<_> = legs.iter().filter(|l| !l.fix_ident.is_empty()).collect();
                let resolved = named.iter().filter(|l| l.fix.is_some()).count();
                assert!(
                    resolved >= named.len().saturating_sub(1),
                    "{appch}: resolved {resolved}/{} fixes",
                    named.len()
                );
                println!("LEMG {rwy}: approach {appch}, {} legs", legs.len());
            }
        }
        // At least one SID resolves too.
        let sid =
            sid_for_runway(&cifp, "LEMG", "30").or_else(|| sid_for_runway(&cifp, "LEMG", "12"));
        let sid = sid.expect("LEMG has a SID for 30/12");
        let mut legs = load_proc_legs(&cifp, "LEMG", ProcKind::Sid, &sid, Some("RW30"));
        if legs.is_empty() {
            legs = load_proc_legs(&cifp, "LEMG", ProcKind::Sid, &sid, Some("RW12"));
        }
        resolve_proc_fixes(
            &fix_path, &nav_path, &airport, "LEMG", &mut legs, near_lat, near_lon,
        );
        assert!(
            legs.iter().any(|l| l.fix.is_some()),
            "SID {sid} resolved a fix"
        );

        // Gibraltar: no coded approach, but the DME fixes reconstruct the
        // straight-in (the whole reason this exists). Runway 27's fixes must
        // chain farthest-first to the threshold, over the water.
        let apt =
            "/Users/duanelicudi/X-Plane 12/Global Scenery/Global Airports/Earth nav data/apt.dat";
        if let Ok(Some(lxgb)) = find_airport(apt, "LXGB") {
            assert!(
                list_procedures(&cifp, "LXGB", ProcKind::Appch).is_empty(),
                "LXGB has no coded approach"
            );
            let recon = reconstruct_approach(&fix_path, &lxgb, "LXGB", "27");
            assert!(
                recon.len() >= 3,
                "reconstructed 27 straight-in from DME fixes: {recon:?}"
            );
            assert!(
                recon.last().unwrap().fix_ident.starts_with("RW"),
                "ends at the threshold"
            );
            assert!(recon.iter().any(|l| l.is_faf), "an FAF is flagged");
        }
    }

    #[test]
    fn none_when_no_runway_nearby() {
        let path = std::env::temp_dir().join("xc-nav-test2.dat");
        let mut f = std::fs::File::create(&path).unwrap();
        writeln!(
            f,
            " 4  47.466600000 -122.317800000  359 11075 18 123840.337 IFAR KSEA K1 28 ILS-cat-I"
        )
        .unwrap();
        f.flush().unwrap();
        // Aircraft far from the only runway-28 → none within 60nm.
        let r = find_ils(path.to_str().unwrap(), "28", 36.71, -4.55, 60.0).unwrap();
        assert!(r.is_none());
    }

    /// A close-in stand must still get a route.
    ///
    /// EGHI 2026-07-30: after landing, the aircraft sat 224 m from taxiway
    /// node 44 and the nearest suitable stand was another 69 m past the same
    /// node — so both ends snapped to node 44, `start == goal`, and the
    /// planner returned None. The tower then answered "taxi to parking, this
    /// frequency" with no route attached, and the crew's taxi-in waited out
    /// its full 60 s before giving up with "no taxi-in clearance came back".
    /// The clearance HAD come; the route had not.
    #[test]
    fn a_stand_on_the_same_node_still_gets_a_route() {
        // Two nodes joined by one taxiway; the stand sits just past node 1,
        // so a nearby aircraft and the stand both snap to it.
        let mut net = TaxiNet::default();
        net.nodes.insert(1, (50.9500, -1.3600));
        net.nodes.insert(2, (50.9520, -1.3600));
        net.edges.push(TaxiEdge {
            a: 1,
            b: 2,
            name: "Alpha".into(),
            runway: false,
            oneway: false,
            zone_of: Vec::new(),
        });
        let (stand_lat, stand_lon) = (50.94995, -1.35985);
        let (ac_lat, ac_lon) = (50.95005, -1.36005);
        let plan = net
            .plan(ac_lat, ac_lon, stand_lat, stand_lon, 0.0, &[])
            .expect("a close stand is a short taxi, not an impossible one");
        assert!(
            plan.path.len() >= 2,
            "the publication drops anything shorter: {:?}",
            plan.path
        );
        // It must START AT THE AIRCRAFT. Starting at the snapped node put the
        // route 224 m from a stopped aircraft at EGHI, so the drive's arrival
        // test was true on tick one and it "secured" in two seconds without
        // moving.
        let first = plan.path[0];
        assert!(
            (first.0 - ac_lat).abs() < 1e-6 && (first.1 - ac_lon).abs() < 1e-6,
            "route must begin where the aircraft is, got {first:?}"
        );
        let end = *plan.path.last().unwrap();
        assert!(
            (end.0 - stand_lat).abs() < 1e-6 && (end.1 - stand_lon).abs() < 1e-6,
            "the route must END at the stand, got {end:?}"
        );
        assert_eq!(plan.seg.len(), plan.path.len() - 1, "seg tracks path");

        // A RUNWAY destination we are already at is still nothing to do. This
        // was written as `.is_none() || true`, which asserts nothing and hid
        // behind a "runway behaviour unchanged" message — probed, the call
        // really does return None here, so assert it.
        assert!(
            net.plan_to_runway(50.95005, -1.36005, 50.94995, -1.35985, 0.0, &[])
                .is_none(),
            "a runway destination on the node we are already at is no taxi"
        );
        // ...and a runway destination that IS somewhere else still plans, even
        // though this net has no runway edges for `runway_entrance_near` to
        // find. No authored entrance degrades to the plain snap; it is never a
        // refusal, because "no entrance in the data" is not a reason to refuse
        // to taxi (776 runway ends worldwide have none).
        let far = net
            .plan_to_runway(50.95005, -1.36005, 50.9520, -1.3600, 0.0, &[])
            .expect("no runway edges means the plain snap, not a refusal");
        let plain = net
            .plan(50.95005, -1.36005, 50.9520, -1.3600, 0.0, &[])
            .expect("the plain plan is the yardstick");
        assert_eq!(
            far.path, plain.path,
            "with no entrance authored, they agree"
        );
    }

    /// A strip running north with a taxiway parallel to it, joined abeam each
    /// runway node. Nodes 1..3 are the runway, 11..13 the taxiway.
    fn vacate_net() -> TaxiNet {
        let mut net = TaxiNet::default();
        // Runway 36/18, three nodes up the centreline (0, 400, 800 m north).
        net.nodes.insert(1, (51.0000, -1.0000));
        net.nodes.insert(2, (51.0036, -1.0000));
        net.nodes.insert(3, (51.0072, -1.0000));
        // Parallel taxiway ~150 m east, abeam each.
        net.nodes.insert(11, (51.0000, -0.99786));
        net.nodes.insert(12, (51.0036, -0.99786));
        net.nodes.insert(13, (51.0072, -0.99786));
        let e = |a, b, name: &str, runway, zone: &[&str]| TaxiEdge {
            a,
            b,
            name: name.into(),
            runway,
            oneway: false,
            zone_of: zone.iter().map(|s| s.to_string()).collect(),
        };
        net.edges.push(e(1, 2, "36/18", true, &["36", "18"]));
        net.edges.push(e(2, 3, "36/18", true, &["36", "18"]));
        // The links onto the taxiway sit INSIDE the runway's active zone...
        net.edges.push(e(1, 11, "A1", false, &["36", "18"]));
        net.edges.push(e(2, 12, "A2", false, &["36", "18"]));
        net.edges.push(e(3, 13, "A3", false, &["36", "18"]));
        // ...and the parallel taxiway itself is outside it.
        net.edges.push(e(11, 12, "Alpha", false, &[]));
        net.edges.push(e(12, 13, "Alpha", false, &[]));
        net
    }

    /// Vacating is the completion of the landing clearance, not a new
    /// instruction — so it plans from where autoland stopped us without
    /// asking anybody, and it must not stop with the tail still on the paint.
    #[test]
    fn a_vacate_leaves_the_runway_forwards_and_gets_fully_clear() {
        let net = vacate_net();
        let zone = ["36".to_string(), "18".to_string()];
        // Stopped just short of node 2, rolled out northbound on 36.
        let (lat, lon) = (51.0034, -1.0000);
        let plan = net
            .plan_vacate(lat, lon, 0.0, &zone, VACATE_CLEAR_M)
            .expect("there is an exit ahead");

        // Starts at the aircraft — a route that begins at the snapped node
        // reads as already-arrived on the drive's first tick (the EGHI stand
        // lesson, one module over).
        assert!(
            (plan.path[0].0 - lat).abs() < 1e-9 && (plan.path[0].1 - lon).abs() < 1e-9,
            "must start where the aircraft is, got {:?}",
            plan.path[0]
        );
        assert_eq!(plan.seg.len(), plan.path.len() - 1, "seg tracks path");

        // It does NOT stop on the link: that edge is inside the active zone,
        // so the walk carries on down Alpha until it is past the boundary.
        assert!(plan.path.len() >= 3, "must not stop on the link: {plan:?}");
        assert_eq!(plan.exit_name, "Alpha");
        let end = *plan.path.last().unwrap();
        let east_m = (end.1 - lon) * 111_320.0 * lat.to_radians().cos();
        assert!(east_m > 140.0, "must end off the strip, {east_m} m east");

        // Never BACKWARDS down the runway. Landing the other way from the same
        // spot, the exit taken must be the one that is now ahead.
        let south = net
            .plan_vacate(lat, lon, 180.0, &zone, VACATE_CLEAR_M)
            .expect("there is an exit ahead that way too");
        assert!(
            south.path[1].0 < lat,
            "a southbound rollout must exit south, got {:?}",
            south.path[1]
        );
    }

    /// EGLL 27L, 2026-09-17. The nearest junction ahead was 84 m away and its
    /// only link, N4W, ran back east-north-east — a rapid exit built for 09R
    /// traffic, 169 degrees off our rollout. The vacate took it: up the
    /// runway, a turn on the strip, and back east past the touchdown point,
    /// 624 m of route and three minutes of occupancy with traffic inside a
    /// mile. The turn-off that works, N5W at 54 degrees, was 220 m further
    /// on. A junction is only an exit if you can turn onto what leaves it.
    #[test]
    fn a_vacate_refuses_an_exit_it_would_have_to_turn_back_to_take() {
        let mut net = TaxiNet::default();
        // Runway 09R/27L along a parallel of latitude; we roll WEST on 27L.
        let m = |east_m: f64| -1.0 + east_m / (111_320.0 * 51.0_f64.to_radians().cos());
        net.nodes.insert(1, (51.0, m(0.0)));
        net.nodes.insert(2, (51.0, m(-300.0)));
        net.nodes.insert(3, (51.0, m(-700.0)));
        // Node 2's only turn-off is angled BACK east (the 09R rapid exit).
        net.nodes.insert(12, (51.0009, m(-210.0)));
        // Node 3 offers a proper forward turn-off, north-west.
        net.nodes.insert(13, (51.0009, m(-790.0)));
        net.nodes.insert(23, (51.0018, m(-790.0)));
        let e = |a, b, name: &str, runway, zone: &[&str]| TaxiEdge {
            a,
            b,
            name: name.into(),
            runway,
            oneway: false,
            zone_of: zone.iter().map(|s| s.to_string()).collect(),
        };
        net.edges.push(e(1, 2, "09R/27L", true, &["09R", "27L"]));
        net.edges.push(e(2, 3, "09R/27L", true, &["09R", "27L"]));
        net.edges.push(e(2, 12, "N4W", false, &["09R", "27L"]));
        net.edges.push(e(3, 13, "N5W", false, &["09R", "27L"]));
        net.edges.push(e(13, 23, "Block 81", false, &[]));
        let zone = ["27L".to_string(), "09R".to_string()];
        // Stopped on the centreline, heading west, N4W's junction 300 m ahead
        // and N5W's 700 m ahead.
        let plan = net
            .plan_vacate(51.0, m(0.0), 270.0, &zone, VACATE_CLEAR_M)
            .expect("the further exit is still an exit");
        assert!(
            plan.seg.iter().all(|(n, _)| n != "N4W"),
            "the backwards rapid exit must not be taken: {:?}",
            plan.seg
        );
        assert_eq!(plan.exit_name, "Block 81", "walks out through N5W");
        // And every step of it keeps going the way we were rolling: no point
        // on the route may lie east of where the aircraft stopped.
        let east_of_start = plan.path.iter().any(|p| p.1 > m(0.0) + 1e-9);
        assert!(!east_of_start, "the route doubles back: {:?}", plan.path);

        // A field where the ONLY turn-off is the backwards one yields no plan
        // at all, and the caller taxis in from the runway — the same
        // degradation the dead end makes, and better than a U-turn on it.
        net.edges
            .retain(|e| e.name != "N5W" && e.name != "Block 81");
        assert!(
            net.plan_vacate(51.0, m(0.0), 270.0, &zone, VACATE_CLEAR_M)
                .is_none(),
            "no exit is better than a reversal"
        );
    }

    /// The two ways the data can be thin. Neither is a reason to sit on a
    /// runway, so neither is a refusal — the same rule `plan_to_runway`
    /// follows for the 776 runway ends with no authored entrance.
    #[test]
    fn a_vacate_degrades_rather_than_refusing() {
        let zone = ["36".to_string(), "18".to_string()];

        // No authored ACTIVE ZONES: nothing marks the boundary, so the walk
        // just runs its clearance distance out from the exit. Still a plan.
        let mut no_zone = vacate_net();
        for e in &mut no_zone.edges {
            e.zone_of.clear();
        }
        let p = no_zone
            .plan_vacate(51.0034, -1.0000, 0.0, &zone, VACATE_CLEAR_M)
            .expect("no zones is not a refusal");
        assert!(p.path.len() >= 2);

        // No authored RUNWAY edges: nothing to be adjacent to, so every
        // taxiway node is a candidate and the nearest ahead wins.
        let mut no_rwy = vacate_net();
        no_rwy.edges.retain(|e| !e.runway);
        let p = no_rwy
            .plan_vacate(51.0034, -1.0000, 0.0, &zone, VACATE_CLEAR_M)
            .expect("no runway edges is not a refusal either");
        assert!(p.path.len() >= 2);

        // No network at all is the one honest None: there is nothing to drive
        // along, and the caller keeps its old behaviour.
        assert!(TaxiNet::default()
            .plan_vacate(51.0034, -1.0000, 0.0, &zone, VACATE_CLEAR_M)
            .is_none());
    }

    /// A junction whose only taxiway link is authored one-way TOWARD the
    /// runway (standard ground-flow authoring) is a dead end for the outward
    /// walk. Before the audit fix this returned a one-leg "plan" from the
    /// aircraft to a node ON the centreline — a drive down the runway that
    /// taxi_in then reported as a vacate. No plan is the honest answer.
    #[test]
    fn a_dead_end_junction_is_a_refusal_not_a_runway_plan() {
        let mut net = TaxiNet::default();
        net.nodes.insert(1, (51.0000, -1.0000));
        net.nodes.insert(2, (51.0036, -1.0000));
        net.nodes.insert(3, (51.0072, -1.0000));
        net.nodes.insert(12, (51.0036, -0.99786));
        let e = |a: i64, b: i64, name: &str, runway: bool, oneway: bool, z: &[&str]| TaxiEdge {
            a,
            b,
            name: name.into(),
            runway,
            oneway,
            zone_of: z.iter().map(|s| s.to_string()).collect(),
        };
        net.edges.push(e(1, 2, "36/18", true, false, &["36", "18"]));
        net.edges.push(e(2, 3, "36/18", true, false, &["36", "18"]));
        // The ONLY link at the junction, one-way toward the runway.
        net.edges.push(e(12, 2, "A2", false, true, &["36", "18"]));
        let zone = ["36".to_string(), "18".to_string()];
        assert!(
            net.plan_vacate(51.0000, -1.0000, 0.0, &zone, VACATE_CLEAR_M)
                .is_none(),
            "an untraversable exit must refuse, never plan down the strip"
        );
    }

    /// A vacate is planned FROM the strip. From the parallel taxiway — the
    /// give-up-then-retry case, where the landing clearance still stands —
    /// the nearest-junction-ahead snap would aim the aircraft back AT the
    /// runway, so where runway edges exist to judge by, off-strip refuses.
    #[test]
    fn a_vacate_from_the_taxiway_is_a_refusal() {
        let net = vacate_net();
        let zone = ["36".to_string(), "18".to_string()];
        // On Alpha, 150 m east of the centreline, pointing up the taxiway.
        assert!(
            net.plan_vacate(51.0034, -0.99786, 0.0, &zone, VACATE_CLEAR_M)
                .is_none(),
            "already off the strip: nothing to vacate"
        );
        // ...and thin data still cannot judge it, so it keeps the old
        // behaviour rather than refusing (the 776-ends rule).
        let mut no_rwy = vacate_net();
        no_rwy.edges.retain(|e| !e.runway);
        assert!(no_rwy
            .plan_vacate(51.0034, -0.99786, 0.0, &zone, VACATE_CLEAR_M)
            .is_some());
    }

    /// At an intersecting-runway field the way off our strip must never be
    /// ACROSS the other one. `zone` names only the runway we landed on, so a
    /// plain in-our-zone test read a link inside the crossing runway's zone
    /// as clean pavement and preferred it over our own exit stub.
    #[test]
    fn the_walk_prefers_our_exit_stub_over_a_crossing_runways_zone() {
        let mut net = vacate_net();
        // A link west from the same junction, inside the CROSSING runway's
        // active zone (drawn as a plain taxiway edge, the EGLL-N7 pattern).
        net.nodes.insert(22, (51.0036, -1.00214));
        net.edges.push(TaxiEdge {
            a: 2,
            b: 22,
            name: "B".into(),
            runway: false,
            oneway: false,
            zone_of: vec!["09".to_string(), "27".to_string()],
        });
        let zone = ["36".to_string(), "18".to_string()];
        let plan = net
            .plan_vacate(51.0034, -1.0000, 0.0, &zone, VACATE_CLEAR_M)
            .expect("still plans");
        // The first step off the junction is our own stub EAST onto Alpha,
        // never the westbound link across the other strip.
        let step = plan.path[2];
        assert!(
            step.1 > -1.0000,
            "walked across the crossing runway's zone: {plan:?}"
        );
    }
}
