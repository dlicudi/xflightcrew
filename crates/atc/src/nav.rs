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
    /// The localizer's true course, degrees: the geometry (which side of the
    /// antenna we are on) is true, the course set on the OBS magnetic.
    pub course_true: f64,
    /// The localizer antenna (the FAR end of the runway it serves).
    pub lat: f64,
    pub lon: f64,
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
        // "28" matches 28L and 28R (the nearest wins); "28L" only 28L.
        if runway_number(f[10]) != Some(want) || !runway_matches(f[10], runway) {
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
            let (course_mag, course_true) = loc_courses(f[6].parse::<f64>().unwrap_or(0.0));
            best = Some(Ils {
                freq_10khz: f[4].parse().unwrap_or(0),
                course_mag,
                course_true,
                lat: rlat,
                lon: rlon,
                airport: f[8].to_string(),
                runway: f[10].to_string(),
                distance_nm: d,
            });
        }
    }
    Ok(best)
}

/// A localizer row's bearing field: `magnetic x 360 + true` since XP 11.50
/// (EGPH 24: 86278.824 = 239 x 360 + 238.824). Dividing the whole field by 360
/// read 239.66 for a 239 course. Returns (magnetic, true). A plain bearing
/// (older data, under 360) is both.
pub fn loc_courses(field: f64) -> (f64, f64) {
    if field < 360.0 {
        let c = field.rem_euclid(360.0);
        return (c, c);
    }
    let mag = (field / 360.0).floor().rem_euclid(360.0);
    let tru = field.rem_euclid(360.0);
    (mag, tru)
}

/// What an ILS chart tells a crew beyond the frequency: the threshold (so
/// the decision altitude, 200 ft above it) and the missed approach altitude.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IlsChart {
    pub threshold_ft: Option<f64>,
    pub missed_alt_ft: Option<f64>,
}

impl IlsChart {
    /// The CAT I decision altitude: 200 ft above the threshold.
    pub fn decision_alt_ft(&self) -> Option<f64> {
        self.threshold_ft.map(|t| t + 200.0)
    }
}

/// The chart figures from one approach's legs: the runway fix's altitude
/// is the threshold, and the highest altitude on the legs AFTER it is the
/// missed approach altitude. Pure.
pub fn ils_chart_from_legs(legs: &[ProcLeg], runway: &str) -> IlsChart {
    let rw = legs
        .iter()
        .position(|l| l.fix_ident.strip_prefix("RW").is_some_and(|r| runway_matches(r, runway)));
    let threshold_ft = rw.and_then(|i| legs[i].alt1_ft);
    let missed_alt_ft = rw.and_then(|i| {
        legs[i + 1..]
            .iter()
            .filter_map(|l| l.alt1_ft)
            .fold(None, |m: Option<f64>, a| Some(m.map_or(a, |m| m.max(a))))
    });
    IlsChart { threshold_ft, missed_alt_ft }
}

/// The ILS chart for `runway` at `icao` from the CIFP: the first ILS
/// approach ('I' ident) to that runway. `None` without one.
pub fn ils_chart(cifp_dir: &str, icao: &str, runway: &str) -> Option<IlsChart> {
    let ident = approach_candidates(cifp_dir, icao, runway)
        .into_iter()
        .find(|id| id.starts_with('I'))?;
    let legs = load_proc_legs(cifp_dir, icao, ProcKind::Appch, &ident, None);
    let chart = ils_chart_from_legs(&legs, runway);
    (chart.threshold_ft.is_some() || chart.missed_alt_ft.is_some()).then_some(chart)
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
                course_mag: loc_courses(f[6].parse::<f64>().unwrap_or(0.0)).0,
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
    /// The 100-row's surface code (1 asphalt, 2 concrete, 3 grass, 4 dirt,
    /// 5 gravel, 20-38 / 50-57 the XP12 asphalt and concrete shades);
    /// 0 = unknown, read as hard.
    pub surface: u32,
}

impl Runway {
    /// Asphalt or concrete (or unknown). Grass, dirt, gravel, lakebed, water
    /// and snow are not.
    pub fn is_hard(&self) -> bool {
        matches!(self.surface, 0 | 1 | 2 | 15 | 20..=38 | 50..=57)
    }

    pub fn length_m(&self) -> f64 {
        haversine_nm(self.lat_a, self.lon_a, self.lat_b, self.lon_b) * 1852.0
    }

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
    /// The field's transition LEVEL in feet (apt.dat `1302 transition_level`,
    /// "7000" at EGKK, "18000" at KJFK; a written "FL070" or a bare "070"
    /// is hundreds). At or above it a controller says a flight level; below
    /// it an altitude in feet with the QNH. `None` = the file says nothing,
    /// and every altitude stays in feet as before.
    pub transition_level_ft: Option<f64>,
    /// apt.dat `1302 allows_circuits` (#182): `Some(false)` where the field's
    /// data says no circuits (LEMG and EGKK say 0; 317 fields say no, 90
    /// yes, 2026-09-29), `None` when the file is silent.
    pub allows_circuits: Option<bool>,
    /// apt.dat `1101 <rwy> left|right`, the VFR traffic pattern side per
    /// runway end inside the `1000` flows (LEMG: 13 left, 31 left), in file
    /// order, one entry per end: `Some(true)` left, `None` when the flows
    /// disagree. Read through [`AirportInfo::pattern_left`].
    pub pattern_sides: Vec<(String, Option<bool>)>,
}

/// A `1302 allows_circuits` value: "0" no, "1" yes, else unknown.
pub fn parse_allows_circuits(v: &str) -> Option<bool> {
    match v.trim() {
        "0" => Some(false),
        "1" => Some(true),
        _ => None,
    }
}

/// One `1101 <rwy> left|right` row (fields after the code) into `sides`:
/// the first flow naming an end sets it, a flow that disagrees makes it
/// `None` (no published side is better than a wrong one).
pub fn note_pattern_side<'a>(
    sides: &mut Vec<(String, Option<bool>)>,
    mut fields: impl Iterator<Item = &'a str>,
) {
    let (Some(rwy), Some(side)) = (fields.next(), fields.next()) else {
        return;
    };
    let left = match side {
        s if s.eq_ignore_ascii_case("left") => true,
        s if s.eq_ignore_ascii_case("right") => false,
        _ => return,
    };
    match sides.iter_mut().find(|(r, _)| same_runway_end(r, rwy)) {
        Some((_, known)) if *known != Some(left) => *known = None,
        Some(_) => {}
        None => sides.push((rwy.to_string(), Some(left))),
    }
}

/// Two runway designators name the same end: "09" and "9", "08r" and "08R".
pub fn same_runway_end(a: &str, b: &str) -> bool {
    let norm = |s: &str| s.trim().trim_start_matches('0').to_ascii_uppercase();
    norm(a) == norm(b)
}

/// A `1302 transition_level` value in feet: "7000" as written, "FL070" and a
/// bare "070" (under 1,000) as flight levels. `None` for anything else ("By
/// ATC" is in the file) and for zero.
pub fn parse_transition_level(v: &str) -> Option<f64> {
    let v = v.trim();
    let (fl, digits) = match v.get(..2) {
        Some(p) if p.eq_ignore_ascii_case("FL") => (true, &v[2..]),
        _ => (false, v),
    };
    let n: f64 = digits.parse().ok()?;
    if n <= 0.0 {
        return None;
    }
    Some(if fl || n < 1000.0 { n * 100.0 } else { n })
}

impl AirportInfo {
    /// The published VFR pattern side for runway end `rwy` (apt.dat `1101`,
    /// #182): `Some(true)` left, `Some(false)` right, `None` unpublished or
    /// disagreeing between the field's flows.
    pub fn pattern_left(&self, rwy: &str) -> Option<bool> {
        self.pattern_sides
            .iter()
            .find(|(r, _)| same_runway_end(r, rwy))
            .and_then(|(_, left)| *left)
    }

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
    let mut transition_level_ft = None;
    let mut allows_circuits = None;
    let mut pattern_sides = Vec::new();
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
            Some("1302") if in_airport => match f.next() {
                Some("transition_level") => {
                    transition_level_ft = f.next().and_then(parse_transition_level);
                }
                Some("allows_circuits") => {
                    allows_circuits = f.next().and_then(parse_allows_circuits);
                }
                _ => {}
            },
            Some("1101") if in_airport => note_pattern_side(&mut pattern_sides, f),
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
        transition_level_ft,
        allows_circuits,
        pattern_sides,
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
            surface: f[1].parse().unwrap_or(0),
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
                            transition_level_ft: None,
                            allows_circuits: None,
                            pattern_sides: Vec::new(),
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
            Some("1302") => {
                if let Some((_, apt)) = cur.as_mut() {
                    match fields.next() {
                        Some("transition_level") => {
                            apt.transition_level_ft =
                                fields.next().and_then(parse_transition_level);
                        }
                        Some("allows_circuits") => {
                            apt.allows_circuits = fields.next().and_then(parse_allows_circuits);
                        }
                        _ => {}
                    }
                }
            }
            Some("1101") => {
                if let Some((_, apt)) = cur.as_mut() {
                    note_pattern_side(&mut apt.pattern_sides, fields);
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
    /// The PAVEMENT: every apt.dat 110 polygon at the field, each as its
    /// rings (the first the outline, the rest holes), beziers sampled. Empty
    /// where the field draws none. A tug's pattern is the sim's geometry,
    /// not ours, and at LIPK it ran 98 m straight and left a 172 on the
    /// grass, where the taxi could not move it (2026-09-26): the push stops
    /// at the pavement's edge ([`TaxiNet::on_pavement`]).
    pub pavement: Vec<Vec<Vec<(f64, f64)>>>,
    /// The stands' LEAD-IN lines (apt.dat line type 60, the painted curve
    /// from a taxiway into a stand), beziers sampled. Read backwards from the
    /// stand it is the way a push should go ([`TaxiNet::push_lane`]); the
    /// push used to aim at the route's first node instead, and swung a 737's
    /// tail into the buildings beside LIBP's stand 1 (2026-09-26). Type 60
    /// ends at 650 stands across fourteen fields surveyed that day.
    pub stand_lines: Vec<Vec<(f64, f64)>>,
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

/// A stand is entered along its parking line from this far back.
const STAND_ALIGN_M: f64 = 25.0;

/// ...when the turn at that point is no more than this.
const STAND_ALIGN_MAX_TURN_DEG: f64 = 90.0;

/// ...unless the straight lead-in already arrives within this of the stand's
/// heading.
const STAND_ALIGN_OK_DEG: f64 = 30.0;

/// The lead-in's points from the route's last node `end` to the stand: the
/// straight line when it already arrives on the stand's heading, else via a
/// point [`STAND_ALIGN_M`] back along the parking line.
fn stand_lead_in(end: (f64, f64), stand: &Stand, m_per_deg_lon: f64) -> Vec<(f64, f64)> {
    let spot = (stand.lat, stand.lon);
    if !stand.heading.is_finite() || m_per_deg_lon <= 0.0 {
        return vec![end, spot];
    }
    let straight = bearing_deg(end.0, end.1, spot.0, spot.1);
    if sim::angles::abs_diff_deg(straight, stand.heading) <= STAND_ALIGN_OK_DEG {
        return vec![end, spot];
    }
    let h = stand.heading.to_radians();
    let back = (
        spot.0 - STAND_ALIGN_M * h.cos() / 111_320.0,
        spot.1 - STAND_ALIGN_M * h.sin() / m_per_deg_lon,
    );
    // Only a turn the aircraft can make: the leg to the back point must not
    // double back on the stand line. LOWI (2026-09-24) put the back point
    // BEHIND the route's end, the drive needed a 140-170 degree reversal at
    // walking pace, circled, and stopped "off the cleared route" 25 m out.
    let to_back = bearing_deg(end.0, end.1, back.0, back.1);
    if sim::angles::abs_diff_deg(to_back, stand.heading) > STAND_ALIGN_MAX_TURN_DEG {
        return vec![end, spot];
    }
    vec![end, back, spot]
}

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
/// rollout, 2026-09-17). But a turn-off a little past abeam is an ordinary
/// turn at taxi pace: EDLP's B leaves 24 at 108 degrees, 406 m ahead of an
/// SF50 rolling at 9 kt, and at 100 it was refused for C, 970 m further
/// down; the vacate drove 1,217 m at turn pace, ran out of time on the
/// runway, and the taxi-in began with a backtrack (2026-09-24).
const VACATE_TURN_MAX_DEG: f64 = 120.0;

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

/// A stand route that begins ON the runway: the drive off it, forward, then
/// the route to the stand from where that drive ends.
///
/// Planned from the runway itself, a stand route snaps to the nearest node
/// whatever the heading, and that node is as likely behind as ahead: EDLP 24
/// (SF50, 2026-09-24) the vacate timed out 714 m short of exit C, the stand
/// route was planned from 51.61388 N 8.61568 E heading 236, and it started
/// back up the runway (the drive joined 179 degrees off) with a clearance
/// that said "cross runway zero six" for the strip we were already on. The
/// vacate knows the way off ([`TaxiNet::plan_vacate`]: exits ahead only,
/// toward the apron); the stand route starts where it ends.
///
/// `rest` is the stand route from the vacate's end, its crossings already
/// marked: the vacate's own runway legs are the landing clearance's, never
/// a crossing. The vacate's exit leads the `via`.
pub fn after_vacate(vacate: VacatePlan, rest: TaxiRoutePlan) -> TaxiRoutePlan {
    let mut path = vacate.path;
    let mut seg = vacate.seg;
    let mut tail_path = rest.path.into_iter();
    // The stand route starts where the vacate ends (its first point is the
    // position it was planned from): the duplicate goes, and the stand
    // route's first leg then runs from the vacate's end. Apart, a connecting
    // leg joins them.
    match (path.last(), tail_path.as_slice().first()) {
        (Some(&end), Some(first)) => {
            let m_per_deg_lon = 111_320.0 * end.0.to_radians().cos();
            let gap = ((first.1 - end.1) * m_per_deg_lon).hypot((first.0 - end.0) * 111_320.0);
            if gap < 1.0 {
                tail_path.next();
            } else {
                seg.push((String::new(), false));
            }
        }
        // No vacate to lead with: the stand route whole.
        (None, _) => seg.clear(),
        (Some(_), None) => {}
    }
    path.extend(tail_path);
    seg.extend(rest.seg);
    let mut via = Vec::with_capacity(rest.via.len() + 1);
    if !vacate.exit_name.is_empty() && rest.via.first() != Some(&vacate.exit_name) {
        via.push(vacate.exit_name);
    }
    via.extend(rest.via);
    TaxiRoutePlan {
        via,
        crossings: rest.crossings,
        holding_point: rest.holding_point,
        path,
        seg,
        lead_on: Vec::new(),
    }
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
/// The stand lead-in line type (see `TaxiNet::stand_lines`).
const LEADIN_TYPE: &str = "60";

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
/// How near the aircraft a painted line must pass to be its stand's lane.
const LANE_END_M: f64 = 20.0;
/// How far the line's direction may differ from the heading.
const LANE_ALIGN_DEG: f64 = 35.0;
/// The shortest painted lane worth pushing along.
const LANE_MIN_M: f64 = 15.0;

/// Even-odd point in a closed ring of (lat, lon).
fn ring_contains(ring: &[(f64, f64)], lat: f64, lon: f64) -> bool {
    let mut inside = false;
    let n = ring.len();
    for i in 0..n {
        let (y1, x1) = ring[i];
        let (y2, x2) = ring[(i + 1) % n];
        if (y1 > lat) != (y2 > lat) && lon < x1 + (lat - y1) * (x2 - x1) / (y2 - y1) {
            inside = !inside;
        }
    }
    inside
}

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
/// running on bearing `brg`: its index in `paint`, the foot of the
/// perpendicular, and how far along the line (m from its first point) the
/// foot sits. `prefer` is the line the previous sample snapped to, which
/// wins whenever it is still a candidate — without that, two lines drawn a
/// metre apart (a taxiway and the stand lead-in leaving it) swap the route
/// between them sample by sample and saw-tooth a straight leg.
fn nearest_paint(
    paint: &[Vec<(f64, f64)>],
    p: (f64, f64),
    brg: f64,
    prefer: Option<usize>,
    m_per_deg_lon: f64,
) -> Option<(usize, (f64, f64), f64)> {
    let mut best: Option<(f64, usize, (f64, f64), f64)> = None;
    let mut best_prefer: Option<(f64, usize, (f64, f64), f64)> = None;
    for (idx, line) in paint.iter().enumerate() {
        let mut along = 0.0;
        for w in line.windows(2) {
            let (ax, ay) = ((w[0].1 - p.1) * m_per_deg_lon, (w[0].0 - p.0) * 111_320.0);
            let (bx, by) = ((w[1].1 - p.1) * m_per_deg_lon, (w[1].0 - p.0) * 111_320.0);
            let (dx, dy) = (bx - ax, by - ay);
            let len2 = dx * dx + dy * dy;
            let seg_start = along;
            along += len2.sqrt();
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
            let at = seg_start + t * len2.sqrt();
            if best.is_none_or(|(bd, ..)| d < bd) {
                best = Some((d, idx, foot, at));
            }
            if prefer == Some(idx) && best_prefer.is_none_or(|(bd, ..)| d < bd) {
                best_prefer = Some((d, idx, foot, at));
            }
        }
    }
    best_prefer.or(best).map(|(_, idx, foot, at)| (idx, foot, at))
}

/// How much longer than the straight gap a drawn line may run between two
/// samples snapped onto it and still be the way the route goes there. A
/// quarter circle is 1.11 times its chord; a line that wanders further is
/// going somewhere else.
const PAINT_FILL_DETOUR: f64 = 1.5;

/// The piece of drawn line `line` between the feet at `from_m` and `to_m`
/// metres along it, EXCLUSIVE of both ends, in the route's direction (the
/// line may be drawn either way), resampled so no gap exceeds
/// [`PAINT_SAMPLE_M`]. Returns the points and the piece's length.
fn paint_between(
    line: &[(f64, f64)],
    from_m: f64,
    to_m: f64,
    m_per_deg_lon: f64,
) -> (Vec<(f64, f64)>, f64) {
    let dist = |a: (f64, f64), b: (f64, f64)| {
        let (dx, dy) = ((a.1 - b.1) * m_per_deg_lon, (a.0 - b.0) * 111_320.0);
        (dx * dx + dy * dy).sqrt()
    };
    // The line's vertices with how far along it each sits.
    let mut at = Vec::with_capacity(line.len());
    let mut acc = 0.0;
    for (i, &p) in line.iter().enumerate() {
        if i > 0 {
            acc += dist(line[i - 1], p);
        }
        at.push(acc);
    }
    let point_at = |m: f64| -> (f64, f64) {
        let k = at.partition_point(|&a| a <= m).clamp(1, line.len() - 1);
        let (a, b) = (line[k - 1], line[k]);
        let span = at[k] - at[k - 1];
        let t = if span > 1e-9 { ((m - at[k - 1]) / span).clamp(0.0, 1.0) } else { 0.0 };
        (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t)
    };
    let (lo, hi) = (from_m.min(to_m), from_m.max(to_m));
    // Stations along the piece: every vertex strictly inside, plus fill-ins
    // so a long straight stays sampled like any other leg.
    let mut stations: Vec<f64> = Vec::new();
    let mut prev = lo;
    for &a in at.iter().filter(|&&a| a > lo + 0.25 && a < hi - 0.25) {
        let n = ((a - prev) / PAINT_SAMPLE_M).ceil() as usize;
        for k in 1..n {
            stations.push(prev + (a - prev) * k as f64 / n as f64);
        }
        stations.push(a);
        prev = a;
    }
    let n = ((hi - prev) / PAINT_SAMPLE_M).ceil() as usize;
    for k in 1..n {
        stations.push(prev + (hi - prev) * k as f64 / n as f64);
    }
    if to_m < from_m {
        stations.reverse();
    }
    (stations.into_iter().map(point_at).collect(), hi - lo)
}

/// A route's first edge is joined AHEAD of the aircraft's projection onto
/// it, never at the snapped node behind it. The planner snaps the start to
/// the nearest node, and when the aircraft sits abeam the middle of an edge
/// that node is behind it along the way the route goes: the route then runs
/// back to the node and hairpins onto the edge. LILB ramp, Lancair Evolution
/// (1790712974, 1790720549, 1790721973; 2026-09-29 and 30): parked at
/// 45.771275 9.164289 facing 28.9, the nearest node 16.7 m away on a bearing
/// of 98, the edge beyond it running 335: a 124 degree reversal 17 m from
/// brake release, flown at 8 to 9 kt with the projection jumping onto the
/// second leg, and every taxi failed 14.2 to 16.3 m off the line. Now the
/// route joins the edge past the projection by the offset itself (bounded
/// by [`JOIN_AHEAD_LEAD_M`] and [`JOIN_AHEAD_MAX_LEAD_M`]: a 45 degree
/// join), and an edge wholly behind the projection is dropped. Only the network's first taxiway leg, only when
/// the aircraft is within [`JOIN_AHEAD_MAX_LAT_M`] of it. Pure.
pub fn join_first_edge_ahead(
    path: &mut Vec<(f64, f64)>,
    seg: &mut Vec<(String, bool)>,
    here: (f64, f64),
) {
    let m_per_deg_lon = 111_320.0 * here.0.to_radians().cos();
    let to_m = |p: (f64, f64)| ((p.1 - here.1) * m_per_deg_lon, (p.0 - here.0) * 111_320.0);
    while path.len() >= 2 && seg.len() + 1 == path.len() {
        if seg[0].1 {
            return; // a runway leg is the runway's geometry
        }
        let (a, b) = (to_m(path[0]), to_m(path[1]));
        let (ex, ey) = (b.0 - a.0, b.1 - a.1);
        let len = (ex * ex + ey * ey).sqrt();
        if len < 0.5 {
            return;
        }
        // The aircraft is the origin: its projection along a->b, and its
        // distance off the edge's line.
        let t = (-a.0 * ex - a.1 * ey) / len;
        let off = (a.0 * ey - a.1 * ex).abs() / len;
        if off > JOIN_AHEAD_MAX_LAT_M || t < JOIN_AHEAD_MIN_M {
            return;
        }
        let join = t + off.clamp(JOIN_AHEAD_LEAD_M, JOIN_AHEAD_MAX_LEAD_M);
        if join >= len - JOIN_AHEAD_MIN_M {
            // The whole edge is behind the join: its far node is the join
            // (a route never shorter than two points), then look again.
            if path.len() <= 2 {
                return;
            }
            path.remove(0);
            seg.remove(0);
            continue;
        }
        let k = join / len;
        let (p0, p1) = (path[0], path[1]);
        path[0] = (p0.0 + (p1.0 - p0.0) * k, p0.1 + (p1.1 - p0.1) * k);
        return;
    }
}
/// The aircraft's projection must be this far past the edge's start before
/// the node counts as behind it, and a join this close to the edge's end is
/// the end node.
const JOIN_AHEAD_MIN_M: f64 = 3.0;
/// The join sits this far past the projection at least, and at most
/// [`JOIN_AHEAD_MAX_LEAD_M`]: the offset itself between, a 45 degree join.
const JOIN_AHEAD_LEAD_M: f64 = 5.0;
const JOIN_AHEAD_MAX_LEAD_M: f64 = 30.0;
/// Further off the first edge than this, the start is not beside it: the
/// planner's node stands.
const JOIN_AHEAD_MAX_LAT_M: f64 = 60.0;

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
///
/// The one place a point moves further: a network corner cut across a drawn
/// fillet turns on that fillet ([`ride_corner_fillets`], #194).
pub fn ride_paint(
    paint: &[Vec<(f64, f64)>],
    path: &[(f64, f64)],
    seg: &[(String, bool)],
) -> RoutePolyline {
    ride_paint_with(paint, path, seg, true)
}

/// [`ride_paint`], with the corners rounded on their fillets or not (the
/// tests compare the two).
fn ride_paint_with(
    paint: &[Vec<(f64, f64)>],
    path: &[(f64, f64)],
    seg: &[(String, bool)],
    fillets: bool,
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
    // Which line each output point sits on, and where along it.
    let mut out_snap: Vec<Option<(usize, f64)>> = Vec::with_capacity(path.len() * 4);
    // Which network leg each output point was sampled from (the end point
    // is the last node's), for the corners (`ride_corner_fillets`).
    let mut out_leg: Vec<usize> = Vec::with_capacity(path.len() * 4);
    let mut prefer: Option<usize> = None;
    for i in 0..path.len() - 1 {
        let (a, b) = (path[i], path[i + 1]);
        let s = seg.get(i).cloned().unwrap_or_default();
        let len = dist_m(a, b);
        // A zero-length leg is no leg: kept, it reads as a bearing of 0,
        // due north, to anything that asks a segment its direction (the
        // LIPR stop swerved on one, 2026-09-26).
        if len < 0.5 {
            continue;
        }
        if s.1 {
            out_path.push(a);
            out_seg.push(s);
            out_snap.push(None);
            out_leg.push(i);
            prefer = None;
            continue;
        }
        let brg = bearing_deg(a.0, a.1, b.0, b.1);
        let n = ((len / PAINT_SAMPLE_M).round() as usize).max(1);
        for k in 0..n {
            let t = k as f64 / n as f64;
            let raw = (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t);
            let hit = nearest_paint(paint, raw, brg, prefer, m_per_deg_lon);
            let snapped = match hit {
                Some((idx, q, _)) => {
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
            out_snap.push(hit.filter(|_| keeps_going).map(|(idx, _, at)| (idx, at)));
            out_leg.push(i);
        }
    }
    // The end point rides the last leg's bearing, so a route that finishes at
    // a holding point finishes ON the line, which is where it will stop.
    let last = path[path.len() - 1];
    let prev = path[path.len() - 2];
    let (end, end_snap) = if seg.last().is_some_and(|s| s.1) {
        (last, None)
    } else {
        let brg = bearing_deg(prev.0, prev.1, last.0, last.1);
        nearest_paint(paint, last, brg, prefer, m_per_deg_lon)
            .map_or((last, None), |(idx, q, at)| (q, Some((idx, at))))
    };
    // The end snapped onto the last sample is the same point twice: merge.
    if out_path.len() >= 2 && out_path.last().is_some_and(|&p| dist_m(p, end) < 0.5) {
        out_path.pop();
        out_seg.pop();
        out_snap.pop();
        out_leg.pop();
    }
    out_path.push(end);
    out_snap.push(end_snap);
    out_leg.push(path.len() - 1);
    if fillets {
        ride_corner_fillets(
            paint,
            path,
            seg,
            &mut out_path,
            &mut out_seg,
            &mut out_snap,
            &mut out_leg,
            m_per_deg_lon,
        );
    }
    fill_paint_gaps(paint, &mut out_path, &mut out_seg, &out_snap, m_per_deg_lon);
    (out_path, out_seg)
}

/// A network corner turning less than this is left to the sideways snap.
const FILLET_MIN_TURN_DEG: f64 = 30.0;
/// How far from the corner's node a fillet's ends are looked for, m.
const FILLET_REACH_M: f64 = 80.0;
/// Two drawn lines meet where an end of one is this close to the other, m.
const PAINT_JOIN_M: f64 = 1.0;
/// The most drawn lines one fillet is pieced from.
const FILLET_MAX_LINES: usize = 4;
/// A fillet turns one way: a vertex turning back against the corner by more
/// than this is a different line, degrees.
const FILLET_WOBBLE_DEG: f64 = 10.0;
/// A fillet leaves along the leg in and arrives along the leg out, within
/// this, degrees.
const FILLET_END_DEG: f64 = 45.0;

/// A drawn line's vertices with how far along it each sits, m.
fn line_stations(line: &[(f64, f64)], m_per_deg_lon: f64) -> Vec<f64> {
    let mut at = Vec::with_capacity(line.len());
    let mut acc = 0.0;
    for (i, &p) in line.iter().enumerate() {
        if i > 0 {
            let (dx, dy) = ((p.1 - line[i - 1].1) * m_per_deg_lon, (p.0 - line[i - 1].0) * 111_320.0);
            acc += (dx * dx + dy * dy).sqrt();
        }
        at.push(acc);
    }
    at
}

/// The point `m` metres along a drawn line.
fn line_point(line: &[(f64, f64)], at: &[f64], m: f64) -> (f64, f64) {
    if line.len() < 2 {
        return line[0];
    }
    let k = at.partition_point(|&a| a <= m).clamp(1, line.len() - 1);
    let (a, b) = (line[k - 1], line[k]);
    let span = at[k] - at[k - 1];
    let t = if span > 1e-9 { ((m - at[k - 1]) / span).clamp(0.0, 1.0) } else { 0.0 };
    (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t)
}

/// Round a route's corners on the drawn fillet the network cut.
///
/// A taxiway meeting another at a T is drawn, in the network, as one node
/// where the two CENTRELINES cross; the paint instead stops short of the
/// junction and sweeps round a fillet onto the other line, one each way.
/// The route out of the planner runs straight to the node and turns there,
/// and the sideways snap (at most [`PAINT_SNAP_M`], within
/// [`PAINT_PARALLEL_DEG`] of the leg) takes each sample near the junction
/// onto whichever fillet leaves closest, left or right, sample by sample.
/// LEMG's Quebec onto Delta (Citation X, sorties 1790734216 and 1790738786,
/// 2026-09-30, #194): Quebec's paint ends 47 m short of Delta's centreline,
/// and the ridden route ran 36, 21, 19, 69, 114, 2, 41 and 318 degrees in
/// seven legs of 5 to 10 m; the drive chased the jog right, swung 4.6 and
/// 4.8 m outside the turn back and failed the taxi both times.
///
/// So at each network corner of [`FILLET_MIN_TURN_DEG`] or more the samples
/// between a sample on the paint before it and a sample on the paint after
/// it are replaced by the drawn lines that JOIN those two, end to end: the
/// shortest chain of at most [`FILLET_MAX_LINES`] lines that turns the
/// corner's way the whole time (never back more than [`FILLET_WOBBLE_DEG`]),
/// leaves and arrives along the legs, lies inside the corner (within
/// [`PAINT_SNAP_M`] of the triangle its two ends make with the node) and is
/// no longer than [`PAINT_FILL_DETOUR`] times the gap. That is the fillet
/// the corner turns on and never the other one, which runs outside the
/// node. No such chain, and the corner is left as the snap made it: runway
/// legs, undrawn fields and junctions drawn square are untouched.
#[allow(clippy::too_many_arguments)]
fn ride_corner_fillets(
    paint: &[Vec<(f64, f64)>],
    path: &[(f64, f64)],
    seg: &[(String, bool)],
    out_path: &mut Vec<(f64, f64)>,
    out_seg: &mut Vec<(String, bool)>,
    out_snap: &mut Vec<Option<(usize, f64)>>,
    out_leg: &mut Vec<usize>,
    m_per_deg_lon: f64,
) {
    if paint.is_empty()
        || path.len() < 3
        || out_path.len() != out_snap.len()
        || out_path.len() != out_leg.len()
        || out_seg.len() + 1 != out_path.len()
    {
        return;
    }
    let dist = |a: (f64, f64), b: (f64, f64)| {
        let (dx, dy) = ((a.1 - b.1) * m_per_deg_lon, (a.0 - b.0) * 111_320.0);
        (dx * dx + dy * dy).sqrt()
    };
    let brg = |a: (f64, f64), b: (f64, f64)| bearing_deg(a.0, a.1, b.0, b.1);
    let stations: Vec<Vec<f64>> = paint.iter().map(|l| line_stations(l, m_per_deg_lon)).collect();
    for k in 1..path.len() - 1 {
        let v = path[k];
        if seg.get(k - 1).is_none_or(|s| s.1) || seg.get(k).is_none_or(|s| s.1) {
            continue;
        }
        if dist(path[k - 1], v) < 0.5 || dist(v, path[k + 1]) < 0.5 {
            continue;
        }
        let (b_in, b_out) = (brg(path[k - 1], v), brg(v, path[k + 1]));
        let turn = sim::angles::signed_diff_deg(b_out, b_in);
        if turn.abs() < FILLET_MIN_TURN_DEG {
            continue;
        }
        let Some(first_out) = out_leg.iter().position(|&l| l >= k) else {
            continue;
        };
        // Exits: samples after the node, on the paint, within reach.
        let exits: Vec<(usize, usize, f64)> = (first_out..out_path.len())
            .take_while(|&j| dist(out_path[j], v) <= FILLET_REACH_M || j == first_out)
            .filter_map(|j| out_snap[j].map(|(l, at)| (j, l, at)))
            .collect();
        if exits.is_empty() {
            continue;
        }
        // Entries: samples on the leg in, on the paint, furthest first, so
        // every sample the snap threw between the two fillets is replaced.
        let entries: Vec<usize> = (0..first_out)
            .rev()
            .take_while(|&i| out_leg[i] + 1 == k && dist(out_path[i], v) <= FILLET_REACH_M)
            .filter(|&i| out_snap[i].is_some())
            .collect();
        type FilletPath = Vec<(f64, f64)>;
        type ChosenFillet = (usize, usize, FilletPath);
        type PaintWalk = (usize, f64, f64, Vec<(usize, f64, f64)>);
        let mut chosen: Option<ChosenFillet> = None;
        for &i in entries.iter().rev() {
            let (li, at_i) = out_snap[i].expect("filtered");
            let fits = |pts: &[(f64, f64)], j: usize| -> Option<f64> {
                let len = fillet_fits(pts, out_path[i], v, out_path[j], b_in, b_out, turn, m_per_deg_lon)?;
                // Spliced in, the fillet must meet the samples either side
                // as smoothly as the route it replaces: no joint sharper
                // than the sharper of [`FILLET_MAX_VERTEX_DEG`] and the old
                // span's own, and none turning back against the corner.
                let lo = i.saturating_sub(1);
                let hi = (j + 1).min(out_path.len() - 1);
                let old_max = vertex_turns(&out_path[lo..=hi], m_per_deg_lon)
                    .iter()
                    .fold(0.0_f64, |a, t| a.max(t.abs()));
                let mut ext: Vec<(f64, f64)> = Vec::with_capacity(pts.len() + 2);
                ext.extend_from_slice(&out_path[lo..i]);
                ext.extend_from_slice(pts);
                ext.extend_from_slice(&out_path[j + 1..=hi]);
                let new = vertex_turns(&ext, m_per_deg_lon);
                let smooth = new.iter().all(|t| {
                    t.abs() <= old_max.max(FILLET_MAX_VERTEX_DEG) && t * turn.signum() >= -FILLET_WOBBLE_DEG
                });
                smooth.then_some(len)
            };
            let mut best: Option<(f64, usize, FilletPath)> = None;
            let dir_i = line_dir(&paint[li], &stations[li], at_i, b_in);
            let mut stack: Vec<PaintWalk> = vec![(li, at_i, dir_i, Vec::new())];
            while let Some((line, at, dir, pieces)) = stack.pop() {
                let len = *stations[line].last().unwrap_or(&0.0);
                let end_at = if dir > 0.0 { len } else { 0.0 };
                let p = line_point(&paint[line], &stations[line], end_at);
                let mut here = pieces.clone();
                here.push((line, at, end_at));
                let used = |l: usize| here.iter().any(|&(u, ..)| u == l);
                // Onto an exit's line: the first exit ahead of where we meet it.
                let mut tried: Vec<usize> = Vec::new();
                for &(j, lj, at_j) in &exits {
                    if used(lj) || tried.contains(&lj) {
                        continue;
                    }
                    let Some(at_e) = project_on_line(&paint[lj], &stations[lj], p, m_per_deg_lon)
                        .filter(|&(_, d)| d <= PAINT_JOIN_M)
                        .map(|(a, _)| a)
                    else {
                        continue;
                    };
                    let dir_o = line_dir(&paint[lj], &stations[lj], at_e, b_out);
                    if (at_j - at_e) * dir_o <= 0.5 {
                        continue;
                    }
                    tried.push(lj);
                    let mut chain = here.clone();
                    chain.push((lj, at_e, at_j));
                    let pts = chain_points(paint, &stations, &chain, m_per_deg_lon);
                    let mut all = vec![out_path[i]];
                    all.extend(pts.iter().copied());
                    all.push(out_path[j]);
                    if let Some(len) = fits(&all, j) {
                        if best.as_ref().is_none_or(|b| len < b.0) {
                            best = Some((len, j, pts));
                        }
                    }
                }
                if here.len() >= FILLET_MAX_LINES {
                    continue;
                }
                for (l2, line2) in paint.iter().enumerate() {
                    if used(l2) || line2.len() < 2 {
                        continue;
                    }
                    let len2 = *stations[l2].last().unwrap_or(&0.0);
                    if dist(p, line2[0]) <= PAINT_JOIN_M {
                        stack.push((l2, 0.0, 1.0, here.clone()));
                    } else if dist(p, line2[line2.len() - 1]) <= PAINT_JOIN_M {
                        stack.push((l2, len2, -1.0, here.clone()));
                    }
                }
            }
            if let Some((_, j, pts)) = best {
                chosen = Some((i, j, pts));
                break;
            }
        }
        let Some((i, j, pts)) = chosen else {
            continue;
        };
        if out_seg[i..j].iter().any(|s| s.1) {
            continue;
        }
        // Each new point takes the name and leg of the replaced sample
        // nearest it, so a name change at the node lands where it was.
        let nearest = |q: (f64, f64)| {
            (i..j)
                .min_by(|&a, &b| dist(out_path[a], q).total_cmp(&dist(out_path[b], q)))
                .unwrap_or(i)
        };
        let new_seg: Vec<(String, bool)> = std::iter::once(out_seg[i].clone())
            .chain(pts.iter().map(|&q| out_seg[nearest(q)].clone()))
            .collect();
        let new_leg: Vec<usize> = pts.iter().map(|&q| out_leg[nearest(q)]).collect();
        let n = pts.len();
        out_path.splice(i + 1..j, pts);
        out_seg.splice(i..j, new_seg);
        out_snap.splice(i + 1..j, std::iter::repeat_n(None, n));
        out_leg.splice(i + 1..j, new_leg);
    }
}

/// +1 when a drawn line runs the way `brg` goes at `at` metres along it,
/// else -1 (a line may be drawn from either end).
fn line_dir(line: &[(f64, f64)], at: &[f64], m: f64, brg: f64) -> f64 {
    let len = *at.last().unwrap_or(&0.0);
    let (a, b) = (
        line_point(line, at, (m - 1.0).max(0.0)),
        line_point(line, at, (m + 1.0).min(len)),
    );
    let tangent = bearing_deg(a.0, a.1, b.0, b.1);
    if sim::angles::abs_diff_deg(tangent, brg) <= 90.0 {
        1.0
    } else {
        -1.0
    }
}

/// The foot of `p` on a drawn line: (metres along it, distance off it).
fn project_on_line(line: &[(f64, f64)], at: &[f64], p: (f64, f64), m_per_deg_lon: f64) -> Option<(f64, f64)> {
    let mut best: Option<(f64, f64)> = None;
    for (k, w) in line.windows(2).enumerate() {
        let (ax, ay) = ((w[0].1 - p.1) * m_per_deg_lon, (w[0].0 - p.0) * 111_320.0);
        let (bx, by) = ((w[1].1 - p.1) * m_per_deg_lon, (w[1].0 - p.0) * 111_320.0);
        let (dx, dy) = (bx - ax, by - ay);
        let len2 = dx * dx + dy * dy;
        let t = if len2 < 1e-9 { 0.0 } else { (-(ax * dx + ay * dy) / len2).clamp(0.0, 1.0) };
        let (fx, fy) = (ax + t * dx, ay + t * dy);
        let d = (fx * fx + fy * fy).sqrt();
        if best.is_none_or(|(_, bd)| d < bd) {
            best = Some((at[k] + t * len2.sqrt(), d));
        }
    }
    best
}

/// The points of a chain of line pieces `(line, from_m, to_m)`, exclusive
/// of the chain's own two ends, each joint included once.
fn chain_points(
    paint: &[Vec<(f64, f64)>],
    stations: &[Vec<f64>],
    chain: &[(usize, f64, f64)],
    m_per_deg_lon: f64,
) -> Vec<(f64, f64)> {
    let mut pts = Vec::new();
    for (n, &(line, from, to)) in chain.iter().enumerate() {
        if (to - from).abs() > 0.25 {
            pts.extend(paint_between(&paint[line], from, to, m_per_deg_lon).0);
        }
        if n + 1 < chain.len() && (to - from).abs() > 0.25 {
            pts.push(line_point(&paint[line], &stations[line], to));
        }
    }
    pts
}

/// Whether the polyline `pts` (entry first, exit last) is the fillet of the
/// corner at `v` turning `turn` degrees from `b_in` to `b_out`: its length
/// if so. See [`ride_corner_fillets`].
#[allow(clippy::too_many_arguments)]
fn fillet_fits(
    pts: &[(f64, f64)],
    entry: (f64, f64),
    v: (f64, f64),
    exit: (f64, f64),
    b_in: f64,
    b_out: f64,
    turn: f64,
    m_per_deg_lon: f64,
) -> Option<f64> {
    let xy = |p: (f64, f64)| ((p.1 - v.1) * m_per_deg_lon, (p.0 - v.0) * 111_320.0);
    let d = |a: (f64, f64), b: (f64, f64)| {
        let (a, b) = (xy(a), xy(b));
        ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt()
    };
    let mut len = 0.0;
    let mut brgs: Vec<f64> = Vec::new();
    let mut last = pts[0];
    for &q in &pts[1..] {
        let step = d(last, q);
        len += step;
        if step >= 0.5 {
            brgs.push(bearing_deg(last.0, last.1, q.0, q.1));
            last = q;
        }
    }
    let (first, final_) = (*brgs.first()?, *brgs.last()?);
    if len > PAINT_FILL_DETOUR * d(entry, exit) + 5.0 {
        return None;
    }
    if sim::angles::abs_diff_deg(first, b_in) > FILLET_END_DEG || sim::angles::abs_diff_deg(final_, b_out) > FILLET_END_DEG {
        return None;
    }
    let side = turn.signum();
    if brgs.windows(2).any(|w| sim::angles::signed_diff_deg(w[1], w[0]) * side < -FILLET_WOBBLE_DEG) {
        return None;
    }
    let tri = [xy(entry), (0.0, 0.0), xy(exit)];
    if pts.iter().any(|&q| off_triangle_m(xy(q), tri) > PAINT_SNAP_M) {
        return None;
    }
    Some(len)
}

/// The most one vertex of a spliced fillet may turn, unless the route it
/// replaces already turned more, degrees. A drawn fillet sampled every
/// [`PAINT_SAMPLE_M`] turns a few degrees per vertex.
const FILLET_MAX_VERTEX_DEG: f64 = 30.0;

/// The turn at each vertex of a polyline, degrees signed right, over steps
/// of half a metre or more.
fn vertex_turns(pts: &[(f64, f64)], m_per_deg_lon: f64) -> Vec<f64> {
    let mut out = Vec::new();
    let mut prev: Option<f64> = None;
    let Some(&first) = pts.first() else {
        return out;
    };
    let mut last = first;
    for &q in &pts[1..] {
        let (dx, dy) = ((q.1 - last.1) * m_per_deg_lon, (q.0 - last.0) * 111_320.0);
        if (dx * dx + dy * dy).sqrt() < 0.5 {
            continue;
        }
        let b = bearing_deg(last.0, last.1, q.0, q.1);
        if let Some(p) = prev {
            out.push(sim::angles::signed_diff_deg(b, p));
        }
        prev = Some(b);
        last = q;
    }
    out
}

/// How far `p` lies outside the triangle `t` (0 inside), in its own units.
fn off_triangle_m(p: (f64, f64), t: [(f64, f64); 3]) -> f64 {
    let cross = |a: (f64, f64), b: (f64, f64), c: (f64, f64)| (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
    let (c1, c2, c3) = (cross(t[0], t[1], p), cross(t[1], t[2], p), cross(t[2], t[0], p));
    if (c1 >= 0.0 && c2 >= 0.0 && c3 >= 0.0) || (c1 <= 0.0 && c2 <= 0.0 && c3 <= 0.0) {
        return 0.0;
    }
    let to_seg = |a: (f64, f64), b: (f64, f64)| {
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let len2 = dx * dx + dy * dy;
        let t = if len2 < 1e-9 { 0.0 } else { (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len2).clamp(0.0, 1.0) };
        ((a.0 + t * dx - p.0).powi(2) + (a.1 + t * dy - p.1).powi(2)).sqrt()
    };
    to_seg(t[0], t[1]).min(to_seg(t[1], t[2])).min(to_seg(t[2], t[0]))
}

/// Close the gaps in a ridden route where the drawn line bulged out of reach.
///
/// A network leg is a CHORD of the drawn curve it approximates, and a curve
/// that sweeps round a junction runs further than [`PAINT_SNAP_M`] from its
/// own chord in the middle: the samples there cannot be moved and stay on
/// the chord while their neighbours sit on the line. The route then zigzags
/// between the two, a vertex of 40 to 60 degrees every five metres. LEMG 13's
/// lead-on did it (2026-09-26, sortie 1790441293): 6 m off the drawn curve in
/// the middle, the line-up's projection jumped 6 and 11 m forward in single
/// ticks and the heading error swung to -43 degrees.
///
/// So a run of unmoved samples between two samples on the SAME line is
/// replaced by that line between their feet, when it is the way the route
/// goes there: no runway leg inside (a runway is never ridden), not more than
/// [`PAINT_FILL_DETOUR`] times the straight gap, and never more than twice
/// the snap distance from the samples it replaces (a line that leaves and
/// comes back is a different line).
fn fill_paint_gaps(
    paint: &[Vec<(f64, f64)>],
    path: &mut Vec<(f64, f64)>,
    seg: &mut Vec<(String, bool)>,
    snap: &[Option<(usize, f64)>],
    m_per_deg_lon: f64,
) {
    let dist = |a: (f64, f64), b: (f64, f64)| {
        let (dx, dy) = ((a.1 - b.1) * m_per_deg_lon, (a.0 - b.0) * 111_320.0);
        (dx * dx + dy * dy).sqrt()
    };
    if path.len() != snap.len() || seg.len() + 1 != path.len() {
        return;
    }
    let mut out_path: Vec<(f64, f64)> = Vec::with_capacity(path.len());
    let mut out_seg: Vec<(String, bool)> = Vec::with_capacity(seg.len());
    let mut i = 0;
    while i < path.len() {
        out_path.push(path[i]);
        let next = (i + 1..path.len()).find(|&j| snap[j].is_some());
        let fill = match (snap[i], next) {
            (Some((li, from_m)), Some(j)) if j > i + 1 => match snap[j] {
                Some((lj, to_m)) if lj == li && (to_m - from_m).abs() > 0.5 => {
                    let (piece, len) = paint_between(&paint[li], from_m, to_m, m_per_deg_lon);
                    let straight = dist(path[i], path[j]);
                    let runway = seg[i..j].iter().any(|s| s.1);
                    let near = path[i + 1..j].iter().all(|&p| {
                        piece
                            .iter()
                            .chain([&path[i], &path[j]])
                            .any(|&q| dist(p, q) <= 2.0 * PAINT_SNAP_M)
                    });
                    (!runway && near && len <= PAINT_FILL_DETOUR * straight + 5.0)
                        .then_some((j, piece))
                }
                _ => None,
            },
            _ => None,
        };
        match fill {
            Some((j, piece)) => {
                // Each new leg keeps the name of the replaced sample nearest
                // it, so a name change at a network node inside the gap
                // lands where the node was.
                let name_near = |q: (f64, f64)| {
                    (i..j)
                        .min_by(|&a, &b| dist(path[a], q).total_cmp(&dist(path[b], q)))
                        .map_or_else(|| seg[i].clone(), |k| seg[k].clone())
                };
                out_seg.push(seg[i].clone());
                for &q in &piece {
                    out_path.push(q);
                    out_seg.push(name_near(q));
                }
                i = j;
            }
            None => {
                if let Some(s) = seg.get(i) {
                    out_seg.push(s.clone());
                }
                i += 1;
            }
        }
    }
    *path = out_path;
    *seg = out_seg;
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
    // The same 120 feature, flagged by LEAD-IN segments (type 60).
    let mut leadin: Vec<PaintNode> = Vec::new();
    // The pavement polygon being read (a 110 block), and its current ring.
    let mut in_pavement = false;
    let mut ring: Vec<PaintNode> = Vec::new();
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
            flush_paint_feature(&mut leadin, &mut net.stand_lines);
            in_feature = in_airport && row == Some("120");
            ring.clear();
            in_pavement = in_airport && row == Some("110");
            if in_pavement {
                net.pavement.push(Vec::new());
            }
        }
        match row {
            Some("111" | "112" | "113" | "114" | "115" | "116") if in_pavement => {
                let (Some(lat), Some(lon)) = (
                    f.next().and_then(|s| s.parse::<f64>().ok()),
                    f.next().and_then(|s| s.parse::<f64>().ok()),
                ) else {
                    continue;
                };
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
                ring.push(PaintNode {
                    p: (lat, lon),
                    ctrl,
                    centre: true,
                });
                // 113/114 close a ring; the next ring of the same polygon
                // is a hole.
                if matches!(row, Some("113" | "114")) {
                    if let Some(&first) = ring.first() {
                        ring.push(first);
                    }
                    let mut sampled: Vec<Vec<(f64, f64)>> = Vec::new();
                    flush_paint_feature(&mut ring, &mut sampled);
                    if let (Some(r), Some(poly)) = (sampled.into_iter().next(), net.pavement.last_mut()) {
                        poly.push(r);
                    }
                    ring.clear();
                }
            }
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
                let attrs: Vec<&str> = f.collect();
                let centre = attrs.iter().any(|a| PAINT_CENTRELINE_TYPES.contains(a));
                let lead = attrs.contains(&LEADIN_TYPE);
                feature.push(PaintNode {
                    p: (lat, lon),
                    ctrl,
                    centre,
                });
                leadin.push(PaintNode {
                    p: (lat, lon),
                    ctrl,
                    centre: lead,
                });
                // 113/114 close a loop: the drawn line returns to its start.
                if matches!(row, Some("113" | "114")) {
                    if let Some(&first) = feature.first() {
                        feature.push(first);
                    }
                    if let Some(&first) = leadin.first() {
                        leadin.push(first);
                    }
                    flush_paint_feature(&mut feature, &mut net.paint);
                    flush_paint_feature(&mut leadin, &mut net.stand_lines);
                    in_feature = false;
                } else if matches!(row, Some("115" | "116")) {
                    flush_paint_feature(&mut feature, &mut net.paint);
                    flush_paint_feature(&mut leadin, &mut net.stand_lines);
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
    net.stand_lines = stitch_lines(std::mem::take(&mut net.stand_lines), 1.0);
    Ok((!net.edges.is_empty()).then_some(net))
}

/// Join polylines whose ends meet (within `tol_m`) into continuous lines.
/// A lead-in is often drawn as several short pieces (LIBP stand 1: a 2-point
/// straight into the stand, then a 9-point curve, end to end), and a piece
/// shorter than the direction probe says nothing about where it goes.
pub fn stitch_lines(mut lines: Vec<Vec<(f64, f64)>>, tol_m: f64) -> Vec<Vec<(f64, f64)>> {
    let near = |a: (f64, f64), b: (f64, f64)| {
        let k = 111_320.0 * a.0.to_radians().cos();
        (((a.1 - b.1) * k).powi(2) + ((a.0 - b.0) * 111_320.0).powi(2)).sqrt() <= tol_m
    };
    let mut out: Vec<Vec<(f64, f64)>> = Vec::new();
    while let Some(mut cur) = lines.pop() {
        loop {
            let (head, tail) = (cur[0], cur[cur.len() - 1]);
            let Some(i) = lines.iter().position(|l| {
                let (a, b) = (l[0], l[l.len() - 1]);
                near(tail, a) || near(tail, b) || near(head, a) || near(head, b)
            }) else {
                break;
            };
            let mut l = lines.swap_remove(i);
            let (a, b) = (l[0], l[l.len() - 1]);
            if near(tail, a) {
                cur.extend(l.drain(1..));
            } else if near(tail, b) {
                l.reverse();
                cur.extend(l.drain(1..));
            } else if near(head, b) {
                l.extend(cur.drain(1..));
                cur = l;
            } else {
                l.reverse();
                l.extend(cur.drain(1..));
                cur = l;
            }
        }
        out.push(cur);
    }
    out
}

impl TaxiNet {
    /// The way a push from this spot should go: the stand's painted lead-in
    /// read BACKWARDS, from its end at the aircraft out to the taxiway. A
    /// lead-in (or, lacking one, a painted centreline) counts when one of its
    /// ends is within [`LANE_END_M`] of the aircraft and runs INTO that end
    /// along the aircraft's heading (a nose-in stand's line points at the
    /// terminal, as the nose does). `None` where nothing is drawn: the push
    /// then plans off the cleared route as before.
    pub fn push_lane(&self, lat: f64, lon: f64, psi_deg: f64) -> Option<Vec<(f64, f64)>> {
        let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
        let xy = |p: (f64, f64)| ((p.1 - lon) * m_per_deg_lon, (p.0 - lat) * 111_320.0);
        let diff = |a: f64, b: f64| sim::angles::abs_diff_deg(a, b);
        let mut best: Option<(f64, Vec<(f64, f64)>)> = None;
        for line in self.stand_lines.iter().chain(self.paint.iter()) {
            let lead = self.stand_lines.contains(line);
            for i in 0..line.len().saturating_sub(1) {
                let (a, b) = (xy(line[i]), xy(line[i + 1]));
                let (dx, dy) = (b.0 - a.0, b.1 - a.1);
                let len2 = dx * dx + dy * dy;
                if len2 < 0.25 {
                    continue;
                }
                let t = (((-a.0) * dx + (-a.1) * dy) / len2).clamp(0.0, 1.0);
                let (px, py) = (a.0 + t * dx, a.1 + t * dy);
                let d = (px * px + py * py).sqrt();
                if d > LANE_END_M {
                    continue;
                }
                let brg = dx.atan2(dy).to_degrees().rem_euclid(360.0);
                // Along the nose: the way out is back down the line; against
                // it: forward down the line.
                let back = if diff(brg, psi_deg) <= LANE_ALIGN_DEG {
                    true
                } else if diff(brg + 180.0, psi_deg) <= LANE_ALIGN_DEG {
                    false
                } else {
                    continue;
                };
                let p = (lat + py / 111_320.0, lon + px / m_per_deg_lon);
                let mut lane = vec![p];
                if back {
                    lane.extend(line[..=i].iter().rev().copied());
                } else {
                    lane.extend(line[i + 1..].iter().copied());
                }
                // A line drawn on past the stop point toward the terminal is
                // not a way out: only a lane long enough to push along.
                let length: f64 = lane
                    .windows(2)
                    .map(|w| haversine_nm(w[0].0, w[0].1, w[1].0, w[1].1) * 1852.0)
                    .sum();
                if length < LANE_MIN_M {
                    continue;
                }
                let score = d + if lead { 0.0 } else { 5.0 };
                if best.as_ref().is_none_or(|(s, _)| score < *s) {
                    best = Some((score, lane));
                }
            }
        }
        best.map(|(_, lane)| lane)
    }


    /// Is this point on the field's drawn pavement? `None` when the field
    /// draws none, so a caller never reads "no data" as "grass".
    pub fn on_pavement(&self, lat: f64, lon: f64) -> Option<bool> {
        if self.pavement.iter().all(|p| p.is_empty()) {
            return None;
        }
        Some(self.pavement.iter().any(|rings| {
            rings
                .first()
                .is_some_and(|outer| ring_contains(outer, lat, lon))
                && !rings[1..].iter().any(|hole| ring_contains(hole, lat, lon))
        }))
    }

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
        //
        // And the lead-in ARRIVES on the stand's own heading: the stand is
        // entered along its parking line through a point [`STAND_ALIGN_M`]
        // back from it. Straight from the last taxiway node, EDLP 7C was
        // reached 2 m from its spot and 147 degrees across it, so the
        // securing check (nose within 60 degrees) said "stopped short of the
        // stand" (2026-09-24).
        let pts = stand_lead_in(end, stand, m_per_deg_lon);
        let segs = vec![(String::new(), false); pts.len() - 1];
        let (lead, lead_seg) = ride_paint(&self.paint, &pts, &segs);
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
            None,
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
    ///
    /// `strip` is the destination runway's own pavement, and it is what keeps
    /// the entrance search on the right strip at a field whose thresholds sit
    /// on top of each other (see [`Self::runway_entrance_near`]). `None`
    /// falls back to nearest-entrance-anywhere.
    #[allow(clippy::too_many_arguments)]
    pub fn plan_to_runway(
        &self,
        from_lat: f64,
        from_lon: f64,
        to_lat: f64,
        to_lon: f64,
        hold_short_m: f64,
        hold_zone: &[String],
        strip: Option<&Runway>,
    ) -> Option<TaxiRoutePlan> {
        self.plan_inner(
            from_lat,
            from_lon,
            to_lat,
            to_lon,
            hold_short_m,
            hold_zone,
            true,
            strip,
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
            sim::angles::abs_diff_deg(brg, psi_true) <= AHEAD_OF_ABEAM_DEG
        };
        // Which side of the rollout a point lies on, metres, right-positive.
        // The rollout heading IS the runway direction to within a few degrees,
        // so this needs no strip geometry passed in.
        let side_of = |p: (f64, f64)| -> f64 {
            let dx = (p.1 - lon) * m_per_deg_lon;
            let dy = (p.0 - lat) * m_per_deg_lat;
            let (sin, cos) = psi_true.to_radians().sin_cos();
            dx * cos - dy * sin
        };
        // ...and which side the PARKING is on, because a vacate turns TOWARD
        // the apron. At Liege (2026-09-22) a 737 landed 04R and took C3, the
        // link to the parallel 04L: 35 m nearer than S3 and the only test was
        // distance. It ended 148 m out on the wrong side of 04R, inside 04L's
        // authored zone, with every stand and every other exit behind it - so
        // the taxi-in clearance came back "via Charlie three, Sierra three,
        // cross runway zero four right" and the drive was asked for a 180 on a
        // taxiway a 737 cannot turn on. The nearest suitable stand is the
        // proxy for the apron: at a field with parking on both sides that
        // picks the near one, which is the answer anyway. No stands in the
        // data leaves the old distance-only rule.
        let apron_side = self
            .nearest_stand(lat, lon)
            .map(|st| side_of((st.lat, st.lon)))
            .filter(|s| s.abs() > 1.0)
            .map(f64::signum);

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
                    if sim::angles::abs_diff_deg(brg, psi_true) > VACATE_TURN_MAX_DEG {
                        return None;
                    }
                    // Never onto ANOTHER runway's protected pavement. Ranking
                    // it last was not enough: at Liege it was the only link C3
                    // offered past its midpoint, so the walk took it and
                    // stopped 62 m from runway 04L's centreline, inside its
                    // departure AND arrival zones. Vacating one runway is
                    // covered by the landing clearance we hold; entering
                    // another is not covered by anything.
                    if zone_rank(e) == 2 {
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
            .min_by(|x, y| {
                // Toward the apron first, then nearest. A turn-off that leads
                // away from the parking is a longer taxi at best and, where
                // the pavement between two parallels is the thing it leads
                // onto, a drive that cannot be completed at all.
                type TurnOffCandidate = (i64, (f64, f64), (i64, (f64, f64), String, u8));
                let toward = |c: &TurnOffCandidate| -> bool {
                    apron_side.is_none_or(|a| side_of(c.2 .1) * a >= 0.0)
                };
                toward(y)
                    .cmp(&toward(x))
                    .then_with(|| dist_m(here, x.1).total_cmp(&dist_m(here, y.1)))
            })
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
                    // ...and the walk refuses it too, for the same reason.
                    if zone_rank(e) == 2 {
                        return None;
                    }
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
        strip: Option<&Runway>,
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
            .then(|| self.runway_entrance_near(to_lat, to_lon, &taxiway_nodes, strip))
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
        // ...and the route JOINS its first taxiway ahead of the aircraft,
        // never at a snapped node behind it (`join_first_edge_ahead`).
        join_first_edge_ahead(&mut path, &mut seg, (from_lat, from_lon));
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
    /// ...and it must be an entrance to THIS strip, decided by geometry like
    /// every other "is that pavement ours" question here: a runway node counts
    /// only when it lies on `strip`'s own rectangle. Nearest-entrance-anywhere
    /// was the rule, on the reasoning that a parallel field's other strip sits
    /// a runway separation away and loses on distance. Le Bourget breaks it
    /// end-on: 07 and 09 start 18 m apart and diverge, so 09's entrance (node
    /// 201, 43 m from 07's threshold) beat 07's own (node 134, 67 m). The
    /// tower cleared "holding point Alpha runway zero seven", the route was
    /// planned to 09's hold, the active-zone trim cut its last three edges,
    /// and the drive stopped 169 m from runway 07's centreline — 19 m outside
    /// [`LINE_UP_MAX_LATERAL_M`](crate::nav) as the line-up sees it. The
    /// lead-on ended 34 m off the 07 centreline too, so the authored entry
    /// was refused as well, and full auto said "I couldn't get us onto the
    /// runway" with the aircraft correctly parked where it had been sent
    /// (2026-09-22). Across the whole apt.dat this is a class like the dead-end
    /// snap above: 5,228 of 75,812 runway ends start within 300 m of a
    /// different strip's end on a different course, at 1,815 fields.
    /// `None` for the strip, or a strip no runway node sits on, keeps the old
    /// nearest-anywhere rule — thin data degrades, never refuses.
    fn runway_entrance_near(
        &self,
        lat: f64,
        lon: f64,
        taxiway_nodes: &std::collections::HashSet<i64>,
        strip: Option<&Runway>,
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
        // How far from the threshold an entrance may be. With the strip
        // known, anywhere along it: its own entrance, however far up, beats
        // a neighbour's beside the numbers (KEDW 33, #36 sweep: 15/33 is
        // 8.9 km of lakebed with no entrance within 3 km of the end, and the
        // plain snap sent the route to 07C/25C).
        let mut reach_m: f64 = 3000.0;
        // Ours, by geometry. The margin is the pavement's own half-width plus
        // a shoulder for an author whose node sits off the painted edge; the
        // along-track window is the strip's length plus the same, so a node
        // just off the numbers still counts.
        if let Some(r) = strip {
            let half = (r.width_m / 2.0).max(15.0) + STRIP_NODE_SHOULDER_M;
            let course = r.heading_a_deg();
            let length = haversine_nm(r.lat_a, r.lon_a, r.lat_b, r.lon_b) * 1852.0;
            let on_strip = |(plat, plon): (f64, f64)| {
                let d = haversine_nm(r.lat_a, r.lon_a, plat, plon) * 1852.0;
                let brg = bearing_deg(r.lat_a, r.lon_a, plat, plon);
                let ang = sim::angles::signed_diff_deg(brg, course).to_radians();
                let along = d * ang.cos();
                (d * ang.sin()).abs() <= half
                    && along >= -STRIP_NODE_SHOULDER_M
                    && along <= length + STRIP_NODE_SHOULDER_M
            };
            // A node is ours when a runway edge ALONG the strip holds it:
            // the edge runs the strip's course and its middle is on the
            // rectangle. A node alone inside the shoulder is not enough:
            // KEND 31 (#36 sweep) ends at node 54, the last node of
            // 17L/35R's edge, 24 m off 13/31's centreline and so "on" it,
            // and the route ended there; a crossing runway's edge can lie
            // wholly inside our rectangle where it crosses (CYXX 01: 07/25's
            // edge 167-168 over 01's numbers).
            let ours: std::collections::HashSet<i64> = self
                .edges
                .iter()
                .filter(|e| e.runway)
                .filter(|e| {
                    let (Some(&pa), Some(&pb)) = (self.nodes.get(&e.a), self.nodes.get(&e.b))
                    else {
                        return false;
                    };
                    let off = sim::angles::abs_diff_deg(bearing_deg(pa.0, pa.1, pb.0, pb.1), course);
                    off.min(180.0 - off) <= STRIP_EDGE_ALIGN_DEG
                        && on_strip(((pa.0 + pb.0) / 2.0, (pa.1 + pb.1) / 2.0))
                })
                .flat_map(|e| [e.a, e.b])
                .collect();
            if !ours.is_empty() {
                runway_nodes = ours;
                reach_m = reach_m.max(length + STRIP_NODE_SHOULDER_M);
            }
        }
        // One hop over a TAXIWAY edge: the stub. A hop over a RUNWAY edge
        // from our strip lands on runway pavement, and once `runway_nodes` is
        // our strip's own that is the NEIGHBOUR's: KGJT 29 (issue #36, the
        // real apt.dat sweep `real_install_converging_thresholds`) ends
        // where 04/22's edge 38-66 runs onto it, node 38 on 04/22's
        // centreline won as "one hop from runway 29", the route was planned
        // to 04/22's entrance and its lead-on ended 122 m off 11/29.
        let mut adjacent: std::collections::HashSet<i64> = std::collections::HashSet::new();
        for e in self.edges.iter().filter(|e| !e.runway) {
            if runway_nodes.contains(&e.a) {
                adjacent.insert(e.b);
            }
            if runway_nodes.contains(&e.b) {
                adjacent.insert(e.a);
            }
        }
        let dist_to = |id: &i64| {
            self.nodes.get(id).map(|&p| {
                let dx = (p.1 - lon) * m_per_deg_lon;
                let dy = (p.0 - lat) * m_per_deg_lat;
                dx.hypot(dy)
            })
        };
        let entrance = taxiway_nodes
            .iter()
            .filter(|id| runway_nodes.contains(id) || adjacent.contains(id))
            .filter_map(|id| dist_to(id).map(|d| (*id, d)))
            .min_by(|x, y| x.1.total_cmp(&y.1))
            .filter(|(_, d)| *d < reach_m)
            .map(|(id, _)| id)?;
        // A node one hop OFF the strip is where the stub starts, not where it
        // meets the runway: the route goes on to the runway node it joins, so
        // the hold-short trim finds the runway and the lead-on it leaves
        // behind reaches the centreline. Ending at the hop, the lead-on
        // stopped where the stub began. LIBP 04 (2026-09-26): taxiway D's
        // first node beside the threshold (68 m off the centreline, 92 m from
        // the numbers) beat the node where D meets the runway (on the
        // centreline, 101 m up it), the lead-on ran from the hold to 68 m
        // off, the line-up refused it as `junction_off_the_runway`, and two
        // 737s drove the synthetic 135-degree corner instead, 6.7 and 7.5 m
        // past the centreline.
        if runway_nodes.contains(&entrance) {
            return Some(entrance);
        }
        let joins = self
            .edges
            .iter()
            .filter(|e| !e.runway)
            .filter_map(|e| {
                if e.a == entrance {
                    Some(e.b)
                } else if e.b == entrance {
                    Some(e.a)
                } else {
                    None
                }
            })
            .filter(|n| runway_nodes.contains(n))
            .filter_map(|n| dist_to(&n).map(|d| (n, d)))
            .min_by(|x, y| x.1.total_cmp(&y.1))
            .map(|(n, _)| n);
        Some(joins.unwrap_or(entrance))
    }
}

/// How far off a strip's pavement a 1201 node may sit and still be read as a
/// node ON that strip. Authors place the centreline nodes by hand, and the
/// runway rectangle is the only geometry we have to judge them against.
const STRIP_NODE_SHOULDER_M: f64 = 15.0;

/// How far off the strip's course a runway edge may run and still be one of
/// the strip's own (authors' centreline nodes wander a degree or two; a
/// crossing runway is tens of degrees off).
const STRIP_EDGE_ALIGN_DEG: f64 = 15.0;

/// Pick the runway end best aligned into the wind. Returns the end's ident,
/// its true heading, and the headwind component (negative = tailwind).
/// Deterministic by construction: it can only name a runway the field has.
pub fn best_runway_for_wind(
    runways: &[Runway],
    wind_from_deg: f64,
    wind_kts: f64,
) -> Option<(String, f64, f64)> {
    let mut best: Option<(String, f64, f64)> = None;
    for r in assignable_runways(runways) {
        for (ident, hdg) in [
            (&r.ident_a, r.heading_a_deg()),
            (&r.ident_b, r.heading_b_deg()),
        ] {
            let diff = sim::angles::signed_diff_deg(wind_from_deg, hdg);
            let headwind = wind_kts * diff.to_radians().cos();
            if best.as_ref().is_none_or(|(_, _, hw)| headwind > *hw) {
                best = Some((ident.clone(), hdg, headwind));
            }
        }
    }
    // The wind picks a DIRECTION; between parallels it cannot pick the
    // strip. Their headings agree to a hundredth of a degree, so the
    // headwind decided EGKK's 26L against 26R on noise and the IFR clearance
    // named 26R, the standby, while ground and tower worked 26L (sortie
    // 1790524628, 2026-09-27). Of the ends facing that way, the LONGEST
    // strip is the field's main runway (apt.dat's thresholds: 26L 3,319 m,
    // 26R 2,563 m).
    let (best_id, best_hdg, _) = best.clone()?;
    let main = assignable_runways(runways)
        .flat_map(|r| {
            [
                (&r.ident_a, r.heading_a_deg(), r.length_m()),
                (&r.ident_b, r.heading_b_deg(), r.length_m()),
            ]
        })
        .filter(|(_, hdg, _)| sim::angles::abs_diff_deg(*hdg, best_hdg) <= PARALLEL_DEG)
        .fold(None::<(&String, f64, f64)>, |acc, e| match acc {
            Some(a) if a.2 >= e.2 => Some(a),
            _ => Some(e),
        });
    match main {
        Some((id, hdg, _)) if *id != best_id => {
            let headwind = wind_kts * sim::angles::signed_diff_deg(wind_from_deg, hdg).to_radians().cos();
            Some((id.clone(), hdg, headwind))
        }
        _ => best,
    }
}

/// Two runway ends within this many degrees face the same way: parallels
/// (EGKK's 26L and 26R differ by a hundredth of a degree).
pub const PARALLEL_DEG: f64 = 5.0;

/// Parallels closer than this cannot be worked as two runways at once
/// (ICAO Annex 14 / Doc 4444: independent parallel departures need 760 m
/// between the centrelines), so one of the pair is the runway and the other
/// its standby.
pub const CLOSE_PARALLEL_M: f64 = 760.0;

/// The runway a CLOSE parallel end stands in for: the same-direction end of
/// the longest strip within [`CLOSE_PARALLEL_M`] of this one's centreline,
/// else the end itself. EGKK's 26R (the standby, 198 m north of 26L) is
/// 26L; EGLL's 27R (1,400 m from 27L) is itself. A ghost whose replayed
/// position sits nearer the standby's centreline flew the main runway, and
/// an overheard "cleared for takeoff runway two six right" named a runway
/// the tower was not using (sortie 1790524628, +99 s).
pub fn close_parallel_main(runways: &[Runway], ident: &str) -> String {
    let Some((own, own_hdg)) = runways.iter().find_map(|r| {
        if r.ident_a == ident {
            Some((r, r.heading_a_deg()))
        } else if r.ident_b == ident {
            Some((r, r.heading_b_deg()))
        } else {
            None
        }
    }) else {
        return ident.to_string();
    };
    let own_len = own.length_m();
    let mid = |r: &Runway| ((r.lat_a + r.lat_b) / 2.0, (r.lon_a + r.lon_b) / 2.0);
    let mut out = (ident.to_string(), own_len);
    for r in assignable_runways(runways) {
        if std::ptr::eq(r, own) {
            continue;
        }
        let (end, hdg) = if sim::angles::abs_diff_deg(r.heading_a_deg(), own_hdg) <= PARALLEL_DEG {
            (&r.ident_a, r.heading_a_deg())
        } else if sim::angles::abs_diff_deg(r.heading_b_deg(), own_hdg) <= PARALLEL_DEG {
            (&r.ident_b, r.heading_b_deg())
        } else {
            continue;
        };
        // Lateral separation: the other strip's midpoint off our centreline.
        let (mlat, mlon) = mid(r);
        let (olat, olon) = mid(own);
        let d_nm = haversine_nm(olat, olon, mlat, mlon);
        let brg = bearing_deg(olat, olon, mlat, mlon);
        let lateral_m = (d_nm * 1852.0 * sim::angles::signed_diff_deg(brg, hdg).to_radians().sin()).abs();
        if lateral_m < CLOSE_PARALLEL_M && r.length_m() > out.1 {
            out = (end.clone(), r.length_m());
        }
    }
    out.0
}

/// The runways a wind pick may choose from: the field's MAIN runways. A hard
/// runway where the field has one, and never under half the longest of its
/// kind. ETND at 260/6 (2026-09-24): 26R, 446 m of grass beside the 1,240 m
/// asphalt 26, faced the wind a hair better, the tower cleared an SF50 onto
/// it, and the line-up found no runway 26R near the paved hold it had taxied
/// to. A grass strip is a runway a pilot ASKS for, not one a tower hands out.
pub fn assignable_runways(runways: &[Runway]) -> impl Iterator<Item = &Runway> {
    let any_hard = runways.iter().any(Runway::is_hard);
    let kind = move |r: &&Runway| !any_hard || r.is_hard();
    let longest = runways
        .iter()
        .filter(kind)
        .map(Runway::length_m)
        .fold(0.0, f64::max);
    runways
        .iter()
        .filter(kind)
        .filter(move |r| r.length_m() >= 0.5 * longest)
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
                return Some(wind_kts * sim::angles::signed_diff_deg(wind_from_deg, hdg).to_radians().cos());
            }
        }
    }
    None
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
#[derive(Debug, Clone, Default)]
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
    /// The missed approach point (description code M): an approach that ends
    /// here, short of the runway, has a visual segment to fly (LSZS).
    pub is_map: bool,
    /// The published vertical path angle, degrees (field 28: "-449" = 4.49).
    pub vpa_deg: Option<f64>,
    /// The recommended navaid (field 13): what a DME-terminated leg (CD, VD,
    /// FD) measures its distance from. Empty when the record names none.
    pub navaid_ident: String,
    /// Its ICAO region (field 14), which narrows namesakes at resolve time.
    pub navaid_region: String,
    /// That navaid resolved, once [`resolve_proc_fixes`] has run.
    pub navaid: Option<Fix>,
    /// The leg's shape beyond its fix: an RF arc's centre and radius, a
    /// holding leg's length.
    pub path: LegPath,
}

/// What an RF or holding leg carries besides its fix (issue #101).
#[derive(Debug, Clone, Default)]
pub struct LegPath {
    /// RF: the arc's centre fix (fields 30 and 31) and radius, nm (field 17,
    /// thousandths: LOWI's "001400" is 1.4 nm).
    pub arc_center_ident: String,
    pub arc_center_region: String,
    /// The centre resolved, once [`resolve_proc_fixes`] has run.
    pub arc_center: Option<Fix>,
    pub arc_radius_nm: Option<f64>,
    /// HM / HA / HF: the inbound leg's time, minutes (field 21, "T010" = 1.0
    /// min); a leg coded by distance keeps it in `dist_nm`.
    pub hold_leg_min: Option<f64>,
}

/// An RF leg the follower can fly: its fix, its centre and its radius known.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Arc {
    pub center_lat: f64,
    pub center_lon: f64,
    pub radius_nm: f64,
    /// 'R' clockwise, 'L' anticlockwise, seen from above.
    pub right: bool,
    pub end_lat: f64,
    pub end_lon: f64,
}

impl ProcLeg {
    /// This leg as a flyable RF arc: `None` for any other leg, or an RF whose
    /// centre, radius, turn or end fix is unknown (flown as before).
    pub fn arc(&self) -> Option<Arc> {
        if self.leg_type != "RF" {
            return None;
        }
        let c = self.path.arc_center.as_ref()?;
        let f = self.fix.as_ref()?;
        let right = match self.turn? {
            'R' => true,
            'L' => false,
            _ => return None,
        };
        // The radius from the chart, else from the geometry (the end fix is
        // on the arc by construction).
        let radius_nm = self
            .path
            .arc_radius_nm
            .filter(|r| *r > 0.05)
            .unwrap_or_else(|| haversine_nm(c.lat, c.lon, f.lat, f.lon));
        (radius_nm > 0.05).then_some(Arc {
            center_lat: c.lat,
            center_lon: c.lon,
            radius_nm,
            right,
            end_lat: f.lat,
            end_lon: f.lon,
        })
    }

    /// A holding leg (HM manual, HA to an altitude, HF one circuit).
    pub fn is_hold(&self) -> bool {
        matches!(self.leg_type.as_str(), "HM" | "HA" | "HF")
    }
}

/// A CIFP distance/time cell: "T010" is 1.0 minute (a hold's leg time),
/// anything else tenths of a mile. Returns (nm, minutes).
fn arinc_dist_or_time(s: &str) -> (Option<f64>, Option<f64>) {
    let s = s.trim();
    match s.strip_prefix('T') {
        Some(t) => (None, arinc_tenths(t)),
        None => (arinc_tenths(s), None),
    }
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
        let leg_type = cifp_field(&f, 11).to_string();
        let (dist_nm, leg_min) = arinc_dist_or_time(cifp_field(&f, 21));
        let rf = leg_type == "RF";
        out.push(ProcLeg {
            seq,
            transition: tr,
            path: LegPath {
                arc_center_ident: if rf { cifp_field(&f, 30).to_string() } else { String::new() },
                arc_center_region: if rf { cifp_field(&f, 31).to_string() } else { String::new() },
                arc_center: None,
                arc_radius_nm: if rf {
                    cifp_field(&f, 17).parse::<f64>().ok().map(|r| r / 1000.0).filter(|r| *r > 0.0)
                } else {
                    None
                },
                hold_leg_min: leg_min,
            },
            leg_type,
            fix_ident: cifp_field(&f, 4).to_string(),
            fix_region: cifp_field(&f, 5).to_string(),
            fix: None,
            course_mag: arinc_tenths(cifp_field(&f, 20)),
            dist_nm,
            // The turn direction is field 9; 12 is only its "valid" flag
            // (LOWI R26-Z's RF leg: "L" in 9, blank in 12).
            turn: cifp_field(&f, 9)
                .chars()
                .next()
                .filter(|c| *c == 'L' || *c == 'R'),
            alt_desc: cifp_field(&f, 22).chars().next().unwrap_or(' '),
            alt1_ft: arinc_alt_ft(cifp_field(&f, 23)),
            alt2_ft: arinc_alt_ft(cifp_field(&f, 24)),
            is_faf: cifp_field(&f, 8).chars().nth(3) == Some('F'),
            is_map: cifp_field(&f, 8).chars().nth(3) == Some('M'),
            vpa_deg: cifp_field(&f, 28)
                .parse::<f64>()
                .ok()
                .map(|v| v.abs() / 100.0)
                .filter(|v| (1.0..=8.0).contains(v)),
            navaid_ident: cifp_field(&f, 13).to_string(),
            navaid_region: cifp_field(&f, 14).to_string(),
            navaid: None,
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
    approach_candidates(cifp_dir, icao, runway).into_iter().next()
}

/// Every approach to `runway` the follower can fly, best first: by class
/// (ILS, RNAV, LOC, VOR), a straight-in to the threshold ahead of one that
/// ends at a missed approach point with a visual segment (`map_final_legs`),
/// then file order. Only an approach that ENDS on this runway or at a MAP on
/// straight legs qualifies: LOWI's RNAV 08 V and 26 E are circling procedures
/// with no runway fix, 08 Z turns onto the runway on RF arcs, and picked by
/// rank alone the circling one won and its missed approach was flown as the
/// approach (2026-09-24). The geometry of a visual segment is the caller's to
/// judge once the fixes are resolved (`visual_segment`).
pub fn approach_candidates(cifp_dir: &str, icao: &str, runway: &str) -> Vec<String> {
    let rank = |id: &str| match id.chars().next() {
        Some('I') => 0,
        Some('X') | Some('R') | Some('Q') => 1,
        Some('L') => 2,
        Some('D') | Some('S') | Some('V') => 3,
        _ => 4,
    };
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let Ok(text) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    let mut out: Vec<(u8, u8, String)> = list_procedures(cifp_dir, icao, ProcKind::Appch)
        .into_iter()
        .filter(|id| approach_runway(id).is_some_and(|r| runway_matches(&r, runway)))
        .filter_map(|id| {
            let legs = parse_proc_legs(text.lines(), ProcKind::Appch, &id, None);
            if straight_in_legs(legs.clone(), runway).is_some() {
                Some((rank(&id), 0, id))
            } else if map_final_legs(legs).is_some() {
                Some((rank(&id), 1, id))
            } else {
                None
            }
        })
        .collect();
    out.sort_by_key(|(r, m, _)| (*r, *m));
    out.into_iter().map(|(_, _, id)| id).collect()
}

/// An approach that ends at a missed approach point SHORT of the runway (no
/// runway fix before it), legs as far as the MAP, with no RF arc before it.
/// LSZS's RNAV 03 and 21: a 4.5 degree final to a MAP about two miles out,
/// then a visual segment. `None` otherwise.
pub fn map_final_legs(legs: Vec<ProcLeg>) -> Option<Vec<ProcLeg>> {
    let at = legs.iter().position(|l| l.is_map || l.fix_ident.starts_with("RW"))?;
    if !legs[at].is_map || legs[..=at].iter().any(|l| l.leg_type == "RF") {
        return None;
    }
    let mut legs = legs;
    legs.truncate(at + 1);
    Some(legs)
}

/// The visual segment from a MAP to the threshold: how far, how far off the
/// extended centreline, how far the final course is from the runway's, and
/// the angle down to the threshold.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VisualSegment {
    pub dist_nm: f64,
    pub lateral_nm: f64,
    pub course_diff_deg: f64,
    pub angle_deg: f64,
}

/// A visual segment the landing can fly from: MAP within this of the threshold...
pub const VISUAL_MAX_NM: f64 = 3.0;
/// ...this close to the extended centreline...
pub const VISUAL_MAX_LATERAL_NM: f64 = 0.4;
/// ...on a final course within this of the runway's (straight-in criteria)...
pub const VISUAL_MAX_COURSE_DEG: f64 = 30.0;
/// ...and no steeper than this down to the threshold.
pub const VISUAL_MAX_ANGLE_DEG: f64 = 5.0;

/// Judge the visual segment of resolved `legs` (ending at the MAP) onto the
/// threshold at `thr`, runway course `course_deg` true, elevation `thr_ft`.
pub fn visual_segment(
    legs: &[ProcLeg],
    thr: (f64, f64),
    course_deg: f64,
    thr_ft: f64,
) -> Option<VisualSegment> {
    let map = legs.last()?;
    let mf = map.fix.as_ref()?;
    let map_ft = map.alt1_ft?;
    let prev = legs[..legs.len() - 1].iter().rev().find_map(|l| l.fix.as_ref())?;
    let final_course = bearing_deg(prev.lat, prev.lon, mf.lat, mf.lon);
    let dist_nm = haversine_nm(mf.lat, mf.lon, thr.0, thr.1);
    // Position relative to the runway: the MAP lies BEFORE the threshold on
    // the approach side, so from the threshold it bears the reciprocal.
    let from_thr = bearing_deg(thr.0, thr.1, mf.lat, mf.lon);
    let off = sim::angles::signed_diff_deg(from_thr, (course_deg + 180.0).rem_euclid(360.0));
    let lateral_nm = dist_nm * off.to_radians().sin().abs();
    let height_ft = map_ft - thr_ft;
    Some(VisualSegment {
        dist_nm,
        lateral_nm,
        course_diff_deg: sim::angles::abs_diff_deg(final_course, course_deg),
        angle_deg: (height_ft / (dist_nm * 6076.12)).atan().to_degrees(),
    })
}

impl VisualSegment {
    /// Flyable as a straight-in visual segment by the landing.
    pub fn flyable(&self) -> bool {
        self.dist_nm <= VISUAL_MAX_NM
            && self.lateral_nm <= VISUAL_MAX_LATERAL_NM
            && self.course_diff_deg <= VISUAL_MAX_COURSE_DEG
            && (2.5..=VISUAL_MAX_ANGLE_DEG).contains(&self.angle_deg)
    }
}

/// The named transitions (route type A) of approach `ident`, file order.
pub fn approach_transitions(cifp_dir: &str, icao: &str, ident: &str) -> Vec<String> {
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let Ok(text) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    let mut out: Vec<String> = Vec::new();
    for line in text.lines() {
        let Some(body) = line.strip_prefix("APPCH:") else { continue };
        let f: Vec<&str> = body.split(',').collect();
        if cifp_field(&f, 1) == "A" && cifp_field(&f, 2) == ident {
            let t = cifp_field(&f, 3).to_string();
            if !t.is_empty() && !out.contains(&t) {
                out.push(t);
            }
        }
    }
    out
}

/// An approach's legs as far as the runway threshold, when it is a straight-in
/// the follower can fly: a runway fix for THIS runway (exact designator, so a
/// grass "26R" never inherits a "26" approach), and no RF (constant-radius
/// arc) leg before it, which the follower would chord across. The missed
/// approach after the threshold is dropped: it is never the approach.
/// `None` for a circling procedure or an arc.
pub fn straight_in_legs(legs: Vec<ProcLeg>, runway: &str) -> Option<Vec<ProcLeg>> {
    let at = legs.iter().position(|l| {
        l.fix_ident
            .strip_prefix("RW")
            .is_some_and(|r| r.eq_ignore_ascii_case(runway) || (runway_number(r) == runway_number(runway) && side_letter(r) == side_letter(runway)))
    })?;
    if legs[..at].iter().any(|l| l.leg_type == "RF") {
        return None;
    }
    let mut legs = legs;
    legs.truncate(at + 1);
    Some(legs)
}

fn side_letter(rwy: &str) -> Option<char> {
    rwy.chars().rev().find(|c| matches!(c, 'L' | 'R' | 'C'))
}

/// The highest minimum sector altitude published for `icao` (the airport's
/// own record in `earth_msa.dat`, else any record belonging to it), feet.
/// LSZS: sectors of 15,500 and 13,500 ft out to 25 nm. `None` without a record.
pub fn airport_msa_ft(msa_path: &str, icao: &str) -> Option<f64> {
    let text = std::fs::read_to_string(msa_path).ok()?;
    airport_msa_in(text.lines(), icao)
}

/// [`airport_msa_ft`] over the file's lines (pure, for tests).
pub fn airport_msa_in<'a>(lines: impl Iterator<Item = &'a str>, icao: &str) -> Option<f64> {
    let mut own: Option<f64> = None;
    let mut any: Option<f64> = None;
    for line in lines {
        let f: Vec<&str> = line.split_whitespace().collect();
        if f.len() < 8 || !f[3].eq_ignore_ascii_case(icao) {
            continue;
        }
        let mut max: Option<f64> = None;
        for t in f[5..].chunks(3) {
            if t.len() < 3 {
                break;
            }
            let (Ok(alt), Ok(radius)) = (t[1].parse::<f64>(), t[2].parse::<f64>()) else {
                break;
            };
            if alt <= 0.0 && radius <= 0.0 {
                break;
            }
            max = Some(max.unwrap_or(0.0).max(alt * 100.0));
        }
        let Some(m) = max else { continue };
        if f[1].eq_ignore_ascii_case(icao) {
            own = Some(own.unwrap_or(0.0).max(m));
        } else {
            any = Some(any.unwrap_or(0.0).max(m));
        }
    }
    own.or(any)
}

/// A minimum altitude a departure must cross a fix at or above.
#[derive(Debug, Clone, PartialEq)]
pub struct ClimbMinimum {
    pub fix: String,
    pub min_ft: f64,
}

/// The minimum altitudes of SID `sid` at `icao` for `runway`, in procedure
/// order: its runway transition (a parallel "RW26B" included), the common
/// part and any enroute transition. `+` and "at" constraints are minimums, a
/// `B` window's lower bound is; `-` constraints bound nothing from below.
/// LOWI's MOGT3H needs 7,850 ft at WI506 nine miles out, and the follower
/// flew SimBrief's predicted profile at cruise speed, about 2,200 ft under it
/// (2026-09-24).
pub fn sid_minimums(cifp_dir: &str, icao: &str, sid: &str, runway: &str) -> Vec<ClimbMinimum> {
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let Ok(text) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    sid_minimums_in(text.lines(), sid, runway)
}

/// [`sid_minimums`] over the file's lines (pure, for tests).
pub fn sid_minimums_in<'a>(
    lines: impl Iterator<Item = &'a str>,
    sid: &str,
    runway: &str,
) -> Vec<ClimbMinimum> {
    let mut out: Vec<ClimbMinimum> = Vec::new();
    for line in lines {
        let Some(body) = line.strip_prefix("SID:") else {
            continue;
        };
        let f: Vec<&str> = body.split(',').collect();
        if !cifp_field(&f, 2).eq_ignore_ascii_case(sid) {
            continue;
        }
        let tr = cifp_field(&f, 3);
        let ours = match tr.strip_prefix("RW") {
            // "RW26B" serves both parallels: its number is enough.
            Some(r) if r.ends_with('B') => runway_number(r) == runway_number(runway),
            Some(r) => runway_matches(r, runway) && r.len() == runway.len(),
            None => true, // the common part, or an enroute transition
        };
        if !ours {
            continue;
        }
        let fix = cifp_field(&f, 4);
        if fix.is_empty() {
            continue;
        }
        let (a1, a2) = (arinc_alt_ft(cifp_field(&f, 23)), arinc_alt_ft(cifp_field(&f, 24)));
        let min = match cifp_field(&f, 22).chars().next().unwrap_or(' ') {
            '+' | ' ' => a1,
            'B' => a2,
            _ => None,
        };
        if let Some(min_ft) = min {
            match out.iter_mut().find(|m| m.fix == fix) {
                Some(m) => m.min_ft = m.min_ft.max(min_ft),
                None => out.push(ClimbMinimum {
                    fix: fix.to_string(),
                    min_ft,
                }),
            }
        }
    }
    out
}

/// X-Plane's grid MORA (`earth_mora.dat`): the minimum off-route altitude of
/// every one-degree cell, feet, the highest ground or obstacle in it plus
/// 1,000 ft (2,000 where that ground is above 5,000 ft). A line is "LAT LON"
/// then 30 cells eastward in hundreds of feet; LAT is the cell's SOUTH edge
/// (Everest, 27.99N 86.93E, is the 311 in row +27, column 86).
#[derive(Debug, Clone, Default)]
pub struct MoraGrid {
    cells: std::collections::HashMap<(i32, i32), f64>,
}

impl MoraGrid {
    /// Parse the file's lines. Unparseable lines are skipped.
    pub fn parse<'a>(lines: impl Iterator<Item = &'a str>) -> MoraGrid {
        let mut cells = std::collections::HashMap::new();
        for line in lines {
            let f: Vec<&str> = line.split_whitespace().collect();
            if f.len() < 3 {
                continue;
            }
            let (Ok(lat), Ok(lon0)) = (f[0].parse::<i32>(), f[1].parse::<i32>()) else {
                continue;
            };
            for (i, v) in f[2..].iter().enumerate() {
                if let Ok(h) = v.parse::<f64>() {
                    cells.insert((lat, lon0 + i as i32), h * 100.0);
                }
            }
        }
        MoraGrid { cells }
    }

    /// Load `earth_mora.dat`; `None` when it cannot be read.
    pub fn load(path: &str) -> Option<MoraGrid> {
        let text = std::fs::read_to_string(path).ok()?;
        let g = MoraGrid::parse(text.lines());
        (!g.cells.is_empty()).then_some(g)
    }

    /// The MORA of the cell holding this point, feet.
    pub fn at(&self, lat: f64, lon: f64) -> Option<f64> {
        self.cells
            .get(&(lat.floor() as i32, lon.floor() as i32))
            .copied()
    }
}

/// The highest ground a MORA stands for, feet: its 2,000 ft margin above
/// 7,000, its 1,000 below. Sea cells (1,000 or less) are sea level.
pub fn mora_ground_ft(mora_ft: f64) -> f64 {
    if mora_ft > 7_000.0 {
        mora_ft - 2_000.0
    } else {
        (mora_ft - 1_000.0).max(0.0)
    }
}

/// One published crossing restriction of an arrival (STAR) fix: altitude
/// bounds in feet (either or both; an "at" sets both) and a speed limit.
#[derive(Debug, Clone, PartialEq)]
pub struct StarConstraint {
    pub fix: String,
    /// At or above (a `+`, an "at", a `B` window's lower bound).
    pub min_ft: Option<f64>,
    /// At or below (a `-`, an "at", a `B` window's upper bound).
    pub max_ft: Option<f64>,
    /// At or below this indicated speed crossing the fix, and from there on.
    pub max_kts: Option<f64>,
}

impl StarConstraint {
    /// The altitude a plan's `planned_ft` becomes at this fix: clamped into
    /// the published window. An "at" is exactly its number.
    pub fn clamp_ft(&self, planned_ft: f64) -> f64 {
        let mut a = planned_ft;
        if let Some(max) = self.max_ft {
            a = a.min(max);
        }
        if let Some(min) = self.min_ft {
            a = a.max(min);
        }
        a
    }
}

/// The crossing restrictions of STAR `star` at `icao`, for the fixes on the
/// route: its common part, the enroute transition the route enters by (a
/// transition named for a fix in `route_fixes`), and the runway transition
/// for `runway` ("ALL" and a parallel's "RW10B" included). Speed cells are
/// field 26 (descriptor) and 27: a blank or `-` descriptor is a limit, a `+`
/// is a minimum the governor has no use for. EGPH's STARs all end "FL070,
/// 230 knots" at TARTN, and the follower flew SimBrief's own guess at a
/// descent there, 278 kt at the last fix (EGTE 2026-09-21 had the same shape).
pub fn star_constraints(
    cifp_dir: &str,
    icao: &str,
    star: &str,
    runway: Option<&str>,
    route_fixes: &[&str],
) -> Vec<StarConstraint> {
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let Ok(text) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    star_constraints_in(text.lines(), star, runway, route_fixes)
}

/// [`star_constraints`] over the file's lines (pure, for tests). One entry
/// per fix, in procedure order; a fix published twice keeps the tighter
/// bounds of both.
pub fn star_constraints_in<'a>(
    lines: impl Iterator<Item = &'a str>,
    star: &str,
    runway: Option<&str>,
    route_fixes: &[&str],
) -> Vec<StarConstraint> {
    let on_route = |f: &str| route_fixes.iter().any(|r| r.eq_ignore_ascii_case(f));
    let mut out: Vec<StarConstraint> = Vec::new();
    for line in lines {
        let Some(body) = line.strip_prefix("STAR:") else {
            continue;
        };
        let f: Vec<&str> = body.split(',').collect();
        if !cifp_field(&f, 2).eq_ignore_ascii_case(star) {
            continue;
        }
        let tr = cifp_field(&f, 3);
        let ours = if tr.is_empty() || tr.eq_ignore_ascii_case("ALL") {
            true
        } else if let Some(r) = tr.strip_prefix("RW") {
            match runway {
                Some(want) if r.ends_with('B') => runway_number(r) == runway_number(want),
                Some(want) => runway_matches(r, want) && r.len() == want.len(),
                // No planned runway: a runway transition's numbers are a
                // guess, and a wrong guess is a wrong number.
                None => false,
            }
        } else {
            on_route(tr)
        };
        let fix = cifp_field(&f, 4);
        if !ours || fix.is_empty() || !on_route(fix) {
            continue;
        }
        let (a1, a2) = (arinc_alt_ft(cifp_field(&f, 23)), arinc_alt_ft(cifp_field(&f, 24)));
        let (min_ft, max_ft) = match cifp_field(&f, 22).chars().next().unwrap_or(' ') {
            ' ' | '@' => (a1, a1),
            '+' => (a1, None),
            '-' => (None, a1),
            'B' => (a2, a1),
            _ => (None, None),
        };
        let max_kts = match cifp_field(&f, 26).chars().next().unwrap_or(' ') {
            ' ' | '-' | '@' => cifp_field(&f, 27).parse::<f64>().ok().filter(|k| *k > 0.0),
            _ => None,
        };
        if min_ft.is_none() && max_ft.is_none() && max_kts.is_none() {
            continue;
        }
        let tighter = |a: Option<f64>, b: Option<f64>, lo: bool| match (a, b) {
            (Some(x), Some(y)) => Some(if lo { x.max(y) } else { x.min(y) }),
            (x, y) => x.or(y),
        };
        match out.iter_mut().find(|c| c.fix.eq_ignore_ascii_case(fix)) {
            Some(c) => {
                c.min_ft = tighter(c.min_ft, min_ft, true);
                c.max_ft = tighter(c.max_ft, max_ft, false);
                c.max_kts = tighter(c.max_kts, max_kts, false);
            }
            None => out.push(StarConstraint {
                fix: fix.to_string(),
                min_ft,
                max_ft,
                max_kts,
            }),
        }
    }
    out
}

/// A SID that departs `runway` at `icao` — the first ident with a leg whose
/// transition is `RW<runway>`. `None` if the field publishes no SID for it.
/// A departure procedure the crew can fly from the runway it was given, and
/// where it joins the planned route.
#[derive(Debug, Clone, PartialEq)]
pub struct SidChoice {
    pub ident: String,
    /// The runway transition ("RW10", "RW26B").
    pub transition: String,
    /// The last SID fix that is also on the route: from here the route goes on.
    pub join: String,
    /// Its index in the route idents given.
    pub join_index: usize,
    /// The SID's fixes up to and including the join.
    pub fixes: Vec<String>,
}

/// The SIDs that serve `runway` and join `route` (its fix idents in order),
/// best first: the one that joins furthest along the route, then the one
/// with the fewest fixes. The crew chooses its own departure from the
/// runway actually in use: at LIME (2026-09-25) the tower gave runway 10 and
/// the plan's LEGL1P departs 28; LEGL1R and LEGL1W depart 10 to the same
/// LEGLO. Pure over the CIFP file's lines.
pub fn sids_for_route_in<'a>(
    lines: impl Iterator<Item = &'a str> + Clone,
    runway: &str,
    route: &[String],
) -> Vec<SidChoice> {
    let idents = sid_transitions_in(lines.clone(), runway);
    let mut out: Vec<SidChoice> = Vec::new();
    for (id, tr) in idents {
        let fixes: Vec<String> = parse_proc_legs(lines.clone(), ProcKind::Sid, &id, Some(&tr))
            .into_iter()
            .map(|l| l.fix_ident)
            .filter(|f| !f.is_empty() && !f.starts_with("RW"))
            .collect();
        let join = fixes.iter().enumerate().filter_map(|(k, f)| {
            route
                .iter()
                .position(|r| r.eq_ignore_ascii_case(f))
                .map(|ri| (k, ri))
        });
        let Some((k, ri)) = join.max_by_key(|(_, ri)| *ri) else { continue };
        out.push(SidChoice {
            ident: id,
            transition: tr,
            join: fixes[k].clone(),
            join_index: ri,
            fixes: fixes[..=k].to_vec(),
        });
    }
    out.sort_by(|a, b| b.join_index.cmp(&a.join_index).then(a.fixes.len().cmp(&b.fixes.len())));
    out
}

/// [`sids_for_route_in`] from the airport's CIFP file.
pub fn sids_for_route(cifp_dir: &str, icao: &str, runway: &str, route: &[String]) -> Vec<SidChoice> {
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let Ok(text) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    sids_for_route_in(text.lines(), runway, route)
}

/// Every SID that serves `runway`, with its runway transition ("RW08R",
/// "RW26B"), in file order. Pure over the CIFP file's lines.
pub fn sid_transitions_in<'a>(lines: impl Iterator<Item = &'a str>, runway: &str) -> Vec<(String, String)> {
    let mut idents: Vec<(String, String)> = Vec::new();
    for line in lines {
        let Some(body) = line.strip_prefix("SID:") else { continue };
        let f: Vec<&str> = body.split(',').collect();
        let (id, tr) = (cifp_field(&f, 2), cifp_field(&f, 3));
        let serves = tr
            .strip_prefix("RW")
            .is_some_and(|r| runway_matches(r.trim_end_matches('B'), runway) || r.eq_ignore_ascii_case(runway));
        if serves && !id.is_empty() && !idents.iter().any(|(i, t)| i == id && t == tr) {
            idents.push((id.to_string(), tr.to_string()));
        }
    }
    idents
}

/// [`sid_transitions_in`] from the airport's CIFP file. Empty with no file.
pub fn sid_transitions(cifp_dir: &str, icao: &str, runway: &str) -> Vec<(String, String)> {
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let Ok(text) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    sid_transitions_in(text.lines(), runway)
}

/// The departure for a FILED route: the filed SID itself when it serves the
/// runway and joins the route, else the best of [`sids_for_route`] (it
/// serves the runway and joins the route furthest along, the fewest fixes
/// on a tie). ONE rule for both sides of the frequency: the tower clears it
/// (the station's IFR clearance) and the crew flies it
/// (`full_auto::choose_departure` walks the same order), so a clearance is
/// never a SID the crew cannot fly to its route. EGKK 08R (sortie
/// 1790601822, 2026-09-28): the tower cleared KENE3P, whose exit KENET lay
/// nearest the bearing to Birmingham, for a route filed LAM L610 HEMEL; the
/// crew flew LAM5P. `None` when no SID off the runway meets the route.
/// Pure over the CIFP file's lines.
pub fn sid_for_filed_route_in<'a>(
    lines: impl Iterator<Item = &'a str> + Clone,
    runway: &str,
    route: &[String],
    filed_sid: Option<&str>,
) -> Option<SidChoice> {
    let mut all = sids_for_route_in(lines, runway, route);
    let filed = filed_sid.and_then(|f| all.iter().position(|c| c.ident.eq_ignore_ascii_case(f)));
    match filed {
        Some(i) => Some(all.swap_remove(i)),
        None => all.into_iter().next(),
    }
}

/// [`sid_for_filed_route_in`] from the airport's CIFP file.
pub fn sid_for_filed_route(
    cifp_dir: &str,
    icao: &str,
    runway: &str,
    route: &[String],
    filed_sid: Option<&str>,
) -> Option<SidChoice> {
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let text = std::fs::read_to_string(path).ok()?;
    sid_for_filed_route_in(text.lines(), runway, route, filed_sid)
}

/// An arrival procedure the crew can fly to the runway it expects, and where
/// it leaves the planned route.
#[derive(Debug, Clone)]
pub struct StarChoice {
    pub ident: String,
    /// The first STAR fix on the route: the route is flown to here, the
    /// STAR from here.
    pub entry: String,
    pub entry_index: usize,
    /// The STAR's legs from the entry on (fixes only, unresolved): its
    /// enroute transition, common part and runway transition, in order.
    pub legs: Vec<ProcLeg>,
    /// Legs the follower cannot fly as published from the entry on: arcs
    /// (AF/RF) flown as chords, vector legs dropped.
    pub bent: usize,
}

/// Every STAR at the field that serves `runway` and meets `route`, the one
/// entering earliest first (the most of the route flown as published), then
/// the one with fewest legs that fix-to-fix flying bends (BAME6N's DME arc
/// against BAME5M's straight RNAV legs, LIPE 12), then the shortest. A STAR's rows are three parts by route type (field 1):
/// 1/4/7 an enroute transition named for its first fix, 2/5/8 the common
/// part (named for its runway, "ALL" or blank), 3/6/9 a runway transition.
/// Vector legs (no fix) are dropped: the follower flies fix to fix, and the
/// approach picks up where the fixes end.
pub fn stars_for_route_in<'a>(
    lines: impl Iterator<Item = &'a str> + Clone,
    runway: &str,
    route: &[String],
) -> Vec<StarChoice> {
    let serves = |tr: &str| {
        tr.strip_prefix("RW").is_some_and(|r| {
            if r.ends_with('B') {
                runway_number(r) == runway_number(runway)
            } else {
                runway_matches(r, runway) && r.len() == runway.len()
            }
        })
    };
    let group = |t: &str| match t {
        "1" | "4" | "7" => 0,
        "2" | "5" | "8" => 1,
        "3" | "6" | "9" => 2,
        _ => 3,
    };
    // ident -> (group, transition) in file order.
    let mut parts: Vec<(String, Vec<(u8, String)>)> = Vec::new();
    for line in lines.clone() {
        let Some(body) = line.strip_prefix("STAR:") else { continue };
        let f: Vec<&str> = body.split(',').collect();
        let (id, tr, g) = (cifp_field(&f, 2), cifp_field(&f, 3), group(cifp_field(&f, 1)));
        if id.is_empty() || g == 3 {
            continue;
        }
        let entry = match parts.iter_mut().find(|(i, _)| i == id) {
            Some(e) => e,
            None => {
                parts.push((id.to_string(), Vec::new()));
                parts.last_mut().expect("just pushed")
            }
        };
        if !entry.1.iter().any(|(gg, t)| *gg == g && t == tr) {
            entry.1.push((g, tr.to_string()));
        }
    }
    let on_route = |f: &str| route.iter().position(|r| r.eq_ignore_ascii_case(f));
    let legs_of = |id: &str, tr: &str| -> Vec<ProcLeg> {
        parse_proc_legs(lines.clone(), ProcKind::Star, id, Some(tr))
            .into_iter()
            .filter(|l| l.transition == tr)
            .collect()
    };
    let mut out = Vec::new();
    for (id, trs) in parts {
        let common: Vec<&String> = trs
            .iter()
            .filter(|(g, t)| *g == 1 && (t.is_empty() || t.eq_ignore_ascii_case("ALL") || serves(t)))
            .map(|(_, t)| t)
            .collect();
        let rw_tr = trs.iter().find(|(g, t)| *g == 2 && serves(t)).map(|(_, t)| t);
        let has_rw_trs = trs.iter().any(|(g, _)| *g == 2);
        let common_for_rw = common.iter().any(|t| serves(t));
        if !(common_for_rw || rw_tr.is_some() || (!common.is_empty() && !has_rw_trs)) {
            continue;
        }
        // The enroute transition the route enters by: the one with a fix
        // earliest on the route.
        let enroute = trs
            .iter()
            .filter(|(g, _)| *g == 0)
            .filter_map(|(_, t)| {
                let legs = legs_of(&id, t);
                let first = legs.iter().filter_map(|l| on_route(&l.fix_ident)).min()?;
                Some((first, legs))
            })
            .min_by_key(|(first, _)| *first)
            .map(|(_, legs)| legs)
            .unwrap_or_default();
        let mut path: Vec<ProcLeg> = enroute;
        for t in &common {
            path.extend(legs_of(&id, t));
        }
        if let Some(t) = rw_tr {
            path.extend(legs_of(&id, t));
        }
        path.retain(|l| !l.fix_ident.starts_with("RW"));
        path.dedup_by(|b, a| a.fix_ident.eq_ignore_ascii_case(&b.fix_ident));
        let Some((p, ri)) = path
            .iter()
            .enumerate()
            .find_map(|(p, l)| on_route(&l.fix_ident).map(|ri| (p, ri)))
        else {
            continue;
        };
        let mut legs = path.split_off(p);
        let bent = legs
            .iter()
            .filter(|l| l.fix_ident.is_empty() || matches!(l.leg_type.as_str(), "AF" | "RF"))
            .count();
        legs.retain(|l| !l.fix_ident.is_empty());
        out.push(StarChoice {
            ident: id,
            entry: legs[0].fix_ident.clone(),
            entry_index: ri,
            legs,
            bent,
        });
    }
    out.sort_by(|a, b| {
        a.entry_index
            .cmp(&b.entry_index)
            .then(a.bent.cmp(&b.bent))
            .then(a.legs.len().cmp(&b.legs.len()))
    });
    out
}

/// [`stars_for_route_in`] from the airport's CIFP file.
pub fn stars_for_route(cifp_dir: &str, icao: &str, runway: &str, route: &[String]) -> Vec<StarChoice> {
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let Ok(text) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    stars_for_route_in(text.lines(), runway, route)
}

/// Where one SID off one runway leaves the terminal area: the last fix of
/// its runway transition plus common route, positioned.
#[derive(Debug, Clone, PartialEq)]
pub struct SidExit {
    pub ident: String,
    /// The runway transition ("RW24", "RW26B").
    pub transition: String,
    pub fix_ident: String,
    pub lat: f64,
    pub lon: f64,
}

/// Every SID's exit off every runway at a field, from the CIFP lines, the
/// fixes positioned through [`resolve_proc_fixes`] (ONE scan of each nav
/// file for the whole field: seconds on the real files, so the owner runs
/// it off the reply path and caches it). A SID whose legs resolve no fix
/// has no exit and is left out.
pub fn sid_exits_in<'a>(
    lines: impl Iterator<Item = &'a str> + Clone,
    fix_path: &str,
    nav_path: &str,
    airport: &AirportInfo,
    icao: &str,
) -> Vec<SidExit> {
    let mut idents: Vec<(String, String)> = Vec::new();
    for line in lines.clone() {
        let Some(body) = line.strip_prefix("SID:") else { continue };
        let f: Vec<&str> = body.split(',').collect();
        let (id, tr) = (cifp_field(&f, 2), cifp_field(&f, 3));
        if !id.is_empty() && tr.starts_with("RW") && !idents.iter().any(|(i, t)| i == id && t == tr) {
            idents.push((id.to_string(), tr.to_string()));
        }
    }
    // All the legs in one vector, so the nav files are scanned once.
    let mut legs: Vec<ProcLeg> = Vec::new();
    let mut spans: Vec<(usize, usize)> = Vec::new();
    for (id, tr) in &idents {
        let start = legs.len();
        legs.extend(parse_proc_legs(lines.clone(), ProcKind::Sid, id, Some(tr)));
        spans.push((start, legs.len()));
    }
    let (alat, alon) = airport.position().unwrap_or((0.0, 0.0));
    resolve_proc_fixes(fix_path, nav_path, airport, icao, &mut legs, alat, alon);
    idents
        .into_iter()
        .zip(spans)
        .filter_map(|((ident, transition), (a, b))| {
            let last = legs[a..b]
                .iter()
                .rev()
                .find(|l| l.fix.is_some() && !l.fix_ident.starts_with("RW"))?;
            let fix = last.fix.as_ref()?;
            Some(SidExit {
                ident,
                transition,
                fix_ident: last.fix_ident.clone(),
                lat: fix.lat,
                lon: fix.lon,
            })
        })
        .collect()
}

/// [`sid_exits_in`] from the field's CIFP file. Empty with no file.
pub fn sid_exits(
    cifp_dir: &str,
    fix_path: &str,
    nav_path: &str,
    airport: &AirportInfo,
    icao: &str,
) -> Vec<SidExit> {
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let Ok(text) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    sid_exits_in(text.lines(), fix_path, nav_path, airport, icao)
}

/// The SID off `runway` that leaves toward the destination: the one whose
/// exit fix lies closest in bearing (from the field) to the great-circle
/// bearing from the field to the destination. `(ident, exit bearing,
/// destination bearing)`, degrees true; `None` when no exit serves the
/// runway. Pure.
pub fn sid_toward(
    exits: &[SidExit],
    runway: &str,
    field: (f64, f64),
    dest: (f64, f64),
) -> Option<(String, f64, f64)> {
    let want = bearing_deg(field.0, field.1, dest.0, dest.1);
    let off = |b: f64| {
        let d = (b - want).rem_euclid(360.0);
        d.min(360.0 - d)
    };
    exits
        .iter()
        .filter(|e| {
            e.transition.strip_prefix("RW").is_some_and(|r| {
                r.eq_ignore_ascii_case(runway) || runway_matches(r.trim_end_matches('B'), runway)
            })
        })
        .map(|e| (e, bearing_deg(field.0, field.1, e.lat, e.lon)))
        .min_by(|a, b| off(a.1).total_cmp(&off(b.1)))
        .map(|(e, b)| (e.ident.clone(), b, want))
}

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
        .chain(
            legs.iter()
                .filter(|l| l.navaid.is_none() && !l.navaid_ident.is_empty())
                .map(|l| (l.navaid_ident.clone(), l.navaid_region.clone())),
        )
        .chain(
            legs.iter()
                .filter(|l| l.path.arc_center.is_none() && !l.path.arc_center_ident.is_empty())
                .map(|l| (l.path.arc_center_ident.clone(), l.path.arc_center_region.clone())),
        )
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
    for leg in legs.iter_mut() {
        if leg.navaid.is_some() || leg.navaid_ident.is_empty() {
            continue;
        }
        if let Some(&(lat, lon, _)) = found.get(&(leg.navaid_ident.clone(), leg.navaid_region.clone())) {
            leg.navaid = Some(Fix {
                ident: leg.navaid_ident.clone(),
                lat,
                lon,
            });
        }
    }
    for leg in legs.iter_mut() {
        if leg.path.arc_center.is_some() || leg.path.arc_center_ident.is_empty() {
            continue;
        }
        let key = (leg.path.arc_center_ident.clone(), leg.path.arc_center_region.clone());
        if let Some(&(lat, lon, _)) = found.get(&key) {
            leg.path.arc_center = Some(Fix {
                ident: key.0,
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
        let off = sim::angles::abs_diff_deg(brg, inbound_from);
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
            is_map: false,
            vpa_deg: None,
            navaid_ident: String::new(),
            navaid_region: String::new(),
            navaid: None,
            path: Default::default(),
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
        is_map: false,
        vpa_deg: None,
        navaid_ident: String::new(),
        navaid_region: String::new(),
        navaid: None,
        path: Default::default(),
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

/// A path under the real X-Plane install for the ignored real-install
/// tests: `XCOPILOT_XPLANE_ROOT`, else `$HOME/X-Plane 12` (the crew's
/// `tools::autoland::xplane_root`, which this crate cannot depend on).
#[cfg(test)]
pub(crate) fn test_xplane_path(rel: &str) -> String {
    let root = std::env::var("XCOPILOT_XPLANE_ROOT").unwrap_or_else(|_| {
        format!("{}/X-Plane 12", std::env::var("HOME").unwrap_or_default())
    });
    format!("{root}/{rel}")
}

#[cfg(test)]
#[path = "nav_tests.rs"]
mod tests;
