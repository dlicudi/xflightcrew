//! Airports, runways and comm frequencies from apt.dat.

use super::*;

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
pub(super) fn parse_runway_row<'a>(fields: impl Iterator<Item = &'a str>) -> Option<Runway> {
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
pub(super) fn parse_comm_row<'a>(code: &str, mut fields: impl Iterator<Item = &'a str>) -> Option<AtcFreq> {
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
