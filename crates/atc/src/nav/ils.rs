//! ILS localisers from `earth_nav.dat` and their CIFP charts.

use super::*;

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
