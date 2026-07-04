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

/// An airport as apt.dat knows it: the header's field elevation plus runways.
#[derive(Debug, Clone)]
pub struct AirportInfo {
    /// Field elevation, feet MSL — the authoritative figure (a flight plan's
    /// destination-waypoint altitude is a *crossing* altitude, seen live 2,500 ft
    /// high over LERL).
    pub elev_ft: f64,
    pub runways: Vec<Runway>,
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
    let mut in_airport = false;
    let mut elev_ft = 0.0_f64;
    // Reused byte buffer + a cheap prefix test before any splitting: the file is
    // ~400 MB and 13M lines, and a String-per-line scan is unusable in a debug
    // build. Only `1 `/`16 `/`17 ` (headers) and `100 ` (runways) lines matter.
    let mut buf = Vec::with_capacity(256);
    loop {
        buf.clear();
        if reader.read_until(b'\n', &mut buf).unwrap_or(0) == 0 {
            break;
        }
        let interesting = matches!(buf.first(), Some(b'1')); // covers 1, 16, 17, 100
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
                }
            }
            Some("100") if in_airport => {
                let f: Vec<&str> = f.collect(); // fields after the row code
                if f.len() >= 19 {
                    if let (Ok(lat_a), Ok(lon_a), Ok(lat_b), Ok(lon_b)) =
                        (f[8].parse(), f[9].parse(), f[17].parse(), f[18].parse())
                    {
                        runways.push(Runway {
                            ident_a: f[7].to_string(),
                            ident_b: f[16].to_string(),
                            lat_a,
                            lon_a,
                            lat_b,
                            lon_b,
                        });
                    }
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
    Ok(Some(AirportInfo { elev_ft, runways }))
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
        writeln!(f, "1   1027 0 0 0MU1 Sunderland").unwrap();
        writeln!(f, "100 22.86 3 0 0.25 0 0 0  09 37.2471001 -94.1770610 0 0 1 0 0 0 27 37.2469784 -94.1700162 0 0 1 0 0 0").unwrap();
        f.flush().unwrap();

        // Field elevation comes off the header row (LEAL sits at 142 ft).
        let apt = find_airport(path.to_str().unwrap(), "LEAL").unwrap().expect("found");
        assert_eq!(apt.elev_ft, 142.0);
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
