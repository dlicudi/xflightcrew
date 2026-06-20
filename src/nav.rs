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
        if best.as_ref().map_or(true, |b| d < b.distance_nm) {
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
        if best.as_ref().map_or(true, |(_, bd)| d < *bd) {
            best = Some((f[8].to_string(), d));
        }
    }
    Ok(best)
}

/// Leading digits of a runway designator ("28L" → 28, "07" → 7).
fn runway_number(s: &str) -> Option<u32> {
    let digits: String = s.chars().take_while(|c| c.is_ascii_digit()).collect();
    digits.parse().ok()
}

/// Great-circle distance in nautical miles.
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
