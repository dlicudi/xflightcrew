//! Where the route tables' fields are and what time it is there
//! (2026-10-13): the airport reference points of every field a route table
//! ([`crate::routes`]) names, read once from X-Plane 12's Global Airports
//! apt.dat (`1302 datum_lat/lon`, else the first runway's middle) and
//! rounded to a hundredth of a degree, and each country's clock. The
//! timetable builds a route's day from them ([`crate::rotation`]): a
//! flight from Gatwick reaches Malaga a block time after it left, an hour
//! later on the clock.
//!
//! A table, not an apt.dat read at runtime: the timetable stays pure and
//! the same on every machine. `every_route_end_has_a_place` keeps it whole.

/// (ICAO, latitude, longitude), degrees.
const PLACES: &[(&str, f64, f64)] = &[
    ("BIKF", 63.98, -22.61),
    ("EBBR", 50.90, 4.48),
    ("EBCI", 50.46, 4.45),
    ("EDDB", 52.36, 13.50),
    ("EDDF", 50.03, 8.57),
    ("EDDG", 52.13, 7.68),
    ("EDDH", 53.63, 9.99),
    ("EDDK", 50.87, 7.14),
    ("EDDL", 51.28, 6.76),
    ("EDDM", 48.35, 11.79),
    ("EDDN", 49.50, 11.08),
    ("EDDS", 48.69, 9.22),
    ("EDDV", 52.46, 9.68),
    ("EDDW", 53.05, 8.79),
    ("EDFH", 49.95, 7.26),
    ("EDHL", 53.81, 10.72),
    ("EDJA", 47.99, 10.24),
    ("EDLV", 51.60, 6.14),
    ("EDSB", 48.78, 8.08),
    ("EFHK", 60.32, 24.96),
    ("EGAA", 54.66, -6.22),
    ("EGBB", 52.45, -1.75),
    ("EGCC", 53.35, -2.27),
    ("EGFF", 51.40, -3.34),
    ("EGGD", 51.38, -2.72),
    ("EGGP", 53.33, -2.85),
    ("EGGW", 51.87, -0.37),
    ("EGHH", 50.78, -1.84),
    ("EGHQ", 50.44, -5.00),
    ("EGKK", 51.15, -0.19),
    ("EGLL", 51.48, -0.46),
    ("EGMC", 51.57, 0.69),
    ("EGNM", 53.87, -1.66),
    ("EGNT", 55.04, -1.69),
    ("EGNV", 54.51, -1.43),
    ("EGNX", 52.83, -1.33),
    ("EGPF", 55.87, -4.43),
    ("EGPH", 55.95, -3.37),
    ("EGPK", 55.51, -4.59),
    ("EGSS", 51.88, 0.23),
    ("EGTE", 50.73, -3.41),
    ("EHAM", 52.31, 4.76),
    ("EHEH", 51.45, 5.37),
    ("EHRD", 51.96, 4.44),
    ("EICK", 51.84, -8.49),
    ("EIDW", 53.42, -6.27),
    ("EIKN", 53.91, -8.82),
    ("EINN", 52.70, -8.92),
    ("EKAH", 56.30, 10.62),
    ("EKBI", 55.74, 9.15),
    ("EKCH", 55.62, 12.66),
    ("EKYT", 57.09, 9.85),
    ("ENBR", 60.29, 5.22),
    ("ENGM", 60.20, 11.08),
    ("ENHD", 59.34, 5.21),
    ("ENTO", 59.19, 10.26),
    ("ENVA", 63.46, 10.92),
    ("ENZV", 58.88, 5.64),
    ("EPGD", 54.38, 18.47),
    ("EPKK", 50.08, 19.78),
    ("EPKT", 50.47, 19.08),
    ("EPLL", 51.72, 19.40),
    ("EPPO", 52.42, 16.83),
    ("EPWA", 52.17, 20.97),
    ("EPWR", 51.10, 16.89),
    ("ESGG", 57.66, 12.29),
    ("ESKN", 58.79, 16.90),
    ("ESSA", 59.65, 17.92),
    ("EVRA", 56.92, 23.97),
    ("EYKA", 54.96, 24.08),
    ("EYVI", 54.64, 25.29),
    ("GCFV", 28.45, -13.86),
    ("GCLP", 27.93, -15.39),
    ("GCRR", 28.95, -13.61),
    ("GCXO", 28.48, -16.34),
    ("GEML", 35.28, -2.96),
    ("GMFF", 33.93, -4.98),
    ("GMME", 34.05, -6.75),
    ("GMMX", 31.61, -8.04),
    ("GMTN", 35.59, -5.32),
    ("GMTT", 35.73, -5.92),
    ("KEWR", 40.69, -74.17),
    ("LATI", 41.41, 19.72),
    ("LBSF", 42.70, 23.41),
    ("LDZA", 45.74, 16.07),
    ("LEBB", 43.30, -2.91),
    ("LEBL", 41.30, 2.08),
    ("LEIB", 38.87, 1.37),
    ("LEMD", 40.47, -3.56),
    ("LEMG", 36.67, -4.50),
    ("LEPA", 39.55, 2.74),
    ("LEST", 42.90, -8.42),
    ("LEVC", 39.49, -0.48),
    ("LFBD", 44.83, -0.72),
    ("LFLL", 45.73, 5.08),
    ("LFML", 43.44, 5.21),
    ("LFMN", 43.67, 7.21),
    ("LFMT", 43.58, 3.96),
    ("LFPG", 49.01, 2.55),
    ("LFPO", 48.72, 2.38),
    ("LFRS", 47.16, -1.61),
    ("LFSB", 47.59, 7.53),
    ("LGAV", 37.94, 23.94),
    ("LHBP", 47.44, 19.26),
    ("LIMC", 45.63, 8.72),
    ("LIME", 45.67, 9.70),
    ("LIMF", 45.20, 7.65),
    ("LIPE", 44.53, 11.30),
    ("LIPH", 45.65, 12.20),
    ("LIPZ", 45.51, 12.35),
    ("LIRF", 41.80, 12.24),
    ("LIRN", 40.88, 14.29),
    ("LIRP", 43.68, 10.40),
    ("LKMT", 49.70, 18.11),
    ("LKPR", 50.10, 14.26),
    ("LKTB", 49.15, 16.69),
    ("LOWW", 48.11, 16.57),
    ("LPPR", 41.24, -8.68),
    ("LPPT", 38.77, -9.13),
    ("LRCL", 46.79, 23.69),
    ("LROP", 44.57, 26.09),
    ("LSGG", 46.24, 6.11),
    ("LSZH", 47.46, 8.55),
    ("LTFM", 41.28, 28.75),
    ("LXGB", 36.15, -5.35),
    ("LZIB", 48.17, 17.21),
    ("LZKZ", 48.66, 21.24),
    ("OMAA", 24.43, 54.65),
    ("OTHH", 25.27, 51.61),
];

/// Where `icao` is (latitude, longitude), if a route table names it.
pub fn position(icao: &str) -> Option<(f64, f64)> {
    PLACES
        .iter()
        .find(|p| p.0.eq_ignore_ascii_case(icao))
        .map(|p| (p.1, p.2))
}

/// The clock at `icao` by its ICAO country prefix: standard time's offset
/// from UTC, hours, and whether it keeps summer time with the EU's dates
/// (the United States' differ by a few weeks, near enough for a
/// timetable). Morocco keeps UTC+1 all year, Iceland UTC, Turkey and the
/// Gulf their own; everything else in the tables is the EU's three zones.
pub fn clock(icao: &str) -> (f64, bool) {
    let p2 = icao.get(..2).unwrap_or("");
    match p2 {
        "BI" => (0.0, false),
        "GM" => (1.0, false),
        "LT" | "OT" => (3.0, false),
        "OM" => (4.0, false),
        "EG" | "EI" | "LP" | "GC" => (0.0, true),
        "EF" | "LG" | "LR" | "LB" | "EV" | "EY" => (2.0, true),
        _ if icao.starts_with('K') => (-5.0, true),
        _ => (1.0, true),
    }
}

/// The offset from UTC at `icao` on a day `summer` or not, hours.
pub fn utc_offset_h(icao: &str, summer: bool) -> f64 {
    let (std, dst) = clock(icao);
    std + if dst && summer { 1.0 } else { 0.0 }
}

/// Great circle distance between two places, nautical miles.
pub fn dist_nm(a: (f64, f64), b: (f64, f64)) -> f64 {
    let (la, lb) = (a.0.to_radians(), b.0.to_radians());
    let dlat = lb - la;
    let dlon = (b.1 - a.1).to_radians();
    let h = (dlat / 2.0).sin().powi(2) + la.cos() * lb.cos() * (dlon / 2.0).sin().powi(2);
    2.0 * h.sqrt().min(1.0).asin() * 3440.065
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every end a route table names has its place, and every field with a
    /// table too: a route with none would fly on a guessed block time.
    #[test]
    fn every_route_end_has_a_place() {
        for f in crate::routes::fields() {
            assert!(position(f.icao).is_some(), "{}", f.icao);
            for r in f.rows {
                assert!(position(r.other).is_some(), "{} {}", f.icao, r.other);
            }
        }
    }

    /// Clocks: London an hour behind Malaga in summer and winter, Helsinki
    /// an hour ahead, Marrakesh two behind in summer and one in winter.
    #[test]
    fn the_clocks_are_the_countries() {
        let d = |a: &str, b: &str, s: bool| utc_offset_h(a, s) - utc_offset_h(b, s);
        assert_eq!(d("EGKK", "LEMG", true), -1.0);
        assert_eq!(d("EGKK", "LEMG", false), -1.0);
        assert_eq!(d("EFHK", "LEMG", true), 1.0);
        assert_eq!(d("GMMX", "LEMG", true), -1.0);
        assert_eq!(d("GMMX", "LEMG", false), 0.0);
        assert_eq!(d("OMAA", "LEMG", true), 2.0);
        let gatwick = dist_nm(position("LEMG").unwrap(), position("EGKK").unwrap());
        assert!((850.0..950.0).contains(&gatwick), "{gatwick}");
    }
}
