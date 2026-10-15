//! Where the route tables' fields are and what time it is there
//! (2026-10-13): every field a route ([`crate::routes`]) names, its airport
//! reference point and its clock, read from the route data
//! (`routes/fields.yaml`, since 2026-10-15; a Rust table and a clock by
//! country prefix until then). The timetable builds a route's day from
//! them ([`crate::rotation`]): a flight from Gatwick reaches Malaga a block
//! time after it left, an hour later on the clock.
//!
//! Data, never an apt.dat read at runtime: the timetable stays pure and the
//! same on every machine. `every_route_end_has_a_place` keeps it whole.

/// Where `icao` is (latitude, longitude), if a route names it.
pub fn position(icao: &str) -> Option<(f64, f64)> {
    crate::routes::place(icao).map(|p| (p.lat, p.lon))
}

/// The offset from UTC at `icao` on a day `summer` or not, hours: its
/// standard time, plus an hour in the summer where it keeps summer time
/// (on the EU's dates; the United States' differ by a few weeks, near
/// enough for a timetable). A field no route names keeps UTC.
pub fn utc_offset_h(icao: &str, summer: bool) -> f64 {
    crate::routes::place(icao).map_or(0.0, |p| {
        p.utc_offset_h + if p.dst && summer { 1.0 } else { 0.0 }
    })
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
