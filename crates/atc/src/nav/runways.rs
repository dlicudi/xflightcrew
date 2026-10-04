//! Runway choice and runway geometry: wind, parallels, thresholds.

use super::*;

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
