//! Talk-down: synthesized visual-approach guidance for any runway, no
//! procedures required — the approach LERL doesn't have, built from geometry
//! the sim does have (apt.dat runway thresholds via [`crate::autoland`]).
//!
//! The pilot arms it by voice (`visual_approach`, "talk me down runway 28");
//! the monitor's poll loop calls [`tick`] ~3 Hz, which speaks PAR-style
//! deviation callouts ("On course, slightly high. Three miles.") on a cadence
//! that tightens as the runway gets close. Advisory only — nothing here ever
//! touches a control; the pilot flies, the copilot narrates.

use std::sync::Mutex;

/// The armed approach: land AT the threshold, tracking `course_deg`.
#[derive(Debug, Clone)]
pub struct ApproachPath {
    pub icao: String,
    pub runway_id: String,
    pub thr_lat: f64,
    pub thr_lon: f64,
    /// Final-approach course, degrees true (threshold toward the far end).
    pub course_deg: f64,
    /// Field elevation, ft MSL.
    pub elev_ft: f64,
}

/// The armed talk-down, if any. Set by the `visual_approach` tool, read by the
/// monitor's tick, cleared on touchdown / cancel / flying away.
pub static GUIDE: Mutex<Option<ApproachPath>> = Mutex::new(None);

pub fn arm(path: ApproachPath) {
    *GUIDE.lock().unwrap_or_else(|e| e.into_inner()) = Some(path);
}

pub fn cancel() -> bool {
    GUIDE.lock().unwrap_or_else(|e| e.into_inner()).take().is_some()
}

pub fn armed() -> Option<ApproachPath> {
    GUIDE.lock().unwrap_or_else(|e| e.into_inner()).clone()
}

/// Glidepath: 3° ≈ 300 ft/nm, aimed 50 ft over the threshold.
const FT_PER_NM: f64 = 300.0;
const THRESHOLD_CROSSING_FT: f64 = 50.0;

/// Where the aircraft sits relative to the approach: distance to the threshold,
/// cross-track (positive = LEFT of centerline, i.e. correct right), and height
/// above the glidepath (positive = high).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Deviations {
    pub dist_nm: f64,
    pub xtk_nm: f64,
    pub vdev_ft: f64,
    /// True when ahead of the threshold on the approach side (not past/beyond).
    pub on_approach_side: bool,
}

pub fn deviations(path: &ApproachPath, lat: f64, lon: f64, alt_ft: f64) -> Deviations {
    let dist_nm = crate::haversine_nm(lat, lon, path.thr_lat, path.thr_lon);
    // Bearing aircraft → threshold vs the final-approach course. On centerline
    // they're equal; if the threshold bears RIGHT of course the aircraft sits
    // LEFT of the extended centerline.
    let brg = bearing_deg(lat, lon, path.thr_lat, path.thr_lon);
    let dev = angle_diff_deg(brg, path.course_deg);
    let xtk_nm = dist_nm * dev.to_radians().sin();
    let along_nm = dist_nm * dev.to_radians().cos();
    let ideal = path.elev_ft + THRESHOLD_CROSSING_FT + along_nm.max(0.0) * FT_PER_NM;
    Deviations {
        dist_nm,
        xtk_nm,
        vdev_ft: alt_ft - ideal,
        on_approach_side: along_nm > 0.05,
    }
}

/// One PAR-style callout for the current deviations, or `None` when there's
/// nothing worth saying (past the threshold — the flare belongs to the pilot).
pub fn callout(d: &Deviations) -> Option<String> {
    if !d.on_approach_side {
        return None;
    }
    if d.dist_nm < 0.4 {
        return Some("Over the threshold. Yours.".to_string());
    }
    let lateral = if d.xtk_nm.abs() < 0.08 {
        "On course".to_string()
    } else {
        let side = if d.xtk_nm > 0.0 { "left" } else { "right" };
        let fix = if d.xtk_nm > 0.0 { "right" } else { "left" };
        if d.xtk_nm.abs() < 0.25 {
            format!("Slightly {side}, come {fix}")
        } else {
            format!("Well {side} of course, correct {fix}")
        }
    };
    let vertical = if d.vdev_ft.abs() < 75.0 {
        "on glidepath".to_string()
    } else {
        let sense = if d.vdev_ft > 0.0 { "high" } else { "low" };
        let amount = ((d.vdev_ft.abs() / 50.0).round() * 50.0) as i64;
        if d.vdev_ft.abs() < 250.0 {
            format!("{amount} {sense}")
        } else {
            format!("well {sense}")
        }
    };
    let dist = if d.dist_nm >= 1.5 {
        format!(" {:.0} miles.", d.dist_nm)
    } else if d.dist_nm >= 0.75 {
        " One mile.".to_string()
    } else {
        String::new() // short final: keep it terse
    };
    Some(format!("{lateral}, {vertical}.{dist}"))
}

/// Callout cadence by distance — tightens on final, like a PAR controller.
pub fn cadence_secs(dist_nm: f64) -> f64 {
    if dist_nm > 5.0 {
        20.0
    } else if dist_nm > 2.0 {
        10.0
    } else {
        6.0
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    /// LERL runway 28: threshold at the east end, course ~283° true — the field
    /// this feature was born for (no procedures of any kind in the nav data).
    fn lerl_28() -> ApproachPath {
        ApproachPath {
            icao: "LERL".into(),
            runway_id: "28".into(),
            thr_lat: 38.8525829,
            thr_lon: -3.9475728,
            course_deg: 283.0,
            elev_ft: 2084.0,
        }
    }

    /// A point `nm` out on the extended centerline (reciprocal of the course).
    fn on_final(path: &ApproachPath, nm: f64) -> (f64, f64) {
        let recip = (path.course_deg + 180.0).to_radians();
        let dlat = nm / 60.0 * recip.cos();
        let dlon = nm / 60.0 * recip.sin() / path.thr_lat.to_radians().cos();
        (path.thr_lat + dlat, path.thr_lon + dlon)
    }

    #[test]
    fn on_profile_reads_on_course_on_glidepath() {
        let p = lerl_28();
        let (lat, lon) = on_final(&p, 3.0);
        let ideal = 2084.0 + 50.0 + 3.0 * 300.0; // 3,034
        let d = deviations(&p, lat, lon, ideal);
        assert!(d.on_approach_side);
        assert!((d.dist_nm - 3.0).abs() < 0.1, "{}", d.dist_nm);
        assert!(d.xtk_nm.abs() < 0.05, "xtk {}", d.xtk_nm);
        assert!(d.vdev_ft.abs() < 30.0, "vdev {}", d.vdev_ft);
        let c = callout(&d).unwrap();
        assert!(c.starts_with("On course, on glidepath"), "{c}");
        assert!(c.contains("3 miles"), "{c}");
    }

    #[test]
    fn high_and_offset_reads_corrections() {
        let p = lerl_28();
        let (lat, lon) = on_final(&p, 3.0);
        // 400 ft high → "well high"; shift ~0.15 nm left of centerline
        // (aircraft north of the line for a westbound approach = right side...
        // verified by sign convention below).
        let d = Deviations { dist_nm: 3.0, xtk_nm: 0.15, vdev_ft: 400.0, on_approach_side: true };
        let c = callout(&d).unwrap();
        assert!(c.contains("Slightly left, come right"), "{c}");
        assert!(c.contains("well high"), "{c}");
        // Mild low reads a number rounded to 50.
        let d2 = deviations(&p, lat, lon, 2084.0 + 50.0 + 900.0 - 120.0);
        assert!(d2.vdev_ft < -75.0);
        let c2 = callout(&d2).unwrap();
        assert!(c2.contains("100 low") || c2.contains("150 low"), "{c2}");
    }

    #[test]
    fn short_final_goes_terse_then_hands_over() {
        let d = Deviations { dist_nm: 0.6, xtk_nm: 0.0, vdev_ft: 10.0, on_approach_side: true };
        let c = callout(&d).unwrap();
        assert_eq!(c, "On course, on glidepath.");
        let over = Deviations { dist_nm: 0.3, xtk_nm: 0.0, vdev_ft: 0.0, on_approach_side: true };
        assert_eq!(callout(&over).unwrap(), "Over the threshold. Yours.");
        // Beyond/behind the threshold: silence, the flare is the pilot's.
        let past = Deviations { dist_nm: 0.2, xtk_nm: 0.0, vdev_ft: 0.0, on_approach_side: false };
        assert!(callout(&past).is_none());
    }

    #[test]
    fn cadence_tightens_on_final() {
        assert_eq!(cadence_secs(8.0), 20.0);
        assert_eq!(cadence_secs(3.0), 10.0);
        assert_eq!(cadence_secs(1.0), 6.0);
    }

    #[test]
    fn arm_and_cancel_roundtrip() {
        arm(lerl_28());
        assert_eq!(armed().unwrap().runway_id, "28");
        assert!(cancel());
        assert!(armed().is_none());
        assert!(!cancel(), "second cancel is a no-op");
    }
}
