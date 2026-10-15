//! Approach fix resolution and reconstruction.

use super::*;

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

/// 3° glidepath, feet per nm (tan 3° × 6076).
pub const GLIDE_FT_PER_NM: f64 = 318.0;

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

/// How far a DME fix's bearing from the threshold may stray from the runway's
/// reciprocal course and still count as ON the final approach course.
pub(super) const RECON_ALIGN_DEG: f64 = 10.0;
