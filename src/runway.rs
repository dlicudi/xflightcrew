//! Runway data and geometry: the landing options parsed from apt.dat, the
//! pick by wind and track, the planned end, pavement widths and the
//! along/cross-track measures. Moved out of `autoland.rs` verbatim (refactor
//! audit 2026-10-04, phase 2); `autoland` re-exports it, so every
//! `autoland::Runway` / `autoland::nearby_runways` path still compiles.

use std::io::{BufRead, BufReader};

use crate::autoland::M_PER_NM;
use crate::nav::{bearing_deg, haversine_nm};

/// apt.dat surface codes we will put wheels on.
///
/// 1 and 2 are the legacy asphalt/concrete pair. X-Plane 12 replaced that pair
/// with a MATERIAL system numbered 20-38 and 50-57, and a whitelist of just
/// [1, 2] therefore hid **36% of every usable runway on earth** from the
/// emergency autoland: 4,870 of the 13,390 runways at or above our 1,200 m
/// minimum. Not obscure strips either — EGLL is surface 25 and 29, and PANC,
/// ENGM, EKCH, ESSA, KSEA, OMDB, EBBR, CYYC, EGAA and LTFM are all in the new
/// range. Heathrow was invisible to the autoland.
///
/// Southampton, 2026-07-30: EGHI 20 is surface 22, so it was never a candidate
/// at 2.16 nm and the search took Lasham's legacy asphalt 15.81 nm away. Note
/// this gate gutted BOTH entry points — `runway_by_id` filters on it too, so
/// even the circuit FORCING its own runway would have been rejected here and
/// fallen back to the same wrong field.
///
/// Still a whitelist and not a blacklist, because the soft codes must never
/// win by default: 3 turf, 4 dirt, 5 gravel, 12 dry lakebed, 13 water, 14
/// snow, 15 transparent. 39-49 are unused in the current global apt.dat and
/// are deliberately NOT accepted — an unknown code should not become a runway.
pub(crate) const fn is_hard_surface(s: i32) -> bool {
    matches!(s, 1 | 2 | 20..=38 | 50..=57)
}

/// One landing option: touch down AT `thr`, tracking `course_deg` toward the far end.
#[derive(Debug, Clone, PartialEq)]
pub struct Runway {
    pub icao: String,
    /// Runway identifier for this end (e.g. "27L").
    pub id: String,
    pub thr_lat: f64,
    pub thr_lon: f64,
    /// True course from this threshold toward the far end (degrees, 0-360).
    pub course_deg: f64,
    pub length_m: f64,
    /// apt.dat surface code (1/2 = asphalt/concrete = hard).
    pub surface: i32,
    /// Distance from the query position to this threshold, nm.
    pub dist_nm: f64,
    /// Field elevation, ft MSL (from the airport header) — the glidepath aim.
    pub elev_ft: f64,
}

impl Runway {
    pub(crate) fn hard(&self) -> bool {
        is_hard_surface(self.surface)
    }
}

/// Parse a type-`100` apt.dat line into its two landing ends (threshold lat/lon,
/// id, and the course to fly landing on that end). `None` if the line is malformed.
/// Pure — the streaming pass adds ICAO + distance. Returns (end1, end2) as
/// (id, thr_lat, thr_lon, course_deg), plus (length_m, surface).
pub(crate) fn parse_runway_100(line: &str) -> Option<(RunwayEnd, RunwayEnd, f64, i32)> {
    let f: Vec<&str> = line.split_whitespace().collect();
    if f.len() < 20 || f[0] != "100" {
        return None;
    }
    let surface: i32 = f[2].parse().ok()?;
    let lat1: f64 = f[9].parse().ok()?;
    let lon1: f64 = f[10].parse().ok()?;
    let lat2: f64 = f[18].parse().ok()?;
    let lon2: f64 = f[19].parse().ok()?;
    let length_m = haversine_nm(lat1, lon1, lat2, lon2) * M_PER_NM;
    let end1 = RunwayEnd {
        id: f[8].to_string(),
        lat: lat1,
        lon: lon1,
        course: bearing_deg(lat1, lon1, lat2, lon2),
    };
    let end2 = RunwayEnd {
        id: f[17].to_string(),
        lat: lat2,
        lon: lon2,
        course: bearing_deg(lat2, lon2, lat1, lon1),
    };
    Some((end1, end2, length_m, surface))
}

pub(crate) struct RunwayEnd {
    pub(crate) id: String,
    pub(crate) lat: f64,
    pub(crate) lon: f64,
    pub(crate) course: f64,
}

/// Candidate landing runways within `max_nm` of (lat, lon), nearest threshold
/// first. Streams the whole apt.dat once (cheap per line; ~a second on 363 MB — an
/// emergency arms this once). Later stages filter by suitability and plan the approach.
pub fn nearby_runways(
    apt_dat: &str,
    lat: f64,
    lon: f64,
    max_nm: f64,
) -> std::io::Result<Vec<Runway>> {
    let rdr = BufReader::new(std::fs::File::open(apt_dat)?);
    let mut out: Vec<Runway> = Vec::new();
    let mut icao = String::new();
    let mut elev_ft = 0.0;
    for line in rdr.lines() {
        let line = line?;
        // Cheap first-token check before any allocation-heavy parse.
        let mut toks = line.split_whitespace();
        let rec = toks.next().unwrap_or("");
        match rec {
            "1" | "16" | "17" => {
                // "1 <elev_ft> <tower> <default> <ICAO> <name...>"
                elev_ft = toks.next().and_then(|s| s.parse().ok()).unwrap_or(0.0);
                icao = line.split_whitespace().nth(4).unwrap_or("").to_string();
            }
            "100" => {
                let Some((e1, e2, length_m, surface)) = parse_runway_100(&line) else {
                    continue;
                };
                let width_m: Option<f64> =
                    line.split_whitespace().nth(1).and_then(|s| s.parse().ok());
                for e in [e1, e2] {
                    let dist_nm = haversine_nm(lat, lon, e.lat, e.lon);
                    if dist_nm <= max_nm {
                        if let Some(w) = width_m {
                            note_runway_width(&icao, &e.id, w);
                        }
                        out.push(Runway {
                            icao: icao.clone(),
                            id: e.id,
                            thr_lat: e.lat,
                            thr_lon: e.lon,
                            course_deg: e.course,
                            length_m,
                            surface,
                            dist_nm,
                            elev_ft,
                        });
                    }
                }
            }
            _ => {}
        }
    }
    out.sort_by(|a, b| {
        a.dist_nm
            .partial_cmp(&b.dist_nm)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    Ok(out)
}

/// Pick the best emergency-landing runway near (lat, lon) given the current track:
/// hard-surfaced and long enough, scored by distance plus a penalty for how much
/// heading change it needs (a runway roughly ahead beats one behind you). `None`
/// if nothing suitable is in range. `min_length_m` gates out too-short strips.
pub fn best_landing_runway(
    apt_dat: &str,
    lat: f64,
    lon: f64,
    track_deg: f64,
    max_nm: f64,
    min_length_m: f64,
) -> std::io::Result<Option<Runway>> {
    best_landing_runway_in_wind(
        apt_dat,
        lat,
        lon,
        track_deg,
        max_nm,
        min_length_m,
        &|_| None,
        None,
    )
}

/// The plan's DESTINATION is not a bonus, it is the answer: with a suitable
/// runway there inside the search, the self-pick chooses among the
/// destination's runways only. "Land the aircraft" with a plan to LIPK picked
/// Cervia's 11 over Forli's 30 at the same six miles (2026-09-25); an 8 nm
/// bonus still let Gibraltar's 09 beat Malaga, the destination, from 26 nm
/// (LEMG 2026-10-05). A nearer field is the emergency path (`nearest`, the
/// planner's `Nearest`), never a tiebreak.
/// The shortest strip the self-pick will take, metres: the airframe's landing
/// distance by the one landing rule ([`crate::aircraft::landing_needed_m`],
/// the planner's and the preflight's rule too), never under the 1,200 m every
/// airframe was held to. The flat 1,200 offered an A330 (1,448 m before any
/// margin) strips it cannot stop on; the planned path has checked the
/// distance at the arm since 2026-09-15 and the emergency path never did.
pub const SELF_PICK_MIN_LENGTH_M: f64 = 1200.0;
pub fn self_pick_min_length_m(landing_distance_m: Option<f64>) -> f64 {
    crate::aircraft::landing_needed_m(landing_distance_m, None, SELF_PICK_MIN_LENGTH_M)
}

/// Nautical miles of detour one knot of TAILWIND is worth: ten knots on the
/// tail costs five miles, so the into-wind end of the same runway wins
/// whichever threshold is nearer, and a far field has to be far indeed.
pub const TAILWIND_NM_PER_KT: f64 = 0.5;
/// The most a LONGER strip is worth, nm: decides between parallels (whose
/// thresholds are a few hundred metres apart) and nothing else.
pub const LENGTH_BONUS_MAX_NM: f64 = 0.5;

/// The self-pick's score for one runway end; lower wins. Distance, plus the
/// turn needed to point at the threshold (30 degrees a mile), plus the
/// tailwind, less a small bonus for pavement beyond the minimum. Pure.
pub fn landing_score(
    dist_nm: f64,
    turn_deg: f64,
    course_deg: f64,
    wind: Option<(f64, f64)>,
    length_m: f64,
    min_length_m: f64,
) -> f64 {
    let tailwind = wind.map_or(0.0, |(dir, kts)| {
        (-kts * (dir - course_deg).to_radians().cos()).max(0.0)
    });
    let spare = ((length_m - min_length_m) / 2000.0).clamp(0.0, 1.0) * LENGTH_BONUS_MAX_NM;
    dist_nm + turn_deg / 30.0 + tailwind * TAILWIND_NM_PER_KT - spare
}

/// [`best_landing_runway`] with the surface wind at each candidate's field
/// (`wind_at(icao)` = direction true, knots; `None` = unknown, no penalty).
pub fn best_landing_runway_in_wind(
    apt_dat: &str,
    lat: f64,
    lon: f64,
    track_deg: f64,
    max_nm: f64,
    min_length_m: f64,
    wind_at: &dyn Fn(&str) -> Option<(f64, f64)>,
    destination: Option<&str>,
) -> std::io::Result<Option<Runway>> {
    let mut cands = nearby_runways(apt_dat, lat, lon, max_nm)?;
    // Suitability: hard surface and long enough.
    cands.retain(|r| r.hard() && r.length_m >= min_length_m);
    // The destination's runways, when it has a suitable one in range.
    let at_dest = |r: &Runway| destination.is_some_and(|d| d.eq_ignore_ascii_case(r.icao.trim()));
    if cands.iter().any(at_dest) {
        cands.retain(at_dest);
    }
    // One wind lookup per field, not per end.
    let mut winds: std::collections::HashMap<String, Option<(f64, f64)>> = Default::default();
    for r in &cands {
        winds
            .entry(r.icao.clone())
            .or_insert_with(|| wind_at(r.icao.trim()));
    }
    let score = |r: &Runway| {
        let brg = bearing_deg(lat, lon, r.thr_lat, r.thr_lon);
        landing_score(
            r.dist_nm,
            crate::angles::abs_diff_deg(track_deg, brg),
            r.course_deg,
            winds.get(&r.icao).copied().flatten(),
            r.length_m,
            min_length_m,
        )
    };
    let best = cands.iter().min_by(|a, b| {
        score(a)
            .partial_cmp(&score(b))
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    Ok(best.cloned())
}

/// The runway an ARRIVAL chose, and the airport it belongs to.
///
/// The two travel together, and that is the entire point: **a runway ident is
/// not unique**. Handed only "24", the lookup below resolved it by proximity
/// and found EGPJ (Fife) instead of the planned EGPH (Edinburgh), nine miles
/// away across the Forth — a plan with a destination in it flew an ILS to a
/// different airport (2026-09-20). When a plan names where we are going, the
/// airport is not a search result.
#[derive(Debug, Clone, PartialEq)]
pub struct PlannedRunway {
    /// The runway END ("24", "27L").
    pub id: String,
    /// The airport that end belongs to. `None` only where nothing named one —
    /// a spoken "land the aircraft" with no plan — and only then may the
    /// ident be resolved by proximity.
    pub icao: Option<String>,
}

impl PlannedRunway {
    pub fn at(icao: &str, id: &str) -> Self {
        PlannedRunway {
            id: id.to_string(),
            icao: Some(icao.to_string()),
        }
    }
    /// An ident with no airport behind it — proximity is all there is.
    pub fn anywhere(id: &str) -> Self {
        PlannedRunway {
            id: id.to_string(),
            icao: None,
        }
    }
}

/// The specific runway END matching `id` within `max_nm`, AT `icao` when one
/// is given — used when the arrival has ALREADY chosen the runway (by wind)
/// and autoland must honour that end instead of re-picking by track. A named
/// field may deliberately be turf (LFOS 06); the unnamed/emergency lookup
/// remains hard-surface-only.
///
/// With `icao: None` this is the old nearest-ident behaviour, which is right
/// for a self-pick and wrong for anything that knows its destination.
pub fn runway_by_id(
    apt_dat: &str,
    lat: f64,
    lon: f64,
    id: &str,
    max_nm: f64,
    icao: Option<&str>,
) -> std::io::Result<Option<Runway>> {
    let want = id.trim();
    let at = icao.map(str::trim);
    // nearby_runways is already nearest-threshold-first.
    let cands = nearby_runways(apt_dat, lat, lon, max_nm)?;
    Ok(cands.into_iter().find(|r| {
        (at.is_some() || r.hard())
            && r.id.trim() == want
            && at.is_none_or(|a| r.icao.trim().eq_ignore_ascii_case(a))
    }))
}

/// The pavement widths read with the runways (apt.dat's `100` row, field 1),
/// keyed by field and end. A runway's width is scenery, not flight state: it
/// is noted as `nearby_runways` reads it, so the arm that found the runway
/// knows its width without a second pass over apt.dat.
static RUNWAY_WIDTHS: std::sync::Mutex<Option<std::collections::HashMap<(String, String), f64>>> =
    std::sync::Mutex::new(None);

fn width_key(icao: &str, id: &str) -> (String, String) {
    (
        icao.trim().to_ascii_uppercase(),
        id.trim().to_ascii_uppercase(),
    )
}

/// Note a runway end's pavement width, metres.
pub fn note_runway_width(icao: &str, id: &str, width_m: f64) {
    if !(width_m.is_finite() && width_m > 1.0) {
        return;
    }
    if let Ok(mut m) = RUNWAY_WIDTHS.lock() {
        m.get_or_insert_with(Default::default)
            .insert(width_key(icao, id), width_m);
    }
}

/// The pavement width of `rwy` when apt.dat gave one; `None` otherwise.
pub fn runway_width_m(rwy: &Runway) -> Option<f64> {
    RUNWAY_WIDTHS
        .lock()
        .ok()?
        .as_ref()?
        .get(&width_key(&rwy.icao, &rwy.id))
        .copied()
}

/// TRUE heading → MAGNETIC, using X-Plane's `magnetic_variation` dataref, whose
/// sign is positive-WEST: `magpsi = psi + variation`. Verified live at KPDK
/// (2026-07-14): psi 191.3 true, magpsi 196.8, variation +5.65. Subtracting
/// instead flew every autoland heading 2*var off (11 degrees at Atlanta) — the
/// whole cross-track cut vanished and the final tracked 0.4 nm left, parallel.
pub fn true_to_mag(true_deg: f64, xp_variation_deg: f64) -> f64 {
    (true_deg + xp_variation_deg).rem_euclid(360.0)
}

/// Signed distance (nm) along the runway centreline: positive = on the approach
/// side (short of the threshold, still to fly), negative = past the threshold
/// (departure side). Detects an overflight so the guidance goes around instead of
/// tracking the runway heading straight out the far end.
pub fn along_track_nm(lat: f64, lon: f64, rwy: &Runway) -> f64 {
    let brg_from_thr = bearing_deg(rwy.thr_lat, rwy.thr_lon, lat, lon);
    let dist = haversine_nm(rwy.thr_lat, rwy.thr_lon, lat, lon);
    let recip = (rwy.course_deg + 180.0) % 360.0;
    // Project threshold→aircraft onto the INBOUND (recip) direction: on the
    // approach side the aircraft bears ~recip from the threshold (cos ≈ +1),
    // past it the aircraft bears ~course_deg (cos ≈ -1).
    let ang = crate::angles::signed_diff_deg(brg_from_thr, recip).to_radians();
    dist * ang.cos()
}

/// Signed cross-track distance (nm) from the runway's extended centreline —
/// positive = right of the inbound course. Used to steer back to the centreline.
pub fn cross_track_nm(lat: f64, lon: f64, rwy: &Runway) -> f64 {
    let brg_from_thr = bearing_deg(rwy.thr_lat, rwy.thr_lon, lat, lon);
    let dist = haversine_nm(rwy.thr_lat, rwy.thr_lon, lat, lon);
    // Perpendicular offset from the extended centreline, signed right-positive
    // relative to the INBOUND landing course (the direction of flight on final).
    let ang = crate::angles::signed_diff_deg(brg_from_thr, rwy.course_deg).to_radians();
    dist * ang.sin()
}
