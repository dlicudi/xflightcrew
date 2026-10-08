//! The active field's runways: occupancy, ends, finals, departure climb.

use super::*;

/// The active field's runway geometry, published by [`Station::install`] for
/// the runway-incursion watch.
pub(super) static ACTIVE_RUNWAYS: Mutex<Vec<nav::Runway>> = Mutex::new(Vec::new());

/// The active field's runways, for the party line (`field_runways`).
pub(crate) fn active_runways() -> Vec<nav::Runway> {
    ACTIVE_RUNWAYS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()
}

/// Lateral capture around a runway centerline when the 100-row carried no
/// width (the one home is nav's: [`nav::RUNWAY_HALF_WIDTH_M`]).
pub(super) const RUNWAY_HALF_WIDTH_M: f64 = nav::RUNWAY_HALF_WIDTH_M;
/// Slack past the real pavement edge ([`nav::RUNWAY_EDGE_SLACK_M`]).
#[cfg_attr(not(test), allow(dead_code))] // strip.rs's tests read it
pub(super) const RUNWAY_EDGE_SLACK_M: f64 = nav::RUNWAY_EDGE_SLACK_M;

/// Is this position ON one of the active field's runways? Point-to-segment in
/// a local flat-earth projection (exact enough over a 3 km runway). Returns
/// the spoken ident of the nearer end, for the blackbox and the call.
pub fn on_active_runway(lat: f64, lon: f64) -> Option<String> {
    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
    runways.iter().find_map(|r| on_runway(r, lat, lon))
}

/// [`on_active_runway`], naming the END the aircraft is pointing down when
/// its true heading is known, else the nearer end. The nearer end is the
/// wrong name for an aircraft that has rolled past the middle: a 737 landed
/// LEMG 12, stopped heading 117 in the far half, and was told "you landed on
/// runway three zero" (2026-09-26).
pub fn on_active_runway_facing(lat: f64, lon: f64, heading_true: Option<f64>) -> Option<String> {
    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
    runways.iter().find_map(|r| {
        let nearer = on_runway(r, lat, lon)?;
        Some(match heading_true.filter(|h| h.is_finite()) {
            Some(h) => {
                let off_a = sim::angles::abs_diff_deg(h, r.heading_a_deg());
                let off_b = sim::angles::abs_diff_deg(h, r.heading_b_deg());
                if off_a <= off_b { r.ident_a.clone() } else { r.ident_b.clone() }
            }
            None => nearer,
        })
    })
}

/// [`on_active_runway`] with "unknown" kept apart from "no": `None` when no
/// field's runways are published (no station yet), never a guessed `false`.
pub fn on_active_runway_known(lat: f64, lon: f64) -> Option<bool> {
    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
    if runways.is_empty() {
        return None;
    }
    Some(runways.iter().any(|r| on_runway(r, lat, lon).is_some()))
}

/// The strip under this position, keyed by its `ident_a` (both ends of one
/// pavement are one strip), `None` off every active runway.
pub(super) fn strip_under(lat: f64, lon: f64) -> Option<String> {
    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
    runways
        .iter()
        .find(|r| on_runway(r, lat, lon).is_some())
        .map(|r| r.ident_a.clone())
}

/// Is this position ON one strip? The shared segment test behind
/// [`on_active_runway`] and the per-runway traffic watches. Returns the
/// spoken-able ident of the nearer end.
pub(super) fn on_runway(r: &nav::Runway, lat: f64, lon: f64) -> Option<String> {
    // nav's test, shared with our traffic's runway crossings.
    nav::on_strip(r, lat, lon)
}

/// The strip a runway END belongs to: landing 27L and departing 09R share
/// the same pavement, so conflict checks compare STRIPS, not end names.
pub(super) fn strip<'a>(runways: &'a [nav::Runway], ident: &str) -> Option<&'a nav::Runway> {
    runways
        .iter()
        .find(|r| r.ident_a == ident || r.ident_b == ident)
}

/// The heading of one runway END, from its strip's geometry.
pub(super) fn end_heading(runways: &[nav::Runway], ident: &str) -> Option<f64> {
    let r = strip(runways, ident)?;
    Some(if r.ident_a == ident {
        r.heading_a_deg()
    } else {
        r.heading_b_deg()
    })
}

/// Two runway ends point against each other: the reciprocal of one strip,
/// or a near-parallel's opposite end (LEMG 31 against 12). More than 120
/// degrees apart; a crossing strip is not opposed. Unknown ends never are.
pub(super) fn ends_oppose(runways: &[nav::Runway], a: &str, b: &str) -> bool {
    match (end_heading(runways, a), end_heading(runways, b)) {
        (Some(ha), Some(hb)) => sim::angles::abs_diff_deg(ha, hb) > 120.0,
        _ => false,
    }
}

/// The arrivals in one strip's final corridor: nearest distance, its spoken
/// callsign (names the condition of a conditional line-up), and how many are
/// in the stream (a conditional is only legal against exactly one).
pub struct FinalTraffic {
    pub nm: f64,
    pub callsign: Option<String>,
    pub count: usize,
    /// Every arrival in the corridor, by its raw callsign: the traffic a
    /// held departure was held FOR ([`TakeoffHold::held_for`]).
    pub ghosts: Vec<String>,
}

/// Lateral capture around the extended centerline: half of the tightest
/// real parallel spacing (Heathrow's pair is ~1,400 m apart) with room
/// for a wobbly final.
pub(super) const CORRIDOR_HALF_M: f64 = 500.0;
/// How far out an arrival still owns the runway.
pub(super) const FINAL_NM: f64 = 4.0;
/// A target climbing faster than this is a DEPARTURE leaving, not an
/// arrival: a real climb-out shows +500 fpm or more, an arrival on final
/// descends, and a level-off or an unfed VS reads ~0 and stays counted
/// (fail toward caution). A ghost lifting off Malaga runway 30 read as
/// "traffic on short final" to the runway 12 incursion watch, 2026-07-12.
pub(super) const CLIMBOUT_FPM: f64 = 300.0;

/// The climb departure gives a check-in: the one it announced, never above
/// the FILED cruise (`cap`, feet). EGKK-EGBB (sortie 1790601822-full_auto,
/// 2026-09-28, #155): filed FL180, the crew announced "climbing flight level
/// two two zero" (HEME1B's HEMEL window on its legs) and departure cleared
/// FL220. A level departure cannot hear, or no cap, stands as announced. The
/// cap is said as the announcement said its level: a flight level at or
/// above the field's transition level, or when the field has none and the
/// crew said one.
pub fn capped_departure_climb(
    announced: &str,
    cap: Option<i64>,
    transition_level_ft: Option<f64>,
) -> String {
    let (Some(heard), Some(cap)) = (
        tower::heard_altitude_ft(announced),
        cap.filter(|c| *c > 0),
    ) else {
        return announced.to_string();
    };
    if heard <= cap {
        return announced.to_string();
    }
    let level = atis::is_flight_level(cap as f64, transition_level_ft)
        || (transition_level_ft.is_none() && announced.to_lowercase().contains("flight level"));
    sim::blackbox::record(
        "atc",
        serde_json::json!({"event": "climb_capped", "announced_ft": heard, "cleared_ft": cap}),
    );
    if level {
        atis::spoken_flight_level(cap as f64)
    } else {
        atis::spoken_alt(cap as f64)
    }
}

/// The altitude a departure check-in announces ("passing one thousand
/// climbing five thousand" gives "five thousand"), for the controller to
/// clear. Whatever follows the last "climbing", trimmed of punctuation.
pub fn climb_announced(transcript: &str) -> Option<String> {
    let t = transcript.trim_end_matches(['.', ',', ' ']);
    let i = t.to_lowercase().rfind("climbing")?;
    let rest = t[i + "climbing".len()..].trim();
    let rest = rest.trim_start_matches("to ").trim();
    (!rest.is_empty()).then(|| rest.to_string())
}

/// The runway END the nearest arrival on final is landing on: on the
/// extended centreline beyond threshold A it is landing on A, beyond B on
/// B. Same corridor and altitude band as [`final_traffic_for`].
pub fn arrival_end(targets: &[TcasTarget], runways: &[nav::Runway], field_elev_ft: f64) -> Option<String> {
    let mut best: Option<(f64, String)> = None;
    for rwy in runways {
        let m_per_deg_lat = 111_320.0;
        let m_per_deg_lon = 111_320.0 * rwy.lat_a.to_radians().cos();
        let (bx, by) = (
            (rwy.lon_b - rwy.lon_a) * m_per_deg_lon,
            (rwy.lat_b - rwy.lat_a) * m_per_deg_lat,
        );
        let len2 = bx * bx + by * by;
        if len2 < 1.0 {
            continue;
        }
        let len_m = len2.sqrt();
        for t in targets
            .iter()
            .filter(|t| !t.on_ground)
            .filter(|t| (30.0..1500.0).contains(&(t.ele_ft - field_elev_ft)))
            .filter(|t| t.vs_fpm <= CLIMBOUT_FPM)
            .filter(|t| !on_independent_final(t, rwy, runways))
        {
            let (px, py) = (
                (t.lon - rwy.lon_a) * m_per_deg_lon,
                (t.lat - rwy.lat_a) * m_per_deg_lat,
            );
            let along = (px * bx + py * by) / len2;
            let cross_m = (px * by - py * bx).abs() / len_m;
            let beyond = FINAL_NM * 1852.0 / len_m;
            if cross_m > CORRIDOR_HALF_M {
                continue;
            }
            let (end, dist_m) = if along < 0.0 && along >= -beyond {
                (&rwy.ident_a, (px * px + py * py).sqrt())
            } else if along > 1.0 && along <= 1.0 + beyond {
                let (qx, qy) = (px - bx, py - by);
                (&rwy.ident_b, (qx * qx + qy * qy).sqrt())
            } else {
                continue;
            };
            if best.as_ref().is_none_or(|(d, _)| dist_m < *d) {
                best = Some((dist_m, end.clone()));
            }
        }
    }
    // A close parallel's corridor is the main runway's too: the stream
    // lands on the runway the field works (`nav::close_parallel_main`).
    best.map(|(_, e)| nav::close_parallel_main(runways, &e))
}

/// Live traffic on final for THIS strip: airborne, low over the field,
/// inside the runway's extended-centerline corridor (either direction), and
/// not climbing away — a departure climbing through the corridor is leaving,
/// not arriving. Distances are to the nearer threshold, in nm. The
/// runway-blind version of this check held a Heathrow 09R departure for the
/// endless 09L arrivals — indefinitely (2026-07-10); a parallel runway's
/// stream is not your traffic.
///
/// `runways` is the field's (the strip's own included): an arrival on an
/// INDEPENDENT runway's final ([`on_independent_final`]) is that runway's
/// traffic, even where its final crosses ours (LEMG, issue #172).
pub fn final_traffic_for(
    targets: &[TcasTarget],
    rwy: &nav::Runway,
    runways: &[nav::Runway],
    field_elev_ft: f64,
) -> Option<FinalTraffic> {
    let m_per_deg_lat = 111_320.0;
    let m_per_deg_lon = 111_320.0 * rwy.lat_a.to_radians().cos();
    let (bx, by) = (
        (rwy.lon_b - rwy.lon_a) * m_per_deg_lon,
        (rwy.lat_b - rwy.lat_a) * m_per_deg_lat,
    );
    let len2 = bx * bx + by * by;
    if len2 < 1.0 {
        return None;
    }
    let len_m = len2.sqrt();
    let mut hits: Vec<(f64, &TcasTarget)> = targets
        .iter()
        .filter(|t| !t.on_ground)
        .filter(|t| (30.0..1500.0).contains(&(t.ele_ft - field_elev_ft)))
        .filter(|t| t.vs_fpm <= CLIMBOUT_FPM)
        .filter(|t| !on_independent_final(t, rwy, runways))
        .filter_map(|t| {
            let (px, py) = (
                (t.lon - rwy.lon_a) * m_per_deg_lon,
                (t.lat - rwy.lat_a) * m_per_deg_lat,
            );
            let along = (px * bx + py * by) / len2;
            let cross_m = (px * by - py * bx).abs() / len_m;
            let beyond = FINAL_NM * 1852.0 / len_m;
            if cross_m > CORRIDOR_HALF_M || along < -beyond || along > 1.0 + beyond {
                return None;
            }
            let da = (px * px + py * py).sqrt();
            let (qx, qy) = (px - bx, py - by);
            let db = (qx * qx + qy * qy).sqrt();
            Some((da.min(db) / 1852.0, t))
        })
        .collect();
    hits.sort_by(|x, y| x.0.total_cmp(&y.0));
    let (nm, nearest) = hits.first()?;
    Some(FinalTraffic {
        nm: *nm,
        callsign: (!nearest.callsign.is_empty()).then(|| spoken_ghost(&nearest.callsign)),
        count: hits.len(),
        ghosts: hits.iter().map(|(_, t)| t.callsign.clone()).collect(),
    })
}

/// A position against one strip's centreline in flat metres: (along as a
/// share of the length from `ident_a`, metres off the centreline, length).
pub(super) fn centreline_place(r: &nav::Runway, lat: f64, lon: f64) -> Option<(f64, f64, f64)> {
    let m_lon = 111_320.0 * r.lat_a.to_radians().cos();
    let (bx, by) = ((r.lon_b - r.lon_a) * m_lon, (r.lat_b - r.lat_a) * 111_320.0);
    let len2 = bx * bx + by * by;
    if len2 < 1.0 {
        return None;
    }
    let (px, py) = ((lon - r.lon_a) * m_lon, (lat - r.lat_a) * 111_320.0);
    Some((
        (px * bx + py * by) / len2,
        (px * by - py * bx).abs() / len2.sqrt(),
        len2.sqrt(),
    ))
}

/// The nearest two strips come to each other, metres (0 when they cross):
/// the least of the four end-to-other-strip distances.
pub(super) fn strips_apart_m(a: &nav::Runway, b: &nav::Runway) -> f64 {
    let m_lon = 111_320.0 * a.lat_a.to_radians().cos();
    let xy = |lat: f64, lon: f64| ((lon - a.lon_a) * m_lon, (lat - a.lat_a) * 111_320.0);
    let (p1, p2) = (xy(a.lat_a, a.lon_a), xy(a.lat_b, a.lon_b));
    let (q1, q2) = (xy(b.lat_a, b.lon_a), xy(b.lat_b, b.lon_b));
    let cross = |o: (f64, f64), p: (f64, f64), q: (f64, f64)| {
        (p.0 - o.0) * (q.1 - o.1) - (p.1 - o.1) * (q.0 - o.0)
    };
    if cross(p1, p2, q1) * cross(p1, p2, q2) < 0.0 && cross(q1, q2, p1) * cross(q1, q2, p2) < 0.0 {
        return 0.0;
    }
    let to_seg = |p: (f64, f64), s0: (f64, f64), s1: (f64, f64)| {
        let (dx, dy) = (s1.0 - s0.0, s1.1 - s0.1);
        let l2 = dx * dx + dy * dy;
        let t = if l2 < 1.0 {
            0.0
        } else {
            (((p.0 - s0.0) * dx + (p.1 - s0.1) * dy) / l2).clamp(0.0, 1.0)
        };
        ((p.0 - s0.0 - t * dx).powi(2) + (p.1 - s0.1 - t * dy).powi(2)).sqrt()
    };
    to_seg(p1, q1, q2)
        .min(to_seg(p2, q1, q2))
        .min(to_seg(q1, p1, p2))
        .min(to_seg(q2, p1, p2))
}

/// Is this arrival on ANOTHER runway's final, one far enough from `rwy`
/// ([`nav::CLOSE_PARALLEL_M`], ICAO's segregated-parallel minimum) to be
/// worked on its own, and nearer that final's centreline than ours? Then it
/// is that runway's traffic. LEMG's 12 and 13 are 14 degrees apart and 826 m
/// at the thresholds (1,490 m at the far end); 12's final crosses 13's
/// extended centreline about 2 nm out, and a Ryanair landing 12 held a 737
/// lined up on 13 as "traffic on 2 mile final" (sortie 1790623420-full_auto,
/// 2026-09-28, issue #172). A close parallel or a crossing strip is one
/// runway's traffic.
pub(super) fn on_independent_final(t: &TcasTarget, rwy: &nav::Runway, runways: &[nav::Runway]) -> bool {
    let Some((_, own_cross, _)) = centreline_place(rwy, t.lat, t.lon) else {
        return false;
    };
    runways
        .iter()
        .filter(|r| !(r.ident_a == rwy.ident_a && r.ident_b == rwy.ident_b))
        .filter(|r| strips_apart_m(r, rwy) >= nav::CLOSE_PARALLEL_M)
        .any(|r| {
            let Some((along, cross, len)) = centreline_place(r, t.lat, t.lon) else {
                return false;
            };
            let beyond = FINAL_NM * 1852.0 / len;
            let on_final = (-beyond..0.0).contains(&along) || (1.0..=1.0 + beyond).contains(&along);
            on_final && cross <= CORRIDOR_HALF_M && cross < own_cross
        })
}

/// The nearest airborne traffic on final for the strip we're standing ON:
/// the runway-incursion watch's traffic check, in nm to the threshold.
/// Strip-aware on purpose — the earlier version ranged EVERY low TCAS target
/// inside 3 nm, and a cleared Malaga runway 12 departure got "exit the
/// runway, expedite" for the parallel runway 31 arrival stream (2026-07-12).
/// A parallel runway's stream is not your traffic — the same Heathrow lesson
/// `final_traffic_for` already encodes for the release watch.
pub async fn inbound_traffic_for_strip(xp: SimView<'_>, ident: &str) -> Option<f64> {
    let targets = targets(xp).await;
    inbound_final_for_strip(&targets, ident)
}

/// The sync core of [`inbound_traffic_for_strip`], testable without a sim.
pub(super) fn inbound_final_for_strip(targets: &[TcasTarget], ident: &str) -> Option<f64> {
    let (rwy, runways) = {
        let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
        (strip(&runways, ident)?.clone(), runways.clone())
    };
    let elev_ft = ACTIVE_PANEL
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()?
        .1;
    final_traffic_for(targets, &rwy, &runways, elev_ft).map(|t| t.nm)
}
