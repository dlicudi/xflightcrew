//! The line-up and backtrack paths, the lookahead, the stop ramp and the
//! schedule.

use super::*;

// ---- Line-up: from the hold-short position onto the runway centreline ----

/// Farther than this from the centreline is not a hold-short position —
/// the same reasoning as [`crate::taxi_guide::MAX_JOIN_M`]: the geometry stops meaning
/// "line up" and starts meaning "drive somewhere".
pub(crate) const LINE_UP_MAX_LATERAL_M: f64 = 150.0;
/// Runway that must remain beyond the alignment stop — lined up with less
/// than this ahead is a line-up onto the wrong end.
pub(super) const LINE_UP_MIN_REMAINING_M: f64 = 400.0;
/// The drive continues this far down the centreline past the entry point,
/// so the nose is pointing down the runway when it stops, not across it.
/// The arrival check is distance-based, so this leg is what buys the
/// alignment — 60 m left the nose 11 degrees off at the stop (2026-07-20).
pub(super) const LINE_UP_ALIGN_M: f64 = 80.0;

/// The synthetic route of a line-up: from where the aircraft sits, onto the
/// centreline ahead, and down it far enough to finish aligned. `None` when
/// the geometry refuses (too far off, or not enough runway past the stop).
pub fn line_up_path(
    rwy: &crate::runway::Runway,
    lat: f64,
    lon: f64,
    heading_deg: Option<f64>,
) -> Option<Vec<(f64, f64)>> {
    use crate::autoland::project_latlon;
    let (entry_m, stop_m) = line_up_stops_m(rwy, lat, lon, heading_deg)?;
    if rwy.length_m - stop_m < LINE_UP_MIN_REMAINING_M {
        return None;
    }
    let e = project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, entry_m / M_PER_NM);
    let b = project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, stop_m / M_PER_NM);
    Some(vec![(lat, lon), e, b])
}

/// A backtrack's leg two: the aircraft has turned round ON the strip and
/// points down it already, so it only settles its offset and straightens.
/// It used the hold-short geometry (an entry 30 to 60 m ahead, then
/// [`LINE_UP_ALIGN_M`]) and the Evolution drove 114 m of LILB's 600 m
/// runway after its turn; the takeoff then found 448 m left against 599
/// needed (2026-09-25). At most [`BACKTRACK_SETTLE_MAX_M`] plus
/// [`BACKTRACK_ALIGN_M`].
///
/// An airframe that PIVOTS (`turn.pivots`: a castoring nose or brake
/// steering, whose line-up then ends in [`LineUpAlign`]) settles in less:
/// the entry at twice the offset, then [`PIVOT_SETTLE_M`], the alignment
/// finishing the heading. LILB 03 Evolution (1790721973, 2026-09-30): from
/// the pocket 24.9 m past the numbers and 14.9 m off, the entry was 55 m
/// down and the drive stopped 84 m down, 516.1 m left against 500 (the
/// alignment then crept 40 m more and lost the fit).
pub fn backtrack_leg_two_path(
    rwy: &crate::runway::Runway,
    lat: f64,
    lon: f64,
    turn: TurnRound,
) -> Option<Vec<(f64, f64)>> {
    use crate::autoland::project_latlon;
    use crate::runway::{along_track_nm, cross_track_nm};
    let lateral = (cross_track_nm(lat, lon, rwy) * M_PER_NM).abs();
    if lateral > LINE_UP_MAX_LATERAL_M {
        return None;
    }
    let a0 = (-along_track_nm(lat, lon, rwy) * M_PER_NM).max(0.0);
    let (entry_m, stop_m) = if turn.pivots {
        let entry = a0 + (lateral * 2.0).clamp(PIVOT_ENTRY_MIN_M, BACKTRACK_SETTLE_MAX_M);
        (entry, entry + PIVOT_SETTLE_M)
    } else {
        let entry = a0 + (lateral * 4.0).clamp(BACKTRACK_SETTLE_MIN_M, BACKTRACK_SETTLE_MAX_M);
        (entry, entry + BACKTRACK_ALIGN_M)
    };
    if rwy.length_m - stop_m < LINE_UP_MIN_REMAINING_M.min(rwy.length_m * 0.5) {
        return None;
    }
    let e = project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, entry_m / M_PER_NM);
    let b = project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, stop_m / M_PER_NM);
    Some(vec![(lat, lon), e, b])
}
pub(super) const BACKTRACK_SETTLE_MIN_M: f64 = 10.0;
pub(super) const BACKTRACK_SETTLE_MAX_M: f64 = 30.0;
pub(super) const BACKTRACK_ALIGN_M: f64 = 30.0;
/// A pivoting airframe's leg two: the entry at least this far down from
/// where it turned, and this much centreline after it (the route's end:
/// the drive stops [`STOP_SHORT_M`] before it).
pub(super) const PIVOT_ENTRY_MIN_M: f64 = 10.0;
pub(super) const PIVOT_SETTLE_M: f64 = 12.0;

/// The way a backtrack's turn-round turns: TOWARD the centreline, across
/// the strip, the way the teardrop pocket was laid out for (+1 right, -1
/// left); `None` on the centreline itself. The pocket leg arrives pointing
/// a little OUTWARD, so the shorter way round is outward, and the drive
/// took it: LILB 03 Evolution (1790721973, 2026-09-30) stood 14.9 m right
/// of the centreline heading 172 for a 024 runway, turned left through
/// 113 and 44 with the aircraft carried out to 21.6 m, in the grass of a
/// 15 m strip, and crossed back 58 degrees off. Pure.
pub fn turn_round_hint(rwy: &crate::runway::Runway, lat: f64, lon: f64) -> Option<f64> {
    let ct_m = crate::runway::cross_track_nm(lat, lon, rwy) * M_PER_NM;
    (ct_m.abs() >= TURN_HINT_MIN_OFFSET_M).then(|| ct_m.signum())
}
pub(super) const TURN_HINT_MIN_OFFSET_M: f64 = 1.0;

/// How a backtrack turns round at the numbers ([`backtrack_pocket_path`],
/// [`backtrack_leg_two_path`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TurnRound {
    /// The airframe pivots on a braked wheel: a castoring nose or brake
    /// steering (the line-up then ends in [`LineUpAlign`]).
    pub pivots: bool,
    /// The strip's pavement width, m, when apt.dat gave one.
    pub width_m: Option<f64>,
}

impl TurnRound {
    /// The teardrop every airframe flew before: room for a steered nose.
    #[cfg(test)]
    pub const TEARDROP: TurnRound = TurnRound { pivots: false, width_m: None };

    /// Where the turn begins down the strip, where the pocket sits down it
    /// and how far off the centreline, m. A pivoting airframe turns nearer
    /// the numbers and on the PAVEMENT: the teardrop's 12 m off put the
    /// Evolution's pocket in the grass beside LILB's 15 m strip, 30 m down
    /// (1790721973, 2026-09-30).
    pub fn pocket(&self) -> (f64, f64, f64) {
        if !self.pivots {
            return (
                BACKTRACK_TURN_START_M,
                BACKTRACK_TURN_START_M / 2.0,
                BACKTRACK_POCKET_OFFSET_M,
            );
        }
        let offset = self.width_m.map_or(PIVOT_POCKET_OFFSET_M, |w| {
            (w / 2.0 - PIVOT_POCKET_EDGE_M)
                .clamp(PIVOT_POCKET_MIN_OFFSET_M, BACKTRACK_POCKET_OFFSET_M)
        });
        (PIVOT_TURN_START_M, PIVOT_POCKET_ALONG_M, offset)
    }
}
/// A pivoting airframe's pocket: the turn begins this far down, the pocket
/// this far down (the drive has been seen to run 5 to 10 m past it at the
/// backtrack's pace), this far off the centreline with no width known, and
/// this far inside the pavement's edge with one, never nearer the
/// centreline than the minimum.
///
/// The edge is the AIRCRAFT's centre: the outer main wheel sits half the
/// track further out (about 1.6 m on a GA single), and the pocket leg stops
/// a metre off its line. At 2 m the Evolution's pocket on LILB's 15 m strip
/// was 5.5 m off; leg one stopped at 6.5 (1790732094, 2026-09-30), the
/// outer wheel about 8.1 m out against the 7.5 m edge, and the turn-round
/// swung the wrong way about it, stuck, and broke away at 9.5 kt. From 5.1 m
/// (1790724857) the same turn worked. So 3.5 m: the minimum's 4 m there.
pub(super) const PIVOT_TURN_START_M: f64 = 50.0;
pub(super) const PIVOT_POCKET_ALONG_M: f64 = 25.0;
pub(super) const PIVOT_POCKET_OFFSET_M: f64 = 6.0;
pub(super) const PIVOT_POCKET_EDGE_M: f64 = 3.5;
pub(super) const PIVOT_POCKET_MIN_OFFSET_M: f64 = 4.0;

/// Where a plain line-up from this position would enter and stop (meters
/// past the threshold), or `None` when the position isn't a hold-short
/// geometry at all. Shared by the path builder and the full-length check.
/// `heading_deg` (the aircraft's current heading) makes leg one roll STRAIGHT
/// FORWARD along the taxiway lead-on onto the centreline instead of cutting a
/// diagonal across to it (the "beeline" the pilot saw, 2026-07-23); `None`
/// keeps the old diagonal (backtrack leg two, tests).
pub fn line_up_stops_m(
    rwy: &crate::runway::Runway,
    lat: f64,
    lon: f64,
    heading_deg: Option<f64>,
) -> Option<(f64, f64)> {
    use crate::runway::{along_track_nm, cross_track_nm};
    let x0 = cross_track_nm(lat, lon, rwy) * M_PER_NM; // signed, right of course +
    let lateral = x0.abs();
    if lateral > LINE_UP_MAX_LATERAL_M {
        return None;
    }
    // Signed distance past the threshold (positive = down the runway).
    let a0 = -along_track_nm(lat, lon, rwy) * M_PER_NM;
    // Roll straight forward along the current heading onto the centreline —
    // the taxiway lead-on line — when the heading gives a clean crossing.
    // Otherwise the old fallback: enter ahead of abeam by ~the lateral offset
    // (a ~45 degree swing), floored at one turn's worth and capped so a far-set
    // hold (LXGB's Echo ~90 m off the strip) gets a sharper swing, not a 90 m lead.
    let entry_m = roll_forward_entry_m(rwy.course_deg, a0, x0, heading_deg)
        .unwrap_or_else(|| a0.max(0.0) + lateral.clamp(30.0, 60.0));
    Some((entry_m, entry_m + LINE_UP_ALIGN_M))
}

/// Distance past the threshold where the aircraft's heading, rolled straight
/// forward, crosses the centreline — following the taxiway lead-on onto the
/// runway. `None` (use the diagonal fallback) when the heading is within ~15
/// degrees of the runway (no clean crossing), points away from the centreline,
/// crosses behind the threshold, or would run implausibly far.
pub(super) fn roll_forward_entry_m(
    course_deg: f64,
    a0: f64,
    x0: f64,
    heading_deg: Option<f64>,
) -> Option<f64> {
    let psi = heading_deg?;
    let rel = crate::angles::signed_diff_deg(psi, course_deg).to_radians();
    let s = rel.sin();
    if s.abs() < 0.26 {
        return None; // within ~15 degrees of the runway: no clean forward crossing
    }
    let fwd = -x0 / s; // meters forward to reach cross-track zero
    if fwd <= 0.0 || fwd > 150.0 {
        return None; // heading away from the centreline, or too shallow / far
    }
    let entry = a0 + fwd * rel.cos();
    (entry >= 0.0).then_some(entry)
}

/// A line-up riding the AUTHORED lead-on: the trimmed tail of the cleared taxi
/// route — the painted yellow line from the holding point onto the runway
/// centreline — instead of a synthetic diagonal (the aircraft cut the Echo
/// stub's curve and stopped crooked, LXGB 2026-07-24). Returns the path and
/// its centreline stop (m past the threshold). `None` when the lead-on isn't
/// THIS hold or THIS runway: it must start near the aircraft and end on the
/// centreline with runway enough beyond the stop.
/// How far from the centreline a lead-on may end and still be on the runway
/// (half the widest strip, 60 m, plus slack), and how far before the end.
pub(super) const LEAD_ON_JUNCTION_XT_M: f64 = 35.0;
pub(super) const LEAD_ON_BEHIND_END_M: f64 = 30.0;

pub fn line_up_path_from_lead_on(
    rwy: &crate::runway::Runway,
    lat: f64,
    lon: f64,
    lead_on: &[(f64, f64)],
) -> Option<(Vec<(f64, f64)>, f64)> {
    use crate::autoland::project_latlon;
    use crate::runway::{along_track_nm, cross_track_nm};
    let (&first, &last) = (lead_on.first()?, lead_on.last()?);
    // OURS, not another stub's: compare ALONG-TRACK positions, not straight-
    // line distance — the stub runs perpendicular to the runway, so a legit
    // hold sits ~90 m from its own junction (LXGB Echo: 92 m; the old 80 m
    // straight-line guard silently rejected it and the diagonal came back,
    // 2026-07-24, probed). A foreign stub is displaced ALONG the runway.
    // Every rejection says which test and the numbers: a remote hold at
    // LIME 28 (200 m off the centreline) was refused as "can't find runway
    // 28" with nothing to say whether the lead-on had failed and why
    // (2026-09-25).
    let reject = |why: &str, v: f64| {
        crate::blackbox::record(
            "autotaxi",
            serde_json::json!({"event": "lead_on_rejected", "rwy": rwy.id, "why": why, "value": v, "pts": lead_on.len()}),
        );
        None
    };
    let a_air = -along_track_nm(lat, lon, rwy) * M_PER_NM;
    let a_first = -along_track_nm(first.0, first.1, rwy) * M_PER_NM;
    if (a_first - a_air).abs() > 80.0 {
        return reject("not_this_hold", a_first - a_air);
    }
    // ...and ends ON this runway's pavement (the junction node): a
    // connector that meets the runway at its end can stop at the edge of a
    // 45 m strip, not on the centreline; the fillet below finishes it.
    let junction_xt = (cross_track_nm(last.0, last.1, rwy) * M_PER_NM).abs();
    if junction_xt > LEAD_ON_JUNCTION_XT_M {
        return reject("junction_off_the_runway", junction_xt);
    }
    // A junction a little before the end is the end: a hold beside the
    // runway's first metres (and a displaced threshold is usable for
    // takeoff).
    let junction_a = -along_track_nm(last.0, last.1, rwy) * M_PER_NM;
    if junction_a < -LEAD_ON_BEHIND_END_M {
        return reject("junction_behind_the_end", junction_a);
    }
    let junction_a = junction_a.max(0.0);
    let stop_m = junction_a + LINE_UP_ALIGN_M;
    if rwy.length_m - stop_m < LINE_UP_MIN_REMAINING_M {
        return reject("too_little_runway_left", rwy.length_m - stop_m);
    }
    // Aircraft, then the authored points (skipping any we're already on top
    // of), then a FILLET ARC around the junction corner onto the centreline,
    // then the alignment stop. The network models the entry as a right angle;
    // the paint curves — a sampled arc where the sharp corner was is what the
    // pursuit can actually follow (it chorded the raw corner, probed
    // 2026-07-24).
    let mut path = vec![(lat, lon)];
    path.extend(lead_on_ahead(lead_on, lat, lon));
    let stop = project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, stop_m / M_PER_NM);
    let corner = path.pop()?; // the junction (or the aircraft itself — refused below)
    if path.is_empty() {
        return reject("already_at_the_junction", 0.0);
    }
    path.extend(fillet_corner(*path.last()?, corner, stop, 18.0));
    path.push(stop);
    (path.len() >= 3).then_some((path, stop_m))
}

/// The lead-on's points still AHEAD of the aircraft and more than 5 m from
/// it (one we are on top of is not a point to drive to), for the line-up
/// and the backtrack. A taxi that rolls through its holding stop (LEMG 31,
/// 2026-09-28: `past_end_ticks` 36, stopped 7.4 m past the hold node) left
/// the node BEHIND the aircraft, and the path ran back to it and forward
/// again through the aircraft: at rest the joining projection read 14.8 m
/// along (the forward pass) and the plain one 0 m (the leg back), and
/// `remaining_m` jumped 14.9 m between two ticks with nothing moving.
pub(super) fn lead_on_ahead(lead_on: &[(f64, f64)], lat: f64, lon: f64) -> Vec<(f64, f64)> {
    let dist_m = |a: (f64, f64), b: (f64, f64)| {
        crate::autoland::haversine_nm(a.0, a.1, b.0, b.1) * M_PER_NM
    };
    let passed_m = crate::taxi_guide::project_signed(lead_on, lat, lon).map_or(0.0, |(a, _)| a);
    let mut along = 0.0;
    let mut prev = lead_on.first().copied();
    lead_on
        .iter()
        .copied()
        .filter(|&p| {
            along += prev.map_or(0.0, |q| dist_m(q, p));
            prev = Some(p);
            along > passed_m - 1.0 && dist_m((lat, lon), p) > 5.0
        })
        .collect()
}

/// A sampled quadratic-Bezier fillet replacing the sharp vertex at `corner`:
/// starts `r_m` back along the inbound leg, ends `r_m` down the outbound leg,
/// bulging toward the corner — the painted lead-on curve the taxi networks
/// don't model. Radii clamp to half of each leg so short legs stay sane.
pub(super) fn fillet_corner(
    prev: (f64, f64),
    corner: (f64, f64),
    next: (f64, f64),
    r_m: f64,
) -> Vec<(f64, f64)> {
    let seg = |a: (f64, f64), b: (f64, f64), t: f64| (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t);
    let len =
        |a: (f64, f64), b: (f64, f64)| crate::autoland::haversine_nm(a.0, a.1, b.0, b.1) * M_PER_NM;
    let l_in = len(prev, corner);
    let l_out = len(corner, next);
    if l_in < 1.0 || l_out < 1.0 {
        return vec![corner];
    }
    let a = seg(corner, prev, (r_m.min(l_in * 0.5)) / l_in); // arc entry, on the inbound leg
    let b = seg(corner, next, (r_m.min(l_out * 0.5)) / l_out); // arc exit, on the outbound leg
                                                               // Quadratic Bezier a -> corner(control) -> b, sampled.
    let bez = |t: f64| {
        let u = 1.0 - t;
        (
            u * u * a.0 + 2.0 * u * t * corner.0 + t * t * b.0,
            u * u * a.1 + 2.0 * u * t * corner.1 + t * t * b.1,
        )
    };
    vec![a, bez(0.25), bez(0.5), bez(0.75), b]
}

/// A backtrack pocket sits this far past the threshold along the runway —
/// room for the turnaround to finish at the numbers.
pub(super) const BACKTRACK_TURN_START_M: f64 = 60.0;
/// ...and this far off the centreline (right of the backtrack direction),
/// so the 180 is a teardrop with turning room, never a pivot on the spot.
pub(super) const BACKTRACK_POCKET_OFFSET_M: f64 = 12.0;
/// Closer to the threshold than this, a backtrack is pointless — the plain
/// line-up already stops within a couple of plane-lengths of the numbers.
pub(super) const BACKTRACK_MIN_M: f64 = 150.0;

/// Leg one of a backtrack: from the hold, onto the centreline heading TOWARD
/// the departure threshold, down the strip, ending in a turn pocket beside
/// the numbers. Leg two is a plain [`line_up_path`] from the pocket — the
/// join machinery already handles "facing the wrong way", and splitting the
/// maneuver keeps each route free of overlapping legs (a single out-and-back
/// polyline is ambiguous to project onto). Real Gibraltar: the taxiways only
/// reach the east half, so a full-length 09 departure backtracks ~1.2 km
/// (2026-07-22, held at Charlie with 580 m ahead).
pub fn backtrack_pocket_path(
    rwy: &crate::runway::Runway,
    lat: f64,
    lon: f64,
    turn: TurnRound,
) -> Option<Vec<(f64, f64)>> {
    let (turn_start_m, pocket_m, pocket_off_m) = turn.pocket();
    use crate::autoland::project_latlon;
    use crate::runway::{along_track_nm, cross_track_nm};
    let lateral = (cross_track_nm(lat, lon, rwy) * M_PER_NM).abs();
    if lateral > LINE_UP_MAX_LATERAL_M {
        return None;
    }
    let past_m = (-along_track_nm(lat, lon, rwy) * M_PER_NM).max(0.0);
    if past_m < BACKTRACK_MIN_M {
        return None;
    }
    let c = |d_m: f64| project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, d_m / M_PER_NM);
    // Enter the runway swinging toward the threshold (the mirror of the
    // line-up's entry lead), never inside the turn pocket itself.
    let entry_m = (past_m - lateral.clamp(30.0, 60.0)).max(turn_start_m + 30.0);
    let entry = c(entry_m);
    let turn_start = c(turn_start_m);
    // Swing the teardrop toward the side the aircraft CAME IN on — the taxiway/
    // apron side, which is the way the painted turn-pad leads. A fixed hand
    // turned the wrong way against the yellow line at LXGB 09 (2026-07-26).
    // `cross_track` is signed (+ = right of the runway course); default to the
    // right of travel when we're dead on the centreline.
    let entry_ct_m = cross_track_nm(lat, lon, rwy) * M_PER_NM;
    let pocket_bearing = if entry_ct_m >= 0.0 {
        rwy.course_deg + 90.0
    } else {
        rwy.course_deg - 90.0
    };
    let mid = c(pocket_m);
    let pocket = project_latlon(mid.0, mid.1, pocket_bearing, pocket_off_m / M_PER_NM);
    Some(vec![(lat, lon), entry, turn_start, pocket])
}

/// The same backtrack, entering the runway along the AUTHORED lead-on — the
/// painted line from the holding point onto the centreline — instead of a
/// diagonal aimed at a point down the strip.
///
/// The plain line-up learned this in 2026-07-24 ([`line_up_path_from_lead_on`]);
/// the backtrack kept its diagonal and it put an MD-82 in the grass at Luton
/// (2026-07-27). Its leg one ran from the holding point to a centreline point
/// 30 to 60 m nearer the threshold — 23 degrees off the stub the aircraft was
/// standing on — so the drive left the pavement within fifty metres, bogged,
/// and the governor called it stagnant with the aircraft off the taxiway. The
/// entry is not ours to invent: the stub is drawn, and the only question is
/// which way we turn at the top of it.
pub fn backtrack_pocket_path_from_lead_on(
    rwy: &crate::runway::Runway,
    lat: f64,
    lon: f64,
    lead_on: &[(f64, f64)],
    turn: TurnRound,
) -> Option<Vec<(f64, f64)>> {
    use crate::autoland::project_latlon;
    use crate::runway::{along_track_nm, cross_track_nm};
    let (turn_start_m, pocket_m, pocket_off_m) = turn.pocket();
    let (&first, &last) = (lead_on.first()?, lead_on.last()?);
    // OURS, and ending on THIS centreline — the same two tests the plain
    // lead-on line-up applies, for the same reason: a foreign stub is
    // displaced along the runway, not merely far away.
    let a_air = -along_track_nm(lat, lon, rwy) * M_PER_NM;
    if (-along_track_nm(first.0, first.1, rwy) * M_PER_NM - a_air).abs() > 80.0 {
        return None;
    }
    if (cross_track_nm(last.0, last.1, rwy) * M_PER_NM).abs() > 20.0 {
        return None;
    }
    // Same "is a backtrack worth it" test as the synthetic version — measured
    // from the AIRCRAFT, not the junction: near the numbers the plain line-up
    // already stops within a plane-length or two of them. And the junction has
    // to sit beyond the turn pocket, or there is nothing to drive back along.
    if a_air < BACKTRACK_MIN_M {
        return None;
    }
    let junction_a = -along_track_nm(last.0, last.1, rwy) * M_PER_NM;
    if junction_a < turn_start_m + 30.0 {
        return None;
    }
    let c = |d_m: f64| project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, d_m / M_PER_NM);
    let turn_start = c(turn_start_m);
    let entry_ct_m = cross_track_nm(lat, lon, rwy) * M_PER_NM;
    let pocket_bearing = if entry_ct_m >= 0.0 {
        rwy.course_deg + 90.0
    } else {
        rwy.course_deg - 90.0
    };
    let mid = c(pocket_m);
    let pocket = project_latlon(mid.0, mid.1, pocket_bearing, pocket_off_m / M_PER_NM);
    // Aircraft, up the drawn stub, a fillet around the junction (the network
    // draws a right angle, the paint curves), then down the strip to the
    // pocket.
    let mut path = vec![(lat, lon)];
    path.extend(lead_on_ahead(lead_on, lat, lon));
    let corner = path.pop()?;
    if path.is_empty() {
        return None;
    }
    path.extend(fillet_corner(*path.last()?, corner, turn_start, 18.0));
    path.push(turn_start);
    path.push(pocket);
    (path.len() >= 4).then_some(path)
}

/// How a line-up will be driven: straight on, or the full-length backtrack.
pub enum LineUpPlan {
    Plain(Vec<(f64, f64)>),
    Backtrack(Vec<(f64, f64)>),
}

/// Pick the runway AND the maneuver: a plain line-up when the runway ahead
/// of its stop covers `needed_m`, otherwise a backtrack to the threshold
/// when the geometry offers one (`force_backtrack` skips straight to it —
/// the pilot asked for the full length in so many words).
#[allow(clippy::too_many_arguments)]
pub fn pick_line_up_plan(
    cands: &[crate::runway::Runway],
    lat: f64,
    lon: f64,
    cleared: Option<&str>,
    needed_m: f64,
    force_backtrack: bool,
    heading_deg: Option<f64>,
    lead_on: Option<&[(f64, f64)]>,
    turn: TurnRound,
) -> Option<(crate::runway::Runway, LineUpPlan)> {
    let mut best: Option<(crate::runway::Runway, LineUpPlan, f64)> = None;
    for r in cands {
        let turn = TurnRound {
            width_m: turn.width_m.or_else(|| crate::runway::runway_width_m(r)),
            ..turn
        };
        if let Some(c) = cleared {
            if !crate::autotakeoff::runway_matches(&r.id, c) {
                continue;
            }
        }
        // Already ON the strip and pointing down it (a turn-round or a
        // line-up that stopped off the line): settle onto the centreline in
        // the least runway, as a backtrack's leg two does.
        if !force_backtrack {
            if let Some((p, stop_m)) = on_strip_settle_path(r, lat, lon, heading_deg, turn) {
                if r.length_m - stop_m >= needed_m {
                    if best.as_ref().is_none_or(|(_, _, d)| r.dist_nm < *d) {
                        best = Some((r.clone(), LineUpPlan::Plain(p), r.dist_nm));
                    }
                    continue;
                }
            }
        }
        // The authored lead-on wins when it belongs to this runway and this
        // hold — the painted line IS the maneuver. Synthetic geometry is the
        // fallback (no published route, wrong stub, backtracks).
        let via_lead = if force_backtrack {
            None
        } else {
            lead_on.and_then(|lo| line_up_path_from_lead_on(r, lat, lon, lo))
        };
        let (plain, stop_m) = match via_lead {
            Some((p, s)) => (Some(p), Some(s)),
            None => (
                line_up_path(r, lat, lon, heading_deg),
                line_up_stops_m(r, lat, lon, heading_deg).map(|(_, stop)| stop),
            ),
        };
        let ahead = stop_m.map(|s| r.length_m - s);
        // Use the FULL length for a departure: if the line-up entry sits well
        // past the numbers (a deep intersection like LXGB's Charlie, which
        // enters the middle of 09) on a short/medium runway, backtrack to the
        // threshold rather than accept the reduced length — even when the roll
        // would fit. A departure lines up at the START of the runway; the pilot
        // shouldn't have to ask (2026-07-26). Long runways keep intersection
        // departures (backtracking 3 km of active runway is not the move).
        const DEEP_INTERSECTION_M: f64 = 200.0;
        const FULL_LENGTH_MATTERS_M: f64 = 2500.0;
        let deep_intersection =
            stop_m.is_some_and(|s| s > DEEP_INTERSECTION_M) && r.length_m <= FULL_LENGTH_MATTERS_M;
        let wants_backtrack = force_backtrack
            || ahead.is_none_or(|a| a < needed_m)
            || plain.is_none()
            || deep_intersection;
        let plan = if wants_backtrack {
            // The drawn stub first, here too — a backtrack still has to get
            // ONTO the runway, and inventing that entry is what put an
            // aircraft in the grass (see `backtrack_pocket_path_from_lead_on`).
            let via_lead_bt = lead_on
                .and_then(|lo| backtrack_pocket_path_from_lead_on(r, lat, lon, lo, turn))
                .or_else(|| backtrack_pocket_path(r, lat, lon, turn));
            match via_lead_bt {
                Some(p) => Some(LineUpPlan::Backtrack(p)),
                None => plain.map(LineUpPlan::Plain),
            }
        } else {
            plain.map(LineUpPlan::Plain)
        };
        let Some(plan) = plan else { continue };
        if best.as_ref().is_none_or(|(_, _, d)| r.dist_nm < *d) {
            best = Some((r.clone(), plan, r.dist_nm));
        }
    }
    best.map(|(r, p, _)| (r, p))
}

/// A line-up asked for from ON the strip, the nose within
/// [`ON_STRIP_MAX_ERR_DEG`] of its course: the route of a backtrack's leg two
/// from here ([`backtrack_leg_two_path`]) and where it stops, m past the
/// threshold. `None` off the pavement (half the width, else
/// [`ON_STRIP_HALF_WIDTH_M`]), short of the threshold or pointing elsewhere.
/// LILB 03 Evolution (1790732094, 2026-09-30): the turn-round stopped 41 m
/// down, 5.6 m off and 14 degrees crooked; the hold-short geometry from
/// there enters 30 to 60 m further on and leaves less than the 500 m fit,
/// so the one retry a failed line-up gets would have been hopeless. Pure.
pub fn on_strip_settle_path(
    rwy: &crate::runway::Runway,
    lat: f64,
    lon: f64,
    heading_deg: Option<f64>,
    turn: TurnRound,
) -> Option<(Vec<(f64, f64)>, f64)> {
    use crate::runway::{along_track_nm, cross_track_nm};
    let hdg = heading_deg?;
    let err = crate::angles::abs_diff_deg(hdg, rwy.course_deg);
    if err > ON_STRIP_MAX_ERR_DEG {
        return None;
    }
    let half = turn.width_m.map_or(ON_STRIP_HALF_WIDTH_M, |w| w / 2.0);
    let ct = (cross_track_nm(lat, lon, rwy) * M_PER_NM).abs();
    let a0 = -along_track_nm(lat, lon, rwy) * M_PER_NM;
    if ct > half || a0 < 0.0 || a0 > rwy.length_m {
        return None;
    }
    let path = backtrack_leg_two_path(rwy, lat, lon, turn)?;
    let end = *path.last()?;
    let stop_m = -along_track_nm(end.0, end.1, rwy) * M_PER_NM;
    Some((path, stop_m))
}
/// Pointing down the strip, for [`on_strip_settle_path`].
pub const ON_STRIP_MAX_ERR_DEG: f64 = 45.0;
/// On it, with no width known.
pub(super) const ON_STRIP_HALF_WIDTH_M: f64 = 10.0;

/// Pick the runway end to line up on: feasible geometry, matching the
/// cleared runway when the tower named one (lining up on 09 with a 27
/// clearance is the backwards trap, refused upstream by returning the
/// match only). Nearest threshold among the feasible wins.
pub fn pick_line_up(
    cands: &[crate::runway::Runway],
    lat: f64,
    lon: f64,
    cleared: Option<&str>,
    heading_deg: Option<f64>,
) -> Option<(crate::runway::Runway, Vec<(f64, f64)>)> {
    type LineUpCandidate = (crate::runway::Runway, Vec<(f64, f64)>, f64);
    let mut best: Option<LineUpCandidate> = None;
    for r in cands {
        if let Some(c) = cleared {
            if !crate::autotakeoff::runway_matches(&r.id, c) {
                continue;
            }
        }
        let Some(path) = line_up_path(r, lat, lon, heading_deg) else {
            continue;
        };
        if best.as_ref().is_none_or(|(_, _, d)| r.dist_nm < *d) {
            best = Some((r.clone(), path, r.dist_nm));
        }
    }
    best.map(|(r, p, _)| (r, p))
}

/// Pure-pursuit lookahead: seconds of taxi ahead, floored so creep-speed
/// taxiing still aims somewhere and capped so the aim point can't cut a
/// corner two junctions away. 5 s, not 3.5: at 14 m of lookahead a 3 m
/// centreline offset read as 12° of error and half-lock steer — the SR22
/// S-weaved ±4 m for whole legs even with the rate lead (2026-07-22).
/// Longer aim turns the same offset into ~8° and a quarter-lock nudge.
/// Test-only since the live loop took the aim from the profile
/// ([`Pace::lookahead_s`]); the tests pin this proven 5 s baseline.
#[cfg(test)]
pub fn lookahead_m(gs_kts: f64) -> f64 {
    lookahead_m_with(gs_kts, 5.0)
}

/// [`lookahead_m`] with the airframe's own aim time. A slow plant needs a
/// LONG aim: the SF50 (castoring wheel, brake-assisted, ~3.5 deg/s of yaw at
/// full command) crossed the centreline 24 degrees ANGLED at 12 kt with the
/// 5 s aim — it cannot turn fast enough to roll out in 31 m, so every
/// crossing overshot ~10 m the other way, a slow drunk S down the taxiway
/// (2026-07-29). Aim further out and the same offset asks for a shallower
/// crossing the plant can actually fly. The cap scales with the aim so a
/// long-aim airframe keeps it at speed.
pub fn lookahead_m_with(gs_kts: f64, aim_s: f64) -> f64 {
    (gs_kts * 0.514 * aim_s).clamp(16.0, 10.0 * aim_s)
}

/// Pure pursuit on a curve cuts inside it by about L^2 / 2R. A long aim in a
/// GENTLE bend is where that bites: the Evolution's 9 s aim at 12 kt is 55 m,
/// and a 107 m radius bend at LIME put it 8.5 m inside the line with the
/// steering near zero, 13 m on another (2026-09-25). The aim is capped so the
/// geometric cut stays under [`BEND_MAX_CUT_M`]; never under the corner
/// floor ([`BEND_AIM_FLOOR_M`], 16 m; a free castoring nose flies `ground_steer`), and
/// straights are exactly as they were.
/// Test-only since the live loop takes the floor per nose
/// ([`steer_look_m`]); the tests pin the 16 m baseline.
#[cfg(test)]
pub fn bend_lookahead_cap_m(gs_kts: f64, turn_dps: f64) -> f64 {
    bend_lookahead_cap_floor_m(gs_kts, turn_dps, BEND_AIM_FLOOR_M)
}

/// [`bend_lookahead_cap_m`] with the corner's own floor.
///
/// The corner is its radius, v over the route's turn rate, and that is the
/// same at any speed the rate was read at: under 0.5 m/s the "straight"
/// threshold scales with the speed (the same 143 m radius) instead of the
/// cap vanishing. It vanished under a knot, and a free nose recovering from
/// a swing at 0.8 to 1.1 kt aimed 12 m and 16 m on alternate ticks: the
/// error read -29 to -38 degrees, then -15 to -20, and the turn builder's
/// brake went 0.06 to 0.07, nothing, 0.07 (LEMG, 1790736973, 2026-09-30).
/// Only a free nose notices: every other nose's crawl aim is its 16 m floor
/// either way.
pub fn bend_lookahead_cap_floor_m(gs_kts: f64, turn_dps: f64, floor_m: f64) -> f64 {
    let v = gs_kts * 0.514_444;
    if v <= 0.0 || turn_dps.abs() < BEND_STRAIGHT_DPS * (v / BEND_STRAIGHT_FROM_MS).min(1.0) {
        return f64::INFINITY;
    }
    let radius_m = v / turn_dps.abs().to_radians();
    (2.0 * radius_m * BEND_MAX_CUT_M).sqrt().max(floor_m)
}
/// Under this turn rate the route is straight, from this speed up; below
/// it the rate that is the same radius.
pub(super) const BEND_STRAIGHT_DPS: f64 = 0.2;
pub(super) const BEND_STRAIGHT_FROM_MS: f64 = 0.5;
/// The most a gentle bend may be cut, m.
pub const BEND_MAX_CUT_M: f64 = 1.5;
/// The shortest aim a corner is flown on: a shorter one asks for a turn
/// tighter than a nosewheel's lock can fly, and the aircraft runs wide.
pub const BEND_AIM_FLOOR_M: f64 = 16.0;
/// Candidate shorter aim for linked noses through tight bends. EGBS Bravo
/// recorded a 7.6 m inside-corner cut; the fixed 16 m floor is a suspected
/// contributor. Exact-route replay and live confirmation remain necessary.
pub const LINKED_BEND_AIM_FLOOR_M: f64 = 8.0;
/// The pure-pursuit aim for one tick, metres: the airframe's aim time,
/// capped in a bend ([`bend_lookahead_cap_floor_m`] over
/// [`BEND_PROBE_M`]) and by what is left of the route. `turn_probe_dps` is
/// `Route::route_turn_rate_dps(progress, BEND_PROBE_M, gs_kts)`.
///
/// The route cap: near the stop the pursuit point must walk the path
/// through the final corner, not chord across it. The 16 m minimum aim
/// against a corner ~20 m before the hold parked the aircraft off the paint,
/// and stop_straighten then froze the leftover angle (LXGB Echo,
/// 2026-07-24, twice). Only bites close-in, where the stop ramp has already
/// slowed us below weave speeds.
///
/// A free castoring nose no longer steers on this aim (`ground_steer`
/// flies its filleted path); the aim still gives its drive the error the
/// join and the guards read, and the route's turn rate the scorer reads.
pub fn steer_look_m(
    pace: &Pace,
    gs_kts: f64,
    turn_probe_dps: f64,
    remaining_m: f64,
    free_nose: bool,
) -> f64 {
    let bend_floor = if free_nose || pace.brake_steer {
        BEND_AIM_FLOOR_M
    } else {
        LINKED_BEND_AIM_FLOOR_M
    };
    lookahead_m_with(gs_kts, pace.lookahead_s)
        .min(bend_lookahead_cap_floor_m(gs_kts, turn_probe_dps, bend_floor))
        .min((remaining_m * 0.6).max(6.0))
}
/// The route span the bend's curvature is read over, m.
pub const BEND_PROBE_M: f64 = 25.0;

/// Deceleration the stop ramp plans for, m/s².
///
/// NOT the deceleration the brakes can achieve — that is about 0.7 m/s² at the
/// gentle end (measured: 0.14-0.18 brake gave a -1.36 kt/s trend at EGHI). It
/// is deliberately far below it, because the ramp has to command a speed the
/// governor will actually ACT on, and `taxi_control` only brakes once the
/// aircraft is 3 kt above target. A √ curve is flat far out and steep near
/// zero, so at 0.5 that gap did not open until ~7 m to go:
///
///   rem 22.9 m -> target 9.3 kt vs 9.54 actual   no brake
///   rem 11.2 m -> target 6.5 kt vs 9.24 actual   no brake
///   rem  6.8 m -> target 5.1 kt vs 9.14 actual   first brake, far too late
///
/// The aircraft then crossed the hold line at 6.7 kt and coasted five metres
/// past it, nose over the yellow — a runway incursion, and with the steering
/// already released it stopped skewed as well (EGHI, photographed 2026-07-30).
///
/// Worse, `taxi_control` cannot tell a planned STOP from a bend trim: both are
/// just a target above 1 kt, so both get the 0.22 comfort cap that three
/// airframes' jerkiness complaints put there. Full 0.6 authority arrives only
/// inside the last metre. Rather than re-open that tuning, this starts the
/// deceleration early enough that comfort braking is enough: at 0.25 the same
/// 22.9 m commands 6.6 kt, the 3 kt gap opens around 15 m out, and there is
/// then twice the distance to stop in with ~2.8x the needed authority.
pub(super) const STOP_DECEL_MS2: f64 = 0.25;

/// The braking-distance speed cap approaching the stop point: v = √(2·a·d) at
/// [`STOP_DECEL_MS2`], floored at a creep so the last meters actually finish.
/// At/inside [`STOP_ZERO_M`] it's a stop.
pub fn stop_ramp_kts(remaining_m: f64) -> f64 {
    if remaining_m <= STOP_ZERO_M {
        return 0.0;
    }
    (MS_TO_KTS * (2.0 * STOP_DECEL_MS2 * remaining_m).sqrt()).max(CREEP_KTS)
}

/// Where the schedule commands a full stop, and the creep it will not go below
/// on the way in. These two are a PAIR, and getting them out of step is what
/// made the SF50 unable to line up at Luton (2026-07-28).
///
/// The old figures were a 2 kt creep held until half a metre to go, against an
/// arrival test of `remaining <= 1 m && gs < 1.5 kt`. So the schedule was still
/// DRIVING at 2 kt through the window the arrival test needed it under 1.5, and
/// 2 kt needs `v^2/2a` = 1.06 m to stop at the ramp's own 0.5 m/s^2 — more than
/// the distance left — before any braking lag. Every leg therefore sailed past
/// its stop point: the blackbox shows `remaining_m` pinned at -5 with the
/// aircraft still rolling, and because `stop_straighten` has faded the steering
/// out by then, it drifted while nobody was holding the nose. Three line-up
/// legs in a row finished 2.5 to 8.6 m off with the aim point behind them, and
/// full auto's `lined_up` check — correctly — called it a failure.
///
/// 1.2 kt needs 0.38 m, comfortably inside the 1 m zero. The creep still moves
/// (the reason it exists at all: too low and the last metres never finish), and
/// the zero now matches the arrival test's own 1 m so the two agree about where
/// the route ends.
pub(super) const CREEP_KTS: f64 = 1.2;
pub(super) const STOP_ZERO_M: f64 = 1.0;

/// The scheduled taxi target: the pilot's pace, capped by any nearby bend
/// and by the stop ramp.
pub fn schedule_kts(base: f64, route: &Route, progress: f64, gs_kts: f64) -> f64 {
    let mut v = base.min(stop_ramp_kts(route.remaining(progress)));
    if let Some(cap) = route.bend_cap_kts(progress, gs_kts) {
        v = v.min(cap);
    }
    v
}
