//! Drawn taxi centrelines (apt.dat 120 linear features) the routes ride.

use super::*;

/// apt.dat painted-line types that mean "taxiway centreline": solid yellow,
/// and the same line drawn with a black border (which is what most modern
/// scenery uses). Deliberately only these two — an edge line, a hold-short
/// bar or a roadway marking is not a line to drive down, and one wrong type
/// here snaps a route sideways onto a marking that merely runs nearby.
pub(super) const PAINT_CENTRELINE_TYPES: [&str; 2] = ["1", "51"];
/// The stand lead-in line type (see `TaxiNet::stand_lines`).
pub(super) const LEADIN_TYPE: &str = "60";

/// Points sampled along one bezier segment of a drawn line. The curves are
/// junction fillets a few tens of metres long, so eight is smooth to well
/// under the snap tolerance and keeps the polyline cheap to project onto.
pub(super) const BEZIER_STEPS: usize = 8;

/// One node of a drawn linear feature while it is being parsed.
#[derive(Clone, Copy)]
pub(super) struct PaintNode {
    pub(super) p: (f64, f64),
    /// The bezier control point, when the row carried one (112/114/116). The
    /// control on the far side of the node is its mirror image, which is what
    /// makes the drawn line smooth through the node.
    pub(super) ctrl: Option<(f64, f64)>,
    /// Does the segment STARTING here carry a centreline line type? apt.dat
    /// attaches the type to the node the segment leaves, and a string's
    /// terminating row (115/116) carries no type at all — so this is a
    /// property of the segment, read off its first node.
    pub(super) centre: bool,
}

/// Cubic bezier point at `t`.
pub(super) fn bezier(p0: (f64, f64), p1: (f64, f64), p2: (f64, f64), p3: (f64, f64), t: f64) -> (f64, f64) {
    let u = 1.0 - t;
    let (a, b, c, d) = (u * u * u, 3.0 * u * u * t, 3.0 * u * t * t, t * t * t);
    (
        a * p0.0 + b * p1.0 + c * p2.0 + d * p3.0,
        a * p0.1 + b * p1.1 + c * p2.1 + d * p3.1,
    )
}

/// Turn one parsed linear feature into the centreline polylines it contains,
/// beziers sampled. A feature can mix types (a taxiway line that becomes a
/// hold bar), so this emits a run per unbroken stretch of centreline rather
/// than one polyline per feature.
/// How near the aircraft a painted line must pass to be its stand's lane.
pub(super) const LANE_END_M: f64 = 20.0;
/// How far the line's direction may differ from the heading.
pub(super) const LANE_ALIGN_DEG: f64 = 35.0;
/// The shortest painted lane worth pushing along.
pub(super) const LANE_MIN_M: f64 = 15.0;

/// Even-odd point in a closed ring of (lat, lon).
pub(super) fn ring_contains(ring: &[(f64, f64)], lat: f64, lon: f64) -> bool {
    let mut inside = false;
    let n = ring.len();
    for i in 0..n {
        let (y1, x1) = ring[i];
        let (y2, x2) = ring[(i + 1) % n];
        if (y1 > lat) != (y2 > lat) && lon < x1 + (lat - y1) * (x2 - x1) / (y2 - y1) {
            inside = !inside;
        }
    }
    inside
}

pub(super) fn flush_paint_feature(nodes: &mut Vec<PaintNode>, out: &mut Vec<Vec<(f64, f64)>>) {
    let mut run: Vec<(f64, f64)> = Vec::new();
    for i in 0..nodes.len().saturating_sub(1) {
        let (a, b) = (nodes[i], nodes[i + 1]);
        if !a.centre {
            if run.len() >= 2 {
                out.push(std::mem::take(&mut run));
            } else {
                run.clear();
            }
            continue;
        }
        if run.is_empty() {
            run.push(a.p);
        }
        match (a.ctrl, b.ctrl) {
            (None, None) => run.push(b.p),
            (ca, cb) => {
                let p1 = ca.unwrap_or(a.p);
                // The incoming control is the outgoing one mirrored through
                // the node — that mirror is the whole reason apt.dat stores
                // only one per node.
                let p2 = cb.map_or(b.p, |c| (2.0 * b.p.0 - c.0, 2.0 * b.p.1 - c.1));
                for k in 1..=BEZIER_STEPS {
                    run.push(bezier(a.p, p1, p2, b.p, k as f64 / BEZIER_STEPS as f64));
                }
            }
        }
    }
    if run.len() >= 2 {
        out.push(run);
    }
    nodes.clear();
}

/// A route polyline with its per-leg `(name, is_runway)`, aligned the way
/// [`TaxiRoutePlan::seg`] is: `seg[i]` describes `path[i] -> path[i+1]`.
pub type RoutePolyline = (Vec<(f64, f64)>, Vec<(String, bool)>);

/// The drawn centreline nearest `p` that could be a continuation of a leg
/// running on bearing `brg`: its index in `paint`, the foot of the
/// perpendicular, and how far along the line (m from its first point) the
/// foot sits. `prefer` is the line the previous sample snapped to, which
/// wins whenever it is still a candidate — without that, two lines drawn a
/// metre apart (a taxiway and the stand lead-in leaving it) swap the route
/// between them sample by sample and saw-tooth a straight leg.
pub(super) fn nearest_paint(
    paint: &[Vec<(f64, f64)>],
    p: (f64, f64),
    brg: f64,
    prefer: Option<usize>,
    m_per_deg_lon: f64,
) -> Option<(usize, (f64, f64), f64)> {
    let mut best: Option<(f64, usize, (f64, f64), f64)> = None;
    let mut best_prefer: Option<(f64, usize, (f64, f64), f64)> = None;
    for (idx, line) in paint.iter().enumerate() {
        let mut along = 0.0;
        for w in line.windows(2) {
            let (ax, ay) = ((w[0].1 - p.1) * m_per_deg_lon, (w[0].0 - p.0) * 111_320.0);
            let (bx, by) = ((w[1].1 - p.1) * m_per_deg_lon, (w[1].0 - p.0) * 111_320.0);
            let (dx, dy) = (bx - ax, by - ay);
            let len2 = dx * dx + dy * dy;
            let seg_start = along;
            along += len2.sqrt();
            if len2 < 1e-6 {
                continue;
            }
            let t = (-(ax * dx + ay * dy) / len2).clamp(0.0, 1.0);
            let (fx, fy) = (ax + t * dx, ay + t * dy);
            let d = (fx * fx + fy * fy).sqrt();
            if d > PAINT_SNAP_M {
                continue;
            }
            // Undirected: a line drawn from the other end is the same line.
            let gap = (dx.atan2(dy).to_degrees() - brg).rem_euclid(180.0);
            if gap.min(180.0 - gap) > PAINT_PARALLEL_DEG {
                continue;
            }
            let foot = (p.0 + fy / 111_320.0, p.1 + fx / m_per_deg_lon);
            let at = seg_start + t * len2.sqrt();
            if best.is_none_or(|(bd, ..)| d < bd) {
                best = Some((d, idx, foot, at));
            }
            if prefer == Some(idx) && best_prefer.is_none_or(|(bd, ..)| d < bd) {
                best_prefer = Some((d, idx, foot, at));
            }
        }
    }
    best_prefer.or(best).map(|(_, idx, foot, at)| (idx, foot, at))
}

/// How much longer than the straight gap a drawn line may run between two
/// samples snapped onto it and still be the way the route goes there. A
/// quarter circle is 1.11 times its chord; a line that wanders further is
/// going somewhere else.
pub(super) const PAINT_FILL_DETOUR: f64 = 1.5;

/// The piece of drawn line `line` between the feet at `from_m` and `to_m`
/// metres along it, EXCLUSIVE of both ends, in the route's direction (the
/// line may be drawn either way), resampled so no gap exceeds
/// [`PAINT_SAMPLE_M`]. Returns the points and the piece's length.
pub(super) fn paint_between(
    line: &[(f64, f64)],
    from_m: f64,
    to_m: f64,
    m_per_deg_lon: f64,
) -> (Vec<(f64, f64)>, f64) {
    let dist = |a: (f64, f64), b: (f64, f64)| {
        let (dx, dy) = ((a.1 - b.1) * m_per_deg_lon, (a.0 - b.0) * 111_320.0);
        (dx * dx + dy * dy).sqrt()
    };
    // The line's vertices with how far along it each sits.
    let mut at = Vec::with_capacity(line.len());
    let mut acc = 0.0;
    for (i, &p) in line.iter().enumerate() {
        if i > 0 {
            acc += dist(line[i - 1], p);
        }
        at.push(acc);
    }
    let point_at = |m: f64| -> (f64, f64) {
        let k = at.partition_point(|&a| a <= m).clamp(1, line.len() - 1);
        let (a, b) = (line[k - 1], line[k]);
        let span = at[k] - at[k - 1];
        let t = if span > 1e-9 { ((m - at[k - 1]) / span).clamp(0.0, 1.0) } else { 0.0 };
        (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t)
    };
    let (lo, hi) = (from_m.min(to_m), from_m.max(to_m));
    // Stations along the piece: every vertex strictly inside, plus fill-ins
    // so a long straight stays sampled like any other leg.
    let mut stations: Vec<f64> = Vec::new();
    let mut prev = lo;
    for &a in at.iter().filter(|&&a| a > lo + 0.25 && a < hi - 0.25) {
        let n = ((a - prev) / PAINT_SAMPLE_M).ceil() as usize;
        for k in 1..n {
            stations.push(prev + (a - prev) * k as f64 / n as f64);
        }
        stations.push(a);
        prev = a;
    }
    let n = ((hi - prev) / PAINT_SAMPLE_M).ceil() as usize;
    for k in 1..n {
        stations.push(prev + (hi - prev) * k as f64 / n as f64);
    }
    if to_m < from_m {
        stations.reverse();
    }
    (stations.into_iter().map(point_at).collect(), hi - lo)
}

/// A route's first edge is joined AHEAD of the aircraft's projection onto
/// it, never at the snapped node behind it. The planner snaps the start to
/// the nearest node, and when the aircraft sits abeam the middle of an edge
/// that node is behind it along the way the route goes: the route then runs
/// back to the node and hairpins onto the edge. LILB ramp, Lancair Evolution
/// (1790712974, 1790720549, 1790721973; 2026-09-29 and 30): parked at
/// 45.771275 9.164289 facing 28.9, the nearest node 16.7 m away on a bearing
/// of 98, the edge beyond it running 335: a 124 degree reversal 17 m from
/// brake release, flown at 8 to 9 kt with the projection jumping onto the
/// second leg, and every taxi failed 14.2 to 16.3 m off the line. Now the
/// route joins the edge past the projection by the offset itself (bounded
/// by [`JOIN_AHEAD_LEAD_M`] and [`JOIN_AHEAD_MAX_LEAD_M`]: a 45 degree
/// join), and an edge wholly behind the projection is dropped. Only the network's first taxiway leg, only when
/// the aircraft is within [`JOIN_AHEAD_MAX_LAT_M`] of it. Pure.
pub fn join_first_edge_ahead(
    path: &mut Vec<(f64, f64)>,
    seg: &mut Vec<(String, bool)>,
    here: (f64, f64),
) {
    let m_per_deg_lon = 111_320.0 * here.0.to_radians().cos();
    let to_m = |p: (f64, f64)| ((p.1 - here.1) * m_per_deg_lon, (p.0 - here.0) * 111_320.0);
    while path.len() >= 2 && seg.len() + 1 == path.len() {
        if seg[0].1 {
            return; // a runway leg is the runway's geometry
        }
        let (a, b) = (to_m(path[0]), to_m(path[1]));
        let (ex, ey) = (b.0 - a.0, b.1 - a.1);
        let len = (ex * ex + ey * ey).sqrt();
        if len < 0.5 {
            return;
        }
        // The aircraft is the origin: its projection along a->b, and its
        // distance off the edge's line.
        let t = (-a.0 * ex - a.1 * ey) / len;
        let off = (a.0 * ey - a.1 * ex).abs() / len;
        if off > JOIN_AHEAD_MAX_LAT_M || t < JOIN_AHEAD_MIN_M {
            return;
        }
        let join = t + off.clamp(JOIN_AHEAD_LEAD_M, JOIN_AHEAD_MAX_LEAD_M);
        if join >= len - JOIN_AHEAD_MIN_M {
            // The whole edge is behind the join: its far node is the join
            // (a route never shorter than two points), then look again.
            if path.len() <= 2 {
                return;
            }
            path.remove(0);
            seg.remove(0);
            continue;
        }
        let k = join / len;
        let (p0, p1) = (path[0], path[1]);
        path[0] = (p0.0 + (p1.0 - p0.0) * k, p0.1 + (p1.1 - p0.1) * k);
        return;
    }
}
/// The aircraft's projection must be this far past the edge's start before
/// the node counts as behind it, and a join this close to the edge's end is
/// the end node.
pub(super) const JOIN_AHEAD_MIN_M: f64 = 3.0;
/// The join sits this far past the projection at least, and at most
/// [`JOIN_AHEAD_MAX_LEAD_M`]: the offset itself between, a 45 degree join.
pub(super) const JOIN_AHEAD_LEAD_M: f64 = 5.0;
pub(super) const JOIN_AHEAD_MAX_LEAD_M: f64 = 30.0;
/// Further off the first edge than this, the start is not beside it: the
/// planner's node stands.
pub(super) const JOIN_AHEAD_MAX_LAT_M: f64 = 60.0;

/// Move a planned route onto the paint it is an approximation of, returning
/// the resampled `(path, seg)`.
///
/// The route out of the planner is a chain of 1201 nodes: the right taxiways
/// in the right order, drawn by hand to within a few metres of the yellow
/// line. Following it exactly — which the drive does, to 0.12 m — puts the
/// aircraft a couple of metres to one side of the line for the whole taxi,
/// which is what the pilot sees and what a ground crew would fail you for.
/// So each leg is sampled and each sample slid sideways onto the drawn
/// centreline, when there is one running the same way close enough to be the
/// same line.
///
/// It can only ever move a point `PAINT_SNAP_M` sideways onto a line the leg
/// was already following, never re-route: the topology, the names, the
/// crossings and the holding point are the network's and are untouched. A
/// field that draws no centrelines (or a net built without them) gets its
/// route back unchanged, not a refusal.
///
/// Runway legs are left alone. A crossing runs square across every taxi line
/// near it, the strip's own markings are not 120 features anyway, and the
/// runway geometry is authoritative where the taxi network is not.
///
/// The one place a point moves further: a network corner cut across a drawn
/// fillet turns on that fillet ([`ride_corner_fillets`], #194).
pub fn ride_paint(
    paint: &[Vec<(f64, f64)>],
    path: &[(f64, f64)],
    seg: &[(String, bool)],
) -> RoutePolyline {
    ride_paint_with(paint, path, seg, true)
}

/// [`ride_paint`], with the corners rounded on their fillets or not (the
/// tests compare the two).
pub(super) fn ride_paint_with(
    paint: &[Vec<(f64, f64)>],
    path: &[(f64, f64)],
    seg: &[(String, bool)],
    fillets: bool,
) -> RoutePolyline {
    if paint.is_empty() || path.len() < 2 {
        return (path.to_vec(), seg.to_vec());
    }
    let m_per_deg_lon = 111_320.0 * path[0].0.to_radians().cos();
    let dist_m = |a: (f64, f64), b: (f64, f64)| {
        let (dx, dy) = ((a.1 - b.1) * m_per_deg_lon, (a.0 - b.0) * 111_320.0);
        (dx * dx + dy * dy).sqrt()
    };
    let mut out_path: Vec<(f64, f64)> = Vec::with_capacity(path.len() * 4);
    let mut out_seg: Vec<(String, bool)> = Vec::with_capacity(path.len() * 4);
    // Which line each output point sits on, and where along it.
    let mut out_snap: Vec<Option<(usize, f64)>> = Vec::with_capacity(path.len() * 4);
    // Which network leg each output point was sampled from (the end point
    // is the last node's), for the corners (`ride_corner_fillets`).
    let mut out_leg: Vec<usize> = Vec::with_capacity(path.len() * 4);
    let mut prefer: Option<usize> = None;
    for i in 0..path.len() - 1 {
        let (a, b) = (path[i], path[i + 1]);
        let s = seg.get(i).cloned().unwrap_or_default();
        let len = dist_m(a, b);
        // A zero-length leg is no leg: kept, it reads as a bearing of 0,
        // due north, to anything that asks a segment its direction (the
        // LIPR stop swerved on one, 2026-09-26).
        if len < 0.5 {
            continue;
        }
        if s.1 {
            out_path.push(a);
            out_seg.push(s);
            out_snap.push(None);
            out_leg.push(i);
            prefer = None;
            continue;
        }
        let brg = bearing_deg(a.0, a.1, b.0, b.1);
        let n = ((len / PAINT_SAMPLE_M).round() as usize).max(1);
        for k in 0..n {
            let t = k as f64 / n as f64;
            let raw = (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t);
            let hit = nearest_paint(paint, raw, brg, prefer, m_per_deg_lon);
            let snapped = match hit {
                Some((idx, q, _)) => {
                    prefer = Some(idx);
                    q
                }
                None => raw,
            };
            // Progress along the route must stay monotonic: every consumer
            // projects onto this polyline and reads how far along it is, and
            // a sample slid BACKWARDS past the one before it would read as
            // the aircraft losing ground. Nothing in the gates above forbids
            // it outright, so the cheap answer is to keep the raw sample —
            // one unsnapped point in the middle of a snapped run is invisible.
            let keeps_going = out_path
                .last()
                .is_none_or(|&prev| dist_m(prev, snapped) > 0.25 && {
                    let step = bearing_deg(prev.0, prev.1, snapped.0, snapped.1);
                    let gap = (step - brg).rem_euclid(360.0);
                    gap.min(360.0 - gap) < 90.0
                });
            out_path.push(if keeps_going { snapped } else { raw });
            out_seg.push(s.clone());
            out_snap.push(hit.filter(|_| keeps_going).map(|(idx, _, at)| (idx, at)));
            out_leg.push(i);
        }
    }
    // The end point rides the last leg's bearing, so a route that finishes at
    // a holding point finishes ON the line, which is where it will stop.
    let last = path[path.len() - 1];
    let prev = path[path.len() - 2];
    let (end, end_snap) = if seg.last().is_some_and(|s| s.1) {
        (last, None)
    } else {
        let brg = bearing_deg(prev.0, prev.1, last.0, last.1);
        nearest_paint(paint, last, brg, prefer, m_per_deg_lon)
            .map_or((last, None), |(idx, q, at)| (q, Some((idx, at))))
    };
    // The end snapped onto the last sample is the same point twice: merge.
    if out_path.len() >= 2 && out_path.last().is_some_and(|&p| dist_m(p, end) < 0.5) {
        out_path.pop();
        out_seg.pop();
        out_snap.pop();
        out_leg.pop();
    }
    out_path.push(end);
    out_snap.push(end_snap);
    out_leg.push(path.len() - 1);
    if fillets {
        ride_corner_fillets(
            paint,
            path,
            seg,
            &mut out_path,
            &mut out_seg,
            &mut out_snap,
            &mut out_leg,
            m_per_deg_lon,
        );
    }
    fill_paint_gaps(paint, &mut out_path, &mut out_seg, &out_snap, m_per_deg_lon);
    (out_path, out_seg)
}

/// A network corner turning less than this is left to the sideways snap.
pub(super) const FILLET_MIN_TURN_DEG: f64 = 30.0;
/// How far from the corner's node a fillet's ends are looked for, m.
pub(super) const FILLET_REACH_M: f64 = 80.0;
/// Two drawn lines meet where an end of one is this close to the other, m.
pub(super) const PAINT_JOIN_M: f64 = 1.0;
/// The most drawn lines one fillet is pieced from.
pub(super) const FILLET_MAX_LINES: usize = 4;
/// A fillet turns one way: a vertex turning back against the corner by more
/// than this is a different line, degrees.
pub(super) const FILLET_WOBBLE_DEG: f64 = 10.0;
/// A fillet leaves along the leg in and arrives along the leg out, within
/// this, degrees.
pub(super) const FILLET_END_DEG: f64 = 45.0;

/// A drawn line's vertices with how far along it each sits, m.
pub(super) fn line_stations(line: &[(f64, f64)], m_per_deg_lon: f64) -> Vec<f64> {
    let mut at = Vec::with_capacity(line.len());
    let mut acc = 0.0;
    for (i, &p) in line.iter().enumerate() {
        if i > 0 {
            let (dx, dy) = ((p.1 - line[i - 1].1) * m_per_deg_lon, (p.0 - line[i - 1].0) * 111_320.0);
            acc += (dx * dx + dy * dy).sqrt();
        }
        at.push(acc);
    }
    at
}

/// The point `m` metres along a drawn line.
pub(super) fn line_point(line: &[(f64, f64)], at: &[f64], m: f64) -> (f64, f64) {
    if line.len() < 2 {
        return line[0];
    }
    let k = at.partition_point(|&a| a <= m).clamp(1, line.len() - 1);
    let (a, b) = (line[k - 1], line[k]);
    let span = at[k] - at[k - 1];
    let t = if span > 1e-9 { ((m - at[k - 1]) / span).clamp(0.0, 1.0) } else { 0.0 };
    (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t)
}

/// Round a route's corners on the drawn fillet the network cut.
///
/// A taxiway meeting another at a T is drawn, in the network, as one node
/// where the two CENTRELINES cross; the paint instead stops short of the
/// junction and sweeps round a fillet onto the other line, one each way.
/// The route out of the planner runs straight to the node and turns there,
/// and the sideways snap (at most [`PAINT_SNAP_M`], within
/// [`PAINT_PARALLEL_DEG`] of the leg) takes each sample near the junction
/// onto whichever fillet leaves closest, left or right, sample by sample.
/// LEMG's Quebec onto Delta (Citation X, sorties 1790734216 and 1790738786,
/// 2026-09-30, #194): Quebec's paint ends 47 m short of Delta's centreline,
/// and the ridden route ran 36, 21, 19, 69, 114, 2, 41 and 318 degrees in
/// seven legs of 5 to 10 m; the drive chased the jog right, swung 4.6 and
/// 4.8 m outside the turn back and failed the taxi both times.
///
/// So at each network corner of [`FILLET_MIN_TURN_DEG`] or more the samples
/// between a sample on the paint before it and a sample on the paint after
/// it are replaced by the drawn lines that JOIN those two, end to end: the
/// shortest chain of at most [`FILLET_MAX_LINES`] lines that turns the
/// corner's way the whole time (never back more than [`FILLET_WOBBLE_DEG`]),
/// leaves and arrives along the legs, lies inside the corner (within
/// [`PAINT_SNAP_M`] of the triangle its two ends make with the node) and is
/// no longer than [`PAINT_FILL_DETOUR`] times the gap. That is the fillet
/// the corner turns on and never the other one, which runs outside the
/// node. No such chain, and the corner is left as the snap made it: runway
/// legs, undrawn fields and junctions drawn square are untouched.
#[allow(clippy::too_many_arguments)]
pub(super) fn ride_corner_fillets(
    paint: &[Vec<(f64, f64)>],
    path: &[(f64, f64)],
    seg: &[(String, bool)],
    out_path: &mut Vec<(f64, f64)>,
    out_seg: &mut Vec<(String, bool)>,
    out_snap: &mut Vec<Option<(usize, f64)>>,
    out_leg: &mut Vec<usize>,
    m_per_deg_lon: f64,
) {
    if paint.is_empty()
        || path.len() < 3
        || out_path.len() != out_snap.len()
        || out_path.len() != out_leg.len()
        || out_seg.len() + 1 != out_path.len()
    {
        return;
    }
    let dist = |a: (f64, f64), b: (f64, f64)| {
        let (dx, dy) = ((a.1 - b.1) * m_per_deg_lon, (a.0 - b.0) * 111_320.0);
        (dx * dx + dy * dy).sqrt()
    };
    let brg = |a: (f64, f64), b: (f64, f64)| bearing_deg(a.0, a.1, b.0, b.1);
    let stations: Vec<Vec<f64>> = paint.iter().map(|l| line_stations(l, m_per_deg_lon)).collect();
    for k in 1..path.len() - 1 {
        let v = path[k];
        if seg.get(k - 1).is_none_or(|s| s.1) || seg.get(k).is_none_or(|s| s.1) {
            continue;
        }
        if dist(path[k - 1], v) < 0.5 || dist(v, path[k + 1]) < 0.5 {
            continue;
        }
        let (b_in, b_out) = (brg(path[k - 1], v), brg(v, path[k + 1]));
        let turn = sim::angles::signed_diff_deg(b_out, b_in);
        if turn.abs() < FILLET_MIN_TURN_DEG {
            continue;
        }
        let Some(first_out) = out_leg.iter().position(|&l| l >= k) else {
            continue;
        };
        // Exits: samples after the node, on the paint, within reach.
        let exits: Vec<(usize, usize, f64)> = (first_out..out_path.len())
            .take_while(|&j| dist(out_path[j], v) <= FILLET_REACH_M || j == first_out)
            .filter_map(|j| out_snap[j].map(|(l, at)| (j, l, at)))
            .collect();
        if exits.is_empty() {
            continue;
        }
        // Entries: samples on the leg in, on the paint, furthest first, so
        // every sample the snap threw between the two fillets is replaced.
        let entries: Vec<usize> = (0..first_out)
            .rev()
            .take_while(|&i| out_leg[i] + 1 == k && dist(out_path[i], v) <= FILLET_REACH_M)
            .filter(|&i| out_snap[i].is_some())
            .collect();
        type FilletPath = Vec<(f64, f64)>;
        type ChosenFillet = (usize, usize, FilletPath);
        type PaintWalk = (usize, f64, f64, Vec<(usize, f64, f64)>);
        let mut chosen: Option<ChosenFillet> = None;
        for &i in entries.iter().rev() {
            let (li, at_i) = out_snap[i].expect("filtered");
            let fits = |pts: &[(f64, f64)], j: usize| -> Option<f64> {
                let len = fillet_fits(pts, out_path[i], v, out_path[j], b_in, b_out, turn, m_per_deg_lon)?;
                // Spliced in, the fillet must meet the samples either side
                // as smoothly as the route it replaces: no joint sharper
                // than the sharper of [`FILLET_MAX_VERTEX_DEG`] and the old
                // span's own, and none turning back against the corner.
                let lo = i.saturating_sub(1);
                let hi = (j + 1).min(out_path.len() - 1);
                let old_max = vertex_turns(&out_path[lo..=hi], m_per_deg_lon)
                    .iter()
                    .fold(0.0_f64, |a, t| a.max(t.abs()));
                let mut ext: Vec<(f64, f64)> = Vec::with_capacity(pts.len() + 2);
                ext.extend_from_slice(&out_path[lo..i]);
                ext.extend_from_slice(pts);
                ext.extend_from_slice(&out_path[j + 1..=hi]);
                let new = vertex_turns(&ext, m_per_deg_lon);
                let smooth = new.iter().all(|t| {
                    t.abs() <= old_max.max(FILLET_MAX_VERTEX_DEG) && t * turn.signum() >= -FILLET_WOBBLE_DEG
                });
                smooth.then_some(len)
            };
            let mut best: Option<(f64, usize, FilletPath)> = None;
            let dir_i = line_dir(&paint[li], &stations[li], at_i, b_in);
            let mut stack: Vec<PaintWalk> = vec![(li, at_i, dir_i, Vec::new())];
            while let Some((line, at, dir, pieces)) = stack.pop() {
                let len = *stations[line].last().unwrap_or(&0.0);
                let end_at = if dir > 0.0 { len } else { 0.0 };
                let p = line_point(&paint[line], &stations[line], end_at);
                let mut here = pieces.clone();
                here.push((line, at, end_at));
                let used = |l: usize| here.iter().any(|&(u, ..)| u == l);
                // Onto an exit's line: the first exit ahead of where we meet it.
                let mut tried: Vec<usize> = Vec::new();
                for &(j, lj, at_j) in &exits {
                    if used(lj) || tried.contains(&lj) {
                        continue;
                    }
                    let Some(at_e) = project_on_line(&paint[lj], &stations[lj], p, m_per_deg_lon)
                        .filter(|&(_, d)| d <= PAINT_JOIN_M)
                        .map(|(a, _)| a)
                    else {
                        continue;
                    };
                    let dir_o = line_dir(&paint[lj], &stations[lj], at_e, b_out);
                    if (at_j - at_e) * dir_o <= 0.5 {
                        continue;
                    }
                    tried.push(lj);
                    let mut chain = here.clone();
                    chain.push((lj, at_e, at_j));
                    let pts = chain_points(paint, &stations, &chain, m_per_deg_lon);
                    let mut all = vec![out_path[i]];
                    all.extend(pts.iter().copied());
                    all.push(out_path[j]);
                    if let Some(len) = fits(&all, j) {
                        if best.as_ref().is_none_or(|b| len < b.0) {
                            best = Some((len, j, pts));
                        }
                    }
                }
                if here.len() >= FILLET_MAX_LINES {
                    continue;
                }
                for (l2, line2) in paint.iter().enumerate() {
                    if used(l2) || line2.len() < 2 {
                        continue;
                    }
                    let len2 = *stations[l2].last().unwrap_or(&0.0);
                    if dist(p, line2[0]) <= PAINT_JOIN_M {
                        stack.push((l2, 0.0, 1.0, here.clone()));
                    } else if dist(p, line2[line2.len() - 1]) <= PAINT_JOIN_M {
                        stack.push((l2, len2, -1.0, here.clone()));
                    }
                }
            }
            if let Some((_, j, pts)) = best {
                chosen = Some((i, j, pts));
                break;
            }
        }
        let Some((i, j, pts)) = chosen else {
            continue;
        };
        if out_seg[i..j].iter().any(|s| s.1) {
            continue;
        }
        // Each new point takes the name and leg of the replaced sample
        // nearest it, so a name change at the node lands where it was.
        let nearest = |q: (f64, f64)| {
            (i..j)
                .min_by(|&a, &b| dist(out_path[a], q).total_cmp(&dist(out_path[b], q)))
                .unwrap_or(i)
        };
        let new_seg: Vec<(String, bool)> = std::iter::once(out_seg[i].clone())
            .chain(pts.iter().map(|&q| out_seg[nearest(q)].clone()))
            .collect();
        let new_leg: Vec<usize> = pts.iter().map(|&q| out_leg[nearest(q)]).collect();
        let n = pts.len();
        out_path.splice(i + 1..j, pts);
        out_seg.splice(i..j, new_seg);
        out_snap.splice(i + 1..j, std::iter::repeat_n(None, n));
        out_leg.splice(i + 1..j, new_leg);
    }
}

/// +1 when a drawn line runs the way `brg` goes at `at` metres along it,
/// else -1 (a line may be drawn from either end).
pub(super) fn line_dir(line: &[(f64, f64)], at: &[f64], m: f64, brg: f64) -> f64 {
    let len = *at.last().unwrap_or(&0.0);
    let (a, b) = (
        line_point(line, at, (m - 1.0).max(0.0)),
        line_point(line, at, (m + 1.0).min(len)),
    );
    let tangent = bearing_deg(a.0, a.1, b.0, b.1);
    if sim::angles::abs_diff_deg(tangent, brg) <= 90.0 {
        1.0
    } else {
        -1.0
    }
}

/// The foot of `p` on a drawn line: (metres along it, distance off it).
pub(super) fn project_on_line(line: &[(f64, f64)], at: &[f64], p: (f64, f64), m_per_deg_lon: f64) -> Option<(f64, f64)> {
    let mut best: Option<(f64, f64)> = None;
    for (k, w) in line.windows(2).enumerate() {
        let (ax, ay) = ((w[0].1 - p.1) * m_per_deg_lon, (w[0].0 - p.0) * 111_320.0);
        let (bx, by) = ((w[1].1 - p.1) * m_per_deg_lon, (w[1].0 - p.0) * 111_320.0);
        let (dx, dy) = (bx - ax, by - ay);
        let len2 = dx * dx + dy * dy;
        let t = if len2 < 1e-9 { 0.0 } else { (-(ax * dx + ay * dy) / len2).clamp(0.0, 1.0) };
        let (fx, fy) = (ax + t * dx, ay + t * dy);
        let d = (fx * fx + fy * fy).sqrt();
        if best.is_none_or(|(_, bd)| d < bd) {
            best = Some((at[k] + t * len2.sqrt(), d));
        }
    }
    best
}

/// The points of a chain of line pieces `(line, from_m, to_m)`, exclusive
/// of the chain's own two ends, each joint included once.
pub(super) fn chain_points(
    paint: &[Vec<(f64, f64)>],
    stations: &[Vec<f64>],
    chain: &[(usize, f64, f64)],
    m_per_deg_lon: f64,
) -> Vec<(f64, f64)> {
    let mut pts = Vec::new();
    for (n, &(line, from, to)) in chain.iter().enumerate() {
        if (to - from).abs() > 0.25 {
            pts.extend(paint_between(&paint[line], from, to, m_per_deg_lon).0);
        }
        if n + 1 < chain.len() && (to - from).abs() > 0.25 {
            pts.push(line_point(&paint[line], &stations[line], to));
        }
    }
    pts
}

/// Whether the polyline `pts` (entry first, exit last) is the fillet of the
/// corner at `v` turning `turn` degrees from `b_in` to `b_out`: its length
/// if so. See [`ride_corner_fillets`].
#[allow(clippy::too_many_arguments)]
pub(super) fn fillet_fits(
    pts: &[(f64, f64)],
    entry: (f64, f64),
    v: (f64, f64),
    exit: (f64, f64),
    b_in: f64,
    b_out: f64,
    turn: f64,
    m_per_deg_lon: f64,
) -> Option<f64> {
    let xy = |p: (f64, f64)| ((p.1 - v.1) * m_per_deg_lon, (p.0 - v.0) * 111_320.0);
    let d = |a: (f64, f64), b: (f64, f64)| {
        let (a, b) = (xy(a), xy(b));
        ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt()
    };
    let mut len = 0.0;
    let mut brgs: Vec<f64> = Vec::new();
    let mut last = pts[0];
    for &q in &pts[1..] {
        let step = d(last, q);
        len += step;
        if step >= 0.5 {
            brgs.push(bearing_deg(last.0, last.1, q.0, q.1));
            last = q;
        }
    }
    let (first, final_) = (*brgs.first()?, *brgs.last()?);
    if len > PAINT_FILL_DETOUR * d(entry, exit) + 5.0 {
        return None;
    }
    if sim::angles::abs_diff_deg(first, b_in) > FILLET_END_DEG || sim::angles::abs_diff_deg(final_, b_out) > FILLET_END_DEG {
        return None;
    }
    let side = turn.signum();
    if brgs.windows(2).any(|w| sim::angles::signed_diff_deg(w[1], w[0]) * side < -FILLET_WOBBLE_DEG) {
        return None;
    }
    let tri = [xy(entry), (0.0, 0.0), xy(exit)];
    if pts.iter().any(|&q| off_triangle_m(xy(q), tri) > PAINT_SNAP_M) {
        return None;
    }
    Some(len)
}

/// The most one vertex of a spliced fillet may turn, unless the route it
/// replaces already turned more, degrees. A drawn fillet sampled every
/// [`PAINT_SAMPLE_M`] turns a few degrees per vertex.
pub(super) const FILLET_MAX_VERTEX_DEG: f64 = 30.0;

/// The turn at each vertex of a polyline, degrees signed right, over steps
/// of half a metre or more.
pub(super) fn vertex_turns(pts: &[(f64, f64)], m_per_deg_lon: f64) -> Vec<f64> {
    let mut out = Vec::new();
    let mut prev: Option<f64> = None;
    let Some(&first) = pts.first() else {
        return out;
    };
    let mut last = first;
    for &q in &pts[1..] {
        let (dx, dy) = ((q.1 - last.1) * m_per_deg_lon, (q.0 - last.0) * 111_320.0);
        if (dx * dx + dy * dy).sqrt() < 0.5 {
            continue;
        }
        let b = bearing_deg(last.0, last.1, q.0, q.1);
        if let Some(p) = prev {
            out.push(sim::angles::signed_diff_deg(b, p));
        }
        prev = Some(b);
        last = q;
    }
    out
}

/// How far `p` lies outside the triangle `t` (0 inside), in its own units.
pub(super) fn off_triangle_m(p: (f64, f64), t: [(f64, f64); 3]) -> f64 {
    let cross = |a: (f64, f64), b: (f64, f64), c: (f64, f64)| (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
    let (c1, c2, c3) = (cross(t[0], t[1], p), cross(t[1], t[2], p), cross(t[2], t[0], p));
    if (c1 >= 0.0 && c2 >= 0.0 && c3 >= 0.0) || (c1 <= 0.0 && c2 <= 0.0 && c3 <= 0.0) {
        return 0.0;
    }
    let to_seg = |a: (f64, f64), b: (f64, f64)| {
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let len2 = dx * dx + dy * dy;
        let t = if len2 < 1e-9 { 0.0 } else { (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len2).clamp(0.0, 1.0) };
        ((a.0 + t * dx - p.0).powi(2) + (a.1 + t * dy - p.1).powi(2)).sqrt()
    };
    to_seg(t[0], t[1]).min(to_seg(t[1], t[2])).min(to_seg(t[2], t[0]))
}

/// Close the gaps in a ridden route where the drawn line bulged out of reach.
///
/// A network leg is a CHORD of the drawn curve it approximates, and a curve
/// that sweeps round a junction runs further than [`PAINT_SNAP_M`] from its
/// own chord in the middle: the samples there cannot be moved and stay on
/// the chord while their neighbours sit on the line. The route then zigzags
/// between the two, a vertex of 40 to 60 degrees every five metres. LEMG 13's
/// lead-on did it (2026-09-26, sortie 1790441293): 6 m off the drawn curve in
/// the middle, the line-up's projection jumped 6 and 11 m forward in single
/// ticks and the heading error swung to -43 degrees.
///
/// So a run of unmoved samples between two samples on the SAME line is
/// replaced by that line between their feet, when it is the way the route
/// goes there: no runway leg inside (a runway is never ridden), not more than
/// [`PAINT_FILL_DETOUR`] times the straight gap, and never more than twice
/// the snap distance from the samples it replaces (a line that leaves and
/// comes back is a different line).
pub(super) fn fill_paint_gaps(
    paint: &[Vec<(f64, f64)>],
    path: &mut Vec<(f64, f64)>,
    seg: &mut Vec<(String, bool)>,
    snap: &[Option<(usize, f64)>],
    m_per_deg_lon: f64,
) {
    let dist = |a: (f64, f64), b: (f64, f64)| {
        let (dx, dy) = ((a.1 - b.1) * m_per_deg_lon, (a.0 - b.0) * 111_320.0);
        (dx * dx + dy * dy).sqrt()
    };
    if path.len() != snap.len() || seg.len() + 1 != path.len() {
        return;
    }
    let mut out_path: Vec<(f64, f64)> = Vec::with_capacity(path.len());
    let mut out_seg: Vec<(String, bool)> = Vec::with_capacity(seg.len());
    let mut i = 0;
    while i < path.len() {
        out_path.push(path[i]);
        let next = (i + 1..path.len()).find(|&j| snap[j].is_some());
        let fill = match (snap[i], next) {
            (Some((li, from_m)), Some(j)) if j > i + 1 => match snap[j] {
                Some((lj, to_m)) if lj == li && (to_m - from_m).abs() > 0.5 => {
                    let (piece, len) = paint_between(&paint[li], from_m, to_m, m_per_deg_lon);
                    let straight = dist(path[i], path[j]);
                    let runway = seg[i..j].iter().any(|s| s.1);
                    let near = path[i + 1..j].iter().all(|&p| {
                        piece
                            .iter()
                            .chain([&path[i], &path[j]])
                            .any(|&q| dist(p, q) <= 2.0 * PAINT_SNAP_M)
                    });
                    (!runway && near && len <= PAINT_FILL_DETOUR * straight + 5.0)
                        .then_some((j, piece))
                }
                _ => None,
            },
            _ => None,
        };
        match fill {
            Some((j, piece)) => {
                // Each new leg keeps the name of the replaced sample nearest
                // it, so a name change at a network node inside the gap
                // lands where the node was.
                let name_near = |q: (f64, f64)| {
                    (i..j)
                        .min_by(|&a, &b| dist(path[a], q).total_cmp(&dist(path[b], q)))
                        .map_or_else(|| seg[i].clone(), |k| seg[k].clone())
                };
                out_seg.push(seg[i].clone());
                for &q in &piece {
                    out_path.push(q);
                    out_seg.push(name_near(q));
                }
                i = j;
            }
            None => {
                if let Some(s) = seg.get(i) {
                    out_seg.push(s.clone());
                }
                i += 1;
            }
        }
    }
    *path = out_path;
    *seg = out_seg;
}
