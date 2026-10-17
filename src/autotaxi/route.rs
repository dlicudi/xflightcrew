//! The route driven: its runway spans and bends.

use super::*;

/// The cleared route, pre-digested into the local flat-earth frame the
/// steering law works in.
pub struct Route {
    /// Lat/lon polyline, for projecting fixes (same code path as the guide).
    pub(super) path: Vec<(f64, f64)>,
    /// The polyline in meters (x east, y north) from `path[0]`.
    pub(super) pts: Vec<(f64, f64)>,
    /// Cumulative along-route distance per vertex.
    pub(super) cum: Vec<f64>,
    pub(super) total: f64,
    /// The corners, for the slow-down ([`route_bends`]).
    pub(super) bends: Vec<Bend>,
    /// Spoken holding point ("November five Whiskey"), may be empty.
    pub holding_point: String,
    /// Overrides the arrival line when set — a LINE-UP drive ends with
    /// "Lined up runway two seven", not "Holding point".
    pub done_line: Option<String>,
    /// This airframe's pacing. `build` defaults it to the GA figures; the
    /// arming tool sets the flying aircraft's own (see [`Pace`]).
    pub pace: Pace,
    /// Along-route spans (start m, end m) that run ON a runway: crossings,
    /// backtracks, a line-up's last leg. From the route's own segment flags,
    /// merged where contiguous. The drive lights the landing light over them.
    pub runway_spans: Vec<(f64, f64)>,
    /// A LINE-UP's runway course, true degrees: on a castoring nose the drive
    /// does not end until the nose points down it ([`LineUpAlign`]). `None`
    /// for every other drive.
    pub align_course_deg: Option<f64>,
    /// ...and the runway it lines up on with the takeoff fit's need, m: the
    /// alignment rolls only the runway the fit can spare
    /// ([`LineUpAlign::with_spare`]). `None` keeps [`ALIGN_MAX_ROLL_M`].
    pub align_runway: Option<(crate::runway::Runway, f64)>,
    /// The way a reversal at the route's start is turned (+1 right, -1
    /// left), when the geometry knows it: a backtrack's turn-round turns
    /// TOWARD the centreline ([`turn_round_hint`]). `None` leaves the
    /// first sign of the error to decide ([`committed_err`]).
    pub turn_hint: Option<f64>,
    /// How far short of the route's end the drive stops its nose
    /// ([`Route::nose_fix`]): [`STOP_SHORT_M`], or a hold line's or a
    /// stand's own mark ([`Route::stop_nose_behind_end`]).
    pub(super) stop_short_m: f64,
}

/// How far before a runway span the landing light comes on.
pub(super) const CROSSING_LEAD_M: f64 = 40.0;

/// Is `progress` on (or within [`CROSSING_LEAD_M`] of) a runway span?
pub fn on_runway_span(spans: &[(f64, f64)], progress: f64) -> bool {
    spans
        .iter()
        .any(|&(a, b)| progress >= a - CROSSING_LEAD_M && progress <= b)
}

/// The runway spans of a route from its per-segment flags (`seg[i]` is the
/// leg `path[i] -> path[i+1]`), contiguous runway legs merged into one.
pub fn runway_spans(seg: &[(String, bool)], cum: &[f64]) -> Vec<(f64, f64)> {
    let mut spans: Vec<(f64, f64)> = Vec::new();
    for (i, (_, is_rwy)) in seg.iter().enumerate() {
        if !*is_rwy || i + 1 >= cum.len() {
            continue;
        }
        match spans.last_mut() {
            Some(last) if (last.1 - cum[i]).abs() < 1e-6 => last.1 = cum[i + 1],
            _ => spans.push((cum[i], cum[i + 1])),
        }
    }
    spans
}

/// One corner of a digested polyline, for the slow-down: where its turning
/// starts and ends along the route (m), the heading it turns through
/// (degrees, unsigned) and the tightest radius flown inside it (m).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct Bend {
    pub(super) start: f64,
    pub(super) end: f64,
    pub(super) deg: f64,
    pub(super) radius: f64,
}

/// The heading change a corner is found by, over [`BEND_FIND_SPAN_M`] of
/// route. Smoothed like the guide's cues (densely-noded fillet arcs must not
/// read as hairpins), and on an arc the same 30 degrees in 30 m whether it
/// is drawn in one vertex or twelve: a 57 m radius or tighter.
pub(super) const BEND_FIND_SPAN_M: f64 = 30.0;
/// The span a corner's radius is read over: the route's length over the
/// heading it changes by across this much of it. On a drawn arc that IS its
/// radius; at a lone vertex of D it is 8 / D (in radians), a little under
/// the circle through the route 8 m either side (4 / sin(D/2)), which the
/// cap read before #194.
pub(super) const BEND_RADIUS_SPAN_M: f64 = 8.0;
/// Stations the route's turning is read at, m.
pub(super) const BEND_STEP_M: f64 = 1.0;
/// A vertex turning less than this is no part of a corner's extent.
pub(super) const BEND_VERTEX_MIN_DEG: f64 = 0.5;

/// The corners of a digested polyline, read from its CURVATURE, the heading
/// it changes by per metre, so a corner is the same corner however finely
/// it is drawn. Before #194 a corner was a VERTEX turning 30 degrees over
/// the 15 m either side, and its radius the circle through the route 8 m
/// either side of that vertex: a fillet drawn in 3 to 5 m segments never
/// turns 30 degrees at one vertex, so LEMG's Quebec onto Delta, 104 degrees
/// on about a 16 m radius once the ride took the fillet (e31f338), had no
/// corner at all and the Citation X went in at 17.4 kt (sortie
/// 1790765957, iMac 2026-09-30): full lock for 3 s, 7.3 m off the line.
///
/// A corner is the stretch of stations where the route turns [`TURN_DEG`]
/// or more within [`BEND_FIND_SPAN_M`] centred on them, one way (so it opens
/// up to 15 m before the turning starts, as the vertex reading's
/// neighbouring vertices did); it turns through the sum of its turning
/// vertices, and its radius is the tightest [`BEND_RADIUS_SPAN_M`] inside.
pub(super) fn route_bends(pts: &[(f64, f64)], cum: &[f64]) -> Vec<Bend> {
    // Each vertex's signed turn, over segments of half a metre or more.
    let mut vt: Vec<(f64, f64)> = Vec::new();
    let mut last_brg: Option<f64> = None;
    for i in 0..pts.len().saturating_sub(1) {
        if cum[i + 1] - cum[i] < 0.5 {
            continue;
        }
        let b = taxi_guide::bearing(pts[i], pts[i + 1]);
        if let Some(p) = last_brg {
            let t = crate::angles::signed_diff_deg(b, p);
            if t != 0.0 {
                vt.push((cum[i], t));
            }
        }
        last_brg = Some(b);
    }
    if vt.is_empty() {
        return Vec::new();
    }
    // The heading turned from the start to `s`, and across (a, b].
    let mut prefix = Vec::with_capacity(vt.len() + 1);
    prefix.push(0.0);
    for &(_, t) in &vt {
        prefix.push(prefix.last().unwrap() + t);
    }
    let turned_to = |s: f64| prefix[vt.partition_point(|&(at, _)| at <= s)];
    let turned = |a: f64, b: f64| turned_to(b) - turned_to(a);
    let total = *cum.last().unwrap_or(&0.0);
    let half = BEND_FIND_SPAN_M / 2.0;
    let r_half = BEND_RADIUS_SPAN_M / 2.0;
    let mut bends: Vec<Bend> = Vec::new();
    let mut group: Option<(f64, f64, f64, f64)> = None; // (first, last, side, most)
    let flush = |g: (f64, f64, f64, f64), bends: &mut Vec<Bend>| {
        let (g0, g1, side, most) = g;
        let turning: Vec<f64> = vt
            .iter()
            .filter(|&&(at, t)| at > g0 - half && at <= g1 + half && t * side >= BEND_VERTEX_MIN_DEG)
            .map(|&(at, _)| at)
            .collect();
        let (Some(&first), Some(&last)) = (turning.first(), turning.last()) else {
            return;
        };
        let through = turned(first - 1e-6, last).abs().max(most);
        let (start, end) = (g0.min(first), g1.max(last));
        let mut radius = f64::INFINITY;
        let mut s = start - r_half;
        while s <= end + r_half {
            let d = turned(s - r_half, s + r_half).abs().to_radians();
            if d > 1e-6 {
                radius = radius.min(BEND_RADIUS_SPAN_M / d);
            }
            s += BEND_STEP_M / 2.0;
        }
        bends.push(Bend { start, end, deg: through, radius });
    };
    let mut s = 0.0;
    while s <= total {
        // Either a corner's worth of heading in the find span, or a turn
        // that tight in the radius span (a jog: +27 then -47 degrees in 9 m
        // turns the aircraft hard and nets little, LIRF).
        let wide = turned(s - half, s + half);
        let tight = turned(s - r_half, s + r_half);
        let w = if tight.abs() >= TURN_DEG && tight.abs() > wide.abs() { tight } else { wide };
        let side = w.signum();
        if w.abs() >= TURN_DEG {
            group = match group {
                Some((g0, _, gs, most)) if gs == side => Some((g0, s, side, most.max(w.abs()))),
                Some(g) => {
                    flush(g, &mut bends);
                    Some((s, s, side, w.abs()))
                }
                None => Some((s, s, side, w.abs())),
            };
        } else if let Some(g) = group.take() {
            flush(g, &mut bends);
        }
        s += BEND_STEP_M;
    }
    if let Some(g) = group {
        flush(g, &mut bends);
    }
    bends
}

impl Route {
    /// The route's whole length, metres.
    pub fn total_m(&self) -> f64 {
        self.total
    }

    /// This route joined by a planned turn ([`join_turn`]): `lead` (lat/lon,
    /// from the aircraft round the turn to the join point) then the route
    /// beyond `join_m`. Pace, holding point and the rest are kept; the runway
    /// spans move with the join. `None` if the result is too short to drive.
    pub(crate) fn with_lead(&self, lead: &[(f64, f64)], join_m: f64) -> Option<Route> {
        let mut path: Vec<(f64, f64)> = lead.to_vec();
        path.extend(
            self.path
                .iter()
                .zip(self.cum.iter())
                .filter(|(_, &c)| c > join_m + 0.5)
                .map(|(p, _)| *p),
        );
        if path.len() < 2 {
            return None;
        }
        let geo = taxi_guide::Geo::new(path[0].0);
        let pts: Vec<(f64, f64)> = path.iter().map(|&p| geo.to_m(path[0], p)).collect();
        let mut cum = vec![0.0_f64];
        for w in pts.windows(2) {
            cum.push(cum.last().unwrap() + taxi_guide::dist(w[0], w[1]));
        }
        let total = *cum.last().unwrap();
        if total < STOP_SHORT_M + 15.0 {
            return None;
        }
        let lead_m: f64 = lead
            .windows(2)
            .map(|w| taxi_guide::dist(geo.to_m(path[0], w[0]), geo.to_m(path[0], w[1])))
            .sum();
        let shift = lead_m - join_m;
        let runway_spans = self
            .runway_spans
            .iter()
            .filter(|&&(_, b)| b > join_m)
            .map(|&(a, b)| ((a.max(join_m) + shift).max(0.0), b + shift))
            .collect();
        Some(Route {
            bends: route_bends(&pts, &cum),
            path,
            pts,
            cum,
            total,
            holding_point: self.holding_point.clone(),
            done_line: self.done_line.clone(),
            pace: self.pace,
            runway_spans,
            align_course_deg: self.align_course_deg,
            align_runway: self.align_runway.clone(),
            turn_hint: self.turn_hint,
            stop_short_m: self.stop_short_m,
        })
    }

    /// Digest a published route. `None` when it's too short to drive.
    pub fn build(watch: &TaxiWatch) -> Option<Route> {
        if watch.path.len() < 2 {
            return None;
        }
        let geo = taxi_guide::Geo::new(watch.path[0].0);
        let pts: Vec<(f64, f64)> = watch
            .path
            .iter()
            .map(|&p| geo.to_m(watch.path[0], p))
            .collect();
        let mut cum = vec![0.0_f64];
        for w in pts.windows(2) {
            cum.push(cum.last().unwrap() + taxi_guide::dist(w[0], w[1]));
        }
        let total = *cum.last().unwrap();
        // Shorter than the stop-short margin plus a rolling start there is
        // nothing to drive — the tool answers "we're already there".
        if total < STOP_SHORT_M + 15.0 {
            return None;
        }
        let bends = route_bends(&pts, &cum);
        let runway_spans = runway_spans(&watch.seg, &cum);
        Some(Route {
            path: watch.path.clone(),
            pts,
            cum,
            total,
            bends,
            holding_point: crate::party_line::spoken_taxiway(&watch.holding_point),
            done_line: None,
            pace: Pace::default(),
            runway_spans,
            align_course_deg: None,
            align_runway: None,
            turn_hint: None,
            stop_short_m: STOP_SHORT_M,
        })
    }

    /// Project a lat/lon fix: (along-route progress m, DISTANCE off the route
    /// in m). Never negative — every gate here asks "how far off", which is
    /// the same question on both sides of the line.
    pub fn fix(&self, lat: f64, lon: f64) -> Option<(f64, f64)> {
        taxi_guide::project(&self.path, lat, lon)
    }

    /// [`fix`](Self::fix) keeping the SIDE: positive is right of the route.
    /// What the drive records, so a trail can be read against the paint
    /// afterwards instead of only saying how far off it was.
    pub fn fix_signed(&self, lat: f64, lon: f64) -> Option<(f64, f64)> {
        taxi_guide::project_signed(&self.path, lat, lon)
    }

    /// [`fix_signed`](Self::fix_signed) onto the legs that run the way the
    /// nose points — the projection a JOIN uses, so the nearest paint behind
    /// the tail never becomes the target (see `taxi_guide::project_signed_facing`).
    pub fn fix_signed_facing(&self, lat: f64, lon: f64, heading_true: f64) -> Option<(f64, f64)> {
        taxi_guide::project_signed_facing(&self.path, lat, lon, heading_true)
    }

    /// Heading error (signed degrees, right positive) from the aircraft's
    /// true heading to the pure-pursuit aim point `lookahead` meters further
    /// along the route. Aiming at a point ON the route (not the tangent) is
    /// what folds the cross-track error back to the centreline.
    pub fn heading_err_deg(
        &self,
        lat: f64,
        lon: f64,
        heading_true: f64,
        progress: f64,
        lookahead: f64,
    ) -> f64 {
        let geo = taxi_guide::Geo::new(self.path[0].0);
        let here = geo.to_m(self.path[0], (lat, lon));
        let aim = taxi_guide::point_at(&self.pts, &self.cum, progress + lookahead);
        crate::angles::signed_diff_deg(taxi_guide::bearing(here, aim), heading_true)
    }

    /// The yaw rate the ROUTE ITSELF asks for across the aim span, degrees
    /// per second at this groundspeed: the route's direction at the aim point
    /// minus its direction underfoot, over the time the aircraft takes to
    /// cover the span.
    ///
    /// On a circular arc of radius R this is exactly v/R — the span subtends
    /// L/R radians and is covered in L/v seconds — and on the polyline a
    /// taxiway really is, it is that same measure sampled across whatever
    /// corner lies inside the aim. Straight paint gives zero.
    ///
    /// This is the turn a correctly flown bend MUST carry, which is what
    /// [`damped_err_deg`] credits back so the rate lead stops fighting it.
    pub fn route_turn_rate_dps(&self, progress: f64, lookahead: f64, gs_kts: f64) -> f64 {
        // Sampled over a short step either end rather than read off a leg
        // index: `point_at` already walks the polyline, and a corner landing
        // between two samples is a corner the aim is flying through anyway.
        const STEP_M: f64 = 4.0;
        if lookahead <= STEP_M || gs_kts <= 0.0 {
            return 0.0;
        }
        let dir = |s: f64| {
            taxi_guide::bearing(
                taxi_guide::point_at(&self.pts, &self.cum, s),
                taxi_guide::point_at(&self.pts, &self.cum, s + STEP_M),
            )
        };
        let turn = crate::angles::signed_diff_deg(dir(progress + lookahead), dir(progress));
        turn * (gs_kts * 0.514) / lookahead
    }

    /// The direction the route's LAST segment runs, degrees true.
    ///
    /// Near the stop this replaces the pure-pursuit aim, because the aim point
    /// saturates: `point_at` clamps at the final point, so once the aircraft
    /// rolls past it the bearing to it swings round BEHIND and the law
    /// commands a hard turn toward a waypoint already passed. Measured at
    /// EGHI (2026-07-30), the error ran away with the aircraft dead on the
    /// centreline:
    ///
    ///   rem  0.04 m -> err   9.5°     rem -3.6 m -> err  38.6°
    ///   rem -4.4  m -> err  70.2°     rem -5.0 m -> err 162.6°
    ///
    /// Steering pinned at its limit and the nose came round: 0.07 m off the
    /// line to 3.00 m in four seconds. Every stop did it — hold, line-up,
    /// backtrack, taxi-in — because they all drive this one loop.
    /// The direction of the route's last REAL segment. The planner can end a
    /// route on a repeated point (LIPR Alpha, 2026-09-26: the last segment
    /// was 0.00 m), and the bearing of a zero-length segment is 0, due north:
    /// with 8 m to run the taxi steered a 220-degree aircraft 140 degrees
    /// toward it, full right, and it swung to 255 as it braked. Ten such
    /// jumps at 8 m to go are in the logs. Segments under a metre are no
    /// direction at all.
    pub fn final_bearing_deg(&self) -> Option<f64> {
        const MIN_SEG_M: f64 = 1.0;
        self.pts
            .windows(2)
            .rev()
            .find(|w| taxi_guide::dist(w[0], w[1]) >= MIN_SEG_M)
            .map(|w| taxi_guide::bearing(w[0], w[1]))
    }

    /// Heading error against the route's final bearing — what to fly once the
    /// pursuit aim can no longer be trusted. Falls back to the pursuit error
    /// when the route is too short to have a final segment.
    pub fn stop_heading_err_deg(
        &self,
        lat: f64,
        lon: f64,
        heading_true: f64,
        progress: f64,
        lookahead: f64,
    ) -> f64 {
        match self.final_bearing_deg() {
            Some(b) => crate::angles::signed_diff_deg(b, heading_true),
            None => self.heading_err_deg(lat, lon, heading_true, progress, lookahead),
        }
    }

    /// Meters left before the stop point short of the holding point.
    pub fn remaining(&self, progress: f64) -> f64 {
        self.total - self.stop_short_m - progress
    }

    /// Stop the measured nose `behind_m` short of the route's end: a hold
    /// line's [`NOSE_BEHIND_LINE_M`], a stand's mark none (a stop bar is the
    /// nose wheel's). Stopped by the sim's point (the CG) a 737 parked its
    /// nose ~18 m past the EGPH stand mark and an A330's went 29.6 m past
    /// EGKK's hold Mike (#320). No nose measured, the drive's nose IS the
    /// sim's point and [`STOP_SHORT_M`] stands.
    pub fn stop_nose_behind_end(&mut self, nose_m: f64, behind_m: f64) {
        if nose_m > 0.0 {
            self.stop_short_m = behind_m;
        }
    }

    /// The nose's own fix: `nose_m` ahead of the sim's point `sim` (lat/lon,
    /// along m, signed off m) along the heading, projected ONTO the route the
    /// way the drive projects the sim's point (facing while `joining`). Never
    /// the sim's along plus `nose_m`: in a bend the two differ, and steered
    /// from the one and stopped by the other the 737's steering inverted in
    /// an EGKK corner (0286548, #320). No nose, the sim's own fix. Pure.
    pub fn nose_fix(
        &self,
        sim: ((f64, f64), f64, f64),
        heading_true: f64,
        nose_m: f64,
        joining: bool,
    ) -> ((f64, f64), f64, f64) {
        if nose_m <= 0.0 {
            return sim;
        }
        let ((lat, lon), psi) = (sim.0, heading_true.to_radians());
        let nose =
            taxi_guide::Geo::new(lat).to_ll((lat, lon), (nose_m * psi.sin(), nose_m * psi.cos()));
        let fix = if joining {
            self.fix_signed_facing(nose.0, nose.1, heading_true)
        } else {
            self.fix_signed(nose.0, nose.1)
        };
        fix.map_or(sim, |(s, off)| (nose, s, off))
    }

    /// The bend-imposed speed cap near `progress`, if a corner is close
    /// enough to matter: inside a speed-scaled lead distance before the
    /// corner, and until 15 m past it (mid-turn is no place to accelerate).
    ///
    /// Before that window the cap is the corner's speed plus what
    /// [`BEND_DECEL_MS2`] can take off in the distance left to the window,
    /// so the governor arrives AT the corner's pace instead of being handed
    /// it at the corner: on a 737 the cap arrived about 30 m out and it went
    /// into LEMG's Alpha at 11.5 kt against 10 (2026-09-26). And where the
    /// airframe declares the yaw rate it can turn at (`Pace::max_yaw_dps`),
    /// no corner is taken faster than that rate times its radius: the same
    /// corner was drawn at 9 m and asked 38 deg/s of an aircraft that turns
    /// at 20, and it swung 5.8 m wide.
    ///
    /// A corner is read from the route's curvature ([`route_bends`]): the
    /// window opens before its FIRST turning vertex and holds to 15 m past
    /// its last, so a drawn arc is slowed for where it starts to turn.
    pub fn bend_cap_kts(&self, progress: f64, gs_kts: f64) -> Option<f64> {
        let lead = (gs_kts * 0.514 * 6.0).clamp(25.0, 90.0);
        self.bends
            .iter()
            .filter(|b| progress < b.end + 15.0 && b.start - progress < lead + BEND_ENVELOPE_M)
            .map(|&Bend { start: at, deg, radius, .. }| {
                let class = if deg >= SHARP_DEG {
                    self.pace.sharp
                } else {
                    self.pace.turn
                };
                let corner = self
                    .pace
                    .max_yaw_dps
                    .map_or(class, |w| class.min(w.to_radians() * radius * MS_TO_KTS).max(CORNER_MIN_KTS));
                let before = (at - progress - lead).max(0.0);
                let v = corner / MS_TO_KTS;
                MS_TO_KTS * (v * v + 2.0 * BEND_DECEL_MS2 * before).sqrt()
            })
            .fold(None, |acc: Option<f64>, cap| {
                Some(acc.map_or(cap, |a| a.min(cap)))
            })
    }

    /// The route still ahead of `progress` metres along it, lat/lon.
    pub(super) fn path_ahead(&self, progress: f64) -> Vec<(f64, f64)> {
        let k = self.cum.partition_point(|&c| c <= progress);
        let (Some(b), Some(&end)) = (k.checked_sub(1), self.path.get(k)) else {
            return if k == 0 {
                self.path.clone()
            } else {
                Vec::new()
            };
        };
        let (a, len) = (self.path[b], self.cum[k] - self.cum[b]);
        let f = ((progress - self.cum[b]) / len.max(1e-9)).clamp(0.0, 1.0);
        let here = (a.0 + (end.0 - a.0) * f, a.1 + (end.1 - a.1) * f);
        std::iter::once(here)
            .chain(self.path[k..].iter().copied())
            .collect()
    }

    /// A fix in the route's own frame, metres (x east, y north).
    pub fn local_m(&self, lat: f64, lon: f64) -> (f64, f64) {
        taxi_guide::Geo::new(self.path[0].0).to_m(self.path[0], (lat, lon))
    }

    /// The route in metres for the free nose's one ground law
    /// ([`crate::ground_steer::RefPath`]).
    pub fn ref_path(&self) -> crate::ground_steer::RefPath {
        crate::ground_steer::RefPath::new(&self.pts, &self.cum)
    }
}
