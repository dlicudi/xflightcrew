//! The copilot's progressive taxi guide — the pilot monitoring with the
//! airport diagram on their knee.
//!
//! ATC says the route once; a real crew then navigates it together: "next
//! left is Whiskey", "approaching zero niner left, that's our crossing",
//! "holding point ahead". This module is the crew half of the taxi story:
//! it CONSUMES the route the tower published to the party line
//! ([`crate::party_line::with_taxi_route`])
//! and speaks just-in-time cues in the copilot's own voice. Read-only by
//! construction (route + position in, text out) — it never actuates, and it
//! never invents route: no published clearance, no guidance.
//!
//! Division of labor with the tower's compliance watch: the guide gives the
//! gentle, EARLY cue at decision points; the off-route CHALLENGE stays ATC's.
//! When the aircraft strays past the same 60 m the compliance watch uses,
//! the guide goes quiet rather than compete with the controller.
//!
//! Everything here is pure geometry over the published polyline, so the
//! whole feature battle-tests as plain `cargo test` — no sim, no audio.

use std::time::Instant;

use crate::station::{first_designator, spoken_taxiway, TaxiWatch};

/// Route bends sharper than this (measured over the smoothing window, so
/// gentle fillet arcs don't count) are called as turns.
const TURN_DEG: f64 = 30.0;
/// Bearing smoothing window, meters — apt.dat curves are densely noded, so a
/// per-vertex angle would call every fillet a "turn".
const WINDOW_M: f64 = 15.0;
/// Lateral distance at which the guide stops guiding, meters. Matches the
/// compliance watch's off-route threshold: past it, the CONTROLLER speaks.
const OFF_ROUTE_M: f64 = 60.0;
/// Minimum gap between spoken cues, seconds — a guide, not a narrator.
const CALL_GAP_S: f64 = 5.0;
/// Once this far past a decision point uncalled, drop the cue silently — a
/// stale "next left" after the turn is worse than nothing.
const PASSED_M: f64 = 8.0;

/// What a cue is about — sets how far out it fires.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CueKind {
    /// A bend to call with direction ("next left, onto Whiskey").
    Turn,
    /// Same direction, new taxiway name ("straight ahead onto Romeo").
    NameChange,
    /// The route enters a runway edge — the safety call.
    RunwayAhead,
    /// End of the route.
    HoldingPoint,
}

impl CueKind {
    /// How far before the decision point the cue speaks, meters. Safety
    /// calls fire earlier than steering ones.
    fn lookahead_m(self) -> f64 {
        match self {
            CueKind::Turn | CueKind::NameChange => 80.0,
            CueKind::RunwayAhead | CueKind::HoldingPoint => 120.0,
        }
    }
}

#[derive(Debug, Clone)]
struct Cue {
    /// Along-route distance of the decision point, meters from route start.
    at_m: f64,
    kind: CueKind,
    line: String,
}

/// The per-clearance guide: cues precomputed from the published route, each
/// spoken at most once as the aircraft's along-route progress approaches it.
pub struct Guide {
    /// The route this guide was built for — the monitor rebuilds on change.
    path: Vec<(f64, f64)>,
    cues: Vec<Cue>,
    called: Vec<bool>,
    last_call: Option<Instant>,
}

impl Guide {
    /// Build the cue list for a published route. `None` when the route is
    /// too short to guide on.
    pub fn build(watch: &TaxiWatch) -> Option<Guide> {
        if watch.path.len() < 2 {
            return None;
        }
        let geo = Geo::new(watch.path[0].0);
        let pts: Vec<(f64, f64)> = watch.path.iter().map(|&p| geo.to_m(watch.path[0], p)).collect();
        let mut cum = vec![0.0_f64];
        for w in pts.windows(2) {
            cum.push(cum.last().unwrap() + dist(w[0], w[1]));
        }
        let total = *cum.last().unwrap();
        if total < 30.0 {
            return None;
        }
        let seg_name = |i: usize| -> Option<&(String, bool)> { watch.seg.get(i) };
        // The last non-empty taxiway name at or before leg `i` — unnamed
        // links "bridge" the name they connect from.
        let named_before = |i: usize| -> Option<&str> {
            (0..=i)
                .rev()
                .filter_map(seg_name)
                .find(|(n, _)| !n.is_empty())
                .map(|(n, _)| n.as_str())
        };
        let mut cues: Vec<Cue> = Vec::new();
        for i in 1..pts.len() - 1 {
            let Some((leg_name, leg_runway)) = seg_name(i).cloned() else { continue };
            // The safety call: this vertex enters a runway edge.
            if leg_runway && !seg_name(i - 1).is_some_and(|(_, r)| *r) {
                let rwy = crate::atis::spoken_runway(first_designator(&leg_name));
                cues.push(Cue {
                    at_m: cum[i],
                    kind: CueKind::RunwayAhead,
                    line: if rwy.is_empty() {
                        "Runway crossing ahead on our route. Check it's clear.".into()
                    } else {
                        format!("Runway {rwy} ahead on our route. Check it's clear, then cross.")
                    },
                });
                continue;
            }
            if leg_runway {
                continue; // taxiing along the crossing itself — nothing to say
            }
            let bearing_in = bearing(point_at(&pts, &cum, cum[i] - WINDOW_M), pts[i]);
            let bearing_out = bearing(pts[i], point_at(&pts, &cum, cum[i] + WINDOW_M));
            let delta = angle_diff(bearing_out, bearing_in);
            let prev_name = named_before(i - 1);
            let new_name = !leg_name.is_empty() && prev_name != Some(leg_name.as_str());
            let spoken = spoken_taxiway(&leg_name);
            if delta.abs() >= TURN_DEG {
                let dir = if delta < 0.0 { "left" } else { "right" };
                cues.push(Cue {
                    at_m: cum[i],
                    kind: CueKind::Turn,
                    line: if new_name && !spoken.is_empty() {
                        format!("Next {dir}, onto {spoken}.")
                    } else {
                        format!("Follow it around to the {dir}.")
                    },
                });
            } else if new_name && !spoken.is_empty() {
                // Crossing exits fold into this naturally: the leg after a
                // runway edge has a fresh name, so the cue reads "straight
                // ahead onto November five Whiskey" as the crossing clears.
                cues.push(Cue {
                    at_m: cum[i],
                    kind: CueKind::NameChange,
                    line: format!("Straight ahead onto {spoken}."),
                });
            }
        }
        let hp = spoken_taxiway(&watch.holding_point);
        cues.push(Cue {
            at_m: total,
            kind: CueKind::HoldingPoint,
            line: if hp.is_empty() {
                "Holding point ahead. That's us.".into()
            } else {
                format!("Holding point {hp} ahead. That's us.")
            },
        });
        let called = vec![false; cues.len()];
        Some(Guide { path: watch.path.clone(), cues, called, last_call: None })
    }

    /// True if this guide was built for `path` — the monitor rebuilds when a
    /// fresh clearance publishes a different route.
    pub fn is_for(&self, path: &[(f64, f64)]) -> bool {
        self.path == path
    }

    /// One monitor tick: the cue to speak now, if any. Quiet unless taxiing
    /// (moving, not takeoff-roll fast), quiet off the route (the compliance
    /// watch owns that conversation), one cue at a time on a cooldown, each
    /// spoken once. A cue overrun before it could speak drops silently.
    pub fn tick(&mut self, lat: f64, lon: f64, gs_kts: f64, now: Instant) -> Option<String> {
        if !(2.0..40.0).contains(&gs_kts) {
            return None;
        }
        let (progress, lateral) = project(&self.path, lat, lon)?;
        if lateral > OFF_ROUTE_M {
            return None;
        }
        if self
            .last_call
            .is_some_and(|t| now.duration_since(t).as_secs_f64() < CALL_GAP_S)
        {
            return None;
        }
        let mut speak: Option<usize> = None;
        for (i, cue) in self.cues.iter().enumerate() {
            if self.called[i] {
                continue;
            }
            let remaining = cue.at_m - progress;
            if remaining < -PASSED_M {
                self.called[i] = true; // sailed past unspoken — stale, drop it
            } else if remaining <= cue.kind.lookahead_m()
                && speak.is_none_or(|s| cue.at_m < self.cues[s].at_m)
            {
                speak = Some(i);
            }
        }
        let i = speak?;
        self.called[i] = true;
        self.last_call = Some(now);
        Some(self.cues[i].line.clone())
    }
}

/// The on-demand answer to "which way?" — the FO reading from the diagram:
/// the next decision point with a distance, the taxiways still to come, and
/// the holding point. Works without a [`Guide`] instance so the command loop
/// can answer straight off the published route.
pub fn describe(watch: &TaxiWatch, lat: f64, lon: f64) -> String {
    let Some((progress, lateral)) = project(&watch.path, lat, lon) else {
        return "I can't make out the taxi route from here.".into();
    };
    if lateral > OFF_ROUTE_M {
        let via = &watch.via;
        return if via.is_empty() {
            format!("We're about {} meters off the cleared route. Let's rejoin it.", round_m(lateral))
        } else {
            format!(
                "We're about {} meters off the cleared route. Rejoin and taxi via {via}.",
                round_m(lateral)
            )
        };
    }
    // The next steering cue ahead of us, phrased with its distance.
    let next = Guide::build(watch).and_then(|g| {
        g.cues
            .into_iter()
            .find(|c| c.at_m - progress > PASSED_M)
            .map(|c| (c.at_m - progress, c.line))
    });
    // The taxiway names still to come, from the leg we're on outward.
    let mut coming: Vec<String> = Vec::new();
    let geo = Geo::new(watch.path[0].0);
    let pts: Vec<(f64, f64)> = watch.path.iter().map(|&p| geo.to_m(watch.path[0], p)).collect();
    let mut cum = 0.0;
    for (i, w) in pts.windows(2).enumerate() {
        let end = cum + dist(w[0], w[1]);
        if end > progress {
            if let Some((name, runway)) = watch.seg.get(i) {
                if !runway && !name.is_empty() {
                    let s = spoken_taxiway(name);
                    if coming.last() != Some(&s) {
                        coming.push(s);
                    }
                }
            }
        }
        cum = end;
    }
    let hp = spoken_taxiway(&watch.holding_point);
    let mut out = String::new();
    match next {
        Some((d, line)) if d > 25.0 => {
            out.push_str(&format!("In about {} meters: {line}", round_m(d)));
        }
        Some((_, line)) => {
            out.push_str(&format!("Just ahead: {line}"));
        }
        None => out.push_str("Straight ahead."),
    }
    if coming.len() > 1 {
        out.push_str(&format!(" After that it's {}.", coming[1..].join(", ")));
    }
    if !hp.is_empty() {
        out.push_str(&format!(" We're cleared to holding point {hp}."));
    }
    out
}

/// Round a distance for speech — nobody says "in 147 meters" out loud.
fn round_m(d: f64) -> i64 {
    ((d / 50.0).round() as i64 * 50).max(50)
}

// --- Flat-earth geometry over the route's own neighborhood -----------------

/// Meters-per-degree scales at the route's latitude — the same flat-earth
/// approximation the planner and the compliance watch use.
struct Geo {
    m_per_deg_lat: f64,
    m_per_deg_lon: f64,
}

impl Geo {
    fn new(lat: f64) -> Geo {
        Geo { m_per_deg_lat: 111_320.0, m_per_deg_lon: 111_320.0 * lat.to_radians().cos() }
    }
    /// (lat, lon) to meters (x east, y north) relative to `origin`.
    fn to_m(&self, origin: (f64, f64), p: (f64, f64)) -> (f64, f64) {
        ((p.1 - origin.1) * self.m_per_deg_lon, (p.0 - origin.0) * self.m_per_deg_lat)
    }
}

fn dist(a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    (dx * dx + dy * dy).sqrt()
}

/// Compass bearing a→b, degrees 0..360 (x east, y north).
fn bearing(a: (f64, f64), b: (f64, f64)) -> f64 {
    (b.0 - a.0).atan2(b.1 - a.1).to_degrees().rem_euclid(360.0)
}

/// Smallest signed angle a−b, degrees, in −180..180. Positive = `a` is
/// clockwise of `b` (a right turn when in/out bearings are compared).
fn angle_diff(a: f64, b: f64) -> f64 {
    (a - b + 540.0).rem_euclid(360.0) - 180.0
}

/// The point `s` meters along the polyline (clamped to its ends).
fn point_at(pts: &[(f64, f64)], cum: &[f64], s: f64) -> (f64, f64) {
    let s = s.clamp(0.0, *cum.last().unwrap());
    for i in 1..pts.len() {
        if s <= cum[i] {
            let seg = cum[i] - cum[i - 1];
            let t = if seg < 1e-6 { 0.0 } else { (s - cum[i - 1]) / seg };
            let (a, b) = (pts[i - 1], pts[i]);
            return (a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1));
        }
    }
    *pts.last().unwrap()
}

/// Project a position onto the route: (along-route progress m, lateral m).
/// `None` on a degenerate path.
fn project(path: &[(f64, f64)], lat: f64, lon: f64) -> Option<(f64, f64)> {
    if path.len() < 2 {
        return None;
    }
    let geo = Geo::new(path[0].0);
    let origin = path[0];
    let p = geo.to_m(origin, (lat, lon));
    let pts: Vec<(f64, f64)> = path.iter().map(|&q| geo.to_m(origin, q)).collect();
    let mut cum = 0.0;
    let mut best: Option<(f64, f64)> = None;
    for w in pts.windows(2) {
        let (a, b) = (w[0], w[1]);
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let len2 = dx * dx + dy * dy;
        let t = if len2 < 1.0 {
            0.0
        } else {
            (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len2).clamp(0.0, 1.0)
        };
        let c = (a.0 + t * dx, a.1 + t * dy);
        let d = dist(p, c);
        let along = cum + t * len2.sqrt();
        if best.is_none_or(|(_, bd)| d < bd) {
            best = Some((along, d));
        }
        cum += len2.sqrt();
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    /// A test field near 36°N: build lat/lon from meters so the geometry
    /// reads like a taxi diagram.
    const LAT0: f64 = 36.0;
    const LON0: f64 = -5.0;
    fn ll(x_m: f64, y_m: f64) -> (f64, f64) {
        let g = Geo::new(LAT0);
        (LAT0 + y_m / g.m_per_deg_lat, LON0 + x_m / g.m_per_deg_lon)
    }

    /// East 200 m on W, left turn north onto S5W, cross runway 09L/27R,
    /// continue north on N5W to the holding point N5W.
    fn route() -> TaxiWatch {
        TaxiWatch {
            path: vec![ll(0.0, 0.0), ll(200.0, 0.0), ll(200.0, 60.0), ll(200.0, 120.0), ll(200.0, 300.0)],
            via: "Whiskey, Sierra five Whiskey, November five Whiskey".into(),
            seg: vec![
                ("W".into(), false),
                ("S5W".into(), false),
                ("09L/27R".into(), true),
                ("N5W".into(), false),
            ],
            holding_point: "N5W".into(),
        }
    }

    #[test]
    fn cues_read_like_a_pilot_monitoring() {
        let g = Guide::build(&route()).expect("guide");
        let lines: Vec<&str> = g.cues.iter().map(|c| c.line.as_str()).collect();
        assert_eq!(
            lines,
            vec![
                "Next left, onto Sierra five Whiskey.",
                "Runway zero niner left ahead on our route. Check it's clear, then cross.",
                "Straight ahead onto November five Whiskey.",
                "Holding point November five Whiskey ahead. That's us.",
            ],
            "cue script: {lines:?}"
        );
    }

    #[test]
    fn cues_fire_in_order_at_taxi_pace() {
        let mut g = Guide::build(&route()).expect("guide");
        let t0 = Instant::now();
        let s = Duration::from_secs;
        // Parked: silent. Rolling early on W, far from the turn: silent.
        assert_eq!(g.tick(ll(0.0, 0.0).0, ll(0.0, 0.0).1, 0.0, t0), None);
        let (lat, lon) = ll(60.0, 0.0);
        assert_eq!(g.tick(lat, lon, 10.0, t0), None);
        // 70 m before the turn: the first cue.
        let (lat, lon) = ll(130.0, 0.0);
        let cue = g.tick(lat, lon, 10.0, t0 + s(10)).expect("turn cue");
        assert!(cue.contains("left"), "{cue}");
        assert!(cue.contains("Sierra five Whiskey"), "{cue}");
        // Same spot again: spoken once.
        assert_eq!(g.tick(lat, lon, 10.0, t0 + s(20)), None);
        // Past the turn, heading north: the runway call (already in range,
        // waiting on the cooldown gap).
        let (lat, lon) = ll(200.0, 10.0);
        let cue = g.tick(lat, lon, 10.0, t0 + s(30)).expect("runway cue");
        assert!(cue.contains("Runway zero niner left"), "{cue}");
        // Off the crossing: the fresh name.
        let (lat, lon) = ll(200.0, 115.0);
        let cue = g.tick(lat, lon, 10.0, t0 + s(40)).expect("name cue");
        assert!(cue.contains("November five Whiskey"), "{cue}");
        // Short final to the holding point.
        let (lat, lon) = ll(200.0, 240.0);
        let cue = g.tick(lat, lon, 10.0, t0 + s(50)).expect("holding point");
        assert!(cue.contains("Holding point"), "{cue}");
        // Done: quiet forever after.
        assert_eq!(g.tick(lat, lon, 10.0, t0 + s(60)), None);
    }

    #[test]
    fn guide_defers_to_the_controller_off_route_and_paces_itself() {
        let mut g = Guide::build(&route()).expect("guide");
        let t0 = Instant::now();
        // 100 m south of the route with a cue in range: the compliance
        // watch owns off-route, the guide stays quiet.
        let (lat, lon) = ll(130.0, -100.0);
        assert_eq!(g.tick(lat, lon, 10.0, t0), None);
        // Back on it: speaks. Two seconds later at the next decision point:
        // still inside the call gap, holds its tongue.
        let (lat, lon) = ll(130.0, 0.0);
        assert!(g.tick(lat, lon, 10.0, t0).is_some());
        let (lat2, lon2) = ll(200.0, 10.0);
        assert_eq!(g.tick(lat2, lon2, 10.0, t0 + Duration::from_secs(2)), None);
        // Runway-roll speeds are not taxiing — no cue at 60 kts.
        assert_eq!(g.tick(lat2, lon2, 60.0, t0 + Duration::from_secs(20)), None);
    }

    #[test]
    fn a_cue_overrun_unspoken_drops_instead_of_arriving_late() {
        let mut g = Guide::build(&route()).expect("guide");
        let t0 = Instant::now();
        // First tick happens only after the turn is 20 m behind us: the
        // stale "next left" is dropped, and the runway call (in range)
        // speaks instead.
        let (lat, lon) = ll(200.0, 20.0);
        let cue = g.tick(lat, lon, 10.0, t0).expect("runway cue");
        assert!(cue.contains("Runway"), "{cue}");
        assert!(g.called[0], "the missed turn cue must be retired");
    }

    #[test]
    fn describe_reads_the_route_off_the_diagram() {
        let w = route();
        // On W, 100 m before the turn.
        let (lat, lon) = ll(100.0, 0.0);
        let d = describe(&w, lat, lon);
        assert!(d.contains("In about 100 meters"), "{d}");
        assert!(d.contains("left"), "{d}");
        assert!(d.contains("Sierra five Whiskey"), "{d}");
        assert!(d.contains("holding point November five Whiskey"), "{d}");
        // Way off the route: own up and point back at the via line.
        let (lat, lon) = ll(100.0, -200.0);
        let d = describe(&w, lat, lon);
        assert!(d.contains("off the cleared route"), "{d}");
        assert!(d.contains("Whiskey"), "{d}");
    }

    #[test]
    fn gentle_fillets_are_not_turns() {
        // A dense 10°-per-node arc (a fillet), then straight: no Turn cue.
        let mut path = vec![ll(0.0, 0.0), ll(100.0, 0.0)];
        // Shallow drift northward over 200 m — total bend under 30°.
        path.push(ll(180.0, 20.0));
        path.push(ll(260.0, 45.0));
        path.push(ll(360.0, 45.0));
        let w = TaxiWatch {
            path,
            via: "Alpha".into(),
            seg: vec![("A".into(), false); 4],
            holding_point: "A1".into(),
        };
        let g = Guide::build(&w).expect("guide");
        assert!(
            g.cues.iter().all(|c| c.kind != CueKind::Turn),
            "fillet called as a turn: {:?}",
            g.cues
        );
    }
}
