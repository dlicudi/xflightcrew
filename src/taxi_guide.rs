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
/// Bends at least this sharp are called "sharp" — the pilot should slow for
/// them, and "next left" undersells a near-reversal.
const SHARP_DEG: f64 = 100.0;
/// Bends below this (but at/above [`TURN_DEG`]) are a "bear left", not a turn.
const BEAR_DEG: f64 = 45.0;
/// A named turn at the end of a straight run at least this long earns an
/// early PREP call ("coming up: ...") as well as the execute cue — on a long
/// leg the pilot has tuned out by the time the junction arrives.
const PREP_MIN_RUN_M: f64 = 300.0;
/// How far before the turn the prep call is anchored.
const PREP_LEAD_M: f64 = 180.0;
/// Lateral drift at which a just-passed turn reads as MISSED (below it is
/// normal turning slop); the missed-turn nudge lives between here and
/// [`OFF_ROUTE_M`], where the controller takes over.
const NUDGE_MIN_LAT_M: f64 = 20.0;
/// How far past a turn's decision point the missed-turn nudge still applies.
const NUDGE_WINDOW_M: f64 = 90.0;
/// Stopped at least this long → re-orient the pilot when the wheels roll
/// again ("we're on Bravo, next is the left onto...").
const STOP_REORIENT_S: f64 = 20.0;
/// Above this groundspeed a sharp turn cue gets an "easy now" in front.
const EASY_KTS: f64 = 15.0;

/// What a cue is about — sets how far out it fires.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CueKind {
    /// A bend to call with direction ("next left, onto Whiskey").
    Turn,
    /// The early heads-up before a turn at the end of a long straight
    /// ("coming up: left onto Whiskey") — the execute cue still follows.
    Prep,
    /// Same direction, new taxiway name ("straight ahead onto Romeo").
    NameChange,
    /// The route enters a runway edge — the safety call.
    RunwayAhead,
    /// End of the route.
    HoldingPoint,
}

impl CueKind {
    /// How far before the decision point the cue speaks, meters. TIME-based
    /// with a distance floor: a fixed 80 m felt "almost exactly at the turn"
    /// at real taxi speeds (pilot feedback, 2026-07-12) — 80 m at 18 kts is
    /// under 9 s, and TTS synthesis plus the speech arbiter eat the front of
    /// that. So the cue leads by seconds of taxi, not meters of pavement,
    /// and the floor only carries creep-speed taxiing. Safety calls lead
    /// longer than steering ones.
    fn lookahead_m(self, gs_kts: f64) -> f64 {
        let gs_mps = gs_kts * 0.514;
        match self {
            CueKind::Turn | CueKind::Prep | CueKind::NameChange => {
                (gs_mps * 15.0).clamp(80.0, 220.0)
            }
            CueKind::RunwayAhead | CueKind::HoldingPoint => (gs_mps * 20.0).clamp(120.0, 280.0),
        }
    }
}

#[derive(Debug, Clone)]
struct Cue {
    /// Along-route distance of the decision point, meters from route start.
    at_m: f64,
    kind: CueKind,
    line: String,
    /// Terse ASCII form for the panel strip ("left onto G11").
    short: String,
    /// Turn direction, for the missed-turn nudge.
    dir: Option<&'static str>,
    /// A sharp bend — slow-down territory.
    sharp: bool,
}

/// The per-clearance guide: cues precomputed from the published route, each
/// spoken at most once as the aircraft's along-route progress approaches it.
pub struct Guide {
    /// The route this guide was built for — the monitor rebuilds on change.
    path: Vec<(f64, f64)>,
    cues: Vec<Cue>,
    called: Vec<bool>,
    /// Missed-turn nudge spoken, per cue — once each, like the cues.
    nudged: Vec<bool>,
    /// Along-route name spans for re-orientation: the spoken taxiway name in
    /// effect up to each cumulative distance.
    names: Vec<(f64, String)>,
    last_call: Option<Instant>,
    /// Last (progress, lateral) fix — the missed-turn nudge needs the drift
    /// TREND, not one noisy read.
    prev_fix: Option<(f64, f64)>,
    /// When the aircraft stopped rolling (gs below taxi), if it has.
    stopped_at: Option<Instant>,
    /// A long stop just ended — re-orient on the next taxiing tick.
    reorient_armed: bool,
}

impl Guide {
    /// Build the cue list for a published route. `None` when the route is
    /// too short to guide on.
    pub fn build(watch: &TaxiWatch) -> Option<Guide> {
        if watch.path.len() < 2 {
            return None;
        }
        let geo = Geo::new(watch.path[0].0);
        let pts: Vec<(f64, f64)> = watch
            .path
            .iter()
            .map(|&p| geo.to_m(watch.path[0], p))
            .collect();
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
            let Some((leg_name, leg_runway)) = seg_name(i).cloned() else {
                continue;
            };
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
                    short: format!("cross rwy {}", first_designator(&leg_name)),
                    dir: None,
                    sharp: false,
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
                let sharp = delta.abs() >= SHARP_DEG;
                let named = new_name && !spoken.is_empty();
                // Sharpness in the words: "next left" undersells a
                // near-reversal, and a 30-45 degree drift isn't a turn at all
                // to the eye — it's a bear.
                let line = match (named, sharp, delta.abs() < BEAR_DEG) {
                    (true, true, _) => format!("Sharp {dir} next, onto {spoken}."),
                    (true, _, true) => format!("Bear {dir}, onto {spoken}."),
                    (true, _, _) => format!("Next {dir}, onto {spoken}."),
                    (false, true, _) => format!("Follow it sharply around to the {dir}."),
                    (false, ..) => format!("Follow it around to the {dir}."),
                };
                cues.push(Cue {
                    at_m: cum[i],
                    kind: CueKind::Turn,
                    line,
                    short: if leg_name.is_empty() {
                        format!("{dir} turn")
                    } else {
                        format!("{dir} onto {leg_name}")
                    },
                    dir: Some(if delta < 0.0 { "left" } else { "right" }),
                    sharp,
                });
            } else if new_name && !spoken.is_empty() {
                // Crossing exits fold into this naturally: the leg after a
                // runway edge has a fresh name, so the cue reads "straight
                // ahead onto November five Whiskey" as the crossing clears.
                cues.push(Cue {
                    at_m: cum[i],
                    kind: CueKind::NameChange,
                    line: format!("Straight ahead onto {spoken}."),
                    short: format!("onto {leg_name}"),
                    dir: None,
                    sharp: false,
                });
            }
        }
        // An arrival route ends at a parking stand — the guide calls it a stand,
        // not a holding point (the tower cleared us "to stand ..."). Departure
        // (and every synthetic-route test) reads the flag as false.
        let (line, short) = if crate::party_line::taxi_to_stand() {
            let s = crate::station::spoken_stand(&watch.holding_point);
            let mut c = s.chars();
            let spoken = c
                .next()
                .map_or(String::new(), |f| f.to_uppercase().chain(c).collect());
            (
                if spoken.is_empty() {
                    "Parking ahead. That's us.".into()
                } else {
                    format!("{spoken} ahead. That's us.")
                },
                if watch.holding_point.is_empty() {
                    "parking".into()
                } else {
                    format!("stand {}", watch.holding_point)
                },
            )
        } else {
            let hp = spoken_taxiway(&watch.holding_point);
            (
                if hp.is_empty() {
                    "Holding point ahead. That's us.".into()
                } else {
                    format!("Holding point {hp} ahead. That's us.")
                },
                if watch.holding_point.is_empty() {
                    "holding point".into()
                } else {
                    format!("hold {}", watch.holding_point)
                },
            )
        };
        cues.push(Cue {
            at_m: total,
            kind: CueKind::HoldingPoint,
            line,
            short,
            dir: None,
            sharp: false,
        });
        // Prep calls: a NAMED turn at the end of a long straight gets an
        // early heads-up as well — on a 400 m leg the pilot has tuned out by
        // the junction (and a single perishable cue can die in a busy-radio
        // window; the prep gives the turn two chances to be heard).
        let mut preps: Vec<Cue> = Vec::new();
        let mut prev_at = 0.0_f64;
        for cue in &cues {
            if cue.kind == CueKind::Turn && cue.at_m - prev_at >= PREP_MIN_RUN_M {
                if let Some(dir) = cue.dir {
                    let sharp = if cue.sharp { "sharp " } else { "" };
                    let onto = cue
                        .short
                        .strip_prefix(&format!("{dir} onto "))
                        .map(spoken_taxiway)
                        .filter(|s| !s.is_empty());
                    preps.push(Cue {
                        at_m: cue.at_m - PREP_LEAD_M,
                        kind: CueKind::Prep,
                        line: match onto {
                            Some(name) => format!("Coming up: {sharp}{dir} onto {name}."),
                            None => format!("Coming up: a {sharp}{dir} turn."),
                        },
                        short: cue.short.clone(),
                        dir: cue.dir,
                        sharp: cue.sharp,
                    });
                }
            }
            prev_at = cue.at_m;
        }
        cues.extend(preps);
        cues.sort_by(|a, b| a.at_m.total_cmp(&b.at_m));
        // Name spans for the post-stop re-orientation: the spoken name in
        // effect up to each leg's end, unnamed links bridging like the cues.
        let mut names: Vec<(f64, String)> = Vec::new();
        for i in 0..pts.len() - 1 {
            let name = named_before(i).map(spoken_taxiway).unwrap_or_default();
            match names.last_mut() {
                Some((end, n)) if *n == name => *end = cum[i + 1],
                _ => names.push((cum[i + 1], name)),
            }
        }
        let called = vec![false; cues.len()];
        let nudged = vec![false; cues.len()];
        Some(Guide {
            path: watch.path.clone(),
            cues,
            called,
            nudged,
            names,
            last_call: None,
            prev_fix: None,
            stopped_at: None,
            reorient_armed: false,
        })
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
        // Stop bookkeeping runs below the speed gate: a 20 s+ stop (holding
        // for traffic, a checklist) earns a re-orientation when rolling
        // resumes — the pilot has lost the thread by then.
        if gs_kts < 2.0 {
            self.stopped_at.get_or_insert(now);
        } else if let Some(t) = self.stopped_at.take() {
            if now.duration_since(t).as_secs_f64() >= STOP_REORIENT_S {
                self.reorient_armed = true;
            }
        }
        if !(2.0..40.0).contains(&gs_kts) {
            return None;
        }
        let (progress, lateral) = project(&self.path, lat, lon)?;
        let prev = self.prev_fix.replace((progress, lateral));
        if lateral > OFF_ROUTE_M {
            return None;
        }
        if self
            .last_call
            .is_some_and(|t| now.duration_since(t).as_secs_f64() < CALL_GAP_S)
        {
            return None;
        }
        // Missed turn: drifting AWAY (lateral above turning slop and rising)
        // right after a turn's decision point. This is the guide's half of
        // the 0-60 m dead zone — past 60 m the controller challenges, and
        // between the two somebody must say it plainly and immediately (the
        // G11 miss ran 3.5 minutes before anyone did, 2026-07-13).
        if lateral >= NUDGE_MIN_LAT_M && prev.is_some_and(|(_, pl)| lateral > pl + 0.5) {
            for i in 0..self.cues.len() {
                let cue = &self.cues[i];
                if cue.kind != CueKind::Turn || self.nudged[i] {
                    continue;
                }
                let Some(dir) = cue.dir else { continue };
                if (cue.at_m - 15.0..cue.at_m + NUDGE_WINDOW_M).contains(&progress) {
                    self.nudged[i] = true;
                    self.called[i] = true; // the moment has passed either way
                    self.last_call = Some(now);
                    return Some(format!(
                        "We've missed our {dir} back there. Let's get back on the route."
                    ));
                }
            }
        }
        // Rolling again after a long stop: say where we are and what's next,
        // without consuming the next cue — it still gets its own call at the
        // decision point.
        if self.reorient_armed {
            self.reorient_armed = false;
            self.last_call = Some(now);
            let name = self
                .names
                .iter()
                .find(|(end, _)| *end >= progress - 1.0)
                .map(|(_, n)| n.as_str())
                .filter(|n| !n.is_empty());
            let next = self
                .cues
                .iter()
                .zip(&self.called)
                .find(|(c, called)| !**called && c.at_m - progress > PASSED_M)
                .map(|(c, _)| c.line.clone());
            return Some(match (name, next) {
                (Some(n), Some(line)) => format!("Rolling again. We're on {n}. {line}"),
                (Some(n), None) => format!("Rolling again. We're on {n}."),
                (None, Some(line)) => format!("Rolling again. {line}"),
                (None, None) => "Rolling again.".to_string(),
            });
        }
        let mut speak: Option<usize> = None;
        for (i, cue) in self.cues.iter().enumerate() {
            if self.called[i] {
                continue;
            }
            let remaining = cue.at_m - progress;
            if remaining < -PASSED_M {
                self.called[i] = true; // sailed past unspoken — stale, drop it
            } else if remaining <= cue.kind.lookahead_m(gs_kts)
                && speak.is_none_or(|s| cue.at_m < self.cues[s].at_m)
            {
                speak = Some(i);
            }
        }
        let i = speak?;
        self.called[i] = true;
        self.last_call = Some(now);
        let cue = &self.cues[i];
        // Fast into a sharp bend: the words carry the speed advice too.
        if cue.kind == CueKind::Turn && cue.sharp && gs_kts > EASY_KTS {
            return Some(format!("Easy now. {}", cue.line));
        }
        Some(cue.line.clone())
    }

    /// The glanceable next-decision-point line for the plugin panel strip —
    /// by GEOMETRY, not spoken state (announced or not, the panel shows what
    /// pavement comes next). ASCII-only; the panel font is picky.
    pub fn strip_hint(&self, lat: f64, lon: f64) -> Option<String> {
        let (progress, lateral) = project(&self.path, lat, lon)?;
        if lateral > OFF_ROUTE_M {
            return Some("TAXI: off route".into());
        }
        let cue = self
            .cues
            .iter()
            .find(|c| c.kind != CueKind::Prep && c.at_m - progress > -PASSED_M)?;
        let d = (cue.at_m - progress).max(0.0);
        Some(format!("NEXT {d:.0}m: {}", cue.short))
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
            format!(
                "We're about {} meters off the cleared route. Let's rejoin it.",
                round_m(lateral)
            )
        } else {
            format!(
                "We're about {} meters off the cleared route. Rejoin and taxi via {via}.",
                round_m(lateral)
            )
        };
    }
    // The next steering cue ahead of us, phrased with its distance. Prep
    // heads-ups are the spoken guide's pacing device, not a decision point —
    // "which way?" answers with the turn itself.
    let next = Guide::build(watch).and_then(|g| {
        g.cues
            .into_iter()
            .filter(|c| c.kind != CueKind::Prep)
            .find(|c| c.at_m - progress > PASSED_M)
            .map(|c| (c.at_m - progress, c.line))
    });
    // The taxiway names still to come, from the leg we're on outward.
    let mut coming: Vec<String> = Vec::new();
    let geo = Geo::new(watch.path[0].0);
    let pts: Vec<(f64, f64)> = watch
        .path
        .iter()
        .map(|&p| geo.to_m(watch.path[0], p))
        .collect();
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
/// approximation the planner and the compliance watch use. `pub(crate)` with
/// the polyline helpers below: the auto-taxi steering law works in the same
/// local frame over the same published route.
pub(crate) struct Geo {
    m_per_deg_lat: f64,
    m_per_deg_lon: f64,
}

impl Geo {
    pub(crate) fn new(lat: f64) -> Geo {
        Geo {
            m_per_deg_lat: 111_320.0,
            m_per_deg_lon: 111_320.0 * lat.to_radians().cos(),
        }
    }
    /// (lat, lon) to meters (x east, y north) relative to `origin`.
    pub(crate) fn to_m(&self, origin: (f64, f64), p: (f64, f64)) -> (f64, f64) {
        (
            (p.1 - origin.1) * self.m_per_deg_lon,
            (p.0 - origin.0) * self.m_per_deg_lat,
        )
    }
}

pub(crate) fn dist(a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    (dx * dx + dy * dy).sqrt()
}

/// Compass bearing a→b, degrees 0..360 (x east, y north).
pub(crate) fn bearing(a: (f64, f64), b: (f64, f64)) -> f64 {
    (b.0 - a.0).atan2(b.1 - a.1).to_degrees().rem_euclid(360.0)
}

/// Smallest signed angle a−b, degrees, in −180..180. Positive = `a` is
/// clockwise of `b` (a right turn when in/out bearings are compared).
pub(crate) fn angle_diff(a: f64, b: f64) -> f64 {
    (a - b + 540.0).rem_euclid(360.0) - 180.0
}

/// The point `s` meters along the polyline (clamped to its ends).
pub(crate) fn point_at(pts: &[(f64, f64)], cum: &[f64], s: f64) -> (f64, f64) {
    let s = s.clamp(0.0, *cum.last().unwrap());
    for i in 1..pts.len() {
        if s <= cum[i] {
            let seg = cum[i] - cum[i - 1];
            let t = if seg < 1e-6 {
                0.0
            } else {
                (s - cum[i - 1]) / seg
            };
            let (a, b) = (pts[i - 1], pts[i]);
            return (a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1));
        }
    }
    *pts.last().unwrap()
}

/// Project a position onto the route: (along-route progress m, lateral m).
/// `None` on a degenerate path.
pub(crate) fn project(path: &[(f64, f64)], lat: f64, lon: f64) -> Option<(f64, f64)> {
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
            path: vec![
                ll(0.0, 0.0),
                ll(200.0, 0.0),
                ll(200.0, 60.0),
                ll(200.0, 120.0),
                ll(200.0, 300.0),
            ],
            via: "Whiskey, Sierra five Whiskey, November five Whiskey".into(),
            seg: vec![
                ("W".into(), false),
                ("S5W".into(), false),
                ("09L/27R".into(), true),
                ("N5W".into(), false),
            ],
            holding_point: "N5W".into(),
            lead_on: Vec::new(),
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
    fn cues_lead_by_time_not_meters() {
        // At 18 kts the turn cue fires ~139 m out (15 s of taxi), not at the
        // old 80 m floor that landed "almost exactly at the turn" (pilot
        // feedback, 2026-07-12). The same spot at creep speed stays quiet —
        // the floor only carries slow taxiing.
        let t0 = Instant::now();
        let (lat, lon) = ll(70.0, 0.0); // 130 m before the turn at x=200
        let mut g = Guide::build(&route()).expect("guide");
        assert_eq!(g.tick(lat, lon, 4.0, t0), None, "creep speed: not yet");
        let cue = g.tick(lat, lon, 18.0, t0).expect("early cue at speed");
        assert!(cue.contains("left"), "{cue}");
        // The floor still fires close-in at walking pace.
        let mut g = Guide::build(&route()).expect("guide");
        let (lat, lon) = ll(130.0, 0.0); // 70 m out
        assert!(
            g.tick(lat, lon, 4.0, t0).is_some(),
            "floor carries slow taxi"
        );
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
    fn a_missed_turn_gets_an_immediate_nudge_once() {
        let mut g = Guide::build(&route()).expect("guide");
        let t0 = Instant::now();
        let s = Duration::from_secs;
        // The turn cue speaks on approach...
        let (lat, lon) = ll(130.0, 0.0);
        assert!(g.tick(lat, lon, 10.0, t0).is_some());
        // ...the pilot sails straight past the corner instead. 15 m past is
        // still turning slop (no nudge; the runway call in range speaks).
        let (lat, lon) = ll(215.0, 0.0);
        let cue = g.tick(lat, lon, 10.0, t0 + s(6)).expect("runway call");
        assert!(cue.contains("Runway"), "{cue}");
        // 25 m out and growing: the nudge, plainly and immediately — this is
        // the 20-60 m dead zone where nobody spoke for 3.5 minutes at Malaga
        // (2026-07-13). Past 60 m the controller takes over.
        let (lat, lon) = ll(225.0, 0.0);
        let cue = g.tick(lat, lon, 10.0, t0 + s(12)).expect("nudge");
        assert!(cue.contains("missed our left"), "{cue}");
        // Once: drifting further gets silence (the controller's zone next).
        let (lat, lon) = ll(235.0, 0.0);
        assert_eq!(g.tick(lat, lon, 10.0, t0 + s(18)), None);
    }

    #[test]
    fn sharp_turns_are_called_sharp_and_fast_approaches_get_easy() {
        // East 200 m, then a 135-degree left back to the northwest.
        let w = TaxiWatch {
            path: vec![ll(0.0, 0.0), ll(200.0, 0.0), ll(150.0, 50.0)],
            via: "Whiskey, Sierra".into(),
            seg: vec![("W".into(), false), ("S".into(), false)],
            holding_point: "S".into(),
            lead_on: Vec::new(),
        };
        let t0 = Instant::now();
        // At speed: the words carry the speed advice.
        let mut g = Guide::build(&w).expect("guide");
        let (lat, lon) = ll(80.0, 0.0);
        let cue = g.tick(lat, lon, 18.0, t0).expect("sharp cue at speed");
        assert_eq!(cue, "Easy now. Sharp left next, onto Sierra.");
        // Slow: same turn, no nagging.
        let mut g = Guide::build(&w).expect("guide");
        let (lat, lon) = ll(130.0, 0.0);
        let cue = g.tick(lat, lon, 10.0, t0).expect("sharp cue slow");
        assert_eq!(cue, "Sharp left next, onto Sierra.");
    }

    #[test]
    fn a_turn_after_a_long_straight_gets_a_prep_call() {
        // 400 m straight run, then left onto B: prep at 220, execute at 400.
        let w = TaxiWatch {
            path: vec![ll(0.0, 0.0), ll(400.0, 0.0), ll(400.0, 100.0)],
            via: "Alpha, Bravo".into(),
            seg: vec![("A".into(), false), ("B".into(), false)],
            holding_point: "B".into(),
            lead_on: Vec::new(),
        };
        let mut g = Guide::build(&w).expect("guide");
        let preps: Vec<&Cue> = g.cues.iter().filter(|c| c.kind == CueKind::Prep).collect();
        assert_eq!(preps.len(), 1, "{:?}", g.cues);
        assert_eq!(preps[0].line, "Coming up: left onto Bravo.");
        // It fires on the straight, well before the execute cue...
        let t0 = Instant::now();
        let (lat, lon) = ll(150.0, 0.0);
        let cue = g.tick(lat, lon, 10.0, t0).expect("prep");
        assert!(cue.starts_with("Coming up"), "{cue}");
        // ...and the execute cue still gets its own call at the junction.
        let (lat, lon) = ll(330.0, 0.0);
        let cue = g
            .tick(lat, lon, 10.0, t0 + Duration::from_secs(20))
            .expect("execute");
        assert!(cue.contains("Next left, onto Bravo"), "{cue}");
        // The short route() straights earn no prep — it's for the long legs.
        let g = Guide::build(&route()).expect("guide");
        assert!(g.cues.iter().all(|c| c.kind != CueKind::Prep));
    }

    #[test]
    fn a_long_stop_earns_a_reorientation_when_rolling_resumes() {
        let mut g = Guide::build(&route()).expect("guide");
        let t0 = Instant::now();
        let s = Duration::from_secs;
        let (lat, lon) = ll(60.0, 0.0);
        // Holding position for 25 s (traffic, a checklist)...
        assert_eq!(g.tick(lat, lon, 0.0, t0), None);
        assert_eq!(g.tick(lat, lon, 0.0, t0 + s(25)), None);
        // ...rolling again: where we are and what's next, without consuming
        // the turn cue.
        let (lat, lon) = ll(80.0, 0.0);
        let cue = g.tick(lat, lon, 8.0, t0 + s(30)).expect("reorientation");
        assert!(cue.starts_with("Rolling again. We're on Whiskey."), "{cue}");
        assert!(cue.contains("Next left, onto Sierra five Whiskey"), "{cue}");
        // The turn cue itself still speaks at the decision point.
        let (lat, lon) = ll(130.0, 0.0);
        let cue = g.tick(lat, lon, 10.0, t0 + s(40)).expect("turn cue");
        assert!(cue.contains("Next left"), "{cue}");
        // A SHORT stop (a brake check) earns nothing.
        let mut g = Guide::build(&route()).expect("guide");
        let (lat, lon) = ll(60.0, 0.0);
        assert_eq!(g.tick(lat, lon, 0.0, t0), None);
        assert_eq!(g.tick(lat, lon, 0.0, t0 + s(5)), None);
        assert_eq!(
            g.tick(lat, lon, 8.0, t0 + s(10)),
            None,
            "short stop: no speech"
        );
    }

    #[test]
    fn strip_hint_shows_the_next_decision_by_geometry() {
        let g = Guide::build(&route()).expect("guide");
        let (lat, lon) = ll(100.0, 0.0);
        let hint = g.strip_hint(lat, lon).expect("hint");
        assert_eq!(hint, "NEXT 100m: left onto S5W");
        // Off route: the strip owns up instead of pointing at stale pavement.
        let (lat, lon) = ll(100.0, -200.0);
        assert_eq!(g.strip_hint(lat, lon).as_deref(), Some("TAXI: off route"));
        // Past the turn, the runway crossing is next.
        let (lat, lon) = ll(200.0, 30.0);
        let hint = g.strip_hint(lat, lon).expect("hint");
        assert!(hint.contains("cross rwy 09L"), "{hint}");
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
            lead_on: Vec::new(),
        };
        let g = Guide::build(&w).expect("guide");
        assert!(
            g.cues.iter().all(|c| c.kind != CueKind::Turn),
            "fillet called as a turn: {:?}",
            g.cues
        );
    }
}
