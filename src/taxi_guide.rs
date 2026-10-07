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
    /// The bend's signed angle, degrees (positive = right); zero for cues
    /// that are not turns. The panel's arrow is drawn from it.
    turn_deg: f64,
}

/// The per-clearance guide: cues precomputed from the published route, each
/// spoken at most once as the aircraft's along-route progress approaches it.
pub struct Guide {
    /// The route this guide was built for — the monitor rebuilds on change.
    path: Vec<(f64, f64)>,
    /// The same route in the local metre frame (x east, y north from
    /// `path[0]`) with cumulative distances, for the turn card's aim point.
    pts: Vec<(f64, f64)>,
    cum: Vec<f64>,
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
    /// Whether the last cue [`Guide::tick`] returned is one the pilot needs
    /// even when the copilot is driving: a runway ahead, the holding point.
    last_safety: bool,
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
                    turn_deg: 0.0,
                });
                continue;
            }
            if leg_runway {
                continue; // taxiing along the crossing itself — nothing to say
            }
            let bearing_in = bearing(point_at(&pts, &cum, cum[i] - WINDOW_M), pts[i]);
            let bearing_out = bearing(pts[i], point_at(&pts, &cum, cum[i] + WINDOW_M));
            let delta = crate::angles::signed_diff_deg(bearing_out, bearing_in);
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
                    turn_deg: delta,
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
                    turn_deg: 0.0,
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
            turn_deg: 0.0,
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
                        turn_deg: cue.turn_deg,
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
            pts,
            cum,
            cues,
            called,
            nudged,
            names,
            last_call: None,
            prev_fix: None,
            stopped_at: None,
            reorient_armed: false,
            last_safety: false,
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
        // A tug has us: nothing about the route ahead is a cue yet.
        if crate::flight_state::Owner::Pushback.active() {
            self.prev_fix = None;
            return None;
        }
        let (progress, lateral) = project(&self.path, lat, lon)?;
        let prev = self.prev_fix.replace((progress, lateral));
        if lateral > OFF_ROUTE_M {
            return None;
        }
        // Moving BACKWARDS along the route — a push, or a pilot reversing.
        // Groundspeed can't tell direction (it is a magnitude), so the route
        // itself has to: "next left, onto Sierra" is never a cue for an
        // aircraft travelling the other way. Slack for GPS jitter at a
        // standstill; the guide simply waits until we roll forward again.
        if prev.is_some_and(|(p, _)| progress < p - 1.0) {
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
                    self.last_safety = false;
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
            self.last_safety = false;
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
        self.last_safety = matches!(cue.kind, CueKind::RunwayAhead | CueKind::HoldingPoint);
        // Fast into a sharp bend: the words carry the speed advice too.
        if cue.kind == CueKind::Turn && cue.sharp && gs_kts > EASY_KTS {
            return Some(format!("Easy now. {}", cue.line));
        }
        Some(cue.line.clone())
    }

    /// The last cue is a safety call (a runway ahead, the holding point),
    /// spoken whoever is driving. The rest is guidance for a pilot's hands:
    /// with the copilot driving, "follow it around to the right" eight times
    /// down one taxi is the copilot narrating its own steering (LIBP
    /// 2026-09-26), so the monitor shows those on the panel and says nothing.
    pub fn last_cue_is_safety(&self) -> bool {
        self.last_safety
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

/// The panel's GPS-style turn card: the next decision point with its
/// distance and bend, the route's bearing a little ahead of the aircraft
/// (the plugin turns it against the live heading every frame, so the arrow
/// is smooth at the sim's rate while this side ticks at the monitor's), and
/// the signed cross-track. Geometry only, like [`Guide::strip_hint`]: the
/// card shows what pavement comes next whether or not the cue was spoken.
#[derive(Debug, Clone, PartialEq)]
pub struct TurnCard {
    /// One letter for the plugin: T turn, C runway crossing, N new taxiway
    /// name, H the holding point or stand, X off the route.
    pub kind: char,
    /// Metres to the decision point along the route (0 off route).
    pub dist_m: f64,
    /// The decision point's terse ASCII label (`Cue::short`).
    pub label: String,
    /// True bearing of the route [`AIM_AHEAD_M`] ahead of the projection.
    pub aim_true_deg: f64,
    /// Signed cross-track, metres, positive right of the route.
    pub xtk_m: f64,
    /// The bend at the decision point, degrees, positive right; 0 unless a
    /// turn.
    pub turn_deg: f64,
    /// Metres of route left to its end.
    pub remaining_m: f64,
}

/// How far ahead of the aircraft the card's aim bearing is taken: enough to
/// lead into a bend the way a pilot looks, not so far that a 90 degree turn
/// reads as a diagonal from the leg before it.
const AIM_AHEAD_M: f64 = 25.0;

impl TurnCard {
    /// The wire form for `xcopilot/status/taxi`:
    /// `kind|dist_m|label|aim_true_deg|xtk_m|turn_deg|remaining_m`, every
    /// number an integer but the cross-track (one decimal). ASCII only; the
    /// label never carries a '|'.
    pub fn payload(&self) -> String {
        format!(
            "{}|{:.0}|{}|{:.0}|{:.1}|{:.0}|{:.0}",
            self.kind,
            self.dist_m,
            self.label.replace('|', "/"),
            self.aim_true_deg.rem_euclid(360.0),
            self.xtk_m,
            self.turn_deg,
            self.remaining_m,
        )
    }
}

impl Guide {
    /// The turn card for a position, `None` on a degenerate route. Off the
    /// route (past the compliance watch's line) the card says so and points
    /// nowhere: the controller owns that conversation, and an arrow back to
    /// the paint would be a second voice on one mistake.
    pub fn turn_card(&self, lat: f64, lon: f64) -> Option<TurnCard> {
        let (progress, xtk) = project_signed(&self.path, lat, lon)?;
        let total = *self.cum.last()?;
        let remaining = (total - progress).max(0.0);
        if xtk.abs() > OFF_ROUTE_M {
            return Some(TurnCard {
                kind: 'X',
                dist_m: 0.0,
                label: "off route".into(),
                aim_true_deg: 0.0,
                xtk_m: xtk,
                turn_deg: 0.0,
                remaining_m: remaining,
            });
        }
        let here = point_at(&self.pts, &self.cum, progress);
        let ahead = point_at(&self.pts, &self.cum, progress + AIM_AHEAD_M);
        // At the very end the route has no "ahead": hold the last leg's
        // bearing rather than a zero-length one.
        let aim = if dist(here, ahead) < 1.0 {
            bearing(point_at(&self.pts, &self.cum, total - AIM_AHEAD_M), ahead)
        } else {
            bearing(here, ahead)
        };
        let cue = self
            .cues
            .iter()
            .find(|c| c.kind != CueKind::Prep && c.at_m - progress > -PASSED_M)?;
        let kind = match cue.kind {
            CueKind::Turn => 'T',
            CueKind::RunwayAhead => 'C',
            CueKind::NameChange => 'N',
            CueKind::HoldingPoint => 'H',
            CueKind::Prep => unreachable!("preps are filtered above"),
        };
        Some(TurnCard {
            kind,
            dist_m: (cue.at_m - progress).max(0.0),
            label: cue.short.clone(),
            aim_true_deg: aim,
            xtk_m: xtk,
            turn_deg: cue.turn_deg,
            remaining_m: remaining,
        })
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

/// Project a position onto the route: (along-route progress m, DISTANCE from
/// it in m — never negative). `None` on a degenerate path.
///
/// The magnitude is what every guard wants: off the route by more than X is
/// the same fault whichever side it happened on, and a signed value silently
/// disables such a test on one side of the line. Use
/// [`project_signed`] where the SIDE is the question — which is diagnosis,
/// and any law that wants to close an offset rather than merely notice it.
pub(crate) fn project(path: &[(f64, f64)], lat: f64, lon: f64) -> Option<(f64, f64)> {
    project_signed(path, lat, lon).map(|(along, off)| (along, off.abs()))
}

/// [`project`] keeping the SIDE: positive is right of the route's direction
/// of travel, negative is left.
///
/// Worth having because a magnitude cannot be diagnosed. Every taxi drive
/// blackboxed `lat_m` as a distance, so a whole flight's record could say the
/// aircraft ran two metres off the line and never which side — which is
/// exactly the question when the pilot is looking at the paint out of the
/// window (Malaga, 2026-08-05). Same family as the groundspeed the taxi guide
/// read as forward motion during a pushback: an unsigned quantity standing in
/// for a signed one reads plausibly and is wrong half the time.
pub(crate) fn project_signed(path: &[(f64, f64)], lat: f64, lon: f64) -> Option<(f64, f64)> {
    project_signed_with(path, lat, lon, None)
}

/// [`project_signed`] onto the part of the route that runs the way the
/// aircraft is FACING (segments within 90 degrees of `heading_true`), falling
/// back to the plain nearest when none does. For joining a route: the nearest
/// paint can be a leg already behind the nose. After a pushback that stopped
/// 60 m short of its node, the plain projection put the Baron on the
/// stand-to-node leg it had just been pushed down, and the join drove a
/// loop back to the node before setting off (Malaga, 12:11Z, 2026-09-11).
///
/// ...but only while the facing paint is paint we could actually be joining.
/// The filter used to be all-or-nothing - whatever survived it won, however
/// far away, and the plain projection was consulted only when NOTHING
/// survived. At Liege (2026-09-22) the taxi-in route ran back the way the
/// aircraft had just vacated, so at the engage every segment was outside the
/// window, the fallback correctly gave 0.9 m at 5 m along, and `remaining`
/// sat frozen there for fourteen seconds while the drive tried to turn round.
/// Then the nose swung far enough for ONE far segment - the leg back across
/// the runway - to enter the window, and it became the whole projection:
/// offset 13 m to 155 m and progress 5 m to 144.6 m between two ticks, which
/// tripped the off-route guard and stopped the aircraft with "we're off the
/// cleared route" about a route it was sitting on. A jump in a projection is
/// never a movement of the aircraft. Past [`FACING_REACH_M`] the facing pick
/// is not a leg we are joining, it is a leg somewhere else, and the nearest
/// paint is the honest answer.
///
/// ...and the reach is measured from the paint under us, not only from the
/// aircraft. LEMG 13 (sortie 1790646817-full_auto, 2026-09-29, #178): a
/// runway start turning round to a route behind it had the plain answer 8.3
/// m off when a far leg swung into the window 149.4 m away, a metre inside
/// the old flat 150 m reach, and the join's own guard (90 m) stopped the
/// aircraft "off the cleared route" 11 s into its turn. A facing leg is a
/// join only within [`FACING_JUMP_M`] of the plain answer, never past
/// [`FACING_REACH_M`].
pub(crate) fn project_signed_facing(
    path: &[(f64, f64)],
    lat: f64,
    lon: f64,
    heading_true: f64,
) -> Option<(f64, f64)> {
    let plain = project_signed_with(path, lat, lon, None);
    let reach = plain.map_or(FACING_REACH_M, |p| {
        (p.1.abs() + FACING_JUMP_M).min(FACING_REACH_M)
    });
    match project_signed_with(path, lat, lon, Some(heading_true)) {
        Some(facing)
            if facing.1.abs() <= reach || plain.is_none_or(|p| facing.1.abs() <= p.1.abs()) =>
        {
            Some(facing)
        }
        _ => plain,
    }
}

/// Lateral drift at which the auto taxi stops the aircraft and gives steering
/// back, once ESTABLISHED on the route. Deliberately much tighter than the
/// guide/compliance 60 m: the law holds a couple of meters in normal work,
/// so 25 m off means something is wrong (bad fix, blocked nosewheel, a
/// route the geometry can't follow).
pub(crate) const OFF_ROUTE_STOP_M: f64 = 25.0;
/// Farther off than this at engage is a refusal: the projection onto the
/// route stops meaning anything, and a blind cross-country drive to it
/// isn't a join, it's a new route the tower never gave us.
pub(crate) const MAX_JOIN_M: f64 = 150.0;
/// Extra lateral allowance granted while joining, on top of the engage
/// distance: room for the turning circle of a turn-around without ever
/// letting a broken law wander further from the route than it started.
pub(crate) const JOIN_GROW_M: f64 = 40.0;

/// How much further off than the plain nearest paint a facing leg may lie
/// and still be the leg we join: the join's own room past the plain answer
/// (the established guard plus the turning circle), so a pick the drive
/// would call off route is never made.
const FACING_JUMP_M: f64 = OFF_ROUTE_STOP_M + JOIN_GROW_M;

/// How far off a facing segment may lie and still be read as the leg the
/// aircraft is joining. The auto-taxi's own engage limit
/// ([`MAX_JOIN_M`]): further off than that is a refusal
/// there, so it cannot be the thing we are steering at here either.
const FACING_REACH_M: f64 = MAX_JOIN_M;

/// How far past the route's end the offset is still taken square to the last
/// leg. An overshoot of a stop is metres; further on, the end point is the
/// honest distance.
const PAST_END_SQUARE_M: f64 = 30.0;

fn project_signed_with(
    path: &[(f64, f64)],
    lat: f64,
    lon: f64,
    facing: Option<f64>,
) -> Option<(f64, f64)> {
    if path.len() < 2 {
        return None;
    }
    let geo = Geo::new(path[0].0);
    let origin = path[0];
    let p = geo.to_m(origin, (lat, lon));
    let pts: Vec<(f64, f64)> = path.iter().map(|&q| geo.to_m(origin, q)).collect();
    let mut cum = 0.0;
    let mut best: Option<(f64, f64)> = None;
    let last = pts.len() - 2;
    for (i, w) in pts.windows(2).enumerate() {
        let (a, b) = (w[0], w[1]);
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let len2 = dx * dx + dy * dy;
        if let Some(hdg) = facing {
            if len2 >= 1.0 && crate::angles::abs_diff_deg(bearing(a, b), hdg) > 90.0 {
                cum += len2.sqrt();
                continue;
            }
        }
        let t = if len2 < 1.0 {
            0.0
        } else {
            (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len2).clamp(0.0, 1.0)
        };
        let c = (a.0 + t * dx, a.1 + t * dy);
        // Coordinates are (east, north), so the cross product of the leg
        // direction with the offset is positive to the LEFT — negate it and
        // the sign reads the way a pilot describes it.
        let side = dy * (p.0 - c.0) - dx * (p.1 - c.1);
        // Past the END of the route the offset is measured square to the
        // last leg, not to its end point: the distance to the point is the
        // overshoot along the paint, and it was being logged as lateral.
        // LIBP (2026-09-26): a 737 rolling straight on through its holding
        // stop, heading error 0.2 degrees, steer 0.01, "lat_m" 0.2 -> 3.85 m
        // over two seconds at 2 kt, and the taxi scored "3.8 m off the line".
        let past_end = if i == last && len2 >= 1.0 {
            (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len2 - 1.0) * len2.sqrt()
        } else {
            0.0
        };
        let d = if past_end > 0.0 && past_end <= PAST_END_SQUARE_M {
            side.abs() / len2.sqrt()
        } else {
            dist(p, c)
        };
        let along = cum + t * len2.sqrt();
        if best.is_none_or(|(_, bd): (f64, f64)| d < bd.abs()) {
            best = Some((along, if side < 0.0 { -d } else { d }));
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

    /// The pushback that let go early (Malaga, 2026-09-11): the route runs
    /// stand → node (south, 120 m) → off to the north-west. The aircraft sits
    /// 60 m down the first leg, 15 m west of it, already facing north-west.
    /// The nearest paint is the leg behind its tail; the leg it should join
    /// is the one it is facing.
    #[test]
    fn a_join_projects_onto_the_leg_the_nose_points_along() {
        // Node at (0,-120); the NW leg runs from it toward (-100, -70).
        let route = [ll(0.0, 0.0), ll(0.0, -120.0), ll(-100.0, -70.0)];
        let (lat, lon) = ll(-15.0, -60.0);
        let (along_plain, _) = project_signed(&route, lat, lon).unwrap();
        assert!(
            (along_plain - 60.0).abs() < 1.0,
            "nearest paint: the stand leg, {along_plain}"
        );
        let nw = bearing((0.0, -120.0), (-100.0, -70.0));
        let (along_facing, off) = project_signed_facing(&route, lat, lon, nw).unwrap();
        assert!(
            along_facing > 120.0,
            "facing NW: the NW leg, {along_facing}"
        );
        assert!(off.abs() < 60.0, "within joining distance: {off}");
        // Facing the way the stand leg runs, the plain answer stands.
        let (along_south, _) = project_signed_facing(&route, lat, lon, 180.0).unwrap();
        assert!((along_south - 60.0).abs() < 1.0, "{along_south}");
        // No leg runs the way we face: the plain nearest, never nothing.
        let east = [ll(0.0, 0.0), ll(200.0, 0.0)];
        assert!(project_signed_facing(&east, ll(100.0, 5.0).0, ll(100.0, 5.0).1, 270.0).is_some());
    }

    /// Liege 2026-09-22: the taxi-in route ran back the way the aircraft had
    /// just vacated, so at the engage no segment faced the nose and the plain
    /// nearest (0.9 m, 5 m along) was right. Fourteen seconds later the nose
    /// had swung far enough for the leg back across the runway to enter the
    /// facing window, and that leg - 155 m away - became the whole projection.
    /// The off-route guard saw 155 m, stopped the aircraft and said "we're off
    /// the cleared route" about a route it was standing on.
    #[test]
    fn a_facing_leg_out_of_reach_never_becomes_the_projection() {
        // A route that leaves us, doubles back 160 m to the side and runs on:
        // the doubled-back leg faces roughly the way we end up pointing.
        let route = [
            ll(0.0, 0.0),
            ll(0.0, -200.0),
            ll(160.0, -200.0),
            ll(160.0, 200.0),
        ];
        // On the first leg, 1 m to the side, pointing back up it (north) -
        // nothing faces us, so the nearest paint is the answer.
        let (lat, lon) = ll(1.0, -5.0);
        let (along, off) = project_signed_facing(&route, lat, lon, 0.0).unwrap();
        assert!(
            (along - 5.0).abs() < 2.0 && off.abs() < 3.0,
            "the paint under us: along {along}, off {off}"
        );
        // Now swung to the north-ish, where the far leg (+160 m out, running
        // north) IS within the facing window. It is still 159 m away and the
        // paint under us is 1 m away: the projection must not jump to it.
        let (along2, off2) = project_signed_facing(&route, lat, lon, 10.0).unwrap();
        assert!(
            off2.abs() < 10.0,
            "a projection that jumps 150 m is not a movement of the aircraft: \
             along {along2}, off {off2}"
        );
        // ...and a facing leg that IS within reach still wins, which is the
        // whole point of the filter (the Malaga pushback above).
        let near = [
            ll(0.0, 0.0),
            ll(0.0, -200.0),
            ll(40.0, -200.0),
            ll(40.0, 200.0),
        ];
        let (_, off3) = project_signed_facing(&near, lat, lon, 10.0).unwrap();
        assert!(off3.abs() > 20.0, "the facing leg 39 m out: {off3}");
    }

    /// #178, LEMG 13 (sortie 1790646817-full_auto, 2026-09-29): a runway
    /// start on 13 (heading 132) joining a route whose first leg runs 334,
    /// behind it. 11 s into the left turn round, nose 107 true, the plain
    /// nearest was the route's start 8.3 m off; a later leg running 110,
    /// about 149 m away, entered the facing window and became the projection
    /// (lateral 149.4 m, a metre inside the old flat 150 m reach), and the
    /// join's 90 m guard stopped the taxi "off the cleared route".
    #[test]
    fn a_turn_round_join_never_jumps_to_a_far_facing_leg() {
        let at = |x0: f64, y0: f64, brg: f64, d: f64| {
            (x0 + d * brg.to_radians().sin(), y0 + d * brg.to_radians().cos())
        };
        let b = at(0.0, 0.0, 334.0, 215.0);
        let c = at(b.0, b.1, 110.0, 400.0);
        let route = [ll(0.0, 0.0), ll(b.0, b.1), ll(c.0, c.1)];
        // Where the aircraft was: 8 m on from the start, nose round to 107.
        let here = at(0.0, 0.0, 111.0, 8.0);
        let (lat, lon) = ll(here.0, here.1);
        let (_, plain) = project_signed(&route, lat, lon).unwrap();
        assert!((plain.abs() - 8.0).abs() < 0.5, "the paint under us: {plain}");
        // The far leg alone, as the old reach saw it: inside 150 m.
        let far = project_signed_with(&route, lat, lon, Some(107.0)).unwrap();
        assert!(far.1.abs() > 140.0 && far.1.abs() <= FACING_REACH_M, "{far:?}");
        let (along, off) = project_signed_facing(&route, lat, lon, 107.0).unwrap();
        assert!(
            off.abs() < 10.0 && along < 1.0,
            "the join stays on the paint it is turning to: along {along}, off {off}"
        );
    }

    /// Right of the route is positive, left is negative, and the plain
    /// [`project`] is still the distance every guard compares against.
    ///
    /// Pinned because the two are one keystroke apart at the call site and
    /// the failure is silent: a guard written `off > LIMIT` against a signed
    /// value simply stops firing on one side of the line, and nothing about
    /// the number looks wrong in a log.
    #[test]
    fn the_side_of_the_route_is_recorded_and_the_guards_still_see_a_distance() {
        // Due east; a point 5 m north of it is to the LEFT.
        let east = [ll(0.0, 0.0), ll(200.0, 0.0)];
        let (along, off) = project_signed(&east, ll(100.0, 5.0).0, ll(100.0, 5.0).1).unwrap();
        assert!((along - 100.0).abs() < 1.0, "{along}");
        assert!(
            (off + 5.0).abs() < 0.2,
            "north of an eastbound leg is left: {off}"
        );
        let (_, right) = project_signed(&east, ll(100.0, -5.0).0, ll(100.0, -5.0).1).unwrap();
        assert!((right - 5.0).abs() < 0.2, "south of it is right: {right}");
        // Reverse the leg and the sides swap — the sign is about the
        // direction of travel, not the compass.
        let west = [ll(200.0, 0.0), ll(0.0, 0.0)];
        let (_, flipped) = project_signed(&west, ll(100.0, 5.0).0, ll(100.0, 5.0).1).unwrap();
        assert!((flipped - 5.0).abs() < 0.2, "{flipped}");
        // And the unsigned door every gate comes through is unchanged.
        for p in [ll(100.0, 5.0), ll(100.0, -5.0)] {
            let (_, d) = project(&east, p.0, p.1).unwrap();
            assert!((d - 5.0).abs() < 0.2, "project stays a distance: {d}");
        }
    }

    /// LIBP 04's holding point (sortie 1790426786, a 737): past the route's
    /// end the aircraft rolled straight on at 2 to 3 kt, 0.19 m right of the
    /// last leg, and the logged `lat_m` grew with the OVERSHOOT (the recorded
    /// values below), scoring the taxi "3.8 m off the line". Measured square
    /// to the last leg it is the 0.19 m it really was.
    #[test]
    fn rolling_past_the_end_of_the_route_is_not_lateral() {
        const RECORDED_LAT: [f64; 15] = [
            0.56, 0.75, 1.16, 1.51, 1.78, 1.91, 2.16, 2.30, 2.52, 2.78, 2.99, 3.18, 3.45, 3.58,
            3.85,
        ];
        let east = [ll(0.0, 0.0), ll(100.0, 0.0)];
        for lat in RECORDED_LAT {
            let over = (lat * lat - 0.19 * 0.19).sqrt();
            let p = ll(100.0 + over, -0.19);
            // The old reading: the distance to the end point.
            assert!((dist((over, -0.19), (0.0, 0.0)) - lat).abs() < 0.01);
            let (along, off) = project_signed(&east, p.0, p.1).unwrap();
            assert!(
                (off - 0.19).abs() < 0.05,
                "{over:.2} m past the end reads {off:.2} m off"
            );
            assert!(
                (along - 100.0).abs() < 0.5,
                "progress stays at the end: {along}"
            );
        }
        // Far past the end the end point is the honest distance again.
        let (_, off) = project_signed(&east, ll(160.0, -0.19).0, ll(160.0, -0.19).1).unwrap();
        assert!(off > 50.0, "{off}");
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
        assert!(!g.last_cue_is_safety(), "a turn is guidance for the pilot's hands");
        assert!(cue.contains("Sierra five Whiskey"), "{cue}");
        // Same spot again: spoken once.
        assert_eq!(g.tick(lat, lon, 10.0, t0 + s(20)), None);
        // Past the turn, heading north: the runway call (already in range,
        // waiting on the cooldown gap).
        let (lat, lon) = ll(200.0, 10.0);
        let cue = g.tick(lat, lon, 10.0, t0 + s(30)).expect("runway cue");
        assert!(cue.contains("Runway zero niner left"), "{cue}");
        assert!(g.last_cue_is_safety(), "a runway ahead is said whoever drives");
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
    fn turn_card_reads_the_next_decision_point_and_the_route_ahead() {
        let g = Guide::build(&route()).expect("guide");
        // 100 m along the eastbound leg, 3 m right (south) of the paint: the
        // left turn onto S5W is 100 m ahead, the route ahead runs east.
        let (lat, lon) = ll(100.0, -3.0);
        let c = g.turn_card(lat, lon).expect("card");
        assert_eq!(c.kind, 'T');
        assert!((c.dist_m - 100.0).abs() < 1.0, "{c:?}");
        assert_eq!(c.label, "left onto S5W");
        assert!((c.aim_true_deg - 90.0).abs() < 1.0, "{c:?}");
        assert!((c.xtk_m - 3.0).abs() < 0.5, "{c:?}");
        assert!((c.turn_deg + 90.0).abs() < 1.0, "left is negative: {c:?}");
        assert!((c.remaining_m - 400.0).abs() < 1.0, "{c:?}");
        assert_eq!(c.payload(), "T|100|left onto S5W|90|3.0|-90|400");
        // Nearing the corner the aim point goes round it and the arrow
        // leans into the turn, more the closer we get: a lead, not a flip.
        let (lat, lon) = ll(180.0, 0.0);
        let c = g.turn_card(lat, lon).expect("card");
        assert!((60.0..89.0).contains(&c.aim_true_deg), "{c:?}");
        let (lat, lon) = ll(195.0, 0.0);
        let c = g.turn_card(lat, lon).expect("card");
        assert!(c.aim_true_deg < 30.0, "{c:?}");
        // Past the turn the crossing is next, then the name, then the hold.
        let (lat, lon) = ll(200.0, 30.0);
        let c = g.turn_card(lat, lon).expect("card");
        assert_eq!((c.kind, c.label.as_str()), ('C', "cross rwy 09L"));
        assert!(
            (c.aim_true_deg).abs() < 1.0 || (c.aim_true_deg - 360.0).abs() < 1.0,
            "{c:?}"
        );
        let (lat, lon) = ll(200.0, 200.0);
        let c = g.turn_card(lat, lon).expect("card");
        assert_eq!((c.kind, c.label.as_str()), ('H', "hold N5W"));
        assert!((c.dist_m - 100.0).abs() < 1.0, "{c:?}");
        // At the end the aim holds the last leg's bearing, no zero-length leg.
        let (lat, lon) = ll(200.0, 300.0);
        let c = g.turn_card(lat, lon).expect("card");
        assert!(
            c.aim_true_deg.abs() < 1.0 || (c.aim_true_deg - 360.0).abs() < 1.0,
            "{c:?}"
        );
        assert!(c.remaining_m < 1.0, "{c:?}");
    }

    #[test]
    fn turn_card_off_route_points_nowhere() {
        let g = Guide::build(&route()).expect("guide");
        let (lat, lon) = ll(100.0, -200.0);
        let c = g.turn_card(lat, lon).expect("card");
        assert_eq!(c.kind, 'X');
        assert!(c.xtk_m > 60.0, "right of the paint is positive: {c:?}");
        assert!(
            c.payload().starts_with("X|0|off route|0|"),
            "{}",
            c.payload()
        );
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
