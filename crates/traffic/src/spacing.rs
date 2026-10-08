//! Ground spacing: a taxiing traffic aircraft keeps its distance from
//! whatever stands on the ground AHEAD of it on its own route, another of
//! the fleet or our own aircraft. The BIEG world sweep (2026-10-08) had
//! two departures stop on the same hold, one inside the other ([4] a
//! Cessna and [1] a Ryanair 737 within a metre at runway 03's Alpha), and
//! traffic taxiing through each other and through us.
//!
//! Pure, on the fleet's clock: [`apply`] runs once a substep, before the
//! aircraft move, and leaves each mover the metres it may still roll
//! ([`GroundSpacing::free_m`]); its drive turns that into a speed it can
//! stop from. "Ahead" is along the route, never a cone: the other
//! aircraft is projected onto the next [`LOOKAHEAD_M`] of path, within a
//! corridor of half the mover's span plus [`LATERAL_MARGIN_M`], so a
//! parallel taxiway or a stand beside the line stops nobody.

use crate::aircraft::TrafficAircraft;
use crate::field::FieldModel;
use crate::kin::{bearing, dist_m, offset_m, signed_diff_deg, LatLon};
use crate::perf::{Perf, JET_MED};
use crate::phase::{Event, OwnTarget, Phase};

/// How far along its route a taxiing aircraft looks for traffic, metres:
/// a jet at taxi pace stops in about 55 m at [`SPACING_DECEL_MS2`].
pub const LOOKAHEAD_M: f64 = 150.0;

/// The corridor along the route is half the mover's wingspan plus this,
/// either side of the line, metres.
pub const LATERAL_MARGIN_M: f64 = 5.0;

/// The braking the spacing plans its stop on, metres a second squared:
/// under the taxi follower's own 2 kt/s (1.03), so the plan is always
/// flyable and the stop never lands inside the spacing.
pub const SPACING_DECEL_MS2: f64 = 0.8;

/// Held this long behind an aircraft that will not move for it (one parked
/// on its stand, or our own), a mover goes on past it rather than gridlock
/// the field ([`Event::SpacingWaived`]).
pub const STUCK_S: f64 = 90.0;

/// A traffic aircraft parked on a stand stops a mover only this close to
/// its line, metres: the network's stand links run past the neighbours'
/// stand points (LEMG, 2026-10-08: three departures in the hour held 90 s
/// on leaving their stands by the aircraft parked 16 m beside them). One
/// truly ON the path still stops it.
pub const PARKED_CORRIDOR_M: f64 = 5.0;

/// Our own aircraft's size, for the spacing behind it: its type is not
/// known here, so it is sized as the commonest airliner, which errs long.
const OWN_PERF: &Perf = &JET_MED;

/// Who a mover is giving way to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Blocker {
    Traffic(u32),
    Own,
}

/// One aircraft's spacing state, set by [`apply`] each substep.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct GroundSpacing {
    /// Metres it may still roll before it is inside the spacing of the
    /// aircraft ahead; `None` when the way ahead is clear.
    pub free_m: Option<f64>,
    /// The aircraft ahead it is spacing on.
    pub blocker: Option<Blocker>,
    /// Seconds stopped behind a blocker that will not move for it.
    pub held_s: f64,
    /// Blockers it has given up waiting for: passed, until its phase
    /// changes.
    pub waived: Vec<Blocker>,
}

/// Nose to tail behind `ahead`, centre to centre, metres: half of each
/// length plus the larger class's gap. Two 172s keep about 38 m, a 737
/// behind a 737 about 78, a 172 behind a 737 about 63.
pub fn spacing_m(mover: &Perf, ahead: &Perf) -> f64 {
    (mover.length_m + ahead.length_m) / 2.0 + mover.taxi_gap_m.max(ahead.taxi_gap_m)
}

/// The half-width of the corridor along a mover's route, metres.
pub fn corridor_m(mover: &Perf) -> f64 {
    mover.span_m / 2.0 + LATERAL_MARGIN_M
}

/// The ground phases that move along a taxi route. The takeoff roll and
/// the landing rollout are the runway's and the tower's, never spaced.
fn taxi_phase(p: Phase) -> bool {
    matches!(
        p,
        Phase::TaxiOut | Phase::LineUp | Phase::Vacate | Phase::TaxiIn
    )
}

/// Right of way between two movers that each have the other ahead (head
/// on): a line-up and a vacate (onto or off a runway) first, then the
/// lower id. Smaller is first.
fn priority(ac: &TrafficAircraft) -> (u8, u32) {
    let rank = if matches!(ac.phase, Phase::Vacate | Phase::LineUp) {
        0
    } else {
        1
    };
    (rank, ac.id)
}

/// The route still ahead of a taxiing aircraft, from where it is, cut at
/// [`LOOKAHEAD_M`]; `None` when it is not taxiing or its route is done.
fn way_ahead(ac: &TrafficAircraft) -> Option<Vec<LatLon>> {
    let r = &ac.route;
    if !taxi_phase(ac.phase) || r.follow.done || r.path.len() < 2 || !ac.kin.on_ground {
        return None;
    }
    let mut way = vec![ac.kin.pos()];
    let mut left = LOOKAHEAD_M;
    for &p in r.path.iter().skip(r.follow.seg + 1) {
        let last = *way.last().unwrap_or(&p);
        let d = dist_m(last, p);
        if d >= left {
            let (e, n) = offset_m(last, p);
            let f = left / d.max(1e-9);
            way.push(crate::kin::shift(last, e * f, n * f));
            break;
        }
        left -= d;
        way.push(p);
    }
    (way.len() >= 2).then_some(way)
}

/// Where `p` sits against `way`: metres along it, metres off it, and the
/// way's bearing there. Nothing behind its start; a point round the
/// outside of a corner (or just past the end) is measured to the vertex.
fn project(way: &[LatLon], p: LatLon) -> Option<(f64, f64, f64)> {
    let mut best: Option<(f64, f64, f64)> = None;
    let mut cum = 0.0;
    let mut keep = |along: f64, off: f64, brg: f64| {
        if best.is_none_or(|(a, o, _)| off < o - 1e-9 || (off < o + 1e-9 && along < a)) {
            best = Some((along, off, brg));
        }
    };
    for w in way.windows(2) {
        let (bx, by) = offset_m(w[0], w[1]);
        let len = bx.hypot(by);
        if len < 1e-6 {
            continue;
        }
        let brg = bearing(w[0], w[1]);
        let (px, py) = offset_m(w[0], p);
        let t = (px * bx + py * by) / (len * len);
        if (0.0..=1.0).contains(&t) {
            keep(cum + t * len, ((px * by - py * bx) / len).abs(), brg);
        }
        // The vertex at this segment's end: a corner's outside, or the end.
        keep(cum + len, dist_m(w[1], p), brg);
        cum += len;
    }
    best
}

/// Something on the ground a mover may have to space on.
struct Obstacle {
    who: Blocker,
    /// Its index in the fleet (traffic only).
    idx: Option<usize>,
    pos: LatLon,
    /// Its heading, where known (ours is not).
    hdg: Option<f64>,
    perf: &'static Perf,
    /// Parked on a stand (or ours): it will not move for anybody.
    parked: bool,
}

/// One obstacle ahead of a mover.
#[derive(Clone, Copy)]
struct Ahead {
    obstacle: usize,
    along_m: f64,
}

/// Space every taxiing aircraft in `fleet` at `field` on the ground
/// traffic ahead of it (the fleet's and our own `own`), for one substep of
/// `dt`. Sets each aircraft's [`GroundSpacing`]; a mover held [`STUCK_S`]
/// behind one that will not move gives up on it and says so in `events`.
pub fn apply(
    fleet: &mut [TrafficAircraft],
    field: &FieldModel,
    own: Option<&OwnTarget>,
    dt: f64,
    events: &mut Vec<Event>,
) {
    let mut obstacles: Vec<Obstacle> = fleet
        .iter()
        .enumerate()
        .filter(|(_, a)| {
            a.kin.on_ground && !matches!(a.phase, Phase::Roll | Phase::Landing | Phase::Gone)
        })
        .map(|(i, a)| Obstacle {
            who: Blocker::Traffic(a.id),
            idx: Some(i),
            pos: a.kin.pos(),
            hdg: Some(a.kin.hdg_deg),
            perf: a.perf,
            parked: a.phase == Phase::Parked,
        })
        .collect();
    if let Some(o) = own.filter(|o| o.on_ground) {
        obstacles.push(Obstacle {
            who: Blocker::Own,
            idx: None,
            pos: o.pos,
            hdg: None,
            perf: OWN_PERF,
            parked: true,
        });
    }
    let ways: Vec<Option<Vec<LatLon>>> = fleet.iter().map(way_ahead).collect();
    let mover = |i: usize| ways[i].is_some();
    // Everything ahead of each mover, nearest first.
    let mut ahead: Vec<Vec<Ahead>> = vec![Vec::new(); fleet.len()];
    for (i, way) in ways.iter().enumerate() {
        let Some(way) = way else { continue };
        let ac = &fleet[i];
        // Never stopped ON a runway: a vacate held there behind the queue
        // for a crossing of that same runway held the runway, the crossing
        // was never cleared and the field gridlocked (BIEG 03, 2026-10-08).
        // Off the strip it spaces as anyone does.
        if field.on_runway(ac.kin.pos()) {
            continue;
        }
        for (k, ob) in obstacles.iter().enumerate() {
            if ob.idx == Some(i) || ac.spacing.waived.contains(&ob.who) {
                continue;
            }
            let Some((along, off, brg)) = project(way, ob.pos) else {
                continue;
            };
            let on_stand = ob.parked && ob.idx.is_some();
            // Started up INSIDE one parked on its own stand (two put on one
            // stand: CYJP and EFMI in the hour sweep, 2026-10-08), it
            // can only drive out of it: waiting held it 90 s every time.
            if on_stand && dist_m(ac.kin.pos(), ob.pos) < PARKED_CORRIDOR_M {
                continue;
            }
            let corridor = if on_stand {
                PARKED_CORRIDOR_M
            } else {
                corridor_m(ac.perf)
            };
            if off > corridor {
                continue;
            }
            // One standing still and facing it (head on: a departure at its
            // hold met by an arrival taxiing in past it, a flight held short
            // of a crossing) is passed: it will not come on, and waiting
            // for the tower to move it only holds the mover. UUYY, the hour
            // sweep (2026-10-08): six arrivals stood 90 s each nose to nose
            // with a departure the tower held at the hold for half an hour.
            // A queue (the same way) is waited in.
            let still = !ob.parked && ob.idx.is_some_and(|j| !mover(j));
            if still && ob.hdg.is_some_and(|h| signed_diff_deg(h, brg).abs() > 90.0) {
                continue;
            }
            ahead[i].push(Ahead {
                obstacle: k,
                along_m: along,
            });
        }
        ahead[i].sort_by(|a, b| a.along_m.total_cmp(&b.along_m));
    }
    // Head on and round in circles: movers each waiting on the next. The
    // first by right of way in every such cycle goes on past the others.
    let n = fleet.len();
    let mut ignore: Vec<Vec<usize>> = vec![Vec::new(); n];
    let nearest = |i: usize, ignore: &[Vec<usize>]| -> Option<Ahead> {
        ahead[i].iter().copied().find(|a| {
            obstacles[a.obstacle]
                .idx
                .is_none_or(|j| !ignore[i].contains(&j))
        })
    };
    for _ in 0..=n {
        let next: Vec<Option<usize>> = (0..n)
            .map(|i| {
                nearest(i, &ignore)
                    .and_then(|a| obstacles[a.obstacle].idx)
                    .filter(|&j| mover(j))
            })
            .collect();
        let mut found = false;
        for start in 0..n {
            let mut chain = vec![start];
            let mut at = start;
            while let Some(j) = next[at] {
                if let Some(pos) = chain.iter().position(|&c| c == j) {
                    let cycle = &chain[pos..];
                    let first = *cycle
                        .iter()
                        .min_by_key(|&&c| priority(&fleet[c]))
                        .expect("a cycle has members");
                    for &c in cycle {
                        if c != first && !ignore[first].contains(&c) {
                            ignore[first].push(c);
                            found = true;
                        }
                    }
                    break;
                }
                if chain.len() > n {
                    break;
                }
                chain.push(j);
                at = j;
            }
        }
        if !found {
            break;
        }
    }
    let picks: Vec<Option<Ahead>> = (0..n).map(|i| nearest(i, &ignore)).collect();
    for (i, ac) in fleet.iter_mut().enumerate() {
        let pick = picks[i].filter(|_| ways[i].is_some());
        let Some(a) = pick else {
            ac.spacing.free_m = None;
            ac.spacing.blocker = None;
            ac.spacing.held_s = 0.0;
            continue;
        };
        let ob = &obstacles[a.obstacle];
        let free = a.along_m - spacing_m(ac.perf, ob.perf);
        if ac.spacing.blocker != Some(ob.who) {
            ac.spacing.held_s = 0.0;
        }
        ac.spacing.blocker = Some(ob.who);
        ac.spacing.free_m = Some(free);
        // Waiting on one that will never move for it: bounded.
        if ob.parked && ac.kin.gs_kts < 0.5 && free < 1.0 {
            ac.spacing.held_s += dt;
            if ac.spacing.held_s >= STUCK_S {
                ac.spacing.waived.push(ob.who);
                ac.spacing.free_m = None;
                ac.spacing.blocker = None;
                ac.spacing.held_s = 0.0;
                events.push(Event::SpacingWaived {
                    id: ac.id,
                    past: match ob.who {
                        Blocker::Traffic(id) => Some(id),
                        Blocker::Own => None,
                    },
                });
            }
        } else if !ob.parked {
            ac.spacing.held_s = 0.0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::perf::{JET_HEAVY, PISTON_SINGLE};

    #[test]
    fn the_spacing_is_sized_from_both_aircraft() {
        let light = spacing_m(&PISTON_SINGLE, &PISTON_SINGLE);
        let jets = spacing_m(&JET_MED, &JET_MED);
        assert!((35.0..45.0).contains(&light), "{light:.0} m");
        assert!((65.0..85.0).contains(&jets), "{jets:.0} m");
        assert!(spacing_m(&PISTON_SINGLE, &JET_MED) > light);
        assert!(spacing_m(&JET_HEAVY, &JET_MED) > jets);
        assert!(corridor_m(&PISTON_SINGLE) < corridor_m(&JET_MED));
    }

    #[test]
    fn a_point_is_ahead_only_along_the_way() {
        use crate::kin::shift;
        let o = (36.0, -5.0);
        let way = vec![o, shift(o, 0.0, 100.0), shift(o, 100.0, 100.0)];
        let (along, off, _) = project(&way, shift(o, 3.0, 50.0)).unwrap();
        assert!((along - 50.0).abs() < 0.5 && (off - 3.0).abs() < 0.5);
        // Round the corner, on the second leg.
        let (along, off, _) = project(&way, shift(o, 40.0, 101.0)).unwrap();
        assert!((along - 140.0).abs() < 0.5 && off < 1.5);
        // Behind the start is never ahead: measured to the far vertices.
        let (_, off, _) = project(&way, shift(o, 0.0, -20.0)).unwrap();
        assert!(off > 100.0);
    }
}
