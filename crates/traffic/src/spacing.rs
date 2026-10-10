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
//!
//! NOTHING IS DRIVEN THROUGH ANYBODY (2026-10-09). The first cut let the
//! mover with the right of way in a head-on meeting go on past the other,
//! and gave up after [`STUCK_S`] on one that would not move: drawn in
//! X-Plane as one aircraft passing through another (the hour sweep counted
//! 102 pairs inside each other). Now two movers about to meet head on
//! along one stretch of taxiway are kept apart BEFORE it: the one giving
//! way holds short of the stretch, at the junction it would join it by,
//! until the other is through ([`meeting_entry_m`]); two about to cross
//! are kept apart the same way, the nearer first. Met too late for
//! that, both stop by the spacing. And the guard, rather than pass, plans
//! the waiting aircraft another way round on the network
//! ([`TrafficAircraft::reroute_around`]); with none, a traffic aircraft
//! that will not move is towed clear ([`Event::SpacingCleared`]). Our own
//! aircraft is never towed and never driven through: traffic waits for us.

use crate::aircraft::TrafficAircraft;
use crate::field::FieldModel;
use crate::kin::{bearing, dist_m, offset_m, project_m, signed_diff_deg, LatLon};
use crate::perf::{Perf, JET_MED};
use crate::phase::{Despawn, Event, OwnTarget, Phase};

/// How far along its route a taxiing aircraft looks for traffic, metres:
/// a jet at taxi pace stops in about 55 m at [`SPACING_DECEL_MS2`].
pub const LOOKAHEAD_M: f64 = 150.0;

/// How far along both routes two movers look for a stretch they would
/// meet head on in, metres: far enough that the one giving way is told
/// while it can still stop short of its junction.
pub const MEETING_LOOKAHEAD_M: f64 = 400.0;

/// The corridor along the route is half the mover's wingspan plus this,
/// either side of the line, metres.
pub const LATERAL_MARGIN_M: f64 = 5.0;

/// The braking the spacing plans its stop on, metres a second squared:
/// under the taxi follower's own 2 kt/s (1.03), so the plan is always
/// flyable and the stop never lands inside the spacing.
pub const SPACING_DECEL_MS2: f64 = 0.8;

/// Held this long behind an aircraft that will not move for it (one parked
/// on its stand, one standing nose to nose with it, or our own), a mover
/// plans another way round it, else has it towed clear
/// ([`Event::SpacingRerouted`], [`Event::SpacingCleared`]); a traffic
/// aircraft standing still this long in somebody's way is stuck.
pub const STUCK_S: f64 = 90.0;

/// A vacate stopped ON the runway behind somebody looks for another way
/// off after only this long: the runway is held while it waits (a vacate
/// held there behind the queue for a crossing of that same runway held the
/// runway, the crossing was never cleared and the field gridlocked, BIEG
/// 03, 2026-10-08).
pub const RUNWAY_STUCK_S: f64 = 5.0;

/// On a runway, braking for one standing off it this much short of its
/// stopping distance brings the guard at once, metres.
const STRIP_GUARD_M: f64 = 10.0;

/// On a runway an aircraft closes up to this short of the one ahead,
/// beyond the two half lengths, metres: never the taxi spacing (it would
/// stop it on the strip), never through it.
pub const CONTACT_MARGIN_M: f64 = 3.0;

/// How far past the contact distance a mover's nose looks straight ahead
/// for one in its way, whatever its route says, metres.
const NOSE_REACH_M: f64 = 30.0;

/// A traffic aircraft parked on a stand stops a mover only this close to
/// its line, metres: the network's stand links run past the neighbours'
/// stand points (LEMG, 2026-10-08: three departures in the hour held 90 s
/// on leaving their stands by the aircraft parked 16 m beside them). One
/// truly ON the path still stops it.
pub const PARKED_CORRIDOR_M: f64 = 5.0;

/// Two routes meet (head on, or across) where they come together at more
/// than this to each other, degrees; less is one behind the other, which
/// the spacing behind the one ahead keeps.
const MEETING_DEG: f64 = 45.0;

/// One standing on the route faces the mover when its heading is more than
/// this off the route's, degrees: it will not come on.
const FACING_DEG: f64 = 120.0;

/// The meeting stretch is looked for every this many metres along a route.
const SAMPLE_M: f64 = 4.0;

/// The one giving way stops with its nose this short of the stretch.
const HOLD_MARGIN_M: f64 = 2.0;

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
    /// aircraft ahead (or at the junction it gives way at); `None` when
    /// the way ahead is clear.
    pub free_m: Option<f64>,
    /// The aircraft ahead it is spacing on, or giving way to.
    pub blocker: Option<Blocker>,
    /// Seconds stopped behind a blocker that will not move for it.
    pub held_s: f64,
    /// Seconds standing still on the ground, whatever the reason.
    pub still_s: f64,
    /// The movers it holds short of a meeting stretch for: kept while it
    /// can still hold, so the right of way does not flip as both move. With
    /// each, where it holds while the other's route runs on past
    /// [`MEETING_LOOKAHEAD_M`]: the stretch is seen only that far, so its
    /// start ran toward the one holding as the other came on, the hold with
    /// it, and the one giving way was let go up the very lane it had
    /// stopped short of (ENAT 11, the hour sweep, 2026-10-09). The hold
    /// never runs toward it now; it moves on only when the stretch does.
    pub giving_way: Vec<(u32, Option<HoldAt>)>,
    /// The blocker the guard found no way round and nothing to tow: it
    /// waits, said once ([`Event::SpacingHeld`]).
    pub no_way: Option<Blocker>,
    /// The one that will not move a vacate already tried to plan its way
    /// off round, on the move.
    pub tried: Option<Blocker>,
}

impl GroundSpacing {
    /// A new phase is a new route: what it waited for is forgotten.
    pub fn new_phase(&mut self) {
        self.held_s = 0.0;
        self.giving_way.clear();
        self.no_way = None;
        self.tried = None;
    }
}

/// Nose to tail behind `ahead`, centre to centre, metres: half of each
/// length plus the larger class's gap. Two 172s keep about 38 m, a 737
/// behind a 737 about 78, a 172 behind a 737 about 63.
pub fn spacing_m(mover: &Perf, ahead: &Perf) -> f64 {
    (mover.length_m + ahead.length_m) / 2.0 + mover.taxi_gap_m.max(ahead.taxi_gap_m)
}

/// The closest two aircraft come on a runway, centre to centre, metres.
pub fn contact_m(mover: &Perf, ahead: &Perf) -> f64 {
    (mover.length_m + ahead.length_m) / 2.0 + CONTACT_MARGIN_M
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

/// Right of way between two movers about to meet: a line-up and a vacate
/// (onto or off a runway) first. Smaller is first.
fn rank(ac: &TrafficAircraft) -> u8 {
    if matches!(ac.phase, Phase::Vacate | Phase::LineUp) {
        0
    } else {
        1
    }
}

/// The route still ahead of a taxiing aircraft, from where it is, cut at
/// `len_m`; `None` when it is not taxiing or its route is done.
fn route_ahead(ac: &TrafficAircraft, len_m: f64) -> Option<Vec<LatLon>> {
    let r = &ac.route;
    if !taxi_phase(ac.phase) || r.follow.done || r.path.len() < 2 || !ac.kin.on_ground {
        return None;
    }
    let mut way = vec![ac.kin.pos()];
    let mut left = len_m;
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

/// Where a mover holds short of a meeting stretch: metres along its own
/// route from the route's first point ([`route_s`]), on the route that
/// starts there.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HoldAt {
    s_m: f64,
    from: LatLon,
}

/// How far along its own route an aircraft is, metres from the route's
/// first point: the legs behind it, then its place along the one it is on.
fn route_s(ac: &TrafficAircraft) -> f64 {
    let path = &ac.route.path;
    if path.len() < 2 {
        return 0.0;
    }
    let seg = ac.route.follow.seg.min(path.len() - 2);
    let behind: f64 = path[..=seg].windows(2).map(|w| dist_m(w[0], w[1])).sum();
    let (bx, by) = offset_m(path[seg], path[seg + 1]);
    let (px, py) = offset_m(path[seg], ac.kin.pos());
    let len = bx.hypot(by).max(1e-9);
    behind + ((px * bx + py * by) / len).clamp(0.0, len)
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

/// Where `way` first runs onto a runway's strip and off it again, metres
/// along it; `None` when it never does. One that does not come off again
/// inside the way is on it to the end and beyond (an infinite way off): a
/// crossing that runs down the runway past the lookahead is on the runway
/// all the way it can see (ENAT 11, the hour sweep, 2026-10-09: a Ryanair
/// cleared across, its way 227 m down the strip, was let on to give way
/// at the far exit's junction, ON the runway, nose to nose with a Cirrus
/// coming up the exit, "expedite crossing", until the Cirrus was towed).
fn runway_span(field: &FieldModel, way: &[LatLon]) -> Option<(f64, f64)> {
    let mut along = 0.0;
    let mut s_in = None;
    for w in way.windows(2) {
        let len = dist_m(w[0], w[1]);
        let n = (len / 2.0).ceil().max(1.0) as usize;
        for k in 0..=n {
            let f = k as f64 / n as f64;
            let p = (
                w[0].0 + (w[1].0 - w[0].0) * f,
                w[0].1 + (w[1].1 - w[0].1) * f,
            );
            let on = field.on_runway(p);
            match s_in {
                None if on => s_in = Some(along + len * f),
                Some(a) if !on => return Some((a, along + len * f)),
                _ => {}
            }
        }
        along += len;
    }
    s_in.map(|a| (a, f64::INFINITY))
}

/// A stretch of route in metres: its start, unit direction and length.
type Seg = ((f64, f64), (f64, f64), f64);

/// A route in metres east and north of `origin`.
fn metres(origin: LatLon, way: &[LatLon]) -> Vec<(f64, f64)> {
    way.iter().map(|&p| offset_m(origin, p)).collect()
}

/// How far along `mine` (from its start, metres) it first comes within
/// `need_m` of `other` on its way to where they meet: a stretch it runs
/// the OTHER WAY along it (head on), or a line it crosses. From a side
/// branch that is short of the junction, where the branch comes within
/// `need_m` of the other's line, not the corner. EFOU (the hour sweep,
/// 2026-10-09): a vacate came off its exit onto the taxiway across a
/// departure's nose, each seen by the other only once inside its
/// corridor, and they stopped 0.3 m apart. `None` when the routes never
/// meet (a queue, one behind the other, a parallel taxiway).
pub(crate) fn meeting_entry_m(
    mine: &[(f64, f64)],
    other: &[(f64, f64)],
    need_m: f64,
) -> Option<f64> {
    // Routes whose boxes are further apart than `need_m` never meet: most
    // pairs at a busy field. Heathrow's peak hour (2026-10-09, 60 an hour
    // and 80 parked) spent most of its time in this function, every pair
    // of movers sampled against every segment each 0.1 s substep.
    let bbox = |w: &[(f64, f64)]| {
        w.iter().fold(
            (
                f64::INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
            ),
            |(x0, y0, x1, y1), &(x, y)| (x0.min(x), y0.min(y), x1.max(x), y1.max(y)),
        )
    };
    let (a, b) = (bbox(mine), bbox(other));
    if a.0 > b.2 + need_m || b.0 > a.2 + need_m || a.1 > b.3 + need_m || b.1 > a.3 + need_m {
        return None;
    }
    let need2 = need_m * need_m;
    let opposed = MEETING_DEG.to_radians().cos();
    let segs: Vec<Seg> = other
        .windows(2)
        .filter_map(|w| {
            let (dx, dy) = (w[1].0 - w[0].0, w[1].1 - w[0].1);
            let len = dx.hypot(dy);
            (len > 1e-6).then_some((w[0], (dx / len, dy / len), len))
        })
        .collect();
    // Every sample: metres along, near the other's line at all, and near
    // it running the other way.
    let mut samples: Vec<(f64, bool, bool)> = Vec::new();
    let mut along = 0.0;
    for w in mine.windows(2) {
        let (dx, dy) = (w[1].0 - w[0].0, w[1].1 - w[0].1);
        let len = dx.hypot(dy);
        if len < 1e-6 {
            continue;
        }
        let dir = (dx / len, dy / len);
        let n = (len / SAMPLE_M).ceil().max(1.0) as usize;
        for k in 0..=n {
            let f = k as f64 / n as f64;
            let p = (w[0].0 + dx * f, w[0].1 + dy * f);
            let (mut near, mut against) = (false, false);
            for &(a, u, l) in &segs {
                let t = ((p.0 - a.0) * u.0 + (p.1 - a.1) * u.1).clamp(0.0, l);
                let (fx, fy) = (a.0 + u.0 * t, a.1 + u.1 * t);
                let (ex, ey) = (p.0 - fx, p.1 - fy);
                if ex * ex + ey * ey < need2 {
                    near = true;
                    against |= dir.0 * u.0 + dir.1 * u.1 <= opposed;
                }
            }
            samples.push((along + len * f, near, against));
            if against {
                // Back to where this approach first came near the line.
                let mut entry = along + len * f;
                for &(s, near, _) in samples.iter().rev() {
                    if !near {
                        break;
                    }
                    entry = s;
                }
                return Some(entry);
            }
        }
        along += len;
    }
    None
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
    /// On a stand: parked there, or a departure not yet off the stand it
    /// started on. It stops only a mover whose line runs over it
    /// ([`PARKED_CORRIDOR_M`]).
    on_stand: bool,
}

/// What a mover stops for this substep.
#[derive(Clone, Copy)]
struct Stop {
    obstacle: usize,
    free_m: f64,
    /// It will not come on: parked, ours, or standing nose to nose.
    immovable: bool,
    /// Held short of a meeting stretch for a mover that will come through.
    junction: bool,
}

/// Space every taxiing aircraft in `fleet` at `field` on the ground
/// traffic ahead of it (the fleet's and our own `own`), for one substep of
/// `dt`. Sets each aircraft's [`GroundSpacing`]; a mover held [`STUCK_S`]
/// behind one that will not move is planned round it, or has it towed, and
/// says so in `events`.
pub fn apply(
    fleet: &mut [TrafficAircraft],
    field: &FieldModel,
    own: Option<&OwnTarget>,
    dt: f64,
    events: &mut Vec<Event>,
) {
    for ac in fleet.iter_mut() {
        if ac.kin.on_ground && ac.kin.gs_kts < 0.5 {
            ac.spacing.still_s += dt;
        } else {
            ac.spacing.still_s = 0.0;
        }
    }
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
            on_stand: a.phase == Phase::Parked
                || (a.phase == Phase::TaxiOut
                    && a.stand
                        .as_deref()
                        .and_then(|n| field.stand(n))
                        .is_some_and(|s| {
                            dist_m(a.kin.pos(), (s.lat, s.lon)) < crate::field::STAND_TAKEN_M
                        })),
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
            on_stand: false,
        });
    }
    let ways: Vec<Option<Vec<LatLon>>> =
        fleet.iter().map(|a| route_ahead(a, LOOKAHEAD_M)).collect();
    let n = fleet.len();
    let mut stops: Vec<Option<Stop>> = vec![None; n];
    let mut on_strip = vec![false; n];
    let keep = |stops: &mut Vec<Option<Stop>>, i: usize, s: Stop| {
        if stops[i].is_none_or(|o| s.free_m < o.free_m) {
            stops[i] = Some(s);
        }
    };
    // Where each mover's way ahead is on a runway, worked out when needed.
    let mut spans: Vec<Option<Option<(f64, f64)>>> = vec![None; n];
    // The nearest obstacle ahead of each mover.
    for (i, way) in ways.iter().enumerate() {
        let Some(way) = way else { continue };
        let ac = &fleet[i];
        on_strip[i] = field.on_runway(ac.kin.pos());
        for (k, ob) in obstacles.iter().enumerate() {
            if ob.idx == Some(i) {
                continue;
            }
            let Some((along, off, brg)) = project(way, ob.pos) else {
                continue;
            };
            // Started up INSIDE one parked on its own stand (two put on one
            // stand: CYJP and EFMI in the hour sweep, 2026-10-08), it
            // can only drive out of it: waiting held it 90 s every time.
            if ob.parked && ob.idx.is_some() && dist_m(ac.kin.pos(), ob.pos) < PARKED_CORRIDOR_M {
                continue;
            }
            // The wider of the two sets the corridor: an A321 took its place
            // in LFPG's departure queue 27 m abeam an A359, outside its own
            // half span and inside the A359's, and the A359 towed four
            // narrowbodies off its way, one after another (the traffic hour
            // sweep, 2026-10-10, the first widebodies).
            let corridor = if ob.on_stand {
                PARKED_CORRIDOR_M
            } else {
                corridor_m(ac.perf) + (ob.perf.span_m - ac.perf.span_m).max(0.0) / 2.0
            };
            if off > corridor {
                continue;
            }
            // Never held back by the taxi spacing ON a runway (it would hold
            // the runway: BIEG 03's gridlock), but never driven through
            // either: closed up to the one ahead and no further.
            let gap = if on_strip[i] {
                contact_m(ac.perf, ob.perf)
            } else {
                spacing_m(ac.perf, ob.perf)
            };
            // One standing still and facing it will not come on: a departure
            // held at its hold met by an arrival taxiing in past it (UUYY,
            // the hour sweep, 2026-10-08), or one met head on too late to
            // hold short. A queue (the same way) is waited in for good.
            let still = ob.idx.is_some_and(|j| fleet[j].kin.gs_kts < 0.5);
            let facing = ob
                .hdg
                .is_some_and(|h| signed_diff_deg(h, brg).abs() > FACING_DEG);
            // Coming on at it, the other closes too: each plans its stop on
            // half the room, so the two stop by the spacing, not inside it.
            let mut free = if facing && !still && !ob.parked {
                (along - gap) / 2.0
            } else {
                along - gap
            };
            // Never onto a runway it cannot get off again: with traffic
            // beyond a crossing, stopped there it stood ON the strip and
            // the tower said "expedite crossing" (PANC and EFMA, the hour
            // sweep, 2026-10-09). It waits short of it instead.
            if !on_strip[i] {
                let span = *spans[i].get_or_insert_with(|| runway_span(field, way));
                if let Some((s_in, s_out)) = span {
                    let clear_of_it = s_out + ac.perf.length_m / 2.0;
                    if along > s_in && free < clear_of_it {
                        free = free.min(s_in - ac.perf.length_m / 2.0 - CONTACT_MARGIN_M);
                    }
                }
            }
            keep(
                &mut stops,
                i,
                Stop {
                    obstacle: k,
                    free_m: free,
                    immovable: ob.parked || (still && facing),
                    junction: false,
                },
            );
        }
    }
    // NOSE TO TAIL WHATEVER THE ROUTE SAYS: one standing right in front of
    // a mover's nose, within its own length and the spacing's contact,
    // stops it there, seen on the route or not. LFPG and EDDF (the traffic
    // hour sweep, 2026-10-10): an A359 and a 747-8 on a long leg of their
    // way to the runway closed up on the departure queue unseen and drove
    // into the one at its end at 5.6 kt.
    for (i, ac) in fleet.iter().enumerate() {
        if !taxi_phase(ac.phase) || !ac.kin.on_ground || on_strip[i] {
            continue;
        }
        for (k, ob) in obstacles.iter().enumerate() {
            if ob.idx == Some(i) || ob.on_stand {
                continue;
            }
            let d = dist_m(ac.kin.pos(), ob.pos);
            let reach = contact_m(ac.perf, ob.perf) + NOSE_REACH_M;
            if d >= reach {
                continue;
            }
            let rel = signed_diff_deg(bearing(ac.kin.pos(), ob.pos), ac.kin.hdg_deg).to_radians();
            let (along, off) = (d * rel.cos(), (d * rel.sin()).abs());
            if along <= 0.0 || off > (ac.perf.span_m + ob.perf.span_m) / 4.0 {
                continue;
            }
            keep(
                &mut stops,
                i,
                Stop {
                    obstacle: k,
                    free_m: along - contact_m(ac.perf, ob.perf),
                    immovable: ob.parked,
                    junction: false,
                },
            );
        }
    }
    // Two movers about to meet head on along one stretch of taxiway: the
    // one giving way holds short of it at its junction until the other is
    // through. Inside the stretch already (or unable to stop short of it),
    // a mover has the right of way; both outside, a line-up or a vacate
    // first, then the nearer to it, then the lower id. Both inside: met too
    // late, both stop by the spacing above, and the guard below sorts it.
    let origin = fleet.first().map(|a| a.kin.pos()).unwrap_or((0.0, 0.0));
    let long: Vec<Option<Vec<LatLon>>> = fleet
        .iter()
        .map(|a| route_ahead(a, MEETING_LOOKAHEAD_M))
        .collect();
    let mine: Vec<Option<Vec<(f64, f64)>>> = long
        .iter()
        .map(|w| w.as_ref().map(|w| metres(origin, w)))
        .collect();
    // As the other party, from its TAIL: the one giving way waits until the
    // whole aircraft is past its junction.
    let theirs: Vec<Option<Vec<(f64, f64)>>> = long
        .iter()
        .zip(fleet.iter())
        .map(|(w, a)| {
            w.as_ref().map(|w| {
                let tail = project_m(a.kin.pos(), a.kin.hdg_deg + 180.0, a.perf.length_m / 2.0);
                metres(origin, &[&[tail][..], w].concat())
            })
        })
        .collect();
    for (i, way) in ways.iter().enumerate() {
        if let (None, Some(w)) = (spans[i], way) {
            spans[i] = Some(runway_span(field, w));
        }
    }
    let mut giving: Vec<Vec<(u32, Option<HoldAt>)>> = vec![Vec::new(); n];
    for i in 0..n {
        for j in i + 1..n {
            let (Some(mi), Some(mj), Some(ti), Some(tj)) =
                (&mine[i], &mine[j], &theirs[i], &theirs[j])
            else {
                continue;
            };
            if dist_m(fleet[i].kin.pos(), fleet[j].kin.pos()) > 2.0 * MEETING_LOOKAHEAD_M {
                continue;
            }
            let need = (fleet[i].perf.span_m + fleet[j].perf.span_m) / 2.0 + LATERAL_MARGIN_M;
            let (Some(ei), Some(ej)) =
                (meeting_entry_m(mi, tj, need), meeting_entry_m(mj, ti, need))
            else {
                continue;
            };
            // Held back from the stretch so the other passes the junction a
            // whole spacing from it, never just a wingtip. Once giving way,
            // at the point first set ([`GroundSpacing::giving_way`]).
            let hold = |k: usize, e: f64, other: usize| {
                let back = spacing_m(fleet[k].perf, fleet[other].perf) - need;
                let short = e - back.max(fleet[k].perf.length_m / 2.0 + HOLD_MARGIN_M);
                let set = fleet[k]
                    .spacing
                    .giving_way
                    .iter()
                    .find(|(id, _)| *id == fleet[other].id)
                    .and_then(|(_, at)| *at)
                    .filter(|a| fleet[k].route.path.first() == Some(&a.from));
                match set {
                    Some(at) => short.max(at.s_m - route_s(&fleet[k])),
                    None => short,
                }
            };
            // Standing still and not in the other's way (a departure just
            // started up on its stand beside the lane an arrival is taxiing
            // in along, XNOC 2026-10-09), it can simply stay where it is.
            let in_way = |k: usize, other: usize| {
                stops[other].is_some_and(|s| obstacles[s.obstacle].idx == Some(k))
            };
            // Never held ON a runway, nor short of a stretch beyond one: it
            // would stand on the strip (EFMA, the hour sweep, 2026-10-09).
            let off_runways = |k: usize, e: f64, other: usize| {
                !on_strip[k]
                    && spans[k].flatten().is_none_or(|(s_in, _)| {
                        hold(k, e, other) < s_in - fleet[k].perf.length_m / 2.0
                    })
            };
            let can_hold = |k: usize, e: f64, other: usize| {
                let v = fleet[k].kin.gs_kts * crate::kin::KT_TO_MS;
                let was = fleet[k]
                    .spacing
                    .giving_way
                    .iter()
                    .any(|(id, _)| *id == fleet[other].id);
                // Standing where the other has stopped for it, it gives way
                // to nobody: it is in the way, for the guard below (held to
                // its hold point it would stand there for good, the other
                // waiting on it: USRK, the hour sweep, 2026-10-09).
                let blocking =
                    fleet[k].kin.gs_kts < 0.5 && fleet[other].kin.gs_kts < 0.5 && in_way(k, other);
                if !off_runways(k, e, other) || blocking {
                    false
                } else if fleet[k].kin.gs_kts < 0.5 && !in_way(k, other) {
                    true
                } else if was {
                    // Stopped where it gave way, it stays: released there
                    // because the other's route came into full view (its
                    // hold point is then measured afresh, from inside the
                    // stretch), it drove at the other still turning off the
                    // runway onto its stretch (XTST's ENAT crossing, with the
                    // 737 taking its corners on its wheels, 2026-10-10).
                    hold(k, e, other) > -1.0 || fleet[k].kin.gs_kts < 0.5
                } else {
                    hold(k, e, other) >= v * v / (2.0 * SPACING_DECEL_MS2)
                }
            };
            let (hi, hj) = (can_hold(i, ei, j), can_hold(j, ej, i));
            let giver = match (hi, hj) {
                (true, false) => Some(i),
                (false, true) => Some(j),
                // Neither can stop short comfortably: one crossing a runway
                // still never gives way, the other brakes for it as best it
                // can. EHAM 22 (the traffic hour sweep, 2026-10-10): a 737
                // taxiing at 18 kt to its hold short met an E190 coming off
                // the crossing there, both stopped nose to nose and the E190
                // stood on the runway.
                (false, false) if on_strip[i] != on_strip[j] => {
                    Some(if on_strip[i] { j } else { i })
                }
                (false, false) => None,
                (true, true) => {
                    let gives = |k: usize, to: usize| {
                        fleet[k]
                            .spacing
                            .giving_way
                            .iter()
                            .any(|(id, _)| *id == fleet[to].id)
                    };
                    let (was_i, was_j) = (gives(i, j), gives(j, i));
                    if was_i != was_j {
                        Some(if was_i { i } else { j })
                    } else {
                        let i_first = match rank(&fleet[i]).cmp(&rank(&fleet[j])) {
                            std::cmp::Ordering::Less => true,
                            std::cmp::Ordering::Greater => false,
                            std::cmp::Ordering::Equal => {
                                ei < ej || (ei == ej && fleet[i].id < fleet[j].id)
                            }
                        };
                        Some(if i_first { j } else { i })
                    }
                }
            };
            if let Some(g) = giver {
                let (other, e) = if g == i { (j, ei) } else { (i, ej) };
                let ob = obstacles
                    .iter()
                    .position(|o| o.idx == Some(other))
                    .expect("a mover is an obstacle");
                // A departure still on its stand gives way ON it: held at a
                // junction down the lane, it met the arrival in a dead-end
                // apron lane with no way past (XNOC, 2026-10-09: the vacate
                // it gave way to turned in up its lane).
                let on_stand = obstacles.iter().any(|o| o.idx == Some(g) && o.on_stand);
                let free = if on_stand {
                    0.0
                } else {
                    hold(g, e, other).max(0.0)
                };
                // Where it holds, as metres along its own route, while the
                // other's way runs on past what is seen of it: the hold then
                // never runs toward it (it moves on only when the stretch
                // does, away).
                let unseen = {
                    let r = &fleet[other].route.path;
                    let whole: f64 = r.windows(2).map(|w| dist_m(w[0], w[1])).sum();
                    whole - route_s(&fleet[other]) > MEETING_LOOKAHEAD_M
                };
                let at = fleet[g]
                    .route
                    .path
                    .first()
                    .filter(|_| free > 0.0 && unseen)
                    .map(|&from| HoldAt {
                        s_m: route_s(&fleet[g]) + free,
                        from,
                    });
                giving[g].push((fleet[other].id, at));
                keep(
                    &mut stops,
                    g,
                    Stop {
                        obstacle: ob,
                        free_m: free,
                        immovable: false,
                        junction: true,
                    },
                );
            }
        }
    }
    // The stops, and the guard on the ones that will not come on.
    // (who, early): a vacate planned round at once, on the move.
    let mut due: Vec<(usize, bool)> = Vec::new();
    let standing: Vec<bool> = fleet.iter().map(|a| a.kin.gs_kts < 0.5).collect();
    for (i, ac) in fleet.iter_mut().enumerate() {
        ac.spacing.giving_way = std::mem::take(&mut giving[i]);
        let Some(s) = stops[i].filter(|_| ways[i].is_some()) else {
            ac.spacing.free_m = None;
            ac.spacing.blocker = None;
            ac.spacing.held_s = 0.0;
            continue;
        };
        let ob = &obstacles[s.obstacle];
        if ac.spacing.blocker != Some(ob.who) {
            ac.spacing.held_s = 0.0;
        }
        ac.spacing.blocker = Some(ob.who);
        ac.spacing.free_m = Some(s.free_m);
        let stopped = ac.kin.gs_kts < 0.5 && s.free_m < 1.0;
        // Waiting on one that will never move for it: bounded. Off the
        // runway only behind one that will not come on (a queue moves up
        // in its own time); on it, behind anybody.
        // On the runway and braking for one standing off it in its way, the
        // guard acts before it stops there: ESNU 14 (the hour sweep,
        // 2026-10-10), a Cessna cleared across and down the runway braked
        // to a stop nose to nose with an ATR holding short at its way off,
        // and the tower called "expedite crossing" before the ATR was
        // towed five seconds on.
        let v = ac.kin.gs_kts * crate::kin::KT_TO_MS;
        let braking_on_strip = on_strip[i]
            && !s.junction
            && ob.idx.is_some_and(|j| standing[j] && !on_strip[j])
            && s.free_m < v * v / (2.0 * SPACING_DECEL_MS2) + STRIP_GUARD_M;
        let mut guard_due = false;
        if braking_on_strip && !stopped {
            ac.spacing.held_s += dt;
            guard_due = true;
        } else if stopped && !s.junction && (s.immovable || on_strip[i]) {
            ac.spacing.held_s += dt;
            let limit = if on_strip[i] { RUNWAY_STUCK_S } else { STUCK_S };
            guard_due = ac.spacing.held_s >= limit;
        } else if stopped && s.junction && ob.idx.is_some_and(|j| standing[j]) {
            // Giving way to one that is standing still itself (giving way in
            // its turn to a third, round a ring): EGLL (the traffic hour
            // sweep, 2026-10-10), a 789 and an A333 each held at a junction
            // for the other's stretch, and the arrivals behind them stood for
            // half an hour. Twice the guard's patience, then it acts.
            ac.spacing.held_s += dt;
            guard_due = ac.spacing.held_s >= 2.0 * STUCK_S;
        } else {
            ac.spacing.held_s = 0.0;
        }
        if guard_due {
            due.push((i, false));
        } else if ac.phase == Phase::Vacate
            && s.immovable
            && !s.junction
            && ac.spacing.tried != Some(ob.who)
        {
            // A vacate heading for one that will not move (a departure held
            // at the hold on its exit) takes another way off AT ONCE:
            // stopped there, it stood on the departure's way onto the
            // runway, nose to nose, until one was towed (ULAM and UUYY, the
            // hour sweep, 2026-10-09). Its vacate is its own, never the
            // tower's route.
            ac.spacing.tried = Some(ob.who);
            due.push((i, true));
        }
    }
    // The one with the least right of way acts first: two nose to nose,
    // only one of them turns away.
    due.sort_by_key(|&(i, _)| std::cmp::Reverse((rank(&fleet[i]), fleet[i].id)));
    for (i, early) in due {
        let Some(s) = stops[i] else { continue };
        if fleet[i].phase == Phase::Gone || (!early && fleet[i].spacing.held_s == 0.0) {
            continue;
        }
        let ob = &obstacles[s.obstacle];
        if ob.idx.is_some_and(|j| fleet[j].phase == Phase::Gone) {
            continue;
        }
        let around = match ob.who {
            Blocker::Traffic(id) => Some(id),
            Blocker::Own => None,
        };
        // The new way keeps the whole spacing from it, or at least as far
        // as it stands now, and never less than wingtip clear of it (a
        // vacate planned round one on the move passed it 17 m off, ULMM).
        let wingtips = corridor_m(fleet[i].perf) + ob.perf.span_m / 2.0;
        let clear = spacing_m(fleet[i].perf, ob.perf)
            .min(dist_m(fleet[i].kin.pos(), ob.pos) - 1.0)
            .max(wingtips);
        fleet[i].spacing.held_s = 0.0;
        if fleet[i].reroute_around(field, ob.pos, clear) {
            let sp = &mut fleet[i].spacing;
            sp.free_m = None;
            sp.blocker = None;
            sp.no_way = None;
            // Its partner nose to nose waits afresh: this one is turning away.
            if let Some(j) = ob.idx {
                fleet[j].spacing.held_s = 0.0;
            }
            events.push(Event::SpacingRerouted {
                id: fleet[i].id,
                around,
            });
            continue;
        }
        if early {
            // No other way off: on, and the guard when it stops.
            continue;
        }
        // No way round: a traffic aircraft that will not move is towed
        // clear (a real tug would); ours never, and one that may yet move
        // (it has only just stopped, or can still find its own way round)
        // is waited for.
        // Nor one that stands waiting on this one: two each stopped for the
        // other never move (USRK, the hour sweep, 2026-10-09: an arrival
        // for the stand a departure had just left, nose to its side).
        let me = Blocker::Traffic(fleet[i].id);
        let towable = ob.idx.filter(|&j| {
            let b = &fleet[j];
            b.phase == Phase::Parked
                || (b.spacing.still_s >= STUCK_S
                    && (ways[j].is_none()
                        || b.spacing.no_way.is_some()
                        || b.spacing.blocker == Some(me)))
        });
        if let Some(j) = towable {
            let from = fleet[j].phase;
            let removed = fleet[j].id;
            fleet[j].phase = Phase::Gone;
            events.push(Event::Phase {
                id: removed,
                from,
                to: Phase::Gone,
            });
            events.push(Event::Despawn {
                id: removed,
                reason: Despawn::Towed,
            });
            events.push(Event::SpacingCleared {
                id: fleet[i].id,
                removed,
            });
            fleet[i].spacing.no_way = None;
            continue;
        }
        if fleet[i].spacing.no_way != Some(ob.who) {
            fleet[i].spacing.no_way = Some(ob.who);
            events.push(Event::SpacingHeld {
                id: fleet[i].id,
                behind: around,
            });
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
        assert!(contact_m(&JET_MED, &JET_MED) < jets);
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

    /// The meeting stretch: a route that joins another from a side branch
    /// and runs back down it meets it head on where it joins, one straight
    /// across it where it crosses; one behind the other on the line (a
    /// queue) and a parallel line never do.
    #[test]
    fn a_meeting_is_where_two_routes_run_the_other_way_or_across() {
        // The other comes west along y = 0 from x = 300 to x = -300.
        let other = vec![(300.0, 0.0), (-300.0, 0.0)];
        // Mine comes up a branch from (0, -100) to the line, then east.
        let mine = vec![(0.0, -100.0), (0.0, 0.0), (300.0, 0.0)];
        // Held short where the branch comes within 25 m of the line, not
        // at the corner 100 m up it.
        let e = meeting_entry_m(&mine, &other, 25.0).expect("met head on");
        assert!((72.0..=80.0).contains(&e), "joined at {e:.0} m");
        // Joining it from the branch the same way is a meeting too (it
        // turns in across the other's line)...
        let join = vec![(0.0, -100.0), (0.0, 0.0), (-300.0, 0.0)];
        assert!(meeting_entry_m(&join, &other, 25.0).is_some());
        // ...but one behind the other on the line is a queue.
        let queue = vec![(350.0, 0.0), (-300.0, 0.0)];
        assert_eq!(meeting_entry_m(&queue, &other, 25.0), None);
        // Straight across it: where it comes within 25 m of the line.
        let across = vec![(0.0, -100.0), (0.0, 100.0)];
        let e = meeting_entry_m(&across, &other, 25.0).expect("met across");
        assert!((72.0..=80.0).contains(&e), "crossed at {e:.0} m");
        // The parallel the other way, 45 m off.
        let parallel = vec![(-300.0, 45.0), (300.0, 45.0)];
        assert_eq!(meeting_entry_m(&parallel, &other, 25.0), None);
    }

    /// One standing right in front of a mover's nose stops it, whatever its
    /// route says: an A359 and a 747-8 whose way ahead did not show the
    /// queue at the hold drove into it at 5.6 kt (LFPG, EDDF, 2026-10-10).
    /// One off to the side, or behind, does not.
    #[test]
    fn one_right_in_front_of_the_nose_stops_a_mover() {
        use crate::aircraft::{Route, TrafficAircraft};
        use crate::kin::{shift, Kin};
        use crate::phase::{Phase, Plan};
        let field = crate::field::fixtures::xtst();
        // Well clear of the field's runways.
        let o = shift(
            (field.net.stands[0].lat, field.net.stands[0].lon),
            5000.0,
            5000.0,
        );
        let mover = |at: LatLon| {
            let mut kin = Kin::parked(at, 0.0, 90.0);
            kin.gs_kts = 10.0;
            let mut a =
                TrafficAircraft::new(1, "A", "A", "B77W", Plan::Parked, kin, Phase::TaxiOut, 1);
            // The way the follower has ahead turns away north at once.
            a.route = Route::taxi(vec![at, shift(at, 0.0, 300.0)]);
            a
        };
        let standing = |at: LatLon| {
            let kin = Kin::parked(at, 0.0, 90.0);
            TrafficAircraft::new(2, "B", "B", "A320", Plan::Parked, kin, Phase::TaxiOut, 2)
        };
        for (east, north, stops) in [(45.0, 0.0, true), (45.0, -60.0, false), (-45.0, 0.0, false)] {
            let mut fleet = vec![mover(o), standing(shift(o, east, north))];
            let mut events = Vec::new();
            apply(&mut fleet, &field, None, 0.1, &mut events);
            let free = fleet[0].spacing.free_m;
            assert_eq!(
                free.is_some_and(|f| f < 0.0),
                stops,
                "{east} east {north} north: {free:?}"
            );
        }
    }
}
