//! THE MID-FLOW START (2026-10-13): at a field with a route table the
//! world is already moving when the traffic starts. Each rotation of the
//! day ([`crate::rotation`]) is put where it would be NOW: an arrival due
//! within its approach time already on its instrument approach, spaced
//! behind the one ahead; one landed in the last minutes taxiing in; the
//! aircraft turning round on their stands; a departure due off the blocks
//! later boarding there; one gone in the last minutes pushing, taxiing,
//! holding short or climbing out. Everything later spawns at its time as
//! before. The whole start is pure ([`plan`]): the station learns of the
//! placed flights through their own first calls (an arrival inside the
//! tower's distance calls the tower established, a holding departure calls
//! ready, a taxi-in is seen on the ground), so the strips come to the state
//! the picture shows through the one door every flight uses.
//!
//! Before it, the fleet started with every flight parked or out of sight
//! (`park_at_start`): the first arrival was ten minutes from the field and
//! the first movement on the ground a minute in. Fields with no table keep
//! that start.
//!
//! The runway rules of the start: nothing ON the runway (no line-up, no
//! roll, no flare); the nearest arrival [`MIN_FINAL_NM`] out, each behind
//! the one ahead by its wake spacing and a runway's time
//! ([`FINAL_GAP_NM`]); one departure holding short and one climbing out at
//! most; no two ground movers whose ways cross ([`WAY_CLEAR_M`]), the later
//! one put back on its stand.

use crate::aircraft::{glide_ft, IfrRadio, Route, TrafficAircraft, TCH_FT};
use crate::arrival::{scheduled_kts, scheduled_time_s, wake_gap_nm, WAKE_BUFFER_NM};
use crate::board::{BoardFlight, Dir};
use crate::field::FieldModel;
use crate::kin::{bearing, cross_track_m, dist_m, Kin, LatLon};
use crate::perf::class_for_type;
use crate::phase::{Leg, Phase, Plan};
use crate::rng::XorShift64;
use crate::rotation::{self, Date, Rotation};
use crate::routes::{airline_name, FieldRoutes};
use crate::schedule::{
    default_runway, entry_clear_of_own, field_seed, free_stands, runway_needed_m, Density,
    FlightInfo, Movement, MovementKind, Refusal, SpawnCtx, Start, OWN_CLEAR_NM, STANDS_KEPT_FREE,
};

const NM: f64 = 1852.0;

/// An arrival this soon or sooner is placed on its approach; later ones
/// spawn at their entry as before (the approach from a 25 nm entry takes
/// about ten minutes at the radar unit's speeds).
pub const APPROACH_LEAD_NM: f64 = 24.0;

/// The nearest an arrival is placed to its threshold, nm: room for its
/// "established" call and the tower's landing clearance well before the
/// 300 ft a flight with none goes around at.
pub const MIN_FINAL_NM: f64 = 5.0;

/// Two arrivals placed on one approach are at least this far apart, nm,
/// besides their wake spacing ([`wake_gap_nm`]): a runway lands one in
/// about 100 s, 4 nm at a final's speeds.
pub const FINAL_GAP_NM: f64 = 4.0;

/// A landing this recent is taxiing in, seconds; earlier it is on its stand.
pub const TAXI_IN_S: f64 = 8.0 * 60.0;

/// A departure off the blocks this recently is pushing (released now); then
/// taxiing until [`TAXI_OUT_S`] at most, holding short for
/// `HOLD_S - TAXI_OUT_S`, climbing out for `CLIMB_S - HOLD_S`; earlier it
/// has gone ([`out_times`]: the taxi is its own way's).
pub const PUSH_S: f64 = 2.0 * 60.0;
pub const TAXI_OUT_S: f64 = 9.0 * 60.0;
pub const HOLD_S: f64 = 11.0 * 60.0;
pub const CLIMB_S: f64 = 14.0 * 60.0;

/// Two ground movers placed at the start keep this far off each other's
/// way ahead, metres (a wingspan each side and a margin).
pub const WAY_CLEAR_M: f64 = 60.0;

/// A stand handed out at the start is this far from any stand already
/// held, where the apron has one, metres.
pub const BESIDE_M: f64 = 80.0;

/// A placed aircraft never starts this close to our own on the ground,
/// metres.
pub const OWN_GROUND_CLEAR_M: f64 = 150.0;

/// What the start produced: the movements (placed ones at 0 s, the rest at
/// their times after now) and every flight of the window for the board.
#[derive(Debug, Clone, Default)]
pub struct Plan0 {
    pub movements: Vec<Movement>,
    pub board: Vec<BoardFlight>,
}

/// A rotation leg as the board's line.
fn info(r: &Rotation, arr: bool) -> FlightInfo {
    let l = if arr { &r.arr } else { &r.dep };
    let (airline, iata) = airline_name(r.carrier.code);
    let n = &l.callsign[3..];
    FlightInfo {
        number: if iata.is_empty() {
            l.callsign.clone()
        } else {
            format!("{iata} {n}")
        },
        airline: airline.to_string(),
        other: l.other.to_string(),
        city: l.city.to_string(),
        sched_s: Some(l.sched_s),
        est_s: Some(l.est_s),
        inbound: (!arr).then(|| r.arr.callsign.clone()),
        stand: None,
    }
}

/// Seconds to fly the last `d_m` metres on the radar unit's speeds, and its
/// inverse: the distance out whose time to the threshold is `t_s`.
fn dist_for_time(icao_type: &str, t_s: f64) -> f64 {
    let p = class_for_type(icao_type).0;
    let (mut lo, mut hi) = (0.0, 60.0 * NM);
    for _ in 0..40 {
        let mid = 0.5 * (lo + hi);
        if scheduled_time_s(p, mid) < t_s {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

/// A ground mover placed at the start: where it is and the way ahead.
struct Mover {
    pos: LatLon,
    ahead: Vec<LatLon>,
}

impl Mover {
    fn clear_of(&self, o: &Mover) -> bool {
        let near =
            |p: LatLon, way: &[LatLon]| way.len() >= 2 && cross_track_m(way, p) < WAY_CLEAR_M;
        dist_m(self.pos, o.pos) >= 2.0 * WAY_CLEAR_M
            && !near(self.pos, &o.ahead)
            && !near(o.pos, &self.ahead)
    }
}

/// The point `frac` (0 to 1) of the way along `path`, the segment it is on,
/// and the way from there to the end.
fn along(path: &[LatLon], frac: f64) -> Option<(LatLon, Vec<LatLon>)> {
    let total: f64 = path.windows(2).map(|w| dist_m(w[0], w[1])).sum();
    if path.len() < 2 || total <= 0.0 {
        return None;
    }
    let mut left = total * frac.clamp(0.0, 1.0);
    for (i, w) in path.windows(2).enumerate() {
        let d = dist_m(w[0], w[1]);
        if left <= d || i == path.len() - 2 {
            let f = if d > 0.0 { (left / d).min(1.0) } else { 0.0 };
            let p = (
                w[0].0 + (w[1].0 - w[0].0) * f,
                w[0].1 + (w[1].1 - w[0].1) * f,
            );
            let mut ahead = vec![p];
            ahead.extend(
                path[i + 1..]
                    .iter()
                    .copied()
                    .skip_while(|q| dist_m(*q, p) < 1.0),
            );
            return Some((p, ahead));
        }
        left -= d;
    }
    None
}

/// The way an arrival landing on `end` takes to `stand`: off the runway at
/// the exit its rollout makes, then the taxi in; from the exit's end (clear
/// of the runway) on.
fn taxi_in_way(
    field: &FieldModel,
    end: &crate::field::RunwayEnd,
    icao_type: &str,
    stand: &nav::Stand,
) -> Option<Vec<LatLon>> {
    let roll = (end.length_m * 0.55).min(1600.0);
    let v = field.vacate(end.at(roll, 0.0), end.course_deg, end, icao_type, &[])?;
    let off = *v.path.last()?;
    let plan = field.taxi_to_stand(off, stand)?;
    let mut way = vec![off];
    way.extend(plan.path.into_iter().skip_while(|p| dist_m(*p, off) < 1.0));
    (way.len() >= 2).then_some(way)
}

/// A departure's way from `stand` to the holding point of `end`, and its
/// drawn lead-on.
fn taxi_out_way(
    field: &FieldModel,
    end: &crate::field::RunwayEnd,
    stand: &nav::Stand,
) -> Option<(Vec<LatLon>, Vec<LatLon>)> {
    let from = (stand.lat, stand.lon);
    let plan = field.taxi_to_runway(from, end)?;
    let mut way = vec![from];
    way.extend(plan.path.into_iter().skip_while(|p| dist_m(*p, from) < 1.0));
    (way.len() >= 2).then_some((way, plan.lead_on))
}

/// A departure's moments after its off-block, seconds: the end of its taxi
/// (the push, then `way` at its pace; [`TAXI_OUT_S`] at most, and with no
/// way), its wheels up (a hold and a minute to line up and roll), the end
/// of its climb out. One rule for the start's placing and the model's
/// wheels up: LXGB (2026-10-13) flew a departure from its stand to 12 nm
/// in 6.5 min, and the flat 14 min of the old clocks placed it again,
/// holding short, at a rebuild 10 min after its off-block.
pub fn out_times(way: Option<&[LatLon]>, icao_type: &str) -> (f64, f64, f64) {
    let pace = class_for_type(icao_type).0.taxi_kts.min(12.0) * 0.514 * TAXI_PACE;
    let taxi_end = way.map_or(TAXI_OUT_S, |w| {
        let m: f64 = w.windows(2).map(|p| dist_m(p[0], p[1])).sum();
        (PUSH_S + m / pace.max(1.0)).min(TAXI_OUT_S)
    });
    let hold_end = taxi_end + (HOLD_S - TAXI_OUT_S);
    (taxi_end, hold_end + 60.0, hold_end + (CLIMB_S - HOLD_S))
}

/// The share of its taxi speed a taxi averages over its way (the turns, the
/// crossings).
const TAXI_PACE: f64 = 0.7;

/// The movement of one leg, before its start is decided.
#[allow(clippy::too_many_arguments)]
fn movement(
    r: &Rotation,
    arr: bool,
    t_s: f64,
    runway: &str,
    field: &FieldModel,
    ctx: &SpawnCtx,
    rng: &mut XorShift64,
) -> Movement {
    let l = if arr { &r.arr } else { &r.dep };
    let end = field.end(runway);
    // The other end where the timetable places it (2026-10-13, the whole
    // flight): an airliner arrives from it and a departure leaves toward
    // it, so the model's great circle meets the simulation's own way in
    // and out. Unplaced, as before (one draw either way: the rest of the
    // start draws the same).
    let other = crate::places::position(l.other);
    let instrument = crate::arrival::is_instrument(class_for_type(r.icao_type).0);
    let entry = arr
        .then(|| {
            end.as_ref().and_then(|e| match other.filter(|_| instrument) {
                Some(o) => crate::arrival::entry_toward(e, o, rng.next_f64(), ctx.own),
                None => entry_clear_of_own(e, ctx.own, rng),
            })
        })
        .flatten();
    let pick = |v: &[String], rng: &mut XorShift64| {
        (!v.is_empty()).then(|| v[rng.below(v.len() as u64) as usize].clone())
    };
    let sid = if arr {
        None
    } else {
        pick(&field.procs.sids, rng)
    };
    let star = if arr {
        pick(&field.procs.stars, rng)
    } else {
        None
    };
    Movement {
        t_s: t_s.max(0.0),
        callsign: l.callsign.clone(),
        spoken: l.spoken.clone(),
        icao_type: r.icao_type.to_string(),
        kind: if arr {
            MovementKind::Arrival
        } else {
            MovementKind::Departure
        },
        runway: runway.to_string(),
        stand: None,
        entry,
        straight_in: field.procs.approach_for(runway).is_some(),
        laps: 0,
        sid,
        star,
        // Toward the destination where the timetable places it, else a
        // seeded way out.
        exit_bearing: {
            let seeded = rng.range_f64(0.0, 360.0);
            match (other, field.reference()) {
                (Some(o), Some(here)) if !arr => crate::track::gc_bearing(here, o),
                _ => seeded,
            }
        },
        seed: rng.next_u64() ^ r.seed,
        parked_from_start: false,
        flight: Some(info(r, arr)),
        start: None,
    }
}

/// The timetable's rotations round `now_s` (two hours back, `horizon_s`
/// on) that `field` flies, and the day's stand plan for them
/// ([`crate::stands::plan`]: the board's stands from the start; the
/// pilot's board at LEMG, 2026-10-13, had departures Boarding with no
/// stand). A type the runway (`runway_m`) or the stands cannot take is not
/// flown: the table is the real airline's, the scenery may be smaller.
#[allow(clippy::too_many_arguments)]
fn flown_window(
    field: &FieldModel,
    table: &FieldRoutes,
    density: Density,
    seed: u64,
    date: Date,
    now_s: f64,
    horizon_s: f64,
    runway_m: f64,
) -> (Vec<Rotation>, std::collections::HashMap<String, String>) {
    let fits = |t: &str| {
        runway_m >= runway_needed_m(t) && field.net.stands.iter().any(|s| field.takes(s, t))
    };
    let rots: Vec<Rotation> = rotation::window(
        table,
        date,
        seed,
        density,
        now_s - 2.0 * 3600.0,
        now_s + horizon_s,
    )
    .into_iter()
    .filter(|r| fits(r.icao_type))
    .collect();
    let stands = crate::stands::plan(field, &rots);
    (rots, stands)
}

/// The board of a field whose aircraft are NOT simulated (2026-10-13, the
/// Board tab's field selector): every flight of its timetable round
/// `now_s` local, as [`plan`] puts them on the board of the field it flies
/// at (the same rotations for the same seed and date, its longest runway,
/// the stand plan), for the statuses to come from the clock
/// ([`crate::board::Status::by_clock`]). Pure and cheap once the field is
/// loaded.
pub fn timetable_board(
    field: &FieldModel,
    table: &FieldRoutes,
    density: Density,
    seed: u64,
    date: Date,
    now_s: f64,
    horizon_s: f64,
) -> Vec<BoardFlight> {
    let Some(end) = default_runway(field).and_then(|r| field.end(&r)) else {
        return Vec::new();
    };
    let (rots, stands) = flown_window(
        field,
        table,
        density,
        seed,
        date,
        now_s,
        horizon_s,
        end.length_m,
    );
    let mut out = Vec::new();
    for r in &rots {
        for (dir, leg, arr) in [(Dir::Arrival, &r.arr, true), (Dir::Departure, &r.dep, false)] {
            let mut info = info(r, arr);
            info.stand = stands.get(&leg.callsign).cloned();
            out.push(BoardFlight {
                dir,
                callsign: leg.callsign.clone(),
                icao_type: r.icao_type.to_string(),
                info,
                own: false,
            });
        }
    }
    out
}

/// The fleet's start at `field` with route table `table`, at `now_s` local
/// seconds after `date`'s midnight, the next `horizon_s` of the timetable
/// (see the module's rules). `ctx.runway` is the tower's runway in use (the
/// field's default with none). Deterministic per field, seed and date.
pub fn plan(
    field: &FieldModel,
    table: &FieldRoutes,
    density: Density,
    seed: u64,
    date: Date,
    now_s: f64,
    horizon_s: f64,
    ctx: &SpawnCtx,
) -> Plan0 {
    let mut out = Plan0::default();
    let Some(runway) = ctx.runway.clone().or_else(|| default_runway(field)) else {
        return out;
    };
    let Some(end) = field.end(&runway) else {
        return out;
    };
    let mut rng = XorShift64::new(field_seed(&field.icao, seed) ^ 0x4D49_4446_4C4F_57);
    let (rots, stand_plan) = flown_window(
        field,
        table,
        density,
        seed,
        date,
        now_s,
        horizon_s,
        end.length_m,
    );
    let mut taken: Vec<LatLon> = ctx
        .own
        .into_iter()
        .chain(ctx.occupied.iter().copied())
        .collect();
    let airline_stands = field
        .net
        .stands
        .iter()
        .filter(|s| field.takes(s, "A320") || field.takes(s, crate::schedule::REGIONAL))
        .count();
    let take_stand = |t: &str,
                      want: Option<&String>,
                      taken: &mut Vec<LatLon>,
                      rng: &mut XorShift64|
     -> Option<String> {
        // Room kept for the arrivals still to come.
        let held = field
            .net
            .stands
            .iter()
            .filter(|s| crate::field::stand_taken(s, taken))
            .count();
        if airline_stands.saturating_sub(held) <= STANDS_KEPT_FREE {
            return None;
        }
        // Its planned stand where that is free, clear of the lanes and of
        // the stands already held as the picks below are.
        if let Some(s) = want.and_then(|n| field.stand(n)).filter(|s| {
            field.takes(s, t)
                && field.lane_clearance_m((s.lat, s.lon)) >= crate::schedule::parked_lane_clear_m(t)
                && taken.iter().all(|p| dist_m(*p, (s.lat, s.lon)) >= BESIDE_M)
        }) {
            taken.push((s.lat, s.lon));
            return Some(s.name.clone());
        }
        // An aircraft turning round stands there for up to two hours: a
        // stand clear of the lanes others taxi along first, as a parked
        // one's (the mid-flow harness, 2026-10-13: a taxi-out passed 18.6 m
        // from one turning round beside its way).
        let all = free_stands(field, t, taken);
        let clear: Vec<String> = all
            .iter()
            .filter(|n| {
                field.stand(n).is_some_and(|s| {
                    field.lane_clearance_m((s.lat, s.lon))
                        >= crate::schedule::parked_lane_clear_m(t)
                })
            })
            .cloned()
            .collect();
        let pool = if clear.is_empty() { all } else { clear };
        // ...and no nearer a stand somebody already stands on than a
        // departure's way off its own runs past (`JET_BESIDE_MOVER_M`'s 80
        // m): the same harness had a departure leave stand 95 past the
        // aircraft turning round on 97, 18.6 m apart.
        let apart: Vec<String> = pool
            .iter()
            .filter(|n| {
                field
                    .stand(n)
                    .is_some_and(|s| taken.iter().all(|p| dist_m(*p, (s.lat, s.lon)) >= BESIDE_M))
            })
            .cloned()
            .collect();
        let free = if apart.is_empty() { pool } else { apart };
        let name = free
            .get(rng.below(free.len().max(1) as u64) as usize)?
            .clone();
        let s = field.stand(&name)?;
        taken.push((s.lat, s.lon));
        Some(name)
    };
    let p_lead = |t: &str| scheduled_time_s(class_for_type(t).0, APPROACH_LEAD_NM * NM);
    let own_ground = ctx.own;
    let near_own = |p: LatLon| own_ground.is_some_and(|o| dist_m(o, p) < OWN_GROUND_CLEAR_M);
    let mut movers: Vec<Mover> = Vec::new();
    let mut held_short = false;
    let mut climbing = false;
    // Arrivals on their approach, placed after the ground: (to_thr_m, wake,
    // movement), spaced in order.
    let mut approaching: Vec<(f64, crate::perf::Wake, Movement)> = Vec::new();
    // Ground first, oldest first: the aircraft on their stands, then the
    // movers among them.
    for r in &rots {
        let (eta, etd) = (r.arr.est_s - now_s, r.dep.est_s - now_s);
        out.board.push(BoardFlight {
            dir: Dir::Arrival,
            callsign: r.arr.callsign.clone(),
            icao_type: r.icao_type.to_string(),
            info: info(r, true),
            own: false,
        });
        out.board.push(BoardFlight {
            dir: Dir::Departure,
            callsign: r.dep.callsign.clone(),
            icao_type: r.icao_type.to_string(),
            info: info(r, false),
            own: false,
        });
        if etd < -CLIMB_S {
            continue;
        }
        let lead = p_lead(r.icao_type);
        if eta > lead {
            // Both legs still to come: the arrival at its entry, the
            // departure off wherever it parks.
            let t = eta - lead;
            out.movements
                .push(movement(r, true, t, &runway, field, ctx, &mut rng));
            out.movements
                .push(movement(r, false, etd, &runway, field, ctx, &mut rng));
            continue;
        }
        if eta > 0.0 {
            let d = dist_for_time(r.icao_type, eta).max(MIN_FINAL_NM * NM);
            let mut m = movement(r, true, 0.0, &runway, field, ctx, &mut rng);
            m.start = Some(Start::Approach { to_thr_m: d });
            approaching.push((d, class_for_type(r.icao_type).1, m));
            out.movements
                .push(movement(r, false, etd, &runway, field, ctx, &mut rng));
            continue;
        }
        if eta > -TAXI_IN_S && etd > 0.0 {
            // Landed: taxiing in to a stand, where its way is clear.
            let placed = take_stand(
                r.icao_type,
                stand_plan.get(&r.arr.callsign),
                &mut taken,
                &mut rng,
            )
            .and_then(|stand| {
                let s = field.stand(&stand)?;
                let way = taxi_in_way(field, &end, r.icao_type, s)?;
                let (pos, ahead) = along(&way, (-eta / TAXI_IN_S).clamp(0.15, 0.9))?;
                let me = Mover { pos, ahead };
                (!field.on_runway(pos) && !near_own(pos) && movers.iter().all(|o| o.clear_of(&me)))
                    .then(|| {
                        movers.push(me);
                        stand.clone()
                    })
                    .or_else(|| {
                        // Not placed: the stand goes back.
                        taken.retain(|p| dist_m(*p, (s.lat, s.lon)) > 0.5);
                        None
                    })
            });
            if let Some(stand) = placed {
                let mut m = movement(r, true, 0.0, &runway, field, ctx, &mut rng);
                m.start = Some(Start::TaxiIn {
                    stand,
                    frac: (-eta / TAXI_IN_S).clamp(0.15, 0.9),
                });
                out.movements.push(m);
                out.movements
                    .push(movement(r, false, etd, &runway, field, ctx, &mut rng));
                continue;
            }
            // Else it is on a stand already, turning round (below).
        }
        // On its stand: the departure stands there from the start.
        let Some(stand) = take_stand(
            r.icao_type,
            stand_plan.get(&r.arr.callsign),
            &mut taken,
            &mut rng,
        ) else {
            // No stand: it appears on one at its time, if it is still to go.
            if etd > 0.0 {
                let mut m = movement(r, false, etd, &runway, field, ctx, &mut rng);
                if let Some(info) = m.flight.as_mut() {
                    info.inbound = None;
                }
                out.movements.push(m);
            }
            continue;
        };
        let mut m = movement(r, false, etd, &runway, field, ctx, &mut rng);
        m.stand = Some(stand.clone());
        if let Some(info) = m.flight.as_mut() {
            info.inbound = None;
        }
        if etd > 0.0 {
            m.parked_from_start = true;
            out.movements.push(m);
            continue;
        }
        // Off the blocks already: where is it now?
        let s = field.stand(&stand).cloned();
        let gone = -etd;
        let way = s.as_ref().and_then(|s| taxi_out_way(field, &end, s));
        let (taxi_end, up, climb_end) =
            out_times(way.as_ref().map(|(w, _)| w.as_slice()), r.icao_type);
        let hold_end = up - 60.0;
        let start = if gone <= PUSH_S {
            None
        } else if gone <= taxi_end {
            way.as_ref().and_then(|(w, _)| {
                let frac = ((gone - PUSH_S) / (taxi_end - PUSH_S).max(1.0)).clamp(0.2, 0.85);
                let (pos, ahead) = along(w, frac)?;
                let me = Mover { pos, ahead };
                (!field.on_runway(pos) && !near_own(pos) && movers.iter().all(|o| o.clear_of(&me)))
                    .then(|| {
                        movers.push(me);
                        Start::TaxiOut { frac }
                    })
            })
        } else if gone <= hold_end && !held_short {
            way.as_ref().and_then(|(w, _)| {
                let pos = *w.last()?;
                let me = Mover {
                    pos,
                    ahead: vec![pos],
                };
                (!near_own(pos) && movers.iter().all(|o| o.clear_of(&me))).then(|| {
                    movers.push(me);
                    held_short = true;
                    Start::HoldShort
                })
            })
        } else if gone <= climb_end && gone > hold_end && !climbing {
            climbing = true;
            let p = class_for_type(r.icao_type).0;
            Some(Start::Climb {
                past_end_m: ((gone - up).max(20.0)) * p.climb_kts * 0.514,
            })
        } else {
            // Its moment has gone and its way is taken: it pushes now.
            None
        };
        match start {
            Some(Start::Climb { .. }) | Some(Start::TaxiOut { .. }) | Some(Start::HoldShort) => {
                // Off its stand: the stand is free again.
                if let Some(s) = &s {
                    taken.retain(|p| dist_m(*p, (s.lat, s.lon)) > 0.5);
                }
                m.t_s = 0.0;
                m.parked_from_start = false;
                m.start = start;
                out.movements.push(m);
            }
            _ => {
                if gone > hold_end {
                    // Long gone and nowhere to put it: off the board's
                    // aircraft, its flight flown (the model's, below).
                    if let Some(s) = &s {
                        taken.retain(|p| dist_m(*p, (s.lat, s.lon)) > 0.5);
                    }
                    continue;
                }
                // Pushing now, staggered a little.
                m.t_s = 5.0
                    + 20.0
                        * (out
                            .movements
                            .iter()
                            .filter(|x| x.t_s < 120.0 && x.parked_from_start)
                            .count() as f64);
                m.parked_from_start = true;
                out.movements.push(m);
            }
        }
    }
    // The approach, nearest first, each behind the one ahead by its wake
    // spacing and a runway's time; pushed past the lead it spawns at its
    // entry instead, a little later.
    approaching.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut last: Option<(f64, crate::perf::Wake)> = None;
    let mut late = 0.0;
    for (mut d, wake, mut m) in approaching {
        if let Some((ld, lw)) = last {
            d = d.max(ld + (wake_gap_nm(lw, wake) + WAKE_BUFFER_NM).max(FINAL_GAP_NM) * NM);
        }
        if d > APPROACH_LEAD_NM * NM {
            late += 120.0;
            m.start = None;
            m.t_s = late;
            out.movements.push(m);
            continue;
        }
        last = Some((d, wake));
        m.start = Some(Start::Approach { to_thr_m: d });
        out.movements.push(m);
    }
    // The arrivals still to enter come no closer than a runway lands
    // ([`crate::rate::MIN_GAP_S`]), the later one held back: four in 45 s
    // at Malaga's 10:00 bank (the harness, 2026-10-13) orbited.
    let mut last_entry = f64::NEG_INFINITY;
    let mut entering: Vec<&mut Movement> = out
        .movements
        .iter_mut()
        .filter(|m| m.kind == MovementKind::Arrival && m.start.is_none())
        .collect();
    entering.sort_by(|a, b| a.t_s.total_cmp(&b.t_s));
    for m in entering {
        m.t_s = m.t_s.max(last_entry + crate::rate::MIN_GAP_S);
        last_entry = m.t_s;
    }
    out.movements.sort_by(|a, b| a.t_s.total_cmp(&b.t_s));
    // Every flight's planned stand on its board line; a departure with no
    // stand yet (waiting for its inbound, or none free at the start) starts
    // there when it appears, where free.
    for b in &mut out.board {
        b.info.stand = stand_plan.get(&b.callsign).cloned();
    }
    for m in &mut out.movements {
        let planned = stand_plan.get(&m.callsign).cloned();
        if m.kind == MovementKind::Departure && m.stand.is_none() {
            m.stand.clone_from(&planned);
        }
        if let Some(f) = m.flight.as_mut() {
            f.stand = planned;
        }
    }
    // The departures already gone at the start (2026-10-13, the whole
    // flight): every one off the blocks before now that no movement above
    // places, back as far as the longest flight lasts, flown on by the
    // model from its wheels up ([`Start::Away`]; the link drops the ones
    // already on the blocks at the other end).
    // By number AND time: a route's numbers repeat every day, and
    // tomorrow's flight is a movement already.
    let key = |cs: &str, est: Option<f64>| (cs.to_string(), est.map(|e| (e / 60.0).round() as i64));
    let placed: std::collections::HashSet<(String, Option<i64>)> = out
        .movements
        .iter()
        .map(|m| key(&m.callsign, m.flight.as_ref().and_then(|f| f.est_s)))
        .collect();
    let fits = |t: &str| {
        end.length_m >= runway_needed_m(t) && field.net.stands.iter().any(|s| field.takes(s, t))
    };
    for r in rotation::window(table, date, seed, density, now_s - AWAY_BACK_S, now_s) {
        if r.dep.est_s >= now_s
            || placed.contains(&key(&r.dep.callsign, Some(r.dep.est_s)))
            || !fits(r.icao_type)
        {
            continue;
        }
        let mut quiet = XorShift64::new(r.seed);
        let mut m = movement(&r, false, 0.0, &runway, field, ctx, &mut quiet);
        // Its stand only on the board: it stands on none now.
        let stand = stand_plan.get(&m.callsign).cloned();
        let way = stand
            .as_deref()
            .and_then(|n| field.stand(n))
            .and_then(|s| taxi_out_way(field, &end, s));
        let off_s = r.dep.est_s - now_s;
        m.start = Some(Start::Away {
            off_s,
            up_s: off_s + out_times(way.as_ref().map(|(w, _)| w.as_slice()), r.icao_type).1,
        });
        if let Some(f) = m.flight.as_mut() {
            f.inbound = None;
            f.stand = stand;
        }
        out.movements.push(m);
    }
    let _ = OWN_CLEAR_NM;
    out
}

/// The start looks this far back for departures still in the air,
/// seconds: the longest flight a table names (Malaga to Doha, about 7 h)
/// and a margin.
pub const AWAY_BACK_S: f64 = 9.0 * 3600.0;

impl Movement {
    /// The aircraft placed where its [`Start`] says (the mid-flow start), as
    /// traffic id `id`, or why not: never within [`OWN_CLEAR_NM`] of us in
    /// the air nor [`OWN_GROUND_CLEAR_M`] on the ground, never on a stand
    /// somebody holds.
    pub(crate) fn place(
        &self,
        id: u32,
        field: &FieldModel,
        ctx: &SpawnCtx,
        start: &Start,
    ) -> Result<TrafficAircraft, Refusal> {
        let plan = self.plan(field)?;
        let end = field.end(&self.runway).ok_or(Refusal::NoRunway)?;
        let ac = match start {
            Start::Approach { to_thr_m } => on_approach(self, id, plan, field, *to_thr_m),
            Start::TaxiIn { stand, frac } => {
                let s = field
                    .stand(stand)
                    .filter(|s| !crate::field::stand_taken(s, &ctx.occupied))
                    .ok_or(Refusal::NoStand)?;
                let way = taxi_in_way(field, &end, &self.icao_type, s).ok_or(Refusal::NoStand)?;
                let (pos, ahead) = along(&way, *frac).ok_or(Refusal::NoStand)?;
                let mut ac = on_ground(self, id, plan, field, pos, &ahead, Phase::TaxiIn);
                ac.stand = Some(s.name.clone());
                Some(ac)
            }
            Start::TaxiOut { .. } | Start::HoldShort => {
                let s = self
                    .stand
                    .as_deref()
                    .and_then(|n| field.stand(n))
                    .ok_or(Refusal::NoStand)?;
                let (way, lead_on) = taxi_out_way(field, &end, s).ok_or(Refusal::NoStand)?;
                let (phase, frac) = match start {
                    Start::TaxiOut { frac } => (Phase::TaxiOut, *frac),
                    _ => (Phase::HoldShort, 1.0),
                };
                let (pos, ahead) = along(&way, frac).ok_or(Refusal::NoStand)?;
                let mut ac = on_ground(self, id, plan, field, pos, &ahead, phase);
                ac.stand = Some(s.name.clone());
                ac.lead_on = lead_on;
                Some(ac)
            }
            Start::Climb { past_end_m } => climbing(self, id, plan, field, &end, *past_end_m),
            // Gone before the start: the whole-flight model flies it
            // ([`crate::track`]), never the simulation.
            Start::Away { .. } => None,
        }
        .ok_or(Refusal::NoRunway)?;
        let pos = ac.kin.pos();
        if let Some(o) = ctx.own {
            let clear = if ac.kin.on_ground {
                OWN_GROUND_CLEAR_M
            } else {
                OWN_CLEAR_NM * NM
            };
            if dist_m(o, pos) < clear {
                return Err(Refusal::NearOwn);
            }
        }
        // Nobody on the ground where it is put (the stands taken, the
        // aircraft already standing).
        if ac.kin.on_ground && ctx.occupied.iter().any(|p| dist_m(*p, pos) < WAY_CLEAR_M) {
            return Err(Refusal::NoStand);
        }
        Ok(ac)
    }
}

/// An instrument arrival `to_thr_m` along its approach from the threshold:
/// the route it would have flown from its entry, the aircraft on it at the
/// radar unit's speed for there, on the 3 degree path, working approach
/// (beyond the tower's call distance) or already the tower's.
fn on_approach(
    m: &Movement,
    id: u32,
    plan: Plan,
    field: &FieldModel,
    to_thr_m: f64,
) -> Option<TrafficAircraft> {
    let mut ac = TrafficAircraft::at_entry(
        id,
        &m.callsign,
        &m.spoken,
        &m.icao_type,
        plan,
        field,
        m.seed,
    )?;
    if !ac.instrument() {
        return None;
    }
    let thr = ac.route.thr_idx?;
    let path = ac.route.path.clone();
    // Walk back from the threshold.
    let mut left = to_thr_m;
    let mut seg = 0;
    let mut pos = path[0];
    for i in (0..thr).rev() {
        let d = dist_m(path[i], path[i + 1]);
        if left <= d {
            let f = 1.0 - left / d;
            pos = (
                path[i].0 + (path[i + 1].0 - path[i].0) * f,
                path[i].1 + (path[i + 1].1 - path[i].1) * f,
            );
            seg = i;
            break;
        }
        left -= d;
    }
    let end = field.end(ac.plan.runway()?)?;
    let alt = (end.elev_ft + TCH_FT + glide_ft(to_thr_m)).min(ac.kin.alt_ft);
    let mut kin = Kin::parked(pos, alt, bearing(pos, path[seg + 1]));
    kin.on_ground = false;
    kin.gear = ac.kin.gear;
    kin.gs_kts = scheduled_kts(ac.perf, to_thr_m);
    kin.lights = ac.kin.lights;
    ac.kin = kin;
    ac.route.follow.seg = seg;
    ac.route.follow.nose_seg = seg;
    let tower_nm = if field.airport.freq(nav::FREQ_APPROACH).is_some() {
        crate::arrival::TOWER_CALL_APPROACH_NM
    } else {
        crate::arrival::TOWER_CALL_NM
    };
    ac.ifr_radio = if to_thr_m <= tower_nm * NM {
        IfrRadio::WithTower
    } else {
        IfrRadio::Quiet
    };
    Some(ac)
}

/// A taxiing aircraft at `pos` driving `ahead`, in `phase`.
fn on_ground(
    m: &Movement,
    id: u32,
    plan: Plan,
    field: &FieldModel,
    pos: LatLon,
    ahead: &[LatLon],
    phase: Phase,
) -> TrafficAircraft {
    let hdg = ahead.get(1).map_or(0.0, |n| bearing(pos, *n));
    let mut kin = Kin::parked(pos, field.elev_ft(), hdg);
    kin.lights.nav = true;
    kin.lights.beacon = true;
    kin.lights.taxi = phase != Phase::HoldShort;
    let mut ac = TrafficAircraft::new(
        id,
        &m.callsign,
        &m.spoken,
        &m.icao_type,
        plan,
        kin,
        phase,
        m.seed,
    );
    ac.kin.gs_kts = if phase == Phase::HoldShort {
        0.0
    } else {
        ac.perf.taxi_kts.min(12.0)
    };
    if ahead.len() >= 2 {
        ac.set_taxi_route(ahead, field);
    } else {
        ac.route = Route::taxi(vec![pos]);
    }
    ac
}

/// A departure `past_end_m` past the far end of `end`, on its way out.
fn climbing(
    m: &Movement,
    id: u32,
    plan: Plan,
    field: &FieldModel,
    end: &crate::field::RunwayEnd,
    past_end_m: f64,
) -> Option<TrafficAircraft> {
    let Plan::Departure { exit_bearing, .. } = &plan else {
        return None;
    };
    let upwind_end = end.at(end.length_m + 1.5 * NM, 0.0);
    let out = crate::kin::project_m(upwind_end, *exit_bearing, 30.0 * NM);
    let route_pts = vec![end.far, upwind_end, out];
    let (pos, seg) = if past_end_m < 1.5 * NM {
        (end.at(end.length_m + past_end_m, 0.0), 0)
    } else {
        (
            crate::kin::project_m(upwind_end, *exit_bearing, past_end_m - 1.5 * NM),
            1,
        )
    };
    let (perf, _) = class_for_type(&m.icao_type);
    let agl = (400.0 + past_end_m / NM * 600.0).min(6000.0);
    let mut kin = Kin::parked(pos, field.elev_ft() + agl, bearing(pos, route_pts[seg + 1]));
    kin.on_ground = false;
    kin.gear = if perf.retractable_gear { 0.0 } else { 1.0 };
    kin.gs_kts = perf.climb_kts;
    kin.lights.nav = true;
    kin.lights.beacon = true;
    kin.lights.strobe = true;
    let phase = if seg == 0 { Phase::Climb } else { Phase::Leave };
    let mut ac = TrafficAircraft::new(
        id,
        &m.callsign,
        &m.spoken,
        &m.icao_type,
        plan,
        kin,
        phase,
        m.seed,
    );
    let mut route = Route::new(route_pts, vec![Leg::Upwind, Leg::Outbound], None);
    route.follow.seg = seg;
    route.follow.nose_seg = seg;
    ac.route = route;
    Some(ac)
}

#[cfg(test)]
mod tests;
