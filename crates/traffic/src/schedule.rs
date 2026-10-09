//! Schedules: who flies when. Generated from the field and a seed
//! ([`generate`]), or read from the pilot's file ([`parse_file`],
//! [`from_file`]); never fetched. A schedule is a list of [`Movement`]s;
//! the bin spawns each at its time through [`Movement::spawn`].
//!
//! Despawn rules live with the phase machine (`aircraft.rs`): a departure
//! is gone [`DEPART_GONE_NM`] out, an arrival [`PARKED_LINGER_S`] after it
//! parks, a circuit aircraft once its laps are flown and it has parked.

use nav::Stand;
use serde::Deserialize;

use crate::aircraft::{landing_distance_m, TrafficAircraft};
pub use crate::aircraft::{DEPART_GONE_NM, PARKED_LINGER_S};
use crate::field::{
    neighbour_taken, stand_taken, stand_width, tightest_first, FieldModel, RunwayEnd,
};
use crate::kin::{dist_m, project_m, LatLon};
use crate::perf::{class_for_type, wingspan_code, Perf, JET_MED, PISTON_SINGLE, TURBOPROP};
use crate::phase::Plan;
use crate::rng::XorShift64;

/// How busy the generated sky is (`XCOPILOT_TRAFFIC_DENSITY`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Density {
    Off,
    Light,
    Normal,
}

impl Density {
    /// `off`, `light` or `normal` (any case); anything else is `None`.
    pub fn parse(s: &str) -> Option<Density> {
        match s.trim().to_ascii_lowercase().as_str() {
            "off" => Some(Density::Off),
            "light" => Some(Density::Light),
            "normal" => Some(Density::Normal),
            _ => None,
        }
    }

    /// Minutes between IFR movements of each kind.
    fn ifr_interval_s(self) -> f64 {
        match self {
            Density::Off => f64::INFINITY,
            Density::Light => 20.0 * 60.0,
            Density::Normal => 10.0 * 60.0,
        }
    }
}

/// A generated schedule covers this much time; the bin generates again.
pub const HORIZON_S: f64 = 3600.0;

/// Nothing spawns in the air this close to our own aircraft.
pub const OWN_CLEAR_NM: f64 = 3.0;

/// An arrival starts this far out.
pub const ENTRY_NM: f64 = 10.0;

/// The schedule's first movement comes this soon after the fleet starts,
/// seconds (a draw between the two): the first live hour at LEMG
/// (2026-10-08) showed an empty field for four to six minutes.
pub const FIRST_MOVE_S: (f64, f64) = (30.0, 60.0);

/// The regional turboprop the schedule flies (an ATR 72, code C), and the
/// smaller one where no stand is wide enough for it (a Saab 340, code B).
pub const REGIONAL: &str = "AT76";
pub const SMALL_REGIONAL: &str = "SF34";

/// The shortest runway a jet is scheduled on, and a turboprop.
pub const JET_MIN_RUNWAY_M: f64 = 1800.0;
pub const TURBOPROP_MIN_RUNWAY_M: f64 = 1100.0;

/// What kind of movement.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum MovementKind {
    Circuit,
    Departure,
    Arrival,
    /// Stands on its stand from the fleet's start and never moves (drawn
    /// only, [`Plan::Parked`]): the field's parked aircraft
    /// ([`park_at_start`]).
    Parked,
}

/// The share of a field's suitable stands filled at the fleet's start, by
/// density ([`park_at_start`]).
pub const PARKED_SHARE_LIGHT: f64 = 0.15;
pub const PARKED_SHARE_NORMAL: f64 = 0.30;

/// Stands left free at the start for the arrivals, whatever the share.
pub const STANDS_KEPT_FREE: usize = 3;

/// Nothing is parked this close to our own aircraft, metres.
pub const PARK_CLEAR_OWN_M: f64 = 60.0;

/// The most aircraft parked at the start, however big the field: each is an
/// XPMP2 aircraft the plugin draws every frame (frame cost unmeasured past
/// 60 aircraft, see the risks).
pub const PARKED_MAX: usize = 40;

/// One scheduled flight.
#[derive(Debug, Clone, PartialEq)]
pub struct Movement {
    /// Seconds after the schedule's start.
    pub t_s: f64,
    pub callsign: String,
    pub spoken: String,
    pub icao_type: String,
    pub kind: MovementKind,
    pub runway: String,
    /// The stand a circuit or a departure starts on.
    pub stand: Option<String>,
    /// Where an arrival starts.
    pub entry: Option<LatLon>,
    pub straight_in: bool,
    pub laps: u32,
    pub sid: Option<String>,
    pub star: Option<String>,
    /// A departure's way out, degrees true.
    pub exit_bearing: f64,
    pub seed: u64,
    /// A departure that stands on its stand, parked and dark, from the
    /// fleet's start until its time, and starts up there
    /// ([`park_at_start`]); else it appears on its stand at its time.
    pub parked_from_start: bool,
}

/// What the world looks like at generation or spawn time.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SpawnCtx {
    /// The tower's runway in use; the field's longest when `None`.
    pub runway: Option<String>,
    /// Our own aircraft: no airborne spawn within [`OWN_CLEAR_NM`], and
    /// its stand is occupied.
    pub own: Option<LatLon>,
    /// Everyone else on the ground (stands they stand on are taken).
    pub occupied: Vec<LatLon>,
}

impl SpawnCtx {
    fn taken(&self) -> Vec<LatLon> {
        self.occupied.iter().copied().chain(self.own).collect()
    }
}

/// Why a movement could not spawn now (the bin tries again later).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Refusal {
    /// The field has no such runway.
    NoRunway,
    /// No free stand to start on.
    NoStand,
    /// The entry point is within [`OWN_CLEAR_NM`] of us.
    NearOwn,
}

impl Movement {
    /// The aircraft, as traffic id `id`, or why not now. A circuit or a
    /// departure takes its stand only when free, else the nearest free
    /// one; an arrival never starts within [`OWN_CLEAR_NM`] of us; a parked
    /// aircraft is [`Movement::park`]ed.
    pub fn spawn(
        &self,
        id: u32,
        field: &FieldModel,
        ctx: &SpawnCtx,
    ) -> Result<TrafficAircraft, Refusal> {
        if self.kind == MovementKind::Parked {
            return self.park(id, field, ctx);
        }
        let plan = self.plan(field)?;
        if let Plan::Arrival { entry, .. } = &plan {
            if ctx
                .own
                .is_some_and(|o| dist_m(o, *entry) < OWN_CLEAR_NM * 1852.0)
            {
                return Err(Refusal::NearOwn);
            }
            return TrafficAircraft::at_entry(
                id,
                &self.callsign,
                &self.spoken,
                &self.icao_type,
                plan,
                field,
                self.seed,
            )
            .ok_or(Refusal::NoRunway);
        }
        let taken = ctx.taken();
        let wanted = self
            .stand
            .as_deref()
            .and_then(|n| field.stand(n))
            .filter(|s| !stand_taken(s, &taken) && field.takes(s, &self.icao_type));
        let stand = match wanted {
            Some(s) => s.name.clone(),
            None => free_stands(field, &self.icao_type, &taken)
                .into_iter()
                .next()
                .ok_or(Refusal::NoStand)?,
        };
        TrafficAircraft::at_stand(
            id,
            &self.callsign,
            &self.spoken,
            &self.icao_type,
            plan,
            field,
            &stand,
            self.seed,
        )
        .ok_or(Refusal::NoStand)
    }

    /// What the movement flies, at its runway (`NoRunway` when the field
    /// has none such; a parked aircraft needs none).
    pub fn plan(&self, field: &FieldModel) -> Result<Plan, Refusal> {
        if self.kind == MovementKind::Parked {
            return Ok(Plan::Parked);
        }
        let end = field.end(&self.runway).ok_or(Refusal::NoRunway)?;
        Ok(match self.kind {
            MovementKind::Circuit => Plan::Circuit {
                runway: end.ident.clone(),
                laps: self.laps.max(1),
                left: end.left,
            },
            MovementKind::Departure => Plan::Departure {
                runway: end.ident.clone(),
                sid: self.sid.clone(),
                exit_bearing: self.exit_bearing,
            },
            MovementKind::Arrival => Plan::Arrival {
                entry: self.entry.unwrap_or_else(|| default_entry(&end)),
                star: self.star.clone(),
                runway: end.ident.clone(),
                straight_in: self.straight_in,
            },
            MovementKind::Parked => Plan::Parked,
        })
    }

    /// The aircraft standing on its own stand, parked and dark
    /// ([`Plan::Parked`]), as traffic id `id`: a parked aircraft, or a
    /// departure waiting for its time ([`Movement::parked_from_start`],
    /// released then by [`crate::Fleet::release`]). Only on its NAMED
    /// stand, free, and never within [`PARK_CLEAR_OWN_M`] of our aircraft.
    pub fn park(
        &self,
        id: u32,
        field: &FieldModel,
        ctx: &SpawnCtx,
    ) -> Result<TrafficAircraft, Refusal> {
        let stand = self
            .stand
            .as_deref()
            .and_then(|n| field.stand(n))
            .filter(|s| !stand_taken(s, &ctx.taken()))
            .ok_or(Refusal::NoStand)?;
        if ctx
            .own
            .is_some_and(|o| dist_m(o, (stand.lat, stand.lon)) < PARK_CLEAR_OWN_M)
        {
            return Err(Refusal::NearOwn);
        }
        TrafficAircraft::at_stand(
            id,
            &self.callsign,
            &self.spoken,
            &self.icao_type,
            Plan::Parked,
            field,
            &stand.name,
            self.seed,
        )
        .ok_or(Refusal::NoStand)
    }
}

/// Ten miles out on the extended centreline.
fn default_entry(end: &RunwayEnd) -> LatLon {
    project_m(end.thr, end.course_deg + 180.0, ENTRY_NM * 1852.0)
}

/// The field's default runway: the longest assignable strip's first end,
/// preferring an end with an approach.
pub fn default_runway(field: &FieldModel) -> Option<String> {
    let strip = nav::assignable_runways(&field.airport.runways)
        .max_by(|a, b| a.length_m().total_cmp(&b.length_m()))?;
    [&strip.ident_a, &strip.ident_b]
        .into_iter()
        .find(|id| field.procs.approach_for(id).is_some())
        .or(Some(&strip.ident_a))
        .cloned()
}

/// Stand names an `icao_type` may start on, free of `taken`, in file
/// order: the stands that take it ([`stand_fits`]: its category and its
/// wingspan), of the tightest size that has one free; a bigger stand only
/// when none of its own size is; of those, the ones with nobody beside them
/// ([`neighbour_taken`]) when there are any. Empty when none fits: no
/// spawn, never a stand too small.
fn free_stands(field: &FieldModel, icao_type: &str, taken: &[LatLon]) -> Vec<String> {
    let fits = tightest_first(
        field
            .net
            .stands
            .iter()
            // Never a stand on or against a runway strip, which starts the
            // aircraft on it (UEMH 27, world sweep 2026-10-09: "you are on
            // runway two seven without clearance" as it began to taxi), nor
            // one whose way out runs through its neighbours.
            .filter(|s| field.takes(s, icao_type))
            .filter(|s| !stand_taken(s, taken)),
        icao_type,
    );
    let Some(size) = fits.first().map(|s| stand_width(s)) else {
        return Vec::new();
    };
    let tier: Vec<&Stand> = fits
        .into_iter()
        .take_while(|s| stand_width(s) == size)
        .collect();
    let lonely: Vec<&Stand> = tier
        .iter()
        .copied()
        .filter(|s| !neighbour_taken(s, taken))
        .collect();
    if lonely.is_empty() { tier } else { lonely }
        .into_iter()
        .map(|s| s.name.clone())
        .collect()
}

/// The shortest runway a `p`-class airliner is scheduled on: `floor_m`, or
/// its good landing ([`landing_distance_m`]) where that is longer. PABV's
/// 1,220 m held an ATR's good landing (1,150 m) and not the long ones that
/// went off its end; a 1,100 m strip did not hold even the good one.
fn runway_needed_m(p: &Perf, floor_m: f64) -> f64 {
    floor_m.max(landing_distance_m(p))
}

/// Some stand at `field` takes an `icao_type` ([`FieldModel::takes`]): the
/// type can come and go there at all.
fn parks_at(field: &FieldModel, icao_type: &str) -> bool {
    field.net.stands.iter().any(|s| field.takes(s, icao_type))
}

/// A seed for the field: the same field and seed, the same schedule.
fn field_seed(icao: &str, seed: u64) -> u64 {
    icao.bytes().fold(seed ^ 0xCBF2_9CE4_8422_2325, |h, b| {
        (h ^ b as u64).wrapping_mul(0x0100_0000_01B3)
    })
}

/// Generate an hour ([`HORIZON_S`]) of traffic for `field`:
/// - 1 (light) or 1 to 3 (normal) circuit aircraft where the field allows
///   circuits (`allows_circuits != Some(false)`) and has stands;
/// - IFR departures by SID where the field publishes SIDs, and IFR
///   arrivals (straight in where the runway has an approach, else to the
///   downwind) where it publishes approaches or STARs, every 20 (light) or
///   10 (normal) minutes, on a runway long enough for the type;
/// - the first movement within [`FIRST_MOVE_S`] of the start: the stream
///   it belongs to (a ground start's, circuits or departures, before the
///   arrivals') is brought forward whole, so its spacing and the hour's
///   count are unchanged;
/// - never on an occupied stand (ours, anyone's in `ctx`, or one already
///   handed out), never an entry within [`OWN_CLEAR_NM`] of us.
pub fn generate(field: &FieldModel, density: Density, seed: u64, ctx: &SpawnCtx) -> Vec<Movement> {
    let mut out = Vec::new();
    if density == Density::Off {
        return out;
    }
    let Some(runway) = ctx.runway.clone().or_else(|| default_runway(field)) else {
        return out;
    };
    let Some(end) = field.end(&runway) else {
        return out;
    };
    let mut rng = XorShift64::new(field_seed(&field.icao, seed));
    let mut taken = ctx.taken();
    let take_stand =
        |icao_type: &str, rng: &mut XorShift64, taken: &mut Vec<LatLon>| -> Option<String> {
            let free = free_stands(field, icao_type, taken);
            let name = free.get(rng.below(free.len() as u64) as usize)?.clone();
            let s = field.stand(&name)?;
            taken.push((s.lat, s.lon));
            Some(name)
        };
    let prefix = registration_prefix(&field.icao);

    if field.airport.allows_circuits != Some(false) {
        let n = match density {
            Density::Normal => 1 + rng.below(3) as u32,
            _ => 1,
        };
        let mut t = rng.range_f64(30.0, 180.0);
        for _ in 0..n {
            let icao_type = ["C172", "C152", "PA28", "SR22"][rng.below(4) as usize];
            let Some(stand) = take_stand(icao_type, &mut rng, &mut taken) else {
                break;
            };
            let (callsign, spoken) = registration(prefix, icao_type, &mut rng);
            out.push(Movement {
                t_s: t,
                callsign,
                spoken,
                icao_type: icao_type.into(),
                kind: MovementKind::Circuit,
                runway: end.ident.clone(),
                stand: Some(stand),
                entry: None,
                straight_in: false,
                laps: 2 + rng.below(3) as u32,
                sid: None,
                star: None,
                exit_bearing: 0.0,
                seed: rng.next_u64(),
                parked_from_start: false,
            });
            t += rng.range_f64(120.0, 420.0);
        }
    }

    // The airline types the runway takes and a stand at the field fits: an
    // airliner is never scheduled where it has nowhere to park. Where no
    // stand is wide enough for the ATR (code C) the regional is the Saab
    // ([`SMALL_REGIONAL`], code B): UNSS's ten stands and ULKK's are all B.
    let regional = if parks_at(field, REGIONAL) {
        REGIONAL
    } else {
        SMALL_REGIONAL
    };
    let pool: Vec<&'static str> = if end.length_m >= runway_needed_m(&JET_MED, JET_MIN_RUNWAY_M) {
        vec!["A320", "B738", "A319", regional]
    } else if end.length_m >= runway_needed_m(&TURBOPROP, TURBOPROP_MIN_RUNWAY_M) {
        vec![regional]
    } else {
        Vec::new()
    }
    .into_iter()
    .filter(|t| parks_at(field, t))
    .collect();
    let ifr_type = |rng: &mut XorShift64| -> Option<&'static str> {
        (!pool.is_empty()).then(|| pool[rng.below(pool.len() as u64) as usize])
    };
    let every = density.ifr_interval_s();
    let has_stands = field.net.stands.iter().any(|s| s.is_suitable());

    // A field that publishes no procedures still has its airline traffic,
    // flown visually: a departure off the runway heading and away, an
    // arrival joining the circuit. LXGB (2026-10-09): Gibraltar's CIFP
    // holds runways only and its stands take jets, so the hour was empty
    // and the pilot sat at the field with no traffic at all.
    let visual = field.procs.sids.is_empty()
        && field.procs.approaches.is_empty()
        && field.procs.stars.is_empty();
    if (!field.procs.sids.is_empty() || visual) && has_stands {
        let mut t = rng.range_f64(60.0, every);
        while t < HORIZON_S {
            let Some(icao_type) = ifr_type(&mut rng) else {
                break;
            };
            let Some(stand) = take_stand(icao_type, &mut rng, &mut taken) else {
                break;
            };
            let sid = (!field.procs.sids.is_empty()).then(|| {
                field.procs.sids[rng.below(field.procs.sids.len() as u64) as usize].clone()
            });
            let (callsign, spoken, icao_type) = airline(&field.icao, icao_type, &mut rng);
            out.push(Movement {
                t_s: t,
                callsign,
                spoken,
                icao_type: icao_type.into(),
                kind: MovementKind::Departure,
                runway: end.ident.clone(),
                stand: Some(stand),
                entry: None,
                straight_in: false,
                laps: 0,
                sid,
                star: None,
                exit_bearing: rng.range_f64(0.0, 360.0),
                seed: rng.next_u64(),
                parked_from_start: false,
            });
            t += every * rng.range_f64(0.8, 1.2);
        }
    }

    let arrivals = !field.procs.approaches.is_empty() || !field.procs.stars.is_empty() || visual;
    if arrivals && has_stands {
        let straight_in = field.procs.approach_for(&end.ident).is_some();
        let mut t = rng.range_f64(every * 0.3, every);
        while t < HORIZON_S {
            let Some(icao_type) = ifr_type(&mut rng) else {
                break;
            };
            let Some(entry) = entry_clear_of_own(&end, ctx.own, &mut rng) else {
                t += every;
                continue;
            };
            let star = (!field.procs.stars.is_empty()).then(|| {
                field.procs.stars[rng.below(field.procs.stars.len() as u64) as usize].clone()
            });
            let (callsign, spoken, icao_type) = airline(&field.icao, icao_type, &mut rng);
            out.push(Movement {
                t_s: t,
                callsign,
                spoken,
                icao_type: icao_type.into(),
                kind: MovementKind::Arrival,
                runway: end.ident.clone(),
                stand: None,
                entry: Some(entry),
                straight_in,
                laps: 0,
                sid: None,
                star,
                exit_bearing: 0.0,
                seed: rng.next_u64(),
                parked_from_start: false,
            });
            t += every * rng.range_f64(0.8, 1.2);
        }
    }
    bring_first_forward(&mut out, &mut rng);
    out.sort_by(|a, b| a.t_s.total_cmp(&b.t_s));
    out
}

/// The first movement within [`FIRST_MOVE_S`]: the earliest ground start
/// (a circuit or a departure: something seen moving on the field), else the
/// earliest arrival, and every movement of its kind with it, by the same
/// shift. Already that soon, nothing moves. One draw of `rng`, always.
fn bring_first_forward(out: &mut [Movement], rng: &mut XorShift64) {
    let first_s = rng.range_f64(FIRST_MOVE_S.0, FIRST_MOVE_S.1);
    let earliest = |ground: bool| {
        out.iter()
            .filter(|m| {
                let start = matches!(m.kind, MovementKind::Circuit | MovementKind::Departure);
                start == ground && m.kind != MovementKind::Parked
            })
            .min_by(|a, b| a.t_s.total_cmp(&b.t_s))
            .map(|m| (m.kind, m.t_s))
    };
    let Some((kind, t)) = earliest(true).or_else(|| earliest(false)) else {
        return;
    };
    let shift = t - first_s;
    if shift > 0.0 {
        for m in out.iter_mut().filter(|m| m.kind == kind) {
            m.t_s -= shift;
        }
    }
}

/// A parked aircraft's stand is at least this far from any lane others
/// taxi along, metres (half a medium jet's span and a margin).
const PARKED_LANE_CLEAR_M: f64 = 25.0;

/// THE FIELD IS NOT EMPTY AT THE START (LEMG, 2026-10-08: no parked
/// aircraft at all): a share of the field's suitable stands
/// ([`PARKED_SHARE_LIGHT`], [`PARKED_SHARE_NORMAL`]; at most
/// [`PARKED_MAX`]) stands occupied from the fleet's start. The `schedule`'s departures fill it first, earliest
/// first: each is marked [`Movement::parked_from_start`], waits on its own
/// stand parked and dark, and starts up there at its time. The rest of the
/// share is returned as [`MovementKind::Parked`] aircraft at `t_s` 0, each
/// of the class its stand takes (an airliner of a carrier that flies the
/// type at a gate or a jet stand, the regional turboprop at a turboprop
/// stand, a light aircraft on a GA stand or a tie-down), never on a stand
/// the schedule or `ctx` holds, and never so many that fewer than
/// [`STANDS_KEPT_FREE`] stands are left for the arrivals. Our own stand and
/// [`PARK_CLEAR_OWN_M`] round us are refused at spawn ([`Movement::park`]).
/// Deterministic per field and seed; nothing at `Off`.
pub fn park_at_start(
    field: &FieldModel,
    density: Density,
    seed: u64,
    ctx: &SpawnCtx,
    schedule: &mut [Movement],
) -> Vec<Movement> {
    let share = match density {
        Density::Off => return Vec::new(),
        Density::Light => PARKED_SHARE_LIGHT,
        Density::Normal => PARKED_SHARE_NORMAL,
    };
    let suitable: Vec<&Stand> = field
        .net
        .stands
        .iter()
        .filter(|s| s.is_suitable())
        .collect();
    let mut budget = ((share * suitable.len() as f64).floor() as usize).min(PARKED_MAX);
    let mut order: Vec<&mut Movement> = schedule
        .iter_mut()
        .filter(|m| m.kind == MovementKind::Departure && m.stand.is_some())
        .collect();
    order.sort_by(|a, b| a.t_s.total_cmp(&b.t_s));
    for m in order {
        if budget == 0 {
            break;
        }
        m.parked_from_start = true;
        budget -= 1;
    }
    let mut taken = ctx.taken();
    taken.extend(
        schedule
            .iter()
            .filter_map(|m| m.stand.as_deref().and_then(|n| field.stand(n)))
            .map(|s| (s.lat, s.lon)),
    );
    // A parked aircraft never moves, so it never stands by a lane others
    // taxi along: BIRK (world sweep, 2026-10-09) parked one 8 m off the
    // way to the apron and every arrival's wingtip went through it.
    let free: Vec<&Stand> = suitable
        .into_iter()
        .filter(|s| !stand_taken(s, &taken))
        .filter(|s| field.lane_clearance_m((s.lat, s.lon)) >= PARKED_LANE_CLEAR_M)
        .collect();
    // Nobody parked beside a stand the schedule's flights leave from or come
    // to while the apron has room elsewhere ([`neighbour_taken`]): the
    // stands beside them are drawn from last.
    let movers: Vec<LatLon> = schedule
        .iter()
        .filter_map(|m| m.stand.as_deref().and_then(|n| field.stand(n)))
        .map(|s| (s.lat, s.lon))
        .collect();
    let (mut free, mut beside): (Vec<&Stand>, Vec<&Stand>) =
        free.into_iter().partition(|s| !neighbour_taken(s, &movers));
    let n = budget.min((free.len() + beside.len()).saturating_sub(STANDS_KEPT_FREE));
    let mut rng = XorShift64::new(field_seed(&field.icao, seed) ^ 0x5041_524B_4544);
    let longest = field
        .airport
        .runways
        .iter()
        .map(|r| r.length_m())
        .fold(0.0, f64::max);
    let prefix = registration_prefix(&field.icao);
    let mut out = Vec::new();
    while out.len() < n && !(free.is_empty() && beside.is_empty()) {
        if free.is_empty() {
            std::mem::swap(&mut free, &mut beside);
        }
        let stand = free.swap_remove(rng.below(free.len() as u64) as usize);
        let Some(icao_type) = parked_type(stand, longest, &mut rng) else {
            continue;
        };
        let (callsign, spoken, icao_type) =
            if class_for_type(icao_type).0.name == PISTON_SINGLE.name {
                let (c, s) = registration(prefix, icao_type, &mut rng);
                (c, s, icao_type)
            } else {
                airline(&field.icao, icao_type, &mut rng)
            };
        out.push(Movement {
            t_s: 0.0,
            callsign,
            spoken,
            icao_type: icao_type.into(),
            kind: MovementKind::Parked,
            runway: String::new(),
            stand: Some(stand.name.clone()),
            entry: None,
            straight_in: false,
            laps: 0,
            sid: None,
            star: None,
            exit_bearing: 0.0,
            seed: rng.next_u64(),
            parked_from_start: false,
        });
    }
    out
}

/// What stands parked on `stand`: the biggest class it takes that the
/// field's longest runway (`longest_m`) is scheduled for (one stand in five
/// that also takes turboprops gets the regional turboprop). A stand naming
/// no class is judged by its kind (a gate takes an airliner, anything else
/// a light aircraft). Never a type wider than the stand ([`stand_width`]):
/// an airliner on a light aircraft's stand stands in its neighbours, and
/// where the ATR is too wide the Saab ([`SMALL_REGIONAL`]) stands. `None`
/// when it takes nothing this field flies.
fn parked_type(stand: &Stand, longest_m: f64, rng: &mut XorShift64) -> Option<&'static str> {
    let takes = |c: &str| stand.accepts.iter().any(|a| a == c);
    let wide = |t: &str| stand_width(stand) >= wingspan_code(t);
    let (jets, turboprops, props) = if stand.accepts.is_empty() {
        let gate = stand.kind.eq_ignore_ascii_case("gate");
        (gate, gate, !gate)
    } else {
        (
            takes("jets") || takes("heavy"),
            takes("turboprops"),
            takes("props"),
        )
    };
    // A stand that takes both is a turboprop's one time in five.
    let regional = turboprops && rng.below(5) == 0;
    if jets && !regional && longest_m >= JET_MIN_RUNWAY_M && wide("A320") {
        Some(["A320", "B738", "A319"][rng.below(3) as usize])
    } else if (jets || turboprops) && longest_m >= TURBOPROP_MIN_RUNWAY_M && wide(REGIONAL) {
        Some(REGIONAL)
    } else if (jets || turboprops) && longest_m >= TURBOPROP_MIN_RUNWAY_M && wide(SMALL_REGIONAL) {
        Some(SMALL_REGIONAL)
    } else if (props || stand.accepts.is_empty()) && wide("C172") {
        Some(["C172", "C152", "PA28", "SR22"][rng.below(4) as usize])
    } else {
        None
    }
}

/// An entry [`ENTRY_NM`] out, within 50 degrees of the extended
/// centreline, clear of us by [`OWN_CLEAR_NM`]; turned further round when
/// we are in the way, `None` if nowhere is clear.
fn entry_clear_of_own(
    end: &RunwayEnd,
    own: Option<LatLon>,
    rng: &mut XorShift64,
) -> Option<LatLon> {
    let base = end.course_deg + 180.0 + rng.range_f64(-50.0, 50.0);
    (0..8)
        .map(|k| base + k as f64 * 45.0)
        .map(|b| project_m(end.thr, b, ENTRY_NM * 1852.0))
        .find(|p| own.is_none_or(|o| dist_m(o, *p) >= OWN_CLEAR_NM * 1852.0))
}

/// The registration prefix by the field's ICAO region.
fn registration_prefix(icao: &str) -> &'static str {
    match icao.get(..2).unwrap_or("") {
        "EG" => "G",
        "LE" | "GC" => "EC",
        "LF" => "F",
        "ED" | "ET" => "D",
        "LI" => "I",
        "EH" => "PH",
        "EB" => "OO",
        "LS" => "HB",
        "LO" => "OE",
        _ => "N",
    }
}

const PHONETIC: [&str; 26] = [
    "Alpha", "Bravo", "Charlie", "Delta", "Echo", "Foxtrot", "Golf", "Hotel", "India", "Juliett",
    "Kilo", "Lima", "Mike", "November", "Oscar", "Papa", "Quebec", "Romeo", "Sierra", "Tango",
    "Uniform", "Victor", "Whiskey", "X-ray", "Yankee", "Zulu",
];

/// A light aircraft's registration and how it is spoken: "N345SP",
/// "Cessna three four five Sierra Papa"; "G-ABCD", "Cessna Golf Alpha
/// Bravo Charlie Delta".
fn registration(prefix: &str, icao_type: &str, rng: &mut XorShift64) -> (String, String) {
    let maker = match icao_type {
        "PA28" => "Piper",
        "SR22" => "Cirrus",
        _ => "Cessna",
    };
    let letter = |rng: &mut XorShift64| (b'A' + rng.below(26) as u8) as char;
    if prefix == "N" {
        let first = char::from(b'1' + rng.below(9) as u8);
        let digits: String = std::iter::once(first)
            .chain((0..2).map(|_| char::from(b'0' + rng.below(10) as u8)))
            .collect();
        let tail: String = (0..2).map(|_| letter(rng)).collect();
        let spoken = format!(
            "{maker} {} {}",
            digits.chars().map(digit_word).collect::<Vec<_>>().join(" "),
            tail.chars().map(phonetic).collect::<Vec<_>>().join(" ")
        );
        (format!("N{digits}{tail}"), spoken)
    } else {
        let n = 5usize.saturating_sub(prefix.len()).max(3);
        let tail: String = (0..n).map(|_| letter(rng)).collect();
        let spoken = format!(
            "{maker} {}",
            prefix
                .chars()
                .chain(tail.chars())
                .map(phonetic)
                .collect::<Vec<_>>()
                .join(" ")
        );
        (format!("{prefix}{tail}"), spoken)
    }
}

/// What each carrier flies: a type the airline does not fly is swapped for its
/// own (a Ryanair A320 at LEMG drew a Wizz Air livery, 2026-10-08).
fn fleet_of(code: &str) -> &'static [&'static str] {
    match code {
        "RYR" | "TVF" => &["B738"],
        "EZY" | "BAW" => &["A320", "A319"],
        "ANE" | "LOG" | "HOP" => &[REGIONAL, SMALL_REGIONAL],
        _ => &["A320"],
    }
}

/// The regional turboprop carrier for the region.
fn regional(icao: &str) -> (&'static str, &'static str) {
    match icao.get(..2).unwrap_or("") {
        "EG" => ("LOG", "Logan"),
        "LF" => ("HOP", "Air Hop"),
        _ => ("ANE", "Air Nostrum"),
    }
}

/// An airline callsign for the region, its telephony, and the type it flies
/// (`icao_type` kept when the carrier flies it, else the carrier's own).
fn airline(icao: &str, icao_type: &'static str, rng: &mut XorShift64) -> (String, String, &'static str) {
    let carriers: &[(&str, &str)] = match icao.get(..2).unwrap_or("") {
        "EG" => &[("BAW", "Speedbird"), ("EZY", "Easy"), ("RYR", "Ryanair")],
        "LE" | "GC" => &[("IBE", "Iberia"), ("VLG", "Vueling"), ("RYR", "Ryanair")],
        "LF" => &[
            ("AFR", "Air France"),
            ("EZY", "Easy"),
            ("TVF", "France Soleil"),
        ],
        "ED" | "ET" => &[
            ("DLH", "Lufthansa"),
            ("EWG", "Eurowings"),
            ("CFG", "Condor"),
        ],
        "LI" => &[("ITY", "Itarrow"), ("RYR", "Ryanair"), ("EZY", "Easy")],
        _ => &[("RYR", "Ryanair"), ("EZY", "Easy"), ("DLH", "Lufthansa")],
    };
    let (mut code, mut telephony) = carriers[rng.below(carriers.len() as u64) as usize];
    if icao_type == REGIONAL || icao_type == SMALL_REGIONAL {
        (code, telephony) = regional(icao);
    }
    let fleet = fleet_of(code);
    let icao_type = if fleet.contains(&icao_type) { icao_type } else { fleet[0] };
    let number = 100 + rng.below(9800);
    let words = number
        .to_string()
        .chars()
        .map(digit_word)
        .collect::<Vec<_>>()
        .join(" ");
    (format!("{code}{number}"), format!("{telephony} {words}"), icao_type)
}

fn digit_word(c: char) -> &'static str {
    match c {
        '0' => "zero",
        '1' => "one",
        '2' => "two",
        '3' => "three",
        '4' => "four",
        '5' => "five",
        '6' => "six",
        '7' => "seven",
        '8' => "eight",
        '9' => "niner",
        _ => "",
    }
}

fn phonetic(c: char) -> &'static str {
    match c.to_ascii_uppercase() {
        c @ 'A'..='Z' => PHONETIC[(c as u8 - b'A') as usize],
        c => digit_word(c),
    }
}

/// One row of the pilot's traffic file (`XCOPILOT_TRAFFIC_FILE`, YAML, a
/// list of these). The bin reads the variable and the file; this crate
/// only parses the text.
///
/// ```yaml
/// - t_s: 0
///   callsign: N345SP
///   type: C172
///   kind: circuit        # circuit | departure | arrival | parked
///   runway: "13"         # optional: the tower's runway in use
///   stand: W4            # optional: a free stand
///   laps: 3              # optional, circuits (default 3)
///   spoken: Cessna three four five   # optional
/// ```
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FileEntry {
    pub t_s: f64,
    pub callsign: String,
    #[serde(rename = "type")]
    pub icao_type: String,
    pub kind: MovementKind,
    #[serde(default)]
    pub runway: Option<String>,
    #[serde(default)]
    pub stand: Option<String>,
    #[serde(default)]
    pub laps: Option<u32>,
    #[serde(default)]
    pub spoken: Option<String>,
}

/// Parse a traffic file. Refused whole on any bad row, with the row named:
/// a half-read schedule is a schedule nobody wrote.
pub fn parse_file(text: &str) -> anyhow::Result<Vec<FileEntry>> {
    let rows: Vec<FileEntry> = serde_yaml::from_str(text)?;
    for (i, r) in rows.iter().enumerate() {
        anyhow::ensure!(
            r.t_s.is_finite() && r.t_s >= 0.0,
            "row {}: t_s must be 0 or more",
            i + 1
        );
        anyhow::ensure!(!r.callsign.trim().is_empty(), "row {}: no callsign", i + 1);
        anyhow::ensure!(!r.icao_type.trim().is_empty(), "row {}: no type", i + 1);
    }
    Ok(rows)
}

/// The file's rows as movements at `field`: the runway defaults to the
/// tower's (or the field's), an arrival enters ten miles out (straight in
/// where the runway has an approach), a departure leaves on a seeded
/// bearing. A row naming a runway the field lacks is dropped.
pub fn from_file(
    rows: &[FileEntry],
    field: &FieldModel,
    seed: u64,
    ctx: &SpawnCtx,
) -> Vec<Movement> {
    let mut rng = XorShift64::new(field_seed(&field.icao, seed));
    let fallback = ctx.runway.clone().or_else(|| default_runway(field));
    let mut out: Vec<Movement> = rows
        .iter()
        .filter_map(|r| {
            let runway = r.runway.clone().or_else(|| fallback.clone())?;
            let end = field.end(&runway)?;
            let entry = (r.kind == MovementKind::Arrival)
                .then(|| entry_clear_of_own(&end, ctx.own, &mut rng))
                .flatten();
            Some(Movement {
                t_s: r.t_s,
                callsign: r.callsign.trim().to_string(),
                spoken: r
                    .spoken
                    .clone()
                    .unwrap_or_else(|| r.callsign.trim().to_string()),
                icao_type: r.icao_type.trim().to_ascii_uppercase(),
                kind: r.kind,
                runway: end.ident.clone(),
                stand: r.stand.clone(),
                entry,
                straight_in: field.procs.approach_for(&end.ident).is_some(),
                laps: r.laps.unwrap_or(3),
                sid: None,
                star: None,
                exit_bearing: rng.range_f64(0.0, 360.0),
                seed: rng.next_u64(),
                parked_from_start: false,
            })
        })
        .collect();
    out.sort_by(|a, b| a.t_s.total_cmp(&b.t_s));
    out
}

#[cfg(test)]
mod tests;
