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
use crate::airlines::{self, Carrier};
use crate::field::stand_fits;
use crate::field::{
    neighbour_taken, stand_taken, stand_width, tightest_first, FieldModel, RunwayEnd,
};
use crate::kin::{dist_m, project_m, LatLon};
use crate::perf::{
    class_for_type, runway_min_m, wingspan_code, JET_HEAVY, PISTON_SINGLE, TURBOPROP,
};
use crate::phase::Plan;
use crate::rate;
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

    /// The share of the field's own rate ([`crate::rate::peak_per_h`])
    /// flown: half at light, the whole at normal.
    pub fn scale(self) -> f64 {
        match self {
            Density::Off => 0.0,
            Density::Light => 0.5,
            Density::Normal => 1.0,
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
/// smaller one where no stand is wide enough for it (a Saab 340, code B):
/// what [`crate::rate::airline_stands`] counts a regional stand by. Which
/// carrier flies what is [`crate::airlines`]'.
pub const REGIONAL: &str = "AT76";
pub const SMALL_REGIONAL: &str = "SF34";

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
/// density ([`park_at_start`]). A real apron by day is about half full; at
/// 0.30 and a cap of 40 Gatwick's 195 stands stood four in five empty.
pub const PARKED_SHARE_LIGHT: f64 = 0.25;
pub const PARKED_SHARE_NORMAL: f64 = 0.45;

/// The share added at night, times how far the day has fallen from its
/// peak ([`crate::rate::day_factor`]): the aircraft night stopping stand on
/// their stands till the morning wave.
pub const PARKED_NIGHT_EXTRA: f64 = 0.20;

/// Stands left free at the start for the arrivals, whatever the share.
pub const STANDS_KEPT_FREE: usize = 3;

/// Nothing is parked this close to our own aircraft, metres.
pub const PARK_CLEAR_OWN_M: f64 = 60.0;

/// The most aircraft parked at the start, however big the field: each is an
/// XPMP2 aircraft the plugin draws every frame. Dormant ones are dark and
/// still, but each is still drawn; with Heathrow's peak of about 15 moving
/// at once the whole picture stays under 100 aircraft (frame cost
/// unmeasured past 60 aircraft, see the risks).
pub const PARKED_MAX: usize = 80;

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
    /// The flight as a board shows it (a route table's rotation, or a
    /// traffic file row that names its ends); `None` for a generated hour.
    pub flight: Option<FlightInfo>,
    /// Where it already is when the fleet starts (the mid-flow start,
    /// [`crate::midflow`]); `None` starts it the usual way at its time.
    pub start: Option<Start>,
}

/// A flight as a flight information board shows it.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct FlightInfo {
    /// The board's flight number ("U2 8901"); the callsign where the
    /// airline has no designator here.
    pub number: String,
    /// The airline's name ("easyJet"); empty for none.
    pub airline: String,
    /// The other end: its ICAO code and its name.
    pub other: String,
    pub city: String,
    /// The timetable's time and today's estimate, LOCAL seconds after the
    /// reference midnight (below 0 the evening before, past 86,400
    /// tomorrow); `None` where the file gave none.
    pub sched_s: Option<f64>,
    pub est_s: Option<f64>,
    /// A departure flown by the airframe that arrives as this callsign
    /// (its rotation's inbound): it leaves from wherever that one parked.
    pub inbound: Option<String>,
}

/// Where a movement already is at the fleet's start ([`crate::midflow`]).
#[derive(Debug, Clone, PartialEq)]
pub enum Start {
    /// On its instrument approach, this far along it from the threshold,
    /// metres.
    Approach { to_thr_m: f64 },
    /// Landed and taxiing in to `stand`, `frac` of the way from the runway.
    TaxiIn { stand: String, frac: f64 },
    /// Taxiing out from its stand, `frac` of the way to the holding point.
    TaxiOut { frac: f64 },
    /// Holding short of its runway, ready.
    HoldShort,
    /// Climbing out, this far past the runway's far end, metres.
    Climb { past_end_m: f64 },
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
    /// The sim's zulu time at the schedule's start, seconds after midnight
    /// (the bin reads `sim/time/zulu_time_sec`): the day's shape at the
    /// field's solar time ([`crate::rate`]). `None` flies the field's peak.
    pub zulu_s: Option<f64>,
    /// Airliners standing dormant on their stands, free to fly again
    /// (parked from the start, or turned round after landing): a later
    /// hour's departure may be scheduled with no stand of its own for as
    /// many of them, since it takes one of their airframes at its time
    /// ([`crate::Fleet::ready_to_leave`]). The bin counts them; 0 needs a
    /// free stand for every departure, as before.
    pub parked_airframes: usize,
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
        if let Some(start) = &self.start {
            return self.place(id, field, ctx, start);
        }
        let plan = self.plan(field)?;
        if let Plan::Arrival { entry, .. } = &plan {
            if ctx
                .own
                .is_some_and(|o| dist_m(o, *entry) < OWN_CLEAR_NM * 1852.0)
            {
                return Err(Refusal::NearOwn);
            }
            let ac = TrafficAircraft::at_entry(
                id,
                &self.callsign,
                &self.spoken,
                &self.icao_type,
                plan,
                field,
                self.seed,
            )
            .ok_or(Refusal::NoRunway)?;
            // An instrument arrival appears further out than its entry
            // (`crate::arrival::entry_point`): clear of us where it IS.
            if ctx
                .own
                .is_some_and(|o| dist_m(o, ac.kin.pos()) < OWN_CLEAR_NM * 1852.0)
            {
                return Err(Refusal::NearOwn);
            }
            return Ok(ac);
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
pub(crate) fn free_stands(field: &FieldModel, icao_type: &str, taken: &[LatLon]) -> Vec<String> {
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

/// The shortest runway an `icao_type` airliner is scheduled on: its own
/// field length ([`runway_min_m`]), or its class's good landing
/// ([`landing_distance_m`]) where that is longer. PABV's 1,220 m held an
/// ATR's good landing (1,150 m) and not the long ones that went off its
/// end; a 1,100 m strip did not hold even the good one.
pub fn runway_needed_m(icao_type: &str) -> f64 {
    runway_min_m(icao_type).max(landing_distance_m(class_for_type(icao_type).0))
}

/// How long `icao_type` stands on its stand between an arrival and its next
/// departure, seconds, drawn from `rng`: a narrowbody 35 to 60 minutes, a
/// turboprop 25 to 40, a widebody 90 to 150, the A380 two to three hours,
/// a light aircraft one to three (the airlines' scheduled minimum ground
/// times, and a little over: nobody turns round at the minimum all day).
pub fn turnaround_s(icao_type: &str, rng: &mut XorShift64) -> f64 {
    let p = class_for_type(icao_type).0;
    let (lo, hi) = if icao_type.eq_ignore_ascii_case("A388") {
        (120.0, 180.0)
    } else if p.name == JET_HEAVY.name {
        (90.0, 150.0)
    } else if p.name == TURBOPROP.name {
        (25.0, 40.0)
    } else if p.name == PISTON_SINGLE.name {
        (60.0, 180.0)
    } else {
        (35.0, 60.0)
    };
    rng.range_f64(lo, hi) * 60.0
}

/// The next flight of the airframe flying `callsign` at `icao`: the same
/// carrier ([`airlines::carrier_of`]), a new number from its range, never
/// the one it came in as; `None` for a registration (a light aircraft
/// keeps its own) or a carrier no table names.
pub fn next_flight(icao: &str, callsign: &str, rng: &mut XorShift64) -> Option<(String, String)> {
    let carrier = airlines::carrier_of(icao, callsign)?;
    (0..8)
        .map(|_| airlines::flight(&carrier, rng))
        .find(|(cs, _)| cs != callsign)
}

/// Some stand at `field` takes an `icao_type` ([`FieldModel::takes`]): the
/// type can come and go there at all.
fn parks_at(field: &FieldModel, icao_type: &str) -> bool {
    field.net.stands.iter().any(|s| field.takes(s, icao_type))
}

/// A seed for the field: the same field and seed, the same schedule.
pub(crate) fn field_seed(icao: &str, seed: u64) -> u64 {
    icao.bytes().fold(seed ^ 0xCBF2_9CE4_8422_2325, |h, b| {
        (h ^ b as u64).wrapping_mul(0x0100_0000_01B3)
    })
}

/// Generate an hour ([`HORIZON_S`]) of traffic for `field`:
/// - 1 (light) or 1 to 3 (normal) circuit aircraft where the field allows
///   circuits (`allows_circuits != Some(false)`) and has stands, by day
///   ([`rate::circuit_hours`]) when `ctx` has the time;
/// - IFR departures by SID where the field publishes SIDs, and IFR
///   arrivals (straight in where the runway has an approach, else to the
///   downwind) where it publishes approaches or STARs, at the field's own
///   rate ([`rate::rates_at`]: its airline stands and tower, the time of
///   day at its solar time, half of it at light, each kind capped by the
///   one runway), on a runway long enough for the type;
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

    // Circuits by day only: a school does not fly the pattern at 03:00.
    let daylight = ctx
        .zulu_s
        .is_none_or(|z| rate::circuit_hours(rate::local_solar_h(z, rate::field_lon(field))));
    if field.airport.allows_circuits != Some(false) && daylight {
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
                flight: None,
                start: None,
            });
            t += rng.range_f64(120.0, 420.0);
        }
    }

    // The field's carriers and the types each flies there
    // ([`airlines::candidates`]), of those the runway takes and a stand at
    // the field fits: an airliner is never scheduled where it has nowhere to
    // park. Where no stand is wide enough for the ATR (code C) the regional
    // is the Saab (code B): UNSS's ten stands and ULKK's are all B.
    let pool = airline_pool(field, end.length_m);
    let mut used: std::collections::HashSet<String> = Default::default();
    let mut ifr = |rng: &mut XorShift64| -> Option<(String, String, &'static str)> {
        let (carrier, icao_type, _) = *airlines::draw(&pool, rng)?;
        let (callsign, spoken) = unique_flight(&carrier, rng, &mut used);
        Some((callsign, spoken, icao_type))
    };
    // A later hour's departures may outnumber the free stands: as many as
    // there are airframes parked on theirs are scheduled with no stand, and
    // take one of those at their time (`ctx.parked_airframes`).
    let mut standless = ctx.parked_airframes;
    // The field's own rate through the day ([`crate::rate::rates_at`]),
    // capped by the one runway everyone uses.
    let scale = density.scale();
    let zulu = ctx.zulu_s;
    let rates = |t: f64| rate::rates_at(field, scale, zulu, t);
    let has_stands = field.net.stands.iter().any(|s| s.is_suitable());

    // A field that publishes no procedures still has its airline traffic,
    // flown visually: a departure off the runway heading and away, an
    // arrival joining the circuit. LXGB (2026-10-09): Gibraltar's CIFP
    // holds runways only and its stands take jets, so the hour was empty
    // and the pilot sat at the field with no traffic at all.
    let visual = field.procs.sids.is_empty()
        && field.procs.approaches.is_empty()
        && field.procs.stars.is_empty();
    // Neither stream starts inside the first minute: the first ground
    // start is brought there ([`bring_first_forward`]) and no arrival
    // comes ahead of it.
    if (!field.procs.sids.is_empty() || visual) && has_stands {
        let phase = rng.range_f64(0.0, 1.0);
        let times = {
            let mut draw = || rng.range_f64(0.8, 1.2);
            rate::stream_times(HORIZON_S, FIRST_MOVE_S.1, phase, |t| rates(t).0, &mut draw)
        };
        for t in times {
            let Some((callsign, spoken, icao_type)) = ifr(&mut rng) else {
                break;
            };
            let stand = match take_stand(icao_type, &mut rng, &mut taken) {
                Some(s) => Some(s),
                None if standless > 0 => {
                    standless -= 1;
                    None
                }
                None => break,
            };
            let sid = (!field.procs.sids.is_empty()).then(|| {
                field.procs.sids[rng.below(field.procs.sids.len() as u64) as usize].clone()
            });
            out.push(Movement {
                t_s: t,
                callsign,
                spoken,
                icao_type: icao_type.into(),
                kind: MovementKind::Departure,
                runway: end.ident.clone(),
                stand,
                entry: None,
                straight_in: false,
                laps: 0,
                sid,
                star: None,
                exit_bearing: rng.range_f64(0.0, 360.0),
                seed: rng.next_u64(),
                parked_from_start: false,
                flight: None,
                start: None,
            });
        }
    }

    let arrivals = !field.procs.approaches.is_empty() || !field.procs.stars.is_empty() || visual;
    if arrivals && has_stands {
        let straight_in = field.procs.approach_for(&end.ident).is_some();
        let phase = rng.range_f64(0.0, 0.7);
        let times = {
            let mut draw = || rng.range_f64(0.8, 1.2);
            rate::stream_times(HORIZON_S, FIRST_MOVE_S.1, phase, |t| rates(t).1, &mut draw)
        };
        for t in times {
            let Some((callsign, spoken, icao_type)) = ifr(&mut rng) else {
                break;
            };
            let Some(entry) = entry_clear_of_own(&end, ctx.own, &mut rng) else {
                continue;
            };
            let star = (!field.procs.stars.is_empty()).then(|| {
                field.procs.stars[rng.below(field.procs.stars.len() as u64) as usize].clone()
            });
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
                flight: None,
                start: None,
            });
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

/// A parked aircraft's stand is at least this far from any stand another
/// aircraft stands on at the start, centre to centre, metres: a narrowbody's
/// length less a margin, so no two jets are drawn wing in wing.
const PARK_SET_CLEAR_M: f64 = 30.0;

/// A jet is parked no nearer a stand a flight of the hour leaves from than
/// this, metres: two narrowbody stands and their wingtip gaps.
const JET_BESIDE_MOVER_M: f64 = 80.0;

/// A parked aircraft's stand is at least this far from any lane others
/// taxi along, metres (half a medium jet's span and a margin).
const PARKED_LANE_CLEAR_M: f64 = 25.0;

/// ...and a wider type's half its own span and the same margin: a 777
/// parked 25 m off a lane put its wing over it (EGLL, the hour sweep
/// 2026-10-10, two heavies drawn 35 m apart as one taxied by).
pub(crate) fn parked_lane_clear_m(icao_type: &str) -> f64 {
    PARKED_LANE_CLEAR_M.max(crate::perf::span_of(icao_type) / 2.0 + 7.0)
}

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
    } + ctx.zulu_s.map_or(0.0, |z| {
        let h = rate::local_solar_h(z, rate::field_lon(field));
        PARKED_NIGHT_EXTRA * (1.0 - rate::day_factor(h))
    });
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
    // Nobody parked under a callsign the hour already flies.
    let mut used: std::collections::HashSet<String> =
        schedule.iter().map(|m| m.callsign.clone()).collect();
    let mut out = Vec::new();
    let mut held: Vec<LatLon> = taken.clone();
    let mut in_beside = false;
    while out.len() < n && !(free.is_empty() && beside.is_empty()) {
        // Beside a stand the hour's flights leave from only for a turboprop
        // or a light aircraft: a departure leaving its stand passes its
        // neighbour, and a jet's wing is in its way. EGKK (the hour sweep,
        // 2026-10-10): with jets where ATRs stood, five departures left
        // through the jet parked beside them.
        if free.is_empty() {
            std::mem::swap(&mut free, &mut beside);
            in_beside = true;
        }
        let stand = free.swap_remove(rng.below(free.len() as u64) as usize);
        // Never on a stand drawn within [`PARK_SET_CLEAR_M`] of one a
        // flight of the hour stands on, or one already parked on: apt.dat
        // draws alternative stands over each other ("64L", "64", "64R"),
        // and at EGKK five departures left their stand through a jet
        // parked 21 m away on the next one of the set (the hour sweep,
        // 2026-10-10, the regional ATRs that stood there before being too
        // short to show it).
        let at = (stand.lat, stand.lon);
        if held.iter().any(|p| dist_m(*p, at) < PARK_SET_CLEAR_M) {
            continue;
        }
        let Some((carrier, icao_type)) = parked_type(field, stand, longest, &mut rng) else {
            continue;
        };
        let jet =
            [crate::perf::JET_MED.name, JET_HEAVY.name].contains(&class_for_type(icao_type).0.name);
        // ...nor, a jet, within [`JET_BESIDE_MOVER_M`] of one: a departure's
        // way off its stand runs past the stands either side and beyond
        // (EGKK, the same sweep: 18.5 m from jets parked two stands away).
        if jet && (in_beside || movers.iter().any(|m| dist_m(*m, at) < JET_BESIDE_MOVER_M)) {
            continue;
        }
        held.push(at);
        let (callsign, spoken) = match carrier {
            Some(c) => unique_flight(&c, &mut rng, &mut used),
            None => registration(prefix, icao_type, &mut rng),
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
            flight: None,
            start: None,
        });
    }
    out
}

/// What stands parked on `stand`, and whose: an airliner of the field's
/// mix ([`airlines::candidates`]) at a stand that takes jets or turboprops
/// (a gate, where it names no class), never a type wider than the stand
/// ([`stand_fits`]) or one the field's longest runway (`longest_m`) is
/// too short for; else a light aircraft (no carrier) on a stand that takes
/// props or names nothing. The airliner's share of each carrier and type
/// is the field's own: the old one stand in five that also took turboprops
/// went to the regional, 27% ATRs on Gatwick's apron (2026-10-09). `None`
/// when it takes nothing this field flies.
fn parked_type(
    field: &FieldModel,
    stand: &Stand,
    longest_m: f64,
    rng: &mut XorShift64,
) -> Option<(Option<Carrier>, &'static str)> {
    let takes = |c: &str| stand.accepts.iter().any(|a| a == c);
    let lane_m = field.lane_clearance_m((stand.lat, stand.lon));
    let (airline, props) = if stand.accepts.is_empty() {
        let gate = stand.kind.eq_ignore_ascii_case("gate");
        (gate, !gate)
    } else {
        (
            takes("jets") || takes("heavy") || takes("turboprops"),
            takes("props"),
        )
    };
    // A light aircraft's code B stand that also takes turboprops is a light
    // aircraft's: LEMG's GA apron stood full of Air Nostrum Saabs (17 of 61
    // parked, the hour sweep 2026-10-10), the regional's understudy.
    let small = props && stand_width(stand) <= 'B';
    if airline && !small {
        let pool = airlines::candidates(&field.icao, &|t: &str| {
            stand_fits(stand, t)
                && field.room_for(stand, t)
                && longest_m >= runway_needed_m(t)
                && lane_m >= parked_lane_clear_m(t)
        });
        if let Some((carrier, icao_type, _)) = airlines::draw(&pool, rng) {
            return Some((Some(*carrier), icao_type));
        }
    }
    let light = ["C172", "C152", "PA28", "SR22"][rng.below(4) as usize];
    ((props || stand.accepts.is_empty()) && stand_width(stand) >= wingspan_code(light))
        .then_some((None, light))
}

/// Whether [`parked_type`] parks anything at all on `stand` (it draws only
/// among what would): the test's measure of a stand left empty.
#[cfg(test)]
pub(crate) fn parks_something(field: &FieldModel, stand: &Stand, longest_m: f64) -> bool {
    let mut rng = XorShift64::new(1);
    (0..64).any(|_| parked_type(field, stand, longest_m, &mut rng).is_some())
}

/// The carrier and type pairs `field` flies off a runway `runway_m` long
/// ([`airlines::candidates`]): each type only where the runway holds it
/// ([`runway_needed_m`]) and some stand takes it ([`FieldModel::takes`]).
fn airline_pool(field: &FieldModel, runway_m: f64) -> Vec<(Carrier, &'static str, f64)> {
    // Each type's stands looked at once: a carrier table names a type many
    // times, and Heathrow has some 280 stands.
    let parks = std::cell::RefCell::new(std::collections::HashMap::<String, bool>::new());
    let fits = |t: &str| {
        runway_m >= runway_needed_m(t)
            && *parks
                .borrow_mut()
                .entry(t.to_string())
                .or_insert_with(|| parks_at(field, t))
    };
    airlines::candidates(&field.icao, &fits)
}

/// An entry [`ENTRY_NM`] out, within 50 degrees of the extended
/// centreline, clear of us by [`OWN_CLEAR_NM`]; turned further round when
/// we are in the way, `None` if nowhere is clear.
pub(crate) fn entry_clear_of_own(
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

/// A flight of `carrier`'s ([`airlines::flight`]) no other aircraft of the
/// hour flies under (`used`, which it joins): two EZY8123s would share a
/// strip's callsign.
fn unique_flight(
    carrier: &Carrier,
    rng: &mut XorShift64,
    used: &mut std::collections::HashSet<String>,
) -> (String, String) {
    let mut f = airlines::flight(carrier, rng);
    for _ in 0..8 {
        if !used.contains(&f.0) {
            break;
        }
        f = airlines::flight(carrier, rng);
    }
    used.insert(f.0.clone());
    f
}

pub(crate) fn digit_word(c: char) -> &'static str {
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
///   origin: EGKK         # optional: an arrival's origin, for the board
///   destination: EGKK    # optional: a departure's destination
///   scheduled: "10:35"   # optional: the timetable's local time
/// ```
///
/// The board fields (2026-10-13) are optional: a file written before them
/// reads as it did.
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
    #[serde(default)]
    pub origin: Option<String>,
    #[serde(default)]
    pub destination: Option<String>,
    #[serde(default)]
    pub scheduled: Option<String>,
}

/// "10:35" as local seconds after midnight; `None` for anything else.
pub fn parse_hhmm(s: &str) -> Option<f64> {
    let (h, m) = s.trim().split_once(':')?;
    let (h, m): (u32, u32) = (h.parse().ok()?, m.parse().ok()?);
    (h < 24 && m < 60).then(|| f64::from(h * 3600 + m * 60))
}

/// A file row's board line: where it names an end or a time.
fn file_flight(r: &FileEntry) -> Option<FlightInfo> {
    let other = match r.kind {
        MovementKind::Arrival => r.origin.clone(),
        MovementKind::Departure => r.destination.clone(),
        _ => None,
    };
    let sched = r.scheduled.as_deref().and_then(parse_hhmm);
    if other.is_none() && sched.is_none() {
        return None;
    }
    let cs = r.callsign.trim();
    let (airline, iata) = cs.get(..3).map_or(("", ""), crate::routes::airline_name);
    let number = match cs.get(3..) {
        Some(n) if !iata.is_empty() => format!("{iata} {n}"),
        _ => cs.to_string(),
    };
    let other = other.unwrap_or_default().trim().to_ascii_uppercase();
    Some(FlightInfo {
        number,
        airline: airline.to_string(),
        city: other.clone(),
        other,
        sched_s: sched,
        est_s: sched,
        inbound: None,
    })
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
                flight: file_flight(r),
                start: None,
            })
        })
        .collect();
    out.sort_by(|a, b| a.t_s.total_cmp(&b.t_s));
    out
}

#[cfg(test)]
mod tests;
