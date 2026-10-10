//! The fleet: every traffic aircraft at one field, stepped together on
//! the sim-time clock. A value the bin owns; never a static.

use crate::aircraft::{Seen, TrafficAircraft};
use crate::clock::{SimClock, SUBSTEP_S};
use crate::field::FieldModel;
use crate::kin::LatLon;
use crate::phase::{Despawn, Event, Order, OwnTarget, Phase, Plan};
use crate::rng::XorShift64;
use std::collections::HashMap;

/// Our own aircraft's id in the picture the fleet's aircraft see: no
/// traffic id (those count up from 1).
const OWN_ID: u32 = u32::MAX;

/// The id of a stand an arrival is taxiing in to, in the picture: no
/// aircraft stands there yet, but the stand is taken.
const CLAIM_ID: u32 = u32::MAX - 1;

/// The traffic at one field.
#[derive(Debug, Clone)]
pub struct Fleet {
    pub field: FieldModel,
    pub clock: SimClock,
    aircraft: Vec<TrafficAircraft>,
    /// Events raised between steps (spawns, removals), handed out by the
    /// next step.
    pending: Vec<Event>,
    /// Our own aircraft as last seen.
    own: Option<OwnTarget>,
    /// TURNAROUNDS (realism review, 2026-10-09): an arrival that parks
    /// stays on its stand, dark, and flies again as a departure once it
    /// has turned round ([`Fleet::ready_to_leave`]), where it vanished
    /// [`crate::aircraft::PARKED_LINGER_S`] after parking and every later
    /// departure appeared out of nothing on its stand. The bin's link turns
    /// it on; off, a finished flight lingers and is gone as before (the
    /// phase machine's own tests fly that way).
    pub turn_round: bool,
    /// When each turned-round aircraft is ready to fly again, fleet
    /// seconds. A dormant aircraft with no entry (parked from the start)
    /// is ready now.
    ready_at: HashMap<u32, f64>,
    /// Flights parked at the end of their flight, waiting for the way past
    /// their stand to clear before they turn round.
    finished: Vec<u32>,
}

/// The margin kept between a parked aircraft's wing and a taxiing one's,
/// metres ([`Fleet::way_runs_past`]).
const WAY_MARGIN_M: f64 = 10.0;

impl Fleet {
    pub fn new(field: FieldModel) -> Fleet {
        Fleet {
            field,
            clock: SimClock::new(),
            aircraft: Vec::new(),
            pending: Vec::new(),
            own: None,
            turn_round: false,
            ready_at: HashMap::new(),
            finished: Vec::new(),
        }
    }

    /// Add an aircraft; its `Spawn` comes out of the next step. An id
    /// already flying is refused (false).
    pub fn spawn(&mut self, ac: TrafficAircraft) -> bool {
        if self.get(ac.id).is_some() {
            return false;
        }
        self.pending.push(Event::Spawn { id: ac.id });
        self.aircraft.push(ac);
        true
    }

    /// Take an aircraft out now (a reinstall, a schedule change).
    pub fn remove(&mut self, id: u32) -> Option<TrafficAircraft> {
        let i = self.aircraft.iter().position(|a| a.id == id)?;
        self.ready_at.remove(&id);
        self.pending.push(Event::Despawn {
            id,
            reason: Despawn::Removed,
        });
        Some(self.aircraft.remove(i))
    }

    /// Give a dormant aircraft (parked, [`crate::Plan::Parked`]) its
    /// flight: it starts up where it stands
    /// ([`TrafficAircraft::release`]). False when `id` is not here or not
    /// dormant.
    pub fn release(&mut self, id: u32, plan: crate::phase::Plan) -> bool {
        let done = self
            .aircraft
            .iter_mut()
            .find(|a| a.id == id)
            .is_some_and(|a| a.release(plan));
        if done {
            self.ready_at.remove(&id);
        }
        done
    }

    /// [`Fleet::release`] under its next flight's callsign: a turned-round
    /// airframe leaves as the same carrier's next flight, never under the
    /// number it came in as. False when `id` is not here or not dormant
    /// (nothing changes).
    pub fn release_as(&mut self, id: u32, plan: Plan, callsign: &str, spoken: &str) -> bool {
        let Some(a) = self.aircraft.iter_mut().find(|a| a.id == id) else {
            return false;
        };
        if !a.dormant() {
            return false;
        }
        let (old_cs, old_spoken) = (a.callsign.clone(), a.spoken.clone());
        a.callsign = callsign.to_string();
        a.spoken = spoken.to_string();
        if !self.release(id, plan) {
            if let Some(a) = self.aircraft.iter_mut().find(|a| a.id == id) {
                a.callsign = old_cs;
                a.spoken = old_spoken;
            }
            return false;
        }
        true
    }

    /// The dormant airframe that has stood longest ready to fly, not among
    /// `held` (a departure is already waiting on it) and that `fits` (its
    /// class, the runway in use): a departure takes it rather than appear
    /// out of nothing. `None` when nobody is ready: a turned-round aircraft
    /// waits its turnaround ([`crate::schedule::turnaround_s`]), one parked
    /// from the start is ready at once.
    pub fn ready_to_leave(
        &self,
        held: &[u32],
        fits: &dyn Fn(&TrafficAircraft) -> bool,
    ) -> Option<u32> {
        let now = self.clock.now_s();
        self.aircraft
            .iter()
            .filter(|a| a.dormant() && a.phase == Phase::Parked && !held.contains(&a.id))
            .map(|a| (a, self.ready_at.get(&a.id).copied().unwrap_or(0.0)))
            .filter(|(a, at)| *at <= now && fits(a))
            .min_by(|x, y| x.1.total_cmp(&y.1).then(x.0.id.cmp(&y.0.id)))
            .map(|(a, _)| a.id)
    }

    /// Dormant airframes `fits` keeps, not among `held`, ready now or by
    /// `by_s` fleet seconds: how many of a coming hour's departures can
    /// take a parked airframe ([`crate::SpawnCtx::parked_airframes`]).
    pub fn airframes_ready_by(
        &self,
        by_s: f64,
        held: &[u32],
        fits: &dyn Fn(&TrafficAircraft) -> bool,
    ) -> usize {
        self.aircraft
            .iter()
            .filter(|a| a.dormant() && a.phase == Phase::Parked && !held.contains(&a.id))
            .filter(|a| self.ready_at.get(&a.id).copied().unwrap_or(0.0) <= by_s && fits(a))
            .count()
    }

    /// Whether `id` came in as an arrival (or a circuit) of this fleet and
    /// turned round on its stand ([`Fleet::turn_round`]), and is still
    /// waiting to fly again.
    pub fn turned_round(&self, id: u32) -> bool {
        self.ready_at.contains_key(&id)
    }

    /// Every flight that finished on its stand this step stays there,
    /// dormant ([`Plan::Parked`]), ready again after its turnaround: a
    /// fresh aircraft of the same identity standing exactly where it
    /// stopped, so the phase machine's finished flight (its linger, its
    /// despawn) is over and nothing in `aircraft.rs` needs to know.
    fn turn_round_parked(&mut self, events: &mut Vec<Event>) {
        self.finished.extend(events.iter().filter_map(|e| match e {
            Event::Phase {
                id,
                to: Phase::Parked,
                ..
            } => Some(*id),
            _ => None,
        }));
        let now = self.clock.now_s();
        let mut waiting = Vec::new();
        for id in std::mem::take(&mut self.finished) {
            let Some(i) = self.aircraft.iter().position(|a| a.id == id) else {
                continue;
            };
            let a = &self.aircraft[i];
            if a.phase != Phase::Parked
                || !matches!(a.plan, Plan::Arrival { .. } | Plan::Circuit { .. })
            {
                continue;
            }
            let Some(stand) = a.stand.clone() else {
                continue;
            };
            // Only a stand clear of every lane keeps an aircraft for its
            // turnaround, as only such a stand is parked on at the start:
            // EGKK (the hour sweep, 2026-10-10) had seven arrivals towed off
            // stands the departures' way out ran past, where before they
            // lingered two minutes and went. Elsewhere the flight lingers
            // and is gone as before.
            // Nor a stand another is drawn within its wingspan of: a
            // neighbour coming or going passes through its wing (EFUT,
            // USDB, USRK, the hour sweep 2026-10-10, 18 m apart).
            let clear = self.field.stand(&stand).is_some_and(|s| {
                self.field.lane_clearance_m((s.lat, s.lon))
                    >= crate::schedule::parked_lane_clear_m(&a.icao_type)
                    && self.field.net.stands.iter().all(|o| {
                        o.name == s.name
                            || crate::kin::dist_m((o.lat, o.lon), (s.lat, s.lon)) >= a.perf.span_m
                    })
            });
            // Nor where it would leave the arrivals still to come fewer than
            // [`crate::schedule::STANDS_KEPT_FREE`] free stands: XTST's few
            // filled with turned-round aircraft and the next arrival, with
            // nowhere to park, was taken away (2026-10-10).
            // Counted as stands an airliner fits (BIEG, 2026-10-10: a
            // Cessna turned round on one of its few jet stands, the free
            // ones left were the light aircraft's, and the next 737 had
            // nowhere to park).
            let held = self.stands_held();
            let airliner = if self
                .field
                .net
                .stands
                .iter()
                .any(|s| self.field.takes(s, "A320"))
            {
                "A320"
            } else {
                crate::schedule::REGIONAL
            };
            let free = self
                .field
                .net
                .stands
                .iter()
                .filter(|s| self.field.takes(s, airliner) && !crate::field::stand_taken(s, &held))
                .count();
            if !clear || free < crate::schedule::STANDS_KEPT_FREE {
                continue;
            }
            // Nor while a taxiing aircraft's way runs past the stand: it
            // waits, lingering, until the way is clear (gone at the end of
            // its linger if it never is). At EGKK a departure stuck behind
            // an arrival parked on its way out towed it off (2026-10-10).
            if self.way_runs_past(id, (a.kin.lat, a.kin.lon), a.perf.span_m) {
                waiting.push(id);
                continue;
            }
            let Some(mut fresh) = TrafficAircraft::at_stand(
                a.id,
                &a.callsign,
                &a.spoken,
                &a.icao_type,
                Plan::Parked,
                &self.field,
                &stand,
                a.seed,
            ) else {
                continue;
            };
            fresh.kin = a.kin.clone();
            fresh.kin.gs_kts = 0.0;
            fresh.kin.lights = Default::default();
            let mut rng = XorShift64::new(a.seed ^ 0x5455_524E);
            let ready = now + crate::schedule::turnaround_s(&a.icao_type, &mut rng);
            self.aircraft[i] = fresh;
            self.ready_at.insert(id, ready);
            events.push(Event::TurnedRound { id });
        }
        self.finished = waiting;
    }

    /// Some aircraft other than `id`, taxiing, has its way within a
    /// wingspan's reach of `at` (both half spans and a margin).
    fn way_runs_past(&self, id: u32, at: LatLon, span_m: f64) -> bool {
        self.aircraft.iter().any(|o| {
            o.id != id
                && o.kin.on_ground
                && matches!(
                    o.phase,
                    Phase::TaxiOut | Phase::TaxiIn | Phase::Vacate | Phase::HoldShort
                )
                && {
                    // The way still ahead of it, from where it is: the
                    // whole route held every arrival back behind ways long
                    // since driven.
                    let ahead: Vec<LatLon> = std::iter::once(o.kin.pos())
                        .chain(o.route.path.iter().skip(o.route.follow.seg + 1).copied())
                        .collect();
                    ahead.len() >= 2
                        && crate::kin::cross_track_m(&ahead, at)
                            < (span_m + o.perf.span_m) / 2.0 + WAY_MARGIN_M
                }
        })
    }

    pub fn aircraft(&self) -> &[TrafficAircraft] {
        &self.aircraft
    }

    pub fn get(&self, id: u32) -> Option<&TrafficAircraft> {
        self.aircraft.iter().find(|a| a.id == id)
    }

    /// Our own aircraft as last given to [`Fleet::step`].
    pub fn own(&self) -> Option<OwnTarget> {
        self.own
    }

    /// Where the fleet's aircraft stand on the ground, and the stands those
    /// taxiing in are bound for: nothing spawns, parks or is sent onto
    /// any of them. A stand was free until its arrival reached it, so a
    /// departure due there spawned on it and the arrival drove in on top
    /// (the hour sweep, 2026-10-08).
    pub fn stands_held(&self) -> Vec<LatLon> {
        self.aircraft
            .iter()
            .filter(|a| a.kin.on_ground)
            .map(|a| a.kin.pos())
            .chain(self.stands_claimed())
            .collect()
    }

    /// The stands the aircraft taxiing in are bound for.
    fn stands_claimed(&self) -> impl Iterator<Item = LatLon> + '_ {
        self.aircraft
            .iter()
            .filter(|a| a.phase == Phase::TaxiIn)
            .filter_map(|a| a.stand.as_deref().and_then(|n| self.field.stand(n)))
            .map(|s| (s.lat, s.lon))
    }

    /// Everyone's id, phase and position.
    pub fn picture(&self) -> Vec<Seen> {
        self.aircraft
            .iter()
            .map(|a| Seen {
                id: a.id,
                phase: a.phase,
                pos: a.kin.pos(),
                gs_kts: a.kin.gs_kts,
                slot: a.slot(),
            })
            .collect()
    }

    /// Advance by `dt` seconds of sim time (clamped and cut into fixed
    /// substeps by the clock; nothing while it is frozen). `orders` is the
    /// tower's level-triggered answer per aircraft, asked every substep.
    pub fn step(
        &mut self,
        dt: f64,
        own: Option<OwnTarget>,
        orders: &dyn Fn(u32) -> Option<Order>,
    ) -> Vec<Event> {
        self.own = own;
        let mut events = std::mem::take(&mut self.pending);
        let n = self.clock.advance(dt);
        for _ in 0..n {
            let mut picture = self.picture();
            // Our own aircraft on the ground takes a stand like a parked
            // one: an arrival never picks the stand we stand on.
            if let Some(o) = own.filter(|o| o.on_ground) {
                picture.push(Seen {
                    id: OWN_ID,
                    phase: Phase::Parked,
                    pos: o.pos,
                    gs_kts: 0.0,
                    slot: None,
                });
            }
            // A stand somebody is taxiing in to is taken already.
            let claimed: Vec<LatLon> = self.stands_claimed().collect();
            picture.extend(claimed.into_iter().map(|pos| Seen {
                id: CLAIM_ID,
                phase: Phase::Parked,
                pos,
                gs_kts: 0.0,
                slot: None,
            }));
            crate::spacing::apply(
                &mut self.aircraft,
                &self.field,
                own.as_ref(),
                SUBSTEP_S,
                &mut events,
            );
            for ac in &mut self.aircraft {
                let order = orders(ac.id);
                ac.substep(
                    SUBSTEP_S,
                    order.as_ref(),
                    &self.field,
                    &picture,
                    &mut events,
                );
            }
            self.aircraft.retain(|a| a.phase != Phase::Gone);
        }
        if self.turn_round {
            self.turn_round_parked(&mut events);
        }
        let ids: Vec<u32> = self.aircraft.iter().map(|a| a.id).collect();
        self.ready_at.retain(|id, _| ids.contains(id));
        events
    }
}
