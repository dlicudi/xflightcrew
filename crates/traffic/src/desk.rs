//! A field's runway desk with no station (multi-field traffic stage 3,
//! `docs/tasks/multi-field-traffic/DESIGN.md`, 2026-10-13): the fleets at
//! the fields in range other than the pilot's station are worked here, as
//! data, silently. The station is ONE installed station with process-wide
//! strips (`party_line`), so a second field's traffic is kept out of it
//! altogether; this desk answers every call the obvious way and keeps the
//! ONE rule a runway needs: one landing or departure on it at a time.
//!
//! Its claims are COMPUTED from the fleet each tick (as the station's
//! runway book is), never stored: a departure cleared for takeoff while it
//! is on the ground, a landing cleared from its final until it is off the
//! pavement, anybody standing on the runway, a crossing just cleared. It
//! clears, in the order asked, the first landing, then the first takeoff
//! (only with no arrival inside [`ARRIVAL_CLEAR_S`] of the threshold and
//! [`DEPART_GAP_S`] after the last), then crossings; a crossing of another
//! strip is cleared at once. No readbacks, no radio: nothing is said.
//! Stage 4 replaces it with the tower FSM per strip.

use std::collections::BTreeMap;

use crate::field::FieldModel;
use crate::fleet::Fleet;
use crate::phase::{Call, Event, Order, Phase, Plan, TowerState};
use crate::TrafficAircraft;

/// Seconds between two takeoff clearances: the station's one departure on
/// the runway at a time (`WAKE_SECS`, 120).
pub const DEPART_GAP_S: f64 = 120.0;

/// No takeoff or crossing is cleared with an arrival this close to its
/// threshold, seconds: about the station's 3 nm release gap
/// (`DEPART_GAP_NM`) at an airliner's approach speed.
pub const ARRIVAL_CLEAR_S: f64 = 90.0;

/// A crossing holds the runway this long after it is cleared, seconds,
/// and its clearance lapses after twice that (a crossing takes well under
/// a minute at taxi pace).
pub const CROSSING_HOLD_S: f64 = 60.0;

/// The runway desk of one field (module docs). A value the link owns.
#[derive(Debug, Clone, Default)]
pub struct FieldDesk {
    /// The runway in use: every flight is cleared for it.
    runway: Option<String>,
    orders: BTreeMap<u32, Order>,
    /// Asked, in the order asked: a takeoff (ready at the hold), a landing
    /// (on final), a crossing (and of which runway).
    want_takeoff: Vec<u32>,
    want_landing: Vec<u32>,
    want_crossing: Vec<(u32, String)>,
    /// Fleet second each crossing was cleared.
    crossed_at: BTreeMap<u32, f64>,
    /// Fleet second of the last takeoff clearance.
    last_takeoff_s: Option<f64>,
}

impl FieldDesk {
    /// A desk clearing everyone for `runway` (the field's default; `None`
    /// for a field with no runway, where nobody flies).
    pub fn new(runway: Option<String>) -> FieldDesk {
        FieldDesk {
            runway: runway.filter(|r| !r.is_empty()),
            ..FieldDesk::default()
        }
    }

    pub fn runway(&self) -> Option<&str> {
        self.runway.as_deref()
    }

    /// Flight `id`'s order, level-triggered.
    pub fn order(&self, id: u32) -> Option<Order> {
        self.orders.get(&id).cloned()
    }

    fn entry(&mut self, id: u32) -> &mut Order {
        let runway = self.runway.clone();
        self.orders.entry(id).or_insert_with(|| Order {
            runway,
            ..Order::default()
        })
    }

    /// A fleet taken over mid-flight (its station uninstalled under it):
    /// every flight's order as its phase says it was cleared, the ones
    /// waiting on the runway queued. A call already made is never made
    /// again, so nothing would ask.
    pub fn adopt(&mut self, fleet: &Fleet) {
        for a in fleet.aircraft().iter().filter(|a| !a.dormant()) {
            let state = match a.phase {
                Phase::Parked if matches!(a.plan, Plan::Arrival { .. }) => continue,
                Phase::Parked | Phase::TaxiOut | Phase::HoldShort => TowerState::TaxiCleared,
                Phase::Vacate | Phase::TaxiIn => TowerState::TaxiCleared,
                Phase::LineUp | Phase::Roll | Phase::Climb | Phase::Leave => {
                    TowerState::TakeoffCleared
                }
                Phase::Landing => TowerState::ClearedToLand,
                Phase::Arrive | Phase::Pattern { .. } | Phase::Final | Phase::GoAround => {
                    TowerState::Inbound
                }
                Phase::Gone => continue,
            };
            self.entry(a.id).state = state;
            match a.phase {
                Phase::HoldShort => self.want_takeoff.push(a.id),
                Phase::Final => self.want_landing.push(a.id),
                _ => {}
            }
        }
    }

    /// The fleet's calls this step, answered; a flight gone is forgotten.
    pub fn hear(&mut self, events: &[Event]) {
        for ev in events {
            match ev {
                Event::Call { id, call } => self.call(*id, call),
                Event::Despawn { id, .. } | Event::TurnedRound { id } => self.forget(*id),
                _ => {}
            }
        }
    }

    fn forget(&mut self, id: u32) {
        self.orders.remove(&id);
        self.crossed_at.remove(&id);
        self.want_takeoff.retain(|i| *i != id);
        self.want_landing.retain(|i| *i != id);
        self.want_crossing.retain(|(i, _)| *i != id);
    }

    fn call(&mut self, id: u32, call: &Call) {
        let queue = |v: &mut Vec<u32>| {
            if !v.contains(&id) {
                v.push(id);
            }
        };
        match call {
            Call::Taxi | Call::Vacated => self.entry(id).state = TowerState::TaxiCleared,
            Call::PushStart { .. } => self.entry(id).push_approved = true,
            Call::Ready => queue(&mut self.want_takeoff),
            Call::Final
            | Call::Established {
                approach: false, ..
            } => {
                self.entry(id).state = TowerState::Inbound;
                queue(&mut self.want_landing);
            }
            Call::GoingAround => {
                self.want_landing.retain(|i| *i != id);
                let o = self.entry(id);
                o.state = TowerState::Inbound;
                o.go_around = false;
            }
            Call::Inbound { .. }
            | Call::Downwind
            | Call::Base
            | Call::ArrivalCheckIn { .. }
            | Call::MissedCheckIn { .. }
            | Call::Established { .. } => {
                // A circuit's next lap is cleared afresh on its final.
                self.entry(id).state = TowerState::Inbound;
            }
            Call::RequestCrossing { runway } => {
                if !self.want_crossing.iter().any(|(i, _)| *i == id) {
                    self.want_crossing.push((id, runway.clone()));
                }
            }
            Call::Readback(_) | Call::SayAgain => {}
        }
    }

    /// The runway's claims now (module docs), and the clearances they let
    /// through: the first landing asked, else the first takeoff, then the
    /// crossings.
    pub fn tick(&mut self, fleet: &Fleet) {
        let now = fleet.clock.now_s();
        let field = &fleet.field;
        let strip = self.runway.as_deref().and_then(|r| strip_index(field, r));
        // A crossing's clearance lapses once it is long crossed.
        let lapsed: Vec<u32> = self
            .crossed_at
            .iter()
            .filter(|(_, at)| now - **at > 2.0 * CROSSING_HOLD_S)
            .map(|(id, _)| *id)
            .collect();
        for id in lapsed {
            self.crossed_at.remove(&id);
            if let Some(o) = self.orders.get_mut(&id) {
                o.cross_cleared = None;
            }
        }
        let claim = |desk: &FieldDesk, except: u32| desk.claimed(fleet, strip, now, except);
        if let Some(&id) = self.want_landing.first() {
            if !claim(self, id) {
                self.want_landing.remove(0);
                self.entry(id).state = TowerState::ClearedToLand;
            }
        }
        if let Some(&id) = self.want_takeoff.first() {
            let at_hold = fleet.get(id).is_some_and(|a| a.phase == Phase::HoldShort);
            let gap = self.last_takeoff_s.is_none_or(|t| now - t >= DEPART_GAP_S);
            if at_hold && gap && !claim(self, id) && !arrival_near(fleet, id) {
                self.want_takeoff.remove(0);
                self.last_takeoff_s = Some(now);
                self.entry(id).state = TowerState::TakeoffCleared;
            }
        }
        let mut k = 0;
        while k < self.want_crossing.len() {
            let (id, rwy) = self.want_crossing[k].clone();
            let ours = strip.is_some() && strip_index(field, &rwy) == strip;
            if !ours || (!claim(self, id) && !arrival_near(fleet, id)) {
                self.want_crossing.remove(k);
                self.crossed_at.insert(id, now);
                self.entry(id).cross_cleared = Some(rwy);
                // One crossing of our strip at a time.
                if ours {
                    break;
                }
                continue;
            }
            k += 1;
        }
    }

    /// Somebody other than `except` holds the runway (`strip`): a takeoff
    /// cleared and still on the ground, a landing cleared and not yet off
    /// the pavement, a crossing just cleared, anybody on it.
    fn claimed(&self, fleet: &Fleet, strip: Option<usize>, now: f64, except: u32) -> bool {
        let field = &fleet.field;
        let on_strip = |a: &TrafficAircraft| {
            a.kin.on_ground
                && field
                    .strip_at(a.kin.pos())
                    .is_some_and(|(i, _)| Some(i) == strip)
        };
        fleet
            .aircraft()
            .iter()
            .filter(|a| a.id != except && !a.dormant())
            .any(|a| {
                let state = self.orders.get(&a.id).map(|o| o.state);
                let crossing = self
                    .crossed_at
                    .get(&a.id)
                    .is_some_and(|t| now - t < CROSSING_HOLD_S);
                on_strip(a)
                    || (state == Some(TowerState::TakeoffCleared) && a.kin.on_ground)
                    || (state == Some(TowerState::ClearedToLand)
                        && matches!(a.phase, Phase::Final | Phase::Landing))
                    || crossing
            })
    }
}

/// The index of `ident`'s strip among the field's runways.
fn strip_index(field: &FieldModel, ident: &str) -> Option<usize> {
    let s = field.strip(ident)?;
    field
        .airport
        .runways
        .iter()
        .position(|r| std::ptr::eq(r, s))
}

/// An arrival other than `except` is airborne within [`ARRIVAL_CLEAR_S`]
/// of its threshold.
fn arrival_near(fleet: &Fleet, except: u32) -> bool {
    fleet.aircraft().iter().any(|a| {
        a.id != except
            && !a.kin.on_ground
            && matches!(
                a.phase,
                Phase::Arrive | Phase::Pattern { .. } | Phase::Final | Phase::Landing
            )
            && a.slot().is_some_and(|s| s.to_thr_s < ARRIVAL_CLEAR_S)
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::field::fixtures;

    fn ev(id: u32, call: Call) -> Event {
        Event::Call { id, call }
    }

    /// THE OBVIOUS ANSWERS (stage 3): a taxi request is cleared to the
    /// desk's runway, a vacate to the stand, a go-around back to inbound,
    /// and a flight gone is forgotten.
    #[test]
    fn a_desk_answers_the_obvious_way() {
        let mut d = FieldDesk::new(Some("09".into()));
        d.hear(&[ev(1, Call::Taxi), ev(2, Call::Final)]);
        let o = d.order(1).unwrap();
        assert_eq!(o.state, TowerState::TaxiCleared);
        assert_eq!(o.runway.as_deref(), Some("09"));
        assert!(o.readback_owed.is_none(), "no radio, no readbacks");
        assert_eq!(d.order(2).unwrap().state, TowerState::Inbound);
        d.hear(&[ev(2, Call::GoingAround)]);
        assert_eq!(d.order(2).unwrap().state, TowerState::Inbound);
        assert!(d.want_landing.is_empty());
        d.hear(&[Event::Despawn {
            id: 1,
            reason: crate::phase::Despawn::LeftTheArea,
        }]);
        assert!(d.order(1).is_none());
    }

    /// ONE ON THE RUNWAY AT A TIME (stage 3): with an aircraft on the
    /// strip, nobody is cleared to land or take off; once it is clear the
    /// landing asked first is cleared, and the takeoff only once that
    /// landing is down and off the pavement.
    #[test]
    fn one_on_the_runway_at_a_time() {
        let field = fixtures::xtst();
        let end = field.end("09").unwrap();
        let mut fleet = Fleet::new(field.clone());
        // A Cessna standing on the runway, and one at a stand.
        let mut on = TrafficAircraft::at_stand(
            1,
            "N1",
            "November one",
            "C172",
            Plan::Circuit {
                runway: "09".into(),
                laps: 1,
                left: true,
            },
            &field,
            &field.net.stands[0].name,
            1,
        )
        .unwrap();
        on.phase = Phase::LineUp;
        let p = end.at(300.0, 0.0);
        on.kin.lat = p.0;
        on.kin.lon = p.1;
        let mut held = on.clone();
        held.id = 2;
        held.phase = Phase::HoldShort;
        let s = &field.net.stands[1];
        held.kin.lat = s.lat;
        held.kin.lon = s.lon;
        assert!(fleet.spawn(on) && fleet.spawn(held));
        let mut d = FieldDesk::new(Some("09".into()));
        d.hear(&[ev(2, Call::Ready)]);
        d.tick(&fleet);
        assert_ne!(
            d.order(2).map(|o| o.state),
            Some(TowerState::TakeoffCleared)
        );
        // The runway clears: the takeoff goes.
        fleet.remove(1);
        d.tick(&fleet);
        assert_eq!(d.order(2).unwrap().state, TowerState::TakeoffCleared);
        // A landing asked now waits while that takeoff is on the ground.
        d.hear(&[ev(3, Call::Final)]);
        d.tick(&fleet);
        assert_eq!(d.order(3).unwrap().state, TowerState::Inbound);
        fleet.remove(2);
        d.tick(&fleet);
        assert_eq!(d.order(3).unwrap().state, TowerState::ClearedToLand);
    }
}
