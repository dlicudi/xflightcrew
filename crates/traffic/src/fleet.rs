//! The fleet: every traffic aircraft at one field, stepped together on
//! the sim-time clock. A value the bin owns; never a static.

use crate::aircraft::{Seen, TrafficAircraft};
use crate::clock::{SimClock, SUBSTEP_S};
use crate::field::FieldModel;
use crate::phase::{Despawn, Event, Order, OwnTarget, Phase};

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
}

impl Fleet {
    pub fn new(field: FieldModel) -> Fleet {
        Fleet {
            field,
            clock: SimClock::new(),
            aircraft: Vec::new(),
            pending: Vec::new(),
            own: None,
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
        self.aircraft
            .iter_mut()
            .find(|a| a.id == id)
            .is_some_and(|a| a.release(plan))
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

    /// Everyone's id, phase and position.
    pub fn picture(&self) -> Vec<Seen> {
        self.aircraft
            .iter()
            .map(|a| Seen {
                id: a.id,
                phase: a.phase,
                pos: a.kin.pos(),
                gs_kts: a.kin.gs_kts,
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
            let picture = self.picture();
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
        events
    }
}
