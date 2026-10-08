//! The join of our own AI traffic (`crates/traffic`) and the station
//! (`atc::world`), stage D (`docs/tasks/pillars-audit/stage-d-traffic.md`).
//! Neither crate names the other; they meet here, and only here.
//!
//! Per monitor tick ([`Link::tick`]): the schedule spawns what is due, the
//! tower's orders are read level-triggered (`world::traffic_order`) and
//! mapped onto `traffic::Order`, the fleet steps on its sim-time clock,
//! every call it keys goes to the station as TEXT (`world::post_traffic_call`,
//! with the picture it saw), every aircraft's state is noted on its strip
//! (`world::note_traffic_state`) and its runway watch ticked
//! (`world::traffic_tick`, with the picture minus itself plus us), and a
//! despawn closes its strip (`world::retire_traffic`). In the command loop
//! ([`drain_calls`]): the traffic desks synced, then each call answered on
//! its own desk (`Station::answer_traffic`).
//!
//! The station never actuates: the fleet moves its own aircraft, and the
//! plugin draws them from the packed frames this side writes.

use std::collections::HashMap;
use std::time::Instant;

use traffic::{
    Call, Despawn, Event, FieldModel, Fleet, Movement, Order, OwnTarget, Phase, SpawnCtx,
    TowerState, TrafficAircraft,
};

use crate::party_line::{
    AircraftState, FlightId, Station, TrafficCall, TrafficReply, TrafficTarget,
};
use crate::world::{TcasTarget, TrafficOrder};

/// A refused spawn (no stand free, too near us) is tried again this much
/// later, seconds of fleet time.
const SPAWN_RETRY_S: f64 = 30.0;

/// A refused spawn is given up after this many tries.
const SPAWN_TRIES: u32 = 10;

/// Our own aircraft, as the link needs it: from the monitor's snapshot.
#[derive(Debug, Clone)]
pub(crate) struct OwnState {
    /// The raw callsign the tower's picture keys us by.
    pub(crate) callsign: String,
    pub(crate) lat: f64,
    pub(crate) lon: f64,
    /// Feet MSL.
    pub(crate) alt_ft: f64,
    pub(crate) on_ground: bool,
    pub(crate) vs_fpm: f64,
}

impl OwnState {
    fn target(&self) -> TrafficTarget {
        TrafficTarget {
            who: FlightId::Own,
            target: TcasTarget {
                callsign: self.callsign.clone(),
                lat: self.lat,
                lon: self.lon,
                ele_ft: self.alt_ft,
                on_ground: self.on_ground,
                vs_fpm: self.vs_fpm,
            },
        }
    }

    fn own_target(&self) -> OwnTarget {
        OwnTarget {
            pos: (self.lat, self.lon),
            alt_ft: self.alt_ft,
            on_ground: self.on_ground,
        }
    }
}

/// One traffic transmission, for the speech side: who keyed and the words.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Transmission {
    pub(crate) id: u32,
    pub(crate) callsign: String,
    pub(crate) text: String,
}

/// What one tick of the link produced.
#[derive(Debug, Default)]
pub(crate) struct TickOut {
    /// The tower's lines from the traffic flights' runway watches
    /// (`world::traffic_tick`), in order: said in the tower's voice.
    pub(crate) tower: Vec<(u32, String)>,
    /// What the traffic keyed this tick (posted to the station already).
    pub(crate) transmissions: Vec<Transmission>,
    /// The fleet's events, for the blackbox.
    pub(crate) events: Vec<Event>,
    /// Movements spawned this tick (id, callsign, kind) and refusals.
    pub(crate) spawned: Vec<(u32, String)>,
}

/// A scheduled movement not yet spawned.
#[derive(Debug, Clone)]
struct Due {
    m: Movement,
    /// Fleet seconds from which it may spawn.
    at_s: f64,
    tries: u32,
}

/// The fleet at the installed field, its schedule and its spawn book.
pub(crate) struct Link {
    fleet: Fleet,
    due: Vec<Due>,
    next_id: u32,
    /// The field's spoken name, for the calls' address ("Malaga Tower").
    tower_name: String,
    /// The runway the tower last cleared anyone for: every movement spawns
    /// for it (an airborne aircraft's route is planned once, for one
    /// runway), and an arrival waits for it ([`ARRIVAL_WAIT_S`]).
    tower_runway: Option<String>,
}

/// An arrival spawns for the tower's runway: with none known yet (nobody
/// has been cleared) it waits this long, then takes the field's default.
const ARRIVAL_WAIT_S: f64 = 120.0;

impl Link {
    /// The fleet at `field`, flying `schedule` (times from now).
    pub(crate) fn new(field: FieldModel, schedule: Vec<Movement>, tower_name: &str) -> Link {
        let mut due: Vec<Due> = schedule
            .into_iter()
            .map(|m| Due {
                at_s: m.t_s,
                m,
                tries: 0,
            })
            .collect();
        due.sort_by(|a, b| a.at_s.total_cmp(&b.at_s));
        Link {
            fleet: Fleet::new(field),
            due,
            next_id: 1,
            tower_name: tower_name.to_string(),
            tower_runway: None,
        }
    }

    pub(crate) fn fleet(&self) -> &Fleet {
        &self.fleet
    }

    /// Movements still to spawn.
    pub(crate) fn pending(&self) -> usize {
        self.due.len()
    }

    /// The field's ICAO.
    pub(crate) fn icao(&self) -> &str {
        &self.fleet.field.icao
    }

    /// Freeze or release the fleet's clock (replay, pause, time
    /// acceleration above 1x).
    pub(crate) fn set_frozen(&mut self, frozen: bool) {
        self.fleet.clock.frozen = frozen;
    }

    /// The picture the crew and the station see of the traffic: every
    /// aircraft as a TCAS target (ours not in it).
    pub(crate) fn targets(&self) -> Vec<TcasTarget> {
        self.fleet.aircraft().iter().map(tcas_target).collect()
    }

    /// One tick: spawn what is due, step the fleet `dt` seconds of sim time
    /// (clamped and substepped by its clock; nothing while frozen), and
    /// trade with the station. `now` is the station's clock for this tick.
    pub(crate) fn tick(&mut self, own: Option<&OwnState>, dt: f64, now: Instant) -> TickOut {
        let mut out = TickOut::default();
        self.spawn_due(own, &mut out);
        let orders: HashMap<u32, Order> = self
            .fleet
            .aircraft()
            .iter()
            .filter_map(|a| crate::world::traffic_order(a.id).map(|o| (a.id, map_order(&o))))
            .collect();
        if let Some(r) = orders.values().find_map(|o| o.runway.clone()) {
            self.tower_runway = Some(r);
        }
        let events = self.fleet.step(dt, own.map(OwnState::own_target), &|id| {
            orders.get(&id).cloned()
        });
        // Where everyone is first: a call's strip is open and placed before
        // the station answers it, and the book sees this tick's positions.
        for ac in self.fleet.aircraft() {
            crate::world::note_traffic_state(ac.id, aircraft_state(ac, now));
        }
        for ev in &events {
            match ev {
                Event::Call { id, call } => {
                    let Some(ac) = self.fleet.get(*id) else {
                        continue;
                    };
                    let order = orders.get(id);
                    // An ordered go-around is acknowledged by its readback
                    // ("Going around, ..."): the report would say it twice.
                    if *call == Call::GoingAround && order.is_some_and(|o| o.go_around) {
                        continue;
                    }
                    let runway = order.and_then(|o| o.runway.as_deref()).or(ac.plan.runway());
                    let text = self.call_text(&ac.spoken, call, runway);
                    crate::world::post_traffic_call(TrafficCall {
                        id: *id,
                        spoken: ac.spoken.clone(),
                        text: text.clone(),
                        state: aircraft_state(ac, now),
                        picture: self.picture_for(*id, own),
                    });
                    out.transmissions.push(Transmission {
                        id: *id,
                        callsign: ac.callsign.clone(),
                        text,
                    });
                }
                Event::Despawn { id, .. } => crate::world::retire_traffic(*id),
                _ => {}
            }
        }
        for ac in self.fleet.aircraft() {
            let picture = self.picture_for(ac.id, own);
            for line in crate::world::traffic_tick(ac.id, &picture, now) {
                out.tower.push((ac.id, line));
            }
        }
        out.events = events;
        out
    }

    /// Every aircraft not parked on a stand leaves the frequency (an
    /// install cleared the strips under them): taken out of the fleet,
    /// their ids returned. Parked aircraft stay (their strips reopen at the
    /// next note).
    pub(crate) fn despawn_moving(&mut self) -> Vec<u32> {
        let ids: Vec<u32> = self
            .fleet
            .aircraft()
            .iter()
            .filter(|a| a.phase != Phase::Parked)
            .map(|a| a.id)
            .collect();
        for id in &ids {
            self.fleet.remove(*id);
            crate::world::retire_traffic(*id);
        }
        ids
    }

    /// A call in words, addressed to the field's tower by name where it
    /// opens with "Tower".
    fn call_text(&self, spoken: &str, call: &Call, runway: Option<&str>) -> String {
        let text = call.text(spoken, runway);
        match text.strip_prefix("Tower, ") {
            Some(rest) if !self.tower_name.is_empty() => {
                format!("{} Tower, {rest}", self.tower_name)
            }
            _ => text,
        }
    }

    /// What traffic flight `id` sees round it: every other traffic flight,
    /// and us.
    fn picture_for(&self, id: u32, own: Option<&OwnState>) -> Vec<TrafficTarget> {
        self.fleet
            .aircraft()
            .iter()
            .filter(|a| a.id != id)
            .map(|a| TrafficTarget {
                who: FlightId::Traffic(a.id),
                target: tcas_target(a),
            })
            .chain(own.map(OwnState::target))
            .collect()
    }

    fn spawn_due(&mut self, own: Option<&OwnState>, out: &mut TickOut) {
        let now_s = self.fleet.clock.now_s();
        if self.fleet.clock.frozen || self.due.first().is_none_or(|d| d.at_s > now_s) {
            return;
        }
        let ctx = SpawnCtx {
            runway: None,
            own: own.map(|o| (o.lat, o.lon)),
            occupied: self
                .fleet
                .aircraft()
                .iter()
                .filter(|a| a.kin.on_ground)
                .map(|a| a.kin.pos())
                .collect(),
        };
        let mut later = Vec::new();
        while self.due.first().is_some_and(|d| d.at_s <= now_s) {
            let mut d = self.due.remove(0);
            match &self.tower_runway {
                Some(r) if self.fleet.field.end(r).is_some() => d.m.runway = r.clone(),
                None if d.m.kind == traffic::MovementKind::Arrival
                    && now_s < d.m.t_s + ARRIVAL_WAIT_S =>
                {
                    d.at_s = now_s + 5.0;
                    later.push(d);
                    continue;
                }
                _ => {}
            }
            match d.m.spawn(self.next_id, &self.fleet.field, &ctx) {
                Ok(ac) => {
                    let id = ac.id;
                    let cs = ac.callsign.clone();
                    if self.fleet.spawn(ac) {
                        self.next_id += 1;
                        out.spawned.push((id, cs));
                    }
                }
                Err(traffic::Refusal::NoRunway) => {}
                Err(_) if d.tries + 1 >= SPAWN_TRIES => {}
                Err(_) => {
                    d.tries += 1;
                    d.at_s = now_s + SPAWN_RETRY_S;
                    later.push(d);
                }
            }
        }
        if !later.is_empty() {
            self.due.extend(later);
            self.due.sort_by(|a, b| a.at_s.total_cmp(&b.at_s));
        }
    }
}

/// The command loop's half: the traffic desks synced with what the ticks
/// said, then every posted traffic call answered on its own desk, in
/// order. A reply with an empty line is an accepted readback.
pub(crate) fn drain_calls(station: &mut Station) -> Vec<TrafficReply> {
    station.sync_traffic_desks();
    crate::world::take_traffic_calls()
        .iter()
        .map(|c| station.answer_traffic(c))
        .collect()
}

/// The tower's order view mapped onto the fleet's. A leader that is ours
/// is no traffic id: the follower sequences on its own picture.
pub(crate) fn map_order(o: &TrafficOrder) -> Order {
    use crate::tower::State;
    let some = |s: &String| (!s.is_empty()).then(|| s.clone());
    Order {
        state: match o.state {
            // The taxi-in clearance parks the FSM with a route to a stand:
            // cleared to taxi, to the stand.
            State::Parked if o.to_stand && !o.taxi_path.is_empty() => TowerState::TaxiCleared,
            State::Parked => TowerState::Parked,
            State::TaxiCleared => TowerState::TaxiCleared,
            // "Behind the landing traffic, line up": held short until the
            // named arrival is past.
            State::LinedUp if o.line_up_condition == Some(false) => TowerState::TaxiCleared,
            State::LinedUp => TowerState::LinedUp,
            State::TakeoffCleared => TowerState::TakeoffCleared,
            State::Inbound => TowerState::Inbound,
            State::ClearedToLand => TowerState::ClearedToLand,
        },
        runway: some(&o.runway),
        taxi_path: (!o.taxi_path.is_empty()).then(|| o.taxi_path.clone()),
        holding_point: some(&o.holding_point),
        follow: match o.follow {
            Some(FlightId::Traffic(n)) => Some(n),
            _ => None,
        },
        hold: o.hold,
        go_around: o.go_around,
        readback_owed: o.readback_owed.clone(),
    }
}

/// A traffic aircraft as the station's state for it.
fn aircraft_state(ac: &TrafficAircraft, at: Instant) -> AircraftState {
    AircraftState {
        lat: ac.kin.lat,
        lon: ac.kin.lon,
        alt_ft: ac.kin.alt_ft,
        on_ground: ac.kin.on_ground,
        gs_kts: ac.kin.gs_kts,
        psi_deg: Some(ac.kin.hdg_deg),
        helicopter: false,
        at,
    }
}

/// A traffic aircraft as a TCAS target, keyed by its raw callsign.
fn tcas_target(ac: &TrafficAircraft) -> TcasTarget {
    TcasTarget {
        callsign: ac.callsign.clone(),
        lat: ac.kin.lat,
        lon: ac.kin.lon,
        ele_ft: ac.kin.alt_ft,
        on_ground: ac.kin.on_ground,
        vs_fpm: ac.kin.vs_fpm,
    }
}

/// Why an aircraft left, in a word, for the blackbox.
#[allow(dead_code)]
pub(crate) fn despawn_word(d: Despawn) -> &'static str {
    match d {
        Despawn::LeftTheArea => "left_the_area",
        Despawn::Parked => "parked",
        Despawn::Removed => "removed",
        Despawn::NoStand => "no_stand",
    }
}

#[cfg(test)]
#[path = "traffic_link_tests.rs"]
mod tests;
