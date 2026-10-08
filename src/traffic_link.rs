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
    /// Movements spawned this tick (id, callsign).
    pub(crate) spawned: Vec<(u32, String)>,
    /// Flights that left the frequency because the station was installed
    /// again under them (the reinstall rule).
    pub(crate) left_frequency: Vec<u32>,
    /// Orders that changed this tick: (id, state, runway), for the blackbox.
    pub(crate) orders: Vec<(u32, TowerState, Option<String>)>,
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
    /// Fleet time of the last heartbeat log line.
    heartbeat_s: f64,
    /// The flights noted on their strips last tick (the reinstall rule).
    noted: Vec<u32>,
    /// Each flight's order as last seen (state, runway): a change is logged.
    seen_orders: HashMap<u32, (TowerState, Option<String>)>,
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
            heartbeat_s: f64::NEG_INFINITY,
            noted: Vec::new(),
            seen_orders: HashMap::new(),
        }
    }

    /// More movements, `offset_s` fleet seconds from the clock's start (the
    /// next hour of a generated schedule).
    pub(crate) fn add_schedule(&mut self, schedule: Vec<Movement>, offset_s: f64) {
        self.due.extend(schedule.into_iter().map(|m| Due {
            at_s: m.t_s + offset_s,
            m,
            tries: 0,
        }));
        self.due.sort_by(|a, b| a.at_s.total_cmp(&b.at_s));
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
        // THE REINSTALL RULE: an install clears the strips. A flight noted
        // last tick whose strip is gone (we retire only what we despawn)
        // means the station was installed again under us: everyone not
        // parked leaves the frequency.
        let reinstalled = self
            .noted
            .iter()
            .any(|id| self.fleet.get(*id).is_some() && crate::world::traffic_order(*id).is_none());
        if reinstalled {
            out.left_frequency = self.despawn_moving();
        }
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
        for (id, o) in &orders {
            let now_seen = (o.state, o.runway.clone());
            if self.seen_orders.get(id) != Some(&now_seen) {
                out.orders.push((*id, o.state, o.runway.clone()));
                self.seen_orders.insert(*id, now_seen);
            }
        }
        self.seen_orders.retain(|id, _| orders.contains_key(id));
        let events = self.fleet.step(dt, own.map(OwnState::own_target), &|id| {
            orders.get(&id).cloned()
        });
        // Where everyone is first: a call's strip is open and placed before
        // the station answers it, and the book sees this tick's positions.
        for ac in self.fleet.aircraft() {
            crate::world::note_traffic_state(ac.id, aircraft_state(ac, now));
        }
        self.noted = self.fleet.aircraft().iter().map(|a| a.id).collect();
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
        // A heartbeat every 30 s of fleet time: the clock, what is due, the tower's runway.
        if now_s - self.heartbeat_s >= 30.0 {
            self.heartbeat_s = now_s;
            tracing::info!(now_s, frozen = self.fleet.clock.frozen, due = self.due.len(),
                next_s = self.due.first().map(|d| d.at_s), runway = ?self.tower_runway,
                flying = self.fleet.aircraft().len(), "traffic: heartbeat");
        }
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
            let spawned = d.m.spawn(self.next_id, &self.fleet.field, &ctx);
            if let Err(why) = &spawned {
                tracing::info!(callsign = %d.m.callsign, kind = ?d.m.kind, runway = %d.m.runway,
                    tries = d.tries, "traffic: spawn refused ({why:?})");
                crate::blackbox::record(
                    "traffic",
                    serde_json::json!({ "event": "spawn_refused", "callsign": d.m.callsign,
                        "runway": d.m.runway, "why": format!("{why:?}"), "tries": d.tries }),
                );
            }
            match spawned {
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
/// Each reply comes with the moment its call was keyed.
pub(crate) fn drain_calls(station: &mut Station) -> Vec<(Instant, TrafficReply)> {
    station.sync_traffic_desks();
    crate::world::take_traffic_calls()
        .iter()
        .map(|c| (c.state.at, station.answer_traffic(c)))
        .collect()
}

/// The tower's answers to our traffic, onto the frequency in its own voice,
/// each just after the call it answers (an accepted readback says nothing).
pub(crate) fn overhear_replies(voice: &crate::voice::Voice, replies: Vec<(Instant, TrafficReply)>) {
    for (keyed, reply) in replies {
        if !reply.line.is_empty() {
            let made = keyed + std::time::Duration::from_millis(1);
            voice.overhear(reply.speaker, reply.line, made);
        }
    }
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

/// Who flies the traffic round us (`XCOPILOT_TRAFFIC`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum TrafficMode {
    /// Today's behaviour: the sim's TCAS traffic, narrated read-only. The
    /// default until the plugin draws our own (stage D step 11).
    Off,
    /// The sim's traffic (LiveTraffic, X-Plane's AI), narrated read-only.
    Real,
    /// Our own fleet at the installed field, controlled by its tower.
    Ours,
}

impl TrafficMode {
    /// `ours` or `real` (any case); anything else is `off`.
    pub(crate) fn parse(s: &str) -> TrafficMode {
        match s.trim().to_ascii_lowercase().as_str() {
            "ours" => TrafficMode::Ours,
            "real" => TrafficMode::Real,
            _ => TrafficMode::Off,
        }
    }

    pub(crate) fn as_str(self) -> &'static str {
        match self {
            TrafficMode::Off => "off",
            TrafficMode::Real => "real",
            TrafficMode::Ours => "ours",
        }
    }

    /// The mode in force now (the live settings).
    pub(crate) fn now() -> TrafficMode {
        TrafficMode::parse(&crate::settings::live().traffic)
    }
}

/// Above this time acceleration the fleet freezes: the station's timers are
/// wall `Instant`s, so at 4x a hold would be judged on a quarter of the
/// flight it waited for.
const FAST_OVER: f64 = 1.2;

/// The callsign our aircraft is keyed by in the traffic's picture (the
/// tower names us by `FlightId::Own`, never by this).
const OWN_KEY: &str = "OWNSHIP";

/// The monitor's side of the traffic: the link for the installed field
/// (built off the tick: two apt.dat scans), its clock, the fast-forward
/// latch. Owned by `WorldTick`, never a static.
#[derive(Default)]
pub(crate) struct TrafficWorld {
    link: Option<Link>,
    /// The field the link is for, or is being built for.
    field: Option<String>,
    /// A link being built, off the tick.
    loading: Option<std::sync::mpsc::Receiver<Option<Link>>>,
    /// The sim's running time at the last tick.
    last_sim_s: Option<f64>,
    /// Frozen for time acceleration (said once per spell).
    fast: bool,
    /// Generated schedules flown so far at this field (the next hour's seed
    /// and offset); `None` for the pilot's file.
    hours: Option<u32>,
    /// Links built this session: each new field's seed.
    built: u64,
    /// The packed sink to the plugin, started with the first link.
    sink: Option<crate::traffic_sink::TrafficSink>,
}

impl TrafficWorld {
    /// One monitor tick. Nothing at all unless `XCOPILOT_TRAFFIC=ours`, ATC
    /// is on and a station is installed: the link is dropped (every strip
    /// it opened retired, the picture withdrawn) the moment any of those
    /// stops being true. `sim_speed` is the monitor's latest read.
    pub(crate) async fn tick(
        &mut self,
        xp: &crate::xplane::XPlaneClient,
        voice: &crate::voice::Voice,
        snap: &crate::monitor::Snapshot,
        sim_speed: Option<f64>,
        now: Instant,
    ) -> Option<TickOut> {
        let field = wanted_field(
            TrafficMode::now(),
            crate::mode::atc_enabled(),
            crate::party_line::installed_icao(),
        );
        if field != self.field {
            self.stand_down();
            self.field = field.clone();
            if let Some(icao) = field {
                self.loading = Some(self.build(&icao));
            }
        }
        if let Some(rx) = &self.loading {
            match rx.try_recv() {
                Ok(link) => {
                    self.loading = None;
                    self.link = link;
                }
                Err(std::sync::mpsc::TryRecvError::Empty) => return None,
                Err(std::sync::mpsc::TryRecvError::Disconnected) => self.loading = None,
            }
        }
        let link = self.link.as_mut()?;
        // Time acceleration freezes the fleet (v1), said once per spell.
        let fast = sim_speed.is_some_and(|v| v > FAST_OVER);
        if fast != self.fast {
            self.fast = fast;
            crate::blackbox::record(
                "traffic",
                serde_json::json!({ "event": "frozen", "fast": fast, "sim_speed": sim_speed }),
            );
            if fast {
                voice.alert(
                    crate::voice::Priority::Advisory,
                    "The traffic holds while the sim runs fast.",
                );
            }
        }
        link.set_frozen(fast);
        // The fleet's clock is the sim's (a failed read is no time), and our
        // height in the tower's picture is true feet MSL (the snapshot's is
        // the altimeter's): one batched read, only while the fleet flies.
        let (sim_s, ele_m) = tokio::join!(
            xp.get_float(crate::datarefs::TOTAL_RUNNING_TIME_SEC),
            xp.get_float(crate::datarefs::ELEVATION_M),
        );
        let sim_s = sim_s.ok().filter(|t| t.is_finite());
        let alt_ft = ele_m
            .ok()
            .filter(|m| m.is_finite())
            .map_or(snap.alt_ft, |m| m * 3.28084);
        let dt = match (self.last_sim_s, sim_s) {
            (Some(prev), Some(t)) if t >= prev => t - prev,
            _ => 0.0,
        };
        if sim_s.is_some() {
            self.last_sim_s = sim_s;
        }
        let own = OwnState {
            callsign: OWN_KEY.to_string(),
            lat: snap.lat,
            lon: snap.lon,
            alt_ft,
            on_ground: snap.on_ground,
            vs_fpm: snap.vs_fpm.unwrap_or(0.0),
        };
        let out = link.tick(Some(&own), dt, now);
        // The next hour of a generated schedule, once this one is spent.
        if link.pending() == 0 {
            if let Some(h) = self.hours.as_mut() {
                let start = link.fleet().clock.now_s();
                if start >= f64::from(*h) * traffic::schedule::HORIZON_S {
                    *h += 1;
                    let ms = generate_for(link.fleet(), u64::from(*h), &own);
                    link.add_schedule(ms, start);
                }
            }
        }
        crate::party_line::publish_traffic_picture(Some(link.targets()));
        if self.sink.is_none() {
            self.sink = crate::traffic_sink::TrafficSink::start(
                crate::settings::live().xplane_base_url.clone(),
            );
        }
        if let Some(sink) = self.sink.as_mut() {
            sink.publish(link.fleet().aircraft(), self.last_sim_s.unwrap_or(0.0));
        }
        record(&out, link);
        // The frequency: each call in its flight's own ghost voice, the
        // tower's runway-watch lines in the tower's.
        for t in &out.transmissions {
            let seed = crate::voice::station_voice_seed(&t.callsign);
            voice.overhear(crate::voice::StationSpeaker::Ghost { seed }, &t.text, now);
        }
        for (_, line) in &out.tower {
            voice.overhear(crate::voice::StationSpeaker::Tower, line, now);
        }
        Some(out)
    }

    /// Drop the link: every strip it opened retired, the picture withdrawn.
    fn stand_down(&mut self) {
        if let Some(link) = self.link.take() {
            let ids: Vec<u32> = link.fleet().aircraft().iter().map(|a| a.id).collect();
            for id in &ids {
                crate::world::retire_traffic(*id);
            }
            crate::blackbox::record(
                "traffic",
                serde_json::json!({ "event": "stand_down", "field": link.icao(), "aircraft": ids.len() }),
            );
        }
        if self.field.is_some() {
            crate::party_line::publish_traffic_picture(None);
        }
        if let Some(sink) = self.sink.as_mut() {
            sink.withdraw();
        }
        self.loading = None;
        self.last_sim_s = None;
        self.hours = None;
    }

    /// Build the link for `icao` off the tick: the field (apt.dat, its
    /// taxi network and CIFP) and its schedule, the pilot's file or a
    /// generated hour.
    fn build(&mut self, icao: &str) -> std::sync::mpsc::Receiver<Option<Link>> {
        let cfg = crate::settings::live();
        let (tx, rx) = std::sync::mpsc::channel();
        let (icao, apt, cifp) = (
            icao.to_string(),
            cfg.apt_data_path.clone(),
            cfg.cifp_dir.clone(),
        );
        let (file, density) = (cfg.traffic_file.clone(), cfg.traffic_density.clone());
        self.built += 1;
        let seed = self.built;
        self.hours = file.is_empty().then_some(1);
        let name = crate::party_line::active_field()
            .map(|(n, ..)| crate::party_line::spoken_airport(&n).to_string())
            .unwrap_or_default();
        let spawned = std::thread::Builder::new()
            .name("traffic-field".into())
            .spawn(move || {
                let link = build_link(&icao, &apt, &cifp, &file, &density, seed, &name);
                let _ = tx.send(link);
            });
        if let Err(e) = spawned {
            tracing::warn!("traffic: the field loader did not start: {e}");
        }
        rx
    }
}

/// The field the fleet flies at: the installed station's, and only with
/// `XCOPILOT_TRAFFIC=ours` and ATC on.
pub(crate) fn wanted_field(
    mode: TrafficMode,
    atc_on: bool,
    installed: Option<String>,
) -> Option<String> {
    (mode == TrafficMode::Ours && atc_on)
        .then_some(installed)
        .flatten()
}

/// The link for one field, built off the tick; `None` (said in the log and
/// the blackbox) when the field or the file cannot be read.
fn build_link(
    icao: &str,
    apt: &str,
    cifp: &str,
    file: &str,
    density: &str,
    seed: u64,
    name: &str,
) -> Option<Link> {
    let field = match FieldModel::load(apt, icao) {
        Ok(Some(f)) => f.with_cifp(cifp),
        Ok(None) => {
            tracing::warn!(%icao, "traffic: the field is not in apt.dat; no traffic");
            return None;
        }
        Err(e) => {
            tracing::warn!(%icao, "traffic: apt.dat scan failed ({e:#}); no traffic");
            return None;
        }
    };
    let ctx = SpawnCtx::default();
    let (schedule, source) = if file.is_empty() {
        let d = traffic::Density::parse(density).unwrap_or(traffic::Density::Normal);
        (traffic::generate(&field, d, seed, &ctx), "generated")
    } else {
        let text = match std::fs::read_to_string(file) {
            Ok(t) => t,
            Err(e) => {
                tracing::warn!(%file, "traffic: the schedule file did not read ({e}); no traffic");
                crate::blackbox::record(
                    "traffic",
                    serde_json::json!({ "event": "file_refused", "file": file, "why": e.to_string() }),
                );
                return None;
            }
        };
        match traffic::schedule::parse_file(&text) {
            Ok(rows) => (
                traffic::schedule::from_file(&rows, &field, seed, &ctx),
                "file",
            ),
            Err(e) => {
                tracing::warn!(%file, "traffic: the schedule file was refused ({e:#}); no traffic");
                crate::blackbox::record(
                    "traffic",
                    serde_json::json!({ "event": "file_refused", "file": file, "why": format!("{e:#}") }),
                );
                return None;
            }
        }
    };
    tracing::info!(%icao, movements = schedule.len(), source, "traffic: the fleet is up");
    crate::blackbox::record(
        "traffic",
        serde_json::json!({
            "event": "up", "field": icao, "movements": schedule.len(), "source": source,
        }),
    );
    Some(Link::new(field, schedule, name))
}

/// The next generated hour at the fleet's field, round us.
fn generate_for(fleet: &Fleet, hour: u64, own: &OwnState) -> Vec<Movement> {
    let density = traffic::Density::parse(&crate::settings::live().traffic_density)
        .unwrap_or(traffic::Density::Normal);
    let ctx = SpawnCtx {
        runway: None,
        own: Some((own.lat, own.lon)),
        occupied: fleet
            .aircraft()
            .iter()
            .filter(|a| a.kin.on_ground)
            .map(|a| a.kin.pos())
            .collect(),
    };
    traffic::generate(&fleet.field, density, hour, &ctx)
}

/// The blackbox's `traffic` records for one tick: one record per event, each
/// one write.
fn record(out: &TickOut, link: &Link) {
    for (id, callsign) in &out.spawned {
        let ac = link.fleet().get(*id);
        crate::blackbox::record(
            "traffic",
            serde_json::json!({
                "event": "spawn", "id": id, "callsign": callsign,
                "type": ac.map(|a| a.icao_type.clone()),
                "plan": ac.map(|a| plan_word(&a.plan)),
                "runway": ac.and_then(|a| a.plan.runway().map(str::to_string)),
            }),
        );
    }
    for id in &out.left_frequency {
        crate::blackbox::record(
            "traffic",
            serde_json::json!({ "event": "despawn", "id": id, "reason": "reinstalled" }),
        );
    }
    for ev in &out.events {
        match ev {
            Event::Despawn { id, reason } => crate::blackbox::record(
                "traffic",
                serde_json::json!({ "event": "despawn", "id": id, "reason": despawn_word(*reason) }),
            ),
            Event::Phase { id, from, to } => crate::blackbox::record(
                "traffic",
                serde_json::json!({
                    "event": "phase", "id": id,
                    "from": format!("{from:?}"), "to": format!("{to:?}"),
                }),
            ),
            Event::Touchdown { id, along_m, .. } => crate::blackbox::record(
                "traffic",
                serde_json::json!({ "event": "touchdown", "id": id, "along_m": along_m.round() }),
            ),
            Event::Call { .. } | Event::Spawn { .. } => {}
        }
    }
    for t in &out.transmissions {
        crate::blackbox::record(
            "traffic",
            serde_json::json!({ "event": "call", "id": t.id, "callsign": t.callsign, "text": t.text }),
        );
    }
    for (id, state, runway) in &out.orders {
        crate::blackbox::record(
            "traffic",
            serde_json::json!({
                "event": "order", "id": id, "state": format!("{state:?}"), "runway": runway,
            }),
        );
    }
    for (id, line) in &out.tower {
        crate::blackbox::record(
            "traffic",
            serde_json::json!({ "event": "tower", "id": id, "line": line }),
        );
    }
}

/// A plan in a word, for the blackbox.
fn plan_word(p: &traffic::Plan) -> &'static str {
    match p {
        traffic::Plan::Circuit { .. } => "circuit",
        traffic::Plan::Departure { .. } => "departure",
        traffic::Plan::Arrival { .. } => "arrival",
        traffic::Plan::Parked => "parked",
    }
}

/// Why an aircraft left, in a word, for the blackbox.
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
