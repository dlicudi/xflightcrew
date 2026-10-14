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
//!
//! The whole flight (2026-10-13, [`whole`]): every timetabled flight of a
//! simulated field exists from its origin's blocks to its destination's,
//! the model flying what the simulation does not.
//!
//! Several fields at once (2026-10-13, `docs/tasks/multi-field-traffic`):
//! [`TrafficWorld`] keeps a link per field simulated, the station's field
//! worked as above, every other timetable field in range by its own runway
//! desk ([`Link::make_desk`], `traffic::FieldDesk`), silent and never
//! through the party line; ids come from one counter ([`Ids`]), each
//! field's day from its own seed ([`SESSION_SEED`]); only the aircraft
//! within 40 nm are drawn (`traffic_sink::DrawCull`).

use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};
use std::time::Instant;

use traffic::{
    Call, Despawn, Event, FieldModel, Fleet, Movement, MovementKind, Order, OwnTarget, Phase,
    SpawnCtx, TowerState, TrafficAircraft,
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
    /// A safety call ([`is_safety_call`]): keyed without waiting its turn.
    pub(crate) safety: bool,
    /// The position whose frequency it is keyed on, as the outbox paces
    /// it ([`Link::channel`]).
    pub(crate) on: crate::voice::StationSpeaker,
    /// Fleet seconds it waited in the outbox for a clear frequency.
    pub(crate) waited_s: f64,
    /// Fleet second the frequency is reckoned clear again after it: the
    /// call's airtime, and the controller's beat and answer when one is
    /// owed ([`Outbox`]).
    pub(crate) clear_s: f64,
}

/// Words spoken per second on the frequency, for a transmission's airtime:
/// radio pace (CAP 413 asks a controller for under 100 words a minute; the
/// piper station voices run faster, the crowd's ghosts near this).
pub(crate) const WORDS_PER_S: f64 = 2.6;

/// Keying the transmitter and the carrier before the first word, seconds.
pub(crate) const KEY_S: f64 = 0.5;

/// The controller's beat between the end of a call and the start of its
/// answer, seconds: nobody answers in the same frame (the review measured
/// a median 1.4 s from the call QUEUED to its reply, i.e. the reply on the
/// call's heels).
pub(crate) const REPLY_BEAT_S: (f64, f64) = (1.0, 3.0);

/// A pilot waits this long after the frequency clears before keying,
/// seconds: listening first, then a gap so two stations do not key into
/// the same silence (120 of 836 consecutive radio lines were under 1 s
/// apart in the review).
pub(crate) const KEY_GAP_S: (f64, f64) = (1.0, 4.0);

/// A controller's answer, reckoned before it is made: a clearance with a
/// callsign, a runway and a wind runs 12 to 18 words.
const REPLY_EST_S: f64 = 5.0;

/// How long a transmission takes on the air, seconds.
pub(crate) fn airtime_s(text: &str) -> f64 {
    KEY_S + text.split_whitespace().count() as f64 / WORDS_PER_S
}

/// A value in `band`, the same for the same `seed`: the fleet's frequency
/// is reproducible run to run (a sweep compares like with like), the gaps
/// still differ call to call.
pub(crate) fn jitter(seed: u64, band: (f64, f64)) -> f64 {
    // splitmix64: one round is plenty for a pause.
    let mut z = seed.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^= z >> 31;
    band.0 + (band.1 - band.0) * (z >> 11) as f64 / (1u64 << 53) as f64
}

/// After our own pilot's transmission (or the tower's answer to it) the
/// frequency is still reckoned busy this long, seconds: the controller's
/// beat before answering a call just made, which the traffic must not key
/// into.
pub(crate) const OWN_TAIL_S: f64 = 3.0;

/// A call that never waits for the frequency: a go-around, or the
/// acknowledgement of a go-around order ("Going around, ...").
pub(crate) fn is_safety_call(call: &Call) -> bool {
    match call {
        Call::GoingAround => true,
        Call::Readback(text) => text.starts_with("Going around"),
        _ => false,
    }
}

/// A call the controller answers: everything but a readback (an accepted
/// readback is met with silence; a wrong one is the exception).
fn owes_reply(call: &Call) -> bool {
    !matches!(call, Call::Readback(_))
}

/// One position's frequency, as the outbox reckons it.
#[derive(Debug, Clone, Copy)]
struct Freq {
    on: crate::voice::StationSpeaker,
    /// Fleet second it is reckoned clear: the last call's airtime, the
    /// controller's beat and answer, a tower line, or our own pilot.
    clear_s: f64,
    /// The pause the next caller leaves after it clears ([`KEY_GAP_S`]).
    gap_s: f64,
}

/// A call [`Outbox::due_on`] keyed.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Keyed {
    pub(crate) id: u32,
    pub(crate) call: Call,
    pub(crate) on: crate::voice::StationSpeaker,
    /// Fleet seconds it waited for the frequency.
    pub(crate) waited_s: f64,
    /// Fleet second the frequency is reckoned clear after it.
    pub(crate) clear_s: f64,
}

/// The traffic's calls not yet keyed, each on its position's frequency
/// (the tower's, Ground's, the approach unit's), in the order made, keyed
/// only on a CLEAR frequency: a pilot listens first and keys once the last
/// exchange (the call, the controller's beat and its answer) has ended and
/// a 1 to 4 s gap has run (#260). Ground's push and taxi calls never wait
/// on the Tower's landing traffic, as two frequencies never do. Pure: the
/// link feeds it the fleet's calls, the tower's own lines and our pilot's
/// transmissions ([`Outbox::occupy`]), and keys what [`Outbox::due_on`]
/// hands back.
#[derive(Debug, Default)]
pub(crate) struct Outbox {
    /// (flight, call, frequency, fleet second queued).
    queue: VecDeque<(u32, Call, crate::voice::StationSpeaker, f64)>,
    freqs: Vec<Freq>,
    /// Calls keyed so far: the jitter's seed.
    keyed: u64,
}

impl Outbox {
    /// Queue a call on the tower's frequency at second 0.
    #[cfg(test)]
    pub(crate) fn push(&mut self, id: u32, call: Call) {
        self.push_on(id, call, crate::voice::StationSpeaker::Tower, 0.0);
    }

    /// Queue a call on position `on`'s frequency at fleet second `now_s`.
    /// The same call from the same flight already waiting is not queued
    /// twice (a crossing asked again while its first ask waits).
    pub(crate) fn push_on(
        &mut self,
        id: u32,
        call: Call,
        on: crate::voice::StationSpeaker,
        now_s: f64,
    ) {
        if !self.queue.iter().any(|(i, c, _, _)| *i == id && *c == call) {
            self.queue.push_back((id, call, on, now_s));
        }
    }

    /// A flight left: its waiting calls go with it.
    pub(crate) fn forget(&mut self, id: u32) {
        self.queue.retain(|(i, _, _, _)| *i != id);
    }

    fn freq_mut(&mut self, on: crate::voice::StationSpeaker) -> &mut Freq {
        let k = match self.freqs.iter().position(|f| f.on == on) {
            Some(k) => k,
            None => {
                self.freqs.push(Freq {
                    on,
                    clear_s: f64::NEG_INFINITY,
                    gap_s: 0.0,
                });
                self.freqs.len() - 1
            }
        };
        &mut self.freqs[k]
    }

    /// Position `on`'s frequency is busy until fleet second `until_s` with
    /// a transmission that is not a traffic call: a tower line, a handoff,
    /// our own pilot keying (or the tower answering us). The next caller
    /// waits it out and then its gap.
    pub(crate) fn occupy(&mut self, on: crate::voice::StationSpeaker, until_s: f64) {
        let f = self.freq_mut(on);
        f.clear_s = f.clear_s.max(until_s);
    }

    /// When a line said on position `on` at `now_s` starts: now, or once
    /// the exchange on the air has ended.
    pub(crate) fn clear_from(&self, on: crate::voice::StationSpeaker, now_s: f64) -> f64 {
        self.freqs
            .iter()
            .find(|f| f.on == on)
            .map_or(now_s, |f| f.clear_s.max(now_s))
    }

    /// When position `on`'s frequency is reckoned clear.
    #[cfg(test)]
    pub(crate) fn clear_at(&self, on: crate::voice::StationSpeaker) -> Option<f64> {
        self.freqs.iter().find(|f| f.on == on).map(|f| f.clear_s)
    }

    /// Calls waiting.
    #[cfg(test)]
    pub(crate) fn waiting(&self) -> usize {
        self.queue.len()
    }

    /// Key `call` on `on` at `now_s`: the frequency is busy for its airtime
    /// and, when the controller owes an answer, its beat and the answer.
    fn key(
        &mut self,
        id: u32,
        call: &Call,
        on: crate::voice::StationSpeaker,
        now_s: f64,
        air_s: f64,
    ) -> f64 {
        self.keyed += 1;
        let seed = self.keyed.wrapping_mul(0x1000_0001) ^ u64::from(id);
        let reply = if owes_reply(call) {
            jitter(seed, REPLY_BEAT_S) + REPLY_EST_S
        } else {
            0.0
        };
        let gap = jitter(seed ^ 0x5bd1_e995, KEY_GAP_S);
        let f = self.freq_mut(on);
        f.clear_s = f.clear_s.max(now_s + air_s + reply);
        f.gap_s = gap;
        f.clear_s
    }

    /// [`Outbox::due_on`] with a stock airtime, for the pure tests.
    #[cfg(test)]
    pub(crate) fn due(&mut self, now_s: f64) -> Vec<(u32, Call)> {
        self.due_on(now_s, |_, c| {
            airtime_s(&c.text("November one two three", None))
        })
        .into_iter()
        .map(|k| (k.id, k.call))
        .collect()
    }

    /// What to key now, at fleet second `now_s`: every safety call at once
    /// (a go-around is reported as it is flown), then on each frequency at
    /// most ONE other, once the frequency is clear and its gap has run (a
    /// readback first: it completes a clearance the flight is waiting on;
    /// else the oldest). `airtime` reckons a call's seconds on the air.
    pub(crate) fn due_on(&mut self, now_s: f64, airtime: impl Fn(u32, &Call) -> f64) -> Vec<Keyed> {
        let mut out = Vec::new();
        let mut i = 0;
        while i < self.queue.len() {
            if !is_safety_call(&self.queue[i].1) {
                i += 1;
                continue;
            }
            if let Some((id, call, on, at)) = self.queue.remove(i) {
                let clear_s = self.key(id, &call, on, now_s, airtime(id, &call));
                out.push(Keyed {
                    id,
                    call,
                    on,
                    waited_s: now_s - at,
                    clear_s,
                });
            }
        }
        let mut channels: Vec<crate::voice::StationSpeaker> = Vec::new();
        for (_, _, on, _) in &self.queue {
            if !channels.contains(on) {
                channels.push(*on);
            }
        }
        for on in channels {
            if out.iter().any(|k: &Keyed| k.on == on) {
                continue;
            }
            let free = self
                .freqs
                .iter()
                .find(|f| f.on == on)
                .is_none_or(|f| now_s >= f.clear_s + f.gap_s - 1e-9);
            if !free {
                continue;
            }
            let mine = |c: &(u32, Call, crate::voice::StationSpeaker, f64)| c.2 == on;
            let next = self
                .queue
                .iter()
                .position(|c| mine(c) && matches!(c.1, Call::Readback(_)))
                .or_else(|| self.queue.iter().position(mine));
            if let Some((id, call, on, at)) = next.and_then(|k| self.queue.remove(k)) {
                let clear_s = self.key(id, &call, on, now_s, airtime(id, &call));
                out.push(Keyed {
                    id,
                    call,
                    on,
                    waited_s: now_s - at,
                    clear_s,
                });
            }
        }
        out
    }
}

/// What one tick of the link produced.
#[derive(Debug, Default)]
pub(crate) struct TickOut {
    /// The tower's lines from the traffic flights' runway watches
    /// (`world::traffic_tick`), in order: said in the tower's voice.
    pub(crate) tower: Vec<(u32, String)>,
    /// The aerodrome positions' handoffs (`world::traffic_position_tick`):
    /// Ground's to the Tower at the hold, the Tower's to Ground on the
    /// rollout, each with the position that says it.
    pub(crate) positions: Vec<(u32, String, crate::voice::StationSpeaker)>,
    /// What the traffic keyed this tick (posted to the station already).
    pub(crate) transmissions: Vec<Transmission>,
    /// The fleet's events, for the blackbox.
    pub(crate) events: Vec<Event>,
    /// Movements spawned this tick (id, callsign).
    pub(crate) spawned: Vec<(u32, String)>,
    /// Of those, the ones that flew off a stand they already stood on (id,
    /// "parked": from the fleet's start; "turned_round": an arrival of this
    /// fleet's after its turnaround); the rest appeared on their stand from
    /// nothing.
    pub(crate) from_stand: Vec<(u32, &'static str)>,
    /// Aircraft put on their stands parked at the fleet's start (id,
    /// callsign): the field's parked aircraft and the departures waiting.
    pub(crate) parked: Vec<(u32, String)>,
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
    /// The aircraft already standing on its stand for it, parked and dark
    /// since the fleet's start ([`Movement::parked_from_start`]): released
    /// at its time rather than spawned.
    held: Option<u32>,
    /// When it was first due, fleet seconds: an arrival's whole-flight model
    /// reaches its entry then ([`whole`]), whatever retries come after.
    first_s: f64,
    /// Its id once the model has shown it (the spawn keeps it: one flight,
    /// one id, one roster entry from the origin to the gate).
    id: Option<u32>,
    /// An arrival's whole-flight model and the runway it was built for
    /// (`None` inside: the model cannot fly it, not tried again).
    model: Option<(String, Option<whole::Modeled>)>,
}

/// The fleet at the installed field, its schedule and its spawn book.
pub(crate) struct Link {
    fleet: Fleet,
    due: Vec<Due>,
    /// Where its new aircraft take their ids: one counter shared by every
    /// field's fleet ([`Ids`]).
    ids: Ids,
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
    /// The frequencies' calls waiting for a clear frequency ([`Outbox`]).
    outbox: Outbox,
    /// The field publishes a Ground position of its own (apt.dat 1053 on
    /// a frequency apart from the tower's): the push, the taxi out and the
    /// taxi in are paced on Ground's frequency.
    has_ground: bool,
    /// The field's parked aircraft ([`traffic::park_at_start`]), put on
    /// their stands at the first tick, where our own position is known.
    to_park: Vec<Movement>,
    /// The first tick has parked them.
    parked: bool,
    /// The flight information board (2026-10-13): every flight of the
    /// timetable's window (a route table's rotations, a file's rows that
    /// name their ends), whatever became of it.
    board: Vec<traffic::board::BoardFlight>,
    /// The local time of day at the fleet's start, seconds after the
    /// timetable's midnight: the board's clock.
    t0_local_s: f64,
    /// When each flight was seen to leave the runway or land on it, local
    /// seconds: the board's "actual" time.
    actual: HashMap<String, f64>,
    /// A field that is not the pilot's station (multi-field traffic stage
    /// 3, 2026-10-13): its traffic worked by this runway desk, silently,
    /// and never through the party line ([`Link::make_desk`]); `None`: the
    /// installed station works it, as it always did.
    desk: Option<traffic::FieldDesk>,
    /// Generated schedules flown so far at this field (the next hour's
    /// seed and offset); `None` for a timetable or the pilot's file.
    hours: Option<u32>,
    /// THE WHOLE FLIGHT (2026-10-13, `traffic::track`): the departures the
    /// model flies on beyond the simulation, to their destinations.
    beyond: Vec<whole::Modeled>,
    /// The departures gone before the start, still to be modelled from
    /// their wheels up at the first model tick (the board's clock is set
    /// after [`Link::new`]).
    away: Vec<Movement>,
    /// Fleet time of the last model tick ([`whole::MODEL_EVERY_S`]).
    model_s: f64,
    /// The timetable's day is in the summer season (the other ends'
    /// clocks, `traffic::places::utc_offset_h`).
    summer: bool,
    /// The whole flight of each arrival the simulation took from the model,
    /// by id: its origin's times for the EFB's card.
    entered: HashMap<u32, (String, traffic::track::Whole)>,
    /// A route table's timetable: what it was planned with, so a jump of
    /// the sim's clock plans it again ([`Link::reanchor`]); `None` for a
    /// generated schedule or the pilot's file.
    timetable: Option<Timetable>,
}

/// What a timetable link was planned with (`timetable_start`): its density,
/// seed and the date its board's local seconds count from.
#[derive(Debug, Clone, Copy)]
struct Timetable {
    density: traffic::Density,
    seed: u64,
    date: traffic::rotation::Date,
}

/// The sim's clock and the timetable's may part by this much, seconds,
/// before the timetable is planned again: a sim time or date change, never
/// the drift of a paused or fast-forwarded spell's few seconds.
pub(crate) const CLOCK_JUMP_S: f64 = 600.0;

/// How often the sim's clock is compared with the timetables'.
const CLOCK_CHECK_EVERY: std::time::Duration = std::time::Duration::from_secs(60);

/// Seconds the sim's local clock (`now_date`, `now_local_s`) runs ahead of
/// a timetable's (`expected_local_s` after `ref_date`'s midnight); negative
/// behind. Pure.
pub(crate) fn clock_jump_s(
    ref_date: traffic::rotation::Date,
    expected_local_s: f64,
    now_date: traffic::rotation::Date,
    now_local_s: f64,
) -> f64 {
    let days = (now_date.epoch_days() - ref_date.epoch_days()) as f64;
    now_local_s + days * 86_400.0 - expected_local_s
}

/// Traffic ids from ONE counter for every field's fleet (multi-field
/// traffic stage 2, 2026-10-13): each `Link` counted from 1, so two fleets
/// at once handed out the same ids, and the party line's strips
/// (`FlightId::Traffic(u32)`, no field in the key) and the plugin's roster
/// would have mixed them. A clone shares the counter. Ids start at 1 and
/// stay below the fleet's own picture ids (`u32::MAX - 1` and up).
#[derive(Debug, Clone)]
pub(crate) struct Ids(std::sync::Arc<std::sync::atomic::AtomicU32>);

impl Default for Ids {
    fn default() -> Ids {
        Ids(std::sync::Arc::new(std::sync::atomic::AtomicU32::new(1)))
    }
}

impl Ids {
    /// The id the next aircraft takes (a spawn may still be refused).
    pub(crate) fn peek(&self) -> u32 {
        self.0.load(std::sync::atomic::Ordering::Relaxed)
    }

    /// That id was taken.
    pub(crate) fn bump(&self) {
        self.0.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    }
}

/// A departure waits this long for the airframe its rotation names (its
/// inbound still flying, taxiing or not yet spawned) before it takes any
/// airframe or appears on a stand, fleet seconds.
const INBOUND_WAIT_S: f64 = 3.0 * 3600.0;

/// An arrival spawns for the tower's runway: with none known yet (nobody
/// has been cleared) it waits this long, then takes the field's default.
const ARRIVAL_WAIT_S: f64 = 120.0;

/// The station's runway in use, as the command loop last saw it: traffic
/// spawns for it before the tower has cleared anyone (the link learnt the
/// runway only from orders, so traffic flew the plan's 13 under a tower on 12,
/// LEMG, 2026-10-08). Overwritten every command-loop turn with
/// `Station::runway_for_now`: the runway in use, else the pick the first
/// traffic call would make, so arrivals spawn for the tower's runway from
/// the install on instead of waiting [`ARRIVAL_WAIT_S`] for a first call.
static STATION_RUNWAY: std::sync::Mutex<Option<String>> = std::sync::Mutex::new(None);

/// Note the station's runway in use (the command loop, each turn).
pub(crate) fn note_station_runway(r: Option<&str>) {
    *STATION_RUNWAY.lock().unwrap_or_else(|e| e.into_inner()) = r.map(str::to_string);
}

fn station_runway() -> Option<String> {
    STATION_RUNWAY.lock().unwrap_or_else(|e| e.into_inner()).clone()
}

impl Link {
    /// The fleet at `field`, flying `schedule` (times from now). Its parked
    /// aircraft ([`MovementKind::Parked`]) and the departures parked from
    /// the start stand on their stands from the first tick.
    pub(crate) fn new(field: FieldModel, schedule: Vec<Movement>, tower_name: &str) -> Link {
        let (to_park, schedule): (Vec<Movement>, Vec<Movement>) = schedule
            .into_iter()
            .partition(|m| m.kind == MovementKind::Parked);
        // Gone before the start: the whole-flight model's from its wheels up.
        let (away, schedule): (Vec<Movement>, Vec<Movement>) = schedule
            .into_iter()
            .partition(|m| matches!(m.start, Some(traffic::Start::Away { .. })));
        let mut due: Vec<Due> = schedule
            .into_iter()
            .map(with_telephony)
            .map(|m| Due {
                at_s: m.t_s,
                first_s: m.t_s,
                m,
                tries: 0,
                held: None,
                id: None,
                model: None,
            })
            .collect();
        due.sort_by(|a, b| a.at_s.total_cmp(&b.at_s));
        let has_ground = match (
            field.airport.freq(crate::nav::FREQ_GROUND),
            field.airport.freq(crate::nav::FREQ_TOWER),
        ) {
            (Some(g), Some(t)) => (g.khz - t.khz).abs() > 5,
            (Some(_), None) => true,
            _ => false,
        };
        // An arrival that parks stays and flies again (`Fleet::turn_round`).
        let mut fleet = Fleet::new(field);
        fleet.turn_round = true;
        Link {
            has_ground,
            fleet,
            due,
            ids: Ids::default(),
            tower_name: tower_name.to_string(),
            tower_runway: None,
            heartbeat_s: f64::NEG_INFINITY,
            noted: Vec::new(),
            seen_orders: HashMap::new(),
            outbox: Outbox::default(),
            to_park: to_park.into_iter().map(with_telephony).collect(),
            parked: false,
            board: Vec::new(),
            t0_local_s: 0.0,
            actual: HashMap::new(),
            desk: None,
            hours: None,
            beyond: Vec::new(),
            away,
            model_s: f64::NEG_INFINITY,
            summer: true,
            entered: HashMap::new(),
            timetable: None,
        }
    }

    /// Take this link's ids from `ids`, shared with the other fields'
    /// fleets ([`Ids`]). Before the first tick: nothing is spawned yet.
    pub(crate) fn share_ids(&mut self, ids: &Ids) {
        self.ids = ids.clone();
    }

    /// Worked by a runway desk of its own ([`traffic::FieldDesk`]), not
    /// the station: the field is not the pilot's station's.
    pub(crate) fn is_desk(&self) -> bool {
        self.desk.is_some()
    }

    /// Hand the field to its own runway desk (multi-field traffic stage 3,
    /// 2026-10-13): built for a field in range that is not the station's,
    /// or the station's field when the pilot's station moves on. Every
    /// strip it held at the station is retired (a station installed since
    /// cleared them already) and the desk takes each flight where its
    /// phase says it was cleared ([`traffic::FieldDesk::adopt`]): nobody
    /// leaves. The runway is the one the tower last cleared, else the
    /// field's default.
    pub(crate) fn make_desk(&mut self) {
        if self.desk.is_some() {
            return;
        }
        for a in self.fleet.aircraft() {
            crate::world::retire_traffic(a.id);
        }
        let runway = self
            .tower_runway
            .clone()
            .filter(|r| self.fleet.field.end(r).is_some())
            .or_else(|| traffic::schedule::default_runway(&self.fleet.field));
        let mut desk = traffic::FieldDesk::new(runway);
        desk.adopt(&self.fleet);
        self.desk = Some(desk);
        self.outbox = Outbox::default();
        self.noted.clear();
        self.seen_orders.clear();
    }

    /// Hand the field to the station (the pilot's station installed on
    /// it): the desk retires and, as the reinstall rule has it, every
    /// aircraft not parked leaves the frequency (stage 4 hands the desk's
    /// strips to the station instead). Their ids.
    pub(crate) fn make_station(&mut self) -> Vec<u32> {
        if self.desk.take().is_none() {
            return Vec::new();
        }
        self.noted.clear();
        self.seen_orders.clear();
        self.despawn_moving()
    }

    /// The runway every movement spawns for: the desk's; else the one the
    /// tower last cleared anyone for, else the station's in use.
    fn runway_now(&self) -> Option<String> {
        match &self.desk {
            Some(d) => d.runway().map(str::to_string),
            None => self.tower_runway.clone().or_else(station_runway),
        }
    }

    /// The station's type for a traffic callsign (its wake and its
    /// telephony): only for the station's own field.
    fn note_type(&self, callsign: &str, icao_type: &str) {
        if self.desk.is_none() {
            crate::world::note_traffic_type(callsign, icao_type);
        }
    }

    /// The board's flights and the local time of day at the fleet's start
    /// (the mid-flow start's timetable, `traffic::midflow::plan`).
    pub(crate) fn set_board(&mut self, board: Vec<traffic::board::BoardFlight>, t0_local_s: f64) {
        self.board = board;
        self.t0_local_s = t0_local_s;
    }

    /// Local seconds after the timetable's midnight now.
    fn local_now_s(&self) -> f64 {
        self.t0_local_s + self.fleet.clock.now_s()
    }

    /// A timetable that opts in to the TOP-UP (`top_up` in its route file,
    /// Gatwick's, 2026-10-14): the generator fills its hours.
    fn tops_up(&self) -> bool {
        self.timetable.is_some()
            && traffic::routes::field_routes(&self.fleet.field.icao).is_some_and(|t| t.top_up)
    }

    /// The next hour of a timetable's top-up from now
    /// (`traffic::midflow::top_up`: the field's rate less the timetable's),
    /// `hour` its draws, clear of every stand the fleet holds and every
    /// stand a timetable movement still to come is planned on. Nothing for
    /// a table that does not opt in.
    pub(crate) fn top_up_hour(
        &self,
        hour: u64,
        own: Option<(f64, f64)>,
        zulu_s: Option<f64>,
    ) -> Vec<Movement> {
        let (Some(tt), Some(table)) = (
            self.timetable,
            traffic::routes::field_routes(&self.fleet.field.icao),
        ) else {
            return Vec::new();
        };
        let field = &self.fleet.field;
        let mut occupied = self.fleet.stands_held();
        occupied.extend(planned_stands(field, self.due.iter().map(|d| &d.m)));
        let ctx = SpawnCtx {
            runway: None,
            own,
            occupied,
            zulu_s,
            parked_airframes: 0,
        };
        traffic::midflow::top_up(
            field,
            table,
            tt.density,
            tt.seed,
            hour,
            tt.date,
            self.local_now_s(),
            &ctx,
        )
    }

    /// The board's payload (`traffic::board::payload`): each flight's status
    /// from where its aircraft is (by its callsign, or a departure's
    /// inbound's while it turns round), the live ETA of an arrival in the
    /// air, the actual time once it happened, its stand; `own` adds our own
    /// flight. Empty with nothing to show.
    pub(crate) fn board_payload(
        &self,
        own: Option<traffic::board::BoardFlight>,
        choices: &[String],
    ) -> String {
        use traffic::board::{Dir, Row, Seen, Status};
        let now = self.local_now_s();
        let find = |cs: &str| self.fleet.aircraft().iter().find(|a| a.callsign == cs);
        let names: Vec<&str> = self
            .fleet
            .field
            .net
            .stands
            .iter()
            .map(|s| s.name.as_str())
            .collect();
        let rows: Vec<Row> = self
            .board
            .iter()
            .cloned()
            .chain(own)
            .map(|f| {
                let ac = find(&f.callsign).or_else(|| {
                    (f.dir == Dir::Departure)
                        .then(|| f.info.inbound.as_deref().and_then(find))
                        .flatten()
                        .filter(|a| a.dormant())
                });
                let seen = ac.map(|a| Seen {
                    phase: a.phase,
                    dormant: a.dormant(),
                });
                let status = if f.own {
                    Status::Scheduled
                } else {
                    Status::of(&f, seen, now)
                };
                let eta = ac
                    .filter(|a| a.phase.airborne() && f.dir == Dir::Arrival)
                    .and_then(|a| a.slot())
                    .map(|s| now + s.to_thr_s.max(0.0));
                let time_s = self
                    .actual
                    .get(&f.callsign)
                    .copied()
                    .or(eta)
                    .or(f.info.est_s);
                // The stand the aircraft holds once it holds one (its own,
                // or a departure's inbound's on the ground: a gate change
                // where it parked elsewhere), else the field's plan; an
                // arrival's only once it is down (2026-10-13: departures
                // Boarding with an empty Stand column).
                let holding = ac.and_then(|a| a.stand.clone()).or_else(|| {
                    (f.dir == Dir::Departure)
                        .then(|| {
                            let inbound = f.info.inbound.as_deref()?;
                            self.fleet
                                .aircraft()
                                .iter()
                                .find(|a| a.callsign == inbound && !a.phase.airborne())?
                                .stand
                                .clone()
                        })
                        .flatten()
                });
                let planned = match f.dir {
                    Dir::Departure => f.info.stand.clone(),
                    Dir::Arrival => matches!(status, Status::Landed | Status::AtGate)
                        .then(|| f.info.stand.clone())
                        .flatten(),
                };
                let stand = holding
                    .or(planned)
                    .map(|n| traffic::stands::label(&n, &names));
                Row {
                    flight: f,
                    status,
                    time_s,
                    stand,
                }
            })
            .collect();
        traffic::board::payload(self.icao(), now, &rows, choices)
    }

    /// The Map tab's picture of this field (2026-10-13): every aircraft
    /// with the board's words for it, and every stand one of them holds
    /// ([`Fleet::stand_holder`], the fleet's own rule). No reads: the
    /// fleet and the board are already here.
    pub(crate) fn efb_traffic(&self) -> crate::efb::feed::TrafficField {
        use crate::efb::feed::{StandTaken, TrafficAc, TrafficField};
        use traffic::board::Dir;
        use traffic::fleet::StandHold;
        let icao = self.icao().to_string();
        let names: Vec<&str> = self
            .fleet
            .field
            .net
            .stands
            .iter()
            .map(|s| s.name.as_str())
            .collect();
        let mut aircraft: Vec<TrafficAc> = self
            .fleet
            .aircraft()
            .iter()
            .filter(|a| a.phase != Phase::Gone)
            .map(|a| {
                let f = self.board.iter().find(|f| f.callsign == a.callsign);
                let (from, to) = match (f.map(|f| (f.dir, &f.info.other)), &a.plan) {
                    (Some((Dir::Departure, o)), _) => (icao.clone(), o.clone()),
                    (Some((Dir::Arrival, o)), _) => (o.clone(), icao.clone()),
                    (None, traffic::Plan::Departure { .. }) => (icao.clone(), String::new()),
                    (None, traffic::Plan::Arrival { .. }) => (String::new(), icao.clone()),
                    (None, traffic::Plan::Circuit { .. }) => (icao.clone(), icao.clone()),
                    (None, traffic::Plan::Parked) => (String::new(), String::new()),
                };
                TrafficAc {
                    id: a.id,
                    callsign: a.callsign.clone(),
                    spoken: a.spoken.clone(),
                    airline: f.map(|f| f.info.airline.clone()).unwrap_or_default(),
                    icao_type: a.icao_type.clone(),
                    from,
                    to,
                    phase: crate::efb::airport::traffic_phase_words(a.phase, a.dormant()),
                    stand: a
                        .stand
                        .as_deref()
                        .map(|n| traffic::stands::label(n, &names)),
                    sched: f.and_then(|f| f.info.sched_s).map(traffic::rotation::hhmm),
                    est: f.and_then(|f| f.info.est_s).map(traffic::rotation::hhmm),
                    lat: a.kin.lat,
                    lon: a.kin.lon,
                    hdg: a.kin.hdg_deg,
                    alt_ft: a.kin.alt_ft,
                    on_ground: a.kin.on_ground,
                    gs_kts: a.kin.gs_kts,
                    ahead: if a.dormant() { Vec::new() } else { a.route.ahead(EFB_AHEAD_POINTS) },
                    journey: f.and_then(|f| self.journey_simulated(a, f)),
                    modeled: false,
                }
            })
            .collect();
        // The whole flights the model flies (2026-10-13): far out, the
        // arrivals still to enter and the departures flown on.
        aircraft.extend(self.modeled().map(|m| {
            let a = &m.ac;
            let f = m.info.as_ref();
            let arriving = matches!(a.plan, traffic::Plan::Arrival { .. });
            // An arrival's way still to come: its entry, where the
            // simulation takes it (the page draws the great circle to it
            // and on to the field; the approach's points come with the
            // simulated aircraft, a map's worth of them for every flight
            // far out would double the tap).
            let ahead = if arriving {
                vec![m.whole.track.b.pos]
            } else {
                Vec::new()
            };
            TrafficAc {
                id: a.id,
                callsign: a.callsign.clone(),
                spoken: a.spoken.clone(),
                airline: f.map(|f| f.airline.clone()).unwrap_or_default(),
                icao_type: a.icao_type.clone(),
                from: m.whole.origin.clone(),
                to: m.whole.dest.clone(),
                phase: m.words(),
                stand: f.and_then(|f| f.stand.as_deref()).map(|n| traffic::stands::label(n, &names)),
                sched: f.and_then(|f| f.sched_s).map(traffic::rotation::hhmm),
                est: f.and_then(|f| f.est_s).map(traffic::rotation::hhmm),
                lat: a.kin.lat,
                lon: a.kin.lon,
                hdg: a.kin.hdg_deg,
                alt_ft: a.kin.alt_ft,
                on_ground: a.kin.on_ground,
                gs_kts: a.kin.gs_kts,
                ahead,
                journey: Some(self.journey_modeled(m)),
                modeled: true,
            }
        }));
        let taken = self
            .fleet
            .field
            .net
            .stands
            .iter()
            .filter_map(|s| {
                let (a, how) = self.fleet.stand_holder(s)?;
                Some(StandTaken {
                    name: s.name.clone(),
                    by: a.callsign.clone(),
                    how: match how {
                        StandHold::OnIt => "parked",
                        StandHold::TaxiingIn => "taxiing in",
                        StandHold::Assigned => "due",
                    },
                })
            })
            .collect();
        TrafficField {
            icao,
            aircraft,
            taken,
        }
    }

    /// The board's actual times from this tick's events: a departure's
    /// wheels up, an arrival's touchdown.
    fn note_actuals(&mut self, events: &[Event]) {
        let now = self.local_now_s();
        for ev in events {
            let id = match ev {
                Event::Touchdown { id, .. } => *id,
                Event::Phase {
                    id,
                    to: Phase::Climb,
                    ..
                } => *id,
                _ => continue,
            };
            if let Some(a) = self.fleet.get(id) {
                self.actual.entry(a.callsign.clone()).or_insert(now);
            }
        }
    }

    /// The sim's clock jumped (LXGB 2026-10-14: a timetable planned at 21:31
    /// local held only the next morning's flights when the sim was moved to
    /// 11:05): the timetable is planned again from the new clock, its
    /// waiting movements replaced by the new plan's still to come and its
    /// board renewed. Never a reinstall: what flies stays flying (a
    /// callsign already flying is not spawned twice) and an aircraft parked
    /// for its departure keeps its release. `None` when nothing to re-plan.
    fn reanchor(
        &mut self,
        table: &traffic::routes::FieldRoutes,
        zulu_s: f64,
        day: Option<u32>,
        runway: Option<String>,
    ) -> Option<usize> {
        let tt = self.timetable?;
        let (date, _, _) = timetable_clock(table, Some(zulu_s), day);
        let (schedule, board, local_s) = timetable_start(
            &self.fleet.field,
            table,
            tt.density,
            tt.seed,
            Some(zulu_s),
            day,
            runway,
        );
        let flying: std::collections::HashSet<&str> = self
            .fleet
            .aircraft()
            .iter()
            .map(|a| a.callsign.as_str())
            .collect();
        // Only the flights still to come: the plan's placed ones (in motion
        // at its start, `Start`) and its parked fill belong to a fresh field.
        let ahead: Vec<Movement> = schedule
            .into_iter()
            .filter(|m| {
                m.kind != MovementKind::Parked
                    && !m.parked_from_start
                    && m.start.is_none()
                    && !flying.contains(m.callsign.as_str())
            })
            .collect();
        let n = ahead.len();
        let now_s = self.fleet.clock.now_s();
        self.due.retain(|d| d.held.is_some());
        self.add_schedule(ahead, now_s);
        self.set_board(board, local_s - now_s);
        self.actual.clear();
        self.set_summer(date.summer());
        self.timetable = Some(Timetable { date, ..tt });
        Some(n)
    }

    /// More movements, `offset_s` fleet seconds from the clock's start (the
    /// next hour of a generated schedule).
    pub(crate) fn add_schedule(&mut self, schedule: Vec<Movement>, offset_s: f64) {
        self.due
            .extend(schedule.into_iter().map(with_telephony).map(|m| Due {
                at_s: m.t_s + offset_s,
                first_s: m.t_s + offset_s,
                m,
                tries: 0,
                held: None,
                id: None,
                model: None,
            }));
        self.due.sort_by(|a, b| a.at_s.total_cmp(&b.at_s));
    }

    pub(crate) fn fleet(&self) -> &Fleet {
        &self.fleet
    }

    /// Movements still to spawn (a departure waiting parked on its stand
    /// is one; the field's parked aircraft are none).
    pub(crate) fn pending(&self) -> usize {
        self.due.len()
    }

    /// Aircraft in the fleet that are going somewhere (not
    /// [`TrafficAircraft::dormant`]).
    pub(crate) fn active(&self) -> usize {
        self.fleet
            .aircraft()
            .iter()
            .filter(|a| !a.dormant())
            .count()
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
    /// aircraft going somewhere as a TCAS target (ours not in it). A
    /// dormant one, parked on its stand, is no TCAS traffic: the plugin
    /// draws it, the crew's picture does not carry it.
    pub(crate) fn targets(&self) -> Vec<TcasTarget> {
        self.fleet
            .aircraft()
            .iter()
            .filter(|a| !a.dormant())
            .map(tcas_target)
            .collect()
    }

    /// Where the fleet's parked aircraft stand (dormant, out of
    /// [`Self::targets`]): our own stand route passes over them (#243).
    pub(crate) fn stands_taken(&self) -> Vec<(f64, f64)> {
        self.fleet
            .aircraft()
            .iter()
            .filter(|a| a.dormant())
            .map(|a| a.kin.pos())
            .collect()
    }

    /// One tick: spawn what is due, step the fleet `dt` seconds of sim time
    /// (clamped and substepped by its clock; nothing while frozen), and
    /// trade with the station. `now` is the station's clock for this tick.
    pub(crate) fn tick(&mut self, own: Option<&OwnState>, dt: f64, now: Instant) -> TickOut {
        if self.desk.is_some() {
            return self.tick_desk(own, dt);
        }
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
        // A dormant aircraft has said nothing: no strip, no state, until it
        // is released and calls.
        for ac in self.fleet.aircraft().iter().filter(|a| !a.dormant()) {
            crate::world::note_traffic_state(ac.id, aircraft_state(ac, now));
        }
        self.noted = self
            .fleet
            .aircraft()
            .iter()
            .filter(|a| !a.dormant())
            .map(|a| a.id)
            .collect();
        for ev in &events {
            match ev {
                Event::Call { id, call } => {
                    // An ordered go-around is acknowledged by its readback
                    // ("Going around, ..."): the report would say it twice.
                    let ordered = orders.get(id).is_some_and(|o| o.go_around);
                    if *call == Call::GoingAround && ordered {
                        continue;
                    }
                    let on = self.channel(*id, call);
                    let at = self.fleet.clock.now_s();
                    self.outbox.push_on(*id, call.clone(), on, at);
                }
                // A flight turned round on its stand is over: its strip
                // retires; the airframe stays, dormant, for its next flight.
                Event::Despawn { id, .. } | Event::TurnedRound { id } => {
                    crate::world::retire_traffic(*id);
                    self.outbox.forget(*id);
                }
                _ => {}
            }
        }
        // The frequency: what is due keys now, from where the flight is as
        // it keys (the rest waits for a clear frequency, `Outbox`).
        let now_s = self.fleet.clock.now_s();
        let fleet = &self.fleet;
        let named = !self.tower_name.is_empty();
        let airtime = |id: u32, call: &Call| {
            let spoken = fleet.get(id).map_or("", |a| a.spoken.as_str());
            let runway = orders.get(&id).and_then(|o| o.runway.as_deref());
            // The field's name ahead of "Tower" is a word or two more.
            airtime_s(&call.text(spoken, runway)) + if named { 1.0 / WORDS_PER_S } else { 0.0 }
        };
        let keyed = self.outbox.due_on(now_s, airtime);
        for Keyed {
            id,
            call,
            on,
            waited_s,
            clear_s,
        } in keyed
        {
            let Some(ac) = self.fleet.get(id) else {
                continue;
            };
            let order = orders.get(&id);
            let runway = order.and_then(|o| o.runway.as_deref()).or(ac.plan.runway());
            let text = self.call_text(&ac.spoken, &call, runway);
            crate::world::post_traffic_call(TrafficCall {
                id,
                spoken: ac.spoken.clone(),
                text: text.clone(),
                state: aircraft_state(ac, now),
                picture: self.picture_for(id, own),
                approach: call.to_approach(),
            });
            out.transmissions.push(Transmission {
                id,
                callsign: ac.callsign.clone(),
                text,
                safety: is_safety_call(&call),
                on,
                waited_s,
                clear_s,
            });
        }
        // The runway watch in landing order, the nearest the threshold
        // first: a withheld landing is released to the arrival AHEAD before
        // the one behind it, whose clearance would otherwise hold the
        // runway in the book against its leader. LFPG 09R (the hour sweep,
        // 2026-10-10): a departure off, the ATR 3.5 nm behind was cleared
        // first (fleet order) and the A320 ahead of it went around uncleared
        // at 300 ft.
        let mut watch: Vec<&TrafficAircraft> = self
            .fleet
            .aircraft()
            .iter()
            .filter(|a| !a.dormant())
            .collect();
        watch.sort_by(|a, b| {
            let to = |x: &TrafficAircraft| x.slot().map_or(f64::INFINITY, |s| s.to_thr_s);
            to(a).total_cmp(&to(b))
        });
        for ac in watch {
            let picture = self.picture_for(ac.id, own);
            for line in crate::world::traffic_tick(ac.id, &picture, now) {
                out.tower.push((ac.id, line));
            }
            if let Some((line, who)) = crate::world::traffic_position_tick(ac.id) {
                out.positions.push((ac.id, line, who));
            }
        }
        // The tower's own lines hold their frequency as long as they take:
        // the next caller waits them out.
        for (_, line) in &out.tower {
            let until = self
                .outbox
                .clear_from(crate::voice::StationSpeaker::Tower, now_s)
                + airtime_s(line);
            self.outbox
                .occupy(crate::voice::StationSpeaker::Tower, until);
        }
        for (_, line, who) in &out.positions {
            let until = self.outbox.clear_from(*who, now_s) + airtime_s(line);
            self.outbox.occupy(*who, until);
        }
        self.note_actuals(&events);
        out.events = events;
        self.fly_models();
        out
    }

    /// One tick of a field worked by its desk ([`Link::make_desk`]): spawn
    /// what is due, the desk's clearances, the fleet stepped on them, its
    /// calls heard by the desk. Nothing reaches the station or the
    /// frequency: no strip, no state noted, no call posted, no line said
    /// (stage 3 runs these fields with no radio).
    fn tick_desk(&mut self, own: Option<&OwnState>, dt: f64) -> TickOut {
        let mut out = TickOut::default();
        self.spawn_due(own, &mut out);
        let Some(desk) = self.desk.as_mut() else {
            self.fly_models();
            return out;
        };
        desk.tick(&self.fleet);
        let orders: HashMap<u32, Order> = self
            .fleet
            .aircraft()
            .iter()
            .filter_map(|a| desk.order(a.id).map(|o| (a.id, o)))
            .collect();
        for (id, o) in &orders {
            let now_seen = (o.state, o.runway.clone());
            if self.seen_orders.get(id) != Some(&now_seen) {
                out.orders.push((*id, o.state, o.runway.clone()));
                self.seen_orders.insert(*id, now_seen);
            }
        }
        self.seen_orders.retain(|id, _| orders.contains_key(id));
        let events = self
            .fleet
            .step(dt, own.map(OwnState::own_target), &|id| orders.get(&id).cloned());
        desk.hear(&events);
        self.note_actuals(&events);
        out.events = events;
        self.fly_models();
        out
    }

    /// Our own pilot is on position `on`'s frequency now (the PTT down, a
    /// line being routed, or the tower answering us): no traffic keys over
    /// it, and the next caller waits [`OWN_TAIL_S`] and its gap after.
    pub(crate) fn note_own_transmission(&mut self, on: crate::voice::StationSpeaker) {
        let until = self.fleet.clock.now_s() + OWN_TAIL_S;
        self.outbox.occupy(on, until);
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
            self.outbox.forget(*id);
        }
        ids
    }

    /// The field's parked aircraft onto their stands, and every departure
    /// parked from the start onto its own, parked and dark
    /// ([`Movement::park`]: never on our stand nor within 60 m of us, never
    /// on a stand somebody stands on). A departure refused here spawns at
    /// its time as any other; a refused parked aircraft is not drawn.
    fn park_at_start(&mut self, own: Option<&OwnState>, out: &mut TickOut) {
        let own = own.map(|o| (o.lat, o.lon));
        let ctx = |fleet: &Fleet| SpawnCtx {
            runway: None,
            own,
            occupied: fleet.stands_held(),
            zulu_s: None,
            parked_airframes: 0,
        };
        for i in 0..self.due.len() {
            if !self.due[i].m.parked_from_start {
                continue;
            }
            let c = ctx(&self.fleet);
            if let Ok(ac) = self.due[i].m.park(self.ids.peek(), &self.fleet.field, &c) {
                let (id, cs) = (ac.id, ac.callsign.clone());
                self.note_type(&ac.callsign, &ac.icao_type);
                if self.fleet.spawn(ac) {
                    self.ids.bump();
                    self.due[i].held = Some(id);
                    out.parked.push((id, cs));
                }
            }
        }
        for m in std::mem::take(&mut self.to_park) {
            let c = ctx(&self.fleet);
            if let Ok(ac) = m.park(self.ids.peek(), &self.fleet.field, &c) {
                let (id, cs) = (ac.id, ac.callsign.clone());
                self.note_type(&ac.callsign, &ac.icao_type);
                if self.fleet.spawn(ac) {
                    self.ids.bump();
                    out.parked.push((id, cs));
                }
            }
        }
    }

    /// A call in words, addressed to the field's tower (or approach unit)
    /// by name where it opens with "Tower" ("Approach"): the station's own
    /// replies and handoffs name its radar unit "<field> Approach".
    fn call_text(&self, spoken: &str, call: &Call, runway: Option<&str>) -> String {
        let text = call.text(spoken, runway);
        // Ground's calls name the position that takes them, as [`Self::channel`]
        // routes them: Ground where the field has one, else the Tower. LEMG
        // (2026-10-13): "Malaga Tower, Ryanair ..., request taxi" on Ground.
        let text = if call.to_ground() {
            let to = if self.has_ground { "Ground, " } else { "Tower, " };
            match text.strip_prefix("Tower, ").or_else(|| text.strip_prefix("Ground, ")) {
                Some(rest) => format!("{to}{rest}"),
                None => text,
            }
        } else {
            text
        };
        if self.tower_name.is_empty() {
            return text;
        }
        if let Some(rest) = text.strip_prefix("Tower, ") {
            format!("{} Tower, {rest}", self.tower_name)
        } else if let Some(rest) = text.strip_prefix("Ground, ") {
            format!("{} Ground, {rest}", self.tower_name)
        } else if let Some(rest) = text.strip_prefix("Approach, ") {
            format!("{} Approach, {rest}", self.tower_name)
        } else {
            text
        }
    }

    /// The frequency a call is keyed on, for its pacing: the approach
    /// unit's for an instrument arrival's calls; a readback (or its "say
    /// again") on the position working the flight, which gave the
    /// instruction; Ground's for the push, the taxi out and the taxi in and
    /// a crossing of a runway not in use, where the field has Ground; else
    /// the Tower's. The station routes each call to the position that owns
    /// it (`Station::owner`); this mirrors it so the pacing is per
    /// frequency, the speech is the station's (`overhear_replies`).
    fn channel(&self, id: u32, call: &Call) -> crate::voice::StationSpeaker {
        use crate::voice::StationSpeaker as P;
        if call.to_approach() {
            return P::Approach;
        }
        if matches!(call, Call::Readback(_) | Call::SayAgain) {
            return match crate::world::traffic_working(id) {
                Some(P::Ground | P::Delivery) => P::Ground,
                _ => P::Tower,
            };
        }
        if !self.has_ground {
            return P::Tower;
        }
        if call.to_ground() {
            return P::Ground;
        }
        if let Call::RequestCrossing { runway } = call {
            let field = &self.fleet.field;
            let in_use = self.tower_runway.clone().or_else(station_runway);
            let same = match (field.strip(runway), in_use.as_deref().and_then(|r| field.strip(r))) {
                (Some(a), Some(b)) => std::ptr::eq(a, b),
                _ => false,
            };
            return if same { P::Tower } else { P::Ground };
        }
        P::Tower
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

    /// A departure or a circuit flown by a dormant airframe on its stand
    /// (`Fleet::ready_to_leave`): an airliner for a departure, if the
    /// runway in use holds its type, a light aircraft for a circuit; not
    /// one a departure is already waiting on. A turned-round airliner
    /// leaves as its carrier's next flight (`schedule::next_flight`); one
    /// parked from the start, or a light aircraft, keeps its callsign.
    /// (id, callsign, where it came from), or `None`: spawn as before.
    fn take_parked(&mut self, m: &Movement) -> Option<(u32, String, &'static str)> {
        // A flight placed mid-flow is where its start says, never a parked
        // airframe's.
        if !matches!(m.kind, MovementKind::Departure | MovementKind::Circuit) || m.start.is_some() {
            return None;
        }
        let plan = m.plan(&self.fleet.field).ok()?;
        // A rotation's departure is flown by ITS inbound's airframe, under
        // its own number (2026-10-13), once that one has turned round.
        if let Some(inbound) = m.flight.as_ref().and_then(|f| f.inbound.as_deref()) {
            let id = self
                .fleet
                .aircraft()
                .iter()
                .find(|a| a.callsign == inbound && a.dormant())
                .map(|a| a.id)?;
            let icao_type = self.fleet.get(id)?.icao_type.clone();
            if !self.fleet.release_as(id, plan, &m.callsign, &m.spoken) {
                return None;
            }
            self.note_type(&m.callsign, &icao_type);
            return Some((id, m.callsign.clone(), "rotation"));
        }
        let runway_m = self.fleet.field.end(&m.runway)?.length_m;
        let light = m.kind == MovementKind::Circuit;
        // Nor an airframe a rotation's departure is waiting for.
        let waited: Vec<&str> = self
            .due
            .iter()
            .filter_map(|d| d.m.flight.as_ref()?.inbound.as_deref())
            .collect();
        let held: Vec<u32> = self
            .due
            .iter()
            .filter_map(|d| d.held)
            .chain(
                self.fleet
                    .aircraft()
                    .iter()
                    .filter(|a| waited.contains(&a.callsign.as_str()))
                    .map(|a| a.id),
            )
            .collect();
        let fits = |a: &TrafficAircraft| airframe_fits(a, light, runway_m);
        let id = self.fleet.ready_to_leave(&held, &fits)?;
        let a = self.fleet.get(id)?;
        let turned = self.fleet.turned_round(id);
        let (icao, old_cs) = (self.fleet.field.icao.clone(), a.callsign.clone());
        let in_use = |cs: &str| self.fleet.aircraft().iter().any(|x| x.callsign == cs);
        let (cs, spoken) = (0..4)
            .filter(|_| turned)
            .filter_map(|k| {
                traffic::schedule::next_flight(
                    &icao,
                    &old_cs,
                    &mut traffic::XorShift64::new(m.seed ^ k),
                )
            })
            .find(|(cs, _)| !in_use(cs))
            .unwrap_or_else(|| (a.callsign.clone(), a.spoken.clone()));
        let icao_type = a.icao_type.clone();
        if !self.fleet.release_as(id, plan, &cs, &spoken) {
            return None;
        }
        self.note_type(&cs, &icao_type);
        Some((id, cs, if turned { "turned_round" } else { "parked" }))
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
        if self.fleet.clock.frozen {
            return;
        }
        if !self.parked {
            self.parked = true;
            self.park_at_start(own, out);
        }
        if self.due.first().is_none_or(|d| d.at_s > now_s) {
            return;
        }
        let ctx = SpawnCtx {
            runway: None,
            own: own.map(|o| (o.lat, o.lon)),
            // The stands arrivals are taxiing in to are taken too: a
            // departure spawned on one, and its arrival drove in on top.
            occupied: self.fleet.stands_held(),
            zulu_s: None,
            parked_airframes: 0,
        };
        let mut later = Vec::new();
        while self.due.first().is_some_and(|d| d.at_s <= now_s) {
            let mut d = self.due.remove(0);
            let runway = self.runway_now();
            match &runway {
                Some(r) if self.fleet.field.end(r).is_some() => d.m.runway = r.clone(),
                None if d.m.kind == MovementKind::Arrival && now_s < d.m.t_s + ARRIVAL_WAIT_S => {
                    d.at_s = now_s + 5.0;
                    later.push(d);
                    continue;
                }
                _ => {}
            }
            // A departure waiting on its stand starts up there.
            if let Some(id) = d.held {
                let plan = d.m.plan(&self.fleet.field);
                if let Ok(plan) = plan {
                    if self.fleet.release(id, plan) {
                        self.note_type(&d.m.callsign, &d.m.icao_type);
                        out.spawned.push((id, d.m.callsign.clone()));
                        out.from_stand.push((id, "parked"));
                        continue;
                    }
                }
            }
            // Else an airframe already standing on its stand, ready to fly
            // again, takes the flight; only with none does one appear on a
            // free stand from nothing (a field with nobody parked yet, or
            // every parked airframe still turning round).
            if let Some((id, cs, from)) = self.take_parked(&d.m) {
                out.spawned.push((id, cs));
                out.from_stand.push((id, from));
                continue;
            }
            // A rotation's departure whose inbound is still on its way (to
            // spawn, in the air, taxiing in) waits for it, late as a real
            // one is; with its inbound gone for good it flies as any other.
            if let Some(inbound) = d.m.flight.as_ref().and_then(|f| f.inbound.clone()) {
                let coming = self
                    .due
                    .iter()
                    .chain(&later)
                    .any(|x| x.m.callsign == inbound)
                    || self.fleet.aircraft().iter().any(|a| a.callsign == inbound);
                if coming && now_s < d.m.t_s + INBOUND_WAIT_S {
                    d.at_s = now_s + SPAWN_RETRY_S;
                    later.push(d);
                    continue;
                }
                if let Some(f) = d.m.flight.as_mut() {
                    f.inbound = None;
                }
                if let Some((id, cs, from)) = self.take_parked(&d.m) {
                    out.spawned.push((id, cs));
                    out.from_stand.push((id, from));
                    continue;
                }
            }
            let id = d.id.unwrap_or_else(|| self.ids.peek());
            let spawned = d.m.spawn(id, &self.fleet.field, &ctx);
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
                    self.note_type(&ac.callsign, &ac.icao_type);
                    let entered = whole::entered(&d, &ac, now_s, self.icao());
                    if self.fleet.spawn(ac) {
                        if d.id.is_none() {
                            self.ids.bump();
                        }
                        if let Some(w) = entered {
                            self.entered.insert(id, (cs.clone(), w));
                        }
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

/// A movement says its callsign as its TELEPHONY: a bench spec gives a raw
/// "RYR4521" with no `spoken`, the tower answered "Ryanair four five two
/// one" and the readback, finding no "RYR4521" in that line, appended it
/// raw ("..., Ryanair four five two one, RYR4521"; the phraseology pass,
/// 2026-10-10). Expanded here, once, the flight and the tower agree.
fn with_telephony(mut m: Movement) -> Movement {
    m.spoken = crate::callsign::spoken_if_raw(&m.spoken);
    m
}

/// The command loop's half: the traffic desks synced with what the ticks
/// said, then every posted traffic call answered on its own desk, in
/// order. A reply with an empty line is an accepted readback.
/// Each reply comes with the moment its call was keyed and the caller's
/// spoken callsign (its ghost voice).
pub(crate) fn drain_calls(station: &mut Station) -> Vec<(Instant, String, TrafficReply)> {
    station.sync_traffic_desks();
    crate::world::take_traffic_calls()
        .iter()
        .map(|c| (c.state.at, c.spoken.clone(), station.answer_traffic(c)))
        .collect()
}

/// Our traffic's exchanges onto the frequency of the position that heard
/// each (Ground's taxi chatter on Ground's, never on the Tower's: the
/// ONE-frequency rule): the call in the flight's own ghost voice as it
/// was keyed, the answer in the position's voice just after it (an
/// accepted readback says nothing).
pub(crate) fn overhear_replies(
    voice: &crate::voice::Voice,
    replies: Vec<(Instant, String, TrafficReply)>,
) {
    for (keyed, spoken, reply) in replies {
        let seed = crate::voice::station_voice_seed(&spoken);
        let on = reply.speaker;
        let ghost = crate::voice::StationSpeaker::Ghost { seed };
        voice.overhear_checked(on, ghost, &reply.call, keyed, still_valid(reply.id));
        if !reply.line.is_empty() {
            let made = keyed + std::time::Duration::from_millis(1);
            voice.overhear_checked(on, reply.speaker, reply.line, made, still_valid(reply.id));
        }
    }
}

/// What a traffic flight's strip held when a line about it was made, for
/// the line's turn on the frequency: a queued line whose clearance has
/// since been withdrawn is overtaken ([`overtaken`]).
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct StripMark {
    pub(crate) state: crate::tower::State,
    pub(crate) runway: String,
    pub(crate) go_around: bool,
}

impl StripMark {
    pub(crate) fn of(o: &TrafficOrder) -> StripMark {
        StripMark {
            state: o.state,
            runway: o.runway.clone(),
            go_around: o.go_around,
        }
    }
}

/// Is a line made when the strip read `was` overtaken now that it reads
/// `now`? Only a WITHDRAWN clearance overtakes a line (#260: one flight's
/// "going around", the tower's "cleared to land" to that same flight and
/// a taxi clearance all queued together, t=1791477548): sent round (the
/// order, or the landing clearance gone back to inbound), a takeoff
/// cancelled, a line-up cancelled, or the runway changed. The strip moving
/// ON (taxi to line-up to takeoff, a landing to the taxi in) or closing
/// (the flight handed off and gone) overtakes nothing: those words were
/// said. A go-around order is never overtaken by the go-around it ordered.
pub(crate) fn overtaken(was: &StripMark, now: Option<&StripMark>) -> Option<&'static str> {
    use crate::tower::State as S;
    let now = now?;
    if was.go_around {
        return None;
    }
    if now.go_around {
        return Some("go_around_ordered");
    }
    match (was.state, now.state) {
        (S::ClearedToLand, S::Inbound) => return Some("went_around"),
        (S::TakeoffCleared, S::LinedUp | S::TaxiCleared) => return Some("takeoff_cancelled"),
        (S::LinedUp, S::TaxiCleared) => return Some("line_up_cancelled"),
        _ => {}
    }
    if !was.runway.is_empty() && !now.runway.is_empty() && was.runway != now.runway {
        return Some("runway_changed");
    }
    None
}

/// The re-validation a line about traffic flight `id` carries to its turn
/// on the frequency: its strip as of now, compared with the strip as the
/// line airs ([`overtaken`]). `None` with no strip open.
pub(crate) fn still_valid(id: u32) -> Option<crate::voice::StillValid> {
    let was = StripMark::of(&crate::world::traffic_order(id)?);
    Some(Box::new(move || {
        let now = crate::world::traffic_order(id).map(|o| StripMark::of(&o));
        overtaken(&was, now.as_ref())
    }))
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
        line_up_behind: o.line_up_condition == Some(false),
        go_around: o.go_around,
        readback_owed: o.readback_owed.clone(),
        cross_cleared: o.crossing.clone(),
        push_approved: o.push.is_some(),
        push_facing: o.push.flatten(),
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

/// The monitor's side of the traffic: a link per field simulated (built
/// off the tick, one at a time: two apt.dat scans each), the clock, the
/// fast-forward latch, the sink. Owned by `WorldTick`, never a static.
///
/// SEVERAL FLEETS AT ONCE (multi-field traffic stage 3, 2026-10-13): the
/// installed station's field, worked by the station as it always was, and
/// every field with a timetable within [`SIM_RANGE_NM`] of us, each worked
/// by its own runway desk ([`Link::make_desk`]), silent and kept out of the
/// party line. A field past [`SIM_OUT_NM`] retires its fleet. When the
/// pilot's station moves to a simulated field, that field's fleet is
/// handed to the station (its moving aircraft leave the frequency, the
/// reinstall rule) and the old field's to a desk.
#[derive(Default)]
pub(crate) struct TrafficWorld {
    /// The fleets, by ICAO upper case: in ICAO order, so the tick is
    /// deterministic.
    links: BTreeMap<String, Link>,
    /// The installed station's field, the one the station works.
    active: Option<String>,
    /// A link being built, off the tick, and its field.
    loading: Option<(String, std::sync::mpsc::Receiver<Option<Link>>)>,
    /// Fields whose build came back empty: not tried again while in range.
    failed: BTreeSet<String>,
    /// The sim's running time at the last tick.
    last_sim_s: Option<f64>,
    /// Frozen for time acceleration (said once per spell).
    fast: bool,
    /// Every fleet's ids, one counter for the session ([`Ids`]): a link
    /// built later never reuses an id an earlier one handed out.
    ids: Ids,
    /// The packed sink to the plugin, started with the first link.
    sink: Option<crate::traffic_sink::TrafficSink>,
    /// Who of the simulated aircraft the sink draws (stage 1 of
    /// multi-field traffic, 2026-10-13), and its counts for the blackbox.
    cull: crate::traffic_sink::DrawCull,
    draw_log: DrawLog,
    /// The Board tab as last sent, and when.
    board_last: String,
    board_sent: Option<Instant>,
    /// When the EFB's boards were last tapped (`note_efb_boards`).
    efb_boards_at: Option<Instant>,
    /// When the EFB's map traffic was last tapped (`note_efb_traffic`).
    efb_traffic_at: Option<Instant>,
    /// The Board tab's other field when nobody simulates it (2026-10-13):
    /// its board by the clock, built off the tick once picked, and one
    /// being built.
    remote: Option<RemoteBoard>,
    remote_loading: Option<(String, std::sync::mpsc::Receiver<Option<RemoteBoard>>)>,
    /// The plugin has no field pick (an older one): not asked again before
    /// this.
    board_pick_quiet_until: Option<Instant>,
    /// When the sim's clock was last compared with the timetables'
    /// ([`TrafficWorld::follow_sim_clock`]).
    clock_checked: Option<Instant>,
}

/// The draw cull's counts in the blackbox (`traffic` `draw`): simulated
/// against drawn, the frame-cost side of multi-field traffic (stage 1,
/// 2026-10-13), read beside `api_load` and the fps dataref. On a change at
/// most every [`DRAW_LOG_GAP`], and every [`DRAW_LOG_EVERY`] regardless:
/// the monitor ticks at about 3 Hz and a record a tick would drown the log.
#[derive(Debug, Default)]
struct DrawLog {
    last: Option<(usize, usize, usize, usize)>,
    at: Option<Instant>,
}

const DRAW_LOG_GAP: std::time::Duration = std::time::Duration::from_secs(5);
const DRAW_LOG_EVERY: std::time::Duration = std::time::Duration::from_secs(30);

impl DrawLog {
    fn note(&mut self, simulated: usize, modeled: usize, drawn: &[&TrafficAircraft], now: Instant) {
        let parked = drawn.iter().filter(|a| a.dormant()).count();
        let counts = (simulated, modeled, drawn.len(), parked);
        let since = self.at.map(|t| now.saturating_duration_since(t));
        let due = match since {
            None => true,
            Some(d) => (self.last != Some(counts) && d >= DRAW_LOG_GAP) || d >= DRAW_LOG_EVERY,
        };
        if due {
            self.last = Some(counts);
            self.at = Some(now);
            crate::blackbox::record(
                "traffic",
                serde_json::json!({ "event": "draw", "simulated": simulated,
                    "modeled": modeled, "drawn": drawn.len(), "parked_drawn": parked }),
            );
        }
    }
}

/// The Board tab is resent at most this often on a change, and this often
/// regardless.
const BOARD_MIN_GAP: std::time::Duration = std::time::Duration::from_secs(2);
const BOARD_RESEND: std::time::Duration = std::time::Duration::from_secs(10);
/// The most route points ahead each aircraft carries to the EFB's map.
const EFB_AHEAD_POINTS: usize = 24;
/// How often the EFB's boards are rebuilt ([`TrafficWorld::note_efb_boards`]).
const EFB_BOARDS_EVERY: std::time::Duration = std::time::Duration::from_secs(5);
/// How often the EFB's map traffic is rebuilt ([`TrafficWorld::note_efb_traffic`]).
const EFB_TRAFFIC_EVERY: std::time::Duration = std::time::Duration::from_secs(1);

/// Our own flight on the board, from the plan on the shelf: a departure
/// from `field`, or an arrival to it; no times (the plan's are not the
/// field's clock), status Scheduled.
fn own_board_flight(field: &str) -> Option<traffic::board::BoardFlight> {
    let plan = crate::simbrief::loaded_plan()?;
    let dir = if plan.origin.eq_ignore_ascii_case(field) {
        traffic::board::Dir::Departure
    } else if plan.destination.eq_ignore_ascii_case(field) {
        traffic::board::Dir::Arrival
    } else {
        return None;
    };
    let other = match dir {
        traffic::board::Dir::Departure => plan.destination.clone(),
        traffic::board::Dir::Arrival => plan.origin.clone(),
    };
    let callsign = plan.callsign.clone().unwrap_or_else(|| "OWN".to_string());
    Some(traffic::board::BoardFlight {
        dir,
        callsign: callsign.clone(),
        icao_type: plan.aircraft.clone(),
        info: traffic::FlightInfo {
            number: callsign,
            airline: "Our flight".to_string(),
            city: other.clone(),
            other,
            ..traffic::FlightInfo::default()
        },
        own: true,
    })
}

impl TrafficWorld {
    /// One monitor tick. Nothing at all unless `XCOPILOT_TRAFFIC=ours`, ATC
    /// is on and a station is installed: every fleet is dropped (every
    /// strip it opened retired, the picture withdrawn) the moment any of
    /// those stops being true. `sim_speed` is the monitor's latest read.
    /// The tick's output is the station's field's.
    pub(crate) async fn tick(
        &mut self,
        xp: &crate::xplane::XPlaneClient,
        voice: &crate::voice::Voice,
        snap: &crate::monitor::Snapshot,
        sim_speed: Option<f64>,
        now: Instant,
        status: Option<&crate::plugin_status::StatusSink>,
    ) -> Option<TickOut> {
        let active = wanted_field(
            TrafficMode::now(),
            crate::mode::atc_enabled(),
            crate::party_line::installed_icao(),
        )
        .map(|f| f.trim().to_ascii_uppercase());
        let Some(active) = active else {
            if self.active.is_some() || !self.links.is_empty() || self.loading.is_some() {
                self.stand_down();
            }
            return None;
        };
        let here = (snap.lat, snap.lon);
        let running: BTreeSet<String> = self.links.keys().cloned().collect();
        let wanted = wanted_fields(&active, here, &running);
        if self.active.as_deref() != Some(active.as_str()) {
            self.change_station(&active, &wanted);
        }
        let gone: Vec<String> = self
            .links
            .keys()
            .filter(|k| !wanted.contains(*k))
            .cloned()
            .collect();
        for icao in gone {
            if let Some(link) = self.links.remove(&icao) {
                retire_link(&link, "out_of_range");
            }
        }
        self.failed.retain(|f| wanted.contains(f));
        self.take_built(&active, &wanted);
        if self.loading.is_none() {
            let built: BTreeSet<String> = self
                .links
                .keys()
                .chain(self.failed.iter())
                .cloned()
                .collect();
            if let Some(icao) = next_build(&active, &wanted, &built) {
                // The schedule follows the field's day (`traffic::rate`):
                // the sim's zulu time, read only when a schedule is made.
                let zulu = sim_zulu_s(xp).await;
                let day = sim_date_days(xp).await;
                let station = icao == active;
                self.loading = Some((icao.clone(), self.build(&icao, station, zulu, day)));
            }
        }
        if self.links.is_empty() {
            return None;
        }
        // Time acceleration freezes the fleets (v1), said once per spell.
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
        for link in self.links.values_mut() {
            link.set_frozen(fast);
        }
        // The fleets' clock is the sim's (a failed read is no time), and our
        // height in the tower's picture is true feet MSL (the snapshot's is
        // the altimeter's): one batched read, only while a fleet flies.
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
        // Our position for the tower's book, which asks for a traffic
        // flight's holds without it: how far out our landing clearance is.
        crate::world::note_own_state(crate::party_line::AircraftState {
            lat: snap.lat,
            lon: snap.lon,
            alt_ft,
            on_ground: snap.on_ground,
            gs_kts: snap.groundspeed_kts,
            psi_deg: None,
            helicopter: false,
            at: now,
        });
        // Our own pilot on the frequency (keying, a line being routed, the
        // tower answering us): the station's traffic waits it out, never
        // keys over it.
        if voice.frequency_ours() || crate::pollers::command_waiting() {
            if let Some(link) = self.links.get_mut(&active) {
                let on = voice
                    .tuned_station()
                    .unwrap_or(crate::voice::StationSpeaker::Tower);
                link.note_own_transmission(on);
            }
        }
        if !fast {
            self.follow_sim_clock(xp, &active, now).await;
        }
        let mut station_out = None;
        let fields: Vec<String> = self.links.keys().cloned().collect();
        for icao in fields {
            let Some(link) = self.links.get_mut(&icao) else {
                continue;
            };
            let out = link.tick(Some(&own), dt, now);
            // The next hour of a generated schedule, once this one is spent;
            // a timetable's top-up every hour, its day never spent.
            let tops_up = link.tops_up();
            if link.pending() == 0 || tops_up {
                let start = link.fleet().clock.now_s();
                let next = link
                    .hours
                    .filter(|h| start >= f64::from(*h) * traffic::schedule::HORIZON_S);
                if let Some(h) = next {
                    link.hours = Some(h + 1);
                    let zulu = sim_zulu_s(xp).await;
                    let ms = if tops_up {
                        link.top_up_hour(u64::from(h + 1), Some((own.lat, own.lon)), zulu)
                    } else {
                        generate_for(link.fleet(), u64::from(h + 1), &own, zulu)
                    };
                    link.add_schedule(ms, start);
                }
            }
            record(&out, link);
            if icao == active {
                station_out = Some(out);
            }
        }
        if let Some(link) = self.links.get(&active) {
            crate::party_line::publish_traffic_picture(Some(link.targets()));
            crate::party_line::publish_stands_taken(link.stands_taken());
        }
        if self.sink.is_none() {
            self.sink = crate::traffic_sink::TrafficSink::start(
                crate::settings::live().xplane_base_url.clone(),
            );
        }
        if let Some(sink) = self.sink.as_mut() {
            // The simulated aircraft and, inside the same ring, the whole
            // flights the model flies (a departure past 12 nm, an arrival
            // still to enter): no aircraft pops in or out of TCAS range.
            let simulated = self.links.values().map(|l| l.fleet().aircraft().len()).sum();
            let all: Vec<&TrafficAircraft> = self
                .links
                .values()
                .flat_map(|l| l.fleet().aircraft().iter().chain(l.modeled_drawn()))
                .collect();
            let drawn = self.cull.select(Some(here), &all);
            sink.publish(&drawn, self.last_sim_s.unwrap_or(0.0));
            self.draw_log
                .note(simulated, all.len() - simulated, &drawn, now);
        }
        // The frequencies: the tower's runway-watch lines in the tower's
        // voice, the positions' handoffs in theirs, each on its own
        // frequency. The calls themselves are said with their answers, on
        // the position that heard them (`overhear_replies`). Only the
        // station's field speaks (the desks have no radio, stage 3).
        if let Some(out) = &station_out {
            let tower = crate::voice::StationSpeaker::Tower;
            for (id, line) in &out.tower {
                voice.overhear_checked(tower, tower, line, now, still_valid(*id));
            }
            for (id, line, who) in &out.positions {
                voice.overhear_checked(*who, *who, line, now, still_valid(*id));
            }
        }
        if let Some(s) = status {
            self.publish_board(xp, s, here, now).await;
        }
        self.note_efb_boards(now);
        self.note_efb_traffic(now);
        station_out
    }

    /// A timetable follows the sim's clock (#280): once a minute the sim's
    /// date and zulu time are read and each timetable link whose clock has
    /// parted from them by [`CLOCK_JUMP_S`] is planned again
    /// ([`Link::reanchor`]). Not while the fleets are frozen: the check
    /// after a fast spell catches the field up to the sim.
    async fn follow_sim_clock(
        &mut self,
        xp: &crate::xplane::XPlaneClient,
        active: &str,
        now: Instant,
    ) {
        if self.clock_checked.is_some_and(|t| now.duration_since(t) < CLOCK_CHECK_EVERY) {
            return;
        }
        if !self.links.values().any(|l| l.timetable.is_some()) {
            return;
        }
        self.clock_checked = Some(now);
        let (zulu, day) = tokio::join!(sim_zulu_s(xp), sim_date_days(xp));
        // A failed read is no measurement.
        let Some(zulu) = zulu else { return };
        for (icao, link) in self.links.iter_mut() {
            let (Some(tt), Some(table)) = (link.timetable, traffic::routes::field_routes(icao))
            else {
                continue;
            };
            let (date, _, local_s) = timetable_clock(table, Some(zulu), day);
            let jump = clock_jump_s(tt.date, link.local_now_s(), date, local_s);
            if jump.abs() < CLOCK_JUMP_S {
                continue;
            }
            let from = link.local_now_s();
            let runway = if icao == active { station_runway() } else { None };
            if let Some(n) = link.reanchor(table, zulu, day, runway) {
                tracing::info!(%icao, jump_s = jump.round(), movements = n,
                    "traffic: the sim's clock jumped; the timetable planned again");
                crate::blackbox::record(
                    "traffic",
                    serde_json::json!({
                        "event": "reanchored", "field": icao, "jump_s": jump.round(),
                        "from_local_s": from.round(), "to_local_s": local_s.round(),
                        "movements": n, "zulu_s": zulu,
                    }),
                );
            }
        }
    }

    /// The EFB's Map tab (2026-10-13): every simulated field's aircraft
    /// and held stands ([`Link::efb_traffic`]), tapped into `efb::feed`
    /// about once a second while the EFB serves (no reads).
    fn note_efb_traffic(&mut self, now: Instant) {
        if self
            .efb_traffic_at
            .is_some_and(|t| now.saturating_duration_since(t) < EFB_TRAFFIC_EVERY)
            || crate::efb::serving_url().is_none()
        {
            return;
        }
        self.efb_traffic_at = Some(now);
        crate::efb::feed::note_traffic(self.links.values().map(Link::efb_traffic).collect());
    }

    /// The EFB's Board card (2026-10-13): every simulated field's board,
    /// the same payload the panel gets, tapped into `efb::feed` every 5 s
    /// while the EFB is serving (no reads: the fleets are already here).
    fn note_efb_boards(&mut self, now: Instant) {
        if self
            .efb_boards_at
            .is_some_and(|t| now.saturating_duration_since(t) < EFB_BOARDS_EVERY)
            || crate::efb::serving_url().is_none()
        {
            return;
        }
        self.efb_boards_at = Some(now);
        let boards = self
            .links
            .iter()
            .map(|(icao, link)| {
                (
                    icao.clone(),
                    link.board_payload(own_board_flight(link.icao()), &[]),
                )
            })
            .collect();
        crate::efb::feed::note_boards(boards);
    }

    /// The pilot's station moved to `active`: the old station's field is
    /// handed to a desk while it stays in range (else retired), the new
    /// one's fleet, if simulated already, to the station; a build for
    /// another field gives way to the station's own.
    fn change_station(&mut self, active: &str, wanted: &BTreeSet<String>) {
        let old = self.active.replace(active.to_string());
        if let Some(old) = old {
            if wanted.contains(&old) {
                if let Some(link) = self.links.get_mut(&old) {
                    link.make_desk();
                    crate::blackbox::record(
                        "traffic",
                        serde_json::json!({ "event": "to_desk", "field": old,
                            "aircraft": link.fleet().aircraft().len() }),
                    );
                }
            } else if let Some(link) = self.links.remove(&old) {
                retire_link(&link, "station_moved");
            }
            crate::party_line::publish_traffic_picture(None);
            crate::party_line::publish_stands_taken(Vec::new());
        }
        if let Some(link) = self.links.get_mut(active) {
            let left = link.make_station();
            crate::blackbox::record(
                "traffic",
                serde_json::json!({ "event": "to_station", "field": active,
                    "left_frequency": left.len() }),
            );
            for id in left {
                crate::blackbox::record(
                    "traffic",
                    serde_json::json!({ "event": "despawn", "id": id, "reason": "reinstalled",
                        "field": active }),
                );
            }
        }
        let other = self.loading.as_ref().is_some_and(|(i, _)| i != active);
        if other && !self.links.contains_key(active) {
            self.loading = None;
        }
        self.board_last.clear();
        self.board_sent = None;
    }

    /// A link built off the tick, taken in: its ids from the world's
    /// counter, a desk of its own unless it is the station's field.
    fn take_built(&mut self, active: &str, wanted: &BTreeSet<String>) {
        let Some((icao, rx)) = &self.loading else {
            return;
        };
        let icao = icao.clone();
        let link = match rx.try_recv() {
            Ok(link) => link,
            Err(std::sync::mpsc::TryRecvError::Empty) => return,
            Err(std::sync::mpsc::TryRecvError::Disconnected) => None,
        };
        self.loading = None;
        match link {
            Some(mut link) if wanted.contains(&icao) => {
                link.share_ids(&self.ids);
                if icao != active {
                    link.make_desk();
                }
                self.links.insert(icao, link);
            }
            Some(_) => {}
            None => {
                self.failed.insert(icao);
            }
        }
    }

    /// The Board tab: on a change, at most every 2 s, and every 10 s
    /// regardless (a plugin reload zeroes its buffer). The field is the
    /// panel's pick among [`board_choices`] (read as the board is due),
    /// else the station's: a simulated field's board is live, any other's
    /// by the clock.
    async fn publish_board(
        &mut self,
        xp: &crate::xplane::XPlaneClient,
        s: &crate::plugin_status::StatusSink,
        own: (f64, f64),
        now: Instant,
    ) {
        let since = self.board_sent.map(|t| now.saturating_duration_since(t));
        if since.is_some_and(|d| d < BOARD_MIN_GAP) {
            return;
        }
        let Some(active) = self.active.clone().filter(|a| self.links.contains_key(a)) else {
            return;
        };
        let choices = board_choices(&active, own);
        let pick = if choices.len() > 1 {
            self.board_pick(xp, now).await
        } else {
            None
        };
        let sim_s = self.last_sim_s.unwrap_or(0.0);
        let shown = board_field(pick.as_deref(), &choices).map(str::to_string);
        if let Some((_, rx)) = &self.remote_loading {
            match rx.try_recv() {
                Ok(b) => {
                    self.remote = b;
                    self.remote_loading = None;
                }
                Err(std::sync::mpsc::TryRecvError::Empty) => {}
                Err(std::sync::mpsc::TryRecvError::Disconnected) => self.remote_loading = None,
            }
        }
        // A field nobody simulates (still loading, or no fleet there): its
        // board by the clock.
        if let Some(icao) = shown.as_ref().filter(|i| !self.links.contains_key(*i)) {
            let fresh = self
                .remote
                .as_ref()
                .is_some_and(|r| r.icao.eq_ignore_ascii_case(icao) && !r.stale(sim_s));
            let loading = self
                .remote_loading
                .as_ref()
                .is_some_and(|(i, _)| i.eq_ignore_ascii_case(icao));
            if !fresh && !loading {
                let zulu = sim_zulu_s(xp).await;
                let day = sim_date_days(xp).await;
                self.remote_loading =
                    Some((icao.clone(), self.build_remote(icao, zulu, day, sim_s)));
            }
        }
        let list: &[String] = if choices.len() > 1 { &choices } else { &[] };
        let live = shown
            .as_deref()
            .and_then(|i| self.links.get(i))
            .or_else(|| self.links.get(&active));
        let payload = match (&shown, &self.remote) {
            (Some(icao), Some(r))
                if !self.links.contains_key(icao) && r.icao.eq_ignore_ascii_case(icao) =>
            {
                r.payload(sim_s, list)
            }
            _ => match live {
                Some(link) => link.board_payload(own_board_flight(link.icao()), list),
                None => return,
            },
        };
        let due = since.is_none_or(|d| payload != self.board_last || d >= BOARD_RESEND);
        if due {
            s.board(&payload);
            self.board_last = payload;
            self.board_sent = Some(now);
        }
    }

    /// The panel's field pick ([`DR_BOARD_FIELD`]), upper case; `None` for
    /// none, or a plugin without it (not asked again for a minute).
    async fn board_pick(
        &mut self,
        xp: &crate::xplane::XPlaneClient,
        now: Instant,
    ) -> Option<String> {
        if self.board_pick_quiet_until.is_some_and(|t| now < t) {
            return None;
        }
        match xp.get_string(DR_BOARD_FIELD).await {
            Ok(v) => {
                self.board_pick_quiet_until = None;
                let v = v.trim().to_ascii_uppercase();
                (!v.is_empty()).then_some(v)
            }
            Err(_) => {
                self.board_pick_quiet_until = Some(now + std::time::Duration::from_secs(60));
                None
            }
        }
    }

    /// Build a remote field's board off the tick.
    fn build_remote(
        &self,
        icao: &str,
        zulu: Option<f64>,
        day: Option<u32>,
        sim0_s: f64,
    ) -> std::sync::mpsc::Receiver<Option<RemoteBoard>> {
        let cfg = crate::settings::live();
        let (tx, rx) = std::sync::mpsc::channel();
        let (icao, apt) = (icao.to_string(), cfg.apt_data_path.clone());
        let density =
            traffic::Density::parse(&cfg.traffic_density).unwrap_or(traffic::Density::Normal);
        let spawned = std::thread::Builder::new()
            .name("traffic-board".into())
            .spawn(move || {
                let b = RemoteBoard::build(&icao, &apt, density, zulu, day, sim0_s);
                tracing::info!(%icao, flights = b.as_ref().map(|b| b.flights.len()),
                    "traffic: a board for another field");
                let _ = tx.send(b);
            });
        if let Err(e) = spawned {
            tracing::warn!("traffic: the board loader did not start: {e}");
        }
        rx
    }

    /// Drop every fleet: every strip opened retired, the picture withdrawn.
    fn stand_down(&mut self) {
        for link in std::mem::take(&mut self.links).values() {
            retire_link(link, "stand_down");
        }
        if self.active.is_some() {
            crate::party_line::publish_traffic_picture(None);
            crate::party_line::publish_stands_taken(Vec::new());
        }
        if let Some(sink) = self.sink.as_mut() {
            sink.withdraw();
        }
        self.cull.clear();
        // The board goes with the field (sent empty on the next tick that
        // has a link, or never while none).
        self.board_last.clear();
        self.board_sent = None;
        self.remote = None;
        self.remote_loading = None;
        self.loading = None;
        self.failed.clear();
        self.active = None;
        self.last_sim_s = None;
    }

    /// Build the link for `icao` off the tick: the field (apt.dat, its
    /// taxi network and CIFP) and its schedule. The `station`'s field flies
    /// the pilot's file or a generated hour where it has no timetable, on
    /// the station's runway; any other field its timetable, on its own.
    fn build(
        &mut self,
        icao: &str,
        station: bool,
        zulu: Option<f64>,
        day: Option<u32>,
    ) -> std::sync::mpsc::Receiver<Option<Link>> {
        let cfg = crate::settings::live();
        let (tx, rx) = std::sync::mpsc::channel();
        let (icao, apt, cifp) = (
            icao.to_string(),
            cfg.apt_data_path.clone(),
            cfg.cifp_dir.clone(),
        );
        let file = if station {
            cfg.traffic_file.clone()
        } else {
            String::new()
        };
        let density = cfg.traffic_density.clone();
        let seed = traffic::field_seed(&icao, SESSION_SEED);
        let name = if station {
            crate::party_line::active_field()
                .map(|(n, ..)| crate::party_line::spoken_airport(&n).to_string())
                .unwrap_or_default()
        } else {
            String::new()
        };
        let runway = if station { station_runway() } else { None };
        let spawned = std::thread::Builder::new()
            .name("traffic-field".into())
            .spawn(move || {
                let ctx = BuildCtx {
                    seed,
                    name: &name,
                    zulu_s: zulu,
                    day,
                    runway,
                };
                let link = build_link(&icao, &apt, &cifp, &file, &density, &ctx);
                let _ = tx.send(link);
            });
        if let Err(e) = spawned {
            tracing::warn!("traffic: the field loader did not start: {e}");
        }
        rx
    }
}

/// A fleet dropped (`why`): every strip it opened at the station retired.
fn retire_link(link: &Link, why: &str) {
    let ids: Vec<u32> = link.fleet().aircraft().iter().map(|a| a.id).collect();
    if !link.is_desk() {
        for id in &ids {
            crate::world::retire_traffic(*id);
        }
    }
    crate::blackbox::record(
        "traffic",
        serde_json::json!({ "event": "stand_down", "field": link.icao(),
            "aircraft": ids.len(), "why": why }),
    );
}

/// Every field with a timetable within this of us is simulated, nm (the
/// pilot's ask, 2026-10-13; Malaga and Gibraltar are 54 nm apart).
pub(crate) const SIM_RANGE_NM: f64 = 100.0;

/// A simulated field is retired only past this, nm: one on the edge of
/// [`SIM_RANGE_NM`] does not churn (a build is two apt.dat scans).
pub(crate) const SIM_OUT_NM: f64 = 120.0;

/// The fields to simulate: the station's `active` field, and every field
/// with a timetable within [`SIM_RANGE_NM`] of `own` (lat, lon), or
/// within [`SIM_OUT_NM`] once `running`. Upper case. Pure.
pub(crate) fn wanted_fields(
    active: &str,
    own: (f64, f64),
    running: &BTreeSet<String>,
) -> BTreeSet<String> {
    let mut out = BTreeSet::from([active.trim().to_ascii_uppercase()]);
    for f in traffic::routes::fields() {
        let Some(at) = traffic::places::position(f.icao) else {
            continue;
        };
        let icao = f.icao.to_ascii_uppercase();
        let reach = if running.contains(&icao) {
            SIM_OUT_NM
        } else {
            SIM_RANGE_NM
        };
        if traffic::places::dist_nm(own, at) <= reach {
            out.insert(icao);
        }
    }
    out
}

/// The field to build next, one at a time: the station's first, then the
/// others in ICAO order; `None` once every wanted field is `built` (or
/// failed). Pure.
pub(crate) fn next_build(
    active: &str,
    wanted: &BTreeSet<String>,
    built: &BTreeSet<String>,
) -> Option<String> {
    if !built.contains(active) {
        return Some(active.to_string());
    }
    wanted.iter().find(|f| !built.contains(*f)).cloned()
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
/// the blackbox) when the field or the file cannot be read. `zulu_s` is
/// the sim's zulu time now, for the field's day (`traffic::rate`).
fn build_link(
    icao: &str,
    apt: &str,
    cifp: &str,
    file: &str,
    density: &str,
    ctx: &BuildCtx,
) -> Option<Link> {
    let BuildCtx {
        seed,
        name,
        zulu_s,
        day,
        ref runway,
    } = *ctx;
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
    let ctx = SpawnCtx {
        zulu_s,
        ..SpawnCtx::default()
    };
    let mut board = None;
    let table = traffic::routes::field_routes(icao);
    let (schedule, source) = if let (true, Some(table)) = (file.is_empty(), table) {
        let d = traffic::Density::parse(density).unwrap_or(traffic::Density::Normal);
        let (mut schedule, flights, local_s) =
            timetable_start(&field, table, d, seed, zulu_s, day, runway.clone());
        // A table that opts in is topped up by the generator from the
        // start (`traffic::midflow::top_up`), clear of its stands.
        let (date, zulu, _) = timetable_clock(table, zulu_s, day);
        let top_ctx = SpawnCtx {
            zulu_s: Some(zulu),
            occupied: planned_stands(&field, schedule.iter()),
            ..SpawnCtx::default()
        };
        let top = traffic::midflow::top_up(&field, table, d, seed, seed, date, local_s, &top_ctx);
        schedule.extend(top);
        board = Some((flights, local_s));
        (schedule, "timetable")
    } else if file.is_empty() {
        let d = traffic::Density::parse(density).unwrap_or(traffic::Density::Normal);
        let mut schedule = traffic::generate(&field, d, seed, &ctx);
        let parked = traffic::park_at_start(&field, d, seed, &ctx, &mut schedule);
        schedule.extend(parked);
        (schedule, "generated")
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
    let parked = schedule
        .iter()
        .filter(|m| m.kind == MovementKind::Parked || m.parked_from_start)
        .count();
    let movements = schedule
        .iter()
        .filter(|m| m.kind != MovementKind::Parked)
        .count();
    tracing::info!(%icao, movements, parked, source, "traffic: the fleet is up");
    crate::blackbox::record(
        "traffic",
        serde_json::json!({
            "event": "up", "field": icao, "movements": movements, "parked": parked,
            "source": source, "zulu_s": zulu_s,
        }),
    );
    // The calls address the station by the name it answers to.
    let name = match field.airport.radio_name() {
        n if n.is_empty() => name.to_string(),
        n => n,
    };
    let mut link = Link::new(field, schedule, &name);
    // A field with a route table flies its timetable's day whole; the rest
    // generate an hour at a time.
    // So does a timetable's top-up.
    let tops_up = source == "timetable" && table.is_some_and(|t| t.top_up);
    link.hours = (source == "generated" || tops_up).then_some(1);
    if let Some((flights, local_s)) = board {
        link.set_board(flights, local_s);
    }
    if let Some(table) = table {
        let date = timetable_clock(table, zulu_s, day).0;
        link.set_summer(date.summer());
        if source == "timetable" {
            link.timetable = Some(Timetable {
                density: traffic::Density::parse(density).unwrap_or(traffic::Density::Normal),
                seed,
                date,
            });
        }
    }
    Some(link)
}

/// The stands `movements` stand on or are planned on, where `field` has
/// them: the top-up never hands one out.
fn planned_stands<'a>(
    field: &FieldModel,
    movements: impl Iterator<Item = &'a Movement>,
) -> Vec<(f64, f64)> {
    movements
        .filter_map(|m| m.stand.as_deref())
        .filter_map(|n| field.stand(n))
        .map(|s| (s.lat, s.lon))
        .collect()
}

/// What a link is built with besides the files: the field's seed
/// (`traffic::field_seed`), the station's spoken name (empty: the field's
/// own), the sim's zulu time and date, the station's runway (`None` for a
/// field worked by its desk: its timetable flies its default runway).
struct BuildCtx<'a> {
    seed: u64,
    name: &'a str,
    zulu_s: Option<f64>,
    day: Option<u32>,
    runway: Option<String>,
}

/// The timetable's day at a field with a route table, already in motion
/// (`traffic::midflow::plan`, 2026-10-13): the movements (the placed ones
/// at 0 s), the board's flights, and the local time of day at the start.
/// The sim's date and zulu time when they read, else the system clock's;
/// the year is the system clock's (X-Plane's date carries none). The
/// stands the rotations leave are filled at light density
/// (`traffic::park_at_start`): the charters and the based aircraft no
/// table holds.
pub(crate) fn timetable_start(
    field: &FieldModel,
    table: &traffic::routes::FieldRoutes,
    density: traffic::Density,
    seed: u64,
    zulu_s: Option<f64>,
    day: Option<u32>,
    runway: Option<String>,
) -> (Vec<Movement>, Vec<traffic::board::BoardFlight>, f64) {
    let (date, zulu, local_s) = timetable_clock(table, zulu_s, day);
    let ctx = SpawnCtx {
        runway,
        zulu_s: Some(zulu),
        ..SpawnCtx::default()
    };
    let plan = traffic::midflow::plan(
        field,
        table,
        density,
        seed,
        date,
        local_s,
        TIMETABLE_HORIZON_S,
        &ctx,
    );
    let mut schedule = plan.movements;
    let fill = match density {
        traffic::Density::Off => traffic::Density::Off,
        _ => traffic::Density::Light,
    };
    let parked = traffic::park_at_start(field, fill, seed, &ctx, &mut schedule);
    schedule.extend(parked);
    (schedule, plan.board, local_s)
}

/// A timetable field flies this much of its day from the start.
pub(crate) const TIMETABLE_HORIZON_S: f64 = 24.0 * 3600.0;

/// A timetable's day and clock: the sim's date and zulu time when they
/// read, else the system clock's (the year is the system clock's, X-Plane's
/// date carries none): (date, zulu seconds, local seconds after the
/// date's midnight at the table's field).
fn timetable_clock(
    table: &traffic::routes::FieldRoutes,
    zulu_s: Option<f64>,
    day: Option<u32>,
) -> (traffic::rotation::Date, f64, f64) {
    use traffic::rotation::Date;
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0.0, |d| d.as_secs_f64());
    let today = Date::from_epoch_days((now / 86_400.0).floor() as i64);
    let date = day.map_or(today, |o| Date::of_year(today.year, o));
    let zulu = zulu_s.unwrap_or(now % 86_400.0);
    (
        date,
        zulu,
        traffic::rotation::local_s(zulu, table.utc_offset_h, date),
    )
}

/// The Board tab offers every field with a timetable this near us, nm
/// (2026-10-13; Malaga and Gibraltar are 54 nm apart).
pub(crate) const BOARD_RANGE_NM: f64 = 100.0;

/// The Board tab's field pick, plugin to app: an ICAO, empty for the field
/// the traffic runs at (2026-10-13). An older plugin has none: the board
/// stays the active field's.
pub(crate) const DR_BOARD_FIELD: &str = "xcopilot/opt/board_field";

/// The fields the Board tab can show: the one the traffic runs at first
/// (its board is the live one), then every other field with a route table
/// within [`BOARD_RANGE_NM`] of `own` (lat, lon), nearest first. Pure.
pub(crate) fn board_choices(active: &str, own: (f64, f64)) -> Vec<String> {
    let mut near: Vec<(f64, &str)> = traffic::routes::fields()
        .iter()
        .filter(|f| !f.icao.eq_ignore_ascii_case(active))
        .filter_map(|f| {
            let d = traffic::places::dist_nm(own, traffic::places::position(f.icao)?);
            (d <= BOARD_RANGE_NM).then_some((d, f.icao))
        })
        .collect();
    near.sort_by(|a, b| a.0.total_cmp(&b.0));
    std::iter::once(active.to_ascii_uppercase())
        .chain(near.into_iter().map(|(_, i)| i.to_string()))
        .collect()
}

/// The field whose board to show: the pick when it is one of `choices`
/// other than the active one (the first), else the active one. Pure.
pub(crate) fn board_field<'a>(pick: Option<&str>, choices: &'a [String]) -> Option<&'a str> {
    let p = pick?.trim();
    choices
        .iter()
        .skip(1)
        .find(|c| c.eq_ignore_ascii_case(p))
        .map(String::as_str)
}

/// A remote board is built again after this much sim time, seconds.
const REMOTE_BOARD_REBUILD_S: f64 = 6.0 * 3600.0;

/// The session's seed, hashed with each field's ICAO into that field's
/// day (`traffic::field_seed`, stage 2, 2026-10-13): a field's fleet and
/// its board by the clock draw the same day, whichever was built first.
/// A constant: the same field is the same day every session, as a sweep
/// is.
pub(crate) const SESSION_SEED: u64 = traffic::SESSION_SEED;

/// The board of a field with a timetable that the fleet does not fly
/// (2026-10-13, the Board tab's selector): its flights round the time it
/// was built (`traffic::midflow::timetable_board`, its planned stands),
/// the statuses by the clock (`Status::by_clock`), since its aircraft are
/// not simulated.
pub(crate) struct RemoteBoard {
    icao: String,
    flights: Vec<traffic::board::BoardFlight>,
    names: Vec<String>,
    /// Local seconds at the field when built, and the sim's running time
    /// then: the board's clock runs with the sim's.
    t0_local_s: f64,
    sim0_s: f64,
}

impl RemoteBoard {
    /// Built off the tick: the field from apt.dat and its timetable;
    /// `None` without either.
    pub(crate) fn build(
        icao: &str,
        apt: &str,
        density: traffic::Density,
        zulu_s: Option<f64>,
        day: Option<u32>,
        sim0_s: f64,
    ) -> Option<RemoteBoard> {
        let table = traffic::routes::field_routes(icao)?;
        let field = FieldModel::load(apt, icao).ok()??;
        let (date, _, local_s) = timetable_clock(table, zulu_s, day);
        Some(RemoteBoard::from_field(
            &field, table, density, date, local_s, sim0_s,
        ))
    }

    /// The board of `field` at `local_s` on `date`. Pure.
    pub(crate) fn from_field(
        field: &FieldModel,
        table: &traffic::routes::FieldRoutes,
        density: traffic::Density,
        date: traffic::rotation::Date,
        local_s: f64,
        sim0_s: f64,
    ) -> RemoteBoard {
        RemoteBoard {
            icao: field.icao.clone(),
            flights: traffic::midflow::timetable_board(
                field,
                table,
                density,
                traffic::field_seed(&field.icao, SESSION_SEED),
                date,
                local_s,
                TIMETABLE_HORIZON_S,
            ),
            names: field.net.stands.iter().map(|s| s.name.clone()).collect(),
            t0_local_s: local_s,
            sim0_s,
        }
    }

    fn stale(&self, sim_s: f64) -> bool {
        sim_s - self.sim0_s > REMOTE_BOARD_REBUILD_S || sim_s < self.sim0_s
    }

    /// The payload at sim running time `sim_s`: each flight's status by
    /// the clock, a departure's planned stand, an arrival's once landed.
    pub(crate) fn payload(&self, sim_s: f64, choices: &[String]) -> String {
        use traffic::board::{Dir, Row, Status};
        let now = self.t0_local_s + (sim_s - self.sim0_s).max(0.0);
        let names: Vec<&str> = self.names.iter().map(String::as_str).collect();
        let rows: Vec<Row> = self
            .flights
            .iter()
            .map(|f| {
                let status = Status::by_clock(f, now);
                let shown = f.dir == Dir::Departure || status == Status::Landed;
                Row {
                    flight: f.clone(),
                    status,
                    time_s: f.info.est_s,
                    stand: f
                        .info
                        .stand
                        .as_deref()
                        .filter(|_| shown)
                        .map(|n| traffic::stands::label(n, &names)),
                }
            })
            .collect();
        traffic::board::payload(&self.icao, now, &rows, choices)
    }
}

/// The sim's date, days since the first of January; `None` on a failed
/// read.
async fn sim_date_days(xp: &crate::xplane::XPlaneClient) -> Option<u32> {
    xp.get_float(crate::datarefs::LOCAL_DATE_DAYS)
        .await
        .ok()
        .filter(|d| d.is_finite() && *d >= 0.0 && *d < 367.0)
        .map(|d| d as u32)
}

/// The next generated hour at the fleet's field, round us.
fn generate_for(fleet: &Fleet, hour: u64, own: &OwnState, zulu_s: Option<f64>) -> Vec<Movement> {
    let density = traffic::Density::parse(&crate::settings::live().traffic_density)
        .unwrap_or(traffic::Density::Normal);
    next_hour(fleet, density, hour, Some((own.lat, own.lon)), zulu_s)
}

/// The next generated hour at the fleet's field: every stand anybody holds
/// taken, and as many departures scheduled without a stand of their own
/// as there are airliners parked on theirs and ready within half the hour
/// (`SpawnCtx::parked_airframes`): each takes one of those airframes at its
/// time (`Link::take_parked`) rather than appear out of nothing.
pub(super) fn next_hour(
    fleet: &Fleet,
    density: traffic::Density,
    hour: u64,
    own: Option<(f64, f64)>,
    zulu_s: Option<f64>,
) -> Vec<Movement> {
    let by = fleet.clock.now_s() + traffic::schedule::HORIZON_S / 2.0;
    let parked_airframes = fleet.airframes_ready_by(by, &[], &|a| {
        traffic::class_for_type(&a.icao_type).0.name != traffic::PISTON_SINGLE.name
    });
    let ctx = SpawnCtx {
        runway: None,
        own,
        occupied: fleet.stands_held(),
        zulu_s,
        parked_airframes,
    };
    traffic::generate(&fleet.field, density, hour, &ctx)
}

/// A dormant airframe `a` may fly a circuit (`light`: a light aircraft) or
/// a departure (an airliner whose type the `runway_m` in use holds,
/// `schedule::runway_needed_m`).
fn airframe_fits(a: &TrafficAircraft, light: bool, runway_m: f64) -> bool {
    let is_light = traffic::class_for_type(&a.icao_type).0.name == traffic::PISTON_SINGLE.name;
    is_light == light && (light || runway_m >= traffic::schedule::runway_needed_m(&a.icao_type))
}

/// The sim's zulu time, seconds after midnight; `None` on a failed read
/// (the schedule then flies the field's peak, never a guessed hour).
async fn sim_zulu_s(xp: &crate::xplane::XPlaneClient) -> Option<f64> {
    xp.get_float(crate::datarefs::ZULU_TIME_SEC)
        .await
        .ok()
        .filter(|z| z.is_finite())
}

/// The blackbox's `traffic` records for one tick: one record per event, each
/// one write.
fn record(out: &TickOut, link: &Link) {
    let field = link.icao();
    for (id, callsign) in &out.spawned {
        let ac = link.fleet().get(*id);
        let from = out
            .from_stand
            .iter()
            .find(|(i, _)| i == id)
            .map_or("new", |(_, f)| *f);
        rec(
            field,
            serde_json::json!({
                "event": "spawn", "id": id, "callsign": callsign, "from": from,
                "type": ac.map(|a| a.icao_type.clone()),
                "plan": ac.map(|a| plan_word(&a.plan)),
                "runway": ac.and_then(|a| a.plan.runway().map(str::to_string)),
            }),
        );
    }
    for (id, callsign) in &out.parked {
        let ac = link.fleet().get(*id);
        rec(
            field,
            serde_json::json!({
                "event": "parked", "id": id, "callsign": callsign,
                "type": ac.map(|a| a.icao_type.clone()),
                "stand": ac.and_then(|a| a.stand.clone()),
            }),
        );
    }
    for id in &out.left_frequency {
        rec(
            field,
            serde_json::json!({ "event": "despawn", "id": id, "reason": "reinstalled" }),
        );
    }
    for ev in &out.events {
        match ev {
            Event::Despawn { id, reason } => rec(
                field,
                serde_json::json!({ "event": "despawn", "id": id, "reason": despawn_word(*reason) }),
            ),
            Event::Phase { id, from, to } => rec(
                field,
                serde_json::json!({
                    "event": "phase", "id": id,
                    "from": format!("{from:?}"), "to": format!("{to:?}"),
                }),
            ),
            Event::TurnedRound { id } => rec(
                field,
                serde_json::json!({ "event": "turned_round", "id": id }),
            ),
            Event::Touchdown { id, along_m, .. } => rec(
                field,
                serde_json::json!({ "event": "touchdown", "id": id, "along_m": along_m.round() }),
            ),
            // The gridlock guard on the ground, behind one that would not
            // move (null: our own aircraft): planned another way round it,
            // the blocker towed clear, or waiting with neither.
            Event::SpacingRerouted { id, around } => rec(
                field,
                serde_json::json!({ "event": "spacing_rerouted", "id": id, "around": around }),
            ),
            Event::SpacingCleared {
                id,
                removed,
                why,
                at,
                phase,
                still_s,
            } => {
                let place = |(p, hdg): ((f64, f64), f64)| {
                    serde_json::json!({ "lat": p.0, "lon": p.1, "hdg": hdg.round() })
                };
                rec(
                    field,
                    serde_json::json!({
                        "event": "spacing_cleared",
                        "id": id,
                        "removed": removed,
                        "why": why,
                        "mover": place(at[0]),
                        "at": place(at[1]),
                        "phase": format!("{phase:?}"),
                        "still_s": still_s.round(),
                    }),
                )
            }
            Event::SpacingHeld { id, behind } => rec(
                field,
                serde_json::json!({ "event": "spacing_held", "id": id, "behind": behind }),
            ),
            Event::Call { .. } | Event::Spawn { .. } => {}
        }
    }
    for t in &out.transmissions {
        rec(
            field,
            serde_json::json!({ "event": "call", "id": t.id, "callsign": t.callsign, "text": t.text,
                "on": t.on.tag(), "waited_s": t.waited_s, "clear_s": t.clear_s }),
        );
    }
    for (id, state, runway) in &out.orders {
        rec(
            field,
            serde_json::json!({
                "event": "order", "id": id, "state": format!("{state:?}"), "runway": runway,
            }),
        );
    }
    for (id, line) in &out.tower {
        rec(
            field,
            serde_json::json!({ "event": "tower", "id": id, "line": line }),
        );
    }
    for (id, line, who) in &out.positions {
        rec(
            field,
            serde_json::json!({ "event": "handoff", "id": id, "line": line, "by": who.tag() }),
        );
    }
}

/// One `traffic` record of `field`'s fleet: the field named in it, since
/// several fleets fly at once (multi-field traffic stage 3, 2026-10-13).
fn rec(field: &str, mut v: serde_json::Value) {
    if let Some(o) = v.as_object_mut() {
        o.insert("field".into(), field.into());
    }
    crate::blackbox::record("traffic", v);
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
        Despawn::ReadbackFailed => "readback_failed",
        Despawn::Towed => "towed",
    }
}

#[path = "traffic_whole.rs"]
mod whole;

#[cfg(test)]
#[path = "traffic_link_tests.rs"]
mod tests;
