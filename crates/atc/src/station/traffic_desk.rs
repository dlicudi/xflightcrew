//! The station's side of our own AI traffic (stage D): the producer (the
//! bin's fleet) reports where each traffic flight is, posts what it says,
//! and reads back what it was told. The traffic is plain data here; this
//! crate never names the fleet, and the fleet never names a strip.
//!
//! A traffic call goes in as TEXT, through `tower::parse_intent` and
//! `route_keyed` exactly as ours does, readback gate included, on the
//! traffic flight's own desk ([`Station::answer_traffic`]). Its context is
//! built without the sim ([`Station::traffic_ctx`]), and it writes none of
//! the FIELD's facts: the runway in use (but to initialise one, logged),
//! the tower's wind, the ghost memory and the ATIS letter are ours.

use super::*;

/// Where traffic flight `id` is now, from its producer: written to its
/// strip's [`FlightStrip::seen`] cell, opening the strip if none is open
/// (the tower sees every aircraft on its field, called or not). Lock order:
/// the map, clone the `Arc`, drop the map, then the cell.
pub fn note_traffic_state(id: u32, state: AircraftState) {
    let strip = open_traffic_strip(id);
    *strip.seen.lock().unwrap_or_else(|e| e.into_inner()) = Some(state);
}

/// One aircraft in a traffic flight's picture: who it is and where, as the
/// tower's radar shows it. Ours is `FlightId::Own`.
#[derive(Clone)]
pub struct TrafficTarget {
    pub who: FlightId,
    pub target: TcasTarget,
}

/// A traffic flight's transmission, as its producer keyed it.
#[derive(Clone)]
pub struct TrafficCall {
    /// The traffic flight's stable number ([`FlightId::Traffic`]).
    pub id: u32,
    /// Its callsign as spoken ("Cessna three four five"): the tower
    /// answers it by this.
    pub spoken: String,
    /// The words, in the field's dialect ("Gatwick Tower, Cessna three four
    /// five, ready for departure runway two six left").
    pub text: String,
    /// Where it is as it calls.
    pub state: AircraftState,
    /// What the tower sees round it as it answers: every OTHER aircraft,
    /// ours included (the producer's picture minus this flight, plus us).
    pub picture: Vec<TrafficTarget>,
}

/// The tower's answer to a traffic call.
#[derive(Debug, Clone, PartialEq)]
pub struct TrafficReply {
    pub id: u32,
    /// What the tower says; empty = an accepted readback (the frequency
    /// stays quiet).
    pub line: String,
    pub speaker: StationSpeaker,
}

/// The traffic calls posted from the monitor's tick, for the command loop
/// (which owns the `Station`) to answer. The field's: reset by `install`
/// and `uninstall` with the strips ([`TrafficStrips`]).
pub(crate) static TRAFFIC_CALLS: Mutex<Vec<TrafficCall>> = Mutex::new(Vec::new());

/// Post a traffic flight's call for the station to answer.
pub fn post_traffic_call(call: TrafficCall) {
    TRAFFIC_CALLS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .push(call);
}

/// Every traffic call posted since the last take, in order. Command loop
/// only: it answers each through [`Station::answer_traffic`].
pub fn take_traffic_calls() -> Vec<TrafficCall> {
    std::mem::take(&mut *TRAFFIC_CALLS.lock().unwrap_or_else(|e| e.into_inner()))
}

impl Station {
    /// The context a traffic flight's call is answered with, built with ITS
    /// desk on frequency and no sim read: the wind the tower last worked
    /// with ([`TOWER_WIND`], else the field's surface wind, else calm), the
    /// pressure our last context was given, the picture handed in (ours
    /// included), the moment the call was keyed (`state.at`) as its clock,
    /// and the same pick, traffic and taxi route planners ours uses
    /// ([`Station::ctx_for_call`]), ending in [`Station::make_ctx`].
    /// Writes no field fact: no ghost memory, no tower wind, no pressure.
    pub(super) fn traffic_ctx(
        &mut self,
        state: &AircraftState,
        spoken: &str,
        intent: tower::Intent,
        picture: &[TrafficTarget],
    ) -> (tower::Ctx, Wx) {
        let (wind_from_deg, wind_kts) = TOWER_WIND
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .or_else(|| surface_wind_at(self.apt.position()))
            .unwrap_or((0.0, 0.0));
        let wx = Wx {
            wind_from_deg,
            wind_kts,
        };
        let qnh = self.last_qnh.clone().unwrap_or_default();
        let targets: Vec<TcasTarget> = picture.iter().map(|t| t.target.clone()).collect();
        let ids: Vec<(&str, FlightId)> = picture
            .iter()
            .map(|t| (t.target.callsign.as_str(), t.who))
            .collect();
        self.ctx_for_call(
            state,
            wx,
            qnh,
            spoken.to_string(),
            intent,
            &targets,
            &ids,
            state.at,
        )
    }

    /// The reply path's clock: the moment the traffic call being answered
    /// was keyed ([`Station::reply_at`]), else the wall clock (ours).
    pub(super) fn reply_now(&self) -> Instant {
        self.reply_at.unwrap_or_else(Instant::now)
    }

    /// Answer a traffic flight's call on its own desk: its strip opened if
    /// none is, the call's runway noted and its context built with that
    /// desk on frequency, then [`Station::respond_for`] (the keyed path,
    /// readback gate included, as ours). Synchronous: never across an await.
    /// Its answer writes its own strip; a challenged readback leaves the
    /// readback owed on it ([`crate::world`]'s order view reads it).
    pub fn answer_traffic(&mut self, call: &TrafficCall) -> TrafficReply {
        self.reply_at = Some(call.state.at);
        let reply = self.answer_traffic_at(call);
        self.reply_at = None;
        reply
    }

    /// [`Station::answer_traffic`] on the call's clock.
    fn answer_traffic_at(&mut self, call: &TrafficCall) -> TrafficReply {
        open_traffic_strip(call.id);
        let id = FlightId::Traffic(call.id);
        let intent = tower::parse_intent(&call.text);
        let built = self.with_desk(id, |st| {
            st.note_stated_runway(&call.text, intent.clone());
            let before = st.desk.twr.state;
            let (ctx, wx) =
                st.traffic_ctx(&call.state, &call.spoken, intent.clone(), &call.picture);
            (ctx, wx, before)
        });
        let line = built.and_then(|(ctx, wx, before)| {
            let line = self.respond_for(id, &call.text, Service::Tower, &ctx, wx)?;
            let after = self
                .desks
                .get(&id)
                .map(|d| format!("{:?}", d.twr.state))
                .unwrap_or_default();
            sim::blackbox::record(
                "atc",
                serde_json::json!({
                    "route": "traffic", "id": call.id,
                    "outcome": if line.is_empty() { "silence" } else { "reply" },
                    "transcript": call.text, "intent": format!("{intent:?}"),
                    "state": format!("{before:?}>{after}"), "reply": line,
                }),
            );
            Some(line)
        });
        TrafficReply {
            id: call.id,
            line: line.unwrap_or_default(),
            speaker: StationSpeaker::Tower,
        }
    }

    /// Keep every traffic desk's tower FSM in step with what the ticks said
    /// on frequency, as our `sync_tower` does for ours: a released takeoff,
    /// a sequenced line-up, a go-around order, a released landing. Each desk
    /// is swapped in for its sync (`with_desk`, never across an await); a
    /// desk whose strip closed (or was reopened) is dropped. Run before the
    /// traffic calls are answered.
    pub fn sync_traffic_desks(&mut self) {
        let ids: Vec<FlightId> = self.desks.keys().copied().collect();
        for id in ids {
            let FlightId::Traffic(n) = id else {
                continue;
            };
            let alive = match (self.desks.get(&id), traffic_strip(n)) {
                (Some(d), Some(open)) => {
                    matches!(&d.strip, StripRef::Traffic(s) if std::sync::Arc::ptr_eq(s, &open))
                }
                _ => false,
            };
            if !alive {
                self.desks.remove(&id);
                continue;
            }
            self.with_desk(id, |st| st.sync_traffic_desk());
        }
    }

    /// One traffic desk's sync, on frequency.
    fn sync_traffic_desk(&mut self) {
        let take =
            |m: &Mutex<bool>| std::mem::take(&mut *m.lock().unwrap_or_else(|e| e.into_inner()));
        if take(&self.desk.strip.pending_takeoff_clearance) {
            let (runway, callsign) = self.cleared_runway_spoken();
            self.sync_takeoff_release(&runway, &callsign);
        }
        if take(&self.desk.strip.pending_line_up) {
            let (runway, callsign) = self.cleared_runway_spoken();
            self.sync_line_up(&runway, &callsign);
        }
        if take(&self.desk.strip.pending_go_around) {
            self.apply_monitor_go_around();
        }
        self.sync_landing_release();
    }

    /// The runway the desk's standing clearance names (else the runway in
    /// use), spoken, and the callsign it was issued to: what a released
    /// clearance is synced with when no context is built for it.
    fn cleared_runway_spoken(&self) -> (String, String) {
        let (ident, callsign) = self
            .desk
            .strip
            .clearance
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .map(|c| (c.runway_ident.clone(), c.callsign.clone()))
            .unwrap_or_default();
        let ident = Some(ident)
            .filter(|r| !r.is_empty())
            .or_else(|| self.runway_in_use.clone())
            .unwrap_or_default();
        (atis::spoken_runway(&ident), callsign)
    }
}

/// What the tower has told a traffic flight, as it stands now: a
/// LEVEL-TRIGGERED view over its strip's cells (nothing here is consumed,
/// so an order a tick did not act on is still there at the next).
#[derive(Debug, Clone, PartialEq)]
pub struct TrafficOrder {
    /// The tower FSM state of its standing clearance (`Parked` with none).
    pub state: tower::State,
    /// The runway END the clearance names ("26L"); empty when none.
    pub runway: String,
    /// The taxi route it is cleared on, (lat, lon) in order; empty when none.
    pub taxi_path: Vec<(f64, f64)>,
    /// The route's end: a holding point, or a stand for an arrival.
    pub holding_point: String,
    /// The route ends at a stand (an arrival's taxi-in).
    pub to_stand: bool,
    /// Its number in the landing sequence, when the tower gave one.
    pub sequence: Option<u8>,
    /// The flight it follows ("number two, follow the traffic"), when the
    /// tower's picture named it.
    pub follow: Option<FlightId>,
    /// A takeoff or landing clearance is being WITHHELD: hold short, or
    /// continue the approach without one.
    pub hold: bool,
    /// A conditional line-up's condition: `Some(false)` while the named
    /// arrival is still on final.
    pub line_up_condition: Option<bool>,
    /// The tower ordered a go-around and it has not been acknowledged.
    pub go_around: bool,
    /// The readback the tower is waiting for, phrased; `None` once heard.
    pub readback_owed: Option<String>,
    /// The runway END the flight is cleared to cross ("13"), from the
    /// clearance until it is seen off that strip again; `None` otherwise.
    pub crossing: Option<String>,
}

/// How the acknowledgement a go-around order owes begins
/// (`Station::apply_monitor_go_around`).
pub(super) const GO_AROUND_READBACK: &str = "Going around, ";

/// The order view for traffic flight `id`; `None` with no strip open. Each
/// cell is locked alone and dropped before the next.
pub fn traffic_order(id: u32) -> Option<TrafficOrder> {
    let strip = traffic_strip(id)?;
    let clr = strip
        .clearance
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|c| (c.state, c.runway_ident.clone(), c.pending_readback.clone()));
    let route = strip
        .taxi_route
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|r| (r.path.clone(), r.holding_point.clone()));
    let to_stand = *strip
        .taxi_to_stand
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    let sequence = *strip.sequence.lock().unwrap_or_else(|e| e.into_inner());
    let crossing = strip
        .crossing
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|c| c.0.clone());
    let takeoff_held = strip
        .takeoff_hold
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .is_some();
    let landing_held = strip
        .landing_hold
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .is_some();
    let line_up_condition = *strip
        .line_up_condition
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    let ordered = *strip
        .pending_go_around
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    let readback_owed = clr
        .as_ref()
        .and_then(|c| c.2.clone())
        .filter(|r| !r.is_empty());
    // The order stands from the tick that gave it (`pending_go_around`)
    // through the desk's sync (which owes "Going around, ...") until the
    // flight has said it.
    let go_around = ordered
        || readback_owed
            .as_deref()
            .is_some_and(|r| r.starts_with(GO_AROUND_READBACK));
    let (taxi_path, holding_point) = route.unwrap_or_default();
    Some(TrafficOrder {
        state: clr.as_ref().map_or(tower::State::Parked, |c| c.0),
        runway: clr.map(|c| c.1).unwrap_or_default(),
        taxi_path,
        holding_point,
        to_stand,
        sequence: sequence.map(|s| s.0),
        follow: sequence.and_then(|s| s.1),
        hold: takeoff_held || landing_held,
        line_up_condition,
        go_around,
        readback_owed,
        crossing,
    })
}

/// The producer despawned traffic flight `id`: its strip closes (its claims
/// leave the book) and its desk is dropped at the next desk sync.
pub fn retire_traffic(id: u32) {
    STRIPS.lock().unwrap_or_else(|e| e.into_inner()).remove(&id);
}

/// One pass of the runway watch for traffic flight `id`, with the picture
/// round it (every OTHER aircraft, ours included): its position noted on
/// its strip, then the go-around judgement, the withheld landing's release
/// and the held takeoff's release, the same `_for` cores ours runs, in our
/// order. Returns what the tower says, in order (the tower's voice, on its
/// frequency). Nothing without an open strip or a reported position. Reads
/// the field's ghost memory, writes none of it.
pub fn traffic_tick(id: u32, picture: &[TrafficTarget], now: Instant) -> Vec<String> {
    let Some(strip) = traffic_strip(id) else {
        return Vec::new();
    };
    let Some(state) = seen(&strip) else {
        return Vec::new();
    };
    let targets: Vec<TcasTarget> = picture.iter().map(|t| t.target.clone()).collect();
    note_aircraft_state_for(&strip, &state);
    let elev = active_field().map_or(0.0, |f| f.3);
    // No height reported is no go-around judgement (never a made-up one).
    let agl = if state.alt_ft.is_finite() {
        state.alt_ft - elev
    } else {
        f64::INFINITY
    };
    let mut lines = Vec::new();
    // Always on frequency: a traffic flight is called by name.
    lines.extend(go_around_tick_for(&strip, &state, &targets, agl, true, now).map(|c| c.line));
    lines.extend(landing_hold_tick_for(&strip, &state, &targets, now));
    lines.extend(takeoff_hold_tick_for(&strip, &state, &targets, elev, now));
    lines
}

/// The compliance watch over traffic flight `id`, as an ORACLE: the same
/// core ours ticks (`compliance_tick_for`), on its strip and its last seen
/// state, tuned. A traffic flight that obeys its orders is never berated,
/// so tests and the traffic harness assert this stays `None`. Never ticked
/// in flight: it writes the strip's compliance memory, which the book
/// reads, and the field's radio cooldowns, which are ours.
pub fn traffic_compliance_oracle(id: u32, now: Instant) -> Option<String> {
    let strip = traffic_strip(id)?;
    let state = seen(&strip)?;
    compliance_tick_for(&strip, &state, true, now).map(|c| c.line)
}
