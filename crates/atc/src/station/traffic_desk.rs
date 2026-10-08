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
    /// included), and the same pick, traffic and taxi route planners ours
    /// uses ([`Station::ctx_for_call`]), ending in [`Station::make_ctx`].
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
        self.ctx_for_call(
            state,
            wx,
            qnh,
            spoken.to_string(),
            intent,
            &targets,
            Instant::now(),
        )
    }

    /// Answer a traffic flight's call on its own desk: its strip opened if
    /// none is, the call's runway noted and its context built with that
    /// desk on frequency, then [`Station::respond_for`] (the keyed path,
    /// readback gate included, as ours). Synchronous: never across an await.
    /// Its answer writes its own strip; a challenged readback leaves the
    /// readback owed on it ([`crate::world`]'s order view reads it).
    pub fn answer_traffic(&mut self, call: &TrafficCall) -> TrafficReply {
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
}
