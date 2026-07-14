//! The ATC station living inside the copilot process — the merge of the
//! standalone `atc` binary's tower service into the one binary the pilot
//! actually flies with. One mic, one PTT, one speech arbiter; the tuned COM1
//! frequency plus the shape of the words decide who answers.
//!
//! Trust boundary: this module only ever *reads* the simulator (frequency,
//! weather, tail number) and returns text. It has no access to the tool
//! registry and can never actuate anything — ATC is the world, not the crew.
//! Compiler-enforced: the sim handle here is `SimView`, a read-only view
//! with no `set_*`/`run_command` on it at all.

use std::sync::Mutex;
use std::time::Instant;

use crate::speaker::StationSpeaker;
use sim::xplane::SimView;
use crate::{atis, nav, tower};
use sim::datarefs;

// The state this module publishes for the crew lives on the party line —
// one audited page. `pub(crate)` re-export so the world side (this module
// and its tests) writes them by their bare names; the crew side reads via
// `party_line`'s accessors, never these statics.
pub(crate) use crate::party_line::{
    CLEARANCE, LANDING_HANDOFF, PENDING_GO_AROUND, PENDING_STATION, PENDING_TAKEOFF_CLEARANCE,
    TAKEOFF_HOLD, TAXI_ROUTE, VECTORS,
};

/// Which of the station's frequencies COM1 is tuned to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Service {
    Tower,
    Approach,
    Departure,
}

impl Service {
    /// The voice this position speaks with — a different controller each.
    fn speaker(self) -> StationSpeaker {
        match self {
            Service::Tower => StationSpeaker::Tower,
            Service::Approach => StationSpeaker::Approach,
            Service::Departure => StationSpeaker::Departure,
        }
    }

    /// The position's name for the blackbox trace.
    fn name(self) -> &'static str {
        match self {
            Service::Tower => "tower",
            Service::Approach => "approach",
            Service::Departure => "departure",
        }
    }
}

/// Live wind for one exchange, read once by the async wrappers so the routing
/// layer below them is pure (transcript in, reply out — it battle-tests dry,
/// with no simulator on the other end).
#[derive(Debug, Clone, Copy)]
pub struct Wx {
    pub wind_from_deg: f64,
    pub wind_kts: f64,
}

pub struct Station {
    apt: nav::AirportInfo,
    tower_khz: i64,
    /// The approach frequency (apt.dat row 1055) — the vectoring service.
    approach_khz: Option<i64>,
    /// The departure frequency (row 1056) — where the tower sends you after
    /// takeoff.
    departure_khz: Option<i64>,
    twr: tower::Tower,
    /// The runway in use, held across calls: with the wind below 5 knots the
    /// into-wind pick is a coin toss, and the 2026-07-08 flight was cleared
    /// for takeoff on two seven, then re-cleared on zero niner two minutes
    /// later in the same calm. A runway change needs actual wind.
    runway_in_use: Option<String>,
    /// The departure controller has said "radar contact" to this flight.
    dep_checked_in: bool,
    /// The ATIS information letter the pilot last reported having ("with
    /// information Alpha"). When it matches the current broadcast, clearances
    /// skip the QNH — the whole point of an ATIS.
    pilot_info: Option<char>,
    /// This flight's assigned discrete squawk, once a radar service (vectors,
    /// flight following) has issued one. Repeated by every squawk check;
    /// `None` means the VFR conspicuity 7000 is the answer.
    assigned_squawk: Option<i64>,
    /// The taxi route planned when the ctx was last built on the ground —
    /// published to [`TAXI_ROUTE`] iff the taxi clearance is actually issued.
    planned_route: Option<nav::TaxiRoutePlan>,
    /// Challenge missing/wrong readbacks of runway-critical clearances
    /// (`XCOPILOT_ATC_READBACK`). The pending state still arms when this is
    /// off — the coach teaches the readback either way; only the challenges
    /// are gated.
    enforce_readback: bool,
    /// Phraseology dialect for this field's controllers (set by the owner
    /// from the ICAO prefix, like the voice casting). Defaults to ICAO.
    dialect: Dialect,
}

/// Radio phraseology dialect. ICAO/CAP 413 style ("descend to altitude two
/// thousand five hundred feet, QNH one zero two zero") serves most of the
/// world; FAA style ("descend and maintain two thousand five hundred") for
/// North America. Same lever as geography-accurate voice casting: a Heathrow
/// controller must not sound like Chicago Center, in accent OR in grammar.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dialect {
    Icao,
    Faa,
}

/// The dialect for a field, by ICAO prefix: K (CONUS), P (US Pacific/Alaska)
/// and C (Canada) speak FAA-style; everywhere else speaks ICAO. Coarse on
/// purpose — refine per-region as fields get flown.
pub fn dialect_for(icao: &str) -> Dialect {
    match icao.chars().next() {
        Some('K') | Some('P') | Some('C') => Dialect::Faa,
        _ => Dialect::Icao,
    }
}

/// The clearance's pressure phrase in the field's own money: "QNH one zero
/// two zero" (hectopascals) everywhere ICAO, "altimeter two niner niner two"
/// (inches, hundredths) at FAA fields.
pub fn spoken_pressure(pascals: f64, dialect: Dialect) -> String {
    match dialect {
        Dialect::Icao => {
            format!("QNH {}", atis::spoken_digits(&format!("{:.0}", pascals / 100.0)))
        }
        Dialect::Faa => {
            let inhg_hundredths = (pascals / 3386.389 * 100.0).round();
            format!("altimeter {}", atis::spoken_digits(&format!("{inhg_hundredths:.0}")))
        }
    }
}

impl Station {
    /// Load the station for `icao` from apt.dat. `None` (with a log line) when
    /// the field is missing or has no tower frequency — the copilot then runs
    /// exactly as before the merge.
    pub fn load(apt_path: &str, icao: &str) -> Option<Station> {
        let apt = match nav::find_airport(apt_path, icao) {
            Ok(Some(a)) => a,
            Ok(None) => {
                tracing::warn!(%icao, "ATC station: airport not in apt.dat; ATC off");
                return None;
            }
            Err(e) => {
                tracing::warn!(%icao, "ATC station: apt.dat scan failed ({e:#}); ATC off");
                return None;
            }
        };
        if apt.freq(nav::FREQ_TOWER).is_none() {
            tracing::warn!(%icao, "ATC station: no tower frequency in apt.dat; ATC off");
            return None;
        }
        Self::from_airport(apt)
    }

    /// Build the station from an already-parsed airport. `None` without a
    /// tower frequency. Split from [`Station::load`] so offline sweeps (the
    /// cross-airport battle test) can drive one apt.dat pass through every
    /// field instead of rescanning the 400 MB file per airport.
    pub fn from_airport(apt: nav::AirportInfo) -> Option<Station> {
        let twr_row = apt.freq(nav::FREQ_TOWER)?;
        tracing::info!(
            airport = %apt.name,
            mhz = twr_row.khz as f64 / 1000.0,
            "ATC station up: tower answers radio calls on this frequency"
        );
        let approach_khz = apt.freq(nav::FREQ_APPROACH).map(|f| f.khz);
        if let Some(k) = approach_khz {
            tracing::info!(mhz = k as f64 / 1000.0, "approach on frequency (vectors)");
        }
        let departure_khz = apt.freq(nav::FREQ_DEPARTURE).map(|f| f.khz);
        if let Some(k) = departure_khz {
            tracing::info!(mhz = k as f64 / 1000.0, "departure on frequency (handoff after takeoff)");
        }
        Some(Station {
            tower_khz: twr_row.khz,
            approach_khz,
            departure_khz,
            apt,
            twr: tower::Tower::new(),
            runway_in_use: None,
            dep_checked_in: false,
            pilot_info: None,
            assigned_squawk: None,
            planned_route: None,
            enforce_readback: true,
            dialect: Dialect::Icao,
        })
    }

    /// Gate the readback challenges on config (`XCOPILOT_ATC_READBACK`).
    pub fn set_enforce_readback(&mut self, on: bool) {
        self.enforce_readback = on;
    }

    /// Set the phraseology dialect (the owner resolves it from the ICAO
    /// prefix via [`dialect_for`] — the station never sees its own ICAO).
    pub fn set_dialect(&mut self, d: Dialect) {
        self.dialect = d;
    }

    /// The active field's spoken name ("Gibraltar") — the per-airport voice
    /// seed and the home-field check key off it.
    pub fn airport_name(&self) -> &str {
        &self.apt.name
    }

    /// Publish this station's process-wide side state — the ATIS broadcast
    /// and the enroute switch identity the monitor's ticks read. Called by
    /// the OWNER once a station goes live (startup and each region handoff),
    /// never by `from_airport` itself, so offline sweeps and tests stay
    /// side-effect free.
    pub fn install(&self) {
        *ACTIVE_FIELD.lock().unwrap_or_else(|e| e.into_inner()) = self
            .apt
            .position()
            .map(|(lat, lon)| (self.apt.name.clone(), lat, lon));
        // Controller frequencies, for the monitor's traffic advisories: a
        // controller narrates traffic whenever the pilot is on frequency,
        // not only while being vectored.
        let mut freqs = vec![(self.tower_khz, StationSpeaker::Tower)];
        if let Some(k) = self.approach_khz {
            freqs.push((k, StationSpeaker::Approach));
        }
        if let Some(k) = self.departure_khz {
            freqs.push((k, StationSpeaker::Departure));
        }
        *CONTROLLER_FREQS.lock().unwrap_or_else(|e| e.into_inner()) = freqs;
        // A new station means the old radar service (and its code) is void,
        // and the new tower has never talked to you — or to its ghosts.
        *SQUAWK_WATCH.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()) = None;
        // Zone-watch grace on a fresh station: a region handoff says
        // "contact <name> Tower" and the pilot deserves the cooldown window
        // to actually call — being challenged as an intruder two seconds
        // after the handoff (2026-07-10 tape) reads as a tower at war with
        // its own radar room.
        *COMPLIANCE.lock().unwrap_or_else(|e| e.into_inner()) = ComplianceState {
            zone_last: Some(Instant::now()),
            ..ComplianceState::NEW
        };
        *TAKEOFF_HOLD.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *PENDING_TAKEOFF_CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()) = false;
        *PENDING_GO_AROUND.lock().unwrap_or_else(|e| e.into_inner()) = false;
        *GO_AROUND_LAST.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *GHOSTS.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *WAKE.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *WAKE_PREV.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *TAXI_ROUTE.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *ACTIVE_PANEL.lock().unwrap_or_else(|e| e.into_inner()) =
            Some((self.apt.name.clone(), self.apt.elev_ft, services(&self.apt)));
        *ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner()) = self.apt.runways.clone();
        *ATIS.lock().unwrap_or_else(|e| e.into_inner()) =
            self.apt.freq(nav::FREQ_ATIS).map(|f| {
                tracing::info!(mhz = f.khz as f64 / 1000.0, "ATIS broadcasting on frequency");
                AtisState {
                    khz: f.khz,
                    airport: spoken_airport(&self.apt.name).to_string(),
                    runways: self.apt.runways.clone(),
                    letter: 'A',
                    edition: None,
                    last_spoken: None,
                }
            });
    }

    /// Content routing (shared PTT): if the utterance is a radio call AND COM1
    /// is on one of this station's frequencies, answer it; `None` falls
    /// through to the copilot untouched. Words are checked before any dataref
    /// read, so the common case (a cockpit command) costs nothing.
    ///
    /// Unlike a keyed transmission, an unkeyed utterance must SOUND like a
    /// radio call — addressed to a station, or classic call vocabulary — not
    /// merely parse to an intent. "Anything else we need to do before
    /// takeoff?" (a question for the crew) earned a takeoff clearance on the
    /// 2026-07-08 flight because "takeoff" alone parsed as a request; the
    /// panel's "land the aircraft" button was hijacked into a join clearance
    /// the same way.
    pub async fn try_handle(
        &mut self,
        transcript: &str,
        xp: SimView<'_>,
    ) -> Option<(String, StationSpeaker)> {
        // Cheap word gates before any dataref read — the common case is a
        // cockpit command, and it must cost nothing. route_unkeyed re-applies
        // the same gates; only the trace differs per exit.
        if !is_radio_call(transcript, &spoken_airport(&self.apt.name).to_lowercase()) {
            return None;
        }
        let intent = tower::parse_intent(transcript);
        if intent == tower::Intent::Unknown {
            // Radio-shaped words that didn't parse — exactly the utterances
            // corpus mining wants to see after a flight.
            sim::blackbox::record(
                "atc",
                serde_json::json!({
                    "route": "unkeyed", "outcome": "fallthrough",
                    "transcript": transcript,
                }),
            );
            return None;
        }
        let Some(service) = self.tuned_service(xp).await else {
            sim::blackbox::record(
                "atc",
                serde_json::json!({
                    "route": "unkeyed", "outcome": "off_frequency",
                    "transcript": transcript, "intent": format!("{intent:?}"),
                }),
            );
            return None;
        };
        let (ctx, wx) = self.build_ctx(xp).await;
        let before = self.twr.state;
        let reply = self.route_unkeyed(transcript, service, &ctx, wx)?;
        sim::blackbox::record(
            "atc",
            serde_json::json!({
                "route": "unkeyed", "outcome": "reply", "service": service.name(),
                "transcript": transcript, "intent": format!("{intent:?}"),
                "state": format!("{before:?}>{:?}", self.twr.state), "reply": reply,
            }),
        );
        Some((reply, service.speaker()))
    }

    /// The pure half of [`Station::try_handle`]: the unkeyed gates plus the
    /// tuned service's answer, no simulator anywhere. `None` falls through to
    /// the copilot.
    fn route_unkeyed(
        &mut self,
        transcript: &str,
        service: Service,
        ctx: &tower::Ctx,
        wx: Wx,
    ) -> Option<String> {
        if !is_radio_call(transcript, &spoken_airport(&self.apt.name).to_lowercase()) {
            return None;
        }
        self.note_information(transcript);
        let intent = tower::parse_intent(transcript);
        if intent == tower::Intent::Unknown {
            return None;
        }
        // Unkeyed "change frequency to the tower" is the pilot telling the
        // COPILOT to retune (the tune_request path) — only a KEYED
        // transmission makes it a request to the controller.
        if intent == tower::Intent::RequestFrequencyChange {
            return None;
        }
        Some(self.respond(service, intent, ctx, wx))
    }

    /// Remember "with information X" from any call — the next clearance skips
    /// the QNH when the letter matches the current broadcast.
    fn note_information(&mut self, transcript: &str) {
        if let Some(l) = heard_information(&transcript.to_lowercase()) {
            tracing::info!(letter = %l, "pilot reports having the information");
            self.pilot_info = Some(l);
        }
    }

    /// Physical routing (the radio PTT was keyed): the utterance was
    /// TRANSMITTED, so it goes to the tuned station or nowhere — never to the
    /// copilot. On frequency, even an unparsed call gets the controller's "say
    /// again" (a real controller answers garble; your crew doesn't). `None`
    /// means nobody was listening on that frequency.
    pub async fn handle_transmission(
        &mut self,
        transcript: &str,
        xp: SimView<'_>,
    ) -> Option<(String, StationSpeaker)> {
        let Some(service) = self.tuned_service(xp).await else {
            sim::blackbox::record(
                "atc",
                serde_json::json!({
                    "route": "keyed", "outcome": "void", "transcript": transcript,
                }),
            );
            return None;
        };
        let (ctx, wx) = self.build_ctx(xp).await;
        let before = self.twr.state;
        let reply = self.route_keyed(transcript, service, &ctx, wx);
        sim::blackbox::record(
            "atc",
            serde_json::json!({
                "route": "keyed",
                "outcome": if reply.is_empty() { "silence" } else { "reply" },
                "service": service.name(), "transcript": transcript,
                "intent": format!("{:?}", tower::parse_intent(transcript)),
                "state": format!("{before:?}>{:?}", self.twr.state), "reply": reply,
            }),
        );
        Some((reply, service.speaker()))
    }

    /// The pure half of [`Station::handle_transmission`]: everything after
    /// "somebody is listening on this frequency". Empty string = a readback
    /// the controller accepted (the frequency stays quiet).
    fn route_keyed(
        &mut self,
        transcript: &str,
        service: Service,
        ctx: &tower::Ctx,
        wx: Wx,
    ) -> String {
        self.note_information(transcript);
        let intent = tower::parse_intent(transcript);
        // The readback gate first: a runway-critical clearance is waiting
        // for its echo. Correct → silence; wrong runway → correction;
        // anything else → the challenge (which repeats the instruction).
        // Runs before the generic readback-silence path so a WRONG runway is
        // caught, not waved through as "a readback".
        if service == Service::Tower {
            if let Some(reply) = self.twr.gate(transcript, intent, ctx) {
                if reply.is_empty() {
                    // Correct readback: the debt is paid; tell the coach.
                    if let Some(c) =
                        CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()).as_mut()
                    {
                        c.pending_readback = None;
                    }
                    return String::new();
                }
                if self.enforce_readback {
                    return reply;
                }
                // Enforcement off (XCOPILOT_ATC_READBACK=false): no
                // challenge — route the call normally; the coach still
                // shows the owed line.
            }
        }
        // A pilot readback earns SILENCE, like a real frequency: the
        // controller only speaks up to correct a wrong one.
        // Exception: the first call to DEPARTURE is a check-in ("passing one
        // thousand five hundred climbing"), and a check-in gets radar contact.
        //
        // "Cleared ..." can only be an echo of a clearance — a pilot ASKING
        // says "request clearance", never "cleared". Without this, reading
        // back "cleared to land runway two seven" re-parsed as a landing
        // REQUEST: the tower answered "join final" again and the sequence
        // walked backwards from ClearedToLand to Inbound.
        let echoes_clearance = transcript
            .to_lowercase()
            .split(|c: char| !c.is_alphanumeric())
            .any(|w| w == "cleared");
        if echoes_clearance || (intent == tower::Intent::Unknown && tower::is_readback(transcript))
        {
            if service == Service::Departure && !self.dep_checked_in {
                self.dep_checked_in = true;
                return format!("{}, {} Departure, radar contact.", ctx.callsign, ctx.airport);
            }
            return String::new();
        }
        self.respond(service, intent, ctx, wx)
    }

    /// The voice of whichever of this station's positions COM1 is tuned to.
    pub async fn tuned_speaker(&self, xp: SimView<'_>) -> Option<StationSpeaker> {
        self.tuned_service(xp).await.map(Service::speaker)
    }

    /// Does this frequency (kHz) belong to one of the station's services?
    /// Same 5 kHz tolerance as [`Station::tuned_service`], no dataref read —
    /// callers with a live COM1 value (the STT prompt switch) check for free.
    pub fn covers_khz(&self, khz: i64) -> bool {
        (khz - self.tower_khz).abs() <= 5
            || self.approach_khz.is_some_and(|k| (khz - k).abs() <= 5)
            || self.departure_khz.is_some_and(|k| (khz - k).abs() <= 5)
    }

    /// Which service COM1 is tuned to. 8.33 channel names can sit 5 kHz off
    /// the carrier; adjacent channels are 8 kHz apart, so 5 can't cross.
    async fn tuned_service(&self, xp: SimView<'_>) -> Option<Service> {
        let com1 = xp.get_float(datarefs::COM1_FREQ_KHZ).await.ok()?.round() as i64;
        if (com1 - self.tower_khz).abs() <= 5 {
            return Some(Service::Tower);
        }
        if self.approach_khz.is_some_and(|k| (com1 - k).abs() <= 5) {
            return Some(Service::Approach);
        }
        if self.departure_khz.is_some_and(|k| (com1 - k).abs() <= 5) {
            return Some(Service::Departure);
        }
        None
    }

    /// Answer one transmission on whichever service is tuned. Pure: the
    /// context and wind were read before the words reached the controller.
    fn respond(
        &mut self,
        service: Service,
        intent: tower::Intent,
        ctx: &tower::Ctx,
        wx: Wx,
    ) -> String {
        let was = self.twr.state;
        // A departure clearance in a RADAR environment carries a discrete
        // code — the zone clearance is where a real controller assigns it
        // (Heathrow flew a whole day on 7000, 2026-07-10). Tower-only strips
        // keep the VFR conspicuity. The watch's "radar contact" belongs to
        // whoever has the scope.
        let assigned_ctx;
        let ctx = if service == Service::Tower
            && intent == tower::Intent::RequestClearance
            && self.assigned_squawk.is_none()
            && (self.approach_khz.is_some() || self.departure_khz.is_some())
        {
            let scope = if self.departure_khz.is_some() {
                StationSpeaker::Departure
            } else {
                StationSpeaker::Approach
            };
            let spoken = self.assign_squawk(&ctx.callsign, scope);
            assigned_ctx = tower::Ctx { squawk: spoken, ..ctx.clone() };
            &assigned_ctx
        } else {
            ctx
        };
        let reply = self.respond_inner(service, intent, ctx, wx);
        // Publish the clearance picture for the monitor's compliance watch:
        // two-way contact has happened, and this is what the pilot is (and
        // isn't) cleared to do. Same reply-path-owns-its-static pattern as
        // HANDOFF and SQUAWK_WATCH.
        *CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()) = Some(ClearanceSnap {
            state: self.twr.state,
            callsign: ctx.callsign.clone(),
            runway_ident: self.runway_in_use.clone().unwrap_or_default(),
            pending_readback: self.twr.suggested_readback(&ctx.callsign),
        });
        // A fresh taxi clearance publishes its route for the off-route
        // compliance watch (and clears any stale one when route-less).
        if service == Service::Tower
            && intent == tower::Intent::RequestTaxi
            && self.twr.state == tower::State::TaxiCleared
        {
            *TAXI_ROUTE.lock().unwrap_or_else(|e| e.into_inner()) =
                self.planned_route.as_ref().filter(|r| r.path.len() >= 2).map(|r| TaxiWatch {
                    path: r.path.clone(),
                    via: spoken_via(&r.via).join(", "),
                    seg: r.seg.clone(),
                    holding_point: r.holding_point.clone(),
                });
        }
        // A takeoff request answered with anything SHORT of the clearance —
        // hold position (traffic, wake, occupied strip), a conditional or
        // plain line-up, a cancellation — arms the release watch, so the
        // tower comes back unprompted the moment the runway is truly free.
        if service == Service::Tower
            && intent == tower::Intent::RequestTakeoff
            && matches!(
                was,
                tower::State::TaxiCleared | tower::State::LinedUp | tower::State::TakeoffCleared
            )
            && self.twr.state != tower::State::TakeoffCleared
            && (ctx.final_traffic_nm.is_some()
                || ctx.runway_occupied
                || ctx.wake_secs_left.is_some())
        {
            *TAKEOFF_HOLD.lock().unwrap_or_else(|e| e.into_inner()) = Some(TakeoffHold {
                callsign: ctx.callsign.clone(),
                runway: ctx.runway.clone(),
                runway_ident: self.runway_in_use.clone().unwrap_or_default(),
                clear_since: None,
            });
            tracing::info!("takeoff clearance withheld; release watch armed");
        }
        reply
    }

    /// Sync the tower FSM with the held takeoff clearance the MONITOR spoke
    /// once the final cleared (see [`takeoff_hold_tick`]): the state moves to
    /// TakeoffCleared and the departure handoff arms, exactly as if the
    /// clearance had been issued on the reply path.
    pub async fn apply_monitor_takeoff_clearance(&mut self, xp: SimView<'_>) {
        let (ctx, _wx) = self.build_ctx(xp).await;
        self.twr.state = tower::State::TakeoffCleared;
        // The released clearance owes its readback exactly like one issued
        // on the reply path.
        self.twr.arm_takeoff_readback(
            ctx.runway.clone(),
            format!("runway {}, cleared for takeoff.", ctx.runway),
        );
        if let Some(c) = CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()).as_mut() {
            c.pending_readback = self.twr.suggested_readback(&ctx.callsign);
        }
        self.arm_departure_handoff(&ctx);
    }

    /// Sync the tower FSM with the go-around order the MONITOR spoke (see
    /// [`go_around_tick`]): the landing clearance is void and the flight is
    /// back in the pattern, exactly as if the pilot had called it. The
    /// coach's next line becomes the acknowledgment the order deserves,
    /// never the readback of the clearance that just died.
    pub fn apply_monitor_go_around(&mut self) {
        self.twr.apply_go_around();
        if let Some(c) = CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()).as_mut() {
            c.state = tower::State::Inbound;
            c.pending_readback = Some(format!("Going around, {}.", c.callsign));
        }
    }

    fn respond_inner(
        &mut self,
        service: Service,
        intent: tower::Intent,
        ctx: &tower::Ctx,
        wx: Wx,
    ) -> String {
        match service {
            Service::Tower => {
                let was = self.twr.state;
                let reply = self.twr.handle(intent, ctx);
                // A fresh takeoff clearance arms the after-departure handoff:
                // once airborne and climbing, the tower sends the flight to
                // departure (or approach) — the frequency chain, not a tower
                // that owns you forever.
                if self.twr.state == tower::State::TakeoffCleared
                    && was != tower::State::TakeoffCleared
                {
                    self.arm_departure_handoff(ctx);
                }
                reply
            }
            Service::Approach => self.respond_approach(intent, ctx, wx),
            Service::Departure => self.respond_departure(intent, ctx, wx),
        }
    }

    /// The departure controller: takes the handoff after takeoff, then sends
    /// arrivals to approach (vectors) and everything runway-shaped back to the
    /// tower.
    fn respond_departure(&mut self, intent: tower::Intent, ctx: &tower::Ctx, wx: Wx) -> String {
        match intent {
            tower::Intent::RadioCheck => {
                format!(
                    "{}, {} Departure, reading you five.",
                    ctx.callsign, ctx.airport
                )
            }
            tower::Intent::WindCheck => {
                format!("{}, wind {}{}.", ctx.callsign, ctx.wind, tower::qnh_clause(ctx))
            }
            tower::Intent::SquawkCheck => {
                format!("{}, squawk seven zero zero zero.", ctx.callsign)
            }
            // Coming back in: vectors are approach's job when the field has
            // one; otherwise departure does what approach would.
            tower::Intent::RequestVectors => match &ctx.approach {
                Some(freq) => format!(
                    "{}, contact {} Approach on {} for vectors.",
                    ctx.callsign, ctx.airport, freq
                ),
                None => self.arm_vectors(ctx, wx),
            },
            tower::Intent::RequestLanding { touch_and_go } => match &ctx.approach {
                Some(freq) => {
                    self.twr.expect_touch_and_go(touch_and_go);
                    format!(
                        "{}, contact {} Approach on {} for vectors.",
                        ctx.callsign, ctx.airport, freq
                    )
                }
                None => {
                    self.twr.expect_touch_and_go(touch_and_go);
                    self.arm_vectors(ctx, wx)
                }
            },
            // A mayday on the departure frequency: departure keeps the flight
            // (an emergency is never told to go retune) and vectors it home.
            // On the ground there is nothing to vector — send the trucks.
            tower::Intent::Mayday if ctx.on_ground => format!(
                "{}, roger mayday. Emergency services on their way to you — hold position if able.",
                ctx.callsign
            ),
            tower::Intent::Mayday => {
                self.twr.expect_touch_and_go(false);
                let vectors = self.arm_vectors(ctx, wx);
                format!("Roger mayday. {vectors}")
            }
            // Runway-shaped business belongs to the tower.
            tower::Intent::ReportFinal
            | tower::Intent::ReportDownwind
            | tower::Intent::ReportBase
            | tower::Intent::ReportCrosswind
            | tower::Intent::GoAround
            | tower::Intent::RequestTakeoff
            | tower::Intent::RequestTaxi
            | tower::Intent::RequestTaxiIn
            | tower::Intent::RequestStartup
            | tower::Intent::RequestShutdown
            | tower::Intent::RequestClearance => format!(
                "{}, contact {} Tower on {}.",
                ctx.callsign,
                ctx.airport,
                atis::spoken_mhz(self.tower_khz)
            ),
            tower::Intent::Greeting => {
                format!("{}, {} Departure, pass your message.", ctx.callsign, ctx.airport)
            }
            tower::Intent::RequestFrequencyChange => {
                format!("{}, frequency change approved.", ctx.callsign)
            }
            // Departure has the scope: assign the code; "radar contact" comes
            // from the watch tick once the code actually squawks.
            tower::Intent::RequestFlightFollowing => {
                let squawk = self.assign_squawk(&ctx.callsign, StationSpeaker::Departure);
                format!(
                    "{}, squawk {}. Maintain VFR, flight following.",
                    ctx.callsign, squawk
                )
            }
            tower::Intent::Unknown => {
                format!("{}, {} Departure, say again.", ctx.callsign, ctx.airport)
            }
        }
    }

    /// Issue this flight its discrete code (idempotent — the same flight
    /// keeps the same code) and arm the radar watch. Returns the spoken code
    /// for the reply. The watch side effect mirrors [`Station::arm_departure_handoff`]'s
    /// `HANDOFF` write: reply paths own their monitor-tick state.
    fn assign_squawk(&mut self, callsign: &str, speaker: StationSpeaker) -> String {
        let code = *self
            .assigned_squawk
            .get_or_insert_with(|| discrete_squawk(callsign));
        let mut w = SQUAWK_WATCH.lock().unwrap_or_else(|e| e.into_inner());
        if w.as_ref().map(|w| w.code) != Some(code) {
            *w = Some(SquawkWatch {
                code,
                callsign: callsign.to_string(),
                speaker,
                observed: false,
                first_seen: None,
                last_nudge: None,
                nudges: 0,
            });
        }
        atis::spoken_digits(&code.to_string())
    }

    /// Arm the after-takeoff handoff. Spoken by [`handoff_tick`] once the
    /// flight is airborne and climbing through ~700 ft above the field —
    /// a tower doesn't say "contact departure" to an aircraft still rolling.
    fn arm_departure_handoff(&mut self, ctx: &tower::Ctx) {
        // Prefer departure; fall back to approach; a tower-only field keeps
        // its traffic and no handoff arms.
        let (name, khz) = match (self.departure_khz, self.approach_khz) {
            (Some(k), _) => ("Departure", k),
            (None, Some(k)) => ("Approach", k),
            (None, None) => return,
        };
        self.dep_checked_in = false;
        *HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = Some(Handoff {
            line: format!(
                "{}, contact {} {} on {}.",
                ctx.callsign,
                ctx.airport,
                name,
                atis::spoken_mhz(khz)
            ),
            min_alt_ft: self.apt.elev_ft + 700.0,
        });
        tracing::info!(to = name, "departure handoff armed");
    }

    /// The approach controller: vectors to final, or a redirect to the tower
    /// for anything that belongs there.
    fn respond_approach(&mut self, intent: tower::Intent, ctx: &tower::Ctx, wx: Wx) -> String {
        match intent {
            tower::Intent::RadioCheck => {
                format!(
                    "{}, {} Approach, reading you five.",
                    ctx.callsign, ctx.airport
                )
            }
            tower::Intent::WindCheck => {
                format!("{}, wind {}{}.", ctx.callsign, ctx.wind, tower::qnh_clause(ctx))
            }
            tower::Intent::SquawkCheck => {
                format!("{}, squawk seven zero zero zero.", ctx.callsign)
            }
            tower::Intent::RequestVectors => self.arm_vectors(ctx, wx),
            tower::Intent::RequestLanding { touch_and_go } => {
                // The option request rides through the vectors: the tower's
                // eventual clearance must say "touch and go", not "land".
                self.twr.expect_touch_and_go(touch_and_go);
                self.arm_vectors(ctx, wx)
            }
            // A mayday to approach: keep the flight, vector it home with
            // priority — an emergency is never told to go retune. On the
            // ground there is nothing to vector — send the trucks.
            tower::Intent::Mayday if ctx.on_ground => format!(
                "{}, roger mayday. Emergency services on their way to you — hold position if able.",
                ctx.callsign
            ),
            tower::Intent::Mayday => {
                self.twr.expect_touch_and_go(false);
                let vectors = self.arm_vectors(ctx, wx);
                format!("Roger mayday. {vectors}")
            }
            // "Established ILS two seven right" (parses as the final report)
            // while being vectored: approach's job is done — acknowledge,
            // hand the flight to the tower, drop the plan so the tick stops
            // steering a pilot who's already on the localizer.
            tower::Intent::ReportFinal
                if VECTORS.lock().unwrap_or_else(|e| e.into_inner()).is_some() =>
            {
                *VECTORS.lock().unwrap_or_else(|e| e.into_inner()) = None;
                format!(
                    "{}, roger. Contact {} Tower on {}.",
                    ctx.callsign,
                    ctx.airport,
                    atis::spoken_mhz(self.tower_khz)
                )
            }
            // Final reports and everything runway-shaped belong to the tower.
            tower::Intent::ReportFinal
            | tower::Intent::ReportDownwind
            | tower::Intent::ReportBase
            | tower::Intent::ReportCrosswind
            | tower::Intent::GoAround
            | tower::Intent::RequestTakeoff
            | tower::Intent::RequestTaxi
            | tower::Intent::RequestTaxiIn
            | tower::Intent::RequestStartup
            | tower::Intent::RequestShutdown
            | tower::Intent::RequestClearance => format!(
                "{}, contact {} Tower on {}.",
                ctx.callsign,
                ctx.airport,
                atis::spoken_mhz(self.tower_khz)
            ),
            tower::Intent::Greeting => {
                format!("{}, {} Approach, pass your message.", ctx.callsign, ctx.airport)
            }
            tower::Intent::RequestFrequencyChange => {
                format!("{}, frequency change approved.", ctx.callsign)
            }
            // Approach has the scope: assign the code; "radar contact" comes
            // from the watch tick once the code actually squawks.
            tower::Intent::RequestFlightFollowing => {
                let squawk = self.assign_squawk(&ctx.callsign, StationSpeaker::Approach);
                format!(
                    "{}, squawk {}. Maintain VFR, flight following.",
                    ctx.callsign, squawk
                )
            }
            tower::Intent::Unknown => {
                format!("{}, {} Approach, say again.", ctx.callsign, ctx.airport)
            }
        }
    }

    /// Arm the vectors-to-final plan and speak the opening instruction. The
    /// monitor's tick ([`vector_tick`]) takes it from here.
    fn arm_vectors(&mut self, ctx: &tower::Ctx, wx: Wx) -> String {
        // The runway END in use: its threshold is the aim point, its heading
        // the final approach course. Same held pick as every clearance, so
        // vectors and the tower always agree on the runway.
        let ident = self.pick_runway(wx.wind_from_deg, wx.wind_kts);
        if ident.is_empty() {
            return format!("{}, unable, no runway data.", ctx.callsign);
        }
        let Some((thr_lat, thr_lon, course_deg)) = self.apt.runways.iter().find_map(|r| {
            if r.ident_a == ident {
                Some((r.lat_a, r.lon_a, r.heading_a_deg()))
            } else if r.ident_b == ident {
                Some((r.lat_b, r.lon_b, r.heading_b_deg()))
            } else {
                None
            }
        }) else {
            return format!("{}, unable, no runway data.", ctx.callsign);
        };
        // ILS when the runway has one (background load may still be running:
        // then it's a visual, exactly as before — honest about what's known).
        let facts = ILS_FACTS.lock().unwrap_or_else(|e| e.into_inner()).clone();
        let ils_rec = facts
            .as_ref()
            .and_then(|f| f.ils.iter().find(|i| i.runway == ident).cloned());
        let ils = ils_rec.is_some();
        // Intercept altitude: the CIFP's real platform (glideslope-intercept)
        // when the procedure is on disk; otherwise a geometric one — 2,500 ft
        // above the field for an ILS (≈3° from 8 nm), 1,500 for the visual.
        let target_alt_ft = facts
            .as_ref()
            .filter(|_| ils)
            .and_then(|f| f.platform_ft.get(&ident).copied())
            .unwrap_or_else(|| {
                let aal = if ils { 2500.0 } else { 1500.0 };
                ((self.apt.elev_ft + aal) / 100.0).round() * 100.0
            });
        // The ILS join sits where the real slope (charted angle when we have
        // it) is still above the platform — intercept from below, always.
        let join_nm = match &ils_rec {
            Some(rec) => {
                ils_join_nm(target_alt_ft - self.apt.elev_ft, rec.gs_deg.unwrap_or(3.0))
            }
            None => JOIN_FINAL_NM,
        };
        let runway_spoken = atis::spoken_runway(&ident);
        *VECTORS.lock().unwrap_or_else(|e| e.into_inner()) = Some(VectorPlan {
            callsign: ctx.callsign.clone(),
            thr_lat,
            thr_lon,
            course_deg,
            runway_spoken: runway_spoken.clone(),
            tower_spoken: Some(atis::spoken_mhz(self.tower_khz)),
            target_alt_ft,
            ils,
            join_nm,
            cleared: false,
            dialect: self.dialect,
            // The arm REPLY (squawk + expect vectors + descend) is about to be
            // spoken — start the vector clock now so the first heading follows
            // after a breath (8 s for a big cut, 25 s otherwise), not stacked
            // onto it. Three transmissions inside two seconds (reply, radar
            // contact, first vector) read as spam, taped 2026-07-12; a real
            // controller consolidates and then lets the pilot fly.
            last_tx: Some(Instant::now()),
            last_heading: None,
        });
        // A radar service starts with a discrete code — "radar contact" is
        // spoken by the watch tick once the code actually shows on the
        // scope, not on faith (it used to be instant theatre).
        let squawk = self.assign_squawk(&ctx.callsign, StationSpeaker::Approach);
        tracing::info!(runway = %ident, alt = target_alt_ft, ils, "vectors armed");
        let plan_phrase = match (ils, self.dialect) {
            // CAP 413 / ICAO: "vectoring ILS approach", altitudes are "to
            // altitude ... feet" with the QNH attached.
            (true, Dialect::Icao) => format!(
                "Vectoring ILS approach runway {}. Descend to altitude {} feet{}.",
                runway_spoken,
                spoken_alt(target_alt_ft),
                tower::qnh_clause(ctx)
            ),
            // FAA: "expect vectors ILS runway ... approach", "descend and
            // maintain", and the pressure clause speaks altimeter in inches
            // (ctx.qnh carries the dialect-correct phrase).
            (true, Dialect::Faa) => format!(
                "Expect vectors ILS runway {} approach. Descend and maintain {}{}.",
                runway_spoken,
                spoken_alt(target_alt_ft),
                tower::qnh_clause(ctx)
            ),
            (false, _) => format!(
                "Expect vectors, visual approach runway {}. Descend and maintain {}{}.",
                runway_spoken,
                spoken_alt(target_alt_ft),
                tower::qnh_clause(ctx)
            ),
        };
        format!(
            "{}, {} Approach, squawk {}. {}",
            ctx.callsign, ctx.airport, squawk, plan_phrase
        )
    }

    /// The runway in use. Re-picked from the wind only when there IS wind
    /// (≥ 5 kt): in calm the into-wind pick flip-flops on noise, and a
    /// controller doesn't swap the runway between two transmissions.
    fn pick_runway(&mut self, wind_from: f64, wind_kts: f64) -> String {
        if wind_kts >= 5.0 || self.runway_in_use.is_none() {
            if let Some((id, _, _)) =
                nav::best_runway_for_wind(&self.apt.runways, wind_from, wind_kts)
            {
                self.runway_in_use = Some(id);
            }
        }
        self.runway_in_use.clone().unwrap_or_default()
    }

    /// Read the sim once and build the live context for a reply. Best-effort:
    /// a failed read falls back to silence on that item rather than a wrong
    /// number.
    async fn build_ctx(&mut self, xp: SimView<'_>) -> (tower::Ctx, Wx) {
        let wx = Wx {
            wind_from_deg: xp
                .get_float(datarefs::WIND_DIR_DEG_TRUE)
                .await
                .unwrap_or(0.0),
            wind_kts: xp.get_float(datarefs::WIND_SPEED_MSC).await.unwrap_or(0.0) * 1.943844,
        };
        let qnh = match xp.get_float(datarefs::QNH_PASCALS).await {
            Ok(pa) => spoken_pressure(pa, self.dialect),
            Err(_) => String::new(),
        };
        let callsign = xp
            .get_string(datarefs::TAILNUM)
            .await
            .ok()
            .map(|t| tower::spoken_callsign(&t))
            .filter(|s| !s.is_empty())
            .unwrap_or_else(|| "aircraft calling".to_string());
        let on_ground = xp.get_float(datarefs::ON_GROUND_ANY).await.unwrap_or(0.0) > 0.5;
        // The controller's own traffic picture, read once per call: an
        // arrival in OUR runway's corridor gates takeoff clearances and
        // sequences downwind joins; a ghost ON the strip blocks line-ups.
        // Restricted to the strip in use — the runway-blind version held a
        // Heathrow 09R departure for the 09L stream, indefinitely
        // (2026-07-10).
        let ident = self.pick_runway(wx.wind_from_deg, wx.wind_kts);
        let (final_traffic, runway_occupied) = match strip(&self.apt.runways, &ident) {
            Some(r) => {
                let r = r.clone();
                let targets = read_tcas_targets(xp).await;
                let occupied = targets
                    .iter()
                    .any(|t| t.on_ground && on_runway(&r, t.lat, t.lon).is_some());
                (final_traffic_for(&targets, &r, self.apt.elev_ft), occupied)
            }
            None => (None, false),
        };
        let wake_secs_left = if on_ground {
            wake_remaining(&ident, Instant::now())
        } else {
            None
        };
        // The taxi route from where the aircraft actually is, planned fresh
        // per ground call so the eventual clearance matches the stand you
        // are on, not the one you asked from an hour ago.
        self.planned_route = None;
        if on_ground {
            if let (Ok(lat), Ok(lon)) = (
                xp.get_float(datarefs::LATITUDE).await,
                xp.get_float(datarefs::LONGITUDE).await,
            ) {
                self.planned_route = plan_taxi_route_to(&self.apt.runways, &ident, lat, lon);
            }
        }
        (self.make_ctx(wx, qnh, callsign, on_ground, final_traffic, runway_occupied, wake_secs_left), wx)
    }

    /// The pure half of [`Station::build_ctx`] — everything after the sim
    /// reads, so tests phrase clearances from scripted weather.
    #[allow(clippy::too_many_arguments)]
    fn make_ctx(
        &mut self,
        wx: Wx,
        qnh: String,
        callsign: String,
        on_ground: bool,
        final_traffic: Option<FinalTraffic>,
        runway_occupied: bool,
        wake_secs_left: Option<u64>,
    ) -> tower::Ctx {
        let runway = self.pick_runway(wx.wind_from_deg, wx.wind_kts);
        // "With information X" matching the current broadcast: the clearance
        // skips the QNH — having the numbers is the whole point of an ATIS.
        // A stale or absent letter keeps them (safety over brevity).
        let qnh = if self.pilot_info.is_some() && self.pilot_info == atis_current_letter() {
            String::new()
        } else {
            qnh
        };
        let route = self.planned_route.as_ref();
        tower::Ctx {
            callsign,
            airport: spoken_airport(&self.apt.name).to_string(),
            runway: atis::spoken_runway(&runway),
            qnh,
            wind: spoken_wind(wx.wind_from_deg, wx.wind_kts),
            holding_point: route
                .map(|r| spoken_taxiway(&r.holding_point))
                .filter(|h| !h.is_empty())
                .unwrap_or_else(|| "Alpha".to_string()),
            approach: self.approach_khz.map(atis::spoken_mhz),
            squawk: self
                .assigned_squawk
                .map(|c| atis::spoken_digits(&c.to_string()))
                .unwrap_or_else(|| "seven zero zero zero".to_string()),
            final_traffic_nm: final_traffic.as_ref().map(|f| f.nm),
            final_traffic_callsign: final_traffic.as_ref().and_then(|f| f.callsign.clone()),
            final_traffic_count: final_traffic.as_ref().map(|f| f.count).unwrap_or(0),
            runway_occupied,
            wake_secs_left,
            taxi_via: route.map(|r| spoken_via(&r.via)).unwrap_or_default(),
            taxi_crossings: route
                .map(|r| {
                    r.crossings
                        .iter()
                        .map(|n| atis::spoken_runway(first_designator(n)))
                        .collect()
                })
                .unwrap_or_default(),
            on_ground,
        }
    }
}

/// Does an UNKEYED utterance sound like a radio call? Addressed to a station
/// by name, or built from call vocabulary a pilot only uses on the radio.
/// This is the content-routing gate: with the radio PTT keyed, keying itself
/// is the addressing and this is never consulted.
/// The field's SPOKEN name. apt.dat names arrive as gazetteer entries —
/// "Badajoz- Talavera La Real", "Merida - Royanejos" — but a controller keys
/// "Badajoz Tower", not the full hyphenated row (heard live 2026-07-10, and
/// it's also what the pilot actually says back, so addressing works too).
/// First dash-separated segment. Accepted tradeoff: "Paris - Charles de
/// Gaulle" shortens to "Paris"; for the GA fields this copilot flies, the
/// first segment is the radio name.
pub fn spoken_airport(name: &str) -> &str {
    name.split('-')
        .map(str::trim)
        .find(|s| !s.is_empty())
        .unwrap_or(name)
}

pub fn is_radio_call(t: &str, airport_lc: &str) -> bool {
    let word = |w: &str| t.split(|c: char| !c.is_alphanumeric()).any(|tok| tok == w);
    t.contains("tower")
        || t.contains("approach")
        || t.contains("departure")
        || (!airport_lc.is_empty() && t.contains(airport_lc))
        || t.contains("request")
        || t.contains("inbound")
        || t.contains("radio check")
        || word("final")
        // "Established (ILS/localizer)" is the vectored arrival's final
        // report — radio speech by nature, like the pattern calls below.
        || t.contains("establish")
        // Distress and pattern calls are radio speech by nature — nobody
        // tells their copilot "mayday" or reports "downwind" to the cockpit.
        || word("mayday")
        || t.contains("may day")
        || t.contains("pan pan")
        || word("declaring")
        || word("declare")
        || t.contains("going around")
        || t.contains("missed approach")
        || word("downwind")
        || word("crosswind")
        || word("upwind")
        || t.contains("turning base")
        || t.contains("left base")
        || t.contains("right base")
        || t.contains("base leg")
}

/// This flight's discrete squawk, deterministic from the callsign so tests
/// and a restarted session reproduce it. Codes live in the 45xx block with
/// octal last digits — every code valid, and the block is clear of all
/// reserved codes (0000, 1200, 2000, 7000, 7500, 7600, 7700).
pub fn discrete_squawk(callsign: &str) -> i64 {
    let seed: u32 = callsign.bytes().map(u32::from).sum();
    4500 + ((seed / 8) % 8) as i64 * 10 + (seed % 8) as i64
}

/// The radar watch on an assigned squawk: armed when a controller issues a
/// discrete code, ticked by the monitor. "Radar contact" is spoken ONCE the
/// sim actually shows the code squawking — until then it's an instruction,
/// not theatre — and while it doesn't, the controller nudges ("confirm
/// squawk ...") a couple of times, then gives up chasing like a real one.
pub struct SquawkWatch {
    pub code: i64,
    callsign: String,
    speaker: StationSpeaker,
    /// "Radar contact" has been spoken; the watch is satisfied.
    observed: bool,
    /// First tick after assignment — starts the grace period.
    first_seen: Option<Instant>,
    last_nudge: Option<Instant>,
    nudges: u32,
}

static SQUAWK_WATCH: Mutex<Option<SquawkWatch>> = Mutex::new(None);

/// Knob-spinning time before the first nudge.
const SQUAWK_GRACE_SECS: f64 = 20.0;
/// Quiet time between nudges.
const SQUAWK_NUDGE_SECS: f64 = 45.0;
/// Then the controller stops chasing.
const SQUAWK_MAX_NUDGES: u32 = 2;

/// Cheap gate for the monitor: only read the transponder datarefs while a
/// watch is armed.
pub fn squawk_watch_active() -> bool {
    SQUAWK_WATCH
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .is_some_and(|w| !w.observed)
}

/// One tick of the radar watch: the actual transponder code and whether the
/// mode is replying (ON/ALT). Returns what to speak and who says it.
pub fn squawk_tick(
    actual: Option<i64>,
    mode_on: bool,
    now: Instant,
) -> Option<(String, StationSpeaker)> {
    let mut guard = SQUAWK_WATCH.lock().unwrap_or_else(|e| e.into_inner());
    let w = guard.as_mut()?;
    let first = *w.first_seen.get_or_insert(now);
    if actual == Some(w.code) && mode_on {
        if w.observed {
            return None;
        }
        w.observed = true;
        return Some((format!("{}, radar contact.", w.callsign), w.speaker));
    }
    // Once contact was called, a later mode change isn't chased — the real
    // frequency stays quiet about it unless it matters (future: radar
    // contact lost on a long mismatch).
    if w.observed {
        return None;
    }
    if now.duration_since(first).as_secs_f64() < SQUAWK_GRACE_SECS
        || w.nudges >= SQUAWK_MAX_NUDGES
        || w.last_nudge
            .is_some_and(|t| now.duration_since(t).as_secs_f64() < SQUAWK_NUDGE_SECS)
    {
        return None;
    }
    w.last_nudge = Some(now);
    w.nudges += 1;
    // Two different problems, two different nudges: the code dialed right
    // with the transponder in standby got "confirm squawk 4503" (Gatwick,
    // 2026-07-13) — a nag that reads as "your digits are wrong" when the
    // digits are the one thing that's right.
    let line = if actual == Some(w.code) {
        format!("{}, negative radar contact, recycle transponder.", w.callsign)
    } else {
        format!(
            "{}, confirm squawk {}.",
            w.callsign,
            atis::spoken_digits(&w.code.to_string())
        )
    };
    Some((line, w.speaker))
}

/// Two-way contact plus the clearance picture, published by the reply path
/// after every answered call, for the monitor's compliance watch. `None`
/// until the pilot's first call to this station (and again after a swap) —
/// no contact means the tower doesn't know you, and the zone watch takes
/// over instead.
pub struct ClearanceSnap {
    pub state: tower::State,
    pub callsign: String,
    /// The runway END in use when this was published (e.g. "09R") — conflict
    /// watches restrict themselves to that strip; a parallel runway's
    /// traffic is somebody else's problem. Empty when no runway is known.
    pub runway_ident: String,
    /// The exact readback the pilot owes right now, phrased — the coach's
    /// first suggestion. `None` = no readback outstanding.
    pub pending_readback: Option<String>,
}

/// A compliance or zone call: what to say, and whether the COPILOT says it
/// (not tuned to the station — a controller can't berate a radio that isn't
/// listening; the crew flags it instead, like a real FO would).
pub struct ComplianceCall {
    pub line: String,
    pub crew: bool,
}

/// The compliance watch's memory between ticks.
struct ComplianceState {
    prev_on_ground: Option<bool>,
    /// Continuously airborne since — a landing accusation needs a FLIGHT
    /// first. A one-sample unstick during the takeoff roll (0.3 ft AGL for
    /// one tick at 83 knots, Jerez 2026-07-11) read as air-then-ground and
    /// earned "you landed without clearance" mid-departure.
    airborne_since: Option<Instant>,
    /// Last violation call of any kind — one thing at a time on frequency.
    last_call: Option<Instant>,
    /// When the tower state was last seen at ClearedToLand — grace for the
    /// rollout and vacating after a legal landing.
    cleared_to_land_seen: Option<Instant>,
    /// Each transition violation is called once, not every tick.
    takeoff_berated: bool,
    landing_berated: bool,
    zone_last: Option<Instant>,
    /// Last off-route taxi challenge — its own cooldown; wandering is a
    /// nudge, not a violation.
    off_route_last: Option<Instant>,
}

impl ComplianceState {
    const NEW: ComplianceState = ComplianceState {
        prev_on_ground: None,
        airborne_since: None,
        last_call: None,
        cleared_to_land_seen: None,
        takeoff_berated: false,
        landing_berated: false,
        zone_last: None,
        off_route_last: None,
    };
}

static COMPLIANCE: Mutex<ComplianceState> = Mutex::new(ComplianceState::NEW);

/// Between repeat violation calls.
const VIOLATION_COOLDOWN_SECS: f64 = 60.0;
/// Rollout/vacating time after a landing clearance before "on the runway
/// without clearance" can fire.
const VACATE_GRACE_SECS: f64 = 120.0;
/// The pseudo control zone around the active towered field: apt.dat has no
/// airspace geometry, so a cylinder stands in for the CTR.
const ZONE_RADIUS_NM: f64 = 5.0;
const ZONE_HEIGHT_FT: f64 = 3000.0;
const ZONE_COOLDOWN_SECS: f64 = 120.0;
/// Continuous air time before ground contact can be called a LANDING —
/// shorter is a skip on the takeoff roll or a bounce, not an arrival.
const MIN_FLIGHT_SECS: f64 = 30.0;

/// One tick of the compliance watch: the tower holds the pilot to the
/// clearances it actually issued, and challenges zone traffic it has never
/// talked to. Everything is derived from the published [`CLEARANCE`] state —
/// a tower that never saw you on its ground can't accuse you of an uncleared
/// takeoff.
pub fn compliance_tick(
    lat: f64,
    lon: f64,
    alt_ft: f64,
    on_ground: bool,
    gs_kts: f64,
    tuned: bool,
    now: Instant,
) -> Option<ComplianceCall> {
    let (field_lat, field_lon) = {
        let f = ACTIVE_FIELD.lock().unwrap_or_else(|e| e.into_inner());
        let (_, la, lo) = f.as_ref()?;
        (*la, *lo)
    };
    let dist_nm = crate::nav::haversine_nm(lat, lon, field_lat, field_lon);
    let clr: Option<(tower::State, String)> = CLEARANCE
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|c| (c.state, c.callsign.clone()));

    let mut w = COMPLIANCE.lock().unwrap_or_else(|e| e.into_inner());
    let prev_ground = w.prev_on_ground.replace(on_ground);
    // How long the flight before THIS ground contact lasted (0.0 while the
    // wheels never left) — read before the touchdown resets the clock.
    let airborne_secs =
        w.airborne_since.map_or(0.0, |t| now.duration_since(t).as_secs_f64());
    if on_ground {
        w.airborne_since = None;
        // Down: the landing-clearance coach is moot, and a stale handoff must
        // not nag on the next takeoff.
        *LANDING_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
    } else if w.airborne_since.is_none() {
        w.airborne_since = Some(now);
    }
    if clr.as_ref().is_some_and(|(s, _)| *s == tower::State::ClearedToLand) {
        w.cleared_to_land_seen = Some(now);
        // Cleared to land — the coach got what it was waiting for.
        *LANDING_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
    }
    let vacating = w
        .cleared_to_land_seen
        .is_some_and(|t| now.duration_since(t).as_secs_f64() < VACATE_GRACE_SECS);
    let quiet = w
        .last_call
        .is_some_and(|t| now.duration_since(t).as_secs_f64() < VIOLATION_COOLDOWN_SECS);

    if let Some((state, callsign)) = &clr {
        // Landing without clearance: airborne last tick, on the active runway
        // now, and the tower never said "cleared to land". The classic ends
        // with a phone number, exactly once.
        if prev_ground == Some(false)
            && on_ground
            && airborne_secs >= MIN_FLIGHT_SECS
            && !w.landing_berated
            && *state != tower::State::ClearedToLand
        {
            if let Some(rwy) = on_active_runway(lat, lon) {
                w.landing_berated = true;
                w.last_call = Some(now);
                let rwy = atis::spoken_runway(&rwy);
                return Some(if tuned {
                    ComplianceCall {
                        line: format!(
                            "{callsign}, you landed on runway {rwy} without clearance. \
                             Taxi to parking. I have a number for you to copy when ready."
                        ),
                        crew: false,
                    }
                } else {
                    ComplianceCall {
                        line: "That was a landing without clearance. Expect a phone number."
                            .to_string(),
                        crew: true,
                    }
                });
            }
        }
        // Takeoff without clearance: ground last tick, airborne now, near the
        // field, and the tower never cleared it. Once.
        if prev_ground == Some(true)
            && !on_ground
            && dist_nm < 5.0
            && !w.takeoff_berated
            && !matches!(
                state,
                tower::State::TakeoffCleared | tower::State::ClearedToLand
            )
        {
            w.takeoff_berated = true;
            w.last_call = Some(now);
            return Some(if tuned {
                ComplianceCall {
                    line: format!("{callsign}, you were not cleared for takeoff. Say intentions."),
                    crew: false,
                }
            } else {
                ComplianceCall {
                    line: "We just took off without a clearance.".to_string(),
                    crew: true,
                }
            });
        }
        // Off the cleared taxi route: taxiing (moving, on the ground, taxi
        // clearance current), a route was issued, and the aircraft has
        // wandered off its polyline. A nudge with the via line, on its own
        // cooldown — controllers redirect strays, they don't phone them in.
        if on_ground
            && (5.0..40.0).contains(&gs_kts)
            && *state == tower::State::TaxiCleared
            && !w
                .off_route_last
                .is_some_and(|t| now.duration_since(t).as_secs_f64() < 45.0)
            && on_active_runway(lat, lon).is_none()
        {
            let stray = TAXI_ROUTE
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .as_ref()
                .and_then(|r| {
                    let d = dist_to_polyline_m(&r.path, lat, lon)?;
                    (d > 60.0).then(|| (r.via.clone(), r.path.last().copied()))
                });
            if let Some((via, goal)) = stray {
                w.off_route_last = Some(now);
                if !tuned {
                    return Some(ComplianceCall {
                        line: "We're off the cleared taxi route.".to_string(),
                        crew: true,
                    });
                }
                // A stray gets redirected from PRESENT POSITION: replan to
                // the same holding point and re-clear, instead of repeating
                // a via line for pavement the pilot left behind (the same
                // stale via spoken seven times across one Malaga taxi while
                // the pilot was 680 m off it, 2026-07-13). The republished
                // route also re-aims the taxi guide.
                let fresh = goal.and_then(|(glat, glon)| {
                    let net = TAXI_NET.lock().unwrap_or_else(|e| e.into_inner()).clone()?;
                    net.plan(lat, lon, glat, glon).filter(|r| r.path.len() >= 2)
                });
                let line = if let Some(r) = fresh {
                    let via = spoken_via(&r.via).join(", ");
                    let hp = spoken_taxiway(&r.holding_point);
                    *TAXI_ROUTE.lock().unwrap_or_else(|e| e.into_inner()) =
                        Some(TaxiWatch {
                            path: r.path.clone(),
                            via: via.clone(),
                            seg: r.seg.clone(),
                            holding_point: r.holding_point.clone(),
                        });
                    format!(
                        "{callsign}, you appear to be off the cleared taxi route. \
                         Taxi to holding point {hp} via {via}."
                    )
                } else if via.is_empty() {
                    format!(
                        "{callsign}, you appear to be off the cleared taxi route. \
                         Continue with caution to the holding point."
                    )
                } else {
                    format!(
                        "{callsign}, you appear to be off the cleared taxi route. \
                         Taxi via {via}."
                    )
                };
                return Some(ComplianceCall { line, crew: false });
            }
        }
        // On the active runway without any clearance to be there: not rolling
        // (a takeoff/landing roll is covered above), not vacating after a
        // legal landing, and the state says holding-point at best. Repeats on
        // the violation cooldown while it persists — runway occupancy is the
        // one the tower keeps shouting about.
        if on_ground
            && gs_kts < 30.0
            && !vacating
            && !quiet
            && !matches!(
                state,
                // LinedUp is the tower's OWN instruction to be on the runway.
                tower::State::LinedUp
                    | tower::State::TakeoffCleared
                    | tower::State::ClearedToLand
            )
        {
            if let Some(rwy) = on_active_runway(lat, lon) {
                // The tower's OWN taxi route may cross (or backtrack) a
                // runway — apt.dat networks do it all the time, and the
                // clearance speaks the crossing ("cross runway two seven").
                // On a runway-flagged leg of the cleared route the occupancy
                // is LEGAL: silence while it keeps moving, and a stopped
                // crossing earns "expedite" — never the berate (the tower
                // shouted "you are on runway two seven without clearance" at
                // a pilot following its own route, 2026-07-14).
                if *state == tower::State::TaxiCleared
                    && on_cleared_runway_crossing(lat, lon)
                {
                    if gs_kts < 3.0 {
                        w.last_call = Some(now);
                        let rwy = atis::spoken_runway(&rwy);
                        return Some(if tuned {
                            ComplianceCall {
                                line: format!(
                                    "{callsign}, expedite crossing runway {rwy}."
                                ),
                                crew: false,
                            }
                        } else {
                            ComplianceCall {
                                line: format!(
                                    "We've stopped on runway {rwy}. Keep it moving across."
                                ),
                                crew: true,
                            }
                        });
                    }
                    return None;
                }
                w.last_call = Some(now);
                let rwy = atis::spoken_runway(&rwy);
                return Some(if tuned {
                    ComplianceCall {
                        line: format!(
                            "{callsign}, you are on runway {rwy} without clearance. \
                             Exit the runway immediately."
                        ),
                        crew: false,
                    }
                } else {
                    ComplianceCall {
                        line: format!(
                            "We're on runway {rwy} without clearance. We should exit."
                        ),
                        crew: true,
                    }
                });
            }
        }
        return None;
    }

    // No two-way contact at all: the zone watch. Inside the pseudo-CTR
    // (cylinder: 5 nm, surface to 3,000 ft above the field), airborne, and
    // the tower has never heard from you — a real controller challenges,
    // a real FO squirms.
    if !on_ground && dist_nm < ZONE_RADIUS_NM {
        let (zone_name, elev_ft, tower_khz) = {
            let p = ACTIVE_PANEL.lock().unwrap_or_else(|e| e.into_inner());
            let (name, elev, svcs) = p.as_ref()?;
            let khz = svcs.iter().find(|(w, _)| w == "tower").map(|(_, k)| *k);
            (spoken_airport(name).to_string(), *elev, khz)
        };
        if alt_ft < elev_ft + ZONE_HEIGHT_FT
            && !w
                .zone_last
                .is_some_and(|t| now.duration_since(t).as_secs_f64() < ZONE_COOLDOWN_SECS)
        {
            w.zone_last = Some(now);
            return Some(if tuned {
                ComplianceCall {
                    line: format!(
                        "Aircraft in the {zone_name} control zone, you are inside \
                         controlled airspace without clearance. Say callsign and intentions."
                    ),
                    crew: false,
                }
            } else {
                ComplianceCall {
                    line: match tower_khz {
                        Some(k) => format!(
                            "We're inside the {zone_name} zone without clearance. \
                             Tower is on {}.",
                            atis::spoken_mhz(k)
                        ),
                        None => format!("We're inside the {zone_name} zone without clearance."),
                    },
                    crew: true,
                }
            });
        }
    }
    None
}

/// The tower's after-takeoff handoff, armed by a takeoff clearance and spoken
/// once airborne — see [`Station::arm_departure_handoff`] and [`handoff_tick`].
pub struct Handoff {
    /// The full transmission, prebuilt ("… contact Gibraltar Departure on …").
    pub line: String,
    /// Speak once climbing through this altitude (ft MSL, field + 700).
    pub min_alt_ft: f64,
}

static HANDOFF: Mutex<Option<Handoff>> = Mutex::new(None);

/// One tick of the after-takeoff handoff, ~monitor rate: silence until the
/// flight is airborne and through the arming altitude, then the handoff
/// exactly once.
pub fn handoff_tick(alt_ft: f64, on_ground: bool) -> Option<String> {
    let mut guard = HANDOFF.lock().unwrap_or_else(|e| e.into_inner());
    let plan = guard.as_ref()?;
    if on_ground || alt_ft < plan.min_alt_ft {
        return None;
    }
    let line = plan.line.clone();
    *guard = None;
    Some(line)
}

/// The in-process ATIS: one broadcast state per station, driven by the
/// monitor's tick while COM1 sits on the ATIS frequency. Editions cut and the
/// letter advances exactly as the standalone binary did (`atis::new_edition`
/// hysteresis included); the report always reads the FROZEN edition, so the
/// numbers can't drift mid-cycle.
pub struct AtisState {
    pub khz: i64,
    airport: String,
    runways: Vec<nav::Runway>,
    letter: char,
    edition: Option<atis::Snapshot>,
    last_spoken: Option<Instant>,
}

static ATIS: Mutex<Option<AtisState>> = Mutex::new(None);

/// Re-broadcast cadence while the pilot stays tuned — a real ATIS loops.
const ATIS_REPEAT_SECS: f64 = 45.0;

/// The live weather one ATIS cycle is cut from (read by the monitor).
#[derive(Debug, Clone, Copy)]
pub struct AtisWx {
    pub zulu_sec: f64,
    pub wind_from_deg: f64,
    pub wind_kts: f64,
    pub visibility_sm: f64,
    pub qnh_hpa: f64,
    pub oat_c: f64,
}

/// The monitor's cheap gate: true when an ATIS exists, COM1 is on it, AND a
/// broadcast is due — only then is the full weather set worth reading (six
/// REST GETs; reading them every tick while parked on the frequency was
/// measurable main-thread load on the sim). Off frequency it also rewinds the
/// loop so a retune starts the broadcast from the top.
pub fn atis_due(com1_khz: i64, now: Instant) -> bool {
    let mut guard = ATIS.lock().unwrap_or_else(|e| e.into_inner());
    let Some(st) = guard.as_mut() else { return false };
    if (com1_khz - st.khz).abs() > 5 {
        st.last_spoken = None;
        return false;
    }
    st.last_spoken
        .is_none_or(|t| now.duration_since(t).as_secs_f64() >= ATIS_REPEAT_SECS)
}

/// Follow COM1 to any in-range field's ATIS. The ATC tab lets the pilot tune
/// every listed frequency, but the broadcast only existed for the ACTIVE
/// station — tuning another field's ATIS was dead air (2026-07-10). Swapping
/// fields restarts that field's loop at information Alpha.
pub fn atis_retune(com1_khz: i64, lat: f64, lon: f64, alt_msl_ft: f64) {
    {
        let cur = ATIS.lock().unwrap_or_else(|e| e.into_inner());
        if cur.as_ref().is_some_and(|a| (a.khz - com1_khz).abs() <= 5) {
            return; // already broadcasting on this frequency
        }
    }
    let region = REGION.lock().unwrap_or_else(|e| e.into_inner());
    let hit = region.iter().find(|a| {
        a.apt
            .freq(nav::FREQ_ATIS)
            .is_some_and(|f| (f.khz - com1_khz).abs() <= 5)
            && crate::nav::haversine_nm(lat, lon, a.lat, a.lon)
                <= vhf_range_nm(alt_msl_ft - a.apt.elev_ft)
    });
    if let Some(a) = hit {
        let khz = a.apt.freq(nav::FREQ_ATIS).map(|f| f.khz).unwrap_or(com1_khz);
        tracing::info!(
            airport = %a.apt.name,
            mhz = khz as f64 / 1000.0,
            "ATIS following COM1 to another field"
        );
        *ATIS.lock().unwrap_or_else(|e| e.into_inner()) = Some(AtisState {
            khz,
            airport: spoken_airport(&a.apt.name).to_string(),
            runways: a.apt.runways.clone(),
            letter: 'A',
            edition: None,
            last_spoken: None,
        });
    }
}

/// One tick of the ATIS broadcast: the full report when tuned and due (first
/// tune-in speaks immediately, then the loop repeats), silence otherwise.
/// Cuts a new edition — advancing the letter — when the weather has genuinely
/// moved (see `atis::new_edition`).
pub fn atis_tick(com1_khz: i64, wx: &AtisWx, now: Instant) -> Option<String> {
    let mut guard = ATIS.lock().unwrap_or_else(|e| e.into_inner());
    let st = guard.as_mut()?;
    if (com1_khz - st.khz).abs() > 5 {
        st.last_spoken = None; // retuning restarts the loop from the top
        return None;
    }
    if st
        .last_spoken
        .is_some_and(|t| now.duration_since(t).as_secs_f64() < ATIS_REPEAT_SECS)
    {
        return None;
    }
    // Runway from the wind — but in calm keep the edition's pick, or the
    // coin-toss flip would cut a fresh edition per broadcast.
    let runway = if wx.wind_kts < 5.0 {
        st.edition.as_ref().map(|e| e.runway.clone())
    } else {
        None
    }
    .or_else(|| {
        nav::best_runway_for_wind(&st.runways, wx.wind_from_deg, wx.wind_kts).map(|(id, _, _)| id)
    })
    .unwrap_or_default();
    let snap = atis::Snapshot {
        airport: st.airport.clone(),
        zulu_sec: wx.zulu_sec,
        runway,
        wind_from_deg: wx.wind_from_deg,
        wind_kts: wx.wind_kts,
        visibility_sm: wx.visibility_sm,
        qnh_hpa: wx.qnh_hpa,
        oat_c: wx.oat_c,
    };
    match &st.edition {
        None => st.edition = Some(snap),
        Some(e) if atis::new_edition(e, &snap) => {
            st.letter = atis::next_letter(st.letter);
            st.edition = Some(snap);
        }
        // Same edition: time still moves in the report (a frozen clock reads
        // as a dead station), everything else stays the edition's numbers.
        Some(_) => {
            if let Some(e) = st.edition.as_mut() {
                e.zulu_sec = wx.zulu_sec;
            }
        }
    }
    st.last_spoken = Some(now);
    Some(atis::report(st.edition.as_ref()?, st.letter))
}

/// The information letter currently on the broadcast — `None` until the first
/// edition is cut (a pilot can't "have information" that never aired).
pub fn atis_current_letter() -> Option<char> {
    ATIS.lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .filter(|a| a.edition.is_some())
        .map(|a| a.letter)
}

/// The enroute region: every towered field from apt.dat, indexed once on a
/// background thread at startup, so the flight can outgrow its departure
/// airport. When another tower becomes decisively nearer, the current
/// controller hands the flight off by name and frequency, and the station
/// rebuilds itself around the new field — Gibraltar stops following you to
/// Malaga.
pub struct RegionApt {
    pub icao: String,
    pub apt: nav::AirportInfo,
    lat: f64,
    lon: f64,
}

static REGION: Mutex<Vec<RegionApt>> = Mutex::new(Vec::new());

/// The active station's identity + position, for the switch decision. Set by
/// [`Station::from_airport`].
static ACTIVE_FIELD: Mutex<Option<(String, f64, f64)>> = Mutex::new(None);

static LAST_REGION_SWITCH: Mutex<Option<Instant>> = Mutex::new(None);

/// The new field must be this much nearer than the active one before a
/// handoff — hysteresis, so the midpoint between two fields can't flap.
const SWITCH_MARGIN_NM: f64 = 5.0;
/// No handoff to a field further than this — enroute over nowhere keeps the
/// last controller (a lightweight "radar" continuity).
const SWITCH_MAX_NM: f64 = 60.0;
/// Minimum quiet time between handoffs.
const SWITCH_COOLDOWN_SECS: f64 = 180.0;

/// Index every towered field once (a full apt.dat pass, seconds — run it on a
/// background thread). Without it [`region_tick`] never fires and the station
/// stays single-airport, exactly the pre-region behavior.
pub fn load_region(apt_path: &str) {
    let mut v: Vec<RegionApt> = Vec::new();
    let res = nav::scan_airports(apt_path, |icao, apt| {
        // Staffed towers only: a Gateway "unnamed entity" TWR row (an
        // ultralight strip's air-to-air frequency, mistyped) must not become
        // a handoff target or "the nearest station" (LEMY, 2026-07-10).
        if apt.staffed_tower().is_some() {
            if let Some((lat, lon)) = apt.position() {
                v.push(RegionApt { icao: icao.to_string(), apt, lat, lon });
            }
        }
    });
    match res {
        Ok(()) => {
            tracing::info!(towered_fields = v.len(), "enroute region indexed");
            *REGION.lock().unwrap_or_else(|e| e.into_inner()) = v;
        }
        Err(e) => tracing::warn!("region index failed ({e:#}); staying single-airport"),
    }
}

/// A due station switch: what to say and who says it. Airborne it's a real
/// controller handoff; on the ground it's the CREW noting the local tower
/// (you didn't fly here under radar — you loaded the flight at a new field,
/// and "radar services terminated" from a parked aircraft would be theatre).
pub struct RegionSwitch {
    pub line: String,
    /// True = the copilot's voice (ground relocation); false = the controller.
    pub crew: bool,
}

/// You're AT a field (parked/taxiing) when it's within this — the ground
/// relocation case ("I started at LERL and Gibraltar answered", 2026-07-09).
const AT_FIELD_NM: f64 = 4.0;

/// One tick of the enroute watch: when another towered field is decisively
/// nearer than the active one, arm the station swap and return what to speak.
/// Airborne: the handoff ("radar services terminated, contact Malaga Tower on
/// ..."). On the ground: a silent relocation with a crew note, but only when
/// physically AT the new field. The main loop consumes [`PENDING_STATION`] on
/// its next pass; a cooldown keeps back-to-back fields from ping-ponging.
/// How much STATION a field is, for the enroute handoff choice: its distinct
/// published services beyond the tower itself. A primary airport scores 3-5;
/// a strip whose apt.dat rows just point at the primary's controllers scores
/// 0-1, so it never wins the sky it doesn't control.
fn region_services(apt: &nav::AirportInfo) -> usize {
    [
        nav::FREQ_ATIS,
        nav::FREQ_CLEARANCE,
        nav::FREQ_GROUND,
        nav::FREQ_APPROACH,
        nav::FREQ_DEPARTURE,
    ]
    .iter()
    .filter(|k| apt.freq(**k).is_some())
    .count()
}

/// Nautical miles of pull each published service adds to a field in the
/// airborne handoff choice (see [`region_services`]).
const SERVICE_BIAS_NM: f64 = 3.0;

pub fn region_tick(lat: f64, lon: f64, on_ground: bool, now: Instant) -> Option<RegionSwitch> {
    if PENDING_STATION
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .is_some()
    {
        return None; // a switch is already waiting to be consumed
    }
    if LAST_REGION_SWITCH
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .is_some_and(|t| now.duration_since(t).as_secs_f64() < SWITCH_COOLDOWN_SECS)
    {
        return None;
    }
    let (active_name, a_lat, a_lon) =
        ACTIVE_FIELD.lock().unwrap_or_else(|e| e.into_inner()).clone()?;
    let region = REGION.lock().unwrap_or_else(|e| e.into_inner());
    // Airborne, "nearest" is service-weighted: a real airport (ATIS, ground,
    // its own approach) outranks a strip a few miles nearer that merely
    // BORROWED the big field's frequencies — Trebujena's apt.dat rows name
    // "Jerez Tower"/"Seville Approach", and picking it by raw distance had
    // approach faithfully vectoring the Jerez arrival onto Trebujena's 800 m
    // grass runway two six (flown 2026-07-11). On the ground, raw nearest:
    // a relocation goes to the field you are physically at, however small.
    let score = |d: f64, apt: &nav::AirportInfo| {
        if on_ground {
            d
        } else {
            d - SERVICE_BIAS_NM * region_services(apt) as f64
        }
    };
    let (d_best, s_best, best) = region
        .iter()
        .map(|a| {
            let d = crate::nav::haversine_nm(lat, lon, a.lat, a.lon);
            (d, score(d, &a.apt), a)
        })
        .min_by(|x, y| x.1.total_cmp(&y.1))?;
    if best.apt.name == active_name {
        return None;
    }
    // Ground: only a relocation TO a field you're actually at. Airborne: the
    // enroute handoff, bounded so cruising over nowhere keeps the last
    // controller.
    if (on_ground && d_best > AT_FIELD_NM) || (!on_ground && d_best > SWITCH_MAX_NM) {
        return None;
    }
    // Hysteresis on the same scale the pick was made on.
    let s_active = region
        .iter()
        .find(|a| a.apt.name == active_name)
        .map(|a| score(crate::nav::haversine_nm(lat, lon, a.lat, a.lon), &a.apt))
        .unwrap_or_else(|| crate::nav::haversine_nm(lat, lon, a_lat, a_lon));
    if s_best + SWITCH_MARGIN_NM > s_active {
        return None;
    }
    let khz = best.apt.freq(nav::FREQ_TOWER)?.khz;
    // Whether ANY controller of the active station has actually worked this
    // flight in this session: two-way contact publishes a clearance state,
    // a radar service arms the squawk watch. Without either there is no
    // service to hand off — a fresh mid-flight boot must not open with a
    // controller terminating a service it never provided (heard live
    // 2026-07-11: app start near Sevilla answered by "radar services
    // terminated" from a departure position never called).
    let had_contact = CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()).is_some()
        || SQUAWK_WATCH.lock().unwrap_or_else(|e| e.into_inner()).is_some();
    let sw = if on_ground {
        RegionSwitch {
            line: format!(
                "We're at {} — tower here is on {}.",
                spoken_airport(&best.apt.name),
                atis::spoken_mhz(khz)
            ),
            crew: true,
        }
    } else if !had_contact {
        // First fix of the session: the copilot orients, no radar theatre.
        RegionSwitch {
            line: format!(
                "Closest field is {} — tower is on {}.",
                spoken_airport(&best.apt.name),
                atis::spoken_mhz(khz)
            ),
            crew: true,
        }
    } else {
        // A discrete code in effect goes back to conspicuity with the service
        // that assigned it.
        let squawk = if SQUAWK_WATCH
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .is_some()
        {
            " Squawk seven thousand."
        } else {
            ""
        };
        RegionSwitch {
            line: format!(
                "Radar services terminated.{squawk} Contact {} Tower on {}.",
                spoken_airport(&best.apt.name),
                atis::spoken_mhz(khz)
            ),
            crew: false,
        }
    };
    tracing::info!(from = %active_name, to = %best.apt.name, ground = on_ground, "region switch");
    *PENDING_STATION.lock().unwrap_or_else(|e| e.into_inner()) =
        Some((best.icao.clone(), best.apt.clone()));
    *LAST_REGION_SWITCH.lock().unwrap_or_else(|e| e.into_inner()) = Some(now);
    Some(sw)
}

/// The active station's controller frequencies (kHz → speaking position),
/// published by [`Station::install`] for the monitor's traffic advisories.
static CONTROLLER_FREQS: Mutex<Vec<(i64, StationSpeaker)>> = Mutex::new(Vec::new());

/// The active station's panel identity — name, field elevation, published
/// services — for the radio row and the ATC tab. Published by
/// [`Station::install`], like the rest of the side state.
static ACTIVE_PANEL: Mutex<Option<(String, f64, Vec<(String, i64)>)>> = Mutex::new(None);

/// A ground station's nominal antenna height. VHF range is line of sight, so
/// even on the ramp the mast buys a few miles.
const ANTENNA_FT: f64 = 50.0;

/// VHF radio range: the line-of-sight horizon for a receiver `h` feet above
/// the station's elevation, plus the station's own mast. Terrain masking is
/// ignored — this drives what the panel *shows*, a planning aid, not a
/// propagation model.
pub fn vhf_range_nm(height_above_field_ft: f64) -> f64 {
    1.23 * (height_above_field_ft.max(0.0).sqrt() + ANTENNA_FT.sqrt())
}

/// One field inside VHF range, for the panel's ATC tab and the crew's
/// "nearest station" note.
pub struct RangedField {
    pub name: String,
    pub dist_nm: f64,
    /// This is the active station — the one that answers radio calls.
    pub active: bool,
    /// Published services in briefing order, from [`services`].
    pub services: Vec<(String, i64)>,
}

/// Every towered field within VHF line of sight of the aircraft, nearest
/// first, capped at `limit`. Reads the region index; with the region off (or
/// not indexed yet) it falls back to the active station alone, so the list
/// never knows less than the station does.
pub fn fields_in_range(lat: f64, lon: f64, alt_msl_ft: f64, limit: usize) -> Vec<RangedField> {
    let active_name = ACTIVE_FIELD
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|(name, _, _)| name.clone());
    let region = REGION.lock().unwrap_or_else(|e| e.into_inner());
    let mut v: Vec<RangedField> = region
        .iter()
        .filter_map(|a| {
            let d = crate::nav::haversine_nm(lat, lon, a.lat, a.lon);
            (d <= vhf_range_nm(alt_msl_ft - a.apt.elev_ft)).then(|| RangedField {
                name: a.apt.name.clone(),
                dist_nm: d,
                active: active_name.as_deref() == Some(a.apt.name.as_str()),
                services: services(&a.apt),
            })
        })
        .collect();
    drop(region);
    if v.is_empty() {
        if let Some((name, svcs)) = active_station_in_range(lat, lon, alt_msl_ft) {
            let (_, alat, alon) = ACTIVE_FIELD
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .clone()
                .unwrap_or_default();
            v.push(RangedField {
                name,
                dist_nm: crate::nav::haversine_nm(lat, lon, alat, alon),
                active: true,
                services: svcs,
            });
        }
    }
    v.sort_by(|x, y| x.dist_nm.total_cmp(&y.dist_nm));
    v.truncate(limit);
    v
}

/// You call it, you get it: when a keyed transmission goes out on a frequency
/// the active station does NOT cover, but which belongs to another in-range
/// field, arm the station swap so THAT field answers the call about to be
/// routed. The ATC tab invites the pilot to tune any in-range station;
/// without this, calling Badajoz on its real tower frequency was answered by
/// dead air and a crew note about Gibraltar (2026-07-10). Unicom is excluded
/// (a shared party line, not a facility), and the nearest match wins when
/// fields share a frequency. Stamps the region cooldown so the automatic
/// handoff can't immediately yank the station away from an explicit tune.
pub fn arm_station_for_freq(com1_khz: i64, lat: f64, lon: f64, alt_msl_ft: f64, now: Instant) {
    if PENDING_STATION
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .is_some()
    {
        return; // a switch is already waiting to be consumed
    }
    let active_name = ACTIVE_FIELD
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|(name, _, _)| name.clone());
    let region = REGION.lock().unwrap_or_else(|e| e.into_inner());
    let hit = region
        .iter()
        .filter(|a| active_name.as_deref() != Some(a.apt.name.as_str()))
        .filter(|a| {
            services(&a.apt)
                .iter()
                .any(|(word, khz)| word != "unicom" && (khz - com1_khz).abs() <= 5)
        })
        .map(|a| (crate::nav::haversine_nm(lat, lon, a.lat, a.lon), a))
        .filter(|(d, a)| *d <= vhf_range_nm(alt_msl_ft - a.apt.elev_ft))
        .min_by(|x, y| x.0.total_cmp(&y.0));
    if let Some((d, a)) = hit {
        tracing::info!(
            to = %a.apt.name,
            khz = com1_khz,
            nm = format!("{d:.1}"),
            "station swap: transmission on another in-range field's frequency"
        );
        *PENDING_STATION.lock().unwrap_or_else(|e| e.into_inner()) =
            Some((a.icao.clone(), a.apt.clone()));
        *LAST_REGION_SWITCH.lock().unwrap_or_else(|e| e.into_inner()) = Some(now);
    }
}

/// The radio row's content: the ACTIVE station's name and services — but only
/// while it's inside VHF range. A tower you can't physically raise isn't a
/// radio row; parked in Extremadura the panel showed Gibraltar's comm plan as
/// if it could answer (2026-07-10). `None` = no station, or out of range.
pub fn active_station_in_range(lat: f64, lon: f64, alt_msl_ft: f64) -> Option<(String, Vec<(String, i64)>)> {
    let (_, alat, alon) = ACTIVE_FIELD
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()?;
    let (name, elev_ft, svcs) = ACTIVE_PANEL
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()?;
    let d = crate::nav::haversine_nm(lat, lon, alat, alon);
    (d <= vhf_range_nm(alt_msl_ft - elev_ft)).then_some((name, svcs))
}

/// Range to the ACTIVE station's field, for the radio's signal-quality model
/// (`radiofx::signal_quality` in the crew crate) — published to the speech arbiter by
/// the monitor each tick. `None` = no active station installed; ground
/// distance, not slant (the altitude term is noise at the ranges where
/// quality changes).
pub fn active_station_range_nm(lat: f64, lon: f64) -> Option<f64> {
    let (_, alat, alon) = ACTIVE_FIELD
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()?;
    Some(crate::nav::haversine_nm(lat, lon, alat, alon))
}

/// The active field's runway geometry, published by [`Station::install`] for
/// the runway-incursion watch.
static ACTIVE_RUNWAYS: Mutex<Vec<nav::Runway>> = Mutex::new(Vec::new());

/// Lateral capture around a runway centerline when the 100-row carried no
/// width: 40 m covers everything up to a wide-body runway (60 m ÷ 2 + slack)
/// without capturing a parallel taxiway.
const RUNWAY_HALF_WIDTH_M: f64 = 40.0;
/// Slack past the real pavement edge: edge lines, a mainwheel over the
/// threshold stripe — not the holding point. Jerez's E1 sits 40 m off the
/// 45 m runway's centerline; the blanket 40 m capture accused a pilot
/// holding SHORT of runway 20 of being ON it (2026-07-11).
const RUNWAY_EDGE_SLACK_M: f64 = 10.0;

/// Is this position ON one of the active field's runways? Point-to-segment in
/// a local flat-earth projection (exact enough over a 3 km runway). Returns
/// the spoken ident of the nearer end, for the blackbox and the call.
pub fn on_active_runway(lat: f64, lon: f64) -> Option<String> {
    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
    runways.iter().find_map(|r| on_runway(r, lat, lon))
}

/// Is this position ON one strip? The shared segment test behind
/// [`on_active_runway`] and the per-runway traffic watches. Returns the
/// spoken-able ident of the nearer end.
fn on_runway(r: &nav::Runway, lat: f64, lon: f64) -> Option<String> {
    let m_per_deg_lat = 111_320.0;
    let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
    let (px, py) = ((lon - r.lon_a) * m_per_deg_lon, (lat - r.lat_a) * m_per_deg_lat);
    let (bx, by) = ((r.lon_b - r.lon_a) * m_per_deg_lon, (r.lat_b - r.lat_a) * m_per_deg_lat);
    let len2 = bx * bx + by * by;
    if len2 < 1.0 {
        return None; // degenerate row
    }
    let t = (px * bx + py * by) / len2;
    let cross_m = (px * by - py * bx).abs() / len2.sqrt();
    let half_width = if r.width_m > 1.0 {
        r.width_m / 2.0 + RUNWAY_EDGE_SLACK_M
    } else {
        RUNWAY_HALF_WIDTH_M
    };
    // Slightly past the thresholds still counts (displaced/overrun).
    if (-0.02..=1.02).contains(&t) && cross_m <= half_width {
        let ident = if t < 0.5 { &r.ident_a } else { &r.ident_b };
        return Some(ident.clone());
    }
    None
}

/// The strip a runway END belongs to: landing 27L and departing 09R share
/// the same pavement, so conflict checks compare STRIPS, not end names.
fn strip<'a>(runways: &'a [nav::Runway], ident: &str) -> Option<&'a nav::Runway> {
    runways
        .iter()
        .find(|r| r.ident_a == ident || r.ident_b == ident)
}

/// The arrivals in one strip's final corridor: nearest distance, its spoken
/// callsign (names the condition of a conditional line-up), and how many are
/// in the stream (a conditional is only legal against exactly one).
pub struct FinalTraffic {
    pub nm: f64,
    pub callsign: Option<String>,
    pub count: usize,
}

/// Live traffic on final for THIS strip: airborne, low over the field,
/// inside the runway's extended-centerline corridor (either direction), and
/// not climbing away — a departure climbing through the corridor is leaving,
/// not arriving. Distances are to the nearer threshold, in nm. The
/// runway-blind version of this check held a Heathrow 09R departure for the
/// endless 09L arrivals — indefinitely (2026-07-10); a parallel runway's
/// stream is not your traffic.
pub fn final_traffic_for(
    targets: &[TcasTarget],
    rwy: &nav::Runway,
    field_elev_ft: f64,
) -> Option<FinalTraffic> {
    /// Lateral capture around the extended centerline: half of the tightest
    /// real parallel spacing (Heathrow's pair is ~1,400 m apart) with room
    /// for a wobbly final.
    const CORRIDOR_HALF_M: f64 = 500.0;
    /// How far out an arrival still owns the runway.
    const FINAL_NM: f64 = 4.0;
    /// A target climbing faster than this is a DEPARTURE leaving, not an
    /// arrival: a real climb-out shows +500 fpm or more, an arrival on final
    /// descends, and a level-off or an unfed VS reads ~0 and stays counted
    /// (fail toward caution). A ghost lifting off Malaga runway 30 read as
    /// "traffic on short final" to the runway 12 incursion watch, 2026-07-12.
    const CLIMBOUT_FPM: f64 = 300.0;
    let m_per_deg_lat = 111_320.0;
    let m_per_deg_lon = 111_320.0 * rwy.lat_a.to_radians().cos();
    let (bx, by) = ((rwy.lon_b - rwy.lon_a) * m_per_deg_lon, (rwy.lat_b - rwy.lat_a) * m_per_deg_lat);
    let len2 = bx * bx + by * by;
    if len2 < 1.0 {
        return None;
    }
    let len_m = len2.sqrt();
    let mut hits: Vec<(f64, &TcasTarget)> = targets
        .iter()
        .filter(|t| !t.on_ground)
        .filter(|t| (30.0..1500.0).contains(&(t.ele_ft - field_elev_ft)))
        .filter(|t| t.vs_fpm <= CLIMBOUT_FPM)
        .filter_map(|t| {
            let (px, py) = ((t.lon - rwy.lon_a) * m_per_deg_lon, (t.lat - rwy.lat_a) * m_per_deg_lat);
            let along = (px * bx + py * by) / len2;
            let cross_m = (px * by - py * bx).abs() / len_m;
            let beyond = FINAL_NM * 1852.0 / len_m;
            if cross_m > CORRIDOR_HALF_M || along < -beyond || along > 1.0 + beyond {
                return None;
            }
            let da = (px * px + py * py).sqrt();
            let (qx, qy) = (px - bx, py - by);
            let db = (qx * qx + qy * qy).sqrt();
            Some((da.min(db) / 1852.0, t))
        })
        .collect();
    hits.sort_by(|x, y| x.0.total_cmp(&y.0));
    let (nm, nearest) = hits.first()?;
    Some(FinalTraffic {
        nm: *nm,
        callsign: (!nearest.callsign.is_empty()).then(|| spoken_ghost(&nearest.callsign)),
        count: hits.len(),
    })
}

/// The nearest airborne traffic on final for the strip we're standing ON:
/// the runway-incursion watch's traffic check, in nm to the threshold.
/// Strip-aware on purpose — the earlier version ranged EVERY low TCAS target
/// inside 3 nm, and a cleared Malaga runway 12 departure got "exit the
/// runway, expedite" for the parallel runway 31 arrival stream (2026-07-12).
/// A parallel runway's stream is not your traffic — the same Heathrow lesson
/// `final_traffic_for` already encodes for the release watch.
pub async fn inbound_traffic_for_strip(xp: SimView<'_>, ident: &str) -> Option<f64> {
    let targets = read_tcas_targets(xp).await;
    inbound_final_for_strip(&targets, ident)
}

/// The sync core of [`inbound_traffic_for_strip`], testable without a sim.
fn inbound_final_for_strip(targets: &[TcasTarget], ident: &str) -> Option<f64> {
    let rwy = {
        let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
        strip(&runways, ident)?.clone()
    };
    let elev_ft = ACTIVE_PANEL.lock().unwrap_or_else(|e| e.into_inner()).as_ref()?.1;
    final_traffic_for(targets, &rwy, elev_ft).map(|t| t.nm)
}

// --- Live-traffic watches (party line, go-around, held clearance) -----------
//
// LiveTraffic aircraft are replays of real flights — nothing can command
// them. The design stance (2026-07-10): the tower controls the ONE aircraft
// with a pilot in it and tells the truth about everything else. The party
// line NARRATES the clearance a ghost is about to comply with; the go-around
// order protects the player when a ghost takes the runway; the held takeoff
// clearance releases itself when the final actually clears.

/// One live TCAS target with an identity and an absolute position.
#[derive(Clone)]
pub struct TcasTarget {
    pub callsign: String,
    pub lat: f64,
    pub lon: f64,
    pub ele_ft: f64,
    pub on_ground: bool,
    /// Vertical speed, ft/min; 0.0 when the feed doesn't populate the array
    /// (reads as level, which every watch treats conservatively).
    pub vs_fpm: f64,
}

/// Read the TCAS target table (slot 0 = own ship, skipped). Targets without
/// a flight id are dropped — a tower can't address an aircraft it can't
/// name. Any missing array (older sim, no traffic feed) = empty.
pub async fn read_tcas_targets(xp: SimView<'_>) -> Vec<TcasTarget> {
    let (Ok(lat), Ok(lon), Ok(ele), Ok(wow), Ok(ids)) = (
        xp.get_float_array(datarefs::TCAS_LAT).await,
        xp.get_float_array(datarefs::TCAS_LON).await,
        xp.get_float_array(datarefs::TCAS_ELE_M).await,
        xp.get_float_array(datarefs::TCAS_WOW).await,
        xp.get_bytes(datarefs::TCAS_FLIGHT_ID).await,
    ) else {
        return Vec::new();
    };
    let ids: Vec<String> = ids
        .chunks(8)
        .map(|c| String::from_utf8_lossy(c).trim_matches('\0').trim().to_string())
        .collect();
    // Vertical speed is OPTIONAL — an older sim or a feed that skips the
    // array must not blank the whole traffic picture, so a failed read pads
    // with level flight rather than joining the all-or-nothing gate above.
    let vs = xp.get_float_array(datarefs::TCAS_VS_FPM).await.unwrap_or_default();
    lat.iter()
        .zip(&lon)
        .zip(&ele)
        .zip(&wow)
        .zip(&ids)
        .zip(vs.into_iter().chain(std::iter::repeat(0.0)))
        .skip(1) // slot 0 is own ship
        .filter(|(((((la, lo), _), _), id), _)| !id.is_empty() && (**la != 0.0 || **lo != 0.0))
        .map(|(((((la, lo), el), ww), id), vs)| TcasTarget {
            callsign: id.clone(),
            lat: *la,
            lon: *lo,
            ele_ft: *el * 3.28084,
            on_ground: *ww > 0.5,
            vs_fpm: vs,
        })
        .collect()
}

/// Airline telephony for the party line — the common carriers over Iberia;
/// anything else is spoken NATO-style like a tail number.
const TELEPHONY: &[(&str, &str)] = &[
    ("IBE", "Iberia"),
    ("BAW", "Speedbird"),
    ("RYR", "Ryanair"),
    ("EZY", "Easy"),
    ("VLG", "Vueling"),
    ("AEA", "Air Europa"),
    ("ANE", "Air Nostrum"),
    ("SWT", "Swift"),
    ("DLH", "Lufthansa"),
    ("AFR", "Air France"),
    ("KLM", "KLM"),
    ("TAP", "Air Portugal"),
    ("EIN", "Shamrock"),
    ("WZZ", "Wizz Air"),
    ("SWR", "Swiss"),
    ("UAE", "Emirates"),
    ("QTR", "Qatari"),
    ("AAL", "American"),
    ("UAL", "United"),
    ("DAL", "Delta"),
    ("VIR", "Virgin"),
    ("EXS", "Channex"),
    ("EWG", "Eurowings"),
    ("MEA", "Cedar Jet"),
    ("BEL", "Bee-line"),
    ("AUA", "Austrian"),
    ("SAS", "Scandinavian"),
    ("THY", "Turkish"),
    ("FIN", "Finnair"),
    ("ICE", "Iceair"),
    ("ACA", "Air Canada"),
    ("CPA", "Cathay"),
    ("SIA", "Singapore"),
    ("ANA", "All Nippon"),
    ("ETD", "Etihad"),
    ("ELY", "El Al"),
    ("QFA", "Qantas"),
    ("ANZ", "New Zealand"),
    ("JBU", "JetBlue"),
    ("CFG", "Condor"),
    ("TRA", "Transavia"),
    ("NAX", "Nor Shuttle"),
    ("TOM", "Tomjet"),
    ("LOG", "Logan"),
    ("AEE", "Aegean"),
    ("LOT", "LOT"),
    ("AIC", "Air India"),
    ("PIA", "Pakistan"),
    ("WJA", "WestJet"),
    ("PGT", "Sunturk"),
];

/// IATA two-letter prefixes — LiveTraffic's flight ids arrive this way
/// ("BA306", measured live 2026-07-10), so both spellings get telephony.
const TELEPHONY_IATA: &[(&str, &str)] = &[
    ("BA", "Speedbird"),
    ("IB", "Iberia"),
    ("FR", "Ryanair"),
    ("U2", "Easy"),
    ("VY", "Vueling"),
    ("UX", "Air Europa"),
    ("LH", "Lufthansa"),
    ("AF", "Air France"),
    ("KL", "KLM"),
    ("TP", "Air Portugal"),
    ("EI", "Shamrock"),
    ("W6", "Wizz Air"),
    ("LX", "Swiss"),
    ("EK", "Emirates"),
    ("QR", "Qatari"),
    ("AA", "American"),
    ("UA", "United"),
    ("DL", "Delta"),
    ("TG", "Thai"),
    ("VS", "Virgin"),
    ("LS", "Channex"),
    ("EW", "Eurowings"),
    ("ME", "Cedar Jet"),
    ("SN", "Bee-line"),
    ("OS", "Austrian"),
    ("SK", "Scandinavian"),
    ("TK", "Turkish"),
    ("AY", "Finnair"),
    ("FI", "Iceair"),
    ("AC", "Air Canada"),
    ("CX", "Cathay"),
    ("SQ", "Singapore"),
    ("NH", "All Nippon"),
    ("EY", "Etihad"),
    ("LY", "El Al"),
    ("QF", "Qantas"),
    ("NZ", "New Zealand"),
    ("B6", "JetBlue"),
    ("DE", "Condor"),
    ("HV", "Transavia"),
    ("DY", "Nor Shuttle"),
    ("BY", "Tomjet"),
    ("LM", "Logan"),
    ("A3", "Aegean"),
    ("LO", "LOT"),
    ("AI", "Air India"),
    ("PK", "Pakistan"),
    ("WS", "WestJet"),
    ("PC", "Sunturk"),
];

/// A ghost's flight id spoken like a controller would: airline telephony +
/// digits for "IBE3421" / "BA306", full NATO for a tail number.
pub fn spoken_ghost(id: &str) -> String {
    let id = id.trim();
    let spoken_rest = |rest: &str| {
        (!rest.is_empty() && rest.chars().all(|c| c.is_ascii_alphanumeric()))
            .then(|| tower::spoken_callsign(rest))
    };
    // ICAO 3-letter prefix first ("IBE3421"), then IATA 2-letter where the
    // third character is a digit ("BA306" — but never "BAC" of a tail).
    if id.len() > 3 {
        let (prefix, rest) = id.split_at(3);
        if let Some((_, telephony)) = TELEPHONY.iter().find(|(p, _)| *p == prefix) {
            if let Some(rest) = spoken_rest(rest) {
                return format!("{telephony} {rest}");
            }
        }
    }
    if id.len() > 2 && id.as_bytes()[2].is_ascii_digit() {
        let (prefix, rest) = id.split_at(2);
        if let Some((_, telephony)) = TELEPHONY_IATA.iter().find(|(p, _)| *p == prefix) {
            if let Some(rest) = spoken_rest(rest) {
                return format!("{telephony} {rest}");
            }
        }
    }
    tower::spoken_callsign(id)
}

/// The runway end a DEPARTING ghost rolled from: nearest strip, end chosen
/// by the roll direction (bearing from last tick's position to this one).
/// `None` when the ghost hasn't moved enough for a bearing — the caller
/// falls back to the nearest end.
fn departure_runway_ident(lat: f64, lon: f64, prev_lat: f64, prev_lon: f64) -> Option<String> {
    let m_per_deg_lat = 111_320.0;
    let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
    let (dx, dy) = ((lon - prev_lon) * m_per_deg_lon, (lat - prev_lat) * m_per_deg_lat);
    if (dx * dx + dy * dy).sqrt() < 10.0 {
        return None; // no meaningful track over one tick
    }
    let track = dx.atan2(dy).to_degrees().rem_euclid(360.0);
    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
    let r = runways.iter().min_by(|a, b| {
        let da = crate::nav::haversine_nm(lat, lon, a.lat_a, a.lon_a)
            .min(crate::nav::haversine_nm(lat, lon, a.lat_b, a.lon_b));
        let db = crate::nav::haversine_nm(lat, lon, b.lat_a, b.lon_a)
            .min(crate::nav::haversine_nm(lat, lon, b.lat_b, b.lon_b));
        da.total_cmp(&db)
    })?;
    let da = angle_diff_deg(track, r.heading_a_deg()).abs();
    let db = angle_diff_deg(track, r.heading_b_deg()).abs();
    Some(if da <= db { r.ident_a.clone() } else { r.ident_b.clone() })
}

/// The active field's runway end nearest a position — the runway a landing
/// ghost is aiming at, for the narrated clearance's flavor.
fn nearest_runway_ident(lat: f64, lon: f64) -> Option<String> {
    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
    runways
        .iter()
        .flat_map(|r| {
            [
                (r.ident_a.clone(), r.lat_a, r.lon_a),
                (r.ident_b.clone(), r.lat_b, r.lon_b),
            ]
        })
        .map(|(id, la, lo)| (crate::nav::haversine_nm(lat, lon, la, lo), id))
        .min_by(|x, y| x.0.total_cmp(&y.0))
        .map(|(_, id)| id)
}

/// Is this the pilot asking the CREW what to say on the radio? ("what do I
/// say", "how do I call the tower", "what's the call") — the copilot as
/// radio coach. Checked before the LLM; the answer comes from
/// [`suggest_call`], never improvised.
pub fn coach_query(t: &str) -> bool {
    t.contains("what do i say")
        || t.contains("what should i say")
        || t.contains("what do i tell")
        || t.contains("what should i tell")
        || t.contains("what's the call")
        || t.contains("what is the call")
        || t.contains("phraseology")
        || (t.contains("how do i")
            && (t.contains("call") || t.contains("ask") || t.contains("request")
                || t.contains("talk")))
        || (t.contains("help") && t.contains("radio"))
}

/// One coached hint from [`suggest_call`]: an optional prerequisite/status
/// line and the transmission itself. At least one is always present.
pub struct Suggestion {
    /// Spoken/shown BEFORE the call — a prerequisite ("First tune tower, one
    /// three one decimal two.", "First copy the ATIS on ...") or a status
    /// ("Under radar vectors. No call owed."). Never itself a transmission.
    pub first: Option<String>,
    /// The transmission to key — always a line the intent parser routes.
    pub call: Option<String>,
    /// The call is an OWED readback (runway-critical): say it now.
    pub urgent: bool,
    /// The flight is on the arrival side — drives the panel's phrase-group
    /// highlight (false = departure side).
    pub arrival: bool,
}

/// The call the pilot SHOULD make next, derived from the tower's own state
/// machine — the copilot as radio coach, and the panel's Calls tab. The
/// suggestions are deliberately the parser's happy path: following the
/// coach always routes. `com1_khz` is the live radio: a suggestion the
/// station can't hear gets a "first tune ..." line ahead of it. `None` =
/// no station to talk to.
pub fn suggest_call(
    callsign: &str,
    on_ground: bool,
    dist_nm: Option<f64>,
    com1_khz: Option<i64>,
) -> Option<Suggestion> {
    let (station, services) = {
        let p = ACTIVE_PANEL.lock().unwrap_or_else(|e| e.into_inner());
        let (name, _, svcs) = p.as_ref()?;
        (spoken_airport(name).to_string(), svcs.clone())
    };
    let clr = CLEARANCE
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|c| (c.state, c.runway_ident.clone(), c.pending_readback.clone()));
    let state = clr.as_ref().map(|(s, _, _)| *s);
    let arrival = !on_ground
        || matches!(state, Some(tower::State::Inbound | tower::State::ClearedToLand));
    // A readback owed outranks everything: it IS the next call — and it was
    // just received, so the radio is by definition on the right frequency.
    if let Some(rb) = clr.as_ref().and_then(|(_, _, rb)| rb.clone()) {
        return Some(Suggestion { first: None, call: Some(rb), urgent: true, arrival });
    }
    // "First tune <service> ..." when the suggested call goes to a
    // frequency COM1 isn't on — a perfect call the station can't hear is
    // the coach's own failure mode. Skipped when another `first` already
    // leads (ATIS, vectors status): one prerequisite at a time.
    let tune_first = |svc: &str| -> Option<String> {
        let com1 = com1_khz?;
        let (_, khz) = services.iter().find(|(w, _)| w == svc)?;
        ((com1 - khz).abs() > 5).then(|| format!("First tune {svc}, {}.", atis::spoken_mhz(*khz)))
    };
    // Approach's side of the picture, airborne only (a plan on the ground is
    // stale — vector_tick clears it on touchdown).
    if !on_ground {
        // Handed to the tower with an APPROACH clearance: the "final" report
        // is owed before any landing clearance — the same fact the monitor's
        // voice coach reads.
        if let Some(rwy) = LANDING_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()).clone() {
            return Some(Suggestion {
                first: tune_first("tower"),
                call: Some(format!("{station} Tower, {callsign}, final runway {rwy}.")),
                urgent: false,
                arrival: true,
            });
        }
        if let Some((ils_cleared, rwy)) = VECTORS
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .map(|p| (p.ils && p.cleared, p.runway_spoken.clone()))
        {
            // Cleared for the ILS on the intercept: the one call approach
            // waits on is "established". Before that, radar vectors are a
            // listening game — coach the silence, not an invented call.
            return Some(if ils_cleared {
                Suggestion {
                    first: Some("Report when established on the localizer.".to_string()),
                    call: Some(format!("{callsign}, established, runway {rwy}.")),
                    urgent: false,
                    arrival: true,
                }
            } else {
                Suggestion {
                    first: Some(
                        "Under radar vectors. No call owed; follow the headings.".to_string(),
                    ),
                    call: None,
                    urgent: false,
                    arrival: true,
                }
            });
        }
    }
    let info = atis_current_letter()
        .map(|l| format!(", with information {}", atis::nato(l)))
        .unwrap_or_default();
    // An ATIS that has never cut an edition is an ATIS nobody has listened
    // to (editions cut while COM1 sits on its frequency) — before the first
    // call to a fresh station, the better first move is to go copy it.
    let atis_first = || {
        let a = ATIS.lock().unwrap_or_else(|e| e.into_inner());
        a.as_ref()
            .filter(|a| a.edition.is_none())
            .map(|a| format!("First copy the ATIS on {}.", atis::spoken_mhz(a.khz)))
    };
    let rwy = clr
        .as_ref()
        .map(|(_, id, _)| id.as_str())
        .filter(|s| !s.is_empty())
        .map(atis::spoken_runway);
    let pos = dist_nm
        .filter(|d| *d >= 2.0)
        .map(|d| format!("{:.0} miles out, ", d))
        .unwrap_or_default();
    let call = match (state, on_ground) {
        // Never talked to this station: the full initial call.
        (None, true) => {
            format!("{station} Tower, {callsign}{info}, request departure clearance.")
        }
        (None, false) | (Some(tower::State::Parked), false) => {
            format!("{station} Tower, {callsign}, {pos}inbound{info}, request join.")
        }
        (Some(tower::State::Parked), true) => format!("{callsign}, request taxi."),
        // Taxi clearance in hand: report ready at the REAL holding point —
        // the cleared route's destination, when the taxi network knows it.
        (Some(tower::State::TaxiCleared), true) => {
            let hp = TAXI_ROUTE
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .as_ref()
                .map(|r| spoken_taxiway(&r.holding_point));
            match (hp, &rwy) {
                (Some(h), Some(r)) => {
                    format!("{callsign}, holding point {h}, runway {r}, ready for departure.")
                }
                (Some(h), None) => format!("{callsign}, holding point {h}, ready for departure."),
                _ => format!("{callsign}, ready for departure."),
            }
        }
        // Lined up (readback already paid): the clearance comes to you —
        // a nudge routes as a fresh ask if you want it.
        (Some(tower::State::LinedUp), true) => {
            format!("{callsign}, ready for immediate departure.")
        }
        // A held/issued clearance wants its readback.
        (Some(tower::State::TakeoffCleared), true) => match &rwy {
            Some(r) => format!("Cleared for takeoff runway {r}, {callsign}."),
            None => format!("Cleared for takeoff, {callsign}."),
        },
        // Airborne after departure: leave, or work the circuit.
        (Some(tower::State::TakeoffCleared), false)
        | (Some(tower::State::TaxiCleared), false)
        | (Some(tower::State::LinedUp), false) => {
            format!("{callsign}, request frequency change. Or report: downwind.")
        }
        (Some(tower::State::Inbound), false) => match &rwy {
            Some(r) => format!("{callsign}, final runway {r}."),
            None => format!("{callsign}, final."),
        },
        (Some(tower::State::ClearedToLand), false) => match &rwy {
            Some(r) => format!("Cleared to land runway {r}, {callsign}."),
            None => format!("Cleared to land, {callsign}."),
        },
        // On the ground as an arrival: get off the runway and home.
        (Some(tower::State::ClearedToLand), true) | (Some(tower::State::Inbound), true) => {
            format!("{callsign}, runway vacated, request taxi to parking.")
        }
    };
    // One prerequisite at a time: the first-ever call prefers the ATIS (the
    // chronological next action — once in contact the tower's own QNH
    // covers you), everything else the frequency check.
    let first = match state {
        None => atis_first().or_else(|| tune_first("tower")),
        _ => tune_first("tower"),
    };
    Some(Suggestion { first, call: Some(call), urgent: false, arrival })
}

/// The active field for the traffic watches: spoken name, lat, lon, field
/// elevation ft. `None` = no station.
pub fn active_field() -> Option<(String, f64, f64, f64)> {
    let (name, lat, lon) = ACTIVE_FIELD
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()?;
    let elev = ACTIVE_PANEL
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|(_, e, _)| *e)
        .unwrap_or(0.0);
    Some((spoken_airport(&name).to_string(), lat, lon, elev))
}

/// What the party line remembers about a ghost between passes.
struct GhostMemory {
    last_seen: Instant,
    prev_ele_ft: f64,
    prev_ground: bool,
    /// Last tick's position — the roll direction at rotation, so a narrated
    /// takeoff names the runway END the ghost is departing FROM, not the
    /// threshold it happens to be nearest (liftoff is past midfield: every
    /// 09R departure read "two seven left" on the 2026-07-10 tape).
    prev_lat: f64,
    prev_lon: f64,
    announced_landing: bool,
    announced_takeoff: bool,
    /// "Continue approach" spoken (its aim runway was blocked by the player).
    announced_continue: bool,
}

static GHOSTS: Mutex<Option<std::collections::HashMap<String, GhostMemory>>> = Mutex::new(None);

/// One line of a party-line exchange: who says it, and what.
pub struct PartyLine {
    /// Spoken by the GHOST pilot (its own pooled voice); false = the tower.
    pub ghost: bool,
    pub text: String,
}

/// A narrated exchange — the ghost pilot's call, the tower's reply, the
/// readback — queued in speaking order through the one arbiter.
pub struct PartyExchange {
    pub lines: Vec<PartyLine>,
    /// Stable voice seed for this ghost (hash of its callsign): Speedbird
    /// 306 is the same person on every call.
    pub ghost_seed: usize,
}

/// One pass of the party line: narrate at most ONE exchange for the ghost
/// traffic — an arrival's final call answered with the landing clearance
/// while the ghost is still descending (spoken BEFORE reality lands it), a
/// takeoff clearance and readback as one rotates. Each ghost gets each
/// exchange once. The tower is never wrong: the "instruction" is derived
/// from what the replay is about to do anyway.
pub fn chatter_tick(
    targets: &[TcasTarget],
    station_name: &str,
    field_lat: f64,
    field_lon: f64,
    field_elev_ft: f64,
    // The runway END the PLAYER currently occupies (or holds a clearance
    // for): a ghost landing that strip gets "continue approach", never a
    // narrated "cleared to land" over the player's head — the 2026-07-10
    // tape cleared a takeoff and a landing on 09L eight seconds apart.
    blocked_ident: Option<&str>,
    now: Instant,
) -> Option<PartyExchange> {
    let mut guard = GHOSTS.lock().unwrap_or_else(|e| e.into_inner());
    let mem = guard.get_or_insert_with(std::collections::HashMap::new);
    mem.retain(|_, m| now.duration_since(m.last_seen).as_secs() < 180);
    let mut exchange = None;
    for t in targets {
        let dist = crate::nav::haversine_nm(t.lat, t.lon, field_lat, field_lon);
        if dist > 8.0 {
            continue; // beyond this tower's world
        }
        let m = mem.entry(t.callsign.clone()).or_insert(GhostMemory {
            last_seen: now,
            prev_ele_ft: t.ele_ft,
            prev_ground: t.on_ground,
            prev_lat: t.lat,
            prev_lon: t.lon,
            announced_landing: false,
            announced_takeoff: false,
            announced_continue: false,
        });
        let descending = t.ele_ft < m.prev_ele_ft - 1.0;
        let was_ground = m.prev_ground;
        let agl = t.ele_ft - field_elev_ft;
        if exchange.is_none()
            && !t.on_ground
            && !m.announced_landing
            && dist < 5.0
            && (150.0..1200.0).contains(&agl)
            && descending
        {
            let cs = spoken_ghost(&t.callsign);
            let aim = nearest_runway_ident(t.lat, t.lon);
            // The player is on (or cleared onto) this ghost's strip: no
            // narrated landing clearance over their head — "continue" is
            // what a real tower says, and it stays true when the replay
            // lands anyway. The clearance may still come once the player
            // vacates and the ghost is re-seen clear.
            let blocked = match (&aim, blocked_ident) {
                (Some(a), Some(b)) => {
                    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
                    matches!(
                        (strip(&runways, a), strip(&runways, b)),
                        (Some(x), Some(y)) if std::ptr::eq(x, y)
                    )
                }
                _ => false,
            };
            if blocked {
                if !m.announced_continue {
                    m.announced_continue = true;
                    exchange = Some(PartyExchange {
                        lines: vec![
                            PartyLine {
                                ghost: true,
                                text: format!(
                                    "{station_name} Tower, {cs}, final runway {}.",
                                    aim.as_deref().map(atis::spoken_runway).unwrap_or_default()
                                ),
                            },
                            PartyLine {
                                ghost: false,
                                text: format!(
                                    "{cs}, continue approach, traffic on the runway ahead."
                                ),
                            },
                        ],
                        ghost_seed: crate::speaker::station_voice_seed(&t.callsign),
                    });
                }
                m.last_seen = now;
                m.prev_ele_ft = t.ele_ft;
                m.prev_ground = t.on_ground;
                m.prev_lat = t.lat;
                m.prev_lon = t.lon;
                continue;
            }
            m.announced_landing = true;
            let rwy = aim.as_deref().map(atis::spoken_runway);
            let lines = match &rwy {
                Some(r) => vec![
                    PartyLine {
                        ghost: true,
                        text: format!("{station_name} Tower, {cs}, final runway {r}."),
                    },
                    PartyLine {
                        ghost: false,
                        text: format!("{cs}, runway {r}, cleared to land."),
                    },
                    PartyLine {
                        ghost: true,
                        text: format!("Cleared to land runway {r}, {cs}."),
                    },
                ],
                None => vec![
                    PartyLine {
                        ghost: true,
                        text: format!("{station_name} Tower, {cs}, final."),
                    },
                    PartyLine { ghost: false, text: format!("{cs}, cleared to land.") },
                ],
            };
            exchange = Some(PartyExchange {
                lines,
                ghost_seed: crate::speaker::station_voice_seed(&t.callsign),
            });
        }
        if exchange.is_none() && was_ground && !t.on_ground && !m.announced_takeoff && dist < 3.0
        {
            m.announced_takeoff = true;
            let cs = spoken_ghost(&t.callsign);
            // Name the end the ghost DEPARTED, by its roll direction —
            // liftoff is past midfield, so the nearest threshold is the
            // reciprocal (every 09R departure read "two seven left" on the
            // 2026-07-10 tape).
            let ident = departure_runway_ident(t.lat, t.lon, m.prev_lat, m.prev_lon)
                .or_else(|| nearest_runway_ident(t.lat, t.lon));
            let clearance = match ident {
                Some(id) => format!(
                    "{cs}, runway {}, cleared for takeoff.",
                    atis::spoken_runway(&id)
                ),
                None => format!("{cs}, cleared for takeoff."),
            };
            exchange = Some(PartyExchange {
                lines: vec![
                    PartyLine { ghost: false, text: clearance },
                    PartyLine { ghost: true, text: format!("Cleared for takeoff, {cs}.") },
                ],
                ghost_seed: crate::speaker::station_voice_seed(&t.callsign),
            });
        }
        m.last_seen = now;
        m.prev_ele_ft = t.ele_ft;
        m.prev_ground = t.on_ground;
        m.prev_lat = t.lat;
        m.prev_lon = t.lon;
    }
    exchange
}

/// The go-around order was last spoken at — once per half minute is urgency,
/// not stutter.
static GO_AROUND_LAST: Mutex<Option<Instant>> = Mutex::new(None);

/// The tower protects the one aircraft it controls: cleared to land, low,
/// and a ghost is ON the runway — the clearance is cancelled out loud
/// instead of letting the world contradict it. Crew phrasing when the pilot
/// isn't tuned (a safety net must not depend on the radio).
pub fn go_around_tick(
    targets: &[TcasTarget],
    own_agl_ft: f64,
    on_ground: bool,
    tuned: bool,
    now: Instant,
) -> Option<ComplianceCall> {
    if on_ground || own_agl_ft > 800.0 {
        return None;
    }
    let (callsign, runway_ident) = CLEARANCE
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .filter(|c| c.state == tower::State::ClearedToLand)
        .map(|c| (c.callsign.clone(), c.runway_ident.clone()))?;
    // Only a ghost on OUR strip cancels the clearance — a parallel runway's
    // occupant is not our conflict (Heathrow, 2026-07-10).
    let ours = {
        let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
        strip(&runways, &runway_ident).cloned()
    };
    targets.iter().find(|t| {
        t.on_ground
            && match &ours {
                Some(r) => on_runway(r, t.lat, t.lon).is_some(),
                None => on_active_runway(t.lat, t.lon).is_some(),
            }
    })?;
    let mut last = GO_AROUND_LAST.lock().unwrap_or_else(|e| e.into_inner());
    if last.is_some_and(|t| now.duration_since(t).as_secs_f64() < 30.0) {
        return None;
    }
    *last = Some(now);
    // The order voids the clearance in the FSM too, or the coach keeps
    // suggesting "cleared to land ..." as the readback owed while the tower
    // is shouting go around (seen live, 2026-07-14). The sync is handed to
    // the main loop, which owns the `Station`, same as the hold release.
    *PENDING_GO_AROUND.lock().unwrap_or_else(|e| e.into_inner()) = true;
    Some(if tuned {
        ComplianceCall {
            line: format!(
                "{callsign}, go around, traffic on the runway. Climb to circuit \
                 altitude, report downwind."
            ),
            crew: false,
        }
    } else {
        ComplianceCall {
            line: "Traffic on the runway ahead. Go around.".to_string(),
            crew: true,
        }
    })
}

/// A takeoff clearance withheld for landing traffic — armed by the reply
/// path, released by the monitor once the final has stayed clear.
pub struct TakeoffHold {
    callsign: String,
    /// Spoken, for the release line ("zero niner right").
    runway: String,
    /// The end ident ("09R"), for the strip-restricted traffic check.
    runway_ident: String,
    clear_since: Option<Instant>,
}

/// The final must stay clear this long before the held clearance releases —
/// an arrival crossing the fence vanishes from the airborne band while still
/// rolling out, and "cleared for takeoff" over a lander is a contradiction.
const HOLD_CLEAR_SECS: f64 = 15.0;

/// One pass of the held-clearance watch: with OUR runway's final clear (no
/// arrival in its corridor, no ghost on the strip — a parallel runway's
/// stream doesn't count) long enough, the tower comes back unprompted —
/// like a real one — and the FSM sync is handed to the main loop.
pub fn takeoff_hold_tick(
    targets: &[TcasTarget],
    field_elev_ft: f64,
    player_on_ground: bool,
    now: Instant,
) -> Option<String> {
    let mut guard = TAKEOFF_HOLD.lock().unwrap_or_else(|e| e.into_inner());
    let hold = guard.as_mut()?;
    // Airborne, the hold is moot: either the pilot departed through the
    // cancel (the compliance watch owns that conversation) or the whole
    // thing is stale. "Traffic clear, cleared for takeoff" spoken to an
    // aircraft two minutes into its climb (Gatwick, 2026-07-13) helps
    // nobody — void it, silently.
    if !player_on_ground {
        *guard = None;
        return None;
    }
    // Our strip, cloned out so the runway lock isn't held across the checks.
    let rwy = {
        let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
        strip(&runways, &hold.runway_ident).cloned()
    };
    let blocked = match &rwy {
        Some(r) => {
            final_traffic_for(targets, r, field_elev_ft).is_some()
                || targets
                    .iter()
                    .any(|t| t.on_ground && on_runway(r, t.lat, t.lon).is_some())
        }
        // No geometry for the held runway: fall back to the blunt any-runway
        // checks rather than clearing blind.
        None => {
            targets.iter().any(|t| {
                !t.on_ground && (30.0..1500.0).contains(&(t.ele_ft - field_elev_ft))
            }) || targets
                .iter()
                .any(|t| t.on_ground && on_active_runway(t.lat, t.lon).is_some())
        }
    // The wake clock blocks the release exactly like traffic does: a held
    // clearance behind a departed heavy waits out the two minutes.
    } || wake_remaining(&hold.runway_ident, now).is_some();
    if blocked {
        hold.clear_since = None;
        return None;
    }
    let since = *hold.clear_since.get_or_insert(now);
    if now.duration_since(since).as_secs_f64() < HOLD_CLEAR_SECS {
        return None;
    }
    let line = format!(
        "{}, traffic clear. Runway {}, cleared for takeoff.",
        hold.callsign, hold.runway
    );
    if let Some(c) = CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()).as_mut() {
        c.state = tower::State::TakeoffCleared;
    }
    *PENDING_TAKEOFF_CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()) = true;
    *guard = None;
    Some(line)
}

// --- Wake turbulence -------------------------------------------------------

/// The last airline-sized departure off a strip: wake separation runs from
/// its liftoff. TCAS carries no aircraft type, so "an airline callsign" is
/// the deterministic stand-in for "medium or heavier" — at the fields where
/// wake matters (Heathrow's replay stream) it's right essentially always,
/// and a GA ghost departing ahead of you correctly starts no clock.
pub struct Wake {
    pub since: Instant,
    /// The strip it departed from (end ident, e.g. "27R").
    pub runway_ident: String,
}

static WAKE: Mutex<Option<Wake>> = Mutex::new(None);

/// ICAO's two minutes for a light departing behind a medium/heavy from the
/// same position.
pub const WAKE_SECS: u64 = 120;

/// Ground/air memory per ghost for liftoff detection, independent of the
/// party line (which is config-gated and must not be a safety dependency).
static WAKE_PREV: Mutex<Option<std::collections::HashMap<String, (bool, Instant)>>> =
    Mutex::new(None);

/// Does this flight id carry an airline prefix (ICAO 3-letter or IATA
/// 2-letter + digit)? The wake heuristic's "medium or heavier".
fn is_airline(id: &str) -> bool {
    let id = id.trim();
    (id.len() > 3 && TELEPHONY.iter().any(|(p, _)| *p == &id[..3]))
        || (id.len() > 2
            && id.as_bytes()[2].is_ascii_digit()
            && TELEPHONY_IATA.iter().any(|(p, _)| *p == &id[..2]))
}

/// One pass of the wake watch: a ghost transitioning ground→air near the
/// field starts the two-minute clock on its strip. Called by the monitor on
/// the same TCAS read as the other traffic watches.
pub fn wake_note_targets(targets: &[TcasTarget], now: Instant) {
    let Some((_, flat, flon, _)) = active_field() else { return };
    let mut guard = WAKE_PREV.lock().unwrap_or_else(|e| e.into_inner());
    let mem = guard.get_or_insert_with(std::collections::HashMap::new);
    mem.retain(|_, (_, seen)| now.duration_since(*seen).as_secs() < 300);
    for t in targets {
        if crate::nav::haversine_nm(t.lat, t.lon, flat, flon) > 3.0 {
            continue;
        }
        let e = mem.entry(t.callsign.clone()).or_insert((t.on_ground, now));
        let was_ground = e.0;
        *e = (t.on_ground, now);
        if was_ground && !t.on_ground && is_airline(&t.callsign) {
            if let Some(ident) = nearest_runway_ident(t.lat, t.lon) {
                tracing::info!(ghost = %t.callsign, runway = %ident, "wake clock started");
                *WAKE.lock().unwrap_or_else(|e| e.into_inner()) =
                    Some(Wake { since: now, runway_ident: ident });
            }
        }
    }
}

/// Seconds of wake separation still to run behind the last departure from
/// THIS strip (end ident). `None` = clock expired, other strip, or no
/// departure seen.
pub fn wake_remaining(runway_ident: &str, now: Instant) -> Option<u64> {
    let guard = WAKE.lock().unwrap_or_else(|e| e.into_inner());
    let w = guard.as_ref()?;
    let elapsed = now.duration_since(w.since).as_secs();
    if elapsed >= WAKE_SECS {
        return None;
    }
    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
    let same = matches!(
        (strip(&runways, &w.runway_ident), strip(&runways, runway_ident)),
        (Some(a), Some(b)) if std::ptr::eq(a, b)
    );
    same.then_some(WAKE_SECS - elapsed)
}

// --- Taxi routes -------------------------------------------------------------

/// The active field's taxi network, loaded in the background (a 400 MB
/// apt.dat scan must never stall the reply path). `None` while loading or
/// when the field has none — taxi clearances then phrase route-less,
/// exactly as before.
static TAXI_NET: Mutex<Option<std::sync::Arc<nav::TaxiNet>>> = Mutex::new(None);

/// The taxi route the pilot is CLEARED on — published by the reply path
/// when a taxi clearance is issued, consumed by the compliance watch (ATC's
/// off-route challenge) and the copilot's progressive taxi guide (the crew
/// side: "next left onto Whiskey").
pub struct TaxiWatch {
    /// Node polyline of the cleared route.
    pub path: Vec<(f64, f64)>,
    /// The spoken via line, for the challenge ("taxi via Alpha, Bravo").
    pub via: String,
    /// Per-segment RAW (name, is_runway) aligned with `path` segments
    /// (`seg[i]` = the leg `path[i] -> path[i+1]`); phrase with
    /// [`spoken_taxiway`] / [`atis::spoken_runway`] at speak time.
    pub seg: Vec<(String, bool)>,
    /// RAW holding-point taxiway name ("A13"), the route's destination.
    pub holding_point: String,
}

/// The active field's ILS facilities (earth_nav.dat) and per-runway platform
/// altitudes (CIFP approach legs), background-loaded per station — both are
/// full-file scans that must never stall the reply path. `None` while loading
/// or when the field has no ILS; vectors then fly the visual profile.
pub struct IlsFacts {
    pub ils: Vec<nav::AirportIls>,
    /// Glideslope-intercept ("descend to X until established") altitude per
    /// runway ident, ft MSL, where the CIFP has the approach.
    pub platform_ft: std::collections::HashMap<String, f64>,
}

static ILS_FACTS: Mutex<Option<std::sync::Arc<IlsFacts>>> = Mutex::new(None);

/// Kick off the background ILS + procedures load for the active airport.
/// Called by the station's owner at install time, like the taxi network.
pub fn spawn_ils_load(nav_path: String, cifp_dir: String, icao: String) {
    *ILS_FACTS.lock().unwrap_or_else(|e| e.into_inner()) = None;
    std::thread::spawn(move || match nav::airport_ils(&nav_path, &icao) {
        Ok(list) if !list.is_empty() => {
            let platform_ft: std::collections::HashMap<String, f64> = list
                .iter()
                .filter_map(|i| {
                    nav::ils_platform_alt_ft(&cifp_dir, &icao, &i.runway)
                        .map(|a| (i.runway.clone(), a))
                })
                .collect();
            tracing::info!(
                %icao,
                ils = list.len(),
                platforms = platform_ft.len(),
                "ILS data loaded: vectors can offer the ILS approach"
            );
            *ILS_FACTS.lock().unwrap_or_else(|e| e.into_inner()) =
                Some(std::sync::Arc::new(IlsFacts { ils: list, platform_ft }));
        }
        Ok(_) => tracing::info!(%icao, "no ILS at this field; visual vectors only"),
        Err(e) => tracing::warn!(%icao, "ILS scan failed: {e:#}"),
    });
}

/// Kick off the background taxi-network load for the active airport.
/// Called by the station's owner at install time.
pub fn spawn_taxi_net_load(apt_path: String, icao: String) {
    *TAXI_NET.lock().unwrap_or_else(|e| e.into_inner()) = None;
    std::thread::spawn(move || match nav::find_taxi_net(&apt_path, &icao) {
        Ok(Some(net)) => {
            tracing::info!(
                %icao,
                nodes = net.nodes.len(),
                edges = net.edges.len(),
                "taxi network loaded: clearances will carry named routes"
            );
            *TAXI_NET.lock().unwrap_or_else(|e| e.into_inner()) =
                Some(std::sync::Arc::new(net));
        }
        Ok(None) => tracing::info!(%icao, "no taxi network in apt.dat; route-less taxi clearances"),
        Err(e) => tracing::warn!(%icao, "taxi network scan failed: {e:#}"),
    });
}

/// Plan the taxi route from the aircraft to the holding point of `ident`'s
/// runway end. `None` when there's no network (yet), no snap, or no path —
/// the clearance then falls back to the route-less phrasing.
fn plan_taxi_route_to(
    runways: &[nav::Runway],
    ident: &str,
    lat: f64,
    lon: f64,
) -> Option<nav::TaxiRoutePlan> {
    let net = TAXI_NET.lock().unwrap_or_else(|e| e.into_inner()).clone()?;
    let r = strip(runways, ident)?;
    let (tlat, tlon) = if r.ident_a == ident {
        (r.lat_a, r.lon_a)
    } else {
        (r.lat_b, r.lon_b)
    };
    let mut plan = net.plan(lat, lon, tlat, tlon)?;
    backfill_unnamed_crossings(&mut plan, runways);
    Some(plan)
}

/// Name the crossings the planner couldn't: apt.dat authors often leave
/// runway network edges UNNAMED, so the planner's crossings list (named
/// edges only) missed them and the clearance never said "cross runway ..."
/// — the pilot's first hint was the (former) incursion berate mid-crossing
/// (2026-07-14). A runway-flagged leg with no name is matched to whichever
/// physical strip its midpoint sits on, and that strip's designator joins
/// the crossings list.
fn backfill_unnamed_crossings(plan: &mut nav::TaxiRoutePlan, runways: &[nav::Runway]) {
    for (i, (name, is_runway)) in plan.seg.iter().enumerate() {
        if !*is_runway || !name.is_empty() {
            continue;
        }
        // Sample the leg every ~20 m (its midpoint alone can miss a strip
        // the leg only clips): first sampled point on a runway names it.
        let (a, b) = (plan.path[i], plan.path[i + 1]);
        let len_m = crate::nav::haversine_nm(a.0, a.1, b.0, b.1) * 1852.0;
        let steps = ((len_m / 20.0).ceil() as usize).max(1);
        let hit = (0..=steps).find_map(|k| {
            let t = k as f64 / steps as f64;
            let (plat, plon) = (a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1));
            runways.iter().find(|r| on_runway(r, plat, plon).is_some())
        });
        if let Some(r) = hit {
            let full = format!("{}/{}", r.ident_a, r.ident_b);
            if !plan.crossings.contains(&full) {
                plan.crossings.push(full);
            }
        }
    }
}

/// A taxiway name spoken like a controller says it: designator letters go
/// NATO, digits are read out, real words stay words — "B2" → "Bravo two",
/// "N4W" → "November four Whiskey", "LINK27" → "Link two seven",
/// "Inner" stays "Inner". Per token: letter runs of 1-2 are designators
/// (NATO), 3+ are words; digit runs are always spoken.
pub fn spoken_taxiway(name: &str) -> String {
    let mut out: Vec<String> = Vec::new();
    for tok in name.split_whitespace() {
        let mut run = String::new();
        let flush = |run: &mut String, out: &mut Vec<String>| {
            if run.is_empty() {
                return;
            }
            if run.chars().all(|c| c.is_ascii_digit()) {
                out.push(atis::spoken_digits(run));
            } else if run.chars().count() <= 2 {
                out.push(tower::spoken_callsign(run));
            } else {
                // A real word: keep it, title-cased so ALL-CAPS apt.dat
                // names ("LINK") don't get spelled out by the TTS.
                let mut w = run.to_lowercase();
                if let Some(c) = w.get_mut(0..1) {
                    c.make_ascii_uppercase();
                }
                out.push(w);
            }
            run.clear();
        };
        let mut prev_digit: Option<bool> = None;
        for c in tok.chars() {
            let digit = c.is_ascii_digit();
            if prev_digit.is_some_and(|p| p != digit) {
                flush(&mut run, &mut out);
            }
            run.push(c);
            prev_digit = Some(digit);
        }
        flush(&mut run, &mut out);
    }
    out.join(" ")
}

/// The designator-and-number split of a taxiway name ("M3" → ("M", 3)) —
/// `None` when the name doesn't end in digits or doesn't start with them.
fn stem_num(name: &str) -> Option<(&str, i64)> {
    let idx = name.find(|c: char| c.is_ascii_digit())?;
    let (stem, num) = name.split_at(idx);
    if stem.is_empty() {
        return None;
    }
    Some((stem, num.parse().ok()?))
}

/// A via list spoken like a controller reads it: a run of three or more
/// same-designator, consecutively numbered segments collapses to its ends —
/// "C1, M1, M2, M3, M4, M5" becomes "Charlie one, Mike one to Mike five".
/// The apt.dat network names every stretch of pavement; a clearance reads
/// like a controller, not like the database (Madrid, 2026-07-13).
pub fn spoken_via(names: &[String]) -> Vec<String> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < names.len() {
        let mut j = i + 1;
        if let Some((stem, first)) = stem_num(&names[i]) {
            let mut prev = first;
            let mut step = 0i64;
            while j < names.len() {
                let Some((s, n)) = stem_num(&names[j]) else { break };
                let d = n - prev;
                if s != stem || d.abs() != 1 || (step != 0 && d != step) {
                    break;
                }
                step = d;
                prev = n;
                j += 1;
            }
            if j - i >= 3 {
                out.push(format!(
                    "{} to {}",
                    spoken_taxiway(&names[i]),
                    spoken_taxiway(&names[j - 1])
                ));
                i = j;
                continue;
            }
        }
        out.push(spoken_taxiway(&names[i]));
        i += 1;
    }
    out
}

/// The first designator of a runway-edge name ("09L/27R" → "09L") — the
/// end a crossing instruction names. `pub` for the copilot's taxi guide,
/// which phrases the same runway-edge names.
pub fn first_designator(name: &str) -> &str {
    name.split('/').next().unwrap_or(name).trim()
}

/// Distance from a point to a polyline, meters — the off-route check.
/// Whether the pilot stands where the cleared taxi route itself runs over a
/// runway — a crossing or backtrack the tower put in the clearance ("cross
/// runway two seven"). Geometry, not the apt.dat runway flag: authors leave
/// crossing edges unnamed and sometimes unflagged, and the flag adds nothing
/// the pavement test doesn't. The pilot must be within the off-route corridor
/// of the route AND the route's nearest point must itself be ON a runway —
/// so nosing past a hold-short point onto a runway the route only ends
/// NEAR still reads as an incursion (the route's nearest point there is the
/// holding point, off the pavement).
fn on_cleared_runway_crossing(lat: f64, lon: f64) -> bool {
    const CORRIDOR_M: f64 = 60.0;
    let nearest = {
        let route = TAXI_ROUTE.lock().unwrap_or_else(|e| e.into_inner());
        let Some(r) = route.as_ref() else { return false };
        nearest_on_polyline(&r.path, lat, lon)
    };
    // Route lock dropped before taking the runway lock — no nesting.
    nearest.is_some_and(|(p, d)| d <= CORRIDOR_M && on_active_runway(p.0, p.1).is_some())
}

/// The closest point of a polyline to (lat, lon), with its distance in
/// meters. Flat-earth at taxi scales, like [`dist_to_polyline_m`].
fn nearest_on_polyline(path: &[(f64, f64)], lat: f64, lon: f64) -> Option<((f64, f64), f64)> {
    let m_per_deg_lat = 111_320.0;
    let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
    path.windows(2)
        .map(|w| {
            let (ax, ay) = ((w[0].1 - lon) * m_per_deg_lon, (w[0].0 - lat) * m_per_deg_lat);
            let (bx, by) = ((w[1].1 - lon) * m_per_deg_lon, (w[1].0 - lat) * m_per_deg_lat);
            let (dx, dy) = (bx - ax, by - ay);
            let len2 = dx * dx + dy * dy;
            let t = if len2 < 1.0 {
                0.0
            } else {
                (-(ax * dx + ay * dy) / len2).clamp(0.0, 1.0)
            };
            let (cx, cy) = (ax + t * dx, ay + t * dy);
            let plat = w[0].0 + t * (w[1].0 - w[0].0);
            let plon = w[0].1 + t * (w[1].1 - w[0].1);
            ((plat, plon), (cx * cx + cy * cy).sqrt())
        })
        .min_by(|x, y| x.1.total_cmp(&y.1))
}

fn dist_to_polyline_m(path: &[(f64, f64)], lat: f64, lon: f64) -> Option<f64> {
    let m_per_deg_lat = 111_320.0;
    let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
    path.windows(2)
        .map(|w| {
            let (ax, ay) = ((w[0].1 - lon) * m_per_deg_lon, (w[0].0 - lat) * m_per_deg_lat);
            let (bx, by) = ((w[1].1 - lon) * m_per_deg_lon, (w[1].0 - lat) * m_per_deg_lat);
            let (dx, dy) = (bx - ax, by - ay);
            let len2 = dx * dx + dy * dy;
            let t = if len2 < 1.0 {
                0.0
            } else {
                (-(ax * dx + ay * dy) / len2).clamp(0.0, 1.0)
            };
            let (cx, cy) = (ax + t * dx, ay + t * dy);
            (cx * cx + cy * cy).sqrt()
        })
        .min_by(|x, y| x.total_cmp(y))
}

/// Which controller COM1 is on, if any — the voice that narrates traffic.
pub fn tuned_controller(com1_khz: i64) -> Option<StationSpeaker> {
    CONTROLLER_FREQS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .iter()
        .find(|(k, _)| (com1_khz - k).abs() <= 5)
        .map(|(_, s)| *s)
}

/// Which station COM1 hears — controllers OR the ATIS loop. Unlike
/// [`tuned_controller`] this resolves the ATIS frequency too, so the speech
/// arbiter can tell "on the ATIS broadcast" from "off frequency" and cut
/// live station audio the instant the pilot tunes away.
pub fn tuned_station(com1_khz: i64) -> Option<StationSpeaker> {
    tuned_controller(com1_khz).or_else(|| {
        ATIS.lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .filter(|a| (com1_khz - a.khz).abs() <= 5)
            .map(|_| StationSpeaker::Atis)
    })
}

/// The information letter in a call ("with information Alpha", "we have
/// information bravo", or just "information B").
pub fn heard_information(t: &str) -> Option<char> {
    let mut toks = t
        .split(|c: char| !c.is_alphanumeric())
        .filter(|w| !w.is_empty());
    while let Some(tok) = toks.next() {
        if tok != "information" {
            continue;
        }
        let next = toks.next()?;
        // A NATO word ("alpha") or a bare letter ("b").
        if next.len() == 1 {
            let c = next.chars().next()?.to_ascii_uppercase();
            if c.is_ascii_uppercase() {
                return Some(c);
            }
        }
        for letter in 'A'..='Z' {
            if atis::nato(letter).to_ascii_lowercase() == next {
                return Some(letter);
            }
        }
        return None;
    }
    None
}

/// An armed vectors-to-final plan — the approach controller's worksheet. Set
/// by [`Station::arm_vectors`], driven by [`vector_tick`] from the monitor's
/// poll loop (the same shape as the talk-down's `approach_guide::GUIDE`).
pub struct VectorPlan {
    pub callsign: String,
    pub thr_lat: f64,
    pub thr_lon: f64,
    /// Final approach course, degrees true.
    pub course_deg: f64,
    pub runway_spoken: String,
    /// Tower frequency, spoken — the handoff at the end of the vectors.
    pub tower_spoken: Option<String>,
    /// Intercept altitude, ft MSL.
    pub target_alt_ft: f64,
    /// ILS mode: the runway has a localizer, so the vectors end in "cleared
    /// ILS approach, report established" instead of the visual clearance.
    pub ils: bool,
    /// Where the vectors aim the final join, nm from the threshold. Visual:
    /// the stock 6. ILS: far enough out that the glideslope is still ABOVE
    /// the platform altitude there — intercepting from below is the whole
    /// discipline; a fixed 6 nm join under Heathrow's 2,500 ft platform
    /// delivered the aircraft on top of the slope.
    join_nm: f64,
    /// The ILS clearance has been spoken (it rides the intercept turn, once).
    cleared: bool,
    /// Phraseology dialect, carried from the station that armed the plan.
    dialect: Dialect,
    last_tx: Option<Instant>,
    last_heading: Option<i64>,
}

/// How far out the vectors aim you onto final (the join point) for a VISUAL
/// approach; an ILS join is computed from its platform instead.
const JOIN_FINAL_NM: f64 = 6.0;

/// The ILS join distance: where a `gs_deg` slope reaches `platform_agl_ft`
/// above the field, plus a mile of level flight to capture from below —
/// never inside the visual join. Pure, so the geometry unit-tests.
fn ils_join_nm(platform_agl_ft: f64, gs_deg: f64) -> f64 {
    let slope_nm = platform_agl_ft.max(0.0) / (6076.12 * gs_deg.max(1.0).to_radians().tan());
    (slope_nm + 1.0).max(JOIN_FINAL_NM)
}

/// One tick of the approach controller, ~monitor rate. Returns an instruction
/// when one is due: a fresh heading when the required course has moved, the
/// approach clearance + tower handoff once established, silence otherwise.
/// Pure narration — the pilot flies. `track_deg` is the ground track (degrees
/// true, `None` = read failed); `now` is injected so the cadence unit-tests.
pub fn vector_tick(
    lat: f64,
    lon: f64,
    alt_ft: f64,
    track_deg: Option<f64>,
    on_ground: bool,
    now: Instant,
) -> Option<String> {
    let mut guard = VECTORS.lock().unwrap_or_else(|e| e.into_inner());
    let plan = guard.as_mut()?;
    if on_ground {
        *guard = None; // landed (or armed on the ramp by mistake): vectors over
        return None;
    }

    let dist_thr = crate::nav::haversine_nm(lat, lon, plan.thr_lat, plan.thr_lon);
    let brg_thr = crate::nav::bearing_deg(lat, lon, plan.thr_lat, plan.thr_lon);
    // Established: on the approach side, on the extended centerline, inside the
    // join distance, AND actually flying the final course — the position wedge
    // alone would clear an aircraft crossing final at ninety degrees. A failed
    // track read doesn't block the clearance (best-effort, like every read).
    let off_course = angle_diff_deg(brg_thr, plan.course_deg).abs();
    let aligned = track_deg.is_none_or(|t| angle_diff_deg(t, plan.course_deg).abs() <= 30.0);
    // Established, two regimes. Beyond a mile: the position wedge — on the
    // approach side, near the extended centerline, inside the join, flying
    // the course. Inside a mile the threshold bearing is too twitchy for a
    // 15-degree wedge, and a tight base that rolls out on final at 0.9 nm
    // never satisfied the old 1.0 nm floor — the plan vectored a short-final
    // Cessna AWAY ("fly heading zero niner zero" at 272 feet, toward the
    // join point behind it) and touchdown then discarded the plan with no
    // clearance ever spoken, handing a compliant pilot the landing berate
    // (Gibraltar, 2026-07-13). Inside the mile the test is landing intent:
    // below the vector altitude, pointed down the runway, threshold still
    // broadly ahead.
    let established = if dist_thr >= 1.0 {
        dist_thr <= plan.join_nm + 2.0 && off_course <= 15.0 && aligned
    } else {
        alt_ft < plan.target_alt_ft && off_course <= 45.0 && aligned
    };
    if established {
        let handoff = plan
            .tower_spoken
            .as_ref()
            .map(|f| format!(" Contact Tower {f}."))
            .unwrap_or_default();
        // A short-final clearance rounds to "1 mile", never "0 miles".
        let miles = (dist_thr.round() as i64).max(1);
        let miles = format!("{miles} mile{}", if miles == 1 { "" } else { "s" });
        let msg = if plan.ils && plan.cleared {
            // Cleared on the intercept, now seen established: hand off. (If
            // the pilot reports established first, respond_approach does
            // this and the plan is already gone.)
            format!("{}, {miles} from touchdown.{handoff}", plan.callsign)
        } else if plan.ils {
            // Straight-in that never needed an intercept turn: clearance and
            // handoff together.
            format!(
                "{}, {miles} from touchdown. Cleared ILS approach runway {}.{handoff}",
                plan.callsign, plan.runway_spoken
            )
        } else {
            format!(
                "{}, {miles} from touchdown. Cleared visual approach runway {}.{handoff}",
                plan.callsign, plan.runway_spoken
            )
        };
        // Handed to the tower with an APPROACH clearance — the pilot still owes
        // a "final" report before the tower's landing clearance. Arm the crew
        // coach so a missed report doesn't become a landing-without-clearance
        // (the compliance watch clears this on the landing clearance and on
        // touchdown). Only when there's a tower to report to.
        if plan.tower_spoken.is_some() {
            *LANDING_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) =
                Some(plan.runway_spoken.clone());
        }
        *guard = None; // approach's job is done
        return Some(msg);
    }

    // Inside a mile and not (yet) landing: no heading. Any vector from here
    // aims at a join point BEHIND the aircraft — a reversal at circuit
    // height. Stay quiet and let the geometry develop; if they blow through
    // final, steering resumes beyond the mile.
    if dist_thr < 1.0 {
        return None;
    }

    // Steering, two regimes. Near the extended centerline on the approach
    // side: ONE honest intercept — a 30° cut toward the final course,
    // relaxing to the course itself as the centerline arrives. This used to
    // be a pursuit curve (correction ∝ cross-track); against a real pilot's
    // turn lag and the 25 s call cadence the gain wound up, marched the
    // headings 200→250, blew through the localizer, and then re-vectored a
    // reversal to the fallen-behind join point — an orbit, flown live at
    // Jerez 2026-07-12. Controllers give a cut, not a curve. Anywhere else:
    // direct to the join point, which is the downwind/base leg for free.
    let along_nm = dist_thr * off_course.to_radians().cos(); // + = approach side
    let xtk_nm = dist_thr * angle_diff_deg(brg_thr, plan.course_deg).to_radians().sin();
    let capturing = along_nm >= 1.0 && xtk_nm.abs() <= 2.5;
    // On the intercept after the ILS clearance, TRACKING the course: the
    // frequency goes quiet. Capturing the localizer is the pilot's job now;
    // a controller who keeps issuing headings against a coupling autopilot
    // fights it ("cleared ILS" followed by "fly heading 360", taped
    // 2026-07-11). But quiet needs BOTH near the centerline AND actually
    // flying its direction — a pilot sailing through the localizer on the
    // cut heading still gets the correction.
    if plan.ils && plan.cleared && capturing && xtk_nm.abs() <= 1.5 && aligned {
        return None;
    }
    let desired = if capturing {
        if xtk_nm.abs() <= 0.4 {
            plan.course_deg
        } else {
            (plan.course_deg + 35.0_f64.copysign(xtk_nm)).rem_euclid(360.0)
        }
    } else {
        // Aim BEYOND the join, not at it: the last leg then converges onto
        // the centerline's axis with room for the 35° cut to finish. Aiming
        // AT the join arrives perpendicular with no room, drops out of the
        // capture band at the bottom, and the re-vector back to a join now
        // BEHIND the aircraft is the other half of the Jerez orbit.
        let recip = (plan.course_deg + 180.0).rem_euclid(360.0);
        let (join_lat, join_lon) = crate::nav::project_latlon(
            plan.thr_lat,
            plan.thr_lon,
            recip,
            plan.join_nm + 2.0,
        );
        crate::nav::bearing_deg(lat, lon, join_lat, join_lon)
    };
    let heading10 = {
        let h = ((desired / 10.0).round() as i64 * 10).rem_euclid(360);
        if h == 0 {
            360
        } else {
            h
        }
    };

    // Cadence: speak immediately the first time, then only when the required
    // heading has genuinely moved — a real drift after 25 s, a big cut after
    // 8 s. Holding the assigned heading stays silent, like a real frequency.
    // The drift floor is 20°: headings are rounded to 10°, so a 10° "move" is
    // one quantization step — re-vectoring it every 25 s trimmed a stable
    // downwind 070→080→070 (taped 2026-07-12). A controller who keeps issuing
    // 10-degree corrections is chasing the aircraft; a real one picks a
    // heading that works and lets the geometry develop (the 35° intercept cut
    // always clears the 30° fast rule, so the turn onto final is never held).
    let elapsed = plan
        .last_tx
        .map(|t| now.duration_since(t).as_secs_f64())
        .unwrap_or(f64::MAX);
    let moved = plan
        .last_heading
        .map(|h| angle_diff_deg(h as f64, heading10 as f64).abs())
        .unwrap_or(360.0);
    let due = plan.last_tx.is_none()
        || (elapsed >= 25.0 && moved >= 20.0)
        || (elapsed >= 8.0 && moved >= 30.0);
    if !due {
        return None;
    }
    plan.last_tx = Some(now);
    plan.last_heading = Some(heading10);

    let mut msg = format!(
        "{}, fly heading {}{}",
        plan.callsign,
        atis::spoken_digits(&format!("{heading10:03}")),
        // CAP 413 headings carry the unit; FAA's don't.
        if plan.dialect == Dialect::Icao { " degrees" } else { "" }
    );
    if alt_ft > plan.target_alt_ft + 400.0 {
        msg.push_str(&match plan.dialect {
            Dialect::Icao => {
                format!(", descend to altitude {} feet", spoken_alt(plan.target_alt_ft))
            }
            Dialect::Faa => format!(", descend and maintain {}", spoken_alt(plan.target_alt_ft)),
        });
    }
    msg.push('.');
    // The ILS clearance rides the intercept turn, once — the heading that
    // captures the localizer is the one that comes with "cleared ILS".
    if plan.ils && !plan.cleared && capturing {
        plan.cleared = true;
        msg.push_str(&match plan.dialect {
            Dialect::Icao => format!(
                " Cleared ILS approach runway {}, report established.",
                plan.runway_spoken
            ),
            // The FAA PTAC: position was the vector, this is turn-altitude-
            // clearance in one breath.
            Dialect::Faa => format!(
                " Maintain {} until established, cleared ILS runway {} approach.",
                spoken_alt(plan.target_alt_ft),
                plan.runway_spoken
            ),
        });
    }
    Some(msg)
}

/// The nearest airborne factor from the TCAS picture: range, clock position,
/// and relative altitude (positive = traffic above us), rounded to the
/// hundred feet a pilot scans in.
pub struct TrafficFactor {
    pub nm: f64,
    pub clock: i32,
    pub ft: f64,
}

/// The nearest TCAS target inside 6 nm and ±1,500 ft; `None` = no factor.
/// Reads the sim's TCAS arrays, which any traffic source (LiveTraffic,
/// default AI aircraft) populates — ATC narrates traffic, never controls it.
pub async fn nearest_traffic_factor(xp: SimView<'_>) -> Option<TrafficFactor> {
    let bear = xp
        .get_float_array("sim/cockpit2/tcas/indicators/relative_bearing_degs")
        .await
        .ok()?;
    let dist = xp
        .get_float_array("sim/cockpit2/tcas/indicators/relative_distance_mtrs")
        .await
        .ok()?;
    let alt = xp
        .get_float_array("sim/cockpit2/tcas/indicators/relative_altitude_mtrs")
        .await
        .ok()?;
    // Slot 0 is own ship; live targets have a real distance.
    let (d, b, a) = dist
        .iter()
        .zip(&bear)
        .zip(&alt)
        .skip(1)
        .filter(|((d, _), _)| **d > 1.0)
        .map(|((d, b), a)| (*d, *b, *a))
        .filter(|(d, _, a)| *d < 6.0 * 1852.0 && a.abs() < 1500.0 / 3.28084)
        .min_by(|x, y| x.0.total_cmp(&y.0))?;
    let clock = match (((b + 15.0).rem_euclid(360.0)) / 30.0) as i32 {
        0 => 12,
        c => c,
    };
    Some(TrafficFactor {
        nm: d / 1852.0,
        clock,
        ft: ((a * 3.28084) / 100.0).round() * 100.0,
    })
}

/// The relative-altitude clause both traffic calls share.
fn vert_phrase(ft: f64) -> String {
    if ft.abs() < 300.0 {
        "same altitude".to_string()
    } else if ft > 0.0 {
        format!("{ft:.0} feet above")
    } else {
        format!("{:.0} feet below", ft.abs())
    }
}

/// Factor traffic near the vectored arrival, spoken as a controller advisory
/// ("traffic, two o'clock, three miles, five hundred feet above").
/// `None` = no factor, frequency stays quiet.
pub async fn traffic_advisory(xp: SimView<'_>) -> Option<String> {
    let f = nearest_traffic_factor(xp).await?;
    let range =
        if f.nm < 1.0 { "under a mile".to_string() } else { format!("{:.0} miles", f.nm) };
    Some(format!("traffic, {} o'clock, {range}, {}.", f.clock, vert_phrase(f.ft)))
}

/// Is this factor a NEAR MISS — unsafe proximity, not just a factor to scan
/// for? Half a mile and 500 ft is the NMAC line; the advisory tier has been
/// narrating the target since 6 nm, so by here the calm call has already
/// failed to separate us.
pub fn is_near_miss(f: &TrafficFactor) -> bool {
    f.nm < 0.5 && f.ft.abs() < 500.0
}

/// Which way OUT of a near miss: vertical when the intruder is clearly above
/// or below (go the other way), lateral when co-altitude — away from the
/// traffic's side, right for head-on (the right-of-way rule every pilot
/// already knows), and vertical again for one directly behind, where a turn
/// keeps it in the blind spot.
fn avoidance(clock: i32, ft: f64) -> &'static str {
    if ft >= 100.0 {
        "descend"
    } else if ft <= -100.0 {
        "climb"
    } else {
        match clock {
            12 => "turn right",
            1..=5 => "turn left",
            7..=11 => "turn right",
            _ => "climb",
        }
    }
}

/// The near-miss call: the controller's safety alert (FAA 7110.65 shape,
/// "traffic alert ... advise you ... immediately") when a frequency is tuned,
/// the crew's shout when nobody is. Advisory words only, both ways — the
/// copilot never maneuvers uncommanded.
pub fn near_miss_line(f: &TrafficFactor, crew: bool) -> String {
    let act = avoidance(f.clock, f.ft);
    if crew {
        format!(
            "Traffic! {} o'clock, half a mile, {}. {act}, now.",
            f.clock,
            vert_phrase(f.ft)
        )
    } else {
        format!(
            "traffic alert, {} o'clock, less than a mile, {}, advise you {act} immediately.",
            f.clock,
            vert_phrase(f.ft)
        )
    }
}

/// Smallest signed angle a−b, degrees, in −180..180.
fn angle_diff_deg(a: f64, b: f64) -> f64 {
    (a - b + 540.0).rem_euclid(360.0) - 180.0
}

/// An altitude spoken controller-style: 2000 → "two thousand", 1500 → "one
/// thousand five hundred".
fn spoken_alt(ft: f64) -> String {
    let hundreds = ((ft / 100.0).round() as i64).max(1);
    let (th, hu) = (hundreds / 10, hundreds % 10);
    let word = |n: i64| atis::spoken_digits(&n.to_string());
    match (th, hu) {
        (0, h) => format!("{} hundred", word(h)),
        (t, 0) => format!("{} thousand", word(t)),
        (t, h) => format!("{} thousand {} hundred", word(t), word(h)),
    }
}

/// Outcome of a named-station tuning request ("tune the tower").
pub enum Tune {
    /// Set COM1 to this kHz; `label` is spoken in the readback.
    Found { label: String, khz: i64 },
    /// The airport has no such frequency — decline by name, don't guess.
    Missing(String),
}

/// Every comm service a field can publish, in briefing order (the order a
/// pilot works them: information first, then the ground-to-air chain).
const SERVICES: &[(&str, u32)] = &[
    ("atis", nav::FREQ_ATIS),
    ("clearance", nav::FREQ_CLEARANCE),
    ("ground", nav::FREQ_GROUND),
    ("tower", nav::FREQ_TOWER),
    ("approach", nav::FREQ_APPROACH),
    ("departure", nav::FREQ_DEPARTURE),
    ("unicom", nav::FREQ_UNICOM),
];

/// The field's published services in briefing order, for the dashboard's
/// radio panel: (service name, frequency in kHz). The same rows the spoken
/// briefing reads, so screen and voice can never disagree.
pub fn services(apt: &nav::AirportInfo) -> Vec<(String, i64)> {
    SERVICES
        .iter()
        .filter_map(|(word, kind)| apt.freq(*kind).map(|f| (word.to_string(), f.khz)))
        .collect()
}

/// Is this a "what frequencies are here" / "who can I talk to" question?
/// Crew knowledge, not a radio call — checked after the tuning verbs so
/// "tune the tower" still tunes. Deterministic on purpose: the answer is a
/// list read from apt.dat, exactly the kind of question the LLM must never
/// improvise.
pub fn freq_query(t: &str) -> bool {
    let interrogative = t.contains("what")
        || t.contains("which")
        || t.contains("list")
        || t.contains("any")
        || t.contains("available");
    let subject = t.contains("frequenc") || t.contains("station");
    let talk = t.contains("who")
        && (t.contains("talk") || t.contains("call") || t.contains("contact"));
    (interrogative && subject) || talk
}

impl Station {
    /// The spoken frequency briefing: every service this field publishes, plus
    /// what COM1 is on right now (named when it matches a service). Pure —
    /// the caller reads COM1 (see [`Station::freq_briefing`]) so this tests
    /// without a simulator.
    pub fn briefing_text(&self, com1_khz: Option<i64>) -> String {
        let parts: Vec<String> = SERVICES
            .iter()
            .filter_map(|(word, kind)| {
                self.apt
                    .freq(*kind)
                    .map(|f| format!("{word} {}", atis::spoken_mhz(f.khz)))
            })
            .collect();
        let mut line = if parts.is_empty() {
            format!("{} has no published frequencies.", spoken_airport(&self.apt.name))
        } else {
            format!("{}: {}.", spoken_airport(&self.apt.name), parts.join(", "))
        };
        if let Some(com1) = com1_khz {
            // Same 5 kHz tolerance as tuned_service: 8.33 channel names can
            // sit 5 kHz off the carrier. Some fields publish ground and tower
            // on the SAME frequency (8 in the 2026-07 apt.dat, military
            // mostly) — name the service that would actually answer, in
            // tuned_service's precedence, not the first row in briefing order.
            let matches: Vec<&str> = SERVICES
                .iter()
                .filter_map(|(word, kind)| {
                    self.apt
                        .freq(*kind)
                        .filter(|f| (f.khz - com1).abs() <= 5)
                        .map(|_| *word)
                })
                .collect();
            let tuned = matches
                .iter()
                .find(|w| matches!(**w, "tower" | "approach" | "departure"))
                .or_else(|| matches.first())
                .copied();
            line.push_str(&match tuned {
                Some(w) => format!(" Com one is on the {w}."),
                None => format!(" Com one is on {}.", atis::spoken_mhz(com1)),
            });
        }
        line
    }

    /// [`Station::briefing_text`] with COM1 read live. A failed read just
    /// drops the "com one is on" sentence — the list still answers.
    pub async fn freq_briefing(&self, xp: SimView<'_>) -> String {
        let com1 = xp
            .get_float(datarefs::COM1_FREQ_KHZ)
            .await
            .ok()
            .map(|v| v.round() as i64);
        self.briefing_text(com1)
    }

    /// Deterministic named-station tuning: "tune/select/switch to/contact the
    /// tower|atis|ground|approach|departure" (or generic "atc"/"station" =
    /// tower) resolved against this airport's apt.dat frequencies. Requires a
    /// tuning verb, so radio chatter that merely mentions a station never
    /// re-tunes the radio. `None` = not a tuning request at all.
    pub fn tune_request(&self, t: &str) -> Option<Tune> {
        // Flight-parameter talk that happens to name a station ("change to
        // approach speed", "select approach flaps") is never a tuning request.
        if t.contains("speed") || t.contains("knots") || t.contains("flaps")
            || t.contains("heading") || t.contains("altitude")
        {
            return None;
        }
        let verb = t.contains("tune")
            || t.contains("switch to")
            || t.contains("select")
            || t.contains("contact")
            || t.contains("change to")
            || (t.contains("set") && t.contains("com"))
            // "change/switch frequency to the tower" — but only when the
            // sentence is about the radio: bare "change"/"switch" are too
            // promiscuous ("switch the landing light").
            || ((t.contains("change") || t.contains("switch"))
                && (t.contains("freq") || t.contains("com") || t.contains("radio")));
        if !verb {
            return None;
        }
        let named = SERVICES.iter().find(|(w, _)| t.contains(w));
        // Generic "tune atc" / "tune the station" means the tower.
        let (word, kind) = match named {
            Some((w, k)) => (*w, *k),
            None if t.contains("atc") || t.contains("station") => ("tower", nav::FREQ_TOWER),
            None => return None,
        };
        Some(match self.apt.freq(kind) {
            Some(f) => Tune::Found {
                label: format!("{} {}", spoken_airport(&self.apt.name), word),
                khz: f.khz,
            },
            None => Tune::Missing(format!(
                "{} has no {} frequency.",
                spoken_airport(&self.apt.name),
                word
            )),
        })
    }
}

/// Wind spoken tower-style ("zero niner zero degrees, three knots" / "calm").
fn spoken_wind(from_deg: f64, kts: f64) -> String {
    if kts < 3.0 {
        return "calm".to_string();
    }
    let dir = ((from_deg / 10.0).round() as i64 * 10).rem_euclid(360);
    let dir = if dir == 0 { 360 } else { dir };
    format!(
        "{} degrees, {} knots",
        atis::spoken_digits(&format!("{dir:03}")),
        atis::spoken_digits(&format!("{kts:.0}"))
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn discrete_squawks_are_valid_and_deterministic() {
        for cs in ["November one seven two Sierra Papa", "G-ABCD", "x"] {
            let code = discrete_squawk(cs);
            assert_eq!(code, discrete_squawk(cs), "stable per callsign");
            assert!((4500..=4577).contains(&code), "{code} outside the block");
            // Every digit octal (transponders have no 8s or 9s).
            assert!(code.to_string().chars().all(|c| ('0'..='7').contains(&c)), "{code}");
            // Clear of every reserved code by construction.
            assert!(![0, 1200, 2000, 7000, 7500, 7600, 7700].contains(&code));
        }
    }

    #[test]
    fn squawk_watch_calls_radar_contact_only_when_the_code_shows() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut st = lxgb();
        let spoken = st.assign_squawk("November one seven two Sierra Papa", StationSpeaker::Approach);
        let code = st.assigned_squawk.expect("code issued");
        assert_eq!(spoken, atis::spoken_digits(&code.to_string()));
        // Same flight asks again: same code, watch not reset.
        assert_eq!(st.assign_squawk("November one seven two Sierra Papa", StationSpeaker::Approach), spoken);

        let t0 = Instant::now();
        // Still on the old code, inside the grace period: silence.
        assert!(squawk_tick(Some(7000), true, t0).is_none());
        // Grace expired, still wrong: the nudge, once, from the assigner.
        let late = t0 + std::time::Duration::from_secs(25);
        let (line, who) = squawk_tick(Some(7000), true, late).expect("nudge");
        assert!(line.contains("confirm squawk"), "{line}");
        assert!(matches!(who, StationSpeaker::Approach));
        // Immediately again: quiet (nudge cooldown).
        assert!(squawk_tick(Some(7000), true, late).is_none());
        // Right code but transponder on standby: not replying, no contact —
        // and the nudge names the REAL problem, never "confirm squawk" (the
        // digits are the one thing that's right; Gatwick 2026-07-13).
        let later = late + std::time::Duration::from_secs(50);
        let (line, _) = squawk_tick(Some(code), false, later).expect("standby nudge");
        assert!(line.contains("negative radar contact, recycle transponder"), "{line}");
        assert!(!line.contains("confirm squawk"), "{line}");
        // Code set and replying: "radar contact", exactly once.
        let (line, _) = squawk_tick(Some(code), true, later + std::time::Duration::from_secs(50))
            .expect("radar contact");
        assert!(line.contains("radar contact"), "{line}");
        assert!(squawk_tick(Some(code), true, later + std::time::Duration::from_secs(60)).is_none());
        assert!(!squawk_watch_active(), "satisfied watch stops the dataref reads");
        *SQUAWK_WATCH.lock().unwrap() = None;
    }

    #[test]
    fn departure_clearance_assigns_a_discrete_code_at_radar_fields() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *SQUAWK_WATCH.lock().unwrap() = None;
        let wx = Wx { wind_from_deg: 90.0, wind_kts: 8.0 };
        // LXGB has an approach service: the zone clearance carries the code.
        let mut st = lxgb();
        let ctx = st.make_ctx(wx, "QNH one zero one seven".into(), "November one seven two Sierra Papa".into(), true, None, false, None);
        let r = st.respond(Service::Tower, tower::Intent::RequestClearance, &ctx, wx);
        let code = st.assigned_squawk.expect("discrete code issued");
        let spoken = atis::spoken_digits(&code.to_string());
        assert!(r.contains(&spoken), "{r}");
        assert!(!r.contains("seven zero zero zero"), "{r}");
        assert!(SQUAWK_WATCH.lock().unwrap().is_some(), "radar watch armed");
        // A tower-only strip stays on the VFR conspicuity.
        *SQUAWK_WATCH.lock().unwrap() = None;
        let mut st = lxgb();
        st.approach_khz = None;
        st.departure_khz = None;
        let ctx = st.make_ctx(wx, "QNH one zero one seven".into(), "November one seven two Sierra Papa".into(), true, None, false, None);
        let r = st.respond(Service::Tower, tower::Intent::RequestClearance, &ctx, wx);
        assert!(r.contains("seven zero zero zero"), "{r}");
        assert!(st.assigned_squawk.is_none());
        assert!(SQUAWK_WATCH.lock().unwrap().is_none());
        *CLEARANCE.lock().unwrap() = None;
    }

    #[test]
    fn tuned_station_resolves_controllers_and_the_atis() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *CONTROLLER_FREQS.lock().unwrap() =
            vec![(118105, StationSpeaker::Tower), (128500, StationSpeaker::Approach)];
        *ATIS.lock().unwrap() = Some(AtisState {
            khz: 125655,
            airport: "Jerez".into(),
            runways: Vec::new(),
            letter: 'A',
            edition: None,
            last_spoken: None,
        });
        // Controllers resolve (within the 5 kHz tolerance).
        assert_eq!(tuned_station(118105), Some(StationSpeaker::Tower));
        assert_eq!(tuned_station(128500), Some(StationSpeaker::Approach));
        // The ATIS loop resolves too — this is what tuned_controller misses,
        // so live ATIS audio isn't wrongly cut while you're ON its frequency.
        assert_eq!(tuned_station(125655), Some(StationSpeaker::Atis));
        assert_eq!(tuned_controller(125655), None, "controllers-only misses ATIS");
        // A real frequency that's none of them → off all stations (→ cut).
        assert_eq!(tuned_station(122800), None);
        *CONTROLLER_FREQS.lock().unwrap() = Vec::new();
        *ATIS.lock().unwrap() = None;
    }

    #[test]
    fn squawk_check_repeats_the_assigned_code() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let wx = Wx { wind_from_deg: 90.0, wind_kts: 8.0 };
        let mut st = lxgb();
        // Before any radar service: the VFR conspicuity.
        let ctx = st.make_ctx(wx, "QNH one zero one seven".into(), "November one seven two Sierra Papa".into(), false, None, false, None);
        let r = st.twr.handle(tower::Intent::SquawkCheck, &ctx);
        assert!(r.contains("seven zero zero zero"), "{r}");
        // Flight following assigns the discrete code...
        let r = st.respond_approach(tower::Intent::RequestFlightFollowing, &ctx, wx);
        let code = st.assigned_squawk.expect("code issued");
        let spoken = atis::spoken_digits(&code.to_string());
        assert!(r.contains(&spoken), "{r}");
        assert!(!r.contains("radar contact"), "contact is the watch's call, not theatre: {r}");
        // ...and every later squawk check repeats THAT code, tower included.
        let ctx = st.make_ctx(wx, "QNH one zero one seven".into(), "November one seven two Sierra Papa".into(), false, None, false, None);
        let r = st.twr.handle(tower::Intent::SquawkCheck, &ctx);
        assert!(r.contains(&spoken), "{r}");
        assert!(!r.contains("seven zero zero zero"), "{r}");
        *SQUAWK_WATCH.lock().unwrap() = None;
    }

    #[test]
    fn compliance_watch_holds_the_pilot_to_clearances() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // Gibraltar-ish: field reference + the real-ish 09/27 geometry.
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.1512, -5.3554));
        *ACTIVE_PANEL.lock().unwrap() =
            Some(("Gibraltar".into(), 11.0, vec![("tower".to_string(), 131200_i64)]));
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway { width_m: 0.0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1512,
            lon_a: -5.3554,
            lat_b: 36.1517,
            lon_b: -5.3366,
        }];
        *CLEARANCE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;

        // ZONE: airborne 3 nm out at 1,500 ft, never called anyone, not
        // tuned: the CREW flags it, with the tower frequency.
        let call = compliance_tick(36.20, -5.35, 1500.0, false, 90.0, false, t0).expect("zone");
        assert!(call.crew);
        assert!(call.line.contains("zone without clearance"), "{}", call.line);
        assert!(call.line.contains("decimal two"), "no tower freq: {}", call.line);
        // Cooldown holds.
        assert!(compliance_tick(36.20, -5.35, 1500.0, false, 90.0, false, t0).is_none());
        // Tuned after the cooldown: the CONTROLLER challenges.
        let call =
            compliance_tick(36.20, -5.35, 1500.0, false, 90.0, true, t0 + sec(130)).expect("challenge");
        assert!(!call.crew);
        assert!(call.line.contains("Say callsign"), "{}", call.line);
        // Above the zone cap: overflying is not an incursion.
        assert!(compliance_tick(36.20, -5.35, 4500.0, false, 90.0, true, t0 + sec(260)).is_none());

        // Contact made (any answered call publishes the snapshot): the zone
        // watch stands down and the clearance watch takes over.
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::Parked,
            callsign: "November one seven two Sierra Papa".into(),
            runway_ident: "09".into(),
            pending_readback: None,
        });
        // On the ground at the parking first (settles the air/ground edge —
        // and a stand is nobody's runway), then taxi onto the runway
        // uncleared: ordered off.
        let t2 = t0 + sec(400);
        assert!(compliance_tick(36.16, -5.35, 15.0, true, 0.0, true, t2 - sec(10)).is_none());
        let call = compliance_tick(36.15155, -5.342, 20.0, true, 0.0, true, t2).expect("occupancy");
        assert!(!call.crew);
        // Past midfield the position names the 27 end (same rule as the
        // occupancy geometry test).
        assert!(call.line.contains("on runway two seven without clearance"), "{}", call.line);
        assert!(call.line.contains("Exit the runway"), "{}", call.line);

        // Airborne next tick with only a taxi clearance: uncleared takeoff,
        // called once.
        CLEARANCE.lock().unwrap().as_mut().unwrap().state = tower::State::TaxiCleared;
        let t3 = t2 + sec(70);
        let call = compliance_tick(36.152, -5.34, 300.0, false, 70.0, true, t3).expect("takeoff");
        assert!(call.line.contains("not cleared for takeoff"), "{}", call.line);

        // Back onto the runway with no landing clearance: the phone number.
        CLEARANCE.lock().unwrap().as_mut().unwrap().state = tower::State::Inbound;
        let t4 = t3 + sec(70);
        let call = compliance_tick(36.15155, -5.342, 12.0, true, 25.0, true, t4).expect("landing");
        assert!(call.line.contains("number for you to copy"), "{}", call.line);
        // The landing berate is once — but still squatting on the runway a
        // minute later earns the occupancy order (no clearance to be there).
        let call = compliance_tick(36.15155, -5.342, 12.0, true, 10.0, true, t4 + sec(70))
            .expect("occupancy after uncleared landing");
        assert!(call.line.contains("Exit the runway"), "{}", call.line);

        // A LEGAL landing: ClearedToLand grants the vacate grace — sitting on
        // the runway right after touchdown is not an occupancy violation.
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        CLEARANCE.lock().unwrap().as_mut().unwrap().state = tower::State::ClearedToLand;
        assert!(compliance_tick(36.15155, -5.342, 12.0, true, 15.0, true, t4 + sec(200)).is_none());

        *ACTIVE_FIELD.lock().unwrap() = None;
        *ACTIVE_PANEL.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *CLEARANCE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    }

    #[test]
    fn a_cleared_route_crossing_is_not_an_incursion() {
        // The live fault (2026-07-14): the tower's own taxi route crossed a
        // runway, then the compliance watch berated the pilot for being "on
        // the runway without clearance" mid-crossing. A runway-flagged leg of
        // the cleared route makes the occupancy legal: silence while moving,
        // "expedite" when stopped — and the berate still fires when the route
        // has no business on that runway.
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.1512, -5.3554));
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway { width_m: 0.0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1512,
            lon_a: -5.3554,
            lat_b: 36.1517,
            lon_b: -5.3366,
        }];
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::TaxiCleared,
            callsign: "November one seven two Sierra Papa".into(),
            runway_ident: "09".into(),
            pending_readback: None,
        });
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        // The cleared route: south to north straight across the runway at
        // lon -5.342, the crossing leg flagged as runway pavement.
        *TAXI_ROUTE.lock().unwrap() = Some(TaxiWatch {
            path: vec![(36.1490, -5.3420), (36.1530, -5.3420), (36.1540, -5.3420)],
            via: "Alpha".into(),
            seg: vec![("09/27".into(), true), ("A".into(), false)],
            holding_point: "A1".into(),
        });
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        // Settle the air/ground edge off the runway first.
        assert!(compliance_tick(36.1490, -5.3420, 15.0, true, 10.0, true, t0).is_none());
        // Mid-crossing at taxi speed, on the cleared runway leg: silence.
        assert!(
            compliance_tick(36.15155, -5.342, 15.0, true, 10.0, true, t0 + sec(2)).is_none(),
            "a cleared crossing must not be berated"
        );
        // Stopped dead mid-crossing: "expedite", never "without clearance".
        let call = compliance_tick(36.15155, -5.342, 15.0, true, 0.0, true, t0 + sec(4))
            .expect("a stopped crossing earns a call");
        assert!(call.line.contains("expedite crossing runway"), "{}", call.line);
        assert!(!call.line.contains("without clearance"), "{}", call.line);
        // Same spot, but the route STOPS at the hold-short point (it only
        // ends NEAR the runway, never crosses): nosing onto the pavement is
        // still an incursion — the berate stands.
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        TAXI_ROUTE.lock().unwrap().as_mut().unwrap().path =
            vec![(36.1490, -5.3420), (36.1505, -5.3420)];
        assert!(compliance_tick(36.1490, -5.3420, 15.0, true, 10.0, true, t0 + sec(70)).is_none());
        let call = compliance_tick(36.15155, -5.342, 15.0, true, 10.0, true, t0 + sec(72))
            .expect("an uncleared runway entry is still an incursion");
        assert!(call.line.contains("without clearance"), "{}", call.line);

        *ACTIVE_FIELD.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *CLEARANCE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        *TAXI_ROUTE.lock().unwrap() = None;
    }

    #[test]
    fn unnamed_runway_edges_still_name_the_crossing() {
        // apt.dat authors often leave runway network edges unnamed: the
        // planner's crossings list (named edges only) came out empty and the
        // clearance never said "cross runway ..." (the KPDK taxi, 2026-07-14).
        // Geometry backfills the designator from the physical strip.
        let runways = vec![nav::Runway { width_m: 0.0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1512,
            lon_a: -5.3554,
            lat_b: 36.1517,
            lon_b: -5.3366,
        }];
        let mut plan = nav::TaxiRoutePlan {
            via: vec!["A".into()],
            crossings: Vec::new(),
            holding_point: "A1".into(),
            path: vec![(36.1490, -5.3420), (36.1530, -5.3420), (36.1540, -5.3420)],
            seg: vec![(String::new(), true), ("A".into(), false)],
        };
        backfill_unnamed_crossings(&mut plan, &runways);
        assert_eq!(plan.crossings, vec!["09/27".to_string()]);
        // Idempotent: a second pass (or an already-named edge) doesn't dupe.
        backfill_unnamed_crossings(&mut plan, &runways);
        assert_eq!(plan.crossings.len(), 1);
        // A route that never touches the pavement names nothing.
        let mut clear = nav::TaxiRoutePlan {
            via: vec!["A".into()],
            crossings: Vec::new(),
            holding_point: "A1".into(),
            path: vec![(36.1490, -5.3420), (36.1505, -5.3420)],
            seg: vec![(String::new(), false)],
        };
        backfill_unnamed_crossings(&mut clear, &runways);
        assert!(clear.crossings.is_empty());
    }

    #[test]
    fn off_route_stray_gets_a_fresh_route_from_present_position() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_FIELD.lock().unwrap() = Some(("Testfield".into(), 36.0, -5.0));
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::TaxiCleared,
            callsign: "November one seven two Sierra Papa".into(),
            runway_ident: "09".into(),
            pending_readback: None,
        });
        // A tiny network: taxiway A east along 36.0000, then H1 north to the
        // holding point. The cleared route rides all of it.
        let mut nodes = std::collections::HashMap::new();
        nodes.insert(1, (36.0000, -5.0000));
        nodes.insert(2, (36.0000, -4.9990));
        nodes.insert(3, (36.0000, -4.9980));
        nodes.insert(4, (36.0010, -4.9980));
        let edge = |a: i64, b: i64, name: &str| nav::TaxiEdge {
            a,
            b,
            oneway: false,
            runway: false,
            name: name.into(),
        };
        *TAXI_NET.lock().unwrap() = Some(std::sync::Arc::new(nav::TaxiNet {
            nodes,
            edges: vec![edge(1, 2, "A"), edge(2, 3, "A"), edge(3, 4, "H1")],
        }));
        *TAXI_ROUTE.lock().unwrap() = Some(TaxiWatch {
            path: vec![
                (36.0000, -5.0000),
                (36.0000, -4.9990),
                (36.0000, -4.9980),
                (36.0010, -4.9980),
            ],
            via: "Alpha, Hotel one".into(),
            seg: vec![("A".into(), false), ("A".into(), false), ("H1".into(), false)],
            holding_point: "H1".into(),
        });
        let t0 = Instant::now();

        // 100 m north of the route at taxi speed: the challenge carries a
        // FRESH route from where the aircraft IS, not the stale original via
        // (the same via line was spoken seven times at Malaga, 2026-07-13).
        let call = compliance_tick(36.0009, -5.0000, 15.0, true, 15.0, true, t0).expect("stray");
        assert!(!call.crew);
        assert!(call.line.contains("off the cleared taxi route"), "{}", call.line);
        assert!(
            call.line.contains("Taxi to holding point Hotel one via Alpha"),
            "{}",
            call.line
        );
        // The fresh route is republished (the guide rebuilds from it), and
        // it starts AT the aircraft — the walk back to the network is now
        // on-route, so the berate cannot repeat while the pilot complies.
        {
            let g = TAXI_ROUTE.lock().unwrap();
            let r = g.as_ref().expect("route republished");
            assert_eq!(r.path[0], (36.0009, -5.0000));
            assert_eq!(r.holding_point, "H1");
        }
        assert!(
            compliance_tick(36.0009, -5.0000, 15.0, true, 15.0, true, t0 + std::time::Duration::from_secs(50))
                .is_none(),
            "same spot is ON the fresh route"
        );

        *TAXI_NET.lock().unwrap() = None;
        *TAXI_ROUTE.lock().unwrap() = None;
        *ACTIVE_FIELD.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *CLEARANCE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    }

    #[test]
    fn landing_handoff_clears_on_clearance_and_on_touchdown() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.1512, -5.3554));
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::Inbound,
            callsign: "November five five zero Charlie Romeo".into(),
            runway_ident: "27".into(),
            pending_readback: None,
        });
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;

        // Approach handed us off: the coach is armed, still owed while airborne
        // and not yet cleared to land.
        *LANDING_HANDOFF.lock().unwrap() = Some("two seven".into());
        assert!(compliance_tick(36.30, -5.35, 1500.0, false, 90.0, true, t0).is_none());
        assert_eq!(
            crate::party_line::landing_handoff_runway().as_deref(),
            Some("two seven"),
            "still owe the tower a final report"
        );

        // Tower clears the landing → the coach's job is done.
        CLEARANCE.lock().unwrap().as_mut().unwrap().state = tower::State::ClearedToLand;
        compliance_tick(36.20, -5.35, 1000.0, false, 90.0, true, t0 + sec(20));
        assert!(
            crate::party_line::landing_handoff_runway().is_none(),
            "landing clearance clears the handoff"
        );

        // Re-armed but this time we touch down without it: on-ground clears it
        // so the next takeoff isn't nagged by a stale handoff.
        *LANDING_HANDOFF.lock().unwrap() = Some("two seven".into());
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        compliance_tick(36.1512, -5.3554, 12.0, true, 5.0, true, t0 + sec(40));
        assert!(
            crate::party_line::landing_handoff_runway().is_none(),
            "touchdown clears the handoff"
        );

        *ACTIVE_FIELD.lock().unwrap() = None;
        *CLEARANCE.lock().unwrap() = None;
        *LANDING_HANDOFF.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    }

    /// The Jerez sortie, 2026-07-11: holding at E1 (40 m off the 45 m
    /// runway's centerline) earned "exit the runway immediately", and a
    /// one-tick unstick on the takeoff roll earned "you landed without
    /// clearance" mid-departure. Real width + a real flight fix both.
    #[test]
    fn a_holding_point_is_not_the_runway_and_a_skip_is_not_a_landing() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_FIELD.lock().unwrap() = Some(("Jerez".into(), 36.7446, -6.0601));
        // The real LEJR 02/20 100-row: 45 m wide.
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
            width_m: 45.0,
            ident_a: "02".into(),
            ident_b: "20".into(),
            lat_a: 36.7349761,
            lon_a: -6.0648168,
            lat_b: 36.7542941,
            lon_b: -6.0553835,
        }];
        // Geometry first: E1 is beside the pavement, the centerline is on it.
        assert_eq!(on_active_runway(36.753995, -6.056011), None, "holding point captured");
        assert_eq!(on_active_runway(36.7523623, -6.0563268).as_deref(), Some("20"));

        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::TaxiCleared,
            callsign: "November five five zero Charlie Romeo".into(),
            runway_ident: "20".into(),
            pending_readback: None,
        });
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        // Holding short at E1 with only a taxi clearance: no occupancy call.
        assert!(
            compliance_tick(36.753995, -6.056011, 40.0, true, 1.0, true, t0).is_none(),
            "holding SHORT is not ON the runway"
        );

        // Cleared for takeoff, rolling; one-sample unstick, wheels back on.
        CLEARANCE.lock().unwrap().as_mut().unwrap().state = tower::State::TakeoffCleared;
        assert!(compliance_tick(36.7504305, -6.0572701, 40.0, true, 60.0, true, t0 + sec(10)).is_none());
        assert!(compliance_tick(36.7513, -6.0568, 40.0, false, 83.0, true, t0 + sec(11)).is_none());
        assert!(
            compliance_tick(36.7523623, -6.0563268, 40.0, true, 86.0, true, t0 + sec(12)).is_none(),
            "a takeoff-roll skip is not a landing"
        );

        // Airborne for real, then back on the strip a minute later with no
        // landing clearance: THAT is the phone number.
        assert!(compliance_tick(36.7450, -6.0601, 500.0, false, 100.0, true, t0 + sec(15)).is_none());
        let call = compliance_tick(36.7523623, -6.0563268, 40.0, true, 40.0, true, t0 + sec(75))
            .expect("a real uncleared landing still fires");
        assert!(call.line.contains("number for you to copy"), "{}", call.line);

        *ACTIVE_FIELD.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *CLEARANCE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    }

    #[test]
    fn ghosts_speak_airline_telephony_or_nato() {
        assert_eq!(spoken_ghost("IBE3421"), "Iberia three four two one");
        assert_eq!(spoken_ghost("RYR12QT"), "Ryanair one two Quebec Tango");
        // LiveTraffic hands over IATA ids ("BA306", live 2026-07-10).
        assert_eq!(spoken_ghost("BA306"), "Speedbird three zero six");
        assert_eq!(spoken_ghost("U2841"), "Easy eight four one");
        // The Heathrow session's NATO fallbacks, now cast properly.
        assert_eq!(spoken_ghost("EW9460"), "Eurowings niner four six zero");
        assert_eq!(spoken_ghost("ME202"), "Cedar Jet two zero two");
        assert_eq!(spoken_ghost("AC869"), "Air Canada eight six niner");
        assert_eq!(spoken_ghost("VS26"), "Virgin two six");
        // A tail number: full NATO, like any callsign — "BAC12" style tails
        // must not become an airline.
        assert!(spoken_ghost("N172SP").starts_with("November one seven two"));
        // Unknown prefix: NATO fallback, never a guessed airline.
        assert!(spoken_ghost("XYZ123").starts_with("X-ray Yankee Zulu"));
    }

    #[test]
    fn party_line_narrates_each_ghost_once() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *GHOSTS.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway { width_m: 0.0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1512,
            lon_a: -5.3554,
            lat_b: 36.1517,
            lon_b: -5.3366,
        }];
        let (flat, flon, felev) = (36.1512, -5.3554, 11.0);
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        // An arrival descending on final, 2 nm out at 700 ft.
        let approach = |ele: f64, dist_deg: f64| TcasTarget {
            callsign: "IBE3421".into(),
            lat: 36.1512,
            lon: -5.3554 - dist_deg,
            ele_ft: ele,
            on_ground: false,
            vs_fpm: -600.0,
        };
        // First sight seeds the memory — no call without a trend.
        assert!(chatter_tick(&[approach(900.0, 0.04)], "Gibraltar", flat, flon, felev, None, t0).is_none());
        // Descending on the next pass: the full exchange — the ghost's final
        // call, the clearance, the readback — aimed at 09.
        let ex = chatter_tick(&[approach(700.0, 0.03)], "Gibraltar", flat, flon, felev, None, t0 + sec(5))
            .expect("landing exchange");
        assert_eq!(ex.lines.len(), 3);
        assert!(ex.lines[0].ghost);
        assert!(
            ex.lines[0].text.contains("Gibraltar Tower, Iberia three four two one, final runway zero niner"),
            "{}",
            ex.lines[0].text
        );
        assert!(!ex.lines[1].ghost);
        assert!(ex.lines[1].text.contains("runway zero niner, cleared to land"), "{}", ex.lines[1].text);
        assert!(ex.lines[2].ghost);
        assert!(ex.lines[2].text.starts_with("Cleared to land"), "{}", ex.lines[2].text);
        assert_eq!(ex.ghost_seed, crate::speaker::station_voice_seed("IBE3421"));
        // Once per ghost — the rest of the approach stays quiet.
        assert!(chatter_tick(&[approach(500.0, 0.02)], "Gibraltar", flat, flon, felev, None, t0 + sec(10)).is_none());

        // A second arrival aiming at a strip the PLAYER occupies: "continue
        // approach", never a narrated clearance over the player's head —
        // and only once.
        let inbound2 = |ele: f64, dist_deg: f64| TcasTarget {
            callsign: "BA435".into(),
            lat: 36.1512,
            lon: -5.3554 - dist_deg,
            ele_ft: ele,
            on_ground: false,
            vs_fpm: -600.0,
        };
        assert!(chatter_tick(&[inbound2(900.0, 0.05)], "Gibraltar", flat, flon, felev, Some("27"), t0 + sec(12)).is_none());
        let ex = chatter_tick(&[inbound2(700.0, 0.04)], "Gibraltar", flat, flon, felev, Some("27"), t0 + sec(14))
            .expect("continue exchange");
        assert_eq!(ex.lines.len(), 2);
        assert!(ex.lines[1].text.contains("continue approach"), "{}", ex.lines[1].text);
        assert!(!ex.lines[1].text.contains("cleared to land"), "{}", ex.lines[1].text);
        assert!(chatter_tick(&[inbound2(600.0, 0.03)], "Gibraltar", flat, flon, felev, Some("27"), t0 + sec(16)).is_none());
        // Player vacates: the clearance can now come.
        let ex = chatter_tick(&[inbound2(500.0, 0.02)], "Gibraltar", flat, flon, felev, None, t0 + sec(18))
            .expect("clearance after vacating");
        assert!(ex.lines[1].text.contains("cleared to land"), "{}", ex.lines[1].text);

        // A departure: ground last pass, airborne now → cleared for takeoff.
        let ground = TcasTarget {
            callsign: "VLG88AB".into(),
            lat: 36.1514,
            lon: -5.345,
            ele_ft: 11.0,
            on_ground: true,
            vs_fpm: 0.0,
        };
        let rotated = TcasTarget {
            callsign: "VLG88AB".into(),
            lat: 36.1514,
            lon: -5.345,
            ele_ft: 100.0,
            on_ground: false,
            vs_fpm: 0.0,
        };
        assert!(chatter_tick(&[ground], "Gibraltar", flat, flon, felev, None, t0 + sec(15)).is_none());
        let ex = chatter_tick(&[rotated], "Gibraltar", flat, flon, felev, None, t0 + sec(20))
            .expect("takeoff exchange");
        assert_eq!(ex.lines.len(), 2);
        assert!(!ex.lines[0].ghost);
        assert!(ex.lines[0].text.contains("Vueling"), "{}", ex.lines[0].text);
        assert!(ex.lines[0].text.contains("cleared for takeoff"), "{}", ex.lines[0].text);
        assert!(ex.lines[1].ghost);
        assert!(ex.lines[1].text.starts_with("Cleared for takeoff"), "{}", ex.lines[1].text);

        *GHOSTS.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    }

    #[test]
    fn go_around_ordered_when_a_ghost_takes_the_runway() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway { width_m: 0.0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1512,
            lon_a: -5.3554,
            lat_b: 36.1517,
            lon_b: -5.3366,
        }];
        *GO_AROUND_LAST.lock().unwrap() = None;
        *PENDING_GO_AROUND.lock().unwrap() = false;
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::ClearedToLand,
            callsign: "November one seven two Sierra Papa".into(),
            runway_ident: "09".into(),
            pending_readback: None,
        });
        let ghost_on_runway = TcasTarget {
            callsign: "IBE3421".into(),
            lat: 36.15155,
            lon: -5.342,
            ele_ft: 11.0,
            on_ground: true,
            vs_fpm: 0.0,
        };
        let t0 = Instant::now();
        // Cleared to land, short final, runway occupied: the order, by name.
        let call = go_around_tick(std::slice::from_ref(&ghost_on_runway), 400.0, false, true, t0)
            .expect("go-around order");
        assert!(!call.crew);
        assert!(call.line.contains("go around, traffic on the runway"), "{}", call.line);
        // The order hands the FSM sync to the main loop, like the hold release.
        assert!(*PENDING_GO_AROUND.lock().unwrap(), "order must arm the FSM sync");
        *PENDING_GO_AROUND.lock().unwrap() = false;
        // Cooldown: not machine-gunned.
        assert!(go_around_tick(std::slice::from_ref(&ghost_on_runway), 350.0, false, true, t0).is_none());
        assert!(!*PENDING_GO_AROUND.lock().unwrap(), "a swallowed repeat must not re-arm");
        // Not tuned: the CREW calls it — and the tower's cancel is just as real.
        *GO_AROUND_LAST.lock().unwrap() = None;
        let call = go_around_tick(std::slice::from_ref(&ghost_on_runway), 400.0, false, false, t0)
            .expect("crew go-around");
        assert!(call.crew);
        assert!(*PENDING_GO_AROUND.lock().unwrap(), "crew-voiced order still voids the clearance");
        *PENDING_GO_AROUND.lock().unwrap() = false;
        // No landing clearance in effect: not the tower's call to cancel.
        *GO_AROUND_LAST.lock().unwrap() = None;
        CLEARANCE.lock().unwrap().as_mut().unwrap().state = tower::State::Inbound;
        assert!(go_around_tick(std::slice::from_ref(&ghost_on_runway), 400.0, false, true, t0).is_none());
        // Too high: the conflict is not yet real.
        CLEARANCE.lock().unwrap().as_mut().unwrap().state = tower::State::ClearedToLand;
        assert!(go_around_tick(std::slice::from_ref(&ghost_on_runway), 2000.0, false, true, t0).is_none());
        // A ghost on the PARALLEL strip is not our conflict: no cancel.
        ACTIVE_RUNWAYS.lock().unwrap().push(nav::Runway { width_m: 0.0,
            ident_a: "09L".into(),
            ident_b: "27R".into(),
            lat_a: 36.1692,
            lon_a: -5.3554,
            lat_b: 36.1697,
            lon_b: -5.3366,
        });
        let ghost_on_parallel = TcasTarget {
            callsign: "IBE9".into(),
            lat: 36.16955,
            lon: -5.342,
            ele_ft: 11.0,
            on_ground: true,
            vs_fpm: 0.0,
        };
        *GO_AROUND_LAST.lock().unwrap() = None;
        assert!(
            go_around_tick(std::slice::from_ref(&ghost_on_parallel), 400.0, false, true, t0).is_none(),
            "parallel-runway squatter must not cancel our clearance"
        );

        *CLEARANCE.lock().unwrap() = None;
        *GO_AROUND_LAST.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    }

    #[test]
    fn a_monitor_go_around_voids_the_clearance_and_flips_the_coach() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_PANEL.lock().unwrap() =
            Some(("Gibraltar".into(), 11.0, vec![("tower".to_string(), 131200_i64)]));
        *ATIS.lock().unwrap() = None;
        *TAXI_ROUTE.lock().unwrap() = None;
        *VECTORS.lock().unwrap() = None;
        *LANDING_HANDOFF.lock().unwrap() = None;
        let cs = "November one seven two Sierra Papa";
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::ClearedToLand,
            callsign: cs.into(),
            runway_ident: "27".into(),
            pending_readback: Some(format!("Cleared to land runway two seven, {cs}.")),
        });
        let mut st = lxgb();
        st.twr.state = tower::State::ClearedToLand;
        // Before the sync: the coach suggests the landing readback — the
        // exact contradiction from the 2026-07-14 tape.
        let s = suggest_call(cs, false, Some(1.0), None).unwrap();
        assert!(s.call.as_deref().unwrap().contains("Cleared to land"), "{:?}", s.call);

        st.apply_monitor_go_around();

        assert_eq!(st.twr.state, tower::State::Inbound, "clearance void in the FSM");
        assert!(st.twr.pending().is_none(), "the owed readback dies with it");
        assert_eq!(
            CLEARANCE.lock().unwrap().as_ref().unwrap().state,
            tower::State::Inbound
        );
        // The coach's next line is the acknowledgment, urgent, and it parses.
        let s = suggest_call(cs, false, Some(1.0), None).unwrap();
        let call = s.call.expect("a call");
        assert!(call.contains("Going around"), "{call}");
        assert!(s.urgent, "the go-around ack is the say-now line");
        assert_eq!(tower::parse_intent(&call), tower::Intent::GoAround);

        *CLEARANCE.lock().unwrap() = None;
        *ACTIVE_PANEL.lock().unwrap() = None;
    }

    #[test]
    fn held_takeoff_clearance_releases_when_the_final_stays_clear() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *TAKEOFF_HOLD.lock().unwrap() = Some(TakeoffHold {
            callsign: "November one seven two Sierra Papa".into(),
            runway: "zero niner".into(),
            runway_ident: "09".into(),
            clear_since: None,
        });
        *PENDING_TAKEOFF_CLEARANCE.lock().unwrap() = false;
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::TaxiCleared,
            callsign: "November one seven two Sierra Papa".into(),
            runway_ident: "09".into(),
            pending_readback: None,
        });
        // Our strip: 09/27. The watches only care about THIS pavement.
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway { width_m: 0.0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1512,
            lon_a: -5.3554,
            lat_b: 36.1517,
            lon_b: -5.3366,
        }];
        let elev = 11.0;
        // An arrival in OUR corridor, 2 nm out at ~400 ft; a PARALLEL
        // arrival the same distance out but 2 km abeam; a ghost parked on
        // our strip.
        let ours = TcasTarget {
            callsign: "BA1".into(),
            lat: 36.1510,
            lon: -5.3554 - 0.037,
            ele_ft: 411.0,
            on_ground: false,
            vs_fpm: -600.0,
        };
        let parallel = TcasTarget { callsign: "BA2".into(), lat: 36.1690, ..ours.clone() };
        let squatter = TcasTarget {
            callsign: "BA3".into(),
            lat: 36.15155,
            lon: -5.342,
            ele_ft: elev,
            on_ground: true,
            vs_fpm: 0.0,
        };
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        // Traffic on OUR final: nothing, and the clear-timer resets.
        assert!(takeoff_hold_tick(std::slice::from_ref(&ours), elev, true, t0).is_none());
        // Only the PARALLEL runway's arrival: not our traffic — the timer
        // runs (the runway-blind check held a Heathrow 09R departure forever).
        assert!(takeoff_hold_tick(std::slice::from_ref(&parallel), elev, true, t0 + sec(1)).is_none());
        // A ghost sitting on our strip resets it again.
        assert!(takeoff_hold_tick(std::slice::from_ref(&squatter), elev, true, t0 + sec(5)).is_none());
        assert!(takeoff_hold_tick(std::slice::from_ref(&parallel), elev, true, t0 + sec(6)).is_none());
        // Our runway clear and stayed clear (parallel stream still landing):
        // the tower comes back unprompted.
        let line = takeoff_hold_tick(std::slice::from_ref(&parallel), elev, true, t0 + sec(25))
            .expect("release");
        assert!(line.contains("traffic clear"), "{line}");
        assert!(line.contains("Runway zero niner, cleared for takeoff"), "{line}");
        // The compliance picture moved with it, and the FSM sync is armed.
        assert_eq!(
            CLEARANCE.lock().unwrap().as_ref().unwrap().state,
            tower::State::TakeoffCleared
        );
        assert!(*PENDING_TAKEOFF_CLEARANCE.lock().unwrap());
        // One shot: the hold is consumed.
        assert!(takeoff_hold_tick(&[], elev, true, t0 + sec(60)).is_none());

        // Airborne, a held clearance is VOID, not released: the pilot who
        // departed through the cancel got "traffic clear, cleared for
        // takeoff" two minutes into the climb (Gatwick, 2026-07-13).
        *PENDING_TAKEOFF_CLEARANCE.lock().unwrap() = false;
        *TAKEOFF_HOLD.lock().unwrap() = Some(TakeoffHold {
            callsign: "November one seven two Sierra Papa".into(),
            runway: "zero niner".into(),
            runway_ident: "09".into(),
            clear_since: Some(t0),
        });
        assert!(takeoff_hold_tick(&[], elev, false, t0 + sec(120)).is_none());
        assert!(TAKEOFF_HOLD.lock().unwrap().is_none(), "airborne must void the hold");
        assert!(
            takeoff_hold_tick(&[], elev, true, t0 + sec(180)).is_none(),
            "a voided hold never releases, even back on the ground"
        );
        assert!(!*PENDING_TAKEOFF_CLEARANCE.lock().unwrap());

        *CLEARANCE.lock().unwrap() = None;
        *PENDING_TAKEOFF_CLEARANCE.lock().unwrap() = false;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    }

    #[test]
    fn incursion_traffic_check_ignores_the_other_runways_stream() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_PANEL.lock().unwrap() = Some(("Malaga".into(), 11.0, Vec::new()));
        // Two strips 2 km apart, Malaga-style: we hold 12, arrivals land 31.
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![
            nav::Runway { width_m: 0.0,
                ident_a: "12".into(),
                ident_b: "30".into(),
                lat_a: 36.1512,
                lon_a: -5.3554,
                lat_b: 36.1517,
                lon_b: -5.3366,
            },
            nav::Runway { width_m: 0.0,
                ident_a: "13".into(),
                ident_b: "31".into(),
                lat_a: 36.1692,
                lon_a: -5.3554,
                lat_b: 36.1697,
                lon_b: -5.3366,
            },
        ];
        // An arrival ~2 nm out at ~400 ft in the OTHER strip's corridor: the
        // runway-blind check shouted "exit the runway, expedite" at a cleared
        // runway 12 departure for exactly this picture (Malaga, 2026-07-12).
        let other = TcasTarget {
            callsign: "EZY5701".into(),
            lat: 36.1690,
            lon: -5.3554 - 0.037,
            ele_ft: 411.0,
            on_ground: false,
            vs_fpm: -600.0,
        };
        assert!(
            inbound_final_for_strip(std::slice::from_ref(&other), "12").is_none(),
            "the other runway's stream is not our traffic"
        );
        // The same arrival in OUR corridor earns the shout.
        let ours = TcasTarget { lat: 36.1510, ..other.clone() };
        let nm = inbound_final_for_strip(std::slice::from_ref(&ours), "12").expect("our final");
        assert!((1.0..3.5).contains(&nm), "{nm}");
        // The same position CLIMBING is a departure leaving, not an arrival:
        // the ghost that lifted off runway 30 read as "traffic on short
        // final" to the runway 12 watch (Malaga, 2026-07-12).
        let climber = TcasTarget { vs_fpm: 1200.0, ..ours.clone() };
        assert!(
            inbound_final_for_strip(std::slice::from_ref(&climber), "12").is_none(),
            "a climbing departure is not final traffic"
        );
        // Unknown vertical speed (unfed array) stays counted — fail toward
        // caution.
        let level = TcasTarget { vs_fpm: 0.0, ..ours.clone() };
        assert!(inbound_final_for_strip(std::slice::from_ref(&level), "12").is_some());
        // A strip the field doesn't have: never fires.
        assert!(inbound_final_for_strip(std::slice::from_ref(&ours), "99").is_none());

        *ACTIVE_PANEL.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    }

    #[test]
    fn the_coach_suggests_the_parsers_happy_path() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_PANEL.lock().unwrap() =
            Some(("Gibraltar".into(), 11.0, vec![("tower".to_string(), 131200_i64)]));
        *CLEARANCE.lock().unwrap() = None;
        *ATIS.lock().unwrap() = None;
        *TAXI_ROUTE.lock().unwrap() = None;
        *VECTORS.lock().unwrap() = None;
        *LANDING_HANDOFF.lock().unwrap() = None;
        let cs = "November one seven two Sierra Papa";
        // The transmission itself, for the parse checks below.
        let call = |s: Suggestion| s.call.expect("a call");

        // Cold on the ground: the full initial call.
        let s = call(suggest_call(cs, true, Some(0.5), None).unwrap());
        assert!(s.starts_with("Gibraltar Tower"), "{s}");
        assert!(s.contains("request departure clearance"), "{s}");
        // Cold in the air: the inbound call, with position.
        let s = call(suggest_call(cs, false, Some(8.0), None).unwrap());
        assert!(s.contains("8 miles out"), "{s}");
        assert!(s.contains("inbound"), "{s}");
        // Every suggestion must parse to a routable intent — the coach IS
        // the parser's happy path.
        assert_eq!(tower::parse_intent(&s), tower::Intent::RequestLanding { touch_and_go: false });

        // Walk the departure states.
        let set = |state: tower::State| {
            *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
                state,
                callsign: cs.into(),
                runway_ident: "09".into(),
                pending_readback: None,
            });
        };
        set(tower::State::Parked);
        let sug = suggest_call(cs, true, Some(0.5), None).unwrap();
        assert!(!sug.arrival, "parked = departure side");
        let s = call(sug);
        assert!(s.contains("request taxi"), "{s}");
        assert_eq!(tower::parse_intent(&s), tower::Intent::RequestTaxi);
        set(tower::State::TaxiCleared);
        // No taxi network: no invented holding point name.
        let s = call(suggest_call(cs, true, Some(0.5), None).unwrap());
        assert!(s.contains("ready for departure"), "{s}");
        assert!(!s.contains("holding point"), "no route, no invented name: {s}");
        assert_eq!(tower::parse_intent(&s), tower::Intent::RequestTakeoff);
        // With the cleared route published: the REAL holding point + runway.
        *TAXI_ROUTE.lock().unwrap() = Some(TaxiWatch {
            path: vec![(36.16, -5.36), (36.16, -5.35)],
            via: "Alpha".into(),
            seg: vec![("A".into(), false)],
            holding_point: "A1".into(),
        });
        let s = call(suggest_call(cs, true, Some(0.5), None).unwrap());
        assert!(s.contains("holding point Alpha one"), "{s}");
        assert!(s.contains("runway zero niner"), "{s}");
        assert_eq!(tower::parse_intent(&s), tower::Intent::RequestTakeoff);
        *TAXI_ROUTE.lock().unwrap() = None;
        set(tower::State::TakeoffCleared);
        let s = call(suggest_call(cs, true, Some(0.5), None).unwrap());
        assert!(s.contains("Cleared for takeoff runway zero niner"), "{s}");
        assert!(tower::is_readback(&s.to_lowercase()), "readback: {s}");
        // Arrival side.
        set(tower::State::Inbound);
        let sug = suggest_call(cs, false, Some(2.0), None).unwrap();
        assert!(sug.arrival);
        let s = call(sug);
        assert!(s.contains("final runway zero niner"), "{s}");
        assert_eq!(tower::parse_intent(&s), tower::Intent::ReportFinal);
        set(tower::State::ClearedToLand);
        let s = call(suggest_call(cs, true, Some(0.2), None).unwrap());
        assert!(s.contains("taxi to parking"), "{s}");
        assert_eq!(tower::parse_intent(&s), tower::Intent::RequestTaxiIn);

        // No station: nothing to suggest.
        *ACTIVE_PANEL.lock().unwrap() = None;
        assert!(suggest_call(cs, true, None, None).is_none());
        *CLEARANCE.lock().unwrap() = None;

        // The question forms that summon the coach.
        for q in [
            "what do i say",
            "what should i say to the tower",
            "how do i ask for taxi",
            "what's the call",
            "help me with the radio",
        ] {
            assert!(coach_query(q), "{q:?} should reach the coach");
        }
        assert!(!coach_query("flaps ten"));
        assert!(!coach_query("what frequencies are here"));
    }

    #[test]
    fn the_coach_leads_with_the_prerequisite() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_PANEL.lock().unwrap() =
            Some(("Gibraltar".into(), 11.0, vec![("tower".to_string(), 131200_i64)]));
        *CLEARANCE.lock().unwrap() = None;
        *ATIS.lock().unwrap() = None;
        *VECTORS.lock().unwrap() = None;
        *LANDING_HANDOFF.lock().unwrap() = None;
        let cs = "November one seven two Sierra Papa";

        // COM1 parked on another frequency: the call can't be heard — tune
        // first. On frequency (or unknown): no nag.
        let s = suggest_call(cs, true, Some(0.5), Some(122800)).unwrap();
        assert_eq!(
            s.first.as_deref(),
            Some("First tune tower, one three one decimal two."),
            "{:?}",
            s.first
        );
        assert!(s.call.is_some());
        let s = suggest_call(cs, true, Some(0.5), Some(131200)).unwrap();
        assert!(s.first.is_none(), "{:?}", s.first);

        // An ATIS that never cut an edition = nobody listened: copy it
        // before the first call, and it outranks the tune nag (it IS the
        // next tuning action).
        *ATIS.lock().unwrap() = Some(AtisState {
            khz: 118500,
            airport: "Gibraltar".into(),
            runways: Vec::new(),
            letter: 'A',
            edition: None,
            last_spoken: None,
        });
        let s = suggest_call(cs, true, Some(0.5), Some(122800)).unwrap();
        assert_eq!(
            s.first.as_deref(),
            Some("First copy the ATIS on one one eight decimal five."),
            "{:?}",
            s.first
        );
        // Once in contact the tower's QNH covers you: no more ATIS nag.
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::Parked,
            callsign: cs.into(),
            runway_ident: String::new(),
            pending_readback: None,
        });
        let s = suggest_call(cs, true, Some(0.5), Some(131200)).unwrap();
        assert!(s.first.is_none(), "{:?}", s.first);

        *ATIS.lock().unwrap() = None;
        *CLEARANCE.lock().unwrap() = None;
        *ACTIVE_PANEL.lock().unwrap() = None;
    }

    #[test]
    fn the_coach_covers_the_approach_phase() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_PANEL.lock().unwrap() =
            Some(("Gibraltar".into(), 11.0, vec![("tower".to_string(), 131200_i64)]));
        *CLEARANCE.lock().unwrap() = None;
        *ATIS.lock().unwrap() = None;
        *LANDING_HANDOFF.lock().unwrap() = None;
        let cs = "November one seven two Sierra Papa";
        let plan = |ils: bool, cleared: bool| VectorPlan {
            callsign: cs.into(),
            thr_lat: 36.15,
            thr_lon: -5.35,
            course_deg: 90.0,
            runway_spoken: "zero niner".into(),
            tower_spoken: Some("one three one decimal two".into()),
            target_alt_ft: 2000.0,
            ils,
            join_nm: 6.0,
            cleared,
            dialect: Dialect::Icao,
            last_tx: None,
            last_heading: None,
        };

        // Under vectors, nothing owed: coach the silence, not a fake call.
        *VECTORS.lock().unwrap() = Some(plan(true, false));
        let s = suggest_call(cs, false, Some(12.0), None).unwrap();
        assert!(s.first.as_deref().unwrap().contains("No call owed"), "{:?}", s.first);
        assert!(s.call.is_none());
        assert!(s.arrival);
        // Cleared for the ILS on the intercept: the "established" report is
        // the one call approach waits on — and it parses as the final report.
        *VECTORS.lock().unwrap() = Some(plan(true, true));
        let s = suggest_call(cs, false, Some(10.0), None).unwrap();
        assert!(s.first.as_deref().unwrap().contains("established"), "{:?}", s.first);
        let c = s.call.expect("a call");
        assert!(c.contains("established"), "{c}");
        assert_eq!(tower::parse_intent(&c), tower::Intent::ReportFinal);
        *VECTORS.lock().unwrap() = None;

        // Handed to the tower with an approach clearance: the "final" report
        // is owed — same fact the monitor's voice coach reads.
        *LANDING_HANDOFF.lock().unwrap() = Some("zero niner".into());
        let s = suggest_call(cs, false, Some(6.0), None).unwrap();
        let c = s.call.expect("a call");
        assert!(c.contains("final runway zero niner"), "{c}");
        assert_eq!(tower::parse_intent(&c), tower::Intent::ReportFinal);
        *LANDING_HANDOFF.lock().unwrap() = None;
        *ACTIVE_PANEL.lock().unwrap() = None;
    }

    #[test]
    fn spoken_airport_uses_the_radio_name() {
        // Gazetteer entries shorten to what a controller actually keys.
        assert_eq!(spoken_airport("Badajoz- Talavera La Real"), "Badajoz");
        assert_eq!(spoken_airport("Merida - Royanejos"), "Merida");
        // Plain names pass through untouched.
        assert_eq!(spoken_airport("Gibraltar"), "Gibraltar");
        assert_eq!(spoken_airport("Ciudad Real Intl"), "Ciudad Real Intl");
        // Degenerate input survives.
        assert_eq!(spoken_airport("- Somewhere"), "Somewhere");
        assert_eq!(spoken_airport(""), "");
    }

    #[test]
    fn vhf_range_is_line_of_sight() {
        // On the ramp the station's own mast still buys a few miles — enough
        // to hear the field you're parked at, not one 20 nm out.
        let ground = vhf_range_nm(0.0);
        assert!((5.0..12.0).contains(&ground), "{ground}");
        // Range grows with height: at 5,000 ft above the field, most of a
        // sector is reachable; 200 nm (Extremadura → Gibraltar) is not.
        let cruise = vhf_range_nm(5000.0);
        assert!((80.0..110.0).contains(&cruise), "{cruise}");
        assert!(vhf_range_nm(20000.0) < 200.0);
        // Below the field's elevation nothing goes negative.
        assert!(vhf_range_nm(-500.0) >= ground - 1e-9);
    }

    fn lxgb() -> Station {
        Station {
            apt: nav::AirportInfo {
                name: "Gibraltar".into(),
                elev_ft: 11.0,
                runways: Vec::new(),
                freqs: vec![
                    nav::AtcFreq {
                        kind: nav::FREQ_TOWER,
                        khz: 131200,
                        name: "Tower Freq".into(),
                    },
                    nav::AtcFreq {
                        kind: nav::FREQ_APPROACH,
                        khz: 122800,
                        name: "Approach".into(),
                    },
                ],
            },
            tower_khz: 131200,
            approach_khz: Some(122800),
            departure_khz: None,
            twr: tower::Tower::new(),
            runway_in_use: None,
            dep_checked_in: false,
            pilot_info: None,
            assigned_squawk: None,
            planned_route: None,
            enforce_readback: true,
            dialect: Dialect::Icao,
        }
    }

    /// `ATIS` is a process-wide static, same discipline as `SIDE_STATE_LOCK`.
    /// Station side state (`CLEARANCE`/`HANDOFF`/`ATIS`/`REGION`/
    /// `TAKEOFF_HOLD`/`TAXI_ROUTE`/...) is process statics, and the reply
    /// path writes them on EVERY call — so every test that drives a station
    /// or asserts on the statics holds this ONE lock. Three finer locks
    /// (ATIS/REGION/HANDOFF) raced each other through `CLEARANCE`: the coach
    /// test nulled it mid-flight under the readback test (2026-07-11).
    static SIDE_STATE_LOCK: Mutex<()> = Mutex::new(());

    fn wx(wind_deg: f64, wind_kts: f64, qnh: f64) -> AtisWx {
        AtisWx {
            zulu_sec: 12.0 * 3600.0,
            wind_from_deg: wind_deg,
            wind_kts,
            visibility_sm: 10.0,
            qnh_hpa: qnh,
            oat_c: 22.0,
        }
    }

    #[test]
    fn atis_loops_while_tuned_and_cuts_editions_on_real_change() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ATIS.lock().unwrap() = Some(AtisState {
            khz: 118500,
            airport: "Gibraltar".into(),
            runways: Vec::new(),
            letter: 'A',
            edition: None,
            last_spoken: None,
        });
        let t0 = Instant::now();
        // Off frequency: silence, no edition.
        assert!(atis_tick(131200, &wx(250.0, 12.0, 1018.0), t0).is_none());
        assert_eq!(atis_current_letter(), None, "nothing aired yet");
        // Tuned: the broadcast, information Alpha.
        let r = atis_tick(118500, &wx(250.0, 12.0, 1018.0), t0).expect("broadcast");
        assert!(r.contains("information Alpha"), "{r}");
        assert!(r.contains("QNH one zero one eight"), "{r}");
        assert_eq!(atis_current_letter(), Some('A'));
        // Still tuned, seconds later: the loop is between repeats — silence.
        assert!(atis_tick(118500, &wx(250.0, 12.0, 1018.0), t0 + std::time::Duration::from_secs(10)).is_none());
        // Repeat due, weather unchanged: SAME letter.
        let r = atis_tick(118500, &wx(251.0, 13.0, 1018.2), t0 + std::time::Duration::from_secs(50))
            .expect("repeat");
        assert!(r.contains("information Alpha"), "wobble must not burn a letter: {r}");
        // A real pressure change cuts information Bravo.
        let r = atis_tick(118500, &wx(250.0, 12.0, 1021.0), t0 + std::time::Duration::from_secs(100))
            .expect("new edition");
        assert!(r.contains("information Bravo"), "{r}");
        assert_eq!(atis_current_letter(), Some('B'));
        *ATIS.lock().unwrap() = None;
    }

    fn region_apt(name: &str, lat: f64, lon: f64) -> RegionApt {
        RegionApt {
            icao: name.get(0..4).unwrap_or("XXXX").to_uppercase(),
            apt: nav::AirportInfo {
                name: name.into(),
                elev_ft: 0.0,
                runways: Vec::new(),
                freqs: vec![nav::AtcFreq {
                    kind: nav::FREQ_TOWER,
                    khz: 118100,
                    name: "Tower".into(),
                }],
            },
            lat,
            lon,
        }
    }

    #[test]
    fn transmission_on_another_fields_freq_arms_the_swap() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut badajoz = region_apt("Badajoz", 38.89, -6.82);
        badajoz.apt.freqs[0].khz = 122100;
        *REGION.lock().unwrap() = vec![region_apt("Gibraltar", 36.15, -5.35), badajoz];
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.15, -5.35));
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        let now = Instant::now();
        // Parked in Extremadura at ~1,000 ft MSL, COM1 on Badajoz tower
        // (22 nm, inside VHF range): the swap arms for Badajoz.
        arm_station_for_freq(122100, 38.74, -6.40, 1075.0, now);
        let armed = PENDING_STATION.lock().unwrap().take().expect("swap armed");
        assert_eq!(armed.1.name, "Badajoz");
        assert!(LAST_REGION_SWITCH.lock().unwrap().is_some(), "region cooldown stamped");
        // A frequency nobody in range publishes: nothing arms.
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        arm_station_for_freq(999000, 38.74, -6.40, 1075.0, now);
        assert!(PENDING_STATION.lock().unwrap().is_none());
        // The ACTIVE station's own frequency never re-arms a swap to itself.
        *ACTIVE_FIELD.lock().unwrap() = Some(("Badajoz".into(), 38.89, -6.82));
        arm_station_for_freq(122100, 38.74, -6.40, 1075.0, now);
        assert!(PENDING_STATION.lock().unwrap().is_none());
        // Same tune from the ramp at Gibraltar (200+ nm away): out of VHF
        // range, no swap — you can't raise a tower you can't reach.
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.15, -5.35));
        arm_station_for_freq(122100, 36.15, -5.35, 15.0, now);
        assert!(PENDING_STATION.lock().unwrap().is_none());
        *REGION.lock().unwrap() = Vec::new();
        *ACTIVE_FIELD.lock().unwrap() = None;
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
    }

    #[test]
    fn unnamed_gateway_tower_rows_are_not_staffed() {
        // LEMY: an ultralight strip whose air-to-air frequency the Gateway
        // typed as "1054 130125 unnamed entity" — a frequency, not a tower.
        let apt = nav::AirportInfo {
            name: "Merida - Royanejos".into(),
            elev_ft: 883.0,
            runways: Vec::new(),
            freqs: vec![nav::AtcFreq {
                kind: nav::FREQ_TOWER,
                khz: 130125,
                name: "unnamed entity".into(),
            }],
        };
        assert!(apt.staffed_tower().is_none());
        // A real tower row (Badajoz: "TWR CIVIL") stays staffed.
        let apt = nav::AirportInfo {
            name: "Badajoz- Talavera La Real".into(),
            elev_ft: 607.0,
            runways: Vec::new(),
            freqs: vec![nav::AtcFreq {
                kind: nav::FREQ_TOWER,
                khz: 122100,
                name: "TWR CIVIL".into(),
            }],
        };
        assert!(apt.staffed_tower().is_some());
    }

    #[test]
    fn region_hands_off_to_the_decisively_nearer_tower() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // Gibraltar-ish and Malaga-ish, ~50 nm apart along a degree of longitude.
        *REGION.lock().unwrap() = vec![
            region_apt("Gibraltar", 36.15, -5.35),
            region_apt("Malaga", 36.67, -4.50),
        ];
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.15, -5.35));
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        // The fresh-boot branch reads the contact state: start from none,
        // whatever an earlier test left behind.
        *CLEARANCE.lock().unwrap() = None;
        *SQUAWK_WATCH.lock().unwrap() = None;
        let t0 = Instant::now();
        // On the ground at Gibraltar (the active field): never.
        assert!(region_tick(36.15, -5.35, true, t0).is_none());
        // Just airborne, Gibraltar still much nearer: quiet.
        assert!(region_tick(36.2, -5.3, false, t0).is_none());
        // Near the midpoint, Malaga slightly nearer but inside the margin:
        // still quiet — hysteresis, no flapping between two fields.
        assert!(region_tick(36.42, -4.92, false, t0).is_none());
        // On the GROUND near the midpoint (not AT any field): also quiet —
        // a ground relocation needs you physically at the new field.
        assert!(region_tick(36.42, -4.92, true, t0).is_none());
        // Decisively closer to Malaga, airborne — but NOBODY here has talked
        // to Gibraltar this session (fresh boot mid-flight): the crew
        // orients, no controller terminating a service it never gave.
        let sw = region_tick(36.6, -4.6, false, t0).expect("fresh-boot fix");
        assert!(sw.crew, "no contact yet: the crew speaks, not a controller");
        assert!(sw.line.contains("Closest field is Malaga"), "{}", sw.line);
        assert!(!sw.line.contains("Radar services"), "{}", sw.line);
        assert!(PENDING_STATION.lock().unwrap().is_some(), "swap armed for the main loop");
        // While the swap waits to be consumed: silence, no re-arm.
        assert!(region_tick(36.6, -4.6, false, t0).is_none());
        // Same geometry WITH two-way contact on record: the controller's
        // proper handoff.
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.15, -5.35));
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::TaxiCleared,
            callsign: "November one seven two Sierra Papa".into(),
            runway_ident: "09".into(),
            pending_readback: None,
        });
        let sw = region_tick(36.6, -4.6, false, t0).expect("handoff");
        assert!(!sw.crew, "with a service on record the controller hands off");
        assert!(sw.line.contains("Contact Malaga Tower"), "{}", sw.line);
        assert!(sw.line.contains("one one eight decimal one"), "{}", sw.line);
        *CLEARANCE.lock().unwrap() = None;
        // Ground relocation: parked AT Malaga with Gibraltar still active
        // (the "started at LERL, Gibraltar answered" case) — the CREW notes
        // the local tower, no radar theatre.
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        let sw = region_tick(36.67, -4.50, true, t0).expect("ground relocation");
        assert!(sw.crew, "ground switch is the crew's note");
        assert!(sw.line.contains("We're at Malaga"), "{}", sw.line);
        run_borrowing_strip_case(); // same SIDE_STATE_LOCK scope, statics owned here
        *REGION.lock().unwrap() = Vec::new();
        *ACTIVE_FIELD.lock().unwrap() = None;
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
    }

    /// The Trebujena trap (flown 2026-07-11), run inside the region test's
    /// lock scope: a tower-only grass strip 12 nm off the nose whose apt.dat
    /// rows borrow "Jerez Tower" / "Seville Approach", with the actual
    /// destination (ATIS, ground, its own approach) 16 nm out. Raw-distance
    /// nearest picked the strip, and its controller then vectored the Jerez
    /// arrival onto Trebujena's 800 m runway two six.
    fn run_borrowing_strip_case() {
        let mut jerez = region_apt("Jerez", 36.7446, -6.0601);
        jerez.apt.freqs.push(nav::AtcFreq {
            kind: nav::FREQ_ATIS,
            khz: 125655,
            name: "ATIS".into(),
        });
        jerez.apt.freqs.push(nav::AtcFreq {
            kind: nav::FREQ_GROUND,
            khz: 133280,
            name: "GND".into(),
        });
        jerez.apt.freqs.push(nav::AtcFreq {
            kind: nav::FREQ_APPROACH,
            khz: 128500,
            name: "Seville Approach".into(),
        });
        *REGION.lock().unwrap() = vec![region_apt("Trebujena", 36.8587, -6.1413), jerez];
        *ACTIVE_FIELD.lock().unwrap() = Some(("Sevilla San Pablo".into(), 37.418, -5.893));
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        *CLEARANCE.lock().unwrap() = None;
        *SQUAWK_WATCH.lock().unwrap() = None;
        // Airborne between the two, at the flight's real 13:09 position:
        // the REAL airport wins the handoff despite being farther.
        let sw = region_tick(36.98726, -5.94432, false, Instant::now()).expect("switch");
        assert!(sw.line.contains("Jerez"), "{}", sw.line);
        let pending = PENDING_STATION.lock().unwrap().take().expect("swap armed");
        assert_eq!(pending.1.name, "Jerez", "vector the arrival to the destination");
        // Parked AT the strip, though: you're at the field you're at — the
        // ground relocation stays raw-nearest, however small the strip.
        *ACTIVE_FIELD.lock().unwrap() = Some(("Jerez".into(), 36.7446, -6.0601));
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        let sw = region_tick(36.8587, -6.1413, true, Instant::now()).expect("ground relocation");
        assert!(sw.line.contains("Trebujena"), "{}", sw.line);
    }

    #[test]
    fn runway_occupancy_is_a_tight_rectangle() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // LXGB 09/27, real-ish geometry: ~1.7 km east-west at 36.151N.
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway { width_m: 0.0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1512,
            lon_a: -5.3554,
            lat_b: 36.1517,
            lon_b: -5.3366,
        }];
        // On the centerline past midfield: on the runway, named by the nearer end.
        assert_eq!(on_active_runway(36.15155, -5.342).as_deref(), Some("27"));
        // Near the 09 threshold: still on, named 09.
        assert_eq!(on_active_runway(36.1512, -5.3550).as_deref(), Some("09"));
        // 100 m abeam the centerline (a parallel taxiway): NOT on the runway.
        assert!(on_active_runway(36.1503, -5.346).is_none());
        // A mile away at the parking: no.
        assert!(on_active_runway(36.16, -5.35).is_none());
        // Past the far threshold by more than the overrun margin: no.
        assert!(on_active_runway(36.152, -5.330).is_none());
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    }

    #[test]
    fn information_letters_parse_from_calls() {
        assert_eq!(heard_information("with information alpha"), Some('A'));
        assert_eq!(heard_information("we have information bravo, request taxi"), Some('B'));
        // "x-ray" splits at the hyphen; the bare "x" carries the letter anyway.
        assert_eq!(heard_information("information x-ray"), Some('X'));
        assert_eq!(heard_information("information b"), Some('B'));
        assert_eq!(heard_information("request taxi"), None);
        assert_eq!(heard_information("for your information the flaps are stuck"), None);
    }

    #[test]
    fn having_the_current_information_skips_the_qnh() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ATIS.lock().unwrap() = Some(AtisState {
            khz: 118500,
            airport: "Gibraltar".into(),
            runways: Vec::new(),
            letter: 'A',
            edition: None,
            last_spoken: None,
        });
        let _ = atis_tick(118500, &wx(250.0, 12.0, 1018.0), Instant::now()); // airs Alpha
        let w = Wx { wind_from_deg: 250.0, wind_kts: 12.0 };
        // With the current letter: the taxi clearance drops the QNH clause.
        let mut st = lxgb();
        let ctx = {
            st.note_information("gibraltar tower, with information alpha, request taxi");
            st.make_ctx(w, "QNH one zero one eight".into(), "November".into(), true, None, false, None)
        };
        let reply = st.twr.handle(tower::parse_intent("request taxi"), &ctx);
        assert!(!reply.contains("QNH"), "{reply}");
        // A STALE letter keeps the numbers — safety over brevity.
        let mut st = lxgb();
        st.note_information("with information zulu, request taxi");
        let ctx = st.make_ctx(w, "QNH one zero one eight".into(), "November".into(), true, None, false, None);
        let reply = st.twr.handle(tower::parse_intent("request taxi"), &ctx);
        assert!(reply.contains("QNH one zero one eight"), "{reply}");
        // No letter at all: numbers as always.
        let mut st = lxgb();
        let ctx = st.make_ctx(w, "QNH one zero one eight".into(), "November".into(), true, None, false, None);
        let reply = st.twr.handle(tower::parse_intent("request taxi"), &ctx);
        assert!(reply.contains("QNH one zero one eight"), "{reply}");
        *ATIS.lock().unwrap() = None;
    }

    #[test]
    fn unkeyed_routing_needs_a_radio_shaped_call() {
        // Crew talk that merely contains intent keywords must fall through —
        // "anything else we need to do before takeoff?" was answered with a
        // takeoff clearance on 2026-07-08, and the panel's "land the aircraft"
        // button got a join clearance.
        for crew in [
            "anything else we need to do before takeoff",
            "land the aircraft",
            "flaps set for takeoff",
            "after landing checklist",
        ] {
            assert!(!is_radio_call(crew, "gibraltar"), "{crew}");
        }
        // Addressed or call-vocabulary utterances still route.
        for call in [
            "hello tower, i would like clearance for takeoff",
            "gibraltar, request taxi",
            "request touch and go",
            "inbound for landing",
            "november one seven two, final",
            "radio check",
        ] {
            assert!(is_radio_call(call, "gibraltar"), "{call}");
        }
    }

    #[test]
    fn calm_wind_holds_the_runway_in_use() {
        let path = lxgb_apt_file();
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        // A real westerly picks 27.
        assert_eq!(st.pick_runway(250.0, 12.0), "27");
        // Calm air with the direction flopping east must NOT flip the runway —
        // the 2026-07-08 flight was cleared on 27, then re-cleared on 09 two
        // minutes later in the same calm.
        assert_eq!(st.pick_runway(90.0, 1.0), "27");
        assert_eq!(st.pick_runway(170.0, 0.0), "27");
        // Actual wind changes the runway like a real controller would.
        assert_eq!(st.pick_runway(90.0, 10.0), "09");
    }

    #[test]
    fn near_miss_alert_names_the_way_out() {
        let f = |nm: f64, clock: i32, ft: f64| TrafficFactor { nm, clock, ft };
        // The line: half a mile, 500 ft. Outside it, only the advisory tier.
        assert!(is_near_miss(&f(0.4, 3, 0.0)));
        assert!(!is_near_miss(&f(0.6, 3, 0.0)), "outside half a mile");
        assert!(!is_near_miss(&f(0.4, 3, 600.0)), "vertical separation holds");
        // Vertical escape when the intruder is clearly above or below.
        assert!(near_miss_line(&f(0.4, 12, 300.0), false).contains("advise you descend"));
        assert!(near_miss_line(&f(0.4, 12, -300.0), false).contains("advise you climb"));
        // Co-altitude: turn AWAY from the traffic's side...
        assert!(near_miss_line(&f(0.4, 2, 0.0), false).contains("turn left"));
        assert!(near_miss_line(&f(0.4, 10, 0.0), false).contains("turn right"));
        // ...right for head-on (the right-of-way rule)...
        assert!(near_miss_line(&f(0.4, 12, 0.0), false).contains("turn right"));
        // ...and vertical for one dead astern, where a turn hides it.
        assert!(near_miss_line(&f(0.4, 6, 0.0), false).contains("climb"));
        // Controller shape vs crew shape: same geometry, right register.
        let atc = near_miss_line(&f(0.4, 2, 0.0), false);
        assert!(atc.starts_with("traffic alert"), "{atc}");
        assert!(atc.ends_with("immediately."), "{atc}");
        let crew = near_miss_line(&f(0.4, 2, 0.0), true);
        assert!(crew.starts_with("Traffic!"), "{crew}");
        assert!(crew.contains("now."), "{crew}");
    }

    /// The adversarial corpus (`bench/atc_corpus.yaml`): real whisper output
    /// mined from the flight logs plus hand-written paraphrases, homophone
    /// mangles, and crew-talk near-misses. Every case judges up to three
    /// parsers — intent, the unkeyed radio-call gate, the readback detector —
    /// and failures collect so one run reports every drift at once.
    #[test]
    fn transcript_corpus_parses_safely() {
        #[derive(serde::Deserialize)]
        struct Case {
            text: String,
            intent: String,
            #[serde(default)]
            radio: Option<bool>,
            #[serde(default)]
            readback: Option<bool>,
        }
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/bench/atc_corpus.yaml");
        let cases: Vec<Case> =
            serde_yaml::from_str(&std::fs::read_to_string(path).expect("corpus file"))
                .expect("corpus yaml");
        assert!(cases.len() >= 100, "corpus shrank to {} cases", cases.len());
        let mut fails = Vec::new();
        for c in &cases {
            let t = c.text.to_lowercase();
            let Some(want) = tower::intent_from_label(&c.intent) else {
                panic!("{:?}: unknown intent label {:?}", c.text, c.intent);
            };
            let got = tower::parse_intent(&t);
            if got != want {
                fails.push(format!("{:?}: parsed {got:?}, wanted {want:?}", c.text));
            }
            if let Some(radio) = c.radio {
                if is_radio_call(&t, "gibraltar") != radio {
                    fails.push(format!("{:?}: is_radio_call should be {radio}", c.text));
                }
            }
            if let Some(rb) = c.readback {
                if tower::is_readback(&t) != rb {
                    fails.push(format!("{:?}: is_readback should be {rb}", c.text));
                }
            }
        }
        assert!(
            fails.is_empty(),
            "{} of {} corpus cases drifted:\n{}",
            fails.len(),
            cases.len(),
            fails.join("\n")
        );
    }

    fn calm() -> Wx {
        Wx {
            wind_from_deg: 0.0,
            wind_kts: 0.0,
        }
    }

    /// The live context exactly as `build_ctx` would assemble it, minus the
    /// simulator: scripted wind, fixed QNH and tail number.
    fn scripted_ctx(st: &mut Station, wx: Wx) -> tower::Ctx {
        st.make_ctx(
            wx,
            "QNH one zero one seven".into(),
            "November one seven two Sierra Papa".into(),
            false, // airborne default; only the mayday answer reads it
            None,  // final clear; traffic-aware cases build their own ctx
            false, // runway unoccupied
            None,  // no wake clock
        )
    }

    /// Whole sorties driven through the pure routing layer — the dialogue
    /// sequences that live between the tower FSM (unit-tested above it) and
    /// the mic (untestable): out-of-order calls, questions on a keyed radio,
    /// readbacks, wrong-service redirects, the departure check-in. Every
    /// scene asserts the reply AND that the clearance sequence only moves
    /// when it should.
    #[test]
    fn sortie_dialogues_battle_test_the_routing() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let path = lxgb_apt_file();
        let wx = Wx {
            wind_from_deg: 250.0,
            wind_kts: 12.0,
        };

        // Scene 1: the departure happy path, keyed on the tower frequency.
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        let ctx = scripted_ctx(&mut st, wx);
        let r = st.route_keyed("gibraltar tower, radio check", Service::Tower, &ctx, wx);
        assert!(r.contains("reading you five"), "{r}");
        assert_eq!(st.twr.state, tower::State::Parked, "a radio check moves nothing");

        let r = st.route_keyed("request clearance for taxi", Service::Tower, &ctx, wx);
        assert!(r.contains("taxi to holding point Alpha runway two seven"), "{r}");
        assert_eq!(st.twr.state, tower::State::TaxiCleared);

        // A readback earns silence; garble earns "say again"; neither moves
        // the sequence.
        let r = st.route_keyed("roger, holding point alpha", Service::Tower, &ctx, wx);
        assert_eq!(r, "", "a readback keeps the frequency quiet: {r}");
        let r = st.route_keyed("uhh gibraltar something something", Service::Tower, &ctx, wx);
        assert!(r.contains("say again"), "{r}");
        assert_eq!(st.twr.state, tower::State::TaxiCleared);

        // A question keyed by mistake gets "say again", never a clearance —
        // the 2026-07-08 regression, now at the routing layer.
        let r = st.route_keyed("what runway shall i take off from", Service::Tower, &ctx, wx);
        assert!(r.contains("say again"), "{r}");
        let r = st.route_keyed("takeoff checklist", Service::Tower, &ctx, wx);
        assert!(r.contains("say again"), "{r}");
        assert_eq!(st.twr.state, tower::State::TaxiCleared, "questions moved the sequence");

        let r = st.route_keyed("ready for departure", Service::Tower, &ctx, wx);
        assert!(r.contains("cleared for takeoff"), "{r}");
        assert_eq!(st.twr.state, tower::State::TakeoffCleared);

        // The takeoff clearance armed the handoff; airborne through 700 ft it
        // fires exactly once, to the departure frequency.
        assert!(handoff_tick(300.0, false).is_none(), "still low");
        let line = handoff_tick(900.0, false).expect("handoff due");
        assert!(line.contains("Departure on one three zero decimal four"), "{line}");
        assert!(handoff_tick(1500.0, false).is_none(), "spoken once");

        // Scene 2: the departure controller. First readback is the check-in
        // and gets radar contact; later readbacks earn silence.
        let r = st.route_keyed(
            "passing one thousand five hundred climbing",
            Service::Departure,
            &ctx,
            wx,
        );
        assert!(r.contains("Departure, radar contact"), "{r}");
        let r = st.route_keyed("maintaining two thousand", Service::Departure, &ctx, wx);
        assert_eq!(r, "", "post-check-in readbacks stay quiet: {r}");

        // Runway-shaped business on the wrong frequency redirects to the
        // tower and must not advance the tower's sequence.
        let was = st.twr.state;
        let r = st.route_keyed("request taxi", Service::Approach, &ctx, wx);
        assert!(r.contains("contact Gibraltar Tower on one three one decimal two"), "{r}");
        let r = st.route_keyed("ready for departure", Service::Departure, &ctx, wx);
        assert!(r.contains("contact Gibraltar Tower"), "{r}");
        assert_eq!(st.twr.state, was, "a redirect must not move the sequence");

        // Coming back in: departure sends arrivals to approach when the field
        // has one, and the touch-and-go option survives the redirect.
        let r = st.route_keyed("request touch and go", Service::Departure, &ctx, wx);
        assert!(r.contains("contact Gibraltar Approach on one two two decimal eight"), "{r}");
        let clear = st.route_keyed("november one seven two, final", Service::Tower, &ctx, wx);
        assert!(clear.contains("cleared touch and go"), "{clear}");

        // Scene 3: the arrival, keyed on the tower — inbound, final, then off
        // the runway. Calm wind must hold the westerly runway pick.
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        let ctx = scripted_ctx(&mut st, wx);
        let r = st.route_keyed("gibraltar tower, inbound for landing", Service::Tower, &ctx, wx);
        assert!(r.contains("join final runway two seven"), "{r}");
        let calm_ctx = scripted_ctx(&mut st, calm());
        assert!(calm_ctx.runway.contains("two seven"), "calm flipped the runway");
        let r = st.route_keyed("final", Service::Tower, &calm_ctx, calm());
        assert!(r.contains("cleared to land"), "{r}");
        // Reading the clearance back must earn silence and hold the state —
        // it used to re-parse as a landing request and walk the sequence
        // backwards to Inbound.
        let r = st.route_keyed(
            "cleared to land runway two seven, november one seven two sierra papa",
            Service::Tower,
            &calm_ctx,
            calm(),
        );
        assert_eq!(r, "", "a clearance echo keeps the frequency quiet: {r}");
        assert_eq!(st.twr.state, tower::State::ClearedToLand, "echo moved the sequence");
        let r = st.route_keyed("request taxi", Service::Tower, &calm_ctx, calm());
        assert!(r.contains("taxi to parking"), "{r}");
        assert_eq!(st.twr.state, tower::State::Parked);
        let r = st.route_keyed("i've just landed", Service::Tower, &calm_ctx, calm());
        assert!(r.contains("taxi to parking"), "{r}");

        // Scene 4: content routing (no PTT). Crew talk falls through even
        // when it contains clearance words; radio-shaped calls route.
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        let ctx = scripted_ctx(&mut st, wx);
        for crew in [
            "anything else we need to do before takeoff",
            "land the aircraft",
            "flaps set for takeoff",
            "after landing checklist",
            "gibraltar is somewhere ahead",
        ] {
            assert!(
                st.route_unkeyed(crew, Service::Tower, &ctx, wx).is_none(),
                "crew talk routed to ATC: {crew}"
            );
        }
        // "hello tower" IS addressed to the tower — since 2026-07-10 a hail
        // on frequency earns the invitation, and moves nothing.
        let r = st
            .route_unkeyed("hello tower", Service::Tower, &ctx, wx)
            .expect("a hail addressed to the tower is answered");
        assert!(r.contains("pass your message"), "{r}");
        assert_eq!(st.twr.state, tower::State::Parked, "crew talk moved the sequence");
        let r = st
            .route_unkeyed("gibraltar tower, request taxi", Service::Tower, &ctx, wx)
            .expect("a proper call routes");
        assert!(r.contains("holding point"), "{r}");

        // Any leftover handoff from this sortie must not leak into the next
        // test holding the lock.
        *HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
    }

    #[test]
    fn readback_discipline_runs_through_the_keyed_route() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let path = lxgb_apt_file();
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        let wx = Wx { wind_from_deg: 90.0, wind_kts: 8.0 }; // runway 09
        let mut ctx = scripted_ctx(&mut st, wx);
        ctx.on_ground = true;

        let r = st.route_keyed("gibraltar tower, request taxi", Service::Tower, &ctx, wx);
        assert!(r.contains("holding point"), "{r}");
        let r = st.route_keyed("ready for departure", Service::Tower, &ctx, wx);
        assert!(r.contains("cleared for takeoff"), "{r}");
        // The coach owes exactly the readback now.
        let owed = CLEARANCE
            .lock()
            .unwrap()
            .as_ref()
            .and_then(|c| c.pending_readback.clone())
            .expect("readback owed");
        assert!(owed.contains("Cleared for takeoff runway zero niner"), "{owed}");
        // A different call is challenged, and the challenge repeats the
        // instruction — nothing moves.
        let r = st.route_keyed("gibraltar tower, request taxi", Service::Tower, &ctx, wx);
        assert!(r.contains("read back"), "{r}");
        assert!(r.contains("cleared for takeoff"), "{r}");
        assert_eq!(st.twr.state, tower::State::TakeoffCleared, "challenge moves nothing");
        // The WRONG runway is corrected, never accepted.
        let r = st.route_keyed(
            "cleared for takeoff runway two seven, november one seven two sierra papa",
            Service::Tower,
            &ctx,
            wx,
        );
        assert!(r.contains("negative"), "{r}");
        // The right echo earns silence and pays the debt.
        let r = st.route_keyed(
            "cleared for takeoff runway zero niner, november one seven two sierra papa",
            Service::Tower,
            &ctx,
            wx,
        );
        assert_eq!(r, "", "correct readback = a quiet frequency");
        assert!(
            CLEARANCE.lock().unwrap().as_ref().unwrap().pending_readback.is_none(),
            "debt paid, coach released"
        );

        // Enforcement off: the challenge disappears, the call routes.
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").unwrap();
        st.set_enforce_readback(false);
        let mut ctx = scripted_ctx(&mut st, wx);
        ctx.on_ground = true;
        st.route_keyed("request taxi", Service::Tower, &ctx, wx);
        let r = st.route_keyed("ready for departure", Service::Tower, &ctx, wx);
        assert!(r.contains("cleared for takeoff"), "{r}");
        let r = st.route_keyed("gibraltar tower, request taxi", Service::Tower, &ctx, wx);
        assert!(r.contains("holding point"), "no challenge with enforcement off: {r}");

        *CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *TAKEOFF_HOLD.lock().unwrap_or_else(|e| e.into_inner()) = None;
    }

    #[test]
    fn conditional_line_up_arms_the_release_watch() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let path = lxgb_apt_file();
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        let wx = Wx { wind_from_deg: 90.0, wind_kts: 8.0 };
        let mut ctx = scripted_ctx(&mut st, wx);
        ctx.on_ground = true;
        ctx.final_traffic_nm = Some(2.5);
        ctx.final_traffic_count = 1;
        ctx.final_traffic_callsign = Some("Speedbird three zero six".into());
        *TAKEOFF_HOLD.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *WAKE.lock().unwrap_or_else(|e| e.into_inner()) = None;

        st.respond(Service::Tower, tower::Intent::RequestTaxi, &ctx, wx);
        let r = st.respond(Service::Tower, tower::Intent::RequestTakeoff, &ctx, wx);
        assert!(r.contains("behind the landing Speedbird three zero six"), "{r}");
        assert!(r.contains("line up runway zero niner behind"), "{r}");
        assert_eq!(st.twr.state, tower::State::LinedUp);
        assert!(
            TAKEOFF_HOLD.lock().unwrap().is_some(),
            "a conditional line-up owes the clearance: release watch armed"
        );

        // The lander is down and gone: the watch releases the clearance
        // after the clear window, and the FSM sync is queued for the main
        // loop exactly like a plain hold.
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway { width_m: 0.0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1508156,
            lon_a: -5.3596312,
            lat_b: 36.1514944,
            lon_b: -5.3400082,
        }];
        *PENDING_TAKEOFF_CLEARANCE.lock().unwrap() = false;
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        assert!(takeoff_hold_tick(&[], 11.0, true, t0).is_none(), "clear window starts");
        let line = takeoff_hold_tick(&[], 11.0, true, t0 + sec(16)).expect("release");
        assert!(line.contains("cleared for takeoff"), "{line}");
        assert!(*PENDING_TAKEOFF_CLEARANCE.lock().unwrap());

        *PENDING_TAKEOFF_CLEARANCE.lock().unwrap() = false;
        *TAKEOFF_HOLD.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner()) = Vec::new();
    }

    #[test]
    fn wake_clock_starts_on_an_airline_liftoff_and_blocks_the_release() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.1512, -5.3554));
        *ACTIVE_PANEL.lock().unwrap() = Some(("Gibraltar".into(), 11.0, Vec::new()));
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway { width_m: 0.0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1508156,
            lon_a: -5.3596312,
            lat_b: 36.1514944,
            lon_b: -5.3400082,
        }];
        *WAKE.lock().unwrap() = None;
        *WAKE_PREV.lock().unwrap() = None;
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        let rolling = TcasTarget {
            callsign: "BA306".into(),
            lat: 36.1510,
            lon: -5.3590,
            ele_ft: 11.0,
            on_ground: true,
            vs_fpm: 0.0,
        };
        wake_note_targets(&[rolling.clone()], t0);
        assert!(WAKE.lock().unwrap().is_none(), "still rolling: no clock");
        let airborne = TcasTarget { ele_ft: 150.0, on_ground: false, ..rolling };
        wake_note_targets(&[airborne], t0 + sec(5));
        let rem = wake_remaining("09", t0 + sec(10)).expect("clock running");
        assert!(rem > 100, "{rem}");
        // Both ends of the same pavement share the wake.
        assert!(wake_remaining("27", t0 + sec(10)).is_some());
        // A strip this field doesn't have: not our wake.
        assert!(wake_remaining("14", t0 + sec(10)).is_none());
        // Two minutes later it has died out.
        assert!(wake_remaining("09", t0 + sec(125)).is_none());

        // A GA tail lifting off starts NO clock — no airline prefix, no
        // assumed weight class.
        *WAKE.lock().unwrap() = None;
        *WAKE_PREV.lock().unwrap() = None;
        let ga = TcasTarget {
            callsign: "N123AB".into(),
            lat: 36.1510,
            lon: -5.3590,
            ele_ft: 11.0,
            on_ground: true,
            vs_fpm: 0.0,
        };
        wake_note_targets(&[ga.clone()], t0);
        wake_note_targets(&[TcasTarget { ele_ft: 150.0, on_ground: false, ..ga }], t0 + sec(5));
        assert!(WAKE.lock().unwrap().is_none(), "GA departure, no wake clock");

        *WAKE_PREV.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    }

    #[test]
    fn off_route_taxiing_gets_a_nudge_with_the_via_line() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.1512, -5.3554));
        *ACTIVE_PANEL.lock().unwrap() =
            Some(("Gibraltar".into(), 11.0, vec![("tower".to_string(), 131200_i64)]));
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway { width_m: 0.0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1508156,
            lon_a: -5.3596312,
            lat_b: 36.1514944,
            lon_b: -5.3400082,
        }];
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::TaxiCleared,
            callsign: "November one seven two Sierra Papa".into(),
            runway_ident: "09".into(),
            pending_readback: None,
        });
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        // The cleared route: a straight east-west line 1 km north of the
        // runway.
        *TAXI_ROUTE.lock().unwrap() = Some(TaxiWatch {
            path: vec![(36.1600, -5.3600), (36.1600, -5.3500)],
            via: "Alpha, Bravo".into(),
            seg: vec![("A".into(), false)],
            holding_point: "A1".into(),
        });
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        // Parked on the route: settles the air/ground edge, no call.
        assert!(compliance_tick(36.1600, -5.3600, 15.0, true, 0.0, true, t0).is_none());
        // Taxiing ALONG the route: quiet.
        assert!(compliance_tick(36.1600, -5.3550, 15.0, true, 12.0, true, t0 + sec(1)).is_none());
        // 200 m off it: the nudge, with the via line, from the controller.
        let call = compliance_tick(36.1580, -5.3550, 15.0, true, 12.0, true, t0 + sec(2))
            .expect("off-route nudge");
        assert!(!call.crew);
        assert!(call.line.contains("off the cleared taxi route"), "{}", call.line);
        assert!(call.line.contains("Taxi via Alpha, Bravo"), "{}", call.line);
        // Its own cooldown holds — a nudge, not a siren.
        assert!(compliance_tick(36.1580, -5.3550, 15.0, true, 12.0, true, t0 + sec(10)).is_none());

        *TAXI_ROUTE.lock().unwrap() = None;
        *CLEARANCE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    }

    #[test]
    fn coach_owes_the_readback_first_and_teaches_the_line_up() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_PANEL.lock().unwrap() = Some(("Gibraltar".into(), 11.0, Vec::new()));
        let cs = "November one seven two Sierra Papa";
        // A readback owed IS the next call — it outranks the state grid.
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::TakeoffCleared,
            callsign: cs.into(),
            runway_ident: "09".into(),
            pending_readback: Some(format!("Cleared for takeoff runway zero niner, {cs}.")),
        });
        let s = suggest_call(cs, true, None, None).expect("a suggestion");
        assert_eq!(
            s.call.as_deref(),
            Some(format!("Cleared for takeoff runway zero niner, {cs}.").as_str())
        );
        assert!(s.urgent, "an owed readback is urgent");
        assert!(s.first.is_none(), "the readback needs no prerequisite");
        // Lined up with the readback paid: the nudge parses as a fresh ask.
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::LinedUp,
            callsign: cs.into(),
            runway_ident: "09".into(),
            pending_readback: None,
        });
        let s = suggest_call(cs, true, None, None).expect("a suggestion");
        assert!(!s.urgent);
        let c = s.call.expect("a call");
        assert_eq!(tower::parse_intent(&c), tower::Intent::RequestTakeoff, "{c}");
        *CLEARANCE.lock().unwrap() = None;
    }

    #[test]
    fn narrated_takeoff_names_the_departure_end_not_the_nearest() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway { width_m: 0.0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1508156,
            lon_a: -5.3596312,
            lat_b: 36.1514944,
            lon_b: -5.3400082,
        }];
        *GHOSTS.lock().unwrap() = None;
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        // Rolling east on 09, still on the ground: seeds the track memory.
        let rolling = TcasTarget {
            callsign: "BAW123".into(),
            lat: 36.1512,
            lon: -5.3500,
            ele_ft: 11.0,
            on_ground: true,
            vs_fpm: 0.0,
        };
        assert!(
            chatter_tick(&[rolling.clone()], "Gibraltar", 36.1512, -5.3554, 11.0, None, t0)
                .is_none()
        );
        // Liftoff PAST midfield, near the 27 threshold: the nearest end is
        // 27, but the roll direction says 09 — the tape read every 09R
        // departure as "two seven left" until this.
        let lifted = TcasTarget {
            lat: 36.1514,
            lon: -5.3410,
            ele_ft: 60.0,
            on_ground: false,
            ..rolling
        };
        let ex = chatter_tick(&[lifted], "Gibraltar", 36.1512, -5.3554, 11.0, None, t0 + sec(1))
            .expect("takeoff narrated");
        let line = &ex.lines[0].text;
        assert!(line.contains("runway zero niner"), "{line}");
        assert!(!line.contains("two seven"), "{line}");
        *GHOSTS.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    }

    #[test]
    fn fresh_station_install_arms_the_zone_grace() {
        // A region handoff says "contact <name> Tower" — the new station's
        // zone watch must give the pilot the cooldown window to actually
        // call, not challenge them as an intruder two seconds later
        // (2026-07-10 tape).
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        lxgb().install();
        assert!(
            COMPLIANCE.lock().unwrap().zone_last.is_some(),
            "check-in grace armed on install"
        );
        *ATIS.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        *ACTIVE_PANEL.lock().unwrap() = None;
        *ACTIVE_FIELD.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *CONTROLLER_FREQS.lock().unwrap() = Vec::new();
    }

    #[test]
    fn taxiways_speak_like_a_controller() {
        assert_eq!(spoken_taxiway("A"), "Alpha");
        assert_eq!(spoken_taxiway("B2"), "Bravo two");
        assert_eq!(spoken_taxiway("A1"), "Alpha one");
        assert_eq!(spoken_taxiway("N4W"), "November four Whiskey");
        assert_eq!(spoken_taxiway("LINK27"), "Link two seven");
        assert_eq!(spoken_taxiway("Link 41"), "Link four one");
        assert_eq!(spoken_taxiway("Inner"), "Inner");
        assert_eq!(first_designator("09L/27R"), "09L");
        assert_eq!(first_designator("14"), "14");
    }

    #[test]
    fn via_lists_collapse_numbered_runs() {
        let names = |v: &[&str]| v.iter().map(|s| s.to_string()).collect::<Vec<_>>();
        // Madrid's real 2026-07-13 clearance: "Charlie one, Mike one, Mike
        // two, Mike three, Mike four, Mike five" reads like the database.
        assert_eq!(
            spoken_via(&names(&["C1", "M1", "M2", "M3", "M4", "M5"])),
            vec!["Charlie one", "Mike one to Mike five"]
        );
        // Descending runs collapse the same way (taxiing the other way).
        assert_eq!(
            spoken_via(&names(&["M5", "M4", "M3", "B"])),
            vec!["Mike five to Mike three", "Bravo"]
        );
        // Two in a row is not a run — say both.
        assert_eq!(
            spoken_via(&names(&["M1", "M2", "A"])),
            vec!["Mike one", "Mike two", "Alpha"]
        );
        // A gap breaks the run; a different designator breaks the run.
        assert_eq!(
            spoken_via(&names(&["M1", "M3", "M4", "N1"])),
            vec!["Mike one", "Mike three", "Mike four", "November one"]
        );
        // Unnumbered names pass straight through.
        assert_eq!(
            spoken_via(&names(&["L", "B", "G11", "A", "HN1"])),
            vec!["Lima", "Bravo", "Golf one one", "Alpha", "Hotel November one"]
        );
    }

    /// The world sweep: EVERY towered airport in the real Global Airports
    /// file stands up a station, briefs its frequencies, and flies the
    /// departure happy path — the shakeout for apt.dat shapes LXGB never
    /// showed us. Opt-in because the file ships with X-Plane, not the repo:
    ///
    ///   cargo test --release -p atc -- --ignored atc_world_sweep
    ///
    /// Override the file with XCOPILOT_APT_DAT.
    #[test]
    #[ignore = "needs X-Plane's Global Airports apt.dat; run with -- --ignored"]
    fn atc_world_sweep_every_towered_field_stands_up() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let path = std::env::var("XCOPILOT_APT_DAT").unwrap_or_else(|_| {
            "/Users/duanelicudi/X-Plane 12/Global Scenery/Global Airports/Earth nav data/apt.dat"
                .to_string()
        });
        let wx = Wx {
            wind_from_deg: 250.0,
            wind_kts: 12.0,
        };
        let (mut total, mut towered, mut no_runways) = (0u32, 0u32, 0u32);
        let mut fails: Vec<String> = Vec::new();
        nav::scan_airports(&path, |icao, apt| {
            total += 1;
            let Some(mut st) = Station::from_airport(apt) else { return };
            towered += 1;
            let mut check = |cond: bool, what: &str| {
                if !cond && fails.len() < 40 {
                    fails.push(format!("{icao}: {what}"));
                }
            };
            check(st.covers_khz(st.tower_khz), "tower frequency not covered");

            // The briefing must read every published service, and name the
            // tower when COM1 sits on it.
            let brief = st.briefing_text(Some(st.tower_khz));
            check(brief.contains("tower"), &format!("no tower in briefing: {brief}"));
            check(
                brief.ends_with("Com one is on the tower."),
                &format!("tuned service unnamed: {brief}"),
            );

            // The departure happy path, spoken with live-shaped context.
            let ctx = scripted_ctx(&mut st, wx);
            if ctx.runway.is_empty() {
                no_runways += 1; // towered heliports/seaplane bases: no 100 rows
            }
            let r = st.route_keyed("radio check", Service::Tower, &ctx, wx);
            check(r.contains("reading you five"), &format!("radio check: {r}"));
            let r = st.route_keyed("request taxi", Service::Tower, &ctx, wx);
            check(r.contains("taxi to holding point"), &format!("taxi: {r}"));
            let r = st.route_keyed("ready for departure", Service::Tower, &ctx, wx);
            check(r.contains("cleared for takeoff"), &format!("takeoff: {r}"));
            if !ctx.runway.is_empty() {
                check(
                    r.contains(&ctx.runway),
                    &format!("clearance names the wrong runway: {r}"),
                );
            }

            // Vectors exercise the runway geometry math per field.
            let r = st.route_keyed("request vectors", Service::Approach, &ctx, wx);
            check(!r.is_empty(), "vectors: empty reply");

            // Statics must not leak between airports (or into other tests).
            *HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
            *VECTORS.lock().unwrap_or_else(|e| e.into_inner()) = None;
        })
        .expect("apt.dat scan");
        println!(
            "swept {total} airports, {towered} towered ({no_runways} with no runway rows)"
        );
        assert!(towered > 100, "suspiciously few towered fields: {towered}");
        assert!(
            fails.is_empty(),
            "{} fields failed (first {}):\n{}",
            fails.len(),
            fails.len().min(40),
            fails.join("\n")
        );
    }

    #[test]
    fn takeoff_clearance_arms_the_departure_handoff() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let path = lxgb_apt_file();
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        assert_eq!(st.departure_khz, Some(130400), "departure row wired");
        let ctx = tower::Ctx {
            callsign: "November one seven two Sierra Papa".into(),
            airport: "Gibraltar".into(),
            runway: "two seven".into(),
            qnh: "QNH one zero one five".into(),
            wind: "calm".into(),
            holding_point: "Alpha".into(),
            approach: Some(atis::spoken_mhz(122800)),
            squawk: "seven zero zero zero".into(),
            final_traffic_nm: None,
            final_traffic_callsign: None,
            final_traffic_count: 0,
            runway_occupied: false,
            wake_secs_left: None,
            taxi_via: Vec::new(),
            taxi_crossings: Vec::new(),
            on_ground: false,
        };
        // Walk the tower to a takeoff clearance and arm, exactly as respond()
        // does around twr.handle().
        st.twr.handle(tower::parse_intent("request taxi"), &ctx);
        let was = st.twr.state;
        st.twr.handle(tower::parse_intent("ready for departure"), &ctx);
        assert_ne!(was, st.twr.state);
        st.arm_departure_handoff(&ctx);

        // On the roll: silence. Climbing through field + 700: the handoff,
        // exactly once.
        assert!(handoff_tick(11.0, true).is_none());
        assert!(handoff_tick(300.0, false).is_none());
        let line = handoff_tick(900.0, false).expect("handoff due");
        assert!(
            line.contains("contact Gibraltar Departure on one three zero decimal four"),
            "{line}"
        );
        assert!(handoff_tick(1500.0, false).is_none(), "spoken once");
    }

    #[test]
    fn named_station_tuning_resolves_from_apt_dat() {
        let st = lxgb();
        match st.tune_request("tune the tower") {
            Some(Tune::Found { label, khz }) => {
                assert_eq!(khz, 131200);
                assert_eq!(label, "Gibraltar tower");
            }
            other => panic!("expected tower: {:?}", other.is_some()),
        }
        // Generic "atc"/"station" means the tower.
        assert!(matches!(
            st.tune_request("tune to atc"),
            Some(Tune::Found { khz: 131200, .. })
        ));
        assert!(matches!(
            st.tune_request("contact approach"),
            Some(Tune::Found { khz: 122800, .. })
        ));
        // A named station the field doesn't have declines by name, no guessing.
        match st.tune_request("tune the atis") {
            Some(Tune::Missing(msg)) => assert!(msg.contains("no atis"), "{msg}"),
            other => panic!("expected Missing: {:?}", other.is_some()),
        }
        // No tuning verb → not a tuning request, even if a station is named
        // ("gibraltar tower request taxi" must reach the tower, not the tuner).
        assert!(st.tune_request("gibraltar tower request taxi").is_none());
        // A verb without any station name is someone else's command.
        assert!(st
            .tune_request("tune com one to one one eight decimal five")
            .is_none());
        // "change frequency to <station>" — how the 2026-07-08 pilot actually
        // asked, four phrasings in a row, all missed.
        assert!(matches!(
            st.tune_request("let's change frequency to gibraltar tower"),
            Some(Tune::Found { khz: 131200, .. })
        ));
        assert!(matches!(
            st.tune_request("change frequency to tower"),
            Some(Tune::Found { khz: 131200, .. })
        ));
        // "switch frequency to tower." (2026-07-09 flight, missed: "switch"
        // only counted when followed directly by "to").
        assert!(matches!(
            st.tune_request("switch frequency to tower."),
            Some(Tune::Found { khz: 131200, .. })
        ));
        // "change"/"switch" without radio context must not retune.
        assert!(st.tune_request("change approach speed to seventy").is_none());
        assert!(st.tune_request("switch the landing light on").is_none());
        // "change to approach" tunes (2026-07-08 night flight: answered with
        // "approach around 65 knots" instead) — but flight-parameter talk
        // naming a station never does.
        assert!(matches!(
            st.tune_request("change to approach"),
            Some(Tune::Found { khz: 122800, .. })
        ));
        assert!(st.tune_request("change to approach speed").is_none());
        assert!(st.tune_request("select approach flaps").is_none());
    }

    #[test]
    fn freq_query_matches_questions_not_commands() {
        assert!(freq_query("what frequencies are here"));
        assert!(freq_query("which stations are in range"));
        assert!(freq_query("list the frequencies"));
        assert!(freq_query("who can i talk to"));
        assert!(freq_query("who can i contact"));
        // Commands and radio calls must fall through to their own paths.
        assert!(!freq_query("set frequency to one one eight decimal five"));
        assert!(!freq_query("tune the tower"));
        assert!(!freq_query("request taxi"));
        assert!(!freq_query("what's the qnh"));
    }

    #[test]
    fn briefing_reads_lxgb_services_and_com1() {
        let path = lxgb_apt_file();
        let st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        // Everything the field publishes, in briefing order, one sentence.
        let b = st.briefing_text(Some(131200));
        assert!(
            b.starts_with(
                "Gibraltar: tower one three one decimal two, \
                 approach one two two decimal eight, \
                 departure one three zero decimal four."
            ),
            "{b}"
        );
        assert!(b.ends_with("Com one is on the tower."), "{b}");
        // Tuned off every service: the raw frequency is read back instead.
        let b = st.briefing_text(Some(118000));
        assert!(b.ends_with("Com one is on one one eight decimal zero."), "{b}");
        // A failed COM1 read drops that sentence, never a wrong number.
        let b = st.briefing_text(None);
        assert!(b.ends_with("decimal four."), "{b}");
        // Ground and tower on the SAME frequency (8 real fields in the
        // 2026-07 apt.dat): the briefing names the service that answers —
        // the tower — not the first row in briefing order.
        let mut st = lxgb();
        st.apt.freqs.push(nav::AtcFreq {
            kind: nav::FREQ_GROUND,
            khz: 131200,
            name: "Ground".into(),
        });
        let b = st.briefing_text(Some(131200));
        assert!(b.ends_with("Com one is on the tower."), "{b}");
    }

    /// A fixture apt.dat with LXGB's rows verbatim from the Global Airports
    /// file (2026-07 cycle), plus a trailing airport to prove the scan stops.
    /// Written exactly once: several tests read it concurrently, and a
    /// per-test `File::create` truncates it under a parallel reader.
    fn lxgb_apt_file() -> std::path::PathBuf {
        use std::io::Write;
        use std::sync::OnceLock;
        static PATH: OnceLock<std::path::PathBuf> = OnceLock::new();
        PATH.get_or_init(|| {
            let path = std::env::temp_dir()
                .join(format!("xc-station-lxgb-{}.dat", std::process::id()));
            let mut f = std::fs::File::create(&path).unwrap();
            writeln!(f, "1     11 0 0 LXGB Gibraltar").unwrap();
            writeln!(f, "100 45.11 1 0 0.20 0 3 0  09 36.1508156 -5.3596312 136 0 5 1 0 3 27 36.1514944 -5.3400082 98 0 5 7 0 3").unwrap();
            writeln!(f, "1054 131200 Tower Freq").unwrap();
            writeln!(f, "1055 122800 Approach Freq").unwrap();
            writeln!(f, "1056 130400 Departure Freq").unwrap();
            writeln!(f, "1   1027 0 0 0MU1 Sunderland").unwrap();
            f.flush().unwrap();
            path
        })
        .clone()
    }

    #[test]
    fn lxgb_station_loads_from_real_apt_rows() {
        let path = lxgb_apt_file();
        let st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        assert_eq!(st.apt.name, "Gibraltar");
        assert_eq!(st.tower_khz, 131200);
        assert_eq!(st.approach_khz, Some(122800), "approach row wired for vectors");
        // One physical runway, both ends named.
        assert_eq!(st.apt.runways.len(), 1);
        let rw = &st.apt.runways[0];
        assert_eq!((rw.ident_a.as_str(), rw.ident_b.as_str()), ("09", "27"));
        // The 09 end points roughly east — the vectors' final approach course.
        assert!((rw.heading_a_deg() - 88.0).abs() < 5.0, "{}", rw.heading_a_deg());
        // Departures: the into-wind pick from the real geometry. Gibraltar's
        // prevailing westerly means 27; an easterly Levanter flips it to 09.
        let (id, _, hw) = nav::best_runway_for_wind(&st.apt.runways, 250.0, 12.0).unwrap();
        assert_eq!((id.as_str(), hw > 0.0), ("27", true));
        assert_eq!(nav::best_runway_for_wind(&st.apt.runways, 90.0, 10.0).unwrap().0, "09");
        // Every service the field actually has resolves by name; ATIS (which
        // LXGB's apt.dat doesn't list) declines instead of guessing.
        assert!(matches!(st.tune_request("contact departure"), Some(Tune::Found { khz: 130400, .. })));
        assert!(matches!(st.tune_request("tune the atis"), Some(Tune::Missing(_))));
    }

    /// The whole LXGB arrival, flown: vectors from north of the Rock onto
    /// final for 09, the visual-approach clearance + tower handoff once
    /// established, then the tower's landing clearance off the "final" report.
    /// One test on purpose — `VECTORS` is a process-wide static and parallel
    /// tests would clobber each other's plan.
    #[test]
    fn lxgb_vectored_arrival_flies_to_a_landing_clearance() {
        use std::time::Duration;

        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let path = lxgb_apt_file();
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        let rw = st.apt.runways[0].clone();
        let (thr_lat, thr_lon, course) = (rw.lat_a, rw.lon_a, rw.heading_a_deg());

        // Arm the plan exactly as arm_vectors would for runway 09.
        let target_alt_ft = ((st.apt.elev_ft + 1500.0) / 100.0).round() * 100.0;
        *VECTORS.lock().unwrap() = Some(VectorPlan {
            callsign: "November one seven two Sierra Papa".into(),
            thr_lat,
            thr_lon,
            course_deg: course,
            runway_spoken: "zero niner".into(),
            tower_spoken: Some(atis::spoken_mhz(st.tower_khz)),
            target_alt_ft,
            ils: false,
            join_nm: 6.0,
            cleared: false,
            dialect: Dialect::Icao,
            last_tx: None,
            last_heading: None,
        });

        // Regression: crossing final perpendicular inside the wedge must NOT
        // be "established" — position said cleared, the track says crossing.
        let t0 = std::time::Instant::now();
        let (x_lat, x_lon) = crate::nav::project_latlon(thr_lat, thr_lon, course + 180.0, 4.0);
        let crossing = vector_tick(x_lat, x_lon, 1600.0, Some(0.0), false, t0);
        if let Some(msg) = &crossing {
            assert!(!msg.contains("Cleared"), "cleared while crossing final: {msg}");
        }

        // Now the real arrival: 10 nm north of the field at 3,000 ft. The sim
        // obeys each assigned heading at 100 kts in 10 s ticks, descending to
        // the assigned altitude — a pilot doing exactly as told, nothing more.
        VECTORS.lock().unwrap().as_mut().unwrap().last_tx = None;
        VECTORS.lock().unwrap().as_mut().unwrap().last_heading = None;
        let (mut lat, mut lon) = crate::nav::project_latlon(thr_lat, thr_lon, 0.0, 10.0);
        let mut alt = 3000.0_f64;
        let mut hdg = 180.0_f64;
        let mut first: Option<String> = None;
        let mut cleared: Option<String> = None;
        for step in 0..240u64 {
            let now = t0 + Duration::from_secs(step * 10);
            if let Some(msg) = vector_tick(lat, lon, alt, Some(hdg), false, now) {
                if msg.contains("Cleared visual approach") {
                    cleared = Some(msg);
                    break;
                }
                first.get_or_insert(msg);
            }
            if let Some(h) = VECTORS.lock().unwrap().as_ref().and_then(|p| p.last_heading) {
                hdg = h as f64; // instant turn: worst case for the capture curve
            }
            let (nlat, nlon) = crate::nav::project_latlon(lat, lon, hdg, 100.0 * 10.0 / 3600.0);
            (lat, lon) = (nlat, nlon);
            alt = (alt - 80.0).max(target_alt_ft);
        }

        let first = first.expect("an opening vector");
        assert!(first.contains("fly heading"), "{first}");
        // Gibraltar speaks ICAO/CAP 413: altitudes are "descend to altitude
        // ... feet", and headings carry "degrees".
        assert!(
            first.contains("descend to altitude one thousand five hundred feet"),
            "{first}"
        );
        assert!(first.contains("degrees"), "{first}");
        let cleared = cleared.expect("never established on final in 40 sim-minutes");
        assert!(cleared.contains("runway zero niner"), "{cleared}");
        assert!(
            cleared.contains("Contact Tower one three one decimal two"),
            "{cleared}"
        );
        assert!(
            VECTORS.lock().unwrap().is_none(),
            "plan must be wiped once approach's job is done"
        );

        // The handoff lands on the tower: "final" alone earns the clearance.
        let clear = st.twr.handle(
            tower::parse_intent("november one seven two sierra papa, final"),
            &tower::Ctx {
                callsign: "November one seven two Sierra Papa".into(),
                airport: "Gibraltar".into(),
                runway: "zero niner".into(),
                qnh: "QNH one zero one seven".into(),
                wind: "calm".into(),
                holding_point: "Alpha".into(),
                approach: Some(atis::spoken_mhz(122800)),
                squawk: "seven zero zero zero".into(),
                final_traffic_nm: None,
                final_traffic_callsign: None,
                final_traffic_count: 0,
                runway_occupied: false,
                wake_secs_left: None,
                taxi_via: Vec::new(),
                taxi_crossings: Vec::new(),
                on_ground: false,
            },
        );
        assert!(clear.contains("cleared to land"), "{clear}");

        // Touchdown: the on-ground tick retires any leftover plan quietly.
        *VECTORS.lock().unwrap() = Some(VectorPlan {
            callsign: String::new(),
            thr_lat,
            thr_lon,
            course_deg: course,
            runway_spoken: String::new(),
            tower_spoken: None,
            target_alt_ft,
            ils: false,
            join_nm: 6.0,
            cleared: false,
            dialect: Dialect::Icao,
            last_tx: None,
            last_heading: None,
        });
        assert!(vector_tick(thr_lat, thr_lon, 15.0, None, true, t0).is_none());
        assert!(VECTORS.lock().unwrap().is_none(), "on-ground tick clears the plan");

        // --- The same arrival flown as an ILS. The clearance rides the
        // intercept turn ("cleared ILS approach..., report established"),
        // headings carry "degrees" (ICAO dialect), and the established
        // wedge hands off WITHOUT re-clearing.
        *VECTORS.lock().unwrap() = Some(VectorPlan {
            callsign: "November one seven two Sierra Papa".into(),
            thr_lat,
            thr_lon,
            course_deg: course,
            runway_spoken: "zero niner".into(),
            tower_spoken: Some(atis::spoken_mhz(st.tower_khz)),
            target_alt_ft,
            ils: true,
            join_nm: 6.0,
            cleared: false,
            dialect: Dialect::Icao,
            last_tx: None,
            last_heading: None,
        });
        let (mut lat, mut lon) = crate::nav::project_latlon(thr_lat, thr_lon, 0.0, 10.0);
        let mut alt = 3000.0_f64;
        let mut hdg = 180.0_f64;
        let mut cleared_ils: Option<String> = None;
        let mut handoff: Option<String> = None;
        // Orbit detector: consecutive vectors must never reverse the
        // aircraft (the pursuit-curve wind-up commanded 250 then 010 at
        // Jerez, 2026-07-12 — an orbit, not an intercept).
        let mut prev_cmd: Option<i64> = None;
        for step in 0..240u64 {
            let now = t0 + Duration::from_secs(7200 + step * 10);
            if let Some(msg) = vector_tick(lat, lon, alt, Some(hdg), false, now) {
                if msg.contains("fly heading") {
                    assert!(msg.contains("degrees"), "ICAO headings carry the unit: {msg}");
                }
                if msg.contains("Cleared ILS approach") {
                    cleared_ils.get_or_insert(msg.clone());
                }
                if msg.contains("Contact Tower") {
                    handoff = Some(msg);
                    break;
                }
            }
            if let Some(h) = VECTORS.lock().unwrap().as_ref().and_then(|p| p.last_heading) {
                if let Some(p) = prev_cmd {
                    assert!(
                        angle_diff_deg(h as f64, p as f64).abs() <= 120.0,
                        "orbit: vector {p} then {h}"
                    );
                }
                prev_cmd = Some(h);
                hdg = h as f64;
            }
            let (nlat, nlon) =
                crate::nav::project_latlon(lat, lon, hdg, 100.0 * 10.0 / 3600.0);
            (lat, lon) = (nlat, nlon);
            alt = (alt - 80.0).max(target_alt_ft);
        }
        let cleared_ils = cleared_ils.expect("ILS clearance never issued");
        assert!(
            cleared_ils.contains("report established"),
            "the intercept clearance asks for the report: {cleared_ils}"
        );
        let handoff = handoff.expect("no handoff after establishing");
        assert!(
            !handoff.contains("Cleared"),
            "already cleared on the intercept, don't re-clear: {handoff}"
        );
        assert!(VECTORS.lock().unwrap().is_none(), "ILS plan wiped after handoff");

        // --- Close-in arrival (the Jerez 2026-07-12 geometry): already at
        // platform altitude only 4 nm from the threshold, a mile east of the
        // centerline. The old pursuit curve orbited here; the fixed cut must
        // walk it onto final and hand off without ever reversing a vector.
        *VECTORS.lock().unwrap() = Some(VectorPlan {
            callsign: "November one seven two Sierra Papa".into(),
            thr_lat,
            thr_lon,
            course_deg: course,
            runway_spoken: "zero niner".into(),
            tower_spoken: Some(atis::spoken_mhz(st.tower_khz)),
            target_alt_ft,
            ils: true,
            join_nm: 8.6,
            cleared: false,
            dialect: Dialect::Icao,
            last_tx: None,
            last_heading: None,
        });
        let recip = (course + 180.0).rem_euclid(360.0);
        let (mut lat, mut lon) = {
            let (a, b) = crate::nav::project_latlon(thr_lat, thr_lon, recip, 4.0);
            crate::nav::project_latlon(a, b, (recip + 90.0).rem_euclid(360.0), 1.0)
        };
        let mut hdg = course;
        let mut alt = target_alt_ft;
        let mut prev_cmd: Option<i64> = None;
        let mut done: Option<String> = None;
        for step in 0..240u64 {
            let now = t0 + Duration::from_secs(14_400 + step * 10);
            if let Some(msg) = vector_tick(lat, lon, alt, Some(hdg), false, now) {
                if msg.contains("Contact Tower") {
                    done = Some(msg);
                    break;
                }
            }
            if let Some(h) = VECTORS.lock().unwrap().as_ref().and_then(|p| p.last_heading) {
                if let Some(p) = prev_cmd {
                    assert!(
                        angle_diff_deg(h as f64, p as f64).abs() <= 120.0,
                        "close-in orbit: vector {p} then {h}"
                    );
                }
                prev_cmd = Some(h);
                hdg = h as f64;
            }
            let (nlat, nlon) =
                crate::nav::project_latlon(lat, lon, hdg, 100.0 * 10.0 / 3600.0);
            (lat, lon) = (nlat, nlon);
            alt = (alt - 80.0).max(target_alt_ft);
        }
        done.expect("close-in arrival never handed off");
        assert!(VECTORS.lock().unwrap().is_none());

        // --- The tight base (Gibraltar 2026-07-13): the pilot rolls out on
        // final at 0.86 nm, below the vector altitude, pointed down the
        // runway. The old 1.0 nm wedge floor never saw it — the plan
        // vectored the short-final aircraft away and touchdown discarded it
        // with no clearance ever spoken, earning a compliant pilot the
        // landing berate. Inside a mile, landing intent IS established.
        let arm_short_final = || {
            *VECTORS.lock().unwrap() = Some(VectorPlan {
                callsign: "November one seven two Sierra Papa".into(),
                thr_lat,
                thr_lon,
                course_deg: course,
                runway_spoken: "zero niner".into(),
                tower_spoken: Some(atis::spoken_mhz(st.tower_khz)),
                target_alt_ft,
                ils: false,
                join_nm: 6.0,
                cleared: false,
                dialect: Dialect::Icao,
                last_tx: None,
                last_heading: None,
            });
        };
        arm_short_final();
        let (sf_lat, sf_lon) =
            crate::nav::project_latlon(thr_lat, thr_lon, recip + 3.0, 0.86);
        let short_final =
            vector_tick(sf_lat, sf_lon, target_alt_ft - 950.0, Some(course + 12.0), false, t0)
                .expect("short final earns the clearance, not a vector away");
        assert!(
            short_final.contains("Cleared visual approach runway zero niner"),
            "{short_final}"
        );
        assert!(short_final.contains("1 mile from touchdown"), "{short_final}");
        assert!(short_final.contains("Contact Tower"), "{short_final}");
        assert!(VECTORS.lock().unwrap().is_none(), "plan done after the clearance");

        // Inside a mile but NOT landing — crossing final perpendicular, and a
        // flyover at altitude — gets silence, never a reversal heading, and
        // the plan survives for when the geometry recovers.
        arm_short_final();
        let (x_lat, x_lon) = crate::nav::project_latlon(thr_lat, thr_lon, recip, 0.5);
        let crossing_track = (course + 90.0).rem_euclid(360.0);
        assert!(
            vector_tick(x_lat, x_lon, target_alt_ft - 950.0, Some(crossing_track), false, t0)
                .is_none(),
            "no vector inside a mile"
        );
        assert!(
            vector_tick(x_lat, x_lon, target_alt_ft + 500.0, Some(course), false, t0).is_none(),
            "a flyover at altitude is not established"
        );
        assert!(VECTORS.lock().unwrap().is_some(), "plan survives the blow-through");
        *VECTORS.lock().unwrap() = None;

        // --- The pilot who reports "established" earns the handoff from the
        // approach controller directly, and the plan dies with it.
        let ctx2 = tower::Ctx {
            callsign: "November one seven two Sierra Papa".into(),
            airport: "Gibraltar".into(),
            runway: "zero niner".into(),
            qnh: "QNH one zero one seven".into(),
            wind: "calm".into(),
            holding_point: "Alpha".into(),
            approach: Some(atis::spoken_mhz(122800)),
            squawk: "seven zero zero zero".into(),
            final_traffic_nm: None,
            final_traffic_callsign: None,
            final_traffic_count: 0,
            runway_occupied: false,
            wake_secs_left: None,
            taxi_via: Vec::new(),
            taxi_crossings: Vec::new(),
            on_ground: false,
        };
        let wx2 = Wx { wind_from_deg: 90.0, wind_kts: 8.0 };
        *VECTORS.lock().unwrap() = Some(VectorPlan {
            callsign: "November one seven two Sierra Papa".into(),
            thr_lat,
            thr_lon,
            course_deg: course,
            runway_spoken: "zero niner".into(),
            tower_spoken: Some(atis::spoken_mhz(st.tower_khz)),
            target_alt_ft,
            ils: true,
            join_nm: 6.0,
            cleared: true,
            dialect: Dialect::Icao,
            last_tx: None,
            last_heading: None,
        });
        let r = st.route_keyed(
            "gibraltar approach, established i l s runway zero niner",
            Service::Approach,
            &ctx2,
            wx2,
        );
        assert!(r.contains("Contact Gibraltar Tower"), "established → handoff: {r}");
        assert!(VECTORS.lock().unwrap().is_none(), "report kills the plan");

        // --- Arm phrasing per dialect, with real chart data on file: the
        // CIFP platform (2,000 ft here) outranks the geometric default.
        *ILS_FACTS.lock().unwrap() = Some(std::sync::Arc::new(IlsFacts {
            ils: vec![nav::AirportIls {
                runway: "09".into(),
                freq_10khz: 11030,
                course_mag: 90.0,
                gs_deg: Some(3.0),
            }],
            platform_ft: [("09".to_string(), 2000.0)].into_iter().collect(),
        }));
        let r = st.route_keyed("request vectors", Service::Approach, &ctx2, wx2);
        assert!(r.contains("Vectoring ILS approach runway zero niner"), "{r}");
        assert!(r.contains("Descend to altitude two thousand feet"), "CIFP platform: {r}");
        st.set_dialect(Dialect::Faa);
        let r = st.route_keyed("request vectors", Service::Approach, &ctx2, wx2);
        assert!(r.contains("Expect vectors ILS runway zero niner approach"), "{r}");
        assert!(r.contains("Descend and maintain two thousand"), "{r}");

        // Statics must not leak into other tests.
        *VECTORS.lock().unwrap() = None;
        *ILS_FACTS.lock().unwrap() = None;
        *CLEARANCE.lock().unwrap() = None;
    }

    #[test]
    fn ils_join_sits_under_the_glideslope() {
        // Heathrow: 2,500 ft platform over an 83 ft field, 3° slope — the
        // slope reaches ~2,417 AAL at ~7.6 nm, so the join must be past it.
        let join = ils_join_nm(2417.0, 3.0);
        assert!((8.0..10.0).contains(&join), "{join}");
        // At the join, the slope is ABOVE the platform: intercept from below.
        let slope_at_join_ft = (join - 1.0) * 6076.12 * 3.0_f64.to_radians().tan();
        assert!(slope_at_join_ft >= 2417.0 - 1.0, "{slope_at_join_ft}");
        // A low platform never pulls the join inside the visual's 6 nm.
        assert_eq!(ils_join_nm(800.0, 3.0), 6.0);
        // A degenerate slope angle can't divide toward infinity.
        assert!(ils_join_nm(2000.0, 0.0).is_finite());
    }

    #[test]
    fn phraseology_follows_geography() {
        assert_eq!(dialect_for("EGLL"), Dialect::Icao);
        assert_eq!(dialect_for("LXGB"), Dialect::Icao);
        assert_eq!(dialect_for("LEMD"), Dialect::Icao);
        assert_eq!(dialect_for("KORD"), Dialect::Faa);
        assert_eq!(dialect_for("PHNL"), Dialect::Faa);
        assert_eq!(dialect_for("CYYZ"), Dialect::Faa);
        // The pressure phrase speaks the field's own money: 1013.25 hPa is
        // "QNH one zero one three" in ICAO land and exactly 29.92 inches —
        // "altimeter two niner niner two" — at an FAA field.
        assert_eq!(spoken_pressure(101325.0, Dialect::Icao), "QNH one zero one three");
        assert_eq!(
            spoken_pressure(101325.0, Dialect::Faa),
            "altimeter two niner niner two"
        );
        assert_eq!(spoken_pressure(102000.0, Dialect::Icao), "QNH one zero two zero");
        // 1020 hPa ≈ 30.12 inHg.
        assert_eq!(
            spoken_pressure(102000.0, Dialect::Faa),
            "altimeter three zero one two"
        );
    }
}
