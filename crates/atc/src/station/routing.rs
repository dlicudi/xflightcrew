//! Routing a transmission to the tuned service, and the radio-call shape.

use super::*;

impl Station {
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
        self.try_handle_own(None, transcript, xp).await
    }

    /// [`Station::try_handle`] with our aircraft's state handed in by a
    /// caller that already holds it: the sim is read for the field (COM1,
    /// wind, pressure, traffic, callsign), never for our own aircraft.
    pub async fn try_handle_with(
        &mut self,
        state: &AircraftState,
        transcript: &str,
        xp: SimView<'_>,
    ) -> Option<(String, StationSpeaker)> {
        self.try_handle_own(Some(state), transcript, xp).await
    }

    /// Both doors of [`Station::try_handle`]: `state` `None` reads our own
    /// aircraft where `build_ctx` starts, after the word gates and COM1.
    async fn try_handle_own(
        &mut self,
        state: Option<&AircraftState>,
        transcript: &str,
        xp: SimView<'_>,
    ) -> Option<(String, StationSpeaker)> {
        // Cheap word gates before any dataref read — the common case is a
        // cockpit command, and it must cost nothing. route_unkeyed re-applies
        // the same gates; only the trace differs per exit. A call that names
        // our own airline callsign is addressed ("Speedbird three zero six,
        // ready to taxi").
        if !is_radio_call(transcript, &spoken_airport(&self.apt.name).to_lowercase())
            && !crate::callsign::addressed_to_own(transcript)
        {
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
        self.note_stated_runway(transcript, intent.clone());
        let (ctx, wx) = self.build_ctx(xp, state, intent.clone()).await;
        let before = self.desk.twr.state;
        let reply = self.route_unkeyed(transcript, service, &ctx, wx)?;
        sim::blackbox::record(
            "atc",
            serde_json::json!({
                "route": "unkeyed", "outcome": "reply", "service": service.name(),
                "transcript": transcript, "intent": format!("{intent:?}"),
                "state": format!("{before:?}>{:?}", self.desk.twr.state), "reply": reply,
            }),
        );
        Some((reply, service.speaker()))
    }

    /// The pure half of [`Station::try_handle`]: the unkeyed gates plus the
    /// tuned service's answer, no simulator anywhere. `None` falls through to
    /// the copilot.
    pub(super) fn route_unkeyed(
        &mut self,
        transcript: &str,
        service: Service,
        ctx: &tower::Ctx,
        wx: Wx,
    ) -> Option<String> {
        self.sync_landing_release();
        if !is_radio_call(transcript, &spoken_airport(&self.apt.name).to_lowercase())
            && !crate::callsign::addressed(transcript, &ctx.callsign)
        {
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
        Some(self.respond(transcript, service, intent, ctx, wx))
    }

    /// Remember "with information X" from any call — the next clearance skips
    /// the QNH when the letter matches the current broadcast.
    pub(super) fn note_information(&mut self, transcript: &str) {
        if let Some(l) = heard_information(&transcript.to_lowercase()) {
            tracing::info!(letter = %l, "pilot reports having the information");
            self.desk.pilot_info = Some(l);
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
        self.handle_transmission_own(None, transcript, xp).await
    }

    /// [`Station::handle_transmission`] with our aircraft's state handed in
    /// by a caller that already holds it (see [`Station::try_handle_with`]).
    pub async fn handle_transmission_with(
        &mut self,
        state: &AircraftState,
        transcript: &str,
        xp: SimView<'_>,
    ) -> Option<(String, StationSpeaker)> {
        self.handle_transmission_own(Some(state), transcript, xp).await
    }

    /// Both doors of [`Station::handle_transmission`]: `state` `None` reads
    /// our own aircraft where `build_ctx` starts, after COM1.
    async fn handle_transmission_own(
        &mut self,
        state: Option<&AircraftState>,
        transcript: &str,
        xp: SimView<'_>,
    ) -> Option<(String, StationSpeaker)> {
        let Some(service) = self.tuned_service(xp).await else {
            sim::blackbox::record(
                "atc",
                serde_json::json!({
                    "route": "keyed", "outcome": void_outcome(transcript, Instant::now()),
                    "transcript": transcript,
                }),
            );
            return None;
        };
        self.note_stated_runway(transcript, tower::parse_intent(transcript));
        let (ctx, wx) = self.build_ctx(xp, state, tower::parse_intent(transcript)).await;
        let before = self.desk.twr.state;
        let reply = self.route_keyed(transcript, service, &ctx, wx);
        sim::blackbox::record(
            "atc",
            serde_json::json!({
                "route": "keyed",
                "outcome": if reply.is_empty() { "silence" } else { "reply" },
                // A readback has no intent of its own ("Unknown"): its
                // silence is the controller accepting it (#176).
                "readback": reply.is_empty() && tower::is_readback(transcript),
                "service": service.name(), "transcript": transcript,
                "intent": format!("{:?}", tower::parse_intent(transcript)),
                "state": format!("{before:?}>{:?}", self.desk.twr.state), "reply": reply,
            }),
        );
        Some((reply, service.speaker()))
    }

    /// The pure half of [`Station::handle_transmission`]: everything after
    /// "somebody is listening on this frequency". Empty string = a readback
    /// the controller accepted (the frequency stays quiet).
    pub(super) fn route_keyed(
        &mut self,
        transcript: &str,
        service: Service,
        ctx: &tower::Ctx,
        wx: Wx,
    ) -> String {
        self.sync_landing_release();
        self.note_information(transcript);
        let intent = tower::parse_intent(transcript);
        // The readback gate first: a runway-critical clearance is waiting
        // for its echo. Correct → silence; wrong runway → correction;
        // anything else → the challenge (which repeats the instruction).
        // Runs before the generic readback-silence path so a WRONG runway is
        // caught, not waved through as "a readback".
        if service.aerodrome() {
            // Judged BEFORE the gate spends it: with enforcement off, an IFR
            // readback that said any of the clearance stands as read.
            let ifr_read = self.desk.twr.pending().is_some_and(|p| {
                matches!(p.awaited, tower::Awaited::IfrClearance { .. })
                    && tower::check_readback(transcript, &p.awaited)
                        != tower::ReadbackCheck::NotAReadback
            });
            if let Some(reply) = self.desk.twr.gate(transcript, intent.clone(), ctx) {
                if reply.is_empty() {
                    // Correct readback: the debt is paid; tell the coach.
                    if let Some(c) = self.desk.strip.clearance.lock().unwrap_or_else(|e| e.into_inner()).as_mut() {
                        c.pending_readback = None;
                    }
                    if let Some(ifr) = self.desk.twr.take_accepted_ifr() {
                        publish_ifr_clearance(&self.desk.strip, ifr, &ctx.callsign, self.desk.ifr_bearings, self.desk.ifr_sid_basis.as_ref());
                    }
                    return String::new();
                }
                if self.enforce_readback {
                    return reply;
                }
                if ifr_read {
                    self.desk.twr.accept_ifr_unenforced();
                    if let Some(ifr) = self.desk.twr.take_accepted_ifr() {
                        publish_ifr_clearance(&self.desk.strip, ifr, &ctx.callsign, self.desk.ifr_bearings, self.desk.ifr_sid_basis.as_ref());
                    }
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
        // An instrument arrival's first call to approach is its check-in
        // ("information Bravo, descending flight level eight zero"), never a
        // readback: it is cleared for the approach (`arrival_check_in`).
        if !echoes_clearance
            && intent == tower::Intent::Unknown
            && self.arrival_check_in(transcript, service, ctx)
        {
            let landing = tower::Intent::RequestLanding { touch_and_go: false };
            return self.respond(transcript, service, landing, ctx, wx);
        }
        if echoes_clearance || (intent == tower::Intent::Unknown && tower::is_readback(transcript))
        {
            // The position the tower handed us to takes the check-in: a
            // field without a departure frequency hands a departure to
            // APPROACH, and until EGBB (2026-09-27) approach heard "passing
            // three thousand one hundred climbing one five thousand" as a
            // readback and said nothing, the tower still holding the flight
            // cleared for takeoff.
            let checks_in = service == Service::Departure || self.desk.handed_to == Some(service);
            if checks_in && !self.desk.dep_checked_in {
                self.desk.dep_checked_in = true;
                self.transfer_departure();
                // This reply IS the radar contact: the watch has nothing
                // left to say.
                radar_contact_at_check_in(&self.desk.strip);
                let position = match service {
                    Service::Approach => "Approach",
                    _ => "Departure",
                };
                // The climb the pilot announced is the climb they get: the
                // zone clearance capped them at two thousand, and "radar
                // contact" alone left "climbing five thousand" unanswered
                // (LXGB, 2026-09-15).
                let cap = self.filed_cruise_ft();
                return match climb_announced(transcript)
                    .map(|alt| capped_departure_climb(&alt, cap, ctx.transition_level_ft))
                {
                    Some(alt) => format!(
                        "{}, {} {position}, radar contact, {}.",
                        ctx.callsign,
                        ctx.airport,
                        check_in_climb(&alt, self.dialect)
                    ),
                    None => format!(
                        "{}, {} {position}, radar contact.",
                        ctx.callsign, ctx.airport
                    ),
                };
            }
            // The departure clearance's suggested readback is paid.
            self.desk.twr.clear_clearance_readback();
            if let Some(c) = self.desk.strip.clearance.lock().unwrap_or_else(|e| e.into_inner()).as_mut() {
                c.pending_readback = None;
            }
            return String::new();
        }
        self.respond(transcript, service, intent, ctx, wx)
    }

    /// [`Station::route_keyed`] for one flight. Ours is answered on the desk
    /// already on frequency; a traffic flight's desk is swapped in for the
    /// synchronous answer only and parked again on the way out, by a drop
    /// guard, so nothing can leave a traffic desk on frequency. Its replies
    /// write its own strip's cells. `None` for a traffic number with no open
    /// strip.
    pub(super) fn respond_for(
        &mut self,
        id: FlightId,
        transcript: &str,
        service: Service,
        ctx: &tower::Ctx,
        wx: Wx,
    ) -> Option<String> {
        self.with_desk(id, |st| st.route_keyed(transcript, service, ctx, wx))
    }

    /// Run `f` with this flight's desk on frequency: ours is already there;
    /// a traffic flight's is swapped in (a fresh one for a strip it has not
    /// worked, or for a strip reopened since its desk was parked) and parked
    /// again on the way out by the drop guard, on every path. Synchronous by
    /// construction: nothing can await inside `f`. `None` for a traffic
    /// number with no open strip.
    pub(super) fn with_desk<R>(
        &mut self,
        id: FlightId,
        f: impl FnOnce(&mut Station) -> R,
    ) -> Option<R> {
        let FlightId::Traffic(n) = id else {
            return Some(f(self));
        };
        let open = traffic_strip(n)?;
        let desk = match self.desks.remove(&id) {
            Some(d) if matches!(&d.strip, StripRef::Traffic(s) if std::sync::Arc::ptr_eq(s, &open)) => d,
            _ => StripDesk::for_strip(StripRef::Traffic(open)),
        };
        let swap = DeskSwap::new(self, id, desk);
        Some(f(&mut *swap.st))
    }

    /// The voice of whichever of this station's positions COM1 is tuned to.
    pub async fn tuned_speaker(&self, xp: SimView<'_>) -> Option<StationSpeaker> {
        self.tuned_service(xp).await.map(Service::speaker)
    }

    /// Does this frequency (kHz) belong to one of the station's services?
    /// Same 5 kHz tolerance as [`Station::tuned_service`], no dataref read —
    /// callers with a live COM1 value (the STT prompt switch) check for free.
    pub fn covers_khz(&self, khz: i64) -> bool {
        self.service_on(khz).is_some()
    }

    /// The position on frequency `khz` (5 kHz tolerance), tower first.
    pub(super) fn service_on(&self, khz: i64) -> Option<Service> {
        let on = |k: Option<i64>| k.is_some_and(|k| (khz - k).abs() <= 5);
        [
            (Some(self.tower_khz), Service::Tower),
            (self.approach_khz, Service::Approach),
            (self.departure_khz, Service::Departure),
            (self.ground_khz, Service::Ground),
            (self.delivery_khz, Service::Delivery),
        ]
        .into_iter()
        .find(|(k, _)| on(*k))
        .map(|(_, s)| s)
    }

    /// The frequency of one of this station's positions.
    pub(super) fn khz_of(&self, service: Service) -> Option<i64> {
        match service {
            Service::Tower => Some(self.tower_khz),
            Service::Approach => self.approach_khz,
            Service::Departure => self.departure_khz,
            Service::Ground => self.ground_khz,
            Service::Delivery => self.delivery_khz,
        }
    }

    /// Which service COM1 is tuned to. 8.33 channel names can sit 5 kHz off
    /// the carrier; adjacent channels are 8 kHz apart, so 5 can't cross.
    async fn tuned_service(&self, xp: SimView<'_>) -> Option<Service> {
        // A field beyond reach hears nothing, whatever is tuned: Gibraltar
        // cleared a 172 parked at LILB, 1,000 nm away, and handed it a taxi
        // route to its own runway 09 (2026-09-25).
        if station_unavailable() {
            return None;
        }
        let com1 = xp.get_float(datarefs::COM1_FREQ_KHZ).await.ok()?.round() as i64;
        self.service_on(com1)
    }
}

/// Does an UNKEYED utterance sound like a radio call? Addressed to a station
/// by name, or built from call vocabulary a pilot only uses on the radio.
/// This is the content-routing gate: with the radio PTT keyed, keying itself
/// is the addressing and this is never consulted.
/// A departure check-in's climb, in the dialect, from the level as spoken
/// ("six thousand", "flight level eight zero", whatever the check-in
/// announced or [`capped_departure_climb`] capped it to). ICAO / CAP 413:
/// "climb altitude six thousand feet", "climb flight level eight zero";
/// FAA: "climb and maintain six thousand". It was "climb six thousand"
/// at every field (Gibraltar Departure, the realism review, 2026-10-09).
pub(super) fn check_in_climb(level: &str, dialect: Dialect) -> String {
    let level = level.trim();
    let bare = level
        .strip_suffix(" feet")
        .or_else(|| level.strip_suffix(" ft"))
        .unwrap_or(level);
    let fl = bare.to_lowercase().starts_with("flight level");
    match (dialect, fl) {
        (Dialect::Faa, _) => format!("climb and maintain {bare}"),
        (Dialect::Icao, true) => format!("climb {bare}"),
        (Dialect::Icao, false) => format!("climb altitude {bare} feet"),
    }
}

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
        || t.contains("ready for push")
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
        // "Runway vacated" is radio speech for the same reason a pattern call
        // is: nobody tells their copilot they have left the runway, and the
        // tower is the one party that needs to know. It earns an
        // acknowledgement and nothing else, so routing it here cannot buy a
        // clearance. "Clear of the runway" needs the runway word with it —
        // "clear left", "all clear" and "clear to taxi" are not this call.
        || word("vacated")
        || word("vacating")
        || t.contains("clear of the runway")
        || t.contains("clear of runway")
        // The IFR pilot's whole request is "ready to copy": nobody says it
        // to their copilot. "IFR" alone is not radio speech ("we're IFR
        // today" is crew talk); the IFR request carries "request" anyway.
        || t.contains("ready to copy")
        // "Cancel IFR" is said to a controller, never to the copilot.
        || tower::parse_intent(t) == tower::Intent::CancelIfr
        // "Request the Lam five Papa departure" names a procedure only a
        // controller issues.
        || tower::requested_departure(t).is_some()
}

/// A traffic flight's desk on frequency for one synchronous answer: our own
/// desk parked here, put back (and the traffic desk parked in
/// `Station::desks`) when the guard drops, on every path out.
struct DeskSwap<'a> {
    st: &'a mut Station,
    id: FlightId,
    own: Option<StripDesk>,
}

impl<'a> DeskSwap<'a> {
    fn new(st: &'a mut Station, id: FlightId, desk: StripDesk) -> DeskSwap<'a> {
        let own = std::mem::replace(&mut st.desk, desk);
        DeskSwap {
            st,
            id,
            own: Some(own),
        }
    }
}

impl Drop for DeskSwap<'_> {
    fn drop(&mut self) {
        if let Some(own) = self.own.take() {
            let traffic = std::mem::replace(&mut self.st.desk, own);
            self.st.desks.insert(self.id, traffic);
        }
    }
}
