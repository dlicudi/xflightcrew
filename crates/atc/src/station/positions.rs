//! The aerodrome's positions: Delivery (apt.dat 1052), Ground (1053) and
//! the Tower (1054) as VIEWS of one station. Until 2026-10-10 one
//! "Gatwick Tower" gave the clearance, the push, the taxi out, the takeoff
//! and the taxi in, while EGKK's apt.dat lists "1052 121955 Gatwick
//! Delivery", "1053 121805 Gatwick Ground" and "1054 124230 Gatwick Tower".
//!
//! The desk (the tower FSM and the worksheet) is the flight's, shared by
//! the three: a position only decides WHO answers ([`Station::owner`]),
//! and the strip moves with its handoffs ([`FlightStrip::working`]). So the
//! runway book, the readbacks and the strips work as they did.
//!
//! Who owns what (ICAO Doc 4444 7.5 and 7.6, CAP 413 ch. 4, FAA 7110.65
//! 3-7): the departure clearance is Delivery's, else Ground's, else the
//! Tower's; push, start, taxi out and taxi in are Ground's; line-up,
//! takeoff, landing, the circuit and a crossing of the runway in use are
//! the Tower's. A field without the row works as before: the Tower does it.

use super::*;

/// Ground hands a departure to the Tower this close to the end of its
/// taxi route (the holding point), metres: as it approaches the hold, so
/// its first call to the Tower is the ready call, never a taxi request.
pub const HOLD_HANDOFF_M: f64 = 150.0;

impl Station {
    /// The name a position keys ("Gatwick Ground"): the Tower, Approach and
    /// Departure by the field's radio name, Ground and Delivery by their
    /// own rows ([`nav::AirportInfo::position_name`]).
    pub(super) fn service_name(&self, s: Service) -> String {
        let field = self.apt.radio_name();
        let row = |kind: u32, word: &str| {
            self.apt
                .position_name(kind, word)
                .unwrap_or_else(|| format!("{field} {word}"))
        };
        match s {
            Service::Tower => format!("{field} Tower"),
            Service::Approach => format!("{field} Approach"),
            Service::Departure => format!("{field} Departure"),
            Service::Ground => row(nav::FREQ_GROUND, "Ground"),
            Service::Delivery => row(nav::FREQ_CLEARANCE, "Delivery"),
        }
    }

    /// Every position this station runs, by the voice it speaks with and
    /// its name, for the ticks that speak as one ([`FieldFacts`]).
    pub(super) fn position_names(&self) -> Vec<(StationSpeaker, String)> {
        [
            Service::Tower,
            Service::Approach,
            Service::Departure,
            Service::Ground,
            Service::Delivery,
        ]
        .into_iter()
        .filter(|s| self.khz_of(*s).is_some())
        .map(|s| (s.speaker(), self.service_name(s)))
        .collect()
    }

    /// Who works a flight on the ground: Ground where the field has one,
    /// else the Tower.
    pub(super) fn ground_service(&self) -> Service {
        if self.ground_khz.is_some() {
            Service::Ground
        } else {
            Service::Tower
        }
    }

    /// Who gives the departure clearance: Delivery, else Ground, else the
    /// Tower.
    pub(super) fn clearance_service(&self) -> Service {
        if self.delivery_khz.is_some() {
            Service::Delivery
        } else {
            self.ground_service()
        }
    }

    /// The aerodrome position that OWNS this call, judged on the desk on
    /// frequency (its FSM state) and the call's context. `None`: whoever
    /// hears it answers it (a radio check, a wind check, a hail, an
    /// emergency, a readback). Pure.
    pub(super) fn owner(&self, intent: &tower::Intent, ctx: &tower::Ctx) -> Option<Service> {
        use tower::Intent as I;
        use tower::State as S;
        let gnd = self.ground_service();
        let state = self.desk.twr.state;
        Some(match intent {
            I::RequestIfrClearance { .. } => self.clearance_service(),
            I::RequestClearance if ctx.on_ground => self.clearance_service(),
            I::RequestStartup | I::RequestPushback | I::RequestShutdown => gnd,
            I::RequestTaxi | I::RequestTaxiIn if ctx.on_ground => gnd,
            // A runway asked for from the stand is a taxi there.
            I::RequestRunway if ctx.on_ground && state == S::Parked => gnd,
            // A ready call from the stand is answered with the taxi: Ground's.
            I::RequestTakeoff
                if ctx.on_ground && state == S::Parked && !ctx.own_on_runway && !ctx.helicopter =>
            {
                gnd
            }
            // A crossing of the runway in use is the Tower's; any other
            // strip is crossed on Ground's word, judged by the same book.
            I::RequestCrossing => {
                if self.crossing_runway_in_use() {
                    Service::Tower
                } else {
                    gnd
                }
            }
            I::RequestTakeoff
            | I::RequestBacktrack
            | I::RejectingTakeoff
            | I::RunwayVacated
            | I::RequestLanding { .. }
            | I::ReportFinal
            | I::ReportDownwind
            | I::ReportBase
            | I::ReportCrosswind
            | I::GoAround
            | I::RequestTransit => Service::Tower,
            _ => return None,
        })
    }

    /// The crossing this call asks for is of the strip in use.
    fn crossing_runway_in_use(&self) -> bool {
        let Some(asked) = self.desk.call.crossing_request.as_deref() else {
            return false;
        };
        let Some(in_use) = self.runway_in_use.as_deref() else {
            return false;
        };
        let strip_of = |id: &str| {
            self.apt.runways.iter().position(|r| {
                tower::runway_matches(&r.ident_a, id) || tower::runway_matches(&r.ident_b, id)
            })
        };
        strip_of(asked).is_some() && strip_of(asked) == strip_of(in_use)
    }

    /// "Contact X on F": the position `to`'s handoff line, in the dialect.
    pub(super) fn contact_line(&self, callsign: &str, to: Service) -> Option<String> {
        let khz = self.khz_of(to)?;
        Some(format!(
            "{callsign}, contact {} on {}.",
            self.service_name(to),
            atis::spoken_mhz_in(khz, self.dialect)
        ))
    }

    /// A call made on the wrong aerodrome position's frequency is answered
    /// as a real controller answers it: "Speedbird one two three, contact
    /// Gatwick Ground on one two one decimal eight zero five." Never taken
    /// over the other position's head: the position that owns it holds its
    /// half of the picture (the taxi is planned against Ground's traffic,
    /// the takeoff against the Tower's final), and a pilot who called the
    /// wrong frequency is told where to call, which is what a controller
    /// says (CAP 413 4.1, FAA 7110.65 2-1-17). The trivial calls (a radio
    /// check, a wind check, a hail) are answered where they are heard.
    /// Our flight's handoff is published for the crew that works the radio.
    pub(super) fn redirect(
        &mut self,
        service: Service,
        intent: &tower::Intent,
        ctx: &tower::Ctx,
    ) -> Option<String> {
        if !service.aerodrome() {
            return None;
        }
        let owner = self.owner(intent, ctx)?;
        if owner == service {
            return None;
        }
        let line = self.contact_line(&ctx.callsign, owner)?;
        sim::blackbox::record(
            "atc",
            serde_json::json!({"event": "wrong_position", "heard_on": service.name(),
                "owner": owner.name(), "intent": format!("{intent:?}")}),
        );
        self.hand_to(owner);
        Some(line)
    }

    /// The flight is handed to `to`: its strip says so, and our crew is
    /// told where to call.
    pub(super) fn hand_to(&mut self, to: Service) {
        *self
            .desk
            .strip
            .working
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = Some(to.speaker());
        if !self.desk.strip.is_traffic() {
            if let Some(khz) = self.khz_of(to) {
                *self
                    .desk
                    .strip
                    .position_handoff
                    .lock()
                    .unwrap_or_else(|e| e.into_inner()) = Some((self.service_name(to), khz));
            }
        }
    }

    /// The position working the flight on the desk: the last that answered
    /// it or that it was handed to.
    pub(super) fn working(&self) -> Option<Service> {
        self.desk
            .strip
            .working
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .and_then(Service::of_speaker)
    }

    /// The position answering this transmission now works the flight.
    pub(super) fn note_working(&mut self, service: Service) {
        if service.aerodrome() {
            *self
                .desk
                .strip
                .working
                .lock()
                .unwrap_or_else(|e| e.into_inner()) = Some(service.speaker());
        }
    }

    /// What the answering position adds to its reply. Delivery's clearance
    /// ends with where to call next ("When ready, contact Gatwick Ground on
    /// one two one decimal eight zero five."); the Tower's acknowledgement
    /// of "runway vacated" is the handoff to Ground ("Speedbird one two
    /// three, contact Gatwick Ground on ..."), as a tower clears its
    /// frequency of an aircraft off the runway (CAP 413 4.5).
    pub(super) fn position_reply(
        &mut self,
        service: Service,
        intent: &tower::Intent,
        ctx: &tower::Ctx,
        reply: String,
    ) -> String {
        use tower::Intent as I;
        match (service, intent) {
            (Service::Delivery, I::RequestIfrClearance { .. } | I::RequestClearance)
                if issued_clearance(intent, ctx) =>
            {
                let next = self.ground_service();
                match self.khz_of(next) {
                    Some(khz) => format!(
                        "{reply} When ready, contact {} on {}.",
                        self.service_name(next),
                        atis::spoken_mhz_in(khz, self.dialect)
                    ),
                    None => reply,
                }
            }
            (Service::Tower, I::RunwayVacated)
                if self.ground_khz.is_some() && ctx.on_ground && self.desk.twr.runway_vacated() =>
            {
                let line = self.contact_line(&ctx.callsign, Service::Ground);
                self.hand_to(Service::Ground);
                line.unwrap_or(reply)
            }
            // A position's own name in its own answers.
            (Service::Ground | Service::Delivery, I::RadioCheck) => format!(
                "{}, {}, reading you five.",
                ctx.callsign,
                self.service_name(service)
            ),
            (Service::Ground | Service::Delivery, I::Greeting) => format!(
                "{}, {}, pass your message.",
                ctx.callsign,
                self.service_name(service)
            ),
            (_, I::RequestPushback) => self.push_line(ctx),
            _ => reply,
        }
    }

    /// Ground's push approval, the way the aircraft will face once pushed
    /// said in the dialect: ICAO "push and start approved, facing west",
    /// CAP 413 "push and start approved, tail east", FAA "push back
    /// approved, tail east" (FAA ground approves no start). A bare "request
    /// push back" (no start asked) is approved as a push. Published on the
    /// strip for the crew and the traffic ([`FlightStrip::push_approved`]).
    pub(super) fn push_line(&mut self, ctx: &tower::Ctx) -> String {
        let facing = self.desk.call.push_facing.take();
        let start = self.desk.call.push_start;
        *self
            .desk
            .strip
            .push_approved
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = Some(facing);
        let what = match (ctx.dialect, start) {
            (Dialect::Faa, _) | (Dialect::Icao, false) => "push back approved",
            (Dialect::Icao, true) => "push and start approved",
        };
        let way = facing
            .map(|f| match (ctx.dialect, ctx.uk) {
                (Dialect::Icao, false) => format!(", facing {}", compass_word(f)),
                _ => format!(", tail {}", compass_word(f + 180.0)),
            })
            .unwrap_or_default();
        format!("{}, {what}{way}.", ctx.callsign)
    }
}

/// The clearance call was answered WITH a clearance: an IFR offer made, or
/// a zone (or circuit) clearance not declined. "Say destination" and a
/// declined circuit hand nobody on.
fn issued_clearance(intent: &tower::Intent, ctx: &tower::Ctx) -> bool {
    match intent {
        tower::Intent::RequestIfrClearance { .. } => ctx.ifr.is_some(),
        _ => !ctx.circuit.as_ref().is_some_and(|c| c.not_available),
    }
}

/// A bearing as the eight compass words a controller uses ("west", "north
/// east").
pub fn compass_word(deg: f64) -> &'static str {
    const WORDS: [&str; 8] = [
        "north",
        "north east",
        "east",
        "south east",
        "south",
        "south west",
        "west",
        "north west",
    ];
    WORDS[((deg.rem_euclid(360.0) + 22.5) / 45.0) as usize % 8]
}

/// The way a push from (`lat`, `lon`) facing `psi` leaves the aircraft
/// facing, degrees true: off the end of the stand's drawn push lane
/// ([`nav::TaxiNet::push_lane`]), toward where its departure's taxi route
/// goes on from there (`route`, the planned route to the holding point), so
/// the push turns the tail AWAY from the way out. With no route passing the
/// lane's end, nose back toward the stand along the lane (a straight push).
/// `None` with no lane drawn.
pub fn push_facing_deg(
    net: &nav::TaxiNet,
    lat: f64,
    lon: f64,
    psi: f64,
    route: Option<&[(f64, f64)]>,
) -> Option<f64> {
    let lane = net.push_lane_or_back(lat, lon, psi)?;
    let end = *lane.last()?;
    let straight = lane
        .len()
        .checked_sub(2)
        .map(|i| bearing_deg(end, lane[i]))
        .unwrap_or(psi);
    let Some(path) = route.filter(|p| p.len() >= 2) else {
        return Some(straight);
    };
    // The route's nearest point to the lane's end, then 40 m on along it.
    let (i, d) = path
        .iter()
        .enumerate()
        .map(|(i, p)| (i, nav::haversine_nm(end.0, end.1, p.0, p.1) * 1852.0))
        .min_by(|a, b| a.1.total_cmp(&b.1))?;
    if d > 60.0 {
        return Some(straight);
    }
    let mut gone = 0.0;
    let mut k = i;
    while k + 1 < path.len() && gone < 40.0 {
        gone += nav::haversine_nm(path[k].0, path[k].1, path[k + 1].0, path[k + 1].1) * 1852.0;
        k += 1;
    }
    let ahead = path[k];
    if nav::haversine_nm(end.0, end.1, ahead.0, ahead.1) * 1852.0 < 10.0 {
        return Some(straight);
    }
    let facing = bearing_deg(end, ahead);
    // A tail swung more than a right angle (and a little) off the lane's
    // own way is a hairpin no tow makes in a taxiway's width: pushed
    // straight back instead (the traffic's push reads the same rule).
    let tail_turn = ((facing - straight + 540.0).rem_euclid(360.0) - 180.0).abs();
    Some(if tail_turn > 110.0 { straight } else { facing })
}

/// Flat-earth bearing from `a` to `b`, degrees true.
fn bearing_deg(a: (f64, f64), b: (f64, f64)) -> f64 {
    let e = (b.1 - a.1) * a.0.to_radians().cos();
    let n = b.0 - a.0;
    e.atan2(n).to_degrees().rem_euclid(360.0)
}

/// Ground's handoff at the holding point, for one strip: a departure
/// Ground cleared to taxi, within [`HOLD_HANDOFF_M`] of its route's end, is
/// told "hold short runway two six left, contact Gatwick Tower on one two
/// four decimal two three zero" (CAP 413 4.3; FAA 7110.65 3-7-2 the same
/// handoff, where "monitor" would need a tower that calls first, which
/// this one never does). Once: the strip then works the Tower. Ours
/// publishes the handoff for the crew. `None` at a field without Ground,
/// or with nothing to say.
pub(super) fn ground_handoff_tick_for(strip: &FlightStrip, lat: f64, lon: f64) -> Option<String> {
    let working = *strip.working.lock().unwrap_or_else(|e| e.into_inner());
    if working != Some(StationSpeaker::Ground) {
        return None;
    }
    let (tower_name, tower_khz) = active_position(StationSpeaker::Tower)?;
    let (state, callsign, runway) = strip
        .clearance
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|c| (c.state, c.callsign.clone(), c.runway_ident.clone()))?;
    if state != tower::State::TaxiCleared
        || *strip
            .taxi_to_stand
            .lock()
            .unwrap_or_else(|e| e.into_inner())
    {
        return None;
    }
    let hold = strip
        .taxi_route
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .and_then(|r| r.path.last().copied())?;
    if nav::haversine_nm(lat, lon, hold.0, hold.1) * 1852.0 > HOLD_HANDOFF_M {
        return None;
    }
    let (dialect, _) = installed_phraseology();
    *strip.working.lock().unwrap_or_else(|e| e.into_inner()) = Some(StationSpeaker::Tower);
    if strip.is_own() {
        *strip
            .position_handoff
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = Some((tower_name.clone(), tower_khz));
    }
    let hold_short = if runway.is_empty() {
        String::new()
    } else {
        format!("hold short runway {}, ", atis::spoken_runway(&runway))
    };
    let line = format!(
        "{callsign}, {hold_short}contact {tower_name} on {}.",
        atis::spoken_mhz_in(tower_khz, dialect)
    );
    sim::blackbox::record(
        "atc",
        serde_json::json!({"event": "ground_handoff", "to": tower_name, "khz": tower_khz,
            "runway": runway, "traffic": !strip.is_own()}),
    );
    Some(line)
}

/// [`ground_handoff_tick_for`] for our own flight, from the monitor's tick.
pub fn ground_handoff_tick(lat: f64, lon: f64, on_ground: bool) -> Option<String> {
    if !on_ground {
        return None;
    }
    ground_handoff_tick_for(&OWN_STRIP, lat, lon)
}

/// The Tower hands a landing to Ground as it slows on the runway: "when
/// vacated, contact Gatwick Ground on one two one decimal eight zero five"
/// (CAP 413 4.5), so its "runway vacated, request taxi" goes to Ground. A
/// flight cleared to land, on the ground below [`VACATE_HANDOFF_KTS`],
/// working the Tower, at a field with Ground. Once: the strip then works
/// Ground.
fn vacate_handoff_tick_for(strip: &FlightStrip, state: &AircraftState) -> Option<String> {
    let working = *strip.working.lock().unwrap_or_else(|e| e.into_inner());
    if working != Some(StationSpeaker::Tower)
        || !state.on_ground
        || state.gs_kts > VACATE_HANDOFF_KTS
    {
        return None;
    }
    let (ground_name, ground_khz) = active_position(StationSpeaker::Ground)?;
    let callsign = strip
        .clearance
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .filter(|c| c.state == tower::State::ClearedToLand)
        .map(|c| c.callsign.clone())?;
    let (dialect, _) = installed_phraseology();
    *strip.working.lock().unwrap_or_else(|e| e.into_inner()) = Some(StationSpeaker::Ground);
    if strip.is_own() {
        *strip
            .position_handoff
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = Some((ground_name.clone(), ground_khz));
    }
    sim::blackbox::record(
        "atc",
        serde_json::json!({"event": "vacate_handoff", "to": ground_name, "khz": ground_khz,
            "traffic": !strip.is_own()}),
    );
    Some(format!(
        "{callsign}, when vacated, contact {ground_name} on {}.",
        atis::spoken_mhz_in(ground_khz, dialect)
    ))
}

/// A landing is handed to Ground once it has slowed to this on the runway,
/// knots: a jet's rollout reaches it a few hundred metres before its exit.
pub const VACATE_HANDOFF_KTS: f64 = 40.0;

/// The aerodrome positions' handoffs for traffic flight `id`, where it was
/// last seen: Ground's to the Tower at the holding point, the Tower's to
/// Ground on the rollout. The line and the position that says it.
pub fn traffic_position_tick(id: u32) -> Option<(String, StationSpeaker)> {
    let strip = traffic_strip(id)?;
    let state = seen(&strip)?;
    if !state.on_ground {
        return None;
    }
    if let Some(line) = ground_handoff_tick_for(&strip, state.lat, state.lon) {
        return Some((line, StationSpeaker::Ground));
    }
    vacate_handoff_tick_for(&strip, &state).map(|l| (l, StationSpeaker::Tower))
}

/// The position traffic flight `id` is working ([`FlightStrip::working`]),
/// for the speech side: its readback is said on that frequency.
pub fn traffic_working(id: u32) -> Option<StationSpeaker> {
    let strip = traffic_strip(id)?;
    let w = *strip.working.lock().unwrap_or_else(|e| e.into_inner());
    w
}
