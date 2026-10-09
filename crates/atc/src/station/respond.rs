//! The controller's replies: departure, IFR, circuit, squawk, approach, vectors.

use super::*;

impl Station {
    /// Answer one transmission on whichever service is tuned. Pure: the
    /// context and wind were read before the words reached the controller.
    pub(super) fn respond(
        &mut self,
        transcript: &str,
        service: Service,
        intent: tower::Intent,
        ctx: &tower::Ctx,
        wx: Wx,
    ) -> String {
        self.sync_landing_release();
        let was = self.desk.twr.state;
        if service == Service::Tower {
            self.note_departure_intent(&intent, transcript);
        }
        // A zone request from circuit traffic (noted just above, from THIS
        // call's words) is answered with the circuit clearance: a circuit
        // never leaves the zone (LEMG 13, sorties 1790590663 and 1790591256,
        // 2026-09-28: "for circuits" got "cleared to leave the Malaga zone").
        let circuit_ctx;
        let ctx = if service == Service::Tower
            && intent == tower::Intent::RequestClearance
            && self.desk.circuits
        {
            let offer = self.circuit_offer(&ctx.runway);
            // Declined now (no circuits at the field); asked again, cleared.
            if offer.not_available {
                self.desk.circuits_declined = true;
            }
            circuit_ctx = tower::Ctx {
                circuit: Some(offer),
                ..ctx.clone()
            };
            &circuit_ctx
        } else {
            ctx
        };
        let declined = ctx.circuit.as_ref().is_some_and(|c| c.not_available);
        // The pressure once per contact: a phrase this flight was already
        // given is not repeated in the next clearance of the exchange (an
        // IFR clearance and then the taxi clearance both said "QNH one zero
        // one seven"). Asked for ("say QNH", a wind check), it is said.
        let qnh_ctx;
        let ctx = if service == Service::Tower
            && !ctx.qnh.is_empty()
            && self.desk.qnh_given.as_deref() == Some(ctx.qnh.as_str())
            && intent != tower::Intent::WindCheck
        {
            qnh_ctx = tower::Ctx {
                qnh: String::new(),
                ..ctx.clone()
            };
            &qnh_ctx
        } else {
            ctx
        };
        // A departure clearance in a RADAR environment carries a discrete
        // code — the zone clearance is where a real controller assigns it
        // (Heathrow flew a whole day on 7000, 2026-07-10). Tower-only strips
        // keep the VFR conspicuity. The watch's "radar contact" belongs to
        // whoever has the scope.
        let assigned_ctx;
        // A declined circuit is cleared for nothing, so it is given no code.
        let ctx = if service == Service::Tower
            && intent == tower::Intent::RequestClearance
            && !declined
            && self.desk.assigned_squawk.is_none()
            && (self.approach_khz.is_some() || self.departure_khz.is_some())
        {
            let scope = if self.departure_khz.is_some() {
                StationSpeaker::Departure
            } else {
                StationSpeaker::Approach
            };
            let spoken = self.assign_squawk(&ctx.callsign, scope);
            assigned_ctx = tower::Ctx {
                squawk: spoken,
                ..ctx.clone()
            };
            &assigned_ctx
        } else {
            ctx
        };
        // An IFR request to a destination the index knows: the station
        // makes up the clearance (SID, altitude, code) and the tower
        // phrases it. A destination it cannot find leaves `ifr` empty and
        // the tower asks again.
        let ifr_ctx;
        let ctx = match (&intent, service) {
            (tower::Intent::RequestIfrClearance { dest: Some(d) }, Service::Tower) => {
                match self.ifr_offer(d, ctx) {
                    Some(offer) => {
                        ifr_ctx = tower::Ctx {
                            squawk: atis::spoken_digits(&format!("{:04}", offer.clearance.squawk)),
                            ifr: Some(offer),
                            ..ctx.clone()
                        };
                        &ifr_ctx
                    }
                    None => ctx,
                }
            }
            _ => ctx,
        };
        let reply = self.respond_inner(transcript, service, intent.clone(), ctx, wx);
        let reply = if service == Service::Tower {
            self.state_runway_change(&intent, ctx, reply)
        } else {
            reply
        };
        let reply = if service == Service::Tower {
            self.note_pressure_given(ctx, reply)
        } else {
            reply
        };
        // A granted runway request makes that runway the one in use; the
        // taxi (or the join) it bought freezes the pick on it from here.
        if service == Service::Tower && intent == tower::Intent::RequestRunway {
            if let Some(ident) = self.desk.call.requested_ident.take() {
                if ctx.runway_refusal.is_none() && self.traffic_may_set_runway() {
                    tracing::info!(runway = %ident, "runway request granted");
                    self.runway_in_use = Some(ident);
                }
            }
        }
        // A crossing the tower cleared is noted on the strip: the flight's
        // order view reads it, and it stands until the flight is seen off
        // the strip it crossed.
        if service == Service::Tower && intent == tower::Intent::RequestCrossing {
            let cleared = ctx.on_ground && ctx.crossing.as_ref().is_some_and(|c| c.1.is_none());
            if let Some(end) = self.desk.call.crossing_ident.take().filter(|_| cleared) {
                let cell = &self.desk.strip.crossing;
                *cell.lock().unwrap_or_else(|e| e.into_inner()) = Some((end, false));
            }
        }
        // Publish the clearance picture for the monitor's compliance watch:
        // two-way contact has happened, and this is what the pilot is (and
        // isn't) cleared to do. Same reply-path-owns-its-static pattern as
        // HANDOFF and SQUAWK_WATCH.
        // ...unless a NEW flight has been set up and this station is the
        // old one, still answering until the main loop installs the new
        // (`retire_for_new_flight`): its worksheet is the ended flight's.
        // A traffic strip's is the traffic's, never gated by OUR new flight.
        if self.desk.strip.is_traffic() || !new_flight_pending() {
            publish_clearance(&self.desk.strip, ClearanceSnap {
                state: self.desk.twr.state,
                callsign: ctx.callsign.clone(),
                runway_ident: self.runway_in_use.clone().unwrap_or_default(),
                pending_readback: self.desk.twr.suggested_readback(&ctx.callsign),
                holds_runway: self.desk.twr.holds_runway(),
            });
        }
        // The landing sequence as the tower just said it: a downwind report
        // is told its number (and, behind a named arrival, whom it follows);
        // a flight no longer Inbound has no place in it.
        if service == Service::Tower {
            let mut seq = self.desk.strip.sequence.lock().unwrap_or_else(|e| e.into_inner());
            if intent == tower::Intent::ReportDownwind && reply.contains("number two") {
                let leader = if ctx.runway_occupied {
                    None
                } else {
                    ctx.final_traffic_id
                };
                *seq = Some((2, leader));
            } else if intent == tower::Intent::ReportDownwind && reply.contains("number one") {
                *seq = Some((1, None));
            } else if self.desk.twr.state != tower::State::Inbound {
                *seq = None;
            }
        }
        // A withheld landing clearance lives only while the flight is still
        // Inbound for it: cleared (a new final report), gone around, landed
        // or elsewhere retires it. A final report onto an occupied runway
        // ("continue approach", tower.rs) arms it, for the monitor's watch
        // to release once the strip clears (`landing_hold_tick`).
        if service == Service::Tower {
            if self.desk.twr.state != tower::State::Inbound || intent == tower::Intent::GoAround {
                *self.desk.strip.landing_hold.lock().unwrap_or_else(|e| e.into_inner()) = None;
            }
            if intent == tower::Intent::ReportFinal
                && self.desk.twr.state == tower::State::Inbound
                && !ctx.on_ground
                && ctx.runway_occupied
            {
                let ident = self.runway_in_use.clone().unwrap_or_default();
                sim::blackbox::record(
                    "atc",
                    serde_json::json!({"event": "landing_withheld", "runway": ident,
                        "why": "runway_occupied"}),
                );
                *self.desk.strip.landing_hold.lock().unwrap_or_else(|e| e.into_inner()) = Some(LandingHold {
                    callsign: ctx.callsign.clone(),
                    runway: ctx.runway.clone(),
                    runway_ident: ident,
                    line: self.desk.twr.landing_clearance_line(ctx),
                });
            }
        }
        // A fresh taxi clearance publishes its route for the off-route
        // compliance watch (and clears any stale one when route-less).
        // ...and so does a ready call answered "taxi to holding point ...
        // first": it is a taxi clearance with a route like any other (#178).
        let taxi_first =
            intent == tower::Intent::RequestTakeoff && was != tower::State::TaxiCleared;
        if service == Service::Tower
            && (matches!(intent, tower::Intent::RequestTaxi | tower::Intent::RequestRunway)
                || taxi_first)
            && self.desk.twr.state == tower::State::TaxiCleared
        {
            *self.desk.strip.taxi_route.lock().unwrap_or_else(|e| e.into_inner()) = self
                .desk.planned_route
                .as_ref()
                .filter(|r| r.path.len() >= 2)
                .map(|r| TaxiWatch {
                    path: r.path.clone(),
                    via: spoken_via(&r.via).join(", "),
                    seg: r.seg.clone(),
                    holding_point: r.holding_point.clone(),
                    lead_on: r.lead_on.clone(),
                });
            *self.desk.strip.taxi_to_stand.lock().unwrap_or_else(|e| e.into_inner()) = false;
            *self.desk.strip.self_taxi.lock().unwrap_or_else(|e| e.into_inner()) = None;
        }
        // The arrival taxi-in publishes its stand route the same way, so the
        // crew's auto_taxi drives it to parking. `was` is the pre-transition
        // state (a "request taxi" right after landing is an arrival too).
        if service == Service::Tower
            && (intent == tower::Intent::RequestTaxiIn
                || (intent == tower::Intent::RequestTaxi && was == tower::State::ClearedToLand))
        {
            *self.desk.strip.taxi_route.lock().unwrap_or_else(|e| e.into_inner()) = self
                .desk.planned_route
                .as_ref()
                .filter(|r| r.path.len() >= 2)
                .map(|r| TaxiWatch {
                    path: r.path.clone(),
                    via: spoken_via(&r.via).join(", "),
                    seg: r.seg.clone(),
                    holding_point: r.holding_point.clone(),
                    lead_on: r.lead_on.clone(),
                });
            *self.desk.strip.taxi_to_stand.lock().unwrap_or_else(|e| e.into_inner()) = true;
        }
        // A takeoff request answered with anything SHORT of the clearance —
        // hold position (traffic, wake, occupied strip), a conditional or
        // plain line-up, a cancellation — arms the release watch, so the
        // tower comes back unprompted the moment the runway is truly free.
        // A RUNWAY START is held from Parked: its first ready call puts it
        // LinedUp and holds it in one reply, and the watch never armed (LEMG
        // 13, sortie 1790623420-full_auto, 2026-09-28, issue #172: "hold
        // position, traffic on the runway" at +50 with nothing to release it).
        let runway_start_held =
            was == tower::State::Parked && self.desk.twr.state == tower::State::LinedUp;
        if service == Service::Tower
            && intent == tower::Intent::RequestTakeoff
            && (matches!(
                was,
                tower::State::TaxiCleared | tower::State::LinedUp | tower::State::TakeoffCleared
            ) || runway_start_held)
            && self.desk.twr.state != tower::State::TakeoffCleared
            && (ctx.final_traffic_nm.is_some()
                || ctx.runway_occupied
                || ctx.wake_secs_left.is_some())
        {
            let mut held = self.desk.strip.takeoff_hold.lock().unwrap_or_else(|e| e.into_inner());
            // Asking again does not restart the separation you have already
            // waited out. The wake gate is capped at one full interval AT THE
            // HOLD (`takeoff_hold_tick`), measured from `armed_at` — and
            // re-stamping it on every request handed that cap back: at a field
            // whose departure interval is shorter than the separation, a pilot
            // who asks a second time resets their own clock, hears "wake
            // turbulence, departing traffic ahead of you" again, and can never
            // take off. The clock belongs to the departure ahead of us in
            // sequence, not to our last transmission — so a hold already
            // standing on this runway keeps the moment it was armed.
            let armed_at = hold_armed_at(
                held.as_ref(),
                &self.runway_in_use.clone().unwrap_or_default(),
                self.reply_now(),
            );
            // So is the traffic it was held for: asking again names nobody new.
            let held_for = held
                .as_ref()
                .filter(|h| h.runway_ident == self.runway_in_use.clone().unwrap_or_default())
                .and_then(|h| h.held_for.clone());
            *held = Some(TakeoffHold {
                callsign: ctx.callsign.clone(),
                runway: ctx.runway.clone(),
                runway_ident: self.runway_in_use.clone().unwrap_or_default(),
                clear_since: None,
                armed_at,
                logged_at: self.reply_now(),
                // A hold issued FROM the runway is already sequenced: the
                // conditional/plain line-up put us there, so the watch owes a
                // release, not another line-up.
                lined_up: self.desk.twr.state == tower::State::LinedUp,
                behind: self.desk.twr.pending().is_some_and(|p| {
                    matches!(p.awaited, tower::Awaited::LineUp { behind: true, .. })
                }),
                held_for,
                cut_in_logged: Default::default(),
            });
            // Name the runway: at a parallel-runway field "held for traffic"
            // is only diagnosable if you can see WHICH strip the watch is
            // looking at (2026-07-27).
            tracing::info!(
                runway = %self.runway_in_use.clone().unwrap_or_default(),
                final_nm = ?ctx.final_traffic_nm,
                occupied = ctx.runway_occupied,
                wake_secs = ?ctx.wake_secs_left,
                "takeoff clearance withheld; release watch armed"
            );
        }
        // A departure that STOPPED on the runway spent everything its
        // clearance armed: the held release (EGKK 2026-09-27: armed while
        // the 737 rolled, never released, it replayed "traffic clear,
        // cleared for takeoff" 1.2 s into the resume), a release the main
        // loop has not synced yet, the conditional line-up's condition and
        // the after-departure handoff.
        if service == Service::Tower
            && intent == tower::Intent::RejectingTakeoff
            && ctx.on_ground
            && matches!(was, tower::State::TakeoffCleared | tower::State::LinedUp)
        {
            *self.desk.strip.pending_takeoff_clearance
                .lock()
                .unwrap_or_else(|e| e.into_inner()) = false;
            *self.desk.strip.pending_line_up.lock().unwrap_or_else(|e| e.into_inner()) = false;
            *self.desk.strip.handoff.lock().unwrap_or_else(|e| e.into_inner()) = None;
            tracing::info!(runway = ?self.runway_in_use, "takeoff rejected; clearance cancelled");
        }
        // A held release lives only while the tower still owes one: the
        // flight holding short or on the runway WITHOUT a clearance. Any
        // other state (cleared, parked, inbound) or a reject retires it, so
        // it can never replay a clearance nobody is waiting for.
        if service == Service::Tower
            && (intent == tower::Intent::RejectingTakeoff
                || !matches!(
                    self.desk.twr.state,
                    tower::State::TaxiCleared | tower::State::LinedUp
                ))
        {
            *self.desk.strip.takeoff_hold.lock().unwrap_or_else(|e| e.into_inner()) = None;
            *self.desk.strip.line_up_condition.lock().unwrap_or_else(|e| e.into_inner()) = None;
        }
        reply
    }

    /// Sync the tower FSM with the held takeoff clearance the MONITOR spoke
    /// once the final cleared (see [`takeoff_hold_tick`]): the state moves to
    /// TakeoffCleared and the departure handoff arms, exactly as if the
    /// clearance had been issued on the reply path.
    pub async fn apply_monitor_takeoff_clearance(&mut self, xp: SimView<'_>) {
        // A departure context (the monitor's release watch) — never an arrival.
        let (ctx, _wx) = self.build_ctx(xp, None, tower::Intent::RequestTakeoff).await;
        self.sync_takeoff_release(&ctx.runway, &ctx.callsign);
        self.arm_departure_handoff(&ctx);
    }

    /// The FSM half of [`Station::apply_monitor_takeoff_clearance`], on the
    /// desk on frequency: TakeoffCleared, its readback owed (`runway`
    /// spoken).
    pub(super) fn sync_takeoff_release(&mut self, runway: &str, callsign: &str) {
        self.desk.twr.state = tower::State::TakeoffCleared;
        // The released clearance owes its readback exactly like one issued
        // on the reply path.
        self.desk.twr.arm_takeoff_readback(
            runway.to_string(),
            format!("runway {runway}, cleared for takeoff."),
        );
        if let Some(c) = self.desk.strip.clearance.lock().unwrap_or_else(|e| e.into_inner()).as_mut() {
            c.pending_readback = self.desk.twr.suggested_readback(callsign);
        }
    }

    /// Sync the tower FSM with the line-up-and-wait the MONITOR spoke when it
    /// sequenced a held departure onto the runway (see [`takeoff_hold_tick`]):
    /// the state moves to LinedUp and the line-up readback is owed, exactly as
    /// if the instruction had been issued on the reply path. The hold stays
    /// armed — the release is still to come.
    pub async fn apply_monitor_line_up(&mut self, xp: SimView<'_>) {
        let (ctx, _wx) = self.build_ctx(xp, None, tower::Intent::RequestTakeoff).await;
        self.sync_line_up(&ctx.runway, &ctx.callsign);
    }

    /// The FSM half of [`Station::apply_monitor_line_up`], on the desk on
    /// frequency: LinedUp, the line-up readback owed (`runway` spoken).
    pub(super) fn sync_line_up(&mut self, runway: &str, callsign: &str) {
        self.desk.twr.state = tower::State::LinedUp;
        self.desk.twr.arm_line_up_readback(
            runway.to_string(),
            format!("line up and wait runway {runway}."),
        );
        if let Some(c) = self.desk.strip.clearance.lock().unwrap_or_else(|e| e.into_inner()).as_mut() {
            c.pending_readback = self.desk.twr.suggested_readback(callsign);
        }
    }

    /// Sync the tower FSM with the go-around order the MONITOR spoke (see
    /// [`go_around_tick`]): the landing clearance is void and the flight is
    /// back in the pattern, exactly as if the pilot had called it. The
    /// coach's next line becomes the acknowledgment the order deserves,
    /// never the readback of the clearance that just died.
    /// Sync the tower FSM with a withheld landing clearance the MONITOR
    /// released (see [`landing_hold_tick`]): ClearedToLand with its readback
    /// owed, exactly as if it had been the reply to the final report. Run
    /// before the station answers anything, so the crew's readback of the
    /// released clearance meets a tower that knows it gave it.
    pub(super) fn sync_landing_release(&mut self) {
        let released = self.desk.strip.pending_landing_release
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .take();
        if let Some((runway, line)) = released {
            self.desk.twr.release_landing(runway, line);
        }
    }

    pub fn apply_monitor_go_around(&mut self) {
        self.desk.twr.apply_go_around();
        // The re-sequence the order announced ("expect runway one two"):
        // the runway in use moves with it, and the published clearance
        // names it, so the crew's approach follows once its missed approach
        // is flown (`autoland::clearance_switch`). Issue #162.
        let reseq = self.desk.strip.pending_resequence
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .take();
        if let Some((from, to)) = &reseq {
            tracing::info!(%from, %to, "arrival re-sequenced after a go-around");
            if self.traffic_may_set_runway() {
                self.runway_in_use = Some(to.clone());
            }
            self.desk.call.stated_runway = None;
        }
        self.desk.resequenced = reseq.clone();
        if let Some(c) = self.desk.strip.clearance.lock().unwrap_or_else(|e| e.into_inner()).as_mut() {
            c.state = tower::State::Inbound;
            c.pending_readback = Some(format!("{GO_AROUND_READBACK}{}.", c.callsign));
            if let Some((_, to)) = &reseq {
                c.runway_ident = to.clone();
            }
        }
    }

    fn respond_inner(
        &mut self,
        transcript: &str,
        service: Service,
        intent: tower::Intent,
        ctx: &tower::Ctx,
        wx: Wx,
    ) -> String {
        // "Cancel IFR" is the IFR book's, whichever position hears it.
        if intent == tower::Intent::CancelIfr {
            return self.cancel_ifr(ctx);
        }
        // A SID amendment is the IFR book's too, and the tower's to give.
        if intent == tower::Intent::RequestSid && service == Service::Tower {
            return self.amend_sid(transcript, ctx);
        }
        match service {
            Service::Tower => {
                let was = self.desk.twr.state;
                let going_around = intent == tower::Intent::GoAround;
                let reply = self.desk.twr.handle(intent.clone(), ctx);
                // The crew's "going around" after a re-sequenced order is
                // answered with the runway it is re-sequenced to, once.
                let reply = match (going_around, self.desk.resequenced.take()) {
                    (true, Some((from, to))) => tower::go_around_resequenced_line(
                        &ctx.callsign,
                        &atis::spoken_runway(&from),
                        &atis::spoken_runway(&to),
                        ctx.dialect,
                        ctx.uk,
                        ctx.missed_approach.as_ref(),
                    ),
                    _ => reply,
                };
                // A final report naming the wrong runway earns its correction
                // in the same breath as the clearance.
                let reply = tower::Tower::correct_final_runway(transcript, intent, ctx, reply);
                // A fresh takeoff clearance arms the after-departure handoff:
                // once airborne and climbing, the tower sends the flight to
                // departure (or approach) — the frequency chain, not a tower
                // that owns you forever.
                if self.desk.twr.state == tower::State::TakeoffCleared
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
                format!(
                    "{}, wind {}{}.",
                    ctx.callsign,
                    ctx.wind,
                    tower::qnh_clause(ctx)
                )
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
                    self.desk.twr.expect_touch_and_go(touch_and_go);
                    format!(
                        "{}, contact {} Approach on {} for vectors.",
                        ctx.callsign, ctx.airport, freq
                    )
                }
                None => {
                    self.desk.twr.expect_touch_and_go(touch_and_go);
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
                self.desk.twr.expect_touch_and_go(false);
                let vectors = self.arm_vectors(ctx, wx);
                format!("Roger mayday. {vectors}")
            }
            // Runway-shaped business belongs to the tower.
            tower::Intent::ReportFinal
            | tower::Intent::ReportDownwind
            | tower::Intent::ReportBase
            | tower::Intent::ReportCrosswind
            | tower::Intent::GoAround
            | tower::Intent::RejectingTakeoff
            | tower::Intent::RequestTakeoff
            | tower::Intent::RequestTaxi
            | tower::Intent::RequestTaxiIn
            | tower::Intent::RunwayVacated
            | tower::Intent::RequestRunway
            | tower::Intent::RequestBacktrack
            | tower::Intent::RequestCrossing
            | tower::Intent::CancelIfr
            | tower::Intent::RequestSid
            | tower::Intent::RequestStartup
            | tower::Intent::RequestShutdown
            | tower::Intent::RequestIfrClearance { .. }
            | tower::Intent::RequestTransit
            | tower::Intent::RequestClearance => format!(
                "{}, contact {} Tower on {}.",
                ctx.callsign,
                ctx.airport,
                atis::spoken_mhz_in(self.tower_khz, self.dialect)
            ),
            tower::Intent::Greeting => {
                format!(
                    "{}, {} Departure, pass your message.",
                    ctx.callsign, ctx.airport
                )
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

    /// "Cancel IFR" (issue #96): the flight's IFR standing ends. The
    /// clearance (owed, accepted or published) and the flight's IFR code
    /// are retired, and with them the runway frozen for the SID, so the next
    /// region handoff is a VFR one (the service terminated, seven thousand)
    /// and a later request is a fresh clearance. A flight with nothing to
    /// cancel is told so rather than acknowledged.
    fn cancel_ifr(&mut self, ctx: &tower::Ctx) -> String {
        let owed = self.desk.twr.cancel_ifr();
        let published = self.desk.strip.ifr_clearance
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .take()
            .is_some();
        let flight = self.desk.strip.ifr_flight
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .take()
            .is_some();
        if !(owed || published || flight) {
            tracing::info!("cancel IFR with no IFR flight");
            return format!("{}, roger, you are not on an IFR clearance.", ctx.callsign);
        }
        self.desk.ifr_runway = None;
        self.desk.ifr_bearings = None;
        self.desk.ifr_sid_basis = None;
        tracing::info!(owed, published, flight, "IFR cancelled");
        match self.dialect {
            Dialect::Faa => format!("{}, IFR cancellation received.", ctx.callsign),
            Dialect::Icao => format!("{}, IFR flight cancelled.", ctx.callsign),
        }
    }

    /// "Request the Lam five Papa departure" (issue #152): an IFR flight
    /// asking to fly another SID off the runway it was cleared on. Granted
    /// when the words name a SID this field publishes for that runway: the
    /// held clearance is amended (the crew sees it in
    /// `party_line::assigned_clearance`) and blackboxed `sid_amended`;
    /// otherwise the cleared departure stands, said, and blackboxed
    /// `sid_request_refused`. A flight with no IFR clearance is told to ask
    /// for one.
    fn amend_sid(&mut self, transcript: &str, ctx: &tower::Ctx) -> String {
        let cs = &ctx.callsign;
        let held = self.desk.strip.ifr_clearance.lock().unwrap_or_else(|e| e.into_inner()).clone();
        let Some(mut c) = held else {
            return format!("{cs}, request your IFR clearance first.");
        };
        let refused = |why: &str, asked: Option<&str>| {
            sim::blackbox::record(
                "atc",
                serde_json::json!({"event": "sid_request_refused", "asked": asked,
                    "cleared": c.sid, "runway": c.runway, "why": why}),
            );
        };
        let Some(words) = tower::requested_departure(transcript) else {
            refused("unheard", None);
            return format!("{cs}, say again the departure.");
        };
        let idents: Vec<String> = self
            .procedures
            .as_ref()
            .map(|(dir, apt)| nav::sid_transitions(dir, apt, &c.runway))
            .unwrap_or_default()
            .into_iter()
            .map(|(i, _)| i)
            .collect();
        let rwy = atis::spoken_runway(&c.runway);
        let stands = match &c.sid {
            Some(s) => format!("The {} departure stands.", tower::spoken_sid(s)),
            None => "Fly runway heading.".to_string(),
        };
        let Some(sid) = tower::match_sid(&words, &idents).map(str::to_string) else {
            refused("not_this_runway", Some(&words));
            return format!("{cs}, negative, no such departure from runway {rwy}. {stands}");
        };
        if c.sid.as_deref().is_some_and(|s| s.eq_ignore_ascii_case(&sid)) {
            return format!("{cs}, affirm, the {} departure.", tower::spoken_sid(&sid));
        }
        tracing::info!(from = ?c.sid, to = %sid, runway = %c.runway, "IFR clearance: SID amended");
        sim::blackbox::record(
            "atc",
            serde_json::json!({"event": "sid_amended", "side": "tower", "callsign": cs,
                "from": c.sid, "to": sid, "runway": c.runway}),
        );
        c.sid = Some(sid.clone());
        *self.desk.strip.ifr_clearance.lock().unwrap_or_else(|e| e.into_inner()) = Some(c);
        format!("{cs}, {} departure approved, runway {rwy}.", tower::spoken_sid(&sid))
    }

    /// The IFR clearance for `dest` (as heard), made up from what this
    /// station knows: the destination from the index, the runway in use,
    /// its SID from the CIFP (none = runway heading, never an invented
    /// name), the initial altitude from the field elevation, and the code:
    /// the discrete one at a radar field (the same one the zone clearance
    /// would assign), the code already held otherwise. A reissued clearance
    /// supersedes the published one until it is read back. `None` = the
    /// destination is unknown to the index.
    /// The cruise level this flight FILED, feet: the accepted IFR
    /// clearance's destination and this field's filed route to it. `None`
    /// without a clearance, a filed route or a filed level.
    pub(super) fn filed_cruise_ft(&self) -> Option<i64> {
        let dest = self.desk.strip.ifr_clearance
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()?
            .dest
            .clone();
        let (_, apt) = self.procedures.as_ref()?;
        crate::party_line::filed_route_for(apt, &dest)?.cruise_ft
    }

    fn ifr_offer(&mut self, dest: &str, ctx: &tower::Ctx) -> Option<tower::IfrOffer> {
        let (icao, name, dest_pos) = resolve_destination(dest)?;
        let runway = self.runway_in_use.clone().unwrap_or_default();
        self.desk.ifr_runway = Some(runway.clone()).filter(|r| !r.is_empty());
        // The flight's FILED route, when the crew has one to this
        // destination from this field: the SID is the one that serves the
        // runway and joins it, by the crew's own rule
        // (`nav::sid_for_filed_route`), so the crew can fly what it is
        // cleared. None joins: runway heading, never a SID that leads
        // elsewhere (EGKK 08R, 2026-09-28: KENE3P toward Birmingham's
        // bearing for a route filed via LAM; the crew flew LAM5P).
        let filed = self.procedures.as_ref().filter(|_| !runway.is_empty()).and_then(|(dir, apt)| {
            let route = crate::party_line::filed_route_for(apt, &icao)?;
            Some((
                nav::sid_for_filed_route(dir, apt, &runway, &route.fixes, route.sid.as_deref()),
                !nav::sid_transitions(dir, apt, &runway).is_empty(),
            ))
        });
        let (sid, basis) = match filed {
            Some((Some(choice), _)) => {
                self.desk.ifr_bearings = None;
                (Some(choice.ident), SidBasis::Route { join: choice.join })
            }
            // The runway has SIDs and none meets the route.
            Some((None, true)) => {
                self.desk.ifr_bearings = None;
                (None, SidBasis::NoneJoins)
            }
            _ => {
                // The SID that leaves toward the destination; with no
                // positioned exits (still loading, no nav data) the first in
                // the file.
                let toward = match (self.apt.position(), runway.is_empty()) {
                    (Some(field), false) => self
                        .sid_exits
                        .lock()
                        .unwrap_or_else(|e| e.into_inner())
                        .as_deref()
                        .and_then(|x| nav::sid_toward(x, &runway, field, dest_pos)),
                    _ => None,
                };
                self.desk.ifr_bearings = toward.as_ref().map(|t| (t.1, t.2));
                match toward {
                    Some((ident, _, _)) => (Some(ident), SidBasis::Bearing),
                    None => (
                        self.procedures
                            .as_ref()
                            .filter(|_| !runway.is_empty())
                            .and_then(|(dir, apt)| nav::sid_for_runway(dir, apt, &runway)),
                        SidBasis::FirstInFile,
                    ),
                }
            }
        };
        self.desk.ifr_sid_basis = Some(basis);
        let radar = self.approach_khz.is_some() || self.departure_khz.is_some();
        let squawk = if radar {
            let scope = if self.departure_khz.is_some() {
                StationSpeaker::Departure
            } else {
                StationSpeaker::Approach
            };
            self.assign_squawk(&ctx.callsign, scope);
            self.desk.assigned_squawk.unwrap_or(7000)
        } else {
            self.desk.assigned_squawk.unwrap_or(7000)
        };
        *self.desk.strip.ifr_clearance.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *self.desk.strip.ifr_flight.lock().unwrap_or_else(|e| e.into_inner()) = None;
        Some(tower::IfrOffer {
            clearance: tower::IfrClearance {
                dest: icao,
                sid,
                runway,
                initial_alt_ft: tower::ifr_initial_altitude_ft(self.apt.elev_ft),
                squawk,
            },
            dest_spoken: spoken_airport(&name).to_string(),
            faa: self.dialect == Dialect::Faa,
            departure_freq: self.departure_khz.or(self.approach_khz).map(|k| atis::spoken_mhz_in(k, self.dialect)),
        })
    }

    /// A tower reply that gave the pressure: noted as given to this flight
    /// (said once per contact, [`StripDesk::qnh_given`]), and, the first
    /// time, the current ATIS named before it when the pilot has not
    /// reported it: "information Bravo is current, QNH one zero one seven"
    /// (CAP 413 / Doc 4444: the controller confirms the current information
    /// to a pilot who did not report receipt). A reply without the pressure
    /// passes through untouched.
    fn note_pressure_given(&mut self, ctx: &tower::Ctx, reply: String) -> String {
        let clause = format!(", {}", ctx.qnh);
        if ctx.qnh.is_empty() || !reply.contains(&clause) {
            return reply;
        }
        self.desk.qnh_given = Some(ctx.qnh.clone());
        let current = atis_current_letter();
        let Some(letter) = current.filter(|_| self.desk.atis_told.is_none() && self.desk.pilot_info != current)
        else {
            return reply;
        };
        self.desk.atis_told = Some(letter);
        let info = match self.dialect {
            Dialect::Icao => format!(", information {} is current", atis::nato(letter)),
            Dialect::Faa => format!(", ATIS information {} is current", atis::nato(letter)),
        };
        reply.replacen(&clause, &format!("{info}{clause}"), 1)
    }

    /// The missed approach this flight's go-around flies, when it is an
    /// instrument arrival ([`instrument_arrival`]); `None` for circuit
    /// traffic, which climbs to the circuit and reports downwind.
    pub(super) fn missed_approach_for(&self, callsign: &str) -> Option<tower::MissedApproach> {
        if self.desk.circuits || !instrument_arrival(&self.desk.strip, callsign) {
            return None;
        }
        let contact = match (self.approach_khz, self.departure_khz) {
            (Some(k), _) => Some(("Approach", k)),
            (None, Some(k)) => Some(("Departure", k)),
            (None, None) => None,
        }
        .map(|(unit, k)| {
            (
                format!("{} {unit}", spoken_airport(&self.apt.name)),
                atis::spoken_mhz_in(k, self.dialect),
            )
        });
        Some(missed_approach_at(
            self.runway_in_use.as_deref().unwrap_or_default(),
            self.apt.elev_ft,
            contact,
        ))
    }

    /// The field speaks CAP 413's own words ([`speaks_cap413`]), by the ICAO
    /// it was loaded or given procedures by.
    pub(super) fn speaks_cap413(&self) -> bool {
        self.cap413
    }

    /// The circuit clearance's facts (#182): the height, the dialect, the
    /// runway's published side (apt.dat `1101`, found by its spoken
    /// designator) and whether the field's data allows circuits (`1302
    /// allows_circuits`; LEMG says 0, blackbox `atc` `circuits_not_allowed`).
    pub(super) fn circuit_offer(&self, spoken_runway: &str) -> tower::CircuitOffer {
        let left = self
            .apt
            .runways
            .iter()
            .flat_map(|r| [&r.ident_a, &r.ident_b])
            .find(|id| atis::spoken_runway(id) == spoken_runway)
            .and_then(|id| self.apt.pattern_left(id));
        let not_available = self.apt.allows_circuits == Some(false) && !self.desk.circuits_declined;
        if not_available {
            sim::blackbox::record(
                "atc",
                serde_json::json!({"event": "circuits_not_allowed", "field": self.apt.name,
                                   "runway": spoken_runway}),
            );
        }
        tower::CircuitOffer {
            height_ft: tower::circuit_height_ft(self.apt.elev_ft),
            faa: self.dialect == Dialect::Faa,
            left,
            not_available,
        }
    }

    /// Issue this flight its discrete code (idempotent — the same flight
    /// keeps the same code) and arm the radar watch. Returns the spoken code
    /// for the reply. The watch side effect mirrors [`Station::arm_departure_handoff`]'s
    /// `OWN_STRIP.handoff` write: reply paths own their monitor-tick state.
    pub(super) fn assign_squawk(&mut self, callsign: &str, speaker: StationSpeaker) -> String {
        let code = *self
            .desk.assigned_squawk
            .get_or_insert_with(|| discrete_squawk(callsign));
        // Publish it for the crew (the transponder gets dialled to this).
        *self.desk.strip.assigned_squawk.lock().unwrap_or_else(|e| e.into_inner()) = Some(code);
        // Circuit traffic gets no radar service: the code, but no watch
        // (no "radar contact" from a tower working the pattern by eye).
        if self.desk.circuits {
            return atis::spoken_digits(&code.to_string());
        }
        let mut w = self.desk.strip.squawk_watch.lock().unwrap_or_else(|e| e.into_inner());
        // Same code from a new controller: the watch keeps its history (no
        // second grace period, no second "radar contact") but the VOICE is
        // whoever issued it last, the one we are actually talking to.
        if let Some(w) = w.as_mut().filter(|w| w.code == code) {
            w.speaker = speaker;
        }
        if w.as_ref().map(|w| w.code) != Some(code) {
            *w = Some(SquawkWatch {
                code,
                callsign: callsign.to_string(),
                speaker,
                observed: false,
                check_in_owed: false,
                first_seen: None,
                last_nudge: None,
                nudges: 0,
            });
        }
        atis::spoken_digits(&code.to_string())
    }

    /// What a departure request says about where the flight is going: the
    /// pattern ("request taxi for circuits") or away. A taxi, zone or IFR
    /// request states it afresh (said or not); a "ready for departure" can
    /// only add it, since pilots do not repeat "for circuits" on every call.
    /// Declaring circuits retires any radar watch and any armed handoff: the
    /// tower keeps pattern traffic. LEMG 31 (sortie 1790584715, 2026-09-28):
    /// a zone clearance from the mission before left a Departure watch
    /// armed, and the tower's voice said "radar contact" to the circuit
    /// 0.4 s after rotation.
    fn note_departure_intent(&mut self, intent: &tower::Intent, transcript: &str) {
        let said = tower::wants_circuits(transcript);
        let now = match intent {
            tower::Intent::RequestTaxi | tower::Intent::RequestClearance => said,
            tower::Intent::RequestIfrClearance { .. } => false,
            tower::Intent::RequestTakeoff => self.desk.circuits || said,
            _ => return,
        };
        if now == self.desk.circuits {
            return;
        }
        self.desk.circuits = now;
        self.desk
            .strip
            .circuits
            .store(now, std::sync::atomic::Ordering::SeqCst);
        sim::blackbox::record(
            "atc",
            serde_json::json!({"event": "circuit_traffic", "circuits": now,
                "intent": format!("{intent:?}")}),
        );
        tracing::info!(circuits = now, "departure intent noted");
        if now {
            *self.desk.strip.squawk_watch.lock().unwrap_or_else(|e| e.into_inner()) = None;
            *self.desk.strip.squawk_nudge.lock().unwrap_or_else(|e| e.into_inner()) = None;
            *self.desk.strip.handoff.lock().unwrap_or_else(|e| e.into_inner()) = None;
            *self.desk.strip.departure_handoff.lock().unwrap_or_else(|e| e.into_inner()) = None;
        }
    }

    /// Arm the after-takeoff handoff. Spoken by [`handoff_tick`] once the
    /// flight is airborne and climbing through ~700 ft above the field —
    /// a tower doesn't say "contact departure" to an aircraft still rolling.
    pub(super) fn arm_departure_handoff(&mut self, ctx: &tower::Ctx) {
        // Circuit traffic is the tower's own: the pattern comes back to this
        // runway, and "contact Departure" on the crosswind leg sends it to a
        // controller who does not work it (LEMG 31, 2026-09-28).
        if self.desk.circuits {
            sim::blackbox::record(
                "atc",
                serde_json::json!({"event": "departure_handoff_withheld", "why": "circuits"}),
            );
            tracing::info!("circuit traffic: the tower keeps it, no departure handoff");
            return;
        }
        // Prefer departure; fall back to approach; a tower-only field keeps
        // its traffic and no handoff arms.
        let (name, khz) = match (self.departure_khz, self.approach_khz) {
            (Some(k), _) => ("Departure", k),
            (None, Some(k)) => ("Approach", k),
            (None, None) => return,
        };
        self.desk.dep_checked_in = false;
        // "Radar contact" is now the check-in's answer on the new frequency.
        if let Some(w) = self.desk.strip.squawk_watch
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_mut()
        {
            w.check_in_owed = true;
        }
        self.desk.handed_to = Some(if name == "Departure" {
            Service::Departure
        } else {
            Service::Approach
        });
        *self.desk.strip.handoff.lock().unwrap_or_else(|e| e.into_inner()) = Some(Handoff {
            line: format!(
                "{}, contact {} {} on {}.",
                ctx.callsign,
                ctx.airport,
                name,
                atis::spoken_mhz_in(khz, self.dialect)
            ),
            min_alt_ft: self.apt.elev_ft + 700.0,
            to: format!("{} {}", ctx.airport, name),
            khz,
        });
        tracing::info!(to = name, "departure handoff armed");
    }

    /// The departure has checked in with the position it was handed to: the
    /// tower's book on it is closed. The takeoff clearance is spent (the
    /// flight is airborne and someone else's), so the tower state goes back
    /// to Parked, the state every arrival starts from, and the published
    /// picture says so: nothing held, nothing owed.
    pub(super) fn transfer_departure(&mut self) {
        if self.desk.twr.state == tower::State::TakeoffCleared {
            self.desk.twr.state = tower::State::Parked;
        }
        self.desk.twr.clear_clearance_readback();
        if let Some(c) = self.desk.strip.clearance.lock().unwrap_or_else(|e| e.into_inner()).as_mut() {
            c.state = self.desk.twr.state;
            c.holds_runway = self.desk.twr.holds_runway();
            c.pending_readback = None;
        }
    }

    /// The approach controller: vectors to final, or a redirect to the tower
    /// for anything that belongs there.
    pub(super) fn respond_approach(&mut self, intent: tower::Intent, ctx: &tower::Ctx, wx: Wx) -> String {
        match intent {
            tower::Intent::RadioCheck => {
                format!(
                    "{}, {} Approach, reading you five.",
                    ctx.callsign, ctx.airport
                )
            }
            tower::Intent::WindCheck => {
                format!(
                    "{}, wind {}{}.",
                    ctx.callsign,
                    ctx.wind,
                    tower::qnh_clause(ctx)
                )
            }
            tower::Intent::SquawkCheck => {
                format!("{}, squawk seven zero zero zero.", ctx.callsign)
            }
            tower::Intent::RequestVectors => self.arm_vectors(ctx, wx),
            tower::Intent::RequestLanding { touch_and_go } => {
                // The option request rides through the vectors: the tower's
                // eventual clearance must say "touch and go", not "land".
                self.desk.twr.expect_touch_and_go(touch_and_go);
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
                self.desk.twr.expect_touch_and_go(false);
                let vectors = self.arm_vectors(ctx, wx);
                format!("Roger mayday. {vectors}")
            }
            // "Established ILS two seven right" (parses as the final report)
            // while being vectored: approach's job is done — acknowledge,
            // hand the flight to the tower, drop the plan so the tick stops
            // steering a pilot who's already on the localizer.
            tower::Intent::ReportFinal
                if self.desk.strip.vectors.lock().unwrap_or_else(|e| e.into_inner()).is_some() =>
            {
                *self.desk.strip.vectors.lock().unwrap_or_else(|e| e.into_inner()) = None;
                format!(
                    "{}, roger. Contact {} Tower on {}.",
                    ctx.callsign,
                    ctx.airport,
                    atis::spoken_mhz_in(self.tower_khz, self.dialect)
                )
            }
            // Final reports and everything runway-shaped belong to the tower.
            tower::Intent::ReportFinal
            | tower::Intent::ReportDownwind
            | tower::Intent::ReportBase
            | tower::Intent::ReportCrosswind
            | tower::Intent::GoAround
            | tower::Intent::RejectingTakeoff
            | tower::Intent::RequestTakeoff
            | tower::Intent::RequestTaxi
            | tower::Intent::RequestTaxiIn
            | tower::Intent::RunwayVacated
            | tower::Intent::RequestRunway
            | tower::Intent::RequestBacktrack
            | tower::Intent::RequestCrossing
            | tower::Intent::CancelIfr
            | tower::Intent::RequestSid
            | tower::Intent::RequestStartup
            | tower::Intent::RequestShutdown
            | tower::Intent::RequestIfrClearance { .. }
            | tower::Intent::RequestTransit
            | tower::Intent::RequestClearance => format!(
                "{}, contact {} Tower on {}.",
                ctx.callsign,
                ctx.airport,
                atis::spoken_mhz_in(self.tower_khz, self.dialect)
            ),
            tower::Intent::Greeting => {
                format!(
                    "{}, {} Approach, pass your message.",
                    ctx.callsign, ctx.airport
                )
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
            Some(rec) => ils_join_nm(target_alt_ft - self.apt.elev_ft, rec.gs_deg.unwrap_or(3.0)),
            None => JOIN_FINAL_NM,
        };
        let runway_spoken = atis::spoken_runway(&ident);
        *self.desk.strip.vectors.lock().unwrap_or_else(|e| e.into_inner()) = Some(VectorPlan {
            callsign: ctx.callsign.clone(),
            thr_lat,
            thr_lon,
            course_deg,
            runway_spoken: runway_spoken.clone(),
            tower_spoken: Some(atis::spoken_mhz_in(self.tower_khz, self.dialect)),
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
        // At or above the transition level the descent is to a flight level
        // and carries no QNH (the pressure goes with the first altitude).
        let level = atis::is_flight_level(target_alt_ft, ctx.transition_level_ft);
        let (icao_alt, faa_alt, qnh) = if level {
            let fl = atis::spoken_flight_level(target_alt_ft);
            (fl.clone(), fl, String::new())
        } else {
            (
                format!("to altitude {} feet", spoken_alt(target_alt_ft)),
                spoken_alt(target_alt_ft),
                tower::qnh_clause(ctx),
            )
        };
        let plan_phrase = match (ils, self.dialect) {
            // CAP 413 / ICAO: "vectoring ILS approach", altitudes are "to
            // altitude ... feet" with the QNH attached.
            (true, Dialect::Icao) => format!(
                "Vectoring ILS approach runway {}. Descend {icao_alt}{qnh}.",
                runway_spoken,
            ),
            // FAA: "expect vectors ILS runway ... approach", "descend and
            // maintain", and the pressure clause speaks altimeter in inches
            // (ctx.qnh carries the dialect-correct phrase).
            (true, Dialect::Faa) => format!(
                "Expect vectors ILS runway {} approach. Descend and maintain {faa_alt}{qnh}.",
                runway_spoken,
            ),
            (false, _) => format!(
                "Expect vectors, visual approach runway {}. Descend and maintain {faa_alt}{qnh}.",
                runway_spoken,
            ),
        };
        format!(
            "{}, {} Approach, squawk {}. {}",
            ctx.callsign, ctx.airport, squawk, plan_phrase
        )
    }
}

/// Is the flight on this strip an INSTRUMENT arrival, whose go-around is
/// the missed approach: an IFR flight, or an airline (its callsign is an
/// airline's telephony, [`crate::callsign::is_airline_telephony`]) not
/// working the circuit. A 737 off the ILS was told "climb to circuit
/// altitude, report downwind" (the realism review, 2026-10-09).
pub(super) fn instrument_arrival(strip: &FlightStrip, callsign: &str) -> bool {
    if strip.circuits.load(Ordering::SeqCst) {
        return false;
    }
    strip
        .ifr_flight
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .is_some()
        || crate::callsign::is_airline_telephony(callsign)
}

/// The missed approach off `runway_ident`: published where the field's nav
/// data has an ILS to it ([`ILS_FACTS`]; still loading reads as none, and
/// the no-procedure climb is said instead), else a climb to
/// [`tower::missed_approach_alt_ft`] on runway heading.
pub(super) fn missed_approach_at(
    runway_ident: &str,
    field_elev_ft: f64,
    contact: Option<(String, String)>,
) -> tower::MissedApproach {
    let published = !runway_ident.is_empty()
        && ILS_FACTS
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .is_some_and(|f| f.ils.iter().any(|i| i.runway == runway_ident));
    tower::MissedApproach {
        published,
        alt_ft: tower::missed_approach_alt_ft(field_elev_ft),
        contact,
    }
}
