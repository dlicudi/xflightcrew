//! The runway in use: stated runways, a caller on the strip, the pick.

use super::*;

impl Station {
    /// Remember the runway a call names for itself, for the pick this call is
    /// about to make (see `pick_runway_with`): an arrival's, and a departure's
    /// asked for AT THE TAXI REQUEST ("request taxi, runway two six"), where a
    /// pilot with a planned departure asks for it. The ready-for-departure
    /// call is told its runway. LOWI (2026-09-24): calm, the pick could have
    /// been 08, and the planned MOGT3H exists only from 26. The wind still
    /// wins when it really favours the other end.
    pub(super) fn note_stated_runway(&mut self, transcript: &str, intent: tower::Intent) {
        self.desk.call.stated_runway = match intent {
            tower::Intent::ReportFinal
            | tower::Intent::ReportDownwind
            | tower::Intent::ReportBase
            | tower::Intent::RequestTaxi
            | tower::Intent::RequestLanding { .. }
            // A departure's clearance request may name its runway, like the
            // taxi request; a runway start names the strip it is lined up on
            // ("request clearance, runway one three", `note_caller_on_strip`).
            | tower::Intent::RequestClearance => tower::stated_runway(transcript),
            _ => None,
        };
        self.desk.call.stated_on_strip = false;
        self.desk.call.runway_request = match intent {
            tower::Intent::RequestRunway => tower::requested_runway(transcript),
            _ => None,
        };
        self.desk.call.requested_ident = None;
    }

    /// The caller of the stated runway stands ON that strip, on the ground:
    /// a runway start (the sim put it there, lined up). Its runway stands
    /// against the calm-wind arrival stream as long as the wind does not
    /// argue (`pick_runway_with`). Pure but for the flag.
    pub(super) fn note_caller_on_strip(&mut self, on_ground: bool, lat: f64, lon: f64) {
        self.desk.call.stated_on_strip = on_ground
            && self.stated_ident().is_some_and(|ident| {
                strip(&self.apt.runways, &ident).is_some_and(|r| on_runway(r, lat, lon).is_some())
            });
    }

    /// A RUNWAY CHANGE IS SAID. The runway in use may turn after a flight's
    /// clearance named another (the calm-wind stream, `pick_runway_with`);
    /// the next ground reply that sends it to the new runway opens "runway
    /// change, runway one three in use". LEMG (sortie 1790631369-full_auto,
    /// 2026-09-28, #178): a runway start on 30 was cleared for the circuit
    /// on 30, a lander on 12 turned the field round to 13, and "ready for
    /// departure" earned "taxi to holding point Hotel November one runway
    /// one three first" with nothing to say the runway had moved. Airborne
    /// forgets it: an arrival's runway is its own business.
    pub(super) fn state_runway_change(
        &mut self,
        intent: &tower::Intent,
        ctx: &tower::Ctx,
        reply: String,
    ) -> String {
        if !ctx.on_ground {
            self.desk.told_runway = None;
            self.desk.start_strip = None;
            return reply;
        }
        if reply.is_empty() {
            return reply;
        }
        let now = self.runway_in_use.clone();
        match intent {
            tower::Intent::RequestClearance
            | tower::Intent::RequestIfrClearance { .. }
            | tower::Intent::RequestRunway => {
                self.desk.told_runway = now;
                reply
            }
            tower::Intent::RequestTakeoff
            | tower::Intent::RequestTaxi
            | tower::Intent::RequestBacktrack => {
                let from = std::mem::replace(&mut self.desk.told_runway, now.clone());
                match (from, now) {
                    (Some(from), Some(to)) if from != to => {
                        sim::blackbox::record(
                            "atc",
                            serde_json::json!({"event": "runway_change_stated",
                                "from": from, "to": to}),
                        );
                        tower::runway_change_line(&ctx.callsign, &ctx.runway, &reply)
                    }
                    _ => reply,
                }
            }
            _ => reply,
        }
    }

    /// The strip end the held stated runway names at this field.
    fn stated_ident(&self) -> Option<String> {
        let stated = self.desk.call.stated_runway.as_deref()?;
        self.apt
            .runways
            .iter()
            .flat_map(|r| [r.ident_a.clone(), r.ident_b.clone()])
            .find(|id| tower::runway_matches(id, stated))
    }

    /// A flight the tower is sequencing owns the runway in use (see
    /// `pick_runway_with`).
    fn runway_committed(&self) -> bool {
        matches!(
            self.desk.twr.state,
            tower::State::TaxiCleared
                | tower::State::LinedUp
                | tower::State::TakeoffCleared
                | tower::State::Inbound
        ) || (self.desk.twr.state == tower::State::ClearedToLand && !self.desk.twr.runway_vacated())
            || (self.desk.ifr_runway.is_some() && self.desk.ifr_runway == self.runway_in_use)
    }

    /// Answer a "request runway ..." call's context: the runway it named,
    /// resolved against this field (`resolve_runway_request`). Granted, the
    /// ctx names THAT runway and, on the ground, the taxi route is re-planned
    /// to it from `pos` (as `RequestTaxi` plans one to the runway in use);
    /// refused, the ctx carries the reason. Pure but for the route planner.
    pub(super) fn apply_runway_request(&mut self, ctx: &mut tower::Ctx, pos: Option<(f64, f64)>) {
        match resolve_runway_request(&self.apt.runways, self.desk.call.runway_request.as_deref()) {
            Err(refusal) => {
                self.desk.call.requested_ident = None;
                ctx.runway_refusal = Some(refusal);
            }
            Ok(ident) => {
                ctx.runway = atis::spoken_runway(&ident);
                ctx.runway_refusal = None;
                if ctx.on_ground {
                    self.desk.planned_route = pos.and_then(|(lat, lon)| {
                        plan_taxi_route_to(&self.apt.runways, &ident, lat, lon)
                    });
                    let route = self.desk.planned_route.as_ref();
                    ctx.holding_point = route
                        .map(|r| spoken_taxiway(&r.holding_point))
                        .filter(|h| !h.is_empty())
                        .unwrap_or_else(|| "Alpha".to_string());
                    ctx.taxi_via = route.map(|r| spoken_via(&r.via)).unwrap_or_default();
                    ctx.taxi_crossings = route
                        .map(|r| {
                            r.crossings
                                .iter()
                                .map(|n| atis::spoken_runway(first_designator(n)))
                                .collect()
                        })
                        .unwrap_or_default();
                }
                self.desk.call.requested_ident = Some(ident);
            }
        }
    }

    /// The runway in use. Re-picked from the wind only when there IS wind
    /// (≥ 5 kt): in calm the into-wind pick flip-flops on noise, and a
    /// controller doesn't swap the runway between two transmissions.
    pub(super) fn pick_runway(&mut self, wind_from: f64, wind_kts: f64) -> String {
        self.pick_runway_with(wind_from, wind_kts, None, None)
    }

    /// [`pick_runway`] with the live arrival stream: in calm wind, with no
    /// runway in use yet, the end the traffic is landing on is the runway
    /// in use. LXGB at 07001KT: the tower picked 27 while LiveTraffic was
    /// landing 09 (the real field's runway), and cleared a departure into
    /// the reciprocal stream (2026-09-15).
    ///
    /// `arrival_end` is the live lander on final, else the remembered
    /// touchdowns, and `departure_end` the remembered liftoffs, both from
    /// [`field_streams`] (issue #164).
    ///
    /// `departure_end` is the live DEPARTURE stream ([`departure_end`]): with
    /// no arrival stream it is the runway in use in calm the same way, and a
    /// stated runway on its reciprocal is refused. ONE STRIP, ONE DIRECTION:
    /// in calm an uncommitted runway in use that points against the live
    /// stream is turned round to it. LEMG (sortie 1790584715-full_auto,
    /// 2026-09-28): 31 stood in use from an earlier into-wind pick, the
    /// replay departed 13 off the same pavement and landed 12, and the
    /// tower cleared our circuit to land 31 while Vueling and Easy rolled
    /// head-on toward it.
    pub(super) fn pick_runway_with(
        &mut self,
        wind_from: f64,
        wind_kts: f64,
        arrival_end: Option<&str>,
        departure_end: Option<&str>,
    ) -> String {
        // THE RUNWAY IN USE IS THE FIELD'S, OURS TO MOVE: a traffic flight
        // is given it, whatever it stated, and none of the writes below run
        // for it ([`Station::traffic_may_set_runway`]); the one exception is
        // a field with no runway in use yet, which its pick initialises.
        if self.desk.strip.is_traffic() {
            if let Some(in_use) = self.runway_in_use.clone() {
                self.desk.call.stated_on_strip = false;
                self.desk.call.stated_runway = None;
                return in_use;
            }
            let ident = self.pick_runway_for_field(wind_from, wind_kts, arrival_end, departure_end);
            self.note_traffic_initialised_runway(&ident);
            return ident;
        }
        self.pick_runway_for_field(wind_from, wind_kts, arrival_end, departure_end)
    }

    /// The pick itself ([`Station::pick_runway_with`]), free to write the
    /// runway in use.
    fn pick_runway_for_field(
        &mut self,
        wind_from: f64,
        wind_kts: f64,
        arrival_end: Option<&str>,
        departure_end: Option<&str>,
    ) -> String {
        // The stream the field is working: arrivals first (where the traffic
        // is landing is what the field IS), else the last departures.
        let stream_end = arrival_end
            .filter(|e| strip(&self.apt.runways, e).is_some())
            .or(departure_end.filter(|e| strip(&self.apt.runways, e).is_some()));
        // A RUNWAY START IS GIVEN THE RUNWAY IT IS LINED UP ON when the wind
        // does not argue: an arrival stream on another end sequences around
        // a departure already on its runway, it does not send it off to taxi.
        // LEMG (sortie 1790584715-full_auto, 2026-09-28): the 737 set up on
        // 13's threshold in 342 at 3 kt was cleared off 31 and taxied 400 s
        // round the field to line up there.
        // ...but never INTO the live traffic: a start whose end points against
        // the arrival or departure stream (13 departures, a start on 31) is
        // the head-on of issue #140, refused and left to the stream below.
        // ...and a start's own strip carries a pilot's tailwind, not the
        // runway-in-use's: LEMG 13 (sortie 1790646817-full_auto, 2026-09-29,
        // #178), lined up on 13 with no traffic, 293 at 7.3 was 6.9 kt down
        // 13, over the 5 kt of STATED_RUNWAY_TAILWIND_KTS, and the circuit
        // was cleared on 30 across the field. Once given, the strip is HELD
        // for the start's later calls (`start_strip`): 30's 14 kt better
        // headwind would win the wind pick's hysteresis on the ready call.
        let start_ok = |st: &Self, ident: &str| {
            let into_stream = [arrival_end, departure_end]
                .into_iter()
                .flatten()
                .any(|end| ends_oppose(&st.apt.runways, end, ident));
            let hw = nav::headwind_for(&st.apt.runways, ident, wind_from, wind_kts);
            let best = nav::best_runway_for_wind(&st.apt.runways, wind_from, wind_kts);
            !into_stream
                && runway_stands_within(
                    &st.apt.runways,
                    ident,
                    hw,
                    best,
                    wind_from,
                    wind_kts,
                    ON_STRIP_TAILWIND_KTS,
                )
        };
        if std::mem::take(&mut self.desk.call.stated_on_strip) && !self.runway_committed() {
            if let Some(ident) = self.stated_ident() {
                if start_ok(self, &ident) {
                    self.desk.call.stated_runway = None;
                    self.runway_in_use = Some(ident.clone());
                    self.desk.start_strip = Some(ident.clone());
                    return ident;
                }
            }
        }
        if let Some(held) = self.desk.start_strip.take() {
            let restated = self
                .desk.call.stated_runway
                .as_deref()
                .is_some_and(|s| !tower::runway_matches(&held, s));
            if !restated
                && self.runway_in_use.as_deref() == Some(held.as_str())
                && start_ok(self, &held)
            {
                self.desk.call.stated_runway = None;
                self.desk.start_strip = Some(held.clone());
                return held;
            }
        }
        if wind_kts < CALM_WIND_KTS && self.runway_in_use.is_none() {
            if let Some(end) = stream_end {
                self.runway_in_use = Some(end.to_string());
                return end.to_string();
            }
        }
        // A departure being SEQUENCED owns its runway. Once a taxi clearance
        // names one, the pick is frozen until the flight leaves or the
        // clearance dies — a controller does not move you to the other
        // parallel between your taxi clearance and your takeoff clearance.
        // ...and so does an ARRIVAL told to join it or cleared to land on it.
        // A go-around takes the state from ClearedToLand back to Inbound; on
        // a wind dead across the strip the pick flipped in that moment, the
        // party line published the other end, and the crew's approach was
        // re-aimed at it from 60 ft (EGTE 08 to 26, 2026-09-21).
        // A landed arrival that has REPORTED the runway vacated no longer
        // owns it: the pick is free to follow the wind and the stream again.
        // ...and so does an IFR clearance: it named the runway and the SID
        // off it, and the taxi that follows goes to THAT runway
        // (`runway_committed`).
        let committed = self.runway_committed();
        if committed && self.runway_in_use.is_some() {
            return self.runway_in_use.clone().unwrap_or_default();
        }
        // AN ARRIVAL'S OWN RUNWAY STANDS WHEN THE WIND DOES NOT ARGUE. "Final
        // runway two six" into 350 at 8, dead across both ends, earned
        // "negative, runway zero eight": a headwind of a fraction of a knot
        // sent a 737 round the field from a five mile final (same sortie). A
        // controller with nothing else on the runway clears the end the
        // traffic is lined up on. Only uncommitted, only without an arrival
        // stream on another end.
        // "Does not argue" is the pick's own calm (under CALM_WIND_KTS any
        // end stands), a tailwind within STATED_RUNWAY_TAILWIND_KTS on no
        // worse a crosswind than the best end's, or a headwind inside the
        // swap margin. LEMG (sortie 1790580676-approach, 2026-09-28): "10
        // mile final runway one three" into 350 at 3 earned "negative, runway
        // three one" because 13's -2.3 kt trailed 31's +2.3 kt by 4.7, over
        // the 3 kt margin, in a wind the same function calls calm.
        if let Some(stated) = self.desk.call.stated_runway.take() {
            let ident = self
                .apt
                .runways
                .iter()
                .flat_map(|r| [r.ident_a.clone(), r.ident_b.clone()])
                .find(|id| tower::runway_matches(id, &stated));
            if let Some(ident) = ident {
                // The departures' reciprocal is refused too: landing into
                // them is the head-on the stream rule exists to prevent.
                let stream_elsewhere = arrival_end.is_some_and(|end| end != ident)
                    || departure_end
                        .is_some_and(|end| ends_oppose(&self.apt.runways, end, &ident));
                let hw = nav::headwind_for(&self.apt.runways, &ident, wind_from, wind_kts);
                let best = nav::best_runway_for_wind(&self.apt.runways, wind_from, wind_kts);
                let wind_indifferent = stated_runway_stands(
                    &self.apt.runways,
                    &ident,
                    hw,
                    best.clone(),
                    wind_from,
                    wind_kts,
                );
                if !stream_elsewhere && wind_indifferent {
                    self.runway_in_use = Some(ident.clone());
                    return ident;
                }
                // A refused stated runway names its reason on file: the
                // stream's end or the wind's numbers. LEMG 13 into 060 at 6
                // (sortie 1790583663-approach) was refused with the wind
                // arithmetic saying it stands (headwind 2.0 against 12's 3.3,
                // crosswind 5.7 against 5.0), and nothing recorded why.
                sim::blackbox::record(
                    "atc",
                    serde_json::json!({"event": "stated_runway_refused", "stated": ident,
                        "stream_end": arrival_end, "departure_end": departure_end,
                        "headwind_kts": hw,
                        "best": best.as_ref().map(|b| b.0.clone()),
                        "best_headwind_kts": best.as_ref().map(|b| b.2),
                        "wind_from": wind_from, "wind_kts": wind_kts,
                        "wind_indifferent": wind_indifferent}),
                );
            }
        }
        // One strip, one direction: in calm the stream's direction is the
        // runway in use. A pick that points against it (the reciprocal, or a
        // near-parallel's opposite end) turns round to the stream's end; the
        // wind below never moves a calm pick, so nothing else would.
        if wind_kts < CALM_WIND_KTS {
            if let (Some(cur), Some(end)) = (self.runway_in_use.clone(), stream_end) {
                if ends_oppose(&self.apt.runways, &cur, end) {
                    tracing::info!(from = %cur, to = %end, "runway in use turned to the live stream");
                    sim::blackbox::record(
                        "atc",
                        serde_json::json!({"event": "runway_follows_stream", "from": cur,
                            "to": end, "arrival_end": arrival_end,
                            "departure_end": departure_end, "wind_kts": wind_kts}),
                    );
                    self.runway_in_use = Some(end.to_string());
                }
            }
        }
        if wind_kts >= CALM_WIND_KTS || self.runway_in_use.is_none() {
            let best = nav::best_runway_for_wind(&self.apt.runways, wind_from, wind_kts);
            let best = if self.desk.call.departing {
                self.departure_wind_pick(best, wind_from, wind_kts)
            } else {
                best
            };
            if let Some((id, _, hw)) = best {
                // Hysteresis: the runway in use keeps its job unless another
                // beats it by a REAL margin. The >= 5 kt guard was meant to
                // stop the flip-flop, but between two parallels the headwinds
                // differ by rounding error at any wind speed — EGLL's strips
                // are both 270 degrees to within a hundredth of a degree, so
                // which one won was decided by noise in the wind direction,
                // and nothing in the phraseology gives it away (both are "two
                // seven"). A silent flip from the 27L taxi clearance to 27R
                // pointed the takeoff hold at Heathrow's ARRIVAL stream, which
                // never has a 15 s gap: held at November one for the full
                // eight minutes and handed back, "traffic on short final" for
                // a departure runway that was empty (2026-07-27).
                // A departure's current end with no way on at its numbers
                // holds nothing against the pick (LEMD 18R, 2026-10-05).
                let current = self
                    .runway_in_use
                    .as_deref()
                    .filter(|cur| !self.desk.call.departing || self.departs_from(cur))
                    .and_then(|cur| nav::headwind_for(&self.apt.runways, cur, wind_from, wind_kts));
                if current.is_none_or(|cur_hw| hw > cur_hw + RUNWAY_SWAP_MARGIN_KTS) {
                    self.runway_in_use = Some(id);
                }
            }
        }
        // An uncommitted pick sitting on a CLOSE parallel's standby moves to
        // the main runway: the hysteresis above can never make that swap
        // (their headwinds agree to rounding), so a 26R picked on noise
        // before the fix, or by a calm-wind stream, would stand for good.
        if let Some(cur) = self.runway_in_use.as_deref() {
            let main = nav::close_parallel_main(&self.apt.runways, cur);
            if main != cur && (!self.desk.call.departing || self.departs_from(&main)) {
                tracing::info!(from = %cur, to = %main, "runway in use moved to the main runway");
                self.runway_in_use = Some(main);
            }
        }
        self.runway_in_use.clone().unwrap_or_default()
    }

    /// A traffic flight may write the runway in use only while the field has
    /// none ([`Station::pick_runway_with`]): every other write of it is
    /// ours. Logged when it does.
    pub(super) fn traffic_may_set_runway(&self) -> bool {
        !self.desk.strip.is_traffic() || self.runway_in_use.is_none()
    }

    /// A traffic call's pick gave the field its first runway in use.
    fn note_traffic_initialised_runway(&self, ident: &str) {
        if ident.is_empty() {
            return;
        }
        tracing::info!(runway = %ident, "runway in use initialised by a traffic call");
        sim::blackbox::record(
            "atc",
            serde_json::json!({"event": "runway_in_use_initialised", "by": "traffic",
                "runway": ident}),
        );
    }

    /// A departure from `ident` gets onto the strip on its threshold's half
    /// (`TaxiNet::departure_end_enterable`); `true` without a network.
    fn departs_from(&self, ident: &str) -> bool {
        let Some(net) = taxi_net() else {
            return true;
        };
        strip(&self.apt.runways, ident).is_none_or(|r| net.departure_end_enterable(r, ident))
    }

    /// The wind's pick for a DEPARTURE: an end with no way on at its
    /// threshold's half gives way to one that has, of the ends the wind
    /// allows (`runway_stands_within`, a 5 kt tailwind), a parallel of the
    /// wind's pick first. None allowed: the wind's pick stands, backtrack
    /// and all (a single strip). LEMD 18R (2026-10-05): its only ways on
    /// are at the 36L end, and the 737 was sent 4 km down the wrong end.
    fn departure_wind_pick(
        &self,
        best: Option<(String, f64, f64)>,
        wind_from: f64,
        wind_kts: f64,
    ) -> Option<(String, f64, f64)> {
        let (id, hdg, _) = best.clone()?;
        if self.departs_from(&id) {
            return best;
        }
        let runways = &self.apt.runways;
        nav::assignable_runways(runways)
            .flat_map(|r| {
                [
                    (r.ident_a.clone(), r.heading_a_deg()),
                    (r.ident_b.clone(), r.heading_b_deg()),
                ]
            })
            .filter(|(e, _)| self.departs_from(e))
            .filter_map(|(e, h)| {
                let hw = nav::headwind_for(runways, &e, wind_from, wind_kts)?;
                runway_stands_within(
                    runways,
                    &e,
                    Some(hw),
                    best.clone(),
                    wind_from,
                    wind_kts,
                    STATED_RUNWAY_TAILWIND_KTS,
                )
                .then_some((e, h, hw))
            })
            .max_by(|a, b| {
                let par = |h: f64| sim::angles::abs_diff_deg(h, hdg) <= nav::PARALLEL_DEG;
                (par(a.1), a.2)
                    .partial_cmp(&(par(b.1), b.2))
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .or(best)
    }
}
