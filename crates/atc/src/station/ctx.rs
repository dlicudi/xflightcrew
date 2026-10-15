//! The live context a reply is built from.

use super::*;

/// Our own aircraft, read from the sim once for one call. Best-effort, as
/// the reads it replaces: a failed ground flag is airborne, a failed
/// position is NaN (no position, so nothing is planned or judged from it),
/// a failed heading `None`, a failed helicopter flag an aeroplane.
pub(super) async fn read_own_state(xp: SimView<'_>) -> AircraftState {
    let on_ground = xp.get_float(datarefs::ON_GROUND_ANY).await.unwrap_or(0.0) > 0.5;
    let (lat, lon) = match (
        xp.get_float(datarefs::LATITUDE).await,
        xp.get_float(datarefs::LONGITUDE).await,
    ) {
        (Ok(lat), Ok(lon)) => (lat, lon),
        _ => (f64::NAN, f64::NAN),
    };
    let alt_ft = xp
        .get_float(datarefs::ELEVATION_M)
        .await
        .map(|m| m * 3.28084)
        .unwrap_or(f64::NAN);
    let gs_kts = xp
        .get_float(datarefs::GROUNDSPEED_MS)
        .await
        .map(|v| v * 1.943844)
        .unwrap_or(f64::NAN);
    let psi_deg = xp.get_float(TRUE_PSI).await.ok();
    let helicopter = xp
        .get_float("sim/aircraft2/metadata/is_helicopter")
        .await
        .is_ok_and(|v| v > 0.5);
    AircraftState {
        lat,
        lon,
        alt_ft,
        on_ground,
        gs_kts,
        psi_deg,
        helicopter,
        at: Instant::now(),
    }
}

impl Station {
    /// Read the sim once and build the live context for a reply. Best-effort:
    /// a failed read falls back to silence on that item rather than a wrong
    /// number. Our own aircraft is `state` when the caller holds it, else
    /// read here, once, before anything else ([`read_own_state`]).
    pub(super) async fn build_ctx(
        &mut self,
        xp: SimView<'_>,
        state: Option<&AircraftState>,
        intent: tower::Intent,
    ) -> (tower::Ctx, Wx) {
        let state = match state {
            Some(s) => *s,
            None => read_own_state(xp).await,
        };
        let mut wx = Wx {
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
        // One resolver with the crew (override > SimBrief > tail). The wake
        // category rides the sim's own MTOW, so the tower says "heavy" to an
        // A330 without the crew's profile in its dep tree.
        let callsign = crate::callsign::live(xp)
            .await
            .map(|r| r.spoken)
            .unwrap_or_else(|| "aircraft calling".to_string());
        // The wind a tower reads out and picks its runway by is the wind ON
        // THE FIELD. The sim's wind datarefs are the air the aircraft is in:
        // for a caller on the ground the same thing, for an arrival at
        // 2,700 ft the winds aloft (350 at 8 against the field's 190 at 3,
        // EGTE 2026-09-21: crew and tower picked opposite ends).
        if !state.on_ground {
            if let Some((from, kts)) = surface_wind_at(self.apt.position()) {
                wx.wind_from_deg = from;
                wx.wind_kts = kts;
            }
        }
        // The controller's own traffic picture, read once per call: an
        // arrival in OUR runway's corridor gates takeoff clearances and
        // sequences downwind joins; a ghost ON the strip blocks line-ups.
        // Restricted to the strip in use: the runway-blind version held a
        // Heathrow 09R departure for the 09L stream, indefinitely
        // (2026-07-10).
        let targets = targets(xp).await;
        let now = Instant::now();
        // The FIELD's facts, ours to write: the ghost memory, the wind the
        // tower worked with and the pressure it gave (a traffic call's ctx
        // reads the last two, `Station::traffic_ctx`).
        note_ground_traffic(&targets, now);
        *TOWER_WIND.lock().unwrap_or_else(|e| e.into_inner()) =
            Some((wx.wind_from_deg, wx.wind_kts));
        if !qnh.is_empty() {
            self.last_qnh = Some(qnh.clone());
        }
        // Our mass, for a departure route that must start ahead of the
        // nose (#288): read only for a ground call of ours.
        self.desk.call.own_mtow_kg = if state.on_ground && !self.desk.strip.is_traffic() {
            xp.get_float(datarefs::ACF_M_MAX_KG).await.ok()
        } else {
            None
        };
        self.ctx_for_call(&state, wx, qnh, callsign, intent, &targets, &[], now)
    }

    /// The desk half of a call's context, after the field's reads (ours in
    /// [`Station::build_ctx`], a traffic flight's in
    /// [`Station::traffic_ctx`]): the runway pick, the traffic on it, the
    /// taxi route from where the caller is, ending in [`Station::make_ctx`].
    /// Writes nothing of the field's but through the pick. `ids` names the
    /// flights in `targets` that the picture knows (by raw callsign): the
    /// nearest arrival's becomes `final_traffic_id`.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn ctx_for_call(
        &mut self,
        state: &AircraftState,
        wx: Wx,
        qnh: String,
        callsign: String,
        intent: tower::Intent,
        targets: &[TcasTarget],
        ids: &[(&str, FlightId)],
        now: Instant,
    ) -> (tower::Ctx, Wx) {
        let on_ground = state.on_ground;
        self.desk.call.field_nm = state
            .position()
            .zip(self.apt.position())
            .map(|((lat, lon), (flat, flon))| crate::nav::haversine_nm(lat, lon, flat, flon));
        self.desk.call.caller_at = state.position().map(|(lat, lon)| (lat, lon, state.alt_ft));
        // An IFR clearance's runway is held for its departure only: airborne
        // it has been flown (or abandoned), and the pick is free again.
        // So is a runway start's strip (`start_strip`).
        if !on_ground {
            self.desk.ifr_runway = None;
            self.desk.start_strip = None;
        }
        // The live lander on final, else the remembered touchdowns (#164).
        let (arrival_end, departure_end) = field_streams(
            &self.apt.runways,
            arrival_end(targets, &self.apt.runways, self.apt.elev_ft),
            now,
        );
        if on_ground && self.desk.call.stated_runway.is_some() {
            if let Some((lat, lon)) = state.position() {
                self.note_caller_on_strip(on_ground, lat, lon);
            }
        }
        self.desk.call.departing = on_ground
            && !matches!(intent, tower::Intent::RequestTaxiIn)
            && self.desk.twr.state != tower::State::ClearedToLand;
        let ident = self.pick_runway_with(
            wx.wind_from_deg,
            wx.wind_kts,
            arrival_end.as_deref(),
            departure_end.as_deref(),
        );
        // Our own position, for a takeoff request made ON the strip: only
        // the pavement ahead of us is in the way of the roll.
        let own = if on_ground && intent == tower::Intent::RequestTakeoff {
            state.position()
        } else {
            None
        };
        let (final_traffic, runway_occupied) = match strip(&self.apt.runways, &ident) {
            Some(r) => {
                let r = r.clone();
                // The landing judgement's own test: on the pavement and not
                // stale data (`occupies`, issue #162); a departure standing
                // on the strip is not held by a ghost behind its tail (#172).
                let occupied = departure_strip_occupied_for(&self.desk.strip, &r, &ident, targets, own, now);
                (final_traffic_for(targets, &r, &self.apt.runways, self.apt.elev_ft), occupied)
            }
            None => (None, false),
        };
        // A downwind report is sequenced further out: behind a straight-in
        // on a longer final ([`sequence_traffic_for`]), or, with the strip
        // held, behind its occupant, whom the downwind follows (extended)
        // until it is off (BIEG 03, 2026-10-08: "number two, traffic on the
        // runway" named no leader, the base was turned on time and the
        // final went around behind a backtracking jet).
        let downwind = intent == tower::Intent::ReportDownwind && !on_ground;
        let final_traffic = match (&final_traffic, strip(&self.apt.runways, &ident)) {
            (None, Some(r)) if downwind && !runway_occupied => {
                sequence_traffic_for(targets, r, &self.apt.runways, self.apt.elev_ft)
            }
            _ => final_traffic,
        };
        let occupant = match strip(&self.apt.runways, &ident) {
            Some(r) if downwind && runway_occupied => {
                departure_strip_picture(r, &ident, targets, None, None, now)
                    .occupants
                    .first()
                    .cloned()
            }
            _ => None,
        };
        let mut final_traffic_id = occupant
            .as_ref()
            .or_else(|| final_traffic.as_ref().and_then(|f| f.ghosts.first()))
            .and_then(|nearest| ids.iter().find(|(cs, _)| cs == nearest))
            .map(|(_, who)| *who);
        // ...and behind the LAST flight the tower has already sequenced,
        // when there is one: that flight is the one to follow.
        let mut sequence_ahead = None;
        if downwind {
            if let Some((n, who)) = landing_sequence_tail(&self.desk.strip) {
                let on_final = final_traffic.as_ref().is_some_and(|f| {
                    ids.iter()
                        .any(|(cs, id)| *id == who && f.ghosts.iter().any(|g| g == cs))
                });
                let at = if on_final { "on final" } else { "on downwind" };
                sequence_ahead = Some((n.saturating_add(1), at.to_string()));
                final_traffic_id = Some(who);
            }
        }
        let wake_secs_left = if on_ground {
            wake_remaining_for(&self.desk.strip, &ident, now)
        } else {
            None
        };
        // The taxi route from where the aircraft actually is, planned fresh
        // per ground call so the eventual clearance matches the stand you
        // are on, not the one you asked from an hour ago.
        // An ARRIVAL taxi-in (the pilot said "taxi to parking" — RequestTaxiIn,
        // or a bare "request taxi" while still holding a landing clearance)
        // routes to the nearest STAND; everything else is a departure to the
        // runway. Keyed on the INTENT, not the FSM state — the state falls back
        // to Parked after landing (region handoff, a prior taxi-in), which used
        // to leave a real "taxi to parking" answered flat with no route
        // (2026-07-26).
        let arrival = matches!(intent, tower::Intent::RequestTaxiIn)
            || (matches!(intent, tower::Intent::RequestTaxi)
                && self.desk.twr.state == tower::State::ClearedToLand);
        self.desk.planned_route = None;
        let mut own_on_runway = false;
        let mut pos = None;
        // The nose a departure's route starts ahead of, when the aircraft
        // cannot turn on the spot (LEMG 2026-10-14, #288).
        let mtow_kg = self.desk.call.own_mtow_kg.take();
        let facing = facing_for_route(self.desk.strip.is_traffic(), state, mtow_kg);
        if on_ground {
            if let Some((lat, lon)) = state.position() {
                pos = Some((lat, lon));
                own_on_runway = strip(&self.apt.runways, &ident)
                    .is_some_and(|r| on_runway(r, lat, lon).is_some());
                self.desk.planned_route = if arrival {
                    // The heading says which way is forward off a runway we
                    // may still be on; a failed read plans as before.
                    // A traffic arrival passes over the stands its picture
                    // shows somebody on (the parked aircraft); ours over
                    // the stands our own fleet's parked aircraft stand on
                    // (#243), else it plans as it always did.
                    let mut occupied: Vec<(f64, f64)> = if self.desk.strip.is_traffic() {
                        targets
                            .iter()
                            .filter(|t| t.on_ground)
                            .map(|t| (t.lat, t.lon))
                            .collect()
                    } else {
                        stands_taken()
                    };
                    // ...and the stands already given to another arrival still
                    // taxiing in (UUYY, world sweep 2026-10-09: two arrivals
                    // sent to "stand one niner" a minute apart).
                    occupied.extend(stands_assigned(&self.desk.strip));
                    plan_taxi_route_to_stand(&self.apt.runways, lat, lon, state.psi_deg, &occupied)
                } else {
                    plan_taxi_route_to(&self.apt.runways, &ident, lat, lon, facing)
                };
                // A push is approved with the way it will leave the nose:
                // off the stand's drawn lane, toward the route on.
                if intent == tower::Intent::RequestPushback {
                    self.desk.call.push_facing = match (taxi_net(), state.psi_deg) {
                        (Some(net), Some(psi)) => push_facing_deg(
                            &net,
                            lat,
                            lon,
                            psi,
                            self.desk.planned_route.as_ref().map(|r| r.path.as_slice()),
                        ),
                        _ => None,
                    };
                }
            }
        }
        let mut ctx = self.make_ctx(
            wx,
            qnh,
            callsign,
            on_ground,
            arrival,
            final_traffic,
            runway_occupied,
            wake_secs_left,
        );
        ctx.own_on_runway = own_on_runway;
        ctx.final_traffic_id = final_traffic_id;
        ctx.sequence_ahead = sequence_ahead;
        if intent == tower::Intent::RequestRunway {
            self.apply_runway_request(&mut ctx, pos, facing);
        }
        if intent == tower::Intent::RequestCrossing && on_ground {
            ctx.crossing = self.judge_crossing(targets, pos, now);
        }
        ctx.helicopter = state.helicopter;
        // A final report with traffic landing ahead that will not be off in
        // time: number two, never a clearance the tower must cancel (#237).
        if intent == tower::Intent::ReportFinal && !on_ground {
            ctx.landing_leader = landing_leader_blocking(&self.desk.strip, &ident, targets, now);
        }
        (ctx, wx)
    }

    /// The pure half of [`Station::build_ctx`] — everything after the sim
    /// reads, so tests phrase clearances from scripted weather.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn make_ctx(
        &mut self,
        wx: Wx,
        qnh: String,
        callsign: String,
        on_ground: bool,
        arrival: bool,
        final_traffic: Option<FinalTraffic>,
        runway_occupied: bool,
        wake_secs_left: Option<u64>,
    ) -> tower::Ctx {
        let runway = self.pick_runway(wx.wind_from_deg, wx.wind_kts);
        // Another strip the tower has cleared onto (or to land on) the
        // runway, or one standing on it, occupies it, by the book, before
        // any ghost is seen on the pavement: the caller's line-up, takeoff
        // and landing wait for it (ours for a traffic strip, theirs for us).
        // For a caller on the ground a lander still flying is an ARRIVAL,
        // never "traffic on the runway" (LEMG 12, 2026-10-13), judged on
        // TIME by the release's one rule ([`DepartureBlock::holds`]): one
        // the departure cannot be off the strip ahead of is landing traffic
        // on final at its distance, any distance; one it can is nothing
        // (LEMG 13, 2026-10-13: RYR5024 cleared from the holding point with
        // us 4.02 nm and 99 s out, and we were sent round).
        let block = if on_ground {
            departure_held_by_others(&self.apt.runways, &runway, &self.desk.strip)
        } else {
            strip_held_by_others(&self.apt.runways, &runway, &self.desk.strip)
                .then_some(DepartureBlock::Occupied)
        };
        let lined_up = self.desk.twr.state == tower::State::LinedUp;
        // A crosser waiting short of the strip holds a new departure as an
        // occupant would ([`DepartureBlock::Crosser`]).
        let runway_occupied = runway_occupied
            || block == Some(DepartureBlock::Occupied)
            || matches!(block, Some(DepartureBlock::Crosser(_))) && !lined_up;
        let final_traffic = match block.filter(|b| b.holds(lined_up)).and_then(|b| b.lander()) {
            Some(Lander { nm, .. }) => Some(match final_traffic {
                Some(f) if f.nm <= nm => f,
                Some(f) => FinalTraffic { nm, ..f },
                None => FinalTraffic {
                    nm,
                    callsign: None,
                    count: 1,
                    ghosts: Vec::new(),
                    type_spoken: None,
                },
            }),
            _ => final_traffic,
        };
        // "With information X" matching the current broadcast: the clearance
        // skips the QNH — having the numbers is the whole point of an ATIS.
        // A stale or absent letter keeps them (safety over brevity).
        let qnh = if self.desk.pilot_info.is_some() && self.desk.pilot_info == atis_current_letter() {
            String::new()
        } else {
            qnh
        };
        let route = self.desk.planned_route.as_ref();
        let missed_approach = self.missed_approach_for(&callsign);
        tower::Ctx {
            dialect: self.dialect,
            uk: self.speaks_cap413(),
            missed_approach,
            final_traffic_type: final_traffic.as_ref().and_then(|f| f.type_spoken.clone()),
            landing_leader: None,
            callsign,
            airport: self.apt.radio_name(),
            runway: atis::spoken_runway(&runway),
            qnh,
            wind: spoken_wind(wx.wind_from_deg, wx.wind_kts, self.dialect),
            holding_point: route
                .map(|r| spoken_taxiway(&r.holding_point))
                .filter(|h| !h.is_empty())
                .unwrap_or_else(|| "Alpha".to_string()),
            // Arrival taxi-in (route was planned to a stand): the spoken stand
            // name for "taxi to stand ...". Empty on a departure.
            stand: if arrival {
                route
                    .map(|r| spoken_stand_in(&r.holding_point, self.dialect))
                    .unwrap_or_default()
            } else {
                String::new()
            },
            approach: self
                .approach_khz
                .map(|k| atis::spoken_mhz_in(k, self.dialect)),
            squawk: self
                .desk.assigned_squawk
                .map(|c| atis::spoken_digits(&c.to_string()))
                .unwrap_or_else(|| "seven zero zero zero".to_string()),
            final_traffic_nm: final_traffic.as_ref().map(|f| f.nm),
            final_traffic_callsign: final_traffic.as_ref().and_then(|f| f.callsign.clone()),
            final_traffic_count: final_traffic.as_ref().map(|f| f.count).unwrap_or(0),
            // Named by the caller's picture where it can (`ctx_for_call`).
            final_traffic_id: None,
            sequence_ahead: None,
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
            // Our own AI traffic holds short of each crossing and asks for
            // it at the runway; ours is cleared across here, as before.
            hold_short_crossings: self.desk.strip.is_traffic(),
            // Judged by `ctx_for_call` for a crossing request.
            crossing: None,
            on_ground,
            // The sim read is `build_ctx`'s; scripted contexts are off it.
            own_on_runway: false,
            helicopter: false,
            ifr: None,
            circuit: None,
            runway_refusal: None,
            transition_level_ft: self.apt.transition_level_ft,
        }
    }

    /// A crossing request judged: the runway its words named at this field,
    /// spoken, and why the flight must hold short when it must. Another
    /// flight's strip holds the strip or stands on it (the runway book, ours
    /// included for a traffic flight), the picture shows somebody on its
    /// pavement, or an arrival is on its final inside [`CROSSING_FINAL_NM`].
    /// The end is noted for the reply (`crossing_ident`): cleared, it is
    /// the crossing; refused, the crosser waits for the next gap.
    /// `None` when the words named no runway here.
    fn judge_crossing(
        &mut self,
        targets: &[TcasTarget],
        pos: Option<(f64, f64)>,
        now: Instant,
    ) -> Option<(String, Option<String>)> {
        let heard = self.desk.call.crossing_request.clone()?;
        let ident = self
            .apt
            .runways
            .iter()
            .flat_map(|r| [r.ident_a.clone(), r.ident_b.clone()])
            .find(|id| tower::runway_matches(id, &heard))?;
        let r = strip(&self.apt.runways, &ident)?.clone();
        let occupied = strip_held_by_others(&self.apt.runways, &ident, &self.desk.strip)
            || departure_strip_picture(&r, &ident, targets, None, None, now).blocked;
        // A "crossing" whose route runs ALONG the strip is a backtrack: BIEG
        // 03 (world sweep, 2026-10-08) rolled to the far end, then taxied
        // 1,850 m back down the runway to the apron, four minutes on it,
        // cleared as a 45 m crossing with a Cessna on base; 22 go-arounds
        // in the hour. The time it holds the strip is judged against
        // everything that could be landing in it.
        let on_strip_m = self
            .desk
            .strip
            .taxi_route
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .map_or(0.0, |w| route_on_strip_m(&w.path, pos, &r));
        let why = if occupied {
            Some("traffic on the runway")
        } else if final_traffic_for(targets, &r, &self.apt.runways, self.apt.elev_ft)
            .is_some_and(|f| f.nm < CROSSING_FINAL_NM)
        {
            Some("traffic on final")
        } else if on_strip_m > BACKTRACK_M
            && arrival_within(
                targets,
                &r,
                &self.apt.runways,
                self.apt.elev_ft,
                backtrack_clear_nm(on_strip_m),
            )
        {
            Some("traffic inbound")
        } else {
            None
        };
        self.desk.call.crossing_ident = Some(ident.clone());
        Some((atis::spoken_runway(&ident), why.map(str::to_string)))
    }
}

/// The landing sequence the tower has given out, not counting `me`: how
/// many flights hold a place in it and the last of them (the one a new
/// downwind follows). A place counts while its flight is still Inbound
/// for it: every strip, ours and the traffic's, holding one ("number two,
/// ..."), the highest number last. `None` with nobody sequenced.
pub(super) fn landing_sequence_tail(me: &FlightStrip) -> Option<(u8, FlightId)> {
    let place = |s: &FlightStrip| {
        let inbound = s
            .clearance
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .is_some_and(|c| c.state == tower::State::Inbound);
        inbound
            .then(|| *s.sequence.lock().unwrap_or_else(|e| e.into_inner()))
            .flatten()
    };
    let own = (!std::ptr::eq(me, &OWN_STRIP))
        .then(|| place(&OWN_STRIP).map(|(n, _)| (n, FlightId::Own)))
        .flatten();
    let placed: Vec<(u8, FlightId)> = traffic_strips()
        .into_iter()
        .filter(|(_, s)| !std::ptr::eq(me, s.as_ref()))
        .filter_map(|(id, s)| place(&s).map(|(n, _)| (n, FlightId::Traffic(id))))
        .chain(own)
        .collect();
    let last = placed.iter().max_by_key(|(n, _)| *n)?.1;
    Some((u8::try_from(placed.len()).unwrap_or(u8::MAX), last))
}

/// Where the stands the tower has given to OTHER arrivals are: the end of
/// every other strip's taxi-in route, ours and the traffic's, not counting
/// `me`. The flight is not on its stand yet, but the stand is spoken for.
pub(super) fn stands_assigned(me: &FlightStrip) -> Vec<(f64, f64)> {
    let end = |s: &FlightStrip| {
        if !*s.taxi_to_stand.lock().unwrap_or_else(|e| e.into_inner()) {
            return None;
        }
        s.taxi_route
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .and_then(|w| w.path.last().copied())
    };
    let own = (!std::ptr::eq(me, &OWN_STRIP)).then(|| end(&OWN_STRIP)).flatten();
    traffic_strips()
        .into_iter()
        .filter(|(_, s)| !std::ptr::eq(me, s.as_ref()))
        .filter_map(|(_, s)| end(&s))
        .chain(own)
        .collect()
}

/// An arrival this close on a strip's final keeps a crossing flight short of
/// it: a light aircraft crossing a 45 m runway is a minute on it.
pub(super) const CROSSING_FINAL_NM: f64 = 3.0;

/// A crossing route this long on the strip's pavement is a backtrack,
/// metres: a crossing of the widest runway is under 100 m of it.
pub(super) const BACKTRACK_M: f64 = 150.0;

/// A backtrack's pace, m/s (12 kt: a brisk taxi down a runway).
const BACKTRACK_MPS: f64 = 6.2;

/// The approach speed an arrival is judged at, knots: the fastest of the
/// fleet's finals, so a jet is never under-judged.
const ARRIVAL_KTS: f64 = 150.0;

/// How far out an arrival must be for a backtrack of `on_strip_m` to be
/// off the strip before it lands, nm: the drive at [`BACKTRACK_MPS`] plus
/// half a minute, flown at [`ARRIVAL_KTS`]; never less than a crossing's.
pub(super) fn backtrack_clear_nm(on_strip_m: f64) -> f64 {
    let secs = on_strip_m / BACKTRACK_MPS + 30.0;
    (secs * ARRIVAL_KTS / 3600.0).max(CROSSING_FINAL_NM)
}

/// Metres of `path` still to drive on `rwy`'s pavement, from the point of
/// the path nearest `pos` on (the whole path with no position): the legs
/// already driven, a vacate's rollout among them, are not owed.
pub(super) fn route_on_strip_m(
    path: &[(f64, f64)],
    pos: Option<(f64, f64)>,
    rwy: &nav::Runway,
) -> f64 {
    let from = pos.map_or(0, |(lat, lon)| {
        path.iter()
            .enumerate()
            .min_by(|a, b| {
                let d = |p: &(f64, f64)| nav::haversine_nm(lat, lon, p.0, p.1);
                d(a.1).total_cmp(&d(b.1))
            })
            .map_or(0, |(i, _)| i)
    });
    path.get(from..)
        .unwrap_or(&[])
        .windows(2)
        .filter(|w| on_runway(rwy, (w[0].0 + w[1].0) / 2.0, (w[0].1 + w[1].1) / 2.0).is_some())
        .map(|w| nav::haversine_nm(w[0].0, w[0].1, w[1].0, w[1].1) * 1852.0)
        .sum()
}

/// Anything that may land on `rwy` within `nm` of it: airborne, low (the
/// circuit and an arrival's descent, under 4,000 ft above the field), not
/// climbing away, not on an independent parallel's final. Straight-line
/// distance, so circuit traffic flying a longer way round is judged
/// early, never late.
pub(super) fn arrival_within(
    targets: &[TcasTarget],
    rwy: &nav::Runway,
    runways: &[nav::Runway],
    field_elev_ft: f64,
    nm: f64,
) -> bool {
    targets
        .iter()
        .filter(|t| !t.on_ground)
        .filter(|t| t.ele_ft - field_elev_ft < 4000.0)
        .filter(|t| t.vs_fpm <= CLIMBOUT_FPM)
        .filter(|t| !on_independent_final(t, rwy, runways))
        .any(|t| {
            nav::haversine_nm(t.lat, t.lon, rwy.lat_a, rwy.lon_a)
                .min(nav::haversine_nm(t.lat, t.lon, rwy.lat_b, rwy.lon_b))
                < nm
        })
}
