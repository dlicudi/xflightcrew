//! The live context a reply is built from.

use super::*;

impl Station {
    /// Read the sim once and build the live context for a reply. Best-effort:
    /// a failed read falls back to silence on that item rather than a wrong
    /// number.
    pub(super) async fn build_ctx(&mut self, xp: SimView<'_>, intent: tower::Intent) -> (tower::Ctx, Wx) {
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
        let on_ground = xp.get_float(datarefs::ON_GROUND_ANY).await.unwrap_or(0.0) > 0.5;
        // An IFR clearance's runway is held for its departure only: airborne
        // it has been flown (or abandoned), and the pick is free again.
        // So is a runway start's strip (`start_strip`).
        if !on_ground {
            self.ifr_runway = None;
            self.start_strip = None;
        }
        // The wind a tower reads out and picks its runway by is the wind ON
        // THE FIELD. The sim's wind datarefs are the air the aircraft is in:
        // for a caller on the ground the same thing, for an arrival at
        // 2,700 ft the winds aloft (350 at 8 against the field's 190 at 3,
        // EGTE 2026-09-21: crew and tower picked opposite ends).
        if !on_ground {
            if let Some((from, kts)) = surface_wind_at(self.apt.position()) {
                wx.wind_from_deg = from;
                wx.wind_kts = kts;
            }
        }
        // The controller's own traffic picture, read once per call: an
        // arrival in OUR runway's corridor gates takeoff clearances and
        // sequences downwind joins; a ghost ON the strip blocks line-ups.
        // Restricted to the strip in use — the runway-blind version held a
        // Heathrow 09R departure for the 09L stream, indefinitely
        // (2026-07-10).
        let targets = read_tcas_targets(xp).await;
        let now = Instant::now();
        note_ground_traffic(&targets, now);
        *TOWER_WIND.lock().unwrap_or_else(|e| e.into_inner()) =
            Some((wx.wind_from_deg, wx.wind_kts));
        // The live lander on final, else the remembered touchdowns (#164).
        let (arrival_end, departure_end) = field_streams(
            &self.apt.runways,
            arrival_end(&targets, &self.apt.runways, self.apt.elev_ft),
            now,
        );
        if on_ground && self.stated_runway.is_some() {
            if let (Ok(lat), Ok(lon)) = (
                xp.get_float(datarefs::LATITUDE).await,
                xp.get_float(datarefs::LONGITUDE).await,
            ) {
                self.note_caller_on_strip(on_ground, lat, lon);
            }
        }
        self.departing = on_ground
            && !matches!(intent, tower::Intent::RequestTaxiIn)
            && self.twr.state != tower::State::ClearedToLand;
        let ident = self.pick_runway_with(
            wx.wind_from_deg,
            wx.wind_kts,
            arrival_end.as_deref(),
            departure_end.as_deref(),
        );
        // Our own position, for a takeoff request made ON the strip: only
        // the pavement ahead of us is in the way of the roll.
        let own = if on_ground && intent == tower::Intent::RequestTakeoff {
            match (
                xp.get_float(datarefs::LATITUDE).await,
                xp.get_float(datarefs::LONGITUDE).await,
            ) {
                (Ok(lat), Ok(lon)) => Some((lat, lon)),
                _ => None,
            }
        } else {
            None
        };
        let (final_traffic, runway_occupied) = match strip(&self.apt.runways, &ident) {
            Some(r) => {
                let r = r.clone();
                // The landing judgement's own test: on the pavement and not
                // stale data (`occupies`, issue #162); a departure standing
                // on the strip is not held by a ghost behind its tail (#172).
                let occupied = departure_strip_occupied(&r, &ident, &targets, own, now);
                (final_traffic_for(&targets, &r, &self.apt.runways, self.apt.elev_ft), occupied)
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
        // An ARRIVAL taxi-in (the pilot said "taxi to parking" — RequestTaxiIn,
        // or a bare "request taxi" while still holding a landing clearance)
        // routes to the nearest STAND; everything else is a departure to the
        // runway. Keyed on the INTENT, not the FSM state — the state falls back
        // to Parked after landing (region handoff, a prior taxi-in), which used
        // to leave a real "taxi to parking" answered flat with no route
        // (2026-07-26).
        let arrival = matches!(intent, tower::Intent::RequestTaxiIn)
            || (matches!(intent, tower::Intent::RequestTaxi)
                && self.twr.state == tower::State::ClearedToLand);
        self.planned_route = None;
        let mut own_on_runway = false;
        let mut pos = None;
        if on_ground {
            if let (Ok(lat), Ok(lon)) = (
                xp.get_float(datarefs::LATITUDE).await,
                xp.get_float(datarefs::LONGITUDE).await,
            ) {
                pos = Some((lat, lon));
                own_on_runway = strip(&self.apt.runways, &ident)
                    .is_some_and(|r| on_runway(r, lat, lon).is_some());
                self.planned_route = if arrival {
                    // The heading says which way is forward off a runway we
                    // may still be on; a failed read plans as before.
                    let psi = xp.get_float(TRUE_PSI).await.ok();
                    plan_taxi_route_to_stand(&self.apt.runways, lat, lon, psi)
                } else {
                    plan_taxi_route_to(&self.apt.runways, &ident, lat, lon)
                };
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
        if intent == tower::Intent::RequestRunway {
            self.apply_runway_request(&mut ctx, pos);
        }
        ctx.helicopter = xp
            .get_float("sim/aircraft2/metadata/is_helicopter")
            .await
            .is_ok_and(|v| v > 0.5);
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
            // Arrival taxi-in (route was planned to a stand): the spoken stand
            // name for "taxi to stand ...". Empty on a departure.
            stand: if arrival {
                route
                    .map(|r| spoken_stand(&r.holding_point))
                    .unwrap_or_default()
            } else {
                String::new()
            },
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
            // The sim read is `build_ctx`'s; scripted contexts are off it.
            own_on_runway: false,
            helicopter: false,
            ifr: None,
            circuit: None,
            runway_refusal: None,
            transition_level_ft: self.apt.transition_level_ft,
        }
    }
}
