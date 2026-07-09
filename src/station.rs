//! The ATC station living inside the copilot process — the merge of the
//! standalone `atc` binary's tower service into the one binary the pilot
//! actually flies with. One mic, one PTT, one speech arbiter; the tuned COM1
//! frequency plus the shape of the words decide who answers.
//!
//! Trust boundary: this module only ever *reads* the simulator (frequency,
//! weather, tail number) and returns text. It has no access to the tool
//! registry and can never actuate anything — ATC is the world, not the crew.

use std::sync::Mutex;
use std::time::Instant;

use crate::voice::StationSpeaker;
use crate::xplane::XPlaneClient;
use crate::{atis, datarefs, nav, tower};

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
        })
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
        *ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner()) = self.apt.runways.clone();
        *ATIS.lock().unwrap_or_else(|e| e.into_inner()) =
            self.apt.freq(nav::FREQ_ATIS).map(|f| {
                tracing::info!(mhz = f.khz as f64 / 1000.0, "ATIS broadcasting on frequency");
                AtisState {
                    khz: f.khz,
                    airport: self.apt.name.clone(),
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
        xp: &XPlaneClient,
    ) -> Option<(String, StationSpeaker)> {
        // Cheap word gates before any dataref read — the common case is a
        // cockpit command, and it must cost nothing. route_unkeyed re-applies
        // the same gates; only the trace differs per exit.
        if !is_radio_call(transcript, &self.apt.name.to_lowercase()) {
            return None;
        }
        let intent = tower::parse_intent(transcript);
        if intent == tower::Intent::Unknown {
            // Radio-shaped words that didn't parse — exactly the utterances
            // corpus mining wants to see after a flight.
            crate::blackbox::record(
                "atc",
                serde_json::json!({
                    "route": "unkeyed", "outcome": "fallthrough",
                    "transcript": transcript,
                }),
            );
            return None;
        }
        let Some(service) = self.tuned_service(xp).await else {
            crate::blackbox::record(
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
        crate::blackbox::record(
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
        if !is_radio_call(transcript, &self.apt.name.to_lowercase()) {
            return None;
        }
        self.note_information(transcript);
        let intent = tower::parse_intent(transcript);
        if intent == tower::Intent::Unknown {
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
        xp: &XPlaneClient,
    ) -> Option<(String, StationSpeaker)> {
        let Some(service) = self.tuned_service(xp).await else {
            crate::blackbox::record(
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
        crate::blackbox::record(
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
    pub async fn tuned_speaker(&self, xp: &XPlaneClient) -> Option<StationSpeaker> {
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
    async fn tuned_service(&self, xp: &XPlaneClient) -> Option<Service> {
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
                format!("{}, wind {}, QNH {}.", ctx.callsign, ctx.wind, ctx.qnh)
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
            | tower::Intent::RequestClearance => format!(
                "{}, contact {} Tower on {}.",
                ctx.callsign,
                ctx.airport,
                atis::spoken_mhz(self.tower_khz)
            ),
            tower::Intent::Unknown => {
                format!("{}, {} Departure, say again.", ctx.callsign, ctx.airport)
            }
        }
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
                format!("{}, wind {}, QNH {}.", ctx.callsign, ctx.wind, ctx.qnh)
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
            // Final reports and everything runway-shaped belong to the tower.
            tower::Intent::ReportFinal
            | tower::Intent::ReportDownwind
            | tower::Intent::ReportBase
            | tower::Intent::ReportCrosswind
            | tower::Intent::GoAround
            | tower::Intent::RequestTakeoff
            | tower::Intent::RequestTaxi
            | tower::Intent::RequestTaxiIn
            | tower::Intent::RequestClearance => format!(
                "{}, contact {} Tower on {}.",
                ctx.callsign,
                ctx.airport,
                atis::spoken_mhz(self.tower_khz)
            ),
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
        // Intercept altitude: 1,500 ft above the field, rounded to the hundred.
        let target_alt_ft = ((self.apt.elev_ft + 1500.0) / 100.0).round() * 100.0;
        let runway_spoken = atis::spoken_runway(&ident);
        *VECTORS.lock().unwrap_or_else(|e| e.into_inner()) = Some(VectorPlan {
            callsign: ctx.callsign.clone(),
            thr_lat,
            thr_lon,
            course_deg,
            runway_spoken: runway_spoken.clone(),
            tower_spoken: Some(atis::spoken_mhz(self.tower_khz)),
            target_alt_ft,
            last_tx: None,
            last_heading: None,
        });
        tracing::info!(runway = %ident, alt = target_alt_ft, "vectors armed");
        format!(
            "{}, {} Approach, radar contact. Expect vectors, visual approach runway {}. \
             Descend and maintain {}, QNH {}.",
            ctx.callsign,
            ctx.airport,
            runway_spoken,
            spoken_alt(target_alt_ft),
            ctx.qnh
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
    async fn build_ctx(&mut self, xp: &XPlaneClient) -> (tower::Ctx, Wx) {
        let wx = Wx {
            wind_from_deg: xp
                .get_float(datarefs::WIND_DIR_DEG_TRUE)
                .await
                .unwrap_or(0.0),
            wind_kts: xp.get_float(datarefs::WIND_SPEED_MSC).await.unwrap_or(0.0) * 1.943844,
        };
        let qnh = match xp.get_float(datarefs::QNH_PASCALS).await {
            Ok(pa) => atis::spoken_digits(&format!("{:.0}", pa / 100.0)),
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
        (self.make_ctx(wx, qnh, callsign, on_ground), wx)
    }

    /// The pure half of [`Station::build_ctx`] — everything after the sim
    /// reads, so tests phrase clearances from scripted weather.
    fn make_ctx(&mut self, wx: Wx, qnh: String, callsign: String, on_ground: bool) -> tower::Ctx {
        let runway = self.pick_runway(wx.wind_from_deg, wx.wind_kts);
        // "With information X" matching the current broadcast: the clearance
        // skips the QNH — having the numbers is the whole point of an ATIS.
        // A stale or absent letter keeps them (safety over brevity).
        let qnh = if self.pilot_info.is_some() && self.pilot_info == atis_current_letter() {
            String::new()
        } else {
            qnh
        };
        tower::Ctx {
            callsign,
            airport: self.apt.name.clone(),
            runway: atis::spoken_runway(&runway),
            qnh,
            wind: spoken_wind(wx.wind_from_deg, wx.wind_kts),
            holding_point: "Alpha".to_string(),
            approach: self.approach_khz.map(atis::spoken_mhz),
            on_ground,
        }
    }
}

/// Does an UNKEYED utterance sound like a radio call? Addressed to a station
/// by name, or built from call vocabulary a pilot only uses on the radio.
/// This is the content-routing gate: with the radio PTT keyed, keying itself
/// is the addressing and this is never consulted.
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

/// The tower's after-takeoff handoff, armed by a takeoff clearance and spoken
/// once airborne — see [`Station::arm_departure_handoff`] and [`handoff_tick`].
pub struct Handoff {
    /// The full transmission, prebuilt ("… contact Gibraltar Departure on …").
    pub line: String,
    /// Speak once climbing through this altitude (ft MSL, field + 700).
    pub min_alt_ft: f64,
}

pub static HANDOFF: Mutex<Option<Handoff>> = Mutex::new(None);

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

pub static ATIS: Mutex<Option<AtisState>> = Mutex::new(None);

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

pub static REGION: Mutex<Vec<RegionApt>> = Mutex::new(Vec::new());

/// The active station's identity + position, for the switch decision. Set by
/// [`Station::from_airport`].
static ACTIVE_FIELD: Mutex<Option<(String, f64, f64)>> = Mutex::new(None);

/// A station swap the main loop must consume (it owns the `Station`): the new
/// field's ICAO (for the voice casting) and airport, armed by [`region_tick`]
/// after the handoff line is spoken.
pub static PENDING_STATION: Mutex<Option<(String, nav::AirportInfo)>> = Mutex::new(None);

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
        if apt.freq(nav::FREQ_TOWER).is_some() {
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
    let (d_best, best) = region
        .iter()
        .map(|a| (crate::haversine_nm(lat, lon, a.lat, a.lon), a))
        .min_by(|x, y| x.0.total_cmp(&y.0))?;
    if best.apt.name == active_name {
        return None;
    }
    // Ground: only a relocation TO a field you're actually at. Airborne: the
    // enroute handoff, bounded so cruising over nowhere keeps the last
    // controller.
    if (on_ground && d_best > AT_FIELD_NM) || (!on_ground && d_best > SWITCH_MAX_NM) {
        return None;
    }
    let d_active = crate::haversine_nm(lat, lon, a_lat, a_lon);
    if d_best + SWITCH_MARGIN_NM > d_active {
        return None;
    }
    let khz = best.apt.freq(nav::FREQ_TOWER)?.khz;
    let sw = if on_ground {
        RegionSwitch {
            line: format!(
                "We're at {} — tower here is on {}.",
                best.apt.name,
                atis::spoken_mhz(khz)
            ),
            crew: true,
        }
    } else {
        RegionSwitch {
            line: format!(
                "Radar services terminated. Contact {} Tower on {}.",
                best.apt.name,
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

/// The active field's runway geometry, published by [`Station::install`] for
/// the runway-incursion watch.
static ACTIVE_RUNWAYS: Mutex<Vec<nav::Runway>> = Mutex::new(Vec::new());

/// Lateral capture around a runway centerline. apt.dat 100-rows carry a width
/// we don't parse yet; 40 m covers everything up to a wide-body runway
/// (60 m ÷ 2 + slack) without capturing a parallel taxiway.
const RUNWAY_HALF_WIDTH_M: f64 = 40.0;

/// Is this position ON one of the active field's runways? Point-to-segment in
/// a local flat-earth projection (exact enough over a 3 km runway). Returns
/// the spoken ident of the nearer end, for the blackbox and the call.
pub fn on_active_runway(lat: f64, lon: f64) -> Option<String> {
    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
    let m_per_deg_lat = 111_320.0;
    let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
    for r in runways.iter() {
        let (px, py) = ((lon - r.lon_a) * m_per_deg_lon, (lat - r.lat_a) * m_per_deg_lat);
        let (bx, by) = ((r.lon_b - r.lon_a) * m_per_deg_lon, (r.lat_b - r.lat_a) * m_per_deg_lat);
        let len2 = bx * bx + by * by;
        if len2 < 1.0 {
            continue; // degenerate row
        }
        let t = (px * bx + py * by) / len2;
        let cross_m = (px * by - py * bx).abs() / len2.sqrt();
        // Slightly past the thresholds still counts (displaced/overrun).
        if (-0.02..=1.02).contains(&t) && cross_m <= RUNWAY_HALF_WIDTH_M {
            let ident = if t < 0.5 { &r.ident_a } else { &r.ident_b };
            return Some(ident.clone());
        }
    }
    None
}

/// The nearest airborne traffic that could be landing on top of us: a TCAS
/// target inside 3 nm, LOW (15-450 m above own ship — an arrival on a 2-mile
/// final is ~100 m up; ground traffic and overflying cruisers are excluded).
/// Returns its range in nm. Same picture the advisories read; any source that
/// feeds the TCAS arrays works (LiveTraffic's real-world feed, AI aircraft).
pub async fn inbound_traffic_close(xp: &XPlaneClient) -> Option<f64> {
    let dist = xp
        .get_float_array("sim/cockpit2/tcas/indicators/relative_distance_mtrs")
        .await
        .ok()?;
    let alt = xp
        .get_float_array("sim/cockpit2/tcas/indicators/relative_altitude_mtrs")
        .await
        .ok()?;
    dist.iter()
        .zip(&alt)
        .skip(1) // slot 0 is own ship
        .filter(|(d, a)| **d > 1.0 && **d < 3.0 * 1852.0 && (15.0..450.0).contains(*a))
        .map(|(d, _)| d / 1852.0)
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
    last_tx: Option<Instant>,
    last_heading: Option<i64>,
}

pub static VECTORS: Mutex<Option<VectorPlan>> = Mutex::new(None);

/// How far out the vectors aim you onto final (the join point).
const JOIN_FINAL_NM: f64 = 6.0;

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

    let dist_thr = crate::haversine_nm(lat, lon, plan.thr_lat, plan.thr_lon);
    let brg_thr = crate::autoland::bearing_deg(lat, lon, plan.thr_lat, plan.thr_lon);
    // Established: on the approach side, on the extended centerline, inside the
    // join distance, AND actually flying the final course — the position wedge
    // alone would clear an aircraft crossing final at ninety degrees. A failed
    // track read doesn't block the clearance (best-effort, like every read).
    let off_course = angle_diff_deg(brg_thr, plan.course_deg).abs();
    let aligned = track_deg.is_none_or(|t| angle_diff_deg(t, plan.course_deg).abs() <= 30.0);
    if (1.0..=JOIN_FINAL_NM + 2.0).contains(&dist_thr) && off_course <= 15.0 && aligned {
        let handoff = plan
            .tower_spoken
            .as_ref()
            .map(|f| format!(" Contact Tower {f}."))
            .unwrap_or_default();
        let msg = format!(
            "{}, {} miles from touchdown. Cleared visual approach runway {}.{}",
            plan.callsign,
            dist_thr.round() as i64,
            plan.runway_spoken,
            handoff
        );
        *guard = None; // approach's job is done
        return Some(msg);
    }

    // Steering, two regimes. Near the extended centerline on the approach
    // side: pursue the centerline with a correction that shrinks with the
    // cross-track error — a capture curve. (Bare course here leaves any
    // parallel offset unconverged; the join point then falls behind and the
    // re-vector to it commands an orbit.) Anywhere else: direct to the join
    // point 6 nm out, which is the downwind/base leg for free.
    let along_nm = dist_thr * off_course.to_radians().cos(); // + = approach side
    let xtk_nm = dist_thr * angle_diff_deg(brg_thr, plan.course_deg).to_radians().sin();
    let desired = if along_nm >= 2.0 && xtk_nm.abs() <= 2.5 {
        (plan.course_deg + (xtk_nm * 20.0).clamp(-45.0, 45.0)).rem_euclid(360.0)
    } else {
        let recip = (plan.course_deg + 180.0).rem_euclid(360.0);
        let (join_lat, join_lon) =
            crate::autoland::project_latlon(plan.thr_lat, plan.thr_lon, recip, JOIN_FINAL_NM);
        crate::autoland::bearing_deg(lat, lon, join_lat, join_lon)
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
    // heading has genuinely moved — a small drift after 25 s, a big cut after
    // 8 s. Holding the assigned heading stays silent, like a real frequency.
    let elapsed = plan
        .last_tx
        .map(|t| now.duration_since(t).as_secs_f64())
        .unwrap_or(f64::MAX);
    let moved = plan
        .last_heading
        .map(|h| angle_diff_deg(h as f64, heading10 as f64).abs())
        .unwrap_or(360.0);
    let due = plan.last_tx.is_none()
        || (elapsed >= 25.0 && moved >= 10.0)
        || (elapsed >= 8.0 && moved >= 30.0);
    if !due {
        return None;
    }
    plan.last_tx = Some(now);
    plan.last_heading = Some(heading10);

    let mut msg = format!(
        "{}, fly heading {}",
        plan.callsign,
        atis::spoken_digits(&format!("{heading10:03}"))
    );
    if alt_ft > plan.target_alt_ft + 400.0 {
        msg.push_str(&format!(
            ", descend and maintain {}",
            spoken_alt(plan.target_alt_ft)
        ));
    }
    msg.push('.');
    Some(msg)
}

/// Factor traffic near the vectored arrival, spoken as a controller advisory
/// ("traffic, two o'clock, three miles, five hundred feet above"). The nearest
/// TCAS target inside 6 nm and ±1,500 ft; `None` = no factor, frequency stays
/// quiet. Reads the sim's TCAS arrays, which any traffic source (LiveTraffic,
/// default AI aircraft) populates — ATC narrates traffic, never controls it.
pub async fn traffic_advisory(xp: &XPlaneClient) -> Option<String> {
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
    let nm = d / 1852.0;
    let range = if nm < 1.0 { "under a mile".to_string() } else { format!("{nm:.0} miles") };
    let clock = match (((b + 15.0).rem_euclid(360.0)) / 30.0) as i32 {
        0 => 12,
        c => c,
    };
    let ft = ((a * 3.28084) / 100.0).round() * 100.0;
    let vert = if ft.abs() < 300.0 {
        "same altitude".to_string()
    } else if ft > 0.0 {
        format!("{ft:.0} feet above")
    } else {
        format!("{:.0} feet below", ft.abs())
    };
    Some(format!("traffic, {clock} o'clock, {range}, {vert}."))
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
            format!("{} has no published frequencies.", self.apt.name)
        } else {
            format!("{}: {}.", self.apt.name, parts.join(", "))
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
    pub async fn freq_briefing(&self, xp: &XPlaneClient) -> String {
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
                label: format!("{} {}", self.apt.name, word),
                khz: f.khz,
            },
            None => Tune::Missing(format!("{} has no {} frequency.", self.apt.name, word)),
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
        }
    }

    /// `ATIS` is a process-wide static, same discipline as `HANDOFF_LOCK`.
    static ATIS_LOCK: Mutex<()> = Mutex::new(());

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
        let _guard = ATIS_LOCK.lock().unwrap_or_else(|e| e.into_inner());
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

    /// `REGION`/`ACTIVE_FIELD`/`PENDING_STATION` are process statics — one
    /// lock for every region test.
    static REGION_LOCK: Mutex<()> = Mutex::new(());

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
    fn region_hands_off_to_the_decisively_nearer_tower() {
        let _guard = REGION_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // Gibraltar-ish and Malaga-ish, ~50 nm apart along a degree of longitude.
        *REGION.lock().unwrap() = vec![
            region_apt("Gibraltar", 36.15, -5.35),
            region_apt("Malaga", 36.67, -4.50),
        ];
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.15, -5.35));
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
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
        // Decisively closer to Malaga, airborne: the controller handoff, once.
        let sw = region_tick(36.6, -4.6, false, t0).expect("handoff");
        assert!(!sw.crew, "airborne switch is the controller's");
        assert!(sw.line.contains("Contact Malaga Tower"), "{}", sw.line);
        assert!(sw.line.contains("one one eight decimal one"), "{}", sw.line);
        assert!(PENDING_STATION.lock().unwrap().is_some(), "swap armed for the main loop");
        // While the swap waits to be consumed: silence, no re-arm.
        assert!(region_tick(36.6, -4.6, false, t0).is_none());
        // Ground relocation: parked AT Malaga with Gibraltar still active
        // (the "started at LERL, Gibraltar answered" case) — the CREW notes
        // the local tower, no radar theatre.
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        let sw = region_tick(36.67, -4.50, true, t0).expect("ground relocation");
        assert!(sw.crew, "ground switch is the crew's note");
        assert!(sw.line.contains("We're at Malaga"), "{}", sw.line);
        *REGION.lock().unwrap() = Vec::new();
        *ACTIVE_FIELD.lock().unwrap() = None;
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
    }

    #[test]
    fn runway_occupancy_is_a_tight_rectangle() {
        let _guard = REGION_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // LXGB 09/27, real-ish geometry: ~1.7 km east-west at 36.151N.
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
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
        let _guard = ATIS_LOCK.lock().unwrap_or_else(|e| e.into_inner());
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
            st.make_ctx(w, "one zero one eight".into(), "November".into(), true)
        };
        let reply = st.twr.handle(tower::parse_intent("request taxi"), &ctx);
        assert!(!reply.contains("QNH"), "{reply}");
        // A STALE letter keeps the numbers — safety over brevity.
        let mut st = lxgb();
        st.note_information("with information zulu, request taxi");
        let ctx = st.make_ctx(w, "one zero one eight".into(), "November".into(), true);
        let reply = st.twr.handle(tower::parse_intent("request taxi"), &ctx);
        assert!(reply.contains("QNH one zero one eight"), "{reply}");
        // No letter at all: numbers as always.
        let mut st = lxgb();
        let ctx = st.make_ctx(w, "one zero one eight".into(), "November".into(), true);
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

    /// `HANDOFF` is a process-wide static: every test that arms or drains it
    /// holds this lock, or parallel tests clobber each other's plan.
    static HANDOFF_LOCK: Mutex<()> = Mutex::new(());

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
            "one zero one seven".into(),
            "November one seven two Sierra Papa".into(),
            false, // airborne default; only the mayday answer reads it
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
        let _guard = HANDOFF_LOCK.lock().unwrap_or_else(|e| e.into_inner());
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
            "hello tower",
            "gibraltar is somewhere ahead",
        ] {
            assert!(
                st.route_unkeyed(crew, Service::Tower, &ctx, wx).is_none(),
                "crew talk routed to ATC: {crew}"
            );
        }
        assert_eq!(st.twr.state, tower::State::Parked, "crew talk moved the sequence");
        let r = st
            .route_unkeyed("gibraltar tower, request taxi", Service::Tower, &ctx, wx)
            .expect("a proper call routes");
        assert!(r.contains("holding point"), "{r}");

        // Any leftover handoff from this sortie must not leak into the next
        // test holding the lock.
        *HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
    }

    /// The world sweep: EVERY towered airport in the real Global Airports
    /// file stands up a station, briefs its frequencies, and flies the
    /// departure happy path — the shakeout for apt.dat shapes LXGB never
    /// showed us. Opt-in because the file ships with X-Plane, not the repo:
    ///
    ///   cargo test --release --bin x-copilot -- --ignored atc_world_sweep
    ///
    /// Override the file with XCOPILOT_APT_DAT.
    #[test]
    #[ignore = "needs X-Plane's Global Airports apt.dat; run with -- --ignored"]
    fn atc_world_sweep_every_towered_field_stands_up() {
        let _guard = HANDOFF_LOCK.lock().unwrap_or_else(|e| e.into_inner());
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
        let _guard = HANDOFF_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let path = lxgb_apt_file();
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        assert_eq!(st.departure_khz, Some(130400), "departure row wired");
        let ctx = tower::Ctx {
            callsign: "November one seven two Sierra Papa".into(),
            airport: "Gibraltar".into(),
            runway: "two seven".into(),
            qnh: "one zero one five".into(),
            wind: "calm".into(),
            holding_point: "Alpha".into(),
            approach: Some(atis::spoken_mhz(122800)),
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
            last_tx: None,
            last_heading: None,
        });

        // Regression: crossing final perpendicular inside the wedge must NOT
        // be "established" — position said cleared, the track says crossing.
        let t0 = std::time::Instant::now();
        let (x_lat, x_lon) = crate::autoland::project_latlon(thr_lat, thr_lon, course + 180.0, 4.0);
        let crossing = vector_tick(x_lat, x_lon, 1600.0, Some(0.0), false, t0);
        if let Some(msg) = &crossing {
            assert!(!msg.contains("Cleared"), "cleared while crossing final: {msg}");
        }

        // Now the real arrival: 10 nm north of the field at 3,000 ft. The sim
        // obeys each assigned heading at 100 kts in 10 s ticks, descending to
        // the assigned altitude — a pilot doing exactly as told, nothing more.
        VECTORS.lock().unwrap().as_mut().unwrap().last_tx = None;
        VECTORS.lock().unwrap().as_mut().unwrap().last_heading = None;
        let (mut lat, mut lon) = crate::autoland::project_latlon(thr_lat, thr_lon, 0.0, 10.0);
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
            let (nlat, nlon) = crate::autoland::project_latlon(lat, lon, hdg, 100.0 * 10.0 / 3600.0);
            (lat, lon) = (nlat, nlon);
            alt = (alt - 80.0).max(target_alt_ft);
        }

        let first = first.expect("an opening vector");
        assert!(first.contains("fly heading"), "{first}");
        assert!(
            first.contains("descend and maintain one thousand five hundred"),
            "{first}"
        );
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
                qnh: "one zero one seven".into(),
                wind: "calm".into(),
                holding_point: "Alpha".into(),
                approach: Some(atis::spoken_mhz(122800)),
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
            last_tx: None,
            last_heading: None,
        });
        assert!(vector_tick(thr_lat, thr_lon, 15.0, None, true, t0).is_none());
        assert!(VECTORS.lock().unwrap().is_none(), "on-ground tick clears the plan");
    }
}
