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
        let Some(twr_row) = apt.freq(nav::FREQ_TOWER) else {
            tracing::warn!(%icao, "ATC station: no tower frequency in apt.dat; ATC off");
            return None;
        };
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
        })
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
        if !is_radio_call(transcript, &self.apt.name.to_lowercase()) {
            return None;
        }
        let intent = tower::parse_intent(transcript);
        if intent == tower::Intent::Unknown {
            return None;
        }
        let service = self.tuned_service(xp).await?;
        Some((self.respond(service, intent, xp).await, service.speaker()))
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
        let service = self.tuned_service(xp).await?;
        let intent = tower::parse_intent(transcript);
        // A pilot readback earns SILENCE, like a real frequency: the
        // controller only speaks up to correct a wrong one. Empty string =
        // "heard, no reply due" (distinct from None = nobody listening).
        // Exception: the first call to DEPARTURE is a check-in ("passing one
        // thousand five hundred climbing"), and a check-in gets radar contact.
        if intent == tower::Intent::Unknown && tower::is_readback(transcript) {
            if service == Service::Departure && !self.dep_checked_in {
                self.dep_checked_in = true;
                let ctx = self.build_ctx(xp).await;
                return Some((
                    format!("{}, {} Departure, radar contact.", ctx.callsign, ctx.airport),
                    service.speaker(),
                ));
            }
            return Some((String::new(), service.speaker()));
        }
        Some((self.respond(service, intent, xp).await, service.speaker()))
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

    /// Answer one transmission on whichever service is tuned.
    async fn respond(
        &mut self,
        service: Service,
        intent: tower::Intent,
        xp: &XPlaneClient,
    ) -> String {
        let ctx = self.build_ctx(xp).await;
        match service {
            Service::Tower => {
                let was = self.twr.state;
                let reply = self.twr.handle(intent, &ctx);
                // A fresh takeoff clearance arms the after-departure handoff:
                // once airborne and climbing, the tower sends the flight to
                // departure (or approach) — the frequency chain, not a tower
                // that owns you forever.
                if self.twr.state == tower::State::TakeoffCleared
                    && was != tower::State::TakeoffCleared
                {
                    self.arm_departure_handoff(&ctx);
                }
                reply
            }
            Service::Approach => self.respond_approach(intent, &ctx, xp).await,
            Service::Departure => self.respond_departure(intent, &ctx, xp).await,
        }
    }

    /// The departure controller: takes the handoff after takeoff, then sends
    /// arrivals to approach (vectors) and everything runway-shaped back to the
    /// tower.
    async fn respond_departure(
        &mut self,
        intent: tower::Intent,
        ctx: &tower::Ctx,
        xp: &XPlaneClient,
    ) -> String {
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
                None => self.arm_vectors(ctx, xp).await,
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
                    self.arm_vectors(ctx, xp).await
                }
            },
            // Runway-shaped business belongs to the tower.
            tower::Intent::ReportFinal
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
    async fn respond_approach(
        &mut self,
        intent: tower::Intent,
        ctx: &tower::Ctx,
        xp: &XPlaneClient,
    ) -> String {
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
            tower::Intent::RequestVectors => self.arm_vectors(ctx, xp).await,
            tower::Intent::RequestLanding { touch_and_go } => {
                // The option request rides through the vectors: the tower's
                // eventual clearance must say "touch and go", not "land".
                self.twr.expect_touch_and_go(touch_and_go);
                self.arm_vectors(ctx, xp).await
            }
            // Final reports and everything runway-shaped belong to the tower.
            tower::Intent::ReportFinal
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
    async fn arm_vectors(&mut self, ctx: &tower::Ctx, xp: &XPlaneClient) -> String {
        // The runway END in use: its threshold is the aim point, its heading
        // the final approach course. Same held pick as every clearance, so
        // vectors and the tower always agree on the runway.
        let wind_from = xp
            .get_float(datarefs::WIND_DIR_DEG_TRUE)
            .await
            .unwrap_or(0.0);
        let wind_kts = xp.get_float(datarefs::WIND_SPEED_MSC).await.unwrap_or(0.0) * 1.943844;
        let ident = self.pick_runway(wind_from, wind_kts);
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

    /// Build the live context for a reply. Best-effort: a failed read falls
    /// back to silence on that item rather than a wrong number.
    async fn build_ctx(&mut self, xp: &XPlaneClient) -> tower::Ctx {
        let wind_from = xp
            .get_float(datarefs::WIND_DIR_DEG_TRUE)
            .await
            .unwrap_or(0.0);
        let wind_kts = xp.get_float(datarefs::WIND_SPEED_MSC).await.unwrap_or(0.0) * 1.943844;
        let qnh = match xp.get_float(datarefs::QNH_PASCALS).await {
            Ok(pa) => atis::spoken_digits(&format!("{:.0}", pa / 100.0)),
            Err(_) => String::new(),
        };
        let runway = self.pick_runway(wind_from, wind_kts);
        let callsign = xp
            .get_string(datarefs::TAILNUM)
            .await
            .ok()
            .map(|t| tower::spoken_callsign(&t))
            .filter(|s| !s.is_empty())
            .unwrap_or_else(|| "aircraft calling".to_string());
        tower::Ctx {
            callsign,
            airport: self.apt.name.clone(),
            runway: atis::spoken_runway(&runway),
            qnh,
            wind: spoken_wind(wind_from, wind_kts),
            holding_point: "Alpha".to_string(),
            approach: self.approach_khz.map(atis::spoken_mhz),
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
            // sit 5 kHz off the carrier.
            let tuned = SERVICES.iter().find_map(|(word, kind)| {
                self.apt
                    .freq(*kind)
                    .filter(|f| (f.khz - com1).abs() <= 5)
                    .map(|_| *word)
            });
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
            // "change frequency to the tower" — but only when the sentence is
            // about the radio: a bare "change" is too promiscuous a verb.
            || (t.contains("change")
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
        }
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
    fn takeoff_clearance_arms_the_departure_handoff() {
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
        // "change" without radio context must not retune.
        assert!(st.tune_request("change approach speed to seventy").is_none());
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
