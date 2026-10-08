//! The radio coach, suggested calls, the field and its surface wind.

use super::*;

/// The runway end a DEPARTING ghost rolled from: nearest strip, end chosen
/// by the roll direction (bearing from last tick's position to this one).
/// `None` when the ghost hasn't moved enough for a bearing — the caller
/// falls back to the nearest end.
pub(super) fn departure_runway_ident(lat: f64, lon: f64, prev_lat: f64, prev_lon: f64) -> Option<String> {
    let m_per_deg_lat = 111_320.0;
    let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
    let (dx, dy) = (
        (lon - prev_lon) * m_per_deg_lon,
        (lat - prev_lat) * m_per_deg_lat,
    );
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
    let da = sim::angles::abs_diff_deg(track, r.heading_a_deg());
    let db = sim::angles::abs_diff_deg(track, r.heading_b_deg());
    Some(if da <= db {
        r.ident_a.clone()
    } else {
        r.ident_b.clone()
    })
}

/// The runway a ghost's end is narrated as: a CLOSE parallel's is the main
/// runway's (`nav::close_parallel_main`), since replayed positions wander
/// between centrelines 200 m apart and the field works one of them.
pub(super) fn main_runway_of(ident: &str) -> String {
    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
    nav::close_parallel_main(&runways, ident)
}

/// The active field's runway end nearest a position — the runway a landing
/// ghost is aiming at, for the narrated clearance's flavor.
pub(super) fn nearest_runway_ident(lat: f64, lon: f64) -> Option<String> {
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
            && (t.contains("call")
                || t.contains("ask")
                || t.contains("request")
                || t.contains("talk")))
        || (t.contains("help") && t.contains("radio"))
        || coach_yes_no(t)
}

/// "Should we check in with ATC?", "do we need to call the tower?", "is it
/// time to contact departure?": a yes/no about whether a call is owed. The
/// coach answers it (yes with the call, or no) rather than the copilot
/// staying silent (missed live, 2026-09-12).
pub fn coach_yes_no(t: &str) -> bool {
    let asks = t.starts_with("should we")
        || t.starts_with("should i")
        || t.starts_with("shall we")
        || t.starts_with("shall i")
        || t.contains("do we need to")
        || t.contains("do i need to")
        || t.contains("do we have to")
        || t.contains("is it time to")
        || t.contains("time to check")
        || t.contains("time we ")
        || t.contains("we owe")
        || t.contains("owe them")
        || t.contains("owe a call");
    let radio = t.contains("check in")
        || t.contains("checkin")
        || t.contains("call ")
        || t.ends_with("call")
        || t.contains("contact")
        || t.contains("talk to")
        || t.contains("report")
        || t.contains("radio")
        || t.contains("atc")
        || t.contains("tower")
        || t.contains("ground")
        || t.contains("departure")
        || t.contains("approach")
        || t.contains("center")
        || t.contains("centre")
        || t.contains("frequency");
    asks && radio
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
    let clr = OWN_STRIP.clearance
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|c| (c.state, c.runway_ident.clone(), c.pending_readback.clone()));
    let state = clr.as_ref().map(|(s, _, _)| *s);
    let arrival = !on_ground
        || matches!(
            state,
            Some(tower::State::Inbound | tower::State::ClearedToLand)
        );
    // A readback owed outranks everything: it IS the next call — and it was
    // just received, so the radio is by definition on the right frequency.
    if let Some(rb) = clr.as_ref().and_then(|(_, _, rb)| rb.clone()) {
        return Some(Suggestion {
            first: None,
            call: Some(rb),
            urgent: true,
            arrival,
        });
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
        if let Some(rwy) = OWN_STRIP.landing_handoff
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
        {
            return Some(Suggestion {
                first: tune_first("tower"),
                call: Some(format!("{station} Tower, {callsign}, final runway {rwy}.")),
                urgent: false,
                arrival: true,
            });
        }
        if let Some((ils_cleared, rwy)) = OWN_STRIP.vectors
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
            let hp = OWN_STRIP.taxi_route
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
    Some(Suggestion {
        first,
        call: Some(call),
        urgent: false,
        arrival,
    })
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

/// A readback of the region's handoff, made on the frequency it came on, is
/// heard by the controller who gave it even though that station has already
/// been swapped for the next: it is not a call into the void. Six flights out
/// of six logged "radio call into the VOID" for "squawking seven thousand,
/// Carlisle Tower on ..." and the debrief listed each as a lost call.
pub(super) const HANDOFF_READBACK_WINDOW_SECS: u64 = 45;

pub(super) fn void_outcome(transcript: &str, now: Instant) -> &'static str {
    let recent = LAST_REGION_SWITCH
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .is_some_and(|t| now.saturating_duration_since(t).as_secs() <= HANDOFF_READBACK_WINDOW_SECS);
    if recent && tower::is_readback(transcript) {
        "handoff_readback"
    } else {
        "void"
    }
}

/// The active field's SURFACE wind as the app last read it from the sim's
/// real-weather METAR file: (field lat, lon, from deg true, kts). The station
/// cannot read weather files (its sim handle is the read-only dataref view),
/// so the app hands the report down, the way it hands the ATIS its weather.
/// Keyed by POSITION: a station knows where it is, not what it is called.
pub(super) static SURFACE_WIND: Mutex<Option<(f64, f64, f64, f64)>> = Mutex::new(None);

/// A published wind is this field's when it was read for a point this near.
pub(super) const SURFACE_WIND_FIELD_NM: f64 = 3.0;

/// Publish (or retire, with `None`) the surface wind of the field at
/// `lat`/`lon`.
pub fn set_surface_wind(lat: f64, lon: f64, wind: Option<(f64, f64)>) {
    *SURFACE_WIND.lock().unwrap_or_else(|e| e.into_inner()) =
        wind.map(|(from, kts)| (lat, lon, from, kts));
}

/// The published surface wind, only if it is THIS field's.
pub(super) fn surface_wind_at(pos: Option<(f64, f64)>) -> Option<(f64, f64)> {
    let (lat, lon) = pos?;
    SURFACE_WIND
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .filter(|(wlat, wlon, _, _)| nav::haversine_nm(lat, lon, *wlat, *wlon) <= SURFACE_WIND_FIELD_NM)
        .map(|(_, _, from, kts)| (*from, *kts))
}

/// The active station's ICAO and position, from the region index (the
/// station itself carries a name, not an identifier): for the app's METAR
/// lookup. `None` before the index has loaded or with no station up.
pub fn active_icao() -> Option<(String, f64, f64)> {
    let (_, lat, lon) = ACTIVE_FIELD
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()?;
    REGION
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .iter()
        .find(|a| nav::haversine_nm(lat, lon, a.lat, a.lon) < 0.5)
        .map(|a| (a.icao.clone(), lat, lon))
}
