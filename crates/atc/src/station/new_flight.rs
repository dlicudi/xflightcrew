//! A new flight's station: retiring the old one, arming the new, reach.

use super::*;

/// A new flight has been set up and the station that knew the old one has
/// not yet been replaced by the main loop ([`retire_for_new_flight`] sets it,
/// `Station::install` and `Station::uninstall` clear it). While it stands the
/// old station's replies publish no clearance picture.
pub(super) static NEW_FLIGHT_PENDING: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);

pub(crate) fn new_flight_pending() -> bool {
    NEW_FLIGHT_PENDING.load(std::sync::atomic::Ordering::SeqCst)
}

/// The reply path's one write of the clearance picture.
pub(super) fn publish_clearance(snap: ClearanceSnap) {
    *OWN_STRIP.clearance.lock().unwrap_or_else(|e| e.into_inner()) = Some(snap);
}

/// A NEW FLIGHT at `icao` (the crew's "put us on a ten mile final", which
/// starts a flight through the sim's own initialisation): the station for that
/// field is rebuilt from nothing on the main loop's next pass, even when it is
/// the one already up. The tower's worksheet for us belongs to the flight that
/// ended: at LEMG (2026-09-26) the station that had watched the 737 sit on
/// runway 30 went on as if the arrival 10 nm out on 13 were the same aircraft,
/// and the landing earned "you landed without clearance". Overrides any swap
/// already waiting (the new flight wins), starts the region cooldown so the
/// nearest-field pick cannot move the station straight back, and drops the
/// old flight's arrival pin and region handoff (install() keeps both on
/// purpose, for a handoff in flight). Returns the tower's frequency, `None`
/// when the field has no staffed tower in the region index (nothing armed).
pub fn arm_new_flight(icao: &str, now: Instant) -> Option<i64> {
    let (code, apt, khz) = {
        let region = REGION.lock().unwrap_or_else(|e| e.into_inner());
        let a = region.iter().find(|a| a.icao.eq_ignore_ascii_case(icao))?;
        let khz = a.apt.freq(nav::FREQ_TOWER)?.khz;
        (a.icao.clone(), a.apt.clone(), khz)
    };
    tracing::info!(%code, khz, "station rebuilt for a new flight");
    *PENDING_STATION.lock().unwrap_or_else(|e| e.into_inner()) = Some((code, apt));
    *LAST_REGION_SWITCH.lock().unwrap_or_else(|e| e.into_inner()) = Some(now);
    *ARRIVAL_PIN.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *REGION_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
    Some(khz)
}

/// A new flight airborne is worked by the nearest towered field within this
/// (a setup's final is at most 30 nm out).
pub(super) const NEW_FLIGHT_AIR_NM: f64 = 30.0;

/// The towered field a new flight at this position belongs to. On the
/// ground: the field whose pavement is under the wheels (the region's own
/// ground rule, nearest runway footprint within [`AT_FIELD_NM`]). Airborne:
/// the nearest within [`NEW_FLIGHT_AIR_NM`]. `None`: no staffed tower here
/// (an uncontrolled field, or nowhere). Pure over the index.
pub(super) fn field_under_in(region: &[RegionApt], lat: f64, lon: f64, on_ground: bool) -> Option<&RegionApt> {
    let reach = if on_ground { AT_FIELD_NM } else { NEW_FLIGHT_AIR_NM };
    let dist = |a: &RegionApt| {
        let direct = crate::nav::haversine_nm(lat, lon, a.lat, a.lon);
        if on_ground {
            a.apt.ground_distance_nm(lat, lon).unwrap_or(direct)
        } else {
            direct
        }
    };
    region
        .iter()
        .filter(|a| a.apt.freq(nav::FREQ_TOWER).is_some())
        .map(|a| (dist(a), a))
        .filter(|(d, _)| *d <= reach)
        .min_by(|x, y| x.0.total_cmp(&y.0))
        .map(|(_, a)| a)
}

/// The towered field under the aircraft, as (ICAO, name); see
/// [`field_under_in`].
pub fn field_under(lat: f64, lon: f64, on_ground: bool) -> Option<(String, String)> {
    let region = REGION.lock().unwrap_or_else(|e| e.into_inner());
    field_under_in(&region, lat, lon, on_ground).map(|a| (a.icao.clone(), a.apt.name.clone()))
}

/// A NEW FLIGHT wherever the sim put us: the station for the field under
/// the aircraft's NEW position is rebuilt ([`arm_new_flight`]), never the
/// field the station happened to be working. EGKK, 2026-09-28: the runner
/// moved the 737 from LEMG to Gatwick stand 71, the new-flight watch re-armed
/// the ACTIVE field (Malaga) and its cooldown held the region's relocation
/// off for 180 s, so full auto departed Gatwick as an uncontrolled field and
/// looked its stand up at LEMG (#151). Returns the field and its tower;
/// `None` when no staffed tower is under the position (nothing armed: the
/// old station, retired and out of reach, stays silent).
pub fn arm_new_flight_at(lat: f64, lon: f64, on_ground: bool, now: Instant) -> Option<(String, i64)> {
    let (icao, _) = field_under(lat, lon, on_ground)?;
    let khz = arm_new_flight(&icao, now)?;
    Some((icao, khz))
}

/// A NEW FLIGHT started AIRBORNE on its plan (the sortie runner's enroute
/// setup): an IFR flight mid-route, already identified, never a departure
/// and never a field's circuit traffic.
#[derive(Debug, Clone, PartialEq)]
pub struct EnrouteStation {
    /// The field whose radar unit works the flight, and its tower (the
    /// station built for it).
    pub icao: String,
    pub tower_khz: i64,
    /// "Approach" or "Departure", and its frequency: where COM1 goes.
    pub position: &'static str,
    pub khz: i64,
    /// The flight's discrete code, kept unit to unit (`OWN_STRIP.ifr_flight`).
    pub code: i64,
}

/// The radar unit an airborne IFR flight at this position is worked by: the
/// region's fields that HAVE one (approach or departure; a tower-only field
/// is passed over, as the enroute handoff passes it), service-weighted like
/// the airborne region pick, within [`SWITCH_MAX_NM`]. Pure over the index.
pub(super) fn enroute_field_in(region: &[RegionApt], lat: f64, lon: f64) -> Option<&RegionApt> {
    region
        .iter()
        .filter(|a| radar_position(&a.apt).is_some() && a.apt.freq(nav::FREQ_TOWER).is_some())
        .map(|a| (crate::nav::haversine_nm(lat, lon, a.lat, a.lon), a))
        .filter(|(d, _)| *d <= SWITCH_MAX_NM)
        .map(|(d, a)| (d - SERVICE_BIAS_NM * region_services(&a.apt) as f64, a))
        .min_by(|x, y| x.0.total_cmp(&y.0))
        .map(|(_, a)| a)
}

/// A NEW FLIGHT put in the air on its plan, mid-route: the world holds it
/// as an IFR flight already under radar service. Its code
/// ([`discrete_squawk`] of the callsign) is published as `OWN_STRIP.ifr_flight`,
/// airborne, so every region handoff after this keeps it and goes to a
/// radar position; the station for the nearest field with a radar unit is
/// rebuilt ([`arm_new_flight`]), and its approach (else departure)
/// frequency is the one to tune. No clearance, no departure handoff, no
/// radar watch: the flight is picked up identified, as a real one handed
/// over mid-route would be. Called after [`retire_for_new_flight`]. `None`:
/// no radar unit within reach (nothing armed, the code still published).
pub fn arm_new_flight_enroute(
    lat: f64,
    lon: f64,
    callsign: &str,
    now: Instant,
) -> Option<EnrouteStation> {
    let code = discrete_squawk(callsign);
    *OWN_STRIP.ifr_flight.lock().unwrap_or_else(|e| e.into_inner()) = Some(IfrFlight {
        code,
        callsign: callsign.to_string(),
        airborne: true,
    });
    let (icao, position, khz) = {
        let region = REGION.lock().unwrap_or_else(|e| e.into_inner());
        let a = enroute_field_in(&region, lat, lon)?;
        let (position, khz) = radar_position(&a.apt)?;
        (a.icao.clone(), position, khz)
    };
    let tower_khz = arm_new_flight(&icao, now)?;
    Some(EnrouteStation { icao, tower_khz, position, khz, code })
}

/// Whether the installed station is the departure field's, judged at a
/// mission's arm.
#[derive(Debug, Clone, PartialEq)]
pub enum DepartureStation {
    /// The station up is the field's (or no towered field is under us: an
    /// uncontrolled departure).
    Ready,
    /// The field's station is armed and the main loop has not installed it.
    Waiting { want: String },
    /// The station up is another field's (or the last flight's worksheet)
    /// and nothing is armed for this one.
    Arm { want: String },
}

/// Judge the station against the field under the aircraft. `active`: the
/// installed station's field; `under`: the towered field under the wheels;
/// `pending`: a swap armed and not yet installed; `new_flight_pending`: the
/// station up still holds the last flight's worksheet. Pure.
pub fn judge_departure_station(
    active: Option<&str>,
    under: Option<&str>,
    pending: Option<&str>,
    new_flight_pending: bool,
) -> DepartureStation {
    let Some(want) = under else {
        return DepartureStation::Ready;
    };
    let same = |x: Option<&str>| x.is_some_and(|x| x.eq_ignore_ascii_case(want));
    if same(pending) {
        return DepartureStation::Waiting { want: want.to_string() };
    }
    if same(active) && !new_flight_pending && pending.is_none() {
        return DepartureStation::Ready;
    }
    DepartureStation::Arm { want: want.to_string() }
}

/// The field of the swap armed and not yet installed.
pub(super) fn pending_station_icao() -> Option<String> {
    PENDING_STATION
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|(icao, _)| icao.clone())
}

/// The departure station at the aircraft's position, on the ground: judged
/// by [`judge_departure_station`], and the field's station armed when `arm`
/// and the verdict asks for it. Returns the verdict and the installed
/// station's field.
pub fn departure_station(
    lat: f64,
    lon: f64,
    arm: bool,
    now: Instant,
) -> (DepartureStation, Option<String>) {
    let under = field_under(lat, lon, true).map(|(icao, _)| icao);
    let active = active_icao().map(|(icao, _, _)| icao);
    let pending = pending_station_icao();
    let verdict = judge_departure_station(
        active.as_deref(),
        under.as_deref(),
        pending.as_deref(),
        new_flight_pending(),
    );
    if let (DepartureStation::Arm { want }, true) = (&verdict, arm) {
        let _ = arm_new_flight(want, now);
    }
    (verdict, active)
}

/// The radio row's content: the ACTIVE station's name and services — but only
/// while it's inside VHF range. A tower you can't physically raise isn't a
/// radio row; parked in Extremadura the panel showed Gibraltar's comm plan as
/// if it could answer (2026-07-10). `None` = no station, or out of range.
pub fn active_station_in_range(
    lat: f64,
    lon: f64,
    alt_msl_ft: f64,
) -> Option<(String, Vec<(String, i64)>)> {
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
/// On the ground this far from the station's field, the station is out of
/// reach: a tower clears the aircraft on its own field, not one ten miles
/// away (the ground's line of sight is 8.7 nm, `vhf_range_nm(0.0)`).
/// Airborne the station keeps the flight ("enroute over nowhere keeps the
/// last controller", [`SWITCH_MAX_NM`]).
pub const GROUND_REACH_NM: f64 = 10.0;

/// The active station cannot hear or be heard: set per monitor tick by
/// [`note_aircraft`] from the aircraft's position.
pub(super) static OUT_OF_REACH: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

pub(crate) fn out_of_reach() -> bool {
    OUT_OF_REACH.load(std::sync::atomic::Ordering::Relaxed)
}

pub(crate) fn station_unavailable() -> bool {
    out_of_reach() || uncontrolled_arrival()
}

/// The world's view of where the aircraft is, once per monitor tick. On the
/// ground beyond [`GROUND_REACH_NM`] of the station's field the station goes
/// silent, and what it had issued is retired: a clearance from a field we
/// are not at is nobody's (the crew then departs as from an uncontrolled
/// field). A route the crew planned for itself is left alone.
pub fn note_aircraft(lat: f64, lon: f64, on_ground: bool) {
    use std::sync::atomic::Ordering::Relaxed;
    // Where we are, for the go-around judgement's time to the threshold.
    note_own_fix(lat, lon, on_ground, Instant::now());
    // The crew's own route and runway end at the liftoff: a departure's
    // runway must never read as a cleared one at the destination (autoland
    // takes the cleared runway).
    if !on_ground {
        let mut own = OWN_STRIP.self_taxi.lock().unwrap_or_else(|e| e.into_inner());
        if own.take().is_some() {
            *OWN_STRIP.taxi_route.lock().unwrap_or_else(|e| e.into_inner()) = None;
        }
    }
    // An IFR flight ends at its landing: airborne since the clearance, the
    // next ground contact retires it (never a region switch).
    {
        let mut ifr = OWN_STRIP.ifr_flight.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(f) = ifr.as_mut() {
            if !on_ground {
                f.airborne = true;
            } else if f.airborne {
                sim::blackbox::record(
                    "atc",
                    serde_json::json!({ "event": "ifr_flight_ended", "code": f.code, "why": "landed" }),
                );
                *ifr = None;
            }
        }
    }
    let far = on_ground
        && active_station_range_nm(lat, lon).is_some_and(|d| d > GROUND_REACH_NM);
    let was = OUT_OF_REACH.swap(far, Relaxed);
    if far && !was {
        let station = ACTIVE_FIELD
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .map(|f| f.0.clone());
        tracing::info!(?station, "ATC station out of reach on the ground: standing it down here");
        sim::blackbox::record("atc", serde_json::json!({ "route": "reach", "station": station, "out": true }));
        *OWN_STRIP.clearance.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *OWN_STRIP.assigned_squawk.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *OWN_STRIP.ifr_clearance.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *OWN_STRIP.ifr_flight.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *OWN_STRIP.takeoff_hold.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *OWN_STRIP.line_up_condition.lock().unwrap_or_else(|e| e.into_inner()) = None;
        if OWN_STRIP.self_taxi.lock().unwrap_or_else(|e| e.into_inner()).is_none() {
            *OWN_STRIP.taxi_route.lock().unwrap_or_else(|e| e.into_inner()) = None;
            *OWN_STRIP.taxi_to_stand.lock().unwrap_or_else(|e| e.into_inner()) = false;
        }
    } else if was && !far {
        sim::blackbox::record("atc", serde_json::json!({ "route": "reach", "out": false }));
    }
}
