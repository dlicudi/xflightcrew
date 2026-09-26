//! The party line: the one page of state the world side (ATC — station.rs,
//! tower.rs) publishes for the crew side (copilot — main.rs, the monitor's
//! crew watches, taxi_guide.rs) to read.
//!
//! Boundary rules, both directions:
//!
//! - The WORLD writes these from its reply path and watch ticks. It never
//!   reads crew state and never actuates the aircraft — its only outputs
//!   are spoken text and this page.
//! - The CREW reads through the accessor functions below, never the statics,
//!   and never writes — with two deliberate exceptions, the `take_*`
//!   consumers, where the main loop drains a queue the world armed for it
//!   (a station swap, a released takeoff clearance).
//!
//! Everything else in station.rs is private to the world side. When the
//! crew needs a new fact, it gets a new accessor here — so "what can the
//! copilot see of ATC?" stays answerable on one page. Until the sides live
//! in separate crates this is convention, not compiler-checked; keep the
//! surface small enough to audit by eye.

use std::sync::{Mutex, MutexGuard};

use crate::nav;
use crate::station::{ClearanceSnap, TakeoffHold, TaxiWatch, VectorPlan};
use crate::tower;

// ---- published state (world-side writes live in station.rs) ----

/// The standing clearance: FSM state, callsign, runway, the readback owed.
pub(crate) static CLEARANCE: Mutex<Option<ClearanceSnap>> = Mutex::new(None);

/// The taxi route the pilot is CLEARED on — feeds the compliance watch
/// (world) and the progressive taxi guide (crew).
pub(crate) static TAXI_ROUTE: Mutex<Option<TaxiWatch>> = Mutex::new(None);

/// Whether the current [`TAXI_ROUTE`] is an ARRIVAL route to a parking stand
/// (vs a departure to a holding point) — set alongside the route so the crew
/// phrases "stand ..." instead of "holding point ...". Kept parallel rather
/// than a `TaxiWatch` field to avoid churning every construction of it.
pub(crate) static TAXI_TO_STAND: Mutex<bool> = Mutex::new(false);

/// A takeoff clearance withheld for landing traffic — armed by the reply
/// path, released by the monitor once the final has stayed clear.
pub(crate) static TAKEOFF_HOLD: Mutex<Option<TakeoffHold>> = Mutex::new(None);

/// An armed vectors-to-final plan — the approach controller's worksheet.
pub(crate) static VECTORS: Mutex<Option<VectorPlan>> = Mutex::new(None);

/// A station swap the main loop must consume (it owns the `Station`): the
/// new field's ICAO (for the voice casting) and airport, armed by
/// `region_tick` after the handoff line is spoken.
pub(crate) static PENDING_STATION: Mutex<Option<(String, nav::AirportInfo)>> = Mutex::new(None);

/// The TOWER frequency of the field the crew has committed to arrive at.
/// While it is set the enroute region may hand the station TO that field and
/// never away from it: a controller we are already talking to, inbound, is
/// not a controller a nearer strip may take from us. Set by the crew at the
/// inbound call, cleared when the arrival is over.
pub(crate) static ARRIVAL_PIN: Mutex<Option<i64>> = Mutex::new(None);

/// Set when the monitor speaks the released takeoff clearance; the main
/// loop consumes it and syncs the tower FSM to what was said on frequency.
pub(crate) static PENDING_TAKEOFF_CLEARANCE: Mutex<bool> = Mutex::new(false);

/// Set when the monitor sequences a held departure onto the runway with a
/// line-up-and-wait; the main loop consumes it and syncs the tower FSM to
/// what was said on frequency, exactly like [`PENDING_TAKEOFF_CLEARANCE`].
pub(crate) static PENDING_LINE_UP: Mutex<bool> = Mutex::new(false);

/// Set when the monitor orders a go-around (traffic on the runway); the main
/// loop consumes it and voids the landing clearance in the tower FSM, so the
/// coach can never keep suggesting the readback of a cancelled clearance.
pub(crate) static PENDING_GO_AROUND: Mutex<bool> = Mutex::new(false);

/// A CONDITIONAL line-up's condition, as the world sees it: `Some(false)`
/// while the named arrival is still on final, `Some(true)` once it is down on
/// the strip or gone, `None` when no conditional line-up stands. Published by
/// the release watch every tick, so the crew waits on the world's picture of
/// the traffic and never on its own guess.
pub(crate) static LINE_UP_CONDITION: Mutex<Option<bool>> = Mutex::new(None);

/// The after-takeoff handoff the tower spoke ("contact Malaga Departure on
/// one two one decimal zero two five"): the station's spoken name and its
/// frequency, armed once when the line goes out and consumed by the crew
/// that works the radio.
pub(crate) static DEPARTURE_HANDOFF: Mutex<Option<(String, i64)>> = Mutex::new(None);

/// The enroute handoff the region watch spoke ("radar services terminated,
/// squawk seven thousand, contact Malaga Tower on ..."): whom to call, on
/// what, and whether the discrete code goes back to conspicuity. Armed once
/// when the line goes out; consumed by whoever is flying (full auto reads it
/// back, dials 7000 and retunes; with the pilot flying the monitor coaches
/// once). It survives `Station::install` on purpose: the main loop installs
/// the NEW station on its very next pass, before the crew's loop has polled,
/// and an install that wiped it would lose every handoff. `uninstall` ends it.
pub(crate) static REGION_HANDOFF: Mutex<Option<RegionHandoff>> = Mutex::new(None);

/// The zone watch challenged us on the frequency ("aircraft in the Granada
/// control zone, you are inside controlled airspace without clearance, say
/// callsign and intentions"): the tower has never heard from this aircraft
/// and is asking. Armed each time the tuned line goes out; consumed by the
/// crew that works the radio, which answers with the inbound call when the
/// challenging tower is the destination's (a full-auto arrival flew the last
/// enroute leg through Granada's zone with three challenges unanswered,
/// 2026-09-15). The crew-side "we're inside the zone" variant, spoken by the
/// copilot when the radio is NOT on the tower, never arms it.
pub(crate) static ZONE_CHALLENGE: Mutex<bool> = Mutex::new(false);

/// What the enroute controller told us at the region boundary.
#[derive(Debug, Clone, PartialEq)]
pub struct RegionHandoff {
    /// The station to call, as spoken ("Malaga Tower").
    pub to: String,
    /// Its frequency in kHz.
    pub khz: i64,
    /// "Squawk seven thousand" was ordered: a radar service assigned a
    /// discrete code that is now void.
    pub squawk_vfr: bool,
}

/// The runway the APPROACH controller cleared you toward and handed to the
/// tower ("cleared visual/ILS approach runway X, contact Tower"). You still
/// owe the tower a "final" report before you're cleared to LAND — an approach
/// clearance is not a landing clearance. The crew's landing-clearance coach
/// reads it; the world clears it on the landing clearance and on touchdown, so
/// it never lingers into the next circuit.
pub(crate) static LANDING_HANDOFF: Mutex<Option<String>> = Mutex::new(None);

/// The discrete transponder code the controller assigned with the clearance,
/// so the crew can dial it into the transponder — the full-auto orchestrator
/// reads it after requesting clearance. Published by the tower on assignment,
/// cleared on `Station::uninstall`.
pub(crate) static ASSIGNED_SQUAWK: Mutex<Option<i64>> = Mutex::new(None);

fn lock<T>(m: &'static Mutex<T>) -> MutexGuard<'static, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

// ---- crew-side accessors ----

/// The tower FSM state of the standing clearance, if any.
/// The active tower's frequency (kHz), so the crew can tune COM1 to it before
/// calling — the full-auto orchestrator does this before requesting taxi.
pub fn home_tower_khz() -> Option<i64> {
    // A tower out of reach is no tower to tune (`station::note_aircraft`).
    if crate::station::out_of_reach() {
        return None;
    }
    crate::station::active_tower_khz()
}

/// A station reaches the aircraft: one is running and it is not out of
/// reach on the ground. False = the crew departs as from an uncontrolled
/// field ([`plan_self_taxi`]).
pub fn station_reaches() -> bool {
    !crate::station::out_of_reach() && crate::station::active_tower_khz().is_some()
}

/// The crew's own taxi at an uncontrolled field, planned from the field's
/// taxi network and published like a controller's route
/// ([`crate::station::plan_self_taxi`]). A full apt.dat pass: off the
/// realtime loop.
pub fn plan_self_taxi(
    apt_path: &str,
    icao: Option<&str>,
    lat: f64,
    lon: f64,
    runway: Option<&str>,
    wind: Option<(f64, f64)>,
) -> Result<crate::station::SelfTaxi, String> {
    crate::station::plan_self_taxi(apt_path, icao, lat, lon, runway, wind)
}

pub fn clearance_state() -> Option<tower::State> {
    lock(&CLEARANCE).as_ref().map(|c| c.state)
}

/// The runway END of the standing clearance ("09R"), if one is known —
/// the auto-takeoff line-up crosscheck reads it against the strip the
/// aircraft is actually pointing down.
pub fn clearance_runway() -> Option<String> {
    lock(&CLEARANCE)
        .as_ref()
        .map(|c| c.runway_ident.clone())
        .filter(|r| !r.is_empty())
        // An uncontrolled departure's own runway stands in for a cleared
        // one: the line-up and the takeoff cross-check it the same way.
        .or_else(crate::station::self_taxi_runway)
}

/// The readback the tower is still waiting for, phrased ("Cleared to land
/// runway 09, November one seven two Sierra Papa."), `None` once it has
/// been heard. The crew's own approach reads it back when an automation is
/// flying: the tower's "negative, runway zero niner ... cleared to land"
/// was neither echoed nor obeyed while autoland flew 27 into a 21 kt
/// tailwind (LXGB, 2026-09-14).
pub fn readback_owed() -> Option<String> {
    lock(&CLEARANCE)
        .as_ref()
        .and_then(|c| c.pending_readback.clone())
        .filter(|r| !r.is_empty())
}

/// Read the cleared taxi route under its lock. Closure-scoped so a guard
/// can never be held across an await.
pub fn with_taxi_route<R>(f: impl FnOnce(Option<&TaxiWatch>) -> R) -> R {
    f(lock(&TAXI_ROUTE).as_ref())
}

/// True when the cleared taxi route is an ARRIVAL to a parking stand — the crew
/// calls the destination a "stand", not a "holding point".
/// The nearest parking position (apt.dat 1300 row, any kind) to a point:
/// metres away, the heading the aircraft parks on, the painted name. `None`
/// when no station is installed, its network has not loaded, or the field
/// publishes no stands; [`stands_known`] tells those apart from "none near".
/// The crew asks this before calling a tug: a pushback belongs to a stand,
/// and the bearing-only trigger alone called one from the middle of a
/// taxiway on every restart after an off-route stop (LEMG, 2026-09-12).
#[derive(Clone, Debug, PartialEq)]
pub struct StandFix {
    pub dist_m: f64,
    pub heading: f64,
    pub name: String,
}

pub fn nearest_stand(lat: f64, lon: f64) -> Option<StandFix> {
    let net = crate::station::taxi_net()?;
    let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
    net.stands
        .iter()
        .map(|s| {
            let dx = (s.lon - lon) * m_per_deg_lon;
            let dy = (s.lat - lat) * 111_320.0;
            StandFix {
                dist_m: (dx * dx + dy * dy).sqrt(),
                heading: s.heading,
                name: s.name.clone(),
            }
        })
        .min_by(|a, b| a.dist_m.total_cmp(&b.dist_m))
}

/// Is this point on the active field's drawn pavement? `None` when no
/// field is loaded or it draws no pavement: no data is never "grass".
/// The painted way out of the stand the aircraft is on, read backwards from
/// it (see `nav::TaxiNet::push_lane`); `None` with no field or no paint.
pub fn push_lane(lat: f64, lon: f64, psi_deg: f64) -> Option<Vec<(f64, f64)>> {
    crate::station::taxi_net()?.push_lane(lat, lon, psi_deg)
}

pub fn on_pavement(lat: f64, lon: f64) -> Option<bool> {
    crate::station::taxi_net()?.on_pavement(lat, lon)
}

/// Is this point on one of the active field's runway strips? `None` when no
/// field's runways are published: no data is never "off the runway". The
/// flight model's `LinedUp` evidence, which the live monitor never fed, so
/// a lined-up aircraft read "Holding" and its roll "TaxiOut" (LIBP 04 and
/// LEMG 13, 2026-09-26: the duty engine asked for a taxi and a line-up
/// under the running takeoff).
pub fn on_runway(lat: f64, lon: f64) -> Option<bool> {
    crate::station::on_active_runway_known(lat, lon)
}

/// Whether the active field publishes any parking positions at all.
pub fn stands_known() -> bool {
    crate::station::taxi_net().is_some_and(|n| !n.stands.is_empty())
}

pub fn taxi_to_stand() -> bool {
    *lock(&TAXI_TO_STAND)
}

/// The drive OFF the runway we have just landed on, planned from the field's
/// taxi network. `psi_true` is the rollout direction; `runway` is the end we
/// landed on.
///
/// This is the one accessor here that is not a published CLEARANCE, and that
/// is deliberate rather than a leak. The world owns the airport's geometry —
/// the taxi network is loaded once per station and the crew has no business
/// keeping a second copy — but vacating needs no permission from the tower:
/// it is the completion of the landing clearance the crew already holds. So
/// the crew asks the world a question about pavement and gets an answer, with
/// no controller in the loop and nothing to read back. The station can still
/// never actuate; this returns geometry.
pub fn plan_vacate(
    lat: f64,
    lon: f64,
    psi_true: f64,
    runway: &str,
) -> Option<crate::nav::VacatePlan> {
    crate::station::plan_vacate_from(lat, lon, psi_true, runway)
}

/// The discrete squawk the controller assigned, if one has been — the crew dials
/// it into the transponder after the clearance.
pub fn assigned_squawk() -> Option<i64> {
    *lock(&ASSIGNED_SQUAWK)
}

/// A takeoff clearance is currently being withheld for traffic.
pub fn takeoff_hold_armed() -> bool {
    lock(&TAKEOFF_HOLD).is_some()
}

/// The condition of a CONDITIONAL line-up ("behind the landing X, line up
/// behind"): `Some(false)` while X is still on final, `Some(true)` once it
/// has passed, `None` when the standing line-up is unconditional or there is
/// none. A crew about to cross the hold line waits on `Some(false)`.
pub fn line_up_condition_clear() -> Option<bool> {
    *lock(&LINE_UP_CONDITION)
}

/// Consume the after-takeoff handoff the tower spoke: the station to call
/// ("Malaga Departure") and its frequency in kHz. The crew that works the
/// radio reads it back, retunes, and checks in.
pub fn take_departure_handoff() -> Option<(String, i64)> {
    lock(&DEPARTURE_HANDOFF).take()
}

/// Consume the enroute handoff the region watch spoke: the tower to call, its
/// frequency, and whether the transponder goes back to seven thousand. The
/// crew that works the radio reads it back on the OLD frequency, sets the
/// code and retunes; a pilot flying by hand gets one nudge if neither moved.
pub fn take_region_handoff() -> Option<RegionHandoff> {
    lock(&REGION_HANDOFF).take()
}

/// Consume the zone watch's challenge ("say callsign and intentions"), spoken
/// on the frequency to an aircraft the tower has never heard from. Drained
/// unconditionally by the crew's loop each pass: an answer is owed the tower
/// it came from, and a challenge nobody answered is not kept for a later one.
pub fn take_zone_challenge() -> bool {
    std::mem::take(&mut *lock(&ZONE_CHALLENGE))
}

/// The callsign under approach vectors, if a plan is armed.
pub fn vectored_callsign() -> Option<String> {
    lock(&VECTORS).as_ref().map(|p| p.callsign.clone())
}

/// What the controller has actually TOLD US to fly, for a crew that intends to
/// comply. `vectored_callsign` only says a plan exists; this says what was
/// assigned, which is what an autoflight arrival needs to obey rather than
/// invent its own path alongside a controller who thinks it is being vectored.
///
/// `None` until the first vector has been transmitted — before that there is
/// nothing owed, and the crew flies its own navigation.
#[derive(Debug, Clone, PartialEq)]
pub struct AssignedVector {
    /// Last assigned heading, degrees. Already rounded to the ten the
    /// controller actually spoke, so the crew flies the SPOKEN number.
    pub heading_deg: f64,
    /// Intercept (platform) altitude, ft MSL.
    pub target_alt_ft: f64,
    /// The approach clearance has been issued ("cleared ILS approach, report
    /// established"), so the vectors are done and the crew owes the report.
    pub cleared: bool,
    /// Landing runway as spoken, for the report and the crosscheck.
    pub runway_spoken: String,
}

/// The vector currently owed, if a controller is working us and has spoken at
/// least one instruction.
pub fn assigned_vector() -> Option<AssignedVector> {
    let g = lock(&VECTORS);
    let p = g.as_ref()?;
    Some(AssignedVector {
        heading_deg: p.last_heading_deg()? as f64,
        target_alt_ft: p.target_alt_ft,
        cleared: p.is_cleared(),
        runway_spoken: p.runway_spoken.clone(),
    })
}

/// The runway the approach handed you toward while you still owe the tower a
/// "final" report — i.e. cleared for the APPROACH but not to LAND. `None` once
/// the tower clears the landing or the wheels are down.
pub fn landing_handoff_runway() -> Option<String> {
    lock(&LANDING_HANDOFF).clone()
}

/// Consume the armed station swap. Main loop only — it owns the `Station`.
pub fn take_pending_station() -> Option<(String, nav::AirportInfo)> {
    lock(&PENDING_STATION).take()
}

/// Consume the released-takeoff-clearance flag. Main loop only.
pub fn take_pending_takeoff_clearance() -> bool {
    std::mem::take(&mut *lock(&PENDING_TAKEOFF_CLEARANCE))
}

/// Consume the sequenced-line-up flag. Main loop only.
pub fn take_pending_line_up() -> bool {
    std::mem::take(&mut *lock(&PENDING_LINE_UP))
}

/// Consume the go-around-ordered flag. Main loop only.
pub fn take_pending_go_around() -> bool {
    std::mem::take(&mut *lock(&PENDING_GO_AROUND))
}
