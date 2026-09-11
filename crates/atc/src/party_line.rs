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
    crate::station::active_tower_khz()
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
}

/// Read the cleared taxi route under its lock. Closure-scoped so a guard
/// can never be held across an await.
pub fn with_taxi_route<R>(f: impl FnOnce(Option<&TaxiWatch>) -> R) -> R {
    f(lock(&TAXI_ROUTE).as_ref())
}

/// True when the cleared taxi route is an ARRIVAL to a parking stand — the crew
/// calls the destination a "stand", not a "holding point".
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
