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

/// Set when the monitor orders a go-around (traffic on the runway); the main
/// loop consumes it and voids the landing clearance in the tower FSM, so the
/// coach can never keep suggesting the readback of a cancelled clearance.
pub(crate) static PENDING_GO_AROUND: Mutex<bool> = Mutex::new(false);

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

/// The discrete squawk the controller assigned, if one has been — the crew dials
/// it into the transponder after the clearance.
pub fn assigned_squawk() -> Option<i64> {
    *lock(&ASSIGNED_SQUAWK)
}

/// A takeoff clearance is currently being withheld for traffic.
pub fn takeoff_hold_armed() -> bool {
    lock(&TAKEOFF_HOLD).is_some()
}

/// The callsign under approach vectors, if a plan is armed.
pub fn vectored_callsign() -> Option<String> {
    lock(&VECTORS).as_ref().map(|p| p.callsign.clone())
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

/// Consume the go-around-ordered flag. Main loop only.
pub fn take_pending_go_around() -> bool {
    std::mem::take(&mut *lock(&PENDING_GO_AROUND))
}
