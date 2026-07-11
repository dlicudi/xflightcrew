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

fn lock<T>(m: &'static Mutex<T>) -> MutexGuard<'static, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

// ---- crew-side accessors ----

/// The tower FSM state of the standing clearance, if any.
pub fn clearance_state() -> Option<tower::State> {
    lock(&CLEARANCE).as_ref().map(|c| c.state)
}

/// Read the cleared taxi route under its lock. Closure-scoped so a guard
/// can never be held across an await.
pub fn with_taxi_route<R>(f: impl FnOnce(Option<&TaxiWatch>) -> R) -> R {
    f(lock(&TAXI_ROUTE).as_ref())
}

/// A takeoff clearance is currently being withheld for traffic.
pub fn takeoff_hold_armed() -> bool {
    lock(&TAKEOFF_HOLD).is_some()
}

/// The callsign under approach vectors, if a plan is armed.
pub fn vectored_callsign() -> Option<String> {
    lock(&VECTORS).as_ref().map(|p| p.callsign.clone())
}

/// Consume the armed station swap. Main loop only — it owns the `Station`.
pub fn take_pending_station() -> Option<(String, nav::AirportInfo)> {
    lock(&PENDING_STATION).take()
}

/// Consume the released-takeoff-clearance flag. Main loop only.
pub fn take_pending_takeoff_clearance() -> bool {
    std::mem::take(&mut *lock(&PENDING_TAKEOFF_CLEARANCE))
}
