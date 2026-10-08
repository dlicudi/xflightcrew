//! The station's side of our own AI traffic (stage D): the producer (the
//! bin's fleet) reports where each traffic flight is, and the runway book
//! reads it like any strip. The traffic is plain data here; this crate never
//! names the fleet, and the fleet never names a strip.

use super::*;

/// Where traffic flight `id` is now, from its producer: written to its
/// strip's [`FlightStrip::seen`] cell, opening the strip if none is open
/// (the tower sees every aircraft on its field, called or not). Lock order:
/// the map, clone the `Arc`, drop the map, then the cell.
pub fn note_traffic_state(id: u32, state: AircraftState) {
    let strip = open_traffic_strip(id);
    *strip.seen.lock().unwrap_or_else(|e| e.into_inner()) = Some(state);
}
