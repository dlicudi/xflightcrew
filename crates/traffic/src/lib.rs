//! Our own AI traffic (stage D, `docs/tasks/pillars-audit/stage-d-traffic.md`).
//!
//! Pure and deterministic: performance tables ([`perf`]), kinematics
//! ([`kin`]), a sim-time clock ([`clock`]), a seeded generator ([`rng`]),
//! the field ([`field`]), the phase machine ([`phase`], [`aircraft`]) and
//! the fleet that steps it ([`fleet`]), keeping its distance on the
//! ground ([`spacing`]). No statics: everything here is a
//! value the bin owns. Depends on `nav` only; never on `atc` (the two meet
//! in the bin) and never on whisper, cpal or ort.

pub mod aircraft;
pub mod airlines;
pub mod arrival;
pub mod board;
pub mod clock;
pub mod field;
pub mod fleet;
pub mod kin;
pub mod midflow;
pub mod perf;
pub mod phase;
pub mod places;
pub mod rate;
pub mod rng;
pub mod rotation;
pub mod routes;
pub mod schedule;
pub mod slot;
pub mod spacing;
pub mod wire;

pub use aircraft::{Route, Seen, TrafficAircraft};
pub use clock::{SimClock, SUBSTEP_S};
pub use field::{FieldModel, Procedures, RunwayEnd};
pub use fleet::Fleet;
pub use kin::{follow, follow_on_gear, Follow, FollowSpec, Gear, Kin, LatLon, Lights};
pub use perf::{class_for_type, Perf, Wake, JET_HEAVY, JET_MED, PISTON_SINGLE, TURBOPROP};
pub use phase::{Call, Despawn, Event, Leg, Order, OwnTarget, Phase, Plan, RadioState, TowerState};
pub use rng::XorShift64;
pub use schedule::{
    generate, park_at_start, Density, FileEntry, FlightInfo, Movement, MovementKind, Refusal,
    SpawnCtx, Start,
};

#[cfg(test)]
mod sortie_tests;
