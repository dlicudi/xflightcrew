//! Our own AI traffic (stage D, `docs/tasks/pillars-audit/stage-d-traffic.md`).
//!
//! Pure and deterministic: performance tables ([`perf`]), kinematics
//! ([`kin`]), a sim-time clock ([`clock`]) and a seeded generator ([`rng`]).
//! No statics: everything here is a value the bin owns. Depends on `nav`
//! only; never on `atc` (the two meet in the bin) and never on whisper,
//! cpal or ort.

pub mod clock;
pub mod kin;
pub mod perf;
pub mod rng;

pub use clock::{SimClock, SUBSTEP_S};
pub use kin::{follow, Follow, FollowSpec, Kin, LatLon, Lights};
pub use perf::{class_for_type, Perf, Wake, JET_HEAVY, JET_MED, PISTON_SINGLE, TURBOPROP};
pub use rng::XorShift64;
