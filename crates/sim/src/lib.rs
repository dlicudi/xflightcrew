//! Shared simulator plumbing — the ground both the crew and the world stand on.
//!
//! `xplane` is the Web API client (read/write, plus the `XPlane` trait and test
//! fake), `datarefs` the curated read-name registry, `blackbox` the JSONL
//! flight recorder. Nothing in here knows about the copilot loop or the ATC
//! station; both depend on this crate, never the other way around.

pub mod blackbox;
pub mod datarefs;
pub mod xplane;

pub mod run;
