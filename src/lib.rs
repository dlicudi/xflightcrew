//! Library surface for the sibling binaries under `src/bin/` (currently
//! `atc`, the old standalone-ATC dev skeleton). The world side (station,
//! tower, nav, atis) lives in the `atc` workspace crate and the sim plumbing
//! in `sim`; this lib re-exports them alongside the crew-owned leaf modules
//! (mic, stt, tts, config) a second binary needs. Nothing is compiled twice
//! anymore — the workspace crates build once and everyone links them.

pub use ::atc::{atis, callsign, nav, tower};
pub use sim::{angles, datarefs, xplane};

pub mod aircraft;
pub mod config;
pub mod mic;
pub mod phrase;
pub mod prefs;
pub mod radiofx;
pub mod stt;
// Test-only (the file's own `#![cfg(test)]`).
mod test_support;
pub mod tts;
#[cfg(feature = "onnx-tts")]
pub mod tts_onnx;
