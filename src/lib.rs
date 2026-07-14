//! Library surface for the sibling binaries under `src/bin/` (currently
//! `atc`, the ATC process). The main copilot binary keeps its own module tree
//! rooted at `src/main.rs`; this exposes only the leaf modules a second
//! process needs, all of which are self-contained (no `crate::` reach into
//! the copilot loop). The proper workspace split (a shared core crate + two
//! thin binaries) can happen once the ATC process proves out; this is the
//! cheap version that gets a separate process today.
//!
//! Note the modules here are compiled twice (once for this lib, once inside
//! the copilot binary's own tree). That's deliberate scaffolding, not an
//! accident: the two processes share code, not state.

pub use sim::{datarefs, xplane};

pub mod aircraft;
pub mod atis;
pub mod config;
pub mod mic;
pub mod nav;
pub mod phrase;
pub mod prefs;
pub mod radiofx;
pub mod stt;
pub mod tower;
pub mod tts;
#[cfg(feature = "onnx-tts")]
pub mod tts_onnx;
