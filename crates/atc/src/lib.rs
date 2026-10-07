//! The world side: ATC as a library.
//!
//! Everything here observes the sim and RETURNS text tagged with a
//! [`speaker::StationSpeaker`] — it never actuates the aircraft and never
//! speaks. The embedder owns the mouth: the copilot's voice arbiter casts
//! and plays the lines; a standalone binary would bring its own TTS. That
//! return-value seam is deliberate — do not add a speech callback/trait
//! here; if a feature ever genuinely needs world-initiated audio, inject a
//! narrow trait into that feature alone.
//!
//! Crew/world state crosses ONLY through `party_line` (its statics stay
//! crate-private) and `world` (the ticks the crew's monitor drives): the
//! station module itself is `pub(crate)`, so the compiler enforces the seam.
//! This crate must stay lean: no whisper, no cpal, no ort; `cargo tree -p
//! atc` is the standalone-product health check.

pub mod airspace;
pub mod atis;
pub mod callsign;
pub mod nav;
pub mod party_line;
pub mod speaker;
pub(crate) mod station;
pub mod tower;
pub mod world;
