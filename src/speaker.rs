//! Who is speaking on the radio — pure identity, no audio.
//!
//! The world side (station/tower) tags every returned line with a
//! [`StationSpeaker`]; the embedder decides what that position sounds like
//! (the copilot's voice arbiter casts Piper models in `voice.rs`, the
//! standalone bin brings its own TTS). Voice casting deliberately does NOT
//! live here: the ATC layer returns text and identity, never speech.

/// Which controller position is speaking — each gets its own warm ONNX voice
/// (different person, different frequency), all through the one speech arbiter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StationSpeaker {
    Tower,
    Approach,
    Departure,
    /// The ATIS broadcast — the looped recording on its own frequency.
    Atis,
    /// A GHOST pilot on the party line — live traffic given a voice. The
    /// seed (hash of its callsign) picks a stable person from the pool, so
    /// Speedbird 306 sounds like the same pilot on every call.
    Ghost { seed: usize },
}

impl StationSpeaker {
    /// Stable position index 0..=4 — the warm-session chair this position
    /// occupies on the copilot side (all ghosts share one chair: the cache is
    /// keyed by model name, so a different ghost voice rebuilds it).
    pub fn slot(self) -> usize {
        match self {
            StationSpeaker::Tower => 0,
            StationSpeaker::Approach => 1,
            StationSpeaker::Departure => 2,
            StationSpeaker::Atis => 3,
            StationSpeaker::Ghost { .. } => 4,
        }
    }

    /// The log/panel tag for this position ("TWR: ..." rows). The plugin maps
    /// each prefix to its own chip, so the transcript names WHICH controller
    /// spoke, not a generic "ATC".
    pub fn tag(self) -> &'static str {
        match self {
            StationSpeaker::Tower => "TWR",
            StationSpeaker::Approach => "APP",
            StationSpeaker::Departure => "DEP",
            StationSpeaker::Atis => "ATIS",
            StationSpeaker::Ghost { .. } => "TFC",
        }
    }
}

/// A stable per-airport voice seed (FNV-1a over the name), never 0 — seed 0
/// is reserved for the configured home field's curated voices.
pub fn station_voice_seed(airport_name: &str) -> usize {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in airport_name.as_bytes() {
        h ^= *b as u64;
        h = h.wrapping_mul(0x100_0000_01b3);
    }
    ((h as usize) | 1).max(1)
}
