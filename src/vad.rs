//! Speech gate — confirms a captured segment is actually human speech before it
//! reaches whisper.
//!
//! The energy segmenter ([`crate::audio::Segmenter`]) triggers on anything louder
//! than the adaptive noise floor, so non-speech bursts — sim/engine audio through
//! speakers, a knock, a chair creak — still become "utterances" and get
//! transcribed (often into confident garbage). This runs a small learned VAD
//! (`earshot`, pure Rust, no native deps) over the segment and drops it when too
//! little of it is voiced, so that noise never reaches the model or the LLM.

use earshot::Detector;

/// Frame `earshot` expects: exactly 256 samples = 16 ms at 16 kHz.
const FRAME: usize = 256;
/// A frame scores in [0, 1]; > this counts as voiced (the crate's suggested mark).
const VOICED_SCORE: f32 = 0.5;
/// Accept the segment as speech if at least this fraction of frames are voiced.
/// A real command is voiced most of the way through; noise bursts are sparse.
const MIN_VOICED_FRAC: f32 = 0.30;
/// Segments shorter than this many frames (~48 ms) are too brief to judge — let
/// them through (the energy gate already passed them, and clipping a real short
/// word is worse than passing a rare blip).
const MIN_FRAMES: u32 = 3;

/// Stateful speech detector, reused across utterances (reset per segment).
pub struct SpeechFilter {
    vad: Detector,
}

impl SpeechFilter {
    pub fn new() -> Self {
        Self {
            vad: Detector::const_default(),
        }
    }

    /// True if enough of `audio` (16 kHz mono f32) is voiced speech.
    pub fn is_speech(&mut self, audio: &[f32]) -> bool {
        self.vad.reset(); // each segment is judged independently
        let mut total = 0u32;
        let mut voiced = 0u32;
        let mut buf = [0f32; FRAME];
        for frame in audio.chunks_exact(FRAME) {
            // earshot asserts samples are in [-1, 1]; a hot mic can clip past it.
            for (b, &s) in buf.iter_mut().zip(frame) {
                *b = s.clamp(-1.0, 1.0);
            }
            total += 1;
            if self.vad.predict_f32(&buf) > VOICED_SCORE {
                voiced += 1;
            }
        }
        if total < MIN_FRAMES {
            return true; // too short to judge
        }
        (voiced as f32 / total as f32) >= MIN_VOICED_FRAC
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn silence_and_noise_are_not_speech() {
        let mut f = SpeechFilter::new();
        // 1s of digital silence.
        assert!(
            !f.is_speech(&vec![0.0f32; 16_000]),
            "silence should be rejected"
        );
        // 1s of low-level hiss (deterministic wrapping LCG, no rng/overflow).
        let mut x: u64 = 1;
        let noise: Vec<f32> = (0..16_000)
            .map(|_| {
                x = x.wrapping_mul(6364136223846793005).wrapping_add(1);
                ((x >> 40) as f32 / 16_777_216.0 - 0.5) * 0.04
            })
            .collect();
        assert!(!f.is_speech(&noise), "quiet hiss should be rejected");
    }

    /// Real speech passes. Synthesized with `say` (no mic in CI), resampled to the
    /// 16 kHz mono the gate expects. Ignored — needs macOS say/afconvert.
    /// Run: `cargo test vad_passes_real_speech -- --ignored --nocapture`
    #[test]
    #[ignore = "requires macOS say/afconvert"]
    fn vad_passes_real_speech() {
        use std::process::Command;
        Command::new("say")
            .args(["-o", "/tmp/xc-vad.aiff", "Roger, flaps ten, gear up."])
            .status()
            .unwrap();
        Command::new("afconvert")
            .args([
                "/tmp/xc-vad.aiff",
                "/tmp/xc-vad.wav",
                "-d",
                "LEI16@16000",
                "-c",
                "1",
                "-f",
                "WAVE",
            ])
            .status()
            .unwrap();
        let samples: Vec<f32> = hound::WavReader::open("/tmp/xc-vad.wav")
            .unwrap()
            .into_samples::<i16>()
            .map(|s| s.unwrap() as f32 / i16::MAX as f32)
            .collect();
        let mut f = SpeechFilter::new();
        assert!(f.is_speech(&samples), "real speech should pass the gate");
    }
}
