//! Speech-to-text via `whisper-rs` (whisper.cpp bindings), running locally.

use anyhow::{Context, Result};
use whisper_rs::{FullParams, SamplingStrategy, WhisperContext, WhisperContextParameters};

/// Cockpit vocabulary used to bias decoding (whisper's `initial_prompt`). It
/// conditions the model toward the words we actually expect, so "beacon" stops
/// becoming "beak" and "flaps" stops becoming "look that's". Phrased as a
/// natural sentence — that's how whisper expects the prompt.
const VOCAB_PROMPT: &str = "Cessna cockpit commands: flaps up, flaps ten, flaps \
twenty, flaps full. Landing light, taxi light, beacon, strobe lights, navigation \
lights on or off. Autopilot heading, altitude, vertical speed, climb, descend. \
Throttle, mixture, magnetos, parking brake, start the engine. Set altimeter, \
tune com one. Knots, feet, degrees, percent.";

/// A transcription plus a confidence score (mean per-token probability,
/// 0.0–1.0). Low confidence means the model wasn't sure — likely noise or a
/// half-caught word — and the command should be re-asked rather than guessed.
pub struct Transcription {
    pub text: String,
    pub confidence: f32,
}

/// Phrases whisper hallucinates on silence/noise — it memorized them from
/// caption-heavy training data. They arrive high-confidence, so they slip the
/// confidence gate; we drop them outright. Matched case-insensitively as
/// substrings. None of these overlap real cockpit commands.
const HALLUCINATIONS: &[&str] = &[
    "beadaholique",
    "thanks for watching",
    "thank you for watching",
    "subtitles by",
    "amara.org",
    "subscribe",
    "like and subscribe",
    "see you next time",
    "see you in the next",
    "♪",
];

/// Structural markers of caption-style hallucinations (website plugs, "go to …
/// for more information"). Catches the whole family without enumerating each
/// phrase — a cockpit command never contains a URL or a "for more information"
/// tail, so these are safe to treat as non-speech.
const CAPTION_MARKERS: &[&str] = &["www.", "http", ".com", ".org", ".net", "for more information"];

/// A loaded whisper model, reusable across utterances.
pub struct Stt {
    ctx: WhisperContext,
    /// Decoding bias prompt (whisper `initial_prompt`). Defaults to the generic
    /// cockpit vocab; [`set_prompt`](Self::set_prompt) swaps in a profile-aware one.
    prompt: String,
}

impl Stt {
    /// Load a whisper.cpp `.bin` model from disk. `use_gpu=false` runs inference
    /// on the CPU instead of the Metal GPU — slower, but it stops STT contending
    /// with a GPU-bound X-Plane for frames (the M5's CPU has headroom to spare).
    pub fn load(model_path: &str, use_gpu: bool) -> Result<Self> {
        let mut params = WhisperContextParameters::default();
        params.use_gpu(use_gpu);
        let ctx = WhisperContext::new_with_params(model_path, params)
            .with_context(|| format!("loading whisper model: {model_path}"))?;
        Ok(Self { ctx, prompt: VOCAB_PROMPT.to_string() })
    }

    /// Replace the decoding bias prompt — used to install the per-aircraft vocab
    /// from [`build_prompt`].
    pub fn set_prompt(&mut self, prompt: String) {
        self.prompt = prompt;
    }

    /// Transcribe 16 kHz mono f32 audio to text plus a confidence score.
    pub fn transcribe(&self, audio: &[f32]) -> Result<Transcription> {
        let started = std::time::Instant::now();
        let mut state = self.ctx.create_state().context("creating whisper state")?;

        // Beam search is more accurate than greedy for short, noisy commands.
        let mut params = FullParams::new(SamplingStrategy::BeamSearch {
            beam_size: 5,
            patience: -1.0,
        });
        params.set_print_progress(false);
        params.set_print_special(false);
        params.set_print_realtime(false);
        params.set_language(Some("en"));
        // Bias decoding toward the cockpit vocabulary (profile-aware if set).
        params.set_initial_prompt(&self.prompt);
        // Reduce hallucination on non-speech: suppress non-speech tokens and
        // leading blanks, and don't carry context between segments (so one ghost
        // can't seed the next).
        params.set_suppress_non_speech_tokens(true);
        params.set_suppress_blank(true);
        params.set_no_context(true);

        state.full(params, audio).context("running whisper inference")?;

        let segments = state
            .full_n_segments()
            .context("counting whisper segments")?;
        let mut text = String::new();
        let mut prob_sum = 0.0f32;
        let mut prob_n = 0u32;
        for i in 0..segments {
            text.push_str(&state.full_get_segment_text(i).context("reading segment")?);
            // Average per-token probability → an utterance confidence.
            let n_tokens = state.full_n_tokens(i).context("counting tokens")?;
            for t in 0..n_tokens {
                if let Ok(p) = state.full_get_token_prob(i, t) {
                    prob_sum += p;
                    prob_n += 1;
                }
            }
        }
        let confidence = if prob_n > 0 { prob_sum / prob_n as f32 } else { 0.0 };
        let text = text.trim().to_string();

        // Drop known whisper hallucinations (they come through high-confidence).
        if is_hallucination(&text) {
            tracing::debug!(transcript = %text, "dropped whisper hallucination");
            return Ok(Transcription { text: String::new(), confidence: 0.0 });
        }

        tracing::info!(
            ms = started.elapsed().as_millis(),
            audio_s = format!("{:.1}", audio.len() as f32 / 16_000.0),
            "stt"
        );
        // Normalize spoken digit runs ("two seven zero" → "270") for the LLM.
        let text = crate::phrase::normalize_numbers(&text);
        Ok(Transcription { text, confidence })
    }
}

/// Build a profile-aware decoding prompt: bias whisper toward the systems and
/// phrasings the *active aircraft* actually exposes, plus number phraseology.
/// Falls back to [`VOCAB_PROMPT`] for callers without a profile.
pub fn build_prompt(p: &crate::aircraft::AircraftProfile) -> String {
    let mut s = format!("{} cockpit commands. ", p.display_name);
    if let Some(f) = &p.flaps {
        let detents: Vec<String> = f.detents.iter().map(|d| format!("flaps {}", d.spoken)).collect();
        if !detents.is_empty() {
            s.push_str(&detents.join(", "));
            s.push_str(". ");
        }
    }
    if p.retractable_gear {
        s.push_str("Gear up, gear down. ");
    }
    if !p.lights.is_empty() {
        let lights: Vec<&str> = p.lights.iter().map(|l| l.spoken.as_str()).collect();
        s.push_str(&format!("Lights: {} on or off. ", lights.join(", ")));
    }
    if p.flood_lights {
        s.push_str("Flood lights. ");
    }
    if p.autopilot {
        s.push_str("Autopilot heading, altitude hold, vertical speed, climb, descend, wings level. ");
    }
    if p.engine {
        s.push_str("Throttle, mixture, start the engine, shut down. ");
    }
    if p.magnetos {
        s.push_str("Magnetos left, right, both. ");
    }
    if p.carb_heat {
        s.push_str("Carb heat. ");
    }
    if p.fuel_pump {
        s.push_str("Fuel pump on or off. ");
    }
    if p.fuel_selector.is_some() {
        s.push_str("Fuel selector left, right, both. ");
    }
    if p.pitot_heat {
        s.push_str("Pitot heat. ");
    }
    if p.pitch_trim {
        s.push_str("Trim nose up, nose down. ");
    }
    if p.parking_brake {
        s.push_str("Parking brake set or released. ");
    }
    if !p.electrical.is_empty() {
        s.push_str("Battery, alternator, avionics master. ");
    }
    if p.com_radio {
        s.push_str("Com one, com two, tune frequency. ");
    }
    if p.nav_radio {
        s.push_str("Nav one, set the OBS. ");
    }
    if p.transponder {
        s.push_str("Squawk, ident, transponder standby. ");
    }
    if p.barometer {
        s.push_str("Set altimeter. ");
    }
    if p.directional_gyro {
        s.push_str("Align the gyro. ");
    }
    if p.ils {
        s.push_str("Prepare the ILS for the runway. ");
    }
    s.push_str(
        "Numbers are spoken digit by digit: heading two seven zero, squawk one two zero zero, \
         com one one eight decimal five, altimeter two niner niner two; altitudes like seven \
         thousand feet; knots, feet, degrees, percent.",
    );
    s
}

/// True if the transcript is a known whisper ghost phrase or a caption-style
/// hallucination (URL / "for more information").
fn is_hallucination(text: &str) -> bool {
    let lower = text.to_lowercase();
    HALLUCINATIONS.iter().any(|h| lower.contains(h))
        || CAPTION_MARKERS.iter().any(|m| lower.contains(m))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::process::Command;

    /// Verify the real whisper model transcribes a cockpit command. We don't
    /// have a mic in CI, so we synthesize speech with macOS `say`, downsample to
    /// the 16 kHz mono whisper wants, and feed that through the model.
    ///
    /// Run with: `cargo test transcribes_spoken_command -- --ignored --nocapture`
    #[test]
    #[ignore = "requires models/ggml-base.en.bin and macOS say/afconvert"]
    fn transcribes_spoken_command() {
        let aiff = "/tmp/xc-stt.aiff";
        let wav = "/tmp/xc-stt.wav";
        let phrase = "flaps ten degrees";

        Command::new("say")
            .args(["-o", aiff, phrase])
            .status()
            .expect("running say");
        // 16 kHz, mono, 16-bit — exactly what we feed whisper at runtime.
        Command::new("afconvert")
            .args([aiff, wav, "-d", "LEI16@16000", "-c", "1", "-f", "WAVE"])
            .status()
            .expect("running afconvert");

        let reader = hound::WavReader::open(wav).expect("opening wav");
        let samples: Vec<f32> = reader
            .into_samples::<i16>()
            .map(|s| s.unwrap() as f32 / i16::MAX as f32)
            .collect();

        let model = std::env::var("WHISPER_MODEL")
            .unwrap_or_else(|_| "models/ggml-base.en.bin".into());
        let stt = Stt::load(&model, true).expect("loading model");
        let result = stt.transcribe(&samples).expect("transcribing");
        let text = result.text.to_lowercase();

        println!("  said: {phrase:?}\n  heard: {text:?} (confidence {:.2})", result.confidence);
        assert!(text.contains("flap"), "expected 'flap' in transcript {text:?}");
        assert!(result.confidence > 0.0, "confidence should be populated");
    }

    #[test]
    fn hallucinations_are_filtered_real_commands_are_not() {
        assert!(is_hallucination(
            "Go to Beadaholique.com for all of your beading supplies needs!"
        ));
        assert!(is_hallucination("Thanks for watching!"));
        // Caption-style ghost seen live — caught by the structural markers, not
        // by an enumerated phrase.
        assert!(is_hallucination("Go to www.cessna.com for more information."));
        assert!(is_hallucination("Visit http://example.org now"));
        assert!(!is_hallucination("flaps ten degrees"));
        assert!(!is_hallucination("autopilot heading 270"));
        // "go to" alone must not trip it — a pilot might say "go to heading 270".
        assert!(!is_hallucination("go to heading 270"));
    }
}
