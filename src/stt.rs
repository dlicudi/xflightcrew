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

    /// Transcribe 16 kHz mono f32 audio to text plus a confidence score, applying
    /// the command-loop domain fixes (mishear repair + digit-run normalization) so
    /// the real keyword/number reaches the gate. Use [`transcribe_verbatim`] for
    /// callers that need the literal words (e.g. voice-clone captioning).
    pub fn transcribe(&self, audio: &[f32]) -> Result<Transcription> {
        let t = self.transcribe_verbatim(audio)?;
        if t.text.is_empty() {
            return Ok(t);
        }
        // On near-silence whisper sometimes hallucinates its own bias prompt
        // back ("Numbers are spoken digit by digit.", seen live with the tower
        // prompt installed). Only a transcript that IS a whole prompt sentence
        // drops — word-level containment would eat real calls like "ready for
        // departure", whose every word appears in the prompt vocabulary.
        if is_prompt_echo(&t.text, &self.prompt) {
            tracing::info!(text = %t.text, "dropped prompt echo (whisper hallucinated its bias prompt)");
            return Ok(Transcription { text: String::new(), ..t });
        }
        // Clean the transcript for the gate and grammar: join hyphenated compounds
        // ("auto-pilot" → "autopilot"), repair whole-word mishears the bias prompt
        // can't prevent ("gyro" → "J-Rule"), then normalize spoken digit runs
        // ("two seven zero" → "270"). All run before the domain gate so the real
        // keyword reaches it.
        let text = crate::phrase::collapse_hyphens(&t.text);
        let text = crate::phrase::fix_mishears(&text);
        let text = crate::phrase::normalize_numbers(&text);
        Ok(Transcription { text, ..t })
    }

    /// Transcribe to the literal words whisper heard — no domain normalization.
    /// The captioning path needs this: a voice-clone reference text must match the
    /// audio's actual phonemes, so "two seven zero" must stay words, not "270".
    pub fn transcribe_verbatim(&self, audio: &[f32]) -> Result<Transcription> {
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
        Ok(Transcription { text, confidence })
    }
}

/// Build a profile-aware decoding prompt: bias whisper toward the systems and
/// phrasings the *active aircraft* actually exposes, plus number phraseology.
/// Falls back to [`VOCAB_PROMPT`] for callers without a profile.
pub fn build_prompt(p: &crate::aircraft::AircraftProfile) -> String {
    // Whisper conditions on `initial_prompt` as if it were prior speech, so this
    // must stay SHORT and sentence-like. An earlier version enumerated every
    // system and value (~130 words); that long, clipped keyword list made the
    // small base.en model bias toward that style and mis-split off-list words —
    // notably "gyro" → "J-Rule" even though "gyro" was *in* the prompt. We now
    // name only the systems the aircraft actually has, in one compact phrase,
    // prime the one known-tricky term, and keep the number phraseology hint.
    let mut terms: Vec<&str> = Vec::new();
    if p.flaps.is_some() {
        terms.push("flaps");
    }
    if p.engine {
        terms.push("throttle");
        terms.push("mixture");
    }
    if p.retractable_gear {
        terms.push("gear");
    }
    if !p.lights.is_empty() {
        terms.push("lights");
    }
    if p.autopilot {
        terms.push("autopilot heading and altitude");
    }
    if p.magnetos {
        terms.push("magnetos");
    }
    if p.fuel_pump || p.fuel_selector.is_some() {
        terms.push("fuel");
    }
    if p.parking_brake {
        terms.push("parking brake");
    }
    // "pitot" defeats whisper without priming ("Pito's heat", "it was heat",
    // "pilot heat" all seen live); name it when the airframe has the switch.
    if p.pitot_heat {
        terms.push("pitot heat");
    }
    if !p.electrical.is_empty() {
        terms.push("battery and avionics");
    }
    if p.com_radio || p.nav_radio {
        terms.push("radios");
    }
    if p.transponder {
        terms.push("squawk");
    }
    if p.barometer {
        terms.push("altimeter");
    }

    // No aircraft name in the prompt: it's not vocabulary the pilot speaks, and
    // whisper conditions on the prompt as prior speech — leading with "Cessna
    // 172SP Skyhawk" leaks "Skyhawk" into short transcripts ("align gyro" →
    // "skyhawk and gyro"). Bias on the systems and phraseology only.
    let mut s = format!("Cockpit commands: {}. ", terms.join(", "));
    if p.directional_gyro {
        // "gyro" is reliably misheard as "J-Rule" without priming.
        s.push_str("Align the gyro, fix the gyro. ");
    }
    s.push_str(
        "Numbers are spoken digit by digit: heading two seven zero, squawk one two zero zero, \
         altimeter two niner niner two.",
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

/// True when the transcript is a whole SENTENCE of the bias prompt, which
/// whisper hallucinates back on near-silence ("Numbers are spoken digit by
/// digit.", seen live with the tower prompt installed). Sentence-exact on
/// purpose: the prompt is a vocabulary list, so word-level matching would eat
/// genuine calls built from the same words ("ready for departure").
fn is_prompt_echo(text: &str, prompt: &str) -> bool {
    fn canon(s: &str) -> String {
        s.to_ascii_lowercase()
            .split(|c: char| !c.is_ascii_alphanumeric())
            .filter(|w| !w.is_empty())
            .collect::<Vec<_>>()
            .join(" ")
    }
    let heard = canon(text);
    if heard.split(' ').count() < 3 {
        return false; // too short to be a prompt sentence
    }
    prompt
        .split(['.', ':'])
        .map(canon)
        .any(|sentence| !sentence.is_empty() && sentence == heard)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::process::Command;

    #[test]
    fn prompt_echoes_drop_but_real_calls_survive() {
        // The live hallucination: whisper reading its bias prompt back.
        assert!(is_prompt_echo(
            "Numbers are spoken digit by digit.",
            crate::tower::STT_PROMPT
        ));
        // Real calls made of prompt vocabulary must NOT be eaten.
        for call in [
            "ready for departure",
            "request taxi",
            "radio check",
            "request departure clearance",
            "runway zero niner",
        ] {
            assert!(!is_prompt_echo(call, crate::tower::STT_PROMPT), "{call}");
        }
        // The profile prompt's tail sentence, as build_prompt writes it.
        let p = build_prompt(&crate::aircraft::builtin("cessna_172sp").unwrap());
        assert!(is_prompt_echo("Numbers are spoken digit by digit", &p));
        assert!(!is_prompt_echo("flaps ten", &p));
    }

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
