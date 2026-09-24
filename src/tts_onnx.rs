//! In-process ONNX TTS (piper VITS) — behind the `onnx-tts` feature.
//!
//! The win over the `say`/`piper` *subprocess* backends is a **warm session**:
//! the voice model loads once (in the [`crate::voice`] speaker thread) and every
//! readback reuses it, removing the per-utterance model-reload tax that a
//! subprocess pays each call. `bench/tts_warm_vs_cold.py` measured that tax at
//! ~6× on this very model (~20 ms warm vs ~280 ms reload for "flaps up").
//!
//! Split of work: phonemes come from `espeak-ng` (shelled — cheap, the same
//! phonemizer piper uses); inference stays in-process via `ort` (the expensive
//! part, now warm). Output PCM is written to a WAV and handed to the existing
//! platform player ([`crate::tts::play`]).

use std::collections::HashMap;
use std::path::Path;
use std::process::{Command, Stdio};

use anyhow::{anyhow, bail, Context, Result};
use ort::session::Session;
use ort::value::Tensor;

/// A loaded piper VITS voice with a warm inference session.
pub struct OnnxTts {
    session: Session,
    /// `phoneme -> token id(s)`, from the voice's `.onnx.json`.
    phoneme_ids: HashMap<String, Vec<i64>>,
    sample_rate: u32,
    /// `[noise_scale, length_scale, noise_w]` — the VITS inference scales.
    scales: [f32; 3],
    /// The model's OWN `length_scale` from its `.onnx.json`: its natural
    /// pace. A pace override is a multiple of it (see [`Self::override_scales`]).
    native_length: f32,
    /// Speaker id for MULTI-speaker voices (`Some` only when the model hosts
    /// more than one): one .onnx file, several distinct people. Selected by
    /// the `#N` suffix on the voice spec ("es_ES-sharvard-medium.onnx#1").
    speaker: Option<i64>,
}

/// Split a voice spec into (model path, speaker id): `model.onnx#1` selects
/// speaker 1 of a multi-speaker voice; a bare path means speaker 0. A `#`
/// that isn't followed by a number is treated as part of the path.
pub fn split_speaker(spec: &str) -> (&str, i64) {
    match spec.rsplit_once('#') {
        Some((path, id)) => match id.parse::<i64>() {
            Ok(n) => (path, n),
            Err(_) => (spec, 0),
        },
        None => (spec, 0),
    }
}

impl OnnxTts {
    /// Load a piper `.onnx` voice and its sibling `.onnx.json` once. The spec
    /// may carry a `#N` speaker suffix for multi-speaker models.
    pub fn load(spec: &str) -> Result<Self> {
        let (model_path, sid) = split_speaker(spec);
        // ort's builder error carries the (non-Send) builder back, so it can't go
        // through `?` into anyhow directly — map via Display first. Chained on the
        // temporary builder so no `mut` binding is needed.
        let session = build_session(model_path, Some(session_threads()), "ONNX voice")?;

        let cfg_path = format!("{model_path}.json");
        let cfg: serde_json::Value = serde_json::from_str(
            &std::fs::read_to_string(&cfg_path).with_context(|| format!("reading {cfg_path}"))?,
        )
        .with_context(|| format!("parsing {cfg_path}"))?;

        let sample_rate = cfg["audio"]["sample_rate"]
            .as_u64()
            .ok_or_else(|| anyhow!("{cfg_path}: missing audio.sample_rate"))?
            as u32;

        let map = cfg["phoneme_id_map"]
            .as_object()
            .ok_or_else(|| anyhow!("{cfg_path}: missing phoneme_id_map"))?;
        let phoneme_ids = map
            .iter()
            .map(|(k, v)| {
                let ids = v
                    .as_array()
                    .into_iter()
                    .flatten()
                    .filter_map(serde_json::Value::as_i64)
                    .collect();
                (k.clone(), ids)
            })
            .collect();

        // VITS inference scales, from the voice's json but overridable by env so the
        // voice can be tuned live without re-vendoring the model. `noise_scale` =
        // timbre/pitch variation, `length_scale` = pace (lower is faster/peppier),
        // `noise_w` = rhythm variation. Nudging a flat voice: length down, noise_w up.
        let inf = &cfg["inference"];
        let native_length = inf["length_scale"].as_f64().unwrap_or(1.0) as f32;
        let scales = [
            env_scale(
                "XCOPILOT_TTS_NOISE",
                inf["noise_scale"].as_f64().unwrap_or(0.667),
            ),
            env_scale(
                "XCOPILOT_TTS_LENGTH",
                inf["length_scale"].as_f64().unwrap_or(1.0),
            ),
            env_scale(
                "XCOPILOT_TTS_NOISE_W",
                inf["noise_w"].as_f64().unwrap_or(0.8),
            ),
        ];

        // Multi-speaker voices take a `sid` input; single-speaker models
        // reject it, so only pass one when the model actually hosts a crowd.
        let num_speakers = cfg["num_speakers"].as_i64().unwrap_or(1);
        let speaker = (num_speakers > 1).then_some(sid.clamp(0, num_speakers - 1));

        Ok(Self {
            session,
            phoneme_ids,
            sample_rate,
            scales,
            native_length,
            speaker,
        })
    }

    /// The voice's output sample rate (from its `.onnx.json`) — callers that
    /// prepare audio to splice with this voice's output need it.
    pub fn sample_rate(&self) -> u32 {
        self.sample_rate
    }

    /// Override this voice's VITS scales after load — per-instance, unlike the
    /// `XCOPILOT_TTS_*` env knobs which apply to every voice in the process.
    /// `None` keeps the current value. noise = timbre variation, noise_w =
    /// rhythm variation, both absolute. `length` = pace, lower is faster, and
    /// it is a MULTIPLE of the model's natural pace, never an absolute scale:
    /// models ship their own (`en_GB-vctk-medium` 1.4, `hfc_male` and `sam`
    /// 0.8, the rest 1.0), and the station's absolute 0.8 ran VCTK's
    /// controllers at 0.57 of their natural length - "too fast" at every
    /// audition (2026-09-22). 0.8 now means 20% brisker than natural for
    /// every voice alike.
    pub fn override_scales(
        &mut self,
        noise: Option<f32>,
        length: Option<f32>,
        noise_w: Option<f32>,
    ) {
        if let Some(v) = noise {
            self.scales[0] = v;
        }
        if let Some(v) = length {
            self.scales[1] = self.native_length * v;
        }
        if let Some(v) = noise_w {
            self.scales[2] = v;
        }
    }

    /// Render one line at a tempo: `length` and `noise_w` MULTIPLY the voice's
    /// current scales for this render only, then the voice is put back as it
    /// was (`tempo::line_tempo`: a warning quick and flat, chat looser).
    pub fn render_at_tempo(
        &mut self,
        text: &str,
        length: f32,
        noise_w: f32,
        fx: impl FnOnce(&mut Vec<f32>, u32),
    ) -> Result<Vec<f32>> {
        let saved = self.scales;
        self.scales[1] *= length;
        self.scales[2] *= noise_w;
        let out = self.render_with(text, fx);
        self.scales = saved;
        out
    }

    fn ids_of(&self, key: &str) -> Vec<i64> {
        self.phoneme_ids.get(key).cloned().unwrap_or_default()
    }

    /// `text -> token ids`, mirroring piper: BOS (`^`), then each phoneme followed
    /// by a pad (`_`), then EOS (`$`). Phonemes that aren't in the map are skipped.
    fn token_ids(&self, text: &str) -> Result<Vec<i64>> {
        let phonemes = phonemize(text)?;
        let pad = self.ids_of("_");
        let mut ids = self.ids_of("^");
        for ch in phonemes.chars() {
            if let Some(mapped) = self.phoneme_ids.get(&ch.to_string()) {
                ids.extend_from_slice(mapped);
                ids.extend_from_slice(&pad);
            }
        }
        ids.extend(self.ids_of("$"));
        if ids.len() <= 2 {
            bail!("no phonemes mapped for {text:?} (phonemized to {phonemes:?})");
        }
        Ok(ids)
    }

    /// Synthesize to mono `f32` samples in [-1, 1]. Reuses the warm session.
    pub fn synth(&mut self, text: &str) -> Result<Vec<f32>> {
        let ids = self.token_ids(text)?;
        let n = ids.len() as i64;
        let input = Tensor::from_array(([1_i64, n], ids))?;
        let lengths = Tensor::from_array(([1_i64], vec![n]))?;
        let scales = Tensor::from_array(([3_i64], self.scales.to_vec()))?;

        let outputs = match self.speaker {
            Some(sid) => {
                let sid = Tensor::from_array(([1_i64], vec![sid]))?;
                self.session.run(ort::inputs![
                    "input" => input,
                    "input_lengths" => lengths,
                    "scales" => scales,
                    "sid" => sid,
                ])?
            }
            None => self.session.run(ort::inputs![
                "input" => input,
                "input_lengths" => lengths,
                "scales" => scales,
            ])?,
        };

        let (_shape, data) = outputs[0].try_extract_tensor::<f32>()?;
        Ok(data.to_vec())
    }

    /// Synthesize and run `fx` over the raw samples, returning them WITHOUT
    /// playing: the seam the ATC radio-transmission effect plugs into (see
    /// `radiofx`). The caller plays via [`crate::tts::play_pcm`]; the speech
    /// arbiter keeps the buffer so a PTT press can "step on" the transmission
    /// and the release can play the remainder from the real-time position.
    pub fn render_with(
        &mut self,
        text: &str,
        fx: impl FnOnce(&mut Vec<f32>, u32),
    ) -> Result<Vec<f32>> {
        let mut samples = self.synth_chunked(text)?;
        fx(&mut samples, self.sample_rate);
        Ok(samples)
    }

    /// The whole utterance as the app renders it, without playing it: for
    /// `xvl tts`. Long texts are synthesized sentence by sentence (see
    /// [`speech_chunks`]) and stitched with a short pause - one giant pass
    /// comes out rushed.
    pub fn render(&mut self, text: &str) -> Result<Vec<f32>> {
        self.synth_chunked(text)
    }

    fn synth_chunked(&mut self, text: &str) -> Result<Vec<f32>> {
        // Timed like `stt ms=` — synthesis shares the machine with the sim, so
        // its cost per utterance must be visible in the flight log.
        let t = std::time::Instant::now();
        let gap = vec![0.0_f32; (self.sample_rate as f64 * SENTENCE_GAP_S) as usize];
        let mut samples = Vec::new();
        for chunk in speech_chunks(&undot_acronyms(text)) {
            if !samples.is_empty() {
                samples.extend_from_slice(&gap);
            }
            samples.extend(self.synth(&chunk)?);
        }
        tracing::info!(
            ms = t.elapsed().as_millis() as u64,
            chars = text.len(),
            "tts synth"
        );
        Ok(samples)
    }
}

/// Pause stitched between sentence chunks, seconds — about what a human reader
/// leaves at a full stop.
const SENTENCE_GAP_S: f64 = 0.22;

/// Split text into sentence-sized chunks for synthesis. VITS-family models are
/// trained on single sentences: a whole paragraph in one pass comes out rushed
/// (the duration predictor compresses the pacing — a nine-sentence flight-plan
/// brief was heard live at roughly double speed), and a style table
/// additionally clamps on long inputs. Sentences end at `.`/`?`/`!` followed by
/// whitespace, so decimals ("altimeter 30.17") never split; a sentence still
/// longer than ~220 chars splits again at comma boundaries.
pub(crate) fn speech_chunks(text: &str) -> Vec<String> {
    const MAX_CHARS: usize = 220;
    let mut sentences: Vec<&str> = Vec::new();
    let mut start = 0;
    let mut prev: Option<(usize, char)> = None;
    for (i, c) in text.char_indices() {
        if let Some((pi, pc)) = prev {
            if matches!(pc, '.' | '?' | '!') && c.is_whitespace() {
                sentences.push(&text[start..=pi]);
                start = i;
            }
        }
        prev = Some((i, c));
    }
    sentences.push(&text[start..]);
    let mut out = Vec::new();
    for s in sentences {
        let s = s.trim();
        if s.is_empty() {
            continue;
        }
        if s.len() <= MAX_CHARS {
            out.push(s.to_string());
            continue;
        }
        // Overlong sentence: greedily pack comma-separated pieces up to MAX.
        let mut cur = String::new();
        for piece in s.split_inclusive(", ") {
            if !cur.is_empty() && cur.len() + piece.len() > MAX_CHARS {
                out.push(cur.trim().to_string());
                cur = String::new();
            }
            cur.push_str(piece);
        }
        if !cur.trim().is_empty() {
            out.push(cur.trim().to_string());
        }
    }
    out
}

/// Dump an ONNX model's input/output signature — used to wire up a new model
/// (a model whose I/O differs from piper VITS). `cargo run -- onnx-info <m>`.
pub fn inspect(model_path: &str) -> Result<String> {
    let session = build_session(model_path, None, "model")?;
    let mut s = format!("model: {model_path}\nINPUTS:\n");
    for i in session.inputs() {
        s.push_str(&format!("  {:<12} {:?}\n", i.name(), i.dtype()));
    }
    s.push_str("OUTPUTS:\n");
    for o in session.outputs() {
        s.push_str(&format!("  {:<12} {:?}\n", o.name(), o.dtype()));
    }
    Ok(s)
}

/// Intra-op thread count for the TTS sessions. onnxruntime's default is every
/// physical core with spin-waiting workers — on the machine that is also
/// rendering X-Plane, each readback's synthesis burst was reported live
/// (2026-07-08) as a sim frame-rate dip. Synthesis stays comfortably faster
/// than realtime on a few cores; `XCOPILOT_TTS_THREADS` overrides.
/// ONNX Runtime builds its environment and its graph-optimizer registries
/// lazily, on whichever thread creates the first session — and two threads
/// reaching that at once is a segfault inside the transpose optimizer, not an
/// error:
///
/// ```text
/// EXC_BAD_ACCESS (SIGSEGV) KERN_INVALID_ADDRESS   thread 5: voice-intercom
///   onnx_transpose_optimization::GetHandler(...)
///   onnxruntime::InferenceSession::Initialize()
///   ort::session::builder::...commit_from_file
/// ```
///
/// Seen at startup on 2026-07-28 and gone on the very next run, which is the
/// signature of a race: the intercom, the station radio and the cabin each load
/// their own voice on their own thread, and a boot where two first-utterances
/// land together hits it. Creation is serialised here; inference afterwards is
/// per-session and stays parallel.
static ORT_SESSION_INIT: std::sync::Mutex<()> = std::sync::Mutex::new(());

/// Build an ONNX session, one thread at a time. `threads` is the intra-op
/// thread count, `None` for the runtime's default (the model inspector).
fn build_session(model_path: &str, threads: Option<usize>, what: &str) -> Result<Session> {
    let _guard = ORT_SESSION_INIT
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    let builder = Session::builder().map_err(|e| anyhow!("ort session builder: {e}"))?;
    let mut builder = match threads {
        Some(n) => builder
            .with_intra_threads(n)
            .map_err(|e| anyhow!("ort session threads: {e}"))?,
        None => builder,
    };
    builder
        .commit_from_file(model_path)
        .map_err(|e| anyhow!("loading {what} {model_path}: {e}"))
}

fn session_threads() -> usize {
    std::env::var("XCOPILOT_TTS_THREADS")
        .ok()
        .and_then(|v| v.trim().parse().ok())
        .filter(|&n| n >= 1)
        .unwrap_or(4)
}

/// A VITS scale: the env override (`name`) if set and parseable, else the voice's
/// own value. Lets the voice be tuned from `.env` without editing the model json.
fn env_scale(name: &str, default: f64) -> f32 {
    std::env::var(name)
        .ok()
        .and_then(|v| v.trim().parse::<f32>().ok())
        .unwrap_or(default as f32)
}

/// Spoken names for A–Z, so acronyms spelled letter-by-letter ("A P U", "C H T")
/// are read as letters. espeak-ng otherwise reads a lone "A" as a schwa ("uh"),
/// turning "A P U" into "uh-pee-you" (sounds like "P.U.").
// Spellings chosen so espeak-ng pronounces each as the letter NAME: "eigh" → /eɪ/
// (a plain "ay" comes out /aɪ/ = "I", and "eff" doubles to "ee-eff"), etc.
const LETTER_NAMES: [&str; 26] = [
    "eigh",
    "bee",
    "see",
    "dee",
    "ee",
    "ef",
    "jee",
    "aitch",
    "eye",
    "jay",
    "kay",
    "el",
    "em",
    "en",
    "oh",
    "pee",
    "cue",
    "ar",
    "ess",
    "tee",
    "you",
    "vee",
    "double you",
    "ex",
    "why",
    "zee",
];

/// Rewrite a dotted acronym ("A.P.U.", "A.D.I.R.S.") as the spaced letters the
/// readbacks use ("A P U"). Each dot is a full stop to everything downstream:
/// `speech_chunks` cut "A.P.U. running." into two synths with a sentence gap
/// between them, and `phonemize` put a stop's pause after every letter. The
/// closing dot survives only where it also ends the sentence (end of text, or a
/// capitalised word follows).
fn undot_acronyms(text: &str) -> String {
    let toks: Vec<&str> = text.split(' ').collect();
    let mut out: Vec<String> = Vec::with_capacity(toks.len());
    for (i, tok) in toks.iter().enumerate() {
        let core = tok.trim_end_matches(|c: char| matches!(c, ',' | ';' | ':' | '!' | '?'));
        let b = core.as_bytes();
        let dotted = b.len() >= 3
            && b.iter().enumerate().all(|(j, c)| {
                if j % 2 == 0 {
                    c.is_ascii_uppercase()
                } else {
                    *c == b'.'
                }
            });
        if !dotted {
            out.push(tok.to_string());
            continue;
        }
        let letters: Vec<String> = core
            .chars()
            .filter(|c| *c != '.')
            .map(String::from)
            .collect();
        let mut spoken = letters.join(" ");
        let tail = &tok[core.len()..];
        let ends_sentence = toks[i + 1..]
            .iter()
            .find(|t| !t.is_empty())
            .map_or(true, |next| next.starts_with(|c: char| c.is_uppercase()));
        if core.ends_with('.') && tail.is_empty() && ends_sentence {
            spoken.push('.');
        }
        spoken.push_str(tail);
        out.push(spoken);
    }
    out.join(" ")
}

/// Expand standalone single capital letters to their spoken names (keeping any
/// attached punctuation), so the spelled-out acronym convention in the readbacks
/// is pronounced correctly. Lowercase words and multi-letter tokens are untouched.
fn say_letters(text: &str) -> String {
    text.split(' ')
        .map(|tok| {
            let core = tok.trim_matches(|c: char| !c.is_alphanumeric());
            if core.len() == 1 {
                if let Some(c) = core.chars().next() {
                    if c.is_ascii_uppercase() {
                        return tok.replacen(core, LETTER_NAMES[(c as u8 - b'A') as usize], 1);
                    }
                }
            }
            tok.to_string()
        })
        .collect::<Vec<_>>()
        .join(" ")
}

/// Sentence/clause punctuation that piper voices model as pause/intonation tokens
/// (they're in the voice's phoneme_id_map). `espeak-ng --ipa` collapses these to
/// newlines and drops the mark itself, so we phonemize *between* them and splice
/// the punctuation back — otherwise every readback is a flat, pauseless run-on.
const PUNCT: &[char] = &[',', '.', '?', '!', ';', ':'];

/// Phonemize `text` to an IPA string with punctuation preserved. Each clause is
/// phonemized on its own (no internal punctuation for espeak to swallow), and the
/// original punctuation mark is reinserted between clauses so its prosody survives.
fn phonemize(text: &str) -> Result<String> {
    let text = say_letters(text); // "A P U" -> "ay pee you" before espeak
    let mut out = String::new();
    let mut seg = String::new();
    for ch in text.chars() {
        if PUNCT.contains(&ch) {
            if !seg.trim().is_empty() {
                out.push_str(&espeak_ipa(seg.trim())?);
            }
            seg.clear();
            out.push(' '); // keep the mark a standalone token for the id map
            out.push(ch);
            out.push(' ');
        } else {
            seg.push(ch);
        }
    }
    if !seg.trim().is_empty() {
        out.push_str(&espeak_ipa(seg.trim())?);
    }
    Ok(out.split_whitespace().collect::<Vec<_>>().join(" "))
}

/// Run espeak-ng on a punctuation-free clause and return its IPA phonemes.
fn espeak_ipa(clause: &str) -> Result<String> {
    let out = Command::new("espeak-ng")
        .args(["-v", "en-us", "-q", "--ipa", clause])
        .stderr(Stdio::null())
        .output()
        .context("running espeak-ng (install it: brew install espeak-ng)")?;
    if !out.status.success() {
        bail!("espeak-ng exited with {}", out.status);
    }
    Ok(String::from_utf8_lossy(&out.stdout)
        .replace('\n', " ")
        .trim()
        .to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    const MODEL: &str = "models/en_US-amy-medium.onnx";
    const PHRASES: [&str; 4] = [
        "Flaps up.",
        "Roger, heading two seven zero, autopilot engaged.",
        "Gear is down, three green.",
        "Caution, fuel endurance forty minutes.",
    ];

    #[test]
    fn chunks_split_sentences_not_decimals() {
        // The flight-plan brief that was heard at double speed, verbatim.
        let brief = "Flight plan: C172, LEAL to LERL, alternate LEMD. Cruise 10000 feet, \
                     202 nautical miles, about 1 hours 59 minutes enroute, wind 351 at 5. \
                     Block fuel 38 gallons, 18 for the trip, 6 reserve. You're at 1945, \
                     8055 to climb to cruise. Top of descent about 19 miles before LERL. \
                     You have 27 aboard, but the plan needs 38; short about 10.";
        let chunks = speech_chunks(brief);
        assert_eq!(chunks.len(), 6, "{chunks:#?}");
        assert!(chunks[0].ends_with("alternate LEMD."));
        assert!(chunks.iter().all(|c| c.len() <= 220));

        // Decimals and short readbacks must not split.
        assert_eq!(
            speech_chunks("Roger, altimeter 30.17."),
            vec!["Roger, altimeter 30.17."]
        );
        assert_eq!(speech_chunks("Flaps up."), vec!["Flaps up."]);
        // Question marks end sentences too.
        assert_eq!(speech_chunks("Say again? Flaps up.").len(), 2);
        // An overlong single sentence falls back to comma packing.
        let long = format!("one{}", ", and some more words here".repeat(20));
        assert!(speech_chunks(&long).iter().all(|c| c.len() <= 220));
    }

    #[test]
    fn dotted_acronyms_are_spoken_as_spaced_letters() {
        // The A330 start, as heard: "A.P.U." then a sentence gap then "running."
        assert_eq!(undot_acronyms("A.P.U. running."), "A P U running.");
        assert_eq!(speech_chunks(&undot_acronyms("A.P.U. running.")).len(), 1);
        assert_eq!(undot_acronyms("Starting the A.P.U."), "Starting the A P U.");
        assert_eq!(
            undot_acronyms("Beacon and A.P.U. bleed on."),
            "Beacon and A P U bleed on."
        );
        assert_eq!(
            undot_acronyms("A.D.I.R.S. one is off. Selecting nav."),
            "A D I R S one is off. Selecting nav."
        );
        // The closing dot is kept where it ends the sentence too.
        assert_eq!(
            undot_acronyms("Check the A.P.U. Your aircraft."),
            "Check the A P U. Your aircraft."
        );
        assert_eq!(
            undot_acronyms("No A.P.U., no start."),
            "No A P U, no start."
        );
        // Letters come out as names, never a schwa.
        assert_eq!(
            say_letters(&undot_acronyms("A.P.U. off.")),
            "eigh pee you off."
        );
        // Decimals, lone letters and plain sentences are untouched.
        for s in [
            "Roger, altimeter 30.17.",
            "Runway 27 L.",
            "Flaps up. Gear up.",
            "A P U running.",
        ] {
            assert_eq!(undot_acronyms(s), s);
        }
    }

    // Needs the vendored model + espeak-ng, so off by default. Run with:
    //   cargo test --features onnx-tts -- --ignored --nocapture onnx
    #[test]
    #[ignore]
    fn onnx_warm_vs_cold() {
        let t0 = Instant::now();
        let mut tts = OnnxTts::load(MODEL).expect("load voice");
        let load_ms = t0.elapsed().as_secs_f64() * 1000.0;
        let _ = tts.synth(PHRASES[0]).expect("warm-up synth"); // JIT kernels
        println!("\none-time session load: {load_ms:7.1} ms");

        println!(
            "{:<52}{:>8}{:>10}{:>10}",
            "phrase", "speech", "WARM", "COLD"
        );
        let (mut wsum, mut csum) = (0.0, 0.0);
        for p in PHRASES {
            let t = Instant::now();
            let s = tts.synth(p).expect("warm synth");
            let wms = t.elapsed().as_secs_f64() * 1000.0;
            assert!(
                s.len() as u32 > tts.sample_rate / 5,
                "got <0.2s of audio for {p:?}"
            );

            let t = Instant::now();
            let mut fresh = OnnxTts::load(MODEL).expect("reload");
            fresh.synth(p).expect("cold synth");
            let cms = t.elapsed().as_secs_f64() * 1000.0;

            let dur = s.len() as f64 / tts.sample_rate as f64;
            println!("  {p:<50}{dur:>7.2}s{wms:>8.1}ms{cms:>8.1}ms");
            wsum += wms;
            csum += cms;
        }
        let (wmean, cmean) = (wsum / 4.0, csum / 4.0);
        println!("  {:<50}{:>8}{wmean:>8.1}ms{cmean:>8.1}ms", "mean", "");
        println!(
            "  → cold pays {:.1}x the warm cost (+{:.0} ms reload tax/utterance)\n",
            cmean / wmean,
            cmean - wmean
        );

        // Write one out so the audio can be ear-checked.
        let wav = std::env::temp_dir().join("xvoicelayer-onnx-test.wav");
        let s = tts.synth("Roger, flaps up.").unwrap();
        write_wav(&wav, &s, tts.sample_rate).unwrap();
        println!("  wrote sample: {}", wav.display());
    }
}

/// Write mono `f32` samples as 16-bit PCM WAV.
pub fn write_wav(path: &Path, samples: &[f32], sample_rate: u32) -> Result<()> {
    let spec = hound::WavSpec {
        channels: 1,
        sample_rate,
        bits_per_sample: 16,
        sample_format: hound::SampleFormat::Int,
    };
    let mut w = hound::WavWriter::create(path, spec)
        .with_context(|| format!("creating {}", path.display()))?;
    for &s in samples {
        w.write_sample((s.clamp(-1.0, 1.0) * i16::MAX as f32) as i16)?;
    }
    w.finalize().context("finalizing wav")?;
    Ok(())
}

#[cfg(test)]
mod station_voice_tests {
    /// Dev tool, not an assertion: synthesize the same tower call through every
    /// en_GB voice in models/ at the station's controller cadence, radio effect
    /// applied, and write WAVs to `$TMPDIR/xc-audition/` for listening. Run:
    /// `cargo test --lib audition_station_voices -- --ignored --nocapture`
    #[test]
    #[ignore = "writes audition WAVs from the real models; run manually"]
    fn audition_station_voices() {
        let phrase = "November one seven two Sierra Papa, Gibraltar Tower, \
                      runway zero niner, wind zero niner zero degrees, three knots, \
                      cleared for takeoff.";
        let out_dir = std::env::temp_dir().join("xc-audition");
        std::fs::create_dir_all(&out_dir).expect("mkdir");
        // alan-medium is gone: voted off 2026-07-09 ("read flat"), veto
        // confirmed 2026-07-12 ("NOT THIS GUY"). alba (tower) and northern
        // male (departure) re-confirmed GOOD the same day.
        for (i, model) in ["en_GB-northern_english_male-medium", "en_GB-alba-medium"]
            .iter()
            .enumerate()
        {
            let path = format!("models/{model}.onnx");
            let mut v = super::OnnxTts::load(&path).expect("load voice");
            v.override_scales(None, Some(0.8), Some(1.0)); // station cadence
            let mut s = v.synth(phrase).expect("synth");
            // Production shape: the recorded AFV click when present.
            match crate::radiofx::load_tail_wav("assets/afv/Click_f32.wav", v.sample_rate) {
                Ok(tail) => {
                    crate::radiofx::radioize_with_tail(&mut s, v.sample_rate, i as u32 + 1, &tail)
                }
                Err(_) => crate::radiofx::radioize_variant(&mut s, v.sample_rate, i as u32 + 1),
            }
            let wav = out_dir.join(format!("{model}.wav"));
            super::write_wav(&wav, &s, v.sample_rate).expect("write wav");
            println!("audition: {}", wav.display());
        }
    }

    /// The Spanish-accent roster, exactly as the caster would use it (station
    /// cadence for accents = slower, radio effect applied). One WAV per
    /// candidate in `$TMPDIR/xc-audition/` — listen, then curate
    /// `voice::local_accents_for` by deleting what doesn't survive. Run:
    /// `cargo test --lib audition_spanish_voices -- --ignored --nocapture`
    #[test]
    #[ignore = "writes audition WAVs from the real models; run manually"]
    fn audition_spanish_voices() {
        let phrase = "November one seven two Sierra Papa, Jerez Tower, \
                      runway two zero, wind two one zero degrees, eight knots, \
                      cleared to land.";
        let out_dir = std::env::temp_dir().join("xc-audition");
        std::fs::create_dir_all(&out_dir).expect("mkdir");
        // The 2026-07-12 audition graveyard: davefx, sharvard #0 and #1,
        // es_MX-ald, es_MX-claude — ALL vetoed by ear (the entire Piper
        // Spanish catalogue above low quality). The harness stays for the
        // next candidates; seat survivors in `voice::local_accents_for`.
        let specs: [(&str, &str); 0] = [];
        for (i, (spec, label)) in specs.iter().enumerate() {
            let mut v = match super::OnnxTts::load(spec) {
                Ok(v) => v,
                Err(e) => {
                    println!("skip {label}: {e:#}");
                    continue;
                }
            };
            // Accented voices run SLOWER than the brisk 0.8 (heard live
            // 2026-07-09) — same 1.1 the station caster applies.
            v.override_scales(None, Some(1.1), Some(1.0));
            let mut s = v.synth(phrase).expect("synth");
            match crate::radiofx::load_tail_wav("assets/afv/Click_f32.wav", v.sample_rate) {
                Ok(tail) => {
                    crate::radiofx::radioize_with_tail(&mut s, v.sample_rate, i as u32 + 1, &tail)
                }
                Err(_) => crate::radiofx::radioize_variant(&mut s, v.sample_rate, i as u32 + 1),
            }
            let wav = out_dir.join(format!("{label}.wav"));
            super::write_wav(&wav, &s, v.sample_rate).expect("write wav");
            println!("audition: {}", wav.display());
        }
    }

    /// The European roster, auditioned the way the Spanish one was: each
    /// candidate reads its own country's tower call at the accent cadence the
    /// caster applies (1.1), radio effect on, one WAV each in
    /// `$XCOPILOT_AUDITION_DIR` (default `$TMPDIR/xc-audition/eu`). The line
    /// printed per voice names the English phonemes the model's map LACKS:
    /// those are dropped in silence by `synth`, so a long list is a voice that
    /// will mumble whatever it sounds like. Also a slice of `libritts_r`'s 904
    /// American speakers at the brisk station cadence, for the neutral pool.
    /// Survivors are seated in `voice::local_accents_for`. Run:
    /// `cargo test --release --lib audition_european_voices -- --ignored --nocapture`
    #[test]
    #[ignore = "writes audition WAVs from the real models; run manually"]
    fn audition_european_voices() {
        let call = |tower: &str, rwy: &str, wind: &str| {
            format!(
                "November two four four Kilo Romeo, {tower}, runway {rwy}, \
                 wind {wind} degrees, eight knots, cleared to land."
            )
        };
        let fr = call("Le Bourget Tower", "two seven", "two six zero");
        let be = call("Liege Tower", "zero four right", "zero five zero");
        let nl = call("Schiphol Tower", "one eight right", "two zero zero");
        let de = call("Frankfurt Tower", "two five left", "two four zero");
        let it = call("Roma Tower", "one six right", "one seven zero");
        let pt = call("Lisboa Tower", "zero two", "three five zero");
        let us = call("Kennedy Tower", "three one left", "three zero zero");
        let mut specs: Vec<(String, String, String, f32)> = Vec::new();
        let mut add = |spec: &str, text: &str, pace: f32| {
            let label = spec
                .trim_start_matches("models/")
                .replace(".onnx", "")
                .replace('#', "-s");
            specs.push((spec.to_string(), label, text.to_string(), pace));
        };
        for m in ["fr_FR-siwis-medium", "fr_FR-tom-medium"] {
            add(&format!("models/{m}.onnx"), &fr, 1.1);
        }
        for sid in [0, 1] {
            add(&format!("models/fr_FR-upmc-medium.onnx#{sid}"), &fr, 1.1);
        }
        for sid in [0, 25, 50, 75, 100] {
            add(&format!("models/fr_FR-mls-medium.onnx#{sid}"), &fr, 1.1);
        }
        for m in ["nl_BE-nathalie-medium", "nl_BE-rdh-medium"] {
            add(&format!("models/{m}.onnx"), &be, 1.1);
        }
        for m in [
            "nl_NL-alex-medium",
            "nl_NL-pim-medium",
            "nl_NL-ronnie-medium",
        ] {
            add(&format!("models/{m}.onnx"), &nl, 1.1);
        }
        for sid in [0, 15, 30, 45] {
            add(&format!("models/nl_NL-mls-medium.onnx#{sid}"), &nl, 1.1);
        }
        add("models/de_DE-thorsten-high.onnx", &de, 1.1);
        for sid in [0, 50, 100, 150, 200] {
            add(&format!("models/de_DE-mls-medium.onnx#{sid}"), &de, 1.1);
        }
        for m in ["it_IT-paola-medium", "it_IT-serena-high"] {
            add(&format!("models/{m}.onnx"), &it, 1.1);
        }
        add("models/pt_PT-tugao-medium.onnx", &pt, 1.1);
        for sid in [0, 90, 180, 270, 360, 450, 540, 630, 720, 810] {
            add(
                &format!("models/en_US-libritts_r-medium.onnx#{sid}"),
                &us,
                0.8,
            );
        }

        let out_dir = std::env::var("XCOPILOT_AUDITION_DIR")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|_| std::env::temp_dir().join("xc-audition").join("eu"));
        std::fs::create_dir_all(&out_dir).expect("mkdir");
        for (i, (spec, label, text, pace)) in specs.iter().enumerate() {
            let mut v = match super::OnnxTts::load(spec) {
                Ok(v) => v,
                Err(e) => {
                    println!("skip {label}: {e:#}");
                    continue;
                }
            };
            let ipa = super::phonemize(text).expect("phonemize");
            let mut missing: Vec<char> = ipa
                .chars()
                .filter(|c| !c.is_whitespace() && !v.phoneme_ids.contains_key(&c.to_string()))
                .collect();
            missing.sort_unstable();
            missing.dedup();
            v.override_scales(None, Some(*pace), Some(1.0));
            let mut s = v.synth(text).expect("synth");
            match crate::radiofx::load_tail_wav("assets/afv/Click_f32.wav", v.sample_rate) {
                Ok(tail) => {
                    crate::radiofx::radioize_with_tail(&mut s, v.sample_rate, i as u32 + 1, &tail)
                }
                Err(_) => crate::radiofx::radioize_variant(&mut s, v.sample_rate, i as u32 + 1),
            }
            let wav = out_dir.join(format!("{label}.wav"));
            super::write_wav(&wav, &s, v.sample_rate).expect("write wav");
            let missing: String = missing.into_iter().collect();
            println!(
                "audition: {label:36} missing=[{missing}]  {}",
                wav.display()
            );
        }
    }

    #[test]
    fn speaker_specs_split_cleanly() {
        assert_eq!(super::split_speaker("models/v.onnx"), ("models/v.onnx", 0));
        assert_eq!(
            super::split_speaker("models/v.onnx#1"),
            ("models/v.onnx", 1)
        );
        assert_eq!(
            super::split_speaker("models/v.onnx#0"),
            ("models/v.onnx", 0)
        );
        // A '#' that isn't a speaker id stays part of the path.
        assert_eq!(
            super::split_speaker("models/v#a.onnx"),
            ("models/v#a.onnx", 0)
        );
    }

    /// Reproduces the live 2026-07-07 station-voice shape: the copilot's Ryan
    /// session already warm, then the station voice (Alba, the shipping
    /// default) loaded as a SECOND session in the same process and spoken
    /// through the radio effect — exactly what the speaker thread does on the
    /// first tower transmission.
    #[test]
    #[ignore = "loads the real voice models; run manually with --ignored"]
    fn second_onnx_session_station_voice_with_radio_effect() {
        let mut ryan = super::OnnxTts::load("models/en_US-ryan-high.onnx").expect("load ryan");
        let r = ryan.synth("Copy, flaps ten.").expect("ryan synth");
        assert!(!r.is_empty());

        let mut alba = super::OnnxTts::load("models/en_GB-alba-medium.onnx").expect("load alba");
        alba.override_scales(None, Some(0.8), Some(1.0));
        let mut a = alba
            .synth("November one seven two, Gibraltar tower, reading you five.")
            .expect("alba synth");
        assert!(!a.is_empty());
        crate::radiofx::radioize(&mut a, alba.sample_rate);
        assert!(!a.is_empty());
    }

    /// Real synthesis through the multi-speaker path — the `#N` suffix must
    /// select a DIFFERENT voice, not silently fall back to speaker 0.
    /// Ignored by default: needs the voice files present.
    #[test]
    #[ignore]
    fn multi_speaker_voices_render_and_differ() {
        let base = "models/en_GB-vctk-medium.onnx";
        if !std::path::Path::new(base).exists() {
            eprintln!("no vctk model, skipping");
            return;
        }
        let say = |spec: &str| {
            let mut tts = super::OnnxTts::load(spec).expect("loads");
            tts.synth("Southampton Tower, good afternoon.")
                .expect("renders")
        };
        let a = say(&format!("{base}#4"));
        let b = say(&format!("{base}#33"));
        let c = say(&format!("{base}#80"));
        for (n, v) in [("4", &a), ("33", &b), ("80", &c)] {
            assert!(
                v.len() > 8000,
                "speaker {n} produced only {} samples",
                v.len()
            );
        }
        // Different speakers must produce genuinely different audio.
        let same =
            |x: &Vec<f32>, y: &Vec<f32>| x.len() == y.len() && x.iter().zip(y).all(|(p, q)| p == q);
        assert!(!same(&a, &b), "speaker 4 and 33 rendered identically");
        assert!(!same(&b, &c), "speaker 33 and 80 rendered identically");
    }
}
