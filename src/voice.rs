//! Speech arbiter — the single owner of the TTS voice.
//!
//! Both the command loop (readbacks) and, later, the monitor (alerts) need to
//! speak, from different threads. If they each called [`tts::speak`] directly
//! they'd talk over each other. So all speech goes through one [`Voice`]: a
//! dedicated speaker thread draining a priority queue, so utterances are
//! serialized and never overlap.
//!
//! Priority orders the *queue*, not interruption: a higher-priority utterance
//! jumps ahead of whatever is still waiting, but never cuts off speech already
//! playing (`say`/piper aren't cheaply interruptible — true cut-off is a later
//! iteration). Within a priority, order is FIFO.
//!
//! Two ways to speak:
//!   - [`Voice::say`]   — blocking. The command loop must not resume listening
//!     until the readback has finished (else it captures its own TTS).
//!   - [`Voice::alert`] — fire-and-forget. The monitor queues and moves on.

use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::sync::{Arc, Condvar, Mutex};
use std::time::Instant;

use crate::audio::MicGate;
use crate::config::Config;
use crate::tts;

/// How long a spoken phrase stays on the echo watch-list. Long enough to cover a
/// readback bleeding back after a pause, short enough not to gag a genuine repeat.
const ECHO_MEMORY: std::time::Duration = std::time::Duration::from_secs(12);
/// Word-overlap at or above this against something just said = treat as self-echo.
/// Tuned so a full readback coming back scores high while a partial overlap (an
/// echo fragment glued to a real command) stays below and is still acted on.
const ECHO_SIMILARITY: f64 = 0.55;

/// Speech priority. Declaration order *is* the ranking (low → high); a max-heap
/// pops the highest first. Warning preempts Caution preempts a command readback
/// preempts an Advisory.
///
/// `Advisory`/`Caution`/`Warning` are the monitor's tiers (wired up in the
/// monitoring milestone); the command loop uses `Readback`.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
#[allow(dead_code)] // Advisory/Caution/Warning are used by the monitor (next milestone)
pub enum Priority {
    Advisory,
    Readback,
    Caution,
    Warning,
}

/// The LOCAL accent for a country, by ICAO prefix — the voice its towers and
/// ATIS speak English with. Deliberately strict about geography: Spain (LE)
/// and the Canaries (GC) get the Castilian es_ES voice, and the es_MX voices
/// serve only Spanish-speaking Latin America — a Madrid tower must not sound
/// like Mexico City ("no mexicans in Spain", 2026-07-09). Fields with no
/// mapping cast entirely from the neutral pool.
pub fn local_accent_for(icao: &str) -> Option<&'static str> {
    let p2 = icao.get(0..2).unwrap_or("");
    match p2 {
        // Spain + Canaries: Castilian-accented English.
        "LE" | "GC" => Some("models/es_ES-davefx-medium.onnx"),
        // Spanish-speaking Latin America (Mexico, Colombia, Ecuador, Peru,
        // Chile, Argentina/Uruguay/Paraguay/Bolivia share the SA/S* prefixes
        // loosely — Brazil (SB/SD/SN/SS/SW) is Portuguese and stays neutral).
        "MM" | "SK" | "SE" | "SP" | "SC" | "SA" | "SU" | "SG" | "SL" | "SV" => {
            Some("models/es_MX-claude-high.onnx")
        }
        _ => None,
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

/// One-shot latch a blocking caller waits on until its utterance is spoken.
struct Latch {
    spoken: Mutex<bool>,
    cv: Condvar,
}

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
    /// The Piper model for this position. Seed 0 (the configured home field)
    /// uses the curated per-position voices; any other seed draws a stable
    /// per-airport crew from `atc_voice_pool`, offset per position so the
    /// controllers at one field are distinct people. An empty pool always
    /// falls back to the curated voices.
    fn model(self, config: &Config, seed: usize, local: Option<&str>) -> String {
        // A ghost pilot's voice comes from ITS OWN seed (per callsign), not
        // the airport's cast — pilots are from everywhere, and the same
        // flight keeps the same voice across calls.
        if let StationSpeaker::Ghost { seed: ghost } = self {
            let pool: Vec<&str> = config
                .atc_voice_pool
                .split(',')
                .map(str::trim)
                .filter(|m| !m.is_empty())
                .collect();
            if !pool.is_empty() {
                return pool[ghost % pool.len()].to_string();
            }
            return config.atc_piper_voice.clone();
        }
        if seed != 0 {
            // The country's own accent mans the tower and reads the ATIS
            // (one recording voice per field is exactly how ATIS sounds);
            // approach/departure fill from the neutral pool.
            if let Some(local) = local {
                if matches!(self, StationSpeaker::Tower | StationSpeaker::Atis) {
                    return local.to_string();
                }
            }
            let pool: Vec<&str> = config
                .atc_voice_pool
                .split(',')
                .map(str::trim)
                .filter(|m| !m.is_empty())
                .collect();
            if !pool.is_empty() {
                // Per-position step sized so all four slots land on distinct
                // entries whenever the pool has at least four voices.
                let step = (pool.len() / 4).max(1);
                return pool[(seed + self.slot() * step) % pool.len()].to_string();
            }
        }
        let fallback = config.atc_piper_voice.as_str();
        match self {
            // Handled above (own-seed pool pick); reachable only with an
            // empty pool, where everyone shares the curated station voice.
            StationSpeaker::Ghost { .. } => fallback,
            StationSpeaker::Tower => fallback,
            StationSpeaker::Approach => {
                let v = config.atc_approach_piper_voice.as_str();
                if v.is_empty() { fallback } else { v }
            }
            StationSpeaker::Departure => {
                let v = config.atc_departure_piper_voice.as_str();
                if v.is_empty() { fallback } else { v }
            }
            StationSpeaker::Atis => {
                let v = config.atc_atis_piper_voice.as_str();
                if v.is_empty() { fallback } else { v }
            }
        }
        .to_string()
    }
    fn slot(self) -> usize {
        match self {
            StationSpeaker::Tower => 0,
            StationSpeaker::Approach => 1,
            StationSpeaker::Departure => 2,
            StationSpeaker::Atis => 3,
            // One warm slot for whichever ghost spoke last: the cache is
            // keyed by model name, so a different ghost voice rebuilds it.
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

/// A station transmission the pilot keyed over, with everything needed to let
/// it keep "running" silently while PTT is down. Half-duplex radio done
/// honestly: transmitting blocks your own receiver, it does not pause the
/// controller. On release the remainder plays from wherever real time has
/// reached (mid-word, like rejoining a real frequency); fully elapsed means
/// it is simply gone, and recovery stays procedural ("say again", readback
/// enforcement), never an automatic repeat. Copilot speech is exempt by
/// design: interrupting your crew is a decision, not physics, and a barged-in
/// readback is stale the moment you key up.
struct SteppedOn {
    /// The full processed sample buffer (radio effect and squelch included).
    samples: Vec<f32>,
    rate: u32,
    /// When the ORIGINAL playback started: the transmission's real-time clock,
    /// preserved across re-presses so a second interruption resumes further in.
    started: Instant,
    speaker: StationSpeaker,
    text: String,
    /// Original queue position, so the requeued tail sorts ahead of everything
    /// enqueued while PTT was held (it was in-flight; it owns that slot).
    priority: Priority,
    seq: u64,
}

/// Where a stepped-on transmission's real-time clock has reached, as a sample
/// offset into its buffer. `None` once it has (nearly) fully elapsed: under
/// 50 ms left is just the squelch stub, not worth keying the speakers for.
/// Pure, so the barge-in math is testable without audio.
fn tail_offset(elapsed_s: f32, len: usize, rate: u32) -> Option<usize> {
    let offset = (elapsed_s.max(0.0) as f64 * rate as f64) as usize;
    (offset + rate as usize / 20 < len).then_some(offset)
}

/// Short linear ramp (~10 ms) so a mid-word entry opens like squelch breaking,
/// not a click.
fn fade_in(samples: &mut [f32], rate: u32) {
    let n = (rate as usize / 100).clamp(1, samples.len());
    for (i, s) in samples[..n].iter_mut().enumerate() {
        *s *= i as f32 / n as f32;
    }
}

struct Utterance {
    priority: Priority,
    /// Monotonic enqueue order, for FIFO tie-breaking within a priority.
    seq: u64,
    text: String,
    /// Play the attention chime just before speaking this one (reminders).
    chime: bool,
    /// A tone-only utterance (empty `text`): play the negative tone and speak
    /// nothing. Used for the "heard you, didn't recognize it" acknowledgement.
    negative: bool,
    /// Speak as an ATC controller instead of the copilot: a warm ONNX voice
    /// per position (tower/approach/departure) with the radio-transmission
    /// effect, falling back to macOS `say`. Going through the same queue keeps
    /// every voice arbitrated: they can never talk over each other, and the
    /// mic stays gated across all of them.
    station: Option<StationSpeaker>,
    /// A stepped-on transmission rejoining the air: pre-rendered samples played
    /// from the real-time position instead of synthesizing `text` again.
    tail: Option<Arc<SteppedOn>>,
    /// Present for blocking callers; signalled once the utterance is spoken.
    done: Option<Arc<Latch>>,
}

// Order by priority (higher first), then by *earlier* seq first. BinaryHeap is a
// max-heap, so we invert seq (smaller seq must compare "greater") to get FIFO.
impl Ord for Utterance {
    fn cmp(&self, other: &Self) -> Ordering {
        self.priority
            .cmp(&other.priority)
            .then_with(|| other.seq.cmp(&self.seq))
    }
}
impl PartialOrd for Utterance {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl PartialEq for Utterance {
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority && self.seq == other.seq
    }
}
impl Eq for Utterance {}

struct Queue {
    heap: BinaryHeap<Utterance>,
    seq: u64,
    shutdown: bool,
}

struct Inner {
    queue: Mutex<Queue>,
    /// Signalled when work is queued (or on shutdown) to wake the speaker.
    cv: Condvar,
    /// Voice seed for the ACTIVE airport's controllers: 0 = the configured
    /// per-position voices (the curated home field); anything else indexes the
    /// voice pool per position, so each airport gets its own stable crew of
    /// controllers ("we need a different voice for Ciudad Real", 2026-07-09).
    /// Changing it drops the warm station sessions on the next utterance.
    station_seed: std::sync::atomic::AtomicUsize,
    /// The active airport's LOCAL accent voice, when its country has one
    /// (see [`local_accent_for`]): cast as the tower and the ATIS reader.
    station_local: Mutex<Option<String>>,
    /// PTT is held: EVERY voice stays quiet — copilot, controllers, ghosts.
    /// You don't talk over the pilot's transmission; the queue holds and
    /// resumes on release (2026-07-10: the party line yakked through PTT).
    hold: std::sync::atomic::AtomicBool,
    /// The station transmission currently on the air, published just before
    /// playback starts and cleared when it ends. `shutup()` steals it so a
    /// PTT press "steps on" the transmission instead of erasing it.
    now_playing: Mutex<Option<Arc<SteppedOn>>>,
    /// The transmission the last PTT press stepped on, consumed at release:
    /// its remainder requeues if real time hasn't outrun it (see [`SteppedOn`]).
    stepped_on: Mutex<Option<Arc<SteppedOn>>>,
    /// What the copilot has spoken recently, with when it finished. The listener
    /// checks heard transcripts against this to reject its own voice coming back
    /// over the speakers (echo) rather than acting on it.
    spoken: Mutex<Vec<(Instant, String)>>,
}

/// Handle to the speech arbiter. Cheap to clone — hand one to every thread that
/// needs to speak.
#[derive(Clone)]
pub struct Voice {
    inner: Arc<Inner>,
    gate: MicGate,
}

impl Voice {
    /// Start the arbiter: spawns the speaker thread that owns TTS for the
    /// process. `config` is moved into that thread (it selects the TTS backend).
    pub fn new(config: Config) -> Self {
        let inner = Arc::new(Inner {
            queue: Mutex::new(Queue {
                heap: BinaryHeap::new(),
                seq: 0,
                shutdown: false,
            }),
            cv: Condvar::new(),
            station_seed: std::sync::atomic::AtomicUsize::new(0),
            station_local: Mutex::new(None),
            hold: std::sync::atomic::AtomicBool::new(false),
            now_playing: Mutex::new(None),
            stepped_on: Mutex::new(None),
            spoken: Mutex::new(Vec::new()),
        });
        let gate = MicGate::new();
        let speaker = inner.clone();
        let speaker_gate = gate.clone();
        std::thread::Builder::new()
            .name("voice-speaker".into())
            .spawn(move || speaker_loop(&speaker, config, speaker_gate))
            .expect("spawn voice speaker thread");
        Voice { inner, gate }
    }

    /// Cast the active airport's controllers: seed 0 = the configured
    /// per-position voices (the curated home field); any other seed picks a
    /// stable per-airport crew from the voice pool, with `local` (the
    /// country's accent, when it has one) seated as the tower and the ATIS
    /// reader. The warm sessions rebuild on the next station utterance.
    pub fn set_station_cast(&self, seed: usize, local: Option<String>) {
        *self
            .inner
            .station_local
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = local;
        self.inner
            .station_seed
            .store(seed, std::sync::atomic::Ordering::Relaxed);
    }

    /// The mic gate this arbiter raises while speaking — hand it to the listener
    /// so it can mute itself and the copilot never hears its own voice.
    pub fn mic_gate(&self) -> MicGate {
        self.gate.clone()
    }

    /// True while the copilot is speaking — the barge-in watcher only interrupts
    /// when there's actually audio to cut.
    pub fn is_speaking(&self) -> bool {
        self.gate.is_speaking()
    }

    /// Barge-in: stop talking *now*. Drop everything queued (unblocking any caller
    /// waiting on a blocking `say`) and kill the audio currently playing. Used when
    /// the pilot keys PTT or interrupts — the copilot must yield, not talk over them.
    /// Hold or release ALL speech (copilot, controllers, ghosts) — raised
    /// while the pilot's PTT is down: nobody talks over a transmission.
    /// Held utterances stay queued and play on release.
    pub fn set_hold(&self, held: bool) {
        self.inner
            .hold
            .store(held, std::sync::atomic::Ordering::Relaxed);
        if !held {
            // A transmission the press stepped on kept "running" silently. If
            // real time hasn't outrun it, requeue its remainder with the
            // ORIGINAL seq: it was in-flight, so it still sorts ahead of
            // everything enqueued during the hold. Expired = gone, like a
            // real stepped-on call; "say again" is the recovery.
            let stepped = self
                .inner
                .stepped_on
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .take();
            if let Some(t) = stepped {
                if tail_offset(t.started.elapsed().as_secs_f32(), t.samples.len(), t.rate)
                    .is_some()
                {
                    let mut q = self.inner.queue.lock().unwrap();
                    q.heap.push(Utterance {
                        priority: t.priority,
                        seq: t.seq,
                        text: t.text.clone(),
                        chime: false,
                        negative: false,
                        station: Some(t.speaker),
                        tail: Some(t),
                        done: None,
                    });
                }
            }
            self.inner.cv.notify_one(); // wake the speaker to drain the backlog
        }
    }

    pub fn shutup(&self) {
        // Steal the on-air station transmission (if any) before killing its
        // audio: the pilot stepped on it, they didn't erase it. Its clock
        // (`started`) keeps running while PTT is down.
        *self
            .inner
            .stepped_on
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = self
            .inner
            .now_playing
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .take();
        {
            let mut q = self.inner.queue.lock().unwrap();
            for utt in q.heap.drain() {
                if let Some(latch) = utt.done {
                    *latch.spoken.lock().unwrap() = true;
                    latch.cv.notify_all();
                }
            }
        }
        // The in-flight utterance (already popped by the speaker) ends when its
        // audio is killed: speak() returns and the speaker signals its latch.
        crate::tts::stop_playback();
        self.gate.set_speaking(false);
    }

    /// True if `heard` closely matches something the copilot said in the last few
    /// seconds — i.e. its own readback bleeding back through the speakers. The
    /// command loop drops such transcripts instead of acting on them.
    pub fn is_self_echo(&self, heard: &str) -> bool {
        let said = self.inner.spoken.lock().unwrap();
        // Long heard text that's mostly a subset of a recent readback is a partial
        // echo (Jaccard is diluted by the readback's extra words). The word-count
        // floor keeps a short real command ("heading 180") from being swallowed.
        let long = crate::phrase::word_count(heard) >= 5;
        said.iter().filter(|(t, _)| t.elapsed() < ECHO_MEMORY).any(|(_, text)| {
            crate::phrase::word_similarity(heard, text) >= ECHO_SIMILARITY
                || (long && crate::phrase::word_containment(heard, text) >= 0.85)
        })
    }

    fn enqueue(&self, priority: Priority, text: String, done: Option<Arc<Latch>>, chime: bool) {
        // No em/en dashes reach the TTS — they trip it up. One chokepoint for
        // every spoken utterance (readbacks, monitor callouts, prompts).
        let text = crate::phrase::scrub_dashes(&text);
        self.push(Utterance { priority, seq: 0, text, chime, negative: false, station: None, tail: None, done });
    }

    fn enqueue_station(
        &self,
        priority: Priority,
        text: String,
        done: Option<Arc<Latch>>,
        speaker: StationSpeaker,
    ) {
        let text = crate::phrase::scrub_dashes(&text);
        tracing::info!(transmitting = %text, "station");
        self.push(Utterance { priority, seq: 0, text, chime: false, negative: false, station: Some(speaker), tail: None, done });
    }

    /// Push a fully-formed utterance onto the queue (stamping its FIFO seq under
    /// the lock). The single enqueue chokepoint for both speech and tones.
    fn push(&self, mut utt: Utterance) {
        let mut q = self.inner.queue.lock().unwrap();
        utt.seq = q.seq;
        q.seq += 1;
        q.heap.push(utt);
        drop(q);
        self.inner.cv.notify_one();
    }

    /// Play the negative tone (no speech) and block until it finishes — the
    /// "heard you, but that isn't a command I recognize" acknowledgement. Blocks
    /// like [`Voice::say`] so the mic stays gated over the tone and the loop
    /// doesn't capture it before resuming.
    pub fn negative(&self) {
        let latch = Arc::new(Latch {
            spoken: Mutex::new(false),
            cv: Condvar::new(),
        });
        self.push(Utterance {
            priority: Priority::Readback,
            seq: 0,
            text: String::new(),
            chime: false,
            negative: true,
            station: None,
            tail: None,
            done: Some(latch.clone()),
        });
        let mut spoken = latch.spoken.lock().unwrap();
        while !*spoken {
            spoken = latch.cv.wait(spoken).unwrap();
        }
    }

    /// Queue an utterance and return immediately (monitor alerts).
    #[allow(dead_code)] // used by the monitor (next milestone)
    pub fn alert(&self, priority: Priority, text: impl Into<String>) {
        self.enqueue(priority, text.into(), None, false);
    }

    /// Queue an ATC-station utterance and return immediately. Used by background
    /// controller callouts (vectors) so they keep the station voice/radio effect
    /// without blocking the monitor loop.
    pub fn alert_station(
        &self,
        priority: Priority,
        speaker: StationSpeaker,
        text: impl Into<String>,
    ) {
        self.enqueue_station(priority, text.into(), None, speaker);
    }

    /// Queue an utterance preceded by the attention chime, and return immediately.
    /// Used by reminders: a short beep, then the heads-up.
    pub fn chime(&self, priority: Priority, text: impl Into<String>) {
        self.enqueue(priority, text.into(), None, true);
    }

    /// Queue an utterance and block until it has been spoken (command readbacks:
    /// the loop must wait so it doesn't listen over its own voice).
    pub fn say(&self, priority: Priority, text: impl Into<String>) {
        let latch = Arc::new(Latch {
            spoken: Mutex::new(false),
            cv: Condvar::new(),
        });
        self.enqueue(priority, text.into(), Some(latch.clone()), false);
        let mut spoken = latch.spoken.lock().unwrap();
        while !*spoken {
            spoken = latch.cv.wait(spoken).unwrap();
        }
    }

    /// Queue an utterance as the ATC STATION and block until spoken: the
    /// station's warm ONNX voice with the radio-transmission effect (falling
    /// back to macOS `say`). Same queue, same mic gate, same echo memory as
    /// the copilot's own voice — the station can never talk over the copilot,
    /// and its audio bleeding back through the mic is rejected the same way.
    pub fn say_station(
        &self,
        priority: Priority,
        speaker: StationSpeaker,
        text: impl Into<String>,
    ) {
        let latch = Arc::new(Latch {
            spoken: Mutex::new(false),
            cv: Condvar::new(),
        });
        let text = crate::phrase::scrub_dashes(&text.into());
        // To the FILE log, not just the TUI: the 2026-07-07 hang was blind
        // because tower speech only reached the dashboard.
        self.enqueue_station(priority, text, Some(latch.clone()), speaker);
        let mut spoken = latch.spoken.lock().unwrap();
        while !*spoken {
            spoken = latch.cv.wait(spoken).unwrap();
        }
    }
}

/// The ATC station's voice: a second warm ONNX session with the COM-radio
/// transmission effect, or the `say` fallback when the model is unset/broken.
/// Built lazily on the first station utterance, so a flight that never talks
/// to ATC never pays for the extra session.
enum StationVoice {
    #[cfg(feature = "onnx-tts")]
    Onnx {
        engine: Box<crate::tts_onnx::OnnxTts>,
        /// Recorded squelch transient (VATSIM-style, e.g. the AFV click),
        /// mono at the engine's rate. `None` = synthesized burst.
        tail: Option<Vec<f32>>,
        /// Counter so each reply gets a fresh radio-effect variant — real
        /// squelch never sounds the same twice.
        transmissions: u32,
    },
    Say(Config),
}

impl StationVoice {
    fn build(config: &Config, model: &str) -> Self {
        #[cfg(feature = "onnx-tts")]
        if !model.is_empty() {
            match crate::tts_onnx::OnnxTts::load(model) {
                Ok(mut engine) => {
                    // Controller cadence: brisker pace and more rhythm variation
                    // than the voice's storybook defaults — alan-medium at stock
                    // length_scale 1.0 was heard live as "drone like and slow".
                    // ACCENTED voices run SLOWER than natural instead: davefx
                    // was "a bit fast" at the brisk 0.8 and still fast at a
                    // natural-ish 0.95 (both heard live 2026-07-09) — through
                    // a radio effect, an accent needs a measured pace to stay
                    // intelligible.
                    let pace = if model.contains("/es_") { 1.1 } else { 0.8 };
                    engine.override_scales(
                        None,
                        Some(env_f32("XCOPILOT_ATC_TTS_LENGTH", pace)),
                        Some(env_f32("XCOPILOT_ATC_TTS_NOISE_W", 1.0)),
                    );
                    // Recorded squelch, resampled once to the voice's rate.
                    // Missing/broken file degrades to the synthesized burst.
                    let tail = (!config.atc_squelch_wav.is_empty())
                        .then(|| {
                            crate::radiofx::load_tail_wav(
                                &config.atc_squelch_wav,
                                engine.sample_rate(),
                            )
                        })
                        .and_then(|r| match r {
                            Ok(t) => Some(t),
                            Err(e) => {
                                tracing::warn!(
                                    "squelch wav unusable ({e:#}); synthesized tail instead"
                                );
                                None
                            }
                        });
                    tracing::info!(
                        model = %model,
                        squelch = if tail.is_some() { "recorded" } else { "synthesized" },
                        "station voice: warm ONNX session + radio effect"
                    );
                    return StationVoice::Onnx { engine: Box::new(engine), tail, transmissions: 0 };
                }
                Err(e) => {
                    tracing::warn!("station voice load failed ({e:#}); falling back to `say`");
                }
            }
        }
        StationVoice::Say(Config {
            tts_backend: "say".into(),
            say_voice: config.atc_voice.clone(),
            say_rate: config.atc_rate.clone(),
            ..Config::default()
        })
    }

    /// Speak `text` as this station. `on_start` fires with the fully processed
    /// sample buffer just before playback begins: the arbiter publishes it as
    /// the on-air transmission so a PTT press can step on it and the release
    /// can play the remainder (ONNX only; the `say` fallback has no buffer to
    /// resume, so a stepped-on call there is simply lost, as before).
    fn speak(&mut self, text: &str, on_start: impl FnOnce(&[f32], u32)) -> anyhow::Result<()> {
        match self {
            #[cfg(feature = "onnx-tts")]
            StationVoice::Onnx { engine, tail, transmissions } => {
                *transmissions = transmissions.wrapping_add(1);
                let variant = *transmissions;
                let tail = tail.clone();
                let samples = engine.render_with(text, move |s, r| match &tail {
                    Some(t) => crate::radiofx::radioize_with_tail(s, r, variant, t),
                    None => crate::radiofx::radioize_variant(s, r, variant),
                })?;
                on_start(&samples, engine.sample_rate());
                tts::play_pcm(&samples, engine.sample_rate())
            }
            StationVoice::Say(config) => {
                let _ = on_start;
                tts::speak(config, text)
            }
        }
    }
}

/// How the speaker thread turns text into sound. Built once, reused for every
/// utterance — which is what lets the in-process ONNX backend keep its model
/// session warm (the subprocess `say`/`piper` paths are stateless per call).
enum Speaker {
    /// Shell out per utterance: `say` or `piper` (see [`tts::speak`]).
    Subprocess(Config),
    /// In-process warm ONNX session (feature `onnx-tts`, `XCOPILOT_TTS=onnx`) — the
    /// default voice backend.
    #[cfg(feature = "onnx-tts")]
    Onnx(Box<crate::tts_onnx::OnnxTts>),
}

impl Speaker {
    fn build(mut config: Config) -> Self {
        #[cfg(feature = "onnx-tts")]
        if config.tts_backend == "onnx" {
            match crate::tts_onnx::OnnxTts::load(&config.piper_voice) {
                Ok(engine) => {
                    tracing::info!(model = %config.piper_voice, "TTS: warm in-process ONNX session");
                    return Speaker::Onnx(Box::new(engine));
                }
                Err(e) => {
                    tracing::error!("onnx TTS load failed ({e:#}); falling back to `say`");
                    config.tts_backend = "say".into();
                }
            }
        }
        // `onnx` requested but the feature isn't built in → fall back to `say`.
        if config.tts_backend == "onnx" {
            tracing::warn!("onnx TTS requested but not built in (need the onnx-tts feature); using `say`");
            config.tts_backend = "say".into();
        }
        // Always announce the resolved backend so the active voice is never a
        // guess — the `say`/`piper` subprocess paths were previously silent.
        tracing::info!(backend = %config.tts_backend, "TTS: subprocess backend (per-utterance)");
        Speaker::Subprocess(config)
    }

    fn speak(&mut self, text: &str) -> anyhow::Result<()> {
        match self {
            Speaker::Subprocess(config) => tts::speak(config, text),
            #[cfg(feature = "onnx-tts")]
            Speaker::Onnx(engine) => engine.speak(text),
        }
    }
}

/// The speaker thread: pop the highest-priority utterance (blocking until one is
/// available), speak it to completion, signal any blocked caller, repeat.
///
/// `gate` is raised for the whole speaking run and only lowered once the queue
/// drains — so a burst of utterances stays gated end to end, with no momentary
/// gap between them for the mic to slip through.
fn speaker_loop(inner: &Arc<Inner>, config: Config, gate: MicGate) {
    let chime_path = resolve_chime(&config.chime_path);
    let negative_path = resolve_negative(&config.negative_tone_path);
    let station_cfg = config.clone();
    // One lazy warm voice per position (tower/approach/departure/ATIS, plus
    // one slot for whichever GHOST pilot spoke last), keyed by model name so
    // a cast change — or a different ghost — rebuilds only that slot. A
    // position that never speaks never loads its model.
    let mut stations: [Option<(String, StationVoice)>; 5] = [None, None, None, None, None];
    let mut stations_seed: usize = 0;
    let mut speaker = Speaker::build(config);
    loop {
        let mut q = inner.queue.lock().unwrap();
        let utt = loop {
            if q.shutdown {
                gate.set_speaking(false);
                return;
            }
            // PTT held: nobody speaks — the queue waits for the release
            // (the wait below re-checks on the timeout).
            if !inner.hold.load(std::sync::atomic::Ordering::Relaxed) {
                if let Some(u) = q.heap.pop() {
                    break u;
                }
            }
            // Nothing to say — we're done speaking; let the mic listen again.
            gate.set_speaking(false);
            q = inner.cv.wait(q).unwrap();
        };
        drop(q); // release the queue while we speak (which is slow)

        gate.set_speaking(true);
        // Tone-only: play the negative "unrecognized" cue and skip TTS entirely
        // (nothing to speak, nothing to add to the echo list).
        // A killed player (SIGTERM) is an intentional barge-in cut-off, not a
        // failure — same treatment as the main TTS path below, which the tone
        // and chime never got (they warned on every interrupted beep).
        let log_play_err = |what: &str, e: anyhow::Error| {
            let msg = format!("{e:#}");
            if msg.contains("SIGTERM") || msg.contains("signal: 15") {
                tracing::debug!("{what} interrupted (barge-in): {msg}");
            } else {
                tracing::warn!("{what} failed: {msg}");
            }
        };
        // A stepped-on station transmission rejoining the air: play whatever
        // remains from the REAL-TIME position (it kept running while the pilot
        // transmitted). Queue delay may have outrun it by now; then it's
        // silently gone, exactly like the real thing. Republished as on-air
        // with its ORIGINAL clock, so a second press resumes further along.
        if let Some(tail) = utt.tail {
            if let Some(offset) =
                tail_offset(tail.started.elapsed().as_secs_f32(), tail.samples.len(), tail.rate)
            {
                let mut rest = tail.samples[offset..].to_vec();
                fade_in(&mut rest, tail.rate);
                *inner.now_playing.lock().unwrap_or_else(|e| e.into_inner()) =
                    Some(tail.clone());
                if let Err(e) = crate::tts::play_pcm(&rest, tail.rate) {
                    log_play_err("stepped-on tail", e);
                }
                inner
                    .now_playing
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .take();
                // Refresh the echo memory: this audio just left the speakers
                // again, and the mic may catch it.
                let mut said = inner.spoken.lock().unwrap();
                said.push((Instant::now(), tail.text.clone()));
                said.retain(|(t, _)| t.elapsed() < ECHO_MEMORY);
            }
            if let Some(latch) = utt.done {
                *latch.spoken.lock().unwrap() = true;
                latch.cv.notify_all();
            }
            continue;
        }
        if utt.negative {
            if let Some(path) = &negative_path {
                if let Err(e) = crate::tts::play(path) {
                    log_play_err("negative tone", e);
                }
            }
            if let Some(latch) = utt.done {
                *latch.spoken.lock().unwrap() = true;
                latch.cv.notify_all();
            }
            continue;
        }
        if utt.chime {
            if let Some(path) = &chime_path {
                if let Err(e) = crate::tts::play(path) {
                    log_play_err("chime", e);
                }
            }
        }
        // A station utterance speaks through the ATC voice (warm ONNX + radio
        // effect, built on first use) instead of the copilot's.
        //
        // catch_unwind: a panic ANYWHERE in a TTS engine must not kill this
        // thread — blocking callers wait on latches only this thread signals,
        // so a dead speaker deadlocks the whole app (seen live 2026-07-07: the
        // first station utterance panicked, the main loop hung unkillable, and
        // the backtrace sprayed over the raw-mode TUI). A panicking station
        // voice is demoted to the `say` fallback for the rest of the session.
        let seed = inner
            .station_seed
            .load(std::sync::atomic::Ordering::Relaxed);
        if utt.station.is_some() && seed != stations_seed {
            stations = [None, None, None, None, None];
            stations_seed = seed;
        }
        let local = utt.station.and_then(|_| {
            inner
                .station_local
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .clone()
        });
        let station_model = utt
            .station
            .map(|who| who.model(&station_cfg, seed, local.as_deref()));
        let spoke = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            if let (Some(who), Some(model)) = (utt.station, &station_model) {
                let slot = &mut stations[who.slot()];
                if slot.as_ref().map(|(m, _)| m.as_str()) != Some(model.as_str()) {
                    *slot = Some((model.clone(), StationVoice::build(&station_cfg, model)));
                }
                // Publish the transmission as on-air just before playback, so
                // a PTT press steps on it (steals it with its clock running)
                // instead of erasing it. Copilot speech is deliberately never
                // published: barging in on your own crew drops their line.
                slot.as_mut().expect("just built").1.speak(&utt.text, |samples, rate| {
                    *inner.now_playing.lock().unwrap_or_else(|e| e.into_inner()) =
                        Some(Arc::new(SteppedOn {
                            samples: samples.to_vec(),
                            rate,
                            started: Instant::now(),
                            speaker: who,
                            text: utt.text.clone(),
                            priority: utt.priority,
                            seq: utt.seq,
                        }));
                })
            } else {
                speaker.speak(&utt.text)
            }
        }))
        .unwrap_or_else(|panic| {
            let what = panic
                .downcast_ref::<&str>()
                .map(|s| s.to_string())
                .or_else(|| panic.downcast_ref::<String>().cloned())
                .unwrap_or_else(|| "non-string panic".into());
            if let (Some(who), Some(model)) = (utt.station, &station_model) {
                tracing::error!("station voice panicked ({what}); demoting to `say` fallback");
                stations[who.slot()] = Some((
                    model.clone(),
                    StationVoice::Say(Config {
                        tts_backend: "say".into(),
                        say_voice: station_cfg.atc_voice.clone(),
                        say_rate: station_cfg.atc_rate.clone(),
                        ..Config::default()
                    }),
                ));
            }
            Err(anyhow::anyhow!("TTS panicked: {what}"))
        });
        if let Err(e) = spoke {
            // An interrupt (PTT / barge-in) kills `say`/piper with SIGTERM — that's
            // an intentional cut-off, not a failure, so don't log it as a warning.
            let msg = format!("{e:#}");
            if msg.contains("SIGTERM") || msg.contains("signal: 15") {
                tracing::debug!("TTS interrupted (barge-in): {msg}");
            } else {
                tracing::warn!("TTS failed: {msg}");
            }
        }
        // Off the air, either way: finished playback clears its own slot; a
        // barge-in already stole it (this take then finds None, harmlessly).
        inner
            .now_playing
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .take();
        // Remember what we just said (it just finished playing) so the listener can
        // reject this audio if it bleeds back through the mic. Prune the old.
        {
            let mut said = inner.spoken.lock().unwrap();
            said.push((Instant::now(), utt.text.clone()));
            said.retain(|(t, _)| t.elapsed() < ECHO_MEMORY);
        }

        if let Some(latch) = utt.done {
            *latch.spoken.lock().unwrap() = true;
            latch.cv.notify_all();
        }
    }
}

/// An env-tunable f32 with a default (station cadence knobs).
#[cfg(feature = "onnx-tts")]
fn env_f32(name: &str, default: f32) -> f32 {
    std::env::var(name).ok().and_then(|v| v.trim().parse().ok()).unwrap_or(default)
}

/// Resolve the reminder chime to a playable file, or `None` to skip it. An
/// explicit `XCOPILOT_CHIME` wins; otherwise fall back to a pleasant macOS system
/// sound. Only returns a path that actually exists, so a stale setting just means
/// "no chime" rather than a failed play every time.
fn resolve_chime(configured: &str) -> Option<std::path::PathBuf> {
    let candidate = if !configured.is_empty() {
        std::path::PathBuf::from(configured)
    } else if cfg!(target_os = "macos") {
        std::path::PathBuf::from("/System/Library/Sounds/Glass.aiff")
    } else {
        return None;
    };
    candidate.exists().then_some(candidate)
}

/// Resolve the "unrecognized command" negative tone to a playable file, or `None`
/// to skip it. An explicit `XCOPILOT_NEGATIVE_TONE` wins; otherwise fall back to
/// the macOS system error sound (a low bassoon thud — reads unmistakably as
/// "no"). Only returns a path that exists, so a stale setting means "no tone"
/// rather than a failed play on every miss.
fn resolve_negative(configured: &str) -> Option<std::path::PathBuf> {
    let candidate = if !configured.is_empty() {
        std::path::PathBuf::from(configured)
    } else if cfg!(target_os = "macos") {
        std::path::PathBuf::from("/System/Library/Sounds/Basso.aiff")
    } else {
        return None;
    };
    candidate.exists().then_some(candidate)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn utt(priority: Priority, seq: u64) -> Utterance {
        Utterance { priority, seq, text: String::new(), chime: false, negative: false, station: None, tail: None, done: None }
    }

    #[test]
    fn airport_voice_crews_are_stable_distinct_and_never_home() {
        // Stable per name, never 0 (0 = the home field's curated voices).
        assert_eq!(station_voice_seed("Ciudad Real"), station_voice_seed("Ciudad Real"));
        assert_ne!(station_voice_seed("Ciudad Real"), 0);
        assert_ne!(station_voice_seed("Ciudad Real"), station_voice_seed("Malaga"));
        // With a pool, a seeded airport picks from it — and the four positions
        // land on distinct voices when the pool has at least four entries.
        let cfg = Config {
            atc_piper_voice: "tower.onnx".into(),
            atc_voice_pool: "a.onnx, b.onnx, c.onnx, d.onnx, e.onnx, f.onnx, g.onnx, h.onnx"
                .into(),
            ..Config::default()
        };
        let seed = station_voice_seed("Ciudad Real");
        let crew: Vec<String> = [
            StationSpeaker::Tower,
            StationSpeaker::Approach,
            StationSpeaker::Departure,
            StationSpeaker::Atis,
        ]
        .iter()
        .map(|s| s.model(&cfg, seed, None))
        .collect();
        let mut unique = crew.clone();
        unique.sort();
        unique.dedup();
        assert_eq!(unique.len(), 4, "positions must be distinct people: {crew:?}");
        // Seed 0 = the curated home voices, pool or not.
        assert_eq!(StationSpeaker::Tower.model(&cfg, 0, None), "tower.onnx");
        // No pool → curated voices even for a seeded airport.
        let bare = Config { atc_piper_voice: "tower.onnx".into(), ..Config::default() };
        assert_eq!(StationSpeaker::Tower.model(&bare, seed, None), "tower.onnx");
        // Country casting: the local accent mans the tower and reads the
        // ATIS; approach/departure stay neutral (from the pool).
        let local = Some("es.onnx");
        assert_eq!(StationSpeaker::Tower.model(&cfg, seed, local), "es.onnx");
        assert_eq!(StationSpeaker::Atis.model(&cfg, seed, local), "es.onnx");
        assert_ne!(StationSpeaker::Approach.model(&cfg, seed, local), "es.onnx");
        // Geography: Castilian in Spain and the Canaries, es_MX only in
        // Spanish Latin America, neutral elsewhere (and NEVER es_MX in Spain).
        assert_eq!(local_accent_for("LEMD"), Some("models/es_ES-davefx-medium.onnx"));
        assert_eq!(local_accent_for("GCXO"), Some("models/es_ES-davefx-medium.onnx"));
        assert_eq!(local_accent_for("MMMX"), Some("models/es_MX-claude-high.onnx"));
        assert_eq!(local_accent_for("SBGR"), None, "Brazil speaks Portuguese, stays neutral");
        assert_eq!(local_accent_for("EGLL"), None);
        assert_eq!(local_accent_for("LXGB"), None, "home field keeps its curated cast anyway");
    }

    #[test]
    fn higher_priority_pops_first() {
        let mut h = BinaryHeap::new();
        h.push(utt(Priority::Advisory, 0));
        h.push(utt(Priority::Warning, 1));
        h.push(utt(Priority::Readback, 2));
        assert_eq!(h.pop().unwrap().priority, Priority::Warning);
        assert_eq!(h.pop().unwrap().priority, Priority::Readback);
        assert_eq!(h.pop().unwrap().priority, Priority::Advisory);
    }

    #[test]
    fn stepped_on_tail_tracks_real_time() {
        let rate = 22_050;
        let len = 3 * rate as usize; // a 3-second transmission
        // One second into the call: the tail starts one second of samples in.
        assert_eq!(tail_offset(1.0, len, rate), Some(rate as usize));
        // The press outlived the transmission: nothing left to hear.
        assert_eq!(tail_offset(3.0, len, rate), None);
        assert_eq!(tail_offset(10.0, len, rate), None);
        // Under 50 ms remaining is just the squelch stub: skip it.
        assert_eq!(tail_offset(2.99, len, rate), None);
        // A clock skew below zero clamps to the start rather than panicking.
        assert_eq!(tail_offset(-0.5, len, rate), Some(0));
    }

    #[test]
    fn tail_fades_in_from_silence() {
        let rate = 22_050;
        let mut s = vec![1.0_f32; rate as usize];
        fade_in(&mut s, rate);
        assert_eq!(s[0], 0.0, "mid-word entry must open from silence");
        assert!(s[rate as usize / 100] > 0.99, "ramp is over within ~10 ms");
        // Degenerate buffers (shorter than the ramp) must not panic.
        let mut tiny = vec![1.0_f32; 3];
        fade_in(&mut tiny, rate);
    }

    #[test]
    fn requeued_tail_keeps_its_place_in_line() {
        // The stepped-on transmission requeues with its ORIGINAL seq, so it
        // pops ahead of everything enqueued while PTT was held.
        let mut h = BinaryHeap::new();
        h.push(utt(Priority::Readback, 7)); // held backlog, queued during the press
        h.push(utt(Priority::Readback, 8));
        h.push(utt(Priority::Readback, 3)); // the tail, preserved seq
        assert_eq!(h.pop().unwrap().seq, 3);
        // But a higher priority still outranks it: a Warning does not wait
        // behind a resumed transmission.
        h.push(utt(Priority::Warning, 9));
        assert_eq!(h.pop().unwrap().priority, Priority::Warning);
    }

    #[test]
    fn same_priority_is_fifo() {
        let mut h = BinaryHeap::new();
        h.push(utt(Priority::Readback, 5));
        h.push(utt(Priority::Readback, 6));
        h.push(utt(Priority::Readback, 7));
        // earliest enqueued (smallest seq) comes out first
        assert_eq!(h.pop().unwrap().seq, 5);
        assert_eq!(h.pop().unwrap().seq, 6);
        assert_eq!(h.pop().unwrap().seq, 7);
    }
}
