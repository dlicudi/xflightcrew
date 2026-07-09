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
}

impl StationSpeaker {
    /// The configured Piper model for this position. Approach/departure fall
    /// back to the tower's voice when unset, never to silence.
    fn model<'a>(self, config: &'a Config) -> &'a str {
        let fallback = config.atc_piper_voice.as_str();
        match self {
            StationSpeaker::Tower => fallback,
            StationSpeaker::Approach => {
                let v = config.atc_approach_piper_voice.as_str();
                if v.is_empty() { fallback } else { v }
            }
            StationSpeaker::Departure => {
                let v = config.atc_departure_piper_voice.as_str();
                if v.is_empty() { fallback } else { v }
            }
        }
    }
    fn slot(self) -> usize {
        match self {
            StationSpeaker::Tower => 0,
            StationSpeaker::Approach => 1,
            StationSpeaker::Departure => 2,
        }
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
    pub fn shutup(&self) {
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
        self.push(Utterance { priority, seq: 0, text, chime, negative: false, station: None, done });
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
        self.push(Utterance { priority, seq: 0, text, chime: false, negative: false, station: Some(speaker), done });
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
                    engine.override_scales(
                        None,
                        Some(env_f32("XCOPILOT_ATC_TTS_LENGTH", 0.8)),
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

    fn speak(&mut self, text: &str) -> anyhow::Result<()> {
        match self {
            #[cfg(feature = "onnx-tts")]
            StationVoice::Onnx { engine, tail, transmissions } => {
                *transmissions = transmissions.wrapping_add(1);
                let variant = *transmissions;
                let tail = tail.clone();
                engine.speak_with(text, move |s, r| match &tail {
                    Some(t) => crate::radiofx::radioize_with_tail(s, r, variant, t),
                    None => crate::radiofx::radioize_variant(s, r, variant),
                })
            }
            StationVoice::Say(config) => tts::speak(config, text),
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
    // One lazy warm voice per controller position (tower/approach/departure):
    // a position that never speaks never loads its model.
    let mut stations: [Option<StationVoice>; 3] = [None, None, None];
    let mut speaker = Speaker::build(config);
    loop {
        let mut q = inner.queue.lock().unwrap();
        let utt = loop {
            if q.shutdown {
                gate.set_speaking(false);
                return;
            }
            if let Some(u) = q.heap.pop() {
                break u;
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
        let spoke = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            if let Some(who) = utt.station {
                stations[who.slot()]
                    .get_or_insert_with(|| StationVoice::build(&station_cfg, who.model(&station_cfg)))
                    .speak(&utt.text)
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
            if let Some(who) = utt.station {
                tracing::error!("station voice panicked ({what}); demoting to `say` fallback");
                stations[who.slot()] = Some(StationVoice::Say(Config {
                    tts_backend: "say".into(),
                    say_voice: station_cfg.atc_voice.clone(),
                    say_rate: station_cfg.atc_rate.clone(),
                    ..Config::default()
                }));
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
        Utterance { priority, seq, text: String::new(), chime: false, negative: false, station: None, done: None }
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
