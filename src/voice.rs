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

struct Utterance {
    priority: Priority,
    /// Monotonic enqueue order, for FIFO tie-breaking within a priority.
    seq: u64,
    text: String,
    /// Play the attention chime just before speaking this one (reminders).
    chime: bool,
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
        let mut q = self.inner.queue.lock().unwrap();
        let seq = q.seq;
        q.seq += 1;
        q.heap.push(Utterance {
            priority,
            seq,
            text,
            chime,
            done,
        });
        drop(q);
        self.inner.cv.notify_one();
    }

    /// Queue an utterance and return immediately (monitor alerts).
    #[allow(dead_code)] // used by the monitor (next milestone)
    pub fn alert(&self, priority: Priority, text: impl Into<String>) {
        self.enqueue(priority, text.into(), None, false);
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
        if utt.chime {
            if let Some(path) = &chime_path {
                if let Err(e) = crate::tts::play(path) {
                    tracing::warn!("chime failed: {e:#}");
                }
            }
        }
        if let Err(e) = speaker.speak(&utt.text) {
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

#[cfg(test)]
mod tests {
    use super::*;

    fn utt(priority: Priority, seq: u64) -> Utterance {
        Utterance { priority, seq, text: String::new(), chime: false, done: None }
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
