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

use crate::audio::MicGate;
use crate::config::Config;
use crate::tts;

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

    fn enqueue(&self, priority: Priority, text: String, done: Option<Arc<Latch>>) {
        let mut q = self.inner.queue.lock().unwrap();
        let seq = q.seq;
        q.seq += 1;
        q.heap.push(Utterance {
            priority,
            seq,
            text,
            done,
        });
        drop(q);
        self.inner.cv.notify_one();
    }

    /// Queue an utterance and return immediately (monitor alerts).
    #[allow(dead_code)] // used by the monitor (next milestone)
    pub fn alert(&self, priority: Priority, text: impl Into<String>) {
        self.enqueue(priority, text.into(), None);
    }

    /// Queue an utterance and block until it has been spoken (command readbacks:
    /// the loop must wait so it doesn't listen over its own voice).
    pub fn say(&self, priority: Priority, text: impl Into<String>) {
        let latch = Arc::new(Latch {
            spoken: Mutex::new(false),
            cv: Condvar::new(),
        });
        self.enqueue(priority, text.into(), Some(latch.clone()));
        let mut spoken = latch.spoken.lock().unwrap();
        while !*spoken {
            spoken = latch.cv.wait(spoken).unwrap();
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
        if let Err(e) = tts::speak(&config, &utt.text) {
            tracing::warn!("TTS failed: {e:#}");
        }

        if let Some(latch) = utt.done {
            *latch.spoken.lock().unwrap() = true;
            latch.cv.notify_all();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn utt(priority: Priority, seq: u64) -> Utterance {
        Utterance { priority, seq, text: String::new(), done: None }
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
