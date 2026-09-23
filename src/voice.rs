//! Speech arbiter — the owner of BOTH voices: the copilot's intercom and the
//! station radio.
//!
//! Two independent half-duplex channels (2026-07-14), like a real audio panel:
//!
//!   - **Intercom** — the copilot's voice. One queue, one speaker thread;
//!     copilot utterances are serialized and never overlap each other.
//!   - **Radio** — every station voice (controllers, ATIS, ghost pilots).
//!     Its own queue and speaker thread; station utterances are serialized
//!     among themselves.
//!
//! The two channels are mixed, not ducked — both reach the headset, as in a
//! real cockpit. Realism priority is crew discipline: the intercom defers
//! *starting* a routine line (anything below `Warning`) while the radio is
//! on the air, and a routine line already in progress STOPS mid-word when a
//! transmission starts and its remainder is queued to follow once the
//! frequency is quiet ([`interrupt_intercom_for_radio`] — a crew stops and
//! listens before it knows whether the call is for them). The perishable
//! `stale_by` machinery drops what expired waiting, remainders included.
//! Urgent callouts (stall, terrain — `Warning`) break through immediately
//! and are never stepped on: a real copilot talks over the radio when you're
//! about to die. The one thing that silences BOTH
//! channels is a PTT press (`set_hold` / the barge-in `shutup`) — mic
//! hygiene: on speakers, any voice still playing bleeds into the capture.
//!
//! Priority orders each *queue*: a higher-priority utterance jumps ahead of
//! whatever is still waiting, but never cuts off speech already playing on
//! its own channel (only the radio going on air does that, above). Within a
//! priority, order is FIFO.
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

/// The LOCAL accent voices for a country, by ICAO prefix — the voices its
/// towers and ATIS speak English with. Deliberately strict about geography:
/// Spain (LE) and the Canaries (GC) get the Castilian es_ES voices, and the
/// es_MX voices serve only Spanish-speaking Latin America — a Madrid tower
/// must not sound like Mexico City ("no mexicans in Spain", 2026-07-09).
/// Fields with no mapping cast entirely from the neutral pool. A LIST per
/// country so a field's tower and ATIS can be different people ("main thing
/// is that we actually have different voices", 2026-07-12); `#N` picks a
/// speaker inside a multi-speaker model. Curation is by deletion: a voice
/// that doesn't survive the audition gets removed here (or its file deleted
/// — [`local_voices_on_disk`] quietly drops specs whose model is missing).
fn local_accents_for(icao: &str) -> &'static [&'static str] {
    // Seated by the 2026-09-22 audition (42 samples, radio effect on, the
    // accent pace, pass/fail by ear): a model that only ever heard its own
    // language reads English as guesses, and most of Piper's catalogue fails
    // on that alone. Six passed (five since thorsten failed in service).
    // Every French, Italian and Belgian candidate failed, as did both
    // multi-speaker `mls` sets, so those countries stay
    // neutral until a voice worth having exists (likely one trained on
    // accented ENGLISH, not a Piper pick).
    match icao.get(0..2).unwrap_or("") {
        // Spain + Canaries: davefx, rejected on 2026-07-12 without the radio
        // effect and passed through it on 2026-09-22. sharvard (both
        // speakers) failed both times.
        "LE" | "GC" => &["models/es_ES-davefx-medium.onnx"],
        // Germany: none. thorsten passed the 2026-09-22 audition and failed
        // in service at EDDV the next day (the pilot: "accent /
        // intelligibility"), so German fields cast from the neutral pool.
        // The Netherlands: two, so the tower and the ATIS differ.
        "EH" => &[
            "models/nl_NL-pim-medium.onnx",
            "models/nl_NL-ronnie-medium.onnx",
        ],
        // Portugal, Madeira and the Azores (Brazil is SB: a different
        // accent, and stays neutral).
        "LP" => &["models/pt_PT-tugao-medium.onnx"],
        // Argentina. Never Spain: geography, as the es_MX rule was.
        "SA" => &["models/es_AR-daniela-high.onnx"],
        // Mexico and the rest of Spanish Latin America: empty since
        // 2026-07-12 (ald and claude-high failed; claude-high again
        // 2026-09-22).
        _ => &[],
    }
}

/// A station voice's pace, as a multiple of the model's natural pace (see
/// `OnnxTts::override_scales`): brisk 0.8 for a controller, 1.1 for an
/// accent, and 1.15 for the VCTK speakers, who read fast by nature - picked
/// by ear from 0.8 to 1.2 on 2026-09-22 ("1.1 or 1.2"), 0.8 and 1.0 both
/// "too fast".
fn station_pace(model: &str) -> f32 {
    if is_accent_model(model) {
        1.1
    } else if let Some((_, pace)) = MEASURED_PACE.iter().find(|(spec, _)| *spec == model) {
        *pace
    } else if model.contains("en_GB-vctk") {
        1.15
    } else {
        0.8
    }
}

/// Per-speaker paces for the multi-speaker British sets, which read at very
/// different natural speeds (VCTK p259 is 0.79 of VCTK p239's pace): each
/// speaker's natural length on one tower call, measured with `xvl tts`,
/// scaled so every one of them speaks at the rate picked by ear for p239 at
/// 1.15 (2026-09-22). Every entry passed the same day's audition, except
/// VCTK #58, #80 and #96, pool members from before it that were not rated.
const MEASURED_PACE: &[(&str, f32)] = &[
    ("models/en_GB-vctk-medium.onnx#0", 1.15),
    ("models/en_GB-vctk-medium.onnx#1", 0.99),
    ("models/en_GB-vctk-medium.onnx#2", 0.83),
    ("models/en_GB-vctk-medium.onnx#3", 0.94),
    ("models/en_GB-vctk-medium.onnx#4", 0.79),
    ("models/en_GB-vctk-medium.onnx#5", 0.85),
    ("models/en_GB-vctk-medium.onnx#6", 0.82),
    ("models/en_GB-vctk-medium.onnx#7", 0.94),
    ("models/en_GB-vctk-medium.onnx#8", 0.95),
    ("models/en_GB-vctk-medium.onnx#9", 0.72),
    ("models/en_GB-vctk-medium.onnx#14", 0.82),
    ("models/en_GB-vctk-medium.onnx#15", 1.16),
    ("models/en_GB-vctk-medium.onnx#18", 1.21),
    ("models/en_GB-vctk-medium.onnx#19", 0.94),
    ("models/en_GB-vctk-medium.onnx#20", 0.97),
    ("models/en_GB-vctk-medium.onnx#22", 0.9),
    ("models/en_GB-vctk-medium.onnx#27", 0.79),
    ("models/en_GB-vctk-medium.onnx#34", 0.98),
    ("models/en_GB-vctk-medium.onnx#37", 0.81),
    ("models/en_GB-vctk-medium.onnx#58", 0.73),
    ("models/en_GB-vctk-medium.onnx#80", 0.79),
    ("models/en_GB-vctk-medium.onnx#96", 0.96),
    ("models/en_GB-aru-medium.onnx#0", 0.77),
    ("models/en_GB-aru-medium.onnx#1", 0.95),
    ("models/en_GB-aru-medium.onnx#2", 1.0),
    ("models/en_GB-aru-medium.onnx#3", 0.66),
    ("models/en_GB-aru-medium.onnx#4", 0.88),
    ("models/en_GB-aru-medium.onnx#5", 0.9),
    ("models/en_GB-aru-medium.onnx#6", 0.78),
    ("models/en_GB-aru-medium.onnx#7", 0.94),
    ("models/en_GB-aru-medium.onnx#8", 0.99),
    ("models/en_GB-aru-medium.onnx#9", 0.91),
    ("models/en_GB-aru-medium.onnx#10", 0.95),
    ("models/en_GB-aru-medium.onnx#11", 0.82),
    ("models/en_GB-semaine-medium.onnx#0", 0.77),
    ("models/en_GB-semaine-medium.onnx#1", 0.74),
    ("models/en_GB-semaine-medium.onnx#2", 0.67),
    ("models/en_GB-jenny_dioco-medium.onnx", 0.65),
];

/// A non-English model reading English: an accent, voiced at the measured
/// pace (see `StationVoice::build`), and never in the neutral pool.
fn is_accent_model(spec: &str) -> bool {
    let file = spec.rsplit('/').next().unwrap_or(spec);
    !file.starts_with("en_")
}

/// The local voices for `icao` whose model files actually exist — what the
/// caster gets. Deleting a model file is how a voice leaves the roster.
pub fn local_voices_on_disk(icao: &str) -> Vec<String> {
    local_accents_for(icao)
        .iter()
        .filter(|spec| {
            let path = spec.split('#').next().unwrap_or(spec);
            std::path::Path::new(path).exists()
        })
        .map(|s| s.to_string())
        .collect()
}

/// Every piper voice on disk — what the panel's Settings picker offers for
/// the copilot's own voice. A model qualifies by having its `.onnx.json`
/// sidecar beside it — the piper shape, and piper is the only engine we run.
/// Sorted so the panel index is stable across runs;
/// entries are full model paths, display names are the file stems.
pub fn pilot_voice_choices() -> Vec<String> {
    let mut voices: Vec<String> = std::fs::read_dir("models")
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|entry| {
            let path = entry.path();
            let is_onnx = path.extension().is_some_and(|e| e == "onnx");
            let has_sidecar = std::path::Path::new(&format!("{}.json", path.display())).exists();
            (is_onnx && has_sidecar).then(|| path.to_string_lossy().into_owned())
        })
        .collect();
    voices.sort();
    voices
}

/// The cast identity a station voice speaks AS — what keys its pitch/pace
/// shift ([`crate::radiofx::shift_factor`]). Model + position + airport seed,
/// so the same pool model is a different person at a different field or in a
/// different chair, and stays the same person all flight (rebuilds of the
/// warm session recompute the same value).
fn station_person(model: &str, slot: usize, seed: usize) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in model
        .as_bytes()
        .iter()
        .chain(&[slot as u8])
        .chain(&seed.to_le_bytes())
    {
        h ^= u64::from(*b);
        h = h.wrapping_mul(0x0000_0100_0000_01B3);
    }
    h
}

// Speaker identity lives with the world side; the arbiter re-exports it so
// crew callers keep their `voice::StationSpeaker` paths.
pub use crate::speaker::{station_voice_seed, StationSpeaker};

/// One-shot latch a blocking caller waits on until its utterance is spoken.
#[derive(Default)]
struct Latch {
    spoken: Mutex<bool>,
    cv: Condvar,
}

/// The Piper model for this position. Seed 0 (the configured home field)
/// uses the curated per-position voices; any other seed draws a stable
/// per-airport crew from `atc_voice_pool`, offset per position so the
/// controllers at one field are distinct people. An empty pool always
/// falls back to the curated voices. Voice casting is the copilot's
/// business — the world side only carries the [`StationSpeaker`] identity.
fn station_model(who: StationSpeaker, config: &Config, seed: usize, locals: &[String]) -> String {
    // A ghost pilot's voice comes from ITS OWN seed (per callsign), not
    // the airport's cast — pilots are from everywhere, and the same
    // flight keeps the same voice across calls.
    if let StationSpeaker::Ghost { seed: ghost } = who {
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
        // The country's own accents man the tower and read the ATIS;
        // approach/departure fill from the neutral pool. With two or
        // more locals on the roster, the tower and the ATIS reader are
        // DIFFERENT people at every field (offset picks), stable per
        // airport via the seed.
        if !locals.is_empty() && matches!(who, StationSpeaker::Tower | StationSpeaker::Atis) {
            let offset = if matches!(who, StationSpeaker::Atis) {
                1
            } else {
                0
            };
            return locals[(seed + offset) % locals.len()].clone();
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
            return pool[(seed + who.slot() * step) % pool.len()].to_string();
        }
    }
    let fallback = config.atc_piper_voice.as_str();
    match who {
        // Handled above (own-seed pool pick); reachable only with an
        // empty pool, where everyone shares the curated station voice.
        StationSpeaker::Ghost { .. } => fallback,
        StationSpeaker::Tower => fallback,
        StationSpeaker::Approach => {
            let v = config.atc_approach_piper_voice.as_str();
            if v.is_empty() {
                fallback
            } else {
                v
            }
        }
        StationSpeaker::Departure => {
            let v = config.atc_departure_piper_voice.as_str();
            if v.is_empty() {
                fallback
            } else {
                v
            }
        }
        StationSpeaker::Atis => {
            let v = config.atc_atis_piper_voice.as_str();
            if v.is_empty() {
                fallback
            } else {
                v
            }
        }
    }
    .to_string()
}

/// Whether `speaker` is audible on the currently-tuned station RIGHT NOW.
/// Ghost pilots ride the tower's party line, so they're heard iff the tower
/// is tuned; every other speaker matches its own position. `None` means
/// definitively OFF all station frequencies (the caller resolved a real
/// COM1 read to no station) — nothing station is audible. Used to cut live
/// audio the instant the pilot tunes away.
fn audible_now(tuned: Option<StationSpeaker>, speaker: StationSpeaker) -> bool {
    match tuned {
        None => false,
        Some(StationSpeaker::Tower) => {
            matches!(
                speaker,
                StationSpeaker::Tower | StationSpeaker::Ghost { .. }
            )
        }
        Some(t) => t == speaker,
    }
}

/// Whether a station utterance can't be heard RIGHT NOW: the radio receiver
/// is unpowered, or COM1 is confidently tuned off the speaker's frequency.
/// Copilot speech (`station == None`) is intercom and always hearable. FAIL
/// OPEN twice over: `radio_powered` defaults true, and the tuning check only
/// applies once a monitor has actually published COM1 state (`com1_known`) —
/// a setup with no monitor tick must never go mysteriously silent.
fn station_unhearable(inner: &Inner, station: Option<StationSpeaker>) -> bool {
    let Some(who) = station else { return false };
    if inner.crashed.load(std::sync::atomic::Ordering::Relaxed) {
        return true; // wreckage receives nothing
    }
    if inner.replay.load(std::sync::atomic::Ordering::Relaxed) {
        return true; // a replay is a recording — the world isn't on the air
    }
    if !inner
        .radio_powered
        .load(std::sync::atomic::Ordering::Relaxed)
    {
        return true;
    }
    inner.com1_known.load(std::sync::atomic::Ordering::Relaxed)
        && !audible_now(
            *inner
                .tuned_station
                .lock()
                .unwrap_or_else(|e| e.into_inner()),
            who,
        )
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
    /// A station line addressed to SOMEONE ELSE (the tower answering a ghost
    /// in an overheard exchange): aired, but never the controller's last
    /// word to us, so a "say again" never repeats another aircraft's clearance.
    overheard: bool,
    /// Speak as the CABIN CREW: the second intercom persona (interphone, not
    /// radio — same channel and discipline as the copilot, different voice).
    /// Meaningless when `station` is set.
    cabin: bool,
    /// Speak by this instant or not at all: time-critical speech (taxi cues)
    /// is worse than silence once its moment has passed — a "next left"
    /// queued behind a party-line exchange spoke 17 s late at Malaga, 130 m
    /// past the junction (2026-07-13). Checked at dequeue; `None` = evergreen.
    stale_by: Option<Instant>,
    /// Present for blocking callers; signalled once the utterance is spoken.
    done: Option<Arc<Latch>>,
    /// Already-rendered audio to play INSTEAD of synthesising `text`: the
    /// remainder of a line the radio stepped on (see
    /// [`interrupt_intercom_for_radio`]). `text` stays for the echo memory.
    resume: Option<Resume>,
}

/// The rest of a stepped-on intercom line, from just before the cut.
#[derive(Clone)]
struct Resume {
    samples: Arc<Vec<f32>>,
    rate: u32,
}

/// What the intercom is saying right now: enough to step on it and come back
/// to it. `audio` is the rendered line (the warm ONNX path); the subprocess
/// voices have no buffer, so a stepped-on line of theirs is said again whole.
/// `done` is the blocking caller's latch, which the REMAINDER inherits: a
/// caller is released when the sentence is finished, not when it was cut.
struct IntercomPlay {
    priority: Priority,
    audio: Option<Resume>,
    started: Instant,
    text: String,
    cabin: bool,
    done: Option<Arc<Latch>>,
}

/// Back up this far before the cut when resuming, so the word that was
/// interrupted is heard whole ("...as I was saying").
const RESUME_LEAD_S: f64 = 0.4;
/// Less than this much of the line left past the cut is not worth coming
/// back for: the sentence was effectively finished.
const RESUME_MIN_LEFT_S: f64 = 0.4;
/// A stepped-on remainder that has waited this long is stale: the moment
/// has passed, and a fragment out of context is worse than silence.
const RESUME_STALE_S: u64 = 20;

/// A fire-and-forget ADVISORY that has not started speaking this long after
/// it was queued is dropped. Advisories are narration ("Beacon's on",
/// "Next, KOPOR", "Flaps up"): the mission queued three of them in one
/// second at the LFLL start and the arbiter was still reading them out ten
/// seconds later, and a burst behind a radio exchange plays a minute after
/// the events it describes ("speech lags and never catches up", 2026-09-19).
/// Spoken late, narration is disinformation; the panel transcript keeps the
/// line (mirrored at enqueue) and a dropped one is logged. Readbacks
/// (blocking, the pilot asked), cautions and warnings never expire.
pub const ADVISORY_STALE_S: u64 = 12;

/// The sample range to resume from: `elapsed` into a line of `len` samples at
/// `rate`, less the lead-in. `None` when the line had effectively finished.
fn resume_range(
    len: usize,
    rate: u32,
    elapsed: std::time::Duration,
) -> Option<std::ops::Range<usize>> {
    let played = (elapsed.as_secs_f64() * rate as f64) as usize;
    let lead = (RESUME_LEAD_S * rate as f64) as usize;
    let min_left = (RESUME_MIN_LEFT_S * rate as f64) as usize;
    (played + min_left < len).then_some(played.saturating_sub(lead)..len)
}

/// Does a transmission starting now step on a line of this priority? Routine
/// lines yield, the way a crew stops mid-word to listen; a Warning (stall,
/// terrain) does not, because the radio can wait and the ground cannot.
fn radio_interrupts(priority: Priority) -> bool {
    priority < Priority::Warning
}

/// Push onto the intercom queue from anywhere that holds the arbiter.
fn push_intercom(inner: &Inner, mut utt: Utterance) {
    let mut q = inner.queue.lock().unwrap_or_else(|e| e.into_inner());
    utt.seq = q.seq;
    q.seq += 1;
    q.heap.push(utt);
    drop(q);
    inner.cv.notify_one();
}

/// The radio has just gone on the air: a routine intercom line in progress
/// stops mid-word, and its remainder is queued to be said once the frequency
/// is quiet — the crew discipline a real cockpit has. Before this the copilot
/// talked through a transmission it had started a second earlier; the
/// arbiter only ever gated what STARTS (2026-09-11). The remainder rides the
/// normal queue, so the same deferral and expiry apply, and a PTT press or a
/// second call can step on it again.
fn interrupt_intercom_for_radio(inner: &Inner) {
    let taken = {
        let mut playing = inner
            .intercom_playing
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        match playing.as_ref() {
            Some(p) if radio_interrupts(p.priority) => playing.take(),
            _ => None,
        }
    };
    let Some(p) = taken else {
        return;
    };
    tts::stop_channel(tts::Channel::Intercom);
    // What is owed: the rendered remainder; the whole line again when there
    // was no buffer; nothing when the sentence was effectively finished.
    let owed = match &p.audio {
        Some(a) => resume_range(a.samples.len(), a.rate, p.started.elapsed()).map(|r| {
            Some(Resume {
                samples: Arc::new(a.samples[r].to_vec()),
                rate: a.rate,
            })
        }),
        None => Some(None),
    };
    match owed {
        Some(resume) => {
            tracing::info!(text = %p.text, resumed = resume.is_some(), "radio stepped on the intercom; remainder queued");
            // Owed until the remainder actually plays; if it perishes
            // waiting, "what were you saying?" still has it.
            if !p.cabin {
                inner
                    .memory
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .cut(&p.text, Instant::now());
            }
            push_intercom(
                inner,
                Utterance {
                    priority: p.priority,
                    seq: 0,
                    text: p.text,
                    chime: false,
                    negative: false,
                    station: None,
                    overheard: false,
                    cabin: p.cabin,
                    stale_by: Some(Instant::now() + std::time::Duration::from_secs(RESUME_STALE_S)),
                    done: p.done,
                    resume,
                },
            );
        }
        // Finished bar the tail: the caller is released now, not never.
        None => fire_latch(p.done),
    }
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
    /// The INTERCOM queue — the copilot's own voice.
    queue: Mutex<Queue>,
    /// Signalled when intercom work is queued, on hold release, or when the
    /// radio goes quiet (a deferred routine line may now start).
    cv: Condvar,
    /// The RADIO queue — every station voice (controllers, ATIS, ghosts).
    radio_queue: Mutex<Queue>,
    /// Signalled when radio work is queued or on hold release.
    radio_cv: Condvar,
    /// The intercom line in progress, for the radio to step on (see
    /// [`interrupt_intercom_for_radio`]); `None` between lines.
    intercom_playing: Mutex<Option<IntercomPlay>>,
    /// Which channels are speaking right now (intercom, radio). The mic gate
    /// is the OR of the two; the intercom's crew discipline reads the radio
    /// flag to defer routine lines while a station is on the air.
    speaking: Mutex<(bool, bool)>,
    /// Voice seed for the ACTIVE airport's controllers: 0 = the configured
    /// per-position voices (the curated home field); anything else indexes the
    /// voice pool per position, so each airport gets its own stable crew of
    /// controllers ("we need a different voice for Ciudad Real", 2026-07-09).
    /// Changing it drops the warm station sessions on the next utterance.
    station_seed: std::sync::atomic::AtomicUsize,
    /// The active airport's LOCAL accent voice, when its country has one
    /// (see [`local_voices_on_disk`]): cast as the tower and the ATIS reader,
    /// different entries per position when the roster has two or more.
    station_local: Mutex<Vec<String>>,
    /// RADIO PTT is held: EVERY voice stays quiet — copilot, controllers,
    /// ghosts. Transmitting is half-duplex; you can't hear the frequency, and
    /// you don't talk over your own call. The queue holds and resumes on
    /// release (2026-07-10: the party line yakked through PTT).
    hold: std::sync::atomic::AtomicBool,
    /// The panel transcript sink, so EVERY spoken line — including the ones the
    /// background flight loops (circuit, full auto, autoland) speak through
    /// `alert` — shows on the panel, not just the ones the main loop routes
    /// through the reporter (the auto-flight narration was audible but absent
    /// from the transcript, 2026-07-23). `None` until the main loop wires it.
    status: Mutex<Option<crate::plugin_status::StatusSink>>,
    /// Which station voice is on the air right now (published just before
    /// playback starts, cleared when it ends): used to CUT it (no resume) the
    /// instant the pilot tunes off its frequency or keys any PTT.
    now_playing: Mutex<Option<StationSpeaker>>,
    /// Which station position COM1 currently hears, as of the monitor's last
    /// good read (`None` = a real read that's off every station frequency).
    /// Read at dequeue time so a station utterance queued behind another
    /// voice (or a PTT hold) is dropped, not played onto whatever frequency
    /// the pilot tuned to meanwhile. Meaningful only once `com1_known` is
    /// set; before the first publish the speaker fails OPEN and plays.
    tuned_station: Mutex<Option<StationSpeaker>>,
    /// Whether COM1 tuning has EVER been published (a monitor is running and
    /// got a good read). Setups with no monitor tick — the standalone atc
    /// binary, tests — never publish, and their station audio must not go
    /// mysteriously silent: same fail-open contract as `radio_powered`.
    com1_known: std::sync::atomic::AtomicBool,
    /// Whether the aircraft's radio RECEIVER is powered (avionics master on).
    /// Station audio (controllers, ATIS, party-line traffic) is radio: with
    /// the avionics off you can't hear it, so it's dropped, not queued —
    /// missed radio is gone, like real life. The COPILOT's own voice is
    /// intercom, not radio, and always plays. FAIL OPEN: the monitor sets
    /// this false only on a CONFIDENT avionics-off read, so a study-level
    /// airframe we can't read never goes mysteriously silent.
    radio_powered: std::sync::atomic::AtomicBool,
    /// The airframe is WRECKAGE (X-Plane's has_crashed): no station audio in,
    /// no transmission out — a working avionics bus on the crash screen kept
    /// the radio alive, and a mayday from the wreck was answered with "hold
    /// position if able" (2026-07-13). The copilot's intercom still plays:
    /// the crew is in the wreck with you, and someone has to say why the
    /// radio's dead. Published by the monitor from the live dataref, so a
    /// flight reset (has_crashed back to 0) restores the world. Fail open:
    /// defaults false, and a failed read never sets it.
    crashed: std::sync::atomic::AtomicBool,
    /// The sim is playing a REPLAY (X-Plane's is_in_replay): the world on
    /// screen is a recording, so no station audio in and no transmission out —
    /// nothing that happens in a replay is happening NOW. The copilot's
    /// intercom still plays (the crew can answer questions about what you're
    /// watching). Published by the monitor from the live dataref; fail open:
    /// defaults false, and a failed read never sets it.
    replay: std::sync::atomic::AtomicBool,
    /// The sim is PAUSED (X-Plane's sim/time/paused): the flight is real but
    /// frozen. Unlike `replay` this does NOT drop station audio — a
    /// controller caught mid-word finishes the sentence, and the monitor's
    /// tick-skip means nothing new gets queued — it only gates the command
    /// loop's transmit paths (the world isn't turning; the FSM must not move).
    /// Published by the monitor; fail open like the others.
    paused: std::sync::atomic::AtomicBool,
    /// What the copilot has spoken recently, with when it finished. The listener
    /// checks heard transcripts against this to reject its own voice coming back
    /// over the speakers (echo) rather than acting on it.
    spoken: Mutex<Vec<(Instant, String)>>,
    /// A pending pilot-voice swap (a piper model path) from the panel's
    /// Settings picker. The intercom speaker thread consumes it before the
    /// next utterance — the swap happens on the thread that owns the warm
    /// engine, never mid-word. Station voices are cast separately (see
    /// [`StationSpeaker`]) and unaffected.
    pilot_swap: Mutex<Option<String>>,
    /// The cabin crew's pending voice pick, applied between utterances like
    /// the pilot's.
    cabin_swap: Mutex<Option<String>>,
    /// Range to the active station in nm (f32 bits; NaN = unknown), published
    /// by the monitor each tick and read per transmission — the radio's
    /// signal-quality model ([`crate::radiofx::signal_quality`]). Fail open:
    /// unknown reads as a perfect signal.
    station_range_nm: std::sync::atomic::AtomicU32,
    /// What the copilot last volunteered and what it was cut off saying, for
    /// "say again" / "what were you saying?" (see [`SpeechMemory`]).
    memory: Mutex<SpeechMemory>,
    /// COM1 in kHz as of the monitor's last good read (`i64::MIN` unknown):
    /// the frequency a controller line went out on, so a "say again" is only
    /// ever answered by the controller who said it.
    com1_khz: std::sync::atomic::AtomicI64,
    /// The last controller transmission addressed to US (see
    /// [`ControllerLine`]).
    radio_log: Mutex<Option<ControllerLine>>,
}

/// A controller line addressed to us, as it went out on the air (or was
/// keyed over by the pilot): what the frequency's "say again" repeats and
/// what "what did the tower say?" reports. Never ATIS, never a ghost pilot,
/// never the controller's half of an exchange with another aircraft.
#[derive(Clone, Debug)]
pub struct ControllerLine {
    pub text: String,
    pub speaker: StationSpeaker,
    /// COM1 when it was aired; `None` before any monitor read.
    pub khz: Option<i64>,
    pub at: Instant,
}

/// A controller repeats its last transmission for this long after it: the
/// pilot's "say again" is immediate practice, and past a minute the picture
/// may have moved on without a word (a readback challenge let go).
pub const RADIO_REPEAT_TTL: std::time::Duration = std::time::Duration::from_secs(60);

/// Is `speaker` a controller position (not the ATIS loop, not a ghost pilot)?
fn is_controller(speaker: StationSpeaker) -> bool {
    matches!(
        speaker,
        StationSpeaker::Tower | StationSpeaker::Approach | StationSpeaker::Departure
    )
}

/// The line a controller repeats for a pilot's "say again" on the air: the
/// last transmission addressed to us, fresh, on the SAME frequency (a region
/// switch or a retune gets a new controller, who never said it). With COM1
/// unknown on either side, the tuned position must match instead, failing
/// open like the rest of the radio. It is always the controller's LAST word
/// to us, so a clearance since cancelled is never repeated: the cancellation
/// was itself a transmission, and it is the one on file.
pub fn controller_repeat(
    line: Option<&ControllerLine>,
    khz_now: Option<i64>,
    tuned_now: Option<StationSpeaker>,
    now: Instant,
) -> Option<&ControllerLine> {
    let line = line?;
    if now.duration_since(line.at) >= RADIO_REPEAT_TTL {
        return None;
    }
    let same_freq = match (line.khz, khz_now) {
        (Some(a), Some(b)) => (a - b).abs() <= 5,
        _ => tuned_now.is_none_or(|t| t == line.speaker),
    };
    same_freq.then_some(line)
}

/// Record a controller line addressed to us.
fn log_controller_line(inner: &Inner, text: &str, speaker: StationSpeaker, at: Instant) {
    if !is_controller(speaker) || text.trim().is_empty() {
        return;
    }
    let khz = inner.com1_khz.load(std::sync::atomic::Ordering::Relaxed);
    *inner.radio_log.lock().unwrap_or_else(|e| e.into_inner()) = Some(ControllerLine {
        text: text.trim().to_string(),
        speaker,
        khz: (khz != i64::MIN).then_some(khz),
        at,
    });
}

/// A line the copilot was cut off saying stays owed this long. Past it the
/// moment has gone, and "what were you saying?" gets the newest line instead.
pub const UNSAID_TTL: std::time::Duration = std::time::Duration::from_secs(90);

/// The copilot's own speech, remembered for the pilot's recovery words. A PTT
/// press drops every crew line (mic hygiene) and "release rejoins LIVE audio,
/// no replay": the recovery is procedural, "say again". But "say again" only
/// ever knew the last command READBACK, so a caution the pilot keyed over
/// ("gear's still up") was gone for good, and the answer was an older line.
/// Here, `unsaid` holds copilot lines a press cut or drained, and radio
/// remainders that perished waiting; `last` is the newest line the copilot
/// VOLUNTEERED (an alert, reminder or cue that actually started playing).
/// Blocking replies stay the main loop's `LAST_READBACK`. Cabin and station
/// lines are never kept here: this is the right seat's memory.
#[derive(Default)]
pub struct SpeechMemory {
    last: Option<(String, Instant)>,
    unsaid: Vec<(String, Instant)>,
}

impl SpeechMemory {
    /// A line started playing. Volunteered lines become `last`; any line that
    /// is being said now is no longer owed (a stepped-on remainder resumed).
    fn started(&mut self, text: &str, volunteered: bool, now: Instant) {
        self.unsaid.retain(|(t, _)| t != text);
        if volunteered {
            self.last = Some((text.to_string(), now));
        }
    }

    /// A line the pilot never heard to the end: cut by a press, drained from
    /// the queue by one, or a stepped-on remainder that went stale.
    fn cut(&mut self, text: &str, now: Instant) {
        let text = text.trim();
        if text.is_empty() {
            return;
        }
        self.unsaid.retain(|(t, _)| t != text);
        self.unsaid.push((text.to_string(), now));
        self.unsaid
            .retain(|(_, at)| now.duration_since(*at) < UNSAID_TTL);
    }

    fn unsaid(&self, now: Instant) -> Vec<(String, Instant)> {
        self.unsaid
            .iter()
            .filter(|(_, at)| now.duration_since(*at) < UNSAID_TTL)
            .cloned()
            .collect()
    }
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
            radio_queue: Mutex::new(Queue {
                heap: BinaryHeap::new(),
                seq: 0,
                shutdown: false,
            }),
            radio_cv: Condvar::new(),
            speaking: Mutex::new((false, false)),
            station_seed: std::sync::atomic::AtomicUsize::new(0),
            station_local: Mutex::new(Vec::new()),
            hold: std::sync::atomic::AtomicBool::new(false),
            now_playing: Mutex::new(None),
            intercom_playing: Mutex::new(None),
            tuned_station: Mutex::new(None),
            com1_known: std::sync::atomic::AtomicBool::new(false),
            radio_powered: std::sync::atomic::AtomicBool::new(true),
            crashed: std::sync::atomic::AtomicBool::new(false),
            replay: std::sync::atomic::AtomicBool::new(false),
            paused: std::sync::atomic::AtomicBool::new(false),
            spoken: Mutex::new(Vec::new()),
            pilot_swap: Mutex::new(None),
            cabin_swap: Mutex::new(None),
            station_range_nm: std::sync::atomic::AtomicU32::new(f32::NAN.to_bits()),
            status: Mutex::new(None),
            memory: Mutex::new(SpeechMemory::default()),
            com1_khz: std::sync::atomic::AtomicI64::new(i64::MIN),
            radio_log: Mutex::new(None),
        });
        // Independent channel volumes (a real audio panel's COM vs intercom
        // knobs). Applied in tts::play per thread channel.
        tts::set_volumes(config.voice_volume, config.atc_volume);
        let gate = MicGate::new();
        let intercom = inner.clone();
        let intercom_gate = gate.clone();
        let intercom_cfg = config.clone();
        std::thread::Builder::new()
            .name("voice-intercom".into())
            .spawn(move || intercom_loop(&intercom, intercom_cfg, intercom_gate))
            .expect("spawn intercom speaker thread");
        let radio = inner.clone();
        let radio_gate = gate.clone();
        std::thread::Builder::new()
            .name("voice-radio".into())
            .spawn(move || radio_loop(&radio, config, radio_gate))
            .expect("spawn radio speaker thread");
        Voice { inner, gate }
    }

    /// Cast the active airport's controllers: seed 0 = the configured
    /// per-position voices (the curated home field); any other seed picks a
    /// stable per-airport crew from the voice pool, with `locals` (the
    /// country's accent voices, when it has any) seated as the tower and the
    /// ATIS reader — different people when the roster allows. The warm
    /// sessions rebuild on the next station utterance.
    pub fn set_station_cast(&self, seed: usize, locals: Vec<String>) {
        *self
            .inner
            .station_local
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = locals;
        self.inner
            .station_seed
            .store(seed, std::sync::atomic::Ordering::Relaxed);
    }

    /// Publish the range to the active station (monitor tick). `None` =
    /// unknown, which plays as a perfect signal — fail open, like radio power.
    pub fn set_station_range(&self, nm: Option<f64>) {
        let bits = nm.map_or(f32::NAN, |v| v as f32).to_bits();
        self.inner
            .station_range_nm
            .store(bits, std::sync::atomic::Ordering::Relaxed);
    }

    /// Swap the copilot's own voice live (the panel's Settings picker writes
    /// an index, main resolves it to a model path and calls this). Takes
    /// effect before the next intercom utterance; a failed model load keeps
    /// the current voice (see [`Speaker::set_voice`]).
    pub fn set_pilot_voice(&self, spec: impl Into<String>) {
        *self
            .inner
            .pilot_swap
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = Some(spec.into());
    }

    /// Swap the cabin crew's voice live (the panel's second picker). The
    /// cabin's engine is built lazily, so a swap only drops the old one: the
    /// next cabin line builds the new voice.
    pub fn set_cabin_voice(&self, spec: impl Into<String>) {
        *self
            .inner
            .cabin_swap
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = Some(spec.into());
    }

    /// The mic gate this arbiter raises while speaking — hand it to the listener
    /// so it can mute itself and the copilot never hears its own voice.
    pub fn mic_gate(&self) -> MicGate {
        self.gate.clone()
    }

    /// True while EITHER channel is speaking (the mic gate is the OR of the
    /// intercom and the radio) — the barge-in watcher only interrupts when
    /// there's actually audio to cut.
    pub fn is_speaking(&self) -> bool {
        self.gate.is_speaking()
    }

    /// Barge-in: stop talking *now*. Drop everything queued (unblocking any caller
    /// waiting on a blocking `say`) and kill the audio currently playing. Used when
    /// the pilot keys PTT or interrupts — the copilot must yield, not talk over them.
    /// Hold or release ALL speech (copilot, controllers, ghosts) — raised
    /// while the pilot's PTT is down: nobody talks over a transmission.
    /// Held utterances stay queued and play on release.
    /// Publish COM1's state each monitor tick: `com1_khz` = the raw frequency
    /// (`None` only when the sim read FAILED), `tuned` = the station it
    /// resolves to (controllers or ATIS), `None` = a good read that's off all
    /// station frequencies. Besides feeding the tail-resume decision, this
    /// CUTS a live station transmission the instant the pilot tunes off its
    /// frequency — a real radio goes silent mid-word when you leave the
    /// frequency, no fade, no "let it finish" ("switch frequency and the
    /// current speech is not cut off", 2026-07-12). No resume, unlike a PTT
    /// step-on: you left, it's gone; "say again" on the new controller.
    /// Publish whether the radio receiver is powered (avionics master on).
    /// `false` suppresses station audio (radio) — cutting a live transmission
    /// mid-word and gating queued ones; the copilot's intercom voice is
    /// unaffected. The monitor passes `false` only on a confident read.
    pub fn set_radio_powered(&self, on: bool) {
        self.inner
            .radio_powered
            .store(on, std::sync::atomic::Ordering::Relaxed);
        // The receiver dies NOW, mid-word — same as the retune cut in
        // `set_com1`, not just a gate on future dequeues. Only station audio
        // is ever published to `now_playing`, so the copilot's intercom
        // voice keeps talking with the panel dark. Idempotent under the
        // monitor's per-tick republish: the first cut leaves it empty.
        if !on
            && self
                .inner
                .now_playing
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .take()
                .is_some()
        {
            crate::tts::stop_channel(crate::tts::Channel::Radio);
        }
    }

    /// Whether the radio receiver/transmitter has power (see
    /// [`Voice::set_radio_powered`]). The command loop checks it before routing
    /// a transmission to the world: a dead panel can't transmit any more than
    /// it can receive. Same fail-open contract — defaults true, false only on
    /// a confident avionics-off read.
    pub fn radio_powered(&self) -> bool {
        self.inner
            .radio_powered
            .load(std::sync::atomic::Ordering::Relaxed)
    }

    /// Is the station talking right now? A crew call rendered with a live
    /// number (the departure check-in's altitude) waits for this to clear
    /// before it reads the gauge, so the number is the one spoken.
    pub fn radio_on_air(&self) -> bool {
        radio_is_on_air(&self.inner)
    }

    /// Publish whether the airframe is crashed (see `Inner::crashed`). Going
    /// crashed cuts a live station transmission mid-word, same as a power
    /// loss — the receiver just became wreckage.
    pub fn set_crashed(&self, crashed: bool) {
        self.inner
            .crashed
            .store(crashed, std::sync::atomic::Ordering::Relaxed);
        if crashed
            && self
                .inner
                .now_playing
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .take()
                .is_some()
        {
            crate::tts::stop_channel(crate::tts::Channel::Radio);
        }
    }

    /// Whether the airframe is crashed. The command loop checks it before
    /// routing a transmission to the world: wreckage doesn't transmit.
    pub fn crashed(&self) -> bool {
        self.inner
            .crashed
            .load(std::sync::atomic::Ordering::Relaxed)
    }

    /// Publish whether the sim is in replay (see `Inner::replay`). Entering a
    /// replay cuts a live station transmission mid-word — the pilot just froze
    /// the world; the controller's voice belongs to the flight, not the film.
    pub fn set_replay(&self, replay: bool) {
        self.inner
            .replay
            .store(replay, std::sync::atomic::Ordering::Relaxed);
        if replay
            && self
                .inner
                .now_playing
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .take()
                .is_some()
        {
            crate::tts::stop_channel(crate::tts::Channel::Radio);
        }
    }

    /// Whether the sim is in replay. The command loop checks it before routing
    /// a transmission to the world: you can't call a tower from a recording.
    pub fn replay(&self) -> bool {
        self.inner.replay.load(std::sync::atomic::Ordering::Relaxed)
    }

    /// Publish whether the sim is paused (see `Inner::paused`). No audio cut:
    /// a transmission already on the air finishes its sentence.
    pub fn set_paused(&self, paused: bool) {
        self.inner
            .paused
            .store(paused, std::sync::atomic::Ordering::Relaxed);
    }

    /// Whether the sim is paused. The command loop checks it before routing a
    /// transmission to the world: the world isn't turning, so the FSM must
    /// not move off a frozen flight.
    pub fn paused(&self) -> bool {
        self.inner.paused.load(std::sync::atomic::Ordering::Relaxed)
    }

    pub fn set_com1(&self, com1_khz: Option<i64>, tuned: Option<StationSpeaker>) {
        // A failed read is no information — never cut a live transmission on
        // a fluke, and don't poison the stored tuning.
        let Some(khz) = com1_khz else {
            return;
        };
        self.inner
            .com1_khz
            .store(khz, std::sync::atomic::Ordering::Relaxed);
        *self
            .inner
            .tuned_station
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = tuned;
        // A monitor is publishing: from here on the speaker may trust the
        // stored tuning to drop queued station audio (fail-open until now).
        self.inner
            .com1_known
            .store(true, std::sync::atomic::Ordering::Relaxed);
        // Is a station on the air, and have we tuned off its frequency?
        let cut = self
            .inner
            .now_playing
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .is_some_and(|&s| !audible_now(tuned, s));
        if cut {
            // Drop it (no resume) and kill the audio.
            *self
                .inner
                .now_playing
                .lock()
                .unwrap_or_else(|e| e.into_inner()) = None;
            crate::tts::stop_channel(crate::tts::Channel::Radio);
        }
    }

    pub fn set_hold(&self, held: bool) {
        self.inner
            .hold
            .store(held, std::sync::atomic::Ordering::Relaxed);
        if !held {
            // Release: whatever was muted during the hold is simply gone — no
            // mid-word replay of a stepped-on transmission. Like a real
            // walkie-talkie: you key up, the frequency's silent to you; you let
            // go, you hear what's happening NOW. Recovery stays procedural
            // ("say again"). Wake both speakers to drain the held backlogs.
            self.inner.cv.notify_one();
            self.inner.radio_cv.notify_one();
        }
    }

    pub fn shutup(&self) {
        // Drop the on-air station transmission (no resume) and drain BOTH
        // queues — a PTT press silences the whole cockpit (mic hygiene).
        *self
            .inner
            .now_playing
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = None;
        // What the copilot was saying, and had queued, is owed: the press
        // silenced it, it did not answer it ("what were you saying?").
        let now = Instant::now();
        let playing = self
            .inner
            .intercom_playing
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .filter(|p| !p.cabin)
            .map(|p| p.text.clone());
        {
            let mut memory = self.inner.memory.lock().unwrap_or_else(|e| e.into_inner());
            if let Some(text) = playing {
                memory.cut(&text, now);
            }
            let q = self.inner.queue.lock().unwrap();
            let mut queued: Vec<&Utterance> = q
                .heap
                .iter()
                .filter(|u| !u.cabin && !u.negative && u.station.is_none())
                .collect();
            queued.sort_by_key(|u| u.seq);
            for utt in queued {
                memory.cut(&utt.text, now);
            }
        }
        {
            // The controller transmitted what was queued for us; the press
            // blocked it. It is what the frequency's "say again" repeats.
            let q = self.inner.radio_queue.lock().unwrap();
            let mut queued: Vec<&Utterance> = q.heap.iter().filter(|u| !u.overheard).collect();
            queued.sort_by_key(|u| u.seq);
            for utt in queued {
                if let Some(who) = utt.station {
                    log_controller_line(&self.inner, &utt.text, who, now);
                }
            }
        }
        for queue in [&self.inner.queue, &self.inner.radio_queue] {
            let mut q = queue.lock().unwrap();
            for utt in q.heap.drain() {
                if let Some(latch) = utt.done {
                    *latch.spoken.lock().unwrap() = true;
                    latch.cv.notify_all();
                }
            }
        }
        // The in-flight utterances (already popped by their speakers) end when
        // their audio is killed: speak() returns and each speaker signals its
        // latch and clears its own speaking flag.
        crate::tts::stop_playback();
        self.gate.set_speaking(false);
    }

    /// Copilot lines the pilot never heard to the end, oldest first, within
    /// [`UNSAID_TTL`].
    pub fn unsaid(&self) -> Vec<(String, Instant)> {
        self.inner
            .memory
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .unsaid(Instant::now())
    }

    /// The owed lines have been said again: owed no longer.
    pub fn forget_unsaid(&self) {
        self.inner
            .memory
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .unsaid
            .clear();
    }

    /// The newest line the copilot volunteered (not a reply), with when it
    /// started playing.
    pub fn last_volunteered(&self) -> Option<(String, Instant)> {
        self.inner
            .memory
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .last
            .clone()
    }

    /// An airframe swap: the old aircraft's owed lines die with it, and so
    /// does the controller's last word to it.
    pub fn forget_speech(&self) {
        *self.inner.memory.lock().unwrap_or_else(|e| e.into_inner()) = SpeechMemory::default();
        *self
            .inner
            .radio_log
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = None;
    }

    /// The last controller transmission addressed to us, whenever it was.
    pub fn last_controller_line(&self) -> Option<ControllerLine> {
        self.inner
            .radio_log
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
    }

    /// What the controller on COM1 now repeats for a pilot's "say again", if
    /// anything (see [`controller_repeat`]).
    pub fn repeat_on_frequency(&self) -> Option<ControllerLine> {
        let khz = self
            .inner
            .com1_khz
            .load(std::sync::atomic::Ordering::Relaxed);
        let tuned = *self
            .inner
            .tuned_station
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        let log = self.last_controller_line();
        controller_repeat(
            log.as_ref(),
            (khz != i64::MIN).then_some(khz),
            tuned,
            Instant::now(),
        )
        .cloned()
    }

    /// Dismiss only the crew side: queued copilot/cabin lines are dropped and
    /// current intercom playback is cut, while any radio transmission remains
    /// owned by the station channel. Used when switching into ATC-only mode.
    pub fn dismiss_crew(&self) {
        {
            let mut q = self.inner.queue.lock().unwrap();
            for utt in q.heap.drain() {
                fire_latch(utt.done);
            }
        }
        crate::tts::stop_channel(crate::tts::Channel::Intercom);
        set_channel_speaking(&self.inner, &self.gate, false, false);
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
        said.iter()
            .filter(|(t, _)| t.elapsed() < ECHO_MEMORY)
            .any(|(_, text)| {
                crate::phrase::word_similarity(heard, text) >= ECHO_SIMILARITY
                    || (long && crate::phrase::word_containment(heard, text) >= 0.85)
            })
    }

    fn enqueue(
        &self,
        priority: Priority,
        text: String,
        done: Option<Arc<Latch>>,
        chime: bool,
        stale_by: Option<Instant>,
    ) {
        // No em/en dashes reach the TTS — they trip it up. One chokepoint for
        // every spoken utterance (readbacks, monitor callouts, prompts).
        let text = crate::phrase::scrub_dashes(&text);
        self.push(Utterance {
            priority,
            seq: 0,
            text,
            chime,
            negative: false,
            station: None,
            overheard: false,
            cabin: false,
            stale_by,
            resume: None,
            done,
        });
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
        self.push(Utterance {
            priority,
            seq: 0,
            text,
            chime: false,
            negative: false,
            station: Some(speaker),
            overheard: false,
            cabin: false,
            stale_by: None,
            resume: None,
            done,
        });
    }

    /// Push a fully-formed utterance onto its channel's queue (stamping the
    /// FIFO seq under the lock): station utterances go to the radio, everything
    /// else to the intercom. The single enqueue chokepoint for speech and tones.
    fn push(&self, mut utt: Utterance) {
        if utt.station.is_none() {
            return push_intercom(&self.inner, utt);
        }
        let mut q = self.inner.radio_queue.lock().unwrap();
        utt.seq = q.seq;
        q.seq += 1;
        q.heap.push(utt);
        drop(q);
        self.inner.radio_cv.notify_one();
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
            overheard: false,
            cabin: false,
            stale_by: None,
            resume: None,
            done: Some(latch.clone()),
        });
        let mut spoken = latch.spoken.lock().unwrap();
        while !*spoken {
            spoken = latch.cv.wait(spoken).unwrap();
        }
    }

    /// Queue an utterance and return immediately (monitor alerts).
    ///
    /// No crew, no intercom: in ATC-only mode every fire-and-forget alert is
    /// the copilot's voice (carry-forward briefs, gyro sync, the crew branch
    /// of the world watches), and the copilot went home — dropped here, at
    /// the one door they all walk through, so no watch can forget the gate.
    /// Deliberate blocking replies (`say`) are NOT gated: the main loop
    /// mode-routes those itself, and the "copilot is off" note must speak.
    #[allow(dead_code)] // used by the monitor (next milestone)
    pub fn alert(&self, priority: Priority, text: impl Into<String>) {
        if !crate::mode::copilot_enabled() {
            return;
        }
        let text = text.into();
        crate::auto_status::note_said(&text);
        self.mirror_to_panel(priority, &text);
        // Narration perishes (`ADVISORY_STALE_S`); anything sterner waits its turn.
        let stale_by = match priority {
            Priority::Advisory => {
                Some(Instant::now() + std::time::Duration::from_secs(ADVISORY_STALE_S))
            }
            _ => None,
        };
        self.enqueue(priority, text, None, false, stale_by);
    }

    /// Wire the panel transcript sink, so background-loop speech shows there.
    pub fn set_status(&self, sink: crate::plugin_status::StatusSink) {
        *self.inner.status.lock().unwrap_or_else(|e| e.into_inner()) = Some(sink);
    }

    /// Mirror a spoken line onto the panel transcript — the ONE place a spoken
    /// line reaches the panel, so callers must NOT also post it (that was the
    /// duplicate-text bug). Advisory rides the green action slot (the crew's
    /// own voice); caution/warning use the coloured alert slot so the chip keeps
    /// its amber/red salience. Cheap, non-blocking, a no-op when unwired.
    fn mirror_to_panel(&self, priority: Priority, text: &str) {
        if let Some(s) = self
            .inner
            .status
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
        {
            match priority {
                Priority::Caution => s.alert(crate::plugin_status::AlertLevel::Caution, text),
                Priority::Warning => s.alert(crate::plugin_status::AlertLevel::Warning, text),
                Priority::Advisory | Priority::Readback => s.action(text),
            }
        }
    }

    /// Queue a PERISHABLE utterance: speak it by `stale_by` or drop it at
    /// dequeue. For time-critical guidance (taxi cues) where late is worse
    /// than never. Crew voice — gated like `alert`.
    pub fn alert_stale_by(&self, priority: Priority, text: impl Into<String>, stale_by: Instant) {
        if !crate::mode::copilot_enabled() {
            return;
        }
        let text = crate::phrase::scrub_dashes(&text.into());
        crate::auto_status::note_said(&text);
        self.push(Utterance {
            priority,
            seq: 0,
            text,
            chime: false,
            negative: false,
            station: None,
            overheard: false,
            cabin: false,
            stale_by: Some(stale_by),
            resume: None,
            done: None,
        });
    }

    /// Queue a CABIN CREW utterance and block until spoken (interphone
    /// replies to an addressed call — the loop must not listen over the
    /// answer). Always chimed: on a real interphone the call tone comes
    /// before the voice, and the chime is also what tells the pilot this
    /// voice is from the back, not the right seat. Crew-side: gated like
    /// `alert` (no crew in ATC-only, cabin included).
    pub fn say_cabin(&self, priority: Priority, text: impl Into<String>) {
        if !crate::mode::copilot_enabled() {
            return;
        }
        let latch = Arc::new(Latch {
            spoken: Mutex::new(false),
            cv: Condvar::new(),
        });
        let text = crate::phrase::scrub_dashes(&text.into());
        self.push(Utterance {
            priority,
            seq: 0,
            text,
            chime: true,
            negative: false,
            station: None,
            overheard: false,
            cabin: true,
            stale_by: None,
            resume: None,
            done: Some(latch.clone()),
        });
        let mut spoken = latch.spoken.lock().unwrap();
        while !*spoken {
            spoken = latch.cv.wait(spoken).unwrap();
        }
    }

    /// Queue a CABIN CREW utterance and return immediately (the monitor's
    /// cabin watches: owed secure reports, the seatbelt-sign call). Chimed
    /// and gated like [`Voice::say_cabin`].
    pub fn alert_cabin(&self, priority: Priority, text: impl Into<String>) {
        if !crate::mode::copilot_enabled() {
            return;
        }
        let text = crate::phrase::scrub_dashes(&text.into());
        self.push(Utterance {
            priority,
            seq: 0,
            text,
            chime: true,
            negative: false,
            station: None,
            overheard: false,
            cabin: true,
            stale_by: None,
            resume: None,
            done: None,
        });
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

    /// Queue a station line addressed to ANOTHER aircraft (the controller's
    /// half of an overheard exchange): aired like any other, never logged as
    /// the controller's last word to us.
    pub fn alert_station_overheard(
        &self,
        priority: Priority,
        speaker: StationSpeaker,
        text: impl Into<String>,
    ) {
        let text = crate::phrase::scrub_dashes(&text.into());
        tracing::info!(transmitting = %text, "station (overheard)");
        self.push(Utterance {
            priority,
            seq: 0,
            text,
            chime: false,
            negative: false,
            station: Some(speaker),
            overheard: true,
            cabin: false,
            stale_by: None,
            resume: None,
            done: None,
        });
    }

    /// Queue an utterance preceded by the attention chime, and return immediately.
    /// Used by reminders: a short beep, then the heads-up. Crew voice — a
    /// reminder set before the pilot flipped to ATC-only stays unspoken.
    pub fn chime(&self, priority: Priority, text: impl Into<String>) {
        if !crate::mode::copilot_enabled() {
            return;
        }
        self.enqueue(priority, text.into(), None, true, None);
    }

    /// Queue an utterance and block until it has been spoken (command readbacks:
    /// the loop must wait so it doesn't listen over its own voice).
    pub fn say(&self, priority: Priority, text: impl Into<String>) {
        let latch = Arc::new(Latch {
            spoken: Mutex::new(false),
            cv: Condvar::new(),
        });
        let text = text.into();
        crate::auto_status::note_said(&text);
        self.enqueue(priority, text, Some(latch.clone()), false, None);
        let mut spoken = latch.spoken.lock().unwrap();
        while !*spoken {
            spoken = latch.cv.wait(spoken).unwrap();
        }
    }

    /// Queue a CREW utterance and block until spoken: the checklist flow's
    /// pacing (one item plays before the next is decided, off the command
    /// loop). Crew-gated and panel-mirrored like `alert`, blocking like `say`.
    pub fn say_crew(&self, priority: Priority, text: impl Into<String>) {
        if !crate::mode::copilot_enabled() {
            return;
        }
        let text = text.into();
        self.mirror_to_panel(priority, &text);
        self.say(priority, text);
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
        // NOTE: no mirror_to_panel here — station lines routed through the main
        // loop already reporter.log to the panel; mirroring here double-wrote
        // the tower's reply to the transcript (2026-07-23).
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
        /// This person's pitch/pace factor (see [`crate::radiofx::voice_shift`]);
        /// 1.0 = disabled.
        shift: f32,
    },
    Say(Box<Config>),
}

impl StationVoice {
    fn build(config: &Config, model: &str, person: u64) -> Self {
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
                    // intelligible. Every non-English model is an accent (the
                    // German, Dutch and Portuguese seats of 2026-09-22 too).
                    let pace = station_pace(model);
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
                    let shift = if config.atc_voice_shift {
                        crate::radiofx::shift_factor(person)
                    } else {
                        1.0
                    };
                    tracing::info!(
                        model = %model,
                        squelch = if tail.is_some() { "recorded" } else { "synthesized" },
                        shift,
                        "station voice: warm ONNX session + radio effect"
                    );
                    return StationVoice::Onnx {
                        engine: Box::new(engine),
                        tail,
                        transmissions: 0,
                        shift,
                    };
                }
                Err(e) => {
                    tracing::warn!("station voice load failed ({e:#}); falling back to `say`");
                }
            }
        }
        StationVoice::Say(Box::new(Config {
            tts_backend: "say".into(),
            say_voice: config.atc_voice.clone(),
            say_rate: config.atc_rate.clone(),
            ..Config::default()
        }))
    }

    /// Speak `text` as this station at received signal `quality` (see
    /// [`crate::radiofx::signal_quality`]; 1.0 = full quieting). `on_start`
    /// fires with the fully processed sample buffer just before playback
    /// begins: the arbiter publishes it as the on-air transmission so a PTT
    /// press can step on it and the release can play the remainder (ONNX
    /// only; the `say` fallback has no buffer to resume, so a stepped-on call
    /// there is simply lost, as before).
    fn speak(
        &mut self,
        text: &str,
        quality: f32,
        on_start: impl FnOnce(&[f32], u32),
    ) -> anyhow::Result<()> {
        match self {
            #[cfg(feature = "onnx-tts")]
            StationVoice::Onnx {
                engine,
                tail,
                transmissions,
                shift,
            } => {
                *transmissions = transmissions.wrapping_add(1);
                let variant = *transmissions;
                let tail = tail.clone();
                let shift = *shift;
                let samples = engine.render_with(text, move |s, r| {
                    // The person's voice shifts; the receiver's clicks, noise
                    // floor, and squelch stay natural — so shift first.
                    crate::radiofx::voice_shift(s, r, shift);
                    match &tail {
                        Some(t) => crate::radiofx::radioize_with_tail_q(s, r, variant, t, quality),
                        None => crate::radiofx::radioize_variant_q(s, r, variant, quality),
                    }
                })?;
                on_start(&samples, engine.sample_rate());
                tts::play_pcm(&samples, engine.sample_rate())
            }
            StationVoice::Say(config) => {
                let _ = (on_start, quality);
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
    Subprocess(Box<Config>),
    /// In-process warm ONNX session (feature `onnx-tts`, `XCOPILOT_TTS=onnx`) — the
    /// default voice backend.
    #[cfg(feature = "onnx-tts")]
    Onnx(Box<crate::tts_onnx::OnnxTts>),
}

/// What the copilot's voice ACTUALLY resolved to, for the health screen:
/// backend plus model, after every fallback. Requested and effective differ on
/// exactly the days it matters (a model that failed to load drops to `say`,
/// which is audible but only diagnosable in the boot log), so the screen reads
/// this rather than `config.tts_backend`.
static PILOT_ENGINE: std::sync::Mutex<Option<(String, String)>> = std::sync::Mutex::new(None);

fn publish_pilot_engine(backend: &str, model: &str) {
    let stem = std::path::Path::new(model)
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| model.to_string());
    *PILOT_ENGINE.lock().unwrap_or_else(|e| e.into_inner()) = Some((backend.to_string(), stem));
}

/// The copilot's live (backend, voice) pair, once the speaker thread has built
/// its engine. `None` until then.
pub fn pilot_engine() -> Option<(String, String)> {
    PILOT_ENGINE
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()
}

impl Speaker {
    fn build(mut config: Config) -> Self {
        #[cfg(feature = "onnx-tts")]
        if config.tts_backend == "onnx" {
            match crate::tts_onnx::OnnxTts::load(&config.piper_voice) {
                Ok(engine) => {
                    tracing::info!(model = %config.piper_voice, "TTS: warm in-process ONNX session");
                    publish_pilot_engine("onnx", &config.piper_voice);
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
            tracing::warn!(
                "onnx TTS requested but not built in (need the onnx-tts feature); using `say`"
            );
            config.tts_backend = "say".into();
        }
        // Always announce the resolved backend so the active voice is never a
        // guess — the `say`/`piper` subprocess paths were previously silent.
        tracing::info!(backend = %config.tts_backend, "TTS: subprocess backend (per-utterance)");
        publish_pilot_engine(
            &config.tts_backend,
            if config.tts_backend == "piper" {
                &config.piper_voice
            } else {
                &config.say_voice
            },
        );
        Speaker::Subprocess(Box::new(config))
    }

    /// Speak `text`. The warm ONNX path renders first and hands the buffer to
    /// `on_start` before playback, so the arbiter can step on the line and
    /// resume its remainder; the subprocess voices have no buffer to offer.
    /// `on_start` returns whether to go ahead and play: a line stepped on
    /// during its own render has already been re-queued whole, and playing
    /// this copy too would be the double speech the step-on exists to stop.
    fn speak(
        &mut self,
        text: &str,
        on_start: impl FnOnce(Arc<Vec<f32>>, u32) -> bool,
    ) -> anyhow::Result<()> {
        match self {
            Speaker::Subprocess(config) => {
                let _ = on_start;
                tts::speak(config, text)
            }
            #[cfg(feature = "onnx-tts")]
            Speaker::Onnx(engine) => {
                let rate = engine.sample_rate();
                let samples = Arc::new(engine.render_with(text, |_, _| {})?);
                if !on_start(samples.clone(), rate) {
                    return Ok(());
                }
                tts::play_pcm(&samples, rate)
            }
        }
    }

    /// Swap to a different voice model (the panel picker's live change). The
    /// warm ONNX path loads the new model FIRST and keeps the current engine
    /// if the load fails — a bad pick must cost nothing, never drop the
    /// copilot to the `say` fallback mid-flight. The subprocess paths read
    /// the model path per utterance, so a config update is the whole swap.
    fn set_voice(&mut self, spec: String) {
        match self {
            Speaker::Subprocess(config) => {
                tracing::info!(model = %spec, "TTS: pilot voice swapped (subprocess)");
                publish_pilot_engine(&config.tts_backend, &spec);
                config.piper_voice = spec;
            }
            #[cfg(feature = "onnx-tts")]
            Speaker::Onnx(engine) => match crate::tts_onnx::OnnxTts::load(&spec) {
                Ok(fresh) => {
                    tracing::info!(model = %spec, "TTS: pilot voice swapped (warm ONNX)");
                    publish_pilot_engine("onnx", &spec);
                    **engine = fresh;
                }
                Err(e) => {
                    tracing::error!(
                        "pilot voice swap to {spec} failed ({e:#}); keeping the current voice"
                    );
                }
            },
        }
    }
}

/// Crew discipline: whether the intercom may START `top` while the radio is
/// (or isn't) on the air. Routine lines — advisories, readbacks, cautions —
/// wait for the frequency to go quiet, like a crew member pausing for a
/// transmission; `Warning` (stall, terrain, airspeed) breaks through
/// immediately, because a real copilot talks over the radio when you're
/// about to die. Speech already playing is never cut by this — it only gates
/// what STARTS.
fn intercom_may_start(top: Priority, radio_on_air: bool) -> bool {
    !radio_on_air || top >= Priority::Warning
}

/// Flip one channel's speaking flag and recompute the mic gate (the OR of the
/// two channels — the mic must stay muted while ANY voice is on the speakers).
/// Lowering the RADIO flag also wakes the intercom: a deferred routine line
/// may start now that the frequency is quiet.
fn set_channel_speaking(inner: &Inner, gate: &MicGate, radio: bool, on: bool) {
    let mut s = inner.speaking.lock().unwrap_or_else(|e| e.into_inner());
    if radio {
        s.1 = on;
    } else {
        s.0 = on;
    }
    gate.set_speaking(s.0 || s.1);
    drop(s);
    if radio && !on {
        inner.cv.notify_one();
    }
}

fn radio_is_on_air(inner: &Inner) -> bool {
    inner.speaking.lock().unwrap_or_else(|e| e.into_inner()).1
}

/// Signal a blocking caller's latch (dropped utterances must fire it too, or
/// a blocking `say`/`say_station` caller deadlocks).
fn fire_latch(latch: Option<Arc<Latch>>) {
    if let Some(latch) = latch {
        *latch.spoken.lock().unwrap() = true;
        latch.cv.notify_all();
    }
}

/// Remember what just finished playing so the listener can reject this audio
/// if it bleeds back through the mic (both channels feed the same echo
/// memory: the tower's voice coming back must not parse as a pilot command
/// any more than the copilot's). Prunes the old.
fn remember_spoken(inner: &Inner, text: &str) {
    let mut said = inner.spoken.lock().unwrap();
    // A phrase the copilot COACHED ("say navigate to follow the plan") is
    // the one thing the pilot is about to say on purpose; it must not be
    // remembered as our own voice.
    said.push((Instant::now(), crate::phrase::without_coached_phrase(text)));
    said.retain(|(t, _)| t.elapsed() < ECHO_MEMORY);
}

/// A killed player (SIGTERM) is an intentional barge-in cut-off, not a
/// failure — log it quietly.
fn log_tts_err(what: &str, e: anyhow::Error) {
    let msg = format!("{e:#}");
    if msg.contains("SIGTERM") || msg.contains("signal: 15") {
        tracing::debug!("{what} interrupted (barge-in): {msg}");
    } else {
        tracing::warn!("{what} failed: {msg}");
    }
}

/// The INTERCOM speaker thread: the copilot's own voice, plus the chime and
/// negative tones. Pops the highest-priority utterance (blocking until one is
/// available AND crew discipline allows it — see [`intercom_may_start`]),
/// speaks it to completion, signals any blocked caller, repeats.
///
/// The channel's speaking flag is raised for the whole run and only lowered
/// once the queue drains — so a burst of utterances keeps the mic gated end
/// to end, with no momentary gap for it to slip through.
fn intercom_loop(inner: &Arc<Inner>, config: Config, gate: MicGate) {
    let chime_path = resolve_chime(&config.chime_path);
    let negative_path = resolve_negative(&config.negative_tone_path);
    // The cabin crew's voice: same channel, different person. Built LAZILY on
    // the first cabin utterance — most flights (and every airframe without a
    // cabin_crew profile flag) never pay the model load.
    let mut cabin_config = {
        let mut c = config.clone();
        c.piper_voice = c.cabin_piper_voice.clone();
        c
    };
    let mut cabin_speaker: Option<Speaker> = None;
    let mut speaker = Speaker::build(config);
    loop {
        let mut q = inner.queue.lock().unwrap();
        let utt = loop {
            if q.shutdown {
                set_channel_speaking(inner, &gate, false, false);
                return;
            }
            // Any PTT held: nobody speaks (mic hygiene). Otherwise start the
            // top utterance only when discipline allows: routine lines wait
            // out a station transmission; a Warning goes now. Peeking the top
            // is sound because the heap's max IS the most urgent thing here.
            if !inner.hold.load(std::sync::atomic::Ordering::Relaxed) {
                let may_start = q
                    .heap
                    .peek()
                    .is_some_and(|top| intercom_may_start(top.priority, radio_is_on_air(inner)));
                if may_start {
                    break q.heap.pop().expect("peeked");
                }
            }
            // Nothing sayable — we're done speaking; let the mic listen again
            // (unless the radio holds the gate). Woken by a push, a hold
            // release, or the radio going quiet.
            set_channel_speaking(inner, &gate, false, false);
            q = inner.cv.wait(q).unwrap();
        };
        drop(q); // release the queue while we speak (which is slow)

        // A pending pilot-voice swap applies here, between utterances — so a
        // test line queued right after the pick already speaks in the new
        // voice. The model load blocks this thread briefly; that's the point
        // (the swap must never race the engine mid-word).
        if let Some(spec) = inner
            .pilot_swap
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .take()
        {
            speaker.set_voice(spec);
        }
        if let Some(spec) = inner
            .cabin_swap
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .take()
        {
            tracing::info!(model = %spec, "TTS: cabin voice swapped");
            cabin_config.piper_voice = spec;
            cabin_speaker = None;
        }

        // Perishable speech that missed its window (including waiting out a
        // radio transmission) is dropped: spoken late it's disinformation,
        // not guidance. The latch still fires — see `fire_latch`.
        if utt.stale_by.is_some_and(|t| Instant::now() > t) {
            tracing::info!(target: "xvl::voice", text = %utt.text, "dropped stale intercom line");
            fire_latch(utt.done);
            continue;
        }

        set_channel_speaking(inner, &gate, false, true);
        // Tone-only: play the negative "unrecognized" cue and skip TTS entirely
        // (nothing to speak, nothing to add to the echo list).
        if utt.negative {
            if let Some(path) = &negative_path {
                if let Err(e) = crate::tts::play(path) {
                    log_tts_err("negative tone", e);
                }
            }
            fire_latch(utt.done);
            continue;
        }
        if utt.chime {
            if let Some(path) = &chime_path {
                if let Err(e) = crate::tts::play(path) {
                    log_tts_err("chime", e);
                }
            }
        }
        // catch_unwind: a panic ANYWHERE in a TTS engine must not kill this
        // thread — blocking callers wait on latches only this thread signals,
        // so a dead speaker deadlocks the whole app (seen live 2026-07-07: the
        // first station utterance panicked, the main loop hung unkillable, and
        // the backtrace sprayed over the raw-mode TUI).
        let who: &mut Speaker = if utt.cabin {
            cabin_speaker.get_or_insert_with(|| Speaker::build(cabin_config.clone()))
        } else {
            &mut speaker
        };
        // What is playing, for the radio to step on: registered before the
        // subprocess voices start (no buffer, so a re-say), replaced with the
        // rendered buffer when the ONNX path hands it over. The record holds
        // the caller's latch; whoever finishes the sentence fires it.
        if !utt.cabin {
            // A line that starts is no longer owed; a VOLUNTEERED one (no
            // blocking caller: alerts, reminders, cues) is what "say again"
            // means when it is newer than the last readback.
            inner
                .memory
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .started(&utt.text, utt.done.is_none(), Instant::now());
        }
        let gen_at_start = tts::channel_generation(tts::Channel::Intercom);
        let register = |audio: Option<Resume>| {
            *inner
                .intercom_playing
                .lock()
                .unwrap_or_else(|e| e.into_inner()) = Some(IntercomPlay {
                priority: utt.priority,
                audio,
                started: Instant::now(),
                text: utt.text.clone(),
                cabin: utt.cabin,
                done: utt.done.clone(),
            });
        };
        let spoke = match &utt.resume {
            Some(r) => {
                register(Some(r.clone()));
                tts::play_pcm(&r.samples, r.rate)
            }
            None => {
                register(None);
                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    who.speak(&utt.text, |samples, rate| {
                        // Stepped on during the render: the interrupt has
                        // already queued this line again whole. Don't play
                        // this copy under the transmission as well.
                        if tts::channel_generation(tts::Channel::Intercom) != gen_at_start {
                            return false;
                        }
                        register(Some(Resume { samples, rate }));
                        true
                    })
                }))
                .unwrap_or_else(|panic| Err(anyhow::anyhow!("TTS panicked: {}", panic_text(panic))))
            }
        };
        // Off the intercom. A record still here means the line finished and
        // the latch is ours to fire; none means the radio stepped on it and
        // the remainder now carries the latch.
        let finished = inner
            .intercom_playing
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .take();
        if let Err(e) = spoke {
            log_tts_err("TTS", e);
        }
        remember_spoken(inner, &utt.text);
        if let Some(rec) = finished {
            fire_latch(rec.done);
        }
    }
}

/// The RADIO speaker thread: every station voice (controllers, ATIS, ghost
/// pilots), serialized among themselves — one frequency, one transmission at
/// a time — but free to overlap the copilot's intercom, like a real audio
/// panel. Tags its playback as the Radio channel so the radio-only cuts
/// (retune, avionics off, crash, replay) never touch the copilot's audio.
fn radio_loop(inner: &Arc<Inner>, config: Config, gate: MicGate) {
    crate::tts::set_thread_channel(crate::tts::Channel::Radio);
    let station_cfg = config;
    // One lazy warm voice per position (tower/approach/departure/ATIS, plus
    // one slot for whichever GHOST pilot spoke last), keyed by model name so
    // a cast change — or a different ghost — rebuilds only that slot. A
    // position that never speaks never loads its model.
    let mut stations: [Option<(String, StationVoice)>; 5] = [None, None, None, None, None];
    let mut stations_seed: usize = 0;
    loop {
        let mut q = inner.radio_queue.lock().unwrap();
        let utt = loop {
            if q.shutdown {
                set_channel_speaking(inner, &gate, true, false);
                return;
            }
            // Any PTT held: the frequency is silent to you while transmitting.
            if !inner.hold.load(std::sync::atomic::Ordering::Relaxed) {
                if let Some(u) = q.heap.pop() {
                    break u;
                }
            }
            set_channel_speaking(inner, &gate, true, false);
            q = inner.radio_cv.wait(q).unwrap();
        };
        drop(q); // release the queue while we speak (which is slow)

        // Dead receiver or tuned away: station audio (controllers, ATIS,
        // party-line traffic) is radio — with the avionics master off you
        // can't hear it, and a transmission still queued behind another
        // voice (or a PTT hold) died with the frequency the moment the
        // pilot retuned (`set_com1` cuts LIVE audio; this catches the
        // queued). Drop it, don't hold it — missed radio is gone, like real
        // life. Checked before the speaking flag is raised so a dropped
        // transmission doesn't even gate the mic, and the done latch still
        // fires so a blocking `say_station` caller is released, never
        // deadlocked. Perishable speech that missed its window drops the
        // same way.
        let stale = utt.stale_by.is_some_and(|t| Instant::now() > t);
        if stale || station_unhearable(inner, utt.station) {
            fire_latch(utt.done);
            continue;
        }

        set_channel_speaking(inner, &gate, true, true);
        interrupt_intercom_for_radio(inner);
        if !utt.overheard {
            if let Some(who) = utt.station {
                log_controller_line(inner, &utt.text, who, Instant::now());
            }
        }
        // A panicking station voice is demoted to the `say` fallback for the
        // rest of the session (same catch_unwind rationale as the intercom).
        let seed = inner
            .station_seed
            .load(std::sync::atomic::Ordering::Relaxed);
        if seed != stations_seed {
            stations = [None, None, None, None, None];
            stations_seed = seed;
        }
        let locals: Vec<String> = inner
            .station_local
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone();
        let who = utt
            .station
            .expect("radio queue only holds station utterances");
        let model = station_model(who, &station_cfg, seed, &locals);
        // Signal quality from the CURRENT range to the station, read per
        // transmission — the same tower fades in as you close on the field.
        // Ghost pilots ride the same value: they're on this field's frequency
        // and pattern, so its range is an honest proxy.
        let range = f32::from_bits(
            inner
                .station_range_nm
                .load(std::sync::atomic::Ordering::Relaxed),
        );
        let quality = crate::radiofx::signal_quality(range.is_finite().then_some(range as f64));
        let spoke = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let slot = &mut stations[who.slot()];
            if slot.as_ref().map(|(m, _)| m.as_str()) != Some(model.as_str()) {
                let person = station_person(&model, who.slot(), seed);
                *slot = Some((
                    model.clone(),
                    StationVoice::build(&station_cfg, &model, person),
                ));
            }
            // Publish the transmission as on-air just before playback, so a
            // PTT press or a retune can CUT it (no resume). Copilot speech
            // is deliberately never published: it's intercom, not radio.
            // Then re-check hearability: a retune or power-off landing
            // during SYNTHESIS falls between the dequeue check and this
            // publish, so `set_com1`/`set_radio_powered` found nothing
            // on the air to cut. The cut generation in `tts::stop_channel`
            // kills even a player that hasn't registered its pid yet.
            slot.as_mut()
                .expect("just built")
                .1
                .speak(&utt.text, quality, |_samples, _rate| {
                    *inner.now_playing.lock().unwrap_or_else(|e| e.into_inner()) = Some(who);
                    if station_unhearable(inner, Some(who)) {
                        crate::tts::stop_channel(crate::tts::Channel::Radio);
                    }
                })
        }))
        .unwrap_or_else(|panic| {
            let what = panic_text(panic);
            tracing::error!("station voice panicked ({what}); demoting to `say` fallback");
            stations[who.slot()] = Some((
                model.clone(),
                StationVoice::Say(Box::new(Config {
                    tts_backend: "say".into(),
                    say_voice: station_cfg.atc_voice.clone(),
                    say_rate: station_cfg.atc_rate.clone(),
                    ..Config::default()
                })),
            ));
            Err(anyhow::anyhow!("TTS panicked: {what}"))
        });
        if let Err(e) = spoke {
            log_tts_err("TTS", e);
        }
        // Off the air, either way: finished playback clears its own slot; a
        // barge-in already stole it (this take then finds None, harmlessly).
        inner
            .now_playing
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .take();
        remember_spoken(inner, &utt.text);
        fire_latch(utt.done);
    }
}

fn panic_text(panic: Box<dyn std::any::Any + Send>) -> String {
    panic
        .downcast_ref::<&str>()
        .map(|s| s.to_string())
        .or_else(|| panic.downcast_ref::<String>().cloned())
        .unwrap_or_else(|| "non-string panic".into())
}

/// An env-tunable f32 with a default (station cadence knobs).
#[cfg(feature = "onnx-tts")]
fn env_f32(name: &str, default: f32) -> f32 {
    std::env::var(name)
        .ok()
        .and_then(|v| v.trim().parse().ok())
        .unwrap_or(default)
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
        Utterance {
            priority,
            seq,
            text: String::new(),
            chime: false,
            negative: false,
            station: None,
            overheard: false,
            cabin: false,
            stale_by: None,
            resume: None,
            done: None,
        }
    }

    #[test]
    fn airport_voice_crews_are_stable_distinct_and_never_home() {
        // Stable per name, never 0 (0 = the home field's curated voices).
        assert_eq!(
            station_voice_seed("Ciudad Real"),
            station_voice_seed("Ciudad Real")
        );
        assert_ne!(station_voice_seed("Ciudad Real"), 0);
        assert_ne!(
            station_voice_seed("Ciudad Real"),
            station_voice_seed("Malaga")
        );
        // With a pool, a seeded airport picks from it — and the four positions
        // land on distinct voices when the pool has at least four entries.
        let cfg = Config {
            atc_piper_voice: "tower.onnx".into(),
            atc_voice_pool: "a.onnx, b.onnx, c.onnx, d.onnx, e.onnx, f.onnx, g.onnx, h.onnx".into(),
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
        .map(|s| station_model(*s, &cfg, seed, &[]))
        .collect();
        let mut unique = crew.clone();
        unique.sort();
        unique.dedup();
        assert_eq!(
            unique.len(),
            4,
            "positions must be distinct people: {crew:?}"
        );
        // Seed 0 = the curated home voices, pool or not.
        assert_eq!(
            station_model(StationSpeaker::Tower, &cfg, 0, &[]),
            "tower.onnx"
        );
        // No pool → curated voices even for a seeded airport.
        let bare = Config {
            atc_piper_voice: "tower.onnx".into(),
            ..Config::default()
        };
        assert_eq!(
            station_model(StationSpeaker::Tower, &bare, seed, &[]),
            "tower.onnx"
        );
        // Country casting: the local accents man the tower and read the
        // ATIS — DIFFERENT people when the roster has two or more ("main
        // thing is that we actually have different voices", 2026-07-12);
        // approach/departure stay neutral (from the pool).
        let locals = vec!["es1.onnx".to_string(), "es2.onnx".to_string()];
        let twr = station_model(StationSpeaker::Tower, &cfg, seed, &locals);
        let atis = station_model(StationSpeaker::Atis, &cfg, seed, &locals);
        assert!(locals.contains(&twr), "{twr}");
        assert!(locals.contains(&atis), "{atis}");
        assert_ne!(twr, atis, "tower and ATIS must be different people");
        assert!(!locals.contains(&station_model(
            StationSpeaker::Approach,
            &cfg,
            seed,
            &locals
        )));
        // A one-voice roster still casts it (tower and ATIS then share).
        let solo = vec!["es1.onnx".to_string()];
        assert_eq!(
            station_model(StationSpeaker::Tower, &cfg, seed, &solo),
            "es1.onnx"
        );
        assert_eq!(
            station_model(StationSpeaker::Atis, &cfg, seed, &solo),
            "es1.onnx"
        );
        // Geography: each country's own language, never a neighbour's -
        // Castilian in Spain and the Canaries, Argentine only in Argentina,
        // and nothing Latin American in Spain.
        for (icao, lang) in [
            ("LEMD", "es_ES"),
            ("GCXO", "es_ES"),
            ("EHAM", "nl_NL"),
            ("LPPT", "pt_PT"),
            ("SAEZ", "es_AR"),
        ] {
            let roster = local_accents_for(icao);
            assert!(!roster.is_empty(), "{icao} has a seat");
            assert!(roster.iter().all(|m| m.contains(lang)), "{icao}");
        }
        assert!(local_accents_for("MMMX")
            .iter()
            .all(|m| m.contains("es_MX")));
        // The countries whose every candidate failed stay neutral.
        for icao in ["LFPB", "EBLG", "LIRF", "EDDV"] {
            assert!(local_accents_for(icao).is_empty(), "{icao}");
        }
        // Accents are voiced slower; English is not an accent.
        assert!(is_accent_model("models/nl_NL-pim-medium.onnx"));
        assert!(is_accent_model("models/es_AR-daniela-high.onnx"));
        assert!(!is_accent_model("models/en_US-libritts_r-medium.onnx#90"));
        assert!(!is_accent_model("models/en_GB-alba-medium.onnx"));
        // And the paces the ear picked.
        assert_eq!(station_pace("models/en_GB-alba-medium.onnx"), 0.8);
        assert_eq!(station_pace("models/en_GB-vctk-medium.onnx#0"), 1.15);
        // A slow reader measured and sped up to the same rate.
        assert!(station_pace("models/en_GB-vctk-medium.onnx#4") < 0.85);
        // Every measured voice is in the pool, and every British pool voice
        // with a measurement is paced by it.
        let pool = crate::config::Config::from_env()
            .expect("config")
            .atc_voice_pool;
        for (spec, _) in MEASURED_PACE {
            assert!(
                pool.contains(&format!("{spec},")) || pool.ends_with(spec),
                "{spec}"
            );
        }
        assert_eq!(station_pace("models/nl_NL-pim-medium.onnx"), 1.1);
        assert!(
            local_accents_for("SBGR").is_empty(),
            "Brazil speaks Portuguese, stays neutral"
        );
        assert!(local_accents_for("EGLL").is_empty());
        assert!(
            local_accents_for("LXGB").is_empty(),
            "home field keeps its curated cast anyway"
        );
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
    fn live_audio_cuts_the_instant_you_tune_off_frequency() {
        use StationSpeaker::*;
        // On the frequency (or its party line): audible, keep playing.
        assert!(audible_now(Some(Approach), Approach));
        assert!(audible_now(Some(Tower), Tower));
        assert!(audible_now(Some(Tower), Ghost { seed: 3 }));
        assert!(audible_now(Some(Atis), Atis));
        // A GOOD read that's off every station frequency (None) → nothing
        // station is audible, so the live transmission is cut.
        assert!(!audible_now(None, Tower));
        assert!(!audible_now(None, Atis));
        assert!(!audible_now(None, Ghost { seed: 3 }));
        // Tuned to a DIFFERENT station: the old one is gone mid-word.
        assert!(!audible_now(Some(Tower), Approach));
        assert!(!audible_now(Some(Approach), Atis));
        assert!(!audible_now(Some(Atis), Tower));
        // A ghost is heard ONLY while the tower is tuned, not on approach.
        assert!(!audible_now(Some(Approach), Ghost { seed: 3 }));
    }

    fn bare_inner() -> Inner {
        Inner {
            queue: Mutex::new(Queue {
                heap: BinaryHeap::new(),
                seq: 0,
                shutdown: false,
            }),
            cv: Condvar::new(),
            radio_queue: Mutex::new(Queue {
                heap: BinaryHeap::new(),
                seq: 0,
                shutdown: false,
            }),
            radio_cv: Condvar::new(),
            speaking: Mutex::new((false, false)),
            status: Mutex::new(None),
            station_seed: std::sync::atomic::AtomicUsize::new(0),
            station_local: Mutex::new(Vec::new()),
            hold: std::sync::atomic::AtomicBool::new(false),
            now_playing: Mutex::new(None),
            intercom_playing: Mutex::new(None),
            tuned_station: Mutex::new(None),
            com1_known: std::sync::atomic::AtomicBool::new(false),
            radio_powered: std::sync::atomic::AtomicBool::new(true),
            crashed: std::sync::atomic::AtomicBool::new(false),
            replay: std::sync::atomic::AtomicBool::new(false),
            paused: std::sync::atomic::AtomicBool::new(false),
            spoken: Mutex::new(Vec::new()),
            pilot_swap: Mutex::new(None),
            cabin_swap: Mutex::new(None),
            station_range_nm: std::sync::atomic::AtomicU32::new(f32::NAN.to_bits()),
            memory: Mutex::new(SpeechMemory::default()),
            com1_khz: std::sync::atomic::AtomicI64::new(i64::MIN),
            radio_log: Mutex::new(None),
        }
    }

    #[test]
    fn station_unhearable_gates_power_and_tuning_but_fails_open() {
        use StationSpeaker::*;
        let inner = bare_inner();
        // Fail open: no monitor has published COM1 yet (tuned_station is
        // None but com1_known is false) — station audio must play.
        assert!(!station_unhearable(&inner, Some(Tower)));
        // Copilot speech is intercom: hearable even with the radio dead.
        inner
            .radio_powered
            .store(false, std::sync::atomic::Ordering::Relaxed);
        assert!(!station_unhearable(&inner, None));
        assert!(station_unhearable(&inner, Some(Tower)));
        inner
            .radio_powered
            .store(true, std::sync::atomic::Ordering::Relaxed);
        // Crashed: wreckage receives nothing, powered bus or not — but the
        // copilot's intercom still plays (someone has to say why it's quiet).
        inner
            .crashed
            .store(true, std::sync::atomic::Ordering::Relaxed);
        assert!(station_unhearable(&inner, Some(Tower)));
        assert!(!station_unhearable(&inner, None));
        inner
            .crashed
            .store(false, std::sync::atomic::Ordering::Relaxed);
        // Replay: the world is a recording — no station audio, but the
        // copilot's intercom still answers.
        inner
            .replay
            .store(true, std::sync::atomic::Ordering::Relaxed);
        assert!(station_unhearable(&inner, Some(Tower)));
        assert!(!station_unhearable(&inner, None));
        inner
            .replay
            .store(false, std::sync::atomic::Ordering::Relaxed);
        // Paused is NOT replay: a transmission already on the air finishes
        // its sentence (the monitor's tick-skip stops anything new; only the
        // transmit paths gate on paused).
        inner
            .paused
            .store(true, std::sync::atomic::Ordering::Relaxed);
        assert!(!station_unhearable(&inner, Some(Tower)));
        inner
            .paused
            .store(false, std::sync::atomic::Ordering::Relaxed);
        // Once tuning is published, only the tuned position (and its ghosts)
        // is hearable; a real read that's off every frequency silences all.
        inner
            .com1_known
            .store(true, std::sync::atomic::Ordering::Relaxed);
        *inner.tuned_station.lock().unwrap() = Some(Approach);
        assert!(!station_unhearable(&inner, Some(Approach)));
        assert!(station_unhearable(&inner, Some(Tower)));
        assert!(station_unhearable(&inner, Some(Ghost { seed: 3 })));
        assert!(!station_unhearable(&inner, None));
        *inner.tuned_station.lock().unwrap() = None;
        assert!(station_unhearable(&inner, Some(Tower)));
        assert!(!station_unhearable(&inner, None));
    }

    #[test]
    fn dropped_station_audio_releases_blocking_callers() {
        // The drop paths must fire the done latch or a blocking `say_station`
        // caller (the tower's replies) deadlocks the app. Both gates: dead
        // receiver, then tuned off the frequency while queued.
        let voice = Voice::new(Config::default());
        voice.set_radio_powered(false);
        voice.say_station(Priority::Readback, StationSpeaker::Tower, "cleared to land");
        voice.set_radio_powered(true);
        voice.set_com1(Some(121_500), Some(StationSpeaker::Approach));
        voice.say_station(
            Priority::Readback,
            StationSpeaker::Tower,
            "line up and wait",
        );
        // Dropped means dropped: neither transmission was actually spoken
        // (finished speech always lands in the echo memory).
        assert!(voice.inner.spoken.lock().unwrap().is_empty());
    }

    #[test]
    fn station_and_copilot_speech_route_to_their_own_channels() {
        // Independent channels (2026-07-14): station utterances must land in
        // the radio queue, copilot speech in the intercom queue — so the two
        // can play concurrently and a radio-only cut never kills the copilot.
        let voice = Voice::new(Config::default());
        voice.alert(Priority::Advisory, "flaps look good");
        voice.alert_station(Priority::Readback, StationSpeaker::Tower, "cleared to land");
        // The speaker threads race us for the queues; assert on what was
        // ENQUEUED by holding the locks straight away and checking whichever
        // items haven't been popped yet still carry the right tag — and that
        // neither queue ever saw the other channel's utterance.
        let q = voice.inner.queue.lock().unwrap();
        assert!(q.heap.iter().all(|u| u.station.is_none()));
        drop(q);
        let rq = voice.inner.radio_queue.lock().unwrap();
        assert!(rq.heap.iter().all(|u| u.station.is_some()));
    }

    #[test]
    fn atc_only_drops_every_fire_and_forget_crew_line() {
        // The 2026-07-15 leak: with the copilot flipped off, background
        // alerts (world-watch crew branches, briefs, taxi cues, reminders)
        // still spoke in the copilot's voice. The gate lives in the arbiter,
        // at the one door they all use — nothing may reach the intercom
        // queue or the echo memory in ATC-only.
        let _guard = crate::mode::TEST_LOCK.lock().unwrap();
        crate::mode::set(crate::mode::Mode::AtcOnly);
        let voice = Voice::new(Config::default());
        voice.alert(
            Priority::Warning,
            "traffic short final and we're on the runway",
        );
        voice.chime(Priority::Advisory, "reminder: carb heat");
        voice.alert_stale_by(
            Priority::Advisory,
            "next left, onto Alpha",
            Instant::now() + std::time::Duration::from_secs(8),
        );
        assert!(voice.inner.queue.lock().unwrap().heap.is_empty());
        assert!(voice.inner.spoken.lock().unwrap().is_empty());
        crate::mode::set(crate::mode::Mode::Both);
    }

    #[test]
    fn advisories_perish_cautions_wait() {
        // Fire-and-forget narration expires if it cannot start in time;
        // cautions, warnings and blocking readbacks never do.
        let _guard = crate::mode::TEST_LOCK.lock().unwrap();
        crate::mode::set(crate::mode::Mode::Both);
        let voice = Voice::new(Config::default());
        voice.set_hold(true);
        let queued_at = Instant::now();
        voice.alert(Priority::Advisory, "Beacon's on, spinning up the engine.");
        voice.alert(Priority::Caution, "Airspeed low.");
        let q = voice.inner.queue.lock().unwrap();
        let by_text = |t: &str| q.heap.iter().find(|u| u.text == t).expect("queued");
        let adv = by_text("Beacon's on, spinning up the engine.");
        let stale = adv.stale_by.expect("an advisory perishes");
        let window = stale.duration_since(queued_at).as_secs();
        assert!(
            (ADVISORY_STALE_S - 1..=ADVISORY_STALE_S).contains(&window),
            "window {window}s"
        );
        assert!(
            by_text("Airspeed low.").stale_by.is_none(),
            "a caution waits"
        );
        drop(q);
        voice.dismiss_crew();
        voice.set_hold(false);
    }

    #[test]
    fn dismiss_crew_drains_intercom_without_touching_radio() {
        let _guard = crate::mode::TEST_LOCK.lock().unwrap();
        crate::mode::set(crate::mode::Mode::Both);
        let voice = Voice::new(Config::default());
        voice.set_hold(true);
        voice.alert(Priority::Advisory, "copilot line still queued");
        voice.alert_cabin(Priority::Advisory, "cabin line still queued");
        voice.alert_station(
            Priority::Advisory,
            StationSpeaker::Tower,
            "station line still queued",
        );

        voice.dismiss_crew();

        assert!(voice.inner.queue.lock().unwrap().heap.is_empty());
        assert_eq!(voice.inner.radio_queue.lock().unwrap().heap.len(), 1);
        voice.set_hold(false);
        crate::mode::set(crate::mode::Mode::Both);
    }

    #[test]
    fn a_radio_call_steps_on_a_routine_line_and_the_remainder_is_queued() {
        // The remainder starts a lead-in before the cut and runs to the end.
        let r = resume_range(5 * 22_050, 22_050, std::time::Duration::from_secs(2)).unwrap();
        assert_eq!(r.start, (1.6 * 22_050.0) as usize);
        assert_eq!(r.end, 5 * 22_050);
        // Cut with nothing worth saying left: nothing owed.
        assert!(resume_range(5 * 22_050, 22_050, std::time::Duration::from_secs(5)).is_none());
        assert!(radio_interrupts(Priority::Advisory) && radio_interrupts(Priority::Caution));
        assert!(
            !radio_interrupts(Priority::Warning),
            "a warning talks over the radio"
        );

        // Live: an advisory two seconds into a five-second line when the
        // tower keys up. The line is cut and its remainder queued, perishable.
        let inner = bare_inner();
        let latch = Arc::new(Latch::default());
        *inner.intercom_playing.lock().unwrap() = Some(IntercomPlay {
            priority: Priority::Advisory,
            audio: Some(Resume {
                samples: Arc::new(vec![0.1_f32; 5 * 22_050]),
                rate: 22_050,
            }),
            started: Instant::now() - std::time::Duration::from_secs(2),
            text: "fuel's balanced, both tanks even".into(),
            cabin: false,
            done: Some(latch.clone()),
        });
        interrupt_intercom_for_radio(&inner);
        assert!(
            inner.intercom_playing.lock().unwrap().is_none(),
            "stepped on"
        );
        let q = inner.queue.lock().unwrap();
        let utt = q.heap.peek().expect("the remainder is queued");
        assert_eq!(utt.priority, Priority::Advisory);
        let r = utt.resume.as_ref().expect("a rendered tail, not a re-say");
        assert!(
            (r.samples.len() as f64 - 3.4 * 22_050.0).abs() < 100.0,
            "{}",
            r.samples.len()
        );
        assert!(utt.stale_by.is_some(), "a fragment out of context expires");
        assert!(utt.done.is_some(), "the caller waits for the remainder");
        assert!(!*latch.spoken.lock().unwrap(), "not released on the cut");
        drop(q);

        // A warning in progress is not touched.
        *inner.intercom_playing.lock().unwrap() = Some(IntercomPlay {
            priority: Priority::Warning,
            audio: None,
            started: Instant::now(),
            text: "airspeed".into(),
            cabin: false,
            done: None,
        });
        interrupt_intercom_for_radio(&inner);
        assert!(
            inner.intercom_playing.lock().unwrap().is_some(),
            "warnings are not stepped on"
        );
        assert_eq!(inner.queue.lock().unwrap().heap.len(), 1);

        // A subprocess voice has no buffer: the line is said again whole.
        *inner.intercom_playing.lock().unwrap() = Some(IntercomPlay {
            priority: Priority::Readback,
            audio: None,
            started: Instant::now(),
            text: "flaps ten".into(),
            cabin: true,
            done: None,
        });
        interrupt_intercom_for_radio(&inner);
        let q = inner.queue.lock().unwrap();
        assert_eq!(q.heap.len(), 2);
        let again = q
            .heap
            .iter()
            .find(|u| u.text == "flaps ten")
            .expect("re-said");
        assert!(again.resume.is_none() && again.cabin);
        drop(q);

        // Cut with only the tail left: nothing owed, and the caller is released.
        let latch = Arc::new(Latch::default());
        *inner.intercom_playing.lock().unwrap() = Some(IntercomPlay {
            priority: Priority::Advisory,
            audio: Some(Resume {
                samples: Arc::new(vec![0.1_f32; 22_050]),
                rate: 22_050,
            }),
            started: Instant::now() - std::time::Duration::from_secs(1),
            text: "done".into(),
            cabin: false,
            done: Some(latch.clone()),
        });
        interrupt_intercom_for_radio(&inner);
        assert_eq!(
            inner.queue.lock().unwrap().heap.len(),
            2,
            "nothing more queued"
        );
        assert!(
            *latch.spoken.lock().unwrap(),
            "released: the sentence was finished"
        );
    }

    #[test]
    fn a_press_keeps_what_the_copilot_was_saying() {
        // The pilot keys over a caution: the press silences the cockpit, but
        // the line playing and the copilot lines queued behind it are OWED,
        // oldest first. Cabin and station lines are not the copilot's.
        let _guard = crate::mode::TEST_LOCK.lock().unwrap();
        crate::mode::set(crate::mode::Mode::Both);
        let voice = Voice::new(Config::default());
        voice.set_hold(true);
        voice.alert(Priority::Caution, "Gear's still up.");
        voice.alert(Priority::Advisory, "Next, KOPOR.");
        voice.alert_cabin(Priority::Advisory, "Cabin is secure.");
        voice.alert_station(
            Priority::Advisory,
            StationSpeaker::Tower,
            "Cleared to land.",
        );
        *voice.inner.intercom_playing.lock().unwrap() = Some(IntercomPlay {
            priority: Priority::Caution,
            audio: None,
            started: Instant::now(),
            text: "We're a little high, about 600 feet.".into(),
            cabin: false,
            done: None,
        });
        voice.shutup();
        let owed: Vec<String> = voice.unsaid().into_iter().map(|(t, _)| t).collect();
        assert_eq!(
            owed,
            [
                "We're a little high, about 600 feet.",
                "Gear's still up.",
                "Next, KOPOR."
            ]
        );
        voice.forget_unsaid();
        assert!(voice.unsaid().is_empty(), "said again: owed no longer");
        *voice.inner.intercom_playing.lock().unwrap() = None;
        voice.set_hold(false);
    }

    #[test]
    fn say_again_on_the_air_repeats_only_this_controllers_last_word() {
        let t0 = Instant::now();
        let line = ControllerLine {
            text: "November one seven two Sierra Papa, cleared for takeoff.".into(),
            speaker: StationSpeaker::Tower,
            khz: Some(118_500),
            at: t0,
        };
        let later = t0 + std::time::Duration::from_secs(8);
        // Same frequency, eight seconds on: the tower says it again.
        assert!(controller_repeat(Some(&line), Some(118_500), None, later).is_some());
        // 8.33 naming: 5 kHz off the carrier is the same channel.
        assert!(controller_repeat(Some(&line), Some(118_505), None, later).is_some());
        // Retuned to departure: that controller never said it.
        assert!(controller_repeat(Some(&line), Some(120_900), None, later).is_none());
        // A minute on: the picture may have moved without a word.
        assert!(
            controller_repeat(Some(&line), Some(118_500), None, t0 + RADIO_REPEAT_TTL).is_none()
        );
        // COM1 unknown: the tuned position must match, failing open.
        let blind = ControllerLine {
            khz: None,
            ..line.clone()
        };
        assert!(controller_repeat(Some(&blind), None, None, later).is_some());
        assert!(
            controller_repeat(Some(&blind), None, Some(StationSpeaker::Tower), later).is_some()
        );
        assert!(
            controller_repeat(Some(&blind), None, Some(StationSpeaker::Approach), later).is_none()
        );
        // Nothing said: nothing to repeat.
        assert!(controller_repeat(None, Some(118_500), None, later).is_none());
    }

    #[test]
    fn a_press_over_the_tower_keeps_its_line_but_never_anothers() {
        // The pilot keys up while the tower's clearance is queued: blocked,
        // but SAID, so the frequency's "say again" repeats it. ATIS, a ghost
        // pilot and the tower's half of another aircraft's exchange are
        // never the controller's word to us.
        let _guard = crate::mode::TEST_LOCK.lock().unwrap();
        crate::mode::set(crate::mode::Mode::Both);
        let voice = Voice::new(Config::default());
        voice.set_com1(Some(118_500), Some(StationSpeaker::Tower));
        voice.set_hold(true);
        voice.alert_station(
            Priority::Advisory,
            StationSpeaker::Atis,
            "Information Bravo.",
        );
        voice.alert_station(
            Priority::Advisory,
            StationSpeaker::Tower,
            "November one seven two Sierra Papa, cleared to land runway two seven.",
        );
        voice.alert_station(
            Priority::Advisory,
            StationSpeaker::Ghost { seed: 3 },
            "Speedbird 306, wilco.",
        );
        voice.alert_station_overheard(
            Priority::Advisory,
            StationSpeaker::Tower,
            "Speedbird 306, line up and wait runway two seven.",
        );
        voice.shutup();
        let last = voice.repeat_on_frequency().expect("the tower's line to us");
        assert_eq!(
            last.text,
            "November one seven two Sierra Papa, cleared to land runway two seven."
        );
        assert_eq!(last.khz, Some(118_500));
        // The airframe changed: its radio history goes with it.
        voice.forget_speech();
        assert!(voice.last_controller_line().is_none());
        voice.set_hold(false);
    }

    #[test]
    fn a_stepped_on_line_is_owed_until_its_remainder_plays() {
        let inner = bare_inner();
        *inner.intercom_playing.lock().unwrap() = Some(IntercomPlay {
            priority: Priority::Advisory,
            audio: None,
            started: Instant::now(),
            text: "Fuel's balanced.".into(),
            cabin: false,
            done: None,
        });
        interrupt_intercom_for_radio(&inner);
        let now = Instant::now();
        let owed = |inner: &Inner| inner.memory.lock().unwrap().unsaid(now).len();
        assert_eq!(owed(&inner), 1, "the tower cut it: owed");
        // The remainder plays once the frequency is quiet: said, not owed.
        inner
            .memory
            .lock()
            .unwrap()
            .started("Fuel's balanced.", true, now);
        assert_eq!(owed(&inner), 0);
        // A cabin line stepped on is the cabin's to repeat, not the copilot's.
        *inner.intercom_playing.lock().unwrap() = Some(IntercomPlay {
            priority: Priority::Advisory,
            audio: None,
            started: Instant::now(),
            text: "Cabin is secure.".into(),
            cabin: true,
            done: None,
        });
        interrupt_intercom_for_radio(&inner);
        assert_eq!(owed(&inner), 0);
    }

    #[test]
    fn speech_memory_keeps_volunteered_lines_and_expires_owed_ones() {
        let t0 = Instant::now();
        let mut m = SpeechMemory::default();
        // A reply to the pilot (blocking caller) is the main loop's readback,
        // not a volunteered line.
        m.started("Flaps ten.", false, t0);
        assert!(m.last.is_none());
        m.started("Top of descent in two minutes.", true, t0);
        assert_eq!(m.last.as_ref().unwrap().0, "Top of descent in two minutes.");
        // Owed lines are unique and perish.
        m.cut("Gear's still up.", t0);
        m.cut("Gear's still up.", t0);
        assert_eq!(m.unsaid(t0).len(), 1);
        assert!(m.unsaid(t0 + UNSAID_TTL).is_empty(), "the moment has gone");
        m.cut("   ", t0);
        assert_eq!(m.unsaid(t0).len(), 1, "nothing said is nothing owed");
    }

    #[test]
    fn crew_discipline_defers_routine_lines_but_not_warnings() {
        // While a station is on the air the copilot holds routine speech
        // (advisories, readbacks, cautions) — a crew member doesn't talk over
        // ATC. A Warning (stall, terrain) breaks through immediately.
        for p in [Priority::Advisory, Priority::Readback, Priority::Caution] {
            assert!(
                !intercom_may_start(p, true),
                "{p:?} must wait for the frequency"
            );
            assert!(
                intercom_may_start(p, false),
                "{p:?} speaks on a quiet frequency"
            );
        }
        assert!(
            intercom_may_start(Priority::Warning, true),
            "a Warning talks over the radio"
        );
        assert!(intercom_may_start(Priority::Warning, false));
    }

    #[test]
    fn radio_going_quiet_wakes_a_deferred_intercom_line() {
        // The intercom defers a routine line while the radio speaks; when the
        // radio's speaking flag clears, the intercom must be woken (via its
        // condvar) or the deferred line sleeps forever. Exercised end to end:
        // a blocking say() enqueued while the radio flag is up must complete
        // once the radio goes quiet.
        let voice = Voice::new(Config::default());
        let gate = voice.mic_gate();
        // Let the radio thread's startup pass park in its condvar wait first —
        // its initial "not speaking" clear would otherwise race this set. (In
        // production only the radio loop itself touches the flag.)
        std::thread::sleep(std::time::Duration::from_millis(100));
        set_channel_speaking(&voice.inner, &gate, true, true);
        let v2 = voice.clone();
        let t = std::thread::spawn(move || {
            v2.say(Priority::Readback, ""); // deferred while the radio is up
        });
        std::thread::sleep(std::time::Duration::from_millis(100));
        assert!(!t.is_finished(), "routine line must wait for the radio");
        set_channel_speaking(&voice.inner, &gate, true, false);
        t.join().unwrap();
    }

    #[test]
    fn higher_priority_outranks_fifo() {
        // A Warning jumps ahead of an earlier-queued Readback.
        let mut h = BinaryHeap::new();
        h.push(utt(Priority::Readback, 3));
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
