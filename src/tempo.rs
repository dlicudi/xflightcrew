//! The crew's timing: when a reply starts, what fills a silence, how fast a
//! line is said, and when the flight deck is too busy for anything that can
//! wait.
//!
//! The words were never the machine tell once `readback::vary_ack` shipped;
//! the timing was. A person answers a beat after you stop talking, not in the
//! same frame; says "stand by" when the answer will take a while instead of
//! going silent for three seconds and then reciting a sentence; talks quicker
//! and flatter when something is wrong; and on the takeoff roll or short final
//! keeps the chat for later. Each of those is one small rule here, with its
//! state process-wide because the voice thread, the command loop and the
//! monitor tick all ask the same questions.
//!
//! - [`Turn`]: one pilot utterance on the intercom, from the end of the
//!   capture to the loop moving on. It times the first reply
//!   ([`reaction_pause`]) and arms the standby filler ([`STANDBY_AFTER`]).
//! - [`set_workload`] / [`workload_high`]: published by the monitor tick from
//!   [`busy_now`], read by the voice's pace, the reaction pause and the
//!   rapport's asides.
//! - [`line_tempo`]: the VITS pace and rhythm multipliers for one line.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

use crate::voice::Priority;

/// How long a turn may go without a single line queued before the copilot
/// says it is working on it. Under this, silence reads as thought; over it,
/// as a copilot who did not hear.
pub const STANDBY_AFTER: Duration = Duration::from_millis(1600);

/// The shortest gap between the end of the pilot's words and the first
/// reply, jittered inside this band per turn: nobody answers in the same
/// frame, and nobody answers at the same instant twice. Measured from the
/// end of the CAPTURE, so the speech-to-text time already counts toward it
/// and a slow transcription adds nothing.
pub const REACTION_MIN: Duration = Duration::from_millis(280);
pub const REACTION_MAX: Duration = Duration::from_millis(480);

/// When the last spoken capture ended (the PTT release or the VAD's end of
/// speech), before transcription. Set by the listener, read at turn start.
static CAPTURE_END: Mutex<Option<Instant>> = Mutex::new(None);

/// The turn in progress: its generation (so a filler thread from an old turn
/// can tell it is stale), when the pilot stopped talking, whether the first
/// reply has gone, and how many lines the voice had queued when it began.
struct TurnState {
    gen: u64,
    heard_at: Option<Instant>,
    replied: bool,
    queued_at_start: u64,
    /// The words went to the radio: the station answers, and the copilot's
    /// "stand by" over a tower call would be the crew talking on the
    /// frequency.
    radio: bool,
}

static TURN: Mutex<Option<TurnState>> = Mutex::new(None);
static TURN_GEN: AtomicU64 = AtomicU64::new(0);

/// The flight deck is busy (the roll, low and slow, a landing or takeoff
/// loop flying, a checklist in progress): terse replies, no reaction pause,
/// brisker pace, and every aside held.
static WORKLOAD_HIGH: AtomicBool = AtomicBool::new(false);

fn lock<T>(m: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

/// The listener finished a capture: the pilot has stopped talking.
pub fn note_capture_end() {
    *lock(&CAPTURE_END) = Some(Instant::now());
}

/// One intercom turn. Created when an utterance reaches the copilot's path;
/// dropped when the loop moves on (every `continue` drops it), which retires
/// the filler.
pub struct Turn {
    gen: u64,
}

impl Turn {
    /// Begin a turn. `spoken` = the words came from the microphone (a typed
    /// line or a panel button is deliberate, instant, and gets no pause).
    /// `say_standby` is what the filler thread does if the turn stays silent
    /// past [`STANDBY_AFTER`]; it runs on its own thread, because the loop
    /// that owns the turn is the thing being slow.
    pub fn begin(spoken: bool, say_standby: impl FnOnce() + Send + 'static) -> Self {
        let gen = TURN_GEN.fetch_add(1, Ordering::SeqCst) + 1;
        let heard_at = spoken.then(|| *lock(&CAPTURE_END)).flatten();
        *lock(&TURN) = Some(TurnState {
            gen,
            heard_at,
            replied: false,
            queued_at_start: crate::voice::lines_queued(),
            radio: false,
        });
        std::thread::Builder::new()
            .name("xvl-standby".into())
            .spawn(move || {
                std::thread::sleep(STANDBY_AFTER);
                if standby_due(gen, crate::voice::lines_queued()) {
                    say_standby();
                }
            })
            .ok();
        Turn { gen }
    }
}

impl Drop for Turn {
    fn drop(&mut self) {
        let mut t = lock(&TURN);
        if t.as_ref().is_some_and(|s| s.gen == self.gen) {
            *t = None;
        }
    }
}

/// Whether the filler for turn `gen` should speak: the turn is still the
/// current one, nothing has been queued on either channel since it began,
/// and it is not a radio call. Consumes the chance (marks the turn replied)
/// so it speaks at most once.
fn standby_due(gen: u64, queued_now: u64) -> bool {
    let mut t = lock(&TURN);
    let Some(s) = t.as_mut().filter(|s| s.gen == gen) else {
        return false;
    };
    if s.radio || s.replied || queued_now != s.queued_at_start {
        return false;
    }
    s.replied = true;
    true
}

/// The words in this turn are being tried on the radio (content routing):
/// no copilot filler while the station decides. `false` hands the turn back
/// to the copilot when the station declined it.
pub fn set_radio_turn(radio: bool) {
    if let Some(s) = lock(&TURN).as_mut() {
        s.radio = radio;
    }
}

/// Hold the first reply of a turn until a human beat has passed since the
/// pilot stopped talking. Called by the command loop's reply door just before
/// it speaks; blocks (the reply blocks anyway). Only the FIRST reply waits,
/// never when busy, never for typed input.
pub fn reaction_pause() {
    let wait = {
        let mut t = lock(&TURN);
        let Some(s) = t.as_mut() else {
            return;
        };
        if s.replied {
            return;
        }
        s.replied = true;
        match s.heard_at {
            Some(at) if !workload_high() => {
                pause_left(at.elapsed(), jitter(REACTION_MIN, REACTION_MAX, s.gen))
            }
            _ => Duration::ZERO,
        }
    };
    if !wait.is_zero() {
        std::thread::sleep(wait);
    }
}

/// What is left of a `target` pause after `elapsed` has already gone. Pure.
pub fn pause_left(elapsed: Duration, target: Duration) -> Duration {
    target.saturating_sub(elapsed)
}

/// A deterministic point inside `[lo, hi]` for `seed`: varies turn to turn
/// without a random source, and a test can pin it.
pub fn jitter(lo: Duration, hi: Duration, seed: u64) -> Duration {
    let span = hi.saturating_sub(lo).as_millis() as u64;
    if span == 0 {
        return lo;
    }
    // Knuth's multiplicative hash spreads consecutive seeds across the band.
    let h = seed.wrapping_mul(2_654_435_761) % (span + 1);
    lo + Duration::from_millis(h)
}

/// What the monitor tick knows about how busy the flight deck is.
#[derive(Clone, Copy, Debug, Default)]
pub struct Load {
    pub on_ground: bool,
    pub groundspeed_kts: f64,
    pub agl_ft: f64,
    /// A loop that owns the aircraft at a critical moment is flying:
    /// autoland, the takeoff, a recovery.
    pub critical_loop: bool,
    /// A checklist flow or a lesson is talking: the copilot is already busy.
    pub crew_busy: bool,
}

/// Low and close to the ground is where a crew stops chatting: the roll in
/// either direction, the first and last thousand feet, anything a landing or
/// takeoff loop is flying, a checklist in progress. Pure.
pub fn busy_now(l: &Load) -> bool {
    let rolling = l.on_ground && l.groundspeed_kts > 30.0;
    let low = !l.on_ground && l.agl_ft < 1000.0;
    rolling || low || l.critical_loop || l.crew_busy
}

/// Published by the monitor tick every pass.
pub fn set_workload(high: bool) {
    WORKLOAD_HIGH.store(high, Ordering::Relaxed);
}

pub fn workload_high() -> bool {
    WORKLOAD_HIGH.load(Ordering::Relaxed)
}

/// `(length, noise_w)` multipliers on the copilot voice's own scales for one
/// line. Length below 1 is quicker; noise_w below 1 is a flatter rhythm. A
/// warning is said quick and level, the way people speak when it matters; a
/// caution a little of that; routine talk in a quiet cockpit a touch looser
/// than the model's default, and brisk again when the flight deck is busy.
/// Pure.
pub fn line_tempo(priority: Priority, busy: bool) -> (f32, f32) {
    match priority {
        Priority::Warning => (0.88, 0.7),
        Priority::Caution => (0.94, 0.85),
        _ if busy => (0.93, 0.85),
        Priority::Advisory => (1.04, 1.1),
        Priority::Readback => (1.0, 1.0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Tests that touch the process-wide turn hold this.
    static LOCK: Mutex<()> = Mutex::new(());

    #[test]
    fn the_pause_is_what_is_left_after_the_transcription() {
        let target = Duration::from_millis(400);
        assert_eq!(
            pause_left(Duration::from_millis(100), target),
            Duration::from_millis(300)
        );
        // A slow transcription has already paid for the beat.
        assert_eq!(
            pause_left(Duration::from_millis(900), target),
            Duration::ZERO
        );
    }

    #[test]
    fn the_jitter_stays_in_its_band_and_moves_turn_to_turn() {
        let seen: std::collections::BTreeSet<u128> = (0..50)
            .map(|s| {
                let d = jitter(REACTION_MIN, REACTION_MAX, s);
                assert!(d >= REACTION_MIN && d <= REACTION_MAX, "{d:?}");
                d.as_millis()
            })
            .collect();
        assert!(seen.len() > 10, "the beat must vary: {seen:?}");
    }

    #[test]
    fn busy_is_the_roll_the_low_air_and_the_critical_loops() {
        let base = Load {
            on_ground: true,
            groundspeed_kts: 10.0,
            agl_ft: 0.0,
            ..Load::default()
        };
        assert!(!busy_now(&base), "taxiing is not busy");
        assert!(busy_now(&Load {
            groundspeed_kts: 90.0,
            ..base
        }));
        let air = Load {
            on_ground: false,
            groundspeed_kts: 120.0,
            agl_ft: 600.0,
            ..base
        };
        assert!(busy_now(&air), "short final");
        assert!(!busy_now(&Load {
            agl_ft: 6000.0,
            ..air
        }));
        assert!(busy_now(&Load {
            agl_ft: 6000.0,
            critical_loop: true,
            ..air
        }));
        assert!(busy_now(&Load {
            crew_busy: true,
            ..base
        }));
    }

    #[test]
    fn a_warning_is_quicker_and_flatter_than_chat() {
        let (wl, wn) = line_tempo(Priority::Warning, false);
        let (al, an) = line_tempo(Priority::Advisory, false);
        assert!(wl < 1.0 && wn < 1.0);
        assert!(al > wl && an > wn);
        // Busy: routine lines go brisk too, a warning is unchanged.
        assert!(line_tempo(Priority::Advisory, true).0 < 1.0);
        assert_eq!(line_tempo(Priority::Warning, true), (wl, wn));
    }

    #[test]
    fn the_standby_speaks_once_only_into_a_silent_copilot_turn() {
        let _g = lock(&LOCK);
        let gen = 9_000_001;
        let set = |radio| {
            *lock(&TURN) = Some(TurnState {
                gen,
                heard_at: None,
                replied: false,
                queued_at_start: 5,
                radio,
            })
        };
        set(false);
        assert!(!standby_due(gen, 6), "something was said: no filler");
        set(false);
        assert!(
            !standby_due(gen + 1, 5),
            "a stale turn's thread says nothing"
        );
        set(true);
        assert!(
            !standby_due(gen, 5),
            "a radio call is the station's to answer"
        );
        set(false);
        assert!(standby_due(gen, 5));
        assert!(!standby_due(gen, 5), "once");
        *lock(&TURN) = None;
    }

    #[test]
    fn only_the_first_reply_of_a_turn_waits() {
        let _g = lock(&LOCK);
        *lock(&TURN) = Some(TurnState {
            gen: 9_000_002,
            // Long ago: the pause has nothing left to wait.
            heard_at: Some(Instant::now() - Duration::from_secs(5)),
            replied: false,
            queued_at_start: 0,
            radio: false,
        });
        let t0 = Instant::now();
        reaction_pause();
        assert!(lock(&TURN).as_ref().unwrap().replied);
        reaction_pause();
        assert!(t0.elapsed() < Duration::from_millis(100));
        *lock(&TURN) = None;
    }
}
