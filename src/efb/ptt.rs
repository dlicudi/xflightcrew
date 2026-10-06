//! The EFB's push to talk: a hold-to-talk button on the device that opens
//! the mic exactly as the plugin's `xcopilot/ptt_active` does (the INTERCOM
//! key: talking to the copilot). It moves nothing: the words still go
//! through STT and the grammar like any spoken line.
//!
//! A phone loses touch-ups (iOS cancels a touch for a notification, a
//! scroll, a locked screen), so a release is never trusted to arrive: the
//! page sends a press, then a heartbeat every 250 ms while the finger is
//! down, then a release. The hold counts as held only while its heartbeat is
//! fresh ([`FRESH`]) and never longer than [`MAX_HOLD`], so a lost release
//! can never leave the mic open.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

/// A hold whose last heartbeat is older than this is released.
pub(crate) const FRESH: Duration = Duration::from_millis(600);
/// No hold lasts longer than this, heartbeats or not.
pub(crate) const MAX_HOLD: Duration = Duration::from_secs(60);

/// One press of the button.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Hold {
    pub(crate) pressed_at: Instant,
    pub(crate) last_beat: Instant,
}

/// Why a hold ended without its release.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Lapse {
    /// The heartbeat stopped (a lost touch-up, the page closed).
    Stale,
    /// Held past [`MAX_HOLD`].
    Capped,
}

impl Lapse {
    fn word(self) -> &'static str {
        match self {
            Lapse::Stale => "heartbeat",
            Lapse::Capped => "cap",
        }
    }
}

impl Hold {
    pub(crate) fn new(now: Instant) -> Hold {
        Hold {
            pressed_at: now,
            last_beat: now,
        }
    }

    /// `None` while held at `now`, else why it lapsed. Pure.
    pub(crate) fn lapse(&self, now: Instant) -> Option<Lapse> {
        if now.saturating_duration_since(self.pressed_at) >= MAX_HOLD {
            Some(Lapse::Capped)
        } else if now.saturating_duration_since(self.last_beat) >= FRESH {
            Some(Lapse::Stale)
        } else {
            None
        }
    }
}

/// The hold in progress (session: one finger, with its own expiry).
static HOLD: Mutex<Option<Hold>> = Mutex::new(None);

/// This run captures on a push to talk at all (a bound PTT dataref or key):
/// in continuous listening or text-only there is nothing for the button to
/// open, and the page says so.
static AVAILABLE: AtomicBool = AtomicBool::new(false);

fn hold() -> std::sync::MutexGuard<'static, Option<Hold>> {
    HOLD.lock().unwrap_or_else(|e| e.into_inner())
}

/// Boot: whether this run's capture is push to talk.
pub(crate) fn set_available(on: bool) {
    AVAILABLE.store(on, Ordering::Relaxed);
}

pub(crate) fn available() -> bool {
    AVAILABLE.load(Ordering::Relaxed)
}

fn record(event: &str, held_ms: Option<u128>, why: Option<&str>) {
    crate::blackbox::record(
        "efb",
        serde_json::json!({ "event": "ptt", "action": event, "held_ms": held_ms, "why": why }),
    );
}

/// The finger went down. A press over a live hold keeps the hold's start.
pub(crate) fn press() -> bool {
    if !available() {
        return false;
    }
    let now = Instant::now();
    let mut h = hold();
    match h.as_mut() {
        Some(x) if x.lapse(now).is_none() => x.last_beat = now,
        _ => {
            *h = Some(Hold::new(now));
            drop(h);
            record("press", None, None);
        }
    }
    true
}

/// The finger is still down: `true` while the hold lives (a lapsed one is
/// not revived; the page shows it released and a new press starts again).
pub(crate) fn beat() -> bool {
    let now = Instant::now();
    let mut h = hold();
    match h.as_mut() {
        Some(x) if x.lapse(now).is_none() => {
            x.last_beat = now;
            true
        }
        _ => false,
    }
}

/// The finger came up.
pub(crate) fn release() {
    if let Some(x) = hold().take() {
        record(
            "release",
            Some(x.pressed_at.elapsed().as_millis()),
            None,
        );
    }
}

/// Is the EFB's button held now? Read by the capture and the barge-in
/// watch beside the plugin's key; a lapsed hold is dropped here, said in
/// the blackbox.
pub(crate) fn held() -> bool {
    let now = Instant::now();
    let mut h = hold();
    let Some(x) = *h else { return false };
    match x.lapse(now) {
        None => true,
        Some(why) => {
            *h = None;
            drop(h);
            record(
                "lapsed",
                Some(now.saturating_duration_since(x.pressed_at).as_millis()),
                Some(why.word()),
            );
            false
        }
    }
}

/// Held, without dropping a lapsed hold (the page's state).
pub(crate) fn showing_held() -> bool {
    hold().is_some_and(|x| x.lapse(Instant::now()).is_none())
}

#[cfg(test)]
pub(crate) static TEST_LOCK: Mutex<()> = Mutex::new(());

#[cfg(test)]
mod tests {
    use super::*;

    /// A lost touch-up cannot leave the mic open: the heartbeat's silence
    /// ends the hold, and so does the minute's cap however many beats come.
    #[test]
    fn a_hold_lives_on_its_heartbeat_and_never_past_the_cap() {
        let t0 = Instant::now();
        let mut h = Hold::new(t0);
        assert_eq!(h.lapse(t0 + Duration::from_millis(400)), None);
        assert_eq!(
            h.lapse(t0 + Duration::from_millis(650)),
            Some(Lapse::Stale),
            "no beat for 650 ms: released"
        );
        let mut t = t0;
        while t < t0 + Duration::from_secs(59) {
            t += Duration::from_millis(250);
            h.last_beat = t;
            assert_eq!(h.lapse(t), None);
        }
        let late = t0 + MAX_HOLD;
        h.last_beat = late;
        assert_eq!(h.lapse(late), Some(Lapse::Capped));
    }

    #[test]
    fn press_beat_release_and_nothing_when_not_push_to_talk() {
        let _g = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        set_available(false);
        assert!(!press());
        assert!(!held());
        set_available(true);
        assert!(press());
        assert!(held() && showing_held());
        assert!(beat());
        release();
        assert!(!held() && !beat(), "a beat after the release opens nothing");
        // A hold whose page went quiet lapses at the next look.
        assert!(press());
        hold().as_mut().unwrap().last_beat -= FRESH;
        assert!(!showing_held());
        assert!(!held());
        assert!(hold().is_none(), "dropped, not left to linger");
        set_available(false);
    }
}
