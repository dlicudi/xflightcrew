//! A start-up at a person's pace: one item, a beat, the next.
//!
//! Full auto brought a cold 172 from "Starting from cold" to "Electrical
//! set" in 2.1 seconds (LIPK, 2026-09-26): battery, beacon, nav, panel,
//! mixture, throttle, mags, crank, alternator and avionics back to back.
//! Nobody's hands move that fast, and the pilot heard a machine. The
//! airliner flow already had a crew's pace (`jet_start::run_steps`, gated
//! on each switch reading back); the piston start and full auto's own
//! ramp items did not.
//!
//! Each item is done in the checklist's order, then a jittered beat before
//! the next. What is SAID follows `XCOPILOT_START_CALLS` ([`StartCalls`]),
//! never the chatter: `milestones` keeps the callouts a crew makes (the
//! flows rule, 2026-09-20), `every item` calls each switch once its
//! readback confirmed it ("Beacon on."). A switch that did not take is
//! NAMED in both modes ("Panel lights did not come on."), and one whose
//! readback failed is no measurement: neither claimed nor named. Every item
//! goes to the blackbox (`start_item`) whatever is spoken.

use std::time::Duration;

use crate::voice::{Priority, Voice};

/// The shortest and longest beat between two items. A switch is reached,
/// moved, looked at and called in two to three seconds; at 1.1 to 1.9 the
/// called ramp still ran together (2026-10-06).
const BEAT_MIN_MS: u64 = 2_000;
const BEAT_MAX_MS: u64 = 3_000;

/// The look outside after "Clear prop!", on top of its beat.
pub const CLEAR_PROP_MS: u64 = 2_000;

/// How much of a start-up is called aloud (`XCOPILOT_START_CALLS`). Its own
/// setting, never the chatter's: a quiet copilot still makes the procedure
/// calls.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StartCalls {
    /// The callouts a crew makes: battery, beacon, the start, the result.
    Milestones,
    /// Each switch called as it is confirmed.
    EveryItem,
}

impl StartCalls {
    /// Lenient: the panel's "every item", an env var's `every_item` or
    /// `items`; anything else is the default.
    pub fn parse(s: &str) -> Self {
        let s = s.trim().to_ascii_lowercase().replace(['_', '-'], " ");
        match s.as_str() {
            "every item" | "every" | "items" | "item" | "all" | "everything" => Self::EveryItem,
            _ => Self::Milestones,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Milestones => "milestones",
            Self::EveryItem => "every item",
        }
    }

    /// Whether each item is spoken as it is confirmed.
    pub fn items_spoken(self) -> bool {
        self == Self::EveryItem
    }
}

/// The live setting.
pub fn start_calls() -> StartCalls {
    StartCalls::parse(&crate::settings::live().start_calls)
}

/// Whether each item is spoken now.
pub fn items_spoken() -> bool {
    start_calls().items_spoken()
}

/// A jittered beat in `[BEAT_MIN_MS, BEAT_MAX_MS]`.
pub fn beat_ms() -> u64 {
    let span = BEAT_MAX_MS - BEAT_MIN_MS;
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.subsec_nanos() as u64)
        .unwrap_or(0);
    BEAT_MIN_MS + nanos % (span + 1)
}

/// What one checked item says, by mode and readback. `None` is silence.
/// Pure. `taken`: `Some(true)` read back as wanted, `Some(false)` read back
/// otherwise, `None` no reading.
pub fn checked_line<'a>(
    calls: StartCalls,
    taken: Option<bool>,
    done: &'a str,
    not_taken: &'a str,
) -> Option<&'a str> {
    match taken {
        Some(true) if calls.items_spoken() => Some(done),
        Some(false) => Some(not_taken),
        _ => None,
    }
}

fn speak(voice: Option<&Voice>, line: &str) {
    if let Some(v) = voice {
        // `alert` mirrors to the panel itself.
        v.alert(Priority::Advisory, line.to_string());
    }
}

/// A call that claims no switch ("Clear prop!", "Starting number two."):
/// blackboxed, spoken when every item is called, then a beat.
pub async fn item(voice: Option<&Voice>, line: &str) {
    item_as(voice, start_calls(), line).await;
}

/// [`item`] with the setting given, for tests.
pub async fn item_as(voice: Option<&Voice>, calls: StartCalls, line: &str) {
    let spoken = calls.items_spoken();
    crate::blackbox::record(
        "start_item",
        serde_json::json!({ "item": line, "spoken": spoken }),
    );
    tracing::info!(target: "xvoicelayer::crew_pace", item = %line, spoken, "start item");
    if spoken {
        speak(voice, line);
    }
    tokio::time::sleep(Duration::from_millis(beat_ms())).await;
}

/// A switch item, called only AFTER its readback: `done` when it read back as
/// wanted (every-item mode), `not_taken` in both modes when it read back
/// otherwise, nothing when there was no reading. Then a beat.
pub async fn checked(voice: Option<&Voice>, taken: Option<bool>, done: &str, not_taken: &str) {
    checked_as(voice, start_calls(), taken, done, not_taken).await;
}

/// [`checked`] with the setting given, for tests.
pub async fn checked_as(
    voice: Option<&Voice>,
    calls: StartCalls,
    taken: Option<bool>,
    done: &str,
    not_taken: &str,
) {
    let line = checked_line(calls, taken, done, not_taken);
    crate::blackbox::record(
        "start_item",
        serde_json::json!({ "item": done, "taken": taken, "spoken": line }),
    );
    tracing::info!(target: "xvoicelayer::crew_pace", item = %done, ?taken, "start item");
    if let Some(l) = line {
        speak(voice, l);
    }
    tokio::time::sleep(Duration::from_millis(beat_ms())).await;
}

/// A milestone: said in both modes, then a beat.
pub async fn milestone(voice: Option<&Voice>, line: &str) {
    crate::blackbox::record(
        "start_item",
        serde_json::json!({ "item": line, "spoken": line, "milestone": true }),
    );
    speak(voice, line);
    tokio::time::sleep(Duration::from_millis(beat_ms())).await;
}

/// A pause of `ms` with nothing done (the look outside before the crank).
pub async fn pause(ms: u64) {
    tokio::time::sleep(Duration::from_millis(ms)).await;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_beat_is_a_person_reaching_for_a_switch() {
        for _ in 0..50 {
            let b = beat_ms();
            assert!((BEAT_MIN_MS..=BEAT_MAX_MS).contains(&b), "{b}");
        }
        assert!(BEAT_MIN_MS >= 2_000 && BEAT_MAX_MS <= 3_000);
    }

    #[test]
    fn only_every_item_calls_every_item() {
        assert!(StartCalls::EveryItem.items_spoken());
        assert!(!StartCalls::Milestones.items_spoken());
    }

    #[test]
    fn the_setting_reads_every_spelling() {
        for s in ["every item", "EVERY_ITEM", "every-item", "items"] {
            assert_eq!(StartCalls::parse(s), StartCalls::EveryItem, "{s}");
        }
        for s in ["milestones", "", "chatty", "banana"] {
            assert_eq!(StartCalls::parse(s), StartCalls::Milestones, "{s}");
        }
        for c in [StartCalls::Milestones, StartCalls::EveryItem] {
            assert_eq!(StartCalls::parse(c.as_str()), c);
        }
    }

    /// Claimed only once read back; a switch that did not take is named in
    /// both modes; a failed read claims and names nothing.
    #[test]
    fn an_item_is_called_only_after_its_readback() {
        let (d, n) = ("Beacon on.", "Beacon did not come on.");
        use StartCalls::*;
        assert_eq!(checked_line(EveryItem, Some(true), d, n), Some(d));
        assert_eq!(checked_line(Milestones, Some(true), d, n), None);
        assert_eq!(checked_line(EveryItem, Some(false), d, n), Some(n));
        assert_eq!(checked_line(Milestones, Some(false), d, n), Some(n));
        assert_eq!(checked_line(EveryItem, None, d, n), None);
        assert_eq!(checked_line(Milestones, None, d, n), None);
    }

    #[tokio::test(start_paused = true)]
    async fn an_item_takes_a_beat() {
        let t0 = tokio::time::Instant::now();
        item_as(None, StartCalls::Milestones, "Clear prop!").await;
        let dt = t0.elapsed().as_millis() as u64;
        assert!(dt >= BEAT_MIN_MS, "{dt}");
        let t0 = tokio::time::Instant::now();
        checked_as(None, StartCalls::EveryItem, Some(true), "Beacon on.", "x").await;
        assert!(t0.elapsed().as_millis() as u64 >= BEAT_MIN_MS);
    }
}
