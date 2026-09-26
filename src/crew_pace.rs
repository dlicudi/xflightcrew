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
//! the next. What is SAID follows `XCOPILOT_CREW_CHATTER`: chatty calls every
//! item as it is done ("Beacon, on."), normal and quiet keep the milestone
//! lines (the flows rule, 2026-09-20). Every item goes to the blackbox
//! (`start_item`) and the panel status whatever is spoken.

use std::time::Duration;

use crate::persona::Chatter;
use crate::voice::{Priority, Voice};

/// The shortest and longest beat between two items. A switch is reached,
/// moved and looked at in about a second and a half.
const BEAT_MIN_MS: u64 = 1_100;
const BEAT_MAX_MS: u64 = 1_900;

/// The look outside after "Clear prop!", on top of its beat.
pub const CLEAR_PROP_MS: u64 = 2_000;

/// Whether each item is spoken as it is done.
pub fn items_spoken(chatter: Chatter) -> bool {
    chatter == Chatter::Chatty
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

/// One item done: blackboxed, spoken when the chatter asks for items, then a
/// beat before whatever comes next.
pub async fn item(voice: Option<&Voice>, line: &str) {
    item_as(voice, crate::persona::chatter(), line).await;
}

/// [`item`] with the chatter given, for tests.
pub async fn item_as(voice: Option<&Voice>, chatter: Chatter, line: &str) {
    let spoken = items_spoken(chatter);
    crate::blackbox::record(
        "start_item",
        serde_json::json!({ "item": line, "spoken": spoken }),
    );
    tracing::info!(target: "xvoicelayer::crew_pace", item = %line, spoken, "start item");
    if spoken {
        if let Some(v) = voice {
            v.alert(Priority::Advisory, line.to_string());
        }
    }
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
    }

    #[test]
    fn only_chatty_calls_every_item() {
        assert!(items_spoken(Chatter::Chatty));
        assert!(!items_spoken(Chatter::Normal));
        assert!(!items_spoken(Chatter::Quiet));
    }

    #[tokio::test(start_paused = true)]
    async fn an_item_takes_a_beat() {
        let t0 = tokio::time::Instant::now();
        item_as(None, Chatter::Normal, "Beacon, on.").await;
        let dt = t0.elapsed().as_millis() as u64;
        assert!(dt >= BEAT_MIN_MS, "{dt}");
    }
}
