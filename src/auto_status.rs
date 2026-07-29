//! The crew-facing automation surface, one page: what the copilot is flying
//! right now, and the request codes the panel's buttons speak through.
//!
//! `active_managers` is the priority-ordered "managing" list (the TUI dashboard
//! and the plugin's Flight tab render the same truth); `compose` packs it into
//! the `xcopilot/status/auto` payload; `cancel_phrase` picks the context cancel
//! the CANCEL button injects; and [`REQUEST_PHRASES`] maps button codes to the
//! English phrases they inject — the same words a pilot would speak, parsed by
//! the same deterministic grammar, so a button can never do anything the voice
//! can't (and never touches the LLM).

use std::sync::atomic::Ordering::Relaxed;

use crate::{approach_guide, autoland, autotakeoff, autotaxi, circuit, full_auto, procedure};
use crate::{pushback, tools};

/// What the copilot is actively managing right now — the continuous control
/// loops that keep working between commands — one short line each. Ordered by
/// priority: an emergency (autoland / recovery, both publish through
/// [`autoland::PANEL`]) outranks the routine governors. Cheap atomic/mutex
/// reads, safe to poll every tick.
pub fn active_managers() -> Vec<String> {
    let mut v = Vec::new();
    if let Ok(p) = autoland::PANEL.lock() {
        if !p.is_empty() {
            v.push(p.clone());
        }
    }
    // Recovery without a panel line yet (the watcher writes it a beat later).
    if v.is_empty() && tools::recover::ACTIVE.load(Relaxed) {
        v.push("recovery".to_string());
    }
    if let Some(a) = approach_guide::armed() {
        v.push(format!("talk-down {} {}", a.icao, a.runway_id));
    }
    if let Some(s) = full_auto::status() {
        v.push(s);
    }
    if let Some(s) = circuit::status() {
        v.push(s);
    }
    if let Some(s) = procedure::status() {
        v.push(s);
    }
    if let Some(s) = autotakeoff::status() {
        v.push(s);
    }
    if let Some(s) = pushback::status() {
        v.push(s);
    }
    if let Some(s) = autotaxi::status() {
        v.push(s);
    }
    if let Some(s) = tools::speedhold::status() {
        v.push(s);
    }
    if tools::controls::brakes_held() {
        v.push("holding brakes".to_string());
    }
    if let Some(s) = tools::reverse::status() {
        v.push(s);
    }
    if let Some(s) = tools::engine::mixture_status() {
        v.push(s.to_string());
    }
    v
}

/// The `xcopilot/status/auto` payload. Byte 0 is a marker ('1' = something is
/// running that the CANCEL button can stand down, '0' = idle), then one line
/// per active manager, newline-separated, highest priority first. The primary
/// line is structured `NAME|stage` for the panel's headline row ("full auto:
/// request taxi" becomes "FULL AUTO|request taxi"); the rest stay plain.
pub fn compose() -> String {
    let mut lines = active_managers();
    let mut out = String::new();
    out.push(if cancel_phrase().is_some() { '1' } else { '0' });
    if lines.is_empty() {
        return out;
    }
    let primary = lines.remove(0);
    let (name, stage) = match primary.split_once(": ") {
        Some((n, s)) => (n.to_uppercase(), s.to_string()),
        None => (primary.to_uppercase(), String::new()),
    };
    out.push_str(&name);
    out.push('|');
    out.push_str(&stage);
    // The clip to the plugin's 512-byte buffer truncates from the tail, so
    // capping the extras keeps the primary line whole no matter what.
    for l in lines.iter().take(5) {
        out.push('\n');
        out.push_str(l);
    }
    out
}

/// The phrase the panel's CANCEL button injects, picked from whatever is
/// actually running — cancelling is per-automation in the grammar ("cancel
/// full auto", "abort takeoff"), so the button must choose like a pilot would.
/// Full auto outranks its own sub-loops because its cancel cascades into all
/// of them. A bare speed hold is not covered here: "MY CONTROLS" releases it.
pub fn cancel_phrase() -> Option<&'static str> {
    if full_auto::ACTIVE.load(Relaxed) {
        return Some("cancel full auto");
    }
    if autoland::ACTIVE.load(Relaxed) {
        return Some("cancel autoland");
    }
    if autotakeoff::ACTIVE.load(Relaxed) {
        return Some("abort takeoff");
    }
    if circuit::ACTIVE.load(Relaxed) {
        return Some("cancel the circuit");
    }
    if procedure::ACTIVE.load(Relaxed) {
        return Some("cancel the approach");
    }
    if pushback::ACTIVE.load(Relaxed) {
        return Some("stop pushback");
    }
    if autotaxi::driving() {
        return Some("cancel auto taxi");
    }
    None
}

/// Panel request codes → the English command each button speaks. The single
/// source of truth for the panel-request poller AND the grammar determinism
/// test — every phrase here must parse via `grammar::try_parse`, never the
/// LLM. Code 11 (CANCEL) is absent by design: its phrase is context-chosen by
/// [`cancel_phrase`].
pub const REQUEST_PHRASES: &[(i64, &str)] = &[
    (1, "take control"),
    (2, "land the aircraft"),
    (3, "my controls"),
    (4, "full auto"),
    (5, "fly my flight plan"),
    (6, "push back"),
    (7, "taxi for me"),
    (8, "line us up"),
    (9, "take off for me"),
    (10, "do a circuit"),
];

/// The CANCEL button's request code, handled outside [`REQUEST_PHRASES`].
pub const REQUEST_CANCEL: i64 = 11;
