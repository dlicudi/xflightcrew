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
use std::sync::Mutex;

/// A manager's live detail line, set by its own loop from the numbers it
/// already has each tick (no extra sim reads): the fix it is steering at,
/// the metres left, what a wait is waiting on. Folded into the manager's
/// `status()` line as "name: detail"; cleared when the loop ends. Rounded by
/// the writer so the change-gated publish is not re-sent for jitter.
pub struct Detail(Mutex<String>);

impl Detail {
    pub const fn new() -> Self {
        Detail(Mutex::new(String::new()))
    }
    pub fn set(&self, text: impl Into<String>) {
        *self.0.lock().unwrap_or_else(|e| e.into_inner()) = text.into();
    }
    pub fn clear(&self) {
        self.set("");
    }
    pub fn get(&self) -> String {
        self.0.lock().unwrap_or_else(|e| e.into_inner()).clone()
    }
    /// `name` alone, or "name: detail" while a detail is published.
    pub fn line(&self, name: &str) -> String {
        let d = self.get();
        if d.is_empty() {
            name.to_string()
        } else {
            format!("{name}: {d}")
        }
    }
}

/// Clears a [`Detail`] (and the AP targets) when dropped: a loop holds one
/// for its whole run so every exit path, early returns included, leaves
/// nothing stale on the panel.
pub struct DetailGuard(&'static Detail);

impl DetailGuard {
    pub fn hold(d: &'static Detail) -> Self {
        DetailGuard(d)
    }
}

impl Drop for DetailGuard {
    fn drop(&mut self) {
        self.0.clear();
        clear_ap_targets();
    }
}

/// What the flying loop is telling the autopilot right now, the line a pilot
/// glances at to see what the AP has been asked to do. Panel-only (never
/// spoken). Written by whichever loop drives the AP each tick.
static AP_TARGETS: Detail = Detail::new();

/// Publish the AP targets: heading bug (magnetic), altitude (ft) and VS
/// command (fpm). `None` leaves a field out. Rounded to what a pilot dials.
pub fn set_ap_targets(hdg_mag: Option<f64>, alt_ft: Option<f64>, vs_fpm: Option<f64>) {
    AP_TARGETS.set(ap_targets_line(hdg_mag, alt_ft, vs_fpm));
}

pub fn clear_ap_targets() {
    AP_TARGETS.clear();
}

/// "ap: hdg 084, alt 5,500, vs +500" from the three dials; empty if none.
pub fn ap_targets_line(hdg_mag: Option<f64>, alt_ft: Option<f64>, vs_fpm: Option<f64>) -> String {
    let mut parts = Vec::new();
    if let Some(h) = hdg_mag {
        parts.push(format!("hdg {:03}", (h.rem_euclid(360.0).round() as i64) % 360));
    }
    if let Some(a) = alt_ft {
        parts.push(format!("alt {}", commas((a / 10.0).round() as i64 * 10)));
    }
    if let Some(v) = vs_fpm {
        let v = (v / 50.0).round() as i64 * 50;
        parts.push(format!("vs {v:+}"));
    }
    if parts.is_empty() {
        String::new()
    } else {
        format!("ap: {}", parts.join(", "))
    }
}

/// 5500 -> "5,500".
pub fn commas(n: i64) -> String {
    let digits = n.abs().to_string();
    let mut out = String::new();
    for (i, c) in digits.chars().enumerate() {
        if i > 0 && (digits.len() - i) % 3 == 0 {
            out.push(',');
        }
        out.push(c);
    }
    if n < 0 {
        format!("-{out}")
    } else {
        out
    }
}

/// The crew's last spoken line, so the Flight tab says why the copilot just
/// spoke without a trip to the Transcript tab. Panel-only.
static LAST_SAID: Detail = Detail::new();

pub fn note_said(text: &str) {
    LAST_SAID.set(text.trim());
}


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
    if let Some(s) = full_auto::wait_status() {
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

/// The panel-only lines under the managers: the AP targets and the last
/// spoken line. Never in [`active_managers`] (they are not tasks, and a
/// spoken status must not read a heading bug aloud).
pub fn panel_extras() -> Vec<String> {
    let mut v = Vec::new();
    let ap = AP_TARGETS.get();
    if !ap.is_empty() {
        v.push(ap);
    }
    let said = LAST_SAID.get();
    if !said.is_empty() {
        v.push(format!("said: {said}"));
    }
    v
}

/// The `xcopilot/status/auto` payload. Byte 0 is a marker ('1' = a manager is
/// actively flying/driving — the AUTO annunciator, NOT CANCEL's gate, which
/// is always available on the plugin side), then one line
/// per active manager, newline-separated, highest priority first. The primary
/// line is structured `NAME|stage` for the panel's headline row ("full auto:
/// request taxi" becomes "FULL AUTO|request taxi"); the rest stay plain.
pub fn compose() -> String {
    let mut lines = active_managers();
    let mut out = String::new();
    // The marker is "a manager is actually flying/driving" — the AUTO
    // annunciator and the panel headline. It stopped being derivable from
    // `cancel_phrase()` the day that gained its "my controls" fallback and
    // became always-Some: the amber AUTO chip lit permanently, parked
    // cold-and-dark included (audit, 2026-08-02). CANCEL's availability is
    // the plugin's own business now — the button always aborts SOMETHING,
    // which is exactly why it can no longer double as this signal.
    out.push(if lines.is_empty() { '0' } else { '1' });
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
    // capping the extras keeps the primary line whole no matter what; the
    // panel-only lines (AP targets, last said) come last so a long manager
    // list costs them, never a task.
    for l in lines.iter().take(6).chain(panel_extras().iter()) {
        out.push('\n');
        out.push_str(l);
    }
    out
}

/// The phrase the panel's CANCEL button injects, picked from whatever is
/// actually running — cancelling is per-automation in the grammar ("cancel
/// full auto", "abort takeoff"), so the button must choose like a pilot would.
/// Full auto outranks its own sub-loops because its cancel cascades into all
/// of them.
///
/// NEVER None: the fallback is "my controls", the universal hand-back
/// (autopilot off, every hold released). The old None was returned exactly
/// when the pilot most needed the button: after the missed-approach give-up
/// every ACTIVE flag was already false — full auto had given up, autoland had
/// broken out — while the AP still flew the aircraft with landing flap out
/// and decaying airspeed, and CANCEL mapped to nothing and was silently
/// consumed (EGHI, 2026-08-02). An abort button that goes dead precisely when
/// things have gone wrong is worse than none; "stop everything, give it to
/// me" must always be available, and covers the bare speed hold too.
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
    Some("my controls")
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

#[cfg(test)]
mod detail_tests {
    use super::*;

    #[test]
    fn ap_targets_line_rounds_to_what_a_pilot_dials() {
        assert_eq!(
            ap_targets_line(Some(83.6), Some(5497.0), Some(512.0)),
            "ap: hdg 084, alt 5,500, vs +500"
        );
        assert_eq!(ap_targets_line(Some(359.7), None, Some(-30.0)), "ap: hdg 000, vs -50");
        assert_eq!(ap_targets_line(None, None, None), "");
    }

    #[test]
    fn detail_folds_into_the_name_and_a_guard_clears_it() {
        static D: Detail = Detail::new();
        assert_eq!(D.line("auto taxi"), "auto taxi");
        {
            let _g = DetailGuard::hold(&D);
            D.set("340 m to go");
            set_ap_targets(Some(90.0), None, None);
            assert_eq!(D.line("auto taxi"), "auto taxi: 340 m to go");
            assert!(panel_extras().iter().any(|l| l.starts_with("ap: hdg 090")));
        }
        assert_eq!(D.line("auto taxi"), "auto taxi");
        assert!(!panel_extras().iter().any(|l| l.starts_with("ap:")));
    }
}
