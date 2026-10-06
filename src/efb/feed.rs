//! What the EFB reads that nothing else kept: the app's own words as they
//! were said (the panel transcript is write-only, a feed to the plugin), the
//! monitor's last tick as it already read it, and the plan audit and
//! preflight verdicts, which were spoken once and dropped.
//!
//! Every writer here is a TAP on a path that already runs: the speaker
//! threads, the command loop's input, the monitor tick. Nothing here asks
//! X-Plane anything (CLAUDE.md "Web API load is a sim cost"): the EFB is a
//! reader of facts the app already holds.

use std::collections::VecDeque;
use std::sync::Mutex;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crate::scoped::Scoped;

/// Who said a line.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "lowercase")]
pub(crate) enum Who {
    /// The pilot: a spoken line as heard, a typed line, a panel button, the EFB.
    Pilot,
    /// The copilot, on the intercom or (with `radio`) on the frequency.
    Crew,
    Cabin,
    /// A controller (tower, approach, departure).
    Atc,
    /// The ATIS loop.
    Atis,
    /// Another aircraft on the party line.
    Traffic,
}

/// One line of the conversation.
#[derive(Clone, Debug, PartialEq, serde::Serialize)]
pub(crate) struct Line {
    /// Seconds since the epoch (wall clock, UTC).
    pub(crate) t: f64,
    pub(crate) who: Who,
    pub(crate) text: String,
    /// Said on the frequency (a controller, traffic, the crew's or the
    /// pilot's transmission) rather than the intercom.
    pub(crate) radio: bool,
    /// The station position's tag ("TWR", "APP"), for a station line.
    pub(crate) tag: Option<&'static str>,
}

/// Lines kept: the Now page shows eight, the Radio page's log the rest.
const LINES_MAX: usize = 80;

/// The conversation, oldest first (session: a log of what was said).
static LINES: Mutex<VecDeque<Line>> = Mutex::new(VecDeque::new());

/// Crew transmissions handed to the intercom speaker but not yet started:
/// the crew's radio calls are spoken by the copilot's own voice, so the
/// speaker thread cannot tell them from intercom lines by itself.
static RADIO_PENDING: Mutex<Vec<(Instant, String)>> = Mutex::new(Vec::new());

fn now_unix() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

fn push(line: Line) {
    if line.text.trim().is_empty() {
        return;
    }
    let mut l = LINES.lock().unwrap_or_else(|e| e.into_inner());
    l.push_back(line);
    while l.len() > LINES_MAX {
        l.pop_front();
    }
}

/// The pilot's line, as the command loop took it (`radio`: keyed on the
/// radio PTT).
pub(crate) fn note_pilot(text: &str, radio: bool) {
    push(Line {
        t: now_unix(),
        who: Who::Pilot,
        text: crate::phrase::scrub_dashes(text.trim()),
        radio,
        tag: None,
    });
}

/// The crew is about to key the radio with these words (the crew radio
/// drain, before `say`): the intercom speaker marks the line as radio when
/// it starts.
pub(crate) fn crew_transmits(text: &str) {
    let mut p = RADIO_PENDING.lock().unwrap_or_else(|e| e.into_inner());
    p.retain(|(at, _)| at.elapsed() < Duration::from_secs(120));
    p.push((
        Instant::now(),
        crate::phrase::scrub_dashes(text.trim()).trim().to_string(),
    ));
}

/// An intercom line started playing (the copilot or the cabin).
pub(crate) fn note_intercom(text: &str, cabin: bool) {
    let text = text.trim();
    let radio = !cabin && {
        let mut p = RADIO_PENDING.lock().unwrap_or_else(|e| e.into_inner());
        match p.iter().position(|(_, t)| t == text) {
            Some(i) => {
                p.remove(i);
                true
            }
            None => false,
        }
    };
    push(Line {
        t: now_unix(),
        who: if cabin { Who::Cabin } else { Who::Crew },
        text: text.to_string(),
        radio,
        tag: None,
    });
}

/// A station line started playing on the radio.
pub(crate) fn note_station(text: &str, speaker: crate::voice::StationSpeaker) {
    use crate::voice::StationSpeaker as S;
    let who = match speaker {
        S::Tower | S::Approach | S::Departure => Who::Atc,
        S::Atis => Who::Atis,
        S::Ghost { .. } => Who::Traffic,
    };
    push(Line {
        t: now_unix(),
        who,
        text: text.trim().to_string(),
        radio: true,
        tag: Some(speaker.tag()),
    });
}

/// The conversation, oldest first.
pub(crate) fn lines() -> Vec<Line> {
    LINES
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .iter()
        .cloned()
        .collect()
}

/// What the monitor's tick had already read, handed over as it was: the
/// EFB's only view of the aircraft.
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct Tick {
    pub(crate) lat: f64,
    pub(crate) lon: f64,
    pub(crate) alt_ft: f64,
    pub(crate) agl_ft: f64,
    pub(crate) ias_kts: f64,
    pub(crate) gs_kts: f64,
    pub(crate) on_ground: bool,
    pub(crate) track_deg: Option<f64>,
    /// True heading (`psi`) as the monitor read it, best-effort: 0 on a
    /// failed read (the monitor's own `Snapshot::heading_deg`).
    pub(crate) heading_deg: f64,
    pub(crate) fuel_kg: Option<f64>,
    pub(crate) endurance_min: Option<f64>,
}

/// A tick older than this is no measurement (the monitor stopped: sim gone,
/// paused, loading).
pub(crate) const TICK_FRESH: Duration = Duration::from_secs(10);

/// The monitor's last tick (session, with its own expiry: [`TICK_FRESH`]).
static TICK: Mutex<Option<(Instant, Tick)>> = Mutex::new(None);

pub(crate) fn note_tick(t: Tick) {
    *TICK.lock().unwrap_or_else(|e| e.into_inner()) = Some((Instant::now(), t));
}

/// The last tick, if fresh.
pub(crate) fn tick() -> Option<Tick> {
    TICK.lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .filter(|(at, _)| at.elapsed() < TICK_FRESH)
        .map(|(_, t)| t.clone())
}

/// The taxi guide's turn card as the monitor computed it for the panel
/// (`taxi_guide::Guide::turn_card`), `None` with no guide, with its own
/// expiry ([`TICK_FRESH`]).
static CARD: Mutex<Option<(Instant, Option<crate::efb::airport::CardFacts>)>> = Mutex::new(None);

pub(crate) fn note_turn_card(card: Option<&crate::taxi_guide::TurnCard>) {
    *CARD.lock().unwrap_or_else(|e| e.into_inner()) =
        Some((Instant::now(), card.map(crate::efb::airport::CardFacts::of)));
}

/// The last turn card, if fresh.
pub(crate) fn turn_card() -> Option<crate::efb::airport::CardFacts> {
    CARD.lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .filter(|(at, _)| at.elapsed() < TICK_FRESH)
        .and_then(|(_, c)| c.clone())
}

/// COM1 as the runway watch read it (kHz), with its own expiry.
static COM1: Mutex<Option<(Instant, i64)>> = Mutex::new(None);

pub(crate) fn note_com1(khz: Option<i64>) {
    if let Some(k) = khz {
        *COM1.lock().unwrap_or_else(|e| e.into_inner()) = Some((Instant::now(), k));
    }
}

pub(crate) fn com1() -> Option<i64> {
    COM1.lock()
        .unwrap_or_else(|e| e.into_inner())
        .filter(|(at, _)| at.elapsed() < Duration::from_secs(30))
        .map(|(_, k)| k)
}

/// The callsign as the one resolver last said it (the monitor, every 5 s),
/// with its own expiry.
static CALLSIGN: Mutex<Option<(Instant, String)>> = Mutex::new(None);

pub(crate) fn note_callsign(spoken: &str) {
    *CALLSIGN.lock().unwrap_or_else(|e| e.into_inner()) =
        Some((Instant::now(), spoken.trim().to_string()));
}

pub(crate) fn callsign() -> Option<String> {
    CALLSIGN
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .filter(|(at, s)| at.elapsed() < Duration::from_secs(30) && !s.is_empty())
        .map(|(_, s)| s.clone())
}

/// A verdict the crew reached about the plan and said once.
#[derive(Clone, Debug, PartialEq, serde::Serialize)]
pub(crate) struct Verdict {
    /// "pass", "caution" or "stop".
    pub(crate) level: &'static str,
    pub(crate) lines: Vec<String>,
    /// What could not be checked (the audit's own list).
    pub(crate) unchecked: Vec<String>,
    pub(crate) t: f64,
}

/// The last plan audit (`plan_audit::check`), this flight's only.
pub(crate) static AUDIT: Scoped<Verdict> = Scoped::flight();
/// The last full-auto preflight at the arm, this flight's only.
pub(crate) static PREFLIGHT: Scoped<Verdict> = Scoped::flight();

/// Retain a plan audit, stamped with the flight it was begun in.
pub(crate) fn note_audit(generation: u64, a: &crate::plan_audit::Audit) {
    use crate::plan_audit::Severity;
    let level = if a.stop().is_some() {
        "stop"
    } else if a.cautions().next().is_some() {
        "caution"
    } else {
        "pass"
    };
    let lines = a
        .findings
        .iter()
        .map(|f| {
            let tag = if f.severity == Severity::Stop {
                "Refused"
            } else {
                "Caution"
            };
            format!("{tag}: {}", f.line)
        })
        .collect();
    AUDIT.set_in(
        generation,
        Verdict {
            level,
            lines,
            unchecked: a.unchecked.iter().map(|s| s.to_string()).collect(),
            t: now_unix(),
        },
    );
}

/// Retain the arm's preflight: its brief, or the refusal that names the number.
pub(crate) fn note_preflight(generation: u64, verdict: &Result<String, String>) {
    let (level, line) = match verdict {
        Ok(brief) => ("pass", brief.clone()),
        Err(why) => ("stop", why.clone()),
    };
    PREFLIGHT.set_in(
        generation,
        Verdict {
            level,
            lines: vec![line],
            unchecked: Vec::new(),
            t: now_unix(),
        },
    );
}

/// A crew alert as the voice mirrored it to the panel (`Voice::alert`,
/// `narrate`, `say_crew`): the level the panel colours it by.
#[derive(Clone, Debug, PartialEq, serde::Serialize)]
pub(crate) struct Alert {
    /// Rises by one per alert in this process: the page keys its local
    /// acknowledgement on it.
    pub(crate) id: u64,
    pub(crate) t: f64,
    /// "warning", "caution" or "advisory".
    pub(crate) level: &'static str,
    pub(crate) text: String,
    /// The flight it was said in (`flight_state::flight_gen`).
    #[serde(skip)]
    pub(crate) flight: u64,
}

/// Alerts kept: plenty for one flight's warnings and cautions.
const ALERTS_MAX: usize = 40;

/// The crew's alerts, oldest first, each stamped with its flight; read
/// through [`alerts`], which keeps this flight's only.
static ALERTS: Mutex<VecDeque<Alert>> = Mutex::new(VecDeque::new());
static ALERT_SEQ: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// An alert the voice mirrored (its priority as the panel's level word).
pub(crate) fn note_alert(level: &'static str, text: &str) {
    let text = crate::phrase::scrub_dashes(text.trim());
    if text.is_empty() {
        return;
    }
    let id = ALERT_SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed) + 1;
    let mut a = ALERTS.lock().unwrap_or_else(|e| e.into_inner());
    a.push_back(Alert {
        id,
        t: now_unix(),
        level,
        text,
        flight: crate::flight_state::flight_gen(),
    });
    while a.len() > ALERTS_MAX {
        a.pop_front();
    }
}

/// This flight's alerts, oldest first: a new flight never shows the last
/// one's warning.
pub(crate) fn alerts() -> Vec<Alert> {
    let now = crate::flight_state::flight_gen();
    ALERTS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .iter()
        .filter(|a| a.flight == now)
        .cloned()
        .collect()
}

/// The panel's activity code as the reporter last set it (0 idle, 1
/// listening, 2 thinking, 3 speaking; `u8::MAX` before the first).
static ACTIVITY: std::sync::atomic::AtomicU8 = std::sync::atomic::AtomicU8::new(u8::MAX);

pub(crate) fn note_activity(code: u8) {
    ACTIVITY.store(code, std::sync::atomic::Ordering::Relaxed);
}

/// "idle", "listening", "thinking" or "speaking"; `None` before the
/// command loop said anything.
pub(crate) fn activity() -> Option<&'static str> {
    match ACTIVITY.load(std::sync::atomic::Ordering::Relaxed) {
        0 => Some("idle"),
        1 => Some("listening"),
        2 => Some("thinking"),
        3 => Some("speaking"),
        _ => None,
    }
}

#[cfg(test)]
pub(crate) static TEST_LOCK: Mutex<()> = Mutex::new(());

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_crew_transmission_is_marked_radio_once() {
        let _g = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        crew_transmits("Gatwick Tower, XVL214, ready.");
        note_intercom("Gatwick Tower, XVL214, ready.", false);
        note_intercom("Gatwick Tower, XVL214, ready.", false);
        let l = lines();
        let mine: Vec<&Line> = l
            .iter()
            .filter(|x| x.text == "Gatwick Tower, XVL214, ready.")
            .collect();
        assert!(mine.len() >= 2);
        let last_two = &mine[mine.len() - 2..];
        assert!(last_two[0].radio, "the transmission");
        assert!(!last_two[1].radio, "the same words again are the intercom");
    }

    /// Alerts are numbered, bounded and this flight's only.
    #[test]
    fn alerts_are_kept_for_their_flight() {
        let _g = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        for i in 0..(ALERTS_MAX + 5) {
            note_alert("caution", &format!("caution {i}"));
        }
        note_alert("warning", "Pull up \u{2014} terrain");
        note_alert("warning", "   ");
        let a = alerts();
        assert!(a.len() <= ALERTS_MAX);
        let last = a.last().unwrap();
        assert_eq!(last.text, "Pull up, terrain", "no dash reaches the page");
        assert_eq!(last.level, "warning");
        assert!(a.windows(2).all(|w| w[0].id < w[1].id));
        let other = ALERTS.lock().unwrap().back().unwrap().flight + 1;
        ALERTS.lock().unwrap().back_mut().unwrap().flight = other;
        assert_ne!(
            alerts().last().map(|x| x.text.clone()),
            Some("Pull up, terrain".into()),
            "another flight's warning is not this one's"
        );
    }

    #[test]
    fn the_log_is_bounded() {
        let _g = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        for i in 0..(LINES_MAX + 10) {
            note_pilot(&format!("line {i}"), false);
        }
        assert_eq!(lines().len(), LINES_MAX);
        note_pilot("   ", false);
        assert_eq!(lines().last().map(|l| l.text.as_str()), Some("line 89"));
    }
}
