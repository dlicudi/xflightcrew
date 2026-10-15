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
    /// The station position's tag ("TWR", "APP"): the speaker of a station
    /// line, the position COM1 was on for our own transmission.
    pub(crate) tag: Option<&'static str>,
    /// COM1 (kHz) as the runway watch last read it, for a radio line: the
    /// Radio page groups the log under its frequency.
    pub(crate) khz: Option<i64>,
    /// A party-line exchange not addressed to us (the voice's `overheard`).
    pub(crate) overheard: bool,
    /// The crew's transmission was a readback (the radio drain's
    /// `queued_readback`).
    pub(crate) readback: bool,
}

/// Lines kept: the Now page shows eight, the Radio page's log the rest.
const LINES_MAX: usize = 80;

/// The conversation, oldest first (session: a log of what was said).
static LINES: Mutex<VecDeque<Line>> = Mutex::new(VecDeque::new());

/// Crew transmissions handed to the intercom speaker but not yet started
/// (with whether each is a readback): the crew's radio calls are spoken by
/// the copilot's own voice, so the speaker thread cannot tell them from
/// intercom lines by itself.
static RADIO_PENDING: Mutex<Vec<(Instant, String, bool)>> = Mutex::new(Vec::new());

/// COM1 now and the position it is on, for a radio line's frequency header.
fn tuned_now() -> (Option<i64>, Option<&'static str>) {
    let khz = com1();
    (
        khz,
        khz.and_then(crate::party_line::tuned_station).map(|s| s.tag()),
    )
}

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
    let (khz, tag) = if radio { tuned_now() } else { (None, None) };
    push(Line {
        t: now_unix(),
        who: Who::Pilot,
        text: crate::phrase::scrub_dashes(text.trim()),
        radio,
        tag,
        khz,
        overheard: false,
        readback: false,
    });
}

/// The station took the pilot's last line as a radio call (an unkeyed
/// call the content router handed to the tower): it went out on the
/// frequency, not the intercom.
pub(crate) fn pilot_line_was_radio() {
    let (khz, tag) = tuned_now();
    let mut l = LINES.lock().unwrap_or_else(|e| e.into_inner());
    if let Some(last) = l.iter_mut().rev().find(|x| x.who == Who::Pilot) {
        if !last.radio {
            last.radio = true;
            last.khz = khz;
            last.tag = tag;
        }
    }
}

/// The crew is about to key the radio with these words (the crew radio
/// drain, before `say`): the intercom speaker marks the line as radio when
/// it starts.
pub(crate) fn crew_transmits(text: &str, readback: bool) {
    let mut p = RADIO_PENDING.lock().unwrap_or_else(|e| e.into_inner());
    p.retain(|(at, _, _)| at.elapsed() < Duration::from_secs(120));
    p.push((
        Instant::now(),
        crate::phrase::scrub_dashes(text.trim()).trim().to_string(),
        readback,
    ));
}

/// Is this line a crew transmission handed to the speaker and not yet
/// started? (The panel's transcript labels it as ours on the radio.)
pub(crate) fn crew_radio_pending(text: &str) -> bool {
    let text = crate::phrase::scrub_dashes(text.trim());
    let text = text.trim();
    RADIO_PENDING
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .iter()
        .any(|(_, t, _)| t == text)
}

/// COM1's position now ("TWR"), for a label of a line keyed on it.
pub(crate) fn tuned_tag() -> Option<&'static str> {
    tuned_now().1
}

/// An intercom line started playing (the copilot or the cabin).
pub(crate) fn note_intercom(text: &str, cabin: bool) {
    let text = text.trim();
    let sent = if cabin {
        None
    } else {
        let mut p = RADIO_PENDING.lock().unwrap_or_else(|e| e.into_inner());
        p.iter()
            .position(|(_, t, _)| t == text)
            .map(|i| p.remove(i).2)
    };
    let (khz, tag) = if sent.is_some() {
        tuned_now()
    } else {
        (None, None)
    };
    push(Line {
        t: now_unix(),
        who: if cabin { Who::Cabin } else { Who::Crew },
        text: text.to_string(),
        radio: sent.is_some(),
        tag,
        khz,
        overheard: false,
        readback: sent.unwrap_or(false),
    });
}

/// A station line started playing on the radio (`overheard`: a party-line
/// exchange not addressed to us).
pub(crate) fn note_station(text: &str, speaker: crate::voice::StationSpeaker, overheard: bool) {
    use crate::voice::StationSpeaker as S;
    let who = match speaker {
        S::Tower | S::Approach | S::Departure | S::Ground | S::Delivery => Who::Atc,
        S::Atis => Who::Atis,
        S::Ghost { .. } => Who::Traffic,
    };
    push(Line {
        t: now_unix(),
        who,
        text: text.trim().to_string(),
        radio: true,
        tag: Some(speaker.tag()),
        khz: com1(),
        overheard,
        readback: false,
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

/// Who has the lateral while the follower flies, as its loop last decided:
/// `true` while the aircraft's LNAV flies the leg (`LnavLateral::owns_lateral`),
/// `false` while the follower's heading bug does. Its own expiry: the loop
/// gone, nobody's lateral is shown.
static LATERAL: Mutex<Option<(Instant, bool)>> = Mutex::new(None);

pub(crate) fn note_lateral(lnav_owns: bool) {
    *LATERAL.lock().unwrap_or_else(|e| e.into_inner()) = Some((Instant::now(), lnav_owns));
}

/// `Some(true)`: LNAV flies; `Some(false)`: the follower's bug; `None`: no
/// follower ticked lately.
pub(crate) fn lateral() -> Option<bool> {
    LATERAL
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .filter(|(at, _)| at.elapsed() < TICK_FRESH)
        .map(|(_, v)| v)
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

/// Every simulated field's flight information board, as the traffic world
/// last built it (`traffic::board::payload`, by ICAO, in the world's
/// order), with its own expiry: a world that stopped ticking (traffic off,
/// stood down) is a dash on the page within 30 s, never a frozen board.
static BOARDS: Mutex<Option<(Instant, Vec<(String, String)>)>> = Mutex::new(None);

/// The traffic world's tap (`TrafficWorld::note_efb_boards`).
pub(crate) fn note_boards(boards: Vec<(String, String)>) {
    *BOARDS.lock().unwrap_or_else(|e| e.into_inner()) = Some((Instant::now(), boards));
}

pub(crate) fn boards() -> Option<Vec<(String, String)>> {
    BOARDS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .filter(|(at, b)| at.elapsed() < Duration::from_secs(30) && !b.is_empty())
        .map(|(_, b)| b.clone())
}

/// One aircraft of our traffic world as its fleet holds it, for the Map
/// tab's airport (2026-10-13): where it is and the board's words for it.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct TrafficAc {
    pub(crate) id: u32,
    pub(crate) callsign: String,
    /// The callsign as the radio says it ("Easy eight nine zero one").
    pub(crate) spoken: String,
    /// The airline's name from the board ("easyJet"); empty for none.
    pub(crate) airline: String,
    pub(crate) icao_type: String,
    /// The two ends, ICAO; empty where neither the board nor the plan says.
    pub(crate) from: String,
    pub(crate) to: String,
    /// The phase in plain words (`airport::traffic_phase_words`).
    pub(crate) phase: &'static str,
    /// The stand it holds or is bound for, as the board labels it.
    pub(crate) stand: Option<String>,
    /// The timetable's time and today's estimate, local "HH:MM".
    pub(crate) sched: Option<String>,
    pub(crate) est: Option<String>,
    pub(crate) lat: f64,
    pub(crate) lon: f64,
    /// Degrees true.
    pub(crate) hdg: f64,
    pub(crate) alt_ft: f64,
    pub(crate) on_ground: bool,
    pub(crate) gs_kts: f64,
    /// The route still ahead (lat, lon), thinned ([`traffic::aircraft::Route::ahead`]).
    pub(crate) ahead: Vec<(f64, f64)>,
    /// The whole flight, origin to destination (2026-10-13), where the
    /// timetable places both ends.
    pub(crate) journey: Option<Journey>,
    /// Flown by the whole-flight model, outside the full simulation
    /// (`traffic::track`).
    pub(crate) modeled: bool,
}

/// A flight's whole journey as FlightAware shows it (2026-10-13,
/// `traffic::track`): both ends, the off-block on the origin's clock and
/// the on-block (estimated) on the destination's, the time since the one
/// and to the other, and how far along the great circle it is.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Journey {
    pub(crate) from: String,
    pub(crate) to: String,
    pub(crate) from_pos: (f64, f64),
    pub(crate) to_pos: (f64, f64),
    /// Local "HH:MM".
    pub(crate) off: String,
    pub(crate) on: String,
    /// Seconds since the off-block (negative: still to come) and to the
    /// on-block.
    pub(crate) since_s: f64,
    pub(crate) to_go_s: f64,
    /// 0 to 1.
    pub(crate) progress: f64,
}

/// A stand one of our traffic holds ([`traffic::fleet::Fleet::stand_holder`]).
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct StandTaken {
    /// The apt.dat name, as the diagram draws it.
    pub(crate) name: String,
    pub(crate) by: String,
    /// "parked", "taxiing in" or "due".
    pub(crate) how: &'static str,
}

/// One simulated field: its aircraft and the stands they hold; with an
/// empty ICAO, the flights no field simulates (`whole::WorldBoard`).
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct TrafficField {
    pub(crate) icao: String,
    pub(crate) aircraft: Vec<TrafficAc>,
    pub(crate) taken: Vec<StandTaken>,
}

/// A traffic tap older than this is no traffic world: the stands go back
/// to unknown, never to free.
pub(crate) const TRAFFIC_FRESH: Duration = Duration::from_secs(5);

/// Every simulated field as the traffic world last tapped it (about once a
/// second while the EFB serves, `TrafficWorld::note_efb_traffic`), with its
/// own expiry ([`TRAFFIC_FRESH`]).
static TRAFFIC: Mutex<Option<(Instant, Vec<TrafficField>)>> = Mutex::new(None);

/// The traffic world's tap.
pub(crate) fn note_traffic(fields: Vec<TrafficField>) {
    *TRAFFIC.lock().unwrap_or_else(|e| e.into_inner()) = Some((Instant::now(), fields));
}

/// The simulated fields, if a traffic world ticked lately (`Some` of none:
/// a world that simulates no field yet).
pub(crate) fn traffic() -> Option<Vec<TrafficField>> {
    TRAFFIC
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .filter(|(at, _)| at.elapsed() < TRAFFIC_FRESH)
        .map(|(_, f)| f.clone())
}

/// Who of our traffic holds `stand` at `icao` ("EZY123, parked"), as last
/// tapped; `None` when nobody does or no world ticked lately.
pub(crate) fn stand_taken_by(icao: &str, stand: &str) -> Option<String> {
    TRAFFIC
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .filter(|(at, _)| at.elapsed() < TRAFFIC_FRESH)?
        .1
        .iter()
        .find(|f| f.icao.eq_ignore_ascii_case(icao))?
        .taken
        .iter()
        .find(|t| t.name.eq_ignore_ascii_case(stand))
        .map(|t| format!("{}, {}", t.by, t.how))
}

/// A move to a stand asked on the Map tab (`POST /reposition`), as it went.
#[derive(Clone, Debug, PartialEq, serde::Serialize)]
pub(crate) struct Reposition {
    /// Rises by one per note: the page toasts each new one.
    pub(crate) id: u64,
    pub(crate) t: f64,
    pub(crate) stand: String,
    /// "asked", "moving", "done", "refused" or "failed".
    pub(crate) state: &'static str,
    pub(crate) text: String,
}

/// The last move's word (session, with its own expiry: two minutes).
static REPOSITION: Mutex<Option<(Instant, Reposition)>> = Mutex::new(None);

pub(crate) fn note_reposition(stand: &str, state: &'static str, text: &str) {
    let mut r = REPOSITION.lock().unwrap_or_else(|e| e.into_inner());
    let id = r.as_ref().map_or(1, |(_, x)| x.id + 1);
    *r = Some((
        Instant::now(),
        Reposition {
            id,
            t: now_unix(),
            stand: stand.to_string(),
            state,
            text: crate::phrase::scrub_dashes(text.trim()),
        },
    ));
}

pub(crate) fn reposition() -> Option<Reposition> {
    REPOSITION
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .filter(|(at, _)| at.elapsed() < Duration::from_secs(120))
        .map(|(_, r)| r.clone())
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

/// Withdraw every alert whose text starts with `prefix`: the condition it
/// reported is gone (a stale plugin replaced and reloaded, 2026-10-06: the
/// EFB kept "Plugin check: the loaded plugin is out of date" after a
/// reload had fixed it). Returns how many went.
pub(crate) fn withdraw_alerts(prefix: &str) -> usize {
    let mut a = ALERTS.lock().unwrap_or_else(|e| e.into_inner());
    let before = a.len();
    a.retain(|x| !x.text.starts_with(prefix));
    before - a.len()
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
        crew_transmits("Gatwick Tower, XVL214, ready.", true);
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
        assert!(last_two[0].readback, "a readback, as the drain said");
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
