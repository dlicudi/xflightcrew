//! The crew radio queue: every call the crew makes to the world, queued
//! for the main loop to transmit. Moved out of `circuit.rs` verbatim
//! (refactor audit 2026-10-04, phase 2); every caller names `crew_radio::`
//! itself since the `circuit` re-export went (pillars audit 2026-10-07,
//! stage A).

use std::collections::VecDeque;
use std::sync::Mutex;

use crate::xplane::XPlaneClient;

/// Radio calls the copilot makes on our behalf during the circuit — the ONE
/// place the crew talks to the world. The circuit thread (which doesn't own the
/// station) pushes a position report here; the main loop (which does) drains it
/// each tick, routes it through the tuned tower like a keyed pilot call, and
/// speaks both the copilot's transmission and the tower's reply. Empty in
/// copilot-only mode or with no station — the calls are simply discarded, since
/// real/external ATC owns the radio then.
static CREW_RADIO: Mutex<VecDeque<Queued>> = Mutex::new(VecDeque::new());

/// A call the crew has queued for the main loop to transmit. `Text` is the
/// finished sentence; `CheckIn` is rendered by the main loop the moment it
/// is about to be spoken, because its altitude must be the one passing
/// THEN: read when queued, the LXGB check-in said "passing one thousand"
/// at 1,207 ft, twenty seconds and a party-line exchange later
/// (08:23Z, 2026-09-15).
#[derive(Debug, Clone, PartialEq)]
pub enum CrewCall {
    Text(String),
    CheckIn {
        callsign: Option<String>,
        to: String,
        climb_to_ft: f64,
    },
}

/// What a queued call is still true for. A POSITION REPORT carries the
/// condition it reports and dies with it: at LEMG (sortie
/// 1790584300-approach, 2026-09-28) four "5 mile final runway one three"
/// calls queued at 4.7 nm went out 93 s later, in the flare, at touchdown,
/// on the rollout and stopped. Requests and readbacks keep their own rules
/// (`Always`): a readback owed stays owed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Holds {
    /// No condition of its own: a request or a readback.
    Always,
    /// True while airborne, above the last [`REPORT_LANDING_AGL_FT`] when
    /// `before_landing`, for `max_age_s`, and (when set) until the aircraft
    /// has flown `max_travel_nm` from where it was made.
    Airborne {
        max_age_s: f64,
        max_travel_nm: Option<f64>,
        before_landing: bool,
    },
    /// True while on the ground, for `max_age_s` ("runway vacated").
    OnGround { max_age_s: f64 },
}

/// A position report is not made in the flare: the last 100 ft is the
/// flight model's landing phase.
pub const REPORT_LANDING_AGL_FT: f64 = crate::flight::LANDING_AGL_FT;

/// One entry in the crew radio queue: the call, what it holds for, and when
/// it was queued.
#[derive(Debug, Clone)]
pub struct Queued {
    pub call: CrewCall,
    pub holds: Holds,
    pub at: std::time::Instant,
    /// Whether the call is a readback, and whose words it copies.
    pub readback: Readback,
}

/// A queued call as a readback. An `OwedCopy` is the tower's own suggested
/// words (`party_line::readback_owed`), read back only while the tower still
/// waits for THEM: LEMG 13 (2026-09-28, sortie 1790627884-full_auto) queued
/// the circuit clearance's readback at +13.7 and again at +34.0 (the first
/// had not gone out), and the second went out at +63.2 in the takeoff roll,
/// earning "read back: ... cleared for takeoff". `Own` is the crew's own
/// words ("cleared for takeoff runway one three"), judged by the tower.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Readback {
    No,
    Own,
    OwedCopy,
}

/// The crew's latest readback: its words, when queued and when it went out
/// (routed to the frequency, or discarded with the radio). One slot: a crew
/// reads back one instruction at a time.
#[derive(Debug, Clone, PartialEq)]
pub struct ReadbackRec {
    pub words: String,
    pub queued_at: std::time::Instant,
    pub out_at: Option<std::time::Instant>,
}

static LAST_READBACK: Mutex<Option<ReadbackRec>> = Mutex::new(None);

/// A call's words for comparing readbacks: lower case, no closing stop.
pub fn readback_words(call: &str) -> String {
    call.trim().trim_end_matches('.').trim().to_ascii_lowercase()
}

/// An owed-words copy that is no longer what the tower waits for: it was
/// heard (nothing owed) or the tower now waits for another readback. Pure.
pub fn owed_copy_stale(call: &str, owed: Option<&str>) -> bool {
    !owed.is_some_and(|o| readback_words(o) == readback_words(call))
}

/// The latest readback's record, if any.
pub fn last_readback() -> Option<ReadbackRec> {
    LAST_READBACK.lock().unwrap_or_else(|e| e.into_inner()).clone()
}

/// The readback of `call` has gone out (or was discarded with the radio):
/// the main loop's word, as it routes it.
pub fn note_readback_out(call: &str) {
    let words = readback_words(call);
    let mut rec = LAST_READBACK.lock().unwrap_or_else(|e| e.into_inner());
    if let Some(r) = rec.as_mut().filter(|r| r.words == words && r.out_at.is_none()) {
        r.out_at = Some(std::time::Instant::now());
    }
}

/// Whether the readback of `call` has gone out.
pub fn readback_out(call: &str) -> bool {
    let words = readback_words(call);
    last_readback().is_some_and(|r| r.words == words && r.out_at.is_some())
}

/// The facts a queued call is judged against as it is about to go out, read
/// live by the main loop. `None` is a failed read: no measurement, and a
/// call is never dropped on it (fail open).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RadioFacts {
    pub on_ground: Option<bool>,
    pub agl_ft: Option<f64>,
    pub gs_kts: Option<f64>,
}

/// What a crew call holds for, from its words. The words are matched per
/// comma segment at the START of a segment, so the callsign in front and a
/// runway after ("inbound for landing, runway one three") do not matter, and
/// a readback of a controller's words ("join final", "report downwind") is
/// never taken for our report.
pub fn holds_for(call: &CrewCall) -> Holds {
    let text = match call {
        CrewCall::CheckIn { .. } => {
            return Holds::Airborne {
                max_age_s: 90.0,
                max_travel_nm: None,
                before_landing: false,
            }
        }
        CrewCall::Text(t) => t.to_ascii_lowercase(),
    };
    for seg in text.split(',').map(str::trim) {
        let mut words = seg.split_whitespace();
        let first = words.next().unwrap_or("");
        let second = words.next().unwrap_or("");
        let third = words.next().unwrap_or("");
        // "5 mile final": a distance, true for about a mile of flying.
        if first.parse::<f64>().is_ok() && second.starts_with("mile") && third == "final" {
            return Holds::Airborne {
                max_age_s: 60.0,
                max_travel_nm: Some(1.0),
                before_landing: true,
            };
        }
        match (first, second) {
            // "Established ILS runway two six left": the instrument
            // arrival's final report, true as long as the final is.
            ("final", _) | ("established", _) => {
                return Holds::Airborne {
                    max_age_s: 60.0,
                    max_travel_nm: None,
                    before_landing: true,
                }
            }
            ("downwind", _) => {
                return Holds::Airborne {
                    max_age_s: 60.0,
                    max_travel_nm: Some(1.5),
                    before_landing: true,
                }
            }
            ("turning", "base") | ("base", _) => {
                return Holds::Airborne {
                    max_age_s: 45.0,
                    max_travel_nm: Some(1.0),
                    before_landing: true,
                }
            }
            ("going", "around") | ("missed", "approach") => {
                return Holds::Airborne {
                    max_age_s: 30.0,
                    max_travel_nm: None,
                    before_landing: false,
                }
            }
            ("inbound", "for") => {
                return Holds::Airborne {
                    max_age_s: 120.0,
                    max_travel_nm: None,
                    before_landing: true,
                }
            }
            ("runway", "vacated") => return Holds::OnGround { max_age_s: 300.0 },
            _ => {}
        }
    }
    Holds::Always
}

/// Why a queued call is no longer true, `None` while it still is. Pure:
/// `age_s` since it was queued, the live facts as it is about to go out.
/// The distance flown is the age at the present groundspeed.
pub fn stale_reason(holds: Holds, age_s: f64, facts: &RadioFacts) -> Option<&'static str> {
    match holds {
        Holds::Always => None,
        Holds::OnGround { max_age_s } => {
            if facts.on_ground == Some(false) {
                Some("airborne")
            } else if age_s > max_age_s {
                Some("max_age")
            } else {
                None
            }
        }
        Holds::Airborne {
            max_age_s,
            max_travel_nm,
            before_landing,
        } => {
            if facts.on_ground == Some(true) {
                return Some("on_ground");
            }
            if before_landing && facts.agl_ft.is_some_and(|h| h < REPORT_LANDING_AGL_FT) {
                return Some("landing");
            }
            if let (Some(max), Some(gs)) = (max_travel_nm, facts.gs_kts) {
                if age_s * gs / 3600.0 > max {
                    return Some("moved_on");
                }
            }
            if age_s > max_age_s {
                return Some("max_age");
            }
            None
        }
    }
}

/// The crew radio's words for the log, a check-in by its destination.
pub fn call_words(call: &CrewCall) -> String {
    match call {
        CrewCall::Text(t) => t.clone(),
        CrewCall::CheckIn { to, .. } => format!("check-in with {to}"),
    }
}

/// The crew radio queue is process-wide: its tests take turns.
#[cfg(test)]
pub(crate) static QUEUE_LOCK: Mutex<()> = Mutex::new(());

/// The ended flight's calls, forgotten at the new-flight door
/// (`flight_state::forget_flight`): a report or readback queued for the
/// last flight is said to nobody on this one, and its readback record must
/// not hold back the same words owed again.
pub fn forget_flight_calls() {
    CREW_RADIO.lock().unwrap_or_else(|e| e.into_inner()).clear();
    *LAST_READBACK.lock().unwrap_or_else(|e| e.into_inner()) = None;
}

/// Withdraw every call still queued, unsent: the mission that queued them
/// has ended (`full_auto::give_up`). Returns their words for the blackbox.
pub fn withdraw_queued() -> Vec<String> {
    let mut q = CREW_RADIO.lock().unwrap_or_else(|e| e.into_inner());
    q.drain(..).map(|c| call_words(&c.call)).collect()
}

/// Drain the pending crew radio calls (main loop only).
pub fn take_crew_radio() -> Vec<Queued> {
    let mut q = CREW_RADIO.lock().unwrap_or_else(|e| e.into_inner());
    q.drain(..).collect()
}

/// A crew call is still queued for the main loop, not yet transmitted: the
/// main loop drains the queue only between its blocking speech, so a
/// readback can wait here behind the clearance it answers. Full auto starts
/// the readback's acceptance clock once this is false.
pub fn crew_radio_pending() -> bool {
    !CREW_RADIO
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .is_empty()
}

/// Queue a call, collapsing it into an identical one queued just before it
/// and not yet drained: stacked approach setups at LEMG queued four
/// identical "5 mile final" reports in the same tenth of a second
/// (2026-09-28).
fn enqueue(call: CrewCall) {
    enqueue_as(call, Readback::No);
}

fn enqueue_as(call: CrewCall, readback: Readback) {
    let mut q = CREW_RADIO.lock().unwrap_or_else(|e| e.into_inner());
    if q.back().is_some_and(|b| b.call == call) {
        drop(q);
        crate::blackbox::record(
            "crew_radio",
            serde_json::json!({ "event": "crew_radio_collapsed", "call": call_words(&call) }),
        );
        return;
    }
    let holds = holds_for(&call);
    q.push_back(Queued {
        call,
        holds,
        at: std::time::Instant::now(),
        readback,
    });
}

/// Queue a radio call for the main loop to transmit. Public so the full-auto
/// orchestrator can request taxi through the same channel.
pub fn transmit(call: String) {
    enqueue(CrewCall::Text(call));
}

/// Queue a READBACK: recorded, so the crew never queues the same words again
/// before they have gone out ([`last_readback`]) and the roll can wait for
/// them ([`readback_out`]). `owed_copy` marks the tower's own suggested
/// words, which die once the tower no longer waits for them.
pub fn transmit_readback(call: String, owed_copy: bool) {
    *LAST_READBACK.lock().unwrap_or_else(|e| e.into_inner()) = Some(ReadbackRec {
        words: readback_words(&call),
        queued_at: std::time::Instant::now(),
        out_at: None,
    });
    let kind = if owed_copy { Readback::OwedCopy } else { Readback::Own };
    enqueue_as(CrewCall::Text(call), kind);
}

/// Queue the departure check-in, rendered with the live altitude when spoken.
pub fn transmit_check_in(callsign: Option<String>, to: String, climb_to_ft: f64) {
    enqueue(CrewCall::CheckIn {
        callsign,
        to,
        climb_to_ft,
    });
}

/// The aircraft's callsign, spoken the way the tower says it ("Speedbird
/// three zero six", "November one seven two Sierra Papa"), so the copilot's
/// calls sound like a real pilot's. The tower's own resolver
/// (`atc::callsign`: override > SimBrief > tail, wake suffix last), so crew
/// and controller can never call the aircraft two names. `None` if no source
/// says anything.
pub async fn crew_callsign(xp: &XPlaneClient) -> Option<String> {
    crate::callsign::live_with(&callsign_inputs(), xp.view())
        .await
        .map(|r| r.spoken)
}

/// What the crew knows of the callsign that the sim does not, read at the
/// moment of asking (so a replaced plan is a new callsign and a cleared one
/// none): the pilot's override (`XCOPILOT_CALLSIGN`) and a SimBrief plan's
/// ATC callsign with its type, for `callsign::plan_fits` to hold against the
/// airframe loaded. Registered with the station at boot
/// (`callsign::provide_inputs`).
pub fn callsign_inputs() -> crate::callsign::Inputs {
    let over = crate::settings::live().callsign.trim().to_string();
    crate::callsign::Inputs {
        override_id: (!over.is_empty()).then_some(over),
        plan: plan_callsign(crate::simbrief::loaded_plan().as_ref()),
    }
}

/// A plan's callsign, only from a SimBrief plan (a local one names no flight).
pub(crate) fn plan_callsign(
    plan: Option<&crate::simbrief::FlightPlan>,
) -> Option<crate::callsign::PlanCallsign> {
    let p = plan.filter(|p| p.source == crate::simbrief::PlanSource::SimBrief)?;
    Some(crate::callsign::PlanCallsign {
        id: p.callsign.clone()?,
        plan_type: p.aircraft.clone(),
        also_types: crate::aircraft::live_profile()
            .map(|a| a.simbrief_types.clone())
            .unwrap_or_default(),
    })
}

/// Obey a "contact X on F" an aerodrome position said to us (Ground at the
/// holding point, the Tower once vacated, a call on the wrong frequency;
/// `party_line::take_position_handoff`): read it back on the frequency that
/// gave it ("Gatwick Tower on one two four decimal two three zero", a
/// frequency readback the controller answers with silence), let it go
/// out, then retune COM1, as the departure handoff is obeyed. Returns the
/// frequency tuned, `None` with no handoff owed.
pub async fn follow_position_handoff(
    xp: &dyn crate::xplane::XPlane,
    callsign: &Option<String>,
) -> Option<i64> {
    follow_handoff(xp, callsign, crate::party_line::take_position_handoff()).await
}

/// The readback of a position handoff: "Gatwick Ground on one two one
/// decimal eight zero five", the frequency readback the controller
/// answers with silence (`tower::is_readback`).
pub fn position_readback(callsign: &Option<String>, to: &str, khz: i64) -> String {
    radio_call(callsign, &format!("{to} on {}", crate::atis::spoken_mhz(khz)))
}

/// [`follow_position_handoff`] for the handoff taken.
pub async fn follow_handoff(
    xp: &dyn crate::xplane::XPlane,
    callsign: &Option<String>,
    handoff: Option<(String, i64)>,
) -> Option<i64> {
    let (to, khz) = handoff?;
    transmit(position_readback(callsign, &to, khz));
    // The main loop routes a queued call on the frequency COM1 is on when
    // it drains it: the readback goes out on the OLD frequency only if the
    // radio is retuned after it has gone.
    let t0 = std::time::Instant::now();
    while crew_radio_pending() && t0.elapsed() < std::time::Duration::from_secs(20) {
        tokio::time::sleep(std::time::Duration::from_millis(250)).await;
    }
    tokio::time::sleep(std::time::Duration::from_millis(1500)).await;
    let tuned = xp
        .set_float(crate::datarefs::COM1_FREQ_KHZ, khz as f64)
        .await
        .is_ok();
    crate::blackbox::record(
        "crew_radio",
        serde_json::json!({ "event": "position_handoff", "to": to, "khz": khz, "tuned": tuned }),
    );
    Some(khz)
}

/// Tune COM1 to the position a call is for (`khz`: Ground's for the push
/// and the taxi, the Tower's for the ready call), obeying a handoff owed
/// first ([`follow_position_handoff`]). Nothing to tune with `None`.
pub async fn work_position(
    xp: &dyn crate::xplane::XPlane,
    callsign: &Option<String>,
    khz: Option<i64>,
) {
    let handed = follow_position_handoff(xp, callsign).await;
    if let Some(k) = khz.filter(|k| handed != Some(*k)) {
        let _ = xp.set_float(crate::datarefs::COM1_FREQ_KHZ, k as f64).await;
    }
}

/// Format a radio call with the callsign in front, pilot-style: "November one
/// seven two Sierra Papa, downwind runway two seven". No callsign → bare.
pub fn radio_call(callsign: &Option<String>, msg: &str) -> String {
    match callsign {
        Some(cs) => format!("{cs}, {msg}"),
        None => msg.to_string(),
    }
}

/// Spell a runway identifier for the radio: "27" -> "two seven", "09L" -> "zero
/// nine left". Digit by digit, with the L/R/C suffix spoken.
pub fn spell_runway(id: &str) -> String {
    const DIGITS: [&str; 10] = [
        "zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine",
    ];
    let mut out: Vec<String> = Vec::new();
    for c in id.chars() {
        match c {
            '0'..='9' => out.push(DIGITS[c as usize - '0' as usize].to_string()),
            'L' | 'l' => out.push("left".to_string()),
            'R' | 'r' => out.push("right".to_string()),
            'C' | 'c' => out.push("center".to_string()),
            _ => {}
        }
    }
    out.join(" ")
}

#[cfg(test)]
mod position_tests {
    use super::*;

    /// THE CREW FOLLOWS "CONTACT GROUND": the handoff is read back first,
    /// on the frequency that gave it ("Gatwick Ground on one two one
    /// decimal eight zero five", a readback the controller answers with
    /// silence), and only once it has gone out is COM1 retuned to Ground.
    #[tokio::test]
    async fn the_crew_reads_back_a_contact_ground_then_retunes() {
        let _q = QUEUE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let _ = take_crew_radio();
        let xp = crate::xplane::FakeXPlane::with_reads(&[]);
        let cs = Some("Speedbird one two three".to_string());
        let drain = tokio::spawn(async {
            tokio::time::sleep(std::time::Duration::from_millis(300)).await;
            take_crew_radio()
        });
        let tuned = follow_handoff(&xp, &cs, Some(("Gatwick Ground".into(), 121_805))).await;
        let sent = drain.await.unwrap();
        assert_eq!(tuned, Some(121_805));
        let words: Vec<String> = sent.iter().map(|q| call_words(&q.call)).collect();
        assert_eq!(
            words,
            vec!["Speedbird one two three, Gatwick Ground on one two one decimal eight zero five"]
        );
        assert!(crate::tower::is_readback(&words[0]));
        assert_eq!(xp.writes_to(crate::datarefs::COM1_FREQ_KHZ), vec![121_805.0]);
        // Nothing owed: the position the call is for is tuned, nothing said.
        let xp = crate::xplane::FakeXPlane::with_reads(&[]);
        work_position(&xp, &cs, Some(124_230)).await;
        assert_eq!(xp.writes_to(crate::datarefs::COM1_FREQ_KHZ), vec![124_230.0]);
        assert!(take_crew_radio().is_empty());
    }
}
