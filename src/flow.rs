//! Checklist flows: the airframe's checklist read at the phase it belongs to,
//! in one of three flavours, off the command loop.
//!
//! The XChecklist interpreter ([`crate::checklist`]) knows what each item
//! says, how to verify it and, for the systems we own, how to act. Until this
//! module it was reached only from a spoken request and ran INLINE in the
//! command loop, listening for its own confirms. Nothing read the CLIMB,
//! CRUISE, DESCENT, BEFORE LANDING or AFTER LANDING lists when the flight
//! actually got there, and nothing could teach a pilot flying by hand.
//!
//! Three flavours of the same list:
//!
//! - **Crew** (pilot flying, copilot acting): the pilot-monitoring's flow.
//!   Items already in state are confirmed, not re-written; items a running
//!   automation owns are named as held ("the speed hold has it") rather than
//!   written under the loop (one writer per dataref); hazard steps ask for a
//!   yes/no that arrives through the command loop.
//! - **Monitor** (the copilot flying under full auto): a read-only
//!   cross-check. The mission's own laws act; the list confirms each item in
//!   silence, blackboxes every discrepancy, and speaks ONE line at the end
//!   naming the items not as listed ([`summary_line`]). Never writes. It
//!   used to recite every item: nine lines on short final at Heathrow
//!   (2026-09-17), queued behind the radio, telling the pilot nothing the
//!   laws had not already said when they set the switch.
//! - **Instruct** (the instructor): every item is a spoken instruction, then
//!   a watch for the pilot to satisfy it ("Good."), advanced by the pilot's
//!   "next" / "set" / "checked" or a timeout. Writes only when asked to
//!   ("help me with that item"), through the crew's own tool.
//!
//! Who starts a flow: the pilot ("descent checklist", "talk me through the
//! before landing checklist"), or the monitor at a phase boundary
//! ([`Tracker`]): under an automation that is flying, the Monitor flavour
//! starts at once; with a pilot flying, the copilot PROPOSES ("Climb
//! checklist when you're ready?") and "go ahead" starts the Crew flavour,
//! "talk me through it" the Instruct flavour. A proposal never acts on its
//! own (the cockpit_auto rule: an unasked action is a fight with the pilot).
//!
//! The runner is a thread like the automations', speaking through the crew
//! voice; replies (yes/no/next/skip/stop) come from the command loop via
//! [`take_reply`], so the mic is never taken away from the pilot.

use std::sync::atomic::{AtomicBool, Ordering::SeqCst};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::json;

use crate::aircraft::AircraftProfile;
use crate::checklist::{self, Checklist, Cond, Item};
use crate::config::Config;
use crate::tools::ToolCtx;
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

/// How the list is run. See the module docs.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Flavour {
    Crew,
    Monitor,
    Instruct,
}

impl Flavour {
    fn name(self) -> &'static str {
        match self {
            Flavour::Crew => "crew",
            Flavour::Monitor => "monitor",
            Flavour::Instruct => "instruct",
        }
    }
}

/// A flow is running (its thread is alive).
pub static ACTIVE: AtomicBool = AtomicBool::new(false);
static ABORT: AtomicBool = AtomicBool::new(false);

/// What the runner is waiting on from the pilot, if anything.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Waiting {
    /// A hazard confirm: yes does it, no skips it.
    Confirm,
    /// The instructor's item: next/set/checked moves on, skip skips.
    Advance,
}

/// A reply the command loop hands the runner. Closed vocabulary
/// ([`classify`]); anything outside it is not a reply and routes on as a
/// normal utterance ("flaps ten" mid-checklist is still a command).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Reply {
    Yes,
    No,
    Next,
    Skip,
    /// "wait" / "hold on" / "standby": the instructor's clock restarts.
    Hold,
    /// "help me with that" / "you do it" / "do it for me": the instructor
    /// performs the item it was waiting on, then moves on.
    DoIt,
    Abort,
}

static WAITING: Mutex<Option<Waiting>> = Mutex::new(None);
static REPLY: Mutex<Option<Reply>> = Mutex::new(None);

/// The installed lists (the airframe's), and the profile's section title
/// overrides (`checklist_flow:` in the YAML, section key -> file title).
static LISTS: Mutex<Option<Arc<Vec<Checklist>>>> = Mutex::new(None);
static OVERRIDES: Mutex<Vec<(String, String)>> = Mutex::new(Vec::new());

/// A proposed section (index into the lists) and when it was offered.
static PROPOSAL: Mutex<Option<(usize, Instant)>> = Mutex::new(None);
/// How long "Climb checklist when you're ready?" stays answerable.
pub const PROPOSAL_TTL: Duration = Duration::from_secs(120);

static DETAIL: crate::auto_status::Detail = crate::auto_status::Detail::new();

fn lock<T>(m: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

/// Install the airframe's lists: parse the profile's checklist file, remember
/// the section overrides, and retire whatever the last airframe had running
/// or proposed (process-wide facts must never outlive their airframe).
/// Returns the lists for the command loop's own matching.
pub fn install(profile: &AircraftProfile) -> Arc<Vec<Checklist>> {
    let lists = Arc::new(checklist::load(profile).unwrap_or_else(|e| {
        tracing::error!("checklist load failed: {e:#}; running without checklists");
        Vec::new()
    }));
    abort();
    *lock(&PROPOSAL) = None;
    *lock(&OVERRIDES) = profile
        .checklist_flow
        .iter()
        .map(|(k, v)| (section_key(k), v.clone()))
        .collect();
    *lock(&LISTS) = Some(lists.clone());
    lists
}

fn lists() -> Arc<Vec<Checklist>> {
    lock(&LISTS).clone().unwrap_or_else(|| Arc::new(Vec::new()))
}

/// "before_landing" / "Before Landing" -> "before landing".
fn section_key(k: &str) -> String {
    k.to_ascii_lowercase().replace('_', " ").trim().to_string()
}

/// The phase sections a flow can be triggered for, in flight order.
#[cfg(test)]
pub const SECTIONS: [&str; 5] = [
    "climb",
    "cruise",
    "descent",
    "before landing",
    "after landing",
];

/// The list for a section key, by the profile's override title first, then
/// by the section's own words against the file's titles (the same matcher a
/// spoken "climb checklist" uses, so the two can never disagree). `None`
/// when this file has no such section: nothing is proposed, nothing said.
pub fn find_section(key: &str) -> Option<usize> {
    let lists = lists();
    find_section_in(&lists, key, &lock(&OVERRIDES))
}

fn find_section_in(
    lists: &[Checklist],
    key: &str,
    overrides: &[(String, String)],
) -> Option<usize> {
    let key = section_key(key);
    if let Some((_, title)) = overrides.iter().find(|(k, _)| *k == key) {
        return lists
            .iter()
            .position(|c| c.title.eq_ignore_ascii_case(title.trim()));
    }
    let want = checklist::match_request(&format!("{key} checklist"), lists)?;
    lists.iter().position(|c| c.id == want.id)
}

/// A list by a spoken title ("before landing", "the descent checklist"):
/// the exact title, or the one candidate whose title carries every word.
/// `Err` carries the candidates (all titles when nothing matched) for the
/// "which checklist?" decline.
pub fn find_titled(lists: &[Checklist], request: &str) -> Result<usize, Vec<String>> {
    let request = request
        .to_ascii_lowercase()
        .trim()
        .trim_start_matches("the ")
        .trim_end_matches(" checklist")
        .to_string();
    let query: Vec<_> = request
        .split(|c: char| !c.is_ascii_alphanumeric())
        .filter(|s| !s.is_empty())
        .collect();
    if query.is_empty() {
        return Err(lists.iter().map(|c| c.title.clone()).collect());
    }
    let candidates: Vec<usize> = lists
        .iter()
        .enumerate()
        .filter(|(_, cl)| {
            let title = cl.title.to_ascii_lowercase();
            let title: Vec<_> = title.split(|c: char| !c.is_ascii_alphanumeric()).collect();
            query.iter().all(|w| title.contains(w))
        })
        .map(|(i, _)| i)
        .collect();
    if let Some(i) = candidates
        .iter()
        .find(|i| lists[**i].title.eq_ignore_ascii_case(&request))
    {
        return Ok(*i);
    }
    match candidates.as_slice() {
        [one] => Ok(*one),
        [] => Err(lists.iter().map(|c| c.title.clone()).collect()),
        many => Err(many.iter().map(|i| lists[*i].title.clone()).collect()),
    }
}

// ---- the phase tracker -----------------------------------------------------

/// What the tracker needs from a monitor tick.
pub struct Sample {
    pub on_ground: bool,
    pub agl_ft: f64,
    pub vsi_fpm: f64,
    pub groundspeed_kts: f64,
}

/// Climb list once the climb is established, not at liftoff, and AFTER the
/// exterior-lights rule has put the landing light out at 1,000 ft: at 800 the
/// cross-check read "landing lights off, not set" every departure.
pub const CLIMB_AGL_FT: f64 = 1200.0;
/// ...and held this long. EVERY other boundary here is sustained; the climb
/// was the one that fired on a single tick, and it fired straight into the
/// automation's own thousand-foot actions. EGLF 2026-09-22: the list ran at
/// 1,200 ft and reported "flaps, up on schedule" at 0.125 and "taxi light,
/// off" still lit - the flaps finished retracting four seconds later and
/// `lights_climb` put both lamps out six seconds later at 1,379 ft. The
/// mission's laws act and the list audits, so the list has to look after
/// they have acted, not through them.
pub const CLIMB_SECS: f64 = 15.0;
/// Cruise list: level this long, this high (a pattern at 1,000 ft is not a cruise).
pub const CRUISE_LEVEL_SECS: f64 = 30.0;
pub const CRUISE_MIN_AGL_FT: f64 = 2000.0;
/// Descent list: down this long from a flight that got at least to cruise height.
pub const DESCENT_SECS: f64 = 20.0;
/// Before-landing list: descending below this, sustained.
pub const BEFORE_LANDING_AGL_FT: f64 = 1500.0;
pub const BEFORE_LANDING_SECS: f64 = 10.0;
/// After-landing list: rolled out below this speed for this long.
pub const AFTER_LANDING_KTS: f64 = 30.0;
pub const AFTER_LANDING_SECS: f64 = 10.0;

/// Phase-boundary detector, one per monitor loop. Each section fires ONCE per
/// flight (a flight starts at the takeoff roll and ends with the after-landing
/// list), on sustained conditions rather than a single tick, so a gust does
/// not read as a descent. Pure: the monitor feeds it, it names a section.
#[derive(Default)]
pub struct Tracker {
    fired: Vec<&'static str>,
    airborne_seen: bool,
    peak_agl_ft: f64,
    climbing_since: Option<Instant>,
    level_since: Option<Instant>,
    descending_since: Option<Instant>,
    low_descent_since: Option<Instant>,
    stopped_since: Option<Instant>,
}

impl Tracker {
    fn fired(&self, s: &str) -> bool {
        self.fired.contains(&s)
    }

    fn fire(&mut self, s: &'static str) -> Option<&'static str> {
        self.fired.push(s);
        Some(s)
    }

    /// Feed one tick; the section whose boundary was just crossed, if any.
    pub fn tick(&mut self, s: &Sample, now: Instant) -> Option<&'static str> {
        let held = |since: &mut Option<Instant>, on: bool, secs: f64| -> bool {
            if !on {
                *since = None;
                return false;
            }
            let t = *since.get_or_insert(now);
            now.duration_since(t).as_secs_f64() >= secs
        };
        if s.on_ground {
            self.climbing_since = None;
            self.level_since = None;
            self.descending_since = None;
            self.low_descent_since = None;
            if !self.airborne_seen {
                // A new takeoff roll arms every section for the flight ahead.
                if s.groundspeed_kts > 35.0 {
                    self.fired.clear();
                }
                self.stopped_since = None;
                return None;
            }
            let stopped = held(
                &mut self.stopped_since,
                s.groundspeed_kts.abs() < AFTER_LANDING_KTS,
                AFTER_LANDING_SECS,
            );
            if stopped && !self.fired("after landing") {
                // The flight is over: the next one starts from scratch.
                self.airborne_seen = false;
                self.peak_agl_ft = 0.0;
                self.stopped_since = None;
                self.fired.clear();
                return self.fire("after landing");
            }
            return None;
        }
        self.airborne_seen = true;
        self.stopped_since = None;
        self.peak_agl_ft = self.peak_agl_ft.max(s.agl_ft);
        let climbing = held(
            &mut self.climbing_since,
            s.vsi_fpm > 300.0 && s.agl_ft >= CLIMB_AGL_FT,
            CLIMB_SECS,
        );
        if climbing && !self.fired("climb") {
            return self.fire("climb");
        }
        let level = held(
            &mut self.level_since,
            s.vsi_fpm.abs() < 300.0 && s.agl_ft >= CRUISE_MIN_AGL_FT,
            CRUISE_LEVEL_SECS,
        );
        if level && !self.fired("cruise") {
            return self.fire("cruise");
        }
        let descending = held(
            &mut self.descending_since,
            s.vsi_fpm < -300.0 && self.peak_agl_ft >= CRUISE_MIN_AGL_FT,
            DESCENT_SECS,
        );
        if descending && !self.fired("descent") {
            return self.fire("descent");
        }
        let low = held(
            &mut self.low_descent_since,
            s.agl_ft < BEFORE_LANDING_AGL_FT && s.vsi_fpm < -100.0 && self.peak_agl_ft >= 1000.0,
            BEFORE_LANDING_SECS,
        );
        if low && !self.fired("before landing") {
            return self.fire("before landing");
        }
        None
    }
}

/// Is an automation flying the aircraft (so the copilot is the pilot flying
/// and the list is its own cross-check)?
fn automation_flying() -> bool {
    use std::sync::atomic::Ordering::Relaxed;
    crate::full_auto::ACTIVE.load(Relaxed)
        || crate::procedure::ACTIVE.load(Relaxed)
        || crate::circuit::ACTIVE.load(Relaxed)
        || crate::autoland::ACTIVE.load(Relaxed)
}

/// The monitor crossed a phase boundary: run the section as the flying
/// copilot's cross-check, or propose it to the pilot flying. Nothing when
/// the file has no such section, or a flow is already running.
pub fn phase_boundary(section: &str, config: &Config, voice: &Voice) {
    let Some(idx) = find_section(section) else {
        crate::blackbox::record(
            "checklist",
            json!({ "event": "no_section", "section": section }),
        );
        return;
    };
    if ACTIVE.load(SeqCst) {
        return;
    }
    let title = lists()
        .get(idx)
        .map(|c| c.title.clone())
        .unwrap_or_default();
    if automation_flying() {
        start(idx, Flavour::Monitor, config.clone(), voice.clone());
        return;
    }
    // The instructor's full-flight syllabus runs the list at its phase
    // itself, read-and-watch, no proposal: the student is flying it.
    if crate::instructor::syllabus_active() {
        start(idx, Flavour::Instruct, config.clone(), voice.clone());
        return;
    }
    *lock(&PROPOSAL) = Some((idx, Instant::now()));
    crate::blackbox::record(
        "checklist",
        json!({ "event": "proposed", "title": title, "section": section }),
    );
    voice.alert(
        Priority::Advisory,
        format!(
            "{} checklist when you're ready?",
            cap_first(&title.to_lowercase())
        ),
    );
}

/// The unexpired proposal, if any.
pub fn proposal() -> Option<usize> {
    let mut p = lock(&PROPOSAL);
    match *p {
        Some((idx, at)) if at.elapsed() < PROPOSAL_TTL => Some(idx),
        Some(_) => {
            *p = None;
            None
        }
        None => None,
    }
}

pub fn decline_proposal() {
    *lock(&PROPOSAL) = None;
    crate::blackbox::record("checklist", json!({ "event": "declined" }));
}

/// How the pilot answered a proposal.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ProposalReply {
    Run,
    Instruct,
    Decline,
}

/// Classify an utterance as an answer to the standing proposal. Closed
/// vocabulary: "go ahead", "yes", "run it", "ready" run it; "talk me through
/// it" teaches it; "no", "not now", "later" decline. Anything else is not an
/// answer and routes on ("flaps ten" after a proposal is still flaps ten).
pub fn proposal_reply(text: &str) -> Option<ProposalReply> {
    let words = words(text);
    if words.is_empty() || words.len() > 8 {
        return None;
    }
    if instruct_words(&words) {
        return Some(ProposalReply::Instruct);
    }
    const NO: &[&str] = &[
        "no", "negative", "nope", "not", "later", "standby", "stand", "wait", "hold",
    ];
    const NO_OK: &[&str] = &[
        "now",
        "thanks",
        "thank",
        "you",
        "yet",
        "by",
        "on",
        "off",
        "it",
        "the",
        "checklist",
        "please",
        "a",
        "minute",
        "moment",
        "sec",
    ];
    if words.iter().any(|w| NO.contains(&w.as_str()))
        && words
            .iter()
            .all(|w| NO.contains(&w.as_str()) || NO_OK.contains(&w.as_str()))
    {
        return Some(ProposalReply::Decline);
    }
    const RUN: &[&str] = &[
        "yes",
        "yeah",
        "yep",
        "affirm",
        "affirmative",
        "go",
        "ahead",
        "run",
        "read",
        "do",
        "ready",
        "proceed",
        "please",
        "okay",
        "ok",
        "roger",
        "sure",
        "lets",
        "start",
    ];
    const RUN_OK: &[&str] = &[
        "it",
        "the",
        "checklist",
        "check",
        "list",
        "now",
        "when",
        "youre",
        "you",
        "are",
        "im",
        "i",
        "am",
    ];
    if words.iter().any(|w| RUN.contains(&w.as_str()))
        && words
            .iter()
            .all(|w| RUN.contains(&w.as_str()) || RUN_OK.contains(&w.as_str()))
    {
        return Some(ProposalReply::Run);
    }
    None
}

/// "talk / walk / guide / coach / instruct me through it", "teach me".
fn instruct_words(words: &[String]) -> bool {
    const VERB: &[&str] = &[
        "talk", "walk", "guide", "coach", "instruct", "teach", "tell",
    ];
    const OK: &[&str] = &[
        "me",
        "us",
        "through",
        "it",
        "the",
        "checklist",
        "check",
        "list",
        "please",
        "what",
        "to",
        "do",
        "step",
        "by",
        "this",
        "one",
    ];
    words.iter().any(|w| VERB.contains(&w.as_str()))
        && words
            .iter()
            .all(|w| VERB.contains(&w.as_str()) || OK.contains(&w.as_str()))
}

/// Is the utterance a request to be TAUGHT a list ("talk me through the
/// descent checklist")? Returns the request words the title is found by.
pub fn instruct_request(text: &str) -> Option<String> {
    let t = text.to_ascii_lowercase();
    let t = t.trim().trim_end_matches(['.', '?', '!']);
    let rest = [
        "talk me through ",
        "walk me through ",
        "guide me through ",
        "coach me through ",
        "instruct me through ",
        "teach me ",
        "tell me what to do for ",
        "tell me what to do on ",
    ]
    .iter()
    .find_map(|p| t.strip_prefix(p))?;
    let rest = rest.trim_start_matches("the ");
    let rest = rest.strip_suffix(" checklist")?;
    (!rest.is_empty()).then(|| rest.to_string())
}

fn words(text: &str) -> Vec<String> {
    text.to_ascii_lowercase()
        .split(|c: char| !c.is_ascii_alphanumeric() && c != '\'')
        .map(|w| w.replace('\'', ""))
        .filter(|w| !w.is_empty())
        .collect()
}

/// Classify a reply to a RUNNING flow. Closed vocabulary; `None` means the
/// words are not for the checklist.
pub fn classify(text: &str) -> Option<Reply> {
    let words = words(text);
    if words.is_empty() || words.len() > 8 {
        return None;
    }
    let all_in = |set: &[&str], extra: &[&str]| {
        words
            .iter()
            .all(|w| set.contains(&w.as_str()) || extra.contains(&w.as_str()))
    };
    let any_in = |set: &[&str]| words.iter().any(|w| set.contains(&w.as_str()));
    const FILLER: &[&str] = &[
        "the",
        "that",
        "it",
        "this",
        "checklist",
        "check",
        "list",
        "please",
        "item",
        "and",
        "on",
        "a",
        "one",
    ];
    const ABORT: &[&str] = &[
        "stop",
        "abort",
        "cancel",
        "belay",
        "disregard",
        "quit",
        "halt",
    ];
    if any_in(ABORT) && all_in(ABORT, FILLER) {
        return Some(Reply::Abort);
    }
    // Asked to DO the item: "can you help me with that item", "you do it",
    // "do it for me", "help me with that". Before yes/next, which "do it"
    // alone still is.
    const DOIT: &[&str] = &[
        "help", "do", "handle", "set", "take", "care", "of", "for", "me", "you", "can", "could",
        "would", "with", "yourself", "instead",
    ];
    let asks_help = any_in(&["help"])
        || (any_in(&["you"]) && any_in(&["do", "handle", "set", "take"]))
        || (any_in(&["do", "handle", "set"]) && any_in(&["for"]) && any_in(&["me"]));
    if asks_help && all_in(DOIT, FILLER) {
        return Some(Reply::DoIt);
    }
    const HOLD: &[&str] = &[
        "wait", "hold", "standby", "stand", "by", "moment", "minute", "sec", "second",
    ];
    if any_in(&["wait", "hold", "standby", "stand"]) && all_in(HOLD, FILLER) {
        return Some(Reply::Hold);
    }
    const SKIP: &[&str] = &["skip", "pass", "later"];
    if any_in(SKIP) && all_in(SKIP, FILLER) {
        return Some(Reply::Skip);
    }
    const NO: &[&str] = &["no", "negative", "nope", "not", "dont"];
    if any_in(NO) && all_in(NO, FILLER) {
        return Some(Reply::No);
    }
    const YES: &[&str] = &[
        "yes",
        "yeah",
        "yep",
        "affirm",
        "affirmative",
        "confirm",
        "confirmed",
        "go",
        "ahead",
        "do",
        "proceed",
        "roger",
        "okay",
        "ok",
    ];
    if any_in(YES) && all_in(YES, FILLER) {
        return Some(Reply::Yes);
    }
    const NEXT: &[&str] = &[
        "next",
        "done",
        "checked",
        "set",
        "complete",
        "completed",
        "continue",
        "go",
        "on",
        "moving",
        "got",
        "ready",
        "roger",
        "okay",
        "ok",
        "in",
        "off",
        "down",
        "up",
        "full",
        "rich",
        "both",
    ];
    if any_in(&[
        "next",
        "done",
        "checked",
        "set",
        "complete",
        "completed",
        "continue",
        "moving",
        "got",
        "ready",
    ]) && all_in(NEXT, FILLER)
    {
        return Some(Reply::Next);
    }
    None
}

/// The command loop's door: if a flow is running and these words are a
/// reply it is waiting for, hand them over and say which. `None` routes the
/// utterance on as usual. Abort is always taken while a flow runs; yes/no
/// only at a confirm; next/skip/hold only at an instructor's item (a "yes"
/// there advances, too).
pub fn take_reply(text: &str) -> Option<Reply> {
    if !ACTIVE.load(SeqCst) {
        return None;
    }
    let reply = classify(text)?;
    let waiting = *lock(&WAITING);
    let taken = matches!(
        (reply, waiting),
        (Reply::Abort, _)
            | (Reply::Yes | Reply::No, Some(Waiting::Confirm))
            | (
                Reply::Yes | Reply::Next | Reply::Skip | Reply::No | Reply::Hold | Reply::DoIt,
                Some(Waiting::Advance)
            )
    );
    if !taken {
        return None;
    }
    if reply == Reply::Abort {
        abort();
    } else {
        *lock(&REPLY) = Some(reply);
    }
    Some(reply)
}

/// Stop the running flow where it is (its summary says so).
pub fn abort() {
    if ACTIVE.load(SeqCst) {
        ABORT.store(true, SeqCst);
    }
}

/// The panel line: "checklist: climb, 3 of 8".
pub fn status() -> Option<String> {
    ACTIVE.load(SeqCst).then(|| DETAIL.line("checklist"))
}

/// Start a flow on the installed list `idx`. `false` if one is already
/// running, the index is stale, or there is no crew aboard (ATC-only).
pub fn start(idx: usize, flavour: Flavour, config: Config, voice: Voice) -> bool {
    if !crate::mode::copilot_enabled() {
        return false;
    }
    // An instructor never writes: with the crew's hands off, the Crew
    // flavour IS the Instruct flavour, decided here at the one door every
    // caller (a spoken title, a proposal's "go ahead") walks through.
    let flavour = if matches!(flavour, Flavour::Crew) && !crate::mode::crew_actuates() {
        Flavour::Instruct
    } else {
        flavour
    };
    let lists = lists();
    if idx >= lists.len() {
        return false;
    }
    if ACTIVE.swap(true, SeqCst) {
        return false;
    }
    ABORT.store(false, SeqCst);
    *lock(&REPLY) = None;
    *lock(&WAITING) = None;
    *lock(&PROPOSAL) = None;
    std::thread::Builder::new()
        .name("checklist-flow".into())
        .spawn(move || {
            if let Ok(rt) = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
            {
                let xp = XPlaneClient::new(config.xplane_base_url.clone());
                rt.block_on(run(&lists[idx], flavour, &xp, &config, &voice));
            }
            DETAIL.clear();
            *lock(&WAITING) = None;
            ACTIVE.store(false, SeqCst);
        })
        .map(|_| true)
        .unwrap_or_else(|_| {
            ACTIVE.store(false, SeqCst);
            false
        })
}

// ---- per-item decisions (pure) ------------------------------------------------

/// What the runner does with one item, decided from the flavour, the live
/// value behind the item's dataref (`None` = no dataref or unreadable) and
/// whether a running automation owns the system.
#[derive(Debug, PartialEq)]
pub enum Step {
    /// Speak the line, move on.
    Say(String),
    /// Say nothing, move on (Monitor skips narration).
    Skip,
    /// Crew: perform the item through its tool (or the verify path's write).
    Act,
    /// The system is a running loop's: say who has it, move on.
    Held(String),
    /// Monitor: the item is not in state and nobody owns it. Say it,
    /// blackbox it.
    Discrepancy(String),
    /// Instruct: speak the instruction, then watch for the pilot to do it.
    Wait(String),
}

/// "the speed hold" when a running loop owns the item's dataref: the
/// governor the throttle, the mixture manager the mixture, autoland the
/// flaps, the flying loops the autopilot. A list must never write under a
/// loop (one writer per dataref), and an instructor should not ask the
/// pilot to fight one either.
pub fn held_by_automation(dataref: &str) -> Option<&'static str> {
    use std::sync::atomic::Ordering::Relaxed;
    let flying = crate::autoland::ACTIVE.load(Relaxed)
        || crate::circuit::ACTIVE.load(Relaxed)
        || crate::procedure::ACTIVE.load(Relaxed);
    if dataref.contains("throttle") && crate::tools::speedhold::ACTIVE.load(Relaxed) {
        return Some("the speed hold");
    }
    // The circuit's own throttle law holds the pattern speed; the climb
    // list's "throttle, full" is not a discrepancy under it (LXGB 2026-09-14).
    if dataref.contains("throttle") && crate::circuit::ACTIVE.load(Relaxed) {
        return Some("the circuit");
    }
    if dataref.contains("mixture") && crate::tools::engine::MIX_MANAGED.load(Relaxed) {
        return Some("the mixture manager");
    }
    if dataref.contains("autopilot") && flying {
        return Some(if crate::autoland::ACTIVE.load(Relaxed) {
            "autoland"
        } else {
            "the follower"
        });
    }
    if dataref.contains("flap") && crate::autoland::ACTIVE.load(Relaxed) {
        return Some("autoland");
    }
    // Full auto works the signs by height (off through ten thousand, on
    // again through it on the way down). The descent list runs at the top of
    // descent, seven thousand feet above that, and "seat belt signs, on" was
    // spoken as a discrepancy on a flight that was exactly as it should be
    // (EGPH-EGTE, 2026-09-21).
    if crate::full_auto::ACTIVE.load(Relaxed) && crate::tools::signs::is_signs_dataref(dataref) {
        return Some("the signs schedule");
    }
    None
}

/// Decide the step for `item`. `value` is the live read of its dataref.
pub fn plan(flavour: Flavour, item: &Item, value: Option<f64>, held: Option<&str>) -> Step {
    match item {
        Item::Narrate(t) => match flavour {
            Flavour::Monitor => Step::Skip,
            _ => Step::Say(t.clone()),
        },
        Item::CallOut(t) => match flavour {
            Flavour::Instruct => Step::Wait(t.clone()),
            Flavour::Monitor => Step::Skip,
            Flavour::Crew => Step::Say(t.clone()),
        },
        Item::Act { say, cond, .. } => stateful(flavour, say, cond, value, held, true),
        Item::Verify {
            say,
            cond,
            settable,
            ..
        } => stateful(flavour, say, cond, value, held, *settable),
    }
}

fn stateful(
    flavour: Flavour,
    say: &str,
    cond: &Cond,
    value: Option<f64>,
    held: Option<&str>,
    settable: bool,
) -> Step {
    let satisfied = value.is_some_and(|v| cond.eval(v));
    if satisfied {
        return match flavour {
            Flavour::Instruct => Step::Say(format!("{say} Already done.")),
            // The monitor confirms in silence: the mission's laws set the
            // item and said so when they did. Reciting nine satisfied items
            // on short final (EGLL 27L, 2026-09-17) told the pilot nothing
            // and queued behind the radio for half a minute.
            Flavour::Monitor => Step::Skip,
            Flavour::Crew => Step::Say(say.to_string()),
        };
    }
    if let Some(owner) = held {
        return Step::Held(format!("{say} {} has it.", cap_first(owner)));
    }
    match flavour {
        Flavour::Crew if settable => Step::Act,
        Flavour::Crew => Step::Say(say.to_string()),
        // Unreadable is not a discrepancy: the monitor blackboxes it and
        // moves on rather than accuse.
        Flavour::Monitor if value.is_none() => Step::Skip,
        Flavour::Monitor => Step::Discrepancy(format!("{say} Not set.")),
        Flavour::Instruct => Step::Wait(say.to_string()),
    }
}

fn cap_first(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
        None => String::new(),
    }
}

// ---- the runner --------------------------------------------------------------

/// How long the instructor waits for an item before moving on.
pub const INSTRUCT_WAIT_SECS: f64 = 45.0;
/// How long a hazard confirm waits for its yes/no.
pub const CONFIRM_WAIT_SECS: f64 = 20.0;

/// Where a wait ended.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Waited {
    Satisfied,
    Advanced,
    Skipped,
    /// The pilot asked the instructor to perform this item.
    DoIt,
    TimedOut,
    Aborted,
}

fn take_pending_reply() -> Option<Reply> {
    lock(&REPLY).take()
}

/// Watch for the pilot to satisfy `cond` on `dataref` (when there is one),
/// or to say next/skip; the clock restarts on "wait".
async fn wait_for(
    xp: &XPlaneClient,
    dataref: Option<(&str, Option<usize>, &Cond)>,
    secs: f64,
) -> Waited {
    *lock(&WAITING) = Some(Waiting::Advance);
    let mut deadline = Instant::now() + Duration::from_secs_f64(secs);
    let out = loop {
        if ABORT.load(SeqCst) {
            break Waited::Aborted;
        }
        match take_pending_reply() {
            Some(Reply::Next | Reply::Yes) => break Waited::Advanced,
            Some(Reply::Skip | Reply::No) => break Waited::Skipped,
            Some(Reply::DoIt) => break Waited::DoIt,
            Some(Reply::Hold) => deadline = Instant::now() + Duration::from_secs_f64(secs),
            _ => {}
        }
        if let Some((d, i, cond)) = dataref {
            if checklist::read_dataref(xp, d, i)
                .await
                .is_some_and(|v| cond.eval(v))
            {
                break Waited::Satisfied;
            }
        }
        if Instant::now() >= deadline {
            break Waited::TimedOut;
        }
        tokio::time::sleep(Duration::from_millis(500)).await;
    };
    *lock(&WAITING) = None;
    out
}

/// Ask a hazard confirm through the command loop: yes, no (skip), or abort.
async fn confirm(voice: &Voice, hazard: &str) -> Waited {
    voice.say_crew(Priority::Readback, format!("Confirm, {hazard}?"));
    *lock(&WAITING) = Some(Waiting::Confirm);
    let deadline = Instant::now() + Duration::from_secs_f64(CONFIRM_WAIT_SECS);
    let out = loop {
        if ABORT.load(SeqCst) {
            break Waited::Aborted;
        }
        match take_pending_reply() {
            Some(Reply::Yes) => break Waited::Advanced,
            Some(Reply::No) => break Waited::Skipped,
            _ => {}
        }
        if Instant::now() >= deadline {
            break Waited::TimedOut;
        }
        tokio::time::sleep(Duration::from_millis(200)).await;
    };
    *lock(&WAITING) = None;
    out
}

fn item_cond(item: &Item) -> Option<(&str, Option<usize>, &Cond)> {
    match item {
        Item::Act {
            dataref,
            index,
            cond,
            ..
        }
        | Item::Verify {
            dataref,
            index,
            cond,
            ..
        } => Some((dataref.as_str(), *index, cond)),
        _ => None,
    }
}

fn item_line(item: &Item) -> &str {
    match item {
        Item::Narrate(t) | Item::CallOut(t) => t,
        Item::Act { say, .. } | Item::Verify { say, .. } => say,
    }
}

async fn run(cl: &Checklist, flavour: Flavour, xp: &XPlaneClient, config: &Config, voice: &Voice) {
    let title_lc = cl.title.to_lowercase();
    let priority = match flavour {
        Flavour::Monitor => Priority::Advisory,
        _ => Priority::Readback,
    };
    let say = |line: &str| {
        tracing::info!(target: "xvoicelayer::flow", spoke = %line, "checklist");
        voice.say_crew(priority, line.to_string());
    };
    crate::blackbox::record(
        "checklist",
        json!({ "event": "start", "title": cl.title, "flavour": flavour.name(), "items": cl.items.len() }),
    );
    // The monitor says ONE line per list, at the end (see `summary_line`):
    // an opening followed two seconds later by "complete" is two lines
    // for nothing.
    match flavour {
        Flavour::Crew => say(&format!("Running the {} checklist.", cl.title)),
        Flavour::Monitor => {}
        Flavour::Instruct => say(&format!(
            "{} checklist. I'll call each item, you do it and tell me when it's done.",
            cap_first(&title_lc)
        )),
    }
    let ctx = ToolCtx {
        xplane: xp,
        config,
        voice: Some(voice),
        status: None,
        reminders: None,
    };
    let n = cl.items.len();
    let mut failed = 0usize;
    let mut discrepancies = 0usize;
    // The monitor's discrepancies, named in its one summary line.
    let mut not_set: Vec<String> = Vec::new();
    let mut aborted = false;
    for (i, item) in cl.items.iter().enumerate() {
        if ABORT.load(SeqCst) {
            aborted = true;
            break;
        }
        DETAIL.set(format!("{title_lc}, {} of {n}", i + 1));
        let cond = item_cond(item);
        let value = match cond {
            Some((d, idx, _)) => checklist::read_dataref(xp, d, idx).await,
            None => None,
        };
        let held = cond.and_then(|(d, _, _)| held_by_automation(d));
        match plan(flavour, item, value, held) {
            Step::Skip => {}
            Step::Say(line) => say(&line),
            Step::Held(line) => {
                crate::blackbox::record(
                    "checklist",
                    json!({ "event": "held", "title": cl.title, "item": item_line(item), "by": held }),
                );
                // Under full auto the owner IS the mission: nothing to say.
                if flavour != Flavour::Monitor {
                    say(&line);
                }
            }
            Step::Discrepancy(line) => {
                discrepancies += 1;
                crate::blackbox::record(
                    "checklist",
                    json!({ "event": "discrepancy", "title": cl.title, "item": item_line(item), "value": value }),
                );
                if flavour == Flavour::Monitor {
                    not_set.push(item_line(item).to_string());
                } else {
                    say(&line);
                }
            }
            Step::Wait(line) => {
                say(&line);
                match wait_for(xp, cond, INSTRUCT_WAIT_SECS).await {
                    Waited::Satisfied => say("Good."),
                    Waited::Advanced => {}
                    Waited::Skipped => say("Skipped."),
                    // "Help me with that item": the one time the instructor
                    // touches the aircraft, asked to, through the same tool
                    // the crew flavour would use. A call-out item has no
                    // switch we can reach.
                    Waited::DoIt => {
                        crate::blackbox::record(
                            "checklist",
                            json!({ "event": "did_for_pilot", "title": cl.title, "item": item_line(item) }),
                        );
                        match act(cl, item, &ctx, xp, voice).await {
                            Some(Acted::Ok) => say("Done."),
                            Some(Acted::Failed(what)) => say(what),
                            Some(Acted::Skipped) => say("Skipped."),
                            Some(Acted::Aborted) => {
                                aborted = true;
                                break;
                            }
                            None => say("That one's yours, there's no switch I can reach for it."),
                        }
                    }
                    Waited::TimedOut => {
                        discrepancies += 1;
                        crate::blackbox::record(
                            "checklist",
                            json!({ "event": "not_done", "title": cl.title, "item": item_line(item) }),
                        );
                        say("Still not done. Moving on.");
                    }
                    Waited::Aborted => {
                        aborted = true;
                        break;
                    }
                }
            }
            Step::Act => {
                let line = item_line(item);
                if let Some(msg) = act(cl, item, &ctx, xp, voice).await {
                    match msg {
                        Acted::Ok => say(line),
                        Acted::Skipped => say(&format!("{line} Skipped, your call.")),
                        Acted::Failed(what) => {
                            failed += 1;
                            say(&format!("{line} {what}"));
                        }
                        Acted::Aborted => {
                            aborted = true;
                            break;
                        }
                    }
                }
            }
        }
    }
    let summary = summary_line(&cl.title, flavour, aborted, failed, discrepancies, &not_set);
    crate::blackbox::record(
        "checklist",
        json!({
            "event": if aborted { "abort" } else { "complete" },
            "title": cl.title, "flavour": flavour.name(),
            "failed": failed, "discrepancies": discrepancies,
        }),
    );
    say(&summary);
}

/// The list's closing line. The monitor's is the ONLY line it speaks for a
/// list, so it carries the discrepancies by name ("Climb checklist complete,
/// 2 items not as listed: flaps, up on schedule; altimeters, standard at
/// transition."); the other flavours said each item as they went.
pub fn summary_line(
    title: &str,
    flavour: Flavour,
    aborted: bool,
    failed: usize,
    discrepancies: usize,
    not_set: &[String],
) -> String {
    let plural = |n: usize| if n == 1 { "" } else { "s" };
    if aborted {
        return format!("{title} checklist stopped where we were.");
    }
    if failed == 0 && discrepancies == 0 {
        return format!("{title} checklist complete.");
    }
    if flavour == Flavour::Crew {
        return format!(
            "{title} checklist complete, {failed} item{} didn't take, check the panel.",
            plural(failed)
        );
    }
    let mut line = format!(
        "{title} checklist complete, {discrepancies} item{} not as listed",
        plural(discrepancies)
    );
    if not_set.is_empty() {
        line.push('.');
    } else {
        let names: Vec<String> = not_set
            .iter()
            .map(|s| {
                let s = s.trim_end_matches('.');
                let mut c = s.chars();
                match c.next() {
                    Some(f) => f.to_lowercase().collect::<String>() + c.as_str(),
                    None => String::new(),
                }
            })
            .collect();
        line.push_str(": ");
        line.push_str(&names.join("; "));
        line.push('.');
    }
    line
}

enum Acted {
    Ok,
    Skipped,
    Failed(&'static str),
    Aborted,
}

/// Crew flavour: perform one item, confirming a hazard step first through
/// the command loop. `None` for an item that is not actionable.
async fn act(
    cl: &Checklist,
    item: &Item,
    ctx: &ToolCtx<'_>,
    xp: &XPlaneClient,
    voice: &Voice,
) -> Option<Acted> {
    match item {
        Item::Act {
            tool,
            args,
            confirm: hazard,
            ..
        } => {
            // Mixture-to-cutoff only stops a RUNNING engine; pre-start it is
            // the normal lean, so the prompt is skipped and the item done.
            let ask = match hazard {
                Some(h) if *h == checklist::CONFIRM_MIXTURE_CUTOFF => {
                    checklist::engine_running(xp).await
                }
                other => other.is_some(),
            };
            if ask {
                match confirm(voice, hazard.expect("ask implies a hazard")).await {
                    Waited::Advanced => {}
                    Waited::Aborted => return Some(Acted::Aborted),
                    _ => return Some(Acted::Skipped),
                }
            }
            let ok = checklist::run_step(
                tool,
                args,
                &cl.lights,
                &cl.flaps,
                &cl.electrical,
                &cl.fuel_selector,
                cl.engines,
                cl.magnetos_per_engine,
                cl.park_brake_valve_set,
                cl.pitot_heat_dataref.clone(),
                ctx,
            )
            .await
            .is_ok();
            Some(if ok {
                Acted::Ok
            } else {
                Acted::Failed("Didn't take, check it.")
            })
        }
        // The verify path's write: the file's own target, then a re-read.
        Item::Verify {
            dataref,
            index,
            cond,
            settable: true,
            ..
        } => {
            let target = cond.target();
            let wrote = match index {
                Some(i) => xp.set_float_array_elem(dataref, *i, target).await.is_ok(),
                None => xp.set_float(dataref, target).await.is_ok(),
            };
            let now = checklist::read_dataref(xp, dataref, *index).await;
            Some(if wrote && now.is_some_and(|v| cond.eval(v)) {
                Acted::Ok
            } else {
                Acted::Failed("Couldn't set it, check it.")
            })
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p172() -> AircraftProfile {
        crate::aircraft::builtin("cessna_172sp").unwrap()
    }

    fn lists172() -> Vec<Checklist> {
        checklist::load(&p172()).expect("172 checklist file")
    }

    fn sample(on_ground: bool, agl: f64, vsi: f64, gs: f64) -> Sample {
        Sample {
            on_ground,
            agl_ft: agl,
            vsi_fpm: vsi,
            groundspeed_kts: gs,
        }
    }

    #[test]
    fn every_phase_section_is_found_in_the_172_file() {
        let lists = lists172();
        let titles: Vec<&str> = SECTIONS
            .iter()
            .map(|s| {
                lists[find_section_in(&lists, s, &[]).unwrap_or_else(|| panic!("{s}"))]
                    .title
                    .as_str()
            })
            .collect();
        assert_eq!(
            titles,
            [
                "Climb",
                "Cruise",
                "Descent",
                "Before landing",
                "After landing"
            ]
        );
        // A profile override names the file's title outright.
        let over = vec![("before landing".to_string(), "NORMAL LANDING".to_string())];
        let i = find_section_in(&lists, "before_landing", &over).unwrap();
        assert_eq!(lists[i].title, "Normal landing");
        // A section the file lacks is None, never a guess.
        assert_eq!(find_section_in(&lists, "holding", &[]), None);
    }

    /// The stock 737's list (2026-09-17) names its sections by the phase
    /// keys outright: every phase the tracker can propose resolves, and so
    /// do the syllabus's ramp lists, with no profile override.
    #[test]
    fn every_phase_section_is_found_in_the_737_file() {
        let p = crate::aircraft::load("boeing_737_800").expect("737 profile");
        let lists = crate::checklist::load(&p).expect("737 checklist parses");
        let ramp = [
            "preflight",
            "before start",
            "engine start",
            "before taxi",
            "before takeoff",
            "shutdown",
        ];
        for key in SECTIONS.iter().chain(ramp.iter()) {
            assert!(
                find_section_in(&lists, key, &[]).is_some(),
                "737 file lacks {key}"
            );
        }
    }

    /// The stock A330's list (2026-09-20), same shape: the tracker's phases
    /// and the syllabus's ramp lists all resolve, and every conditioned item
    /// reads a dataref the aircraft owns or leaves alone (the map turns the
    /// ones with a tool into acts, the rest stay verifies).
    #[test]
    fn every_phase_section_is_found_in_the_a330_file() {
        let p = crate::aircraft::load("airbus_a330_300").expect("A330 profile");
        let lists = crate::checklist::load(&p).expect("A330 checklist parses");
        let ramp = [
            "preflight",
            "before start",
            "engine start",
            "before taxi",
            "before takeoff",
            "shutdown",
        ];
        for key in SECTIONS.iter().chain(ramp.iter()) {
            assert!(
                find_section_in(&lists, key, &[]).is_some(),
                "A330 file lacks {key}"
            );
        }
        // The before-taxi list's taxi light is an ACT through the map, not a
        // bare verify: the A330's nose light is a three-position switch on
        // one element of `landing_lights_switch`, so the map entry carries
        // the index. (The transponder stays a verify: its mode is a laminar
        // knob whose values the shared stock decoding does not speak.)
        let bt = &lists[find_section_in(&lists, "before taxi", &[]).unwrap()];
        let acts: Vec<&str> = bt
            .items
            .iter()
            .filter_map(|i| match i {
                crate::checklist::Item::Act { tool, .. } => Some(*tool),
                _ => None,
            })
            .collect();
        assert!(
            acts.contains(&"set_lights"),
            "taxi light is an act: {acts:?}"
        );
    }

    #[test]
    fn titled_lookup_is_exact_or_unique() {
        let lists = lists172();
        assert_eq!(
            lists[find_titled(&lists, "the descent checklist").unwrap()].title,
            "Descent"
        );
        assert_eq!(
            lists[find_titled(&lists, "before landing").unwrap()].title,
            "Before landing"
        );
        // "landing" alone is three sections: the caller asks which.
        let err = find_titled(&lists, "landing").unwrap_err();
        assert!(err.len() >= 2, "{err:?}");
        assert!(find_titled(&lists, "holding").is_err());
    }

    #[test]
    fn tracker_fires_each_section_once_in_flight_order() {
        let mut t = Tracker::default();
        let t0 = Instant::now();
        let s = |secs: f64| t0 + Duration::from_secs_f64(secs);
        // Parked, then the roll.
        assert_eq!(t.tick(&sample(true, 0.0, 0.0, 0.0), s(0.0)), None);
        assert_eq!(t.tick(&sample(true, 0.0, 0.0, 50.0), s(5.0)), None);
        // Liftoff: not yet a climb list.
        assert_eq!(t.tick(&sample(false, 100.0, 700.0, 70.0), s(10.0)), None);
        // Through 1,200 ft climbing - and it WAITS. The single tick used to
        // fire here, straight into the automation's own thousand-foot flap
        // and light actions, and audited them half done (EGLF, 2026-09-22).
        assert_eq!(t.tick(&sample(false, 1300.0, 700.0, 75.0), s(60.0)), None);
        assert_eq!(t.tick(&sample(false, 1900.0, 700.0, 75.0), s(70.0)), None);
        assert_eq!(
            t.tick(&sample(false, 2600.0, 700.0, 75.0), s(76.0)),
            Some("climb")
        );
        assert_eq!(t.tick(&sample(false, 3000.0, 700.0, 75.0), s(80.0)), None);
        // Level at 4,500: cruise after the hold.
        assert_eq!(t.tick(&sample(false, 4500.0, 0.0, 110.0), s(300.0)), None);
        assert_eq!(t.tick(&sample(false, 4500.0, 50.0, 110.0), s(320.0)), None);
        assert_eq!(
            t.tick(&sample(false, 4500.0, -50.0, 110.0), s(331.0)),
            Some("cruise")
        );
        // A brief sink is not a descent.
        assert_eq!(
            t.tick(&sample(false, 4400.0, -500.0, 110.0), s(400.0)),
            None
        );
        assert_eq!(t.tick(&sample(false, 4500.0, 0.0, 110.0), s(405.0)), None);
        // A sustained one is.
        assert_eq!(
            t.tick(&sample(false, 4300.0, -500.0, 115.0), s(900.0)),
            None
        );
        assert_eq!(
            t.tick(&sample(false, 4000.0, -500.0, 115.0), s(921.0)),
            Some("descent")
        );
        // Low and descending: before landing.
        assert_eq!(
            t.tick(&sample(false, 1400.0, -400.0, 90.0), s(1200.0)),
            None
        );
        assert_eq!(
            t.tick(&sample(false, 1300.0, -400.0, 90.0), s(1211.0)),
            Some("before landing")
        );
        // Touchdown, rollout, slow: after landing.
        assert_eq!(t.tick(&sample(true, 0.0, 0.0, 55.0), s(1400.0)), None);
        assert_eq!(t.tick(&sample(true, 0.0, 0.0, 20.0), s(1405.0)), None);
        assert_eq!(
            t.tick(&sample(true, 0.0, 0.0, 15.0), s(1416.0)),
            Some("after landing")
        );
        // Sitting there says nothing more; the next roll re-arms the flight.
        assert_eq!(t.tick(&sample(true, 0.0, 0.0, 0.0), s(1500.0)), None);
        assert_eq!(t.tick(&sample(true, 0.0, 0.0, 50.0), s(1600.0)), None);
        assert_eq!(t.tick(&sample(false, 1300.0, 700.0, 75.0), s(1700.0)), None);
        assert_eq!(
            t.tick(&sample(false, 1900.0, 700.0, 75.0), s(1716.0)),
            Some("climb")
        );
    }

    /// A climb that stops being one before the hold is up starts the clock
    /// again: levelling at 1,300 ft is not the climb the list belongs to.
    #[test]
    fn the_climb_list_needs_a_sustained_climb_not_a_moment_of_one() {
        let mut t = Tracker::default();
        let t0 = Instant::now();
        let s = |secs: f64| t0 + Duration::from_secs_f64(secs);
        t.tick(&sample(true, 0.0, 0.0, 50.0), s(0.0));
        assert_eq!(t.tick(&sample(false, 1300.0, 700.0, 75.0), s(10.0)), None);
        // Levels off: the climb is over and the clock is thrown away.
        assert_eq!(t.tick(&sample(false, 1400.0, 0.0, 75.0), s(18.0)), None);
        assert_eq!(t.tick(&sample(false, 1400.0, 0.0, 75.0), s(30.0)), None);
        // Climbing again: the hold starts from HERE, not from the first pass.
        assert_eq!(t.tick(&sample(false, 1500.0, 700.0, 75.0), s(31.0)), None);
        assert_eq!(t.tick(&sample(false, 2000.0, 700.0, 75.0), s(40.0)), None);
        assert_eq!(
            t.tick(&sample(false, 2600.0, 700.0, 75.0), s(47.0)),
            Some("climb")
        );
    }

    #[test]
    fn a_pattern_never_earns_a_cruise_or_descent_list() {
        let mut t = Tracker::default();
        let t0 = Instant::now();
        let s = |secs: f64| t0 + Duration::from_secs_f64(secs);
        t.tick(&sample(true, 0.0, 0.0, 50.0), s(0.0));
        assert_eq!(t.tick(&sample(false, 1300.0, 700.0, 75.0), s(30.0)), None);
        assert_eq!(
            t.tick(&sample(false, 1400.0, 700.0, 75.0), s(46.0)),
            Some("climb")
        );
        // Downwind at 1,000 ft for two minutes.
        for k in 0..24 {
            assert_eq!(
                t.tick(&sample(false, 1000.0, 0.0, 95.0), s(60.0 + k as f64 * 5.0)),
                None
            );
        }
        // Base and final: before landing only.
        assert_eq!(t.tick(&sample(false, 800.0, -500.0, 80.0), s(200.0)), None);
        assert_eq!(
            t.tick(&sample(false, 600.0, -500.0, 75.0), s(211.0)),
            Some("before landing")
        );
    }

    #[test]
    fn replies_are_a_closed_vocabulary() {
        assert_eq!(classify("stop the checklist"), Some(Reply::Abort));
        assert_eq!(classify("Cancel."), Some(Reply::Abort));
        assert_eq!(classify("yes"), Some(Reply::Yes));
        assert_eq!(classify("go ahead"), Some(Reply::Yes));
        assert_eq!(classify("negative"), Some(Reply::No));
        assert_eq!(classify("next"), Some(Reply::Next));
        assert_eq!(classify("Set."), Some(Reply::Next));
        assert_eq!(classify("checked, next item"), Some(Reply::Next));
        assert_eq!(classify("done"), Some(Reply::Next));
        assert_eq!(classify("skip that"), Some(Reply::Skip));
        assert_eq!(classify("hold on"), Some(Reply::Hold));
        assert_eq!(
            classify("Can you help me with that item?"),
            Some(Reply::DoIt)
        );
        assert_eq!(classify("you do it"), Some(Reply::DoIt));
        assert_eq!(classify("do it for me"), Some(Reply::DoIt));
        assert_eq!(classify("help me with that"), Some(Reply::DoIt));
        assert_eq!(classify("do it"), Some(Reply::Yes));
        assert_eq!(classify("stand by"), Some(Reply::Hold));
        // Not replies: commands and chatter route on.
        assert_eq!(classify("flaps ten"), None);
        assert_eq!(classify("stop pushback"), None);
        assert_eq!(classify("set heading two seven zero"), None);
        assert_eq!(classify("yes please give me the fuel"), None);
        assert_eq!(classify(""), None);
    }

    #[test]
    fn proposal_replies() {
        assert_eq!(proposal_reply("go ahead"), Some(ProposalReply::Run));
        assert_eq!(proposal_reply("Yes, run it."), Some(ProposalReply::Run));
        assert_eq!(proposal_reply("ready"), Some(ProposalReply::Run));
        assert_eq!(
            proposal_reply("talk me through it"),
            Some(ProposalReply::Instruct)
        );
        assert_eq!(
            proposal_reply("walk me through the checklist"),
            Some(ProposalReply::Instruct)
        );
        assert_eq!(proposal_reply("not now"), Some(ProposalReply::Decline));
        assert_eq!(
            proposal_reply("later, thanks"),
            Some(ProposalReply::Decline)
        );
        assert_eq!(proposal_reply("negative"), Some(ProposalReply::Decline));
        assert_eq!(proposal_reply("flaps ten"), None);
        assert_eq!(proposal_reply("yes flaps ten"), None);
        assert_eq!(proposal_reply("go around"), None);
    }

    #[test]
    fn instruct_requests_name_the_list() {
        assert_eq!(
            instruct_request("talk me through the descent checklist"),
            Some("descent".into())
        );
        assert_eq!(
            instruct_request("Walk me through the before landing checklist."),
            Some("before landing".into())
        );
        assert_eq!(
            instruct_request("teach me the climb checklist"),
            Some("climb".into())
        );
        // No list named, or no "checklist": not an instruct request.
        assert_eq!(instruct_request("talk me through the approach"), None);
        assert_eq!(instruct_request("walk me through it"), None);
        assert_eq!(instruct_request("run the descent checklist"), None);
    }

    fn act_item(say: &str, cond: Cond) -> Item {
        Item::Act {
            say: say.into(),
            tool: "set_lights",
            args: json!({}),
            confirm: None,
            dataref: "sim/cockpit/electrical/landing_lights_on".into(),
            index: None,
            cond,
        }
    }

    #[test]
    fn plans_by_flavour() {
        let it = act_item("Landing lights, off.", Cond::Eq(0.0));
        // Already in state: confirmed, never written, in every flavour.
        assert_eq!(
            plan(Flavour::Crew, &it, Some(0.0), None),
            Step::Say("Landing lights, off.".into())
        );
        // The monitor confirms in silence (its one line is the summary).
        assert_eq!(plan(Flavour::Monitor, &it, Some(0.0), None), Step::Skip);
        assert_eq!(
            plan(Flavour::Instruct, &it, Some(0.0), None),
            Step::Say("Landing lights, off. Already done.".into())
        );
        // Not in state: crew acts, monitor reports, instructor waits.
        assert_eq!(plan(Flavour::Crew, &it, Some(1.0), None), Step::Act);
        assert_eq!(
            plan(Flavour::Monitor, &it, Some(1.0), None),
            Step::Discrepancy("Landing lights, off. Not set.".into())
        );
        assert_eq!(
            plan(Flavour::Instruct, &it, Some(1.0), None),
            Step::Wait("Landing lights, off.".into())
        );
        // Owned by a loop: named as held, no write, no accusation.
        assert_eq!(
            plan(Flavour::Crew, &it, Some(1.0), Some("the speed hold")),
            Step::Held("Landing lights, off. The speed hold has it.".into())
        );
        // Unreadable: the crew still acts (the tool reads back itself); the
        // monitor moves on rather than call it a discrepancy.
        assert_eq!(plan(Flavour::Crew, &it, None, None), Step::Act);
        assert_eq!(plan(Flavour::Monitor, &it, None, None), Step::Skip);
        // Narration and call-outs: skipped by the monitor, spoken by the
        // crew. Call-outs: the instructor waits on the pilot's word.
        let n = Item::Narrate("Recommended climb airspeed 70 to 85 knots.".into());
        assert_eq!(plan(Flavour::Monitor, &n, None, None), Step::Skip);
        assert!(matches!(plan(Flavour::Crew, &n, None, None), Step::Say(_)));
        let c = Item::CallOut("Fuel quantity, sufficient.".into());
        assert_eq!(plan(Flavour::Monitor, &c, None, None), Step::Skip);
        assert!(matches!(plan(Flavour::Crew, &c, None, None), Step::Say(_)));
        assert!(matches!(
            plan(Flavour::Instruct, &c, None, None),
            Step::Wait(_)
        ));
        // A verify item that is observe-only never becomes an Act.
        let v = Item::Verify {
            say: "Brakes, minimum possible.".into(),
            dataref: "sim/cockpit2/gauges/indicators/airspeed_kts_pilot".into(),
            index: None,
            cond: Cond::Lt(50.0),
            settable: false,
        };
        assert_eq!(
            plan(Flavour::Crew, &v, Some(70.0), None),
            Step::Say("Brakes, minimum possible.".into())
        );
    }

    #[test]
    fn monitor_summary_names_its_discrepancies() {
        // The monitor's one line per list: clean, or the items by name.
        assert_eq!(
            summary_line("Climb", Flavour::Monitor, false, 0, 0, &[]),
            "Climb checklist complete."
        );
        let names = vec![
            "Flaps, up on schedule.".to_string(),
            "Altimeters, standard at transition.".to_string(),
        ];
        assert_eq!(
            summary_line("Climb", Flavour::Monitor, false, 0, 2, &names),
            "Climb checklist complete, 2 items not as listed: flaps, up on schedule; altimeters, standard at transition."
        );
        assert_eq!(
            summary_line("Descent", Flavour::Monitor, false, 0, 1, &names[..1]),
            "Descent checklist complete, 1 item not as listed: flaps, up on schedule."
        );
        // The crew flavour counts failed writes; the instructor counts what
        // the pilot never did (no names: each was said as it happened).
        assert_eq!(
            summary_line("Descent", Flavour::Crew, false, 1, 0, &[]),
            "Descent checklist complete, 1 item didn't take, check the panel."
        );
        assert_eq!(
            summary_line("Descent", Flavour::Instruct, false, 0, 3, &[]),
            "Descent checklist complete, 3 items not as listed."
        );
        assert_eq!(
            summary_line("Descent", Flavour::Monitor, true, 0, 1, &names),
            "Descent checklist stopped where we were."
        );
    }

    #[test]
    fn take_reply_honours_what_is_waited_on() {
        let _g = crate::mode::TEST_LOCK
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        // Nothing running: nothing is a reply.
        ACTIVE.store(false, SeqCst);
        assert_eq!(take_reply("yes"), None);
        ACTIVE.store(true, SeqCst);
        *lock(&WAITING) = None;
        *lock(&REPLY) = None;
        // Running, waiting on nothing: a stray "yes" is not consumed, "stop" is.
        assert_eq!(take_reply("yes"), None);
        assert_eq!(take_reply("stop the checklist"), Some(Reply::Abort));
        assert!(ABORT.load(SeqCst));
        ABORT.store(false, SeqCst);
        // At a confirm: yes/no taken, "next" is not an answer.
        *lock(&WAITING) = Some(Waiting::Confirm);
        assert_eq!(take_reply("next"), None);
        assert_eq!(take_reply("affirm"), Some(Reply::Yes));
        assert_eq!(lock(&REPLY).take(), Some(Reply::Yes));
        // At an instructor item: set/next/skip/yes all taken.
        *lock(&WAITING) = Some(Waiting::Advance);
        assert_eq!(take_reply("set"), Some(Reply::Next));
        assert_eq!(take_reply("skip"), Some(Reply::Skip));
        assert_eq!(
            take_reply("can you help me with that item"),
            Some(Reply::DoIt)
        );
        // ...but not at a hazard confirm, where "help me" is not a yes.
        *lock(&WAITING) = Some(Waiting::Confirm);
        assert_eq!(take_reply("help me with that"), None);
        *lock(&WAITING) = Some(Waiting::Advance);
        assert_eq!(take_reply("flaps ten"), None);
        ACTIVE.store(false, SeqCst);
        *lock(&WAITING) = None;
        *lock(&REPLY) = None;
    }

    #[test]
    fn proposals_expire() {
        let _g = crate::mode::TEST_LOCK
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        *lock(&PROPOSAL) = Some((2, Instant::now() - PROPOSAL_TTL - Duration::from_secs(1)));
        assert_eq!(proposal(), None);
        *lock(&PROPOSAL) = Some((2, Instant::now()));
        assert_eq!(proposal(), Some(2));
        decline_proposal();
        assert_eq!(proposal(), None);
    }
}
