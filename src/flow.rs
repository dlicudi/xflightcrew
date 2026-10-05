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
//! The after-landing list is not proposed: it is the copilot's own clean-up
//! off the runway and runs at once in the Crew flavour.
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
/// The running flow's run: a cancel stops it where it is (the pilot's
/// abort, a stand-down, `flight_state::LOOPS`); its summary says so.
pub(crate) static RUN: sim::run::Slot = sim::run::Slot::new();

/// The running flow was stopped where it is.
fn stopped() -> bool {
    RUN.running() && !RUN.live()
}

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

/// The running flow's flavour (`None` when no flow runs).
static RUNNING: Mutex<Option<Flavour>> = Mutex::new(None);
/// Why a running Crew flow was stood down, for its closing line.
static STOOD_DOWN: Mutex<Option<StandDown>> = Mutex::new(None);
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
    pub agl_ft: f64,
    pub vsi_fpm: f64,
    /// An automation has the aircraft (the list would run as Monitor).
    pub automation: bool,
    /// Gear down and flaps at the landing detent ([`landing_config`]).
    pub landing_config: bool,
    /// The flaps are up ([`climb_config`]): the climb-out schedule is done.
    pub climb_config: bool,
    /// On runway pavement (`None` = no runway data, or airborne).
    pub on_runway: Option<bool>,
}

/// The climb-out the CLIMB list audits: the flap handle up (its own test,
/// `flap_ratio < 0.05`), or an airframe with no flap profile. Pure.
pub fn climb_config(flap_ratio: f64, has_flaps: bool) -> bool {
    !has_flaps || flap_ratio < 0.05
}

/// The landing configuration the BEFORE LANDING list audits: the gear down
/// (a fixed gear reads down) and the flap handle at the landing detent, or
/// an airframe with no flap profile. Pure.
pub fn landing_config(gear_down: bool, flap_ratio: f64, has_flaps: bool) -> bool {
    gear_down && (!has_flaps || flap_ratio >= crate::autoland::LANDING_FLAP_RATIO)
}

/// The landing configuration an automation's Monitor list audits: the gear
/// and flap of [`landing_config`] AND autoland's own landing items set for
/// this approach (`autoland_configured`, its `landing_config` event: the
/// spoilers armed, the autobrake). The circuit puts the landing flap out on
/// the final turn, but autoland arms the spoilers and the autobrake only once
/// it has the final: the A330's list ran at +411 s with the gear and flap
/// down and named both, 48 s before autoland set them (LEMG 30, sortie
/// 1790633832-full_auto, #169). With a pilot flying it is [`landing_config`]
/// alone (and the trigger ignores it). Pure.
pub fn landing_config_for(
    automation: bool,
    gear_down: bool,
    flap_ratio: f64,
    has_flaps: bool,
    autoland_configured: bool,
) -> bool {
    landing_config(gear_down, flap_ratio, has_flaps) && (!automation || autoland_configured)
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
/// After-landing list: this long after the rollout slowed through 30 kt
/// (the model's taxi-in) AND off the runway pavement.
pub const AFTER_LANDING_SECS: f64 = 10.0;

/// Phase-boundary detector, one per monitor loop. Each section fires ONCE per
/// flight, on sustained conditions rather than a single tick, so a gust does
/// not read as a descent. The FLIGHT is the model's (`flight::Flight`): it
/// begins at the model's takeoff (or a reposition), its peak height is the
/// model's memory, on the ground or airborne is the model's phase (a bounce
/// is not a new flight), and the after-landing list needs the model's taxi-in
/// after a real touchdown (an impact or a wreck runs no list). The sustained
/// thresholds stay the tracker's own: each carries a sortie behind it.
#[derive(Default)]
pub struct Tracker {
    fired: Vec<&'static str>,
    climbing_since: Option<Instant>,
    level_since: Option<Instant>,
    descending_since: Option<Instant>,
    low_descent_since: Option<Instant>,
    taxi_in_since: Option<Instant>,
}

/// The model's rollout ends 3 s under 30 kt; the list waited 10 s under it.
const AFTER_LANDING_TAXI_IN_SECS: f64 = AFTER_LANDING_SECS - 3.0;

impl Tracker {
    fn fired(&self, s: &str) -> bool {
        self.fired.contains(&s)
    }

    fn fire(&mut self, s: &'static str) -> Option<&'static str> {
        self.fired.push(s);
        Some(s)
    }

    /// Feed one tick with the model as it stands after this tick and the
    /// events the tick produced; the section whose boundary was just
    /// crossed, if any.
    pub fn tick(
        &mut self,
        s: &Sample,
        f: &crate::flight::Flight,
        events: &[crate::flight::Event],
        now: Instant,
    ) -> Option<&'static str> {
        use crate::flight::{Abnormal, Event, Phase};
        if events
            .iter()
            .any(|e| matches!(e, Event::Takeoff | Event::Reposition { .. }))
        {
            // A new flight arms every section.
            *self = Tracker::default();
        }
        if matches!(f.abnormal(), Abnormal::Crashed) {
            return None;
        }
        let held = |since: &mut Option<Instant>, on: bool, secs: f64| -> bool {
            if !on {
                *since = None;
                return false;
            }
            let t = *since.get_or_insert(now);
            now.duration_since(t).as_secs_f64() >= secs
        };
        let phase = f.phase();
        let m = f.memory();
        if !phase.airborne() {
            self.climbing_since = None;
            self.level_since = None;
            self.descending_since = None;
            self.low_descent_since = None;
            // The list is for OFF the runway: it waited only for the taxi
            // speed and ran "landing lights, off" on the strip during a
            // 730 m roll to the exit (EGBB 15, #174). The pavement's word,
            // the same test as "runway vacated"; no runway data is no
            // evidence and leaves the old trigger.
            let landed =
                phase == Phase::TaxiIn && m.landings > 0 && s.on_runway != Some(true);
            let stopped = held(&mut self.taxi_in_since, landed, AFTER_LANDING_TAXI_IN_SECS);
            if stopped && !self.fired("after landing") {
                // The flight is over: the next one starts from scratch.
                self.fired.clear();
                self.taxi_in_since = None;
                return self.fire("after landing");
            }
            return None;
        }
        self.taxi_in_since = None;
        let peak = m.peak_agl_ft;
        let climbing = held(
            &mut self.climbing_since,
            s.vsi_fpm > 300.0 && s.agl_ft >= CLIMB_AGL_FT,
            CLIMB_SECS,
        );
        // Under an automation the list audits the climb-out, so it waits for
        // the paced flap schedule to finish: a 737 is still at flaps one at
        // 1,550 ft and the list read "flaps, up on schedule" at 0.125 (EGKK
        // 08R, sortie 1790624015-full_auto, #174). A pilot flying is proposed
        // the list on the old trigger.
        let clean = !s.automation || s.climb_config;
        if climbing && clean && !self.fired("climb") {
            return self.fire("climb");
        }
        let level = held(
            &mut self.level_since,
            s.vsi_fpm.abs() < 300.0 && s.agl_ft >= CRUISE_MIN_AGL_FT,
            CRUISE_LEVEL_SECS,
        );
        // A level-off inside the descent is a step of it, never a cruise.
        if level && !self.fired("cruise") && !self.fired("descent") {
            return self.fire("cruise");
        }
        // The model's descent too: an idle descent saws between -2,000 fpm
        // and level, restarting the held needle every 15 s, and the list came
        // 7,000 ft into it (EGKK-EGBB, #174). The model reads the height lost.
        let descending = held(
            &mut self.descending_since,
            s.vsi_fpm < -300.0 && peak >= CRUISE_MIN_AGL_FT,
            DESCENT_SECS,
        ) || (phase == Phase::Descent && peak >= CRUISE_MIN_AGL_FT);
        if descending && !self.fired("descent") {
            return self.fire("descent");
        }
        let low = held(
            &mut self.low_descent_since,
            s.agl_ft < BEFORE_LANDING_AGL_FT && s.vsi_fpm < -100.0 && peak >= 1000.0,
            BEFORE_LANDING_SECS,
        );
        // Under an automation the list audits the landing configuration, so
        // it waits for it: a circuit's downwind descent is low and sinking
        // long before autoland configures, and the A330's Monitor list ran
        // at +527 s against the config set at +667 s, five items "not as
        // listed" (LEMG 13, sortie 1790621184-full_auto, #169). A pilot
        // flying is proposed the list on the old trigger.
        let configured = !s.automation || s.landing_config;
        if low && configured && !self.fired("before landing") {
            return self.fire("before landing");
        }
        None
    }
}

/// Is an automation flying the aircraft (so the copilot is the pilot flying
/// and the list is its own cross-check)? [`crate::flight_state::crew_flying`]:
/// the takeoff, the helicopter's loops and a recovery count too (they were
/// missing here, so a stand-alone takeoff's climb list was PROPOSED to a
/// pilot who was not flying; refactor audit 2026-10-04, duplicates #1).
pub fn automation_flying() -> bool {
    crate::flight_state::crew_flying().is_some()
}

/// The automation that has TAKEN the aircraft, if any, outermost first: a
/// mission or a loop that flies it (full auto from its first stage, the
/// takeoff, the circuit, the follower, autoland, the helicopter's loops, a
/// recovery). The ground drives (the taxi, the tug) are not on it: they
/// share the cockpit with an after-landing clean-up, which is where that
/// list runs. Spoken ("held by the circuit").
pub fn mission_owner() -> Option<&'static str> {
    use crate::flight_state::Owner;
    Owner::OUTERMOST_FIRST
        .into_iter()
        .find(|o| o.flies() && o.active())
        .map(Owner::spoken)
}

/// Why a Crew flow stopped before its end.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum StandDown {
    /// An automation took the aircraft after the list began (its name).
    Mission(&'static str),
    /// A new flight was set up or the sim moved the aircraft (the cause).
    NewFlight(&'static str),
}

impl StandDown {
    fn by(self) -> &'static str {
        match self {
            StandDown::Mission(m) | StandDown::NewFlight(m) => m,
        }
    }
}

/// A Crew flow's watch for an automation TAKING the aircraft under it: the
/// owner at the list's start is the baseline (a list the pilot asked for
/// under full auto names the mission's items as held and goes on); an owner
/// appearing where there was none stands the list down. Pure.
///
/// LEMG (2026-09-28, sorties 1790585651 and 1790586620): the After landing
/// list began as Crew in the taxi in, blocked on its own speech, and ran on
/// into the next full-auto departure: "Speed brake, down." at +20 s and
/// "Flaps, up." at +46 s over the takeoff flaps full auto had set at
/// +0.3 s, twice. The second "Flaps, up." came 1.8 s AFTER full auto gave
/// up, so the stand-down is a latch, never a check of the moment.
#[derive(Clone, Copy, Debug)]
pub struct CrewWatch {
    last: Option<&'static str>,
}

impl CrewWatch {
    pub fn new(at_start: Option<&'static str>) -> CrewWatch {
        CrewWatch { last: at_start }
    }

    /// The owner now; `Some(owner)` once, when one took the aircraft.
    pub fn step(&mut self, now: Option<&'static str>) -> Option<&'static str> {
        let took = match (self.last, now) {
            (None, Some(owner)) => Some(owner),
            _ => None,
        };
        self.last = now;
        took
    }
}

/// A new flight set up by the plugin's init mailbox while a list runs
/// (the crew's ramp, runway and approach setups, the sortie runner): the
/// plugin's `init_seq` counts completed inits. The first good read is the
/// baseline; a failed read is no measurement. Pure.
#[derive(Clone, Copy, Debug, Default)]
pub struct InitWatch {
    base: Option<f64>,
}

impl InitWatch {
    /// `true` once the counter moved off the baseline.
    pub fn step(&mut self, seq: Option<f64>) -> bool {
        match (self.base, seq) {
            (None, Some(s)) => {
                self.base = Some(s);
                false
            }
            (Some(b), Some(s)) => s != b,
            _ => false,
        }
    }
}

/// Stand a running flow down where it is, and drop any open proposal. An
/// automation taking the aircraft stands down a CREW flow only (a Monitor
/// flow writes nothing and is the audit that automation wants); a new flight
/// stands down every flavour, since the list belongs to the flight that was.
/// The first cause is the one said. `true` when a flow was stood down.
pub fn stand_down(why: StandDown) -> bool {
    *lock(&PROPOSAL) = None;
    let running = *lock(&RUNNING);
    let applies = match why {
        StandDown::Mission(_) => running == Some(Flavour::Crew),
        StandDown::NewFlight(_) => running.is_some(),
    };
    if !ACTIVE.load(SeqCst) || !applies {
        return false;
    }
    lock(&STOOD_DOWN).get_or_insert(why);
    RUN.cancel();
    true
}

/// The stood-down list's closing line. Pure.
pub fn stood_down_line(title: &str, why: StandDown) -> String {
    match why {
        StandDown::Mission(m) => format!("{title} checklist stopped, {m} has the aircraft."),
        StandDown::NewFlight(_) => format!("{title} checklist dropped, new flight."),
    }
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
    // The after-landing list is the copilot's own clean-up once off the
    // runway (flaps, lights, transponder): the pilot is taxiing, and the
    // proposal was one more thing to answer (asked, 2026-09-22). It runs at
    // once, Crew flavour, which still reads each item first and never
    // rewrites one in state. In instructor mode `start` makes it Instruct.
    if section == "after landing" {
        start(idx, Flavour::Crew, config.clone(), voice.clone());
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
        RUN.cancel();
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
    // ACTIVE drops only after the last run's lease: the slot is free.
    let Some(lease) = RUN.begin() else {
        ACTIVE.store(false, SeqCst);
        return false;
    };
    *lock(&REPLY) = None;
    *lock(&WAITING) = None;
    *lock(&PROPOSAL) = None;
    *lock(&STOOD_DOWN) = None;
    *lock(&RUNNING) = Some(flavour);
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
            lease.record(sim::run::Outcome::Completed);
            drop(lease);
            DETAIL.clear();
            *lock(&WAITING) = None;
            *lock(&RUNNING) = None;
            ACTIVE.store(false, SeqCst);
        })
        .map(|_| true)
        .unwrap_or_else(|_| {
            *lock(&RUNNING) = None;
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
    // The loop flying the aircraft holds its autopilot
    // (`flight_state::crew_flying`): the takeoff, full auto, the helicopter's
    // loops and a recovery too, each by its own name (a circuit was named
    // "the follower"; refactor audit 2026-10-04, duplicates #1).
    let flying = crate::flight_state::crew_flying();
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
    if dataref.contains("autopilot") {
        if let Some(owner) = flying {
            return Some(owner.spoken());
        }
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
        if stopped() {
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
        if stopped() {
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
    // A Crew list writes the cockpit: an automation taking the aircraft
    // after it began stands it down (the list belongs to the flight that
    // was); one already flying at its start names its items as held.
    let mut watch = CrewWatch::new(mission_owner());
    // ...and a new flight set up under any list ends it.
    let mut init = InitWatch::default();
    let init_seq = || async { xp.get_float(crate::tools::setup::INIT_SEQ).await.ok() };
    init.step(init_seq().await);
    for (i, item) in cl.items.iter().enumerate() {
        if flavour == Flavour::Crew {
            if let Some(owner) = watch.step(mission_owner()) {
                stand_down(StandDown::Mission(owner));
            }
        }
        if init.step(init_seq().await) {
            stand_down(StandDown::NewFlight("setup"));
        }
        if stopped() {
            aborted = true;
            break;
        }
        DETAIL.set(format!("{title_lc}, {} of {n}", i + 1));
        let cond = item_cond(item);
        let value = match cond {
            Some((d, idx, _)) => checklist::read_dataref(xp, d, idx).await,
            None => None,
        };
        let held = cond
            .and_then(|(d, _, _)| held_by_automation(d))
            .or_else(|| {
                // Under a mission a Crew list writes nothing: every item not in
                // state is the mission's (one writer per dataref).
                if flavour == Flavour::Crew && cond.is_some() {
                    mission_owner()
                } else {
                    None
                }
            });
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
    let stood_down = if aborted { *lock(&STOOD_DOWN) } else { None };
    let summary = match stood_down {
        Some(why) => stood_down_line(&cl.title, why),
        None => summary_line(&cl.title, flavour, aborted, failed, discrepancies, &not_set),
    };
    crate::blackbox::record(
        "checklist",
        json!({
            "event": if stood_down.is_some() { "stood_down" } else if aborted { "abort" } else { "complete" },
            "title": cl.title, "flavour": flavour.name(),
            "failed": failed, "discrepancies": discrepancies,
            "by": stood_down.map(StandDown::by),
            "new_flight": matches!(stood_down, Some(StandDown::NewFlight(_))),
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

    /// The tracker flown through the real flight model at 1 Hz: the model
    /// decides the flight (takeoff, peak, taxi-in), the tracker its
    /// sustained boundaries.
    struct Rig {
        f: crate::flight::Flight,
        t: Tracker,
        t0: Instant,
        secs: f64,
        agl: f64,
        fired: Vec<(f64, &'static str)>,
        /// An automation flying, and whether it has configured to land.
        automation: bool,
        configured: bool,
        /// Flaps up (the climb list's wait under an automation).
        clean: bool,
        on_runway: Option<bool>,
    }

    impl Rig {
        fn new() -> Rig {
            Rig {
                f: crate::flight::Flight::new(),
                t: Tracker::default(),
                t0: Instant::now(),
                secs: 0.0,
                agl: 0.0,
                fired: Vec::new(),
                automation: false,
                configured: false,
                clean: true,
                on_runway: None,
            }
        }
        /// `n` seconds on the ground at `gs`, or airborne at `vsi` (the
        /// height integrates), one sample a second.
        fn fly(&mut self, n: usize, ground: bool, vsi: f64, gs: f64) -> &mut Rig {
            for _ in 0..n {
                self.secs += 1.0;
                if ground {
                    self.agl = 0.0;
                } else {
                    self.agl = (self.agl + vsi / 60.0).max(60.0);
                }
                let fs = crate::flight::Sample {
                    t: self.secs,
                    on_ground: ground,
                    gs_kts: gs,
                    ias_kts: gs,
                    vsi_fpm: vsi,
                    agl_ft: self.agl,
                    alt_ft: self.agl + 100.0,
                    engine_running: true,
                    ..crate::flight::Sample::default()
                };
                let ev = self.f.tick(&fs);
                let now = self.t0 + Duration::from_secs_f64(self.secs);
                let s = Sample {
                    agl_ft: self.agl,
                    vsi_fpm: vsi,
                    automation: self.automation,
                    landing_config: self.configured,
                    climb_config: self.clean,
                    on_runway: self.on_runway,
                };
                if let Some(sec) = self.t.tick(&s, &self.f, &ev, now) {
                    self.fired.push((self.agl, sec));
                }
            }
            self
        }
        fn names(&self) -> Vec<&'static str> {
            self.fired.iter().map(|(_, n)| *n).collect()
        }
    }

    #[test]
    fn tracker_fires_each_section_once_in_flight_order() {
        let mut r = Rig::new();
        r.fly(5, true, 0.0, 0.0).fly(8, true, 0.0, 50.0);
        // Climb at 700 fpm to 4,500 ft: the climb list waits for 1,200 ft
        // AND fifteen seconds of it (EGLF, 2026-09-22).
        r.fly(380, false, 700.0, 75.0);
        let climb_at = r.fired.iter().find(|(_, n)| *n == "climb").map(|(a, _)| *a);
        assert!(climb_at.is_some_and(|a| a >= 1200.0 + 700.0 / 4.0 - 20.0), "{:?}", r.fired);
        // Level: cruise after the hold. A five second sink is not a descent.
        r.fly(40, false, 0.0, 110.0).fly(5, false, -500.0, 110.0).fly(10, false, 0.0, 110.0);
        assert_eq!(r.names(), vec!["climb", "cruise"]);
        // A sustained descent is; then low and descending, before landing.
        r.fly(560, false, -500.0, 110.0);
        assert_eq!(r.names(), vec!["climb", "cruise", "descent", "before landing"]);
        assert!(r.agl <= 60.0, "on short final: {}", r.agl);
        // Touchdown, rollout, slow: after landing, once.
        r.fly(8, true, 0.0, 55.0).fly(30, true, 0.0, 15.0);
        assert_eq!(
            r.names(),
            vec!["climb", "cruise", "descent", "before landing", "after landing"]
        );
        // The next flight arms them all again.
        r.fly(10, true, 0.0, 0.0).fly(8, true, 0.0, 50.0).fly(160, false, 700.0, 75.0);
        assert_eq!(r.names().last(), Some(&"climb"));
        assert_eq!(r.names().iter().filter(|n| **n == "climb").count(), 2);
    }

    /// A climb that stops being one before the hold is up starts the clock
    /// again: levelling at 1,300 ft is not the climb the list belongs to.
    #[test]
    fn the_climb_list_needs_a_sustained_climb_not_a_moment_of_one() {
        let mut r = Rig::new();
        r.fly(5, true, 0.0, 0.0).fly(8, true, 0.0, 50.0);
        r.fly(110, false, 700.0, 75.0); // to ~1,300 ft, 5 s past 1,200
        r.fly(20, false, 0.0, 75.0);
        assert!(r.names().is_empty(), "{:?}", r.fired);
        r.fly(14, false, 700.0, 75.0);
        assert!(r.names().is_empty(), "the hold restarts: {:?}", r.fired);
        r.fly(3, false, 700.0, 75.0);
        assert_eq!(r.names(), vec!["climb"]);
    }

    #[test]
    fn a_pattern_never_earns_a_cruise_or_descent_list() {
        let mut r = Rig::new();
        r.fly(5, true, 0.0, 0.0).fly(8, true, 0.0, 50.0);
        r.fly(130, false, 700.0, 75.0);
        assert_eq!(r.names(), vec!["climb"]);
        // Downwind at circuit height for two minutes.
        r.fly(120, false, 0.0, 95.0);
        // Base and final: before landing only.
        r.fly(100, false, -500.0, 80.0);
        assert_eq!(r.names(), vec!["climb", "before landing"]);
    }

    /// #169: under an automation the BEFORE LANDING list audits the landing
    /// configuration, so a circuit's downwind descent does not run it; it
    /// runs once gear and landing flap are set. A pilot flying keeps the
    /// old trigger (a proposal, which the pilot answers when configured).
    #[test]
    fn an_automations_before_landing_list_waits_for_the_landing_config() {
        let mut r = Rig::new();
        r.automation = true;
        r.fly(5, true, 0.0, 0.0).fly(8, true, 0.0, 50.0);
        r.fly(130, false, 700.0, 75.0);
        assert_eq!(r.names(), vec!["climb"]);
        // Downwind and base, descending, clean: no list.
        r.fly(60, false, -500.0, 80.0);
        assert_eq!(r.names(), vec!["climb"], "{:?}", r.fired);
        // Autoland's landing configuration: the list, at once.
        r.configured = true;
        r.fly(1, false, -500.0, 80.0);
        assert_eq!(r.names(), vec!["climb", "before landing"]);

        let mut p = Rig::new();
        p.fly(5, true, 0.0, 0.0).fly(8, true, 0.0, 50.0);
        p.fly(130, false, 700.0, 75.0).fly(60, false, -500.0, 80.0);
        assert_eq!(p.names(), vec!["climb", "before landing"]);

        assert!(landing_config(true, 1.0, true));
        assert!(!landing_config(true, 0.75, true), "CONF 3 is not the landing flap");
        assert!(!landing_config(false, 1.0, true), "gear up");
        assert!(landing_config(true, 0.0, false), "no flaps to set");
    }

    /// #169, LEMG 30 (sortie 1790633832-full_auto, 2026-09-28): the circuit
    /// put the A330's landing flap out on the final turn (+408.7 s), the
    /// Monitor list ran at +410.9 and named the speed brake and the autobrake,
    /// and autoland set both at +459.3, once the circuit handed it the final.
    /// Under an automation the list's configuration is autoland's too.
    #[test]
    fn an_automations_landing_config_is_autolands() {
        // The circuit's final: gear down, full flap, autoland not yet set.
        assert!(!landing_config_for(true, true, 1.0, true, false));
        // Autoland's landing_config event.
        assert!(landing_config_for(true, true, 1.0, true, true));
        // Autoland's items alone are not the configuration.
        assert!(!landing_config_for(true, true, 0.5, true, true));
        // A pilot flying: the gear and flap, as before.
        assert!(landing_config_for(false, true, 1.0, true, false));
    }

    /// #174 (EGKK 08R, sortie 1790624015-full_auto): the Monitor CLIMB list
    /// audited at 1,550 ft with the 737 at flaps one, 13 s before the paced
    /// schedule put them up at 2,747 ft, and named "flaps, up on schedule".
    /// Under an automation it waits for the flaps; a pilot flying is proposed
    /// the list on the old trigger.
    #[test]
    fn an_automations_climb_list_waits_for_the_flaps() {
        let mut r = Rig::new();
        r.automation = true;
        r.clean = false;
        r.fly(5, true, 0.0, 0.0).fly(8, true, 0.0, 50.0);
        // 2,000 fpm to 2,700 ft, flaps one all the way.
        r.fly(81, false, 2000.0, 170.0);
        assert!(r.names().is_empty(), "{:?}", r.fired);
        r.clean = true;
        r.fly(1, false, 2000.0, 190.0);
        assert_eq!(r.names(), vec!["climb"]);

        let mut p = Rig::new();
        p.clean = false;
        p.fly(5, true, 0.0, 0.0).fly(8, true, 0.0, 50.0).fly(81, false, 2000.0, 170.0);
        assert_eq!(p.names(), vec!["climb"]);

        assert!(climb_config(0.0, true));
        assert!(!climb_config(0.125, true), "flaps one");
        assert!(climb_config(0.5, false), "no flaps to raise");
    }

    /// Recorded `state` records through the flight model and the tracker,
    /// with the pavement's word per tick; the sections and when (+s).
    fn replay_tracker(
        text: &str,
        automation: bool,
        on_runway: impl Fn(f64) -> Option<bool>,
    ) -> Vec<(f64, &'static str)> {
        let t0 = 1790624015.0;
        let start = Instant::now();
        let mut r = crate::flight::Replay::default();
        let mut tr = Tracker::default();
        let mut fired = Vec::new();
        let mut first = None;
        for v in text.lines().filter_map(|l| serde_json::from_str::<serde_json::Value>(l).ok()) {
            let Some((s, ev)) = r.feed(&v) else { continue };
            let f0 = *first.get_or_insert(s.t);
            let now = start + Duration::from_secs_f64(s.t - f0);
            let fs = Sample {
                agl_ft: s.agl_ft,
                vsi_fpm: s.vsi_fpm,
                automation,
                landing_config: true,
                climb_config: true,
                on_runway: if s.on_ground { on_runway(s.t - t0) } else { None },
            };
            if let Some(sec) = tr.tick(&fs, &r.flight, &ev, now) {
                fired.push((s.t - t0, sec));
            }
        }
        fired
    }

    /// #174 (EGBB 15): the Crew AFTER LANDING list ran at +2846, on the
    /// runway, during a 730 m roll to exit C ("landing lights, off" on the
    /// strip); the vacate was +3005.4 (`taxi_in` `vacated`, `on_runway`
    /// false). The list waits for the pavement.
    #[test]
    fn the_after_landing_list_waits_for_the_vacate() {
        const ROLL: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/flight/egbb15_rollout_to_vacate.jsonl"));
        let blind = replay_tracker(ROLL, false, |_| None);
        let at = blind.iter().find(|(_, n)| *n == "after landing").map(|(t, _)| *t);
        assert!(at.is_some_and(|t| t < 2850.0), "the recorded failure: {blind:?}");
        let fed = replay_tracker(ROLL, false, |t| Some(t < 3005.4));
        let at = fed.iter().find(|(_, n)| *n == "after landing").map(|(t, _)| *t);
        assert!(at.is_some_and(|t| t >= 3005.4), "{fed:?}");
    }

    /// #174 (EGKK-EGBB): the DESCENT list came at +1750, 7,000 ft into an
    /// idle descent that saws between -2,000 fpm and level; the model's
    /// descent (the height lost) runs it near the top, and the level-off at
    /// 12,700 inside it earns no cruise list.
    #[test]
    fn the_descent_list_runs_near_the_top_of_a_sawtooth_descent() {
        const TOD: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/flight/egkk_egbb_top_of_descent.jsonl"));
        let fired = replay_tracker(TOD, true, |_| None);
        let at = fired.iter().find(|(_, n)| *n == "descent").map(|(t, _)| *t);
        assert!(at.is_some_and(|t| t < 1630.0), "{fired:?}");
        let cruise = fired.iter().find(|(_, n)| *n == "cruise").map(|(t, _)| *t);
        assert!(cruise.is_none_or(|c| c < 1594.0), "{fired:?}");
    }

    /// The vacate's lights and the list's agree: landing light off, which
    /// the list checks; the taxi light on, which it does not (one state for
    /// every writer at that moment).
    #[test]
    fn the_vacate_lights_agree_with_the_after_landing_list() {
        let text = std::fs::read_to_string("checklists/boeing_737_800.txt").unwrap();
        let section: Vec<&str> = text
            .lines()
            .skip_while(|l| !l.starts_with("sw_checklist:AFTER LANDING"))
            .take_while(|l| !l.starts_with("sw_remark:AFTER LANDING COMPLETE"))
            .collect();
        assert!(section.iter().any(|l| l.contains("landing_lights_on:0")), "{section:?}");
        assert!(!section.iter().any(|l| l.contains("taxi_light_on")), "{section:?}");
        let lp = |key: &str| crate::aircraft::LightProfile {
            key: key.into(),
            spoken: key.into(),
            dataref: format!("sim/test/{key}"),
            command_on: None,
            command_off: None,
            also_datarefs: Vec::new(),
            toggles: Vec::new(),
            auto_at_night: false,
        };
        let both = crate::tools::lights::Exterior {
            landing: Some(lp("landing")),
            taxi: Some(lp("taxi")),
        };
        for dark in [false, true] {
            let (landing, _) = crate::tools::lights::after_vacate_for(&both, dark);
            assert_eq!(landing, Some(false));
        }
    }

    /// An impact is no landing, and a wreck runs no list (the S-76 at LILB,
    /// 2026-09-24, would have been offered its after-landing checklist).
    #[test]
    fn a_crash_runs_no_after_landing_list() {
        let mut r = Rig::new();
        r.fly(5, true, 0.0, 0.0).fly(8, true, 0.0, 50.0).fly(60, false, 900.0, 70.0);
        // Straight in at 2,900 fpm.
        r.fly(20, false, -2900.0, 60.0).fly(30, true, 0.0, 2.0);
        assert!(!r.names().contains(&"after landing"), "{:?}", r.fired);
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
        let lease = RUN.begin().unwrap();
        *lock(&WAITING) = None;
        *lock(&REPLY) = None;
        // Running, waiting on nothing: a stray "yes" is not consumed, "stop" is.
        assert_eq!(take_reply("yes"), None);
        assert_eq!(take_reply("stop the checklist"), Some(Reply::Abort));
        assert!(stopped());
        drop(lease);
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

    /// Walk a Crew list's item boundaries against a recorded timeline of
    /// who had the aircraft; the items the list would have written, and the
    /// stand-down (when, by whom).
    fn crew_items_written(
        boundaries: &[(f64, &'static str)],
        owner_at: impl Fn(f64) -> Option<&'static str>,
        start_t: f64,
    ) -> (Vec<&'static str>, Option<(f64, &'static str)>) {
        let mut watch = CrewWatch::new(owner_at(start_t));
        let mut written = Vec::new();
        for &(t, item) in boundaries {
            if let Some(owner) = watch.step(owner_at(t)) {
                return (written, Some((t, owner)));
            }
            written.push(item);
        }
        (written, None)
    }

    #[test]
    fn a_mission_start_stands_the_after_landing_list_down() {
        // Sortie 1790586620 (LEMG, 2026-09-28): the After landing list began
        // as Crew at 593.0 with nobody flying; full auto started at 620.3
        // and gave up at 664.2. Each item was read and written just before
        // its line was spoken (the runner blocks on its own speech): the
        // narration at 623.2, "Speed brake, down." at 640.0 and "Flaps, up."
        // at 666.0, over the takeoff flaps full auto set at 620.6.
        let full_auto = |t: f64| (620.3..664.2).contains(&t).then_some("full auto");
        let boundaries = [
            (593.0, "After landing."),
            (623.2, "Speed brake, down."),
            (640.0, "Flaps, up."),
            (666.0, "Landing lights, off."),
            (679.8, "Strobe lights, off."),
        ];
        let (written, stood) = crew_items_written(&boundaries, full_auto, 593.0);
        assert_eq!(written, vec!["After landing."]);
        assert_eq!(stood, Some((623.2, "full auto")));
        // The give-up at 664.2 does not revive it: the stand-down aborts the
        // runner, which never reads the watch again.

        // Sortie 1790585651: began at 628.2, full auto from 651.1 on; "Speed
        // brake, down." at 680.4, "Flaps, up." at 682.9.
        let full_auto = |t: f64| (t >= 651.1).then_some("full auto");
        let boundaries = [
            (628.2, "After landing."),
            (663.7, "Speed brake, down."),
            (680.4, "Flaps, up."),
            (682.9, "Landing lights, off."),
        ];
        let (written, stood) = crew_items_written(&boundaries, full_auto, 628.2);
        assert_eq!(written, vec!["After landing."]);
        assert_eq!(stood, Some((663.7, "full auto")));
    }

    #[test]
    fn a_list_asked_for_under_a_mission_runs_and_names_its_items_held() {
        // The mission was flying when the list began: no stand-down...
        let mut w = CrewWatch::new(Some("full auto"));
        assert_eq!(w.step(Some("full auto")), None);
        // ...but one that ends and another that starts is a new taker.
        assert_eq!(w.step(None), None);
        assert_eq!(w.step(Some("the circuit")), Some("the circuit"));
        // Its items not in state are the mission's, never written.
        let flaps = act_item("Flaps, up.", Cond::Eq(0.0));
        assert_eq!(
            plan(Flavour::Crew, &flaps, Some(1.0), Some("full auto")),
            Step::Held("Flaps, up. Full auto has it.".into())
        );
        // In state it is only confirmed.
        assert_eq!(
            plan(Flavour::Crew, &flaps, Some(0.0), Some("full auto")),
            Step::Say("Flaps, up.".into())
        );
    }

    #[test]
    fn a_new_flight_is_the_init_counter_moving() {
        let mut w = InitWatch::default();
        // A failed read is no baseline and no new flight.
        assert!(!w.step(None));
        assert!(!w.step(Some(4.0)));
        assert!(!w.step(None));
        assert!(!w.step(Some(4.0)));
        assert!(w.step(Some(5.0)));
    }

    #[test]
    fn stand_down_by_cause_and_flavour() {
        let _g = crate::mode::TEST_LOCK
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        // Nothing running: nothing to stand down.
        ACTIVE.store(false, SeqCst);
        *lock(&RUNNING) = None;
        assert!(!stand_down(StandDown::Mission("full auto")));
        // A Monitor list is the mission's own audit: it goes on...
        ACTIVE.store(true, SeqCst);
        let lease = RUN.begin().unwrap();
        *lock(&RUNNING) = Some(Flavour::Monitor);
        *lock(&STOOD_DOWN) = None;
        assert!(!stand_down(StandDown::Mission("full auto")));
        assert!(!stopped());
        // ...but not into a new flight.
        assert!(stand_down(StandDown::NewFlight("setup")));
        assert!(stopped());
        drop(lease);
        *lock(&STOOD_DOWN) = None;
        // A Crew list stops for the mission, and the first cause is kept.
        *lock(&RUNNING) = Some(Flavour::Crew);
        assert!(stand_down(StandDown::Mission("full auto")));
        assert!(stand_down(StandDown::NewFlight("reposition")));
        assert_eq!(*lock(&STOOD_DOWN), Some(StandDown::Mission("full auto")));
        assert_eq!(
            stood_down_line("After landing", StandDown::Mission("full auto")),
            "After landing checklist stopped, full auto has the aircraft."
        );
        ACTIVE.store(false, SeqCst);
        *lock(&RUNNING) = None;
        *lock(&STOOD_DOWN) = None;
    }
}
