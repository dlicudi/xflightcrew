//! The unattended sortie runner: `xvl sortie <spec.yaml> [--repeat N]`.
//!
//! A landing test took a person at the keyboard for every step (LEMG,
//! 2026-09-26: reposition, arm, watch, read the verdict). The runner is that
//! person: a second process beside a LIVE X-Plane and a LIVE xvl, talking to
//! the sim's Web API and to the app through the plugin's mailboxes, never the
//! STT or the sim's SDK. Per case it sets the flight up, types the mission
//! into the panel's typed line (`xcopilot/typed`, the same queue a pilot's
//! Enter feeds), follows the blackbox the app writes, and takes the verdict
//! the live scorer concludes (`sortie_result`, `src/scorecard.rs`). One row
//! per case, printed as it goes and written to `logs/sorties/run-<stamp>.md`;
//! the exit status is non-zero when any case failed.
//!
//! The pure parts (the spec, the case state machine over blackbox records,
//! the run table, the blackbox tail) are plain `cargo test`; the driver runs
//! behind [`RunnerSim`] so it never needs a sim to be tested.

use anyhow::{anyhow, Result};
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use crate::tools::setup::{InitMailbox, InitOutcome, InitTiming};

const TYPED: &str = "xcopilot/typed";
const STATUS_AUTO: &str = "xcopilot/status/auto";
const SIM_SPEED: &str = "sim/time/sim_speed";
const ACF_PATH: &str = "sim/aircraft/view/acf_relative_path";
const ACF_ICAO: &str = "sim/aircraft/view/acf_ICAO";
const GROUNDSPEED: &str = "sim/flightmodel/position/groundspeed";
const PAUSE_ON: &str = "sim/operation/pause_on";

/// A case with no deadline of its own ends failed after this long.
pub const DEFAULT_DEADLINE_MIN: f64 = 20.0;
/// The app answers a typed command (a `tool` record or a crew line) within this.
pub const ACK_S: f64 = 5.0;
/// A line the app's poller has TAKEN (blackbox `typed` `taken`) is on the
/// command channel, not lost: it is waited for this long to be read, never
/// typed again. LEMG (2026-09-28, sortie 1790586620): "fly a circuit" was
/// taken at +0.0, retyped at +5.8 as unanswered, failed at +11 as "the app
/// did not take the command", and then both copies were read at +11.7 (the
/// command loop was held behind a tower reply): the mission flew
/// unobserved, and the second copy was answered "I'm already running".
pub const QUEUED_S: f64 = 60.0;
/// From the command's answer to the mission's `sortie` begin: an approach
/// setup waits on the sim's reload (up to two minutes) before it arms.
pub const SORTIE_GRACE_S: f64 = 180.0;
/// After a ramp or runway start is still, the app's profile detection and
/// the systems survey get this long before the command.
pub const DEFAULT_SETTLE_S: f64 = 10.0;
/// The words that take every loop off the aircraft.
pub const ABORT_LINE: &str = "my controls";

// ---- The spec ----------------------------------------------------------------

/// The spec file: a list of cases.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Spec {
    pub cases: Vec<Case>,
    /// Every case's [`After`] unless it names its own; `park` when silent.
    #[serde(default)]
    pub after: Option<After>,
}

/// What the runner does with a case's aircraft once its verdict is in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Default)]
#[serde(rename_all = "lowercase")]
pub enum After {
    /// Leave its taxi-in to finish by itself before the next setup (up to
    /// [`STOP_MAX_S`]): the case is about the taxi in, the vacate or the stand.
    #[default]
    Park,
    /// On the ground, "my controls" at once (`hand_back_hold` brakes it to a
    /// stop and parks it): the taxi in is not what the case tests. A full-auto
    /// landing's verdict comes at its `taxi_in` stage, and the vacate and the
    /// drive to the stand cost 124 to 309 s before every next setup (LEMG,
    /// 2026-09-28). Airborne, as `park`: the loops keep it.
    Stop,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(try_from = "CaseRaw")]
pub struct Case {
    pub name: String,
    pub setup: Setup,
    /// The aircraft the setup loads (`setup.aircraft`, an `.acf` path
    /// relative to the X-Plane root, or [`AIRCRAFT_FROM_PLAN`]); `None`
    /// keeps the loaded one.
    pub aircraft: Option<String>,
    /// The mission, typed as a pilot would ("full auto", "fly the flight
    /// plan"). An approach case has none: the setup itself arms autoland.
    /// Sugar for a first step `when: start`.
    pub command: Option<String>,
    /// The lines typed AFTER the opening one (the command, or the approach
    /// setup's own words), each when its trigger fires.
    pub steps: Vec<ScriptStep>,
    pub deadline_min: Option<f64>,
    /// X-Plane's time multiplier the spec wants from the mission's sortie on
    /// (`sim/time/sim_speed`), flown only where [`SpeedGuard`] allows and
    /// put back to 1 at the case's end. Speed steps change it later.
    pub sim_speed: Option<u32>,
    /// Seconds after a ramp or runway start is still before the command.
    pub settle_s: Option<f64>,
    /// The case is MEANT to end in this give-up (a rejected takeoff ends
    /// full auto with `takeoff_not_completed`): that outcome passes, with
    /// every other rule of the verdict standing.
    pub expect_give_up: Option<String>,
    /// See [`After`]; the spec's when `None`.
    pub after: Option<After>,
}

/// `setup.aircraft: plan`: the SimBrief plan's type, the installed `.acf`
/// the set-up tool would load for it (`setup::find_acf`).
pub const AIRCRAFT_FROM_PLAN: &str = "plan";
/// `field: plan_origin` / `field: plan_destination`: the pilot's SimBrief
/// plan's ends, fetched once per run through the app's own
/// `simbrief::fetch` (the id from `XCOPILOT_SIMBRIEF_ID`).
pub const FIELD_PLAN_ORIGIN: &str = "PLAN_ORIGIN";
pub const FIELD_PLAN_DESTINATION: &str = "PLAN_DESTINATION";
/// `runway: plan`: the plan's runway at that end (`origin/plan_rwy`,
/// `destination/plan_rwy`).
pub const RUNWAY_FROM_PLAN: &str = "PLAN";
/// `runway: wind`: the into-wind end of the field's runways, picked at the
/// case's setup by the tower's own rule (`nav::best_runway_for_wind`) from
/// the field's surface wind at that moment. LEMG (2026-09-28, #158): the
/// backlog's cases named 13 with the wind at 280/5, the tower sent the
/// arrivals to 30, and a slow SR22 spent its whole deadline reversing.
pub const RUNWAY_FROM_WIND: &str = "WIND";

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CaseRaw {
    name: String,
    setup: SetupMap,
    #[serde(default)]
    command: Option<String>,
    #[serde(default)]
    steps: Vec<StepRaw>,
    #[serde(default)]
    deadline_min: Option<f64>,
    #[serde(default)]
    sim_speed: Option<u32>,
    #[serde(default)]
    settle_s: Option<f64>,
    #[serde(default)]
    expect_give_up: Option<String>,
    #[serde(default)]
    after: Option<After>,
}

impl TryFrom<CaseRaw> for Case {
    type Error = String;
    fn try_from(c: CaseRaw) -> std::result::Result<Self, String> {
        let aircraft = c
            .setup
            .aircraft
            .clone()
            .map(|a| a.trim().to_string())
            .filter(|a| !a.is_empty());
        let setup = Setup::try_from(c.setup)?;
        let steps = c
            .steps
            .into_iter()
            .map(ScriptStep::try_from)
            .collect::<std::result::Result<Vec<_>, _>>()?;
        Ok(Case {
            name: c.name,
            setup,
            aircraft,
            command: c.command,
            steps,
            deadline_min: c.deadline_min,
            sim_speed: c.sim_speed,
            settle_s: c.settle_s,
            expect_give_up: c.expect_give_up,
            after: c.after,
        })
    }
}

// ---- The script: lines typed after the opening one ------------------------------

/// When a scripted line is typed. Every clock and watch starts at the
/// previous line (the first later step's at the opening line's answer, or
/// on an approach at its sortie's begin: the reload is not the flight).
#[derive(Debug, Clone, PartialEq)]
pub enum Trigger {
    /// The opening line, right after the setup (only the first step).
    Start,
    /// Seconds since the previous line.
    AfterS(f64),
    /// Indicated airspeed above this, knots (read from the sim).
    IasAbove(f64),
    /// Height above the ground above this, feet (read from the sim).
    AglAbove(f64),
    /// The flight model's phase (`flight::Phase`, as the `state` record's
    /// `fphase` and the `flight` `phase` event's `to` spell it).
    Phase(String),
    /// A blackbox record of this kind (and event) appears.
    Event { kind: String, event: Option<String> },
}

/// The flight model's phases, as `format!("{:?}")` writes them (pinned
/// against `flight::Phase` by a test).
pub const PHASES: &[&str] = &[
    "Parked",
    "EngineStart",
    "Ready",
    "Pushback",
    "TaxiOut",
    "Holding",
    "LinedUp",
    "TakeoffRoll",
    "RejectedTakeoff",
    "InitialClimb",
    "Climb",
    "Cruise",
    "Descent",
    "Approach",
    "Landing",
    "GoAround",
    "Hover",
    "Rollout",
    "TaxiIn",
];

/// One scripted step: a typed line (`say`), or the time multiplier the
/// spec wants from here on (`sim_speed`), never both. A speed step types
/// nothing: the runner's [`SpeedGuard`] applies it only while the flight is
/// one that tolerates it.
#[derive(Debug, Clone, PartialEq)]
pub struct ScriptStep {
    /// The typed line, as a pilot says it; empty on a speed step.
    pub say: String,
    pub when: Trigger,
    /// The tool the line must route to (a test pins it against the grammar;
    /// `plan_flight` is the local planner's branch in the command loop).
    pub tool: Option<String>,
    /// The wanted `sim/time/sim_speed`, 1 to [`MAX_SIM_SPEED`].
    pub sim_speed: Option<u32>,
}

impl ScriptStep {
    /// A typed line (not a speed step).
    pub fn is_say(&self) -> bool {
        self.sim_speed.is_none()
    }

    /// The step in words, for a row.
    pub fn words(&self) -> String {
        match self.sim_speed {
            Some(n) => format!("sim_speed {n}"),
            None => format!("{:?}", self.say),
        }
    }
}

/// The highest `sim_speed` a spec may ask for (X-Plane's own top).
pub const MAX_SIM_SPEED: u32 = 16;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct StepRaw {
    #[serde(default)]
    say: Option<String>,
    #[serde(default)]
    sim_speed: Option<i64>,
    #[serde(default)]
    when: Option<serde_yaml::Value>,
    #[serde(default)]
    tool: Option<String>,
}

impl TryFrom<StepRaw> for ScriptStep {
    type Error = String;
    fn try_from(s: StepRaw) -> std::result::Result<Self, String> {
        let say = s.say.as_deref().unwrap_or("").trim().to_string();
        let sim_speed = match (s.say.is_some(), s.sim_speed) {
            (true, Some(_)) => return Err(format!("step {say:?}: say or sim_speed, not both")),
            (false, None) => return Err("a step needs a say (or a sim_speed)".into()),
            (true, None) if say.is_empty() => return Err("a step needs a say".into()),
            (true, None) => None,
            (false, Some(n)) => {
                if !(1..=i64::from(MAX_SIM_SPEED)).contains(&n) {
                    return Err(format!(
                        "step sim_speed {n}: sim_speed is 1 to {MAX_SIM_SPEED}"
                    ));
                }
                if s.tool.is_some() {
                    return Err(format!(
                        "step sim_speed {n}: a speed step routes to no tool"
                    ));
                }
                Some(n as u32)
            }
        };
        let name = match sim_speed {
            Some(n) => format!("sim_speed {n}"),
            None => format!("{say:?}"),
        };
        let when = match s.when {
            None => return Err(format!("step {name} needs a when")),
            Some(v) => trigger(&v).map_err(|e| format!("step {name}: {e}"))?,
        };
        if sim_speed.is_some() && when == Trigger::Start {
            return Err(format!(
                "step {name}: a speed step is never the start (the case's sim_speed is)"
            ));
        }
        Ok(ScriptStep {
            say,
            when,
            tool: s.tool,
            sim_speed,
        })
    }
}

/// `start`, or a map with exactly one of `after_s`, `ias_above`,
/// `agl_above`, `phase`, `event: {kind, event}`.
fn trigger(v: &serde_yaml::Value) -> std::result::Result<Trigger, String> {
    use serde_yaml::Value as Y;
    const WHICH: &str =
        "when is start, or one of after_s, ias_above, agl_above, phase, event: {kind, event}";
    if v.as_str() == Some("start") {
        return Ok(Trigger::Start);
    }
    let Some(m) = v.as_mapping() else {
        return Err(WHICH.into());
    };
    if m.len() != 1 {
        return Err(WHICH.into());
    }
    let (k, x) = m.iter().next().ok_or(WHICH)?;
    let num = |what: &str| -> std::result::Result<f64, String> {
        x.as_f64()
            .filter(|n| n.is_finite() && *n >= 0.0)
            .ok_or_else(|| format!("{what} needs a number, zero or more"))
    };
    match k.as_str().unwrap_or("") {
        "after_s" => Ok(Trigger::AfterS(num("after_s")?)),
        "ias_above" => Ok(Trigger::IasAbove(num("ias_above")?)),
        "agl_above" => Ok(Trigger::AglAbove(num("agl_above")?)),
        "phase" => {
            let p = x.as_str().unwrap_or("").trim();
            PHASES
                .iter()
                .find(|n| n.eq_ignore_ascii_case(p))
                .map(|n| Trigger::Phase(n.to_string()))
                .ok_or_else(|| format!("phase {p:?} is none of {}", PHASES.join(", ")))
        }
        "event" => {
            let m = x.as_mapping().ok_or("event is {kind, event}")?;
            let mut kind = None;
            let mut event = None;
            for (k, v) in m {
                match (k.as_str(), v) {
                    (Some("kind"), Y::String(s)) => kind = Some(s.trim().to_string()),
                    (Some("event"), Y::String(s)) => event = Some(s.trim().to_string()),
                    _ => return Err("event is {kind, event}".into()),
                }
            }
            let kind = kind.filter(|k| !k.is_empty()).ok_or("event needs a kind")?;
            Ok(Trigger::Event {
                kind,
                event: event.filter(|e| !e.is_empty()),
            })
        }
        _ => Err(WHICH.into()),
    }
}

/// The later steps of a case, fired in order off the blackbox and the sim.
/// Pure: the driver feeds it records and readings and types what it says.
#[derive(Debug, Clone, Default)]
pub struct Script {
    steps: Vec<ScriptStep>,
    next: usize,
    /// The previous line's time (the arm's, for the first): `None` until armed.
    since: Option<f64>,
    /// The flight model's phase, last seen.
    phase: Option<String>,
    /// The next step's event has appeared since the previous line.
    event_seen: bool,
}

impl Script {
    pub fn new(steps: Vec<ScriptStep>) -> Self {
        Self {
            steps,
            ..Self::default()
        }
    }

    /// The clocks start: the opening line was answered. Idempotent.
    pub fn arm(&mut self, now: f64) {
        self.since.get_or_insert(now);
    }

    pub fn armed(&self) -> bool {
        self.since.is_some()
    }

    /// Steps not yet typed.
    pub fn pending(&self) -> usize {
        self.steps.len() - self.next
    }

    /// The step due next, if any.
    pub fn next_step(&self) -> Option<&ScriptStep> {
        self.steps.get(self.next)
    }

    /// The steps not yet taken, the next first.
    pub fn remaining(&self) -> &[ScriptStep] {
        &self.steps[self.next.min(self.steps.len())..]
    }

    /// Typed lines not yet typed (speed steps type nothing).
    pub fn pending_says(&self) -> usize {
        self.remaining().iter().filter(|s| s.is_say()).count()
    }

    /// One blackbox record, in the order the app wrote it.
    pub fn feed(&mut self, r: &Value) {
        if let Some(p) = phase_of(r) {
            self.phase = Some(p.to_string());
        }
        if !self.armed() {
            return;
        }
        if let Some(ScriptStep {
            when: Trigger::Event { kind, event },
            ..
        }) = self.steps.get(self.next)
        {
            if st(r, "kind") == kind && event.as_deref().is_none_or(|e| st(r, "event") == e) {
                self.event_seen = true;
            }
        }
    }

    /// Whether the next step is due now: `ias_kts` and `agl_ft` are the
    /// sim's (`None`: unreadable, no measurement, never due on it).
    pub fn due(&self, now: f64, ias_kts: Option<f64>, agl_ft: Option<f64>) -> bool {
        let (Some(since), Some(s)) = (self.since, self.steps.get(self.next)) else {
            return false;
        };
        match &s.when {
            Trigger::Start => true,
            Trigger::AfterS(n) => now - since >= *n,
            Trigger::IasAbove(k) => ias_kts.is_some_and(|v| v > *k),
            Trigger::AglAbove(h) => agl_ft.is_some_and(|v| v > *h),
            Trigger::Phase(p) => self.phase.as_deref() == Some(p.as_str()),
            Trigger::Event { .. } => self.event_seen,
        }
    }

    /// Which sim reads the next step's trigger needs: (ias, agl).
    pub fn wants_reads(&self) -> (bool, bool) {
        match self.steps.get(self.next).map(|s| &s.when) {
            Some(Trigger::IasAbove(_)) => (true, false),
            Some(Trigger::AglAbove(_)) => (false, true),
            _ => (false, false),
        }
    }

    /// The next step was typed at `now`.
    pub fn sent(&mut self, now: f64) {
        self.next += 1;
        self.since = Some(now);
        self.event_seen = false;
    }
}

/// The flight model's phase a record carries: the `state` record's
/// `fphase`, or a `flight` `phase` event's `to`.
fn phase_of(r: &Value) -> Option<&str> {
    match st(r, "kind") {
        "state" if !st(r, "fphase").is_empty() => Some(st(r, "fphase")),
        "flight" if st(r, "ev") == "phase" && !st(r, "to").is_empty() => Some(st(r, "to")),
        _ => None,
    }
}

// ---- Fast forward: a time multiplier only where the flight tolerates it ------------

/// The flight model's phases a fast sim may fly (the autopilot's enroute
/// legs, where the follower only moves the bug and the dials now and then).
pub const FAST_PHASES: &[&str] = &["Climb", "Cruise", "Descent"];
/// Below this height above the ground the sim runs at 1x.
pub const FAST_FLOOR_AGL_FT: f64 = 5_000.0;
/// Every condition holds this long before a fast speed is (re)applied: a
/// height or a phase flickering at its edge never flip-flops the sim.
pub const FAST_RESUME_S: f64 = 10.0;
/// What the sim achieved after its frame-rate limiting (read-only).
const SIM_SPEED_ACTUAL: &str = "sim/time/sim_speed_actual";

/// Status lines (`xcopilot/status/auto`, lower-cased, the headline's
/// `NAME|stage` included) of the loops that fly close to the ground or to
/// the aircraft on wall-clock ticks: at 4x their gains and delays are four
/// times what they were tuned for.
const SLOW_LOOPS: &[&str] = &[
    "autoland",
    "recovering",
    "recovery",
    "talk-down",
    "auto circuit",
    "heli circuit",
    "route",
    "auto takeoff",
    "auto taxi",
    "pushback",
    "air taxi",
    "hover ",
    "setting down",
];

/// The first loop on the panel's status that must fly at 1x, as its line
/// says it; `None` when none is. The hover's own flying line ("flying 40
/// kt, 500 ft") is one; the follower's ("flying a procedure") is not.
pub fn slow_loop(status: &str) -> Option<String> {
    let body = status.get(1..).unwrap_or("");
    body.lines().map(str::trim).find_map(|l| {
        let low = l.to_ascii_lowercase();
        let hover_fly = low
            .strip_prefix("flying ")
            .is_some_and(|r| r.starts_with(|c: char| c.is_ascii_digit()));
        (hover_fly || SLOW_LOOPS.iter().any(|p| low.starts_with(p))).then(|| l.to_string())
    })
}

/// One poll's measurements for the guard; `None` is a failed read, which
/// is no measurement and never lets the sim run fast.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FastReads {
    pub crashed: Option<bool>,
    pub on_ground: Option<bool>,
    pub agl_ft: Option<f64>,
    pub servos_on: Option<bool>,
    /// `xcopilot/status/auto`.
    pub status: Option<String>,
}

/// What the driver does with the sim's time multiplier this poll.
#[derive(Debug, Clone, PartialEq)]
pub enum SpeedAct {
    Keep,
    /// Write this multiplier and read it back.
    Write {
        to: u32,
        reason: String,
    },
    /// A fast speed is wanted and withheld: why (once per reason).
    Held {
        reason: String,
    },
}

/// The runner's hard guard over `sim/time/sim_speed`, which a spec cannot
/// override. The control laws tick on WALL time over REST at a few Hz: at
/// 4x the plant moves four times as far per tick, so taxi steering, the
/// roll, the flare, pushback and the hover would ring or crash, and every
/// "held N s" watchdog is a quarter as long in sim time. A fast speed is
/// flown only while ALL hold: a sortie flying, no wreck, airborne, the
/// flight model in [`FAST_PHASES`], above [`FAST_FLOOR_AGL_FT`], the
/// autopilot's servos on, no [`slow_loop`] on the panel, and in this sortie
/// no give-up, intervention or pilot's command (those latch it off until the
/// next sortie). The moment one fails it is 1x with the reason; the spec's
/// speed returns only after all have held [`FAST_RESUME_S`]. Pure: the
/// driver feeds it records and reads and writes what it says.
#[derive(Debug, Clone)]
pub struct SpeedGuard {
    /// The spec's speed (the case's `sim_speed`, then its speed steps).
    wanted: u32,
    /// The sim's multiplier as last read back; `None`: unknown, restored.
    set: Option<u32>,
    phase: Option<String>,
    sortie: Option<String>,
    /// Why fast is off for the rest of this sortie.
    latched: Option<String>,
    ok_since: Option<f64>,
    held: Option<String>,
    last_t: Option<f64>,
    /// Wall seconds flown fast.
    pub fast_s: f64,
    /// Wall seconds of those with `sim_speed_actual` read, and the sim
    /// seconds they flew.
    measured_s: f64,
    sim_s: f64,
    /// [`FAST_RESUME_S`] live.
    resume_s: f64,
}

impl SpeedGuard {
    /// The case's own `sim_speed` is the first wanted speed; the sim is at
    /// 1x (the runner restores it before every setup).
    pub fn new(wanted: Option<u32>, resume_s: f64) -> Self {
        Self {
            resume_s,
            wanted: wanted.unwrap_or(1).max(1),
            set: Some(1),
            phase: None,
            sortie: None,
            latched: None,
            ok_since: None,
            held: None,
            last_t: None,
            fast_s: 0.0,
            measured_s: 0.0,
            sim_s: 0.0,
        }
    }

    /// A speed step fired.
    pub fn want(&mut self, n: u32) {
        self.wanted = n.max(1);
        self.held = None;
    }

    pub fn wanted(&self) -> u32 {
        self.wanted
    }

    /// The sim's multiplier as last read back (`None`: unknown).
    pub fn set(&self) -> Option<u32> {
        self.set
    }

    /// The sim is (or may be) running fast: a restore is owed.
    pub fn owes_restore(&self) -> bool {
        self.set != Some(1)
    }

    /// The guard needs the sim's reads this poll.
    pub fn needs_reads(&self) -> bool {
        self.wanted > 1 || self.owes_restore()
    }

    /// The mean multiplier the sim achieved while fast (`None`: never read).
    pub fn achieved(&self) -> Option<f64> {
        (self.measured_s > 0.0).then(|| self.sim_s / self.measured_s)
    }

    /// One blackbox record, in the order the app wrote it.
    pub fn feed(&mut self, r: &Value) {
        if let Some(p) = phase_of(r) {
            self.phase = Some(p.to_string());
        }
        let (kind, event) = (st(r, "kind"), st(r, "event"));
        match (kind, event) {
            ("sortie", "begin") => {
                self.sortie = Some(st(r, "sortie").to_string());
                self.latched = None;
                return;
            }
            ("sortie_result", _) if self.sortie.as_deref() == Some(st(r, "sortie")) => {
                self.sortie = None;
                return;
            }
            _ => {}
        }
        if self.sortie.is_none() || self.latched.is_some() {
            return;
        }
        self.latched = match (kind, event) {
            ("intervention", _) => Some(format!("an intervention ({})", st(r, "what"))),
            ("tool", _) => Some(format!("a pilot's command ({})", st(r, "name"))),
            ("full_auto" | "heli_route", "give_up") => {
                Some(format!("a give-up ({})", st(r, "reason")))
            }
            ("full_auto", _) if st(r, "stage") == "give_up" => Some("a give-up".into()),
            ("flight", _)
                if st(r, "ev") == "impact"
                    || (st(r, "ev") == "abnormal" && st(r, "to") == "Crashed") =>
            {
                Some("an impact".into())
            }
            _ => None,
        };
    }

    /// Why the sim may not run fast now; `None`: every condition holds.
    pub fn blocker(&self, x: &FastReads) -> Option<String> {
        if let Some(l) = &self.latched {
            return Some(format!("{l} in this sortie"));
        }
        if self.sortie.is_none() {
            return Some("no sortie flying".into());
        }
        match x.crashed {
            Some(false) => {}
            Some(true) => return Some("crashed".into()),
            None => return Some("the wreck flag unreadable".into()),
        }
        match x.on_ground {
            Some(false) => {}
            Some(true) => return Some("on the ground".into()),
            None => return Some("the wheels unreadable".into()),
        }
        match self.phase.as_deref() {
            Some(p) if FAST_PHASES.contains(&p) => {}
            Some(p) => return Some(format!("phase {p}")),
            None => return Some("no flight phase yet".into()),
        }
        match x.agl_ft {
            Some(h) if h >= FAST_FLOOR_AGL_FT => {}
            Some(h) => return Some(format!("{h:.0} ft AGL, under {FAST_FLOOR_AGL_FT:.0}")),
            None => return Some("the height unreadable".into()),
        }
        match x.servos_on {
            Some(true) => {}
            Some(false) => return Some("the autopilot servos off".into()),
            None => return Some("the servos unreadable".into()),
        }
        match &x.status {
            Some(s) => slow_loop(s).map(|l| format!("{l} flying")),
            None => Some("the loop status unreadable".into()),
        }
    }

    /// One poll: what to do with the sim's multiplier.
    pub fn tick(&mut self, now: f64, x: &FastReads) -> SpeedAct {
        let fast_now = self.set.is_some_and(|s| s > 1);
        if self.wanted <= 1 {
            self.ok_since = None;
            return match self.owes_restore() {
                true => SpeedAct::Write {
                    to: 1,
                    reason: "the spec wants 1x".into(),
                },
                false => SpeedAct::Keep,
            };
        }
        if let Some(why) = self.blocker(x) {
            self.ok_since = None;
            if self.owes_restore() {
                self.held = Some(why.clone());
                return SpeedAct::Write { to: 1, reason: why };
            }
            if self.held.as_deref() != Some(why.as_str()) {
                self.held = Some(why.clone());
                return SpeedAct::Held { reason: why };
            }
            return SpeedAct::Keep;
        }
        self.held = None;
        let since = *self.ok_since.get_or_insert(now);
        if self.set == Some(self.wanted) {
            return SpeedAct::Keep;
        }
        // Already fast (another fast speed asked): at once; from 1x, after
        // the conditions have held.
        if fast_now || now - since >= self.resume_s {
            return SpeedAct::Write {
                to: self.wanted,
                reason: "every condition holds".into(),
            };
        }
        SpeedAct::Keep
    }

    /// The multiplier `to` was written and read back as `read`. A fast
    /// write that did not take latches fast off for the sortie (never
    /// written again each poll); an unread one is unknown, so restored.
    pub fn written(&mut self, to: u32, read: Option<f64>) {
        let got = read
            .filter(|v| v.is_finite())
            .map(|v| v.round().max(0.0) as u32);
        self.set = got;
        if to > 1 && got != Some(to) {
            self.latched = Some(format!(
                "sim_speed {to} read back {}",
                read.map_or("nothing".to_string(), |v| format!("{v}"))
            ));
        }
    }

    /// The clock, every poll: wall seconds flown fast, and the sim seconds
    /// `sim_speed_actual` says they were.
    pub fn accrue(&mut self, now: f64, actual: Option<f64>) {
        if let (Some(t), Some(s)) = (self.last_t, self.set) {
            let dt = (now - t).max(0.0);
            if s > 1 {
                self.fast_s += dt;
                if let Some(a) = actual.filter(|a| a.is_finite() && *a > 0.0) {
                    self.measured_s += dt;
                    self.sim_s += dt * a;
                }
            }
        }
        self.last_t = Some(now);
    }
}

/// Write the multiplier, read it back and what the sim achieved, tell the
/// guard, and blackbox it (`sortie_runner` `sim_speed`).
async fn apply_sim_speed(
    sim: &dyn RunnerSim,
    guard: &mut SpeedGuard,
    to: u32,
    reason: &str,
    case: &str,
) {
    let wrote = sim.write_float(SIM_SPEED, f64::from(to)).await;
    let (read, actual) = tokio::join!(sim.read(SIM_SPEED), sim.read(SIM_SPEED_ACTUAL));
    let read = read.ok();
    guard.written(to, read);
    record_sim_speed(
        guard.wanted(),
        Some(to),
        read,
        actual.ok(),
        reason,
        case,
        wrote.err().map(|e| e.to_string()),
    );
    eprintln!(
        "{case}: sim_speed {to} ({reason}), read back {}",
        read.map_or("nothing".to_string(), |v| format!("{v}"))
    );
}

fn record_sim_speed(
    wanted: u32,
    set: Option<u32>,
    read: Option<f64>,
    actual: Option<f64>,
    reason: &str,
    case: &str,
    error: Option<String>,
) {
    crate::blackbox::record(
        "sortie_runner",
        json!({
            "event": "sim_speed",
            "case": case,
            "wanted": wanted,
            "set": set,
            "read_back": read,
            "actual": actual,
            "reason": reason,
            "error": error,
        }),
    );
}

/// The sim at 1x, read first: before a setup, at a run's end, on Ctrl-C.
/// An unreadable multiplier is written anyway. Returns whether it reads 1.
async fn ensure_real_time(sim: &dyn RunnerSim, reason: &str) -> bool {
    if sim.read(SIM_SPEED).await.ok() == Some(1.0) {
        return true;
    }
    let wrote = sim.write_float(SIM_SPEED, 1.0).await;
    let (read, actual) = tokio::join!(sim.read(SIM_SPEED), sim.read(SIM_SPEED_ACTUAL));
    let read = read.ok();
    record_sim_speed(
        1,
        Some(1),
        read,
        actual.ok(),
        reason,
        "",
        wrote.err().map(|e| e.to_string()),
    );
    read == Some(1.0)
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(try_from = "SetupMap")]
pub enum Setup {
    /// A new flight on a runway final, the loaded aircraft, engines running,
    /// autoland armed by the app on the result (`setup_approach`).
    Approach {
        field: String,
        runway: String,
        final_nm: i64,
    },
    /// The loaded aircraft on the runway's threshold.
    RunwayStart {
        field: String,
        runway: String,
        engines_running: bool,
    },
    /// The loaded aircraft at a stand, cold and dark (no stand named: the
    /// setup tool's own pick for the type).
    Ramp {
        field: String,
        stand: Option<String>,
    },
    /// A new flight IN THE AIR on the SimBrief plan at the point under test
    /// (`setup_enroute`, typed as the app's own words and waited for), the
    /// case's command ("fly the flight plan") then the airborne resume.
    /// `field` and `runway` are the plan's origin and its runway, where
    /// another aircraft is loaded first (engines running) before the setup.
    Enroute {
        field: String,
        runway: String,
        point: EnroutePoint,
    },
}

pub use crate::tools::setup_enroute::EnroutePoint;

/// `enroute: {at: ...}`: the points a case may start at.
pub const ENROUTE_AT: &[&str] = &["fix", "before_tod_nm", "before_destination_nm"];

/// The YAML shape: a map with exactly one of the three keys (serde_yaml
/// reads a plain enum only from `!tags`).
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct SetupMap {
    #[serde(default)]
    aircraft: Option<String>,
    #[serde(default)]
    approach: Option<ApproachSpec>,
    #[serde(default)]
    runway_start: Option<RunwayStartSpec>,
    #[serde(default)]
    ramp: Option<RampSpec>,
    #[serde(default)]
    enroute: Option<EnrouteSpec>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct EnrouteSpec {
    at: String,
    value: serde_yaml::Value,
}

impl EnrouteSpec {
    /// The point, or why the spec's words are not one.
    fn point(&self) -> std::result::Result<EnroutePoint, String> {
        let miles = |max: f64| match &self.value {
            serde_yaml::Value::Number(n) => n
                .as_f64()
                .filter(|v| *v >= 0.0 && *v <= max && v.fract() == 0.0)
                .ok_or_else(|| {
                    format!("enroute {} is a whole number of miles, 0 to {max:.0}", self.at)
                }),
            _ => Err(format!("enroute {} needs a number of miles", self.at)),
        };
        let max = crate::tools::setup_enroute::MAX_POINT_NM;
        match self.at.trim() {
            "fix" => match &self.value {
                serde_yaml::Value::String(f)
                    if (2..=7).contains(&f.trim().len())
                        && f.trim().chars().all(|c| c.is_ascii_alphanumeric())
                        && f.trim().chars().any(|c| c.is_ascii_alphabetic()) =>
                {
                    Ok(EnroutePoint::Fix(f.trim().to_ascii_uppercase()))
                }
                _ => Err(
                    "enroute fix names one fix of the plan (2 to 7 letters and digits)".into()
                ),
            },
            "before_tod_nm" => Ok(EnroutePoint::BeforeTod(miles(max)?)),
            "before_destination_nm" => {
                let n = miles(max)?;
                if n < 1.0 {
                    return Err("enroute before_destination_nm is at least 1".into());
                }
                Ok(EnroutePoint::BeforeDestination(n))
            }
            other => Err(format!("enroute at {other:?} is not one of {}", ENROUTE_AT.join(", "))),
        }
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ApproachSpec {
    #[serde(deserialize_with = "ident")]
    field: String,
    #[serde(deserialize_with = "ident")]
    runway: String,
    #[serde(default = "default_final")]
    final_nm: i64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RunwayStartSpec {
    #[serde(deserialize_with = "ident")]
    field: String,
    #[serde(deserialize_with = "ident")]
    runway: String,
    #[serde(default)]
    engines_running: bool,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RampSpec {
    #[serde(deserialize_with = "ident")]
    field: String,
    #[serde(default)]
    stand: Option<String>,
}

impl TryFrom<SetupMap> for Setup {
    type Error = String;
    fn try_from(m: SetupMap) -> std::result::Result<Self, String> {
        match (m.approach, m.runway_start, m.ramp, m.enroute) {
            (None, None, None, Some(e)) => Ok(Setup::Enroute {
                field: FIELD_PLAN_ORIGIN.into(),
                runway: RUNWAY_FROM_PLAN.into(),
                point: e.point()?,
            }),
            (Some(a), None, None, None) => Ok(Setup::Approach {
                field: a.field,
                runway: a.runway,
                final_nm: a.final_nm,
            }),
            (None, Some(r), None, None) => Ok(Setup::RunwayStart {
                field: r.field,
                runway: r.runway,
                engines_running: r.engines_running,
            }),
            (None, None, Some(r), None) => Ok(Setup::Ramp {
                field: r.field,
                stand: r.stand,
            }),
            _ => Err("setup is exactly one of approach, runway_start, ramp, enroute".into()),
        }
    }
}

fn default_final() -> i64 {
    crate::tools::setup::DEFAULT_FINAL_NM
}

/// A field or runway as YAML may spell it: `13` is a number to YAML.
fn ident<'de, D: serde::Deserializer<'de>>(d: D) -> std::result::Result<String, D::Error> {
    let v = serde_yaml::Value::deserialize(d)?;
    match v {
        serde_yaml::Value::String(s) => Ok(s.trim().to_ascii_uppercase()),
        serde_yaml::Value::Number(n) => Ok(n.to_string()),
        other => Err(serde::de::Error::custom(format!(
            "expected a name, got {other:?}"
        ))),
    }
}

/// The approach speed the default deadline is sized for: the 737's
/// (`profiles/boeing_737_800.yaml`), the airframe the default was set on.
pub const REF_APPROACH_KTS: f64 = 145.0;
/// A slow airframe's default deadline grows by at most this factor.
pub const DEADLINE_SCALE_MAX: f64 = 2.5;

/// The deadline of a case with none of its own: [`DEFAULT_DEADLINE_MIN`]
/// scaled by how much slower the airframe flies its approach than the
/// [`REF_APPROACH_KTS`] it was set for (every leg of a pattern and a final
/// is distance over speed), never shorter than the default. Unknown speed:
/// the default. The SR22 at LEMG (2026-09-28, #158) was still on final at
/// its deadline.
pub fn default_deadline_min(approach_kts: Option<f64>) -> f64 {
    let scale = approach_kts
        .filter(|k| k.is_finite() && *k > 0.0)
        .map_or(1.0, |k| (REF_APPROACH_KTS / k).clamp(1.0, DEADLINE_SCALE_MAX));
    DEFAULT_DEADLINE_MIN * scale
}

impl Case {
    /// What follows the verdict: the case's own, else [`After::Park`].
    pub fn after(&self) -> After {
        self.after.unwrap_or_default()
    }

    pub fn deadline_s(&self) -> f64 {
        self.deadline_s_for(None)
    }

    /// The case's deadline for an airframe that approaches at this speed:
    /// its own `deadline_min`, else [`default_deadline_min`].
    pub fn deadline_s_for(&self, approach_kts: Option<f64>) -> f64 {
        self.deadline_min
            .unwrap_or_else(|| default_deadline_min(approach_kts))
            * 60.0
    }

    /// The case's runway is the wind's, picked at its setup.
    pub fn wants_wind(&self) -> bool {
        match &self.setup {
            Setup::Approach { runway, .. } | Setup::RunwayStart { runway, .. } => {
                runway == RUNWAY_FROM_WIND
            }
            Setup::Ramp { .. } | Setup::Enroute { .. } => false,
        }
    }

    /// The case with its runway named.
    pub fn with_runway(&self, ident: &str) -> Case {
        let mut c = self.clone();
        if let (_, Some(r)) = c.field_mut() {
            *r = ident.to_string();
        }
        c
    }

    /// The line typed to start the case: the approach setup's own words, or
    /// the case's command (its first step `when: start`).
    pub fn first_line(&self) -> String {
        match &self.setup {
            Setup::Approach {
                field,
                runway,
                final_nm,
            } => approach_line(field, runway, *final_nm),
            _ => match &self.command {
                Some(c) => c.clone(),
                None => self
                    .steps
                    .first()
                    .map(|s| s.say.clone())
                    .unwrap_or_default(),
            },
        }
    }

    /// The steps typed after the opening line.
    pub fn later_steps(&self) -> Vec<ScriptStep> {
        let opening_is_a_step =
            !matches!(self.setup, Setup::Approach { .. }) && self.command.is_none();
        self.steps
            .iter()
            .skip(usize::from(opening_is_a_step))
            .cloned()
            .collect()
    }

    fn field_mut(&mut self) -> (&mut String, Option<&mut String>) {
        match &mut self.setup {
            Setup::Approach { field, runway, .. }
            | Setup::RunwayStart { field, runway, .. }
            | Setup::Enroute { field, runway, .. } => (field, Some(runway)),
            Setup::Ramp { field, .. } => (field, None),
        }
    }

    fn field(&self) -> &str {
        match &self.setup {
            Setup::Approach { field, .. }
            | Setup::RunwayStart { field, .. }
            | Setup::Ramp { field, .. }
            | Setup::Enroute { field, .. } => field,
        }
    }

    /// The case needs the pilot's SimBrief plan to be set up.
    pub fn uses_plan(&self) -> bool {
        matches!(self.field(), FIELD_PLAN_ORIGIN | FIELD_PLAN_DESTINATION)
            || self.aircraft.as_deref() == Some(AIRCRAFT_FROM_PLAN)
    }
}

/// What a case takes from the pilot's SimBrief plan.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PlanFacts {
    pub origin: String,
    pub destination: String,
    pub origin_rwy: Option<String>,
    pub dest_rwy: Option<String>,
    /// The ICAO type.
    pub aircraft: String,
    /// The route's fixes, origin to destination, and whether the navlog
    /// has a top of descent (an enroute case's point is checked on them).
    pub fixes: Vec<String>,
    pub has_tod: bool,
}

impl PlanFacts {
    pub fn of(p: &crate::simbrief::FlightPlan) -> Self {
        Self {
            origin: p.origin.trim().to_ascii_uppercase(),
            destination: p.destination.trim().to_ascii_uppercase(),
            origin_rwy: p.procedures.origin_rwy.clone(),
            dest_rwy: p.procedures.dest_rwy.clone(),
            aircraft: p.aircraft.trim().to_ascii_uppercase(),
            fixes: p.waypoints.iter().map(|w| w.ident.to_ascii_uppercase()).collect(),
            has_tod: p.schedule.tod.is_some(),
        }
    }
}

/// The plan's words in a case (`plan_origin`, `plan_destination`, runway
/// `plan`, aircraft `plan`) made the plan's facts. `find_acf` is the set-up
/// tool's installed aircraft for a type. Pure.
pub fn resolve_case(
    case: &Case,
    plan: &PlanFacts,
    find_acf: &dyn Fn(&str) -> Option<String>,
) -> Result<Case> {
    let mut c = case.clone();
    let bad = |why: String| anyhow!("case {:?}: {why}", case.name);
    let (field, runway) = c.field_mut();
    let rwy = match field.as_str() {
        FIELD_PLAN_ORIGIN => {
            *field = plan.origin.clone();
            plan.origin_rwy.clone()
        }
        FIELD_PLAN_DESTINATION => {
            *field = plan.destination.clone();
            plan.dest_rwy.clone()
        }
        _ => None,
    };
    if field.is_empty() {
        return Err(bad("the flight plan has no such field".into()));
    }
    let enroute = matches!(case.setup, Setup::Enroute { .. });
    if let Some(r) = runway {
        if r.as_str() == RUNWAY_FROM_PLAN && enroute {
            // Only a swap needs it (the aircraft loaded on the origin's
            // runway first): an absent one is said at that swap.
            *r = rwy.map(|x| x.trim().to_ascii_uppercase()).unwrap_or_default();
        } else if r.as_str() == RUNWAY_FROM_PLAN {
            *r = rwy
                .map(|x| x.trim().to_ascii_uppercase())
                .filter(|x| !x.is_empty())
                .ok_or_else(|| bad("the flight plan names no runway there".into()))?;
        }
    }
    if let Setup::Enroute { point, .. } = &c.setup {
        match point {
            EnroutePoint::Fix(f) => {
                let i = plan.fixes.iter().position(|x| x.eq_ignore_ascii_case(f));
                match i {
                    None => return Err(bad(format!("enroute fix {f} is not on the flight plan"))),
                    Some(i) if i == 0 || i + 1 >= plan.fixes.len() => {
                        return Err(bad(format!("enroute fix {f} is an end of the flight plan")))
                    }
                    _ => {}
                }
            }
            EnroutePoint::BeforeTod(_) if !plan.has_tod => {
                return Err(bad("the flight plan has no top of descent on its navlog".into()))
            }
            _ => {}
        }
    }
    if c.aircraft.as_deref() == Some(AIRCRAFT_FROM_PLAN) {
        c.aircraft = Some(find_acf(&plan.aircraft).ok_or_else(|| {
            bad(format!(
                "no installed aircraft of the plan's type {}",
                plan.aircraft
            ))
        })?);
    }
    Ok(c)
}

/// An `.acf` path as the detector keys it (`detect::model_key`).
pub fn norm_acf(p: &str) -> String {
    p.trim().replace('\\', "/").to_lowercase()
}

/// The app's detector has resolved and probed this aircraft: its
/// `profile_probe` record, written once per model change (`detect.rs`).
pub fn probe_is_for(r: &Value, want_acf: &str) -> bool {
    st(r, "kind") == "profile_probe"
        && r.pointer("/identity/relative_path")
            .and_then(Value::as_str)
            .is_some_and(|p| norm_acf(p) == norm_acf(want_acf))
}

/// The typed words for a new flight on a final (the grammar's
/// `setup_approach`, pinned by a test against the real parser).
pub fn approach_line(field: &str, runway: &str, final_nm: i64) -> String {
    format!("put us on a {final_nm} mile final to {field} runway {runway}")
}

/// Parse and check a spec against the X-Plane install: every
/// `setup.aircraft` path must be an `.acf` under `xplane_root()`.
pub fn parse_spec(text: &str) -> Result<Spec> {
    parse_spec_with(text, &|rel| {
        std::path::Path::new(&crate::config::xplane_path(rel)).is_file()
    })
}

/// [`parse_spec`] with the install's answer to "is this `.acf` there?".
/// Every problem is named with its case.
pub fn parse_spec_with(text: &str, acf_exists: &dyn Fn(&str) -> bool) -> Result<Spec> {
    let mut spec: Spec = serde_yaml::from_str(text).map_err(|e| anyhow!("spec: {e}"))?;
    if spec.cases.is_empty() {
        return Err(anyhow!("spec: no cases"));
    }
    let after = spec.after;
    for c in &mut spec.cases {
        c.after = c.after.or(after);
    }
    for c in &spec.cases {
        let bad = |why: &str| Err(anyhow!("case {:?}: {why}", c.name));
        if c.name.trim().is_empty() {
            return bad("needs a name");
        }
        if c.deadline_min.is_some_and(|d| !(d > 0.0)) {
            return bad("deadline_min must be above zero");
        }
        if c.sim_speed
            .is_some_and(|s| !(1..=MAX_SIM_SPEED).contains(&s))
        {
            return bad(&format!("sim_speed is 1 to {MAX_SIM_SPEED}"));
        }
        if c.command.as_deref().is_some_and(|s| s.trim().is_empty()) {
            return bad("the command is empty");
        }
        let has_cmd = c.command.is_some();
        let starts = c.steps.iter().filter(|s| s.when == Trigger::Start).count();
        match &c.setup {
            Setup::Approach { final_nm, .. } => {
                if !(1..=crate::tools::setup::MAX_FINAL_NM).contains(final_nm) {
                    return bad(&format!(
                        "final_nm is 1 to {}",
                        crate::tools::setup::MAX_FINAL_NM
                    ));
                }
                // A command typed into the approach's own sortie is a
                // "tool" record: the scorer calls that attended, a fail.
                if has_cmd {
                    return bad("an approach case has no command: the setup arms autoland");
                }
                if starts > 0 {
                    return bad("an approach case has no start step: the setup is the start");
                }
            }
            _ if has_cmd && starts > 0 => {
                return bad("the command is the start step: give one or the other")
            }
            _ if !has_cmd && c.steps.first().map(|s| &s.when) != Some(&Trigger::Start) => {
                return bad("needs a command (or a first step when: start)")
            }
            _ if starts > 1 => return bad("only the first step is when: start"),
            _ => {}
        }
        if let Some(a) = c.aircraft.as_deref() {
            if a != AIRCRAFT_FROM_PLAN {
                if !a.to_ascii_lowercase().ends_with(".acf") {
                    return bad(&format!("aircraft {a:?} is not an .acf path"));
                }
                if !acf_exists(a) {
                    return bad(&format!(
                        "aircraft {a:?} is not installed under {}",
                        crate::config::xplane_root()
                    ));
                }
            }
        }
        let plan_field = matches!(c.field(), FIELD_PLAN_ORIGIN | FIELD_PLAN_DESTINATION);
        let plan_rwy = match &c.setup {
            Setup::Approach { runway, .. } | Setup::RunwayStart { runway, .. } => {
                runway == RUNWAY_FROM_PLAN
            }
            Setup::Ramp { .. } | Setup::Enroute { .. } => false,
        };
        if plan_rwy && !plan_field {
            return bad("runway plan needs field plan_origin or plan_destination");
        }
        if c.expect_give_up
            .as_deref()
            .is_some_and(|g| g.trim().is_empty())
        {
            return bad("expect_give_up names the give-up reason");
        }
    }
    Ok(spec)
}

/// The flight-initialisation JSON for a start on the runway's threshold:
/// the SDK's `runway_start` without a final distance. Pure.
pub fn runway_start_json(
    acf_rel: &str,
    airport: &str,
    runway: &str,
    engines_running: bool,
) -> String {
    json!({
        "aircraft": { "path": acf_rel },
        "runway_start": { "airport_id": airport, "runway": runway },
        "engine_status": { "all_engines": { "running": engines_running } },
    })
    .to_string()
}

// ---- The runway the wind picks ---------------------------------------------------

/// The aircraft on the ground within this of a field's runway ends is ON
/// the field: the sim's wind at the aircraft is the field's surface wind.
pub const FIELD_WIND_NM: f64 = 3.0;

/// A `runway: wind` resolved: the end, the wind it was picked into, and
/// where that wind came from. Its words ride the case's row.
#[derive(Debug, Clone, PartialEq)]
pub struct WindPick {
    pub runway: String,
    pub from_deg: f64,
    pub kts: f64,
    pub headwind_kts: f64,
    /// "METAR" or "sim".
    pub source: &'static str,
}

impl WindPick {
    pub fn words(&self) -> String {
        format!(
            "runway {} into {:03.0}/{:.0} {}, headwind {:.1} kt",
            self.runway, self.from_deg, self.kts, self.source, self.headwind_kts
        )
    }
}

/// The into-wind end by the tower's own rule (`nav::best_runway_for_wind`:
/// the most headwind among the field's assignable runways, the longest of
/// parallels). Pure.
pub fn pick_into_wind(
    runways: &[crate::nav::Runway],
    wind_from_deg: f64,
    wind_kts: f64,
    source: &'static str,
) -> Option<WindPick> {
    crate::nav::best_runway_for_wind(runways, wind_from_deg, wind_kts).map(|(id, _, hw)| {
        WindPick {
            runway: id,
            from_deg: wind_from_deg,
            kts: wind_kts,
            headwind_kts: (hw * 10.0).round() / 10.0,
            source,
        }
    })
}

/// The field's surface wind now: the newest METAR the app reads
/// (`weather::surface_wind`), else the sim's wind at the aircraft when it
/// sits on the field. Aloft or elsewhere the aircraft's wind is not the
/// field's (the tower's own rule, `monitor::feed_surface_wind`): no answer.
async fn field_wind(
    sim: &dyn RunnerSim,
    field: &str,
    runways: &[crate::nav::Runway],
) -> Result<(f64, f64, &'static str)> {
    if let Some((from, kts)) = sim.metar_wind(field).await {
        return Ok((from, kts, "METAR"));
    }
    let on_ground = sim
        .read(crate::datarefs::ON_GROUND_ANY)
        .await
        .ok()
        .is_some_and(|g| g > 0.5);
    let (lat, lon) = (
        sim.read(crate::datarefs::LATITUDE).await.ok(),
        sim.read(crate::datarefs::LONGITUDE).await.ok(),
    );
    let on_field = match (lat, lon) {
        (Some(a), Some(b)) => runways.iter().any(|r| {
            crate::nav::haversine_nm(a, b, r.lat_a, r.lon_a) <= FIELD_WIND_NM
                || crate::nav::haversine_nm(a, b, r.lat_b, r.lon_b) <= FIELD_WIND_NM
        }),
        _ => false,
    };
    if on_ground && on_field {
        let dir = sim.read(crate::datarefs::WIND_DIR_DEG_TRUE).await;
        let spd = sim.read(crate::datarefs::WIND_SPEED_MSC).await;
        if let (Ok(d), Ok(v)) = (dir, spd) {
            return Ok((d.rem_euclid(360.0), v * MPS_TO_KTS, "sim"));
        }
    }
    Err(anyhow!(
        "no surface wind for {field}: no METAR on file and the aircraft is not on the field"
    ))
}

/// A `runway: wind` case made the into-wind end now; any other case as it
/// is. The pick is the row's to report.
async fn resolve_wind(
    sim: &dyn RunnerSim,
    case: &Case,
    apt_dat: &str,
) -> Result<(Case, Option<WindPick>)> {
    if !case.wants_wind() {
        return Ok((case.clone(), None));
    }
    let field = case.field().to_string();
    let (path, icao) = (apt_dat.to_string(), field.clone());
    let runways = tokio::task::spawn_blocking(move || crate::nav::find_runways(&path, &icao))
        .await
        .map_err(|e| anyhow!("{e}"))??;
    if runways.is_empty() {
        return Err(anyhow!("{field} has no runways in apt.dat"));
    }
    let (from, kts, source) = field_wind(sim, &field, &runways).await?;
    let pick = pick_into_wind(&runways, from, kts, source)
        .ok_or_else(|| anyhow!("no runway at {field} to face the wind"))?;
    Ok((case.with_runway(&pick.runway), Some(pick)))
}

/// The approach speed of an installed `.acf`, from the profile the app's
/// detector would fly it on (`detect::pick_profile`: a curated profile, else
/// its own placards). `None` when the file is unreadable.
fn approach_kts_of_acf(acf_rel: &str) -> Option<f64> {
    let text = std::fs::read_to_string(crate::config::xplane_path(acf_rel)).ok()?;
    let key = |k: &str| {
        text.lines()
            .find_map(|l| l.strip_prefix(k).map(|v| v.trim().to_string()))
            .unwrap_or_default()
    };
    let who = crate::detect::AcfIdentity {
        icao: crate::tools::setup::acf_icao(&text).unwrap_or_default(),
        relative_path: acf_rel.to_string(),
        author: key("P acf/_author "),
        descrip: key("P acf/_descrip "),
    };
    crate::detect::pick_profile(&who)
        .profile?
        .reference?
        .approach_kts
}

// ---- The case state machine ----------------------------------------------------

/// Where a case is, between the typed command and the verdict.
#[derive(Debug, Clone, PartialEq)]
pub enum Phase {
    /// Typed, waiting for the app to act on it.
    AwaitAck {
        sent_t: f64,
        attempts: u32,
    },
    /// Taken by the app's poller, waiting on its command loop: never resent.
    Queued {
        since: f64,
    },
    /// Taken, waiting for the mission's sortie to begin.
    AwaitSortie {
        since: f64,
    },
    /// The mission is flying in this sortie.
    Flying {
        sortie: String,
    },
    Done,
}

/// What the driver does next.
#[derive(Debug, Clone, PartialEq)]
pub enum Step {
    Wait,
    /// Type the case's line again (no answer within [`ACK_S`]).
    Resend,
    /// The sortie just began: the moment for a case's sim speed.
    Began,
    Finished(Verdict),
}

/// A case's verdict.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Verdict {
    pub pass: bool,
    pub sortie: Option<String>,
    pub stages: Vec<Value>,
    pub interventions: u64,
    pub commands: u64,
    pub outcome: String,
    /// Empty on a pass; otherwise why, in words.
    pub reason: String,
    /// The runner must take the loops off the aircraft ("my controls").
    pub abort: bool,
    /// The case ended on the wreck flag (with [`Verdict::abort`]).
    pub crashed: bool,
    /// The sim is still wrecked after a setup: the rest of the run would
    /// fail in seconds each, so the run stops here.
    pub stop_run: bool,
    /// The case ended with the aircraft airborne and its loops left flying
    /// ([`HandBack::LeaveFlying`]): the next case's setup replaces them.
    pub left_flying: bool,
    /// How the last case's aircraft came to rest before this case's setup,
    /// and how long it took ([`SettleWatch`]).
    pub settle: Option<Settle>,
    /// Scripted steps typed into the verdict's sortie: the scorer counts
    /// each as a pilot's command, the runner as the case's own words.
    pub scripted: u64,
    /// What the row says beside the verdict: the sorties a script ended on
    /// the way, the scripted commands forgiven.
    pub note: String,
    /// How the case was set up where the spec left it to the runner: the
    /// runway the wind picked, the deadline scaled to the airframe.
    pub setup: String,
    /// `after: stop` and the case ended on the ground: the next settle (or
    /// the run's end) types "my controls" at once ([`SettleWatch::stop_now`]).
    pub stop_owed: bool,
    /// Wall seconds the sim flew faster than real time ([`SpeedGuard`]).
    pub fast_s: f64,
    /// The mean multiplier it achieved meanwhile (`sim_speed_actual`).
    pub fast_x: Option<f64>,
}

impl Verdict {
    /// The row's words for the fast-forward; `None` when it never ran fast.
    pub fn fast_words(&self) -> Option<String> {
        (self.fast_s >= 0.5).then(|| match self.fast_x {
            Some(x) => format!("{:.0} s at x{x:.1}", self.fast_s),
            None => format!("{:.0} s", self.fast_s),
        })
    }
}

/// The line that stops a run on a wreck that outlived its setup.
pub const STILL_CRASHED: &str = "the sim is still crashed; reset it";
/// After the case's line, the wreck flag of the LAST case has this long to
/// clear before it counts as this case's: an approach setup's reload takes up
/// to two minutes (`InitTiming`), a ramp or runway start has already reloaded.
/// LEMG 2026-09-28: after 1790584266 crashed, three cases each failed in 2 s
/// as "the aircraft crashed", on the wreck the flag still reported.
pub const CRASH_CLEAR_S: f64 = 150.0;

/// What the runner does with the aircraft when a case ends.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HandBack {
    /// Nothing was armed, or the case ended by itself.
    Nothing,
    /// "my controls": on the ground, or a wreck.
    MyControls,
    /// Airborne (or the wheels unreadable): the loops keep flying it. "my
    /// controls" there hands a flying aircraft to nobody (the deadline once
    /// did at 1,600 ft on a 5 nm final); the next case's setup replaces the
    /// flight, and a run that ends pauses the sim ([`secure_at_end`]).
    LeaveFlying,
}

/// The hand-back for an ended case. `on_ground` is `None` when the read
/// failed: no measurement, so the aircraft is treated as flying.
pub fn hand_back(v: &Verdict, on_ground: Option<bool>) -> HandBack {
    if !v.abort {
        HandBack::Nothing
    } else if v.crashed || on_ground == Some(true) {
        HandBack::MyControls
    } else {
        HandBack::LeaveFlying
    }
}

pub struct CaseMachine {
    pub phase: Phase,
    started: f64,
    deadline_s: f64,
    stages: Vec<Value>,
    last_crew_line: Option<String>,
    give_up: Option<String>,
    tool: Option<String>,
    /// The wreck flag has read clear since the case began: only then is a
    /// wreck this case's.
    crash_clear: bool,
    /// How long the last case's wreck may take to clear ([`CRASH_CLEAR_S`]).
    pub crash_clear_s: f64,
    /// The typed line, so the app's `typed` records are matched to it;
    /// `None` takes any typed line as this one.
    pub line: Option<String>,
    /// Scripted steps not yet typed ([`CaseMachine::note_step_sent`]).
    pub pending_steps: usize,
    /// Scripted lines typed and not yet read by the app's command loop,
    /// with when: one unread for [`QUEUED_S`] is let go (the deadline
    /// judges what it did or did not do).
    unread: Vec<(String, f64)>,
    /// Scripted lines typed into the sortie now flying.
    scripted: u64,
    /// A sortie has begun in this case.
    began: bool,
    /// The sorties a script ended on the way to the verdict's.
    earlier: Vec<String>,
    /// See [`Case::expect_give_up`].
    pub expect_give_up: Option<String>,
}

fn st<'a>(r: &'a Value, k: &str) -> &'a str {
    r.get(k).and_then(Value::as_str).unwrap_or("")
}

impl CaseMachine {
    /// A line that must never go twice (a SETUP: a second one would start
    /// another new flight under the first one's reload) fails unanswered
    /// instead of being sent again.
    pub fn once(now: f64, deadline_s: f64) -> Self {
        let mut m = Self::new(now, deadline_s);
        m.phase = Phase::AwaitAck {
            sent_t: now,
            attempts: 2,
        };
        m
    }

    /// The line was typed at `now`.
    pub fn new(now: f64, deadline_s: f64) -> Self {
        Self {
            phase: Phase::AwaitAck {
                sent_t: now,
                attempts: 1,
            },
            started: now,
            deadline_s,
            stages: Vec::new(),
            last_crew_line: None,
            give_up: None,
            tool: None,
            crash_clear: false,
            crash_clear_s: CRASH_CLEAR_S,
            line: None,
            pending_steps: 0,
            unread: Vec::new(),
            scripted: 0,
            began: false,
            earlier: Vec::new(),
            expect_give_up: None,
        }
    }

    /// The opening line has had its answer: a command's tool record or crew
    /// line, or, on an approach setup (`sortie_begun`), its sortie's begin.
    /// The script's clocks start here.
    pub fn opened(&self, sortie_begun: bool) -> bool {
        if sortie_begun {
            self.began
        } else {
            !matches!(self.phase, Phase::AwaitAck { .. } | Phase::Queued { .. })
        }
    }

    /// A scripted step was typed at `now`.
    pub fn note_step_sent(&mut self, now: f64, line: &str) {
        self.pending_steps = self.pending_steps.saturating_sub(1);
        self.unread.push((line.trim().to_ascii_lowercase(), now));
        match &mut self.phase {
            Phase::Flying { .. } => self.scripted += 1,
            // The step may be the one that begins the mission: its grace
            // runs from here.
            Phase::AwaitSortie { since } => *since = now,
            _ => {}
        }
    }

    /// Every scripted line typed and read: the next verdict is the case's.
    fn script_done(&self) -> bool {
        self.pending_steps == 0 && self.unread.is_empty()
    }

    fn fail(&mut self, reason: String, abort: bool) -> Step {
        self.phase = Phase::Done;
        Step::Finished(Verdict {
            pass: false,
            sortie: None,
            stages: std::mem::take(&mut self.stages),
            outcome: "runner".into(),
            reason,
            abort,
            ..Verdict::default()
        })
    }

    /// A `typed` record for this case's line (any line when unknown).
    fn is_my_line(&self, r: &Value) -> bool {
        self.line
            .as_deref()
            .is_none_or(|l| st(r, "text").trim().eq_ignore_ascii_case(l.trim()))
    }

    fn with_said(&self, what: &str) -> String {
        match &self.last_crew_line {
            Some(l) => format!("{what}; the crew said \"{l}\""),
            None => what.to_string(),
        }
    }

    /// One blackbox record, in the order the app wrote it.
    pub fn feed(&mut self, r: &Value) -> Step {
        let t = r.get("t").and_then(Value::as_f64).unwrap_or(0.0);
        let (kind, event) = (st(r, "kind"), st(r, "event"));
        if kind == "said" && st(r, "who") == "crew" {
            self.last_crew_line = Some(st(r, "text").to_string());
        }
        if kind == "typed" && event == "consumed" {
            let text = st(r, "text").trim().to_ascii_lowercase();
            if let Some(i) = self.unread.iter().position(|(l, _)| *l == text) {
                self.unread.remove(i);
            }
        }
        match self.phase.clone() {
            Phase::Done => Step::Wait,
            Phase::AwaitAck { attempts, .. } => {
                if kind == "typed" && event == "taken" && self.is_my_line(r) {
                    self.phase = Phase::Queued { since: t };
                    return Step::Wait;
                }
                // Read at once: the answer's clock starts at the reading.
                if kind == "typed" && event == "consumed" && self.is_my_line(r) {
                    self.phase = Phase::AwaitAck {
                        sent_t: t,
                        attempts: attempts.max(2),
                    };
                    return Step::Wait;
                }
                if kind == "tool" || (kind == "said" && st(r, "who") == "crew") {
                    if kind == "tool" {
                        self.tool = Some(st(r, "name").to_string());
                    }
                    self.phase = Phase::AwaitSortie { since: t };
                }
                Step::Wait
            }
            // On the channel: the loop reading it is the next word. A crew
            // line meanwhile is chatter, not an answer to an unread line.
            Phase::Queued { .. } => {
                if kind == "typed" && event == "consumed" && self.is_my_line(r) {
                    // Read: answered within ACK_S of now, and never retyped.
                    self.phase = Phase::AwaitAck {
                        sent_t: t,
                        attempts: 2,
                    };
                } else if kind == "tool" {
                    self.tool = Some(st(r, "name").to_string());
                    self.phase = Phase::AwaitSortie { since: t };
                }
                Step::Wait
            }
            Phase::AwaitSortie { .. } => match (kind, event) {
                ("tool", _) if self.tool.is_none() => {
                    self.tool = Some(st(r, "name").to_string());
                    Step::Wait
                }
                ("sortie", "begin") => {
                    self.phase = Phase::Flying {
                        sortie: st(r, "sortie").to_string(),
                    };
                    self.began = true;
                    self.scripted = 0;
                    self.stages.clear();
                    self.give_up = None;
                    Step::Began
                }
                // The approach setup's own answer: armed, or why not.
                ("setup", "approach_result") if r.get("armed") != Some(&Value::Bool(true)) => {
                    let why = format!(
                        "the approach setup did not arm ({}, result {})",
                        st(r, "outcome"),
                        r.get("result").cloned().unwrap_or(Value::Null)
                    );
                    self.fail(self.with_said(&why), false)
                }
                _ => Step::Wait,
            },
            Phase::Flying { sortie } => {
                let mine = st(r, "sortie") == sortie;
                match (kind, event) {
                    ("stage_result", _) if mine => {
                        self.stages.push(r.clone());
                        Step::Wait
                    }
                    // Full auto writes its give-up as a stage with the reason
                    // under `detail`; the sortie's end repeats it. Either
                    // names the give-up (the reject at LEMG 13, 2026-09-28,
                    // was read as no reason at all).
                    ("full_auto" | "heli_route", _)
                        if mine && (event == "give_up" || st(r, "stage") == "give_up") =>
                    {
                        let reason = match st(r, "reason") {
                            "" => r
                                .get("detail")
                                .and_then(|d| d.get("reason"))
                                .and_then(Value::as_str)
                                .unwrap_or(""),
                            s => s,
                        };
                        self.give_up = Some(reason.to_string());
                        Step::Wait
                    }
                    ("sortie", "end")
                        if mine && st(r, "outcome") == "gave_up" && self.give_up.is_none() =>
                    {
                        self.give_up = Some(st(r, "reason").to_string());
                        Step::Wait
                    }
                    // A sortie the script ends on the way (its "my controls",
                    // or the next mission superseding it) is not the case's
                    // verdict: the next one is.
                    ("sortie_result", _)
                        if mine
                            && (self.scripted > 0 || !self.unread.is_empty())
                            && (!self.script_done() || st(r, "outcome") == "superseded") =>
                    {
                        self.earlier.push(format!(
                            "{sortie} {} ({})",
                            if r.get("pass") == Some(&Value::Bool(true)) {
                                "passed"
                            } else {
                                "failed"
                            },
                            st(r, "outcome")
                        ));
                        self.stages.clear();
                        self.phase = Phase::AwaitSortie { since: t };
                        Step::Wait
                    }
                    ("sortie_result", _) if mine => {
                        self.phase = Phase::Done;
                        Step::Finished(self.verdict_from(r, &sortie))
                    }
                    _ => Step::Wait,
                }
            }
        }
    }

    fn verdict_from(&mut self, r: &Value, sortie: &str) -> Verdict {
        let n = |k: &str| r.get(k).and_then(Value::as_u64).unwrap_or(0);
        let mut pass = r.get("pass").and_then(Value::as_bool) == Some(true);
        let outcome = st(r, "outcome").to_string();
        let stages = std::mem::take(&mut self.stages);
        let crashed = r.get("crashed").and_then(Value::as_bool) == Some(true);
        // Ended in the air: never finished, whoever's words flew it (#160).
        let airborne = r.get("ended_airborne").and_then(Value::as_bool) == Some(true);
        let why_of = |s: &Value| -> Vec<String> {
            s.get("why")
                .and_then(Value::as_array)
                .map(|a| {
                    a.iter()
                        .filter_map(Value::as_str)
                        .map(str::to_string)
                        .collect()
                })
                .unwrap_or_default()
        };
        let failed: Vec<&Value> = stages
            .iter()
            .filter(|s| s.get("pass") != Some(&Value::Bool(true)))
            .collect();
        // The script's own lines are the case's words, not a pilot's: the
        // scorer's "attended" is re-judged against them.
        let (interventions, commands) = (n("interventions"), n("commands"));
        let own_words = interventions == 0 && commands <= self.scripted;
        let mut notes: Vec<String> = Vec::new();
        if !self.earlier.is_empty() {
            notes.push(format!("before it: {}", self.earlier.join(", ")));
        }
        let mut expected_why: Option<String> = None;
        if let Some(g) = self.expect_give_up.clone() {
            // Meant to end in this give-up: a stage it cut short ("ended by
            // give_up") is the script's, any other failure still fails.
            let gave = outcome == "gave_up" && self.give_up.as_deref() == Some(g.as_str());
            let cut_short_only = failed
                .iter()
                .all(|s| why_of(s).iter().all(|w| w.starts_with("ended by")));
            pass = gave && own_words && !crashed && cut_short_only;
            if !gave {
                expected_why = Some(format!(
                    "expected the give-up {g}, the outcome was {outcome}{}",
                    self.give_up
                        .as_deref()
                        .map(|x| format!(" ({x})"))
                        .unwrap_or_default()
                ));
            } else if pass {
                notes.push(format!("gave up as meant ({g})"));
            }
        } else if !pass
            && commands > 0
            && own_words
            && !crashed
            && !airborne
            && failed.is_empty()
            && !stages.is_empty()
            && outcome != "gave_up"
            && outcome != "superseded"
        {
            pass = true;
        }
        if pass && commands > 0 {
            notes.push(format!("{commands} commands were the script's"));
        }
        let mut why: Vec<String> = Vec::new();
        if !pass {
            if crashed {
                why.push("the aircraft crashed".into());
            }
            if airborne && !crashed {
                why.push("it ended in the air".into());
            }
            if let Some(e) = expected_why {
                why.push(e);
            }
            for s in failed.iter().copied() {
                let reasons: Vec<&str> = s
                    .get("why")
                    .and_then(Value::as_array)
                    .map(|a| a.iter().filter_map(Value::as_str).collect())
                    .unwrap_or_default();
                why.push(format!("{}: {}", st(s, "stage"), reasons.join(", ")));
            }
            if outcome == "gave_up" {
                why.push(format!(
                    "gave up{}",
                    self.give_up
                        .as_deref()
                        .map(|g| format!(" ({g})"))
                        .unwrap_or_default()
                ));
            }
            if outcome == "superseded" {
                why.push("superseded by another mission".into());
            }
            if r.get("unattended").and_then(Value::as_bool) == Some(false) && !own_words {
                why.push(format!(
                    "attended ({interventions} interventions, {commands} commands{})",
                    if self.scripted > 0 {
                        format!(", {} of them the script's", self.scripted.min(commands))
                    } else {
                        String::new()
                    }
                ));
            }
            if stages.is_empty() && why.is_empty() {
                why.push("no stage was scored".into());
            }
        }
        Verdict {
            pass,
            sortie: Some(sortie.to_string()),
            stages,
            interventions,
            commands,
            outcome,
            reason: why.join("; "),
            abort: false,
            crashed,
            scripted: self.scripted,
            note: notes.join("; "),
            ..Verdict::default()
        }
    }

    /// The clock and the wreck flag, every driver pass. `crashed` is `None`
    /// when the read failed: no measurement, never a crash.
    pub fn tick(&mut self, now: f64, crashed: Option<bool>) -> Step {
        if self.phase == Phase::Done {
            return Step::Wait;
        }
        self.unread.retain(|(_, sent)| now - sent <= QUEUED_S);
        let sortie = match &self.phase {
            Phase::Flying { sortie } => Some(sortie.clone()),
            _ => None,
        };
        let tag = |mut v: Step| {
            if let Step::Finished(x) = &mut v {
                x.sortie = sortie.clone();
            }
            v
        };
        match crashed {
            Some(false) => self.crash_clear = true,
            Some(true) if self.crash_clear => {
                let mut v = tag(self.fail("the aircraft crashed".into(), true));
                if let Step::Finished(x) = &mut v {
                    x.crashed = true;
                }
                return v;
            }
            // The last case's wreck, until the setup's reload clears it.
            Some(true) if now - self.started > self.crash_clear_s => {
                let mut v = tag(self.fail(STILL_CRASHED.into(), false));
                if let Step::Finished(x) = &mut v {
                    x.stop_run = true;
                }
                return v;
            }
            _ => {}
        }
        if now - self.started > self.deadline_s {
            let why = format!("deadline {:.0} min passed", self.deadline_s / 60.0);
            return tag(self.fail(self.with_said(&why), true));
        }
        match self.phase.clone() {
            Phase::AwaitAck { sent_t, attempts } if now - sent_t > ACK_S => {
                if attempts >= 2 {
                    return self.fail(
                        format!("the app did not take the command (no tool record or crew line in {ACK_S:.0} s)"),
                        false,
                    );
                }
                self.phase = Phase::AwaitAck {
                    sent_t: now,
                    attempts: attempts + 1,
                };
                Step::Resend
            }
            // Taken and never read: the app may still act on it, so the
            // aircraft is taken back (`abort`), and it is never typed again.
            Phase::Queued { since } if now - since > QUEUED_S => self.fail(
                format!("the app took the command but its loop never read it in {QUEUED_S:.0} s"),
                true,
            ),
            // A step still to come may be the one that begins it.
            Phase::AwaitSortie { since }
                if self.pending_steps == 0 && now - since > SORTIE_GRACE_S =>
            {
                let what = format!(
                    "no mission began{}",
                    self.tool
                        .as_deref()
                        .map(|t| format!(" after {t}"))
                        .unwrap_or_default()
                );
                self.fail(self.with_said(&what), true)
            }
            _ => Step::Wait,
        }
    }
}

// ---- The run table -----------------------------------------------------------------

/// One case flown.
#[derive(Debug, Clone)]
pub struct Row {
    pub case: String,
    pub rep: u32,
    pub verdict: Verdict,
    pub secs: f64,
}

/// The number a stage is judged on, against its limit (the scorer's own
/// constants), for the row.
fn stage_numbers(s: &Value) -> String {
    use crate::scorecard as sc;
    let f = |k: &str| s.get(k).and_then(Value::as_f64);
    let mut parts: Vec<String> = Vec::new();
    let mut add = |label: &str, v: Option<f64>, limit: f64, unit: &str| {
        if let Some(v) = v {
            parts.push(format!("{label} {v}/{limit} {unit}"));
        }
    };
    match st(s, "stage") {
        "landing" => {
            add("sink", f("sink_fpm"), sc::TOUCHDOWN_MAX_SINK_FPM, "fpm");
            add("drift", f("drift_deg"), sc::TOUCHDOWN_MAX_DRIFT_DEG, "deg");
            add(
                "rollout",
                f("rollout_max_xtk_m"),
                sc::ROLLOUT_MAX_XTK_M,
                "m",
            );
        }
        "takeoff" => add("xtk", f("max_xtk_m"), sc::ROLL_MAX_XTK_M, "m"),
        "alignment" => {
            add("err", f("err_deg"), sc::LINEUP_MAX_ERR_DEG, "deg");
            add("xtk", f("xtk_m"), sc::LINEUP_MAX_XTK_M, "m");
        }
        _ if s.get("max_lat_m").is_some() => {
            add("lat", f("max_lat_m"), sc::TAXI_MAX_LAT_M, "m");
            add("cut", f("max_cut_m"), sc::TAXI_MAX_CUT_M, "m");
        }
        _ => {}
    }
    let ok = if s.get("pass") == Some(&Value::Bool(true)) {
        "pass"
    } else {
        "FAIL"
    };
    if parts.is_empty() {
        format!("{} {ok}", st(s, "stage"))
    } else {
        format!("{} {ok} ({})", st(s, "stage"), parts.join(", "))
    }
}

/// The row as one line for the terminal.
pub fn row_line(r: &Row) -> String {
    let v = &r.verdict;
    let stages: Vec<String> = v.stages.iter().map(stage_numbers).collect();
    let before = v
        .settle
        .and_then(|s| s.words())
        .map(|w| format!(" ({w})"))
        .unwrap_or_default();
    let setup = match v.setup.is_empty() {
        true => String::new(),
        false => format!(" ({})", v.setup),
    };
    let before = match v.fast_words() {
        Some(f) => format!(" (fast {f}){before}"),
        None => before,
    };
    format!(
        "{} {} #{}{setup}: {} in {:.0} s{before}, sortie {}, {}{}{}",
        if v.pass { "PASS" } else { "FAIL" },
        r.case,
        r.rep,
        if stages.is_empty() {
            "no stages".into()
        } else {
            stages.join("; ")
        },
        r.secs,
        v.sortie.as_deref().unwrap_or("none"),
        format!("{} interventions", v.interventions),
        if v.commands > 0 {
            format!(", {} commands", v.commands)
        } else {
            String::new()
        },
        match (v.reason.is_empty(), v.note.is_empty()) {
            (true, true) => String::new(),
            (true, false) => format!(" [{}]", v.note),
            (false, true) => format!(": {}", v.reason),
            (false, false) => format!(": {} [{}]", v.reason, v.note),
        },
    )
}

fn cell(s: &str) -> String {
    s.replace('|', "/").replace('\n', " ")
}

/// The run table as Markdown: one row per case flown, then the count.
pub fn table(spec_path: &str, started: &str, rows: &[Row]) -> String {
    let mut out = format!(
        "# Sortie run {started}\n\nSpec: `{spec_path}`\n\n\
         | Case | # | Sortie | Verdict | Stages | Interventions | Wall s | Fast | Before | Reason |\n\
         |---|---|---|---|---|---|---|---|---|---|\n"
    );
    for r in rows {
        let v = &r.verdict;
        let stages: Vec<String> = v.stages.iter().map(stage_numbers).collect();
        let before = v
            .settle
            .and_then(|s| s.words())
            .unwrap_or_else(|| "-".into());
        let case = match v.setup.is_empty() {
            true => r.case.clone(),
            false => format!("{} ({})", r.case, v.setup),
        };
        out.push_str(&format!(
            "| {} | {} | {} | {} | {} | {} | {:.0} | {} | {} | {} |\n",
            cell(&case),
            r.rep,
            v.sortie.as_deref().unwrap_or("-"),
            if v.pass { "PASS" } else { "FAIL" },
            cell(&stages.join("; ")),
            v.interventions,
            r.secs,
            v.fast_words().unwrap_or_else(|| "-".into()),
            cell(&before),
            cell(&match (v.reason.is_empty(), v.note.is_empty()) {
                (_, true) => v.reason.clone(),
                (true, false) => format!("[{}]", v.note),
                (false, false) => format!("{} [{}]", v.reason, v.note),
            }),
        ));
    }
    if rows.last().is_some_and(|r| r.verdict.stop_run) {
        eprintln!("run stopped: {STILL_CRASHED}");
    }
    let passed = rows.iter().filter(|r| r.verdict.pass).count();
    out.push_str(&format!("\n{passed} of {} passed.\n", rows.len()));
    out
}

// ---- The blackbox tail -------------------------------------------------------------

/// The app's blackbox, read from where the runner started: new whole lines
/// only, a rotated (shorter) file read again from its start.
pub struct BlackboxTail {
    path: std::path::PathBuf,
    offset: u64,
    partial: Vec<u8>,
}

impl BlackboxTail {
    /// From the file's current end (nothing already written is fed).
    pub fn at_end(path: impl Into<std::path::PathBuf>) -> Self {
        let path = path.into();
        let offset = std::fs::metadata(&path).map(|m| m.len()).unwrap_or(0);
        Self {
            path,
            offset,
            partial: Vec::new(),
        }
    }

    /// Skip everything written so far.
    pub fn skip_to_end(&mut self) {
        self.offset = std::fs::metadata(&self.path).map(|m| m.len()).unwrap_or(0);
        self.partial.clear();
    }

    pub fn poll(&mut self) -> Vec<Value> {
        use std::io::{Read, Seek, SeekFrom};
        let Ok(mut f) = std::fs::File::open(&self.path) else {
            return Vec::new();
        };
        let len = f.metadata().map(|m| m.len()).unwrap_or(0);
        if len < self.offset {
            self.offset = 0;
            self.partial.clear();
        }
        if f.seek(SeekFrom::Start(self.offset)).is_err() {
            return Vec::new();
        }
        let mut buf = Vec::new();
        if f.read_to_end(&mut buf).is_err() {
            return Vec::new();
        }
        self.offset += buf.len() as u64;
        // Bytes until a newline: a read that ends inside a multi-byte
        // character must not mangle it. A line that does not parse (two
        // records interleaved, #144) is skipped alone; the next line stands.
        self.partial.extend_from_slice(&buf);
        let mut out = Vec::new();
        while let Some(i) = self.partial.iter().position(|&b| b == b'\n') {
            let line: Vec<u8> = self.partial.drain(..=i).collect();
            if let Ok(v) = serde_json::from_slice::<Value>(line.trim_ascii()) {
                out.push(v);
            }
        }
        out
    }
}

// ---- The sim, as the runner sees it ---------------------------------------------------

#[async_trait]
pub trait RunnerSim: Send + Sync {
    async fn read(&self, name: &str) -> Result<f64>;
    async fn read_string(&self, name: &str) -> Result<String>;
    async fn write_float(&self, name: &str, v: f64) -> Result<()>;
    async fn write_bytes(&self, name: &str, bytes: &[u8]) -> Result<()>;
    async fn command(&self, name: &str) -> Result<()>;
    /// The field's surface wind from the newest METAR on file (direction
    /// true, knots), the app's own `weather::surface_wind`.
    async fn metar_wind(&self, icao: &str) -> Option<(f64, f64)> {
        crate::weather::surface_wind(&crate::config::xplane_root(), icao)
    }
    /// The approach speed of an installed aircraft (`.acf` path relative to
    /// the X-Plane root), knots.
    async fn approach_kts(&self, acf_rel: &str) -> Option<f64> {
        approach_kts_of_acf(acf_rel)
    }
}

#[async_trait]
impl RunnerSim for crate::xplane::XPlaneClient {
    async fn command(&self, name: &str) -> Result<()> {
        self.run_command(name).await
    }
    async fn read(&self, name: &str) -> Result<f64> {
        self.get_float(name).await
    }
    async fn read_string(&self, name: &str) -> Result<String> {
        self.get_string(name).await
    }
    async fn write_float(&self, name: &str, v: f64) -> Result<()> {
        self.set_float(name, v).await
    }
    async fn write_bytes(&self, name: &str, bytes: &[u8]) -> Result<()> {
        self.set_data(name, bytes).await
    }
}

/// The setup mailbox through the runner's sim.
struct Mailbox<'a>(&'a dyn RunnerSim);

#[async_trait]
impl InitMailbox for Mailbox<'_> {
    async fn read(&self, name: &str) -> Result<f64> {
        self.0.read(name).await
    }
    async fn post(&self, body: &[u8]) -> Result<()> {
        self.0.write_bytes("xcopilot/sim/init_flight", body).await
    }
}

/// How the driver keeps time; the tests shrink it.
#[derive(Debug, Clone, Copy)]
pub struct Pace {
    pub poll: std::time::Duration,
    pub init: InitTiming,
    /// A set-up aircraft is settled once still this long...
    pub still_s: f64,
    /// ...and the wait for it ends failed after this long.
    pub still_max_s: f64,
    /// The case's `settle_s` is multiplied by this (1 live).
    pub settle_scale: f64,
    /// Longest wait for the loops of the last case to let go once "my
    /// controls" is typed.
    pub idle_max_s: f64,
    /// Before a setup the last case's aircraft is at rest (no loop driving,
    /// on the ground under [`STOPPED_KTS`]) this long...
    pub quiet_s: f64,
    /// ...waited for this long (its taxi-in finishing), then "my controls".
    pub stop_max_s: f64,
    /// Longest wait for the last case's wreck to clear ([`CRASH_CLEAR_S`]).
    pub crash_clear_s: f64,
    /// Longest wait for the app to swap to a case's aircraft ([`SWAP_MAX_S`]).
    pub swap_max_s: f64,
    /// Every fast condition holds this long first ([`FAST_RESUME_S`]).
    pub fast_resume_s: f64,
    /// Longest wait for an enroute setup's `enroute_result` ([`ENROUTE_MAX_S`]).
    pub enroute_max_s: f64,
}

impl Pace {
    pub const LIVE: Pace = Pace {
        poll: std::time::Duration::from_millis(500),
        init: InitTiming::RAMP,
        still_s: 5.0,
        still_max_s: 240.0,
        settle_scale: 1.0,
        idle_max_s: 60.0,
        quiet_s: QUIET_S,
        stop_max_s: STOP_MAX_S,
        crash_clear_s: CRASH_CLEAR_S,
        swap_max_s: SWAP_MAX_S,
        fast_resume_s: FAST_RESUME_S,
        enroute_max_s: ENROUTE_MAX_S,
    };
}

/// From the typed enroute setup to its `enroute_result`: the reload (up to
/// two minutes, `InitTiming::LIVE`), the autopilot's engage (at most a
/// minute), the settle and the station (a minute and fifteen seconds).
pub const ENROUTE_MAX_S: f64 = 300.0;

/// From a setup that loads another aircraft to the app's `profile_probe`
/// of it: the reload (up to two minutes, `InitTiming`), then the
/// detector's three-second poll and the probe's index fetch.
pub const SWAP_MAX_S: f64 = 180.0;

fn now_s() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

/// A loop is flying: the status buffer's marker byte is '1' (`auto_status::compose`).
/// `None`: unreadable.
pub fn loop_flying(status: &str) -> bool {
    status.starts_with('1')
}

/// Type one line and see the app's poller consume it (it writes a NUL back).
async fn type_line(sim: &dyn RunnerSim, line: &str, pace: &Pace) -> Result<()> {
    // A line still waiting is the app's to take first: never overwrite it.
    for _ in 0..10 {
        match sim.read_string(TYPED).await {
            Ok(s) if s.is_empty() => break,
            _ => tokio::time::sleep(pace.poll).await,
        }
    }
    sim.write_bytes(TYPED, line.as_bytes()).await?;
    for _ in 0..10 {
        tokio::time::sleep(pace.poll).await;
        if sim.read_string(TYPED).await.is_ok_and(|s| s.is_empty()) {
            return Ok(());
        }
    }
    Err(anyhow!("the typed line was never taken: is xvl running?"))
}

/// Refuse to start over a flying loop or a replay.
pub async fn preflight(sim: &dyn RunnerSim) -> Result<()> {
    if sim
        .read(crate::datarefs::IS_IN_REPLAY)
        .await
        .is_ok_and(|v| v > 0.5)
    {
        return Err(anyhow!("X-Plane is in replay: go back to live first"));
    }
    let status = sim
        .read_string(STATUS_AUTO)
        .await
        .map_err(|e| anyhow!("can't read {STATUS_AUTO} (plugin loaded?): {e}"))?;
    if loop_flying(&status) {
        let what = status
            .get(1..)
            .unwrap_or("")
            .lines()
            .next()
            .unwrap_or("")
            .to_string();
        return Err(anyhow!("a loop is flying ({what}): say my controls first"));
    }
    Ok(())
}

/// Below this the last case's aircraft is stopped.
pub const STOPPED_KTS: f64 = 1.0;
/// `sim/flightmodel/position/groundspeed` is metres per second.
const MPS_TO_KTS: f64 = 1.943_844;
/// At rest this long before a setup: a taxi-in stops between its vacate and
/// its drive to the stand (2.5 s at LEMG 31, 2026-09-28) with no loop
/// driving, and that pause is not its end.
pub const QUIET_S: f64 = 10.0;
/// The longest wait for the last case's taxi-in to finish by itself (LEMG
/// 31's stand was 3,350 m away at 18 kt, about six minutes).
pub const STOP_MAX_S: f64 = 600.0;
/// `after: stop`: a loop still driving this long after "my controls" gets
/// it once more.
pub const STOP_RETYPE_S: f64 = 10.0;

/// How the last case's aircraft was left before this case's setup: the run
/// row carries it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SettleEnd {
    /// At rest by itself (or nothing to wait for).
    AtRest,
    /// Still moving on the ground at the bound: "my controls", then stopped
    /// (`full_auto::hand_back_hold` stops and parks it).
    MyControls,
    /// In the air: taken back with "my controls" right before the setup
    /// replaces the flight, never handed to nobody for longer (#132).
    TakenBack,
    /// `after: stop`: "my controls" at the last verdict, then at rest.
    Stopped,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Settle {
    /// From the case's start to its setup.
    pub secs: f64,
    pub end: SettleEnd,
}

impl Settle {
    /// The row's words; `None` when nothing was waited for.
    pub fn words(&self) -> Option<String> {
        match self.end {
            SettleEnd::AtRest if self.secs < 1.0 => None,
            SettleEnd::AtRest => Some(format!(
                "waited {:.0} s for the last aircraft to stop",
                self.secs
            )),
            SettleEnd::MyControls => Some(format!(
                "my controls after {:.0} s, the last aircraft still moving",
                self.secs
            )),
            SettleEnd::TakenBack => Some("the last aircraft taken back in the air".into()),
            SettleEnd::Stopped => Some(format!(
                "my controls at the last verdict, at rest after {:.0} s",
                self.secs
            )),
        }
    }
}

/// What the driver does next between cases.
#[derive(Debug, Clone, PartialEq)]
pub enum SettleStep {
    Wait,
    TypeMyControls,
    Ready(Settle),
    Fail(String),
}

/// Between cases: the last case's aircraft at rest before the next setup
/// reloads the sim. A full-auto sortie's verdict comes at its `taxi_in`
/// stage with the aircraft still rolling (13 to 16 kt, sorties 1790584715
/// and 1790585651, LEMG 2026-09-28), and the next runway start was posted
/// 0.3 s later under the running taxi. At rest is no loop driving (the
/// panel's marker) AND on the ground under [`STOPPED_KTS`], held
/// [`Pace::quiet_s`]; past [`Pace::stop_max_s`] on the ground "my controls"
/// stops it. In the air (or loops the last case LEFT flying) it is taken
/// back at once, right before this case's setup replaces the flight. A
/// failed read is no measurement: never at rest.
pub struct SettleWatch {
    started: f64,
    quiet_since: Option<f64>,
    /// "my controls" typed at, and whether in the air.
    told: Option<(f64, bool)>,
    take_back_now: bool,
    /// See [`SettleWatch::stop_now`].
    stop_now: bool,
    /// The stop's "my controls" was typed a second time.
    retold: bool,
    quiet_s: f64,
    stop_max_s: f64,
    idle_max_s: f64,
}

impl SettleWatch {
    pub fn new(now: f64, pace: &Pace, take_back_now: bool) -> Self {
        Self {
            started: now,
            quiet_since: None,
            told: None,
            take_back_now,
            stop_now: false,
            retold: false,
            quiet_s: pace.quiet_s,
            stop_max_s: pace.stop_max_s,
            idle_max_s: pace.idle_max_s,
        }
    }

    /// `after: stop` on the last case: on the ground and not at rest,
    /// "my controls" at once instead of waiting out its taxi-in; typed once
    /// more after [`STOP_RETYPE_S`] with a loop still driving (a line read
    /// before the taxi-in began releases nothing).
    pub fn stop_now(mut self, yes: bool) -> Self {
        self.stop_now = yes;
        self
    }

    /// At rest for `quiet_s`: the quiet clock runs only while it holds.
    fn held(&mut self, now: f64, at_rest: bool) -> bool {
        if !at_rest {
            self.quiet_since = None;
            return false;
        }
        let since = *self.quiet_since.get_or_insert(now);
        now - since >= self.quiet_s
    }

    /// One poll: the marker (`Some(true)`: a loop driving), groundspeed in
    /// knots, the wheels.
    pub fn tick(
        &mut self,
        now: f64,
        loop_flying: Option<bool>,
        gs_kts: Option<f64>,
        on_ground: Option<bool>,
    ) -> SettleStep {
        let idle = loop_flying == Some(false);
        let at_rest =
            idle && on_ground == Some(true) && gs_kts.is_some_and(|g| g.abs() < STOPPED_KTS);
        let secs = now - self.started;
        let ready = |end| SettleStep::Ready(Settle { secs, end });
        match self.told {
            None => {
                if self.take_back_now || on_ground == Some(false) {
                    // Nothing flying it: the setup replaces the flight.
                    if idle {
                        return ready(SettleEnd::AtRest);
                    }
                    self.told = Some((now, true));
                    return SettleStep::TypeMyControls;
                }
                let rest_now =
                    idle && gs_kts.is_some_and(|g| g.abs() < STOPPED_KTS);
                if self.stop_now && on_ground == Some(true) && !rest_now {
                    self.told = Some((now, false));
                    self.quiet_since = None;
                    return SettleStep::TypeMyControls;
                }
                if self.held(now, at_rest) {
                    return ready(SettleEnd::AtRest);
                }
                if secs > self.stop_max_s {
                    // Unreadable wheels are no measurement: treated as flying.
                    self.told = Some((now, on_ground != Some(true)));
                    self.quiet_since = None;
                    return SettleStep::TypeMyControls;
                }
                SettleStep::Wait
            }
            Some((t, true)) => {
                if idle {
                    ready(SettleEnd::TakenBack)
                } else if now - t > self.idle_max_s {
                    SettleStep::Fail("a loop is still flying after my controls".into())
                } else {
                    SettleStep::Wait
                }
            }
            Some((t, false)) => {
                if self.held(now, at_rest) {
                    return ready(match self.stop_now {
                        true => SettleEnd::Stopped,
                        false => SettleEnd::MyControls,
                    });
                }
                if self.stop_now
                    && !self.retold
                    && loop_flying == Some(true)
                    && now - t > STOP_RETYPE_S
                {
                    self.retold = true;
                    return SettleStep::TypeMyControls;
                }
                // At rest when the bound comes is settling: the quiet clock
                // runs out by itself (#168: stopped 9 s of the 10 at the
                // bound, and failed "still moving at 0 kt").
                if now - t > self.idle_max_s && self.quiet_since.is_none() {
                    let what = unsettled(loop_flying, gs_kts, on_ground);
                    return SettleStep::Fail(format!(
                        "the last aircraft never stopped after my controls ({what})"
                    ));
                }
                SettleStep::Wait
            }
        }
    }
}

/// What was not at rest, named from the same reads the rest test takes.
fn unsettled(loop_flying: Option<bool>, gs_kts: Option<f64>, on_ground: Option<bool>) -> String {
    match (loop_flying, on_ground, gs_kts) {
        (Some(true), ..) => "a loop is still driving".into(),
        (None, ..) => "the panel marker unreadable".into(),
        (_, None, _) => "its wheels unreadable".into(),
        (_, Some(false), _) => "off the ground".into(),
        (_, _, None) => "its groundspeed unreadable".into(),
        (_, _, Some(g)) if g.abs() >= STOPPED_KTS => format!("still moving at {:.1} kt", g.abs()),
        (_, _, Some(g)) => format!("at {:.1} kt, not yet held still", g.abs()),
    }
}

/// Drive [`SettleWatch`] against the sim until the next setup may go.
async fn settle_before_setup(
    sim: &dyn RunnerSim,
    pace: &Pace,
    take_back_now: bool,
    stop_now: bool,
) -> Result<Settle> {
    let mut w = SettleWatch::new(now_s(), pace, take_back_now).stop_now(stop_now);
    loop {
        let (status, gs, ground) = tokio::join!(
            sim.read_string(STATUS_AUTO),
            sim.read(GROUNDSPEED),
            sim.read(crate::datarefs::ON_GROUND_ANY),
        );
        let step = w.tick(
            now_s(),
            status.ok().map(|s| loop_flying(&s)),
            gs.ok().map(|v| v * MPS_TO_KTS),
            ground.ok().map(|g| g > 0.5),
        );
        match step {
            SettleStep::Ready(s) => return Ok(s),
            SettleStep::Fail(why) => return Err(anyhow!("{why}")),
            SettleStep::TypeMyControls => type_line(sim, ABORT_LINE, pace).await?,
            SettleStep::Wait => {}
        }
        tokio::time::sleep(pace.poll).await;
    }
}

/// A ramp or runway start: write the mailbox, wait for the plugin's answer
/// through the reload, then for the aircraft to sit still and, when the case
/// named another aircraft, for the app's detector to have swapped to it
/// ([`probe_is_for`]). An approach case is set up by the app's own tool from
/// the typed line, on the LOADED aircraft: another one is loaded first, on
/// the same runway with its engines running, and waited for the same way.
async fn set_up_start(
    sim: &dyn RunnerSim,
    tail: &mut BlackboxTail,
    case: &Case,
    apt_dat: &str,
    pace: &Pace,
) -> Result<()> {
    let loaded = sim
        .read_string(ACF_PATH)
        .await
        .ok()
        .filter(|p| !p.trim().is_empty());
    let swap = match (&case.aircraft, &loaded) {
        (Some(w), Some(l)) => norm_acf(w) != norm_acf(l),
        (Some(_), None) => true,
        (None, _) => false,
    };
    let acf = case
        .aircraft
        .clone()
        .or(loaded)
        .ok_or_else(|| anyhow!("can't read which aircraft is loaded"))?;
    let body = match &case.setup {
        Setup::RunwayStart {
            field,
            runway,
            engines_running,
        } => runway_start_json(&acf, field, runway, *engines_running),
        Setup::Ramp { field, stand } => {
            let stand = match stand {
                Some(s) => s.clone(),
                None => {
                    // The stand suits the aircraft being LOADED: its own
                    // .acf's type when the case names one.
                    let icao = match &case.aircraft {
                        Some(a) => std::fs::read_to_string(crate::config::xplane_path(a))
                            .ok()
                            .and_then(|t| crate::tools::setup::acf_icao(&t))
                            .unwrap_or_default(),
                        None => sim.read_string(ACF_ICAO).await.unwrap_or_default(),
                    };
                    pick_stand(apt_dat, field, &icao)?
                }
            };
            crate::tools::setup::init_json(&acf, field, &stand)
        }
        Setup::Approach { field, runway, .. } if swap => {
            runway_start_json(&acf, field, runway, true)
        }
        Setup::Enroute { field, runway, .. } if swap => {
            if runway.is_empty() {
                return Err(anyhow!(
                    "the flight plan names no runway at {field} to load the {acf} on"
                ));
            }
            runway_start_json(&acf, field, runway, true)
        }
        Setup::Approach { .. } => return Ok(()),
        Setup::Enroute { point, .. } => return enroute_setup(sim, tail, point, pace).await,
    };
    tail.skip_to_end();
    let run =
        crate::tools::setup::run_init(&Mailbox(sim), body.as_bytes(), pace.init, &|_| {}).await;
    match run.outcome {
        InitOutcome::Done(0) => {}
        InitOutcome::Done(code) => {
            let (field, rwy) = match &case.setup {
                Setup::RunwayStart { field, runway, .. }
                | Setup::Approach { field, runway, .. }
                | Setup::Enroute { field, runway, .. } => (field.as_str(), runway.as_str()),
                Setup::Ramp { field, .. } => (field.as_str(), ""),
            };
            return Err(anyhow!(
                "{}",
                crate::tools::setup::init_refusal(code, field, rwy)
            ));
        }
        o => {
            return Err(anyhow!(
                "the setup mailbox: {} ({} read errors)",
                o.name(),
                run.read_errors
            ))
        }
    }
    // Still for five seconds, the sim reloaded and its ids re-resolved by
    // the client on the way; and the new aircraft's profile loaded and
    // probed by the app (every tool the command reaches is the airframe's).
    let t0 = now_s();
    let mut still_since: Option<f64> = None;
    let mut swapped = !swap;
    loop {
        tokio::time::sleep(pace.poll).await;
        if !swapped {
            swapped = tail.poll().iter().any(|r| probe_is_for(r, &acf));
        }
        let (g, gs) = (
            sim.read(crate::datarefs::ON_GROUND_ANY).await,
            sim.read(GROUNDSPEED).await,
        );
        let still = if g.is_ok_and(|g| g > 0.5) && gs.is_ok_and(|v| v.abs() < 0.5) {
            let s = *still_since.get_or_insert(now_s());
            now_s() - s >= pace.still_s
        } else {
            still_since = None;
            false
        };
        if still && swapped {
            if let Setup::Enroute { point, .. } = &case.setup {
                // The aircraft is loaded: now the new flight in the air.
                return enroute_setup(sim, tail, point, pace).await;
            }
            let settle = case.settle_s.unwrap_or(DEFAULT_SETTLE_S) * pace.settle_scale;
            tokio::time::sleep(std::time::Duration::from_secs_f64(settle)).await;
            return Ok(());
        }
        let waited = now_s() - t0;
        if !swapped && waited > pace.swap_max_s {
            return Err(anyhow!(
                "the app never took the {acf} (no profile_probe for it in {:.0} s): is xvl running?",
                pace.swap_max_s
            ));
        }
        if !still && waited > pace.still_max_s {
            return Err(anyhow!("the aircraft never sat still after the setup"));
        }
    }
}

/// What the app's enroute setup came to, from its blackbox records: `None`
/// while it is still working; `Some(Ok)` once the new flight is up with the
/// autopilot's servos confirmed; `Some(Err(why))` for a refusal (the tool's
/// own words, `enroute_refused`) or a result without the autopilot. Pure.
pub fn enroute_outcome(r: &Value) -> Option<std::result::Result<(), String>> {
    if st(r, "kind") != "setup" {
        return None;
    }
    match st(r, "event") {
        "enroute_refused" => Some(Err(format!("the app refused: {}", st(r, "why")))),
        "enroute_result" if st(r, "outcome") == "ok" && r["engaged"] == json!(true) => {
            Some(Ok(()))
        }
        "enroute_result" => Some(Err(match st(r, "why") {
            "" => format!(
                "the enroute setup ended {} without the autopilot engaged",
                st(r, "outcome")
            ),
            why => why.to_string(),
        })),
        _ => None,
    }
}

/// Type the enroute setup's words and wait for the app to have put the new
/// flight in the air with the autopilot holding it ([`enroute_outcome`]);
/// then the case's command is typed as on any other start.
async fn enroute_setup(
    sim: &dyn RunnerSim,
    tail: &mut BlackboxTail,
    point: &EnroutePoint,
    pace: &Pace,
) -> Result<()> {
    tail.skip_to_end();
    let line = crate::tools::setup_enroute::enroute_line(point);
    type_line(sim, &line, pace).await?;
    let t0 = now_s();
    loop {
        for r in tail.poll() {
            match enroute_outcome(&r) {
                Some(Ok(())) => return Ok(()),
                Some(Err(why)) => return Err(anyhow!("{why}")),
                None => {}
            }
        }
        if now_s() - t0 > pace.enroute_max_s {
            return Err(anyhow!(
                "no enroute_result in {:.0} s after {line:?}: is xvl running?",
                pace.enroute_max_s
            ));
        }
        tokio::time::sleep(pace.poll).await;
    }
}

/// The stand the setup tool would pick for this type (ICAO; empty: the
/// setup tool's pick for an unknown type).
fn pick_stand(apt_dat: &str, field: &str, icao: &str) -> Result<String> {
    let known = crate::aircraft::profile_for_type(icao);
    let jet = known.as_ref().is_some_and(|p| {
        p.reference
            .as_ref()
            .and_then(|r| r.kind.as_deref())
            .is_some_and(|k| k.to_ascii_lowercase().contains("jet"))
    });
    let heli = known.as_ref().is_some_and(|p| p.rotorcraft);
    let class: &[&str] = if known.is_some() {
        crate::tools::setup::stand_class(jet, heli)
    } else {
        &[]
    };
    let net = crate::nav::find_taxi_net(apt_dat, field)
        .ok()
        .flatten()
        .ok_or_else(|| anyhow!("{field} has no parking positions in apt.dat"))?;
    crate::tools::setup::pick_stand(&net.stands, class, jet)
        .map(|s| s.name.clone())
        .ok_or_else(|| anyhow!("no stand at {field} takes this aircraft"))
}

/// Indicated airspeed, knots.
const IAS_KTS: &str = "sim/flightmodel/position/indicated_airspeed";
const M_TO_FT: f64 = 3.280_84;

/// Fly one case to its verdict. Setup failures are verdicts too.
pub async fn run_case(
    sim: &dyn RunnerSim,
    tail: &mut BlackboxTail,
    case: &Case,
    apt_dat: &str,
    pace: &Pace,
    take_back_now: bool,
    stop_now: bool,
) -> Verdict {
    let failed = |reason: String| Verdict {
        reason,
        outcome: "runner".into(),
        ..Verdict::default()
    };
    // No setup (and no take-back of the last aircraft) at a fast sim.
    ensure_real_time(sim, "before the setup").await;
    let settle = match settle_before_setup(sim, pace, take_back_now, stop_now).await {
        Ok(s) => s,
        Err(e) => return failed(e.to_string()),
    };
    let failed = |reason: String| Verdict {
        settle: Some(settle),
        ..failed(reason)
    };
    let (resolved, pick) = match resolve_wind(sim, case, apt_dat).await {
        Ok(r) => r,
        Err(e) => return failed(format!("setup: {e}")),
    };
    let case = &resolved;
    // The deadline for the airframe that will fly it: the case's own, else
    // the default scaled to its approach speed.
    let acf = match &case.aircraft {
        Some(a) => Some(a.clone()),
        None => sim
            .read_string(ACF_PATH)
            .await
            .ok()
            .filter(|p| !p.trim().is_empty()),
    };
    let vapp = match (&case.deadline_min, &acf) {
        (None, Some(a)) => sim.approach_kts(a).await,
        _ => None,
    };
    let deadline_s = case.deadline_s_for(vapp);
    let mut setup_words: Vec<String> = pick.iter().map(WindPick::words).collect();
    if let Some(k) = vapp.filter(|_| deadline_s > DEFAULT_DEADLINE_MIN * 60.0) {
        setup_words.push(format!("deadline {:.0} min at {k:.0} kt", deadline_s / 60.0));
    }
    let setup_note = setup_words.join(", ");
    if !setup_note.is_empty() {
        eprintln!("{}: {setup_note}", case.name);
    }
    let failed = |reason: String| Verdict {
        setup: setup_note.clone(),
        ..failed(reason)
    };
    if let Err(e) = set_up_start(sim, tail, case, apt_dat, pace).await {
        return failed(format!("setup: {e}"));
    }
    let line = case.first_line();
    tail.skip_to_end();
    if let Err(e) = type_line(sim, &line, pace).await {
        return failed(e.to_string());
    }
    let approach = matches!(case.setup, Setup::Approach { .. });
    let mut m = match approach {
        true => CaseMachine::once(now_s(), deadline_s),
        false => CaseMachine::new(now_s(), deadline_s),
    };
    m.crash_clear_s = pace.crash_clear_s;
    m.line = Some(line.clone());
    m.expect_give_up = case.expect_give_up.clone();
    let mut script = Script::new(case.later_steps());
    m.pending_steps = script.pending_says();
    // The case's sim_speed and its speed steps are what the spec WANTS; the
    // guard decides when the sim may have it.
    let mut guard = SpeedGuard::new(case.sim_speed, pace.fast_resume_s);
    let verdict = 'case: loop {
        tokio::time::sleep(pace.poll).await;
        let mut done = None;
        for r in tail.poll() {
            let step = m.feed(&r);
            // The script's clocks start at the opening line's answer (an
            // approach's: its sortie), and it hears every record after.
            if !script.armed() && m.opened(approach) {
                script.arm(now_s());
            }
            script.feed(&r);
            guard.feed(&r);
            if let Step::Finished(v) = step {
                done = Some(v);
                break;
            }
        }
        if let Some(v) = done {
            break v;
        }
        if script.armed() && script.pending() > 0 {
            let (want_ias, want_agl) = script.wants_reads();
            let ias = match want_ias {
                true => sim.read(IAS_KTS).await.ok(),
                false => None,
            };
            let agl = match want_agl {
                true => sim
                    .read(crate::datarefs::AGL_METERS)
                    .await
                    .ok()
                    .map(|m| m * M_TO_FT),
                false => None,
            };
            if script.due(now_s(), ias, agl) {
                match script.next_step().cloned() {
                    Some(ScriptStep {
                        sim_speed: Some(n), ..
                    }) => {
                        // Nothing typed: the guard takes the new want.
                        guard.want(n);
                        script.sent(now_s());
                    }
                    Some(s) => {
                        let say = s.say;
                        if let Err(e) = type_line(sim, &say, pace).await {
                            // Whatever was armed may still be flying: taken back.
                            let mut v = failed(format!("step {say:?}: {e}"));
                            v.abort = true;
                            break 'case v;
                        }
                        let t = now_s();
                        script.sent(t);
                        m.note_step_sent(t, &say);
                    }
                    None => {}
                }
            }
        }
        let crashed = sim
            .read(crate::datarefs::HAS_CRASHED)
            .await
            .ok()
            .map(|v| v > 0.5);
        if guard.needs_reads() {
            let (ground, agl, servos, status) = tokio::join!(
                sim.read(crate::datarefs::ON_GROUND_ANY),
                sim.read(crate::datarefs::AGL_METERS),
                sim.read(crate::datarefs::SERVOS_ON),
                sim.read_string(STATUS_AUTO),
            );
            let reads = FastReads {
                crashed,
                on_ground: ground.ok().map(|g| g > 0.5),
                agl_ft: agl.ok().map(|m| m * M_TO_FT),
                servos_on: servos.ok().map(|s| s > 0.5),
                status: status.ok(),
            };
            match guard.tick(now_s(), &reads) {
                SpeedAct::Write { to, reason } => {
                    apply_sim_speed(sim, &mut guard, to, &reason, &case.name).await
                }
                SpeedAct::Held { reason } => record_sim_speed(
                    guard.wanted(),
                    guard.set(),
                    None,
                    None,
                    &format!("held: {reason}"),
                    &case.name,
                    None,
                ),
                SpeedAct::Keep => {}
            }
            let actual = match guard.set().is_some_and(|s| s > 1) {
                true => sim.read(SIM_SPEED_ACTUAL).await.ok(),
                false => None,
            };
            guard.accrue(now_s(), actual);
        }
        match m.tick(now_s(), crashed) {
            Step::Finished(v) => break v,
            Step::Resend => {
                let _ = type_line(sim, &line, pace).await;
            }
            _ => {}
        }
    };
    // Back to 1x before anything else happens to the aircraft.
    if guard.owes_restore() {
        let why = if verdict.crashed {
            "the case ended: crashed"
        } else if verdict.abort {
            "the case ended: taken back"
        } else {
            "the case ended"
        };
        apply_sim_speed(sim, &mut guard, 1, why, &case.name).await;
    }
    let mut verdict = verdict;
    verdict.settle = Some(settle);
    verdict.setup = setup_note;
    verdict.fast_s = guard.fast_s;
    verdict.fast_x = guard.achieved();
    // A script that never finished proved less than it meant to. A speed
    // step that never came is no finding: the case ends at 1x regardless.
    if let Some(s) = script.remaining().iter().find(|s| s.is_say()) {
        verdict.pass = false;
        let why = format!("the script's step {} never came ({:?})", s.words(), s.when);
        verdict.reason = match verdict.reason.is_empty() {
            true => why,
            false => format!("{}; {why}", verdict.reason),
        };
    }
    let on_ground = match verdict.abort {
        true => sim
            .read(crate::datarefs::ON_GROUND_ANY)
            .await
            .ok()
            .map(|g| g > 0.5),
        false => None,
    };
    match hand_back(&verdict, on_ground) {
        HandBack::MyControls => {
            let _ = type_line(sim, ABORT_LINE, pace).await;
        }
        HandBack::LeaveFlying => verdict.left_flying = true,
        HandBack::Nothing => {}
    }
    // `after: stop`: the taxi in is not this case's; the next settle stops it.
    if case.after() == After::Stop && !verdict.abort && !verdict.stop_run {
        verdict.stop_owed = sim
            .read(crate::datarefs::ON_GROUND_ANY)
            .await
            .is_ok_and(|g| g > 0.5);
    }
    verdict
}

/// The run is over: an aircraft still in the air (its loops left flying at a
/// deadline, or a setup that failed after the take-back) is paused, never
/// handed to nobody. The pilot comes back to a frozen sim with whatever was
/// flying it still armed. An unreadable wheels flag pauses too.
pub async fn secure_at_end(sim: &dyn RunnerSim) -> bool {
    let on_ground = sim
        .read(crate::datarefs::ON_GROUND_ANY)
        .await
        .ok()
        .map(|g| g > 0.5);
    if on_ground == Some(true) {
        return false;
    }
    sim.command(PAUSE_ON).await.is_ok()
}

/// Every case, `repeat` times, in order; a row per case as it ends.
pub async fn run_all(
    sim: &dyn RunnerSim,
    tail: &mut BlackboxTail,
    spec: &Spec,
    repeat: u32,
    apt_dat: &str,
    pace: &Pace,
    on_row: &mut (dyn FnMut(&Row) + Send),
) -> Vec<Row> {
    let mut rows: Vec<Row> = Vec::new();
    'run: for case in &spec.cases {
        for rep in 1..=repeat.max(1) {
            let t0 = now_s();
            let left_flying = rows.last().is_some_and(|r| r.verdict.left_flying);
            let stop_now = rows.last().is_some_and(|r| r.verdict.stop_owed);
            let verdict =
                run_case(sim, tail, case, apt_dat, pace, left_flying, stop_now).await;
            let stop = verdict.stop_run;
            let row = Row {
                case: case.name.clone(),
                rep,
                verdict,
                secs: now_s() - t0,
            };
            on_row(&row);
            rows.push(row);
            if stop {
                break 'run;
            }
        }
    }
    if !ensure_real_time(sim, "the run ended").await {
        eprintln!("sim_speed did not read back 1 at the run's end: check it");
    }
    // The last case's stop is owed too: its aircraft parked, not taxiing on.
    if rows.last().is_some_and(|r| r.verdict.stop_owed) {
        if let Err(e) = settle_before_setup(sim, pace, false, true).await {
            eprintln!("the last aircraft after my controls: {e}");
        }
    }
    if secure_at_end(sim).await {
        eprintln!("the run ended with the aircraft in the air: X-Plane is paused");
    }
    rows
}

/// `xvl sortie <spec.yaml> [--repeat N] [--log logs/blackbox.jsonl]`: the
/// entry from main. Returns whether every case passed.
pub async fn main_cmd(args: &[String], base_url: &str, apt_dat: &str) -> Result<bool> {
    let mut spec_path = None;
    let mut repeat = 1u32;
    let mut log = "logs/blackbox.jsonl".to_string();
    let mut it = args.iter();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--repeat" => {
                repeat = it
                    .next()
                    .ok_or_else(|| anyhow!("--repeat needs a number"))?
                    .parse()
                    .map_err(|_| anyhow!("--repeat needs a number"))?
            }
            "--log" => {
                log = it
                    .next()
                    .ok_or_else(|| anyhow!("--log needs a path"))?
                    .clone()
            }
            _ => spec_path = Some(a.clone()),
        }
    }
    let spec_path = spec_path
        .ok_or_else(|| anyhow!("usage: xvl sortie <spec.yaml> [--repeat N] [--log path]"))?;
    let text =
        std::fs::read_to_string(&spec_path).map_err(|e| anyhow!("reading {spec_path}: {e}"))?;
    let mut spec = parse_spec(&text)?;
    if spec.cases.iter().any(Case::uses_plan) {
        spec = with_the_plan(spec).await?;
    }
    if !std::path::Path::new(&log).exists() {
        return Err(anyhow!(
            "{log} not found: run from the xvl checkout the app writes into"
        ));
    }
    let xp = crate::xplane::XPlaneClient::new(base_url.to_string());
    preflight(&xp).await?;
    let stamp = chrono_stamp();
    let out_path = format!("logs/sorties/run-{stamp}.md");
    let _ = std::fs::create_dir_all("logs/sorties");
    eprintln!(
        "sortie run {stamp}: {} cases x {repeat}, table in {out_path}. Ctrl-C takes the aircraft back.",
        spec.cases.len()
    );
    let mut tail = BlackboxTail::at_end(&log);
    let rows = std::sync::Arc::new(std::sync::Mutex::new(Vec::<Row>::new()));
    let write_table = |rows: &[Row]| {
        let _ = std::fs::write(&out_path, table(&spec_path, &stamp, rows));
    };
    let sink = rows.clone();
    let mut on_row = move |r: &Row| {
        // Each row as it ends, flushed: the run is watched through a pipe.
        use std::io::Write;
        let mut out = std::io::stdout().lock();
        let _ = writeln!(out, "{}", row_line(r));
        let _ = out.flush();
        sink.lock()
            .unwrap_or_else(|e| e.into_inner())
            .push(r.clone());
    };
    let run = run_all(
        &xp,
        &mut tail,
        &spec,
        repeat,
        apt_dat,
        &Pace::LIVE,
        &mut on_row,
    );
    tokio::select! {
        _ = run => {}
        _ = tokio::signal::ctrl_c() => {
            // In the air the sim is paused first: the pilot at the keyboard
            // takes the aircraft back with it frozen, not diving, and at 1x.
            let paused = secure_at_end(&xp).await;
            let real_time = ensure_real_time(&xp, "interrupted").await;
            eprintln!(
                "interrupted: my controls{}{}",
                if paused { ", X-Plane paused" } else { "" },
                if real_time { "" } else { "; sim_speed did not read back 1, check it" }
            );
            let _ = type_line(&xp, ABORT_LINE, &Pace::LIVE).await;
            let rows = rows.lock().unwrap_or_else(|e| e.into_inner()).clone();
            write_table(&rows);
            return Ok(false);
        }
    }
    let rows = rows.lock().unwrap_or_else(|e| e.into_inner()).clone();
    write_table(&rows);
    let passed = rows.iter().filter(|r| r.verdict.pass).count();
    println!("{passed} of {} passed. {out_path}", rows.len());
    Ok(passed == rows.len())
}

/// The plan's words in the spec made the pilot's SimBrief plan's facts,
/// fetched once for the run through the app's own `simbrief::fetch`; the
/// aircraft of the plan's type as the set-up tool finds it installed.
async fn with_the_plan(spec: Spec) -> Result<Spec> {
    let id = crate::settings::live().simbrief_id.clone();
    let plan = crate::simbrief::fetch(&id)
        .await
        .map_err(|e| anyhow!("the spec names the flight plan, and it could not be fetched: {e}"))?;
    let facts = PlanFacts::of(&plan);
    eprintln!(
        "flight plan: {} to {} ({}), runways {} and {}",
        facts.origin,
        facts.destination,
        facts.aircraft,
        facts.origin_rwy.as_deref().unwrap_or("-"),
        facts.dest_rwy.as_deref().unwrap_or("-")
    );
    let root = crate::config::xplane_root();
    let find = move |icao: &str| {
        crate::tools::setup::find_acf(crate::tools::setup::aircraft_library(&root), icao)
            .map(str::to_string)
    };
    let cases = spec
        .cases
        .iter()
        .map(|c| resolve_case(c, &facts, &find))
        .collect::<Result<Vec<_>>>()?;
    // An enroute point must be a placement the plan can give (its height,
    // its speed), checked with the plan's type's profile as the app will.
    let speeds = crate::tools::setup_enroute::Speeds {
        profile_kts: crate::aircraft::profile_for_type(&plan.aircraft)
            .and_then(|p| p.reference.and_then(|r| r.cruise_ias_kts)),
        ..Default::default()
    };
    for c in &cases {
        if let Setup::Enroute { point, .. } = &c.setup {
            crate::tools::setup_enroute::place(&plan, point, &speeds)
                .map_err(|e| anyhow!("case {:?}: {e}", c.name))?;
        }
    }
    Ok(Spec {
        cases,
        after: spec.after,
    })
}

/// A UTC stamp for the run's file name, from the clock alone.
fn chrono_stamp() -> String {
    let secs = now_s() as i64;
    let (days, rem) = (secs.div_euclid(86_400), secs.rem_euclid(86_400));
    // Civil date from days since 1970-01-01 (Howard Hinnant's algorithm).
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!(
        "{y:04}{m:02}{d:02}-{:02}{:02}{:02}Z",
        rem / 3600,
        rem % 3600 / 60,
        rem % 60
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    const LEMG: &str = include_str!("../bench/sorties/lemg-737.yaml");

    /// Every shipped spec, parsed against the install when there is one
    /// (a machine without X-Plane checks the shapes, not the paths).
    fn bench_specs() -> Vec<(std::path::PathBuf, Spec)> {
        let installed = std::path::Path::new(&crate::config::xplane_root()).is_dir();
        let mut out = Vec::new();
        for f in std::fs::read_dir("bench/sorties").unwrap().flatten() {
            let p = f.path();
            if p.extension().is_some_and(|e| e == "yaml") {
                let text = std::fs::read_to_string(&p).unwrap();
                let spec = if installed {
                    parse_spec(&text)
                } else {
                    parse_spec_with(&text, &|_| true)
                };
                out.push((
                    p.clone(),
                    spec.unwrap_or_else(|e| panic!("{}: {e}", p.display())),
                ));
            }
        }
        out.sort_by(|a, b| a.0.cmp(&b.0));
        out
    }

    /// The profile a bench aircraft flies with: the detector's own answer
    /// when the `.acf` is installed, pinned to this table either way.
    fn bench_profile(acf: Option<&str>) -> crate::aircraft::AircraftProfile {
        let Some(acf) = acf else {
            return crate::aircraft::load("boeing_737_800").unwrap();
        };
        let file = acf.rsplit('/').next().unwrap();
        let id = match file {
            "b738.acf" => "boeing_737_800",
            "Cirrus SR22.acf" => "cirrus_sr22",
            "CirrusSF50.acf" => "cirrus_vision_sf50",
            "A330.acf" => "airbus_a330_300",
            "Cessna_CitationX.acf" => "citation_x",
            "S-76C.acf" => "sikorsky_s76c",
            other => panic!("{other}: add the bench aircraft to this table"),
        };
        if let Some(icao) = std::fs::read_to_string(crate::config::xplane_path(acf))
            .ok()
            .and_then(|t| crate::tools::setup::acf_icao(&t))
        {
            assert_eq!(
                crate::detect::resolved_id(&icao, acf).as_deref(),
                Some(id),
                "{acf} ({icao}) is detected as another profile"
            );
        }
        crate::aircraft::load(id).unwrap_or_else(|| panic!("profile {id}"))
    }

    /// The tool a typed line reaches, as the command loop routes it: the
    /// local planner's branch first (`flightplan::parse_request`), then the
    /// grammar.
    fn routed(line: &str, prof: &crate::aircraft::AircraftProfile) -> Option<String> {
        if crate::flightplan::parse_request(line).is_some_and(|r| r.dest.is_some()) {
            return Some("plan_flight".into());
        }
        let reg = crate::tools::Registry::for_aircraft(prof);
        crate::grammar::try_parse(line, &reg, prof).map(|c| c[0].name.clone())
    }

    /// Every line a shipped spec types is the app's real phrasing: each
    /// step reaches the tool it names, each command reaches a tool, on the
    /// case's own airframe.
    #[test]
    fn every_scripted_line_routes_to_its_tool() {
        let mut checked = 0;
        for (path, spec) in bench_specs() {
            for c in &spec.cases {
                let prof = bench_profile(c.aircraft.as_deref());
                let at = |what: &str| format!("{} {:?} {what}", path.display(), c.name);
                if let Some(cmd) = &c.command {
                    assert!(routed(cmd, &prof).is_some(), "{}", at(cmd));
                    checked += 1;
                }
                for s in c.steps.iter().filter(|s| s.is_say()) {
                    let want = s
                        .tool
                        .as_deref()
                        .unwrap_or_else(|| panic!("{}: name the tool", at(&s.say)));
                    assert_eq!(
                        routed(&s.say, &prof).as_deref(),
                        Some(want),
                        "{}",
                        at(&s.say)
                    );
                    checked += 1;
                }
            }
        }
        assert!(checked >= 15, "{checked}");
        // The reject is the pilot's command, auto_takeoff's off switch; the
        // crew's radio call is autotakeoff::REJECT_CALL, a different line.
        let b738 = bench_profile(None);
        let reg = crate::tools::Registry::for_aircraft(&b738);
        let calls = crate::grammar::try_parse("reject the takeoff", &reg, &b738).unwrap();
        assert_eq!(calls[0].arguments, json!({"off": true}));
        let calls = crate::grammar::try_parse("resume flight plan", &reg, &b738).unwrap();
        assert_eq!(calls[0].arguments["resume"], true, "{calls:?}");
    }

    /// The aircraft named in a spec is an installed `.acf`, checked when the
    /// spec is read, never found missing at the setup.
    #[test]
    fn a_setup_aircraft_must_be_an_installed_acf() {
        let spec = |a: &str| {
            format!(
                "cases: [{{name: a, setup: {{aircraft: \"{a}\", runway_start: {{field: LEMG, runway: 13}}}}, command: fly a circuit}}]"
            )
        };
        let there = |p: &str| p == "Aircraft/x/y.acf";
        let s = parse_spec_with(&spec("Aircraft/x/y.acf"), &there).unwrap();
        assert_eq!(s.cases[0].aircraft.as_deref(), Some("Aircraft/x/y.acf"));
        let e = parse_spec_with(&spec("Aircraft/x/z.acf"), &there).unwrap_err();
        assert!(e.to_string().contains("is not installed under"), "{e}");
        let e = parse_spec_with(&spec("Aircraft/x/y"), &there).unwrap_err();
        assert!(e.to_string().contains("not an .acf path"), "{e}");
        // "plan" is resolved from the fetched plan, not checked here.
        let s = parse_spec_with(&spec("plan"), &there).unwrap();
        assert!(s.cases[0].uses_plan());
        // Every setup kind takes one.
        for setup in [
            "{aircraft: Aircraft/x/y.acf, approach: {field: LEMG, runway: 13}}",
            "{aircraft: Aircraft/x/y.acf, ramp: {field: LEMG}}",
        ] {
            let text = format!("cases: [{{name: a, setup: {setup}, steps: []}}]").replace(
                "steps: []",
                if setup.contains("ramp") {
                    "command: full auto"
                } else {
                    "deadline_min: 5"
                },
            );
            let s = parse_spec_with(&text, &there).unwrap_or_else(|e| panic!("{setup}: {e}"));
            assert_eq!(s.cases[0].aircraft.as_deref(), Some("Aircraft/x/y.acf"));
        }
    }

    #[test]
    fn steps_parse_every_trigger_and_refuse_a_bad_script() {
        let s = parse_spec_with(
            "cases: [{name: a, setup: {runway_start: {field: LEMG, runway: 13}}, steps: [\n\
               {say: fly a circuit, when: start, tool: full_auto},\n\
               {say: a, when: {after_s: 20}},\n\
               {say: b, when: {ias_above: 60}},\n\
               {say: c, when: {agl_above: 400}},\n\
               {say: d, when: {phase: takeoffroll}},\n\
               {say: e, when: {event: {kind: autotakeoff, event: reject}}},\n\
               {say: f, when: {event: {kind: loading}}}]}]",
            &|_| true,
        )
        .unwrap();
        let c = &s.cases[0];
        assert_eq!(c.first_line(), "fly a circuit");
        let whens: Vec<Trigger> = c.later_steps().into_iter().map(|s| s.when).collect();
        assert_eq!(
            whens,
            vec![
                Trigger::AfterS(20.0),
                Trigger::IasAbove(60.0),
                Trigger::AglAbove(400.0),
                Trigger::Phase("TakeoffRoll".into()),
                Trigger::Event {
                    kind: "autotakeoff".into(),
                    event: Some("reject".into())
                },
                Trigger::Event {
                    kind: "loading".into(),
                    event: None
                },
            ]
        );
        // The command is sugar for the start step; the steps after it.
        let s = parse_spec_with(
            "cases: [{name: a, setup: {ramp: {field: LEMG}}, command: full auto, steps: [{say: x, when: {after_s: 5}}]}]",
            &|_| true,
        )
        .unwrap();
        assert_eq!(s.cases[0].first_line(), "full auto");
        assert_eq!(s.cases[0].later_steps().len(), 1);
        // An approach's steps all come after its setup.
        let s = parse_spec_with(
            "cases: [{name: a, setup: {approach: {field: LEMG, runway: 13}}, steps: [{say: my controls, when: {after_s: 30}}]}]",
            &|_| true,
        )
        .unwrap();
        assert_eq!(s.cases[0].later_steps().len(), 1);
        let bad = [
            ("steps: [{say: x, when: {after_s: 5}}]", "needs a command"),
            ("steps: [{say: x}]", "needs a when"),
            (
                "command: y, steps: [{say: x, when: start}]",
                "the command is the start step",
            ),
            (
                "steps: [{say: x, when: start}, {say: y, when: start}]",
                "only the first step",
            ),
            ("steps: [{say: x, when: soon}]", "when is start"),
            (
                "steps: [{say: x, when: start}, {say: y, when: {phase: Taxiing}}]",
                "phase \"Taxiing\"",
            ),
            (
                "steps: [{say: x, when: start}, {say: y, when: {after_s: -1}}]",
                "zero or more",
            ),
            (
                "steps: [{say: x, when: start}, {say: y, when: {after_s: 1, ias_above: 2}}]",
                "when is start",
            ),
            (
                "steps: [{say: x, when: start}, {say: y, when: {event: {event: z}}}]",
                "needs a kind",
            ),
            ("steps: [{say: x, when: start, tol: z}]", "unknown field"),
            ("command: x, expect_give_up: \" \"", "expect_give_up"),
        ];
        for (tail, want) in bad {
            let text = format!(
                "cases: [{{name: a, setup: {{runway_start: {{field: LEMG, runway: 13}}}}, {tail}}}]"
            );
            let e = parse_spec_with(&text, &|_| true)
                .expect_err(tail)
                .to_string();
            assert!(e.contains(want), "{tail}: {e}");
        }
        let e = parse_spec_with(
            "cases: [{name: a, setup: {approach: {field: LEMG, runway: 13}}, steps: [{say: x, when: start}]}]",
            &|_| true,
        )
        .unwrap_err();
        assert!(e.to_string().contains("no start step"), "{e}");
        let e = parse_spec_with(
            "cases: [{name: a, setup: {approach: {field: LEMG, runway: plan}}}]",
            &|_| true,
        )
        .unwrap_err();
        assert!(e.to_string().contains("runway plan needs"), "{e}");
    }

    /// Every phase a spec may name is one the flight model has: this match
    /// stops compiling when a phase is added, and the names are `{:?}`'s.
    #[test]
    fn the_phase_names_are_the_flight_models() {
        use crate::flight::Phase as P;
        let all = [
            P::Parked,
            P::EngineStart,
            P::Ready,
            P::Pushback,
            P::TaxiOut,
            P::Holding,
            P::LinedUp,
            P::TakeoffRoll,
            P::RejectedTakeoff,
            P::InitialClimb,
            P::Climb,
            P::Cruise,
            P::Descent,
            P::Approach,
            P::Landing,
            P::GoAround,
            P::Hover,
            P::Rollout,
            P::TaxiIn,
        ];
        for p in all {
            match p {
                P::Parked
                | P::EngineStart
                | P::Ready
                | P::Pushback
                | P::TaxiOut
                | P::Holding
                | P::LinedUp
                | P::TakeoffRoll
                | P::RejectedTakeoff
                | P::InitialClimb
                | P::Climb
                | P::Cruise
                | P::Descent
                | P::Approach
                | P::Landing
                | P::GoAround
                | P::Hover
                | P::Rollout
                | P::TaxiIn => {}
            }
        }
        let names: Vec<String> = all.iter().map(|p| format!("{p:?}")).collect();
        assert_eq!(names, PHASES);
    }

    fn step(say: &str, when: Trigger) -> ScriptStep {
        ScriptStep {
            say: say.into(),
            when,
            tool: None,
            sim_speed: None,
        }
    }

    #[test]
    fn each_trigger_fires_on_its_own_measure_after_the_previous_line() {
        let mut s = Script::new(vec![
            step("a", Trigger::AfterS(20.0)),
            step("b", Trigger::IasAbove(60.0)),
            step("c", Trigger::AglAbove(400.0)),
            step("d", Trigger::Phase("Climb".into())),
            step(
                "e",
                Trigger::Event {
                    kind: "autotakeoff".into(),
                    event: Some("reject".into()),
                },
            ),
            step(
                "f",
                Trigger::Event {
                    kind: "loading".into(),
                    event: None,
                },
            ),
        ]);
        // Nothing before the opening line is answered.
        assert!(!s.due(1000.0, Some(200.0), Some(5000.0)));
        s.arm(100.0);
        assert!(!s.due(119.0, None, None));
        assert!(s.due(120.0, None, None), "after_s from the arm");
        s.sent(121.0);
        assert_eq!(s.wants_reads(), (true, false));
        assert!(!s.due(200.0, Some(60.0), None), "above, not at");
        assert!(
            !s.due(200.0, None, None),
            "an unreadable airspeed is no measurement"
        );
        assert!(s.due(200.0, Some(61.0), None));
        s.sent(201.0);
        assert_eq!(s.wants_reads(), (false, true));
        assert!(!s.due(210.0, Some(200.0), Some(390.0)));
        assert!(s.due(210.0, None, Some(410.0)));
        s.sent(211.0);
        // The phase from the state records and the flight model's events.
        s.feed(&json!({"kind": "state", "fphase": "InitialClimb"}));
        assert!(!s.due(212.0, None, None));
        s.feed(&json!({"kind": "flight", "ev": "phase", "from": "InitialClimb", "to": "Climb"}));
        assert!(s.due(213.0, None, None));
        s.sent(214.0);
        // Another event, or another kind, is not it.
        s.feed(&json!({"kind": "autotakeoff", "event": "roll"}));
        s.feed(&json!({"kind": "autoland", "event": "reject"}));
        assert!(!s.due(215.0, None, None));
        s.feed(&json!({"kind": "autotakeoff", "event": "reject", "reason": "pilot"}));
        assert!(s.due(215.0, None, None));
        s.sent(216.0);
        // An event seen before the previous line was typed does not count.
        let mut early = Script::new(vec![
            step("x", Trigger::AfterS(0.0)),
            step(
                "y",
                Trigger::Event {
                    kind: "loading".into(),
                    event: None,
                },
            ),
        ]);
        early.arm(0.0);
        early.feed(&json!({"kind": "loading", "event": "load_aircraft"}));
        early.sent(1.0);
        assert!(!early.due(2.0, None, None));
        // Any event of the kind.
        s.feed(&json!({"kind": "loading", "event": "load_aircraft"}));
        assert!(s.due(217.0, None, None));
        s.sent(218.0);
        assert_eq!(s.pending(), 0);
        assert!(!s.due(1e9, Some(1e3), Some(1e5)));
    }

    #[test]
    fn the_plans_words_become_its_fields_runways_and_type() {
        let plan = PlanFacts {
            origin: "EGBB".into(),
            destination: "EGKK".into(),
            origin_rwy: Some("33".into()),
            dest_rwy: Some("26l".into()),
            aircraft: "B738".into(),
            ..PlanFacts::default()
        };
        let find = |icao: &str| {
            (icao == "B738")
                .then(|| "Aircraft/Laminar Research/Boeing 737-800/b738.acf".to_string())
        };
        let spec = parse_spec_with(
            "cases: [\n\
             {name: a, setup: {aircraft: plan, ramp: {field: plan_origin}}, command: fly the flight plan},\n\
             {name: b, setup: {approach: {field: plan_destination, runway: plan, final_nm: 15}}},\n\
             {name: c, setup: {runway_start: {field: plan_origin, runway: 27}}, command: fly a circuit},\n\
             {name: d, setup: {ramp: {field: LEMG}}, command: full auto}]",
            &|_| false,
        )
        .unwrap();
        assert!(spec.cases[..3].iter().all(Case::uses_plan));
        assert!(!spec.cases[3].uses_plan());
        let got: Vec<Case> = spec
            .cases
            .iter()
            .map(|c| resolve_case(c, &plan, &find).unwrap())
            .collect();
        assert_eq!(
            got[0].setup,
            Setup::Ramp {
                field: "EGBB".into(),
                stand: None
            }
        );
        assert_eq!(
            got[0].aircraft.as_deref(),
            Some("Aircraft/Laminar Research/Boeing 737-800/b738.acf")
        );
        assert_eq!(
            got[1].setup,
            Setup::Approach {
                field: "EGKK".into(),
                runway: "26L".into(),
                final_nm: 15
            }
        );
        assert_eq!(
            got[1].first_line(),
            "put us on a 15 mile final to EGKK runway 26L"
        );
        assert_eq!(
            got[2].setup,
            Setup::RunwayStart {
                field: "EGBB".into(),
                runway: "27".into(),
                engines_running: false
            }
        );
        assert_eq!(got[3].setup, spec.cases[3].setup);
        // What the plan lacks is said, naming the case.
        let bare = PlanFacts {
            aircraft: "A20N".into(),
            ..plan.clone()
        };
        let e = resolve_case(&spec.cases[0], &bare, &find).unwrap_err();
        assert!(
            e.to_string()
                .contains("no installed aircraft of the plan's type A20N"),
            "{e}"
        );
        let no_rwy = PlanFacts {
            dest_rwy: None,
            ..plan
        };
        let e = resolve_case(&spec.cases[1], &no_rwy, &find).unwrap_err();
        assert!(e.to_string().contains("names no runway"), "{e}");
    }

    /// The detector's record for the model the case loads, and no other.
    #[test]
    fn the_swap_is_the_profile_probe_of_the_named_acf() {
        let rec = |p: &str| {
            json!({"kind": "profile_probe", "profile": "cirrus_sr22", "missing": [],
                   "identity": {"icao": "SR22", "relative_path": p}})
        };
        let want = "Aircraft/Laminar Research/Cirrus SR22/Cirrus SR22.acf";
        assert!(probe_is_for(&rec(want), want));
        assert!(probe_is_for(
            &rec("aircraft\\laminar research\\cirrus sr22\\cirrus sr22.acf"),
            want
        ));
        assert!(!probe_is_for(
            &rec("Aircraft/Laminar Research/Boeing 737-800/b738.acf"),
            want
        ));
        assert!(!probe_is_for(
            &json!({"kind": "systems", "identity": {"relative_path": want}}),
            want
        ));
        // A plugin probe or an older record without the identity is no swap.
        assert!(!probe_is_for(
            &json!({"kind": "profile_probe", "profile": "cirrus_sr22"}),
            want
        ));
    }

    fn begin(t: f64, s: &str) -> Value {
        rec(t, "sortie", json!({"event": "begin", "sortie": s}))
    }

    fn result(t: f64, s: &str, pass: bool, outcome: &str, commands: u64) -> Value {
        rec(
            t,
            "sortie_result",
            json!({"pass": pass, "outcome": outcome, "interventions": 0,
                   "commands": commands, "unattended": commands == 0, "sortie": s}),
        )
    }

    fn consumed(t: f64, text: &str) -> Value {
        rec(t, "typed", json!({"event": "consumed", "text": text}))
    }

    /// The resume case: "my controls" ends the approach's sortie on the
    /// way; the verdict is the resume's, typed after it.
    #[test]
    fn a_sortie_the_script_ends_is_not_the_verdict_the_next_one_is() {
        let mut m = CaseMachine::once(0.0, 900.0);
        m.pending_steps = 2;
        m.feed(&rec(0.5, "tool", json!({"name": "setup_approach"})));
        assert!(!m.opened(true), "an approach opens at its sortie");
        assert!(m.opened(false));
        assert_eq!(m.feed(&begin(60.0, "1-approach")), Step::Began);
        assert!(m.opened(true));
        m.note_step_sent(90.0, "my controls");
        m.feed(&consumed(90.2, "my controls"));
        m.feed(&rec(
            90.3,
            "tool",
            json!({"name": "autopilot_off", "sortie": "1-approach"}),
        ));
        m.feed(&rec(
            91.0,
            "stage_result",
            json!({"stage": "landing", "pass": false, "why": ["no touchdown"], "sortie": "1-approach"}),
        ));
        assert_eq!(
            m.feed(&result(91.1, "1-approach", false, "ended", 1)),
            Step::Wait,
            "a step is still to come"
        );
        assert!(matches!(m.phase, Phase::AwaitSortie { .. }));
        // No mission while a step is pending is no failure.
        assert_eq!(m.tick(91.1 + SORTIE_GRACE_S + 5.0, Some(false)), Step::Wait);
        let t = 91.1 + SORTIE_GRACE_S + 10.0;
        m.note_step_sent(t, "resume flight plan");
        m.feed(&consumed(t + 0.1, "resume flight plan"));
        m.feed(&rec(t + 0.2, "tool", json!({"name": "full_auto"})));
        assert_eq!(m.feed(&begin(t + 0.3, "2-full_auto")), Step::Began);
        m.feed(&rec(
            t + 300.0,
            "stage_result",
            json!({"stage": "landing", "pass": true, "why": [], "sortie": "2-full_auto"}),
        ));
        let v = finished(m.feed(&result(t + 301.0, "2-full_auto", true, "ended", 0)));
        assert!(v.pass, "{v:?}");
        assert_eq!(v.sortie.as_deref(), Some("2-full_auto"));
        assert_eq!(v.stages.len(), 1, "only the verdict sortie's stages");
        assert_eq!(v.note, "before it: 1-approach failed (ended)");
        // And the grace runs again once every step is typed.
        let mut m = CaseMachine::new(0.0, 900.0);
        m.pending_steps = 1;
        m.feed(&rec(0.5, "tool", json!({"name": "load_aircraft"})));
        assert_eq!(m.tick(SORTIE_GRACE_S + 10.0, Some(false)), Step::Wait);
        m.note_step_sent(SORTIE_GRACE_S + 20.0, "fly the flight plan");
        assert_eq!(m.tick(2.0 * SORTIE_GRACE_S + 19.0, Some(false)), Step::Wait);
        let v = finished(m.tick(2.0 * SORTIE_GRACE_S + 21.0, Some(false)));
        assert!(v.reason.contains("no mission began"), "{v:?}");
    }

    /// A mission a step supersedes is on the way too; without a step it is
    /// someone else's, and the case's failure.
    #[test]
    fn a_mission_superseded_by_a_step_hands_the_verdict_on() {
        let mut m = CaseMachine::new(0.0, 900.0);
        m.pending_steps = 1;
        m.feed(&rec(0.5, "tool", json!({"name": "full_auto"})));
        m.feed(&begin(1.0, "1-full_auto"));
        m.note_step_sent(30.0, "fly the flight plan");
        m.feed(&consumed(30.1, "fly the flight plan"));
        assert_eq!(
            m.feed(&result(30.2, "1-full_auto", false, "superseded", 1)),
            Step::Wait
        );
        m.feed(&begin(30.3, "2-full_auto"));
        m.feed(&rec(
            99.0,
            "stage_result",
            json!({"stage": "landing", "pass": true, "sortie": "2-full_auto"}),
        ));
        let v = finished(m.feed(&result(100.0, "2-full_auto", true, "ended", 0)));
        assert!(
            v.pass && v.sortie.as_deref() == Some("2-full_auto"),
            "{v:?}"
        );

        let mut m = CaseMachine::new(0.0, 900.0);
        m.feed(&rec(0.5, "tool", json!({"name": "full_auto"})));
        m.feed(&begin(1.0, "1-full_auto"));
        let v = finished(m.feed(&result(30.2, "1-full_auto", false, "superseded", 0)));
        assert!(v.reason.contains("superseded by another mission"), "{v:?}");
    }

    /// The scorer calls a sortie with a pilot's command attended; the
    /// runner knows which commands were its script's.
    #[test]
    fn the_scripts_own_commands_are_not_a_pilots() {
        let landing = |s: &str, pass: bool| {
            rec(
                50.0,
                "stage_result",
                json!({"stage": "landing", "pass": pass, "why": if pass { json!([]) } else { json!(["touched down at 700 fpm"]) }, "sortie": s}),
            )
        };
        let run = |sent: usize, commands: u64, stage_pass: bool| {
            let mut m = CaseMachine::new(0.0, 900.0);
            m.pending_steps = sent;
            m.feed(&rec(0.5, "tool", json!({"name": "full_auto"})));
            m.feed(&begin(1.0, "s"));
            for i in 0..sent {
                let line = format!("line {i}");
                m.note_step_sent(10.0, &line);
                m.feed(&consumed(10.1, &line));
            }
            m.feed(&landing("s", stage_pass));
            finished(m.feed(&result(60.0, "s", false, "ended", commands)))
        };
        let v = run(1, 1, true);
        assert!(v.pass, "{v:?}");
        assert_eq!(v.scripted, 1);
        assert_eq!(v.note, "1 commands were the script's");
        // One more than the script typed: a pilot's.
        let v = run(1, 2, true);
        assert!(!v.pass);
        assert_eq!(
            v.reason,
            "attended (0 interventions, 2 commands, 1 of them the script's)"
        );
        // A failed stage still fails, and the script is not blamed.
        let v = run(1, 1, false);
        assert!(!v.pass);
        assert_eq!(v.reason, "landing: touched down at 700 fpm");
        // No script, a command: attended, as before.
        let v = run(0, 1, true);
        assert!(!v.pass && v.reason.starts_with("attended"), "{v:?}");
        // Ended in the air (#160): the script's words never make that a pass.
        let mut m = CaseMachine::new(0.0, 900.0);
        m.pending_steps = 1;
        m.feed(&rec(0.5, "tool", json!({"name": "full_auto"})));
        m.feed(&begin(1.0, "s"));
        m.note_step_sent(10.0, "line 0");
        m.feed(&consumed(10.1, "line 0"));
        m.feed(&landing("s", true));
        let mut r = result(60.0, "s", false, "ended", 1);
        r["ended_airborne"] = json!(true);
        let v = finished(m.feed(&r));
        assert!(!v.pass, "{v:?}");
        assert_eq!(v.reason, "it ended in the air");
    }

    /// A rejected takeoff is meant to end full auto: that give-up passes,
    /// the roll it cut short too, and a roll off the centreline does not.
    #[test]
    fn a_meant_give_up_passes_and_nothing_else_does() {
        let run = |reason: &str, takeoff_why: Value, interventions: u64| {
            let mut m = CaseMachine::new(0.0, 900.0);
            m.expect_give_up = Some("takeoff_not_completed".into());
            m.pending_steps = 1;
            m.feed(&rec(0.5, "tool", json!({"name": "full_auto"})));
            m.feed(&begin(1.0, "s"));
            m.note_step_sent(30.0, "reject the takeoff");
            m.feed(&consumed(30.1, "reject the takeoff"));
            m.feed(&rec(
                30.2,
                "tool",
                json!({"name": "auto_takeoff", "sortie": "s"}),
            ));
            m.feed(&rec(
                40.0,
                "stage_result",
                json!({"stage": "alignment", "pass": true, "why": [], "sortie": "s"}),
            ));
            let pass = takeoff_why.as_array().is_some_and(|a| a.is_empty());
            m.feed(&rec(
                40.0,
                "stage_result",
                json!({"stage": "takeoff", "pass": pass, "why": takeoff_why, "sortie": "s"}),
            ));
            m.feed(&rec(
                41.0,
                "full_auto",
                json!({"event": "give_up", "reason": reason, "sortie": "s"}),
            ));
            finished(m.feed(&rec(
                42.0,
                "sortie_result",
                json!({"pass": false, "outcome": "gave_up", "interventions": interventions,
                       "commands": 1, "unattended": false, "sortie": "s"}),
            )))
        };
        let v = run("takeoff_not_completed", json!(["ended by give_up"]), 0);
        assert!(v.pass, "{v:?}");
        assert_eq!(
            v.note,
            "gave up as meant (takeoff_not_completed); 1 commands were the script's"
        );
        let v = run(
            "takeoff_not_completed",
            json!(["ended by give_up", "6.1 m off the centreline"]),
            0,
        );
        assert!(!v.pass);
        assert!(v.reason.contains("6.1 m off the centreline"), "{v:?}");
        let v = run("takeoff_rejected", json!(["ended by give_up"]), 0);
        assert!(!v.pass);
        assert!(
            v.reason
                .contains("expected the give-up takeoff_not_completed, the outcome was gave_up (takeoff_rejected)"),
            "{}",
            v.reason
        );
        let v = run("takeoff_not_completed", json!([]), 1);
        assert!(
            !v.pass && v.reason.contains("attended (1 interventions"),
            "{v:?}"
        );
        // The give-up as full auto really writes it (LEMG 13, sortie
        // 1790631201-full_auto): a stage with the reason under detail.
        let mut m = CaseMachine::new(0.0, 900.0);
        m.expect_give_up = Some("takeoff_not_completed".into());
        m.pending_steps = 1;
        m.feed(&rec(0.5, "tool", json!({"name": "full_auto"})));
        m.feed(&begin(1.0, "s"));
        m.note_step_sent(30.0, "reject the takeoff");
        m.feed(&consumed(30.1, "reject the takeoff"));
        m.feed(&rec(30.2, "tool", json!({"name": "auto_takeoff", "sortie": "s"})));
        m.feed(&rec(
            40.0,
            "stage_result",
            json!({"stage": "takeoff", "pass": false, "why": ["ended by reject"], "sortie": "s"}),
        ));
        m.feed(&rec(
            41.0,
            "full_auto",
            json!({"stage": "give_up", "detail": {"reason": "takeoff_not_completed"},
                   "sortie": "s"}),
        ));
        let v = finished(m.feed(&rec(
            42.0,
            "sortie_result",
            json!({"pass": false, "outcome": "gave_up", "interventions": 0,
                   "commands": 1, "unattended": false, "sortie": "s"}),
        )));
        assert!(v.pass, "{v:?}");
    }

    #[test]
    fn the_shipped_spec_parses() {
        let spec = parse_spec(LEMG).expect("lemg spec");
        assert_eq!(spec.cases.len(), 3);
        assert_eq!(
            spec.cases[0].setup,
            Setup::Approach {
                field: "LEMG".into(),
                runway: RUNWAY_FROM_WIND.into(),
                final_nm: 10
            }
        );
        assert_eq!(
            spec.cases[1].setup,
            Setup::Approach {
                field: "LEMG".into(),
                runway: RUNWAY_FROM_WIND.into(),
                final_nm: 5
            }
        );
        assert!(matches!(
            spec.cases[2].setup,
            Setup::RunwayStart {
                engines_running: true,
                ..
            }
        ));
        assert!(spec.cases.iter().all(Case::wants_wind), "#158: the wind's runway");
        assert_eq!(spec.cases[2].command.as_deref(), Some("fly a circuit"));
        assert_eq!(spec.cases[1].deadline_s(), 8.0 * 60.0);
        assert_eq!(spec.cases[0].deadline_s(), 10.0 * 60.0);
        let bare =
            parse_spec("cases: [{name: a, setup: {approach: {field: LEMG, runway: 13}}}]").unwrap();
        assert_eq!(bare.cases[0].deadline_s(), DEFAULT_DEADLINE_MIN * 60.0);
        assert_eq!(
            bare.cases[0].setup,
            Setup::Approach {
                field: "LEMG".into(),
                runway: "13".into(),
                final_nm: 10
            },
            "a final with no distance is the setup tool's ten miles"
        );
        assert!(bench_specs().len() >= 3);
    }

    #[test]
    fn a_bad_spec_is_refused_naming_the_case() {
        let cases = [
            ("cases: []", "no cases"),
            ("cases: [{name: a, setup: {ramp: {field: LEMG}}}]", "needs a command"),
            (
                "cases: [{name: a, setup: {approach: {field: LEMG, runway: 13}}, command: land the aircraft}]",
                "has no command",
            ),
            ("cases: [{name: a, setup: {approach: {field: LEMG, runway: 13, final_nm: 40}}}]", "final_nm"),
            ("cases: [{name: a, setup: {approach: {field: LEMG, runway: 13}}, sim_speed: 0}]", "sim_speed"),
            ("cases: [{name: a, setup: {approach: {field: LEMG, runway: 13}}, deadline_min: 0}]", "deadline"),
            ("cases: [{name: a, setup: {hover: {field: LEMG}}, command: x}]", "unknown field `hover`"),
            (
                "cases: [{name: a, setup: {ramp: {field: LEMG}, runway_start: {field: LEMG, runway: 1}}, command: x}]",
                "exactly one of",
            ),
            ("cases: [{name: a, setup: {ramp: {field: LEMG}}, comand: x}]", "unknown field"),
        ];
        for (yaml, want) in cases {
            let e = parse_spec(yaml).expect_err(yaml).to_string();
            assert!(e.contains(want), "{yaml}: {e}");
        }
        // A runway YAML reads as a number, and a lower-case field, are names.
        let s = parse_spec("cases: [{name: a, setup: {runway_start: {field: lemg, runway: 31}}, command: full auto}]")
            .unwrap();
        assert_eq!(
            s.cases[0].setup,
            Setup::RunwayStart {
                field: "LEMG".into(),
                runway: "31".into(),
                engines_running: false
            }
        );
    }

    #[test]
    fn the_approach_line_is_the_setup_tool_in_the_grammar() {
        let prof = crate::aircraft::load("boeing_737_800").expect("737 profile");
        let reg = crate::tools::Registry::for_aircraft(&prof);
        for (field, rwy, nm, want_rwy) in [("LEMG", "13", 10, "13"), ("EGKK", "26L", 5, "26L")] {
            let line = approach_line(field, rwy, nm);
            let calls =
                crate::grammar::try_parse(&line, &reg, &prof).unwrap_or_else(|| panic!("{line}"));
            assert_eq!(calls[0].name, "setup_approach", "{line}");
            let a = &calls[0].arguments;
            assert_eq!(a["final_nm"], nm, "{line}");
            assert_eq!(a["runway"], want_rwy, "{line}");
            assert_eq!(a["to_codes"], json!([field]), "{line}");
            assert!(a.get("hand_fly").is_none(), "{line}");
        }
        // The shipped runway-start case's command is full auto's circuit,
        // a sortie of its own.
        let spec = parse_spec(LEMG).unwrap();
        let cmd = spec.cases[2].command.clone().unwrap();
        let calls = crate::grammar::try_parse(&cmd, &reg, &prof).expect("circuit command");
        assert_eq!(calls[0].name, "full_auto");
    }

    #[test]
    fn the_runway_start_json_is_the_sdks_shape() {
        let v: Value =
            serde_json::from_str(&runway_start_json("Aircraft/x.acf", "LEMG", "13", true)).unwrap();
        assert_eq!(v["runway_start"]["airport_id"], "LEMG");
        assert_eq!(v["runway_start"]["runway"], "13");
        assert!(v["runway_start"]
            .get("final_distance_in_nautical_miles")
            .is_none());
        assert_eq!(v["engine_status"]["all_engines"]["running"], true);
        assert_eq!(v["aircraft"]["path"], "Aircraft/x.acf");
    }

    fn rec(t: f64, kind: &str, extra: Value) -> Value {
        let mut v = json!({"t": t, "kind": kind});
        for (k, x) in extra.as_object().unwrap() {
            v[k] = x.clone();
        }
        v
    }

    fn finished(s: Step) -> Verdict {
        match s {
            Step::Finished(v) => v,
            other => panic!("not finished: {other:?}"),
        }
    }

    #[test]
    fn a_landing_case_runs_from_the_command_to_the_verdict() {
        let mut m = CaseMachine::new(100.0, 600.0);
        assert_eq!(
            m.feed(&rec(100.5, "tool", json!({"name": "setup_approach"}))),
            Step::Wait
        );
        assert!(matches!(m.phase, Phase::AwaitSortie { .. }));
        // The reload's wait is no failure inside the grace.
        assert_eq!(m.tick(150.0, None), Step::Wait);
        let s = "1790600000-approach";
        assert_eq!(
            m.feed(&rec(
                160.0,
                "sortie",
                json!({"event": "begin", "sortie": s})
            )),
            Step::Began
        );
        // Another sortie's stage is not this case's.
        m.feed(&rec(
            170.0,
            "stage_result",
            json!({"stage": "taxi", "pass": false, "sortie": "x"}),
        ));
        m.feed(&rec(
            300.0,
            "stage_result",
            json!({"stage": "landing", "pass": true, "why": [],
            "sink_fpm": 222.0, "drift_deg": 1.2, "rollout_max_xtk_m": 0.8, "sortie": s}),
        ));
        let v = finished(m.feed(&rec(
            320.0,
            "sortie_result",
            json!({"pass": true, "outcome": "ended",
            "interventions": 0, "commands": 0, "unattended": true, "sortie": s}),
        )));
        assert!(v.pass, "{v:?}");
        assert_eq!(v.sortie.as_deref(), Some(s));
        assert_eq!(v.stages.len(), 1);
        assert!(v.reason.is_empty());
        assert!(!v.abort);
        assert_eq!(
            m.tick(330.0, Some(true)),
            Step::Wait,
            "a finished case stays finished"
        );
    }

    #[test]
    fn a_failed_sortie_names_its_stages_and_the_hands() {
        let mut m = CaseMachine::new(0.0, 600.0);
        m.feed(&rec(1.0, "tool", json!({"name": "full_auto"})));
        m.feed(&rec(
            5.0,
            "sortie",
            json!({"event": "begin", "sortie": "9-full_auto"}),
        ));
        m.feed(&rec(
            50.0,
            "full_auto",
            json!({"event": "give_up", "reason": "stagnant", "sortie": "9-full_auto"}),
        ));
        m.feed(&rec(
            50.0,
            "stage_result",
            json!({"stage": "landing", "pass": false,
            "why": ["touched down at 794 fpm", "bounced to 12 ft"], "sortie": "9-full_auto"}),
        ));
        let v = finished(m.feed(&rec(
            51.0,
            "sortie_result",
            json!({"pass": false, "outcome": "gave_up",
            "interventions": 1, "commands": 0, "unattended": false, "sortie": "9-full_auto"}),
        )));
        assert!(!v.pass);
        assert_eq!(
            v.reason,
            "landing: touched down at 794 fpm, bounced to 12 ft; gave up (stagnant); attended (1 interventions, 0 commands)"
        );
        assert_eq!(v.interventions, 1);
    }

    #[test]
    fn a_command_nobody_takes_is_sent_again_once_then_fails() {
        let mut m = CaseMachine::new(0.0, 600.0);
        assert_eq!(m.tick(4.0, Some(false)), Step::Wait);
        assert_eq!(m.tick(5.5, Some(false)), Step::Resend);
        assert_eq!(m.tick(9.0, Some(false)), Step::Wait);
        let v = finished(m.tick(11.0, Some(false)));
        assert!(v.reason.contains("did not take"), "{v:?}");
        assert!(!v.abort, "nothing was armed, nothing to take back");
        // A crew line is an answer too: the resend's clock is stopped.
        let mut m = CaseMachine::new(0.0, 600.0);
        m.feed(&rec(
            2.0,
            "said",
            json!({"who": "crew", "text": "Setting up a 5 mile final."}),
        ));
        assert!(matches!(m.phase, Phase::AwaitSortie { .. }));
        // A setup line is never typed twice.
        let mut m = CaseMachine::once(0.0, 600.0);
        let v = finished(m.tick(5.5, Some(false)));
        assert!(v.reason.contains("did not take"), "{v:?}");
    }

    /// LEMG 2026-09-28 (sortie 1790586620, #135): "fly a circuit" was taken
    /// by the app's poller at 1790586608.6, retyped at 614.4, failed as "the
    /// app did not take the command", and both copies were read at 620.2 and
    /// 620.3 behind a tower reply: the mission flew unobserved and the second
    /// copy got "I'm already running the full sequence". A TAKEN line is
    /// waited for, never retyped, and the case follows the mission it began.
    #[test]
    fn a_line_the_app_has_taken_is_waited_for_never_typed_again() {
        let t0 = 1790586608.3;
        let mut m = CaseMachine::new(t0, 1200.0);
        m.line = Some("fly a circuit".into());
        m.feed(&rec(
            1790586608.6,
            "typed",
            json!({"event": "taken", "text": "fly a circuit", "age_s": 0.25}),
        ));
        assert!(matches!(m.phase, Phase::Queued { .. }));
        // The old clocks: a resend at +5 s, a failure at +10 s. Neither now.
        assert_eq!(m.tick(t0 + 5.5, Some(false)), Step::Wait);
        assert_eq!(m.tick(t0 + 11.0, Some(false)), Step::Wait);
        // A crew line while it waits is chatter (the After landing list).
        m.feed(&rec(1790586615.0, "said", json!({"who": "crew", "text": "After landing."})));
        assert!(matches!(m.phase, Phase::Queued { .. }));
        m.feed(&rec(
            1790586620.2,
            "typed",
            json!({"event": "consumed", "text": "fly a circuit", "queued_ms": 11663}),
        ));
        m.feed(&rec(1790586620.2, "tool", json!({"name": "full_auto"})));
        assert!(matches!(m.phase, Phase::AwaitSortie { .. }));
        assert_eq!(
            m.feed(&rec(
                1790586620.3,
                "sortie",
                json!({"event": "begin", "sortie": "1790586620-full_auto"}),
            )),
            Step::Began
        );

        // Taken and never read: failed, and the aircraft taken back, since
        // the app still holds the line and may act on it later.
        let mut m = CaseMachine::new(0.0, 1200.0);
        m.line = Some("fly a circuit".into());
        m.feed(&rec(0.3, "typed", json!({"event": "taken", "text": "fly a circuit"})));
        assert_eq!(m.tick(59.0, Some(false)), Step::Wait);
        let v = finished(m.tick(61.0, Some(false)));
        assert!(v.reason.contains("never read it"), "{v:?}");
        assert!(v.abort);

        // Another line taken (the pilot's own) is not this case's.
        let mut m = CaseMachine::new(0.0, 1200.0);
        m.line = Some("fly a circuit".into());
        m.feed(&rec(0.3, "typed", json!({"event": "taken", "text": "my controls"})));
        assert_eq!(m.tick(5.5, Some(false)), Step::Resend);

        // Read at once and never answered: no second copy either.
        let mut m = CaseMachine::new(0.0, 1200.0);
        m.feed(&rec(0.3, "typed", json!({"event": "taken", "text": "full auto"})));
        m.feed(&rec(0.4, "typed", json!({"event": "consumed", "text": "full auto"})));
        let v = finished(m.tick(5.5, Some(false)));
        assert!(v.reason.contains("did not take"), "{v:?}");
    }

    #[test]
    fn a_setup_that_never_arms_fails_with_the_crews_words() {
        let mut m = CaseMachine::new(0.0, 600.0);
        m.feed(&rec(1.0, "tool", json!({"name": "setup_approach"})));
        m.feed(&rec(
            2.0,
            "said",
            json!({"who": "crew", "text": "The approach setup didn't answer. Check X-Plane."}),
        ));
        let v = finished(m.feed(&rec(
            3.0,
            "setup",
            json!({"event": "approach_result", "outcome": "timeout",
            "result": null, "armed": false}),
        )));
        assert!(v.reason.contains("did not arm (timeout"), "{}", v.reason);
        assert!(v.reason.contains("didn't answer"), "{}", v.reason);
        // Taken, and no mission ever began: the grace ends it, with the
        // refusal the crew spoke.
        let mut m = CaseMachine::new(0.0, 6000.0);
        m.feed(&rec(1.0, "tool", json!({"name": "full_auto"})));
        m.feed(&rec(
            1.5,
            "said",
            json!({"who": "crew", "text": "We're in replay."}),
        ));
        assert_eq!(m.tick(100.0, Some(false)), Step::Wait);
        let v = finished(m.tick(1.0 + SORTIE_GRACE_S + 1.0, Some(false)));
        assert_eq!(
            v.reason,
            "no mission began after full_auto; the crew said \"We're in replay.\""
        );
        assert!(v.abort);
    }

    #[test]
    fn a_crash_or_the_deadline_ends_the_case_and_takes_the_aircraft_back() {
        let mut m = CaseMachine::new(0.0, 600.0);
        m.feed(&rec(1.0, "tool", json!({"name": "full_auto"})));
        m.feed(&rec(
            2.0,
            "sortie",
            json!({"event": "begin", "sortie": "s"}),
        ));
        // A failed read is no measurement, never a crash.
        assert_eq!(m.tick(10.0, None), Step::Wait);
        assert_eq!(m.tick(10.5, Some(false)), Step::Wait);
        let v = finished(m.tick(11.0, Some(true)));
        assert_eq!(v.reason, "the aircraft crashed");
        assert_eq!(v.sortie.as_deref(), Some("s"));
        assert!(v.abort && v.crashed && !v.stop_run);
        assert_eq!(
            hand_back(&v, Some(false)),
            HandBack::MyControls,
            "a wreck is nobody's flight"
        );
        let mut m = CaseMachine::new(0.0, 600.0);
        m.feed(&rec(1.0, "tool", json!({"name": "full_auto"})));
        m.feed(&rec(
            2.0,
            "sortie",
            json!({"event": "begin", "sortie": "s"}),
        ));
        assert_eq!(m.tick(600.0, Some(false)), Step::Wait);
        let v = finished(m.tick(601.0, Some(false)));
        assert_eq!(v.reason, "deadline 10 min passed");
        assert!(v.abort && !v.pass && !v.crashed);
    }

    /// LEMG 2026-09-28: 1790584266-approach crashed, and the next three cases
    /// each failed in 2 s as "the aircraft crashed", no sortie: the wreck
    /// flag stays up until the sim resets, and the new case's setup is that
    /// reset.
    #[test]
    fn the_last_cases_wreck_is_waited_out_then_stops_the_run() {
        // The setup's reload clears it: the case flies on, and a wreck
        // after that is this case's.
        let mut m = CaseMachine::once(0.0, 600.0);
        assert_eq!(m.tick(1.0, Some(true)), Step::Wait, "the last case's wreck");
        m.feed(&rec(1.5, "tool", json!({"name": "setup_approach"})));
        assert_eq!(m.tick(60.0, Some(true)), Step::Wait, "still reloading");
        assert_eq!(
            m.tick(70.0, Some(false)),
            Step::Wait,
            "the reload cleared it"
        );
        m.feed(&rec(
            71.0,
            "sortie",
            json!({"event": "begin", "sortie": "s"}),
        ));
        let v = finished(m.tick(200.0, Some(true)));
        assert_eq!(v.reason, "the aircraft crashed");
        assert!(v.crashed && !v.stop_run);

        // It never clears: one line, and the run stops.
        let mut m = CaseMachine::once(0.0, 600.0);
        m.feed(&rec(1.5, "tool", json!({"name": "setup_approach"})));
        assert_eq!(m.tick(CRASH_CLEAR_S, Some(true)), Step::Wait);
        let v = finished(m.tick(CRASH_CLEAR_S + 0.5, Some(true)));
        assert_eq!(v.reason, STILL_CRASHED);
        assert!(v.stop_run && !v.crashed && !v.abort, "{v:?}");
        assert_eq!(hand_back(&v, None), HandBack::Nothing);
    }

    #[test]
    fn a_case_ended_in_the_air_is_never_handed_to_nobody() {
        let deadline = Verdict {
            abort: true,
            reason: "deadline 10 min passed".into(),
            ..Verdict::default()
        };
        // 1,600 ft on a 5 nm final: the loops keep it.
        assert_eq!(hand_back(&deadline, Some(false)), HandBack::LeaveFlying);
        // Unreadable wheels are no measurement: treated as flying.
        assert_eq!(hand_back(&deadline, None), HandBack::LeaveFlying);
        assert_eq!(hand_back(&deadline, Some(true)), HandBack::MyControls);
        assert_eq!(
            hand_back(&Verdict::default(), Some(false)),
            HandBack::Nothing
        );
    }

    #[test]
    fn the_run_table_has_a_row_per_case_with_numbers_and_limits() {
        let pass = Row {
            case: "LEMG 13, 5 nm final".into(),
            rep: 2,
            secs: 181.4,
            verdict: Verdict {
                pass: true,
                sortie: Some("1790600000-approach".into()),
                stages: vec![json!({"stage": "landing", "pass": true, "sink_fpm": 222.0,
                    "drift_deg": 1.2, "rollout_max_xtk_m": 0.8})],
                outcome: "ended".into(),
                ..Verdict::default()
            },
        };
        let fail = Row {
            case: "LEMG circuit".into(),
            rep: 1,
            secs: 900.0,
            verdict: Verdict {
                sortie: Some("1790600300-full_auto".into()),
                stages: vec![
                    json!({"stage": "taxi", "pass": false, "max_lat_m": 5.8, "max_cut_m": 1.0}),
                    json!({"stage": "alignment", "pass": true, "err_deg": 0.4, "xtk_m": 0.3}),
                    json!({"stage": "takeoff", "pass": true, "max_xtk_m": 1.1}),
                ],
                interventions: 1,
                reason: "taxi: 5.8 m off the line | x".into(),
                settle: Some(Settle {
                    secs: 42.3,
                    end: SettleEnd::AtRest,
                }),
                fast_s: 600.2,
                fast_x: Some(3.94),
                ..Verdict::default()
            },
        };
        assert!(
            row_line(&fail).contains("in 900 s (fast 600 s at x3.9) (waited 42 s"),
            "{}",
            row_line(&fail)
        );
        assert_eq!(
            row_line(&pass),
            "PASS LEMG 13, 5 nm final #2: landing pass (sink 222/400 fpm, drift 1.2/5 deg, rollout 0.8/5 m) \
             in 181 s, sortie 1790600000-approach, 0 interventions"
        );
        let t = table(
            "bench/sorties/lemg-737.yaml",
            "20260928-010203Z",
            &[pass, fail],
        );
        let lines: Vec<&str> = t.lines().collect();
        assert_eq!(lines[0], "# Sortie run 20260928-010203Z");
        assert!(lines[4].starts_with("| Case |"), "{t}");
        assert_eq!(
            lines[7],
            "| LEMG circuit | 1 | 1790600300-full_auto | FAIL | taxi FAIL (lat 5.8/3 m, cut 1/5 m); \
             alignment pass (err 0.4/3 deg, xtk 0.3/2 m); takeoff pass (xtk 1.1/5 m) | 1 | 900 | 600 s at x3.9 | \
             waited 42 s for the last aircraft to stop | taxi: 5.8 m off the line / x |"
        );
        assert!(
            lines[6].contains("| 181 | - |"),
            "nothing waited for is a dash: {t}"
        );
        assert_eq!(*lines.last().unwrap(), "1 of 2 passed.");
    }

    #[test]
    fn the_tail_reads_whole_new_lines_and_survives_a_rotation() {
        use std::io::Write;
        let dir = std::env::temp_dir().join(format!("xvl-tail-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join("bb.jsonl");
        std::fs::write(&p, "{\"kind\":\"old\"}\n").unwrap();
        let mut tail = BlackboxTail::at_end(&p);
        assert!(tail.poll().is_empty(), "what was there before is not fed");
        let mut f = std::fs::OpenOptions::new().append(true).open(&p).unwrap();
        write!(f, "{{\"kind\":\"a\"}}\n{{\"kind\":\"b\"").unwrap();
        f.flush().unwrap();
        let got = tail.poll();
        assert_eq!(got.len(), 1, "a half-written line waits");
        assert_eq!(got[0]["kind"], "a");
        writeln!(f, "}}").unwrap();
        f.flush().unwrap();
        assert_eq!(tail.poll()[0]["kind"], "b");
        // Rotated: the new, shorter file from its start.
        std::fs::write(&p, "{\"kind\":\"c\"}\n").unwrap();
        assert_eq!(tail.poll()[0]["kind"], "c");
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// #144: a corrupt line (two records interleaved, as the 2026-09-28 log
    /// held 70 of) is skipped and the whole line after it is still read; a
    /// multi-byte character split across two polls arrives intact.
    #[test]
    fn the_tail_skips_a_corrupt_line_and_keeps_the_next() {
        use std::io::Write;
        let dir = std::env::temp_dir().join(format!("xvl-tail-bad-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join("bb.jsonl");
        std::fs::write(&p, "").unwrap();
        let mut tail = BlackboxTail::at_end(&p);
        let mut f = std::fs::OpenOptions::new().append(true).open(&p).unwrap();
        write!(
            f,
            "{{\"{{dtcache_hits\"\"::0.6,,\"event\":\"power\"\n\
             throttle_before\":0.27,\"write_ok\":true}}\n\
             {{\"kind\":\"stage_result\",\"stage\":\"landing\"}}\n"
        )
        .unwrap();
        f.flush().unwrap();
        let got = tail.poll();
        assert_eq!(got.len(), 1, "{got:?}");
        assert_eq!(got[0]["kind"], "stage_result");
        let line = "{\"kind\":\"said\",\"text\":\"10\u{b0}C\"}\n".as_bytes();
        let cut = line.iter().position(|&b| b == 0xC2).unwrap() + 1;
        f.write_all(&line[..cut]).unwrap();
        f.flush().unwrap();
        assert!(tail.poll().is_empty());
        f.write_all(&line[cut..]).unwrap();
        f.flush().unwrap();
        assert_eq!(tail.poll()[0]["text"], "10\u{b0}C");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn the_stamp_is_a_utc_date() {
        let s = chrono_stamp();
        assert_eq!(s.len(), 16, "{s}");
        assert!(s.starts_with("20") && s.ends_with('Z'), "{s}");
    }

    /// A sim that answers the runner's reads, takes typed lines at once, and
    /// writes the app's blackbox records for the approach it was typed.
    struct FakeSim {
        typed: std::sync::Mutex<Vec<String>>,
        speeds: std::sync::Mutex<Vec<f64>>,
        log: std::path::PathBuf,
        flying: std::sync::atomic::AtomicBool,
        inits: std::sync::Mutex<Vec<String>>,
        /// The wreck flag, as the sim reports it.
        crashed: std::sync::atomic::AtomicBool,
        /// The wheels are off the ground.
        airborne: std::sync::atomic::AtomicBool,
        /// A typed mission begins and then never ends (the deadline's case).
        endless: std::sync::atomic::AtomicBool,
        commands: std::sync::Mutex<Vec<String>>,
        /// After a mission's verdict the aircraft rolls on (its taxi-in, no
        /// loop on the marker, as LEMG 31 read) for this many groundspeed
        /// reads; `u32::MAX` until "my controls".
        roll_after_mission: std::sync::atomic::AtomicU32,
        rolling_reads: std::sync::atomic::AtomicU32,
        /// A setup (mailbox post or approach line) while it rolled.
        setups_while_rolling: std::sync::atomic::AtomicU32,
        /// A mailbox post loading another aircraft is answered by the app's
        /// detector (`profile_probe` of it).
        probe_on_swap: std::sync::atomic::AtomicBool,
        /// The METAR on file for any field (direction true, knots).
        metar: std::sync::Mutex<Option<(f64, f64)>>,
        /// The aircraft's position and the wind at it (m/s).
        at: std::sync::Mutex<(f64, f64, f64, f64)>,
        /// Every aircraft's approach speed.
        vapp: std::sync::Mutex<Option<f64>>,
        /// `sim/time/sim_speed` (and what it achieves).
        speed: std::sync::Mutex<f64>,
        /// The follower's cruise: 3,000 m up, the servos in, the panel on
        /// full auto's cruise, and an endless mission's model in Cruise.
        enroute: std::sync::atomic::AtomicBool,
        /// The first fast write wrecks the aircraft.
        crash_when_fast: std::sync::atomic::AtomicBool,
        /// The app refuses a typed enroute setup (its `enroute_refused`).
        enroute_refuses: std::sync::atomic::AtomicBool,
    }

    /// The fake's taxi-in pace, 14 kt in metres per second.
    const ROLL_MPS: f64 = 7.2;

    impl FakeSim {
        /// A setup reloads the sim: whatever rolled is stopped, and counted.
        fn setup_posted(&self) {
            use std::sync::atomic::Ordering::SeqCst;
            if self.rolling_reads.swap(0, SeqCst) > 0 {
                self.setups_while_rolling.fetch_add(1, SeqCst);
            }
        }
        fn new(log: std::path::PathBuf) -> Self {
            Self {
                typed: Default::default(),
                speeds: Default::default(),
                log,
                flying: false.into(),
                inits: Default::default(),
                crashed: false.into(),
                airborne: false.into(),
                endless: false.into(),
                commands: Default::default(),
                roll_after_mission: 0.into(),
                rolling_reads: 0.into(),
                setups_while_rolling: 0.into(),
                probe_on_swap: false.into(),
                metar: Default::default(),
                at: Default::default(),
                vapp: Default::default(),
                speed: std::sync::Mutex::new(1.0),
                enroute: false.into(),
                crash_when_fast: false.into(),
                enroute_refuses: false.into(),
            }
        }
    }

    #[async_trait]
    impl RunnerSim for FakeSim {
        async fn read(&self, name: &str) -> Result<f64> {
            use std::sync::atomic::Ordering::SeqCst;
            let posted = self.inits.lock().unwrap().len() as f64;
            let flag = |b: bool| if b { 1.0 } else { 0.0 };
            Ok(match name {
                "xcopilot/sim/init_seq" => posted,
                crate::datarefs::ON_GROUND_ANY => flag(!self.airborne.load(SeqCst)),
                crate::datarefs::HAS_CRASHED => flag(self.crashed.load(SeqCst)),
                crate::datarefs::LATITUDE => self.at.lock().unwrap().0,
                crate::datarefs::LONGITUDE => self.at.lock().unwrap().1,
                crate::datarefs::WIND_DIR_DEG_TRUE => self.at.lock().unwrap().2,
                crate::datarefs::WIND_SPEED_MSC => self.at.lock().unwrap().3,
                SIM_SPEED | SIM_SPEED_ACTUAL => *self.speed.lock().unwrap(),
                crate::datarefs::AGL_METERS if self.enroute.load(SeqCst) => 3_000.0,
                crate::datarefs::SERVOS_ON => flag(self.enroute.load(SeqCst)),
                GROUNDSPEED => {
                    let left = self.rolling_reads.load(SeqCst);
                    if left > 0 && left != u32::MAX {
                        self.rolling_reads.store(left - 1, SeqCst);
                    }
                    if left > 0 {
                        ROLL_MPS
                    } else {
                        0.0
                    }
                }
                _ => 0.0,
            })
        }
        async fn read_string(&self, name: &str) -> Result<String> {
            Ok(match name {
                STATUS_AUTO
                    if self.enroute.load(std::sync::atomic::Ordering::SeqCst)
                        && self.flying.load(std::sync::atomic::Ordering::SeqCst) =>
                {
                    "1FULL AUTO|cruise\nflying a procedure: to PIMOS, 42.0 nm".into()
                }
                STATUS_AUTO if self.flying.load(std::sync::atomic::Ordering::SeqCst) => {
                    "1AUTOLAND|final".into()
                }
                STATUS_AUTO => "0MONITORING|".into(),
                ACF_PATH => "Aircraft/Laminar Research/Boeing 737-800/b738.acf".into(),
                _ => String::new(),
            })
        }
        async fn write_float(&self, name: &str, v: f64) -> Result<()> {
            use std::sync::atomic::Ordering::SeqCst;
            assert_eq!(name, SIM_SPEED);
            self.speeds.lock().unwrap().push(v);
            *self.speed.lock().unwrap() = v;
            if v > 1.0 && self.crash_when_fast.load(SeqCst) {
                self.crashed.store(true, SeqCst);
            }
            Ok(())
        }
        async fn command(&self, name: &str) -> Result<()> {
            self.commands.lock().unwrap().push(name.to_string());
            Ok(())
        }
        async fn metar_wind(&self, _icao: &str) -> Option<(f64, f64)> {
            *self.metar.lock().unwrap()
        }
        async fn approach_kts(&self, _acf: &str) -> Option<f64> {
            *self.vapp.lock().unwrap()
        }
        async fn write_bytes(&self, name: &str, bytes: &[u8]) -> Result<()> {
            use std::sync::atomic::Ordering::SeqCst;
            let line = String::from_utf8_lossy(bytes).to_string();
            if name == "xcopilot/sim/init_flight" {
                self.setup_posted();
                let v: Value = serde_json::from_str(&line).unwrap();
                let acf = v["aircraft"]["path"].as_str().unwrap_or("").to_string();
                self.inits.lock().unwrap().push(line);
                if self.probe_on_swap.load(SeqCst) && !acf.ends_with("b738.acf") {
                    use std::io::Write;
                    let mut f = std::fs::OpenOptions::new()
                        .append(true)
                        .open(&self.log)
                        .unwrap();
                    let r = json!({"t": 1.0, "kind": "profile_probe", "missing": [],
                                   "identity": {"relative_path": acf}});
                    writeln!(f, "{r}").unwrap();
                }
                return Ok(());
            }
            if line.starts_with("put us on") {
                self.setup_posted();
            }
            if line.starts_with("put us enroute") {
                // The app's enroute setup: its tool, the new flight in the
                // air with the servos confirmed (or its refusal).
                self.setup_posted();
                self.typed.lock().unwrap().push(line.clone());
                use std::io::Write;
                let mut f = std::fs::OpenOptions::new()
                    .append(true)
                    .open(&self.log)
                    .unwrap();
                let end = if self.enroute_refuses.load(SeqCst) {
                    json!({"t": 2.0, "kind": "setup", "event": "enroute_refused",
                           "why": "The plan is for a A320, and this is a B738."})
                } else {
                    self.airborne.store(true, SeqCst);
                    json!({"t": 2.0, "kind": "setup", "event": "enroute_result", "outcome": "ok",
                           "engaged": true, "next_fix": "WELIN"})
                };
                for r in [json!({"t": 1.0, "kind": "tool", "name": "setup_enroute"}), end] {
                    writeln!(f, "{r}").unwrap();
                }
                return Ok(());
            }
            assert_eq!(name, TYPED);
            self.typed.lock().unwrap().push(line.clone());
            use std::io::Write;
            let mut f = std::fs::OpenOptions::new()
                .append(true)
                .open(&self.log)
                .unwrap();
            if line == ABORT_LINE {
                self.flying.store(false, SeqCst);
                // hand_back_hold: brakes to a stop in a few reads.
                let left = self.rolling_reads.load(SeqCst);
                self.rolling_reads.store(left.min(3), SeqCst);
            } else if self.endless.load(SeqCst) {
                let s = format!("{}-endless", self.typed.lock().unwrap().len());
                self.flying.store(true, SeqCst);
                for r in [
                    json!({"t": 1.0, "kind": "tool", "name": "setup_approach"}),
                    json!({"t": 2.0, "kind": "sortie", "event": "begin", "sortie": s}),
                ] {
                    writeln!(f, "{r}").unwrap();
                }
                if self.enroute.load(SeqCst) {
                    let r = json!({"t": 3.0, "kind": "state", "fphase": "Cruise", "sortie": s});
                    writeln!(f, "{r}").unwrap();
                }
            } else {
                let s = "77-mission";
                let tool = if line.starts_with("put us on") {
                    "setup_approach"
                } else {
                    "full_auto"
                };
                for r in [
                    json!({"t": 1.0, "kind": "tool", "name": tool}),
                    json!({"t": 2.0, "kind": "sortie", "event": "begin", "sortie": s}),
                    json!({"t": 3.0, "kind": "stage_result", "stage": "landing", "pass": true,
                           "why": [], "sink_fpm": 180.0, "sortie": s}),
                    json!({"t": 4.0, "kind": "sortie_result", "pass": true, "outcome": "ended",
                           "interventions": 0, "commands": 0, "unattended": true, "sortie": s}),
                ] {
                    writeln!(f, "{r}").unwrap();
                }
                self.rolling_reads
                    .store(self.roll_after_mission.load(SeqCst), SeqCst);
            }
            Ok(())
        }
    }

    #[tokio::test]
    async fn the_driver_flies_a_spec_against_a_fake_sim() {
        let dir = std::env::temp_dir().join(format!("xvl-runner-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let log = dir.join("blackbox.jsonl");
        std::fs::write(&log, "").unwrap();
        let sim = FakeSim::new(log.clone());
        let spec = parse_spec(
            "cases: [{name: five, setup: {approach: {field: LEMG, runway: 13, final_nm: 5}}, sim_speed: 2},\n\
                     {name: circuit, setup: {runway_start: {field: LEMG, runway: 13, engines_running: true}},\n\
                      command: fly a circuit}]",
        )
        .unwrap();
        let pace = Pace {
            poll: std::time::Duration::from_millis(5),
            init: InitTiming::LIVE,
            still_s: 0.0,
            still_max_s: 1.0,
            settle_scale: 0.0,
            idle_max_s: 1.0,
            quiet_s: 0.02,
            stop_max_s: 0.5,
            crash_clear_s: 0.2,
            swap_max_s: 1.0,
            fast_resume_s: 0.0,
            enroute_max_s: 1.0,
        };
        let mut tail = BlackboxTail::at_end(&log);
        let mut seen = Vec::new();
        let rows = run_all(&sim, &mut tail, &spec, 2, "", &pace, &mut |r: &Row| {
            seen.push(row_line(r))
        })
        .await;
        assert_eq!(rows.len(), 4);
        assert!(rows.iter().all(|r| r.verdict.pass), "{seen:?}");
        assert_eq!(rows[1].rep, 2);
        assert_eq!(
            *sim.typed.lock().unwrap(),
            vec![
                "put us on a 5 mile final to LEMG runway 13",
                "put us on a 5 mile final to LEMG runway 13",
                "fly a circuit",
                "fly a circuit",
            ],
            "one setup per case, never a second while one runs"
        );
        // The runway start went through the mailbox, once per case.
        let inits = sim.inits.lock().unwrap().clone();
        assert_eq!(inits.len(), 2);
        let v: Value = serde_json::from_str(&inits[0]).unwrap();
        assert_eq!(v["runway_start"]["runway"], "13");
        assert_eq!(v["engine_status"]["all_engines"]["running"], true);
        assert!(
            sim.speeds.lock().unwrap().is_empty(),
            "the case's 2x is never flown on the ground (the guard)"
        );
        // A flying loop refuses the run.
        sim.flying.store(true, std::sync::atomic::Ordering::SeqCst);
        assert!(preflight(&sim)
            .await
            .unwrap_err()
            .to_string()
            .contains("AUTOLAND|final"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A case naming another aircraft loads it (an approach: on its runway,
    /// engines running, before the app's own setup) and waits for the app's
    /// detector to have swapped to it; one never taken is a setup failure.
    #[tokio::test]
    async fn a_named_aircraft_is_loaded_and_waited_for_before_the_first_line() {
        use std::sync::atomic::Ordering::SeqCst;
        let (dir, log) = scratch_log("swap");
        let sim = FakeSim::new(log.clone());
        let sr22 = "Aircraft/Laminar Research/Cirrus SR22/Cirrus SR22.acf";
        let case = |a: &str| {
            parse_spec_with(
                &format!(
                    "cases: [{{name: a, setup: {{aircraft: \"{a}\", approach: {{field: LEMG, runway: 13, final_nm: 10}}}}}}]"
                ),
                &|_| true,
            )
            .unwrap()
            .cases
            .remove(0)
        };
        let mut tail = BlackboxTail::at_end(&log);
        // The loaded aircraft: the approach tool sets it up, nothing posted.
        let same = case("Aircraft/Laminar Research/Boeing 737-800/b738.acf");
        set_up_start(&sim, &mut tail, &same, "", &quick_pace())
            .await
            .unwrap();
        assert!(sim.inits.lock().unwrap().is_empty());
        // Another: loaded on the runway with engines running, then the swap.
        sim.probe_on_swap.store(true, SeqCst);
        set_up_start(&sim, &mut tail, &case(sr22), "", &quick_pace())
            .await
            .unwrap();
        let inits = sim.inits.lock().unwrap().clone();
        assert_eq!(inits.len(), 1);
        let v: Value = serde_json::from_str(&inits[0]).unwrap();
        assert_eq!(v["aircraft"]["path"], sr22);
        assert_eq!(v["runway_start"]["airport_id"], "LEMG");
        assert_eq!(v["engine_status"]["all_engines"]["running"], true);
        // An app that never probes it: named, not flown on the wrong type.
        sim.probe_on_swap.store(false, SeqCst);
        let e = set_up_start(&sim, &mut tail, &case(sr22), "", &quick_pace())
            .await
            .unwrap_err();
        assert!(e.to_string().contains("the app never took"), "{e}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// `setup: {enroute: {at, value}}`: a point on the plan, the plan's
    /// origin and runway kept for a swap; a point that is none is refused
    /// naming the case, and an enroute case types its command.
    #[test]
    fn an_enroute_case_parses_and_refuses_what_is_no_point() {
        let spec = |setup: &str, rest: &str| {
            format!("cases: [{{name: e, setup: {{enroute: {setup}}}{rest}}}]")
        };
        let cmd = ", command: fly the flight plan";
        let one = |setup: &str| parse_spec_with(&spec(setup, cmd), &|_| true);
        let c = one("{at: before_tod_nm, value: 30}").unwrap().cases.remove(0);
        assert_eq!(
            c.setup,
            Setup::Enroute {
                field: FIELD_PLAN_ORIGIN.into(),
                runway: RUNWAY_FROM_PLAN.into(),
                point: EnroutePoint::BeforeTod(30.0),
            }
        );
        assert!(c.uses_plan() && !c.wants_wind());
        assert_eq!(c.first_line(), "fly the flight plan");
        let fix = one("{at: fix, value: welin}").unwrap().cases.remove(0);
        let at = |c: &Case| match &c.setup {
            Setup::Enroute { point, .. } => point.clone(),
            other => panic!("{other:?}"),
        };
        assert_eq!(at(&fix), EnroutePoint::Fix("WELIN".into()));
        let dest = one("{at: before_destination_nm, value: 40}").unwrap().cases.remove(0);
        assert_eq!(at(&dest), EnroutePoint::BeforeDestination(40.0));
        // Fast forward is a want the guard decides, as on any case.
        let fast_case = spec("{at: before_tod_nm, value: 30}", &format!("{cmd}, sim_speed: 4"));
        let fast = parse_spec_with(&fast_case, &|_| true).unwrap();
        assert_eq!(fast.cases[0].sim_speed, Some(4));
        for (bad, why) in [
            (
                "{at: top_of_climb, value: 3}",
                "is not one of fix, before_tod_nm, before_destination_nm",
            ),
            ("{at: before_tod_nm, value: -5}", "whole number of miles"),
            ("{at: before_tod_nm, value: 2.5}", "whole number of miles"),
            ("{at: before_tod_nm, value: WELIN}", "needs a number of miles"),
            ("{at: before_destination_nm, value: 0}", "at least 1"),
            ("{at: fix, value: 30}", "names one fix"),
            ("{at: fix, value: \"WEL IN\"}", "names one fix"),
            ("{at: fix}", "missing field"),
        ] {
            let e = one(bad).unwrap_err().to_string();
            assert!(e.contains(why), "{bad}: {e}");
        }
        let e = parse_spec_with(&spec("{at: fix, value: WELIN}", ""), &|_| true).unwrap_err();
        assert!(e.to_string().contains("needs a command"), "{e}");
        let e = parse_spec_with(
            "cases: [{name: e, setup: {enroute: {at: fix, value: WELIN}, ramp: {field: EGKK}},\
             command: x}]",
            &|_| true,
        )
        .unwrap_err();
        assert!(e.to_string().contains("exactly one of"), "{e}");
    }

    /// The point is checked on the fetched plan: a fix it does not hold, an
    /// end of it, a top of descent its navlog lacks, and the placement
    /// itself (the pilot's EGKK-EGBB OFP) are refused before the run.
    #[test]
    fn an_enroute_point_is_checked_on_the_plan() {
        let v: Value =
            serde_json::from_str(include_str!("../bench/ofp/egkk-egbb-738.trimmed.json")).unwrap();
        let fp = crate::simbrief::plan_from_ofp(&v).unwrap();
        let facts = PlanFacts::of(&fp);
        assert!(facts.has_tod && facts.fixes.contains(&"WELIN".to_string()));
        let find = |_: &str| None;
        let case = |point: EnroutePoint| Case {
            name: "e".into(),
            setup: Setup::Enroute {
                field: FIELD_PLAN_ORIGIN.into(),
                runway: RUNWAY_FROM_PLAN.into(),
                point,
            },
            aircraft: None,
            command: Some("fly the flight plan".into()),
            steps: Vec::new(),
            deadline_min: None,
            sim_speed: None,
            settle_s: None,
            expect_give_up: None,
            after: None,
        };
        let ok = resolve_case(&case(EnroutePoint::BeforeTod(30.0)), &facts, &find).unwrap();
        assert_eq!(
            ok.setup,
            Setup::Enroute {
                field: "EGKK".into(),
                runway: "26L".into(),
                point: EnroutePoint::BeforeTod(30.0)
            }
        );
        let e = |p: EnroutePoint, f: &PlanFacts| {
            resolve_case(&case(p), f, &find).unwrap_err().to_string()
        };
        assert!(e(EnroutePoint::Fix("NOSUCH".into()), &facts).contains("not on the flight plan"));
        assert!(e(EnroutePoint::Fix("EGBB".into()), &facts).contains("an end of the flight plan"));
        let no_tod = PlanFacts { has_tod: false, ..facts.clone() };
        assert!(e(EnroutePoint::BeforeTod(30.0), &no_tod).contains("no top of descent"));
        // A plan without an origin runway still sets up on the loaded type.
        let no_rwy = PlanFacts { origin_rwy: None, ..facts.clone() };
        let r = resolve_case(&case(EnroutePoint::Fix("HEMEL".into())), &no_rwy, &find).unwrap();
        assert!(matches!(r.setup, Setup::Enroute { ref runway, .. } if runway.is_empty()));
        // The placement the app will make, with the 737's cruise.
        let speeds = crate::tools::setup_enroute::Speeds {
            profile_kts: Some(255.0),
            ..Default::default()
        };
        let place = |p: EnroutePoint| crate::tools::setup_enroute::place(&fp, &p, &speeds);
        assert!(place(EnroutePoint::BeforeTod(30.0)).is_ok());
        let low = place(EnroutePoint::BeforeDestination(2.0)).unwrap_err();
        assert!(low.contains("under the 3000"), "{low}");
    }

    #[test]
    fn the_enroute_outcome_is_read_from_the_apps_records() {
        let r = |v: Value| enroute_outcome(&v);
        assert_eq!(r(json!({"kind": "setup", "event": "enroute_init"})), None);
        assert_eq!(r(json!({"kind": "tool", "name": "setup_enroute"})), None);
        assert_eq!(
            r(json!({"kind": "setup", "event": "enroute_result", "outcome": "ok",
                     "engaged": true})),
            Some(Ok(()))
        );
        let no_ap = r(json!({"kind": "setup", "event": "enroute_result", "outcome": "ok",
                             "engaged": false}));
        assert!(no_ap.unwrap().unwrap_err().contains("without the autopilot"));
        let refused = r(json!({"kind": "setup", "event": "enroute_result", "outcome": "refused",
                               "engaged": false, "why": "X-Plane refused it, code 1."}));
        assert_eq!(refused, Some(Err("X-Plane refused it, code 1.".into())));
        let said =
            r(json!({"kind": "setup", "event": "enroute_refused", "why": "No flight plan."}));
        assert_eq!(said, Some(Err("the app refused: No flight plan.".into())));
    }

    /// An enroute case: the app's own setup words, the new flight in the
    /// air waited for, then the case's command and its sortie's verdict. A
    /// refusal is the case's failure, in the app's words, nothing typed
    /// after it. The typed words route to the setup tool on the 737.
    #[tokio::test]
    async fn an_enroute_case_is_set_up_in_the_air_then_resumed() {
        use std::sync::atomic::Ordering::SeqCst;
        let (dir, log) = scratch_log("enroute");
        let sim = FakeSim::new(log.clone());
        let mut spec = parse_spec_with(
            "cases: [{name: arrival, setup: {enroute: {at: before_tod_nm, value: 30}},\n\
                      command: fly the flight plan}]",
            &|_| true,
        )
        .unwrap();
        spec.cases[0].setup = Setup::Enroute {
            field: "EGKK".into(),
            runway: "26L".into(),
            point: EnroutePoint::BeforeTod(30.0),
        };
        let mut tail = BlackboxTail::at_end(&log);
        let rows = run_all(&sim, &mut tail, &spec, 1, "", &quick_pace(), &mut |_: &Row| {}).await;
        assert!(rows[0].verdict.pass, "{:?}", rows[0].verdict);
        assert_eq!(
            *sim.typed.lock().unwrap(),
            vec!["put us enroute 30 miles before top of descent", "fly the flight plan"]
        );
        assert!(sim.inits.lock().unwrap().is_empty(), "the app's setup, not a runner post");
        let b738 = bench_profile(None);
        assert_eq!(
            routed("put us enroute 30 miles before top of descent", &b738).as_deref(),
            Some("setup_enroute")
        );
        assert_eq!(routed("fly the flight plan", &b738).as_deref(), Some("full_auto"));
        // Refused: the case fails with the app's words, no command typed.
        sim.typed.lock().unwrap().clear();
        sim.airborne.store(false, SeqCst);
        sim.enroute_refuses.store(true, SeqCst);
        let rows = run_all(&sim, &mut tail, &spec, 1, "", &quick_pace(), &mut |_: &Row| {}).await;
        assert!(!rows[0].verdict.pass);
        let why = &rows[0].verdict.reason;
        assert!(why.contains("the app refused: The plan is for a A320"), "{why}");
        assert_eq!(sim.typed.lock().unwrap().len(), 1);
        // Another aircraft: loaded on the plan's origin runway first.
        sim.enroute_refuses.store(false, SeqCst);
        sim.probe_on_swap.store(true, SeqCst);
        let mut swap = spec.cases[0].clone();
        swap.aircraft = Some("Aircraft/Laminar Research/Cirrus SR22/Cirrus SR22.acf".into());
        set_up_start(&sim, &mut tail, &swap, "", &quick_pace()).await.unwrap();
        let inits = sim.inits.lock().unwrap().clone();
        let v: Value = serde_json::from_str(inits.last().unwrap()).unwrap();
        assert_eq!(v["runway_start"]["airport_id"], "EGKK");
        assert_eq!(v["runway_start"]["runway"], "26L");
        swap.setup = Setup::Enroute {
            field: "EGKK".into(),
            runway: String::new(),
            point: EnroutePoint::BeforeTod(30.0),
        };
        let e = set_up_start(&sim, &mut tail, &swap, "", &quick_pace()).await.unwrap_err();
        assert!(e.to_string().contains("names no runway at EGKK"), "{e}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// LEMG's two runways, verbatim from the Global Airports apt.dat.
    fn lemg_apt(dir: &std::path::Path) -> String {
        let p = dir.join("apt.dat");
        std::fs::write(
            &p,
            "1    52 0 0 LEMG Malaga\n\
             100 45.11 1 1 0.00 1 3 0  12 36.6910753 -4.5078270 0 190 3 1 1 1 30 36.6795648 -4.4805125 0 489 3 0 0 1\n\
             100 45.11 27 27 0.00 1 3 0  13 36.6845573 -4.5126253 0 75 3 1 0 2 31 36.6654027 -4.4858140 0 75 3 1 0 3\n\
             1   1027 0 0 0MU1 Sunderland\n",
        )
        .unwrap();
        p.to_string_lossy().to_string()
    }

    /// #158: the backlog named LEMG 13 with the wind at 280/5; the tower
    /// sent arrivals to 30. `runway: wind` is picked at the setup by the
    /// tower's own rule, from the METAR, else the sim's wind on the field,
    /// and the row says which.
    #[tokio::test]
    async fn runway_wind_is_the_towers_into_wind_end_at_the_setup() {
        let (dir, log) = scratch_log("wind-pick");
        let apt = lemg_apt(&dir);
        let sim = FakeSim::new(log.clone());
        let spec = parse_spec(
            "cases: [{name: final, setup: {approach: {field: LEMG, runway: wind, final_nm: 5}}},\n\
                     {name: start, setup: {runway_start: {field: LEMG, runway: wind, engines_running: true}},\n\
                      command: fly a circuit}]",
        )
        .unwrap();
        assert!(spec.cases.iter().all(Case::wants_wind));
        // The tower's rule on the recorded wind: 30, not 13.
        let runways = crate::nav::find_runways(&apt, "LEMG").unwrap();
        let p = pick_into_wind(&runways, 280.0, 5.0, "METAR").unwrap();
        assert_eq!(p.runway, "30", "{p:?}");
        assert_eq!(p.words(), "runway 30 into 280/5 METAR, headwind 4.8 kt");
        assert_eq!(pick_into_wind(&runways, 130.0, 8.0, "sim").unwrap().runway, "13");
        *sim.metar.lock().unwrap() = Some((280.0, 5.0));
        let mut tail = BlackboxTail::at_end(&log);
        let mut seen = Vec::new();
        let rows = run_all(&sim, &mut tail, &spec, 1, &apt, &quick_pace(), &mut |r: &Row| {
            seen.push(row_line(r))
        })
        .await;
        assert!(rows.iter().all(|r| r.verdict.pass), "{seen:?}");
        assert_eq!(
            sim.typed.lock().unwrap()[0],
            "put us on a 5 mile final to LEMG runway 30"
        );
        let inits = sim.inits.lock().unwrap().clone();
        let v: Value = serde_json::from_str(&inits[0]).unwrap();
        assert_eq!(v["runway_start"]["runway"], "30");
        assert!(
            seen[0].starts_with("PASS final #1 (runway 30 into 280/5 METAR, headwind 4.8 kt): "),
            "{seen:?}"
        );
        // No METAR: the sim's wind at the aircraft on the field (LEMG 13's
        // threshold, 130 at 8 kt) picks 13.
        *sim.metar.lock().unwrap() = None;
        *sim.at.lock().unwrap() = (36.6845, -4.5126, 130.0, 8.0 / MPS_TO_KTS);
        let one = Spec { cases: vec![spec.cases[1].clone()], after: None };
        let rows = run_all(&sim, &mut tail, &one, 1, &apt, &quick_pace(), &mut |_| {}).await;
        assert!(rows[0].verdict.pass, "{:?}", rows[0].verdict);
        assert!(rows[0].verdict.setup.starts_with("runway 13 into 130/8 sim"), "{:?}", rows[0].verdict);
        // Away from the field the aircraft's wind is not the field's: the
        // case fails its setup, naming why, and nothing is flown.
        *sim.at.lock().unwrap() = (51.15, -0.18, 130.0, 4.0);
        let typed = sim.typed.lock().unwrap().len();
        let rows = run_all(&sim, &mut tail, &one, 1, &apt, &quick_pace(), &mut |_| {}).await;
        assert!(!rows[0].verdict.pass);
        assert!(rows[0].verdict.reason.contains("no surface wind for LEMG"), "{:?}", rows[0].verdict);
        assert_eq!(sim.typed.lock().unwrap().len(), typed, "nothing typed");
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// #158: a case with no deadline of its own gets the default scaled by
    /// the airframe's approach speed; one that names its deadline keeps it.
    #[tokio::test]
    async fn the_default_deadline_scales_with_the_approach_speed() {
        assert_eq!(default_deadline_min(None), DEFAULT_DEADLINE_MIN);
        assert_eq!(default_deadline_min(Some(145.0)), DEFAULT_DEADLINE_MIN);
        // A faster airframe never gets less than the default.
        assert_eq!(default_deadline_min(Some(154.0)), DEFAULT_DEADLINE_MIN);
        // The SF50's 87 kt: 145/87 of it.
        assert!((default_deadline_min(Some(87.0)) - 20.0 * 145.0 / 87.0).abs() < 1e-9);
        // A 50 kt ultralight is capped.
        assert_eq!(default_deadline_min(Some(50.0)), DEFAULT_DEADLINE_MIN * DEADLINE_SCALE_MAX);
        let spec = parse_spec(
            "cases: [{name: a, setup: {approach: {field: LEMG, runway: 13}}},\n\
                     {name: b, setup: {approach: {field: LEMG, runway: 13}}, deadline_min: 7}]",
        )
        .unwrap();
        assert_eq!(spec.cases[0].deadline_s_for(Some(87.0)), 20.0 * 145.0 / 87.0 * 60.0);
        assert_eq!(spec.cases[1].deadline_s_for(Some(87.0)), 7.0 * 60.0);
        // The driver sizes the case's deadline from the airframe: an endless
        // mission on an 87 kt aircraft is judged at the scaled deadline, and
        // the row says so.
        let (dir, log) = scratch_log("vapp-deadline");
        let sim = FakeSim::new(log.clone());
        *sim.vapp.lock().unwrap() = Some(87.0);
        let mut tail = BlackboxTail::at_end(&log);
        let one = Spec { cases: vec![spec.cases[0].clone()], after: None };
        let rows = run_all(&sim, &mut tail, &one, 1, "", &quick_pace(), &mut |_| {}).await;
        assert_eq!(rows[0].verdict.setup, "deadline 33 min at 87 kt", "{:?}", rows[0].verdict);
        let _ = std::fs::remove_dir_all(&dir);
    }

    fn quick_pace() -> Pace {
        Pace {
            poll: std::time::Duration::from_millis(5),
            init: InitTiming::LIVE,
            still_s: 0.0,
            still_max_s: 1.0,
            settle_scale: 0.0,
            idle_max_s: 1.0,
            quiet_s: 0.02,
            stop_max_s: 0.5,
            crash_clear_s: 0.2,
            swap_max_s: 1.0,
            fast_resume_s: 0.0,
            enroute_max_s: 1.0,
        }
    }

    fn scratch_log(tag: &str) -> (std::path::PathBuf, std::path::PathBuf) {
        let dir = std::env::temp_dir().join(format!("xvl-runner-{tag}-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let log = dir.join("blackbox.jsonl");
        std::fs::write(&log, "").unwrap();
        (dir, log)
    }

    /// LEMG 2026-09-28: three cases burned in 2 s each on the last case's
    /// wreck. Now the wreck that outlives a setup stops the run, once.
    #[tokio::test]
    async fn a_wreck_that_outlives_the_setup_stops_the_run_once() {
        let (dir, log) = scratch_log("wreck");
        let sim = FakeSim::new(log.clone());
        sim.crashed.store(true, std::sync::atomic::Ordering::SeqCst);
        sim.endless.store(true, std::sync::atomic::Ordering::SeqCst);
        let spec = parse_spec(
            "cases: [{name: five, setup: {approach: {field: LEMG, runway: 13, final_nm: 5}}}]",
        )
        .unwrap();
        let mut tail = BlackboxTail::at_end(&log);
        let rows = run_all(
            &sim,
            &mut tail,
            &spec,
            3,
            "",
            &quick_pace(),
            &mut |_: &Row| {},
        )
        .await;
        assert_eq!(rows.len(), 1, "the run stops at the first case");
        assert_eq!(rows[0].verdict.reason, STILL_CRASHED);
        assert!(rows[0].verdict.stop_run);
        assert_eq!(
            *sim.typed.lock().unwrap(),
            vec!["put us on a 5 mile final to LEMG runway 13"],
            "no \"my controls\" over a wreck that was never this case's"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The deadline typed "my controls" at 1,600 ft on a 5 nm final: the
    /// aircraft was handed to nobody. Now the loops keep it until the next
    /// case's setup replaces the flight, and a run that ends pauses the sim.
    #[tokio::test]
    async fn a_deadline_in_the_air_leaves_the_loops_flying_to_the_next_setup() {
        let (dir, log) = scratch_log("deadline");
        let sim = FakeSim::new(log.clone());
        sim.airborne
            .store(true, std::sync::atomic::Ordering::SeqCst);
        sim.endless.store(true, std::sync::atomic::Ordering::SeqCst);
        // 0.005 min: 0.3 s.
        let spec = parse_spec(
            "cases: [{name: five, setup: {approach: {field: LEMG, runway: 13, final_nm: 5}}, deadline_min: 0.005}]",
        )
        .unwrap();
        let mut tail = BlackboxTail::at_end(&log);
        let rows = run_all(
            &sim,
            &mut tail,
            &spec,
            2,
            "",
            &quick_pace(),
            &mut |_: &Row| {},
        )
        .await;
        assert_eq!(rows.len(), 2);
        for r in &rows {
            assert!(r.verdict.reason.starts_with("deadline"), "{:?}", r.verdict);
            assert!(r.verdict.left_flying && !r.verdict.pass);
        }
        let setup = "put us on a 5 mile final to LEMG runway 13";
        assert_eq!(
            *sim.typed.lock().unwrap(),
            vec![setup, ABORT_LINE, setup],
            "taken back only to be replaced by the next setup, never after the last"
        );
        assert_eq!(
            *sim.commands.lock().unwrap(),
            vec![PAUSE_ON],
            "the run ends with the aircraft in the air: paused, still flown"
        );
        assert!(sim.flying.load(std::sync::atomic::Ordering::SeqCst));

        // On the ground the deadline takes the aircraft back, and nothing is paused.
        let sim = FakeSim::new(log.clone());
        sim.endless.store(true, std::sync::atomic::Ordering::SeqCst);
        let mut tail = BlackboxTail::at_end(&log);
        let rows = run_all(
            &sim,
            &mut tail,
            &spec,
            1,
            "",
            &quick_pace(),
            &mut |_: &Row| {},
        )
        .await;
        assert!(!rows[0].verdict.left_flying);
        assert_eq!(*sim.typed.lock().unwrap(), vec![setup, ABORT_LINE]);
        assert!(sim.commands.lock().unwrap().is_empty());
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// #145, LEMG 31 (sortie 1790584715, 2026-09-28): the verdict came at
    /// the taxi_in stage at 1790585617.8 with the aircraft at 16.1 kt, and
    /// the next runway start was posted 0.3 s later. On the way to the
    /// stand the taxi-in stood still with no loop driving from 618.2 to
    /// 620.7 (vacated, then the taxi request), which is not its end.
    #[test]
    fn the_next_setup_waits_for_the_last_aircraft_at_rest() {
        let pace = Pace {
            quiet_s: QUIET_S,
            stop_max_s: STOP_MAX_S,
            idle_max_s: 60.0,
            ..Pace::LIVE
        };
        let t0 = 1790585617.8;
        let mut w = SettleWatch::new(t0, &pace, false);
        // Rolling at 16 kt, the marker idle: never at rest.
        assert_eq!(
            w.tick(t0, Some(false), Some(16.1), Some(true)),
            SettleStep::Wait
        );
        // The vacate-to-request pause: stopped 2.5 s, then driving again.
        assert_eq!(
            w.tick(618.2 + 1790585000.0, Some(false), Some(0.2), Some(true)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(620.6 + 1790585000.0, Some(false), Some(0.2), Some(true)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(620.9 + 1790585000.0, Some(true), Some(0.5), Some(true)),
            SettleStep::Wait
        );
        // A loop driving at walking pace, or a failed read, is not at rest.
        let t = 1790585700.0;
        assert_eq!(
            w.tick(t, Some(true), Some(0.0), Some(true)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(t + 20.0, None, Some(0.0), Some(true)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(t + 40.0, Some(false), None, Some(true)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(t + 60.0, Some(false), Some(0.0), None),
            SettleStep::Wait
        );
        // At the stand: at rest, held QUIET_S.
        let s = 1790585960.0;
        assert_eq!(
            w.tick(s, Some(false), Some(0.3), Some(true)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(s + QUIET_S - 0.5, Some(false), Some(0.3), Some(true)),
            SettleStep::Wait
        );
        match w.tick(s + QUIET_S, Some(false), Some(0.3), Some(true)) {
            SettleStep::Ready(Settle {
                secs,
                end: SettleEnd::AtRest,
            }) => {
                assert!((secs - (s + QUIET_S - t0)).abs() < 1e-6, "{secs}")
            }
            other => panic!("{other:?}"),
        }

        // Past the bound on the ground: "my controls", then at rest.
        let mut w = SettleWatch::new(0.0, &pace, false);
        assert_eq!(
            w.tick(STOP_MAX_S, Some(false), Some(14.0), Some(true)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(STOP_MAX_S + 0.5, Some(false), Some(14.0), Some(true)),
            SettleStep::TypeMyControls
        );
        assert_eq!(
            w.tick(STOP_MAX_S + 5.0, Some(false), Some(6.0), Some(true)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(STOP_MAX_S + 20.0, Some(false), Some(0.0), Some(true)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(
                STOP_MAX_S + 20.0 + QUIET_S,
                Some(false),
                Some(0.0),
                Some(true)
            ),
            SettleStep::Ready(Settle {
                secs: STOP_MAX_S + 20.0 + QUIET_S,
                end: SettleEnd::MyControls
            })
        );
        // ...and one that never stops fails the case, naming the speed.
        let mut w = SettleWatch::new(0.0, &pace, false);
        w.tick(STOP_MAX_S + 0.5, Some(false), Some(14.0), Some(true));
        match w.tick(STOP_MAX_S + 61.0, Some(false), Some(9.0), Some(true)) {
            SettleStep::Fail(why) => assert!(why.contains("still moving at 9.0 kt"), "{why}"),
            other => panic!("{other:?}"),
        }

        // In the air nothing is waited for: never handed to nobody for
        // longer than the take-back before the setup (#132).
        let mut w = SettleWatch::new(0.0, &pace, false);
        assert_eq!(
            w.tick(0.0, Some(true), Some(140.0), Some(false)),
            SettleStep::TypeMyControls
        );
        assert_eq!(
            w.tick(1.0, Some(true), Some(140.0), Some(false)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(2.0, Some(false), Some(140.0), Some(false)),
            SettleStep::Ready(Settle {
                secs: 2.0,
                end: SettleEnd::TakenBack
            })
        );
        // Loops left flying with the wheels unreadable: taken back at once.
        let mut w = SettleWatch::new(0.0, &pace, true);
        assert_eq!(
            w.tick(0.0, Some(true), None, None),
            SettleStep::TypeMyControls
        );
        // Past the bound with the wheels unreadable: no measurement, as flying.
        let mut w = SettleWatch::new(0.0, &pace, false);
        assert_eq!(
            w.tick(STOP_MAX_S + 1.0, Some(true), Some(3.0), None),
            SettleStep::TypeMyControls
        );
        assert_eq!(
            w.tick(STOP_MAX_S + 2.0, Some(false), None, None),
            SettleStep::Ready(Settle {
                secs: STOP_MAX_S + 2.0,
                end: SettleEnd::TakenBack
            })
        );
    }

    fn circuit_spec() -> Spec {
        parse_spec(
            "cases: [{name: circuit, setup: {runway_start: {field: LEMG, runway: 31, engines_running: true}},\n\
              command: fly a circuit}]",
        )
        .unwrap()
    }

    /// #145 on the fake sim: the circuit's verdict leaves the aircraft
    /// rolling on its taxi-in with the marker idle; the next runway start
    /// goes only once it is at rest, and the row says how long that took.
    #[tokio::test]
    async fn a_rolling_taxi_in_is_waited_out_before_the_next_setup() {
        use std::sync::atomic::Ordering::SeqCst;
        let (dir, log) = scratch_log("taxi-in");
        let sim = FakeSim::new(log.clone());
        sim.roll_after_mission.store(30, SeqCst);
        let mut tail = BlackboxTail::at_end(&log);
        let pace = Pace {
            stop_max_s: 5.0,
            ..quick_pace()
        };
        let rows = run_all(
            &sim,
            &mut tail,
            &circuit_spec(),
            2,
            "",
            &pace,
            &mut |_: &Row| {},
        )
        .await;
        assert_eq!(rows.len(), 2);
        assert!(rows.iter().all(|r| r.verdict.pass), "{:?}", rows[1].verdict);
        assert_eq!(
            sim.setups_while_rolling.load(SeqCst),
            0,
            "no setup under a running taxi-in"
        );
        assert_eq!(sim.inits.lock().unwrap().len(), 2);
        assert_eq!(
            *sim.typed.lock().unwrap(),
            vec!["fly a circuit", "fly a circuit"],
            "a taxi-in that finishes by itself is never interrupted"
        );
        let settle = rows[1].verdict.settle.expect("the wait is on the row");
        assert_eq!(settle.end, SettleEnd::AtRest);
        assert!(settle.secs > 0.0);
        assert_eq!(
            rows[0].verdict.settle.map(|s| s.end),
            Some(SettleEnd::AtRest),
            "the first case had nothing to wait for"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A taxi-in that never ends by itself (the LEMG 31 drive stood down 1 km
    /// off its line and was taken over by a second one): past the bound on
    /// the ground "my controls" stops it (`hand_back_hold` parks), then the
    /// setup, and the row says so.
    #[tokio::test]
    async fn a_taxi_in_past_the_bound_gets_my_controls_then_the_setup() {
        use std::sync::atomic::Ordering::SeqCst;
        let (dir, log) = scratch_log("taxi-in-bound");
        let sim = FakeSim::new(log.clone());
        sim.roll_after_mission.store(u32::MAX, SeqCst);
        let mut tail = BlackboxTail::at_end(&log);
        let pace = Pace {
            stop_max_s: 0.1,
            ..quick_pace()
        };
        let mut seen = Vec::new();
        let rows = run_all(
            &sim,
            &mut tail,
            &circuit_spec(),
            2,
            "",
            &pace,
            &mut |r: &Row| seen.push(row_line(r)),
        )
        .await;
        assert_eq!(rows.len(), 2);
        assert_eq!(sim.setups_while_rolling.load(SeqCst), 0);
        assert_eq!(
            *sim.typed.lock().unwrap(),
            vec!["fly a circuit", ABORT_LINE, "fly a circuit"],
        );
        assert_eq!(
            rows[1].verdict.settle.map(|s| s.end),
            Some(SettleEnd::MyControls)
        );
        assert!(
            seen[1].contains("my controls after 0 s, the last aircraft still moving"),
            "{}",
            seen[1]
        );
        // The run's last taxi-in is left to finish: no setup follows it.
        assert_eq!(sim.rolling_reads.load(SeqCst), u32::MAX);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn after_is_the_cases_else_the_specs_else_park() {
        let s = parse_spec_with(
            "after: stop\n\
             cases: [{name: a, setup: {approach: {field: LEMG, runway: 13, final_nm: 5}}},\n\
                     {name: b, setup: {ramp: {field: LEMG}}, command: fly a circuit,\n\
                      after: park}]",
            &|_| true,
        )
        .unwrap();
        assert_eq!(s.cases[0].after(), After::Stop);
        assert_eq!(s.cases[1].after(), After::Park);
        let s = parse_spec_with(
            "cases: [{name: a, setup: {approach: {field: LEMG, runway: 13, final_nm: 5}}}]",
            &|_| true,
        )
        .unwrap();
        assert_eq!(s.cases[0].after(), After::Park, "silent is today's rule");
        let bad = parse_spec_with(
            "cases: [{name: a, setup: {approach: {field: LEMG, runway: 13, final_nm: 5}},\n\
                      after: taxi}]",
            &|_| true,
        );
        assert!(bad.is_err());
        // The bench specs parse with it, and keep the taxi-in cases parking.
        for (file, parked) in [
            ("lemg-737.yaml", vec![]),
            ("backlog-737.yaml", vec!["737 gate to gate on the SimBrief plan"]),
            (
                "backlog-types.yaml",
                vec![
                    "SR22 LEMG circuit from the ramp",
                    "S-76 LEMG to LXGB on a local plan",
                    "S-76 LEMG runway start into the wind, circuit",
                ],
            ),
        ] {
            let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("bench/sorties")
                .join(file);
            let spec = parse_spec_with(&std::fs::read_to_string(path).unwrap(), &|_| true)
                .unwrap();
            for c in &spec.cases {
                let want = match parked.contains(&c.name.as_str()) {
                    true => After::Park,
                    false => After::Stop,
                };
                assert_eq!(c.after(), want, "{file}: {}", c.name);
            }
        }
    }

    /// `after: stop`: the last verdict came with the aircraft on its taxi-in
    /// (LEMG, 2026-09-28: the SF50 circuit's vacate and taxi-in held the next
    /// setup 306 s). "my controls" at once, again if a loop still drives past
    /// [`STOP_RETYPE_S`], then the quiet hold; in the air, today's take-back.
    #[test]
    fn a_stop_after_the_verdict_takes_the_aircraft_back_on_the_ground() {
        let pace = Pace {
            quiet_s: QUIET_S,
            stop_max_s: STOP_MAX_S,
            idle_max_s: 60.0,
            ..Pace::LIVE
        };
        let mut w = SettleWatch::new(0.0, &pace, false).stop_now(true);
        assert_eq!(
            w.tick(0.0, Some(true), Some(14.0), Some(true)),
            SettleStep::TypeMyControls
        );
        // Still driving past the retype: once more, and only once.
        assert_eq!(
            w.tick(STOP_RETYPE_S + 0.5, Some(true), Some(12.0), Some(true)),
            SettleStep::TypeMyControls
        );
        assert_eq!(
            w.tick(STOP_RETYPE_S + 1.0, Some(true), Some(10.0), Some(true)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(15.0, Some(false), Some(0.0), Some(true)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(15.0 + QUIET_S, Some(false), Some(0.0), Some(true)),
            SettleStep::Ready(Settle {
                secs: 15.0 + QUIET_S,
                end: SettleEnd::Stopped
            })
        );
        // Already at rest: nothing typed, the quiet hold as always.
        let mut w = SettleWatch::new(0.0, &pace, false).stop_now(true);
        assert_eq!(
            w.tick(0.0, Some(false), Some(0.0), Some(true)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(QUIET_S, Some(false), Some(0.0), Some(true)),
            SettleStep::Ready(Settle {
                secs: QUIET_S,
                end: SettleEnd::AtRest
            })
        );
        // In the air: taken back right before the setup, as without it.
        let mut w = SettleWatch::new(0.0, &pace, false).stop_now(true);
        assert_eq!(
            w.tick(0.0, Some(true), Some(140.0), Some(false)),
            SettleStep::TypeMyControls
        );
        assert_eq!(
            w.tick(1.0, Some(false), Some(140.0), Some(false)),
            SettleStep::Ready(Settle {
                secs: 1.0,
                end: SettleEnd::TakenBack
            })
        );
        // Unreadable wheels are no measurement: never stopped on a guess.
        let mut w = SettleWatch::new(0.0, &pace, false).stop_now(true);
        assert_eq!(w.tick(0.0, Some(true), Some(14.0), None), SettleStep::Wait);
    }

    /// #168, run 20260928-203723Z case 2 (737 LEMG 13): a rejected takeoff
    /// rolled on unbraked at 73 kt, "my controls" at 1790627961.4, the park
    /// brake set at 2007.9, under a knot by 2012.7. The 60 s bound came at
    /// 2021.4 with the aircraft 9 s into its 10 s quiet hold, and the case
    /// failed "still moving at 0 kt". At rest at the bound is settling.
    #[test]
    fn a_stopped_aircraft_at_the_bound_settles() {
        let pace = Pace {
            quiet_s: QUIET_S,
            stop_max_s: STOP_MAX_S,
            idle_max_s: 60.0,
            ..Pace::LIVE
        };
        // The state records' groundspeed (kt), the brake at 2007.9.
        let gs = [
            (1961.4, 72.9),
            (1979.5, 60.3),
            (2006.4, 50.6),
            (2007.9, 50.1),
            (2009.5, 36.1),
            (2011.1, 17.7),
            (2012.7, 0.1),
            (2014.3, 0.0),
            (2040.0, 0.0),
        ];
        let at = |t: f64| {
            let i = gs.windows(2).position(|w| t < w[1].0).unwrap_or(gs.len() - 2);
            let ((t0, g0), (t1, g1)) = (gs[i], gs[i + 1]);
            g0 + (g1 - g0) * ((t - t0) / (t1 - t0)).clamp(0.0, 1.0)
        };
        let t0 = 1790627961.2;
        let mut w = SettleWatch::new(t0, &pace, false).stop_now(true);
        assert_eq!(
            w.tick(t0, Some(false), Some(73.3), Some(true)),
            SettleStep::TypeMyControls
        );
        let mut t = 1961.7;
        let end = loop {
            let step = w.tick(t0 - 1961.2 + t, Some(false), Some(at(t)), Some(true));
            if step != SettleStep::Wait {
                break (t, step);
            }
            t += 0.5;
            assert!(t < 2040.0, "never settled");
        };
        assert!(end.0 > 2021.4 && end.0 < 2024.0, "settled at {}", end.0);
        assert!(
            matches!(end.1, SettleStep::Ready(Settle { end: SettleEnd::Stopped, .. })),
            "{:?}",
            end.1
        );
        // Still rolling at the bound: the reason names the speed.
        let mut w = SettleWatch::new(0.0, &pace, false).stop_now(true);
        w.tick(0.0, Some(false), Some(73.0), Some(true));
        match w.tick(61.0, Some(false), Some(4.2), Some(true)) {
            SettleStep::Fail(why) => assert!(why.contains("still moving at 4.2 kt"), "{why}"),
            s => panic!("{s:?}"),
        }
        // Not a groundspeed at all: the reason says which read failed.
        let mut w = SettleWatch::new(0.0, &pace, false).stop_now(true);
        w.tick(0.0, Some(false), Some(73.0), Some(true));
        match w.tick(61.0, Some(false), Some(0.0), None) {
            SettleStep::Fail(why) => assert!(why.contains("wheels unreadable"), "{why}"),
            s => panic!("{s:?}"),
        }
    }

    /// The fake sim's taxi-in never ends by itself; with `after: stop` the
    /// runner never waits for the bound: "my controls" after each verdict
    /// (the last one's at the run's end), and the rows say so.
    #[tokio::test]
    async fn after_stop_types_my_controls_at_the_verdict() {
        use std::sync::atomic::Ordering::SeqCst;
        let (dir, log) = scratch_log("after-stop");
        let sim = FakeSim::new(log.clone());
        sim.roll_after_mission.store(u32::MAX, SeqCst);
        let mut tail = BlackboxTail::at_end(&log);
        let pace = Pace {
            stop_max_s: 30.0,
            ..quick_pace()
        };
        let mut spec = circuit_spec();
        spec.cases[0].after = Some(After::Stop);
        let mut seen = Vec::new();
        let t0 = std::time::Instant::now();
        let rows = run_all(
            &sim,
            &mut tail,
            &spec,
            2,
            "",
            &pace,
            &mut |r: &Row| seen.push(row_line(r)),
        )
        .await;
        assert!(t0.elapsed().as_secs_f64() < 20.0, "never waited out the bound");
        assert_eq!(rows.len(), 2);
        assert!(rows.iter().all(|r| r.verdict.pass && r.verdict.stop_owed));
        assert_eq!(sim.setups_while_rolling.load(SeqCst), 0);
        assert_eq!(
            *sim.typed.lock().unwrap(),
            vec!["fly a circuit", ABORT_LINE, "fly a circuit", ABORT_LINE],
        );
        assert_eq!(
            rows[1].verdict.settle.map(|s| s.end),
            Some(SettleEnd::Stopped)
        );
        assert!(seen[1].contains("my controls at the last verdict"), "{}", seen[1]);
        assert_eq!(sim.rolling_reads.load(SeqCst), 0, "the last one stopped too");
        let _ = std::fs::remove_dir_all(&dir);
    }

    // ---- Fast forward ------------------------------------------------------------

    #[test]
    fn a_speed_step_parses_and_a_bad_one_is_refused() {
        let s = parse_spec_with(
            "cases: [{name: a, setup: {ramp: {field: LEMG}}, sim_speed: 2, steps: [\n\
               {say: fly the flight plan, when: start, tool: full_auto},\n\
               {sim_speed: 4, when: {phase: cruise}},\n\
               {sim_speed: 1, when: {event: {kind: procedure, event: tod}}}]}]",
            &|_| true,
        )
        .unwrap();
        let c = &s.cases[0];
        assert_eq!(c.sim_speed, Some(2));
        let later = c.later_steps();
        assert_eq!(later.len(), 2);
        assert_eq!(later[0].sim_speed, Some(4));
        assert_eq!(later[0].when, Trigger::Phase("Cruise".into()));
        assert!(!later[0].is_say() && later[0].say.is_empty());
        assert_eq!(later[1].sim_speed, Some(1));
        let mut script = Script::new(later);
        assert_eq!((script.pending(), script.pending_says()), (2, 0));
        script.arm(0.0);
        script.sent(1.0);
        assert_eq!(script.remaining().len(), 1);
        let bad = [
            ("sim_speed: 0", "sim_speed is 1 to 16"),
            ("sim_speed: 17", "sim_speed is 1 to 16"),
            (
                "steps: [{sim_speed: 0, when: {after_s: 1}}]",
                "sim_speed is 1 to 16",
            ),
            (
                "steps: [{sim_speed: 32, when: {after_s: 1}}]",
                "sim_speed is 1 to 16",
            ),
            (
                "steps: [{sim_speed: -4, when: {after_s: 1}}]",
                "sim_speed is 1 to 16",
            ),
            (
                "steps: [{say: x, sim_speed: 4, when: {after_s: 1}}]",
                "not both",
            ),
            (
                "steps: [{sim_speed: 4, when: {after_s: 1}, tool: full_auto}]",
                "routes to no tool",
            ),
            ("steps: [{sim_speed: 4, when: start}]", "never the start"),
            ("steps: [{sim_speed: 4}]", "needs a when"),
            ("steps: [{when: {after_s: 1}}]", "needs a say"),
        ];
        for (tail, want) in bad {
            let text = format!(
                "cases: [{{name: a, setup: {{runway_start: {{field: LEMG, runway: 13}}}}, command: fly a circuit, {tail}}}]"
            );
            let e = parse_spec_with(&text, &|_| true)
                .expect_err(tail)
                .to_string();
            assert!(e.contains(want), "{tail}: {e}");
        }
    }

    #[test]
    fn a_slow_loop_on_the_panel_is_named_the_follower_is_not() {
        assert_eq!(
            slow_loop("0MONITORING|You have control; check autopilot"),
            None
        );
        assert_eq!(
            slow_loop("1FULL AUTO|cruise\nflying a procedure: to PIMOS\ncontrol: copilot flying"),
            None
        );
        for (s, want) in [
            (
                "1AUTOLAND LEMG 13 12.4nm FINAL",
                "AUTOLAND LEMG 13 12.4nm FINAL",
            ),
            ("1FULL AUTO|approach\nautoland LEMG 13", "autoland LEMG 13"),
            (
                "1AUTO CIRCUIT|downwind runway 13",
                "AUTO CIRCUIT|downwind runway 13",
            ),
            (
                "1FULL AUTO|taxi\nauto taxi: 340 m to go",
                "auto taxi: 340 m to go",
            ),
            ("1RECOVERING: STALL|", "RECOVERING: STALL|"),
            ("1ROUTE|cruise", "ROUTE|cruise"),
            ("1ROUTE|cruise\nflying 40 kt, 500 ft", "ROUTE|cruise"),
            ("1FLYING 40 KT, 500 FT|", "FLYING 40 KT, 500 FT|"),
            ("1PUSHBACK|", "PUSHBACK|"),
            ("1TALK-DOWN LSZS 21|", "TALK-DOWN LSZS 21|"),
        ] {
            assert_eq!(slow_loop(s).as_deref(), Some(want), "{s}");
        }
    }

    /// Every reading of the follower's cruise that lets the sim run fast.
    fn cruising() -> FastReads {
        FastReads {
            crashed: Some(false),
            on_ground: Some(false),
            agl_ft: Some(20_000.0),
            servos_on: Some(true),
            status: Some("1FULL AUTO|cruise\nflying a procedure: to PIMOS".into()),
        }
    }

    /// A guard flying 4x in cruise, its sortie begun at t 0 and fast from t 20.
    fn fast_guard() -> SpeedGuard {
        let mut g = SpeedGuard::new(Some(4), FAST_RESUME_S);
        g.feed(&json!({"kind": "sortie", "event": "begin", "sortie": "9-full_auto"}));
        g.feed(&json!({"kind": "state", "fphase": "Cruise", "sortie": "9-full_auto"}));
        assert_eq!(g.tick(0.0, &cruising()), SpeedAct::Keep, "held first");
        assert_eq!(g.tick(9.0, &cruising()), SpeedAct::Keep);
        assert!(matches!(
            g.tick(10.0, &cruising()),
            SpeedAct::Write { to: 4, .. }
        ));
        g.written(4, Some(4.0));
        assert_eq!(g.tick(11.0, &cruising()), SpeedAct::Keep);
        g
    }

    /// The runner's guard: 1x the moment any condition fails, the reason
    /// said; fast again only after they have all held; the latches last the
    /// sortie.
    #[test]
    fn the_guard_drops_to_1x_on_each_condition() {
        let one = |g: &mut SpeedGuard, x: FastReads, want: &str| match g.tick(30.0, &x) {
            SpeedAct::Write { to: 1, reason } => {
                assert!(reason.contains(want), "{reason} vs {want}");
                g.written(1, Some(1.0));
            }
            other => panic!("{want}: {other:?}"),
        };
        let cases: Vec<(FastReads, &str)> = vec![
            (
                FastReads {
                    crashed: Some(true),
                    ..cruising()
                },
                "crashed",
            ),
            (
                FastReads {
                    crashed: None,
                    ..cruising()
                },
                "wreck flag unreadable",
            ),
            (
                FastReads {
                    on_ground: Some(true),
                    ..cruising()
                },
                "on the ground",
            ),
            (
                FastReads {
                    on_ground: None,
                    ..cruising()
                },
                "wheels unreadable",
            ),
            (
                FastReads {
                    agl_ft: Some(4_900.0),
                    ..cruising()
                },
                "4900 ft AGL",
            ),
            (
                FastReads {
                    agl_ft: None,
                    ..cruising()
                },
                "height unreadable",
            ),
            (
                FastReads {
                    servos_on: Some(false),
                    ..cruising()
                },
                "servos off",
            ),
            (
                FastReads {
                    servos_on: None,
                    ..cruising()
                },
                "servos unreadable",
            ),
            (
                FastReads {
                    status: Some("1FULL AUTO|approach\nautoland LEMG 13".into()),
                    ..cruising()
                },
                "autoland LEMG 13 flying",
            ),
            (
                FastReads {
                    status: None,
                    ..cruising()
                },
                "status unreadable",
            ),
        ];
        for (x, want) in cases {
            let mut g = fast_guard();
            one(&mut g, x.clone(), want);
            // Held (said once), then back only after the conditions held.
            assert_eq!(g.tick(31.0, &x), SpeedAct::Keep);
            assert_eq!(g.tick(40.0, &cruising()), SpeedAct::Keep);
            assert!(matches!(
                g.tick(50.0, &cruising()),
                SpeedAct::Write { to: 4, .. }
            ));
        }
        // The phase: an approach or the ground.
        for p in ["Approach", "InitialClimb", "Landing", "GoAround"] {
            let mut g = fast_guard();
            g.feed(&json!({"kind": "flight", "ev": "phase", "from": "Descent", "to": p}));
            one(&mut g, cruising(), &format!("phase {p}"));
        }
        // Latched for the sortie: a give-up, a hand, a pilot's command, an
        // impact; a new sortie clears it.
        for (r, want) in [
            (
                json!({"kind": "full_auto", "event": "give_up", "reason": "ap_not_engaged"}),
                "give-up (ap_not_engaged)",
            ),
            (
                json!({"kind": "intervention", "what": "yoke_under_ap"}),
                "intervention (yoke_under_ap)",
            ),
            (
                json!({"kind": "tool", "name": "set_heading"}),
                "pilot's command (set_heading)",
            ),
            (json!({"kind": "flight", "ev": "impact"}), "impact"),
        ] {
            let mut g = fast_guard();
            g.feed(&r);
            one(&mut g, cruising(), want);
            assert_eq!(
                g.tick(100.0, &cruising()),
                SpeedAct::Keep,
                "{want}: latched"
            );
            assert_eq!(
                g.tick(200.0, &cruising()),
                SpeedAct::Keep,
                "{want}: latched"
            );
            g.feed(&json!({"kind": "sortie", "event": "begin", "sortie": "10-full_auto"}));
            assert_eq!(g.tick(201.0, &cruising()), SpeedAct::Keep);
            assert!(matches!(
                g.tick(211.0, &cruising()),
                SpeedAct::Write { to: 4, .. }
            ));
        }
        // The sortie's end, and the spec's own 1x.
        let mut g = fast_guard();
        g.feed(&json!({"kind": "sortie_result", "sortie": "9-full_auto", "pass": true}));
        one(&mut g, cruising(), "no sortie flying");
        let mut g = fast_guard();
        g.want(1);
        one(&mut g, cruising(), "the spec wants 1x");
        assert!(!g.needs_reads(), "nothing wanted, nothing owed: no reads");
        // Withheld before it ever ran: said once per reason, never written.
        let mut g = SpeedGuard::new(Some(4), FAST_RESUME_S);
        let ground = FastReads {
            on_ground: Some(true),
            ..cruising()
        };
        assert_eq!(
            g.tick(0.0, &ground),
            SpeedAct::Held {
                reason: "no sortie flying".into()
            }
        );
        assert_eq!(g.tick(1.0, &ground), SpeedAct::Keep);
        // A fast write that does not read back is latched off and restored.
        let mut g = SpeedGuard::new(Some(4), 0.0);
        g.feed(&json!({"kind": "sortie", "event": "begin", "sortie": "s"}));
        g.feed(&json!({"kind": "state", "fphase": "Climb"}));
        assert!(matches!(
            g.tick(0.0, &cruising()),
            SpeedAct::Write { to: 4, .. }
        ));
        g.written(4, Some(1.0));
        assert!(!g.owes_restore());
        assert!(matches!(
            g.tick(1.0, &cruising()),
            SpeedAct::Held { reason } if reason.contains("sim_speed 4 read back 1")
        ));
        let mut g = SpeedGuard::new(Some(4), 0.0);
        g.feed(&json!({"kind": "sortie", "event": "begin", "sortie": "s"}));
        g.feed(&json!({"kind": "state", "fphase": "Descent"}));
        assert!(matches!(
            g.tick(0.0, &cruising()),
            SpeedAct::Write { to: 4, .. }
        ));
        g.written(4, None);
        assert!(g.owes_restore(), "an unread multiplier is restored");
        assert!(matches!(
            g.tick(1.0, &cruising()),
            SpeedAct::Write { to: 1, .. }
        ));
        // The clock: wall seconds fast, and what the sim achieved.
        let mut g = fast_guard();
        g.accrue(11.0, Some(3.8));
        g.accrue(21.0, Some(3.8));
        g.accrue(31.0, None);
        assert!((g.fast_s - 20.0).abs() < 1e-9);
        assert!((g.achieved().unwrap() - 3.8).abs() < 1e-9);
    }

    /// Against a fake sim: a speed step at the cruise runs the sim fast
    /// through the guard, and every way out puts it back to 1x: a sim left
    /// fast before the setup, the case's end, a crash.
    #[tokio::test]
    async fn the_cruise_flies_fast_and_every_end_comes_back_to_1x() {
        use std::sync::atomic::Ordering::SeqCst;
        let (dir, log) = scratch_log("fast");
        let sim = FakeSim::new(log.clone());
        sim.airborne.store(true, SeqCst);
        sim.endless.store(true, SeqCst);
        sim.enroute.store(true, SeqCst);
        // A sim left fast by something else: 1x before the setup.
        *sim.speed.lock().unwrap() = 8.0;
        // 0.01 min: 0.6 s, then the deadline.
        let spec = parse_spec(
            "cases: [{name: cruise, setup: {approach: {field: LEMG, runway: 13, final_nm: 5}},\n\
                      deadline_min: 0.01, steps: [{sim_speed: 4, when: {phase: Cruise}}]}]",
        )
        .unwrap();
        let mut tail = BlackboxTail::at_end(&log);
        let rows = run_all(
            &sim,
            &mut tail,
            &spec,
            1,
            "",
            &quick_pace(),
            &mut |_: &Row| {},
        )
        .await;
        assert_eq!(
            *sim.speeds.lock().unwrap(),
            vec![1.0, 4.0, 1.0],
            "1x before the setup, 4x in the cruise, 1x at the case's end"
        );
        let v = &rows[0].verdict;
        assert!(v.reason.starts_with("deadline"), "{v:?}");
        assert!(v.fast_s > 0.0, "{v:?}");
        assert_eq!(v.fast_x, Some(4.0));
        assert!(
            row_line(&rows[0]).contains("(fast "),
            "{}",
            row_line(&rows[0])
        );

        // A crash while fast: 1x at once, and the case ends crashed.
        let sim = FakeSim::new(log.clone());
        sim.airborne.store(true, SeqCst);
        sim.endless.store(true, SeqCst);
        sim.enroute.store(true, SeqCst);
        sim.crash_when_fast.store(true, SeqCst);
        let spec = parse_spec(
            "cases: [{name: cruise, setup: {approach: {field: LEMG, runway: 13, final_nm: 5}},\n\
                      deadline_min: 0.05, sim_speed: 4}]",
        )
        .unwrap();
        let mut tail = BlackboxTail::at_end(&log);
        let rows = run_all(
            &sim,
            &mut tail,
            &spec,
            1,
            "",
            &quick_pace(),
            &mut |_: &Row| {},
        )
        .await;
        assert!(rows[0].verdict.crashed, "{:?}", rows[0].verdict);
        assert_eq!(*sim.speeds.lock().unwrap(), vec![4.0, 1.0]);
        assert_eq!(*sim.speed.lock().unwrap(), 1.0);

        // Below the floor (or on any failed condition) the case's speed is
        // never written at all.
        let sim = FakeSim::new(log.clone());
        sim.airborne.store(true, SeqCst);
        sim.endless.store(true, SeqCst);
        let spec = parse_spec(
            "cases: [{name: cruise, setup: {approach: {field: LEMG, runway: 13, final_nm: 5}},\n\
                      deadline_min: 0.005, sim_speed: 4}]",
        )
        .unwrap();
        let mut tail = BlackboxTail::at_end(&log);
        let rows = run_all(
            &sim,
            &mut tail,
            &spec,
            1,
            "",
            &quick_pace(),
            &mut |_: &Row| {},
        )
        .await;
        assert_eq!(rows[0].verdict.fast_s, 0.0);
        assert!(sim.speeds.lock().unwrap().is_empty());
        let _ = std::fs::remove_dir_all(&dir);
    }
}
