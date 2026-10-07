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
const ACF_PATH: &str = crate::datarefs::ACF_RELATIVE_PATH;
const ACF_ICAO: &str = crate::datarefs::ACF_ICAO;
const GROUNDSPEED: &str = crate::datarefs::GROUNDSPEED_MS;
const PAUSE_ON: &str = "sim/operation/pause_on";
const PAUSE_TOGGLE: &str = "sim/operation/pause_toggle";
const PAUSED: &str = "sim/time/paused";

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
    /// `traffic: off` hides LiveTraffic's aircraft for the case (a tower
    /// holding or ordering a go-around for them cost about 25 min on
    /// 2026-09-28 in cases that did not test ATC); any other case shows
    /// them, and the run's end puts them back.
    pub traffic: Option<bool>,
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
    #[serde(default, deserialize_with = "on_off")]
    traffic: Option<bool>,
}

/// `on`/`off` (or a YAML bool) for a switch.
fn on_off<'de, D: serde::Deserializer<'de>>(d: D) -> std::result::Result<Option<bool>, D::Error> {
    #[derive(Deserialize)]
    #[serde(untagged)]
    enum V {
        B(bool),
        S(String),
    }
    match Option::<V>::deserialize(d)? {
        None => Ok(None),
        Some(V::B(b)) => Ok(Some(b)),
        Some(V::S(s)) => match s.trim().to_ascii_lowercase().as_str() {
            "on" => Ok(Some(true)),
            "off" => Ok(Some(false)),
            o => Err(serde::de::Error::custom(format!("traffic is on or off, not {o}"))),
        },
    }
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
            traffic: c.traffic,
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
        if let Some(g) = give_up_of(r) {
            self.latched = Some(format!("a give-up ({g})"));
            return;
        }
        self.latched = match (kind, event) {
            ("intervention", _) => Some(format!("an intervention ({})", st(r, "what"))),
            ("tool", _) => Some(format!("a pilot's command ({})", st(r, "name"))),
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
/// LiveTraffic's own display switch; an unread one (no LiveTraffic) is left
/// alone. Blackboxed `sortie_runner` `traffic`.
const TRAFFIC_SHOWN: &str = "livetraffic/cfg/aircrafts_displayed";

async fn set_traffic(sim: &dyn RunnerSim, on: bool) {
    let Ok(now) = sim.read(TRAFFIC_SHOWN).await else {
        return;
    };
    if (now > 0.5) == on {
        return;
    }
    let wrote = sim.write_float(TRAFFIC_SHOWN, if on { 1.0 } else { 0.0 }).await;
    let read = sim.read(TRAFFIC_SHOWN).await.ok();
    crate::blackbox::record(
        "sortie_runner",
        serde_json::json!({"event": "traffic", "on": on, "read_back": read,
                           "error": wrote.err().map(|e| e.to_string())}),
    );
}

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

    #[cfg(test)]
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

/// Full auto's give-up as it writes it (`full_auto::event`): a `full_auto`
/// stage `give_up`, the reason under `detail`. `None`: not one. The
/// helicopter's route writes none (its end is `heli_route` `ended`); the
/// sortie's `end` carries the reason for both.
fn give_up_of(r: &Value) -> Option<&str> {
    (st(r, "kind") == "full_auto" && st(r, "stage") == "give_up").then(|| {
        r.pointer("/detail/reason")
            .and_then(Value::as_str)
            .unwrap_or("")
    })
}

/// A blackbox record the runner reads, as the app writes it: its `kind`,
/// the field that picks it out (`key` = `value`; empty: any record of the
/// kind), the fields read from it, and where the app writes it (a file and
/// a snippet of the writing code, compared without whitespace). A test pins
/// every entry to the source, and every field the runner reads to this
/// table: the runner read full auto's give-up in a shape it never wrote
/// (d270a29, LEMG 13 2026-09-28), and a heli_route give_up nobody writes.
pub struct ReadShape {
    pub kind: &'static str,
    pub key: &'static str,
    pub value: &'static str,
    pub fields: &'static [&'static str],
    pub producer: &'static [(&'static str, &'static str)],
}

/// Fields the blackbox writer adds to every record (`sim::blackbox::write`):
/// `t`, `kind`, and `sortie` while one runs.
pub const WRITER_FIELDS: &[&str] = &["t", "kind", "sortie"];

/// Every record shape the runner's machines match on.
pub const READ_SHAPES: &[ReadShape] = &[
    ReadShape {
        kind: "said",
        key: "who",
        value: "crew",
        fields: &["who", "text"],
        producer: &[("src/voice.rs", r#"crate::blackbox::record("said","#)],
    },
    ReadShape {
        kind: "typed",
        key: "event",
        value: "taken",
        fields: &["text"],
        producer: &[(
            "src/pollers.rs",
            r#""typed", serde_json::json!({ "event": "taken", "text""#,
        )],
    },
    ReadShape {
        kind: "typed",
        key: "event",
        value: "consumed",
        fields: &["text"],
        producer: &[(
            "src/main.rs",
            r#""typed", serde_json::json!({ "event": "consumed", "text""#,
        )],
    },
    ReadShape {
        kind: "tool",
        key: "",
        value: "",
        fields: &["name"],
        producer: &[("src/main/dispatch.rs", r#""tool", serde_json::json!({ "name": call.name"#)],
    },
    ReadShape {
        kind: "sortie",
        key: "event",
        value: "begin",
        fields: &[],
        producer: &[(
            "crates/sim/src/blackbox.rs",
            r#"o.insert("event".into(), "begin".into());"#,
        )],
    },
    ReadShape {
        kind: "sortie",
        key: "event",
        value: "end",
        fields: &["outcome"],
        producer: &[("crates/sim/src/blackbox.rs", r#"o.insert("event".into(), "end".into());"#)],
    },
    ReadShape {
        kind: "sortie",
        key: "outcome",
        value: "gave_up",
        fields: &["reason"],
        producer: &[("src/full_auto.rs", r#"end_sortie("gave_up", json!({ "reason": reason"#)],
    },
    ReadShape {
        kind: "full_auto",
        key: "stage",
        value: "give_up",
        fields: &["detail", "reason"],
        producer: &[
            (
                "src/full_auto.rs",
                r#"record("full_auto", json!({ "stage": stage, "detail": detail }))"#,
            ),
            ("src/full_auto.rs", r#"event("give_up", json!({ "reason": reason }));"#),
        ],
    },
    ReadShape {
        kind: "stage_result",
        key: "",
        value: "",
        fields: &["stage", "pass", "why"],
        producer: &[("src/scorecard.rs", r#"out.push(("stage_result".into(), metrics));"#)],
    },
    ReadShape {
        kind: "stage_result",
        key: "stage",
        value: "landing",
        fields: &["sink_fpm", "drift_deg", "rollout_max_xtk_m"],
        producer: &[("src/scorecard.rs", r#"self.push("landing","#)],
    },
    ReadShape {
        kind: "stage_result",
        key: "stage",
        value: "takeoff",
        fields: &["max_xtk_m"],
        producer: &[("src/scorecard.rs", r#"self.push("takeoff","#)],
    },
    ReadShape {
        kind: "stage_result",
        key: "stage",
        value: "alignment",
        fields: &["err_deg", "xtk_m"],
        producer: &[("src/scorecard.rs", r#"self.push("alignment","#)],
    },
    ReadShape {
        kind: "stage_result",
        key: "stage",
        value: "taxi",
        fields: &["max_lat_m", "max_cut_m"],
        producer: &[(
            "src/scorecard.rs",
            r#"let stage = if x.line_up { "line_up" } else { "taxi" };"#,
        )],
    },
    ReadShape {
        kind: "sortie_result",
        key: "",
        value: "",
        fields: &[
            "outcome",
            "interventions",
            "commands",
            "unattended",
            "crashed",
            "ended_airborne",
            "pass",
        ],
        producer: &[("src/scorecard.rs", r#"out.push(("sortie_result".into(), v));"#)],
    },
    ReadShape {
        kind: "setup",
        key: "event",
        value: "approach_result",
        fields: &["outcome", "result", "armed"],
        producer: &[("src/tools/setup.rs", r#"json!({ "event": "approach_result", "outcome""#)],
    },
    ReadShape {
        kind: "setup",
        key: "event",
        value: "enroute_refused",
        fields: &["why"],
        producer: &[(
            "src/tools/setup_enroute.rs",
            r#"json!({ "event": "enroute_refused", "why""#,
        )],
    },
    ReadShape {
        kind: "setup",
        key: "event",
        value: "enroute_result",
        fields: &["outcome", "engaged", "why"],
        producer: &[(
            "src/tools/setup_enroute.rs",
            r#"json!({ "event": "enroute_result", "outcome""#,
        )],
    },
    ReadShape {
        kind: "state",
        key: "",
        value: "",
        fields: &["fphase"],
        producer: &[("src/monitor.rs", r#""fphase": format!("{:?}", flight.phase())"#)],
    },
    ReadShape {
        kind: "flight",
        key: "ev",
        value: "phase",
        fields: &["to"],
        producer: &[
            ("src/monitor.rs", r#"crate::blackbox::record("flight", ev.to_json());"#),
            ("src/flight.rs", r#"json!({ "ev": "phase", "from": ph(from), "to": ph(to)"#),
        ],
    },
    ReadShape {
        kind: "flight",
        key: "ev",
        value: "impact",
        fields: &[],
        producer: &[("src/flight.rs", r#""ev": "impact","#)],
    },
    ReadShape {
        kind: "flight",
        key: "ev",
        value: "abnormal",
        fields: &["to"],
        producer: &[
            ("src/flight.rs", r#"json!({ "ev": "abnormal", "from": abn(from), "to": abn(to) })"#),
            ("src/flight.rs", "    Crashed,"),
        ],
    },
    ReadShape {
        kind: "intervention",
        key: "",
        value: "",
        fields: &["what"],
        producer: &[(
            "src/flight.rs",
            r#"record("intervention", serde_json::json!({ "what": what }))"#,
        )],
    },
    ReadShape {
        kind: "profile_probe",
        key: "",
        value: "",
        fields: &["identity", "relative_path"],
        producer: &[
            ("src/detect.rs", r#""missing": missing, "identity": identity }"#),
            ("src/detect.rs", r#""relative_path": self.who.relative_path,"#),
        ],
    },
];

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
                    _ if mine && give_up_of(r).is_some() => {
                        self.give_up = give_up_of(r).map(str::to_string);
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
/// minute), the settle and the station (a minute and fifteen seconds), then
/// the level, speed and altimeter settling (a minute, #177).
pub const ENROUTE_MAX_S: f64 = 360.0;

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
pub const STOPPED_KTS: f64 = sim::ground::STOPPED_KTS;
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
        let still = if sim::ground::stopped_on_ground(g.ok(), gs.ok()) {
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
const IAS_KTS: &str = crate::datarefs::INDICATED_AIRSPEED_KTS;
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
    // Never freeze a crisis: low and going down, the loops fly on until it
    // climbs or has height (a 737 paused mid-recovery at 200 ft, 7 degrees
    // nose down, was unpaused into the ground, LEMG 13 2026-09-29; #181).
    let t0 = now_s();
    while now_s() - t0 < PAUSE_WAIT_LOW_S {
        let (agl, vs) = tokio::join!(
            sim.read(crate::datarefs::AGL_METERS),
            sim.read(crate::datarefs::VS_FPM_FLIGHT_MODEL)
        );
        let (Ok(agl), Ok(vs)) = (agl, vs) else { break };
        if pause_safe(agl * 3.28084, vs) {
            break;
        }
        tokio::time::sleep(std::time::Duration::from_secs(1)).await;
    }
    // Paused is what the sim SAYS, not what the command returned: an
    // interrupted A330 climbing at 190 kt was reported "X-Plane paused" with
    // `sim/time/paused` reading 0 (LEMG 30, 2026-09-29). Read back, and try
    // the toggle once if the pause did not take.
    for cmd in [PAUSE_ON, PAUSE_TOGGLE] {
        let _ = sim.command(cmd).await;
        tokio::time::sleep(std::time::Duration::from_millis(700)).await;
        if sim.read(PAUSED).await.is_ok_and(|p| p > 0.5) {
            return true;
        }
    }
    false
}

/// How long the run's end waits for a low, descending aircraft to climb.
const PAUSE_WAIT_LOW_S: f64 = 90.0;

/// An airborne aircraft may be frozen here: not going down, or out of the
/// ground's reach AND a minute from it at the rate it is sinking. Height
/// alone was not enough: the run's end froze a Zibo 0.6 s into a recovery at
/// 407 ft going down at 1,850 fpm (13 s from the ground), banked 27 degrees
/// (LEMG 30, 2026-10-07, #232).
pub fn pause_safe(agl_ft: f64, vs_fpm: f64) -> bool {
    if vs_fpm > -200.0 {
        return true;
    }
    let to_ground_s = agl_ft / (-vs_fpm / 60.0);
    agl_ft >= crate::tools::recover::GROUND_ESCAPE_AGL_FT && to_ground_s >= PAUSE_TO_GROUND_S
}

/// The least time from the ground, at the sink rate, a descending aircraft
/// may be frozen at.
const PAUSE_TO_GROUND_S: f64 = 60.0;

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
    // What the runner last set LiveTraffic to: shown, as found. Only a case
    // that wants otherwise touches it, so a spec without `traffic` reads
    // and writes nothing.
    let mut traffic_shown = true;
    'run: for case in &spec.cases {
        for rep in 1..=repeat.max(1) {
            let t0 = now_s();
            let want = case.traffic.unwrap_or(true);
            if want != traffic_shown {
                set_traffic(sim, want).await;
                traffic_shown = want;
            }
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
    if !traffic_shown {
        set_traffic(sim, true).await;
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
// ---- The check: every typed line routed offline, no sim --------------------------

/// The tool a typed line reaches through the grammar, as the command loop
/// routes it: the words normalised (`grammar::normalize`), the local
/// planner's branch named `plan_flight`.
pub fn route_of(
    line: &str,
    reg: &crate::tools::Registry,
    prof: &crate::aircraft::AircraftProfile,
) -> Option<String> {
    if crate::flightplan::parse_request(line).is_some_and(|r| r.dest.is_some()) {
        return Some("plan_flight".into());
    }
    let matchable = crate::grammar::normalize(line);
    crate::grammar::try_parse(&matchable, reg, prof).map(|c| c[0].name.clone())
}

/// One typed line of a case, checked.
#[derive(Debug, Clone, PartialEq)]
pub struct LineCheck {
    pub case: String,
    pub line: String,
    /// The tool (or the pre-grammar branch) it reaches; "-" when checked
    /// not at all or reaching nothing.
    pub routes_to: String,
    /// Why the line will not do what the case wants; `None`: OK.
    pub problem: Option<String>,
    /// What the check could not judge, or what only a running flight
    /// decides.
    pub note: String,
}

/// What a line must reach.
enum Want<'a> {
    /// The step's own tool (`plan_flight`: the local planner).
    Tool(&'a str),
    /// Any tool: a case's command.
    AnyTool,
}

/// One line routed as the command loop routes a typed line: taken by a
/// pre-grammar branch (`main::pre_grammar_capture`, the loop's own
/// `question_shortcut`), else the grammar. Pure.
fn check_line(
    case: &str,
    line: &str,
    want: Want,
    reg: &crate::tools::Registry,
    prof: &crate::aircraft::AircraftProfile,
    note: &str,
) -> LineCheck {
    let capture = crate::pre_grammar_capture(line, reg, prof);
    let plan_flight = crate::Shortcut::PlanFlight.name();
    let (routes_to, mut problem) = match capture {
        Some(c) if c == plan_flight => (c.to_string(), None),
        // A step that names the capture wants it: "say again", the cabin's
        // open line, the instructor (#53, #56, #86, #88 could carry no case).
        Some(c) if matches!(want, Want::Tool(w) if w == c) => (c.to_string(), None),
        Some(c) => (
            c.to_string(),
            Some(format!("taken before the grammar ({c})")),
        ),
        None => match route_of(line, reg, prof) {
            Some(t) => (t, None),
            None => ("-".to_string(), Some("reaches no tool".to_string())),
        },
    };
    if problem.is_none() {
        if let Want::Tool(w) = want {
            if routes_to != w {
                problem = Some(format!("the step names {w}"));
            }
        }
    }
    let notes: Vec<&str> = [crate::state_capture_note(line), Some(note)]
        .into_iter()
        .flatten()
        .filter(|n| !n.is_empty())
        .collect();
    LineCheck {
        case: case.to_string(),
        line: line.to_string(),
        routes_to,
        problem,
        note: notes.join("; "),
    }
}

/// Every line a case types, checked on the airframe's profile (`note`:
/// why this profile, when not its own): the setup's words (an approach's
/// only once its field and runway are known, an enroute point's always),
/// the command, every step that says something. Pure.
pub fn check_case(
    case: &Case,
    prof: &crate::aircraft::AircraftProfile,
    note: &str,
) -> Vec<LineCheck> {
    let reg = crate::tools::Registry::for_aircraft(prof);
    let name = case.name.as_str();
    let mut out = Vec::new();
    let skipped = |line: String, why: &str| LineCheck {
        case: name.to_string(),
        line,
        routes_to: "-".into(),
        problem: None,
        note: why.to_string(),
    };
    match &case.setup {
        Setup::Approach {
            field,
            runway,
            final_nm,
        } => {
            let line = approach_line(field, runway, *final_nm);
            if matches!(field.as_str(), FIELD_PLAN_ORIGIN | FIELD_PLAN_DESTINATION)
                || runway == RUNWAY_FROM_PLAN
            {
                out.push(skipped(line, "not checked: the field or runway is the plan's"));
            } else if runway == RUNWAY_FROM_WIND {
                out.push(skipped(line, "not checked: the runway is picked at the setup"));
            } else {
                out.push(check_line(name, &line, Want::Tool("setup_approach"), &reg, prof, note));
            }
        }
        Setup::Enroute { point, .. } => {
            let line = crate::tools::setup_enroute::enroute_line(point);
            out.push(check_line(name, &line, Want::Tool("setup_enroute"), &reg, prof, note));
        }
        Setup::RunwayStart { .. } | Setup::Ramp { .. } => {}
    }
    if let Some(cmd) = &case.command {
        out.push(check_line(name, cmd, Want::AnyTool, &reg, prof, note));
    }
    for s in case.steps.iter().filter(|s| s.is_say()) {
        match s.tool.as_deref() {
            Some(t) => out.push(check_line(name, &s.say, Want::Tool(t), &reg, prof, note)),
            None => {
                let mut c = check_line(name, &s.say, Want::AnyTool, &reg, prof, note);
                c.problem.get_or_insert_with(|| "the step names no tool".into());
                out.push(c);
            }
        }
    }
    out
}

/// The profile a case's lines are checked on: the detector's for its
/// `.acf` (read under `xplane_root()`), the generic one for an unmatched
/// model, else the 737's; the second value says why when not its own.
pub fn check_profile(acf: Option<&str>) -> (crate::aircraft::AircraftProfile, String) {
    let b738 = || crate::aircraft::load("boeing_737_800").expect("the 737 profile");
    let Some(acf) = acf.filter(|a| *a != AIRCRAFT_FROM_PLAN) else {
        return (b738(), "on the 737: the aircraft is the loaded one or the plan's".into());
    };
    let Ok(text) = std::fs::read_to_string(crate::config::xplane_path(acf)) else {
        return (b738(), "on the 737: the .acf is not readable here".into());
    };
    let icao = crate::tools::setup::acf_icao(&text).unwrap_or_default();
    match crate::detect::resolved_id(&icao, acf).and_then(|id| crate::aircraft::load(&id)) {
        Some(p) => (p, String::new()),
        None => (
            crate::aircraft::generic_from_acf(&icao, &text),
            format!("no profile for this {icao}: the generic one"),
        ),
    }
}

/// The check as a table, one row per line.
pub fn check_table(spec_path: &str, rows: &[LineCheck], notes: &[String]) -> String {
    let mut out = format!("# sortie check: {spec_path}\n\n");
    for n in notes {
        out.push_str(&format!("- {n}\n"));
    }
    if !notes.is_empty() {
        out.push('\n');
    }
    out.push_str("| Case | Line | Routes to | Result | Note |\n|---|---|---|---|---|\n");
    for r in rows {
        let result = match &r.problem {
            Some(p) => format!("PROBLEM: {p}"),
            None if r.routes_to == "-" => "skipped".into(),
            None => "OK".into(),
        };
        out.push_str(&format!(
            "| {} | {} | {} | {} | {} |\n",
            cell(&r.case),
            cell(&r.line),
            cell(&r.routes_to),
            cell(&result),
            cell(&r.note)
        ));
    }
    let bad = rows.iter().filter(|r| r.problem.is_some()).count();
    out.push_str(&format!("\n{bad} problems in {} lines.\n", rows.len()));
    out
}

/// A source file's code without its test module, and without whitespace
/// (a snippet matches however the file is formatted).
fn code_of(text: &str) -> String {
    let flat: String = text.chars().filter(|c| !c.is_whitespace()).collect();
    match flat.find("#[cfg(test)]mod") {
        Some(i) => flat[..i].to_string(),
        None => flat,
    }
}

/// How far around a producer's snippet its fields are looked for, in
/// characters of whitespace-free code (a `json!` of a dozen lines).
const SHAPE_WINDOW: usize = 1_200;

/// The [`READ_SHAPES`] the source does not write as the runner reads them:
/// a producer's snippet missing from its file, or a field read that none of
/// its producers writes near the snippet. `read`: a file's text by its path
/// from the checkout's root.
pub fn shape_problems(read: &dyn Fn(&str) -> Option<String>) -> Vec<String> {
    let mut out = Vec::new();
    for s in READ_SHAPES {
        let what = match s.key {
            "" => s.kind.to_string(),
            k => format!("{} {k}={}", s.kind, s.value),
        };
        let mut windows = Vec::new();
        for (file, snippet) in s.producer {
            let Some(text) = read(file) else {
                out.push(format!("{what}: {file} is not readable"));
                continue;
            };
            let code = code_of(&text);
            let snip = code_of(snippet);
            match code.find(&snip) {
                Some(i) => {
                    let mut lo = i.saturating_sub(SHAPE_WINDOW);
                    while !code.is_char_boundary(lo) {
                        lo -= 1;
                    }
                    let mut hi = (i + snip.len() + SHAPE_WINDOW).min(code.len());
                    while !code.is_char_boundary(hi) {
                        hi += 1;
                    }
                    windows.push(code[lo..hi].to_string());
                }
                None => out.push(format!("{what}: {file} no longer writes {snippet:?}")),
            }
        }
        for f in s.fields {
            let lit = format!("\"{f}\"");
            if !windows.is_empty() && !windows.iter().any(|w| w.contains(&lit)) {
                out.push(format!("{what}: no producer writes the field {f}"));
            }
        }
    }
    // The writer's own fields, on every record.
    let writer = "crates/sim/src/blackbox.rs";
    match read(writer).map(|t| code_of(&t)) {
        Some(code) => {
            for f in WRITER_FIELDS {
                if !code.contains(&format!("\"{f}\"")) {
                    out.push(format!("{writer} no longer adds {f} to a record"));
                }
            }
        }
        None => out.push(format!("{writer} is not readable")),
    }
    out
}

/// A test-only source file by its path: `tests.rs`, `*_tests.rs` or under
/// a `tests/` directory. A test module moved to its own file carries no
/// `#[cfg(test)]` of its own, so its words are not the app's.
fn is_test_source(path: &str) -> bool {
    path.ends_with("/tests.rs") || path.ends_with("_tests.rs") || path.contains("/tests/")
}

/// A file's text before its first test module (where [`code_of`] cuts).
fn live_part(text: &str) -> &str {
    let mut from = 0;
    while let Some(i) = text[from..].find("#[cfg(test)]") {
        let at = from + i;
        let rest = text[at + "#[cfg(test)]".len()..].trim_start();
        if rest.starts_with("mod") {
            return &text[..at];
        }
        from = at + 1;
    }
    text
}

/// A module's live text by its file's path (`src/full_auto.rs`): the file
/// itself, or, once the module is a directory too, the live part of the
/// file and of each non-test file under `src/full_auto/`, so a producer
/// that moved to a submodule is still found. `sources`: (path, text).
fn module_text(path: &str, sources: &[(String, String)]) -> Option<String> {
    let own = sources.iter().find(|(p, _)| p == path).map(|(_, t)| t)?;
    let dir = format!("{}/", path.trim_end_matches(".rs"));
    let mut subs: Vec<&(String, String)> = sources
        .iter()
        .filter(|(p, _)| p.starts_with(&dir) && !is_test_source(p))
        .collect();
    if subs.is_empty() {
        return Some(own.clone());
    }
    subs.sort_by(|a, b| a.0.cmp(&b.0));
    let mut text = live_part(own).to_string();
    for (_, t) in subs {
        text.push('\n');
        text.push_str(live_part(t));
    }
    Some(text)
}

/// Every `.rs` file under the checkout's `src/` and `crates/`, by its path
/// from the root (`None`: not run from a checkout).
fn checkout_sources() -> Option<Vec<(String, String)>> {
    fn walk(dir: &std::path::Path, out: &mut Vec<(String, String)>) {
        let Ok(rd) = std::fs::read_dir(dir) else {
            return;
        };
        for e in rd.flatten() {
            let p = e.path();
            if p.is_dir() && p.file_name().is_some_and(|n| n != "target") {
                walk(&p, out);
            } else if p.extension().is_some_and(|x| x == "rs") {
                let path = p.to_string_lossy().replace('\\', "/");
                if is_test_source(&path) {
                    continue;
                }
                if let Ok(t) = std::fs::read_to_string(&p) {
                    out.push((path, t));
                }
            }
        }
    }
    if !std::path::Path::new("src/sortie_runner.rs").is_file() {
        return None;
    }
    let mut out = Vec::new();
    for root in ["src", "crates"] {
        walk(std::path::Path::new(root), &mut out);
    }
    Some(out)
}

/// A spec's words the app must write for them to mean anything: every
/// `event` trigger's kind and event, and every `expect_give_up`, as string
/// literals somewhere in the app's code. `sources`: (path, text).
pub fn spec_word_problems(spec: &Spec, sources: &[(String, String)]) -> Vec<String> {
    let code: Vec<String> = sources
        .iter()
        .filter(|(p, _)| !p.ends_with("sortie_runner.rs"))
        .map(|(_, t)| code_of(t))
        .collect();
    let written = |w: &str| {
        let lit = format!("\"{w}\"");
        code.iter().any(|c| c.contains(&lit))
    };
    let mut out = Vec::new();
    for c in &spec.cases {
        for s in &c.steps {
            if let Trigger::Event { kind, event } = &s.when {
                for w in std::iter::once(kind).chain(event.as_ref()) {
                    if !written(w) {
                        out.push(format!("case {:?}: no record writes {w:?}", c.name));
                    }
                }
            }
        }
        if let Some(g) = &c.expect_give_up {
            if !written(g) {
                out.push(format!("case {:?}: no give-up is named {g:?}", c.name));
            }
        }
    }
    out
}

/// `xvl sortie <spec> --check`: the spec parsed, its plan words resolved
/// when a SimBrief id is set (else said), every typed line routed offline
/// on its airframe's profile. `Ok(true)`: no problem. No sim is asked.
async fn check_cmd(spec_path: &str) -> Result<bool> {
    let text =
        std::fs::read_to_string(spec_path).map_err(|e| anyhow!("reading {spec_path}: {e}"))?;
    let mut notes = Vec::new();
    let mut shape_bad = 0;
    let installed = std::path::Path::new(&crate::config::xplane_root()).is_dir();
    let mut spec = if installed {
        parse_spec(&text)?
    } else {
        notes.push("no X-Plane install here: the aircraft paths are not checked".into());
        parse_spec_with(&text, &|_| true)?
    };
    if spec.cases.iter().any(Case::uses_plan) {
        if crate::settings::live().simbrief_id.trim().is_empty() {
            notes.push("no XCOPILOT_SIMBRIEF_ID: the plan's fields are not resolved".into());
        } else {
            match with_the_plan(spec.clone()).await {
                Ok(s) => spec = s,
                Err(e) => notes.push(format!("the plan's fields are not resolved: {e}")),
            }
        }
    }
    match checkout_sources() {
        Some(src) => {
            let read = |p: &str| module_text(p, &src);
            let problems: Vec<String> = shape_problems(&read)
                .into_iter()
                .chain(spec_word_problems(&spec, &src))
                .collect();
            for p in &problems {
                notes.push(format!("PROBLEM: {p}"));
            }
            shape_bad = problems.len();
        }
        None => notes.push("not run from the checkout: the record shapes are not checked".into()),
    }
    let mut rows = Vec::new();
    for c in &spec.cases {
        let (prof, note) = check_profile(c.aircraft.as_deref());
        rows.extend(check_case(c, &prof, &note));
    }
    print!("{}", check_table(spec_path, &rows, &notes));
    Ok(shape_bad == 0 && rows.iter().all(|r| r.problem.is_none()))
}

pub async fn main_cmd(args: &[String], base_url: &str, apt_dat: &str) -> Result<bool> {
    let mut spec_path = None;
    let mut repeat = 1u32;
    let mut log = "logs/blackbox.jsonl".to_string();
    let mut check = false;
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
            "--check" => check = true,
            _ => spec_path = Some(a.clone()),
        }
    }
    let spec_path = spec_path.ok_or_else(|| {
        anyhow!("usage: xvl sortie <spec.yaml> [--repeat N] [--log path] [--check]")
    })?;
    if check {
        return check_cmd(&spec_path).await;
    }
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
mod tests;
