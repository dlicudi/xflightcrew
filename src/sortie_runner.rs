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
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Case {
    pub name: String,
    pub setup: Setup,
    /// The mission, typed as a pilot would ("full auto", "fly the flight
    /// plan"). An approach case has none: the setup itself arms autoland.
    #[serde(default)]
    pub command: Option<String>,
    #[serde(default)]
    pub deadline_min: Option<f64>,
    /// X-Plane's time multiplier for the flying (`sim/time/sim_speed`), set
    /// once the mission's sortie begins and put back to 1 at the case's end.
    #[serde(default)]
    pub sim_speed: Option<u32>,
    /// Seconds after a ramp or runway start is still before the command.
    #[serde(default)]
    pub settle_s: Option<f64>,
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
}

/// The YAML shape: a map with exactly one of the three keys (serde_yaml
/// reads a plain enum only from `!tags`).
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct SetupMap {
    #[serde(default)]
    approach: Option<ApproachSpec>,
    #[serde(default)]
    runway_start: Option<RunwayStartSpec>,
    #[serde(default)]
    ramp: Option<RampSpec>,
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
        match (m.approach, m.runway_start, m.ramp) {
            (Some(a), None, None) => Ok(Setup::Approach {
                field: a.field,
                runway: a.runway,
                final_nm: a.final_nm,
            }),
            (None, Some(r), None) => Ok(Setup::RunwayStart {
                field: r.field,
                runway: r.runway,
                engines_running: r.engines_running,
            }),
            (None, None, Some(r)) => Ok(Setup::Ramp {
                field: r.field,
                stand: r.stand,
            }),
            _ => Err("setup is exactly one of approach, runway_start, ramp".into()),
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

impl Case {
    pub fn deadline_s(&self) -> f64 {
        self.deadline_min.unwrap_or(DEFAULT_DEADLINE_MIN) * 60.0
    }

    /// The line typed to start the case: the approach setup's own words, or
    /// the case's command.
    pub fn first_line(&self) -> String {
        match &self.setup {
            Setup::Approach {
                field,
                runway,
                final_nm,
            } => approach_line(field, runway, *final_nm),
            _ => self.command.clone().unwrap_or_default(),
        }
    }
}

/// The typed words for a new flight on a final (the grammar's
/// `setup_approach`, pinned by a test against the real parser).
pub fn approach_line(field: &str, runway: &str, final_nm: i64) -> String {
    format!("put us on a {final_nm} mile final to {field} runway {runway}")
}

/// Parse and check a spec. Every problem is named with its case.
pub fn parse_spec(text: &str) -> Result<Spec> {
    let spec: Spec = serde_yaml::from_str(text).map_err(|e| anyhow!("spec: {e}"))?;
    if spec.cases.is_empty() {
        return Err(anyhow!("spec: no cases"));
    }
    for c in &spec.cases {
        let bad = |why: &str| Err(anyhow!("case {:?}: {why}", c.name));
        if c.name.trim().is_empty() {
            return bad("needs a name");
        }
        if c.deadline_min.is_some_and(|d| !(d > 0.0)) {
            return bad("deadline_min must be above zero");
        }
        if c.sim_speed.is_some_and(|s| !(1..=16).contains(&s)) {
            return bad("sim_speed is 1 to 16");
        }
        let has_cmd = c.command.as_deref().is_some_and(|s| !s.trim().is_empty());
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
            }
            _ if !has_cmd => return bad("needs a command"),
            _ => {}
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
        }
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
                    ("full_auto" | "heli_route", "give_up") if mine => {
                        self.give_up = Some(st(r, "reason").to_string());
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
        let pass = r.get("pass").and_then(Value::as_bool) == Some(true);
        let outcome = st(r, "outcome").to_string();
        let stages = std::mem::take(&mut self.stages);
        let mut why: Vec<String> = Vec::new();
        if !pass {
            if r.get("crashed").and_then(Value::as_bool) == Some(true) {
                why.push("the aircraft crashed".into());
            }
            for s in stages
                .iter()
                .filter(|s| s.get("pass") != Some(&Value::Bool(true)))
            {
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
            if r.get("unattended").and_then(Value::as_bool) == Some(false) {
                why.push(format!(
                    "attended ({} interventions, {} commands)",
                    n("interventions"),
                    n("commands")
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
            interventions: n("interventions"),
            commands: n("commands"),
            outcome,
            reason: why.join("; "),
            abort: false,
            crashed: r.get("crashed").and_then(Value::as_bool) == Some(true),
            ..Verdict::default()
        }
    }

    /// The clock and the wreck flag, every driver pass. `crashed` is `None`
    /// when the read failed: no measurement, never a crash.
    pub fn tick(&mut self, now: f64, crashed: Option<bool>) -> Step {
        if self.phase == Phase::Done {
            return Step::Wait;
        }
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
            Phase::AwaitSortie { since } if now - since > SORTIE_GRACE_S => {
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
    format!(
        "{} {} #{}: {} in {:.0} s, sortie {}, {}{}{}",
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
        if v.reason.is_empty() {
            String::new()
        } else {
            format!(": {}", v.reason)
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
         | Case | # | Sortie | Verdict | Stages | Interventions | Secs | Reason |\n\
         |---|---|---|---|---|---|---|---|\n"
    );
    for r in rows {
        let v = &r.verdict;
        let stages: Vec<String> = v.stages.iter().map(stage_numbers).collect();
        out.push_str(&format!(
            "| {} | {} | {} | {} | {} | {} | {:.0} | {} |\n",
            cell(&r.case),
            r.rep,
            v.sortie.as_deref().unwrap_or("-"),
            if v.pass { "PASS" } else { "FAIL" },
            cell(&stages.join("; ")),
            v.interventions,
            r.secs,
            cell(&v.reason),
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
    partial: String,
}

impl BlackboxTail {
    /// From the file's current end (nothing already written is fed).
    pub fn at_end(path: impl Into<std::path::PathBuf>) -> Self {
        let path = path.into();
        let offset = std::fs::metadata(&path).map(|m| m.len()).unwrap_or(0);
        Self {
            path,
            offset,
            partial: String::new(),
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
        self.partial.push_str(&String::from_utf8_lossy(&buf));
        let mut out = Vec::new();
        while let Some(i) = self.partial.find('\n') {
            let line: String = self.partial.drain(..=i).collect();
            if let Ok(v) = serde_json::from_str::<Value>(line.trim()) {
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
    /// Longest wait for the loops of the last case to let go.
    pub idle_max_s: f64,
    /// Longest wait for the last case's wreck to clear ([`CRASH_CLEAR_S`]).
    pub crash_clear_s: f64,
}

impl Pace {
    pub const LIVE: Pace = Pace {
        poll: std::time::Duration::from_millis(500),
        init: InitTiming::RAMP,
        still_s: 5.0,
        still_max_s: 240.0,
        settle_scale: 1.0,
        idle_max_s: 60.0,
        crash_clear_s: CRASH_CLEAR_S,
    };
}

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

/// Between cases: the last case's loops let go, else "my controls". Loops the
/// last case LEFT flying (airborne at its end) are taken back at once, right
/// before this case's setup replaces the flight: the aircraft is nobody's
/// only for the seconds between the two lines.
async fn wait_idle(sim: &dyn RunnerSim, pace: &Pace, take_back_now: bool) -> Result<()> {
    let t0 = now_s();
    let mut told: Option<f64> = None;
    loop {
        match sim.read_string(STATUS_AUTO).await {
            Ok(s) if !loop_flying(&s) => return Ok(()),
            _ => {}
        }
        match told {
            None if take_back_now || now_s() - t0 > pace.idle_max_s => {
                type_line(sim, ABORT_LINE, pace).await?;
                told = Some(now_s());
            }
            Some(t) if now_s() - t > pace.idle_max_s => {
                return Err(anyhow!("a loop is still flying after my controls"));
            }
            _ => {}
        }
        tokio::time::sleep(pace.poll).await;
    }
}

/// A ramp or runway start: write the mailbox, wait for the plugin's answer
/// through the reload, then for the aircraft to sit still.
async fn set_up_start(sim: &dyn RunnerSim, case: &Case, apt_dat: &str, pace: &Pace) -> Result<()> {
    // An approach is set up by the app's own tool, from the typed line.
    if matches!(case.setup, Setup::Approach { .. }) {
        return Ok(());
    }
    let acf = sim
        .read_string(ACF_PATH)
        .await
        .ok()
        .filter(|p| !p.trim().is_empty())
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
                None => pick_stand(sim, apt_dat, field).await?,
            };
            crate::tools::setup::init_json(&acf, field, &stand)
        }
        Setup::Approach { .. } => return Ok(()),
    };
    let run =
        crate::tools::setup::run_init(&Mailbox(sim), body.as_bytes(), pace.init, &|_| {}).await;
    match run.outcome {
        InitOutcome::Done(0) => {}
        InitOutcome::Done(code) => {
            let (field, rwy) = match &case.setup {
                Setup::RunwayStart { field, runway, .. } => (field.as_str(), runway.as_str()),
                Setup::Ramp { field, .. } => (field.as_str(), ""),
                _ => ("", ""),
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
    // the client on the way.
    let t0 = now_s();
    let mut still_since: Option<f64> = None;
    while now_s() - t0 < pace.still_max_s {
        tokio::time::sleep(pace.poll).await;
        let (g, gs) = (
            sim.read(crate::datarefs::ON_GROUND_ANY).await,
            sim.read(GROUNDSPEED).await,
        );
        if g.is_ok_and(|g| g > 0.5) && gs.is_ok_and(|v| v.abs() < 0.5) {
            let s = *still_since.get_or_insert(now_s());
            if now_s() - s >= pace.still_s {
                let settle = case.settle_s.unwrap_or(DEFAULT_SETTLE_S) * pace.settle_scale;
                tokio::time::sleep(std::time::Duration::from_secs_f64(settle)).await;
                return Ok(());
            }
        } else {
            still_since = None;
        }
    }
    Err(anyhow!("the aircraft never sat still after the setup"))
}

/// The stand the setup tool would pick for the loaded type.
async fn pick_stand(sim: &dyn RunnerSim, apt_dat: &str, field: &str) -> Result<String> {
    let icao = sim.read_string(ACF_ICAO).await.unwrap_or_default();
    let known = crate::aircraft::profile_for_type(&icao);
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

/// Fly one case to its verdict. Setup failures are verdicts too.
pub async fn run_case(
    sim: &dyn RunnerSim,
    tail: &mut BlackboxTail,
    case: &Case,
    apt_dat: &str,
    pace: &Pace,
    take_back_now: bool,
) -> Verdict {
    let failed = |reason: String| Verdict {
        reason,
        outcome: "runner".into(),
        ..Verdict::default()
    };
    if let Err(e) = wait_idle(sim, pace, take_back_now).await {
        return failed(e.to_string());
    }
    if let Err(e) = set_up_start(sim, case, apt_dat, pace).await {
        return failed(format!("setup: {e}"));
    }
    let line = case.first_line();
    tail.skip_to_end();
    if let Err(e) = type_line(sim, &line, pace).await {
        return failed(e.to_string());
    }
    let mut m = match case.setup {
        Setup::Approach { .. } => CaseMachine::once(now_s(), case.deadline_s()),
        _ => CaseMachine::new(now_s(), case.deadline_s()),
    };
    m.crash_clear_s = pace.crash_clear_s;
    m.line = Some(line.clone());
    let speed = case.sim_speed.filter(|s| *s != 1);
    let verdict = loop {
        tokio::time::sleep(pace.poll).await;
        let mut done = None;
        for r in tail.poll() {
            match m.feed(&r) {
                Step::Finished(v) => {
                    done = Some(v);
                    break;
                }
                Step::Began => {
                    if let Some(s) = speed {
                        let _ = sim.write_float(SIM_SPEED, s as f64).await;
                    }
                }
                _ => {}
            }
        }
        if let Some(v) = done {
            break v;
        }
        let crashed = sim
            .read(crate::datarefs::HAS_CRASHED)
            .await
            .ok()
            .map(|v| v > 0.5);
        match m.tick(now_s(), crashed) {
            Step::Finished(v) => break v,
            Step::Resend => {
                let _ = type_line(sim, &line, pace).await;
            }
            _ => {}
        }
    };
    if speed.is_some() {
        let _ = sim.write_float(SIM_SPEED, 1.0).await;
    }
    let mut verdict = verdict;
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
            let verdict = run_case(sim, tail, case, apt_dat, pace, left_flying).await;
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
    let spec = parse_spec(&text)?;
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
        println!("{}", row_line(r));
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
            // takes the aircraft back with it frozen, not diving.
            let paused = secure_at_end(&xp).await;
            eprintln!(
                "interrupted: my controls{}",
                if paused { ", X-Plane paused" } else { "" }
            );
            let _ = xp.set_float(SIM_SPEED, 1.0).await;
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

    #[test]
    fn the_shipped_spec_parses() {
        let spec = parse_spec(LEMG).expect("lemg spec");
        assert_eq!(spec.cases.len(), 3);
        assert_eq!(
            spec.cases[0].setup,
            Setup::Approach {
                field: "LEMG".into(),
                runway: "13".into(),
                final_nm: 10
            }
        );
        assert_eq!(
            spec.cases[1].setup,
            Setup::Approach {
                field: "LEMG".into(),
                runway: "13".into(),
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
        for f in std::fs::read_dir("bench/sorties").unwrap().flatten() {
            let p = f.path();
            if p.extension().is_some_and(|e| e == "yaml") {
                let text = std::fs::read_to_string(&p).unwrap();
                parse_spec(&text).unwrap_or_else(|e| panic!("{}: {e}", p.display()));
            }
        }
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
                ..Verdict::default()
            },
        };
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
             alignment pass (err 0.4/3 deg, xtk 0.3/2 m); takeoff pass (xtk 1.1/5 m) | 1 | 900 | \
             taxi: 5.8 m off the line / x |"
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
    }

    impl FakeSim {
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
                _ => 0.0,
            })
        }
        async fn read_string(&self, name: &str) -> Result<String> {
            Ok(match name {
                STATUS_AUTO if self.flying.load(std::sync::atomic::Ordering::SeqCst) => {
                    "1AUTOLAND|final".into()
                }
                STATUS_AUTO => "0MONITORING|".into(),
                ACF_PATH => "Aircraft/Laminar Research/Boeing 737-800/b738.acf".into(),
                _ => String::new(),
            })
        }
        async fn write_float(&self, name: &str, v: f64) -> Result<()> {
            assert_eq!(name, SIM_SPEED);
            self.speeds.lock().unwrap().push(v);
            Ok(())
        }
        async fn command(&self, name: &str) -> Result<()> {
            self.commands.lock().unwrap().push(name.to_string());
            Ok(())
        }
        async fn write_bytes(&self, name: &str, bytes: &[u8]) -> Result<()> {
            use std::sync::atomic::Ordering::SeqCst;
            let line = String::from_utf8_lossy(bytes).to_string();
            if name == "xcopilot/sim/init_flight" {
                self.inits.lock().unwrap().push(line);
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
            } else if self.endless.load(SeqCst) {
                let s = format!("{}-endless", self.typed.lock().unwrap().len());
                self.flying.store(true, SeqCst);
                for r in [
                    json!({"t": 1.0, "kind": "tool", "name": "setup_approach"}),
                    json!({"t": 2.0, "kind": "sortie", "event": "begin", "sortie": s}),
                ] {
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
            crash_clear_s: 0.2,
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
        assert_eq!(
            *sim.speeds.lock().unwrap(),
            vec![2.0, 1.0, 2.0, 1.0],
            "2x while flying, 1x after"
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

    fn quick_pace() -> Pace {
        Pace {
            poll: std::time::Duration::from_millis(5),
            init: InitTiming::LIVE,
            still_s: 0.0,
            still_max_s: 1.0,
            settle_scale: 0.0,
            idle_max_s: 1.0,
            crash_clear_s: 0.2,
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
}
