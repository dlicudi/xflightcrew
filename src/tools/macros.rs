//! Checklists the copilot **runs for you** — say "run the before-takeoff
//! checklist" (or "configure for takeoff") and it reads each item aloud, does the
//! ones it can (the switches), and calls out the ones that are yours (visual
//! checks, the run-up, judgment items). Autonomous: no back-and-forth, it just
//! performs the flow top to bottom, like a pilot-monitoring would.
//!
//! Each checklist is a plain table of [`Step`]s (no checklist files, no parsing).
//! A step either carries an action the copilot performs, or is a spoken call-out
//! the pilot owns. The interactive runner lives in `main.rs` (it needs the voice
//! the way the old per-tool readback path doesn't).

use anyhow::{bail, Result};
use serde_json::{json, Value};

use super::autopilot::AutopilotOff;
use super::controls::{SetParkingBrake, SetPitchTrim};
use super::electrical::SetElectrical;
use super::engine::{SetFuelSelector, SetMixture, SetThrottle, StopEngine};
use super::flaps::SetFlaps;
use super::lights::SetLights;
use super::radio::SetTransponderMode;
use super::{Tool, ToolCtx, ToolSpec};
use crate::aircraft::{FlapProfile, LightProfile};

/// One checklist item. The copilot speaks `say`; if `act` is set it performs
/// that action, otherwise the item is the pilot's and is only called out.
pub struct Step {
    pub say: String,
    pub act: Option<(&'static str, Value)>,
}

fn act(say: &str, tool: &'static str, args: Value) -> Step {
    Step { say: say.to_string(), act: Some((tool, args)) }
}

/// A pilot-owned item — spoken so the pilot knows it's theirs; not performed.
fn pilot(say: &str) -> Step {
    Step { say: say.to_string(), act: None }
}

/// One checklist: a name, the LLM-facing description, the steps, and the profile
/// bits the actions need.
pub struct Checklist {
    pub id: &'static str,
    pub title: &'static str,
    pub description: String,
    pub steps: Vec<Step>,
    pub lights: Vec<LightProfile>,
    pub flaps: Option<FlapProfile>,
}

impl Checklist {
    /// The tool spec the LLM routes against (no arguments — the name is the pick).
    pub fn spec(&self) -> ToolSpec {
        ToolSpec {
            name: self.id.to_string(),
            description: self.description.clone(),
            parameters: json!({ "type": "object", "properties": {}, "additionalProperties": false }),
        }
    }
}

/// Perform one checklist action by dispatching to the matching single-action
/// tool (which reads the dataref back, so the action is verified). Lights/flaps
/// tools are built from the profile.
pub async fn run_step(
    tool: &str,
    args: &Value,
    lights: &[LightProfile],
    flaps: &Option<FlapProfile>,
    ctx: &ToolCtx<'_>,
) -> Result<String> {
    match tool {
        "set_parking_brake" => SetParkingBrake.execute(args, ctx).await,
        "set_pitch_trim" => SetPitchTrim.execute(args, ctx).await,
        "set_electrical" => SetElectrical.execute(args, ctx).await,
        "set_mixture" => SetMixture.execute(args, ctx).await,
        "set_throttle" => SetThrottle.execute(args, ctx).await,
        "set_fuel_selector" => SetFuelSelector.execute(args, ctx).await,
        "set_transponder_mode" => SetTransponderMode.execute(args, ctx).await,
        "autopilot_off" => AutopilotOff.execute(args, ctx).await,
        "stop_engine" => StopEngine.execute(args, ctx).await,
        "set_lights" => SetLights::new(lights.to_vec()).execute(args, ctx).await,
        "set_flaps" => match flaps {
            Some(f) => SetFlaps::new(f.clone()).execute(args, ctx).await,
            None => bail!("no flaps on this airframe"),
        },
        other => bail!("checklist step references unknown tool {other}"),
    }
}

/// Build a checklist's LLM description: trigger phrases plus the anti-greedy
/// guard so it doesn't steal a single-system command.
fn describe(name: &str, triggers: &str) -> String {
    format!(
        "Run the {name} checklist — the copilot reads each item aloud, does the \
         ones it can, and calls out the pilot's items. ONLY for a single command \
         asking for the whole checklist: {triggers}. Do NOT use this when one \
         system is named ('mixture rich', 'flaps up') — those have their own \
         tools; pick this only when no individual system is mentioned."
    )
}

/// The operational checklists, transposed step-for-step. The copilot performs the
/// settable items and calls out the tests/judgment/visual ones as the pilot's.
pub fn checklists(lights: Vec<LightProfile>, flaps: Option<FlapProfile>) -> Vec<Checklist> {
    let make = |id, title, description, steps| Checklist {
        id,
        title,
        description,
        steps,
        lights: lights.clone(),
        flaps: flaps.clone(),
    };

    let mut out = vec![
        make(
            "before_starting_engine",
            "before starting engine",
            describe("before-starting-engine", "'before starting engine', 'before start', 'pre-start checklist'"),
            vec![
                act("Parking brake, set.", "set_parking_brake", json!({ "on": true })),
                act("Autopilot, off.", "autopilot_off", json!({})),
                act("Avionics, off.", "set_electrical", json!({ "system": "avionics", "on": false })),
                act("Fuel selector, both.", "set_fuel_selector", json!({ "position": "both" })),
            ],
        ),
        make(
            "before_taxi",
            "before taxi",
            describe("before-taxi", "'before taxi', 'before taxi checklist', 'ready to taxi'"),
            vec![
                pilot("Fuel quantity — your check."),
                pilot("Flight controls — free and correct, your check."),
                pilot("Flight instruments — set, your check."),
                act("Transponder, standby.", "set_transponder_mode", json!({ "mode": "standby" })),
                act("Taxi light, on.", "set_lights", json!({ "light": "taxi", "on": true })),
                pilot("Brakes — your check."),
                act("Parking brake, set.", "set_parking_brake", json!({ "on": true })),
            ],
        ),
        make(
            "before_takeoff",
            "before takeoff",
            describe(
                "before-takeoff",
                "'before takeoff', 'before takeoff checklist', 'configure for takeoff', 'takeoff config', 'set up for takeoff', 'get ready for takeoff'",
            ),
            vec![
                act("Parking brake, set.", "set_parking_brake", json!({ "on": true })),
                act("Mixture, rich.", "set_mixture", json!({ "percent": 100 })),
                act("Fuel selector, both.", "set_fuel_selector", json!({ "position": "both" })),
                pilot("Mag check — your run-up. Maximum drop one twenty-five, difference fifty."),
                pilot("Engine instruments — green, your check."),
                act("Strobe lights, on.", "set_lights", json!({ "light": "strobe", "on": true })),
                act("Landing light, on.", "set_lights", json!({ "light": "landing", "on": true })),
                act("Autopilot, off.", "autopilot_off", json!({})),
                act("Trim, set for takeoff.", "set_pitch_trim", json!({ "action": "takeoff" })),
                pilot("Flaps — as required, your call."),
            ],
        ),
        make(
            "normal_takeoff",
            "normal takeoff",
            describe("normal-takeoff", "'normal takeoff', 'normal takeoff checklist', 'lined up'"),
            vec![
                act("Taxi light, off.", "set_lights", json!({ "light": "taxi", "on": false })),
                act("Transponder, altitude.", "set_transponder_mode", json!({ "mode": "altitude" })),
                act("Mixture, rich.", "set_mixture", json!({ "percent": 100 })),
                pilot("Throttle and brakes — yours for the roll. Rotate around sixty."),
            ],
        ),
        make(
            "climb",
            "climb",
            describe("climb", "'climb', 'climb checklist', 'climb flow'"),
            vec![
                act("Landing light, off.", "set_lights", json!({ "light": "landing", "on": false })),
                act("Throttle, full.", "set_throttle", json!({ "percent": 100 })),
                act("Mixture, rich.", "set_mixture", json!({ "percent": 100 })),
            ],
        ),
        make(
            "cruise",
            "cruise",
            describe("cruise", "'cruise', 'cruise checklist', 'level off complete'"),
            vec![
                pilot("Power — your call, twenty-one to twenty-five hundred RPM."),
                pilot("Trim — your call."),
                act("Mixture, leaned.", "set_mixture", json!({ "percent": 65 })),
                pilot("Fuel quantity — your check."),
            ],
        ),
        make(
            "descent",
            "descent",
            describe("descent", "'descent', 'descent checklist', 'starting down'"),
            vec![
                pilot("Power — your call."),
                act("Mixture, rich.", "set_mixture", json!({ "percent": 100 })),
                pilot("Altimeter — set, your check."),
                act("Fuel selector, both.", "set_fuel_selector", json!({ "position": "both" })),
                pilot("Flaps — as required, your call."),
            ],
        ),
        make(
            "before_landing",
            "before landing",
            describe(
                "before-landing",
                "'before landing', 'before landing checklist', 'configure for landing', 'set up for landing', 'get ready for landing'",
            ),
            vec![
                act("Fuel selector, both.", "set_fuel_selector", json!({ "position": "both" })),
                act("Mixture, rich.", "set_mixture", json!({ "percent": 100 })),
                act("Landing light, on.", "set_lights", json!({ "light": "landing", "on": true })),
                act("Taxi light, on.", "set_lights", json!({ "light": "taxi", "on": true })),
                act("Autopilot, off.", "autopilot_off", json!({})),
            ],
        ),
        make(
            "after_landing",
            "after landing",
            describe("after-landing", "'after landing', 'after landing checklist', 'clean up', 'clear of the runway'"),
            vec![
                act("Flaps, up.", "set_flaps", json!({ "notch": "up" })),
                act("Landing light, off.", "set_lights", json!({ "light": "landing", "on": false })),
                act("Strobe lights, off.", "set_lights", json!({ "light": "strobe", "on": false })),
                act("Nav lights, off.", "set_lights", json!({ "light": "nav", "on": false })),
                act("Transponder, standby.", "set_transponder_mode", json!({ "mode": "standby" })),
            ],
        ),
    ];

    // Engine shutdown: brake, kill the engine, every light off, then shed the
    // electrical load with the master last. Built dynamically for the lights.
    let mut secure = vec![
        act("Parking brake, set.", "set_parking_brake", json!({ "on": true })),
        act("Transponder, off.", "set_transponder_mode", json!({ "mode": "off" })),
        act("Autopilot, off.", "autopilot_off", json!({})),
        act("Engine, shutting down — mixture cutoff, mags off.", "stop_engine", json!({})),
    ];
    for l in &lights {
        secure.push(act(&format!("{}, off.", capitalize(&l.spoken)), "set_lights", json!({ "light": l.key, "on": false })));
    }
    secure.push(act("Avionics, off.", "set_electrical", json!({ "system": "avionics", "on": false })));
    secure.push(act("Alternator, off.", "set_electrical", json!({ "system": "alternator", "on": false })));
    secure.push(act("Master switch, off.", "set_electrical", json!({ "system": "master", "on": false })));
    secure.push(pilot("Fuel selector — left or right if parked on a slope, your call."));
    out.push(make(
        "secure_aircraft",
        "secure the aircraft",
        describe(
            "shutdown-and-secure",
            "'secure the aircraft', 'secure', 'engine shutdown', 'shutdown checklist', 'shut everything down', 'full shutdown'. For JUST the engine say 'shut down the engine'",
        ),
        secure,
    ));

    out
}

fn capitalize(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
        None => String::new(),
    }
}
