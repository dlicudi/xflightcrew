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
use super::engine::{
    SetCarbHeat, SetFuelPump, SetFuelSelector, SetMagnetos, SetMixture, SetThrottle, StartEngine, StopEngine,
};
use super::flaps::SetFlaps;
use super::instruments::SetPitotHeat;
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
        "set_magnetos" => SetMagnetos.execute(args, ctx).await,
        "set_pitot_heat" => SetPitotHeat.execute(args, ctx).await,
        "set_carb_heat" => SetCarbHeat.execute(args, ctx).await,
        "set_fuel_pump" => SetFuelPump.execute(args, ctx).await,
        "set_transponder_mode" => SetTransponderMode.execute(args, ctx).await,
        "autopilot_off" => AutopilotOff.execute(args, ctx).await,
        // The start crank (voltage-gated, waits for catch, brings alt + avionics
        // up) — invoked as the "Ignition, start" step of the starting checklist.
        "start_engine" => StartEngine.execute(args, ctx).await,
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

/// The operational checklists, transposed step-for-step from the file. The
/// copilot performs the settable items and calls out the tests/judgment/visual
/// ones as the pilot's, so the whole list is read out top to bottom.
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
            "performance_limits",
            "performance limits",
            describe("performance-limits reference (spoken, nothing to do)", "'performance limits', 'limits', 'v speeds', 'say the limits'"),
            vec![
                pilot("Never-exceed, one sixty-three knots. Max cruise, one twenty-nine. Maneuvering, one oh-five."),
                pilot("Flaps ten, one ten knots. Flaps twenty and full, eighty-five."),
                pilot("Useable fuel, fifty-three gallons. Max gross weight, twenty-five fifty-eight pounds."),
                pilot("Service ceiling, fourteen thousand feet. Max crosswind, fifteen knots."),
            ],
        ),
        make(
            "preflight",
            "preflight",
            describe("preflight cabin", "'preflight', 'preflight checklist', 'cabin preflight'"),
            vec![
                act("Parking brake, set.", "set_parking_brake", json!({ "on": true })),
                act("Ignition, off.", "set_magnetos", json!({ "position": "off" })),
                act("Avionics, off.", "set_electrical", json!({ "system": "avionics", "on": false })),
                act("Master switch, on.", "set_electrical", json!({ "system": "battery", "on": true })),
                pilot("Fuel quantity — sufficient, your check."),
                act("Avionics, on.", "set_electrical", json!({ "system": "avionics", "on": true })),
                pilot("Avionics cooling fan — audible, your check."),
                act("Avionics, off.", "set_electrical", json!({ "system": "avionics", "on": false })),
                pilot("Static source — normal, your check."),
                pilot("Warning lights — tested, your check."),
                act("Fuel selector, both.", "set_fuel_selector", json!({ "position": "both" })),
                pilot("Fuel shutoff — pushed in, your check."),
                act("Flaps, full — testing.", "set_flaps", json!({ "notch": "full" })),
                act("Flaps, up.", "set_flaps", json!({ "notch": "up" })),
                act("Pitot heat, on.", "set_pitot_heat", json!({ "on": true })),
                pilot("Ammeter — load check, your check."),
                act("Pitot heat, off.", "set_pitot_heat", json!({ "on": false })),
                act("Master switch, off.", "set_electrical", json!({ "system": "battery", "on": false })),
                act("Trim, set for takeoff.", "set_pitch_trim", json!({ "action": "takeoff" })),
            ],
        ),
        make(
            "before_starting_engine",
            "before starting engine",
            describe("before-starting-engine (the pre-start setup, NOT the start itself)", "'before starting engine', 'before start', 'pre-start checklist'"),
            vec![
                act("Parking brake, set.", "set_parking_brake", json!({ "on": true })),
                act("Autopilot, off.", "autopilot_off", json!({})),
                act("Avionics, off.", "set_electrical", json!({ "system": "avionics", "on": false })),
                act("Fuel selector, both.", "set_fuel_selector", json!({ "position": "both" })),
                pilot("Fuel shutoff — pushed in, your check."),
            ],
        ),
        make(
            "starting_engine",
            "starting engine",
            describe("starting-engine (actually STARTS the engine — cranks it)", "'starting engine', 'starting engine checklist', 'start the engine checklist'"),
            vec![
                act("Master switch, on.", "set_electrical", json!({ "system": "battery", "on": true })),
                act("Beacon, on.", "set_lights", json!({ "light": "beacon", "on": true })),
                act("Throttle, cracked.", "set_throttle", json!({ "percent": 15 })),
                act("Mixture, rich.", "set_mixture", json!({ "percent": 100 })),
                act("Carb heat, cold.", "set_carb_heat", json!({ "on": false })),
                act("Aux fuel pump, prime.", "set_fuel_pump", json!({ "on": true })),
                act("Aux fuel pump, off.", "set_fuel_pump", json!({ "on": false })),
                pilot("Propeller area — clear, your check."),
                act("Ignition, start.", "start_engine", json!({})),
                pilot("Oil pressure — checked, your check."),
                act("Alternator, on.", "set_electrical", json!({ "system": "alternator", "on": true })),
                pilot("Ammeter — checked, your check."),
                pilot("Nav lights — on at night, your call."),
                act("Avionics, on.", "set_electrical", json!({ "system": "avionics", "on": true })),
                act("Flaps, up.", "set_flaps", json!({ "notch": "up" })),
            ],
        ),
        make(
            "before_taxi",
            "before taxi",
            describe("before-taxi", "'before taxi', 'before taxi checklist', 'ready to taxi'"),
            vec![
                pilot("Fuel quantity — checked, your check."),
                pilot("Flight controls — free and correct, your check."),
                pilot("Cabin doors — closed and latched, your check."),
                pilot("Flight instruments — set, your check."),
                pilot("Radios and avionics — set, your check."),
                act("Transponder, standby.", "set_transponder_mode", json!({ "mode": "standby" })),
                act("Taxi light, on.", "set_lights", json!({ "light": "taxi", "on": true })),
                pilot("Brakes — checked, your check."),
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
                pilot("Warning lights — clear, your check."),
                act("Strobe lights, on.", "set_lights", json!({ "light": "strobe", "on": true })),
                act("Landing light, on.", "set_lights", json!({ "light": "landing", "on": true })),
                pilot("Radios — checked and set, your check."),
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
                pilot("Flaps — as required, your call."),
                act("Mixture, rich.", "set_mixture", json!({ "percent": 100 })),
                pilot("Throttle and brakes — yours for the roll. Rotate around sixty, climb seventy to eighty-five."),
                pilot("Flaps — up once climbing, your call."),
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
                pilot("Climb airspeed — seventy to eighty-five knots."),
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
                pilot("Navaids and radios — set, your check."),
                pilot("Navigation — on course, your check."),
                pilot("Flight instruments — checked, your check."),
                pilot("Engine instruments — green, your check."),
                pilot("Fuel quantity — sufficient, your check."),
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
                pilot("Nav and GPS — set, your check."),
                act("Fuel selector, both.", "set_fuel_selector", json!({ "position": "both" })),
                pilot("Flaps — as required. Zero to ten below a hundred and ten, ten to thirty below eighty-five."),
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
            "normal_landing",
            "normal landing",
            describe("normal-landing", "'normal landing', 'normal landing checklist', 'on final', 'short final'"),
            vec![
                pilot("Flaps — as required. Approach flaps up sixty-five to seventy-five, flaps down sixty to seventy."),
                pilot("Touchdown — mains first, then lower the nose gently."),
                pilot("Brakes — minimum required, your call."),
                act("Flaps, up.", "set_flaps", json!({ "notch": "up" })),
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

    // Engine shutdown: brake, shed the load, kill the engine, then master last.
    // Built dynamically for the lights.
    let mut secure = vec![
        act("Parking brake, set.", "set_parking_brake", json!({ "on": true })),
        act("Transponder, off.", "set_transponder_mode", json!({ "mode": "off" })),
        act("Autopilot, off.", "autopilot_off", json!({})),
        act("Avionics, off.", "set_electrical", json!({ "system": "avionics", "on": false })),
        act("Throttle, idle, a thousand RPM.", "set_throttle", json!({ "percent": 12 })),
        act("Engine, shutting down — mixture cutoff, mags off.", "stop_engine", json!({})),
    ];
    for l in &lights {
        secure.push(act(&format!("{}, off.", capitalize(&l.spoken)), "set_lights", json!({ "light": l.key, "on": false })));
    }
    secure.push(act("Alternator, off.", "set_electrical", json!({ "system": "alternator", "on": false })));
    secure.push(act("Master switch, off.", "set_electrical", json!({ "system": "master", "on": false })));
    secure.push(pilot("Fuel selector — left or right if parked on a slope, your call."));
    out.push(make(
        "secure_aircraft",
        "secure the aircraft",
        describe(
            "shutdown-and-secure",
            "'secure the aircraft', 'secure', 'engine shutdown', 'shutdown checklist', 'shut everything down', 'full shutdown'. For JUST the engine (no securing) say 'shut down the engine'",
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
