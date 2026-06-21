//! Phase-config macros — the operational checklists transposed into one-command
//! sequences, in the pure actor model (no checklist files, no verify/challenge,
//! no per-item interaction; like [`engine::StartEngine`](super::engine::StartEngine)
//! bundles the start). Each macro is a *table of steps* — literally the
//! checklist — that composes the existing single-action tools, so there's no
//! duplicated dataref logic.
//!
//! Principle: a macro performs the **steady SET items** of a phase. It does NOT
//! do *tests* (the brake check, the mag run-up — the pilot observes those),
//! *judgment* items ("flaps as required", power settings), or *visual* checks
//! (controls free, oil pressure). Those stay with the pilot.

use anyhow::{bail, Result};
use async_trait::async_trait;
use serde_json::{json, Value};

use super::autopilot::AutopilotOff;
use super::controls::{SetParkingBrake, SetPitchTrim};
use super::electrical::SetElectrical;
use super::engine::{SetCarbHeat, SetFuelSelector, SetMixture, SetThrottle, StopEngine};
use super::flaps::SetFlaps;
use super::lights::SetLights;
use super::radio::SetTransponderMode;
use super::{Tool, ToolCtx};
use crate::aircraft::{FlapProfile, LightProfile};

/// Dispatch one macro step to the matching single-action tool. Keeps the macro
/// definitions a plain `(tool, args)` table; lights/flaps tools are built from
/// the profile passed through.
async fn run_step(
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
        "set_carb_heat" => SetCarbHeat.execute(args, ctx).await,
        "set_transponder_mode" => SetTransponderMode.execute(args, ctx).await,
        "autopilot_off" => AutopilotOff.execute(args, ctx).await,
        "stop_engine" => StopEngine.execute(args, ctx).await,
        "set_lights" => SetLights::new(lights.to_vec()).execute(args, ctx).await,
        "set_flaps" => match flaps {
            Some(f) => SetFlaps::new(f.clone()).execute(args, ctx).await,
            None => bail!("no flaps on this airframe"),
        },
        other => bail!("macro step references unknown tool {other}"),
    }
}

/// One checklist transposed into a macro: a name, the LLM-facing description, the
/// step table, and a spoken summary.
pub struct PhaseMacro {
    name: &'static str,
    description: String,
    summary: &'static str,
    steps: Vec<(&'static str, Value)>,
    lights: Vec<LightProfile>,
    flaps: Option<FlapProfile>,
}

#[async_trait]
impl Tool for PhaseMacro {
    fn name(&self) -> &str {
        self.name
    }

    fn description(&self) -> &str {
        &self.description
    }

    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }

    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let mut failed = 0usize;
        for (tool, args) in &self.steps {
            if run_step(tool, args, &self.lights, &self.flaps, ctx).await.is_err() {
                failed += 1;
            }
        }
        if failed == self.steps.len() {
            bail!("couldn't run the {} flow — is X-Plane reachable?", self.name);
        }
        if failed == 0 {
            Ok(self.summary.to_string())
        } else {
            Ok(format!("{} — but {failed} item(s) didn't take; check the panel.", self.summary))
        }
    }
}

/// Build the description with trigger phrases and the anti-greedy guard the model
/// needs so a macro doesn't steal a single-system command.
fn describe(does: &str, triggers: &str) -> String {
    format!(
        "Run the WHOLE {does} in one step. ONLY for a single command asking for \
         the entire flow: {triggers}. Do NOT use this when a single system is \
         named (e.g. 'mixture rich', 'flaps up') — those have their own tools; \
         pick this only when no individual system is mentioned."
    )
}

/// The operational checklists, transposed. Each is the *settable* subset of the
/// checklist in order; tests, judgment, and visual items are left to the pilot.
pub fn phase_macros(lights: Vec<LightProfile>, flaps: Option<FlapProfile>) -> Vec<Box<dyn Tool>> {
    let m = |name, description, summary, steps: Vec<(&'static str, Value)>| -> Box<dyn Tool> {
        Box::new(PhaseMacro { name, description, summary, steps, lights: lights.clone(), flaps: flaps.clone() })
    };

    vec![
        m(
            "before_starting_engine",
            describe("before-starting-engine flow", "'before starting engine', 'before start', 'pre-start checklist'"),
            "Roger — before start: parking brake set, autopilot off, avionics off, fuel selector both.",
            vec![
                ("set_parking_brake", json!({ "on": true })),
                ("autopilot_off", json!({})),
                ("set_electrical", json!({ "system": "avionics", "on": false })),
                ("set_fuel_selector", json!({ "position": "both" })),
            ],
        ),
        m(
            "before_taxi",
            describe("before-taxi flow", "'before taxi', 'before taxi checklist', 'ready to taxi'"),
            "Roger — before taxi: transponder standby, taxi light on.",
            vec![
                ("set_transponder_mode", json!({ "mode": "standby" })),
                ("set_lights", json!({ "light": "taxi", "on": true })),
            ],
        ),
        m(
            "before_takeoff",
            describe(
                "before-takeoff configuration",
                "'before takeoff', 'before takeoff checklist', 'configure for takeoff', 'takeoff config', 'set up for takeoff', 'get ready for takeoff'",
            ),
            "Roger — before takeoff: brake set, mixture rich, fuel both, strobes and landing light on, autopilot off, trim set for takeoff. The mag check and flaps are yours.",
            vec![
                ("set_parking_brake", json!({ "on": true })),
                ("set_mixture", json!({ "percent": 100 })),
                ("set_fuel_selector", json!({ "position": "both" })),
                ("set_lights", json!({ "light": "strobe", "on": true })),
                ("set_lights", json!({ "light": "landing", "on": true })),
                ("autopilot_off", json!({})),
                ("set_pitch_trim", json!({ "action": "takeoff" })),
            ],
        ),
        m(
            "normal_takeoff",
            describe("normal-takeoff configuration", "'normal takeoff', 'normal takeoff checklist', 'lined up'"),
            "Roger — takeoff items: taxi light off, transponder altitude, mixture rich. Throttle and brakes are yours.",
            vec![
                ("set_lights", json!({ "light": "taxi", "on": false })),
                ("set_transponder_mode", json!({ "mode": "altitude" })),
                ("set_mixture", json!({ "percent": 100 })),
            ],
        ),
        m(
            "climb",
            describe("climb flow", "'climb', 'climb checklist', 'climb flow'"),
            "Roger — climb: landing light off, throttle full, mixture rich.",
            vec![
                ("set_lights", json!({ "light": "landing", "on": false })),
                ("set_throttle", json!({ "percent": 100 })),
                ("set_mixture", json!({ "percent": 100 })),
            ],
        ),
        m(
            "cruise",
            describe("cruise flow", "'cruise', 'cruise checklist', 'level off complete'"),
            "Roger — cruise: mixture leaned. Power and trim are yours.",
            vec![("set_mixture", json!({ "percent": 65 }))],
        ),
        m(
            "descent",
            describe("descent flow", "'descent', 'descent checklist', 'starting down'"),
            "Roger — descent: mixture rich, fuel both. Power, altimeter, and flaps are yours.",
            vec![
                ("set_mixture", json!({ "percent": 100 })),
                ("set_fuel_selector", json!({ "position": "both" })),
            ],
        ),
        m(
            "before_landing",
            describe(
                "before-landing configuration",
                "'before landing', 'before landing checklist', 'configure for landing', 'set up for landing', 'get ready for landing'",
            ),
            "Roger — before landing: fuel both, mixture rich, landing and taxi lights on, autopilot off.",
            vec![
                ("set_fuel_selector", json!({ "position": "both" })),
                ("set_mixture", json!({ "percent": 100 })),
                ("set_lights", json!({ "light": "landing", "on": true })),
                ("set_lights", json!({ "light": "taxi", "on": true })),
                ("autopilot_off", json!({})),
            ],
        ),
        m(
            "after_landing",
            describe("after-landing cleanup", "'after landing', 'after landing checklist', 'clean up', 'clear of the runway'"),
            "Roger — after landing: flaps up, landing, strobe and nav lights off, transponder standby. Taxi light's still on.",
            vec![
                ("set_flaps", json!({ "notch": "up" })),
                ("set_lights", json!({ "light": "landing", "on": false })),
                ("set_lights", json!({ "light": "strobe", "on": false })),
                ("set_lights", json!({ "light": "nav", "on": false })),
                ("set_transponder_mode", json!({ "mode": "standby" })),
            ],
        ),
        m(
            "secure_aircraft",
            describe(
                "shutdown-and-secure flow",
                "'secure the aircraft', 'secure', 'engine shutdown', 'shutdown checklist', 'shut everything down', 'full shutdown'. For JUST the engine say 'shut down the engine'",
            ),
            "Roger — secured: parking brake set, engine shut down, all lights off, transponder off, avionics, alternator and master off. Have a good one.",
            {
                // Lights off: every exterior light this airframe has.
                let mut steps: Vec<(&'static str, Value)> = vec![
                    ("set_parking_brake", json!({ "on": true })),
                    ("stop_engine", json!({})),
                    ("set_transponder_mode", json!({ "mode": "off" })),
                ];
                for l in &lights {
                    steps.push(("set_lights", json!({ "light": l.key, "on": false })));
                }
                steps.push(("set_electrical", json!({ "system": "avionics", "on": false })));
                steps.push(("set_electrical", json!({ "system": "alternator", "on": false })));
                steps.push(("set_electrical", json!({ "system": "master", "on": false })));
                steps
            },
        ),
    ]
}
