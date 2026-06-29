//! Jet engine start — `start_engines`. A turbine start is a timed sequence with
//! feedback (crank, watch N2, introduce fuel, wait for idle). On a study-level
//! aircraft like the MD-82 the start-critical controls are custom datarefs/commands
//! the X-Plane Web (REST) API can't drive from outside the sim, so the actual
//! sequence runs IN the bundled plugin (in-process `XPLMCommandOnce` reaches
//! laminar's own handlers). This tool just triggers it via the plugin dataref —
//! the same hand-off the FMS-route loader uses.

use anyhow::Result;
use async_trait::async_trait;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};

/// Plugin dataref: write 1 to kick off the in-plugin engine-start state machine.
const START_CMD: &str = "xcopilot/cmd/start_engines";
/// Read to tell if the engines are already running (per-engine N2, %).
const N2: &str = "sim/cockpit2/engine/indicators/N2_percent";

pub struct StartEngines;

#[async_trait]
impl Tool for StartEngines {
    fn name(&self) -> &str {
        "start_engines"
    }
    fn description(&self) -> &str {
        "Start the jet engines: run the turbine start sequence (bleed air, fuel \
         pumps, ignition, crank each engine to idle). Use for 'start engines', \
         'start the engines', 'engines on', 'start up the engines', 'spool up the \
         engines', 'crank the engines', 'fire up the engines'."
    }
    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {} })
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        // Already turning and burning?
        if let Ok(n2) = ctx.xplane.get_float_array(N2).await {
            if !n2.is_empty() && n2.iter().take(2).all(|&v| v > 50.0) {
                return Ok("Engines are already running.".to_string());
            }
        }
        // Hand the start to the plugin (only it can drive the laminar controls).
        if ctx.xplane.set_int(START_CMD, 1).await.is_err() {
            return Ok("I can't reach the plugin to start the engines. Is the X-COPILOT plugin running?".to_string());
        }
        Ok("Starting engines, stand by.".to_string())
    }
}
