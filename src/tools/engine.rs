//! Engine controls: `set_throttle` and `set_mixture`.
//!
//! Both are continuous 0.0–1.0 ratios in X-Plane; we expose them as 0–100 % so
//! the pilot can say "throttle 60 percent" / "mixture full rich".

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};

const THROTTLE: &str = "sim/cockpit2/engine/actuators/throttle_ratio_all";
const MIXTURE: &str = "sim/cockpit2/engine/actuators/mixture_ratio_all";
const BATTERY: &str = "sim/cockpit/electrical/battery_on";
const ONGROUND: &str = "sim/flightmodel/failures/onground_any";
const GROUNDSPEED: &str = "sim/flightmodel/position/groundspeed"; // m/s
const PARK_BRAKE: &str = "sim/cockpit2/controls/parking_brake_ratio";
const ENGN_RUNNING: &str = "sim/flightmodel/engine/ENGN_running";
const CMD_MAGS_BOTH: &str = "sim/magnetos/magnetos_both";
const CMD_MAGS_OFF: &str = "sim/magnetos/magnetos_off";
const CMD_STARTER: &str = "sim/starters/engage_starter_1";

#[derive(Deserialize)]
struct PercentArg {
    percent: f64,
}

pub struct SetThrottle;

#[async_trait]
impl Tool for SetThrottle {
    fn name(&self) -> &str {
        "set_throttle"
    }
    fn description(&self) -> &str {
        "Set the throttle as a percentage, 0 (idle) to 100 (full power). Use for \
         'throttle 60 percent', 'full throttle', 'throttle idle'."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "percent": {"type": "integer", "minimum": 0, "maximum": 100,
                            "description": "0 = idle, 100 = full power"}
            },
            "required": ["percent"]
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let p: PercentArg = serde_json::from_value(args.clone())?;
        let ratio = (p.percent / 100.0).clamp(0.0, 1.0);
        ctx.xplane.set_float(THROTTLE, ratio).await?;
        let actual = ctx.xplane.get_float(THROTTLE).await? * 100.0;
        Ok(format!("Roger, throttle {actual:.0} percent."))
    }
}

pub struct SetMixture;

#[async_trait]
impl Tool for SetMixture {
    fn name(&self) -> &str {
        "set_mixture"
    }
    fn description(&self) -> &str {
        "Set the fuel mixture as a percentage: 100 = full rich, 0 = idle cutoff, \
         intermediate values lean the mixture. Use for 'mixture full rich', \
         'lean the mixture to 70 percent', 'mixture idle cutoff'."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "percent": {"type": "integer", "minimum": 0, "maximum": 100,
                            "description": "100 = full rich, 0 = idle cutoff"}
            },
            "required": ["percent"]
        })
    }
    fn confirm_prompt(&self, args: &Value) -> Option<String> {
        let p: PercentArg = serde_json::from_value(args.clone()).ok()?;
        // Only idle cutoff (0) is gated — it stops the engine. Leaning is routine
        // and executes immediately.
        if p.percent <= 0.0 {
            Some("mixture to idle cutoff — that will stop the engine".to_string())
        } else {
            None
        }
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let p: PercentArg = serde_json::from_value(args.clone())?;
        let ratio = (p.percent / 100.0).clamp(0.0, 1.0);
        ctx.xplane.set_float(MIXTURE, ratio).await?;
        let actual = ctx.xplane.get_float(MIXTURE).await? * 100.0;
        Ok(format!("Roger, mixture {actual:.0} percent."))
    }
}

pub struct StartEngine;

#[async_trait]
impl Tool for StartEngine {
    fn name(&self) -> &str {
        "start_engine"
    }
    fn description(&self) -> &str {
        "Start the engine: runs the start sequence (battery on, mixture rich, \
         throttle cracked, magnetos both, engage the starter). Use for 'start the \
         engine', 'start her up', 'engine start', 'engine on', 'engines on', \
         'switch the engine on', 'turn the engine on'. Note: 'engine on' means \
         START the engine, not an electrical switch."
    }
    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        // Already running? Don't re-crank a live engine.
        let already_running = ctx
            .xplane
            .get_float_array(ENGN_RUNNING)
            .await
            .ok()
            .and_then(|v| v.first().copied())
            .map(|r| r > 0.5)
            .unwrap_or(false);
        if already_running {
            return Ok("Engine's already running.".to_string());
        }

        // Pre-start configuration.
        ctx.xplane.set_int(BATTERY, 1).await?;
        ctx.xplane.set_float(MIXTURE, 1.0).await?; // full rich
        ctx.xplane.set_float(THROTTLE, 0.1).await?; // crack the throttle
        ctx.xplane.run_command(CMD_MAGS_BOTH).await?;

        // Crank the starter (held a few seconds so it can catch).
        ctx.xplane.run_command_for(CMD_STARTER, 3.0).await?;

        // Give it a moment to fire, then confirm engine 1 is running.
        for _ in 0..6 {
            tokio::time::sleep(std::time::Duration::from_millis(700)).await;
            let running = ctx
                .xplane
                .get_float_array(ENGN_RUNNING)
                .await
                .ok()
                .and_then(|v| v.first().copied())
                .map(|r| r > 0.5)
                .unwrap_or(false);
            if running {
                return Ok("Roger, engine started.".to_string());
            }
        }
        Ok("Cranking — the engine hasn't caught. Check fuel and mixture, then try again.".to_string())
    }
}

pub struct StopEngine;

#[async_trait]
impl Tool for StopEngine {
    fn name(&self) -> &str {
        "stop_engine"
    }
    fn description(&self) -> &str {
        "Shut down the engine: pull the mixture to idle cutoff and turn the \
         magnetos off. Use for 'shut down the engine', 'engine off', 'kill the \
         engine', 'shutdown'."
    }
    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        // Shutting the engine down in flight is an engine failure. Rather than
        // ask for confirmation, refuse unless it's actually safe: on the ground,
        // stopped, and parking brake set.
        if ctx.xplane.get_float(ONGROUND).await? < 0.5 {
            return Ok("Negative — we're airborne. I won't shut the engine down in flight.".to_string());
        }
        if ctx.xplane.get_float(GROUNDSPEED).await? > 1.0 {
            return Ok("Negative — still moving. Bring her to a stop first.".to_string());
        }
        if ctx.xplane.get_float(PARK_BRAKE).await? < 0.5 {
            return Ok("Set the parking brake first, then I'll shut the engine down.".to_string());
        }

        ctx.xplane.set_float(MIXTURE, 0.0).await?; // idle cutoff
        ctx.xplane.run_command(CMD_MAGS_OFF).await?;

        for _ in 0..5 {
            tokio::time::sleep(std::time::Duration::from_millis(600)).await;
            let running = ctx
                .xplane
                .get_float_array(ENGN_RUNNING)
                .await
                .ok()
                .and_then(|v| v.first().copied())
                .map(|r| r > 0.5)
                .unwrap_or(false);
            if !running {
                return Ok("Roger, engine shut down.".to_string());
            }
        }
        Ok("Mixture to cutoff, engine winding down.".to_string())
    }
}
