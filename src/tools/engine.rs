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
// Per-engine actuator arrays (the 172 is single-engine → element 0).
const FUEL_PUMP: &str = "sim/cockpit2/engine/actuators/fuel_pump_on";
const IGNITION_KEY: &str = "sim/cockpit2/engine/actuators/ignition_key"; // 0 off,1 R,2 L,3 both,4 start
const CARB_HEAT: &str = "sim/cockpit2/engine/actuators/carb_heat_ratio"; // 0 cold .. 1 hot
const FUEL_SELECTOR: &str = "sim/cockpit2/fuel/fuel_tank_selector"; // 0 off,1 left,2 right,4 both

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

pub struct SetFuelPump;

#[derive(Deserialize)]
struct OnArg {
    on: bool,
}

#[async_trait]
impl Tool for SetFuelPump {
    fn name(&self) -> &str {
        "set_fuel_pump"
    }
    fn description(&self) -> &str {
        "Turn the electric (auxiliary) fuel pump on or off. Use for 'fuel pump on', \
         'boost pump on', 'fuel pump off', 'aux fuel pump on'."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": { "on": {"type": "boolean", "description": "true = on, false = off"} },
            "required": ["on"]
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: OnArg = serde_json::from_value(args.clone())?;
        ctx.xplane.set_int_array_elem(FUEL_PUMP, 0, a.on as i64).await?;
        let on = ctx.xplane.get_float_array(FUEL_PUMP).await?.first().copied().unwrap_or(0.0) > 0.5;
        Ok(format!("Roger, fuel pump {}.", if on { "on" } else { "off" }))
    }
}

pub struct SetCarbHeat;

#[async_trait]
impl Tool for SetCarbHeat {
    fn name(&self) -> &str {
        "set_carb_heat"
    }
    fn description(&self) -> &str {
        "Turn carburetor (alternate air) heat on or off. Use for 'carb heat on', \
         'carb heat off', 'carburetor heat on'."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": { "on": {"type": "boolean", "description": "true = hot (on), false = cold (off)"} },
            "required": ["on"]
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: OnArg = serde_json::from_value(args.clone())?;
        ctx.xplane.set_int_array_elem(CARB_HEAT, 0, a.on as i64).await?;
        let on = ctx.xplane.get_float_array(CARB_HEAT).await?.first().copied().unwrap_or(0.0) > 0.5;
        Ok(format!("Roger, carb heat {}.", if on { "on" } else { "off" }))
    }
}

pub struct SetMagnetos;

#[derive(Deserialize)]
struct MagArgs {
    /// Magneto position: off, left, right, both.
    position: String,
}

#[async_trait]
impl Tool for SetMagnetos {
    fn name(&self) -> &str {
        "set_magnetos"
    }
    fn description(&self) -> &str {
        "Set the magnetos / ignition position: off, left, right, or both. Used in \
         the run-up mag check. Use for 'magnetos both', 'mags to left', 'mags \
         right', 'magnetos off'. To start the engine use start_engine, not this."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "position": {"type": "string", "enum": ["off", "left", "right", "both"],
                             "description": "Magneto selector position."}
            },
            "required": ["position"]
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: MagArgs = serde_json::from_value(args.clone())?;
        let key = match a.position.to_ascii_lowercase().as_str() {
            "off" => 0,
            "right" => 1,
            "left" => 2,
            "both" => 3,
            other => return Ok(format!("I didn't catch the magneto position '{other}'. Say off, left, right, or both.")),
        };
        ctx.xplane.set_int_array_elem(IGNITION_KEY, 0, key).await?;
        let actual = ctx.xplane.get_float_array(IGNITION_KEY).await?.first().copied().unwrap_or(-1.0).round() as i64;
        let name = match actual { 0 => "off", 1 => "right", 2 => "left", 3 => "both", 4 => "start", _ => "unknown" };
        Ok(format!("Roger, magnetos {name}."))
    }
}

pub struct SetFuelSelector;

#[derive(Deserialize)]
struct FuelSelArgs {
    /// Tank position: off, left, right, both.
    position: String,
}

#[async_trait]
impl Tool for SetFuelSelector {
    fn name(&self) -> &str {
        "set_fuel_selector"
    }
    fn description(&self) -> &str {
        "Set the fuel tank selector: off, left, right, or both. Use for 'fuel \
         selector both', 'fuel to left tank', 'select right tank', 'fuel selector \
         off'."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "position": {"type": "string", "enum": ["off", "left", "right", "both"],
                             "description": "Fuel tank selector position."}
            },
            "required": ["position"]
        })
    }
    fn confirm_prompt(&self, args: &Value) -> Option<String> {
        let a: FuelSelArgs = serde_json::from_value(args.clone()).ok()?;
        // Selecting OFF starves the engine — confirm that one.
        if a.position.eq_ignore_ascii_case("off") {
            Some("fuel selector to off — that cuts fuel to the engine".to_string())
        } else {
            None
        }
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: FuelSelArgs = serde_json::from_value(args.clone())?;
        let val = match a.position.to_ascii_lowercase().as_str() {
            "off" => 0,
            "left" => 1,
            "right" => 2,
            "both" => 4,
            other => return Ok(format!("I didn't catch the fuel position '{other}'. Say off, left, right, or both.")),
        };
        ctx.xplane.set_int(FUEL_SELECTOR, val).await?;
        let actual = ctx.xplane.get_float(FUEL_SELECTOR).await?.round() as i64;
        let name = match actual { 0 => "off", 1 => "left", 2 => "right", 4 => "both", _ => "set" };
        Ok(format!("Roger, fuel selector {name}."))
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
