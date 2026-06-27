//! Ramp / ground-ops — the crew's coordination with the ground: chocks, ground
//! power, pushback, and ground services. These aren't airframe systems; they're
//! X-Plane ground-crew actions (`sim/ground_ops/*` and the chock toggle), so
//! they're available on any aircraft, not gated by the profile.
//!
//! The `toggle_*` commands are toggles, so the on/off tools read the current
//! state first and only fire the command when it differs — the same "confirm the
//! action, not the intent" discipline as the rest of the surface.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};

// --- chocks ---
const CMD_TOGGLE_CHOCKS: &str = "sim/flight_controls/toggle_chocks";
/// Per-wheel chock state (array, 1 = chocked).
const IS_CHOCKED: &str = "sim/flightmodel2/gear/is_chocked";
// --- ground power ---
const CMD_TOGGLE_GPU: &str = "sim/ground_ops/toggle_gpu_request";
/// GPU output volts — > 0 means the aircraft is *receiving* ground power.
const GPU_VOLTS: &str = "sim/cockpit2/electrical/GPU_generator_volts";
// --- pushback ---
const CMD_PUSH_STRAIGHT: &str = "sim/ground_ops/pushback_straight";
const CMD_PUSH_LEFT: &str = "sim/ground_ops/pushback_left";
const CMD_PUSH_RIGHT: &str = "sim/ground_ops/pushback_right";
const CMD_PUSH_STOP: &str = "sim/ground_ops/pushback_stop";
// --- services ---
const CMD_SERVICE: &str = "sim/ground_ops/service_plane";

#[derive(Deserialize)]
struct OnArg {
    on: bool,
}

/// True when the chocks are in.
async fn chocked(ctx: &ToolCtx<'_>) -> bool {
    ctx.xplane
        .get_float_array(IS_CHOCKED)
        .await
        .ok()
        .and_then(|a| a.first().copied())
        .unwrap_or(0.0)
        > 0.5
}

/// True when the aircraft is receiving ground power.
async fn gpu_connected(ctx: &ToolCtx<'_>) -> bool {
    ctx.xplane.get_float(GPU_VOLTS).await.unwrap_or(0.0) > 1.0
}

fn on_off_schema() -> Value {
    json!({
        "type": "object",
        "properties": { "on": { "type": "boolean", "description": "true = on/in, false = off/out" } },
        "required": ["on"]
    })
}

pub struct SetChocks;

#[async_trait]
impl Tool for SetChocks {
    fn name(&self) -> &str {
        "set_chocks"
    }
    fn description(&self) -> &str {
        "Place or remove the wheel chocks. Use for 'chocks in', 'chocks out', \
         'set the chocks', 'remove the chocks', 'pull the chocks', 'chocks away'."
    }
    fn parameters(&self) -> Value {
        on_off_schema()
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: OnArg = serde_json::from_value(args.clone())?;
        // The command is a toggle — only fire it if the state isn't already what
        // was asked. is_chocked updates immediately, so the readback is honest.
        if chocked(ctx).await != a.on {
            ctx.xplane.run_command(CMD_TOGGLE_CHOCKS).await?;
        }
        Ok(if chocked(ctx).await {
            "Roger, chocks in.".to_string()
        } else {
            "Roger, chocks out.".to_string()
        })
    }
}

pub struct SetGroundPower;

#[async_trait]
impl Tool for SetGroundPower {
    fn name(&self) -> &str {
        "set_ground_power"
    }
    fn description(&self) -> &str {
        "Request or disconnect external ground power (GPU). Use for 'ground power \
         on', 'connect ground power', 'request ground power', 'ground power off', \
         'disconnect ground power', 'remove ground power'."
    }
    fn parameters(&self) -> Value {
        on_off_schema()
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: OnArg = serde_json::from_value(args.clone())?;
        let connected = gpu_connected(ctx).await;
        if connected == a.on {
            return Ok(if connected {
                "Ground power's already connected.".to_string()
            } else {
                "Ground power's already disconnected.".to_string()
            });
        }
        ctx.xplane.run_command(CMD_TOGGLE_GPU).await?;
        // The cart takes a moment to roll up and connect, so we can't read the
        // volts back instantly — confirm the request, not an immediate state.
        Ok(if a.on {
            "Roger, requesting ground power.".to_string()
        } else {
            "Roger, disconnecting ground power.".to_string()
        })
    }
}

pub struct Pushback;

#[derive(Deserialize)]
struct PushArg {
    /// "straight" (start), "left", "right", or "stop".
    action: String,
}

#[async_trait]
impl Tool for Pushback {
    fn name(&self) -> &str {
        "pushback"
    }
    fn description(&self) -> &str {
        "Start, steer, or stop a pushback from the gate/parking. Use for 'push \
         back', 'pushback', 'pushback straight', 'pushback left', 'pushback \
         right', 'stop pushback', 'pushback stop', 'hold the pushback'."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "action": {
                    "type": "string",
                    "enum": ["straight", "left", "right", "stop"],
                    "description": "straight = start pushing; left/right steer; stop ends it"
                }
            },
            "required": ["action"]
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: PushArg = serde_json::from_value(args.clone())?;
        let (cmd, say) = match a.action.as_str() {
            "stop" | "halt" => (CMD_PUSH_STOP, "Roger, pushback stopped."),
            "left" => (CMD_PUSH_LEFT, "Roger, pushing back, turning left."),
            "right" => (CMD_PUSH_RIGHT, "Roger, pushing back, turning right."),
            "straight" | "start" => (CMD_PUSH_STRAIGHT, "Roger, pushing back."),
            other => return Ok(format!("Say pushback straight, left, right, or stop — not '{other}'.")),
        };
        ctx.xplane.run_command(cmd).await?;
        Ok(say.to_string())
    }
}

pub struct RequestService;

#[async_trait]
impl Tool for RequestService {
    fn name(&self) -> &str {
        "request_service"
    }
    fn description(&self) -> &str {
        "Call ground services to the aircraft (fuel and ramp services). Use for \
         'request ground services', 'service the aircraft', 'call the trucks', \
         'refuel', 'request fuel', 'top off the tanks', 'get us fuelled'."
    }
    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        ctx.xplane.run_command(CMD_SERVICE).await?;
        Ok("Roger, requesting ground services.".to_string())
    }
}
