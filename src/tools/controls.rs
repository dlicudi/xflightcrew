//! Ground controls: `set_parking_brake`, and pitch trim (`set_pitch_trim`).

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};

const PARK_BRAKE: &str = "sim/cockpit2/controls/parking_brake_ratio";
/// Elevator trim, -1.0 (full nose down) .. +1.0 (full nose up), 0 = neutral.
const ELEV_TRIM: &str = "sim/cockpit2/controls/elevator_trim";
/// How much one "trim nose up/down" nudge moves the trim wheel.
const TRIM_STEP: f64 = 0.10;
/// Takeoff trim setting for the 172 — roughly neutral (the takeoff band).
const TRIM_TAKEOFF: f64 = 0.0;

pub struct SetParkingBrake;

#[derive(Deserialize)]
struct Args {
    /// true = set the parking brake, false = release it.
    on: bool,
}

#[async_trait]
impl Tool for SetParkingBrake {
    fn name(&self) -> &str {
        "set_parking_brake"
    }

    fn description(&self) -> &str {
        "Set or release the parking brake. Use for 'parking brake set', 'set the \
         brakes', 'release parking brake', 'brakes off'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "on": {"type": "boolean", "description": "true = set, false = release"}
            },
            "required": ["on"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: Args = serde_json::from_value(args.clone())?;

        // If it's already where you asked, say so instead of a redundant "Roger".
        let already = ctx.xplane.get_float(PARK_BRAKE).await? > 0.5;
        if already == args.on {
            return Ok(format!(
                "Parking brake is already {}.",
                if already { "set" } else { "off" }
            ));
        }

        ctx.xplane.set_float(PARK_BRAKE, if args.on { 1.0 } else { 0.0 }).await?;
        let set = ctx.xplane.get_float(PARK_BRAKE).await? > 0.5;
        Ok(format!("Roger, parking brake {}.", if set { "set" } else { "released" }))
    }
}

pub struct SetPitchTrim;

#[derive(Deserialize)]
struct TrimArgs {
    /// What to do: nudge "nose_up"/"nose_down", or set "neutral"/"takeoff".
    action: String,
}

#[async_trait]
impl Tool for SetPitchTrim {
    fn name(&self) -> &str {
        "set_pitch_trim"
    }

    fn description(&self) -> &str {
        "Adjust the elevator (pitch) trim. 'nose_up' and 'nose_down' nudge the \
         trim a step at a time; 'neutral' centres it; 'takeoff' sets the takeoff \
         position. Use for 'trim nose up', 'trim nose down', 'a little nose up \
         trim', 'centre the trim', 'set takeoff trim'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "action": {"type": "string", "enum": ["nose_up", "nose_down", "neutral", "takeoff"],
                           "description": "Trim action."}
            },
            "required": ["action"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: TrimArgs = serde_json::from_value(args.clone())?;
        let current = ctx.xplane.get_float(ELEV_TRIM).await?;
        let target = match a.action.to_ascii_lowercase().as_str() {
            "nose_up" => (current + TRIM_STEP).min(1.0),
            "nose_down" => (current - TRIM_STEP).max(-1.0),
            "neutral" => 0.0,
            "takeoff" => TRIM_TAKEOFF,
            other => return Ok(format!("I didn't catch the trim command '{other}'.")),
        };
        ctx.xplane.set_float(ELEV_TRIM, target).await?;
        let actual = ctx.xplane.get_float(ELEV_TRIM).await?;
        Ok(format!("Roger, {}.", describe_trim(actual)))
    }
}

/// Spoken description of a trim position.
fn describe_trim(t: f64) -> String {
    let pct = (t.abs() * 100.0).round();
    if t.abs() < 0.03 {
        "trim neutral".to_string()
    } else if t > 0.0 {
        format!("trim {pct:.0} percent nose up")
    } else {
        format!("trim {pct:.0} percent nose down")
    }
}
