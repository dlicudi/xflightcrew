//! Ground controls: `set_parking_brake`.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};

const PARK_BRAKE: &str = "sim/cockpit2/controls/parking_brake_ratio";

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
