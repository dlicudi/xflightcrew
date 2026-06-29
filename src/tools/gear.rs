//! Landing gear — `gear(up|down)`. Only registered for retractable-gear airframes
//! (the profile's `retractable_gear` flag); the 172 has fixed gear, so it never
//! gets this tool. Owns the gear handle and a ground safety gate.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};

/// Gear handle position: 1 = down, 0 = up. Writable.
const GEAR_HANDLE: &str = "sim/cockpit2/controls/gear_handle_down";
/// 1 while any wheel is on the ground — the gate against retracting on the runway.
const ON_GROUND: &str = "sim/flightmodel/failures/onground_any";
const IAS_KTS: &str = "sim/cockpit2/gauges/indicators/airspeed_kts_pilot";

pub struct Gear {
    /// Gear limit speed (VLO/VLE), KIAS — refuse cycling above it. `None` = no gate.
    pub vlo_kts: Option<f64>,
}

#[derive(Deserialize)]
struct Args {
    /// "up" or "down".
    position: String,
}

#[async_trait]
impl Tool for Gear {
    fn name(&self) -> &str {
        "gear"
    }

    fn description(&self) -> &str {
        "Raise or lower the landing gear. Use for 'gear up', 'gear down', 'landing \
         gear up', 'positive rate, gear up', 'gear down for landing', 'retract the \
         gear', 'put the gear down'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "position": {"type": "string", "enum": ["up", "down"]}
            },
            "required": ["position"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: Args = serde_json::from_value(args.clone())?;
        let up = matches!(a.position.to_ascii_lowercase().as_str(), "up" | "retract" | "raise");

        // Safety gate: never raise the gear while we're on the ground.
        if up {
            let on_ground = ctx.xplane.get_float(ON_GROUND).await.unwrap_or(0.0) > 0.5;
            if on_ground {
                return Ok("Negative, we're on the ground. I won't raise the gear.".to_string());
            }
        }
        // Safety gate: don't cycle the gear above its limit speed (VLO/VLE).
        if let Some(vlo) = self.vlo_kts {
            let ias = ctx.xplane.get_float(IAS_KTS).await.unwrap_or(0.0);
            if ias > vlo {
                return Ok(format!(
                    "Negative, {ias:.0} knots is over the {vlo:.0} knot gear limit. Slow down first."
                ));
            }
        }

        ctx.xplane.set_float(GEAR_HANDLE, if up { 0.0 } else { 1.0 }).await?;
        // Read the handle back (the gear itself takes seconds to cycle, but the
        // handle reflects the commanded position immediately).
        let down = ctx.xplane.get_float(GEAR_HANDLE).await.unwrap_or(if up { 0.0 } else { 1.0 }) > 0.5;
        Ok(format!("Roger, gear {}.", if down { "down" } else { "up" }))
    }
}
