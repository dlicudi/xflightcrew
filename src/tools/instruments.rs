//! Instrument settings: `set_barometer` (altimeter setting).

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};

const BARO: &str = "sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot";
const DG_SYNC: &str = "sim/instruments/DG_sync_mag";
const DG_DRIFT: &str = "sim/cockpit/gyros/dg_drift_vac_deg";

pub struct SetBarometer;

#[derive(Deserialize)]
struct Args {
    /// Altimeter setting in inches of mercury (e.g. 29.92).
    inhg: f64,
}

#[async_trait]
impl Tool for SetBarometer {
    fn name(&self) -> &str {
        "set_barometer"
    }

    fn description(&self) -> &str {
        "Set the altimeter (barometric pressure) in inches of mercury. Use for \
         'set altimeter two niner niner two' (29.92), 'altimeter 30.15', \
         'set baro 29.80'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "inhg": {
                    "type": "number",
                    "minimum": 28.0,
                    "maximum": 31.5,
                    "description": "Altimeter setting in inches of mercury, e.g. 29.92."
                }
            },
            "required": ["inhg"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: Args = serde_json::from_value(args.clone())?;
        let inhg = args.inhg.clamp(28.0, 31.5);
        ctx.xplane.set_float(BARO, inhg).await?;
        let actual = ctx.xplane.get_float(BARO).await?;
        Ok(format!("Roger, altimeter {actual:.2}."))
    }
}

pub struct AlignGyro;

#[async_trait]
impl Tool for AlignGyro {
    fn name(&self) -> &str {
        "align_gyro"
    }

    fn description(&self) -> &str {
        "Align the directional gyro (heading indicator) to the magnetic compass. \
         Use for 'align the gyro', 'sync the heading indicator', 'set the DG', \
         'sync the directional gyro'."
    }

    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }

    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        ctx.xplane.run_command(DG_SYNC).await?;
        // Confirm the drift is gone (best-effort).
        let drift = ctx.xplane.get_float(DG_DRIFT).await.unwrap_or(0.0);
        if drift.abs() < 1.0 {
            Ok("Roger, directional gyro synced to the compass.".to_string())
        } else {
            Ok(format!("Gyro synced; still {drift:.0} degrees of drift."))
        }
    }
}
