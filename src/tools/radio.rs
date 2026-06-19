//! Radio tuning: `set_com1` (COM1 active frequency).
//!
//! X-Plane stores the frequency as an integer in kHz (8.33-capable), e.g.
//! 118075 = 118.075 MHz. We take MHz from the pilot and convert.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};

const COM1: &str = "sim/cockpit2/radios/actuators/com1_frequency_hz_833";
const XPDR_CODE: &str = "sim/cockpit2/radios/actuators/transponder_code";

pub struct SetCom1;

#[derive(Deserialize)]
struct Args {
    /// Frequency in MHz (e.g. 118.5, 121.9).
    mhz: f64,
}

#[async_trait]
impl Tool for SetCom1 {
    fn name(&self) -> &str {
        "set_com1"
    }

    fn description(&self) -> &str {
        "Tune the COM1 radio to a frequency in megahertz (VHF comm band, \
         118.0–136.975). Use for 'com one 118.5', 'set com one to 121.9', \
         'tune com one one two one decimal niner'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "mhz": {
                    "type": "number",
                    "minimum": 118.0,
                    "maximum": 136.975,
                    "description": "COM frequency in MHz, e.g. 118.5"
                }
            },
            "required": ["mhz"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: Args = serde_json::from_value(args.clone())?;
        let mhz = args.mhz.clamp(118.0, 136.975);
        let khz = (mhz * 1000.0).round() as i64;
        ctx.xplane.set_int(COM1, khz).await?;
        let actual_mhz = ctx.xplane.get_float(COM1).await? / 1000.0;
        Ok(format!("Roger, com one {actual_mhz:.3}."))
    }
}

pub struct SetTransponder;

#[derive(Deserialize)]
struct XpdrArgs {
    /// 4-digit octal squawk code (each digit 0–7), e.g. 1200.
    code: i64,
}

#[async_trait]
impl Tool for SetTransponder {
    fn name(&self) -> &str {
        "set_transponder"
    }

    fn description(&self) -> &str {
        "Set the transponder squawk code (4 digits, each 0-7). Use for 'squawk \
         1200', 'squawk 7000', 'set transponder to 4521'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "code": {"type": "integer", "minimum": 0, "maximum": 7777,
                         "description": "4-digit octal squawk code, e.g. 1200"}
            },
            "required": ["code"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: XpdrArgs = serde_json::from_value(args.clone())?;
        let code = args.code;
        // Squawk codes are octal: 0000–7777, no digit above 7.
        let valid = (0..=7777).contains(&code)
            && format!("{:04}", code).bytes().all(|b| (b'0'..=b'7').contains(&b));
        if !valid {
            return Ok(format!(
                "{code:04} isn't a valid squawk — digits must be zero through seven."
            ));
        }
        ctx.xplane.set_int(XPDR_CODE, code).await?;
        let actual = ctx.xplane.get_float(XPDR_CODE).await? as i64;
        Ok(format!("Roger, squawking {actual:04}."))
    }
}
