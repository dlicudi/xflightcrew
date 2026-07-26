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
const COM2: &str = "sim/cockpit2/radios/actuators/com2_frequency_hz_833";
const XPDR_CODE: &str = "sim/cockpit2/radios/actuators/transponder_code";
const XPDR_MODE: &str = "sim/cockpit2/radios/actuators/transponder_mode";
/// NAV1 active frequency, integer in units of 10 kHz (10950 = 109.50 MHz).
const NAV1: &str = "sim/cockpit2/radios/actuators/nav1_frequency_hz";
const NAV1_OBS: &str = "sim/cockpit2/radios/actuators/nav1_obs_deg_mag_pilot";
const CMD_COM1_FLIP: &str = "sim/radios/com1_standy_flip";
const CMD_COM2_FLIP: &str = "sim/radios/com2_standy_flip";
const CMD_XPDR_IDENT: &str = "sim/transponder/transponder_ident";

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
         1200', 'squawk 7000', 'set transponder to 4521', 'squawk VFR' (1200), \
         'set squawk 0021'. Common codes: VFR = 1200, emergency = 7700, lost \
         comms = 7600, hijack = 7500."
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
            && format!("{:04}", code)
                .bytes()
                .all(|b| (b'0'..=b'7').contains(&b));
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

pub struct SetCom2;

#[async_trait]
impl Tool for SetCom2 {
    fn name(&self) -> &str {
        "set_com2"
    }

    fn description(&self) -> &str {
        "Tune the COM2 radio to a frequency in megahertz (118.0–136.975). Use for \
         'com two 121.9', 'set com two to 119.1'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "mhz": {"type": "number", "minimum": 118.0, "maximum": 136.975,
                        "description": "COM frequency in MHz, e.g. 121.9"}
            },
            "required": ["mhz"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: Args = serde_json::from_value(args.clone())?;
        let mhz = args.mhz.clamp(118.0, 136.975);
        let khz = (mhz * 1000.0).round() as i64;
        ctx.xplane.set_int(COM2, khz).await?;
        let actual_mhz = ctx.xplane.get_float(COM2).await? / 1000.0;
        Ok(format!("Roger, com two {actual_mhz:.3}."))
    }
}

pub struct SwapRadio;

#[derive(Deserialize)]
struct SwapArgs {
    /// Which radio to flip active/standby: "com1" or "com2".
    radio: String,
}

#[async_trait]
impl Tool for SwapRadio {
    fn name(&self) -> &str {
        "swap_radio"
    }

    fn description(&self) -> &str {
        "Swap a COM radio's active and standby frequencies. Use for 'swap com one', \
         'flip com two', 'swap com one frequencies', 'make standby active on com \
         one', 'flip the frequencies', 'swap com radios'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "radio": {"type": "string", "enum": ["com1", "com2"],
                          "description": "Which COM radio to swap."}
            },
            "required": ["radio"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: SwapArgs = serde_json::from_value(args.clone())?;
        let (cmd, spoken) = match a.radio.to_ascii_lowercase().as_str() {
            "com1" => (CMD_COM1_FLIP, "com one"),
            "com2" => (CMD_COM2_FLIP, "com two"),
            other => {
                return Ok(format!(
                    "I didn't catch which radio '{other}'. Say com one or com two."
                ))
            }
        };
        ctx.xplane.run_command(cmd).await?;
        Ok(format!("Roger, swapped {spoken}."))
    }
}

pub struct SetNav1;

#[derive(Deserialize)]
struct NavArgs {
    /// Frequency in MHz (e.g. 109.50, 111.30).
    mhz: f64,
}

#[async_trait]
impl Tool for SetNav1 {
    fn name(&self) -> &str {
        "set_nav1"
    }

    fn description(&self) -> &str {
        "Tune the NAV1 radio (VOR/ILS) to a frequency in megahertz \
         (108.0–117.95). Use for 'nav one 109.5', 'set nav one to 111.30', \
         'tune nav one one zero eight decimal one'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "mhz": {"type": "number", "minimum": 108.0, "maximum": 117.95,
                        "description": "NAV frequency in MHz, e.g. 109.50"}
            },
            "required": ["mhz"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: NavArgs = serde_json::from_value(args.clone())?;
        let mhz = a.mhz.clamp(108.0, 117.95);
        // NAV freq dataref is in units of 10 kHz: 109.50 MHz -> 10950.
        let units = (mhz * 100.0).round() as i64;
        ctx.xplane.set_int(NAV1, units).await?;
        let actual = ctx.xplane.get_float(NAV1).await? / 100.0;
        Ok(format!("Roger, nav one {actual:.2}."))
    }
}

pub struct SetObs;

#[derive(Deserialize)]
struct ObsArgs {
    /// Course/radial in degrees (0–359).
    course: f64,
}

#[async_trait]
impl Tool for SetObs {
    fn name(&self) -> &str {
        "set_obs"
    }

    fn description(&self) -> &str {
        "Set the NAV1 OBS course (the selected VOR radial / course pointer) in \
         degrees. Use for 'set OBS 240', 'course 090', 'set the OBS to two four \
         zero', 'select the 180 radial', 'set the radial', 'course pointer to \
         090', 'OBS to 360'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "course": {"type": "integer", "minimum": 0, "maximum": 360,
                           "description": "Course in degrees magnetic (0-359)."}
            },
            "required": ["course"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: ObsArgs = serde_json::from_value(args.clone())?;
        let course = a.course.rem_euclid(360.0);
        ctx.xplane.set_float(NAV1_OBS, course).await?;
        let actual = ctx.xplane.get_float(NAV1_OBS).await?;
        Ok(format!("Roger, OBS {actual:03.0}."))
    }
}

pub struct SetTransponderMode;

#[derive(Deserialize)]
struct ModeArgs {
    /// off, standby, on, altitude, or ident.
    mode: String,
}

#[async_trait]
impl Tool for SetTransponderMode {
    fn name(&self) -> &str {
        "set_transponder_mode"
    }

    fn description(&self) -> &str {
        "Set the transponder mode (off, standby, on, altitude) or send ident. Use \
         for 'transponder standby', 'squawk altitude', 'squawk mode charlie', \
         'transponder on', 'ident', 'squawk ident', 'transponder off', \
         'go to standby', 'mode C on'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "mode": {"type": "string", "enum": ["off", "standby", "on", "altitude", "ident"],
                         "description": "Transponder mode, or 'ident' to send a squawk ident."}
            },
            "required": ["mode"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: ModeArgs = serde_json::from_value(args.clone())?;
        // 'ident' is momentary — it presses the ident button, leaving the mode as-is.
        if a.mode.eq_ignore_ascii_case("ident") {
            ctx.xplane.run_command(CMD_XPDR_IDENT).await?;
            return Ok("Roger, ident.".to_string());
        }
        let (val, spoken) = match a.mode.to_ascii_lowercase().as_str() {
            "off" => (0, "off"),
            "standby" => (1, "standby"),
            "on" => (2, "on"),
            "altitude" => (3, "altitude"),
            other => return Ok(format!("I didn't catch the transponder mode '{other}'.")),
        };
        ctx.xplane.set_int(XPDR_MODE, val).await?;
        Ok(format!("Roger, transponder {spoken}."))
    }
}
