//! `set_heading` — command the autopilot to fly a magnetic heading.
//!
//! "Autopilot heading 270" is three actions: dial the heading bug (a dataref
//! write), engage the servos, and engage HDG mode (two commands). The readback
//! re-reads the bug and the servo state and confirms what actually happened.
//!
//! Datarefs/commands are standard X-Plane (confirmed against the rustdecks
//! profiles), so the tool owns them; the aircraft profile only declares that an
//! autopilot exists.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};

/// Pilot-side heading bug, degrees magnetic (writable).
const HEADING_BUG: &str = "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot";
/// Selected altitude, feet (writable).
const ALT_DIAL: &str = "sim/cockpit2/autopilot/altitude_dial_ft";
/// Selected vertical speed, feet per minute (writable).
const VS_DIAL: &str = "sim/cockpit2/autopilot/vvi_dial_fpm";
/// Servo engagement status (read-only; 1 = autopilot flying).
const SERVOS_ON: &str = "sim/cockpit2/autopilot/servos_on";
/// Command: engage the servos.
const CMD_SERVOS_ON: &str = "sim/autopilot/servos_on";
/// Command: disengage the servos (autopilot disconnect).
const CMD_SERVOS_OFF: &str = "sim/autopilot/servos_off";
/// Command: engage heading-select (HDG) mode.
const CMD_HEADING_MODE: &str = "sim/autopilot/heading";
/// Command: engage altitude select/hold (ALT) mode.
const CMD_ALT_HOLD: &str = "sim/autopilot/altitude_hold";
/// Command: engage vertical-speed (VS) mode.
const CMD_VS_MODE: &str = "sim/autopilot/vertical_speed";
/// Descents steeper than this (fpm) are confirmed — anything shallower, and all
/// climbs, execute immediately. A 172 descends ~500–1000 fpm routinely.
const VS_CONFIRM_DESCENT_FPM: f64 = -1500.0;

/// Read the servo status as a bool ("is the autopilot actually flying").
async fn servos_engaged(ctx: &ToolCtx<'_>) -> Result<bool> {
    Ok(ctx.xplane.get_float(SERVOS_ON).await? > 0.5)
}

pub struct SetHeading;

#[derive(Deserialize)]
struct Args {
    /// Target magnetic heading in degrees.
    heading: f64,
}

#[async_trait]
impl Tool for SetHeading {
    fn name(&self) -> &str {
        "set_heading"
    }

    fn description(&self) -> &str {
        "Command the autopilot to turn to and hold a magnetic heading. Engages \
         the autopilot and heading mode, then flies the given heading in degrees. \
         Use for 'autopilot heading 270', 'fly heading 090', 'turn to heading 180'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "heading": {
                    "type": "integer",
                    "minimum": 0,
                    "maximum": 360,
                    "description": "Magnetic heading in degrees (0-359)."
                }
            },
            "required": ["heading"]
        })
    }

    // No confirmation: a misheard heading just turns the wrong way and is
    // trivially corrected — unlike altitude/VS (terrain) or engine cutoff.

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: Args = serde_json::from_value(args.clone())?;
        // Normalize into 0..360 (so "370" → 10, "-10" → 350).
        let heading = args.heading.rem_euclid(360.0);

        // Engage the autopilot and mode FIRST, then write the bug LAST —
        // engaging a mode can snap the dial to a current value, so the target
        // must be the final write or it gets clobbered.
        ctx.xplane.run_command(CMD_SERVOS_ON).await?;
        ctx.xplane.run_command(CMD_HEADING_MODE).await?;
        ctx.xplane.set_float(HEADING_BUG, heading).await?;

        // Confirm the actual state, not the intent.
        let actual = ctx.xplane.get_float(HEADING_BUG).await?;
        if servos_engaged(ctx).await? {
            Ok(format!("Roger, heading {:03.0}, autopilot engaged.", actual))
        } else {
            Ok(format!(
                "Heading bug set to {:03.0}, but the autopilot didn't engage — check the AP master.",
                actual
            ))
        }
    }
}

pub struct SetAltitude;

#[derive(Deserialize)]
struct AltArgs {
    /// Target altitude in feet.
    altitude: f64,
}

#[async_trait]
impl Tool for SetAltitude {
    fn name(&self) -> &str {
        "set_altitude"
    }

    fn description(&self) -> &str {
        "Set the autopilot target altitude in feet and engage altitude hold. Use \
         for 'autopilot altitude 5500', 'climb and maintain 7000', 'set altitude \
         3000'. Pair with set_vertical_speed to start a climb or descent."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "altitude": {
                    "type": "integer",
                    "minimum": 0,
                    "maximum": 60000,
                    "description": "Target altitude in feet MSL."
                }
            },
            "required": ["altitude"]
        })
    }

    fn confirm_prompt(&self, args: &Value) -> Option<String> {
        let a: AltArgs = serde_json::from_value(args.clone()).ok()?;
        Some(format!("altitude {:.0} feet", a.altitude.max(0.0)))
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: AltArgs = serde_json::from_value(args.clone())?;
        let altitude = args.altitude.max(0.0);

        // Engage first, then set the target LAST (see set_heading note).
        ctx.xplane.run_command(CMD_SERVOS_ON).await?;
        ctx.xplane.run_command(CMD_ALT_HOLD).await?;
        ctx.xplane.set_float(ALT_DIAL, altitude).await?;

        let actual = ctx.xplane.get_float(ALT_DIAL).await?;
        if servos_engaged(ctx).await? {
            Ok(format!("Roger, altitude {:.0} feet, autopilot engaged.", actual))
        } else {
            Ok(format!(
                "Altitude set to {:.0} feet, but the autopilot didn't engage — check the AP master.",
                actual
            ))
        }
    }
}

pub struct SetVerticalSpeed;

#[derive(Deserialize)]
struct VsArgs {
    /// Vertical speed in feet per minute; positive climbs, negative descends.
    fpm: f64,
}

#[async_trait]
impl Tool for SetVerticalSpeed {
    fn name(&self) -> &str {
        "set_vertical_speed"
    }

    fn description(&self) -> &str {
        "Set the autopilot vertical speed in feet per minute and engage vertical \
         speed mode. Positive climbs, negative descends. Use for 'vertical speed \
         500 up', 'descend at 700', 'VS 1000', 'vertical speed down 500'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "fpm": {
                    "type": "integer",
                    "minimum": -2000,
                    "maximum": 2000,
                    "description": "Vertical speed, feet per minute. Positive = climb, negative = descend."
                }
            },
            "required": ["fpm"]
        })
    }

    fn confirm_prompt(&self, args: &Value) -> Option<String> {
        let a: VsArgs = serde_json::from_value(args.clone()).ok()?;
        let fpm = a.fpm.clamp(-2000.0, 2000.0);
        // Only an extreme descent is gated — climbs can't hit terrain, and a
        // normal descent (up to ~1500 fpm) is routine for a 172.
        if fpm <= VS_CONFIRM_DESCENT_FPM {
            Some(describe_vs(fpm))
        } else {
            None
        }
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: VsArgs = serde_json::from_value(args.clone())?;
        // Clamp to a sane light-GA envelope so a misheard magnitude can't
        // command an extreme dive/climb.
        let fpm = args.fpm.clamp(-2000.0, 2000.0);

        // Engage VS mode FIRST — `sim/autopilot/vertical_speed` is "at current
        // VSI", so it snaps the dial to the live climb rate. Writing our target
        // LAST is what makes the aircraft fly the value we confirmed.
        ctx.xplane.run_command(CMD_SERVOS_ON).await?;
        ctx.xplane.run_command(CMD_VS_MODE).await?;
        ctx.xplane.set_float(VS_DIAL, fpm).await?;

        let actual = ctx.xplane.get_float(VS_DIAL).await?;
        let phrase = describe_vs(actual);
        if servos_engaged(ctx).await? {
            Ok(format!("Roger, {phrase}, autopilot engaged."))
        } else {
            Ok(format!("{phrase} set, but the autopilot didn't engage — check the AP master."))
        }
    }
}

pub struct AutopilotOff;

#[async_trait]
impl Tool for AutopilotOff {
    fn name(&self) -> &str {
        "autopilot_off"
    }

    fn description(&self) -> &str {
        "Disengage the autopilot — disconnect the servos so the pilot hand-flies. \
         Use for 'autopilot off', 'disconnect the autopilot', 'disengage AP', \
         'kill the autopilot', 'I have the airplane'."
    }

    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }

    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        if !servos_engaged(ctx).await? {
            return Ok("Autopilot's already off.".to_string());
        }
        ctx.xplane.run_command(CMD_SERVOS_OFF).await?;
        if servos_engaged(ctx).await? {
            Ok("The autopilot didn't disengage — disconnect it manually.".to_string())
        } else {
            Ok("Roger, autopilot off. You have the airplane.".to_string())
        }
    }
}

/// Turn a signed fpm value into a spoken phrase.
fn describe_vs(fpm: f64) -> String {
    let mag = fpm.abs();
    if mag < 1.0 {
        "vertical speed level".to_string()
    } else if fpm > 0.0 {
        format!("climb {mag:.0} feet per minute")
    } else {
        format!("descend {mag:.0} feet per minute")
    }
}
