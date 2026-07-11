//! Ground controls: `set_parking_brake`, and pitch trim (`set_pitch_trim`).

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx, PARK_BRAKE};

/// Elevator trim, -1.0 (full nose down) .. +1.0 (full nose up), 0 = neutral.
const ELEV_TRIM: &str = "sim/cockpit2/controls/elevator_trim";
/// How much one "trim nose up/down" nudge moves the trim wheel.
const TRIM_STEP: f64 = 0.10;
/// Takeoff trim setting for the 172 — roughly neutral (the takeoff band).
const TRIM_TAKEOFF: f64 = 0.0;

/// Park-brake VALVE (X-Plane 12). Aircraft that model it animate their handle
/// from this, and its SENSE is per-aircraft: on the C90B 0 = set (and the sim
/// pins `parking_brake_ratio` back to 1 while set); on the SR22 it's the
/// opposite (1 = set, no ratio pinning). So the tool only drives the valve when
/// the profile says which value means SET (`park_brake_valve_set`).
const PARK_BRAKE_VALVE: &str = "sim/cockpit2/controls/park_brake_valve";
const CMD_VALVE_TOGGLE: &str = "sim/flight_controls/park_brake_valve_toggle";
/// The parking-brake toggle command — preferred over a raw dataref write:
/// custom airframes intercept commands, while a written ratio can be ignored or
/// overwritten by their systems scripts (the MD-82 pitot-heat lesson).
pub(crate) const CMD_PARK_TOGGLE: &str = "sim/flight_controls/brakes_toggle_max";

pub struct SetParkingBrake {
    /// The valve value meaning SET on this airframe; `None` = leave the valve alone.
    pub valve_set: Option<f64>,
}

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
        "Set or release the PARKING brake (the latching park-brake valve + handle) \
         — NOT momentary wheel braking. Use only for 'parking brake set', 'park \
         brake on', 'release the parking brake', 'parking brake off'. Do not use \
         for 'hold the brakes' / 'brake' while rolling (that's regular braking, \
         which has no tool)."
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
        // Setting the brake while the copilot is taxiing = the pilot is
        // stopping: the governor sees the brake within a tick and closes the
        // throttle itself — read the flag BEFORE the write so the readback
        // can say so (the governor may already be standing down after it).
        let taxi_hold = args.on && super::speedhold::taxi_hold_active();
        // Where the profile declares the valve's SET value, the valve must agree
        // with the request — on airframes that pin the ratio from it (C90B) a
        // release can't stick otherwise, and the handle animates from it either
        // way. The desired value depends on the per-aircraft sense.
        let desired_valve = self.valve_set.map(|set| if args.on { set } else { 1.0 - set });
        let valve_now = match desired_valve {
            Some(_) => ctx.xplane.get_float(PARK_BRAKE_VALVE).await.ok(),
            None => None,
        };

        // If everything's already where you asked, say so instead of a "Roger" —
        // but only when the valve (if driven) agrees too.
        let already = ctx.xplane.get_float(PARK_BRAKE).await? > 0.5;
        let valve_aligned = match (desired_valve, valve_now) {
            (Some(want), Some(now)) => (now - want).abs() < 0.5,
            _ => true,
        };
        if already == args.on && valve_aligned {
            return Ok(format!(
                "Parking brake is already {}.",
                if already { "set" } else { "off" }
            ));
        }

        // Valve first (to the airframe's SET/RELEASED sense), then the brake —
        // command-first, verified by readback, with the raw dataref write kept
        // only as a fallback for models that ignore the toggle.
        if let (Some(want), Some(now)) = (desired_valve, valve_now) {
            if (now - want).abs() > 0.5 {
                ctx.xplane.run_command(CMD_VALVE_TOGGLE).await?;
            }
        }
        if already != args.on {
            ctx.xplane.run_command(CMD_PARK_TOGGLE).await?;
        }
        // The command applies on the next sim frame — give it a beat.
        tokio::time::sleep(std::time::Duration::from_millis(150)).await;
        let mut set = ctx.xplane.get_float(PARK_BRAKE).await? > 0.5;
        if set != args.on {
            ctx.xplane.set_float(PARK_BRAKE, if args.on { 1.0 } else { 0.0 }).await?;
            set = ctx.xplane.get_float(PARK_BRAKE).await? > 0.5;
        }
        if set != args.on {
            return Ok(format!(
                "Tried, but the parking brake still reads {}. Check the handle.",
                if set { "set" } else { "released" }
            ));
        }
        Ok(format!(
            "Roger, parking brake {}.{}",
            if set { "set" } else { "released" },
            if set && taxi_hold { " Throttle's coming to idle — taxi hold released." } else { "" }
        ))
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
         trim', 'centre the trim', 'set takeoff trim', 'trim up', 'trim down', \
         'roll in some nose up trim', 'center the trim', 'trim for takeoff'."
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

/// Speedbrake/spoiler lever: 1 = fully extended, 0 = stowed, -0.5 = armed (auto
/// deploy on touchdown). The MD-82 uses this standard, writable dataref.
const SPEEDBRAKE: &str = "sim/cockpit2/controls/speedbrake_ratio";
/// Gear handle (1 = down). Read by the speedbrake-on-final guard.
const GEAR_HANDLE_DOWN: &str = "sim/cockpit2/controls/gear_handle_down";

pub struct SetSpeedbrake;

#[derive(Deserialize)]
struct SpeedbrakeArgs {
    /// "extend", "retract", or "arm".
    action: String,
}

#[async_trait]
impl Tool for SetSpeedbrake {
    fn name(&self) -> &str {
        "speedbrake"
    }

    fn description(&self) -> &str {
        "Extend, retract, or arm the speedbrakes (spoilers). Use for 'speedbrakes \
         out', 'speedbrakes extend', 'deploy the speedbrakes', 'speedbrakes in', \
         'speedbrakes retract', 'stow the speedbrakes', 'arm the speedbrakes', \
         'speed brakes up', 'spoilers up'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "action": {"type": "string", "enum": ["extend", "retract", "arm"]}
            },
            "required": ["action"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: SpeedbrakeArgs = serde_json::from_value(args.clone())?;
        let (target, word) = match a.action.to_ascii_lowercase().as_str() {
            "extend" | "deploy" | "out" | "up" => (1.0, "extended"),
            "retract" | "stow" | "in" | "down" => (0.0, "retracted"),
            "arm" => (-0.5, "armed"),
            other => return Ok(format!("I can extend, retract, or arm the speedbrakes, not '{other}'.")),
        };
        // Safety gate: don't deploy the speedbrakes on short final. With the gear
        // down and low above the ground, full speedbrake dumps lift right where a
        // go-around or flare needs it. (Arming for landing is fine — that's the
        // ground-spoiler auto-deploy, not lift-dumping in the air.)
        if target > 0.5 {
            let agl_ft = ctx.xplane.get_float(crate::datarefs::AGL_METERS).await.unwrap_or(99999.0) * 3.28084;
            let gear_down = ctx.xplane.get_float(GEAR_HANDLE_DOWN).await.unwrap_or(0.0) > 0.5;
            let on_ground = ctx.xplane.get_float(crate::datarefs::ON_GROUND_ANY).await.unwrap_or(1.0) > 0.5;
            if !on_ground && gear_down && agl_ft < 1000.0 {
                return Ok(format!(
                    "Negative, we're on final at {agl_ft:.0} feet with the gear down. I won't deploy the speedbrakes here."
                ));
            }
        }
        ctx.xplane.set_float(SPEEDBRAKE, target).await?;
        let actual = ctx.xplane.get_float(SPEEDBRAKE).await.unwrap_or(target);
        let state = if actual > 0.05 {
            "extended"
        } else if actual < -0.05 {
            "armed"
        } else {
            "retracted"
        };
        Ok(format!("Speedbrakes {state}.", state = if state == word { word } else { state }))
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
