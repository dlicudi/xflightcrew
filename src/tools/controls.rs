//! Ground controls: `set_parking_brake`, crew-held regular brakes
//! (`hold_brakes`), and pitch trim (`set_pitch_trim`).

use std::sync::atomic::{AtomicBool, Ordering};

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx, BRAKE_HOLD_SECS, PARK_BRAKE};
#[cfg(not(test))]
use crate::xplane::XPlaneClient;

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
/// The parking-brake commands — preferred over a raw dataref write: custom
/// airframes intercept commands, while a written ratio can be ignored or
/// overwritten by their systems scripts (the MD-82 pitot-heat lesson).
/// Deterministic set/release, not `brakes_toggle_max`: a toggle flips the
/// wrong way off a stale state read; these are idempotent (pilot's find,
/// 2026-07-11).
const CMD_PARK_SET: &str = "sim/flight_controls/park_brake_set";
const CMD_PARK_RELEASE: &str = "sim/flight_controls/park_brake_release";

pub struct SetParkingBrake {
    /// The valve value meaning SET on this airframe; `None` = leave the valve alone.
    pub valve_set: Option<f64>,
}

/// Drive the parking brake to `on`, valve-aware — THE one way any tool moves
/// it. Valve first (per the airframe's SET sense), then the brake, command
/// before raw write, verified by readback. The taxi hold's auto-release once
/// skipped the valve and told an SR22 pilot "parking brake off" while the
/// valve held the wheels (2026-07-11). Returns whether the brake now agrees.
pub(crate) async fn drive_park_brake(
    xp: &dyn crate::xplane::XPlane,
    on: bool,
    valve_set: Option<f64>,
) -> Result<bool> {
    if let Some(want) = valve_set.map(|set| if on { set } else { 1.0 - set }) {
        if let Ok(now) = xp.get_float(PARK_BRAKE_VALVE).await {
            if (now - want).abs() > 0.5 {
                xp.run_command(CMD_VALVE_TOGGLE).await?;
            }
        }
    }
    xp.run_command(if on { CMD_PARK_SET } else { CMD_PARK_RELEASE }).await?;
    // Commands apply on the next sim frame — give it a beat.
    tokio::time::sleep(std::time::Duration::from_millis(150)).await;
    let mut set = xp.get_float(PARK_BRAKE).await? > 0.5;
    if set != on {
        xp.set_float(PARK_BRAKE, if on { 1.0 } else { 0.0 }).await?;
        set = xp.get_float(PARK_BRAKE).await? > 0.5;
    }
    Ok(set == on)
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
         brake on', 'release the parking brake', 'parking brake off'. For 'brakes \
         on' / 'hold the brakes' / 'brakes off' without the word parking, use \
         hold_brakes instead."
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

        if !drive_park_brake(ctx.xplane, args.on, self.valve_set).await? {
            return Ok(format!(
                "Tried, but the parking brake still reads {}. Check the handle.",
                if args.on { "released" } else { "set" }
            ));
        }
        // The handle holds us now: a crew brake hold still renewing under it
        // would just be a boot nobody needs, and one more thing to forget.
        let crew_released = args.on && release_held_brakes();
        Ok(format!(
            "Roger, parking brake {}.{}{}",
            if args.on { "set" } else { "released" },
            if args.on && taxi_hold { " Throttle's coming to idle — taxi hold released." } else { "" },
            if crew_released { " I'm off the pedals." } else { "" }
        ))
    }
}

// ---- hold_brakes: crew-held regular brakes ("brakes on") ------------------

/// Toe-brake pedal readback (left side). The held command drives the ratios
/// through the input layer, so this confirms the ACTION, not the intent.
const LEFT_BRAKE: &str = "sim/cockpit2/controls/left_brake_ratio";
/// Thrust lever, for the two power guards: engaging refuses with power up (a
/// misheard "brakes on" mid-takeoff-roll must never latch the wheels), and
/// the holder warns once if power comes up while the boot is on.
const THROTTLE: &str = "sim/cockpit2/engine/actuators/throttle_ratio_all";
/// Engage refuses above this throttle. Comfortably over the taxi governor's
/// 0.35 power cap, comfortably under any takeoff or run-up setting.
const HOLD_MAX_THROTTLE: f64 = 0.4;

/// True while the crew-held brake loop runs.
static BRAKES_HELD: AtomicBool = AtomicBool::new(false);

/// For the dashboard's "managing" panel.
pub fn brakes_held() -> bool {
    BRAKES_HELD.load(Ordering::Relaxed)
}

/// Drop a crew brake hold, if one is on; the held command then frees itself
/// by expiry. Returns whether there was one. Called when another intent
/// supersedes the hold: a commanded taxi (`hold_speed`), or the parking
/// brake taking over.
pub(crate) fn release_held_brakes() -> bool {
    BRAKES_HELD.swap(false, Ordering::Relaxed)
}

/// The holder loop: renew the held-brake command until released. Renewal
/// well inside `BRAKE_HOLD_SECS` keeps the hold continuous; ANY exit,
/// including a crash, frees the brakes by expiry. Airborne auto-releases
/// (a held brake is a locked wheel at the next touchdown), and a failed
/// squat read is a hiccup, not "airborne": the boot stays on. Power coming
/// up under the hold gets one spoken warning, never an auto-release:
/// holding brakes at power is a real technique (the run-up, a short-field
/// takeoff), so the pilot decides.
#[cfg(not(test))]
async fn brake_holder(xp: XPlaneClient, voice: Option<crate::voice::Voice>) {
    tracing::info!("brake hold started");
    let mut power_warned = false;
    while BRAKES_HELD.load(Ordering::Relaxed) {
        if xp
            .get_float(crate::datarefs::ON_GROUND_ANY)
            .await
            .is_ok_and(|v| v < 0.5)
        {
            BRAKES_HELD.store(false, Ordering::Relaxed);
            if let Some(v) = &voice {
                v.say(crate::voice::Priority::Advisory, "Airborne, brakes released.");
            }
            break;
        }
        if let Err(e) = xp.run_command_for(super::BRAKES_REGULAR, BRAKE_HOLD_SECS).await {
            tracing::warn!("brake hold: command failed: {e:#}");
        }
        if !power_warned && xp.get_float(THROTTLE).await.is_ok_and(|t| t > 0.7) {
            power_warned = true;
            if let Some(v) = &voice {
                v.say(
                    crate::voice::Priority::Caution,
                    "Power's up and I'm still holding the brakes. Say brakes off when you want them.",
                );
            }
        }
        tokio::time::sleep(std::time::Duration::from_millis(400)).await;
    }
    tracing::info!("brake hold stopped");
    // Nothing to clean on the way out: the held command expires by itself.
}

pub struct HoldBrakes;

#[derive(Deserialize)]
struct HoldBrakesArgs {
    /// true = hold the brakes, false = release them.
    on: bool,
}

#[async_trait]
impl Tool for HoldBrakes {
    fn name(&self) -> &str {
        "hold_brakes"
    }

    fn description(&self) -> &str {
        "Hold or release the REGULAR wheel brakes: the copilot stands on the \
         pedals and stays on them until told otherwise. For the run-up, \
         holding at the hold-short line, or keeping the aircraft still without \
         the parking brake. Use for 'brakes on', 'hold the brakes', 'brakes \
         off', 'release the brakes'. NOT the latching parking brake, that's \
         set_parking_brake."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "on": {"type": "boolean", "description": "true = hold the brakes, false = release"}
            },
            "required": ["on"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: HoldBrakesArgs = serde_json::from_value(args.clone())?;
        if !a.on {
            if !release_held_brakes() {
                let park = ctx.xplane.get_float(PARK_BRAKE).await.unwrap_or(0.0) > 0.5;
                return Ok(if park {
                    "I wasn't holding the brakes, the parking brake is set. \
                     Say parking brake off for that."
                        .to_string()
                } else {
                    "I wasn't holding the brakes.".to_string()
                });
            }
            // The held command frees itself by expiry; wait it out so the
            // readback reports the pedals, not the promise.
            tokio::time::sleep(std::time::Duration::from_millis(
                (BRAKE_HOLD_SECS * 1000.0) as u64 + 300,
            ))
            .await;
            let park = ctx.xplane.get_float(PARK_BRAKE).await.unwrap_or(0.0) > 0.5;
            let still = ctx.xplane.get_float(LEFT_BRAKE).await.is_ok_and(|r| r > 0.3);
            return Ok(if park {
                "Brakes released. Parking brake's still set, mind.".to_string()
            } else if still {
                "Brakes off my side, but the pedals still read pressure. \
                 If that's not your feet, check them."
                    .to_string()
            } else {
                "Brakes released.".to_string()
            });
        }

        // Gates, in order of what a mishear could cost. Airborne: nothing to
        // hold, and a held brake is a blown tyre at the next touchdown. A
        // failed read assumes ground — if the API is down the command below
        // fails loudly anyway.
        let on_ground =
            ctx.xplane.get_float(crate::datarefs::ON_GROUND_ANY).await.unwrap_or(1.0) > 0.5;
        if !on_ground {
            return Ok("We're airborne, nothing to hold the brakes for up here.".to_string());
        }
        // THE mishear gate: power up means a takeoff roll (or a run-up
        // already running). Latched brakes at takeoff power is the one
        // dangerous thing a misheard 'brakes on' could buy, so it's refused,
        // never guessed.
        if ctx.xplane.get_float(THROTTLE).await.unwrap_or(0.0) > HOLD_MAX_THROTTLE {
            return Ok(
                "Negative, power's up, I'm not grabbing the brakes. Close the \
                 throttle first."
                    .to_string(),
            );
        }
        if ctx.xplane.get_float(PARK_BRAKE).await.unwrap_or(0.0) > 0.5 {
            return Ok("Parking brake's already set, we're staying put.".to_string());
        }
        if BRAKES_HELD.load(Ordering::Relaxed) {
            return Ok("I'm already holding the brakes.".to_string());
        }
        // Brakes commanded while the copilot is taxiing = the pilot is
        // stopping: the governor stands down and the throttle closes before
        // the boot goes on, never brake against thrust. The short sleep lets
        // an in-flight glide land before the close sticks.
        let taxi_hold = super::speedhold::taxi_hold_active();
        if taxi_hold {
            super::speedhold::ACTIVE.store(false, Ordering::Relaxed);
            tokio::time::sleep(std::time::Duration::from_millis(350)).await;
            let _ = ctx.xplane.set_float(THROTTLE, 0.0).await;
        }
        if !BRAKES_HELD.swap(true, Ordering::Relaxed) {
            // The holder drives the REAL sim, so it's compiled out under
            // test: `cargo test` mid-flight must never pulse the live brakes.
            // Its own thread with its own runtime, like the speed governor:
            // the main runtime blocks on the mic between commands.
            #[cfg(not(test))]
            {
                let base = ctx.config.xplane_base_url.clone();
                let voice = ctx.voice.cloned();
                std::thread::spawn(move || {
                    let Ok(rt) =
                        tokio::runtime::Builder::new_current_thread().enable_all().build()
                    else {
                        BRAKES_HELD.store(false, Ordering::Relaxed);
                        return;
                    };
                    rt.block_on(brake_holder(XPlaneClient::new(base), voice));
                });
            }
        }
        // Readback off the pedals, not the intent. A beat for the first
        // command to land in the sim.
        tokio::time::sleep(std::time::Duration::from_millis(250)).await;
        let pressed = ctx.xplane.get_float(LEFT_BRAKE).await.map(|r| r > 0.2);
        Ok(match pressed {
            Ok(false) => "Tried, but the pedals don't read pressed. Check the brakes.".to_string(),
            _ => format!(
                "Brakes held.{} Say brakes off when you want them.",
                if taxi_hold { " Taxi hold released, throttle's closed." } else { "" }
            ),
        })
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tools::ToolCtx;
    use crate::xplane::FakeXPlane;

    fn ctx<'a>(fake: &'a FakeXPlane, cfg: &'a crate::config::Config) -> ToolCtx<'a> {
        ToolCtx { xplane: fake, config: cfg, voice: None, status: None, reminders: None }
    }

    // The refusal gates all sit BEFORE the already-held check, so these
    // assertions hold no matter what the round-trip test below is doing to
    // the shared BRAKES_HELD flag in parallel.
    #[tokio::test]
    async fn brakes_on_refuses_airborne() {
        let cfg = crate::config::Config::default();
        let fake = FakeXPlane::with_reads(&[(crate::datarefs::ON_GROUND_ANY, 0.0)]);
        let out = HoldBrakes.execute(&json!({"on": true}), &ctx(&fake, &cfg)).await.unwrap();
        assert!(out.to_lowercase().contains("airborne"), "{out}");
    }

    #[tokio::test]
    async fn brakes_on_refuses_with_power_up() {
        // THE mishear gate: a "brakes on" landing mid-takeoff-roll must be
        // refused, never latched.
        let cfg = crate::config::Config::default();
        let fake = FakeXPlane::with_reads(&[
            (crate::datarefs::ON_GROUND_ANY, 1.0),
            (THROTTLE, 0.8),
        ]);
        let out = HoldBrakes.execute(&json!({"on": true}), &ctx(&fake, &cfg)).await.unwrap();
        assert!(out.to_lowercase().contains("power"), "{out}");
    }

    #[tokio::test]
    async fn brakes_on_defers_to_a_set_parking_brake() {
        let cfg = crate::config::Config::default();
        let fake = FakeXPlane::with_reads(&[
            (crate::datarefs::ON_GROUND_ANY, 1.0),
            (THROTTLE, 0.0),
            (PARK_BRAKE, 1.0),
        ]);
        let out = HoldBrakes.execute(&json!({"on": true}), &ctx(&fake, &cfg)).await.unwrap();
        assert!(out.contains("Parking brake"), "{out}");
    }

    // The one test that owns the BRAKES_HELD flag end to end: on, held,
    // off, released, off again = honest "wasn't holding". (Under test the
    // holder thread is compiled out, so only the flag and readbacks run.)
    #[tokio::test]
    async fn brakes_on_then_off_round_trip() {
        let cfg = crate::config::Config::default();
        let fake = FakeXPlane::with_reads(&[
            (crate::datarefs::ON_GROUND_ANY, 1.0),
            (THROTTLE, 0.0),
            (PARK_BRAKE, 0.0),
            (LEFT_BRAKE, 0.6), // pedals read pressed for the readback
        ]);
        let out = HoldBrakes.execute(&json!({"on": true}), &ctx(&fake, &cfg)).await.unwrap();
        assert!(out.contains("Brakes held"), "{out}");
        assert!(brakes_held());

        // Second "brakes on" while holding: idempotent, no double-spawn.
        let out = HoldBrakes.execute(&json!({"on": true}), &ctx(&fake, &cfg)).await.unwrap();
        assert!(out.contains("already holding"), "{out}");

        let fake = FakeXPlane::with_reads(&[
            (crate::datarefs::ON_GROUND_ANY, 1.0),
            (PARK_BRAKE, 0.0),
            (LEFT_BRAKE, 0.0), // pedals free once the held command expires
        ]);
        let out = HoldBrakes.execute(&json!({"on": false}), &ctx(&fake, &cfg)).await.unwrap();
        assert!(out.contains("released"), "{out}");
        assert!(!brakes_held());

        let out = HoldBrakes.execute(&json!({"on": false}), &ctx(&fake, &cfg)).await.unwrap();
        assert!(out.contains("wasn't holding"), "{out}");
    }
}
