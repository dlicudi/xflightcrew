//! Phase-config macros: one spoken command that fires a curated *sequence* of
//! actuator writes, the way [`engine::StartEngine`](super::engine::StartEngine)
//! bundles the start procedure. This is "checklist-like" convenience kept purely
//! in the actor model — no checklist files, no verify/challenge, no per-item
//! interaction. A macro composes the existing single-action tools (so there's no
//! duplicated dataref logic) and only sets the **deterministic** items of a
//! phase. Judgment and visual items (flaps "as required", throttle, brakes,
//! controls-free) stay with the pilot — the macro flips switches, not airmanship.

use anyhow::{bail, Result};
use async_trait::async_trait;
use serde_json::{json, Value};

use super::controls::{SetParkingBrake, SetPitchTrim};
use super::electrical::SetElectrical;
use super::engine::{SetCarbHeat, SetFuelSelector, SetMixture, StopEngine};
use super::flaps::SetFlaps;
use super::lights::SetLights;
use super::radio::SetTransponderMode;
use super::{Tool, ToolCtx};
use crate::aircraft::{FlapProfile, LightProfile};

/// `configure_for_takeoff` — set the aircraft up for departure in one call.
pub struct ConfigureForTakeoff {
    lights: Vec<LightProfile>,
}

impl ConfigureForTakeoff {
    pub fn new(lights: Vec<LightProfile>) -> Self {
        Self { lights }
    }
}

#[async_trait]
impl Tool for ConfigureForTakeoff {
    fn name(&self) -> &str {
        "configure_for_takeoff"
    }

    fn description(&self) -> &str {
        "Configure the WHOLE aircraft for takeoff at once. ONLY for a single \
         command that asks to set up everything for departure: 'configure for \
         takeoff', 'set up for takeoff', 'takeoff config', 'before takeoff setup', \
         'get ready for takeoff', 'prep for departure'. It then sets mixture rich, \
         fuel both, carb heat cold, trim takeoff, transponder altitude, and \
         strobe + landing lights together. Do NOT use this when a single system \
         is named — 'mixture rich', 'trim for takeoff', 'transponder altitude', \
         'strobes on' each have their own tool; pick this only when no individual \
         system is mentioned. Does not touch the throttle, brakes, or flaps."
    }

    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }

    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let lights = SetLights::new(self.lights.clone());

        // Fire each config item, tolerating an individual failure (e.g. a system
        // this airframe lacks) rather than aborting half-configured. Collect the
        // ones that didn't take so the readback is honest about it.
        let mut failed: Vec<&str> = Vec::new();
        let mut step = |name: &'static str, r: Result<String>| {
            if r.is_err() {
                failed.push(name);
            }
        };

        step("mixture", SetMixture.execute(&json!({ "percent": 100 }), ctx).await);
        step("fuel selector", SetFuelSelector.execute(&json!({ "position": "both" }), ctx).await);
        step("carb heat", SetCarbHeat.execute(&json!({ "on": false }), ctx).await);
        step("trim", SetPitchTrim.execute(&json!({ "action": "takeoff" }), ctx).await);
        step("transponder", SetTransponderMode.execute(&json!({ "mode": "altitude" }), ctx).await);
        step("strobes", lights.execute(&json!({ "light": "strobe", "on": true }), ctx).await);
        step("landing light", lights.execute(&json!({ "light": "landing", "on": true }), ctx).await);

        // Everything failed → the aircraft is unreachable; surface a real error.
        if failed.len() == 7 {
            bail!("couldn't set the takeoff configuration — is X-Plane reachable?");
        }
        if failed.is_empty() {
            Ok("Roger — takeoff config: mixture rich, fuel both, carb heat cold, trim takeoff, \
                transponder altitude, strobes and landing light on. Throttle, brakes, and flaps are yours."
                .to_string())
        } else {
            Ok(format!(
                "Takeoff config set — but check {}: I couldn't confirm {}.",
                failed.join(", "),
                if failed.len() == 1 { "it" } else { "them" }
            ))
        }
    }
}

/// `configure_for_landing` — set the aircraft up for the approach in one call.
pub struct ConfigureForLanding {
    lights: Vec<LightProfile>,
}

impl ConfigureForLanding {
    pub fn new(lights: Vec<LightProfile>) -> Self {
        Self { lights }
    }
}

#[async_trait]
impl Tool for ConfigureForLanding {
    fn name(&self) -> &str {
        "configure_for_landing"
    }

    fn description(&self) -> &str {
        "Configure the WHOLE aircraft for landing at once. ONLY for a single \
         command that asks to set up everything for the approach: 'configure for \
         landing', 'set up for landing', 'landing config', 'before landing setup', \
         'get ready for landing', 'prep for landing'. It then sets mixture rich, \
         fuel both, and landing + taxi lights on. Do NOT use this when a single \
         system is named — pick this only when no individual system is mentioned. \
         Does NOT disconnect the autopilot (say 'autopilot off' when you're ready \
         to hand-fly) and does not touch the flaps."
    }

    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }

    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let lights = SetLights::new(self.lights.clone());
        let mut failed: Vec<&str> = Vec::new();
        let mut step = |name: &'static str, r: Result<String>| {
            if r.is_err() {
                failed.push(name);
            }
        };

        step("mixture", SetMixture.execute(&json!({ "percent": 100 }), ctx).await);
        step("fuel selector", SetFuelSelector.execute(&json!({ "position": "both" }), ctx).await);
        step("landing light", lights.execute(&json!({ "light": "landing", "on": true }), ctx).await);
        step("taxi light", lights.execute(&json!({ "light": "taxi", "on": true }), ctx).await);

        if failed.len() == 4 {
            bail!("couldn't set the landing configuration — is X-Plane reachable?");
        }
        if failed.is_empty() {
            Ok("Roger — landing config: mixture rich, fuel both, landing and taxi lights on. \
                Autopilot and flaps are yours."
                .to_string())
        } else {
            Ok(format!("Landing config set — but check {}.", failed.join(", ")))
        }
    }
}

/// `after_landing` — clean up once clear of the runway, in one call.
pub struct AfterLanding {
    lights: Vec<LightProfile>,
    flaps: Option<FlapProfile>,
}

impl AfterLanding {
    pub fn new(lights: Vec<LightProfile>, flaps: Option<FlapProfile>) -> Self {
        Self { lights, flaps }
    }
}

#[async_trait]
impl Tool for AfterLanding {
    fn name(&self) -> &str {
        "after_landing"
    }

    fn description(&self) -> &str {
        "Run the after-landing cleanup in one step, once clear of the runway: \
         flaps up, landing + strobe + nav lights off, transponder to standby. Use \
         for 'after landing', 'after landing flow', 'after landing checklist', \
         'clean up', 'clean up after landing', 'clear of the runway'. The taxi \
         light stays on. Do NOT use this when a single system is named."
    }

    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }

    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let lights = SetLights::new(self.lights.clone());
        let mut failed: Vec<&str> = Vec::new();
        let mut step = |name: &'static str, r: Result<String>| {
            if r.is_err() {
                failed.push(name);
            }
        };

        if let Some(flaps) = &self.flaps {
            step("flaps", SetFlaps::new(flaps.clone()).execute(&json!({ "notch": "up" }), ctx).await);
        }
        step("landing light", lights.execute(&json!({ "light": "landing", "on": false }), ctx).await);
        step("strobes", lights.execute(&json!({ "light": "strobe", "on": false }), ctx).await);
        step("nav lights", lights.execute(&json!({ "light": "nav", "on": false }), ctx).await);
        step("transponder", SetTransponderMode.execute(&json!({ "mode": "standby" }), ctx).await);

        if failed.is_empty() {
            Ok("Roger — after landing: flaps up, landing, strobe and nav lights off, \
                transponder standby. Taxi light's still on."
                .to_string())
        } else {
            Ok(format!("After-landing cleanup done — but check {}.", failed.join(", ")))
        }
    }
}

/// `secure_aircraft` — the full shutdown-and-secure flow in one call. Extends
/// what `stop_engine` does (engine off) with securing the electrics and lights.
pub struct SecureAircraft {
    lights: Vec<LightProfile>,
}

impl SecureAircraft {
    pub fn new(lights: Vec<LightProfile>) -> Self {
        Self { lights }
    }
}

#[async_trait]
impl Tool for SecureAircraft {
    fn name(&self) -> &str {
        "secure_aircraft"
    }

    fn description(&self) -> &str {
        "Secure the aircraft after the flight in one step: parking brake on, \
         engine off (mixture cutoff, magnetos off), all exterior lights off, \
         transponder off, avionics off, alternator off, then the master off last. \
         Use for 'secure the aircraft', 'secure', 'shut everything down', 'park \
         and secure', 'full shutdown', 'secure for shutdown'. This is the FULL \
         secure — for just the engine, say 'shut down the engine' (that's a \
         different tool). Do NOT use this when a single system is named."
    }

    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }

    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let lights = SetLights::new(self.lights.clone());
        let mut failed: Vec<&str> = Vec::new();
        let mut step = |name: &'static str, r: Result<String>| {
            if r.is_err() {
                failed.push(name);
            }
        };

        // Order matters: brake, kill the engine, then shed the electrical load,
        // master switch LAST so the rest is commanded while the bus is still live.
        step("parking brake", SetParkingBrake.execute(&json!({ "on": true }), ctx).await);
        step("engine", StopEngine.execute(&json!({}), ctx).await);
        for l in &self.lights {
            step("lights", lights.execute(&json!({ "light": l.key, "on": false }), ctx).await);
        }
        step("transponder", SetTransponderMode.execute(&json!({ "mode": "off" }), ctx).await);
        step("avionics", SetElectrical.execute(&json!({ "system": "avionics", "on": false }), ctx).await);
        step("alternator", SetElectrical.execute(&json!({ "system": "alternator", "on": false }), ctx).await);
        step("master", SetElectrical.execute(&json!({ "system": "master", "on": false }), ctx).await);

        if failed.is_empty() {
            Ok("Roger — aircraft secured: parking brake set, engine shut down, lights off, \
                transponder off, avionics, alternator and master off. Have a good one."
                .to_string())
        } else {
            // De-dup the repeated "lights" so the readback isn't noisy.
            failed.dedup();
            Ok(format!("Aircraft secured — but check {}.", failed.join(", ")))
        }
    }
}
