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

use super::controls::SetPitchTrim;
use super::engine::{SetCarbHeat, SetFuelSelector, SetMixture};
use super::lights::SetLights;
use super::radio::SetTransponderMode;
use super::{Tool, ToolCtx};
use crate::aircraft::LightProfile;

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
