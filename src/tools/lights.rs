//! `set_lights` — switch an exterior/interior light on or off.
//!
//! Like `set_flaps`, the set of switchable lights comes from the aircraft
//! profile, so the LLM's `light` enum only ever contains lights the airframe
//! actually has. Same three-part pattern: own the dataref + valid values, no
//! safety gate needed here, read back the switch and confirm the actual state.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::aircraft::LightProfile;

pub struct SetLights {
    lights: Vec<LightProfile>,
}

#[derive(Deserialize)]
struct Args {
    /// Light key, one of the profile's lights (e.g. "landing", "strobe").
    light: String,
    /// Desired state: true = on, false = off.
    on: bool,
}

impl SetLights {
    pub fn new(lights: Vec<LightProfile>) -> Self {
        Self { lights }
    }

    fn keys(&self) -> Vec<&str> {
        self.lights.iter().map(|l| l.key.as_str()).collect()
    }
}

#[async_trait]
impl Tool for SetLights {
    fn name(&self) -> &str {
        "set_lights"
    }

    fn description(&self) -> &str {
        "Turn an exterior or interior light on or off. Use for commands like \
         'landing light on', 'strobes off', 'beacon on', 'taxi light off', \
         'nav lights on'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "light": {
                    "type": "string",
                    "enum": self.keys(),
                    "description": "Which light to switch."
                },
                "on": {
                    "type": "boolean",
                    "description": "true to turn on, false to turn off."
                }
            },
            "required": ["light", "on"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: Args = serde_json::from_value(args.clone())?;

        let light = match self
            .lights
            .iter()
            .find(|l| l.key.eq_ignore_ascii_case(&args.light))
        {
            Some(l) => l,
            None => {
                // Don't hard-fail (e.g. whisper hears "beak" for "beacon") —
                // ask, listing what we actually have.
                let names: Vec<&str> = self.lights.iter().map(|l| l.spoken.as_str()).collect();
                return Ok(format!("I didn't catch which light. I have {}.", names.join(", ")));
            }
        };

        // Write, then read back the actual switch state and confirm *that*.
        ctx.xplane.set_int(&light.dataref, args.on as i64).await?;
        let actual_on = ctx.xplane.get_float(&light.dataref).await? > 0.5;

        Ok(format!(
            "Roger, {} {}.",
            light.spoken,
            if actual_on { "on" } else { "off" }
        ))
    }
}
