//! Electrical systems: `set_electrical` (battery, alternator, avionics master).
//!
//! Battery and avionics are scalar switches; the alternator is element 0 of the
//! per-engine `generator_on` array. (X-Plane exposes only one avionics switch —
//! the panel's BUS 1/BUS 2 both drive it — so there's nothing finer to control.)

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};

const BATTERY: &str = "sim/cockpit/electrical/battery_on";
const AVIONICS: &str = "sim/cockpit2/switches/avionics_power_on";
const GENERATOR: &str = "sim/cockpit/electrical/generator_on"; // array; [0] = alternator

/// (key the LLM selects by, spoken form).
const SYSTEMS: &[(&str, &str)] = &[
    ("battery", "battery"),
    ("alternator", "alternator"),
    ("avionics", "avionics master"),
];

pub struct SetElectrical;

#[derive(Deserialize)]
struct Args {
    system: String,
    on: bool,
}

#[async_trait]
impl Tool for SetElectrical {
    fn name(&self) -> &str {
        "set_electrical"
    }

    fn description(&self) -> &str {
        "Turn an electrical system on or off: the battery master, the alternator, \
         the avionics master, or 'all' for every electrical system at once. Use \
         for 'battery on', 'alternator on', 'avionics off', 'switch on all \
         electrical systems', 'all electrical off'."
    }

    fn parameters(&self) -> Value {
        let mut keys: Vec<&str> = SYSTEMS.iter().map(|(k, _)| *k).collect();
        keys.push("all");
        json!({
            "type": "object",
            "properties": {
                "system": {"type": "string", "enum": keys, "description": "Which electrical system, or 'all'."},
                "on": {"type": "boolean", "description": "true = on, false = off"}
            },
            "required": ["system", "on"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: Args = serde_json::from_value(args.clone())?;
        let v = args.on as i64;

        // "all" → battery + alternator + avionics together.
        if args.system.eq_ignore_ascii_case("all") {
            ctx.xplane.set_int(BATTERY, v).await?;
            ctx.xplane.set_int_array_elem(GENERATOR, 0, v).await?;
            ctx.xplane.set_int(AVIONICS, v).await?;
            return Ok(format!(
                "Roger, all electrical systems {}.",
                if args.on { "on" } else { "off" }
            ));
        }

        let Some((key, spoken)) = SYSTEMS
            .iter()
            .find(|(k, _)| k.eq_ignore_ascii_case(&args.system))
            .copied()
        else {
            let names: Vec<&str> = SYSTEMS.iter().map(|(_, s)| *s).collect();
            return Ok(format!("I didn't catch which system. I have {}.", names.join(", ")));
        };

        let on = match key {
            "battery" => {
                ctx.xplane.set_int(BATTERY, v).await?;
                ctx.xplane.get_float(BATTERY).await? > 0.5
            }
            "avionics" => {
                ctx.xplane.set_int(AVIONICS, v).await?;
                ctx.xplane.get_float(AVIONICS).await? > 0.5
            }
            "alternator" => {
                ctx.xplane.set_int_array_elem(GENERATOR, 0, v).await?;
                ctx.xplane.get_float_array(GENERATOR).await?.first().copied().unwrap_or(0.0) > 0.5
            }
            _ => unreachable!(),
        };
        Ok(format!("Roger, {spoken} {}.", if on { "on" } else { "off" }))
    }
}
