//! Electrical systems: `set_electrical` (battery, alternator, avionics master).
//!
//! Battery is a scalar switch; the alternator is element 0 of the per-engine
//! `generator_on` array. The C172's two AVIONICS panel switches are two separate
//! systems — BUS 1 is the avionics master, BUS 2 is the electrical bus cross-tie
//! — each driven by its own command (see the consts below). "avionics on" fires
//! both so both panel switches move, not just one.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};

// Battery and alternator: drive the panel switch with commands (a raw dataref
// write can set the value without animating the 3D switch); the datarefs are
// read back to confirm. The alternator is generator #1 (element 0 of the array).
const BATTERY: &str = "sim/cockpit/electrical/battery_on";
const CMD_BATTERY_ON: &str = "sim/electrical/battery_1_on";
const CMD_BATTERY_OFF: &str = "sim/electrical/battery_1_off";
const CMD_ALTERNATOR_ON: &str = "sim/electrical/generator_1_on";
const CMD_ALTERNATOR_OFF: &str = "sim/electrical/generator_1_off";
// The C172's two AVIONICS panel switches are two *different* systems, each
// driven by a command (a dataref write sets the value but won't animate the 3D
// switch — same quirk as the fuel selector). Verified with DataRefTool:
//   BUS 1 = avionics master → sim/systems/avionics_on,  reads back via avionics_on
//   BUS 2 = bus cross-tie    → sim/electrical/cross_tie_*, reads back via cross_tie
const AVIONICS_BUS1: &str = "sim/cockpit/electrical/avionics_on";
const AVIONICS_BUS2: &str = "sim/cockpit2/electrical/cross_tie";
const CMD_AVIONICS_ON: &str = "sim/systems/avionics_on";
const CMD_AVIONICS_OFF: &str = "sim/systems/avionics_off";
const CMD_CROSS_TIE_ON: &str = "sim/electrical/cross_tie_on";
const CMD_CROSS_TIE_OFF: &str = "sim/electrical/cross_tie_off";
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
         electrical systems', 'all electrical off', 'electrics on', 'electrics \
         off', 'master switch on', 'battery master on', 'generator on', \
         'kill the avionics', 'power up the avionics'. 'electrics' or 'master' \
         with no system named means 'all'. The alternator is also called the \
         generator."
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

        // "all" → battery + alternator + avionics together (all via commands so
        // the panel switches animate).
        if args.system.eq_ignore_ascii_case("all") {
            if args.on {
                ctx.xplane.run_command(CMD_BATTERY_ON).await?;
                ctx.xplane.run_command(CMD_ALTERNATOR_ON).await?;
                ctx.xplane.run_command(CMD_AVIONICS_ON).await?;
                ctx.xplane.run_command(CMD_CROSS_TIE_ON).await?;
            } else {
                ctx.xplane.run_command(CMD_BATTERY_OFF).await?;
                ctx.xplane.run_command(CMD_ALTERNATOR_OFF).await?;
                ctx.xplane.run_command(CMD_AVIONICS_OFF).await?;
                ctx.xplane.run_command(CMD_CROSS_TIE_OFF).await?;
            }
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
                ctx.xplane
                    .run_command(if args.on { CMD_BATTERY_ON } else { CMD_BATTERY_OFF })
                    .await?;
                ctx.xplane.get_float(BATTERY).await? > 0.5
            }
            "avionics" => {
                // Two switches, two systems — commands (not dataref writes) so both
                // 3D switches animate: BUS 1 = avionics master, BUS 2 = cross-tie.
                if args.on {
                    ctx.xplane.run_command(CMD_AVIONICS_ON).await?;
                    ctx.xplane.run_command(CMD_CROSS_TIE_ON).await?;
                } else {
                    ctx.xplane.run_command(CMD_AVIONICS_OFF).await?;
                    ctx.xplane.run_command(CMD_CROSS_TIE_OFF).await?;
                }
                // Read both back and name them so the pilot hears that both buses
                // moved, not just a vague "avionics on".
                let bus1 = ctx.xplane.get_float(AVIONICS_BUS1).await? > 0.5;
                let bus2 = ctx.xplane.get_float(AVIONICS_BUS2).await? > 0.5;
                return Ok(match (bus1, bus2) {
                    (true, true) => "Roger, avionics master on — bus 1 and bus 2.".to_string(),
                    (false, false) => "Roger, avionics master off — bus 1 and bus 2.".to_string(),
                    // Mixed: report exactly what the panel shows rather than lie.
                    (b1, b2) => format!(
                        "Avionics bus 1 {}, bus 2 {}.",
                        if b1 { "on" } else { "off" },
                        if b2 { "on" } else { "off" }
                    ),
                });
            }
            "alternator" => {
                ctx.xplane
                    .run_command(if args.on { CMD_ALTERNATOR_ON } else { CMD_ALTERNATOR_OFF })
                    .await?;
                ctx.xplane.get_float_array(GENERATOR).await?.first().copied().unwrap_or(0.0) > 0.5
            }
            _ => unreachable!(),
        };
        Ok(format!("Roger, {spoken} {}.", if on { "on" } else { "off" }))
    }
}
