//! Electrical systems: `set_electrical`, built from the airframe profile.
//!
//! Each [`ElectricalSystem`] is a named set of on/off *commands* (commands drive
//! the panel switch — a raw dataref write may set the value without animating the
//! 3D switch) plus *readback* datarefs to confirm. So different wiring is config,
//! not code: the 172 has a single battery + a split BAT/ALT master; the SR22 has
//! dual battery/alternator. `all` switches every configured system at once.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::aircraft::ElectricalSystem;

pub struct SetElectrical {
    systems: Vec<ElectricalSystem>,
}

#[derive(Deserialize)]
struct Args {
    system: String,
    on: bool,
}

impl SetElectrical {
    pub fn new(systems: Vec<ElectricalSystem>) -> Self {
        Self { systems }
    }

    /// Read a readback dataref (`name` or `name[i]`) as a 0/1-ish float.
    async fn read(ctx: &ToolCtx<'_>, dref: &str) -> Option<f64> {
        if let Some(open) = dref.find('[') {
            let base = &dref[..open];
            let idx: usize = dref[open + 1..dref.find(']')?].parse().ok()?;
            ctx.xplane.get_float_array(base).await.ok()?.get(idx).copied()
        } else {
            ctx.xplane.get_float(dref).await.ok()
        }
    }

    /// Fire a system's commands and read its state back.
    async fn switch(&self, sys: &ElectricalSystem, on: bool, ctx: &ToolCtx<'_>) -> Result<String> {
        let cmds = if on { &sys.on_commands } else { &sys.off_commands };
        for c in cmds {
            ctx.xplane.run_command(c).await?;
        }
        // Confirm from the readback datarefs: all on / all off / mixed.
        let mut states = Vec::new();
        for d in &sys.readback {
            if let Some(v) = Self::read(ctx, d).await {
                states.push(v > 0.5);
            }
        }
        Ok(if states.is_empty() {
            // No readback configured — trust the command.
            format!("Roger, {} {}.", sys.spoken, if on { "on" } else { "off" })
        } else if states.iter().all(|&s| s) {
            format!("Roger, {} on.", sys.spoken)
        } else if states.iter().all(|&s| !s) {
            format!("Roger, {} off.", sys.spoken)
        } else {
            // Mixed — report rather than lie. (e.g. one of a pair didn't move.)
            let on_n = states.iter().filter(|&&s| s).count();
            format!("{}: {} of {} on — check the panel.", sys.spoken, on_n, states.len())
        })
    }
}

#[async_trait]
impl Tool for SetElectrical {
    fn name(&self) -> &str {
        "set_electrical"
    }

    fn description(&self) -> &str {
        // Static base; the available systems are listed in `parameters`.
        "Turn an electrical system on or off — e.g. the battery, the alternator \
         (a.k.a. generator), the avionics master, or (where present) the split \
         'master' switch / individual buses. Pass 'all' for every electrical \
         system at once. Use for 'battery on', 'alternator on', 'avionics off', \
         'master switch on', 'all electrical off', 'electrics on', 'kill the \
         avionics'. 'electrics'/'all electrical' with no system named → 'all'."
    }

    fn parameters(&self) -> Value {
        let mut keys: Vec<&str> = self.systems.iter().map(|s| s.key.as_str()).collect();
        keys.push("all");
        let listing: Vec<String> = self.systems.iter().map(|s| format!("{} ({})", s.key, s.spoken)).collect();
        json!({
            "type": "object",
            "properties": {
                "system": {
                    "type": "string",
                    "enum": keys,
                    "description": format!("Which system, or 'all'. Available: {}.", listing.join(", "))
                },
                "on": {"type": "boolean", "description": "true = on, false = off"}
            },
            "required": ["system", "on"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: Args = serde_json::from_value(args.clone())?;

        if args.system.eq_ignore_ascii_case("all") {
            for sys in &self.systems {
                self.switch(sys, args.on, ctx).await?;
            }
            return Ok(format!(
                "Roger, all electrical systems {}.",
                if args.on { "on" } else { "off" }
            ));
        }

        let Some(sys) = self.systems.iter().find(|s| s.key.eq_ignore_ascii_case(&args.system)) else {
            let names: Vec<&str> = self.systems.iter().map(|s| s.spoken.as_str()).collect();
            return Ok(format!("I didn't catch which system. I have {}.", names.join(", ")));
        };
        self.switch(sys, args.on, ctx).await
    }
}
