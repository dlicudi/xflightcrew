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
            ctx.xplane
                .get_float_array(base)
                .await
                .ok()?
                .get(idx)
                .copied()
        } else {
            ctx.xplane.get_float(dref).await.ok()
        }
    }

    /// Resolve a requested system key against the profile, tolerantly: exact
    /// match first; then a bare noun to its pair entry ("alternator" →
    /// "alternators" on the SR22's split switches — switching both is what the
    /// bare word means); then a unique prefix; then a numbered request back to
    /// the unnumbered single ("battery one" where the only key is "battery").
    fn find_system(&self, requested: &str) -> Option<&ElectricalSystem> {
        let req = requested.to_ascii_lowercase();
        if let Some(s) = self
            .systems
            .iter()
            .find(|s| s.key.eq_ignore_ascii_case(&req))
        {
            return Some(s);
        }
        let plural = match req.strip_suffix('y') {
            Some(stem) => format!("{stem}ies"),
            None => format!("{req}s"),
        };
        if let Some(s) = self
            .systems
            .iter()
            .find(|s| s.key.eq_ignore_ascii_case(&plural))
        {
            return Some(s);
        }
        let mut pref = self
            .systems
            .iter()
            .filter(|s| s.key.to_ascii_lowercase().starts_with(&req));
        if let (Some(one), None) = (pref.next(), pref.next()) {
            return Some(one);
        }
        if let Some(stem) = req.strip_suffix(|c: char| c.is_ascii_digit()) {
            return self
                .systems
                .iter()
                .find(|s| s.key.eq_ignore_ascii_case(stem));
        }
        None
    }

    /// Fire a system's commands and read its state back.
    async fn switch(&self, sys: &ElectricalSystem, on: bool, ctx: &ToolCtx<'_>) -> Result<String> {
        let cmds = if on {
            &sys.on_commands
        } else {
            &sys.off_commands
        };
        for c in cmds {
            ctx.xplane.run_command(c).await?;
        }
        // Toggle-only switches: read each, fire only if it doesn't already
        // match. Firing a toggle blind as "on" switches OFF anything that was
        // already on — and on an xlua airframe the toggle is the ONLY way in,
        // because its datarefs refuse writes (403).
        for t in &sys.toggles {
            let now_on = Self::read(ctx, &t.dataref).await.is_some_and(|v| v > 0.5);
            if now_on != on {
                ctx.xplane.run_command(&t.command).await?;
            }
        }
        // Toggle-only systems are driven by writing their switch dataref
        // instead — see `ElectricalSystem::write_datarefs`.
        for d in &sys.write_datarefs {
            ctx.xplane.set_int(d, i64::from(on)).await?;
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
            format!(
                "{}: {} of {} on — check the panel.",
                sys.spoken,
                on_n,
                states.len()
            )
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
        let listing: Vec<String> = self
            .systems
            .iter()
            .map(|s| format!("{} ({})", s.key, s.spoken))
            .collect();
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

        let Some(sys) = self.find_system(&args.system) else {
            let names: Vec<&str> = self.systems.iter().map(|s| s.spoken.as_str()).collect();
            return Ok(format!(
                "I didn't catch which system. I have {}.",
                names.join(", ")
            ));
        };
        self.switch(sys, args.on, ctx).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sys(key: &str) -> ElectricalSystem {
        ElectricalSystem {
            write_datarefs: vec![],
            toggles: vec![],
            key: key.into(),
            spoken: key.into(),
            on_commands: vec![],
            off_commands: vec![],
            readback: vec![],
        }
    }

    #[test]
    fn find_system_resolves_split_and_single_airframes() {
        // SR22-style split systems: bare noun -> the pair entry; numbered -> exact.
        let sr22 = SetElectrical::new(vec![
            sys("battery1"),
            sys("battery2"),
            sys("alternator1"),
            sys("alternator2"),
            sys("batteries"),
            sys("alternators"),
            sys("avionics"),
        ]);
        assert_eq!(sr22.find_system("alternator").unwrap().key, "alternators");
        assert_eq!(sr22.find_system("battery").unwrap().key, "batteries");
        assert_eq!(sr22.find_system("alternator2").unwrap().key, "alternator2");
        assert_eq!(sr22.find_system("avionics").unwrap().key, "avionics");
        // Single-system airframe: a numbered request strips back to the single.
        let single = SetElectrical::new(vec![sys("battery"), sys("alternator")]);
        assert_eq!(single.find_system("battery1").unwrap().key, "battery");
        assert_eq!(single.find_system("alternator").unwrap().key, "alternator");
        assert!(single.find_system("inverter").is_none());
    }
}
