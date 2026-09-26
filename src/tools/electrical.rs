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
use crate::aircraft::{ElectricalSystem, ToggleSwitch};

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
        // A guarded switch going OFF: open its guard first, and wait for the
        // guard to READ open, because the aircraft's own logic holds the
        // switch on for as long as the guard reads shut.
        let mut guard_stuck = false;
        if !on {
            for g in &sys.guards {
                if !self.move_guard(g, true, ctx).await? {
                    guard_stuck = true;
                }
            }
        }
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
        if on {
            // Guard back down over a switch that is on: the crew's habit, and
            // on the 737 what holds the battery on.
            for g in &sys.guards {
                self.move_guard(g, false, ctx).await?;
            }
        }
        if !sys.guards.is_empty() {
            // Give the aircraft's per-frame logic a few frames to undo the
            // switch if it is going to, so the readback sees what stays.
            tokio::time::sleep(std::time::Duration::from_millis(GUARD_SETTLE_MS)).await;
        }
        // Confirm from the readback datarefs, AGAINST THE INTENT. Reporting
        // whatever came back answered "Roger, avionics master on" to "avionics
        // off" when the command silently failed — the readback confirming
        // itself (the set_lights lesson, applied here).
        let mut states = Vec::new();
        for d in &sys.readback {
            if let Some(v) = Self::read(ctx, d).await {
                states.push(v > 0.5);
            }
        }
        Ok(if states.is_empty() {
            // No readback configured — trust the command.
            format!("Roger, {} {}.", sys.spoken, if on { "on" } else { "off" })
        } else if states.iter().all(|&s| s == on) {
            format!("Roger, {} {}.", sys.spoken, if on { "on" } else { "off" })
        } else if states.iter().all(|&s| s != on) {
            stuck_line(&sys.spoken, guard_stuck)
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

    /// Put one guard cover open (`open`) or shut, read first and toggled only
    /// on a difference, then read until it agrees (bounded). `Ok(true)` when
    /// it reads as wanted. A failed read is no measurement: the toggle is
    /// never fired blind (it could shut an open guard), and the switch's own
    /// readback then says what happened.
    async fn move_guard(&self, g: &ToggleSwitch, open: bool, ctx: &ToolCtx<'_>) -> Result<bool> {
        match Self::read(ctx, &g.dataref).await {
            None => return Ok(false),
            Some(pos) if guard_is_open(pos) == open => return Ok(true),
            Some(_) => ctx.xplane.run_command(&g.command).await?,
        }
        let deadline = std::time::Instant::now() + std::time::Duration::from_millis(GUARD_WAIT_MS);
        loop {
            tokio::time::sleep(std::time::Duration::from_millis(100)).await;
            if let Some(pos) = Self::read(ctx, &g.dataref).await {
                if guard_is_open(pos) == open {
                    return Ok(true);
                }
            }
            if std::time::Instant::now() >= deadline {
                return Ok(false);
            }
        }
    }
}

/// How long a guard gets to swing (the 737's covers animate at 20 a second,
/// about 50 ms end to end; the rest is Web API round trips).
const GUARD_WAIT_MS: u64 = 1_500;
/// Frames for the aircraft's own logic to re-assert a guarded switch before
/// the readback.
const GUARD_SETTLE_MS: u64 = 300;

/// The 737's own test (`B738_guard_cover_logic`): a cover under 0.5 is shut.
fn guard_is_open(pos: f64) -> bool {
    pos >= 0.5
}

/// A switch that did not move, said honestly, naming the guard when that is
/// what held it.
fn stuck_line(spoken: &str, guard_stuck: bool) -> String {
    if guard_stuck {
        format!("The {spoken} didn't move; its guard stayed shut.")
    } else {
        format!("The {spoken} didn't move.")
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
            // Aggregate the per-system results instead of announcing success
            // unconditionally: a switch that didn't move must reach the pilot,
            // not vanish behind a cheerful summary.
            let mut problems: Vec<String> = Vec::new();
            for sys in &self.systems {
                let line = self.switch(sys, args.on, ctx).await?;
                if !line.starts_with("Roger") {
                    problems.push(line);
                }
            }
            return Ok(if problems.is_empty() {
                format!(
                    "Roger, all electrical systems {}.",
                    if args.on { "on" } else { "off" }
                )
            } else {
                format!(
                    "All electrical systems {}, except: {}",
                    if args.on { "on" } else { "off" },
                    problems.join(" ")
                )
            });
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
            guards: vec![],
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

    /// The stock 737's battery as its xlua keeps it: `B738_guard_cover_logic`
    /// sets `battery_on[0]` to 1 every frame while `guard_cover_pos[0]` reads
    /// under 0.5, `battery_1_off` is replaced by a handler that sets it to 0,
    /// and `laminar/B738/guard_cover00` toggles the cover.
    struct B738Battery {
        guard: std::sync::Mutex<f64>,
        switch: std::sync::Mutex<f64>,
        commands: std::sync::Mutex<Vec<String>>,
    }

    const GUARD_CMD: &str = "laminar/B738/guard_cover00";
    const GUARD_POS: &str = "laminar/B738/button_switch/guard_cover_pos";
    const BATTERY_ON: &str = "sim/cockpit/electrical/battery_on";

    impl B738Battery {
        /// At the stand after the flight: guard shut, battery on.
        fn at_the_stand() -> Self {
            Self {
                guard: std::sync::Mutex::new(0.0),
                switch: std::sync::Mutex::new(1.0),
                commands: std::sync::Mutex::new(Vec::new()),
            }
        }
        fn battery(&self) -> f64 {
            if *self.guard.lock().unwrap() < 0.5 {
                1.0
            } else {
                *self.switch.lock().unwrap()
            }
        }
    }

    #[async_trait]
    impl crate::xplane::XPlane for B738Battery {
        async fn get_float(&self, name: &str) -> Result<f64> {
            match name {
                BATTERY_ON => Ok(self.battery()),
                _ => Err(anyhow::anyhow!("no read for {name}")),
            }
        }
        async fn set_float(&self, _: &str, _: f64) -> Result<()> {
            Ok(())
        }
        async fn set_int(&self, _: &str, _: i64) -> Result<()> {
            Ok(())
        }
        async fn run_command(&self, name: &str) -> Result<()> {
            self.commands.lock().unwrap().push(name.to_string());
            match name {
                GUARD_CMD => {
                    let mut g = self.guard.lock().unwrap();
                    *g = if *g < 0.5 { 1.0 } else { 0.0 };
                }
                "sim/electrical/battery_1_off" => *self.switch.lock().unwrap() = 0.0,
                "sim/electrical/battery_1_on" => *self.switch.lock().unwrap() = 1.0,
                _ => {}
            }
            Ok(())
        }
        async fn run_command_for(&self, name: &str, _: f64) -> Result<()> {
            self.run_command(name).await
        }
        async fn get_float_array(&self, name: &str) -> Result<Vec<f64>> {
            match name {
                GUARD_POS => Ok(vec![*self.guard.lock().unwrap(), 0.0]),
                _ => Err(anyhow::anyhow!("no read for {name}")),
            }
        }
        async fn set_int_array_elem(&self, _: &str, _: usize, _: i64) -> Result<()> {
            Ok(())
        }
        async fn set_float_array_elem(&self, _: &str, _: usize, _: f64) -> Result<()> {
            Ok(())
        }
    }

    fn b738_battery() -> ElectricalSystem {
        crate::aircraft::load("boeing_737_800")
            .expect("737 profile loads")
            .electrical
            .into_iter()
            .find(|s| s.key == "battery")
            .expect("the 737 has a battery")
    }

    fn ctx<'a>(xp: &'a B738Battery, cfg: &'a crate::config::Config) -> ToolCtx<'a> {
        ToolCtx {
            xplane: xp,
            config: cfg,
            voice: None,
            status: None,
            reminders: None,
        }
    }

    /// 17:07Z 2026-09-26: "All electrical systems off, except: The battery
    /// didn't move." The profile without its guard reproduces the line; with
    /// the guard it opens the cover first and the battery goes off.
    #[tokio::test(start_paused = true)]
    async fn the_737_battery_goes_off_through_its_guard() {
        let cfg = crate::config::Config::default();
        let mut unguarded = b738_battery();
        unguarded.guards.clear();
        let xp = B738Battery::at_the_stand();
        let line = SetElectrical::new(vec![unguarded])
            .execute(&json!({"system": "battery", "on": false}), &ctx(&xp, &cfg))
            .await
            .unwrap();
        assert_eq!(line, "The battery didn't move.", "the recorded failure");

        let xp = B738Battery::at_the_stand();
        let line = SetElectrical::new(vec![b738_battery()])
            .execute(&json!({"system": "all", "on": false}), &ctx(&xp, &cfg))
            .await
            .unwrap();
        assert_eq!(line, "Roger, all electrical systems off.");
        assert_eq!(xp.battery(), 0.0);
        let cmds = xp.commands.lock().unwrap().clone();
        assert_eq!(
            cmds,
            vec![GUARD_CMD, "sim/electrical/battery_1_off"],
            "guard FIRST"
        );

        // Back on: the switch, then the guard shut over it.
        let line = SetElectrical::new(vec![b738_battery()])
            .execute(&json!({"system": "battery", "on": true}), &ctx(&xp, &cfg))
            .await
            .unwrap();
        assert_eq!(line, "Roger, battery on.");
        assert!(*xp.guard.lock().unwrap() < 0.5, "guard closed again");
    }

    /// Cold and dark the guard is already open: it is never toggled shut by a
    /// blind press on the way off.
    #[tokio::test(start_paused = true)]
    async fn an_open_guard_is_left_open() {
        let cfg = crate::config::Config::default();
        let xp = B738Battery::at_the_stand();
        *xp.guard.lock().unwrap() = 1.0;
        let line = SetElectrical::new(vec![b738_battery()])
            .execute(&json!({"system": "battery", "on": false}), &ctx(&xp, &cfg))
            .await
            .unwrap();
        assert_eq!(line, "Roger, battery off.");
        assert!(!xp.commands.lock().unwrap().iter().any(|c| c == GUARD_CMD));
    }

    /// A guard whose position cannot be read is not pressed blind, and the
    /// switch that stays on is said, naming the guard.
    #[tokio::test(start_paused = true)]
    async fn a_guard_that_will_not_open_is_named() {
        let cfg = crate::config::Config::default();
        let xp = B738Battery::at_the_stand();
        let mut sys = b738_battery();
        sys.guards[0].dataref = "laminar/B738/nowhere[0]".into();
        let line = SetElectrical::new(vec![sys])
            .execute(&json!({"system": "all", "on": false}), &ctx(&xp, &cfg))
            .await
            .unwrap();
        assert_eq!(
            line,
            "All electrical systems off, except: The battery didn't move; its guard stayed shut."
        );
        assert!(!xp.commands.lock().unwrap().iter().any(|c| c == GUARD_CMD));
    }
}
