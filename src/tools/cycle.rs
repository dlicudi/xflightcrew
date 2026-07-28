//! Cycle-switch systems — for airframes (the AWX DC-3) whose power, fuel, and
//! mixture run through xlua scripts that OVERRIDE the stock datarefs and expose
//! only up/down *cycle* commands. Writing the position dataref is a no-op on
//! those add-ons, so reaching a target means reading the current detent and
//! firing `up`/`down` the right number of times.
//!
//! Everything airframe-specific is config (the commands/datarefs come from the
//! profile, [`crate::aircraft`]); the sequencing logic here is shared. Universal
//! datarefs (throttle, engine-running, bus volts) stay as consts.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};
use std::time::Duration;

use super::{Tool, ToolCtx};
use crate::aircraft::{
    CycleSelectorProfile, CycleSwitch, MixtureNotchProfile, PistonStartProfile, PowerSwitchProfile,
    TurbineStartProfile,
};

/// Universal throttle (all engines), 0..1.
/// The thrust lever THIS airframe obeys — not always the stock one (the
/// Citation X tracks `throttle_jet_rev_ratio`, so every write to
/// `throttle_ratio_all` went nowhere and it flew a whole pattern at takeoff
/// thrust). Resolved per call, so a profile swap takes effect immediately.
fn throttle_dref() -> String {
    crate::aircraft::throttle_dataref()
}
/// Per-engine "is running" flags.
const ENGN_RUNNING: &str = "sim/flightmodel/engine/ENGN_running";
/// N1 at which the condition lever introduces fuel during a PT6 start (POM: crank
/// first, fuel at 12% — fuel-before-crank gives no light-off, verified live).
const FUEL_INTRO_N1: f64 = 12.0;
/// Universal per-engine boost/aux pump switch array.
const FUEL_PUMP_ON: &str = "sim/cockpit2/engine/actuators/fuel_pump_on";
/// Electrical bus volts (any element) — enough to crank?
const BUS_VOLTS: &str = "sim/cockpit2/electrical/bus_volts";
/// The DC-3 is a 24V system whose healthy battery bus reads ~21.7–24V (it varies
/// run to run), and it cranks fine across that range. The gate only needs to
/// catch a battery that's actually off/flat (which reads ~0V), so it sits well
/// below the live range rather than at the nominal bus voltage.
const MIN_START_VOLTS: f64 = 12.0;
const ONGROUND: &str = "sim/flightmodel/failures/onground_any";
const GROUNDSPEED: &str = "sim/flightmodel/position/groundspeed"; // m/s
const N1_PERCENT: &str = "sim/cockpit2/engine/indicators/N1_percent"; // per-engine, turbine spool
/// Per-engine N2 — the high-pressure spool a TURBOFAN's start is flown
/// against (see [`crate::aircraft::TurbineStartProfile::fuel_intro_n2`]).
const N2_PERCENT: &str = "sim/cockpit2/engine/indicators/N2_percent";

/// Drive a [`CycleSwitch`] to `target` by reading its position and firing
/// `up`/`down`. Bounded by the detent span so a stuck switch can't loop forever.
/// Returns the final position read back.
async fn cycle_to(ctx: &ToolCtx<'_>, sw: &CycleSwitch, target: i64) -> Result<i64> {
    let span = (sw.max - sw.min + 1).max(1);
    let mut last = (ctx.xplane.get_float(&sw.position).await?).round() as i64;
    for _ in 0..=span {
        if last == target {
            return Ok(last);
        }
        let go_up = if sw.wrap {
            // Shortest way round the ring.
            let up_steps = (target - last).rem_euclid(span);
            let down_steps = (last - target).rem_euclid(span);
            up_steps <= down_steps
        } else {
            target > last
        };
        ctx.xplane
            .run_command(if go_up { &sw.up } else { &sw.down })
            .await?;
        tokio::time::sleep(Duration::from_millis(140)).await;
        last = (ctx.xplane.get_float(&sw.position).await?).round() as i64;
    }
    Ok(last)
}

async fn engine_running(ctx: &ToolCtx<'_>, idx: usize) -> bool {
    ctx.xplane
        .get_float_array(ENGN_RUNNING)
        .await
        .ok()
        .and_then(|v| v.get(idx).copied())
        .map(|r| r > 0.5)
        .unwrap_or(false)
}

// ----------------------------------------------------------------------------
// Power switch (battery / off / ground power)
// ----------------------------------------------------------------------------

pub struct SetPower {
    profile: PowerSwitchProfile,
}

impl SetPower {
    pub fn new(profile: PowerSwitchProfile) -> Self {
        Self { profile }
    }
}

#[derive(Deserialize)]
struct StateArg {
    state: String,
}

#[async_trait]
impl Tool for SetPower {
    fn name(&self) -> &str {
        "set_power"
    }
    fn description(&self) -> &str {
        "Set the DC power switch: battery on, power off, or ground power (GPU). \
         Use for 'battery on', 'battery off', 'power on', 'power off', 'turn on \
         the battery', 'ground power', 'connect the GPU', 'ground power unit on'. \
         'Battery on' selects the battery; 'battery off' or 'power off' selects off."
    }
    fn parameters(&self) -> Value {
        let keys: Vec<&str> = self.profile.states.iter().map(|s| s.key.as_str()).collect();
        let listing: Vec<String> = self
            .profile
            .states
            .iter()
            .map(|s| format!("{} ({})", s.key, s.spoken))
            .collect();
        json!({
            "type": "object",
            "properties": {
                "state": {"type": "string", "enum": keys,
                          "description": format!("Power switch position. Available: {}.", listing.join(", "))}
            },
            "required": ["state"]
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: StateArg = serde_json::from_value(args.clone())?;
        let Some(state) = self
            .profile
            .states
            .iter()
            .find(|s| s.key.eq_ignore_ascii_case(&a.state))
        else {
            let names: Vec<&str> = self
                .profile
                .states
                .iter()
                .map(|s| s.spoken.as_str())
                .collect();
            return Ok(format!(
                "I didn't catch the power setting '{}'. I have {}.",
                a.state,
                names.join(", ")
            ));
        };
        let reached = cycle_to(ctx, &self.profile.switch, state.value).await?;
        if reached != state.value {
            return Ok(format!(
                "I couldn't move the power switch to {} — it stuck at {}.",
                state.spoken, reached
            ));
        }
        // Confirm from the battery readback when given.
        if let Some(dref) = &self.profile.battery_readback {
            if let Ok(arr) = ctx.xplane.get_float_array(dref).await {
                let live = arr.iter().any(|&v| v > 0.5);
                let volts = ctx
                    .xplane
                    .get_float_array(BUS_VOLTS)
                    .await
                    .ok()
                    .and_then(|v| v.into_iter().find(|&x| x > 0.0));
                if live {
                    return match volts {
                        Some(v) => Ok(format!("Roger, {} — bus at {v:.0} volts.", state.spoken)),
                        None => Ok(format!("Roger, {}.", state.spoken)),
                    };
                }
            }
        }
        Ok(format!("Roger, {}.", state.spoken))
    }
}

// ----------------------------------------------------------------------------
// Fuel selectors (one per side, cycle-driven)
// ----------------------------------------------------------------------------

pub struct SetCycleFuelSelector {
    selectors: Vec<CycleSelectorProfile>,
}

impl SetCycleFuelSelector {
    pub fn new(selectors: Vec<CycleSelectorProfile>) -> Self {
        Self { selectors }
    }
}

#[derive(Deserialize)]
struct FuelSelArgs {
    /// Which selector (e.g. `left`, `right`).
    selector: String,
    /// Target position token (one of that selector's positions).
    position: String,
}

#[async_trait]
impl Tool for SetCycleFuelSelector {
    fn name(&self) -> &str {
        "set_fuel_selector"
    }
    fn description(&self) -> &str {
        "Set a fuel tank selector. This aircraft has one selector per side, each \
         feeding its engine. Use for 'left fuel selector to left main', 'right \
         tank selector to right main', 'left selector off', 'feed the left engine \
         from the right main', 'fuel selectors to mains'."
    }
    fn parameters(&self) -> Value {
        let sel_keys: Vec<&str> = self.selectors.iter().map(|s| s.key.as_str()).collect();
        // Position tokens are shared across selectors on the DC-3; list the union.
        let mut pos_keys: Vec<&str> = Vec::new();
        for s in &self.selectors {
            for p in &s.positions {
                if !pos_keys.contains(&p.key.as_str()) {
                    pos_keys.push(p.key.as_str());
                }
            }
        }
        json!({
            "type": "object",
            "properties": {
                "selector": {"type": "string", "enum": sel_keys, "description": "Which fuel selector."},
                "position": {"type": "string", "enum": pos_keys,
                             "description": "Target tank position (off, left_main, right_main, left_aux, right_aux)."}
            },
            "required": ["selector", "position"]
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: FuelSelArgs = serde_json::from_value(args.clone())?;
        let Some(sel) = self
            .selectors
            .iter()
            .find(|s| s.key.eq_ignore_ascii_case(&a.selector))
        else {
            let names: Vec<&str> = self.selectors.iter().map(|s| s.spoken.as_str()).collect();
            return Ok(format!(
                "Which selector? I have the {}.",
                names.join(" and the ")
            ));
        };
        let Some(pos) = sel
            .positions
            .iter()
            .find(|p| p.key.eq_ignore_ascii_case(&a.position))
        else {
            let names: Vec<&str> = sel.positions.iter().map(|p| p.spoken.as_str()).collect();
            return Ok(format!(
                "I didn't catch the position '{}' for the {}. I have {}.",
                a.position,
                sel.spoken,
                names.join(", ")
            ));
        };
        let reached = cycle_to(ctx, &sel.switch, pos.value).await?;
        if let Some(hit) = sel.positions.iter().find(|p| p.value == reached) {
            return Ok(format!("Roger, {} to {}.", sel.spoken, hit.spoken));
        }
        Ok(format!("Roger, {} to {}.", sel.spoken, pos.spoken))
    }
}

// ----------------------------------------------------------------------------
// Notched mixture lever
// ----------------------------------------------------------------------------

pub struct SetMixture {
    profile: MixtureNotchProfile,
}

impl SetMixture {
    pub fn new(profile: MixtureNotchProfile) -> Self {
        Self { profile }
    }
}

#[derive(Deserialize)]
struct NotchArg {
    notch: String,
}

#[async_trait]
impl Tool for SetMixture {
    fn name(&self) -> &str {
        "set_mixture"
    }
    fn description(&self) -> &str {
        "Set the mixture lever to a notch: idle cutoff, auto-lean, auto-rich, or \
         full rich. This aircraft's mixture is a detented lever, not a percentage. \
         Use for 'mixture rich', 'mixture full rich', 'mixture auto rich', 'lean \
         the mixture', 'auto lean', 'mixture to cutoff', 'idle cutoff', 'mixture \
         off'. To shut the engines down use stop_engines."
    }
    fn parameters(&self) -> Value {
        let keys: Vec<&str> = self
            .profile
            .notches
            .iter()
            .map(|n| n.key.as_str())
            .collect();
        let listing: Vec<String> = self
            .profile
            .notches
            .iter()
            .map(|n| format!("{} ({})", n.key, n.spoken))
            .collect();
        json!({
            "type": "object",
            "properties": {
                "notch": {"type": "string", "enum": keys,
                          "description": format!("Mixture notch, lean to rich. Available: {}.", listing.join(", "))}
            },
            "required": ["notch"]
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: NotchArg = serde_json::from_value(args.clone())?;
        let Some(notch) = self
            .profile
            .notches
            .iter()
            .find(|n| n.key.eq_ignore_ascii_case(&a.notch))
        else {
            let names: Vec<&str> = self
                .profile
                .notches
                .iter()
                .map(|n| n.spoken.as_str())
                .collect();
            return Ok(format!(
                "I didn't catch the mixture setting '{}'. I have {}.",
                a.notch,
                names.join(", ")
            ));
        };
        ctx.xplane
            .set_float(&self.profile.lever, notch.value as f64)
            .await?;
        // Confirm the lever settled on the requested notch when a readback is set
        // (the lever animates, so give it a moment, then round to the nearest).
        if let Some(dref) = &self.profile.readback {
            tokio::time::sleep(Duration::from_millis(400)).await;
            if let Ok(actual) = ctx.xplane.get_float(dref).await {
                if let Some(hit) = self
                    .profile
                    .notches
                    .iter()
                    .find(|n| (n.value as f64 - actual).abs() < 0.5)
                {
                    return Ok(format!("Roger, mixture {}.", hit.spoken));
                }
            }
        }
        Ok(format!("Roger, mixture {}.", notch.spoken))
    }
}

// ----------------------------------------------------------------------------
// Twin piston start / shutdown
// ----------------------------------------------------------------------------

pub struct StartEngines {
    start: PistonStartProfile,
    selectors: Vec<CycleSelectorProfile>,
    power: Option<PowerSwitchProfile>,
}

impl StartEngines {
    pub fn new(
        start: PistonStartProfile,
        selectors: Vec<CycleSelectorProfile>,
        power: Option<PowerSwitchProfile>,
    ) -> Self {
        Self {
            start,
            selectors,
            power,
        }
    }
}

#[async_trait]
impl Tool for StartEngines {
    fn name(&self) -> &str {
        "start_engines"
    }
    fn description(&self) -> &str {
        "Start the engines: runs the full sequence (battery on, fuel selectors to \
         the main tanks, mixture rich, throttle cracked, magnetos both, crank each \
         starter, generators on once running). Use for 'start the engines', 'start \
         her up', 'engines start', 'engines on', 'start up', 'fire them up', 'start \
         the engine'. 'Engines on' means START, not an electrical switch."
    }
    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let n = self.start.starters.len();
        // Already running? Don't re-crank a live engine.
        let any_running = {
            let mut r = false;
            for i in 0..n {
                if engine_running(ctx, i).await {
                    r = true;
                    break;
                }
            }
            r
        };
        if any_running {
            return Ok("Engines are already running.".to_string());
        }

        // Battery on (cycle the power switch to its first state) and check volts.
        if let Some(power) = &self.power {
            if let Some(batt) = power.states.first() {
                cycle_to(ctx, &power.switch, batt.value).await?;
            }
        }
        let volts = ctx
            .xplane
            .get_float_array(BUS_VOLTS)
            .await?
            .into_iter()
            .fold(0.0_f64, f64::max);
        if volts < MIN_START_VOLTS {
            return Ok(format!(
                "Battery's low, only {volts:.0} volts — connect ground power before starting."
            ));
        }

        // Fuel selectors to their start (main-tank) positions, so fuel actually
        // reaches the engines (the cycle command opens the valve; a write won't).
        for sel in &self.selectors {
            if let Some(target) = sel.start_value {
                cycle_to(ctx, &sel.switch, target).await?;
            }
        }

        // Mixture rich, mags both. Crack the throttle ONLY on the ground: this is
        // also the air-restart path (a flamed-out engine while flying), and there
        // chopping the throttle to a 12% crack would yank power off the live
        // engine and dump the pilot's setting — so airborne we leave the throttle
        // where they have it and just re-light.
        ctx.xplane
            .set_float(&self.start.mixture_lever, self.start.mixture_start_notch)
            .await?;
        let on_ground = ctx.xplane.get_float(ONGROUND).await.unwrap_or(1.0) > 0.5;
        if on_ground {
            ctx.xplane
                .set_float(&throttle_dref(), self.start.throttle_crack)
                .await?;
        }
        ctx.xplane.run_command(&self.start.mags_both).await?;

        // Crank each starter, then wait for that engine to catch.
        let mut started = 0usize;
        for (i, starter) in self.start.starters.iter().enumerate() {
            ctx.xplane
                .run_command_for(starter, self.start.crank_secs)
                .await?;
            let mut caught = false;
            for _ in 0..8 {
                tokio::time::sleep(Duration::from_millis(700)).await;
                if engine_running(ctx, i).await {
                    caught = true;
                    break;
                }
            }
            if caught {
                started += 1;
                // Bring this engine's generator online if configured.
                if let Some(gen) = self.start.generators.get(i) {
                    let _ = ctx.xplane.run_command(gen).await;
                }
            }
        }

        match started {
            0 => Ok("Cranking — nothing caught. Check the fuel selectors and mixture, then try again.".to_string()),
            s if s == n => Ok(format!(
                "Roger — fuel to the mains, mixture rich, mags both, cranking. All {n} running, generators on."
            )),
            s => Ok(format!(
                "Got {s} of {n} running. The other didn't catch — check its fuel selector and try again."
            )),
        }
    }
}

pub struct StopEngines {
    start: PistonStartProfile,
}

impl StopEngines {
    pub fn new(start: PistonStartProfile) -> Self {
        Self { start }
    }
}

#[async_trait]
impl Tool for StopEngines {
    fn name(&self) -> &str {
        "stop_engines"
    }
    fn description(&self) -> &str {
        "Shut down the engines: mixtures to idle cutoff and magnetos off. Use for \
         'shut down the engines', 'engines off', 'kill the engines', 'shutdown', \
         'cut the engines', 'secure the engines', 'shut her down'."
    }
    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        // Don't shut engines down in flight or while still rolling.
        if ctx.xplane.get_float(ONGROUND).await? < 0.5 {
            return Ok(
                "Negative — we're airborne. I won't shut the engines down in flight.".to_string(),
            );
        }
        if ctx.xplane.get_float(GROUNDSPEED).await? > 1.0 {
            return Ok("Negative — still moving. Bring her to a stop first.".to_string());
        }
        ctx.xplane
            .set_float(&self.start.mixture_lever, self.start.mixture_cutoff_notch)
            .await?;
        ctx.xplane.run_command(&self.start.mags_off).await?;

        for _ in 0..6 {
            tokio::time::sleep(Duration::from_millis(600)).await;
            if !engine_running(ctx, 0).await {
                return Ok(
                    "Roger — mixtures to idle cutoff, magnetos off. Engines shut down.".to_string(),
                );
            }
        }
        Ok("Mixtures to cutoff, engines winding down.".to_string())
    }
}

// ----------------------------------------------------------------------------
// Turboprop start / shutdown (switch + condition-lever driven; Laminar King Air)
// ----------------------------------------------------------------------------

async fn n1_percent(ctx: &ToolCtx<'_>, idx: usize) -> f64 {
    ctx.xplane
        .get_float_array(N1_PERCENT)
        .await
        .ok()
        .and_then(|v| v.get(idx).copied())
        .unwrap_or(0.0)
}

/// Bring the APU up and open its bleed, so the engines have something to be
/// motored by. `Err` carries the line to speak when it doesn't come up.
async fn start_apu(ctx: &ToolCtx<'_>, apu: &crate::aircraft::ApuStartProfile) -> Result<(), String> {
    let running = ctx
        .xplane
        .get_float(&apu.n1_dataref)
        .await
        .unwrap_or(0.0);
    if running < apu.ready_n1 {
        // The master is a toggle on every airframe seen so far: read it, and
        // fire only if it's actually off. Blind toggling switches OFF an APU
        // that a previous attempt already started.
        let master_on = ctx
            .xplane
            .get_float(&apu.master_readback)
            .await
            .unwrap_or(0.0)
            > 0.5;
        if !master_on {
            let _ = ctx.xplane.run_command(&apu.master_toggle).await;
            tokio::time::sleep(Duration::from_millis(800)).await;
        }
        for cmd in &apu.starter_commands {
            let _ = ctx.xplane.run_command_for(cmd, 1.5).await;
            tokio::time::sleep(Duration::from_millis(700)).await;
        }
        let deadline = (apu.ready_secs / 1.0) as u32;
        let mut up = false;
        for _ in 0..deadline {
            tokio::time::sleep(Duration::from_secs(1)).await;
            if ctx.xplane.get_float(&apu.n1_dataref).await.unwrap_or(0.0) >= apu.ready_n1 {
                up = true;
                break;
            }
        }
        if !up {
            return Err(
                "The APU didn't come up, so there's no bleed air to turn the engines. Your aircraft."
                    .to_string(),
            );
        }
    }
    // Generator and bleed: the engines need the air, and the left starter needs
    // the bus the APU generator powers.
    for cmd in &apu.after_ready_commands {
        let _ = ctx.xplane.run_command(cmd).await;
        tokio::time::sleep(Duration::from_millis(400)).await;
    }
    tokio::time::sleep(Duration::from_secs(2)).await;
    Ok(())
}

async fn n2_percent(ctx: &ToolCtx<'_>, idx: usize) -> f64 {
    ctx.xplane
        .get_float_array(N2_PERCENT)
        .await
        .ok()
        .and_then(|v| v.get(idx).copied())
        .unwrap_or(0.0)
}

pub struct StartTurbines {
    start: TurbineStartProfile,
}

impl StartTurbines {
    pub fn new(start: TurbineStartProfile) -> Self {
        Self { start }
    }
}

#[async_trait]
impl Tool for StartTurbines {
    fn name(&self) -> &str {
        "start_engines"
    }
    fn description(&self) -> &str {
        "Start the engines: runs the full turboprop start (condition levers to low \
         idle, then for each engine engage the starter, wait for it to light and \
         reach idle, release the starter, bring its generator on, and finally \
         condition levers to run). Use for 'start the engines', 'start her up', \
         'engines start', 'engines on', 'start up', 'fire them up', 'spool them up'."
    }
    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let n = self.start.start_up.len();
        // Start only the DEAD engines: a twin with one flamed out must relight just
        // that one (this used to refuse outright if ANY engine was turning — seen
        // live with the left dead and the right at 93%). Running engines are left
        // completely alone, condition levers included.
        let mut dead: Vec<usize> = Vec::new();
        for i in 0..n {
            if !engine_running(ctx, i).await {
                dead.push(i);
            }
        }
        if dead.is_empty() {
            return Ok("Engines are already running.".to_string());
        }
        // Need battery/bus power to crank (the generators come online as we go).
        let volts = ctx
            .xplane
            .get_float_array(BUS_VOLTS)
            .await?
            .into_iter()
            .fold(0.0_f64, f64::max);
        if volts < 12.0 {
            return Ok(format!(
                "No bus power, only {volts:.0} volts — battery on (or ground power) before starting."
            ));
        }

        // A turbofan is motored by BLEED AIR: without the APU running and its
        // bleed open, the starter command is accepted, the annunciator lights,
        // ignition_key goes to 4 — and the engine does not turn at all. Proven
        // live on the Citation X (2026-07-28): APU off, N2 flat at zero; APU
        // generator and bleed on, the same command spooled N2 0 -> 30% in
        // fifteen seconds. Its generator also powers the bus the left starter's
        // own gate reads, which batteries alone leave dead.
        if let Some(apu) = &self.start.apu {
            if let Err(why) = start_apu(ctx, apu).await {
                return Ok(why);
            }
        }

        let mut started = 0usize;
        for &i in &dead {
            // POM start items from the FUEL panel + checklist: this engine's boost
            // pump feeds it until its engine-driven pump runs, and auto-ignition is
            // armed before cranking.
            if self.start.boost_pumps {
                let _ = ctx.xplane.set_int_array_elem(FUEL_PUMP_ON, i, 1).await;
            }
            if let Some(arm) = self.start.ignition_arm.get(i) {
                let _ = ctx.xplane.run_command(arm).await;
            }
            // POM ground-start order — verified live BOTH ways on the C90: fuel in
            // before cranking = no light-off (ITT falls, N1 stalls ~24%); condition
            // CUTOFF -> crank -> fuel (low idle) at 12% N1 -> lights every time.
            ctx.xplane
                .set_float_array_elem(&self.start.condition_lever, i, 0.0)
                .await?;
            // Held starters are re-armed every poll tick with a short duration
            // (the Web API caps a single activation well under a PT6's crank
            // time); once we stop re-arming, the button releases itself.
            if self.start.starter_held {
                ctx.xplane
                    .run_command_for(&self.start.start_up[i], 2.0)
                    .await?;
            } else {
                ctx.xplane.run_command(&self.start.start_up[i]).await?; // starter on
            }
            let deadline = (self.start.crank_secs / 0.7) as u32;
            let mut fuel_in = false;
            let mut lit = false;
            for _ in 0..deadline {
                tokio::time::sleep(Duration::from_millis(700)).await;
                if self.start.starter_held {
                    let _ = ctx
                        .xplane
                        .run_command_for(&self.start.start_up[i], 2.0)
                        .await;
                }
                let n1 = n1_percent(ctx, i).await;
                // Fuel goes in on the spool the AIRFRAME's own procedure uses:
                // N2 on a turbofan, N1 on a free turbine. The Citation X cuts
                // its own starter at 40% N2, which it reaches with N1 barely
                // off the stop — waiting for 12% N1 there means the starter
                // quits before fuel is ever introduced (2026-07-28).
                let (spool, intro) = match self.start.fuel_intro_n2 {
                    Some(n2) => (n2_percent(ctx, i).await, n2),
                    None => (n1, FUEL_INTRO_N1),
                };
                if !fuel_in && spool >= intro {
                    ctx.xplane
                        .set_float_array_elem(
                            &self.start.condition_lever,
                            i,
                            self.start.condition_start,
                        )
                        .await?;
                    fuel_in = true;
                }
                if fuel_in && n1 >= self.start.n1_idle && engine_running(ctx, i).await {
                    lit = true;
                    break;
                }
            }
            if let Some(dn) = self.start.start_dn.get(i) {
                ctx.xplane.run_command(dn).await?; // starter off (held starters release themselves)
            }
            if lit {
                started += 1;
                ctx.xplane
                    .set_float_array_elem(&self.start.condition_lever, i, self.start.condition_run)
                    .await?;
                // Generator online so the bus can power the next start. The list is
                // per-engine GROUPS (the C90's 3-pos switches need a dn,up pair per
                // engine), so fire this engine's slice, not a single command.
                let per = (self.start.generators.len() / n.max(1)).max(1);
                for cmd in self.start.generators.iter().skip(i * per).take(per) {
                    let _ = ctx.xplane.run_command(cmd).await;
                }
            } else {
                // Didn't catch — back to cutoff rather than leaving fuel flowing
                // into a dead engine.
                ctx.xplane
                    .set_float_array_elem(&self.start.condition_lever, i, 0.0)
                    .await?;
            }
        }

        match started {
            0 => Ok(
                "Cranking — nothing lit. Check the condition levers and bus power, then try again."
                    .to_string(),
            ),
            s if s == dead.len() && dead.len() == n => Ok(format!(
                "Roger — starters cut at idle, generators on. All {n} running."
            )),
            s if s == dead.len() => Ok(format!("Roger — relit {s}, all engines running.")),
            s => Ok(format!(
                "Got {s} of {} lit. The other didn't catch — check it and try again.",
                dead.len()
            )),
        }
    }
}

pub struct StopTurbines {
    start: TurbineStartProfile,
}

impl StopTurbines {
    pub fn new(start: TurbineStartProfile) -> Self {
        Self { start }
    }
}

#[async_trait]
impl Tool for StopTurbines {
    fn name(&self) -> &str {
        "stop_engines"
    }
    fn description(&self) -> &str {
        "Shut down the engines: condition levers to fuel cutoff. Use for 'shut down \
         the engines', 'engines off', 'kill the engines', 'shutdown', 'cut the \
         engines', 'secure the engines', 'shut her down'."
    }
    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        if ctx.xplane.get_float(ONGROUND).await? < 0.5 {
            return Ok(
                "Negative — we're airborne. I won't shut the engines down in flight.".to_string(),
            );
        }
        if ctx.xplane.get_float(GROUNDSPEED).await? > 1.0 {
            return Ok("Negative — still moving. Bring her to a stop first.".to_string());
        }
        let n = self.start.start_up.len();
        for i in 0..n {
            ctx.xplane
                .set_float_array_elem(&self.start.condition_lever, i, 0.0)
                .await?; // fuel cutoff
        }
        for _ in 0..6 {
            tokio::time::sleep(Duration::from_millis(600)).await;
            if !engine_running(ctx, 0).await {
                return Ok(
                    "Roger — condition levers to fuel cutoff. Engines shut down.".to_string(),
                );
            }
        }
        Ok("Condition levers to cutoff, engines winding down.".to_string())
    }
}

#[cfg(test)]
mod tests {
    /// Fuel goes in on the spool the AIRFRAME uses. The sequence was written
    /// against a PT6 (12% N1); the Citation X's own script cuts its starter at
    /// 40% N2, which an AE3007 reaches with N1 barely off the stop — so waiting
    /// for 12% N1 there means the starter quits before fuel is ever introduced
    /// and the engine never lights (2026-07-28).
    /// A turbofan is motored by BLEED AIR. Without the APU the starter is
    /// accepted and the engine does not turn — proven live on the Citation X
    /// (2026-07-28): N2 flat at zero with the APU off, 0 to 30% in fifteen
    /// seconds with its generator and bleed on. "Cranking, nothing lit" was
    /// the sequence doing everything right with no air to do it with.
    #[test]
    fn a_turbofan_start_brings_the_apu_up_first() {
        let p = crate::aircraft::load("citation_x").expect("citation_x loads");
        let s = p.turboprop_start.expect("start sequence");
        let apu = s.apu.expect("a jet start needs the APU");
        assert!(!apu.starter_commands.is_empty());
        assert!(apu.ready_n1 >= 90.0, "wait for a RUNNING APU: {}", apu.ready_n1);
        // Generator AND bleed: the air motors the engine, the generator powers
        // the bus the left starter's own gate reads.
        assert_eq!(apu.after_ready_commands.len(), 2, "generator and bleed");
        // Free turbines have no APU step at all.
        for id in ["dhc6_twin_otter", "king_air_c90b", "lancair_evolution"] {
            let t = crate::aircraft::load(id)
                .and_then(|a| a.turboprop_start)
                .unwrap_or_else(|| panic!("{id} start"));
            assert!(t.apu.is_none(), "{id} needs no APU");
        }
    }

    #[test]
    fn a_turbofan_introduces_fuel_on_n2_not_n1() {
        let p = crate::aircraft::load("citation_x").expect("citation_x loads");
        let s = p.turboprop_start.expect("declares a start sequence");
        assert_eq!(s.fuel_intro_n2, Some(10.0));
        // Held, not pressed: cmd_starter_* ignores anything under 0.1 s, so an
        // ordinary command press does nothing at all on this airframe.
        assert!(s.starter_held, "an instantaneous press is a no-op here");
        // Idle N1 has to be reachable — the PT6 default of 50 never arrives.
        assert!(s.n1_idle < 30.0, "{}", s.n1_idle);
        // And every free turbine keeps the behaviour it was tuned on.
        for id in ["dhc6_twin_otter", "king_air_c90b", "lancair_evolution"] {
            let t = crate::aircraft::load(id)
                .and_then(|a| a.turboprop_start)
                .unwrap_or_else(|| panic!("{id} start"));
            assert_eq!(t.fuel_intro_n2, None, "{id} still watches N1");
        }
    }
}
