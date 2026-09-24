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
/// Bleed duct pressure, left / centre / right: the air a turbofan's starter
/// turns the engine with.
const BLEED_AVAILABLE: [&str; 3] = [
    "sim/cockpit2/bleedair/indicators/bleed_available_left",
    "sim/cockpit2/bleedair/indicators/bleed_available_center",
    "sim/cockpit2/bleedair/indicators/bleed_available_right",
];
/// A starter that has not moved the spool past this in `NO_TURN_SECS` is not
/// turning the engine, and cranking on is only time.
const NO_TURN_PCT: f64 = 2.0;
const NO_TURN_SECS: f64 = 10.0;
/// After the crank time, how long a lit engine may take to reach idle.
const SPOOL_UP_SECS: u32 = 30;

/// Drive a [`CycleSwitch`] to `target` by reading its position and firing
/// `up`/`down`. Bounded by the detent span so a stuck switch can't loop forever.
/// Returns the final position read back.
pub(crate) async fn cycle_to(ctx: &ToolCtx<'_>, sw: &CycleSwitch, target: i64) -> Result<i64> {
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
        // Start only the DEAD engines: a twin with one flamed out must relight
        // just that one (the old gate refused outright if ANY engine was
        // turning, so a one-engine relight was impossible — the same trap the
        // turbine sequence fixed). Running engines are left completely alone.
        let mut dead: Vec<usize> = Vec::new();
        for i in 0..n {
            if !engine_running(ctx, i).await {
                dead.push(i);
            }
        }
        if dead.is_empty() {
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

        // Crank each DEAD engine's starter, then wait for it to catch. A
        // running engine's starter is never engaged.
        let mut started = 0usize;
        for &i in &dead {
            let Some(starter) = self.start.starters.get(i) else {
                continue;
            };
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
            s if s == dead.len() && dead.len() == n => Ok(format!(
                "Roger — fuel to the mains, mixture rich, mags both, cranking. All {n} running, generators on."
            )),
            s if s == dead.len() => Ok(format!(
                "Roger — relit {s}, all engines running."
            )),
            s => Ok(format!(
                "Got {s} of {} running. The other didn't catch — check its fuel selector and try again.",
                dead.len()
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

        let n = self.start.starters.len();
        for _ in 0..6 {
            tokio::time::sleep(Duration::from_millis(600)).await;
            // EVERY engine must be down before "shut down" is spoken —
            // element 0 alone can read stopped with the other still turning.
            let mut any_running = false;
            for i in 0..n.max(1) {
                if engine_running(ctx, i).await {
                    any_running = true;
                    break;
                }
            }
            if !any_running {
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
async fn start_apu(
    ctx: &ToolCtx<'_>,
    apu: &crate::aircraft::ApuStartProfile,
) -> Result<(), String> {
    let running = ctx.xplane.get_float(&apu.n1_dataref).await.unwrap_or(0.0);
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
        // HOLD the starter until it lights. The switch is spring-loaded and the
        // script arms the starter only past a second of hold, so a press — even
        // a press repeated — buys a fraction of a second of cranking and the
        // APU never catches. Re-armed on a cadence shorter than each hold, so
        // the switch never springs back mid-start.
        let mut lit = false;
        for _ in 0..(apu.ready_secs / 1.5) as u32 {
            for cmd in &apu.starter_commands {
                let _ = ctx.xplane.run_command_for(cmd, 2.0).await;
            }
            tokio::time::sleep(Duration::from_millis(1500)).await;
            if ctx.xplane.get_float(&apu.n1_dataref).await.unwrap_or(0.0) >= apu.light_n1 {
                lit = true;
                break;
            }
        }
        // Lit and self-sustaining: let the switch go and let it spool up.
        let mut up = false;
        if lit {
            for _ in 0..(apu.ready_secs as u32) {
                tokio::time::sleep(Duration::from_secs(1)).await;
                if ctx.xplane.get_float(&apu.n1_dataref).await.unwrap_or(0.0) >= apu.ready_n1 {
                    up = true;
                    break;
                }
            }
        }
        crate::blackbox::record(
            "turbine_start",
            json!({
                "event": "apu", "lit": lit, "up": up,
                "n1": ctx.xplane.get_float(&apu.n1_dataref).await.ok(),
            }),
        );
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
    // READ BACK the air, not the switch: the crank is only worth starting
    // once a duct has pressure in it. Measured on the Citation X with the APU
    // bleed ON: 1.2 to 1.5 in the centre and right ducts (2026-09-22).
    let mut air = bleed_air(ctx).await;
    for _ in 0..BLEED_WAIT_SECS {
        if has_air(&air) {
            break;
        }
        tokio::time::sleep(Duration::from_secs(1)).await;
        air = bleed_air(ctx).await;
    }
    crate::blackbox::record(
        "turbine_start",
        json!({ "event": "apu_loaded", "air": air, "volts": bus_volts(ctx).await }),
    );
    if !has_air(&air) {
        return Err(
            "The APU is running but no bleed air reached the engines. Check the APU bleed switch. \
             Your aircraft."
                .to_string(),
        );
    }
    Ok(())
}

/// Any duct with pressure in it. A failed read is no measurement: all three
/// unreadable is "no answer", which is not a reason to refuse the start.
fn has_air(air: &[Option<f64>]) -> bool {
    air.iter().all(Option::is_none) || air.iter().flatten().any(|&p| p >= BLEED_MIN_PSI)
}

/// Duct pressure that counts as air for the starters, and how long to wait
/// for it after the APU bleed is opened.
const BLEED_MIN_PSI: f64 = 0.5;
const BLEED_WAIT_SECS: u32 = 10;

/// The three bleed ducts' pressure (left, centre, right), for the trace: a
/// turbofan that does not turn on its starter is a question about the air.
/// A failed read is `null`, never a number.
async fn bleed_air(ctx: &ToolCtx<'_>) -> Vec<Option<f64>> {
    let mut out = Vec::with_capacity(BLEED_AVAILABLE.len());
    for d in BLEED_AVAILABLE {
        out.push(ctx.xplane.get_float(d).await.ok());
    }
    out
}

async fn bus_volts(ctx: &ToolCtx<'_>) -> Option<Vec<f64>> {
    ctx.xplane.get_float_array(BUS_VOLTS).await.ok()
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
        let mut verdicts: Vec<(usize, CrankVerdict)> = Vec::new();
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
            let mut peak = 0.0_f64;
            let cranked = std::time::Instant::now();
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
                peak = peak.max(spool);
                // Nothing turning: the starter is not moving this engine, and
                // the full crank was a minute and a half of silence before
                // "nothing lit" (the Citation from a cold APU, 2026-09-22).
                if peak < NO_TURN_PCT && cranked.elapsed().as_secs_f64() >= NO_TURN_SECS {
                    break;
                }
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
            // Lit but still spooling to idle when the crank time ran out: that
            // is a start, not a failure. The Citation's right engine was at
            // 59 % N2 and 17.5 % N1 against an 18 % idle at 31 s, and the
            // fuel was cut on a running engine (EDDV 2026-09-22). It gets its
            // own time to reach idle, and fuel stays in while it runs.
            if fuel_in && !lit && engine_running(ctx, i).await {
                for _ in 0..SPOOL_UP_SECS {
                    tokio::time::sleep(Duration::from_secs(1)).await;
                    if n1_percent(ctx, i).await >= self.start.n1_idle {
                        lit = true;
                        break;
                    }
                }
                // Running and short of the idle figure is still running: the
                // figure is the profile's hypothesis, the flag is the sim's.
                lit = lit || engine_running(ctx, i).await;
            }
            if let Some(dn) = self.start.start_dn.get(i) {
                ctx.xplane.run_command(dn).await?; // starter off (held starters release themselves)
            }
            let verdict = CrankVerdict::judge(peak, fuel_in, lit);
            crate::blackbox::record(
                "turbine_start",
                json!({
                    "event": "crank", "engine": i, "verdict": format!("{verdict:?}"),
                    "peak_spool": peak, "intro": self.start.fuel_intro_n2.unwrap_or(FUEL_INTRO_N1),
                    "spool": if self.start.fuel_intro_n2.is_some() { "n2" } else { "n1" },
                    "n1": n1_percent(ctx, i).await, "secs": cranked.elapsed().as_secs_f64(),
                    "air": bleed_air(ctx).await, "volts": bus_volts(ctx).await,
                }),
            );
            verdicts.push((i, verdict));
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
            0 => Ok(failed_start_line(
                &verdicts,
                n,
                self.start.fuel_intro_n2.is_some(),
            )),
            s if s == dead.len() && dead.len() == n => Ok(format!(
                "Roger — starters cut at idle, generators on. All {n} running."
            )),
            s if s == dead.len() => Ok(format!("Roger — relit {s}, all engines running.")),
            s => Ok(format!(
                "Got {s} of {} lit. {}",
                dead.len(),
                failed_start_line(&verdicts, n, self.start.fuel_intro_n2.is_some())
            )),
        }
    }
}

/// What one engine's crank came to, from what was measured on it.
#[derive(Debug, Clone, Copy, PartialEq)]
enum CrankVerdict {
    Lit,
    /// The starter never moved the spool: no air, or no starter at all.
    NeverTurned,
    /// It turned, but never reached the speed where fuel goes in.
    TooSlowForFuel,
    /// Turned, fuelled, and did not light.
    NoLight,
}

impl CrankVerdict {
    fn judge(peak_spool: f64, fuel_in: bool, lit: bool) -> Self {
        if lit {
            Self::Lit
        } else if fuel_in {
            Self::NoLight
        } else if peak_spool < NO_TURN_PCT {
            Self::NeverTurned
        } else {
            Self::TooSlowForFuel
        }
    }
}

fn engine_name(i: usize, n: usize) -> String {
    match (n, i) {
        (2, 0) => "the left engine".to_string(),
        (2, 1) => "the right engine".to_string(),
        _ => format!("engine {}", i + 1),
    }
}

/// The spoken line for engines that did not start: what each one did, so
/// the pilot hears where it stopped instead of one line for every failure
/// ("Cranking, nothing lit. Check the condition levers" to a Citation whose
/// engines never turned, 2026-09-22). `turbofan` names the fuel levers.
fn failed_start_line(verdicts: &[(usize, CrankVerdict)], n: usize, turbofan: bool) -> String {
    let failed: Vec<(usize, CrankVerdict)> = verdicts
        .iter()
        .copied()
        .filter(|(_, v)| *v != CrankVerdict::Lit)
        .collect();
    let Some(&(_, first)) = failed.first() else {
        return "Nothing lit. Check the fuel and bus power, then try again.".to_string();
    };
    let levers = if turbofan {
        "fuel levers"
    } else {
        "condition levers"
    };
    let turn_fix = if turbofan {
        "Check the bleed air, then try again."
    } else {
        "Check the starters and bus power, then try again."
    };
    let slow_fix = if turbofan { "bleed air" } else { "bus power" };
    // Every engine failed the same way: say it once.
    if failed.len() > 1 && failed.iter().all(|(_, v)| *v == first) {
        let (none, all) = if failed.len() == 2 {
            ("Neither engine", "Both engines")
        } else {
            ("No engine", "All the engines")
        };
        return match first {
            CrankVerdict::NeverTurned => format!("{none} turned on the starter. {turn_fix}"),
            CrankVerdict::TooSlowForFuel => format!(
                "{all} turned but never got fast enough for fuel. \
                 Check the {slow_fix}, then try again."
            ),
            _ => format!(
                "{all} had fuel and did not light. \
                 Check the {levers} and ignition, then try again."
            ),
        };
    }
    failed
        .iter()
        .map(|&(i, v)| {
            let who = cap(&engine_name(i, n));
            match v {
                CrankVerdict::NeverTurned => format!("{who} never turned on the starter. {turn_fix}"),
                CrankVerdict::TooSlowForFuel => format!(
                    "{who} turned but never got fast enough for fuel. Check the {slow_fix}, then try again."
                ),
                _ => format!(
                    "{who} had fuel and did not light. Check the {levers} and ignition, then try again."
                ),
            }
        })
        .collect::<Vec<_>>()
        .join(" ")
}

fn cap(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
        None => String::new(),
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
        // ...then the airframe's own securing control, where it has one. On the
        // SF50 the ignition knob is what fires the sim's shutdown AND drops the
        // boost pump and igniter; cutting the mixture alone leaves the knob at
        // RUN with both still live, which is not a secured aircraft. Best
        // effort: the cutoff above has already stopped the fuel, so a command
        // that fails must not turn a successful shutdown into an error.
        let secured = !self.start.shutdown_commands.is_empty();
        for cmd in &self.start.shutdown_commands {
            let _ = ctx.xplane.run_command(cmd).await;
            // A sequence (knob, then button) is read one step at a time by
            // the aircraft's script: a beat between them.
            tokio::time::sleep(Duration::from_millis(400)).await;
        }
        let done = if secured {
            "Roger — fuel cutoff and the ignition off. Engines shut down and secured."
        } else {
            "Roger — condition levers to fuel cutoff. Engines shut down."
        };
        for _ in 0..20 {
            tokio::time::sleep(Duration::from_millis(600)).await;
            // EVERY engine must be down before "shut down" is spoken —
            // element 0 alone can read stopped with the other still turning.
            let mut any_running = false;
            for i in 0..n.max(1) {
                if engine_running(ctx, i).await {
                    any_running = true;
                    break;
                }
            }
            if !any_running {
                return Ok(done.to_string());
            }
        }
        // Still running after twelve seconds: said as it is, never "winding
        // down" (the SF50's script rewrites the cutoff lever and the engine
        // ran on while the tool claimed success, LSZS 2026-09-24).
        Ok("The engine is still running; the shutdown didn't take. Check the ignition \
            and the start/stop control."
            .to_string())
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
        // Loaded at the aircraft's own READY TO LOAD (CitX_APU.lua lights
        // it at N1 >= 99), not at a spooling 95.
        assert!(
            apu.ready_n1 >= 99.0,
            "wait for a READY APU: {}",
            apu.ready_n1
        );
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

    /// A failed start names where each engine stopped. At EDDV (2026-09-22)
    /// the Citation's engines never turned from a cold APU and the pilot
    /// heard "Cranking, nothing lit. Check the condition levers", a
    /// turboprop's line about fuel, after a minute and a half.
    #[test]
    fn a_failed_start_says_where_each_engine_stopped() {
        use super::{failed_start_line, CrankVerdict as V};
        assert_eq!(V::judge(0.0, false, false), V::NeverTurned);
        assert_eq!(V::judge(8.0, false, false), V::TooSlowForFuel);
        assert_eq!(V::judge(30.0, true, false), V::NoLight);
        assert_eq!(V::judge(64.0, true, true), V::Lit);

        let both = [(0, V::NeverTurned), (1, V::NeverTurned)];
        assert_eq!(
            failed_start_line(&both, 2, true),
            "Neither engine turned on the starter. Check the bleed air, then try again."
        );
        let mixed = [(0, V::Lit), (1, V::NoLight)];
        assert_eq!(
            failed_start_line(&mixed, 2, true),
            "The right engine had fuel and did not light. \
             Check the fuel levers and ignition, then try again."
        );
        // A free turbine keeps its condition levers.
        assert!(failed_start_line(&[(0, V::NoLight)], 1, false).contains("condition levers"));
        // Never a dash: the TTS and the panel font choke on them.
        for v in [V::NeverTurned, V::TooSlowForFuel, V::NoLight] {
            for line in [
                failed_start_line(&[(0, v), (1, v)], 2, true),
                failed_start_line(&[(0, V::Lit), (1, v)], 2, false),
            ] {
                assert!(
                    !line.contains('\u{2014}') && !line.contains('\u{2013}'),
                    "{line}"
                );
            }
        }
    }

    #[test]
    fn the_crank_waits_for_air_in_a_duct() {
        use super::has_air;
        assert!(!has_air(&[Some(0.0), Some(0.0), Some(0.0)]));
        assert!(
            has_air(&[Some(0.0), Some(1.5), Some(1.5)]),
            "APU bleed ON, measured"
        );
        // Unreadable is no verdict: the start goes on and the crank judges it.
        assert!(has_air(&[None, None, None]));
        assert!(!has_air(&[None, Some(0.1), None]));
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
