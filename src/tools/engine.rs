//! Engine controls: `set_throttle` and `set_mixture`.
//!
//! Both are continuous 0.0–1.0 ratios in X-Plane; we expose them as 0–100 % so
//! the pilot can say "throttle 60 percent" / "mixture full rich".

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx, PARK_BRAKE};
use crate::aircraft::FuelSelectorProfile;

const THROTTLE: &str = "sim/cockpit2/engine/actuators/throttle_ratio_all";
const MIXTURE: &str = "sim/cockpit2/engine/actuators/mixture_ratio_all";
const BATTERY: &str = "sim/cockpit/electrical/battery_on";
const BUS_VOLTS: &str = "sim/cockpit2/electrical/bus_volts";
/// Below this bus voltage the battery is too flat to reliably crank the starter
/// (the 172 is a 24V system; a healthy pre-start bus sits ~24-25V). Drops to the
/// low 22s after accessories run with the engine off.
const MIN_START_VOLTS: f64 = 23.0;
const ONGROUND: &str = "sim/flightmodel/failures/onground_any";
const GROUNDSPEED: &str = "sim/flightmodel/position/groundspeed"; // m/s
const ENGN_RUNNING: &str = "sim/flightmodel/engine/ENGN_running";
const CMD_MAGS_BOTH: &str = "sim/magnetos/magnetos_both";
const CMD_MAGS_OFF: &str = "sim/magnetos/magnetos_off";
const CMD_STARTER: &str = "sim/starters/engage_starter_1";
// Alternator (generator #1): the ALT half of the master switch. Brought online
// as part of the start so the running engine charges the bus — and so the
// monitor's "alternator offline" check doesn't fire the instant we finish.
const CMD_ALTERNATOR_ON: &str = "sim/electrical/generator_1_on";
const GENERATOR: &str = "sim/cockpit/electrical/generator_on"; // array; [0] = alternator
// Per-engine actuator arrays (the 172 is single-engine → element 0).
const FUEL_PUMP: &str = "sim/cockpit2/engine/actuators/fuel_pump_on";
const IGNITION_KEY: &str = "sim/cockpit2/engine/actuators/ignition_key"; // 0 off,1 R,2 L,3 both,4 start
// Per-engine magneto selector (0 off,1 R,2 L,3 both) — for radials whose mags
// aren't the single `ignition_key` (e.g. the DC-3, one selector per engine).
const IGNITION_ON: &str = "sim/cockpit/engine/ignition_on";
const CARB_HEAT: &str = "sim/cockpit2/engine/actuators/carb_heat_ratio"; // 0 cold .. 1 hot

/// Write the same value to elements `0..engines` of an int array dataref — for
/// per-engine accessories on a multi (carb heat, boost pump, magnetos). A single
/// engine writes just element 0, exactly as before.
async fn set_each_engine(ctx: &ToolCtx<'_>, dataref: &str, engines: usize, value: i64) -> Result<()> {
    for i in 0..engines.max(1) {
        ctx.xplane.set_int_array_elem(dataref, i, value).await?;
    }
    Ok(())
}
// Engine indicators (per-engine arrays; element 0). Names verified against the
// stock 172's cockpitdecks mapping.
const EGT_CEL: &str = "sim/cockpit2/engine/indicators/EGT_deg_cel";
const PROP_RPM: &str = "sim/cockpit2/engine/indicators/prop_speed_rpm";
const FUEL_FLOW_KG_S: &str = "sim/cockpit2/engine/indicators/fuel_flow_kg_sec";
const ALT_FT: &str = "sim/cockpit2/gauges/indicators/altitude_ft_pilot";
// Above this altitude (ft MSL) a normally-aspirated engine makes well under 75%
// power even at full throttle, so leaning to peak EGT is safe and called for;
// below it we keep the mixture rich (the takeoff/climb/pattern regime). Mirrors
// the POH's "as altitude increases, the pilot leans the mixture".
const LEAN_FLOOR_ALT_FT: f64 = 3000.0;
// Auto-lean never pulls the mixture below this — a guard against an over-lean that
// would stumble the engine; real cruise leaning stays well rich of idle-cutoff.
const MIX_LEAN_FLOOR: f64 = 0.30;
// Avgas mass per US gallon (~6.0 lb/gal), to turn fuel flow (kg/s) into gal/hr.
const AVGAS_KG_PER_GAL: f64 = 2.72;

/// First element of a per-engine array dataref (engine 1), or `default`.
async fn engine0(ctx: &ToolCtx<'_>, dataref: &str, default: f64) -> f64 {
    ctx.xplane
        .get_float_array(dataref)
        .await
        .ok()
        .and_then(|v| v.first().copied())
        .unwrap_or(default)
}

#[derive(Deserialize)]
struct PercentArg {
    percent: f64,
}

/// Drive a 0.0–1.0 actuator smoothly from its current value to `to`, like a hand
/// easing a lever, rather than snapping it. The travel time scales with the size
/// of the change (a small nudge is quick; a full idle→full sweep takes ~1.5s),
/// and a smoothstep ease makes the start and stop of the motion feel natural.
async fn ramp_actuator(ctx: &ToolCtx<'_>, dataref: &str, to: f64) -> Result<()> {
    const STEP_MS: u64 = 50;
    const FULL_SWEEP_MS: f64 = 1500.0; // time for a full 0.0→1.0 sweep
    const MIN_MS: f64 = 150.0;

    let from = ctx.xplane.get_float(dataref).await?.clamp(0.0, 1.0);
    let to = to.clamp(0.0, 1.0);
    let delta = (to - from).abs();

    // Negligible change: just set it and skip the animation.
    if delta < 0.01 {
        ctx.xplane.set_float(dataref, to).await?;
        return Ok(());
    }

    let steps = ((FULL_SWEEP_MS * delta).max(MIN_MS) / STEP_MS as f64).ceil() as u32;
    for i in 1..=steps {
        let t = i as f64 / steps as f64;
        let eased = t * t * (3.0 - 2.0 * t); // smoothstep
        ctx.xplane.set_float(dataref, from + (to - from) * eased).await?;
        tokio::time::sleep(std::time::Duration::from_millis(STEP_MS)).await;
    }
    // Land exactly on the target so float rounding can't leave it a hair short.
    ctx.xplane.set_float(dataref, to).await?;
    Ok(())
}

pub struct SetThrottle;

#[derive(Deserialize)]
struct ThrottleArgs {
    /// Absolute setting, 0-100.
    #[serde(default)]
    percent: Option<f64>,
    /// RELATIVE change, e.g. -10 for "reduce throttle by ten percent" — applied
    /// to the current lever. Wins over `percent` when both are present.
    #[serde(default)]
    delta: Option<f64>,
}

#[async_trait]
impl Tool for SetThrottle {
    fn name(&self) -> &str {
        "set_throttle"
    }
    fn description(&self) -> &str {
        "Set the throttle (engine power / thrust). Absolute: percent, 0 (idle) to \
         100 (full) — 'throttle 60 percent', 'full throttle', 'power to 75'. \
         RELATIVE: delta, signed — 'reduce throttle by ten percent' is delta -10, \
         'add twenty percent power' is delta 20. Never map a 'by N' request to \
         percent N."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "percent": {"type": "integer", "minimum": 0, "maximum": 100,
                            "description": "absolute: 0 = idle, 100 = full power"},
                "delta": {"type": "integer", "minimum": -100, "maximum": 100,
                          "description": "relative change in percent (negative reduces)"}
            }
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: ThrottleArgs = serde_json::from_value(args.clone())?;
        // A relative request adjusts the CURRENT lever — "reduce by ten" from 45%
        // goes to 35%, never to an absolute 10% (a power chop, seen live).
        let target_pct = if let Some(d) = a.delta {
            ctx.xplane.get_float(THROTTLE).await? * 100.0 + d
        } else if let Some(p) = a.percent {
            p
        } else {
            return Ok("Say a throttle setting — a percent, or by how much to change it.".to_string());
        };
        let ratio = (target_pct / 100.0).clamp(0.0, 1.0);
        // Ease the lever to the target instead of snapping it — feels like a real
        // hand on the throttle.
        ramp_actuator(ctx, THROTTLE, ratio).await?;
        let actual = ctx.xplane.get_float(THROTTLE).await? * 100.0;
        Ok(format!("Roger, throttle {actual:.0} percent."))
    }
}

pub struct SetMixture;

#[async_trait]
impl Tool for SetMixture {
    fn name(&self) -> &str {
        "set_mixture"
    }
    fn description(&self) -> &str {
        "Set the fuel mixture to an explicit percentage: 100 = full rich, 0 = idle \
         cutoff, intermediate values lean the mixture. Use for 'mixture full rich', \
         'lean the mixture to 70 percent', 'mixture idle cutoff', 'richen the \
         mixture', 'enrich the mixture', 'mixture 80 percent', 'mixture cutoff'. \
         For automatic/optimal leaning (peak EGT, max RPM, by phase) with no number, \
         use lean_mixture instead."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "percent": {"type": "integer", "minimum": 0, "maximum": 100,
                            "description": "100 = full rich, 0 = idle cutoff"}
            },
            "required": ["percent"]
        })
    }
    fn confirm_prompt(&self, args: &Value) -> Option<String> {
        let p: PercentArg = serde_json::from_value(args.clone()).ok()?;
        // Only idle cutoff (0) is gated — it stops the engine. Leaning is routine
        // and executes immediately.
        if p.percent <= 0.0 {
            Some("mixture to idle cutoff — that will stop the engine".to_string())
        } else {
            None
        }
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let p: PercentArg = serde_json::from_value(args.clone())?;
        // Safety gate: don't cut the mixture to idle cutoff in flight — that stops
        // the engine. (On the ground it's a normal shutdown, confirmed separately.)
        // Default to airborne if the read fails, so an unknown state errs safe.
        if p.percent <= 0.0
            && ctx.xplane.get_float("sim/flightmodel/failures/onground_any").await.unwrap_or(0.0) <= 0.5
        {
            return Ok("Negative, I won't cut the mixture in flight.".to_string());
        }
        let ratio = (p.percent / 100.0).clamp(0.0, 1.0);
        // Ease the mixture lever to the target instead of snapping it.
        ramp_actuator(ctx, MIXTURE, ratio).await?;
        let actual = ctx.xplane.get_float(MIXTURE).await? * 100.0;
        Ok(format!("Roger, mixture {actual:.0} percent."))
    }
}

/// Draw the mixture leaner in a smooth, continuous creep from a rich start,
/// sampling `metric` (EGT or RPM) as it goes, until it peaks (drops `drop_margin`
/// below the best seen) or the lean floor is reached. Returns the mixture at the
/// observed peak and the peak value. Approaching from the rich side means the
/// metric rises to the peak then falls, which is what makes the peak detectable.
///
/// Done the way a pilot does it: glide the lever a small amount, hold and watch
/// the gauge, repeat — slow enough that the metric actually responds and the peak
/// is real, not a reading that's lagging the lever. Each step eases smoothly
/// (like the throttle), then holds long enough for the metric to settle before
/// it's sampled. The long start hold lets the rich baseline settle so it can't
/// masquerade as the peak.
async fn seek_metric_peak(ctx: &ToolCtx<'_>, metric: &str, drop_margin: f64) -> Result<(f64, f64)> {
    const STEP: f64 = 0.02;
    const SETTLE_MS: u64 = 900; // hold to let the metric respond before reading
    const START_SETTLE_MS: u64 = 1500;
    // Lean a few steps past the rich start before a drop can end it (clears the
    // rich plateau), then require the metric to stay down to confirm the peak.
    const MIN_STEPS_BEFORE_STOP: u32 = 3;
    const CONSEC_DROPS_TO_STOP: u32 = 2;

    ramp_actuator(ctx, MIXTURE, 0.95).await?; // ease to the rich start
    tokio::time::sleep(std::time::Duration::from_millis(START_SETTLE_MS)).await;

    let mut mix = 0.95;
    let mut best_mix = mix;
    let mut best_val = engine0(ctx, metric, 0.0).await;
    let mut steps = 0u32;
    let mut drops = 0u32;
    while mix > MIX_LEAN_FLOOR {
        mix = (mix - STEP).max(MIX_LEAN_FLOOR);
        ramp_actuator(ctx, MIXTURE, mix).await?; // smooth glide to the next step
        tokio::time::sleep(std::time::Duration::from_millis(SETTLE_MS)).await; // watch the gauge
        let val = engine0(ctx, metric, 0.0).await;
        steps += 1;
        if val > best_val {
            best_val = val;
            best_mix = mix;
            drops = 0;
        } else if best_val - val >= drop_margin {
            drops += 1;
            if steps > MIN_STEPS_BEFORE_STOP && drops >= CONSEC_DROPS_TO_STOP {
                break; // confirmed past the peak
            }
        } else {
            drops = 0;
        }
    }
    Ok((best_mix, best_val))
}

pub struct LeanMixture;

#[derive(Deserialize, Default)]
struct LeanArgs {
    /// auto | economy | power. Absent = auto.
    #[serde(default)]
    mode: Option<String>,
}

#[async_trait]
impl Tool for LeanMixture {
    fn name(&self) -> &str {
        "lean_mixture"
    }
    fn description(&self) -> &str {
        "Automatically set the correct fuel mixture for the current phase of flight \
         — the smart alternative to set_mixture (which takes an explicit percentage). \
         It leans to peak EGT for cruise economy, leans for maximum RPM when power is \
         asked for, and keeps the mixture full rich below 3000 feet. Use for 'lean \
         the mixture', 'lean for cruise', 'set the mixture for cruise', 'lean it out', \
         'lean to peak EGT', 'lean for max power', 'lean for max RPM', 'set an \
         appropriate mixture', 'sort out the mixture'. For an exact percentage use \
         set_mixture instead."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "mode": {"type": "string", "enum": ["auto", "economy", "power"],
                         "description": "auto = pick by altitude (default); economy = lean to peak EGT for cruise; power = lean for max RPM (climb / high-elevation takeoff)"}
            }
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: LeanArgs = serde_json::from_value(args.clone()).unwrap_or_default();
        let mode = a.mode.unwrap_or_else(|| "auto".into()).to_ascii_lowercase();

        if engine0(ctx, ENGN_RUNNING, 0.0).await <= 0.5 {
            return Ok("Engine's not running — nothing to lean.".to_string());
        }
        let alt = ctx.xplane.get_float(ALT_FT).await.unwrap_or(0.0);

        // Resolve `auto` by altitude: rich down low, lean to economy up in the cruise.
        let resolved = match mode.as_str() {
            "economy" => "economy",
            "power" => "power",
            _ => if alt >= LEAN_FLOOR_ALT_FT { "economy" } else { "rich" },
        };

        match resolved {
            "rich" => {
                ramp_actuator(ctx, MIXTURE, 1.0).await?;
                Ok(format!(
                    "Below {LEAN_FLOOR_ALT_FT:.0} feet, mixture full rich. I'll lean for cruise once we're higher."
                ))
            }
            "economy" => {
                // Don't lean hard at high power down low (detonation risk). Up high,
                // full throttle is still under 75% power, so leaning is correct.
                if alt < LEAN_FLOOR_ALT_FT && ctx.xplane.get_float(THROTTLE).await.unwrap_or(0.0) > 0.80 {
                    return Ok(
                        "We're at high power down low, I'll hold off leaning; \
                         leaning hard there risks detonation. Reduce to cruise power or climb first."
                            .to_string(),
                    );
                }
                // Lean to peak EGT, then enrich slightly so EGT sits just rich of
                // peak — the safe side the POH calls for ("slightly below the peak
                // observed EGT").
                let (best, egt) = seek_metric_peak(ctx, EGT_CEL, 12.0).await?;
                if best > 0.92 {
                    // Never saw EGT climb — low power, or the engine isn't loaded
                    // enough to lean. Leave it rich rather than report a bogus value.
                    ramp_actuator(ctx, MIXTURE, 1.0).await?;
                    return Ok("Couldn't find a clear EGT peak, leaving the mixture rich. Try leaning at cruise power.".to_string());
                }
                ramp_actuator(ctx, MIXTURE, (best + 0.03).min(1.0)).await?;
                let pct = ctx.xplane.get_float(MIXTURE).await? * 100.0;
                let gph = engine0(ctx, FUEL_FLOW_KG_S, 0.0).await * 3600.0 / AVGAS_KG_PER_GAL;
                let egt_f = egt * 9.0 / 5.0 + 32.0; // dataref is °C
                Ok(format!(
                    "Leaned to peak EGT, mixture {pct:.0} percent, EGT {egt_f:.0} degrees, burning {gph:.1} gallons an hour."
                ))
            }
            _ => {
                // power: lean for max RPM (the high-DA takeoff / climb procedure).
                let (best, rpm) = seek_metric_peak(ctx, PROP_RPM, 25.0).await?;
                ramp_actuator(ctx, MIXTURE, best).await?;
                let pct = ctx.xplane.get_float(MIXTURE).await? * 100.0;
                Ok(format!("Leaned for max power — {rpm:.0} RPM at mixture {pct:.0} percent."))
            }
        }
    }
}

pub struct StartEngine;

#[async_trait]
impl Tool for StartEngine {
    fn name(&self) -> &str {
        "start_engine"
    }
    fn description(&self) -> &str {
        "Start the engine: runs the start sequence (battery on, mixture rich, \
         throttle cracked, magnetos both, engage the starter). Use for 'start the \
         engine', 'start her up', 'engine start', 'engine on', 'engines on', \
         'switch the engine on', 'turn the engine on'. Note: 'engine on' means \
         START the engine, not an electrical switch."
    }
    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        // Already running? Don't re-crank a live engine.
        let already_running = ctx
            .xplane
            .get_float_array(ENGN_RUNNING)
            .await
            .ok()
            .and_then(|v| v.first().copied())
            .map(|r| r > 0.5)
            .unwrap_or(false);
        if already_running {
            return Ok("Engine's already running.".to_string());
        }

        // Battery check: the battery must be on, and with enough voltage to crank.
        // Cranking a flat battery just drains it further without catching.
        ctx.xplane.set_int(BATTERY, 1).await?;
        let volts = ctx.xplane.get_float_array(BUS_VOLTS).await?.first().copied().unwrap_or(0.0);
        if volts < MIN_START_VOLTS {
            return Ok(format!(
                "Battery's low, only {volts:.0} volts — connect ground power or you may not get a start."
            ));
        }

        // Pre-start configuration.
        ctx.xplane.set_float(MIXTURE, 1.0).await?; // full rich
        ctx.xplane.set_float(THROTTLE, 0.1).await?; // crack the throttle
        ctx.xplane.run_command(CMD_MAGS_BOTH).await?;

        // Crank the starter (held a few seconds so it can catch).
        ctx.xplane.run_command_for(CMD_STARTER, 3.0).await?;

        // Give it a moment to fire, then confirm engine 1 is running.
        for _ in 0..6 {
            tokio::time::sleep(std::time::Duration::from_millis(700)).await;
            let running = ctx
                .xplane
                .get_float_array(ENGN_RUNNING)
                .await
                .ok()
                .and_then(|v| v.first().copied())
                .map(|r| r > 0.5)
                .unwrap_or(false);
            if running {
                // Bring the alternator online now there's an engine to drive it,
                // so we charge the bus instead of draining the battery (and so
                // the monitor doesn't immediately call out an offline alternator).
                ctx.xplane.run_command(CMD_ALTERNATOR_ON).await?;
                let alt_on = ctx
                    .xplane
                    .get_float_array(GENERATOR)
                    .await
                    .ok()
                    .and_then(|v| v.first().copied())
                    .map(|g| g > 0.5)
                    .unwrap_or(false);
                // Bring the avionics master online now the engine's running and
                // the alternator is charging — matches the STARTING ENGINE
                // checklist (avionics OFF for the start, ON after) and closes the
                // gap where the transponder/radios sat on a dead bus. Commands
                // verified present in the sim.
                // The avionics-master ON command is universal (sim/systems/...),
                // so fire it directly rather than coupling to the (now
                // profile-built) electrical tool.
                let avionics_on = ctx.xplane.run_command("sim/systems/avionics_on").await.is_ok();
                // Narrate the start sequence so the pilot hears what was done on
                // their behalf, not just the end state.
                let alt = if alt_on { "alternator on" } else { "but the alternator didn't come on, check it" };
                let avi = if avionics_on { ", avionics on" } else { "" };
                return Ok(format!(
                    "Roger — mixture rich, throttle cracked, mags both, cranking. \
                     Engine started, {alt}{avi}."
                ));
            }
        }
        Ok("Cranking — the engine hasn't caught. Check fuel and mixture, then try again.".to_string())
    }
}

/// Electric/auxiliary fuel pump(s). `engines` = how many to switch (a twin
/// drives both boost pumps off one "fuel pump on" command).
pub struct SetFuelPump {
    pub engines: usize,
}

#[derive(Deserialize)]
struct OnArg {
    on: bool,
}

#[async_trait]
impl Tool for SetFuelPump {
    fn name(&self) -> &str {
        "set_fuel_pump"
    }
    fn description(&self) -> &str {
        "Turn the electric (auxiliary) fuel pump on or off. Use for 'fuel pump on', \
         'boost pump on', 'fuel pump off', 'aux fuel pump on', 'electric pump on', \
         'auxiliary pump off'."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": { "on": {"type": "boolean", "description": "true = on, false = off"} },
            "required": ["on"]
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: OnArg = serde_json::from_value(args.clone())?;
        set_each_engine(ctx, FUEL_PUMP, self.engines, a.on as i64).await?;
        let on = ctx.xplane.get_float_array(FUEL_PUMP).await?.first().copied().unwrap_or(0.0) > 0.5;
        Ok(format!("Roger, fuel pump {}.", if on { "on" } else { "off" }))
    }
}

/// Carburetor (alternate air) heat. `engines` = how many to switch together.
pub struct SetCarbHeat {
    pub engines: usize,
}

#[async_trait]
impl Tool for SetCarbHeat {
    fn name(&self) -> &str {
        "set_carb_heat"
    }
    fn description(&self) -> &str {
        "Turn carburetor (alternate air) heat on or off. Use for 'carb heat on', \
         'carb heat off', 'carburetor heat on', 'apply carb heat', 'carb heat \
         cold', 'carb heat hot', 'alternate air on'."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": { "on": {"type": "boolean", "description": "true = hot (on), false = cold (off)"} },
            "required": ["on"]
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: OnArg = serde_json::from_value(args.clone())?;
        set_each_engine(ctx, CARB_HEAT, self.engines, a.on as i64).await?;
        let on = ctx.xplane.get_float_array(CARB_HEAT).await?.first().copied().unwrap_or(0.0) > 0.5;
        Ok(format!("Roger, carb heat {}.", if on { "on" } else { "off" }))
    }
}

/// Magneto/ignition selector. `engines` = how many to set together; when
/// `per_engine` the selector is the per-engine `ignition_on` array (the DC-3),
/// otherwise the single `ignition_key` (the GA singles).
pub struct SetMagnetos {
    pub engines: usize,
    pub per_engine: bool,
}

#[derive(Deserialize)]
struct MagArgs {
    /// Magneto position: off, left, right, both.
    position: String,
}

#[async_trait]
impl Tool for SetMagnetos {
    fn name(&self) -> &str {
        "set_magnetos"
    }
    fn description(&self) -> &str {
        "Set the magnetos / ignition position: off, left, right, or both. Used in \
         the run-up mag check. Use for 'magnetos both', 'mags to left', 'mags \
         right', 'magnetos off', 'ignition both', 'check the left mag', \
         'mags to right'. To start the engine use start_engine, not this."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "position": {"type": "string", "enum": ["off", "left", "right", "both"],
                             "description": "Magneto selector position."}
            },
            "required": ["position"]
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: MagArgs = serde_json::from_value(args.clone())?;
        let key = match a.position.to_ascii_lowercase().as_str() {
            "off" => 0,
            "right" => 1,
            "left" => 2,
            "both" => 3,
            other => return Ok(format!("I didn't catch the magneto position '{other}'. Say off, left, right, or both.")),
        };
        let dref = if self.per_engine { IGNITION_ON } else { IGNITION_KEY };
        if self.per_engine {
            set_each_engine(ctx, IGNITION_ON, self.engines, key).await?;
        } else {
            ctx.xplane.set_int_array_elem(IGNITION_KEY, 0, key).await?;
        }
        let actual = ctx.xplane.get_float_array(dref).await?.first().copied().unwrap_or(-1.0).round() as i64;
        let name = match actual { 0 => "off", 1 => "right", 2 => "left", 3 => "both", 4 => "start", _ => "unknown" };
        Ok(format!("Roger, magnetos {name}."))
    }
}

/// The fuel tank selector, built from the airframe profile. The positions differ
/// per airframe (the 172 is LEFT/BOTH/RIGHT; the SR22 is OFF/LEFT/RIGHT, no
/// "both"), so the surface and wiring are config, not code: each position fires a
/// command (commands animate the 3D knob; a raw write may not), then — when the
/// profile gives a readback dataref and the position a known value — the dataref
/// is read to confirm the actual position.
pub struct SetFuelSelector {
    profile: FuelSelectorProfile,
}

impl SetFuelSelector {
    pub fn new(profile: FuelSelectorProfile) -> Self {
        Self { profile }
    }
}

#[derive(Deserialize)]
struct FuelSelArgs {
    /// Selector position token (one of the profile's `positions`).
    position: String,
}

#[async_trait]
impl Tool for SetFuelSelector {
    fn name(&self) -> &str {
        "set_fuel_selector"
    }
    fn description(&self) -> &str {
        // Static base; the available positions are listed in `parameters`.
        "Set the fuel tank selector. Use for 'fuel selector both', 'fuel to left \
         tank', 'select right tank', 'switch tanks', 'change to the left tank', \
         'both tanks', 'feed from the right', 'fuel selector off'."
    }
    fn parameters(&self) -> Value {
        let mut keys: Vec<&str> = self.profile.positions.iter().map(|p| p.key.as_str()).collect();
        keys.push("switch");
        let listing: Vec<String> =
            self.profile.positions.iter().map(|p| format!("{} ({})", p.key, p.spoken)).collect();
        json!({
            "type": "object",
            "properties": {
                "position": {"type": "string", "enum": keys,
                             "description": format!(
                                 "Fuel tank selector position, or 'switch' for the OTHER wing tank. Available: {}.",
                                 listing.join(", "))}
            },
            "required": ["position"]
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: FuelSelArgs = serde_json::from_value(args.clone())?;
        // "switch" = feed from the OTHER wing tank — the routine balance call (the
        // SR22 has no BOTH, so this happens every half hour). Resolved from the
        // current selector; from anything that isn't a wing tank, ask for a side.
        let requested = a.position.to_ascii_lowercase();
        let requested = if matches!(requested.as_str(), "switch" | "swap" | "other") {
            let current = match &self.profile.readback {
                Some(dref) => ctx.xplane.get_float(dref).await.ok().and_then(|actual| {
                    self.profile
                        .positions
                        .iter()
                        .find(|p| p.value.is_some_and(|v| (v - actual).abs() < 0.5))
                        .map(|p| p.key.as_str())
                }),
                None => None,
            };
            match current {
                Some("left") => "right".to_string(),
                Some("right") => "left".to_string(),
                _ => {
                    return Ok(
                        "I can't tell which tank we're feeding from — say left tank or right tank."
                            .to_string(),
                    )
                }
            }
        } else {
            requested
        };
        let Some(pos) = self
            .profile
            .positions
            .iter()
            .find(|p| p.key.eq_ignore_ascii_case(&requested))
        else {
            let names: Vec<&str> = self.profile.positions.iter().map(|p| p.spoken.as_str()).collect();
            return Ok(format!(
                "I didn't catch the fuel position '{}'. I have {}.",
                a.position,
                names.join(", ")
            ));
        };
        // Command so the cockpit knob moves with the fuel logic.
        ctx.xplane.run_command(&pos.command).await?;
        // Confirm from the readback dataref when the position carries a known
        // value; otherwise trust the command.
        if let Some(dref) = &self.profile.readback {
            if let Ok(actual) = ctx.xplane.get_float(dref).await {
                if let Some(hit) = self
                    .profile
                    .positions
                    .iter()
                    .find(|p| p.value.is_some_and(|v| (v - actual).abs() < 0.5))
                {
                    return Ok(format!("Roger, fuel selector {}.", hit.spoken));
                }
            }
        }
        Ok(format!("Roger, fuel selector {}.", pos.spoken))
    }
}

pub struct StopEngine;

#[async_trait]
impl Tool for StopEngine {
    fn name(&self) -> &str {
        "stop_engine"
    }
    fn description(&self) -> &str {
        "Shut down the engine: pull the mixture to idle cutoff and turn the \
         magnetos off. Use for 'shut down the engine', 'engine off', 'kill the \
         engine', 'shutdown', 'cut the engine', 'shut her down', 'secure the \
         engine', 'stop the engine'."
    }
    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        // Shutting the engine down in flight is an engine failure. Rather than
        // ask for confirmation, refuse unless it's actually safe: on the ground,
        // stopped, and parking brake set.
        if ctx.xplane.get_float(ONGROUND).await? < 0.5 {
            return Ok("Negative — we're airborne. I won't shut the engine down in flight.".to_string());
        }
        if ctx.xplane.get_float(GROUNDSPEED).await? > 1.0 {
            return Ok("Negative — still moving. Bring her to a stop first.".to_string());
        }
        if ctx.xplane.get_float(PARK_BRAKE).await? < 0.5 {
            return Ok("Set the parking brake first, then I'll shut the engine down.".to_string());
        }

        ctx.xplane.set_float(MIXTURE, 0.0).await?; // idle cutoff
        ctx.xplane.run_command(CMD_MAGS_OFF).await?;

        for _ in 0..5 {
            tokio::time::sleep(std::time::Duration::from_millis(600)).await;
            let running = ctx
                .xplane
                .get_float_array(ENGN_RUNNING)
                .await
                .ok()
                .and_then(|v| v.first().copied())
                .map(|r| r > 0.5)
                .unwrap_or(false);
            if !running {
                return Ok("Roger — mixture to idle cutoff, magnetos off. Engine shut down.".to_string());
            }
        }
        Ok("Mixture to cutoff, engine winding down.".to_string())
    }
}
