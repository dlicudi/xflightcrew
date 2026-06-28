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
const CARB_HEAT: &str = "sim/cockpit2/engine/actuators/carb_heat_ratio"; // 0 cold .. 1 hot
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

#[async_trait]
impl Tool for SetThrottle {
    fn name(&self) -> &str {
        "set_throttle"
    }
    fn description(&self) -> &str {
        "Set the throttle (engine power / thrust) as a percentage, 0 (idle) to \
         100 (full power). Use for 'throttle 60 percent', 'full throttle', \
         'maximum power', 'set thrust to 80 percent', 'throttle idle', \
         'power up', 'power to 75', 'throttle back to 40', 'reduce power', \
         'advance the throttle', 'firewall it', 'throttle back'."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "percent": {"type": "integer", "minimum": 0, "maximum": 100,
                            "description": "0 = idle, 100 = full power"}
            },
            "required": ["percent"]
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let p: PercentArg = serde_json::from_value(args.clone())?;
        let ratio = (p.percent / 100.0).clamp(0.0, 1.0);
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
/// The lever moves in tiny direct increments on a steady tick (no per-step ramp or
/// long settle pauses), so it looks like a hand slowly leaning it rather than
/// hopping between detents. The metric lags the lever by ~a second in the sim, so
/// the detected peak sits a little lean of the true peak — the caller nudges back
/// rich to compensate.
async fn seek_metric_peak(ctx: &ToolCtx<'_>, metric: &str, drop_margin: f64) -> Result<(f64, f64)> {
    const INCREMENT: f64 = 0.004; // per tick — tiny, for a smooth continuous creep
    const TICK_MS: u64 = 70;
    // Lean at least this far past the rich start before a drop can end it (clears
    // the rich plateau and the initial lag).
    const MIN_LEAN_BEFORE_STOP: f64 = 0.08;
    // Sustained ticks below the peak before stopping (debounce lag/noise).
    const CONSEC_DROPS_TO_STOP: u32 = 6;

    ramp_actuator(ctx, MIXTURE, 0.95).await?; // ease to the rich start
    tokio::time::sleep(std::time::Duration::from_millis(600)).await; // let it settle

    let mut mix = 0.95;
    let mut best_mix = mix;
    let mut best_val = engine0(ctx, metric, 0.0).await;
    let mut drops = 0u32;
    while mix > MIX_LEAN_FLOOR {
        mix = (mix - INCREMENT).max(MIX_LEAN_FLOOR);
        ctx.xplane.set_float(MIXTURE, mix).await?; // direct tiny write = smooth creep
        tokio::time::sleep(std::time::Duration::from_millis(TICK_MS)).await;
        let val = engine0(ctx, metric, 0.0).await;
        if val > best_val {
            best_val = val;
            best_mix = mix;
            drops = 0;
        } else if best_val - val >= drop_margin && (0.95 - mix) > MIN_LEAN_BEFORE_STOP {
            drops += 1;
            if drops >= CONSEC_DROPS_TO_STOP {
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
                // Altitude-based target: the best-economy lever position leans with
                // altitude (EGT peak tracks air density). Deterministic and reliable
                // — a live peak-EGT seek is too fragile with the sim's EGT lag.
                // ~0.92 near sea level down to ~0.55 up high.
                let target = (0.92 - (alt / 1000.0) * 0.033).clamp(0.50, 0.95);
                // Smooth eased ramp to the target, same as the throttle.
                ramp_actuator(ctx, MIXTURE, target).await?;
                let pct = ctx.xplane.get_float(MIXTURE).await? * 100.0;
                let gph = engine0(ctx, FUEL_FLOW_KG_S, 0.0).await * 3600.0 / AVGAS_KG_PER_GAL;
                let egt_f = engine0(ctx, EGT_CEL, 0.0).await * 9.0 / 5.0 + 32.0; // dataref is °C
                Ok(format!(
                    "Leaned for cruise, mixture {pct:.0} percent, EGT {egt_f:.0} degrees, burning {gph:.1} gallons an hour."
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

pub struct SetFuelPump;

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
        ctx.xplane.set_int_array_elem(FUEL_PUMP, 0, a.on as i64).await?;
        let on = ctx.xplane.get_float_array(FUEL_PUMP).await?.first().copied().unwrap_or(0.0) > 0.5;
        Ok(format!("Roger, fuel pump {}.", if on { "on" } else { "off" }))
    }
}

pub struct SetCarbHeat;

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
        ctx.xplane.set_int_array_elem(CARB_HEAT, 0, a.on as i64).await?;
        let on = ctx.xplane.get_float_array(CARB_HEAT).await?.first().copied().unwrap_or(0.0) > 0.5;
        Ok(format!("Roger, carb heat {}.", if on { "on" } else { "off" }))
    }
}

pub struct SetMagnetos;

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
        ctx.xplane.set_int_array_elem(IGNITION_KEY, 0, key).await?;
        let actual = ctx.xplane.get_float_array(IGNITION_KEY).await?.first().copied().unwrap_or(-1.0).round() as i64;
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
        let keys: Vec<&str> = self.profile.positions.iter().map(|p| p.key.as_str()).collect();
        let listing: Vec<String> =
            self.profile.positions.iter().map(|p| format!("{} ({})", p.key, p.spoken)).collect();
        json!({
            "type": "object",
            "properties": {
                "position": {"type": "string", "enum": keys,
                             "description": format!("Fuel tank selector position. Available: {}.", listing.join(", "))}
            },
            "required": ["position"]
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: FuelSelArgs = serde_json::from_value(args.clone())?;
        let Some(pos) = self
            .profile
            .positions
            .iter()
            .find(|p| p.key.eq_ignore_ascii_case(&a.position))
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
