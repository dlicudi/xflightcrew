//! Engine controls: `set_throttle` and `set_mixture`.
//!
//! Both are continuous 0.0–1.0 ratios in X-Plane; we expose them as 0–100 % so
//! the pilot can say "throttle 60 percent" / "mixture full rich".

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::aircraft::FuelSelectorProfile;

const MIXTURE: &str = "sim/cockpit2/engine/actuators/mixture_ratio_all";
const BATTERY: &str = "sim/cockpit/electrical/battery_on";
const BUS_VOLTS: &str = crate::datarefs::BUS_VOLTS;
const GROUNDSPEED: &str = crate::datarefs::GROUNDSPEED_MS; // m/s
const ENGN_RUNNING: &str = crate::datarefs::ENGINE_RUNNING;
const CMD_MAGS_BOTH: &str = "sim/magnetos/magnetos_both";
const CMD_MAGS_OFF: &str = "sim/magnetos/magnetos_off";
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
/// The avionics master's position, the readback for `sim/systems/avionics_on`.
const AVIONICS_POWER: &str = "sim/cockpit2/switches/avionics_power_on";
const IGNITION_ON: &str = "sim/cockpit/engine/ignition_on";
const CARB_HEAT: &str = "sim/cockpit2/engine/actuators/carb_heat_ratio"; // 0 cold .. 1 hot

/// Write the same value to elements `0..engines` of an int array dataref — for
/// per-engine accessories on a multi (carb heat, boost pump, magnetos). A single
/// engine writes just element 0, exactly as before.
async fn set_each_engine(
    ctx: &ToolCtx<'_>,
    dataref: &str,
    engines: usize,
    value: i64,
) -> Result<()> {
    for i in 0..engines.max(1) {
        ctx.xplane.set_int_array_elem(dataref, i, value).await?;
    }
    Ok(())
}
// Engine indicators (per-engine arrays; element 0). Names verified against the
// stock 172's cockpitdecks mapping.
const EGT_CEL: &str = "sim/cockpit2/engine/indicators/EGT_deg_cel";
const PROP_RPM: &str = crate::datarefs::PROP_SPEED_RPM;
const FUEL_FLOW_KG_S: &str = crate::datarefs::FUEL_FLOW_KG_SEC;
const ALT_FT: &str = crate::datarefs::ALTITUDE_FT;
// Above this altitude (ft MSL) a normally-aspirated engine makes well under 75%
// power even at full throttle, so leaning to peak EGT is safe and called for;
// below it we keep the mixture rich (the takeoff/climb/pattern regime). Mirrors
// the POH's "as altitude increases, the pilot leans the mixture".
const LEAN_FLOOR_ALT_FT: f64 = 3000.0;
// Auto-lean never pulls the mixture below this — a guard against an over-lean that
// would stumble the engine; real cruise leaning stays well rich of idle-cutoff.
const MIX_LEAN_FLOOR: f64 = 0.30;

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

/// Spoken query for an impossible percentage, or None if the value is sane.
/// An out-of-range value is a mishear, not an intent ("Power two hundred
/// percent" heard live), and the old clamp silently turned it into full power.
/// A real copilot queries a nonsense number; it doesn't round it to the nearest
/// lever stop and act.
fn bad_percent_reply(system: &str, p: f64) -> Option<String> {
    (!(0.0..=100.0).contains(&p)).then(|| {
        format!("Say again? I heard {system} {p:.0} percent, and it runs zero to one hundred.")
    })
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
        ctx.xplane
            .set_float(dataref, from + (to - from) * eased)
            .await?;
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
        if let Some(reply) = a.percent.and_then(|p| bad_percent_reply("throttle", p)) {
            return Ok(reply);
        }
        // Same for a relative change: a garbled "by two hundred" would clamp to
        // idle or full power, the worst possible readings of a mishear.
        if let Some(d) = a.delta.filter(|d| d.abs() > 100.0) {
            return Ok(format!(
                "Say again? I heard a throttle change of {d:.0} percent, and that can't be right."
            ));
        }
        // A relative request adjusts the CURRENT lever — "reduce by ten" from 45%
        // goes to 35%, never to an absolute 10% (a power chop, seen live).
        let target_pct = if let Some(d) = a.delta {
            ctx.xplane.get_float(&crate::aircraft::throttle_dataref()).await? * 100.0 + d
        } else if let Some(p) = a.percent {
            p
        } else {
            return Ok(
                "Say a throttle setting — a percent, or by how much to change it.".to_string(),
            );
        };
        let ratio = (target_pct / 100.0).clamp(0.0, 1.0);
        // Ease the lever to the target instead of snapping it — feels like a real
        // hand on the throttle.
        ramp_actuator(ctx, &crate::aircraft::throttle_dataref(), ratio).await?;
        let actual = ctx.xplane.get_float(&crate::aircraft::throttle_dataref()).await? * 100.0;
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
        if let Some(reply) = bad_percent_reply("mixture", p.percent) {
            return Ok(reply);
        }
        // Safety gate: don't cut the mixture to idle cutoff in flight — that stops
        // the engine. (On the ground it's a normal shutdown, confirmed separately.)
        // Default to airborne if the read fails, so an unknown state errs safe.
        if p.percent <= 0.0
            && ctx
                .xplane
                .get_float(crate::datarefs::ON_GROUND_ANY)
                .await
                .unwrap_or(0.0)
                <= 0.5
        {
            return Ok("Negative, I won't cut the mixture in flight.".to_string());
        }
        // An explicit percentage is the pilot taking the lever — the continuous
        // manager stands down rather than re-trimming over their setting, and
        // a lean seek mid-hunt lets go the same way.
        release_mixture();
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
/// observed peak and the peak value.
///
/// Done the way an experienced pilot does it — from wherever the lever already
/// is, no full-rich reset ritual: coarse steps to *bracket* the peak (flipping
/// direction once if the first step shows we started lean of it), then fine
/// steps back through the bracket to pin it. Small quick motions with just
/// enough settle for the gauge to respond: ~8–14 s instead of the old rich-
/// reset sweep's 25–30.
/// Returns `Ok(None)` when the seek was released mid-hunt — the pilot took
/// the mixture back, or a recovery/autoland owns the levers now. Whoever
/// released it already spoke; the seek just stops moving the lever.
async fn seek_metric_peak(
    ctx: &ToolCtx<'_>,
    metric: &str,
    drop_margin: f64,
) -> Result<Option<(f64, f64)>> {
    const COARSE: f64 = 0.05;
    const FINE: f64 = 0.015;
    const COARSE_SETTLE_MS: u64 = 450;
    const FINE_SETTLE_MS: u64 = 600;

    let mut mix = ctx
        .xplane
        .get_float(MIXTURE)
        .await?
        .clamp(MIX_LEAN_FLOOR, 0.95);
    tokio::time::sleep(std::time::Duration::from_millis(COARSE_SETTLE_MS)).await;
    let mut best_mix = mix;
    let mut best_val = engine0(ctx, metric, 0.0).await;

    // Coarse bracket: lean first (the usual case is rich of peak). If the very
    // first step drops the metric sharply, we started lean of the peak — flip
    // once and climb the other way.
    let mut dir = -1.0_f64;
    let mut flipped = false;
    let mut first = true;
    loop {
        if lean_seek_cancelled(ctx.xplane) {
            return Ok(None);
        }
        let next = (mix + dir * COARSE).clamp(MIX_LEAN_FLOOR, 1.0);
        if (next - mix).abs() < 1e-6 {
            break; // pinned at a stop
        }
        mix = next;
        ramp_actuator(ctx, MIXTURE, mix).await?;
        tokio::time::sleep(std::time::Duration::from_millis(COARSE_SETTLE_MS)).await;
        let val = engine0(ctx, metric, 0.0).await;
        if val > best_val {
            best_val = val;
            best_mix = mix;
        } else if first && !flipped && best_val - val >= drop_margin / 2.0 {
            dir = 1.0;
            flipped = true;
        } else if best_val - val >= drop_margin {
            break; // bracketed: clearly past the peak
        }
        first = false;
    }

    // Fine pass: from slightly rich of the bracket down through it.
    let mut fine = (best_mix + COARSE).min(0.95);
    let floor = (best_mix - COARSE).max(MIX_LEAN_FLOOR);
    ramp_actuator(ctx, MIXTURE, fine).await?;
    tokio::time::sleep(std::time::Duration::from_millis(FINE_SETTLE_MS)).await;
    while fine > floor {
        if lean_seek_cancelled(ctx.xplane) {
            return Ok(None);
        }
        fine = (fine - FINE).max(floor);
        ramp_actuator(ctx, MIXTURE, fine).await?;
        tokio::time::sleep(std::time::Duration::from_millis(FINE_SETTLE_MS)).await;
        let val = engine0(ctx, metric, 0.0).await;
        if val > best_val {
            best_val = val;
            best_mix = fine;
        } else if best_val - val >= drop_margin {
            break;
        }
    }
    Ok(Some((best_mix, best_val)))
}

/// True while the continuous mixture manager runs — armed by `lean_mixture`,
/// released by the pilot ("my mixture"), by an explicit `set_mixture`, or by
/// touching the lever. Mirrors `speedhold::ACTIVE`.
/// Owned by its run, [`MIXTURE_MANAGER`]: read it, never write it.
pub static MIX_MANAGED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// The continuous mixture manager's run: the ONE writer of [`MIX_MANAGED`],
/// cancelled by `flight_state::stand_down` with every loop in its `LOOPS`
/// (a bare flag stored false let a re-arm inside the manager's sleep run a
/// second manager beside the first), and its client refuses every write
/// once cancelled, so a trim already under way never lands a lean lever
/// over the landing's full rich.
pub(crate) static MIXTURE_MANAGER: sim::run::Slot = sim::run::Slot::flagged(&MIX_MANAGED);

/// True while a `lean_mixture` seek is hunting the peak in the background —
/// guards re-entry, and clearing it (pilot "my mixture", an explicit
/// `set_mixture`) stops the hunt mid-step.
/// Owned by its run, [`LEAN_SEEK`]: read it, never write it.
pub static LEAN_SEEKING: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// A `lean_mixture` seek's run, the one writer of [`LEAN_SEEKING`]: in
/// `flight_state::LOOPS` like the manager, its client cancelled with it.
pub(crate) static LEAN_SEEK: sim::run::Slot = sim::run::Slot::flagged(&LEAN_SEEKING);

/// The pilot takes the mixture back ("my mixture", an explicit percentage,
/// "your aircraft"), or a landing wants it rich: the manager and any seek
/// stop, their writes refused from now. Whether either was running.
pub(crate) fn release_mixture() -> bool {
    let managed = MIXTURE_MANAGER.cancel();
    let seeking = LEAN_SEEK.cancel();
    managed || seeking
}

/// True when a background lean seek must stand down: the pilot took the
/// mixture back (its run cancelled, or superseded by another seek), or a
/// recovery/autoland owns the levers now.
fn lean_seek_cancelled(xp: &dyn crate::xplane::XPlane) -> bool {
    use std::sync::atomic::Ordering;
    !LEAN_SEEKING.load(Ordering::Relaxed)
        || xp.cancellation().iter().any(|c| c.load(Ordering::SeqCst))
        || crate::flight_state::crew_landing_or_recovering()
}

/// One-line status for the dashboard's "managing" panel, when a lean seek is
/// hunting or the continuous mixture manager is running.
pub fn mixture_status() -> Option<&'static str> {
    use std::sync::atomic::Ordering;
    if LEAN_SEEKING.load(Ordering::Relaxed) {
        return Some("mixture lean in progress");
    }
    MIX_MANAGED
        .load(Ordering::Relaxed)
        .then_some("mixture auto-lean")
}

/// Should the manager re-trim? Altitude or power moved enough that the last
/// lean is stale — the same judgement a pilot makes ("we've climbed a bit,
/// touch up the mixture").
pub fn needs_retrim(alt_ft: f64, last_alt_ft: f64, throttle: f64, last_throttle: f64) -> bool {
    (alt_ft - last_alt_ft).abs() > 400.0 || (throttle - last_throttle).abs() > 0.08
}

/// A lever at or above this is "rich": the manager has never leaned it (or
/// it has been pushed back up), so a trim from here is a full search for the
/// peak, not a touch-up around it.
const RICH_LEVER: f64 = 0.90;

/// The manager's seek window from the lever it finds: `(top, floor, step)`.
/// A leaned lever gets a short fine bracket around itself (the world moved a
/// little, the peak moved a little). A RICH lever gets the whole range in
/// coarser steps: the old ±0.05 bracket from full rich was 0.95 to 0.95, one
/// sample, and the lever sat at 0.98 for the rest of the cruise (the manager
/// "leaned" nothing when armed from below the floor and flown through it).
pub fn manager_seek_window(mix: f64) -> (f64, f64, f64) {
    if mix >= RICH_LEVER {
        (0.95, MIX_LEAN_FLOOR, 0.03)
    } else {
        (
            (mix + 0.05).min(0.95),
            (mix - 0.05).max(MIX_LEAN_FLOOR),
            0.015,
        )
    }
}

/// Mixture full rich for the landing (the BEFORE LANDING item), releasing the
/// continuous manager and any lean seek first so nothing re-leans behind it.
/// `Some(true)` when the lever moved, `Some(false)` when it was already rich,
/// `None` when the lever could not be read (nothing written).
pub async fn mixture_rich_for_landing(xp: &crate::xplane::XPlaneClient) -> Option<bool> {
    release_mixture();
    let mix = xp.get_float(MIXTURE).await.ok()?;
    if mix >= 0.98 {
        return Some(false);
    }
    glide_lever(xp, MIXTURE, 1.0).await;
    Some(true)
}

/// The continuous mixture manager: every couple of seconds, check whether the
/// world moved (altitude, power) and quietly re-trim — full rich below the
/// leaning floor, a short fine EGT seek around the current lever above it.
/// Yields to recovery/autoland, and to the pilot's hand on the lever.
#[cfg_attr(test, allow(dead_code))]
async fn mixture_manager(xp: crate::xplane::XPlaneClient, run: &sim::run::Lease<'_>) {
    tracing::info!("mixture manager started");
    let mut last_alt = f64::MIN; // force an immediate first trim decision
    let mut last_thr = f64::MIN;
    let mut last_cmd: Option<f64> = None;
    loop {
        if !run.live() {
            break;
        }
        if crate::flight_state::crew_landing_or_recovering() {
            // Whoever owns the levers now wants them rich (a recovery, a
            // landing): a lever the manager leaned is pushed back up on the
            // way out, never left lean under an approach.
            run.lower_flag();
            if last_cmd.is_some_and(|lc| lc < 0.98) {
                glide_lever(&xp, MIXTURE, 1.0).await;
                tracing::info!("mixture manager: yielding, full rich");
            }
            break;
        }
        let (Ok(alt), Ok(thr), Ok(mix)) = (
            xp.get_float(ALT_FT).await,
            xp.get_float(&crate::aircraft::throttle_dataref()).await,
            xp.get_float(MIXTURE).await,
        ) else {
            tokio::time::sleep(std::time::Duration::from_millis(2000)).await;
            continue;
        };
        // Pilot's hand on the lever → theirs, immediately.
        if last_cmd.is_some_and(|lc| (mix - lc).abs() > 0.10) {
            tracing::info!("mixture manager: pilot override, releasing");
            run.lower_flag();
            break;
        }
        let running = xp
            .get_float_array(ENGN_RUNNING)
            .await
            .ok()
            .and_then(|v| v.first().copied())
            .unwrap_or(0.0)
            > 0.5;
        if running && needs_retrim(alt, last_alt, thr, last_thr) {
            if alt < LEAN_FLOOR_ALT_FT {
                if mix < 0.98 {
                    let _ = xp.set_float(MIXTURE, 1.0).await;
                    last_cmd = Some(1.0);
                    tracing::info!("mixture manager: below the floor, full rich");
                }
            } else {
                // Seek the peak from the lever we find (`manager_seek_window`):
                // a fine bracket around a leaned lever, the whole range from
                // a rich one. EGT judged after a short settle. Quiet and quick.
                let mut best_mix = mix;
                let mut best_egt = read_egt(&xp).await;
                let (top, floor, step) = manager_seek_window(mix);
                let mut m = top;
                let _ = xp.set_float(MIXTURE, m).await;
                tokio::time::sleep(std::time::Duration::from_millis(500)).await;
                let mut released = false;
                while m > floor {
                    // Released mid-seek (the pilot, a landing's full-rich):
                    // stop moving the lever, and leave it where THEY put it.
                    if !run.live() {
                        released = true;
                        break;
                    }
                    m = (m - step).max(floor);
                    let _ = xp.set_float(MIXTURE, m).await;
                    tokio::time::sleep(std::time::Duration::from_millis(500)).await;
                    let egt = read_egt(&xp).await;
                    if egt > best_egt {
                        best_egt = egt;
                        best_mix = m;
                    } else if best_egt - egt >= 12.0 {
                        break;
                    }
                }
                if released {
                    break;
                }
                // Sit just rich of the peak, per the POH.
                let set = (best_mix + 0.03).min(1.0);
                glide_lever(&xp, MIXTURE, set).await;
                last_cmd = Some(set);
                tracing::info!(mix = %format!("{set:.2}"), "mixture manager: re-trimmed");
            }
            (last_alt, last_thr) = (alt, thr);
        }
        tokio::time::sleep(std::time::Duration::from_millis(2000)).await;
    }
    tracing::info!("mixture manager stopped");
}

/// Micro-step a lever smoothly (the manager's own small glide — 50 ms steps,
/// same feel as `ramp_actuator` without needing a ToolCtx).
async fn glide_lever(xp: &crate::xplane::XPlaneClient, dataref: &str, to: f64) {
    let Ok(from) = xp.get_float(dataref).await else {
        let _ = xp.set_float(dataref, to).await;
        return;
    };
    let steps = ((to - from).abs() / 0.005).ceil().clamp(1.0, 6.0) as u32;
    for i in 1..=steps {
        let v = from + (to - from) * (i as f64 / steps as f64);
        let _ = xp.set_float(dataref, v).await;
        if i < steps {
            tokio::time::sleep(std::time::Duration::from_millis(50)).await;
        }
    }
}

async fn read_egt(xp: &crate::xplane::XPlaneClient) -> f64 {
    xp.get_float_array(EGT_CEL)
        .await
        .ok()
        .and_then(|v| v.first().copied())
        .unwrap_or(0.0)
}

/// Arm the continuous manager (idempotent): spawned on its own thread with its
/// own runtime, like every background loop here — the main runtime blocks on
/// the mic and starves tokio tasks (learned the hard way with the speed hold).
pub fn arm_mixture_manager(base_url: String) {
    let Some(run) = MIXTURE_MANAGER.claim() else {
        return;
    };
    // The manager drives the REAL sim, so it is compiled out under test:
    // there the run is only held, until cancelled.
    #[cfg(not(test))]
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            return;
        };
        let xp = crate::xplane::XPlaneClient::new(base_url).with_cancel(run.cancel.clone());
        rt.block_on(mixture_manager(xp, &run));
    });
    #[cfg(test)]
    {
        let _ = base_url;
        std::thread::spawn(move || {
            while run.live() {
                std::thread::sleep(std::time::Duration::from_millis(5));
            }
        });
    }
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
            _ => {
                if alt >= LEAN_FLOOR_ALT_FT {
                    "economy"
                } else {
                    "rich"
                }
            }
        };

        match resolved {
            "rich" => {
                ramp_actuator(ctx, MIXTURE, 1.0).await?;
                // The manager makes the promise true: it leans by itself once
                // we climb past the floor.
                arm_mixture_manager(ctx.config.xplane_base_url.clone());
                Ok(format!(
                    "Below {LEAN_FLOOR_ALT_FT:.0} feet, mixture full rich. I'll lean for cruise once we're higher."
                ))
            }
            "economy" => {
                // Don't lean hard at high power down low (detonation risk). Up high,
                // full throttle is still under 75% power, so leaning is correct.
                if alt < LEAN_FLOOR_ALT_FT
                    && ctx.xplane.get_float(&crate::aircraft::throttle_dataref()).await.unwrap_or(0.0) > 0.80
                {
                    return Ok(
                        "We're at high power down low, I'll hold off leaning; \
                         leaning hard there risks detonation. Reduce to cruise power or climb first."
                            .to_string(),
                    );
                }
                let Some(run) = LEAN_SEEK.claim() else {
                    return Ok("Already working the mixture, stand by.".to_string());
                };
                spawn_lean_seek(ctx, true, run);
                Ok("Roger, leaning to peak EGT. Give me ten seconds or so.".to_string())
            }
            _ => {
                // power: lean for max RPM (the high-DA takeoff / climb procedure).
                let Some(run) = LEAN_SEEK.claim() else {
                    return Ok("Already working the mixture, stand by.".to_string());
                };
                spawn_lean_seek(ctx, false, run);
                Ok("Roger, leaning for max power. Stand by.".to_string())
            }
        }
    }
}

/// The slow half of `lean_mixture`, run OFF the voice loop. The ack goes out
/// the moment the command lands and the RESULT is read back once it's
/// measured — the old inline seek was silent for its whole 10-15 seconds
/// ("did it register at all?", 2026-07-14) and, worse, blocked the voice
/// loop, so the copilot was deaf the whole time too. Own thread + runtime
/// like every background loop here (the main runtime blocks on the mic).
fn spawn_lean_seek(ctx: &ToolCtx<'_>, economy: bool, run: sim::run::Lease<'static>) {
    let config = ctx.config.clone();
    let voice = ctx.voice.cloned();
    let status = ctx.status.cloned();
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            return;
        };
        rt.block_on(async move {
            let xp = crate::xplane::XPlaneClient::new(config.xplane_base_url.clone())
                .with_cancel(run.cancel.clone());
            let ctx = ToolCtx {
                xplane: &xp,
                config: &config,
                voice: voice.as_ref(),
                status: status.as_ref(),
                reminders: None,
            };
            // Released mid-hunt (None, or a last write refused once the run
            // was cancelled): whoever took the mixture back already spoke,
            // the seek just stops moving the lever.
            let line = lean_seek(&ctx, economy).await.filter(|_| run.live());
            drop(run);
            // The voice mirrors what it speaks onto the panel itself, so the
            // line is posted here ONLY when there is no voice to say it (both
            // paths wrote it twice, blue then green, 2026-09-12).
            if let Some(line) = line {
                match (&voice, &status) {
                    (Some(v), _) => v.alert(crate::voice::Priority::Advisory, line),
                    (None, Some(s)) => s.action(&line),
                    (None, None) => {}
                }
            }
        });
    });
}

/// The seek itself: hunt the peak, set the lever, and return the line to read
/// back — or `None` when the seek was released mid-hunt. Failures speak too:
/// a silent brake error hid a broken override for days, and a silent lean
/// would strand the lever wherever the hunt died.
async fn lean_seek(ctx: &ToolCtx<'_>, economy: bool) -> Option<String> {
    if economy {
        // Lean to peak EGT, then enrich slightly so EGT sits just rich of
        // peak — the safe side the POH calls for ("slightly below the peak
        // observed EGT").
        match seek_metric_peak(ctx, EGT_CEL, 12.0).await {
            Ok(None) => None,
            Ok(Some((best, _))) if best > 0.92 => {
                // Never saw EGT climb — low power, or the engine isn't loaded
                // enough to lean. Leave it rich rather than report a bogus value.
                let _ = ramp_actuator(ctx, MIXTURE, 1.0).await;
                Some("Couldn't find a clear EGT peak, leaving the mixture rich. Try leaning at cruise power.".to_string())
            }
            Ok(Some((best, egt))) => {
                if let Err(e) = ramp_actuator(ctx, MIXTURE, (best + 0.03).min(1.0)).await {
                    tracing::warn!("lean seek: setting the peak failed: {e:#}");
                    return Some(
                        "Couldn't finish the lean, mixture's where I left it.".to_string(),
                    );
                }
                let pct = ctx.xplane.get_float(MIXTURE).await.unwrap_or(best + 0.03) * 100.0;
                // Piston-only by construction, so this resolves to the avgas
                // gallons it always was — but through the airframe's own
                // unit, so the density lives in exactly one place.
                let burn = crate::aircraft::say_fuel_flow_kg_hr(
                    engine0(ctx, FUEL_FLOW_KG_S, 0.0).await * 3600.0,
                );
                let egt_f = egt * 9.0 / 5.0 + 32.0; // dataref is °C
                                                    // From here it's continuous: the manager re-trims as altitude
                                                    // or power changes, until the pilot takes the lever back.
                arm_mixture_manager(ctx.config.xplane_base_url.clone());
                Some(format!(
                    "Leaned to peak EGT, mixture {pct:.0} percent, EGT {egt_f:.0} degrees, burning {burn}. I'll keep it trimmed as we climb or descend."
                ))
            }
            Err(e) => {
                tracing::warn!("lean seek: EGT hunt failed: {e:#}");
                Some("Couldn't finish the lean, mixture's where I left it.".to_string())
            }
        }
    } else {
        match seek_metric_peak(ctx, PROP_RPM, 25.0).await {
            Ok(None) => None,
            Ok(Some((best, rpm))) => {
                if let Err(e) = ramp_actuator(ctx, MIXTURE, best).await {
                    tracing::warn!("lean seek: setting max power failed: {e:#}");
                    return Some(
                        "Couldn't finish the lean, mixture's where I left it.".to_string(),
                    );
                }
                let pct = ctx.xplane.get_float(MIXTURE).await.unwrap_or(best) * 100.0;
                Some(format!(
                    "Leaned for max power — {rpm:.0} RPM at mixture {pct:.0} percent."
                ))
            }
            Err(e) => {
                tracing::warn!("lean seek: RPM hunt failed: {e:#}");
                Some("Couldn't finish the lean, mixture's where I left it.".to_string())
            }
        }
    }
}

pub struct StartEngine {
    /// How many engines to crank, in order. A piston twin has a starter per
    /// engine and no combined command, so the sequence runs once per engine.
    pub engines: usize,
    /// Per-side CYCLE selectors (one per engine), for airframes whose fuel
    /// valves are knobs stepped by up/down commands rather than set directly.
    /// Same failure as the singular selector below, through a different shape:
    /// the Baron 58's selectors sat at OFF, so a textbook start — mags both,
    /// mixture rich, throttle cracked, 28 volts on the bus from a ground cart —
    /// cranked two dry engines and reported "neither engine has caught"
    /// (2026-08-01). Its position dataref is READ-ONLY, so nothing but the
    /// command could ever have opened them.
    pub cycle_selectors: Vec<crate::aircraft::CycleSelectorProfile>,
    /// The airframe's fuel selector, when it has one. A start with the selector
    /// OFF just cranks dry (seen live on the SR22 after an xlua plugin reload
    /// reset it), so the sequence selects a tank first, like the checklist does.
    pub fuel_selector: Option<FuelSelectorProfile>,
    /// ON commands for every charging source in the electrical profile (the
    /// SR22 has ALT 1 *and* ALT 2). Empty = fall back to generator 1 only.
    pub alternator_on_cmds: Vec<String>,
    /// ON commands to bring the DC bus alive before the start (battery + DC
    /// master), for airframes whose stock `battery_on` is overwritten every
    /// frame by a custom switch sync (the Twin Otter). Empty = the stock write
    /// below is enough (the pistons).
    pub bus_power_on_cmds: Vec<String>,
}

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
        // Start only the DEAD engines. The old gate read element 0 alone, so a
        // Baron with the left running and the right cold answered "already
        // running" and the dead engine could never be started; the mirror case
        // (left cold, right running) cranked a starter into a live engine.
        // Per-engine, like the crank/readback fix of 2026-08-01 and the
        // turbine sequence's relight.
        let engines = self.engines.max(1);
        let mut dead: Vec<usize> = Vec::new();
        for i in 0..engines {
            if !engine_running(ctx, i).await {
                dead.push(i);
            }
        }
        if dead.is_empty() {
            return Ok(if engines > 1 {
                "Engines are already running.".to_string()
            } else {
                "Engine's already running.".to_string()
            });
        }

        // Battery check: the battery must be on, and with enough voltage to crank.
        // Cranking a flat battery just drains it further without catching.
        //
        // Turn the bus on the way THIS airframe does it. On the pistons the stock
        // `battery_on` write is enough; on an add-on that frame-syncs that dataref
        // FROM a custom switch (the Twin Otter — master.lua overwrites it every
        // frame), the stock write is a no-op and the bus reads 0 V, so we fire the
        // profile's battery + DC-master commands and give the sync a frame to
        // energize the bus before reading it (a cold-dark Twin Otter otherwise
        // failed every full-auto start on "only 0 volts", 2026-07-26).
        if self.bus_power_on_cmds.is_empty() {
            ctx.xplane.set_int(BATTERY, 1).await?;
        } else {
            for cmd in &self.bus_power_on_cmds {
                ctx.xplane.run_command(cmd).await?;
            }
            tokio::time::sleep(std::time::Duration::from_millis(400)).await;
        }
        let volts = ctx
            .xplane
            .get_float_array(BUS_VOLTS)
            .await?
            .first()
            .copied()
            .unwrap_or(0.0);
        // The gate scales with the airframe's system voltage (profile
        // `bus_volts_nominal`): 23 on a 24 volt system, 11.5 on the RV-10's
        // 12, which the flat 23 called "low" at a healthy 12 (2026-09-11).
        if volts < crate::aircraft::min_start_volts() {
            return Ok(format!(
                "Battery's low, only {volts:.0} volts — connect ground power or you may not get a start."
            ));
        }

        // Fuel selector: cranking with it OFF never catches, so select a tank
        // first, per the checklist — BOTH where the airframe has it, otherwise
        // the fullest wing tank (the SR22 flow: "fuel selector, fullest tank").
        let mut fuel_note = String::new();
        if let Some(fs) = &self.fuel_selector {
            let off = match &fs.readback {
                Some(dref) => ctx.xplane.get_float(dref).await.ok().and_then(|actual| {
                    fs.positions
                        .iter()
                        .find(|p| p.value.is_some_and(|v| (v - actual).abs() < 0.5))
                        .map(|p| p.key.eq_ignore_ascii_case("off"))
                }),
                None => None,
            };
            if off == Some(true) {
                let mut target = fs
                    .positions
                    .iter()
                    .find(|p| p.key.eq_ignore_ascii_case("both"));
                if target.is_none() {
                    let tanks = ctx
                        .xplane
                        .get_float_array(crate::datarefs::FUEL_PER_TANK_KG)
                        .await
                        .unwrap_or_default();
                    let side = if tanks.len() >= 2 && tanks[1] > tanks[0] {
                        "right"
                    } else {
                        "left"
                    };
                    target = fs
                        .positions
                        .iter()
                        .find(|p| p.key.eq_ignore_ascii_case(side));
                }
                if let Some(pos) = target {
                    ctx.xplane.run_command(&pos.command).await?;
                    // Claimed only once the selector READS the tank: a
                    // command that did not land is named, never "now both".
                    let landed = match (&fs.readback, pos.value) {
                        (Some(dref), Some(want)) => ctx
                            .xplane
                            .get_float(dref)
                            .await
                            .ok()
                            .map(|v| (v - want).abs() < 0.5),
                        _ => None,
                    };
                    match landed {
                        Some(true) => {
                            fuel_note = format!("fuel selector was off, now {}, ", pos.spoken)
                        }
                        Some(false) => {
                            return Ok(format!(
                                "The fuel selector is off and didn't move to {}. Check it before I crank.",
                                pos.spoken
                            ));
                        }
                        None => {}
                    }
                }
            }
        }

        // Cycle selectors to their start positions BEFORE anything else: on
        // these airframes only the command opens the valve, and everything
        // below is just cranking a dry engine until it says so.
        let mut cycled = Vec::new();
        for sel in &self.cycle_selectors {
            if let Some(target) = sel.start_value {
                let landed = super::cycle::cycle_to(ctx, &sel.switch, target).await?;
                if landed != target {
                    return Ok(format!(
                        "The {} won't move off {landed} — check it before I crank.",
                        sel.spoken
                    ));
                }
                cycled.push(sel.spoken.clone());
            }
        }
        if !cycled.is_empty() {
            fuel_note = format!("{} on, ", crate::join_and(&cycled));
        }

        // Pre-start configuration, in the STARTING ENGINE checklist's order
        // and at a person's pace (`crew_pace`): all three used to land in
        // the same few milliseconds.
        let voice = ctx.voice;
        // Each called only once it READS back (`crew_pace::checked`): a
        // switch that did not take is named in both modes.
        if !fuel_note.is_empty() {
            let note = fuel_note.trim_end_matches(", ");
            let mut c = note.chars();
            let note = match c.next() {
                Some(f) => format!("{}{}.", f.to_uppercase(), c.as_str()),
                None => String::new(),
            };
            crate::crew_pace::checked(voice, Some(true), &note, &note).await;
        }
        let throttle = crate::aircraft::throttle_dataref();
        ctx.xplane.set_float(&throttle, 0.1).await?; // crack the throttle
        let cracked = ctx
            .xplane
            .get_float(&throttle)
            .await
            .ok()
            .map(|v| (v - 0.1).abs() < 0.05);
        crate::crew_pace::checked(
            voice,
            cracked,
            "Throttle cracked.",
            "The throttle didn't move.",
        )
        .await;
        ctx.xplane.set_float(MIXTURE, 1.0).await?; // full rich
        let rich = ctx.xplane.get_float(MIXTURE).await.ok().map(|v| v >= 0.95);
        crate::crew_pace::checked(
            voice,
            rich,
            "Mixture full rich.",
            "The mixture didn't go rich.",
        )
        .await;
        ctx.xplane.run_command(CMD_MAGS_BOTH).await?;
        let mags = ctx
            .xplane
            .get_float_array(IGNITION_KEY)
            .await
            .ok()
            .and_then(|k| {
                let k: Vec<f64> = k.into_iter().take(engines).collect();
                (k.len() == engines).then(|| k.iter().all(|v| v.round() as i64 == 3))
            });
        crate::crew_pace::checked(voice, mags, "Mags both.", "The mags didn't go to both.").await;
        // The one call a pilot makes aloud before any crank, in both modes.
        crate::crew_pace::milestone(voice, "Clear prop!").await;
        crate::crew_pace::pause(crate::crew_pace::CLEAR_PROP_MS).await;

        // Crank each DEAD engine in turn, one starter at a time (they share a
        // battery, and a real start sequence is sequential anyway). A twin
        // whose second starter is never touched taxis out on one engine while
        // the copilot reports a successful start — the check below reads the
        // engine it just cranked, not always element 0. Running engines are
        // never cranked: a starter into a live engine is the mirror bug.
        let mut failed: Vec<usize> = Vec::new();
        for &i in &dead {
            if engines > 1 {
                crate::crew_pace::item(voice, &format!("Starting number {}.", i + 1)).await;
            }
            ctx.xplane.run_command_for(&starter_command(i), 3.0).await?;
            let mut lit = false;
            for _ in 0..6 {
                tokio::time::sleep(std::time::Duration::from_millis(500)).await;
                if engine_running(ctx, i).await {
                    lit = true;
                    break;
                }
            }
            if !lit {
                failed.push(i);
            }
        }
        {
            if failed.is_empty() {
                // Bring the charging sources online now there's an engine to
                // drive them — ALL of them (the SR22's ALT 2 sat off forever
                // when this only fired generator 1), so we charge the bus
                // instead of draining the battery (and so the monitor doesn't
                // immediately call out an offline alternator).
                // Read back already: every cranked engine reads running.
                crate::crew_pace::checked(
                    voice,
                    Some(true),
                    if engines > 1 {
                        "Engines running."
                    } else {
                        "Engine running."
                    },
                    "",
                )
                .await;
                let n_alts = if self.alternator_on_cmds.is_empty() {
                    ctx.xplane.run_command(CMD_ALTERNATOR_ON).await?;
                    1
                } else {
                    for c in &self.alternator_on_cmds {
                        ctx.xplane.run_command(c).await?;
                    }
                    self.alternator_on_cmds.len()
                };
                let gens = ctx
                    .xplane
                    .get_float_array(GENERATOR)
                    .await
                    .unwrap_or_default();
                let alt_on = gens.len() >= n_alts && gens.iter().take(n_alts).all(|g| *g > 0.5);
                crate::crew_pace::checked(
                    voice,
                    Some(alt_on),
                    if n_alts > 1 {
                        "Alternators on."
                    } else {
                        "Alternator on."
                    },
                    if n_alts > 1 {
                        "Alternators did not come on."
                    } else {
                        "Alternator did not come on."
                    },
                )
                .await;
                // Bring the avionics master online now the engine's running and
                // the alternator is charging — matches the STARTING ENGINE
                // checklist (avionics OFF for the start, ON after) and closes the
                // gap where the transponder/radios sat on a dead bus. Commands
                // verified present in the sim.
                // The avionics-master ON command is universal (sim/systems/...),
                // so fire it directly rather than coupling to the (now
                // profile-built) electrical tool.
                let avionics_on = ctx
                    .xplane
                    .run_command("sim/systems/avionics_on")
                    .await
                    .is_ok();
                // The command's Ok is not a readback: the master is read.
                let avionics_read = if avionics_on {
                    ctx.xplane
                        .get_float(AVIONICS_POWER)
                        .await
                        .ok()
                        .map(|v| v > 0.5)
                } else {
                    Some(false)
                };
                crate::crew_pace::checked(
                    voice,
                    avionics_read,
                    "Avionics on.",
                    "Avionics did not come on.",
                )
                .await;
                // Called item by item already: the closing line is the result.
                if crate::crew_pace::items_spoken() {
                    return Ok(if alt_on {
                        "Start complete.".to_string()
                    } else {
                        "Start complete, but the alternator didn't come on, check it.".to_string()
                    });
                }
                // Narrate the start sequence so the pilot hears what was done on
                // their behalf, not just the end state.
                // The result, not a recap: every step was done at its own
                // beat, and "Roger, mixture rich, throttle cracked, mags
                // both, cranking. Engine started, alternator on, avionics on"
                // after the fact was the machine tell (LIPK 2026-09-26). What
                // is still worth saying is what the pilot would not expect:
                // a fuel selector we moved, a charging source that did not
                // come on.
                let _ = (n_alts, avionics_on);
                let alt = if alt_on {
                    ""
                } else {
                    ", but the alternator didn't come on, check it"
                };
                // A partial relight names the engine it started.
                let started = if engines == 1 {
                    "Engine started".to_string()
                } else if dead.len() == engines {
                    "Both engines started".to_string()
                } else {
                    format!("Engine {} started, all engines running", dead[0] + 1)
                };
                let fuel = fuel_note.trim_end_matches(", ");
                let fuel = match fuel.chars().next() {
                    Some(c) => format!("{}{}. ", c.to_uppercase(), &fuel[c.len_utf8()..]),
                    None => String::new(),
                };
                return Ok(format!("{fuel}{started}{alt}."));
            }
        }
        // Say WHICH engine on a twin: "the engine hasn't caught" sends the
        // pilot looking at the one that is already running. The failed list
        // carries the real indices now, not a count read as an index.
        let which = match (engines, failed.as_slice()) {
            (1, _) => "the engine hasn't caught".to_string(),
            (n, f) if f.len() == n => "neither engine has caught".to_string(),
            (_, f) => {
                let names: Vec<String> = f.iter().map(|i| format!("{}", i + 1)).collect();
                format!("engine {} hasn't caught", names.join(" and "))
            }
        };
        Ok(format!(
            "Cranking — {fuel_note}{which}. Check fuel and mixture, then try again."
        ))
    }
}

/// The starter command for engine `i` (0-based): X-Plane numbers them from one.
fn starter_command(i: usize) -> String {
    format!("sim/starters/engage_starter_{}", i + 1)
}

/// Is engine `i` running? A failed read is "not running", so the caller keeps
/// waiting rather than declaring a start it never saw.
async fn engine_running(ctx: &ToolCtx<'_>, i: usize) -> bool {
    ctx.xplane
        .get_float_array(ENGN_RUNNING)
        .await
        .ok()
        .and_then(|v| v.get(i).copied())
        .is_some_and(|r| r > 0.5)
}

/// Electric/auxiliary fuel pump(s). `engines` = how many to switch (a twin
/// drives both boost pumps off one "fuel pump on" command). `commands`
/// overrides the universal array with the airframe's own on/off commands (the
/// RWDesigns Twin Otter — its pumps ignore `fuel_pump_on` entirely).
pub struct SetFuelPump {
    pub engines: usize,
    pub commands: Option<crate::aircraft::PumpCommands>,
}

/// Read a readback dataref (`name` or `name[i]`) as a 0/1-ish float.
async fn read_switch(ctx: &ToolCtx<'_>, dref: &str) -> Option<f64> {
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
        // Command-driven pumps (profile override): fire the airframe's own
        // switches and confirm from its readbacks, not the universal array.
        if let Some(pc) = &self.commands {
            let cmds = if a.on {
                &pc.on_commands
            } else {
                &pc.off_commands
            };
            for c in cmds {
                ctx.xplane.run_command(c).await?;
            }
            let mut states = Vec::new();
            for d in &pc.readback {
                if let Some(v) = read_switch(ctx, d).await {
                    states.push(v > 0.5);
                }
            }
            let word = if a.on { "on" } else { "off" };
            return Ok(if states.is_empty() || states.iter().all(|&s| s == a.on) {
                format!("Roger, boost pumps {word}.")
            } else {
                // Mixed — report rather than lie (one pump didn't move).
                let n = states.iter().filter(|&&s| s).count();
                format!("Boost pumps: {n} of {} on — check the panel.", states.len())
            });
        }
        set_each_engine(ctx, FUEL_PUMP, self.engines, a.on as i64).await?;
        let on = ctx
            .xplane
            .get_float_array(FUEL_PUMP)
            .await?
            .first()
            .copied()
            .unwrap_or(0.0)
            > 0.5;
        Ok(format!(
            "Roger, fuel pump {}.",
            if on { "on" } else { "off" }
        ))
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
        let on = ctx
            .xplane
            .get_float_array(CARB_HEAT)
            .await?
            .first()
            .copied()
            .unwrap_or(0.0)
            > 0.5;
        Ok(format!(
            "Roger, carb heat {}.",
            if on { "on" } else { "off" }
        ))
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
            other => {
                return Ok(format!(
                    "I didn't catch the magneto position '{other}'. Say off, left, right, or both."
                ))
            }
        };
        // Both selectors are per-engine ARRAYS; they differ only in which one
        // the airframe wires its mags to. So both write every engine: the key
        // switches on a piston twin are one per engine (the Baron 58 has a left
        // and a right), and writing element 0 alone left the right engine on
        // whatever the pilot last selected while the readback — also element 0
        // — happily confirmed the change. A single engine writes element 0 and
        // nothing else, exactly as before.
        let dref = if self.per_engine {
            IGNITION_ON
        } else {
            IGNITION_KEY
        };
        set_each_engine(ctx, dref, self.engines, key).await?;
        // Confirm from the LAST engine written: element 0 can be right while a
        // later one silently refused (a failed write, a model that drives its
        // own switch), and the mag check is exactly where that matters.
        let actual = ctx
            .xplane
            .get_float_array(dref)
            .await?
            .get(self.engines.max(1) - 1)
            .copied()
            .unwrap_or(-1.0)
            .round() as i64;
        let name = match actual {
            0 => "off",
            1 => "right",
            2 => "left",
            3 => "both",
            4 => "start",
            _ => "unknown",
        };
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
        let mut keys: Vec<&str> = self
            .profile
            .positions
            .iter()
            .map(|p| p.key.as_str())
            .collect();
        keys.push("switch");
        let listing: Vec<String> = self
            .profile
            .positions
            .iter()
            .map(|p| format!("{} ({})", p.key, p.spoken))
            .collect();
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
        let requested =
            if matches!(requested.as_str(), "switch" | "swap" | "other") {
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
                    _ => return Ok(
                        "I can't tell which tank we're feeding from — say left tank or right tank."
                            .to_string(),
                    ),
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
            let names: Vec<&str> = self
                .profile
                .positions
                .iter()
                .map(|p| p.spoken.as_str())
                .collect();
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

pub struct StopEngine {
    /// How many engines the shutdown confirmation polls. Element 0 alone can
    /// read stopped while a later engine keeps turning (the magnetos lesson:
    /// confirm the LAST engine, not the first).
    pub engines: usize,
    /// The airframe's park-brake valve sense, for the brake the shutdown sets.
    pub park_brake_valve_set: Option<f64>,
}

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
        // ask for confirmation, refuse unless it's actually safe: on the ground
        // and stopped. The parking brake the shutdown sets itself.
        if ctx.xplane.get_float(crate::datarefs::ON_GROUND_ANY).await? < 0.5 {
            return Ok(
                "Negative — we're airborne. I won't shut the engine down in flight.".to_string(),
            );
        }
        if ctx.xplane.get_float(GROUNDSPEED).await? > 1.0 {
            return Ok("Negative — still moving. Bring her to a stop first.".to_string());
        }
        let parked =
            match super::controls::park_for_shutdown(ctx.xplane, self.park_brake_valve_set).await {
                Ok(p) => p.unwrap_or(""),
                Err(line) => return Ok(line),
            };

        ctx.xplane.set_float(MIXTURE, 0.0).await?; // idle cutoff
        ctx.xplane.run_command(CMD_MAGS_OFF).await?;

        for _ in 0..5 {
            tokio::time::sleep(std::time::Duration::from_millis(600)).await;
            // EVERY engine must be down before "shut down" is spoken. Element
            // 0 stopping first with the right engine still turning was a
            // readback confirming the half it looked at.
            let mut any_running = false;
            for i in 0..self.engines.max(1) {
                if engine_running(ctx, i).await {
                    any_running = true;
                    break;
                }
            }
            if !any_running {
                return Ok(format!(
                    "{parked}Roger, mixture to idle cutoff, magnetos off. Engine shut down."
                ));
            }
        }
        Ok(format!("{parked}Mixture to cutoff, engine winding down."))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    static MIX_TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    /// The mixture manager and the lean seek wrote the lever on bare flags,
    /// outside every stand-down registry: a stand-down (swap, instructor, a
    /// new flight) stored false and a re-arm inside the manager's 2 s sleep
    /// spawned a SECOND manager the first one woke up beside, both trimming;
    /// and a tick already past its check (the seek's last step, the lean
    /// glide after it) wrote a lean lever over the landing's full rich. Each
    /// is now a run (`sim::run::Slot`) in `flight_state::LOOPS`: a cancel
    /// drops the flag, the old run's client refuses every write, and a
    /// re-arm supersedes it, so one manager trims at a time.
    #[test]
    fn a_stand_down_ends_the_mixture_runs_and_a_rearm_runs_one() {
        use std::sync::atomic::Ordering;
        let _g = MIX_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let names: Vec<&str> = crate::flight_state::LOOPS.iter().map(|(n, _)| *n).collect();
        assert!(names.contains(&"tools::engine::MIXTURE_MANAGER"), "{names:?}");
        assert!(names.contains(&"tools::engine::LEAN_SEEK"), "{names:?}");

        let first = MIXTURE_MANAGER.claim().expect("idle");
        assert!(MIX_MANAGED.load(Ordering::SeqCst));
        assert!(MIXTURE_MANAGER.claim().is_none(), "one manager at a time");
        // What stand_down does to every run in LOOPS.
        MIXTURE_MANAGER.cancel();
        assert!(!MIX_MANAGED.load(Ordering::SeqCst), "the flag drops at once");
        let xp = crate::xplane::XPlaneClient::new("http://127.0.0.1:9".into())
            .with_cancel(first.cancel.clone());
        assert!(xp.cancelled(), "the old manager's writes are refused");
        // A re-arm while the old run is still winding down supersedes it.
        let second = MIXTURE_MANAGER.claim().expect("a cancelled run is superseded");
        assert!(!first.live() && second.live());
        assert!(MIX_MANAGED.load(Ordering::SeqCst));
        drop(first);
        assert!(MIX_MANAGED.load(Ordering::SeqCst), "the old run's exit is not the new one's");
        drop(second);
        assert!(!MIX_MANAGED.load(Ordering::SeqCst));

        // The seek and the pilot's "my mixture" ([`release_mixture`]).
        let seek = LEAN_SEEK.claim().expect("idle");
        assert!(LEAN_SEEKING.load(Ordering::SeqCst) && seek.live());
        let manager = MIXTURE_MANAGER.claim().expect("idle");
        assert!(release_mixture(), "something was held");
        assert!(!LEAN_SEEKING.load(Ordering::SeqCst) && !MIX_MANAGED.load(Ordering::SeqCst));
        assert!(!seek.live() && !manager.live());
        assert!(!release_mixture(), "nothing left to release");
    }

    /// The manager armed below the floor and flown through it found the lever
    /// full rich and bracketed ±0.05 around it: one sample at 0.95, "peak"
    /// there, lever parked at 0.98 for the whole cruise. A rich lever is a
    /// full search; a leaned one is a touch-up.
    #[test]
    fn a_rich_lever_gets_the_whole_range_and_a_leaned_one_a_bracket() {
        let (top, floor, step) = manager_seek_window(1.0);
        assert_eq!((top, floor, step), (0.95, MIX_LEAN_FLOOR, 0.03));
        assert!(floor < 0.5, "a full walk must reach cruise mixtures");
        let (top, floor, step) = manager_seek_window(0.65);
        assert!((top - 0.70).abs() < 1e-9 && (floor - 0.60).abs() < 1e-9);
        assert_eq!(step, 0.015);
        // The bracket never goes under the lean floor.
        let (_, floor, _) = manager_seek_window(0.32);
        assert_eq!(floor, MIX_LEAN_FLOOR);
    }

    /// A cold start with the fuel selectors OFF must OPEN them, not crank two
    /// dry engines and report that neither caught (Baron 58, 2026-08-01). The
    /// aircraft's own selector dataref is read-only, so the cycle command is
    /// the only thing that can do it.
    #[tokio::test(start_paused = true)]
    async fn a_cold_start_opens_cycle_fuel_selectors_first() {
        let profile = crate::aircraft::load("beechcraft_baron_58").expect("baron profile");
        let cfg = crate::config::Config::default();
        // Selectors already ON: the start proceeds and cranks BOTH engines.
        // The engines "catch" shortly after the first crank, driven from a
        // task alongside — otherwise the sequence waits out its full
        // three-second-per-engine give-up and the test costs six seconds.
        let fake = std::sync::Arc::new(crate::xplane::FakeXPlane::with_reads(&[
            (BATTERY, 1.0),
            (crate::datarefs::ON_GROUND_ANY, 1.0),
            ("laminar/b58/fuel_selector_L", 1.0),
            ("laminar/b58/fuel_selector_R", 1.0),
        ]));
        fake.set_read_array(BUS_VOLTS, &[28.0]);
        fake.set_read_array(ENGN_RUNNING, &[0.0, 0.0]);
        let catching = fake.clone();
        tokio::spawn(async move {
            tokio::time::sleep(std::time::Duration::from_millis(200)).await;
            catching.set_read_array(ENGN_RUNNING, &[1.0, 1.0]);
        });
        let start = StartEngine {
            engines: 2,
            cycle_selectors: profile.fuel_selectors.clone(),
            fuel_selector: None,
            alternator_on_cmds: Vec::new(),
            bus_power_on_cmds: Vec::new(),
        };
        let ctx = ToolCtx {
            xplane: fake.as_ref(),
            config: &cfg,
            voice: None,
            status: None,
            reminders: None,
        };
        let reply = start.execute(&json!({}), &ctx).await.expect("start runs");
        assert!(reply.contains("Both engines started"), "{reply}");
        assert!(fake.ran("sim/starters/engage_starter_1"));
        assert!(
            fake.ran("sim/starters/engage_starter_2"),
            "a twin cranks both"
        );

        // Selectors stuck OFF (the position never moves): refuse and say so,
        // rather than cranking a dry engine to no effect.
        let stuck = crate::xplane::FakeXPlane::with_reads(&[
            (BATTERY, 1.0),
            (crate::datarefs::ON_GROUND_ANY, 1.0),
            ("laminar/b58/fuel_selector_L", 0.0),
            ("laminar/b58/fuel_selector_R", 0.0),
        ]);
        stuck.set_read_array(BUS_VOLTS, &[28.0]);
        stuck.set_read_array(ENGN_RUNNING, &[0.0, 0.0]);
        let ctx = ToolCtx {
            xplane: &stuck,
            config: &cfg,
            voice: None,
            status: None,
            reminders: None,
        };
        let reply = start.execute(&json!({}), &ctx).await.expect("start runs");
        assert!(
            stuck.ran("laminar/b58/fuel_selector_L_up"),
            "it must try the command: {reply}"
        );
        assert!(
            reply.contains("left fuel selector"),
            "name what is wrong: {reply}"
        );
        assert!(
            !stuck.ran("sim/starters/engage_starter_1"),
            "never crank a dry engine: {reply}"
        );
    }

    /// One engine running, one cold: the start must relight ONLY the dead one.
    /// The old entry gate read element 0 alone, so left-running/right-cold
    /// answered "already running" and the dead engine could never be started,
    /// while the mirror case cranked a starter into a live engine.
    #[tokio::test(start_paused = true)]
    async fn a_twin_relight_cranks_only_the_dead_engine() {
        let profile = crate::aircraft::load("beechcraft_baron_58").expect("baron profile");
        let cfg = crate::config::Config::default();
        let start = StartEngine {
            engines: 2,
            cycle_selectors: profile.fuel_selectors.clone(),
            fuel_selector: None,
            alternator_on_cmds: Vec::new(),
            bus_power_on_cmds: Vec::new(),
        };

        // Left running, right cold: crank starter 2 and never starter 1.
        let fake = std::sync::Arc::new(crate::xplane::FakeXPlane::with_reads(&[
            (BATTERY, 1.0),
            (crate::datarefs::ON_GROUND_ANY, 1.0),
            ("laminar/b58/fuel_selector_L", 1.0),
            ("laminar/b58/fuel_selector_R", 1.0),
        ]));
        fake.set_read_array(BUS_VOLTS, &[28.0]);
        fake.set_read_array(ENGN_RUNNING, &[1.0, 0.0]);
        let catching = fake.clone();
        tokio::spawn(async move {
            tokio::time::sleep(std::time::Duration::from_millis(200)).await;
            catching.set_read_array(ENGN_RUNNING, &[1.0, 1.0]);
        });
        let ctx = ToolCtx {
            xplane: fake.as_ref(),
            config: &cfg,
            voice: None,
            status: None,
            reminders: None,
        };
        let reply = start.execute(&json!({}), &ctx).await.expect("start runs");
        assert!(
            !fake.ran("sim/starters/engage_starter_1"),
            "never crank a live engine: {reply}"
        );
        assert!(fake.ran("sim/starters/engage_starter_2"), "{reply}");
        assert!(reply.contains("Engine 2 started"), "{reply}");

        // Mirror case: left cold, right running.
        let fake = std::sync::Arc::new(crate::xplane::FakeXPlane::with_reads(&[
            (BATTERY, 1.0),
            (crate::datarefs::ON_GROUND_ANY, 1.0),
            ("laminar/b58/fuel_selector_L", 1.0),
            ("laminar/b58/fuel_selector_R", 1.0),
        ]));
        fake.set_read_array(BUS_VOLTS, &[28.0]);
        fake.set_read_array(ENGN_RUNNING, &[0.0, 1.0]);
        let catching = fake.clone();
        tokio::spawn(async move {
            tokio::time::sleep(std::time::Duration::from_millis(200)).await;
            catching.set_read_array(ENGN_RUNNING, &[1.0, 1.0]);
        });
        let ctx = ToolCtx {
            xplane: fake.as_ref(),
            config: &cfg,
            voice: None,
            status: None,
            reminders: None,
        };
        let reply = start.execute(&json!({}), &ctx).await.expect("start runs");
        assert!(fake.ran("sim/starters/engage_starter_1"), "{reply}");
        assert!(
            !fake.ran("sim/starters/engage_starter_2"),
            "never crank a live engine: {reply}"
        );

        // Both running: honest refusal, nothing cranked.
        let fake = crate::xplane::FakeXPlane::new();
        fake.set_read_array(ENGN_RUNNING, &[1.0, 1.0]);
        let ctx = ToolCtx {
            xplane: &fake,
            config: &cfg,
            voice: None,
            status: None,
            reminders: None,
        };
        let reply = start.execute(&json!({}), &ctx).await.expect("start runs");
        assert!(reply.contains("already running"), "{reply}");
        assert!(!fake.ran("sim/starters/engage_starter_1"));
    }

    #[test]
    fn impossible_percent_is_queried_not_clamped() {
        // "Power two hundred percent" (seen live) was silently clamped to full
        // power. It must come back as a query, never an action.
        let reply = bad_percent_reply("throttle", 200.0).expect("200% must be rejected");
        assert!(
            reply.contains("200"),
            "reply names the heard value: {reply}"
        );
        assert!(bad_percent_reply("mixture", -5.0).is_some());
        // Sane values pass through untouched, including the extremes.
        assert!(bad_percent_reply("throttle", 0.0).is_none());
        assert!(bad_percent_reply("throttle", 100.0).is_none());
        assert!(bad_percent_reply("throttle", 65.0).is_none());
    }

    #[test]
    fn released_seek_stops_without_touching_the_lever() {
        let _g = MIX_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // "My mixture" (or an explicit set_mixture) clears LEAN_SEEKING while
        // the background hunt runs; the hunt must stand down at the next step
        // and leave the lever alone. LEAN_SEEKING is false here — the
        // released state — so the seek returns None having written nothing.
        let fake = crate::xplane::FakeXPlane::with_reads(&[(MIXTURE, 0.8)]);
        let config = crate::config::Config::default();
        let ctx = ToolCtx {
            xplane: &fake,
            config: &config,
            voice: None,
            status: None,
            reminders: None,
        };
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        let out = rt.block_on(seek_metric_peak(&ctx, EGT_CEL, 12.0)).unwrap();
        assert!(out.is_none(), "released seek must not report a peak");
        assert!(
            !fake.wrote(MIXTURE),
            "released seek must not move the lever"
        );
    }

    #[test]
    fn retrim_triggers_on_altitude_or_power_change() {
        assert!(
            !needs_retrim(10_000.0, 10_200.0, 0.65, 0.65),
            "small drift stays put"
        );
        assert!(
            needs_retrim(10_500.0, 10_000.0, 0.65, 0.65),
            "climbed 500 → re-lean"
        );
        assert!(
            needs_retrim(10_000.0, 10_000.0, 0.80, 0.65),
            "power change → re-lean"
        );
        assert!(
            needs_retrim(8_000.0, f64::MIN, 0.65, f64::MIN),
            "first pass always trims"
        );
    }
}
