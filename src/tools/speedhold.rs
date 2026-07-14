//! `hold_speed` — the copilot works the throttle to hold a target airspeed.
//!
//! For airframes with no autothrottle (the 172): "hold 70 knots" arms a
//! background governor that nudges `throttle_ratio_all` toward the target a
//! few times a second-and-a-half. Deliberately gentle: small bounded steps
//! with a deadband, so it trims toward the speed instead of chasing gusts —
//! and it never touches pitch, so the pilot still owns the attitude (speed
//! hold with power is the stable GA division of labour on approach).
//!
//! Release: "stop holding speed" / "my throttle" (`off`), and it stands down
//! by itself the moment a recovery or autoland takes the aircraft, or on the
//! ground below taxi speed.
//!
//! Taxiing, the pilot's BRAKES are a pause, not a release: both pedals down
//! brings the throttle to idle and the governor sits on its hands; pedals
//! fully off rolls back up to the held target ("resume when I release the
//! brakes", 2026-07-13). The full stops stay explicit — parking brake or a
//! throttle push releases the hold entirely.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx, BRAKES_REGULAR, BRAKE_HOLD_SECS};
use crate::xplane::XPlaneClient;

const IAS: &str = "sim/cockpit2/gauges/indicators/airspeed_kts_pilot";
const THROTTLE: &str = "sim/cockpit2/engine/actuators/throttle_ratio_all";
const GS_MS: &str = "sim/flightmodel/position/groundspeed";
const ON_GROUND: &str = "sim/flightmodel/failures/onground_any";
const LEFT_BRAKE: &str = "sim/cockpit2/controls/left_brake_ratio";
const RIGHT_BRAKE: &str = "sim/cockpit2/controls/right_brake_ratio";
// The held-brake command and its expiry (`BRAKES_REGULAR`/`BRAKE_HOLD_SECS`)
// live in `super` now: `hold_brakes` shares the same brake key. The history
// of WHY it's a command, not ratio writes (the override that never existed,
// the re-drive fight REST-rate writes lost) is pinned on the consts there.
/// Proportional taxi brake, served by the bundled plugin: it drives the
/// toe-brake ratios from this value every FRAME (override_toe_brakes held
/// only while braking — per-frame writes win the re-drive fight that
/// REST-rate writes lost), and expires it within a second of writes
/// stopping, ratios zeroed and override dropped — the same can't-be-left-on
/// property as the held command, but it keeps the law's magnitude. Speed
/// TRIMS go here (regular effort is right for "stop", wrong for shaving two
/// knots); stops keep the firm held command. Missing (plugin absent or old)
/// falls back to the held command — harsher but braking, never silent
/// nothing.
const TAXI_BRAKE: &str = "xcopilot/taxi_brake";
const PARK_BRAKE: &str = "sim/cockpit2/controls/parking_brake_ratio";
const MS_TO_KTS: f64 = 1.943_844;
/// Toe-brake ratio above which the pilot is deliberately braking — enough to
/// clear pedal rest/noise, well below a firm press. Only read once our own
/// held-brake command has expired, so the ratio IS the pilot's pedal.
const PILOT_BRAKE_YIELD: f64 = 0.1;
/// Both pedals back below this = fully released, the brake-pause is over and
/// the taxi hold rolls again. Lower than the yield threshold on purpose —
/// hysteresis, so a pedal hovering at the yield point can't flap the hold
/// between paused and rolling.
const PILOT_BRAKE_RESUME: f64 = 0.05;

/// The pilot has taken the brakes to SLOW: they're pressing BOTH toe brakes
/// while the governor isn't (so the ratio reflects hardware, not our own
/// command). The copilot must hand back rather than throttle up against the
/// pedals — the same yield it gives a throttle-lever nudge or the parking brake.
///
/// Deliberately `min`, not `max`: a single pedal is DIFFERENTIAL braking, which
/// is how you steer a taxiing aircraft — and on a castoring-nosewheel type (the
/// SR22) it's the ONLY low-speed steering. Standing down on one pedal made the
/// taxi hold cut throttle to 0 every time the pilot turned (2026-07-12). Only
/// both pedals together reads as "I'm slowing us," not "I'm turning."
fn pilot_took_the_brakes(left: f64, right: f64, governor_braking: bool) -> bool {
    !governor_braking && left.min(right) > PILOT_BRAKE_YIELD
}

/// True while the governor loop runs. Public so other control loops could
/// check it, mirroring `recover::ACTIVE` / `autoland::ACTIVE`.
pub static ACTIVE: AtomicBool = AtomicBool::new(false);

/// True while the governor is actively TAXIING (not a flight hold) — the
/// parking-brake tool uses this to tell the pilot the throttle is coming to
/// idle with the brake.
pub fn taxi_hold_active() -> bool {
    ACTIVE.load(Ordering::Relaxed)
        && *MODE.lock().unwrap_or_else(|e| e.into_inner()) == Mode::Taxi
}
/// Target knots — updated live by a new `hold_speed` call, no restart.
static TARGET_KTS: Mutex<f64> = Mutex::new(0.0);
/// The last rolling taxi target, so "go" after a "stop" resumes the same
/// pace. 0.0 = never taxied this session.
static LAST_TAXI_KTS: Mutex<f64> = Mutex::new(0.0);

/// Which speed the governor chases. Flight = indicated airspeed with throttle
/// only; Taxi = groundspeed with throttle *and brakes* (idle thrust still
/// accelerates a 172 downhill — on the ground "less power" isn't enough).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Mode {
    Flight,
    Taxi,
}
static MODE: Mutex<Mode> = Mutex::new(Mode::Flight);

/// One-line status for the dashboard's "managing" panel: what the governor is
/// chasing, or `None` when it isn't running.
pub fn status() -> Option<String> {
    if !ACTIVE.load(std::sync::atomic::Ordering::Relaxed) {
        return None;
    }
    let kts = *TARGET_KTS.lock().unwrap_or_else(|e| e.into_inner());
    let mode = *MODE.lock().unwrap_or_else(|e| e.into_inner());
    Some(match mode {
        Mode::Taxi if kts < 1.0 => "holding position".to_string(),
        Mode::Taxi => format!("taxi {kts:.0} kt"),
        Mode::Flight => format!("speed {kts:.0} kt"),
    })
}

/// One governor step: throttle change for a speed error and the airspeed
/// trend, knots and knots/second → ratio. Human-like: many SMALL corrections
/// on a fast tick (500 ms) instead of big slow shoves — and the derivative
/// term reacts to a *decaying* airspeed before the error grows, which is the
/// anti-stall reflex ("the needle's unwinding, power NOW" — not "we are now
/// 10 knots slow"). The slow side gets extra authority: overspeed is a
/// comfort problem, stall is not.
pub fn throttle_adjust(err_kts: f64, trend_kts_s: f64) -> f64 {
    if err_kts.abs() < 1.5 && trend_kts_s.abs() < 1.0 {
        return 0.0; // in the groove and steady: hands off
    }
    let p = err_kts * 0.003;
    let d = -trend_kts_s * 0.008; // decaying speed → positive (add power) early
    let max_up = if err_kts > 10.0 || trend_kts_s < -3.0 { 0.04 } else { 0.015 };
    (p + d).clamp(-0.015, max_up)
}

/// Sanity bounds for a spoken flight target — outside this it's a mishear.
pub fn clamp_target(kts: f64) -> f64 {
    kts.clamp(55.0, 250.0)
}

/// Sanity bounds for a taxi target. 0 is legitimate ("hold short" = stop).
pub fn clamp_taxi_target(kts: f64) -> f64 {
    kts.clamp(0.0, 25.0)
}

/// Above this spoken target on the ground, the pilot means a flight hold, not a
/// taxi — and below it airborne, they don't (55-kt floor applies).
const TAXI_MAX_KTS: f64 = 30.0;

/// One taxi-governor step: `(new_throttle, brake_ratio)` for a target,
/// current groundspeed, and the speed TREND (knots/second). Throttle capped
/// well below takeoff power; brakes are for deep overspeed and stops only,
/// never speed trims; near-zero targets hold the aircraft on the brakes so
/// idle thrust can't creep it forward.
///
/// The trend term is the same anticipation the flight law has, and it's the
/// fix for the taxi limit cycle: a 10-knot taxi sawtoothed 6.6 to 11.9 knots
/// forever (blackbox, 2026-07-14) because power only came back once the
/// error was already big, seconds into the idle-throttle decel, and braking
/// at one knot over turned every crest into a stab. Leading the error by the
/// trend acts on where the speed is GOING. The 2.5 s lead matches the engine
/// spool the correction has to ride through; the caller must hand in a
/// SMOOTHED trend (the governor EMAs it) — raw tick-to-tick differences of a
/// noisy groundspeed read turn the lead into phantom brake stabs.
pub fn taxi_control(
    target_kts: f64,
    gs_kts: f64,
    throttle: f64,
    trend_kts_s: f64,
) -> (Option<f64>, f64) {
    if target_kts < 1.0 && gs_kts < 1.5 {
        return (Some(0.0), 0.5); // holding position: idle + parked on the brakes
    }
    // Where we'll be once a lever change has worked through the spool.
    let eff = (target_kts - gs_kts) - trend_kts_s * 2.5;
    if eff > 1.0 {
        // Too slow, or decaying toward it: a touch more power (never takeoff
        // power), brakes off. Small steps on a fast tick, human feet, not a
        // bang-bang thermostat; well behind the pace the step doubles so the
        // rebuild from idle doesn't undershoot for ten seconds.
        let step = (eff * 0.003).min(if eff > 3.0 { 0.02 } else { 0.01 });
        (Some((throttle + step).min(0.35)), 0.0)
    } else if eff < -3.0 {
        // WELL past the target: idle now and brake in proportion, together —
        // the polite step-down let an SR22 run a 10-knot taxi through 20
        // (2026-07-11). Ramped from the boundary: the old flat 0.2 minimum
        // stabbed the pedals the instant this branch was crossed.
        (Some(0.0), ((-eff - 3.0) * 0.12 + 0.1).clamp(0.1, 0.6))
    } else if eff < -1.0 {
        // A little fast: power off is the whole correction for a rolling
        // taxi — idle thrust won't hold it fast on the flat, and braking to
        // shave one knot is how the ride turns into a rodeo. Gently: the
        // old 0.04 dump slammed the lever shut in half a second and bought
        // the deep undershoot on the other side. A STOP keeps braking to
        // the halt once the throttle's closed: idle creep never finishes a
        // stop on its own.
        if throttle > 0.03 {
            (Some((throttle - 0.01).max(0.0)), 0.0)
        } else if target_kts < 1.0 {
            (Some(0.0), ((-eff) * 0.08).clamp(0.1, 0.6))
        } else {
            (Some(0.0), 0.0)
        }
    } else {
        (None, 0.0) // in the groove: hands off, brakes released
    }
}

pub struct HoldSpeed {
    /// The profile's final-approach speed, for "hold approach speed".
    pub approach_kts: Option<f64>,
    /// The airframe's park-brake valve SET sense, for the taxi auto-release.
    pub park_brake_valve_set: Option<f64>,
}

#[derive(Deserialize)]
struct Args {
    /// Target IAS, knots. Absent = hold the current speed.
    #[serde(default)]
    knots: Option<f64>,
    /// Named target: "approach" = the profile's final-approach speed.
    #[serde(default)]
    preset: Option<String>,
    /// true = release the throttle back to the pilot.
    #[serde(default)]
    off: bool,
    /// true = "go": resume the last rolling taxi speed after a stop.
    #[serde(default)]
    resume: bool,
}

#[async_trait]
impl Tool for HoldSpeed {
    fn name(&self) -> &str {
        "hold_speed"
    }

    fn description(&self) -> &str {
        "Hold a target speed by managing the throttle. Airborne: airspeed hold \
         ('hold 70 knots', 'maintain approach speed') — power only, the pilot \
         flies pitch. On the ground: taxi speed hold with throttle AND brakes \
         ('taxi at 10 knots'; 'stop' / 'hold short' / 'hold position' = knots 0, \
         brake to a halt and hold; 'go' / 'continue taxi' = resume the last taxi \
         speed) — the pilot steers. Release with 'stop holding speed' or 'my \
         throttle'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "knots": {"type": "number", "description": "target IAS, knots; absent = hold current speed"},
                "preset": {"type": "string", "enum": ["approach"], "description": "named target: the profile's final-approach speed"},
                "off": {"type": "boolean", "description": "true = release the throttle"},
                "resume": {"type": "boolean", "description": "true = resume the last taxi speed after a stop ('go')"}
            }
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let mut a: Args = serde_json::from_value(args.clone())?;
        if a.off {
            return Ok(if ACTIVE.swap(false, Ordering::Relaxed) {
                "Speed hold off — your throttle.".to_string()
            } else {
                "I wasn't holding the speed.".to_string()
            });
        }
        let on_ground = ctx.xplane.get_float(ON_GROUND).await.unwrap_or(0.0) > 0.5;
        // "Go" — resume the pace we were taxiing at before the stop.
        if a.resume {
            if !on_ground {
                return Ok("We're airborne — nothing to resume down there.".to_string());
            }
            let last = *LAST_TAXI_KTS.lock().unwrap_or_else(|e| e.into_inner());
            if last < 1.0 {
                return Ok("We weren't taxiing — say a speed, like taxi at ten knots.".to_string());
            }
            a.knots = Some(last);
        }
        // A spoken "stop" in the air is a mishear or a panic; either way the
        // flight hold's 55-knot floor must not be the answer to it.
        if !on_ground && a.knots.is_some_and(|k| k < 1.0) {
            return Ok("We're airborne — I can't stop the aircraft. Say a speed to hold.".to_string());
        }
        // Mode by context: a small number on the ground is a taxi; airborne (or
        // a big number) is a flight hold. A big number on the ground is neither.
        let taxi = on_ground && a.knots.map_or(a.preset.is_none(), |k| k <= TAXI_MAX_KTS);
        if on_ground && !taxi {
            return Ok("We're on the ground — say a taxi speed like 'taxi at 10 knots', or ask again airborne.".to_string());
        }
        // A commanded taxi carries the intent to ROLL: release the parking
        // brake ourselves and say so — "release it first" sent the pilot to
        // do the copilot's job. Through the ONE valve-aware routine the
        // parking-brake tool uses: the ratio-only shortcut said "parking
        // brake off" on the SR22 while the valve held the wheels
        // (2026-07-11). A stop target with the brake set needs no governor.
        let mut brake_released = false;
        if taxi && ctx.xplane.get_float(PARK_BRAKE).await.unwrap_or(0.0) > 0.5 {
            if a.knots.is_some_and(|k| k < 1.0) {
                return Ok("Parking brake's set — we're staying put.".to_string());
            }
            brake_released =
                super::controls::drive_park_brake(ctx.xplane, false, self.park_brake_valve_set)
                    .await
                    .unwrap_or(false);
            if !brake_released {
                return Ok(
                    "Parking brake's set and it won't release from here — check the handle."
                        .to_string(),
                );
            }
        }
        // A crew-held "brakes on" yields to a commanded taxi the same way the
        // parking brake does: rolling was just asked for. A stop target hands
        // over silently, the governor parks on its own boot before the held
        // command expires.
        let crew_brakes_released = taxi && super::controls::release_held_brakes();
        let target = if taxi {
            match a.knots {
                Some(k) => clamp_taxi_target(k),
                // "Hold this speed" while rolling = keep the current pace.
                None => clamp_taxi_target(ctx.xplane.get_float(GS_MS).await? * MS_TO_KTS),
            }
        } else if let Some(k) = a.knots {
            clamp_target(k)
        } else if a.preset.as_deref() == Some("approach") {
            match self.approach_kts {
                Some(k) => clamp_target(k),
                None => {
                    return Ok(
                        "No approach speed on file for this airframe — give me a number.".to_string()
                    )
                }
            }
        } else {
            // "Maintain this speed" — hold whatever we're doing right now.
            clamp_target(ctx.xplane.get_float(IAS).await?)
        };
        if taxi && target >= 1.0 {
            *LAST_TAXI_KTS.lock().unwrap_or_else(|e| e.into_inner()) = target;
        }
        *TARGET_KTS.lock().unwrap_or_else(|e| e.into_inner()) = target;
        *MODE.lock().unwrap_or_else(|e| e.into_inner()) =
            if taxi { Mode::Taxi } else { Mode::Flight };
        // Already governing → just the new target/mode; otherwise start the
        // loop — on its OWN thread with its own runtime, like recover and the
        // monitor. The main runtime is current-thread and blocks on the mic
        // between commands, so a tokio::spawn there starves: the first live
        // governor sat unpolled for 23 seconds until the next voice command.
        if !ACTIVE.swap(true, Ordering::Relaxed) {
            let base = ctx.config.xplane_base_url.clone();
            let voice = ctx.voice.cloned();
            std::thread::spawn(move || {
                let Ok(rt) = tokio::runtime::Builder::new_current_thread().enable_all().build()
                else {
                    ACTIVE.store(false, Ordering::Relaxed);
                    return;
                };
                rt.block_on(governor(XPlaneClient::new(base), voice));
            });
        }
        Ok(if taxi {
            let brake_note = match (brake_released, crew_brakes_released && target >= 1.0) {
                (true, true) => "Parking brake off, my brakes too. ",
                (true, false) => "Parking brake off. ",
                (false, true) => "My brakes are off. ",
                (false, false) => "",
            };
            if target < 1.0 {
                "Stopping — brakes on. Say go to roll again.".to_string()
            } else if a.resume {
                format!("{brake_note}Rolling — back to {target:.0} knots.")
            } else {
                format!(
                    "{brake_note}Taxiing at {target:.0} knots — I have throttle and brakes, you steer. Say my throttle to take it back."
                )
            }
        } else {
            format!(
                "I have the throttle, holding {target:.0} knots. Say stop holding speed to take it back."
            )
        })
    }
}

/// Glide a lever dataref from wherever it is to `to` in 50 ms micro-steps —
/// continuous motion like a hand, not a stepped jump (per-tick 1.5% snaps
/// still read as jerky on the quadrant; this is the smoothing between ticks).
async fn glide(xp: &XPlaneClient, dataref: &str, to: f64, over_ms: u64) {
    let Ok(from) = xp.get_float(dataref).await else {
        let _ = xp.set_float(dataref, to).await;
        return;
    };
    let steps = (over_ms / 50).max(1);
    for i in 1..=steps {
        let v = from + (to - from) * (i as f64 / steps as f64);
        let _ = xp.set_float(dataref, v).await;
        if i < steps {
            tokio::time::sleep(std::time::Duration::from_millis(50)).await;
        }
    }
}

/// The governor loop: gentle bounded nudges toward the target until released.
/// Flight mode chases IAS with throttle only; taxi mode chases groundspeed with
/// throttle and brakes on a faster cadence. Brakes are always released on the
/// way out — never hand the pilot an aircraft with a hidden boot on the pedals.
async fn governor(xp: XPlaneClient, voice: Option<crate::voice::Voice>) {
    tracing::info!("speed governor started");
    let mut braking = false;
    // When the held-brake command was last renewed: until it expires, the
    // pedal ratios can be our own braking, not the pilot's feet. (Only the
    // held COMMAND pollutes the pedals — the proportional channel is
    // additive and leaves them alone.)
    let mut last_brake_cmd: Option<std::time::Instant> = None;
    // The proportional channel failed once (plugin absent or old): warned,
    // trims fall back to the held command for the rest of this governor.
    let mut trim_fallback = false;
    // Brake-paused: the pilot pressed both pedals mid-taxi, the throttle went
    // to idle, and the hold is waiting for the pedals to come fully off.
    let mut paused = false;
    // The throttle position this loop last commanded. If the lever is later
    // found far from where we put it, that's the PILOT's hand — yield
    // immediately rather than fight them for the lever (a taxi hold once
    // yanked the throttle back from a takeoff push; never again).
    let mut last_cmd: Option<f64> = None;
    // Last IAS sample, for the flight governor's trend (anticipation) term.
    let mut prev_ias: Option<(f64, std::time::Instant)> = None;
    // Last groundspeed sample and the EMA-smoothed trend for the taxi law's
    // anticipation term. Smoothed on purpose: the 2.5 s lead amplifies any
    // read noise 25x if fed raw tick-to-tick differences.
    let mut prev_gs: Option<(f64, std::time::Instant)> = None;
    let mut gs_trend = 0.0_f64;
    loop {
        if !ACTIVE.load(Ordering::Relaxed) {
            break;
        }
        // Yield instantly to anything that takes the whole aircraft.
        if super::recover::ACTIVE.load(Ordering::Relaxed)
            || crate::autoland::ACTIVE.load(Ordering::Relaxed)
        {
            ACTIVE.store(false, Ordering::Relaxed);
            break;
        }
        let target = *TARGET_KTS.lock().unwrap_or_else(|e| e.into_inner());
        let mode = *MODE.lock().unwrap_or_else(|e| e.into_inner());
        // Three-valued on purpose: a FAILED read is a sim hiccup, not an
        // answer — it must never impersonate "airborne" (the Web API blinked
        // during a plugin reload and the taxi governor dropped a braking
        // stop as 'airborne, released' at 10 knots, 2026-07-13).
        let on_ground = xp.get_float(ON_GROUND).await.ok().map(|v| v > 0.5);
        match mode {
            Mode::Flight => {
                if let (Ok(ias), Ok(thr)) = (xp.get_float(IAS).await, xp.get_float(THROTTLE).await)
                {
                    if last_cmd.is_some_and(|lc| (thr - lc).abs() > 0.15) {
                        tracing::info!("speed governor: pilot override, releasing");
                        ACTIVE.store(false, Ordering::Relaxed);
                        break;
                    }
                    // Stand down when parked — holding 70 knots against the
                    // brakes is not a thing anyone wants. (Never silent: this
                    // exact path once ate three arm requests invisibly.)
                    if on_ground == Some(true) && xp.get_float(GS_MS).await.unwrap_or(0.0) < 5.0 {
                        tracing::info!("speed governor: parked, standing down");
                        ACTIVE.store(false, Ordering::Relaxed);
                        break;
                    }
                    // IAS trend across ticks feeds the anticipation term.
                    let now_t = std::time::Instant::now();
                    let trend = match prev_ias {
                        Some((last_ias, last_t)) => {
                            let dt = now_t.duration_since(last_t).as_secs_f64();
                            if (0.2..=2.0).contains(&dt) { (ias - last_ias) / dt } else { 0.0 }
                        }
                        None => 0.0,
                    };
                    prev_ias = Some((ias, now_t));
                    let adj = throttle_adjust(target - ias, trend);
                    if adj != 0.0 {
                        let cmd = (thr + adj).clamp(0.0, 1.0);
                        glide(&xp, THROTTLE, cmd, 200).await; // ~most of the tick
                        tokio::time::sleep(std::time::Duration::from_millis(50)).await;
                    } else {
                        // In the groove: don't spin the API at 20 Hz doing nothing.
                        tokio::time::sleep(std::time::Duration::from_millis(250)).await;
                    }
                    if adj != 0.0 {
                        last_cmd = Some((thr + adj).clamp(0.0, 1.0));
                    }
                } else {
                    // Reads failed (sim hiccup): back off, don't spin.
                    tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
                }
            }
            Mode::Taxi => {
                // Can't read the squat switch: hold everything exactly as it
                // is (the brake ratios persist in the sim under the override)
                // and retry — the one wrong move is releasing a stop we may
                // be mid-way through.
                if on_ground.is_none() {
                    tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
                    continue;
                }
                // Airborne means the taxi is over — release everything, done.
                if on_ground == Some(false) {
                    tracing::info!("speed governor: airborne, taxi hold released");
                    ACTIVE.store(false, Ordering::Relaxed);
                    break;
                }
                // Parking brake set mid-roll is the pilot STOPPING: close the
                // throttle and hand the aircraft back — never chase the
                // target against the brake at taxi power.
                if xp.get_float(PARK_BRAKE).await.unwrap_or(0.0) > 0.5 {
                    tracing::info!(
                        "speed governor: parking brake set, closing throttle, standing down"
                    );
                    glide(&xp, THROTTLE, 0.0, 400).await;
                    ACTIVE.store(false, Ordering::Relaxed);
                    break;
                }
                // Brake-paused: idle throttle, hands off, waiting on the
                // pedals. Fully released rolls the taxi again; a hand
                // advancing the throttle meanwhile is the pilot taking the
                // aircraft — release for good.
                if paused {
                    let lb = xp.get_float(LEFT_BRAKE).await.unwrap_or(0.0);
                    let rb = xp.get_float(RIGHT_BRAKE).await.unwrap_or(0.0);
                    if lb.max(rb) > PILOT_BRAKE_RESUME {
                        if xp.get_float(THROTTLE).await.is_ok_and(|t| t > 0.15) {
                            tracing::info!(
                                "speed governor: throttle advanced while brake-paused, releasing"
                            );
                            ACTIVE.store(false, Ordering::Relaxed);
                            break;
                        }
                        tokio::time::sleep(std::time::Duration::from_millis(200)).await;
                        continue;
                    }
                    paused = false;
                    last_cmd = None;
                    tracing::info!("speed governor: brakes released, resuming taxi");
                    if let Some(v) = &voice {
                        v.say(
                            crate::voice::Priority::Advisory,
                            format!("Rolling again, {target:.0} knots."),
                        );
                    }
                }
                // The pilot on BOTH toe brakes is a PAUSE, not a hand-back:
                // throttle to idle and wait for the pedals (it used to stand
                // down for good, and every brake press cost a "go" to get the
                // taxi back — resume-on-release asked for 2026-07-13). A
                // single pedal is differential steering (turning the
                // aircraft), which the hold must ride through. Only read the
                // pedals once OUR held-brake command has expired — while it
                // is (or was just) active, the ratios can be our own braking
                // wearing the pilot's shoes.
                let ours_expired = last_brake_cmd
                    .is_none_or(|t| t.elapsed().as_secs_f64() > BRAKE_HOLD_SECS + 0.2);
                if !braking && ours_expired {
                    let lb = xp.get_float(LEFT_BRAKE).await.unwrap_or(0.0);
                    let rb = xp.get_float(RIGHT_BRAKE).await.unwrap_or(0.0);
                    if pilot_took_the_brakes(lb, rb, braking) {
                        tracing::info!(brake = lb.max(rb), "speed governor: pilot braking, throttle idle, paused");
                        glide(&xp, THROTTLE, 0.0, 300).await;
                        last_cmd = Some(0.0);
                        paused = true;
                        if let Some(v) = &voice {
                            v.say(
                                crate::voice::Priority::Advisory,
                                "Brakes yours, throttle's idle. I'll roll again when you're off them.",
                            );
                        }
                        continue;
                    }
                }
                if let (Ok(gs_ms), Ok(thr)) =
                    (xp.get_float(GS_MS).await, xp.get_float(THROTTLE).await)
                {
                    if last_cmd.is_some_and(|lc| (thr - lc).abs() > 0.15) {
                        tracing::info!("speed governor: pilot override, releasing");
                        ACTIVE.store(false, Ordering::Relaxed);
                        break;
                    }
                    let gs_kts = gs_ms * MS_TO_KTS;
                    let now_t = std::time::Instant::now();
                    let raw_trend = match prev_gs {
                        Some((last_gs, last_t)) => {
                            let dt = now_t.duration_since(last_t).as_secs_f64();
                            if (0.05..=2.0).contains(&dt) { (gs_kts - last_gs) / dt } else { 0.0 }
                        }
                        None => 0.0,
                    };
                    prev_gs = Some((gs_kts, now_t));
                    gs_trend = gs_trend * 0.7 + raw_trend * 0.3;
                    let (new_thr, brake) = taxi_control(target, gs_kts, thr, gs_trend);
                    let wrote = new_thr.is_some() || brake > 0.0 || braking;
                    if let Some(t) = new_thr {
                        glide(&xp, THROTTLE, t, 200).await;
                        last_cmd = Some(t);
                    }
                    // Two brake channels, by intent. A STOP (target zero) is
                    // the held INPUT command renewed every tick — the pilot's
                    // own brake key, regular effort, which demonstrably
                    // stops. A speed TRIM is feathered: the law's magnitude
                    // through the plugin's proportional channel, so shaving
                    // two knots doesn't get the same stab as a full stop.
                    // Never silent on failure: a swallowed brake error hid
                    // the broken override for days of flights.
                    if brake > 0.0 {
                        let mut trimmed = false;
                        if target >= 1.0 && !trim_fallback {
                            match xp.set_float(TAXI_BRAKE, brake).await {
                                Ok(()) => trimmed = true,
                                Err(e) => {
                                    trim_fallback = true;
                                    tracing::warn!(
                                        "speed governor: proportional brake unavailable \
                                         ({e:#}), falling back to the brake key"
                                    );
                                }
                            }
                        }
                        if !trimmed {
                            match xp.run_command_for(BRAKES_REGULAR, BRAKE_HOLD_SECS).await {
                                Ok(()) => last_brake_cmd = Some(std::time::Instant::now()),
                                Err(e) => {
                                    tracing::warn!("speed governor: brake command failed: {e:#}")
                                }
                            }
                        }
                    } else if braking && !trim_fallback {
                        // Release the trim channel promptly instead of riding
                        // out its expiry; a failure here is covered by that
                        // same expiry, so it's only worth a debug line.
                        if let Err(e) = xp.set_float(TAXI_BRAKE, 0.0).await {
                            tracing::debug!("speed governor: brake release write failed: {e:#}");
                        }
                    }
                    braking = brake > 0.0;
                    tokio::time::sleep(std::time::Duration::from_millis(if wrote { 100 } else { 300 })).await;
                } else {
                    tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
                }
            }
        }
    }
    tracing::info!("speed governor stopped");
    // Nothing to clean on the way out: the held brake command expires by
    // itself within BRAKE_HOLD_SECS, however this loop ended.
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn governor_law_is_gentle_and_deadbanded() {
        assert_eq!(throttle_adjust(1.0, 0.0), 0.0, "in the groove, steady → hands off");
        assert_eq!(throttle_adjust(-1.2, 0.5), 0.0);
        assert!(throttle_adjust(3.0, 0.0) > 0.0, "slow → more power");
        assert!(throttle_adjust(-3.0, 0.0) < 0.0, "fast → less power");
        // The anti-stall reflex: on speed but DECAYING fast → power comes in
        // before the error exists at all.
        assert!(throttle_adjust(0.0, -2.5) > 0.0, "unwinding needle → power now");
        // And rising speed damps the correction rather than piling on.
        assert!(throttle_adjust(3.0, 3.0) < throttle_adjust(3.0, 0.0));
        // Authority is bounded, asymmetrically: overspeed is a comfort issue,
        // stall is not — the slow side gets the bigger clamp.
        assert_eq!(throttle_adjust(100.0, 0.0), 0.04);
        assert_eq!(throttle_adjust(-100.0, 0.0), -0.015);
        assert_eq!(throttle_adjust(0.0, -10.0), 0.04, "fast decay unlocks urgency");
    }

    #[test]
    fn spoken_targets_are_sanity_bounded() {
        assert_eq!(clamp_target(70.0), 70.0);
        assert_eq!(clamp_target(7.0), 55.0, "mishear can't idle us into a stall");
        assert_eq!(clamp_target(700.0), 250.0);
        assert_eq!(clamp_taxi_target(10.0), 10.0);
        assert_eq!(clamp_taxi_target(60.0), 25.0, "no takeoff-roll 'taxi'");
        assert_eq!(clamp_taxi_target(0.0), 0.0, "hold short is a real target");
    }

    #[test]
    fn taxi_law_throttle_then_brakes() {
        // Too slow: power comes up, capped below anything takeoff-like.
        let (thr, brake) = taxi_control(10.0, 4.0, 0.10, 0.0);
        assert!(thr.unwrap() > 0.10 && thr.unwrap() <= 0.35);
        assert_eq!(brake, 0.0);
        // A little fast with power in: close the throttle before the brakes.
        let (thr, brake) = taxi_control(10.0, 12.0, 0.20, 0.0);
        assert!(thr.unwrap() < 0.20);
        assert_eq!(brake, 0.0, "no braking against thrust");
        // WELL past the target: idle now AND brake — the polite step-down
        // let an SR22 run a 10-knot taxi through 20 (2026-07-11).
        let (thr, brake) = taxi_control(10.0, 16.0, 0.20, 0.0);
        assert_eq!(thr, Some(0.0), "deep overspeed cuts to idle immediately");
        assert!(brake > 0.0 && brake <= 0.6, "{brake}");
        // Too fast at idle, deep: brakes, proportionally, bounded.
        let (thr, brake) = taxi_control(10.0, 18.0, 0.02, 0.0);
        assert_eq!(thr, Some(0.0));
        assert!(brake > 0.0 && brake <= 0.6, "{brake}");
        // In the groove: hands off everything, brakes released.
        assert_eq!(taxi_control(10.0, 10.5, 0.15, 0.0), (None, 0.0));
        // Hold short: idle + parked on the brakes even when stopped.
        assert_eq!(taxi_control(0.0, 0.2, 0.0, 0.0), (Some(0.0), 0.5));
    }

    #[test]
    fn taxi_law_never_brakes_a_speed_trim() {
        // The limit-cycle fix (6.6 ↔ 11.9 kt sawtooth, 2026-07-14): one or
        // two knots over at idle is NOT a braking problem — coast it off.
        let (thr, brake) = taxi_control(10.0, 11.8, 0.02, 0.0);
        assert_eq!(brake, 0.0, "no brake stab to shave two knots");
        assert_eq!(thr, Some(0.0));
        // But a STOP still brakes all the way to the halt once the
        // throttle's closed — idle creep never finishes a stop.
        let (thr, brake) = taxi_control(0.0, 2.5, 0.0, 0.0);
        assert_eq!(thr, Some(0.0));
        assert!(brake > 0.0, "a stop stays on the brakes to the halt");
        // And the deep brake ramps from the boundary instead of stabbing at
        // 0.2 the instant the branch is crossed.
        let (_, just_over) = taxi_control(10.0, 13.2, 0.0, 0.0);
        assert!(just_over < 0.2, "gentle at the boundary: {just_over}");
        let (_, way_over) = taxi_control(10.0, 18.0, 0.0, 0.0);
        assert!(way_over > just_over, "deeper overspeed brakes harder");
    }

    #[test]
    fn taxi_law_anticipates_the_trend() {
        // Decaying toward the target at idle: power comes back BEFORE the
        // speed falls through it — the other half of the limit-cycle fix.
        let steady = taxi_control(10.0, 9.7, 0.05, 0.0);
        assert_eq!(steady, (None, 0.0), "on pace and steady: hands off");
        let (thr, brake) = taxi_control(10.0, 9.7, 0.05, -1.5);
        assert!(thr.unwrap() > 0.05, "decaying through target: power early");
        assert_eq!(brake, 0.0);
        // Accelerating gently through the target with power in: trim it
        // early too, still no brakes.
        let (thr, brake) = taxi_control(10.0, 10.5, 0.20, 0.8);
        assert!(thr.unwrap() < 0.20, "building through target: power off early");
        assert_eq!(brake, 0.0);
        // A genuine SURGE (downhill, +1.5 kt/s) reads as deep overspeed
        // before the speed gets there: idle and brakes, by design.
        let (thr, brake) = taxi_control(10.0, 10.5, 0.20, 1.5);
        assert_eq!(thr, Some(0.0));
        assert!(brake > 0.0, "a surge earns the brakes early");
    }

    #[test]
    fn manual_toe_brakes_make_the_governor_yield() {
        // BOTH pedals down while we're NOT braking (override off → the ratios
        // are the pilot's pedals): the pilot is slowing us, yield.
        assert!(pilot_took_the_brakes(0.2, 0.2, false), "both pedals = slowing");
        assert!(pilot_took_the_brakes(0.5, 0.2, false), "both engaged, uneven, still slowing");
        // A SINGLE pedal is differential braking = steering a turn (the only
        // low-speed steering on a castoring nosewheel). The hold must ride
        // through it, not cut throttle to 0 the moment the pilot turns.
        assert!(!pilot_took_the_brakes(0.4, 0.0, false), "left pedal = steering, ride through");
        assert!(!pilot_took_the_brakes(0.0, 0.4, false), "right pedal = steering, ride through");
        // Pedal rest / sensor noise below the threshold is NOT a takeover.
        assert!(!pilot_took_the_brakes(0.05, 0.03, false));
        assert!(!pilot_took_the_brakes(0.0, 0.0, false));
        // While the governor is braking (override on) the ratios are OUR own
        // command, not the pilot's — never read them as a takeover, or we'd
        // stand ourselves down the instant we touched the brakes.
        assert!(!pilot_took_the_brakes(0.6, 0.6, true), "that's our brake, not theirs");
    }
}
