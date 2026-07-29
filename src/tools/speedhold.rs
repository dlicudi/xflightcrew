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
//! Taxiing, the pilot's BRAKES are a pause, not a release: both pedals down —
//! or the keyboard/joystick brake TOGGLE, which never moves the pedals and
//! instead latches the parking-brake ratio at regular effort (~0.36, measured
//! live 2026-07-14) — brings the throttle to idle and the governor sits on
//! its hands; brakes fully off rolls back up to the held target ("resume when
//! I release the brakes", 2026-07-13; "ease off the throttle when I toggle
//! the brakes", 2026-07-14). The full stops stay explicit — the park handle
//! (ratio past 0.5) or a throttle push releases the hold entirely.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx, BRAKES_REGULAR, BRAKE_HOLD_SECS};
use crate::xplane::XPlaneClient;

const IAS: &str = "sim/cockpit2/gauges/indicators/airspeed_kts_pilot";
/// The thrust lever THIS airframe obeys — not always the stock one (the
/// Citation X tracks `throttle_jet_rev_ratio`, so every write to
/// `throttle_ratio_all` went nowhere and it flew a whole pattern at takeoff
/// thrust). Resolved per call, so a profile swap takes effect immediately.
fn throttle_dref() -> String {
    crate::aircraft::throttle_dataref()
}
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

/// The pilot has taken the brakes to SLOW, on either input path, while the
/// governor isn't braking (so the reads reflect hardware, not our own
/// command). The copilot must ease the throttle off rather than power up
/// against the brakes — the same yield it gives a throttle-lever nudge.
///
/// Two paths because X-Plane has two (both measured live, 2026-07-14):
/// - TOE PEDALS drive the left/right ratios. Deliberately `min`, not `max`:
///   a single pedal is DIFFERENTIAL braking, which is how you steer a
///   taxiing aircraft — and on a castoring-nosewheel type (the SR22) it's
///   the ONLY low-speed steering. Standing down on one pedal made the taxi
///   hold cut throttle to 0 every time the pilot turned (2026-07-12). Only
///   both pedals together reads as "I'm slowing us," not "I'm turning."
/// - The brake TOGGLE (keyboard "b" / a joystick button) never moves the
///   pedals: it latches the PARKING-BRAKE ratio at regular effort (~0.36).
///   That sat below the park-handle stand-down (> 0.5), above nothing — the
///   governor throttled against toggled brakes and never saw them
///   (2026-07-14). Anything past pedal-noise on that ratio is the pilot
///   braking; past 0.5 is the park handle, which the caller turns into a
///   full stand-down before this is ever consulted.
///
/// The caller must only trust these reads once `tools::brake_cmd_quiet()` —
/// our own held stop command rides the same parking-brake ratio while active.
fn pilot_took_the_brakes(left: f64, right: f64, parkbrake: f64, governor_braking: bool) -> bool {
    !governor_braking && (left.min(right) > PILOT_BRAKE_YIELD || parkbrake > PILOT_BRAKE_YIELD)
}

/// True while the governor loop runs. Public so other control loops could
/// check it, mirroring `recover::ACTIVE` / `autoland::ACTIVE`.
pub static ACTIVE: AtomicBool = AtomicBool::new(false);

/// True while the governor is actively TAXIING (not a flight hold) — the
/// parking-brake tool uses this to tell the pilot the throttle is coming to
/// idle with the brake.
pub fn taxi_hold_active() -> bool {
    ACTIVE.load(Ordering::Relaxed) && *MODE.lock().unwrap_or_else(|e| e.into_inner()) == Mode::Taxi
}

/// The running taxi hold's target, or `None` when the governor isn't taxiing.
/// The auto-taxi loop reads this to spot an EXTERNAL retarget (a spoken
/// "stop" / "go" / "taxi at 12") and fold it into its own speed schedule.
pub fn taxi_target() -> Option<f64> {
    taxi_hold_active().then(|| *TARGET_KTS.lock().unwrap_or_else(|e| e.into_inner()))
}

/// Retarget a RUNNING taxi hold — the auto-taxi speed schedule's knob (slow
/// for a bend, ramp down to the holding point). No-op unless the governor is
/// actively taxiing: this must never convert a flight hold into a taxi.
/// Deliberately does NOT touch `LAST_TAXI_KTS`, so a schedule write of 0 at
/// the hold-short line doesn't eat the pace a spoken "go" would resume.
pub fn set_taxi_target(kts: f64) {
    if taxi_hold_active() {
        *TARGET_KTS.lock().unwrap_or_else(|e| e.into_inner()) = clamp_taxi_target(kts);
    }
}

/// Arm (or retarget) the FLIGHT speed-hold governor from code — the autoflight
/// cruise-power hold. Same Flight-mode arm the `hold_speed` tool does: sets the
/// target and, if the governor isn't already running, starts it on its own
/// thread. Idempotent; a running hold just picks up the new target.
pub fn arm_flight(base_url: String, voice: Option<crate::voice::Voice>, target_kts: f64) {
    *TARGET_KTS.lock().unwrap_or_else(|e| e.into_inner()) = clamp_target(target_kts);
    *MODE.lock().unwrap_or_else(|e| e.into_inner()) = Mode::Flight;
    if !ACTIVE.swap(true, Ordering::Relaxed) {
        std::thread::spawn(move || {
            let Ok(rt) = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
            else {
                ACTIVE.store(false, Ordering::Relaxed);
                return;
            };
            rt.block_on(governor(XPlaneClient::new(base_url), voice));
        });
    }
}

/// Stand the governor down (no-op if idle) — the follower calls this before
/// handing off to autoland, which runs its own speed law and must own the
/// throttle alone.
pub fn release() {
    ACTIVE.store(false, Ordering::Relaxed);
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
    let max_up = if err_kts > 10.0 || trend_kts_s < -3.0 {
        0.04
    } else {
        0.015
    };
    // The fast side scales with urgency too. A flat 0.015 was sized on GA,
    // where nothing accelerates hard at pattern power; a jet off the runway
    // accelerated 125 to 241 knots across one upwind-to-downwind while the
    // circuit's law pulled the lever at the polite rate the whole way (SF50,
    // Luton 2026-07-28). Well past the target, or building fast: pull like
    // you mean it. Overspeed is a comfort problem right up until it's Vmo.
    let max_down = if err_kts < -10.0 || trend_kts_s > 3.0 {
        0.04
    } else {
        0.015
    };
    (p + d).clamp(-max_down, max_up)
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
        // Finish the roll on light pressure; parked effort only once the
        // wheels are still — half brakes arriving at walking pace snapped
        // the aircraft to a halt nose-first ("jerks forward as it brakes",
        // 2026-07-22). The slew carries the squeeze between the two.
        let brake = if gs_kts > 0.3 { 0.18 } else { 0.5 };
        return (Some(0.0), brake); // holding position: idle + brakes
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
        // stabbed the pedals the instant this branch was crossed. Rolling
        // TRIMS (a bend slow-down, target still moving) cap well below a
        // stop's effort: a 12→6 kt bend target braked at 0.45 and threw
        // everyone forward ("still very abrupt", SR22 2026-07-22).
        // Rolling caps came down AGAIN for the SF50 ("braking / slowing down
        // is a bit jerky", 2026-07-28) — the third airframe to say so, and
        // every complaint has moved these numbers the same direction. A stop
        // keeps its 0.6 authority; a rolling trim is passengers' comfort.
        let cap = if target_kts < 1.0 { 0.6 } else { 0.22 };
        (Some(0.0), ((-eff - 3.0) * 0.09 + 0.08).clamp(0.08, cap))
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

/// Smooth the proportional brake toward the law's demand: pedals SQUEEZE
/// on, they don't stab ("braking is TOO jerky", 2026-07-20 — every full
/// stop was the binary brake key at regular effort in one frame, and trim
/// steps hit the channel raw). Asymmetric on purpose: releasing quickly is
/// not a jerk, applying quickly is.
pub fn brake_slew(prev: f64, want: f64, dt_s: f64) -> f64 {
    let dt = dt_s.clamp(0.02, 1.0);
    // 0.35/s apply: the 0.5/s squeeze still read as a stab on the SR22's
    // effective brakes ("still very abrupt", 2026-07-22).
    // 0.28/s apply: 0.35 still drew "a bit jerky" from the SF50's cabin
    // (2026-07-28). Releases stay quick — that is never the jerk.
    let step = if want > prev { 0.28 * dt } else { 1.5 * dt };
    prev + (want - prev).clamp(-step, step)
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
            return Ok(
                "We're airborne — I can't stop the aircraft. Say a speed to hold.".to_string(),
            );
        }
        // ONE owner for the lever. The circuit, a procedure and autoland each
        // manage speed themselves, and arming a second governor on top is a
        // tug-of-war, not a handover: "reduce airspeed to 160" spoken on a
        // circuit's downwind armed this hold against the circuit's own power
        // law, each side re-commanding over the other, and the lever never
        // came down — the aircraft rode X-Plane's overspeed protection at
        // 270 KIAS for three minutes while the circuit commanded a descent it
        // never got (SF50, Luton 2026-07-28). Refuse with the real answer.
        // `off: true` still passes: releasing a hold is always safe.
        if !on_ground && !a.off {
            if crate::circuit::ACTIVE.load(Ordering::Relaxed) {
                return Ok(
                    "The circuit has the throttle — it flies the pattern speed. \
                           Say cancel circuit if you want the speed."
                        .to_string(),
                );
            }
            if crate::autoland::ACTIVE.load(Ordering::Relaxed) {
                return Ok(
                    "The autoland has the throttle. Say cancel autoland if you want the speed."
                        .to_string(),
                );
            }
            if crate::procedure::ACTIVE.load(Ordering::Relaxed) {
                return Ok(
                    "The approach has the throttle. Say cancel the approach if you want the speed."
                        .to_string(),
                );
            }
        }
        // Mode by context: a small number on the ground is a taxi; airborne (or
        // a big number) is a flight hold. A big number on the ground is neither.
        let taxi = on_ground && a.knots.map_or(a.preset.is_none(), |k| k <= TAXI_MAX_KTS);
        if on_ground && !taxi {
            return Ok("We're on the ground — say a taxi speed like 'taxi at 10 knots', or ask again airborne.".to_string());
        }
        // A crew-held "brakes on" yields to a commanded taxi the same way the
        // parking brake does: rolling was just asked for. A stop target hands
        // over silently, the governor parks on its own boot before the held
        // command expires. Released FIRST: the held command rides the
        // parking-brake ratio, so with the hold still on the check below
        // would read our own braking as the pilot's brake toggle.
        let crew_brakes_released = taxi && super::controls::release_held_brakes();
        // A commanded taxi carries the intent to ROLL: release the brakes
        // ourselves and say so — "release it first" sent the pilot to do the
        // copilot's job. Past 0.5 the ratio is the park HANDLE; the band
        // between pedal-noise and 0.5 is the keyboard/joystick brake TOGGLE's
        // regular-effort latch (~0.36, measured 2026-07-14) — unless a crew
        // hold was JUST dropped above, whose decaying command sits in that
        // same band and needs no release. Through the ONE valve-aware routine
        // the parking-brake tool uses: the ratio-only shortcut said "parking
        // brake off" on the SR22 while the valve held the wheels
        // (2026-07-11). A stop target with the brake set needs no governor.
        let mut brake_released = false;
        let pb = ctx.xplane.get_float(PARK_BRAKE).await.unwrap_or(0.0);
        if taxi && (pb > 0.5 || (pb > PILOT_BRAKE_YIELD && !crew_brakes_released)) {
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
                        "No approach speed on file for this airframe — give me a number."
                            .to_string(),
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
                let Ok(rt) = tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
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
    // Our own brake pollution, by channel: the proportional channel drives
    // the PEDAL ratios (gated by `braking`), and the held stop command rides
    // the PARKING-BRAKE ratio (gated by the shared `brake_cmd_quiet` clock,
    // which also covers the crew's `hold_brakes` holder). Both gates must
    // pass before a brake read is believed to be the pilot's.
    // The proportional channel failed once (plugin absent or old): warned,
    // trims fall back to the held command for the rest of this governor.
    let mut trim_fallback = false;
    // The slewed brake actually written (the squeeze), and its clock.
    let mut brake_out = 0.0_f64;
    let mut last_brake_t: Option<std::time::Instant> = None;
    // Brake-paused: the pilot pressed both pedals mid-taxi, the throttle went
    // to idle, and the hold is waiting for the pedals to come fully off.
    let mut paused = false;
    // The throttle position this loop last commanded. If the lever is later
    // found far from where we put it, that's the PILOT's hand — yield
    // rather than fight them for the lever (a taxi hold once yanked the
    // throttle back from a takeoff push; never again).
    let mut last_cmd: Option<f64> = None;
    // One deviated read is NOT yet a hand: a hardware-axis micro-bump
    // re-drives the dataref to wherever the physical lever RESTS (the jump
    // size says nothing about the movement size), and that killed an auto
    // taxi into a silent runaway (Gatwick, 2026-07-20). So the first
    // deviation is challenged — re-command ours once and look again next
    // tick. A hand keeps re-driving over the re-command and wins on the
    // second read (~a tick of contest, imperceptible); a bump doesn't and
    // the hold rides on.
    let mut override_pending = false;
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
        // Frozen sim (paused or replay): both laws are P+D against a state
        // that isn't moving — the error never changes, so the throttle (or
        // brake) command walks to its clamp and springs on unfreeze. Hold
        // everything exactly as it is and wait for live. The trend windows
        // (0.2–2 s) discard the stale sample on resume.
        if crate::xplane::sim_frozen(&xp).await {
            tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            continue;
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
                if let (Ok(ias), Ok(thr)) = (
                    xp.get_float(IAS).await,
                    xp.get_float(&throttle_dref()).await,
                ) {
                    if let Some(lc) = last_cmd.filter(|lc| (thr - lc).abs() > 0.15) {
                        if override_pending {
                            tracing::info!(thr, lc, "speed governor: pilot override, releasing");
                            ACTIVE.store(false, Ordering::Relaxed);
                            break;
                        }
                        override_pending = true;
                        tracing::info!(
                            thr,
                            lc,
                            "speed governor: throttle deviated, challenging once"
                        );
                        let _ = xp.set_float(&throttle_dref(), lc).await;
                        tokio::time::sleep(std::time::Duration::from_millis(250)).await;
                        continue;
                    }
                    override_pending = false;
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
                            if (0.2..=2.0).contains(&dt) {
                                (ias - last_ias) / dt
                            } else {
                                0.0
                            }
                        }
                        None => 0.0,
                    };
                    prev_ias = Some((ias, now_t));
                    let adj = throttle_adjust(target - ias, trend);
                    if adj != 0.0 {
                        let cmd = (thr + adj).clamp(0.0, 1.0);
                        glide(&xp, &throttle_dref(), cmd, 200).await; // ~most of the tick
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
                // The parking-brake ratio, read every tick: the park HANDLE
                // (past 0.5) set mid-roll is the pilot STOPPING — close the
                // throttle and hand the aircraft back, never chase the target
                // against the brake at taxi power. Below that, the ratio can
                // be the pilot's brake TOGGLE (regular effort ~0.36), which
                // feeds the pause logic further down.
                let pb = xp.get_float(PARK_BRAKE).await.unwrap_or(0.0);
                if pb > 0.5 {
                    tracing::info!(
                        "speed governor: parking brake set, closing throttle, standing down"
                    );
                    glide(&xp, &throttle_dref(), 0.0, 400).await;
                    ACTIVE.store(false, Ordering::Relaxed);
                    break;
                }
                // Brake-paused: idle throttle, hands off, waiting on the
                // brakes — pedals AND the toggle latch, whichever the pilot
                // used. Fully released rolls the taxi again; a hand advancing
                // the throttle meanwhile is the pilot taking the aircraft —
                // release for good.
                if paused {
                    let lb = xp.get_float(LEFT_BRAKE).await.unwrap_or(0.0);
                    let rb = xp.get_float(RIGHT_BRAKE).await.unwrap_or(0.0);
                    if lb.max(rb).max(pb) > PILOT_BRAKE_RESUME {
                        if xp.get_float(&throttle_dref()).await.is_ok_and(|t| t > 0.15) {
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
                // The pilot on the brakes — BOTH toe pedals, or the brake
                // toggle's parking-brake latch — is a PAUSE, not a hand-back:
                // throttle to idle and wait for the brakes (it used to stand
                // down for good, and every brake press cost a "go" to get the
                // taxi back — resume-on-release asked for 2026-07-13; the
                // toggle went entirely unseen and got throttled against,
                // 2026-07-14). A single pedal is differential steering
                // (turning the aircraft), which the hold must ride through.
                // Only believed once every crew brake command is quiet — the
                // held stop rides the same parking-brake ratio while active.
                if !braking && super::brake_cmd_quiet() {
                    let lb = xp.get_float(LEFT_BRAKE).await.unwrap_or(0.0);
                    let rb = xp.get_float(RIGHT_BRAKE).await.unwrap_or(0.0);
                    if pilot_took_the_brakes(lb, rb, pb, braking) {
                        tracing::info!(
                            brake = lb.max(rb).max(pb),
                            "speed governor: pilot braking, throttle idle, paused"
                        );
                        glide(&xp, &throttle_dref(), 0.0, 300).await;
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
                if let (Ok(gs_ms), Ok(thr)) = (
                    xp.get_float(GS_MS).await,
                    xp.get_float(&throttle_dref()).await,
                ) {
                    if let Some(lc) = last_cmd.filter(|lc| (thr - lc).abs() > 0.15) {
                        if override_pending {
                            tracing::info!(thr, lc, "speed governor: pilot override, releasing");
                            ACTIVE.store(false, Ordering::Relaxed);
                            break;
                        }
                        override_pending = true;
                        tracing::info!(
                            thr,
                            lc,
                            "speed governor: throttle deviated, challenging once"
                        );
                        let _ = xp.set_float(&throttle_dref(), lc).await;
                        tokio::time::sleep(std::time::Duration::from_millis(250)).await;
                        continue;
                    }
                    override_pending = false;
                    let gs_kts = gs_ms * MS_TO_KTS;
                    let now_t = std::time::Instant::now();
                    let raw_trend = match prev_gs {
                        Some((last_gs, last_t)) => {
                            let dt = now_t.duration_since(last_t).as_secs_f64();
                            if (0.05..=2.0).contains(&dt) {
                                (gs_kts - last_gs) / dt
                            } else {
                                0.0
                            }
                        }
                        None => 0.0,
                    };
                    prev_gs = Some((gs_kts, now_t));
                    gs_trend = gs_trend * 0.7 + raw_trend * 0.3;
                    let (new_thr, brake) = taxi_control(target, gs_kts, thr, gs_trend);
                    let wrote = new_thr.is_some() || brake > 0.0 || braking;
                    if let Some(t) = new_thr {
                        glide(&xp, &throttle_dref(), t, 200).await;
                        last_cmd = Some(t);
                    }
                    // Brakes go through the plugin's proportional channel —
                    // BOTH toe ratios, equally, every frame — with the demand
                    // slewed so pedals squeeze instead of stab ("braking is
                    // TOO jerky", 2026-07-20: every stop was the binary key
                    // at regular effort in one frame). Stops and trims share
                    // the smooth channel now; the binary brake key survives
                    // only as the fallback for a missing/old plugin. Never
                    // silent on failure: a swallowed brake error hid the
                    // broken override for days of flights.
                    let now_bt = std::time::Instant::now();
                    let brake_dt = last_brake_t.map_or(0.1, |t: std::time::Instant| {
                        now_bt.duration_since(t).as_secs_f64()
                    });
                    last_brake_t = Some(now_bt);
                    brake_out = brake_slew(brake_out, brake, brake_dt);
                    // The brake trace was invisible: "braking is jerky" has
                    // been reported three times and diagnosed by feel every
                    // time. Log only while anything is happening.
                    if brake_out > 0.005 || brake > 0.0 {
                        crate::blackbox::record(
                            "speedhold",
                            serde_json::json!({
                                "event": "brake", "cmd": brake, "out": brake_out,
                                "gs": gs_kts, "target": target, "trend": gs_trend,
                            }),
                        );
                    }
                    if brake_out > 0.005 {
                        let mut smoothed = false;
                        if !trim_fallback {
                            match xp.set_float(TAXI_BRAKE, brake_out).await {
                                Ok(()) => smoothed = true,
                                Err(e) => {
                                    trim_fallback = true;
                                    tracing::warn!(
                                        "speed governor: proportional brake unavailable \
                                         ({e:#}), falling back to the brake key"
                                    );
                                }
                            }
                        }
                        if !smoothed && brake > 0.0 {
                            match xp.run_command_for(BRAKES_REGULAR, BRAKE_HOLD_SECS).await {
                                Ok(()) => super::note_brake_cmd(),
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
                        brake_out = 0.0;
                    }
                    braking = brake_out > 0.005;
                    tokio::time::sleep(std::time::Duration::from_millis(if wrote {
                        100
                    } else {
                        300
                    }))
                    .await;
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
    fn brakes_squeeze_on_and_let_go_quickly() {
        // A demanded 0.5 park from nothing arrives over ~a second of
        // squeeze, never in one frame (the head-nod at every stop).
        let first = brake_slew(0.0, 0.5, 0.1);
        assert!(first <= 0.06, "first tick is a touch, got {first}");
        let mut b = 0.0;
        let mut ticks = 0;
        while b < 0.49 && ticks < 50 {
            b = brake_slew(b, 0.5, 0.1);
            ticks += 1;
        }
        assert!(
            (10..=18).contains(&ticks),
            "~1.5 s to full park, took {ticks} ticks"
        );
        // Release is quick — letting go is not a jerk.
        let released = brake_slew(0.5, 0.0, 0.1);
        assert!(released < 0.4, "{released}");
        // Small trims pass through unslewed.
        assert_eq!(brake_slew(0.2, 0.22, 0.1), 0.22);
    }

    #[test]
    fn governor_law_is_gentle_and_deadbanded() {
        assert_eq!(
            throttle_adjust(1.0, 0.0),
            0.0,
            "in the groove, steady → hands off"
        );
        assert_eq!(throttle_adjust(-1.2, 0.5), 0.0);
        assert!(throttle_adjust(3.0, 0.0) > 0.0, "slow → more power");
        assert!(throttle_adjust(-3.0, 0.0) < 0.0, "fast → less power");
        // The anti-stall reflex: on speed but DECAYING fast → power comes in
        // before the error exists at all.
        assert!(
            throttle_adjust(0.0, -2.5) > 0.0,
            "unwinding needle → power now"
        );
        // And rising speed damps the correction rather than piling on.
        assert!(throttle_adjust(3.0, 3.0) < throttle_adjust(3.0, 0.0));
        // Authority is bounded, asymmetrically: overspeed is a comfort issue,
        // stall is not — the slow side gets the bigger clamp.
        assert_eq!(throttle_adjust(100.0, 0.0), 0.04);
        // The fast side pulls with the same urgency it pushes: a flat -0.015
        // let a jet run 125 to 270 knots through a whole circuit (2026-07-28).
        assert_eq!(throttle_adjust(-100.0, 0.0), -0.04);
        // ...but a small overspeed still gets the polite step.
        assert_eq!(throttle_adjust(-5.0, 0.0), -0.015);
        // Building fast counts as urgent even before the error is big.
        assert_eq!(throttle_adjust(-5.0, 4.0), -0.04);
        assert_eq!(
            throttle_adjust(0.0, -10.0),
            0.04,
            "fast decay unlocks urgency"
        );
    }

    #[test]
    fn spoken_targets_are_sanity_bounded() {
        assert_eq!(clamp_target(70.0), 70.0);
        assert_eq!(
            clamp_target(7.0),
            55.0,
            "mishear can't idle us into a stall"
        );
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
        // Still rolling into the stop: light pressure finishes the roll —
        // parked effort at walking pace is the nose-first jerk.
        let (thr, brake) = taxi_control(0.0, 1.0, 0.0, 0.0);
        assert_eq!(thr, Some(0.0));
        assert!(brake > 0.0 && brake < 0.25, "gentle finish: {brake}");
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
        assert!(
            thr.unwrap() < 0.20,
            "building through target: power off early"
        );
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
        assert!(
            pilot_took_the_brakes(0.2, 0.2, 0.0, false),
            "both pedals = slowing"
        );
        assert!(
            pilot_took_the_brakes(0.5, 0.2, 0.0, false),
            "both engaged, uneven, still slowing"
        );
        // A SINGLE pedal is differential braking = steering a turn (the only
        // low-speed steering on a castoring nosewheel). The hold must ride
        // through it, not cut throttle to 0 the moment the pilot turns.
        assert!(
            !pilot_took_the_brakes(0.4, 0.0, 0.0, false),
            "left pedal = steering, ride through"
        );
        assert!(
            !pilot_took_the_brakes(0.0, 0.4, 0.0, false),
            "right pedal = steering, ride through"
        );
        // Pedal rest / sensor noise below the threshold is NOT a takeover.
        assert!(!pilot_took_the_brakes(0.05, 0.03, 0.0, false));
        assert!(!pilot_took_the_brakes(0.0, 0.0, 0.02, false));
        // While the governor is braking (override on) the ratios are OUR own
        // command, not the pilot's — never read them as a takeover, or we'd
        // stand ourselves down the instant we touched the brakes.
        assert!(
            !pilot_took_the_brakes(0.6, 0.6, 0.0, true),
            "that's our brake, not theirs"
        );
    }

    #[test]
    fn brake_toggle_latch_makes_the_governor_yield() {
        // The keyboard/joystick brake TOGGLE never moves the pedals: it
        // latches the parking-brake ratio at regular effort (~0.36 measured
        // live, 2026-07-14). That must read as the pilot braking — the
        // governor throttled straight against it before.
        assert!(
            pilot_took_the_brakes(0.0, 0.0, 0.36, false),
            "toggle latch = slowing"
        );
        // A partially-set park handle below the stand-down threshold is
        // braking all the same.
        assert!(pilot_took_the_brakes(0.0, 0.0, 0.3, false));
        // The decay tail after our own held command expires (~0.02 within a
        // second) is noise, not a takeover — and the fresher, higher tail is
        // excluded by the shared brake_cmd_quiet clock, not this threshold.
        assert!(
            !pilot_took_the_brakes(0.0, 0.0, 0.02, false),
            "decay tail is not the pilot"
        );
        // While the governor itself is braking, nothing reads as the pilot.
        assert!(
            !pilot_took_the_brakes(0.0, 0.0, 0.36, true),
            "our stop, not their toggle"
        );
    }
}
