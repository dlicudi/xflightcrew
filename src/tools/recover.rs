//! `recover` — take the controls and fly the aircraft out of an upset into a
//! stable, wings-level state, then hand it to the autopilot.
//!
//! The autopilot can't do this: in a stall it holds the nose UP (deepening the
//! stall), and it won't roll out of a steep bank. So this takes direct control of
//! pitch and roll (`override_joystick_*` + `yoke_*_ratio`), runs a conservative
//! **unstall → roll upright → pitch to level** sequence in a background loop, and
//! once stable engages ALT hold + wing-level and releases the controls.
//!
//! SAFETY: the control law is deliberately gentle (low gains, rate-damped, hard
//! clamped) — it needs in-sim tuning and should be tested with altitude to spare.
//! The pilot takes it back instantly with "my controls" (sets [`ABORT`]).

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use anyhow::Result;
use async_trait::async_trait;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::tools::autopilot::ApConfig;
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

// --- controls (written under override) ---
const OVR_PITCH: &str = "sim/operation/override/override_joystick_pitch";
const OVR_ROLL: &str = "sim/operation/override/override_joystick_roll";
const YOKE_PITCH: &str = "sim/joystick/yoke_pitch_ratio"; // +1 nose up, -1 nose down
const YOKE_ROLL: &str = "sim/joystick/yoke_roll_ratio"; // +1 right, -1 left
const THROTTLE_ALL: &str = "sim/cockpit2/engine/actuators/throttle_ratio_all";

// --- state reads ---
const BANK: &str = "sim/flightmodel/position/phi"; // roll, deg (+ = right wing down)
const PITCH: &str = "sim/flightmodel/position/theta"; // pitch, deg (+ = nose up)
const ROLL_RATE: &str = "sim/flightmodel/position/P"; // deg/s
const PITCH_RATE: &str = "sim/flightmodel/position/Q"; // deg/s
const IAS: &str = "sim/flightmodel/position/indicated_airspeed"; // kt
const STALL_RATIO: &str = "sim/cockpit2/annunciators/stall_warning_ratio"; // 0..1
const ELEV_TRIM: &str = "sim/cockpit2/controls/elevator_trim"; // -1 nose down .. +1 nose up

// --- AP handoff (mirror of autopilot.rs constants) ---
const CMD_SERVOS_ON: &str = "sim/autopilot/servos_on";
const CMD_HEADING_MODE: &str = "sim/autopilot/heading";
const CMD_ALT_HOLD: &str = "sim/autopilot/altitude_hold";
const ALT_DIAL: &str = "sim/cockpit2/autopilot/altitude_dial_ft";
const CURRENT_ALT: &str = "sim/cockpit2/gauges/indicators/altitude_ft_pilot";
const CURRENT_HDG: &str = "sim/cockpit2/gauges/indicators/compass_heading_deg_magnetic";
const HEADING_STATUS: &str = "sim/cockpit2/autopilot/heading_status";
const ALT_HOLD_STATUS: &str = "sim/cockpit2/autopilot/altitude_hold_status";

/// Cruise power to settle at after recovery; firewall power while unstalling.
const CRUISE_THROTTLE: f64 = 0.6;
const RECOVERY_THROTTLE: f64 = 0.95;
/// Above this stall-warning ratio, treat it as stalling and unstall first.
const STALL_TRIP: f64 = 0.15;

/// Set true by "my controls" to hand the aircraft straight back to the pilot.
pub static ABORT: AtomicBool = AtomicBool::new(false);
/// True while a recovery is actively flying — so "my controls" can acknowledge it
/// ("you have the aircraft") rather than fall through to the AP-off readback.
pub static ACTIVE: AtomicBool = AtomicBool::new(false);
/// Bumped each recovery so a superseding one (or an abort) stands the old down.
static GEN: AtomicU64 = AtomicU64::new(0);

/// The live state the control law reacts to.
#[derive(Clone, Copy, Debug)]
pub struct RecoveryState {
    pub bank_deg: f64,
    pub pitch_deg: f64,
    pub roll_rate_dps: f64,
    pub pitch_rate_dps: f64,
    pub ias_kts: f64,
    /// Stall-warning ratio, 0 (clean) .. 1 (full stall).
    pub stall_ratio: f64,
    /// The slowest airspeed to hold during recovery (≈1.3×Vs) — below it the law
    /// commands full power; well above it the power comes back to cruise.
    pub min_safe_ias: f64,
}

/// What to command this tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RecoveryCmd {
    pub pitch: f64,
    pub roll: f64,
    /// Set the throttle this tick, if the phase calls for it.
    pub throttle: Option<f64>,
    /// Recovery complete — hand off to the autopilot.
    pub stable: bool,
}

/// The recovery control law. Pure, so it's testable without a sim. Priorities:
/// unstall first (push to reduce AoA, gentle wings, power up), then roll upright
/// and pitch to level — all rate-damped (PD) and hard-clamped so it can't snatch.
pub fn recovery_inputs(s: &RecoveryState) -> RecoveryCmd {
    // Gains — conservative; tune live against the "recover" log line.
    const KP_ROLL: f64 = 0.014; // per deg bank
    const KD_ROLL: f64 = 0.010; // per deg/s roll rate (damping)
    const KP_PITCH: f64 = 0.030; // per deg pitch error
    const KD_PITCH: f64 = 0.015; // per deg/s pitch rate (damping)
    const TARGET_PITCH: f64 = 2.0; // slightly nose-up = roughly level flight
    const STALL_PUSH: f64 = 0.55; // nose-down authority to break a stall (decisive)

    // Roll toward wings level: a right bank (phi > 0) needs left yoke (negative).
    // Damp with roll rate so it doesn't overshoot through level.
    let roll = -(KP_ROLL * s.bank_deg + KD_ROLL * s.roll_rate_dps);

    if s.stall_ratio > STALL_TRIP {
        // Unstall: push the nose down (reduce AoA), keep wings only gently level
        // (aggressive aileron near the stall risks a wing drop / spin), full power.
        // The push scales with how deep the stall is.
        let push = -STALL_PUSH * (s.stall_ratio.clamp(0.0, 1.0)).max(0.4);
        return RecoveryCmd {
            pitch: push.clamp(-STALL_PUSH, 0.0),
            roll: roll.clamp(-0.25, 0.25),
            throttle: Some(RECOVERY_THROTTLE),
            stable: false,
        };
    }

    // Flying: roll upright and pitch to level, and keep the energy up — full power
    // while below the safe speed, otherwise leave the throttle where the handoff
    // will set cruise. (Props are driven to full forward once at recovery start.)
    let pitch = KP_PITCH * (TARGET_PITCH - s.pitch_deg) - KD_PITCH * s.pitch_rate_dps;
    let throttle = (s.ias_kts < s.min_safe_ias).then_some(RECOVERY_THROTTLE);
    let stable = s.bank_deg.abs() < 5.0
        && s.pitch_deg.abs() < 8.0
        && s.roll_rate_dps.abs() < 5.0
        && s.stall_ratio <= STALL_TRIP
        && s.ias_kts >= s.min_safe_ias;

    RecoveryCmd {
        // More nose-DOWN authority than up — getting the nose down (out of a stall /
        // nose-high) is the hard, safety-critical direction.
        pitch: pitch.clamp(-0.6, 0.4),
        roll: roll.clamp(-0.6, 0.6),
        throttle,
        stable,
    }
}

pub struct Recover {
    pub cfg: ApConfig,
    /// Constant-speed prop, if any — driven full forward for max power at the start
    /// of a recovery (a turboprop go-around action). `None` = fixed-pitch.
    pub prop: Option<crate::aircraft::PropProfile>,
    /// Clean stall speed (KIAS). The recovery holds ≈1.3×Vs; defaults if unknown.
    pub stall_speed_kts: f64,
}

/// rad/sec → RPM (shared with the prop tool's convention).
const RAD_PER_SEC_TO_RPM: f64 = 9.549_296_585_513_72;

#[async_trait]
impl Tool for Recover {
    fn name(&self) -> &str {
        "recover"
    }
    fn description(&self) -> &str {
        "EMERGENCY: take control and fly the aircraft back to stable, wings-level \
         flight, recovering from a bank, dive, or stall, then hold altitude. Use for \
         'take control', 'you have control', 'recover', 'stabilize', 'level the \
         aircraft', 'save it', 'help'. The pilot takes back control with 'my controls'."
    }
    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        ABORT.store(false, Ordering::SeqCst);
        run_recovery(
            ctx.config.xplane_base_url.clone(),
            self.cfg.clone(),
            self.prop.clone(),
            self.stall_speed_kts,
            ctx.voice.cloned(),
            ctx.status.cloned(),
        );
        Ok("I have the aircraft — recovering. Say 'my controls' to take it back.".to_string())
    }
}

/// Read the current recovery state; `None` if a read fails (skip the tick).
/// `min_safe_ias` is the target speed floor, constant for the recovery.
async fn read_state(xp: &XPlaneClient, min_safe_ias: f64) -> Option<RecoveryState> {
    Some(RecoveryState {
        bank_deg: xp.get_float(BANK).await.ok()?,
        pitch_deg: xp.get_float(PITCH).await.ok()?,
        roll_rate_dps: xp.get_float(ROLL_RATE).await.unwrap_or(0.0),
        pitch_rate_dps: xp.get_float(PITCH_RATE).await.unwrap_or(0.0),
        ias_kts: xp.get_float(IAS).await.ok()?,
        stall_ratio: xp.get_float(STALL_RATIO).await.unwrap_or(0.0),
        min_safe_ias,
    })
}

/// Own thread + runtime + client (like the AP capture watcher): drive the controls
/// in a tight loop until stable or a deadline, then hand to the autopilot and
/// release the override. Releases immediately on [`ABORT`] ("my controls").
fn run_recovery(
    base: String,
    cfg: ApConfig,
    prop: Option<crate::aircraft::PropProfile>,
    stall_speed_kts: f64,
    voice: Option<Voice>,
    status: Option<crate::plugin_status::StatusSink>,
) {
    let my_gen = GEN.fetch_add(1, Ordering::SeqCst) + 1;
    // Hold ~1.3×Vs during the recovery; a sane floor if Vs is unknown.
    let min_safe_ias = (stall_speed_kts * 1.3).max(70.0);
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            return;
        };
        rt.block_on(async move {
            let xp = XPlaneClient::new(base);
            ACTIVE.store(true, Ordering::SeqCst);
            // Take the controls, and give ourselves max power to work with: props
            // full forward (a turboprop go-around action) so the throttle has the
            // full power band available while unstalling/climbing away.
            let _ = xp.set_float(OVR_PITCH, 1.0).await;
            let _ = xp.set_float(OVR_ROLL, 1.0).await;
            if let Some(p) = &prop {
                let _ = xp.set_float(&p.dataref, p.max_rpm / RAD_PER_SEC_TO_RPM).await;
            }

            let mut deadline = std::time::Instant::now() + std::time::Duration::from_secs(25);
            let mut stable_ticks = 0;
            let mut last_phase = ""; // announce ANNOUNCE-mode actions on phase change
            loop {
                // Superseded, aborted, or timed out → release and stop.
                if GEN.load(Ordering::SeqCst) != my_gen
                    || ABORT.load(Ordering::SeqCst)
                    || std::time::Instant::now() >= deadline
                {
                    break;
                }
                // Frozen sim (paused or replay): nothing to read, nothing to
                // write — and the wall-clock deadline must not burn while the
                // aircraft isn't moving, or a mid-recovery pause hands back a
                // half-recovered upset. Slide it by the frozen time.
                if crate::xplane::sim_frozen(&xp).await {
                    deadline += std::time::Duration::from_millis(250);
                    tokio::time::sleep(std::time::Duration::from_millis(250)).await;
                    continue;
                }
                let Some(s) = read_state(&xp, min_safe_ias).await else {
                    tokio::time::sleep(std::time::Duration::from_millis(100)).await;
                    continue;
                };
                let cmd = recovery_inputs(&s);
                let _ = xp.set_float(YOKE_PITCH, cmd.pitch).await;
                let _ = xp.set_float(YOKE_ROLL, cmd.roll).await;
                if let Some(t) = cmd.throttle {
                    let _ = xp.set_float(THROTTLE_ALL, t).await;
                }
                // Re-trim to RELIEVE the yoke: a nose-up trim (from slowing toward a
                // stall) fights a nose-down push, so when we're holding a big pitch
                // input, walk the trim that way. This is what makes "nose down" work
                // when the aircraft is trimmed against us. Rate-limited and bounded.
                if cmd.pitch.abs() > 0.15 {
                    let trim = xp.get_float(ELEV_TRIM).await.unwrap_or(0.0);
                    let step = 0.012 * cmd.pitch.signum();
                    let _ = xp.set_float(ELEV_TRIM, (trim + step).clamp(-0.6, 0.4)).await;
                }
                // ANNOUNCE mode: say what we're DOING as the phase changes (not every
                // tick), rather than the monitor's pilot-directed advice (which is
                // muted while we fly).
                let phase = if s.stall_ratio > STALL_TRIP {
                    "unstall"
                } else if s.bank_deg.abs() > 25.0 {
                    "roll"
                } else {
                    "pitch"
                };
                if phase != last_phase {
                    last_phase = phase;
                    crate::autoland::set_panel(&format!("RECOVERING: {}", phase.to_uppercase()));
                    let line = match phase {
                        "unstall" => "Breaking the stall, nose down, full power.",
                        "roll" => "Rolling wings level.",
                        _ => "Easing the nose to level.",
                    };
                    if let Some(s) = &status {
                        s.action(line);
                    }
                    if let Some(v) = &voice {
                        v.alert(Priority::Warning, line.to_string());
                    }
                }
                tracing::info!(
                    target: "x_copilot::recover",
                    bank = s.bank_deg, pitch = s.pitch_deg, stall = s.stall_ratio,
                    ias = s.ias_kts, yoke_pitch = cmd.pitch, yoke_roll = cmd.roll, "recover"
                );
                crate::blackbox::record("recover", serde_json::json!({
                    "bank": s.bank_deg, "pitch": s.pitch_deg, "stall": s.stall_ratio,
                    "ias": s.ias_kts, "yoke_pitch": cmd.pitch, "yoke_roll": cmd.roll,
                }));
                // Require a few consecutive stable ticks so a momentary pass through
                // level doesn't end the recovery prematurely.
                if cmd.stable {
                    stable_ticks += 1;
                    if stable_ticks >= 3 {
                        break;
                    }
                } else {
                    stable_ticks = 0;
                }
                tokio::time::sleep(std::time::Duration::from_millis(100)).await;
            }

            // Center and release the controls (the sim resumes its own inputs).
            let _ = xp.set_float(YOKE_PITCH, 0.0).await;
            let _ = xp.set_float(YOKE_ROLL, 0.0).await;
            let _ = xp.set_float(OVR_PITCH, 0.0).await;
            let _ = xp.set_float(OVR_ROLL, 0.0).await;
            crate::autoland::set_panel("");
            ACTIVE.store(false, Ordering::SeqCst);

            // If the pilot took it back, don't grab it again via the AP.
            if ABORT.load(Ordering::SeqCst) || GEN.load(Ordering::SeqCst) != my_gen {
                return;
            }

            // Hand to the autopilot: FD up, servos on, then HOLD the current heading
            // AND altitude (heading hold, not just wings-level, so it doesn't drift),
            // and cruise power. The mode commands are toggles — only fire when the
            // mode is off, and write the heading bug LAST (engaging snaps the dial).
            for c in &cfg.fd_commands {
                let _ = xp.run_command(c).await;
            }
            let _ = xp.run_command(CMD_SERVOS_ON).await;
            // Heading hold on the heading we recovered on.
            if xp.get_float(HEADING_STATUS).await.unwrap_or(0.0) <= 0.5 {
                let _ = xp.run_command(CMD_HEADING_MODE).await;
            }
            let hdg = xp.get_float(CURRENT_HDG).await.unwrap_or(0.0);
            let _ = crate::tools::autopilot::drive_heading_bug(&xp, hdg).await;
            // Altitude hold at the current altitude.
            let alt = xp.get_float(CURRENT_ALT).await.unwrap_or(0.0);
            let hold = (alt / 100.0).round() * 100.0;
            let _ = xp.set_float(ALT_DIAL, hold).await;
            if xp.get_float(ALT_HOLD_STATUS).await.unwrap_or(0.0) <= 0.5 {
                let _ = xp.run_command(CMD_ALT_HOLD).await;
            }
            let _ = xp.set_float(THROTTLE_ALL, CRUISE_THROTTLE).await;

            let line = format!(
                "Recovered. Autopilot holding heading {hdg:.0} and {hold:.0} feet. You have control when ready."
            );
            if let Some(s) = &status {
                s.action(&line);
            }
            if let Some(v) = &voice {
                v.say(Priority::Warning, line);
            }
        });
    });
}

/// Signal an active recovery to release the controls immediately ("my controls").
/// Returns whether one was actually running, so the caller can acknowledge it
/// ("you have the aircraft") instead of the plain AP-off readback. Safe to call
/// when none is running.
pub fn abort() -> bool {
    ABORT.store(true, Ordering::SeqCst);
    ACTIVE.swap(false, Ordering::SeqCst)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn state(bank: f64, pitch: f64, stall: f64, ias: f64) -> RecoveryState {
        RecoveryState {
            bank_deg: bank,
            pitch_deg: pitch,
            roll_rate_dps: 0.0,
            pitch_rate_dps: 0.0,
            ias_kts: ias,
            stall_ratio: stall,
            min_safe_ias: 90.0,
        }
    }

    #[test]
    fn stall_pushes_nose_down_and_powers_up_before_rolling() {
        // Deep in a stall: nose-down push, gentle roll, full power, not stable.
        let c = recovery_inputs(&state(30.0, 15.0, 0.8, 75.0));
        assert!(
            c.pitch < 0.0,
            "must push nose down to unstall, got {}",
            c.pitch
        );
        assert_eq!(c.throttle, Some(RECOVERY_THROTTLE));
        assert!(c.roll.abs() <= 0.25, "roll stays gentle near the stall");
        assert!(!c.stable);
    }

    #[test]
    fn right_bank_commands_left_roll() {
        // A right bank (phi > 0), flying: command left roll (negative), no power change.
        let c = recovery_inputs(&state(45.0, 0.0, 0.0, 150.0));
        assert!(c.roll < 0.0, "right bank needs left roll, got {}", c.roll);
        assert_eq!(c.throttle, None);
        assert!(!c.stable, "45 deg of bank is not stable");
    }

    #[test]
    fn nose_high_commands_nose_down() {
        // Nose way up, flying: command nose down (negative pitch).
        let c = recovery_inputs(&state(0.0, 25.0, 0.0, 140.0));
        assert!(c.pitch < 0.0, "nose-high needs nose-down, got {}", c.pitch);
    }

    #[test]
    fn wings_level_and_flying_is_stable() {
        let c = recovery_inputs(&state(2.0, 1.5, 0.0, 150.0));
        assert!(c.stable, "wings level, on speed, not stalling → stable");
    }

    #[test]
    fn slow_but_not_stalling_still_commands_full_power() {
        // Below the safe speed (min_safe_ias 90) even without a stall warning:
        // add power and don't call it stable.
        let c = recovery_inputs(&state(0.0, 2.0, 0.0, 80.0));
        assert_eq!(c.throttle, Some(RECOVERY_THROTTLE));
        assert!(!c.stable, "below safe speed is not stable");
    }

    #[test]
    fn a_stall_is_never_reported_stable_even_wings_level() {
        // Wings level but stalling → NOT stable (must unstall first).
        let c = recovery_inputs(&state(0.0, 12.0, 0.6, 80.0));
        assert!(!c.stable);
        assert!(c.pitch < 0.0);
    }

    #[test]
    fn commands_are_always_clamped() {
        // An extreme upset can't produce out-of-range control inputs.
        let c = recovery_inputs(&state(179.0, 89.0, 0.0, 200.0));
        assert!((-1.0..=1.0).contains(&c.pitch) && (-1.0..=1.0).contains(&c.roll));
        // Asymmetric pitch clamp: up to 0.6 nose-down, 0.4 nose-up; roll ±0.6.
        assert!((-0.6..=0.4).contains(&c.pitch) && c.roll.abs() <= 0.6);
    }
}
