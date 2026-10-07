//! `reverse_thrust` — the copilot pulls the power levers into reverse on the
//! landing rollout, then stows them as the speed decays.
//!
//! Turboprop reverse is a rollout tool with a sharp edge: selected in flight it
//! is unrecoverable on most types, and held to a standstill it sandblasts the
//! props. So the gates are absolute — on the ground, rolling, engines running —
//! and a watcher stows automatically below taxi speed (SOP: out of reverse by
//! ~25 knots) or the instant the wheels leave again (a bounce must not go
//! around with the levers aft). The pilot can stow early ("cancel reverse") or
//! take the levers physically — the watcher yields to a hand on the quadrant
//! rather than fight it, same contract as the speed governor.
//!
//! Lever convention (X-Plane `throttle_beta_rev_ratio`): 0..1 forward thrust,
//! 0..-1 beta, -1..-2 reverse. Full reverse = -2. The per-engine array is used
//! (not an `_all` variant) — it's the actuator the RWDesigns Twin Otter reads
//! for its own levers, so writes are guaranteed to reach the model.

use std::sync::atomic::{AtomicBool, Ordering};

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::xplane::{XPlane, XPlaneClient};

const BETA_REV: &str = "sim/cockpit2/engine/actuators/throttle_beta_rev_ratio";
const ENGN_RUNNING: &str = crate::datarefs::ENGINE_RUNNING;
const GS_MS: &str = crate::datarefs::GROUNDSPEED_MS;
const MS_TO_KTS: f64 = 1.943_844;

/// Full reverse on the lever scale (see module docs).
const FULL_REVERSE: f64 = -2.0;
/// Stow (back to ground idle) as groundspeed decays through this.
pub const STOW_KTS: f64 = 25.0;
/// One hold of a declared reverse command, s; re-held every
/// [`HOLD_REISSUE_S`] so the holds overlap and the lever never leaves full.
const HOLD_CHUNK_S: f64 = 1.5;
const HOLD_REISSUE_S: f64 = 1.0;
/// Below this there's no rollout to slow — refuse to engage.
const MIN_ENGAGE_KTS: f64 = 5.0;

/// True while reverse is in and the watcher runs. Mirrors `speedhold::ACTIVE`.
/// Written only by the watcher's run ([`WATCH`]).
pub static ACTIVE: AtomicBool = AtomicBool::new(false);
/// The rollout watcher's run: claimed by the tool, cancelled by [`stow`].
pub(crate) static WATCH: sim::run::Slot = sim::run::Slot::flagged(&ACTIVE);

/// Stow reverse: the watcher sees its run end and glides the levers to idle.
/// Returns whether reverse was in.
pub fn stow() -> bool {
    WATCH.cancel()
}

/// One-line status for the dashboard's "managing" panel.
pub fn status() -> Option<String> {
    ACTIVE
        .load(Ordering::Relaxed)
        .then(|| "reverse thrust".to_string())
}

/// The stow decision, pure for tests: stow when the speed has decayed to taxi
/// pace, or the instant the aircraft is airborne again (a bounced landing must
/// never fly with the levers aft of idle).
pub fn should_stow(gs_kts: f64, on_ground: bool) -> bool {
    !on_ground || gs_kts < STOW_KTS
}

pub struct ReverseThrust {
    /// How many engines' lever elements to drive.
    pub engines: usize,
}

#[derive(Deserialize)]
struct Args {
    /// true = select reverse (default); false = stow back to idle.
    #[serde(default = "default_on")]
    on: bool,
}

fn default_on() -> bool {
    true
}

#[async_trait]
impl Tool for ReverseThrust {
    fn name(&self) -> &str {
        "reverse_thrust"
    }

    fn description(&self) -> &str {
        "Select reverse thrust on the landing rollout (ground only, engines \
         running, rolling). Stows itself below 25 knots. Use for 'reverse \
         thrust', 'full reverse', 'reversers'. Stow early with on=false — \
         'cancel reverse', 'stow reverse', 'out of reverse'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "on": {"type": "boolean", "description": "true = select reverse (default), false = stow back to idle"}
            }
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: Args = serde_json::from_value(args.clone())?;
        if !a.on {
            return Ok(if stow() {
                // The watcher sees the flag drop and glides the levers to idle.
                "Stowing reverse.".to_string()
            } else {
                "Reverse isn't in.".to_string()
            });
        }
        // The gates, in the order a refusal is most useful.
        if ctx.xplane.get_float(crate::datarefs::ON_GROUND_ANY).await.unwrap_or(0.0) < 0.5 {
            return Ok(
                "Negative — reverse stays stowed in the air. Ask again on the rollout.".to_string(),
            );
        }
        let running = ctx
            .xplane
            .get_float_array(ENGN_RUNNING)
            .await
            .unwrap_or_default();
        if !running.iter().take(self.engines.max(1)).any(|&r| r > 0.5) {
            return Ok("Engines aren't running.".to_string());
        }
        let gs_kts = ctx.xplane.get_float(GS_MS).await.unwrap_or(0.0) * MS_TO_KTS;
        if gs_kts < MIN_ENGAGE_KTS {
            return Ok("We're stopped — no call for reverse.".to_string());
        }
        let _ = select(
            ctx.xplane,
            ctx.config.xplane_base_url.clone(),
            self.engines,
            "pilot",
        )
        .await;
        Ok(format!(
            "Reverse thrust — stowing below {STOW_KTS:.0} knots, or say cancel reverse."
        ))
    }
}

/// Select reverse and start the watcher: the pilot's call and autoland's
/// rollout ([`rollout_reverse_due`]) go through this one door, so the stow at
/// [`STOW_KTS`], the stow on a bounce and the yield to a hand are the same for
/// both. The caller has checked the gates. `by` is blackboxed. Returns
/// whether the lever READ BACK in reverse (`None`: no reading).
pub async fn select(xp: &dyn XPlane, base: String, engines: usize, by: &str) -> Option<bool> {
    // Reverse owns the power levers: the speed governor lets go first, and an
    // engaged autothrottle comes out so it does not drive them back up.
    if super::speedhold::release() {
        tracing::info!("reverse thrust: released the speed governor");
    }
    let at_off = crate::autoflight::drop_thrust_now(xp, "reverse").await;
    let engines = engines.max(1);
    let hold = crate::aircraft::reverse_hold_command();
    let lever = match &hold {
        // The airframe's reverse is a held command (the Zibo: its levers undo
        // a beta write; held, the reversers deploy and the lever runs to
        // full in about 3 s, stowing on release; probed parked 2026-10-07).
        Some(cmd) => {
            let _ = xp.run_command_for(cmd, HOLD_CHUNK_S).await;
            tokio::time::sleep(std::time::Duration::from_millis(700)).await;
            xp.get_float_array(BETA_REV).await.ok().and_then(|v| v.first().copied())
        }
        None => {
            glide_levers(xp, engines, FULL_REVERSE, 400).await;
            xp.get_float_array(BETA_REV).await.ok().and_then(|v| v.first().copied())
        }
    };
    crate::blackbox::record(
        "reverse",
        json!({"event": "selected", "by": by, "engines": engines,
            "lever": lever, "at_off": at_off}),
    );
    // Watcher on its own thread + runtime (the main runtime blocks on the
    // mic between commands — same reasoning as the speed governor).
    if let Some(run) = WATCH.claim() {
        std::thread::spawn(move || {
            let Ok(rt) = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
            else {
                return;
            };
            rt.block_on(watcher(XPlaneClient::new(base), engines, hold, &run));
        });
    }
    lever.map(|l| l <= -0.9)
}

/// Wheels on this long before autoland selects reverse: the mains touch
/// first, and reverse with the nose still in the air pitches it up. No
/// nosewheel dataref is read; a second and a half settled is the proxy.
pub const ROLLOUT_WHEELS_ON_S: f64 = 1.5;
/// Below this on the rollout reverse is not worth selecting (it would stow
/// again within seconds of [`STOW_KTS`]).
pub const ROLLOUT_MIN_KTS: f64 = 60.0;

/// Autoland's rollout selects reverse once, on an airframe that declares
/// `rollout_reverse`, when the wheels have been on [`ROLLOUT_WHEELS_ON_S`]
/// without a bounce and there is still speed to take off. Pure.
pub fn rollout_reverse_due(
    declared: bool,
    already: bool,
    airborne: bool,
    wheels_on_s: Option<f64>,
    gs_kts: f64,
) -> bool {
    declared
        && !already
        && !airborne
        && wheels_on_s.is_some_and(|s| s >= ROLLOUT_WHEELS_ON_S)
        && gs_kts >= ROLLOUT_MIN_KTS
}

/// Glide all engines' levers to `to` in 50 ms micro-steps, together — one hand
/// across the quadrant, not a stepped jump.
async fn glide_levers(xp: &dyn XPlane, engines: usize, to: f64, over_ms: u64) {
    let from = match xp.get_float_array(BETA_REV).await {
        Ok(v) => v.first().copied().unwrap_or(0.0),
        Err(_) => 0.0,
    };
    let steps = (over_ms / 50).max(1);
    for i in 1..=steps {
        let v = from + (to - from) * (i as f64 / steps as f64);
        for e in 0..engines {
            let _ = xp.set_float_array_elem(BETA_REV, e, v).await;
        }
        if i < steps {
            tokio::time::sleep(std::time::Duration::from_millis(50)).await;
        }
    }
}

/// The rollout watcher: stow at taxi speed or on liftoff, yield to the pilot's
/// hand, glide to idle on any exit except a pilot override.
async fn watcher(xp: XPlaneClient, engines: usize, hold: Option<String>, run: &sim::run::Lease<'_>) {
    tracing::info!("reverse thrust in, watcher started");
    let mut pilot_has_levers = false;
    let mut last_hold = std::time::Instant::now();
    loop {
        // Released by voice ("cancel reverse" / "my throttle") — stow and go.
        if !run.live() {
            break;
        }
        let lever = xp
            .get_float_array(BETA_REV)
            .await
            .ok()
            .and_then(|v| v.first().copied())
            .unwrap_or(FULL_REVERSE);
        // A lever found well out of reverse is the PILOT stowing by hand —
        // yield, and don't move what's now theirs. Not on a held command:
        // the lever reads 0 while the reversers deploy, and a hand there is
        // the pilot's own "cancel reverse".
        if hold.is_none() && lever > -0.8 {
            tracing::info!("reverse thrust: pilot has the levers, yielding");
            pilot_has_levers = true;
            run.lower_flag();
            break;
        }
        let gs_kts = xp.get_float(GS_MS).await.unwrap_or(0.0) * MS_TO_KTS;
        let on_ground = xp
            .get_float(crate::datarefs::ON_GROUND_ANY)
            .await
            .unwrap_or(1.0)
            > 0.5;
        if should_stow(gs_kts, on_ground) {
            tracing::info!(gs_kts, on_ground, "reverse thrust: stowing");
            run.lower_flag();
            break;
        }
        // A held command is re-held before the last hold runs out.
        if let Some(cmd) = &hold {
            if last_hold.elapsed().as_secs_f64() >= HOLD_REISSUE_S {
                let _ = xp.run_command_for(cmd, HOLD_CHUNK_S).await;
                last_hold = std::time::Instant::now();
            }
        }
        tokio::time::sleep(std::time::Duration::from_millis(200)).await;
    }
    // A watcher superseded by a newer reverse leaves the levers to it. A
    // held command stows itself as its last hold runs out.
    if hold.is_none() && !pilot_has_levers && run.current() {
        glide_levers(&xp, engines, 0.0, 400).await;
    }
    tracing::info!("reverse thrust stowed, watcher stopped");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stow_law() {
        // Rolling out fast on the runway: reverse stays in.
        assert!(!should_stow(60.0, true));
        assert!(!should_stow(26.0, true));
        // Decayed to taxi pace: stow (props don't eat gravel).
        assert!(should_stow(24.0, true));
        assert!(should_stow(0.0, true));
        // Airborne again — a bounce — stow IMMEDIATELY regardless of speed.
        assert!(should_stow(80.0, false));
        assert!(should_stow(24.0, false));
    }

    #[test]
    fn autoland_selects_reverse_once_on_settled_wheels() {
        // Settled on the wheels, fast, declared: select.
        assert!(rollout_reverse_due(true, false, false, Some(2.0), 120.0));
        // Not declared: the pilot's call only.
        assert!(!rollout_reverse_due(false, false, false, Some(2.0), 120.0));
        // Once per landing.
        assert!(!rollout_reverse_due(true, true, false, Some(2.0), 120.0));
        // Mains only just down (the nose still flying), or a bounce.
        assert!(!rollout_reverse_due(true, false, false, Some(0.5), 120.0));
        assert!(!rollout_reverse_due(true, false, false, None, 120.0));
        assert!(!rollout_reverse_due(true, false, true, Some(2.0), 120.0));
        // Too slow to be worth it.
        assert!(!rollout_reverse_due(true, false, false, Some(4.0), 50.0));
    }
}
