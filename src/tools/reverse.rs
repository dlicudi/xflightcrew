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
const ENGN_RUNNING: &str = "sim/flightmodel/engine/ENGN_running";
const GS_MS: &str = "sim/flightmodel/position/groundspeed";
const MS_TO_KTS: f64 = 1.943_844;

/// Full reverse on the lever scale (see module docs).
const FULL_REVERSE: f64 = -2.0;
/// Stow (back to ground idle) as groundspeed decays through this.
pub const STOW_KTS: f64 = 25.0;
/// Below this there's no rollout to slow — refuse to engage.
const MIN_ENGAGE_KTS: f64 = 5.0;

/// True while reverse is in and the watcher runs. Mirrors `speedhold::ACTIVE`.
pub static ACTIVE: AtomicBool = AtomicBool::new(false);

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
            return Ok(if ACTIVE.swap(false, Ordering::Relaxed) {
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
        // Reverse owns the power levers: the speed governor lets go first.
        if super::speedhold::ACTIVE.swap(false, Ordering::Relaxed) {
            tracing::info!("reverse thrust: released the speed governor");
        }
        let engines = self.engines.max(1);
        glide_levers(ctx.xplane, engines, FULL_REVERSE, 400).await;
        // Watcher on its own thread + runtime (the main runtime blocks on the
        // mic between commands — same reasoning as the speed governor).
        if !ACTIVE.swap(true, Ordering::Relaxed) {
            let base = ctx.config.xplane_base_url.clone();
            std::thread::spawn(move || {
                let Ok(rt) = tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                else {
                    ACTIVE.store(false, Ordering::Relaxed);
                    return;
                };
                rt.block_on(watcher(XPlaneClient::new(base), engines));
            });
        }
        Ok(format!(
            "Reverse thrust — stowing below {STOW_KTS:.0} knots, or say cancel reverse."
        ))
    }
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
async fn watcher(xp: XPlaneClient, engines: usize) {
    tracing::info!("reverse thrust in, watcher started");
    let mut pilot_has_levers = false;
    loop {
        // Released by voice ("cancel reverse" / "my throttle") — stow and go.
        if !ACTIVE.load(Ordering::Relaxed) {
            break;
        }
        let lever = xp
            .get_float_array(BETA_REV)
            .await
            .ok()
            .and_then(|v| v.first().copied())
            .unwrap_or(FULL_REVERSE);
        // A lever found well out of reverse is the PILOT stowing by hand —
        // yield, and don't move what's now theirs.
        if lever > -0.8 {
            tracing::info!("reverse thrust: pilot has the levers, yielding");
            pilot_has_levers = true;
            ACTIVE.store(false, Ordering::Relaxed);
            break;
        }
        let gs_kts = xp.get_float(GS_MS).await.unwrap_or(0.0) * MS_TO_KTS;
        let on_ground = xp.get_float(crate::datarefs::ON_GROUND_ANY).await.unwrap_or(1.0) > 0.5;
        if should_stow(gs_kts, on_ground) {
            tracing::info!(gs_kts, on_ground, "reverse thrust: stowing");
            ACTIVE.store(false, Ordering::Relaxed);
            break;
        }
        tokio::time::sleep(std::time::Duration::from_millis(200)).await;
    }
    if !pilot_has_levers {
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
}
