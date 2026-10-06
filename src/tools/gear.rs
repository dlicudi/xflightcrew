//! Landing gear — `gear(up|down)`. Only registered for retractable-gear airframes
//! (the profile's `retractable_gear` flag); the 172 has fixed gear, so it never
//! gets this tool. Owns the gear handle and a ground safety gate.

use crate::datarefs::ON_GROUND_ANY;
use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::aircraft::GearCommands;
use crate::xplane::XPlane;

/// Gear handle position: 1 = down, 0 = up. Writable.
const GEAR_HANDLE: &str = crate::datarefs::GEAR_HANDLE_DOWN;
/// 1 while any wheel is on the ground — the gate against retracting on the runway.
const IAS_KTS: &str = crate::datarefs::AIRSPEED_KTS;

const CMD_GEAR_UP: &str = "sim/flight_controls/landing_gear_up";
const CMD_GEAR_DOWN: &str = "sim/flight_controls/landing_gear_down";

pub struct Gear {
    /// Gear limit speed (VLO/VLE), KIAS — refuse cycling above it. `None` = no gate.
    pub vlo_kts: Option<f64>,
    /// Move the lever with the stock gear COMMANDS instead of writing the
    /// handle. For an add-on whose lever is its own and whose script replaces
    /// those commands to move it (the Sparky 747: a three-position
    /// `laminar/B747/actuator/gear_handle`, the stock handle its output). The
    /// readback is the stock handle either way.
    pub by_command: bool,
    /// The lever's OWN commands and handle (`gear_commands:`, the Zibo's
    /// `push_button/gear_up` / `gear_down` and `controls/gear_handle_down`):
    /// pressed, and the airframe's handle read back. Wins over `by_command`.
    pub commands: Option<GearCommands>,
}

/// How long the airframe's own handle is given to move after its command.
const HANDLE_WAIT_SECS: f64 = 2.0;
const HANDLE_POLL_MS: u64 = 100;
/// The gear legs, for the blackbox beside the handle: they take seconds to
/// travel, so the reply is the HANDLE's, the legs are recorded.
const GEAR_DEPLOY: &str = "sim/flightmodel2/gear/deploy_ratio";

/// The live airframe's declared gear commands.
fn live_commands() -> Option<GearCommands> {
    crate::aircraft::lever_profile().and_then(|p| p.gear_commands.clone())
}

/// Press the airframe's own gear command and wait for ITS handle to read
/// the position. `Some(true)` there, `Some(false)` it did not move,
/// `None` the handle could not be read.
pub async fn press_and_confirm(xp: &dyn XPlane, g: &GearCommands, up: bool) -> Option<bool> {
    let want = if up { g.handle_up } else { g.handle_down };
    let there = |v: f64| (v - want).abs() < 0.1;
    let before = xp.get_float(&g.handle).await.ok();
    if before.is_some_and(there) {
        return Some(true);
    }
    let _ = xp.run_command(if up { &g.up } else { &g.down }).await;
    let deadline =
        tokio::time::Instant::now() + std::time::Duration::from_secs_f64(HANDLE_WAIT_SECS);
    let mut last;
    let took = loop {
        tokio::time::sleep(std::time::Duration::from_millis(HANDLE_POLL_MS)).await;
        last = xp.get_float(&g.handle).await.ok();
        if last.is_some_and(there) {
            break Some(true);
        }
        if tokio::time::Instant::now() >= deadline {
            break last.map(|_| false);
        }
    };
    let deploy = xp.get_float_array(GEAR_DEPLOY).await.ok();
    crate::blackbox::record(
        "gear",
        serde_json::json!({"event": "by_command", "up": up, "before": before,
                           "handle": last, "took": took, "deploy": deploy}),
    );
    took
}

/// THE gear seam for the loops (takeoff, circuit, follower, autoland, the
/// enroute setup): where the airframe declares its own gear commands they
/// are pressed and the handle read back; elsewhere the stock handle is
/// written, byte-identical to the write it replaced. An `Err` is a handle
/// that did not move.
pub async fn set_handle(xp: &dyn XPlane, up: bool) -> Result<()> {
    match live_commands() {
        None => xp.set_float(GEAR_HANDLE, if up { 0.0 } else { 1.0 }).await,
        Some(g) => match press_and_confirm(xp, &g, up).await {
            Some(true) => Ok(()),
            other => Err(anyhow::anyhow!("the gear handle did not move ({other:?})")),
        },
    }
}

#[derive(Deserialize)]
struct Args {
    /// "up" or "down".
    position: String,
}

#[async_trait]
impl Tool for Gear {
    fn name(&self) -> &str {
        "gear"
    }

    fn description(&self) -> &str {
        "Raise or lower the landing gear. Use for 'gear up', 'gear down', 'landing \
         gear up', 'positive rate, gear up', 'gear down for landing', 'retract the \
         gear', 'put the gear down'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "position": {"type": "string", "enum": ["up", "down"]}
            },
            "required": ["position"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: Args = serde_json::from_value(args.clone())?;
        let up = matches!(
            a.position.to_ascii_lowercase().as_str(),
            "up" | "retract" | "raise"
        );

        // Safety gate: never raise the gear while we're on the ground. A
        // FAILED read defaults to "on the ground" so an unknown state refuses
        // rather than folding the gear on the ramp (every sibling gate errs
        // the safe way; this one used to default to airborne and fail open).
        if up {
            let on_ground = ctx.xplane.get_float(ON_GROUND_ANY).await.unwrap_or(1.0) > 0.5;
            if on_ground {
                return Ok("Negative, we're on the ground. I won't raise the gear.".to_string());
            }
        }
        // Safety gate: don't cycle the gear above its limit speed (VLO/VLE).
        if let Some(vlo) = self.vlo_kts {
            let ias = ctx.xplane.get_float(IAS_KTS).await.unwrap_or(0.0);
            if ias > vlo {
                return Ok(format!(
                    "Negative, {ias:.0} knots is over the {vlo:.0} knot gear limit. Slow down first."
                ));
            }
        }

        let want = if up { "up" } else { "down" };
        if let Some(g) = &self.commands {
            return Ok(match press_and_confirm(ctx.xplane, g, up).await {
                Some(true) => format!("Roger, gear {want}."),
                Some(false) => format!(
                    "The gear handle didn't move. It's still {}.",
                    if up { "down" } else { "up" }
                ),
                None => {
                    format!("Gear {want} selected, but I can't read the handle. Check the lights.")
                }
            });
        }
        if self.by_command {
            ctx.xplane
                .run_command(if up { CMD_GEAR_UP } else { CMD_GEAR_DOWN })
                .await?;
            // The add-on moves its lever on its own thread.
            tokio::time::sleep(std::time::Duration::from_millis(300)).await;
        } else {
            ctx.xplane
                .set_float(GEAR_HANDLE, if up { 0.0 } else { 1.0 })
                .await?;
        }
        // Read the handle back (the gear itself takes seconds to cycle, but the
        // handle reflects the commanded position immediately). Compared with
        // the INTENT: a failed read used to answer with whatever was asked,
        // and a handle that stayed put answered "Roger" with the wrong word.
        match ctx.xplane.get_float(GEAR_HANDLE).await {
            Ok(v) if (v > 0.5) != up => Ok(format!("Roger, gear {want}.")),
            Ok(_) => Ok(format!(
                "The gear handle didn't move. It's still {}.",
                if up { "down" } else { "up" }
            )),
            Err(_) => Ok(format!(
                "Gear {want} selected, but I can't read the handle. Check the lights."
            )),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xplane::FakeXPlane;

    fn ctx<'a>(fake: &'a FakeXPlane, cfg: &'a crate::config::Config) -> ToolCtx<'a> {
        ToolCtx {
            xplane: fake,
            config: cfg,
            voice: None,
            status: None,
            reminders: None,
        }
    }

    #[tokio::test]
    async fn gear_up_refuses_on_the_ground_and_when_the_ground_read_fails() {
        let cfg = crate::config::Config::default();
        // On the ground: refused, and the handle is never touched.
        let grounded = FakeXPlane::with_reads(&[(ON_GROUND_ANY, 1.0)]);
        let out = Gear {
            vlo_kts: None,
            by_command: false,
            commands: None,
        }
        .execute(&json!({"position": "up"}), &ctx(&grounded, &cfg))
        .await
        .unwrap();
        assert!(out.contains("on the ground"), "{out}");
        assert!(
            !grounded.wrote(GEAR_HANDLE),
            "refusal must not move the handle"
        );

        // The ground read FAILS (no preset): an unknown state must refuse too.
        // This gate used to default to airborne and raise the gear on the ramp.
        let unknown = FakeXPlane::new();
        let out = Gear {
            vlo_kts: None,
            by_command: false,
            commands: None,
        }
        .execute(&json!({"position": "up"}), &ctx(&unknown, &cfg))
        .await
        .unwrap();
        assert!(
            out.contains("on the ground"),
            "failed read errs safe: {out}"
        );
        assert!(!unknown.wrote(GEAR_HANDLE));

        // Airborne and under the gear speed: the handle moves.
        let flying =
            FakeXPlane::with_reads(&[(ON_GROUND_ANY, 0.0), (IAS_KTS, 100.0), (GEAR_HANDLE, 0.0)]);
        let out = Gear {
            vlo_kts: Some(150.0),
            by_command: false,
            commands: None,
        }
        .execute(&json!({"position": "up"}), &ctx(&flying, &cfg))
        .await
        .unwrap();
        assert!(out.contains("gear up"), "{out}");
        assert_eq!(flying.last_write(GEAR_HANDLE), Some(0.0));
    }

    fn zibo_gear() -> GearCommands {
        GearCommands {
            up: "laminar/B738/push_button/gear_up".into(),
            down: "laminar/B738/push_button/gear_down".into(),
            handle: "laminar/B738/controls/gear_handle_down".into(),
            handle_up: 0.0,
            handle_down: 1.0,
        }
    }

    /// The Zibo rewrites the stock gear commands ("down" is "off, down"):
    /// its own buttons are pressed and ITS handle read back.
    #[tokio::test(start_paused = true)]
    async fn declared_gear_commands_are_pressed_and_the_own_handle_read_back() {
        let cfg = crate::config::Config::default();
        let g = zibo_gear();
        let xp =
            FakeXPlane::with_reads(&[(ON_GROUND_ANY, 0.0), (IAS_KTS, 180.0), (&g.handle, 1.0)]);
        xp.set_on_command(&g.up, &g.handle, 0.0);
        let tool = Gear {
            vlo_kts: Some(235.0),
            by_command: true,
            commands: Some(g.clone()),
        };
        let out = tool
            .execute(&json!({"position": "up"}), &ctx(&xp, &cfg))
            .await
            .unwrap();
        assert_eq!(out, "Roger, gear up.");
        assert!(xp.ran(&g.up));
        assert!(!xp.ran(CMD_GEAR_UP), "never the stock command it rewrites");
        assert!(!xp.wrote(GEAR_HANDLE) && !xp.wrote(&g.handle));

        // A press that moves nothing is said, never "Roger".
        let dead = FakeXPlane::with_reads(&[(IAS_KTS, 180.0), (&g.handle, 0.0)]);
        let out = tool
            .execute(&json!({"position": "down"}), &ctx(&dead, &cfg))
            .await
            .unwrap();
        assert_eq!(out, "The gear handle didn't move. It's still up.");
        assert_eq!(dead.ran_count(&g.down), 1);
    }
}
