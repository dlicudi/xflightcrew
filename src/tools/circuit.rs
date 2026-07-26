//! `circuit` — the copilot flies a full traffic pattern, takeoff to landing.
//!
//! The arming end of [`crate::circuit`]. A circuit BEGINS with a takeoff, so
//! this runs the same arm-time gates as `auto_takeoff` (replay, on the ground,
//! stopped, the ATC clearance when a tower is running, the steer channel), then
//! hands off to the orchestrator, which arms the takeoff loop, flies the
//! pattern on the autopilot, and hands the landing to `autoland`. EXPERIMENTAL,
//! like `autoland` — VMC, over the field, a hand ready.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::aircraft::PropProfile;
use crate::tools::autopilot::ApConfig;
use crate::{autotakeoff, circuit, party_line, tower};

pub struct Circuit {
    pub cfg: ApConfig,
    pub vr_kts: f64,
    pub vy_kts: f64,
    pub climb_fpm: f64,
    pub retract_gear: bool,
    pub takeoff_roll_m: Option<f64>,
    /// Approach speed handed to autoland on final (KIAS).
    pub approach_ias: f64,
    /// Constant-speed prop levers, advanced out of feather before the circuit's
    /// takeoff roll. `None` on the pistons.
    pub prop: Option<PropProfile>,
}

/// Default pattern altitude above the field, feet (a standard light-aircraft
/// circuit height). Not yet pilot-selectable by voice; a constant for now.
const DEFAULT_TPA_AGL_FT: f64 = 1000.0;

#[derive(Deserialize, Default)]
struct Args {
    /// true = cancel the circuit.
    #[serde(default)]
    off: bool,
    /// true = fly a right-hand pattern (default is the standard left-hand).
    #[serde(default)]
    right: bool,
}

#[async_trait]
impl Tool for Circuit {
    fn name(&self) -> &str {
        "circuit"
    }

    fn description(&self) -> &str {
        "The copilot flies a COMPLETE traffic pattern: takeoff, climb, the four \
         pattern legs, and a landing back at the departure runway. Use when the \
         pilot asks for the whole circuit in one go: 'fly a circuit', 'do a \
         circuit', 'take off and come back around', 'take off and land', 'fly \
         the pattern', 'circuits and bumps'. right=true for a right-hand \
         pattern. off=true for 'cancel the circuit'. (For JUST a takeoff use \
         auto_takeoff; for JUST a landing use autoland.)"
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "off": {"type": "boolean", "description": "true = cancel the circuit"},
                "right": {"type": "boolean", "description": "true = right-hand pattern (default left)"}
            }
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        use std::sync::atomic::Ordering::Relaxed;
        let a: Args = serde_json::from_value(args.clone()).unwrap_or_default();
        if a.off {
            return Ok(if circuit::abort() {
                "Circuit cancelled. You have the aircraft.".to_string()
            } else {
                "No circuit running.".to_string()
            });
        }
        if circuit::ACTIVE.load(Relaxed) {
            return Ok("I'm already flying the circuit. Say 'cancel the circuit' to stop.".to_string());
        }
        // The same gates the takeoff arms behind — a circuit starts with one.
        if ctx.xplane.get_float(crate::datarefs::IS_IN_REPLAY).await.is_ok_and(|v| v > 0.5) {
            return Ok("We're in replay. Go back to live and ask again.".to_string());
        }
        if ctx.xplane.get_float(crate::datarefs::ON_GROUND_ANY).await.unwrap_or(1.0) < 0.5 {
            return Ok("We're already flying — say 'land the aircraft' for the landing.".to_string());
        }
        if crate::autotaxi::ACTIVE.load(Relaxed) {
            return Ok(
                "I'm still driving us. When I call us lined up, ask for the circuit again."
                    .to_string(),
            );
        }
        let gs_kts = ctx
            .xplane
            .get_float("sim/flightmodel/position/groundspeed")
            .await
            .unwrap_or(0.0)
            * 1.943_844;
        if gs_kts > 5.0 {
            return Ok("We're still moving. Let us come to a stop, then ask again.".to_string());
        }
        // A circuit's takeoff needs its clearance when a tower is running —
        // same rule as auto_takeoff (autoland skips it, being an emergency).
        match party_line::clearance_state() {
            None | Some(tower::State::TakeoffCleared) => {}
            Some(tower::State::LinedUp) => {
                return Ok(if party_line::takeoff_hold_armed() {
                    "The tower is holding us for traffic. When the clearance comes through, ask me again."
                        .to_string()
                } else {
                    "We're only cleared to line up and wait. No takeoff clearance yet.".to_string()
                });
            }
            Some(_) => {
                return Ok(
                    "We don't have a takeoff clearance. Call the tower, then I'll fly the circuit."
                        .to_string(),
                );
            }
        }
        if !crate::autotaxi::steer_channel_ok(ctx.xplane).await {
            return Ok(
                "I can't steer. The x-copilot plugin needs updating, so this one is yours."
                    .to_string(),
            );
        }
        let (pp, pr) = tokio::join!(
            ctx.xplane.set_float("xcopilot/pitch", 0.0),
            ctx.xplane.set_float("xcopilot/roll", 0.0)
        );
        if pp.is_err() || pr.is_err() {
            return Ok(
                "I can't reach the yoke. The x-copilot plugin needs updating, so this one is yours."
                    .to_string(),
            );
        }
        // Props full forward before the circuit's takeoff roll begins.
        super::prop::advance_for_ground_roll(ctx.xplane, &self.prop).await;
        circuit::start(
            ctx.config.xplane_base_url.clone(),
            ctx.voice.cloned(),
            circuit::Params {
                apt_dat: super::autoland::apt_dat_path(),
                cifp_dir: ctx.config.cifp_dir.clone(),
                fix_data_path: ctx.config.fix_data_path.clone(),
                nav_data_path: ctx.config.nav_data_path.clone(),
                airspace_path: ctx.config.airspace_path.clone(),
                cfg: self.cfg.clone(),
                takeoff: autotakeoff::Params {
                    apt_dat: super::autoland::apt_dat_path(),
                    cfg: self.cfg.clone(),
                    vr_kts: self.vr_kts,
                    vy_kts: self.vy_kts,
                    climb_fpm: self.climb_fpm,
                    retract_gear: self.retract_gear,
                    cleared_runway: party_line::clearance_runway(),
                    needed_m: self.takeoff_roll_m.map_or(500.0, |m| m * 1.4),
                    // Hand off low so the circuit turns crosswind while still
                    // climbing — a normal-size pattern, not a 4 km straightaway.
                    handoff_agl_ft: 500.0,
                },
                approach_ias: self.approach_ias,
                tpa_agl_ft: DEFAULT_TPA_AGL_FT,
                left_pattern: !a.right,
            },
        );
        Ok("My circuit — takeoff, the pattern, and back around to land. Say 'cancel \
            the circuit' or 'my controls' any time."
            .to_string())
    }
}
