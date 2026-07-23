//! `full_auto` — the copilot flies the WHOLE thing from cold: power up, start
//! the engine, request taxi, drive to the runway, line up, and fly a circuit.
//!
//! The arming end of [`crate::full_auto`]. It holds the profile params the
//! sub-tools need and hands a freshly-built set to the orchestrator thread.
//! EXPERIMENTAL — eight subsystems in a chain; VMC, hand ready.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::aircraft::FuelSelectorProfile;
use crate::tools::autopilot::ApConfig;

pub struct FullAuto {
    // engine start
    pub fuel_selector: Option<FuelSelectorProfile>,
    pub alternator_on_cmds: Vec<String>,
    // taxi / line-up
    pub park_brake_valve_set: Option<f64>,
    pub takeoff_roll_m: Option<f64>,
    // circuit / climb-out
    pub cfg: ApConfig,
    pub vr_kts: f64,
    pub vy_kts: f64,
    pub climb_fpm: f64,
    pub retract_gear: bool,
    pub approach_ias: f64,
    /// Turbine airframe → run the APU/electrical power-up before engine start.
    pub power_up_first: bool,
}

#[derive(Deserialize, Default)]
struct Args {
    #[serde(default)]
    off: bool,
    #[serde(default)]
    right: bool,
}

#[async_trait]
impl Tool for FullAuto {
    fn name(&self) -> &str {
        "full_auto"
    }

    fn description(&self) -> &str {
        "Fly the ENTIRE flight autonomously from cold: power up, start the \
         engine, request taxi from the tower, taxi to the runway, line up, take \
         off, fly the pattern, and land. Use for 'full auto', 'do everything', \
         'start up and fly a circuit', 'take us up from cold'. off=true to \
         cancel. (For just the airborne circuit from the runway, use circuit.)"
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "off": {"type": "boolean", "description": "true = cancel full auto"},
                "right": {"type": "boolean", "description": "true = right-hand pattern"}
            }
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        use std::sync::atomic::Ordering::SeqCst;
        let a: Args = serde_json::from_value(args.clone()).unwrap_or_default();
        if a.off {
            return Ok(if crate::full_auto::abort() {
                "Full auto cancelled. You have the aircraft.".to_string()
            } else {
                "Full auto isn't running.".to_string()
            });
        }
        if crate::full_auto::ACTIVE.load(SeqCst) {
            return Ok("I'm already running the full sequence. Say cancel to stop.".to_string());
        }
        if ctx.xplane.get_float(crate::datarefs::IS_IN_REPLAY).await.is_ok_and(|v| v > 0.5) {
            return Ok("We're in replay. Go back to live and ask again.".to_string());
        }

        let sub = crate::full_auto::SubTools {
            start_engine: crate::tools::engine::StartEngine {
                fuel_selector: self.fuel_selector.clone(),
                alternator_on_cmds: self.alternator_on_cmds.clone(),
            },
            taxi: crate::tools::taxi::AutoTaxi { park_brake_valve_set: self.park_brake_valve_set },
            line_up: crate::tools::taxi::LineUp {
                park_brake_valve_set: self.park_brake_valve_set,
                takeoff_roll_m: self.takeoff_roll_m,
            },
            circuit: crate::tools::circuit::Circuit {
                cfg: self.cfg.clone(),
                vr_kts: self.vr_kts,
                vy_kts: self.vy_kts,
                climb_fpm: self.climb_fpm,
                retract_gear: self.retract_gear,
                takeoff_roll_m: self.takeoff_roll_m,
                approach_ias: self.approach_ias,
            },
            right_pattern: a.right,
            power_up_first: self.power_up_first,
        };
        crate::full_auto::start(
            ctx.config.xplane_base_url.clone(),
            ctx.config.clone(),
            ctx.voice.cloned(),
            sub,
        );
        Ok("Full auto. I'll power up, start the engine, get our taxi, line us up, \
            and fly the circuit. Say cancel or my controls any time."
            .to_string())
    }
}
