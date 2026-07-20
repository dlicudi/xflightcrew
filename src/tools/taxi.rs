//! `auto_taxi` — the copilot taxis the aircraft to the holding point.
//!
//! The arming end of [`crate::autotaxi`]: validates that there is a cleared
//! route to drive and that we're actually on it, arms the taxi-speed
//! governor through the same `hold_speed` path a spoken taxi uses (so every
//! parking-brake and brake-toggle gate applies unchanged), then hands the
//! steering to the background loop. Distinct from `hold_speed`'s taxi mode
//! on purpose: that one is "I have the throttle, you steer" — this one is
//! the copilot steering too, and it only exists once ATC has published a
//! route to steer along.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{speedhold, Tool, ToolCtx};
use crate::autotaxi;
use crate::party_line;

/// Default pace when the pilot doesn't name one — a brisk GA taxi.
const DEFAULT_KTS: f64 = 10.0;
/// Facing more than this off the route's direction of travel at engage is a
/// refusal: pure pursuit from a backwards start is a pirouette, not a taxi.
const MAX_ENGAGE_ERR_DEG: f64 = 100.0;

pub struct AutoTaxi {
    /// The airframe's park-brake valve SET sense, passed through to the
    /// inner `hold_speed` arm so its release gates work unchanged.
    pub park_brake_valve_set: Option<f64>,
}

#[derive(Deserialize, Default)]
struct Args {
    /// Taxi pace, knots. Absent = 10.
    #[serde(default)]
    knots: Option<f64>,
    /// true = stop auto-taxiing and hand everything back.
    #[serde(default)]
    off: bool,
}

#[async_trait]
impl Tool for AutoTaxi {
    fn name(&self) -> &str {
        "auto_taxi"
    }

    fn description(&self) -> &str {
        "The copilot taxis the aircraft along the CLEARED taxi route to the \
         holding point: steering, throttle and brakes, stopping short of the \
         hold line. Use ONLY when the pilot asks the copilot to do the taxiing \
         or steering itself: 'taxi for me', 'you taxi', 'you have the taxi', \
         'take us to the holding point', 'auto taxi'. Optional knots sets the \
         pace. off=true for 'stop auto taxi' / 'cancel auto taxi'. (A plain \
         'taxi at 10 knots' where the pilot steers is hold_speed, not this.)"
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "knots": {"type": "number", "description": "taxi pace, knots; absent = 10"},
                "off": {"type": "boolean", "description": "true = stop auto-taxiing, hand everything back"}
            }
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        use std::sync::atomic::Ordering::Relaxed;
        let a: Args = serde_json::from_value(args.clone()).unwrap_or_default();
        if a.off {
            let was_driving = autotaxi::ACTIVE.swap(false, Relaxed);
            let had_speed = speedhold::ACTIVE.swap(false, Relaxed);
            return Ok(if was_driving || had_speed {
                "Auto taxi off. You have the aircraft.".to_string()
            } else {
                "I wasn't taxiing us.".to_string()
            });
        }
        // Already driving: a pace request retargets, anything else reassures.
        if autotaxi::ACTIVE.load(Relaxed) {
            return Ok(if let Some(k) = a.knots {
                let k = speedhold::clamp_taxi_target(k);
                speedhold::set_taxi_target(k);
                format!("Making it {k:.0} knots.")
            } else {
                "I already have the taxi. We're on our way to the holding point.".to_string()
            });
        }
        // A replay can't be taxied: the position is a recording, and the
        // governor + steering would spring on the pilot at live.
        if ctx
            .xplane
            .get_float(crate::datarefs::IS_IN_REPLAY)
            .await
            .is_ok_and(|v| v > 0.5)
        {
            return Ok("We're in replay. Go back to live and ask again.".to_string());
        }
        if ctx.xplane.get_float(crate::datarefs::ON_GROUND_ANY).await.unwrap_or(0.0) < 0.5 {
            return Ok("We're airborne. No taxiing up here.".to_string());
        }
        // The route is ATC's: no clearance, no guidance, no driving.
        let Some(watch) = party_line::with_taxi_route(|r| r.cloned()) else {
            return Ok(
                "We don't have a taxi clearance yet. Call the tower and request taxi, then I'll take it."
                    .to_string(),
            );
        };
        let Some(route) = autotaxi::Route::build(&watch) else {
            return Ok("The cleared route's too short to drive. We're basically there.".to_string());
        };
        let (Ok(lat), Ok(lon), Ok(psi)) = (
            ctx.xplane.get_float(autotaxi::LAT).await,
            ctx.xplane.get_float(autotaxi::LON).await,
            ctx.xplane.get_float(autotaxi::PSI_TRUE).await,
        ) else {
            return Ok("I can't read our position right now.".to_string());
        };
        let Some((progress, lateral)) = route.fix(lat, lon) else {
            return Ok("I can't make out the taxi route from here.".to_string());
        };
        if lateral > autotaxi::OFF_ROUTE_STOP_M {
            return Ok(format!(
                "We're about {} meters off the cleared route. Get us onto it and ask again.",
                (lateral / 10.0).round() as i64 * 10
            ));
        }
        if route.remaining(progress) <= 1.0 {
            return Ok("We're already at the holding point.".to_string());
        }
        if route.heading_err_deg(lat, lon, psi, progress, 20.0).abs() > MAX_ENGAGE_ERR_DEG {
            return Ok(
                "We're facing against the route. Turn us around first, then I'll take it."
                    .to_string(),
            );
        }
        // No steering channel (plugin missing or old) — offer what still works.
        if !autotaxi::steer_channel_ok(ctx.xplane).await {
            return Ok(
                "I can't steer. The x-copilot plugin needs updating. I can hold the taxi speed while you steer."
                    .to_string(),
            );
        }
        let base = speedhold::clamp_taxi_target(a.knots.unwrap_or(DEFAULT_KTS)).max(3.0);
        // Arm the governor through the front door: every parking-brake and
        // held-brake gate in hold_speed applies to an auto taxi unchanged.
        let arm_line = speedhold::HoldSpeed {
            approach_kts: None,
            park_brake_valve_set: self.park_brake_valve_set,
        }
        .execute(&json!({ "knots": base }), ctx)
        .await?;
        if !speedhold::taxi_hold_active() {
            // The governor refused (parking brake stuck, etc.) — its answer
            // says why, and it's ours too.
            return Ok(arm_line);
        }
        autotaxi::start(
            ctx.config.xplane_base_url.clone(),
            ctx.voice.cloned(),
            route,
            base,
        );
        let hp = crate::station::spoken_taxiway(&watch.holding_point);
        let dest = if hp.is_empty() {
            "the holding point".to_string()
        } else {
            format!("holding point {hp}")
        };
        Ok(format!(
            "My taxi. {dest} via {}, {base:.0} knots. Your pedals or my controls take it back any time.",
            watch.via
        ))
    }
}
