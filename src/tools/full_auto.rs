//! `full_auto` — the copilot flies the WHOLE thing from cold: power up, start
//! the engine, request taxi, drive to the runway, line up, and fly the mission
//! — a circuit, or (flight=true) the pilot's actual SimBrief flight plan to
//! its destination.
//!
//! The arming end of [`crate::full_auto`]. It holds the profile params the
//! sub-tools need and hands a freshly-built set to the orchestrator thread.
//! The SimBrief mission fetches the OFP here (on demand, never in the realtime
//! loop), refuses honestly when there's no plan or we're not at its origin,
//! and best-effort loads the route into the GPS so the panel shows where we're
//! going. EXPERIMENTAL — nine subsystems in a chain; VMC, hand ready.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::aircraft::FuelSelectorProfile;
use crate::full_auto::{Mission, Route};
use crate::tools::autopilot::ApConfig;

pub struct FullAuto {
    // engine start
    pub fuel_selector: Option<FuelSelectorProfile>,
    pub alternator_on_cmds: Vec<String>,
    /// DC-bus power-on commands (battery + master) for airframes whose stock
    /// battery dataref is overwritten by a custom switch sync (the Twin Otter).
    pub bus_power_on_cmds: Vec<String>,
    /// The turboprop start sequence, when the airframe is a turboprop — full
    /// auto cranks a PT6 with this instead of the piston start (which never
    /// catches on a turbine).
    pub turboprop_start: Option<crate::aircraft::TurbineStartProfile>,
    /// EVERY electrical system's ON command — the full bring-up run after the
    /// engine catches (generators AND bleed air, which feed the Twin Otter's
    /// autopilot). Empty on airframes with no electrical profile.
    pub all_electrical_on_cmds: Vec<String>,
    // taxi / line-up
    pub park_brake_valve_set: Option<f64>,
    pub takeoff_roll_m: Option<f64>,
    /// Constant-speed prop levers, threaded into every ground-roll sub-tool so
    /// they unfeather before taxi/line-up/takeoff. `None` on the pistons.
    pub prop: Option<crate::aircraft::PropProfile>,
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
    /// true = fly the pilot's SimBrief flight plan instead of a circuit.
    #[serde(default)]
    flight: bool,
}

#[async_trait]
impl Tool for FullAuto {
    fn name(&self) -> &str {
        "full_auto"
    }

    fn description(&self) -> &str {
        "Fly the ENTIRE flight autonomously from cold: power up, start the \
         engine, request taxi from the tower, taxi to the runway, line up, take \
         off, and fly the mission. Use for 'full auto', 'do everything', \
         'start up and fly a circuit', 'take us up from cold'. flight=true \
         flies the pilot's SimBrief FLIGHT PLAN to its destination instead of \
         a circuit — 'fly my flight plan', 'fly the simbrief flight', 'fly us \
         to the destination'. off=true to cancel. (For just the airborne \
         circuit from the runway, use circuit.)"
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "off": {"type": "boolean", "description": "true = cancel full auto"},
                "right": {"type": "boolean", "description": "true = right-hand pattern (circuit only)"},
                "flight": {"type": "boolean", "description": "true = fly the SimBrief flight plan to its destination"}
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

        let (mission, ack) = if a.flight {
            match self.build_flight_mission(ctx).await {
                Ok(v) => v,
                Err(why) => return Ok(why),
            }
        } else {
            (
                Mission::Circuit { right: a.right },
                "Full auto. I'll power up, start the engine, get our taxi, line us up, \
                 and fly the circuit. Say cancel or my controls any time."
                    .to_string(),
            )
        };

        let sub = crate::full_auto::SubTools {
            // A turboprop cranks with the PT6 start; everything else with the
            // piston start. Both run behind full auto's own bus-power step.
            start_engine: match &self.turboprop_start {
                Some(ts) => Box::new(crate::tools::cycle::StartTurbines::new(ts.clone())),
                None => Box::new(crate::tools::engine::StartEngine {
                    fuel_selector: self.fuel_selector.clone(),
                    alternator_on_cmds: self.alternator_on_cmds.clone(),
                    bus_power_on_cmds: self.bus_power_on_cmds.clone(),
                }),
            },
            bus_power_on_cmds: self.bus_power_on_cmds.clone(),
            all_electrical_on_cmds: self.all_electrical_on_cmds.clone(),
            taxi: crate::tools::taxi::AutoTaxi {
                park_brake_valve_set: self.park_brake_valve_set,
                prop: self.prop.clone(),
            },
            line_up: crate::tools::taxi::LineUp {
                park_brake_valve_set: self.park_brake_valve_set,
                takeoff_roll_m: self.takeoff_roll_m,
                prop: self.prop.clone(),
            },
            circuit: crate::tools::circuit::Circuit {
                cfg: self.cfg.clone(),
                vr_kts: self.vr_kts,
                vy_kts: self.vy_kts,
                climb_fpm: self.climb_fpm,
                retract_gear: self.retract_gear,
                takeoff_roll_m: self.takeoff_roll_m,
                approach_ias: self.approach_ias,
                prop: self.prop.clone(),
            },
            takeoff: crate::tools::takeoff::AutoTakeoff {
                cfg: self.cfg.clone(),
                vr_kts: self.vr_kts,
                vy_kts: self.vy_kts,
                climb_fpm: self.climb_fpm,
                retract_gear: self.retract_gear,
                takeoff_roll_m: self.takeoff_roll_m,
                prop: self.prop.clone(),
            },
            mission,
            power_up_first: self.power_up_first,
        };
        crate::full_auto::start(
            ctx.config.xplane_base_url.clone(),
            ctx.config.clone(),
            ctx.voice.cloned(),
            sub,
        );
        Ok(ack)
    }
}

impl FullAuto {
    /// Fetch the SimBrief OFP and distil it to a flyable [`Mission::Flight`] —
    /// or an honest spoken refusal (no plan, no waypoints, or a plan that
    /// departs somewhere we aren't).
    async fn build_flight_mission(&self, ctx: &ToolCtx<'_>) -> std::result::Result<(Mission, String), String> {
        if ctx.config.simbrief_id.trim().is_empty() {
            return Err(
                "No SimBrief ID set. Add XCOPILOT_SIMBRIEF_ID and I can fly your plan.".to_string()
            );
        }
        let fp = crate::simbrief::fetch(&ctx.config.simbrief_id).await.map_err(|e| {
            tracing::warn!("simbrief fetch failed: {e:#}");
            "Couldn't fetch your SimBrief plan. Generate one first, or check the ID.".to_string()
        })?;

        // The plan must depart from where we're parked — flying LEJR's route
        // out of some other field would thread the wrong corridor entirely.
        let (Ok(lat), Ok(lon)) = (
            ctx.xplane.get_float("sim/flightmodel/position/latitude").await,
            ctx.xplane.get_float("sim/flightmodel/position/longitude").await,
        ) else {
            return Err("I can't read our position, so I can't line the plan up with where we are.".to_string());
        };
        let apt_dat = super::autoland::apt_dat_path();
        let here = crate::autoland::nearby_runways(&apt_dat, lat, lon, 4.0)
            .unwrap_or_default()
            .into_iter()
            .any(|r| r.icao.eq_ignore_ascii_case(&fp.origin));
        if !here {
            return Err(format!(
                "Your plan departs {} but we're not there. Generate a plan from this field, \
                 or say full auto for a circuit here.",
                fp.origin
            ));
        }

        let legs = crate::procedure::enroute_legs(&fp.waypoints, &fp.origin, &fp.destination, fp.cruise_alt_ft);
        if legs.is_empty() {
            return Err(format!(
                "Your plan to {} has no enroute waypoints I can fly. Generate it with a route.",
                fp.destination
            ));
        }

        // The spoken destination: the airport's name when apt.dat knows it.
        let dest_name = {
            let (path, icao) = (ctx.config.apt_data_path.clone(), fp.destination.clone());
            tokio::task::spawn_blocking(move || crate::nav::find_airport(&path, &icao))
                .await
                .ok()
                .and_then(|r| r.ok())
                .flatten()
                .map(|a| a.name)
                .unwrap_or_else(|| fp.destination.clone())
        };

        // The GPS route payload — written by the orchestrator (only the
        // concrete client can set a byte-array dataref).
        let fms_payload = fp
            .waypoints
            .iter()
            .map(|w| format!("{},{:.4},{:.4},{}", w.ident, w.lat, w.lon, w.alt as i64))
            .collect::<Vec<_>>()
            .join("|");

        let n = legs.len();
        let ack = format!(
            "Full auto, {} to {}. I'll start up, get our taxi, take off, and fly the \
             {} enroute {} at {:.0} feet, then the approach into {}. Say cancel or my \
             controls any time.",
            fp.origin,
            fp.destination,
            n,
            if n == 1 { "fix" } else { "fixes" },
            fp.cruise_alt_ft,
            dest_name,
        );
        Ok((
            Mission::Flight(Route { dest_icao: fp.destination.clone(), dest_name, legs, fms_payload }),
            ack,
        ))
    }
}
