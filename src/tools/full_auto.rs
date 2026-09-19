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

/// Which engine start a full-auto sequence runs — one per airframe CLASS, and
/// they are not interchangeable: the piston start cranks a starter that isn't
/// there on a turbine and then reports "check fuel and mixture" about levers
/// the aircraft hasn't got.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StartKind {
    Piston,
    Turboprop,
    Jet,
    /// The aircraft's own scripted start behind the stock `auto_start`
    /// command (`AircraftProfile::scripted_auto_start`): one press, the
    /// aircraft's real switches, its own timing. The stock 737-800.
    Scripted,
    /// A turbine airframe that declares NO start sequence — the Citation X,
    /// whose stock systems the MD-82's APU/bleed/cross-tie sequence does not
    /// fit. There is nothing to crank, so full auto says so instead of
    /// falling through to the piston start and cranking a starter that isn't
    /// there (which is exactly what it did to the MD-82 until 2026-07-27).
    None,
}

/// The class from the profile's start declarations. The PT6 sequence wins where
/// an airframe declares both, because a turboprop's start is the specific one;
/// `turbine_start` alone means a jet with an APU and bleed-air starters. The
/// absence of both is a piston — but only if the airframe HAS pistons.
///
/// The jet arm was MISSING until 2026-07-27: `turbine_start` airframes fell
/// through to the piston start, so full auto on an MD-82 announced "spinning
/// up the engine" and then "cranking, the engine hasn't caught" at a cold jet
/// that was never going to catch. `piston` closes the other half of that hole:
/// a turbine with no sequence declared is not a piston, it is an airframe we
/// cannot start.
pub fn start_kind(turboprop: bool, jet: bool, piston: bool, scripted: bool) -> StartKind {
    match (scripted, turboprop, jet, piston) {
        (true, _, _, _) => StartKind::Scripted,
        (false, true, _, _) => StartKind::Turboprop,
        (false, false, true, _) => StartKind::Jet,
        (false, false, false, true) => StartKind::Piston,
        (false, false, false, false) => StartKind::None,
    }
}

impl StartKind {
    /// How long full auto waits for the engine to be RUNNING after it fires the
    /// start — a per-class figure, because the three starts are not the same
    /// order of duration. A piston catches in seconds. The jet sequence is a
    /// procedure: bleed pressure, then crank/fuel/idle on the right engine, then
    /// the same again on the left (which is the one full auto watches), and its
    /// own internal waits alone bound it near three minutes. The single
    /// piston-shaped 40 s deadline gave up on an MD-82 forty seconds in, while
    /// the start sequence was still waiting on bleed pressure and went on to
    /// crank both engines for a full-auto run that had already called "your
    /// aircraft" (Luton, 2026-07-27).
    pub fn engine_start_secs(self) -> f64 {
        match self {
            // The 737's script took 124 s from cold to both running; its
            // fire tests and APU spin-up are inside that.
            StartKind::Jet | StartKind::Scripted => 300.0,
            StartKind::Turboprop | StartKind::Piston | StartKind::None => 40.0,
        }
    }
}

pub struct FullAuto {
    // engine start
    /// Engine count, so a piston TWIN gets both starters cranked. The piston
    /// start is sequential and per-engine; a chain that cranked only number one
    /// would taxi out on one engine.
    pub engines: usize,
    /// Cycle (up/down) fuel selectors, so the piston start opens the valves
    /// before cranking — see `StartEngine::cycle_selectors`.
    pub cycle_selectors: Vec<crate::aircraft::CycleSelectorProfile>,
    pub fuel_selector: Option<FuelSelectorProfile>,
    pub alternator_on_cmds: Vec<String>,
    /// DC-bus power-on commands (battery + master) for airframes whose stock
    /// battery dataref is overwritten by a custom switch sync (the Twin Otter).
    pub bus_power_on_cmds: Vec<String>,
    /// The turboprop start sequence, when the airframe is a turboprop — full
    /// auto cranks a PT6 with this instead of the piston start (which never
    /// catches on a turbine).
    pub turboprop_start: Option<crate::aircraft::TurbineStartProfile>,
    /// Jet with an APU (profile `turbine_start`): full auto cranks it with the
    /// `start_engines` sequence — bleed air, N2, fuel at 20% — not the piston
    /// start. THREE airframe classes, three starts; the missing third had full
    /// auto piston-cranking an MD-82 ("cranking, the engine hasn't caught,
    /// check fuel and mixture") on an aircraft with neither, 2026-07-27.
    pub jet_start: bool,
    /// The aircraft scripts its own start behind `sim/operation/auto_start`.
    pub scripted_auto_start: bool,
    /// EVERY electrical system's ON command — the full bring-up run after the
    /// engine catches (generators AND bleed air, which feed the Twin Otter's
    /// autopilot). Empty on airframes with no electrical profile.
    pub all_electrical_on_cmds: Vec<String>,
    /// Toggle-only electrical switches (see `tools::all_electrical_toggles`).
    pub all_electrical_toggles: Vec<crate::aircraft::ToggleSwitch>,
    // taxi / line-up
    pub park_brake_valve_set: Option<f64>,
    pub takeoff_roll_m: Option<f64>,
    /// Book figures for the arm-time preflight of a Flight mission
    /// (`full_auto::preflight`): the runway at each end and the all-up weight
    /// are checked at the gate, not found at the hold or 25 nm out.
    pub landing_distance_ft: Option<f64>,
    pub takeoff_distance_ft: Option<f64>,
    pub max_gross_lb: Option<f64>,
    /// Flap-handle ratio set for takeoff and retracted climbing out (profile's
    /// `flaps.takeoff_ratio`); `None` leaves the flaps alone.
    pub takeoff_flap_ratio: Option<f64>,
    /// Indicated cruise speed for the enroute cruise-power hold (profile's
    /// `reference.cruise_ias_kts`); `None` leaves cruise at climb power.
    pub cruise_ias_kts: Option<f64>,
    /// The airframe's transition altitude (QNH below, standard above).
    pub transition_altitude_ft: f64,
    /// Constant-speed prop levers, threaded into every ground-roll sub-tool so
    /// they unfeather before taxi/line-up/takeoff. `None` on the pistons.
    pub prop: Option<crate::aircraft::PropProfile>,
    /// This airframe's taxi pacing, threaded into the ground-roll sub-tools it
    /// builds — the chain must taxi a jet like a jet, same as a spoken ask.
    pub pace: crate::autotaxi::Pace,
    /// Its tail-strike attitude, threaded the same way.
    pub tail_strike_deg: Option<f64>,
    /// Vne/Vmo for the climb's overspeed protection.
    pub vne_kts: Option<f64>,
    /// Slowest the pattern may climb toward — see `circuit::climb_limit_fpm`.
    pub climb_floor_kts: f64,
    /// Clean stall speed, the floor under the pattern speed (flown clean).
    pub vs_clean_kts: Option<f64>,
    /// The airframe's exterior lights, for the departure light schedule.
    pub lights: Vec<crate::aircraft::LightProfile>,
    /// Panel flood-lighting dataref override, when the airframe has its own.
    pub flood_dataref: Option<String>,
    /// ...or a toggle command, when the switch is one (the Baron 58). Carried
    /// alongside the dataref because full auto builds its OWN copy of the
    /// flood tool: an override wired here and not there is a tool that behaves
    /// one way for the pilot and another for the automation.
    pub flood_toggle: Option<crate::aircraft::ToggleSwitch>,
    // circuit / climb-out
    pub cfg: ApConfig,
    pub vr_kts: f64,
    pub vy_kts: f64,
    /// Initial climb speed (see `autotakeoff::Params::initial_climb_kts`).
    pub initial_climb_kts: f64,
    pub climb_fpm: f64,
    pub retract_gear: bool,
    pub approach_ias: f64,
    /// Turbine airframe → run the APU/electrical power-up before engine start.
    pub power_up_first: bool,
    /// The airframe has SOME piston start we can fly — the generic `engine`
    /// group, or a custom cycle-switch sequence (`piston_start`, the DC-3).
    /// Without it and without a turbine sequence there is no start at all —
    /// see [`StartKind::None`].
    pub piston_engine: bool,
    /// The generic continuous mixture lever (`engine` group) specifically,
    /// not any piston: the DC-3 is a piston with a notched xlua lever the
    /// manager must not creep. Arms the enroute mixture manager and the
    /// arrival's full rich.
    pub piston_mixture: bool,
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
    #[serde(default)]
    resume: bool,
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
                "flight": {"type": "boolean", "description": "true = fly the SimBrief flight plan to its destination"},
                "resume": {"type": "boolean", "description": "resume the loaded flight plan from a checked airborne rejoin"}
            }
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: Args = serde_json::from_value(args.clone()).unwrap_or_default();
        if a.off {
            return Ok(if crate::full_auto::abort() {
                "Full auto cancelled. You have the aircraft.".to_string()
            } else {
                "Full auto isn't running.".to_string()
            });
        }
        if crate::full_auto::running() {
            return Ok("I'm already running the full sequence. Say cancel to stop.".to_string());
        }
        if ctx
            .xplane
            .get_float(crate::datarefs::IS_IN_REPLAY)
            .await
            .is_ok_and(|v| v > 0.5)
        {
            return Ok("We're in replay. Go back to live and ask again.".to_string());
        }

        let (mission, ack) = if a.resume {
            match self.build_resume_mission(ctx).await {
                Ok(v) => v,
                Err(why) => return Ok(why),
            }
        } else if a.flight {
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

        let kind = start_kind(
            self.turboprop_start.is_some(),
            self.jet_start,
            self.piston_engine,
            self.scripted_auto_start,
        );
        let sub = crate::full_auto::SubTools {
            lights: crate::tools::lights::SetLights::new(self.lights.clone()),
            flood: crate::tools::lights::SetFloodLights::new(
                self.flood_dataref.clone(),
                self.flood_toggle.clone(),
            ),
            no_start_sequence: kind == StartKind::None,
            // One start per airframe CLASS: a turboprop cranks with the PT6
            // sequence, a jet with the APU/bleed-air sequence, a piston with
            // the one-shot start. All run behind full auto's own bus-power
            // step. Getting this wrong is silent and total — the piston start
            // on a turbine cranks something that isn't there and reports
            // "check fuel and mixture" about levers the aircraft hasn't got.
            start_engine: match (kind, &self.turboprop_start) {
                (StartKind::Scripted, _) => Box::new(crate::tools::jet_start::ScriptedAutoStart),
                (StartKind::Turboprop, Some(ts)) => {
                    Box::new(crate::tools::cycle::StartTurbines::new(ts.clone()))
                }
                (StartKind::Jet, _) => Box::new(crate::tools::jet_start::StartEngines),
                _ => Box::new(crate::tools::engine::StartEngine {
                    engines: self.engines,
                    cycle_selectors: self.cycle_selectors.clone(),
                    fuel_selector: self.fuel_selector.clone(),
                    alternator_on_cmds: self.alternator_on_cmds.clone(),
                    bus_power_on_cmds: self.bus_power_on_cmds.clone(),
                }),
            },
            bus_power_on_cmds: self.bus_power_on_cmds.clone(),
            all_electrical_on_cmds: self.all_electrical_on_cmds.clone(),
            all_electrical_toggles: self.all_electrical_toggles.clone(),
            taxi: crate::tools::taxi::AutoTaxi {
                park_brake_valve_set: self.park_brake_valve_set,
                prop: self.prop.clone(),
                pace: self.pace,
            },
            line_up: crate::tools::taxi::LineUp {
                park_brake_valve_set: self.park_brake_valve_set,
                takeoff_roll_m: self.takeoff_roll_m,
                prop: self.prop.clone(),
                pace: self.pace,
            },
            circuit: crate::tools::circuit::Circuit {
                park_brake_valve_set: self.park_brake_valve_set,
                tail_strike_deg: self.tail_strike_deg,
                vne_kts: self.vne_kts,
                cfg: self.cfg.clone(),
                vr_kts: self.vr_kts,
                vy_kts: self.vy_kts,
                initial_climb_kts: self.initial_climb_kts,
                climb_fpm: self.climb_fpm,
                retract_gear: self.retract_gear,
                takeoff_flap_ratio: self.takeoff_flap_ratio,
                takeoff_roll_m: self.takeoff_roll_m,
                approach_ias: self.approach_ias,
                prop: self.prop.clone(),
                climb_floor_kts: self.climb_floor_kts,
                vs_clean_kts: self.vs_clean_kts,
            },
            takeoff: crate::tools::takeoff::AutoTakeoff {
                // The Flight mission arms the takeoff through this copy, and
                // the orchestrator takes the climb from the handoff — the
                // pilot must not be told "your aircraft" there.
                handoff_to_caller: true,
                park_brake_valve_set: self.park_brake_valve_set,
                tail_strike_deg: self.tail_strike_deg,
                vne_kts: self.vne_kts,
                cfg: self.cfg.clone(),
                vr_kts: self.vr_kts,
                vy_kts: self.vy_kts,
                initial_climb_kts: self.initial_climb_kts,
                climb_fpm: self.climb_fpm,
                retract_gear: self.retract_gear,
                takeoff_flap_ratio: self.takeoff_flap_ratio,
                takeoff_roll_m: self.takeoff_roll_m,
                prop: self.prop.clone(),
            },
            cruise_ias_kts: self.cruise_ias_kts,
            transition_altitude_ft: self.transition_altitude_ft,
            piston_mixture: self.piston_mixture,
            mission,
            power_up_first: self.power_up_first,
            engine_start_secs: kind.engine_start_secs(),
        };
        if !crate::full_auto::start(
            ctx.config.xplane_base_url.clone(),
            ctx.config.clone(),
            ctx.voice.cloned(),
            sub,
        ) {
            return Ok(
                "The previous automation is still stopping. Ask again once it has stopped."
                    .to_string(),
            );
        }
        Ok(ack)
    }
}

impl FullAuto {
    /// An explicit resume uses only the plan on the shelf. It never refetches,
    /// starts engines or walks the ground stages against an airborne aircraft.
    /// Every refusal is blackboxed with its reason (five in a row over the
    /// Gulf of Lion, 2026-09-16, and the debrief could not say why).
    async fn build_resume_mission(&self, ctx: &ToolCtx<'_>) -> std::result::Result<(Mission, String), String> {
        let built = self.resume_mission_checked(ctx).await;
        match &built {
            Ok((_, ack)) => crate::full_auto::event("resume", json!({ "ack": ack })),
            Err(why) => crate::full_auto::event("resume_refused", json!({ "why": why })),
        }
        built
    }

    async fn resume_mission_checked(&self, ctx: &ToolCtx<'_>) -> std::result::Result<(Mission, String), String> {
        let fp = crate::simbrief::loaded_plan().ok_or_else(||
            "No plan is loaded. I'm monitoring only; you have control. Load and review the flight plan first.".to_string())?;
        if fp.source == crate::simbrief::PlanSource::Local {
            return Err("This local route needs a fresh terrain-checked plan after interruption. I haven't taken control. Request an approach or land for me.".into());
        }
        let xp = crate::xplane::XPlaneClient::new(ctx.config.xplane_base_url.clone());
        let (ground, agl, lat, lon, altitude, bank, ias, frozen, engines, aircraft, mag_var) = tokio::join!(
            ctx.xplane.get_float(crate::datarefs::ON_GROUND_ANY),
            ctx.xplane.get_float(crate::datarefs::AGL_METERS),
            ctx.xplane.get_float(crate::datarefs::LATITUDE),
            ctx.xplane.get_float(crate::datarefs::LONGITUDE),
            ctx.xplane.get_float(crate::datarefs::ALTITUDE_FT),
            ctx.xplane.get_float(crate::datarefs::BANK_DEG),
            ctx.xplane.get_float(crate::datarefs::AIRSPEED_KTS),
            async {
                let (pause,replay) = tokio::join!(ctx.xplane.get_float(crate::datarefs::PAUSED), ctx.xplane.get_float(crate::datarefs::IS_IN_REPLAY));
                !matches!((pause,replay), (Ok(p),Ok(r)) if p.is_finite() && r.is_finite() && p < 0.5 && r < 0.5)
            },
            ctx.xplane.get_float_array("sim/flightmodel/engine/ENGN_running"),
            xp.get_string("sim/aircraft/view/acf_ICAO"),
            ctx.xplane.get_float("sim/flightmodel/position/magnetic_variation"),
        );
        let Some([ground, agl, lat, lon, altitude, bank, ias]) = crate::control::measurements(
            "resume", [ground, agl, lat, lon, altitude, bank, ias]) else {
            return Err("I can't verify the aircraft state. I haven't taken control; try again once the sim is connected.".into());
        };
        // Each gate names itself: "stable flight" told a pilot in a 28 degree
        // orbit nothing about what to change (2026-09-16).
        let speed_floor = crate::aircraft::floored_target(self.approach_ias, crate::aircraft::speed_floor_kts());
        let agl_ft = agl * 3.28084;
        let gate = if frozen {
            Some("We're paused or in replay. Go back to live and ask again; I haven't taken control.".to_string())
        } else if ground > 0.5 {
            Some("We're on the ground. Resume is for the air; say fly the flight plan for the whole mission.".to_string())
        } else if agl_ft < 1000.0 {
            Some(format!("Resume needs a thousand feet; we're {agl_ft:.0} feet above the ground. I haven't taken control."))
        } else if bank.abs() > 20.0 {
            Some(format!("Wings level first; we're in {:.0} degrees of bank. Then ask again. I haven't taken control.", bank.abs()))
        } else if ias < speed_floor {
            Some(format!("We're slow at {ias:.0} knots; resume needs {speed_floor:.0}. I haven't taken control."))
        } else {
            None
        };
        if let Some(why) = gate {
            return Err(why);
        }
        if !engines.ok().is_some_and(|v| v.len() >= self.engines && v.iter().take(self.engines).all(|v| v.is_finite() && *v > 0.5)) {
            return Err("I can't confirm all engines running. I haven't taken control; stabilise the aircraft first.".into());
        }
        if !aircraft.ok().is_some_and(|a| a.trim().eq_ignore_ascii_case(&fp.aircraft)) {
            return Err("The loaded plan's aircraft doesn't match this aircraft. Review a matching plan first; you have control.".into());
        }
        let mut legs = crate::procedure::enroute_legs(&fp.waypoints, &fp.origin, &fp.destination, fp.cruise_alt_ft);
        let index = crate::interaction::rejoin_index(&legs, lat, lon, mag_var.ok().filter(|v| v.is_finite()))?;
        legs.drain(..index);
        let first = legs.first().unwrap();
        let fix = first.fix.as_ref().unwrap();
        let distance = crate::nav::haversine_nm(lat, lon, fix.lat, fix.lon);
        if distance > 30.0 {
            return Err("The next fix is too far away to verify the rejoin terrain. I haven't taken control; follow the GPS or request an approach.".into());
        }
        let target = first.alt1_ft.unwrap_or(fp.cruise_alt_ft);
        if !target.is_finite() || target <= 0.0 || target > self.cfg.service_ceiling_ft {
            return Err("The resumed altitude is outside this aircraft's limits. Review the plan; you have control.".into());
        }
        // Use the LOWER of present/target true altitude over the complete join.
        let true_ft = xp.get_float("sim/flightmodel/position/elevation").await
            .map_err(|_| "True altitude unavailable; I haven't taken control.".to_string())? * 3.28084;
        let floor = altitude.min(target) + (true_ft-altitude) - 1000.0;
        if !floor.is_finite() { return Err("Altitude unavailable; I haven't taken control.".into()); }
        let count = ((distance*1852.0/150.0).ceil() as usize).max(1);
        let points: Vec<_> = (0..=count).map(|i| {
            let f=i as f64/count as f64;
            (lat+(fix.lat-lat)*f, lon+(fix.lon-lon)*f)
        }).collect();
        for chunk in points.chunks(crate::terrain::MAX_POINTS) {
            let terrain = crate::terrain::sample_ft(&xp, chunk).await.ok_or_else(||
                "The terrain probe isn't answering, so I can't check the rejoin. I haven't taken control. Check the plugin build on the Health screen.".to_string())?;
            if terrain.iter().any(|h| !h.is_finite() || *h > floor) {
                return Err("The rejoin lacks terrain clearance at the planned altitude. I haven't taken control.".into());
            }
        }
        let apt = ctx.config.apt_data_path.clone();
        let cifp = ctx.config.cifp_dir.clone();
        let fixes = ctx.config.fix_data_path.clone();
        let dest = fp.destination.clone();
        let arrival = tokio::task::spawn_blocking(move || crate::full_auto::brief_arrival(&apt,&cifp,&fixes,&dest))
            .await.map_err(|_| "Arrival verification failed; you have control.".to_string())??;
        // Probes can take time: never arm from a position that has moved away
        // from the verified join while the pilot was waiting.
        let (new_lat, new_lon, new_alt) = tokio::join!(
            xp.get_float(crate::datarefs::LATITUDE),
            xp.get_float(crate::datarefs::LONGITUDE),
            xp.get_float(crate::datarefs::ALTITUDE_FT),
        );
        let Some([new_lat, new_lon, new_alt]) = crate::control::measurements(
            "resume_recheck", [new_lat, new_lon, new_alt]) else {
            return Err("Connection changed during verification. I haven't taken control.".into());
        };
        if crate::nav::haversine_nm(lat, lon, new_lat, new_lon) > 1.0 || (new_alt-altitude).abs() > 200.0 {
            return Err("The aircraft moved beyond the verified rejoin. I haven't taken control; stabilise and try again.".into());
        }
        let ack = format!("Resuming toward {} at {:.0} feet, continuing to {} and the arrival. {} Say my controls to take over.", fix.ident,target,fp.destination,arrival);
        let payload=fp.waypoints.iter().map(|w| format!("{},{:.4},{:.4},{}",w.ident,w.lat,w.lon,w.alt as i64)).collect::<Vec<_>>().join("|");
        Ok((Mission::Resume(Route { dest_icao: fp.destination.clone(), dest_name: fp.destination,
            legs, fms_payload: payload, local_runway: None }),ack))
    }

    /// Fetch the SimBrief OFP and distil it to a flyable [`Mission::Flight`] —
    /// or an honest spoken refusal (no plan, no waypoints, or a plan that
    /// departs somewhere we aren't).
    async fn build_flight_mission(
        &self,
        ctx: &ToolCtx<'_>,
    ) -> std::result::Result<(Mission, String), String> {
        // Airborne, the mission cannot arm — and the refusal must say what
        // DOES fly up here. The origin check below used to answer first,
        // telling a pilot enroute at 8,000 ft "we're not at LXGB... say full
        // auto for a circuit here" (2026-08-02). A failed read falls
        // through: the sequence re-checks on its own thread.
        if ctx
            .xplane
            .get_float(crate::datarefs::ON_GROUND_ANY)
            .await
            .is_ok_and(|og| og < 0.5)
        {
            return Err(
                "We're already flying, so full auto can't start. Say navigate to follow \
                 the plan, fly the ILS for the arrival, or land for me."
                    .to_string(),
            );
        }
        let mut fp = if let Some(local) = crate::simbrief::local_plan() {
            local
        } else {
            if ctx.config.simbrief_id.trim().is_empty() {
                return Err("No flight plan. Say plan a flight to, and a destination, or configure SimBrief.".into());
            }
            crate::simbrief::fetch(&ctx.config.simbrief_id)
                .await
                .map_err(|e| {
                    tracing::warn!("simbrief fetch failed: {e:#}");
                    "Couldn't fetch your SimBrief plan. Generate one first, or check the ID."
                        .to_string()
                })?
        };

        // The plan must depart from where we're parked — flying LEJR's route
        // out of some other field would thread the wrong corridor entirely.
        let (Ok(lat), Ok(lon)) = (
            ctx.xplane
                .get_float("sim/flightmodel/position/latitude")
                .await,
            ctx.xplane
                .get_float("sim/flightmodel/position/longitude")
                .await,
        ) else {
            return Err(
                "I can't read our position, so I can't line the plan up with where we are."
                    .to_string(),
            );
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

        // Both fields from one blocking pass: the destination's name for the
        // ack, and each end's longest runway for the preflight.
        let (origin_apt, dest_apt) = {
            let (path, o, d) = (
                ctx.config.apt_data_path.clone(),
                fp.origin.clone(),
                fp.destination.clone(),
            );
            tokio::task::spawn_blocking(move || {
                (
                    crate::nav::find_airport(&path, &o).ok().flatten(),
                    crate::nav::find_airport(&path, &d).ok().flatten(),
                )
            })
            .await
            .unwrap_or((None, None))
        };
        let local_runway = if fp.source == crate::simbrief::PlanSource::Local {
            let origin = origin_apt
                .as_ref()
                .ok_or("The origin airport is missing from the data.")?;
            let dest = dest_apt
                .as_ref()
                .ok_or("The destination airport is missing from the data.")?;
            let speed = self
                .cruise_ias_kts
                .ok_or("The airframe needs a cruise speed to plan this local flight.")?;
            let route = crate::flightplan::local_route(
                &fp,
                origin,
                dest,
                speed,
                self.climb_fpm,
                crate::flightplan::runway_needed_ft(
                    self.landing_distance_ft,
                    self.takeoff_distance_ft,
                )
                .unwrap_or(0.0),
                crate::weather::surface_wind(&super::autoland::xplane_root(), &fp.destination),
            )?;
            let xp = crate::xplane::XPlaneClient::new(ctx.config.xplane_base_url.clone());
            let mut heights = Vec::new();
            for chunk in route.terrain.chunks(crate::terrain::MAX_POINTS) {
                let pts: Vec<_> = chunk.iter().map(|(p, _)| *p).collect();
                heights.extend(crate::terrain::sample_ft(&xp, &pts).await
                    .ok_or("I can't verify terrain along the local route. The updated plugin and loaded scenery are required.")?);
            }
            if !crate::flightplan::local_terrain_clear(&route, &heights) {
                return Err("Terrain blocks the local climb or arrival corridor. Choose another destination or route.".into());
            }
            // Price the authored track, including the final alignment, into
            // the same preflight fuel check. Reserve stays untouched.
            if fp.distance_nm > 0.0 {
                let extra = fp.trip_fuel_gal * (route.distance_nm / fp.distance_nm - 1.0).max(0.0);
                fp.trip_fuel_gal += extra;
                fp.block_fuel_gal += extra;
                fp.block_fuel_kg += extra * 2.72;
                fp.min_takeoff_fuel_kg += extra * 2.72;
                fp.ete_min *= route.distance_nm / fp.distance_nm;
            }
            fp.distance_nm = route.distance_nm;
            fp.waypoints = route.waypoints;
            Some(route.runway)
        } else {
            None
        };

        let legs = crate::procedure::enroute_legs(
            &fp.waypoints,
            &fp.origin,
            &fp.destination,
            fp.cruise_alt_ft,
        );
        if legs.is_empty() {
            return Err(format!(
                "Your plan to {} has no enroute waypoints I can fly. Generate it with a route.",
                fp.destination
            ));
        }

        let dest_name = dest_apt
            .as_ref()
            .map(|a| a.name.trim().to_string())
            .filter(|n| !n.is_empty())
            .unwrap_or_else(|| fp.destination.clone());

        // The preflight: tanks, load and all-up weight as the sim has them
        // now, against the plan and the book. A refusal here costs nothing;
        // the same shortfall found airborne is a diversion.
        let (fuel, payload, total) = tokio::join!(
            ctx.xplane.get_float(crate::datarefs::FUEL_TOTAL_KG),
            ctx.xplane.get_float(crate::datarefs::PAYLOAD_KG),
            ctx.xplane.get_float(crate::datarefs::TOTAL_WEIGHT_KG),
        );
        let facts = crate::full_auto::PreflightFacts {
            fuel_kg: fuel.ok(),
            payload_kg: payload.ok(),
            total_kg: total.ok(),
            max_gross_lb: self.max_gross_lb,
            takeoff_roll_m: self.takeoff_roll_m,
            landing_distance_ft: self.landing_distance_ft,
            takeoff_distance_ft: self.takeoff_distance_ft,
            origin_longest_ft: origin_apt
                .as_ref()
                .map_or(0.0, crate::flightplan::longest_runway_ft),
            dest_longest_ft: dest_apt
                .as_ref()
                .map_or(0.0, crate::flightplan::longest_runway_ft),
        };
        let preflight = crate::full_auto::preflight(&fp, &facts);
        crate::full_auto::event(
            "preflight",
            json!({
                "origin": fp.origin, "destination": fp.destination,
                "fuel_kg": facts.fuel_kg, "block_kg": fp.block_fuel_kg,
                "min_takeoff_kg": fp.min_takeoff_fuel_kg,
                "payload_kg": facts.payload_kg,
                "plan_payload_kg": fp.weights.as_ref().map(|w| w.payload_kg),
                "pax": fp.weights.as_ref().map(|w| w.pax),
                "cargo_kg": fp.weights.as_ref().map(|w| w.cargo_kg),
                "total_kg": facts.total_kg,
                "max_tow_kg": fp.weights.as_ref().map(|w| w.max_tow_kg),
                "origin_longest_ft": facts.origin_longest_ft,
                "dest_longest_ft": facts.dest_longest_ft,
                "refused": preflight.as_ref().err(),
            }),
        );
        let preflight = preflight?;

        // The GPS route payload — written by the orchestrator (only the
        // concrete client can set a byte-array dataref).
        let fms_payload = crate::simbrief::fms_route_payload(&fp.waypoints);

        // The arrival, briefed NOW: what the destination offers and what
        // full auto will do there. Found at the gate, it is a briefing item;
        // found 25 nm out, it was a crash (Gibraltar, 2026-09-11). A
        // destination the data can't land at is refused here, not there.
        let arrival = if let Some(runway) = &local_runway {
            format!(
                "I'll fly the checked visual final on runway {}.",
                crate::circuit::spell_runway(runway)
            )
        } else {
            let (apt, cifp, fixp, dest) = (
                ctx.config.apt_data_path.clone(),
                ctx.config.cifp_dir.clone(),
                ctx.config.fix_data_path.clone(),
                fp.destination.clone(),
            );
            tokio::task::spawn_blocking(move || {
                crate::full_auto::brief_arrival(&apt, &cifp, &fixp, &dest)
            })
            .await
            .map_err(|e| format!("The arrival brief failed: {e}"))??
        };

        crate::simbrief::publish_plan(&fp);

        let n = legs.len();
        let preflight = if preflight.is_empty() {
            String::new()
        } else {
            format!("{preflight} ")
        };
        let ack = format!(
            "Full auto, {} to {}. {}I'll start up, get our taxi, take off, and fly the \
             {} enroute {} at {:.0} feet, then the approach into {}. {} Say cancel or my \
             controls any time.",
            fp.origin,
            fp.destination,
            preflight,
            n,
            if n == 1 { "fix" } else { "fixes" },
            fp.cruise_alt_ft,
            dest_name,
            arrival,
        );
        Ok((
            Mission::Flight(Route {
                dest_icao: fp.destination.clone(),
                dest_name,
                legs,
                fms_payload,
                local_runway,
            }),
            ack,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every shipped profile is exactly ONE airframe class, and full auto
    /// picks that class's start. The MD-82 is the case that found the gap:
    /// `turbine_start` with no `turboprop_start` fell through to the piston
    /// start, and full auto sat at a cold jet cranking a starter it doesn't
    /// have, telling the pilot to check a mixture lever it doesn't have
    /// either (Gatwick, 2026-07-27).
    #[test]
    fn full_auto_starts_each_airframe_the_way_its_engines_start() {
        for (id, want) in [
            ("cessna_172sp", StartKind::Piston),
            ("cirrus_sr22", StartKind::Piston),
            ("dc3", StartKind::Piston),
            ("lancair_evolution", StartKind::Turboprop),
            ("dhc6_twin_otter", StartKind::Turboprop),
            ("king_air_c90b", StartKind::Turboprop),
            ("md82", StartKind::Jet),
            // The Citation X is a turbofan, but the generic crank/fuel/idle
            // sequence fits it once the fuel goes in on N2 — so it starts
            // through the same path a PT6 does, not the MD-82's APU/bleed one.
            ("citation_x", StartKind::Turboprop),
            // The stock 737 scripts its own start behind `auto_start`.
            ("boeing_737_800", StartKind::Scripted),
        ] {
            let p = crate::aircraft::load(id).unwrap_or_else(|| panic!("{id} profile loads"));
            let got = start_kind(
                p.turboprop_start.is_some(),
                p.turbine_start,
                p.engine || p.piston_start.is_some(),
                p.scripted_auto_start,
            );
            assert_eq!(got, want, "{id}");
            // A jet has no piston engine group to fall back on, which is
            // exactly why the wrong start is silent and total.
            if got == StartKind::Jet {
                assert!(!p.engine, "{id}: a jet declares no piston engine group");
            }
        }
    }

    /// The wait for the engine has to cover the start it just fired. The jet
    /// sequence's own internal waits bound it near three minutes (bleed
    /// pressure, then crank/fuel/idle twice), so a deadline shorter than that
    /// gives up on a start that is still running normally — which is what a
    /// single 40 s figure did at Luton on 2026-07-27.
    #[test]
    fn the_engine_wait_outlasts_the_start_it_fired() {
        let jet = StartKind::Jet.engine_start_secs();
        assert!(jet >= 200.0, "jet start deadline too short: {jet}");
        assert!(jet > StartKind::Piston.engine_start_secs());
        assert!(StartKind::Piston.engine_start_secs() >= 30.0);
        assert!(StartKind::Turboprop.engine_start_secs() >= 30.0);
    }
}
