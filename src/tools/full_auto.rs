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
pub fn start_kind(turboprop: bool, jet: bool, piston: bool) -> StartKind {
    match (turboprop, jet, piston) {
        (true, _, _) => StartKind::Turboprop,
        (false, true, _) => StartKind::Jet,
        (false, false, true) => StartKind::Piston,
        (false, false, false) => StartKind::None,
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
            StartKind::Jet => 300.0,
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
    /// EVERY electrical system's ON command — the full bring-up run after the
    /// engine catches (generators AND bleed air, which feed the Twin Otter's
    /// autopilot). Empty on airframes with no electrical profile.
    pub all_electrical_on_cmds: Vec<String>,
    /// Toggle-only electrical switches (see `tools::all_electrical_toggles`).
    pub all_electrical_toggles: Vec<crate::aircraft::ToggleSwitch>,
    // taxi / line-up
    pub park_brake_valve_set: Option<f64>,
    pub takeoff_roll_m: Option<f64>,
    /// Flap-handle ratio set for takeoff and retracted climbing out (profile's
    /// `flaps.takeoff_ratio`); `None` leaves the flaps alone.
    pub takeoff_flap_ratio: Option<f64>,
    /// Indicated cruise speed for the enroute cruise-power hold (profile's
    /// `reference.cruise_ias_kts`); `None` leaves cruise at climb power.
    pub cruise_ias_kts: Option<f64>,
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
        if ctx
            .xplane
            .get_float(crate::datarefs::IS_IN_REPLAY)
            .await
            .is_ok_and(|v| v > 0.5)
        {
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

        let kind = start_kind(
            self.turboprop_start.is_some(),
            self.jet_start,
            self.piston_engine,
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
            start_engine: match (
                start_kind(
                    self.turboprop_start.is_some(),
                    self.jet_start,
                    self.piston_engine,
                ),
                &self.turboprop_start,
            ) {
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
                climb_fpm: self.climb_fpm,
                retract_gear: self.retract_gear,
                takeoff_flap_ratio: self.takeoff_flap_ratio,
                takeoff_roll_m: self.takeoff_roll_m,
                prop: self.prop.clone(),
            },
            cruise_ias_kts: self.cruise_ias_kts,
            mission,
            power_up_first: self.power_up_first,
            engine_start_secs: start_kind(
                self.turboprop_start.is_some(),
                self.jet_start,
                self.piston_engine,
            )
            .engine_start_secs(),
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
        if ctx.config.simbrief_id.trim().is_empty() {
            return Err(
                "No SimBrief ID set. Add XCOPILOT_SIMBRIEF_ID and I can fly your plan.".to_string(),
            );
        }
        let fp = crate::simbrief::fetch(&ctx.config.simbrief_id)
            .await
            .map_err(|e| {
                tracing::warn!("simbrief fetch failed: {e:#}");
                "Couldn't fetch your SimBrief plan. Generate one first, or check the ID."
                    .to_string()
            })?;

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

        // The arrival, briefed NOW: what the destination offers and what
        // full auto will do there. Found at the gate, it is a briefing item;
        // found 25 nm out, it was a crash (Gibraltar, 2026-09-11). A
        // destination the data can't land at is refused here, not there.
        let arrival = {
            let (apt, cifp, fixp, dest) = (
                ctx.config.apt_data_path.clone(),
                ctx.config.cifp_dir.clone(),
                ctx.config.fix_data_path.clone(),
                fp.destination.clone(),
            );
            tokio::task::spawn_blocking(move || crate::full_auto::brief_arrival(&apt, &cifp, &fixp, &dest))
                .await
                .map_err(|e| format!("The arrival brief failed: {e}"))??
        };

        let n = legs.len();
        let ack = format!(
            "Full auto, {} to {}. I'll start up, get our taxi, take off, and fly the \
             {} enroute {} at {:.0} feet, then the approach into {}. {} Say cancel or my \
             controls any time.",
            fp.origin,
            fp.destination,
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
        ] {
            let p = crate::aircraft::load(id).unwrap_or_else(|| panic!("{id} profile loads"));
            let got = start_kind(
                p.turboprop_start.is_some(),
                p.turbine_start,
                p.engine || p.piston_start.is_some(),
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
