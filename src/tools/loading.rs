//! Load the aircraft from the flight plan: the block fuel into the tanks and
//! the load sheet's payload into the cabin and holds.
//!
//! Until 2026-09-21 the plan's fuel and payload were only ever CHECKED (the
//! brief's "short about", `full_auto::preflight`'s refusals) and the pilot
//! set them by hand in the sim's Weight & Balance screen, one number at a
//! time from the OFP. The crew does the load sheet now, on the ground and
//! stopped, and reads the sim back afterwards: an add-on that manages its own
//! fuel rewrites the tanks and that is said, never "Roger".
//!
//! Fuel goes in the way a fueller puts it in: the wing tanks first, in
//! proportion to their size, and the centreline tanks only with what the
//! wings cannot hold (`distribute_fuel`). The 737 on 5.8 t has dry centre
//! tanks; a proportional fill would put 3.6 t of it in the centre.

use anyhow::Result;
use async_trait::async_trait;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::simbrief::FlightPlan;
use crate::xplane::XPlane;

/// Total fuel capacity, kg, and each tank's share of it.
const FUEL_CAPACITY_KG: &str = "sim/aircraft/weight/acf_m_fuel_tot";
const TANK_RATIO: &str = "sim/aircraft/overflow/acf_tank_rat";
/// Each tank's lateral position, m: zero is a centreline tank.
const TANK_LATERAL_M: &str = "sim/aircraft/overflow/acf_tank_X";
const EMPTY_WEIGHT_KG: &str = "sim/aircraft/weight/acf_m_empty";
const MAX_WEIGHT_KG: &str = "sim/aircraft/weight/acf_m_max";
pub(crate) const GROUNDSPEED_MS: &str = "sim/flightmodel/position/groundspeed";

/// A tank or a cabin within this of the plan is loaded: one percent, and
/// never tighter than two kilos.
const MATCH_FRAC: f64 = 0.01;
const MATCH_KG: f64 = 2.0;
/// A tank this far off the centreline is a wing tank.
pub(crate) const WING_TANK_M: f64 = 0.1;

fn matches(actual: f64, want: f64) -> bool {
    (actual - want).abs() <= MATCH_KG.max(want * MATCH_FRAC)
}

/// Which half of the load sheet was asked for.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum What {
    Fuel,
    Payload,
    Both,
}

impl What {
    fn from_args(args: &Value) -> Self {
        match args.get("what").and_then(Value::as_str) {
            Some("fuel") => What::Fuel,
            Some("payload") => What::Payload,
            _ => What::Both,
        }
    }
    fn fuel(self) -> bool {
        self != What::Payload
    }
    fn payload(self) -> bool {
        self != What::Fuel
    }
}

/// `target_kg` of fuel over the tanks: wing tanks first in proportion to
/// their capacity, the centreline tanks with the remainder. With no lateral
/// positions (or no wing tanks at all) every tank is filled in proportion.
/// `None` when the tanks cannot hold it or there are none.
/// The fuel to load so `zero_fuel_kg` plus fuel sits at the plan's takeoff
/// limit, when the plan's block would put it over and trimming stays at or
/// above the plan's minimum takeoff fuel. `None`: no trim needed, or none
/// that keeps the minimum (then the block is loaded and the limit said). Pure.
pub fn trim_fuel_to_limit(zero_fuel_kg: f64, block_kg: f64, max_tow_kg: f64, min_fuel_kg: f64) -> Option<f64> {
    if max_tow_kg <= 0.0 || min_fuel_kg <= 0.0 || zero_fuel_kg + block_kg <= max_tow_kg * 1.005 {
        return None;
    }
    let fuel = max_tow_kg - zero_fuel_kg;
    (fuel >= min_fuel_kg && fuel < block_kg).then_some(fuel)
}

pub fn distribute_fuel(target_kg: f64, capacity_kg: &[f64], lateral_m: &[f64]) -> Option<Vec<f64>> {
    let total: f64 = capacity_kg.iter().sum();
    if total <= 0.0 || !target_kg.is_finite() || target_kg < 0.0 || target_kg > total * 1.005 {
        return None;
    }
    let is_wing = |i: usize| lateral_m.get(i).is_some_and(|x| x.abs() > WING_TANK_M);
    let wing_cap: f64 = (0..capacity_kg.len())
        .filter(|i| is_wing(*i))
        .map(|i| capacity_kg[i])
        .sum();
    // No wings to prefer: one group, filled in proportion.
    if wing_cap <= 0.0 {
        return Some(capacity_kg.iter().map(|c| target_kg * c / total).collect());
    }
    let in_wings = target_kg.min(wing_cap);
    let centre_cap = total - wing_cap;
    let in_centre = (target_kg - in_wings).clamp(0.0, centre_cap);
    Some(
        capacity_kg
            .iter()
            .enumerate()
            .map(|(i, c)| {
                if is_wing(i) {
                    in_wings * c / wing_cap
                } else if centre_cap > 0.0 {
                    in_centre * c / centre_cap
                } else {
                    0.0
                }
            })
            .collect(),
    )
}

/// The `load_aircraft` tool.
pub struct LoadAircraft;

#[async_trait]
impl Tool for LoadAircraft {
    fn name(&self) -> &str {
        "load_aircraft"
    }
    fn description(&self) -> &str {
        "Set the fuel and the payload from the flight plan's load sheet. Use for \
         'load the aircraft', 'load the fuel and payload', 'set the fuel from the plan', \
         'fuel the aircraft', 'board the passengers'. On the ground and stopped only."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "what": { "type": "string", "enum": ["fuel", "payload", "both"] }
            },
            "additionalProperties": false
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let what = What::from_args(args);
        let fp = match plan_for_loading(&ctx.config.simbrief_id).await {
            Ok(fp) => fp,
            Err(line) => return Ok(line),
        };
        load_from_plan(ctx.xplane, &fp, what).await
    }
}

/// The plan the load comes from. A local plan is the pilot's own and stays;
/// a SimBrief plan is fetched again, because the pilot regenerates it on the
/// website and says the same words (`PlanFetch::Asked`'s rule), with the
/// shelf as the fallback when the site does not answer.
pub(crate) async fn plan_for_loading(simbrief_id: &str) -> std::result::Result<FlightPlan, String> {
    if let Some(local) = crate::simbrief::local_plan() {
        return Ok(local);
    }
    if !simbrief_id.trim().is_empty() {
        match crate::simbrief::fetch(simbrief_id).await {
            Ok(fp) => {
                // The Route tab's load sheet shows the plan that was loaded.
                crate::simbrief::publish_plan(&fp);
                return Ok(fp);
            }
            Err(e) => tracing::warn!("simbrief fetch failed: {e:#}"),
        }
    }
    crate::simbrief::loaded_plan().ok_or_else(|| {
        "No flight plan to load from. Say load the flight plan, or plan a flight to, first."
            .to_string()
    })
}

/// Gates, the arithmetic, the writes and the readback, against a given plan.
pub async fn load_from_plan(xp: &dyn XPlane, fp: &FlightPlan, what: What) -> Result<String> {
    let (on_ground, gs) = tokio::join!(
        xp.get_float(crate::datarefs::ON_GROUND_ANY),
        xp.get_float(GROUNDSPEED_MS),
    );
    if !on_ground.is_ok_and(sim::ground::on_ground) {
        return Ok("Negative. The aircraft is loaded on the ground, not in the air.".to_string());
    }
    if !gs.is_ok_and(sim::ground::stopped_mps) {
        return Ok("Negative. Stop the aircraft first and I'll load it.".to_string());
    }

    let (tanks, capacity, ratios, lateral, payload_now, empty, max) = tokio::join!(
        xp.get_float_array(crate::datarefs::FUEL_PER_TANK_KG),
        xp.get_float(FUEL_CAPACITY_KG),
        xp.get_float_array(TANK_RATIO),
        xp.get_float_array(TANK_LATERAL_M),
        xp.get_float(crate::datarefs::PAYLOAD_KG),
        xp.get_float(EMPTY_WEIGHT_KG),
        xp.get_float(MAX_WEIGHT_KG),
    );
    // A failed read is no measurement: nothing is written over tanks unseen.
    let Ok(tanks) = tanks else {
        return Ok("I can't read the tanks, so I'm not touching the load.".to_string());
    };
    let fuel_now: f64 = tanks.iter().sum();

    let spoken = |kg: f64| match &fp.weights {
        Some(w) => w.spoken(kg),
        None => crate::aircraft::say_fuel_kg(kg),
    };
    // A weight with no load sheet to name its unit is said in pounds, as the
    // preflight says it.
    let spoken_weight = |kg: f64| match &fp.weights {
        Some(w) => w.spoken(kg),
        None => crate::simbrief::spoken_weight(kg * 2.204_62, "pounds"),
    };

    // What the plan asks for. A half the plan does not carry is said and
    // skipped, never guessed.
    let mut notes: Vec<String> = Vec::new();
    let fuel_want = (what.fuel() && fp.block_fuel_kg > 0.0).then_some(fp.block_fuel_kg);
    if what.fuel() && fuel_want.is_none() {
        notes.push("The plan carries no fuel figure, so the tanks are as they were.".to_string());
    }
    let payload_want = fp
        .weights
        .as_ref()
        .filter(|w| what.payload() && w.payload_kg > 0.0)
        .map(|w| w.payload_kg);
    if what.payload() && payload_want.is_none() {
        notes.push("The plan has no load sheet, so the payload is as it was.".to_string());
    }
    if fuel_want.is_none() && payload_want.is_none() {
        return Ok(notes.join(" "));
    }

    // The plan's payload on an aircraft that is heavier empty than the plan
    // assumed goes over the plan's own takeoff limit. A dispatcher trims the
    // FUEL, never below the plan's minimum takeoff fuel, rather than load an
    // aircraft the crew will then refuse to fly: "Loaded... over the plan's
    // takeoff limit", then "Negative... over the plan's takeoff limit" at the
    // arm (LIPR 172, 2026-09-26, 100 lb over with 220 lb of fuel aboard).
    let mut fuel_want = fuel_want;
    let mut trimmed_from: Option<f64> = None;
    // Only for a plan that fits its own limit on its own numbers: the excess
    // is then the airframe's empty weight, not a plan that never added up.
    let plan_fits = fp
        .weights
        .as_ref()
        .is_some_and(|w| w.est_tow_kg > 0.0 && w.est_tow_kg <= w.max_tow_kg * 1.005);
    if let (Some(want), Some(w), Ok(empty), true) = (fuel_want, fp.weights.as_ref(), empty.as_ref(), plan_fits) {
        let payload = payload_want.or(payload_now.as_ref().ok().copied()).unwrap_or(0.0);
        // Over the AIRFRAME's maximum is the refusal below, never a trim.
        let within_airframe = max
            .as_ref()
            .is_ok_and(|m| *m <= 0.0 || *empty + payload + want <= m * 1.005);
        if let Some(fuel) = within_airframe
            .then(|| trim_fuel_to_limit(*empty + payload, want, w.max_tow_kg, fp.min_takeoff_fuel_kg))
            .flatten()
        {
            trimmed_from = Some(want);
            fuel_want = Some(fuel);
        }
    }

    // The tanks must hold it, and the airframe must carry it.
    let mut per_tank = None;
    if let Some(want) = fuel_want {
        let (Ok(cap), Ok(ratios)) = (capacity.as_ref(), ratios.as_ref()) else {
            return Ok("I can't read the tank sizes, so I'm not touching the fuel.".to_string());
        };
        let caps: Vec<f64> = ratios
            .iter()
            .take(tanks.len())
            .map(|r| r.max(0.0) * cap)
            .collect();
        let lateral = lateral.unwrap_or_default();
        match distribute_fuel(want, &caps, &lateral) {
            Some(d) => per_tank = Some(d),
            None => {
                return Ok(format!(
                    "Negative. The plan wants {} of fuel and the tanks hold {}.",
                    spoken(want),
                    spoken(caps.iter().sum())
                ))
            }
        }
    }
    if let (Ok(empty), Ok(max)) = (empty, max) {
        let all_up = empty
            + payload_want
                .or(payload_now.as_ref().ok().copied())
                .unwrap_or(0.0)
            + fuel_want.unwrap_or(fuel_now);
        if max > 0.0 && all_up > max * 1.005 {
            return Ok(format!(
                "Negative. That load is {} all up, over the airframe's maximum of {}. Nothing changed.",
                spoken_weight(all_up),
                spoken_weight(max)
            ));
        }
    }

    // Read first, left alone in state, then written and read back.
    let fuel_set = fuel_want.is_some_and(|w| matches(fuel_now, w));
    let payload_set = payload_want
        .zip(payload_now.as_ref().ok())
        .is_some_and(|(w, p)| matches(*p, w));
    if let (Some(d), false) = (per_tank.as_ref(), fuel_set) {
        for (i, kg) in d.iter().enumerate() {
            if tanks.get(i).is_none_or(|t| (t - kg).abs() > 0.5) {
                xp.set_float_array_elem(crate::datarefs::FUEL_PER_TANK_KG, i, *kg)
                    .await?;
            }
        }
    }
    if let (Some(w), false) = (payload_want, payload_set) {
        xp.set_float(crate::datarefs::PAYLOAD_KG, w).await?;
    }
    tokio::time::sleep(std::time::Duration::from_millis(300)).await;
    let (fuel_after, payload_after, total_after) = tokio::join!(
        xp.get_float(crate::datarefs::FUEL_TOTAL_KG),
        xp.get_float(crate::datarefs::PAYLOAD_KG),
        xp.get_float(crate::datarefs::TOTAL_WEIGHT_KG),
    );

    let mut ok = true;
    let mut lines = vec![format!("Loaded for {} to {}.", fp.origin, fp.destination)];
    if let Some(want) = fuel_want {
        match fuel_after {
            Ok(f) if matches(f, want) => lines.push(match trimmed_from {
                Some(block) => format!(
                    "Fuel {}, trimmed from the plan's {} to stay inside its takeoff limit.",
                    spoken(f),
                    spoken(block)
                ),
                None => format!(
                    "Fuel {}{}.",
                    spoken(f),
                    if fuel_set { ", already aboard" } else { "" }
                ),
            }),
            Ok(f) => {
                ok = false;
                lines.push(format!(
                    "The fuel did not take: {} aboard against the plan's {}. This aircraft may manage its own tanks.",
                    spoken(f),
                    spoken(want)
                ));
            }
            Err(_) => {
                ok = false;
                lines.push("I can't read the fuel back.".to_string());
            }
        }
    }
    if let Some(want) = payload_want {
        let who = fp.weights.as_ref().map_or(String::new(), |w| match w.pax {
            0 => String::new(),
            1 => ", 1 passenger".to_string(),
            n => format!(", {n} passengers"),
        });
        match payload_after {
            Ok(p) if matches(p, want) => lines.push(format!(
                "Payload {}{who}{}.",
                spoken(p),
                if payload_set { ", already aboard" } else { "" }
            )),
            Ok(p) => {
                ok = false;
                lines.push(format!(
                    "The payload did not take: {} aboard against the plan's {}.",
                    spoken(p),
                    spoken(want)
                ));
            }
            Err(_) => {
                ok = false;
                lines.push("I can't read the payload back.".to_string());
            }
        }
    }
    if ok {
        if let Ok(total) = total_after {
            let mut weight = format!("All up {}", spoken_weight(total));
            // The plan's limit is the operational one; over it is preflight's
            // refusal later, so it is said here while it costs nothing.
            if let Some(w) = fp.weights.as_ref().filter(|w| w.max_tow_kg > 0.0) {
                if total > w.max_tow_kg * 1.005 {
                    weight.push_str(&format!(
                        ", over the plan's takeoff limit of {}",
                        spoken_weight(w.max_tow_kg)
                    ));
                }
            }
            weight.push('.');
            lines.push(weight);
        }
    } else {
        lines[0] = format!("Loading for {} to {}.", fp.origin, fp.destination);
    }
    lines.extend(notes);

    crate::blackbox::record(
        "loading",
        json!({
            "event": "load_aircraft", "what": format!("{what:?}"),
            "origin": fp.origin, "destination": fp.destination,
            "fuel_want_kg": fuel_want, "fuel_trimmed_from_kg": trimmed_from, "fuel_before_kg": fuel_now,
            "fuel_after_kg": fuel_after.ok(), "per_tank_kg": per_tank,
            "payload_want_kg": payload_want, "payload_before_kg": payload_now.ok(),
            "payload_after_kg": payload_after.ok(), "total_kg": total_after.ok(),
            "ok": ok,
        }),
    );
    Ok(lines.join(" "))
}

#[cfg(test)]
mod tests {
    /// LIPR-LIBP 172 (2026-09-26): 2,550 lb all up with the plan's block,
    /// the plan's limit 2,450: trimmed by 100 lb, still over the minimum.
    #[test]
    fn a_block_over_the_plans_limit_is_trimmed_never_below_the_minimum() {
        let lb = |v: f64| v / 2.20462;
        let zfw = lb(2550.0 - 220.0);
        let fuel = super::trim_fuel_to_limit(zfw, lb(220.0), lb(2450.0), lb(90.0)).expect("trimmed");
        assert!((fuel * 2.20462 - 120.0).abs() < 1.0, "{}", fuel * 2.20462);
        assert_eq!(super::trim_fuel_to_limit(zfw, lb(220.0), lb(2450.0), lb(150.0)), None, "the minimum wins");
        assert_eq!(super::trim_fuel_to_limit(zfw, lb(100.0), lb(2450.0), lb(90.0)), None, "under the limit");
    }

    use super::*;
    use crate::simbrief::{PlanSource, PlanWeights};
    use crate::xplane::FakeXPlane;

    /// The 737 as probed live on the EGPH ramp, 2026-09-21.
    const B738_RATIOS: [f64; 3] = [0.187, 0.626, 0.187];
    const B738_LATERAL: [f64; 3] = [-7.01, 0.0, 7.01];
    const B738_CAPACITY: f64 = 20_893.8;

    fn b738_caps() -> Vec<f64> {
        B738_RATIOS.iter().map(|r| r * B738_CAPACITY).collect()
    }

    #[test]
    fn the_wings_fill_before_the_centre_tank() {
        let d = distribute_fuel(5_800.0, &b738_caps(), &B738_LATERAL).unwrap();
        assert!(
            (d[0] - 2_900.0).abs() < 0.1 && (d[2] - 2_900.0).abs() < 0.1,
            "{d:?}"
        );
        assert_eq!(d[1], 0.0, "a short leg flies with a dry centre tank");

        // Past the wings' 7,814 kg the rest goes to the centre.
        let d = distribute_fuel(12_000.0, &b738_caps(), &B738_LATERAL).unwrap();
        let wing = 0.187 * B738_CAPACITY;
        assert!(
            (d[0] - wing).abs() < 0.1 && (d[2] - wing).abs() < 0.1,
            "{d:?}"
        );
        assert!((d.iter().sum::<f64>() - 12_000.0).abs() < 0.1, "{d:?}");
    }

    #[test]
    fn two_wing_tanks_share_it_and_too_much_is_refused() {
        // The 172: two wing tanks, nothing on the centreline.
        let d = distribute_fuel(100.0, &[72.0, 72.0], &[-1.2, 1.2]).unwrap();
        assert_eq!(d, vec![50.0, 50.0]);
        // No lateral positions read: proportional over every tank.
        let d = distribute_fuel(20.0, &[30.0, 10.0], &[]).unwrap();
        assert_eq!(d, vec![15.0, 5.0]);
        assert!(distribute_fuel(150.0, &[72.0, 72.0], &[-1.2, 1.2]).is_none());
        assert!(distribute_fuel(10.0, &[], &[]).is_none());
        assert!(distribute_fuel(f64::NAN, &[72.0, 72.0], &[]).is_none());
    }

    fn plan(block_kg: f64, payload_kg: Option<f64>) -> FlightPlan {
        FlightPlan {
            source: PlanSource::SimBrief,
            origin: "EGPH".into(),
            destination: "EGTE".into(),
            alternate: String::new(),
            aircraft: "B738".into(),
            cruise_alt_ft: 35_000.0,
            distance_nm: 330.0,
            ete_min: 60.0,
            trip_fuel_kg: 3_400.0,
            reserve_kg: 1_100.0,
            route: "DCT".into(),
            wind_comp_kts: 0.0,
            wind_dir: 0.0,
            wind_spd_kts: 0.0,
            tod_to_dest_nm: 110.0,
            waypoints: Vec::new(),
            block_fuel_kg: block_kg,
            min_takeoff_fuel_kg: 5_600.0,
            weights: payload_kg.map(|p| PlanWeights {
                pax: 148,
                bags: 148,
                cargo_kg: 0.0,
                payload_kg: p,
                est_tow_kg: 62_000.0,
                max_tow_kg: 79_000.0,
                est_ldw_kg: 58_000.0,
                max_ldw_kg: 66_000.0,
                spoken_lbs: false,
            }),
            transition_altitude_ft: None,
            transition_level_ft: None,
            procedures: Default::default(),
            callsign: None,
            schedule: Default::default(),
        }
    }

    fn ramp(fuel: [f64; 3], payload: f64) -> FakeXPlane {
        let xp = FakeXPlane::with_reads(&[
            (crate::datarefs::ON_GROUND_ANY, 1.0),
            (GROUNDSPEED_MS, 0.0),
            (FUEL_CAPACITY_KG, B738_CAPACITY),
            (crate::datarefs::PAYLOAD_KG, payload),
            (EMPTY_WEIGHT_KG, 41_413.0),
            (MAX_WEIGHT_KG, 79_015.8),
            (crate::datarefs::FUEL_TOTAL_KG, fuel.iter().sum()),
            (
                crate::datarefs::TOTAL_WEIGHT_KG,
                41_413.0 + payload + fuel.iter().sum::<f64>(),
            ),
        ]);
        xp.set_read_array(crate::datarefs::FUEL_PER_TANK_KG, &fuel);
        xp.set_read_array(TANK_RATIO, &B738_RATIOS);
        xp.set_read_array(TANK_LATERAL_M, &B738_LATERAL);
        xp
    }

    /// The fake's reads never move, so a write into it is a write that did
    /// not take: exactly the add-on that rewrites its own tanks.
    #[tokio::test(start_paused = true)]
    async fn a_write_that_did_not_take_is_said_with_both_numbers() {
        let xp = ramp([2_921.0, 0.0, 2_921.0], 16_000.0);
        let line = load_from_plan(&xp, &plan(7_200.0, Some(14_300.0)), What::Both)
            .await
            .unwrap();
        assert!(
            xp.wrote(crate::datarefs::FUEL_PER_TANK_KG) && xp.wrote(crate::datarefs::PAYLOAD_KG)
        );
        assert_eq!(xp.last_write(crate::datarefs::PAYLOAD_KG), Some(14_300.0));
        assert!(
            line.contains(
                "The fuel did not take: 5,840 kilos aboard against the plan's 7,200 kilos."
            ),
            "{line}"
        );
        assert!(line.contains("The payload did not take"), "{line}");
        assert!(!line.contains("All up"), "{line}");
    }

    #[tokio::test(start_paused = true)]
    async fn a_load_already_aboard_is_left_alone() {
        let xp = ramp([3_600.0, 0.0, 3_600.0], 14_310.0);
        let line = load_from_plan(&xp, &plan(7_200.0, Some(14_300.0)), What::Both)
            .await
            .unwrap();
        assert!(
            !xp.wrote(crate::datarefs::FUEL_PER_TANK_KG) && !xp.wrote(crate::datarefs::PAYLOAD_KG)
        );
        assert_eq!(
            line,
            "Loaded for EGPH to EGTE. Fuel 7,200 kilos, already aboard. \
             Payload 14,310 kilos, 148 passengers, already aboard. All up 62,920 kilos."
        );
    }

    #[tokio::test(start_paused = true)]
    async fn only_the_half_that_was_asked_for_is_touched() {
        let xp = ramp([2_921.0, 0.0, 2_921.0], 16_000.0);
        load_from_plan(&xp, &plan(7_200.0, Some(14_300.0)), What::Fuel)
            .await
            .unwrap();
        assert!(
            xp.wrote(crate::datarefs::FUEL_PER_TANK_KG) && !xp.wrote(crate::datarefs::PAYLOAD_KG)
        );
    }

    #[tokio::test(start_paused = true)]
    async fn the_gates_refuse_and_write_nothing() {
        let fp = plan(7_200.0, Some(14_300.0));
        let air = ramp([2_921.0, 0.0, 2_921.0], 16_000.0);
        air.set_read(crate::datarefs::ON_GROUND_ANY, 0.0);
        assert!(load_from_plan(&air, &fp, What::Both)
            .await
            .unwrap()
            .starts_with("Negative"));
        let rolling = ramp([2_921.0, 0.0, 2_921.0], 16_000.0);
        rolling.set_read(GROUNDSPEED_MS, 4.0);
        assert!(load_from_plan(&rolling, &fp, What::Both)
            .await
            .unwrap()
            .contains("Stop the aircraft"));
        // More fuel than the tanks hold, and more load than the airframe carries.
        let parked = ramp([2_921.0, 0.0, 2_921.0], 16_000.0);
        let line = load_from_plan(&parked, &plan(25_000.0, None), What::Fuel)
            .await
            .unwrap();
        // No load sheet: the fuel is said in the airframe's own unit.
        assert!(
            line.starts_with("Negative. The plan wants") && line.contains("the tanks hold"),
            "{line}"
        );
        let line = load_from_plan(&parked, &plan(20_000.0, Some(19_000.0)), What::Both)
            .await
            .unwrap();
        assert!(
            line.contains("over the airframe's maximum of 79,020 kilos"),
            "{line}"
        );
        for xp in [&air, &rolling, &parked] {
            assert!(
                !xp.wrote(crate::datarefs::FUEL_PER_TANK_KG)
                    && !xp.wrote(crate::datarefs::PAYLOAD_KG)
            );
        }
    }

    /// The Route tab's feed: the plan's side in kilos, the unit the plan
    /// speaks, zeros where it carries no sheet, nothing with no plan.
    #[test]
    fn the_panel_feed_carries_the_plans_side_of_the_sheet() {
        use crate::simbrief::loadsheet_payload;
        assert_eq!(
            loadsheet_payload(Some(&plan(7_200.0, Some(14_300.0)))),
            "EGPH|EGTE|7200|14300|79000|148|kgs"
        );
        assert_eq!(
            loadsheet_payload(Some(&plan(7_200.0, None))),
            "EGPH|EGTE|7200|0|0|0|kgs"
        );
        assert_eq!(loadsheet_payload(None), "");
    }

    /// #95: full auto's arm decides from the ramp as read, and the loader
    /// then touches only the half the decision asked for.
    #[tokio::test(start_paused = true)]
    async fn the_arm_loads_the_fuel_and_leaves_a_payload_already_aboard() {
        let xp = ramp([1_000.0, 0.0, 1_000.0], 14_310.0);
        let fp = plan(7_200.0, Some(14_300.0));
        let decision = crate::full_auto::arm_load(
            &fp,
            true,
            Some(2_000.0),
            Some(14_310.0),
            crate::aircraft::LoadSheet::Stock,
            "Boeing 737-800",
        );
        assert_eq!(decision, crate::full_auto::ArmLoad::Load(What::Fuel));
        let crate::full_auto::ArmLoad::Load(what) = decision else { unreachable!() };
        let line = load_from_plan(&xp, &fp, what).await.unwrap();
        assert!(xp.wrote(crate::datarefs::FUEL_PER_TANK_KG), "{line}");
        assert!(!xp.wrote(crate::datarefs::PAYLOAD_KG), "{line}");
        // The fake never moves, so this is the add-on that undid the write.
        assert_eq!(crate::full_auto::arm_load_outcome(&line), "not_taken", "{line}");
    }

    /// The A330's load is unprobed (#95) and the 737's is the stock one.
    #[test]
    fn the_unprobed_airframes_say_so_in_their_profiles() {
        use crate::aircraft::{load, LoadSheet};
        assert_eq!(load("airbus_a330_300").unwrap().load_sheet, LoadSheet::Unprobed);
        assert_eq!(load("boeing_747_400").unwrap().load_sheet, LoadSheet::Unprobed);
        assert_eq!(load("boeing_737_800").unwrap().load_sheet, LoadSheet::Stock);
        assert_eq!(load("cessna_172sp").unwrap().load_sheet, LoadSheet::Stock);
    }

    /// A local plan has no load sheet: the fuel goes in, the payload is said
    /// to be as it was, never guessed.
    #[tokio::test(start_paused = true)]
    async fn a_plan_without_a_load_sheet_loads_the_fuel_only() {
        let xp = ramp([2_921.0, 0.0, 2_921.0], 16_000.0);
        let line = load_from_plan(&xp, &plan(7_200.0, None), What::Both)
            .await
            .unwrap();
        assert!(!xp.wrote(crate::datarefs::PAYLOAD_KG));
        assert!(
            line.ends_with("The plan has no load sheet, so the payload is as it was."),
            "{line}"
        );
    }
}
