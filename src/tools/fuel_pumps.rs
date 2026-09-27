//! Centre tank pumps: the one writer for `sim/cockpit2/fuel/fuel_tank_pump_on`.
//!
//! The stock 737's scripted start switches the CTR pumps on whatever the
//! centre holds (`B738.Z.autostart.lua`, step 4), and the load sheet fills the
//! wings first (`loading::distribute_fuel`), so any sector that fits in the
//! wings flew with both CTR LOW PRESSURE lights lit and the pumps running dry
//! for the whole flight (probed live 2026-09-27: `m_fuel` `[2916.8, 0.0,
//! 2918.1]`, pumps `[1, 1, 1]`). A real crew switches them off at the lights.
//!
//! Declared per airframe (`fuel_tank_pumps_off_when_empty:` in the profile,
//! the tank indices of the stock array), and honoured only for a tank the
//! airframe's own geometry puts on the CENTRELINE with wing tanks beside it
//! (`acf_tank_X`, the load sheet's rule): a main tank running empty is a fuel
//! emergency, never a switch to tidy, so a wrongly declared wing tank is
//! ignored (and said in the log), never written.
//!
//! Callers: the monitor's `centre_tank_dry` check (an automation flying: the
//! crew switches them off; a pilot flying: advised once) and the spoken
//! `set_centre_pumps` tool. The MD-82 power-up switches its pumps on here too.

use anyhow::Result;
use async_trait::async_trait;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::xplane::XPlane;

/// The tank boost pump switches, one element per tank. On the stock 737 and
/// the MD-82 this IS the cockpit switch (their xlua writes it only at the
/// start and at cold-and-dark init); on the A330 it is an OUTPUT of the
/// add-on's own buttons, rewritten every frame, so it is not declared there.
pub const TANK_PUMP_ON: &str = crate::datarefs::FUEL_TANK_PUMP_ON;
/// Fuel in each tank, kg.
pub const TANK_FUEL_KG: &str = crate::datarefs::FUEL_PER_TANK_KG;
/// Each tank's lateral position, m: zero is a centreline tank.
pub const TANK_LATERAL_M: &str = crate::datarefs::FUEL_TANK_LATERAL_M;

/// A tank at or below this is empty: the unusable few kilos a gauge still
/// shows, not zero. The recorded 737 read exactly 0.0; a tank with more than
/// this in it still feeds, and switching its pumps off would strand the fuel.
pub const EMPTY_TANK_KG: f64 = 5.0;

/// A tank this far off the centreline is a wing tank (the load sheet's rule,
/// `loading::WING_TANK_M`).
const WING_TANK_M: f64 = super::loading::WING_TANK_M;

/// How long a switch is given to show its new position before the readback.
const SETTLE_MS: u64 = 400;

/// One tick's picture of the tanks. All three arrays or none: a failed read
/// of any is NO measurement, and nothing is decided on it.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Tanks {
    pub kg: Vec<f64>,
    pub pumps: Vec<f64>,
    pub lateral_m: Vec<f64>,
}

/// A centreline tank with wing tanks beside it: the only kind whose pumps
/// come off when it runs dry. A single-tank airframe's one tank is its main,
/// wherever it sits.
pub fn is_centre_tank(i: usize, lateral_m: &[f64]) -> bool {
    lateral_m.get(i).is_some_and(|x| x.abs() <= WING_TANK_M)
        && lateral_m.iter().any(|x| x.abs() > WING_TANK_M)
}

/// The declared tanks that are centre tanks by the airframe's geometry, in
/// declared order. A declared wing tank is dropped here, never written.
pub fn honoured(declared: &[usize], lateral_m: &[f64]) -> Vec<usize> {
    declared
        .iter()
        .copied()
        .filter(|i| is_centre_tank(*i, lateral_m))
        .collect()
}

/// The declared tanks that are wrongly declared (a main tank, or no tank at
/// all by the geometry): for the log, so a bad profile line is seen.
pub fn rejected(declared: &[usize], lateral_m: &[f64]) -> Vec<usize> {
    declared
        .iter()
        .copied()
        .filter(|i| !is_centre_tank(*i, lateral_m))
        .collect()
}

/// THE decision, pure: which declared centre tanks are empty with their pumps
/// still on while an engine runs. Empty on a failed read (`None`), with no
/// engine running (the scripted start switches them on before the engines),
/// or on an airframe that declares nothing.
pub fn pumps_to_switch_off(
    declared: &[usize],
    tanks: Option<&Tanks>,
    engine_running: bool,
) -> Vec<usize> {
    let Some(t) = tanks else {
        return Vec::new();
    };
    if !engine_running {
        return Vec::new();
    }
    honoured(declared, &t.lateral_m)
        .into_iter()
        .filter(|i| {
            let on = t.pumps.get(*i).is_some_and(|p| *p > 0.5);
            let empty =
                t.kg.get(*i)
                    .is_some_and(|kg| kg.is_finite() && *kg <= EMPTY_TANK_KG);
            on && empty
        })
        .collect()
}

/// One switch as it went: what the tank held, the pump before and after.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PumpSwitch {
    pub tank: usize,
    pub kg: Option<f64>,
    pub before: Option<f64>,
    pub after: Option<f64>,
    pub want_on: bool,
}

impl PumpSwitch {
    /// The readback agrees with the intent.
    pub fn took(&self) -> bool {
        self.after.is_some_and(|a| (a > 0.5) == self.want_on)
    }
}

/// Every switch took.
pub fn all_took(s: &[PumpSwitch]) -> bool {
    !s.is_empty() && s.iter().all(PumpSwitch::took)
}

async fn read_array(xp: &dyn XPlane, name: &str) -> Option<Vec<f64>> {
    xp.get_float_array(name)
        .await
        .ok()
        .filter(|v| !v.is_empty())
}

/// Read the tank picture, all or nothing.
pub async fn read_tanks(xp: &dyn XPlane) -> Option<Tanks> {
    let (kg, pumps, lateral_m) = tokio::join!(
        read_array(xp, TANK_FUEL_KG),
        read_array(xp, TANK_PUMP_ON),
        read_array(xp, TANK_LATERAL_M),
    );
    Some(Tanks {
        kg: kg?,
        pumps: pumps?,
        lateral_m: lateral_m?,
    })
}

/// Switch the named tanks' pumps and read each back against the intent. A
/// switch already in place is left alone (its `before` equals its `after`).
/// The only write to [`TANK_PUMP_ON`] in the app.
pub async fn switch_pumps(xp: &dyn XPlane, tanks: &[usize], on: bool) -> Vec<PumpSwitch> {
    let before = read_array(xp, TANK_PUMP_ON).await;
    let kg = read_array(xp, TANK_FUEL_KG).await;
    let mut wrote = false;
    for &i in tanks {
        let is_on = before.as_ref().and_then(|b| b.get(i)).map(|v| *v > 0.5);
        if is_on != Some(on)
            && xp
                .set_int_array_elem(TANK_PUMP_ON, i, on as i64)
                .await
                .is_ok()
        {
            wrote = true;
        }
    }
    if wrote {
        tokio::time::sleep(std::time::Duration::from_millis(SETTLE_MS)).await;
    }
    let after = read_array(xp, TANK_PUMP_ON).await;
    tanks
        .iter()
        .map(|&i| PumpSwitch {
            tank: i,
            kg: kg.as_ref().and_then(|k| k.get(i)).copied(),
            before: before.as_ref().and_then(|b| b.get(i)).copied(),
            after: after.as_ref().and_then(|a| a.get(i)).copied(),
            want_on: on,
        })
        .collect()
}

/// Blackbox `fuel_pumps`: one record per switch, with who asked.
pub fn record(event: &str, source: &str, switches: &[PumpSwitch]) {
    for s in switches {
        crate::blackbox::record(
            "fuel_pumps",
            json!({
                "event": event, "source": source, "tank": s.tank, "kg": s.kg,
                "before": s.before, "after": s.after, "want_on": s.want_on,
                "took": s.took(),
            }),
        );
    }
}

/// The spoken line for a monitor switch-off (the callout already said
/// "centre pumps off"): only a switch that did not take is worth a word.
pub fn failed_line(switches: &[PumpSwitch]) -> Option<String> {
    (!all_took(switches))
        .then(|| "The centre pumps did not switch off. Check them on the overhead.".to_string())
}

/// "Centre pumps off" / "centre pumps on": the declared centre tanks' pumps,
/// read back. Off is refused while the tank still holds fuel (the pumps are
/// what feeds it; switching them off strands it), naming the kilos.
pub struct SetCentrePumps {
    pub declared: Vec<usize>,
}

#[async_trait]
impl Tool for SetCentrePumps {
    fn name(&self) -> &str {
        "set_centre_pumps"
    }
    fn description(&self) -> &str {
        "Switch the centre tank fuel pumps on or off. Use for 'centre pumps off', \
         'center pumps on', 'turn off the centre pumps'."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": { "on": { "type": "boolean" } },
            "required": ["on"]
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let on = args.get("on").and_then(Value::as_bool).unwrap_or(false);
        Ok(set_centre_pumps(ctx.xplane, &self.declared, on).await)
    }
}

/// The tool's body over any sim, so it tests on the fake.
pub async fn set_centre_pumps(xp: &dyn XPlane, declared: &[usize], on: bool) -> String {
    if declared.is_empty() {
        return "This aircraft has no centre tank pumps for me to switch.".to_string();
    }
    let Some(tanks) = read_tanks(xp).await else {
        return "I can't read the fuel tanks, so I've left the centre pumps alone.".to_string();
    };
    let centre = honoured(declared, &tanks.lateral_m);
    if centre.is_empty() {
        tracing::warn!(
            ?declared,
            "fuel pumps: no declared tank is a centre tank; ignored"
        );
        return "I can't find a centre tank on this aircraft, so I've left the pumps alone."
            .to_string();
    }
    if !on {
        let held: f64 = centre
            .iter()
            .filter_map(|i| tanks.kg.get(*i))
            .filter(|kg| **kg > EMPTY_TANK_KG)
            .sum();
        if held > 0.0 {
            return format!(
                "Negative, the centre tank still holds {} kilos. Its pumps stay on until it's empty.",
                crate::auto_status::commas(held.round() as i64)
            );
        }
    }
    let already = centre
        .iter()
        .all(|i| tanks.pumps.get(*i).is_some_and(|p| (*p > 0.5) == on));
    if already {
        return format!(
            "Centre pumps are already {}.",
            if on { "on" } else { "off" }
        );
    }
    let switches = switch_pumps(xp, &centre, on).await;
    record("switched", "pilot", &switches);
    let word = if on { "on" } else { "off" };
    if all_took(&switches) {
        format!("Centre pumps {word}.")
    } else {
        format!("The centre pumps did not switch {word}. Check them on the overhead.")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim::xplane::FakeXPlane;

    /// The stock 737 as probed live, 2026-09-27: centre (index 1) empty,
    /// every pump on, the wings 7 m either side.
    fn b738() -> Tanks {
        Tanks {
            kg: vec![2916.8, 0.0, 2918.1],
            pumps: vec![1.0, 1.0, 1.0],
            lateral_m: vec![-7.01, 0.0, 7.01],
        }
    }

    #[test]
    fn an_empty_centre_with_its_pumps_on_and_an_engine_running_is_switched_off() {
        assert_eq!(pumps_to_switch_off(&[1], Some(&b738()), true), vec![1]);
    }

    #[test]
    fn a_centre_with_fuel_in_it_keeps_its_pumps() {
        let mut t = b738();
        t.kg[1] = 3_200.0;
        assert!(pumps_to_switch_off(&[1], Some(&t), true).is_empty());
        // A few kilos over the unusable is still fuel.
        t.kg[1] = EMPTY_TANK_KG + 1.0;
        assert!(pumps_to_switch_off(&[1], Some(&t), true).is_empty());
    }

    #[test]
    fn a_pump_already_off_is_left_alone() {
        let mut t = b738();
        t.pumps[1] = 0.0;
        assert!(pumps_to_switch_off(&[1], Some(&t), true).is_empty());
    }

    #[test]
    fn a_failed_read_is_no_measurement() {
        assert!(pumps_to_switch_off(&[1], None, true).is_empty());
        // A short array (an element missing) decides nothing for that tank.
        let mut t = b738();
        t.kg.truncate(1);
        assert!(pumps_to_switch_off(&[1], Some(&t), true).is_empty());
    }

    #[test]
    fn nothing_before_an_engine_runs() {
        // The scripted start switches the CTR pumps on as step 4, before
        // either engine turns: that is its business, not a callout.
        assert!(pumps_to_switch_off(&[1], Some(&b738()), false).is_empty());
    }

    #[test]
    fn an_airframe_that_declares_nothing_is_never_touched() {
        assert!(pumps_to_switch_off(&[], Some(&b738()), true).is_empty());
    }

    #[test]
    fn a_main_tank_is_never_touched_even_when_declared() {
        // Tank 0 is the 737's LEFT wing: declared by mistake, and dry, it is
        // still a fuel emergency, not a switch to tidy.
        let mut t = b738();
        t.kg[0] = 0.0;
        assert!(pumps_to_switch_off(&[0, 2], Some(&t), true).is_empty());
        assert_eq!(rejected(&[0, 1, 2], &t.lateral_m), vec![0, 2]);
        // A single-tank airframe's one tank is its main, on the centreline.
        let single = Tanks {
            kg: vec![0.0],
            pumps: vec![1.0],
            lateral_m: vec![0.0],
        };
        assert!(pumps_to_switch_off(&[0], Some(&single), true).is_empty());
        // An index past the tanks is nothing.
        assert!(pumps_to_switch_off(&[7], Some(&b738()), true).is_empty());
    }

    #[test]
    fn the_md82_centre_is_tank_zero() {
        // MD80.acf: tank 0 "Center" at x 0, the wings at 25.4 ft either side.
        let t = Tanks {
            kg: vec![0.0, 3_000.0, 3_000.0],
            pumps: vec![1.0, 1.0, 1.0],
            lateral_m: vec![0.0, -7.74, 7.74],
        };
        assert_eq!(pumps_to_switch_off(&[0], Some(&t), true), vec![0]);
    }

    fn fake_737(kg_centre: f64, centre_pump: f64) -> FakeXPlane {
        let xp = FakeXPlane::new();
        xp.set_read_array(TANK_FUEL_KG, &[2916.8, kg_centre, 2918.1]);
        xp.set_read_array(TANK_PUMP_ON, &[1.0, centre_pump, 1.0]);
        xp.set_read_array(TANK_LATERAL_M, &[-7.01, 0.0, 7.01]);
        xp.apply_array_writes(TANK_PUMP_ON);
        xp
    }

    #[tokio::test]
    async fn the_spoken_off_switches_the_centre_and_reads_it_back() {
        let xp = fake_737(0.0, 1.0);
        let line = set_centre_pumps(&xp, &[1], false).await;
        assert_eq!(line, "Centre pumps off.");
        assert_eq!(xp.index_writes(TANK_PUMP_ON), vec![(1, 0.0)]);
    }

    #[tokio::test]
    async fn the_spoken_off_is_refused_while_the_centre_holds_fuel() {
        let xp = fake_737(3_240.0, 1.0);
        let line = set_centre_pumps(&xp, &[1], false).await;
        assert!(line.contains("3,240 kilos"), "{line}");
        assert!(!xp.wrote(TANK_PUMP_ON));
    }

    #[tokio::test]
    async fn the_spoken_on_is_allowed_whatever_the_centre_holds() {
        let xp = fake_737(3_240.0, 0.0);
        assert_eq!(set_centre_pumps(&xp, &[1], true).await, "Centre pumps on.");
        assert_eq!(xp.index_writes(TANK_PUMP_ON), vec![(1, 1.0)]);
    }

    #[tokio::test]
    async fn an_airframe_with_no_centre_pumps_says_so() {
        let xp = fake_737(0.0, 1.0);
        let line = set_centre_pumps(&xp, &[], false).await;
        assert!(line.contains("no centre tank pumps"), "{line}");
        assert!(!xp.wrote(TANK_PUMP_ON));
    }

    #[tokio::test]
    async fn a_switch_that_does_not_take_is_said() {
        // The switch never moves: the preset array stays as it was.
        let xp = FakeXPlane::new();
        xp.set_read_array(TANK_FUEL_KG, &[2916.8, 0.0, 2918.1]);
        xp.set_read_array(TANK_PUMP_ON, &[1.0, 1.0, 1.0]);
        xp.set_read_array(TANK_LATERAL_M, &[-7.01, 0.0, 7.01]);
        let line = set_centre_pumps(&xp, &[1], false).await;
        assert!(line.contains("did not switch off"), "{line}");
    }
}
