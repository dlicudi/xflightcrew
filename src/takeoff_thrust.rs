//! Takeoff and climb thrust flown to the type's N1, as a crew sets it
//! (`takeoff_thrust:`, [`crate::aircraft::TakeoffThrustProfile`]).
//!
//! The light Zibo 737 (43 to 48 t) took off with the levers at 1.0, 104 %
//! N1, above even the 26K rating's figure (98.0 at 15 C on the field), and
//! the autopilot took a 4,600 to 4,700 fpm climb at 167 to 169 kt off the
//! hand-flown climb (LEMG, 2026-10-07: "it still felt steep"). Its reduction
//! at 1,000 ft went to a lever ratio of 0.9, a stock-737 hypothesis nothing
//! measured. A crew derates when light: it picks the takeoff rating for the
//! weight, reads the takeoff N1 for the field's OAT and pressure altitude
//! off the FMC, sets it by 80 kt, and at the thrust reduction height sets
//! climb N1.
//!
//! Here the rating is chosen by weight (lightest first, a lighter rating
//! only where the runway still fits its longer roll), reduced further by
//! the highest assumed temperature its tables allow where the runway has
//! room for the most that can take off, the N1 read off the declared
//! tables (the Zibo FMC's own, `B738.calc.lua`), and the lever
//! flown to it on the MEASURED N1 ([`N1Loop`]): led on the declared
//! lever-to-N1 line, trimmed on the N1 only once the engines have stopped
//! spooling, never above the airframe's highest lever.
//!
//! An airframe that declares nothing keeps the levers to full and its
//! `climb_thrust_ratio`.

use crate::aircraft::{AssumedTemp, Grid, N1Table, TakeoffThrustProfile, ThrustRating};
use crate::takeoff_trim::{bracket, lerp};
use crate::xplane::XPlane;

/// The field's pressure altitude, feet (the Zibo FMC reads the same).
const PRESSURE_ALT_FT: &str = "sim/flightmodel2/position/pressure_altitude";

/// The table's N1 at `temp_c` and `alt_ft`: bilinear between the rows and
/// columns that bracket them, clamped at the table's edges (never
/// extrapolated). `None` for a malformed table or a reading that is not a
/// number. Pure.
pub fn table_n1(t: &N1Table, temp_c: f64, alt_ft: f64) -> Option<f64> {
    bilinear(&t.temp_c, &t.alt_ft, &t.n1, temp_c, alt_ft)
}

/// [`table_n1`] for a [`Grid`]: `row` on its rows, `col` on its columns.
/// Pure.
pub fn grid_at(g: &Grid, row: f64, col: f64) -> Option<f64> {
    bilinear(&g.rows, &g.cols, &g.values, row, col)
}

fn bilinear(rows: &[f64], cols: &[f64], v: &[Vec<f64>], row: f64, col: f64) -> Option<f64> {
    if !row.is_finite() || !col.is_finite() || rows.is_empty() || cols.is_empty() {
        return None;
    }
    if v.len() != rows.len() || v.iter().any(|r| r.len() != cols.len()) {
        return None;
    }
    let (r0, r1) = bracket(rows, row);
    let (c0, c1) = bracket(cols, col);
    let at_row = |r: usize| lerp(cols[c0], v[r][c0], cols[c1], v[r][c1], col);
    Some(lerp(rows[r0], at_row(r0), rows[r1], at_row(r1), row))
}

/// The FMC's correction is no figure past this (its tables mark the
/// combinations they do not cover 99, and it refuses above 17).
const ADJUST_MAX_N1: f64 = 17.0;

/// The assumed temperature a crew takes (the highest the tables allow,
/// whole degrees) and its N1, as the FMC computes it: the rating's N1 at
/// that temperature, less the correction for how far it is above the OAT.
/// `None` where the tables allow none: at or under the flat-rated minimum
/// or the OAT, off the reduced table, or past the correction's cover. Pure.
pub fn assumed_n1(a: &AssumedTemp, oat_c: f64, pa_ft: f64) -> Option<(f64, f64)> {
    let top = a.reduced.temp_c.last().copied()?;
    let max = grid_at(&a.max_temp, oat_c, pa_ft)?;
    if max > top {
        return None;
    }
    let atm = max.floor();
    let min = grid_at(&a.min_temp, 0.0, pa_ft)?;
    if atm < min || atm <= oat_c {
        return None;
    }
    let reduced = table_n1(&a.reduced, atm, pa_ft)?;
    let adjust = grid_at(&a.adjust, atm - oat_c, oat_c)?;
    (0.0..=ADJUST_MAX_N1)
        .contains(&adjust)
        .then_some((atm, reduced - adjust))
}

/// An assumed temperature takes at most a quarter of the rating's thrust
/// (its maximum table is drawn to that): the roll it may need is the
/// rating's over this.
const ASSUMED_THRUST_FLOOR: f64 = 0.75;

/// The runway a rating must still fit: what the full rating needs (the
/// one runway rule, `aircraft::takeoff_needed_m`) and what lies ahead.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RunwayFit {
    pub needed_full_m: f64,
    pub available_m: f64,
}

/// The roll a rating needs against the full rating's: in proportion to the
/// thrust it gives up (a roll is about W^2 / T). Pure.
pub fn needed_at_rating_m(needed_full_m: f64, full_klbf: f64, klbf: f64) -> f64 {
    if klbf > 0.0 && full_klbf > klbf {
        needed_full_m * full_klbf / klbf
    } else {
        needed_full_m
    }
}

/// The rating a crew takes for this weight: the first declared (lightest
/// first) the aircraft is under `below_t` for whose longer roll still fits
/// the runway; else the full rating (the one with no `below_t`, else the
/// strongest). `None` for a weight that is not a number or no ratings. Pure.
pub fn choose_rating(
    p: &TakeoffThrustProfile,
    weight_t: f64,
    runway: Option<RunwayFit>,
) -> Option<&ThrustRating> {
    if !weight_t.is_finite() || p.ratings.is_empty() {
        return None;
    }
    let full_klbf = p.ratings.iter().map(|r| r.klbf).fold(0.0, f64::max);
    let full = p
        .ratings
        .iter()
        .find(|r| r.below_t.is_none())
        .or_else(|| p.ratings.iter().max_by(|a, b| a.klbf.total_cmp(&b.klbf)));
    let fits = |r: &ThrustRating| {
        runway
            .is_none_or(|f| needed_at_rating_m(f.needed_full_m, full_klbf, r.klbf) <= f.available_m)
    };
    p.ratings
        .iter()
        .find(|r| r.below_t.is_some_and(|b| weight_t < b) && fits(r))
        .or(full)
}

/// What the takeoff will fly: the rating, its takeoff N1 and the climb N1
/// at the reduction height.
#[derive(Debug, Clone, PartialEq)]
pub struct ThrustPlan {
    pub rating: String,
    /// The assumed temperature, C, where one was taken.
    pub assumed_c: Option<f64>,
    pub to_n1: f64,
    pub climb_n1: f64,
    pub weight_t: f64,
    pub oat_c: f64,
    pub field_pa_ft: f64,
}

/// The plan for this weight, OAT and field pressure altitude. The climb N1
/// is read at the reduction height on the field's OAT (the climb's ram rise
/// and its lapse over 1,000 ft about cancel), and never above the takeoff
/// N1: a crew does not add thrust at the reduction (the FMC's derated climb
/// after a derated takeoff). Pure.
pub fn plan(
    p: &TakeoffThrustProfile,
    weight_t: f64,
    oat_c: f64,
    field_pa_ft: f64,
    reduction_agl_ft: f64,
    runway: Option<RunwayFit>,
) -> Option<ThrustPlan> {
    let r = choose_rating(p, weight_t, runway)?;
    let rated = table_n1(&r.takeoff, oat_c, field_pa_ft)?.min(p.full_n1);
    // An assumed temperature only on a runway with room for the most it
    // can take; a runway not known has none.
    let full_klbf = p.ratings.iter().map(|x| x.klbf).fold(0.0, f64::max);
    let room = runway.is_some_and(|f| {
        needed_at_rating_m(f.needed_full_m, full_klbf, r.klbf) / ASSUMED_THRUST_FLOOR
            <= f.available_m
    });
    let assumed = r
        .assumed
        .as_ref()
        .filter(|_| room)
        .and_then(|a| assumed_n1(a, oat_c, field_pa_ft))
        .filter(|&(_, n1)| n1 < rated);
    let to_n1 = assumed.map_or(rated, |(_, n1)| n1);
    let climb_n1 = table_n1(&p.climb, oat_c, field_pa_ft + reduction_agl_ft.max(0.0))?.min(to_n1);
    Some(ThrustPlan {
        rating: r.name.clone(),
        assumed_c: assumed.map(|(t, _)| t),
        to_n1,
        climb_n1,
        weight_t,
        oat_c,
        field_pa_ft,
    })
}

/// What the crew says: "Takeoff thrust 22K, N1 91.6.", with an assumed
/// temperature "Takeoff thrust 22K, assumed temperature 61, N1 85.9." Pure.
pub fn line(plan: &ThrustPlan) -> String {
    match plan.assumed_c {
        Some(t) => format!(
            "Takeoff thrust {}, assumed temperature {t:.0}, N1 {:.1}.",
            plan.rating, plan.to_n1
        ),
        None => format!("Takeoff thrust {}, N1 {:.1}.", plan.rating, plan.to_n1),
    }
}

/// An engine under this N1 is not running.
const RUNNING_N1: f64 = 10.0;

/// The N1 the levers are flown on: the mean of the running engines (one
/// lever axis drives them all). `None` with none running. Pure.
pub fn mean_n1(arr: &[f64]) -> Option<f64> {
    let running: Vec<f64> = arr
        .iter()
        .copied()
        .filter(|v| v.is_finite() && *v > RUNNING_N1)
        .collect();
    (!running.is_empty()).then(|| running.iter().sum::<f64>() / running.len() as f64)
}

/// The plan for the aircraft as it sits now: its weight, the OAT and the
/// pressure altitude, read together. `None` when any does not read (no
/// measurement: the takeoff flies the levers as it always did).
pub async fn plan_now(
    xp: &dyn XPlane,
    p: &TakeoffThrustProfile,
    reduction_agl_ft: f64,
    runway: Option<RunwayFit>,
) -> Option<ThrustPlan> {
    let (kg, oat, pa) = tokio::join!(
        xp.get_float(sim::datarefs::TOTAL_WEIGHT_KG),
        xp.get_float(sim::datarefs::OAT_DEG_C),
        xp.get_float(PRESSURE_ALT_FT),
    );
    let kg = kg.ok().filter(|w| w.is_finite() && *w > 0.0)?;
    plan(
        p,
        kg / 1000.0,
        oat.ok()?,
        pa.ok()?,
        reduction_agl_ft,
        runway,
    )
}

/// The rating for the aircraft as it weighs now, for its trim table. No
/// runway: the after-start configuration does not know it yet, and the
/// pre-roll one sets the trim again on the rating the roll flies.
pub async fn rating_now(xp: &dyn XPlane, p: &TakeoffThrustProfile) -> Option<String> {
    let kg = xp
        .get_float(sim::datarefs::TOTAL_WEIGHT_KG)
        .await
        .ok()
        .filter(|w| w.is_finite() && *w > 0.0)?;
    choose_rating(p, kg / 1000.0, None).map(|r| r.name.clone())
}

/// The N1's rate is low-passed over this, seconds.
const RATE_TAU_S: f64 = 1.0;
/// Faster than this (percent a second) the engines are still spooling to
/// the last lever, and the shortfall is the spool's, not the line's.
const SPOOL_RATE: f64 = 1.5;
/// The trim works on where the N1 is GOING: the reading plus its rate
/// over about the spool's time constant, seconds. On the reading alone an
/// integral against a spooling engine winds past and rings.
const LEAD_S: f64 = 2.0;
/// Within this of the target the lever is left (percent).
const DEADBAND: f64 = 0.2;
/// Lever per percent of N1 error per second. Against the CFM56's 84 % of N1
/// over the lever and a 2 to 3 s spool, a crossover near 0.25 rad/s: settles
/// in about ten seconds without ringing.
const KI: f64 = 0.006;
/// Above the target and still rising the lever comes back this much
/// harder: an overshoot is taken off before the spool carries it further.
const KI_OVER: f64 = KI;
/// The line is first aimed this far under the target (percent): the N1
/// comes up to it from below, and an engine whose line runs high does not
/// carry the spool past the target before the trim can take it off.
const APPROACH_UNDER: f64 = 1.5;
/// The most the measured N1 may move the lever off its line.
const TRIM_MAX: f64 = 0.2;
/// The lever is the law's when the commanded lever is within this of it.
const AT_CAP: f64 = 0.01;

/// What the N1 loop did this tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum N1Step {
    /// No N1 read: no measurement, the lever held where it was led.
    Blind,
    /// Another law holds the lever under this one (the ramp, the swinging
    /// nose, the overspeed guard): the shortfall is not the line's.
    Limited,
    /// The engines are still spooling to the last lever.
    Spooling,
    /// Settled within the deadband.
    Held,
    /// Settled off the target: the lever trimmed toward it.
    Trimmed,
}

/// The lever that gives the target N1, flown on the measured N1. The lever
/// LEADS (the line from idle to full puts it where the N1 should end) and
/// the N1 lags it by seconds, so the measured N1 only trims the line once
/// the engines have settled; above the target and still rising it trims at
/// once (an overshoot is never waited out).
#[derive(Debug, Clone, PartialEq)]
pub struct N1Loop {
    idle_n1: f64,
    full_n1: f64,
    lever_max: f64,
    trim: f64,
    rate: f64,
    last_n1: Option<f64>,
}

impl N1Loop {
    pub fn new(p: &TakeoffThrustProfile) -> Self {
        let lever_max = if p.lever_max.is_finite() && p.lever_max > 0.0 {
            p.lever_max.min(1.0)
        } else {
            1.0
        };
        let span = p.full_n1 - p.idle_n1;
        let under = if span.is_finite() && span > 0.0 {
            -lever_max * APPROACH_UNDER / span
        } else {
            0.0
        };
        N1Loop {
            idle_n1: p.idle_n1,
            full_n1: p.full_n1,
            lever_max,
            trim: under,
            rate: 0.0,
            last_n1: None,
        }
    }

    /// The line's lever for `n1`. Pure.
    pub fn feed_forward(&self, n1: f64) -> f64 {
        let span = self.full_n1 - self.idle_n1;
        if !span.is_finite() || span <= 0.0 {
            return self.lever_max;
        }
        (self.lever_max * (n1 - self.idle_n1) / span).clamp(0.0, self.lever_max)
    }

    /// The lever the law allows this tick for `target_n1`, given the N1
    /// read (`None`: not read), the lever commanded last tick and the tick.
    pub fn step(
        &mut self,
        target_n1: f64,
        n1: Option<f64>,
        lever_now: f64,
        dt_s: f64,
    ) -> (f64, N1Step) {
        let dt = dt_s.clamp(0.0, 1.0);
        let n1 = n1.filter(|v| v.is_finite());
        // A first reading has no rate yet: the engines may be spooling.
        let rated = n1.is_some() && self.last_n1.is_some();
        if let (Some(now), Some(last)) = (n1, self.last_n1) {
            if dt > 0.0 {
                let raw = (now - last) / dt;
                self.rate += (raw - self.rate) * (dt / RATE_TAU_S).min(1.0);
            }
        }
        self.last_n1 = n1;
        let cap = |s: &Self| (s.feed_forward(target_n1) + s.trim).clamp(0.0, s.lever_max);
        let before = cap(self);
        let step = match n1 {
            None => N1Step::Blind,
            Some(now) => {
                let over = target_n1 - now < -DEADBAND && self.rate >= 0.0;
                let err = target_n1 - (now + LEAD_S * self.rate);
                if !over && lever_now < before - AT_CAP {
                    N1Step::Limited
                } else if !rated || (!over && self.rate.abs() > SPOOL_RATE) {
                    N1Step::Spooling
                } else if err.abs() <= DEADBAND {
                    N1Step::Held
                } else {
                    let ki = if over { KI_OVER } else { KI };
                    self.trim = (self.trim + ki * err * dt).clamp(-TRIM_MAX, TRIM_MAX);
                    N1Step::Trimmed
                }
            }
        };
        (cap(self), step)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn zibo() -> TakeoffThrustProfile {
        let yaml = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/profiles/zibo_738.yaml"
        ))
        .unwrap();
        let p: crate::aircraft::AircraftProfile = serde_yaml::from_str(&yaml).unwrap();
        p.takeoff_thrust
            .expect("the Zibo declares its takeoff thrust")
    }

    fn rating<'a>(p: &'a TakeoffThrustProfile, name: &str) -> &'a ThrustRating {
        p.ratings.iter().find(|r| r.name == name).unwrap()
    }

    /// The FMC's own figures (B738.calc.lua `takeoff_thrust`, `climb_thrust`):
    /// corners, a grid point, a bilinear middle, clamped off the table.
    #[test]
    fn the_tables_read_as_the_fmc_reads_them() {
        let p = zibo();
        assert_eq!(table_n1(&rating(&p, "22K").takeoff, 15.0, 0.0), Some(91.6));
        assert_eq!(table_n1(&rating(&p, "24K").takeoff, 15.0, 0.0), Some(93.8));
        assert_eq!(table_n1(&rating(&p, "26K").takeoff, 15.0, 0.0), Some(98.0));
        assert_eq!(
            table_n1(&rating(&p, "26K").takeoff, -50.0, -2000.0),
            Some(84.4)
        );
        assert_eq!(
            table_n1(&rating(&p, "22K").takeoff, 60.0, 10000.0),
            Some(89.2)
        );
        // 22K at 17.5 C and 500 ft: between 91.6/92.2 (15 C) and 92.3/93.0
        // (20 C): 91.9 and 92.65, 92.275 between.
        let n = table_n1(&rating(&p, "22K").takeoff, 17.5, 500.0).unwrap();
        assert!((n - 92.275).abs() < 0.001, "{n}");
        // Off the table: its edge.
        assert_eq!(
            table_n1(&rating(&p, "22K").takeoff, 70.0, 12000.0),
            table_n1(&rating(&p, "22K").takeoff, 60.0, 10000.0)
        );
        assert_eq!(table_n1(&p.climb, 15.0, 0.0), Some(90.6));
        assert_eq!(table_n1(&p.climb, 15.0, 5000.0), Some(93.2));
        assert_eq!(table_n1(&p.climb, f64::NAN, 0.0), None);
    }

    /// Lightest first: 22K under 60 t, 24K under 70 t, else 26K; the
    /// boundaries go to the heavier rating.
    #[test]
    fn the_rating_is_chosen_for_the_weight() {
        let p = zibo();
        let name = |w: f64| choose_rating(&p, w, None).map(|r| r.name.as_str());
        assert_eq!(name(43.7), Some("22K"));
        assert_eq!(name(59.9), Some("22K"));
        assert_eq!(name(60.0), Some("24K"));
        assert_eq!(name(69.9), Some("24K"));
        assert_eq!(name(70.0), Some("26K"));
        assert_eq!(name(79.0), Some("26K"));
        assert_eq!(name(f64::NAN), None);
    }

    /// A lighter rating's roll is longer: one that no longer fits the
    /// runway goes to the next, and the full rating flies whatever fits.
    #[test]
    fn a_derate_the_runway_cannot_take_is_not_taken() {
        let p = zibo();
        let at = |needed: f64, avail: f64| {
            choose_rating(
                &p,
                45.0,
                Some(RunwayFit {
                    needed_full_m: needed,
                    available_m: avail,
                }),
            )
            .map(|r| r.name.clone())
        };
        // 1,500 m at 26K is 1,773 at 22K and 1,625 at 24K.
        assert_eq!(at(1500.0, 3000.0).as_deref(), Some("22K"));
        assert_eq!(at(1500.0, 1700.0).as_deref(), Some("24K"));
        assert_eq!(at(1500.0, 1600.0).as_deref(), Some("26K"));
        assert_eq!(at(1500.0, 1400.0).as_deref(), Some("26K"));
    }

    /// The LEMG load: 44 t, 15 C, 50 ft. 22K, 91.6; climb 91.1 at 1,050 ft.
    #[test]
    fn the_plan_and_its_line() {
        let p = zibo();
        let pl = plan(&p, 44.0, 15.0, 50.0, 1000.0, None).unwrap();
        assert_eq!(pl.rating, "22K");
        assert!((pl.to_n1 - 91.63).abs() < 0.01, "{pl:?}");
        assert!((pl.climb_n1 - 91.15).abs() < 0.01, "{pl:?}");
        assert_eq!(line(&pl), "Takeoff thrust 22K, N1 91.6.");
        // A cold day: the climb table's 85.9 at -20 C over the 22K's 86.0 is
        // held to the takeoff figure, never above it.
        let cold = plan(&p, 44.0, -20.0, 0.0, 1000.0, None).unwrap();
        assert!(cold.climb_n1 <= cold.to_n1, "{cold:?}");
    }

    /// The FMC's assumed temperature: 22K at sea level and 15 C takes the
    /// highest its table allows, 61, at 88.52 less the 2.66 correction for
    /// 46 degrees above the OAT. None past the tables' cover (their 99s),
    /// nor at a maximum the OAT or the flat rating already reaches.
    #[test]
    fn the_assumed_temperature_is_the_fmcs() {
        let p = zibo();
        let a = rating(&p, "22K").assumed.as_ref().unwrap();
        let (t, n1) = assumed_n1(a, 15.0, 0.0).unwrap();
        assert_eq!(t, 61.0);
        assert!((n1 - 85.86).abs() < 0.001, "{n1}");
        // Hot and high: the maximum table's 99.
        assert_eq!(assumed_n1(a, 50.0, 6000.0), None);
        // A maximum at or under the OAT, or under the flat-rated minimum:
        // nothing to assume.
        let mut low = a.clone();
        low.max_temp
            .values
            .iter_mut()
            .flatten()
            .for_each(|v| *v = 25.0);
        assert_eq!(assumed_n1(&low, 25.0, 0.0), None);
        assert_eq!(assumed_n1(&low, 15.0, 0.0), None);
    }

    /// Only a runway with room for the most an assumed temperature takes
    /// (the 22K's roll over 0.75) gets one; a runway not known gets none.
    #[test]
    fn an_assumed_temperature_needs_the_runway() {
        let p = zibo();
        let fit = |avail: f64| {
            Some(RunwayFit {
                needed_full_m: 1310.0,
                available_m: avail,
            })
        };
        let at = |r: Option<RunwayFit>| plan(&p, 44.0, 15.0, 0.0, 1000.0, r).unwrap();
        // 1,310 m at 26K: 1,548 at 22K, 2,064 with the room.
        let long = at(fit(3200.0));
        assert_eq!((long.rating.as_str(), long.assumed_c), ("22K", Some(61.0)));
        assert!((long.to_n1 - 85.86).abs() < 0.001, "{long:?}");
        assert_eq!(
            line(&long),
            "Takeoff thrust 22K, assumed temperature 61, N1 85.9."
        );
        // The climb N1 never above the takeoff's.
        assert!((long.climb_n1 - long.to_n1).abs() < 1e-9, "{long:?}");
        let mid = at(fit(2000.0));
        assert_eq!((mid.rating.as_str(), mid.assumed_c), ("22K", None));
        assert_eq!(at(None).assumed_c, None);
    }

    #[test]
    fn the_running_engines_are_averaged() {
        let mut a = vec![0.0; 16];
        (a[0], a[1]) = (92.0, 93.0);
        assert_eq!(mean_n1(&a), Some(92.5));
        a[1] = 0.0;
        assert_eq!(mean_n1(&a), Some(92.0));
        assert_eq!(mean_n1(&[0.0, 0.0]), None);
        assert_eq!(mean_n1(&[f64::NAN, 50.0]), Some(50.0));
    }

    /// The line: idle at 0, full at the highest lever, clamped.
    #[test]
    fn the_lever_leads_on_its_line() {
        let l = N1Loop::new(&zibo());
        assert!((l.feed_forward(104.0) - 1.0).abs() < 1e-9);
        assert_eq!(l.feed_forward(10.0), 0.0);
        assert_eq!(l.feed_forward(120.0), 1.0);
        let mid = l.feed_forward(91.6);
        assert!(mid > 0.8 && mid < 0.9, "{mid}");
    }

    /// Each decision: no read holds the lever on its line (aimed under the
    /// target until the N1 says), a lever held under the law by another
    /// is not the line's fault, a spooling engine is waited for, a settled
    /// shortfall trims the lever up and a settled excess trims it down, and
    /// an overshoot still rising trims at once.
    #[test]
    fn the_n1_loop_decides() {
        let p = zibo();
        let target = 91.6;
        let mut l = N1Loop::new(&p);
        // The line, first aimed a little under the target.
        let ff = l.step(target, None, 1.0, 0.25).0;
        assert!(
            ff < l.feed_forward(target) && ff > l.feed_forward(target) - 0.03,
            "{ff}"
        );
        assert_eq!(l.step(target, None, ff, 0.25), (ff, N1Step::Blind));
        // The ramp holds the lever at 0.5.
        assert_eq!(l.step(target, Some(60.0), 0.5, 0.25).1, N1Step::Limited);
        // At the cap: a first reading has no rate; then the N1 climbing
        // 4 %/s.
        let mut l = N1Loop::new(&p);
        assert_eq!(l.step(target, Some(80.0), ff, 0.25), (ff, N1Step::Spooling));
        let (_, s) = l.step(target, Some(82.0), ff, 0.25);
        assert_eq!(s, N1Step::Spooling);
        // Settled 2 % short, the lever at the cap: it comes up, a little.
        let settled = |n1: f64| {
            let mut l = N1Loop::new(&p);
            let mut lever = ff;
            let mut last = (ff, N1Step::Blind);
            for _ in 0..11 {
                last = l.step(target, Some(n1), lever, 0.25);
                lever = last.0;
            }
            last
        };
        let (cap, s) = settled(89.6);
        assert_eq!(s, N1Step::Trimmed);
        assert!(cap > ff && cap < ff + 0.1, "{cap} vs {ff}");
        // Settled 2 % over: down.
        let (cap, s) = settled(93.6);
        assert_eq!(s, N1Step::Trimmed);
        assert!(cap < ff, "{cap} vs {ff}");
        // Within the deadband: held on the line.
        assert_eq!(settled(91.5), (ff, N1Step::Held));
        // Over and still rising: trimmed now, spool or not.
        let mut l = N1Loop::new(&p);
        l.step(target, Some(91.4), ff, 0.25);
        let (cap, s) = l.step(target, Some(93.4), ff, 0.25);
        assert_eq!(s, N1Step::Trimmed);
        assert!(cap < ff);
    }

    /// Never above the highest lever, however short the N1 (a tired engine
    /// asks for more than the lever has).
    #[test]
    fn the_lever_never_passes_its_max() {
        let mut p = zibo();
        p.lever_max = 0.95;
        let mut l = N1Loop::new(&p);
        let mut cap = 0.0;
        for _ in 0..2000 {
            cap = l.step(103.0, Some(80.0), cap, 0.25).0;
        }
        assert!(cap <= 0.95 + 1e-12, "{cap}");
    }

    /// The engines as a plant: N1 follows the lever through a spool (two
    /// lags, the fuel control's and the rotor's, slower from idle) after a
    /// write's transport delay, on a lever-to-N1 map that is NOT the
    /// declared line. `gain` tilts it, `offset` shifts it.
    struct Engines {
        n1: f64,
        fuel: f64,
        gain: f64,
        offset: f64,
        sent: std::collections::VecDeque<f64>,
    }

    impl Engines {
        /// A quarter second of transport at the 0.25 s tick.
        const DELAY_TICKS: usize = 1;
        fn new(n1: f64, gain: f64, offset: f64) -> Self {
            Engines {
                n1,
                fuel: n1,
                gain,
                offset,
                sent: std::collections::VecDeque::new(),
            }
        }
        fn settled(&self, lever: f64) -> f64 {
            (20.0 + 84.0 * self.gain * lever + self.offset).clamp(20.0, 106.0)
        }
        fn advance(&mut self, lever: f64, dt: f64) {
            self.sent.push_back(lever);
            let applied = if self.sent.len() > Self::DELAY_TICKS {
                self.sent.pop_front().unwrap()
            } else {
                *self.sent.front().unwrap()
            };
            let want = self.settled(applied);
            let tau = if self.n1 < 60.0 { 3.5 } else { 2.2 };
            let steps = 25;
            let h = dt / steps as f64;
            for _ in 0..steps {
                self.fuel += (want - self.fuel) * h / 0.4;
                self.n1 += (self.fuel - self.n1) * h / tau;
            }
        }
    }

    /// The roll as the loop flies it: the ramp's lever (0.3 to full over
    /// about 4 s, slewed), capped by the N1 law, against engines whose map
    /// is off the line by up to 4 % of N1 either way. The N1 is set within
    /// half a percent by 20 s (on this plant 16 to 20 s; the worst
    /// overshoot 1.1, the line 4 % high), never more than 1.5 % over the
    /// target, and at the reduction it comes back to the climb N1 within
    /// half a percent in 20 s, the lever never past its max.
    #[test]
    fn the_roll_sets_takeoff_n1_and_the_reduction_climb_n1() {
        let p = zibo();
        for (gain, offset) in [
            (1.0, 0.0),
            (1.0, 4.0),
            (1.0, -4.0),
            (0.95, 2.0),
            (1.05, -2.0),
        ] {
            let mut e = Engines::new(22.0, gain, offset);
            let mut l = N1Loop::new(&p);
            let (to, clb) = (91.6, 91.1);
            let mut lever = 0.3;
            let dt = 0.25;
            let mut max_over = f64::MIN;
            let mut t = 0.0;
            while t < 60.0 {
                let target = if t < 30.0 { to } else { clb };
                let ramp = (0.3_f64 + 0.2 * t).min(1.0);
                let (cap, _) = l.step(target, Some(e.n1), lever, dt);
                let want = ramp.min(cap);
                // The takeoff's slew up, the reduction's 0.1 a second down.
                lever = if want > lever {
                    (lever + 0.25 * dt).min(want)
                } else {
                    (lever - 0.1 * dt).max(want)
                };
                assert!(lever <= p.lever_max + 1e-12);
                e.advance(lever, dt);
                t += dt;
                if t < 30.0 {
                    max_over = max_over.max(e.n1 - to);
                }
                if (20.0..30.0).contains(&t) {
                    assert!(
                        (e.n1 - to).abs() < 0.5,
                        "gain {gain} offset {offset}: {:.2} at {t}",
                        e.n1
                    );
                }
                if t >= 50.0 {
                    assert!(
                        (e.n1 - clb).abs() < 0.5,
                        "gain {gain} offset {offset}: {:.2} at {t}",
                        e.n1
                    );
                }
            }
            assert!(
                max_over < 1.5,
                "gain {gain} offset {offset}: over by {max_over:.2}"
            );
        }
    }
}
