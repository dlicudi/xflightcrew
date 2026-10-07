//! The takeoff stabilizer trim, set as a crew sets it: the figure read off
//! the type's table for this weight and centre of gravity, then the trim
//! switch held until the stabilizer gauge reads it (`takeoff_trim:`,
//! [`crate::aircraft::TakeoffTrimProfile`]).
//!
//! The generic takeoff trim (`tools::controls::trim_for_takeoff`, a neutral
//! 0.0 on the stock ratio) is 8.35 units on the Zibo 737, near the nose-up
//! stop of 8.5. At 46 t with the CG at 19 % MAC its table says 4.4. With
//! twice the nose-up trim and full thrust the aircraft rotated itself from
//! 112 kt: at LEMG it lifted off at 118 kt against a Vr of 145, and a reject
//! that then fired in the air crashed it (2026-10-07, #231).
//!
//! An airframe that declares nothing keeps the generic path.

use std::time::Duration;

use serde_json::json;

use crate::aircraft::TakeoffTrimProfile;
use crate::xplane::XPlane;

/// The stabilizer is set when its gauge reads within this of the figure, units.
pub const TOLERANCE_UNITS: f64 = 0.1;
/// Holds of the switch before the setting is given up and named.
const MAX_HOLDS: usize = 12;
/// The longest single hold, seconds: a crew blips the switch, it does not
/// lean on it.
const MAX_HOLD_S: f64 = 3.0;
/// The shortest hold that moves the stabilizer, seconds.
const MIN_HOLD_S: f64 = 0.2;
/// The trim rate assumed for the first hold, units a second (measured on the
/// Zibo's battery: 0.175 units in 2 s). Later holds use the rate measured.
const FIRST_RATE_UNITS_S: f64 = 0.09;

/// Linear interpolation of `y` at `x` between `(x0, y0)` and `(x1, y1)`.
pub(crate) fn lerp(x0: f64, y0: f64, x1: f64, y1: f64, x: f64) -> f64 {
    if (x1 - x0).abs() < f64::EPSILON {
        y0
    } else {
        y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    }
}

/// The bracketing indices of `x` in an ascending axis, clamped at its ends.
pub(crate) fn bracket(axis: &[f64], x: f64) -> (usize, usize) {
    if x <= axis[0] {
        return (0, 0);
    }
    for i in 1..axis.len() {
        if x <= axis[i] {
            return (i - 1, i);
        }
    }
    let last = axis.len() - 1;
    (last, last)
}

/// The table's takeoff trim for this CG (% MAC) and weight (tonnes):
/// bilinear between the rows and columns that bracket them, clamped at the
/// table's edges and to its trim range. `None` for a malformed table or a
/// reading that is not a number. Pure.
pub fn table_units(t: &TakeoffTrimProfile, cg_mac: f64, weight_t: f64) -> Option<f64> {
    if !cg_mac.is_finite() || !weight_t.is_finite() || t.cg.is_empty() || t.weights_t.is_empty() {
        return None;
    }
    if t.table.len() != t.cg.len() || t.table.iter().any(|r| r.len() != t.weights_t.len()) {
        return None;
    }
    let (c0, c1) = bracket(&t.cg, cg_mac);
    let (w0, w1) = bracket(&t.weights_t, weight_t);
    let at_cg = |c: usize| {
        lerp(
            t.weights_t[w0],
            t.table[c][w0],
            t.weights_t[w1],
            t.table[c][w1],
            weight_t,
        )
    };
    let units = lerp(t.cg[c0], at_cg(c0), t.cg[c1], at_cg(c1), cg_mac);
    Some(units.clamp(t.min_units, t.max_units))
}

/// The next hold: which switch, and for how long, to close `err` units
/// (want minus now) at `rate` units a second. Pure.
pub fn next_hold(t: &TakeoffTrimProfile, err: f64, rate: f64) -> (&str, f64) {
    let secs = (err.abs() / rate.max(0.01)).clamp(MIN_HOLD_S, MAX_HOLD_S);
    if err > 0.0 {
        (t.nose_up.as_str(), secs)
    } else {
        (t.nose_down.as_str(), secs)
    }
}

/// What the setting came to.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Setting {
    /// Within tolerance of the figure (`now` units).
    Set { want: f64, now: f64 },
    /// The switch was held and the gauge never reached it.
    NotTaken { want: f64, now: f64 },
}

/// Set the takeoff trim from the table. `None` when the weight, the CG or
/// the gauge did not read: no measurement, nothing pressed.
pub async fn set(xp: &dyn XPlane, t: &TakeoffTrimProfile) -> Option<Setting> {
    let (cg, kg, units) = tokio::join!(
        xp.get_float(&t.cg_mac),
        xp.get_float(sim::datarefs::TOTAL_WEIGHT_KG),
        xp.get_float(&t.units),
    );
    let (cg, kg, mut now) = (cg.ok()?, kg.ok()?, units.ok()?);
    if !now.is_finite() {
        return None;
    }
    let want = table_units(t, cg, kg / 1000.0)?;
    let was = now;
    let mut rate = FIRST_RATE_UNITS_S;
    let mut holds = 0;
    while (want - now).abs() > TOLERANCE_UNITS && holds < MAX_HOLDS {
        let (cmd, secs) = next_hold(t, want - now, rate);
        if xp.run_command_for(cmd, secs).await.is_err() {
            break;
        }
        // The hold runs in the sim; read once it has ended.
        tokio::time::sleep(Duration::from_secs_f64(secs + 0.3)).await;
        let Some(after) = xp.get_float(&t.units).await.ok().filter(|v| v.is_finite()) else {
            break;
        };
        let moved = (after - now).abs();
        if moved > 0.01 {
            rate = moved / secs;
        }
        now = after;
        holds += 1;
    }
    let ok = (want - now).abs() <= TOLERANCE_UNITS;
    crate::blackbox::record(
        "takeoff_trim",
        json!({"cg_mac": cg, "weight_kg": kg, "want": want, "was": was, "now": now,
               "holds": holds, "ok": ok}),
    );
    Some(if ok {
        Setting::Set { want, now }
    } else {
        Setting::NotTaken { want, now }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The 737-800's FCOM table, flaps 1 and 5, 26K: the profile's.
    fn b738() -> TakeoffTrimProfile {
        let yaml = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/profiles/zibo_738.yaml"
        ))
        .unwrap();
        let p: crate::aircraft::AircraftProfile = serde_yaml::from_str(&yaml).unwrap();
        p.takeoff_trim.expect("the Zibo declares its takeoff trim")
    }

    /// The LEMG load: 45.7 t, CG 18.75 % MAC. Between the 16 and 21 rows at
    /// 45 t (4.75 and 4.0) and a little toward 50 t: about 4.3, where the
    /// generic 0.0 put 8.35.
    #[test]
    fn the_lemg_load_reads_about_four_point_three() {
        let u = table_units(&b738(), 18.75, 45.75).unwrap();
        assert!((u - 4.40).abs() < 0.01, "{u}");
    }

    /// A derated takeoff rotates on its own table (`by_rating`): the LEMG
    /// load at 24K and 22K; a rating with none, or none chosen, the base.
    #[test]
    fn a_derate_reads_its_own_table() {
        let t = b738();
        let at = |r: Option<&str>| table_units(&t.for_rating(r), 18.75, 45.75).unwrap();
        assert!((at(Some("24K")) - 4.779).abs() < 0.001, "{}", at(Some("24K")));
        assert!((at(Some("22K")) - 5.125).abs() < 0.001, "{}", at(Some("22K")));
        assert!((at(Some("26K")) - 4.40).abs() < 0.01);
        assert!((at(None) - 4.40).abs() < 0.01);
    }

    #[test]
    fn table_corners_and_clamps() {
        let t = b738();
        assert_eq!(table_units(&t, 6.0, 85.0), Some(8.5));
        assert_eq!(table_units(&t, 36.0, 40.0), Some(2.25));
        // Off the table: the edge row and column, never extrapolated.
        assert_eq!(table_units(&t, 2.0, 30.0), table_units(&t, 6.0, 40.0));
        assert_eq!(table_units(&t, 40.0, 95.0), table_units(&t, 36.0, 85.0));
        assert_eq!(table_units(&t, f64::NAN, 50.0), None);
    }

    #[test]
    fn a_hold_is_toward_the_figure_and_bounded() {
        let t = b738();
        let (cmd, s) = next_hold(&t, -4.0, 0.09);
        assert_eq!(cmd, t.nose_down);
        assert_eq!(s, MAX_HOLD_S);
        let (cmd, s) = next_hold(&t, 0.05, 0.5);
        assert_eq!(cmd, t.nose_up);
        assert_eq!(s, MIN_HOLD_S);
    }

    /// Holds until the gauge reads the figure, and says what it came to.
    #[tokio::test(start_paused = true)]
    async fn the_switch_is_held_until_the_gauge_reads_the_figure() {
        let t = b738();
        let xp = crate::xplane::FakeXPlane::new();
        xp.set_read(&t.cg_mac, 18.75);
        xp.set_read(sim::datarefs::TOTAL_WEIGHT_KG, 45_750.0);
        xp.set_read(&t.units, 8.35);
        // Each nose-down hold takes the gauge down a unit and a half.
        xp.set_on_command(&t.nose_down, &t.units, 6.85);
        let s = set(&xp, &t).await.unwrap();
        assert!(
            matches!(s, Setting::NotTaken { .. }),
            "a fake that sticks at 6.85 is named: {s:?}"
        );
        assert!(xp.ran(&t.nose_down));
        assert!(!xp.ran(&t.nose_up));
    }
}
