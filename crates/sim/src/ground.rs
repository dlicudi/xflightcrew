//! "On the ground" and "stopped on the ground": one answer each.
//!
//! Refactor audit 2026-10-04, duplicates #9: the on-ground dataref was
//! redeclared in fifteen files and "stopped" meant under 0.5 m/s in the load
//! sheet and the engine shutdown, under 1.0 m/s (2 kt) in the flight setup,
//! under 1 kt in the hand-back, the flight model and the sortie runner, so
//! the setup tool would end a flight the load sheet refused to load. A
//! tolerance that means something else (a taxi governor's "arrived", a
//! crossing that has stopped moving, a helicopter settling on its wheels)
//! keeps its own name where it is used.

use crate::datarefs::{GROUNDSPEED_MS, ON_GROUND_ANY};
use crate::xplane::XPlane;

/// Under this groundspeed the aircraft is stopped, kt.
pub const STOPPED_KTS: f64 = 1.0;
/// Metres per second to knots.
pub const MS_TO_KTS: f64 = 1.943_844;
/// The ground check's second instrument: on the ground AND no higher than
/// this above it, ft (`onground_any` alone reads a wheel brushing a hump).
pub const CONFIRMED_GROUND_AGL_FT: f64 = 50.0;
const M_TO_FT: f64 = 3.280_84;

/// `onground_any` read as a fact: any wheel on the ground.
pub fn on_ground(raw: f64) -> bool {
    raw > 0.5
}

/// Stopped: a groundspeed magnitude under [`STOPPED_KTS`]. NaN is not stopped.
pub fn stopped_kts(gs_kts: f64) -> bool {
    gs_kts.abs() < STOPPED_KTS
}

/// [`stopped_kts`] from the sim's metres per second.
pub fn stopped_mps(gs_mps: f64) -> bool {
    stopped_kts(gs_mps * MS_TO_KTS)
}

/// Stopped on the ground, from the two reads (`onground_any`, groundspeed in
/// m/s). A failed read is no measurement, so it is not stopped.
pub fn stopped_on_ground(on_ground_raw: Option<f64>, gs_mps: Option<f64>) -> bool {
    on_ground_raw.is_some_and(on_ground) && gs_mps.is_some_and(stopped_mps)
}

/// Both instruments affirm the ground: `onground_any` set AND the radio
/// height under [`CONFIRMED_GROUND_AGL_FT`]. A failed or non-finite read is
/// no clearance.
pub fn on_ground_confirmed(agl_m: Option<f64>, on_ground_raw: Option<f64>) -> bool {
    match (agl_m, on_ground_raw) {
        (Some(agl), Some(g)) if agl.is_finite() && g.is_finite() => {
            agl * M_TO_FT <= CONFIRMED_GROUND_AGL_FT && on_ground(g)
        }
        _ => false,
    }
}

/// [`stopped_on_ground`] read from the sim, both in one round.
pub async fn read_stopped_on_ground(xp: &dyn XPlane) -> bool {
    let (g, gs) = tokio::join!(xp.get_float(ON_GROUND_ANY), xp.get_float(GROUNDSPEED_MS));
    stopped_on_ground(g.ok(), gs.ok())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stopped_is_under_a_knot_either_way() {
        assert!(stopped_kts(0.0));
        assert!(stopped_kts(-0.9));
        assert!(!stopped_kts(1.0));
        assert!(!stopped_kts(f64::NAN));
        // 0.5 m/s is 0.97 kt: stopped; 1.0 m/s is 1.94 kt: rolling (the
        // flight setup used to call that stopped).
        assert!(stopped_mps(0.5));
        assert!(!stopped_mps(1.0));
    }

    #[test]
    fn stopped_on_ground_needs_both_reads() {
        assert!(stopped_on_ground(Some(1.0), Some(0.1)));
        assert!(!stopped_on_ground(Some(0.0), Some(0.1)), "airborne");
        assert!(!stopped_on_ground(Some(1.0), Some(2.0)), "rolling");
        assert!(!stopped_on_ground(None, Some(0.0)), "unread ground");
        assert!(!stopped_on_ground(Some(1.0), None), "unread speed");
    }

    #[test]
    fn confirmed_ground_needs_both_instruments() {
        assert!(on_ground_confirmed(Some(0.0), Some(1.0)));
        assert!(!on_ground_confirmed(Some(500.0), Some(0.0)));
        assert!(!on_ground_confirmed(Some(0.0), Some(0.0)), "just lifted off");
        assert!(!on_ground_confirmed(Some(500.0), Some(1.0)), "conflicting");
        assert!(!on_ground_confirmed(None, Some(1.0)));
        assert!(!on_ground_confirmed(Some(f64::NAN), Some(1.0)));
    }

    #[tokio::test]
    async fn the_reader_reads_both_in_one_round() {
        let xp = crate::xplane::FakeXPlane::new();
        xp.set_read(ON_GROUND_ANY, 1.0);
        xp.set_read(GROUNDSPEED_MS, 0.2);
        assert!(read_stopped_on_ground(&xp).await);
        xp.set_read(GROUNDSPEED_MS, 3.0);
        assert!(!read_stopped_on_ground(&xp).await);
    }
}
