//! The centreline as a bank: a velocity loop inside a position loop.
//!
//! The track is what moves the aircraft over the paint, so the track is
//! what this closes: the offset asks for a sideways closure rate, and the
//! bank takes out the difference between that and the sideways velocity
//! over the ground. It flew a HEADING first (the rollout's centreline law
//! plus the crab), and the 737's nose did not follow its bank: LEMG 13
//! (2026-09-26, 17:43Z) went into the gate 3.9 m right crossing left at
//! 2.3 m/s and touched down 15.3 m left. The closure goes to ZERO at the
//! centreline, sized to be stopped by a gentle deceleration
//! ([`STOP_M_S2`]), and the drift is judged on the one the CURRENT bank
//! will have made [`BANK_LEAD_S`] from now: LEMG 13 (2026-09-26, 20:17Z)
//! went into the gate 11.9 m right, built 2.65 m/s of drift and touched
//! down 6.5 m left.
//!
//! The crab into a crosswind comes out of it by itself: holding the track
//! on the course turns the heading into the wind.

/// The closure the offset asks for: the offset over this many seconds.
const CLOSE_S: f64 = 4.0;
/// ...never faster than this sideways, m/s.
const CLOSE_MAX_M_S: f64 = 2.5;
/// The sideways deceleration the closure is sized to be stopped by, m/s^2:
/// well inside the 0.5 a 3 degree bank gives, since the bank takes seconds
/// to get there.
const STOP_M_S2: f64 = 0.2;
/// How quickly a sideways-velocity error is taken out by the bank, s. It
/// was 1.5 for the arrest's 50 ft; from minimums, on the six fitted roll
/// plants (`plants_tests`), 1.0 holds a gusty final 0.5 m nearer.
const VEL_TAU_S: f64 = 1.0;
/// How far ahead the current bank's push on the drift is counted, s: the
/// 737's roll lag at the arrest's aileron (LEMG 13, 2026-09-26).
const BANK_LEAD_S: f64 = 1.0;
/// The steepest bank, degrees, tapering by [`BANK_PER_FT`] to
/// [`BANK_WHEELS_DEG`] at the wheels: 5 degrees from 15 ft up.
pub const BANK_MAX_DEG: f64 = 5.0;
pub const BANK_WHEELS_DEG: f64 = 3.0;
const BANK_PER_FT: f64 = 0.2;
const G_M_S2: f64 = 9.81;

/// The bank that closes the centreline: the offset (`xtk_m`, right
/// positive), the sideways velocity over the ground (`side_m_s`, right
/// positive), the bank read and the height. No offset read is no bank.
/// Pure.
pub fn bank_for_line(xtk_m: f64, side_m_s: f64, phi_deg: f64, radio_ft: f64) -> f64 {
    if !xtk_m.is_finite() || !side_m_s.is_finite() {
        return 0.0;
    }
    let phi = if phi_deg.is_finite() {
        phi_deg.clamp(-30.0, 30.0)
    } else {
        0.0
    };
    let off = xtk_m.abs();
    let rate = (off / CLOSE_S)
        .min((2.0 * STOP_M_S2 * off).sqrt())
        .min(CLOSE_MAX_M_S);
    let want = -xtk_m.signum() * rate;
    let side_ahead = side_m_s + G_M_S2 * phi.to_radians().tan() * BANK_LEAD_S;
    let bank = ((want - side_ahead) / (G_M_S2 * VEL_TAU_S))
        .atan()
        .to_degrees();
    let cap = if radio_ft.is_finite() {
        (BANK_WHEELS_DEG + radio_ft.max(0.0) * BANK_PER_FT).min(BANK_MAX_DEG)
    } else {
        BANK_WHEELS_DEG
    };
    bank.clamp(-cap, cap)
}

/// The sideways velocity over the line, m/s right positive, from the ground
/// track (true) against the course (true). `None` without a track read.
/// Pure.
pub fn side_velocity_m_s(track_true: Option<f64>, course_true: f64, gs_kts: f64) -> Option<f64> {
    let trk = track_true.filter(|t| t.is_finite())?;
    if !gs_kts.is_finite() {
        return None;
    }
    let off = crate::angles::signed_diff_deg(trk, course_true).to_radians();
    Some(gs_kts * 0.514_444 * off.sin())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_bank_tapers_to_the_wheels_and_needs_a_reading() {
        assert_eq!(bank_for_line(-30.0, 0.0, 0.0, 80.0), BANK_MAX_DEG);
        assert_eq!(bank_for_line(-30.0, 0.0, 0.0, 0.0), BANK_WHEELS_DEG);
        assert!(
            bank_for_line(30.0, 0.0, 0.0, 80.0) < 0.0,
            "right of the line banks left"
        );
        assert_eq!(bank_for_line(f64::NAN, 0.0, 0.0, 50.0), 0.0);
        assert_eq!(bank_for_line(0.0, 0.0, 0.0, 50.0), 0.0);
        // Crossing left at 2.3 m/s over the centreline: bank right now.
        assert!(bank_for_line(0.0, -2.3, 0.0, 50.0) > 3.0);
        assert!(side_velocity_m_s(None, 130.0, 140.0).is_none());
        let v = side_velocity_m_s(Some(129.0), 130.0, 140.0).unwrap();
        assert!((v + 1.26).abs() < 0.05, "1 degree left at 140 kt: {v}");
    }
}
