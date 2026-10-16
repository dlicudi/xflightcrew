//! The centreline as a bank: PX4's NPFG in its linear form (Stastny and
//! Siegwart), the L1 family's track keeper.
//!
//! The track is what moves the aircraft over the paint, so the track is
//! what this closes. The guidance is ONE second-order loop on the offset:
//! a lateral acceleration of `w^2 x + 2 zeta w v` (x the offset, v the
//! sideways velocity over the ground), flown as a bank of `atan(a / g)`,
//! with `w = 2 pi / period`. Its two numbers are NPFG's: a damping
//! ([`DAMPING`], NPFG_DAMPING 0.7) and a period bounded BELOW by the bank's
//! own measured answer, `4 pi zeta tc` times a safety factor
//! ([`PERIOD_SF`]; NPFG_ROLL_TC is the time constant the roll loop closes
//! in). A guidance faster than its inner loop rings: the period is the
//! inner loop's, never typed. The sideways velocity is judged where the bank
//! will have taken it one `tc` from now (the inner loop's lag, seen by the
//! outer): the bank the inner loop is being ASKED for, as that loop answers
//! it ([`bank_ahead`]), never the bank read, which is the inner loop's own
//! state. It replaced a closure over 4 s, a velocity loop of
//! 1 s and a bank lead of 1 s (2026-10-12), all hand picked.
//!
//! The safety factor is 1, not NPFG's default 1.5: our `tc` is the closed
//! bank loop's with its delay and the tick in it
//! (`response::attitude_tc_s`), where NPFG_ROLL_TC is the bare roll mode's.
//! At 1.5 the A330's slow sidestick (0.35 to 0.6 s late) had a 30 s period
//! and could not close its recorded gates in the 17 s they had (#180, the
//! plants).
//!
//! The crab into a crosswind comes out of it by itself: holding the track
//! on the course turns the heading into the wind. The closure toward the
//! line is never faster than [`CLOSE_MAX_M_S`] sideways (NPFG's track
//! error bound does the same: far off, a bearing in, not a dive).

/// NPFG's damping.
pub const DAMPING: f64 = 0.7;
/// The period's safety factor over the inner loop's bound (see above).
pub const PERIOD_SF: f64 = 1.0;
/// The closure toward the line, at most, m/s sideways.
const CLOSE_MAX_M_S: f64 = 2.5;
/// The steepest bank, degrees, tapering by [`BANK_PER_FT`] to
/// [`BANK_WHEELS_DEG`] at the wheels: 5 degrees from 10 ft up.
pub const BANK_MAX_DEG: f64 = 5.0;
pub const BANK_WHEELS_DEG: f64 = 3.0;
const BANK_PER_FT: f64 = 0.2;
const G_M_S2: f64 = 9.81;

/// The track keeper's period for a bank loop closing in `roll_tc_s`
/// seconds: NPFG's lower bound times its safety factor. Pure.
pub fn period_s(roll_tc_s: f64) -> f64 {
    PERIOD_SF * 4.0 * std::f64::consts::PI * DAMPING * roll_tc_s.clamp(0.2, 5.0)
}

/// The bank loop's time constant the period was bounded by, s.
fn bank_tc_s(period_s: f64) -> f64 {
    period_s / (PERIOD_SF * 4.0 * std::f64::consts::PI * DAMPING)
}

/// The bank that closes the centreline: the offset (`xtk_m`, right
/// positive), the sideways velocity over the ground (`side_m_s`, right
/// positive), the bank the bank loop is taking the aircraft to
/// (`ahead_deg`, [`bank_ahead`]; the bank read on the first tick), the
/// height, the track keeper's period, and the sideways acceleration the
/// sideslip already gives (`slip_m_s2`, [`slip_accel_m_s2`]: in the decrab
/// the wing goes down into the wind by that much). No offset read is no
/// bank. Pure.
pub fn bank_for_line(xtk_m: f64, side_m_s: f64, ahead_deg: f64, radio_ft: f64, period_s: f64, slip_m_s2: f64) -> f64 {
    if !xtk_m.is_finite() || !side_m_s.is_finite() {
        return 0.0;
    }
    let slip = if slip_m_s2.is_finite() { slip_m_s2 } else { 0.0 };
    // The sideways velocity the bank will have made by the time the bank
    // loop answers (its time constant, the period's bound): a 5 degree bank
    // inherited at the A330's floated gate carried it 5 m past the line
    // before a slow sidestick could take it out. The bank the loop is
    // taking the aircraft to, never the bank read: the read is the inner
    // loop's own state, and subtracted here it doubled that loop's gain
    // (the 737 rang at the tick below minimums, EGPH 24, 1792158539).
    let phi = if ahead_deg.is_finite() { ahead_deg.clamp(-30.0, 30.0) } else { 0.0 };
    let side_m_s = side_m_s + (G_M_S2 * phi.to_radians().sin() + slip) * bank_tc_s(period_s);
    let cap = if radio_ft.is_finite() {
        (BANK_WHEELS_DEG + radio_ft.max(0.0) * BANK_PER_FT).min(BANK_MAX_DEG)
    } else {
        BANK_WHEELS_DEG
    };
    let w = 2.0 * std::f64::consts::PI / period_s.max(1.0);
    // a = -(w^2 x + 2 zeta w v) = -2 zeta w (v - v_want), the closure
    // wanted -w x / (2 zeta), bounded by the speed and by what half the
    // capped bank can stop before the line (NPFG's track error bound).
    let stop = 0.5 * G_M_S2 * cap.to_radians().tan();
    let off = xtk_m.abs();
    let want = -xtk_m.signum() * (w / (2.0 * DAMPING) * off).min((2.0 * stop * off).sqrt()).min(CLOSE_MAX_M_S);
    let accel = 2.0 * DAMPING * w * (want - side_m_s);
    // The correction within the cap; the wing the sideslip needs down on
    // top of it (the slip's side force is cancelled, not corrected), within
    // a wing-low touchdown's limit.
    let correct = (accel / G_M_S2).atan().to_degrees().clamp(-cap, cap);
    let hold = (-slip / G_M_S2).clamp(-1.0, 1.0).asin().to_degrees();
    (correct + hold).clamp(-SLIP_BANK_MAX_DEG, SLIP_BANK_MAX_DEG)
}

/// Where the bank loop has taken the bank `dt` seconds on from `from_deg`
/// toward `target_deg`: the closed loop's first-order answer over its time
/// constant (the period's bound). Fed back its own output tick by tick,
/// from the bank read on the first, it is the bank [`bank_for_line`] leads
/// on. Pure.
pub fn bank_ahead(from_deg: f64, target_deg: f64, dt: f64, period_s: f64) -> f64 {
    from_deg + (target_deg - from_deg) * (1.0 - (-dt.max(0.0) / bank_tc_s(period_s).max(0.05)).exp())
}

/// The steepest bank with the wing down into a crosswind, degrees: a wing
/// low touchdown, short of a light aircraft's or a 737's wingtip and pod
/// (about 10 degrees on extended struts).
pub const SLIP_BANK_MAX_DEG: f64 = 8.0;

/// The sideways acceleration a sideslip gives the airpath, m/s^2 right
/// positive: X-Plane's beta (the nose right of the airpath) times the side
/// force's answer (the yaw response's `path`, 1/s) and the true airspeed.
/// Pure.
pub fn slip_accel_m_s2(beta_deg: f64, path_per_s: f64, tas_kts: f64) -> f64 {
    tas_kts * 0.514_444 * path_per_s * beta_deg.to_radians()
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
        let p = period_s(1.1);
        assert!((9.0..11.0).contains(&p), "a 1.1 s bank loop: {p}");
        assert_eq!(bank_for_line(-30.0, 0.0, 0.0, 80.0, p, 0.0), BANK_MAX_DEG);
        assert_eq!(bank_for_line(-30.0, 0.0, 0.0, 0.0, p, 0.0), BANK_WHEELS_DEG);
        assert!(bank_for_line(30.0, 0.0, 0.0, 80.0, p, 0.0) < 0.0, "right of the line banks left");
        assert_eq!(bank_for_line(f64::NAN, 0.0, 0.0, 50.0, p, 0.0), 0.0);
        assert_eq!(bank_for_line(0.0, 0.0, 0.0, 50.0, p, 0.0), 0.0);
        // Crossing left at 2.3 m/s over the centreline: bank right now.
        assert!(bank_for_line(0.0, -2.3, 0.0, 50.0, p, 0.0) > 3.0);
        // A slower bank loop asks a longer period, and a gentler bank.
        assert!(bank_for_line(5.0, 0.0, 0.0, 80.0, period_s(2.0), 0.0).abs() < bank_for_line(5.0, 0.0, 0.0, 80.0, p, 0.0).abs());
        assert!(side_velocity_m_s(None, 130.0, 140.0).is_none());
        let v = side_velocity_m_s(Some(129.0), 130.0, 140.0).unwrap();
        assert!((v + 1.26).abs() < 0.05, "1 degree left at 140 kt: {v}");
    }
}
