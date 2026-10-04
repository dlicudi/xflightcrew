//! Angle wrap and heading difference: the one home (refactor audit
//! 2026-10-04, `docs/tasks/refactor-audit/audit-duplicates.md` #14).
//!
//! Twenty-odd private copies of `(a - b + 540) rem 360 - 180` lived across
//! the crew, the world and the replays, and the same NAME meant different
//! things: `autoland::heading_diff` was the ABSOLUTE difference while
//! `reversal::heading_diff` and `course_leg::heading_diff` were SIGNED, and
//! `answers::angle_delta_deg(a, b)` was `b - a`, the opposite sign to every
//! other `(a, b)` copy. Every caller now says which one it means:
//!
//! - [`signed_diff_deg`]`(a, b)`: `a - b`, wrapped to -180..180. Positive
//!   when `a` is CLOCKWISE of `b`, i.e. the turn FROM `b` TO `a` is to the
//!   right. The convention of almost every copy it replaced, kept so no
//!   caller had to swap its arguments.
//! - [`turn_deg`]`(from, to)`: the same number named by direction, the turn
//!   from heading `from` onto heading `to` (positive = right).
//! - [`abs_diff_deg`]: the size of that turn, 0..180.
//! - [`wrap180`] / [`wrap360`]: one angle into -180..180 / 0..360.
//!
//! All use `rem_euclid`, correct for any finite input; the old `%` copies
//! agreed with it only for differences above -540 degrees (every heading
//! pair in 0..360 is). Pure.

/// Wrap any angle into -180..180 degrees (exactly 180 maps to -180).
pub fn wrap180(deg: f64) -> f64 {
    (deg + 540.0).rem_euclid(360.0) - 180.0
}

/// Wrap any angle into 0..360 degrees.
pub fn wrap360(deg: f64) -> f64 {
    deg.rem_euclid(360.0)
}

/// `a - b` wrapped to -180..180: positive when `a` is clockwise of `b`.
pub fn signed_diff_deg(a: f64, b: f64) -> f64 {
    wrap180(a - b)
}

/// The turn from heading `from` onto heading `to`, -180..180 (positive =
/// right). `turn_deg(from, to) == signed_diff_deg(to, from)`.
pub fn turn_deg(from: f64, to: f64) -> f64 {
    signed_diff_deg(to, from)
}

/// The smallest absolute difference between two angles, 0..180.
pub fn abs_diff_deg(a: f64, b: f64) -> f64 {
    signed_diff_deg(a, b).abs()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signed_diff_is_a_minus_b_positive_clockwise() {
        // 10 is clockwise of 350: from 350 to 10 is a right turn of 20.
        assert_eq!(signed_diff_deg(10.0, 350.0), 20.0);
        assert_eq!(signed_diff_deg(350.0, 10.0), -20.0);
        assert_eq!(signed_diff_deg(90.0, 0.0), 90.0);
        assert_eq!(signed_diff_deg(0.0, 90.0), -90.0);
        assert_eq!(signed_diff_deg(271.0, 2.0), -91.0);
    }

    #[test]
    fn turn_names_the_direction() {
        assert_eq!(turn_deg(350.0, 10.0), 20.0, "350 onto 10 is right");
        assert_eq!(turn_deg(10.0, 350.0), -20.0, "10 onto 350 is left");
        assert_eq!(turn_deg(2.0, 271.0), -91.0);
    }

    #[test]
    fn abs_diff_is_the_size_of_the_turn() {
        assert_eq!(abs_diff_deg(10.0, 350.0), 20.0);
        assert_eq!(abs_diff_deg(350.0, 10.0), 20.0);
        assert_eq!(abs_diff_deg(0.0, 180.0), 180.0);
        assert_eq!(abs_diff_deg(123.0, 123.0), 0.0);
    }

    #[test]
    fn wraps_hold_for_any_input() {
        assert_eq!(wrap180(180.0), -180.0);
        assert_eq!(wrap180(-180.0), -180.0);
        assert_eq!(wrap180(190.0), -170.0);
        // The old `((d + 540) % 360) - 180` read -280 here.
        assert_eq!(wrap180(-1000.0), 80.0);
        assert_eq!(wrap180(720.0 + 45.0), 45.0);
        assert_eq!(wrap360(-10.0), 350.0);
        assert_eq!(wrap360(370.0), 10.0);
        assert_eq!(wrap360(0.0), 0.0);
    }

    #[test]
    fn the_old_percent_copies_agree_in_their_domain() {
        let old = |a: f64, b: f64| ((a - b) % 360.0 + 540.0) % 360.0 - 180.0;
        for a in (0..360).step_by(7) {
            for b in (0..360).step_by(11) {
                let (a, b) = (a as f64, b as f64);
                assert!((old(a, b) - signed_diff_deg(a, b)).abs() < 1e-9, "{a} {b}");
            }
        }
    }
}
