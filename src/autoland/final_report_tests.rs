//! The crew's "final" report waits for a final (`final_report_due`).
//! LEGR-LXGB, 2026-10-06 (sortie 1791275171-full_auto): "final runway zero
//! nine" went out as autoland took the arrival, 12 nm EAST of the field at
//! 13,300 ft with a reposition round the Rock still to fly, and the tower
//! cleared the landing on it.

use super::*;

#[test]
fn final_is_reported_established_on_it_never_at_the_start_of_a_reposition() {
    // The sortie's tick: Intercept, a pattern about to begin, flying 281
    // away from runway 09's 090.
    assert!(!final_report_due(false, false, false, 281.0, 90.0));
    // The guidance calls it Final but the pattern still flies (or a
    // pattern's plan is in hand): not yet.
    assert!(!final_report_due(true, true, false, 95.0, 90.0));
    // Going around: never.
    assert!(!final_report_due(true, false, true, 90.0, 90.0));
    // A Final still turning onto the line (the pursuit's 35 degree cut is
    // an intercept): not yet.
    assert!(!final_report_due(true, false, false, 125.0, 90.0));
    // Established: the pattern done, lined up within 30 degrees, Final.
    assert!(final_report_due(true, false, false, 96.0, 90.0));
    assert!(final_report_due(true, false, false, 357.0, 2.0));
}
