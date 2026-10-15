use super::*;
use crate::autoflight::{Modes, WindowRule};
use crate::ils::{Coupling, Needles};

/// EGBB 15 (field 327 ft): the window parked at the field went to 300, under
/// the runway, and the 737's ALT HOLD reversion in the go-around drove for
/// it. The missed approach parks it at the level it climbs to, and the
/// ordinary approach takes it back to the field, rounded up.
#[test]
fn the_go_around_window_is_the_missed_level_never_under_the_runway() {
    let r = WindowRule::AtField;
    let field = 327.0;
    assert_eq!(missed_window_ft(r, false, field, None), None);
    let w = missed_window_ft(r, true, field, None).unwrap();
    assert!(
        w >= field + REPOSITION_MIN_AGL_FT && w % 100.0 == 0.0,
        "{w}"
    );
    assert!(w - (field + REPOSITION_MIN_AGL_FT) < 100.0, "{w}");
    assert_eq!(
        missed_window_ft(r, true, field, Some(2_540.0)),
        Some(2_600.0),
        "a pattern's platform"
    );
    assert_eq!(
        missed_window_ft(WindowRule::ParkAbove, true, field, None),
        None,
        "parked above already"
    );
    assert_eq!(autoflight::field_window_ft(field), 400.0);
}

/// 15 nm out the 737 levelled at 5,150 ft for a glideslope while 10 km off
/// the localiser: APP armed, the display's GS "valid", `vdef` a frozen
/// -5.6e-45. Level for the glideslope only once the localiser is captured.
#[test]
fn the_glideslope_is_waited_for_level_only_on_the_localiser() {
    let n = Some(Needles {
        hdef: Some(2.5),
        vdef: Some(-5.6e-45),
        loc_valid: true,
        gs_valid: true,
    });
    let armed = Some(Modes(
        Modes::LOC_ARMED | Modes::GS_ARMED | Modes::HEADING | Modes::VS,
    ));
    assert!(!level_for_glideslope(Coupling::Armed, armed, n));
    let loc = Some(Modes(Modes::LOC | Modes::GS_ARMED | Modes::ALT_HOLD));
    assert!(level_for_glideslope(Coupling::LocCaptured, loc, n));
    let gs = Some(Modes(Modes::LOC | Modes::GS));
    assert!(
        !level_for_glideslope(Coupling::LocCaptured, gs, n),
        "captured"
    );
    let below = Some(Needles {
        vdef: Some(0.4),
        ..n.unwrap()
    });
    assert!(
        !level_for_glideslope(Coupling::LocCaptured, loc, below),
        "beam below"
    );
}

/// 17:26:37: the pilot pushed (yp -0.54), the 737 went to CWS (164, 162),
/// and autoland pressed V/S and HDG and drove TOGA for 15 s against him.
/// A hand or CWS held is a takeover; a blip, or the engage's own
/// pitch sync for the tick before its mode press, is not.
#[test]
fn a_held_hand_on_the_yoke_takes_the_aircraft_from_autoland() {
    assert!(Modes(164).cws() && Modes(162).cws() && Modes(132).cws());
    assert!(!Modes(50).cws() && !Modes(4146).cws() && !Modes(16418).cws());
    assert!(yoke_hands(Some(-0.54), Some(0.34)));
    assert!(!yoke_hands(Some(0.1), None) && !yoke_hands(None, None));

    let mut t = YokeTakeover::default();
    assert!(!t.step(0.0, true, false));
    assert!(!t.step(1.0, false, false), "a blip");
    assert!(!t.step(2.0, true, false));
    assert!(!t.step(4.5, true, true));
    assert!(t.step(5.0, false, true), "held 3 s");

    let mut engage = YokeTakeover::default();
    assert!(!engage.step(0.0, false, true));
    assert!(!engage.step(0.3, false, false), "the mode press took");
    assert!(!engage.step(10.0, false, false));
}
