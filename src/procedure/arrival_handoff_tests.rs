    use super::*;

    #[test]
    fn the_egnx_arrival_slows_to_210_before_the_end_of_the_route() {
        // EGBB-EGNX (2026-09-30): 250 kt at 3,000 ft, about 260 over the
        // ground, the 737's clean approach speed 210. The slow-down needs
        // 40 kt at 0.5 kt/s: 5.8 nm, plus the 2 nm lead.
        let mut begun = false;
        assert_eq!(arrival_handoff_cap(12.0, 250.0, 260.0, 210.0, &mut begun), None);
        assert!(!begun);
        assert_eq!(arrival_handoff_cap(7.7, 250.0, 260.0, 210.0, &mut begun), Some(210.0));
        // Held once begun, whatever the speed and distance say next.
        assert_eq!(arrival_handoff_cap(9.0, 215.0, 220.0, 210.0, &mut begun), Some(210.0));
        // Already slow: only the lead before the end.
        let mut slow = false;
        assert_eq!(arrival_handoff_cap(3.0, 200.0, 200.0, 210.0, &mut slow), None);
        assert_eq!(arrival_handoff_cap(1.9, 200.0, 200.0, 210.0, &mut slow), Some(210.0));
    }

    #[test]
    fn the_handoff_speed_rides_an_enroute_plan_only() {
        assert_eq!(arrival_handoff_for(Some(210.25), &EndAction::Enroute), Some(210.25));
        assert_eq!(arrival_handoff_for(Some(f64::NAN), &EndAction::Enroute), None);
        assert_eq!(arrival_handoff_for(Some(0.0), &EndAction::Enroute), None);
        // A plan launched without one (every caller but full auto's
        // enroute) gets none.
        assert_eq!(arrival_handoff_for(None, &EndAction::Enroute), None);
        assert_eq!(
            arrival_handoff_for(Some(210.0), &EndAction::Level),
            None
        );
    }
