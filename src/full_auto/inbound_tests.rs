    use super::{inbound_due, INBOUND_CALL_NM};

    /// LEJR-LEGR, 2026-09-15: the region handed the flight to Granada Tower
    /// 20 nm out and the inbound waited for the last fix, a mile from the
    /// runway. Owed on the destination's own frequency inside the call
    /// distance; never on a transit field's, never twice.
    #[test]
    fn inbound_owed_on_the_destination_tower_inside_the_call_distance() {
        let dest = Some(118_850);
        assert!(
            !inbound_due(dest, dest, INBOUND_CALL_NM + 5.0, false, false, false),
            "still far out"
        );
        assert!(
            inbound_due(dest, dest, INBOUND_CALL_NM, false, false, false),
            "at the call distance"
        );
        assert!(inbound_due(dest, dest, 3.0, false, false, false));
        assert!(!inbound_due(dest, dest, 3.0, false, false, true), "already made");
        // Tangier held the frequency mid-strait: not the tower we are landing at.
        assert!(
            !inbound_due(dest, Some(121_700), 3.0, false, false, false),
            "another region's tower"
        );
        assert!(
            !inbound_due(dest, None, 3.0, false, false, false),
            "no station on the air"
        );
        assert!(
            !inbound_due(None, dest, 3.0, false, false, false),
            "no tower row to call"
        );
    }

    /// The tower's "say callsign and intentions" is answered at any range,
    /// but only by the tower we are inbound to.
    #[test]
    fn a_challenge_from_the_destination_tower_earns_the_inbound_now() {
        let dest = Some(118_850);
        assert!(inbound_due(dest, dest, 30.0, true, false, false));
        assert!(
            !inbound_due(dest, dest, 30.0, true, false, true),
            "already made: no second call"
        );
        assert!(
            !inbound_due(dest, Some(121_700), 2.0, true, false, false),
            "a transit field's question"
        );
    }

    /// EGKK-EGBB, 2026-10-15: handed to Birmingham Approach, the call waited
    /// for 10 nm and approach had 7.8 nm to bring us down 7,300 ft. An
    /// instrument arrival whose call goes to approach makes it at the
    /// handoff, at any range; a tower-only field still waits for the call
    /// distance, and a transit field is never called.
    #[test]
    fn an_approach_first_arrival_calls_at_the_handoff() {
        let dest = Some(118_305);
        assert!(inbound_due(dest, dest, 30.0, false, true, false));
        assert!(
            !inbound_due(dest, dest, 30.0, false, false, false),
            "the tower waits for the call distance"
        );
        assert!(
            !inbound_due(dest, Some(121_700), 30.0, false, true, false),
            "another region's station"
        );
        assert!(
            !inbound_due(dest, dest, 30.0, false, true, true),
            "already made"
        );
    }
