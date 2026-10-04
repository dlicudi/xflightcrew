    use super::{inbound_due, INBOUND_CALL_NM};

    /// LEJR-LEGR, 2026-09-15: the region handed the flight to Granada Tower
    /// 20 nm out and the inbound waited for the last fix, a mile from the
    /// runway. Owed on the destination's own frequency inside the call
    /// distance; never on a transit field's, never twice.
    #[test]
    fn inbound_owed_on_the_destination_tower_inside_the_call_distance() {
        let dest = Some(118_850);
        assert!(
            !inbound_due(dest, dest, INBOUND_CALL_NM + 5.0, false, false),
            "still far out"
        );
        assert!(
            inbound_due(dest, dest, INBOUND_CALL_NM, false, false),
            "at the call distance"
        );
        assert!(inbound_due(dest, dest, 3.0, false, false));
        assert!(!inbound_due(dest, dest, 3.0, false, true), "already made");
        // Tangier held the frequency mid-strait: not the tower we are landing at.
        assert!(
            !inbound_due(dest, Some(121_700), 3.0, false, false),
            "another region's tower"
        );
        assert!(
            !inbound_due(dest, None, 3.0, false, false),
            "no station on the air"
        );
        assert!(
            !inbound_due(None, dest, 3.0, false, false),
            "no tower row to call"
        );
    }

    /// The tower's "say callsign and intentions" is answered at any range,
    /// but only by the tower we are inbound to.
    #[test]
    fn a_challenge_from_the_destination_tower_earns_the_inbound_now() {
        let dest = Some(118_850);
        assert!(inbound_due(dest, dest, 30.0, true, false));
        assert!(
            !inbound_due(dest, dest, 30.0, true, true),
            "already made: no second call"
        );
        assert!(
            !inbound_due(dest, Some(121_700), 2.0, true, false),
            "a transit field's question"
        );
    }
