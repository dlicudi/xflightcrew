    use super::*;

    #[test]
    fn a_readback_copy_goes_out_only_while_the_tower_waits_for_it() {
        let copy = "Runway one three, circuit height one thousand feet, QNH one zero one \
                    seven, squawk four five zero one, November eight one six November November";
        let owed = format!("{copy}.");
        // +36.1: the tower still waits for these words.
        assert!(!owed_copy_stale(copy, Some(&owed)));
        // +63.2: heard, and the tower now waits for the takeoff's readback.
        assert!(owed_copy_stale(copy, None));
        let takeoff = "Cleared for takeoff runway one three, November eight one six November \
                       November.";
        assert!(owed_copy_stale(copy, Some(takeoff)));
        assert_eq!(readback_words(&owed), readback_words(copy));
    }
