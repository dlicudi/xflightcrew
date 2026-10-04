    use super::region_coach_line;
    use crate::party_line::RegionHandoff;

    fn h(squawk_vfr: bool) -> RegionHandoff {
        RegionHandoff {
            to: "Malaga Tower".into(),
            khz: 118_100,
            squawk_vfr,
            ifr_code: None,
        }
    }

    /// An IFR flight keeps its code: the coach never mentions a squawk and
    /// never says the service ended.
    #[test]
    fn an_ifr_handoff_is_coached_as_a_handover() {
        let ifr = RegionHandoff {
            ifr_code: Some(4501),
            ..h(false)
        };
        assert_eq!(region_coach_line(&ifr, Some(118_100), None), None);
        assert_eq!(
            region_coach_line(&ifr, Some(130_400), None).as_deref(),
            Some("We've been handed over: Malaga Tower on one one eight decimal one.")
        );
    }

    #[test]
    fn names_only_what_is_still_undone() {
        // Both done: silence.
        assert_eq!(region_coach_line(&h(true), Some(118_100), Some(7000)), None);
        // 8.33 kHz spacing rounds the read within 5 kHz: still tuned.
        assert_eq!(region_coach_line(&h(true), Some(118_105), Some(7000)), None);
        // Code still discrete, frequency right.
        assert_eq!(
            region_coach_line(&h(true), Some(118_100), Some(4503)).as_deref(),
            Some("Radar's done with us: squawk seven thousand.")
        );
        // Frequency wrong, code right.
        assert_eq!(
            region_coach_line(&h(true), Some(130_400), Some(7000)).as_deref(),
            Some("Radar's done with us: Malaga Tower on one one eight decimal one.")
        );
        // Neither done.
        assert_eq!(
            region_coach_line(&h(true), Some(130_400), Some(4503)).as_deref(),
            Some("Radar's done with us: squawk seven thousand, and Malaga Tower on one one eight decimal one.")
        );
        // No squawk ordered: the code is never mentioned, whatever it reads.
        assert_eq!(region_coach_line(&h(false), Some(118_100), None), None);
        assert_eq!(
            region_coach_line(&h(false), None, Some(4503)).as_deref(),
            Some("Radar's done with us: Malaga Tower on one one eight decimal one.")
        );
    }
