    use super::*;

    /// The published side is the side asked of the terrain pick; a runway
    /// without one keeps the pilot's.
    #[test]
    fn the_published_side_is_asked_of_the_terrain_pick() {
        assert!(pattern_side_wanted(false, Some(true)), "LEMG 13 left");
        assert!(!pattern_side_wanted(true, Some(false)));
        assert!(!pattern_side_wanted(false, None), "unpublished: as asked");
        assert!(pattern_side_wanted(true, None));
    }

    /// ...and terrain still vetoes it, said as a switch: the published left
    /// over 1,400 ft of ridge flies right (the Rock's numbers, LXGB).
    #[test]
    fn terrain_still_vetoes_the_published_side() {
        let facts = |t: f64| SideFacts {
            terrain_max_ft: Some(t),
            ..Default::default()
        };
        let want = pattern_side_wanted(false, Some(true));
        let pick = choose_pattern(want, &facts(1400.0), &facts(11.0), 11.0, 1000.0);
        assert!(pick.refused.is_none());
        assert!(!pick.left);
        assert_eq!(pick.why, "switched_side_for_terrain");
        let pick = choose_pattern(want, &facts(11.0), &facts(5.0), 11.0, 1000.0);
        assert!(pick.left, "both clear: the published side stands");
    }

    /// Said once before the takeoff: a field without circuits, flown as
    /// asked, and a published side other than the one asked for.
    #[test]
    fn the_field_data_is_said_before_the_takeoff() {
        let line = field_circuit_line("Malaga", "31", false, Some(true), Some(false)).unwrap();
        assert_eq!(
            line,
            "The airport data says no circuits at Malaga, flying this one as asked. Runway \
             three one has a published left hand circuit, flying it left."
        );
        assert!(!line.contains('-') && !line.contains('\u{2014}'));
        assert_eq!(
            field_circuit_line("Malaga", "13", true, Some(true), Some(true)),
            None
        );
        assert_eq!(field_circuit_line("Malaga", "13", true, None, None), None);
        let line = field_circuit_line("Gatwick", "26L", true, Some(true), Some(false)).unwrap();
        assert!(!line.contains("published"), "{line}");
    }
