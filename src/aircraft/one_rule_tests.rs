    use super::*;

    /// Every shipped profile's id.
    fn profile_ids() -> Vec<String> {
        std::fs::read_dir("profiles")
            .expect("profiles dir")
            .filter_map(|e| e.ok())
            .filter_map(|e| {
                let name = e.file_name().to_string_lossy().to_string();
                name.strip_suffix(".yaml").map(str::to_string)
            })
            .collect()
    }

    /// max(1.3 Vso, 1.2 Vs), and no flat 70: the SR22's 84, the 737's 167,
    /// a 172 declaring no Vso its 58 (the registry's old floor gave 70).
    #[test]
    fn the_fallback_approach_is_the_landing_stalls_without_a_flat_floor() {
        assert_eq!(fallback_approach_kts(Some(60.0), 70.0), 84.0);
        assert_eq!(fallback_approach_kts(Some(114.0), 139.0), 167.0);
        assert_eq!(fallback_approach_kts(None, 48.0), 58.0);
        assert_eq!(fallback_approach_kts(Some(f64::NAN), 48.0), 58.0);
        // Never under the automation floor.
        for (vso, vs) in [(Some(40.0), 48.0), (Some(125.0), 175.0), (None, 90.0)] {
            assert!(fallback_approach_kts(vso, vs) >= (vs * SPEED_FLOOR_RATIO).round());
        }
    }

    /// A declared approach speed wins; else the fallback from the stalls;
    /// with neither, nothing. The instructor reads this, so its brief
    /// agrees with autoland (it briefed 1.3 x the clean stall).
    #[test]
    fn a_profile_approach_is_declared_else_the_one_fallback() {
        let mut p = load("cessna_172sp").unwrap();
        assert_eq!(approach_kts(&p), Some(65.0));
        p.reference.as_mut().unwrap().approach_kts = None;
        // 172: Vso 40, Vs 48: max(52, 57.6) = 58 (1.3 x clean was 70).
        assert_eq!(approach_kts(&p), Some(58.0));
        p.vs_clean_kts = None;
        assert_eq!(approach_kts(&p), None);
        for id in ["cessna_172sp", "cirrus_sr22", "boeing_737_800", "airbus_a330_300"] {
            let p = load(id).unwrap();
            assert_eq!(approach_kts(&p), p.reference.as_ref().unwrap().approach_kts, "{id}");
        }
    }

    /// One Vr fallback, 1.1 x the clean stall (the registry's 1.15 is gone).
    #[test]
    fn the_fallback_rotation_is_one_point_one_vs() {
        assert_eq!(fallback_vr_kts(48.0), 53.0);
        assert_eq!(fallback_vr_kts(65.0), 72.0);
        assert_eq!(fallback_vr_kts(139.0), 153.0);
    }

    /// The recovery holds 1.3 Vs with its 70 kt floor.
    #[test]
    fn a_recovery_holds_its_margin() {
        assert_eq!(recovery_hold_kts(48.0), 70.0);
        assert!((recovery_hold_kts(139.0) - 180.7).abs() < 1e-9);
    }

    /// The landing rule: the book landing distance, else the takeoff
    /// distance, x 1.3, never under the caller's floor. The self-pick's
    /// 1,200 m and the planner's none are both this function.
    #[test]
    fn a_landing_needs_the_book_distance() {
        let ft = 0.3048;
        // 737: the POM's 5,360 ft is already a factored field length, so a
        // plan needs 5,360 and LXGB's 5,449 ft is a destination.
        let need = landing_needed_m(Some(5360.0 * ft), None, 0.0) / ft;
        assert!((need - 5360.0).abs() < 1.0, "{need}");
        assert!(need < 5449.0, "a 737 into LXGB");
        // Self-pick: the floor rules a light aircraft.
        assert_eq!(landing_needed_m(Some(407.0), None, 1200.0), 1200.0);
        assert_eq!(landing_needed_m(None, None, 1200.0), 1200.0);
        assert_eq!(landing_needed_m(Some(f64::NAN), None, 1200.0), 1200.0);
        // The takeoff figure stands in only when no landing figure is on file.
        assert!((landing_needed_m(None, Some(500.0), 0.0) - 500.0).abs() < 1e-9);
        assert!((landing_needed_m(Some(400.0), Some(500.0), 0.0) - 400.0).abs() < 1e-9);
        // The plan's form: feet, no floor, None with nothing on file.
        assert_eq!(planned_landing_needed_ft(None, None), None);
        let c172 = planned_landing_needed_ft(Some(1335.0), None).unwrap();
        assert!((c172 - 1335.0).abs() <= 1.0, "{c172}");
        // A destination the plan passes, the self-pick would take too (for
        // any runway past the self-pick's floor).
        for book_ft in [1335.0, 1721.0, 4750.0, 5360.0] {
            let plan_m = planned_landing_needed_ft(Some(book_ft), None).unwrap() * ft;
            let pick_m = landing_needed_m(Some(book_ft * ft), None, 1200.0);
            assert!(plan_m.max(1200.0) + 1.0 >= pick_m, "{book_ft}");
        }
    }

    /// One placard lookup: the lowest Vfe at or inside the flap, no ratio-0
    /// placard, a reading within 0.02 of a detent at it, and no further.
    #[test]
    fn the_flap_placard_is_one_lookup() {
        let limits = [
            (0.0, Some(250.0)), // a generic .acf's retracted row with a Vfe
            (0.125, Some(250.0)),
            (0.375, Some(250.0)),
            (0.5, Some(210.0)),
            (1.0, Some(162.0)),
        ];
        assert_eq!(flap_placard_kts(0.0, &limits), None, "clean has no flap placard");
        assert_eq!(flap_placard_kts(0.125, &limits), Some(250.0));
        assert_eq!(flap_placard_kts(0.49, &limits), Some(210.0), "a reading at its detent");
        assert_eq!(flap_placard_kts(0.46, &limits), Some(250.0), "not the next detent's");
        assert_eq!(flap_placard_kts(1.0, &limits), Some(162.0));
        assert_eq!(flap_placard_kts(f64::NAN, &limits), None);
        assert_eq!(flap_placard_kts(0.5, &[(0.5, None)]), None);
        // Under the closest detent spacing of every shipped profile.
        for id in profile_ids() {
            let Some(f) = load(&id).and_then(|p| p.flaps) else {
                continue;
            };
            for w in f.detents.windows(2) {
                assert!(
                    w[1].ratio - w[0].ratio > 2.0 * FLAP_DETENT_TOLERANCE,
                    "{id}: detents {} and {} closer than the tolerance",
                    w[0].ratio,
                    w[1].ratio
                );
            }
        }
    }

    /// The "too slow" ratios keep their order: warning under the automation
    /// caution, under the automation floor (which the climb floor now IS),
    /// at or under the hand-flown caution, under the pattern minimum; the
    /// fallback approach never under the floor.
    #[test]
    fn slow_ratios_are_ordered() {
        assert!(STALL_WARNING_RATIO < CAUTION_AUTO_RATIO);
        assert!(CAUTION_AUTO_RATIO < SPEED_FLOOR_RATIO);
        assert!(CLIMB_FLOOR_RATIO >= SPEED_FLOOR_RATIO, "no climb bleeds under the floor");
        assert!(SPEED_FLOOR_RATIO <= CAUTION_HAND_RATIO);
        assert!(CAUTION_HAND_RATIO < PATTERN_MIN_RATIO);
        assert!(APPROACH_VS_RATIO >= SPEED_FLOOR_RATIO);
        assert!(APPROACH_VSO_RATIO > SPEED_FLOOR_RATIO);
    }
