    use super::lemg_replay::*;
    use super::*;

    /// The logged cross-track is the aircraft's real offset, and the flip
    /// across the threshold was the aircraft crossing the centreline, not a
    /// reference changing: sortie 1790591256-full_auto's `state` positions
    /// against apt.dat's 13/31 row, with the offsets computed independently
    /// (east/north metres off the 13 threshold, projected on the line to 31)
    /// and the `autoland` tick's `xtk_m` nearest in time.
    #[test]
    fn the_lemg_13_cross_track_is_the_true_offset_across_the_threshold() {
        let rwy = lemg("13");
        assert!((rwy.course_deg - 131.70).abs() < 0.02, "{}", rwy.course_deg);
        // (lat, lon, true along-track m (approach side positive), true
        // cross-track m (right positive), the logged xtk_m, radio ft)
        let pts: [(f64, f64, f64, f64, f64, f64); 7] = [
            (
                36.70614964963893,
                -4.541190214471253,
                3499.0,
                -98.4,
                -91.9,
                465.0,
            ),
            (
                36.69694294373395,
                -4.529138374167257,
                2015.6,
                -48.8,
                -48.2,
                413.0,
            ),
            (
                36.68657643707785,
                -4.514858502002209,
                298.0,
                -35.2,
                -32.4,
                90.0,
            ),
            (
                36.68514500057357,
                -4.513137373226305,
                77.6,
                -18.4,
                -14.6,
                44.0,
            ),
            (
                36.68453229043331,
                -4.5124318927144085,
                -14.7,
                -9.4,
                -2.2,
                34.0,
            ),
            (
                36.683699213228394,
                -4.511474633941739,
                -140.1,
                3.0,
                7.8,
                32.0,
            ),
            (
                36.67955142537449,
                -4.506237707168569,
                -795.6,
                36.7,
                38.2,
                0.0,
            ),
        ];
        for (lat, lon, along, xtk, logged, radio) in pts {
            let x = cross_track_nm(lat, lon, &rwy) * M_PER_NM;
            let a = along_track_nm(lat, lon, &rwy) * M_PER_NM;
            // A metre, plus the flat-earth sum's own 0.1 % over distance.
            let tol = 1.0 + along.abs() * 1e-3;
            assert!(
                (x - xtk).abs() < tol,
                "at {radio} ft: {x:.1} m against {xtk:.1} true"
            );
            assert!(
                (a - along).abs() < 2.0,
                "at {radio} ft: {a:.0} m along against {along:.0}"
            );
            // The loop's own number agrees with the position to within the
            // tick it was sampled apart (7.7 m/s sideways over the threshold).
            assert!(
                (x - logged).abs() < 8.0,
                "at {radio} ft: logged {logged:.1}, {x:.1} m"
            );
        }
        // Left of the line 300 m short, across it at the threshold, right of
        // it at the wheels: an S-turn, and 36.7 m on a 45 m strip is 14 m
        // into the grass.
        let wheels = cross_track_nm(pts[6].0, pts[6].1, &rwy) * M_PER_NM;
        assert!(wheels > LEMG_WIDTH_M / 2.0 + 10.0, "{wheels:.1}");
    }

    /// The same final's positions, judged by the gates on LEMG's 45 m
    /// pavement: at the handoff (98 m off at 1.9 nm, 471 ft) the pursuit's
    /// own time model leaves 20 m at the threshold, 5 m past the landing
    /// limit, and the final is not stabilised below the floor; 49 m off at
    /// 1.1 nm the same; 29 m off at 78 ft is off the centreline low. On a
    /// runway of unknown width all three were FINAL, which is what flew.
    #[test]
    fn the_lemg_13_final_goes_around_below_the_floor_on_a_45_m_strip() {
        let rwy = lemg("13");
        assert!((landing_xtk_limit_m(Some(LEMG_WIDTH_M)) - 15.05).abs() < 0.01);
        // (lat, lon, alt ft MSL, radio ft, gs kt, the balk wanted)
        let pts = [
            (
                36.70614964963893,
                -4.541190214471253,
                533.4,
                471.6,
                148.6,
                "unstabilised_below_floor",
            ),
            (
                36.69694294373395,
                -4.529138374167257,
                479.5,
                432.0,
                150.1,
                "unstabilised_below_floor",
            ),
            (
                36.68578005055136,
                -4.513846406166833,
                136.2,
                77.6,
                146.5,
                "off_centreline_low",
            ),
        ];
        for (lat, lon, alt, radio, gs, want) in pts {
            let g = guidance_on(
                lat,
                lon,
                alt,
                radio,
                gs,
                gs,
                &rwy,
                145.0,
                25.0,
                0.0,
                Some(LEMG_WIDTH_M),
            );
            assert_eq!(g.balk, Some(want), "at {radio} ft");
            assert_eq!(g.phase, ApproachPhase::Intercept, "at {radio} ft");
            let old = guidance(lat, lon, alt, radio, gs, gs, &rwy, 145.0, 25.0, 0.0);
            assert_eq!(
                (old.phase, old.balk),
                (ApproachPhase::Final, None),
                "what flew, at {radio} ft"
            );
        }
        // On the line it is a final: 5 m off at 1.1 nm and at 78 ft.
        for (along, radio) in [(2015.6, 432.0), (171.7, 77.6)] {
            let (lat, lon) = at(&rwy, along, -5.0);
            let alt = rwy.elev_ft + radio;
            let g = guidance_on(
                lat,
                lon,
                alt,
                radio,
                148.0,
                148.0,
                &rwy,
                145.0,
                25.0,
                0.0,
                Some(LEMG_WIDTH_M),
            );
            assert_eq!(
                (g.phase, g.balk),
                (ApproachPhase::Final, None),
                "5 m off at {radio} ft"
            );
        }
    }

    /// LEMG 31 (sortie 1790584715-full_auto): handed over 103 m right at
    /// 1.8 nm, still rolling out of the circuit's intercept (heading 299 for
    /// a 311.7 runway, 15 degrees of bank). The old track trim learnt the
    /// roll-out as a bias (+0.78 recorded) and the final sat 30 to 32 m right
    /// from 1.4 nm to 0.3 with the autopilot on its bug. The final's one law
    /// (`FinalLateral`) learns nothing from a roll-out: its integral is of
    /// the offset, bounded at 8 m. Replayed: inside the landing limit by
    /// 0.5 nm.
    #[test]
    fn the_final_closes_from_the_handoffs_roll_out() {
        let rwy = lemg("31");
        let (lat, lon) = at(&rwy, 3392.9, 103.1);
        let mut jet = Jet::new(lat, lon, 299.4, 149.0);
        jet.wind_mps = 3.0;
        jet.phi = 14.8;
        let mut lp = FinalLoop::new(None);
        let mut at_half = f64::NAN;
        for _ in 0..1000 {
            let (bug, g) = lp.step(&jet, &rwy, 0.35);
            if g.dist_nm < 0.5 {
                at_half = cross_track_nm(jet.lat, jet.lon, &rwy) * M_PER_NM;
                break;
            }
            jet.step(bug, 0.35);
        }
        assert!(
            at_half.abs() < landing_xtk_limit_m(Some(LEMG_WIDTH_M)),
            "{at_half:.1} m at 0.5 nm"
        );
    }

    /// Issue #93: autoland's final with the knob set as a pilot sets it (a
    /// quarter degree at most every second, down to the arrest) holds the
    /// line the every-tick knob held, from the LEMG 31 handover and offsets
    /// either side of 13 in four winds: no balk, the roundout gate within
    /// 2 m of it, the worst offset below 500 ft within 3 m of it or inside
    /// the landing limit (the plant has no gusts, so the knob's moves are
    /// counted on the recordings: `the_recorded_finals_move_the_knob_less`).
    #[test]
    fn the_final_on_a_held_knob_keeps_the_line() {
        let limit = landing_xtk_limit_m(Some(LEMG_WIDTH_M));
        // (runway, along m, xtk m, heading true, IAS, wind from, m/s)
        let starts = [
            ("31", 3392.9, 103.1, 299.4, 149.0, 106.0, 3.0),
            ("13", 3700.0, 30.0, 126.0, 146.0, 106.0, 6.0),
            ("13", 3700.0, -40.0, 136.0, 146.0, 200.0, 8.0),
            ("13", 5500.0, 0.0, 131.7, 150.0, 40.0, 7.0),
        ];
        for (id, along, xtk, psi, ias, wf, wm) in starts {
            let rwy = lemg(id);
            let fly = |held: bool| {
                let (lat, lon) = at(&rwy, along, xtk);
                let mut jet = Jet::new(lat, lon, psi, ias);
                jet.wind_to_deg = (wf + 180.0_f64).rem_euclid(360.0);
                jet.wind_mps = wm;
                let lp = FinalLoop::new(Some(LEMG_WIDTH_M));
                let mut lp = if held { lp.held() } else { lp };
                let f = fly_final(&mut jet, &mut lp, &rwy, 0.35);
                (f, lp.moves)
            };
            let (free, free_moves) = fly(false);
            let (f, moves) = fly(true);
            assert!(f.balk.is_none() && free.balk.is_none(), "{id} {along} {xtk}: {f:?}");
            // Within 2 m of the every-tick knob or 5 m of the line: the
            // trim that learns no roll-out (#189) left the free knob 0.1 m
            // at the gate from LEMG 31 where the held one keeps 3.2.
            assert!(
                f.gate_xtk_m.abs() <= (free.gate_xtk_m.abs() + 2.0).max(5.0)
                    && f.worst_low_xtk_m <= (free.worst_low_xtk_m + 3.0).max(limit),
                "{id} {along} {xtk}: {f:?} against {free:?}"
            );
            // The knob moves no more than the every-tick one would, or a
            // handful of times: with no roll-out learnt (#189) the free bug
            // barely moves in this still air (5 moves from 13 3700 30,
            // against the held knob's 8).
            assert!(moves <= free_moves.max(10), "{id}: {moves} against {free_moves}");
        }
    }

    /// The knob's keys: the arrest and the flare steer every tick; a
    /// pattern leg, a missed approach stage, the final and an intercept are
    /// each set once and corrected past their deadband.
    #[test]
    fn the_knob_keys() {
        use crate::reversal::Leg;
        assert_eq!(bug_knob(ApproachPhase::Flare, false, None, None), None);
        assert_eq!(bug_knob(ApproachPhase::Final, true, None, None), None);
        assert_eq!(
            bug_knob(ApproachPhase::Final, false, None, None),
            Some((2, AUTOLAND_FINAL_DEADBAND))
        );
        let out = bug_knob(ApproachPhase::Intercept, false, Some(Leg::Outbound), None);
        let base = bug_knob(ApproachPhase::Intercept, false, Some(Leg::Base), None);
        assert!(out.is_some() && base.is_some() && out != base);
        let climb = bug_knob(ApproachPhase::Intercept, false, None, Some(MissedStage::Climb));
        let repo = bug_knob(ApproachPhase::Intercept, false, None, Some(MissedStage::Reposition));
        assert!(climb.is_some() && repo.is_some() && climb != repo);
        assert_eq!(
            bug_knob(ApproachPhase::Intercept, false, None, None),
            Some((1, AUTOLAND_INTERCEPT_DEADBAND))
        );
    }

    /// The limits follow the pavement, and a runway of unknown width keeps
    /// the old 37 m.
    #[test]
    fn the_landing_limits_follow_the_pavement() {
        let legacy = FLARE_MAX_XTK_NM * M_PER_NM;
        assert_eq!(landing_xtk_limit_m(None), legacy);
        assert_eq!(flare_xtk_limit_m(None), legacy);
        assert!((landing_xtk_limit_m(Some(45.11)) - 15.05).abs() < 0.01);
        assert!((flare_xtk_limit_m(Some(45.11)) - 18.55).abs() < 0.01);
        assert!((landing_xtk_limit_m(Some(60.0)) - 22.5).abs() < 1e-9);
        // A narrow strip keeps a quarter of its width; a wide one the old cap.
        assert!((landing_xtk_limit_m(Some(18.0)) - 4.5).abs() < 1e-9);
        assert_eq!(landing_xtk_limit_m(Some(150.0)), legacy);
        assert_eq!(landing_xtk_limit_m(Some(f64::NAN)), legacy);
        assert!(flare_xtk_limit_m(Some(18.0)) >= landing_xtk_limit_m(Some(18.0)));
    }

    /// The width is read with the runway: `nearby_runways` notes it, and the
    /// arm's runway finds it.
    #[test]
    fn the_runway_read_from_apt_dat_carries_its_width() {
        let dir = std::env::temp_dir().join(format!("xvl-147-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let apt = dir.join("apt.dat");
        std::fs::write(
            &apt,
            format!("I\n1200 test\n\n1     48 0 0 LEMG Malaga\n{LEMG_13_31}\n99\n"),
        )
        .unwrap();
        let rwy = lemg("13");
        let found = runway_by_id(
            apt.to_str().unwrap(),
            rwy.thr_lat,
            rwy.thr_lon,
            "13",
            5.0,
            Some("LEMG"),
        )
        .unwrap()
        .expect("LEMG 13");
        assert_eq!(runway_width_m(&found), Some(LEMG_WIDTH_M));
        assert_eq!(runway_width_m(&lemg("31")), Some(LEMG_WIDTH_M));
        let _ = std::fs::remove_dir_all(&dir);
    }
