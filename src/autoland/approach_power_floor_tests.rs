    use super::{approach_power_floor, ApproachPhase::*};

    /// The 737 (thrust floor 0.3): on final at Vref the lever never goes
    /// under 0.3; 20 kt fast it may idle; a piston with no floor idles; a
    /// reposition under 500 ft is full power whatever the speed.
    #[test]
    fn the_lever_floor_follows_the_phase_and_the_airframe() {
        assert_eq!(
            approach_power_floor(Final, 300.0, 0.0, Some(0.3), true),
            0.3
        );
        assert_eq!(
            approach_power_floor(Final, 300.0, -6.0, Some(0.3), true),
            0.3
        );
        assert_eq!(
            approach_power_floor(Final, 300.0, -20.0, Some(0.3), true),
            0.1
        );
        assert_eq!(approach_power_floor(Final, 300.0, 0.0, None, true), 0.1);
        assert_eq!(
            approach_power_floor(Intercept, 140.0, -58.0, Some(0.3), false),
            1.0
        );
        assert_eq!(
            approach_power_floor(Intercept, 1500.0, -58.0, Some(0.3), false),
            0.1
        );
        assert_eq!(approach_power_floor(Flare, 20.0, 0.0, Some(0.3), true), 0.1);
    }

    /// LEMG 13, 2026-09-26, every final: flaps 10 out from the FAP, the
    /// target floored to that configuration's 152 kt, and the 737 sat at 161
    /// to 164 kt for a minute with the lever flipping 0.1 and 0.3 on the
    /// ten-knot edge (0.3 on a 3 degree path with flaps 10 accelerates it).
    /// Short of the landing flaps the speed law may idle; with them out the
    /// floor stands as before. An airframe with no floor never had one.
    #[test]
    fn the_thrust_floor_is_the_landing_configurations() {
        // 161 kt against the flaps-10 floor of 152: nine knots fast.
        assert_eq!(
            approach_power_floor(Final, 1200.0, -9.0, Some(0.3), false),
            0.1
        );
        assert_eq!(
            approach_power_floor(Final, 1200.0, -9.0, Some(0.3), true),
            0.3
        );
        assert_eq!(approach_power_floor(Final, 1200.0, -9.0, None, false), 0.1);
        assert_eq!(approach_power_floor(Final, 1200.0, -9.0, None, true), 0.1);
    }

    /// EGCC 23L, 2026-09-21: the servos stayed on through the rollout and the
    /// taxi, and the heading servo wound the yoke over on the ground.
    #[tokio::test]
    async fn the_autopilot_comes_off_at_the_wheels() {
        use crate::xplane::FakeXPlane;
        let xp = FakeXPlane::with_reads(&[(super::SERVOS_ON, 1.0)]);
        // The fake never moves, so the readback disagrees and it is SAID so.
        assert_eq!(super::disconnect_autopilot(&xp).await, Some(false));
        assert!(xp.ran(super::CMD_SERVOS_FDIR_OFF));
        // Servos already out by the arrest, the master and yaw damper still
        // on (King Air LEMG 13, #197): both come out.
        let xp = FakeXPlane::with_reads(&[
            (super::SERVOS_ON, 0.0),
            (super::AP_MASTER_ON, 1.0),
            (super::YAW_DAMPER_ON, 1.0),
        ]);
        assert!(super::disconnect_autopilot(&xp).await.is_some());
        assert!(xp.ran(super::CMD_SERVOS_FDIR_OFF));
        assert!(xp.ran(super::CMD_YAW_DAMPER_OFF));
        // All off (a hand-flown landing under autoland's callouts): left.
        let xp = FakeXPlane::with_reads(&[
            (super::SERVOS_ON, 0.0),
            (super::AP_MASTER_ON, 0.0),
            (super::YAW_DAMPER_ON, 0.0),
        ]);
        assert_eq!(super::disconnect_autopilot(&xp).await, None);
        assert!(!xp.ran(super::CMD_SERVOS_FDIR_OFF));
    }

    /// LSZS RNAV 03 (2026-09-24): the MAP is 1.98 nm from the threshold,
    /// 996 ft above it, 380 m right of the extended centreline. Handed the
    /// visual segment's 4.74 degrees, the landing takes it from there on its
    /// own path; at 3 degrees the same point would be 300 ft high.
    #[test]
    fn a_visual_segment_is_flown_on_its_own_path() {
        static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let rwy = super::Runway {
            icao: "LSZS".into(),
            id: "03".into(),
            thr_lat: 46.5270195,
            thr_lon: 9.8780890,
            course_deg: 29.0,
            length_m: 1842.0,
            surface: 1,
            dist_nm: 0.0,
            elev_ft: 5602.0,
        };
        let (map_lat, map_lon) = (46.5, 9.850666667);
        // 996 ft over the threshold at 1.98 nm, measured to the aim point
        // 300 m past it: what the follower hands down.
        let aim = super::TOUCHDOWN_AIM_NM;
        let deg = (996.0_f64 / ((1.98 + aim) * 6076.12)).atan().to_degrees();
        let hand = super::HandDown::visual_segment(Some(deg));
        assert!((super::arm_glidepath(hand.glidepath) - deg).abs() < 1e-9);
        assert!(deg > 4.2 && deg < 4.6, "{deg}");
        let g = super::guidance(map_lat, map_lon, 6598.0, 996.0, 80.0, 90.0, &rwy, 87.0, 25.0, 0.0);
        let above = 6598.0 - g.glidepath_alt_ft;
        assert!(above.abs() < 40.0, "on the steep path at the MAP: {above:.0} ft");
        assert_eq!(g.balk, None, "no balk at the MAP");
        assert_eq!(g.phase, super::ApproachPhase::Final);
        assert!(g.target_vs_fpm < -450.0 && g.target_vs_fpm > -1000.0, "{}", g.target_vs_fpm);
        // The next arm without a handed angle is 3 degrees again.
        assert_eq!(super::arm_glidepath(None), 3.0);
        assert_eq!(super::gp_ft_per_nm(), super::GLIDEPATH_FT_PER_NM);
        let g3 = super::guidance(map_lat, map_lon, 6598.0, 996.0, 80.0, 90.0, &rwy, 87.0, 25.0, 0.0);
        assert!(6598.0 - g3.glidepath_alt_ft > 250.0, "a 3 degree path calls it high");
    }

    /// LEMG 13 SR22 circuit, 2026-09-29 (sortie 1790692781-full_auto): the
    /// circuit steepened its path to 3.26 degrees over the ground (#182)
    /// and autoland took that for LSZS's valley: no pattern when it arrived
    /// 560 ft high, and after the balk no published missed approach, so
    /// "climbing straight ahead. Your aircraft." to nobody at 805 ft. A
    /// circuit's angle is flown but is not a published visual segment.
    #[test]
    fn a_circuit_path_is_steep_but_not_a_valley() {
        use super::HandDown;
        let circuit = HandDown::circuit(Some(3.2576));
        assert!((super::arm_glidepath(circuit.glidepath) - 3.2576).abs() < 1e-9, "the angle is flown");
        assert!(!super::published_steep(), "a circuit's path is not a valley");
        let map = HandDown::visual_segment(Some(4.5));
        assert!((super::arm_glidepath(map.glidepath) - 4.5).abs() < 1e-9);
        assert!(super::published_steep(), "the follower's MAP path is");
        // A handed 3 degrees (or none) is never steep, whoever handed it.
        let three = HandDown::visual_segment(Some(3.0));
        assert_eq!(super::arm_glidepath(three.glidepath), 3.0);
        assert!(!super::published_steep());
        assert_eq!(super::arm_glidepath(None), 3.0);
        assert!(!super::published_steep());
    }

    /// The gate and angle ride the launch, so an arm handed nothing flies
    /// the instrument gate on 3 degrees: a circuit's visual gate or path can
    /// never reach the next arm, perhaps another flight's instrument
    /// approach (a stand-down during the circuit's handoff once left them
    /// in a static for it).
    #[test]
    fn a_launch_handed_nothing_flies_the_defaults() {
        use crate::stable_approach::GateKind;
        let none = super::HandDown::default();
        assert_eq!(none.gate, GateKind::Instrument);
        assert_eq!(none.glidepath, None);
        assert_eq!(super::arm_glidepath(none.glidepath), 3.0);
        assert!(!super::published_steep());
        let circuit = super::HandDown::circuit(None);
        assert_eq!(circuit.gate, GateKind::Visual);
        assert_eq!(super::arm_glidepath(circuit.glidepath), 3.0);
        assert_eq!(super::HandDown::visual_segment(Some(4.5)).gate, GateKind::Instrument);
        super::forget_flight();
        assert_eq!(super::landed_runway(), None);

        let src = crate::test_support::module_source("autoland");
        let body = src
            .split("fn spawn_autoland(launch: AutolandLaunch)")
            .nth(1)
            .unwrap();
        assert!(body.contains("let gate = hand_down.gate;"));
        assert!(body.contains("arm_glidepath(hand_down.glidepath)"));
        // The flag rises when the run is taken over (`RUN`, flagged).
        let active = body.find("RUN.take_over()").unwrap();
        let cleared = body.find("TRACKING_ERROR.clear();").unwrap();
        assert!(
            cleared < active,
            "the last tracking goes before ACTIVE is set"
        );
    }

    /// The same sortie's other half: the steep branch's missed approach that
    /// cannot be flown falls through to the terrain-probed pattern, never to
    /// "Your aircraft" (source-pinned: the loop is not a plain test).
    #[test]
    fn an_unflyable_missed_procedure_is_never_your_aircraft() {
        let src = crate::test_support::module_source("autoland");
        let body = src
            .split("if wanted && steep {")
            .nth(1)
            .expect("the steep branch")
            .split("if wanted && reversals_flown >= crate::reversal::MAX_REVERSALS")
            .next()
            .unwrap();
        assert!(!body.contains("Your aircraft"), "no hand-back to nobody");
        assert!(body.contains("missed_procedure_tried = true"), "asked once");
        assert!(src.contains("let steep = published_steep() && !missed_procedure_tried;"));
        let circuit = crate::test_support::module_source("circuit");
        assert!(circuit.contains("descent_shape.steep().then(|| descent_shape.glidepath_deg())"));
        assert!(!circuit.contains("HandDown::visual_segment("));
    }
