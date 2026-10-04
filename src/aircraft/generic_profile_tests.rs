    // An unknown airframe flies on its own placards, never the 172's. The
    // fixture is the RV-10's acf, the airframe that showed why (2026-09-11).
    #[test]
    fn an_unknown_airframe_gets_a_profile_from_its_own_acf() {
        let acf = "\
A
1100
ACF
P acf/_ICAO RV10
P acf/_name Van's RV-10
P acf/_Vs_kts 65.000000000
P acf/_Vso_kts 50.000000000
P acf/_Vfe1_kts 96.000000000
P acf/_Vfem_kts 87.000000000
P acf/_Vno_kts 156.000000000
P acf/_Vne_kts 200.000000000
P acf/_Vle_kts 9.9999E4
P acf/_Vx_kts 78.000000000
P acf/_Vy_kts 90.000000000
P acf/_Vbg_kts 85.000000000
P acf/_Va_kts 0.000000000
P acf/_flap_detents 2
P acf/_flap1_dn/0 0.000000000
P acf/_flap1_dn/1 10.000000000
P acf/_flap1_dn/2 30.000000000
P acf/_flap1_dn/3 0.000000000
P acf/_m_empty 1700.000000000
P acf/_m_max 2700.000000000
P _engn/0/_type RCP_INJ
";
        let p = super::generic_from_acf("RV10", acf);
        assert_eq!(p.id, "generic_rv10");
        assert!(
            p.display_name.contains("(no profile)"),
            "{}",
            p.display_name
        );
        assert_eq!(p.vs_clean_kts, Some(65.0));
        assert_eq!(p.vne_kts, Some(200.0));
        assert!(!p.retractable_gear, "Vle 99999 is fixed gear");
        assert!(!p.carb_heat, "injected");
        assert!(p.magnetos, "still a piston");
        let r = p.reference.as_ref().unwrap();
        // 1.3 Vso is 65 — the clean stall. 1.2 Vs is 78, and that is the floor.
        assert_eq!(r.approach_kts, Some(78.0));
        assert_eq!(r.vr_kts, Some(72.0));
        assert_eq!(
            (r.vno_kts, r.vx_kts, r.vy_kts, r.vs0_kts),
            (Some(156.0), Some(78.0), Some(90.0), Some(50.0))
        );
        assert_eq!(
            r.va_kts,
            Some(99.0),
            "a zero placard keeps the base's figure"
        );
        assert_eq!(r.cruise_ias_kts, Some(133.0));
        assert_eq!(r.max_gross_lb, Some(2700.0));
        assert_eq!(r.useful_load_lb, Some(1000.0));
        assert_eq!(r.takeoff_distance_ft, None, "not in the acf: not recited");
        let flaps = p.flaps.as_ref().unwrap();
        let d: Vec<(String, f64, Option<f64>)> = flaps
            .detents
            .iter()
            .map(|d| (d.label.clone(), d.ratio, d.vfe_kts))
            .collect();
        assert_eq!(
            d,
            vec![
                ("up".into(), 0.0, None),
                ("10".into(), 0.3333, Some(96.0)),
                ("full".into(), 1.0, Some(87.0)),
            ]
        );
        assert!(
            p.checklist.is_none(),
            "the 172's checklist is not this aircraft's"
        );
    }

    /// The floor statics are process-wide; the tests that publish them take
    /// this so they never read each other's airframe.
    static FLOOR_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    #[test]
    fn no_target_goes_below_the_stall_floor() {
        let _g = FLOOR_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // The RV-10: Vs 65, floor 78. The 172's 65 kt approach is under it.
        super::publish_speed_floor(Some(65.0));
        assert_eq!(super::speed_floor_kts(), Some(78.0));
        assert_eq!(super::floored_target(65.0, super::speed_floor_kts()), 78.0);
        assert_eq!(
            super::floored_target(100.0, super::speed_floor_kts()),
            100.0
        );
        // A profile with no Vs floors nothing (the old behaviour, honestly).
        super::publish_speed_floor(None);
        assert_eq!(super::floored_target(65.0, super::speed_floor_kts()), 65.0);
    }

    /// The 737: clean Vs 139, Vso 114, Vref 145. The approach is floored
    /// against the LANDING configuration (137), never the clean floor (167)
    /// that flew it to the threshold 22 kt fast (LFLL, 2026-09-17). Without
    /// a Vso the clean floor stands in; without either, nothing floors.
    #[test]
    fn the_approach_floor_is_the_landing_configuration() {
        let _g = FLOOR_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        super::publish_approach_floor(Some(114.0), Some(139.0));
        assert_eq!(super::approach_floor_kts(), Some(137.0));
        assert_eq!(
            super::floored_target(145.0, super::approach_floor_kts()),
            145.0
        );
        super::publish_approach_floor(None, Some(139.0));
        assert_eq!(super::approach_floor_kts(), Some(167.0));
        super::publish_approach_floor(Some(0.0), None);
        assert_eq!(super::approach_floor_kts(), None);
    }

    /// EGPF 2026-09-21: the A330's monitor called "getting slow" through a
    /// CONF 2 base at 175 kt because its stall ran on a straight line from
    /// 175 clean to 125 full (150 at CONF 2, caution under 180). The profile
    /// now declares each notch; the line stays for a profile that declares
    /// nothing, and the floor follows the declared stall.
    #[test]
    fn a_declared_stall_table_replaces_the_straight_line() {
        let a330 = super::load("airbus_a330_300").expect("the A330 loads");
        let pts = super::detent_stall_points(a330.flaps.as_ref());
        assert_eq!(pts, vec![(0.25, 147.0), (0.5, 139.0), (0.75, 131.0)]);
        let at = |r: f64| super::stall_speed_in_config(175.0, Some(125.0), &pts, r);
        assert_eq!((at(0.0), at(0.5), at(1.0)), (175.0, 139.0, 125.0));
        assert!((at(0.375) - 143.0).abs() < 1e-9, "straight between notches");
        assert!(at(f64::NAN) == 175.0, "an unreadable flap is clean");
        // Nothing declared: exactly the old line, and the trainer's 17 %
        // without a Vso.
        assert_eq!(super::stall_speed_in_config(175.0, Some(125.0), &[], 0.5), 150.0);
        assert!((super::stall_speed_in_config(48.0, None, &[], 1.0) - 48.0 * 0.83).abs() < 1e-9);
        // A declared stall above clean is clamped, never a raised stall.
        assert_eq!(super::stall_speed_in_config(100.0, Some(80.0), &[(0.5, 130.0)], 0.5), 100.0);
        for id in ["cessna_172sp", "boeing_737_800", "cirrus_vision_sf50", "citation_x", "md82"] {
            let p = super::load(id).expect(id);
            assert!(
                super::detent_stall_points(p.flaps.as_ref()).is_empty(),
                "{id} declares nothing and keeps the line"
            );
        }

        let _g = FLOOR_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        super::publish_speed_floor(Some(175.0));
        super::publish_approach_floor(Some(125.0), Some(175.0));
        super::publish_stall_curve(Some(175.0), Some(125.0), a330.flaps.as_ref());
        assert_eq!(super::config_floor_kts(0.0), Some(210.0));
        assert_eq!(super::config_floor_kts(0.5), Some(167.0), "the line said 180");
        assert_eq!(super::config_floor_kts(1.0), Some(150.0));
        super::publish_stall_curve(Some(175.0), Some(125.0), None);
        assert_eq!(super::stall_curve(), None);
        assert_eq!(super::config_floor_kts(0.5), Some(180.0), "no table, the line");
        super::publish_speed_floor(None);
        super::publish_approach_floor(None, None);
    }

    /// The A330's .acf: slats 0 / 0.70 / 0.87 / 1 / 1 of 8 degrees, flaps
    /// 0 / 8 / 14 / 22 / 32 at CL 1.1, Vs 175, Vso 125. The profile's table
    /// is what this reads; a flap-only wing (the 172: slat_inc 0) reads
    /// nothing and keeps the line.
    #[test]
    fn the_acf_gives_a_slatted_wings_stall_table() {
        let mut a330 = String::from(
            "P acf/_Vs_kts 175.0\nP acf/_Vso_kts 125.0\nP acf/_flap_detents 4\n\
             P acf/_slat1_inc 8.0\nP acf/_flap1_cl 1.1\n",
        );
        for (i, (f, s)) in [(0.0, 0.0), (8.0, 0.695652187), (14.0, 0.869565189), (22.0, 1.0), (32.0, 1.0)]
            .iter()
            .enumerate()
        {
            a330.push_str(&format!("P acf/_flap1_dn/{i} {f}\nP acf/_slat1_dn/{i} {s}\n"));
        }
        assert_eq!(
            super::detent_stall_from_acf(&a330),
            Some(vec![147.0, 139.0, 131.0, 125.0])
        );
        let c172 = "P acf/_Vs_kts 48\nP acf/_Vso_kts 40\nP acf/_flap_detents 3\n\
                    P acf/_slat1_inc 0\nP acf/_flap1_cl 0.86\n\
                    P acf/_flap1_dn/1 10\nP acf/_flap1_dn/2 20\nP acf/_flap1_dn/3 30\n\
                    P acf/_slat1_dn/1 1\nP acf/_slat1_dn/2 1\nP acf/_slat1_dn/3 1\n";
        assert_eq!(super::detent_stall_from_acf(c172), None);
    }

    /// The 737 (Vs 139, Vso 114): flaps up floors at 167, flaps 40 at 137,
    /// flaps 5 (0.375) at 156. A wing with no Vso floors flat at the clean
    /// number; no stall speeds at all floors nothing.
    #[test]
    fn the_floor_follows_the_flaps() {
        let _g = FLOOR_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        super::publish_stall_curve(None, None, None);
        super::publish_speed_floor(Some(139.0));
        super::publish_approach_floor(Some(114.0), Some(139.0));
        assert_eq!(super::config_floor_kts(0.0), Some(167.0));
        assert_eq!(super::config_floor_kts(1.0), Some(137.0));
        assert_eq!(super::config_floor_kts(0.375), Some(156.0));
        assert_eq!(
            super::config_floor_kts(f64::NAN),
            Some(167.0),
            "an unreadable flap is the clean floor"
        );
        super::publish_approach_floor(None, Some(139.0));
        assert_eq!(super::config_floor_kts(1.0), Some(167.0));
        super::publish_speed_floor(None);
        super::publish_approach_floor(None, None);
        assert_eq!(super::config_floor_kts(0.5), None);
        super::publish_approach_thrust_min(Some(0.3));
        assert_eq!(super::approach_thrust_min(), Some(0.3));
        super::publish_approach_thrust_min(Some(7.0));
        assert_eq!(
            super::approach_thrust_min(),
            None,
            "a lever ratio outside 0..1 is refused"
        );
    }

    // The probe asks the sim about every name a profile relies on. The RV-10's
    // xlua fuel selector is the case: stock names would have probed fine and
    // the real ones were never asked (2026-09-11).
    #[test]
    fn a_profiles_datarefs_and_commands_are_all_found_by_the_walk() {
        let p = super::load("vans_rv10").expect("RV-10 profile");
        let refs = super::profile_references(&p);
        for must in [
            "laminar/rv10/switch/fuel_tank_pos",
            "laminar/rv10/tank_sel_left",
            "sim/electrical/battery_1_on",
            "sim/cockpit2/switches/taxi_light_on",
            "sim/cockpit2/engine/actuators/prop_rotation_speed_rad_sec_all",
        ] {
            assert!(refs.iter().any(|r| r == must), "missing {must} in {refs:?}");
        }
        assert!(
            !refs.iter().any(|r| r.contains("checklists/")),
            "file paths are not datarefs"
        );
        assert!(
            !refs.iter().any(|r| r.contains('[')),
            "indices dropped: {refs:?}"
        );
        assert!(
            !refs.iter().any(|r| r.contains(' ')),
            "prose is not a dataref: {refs:?}"
        );
        let n = refs.len();
        assert_eq!(
            refs.iter().collect::<std::collections::BTreeSet<_>>().len(),
            n,
            "deduplicated"
        );

        let have: std::collections::HashSet<String> = refs
            .iter()
            .filter(|r| r.starts_with("sim/"))
            .cloned()
            .collect();
        let cmds: std::collections::HashSet<String> = ["laminar/rv10/tank_sel_left".to_string()]
            .into_iter()
            .collect();
        let missing = super::missing_references(&refs, &have, &cmds);
        assert!(
            missing.iter().any(|m| m == "laminar/rv10/tank_sel_right"),
            "{missing:?}"
        );
        assert!(
            !missing.iter().any(|m| m == "laminar/rv10/tank_sel_left"),
            "a command counts"
        );
        assert!(
            !missing.iter().any(|m| m.starts_with("sim/")),
            "{missing:?}"
        );

        assert_eq!(super::probe_summary(&[], 40), None);
        let s = super::probe_summary(&["laminar/rv10/tank_sel_left".into()], 40).unwrap();
        assert!(s.contains("1 of 40") && s.contains("tank sel left"), "{s}");
        let many: Vec<String> = (0..5).map(|i| format!("sim/x/name_{i}")).collect();
        assert!(super::probe_summary(&many, 40)
            .unwrap()
            .contains("and 2 more"));
    }

    #[test]
    fn the_start_gate_scales_with_the_system_voltage() {
        super::publish_bus_volts(None);
        assert!(
            (super::min_start_volts() - 23.0).abs() < 1e-9,
            "24 volt default"
        );
        super::publish_bus_volts(Some(12.0));
        assert!(
            (super::min_start_volts() - 11.5).abs() < 1e-9,
            "a healthy 12 is not low"
        );
        super::publish_bus_volts(None);
    }
