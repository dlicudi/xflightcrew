    /// An IFR clearance's altitude caps the climb, below the plan's cruise,
    /// until released; a pilot's own altitude outranks it and the ground
    /// ahead outranks both.
    #[test]
    fn the_cleared_altitude_caps_the_climb() {
        assert_eq!(cap_at_clearance(35_000.0, Some(5_000.0), false, None), 5_000.0);
        assert_eq!(cap_at_clearance(4_000.0, Some(5_000.0), false, None), 4_000.0, "below it: the plan");
        assert_eq!(cap_at_clearance(35_000.0, None, false, None), 35_000.0, "released");
        assert_eq!(cap_at_clearance(9_000.0, Some(5_000.0), true, None), 9_000.0, "the pilot's");
        assert_eq!(
            cap_at_clearance(35_000.0, Some(5_000.0), false, Some(7_200.0)),
            7_200.0,
            "the terrain floor wins"
        );
    }

    fn fix_leg(ident: &str, lat: f64, lon: f64) -> ProcLeg {
        ProcLeg {
            seq: 0,
            transition: String::new(),
            leg_type: "TF".into(),
            fix_ident: ident.into(),
            fix_region: String::new(),
            fix: Some(crate::nav::Fix { ident: ident.into(), lat, lon }),
            course_mag: None,
            dist_nm: None,
            turn: None,
            alt_desc: ' ',
            alt1_ft: Some(20_000.0),
            alt2_ft: None,
            is_faf: false,
            is_map: false,
            vpa_deg: None,
            navaid_ident: String::new(),
            navaid_region: String::new(),
            navaid: None,
            path: Default::default(),
        }
    }

    /// #79: an in-flight re-pick replaces the route from the STAR's entry on,
    /// the leg flying keeps its index, and a splice the aircraft has flown
    /// past (or computed against another route) is refused, never applied.
    #[test]
    fn a_splice_replaces_the_route_from_its_entry_while_the_entry_is_ahead() {
        let legs: Vec<ProcLeg> = ["A", "B", "BAMEK", "OLD1", "OLD2"]
            .iter()
            .enumerate()
            .map(|(i, f)| fix_leg(f, 45.0 + i as f64 * 0.1, 10.0))
            .collect();
        let s = Splice {
            from: 2,
            entry: "BAMEK".into(),
            legs: ["BAMEK", "NEW1", "NEW2", "NEW3"].iter().map(|f| fix_leg(f, 44.8, 11.0)).collect(),
            constraints: Vec::new(),
            why: "wind".into(),
        };
        for idx in [0, 1, 2] {
            let out = apply_splice(&legs, idx, &s).unwrap();
            let ids: Vec<&str> = out.iter().map(|l| l.fix_ident.as_str()).collect();
            assert_eq!(ids, ["A", "B", "BAMEK", "NEW1", "NEW2", "NEW3"]);
            assert_eq!(out[idx].fix_ident, legs[idx].fix_ident, "the leg flying keeps its index");
        }
        assert_eq!(apply_splice(&legs, 3, &s).unwrap_err(), "past_entry");
        let moved = Splice { from: 1, ..s.clone() };
        assert_eq!(apply_splice(&legs, 0, &moved).unwrap_err(), "route_changed");
        let off_end = Splice { from: 9, ..s.clone() };
        assert_eq!(apply_splice(&legs, 0, &off_end).unwrap_err(), "route_changed");
        let headless = Splice { legs: vec![fix_leg("NEW1", 44.8, 11.0)], ..s };
        assert_eq!(apply_splice(&legs, 0, &headless).unwrap_err(), "no_entry");
    }

    /// LSZS 03 hand-over at the MAP (1.98 nm, 6,598 ft, threshold 5,602):
    /// the talk-down's path runs from here to 50 ft over the threshold, so it
    /// starts "on glidepath" on the steep segment instead of 300 ft high on 3.
    #[test]
    fn a_map_handover_talks_down_the_path_actually_flown() {
        let rwy = crate::autoland::Runway {
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
        let (lat, lon) = (46.5, 9.850666667);
        let path = super::map_handover_path(&rwy, lat, lon, 6598.0);
        assert!((path.ft_per_nm - 478.0).abs() < 10.0, "{}", path.ft_per_nm);
        let d = crate::approach_guide::deviations(&path, lat, lon, 6598.0);
        assert!(d.vdev_ft.abs() < 60.0, "on glidepath at the MAP: {}", d.vdev_ft);
        assert!(crate::approach_guide::callout(&d).unwrap().contains("on glidepath"));
        assert!(!super::MAP_AUTOLAND, "the steep autoland stays off until proven");
    }

    #[test]
    fn a_map_on_the_runway_is_a_landing_not_a_visual_segment() {
        let mut leg = super::ProcLeg {
            seq: 30,
            transition: String::new(),
            leg_type: "CF".into(),
            fix_ident: "RW12".into(),
            fix_region: String::new(),
            fix: None,
            course_mag: None,
            dist_nm: None,
            turn: None,
            alt_desc: ' ',
            alt1_ft: None,
            alt2_ft: None,
            is_faf: false,
            is_map: true,
            vpa_deg: None,
            navaid_ident: String::new(),
            navaid_region: String::new(),
            navaid: None,
            path: Default::default(),
        };
        assert!(!super::map_short_of_runway(&leg), "LIPE I12: MAP on RW12");
        leg.fix_ident = "MA03".into();
        assert!(super::map_short_of_runway(&leg), "LSZS: MAP short of the runway");
        leg.is_map = false;
        assert!(!super::map_short_of_runway(&leg));
    }

    /// LSZS RNAV 03: FAF ZS702 at 11,800, MAP ZS705 at 6,598 on a 4.49
    /// degree path. The final is flown on the path with its rate fed forward,
    /// and handed over at the MAP, not the FAF.
    #[test]
    fn a_map_final_is_flown_on_its_path_and_handed_over_at_the_map() {
        let mut faf = fix_leg("ZS702", 46.357, 9.688);
        faf.is_faf = true;
        let mut map = fix_leg("ZS705", 46.500, 9.851);
        map.is_map = true;
        map.vpa_deg = Some(4.49);
        map.alt1_ft = Some(6598.0);
        let legs = vec![fix_leg("ZS701", 46.327, 9.576), faf, map.clone()];
        assert_eq!(super::handoff_index(&legs), Some(2), "at the MAP, not the FAF");
        // 5 nm out: 6,598 + 5 x 477 = 8,983 ft, and at 100 kt ~795 fpm down.
        let (alt, ff) = super::map_path(&map, Some(5.0), 100.0).unwrap();
        assert!((alt - 8983.0).abs() < 15.0, "{alt}");
        assert!((ff + 795.0).abs() < 10.0, "{ff}");
        assert_eq!(super::map_path(&legs[1], Some(5.0), 100.0), None, "not the MAP leg");
    }

    /// LOWI MOGT3H (2026-09-24): 7,850 at WI506, about nine miles from the
    /// runway. At cruise speed the SF50 made 375 ft a mile (1,500 fpm at
    /// 240 kt); at Vy it makes the minimum.
    #[test]
    fn a_departure_minimum_is_watched_and_an_unmakeable_one_hands_back() {
        let mins = vec![
            crate::nav::ClimbMinimum { fix: "WI505".into(), min_ft: 4000.0 },
            crate::nav::ClimbMinimum { fix: "WI506".into(), min_ft: 7850.0 },
        ];
        let legs = vec![
            fix_leg("WI505", 47.252, 11.269),
            fix_leg("WI506", 47.290, 11.139),
            fix_leg("WI507", 47.306, 11.086),
        ];
        // Past WI505 at 4,500: WI506 is the one ahead and unmet.
        assert_eq!(super::unmet_minimum(&mins, &legs, 1, 4500.0), Some((1, 7850.0)));
        assert_eq!(super::unmet_minimum(&mins, &legs, 1, 8000.0), None, "met");
        assert_eq!(super::unmet_minimum(&mins, &legs, 1, 7849.0), None, "levelled at it is met");
        assert_eq!(super::unmet_minimum(&mins, &legs, 1, 7700.0), Some((1, 7850.0)));
        // Behind us is not ahead.
        assert_eq!(super::unmet_minimum(&mins, &legs, 2, 4500.0), None);
        let d = super::distance_to_leg_nm(47.252, 11.269, &legs, 1, 1);
        assert!((d - 5.8).abs() < 0.3, "WI505 to WI506 ~5.8 nm: {d}");
        // Cruise-speed climb, 6 nm out at 4,500: short, said.
        assert!(matches!(
            super::watch_minimum(4500.0, 1500.0, 240.0, 6.0, 7850.0),
            super::MinimumWatch::Short { .. }
        ));
        // Vy climb: 2,000 fpm at 130 kt makes it.
        assert_eq!(super::watch_minimum(4500.0, 2000.0, 130.0, 6.0, 7850.0), super::MinimumWatch::OnTrack);
        // Two miles out and 1,400 short: unable.
        assert!(matches!(
            super::watch_minimum(6000.0, 1200.0, 200.0, 2.0, 7850.0),
            super::MinimumWatch::Unable { .. }
        ));
        // A descent is not a climb that will make it.
        assert!(matches!(
            super::watch_minimum(6000.0, -500.0, 150.0, 2.0, 7850.0),
            super::MinimumWatch::Unable { .. }
        ));
    }

    /// EGPH-EGTE, 2026-09-21: 2,000 fpm down at idle, 258 kt through ten
    /// thousand against 250, 278 at the last fix. Since #156 the descent
    /// gives way to the speed first (`speed_protected_vs`) and the brake is
    /// for the height that leaves (or a runaway past 8 kt).
    #[test]
    fn a_descent_gives_way_to_its_speed_and_the_brake_is_for_height() {
        // On the target and steady: the whole descent. Three over: half.
        // Six over: level. Building at 1 kt/s from two over is five in
        // three seconds: already mostly held back.
        assert_eq!(speed_protected_vs(-2000.0, Some(0.0), 0.0), -2000.0);
        assert_eq!(speed_protected_vs(-2000.0, Some(-3.0), 0.0), -2000.0);
        assert!((speed_protected_vs(-2000.0, Some(3.0), 0.0) + 1000.0).abs() < 1.0);
        assert_eq!(speed_protected_vs(-2000.0, Some(6.0), 0.0), 0.0);
        assert!(speed_protected_vs(-2000.0, Some(2.0), 1.0) > -400.0);
        // Slowing toward the target lets the descent back early.
        assert!(speed_protected_vs(-2000.0, Some(4.0), -1.0) < -1600.0);
        // Climbs, level and no target pass through.
        assert_eq!(speed_protected_vs(1500.0, Some(20.0), 0.0), 1500.0);
        assert_eq!(speed_protected_vs(-2000.0, None, 0.0), -2000.0);
        // The brake: never for 8 kt over any more, 9 is a runaway.
        assert!(!descent_drag(false, f64::INFINITY, 258.0, Some(250.0), -2000.0, 0.0));
        assert!(descent_drag(false, f64::INFINITY, 259.0, Some(250.0), -2000.0, 0.0));
        // High on the level commanded with the speed at the target: out...
        assert!(!descent_drag(false, f64::INFINITY, 250.5, Some(250.0), -2000.0, 350.0));
        assert!(descent_drag(false, f64::INFINITY, 250.5, Some(250.0), -2000.0, 450.0));
        // ...not when slow (the protection lets the whole rate through), not
        // for a level or a climb, not inside its hold.
        assert!(!descent_drag(false, f64::INFINITY, 244.0, Some(250.0), -2000.0, 900.0));
        assert!(!descent_drag(false, f64::INFINITY, 250.5, Some(250.0), 0.0, 900.0));
        assert!(!descent_drag(false, 5.0, 250.5, Some(250.0), -2000.0, 900.0));
        // Out, it stays out until back on the level with the speed in...
        assert!(descent_drag(true, f64::INFINITY, 251.0, Some(250.0), -2000.0, 300.0));
        assert!(!descent_drag(true, f64::INFINITY, 251.0, Some(250.0), -2000.0, 100.0));
        assert!(descent_drag(true, f64::INFINITY, 255.0, Some(250.0), -2000.0, 100.0));
        // ...or slow, or a climb; and through a level-off still fast (#126).
        assert!(!descent_drag(true, f64::INFINITY, 245.0, Some(250.0), -2000.0, 600.0));
        assert!(!descent_drag(true, f64::INFINITY, 278.0, Some(250.0), 800.0, 0.0));
        assert!(descent_drag(true, f64::INFINITY, 278.0, Some(250.0), 0.0, 0.0));
        assert!(!descent_drag(false, f64::INFINITY, 278.0, None, -2000.0, 900.0));
    }

    /// EGKK 26L, 2026-09-27 (sortie 1790510400-full_auto, stock 737): out at
    /// 218 kt against 210.25 and 2,000 fpm down, kept out through gear down
    /// and flaps 10 to 40. The speed rule still asks for it; the
    /// configuration says no, and the shallower rate is what is left.
    #[test]
    fn the_speedbrake_is_never_out_with_landing_flap_or_gear() {
        // The 737's detents, as the profile declares them.
        let b738: Vec<(f64, Option<f64>)> = [0.0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1.0]
            .iter()
            .map(|r| (*r, None))
            .collect();
        // The speed rule, alone, wanted it out at the sortie's numbers...
        assert!(descent_drag(false, f64::INFINITY, 218.36, Some(210.25), -2000.0, 0.0));
        // ...clean it may be, and flaps 1 is still the clean descent...
        assert_eq!(drag_barred(0.0, false, &b738), None);
        assert_eq!(drag_barred(0.125, false, &b738), None);
        // ...flaps 10 (the sortie's 0.5), 5, 40 and the gear say in.
        assert_eq!(drag_barred(0.5, false, &b738), Some("flaps"));
        assert_eq!(drag_barred(0.375, false, &b738), Some("flaps"));
        assert_eq!(drag_barred(1.0, false, &b738), Some("flaps"));
        assert_eq!(drag_barred(0.0, true, &b738), Some("gear"));
        assert_eq!(drag_barred(1.0, true, &b738), Some("gear"));
        // No detents declared: any flap at all.
        assert_eq!(drag_barred(0.1, false, &[]), Some("flaps"));
        assert_eq!(drag_barred(0.0, false, &[]), None);
        // The 5 s re-deployment at full flap: 153.6 kt against 145 and
        // 968 fpm down is out by speed, in by configuration.
        assert!(descent_drag(false, f64::INFINITY, 153.56, Some(145.0), -967.7, 0.0));
        assert_eq!(drag_barred(1.0, true, &b738), Some("gear"));
        // A runaway still levels the descent with the brake barred.
        assert_eq!(speed_protected_vs(-2000.0, Some(25.0), 0.0), 0.0);
    }

    /// #104 replayed from the recorded sortie itself (EGKK 26L, stock 737,
    /// `logs/sorties/1790510400-full_auto.jsonl`; not in the repo, so
    /// ignored): every `state` tick while the flown speedbrake was out,
    /// judged by `drag_barred` with the 737 profile's own detents and the
    /// gear from the `map_config` records. As flown it stayed out 57 s with
    /// gear down and flap past the first detent and was put out again at
    /// flaps 40; by the rule it goes in on the first tick the gear or flaps
    /// say so and the re-deployment is refused.
    /// `XVL_SORTIE_DIR` overrides where the sortie file is looked for.
    /// `cargo test --release -- --ignored real_sortie_speedbrake_barred --nocapture`
    #[test]
    #[ignore] // needs the recorded sortie; run with --ignored
    fn real_sortie_speedbrake_barred_by_gear_and_flap() {
        const SORTIE: &str = "1790510400-full_auto.jsonl";
        let dirs = [
            std::env::var("XVL_SORTIE_DIR").unwrap_or_default(),
            format!("{}/logs/sorties", env!("CARGO_MANIFEST_DIR")),
            format!("{}/GitHub/xvoicelayer/logs/sorties", std::env::var("HOME").unwrap_or_default()),
        ];
        let Some(text) = dirs
            .iter()
            .filter(|d| !d.is_empty())
            .find_map(|d| std::fs::read_to_string(format!("{d}/{SORTIE}")).ok())
        else {
            return;
        };
        let t0 = 1_790_510_400.0;
        let recs: Vec<serde_json::Value> =
            text.lines().filter_map(|l| serde_json::from_str(l).ok()).collect();
        let limits: Vec<(f64, Option<f64>)> = crate::aircraft::load("boeing_737_800")
            .and_then(|p| p.flaps)
            .expect("the 737's flaps")
            .detents
            .iter()
            .map(|d| (d.ratio, d.vfe_kts))
            .collect();
        let t = |r: &serde_json::Value| r["t"].as_f64().unwrap_or(0.0) - t0;
        let gear_down_at = recs
            .iter()
            .filter(|r| r["event"] == "map_config" && r["gear"] == true)
            .map(t)
            .fold(f64::INFINITY, f64::min);
        // The brake as flown: (time, out).
        let brake: Vec<(f64, bool)> = recs
            .iter()
            .filter(|r| r["kind"] == "procedure" && r["event"] == "speedbrake")
            .map(|r| (t(r), r["out"] == true))
            .collect();
        let out_at = |at: f64| brake.iter().filter(|(bt, _)| *bt <= at).last().is_some_and(|b| b.1);
        let (mut barred_out_s, mut first_barred, mut last_t) = (0.0, None, None::<f64>);
        for r in recs.iter().filter(|r| r["kind"] == "state") {
            let (at, flap) = (t(r), r["flap"].as_f64().unwrap_or(0.0));
            let why = drag_barred(flap, at >= gear_down_at, &limits);
            if out_at(at) && why.is_some() {
                if let Some(prev) = last_t {
                    barred_out_s += at - prev;
                }
                first_barred.get_or_insert((at, why.unwrap(), flap));
                last_t = Some(at);
            } else {
                last_t = None;
            }
        }
        let stowed = brake.iter().find(|(bt, out)| *bt > gear_down_at && !out).map(|b| b.0);
        let redeployed: Vec<f64> =
            brake.iter().filter(|(bt, out)| *bt > gear_down_at && *out).map(|b| b.0).collect();
        println!(
            "gear down +{gear_down_at:.1}; as flown out while barred {barred_out_s:.0} s, first barred \
             tick {first_barred:?}, stowed +{stowed:?}, put out again at {redeployed:?}"
        );
        let (first_at, _, _) = first_barred.expect("the brake was out with gear or flap");
        assert!(barred_out_s > 30.0, "{barred_out_s}");
        // The rule stows it at the first barred tick, within a tick of the gear.
        assert!(first_at - gear_down_at < 2.0, "+{first_at:.1}");
        assert!(stowed.is_some_and(|s| s > first_at + 30.0), "as flown it stayed out");
        // Every re-deployment after the gear came down is refused.
        for at in &redeployed {
            let flap = recs
                .iter()
                .filter(|r| r["kind"] == "state" && t(r) <= *at)
                .last()
                .and_then(|r| r["flap"].as_f64())
                .unwrap_or(0.0);
            assert!(drag_barred(flap, true, &limits).is_some(), "+{at:.1} at flap {flap}");
        }
        assert!(!redeployed.is_empty(), "the sortie's re-deployment at flaps 40");
    }

    /// EGKK-EGBB, 2026-09-27 (sortie 1790541304-full_auto, #126), the
    /// approach at 210.25 kt: (seconds after +2600, the state records' IAS,
    /// the VS law's command rebuilt from the leg records' altitude and
    /// target). The old rule put the brake out and in three times, once in
    /// under 2 s at a level-off 8 kt fast, and said every change.
    #[rustfmt::skip]
    const EGBB_APPROACH: &[(f64, f64, f64)] = &[
            (5.9, 210.4, 10.0), (7.5, 210.1, 10.0), (9.3, 209.6, -2000.0), (11.0, 209.2, -2000.0),
            (12.7, 209.4, -2000.0), (14.4, 210.3, -2000.0), (16.1, 211.7, -2000.0), (17.9, 213.6, -2000.0),
            (19.3, 215.6, -2000.0), (20.9, 217.0, -2000.0), (22.4, 218.6, -2000.0), (24.1, 219.7, -2000.0),
            (25.8, 220.3, -2000.0), (27.5, 220.9, -2000.0), (29.2, 221.6, -2000.0), (30.9, 222.4, -2000.0),
            (32.6, 222.9, -2000.0), (34.4, 223.2, -2000.0), (36.1, 223.3, -2000.0), (37.9, 223.5, -2000.0),
            (39.6, 223.9, -2000.0), (41.3, 223.8, -2000.0), (43.0, 223.5, -2000.0), (44.8, 223.2, -2000.0),
            (46.4, 223.0, -1966.0), (48.1, 222.5, -1910.0), (49.8, 221.9, -1919.0), (51.5, 222.0, -2000.0),
            (53.2, 222.4, -2000.0), (54.9, 222.6, -2000.0), (56.5, 222.5, -2000.0), (58.1, 222.2, -2000.0),
            (59.8, 222.1, -2000.0), (61.5, 222.3, -2000.0), (63.0, 222.3, -1983.0), (64.7, 222.3, -1956.0),
            (66.4, 222.5, -1732.0), (68.1, 222.5, -1855.0), (69.3, 222.4, -1743.0), (71.0, 222.3, -1760.0),
            (72.7, 222.1, -1742.0), (74.4, 222.0, -1724.0), (76.1, 222.0, -2000.0), (77.8, 221.9, -1542.0),
            (79.6, 221.9, -1380.0), (81.3, 222.4, -1348.0), (83.0, 222.4, -1093.0), (84.6, 221.9, -847.0),
            (86.3, 220.8, -674.0), (88.0, 218.8, -584.0), (89.7, 216.4, -534.0), (91.4, 214.3, -735.0),
            (93.2, 212.1, -872.0), (94.6, 210.3, -946.0), (96.2, 209.0, -1080.0), (97.9, 207.8, -1814.0),
            (99.6, 207.8, -1969.0), (101.2, 209.1, -2000.0), (102.9, 211.3, -2000.0), (104.6, 213.1, -1895.0),
            (106.3, 214.6, -1466.0), (107.9, 215.6, -1208.0), (109.5, 215.9, -930.0), (111.2, 216.1, -536.0),
            (112.8, 215.7, -582.0), (114.5, 214.8, -323.0), (116.2, 213.1, -279.0), (117.9, 211.3, -354.0),
            (119.6, 209.2, -392.0), (121.0, 207.8, -524.0), (122.6, 207.4, -816.0), (124.3, 207.0, -1059.0),
            (125.9, 206.9, -1221.0), (127.3, 207.7, -1362.0), (129.0, 209.1, -1805.0), (130.7, 212.0, -1446.0),
            (132.5, 215.1, -1040.0), (134.1, 217.2, -982.0), (135.5, 218.3, -663.0), (137.2, 218.5, -305.0),
            (138.8, 217.7, -38.0), (140.5, 216.7, 164.0), (142.2, 215.7, 209.0), (144.2, 214.5, 213.0),
            (145.9, 213.4, 187.0), (147.3, 211.9, 145.0), (148.8, 210.6, 120.0), (150.5, 208.9, 80.0),
            (152.1, 206.9, 69.0), (153.8, 205.8, 67.0), (155.5, 205.8, 74.0), (157.2, 206.7, 76.0),
            (159.0, 208.2, 78.0), (339.6, 213.3, 28.0), (341.3, 212.7, -1131.0), (342.8, 211.9, -1326.0),
            (344.3, 211.1, -1384.0), (346.0, 210.3, -1397.0), (347.5, 209.7, -1550.0), (349.2, 209.0, -1776.0),
            (350.7, 208.9, -2000.0), (352.4, 209.2, -2000.0), (354.1, 210.2, -2000.0), (355.8, 212.4, -2000.0),
            (357.5, 214.1, -2000.0), (359.2, 215.5, -2000.0), (361.0, 216.9, -2000.0), (362.7, 218.2, -2000.0),
            (364.4, 219.7, -2000.0), (366.1, 220.8, -2000.0), (367.8, 222.1, -2000.0), (369.4, 222.9, -2000.0),
            (371.2, 223.4, -1811.0), (372.8, 223.3, -1770.0), (374.5, 223.6, -1839.0), (376.2, 223.4, -1855.0),
            (377.9, 222.9, -1822.0), (379.6, 222.7, -1740.0), (381.0, 222.2, -1961.0), (382.7, 222.3, -2000.0),
            (384.4, 222.8, -1988.0), (386.1, 223.2, -1722.0), (387.8, 223.1, -1990.0), (389.4, 223.5, -2000.0),
            (391.1, 224.0, -1852.0), (392.9, 223.9, -1864.0), (394.6, 223.6, -1773.0), (396.3, 223.6, -1889.0),
            (397.9, 223.0, -2000.0), (399.6, 222.5, -1897.0), (401.3, 222.0, -1898.0), (403.0, 222.0, -1943.0),
            (404.7, 223.1, -1109.0), (406.5, 222.4, -271.0), (408.2, 221.6, -184.0), (409.6, 221.5, -184.0),
    ];

    /// The recorded approach through the old rule and the new: every change
    /// `(t, out)`, and which are said.
    fn replay_drag(new: bool) -> (Vec<(f64, bool)>, Vec<(f64, bool)>) {
        const TARGET: f64 = 210.25;
        let (mut out, mut since) = (false, f64::NEG_INFINITY);
        let mut voice = DragVoice::default();
        let (mut changes, mut said) = (Vec::new(), Vec::new());
        for &(t, ias, vs) in EGBB_APPROACH {
            let want = if new {
                descent_drag(out, t - since, ias, Some(TARGET), vs, 0.0)
            } else {
                // The rule as flown: the descent gate, then 8 out / 3 in.
                vs <= -DRAG_MIN_DESCENT_FPM
                    && if out { ias - TARGET > DRAG_IN_KTS } else { ias - TARGET > DRAG_OUT_KTS }
            };
            if want != out {
                out = want;
                since = t;
                changes.push((t, out));
                if !new || voice.say(out, t) {
                    said.push((t, out));
                }
            }
        }
        (changes, said)
    }

    /// #126: out and in around the approach speed, every change spoken.
    #[test]
    fn the_speedbrake_holds_through_a_level_off_and_is_said_once() {
        let (old, old_said) = replay_drag(false);
        // As flown: three extensions, one stowed within 2 s by the gate at
        // a level-off still 8 kt fast, and all six said.
        assert_eq!(old.iter().filter(|c| c.1).count(), 3, "{old:?}");
        assert!(old.windows(2).any(|w| w[0].1 && !w[1].1 && w[1].0 - w[0].0 < 5.0), "{old:?}");
        assert_eq!(old_said.len(), 6);
        let (new, said) = replay_drag(true);
        // Every state is held at least the minimum.
        for w in new.windows(2) {
            assert!(w[1].0 - w[0].0 >= DRAG_MIN_HOLD_S, "{new:?}");
        }
        // Never stowed while still fast and not climbing: the level-offs at
        // +2739 and +3004 keep it out.
        for &(t, out) in &new {
            if !out {
                let &(_, ias, vs) = EGBB_APPROACH.iter().find(|r| r.0 == t).unwrap();
                assert!(ias - 210.25 <= DRAG_IN_KTS || vs >= DRAG_KEEP_BELOW_FPM, "in at {t}: {ias} {vs}");
            }
        }
        assert_eq!(new.last().map(|c| c.1), Some(true), "still out 12 kt fast at the handoff");
        // Said: an extension and its stow, then silence for the quick
        // re-extension, then the last extension 4 minutes on.
        assert!(said.len() <= 3, "{said:?}");
        assert!(said.len() < new.len(), "{said:?} of {new:?}");
        for w in said.windows(2) {
            assert!(!w[1].1 || w[1].0 - w[0].0 >= DRAG_SPEAK_GAP_S, "{said:?}");
        }
    }

    /// +2295 and +3001 on EGKK-EGBB: the VS mode dropped as the autopilot
    /// captured the dial, and the follower re-engaged VS over it.
    #[test]
    fn a_capture_of_the_dial_is_not_a_dropped_vs_mode() {
        // HON: 9,202 ft, path 9,068, dial 9,000, -1,900 commanded.
        assert!(vs_mode_capturing(9_202.0, 9_068.0, 9_000.0, -1_900.0));
        // FI33: 2,046 ft, path 1,895, dial 1,800.
        assert!(vs_mode_capturing(2_046.0, 1_895.0, 1_800.0, -1_900.0));
        // Far from the dial, or moving away from it: a dropped mode.
        assert!(!vs_mode_capturing(9_900.0, 9_700.0, 9_000.0, -1_900.0));
        assert!(!vs_mode_capturing(9_202.0, 9_400.0, 9_000.0, 1_500.0));
        // Near the target, as before.
        assert!(vs_mode_capturing(5_050.0, 5_000.0, 3_000.0, -1_000.0));
    }

    #[test]
    fn a_stowed_speedbrake_is_said_only_when_its_extension_was() {
        let mut v = DragVoice::default();
        assert!(v.say(true, 0.0));
        assert!(v.say(false, 60.0));
        assert!(!v.say(true, 100.0), "40 s after the last call");
        assert!(!v.say(false, 130.0), "its extension was not said");
        assert!(v.say(true, 300.0));
    }

    /// #126: OSKOT's 210 on EGKK-EGBB (2026-09-27, +2370 to +2440), the
    /// leg records' distance with the state records' IAS and groundspeed
    /// (every other tick). Judged afresh each tick, the limit came and went
    /// as the aircraft slowed at the planned rate; begun, it holds.
    #[test]
    fn a_star_speed_limit_once_begun_holds() {
        #[rustfmt::skip]
        let recorded: &[(f64, f64, f64)] = &[
            (10.19, 248.8, 261.0), (10.10, 248.8, 261.0), (10.03, 248.1, 260.0), (9.94, 247.6, 259.0),
            (9.86, 247.4, 259.0), (9.78, 247.8, 260.0), (9.70, 248.5, 261.0), (9.62, 249.5, 262.0),
            (9.53, 250.6, 263.0), (9.45, 251.8, 265.0), (9.37, 252.7, 266.0), (9.29, 253.8, 267.0),
            (9.21, 254.7, 268.0), (9.12, 255.6, 268.0), (9.04, 256.2, 269.0), (8.96, 256.5, 269.0),
            (8.86, 256.6, 269.0), (8.77, 256.6, 269.0), (8.68, 256.2, 268.0), (8.60, 255.7, 267.0),
            (8.52, 255.2, 267.0), (8.43, 254.7, 265.0), (8.34, 253.7, 264.0), (8.26, 252.9, 263.0),
            (8.18, 252.2, 262.0), (8.10, 251.8, 262.0), (8.01, 251.5, 261.0), (7.92, 251.4, 261.0),
            (7.84, 251.4, 261.0), (7.76, 251.0, 261.0), (7.68, 250.5, 261.0), (7.59, 249.9, 260.0),
            (7.51, 249.4, 259.0), (7.43, 248.8, 258.0), (7.34, 248.0, 257.0), (7.25, 247.0, 256.0),
            (7.17, 246.3, 255.0), (7.09, 245.8, 255.0), (7.01, 245.6, 255.0), (6.92, 245.9, 255.0),
            (6.84, 246.4, 256.0), (6.76, 247.1, 256.0), (6.68, 247.0, 256.0), (6.60, 246.3, 255.0),
            (6.51, 245.4, 254.0), (6.43, 244.5, 253.0), (6.35, 243.8, 252.0), (6.26, 242.9, 251.0),
            (6.19, 242.2, 250.0), (6.11, 241.5, 249.0), (6.03, 240.8, 248.0), (5.94, 240.0, 247.0),
            (5.86, 238.9, 246.0), (5.78, 238.1, 245.0), (5.70, 237.4, 245.0), (5.62, 237.0, 244.0),
            (5.54, 237.3, 245.0), (5.47, 237.8, 246.0), (5.39, 238.1, 246.0), (5.31, 237.9, 246.0),
            (5.23, 237.9, 246.0),
        ];
        let legs = vec![leg("HON", 1.0, 0.0), leg("OSKOT", 0.0, 0.0)];
        let cons = vec![crate::nav::StarConstraint {
            fix: "OSKOT".into(),
            min_ft: None,
            max_ft: None,
            max_kts: Some(210.0),
        }];
        let caps = |fresh: bool| {
            let mut begun = std::collections::HashSet::new();
            recorded
                .iter()
                .map(|&(d, ias, gs)| {
                    if fresh {
                        begun.clear();
                    }
                    arrival_speed_cap(&legs, &cons, 1, -d / 60.0, 0.0, ias, gs, &mut begun).map(|c| c.0)
                })
                .collect::<Vec<_>>()
        };
        let flips = |v: &[Option<f64>]| v.windows(2).filter(|w| w[0] != w[1]).count();
        let old = caps(true);
        assert!(flips(&old) >= 3, "the limit came and went: {old:?}");
        let new = caps(false);
        assert_eq!(flips(&new), 1, "{new:?}");
        assert_eq!(new.last(), Some(&Some(210.0)));
    }

    use super::*;

    #[test]
    fn missing_or_nonfinite_instruments_never_reach_the_laws() {
        for missing in 0..11 {
            let values = std::array::from_fn(|i| {
                if i == missing {
                    Err(anyhow::anyhow!("read failed"))
                } else {
                    Ok(100.0)
                }
            });
            assert!(measurements::<11>("procedure", values).is_none());
        }
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(measurements("procedure", [Ok(8000.0), Ok(invalid)]).is_none());
        }
        assert_eq!(
            measurements("procedure", [Ok(0.0), Ok(-100.0)]),
            Some([0.0, -100.0])
        );
    }

    // Inject failures at the actual write/read boundary, including a sim that
    // acknowledges PATCH but silently keeps its old dial value.
    struct DialIo {
        value: std::sync::Mutex<f64>,
        writes: std::sync::atomic::AtomicUsize,
        failure: std::sync::atomic::AtomicUsize,
    }

    #[async_trait::async_trait]
    impl crate::xplane::XPlane for DialIo {
        async fn get_float(&self, _: &str) -> anyhow::Result<f64> {
            if self.failure.load(Ordering::SeqCst) == 2 {
                anyhow::bail!("read failed");
            }
            Ok(*self.value.lock().unwrap())
        }
        async fn set_float(&self, _: &str, value: f64) -> anyhow::Result<()> {
            self.writes.fetch_add(1, Ordering::SeqCst);
            match self.failure.load(Ordering::SeqCst) {
                1 => anyhow::bail!("write failed"),
                3 => (),
                _ => *self.value.lock().unwrap() = value,
            }
            Ok(())
        }
        async fn set_int(&self, _: &str, _: i64) -> anyhow::Result<()> {
            unreachable!()
        }
        async fn run_command(&self, _: &str) -> anyhow::Result<()> {
            unreachable!()
        }
        async fn run_command_for(&self, _: &str, _: f64) -> anyhow::Result<()> {
            unreachable!()
        }
        async fn get_float_array(&self, _: &str) -> anyhow::Result<Vec<f64>> {
            unreachable!()
        }
        async fn set_int_array_elem(&self, _: &str, _: usize, _: i64) -> anyhow::Result<()> {
            unreachable!()
        }
        async fn set_float_array_elem(&self, _: &str, _: usize, _: f64) -> anyhow::Result<()> {
            unreachable!()
        }
    }

    /// LIPR 30 (2026-09-26): the dial written with the paced target every
    /// tick made a staircase; it is the level the movement ends at.
    #[test]
    fn the_altitude_dial_is_where_the_descent_ends_not_where_it_is_now() {
        // A descent from 2,100 toward a 1,500 constraint, paced: the target
        // walks down, the dial stays at 1,500 the whole way.
        for (want, alt) in [(1990.0, 2105.0), (1850.0, 2000.0), (1600.0, 1750.0)] {
            assert_eq!(super::dial_target_ft(want, alt, 1500.0, None, None), 1500.0);
        }
        // Held level before the top of descent: the level held.
        assert_eq!(super::dial_target_ft(4020.0, 4000.0, 1500.0, None, None), 4000.0);
        // A terrain floor above the constraint: never under the floor.
        assert_eq!(super::dial_target_ft(2340.0, 3000.0, 1500.0, Some(2340.0), None), 2400.0);
        // Climbing to a minimum above the plan's altitude: the minimum.
        assert_eq!(super::dial_target_ft(4000.0, 1200.0, 3000.0, None, None), 4000.0);
        // Down a published path: its bottom, once.
        assert_eq!(super::dial_target_ft(800.0, 900.0, 1500.0, None, Some(147.0)), 200.0);
    }

    #[test]
    fn a_minimum_above_the_cruise_is_not_the_followers_to_judge() {
        let m = |f: &str, a: f64| crate::nav::ClimbMinimum { fix: f.into(), min_ft: a };
        let (keep, above) = super::minimums_within_cruise(vec![m("RIM", 4000.0), m("D121Z", 9500.0)], 5000.0);
        assert_eq!(keep.len(), 1);
        assert_eq!(keep[0].fix, "RIM");
        assert_eq!(above[0].fix, "D121Z");
    }

    /// LIPR 2026-09-26: set the heading, leave it. One write per leg, a
    /// whole degree; small drift and a banked aircraft leave the knob alone.
    #[test]
    fn the_heading_is_set_once_per_leg_like_a_pilot_sets_it() {
        let mut h = HeadingHold::default();
        assert_eq!(h.decide(0, 127.4, 125.0, 0.0, 0.0), Some(127.0), "a new leg: set, whole degrees");
        for t in 1..60 {
            let wobble = 127.0 + (t as f64 * 0.7).sin() * 2.5;
            assert_eq!(h.decide(0, wobble, 127.0, 1.0, t as f64), None, "under 5 degrees: leave it");
        }
        assert_eq!(h.decide(0, 134.0, 127.0, 8.0, 70.0), None, "a correction waits for level wings");
        assert_eq!(h.decide(0, 134.0, 127.0, 1.0, 71.0), Some(134.0), "drifted 7, level: correct");
        assert_eq!(h.decide(0, 141.0, 134.0, 1.0, 80.0), None, "not twice inside 30 s");
        assert_eq!(h.decide(1, 250.0, 130.0, 0.0, 81.0), Some(155.0), "a large next leg starts with a 25 degree lead");
    }

    #[test]
    fn a_new_heading_against_the_bank_waits_for_the_roll_out() {
        let mut h = HeadingHold::default();
        // Banked 20 right, the new leg is 60 left: setting it now would send
        // the autopilot round the long way. Capture the present heading, wait
        // for level wings, then use a 25 degree left entry.
        assert_eq!(h.decide(3, 240.0, 300.0, 20.0, 0.0), Some(300.0));
        assert_eq!(h.decide(3, 240.0, 305.0, 2.0, 5.0), None, "not level yet");
        assert_eq!(h.decide(3, 240.0, 305.0, 0.5, 6.0), Some(280.0), "level: lead left");
        assert_eq!(h.decide(3, 240.0, 303.0, -4.0, 7.0), Some(240.0), "committed left: set the leg");
        // With the bank, it goes in at once.
        let mut h = HeadingHold::default();
        assert_eq!(h.decide(3, 340.0, 300.0, 20.0, 0.0), Some(325.0));
    }

    /// EGBB-EGTF 2026-10-01, sortie 1790844668: at TOBID the follower
    /// jumped the bug from about 190 to 160. The C172 banked right, flew the
    /// long way through north, and stood down after three `leg_stalled`s.
    #[test]
    fn a_wrong_way_enroute_entry_is_neutralised_then_retried_short_way() {
        let mut h = HeadingHold::default();
        assert_eq!(h.decide(2, 160.0, 190.0, 0.5, 0.0), Some(165.0), "start 25 left, not at the final bug");
        assert_eq!(h.decide(2, 160.0, 195.0, 10.0, 1.0), Some(195.0), "wrong-way right bank: capture the nose");
        assert_eq!(h.decide(2, 160.0, 198.0, 2.0, 2.0), None, "hold the capture until genuinely level");
        assert_eq!(h.decide(2, 160.0, 198.0, 0.5, 3.0), Some(173.0), "retry with a left lead");
        assert_eq!(h.decide(2, 160.0, 196.0, -4.0, 4.0), Some(160.0), "only after a left bank, select WOD's heading");

        // Issue #70 was an exact 25-degree right step that banked left. It
        // gets the same response detection even though no smaller lead exists.
        let mut h = HeadingHold::default();
        assert_eq!(h.decide(3, 125.0, 100.0, 0.0, 0.0), Some(125.0));
        assert_eq!(h.decide(3, 125.0, 97.0, -4.0, 1.0), Some(97.0));
    }

    #[tokio::test]
    async fn failed_or_refused_dial_writes_retry_the_same_target_after_recovery() {
        for name in [ALT_DIAL, VS_DIAL] {
            for failure in [1, 2, 3] {
                let io = DialIo {
                    value: std::sync::Mutex::new(0.0),
                    writes: std::sync::atomic::AtomicUsize::new(0),
                    failure: std::sync::atomic::AtomicUsize::new(failure),
                };
                let mut last = Some(0.0);
                assert!(!set_confirmed("procedure", &io, name, 700.0, &mut last, 40.0).await);
                assert_eq!(last, None);
                io.failure.store(0, Ordering::SeqCst);
                assert!(set_confirmed("procedure", &io, name, 700.0, &mut last, 40.0).await);
                assert_eq!(*io.value.lock().unwrap(), 700.0);
                assert_eq!(io.writes.load(Ordering::SeqCst), 2);
                assert!(set_confirmed("procedure", &io, name, 700.0, &mut last, 40.0).await);
                assert_eq!(io.writes.load(Ordering::SeqCst), 2);
            }
        }
    }

    /// The one formula the follower's law, the advisor and the spoken answer
    /// all read: LEJR-LXGB, 7,000 ft to 27D10 at 3,143 ft, 113 kt over the
    /// ground, planned at 75% of the 700 fpm: 13.8 nm of descent plus the lead.
    #[test]
    fn descent_point_is_one_formula() {
        let tod = descent_point_nm(3_143.0, 7_000.0, 113.0, GA_VS_FPM).unwrap();
        assert!((tod - 14.84).abs() < 0.05, "{tod}");
        // The law starts down exactly there, not before.
        assert_eq!(
            descent_target_ft(3_143.0, 7_000.0, 7_000.0, Some(tod + 0.1), 113.0, GA_VS_FPM),
            7_000.0
        );
        // Inside it, down a PATH (2026-09-26), not straight to the
        // constraint: a little under the level, and the constraint itself
        // only at the fix's lead.
        let just_in = descent_target_ft(3_143.0, 7_000.0, 7_000.0, Some(tod - 0.1), 113.0, GA_VS_FPM);
        assert!(just_in < 7_000.0 && just_in > 6_900.0, "{just_in}");
        let halfway = descent_target_ft(3_143.0, 7_000.0, 5_200.0, Some((tod + 1.0) / 2.0), 113.0, GA_VS_FPM);
        assert!((halfway - 5_070.0).abs() < 50.0, "{halfway}");
        assert_eq!(
            descent_target_ft(3_143.0, 7_000.0, 3_300.0, Some(1.0), 113.0, GA_VS_FPM),
            3_143.0
        );
        // Nothing to descend to: no point to announce.
        assert_eq!(descent_point_nm(7_000.0, 7_050.0, 113.0, GA_VS_FPM), None);
        assert_eq!(descent_point_nm(8_000.0, 7_000.0, 113.0, GA_VS_FPM), None);
        // Between procedures the accessor is empty whatever was published.
        publish_tod(Some(PlannedTod {
            fix: "X".into(),
            dist_nm: 20.0,
            to_tod_nm: 8.0,
            target_ft: 3_000.0,
            started: false,
            held_ft: None,
        }));
        assert_eq!(planned_tod(), None);
        publish_tod(None);
    }
    #[test]
    fn a_pilot_held_altitude_is_reminded_once_and_never_called_a_missed_descent() {
        let t = |fix: &str, to: f64| PlannedTod {
            fix: fix.into(),
            dist_nm: to + 3.0,
            to_tod_nm: to,
            target_ft: 18_000.0,
            started: false,
            held_ft: None,
        };
        let mut held = None;
        // No pilot altitude: the plan's point, untouched.
        assert_eq!(held_tod(Some(t("IDREK", 5.0)), None, &mut held), Some(t("IDREK", 5.0)));
        // LIME-LIPE: 19,000 held over IDREK, ADOSA, OSTEG, BENUS.
        let a = held_tod(Some(t("IDREK", 20.0)), Some(19_000.0), &mut held).unwrap();
        assert_eq!(a.held_ft, Some(19_000.0));
        assert_eq!(held, None, "not yet in the reminder's window");
        let a = held_tod(Some(t("IDREK", 5.0)), Some(19_000.0), &mut held).unwrap();
        assert_eq!(a.held_ft, Some(19_000.0));
        assert_eq!(held_tod(Some(t("IDREK", -2.0)), Some(19_000.0), &mut held).unwrap().fix, "IDREK");
        assert_eq!(held_tod(Some(t("ADOSA", 4.0)), Some(19_000.0), &mut held), None, "said once");
        assert_eq!(held_tod(Some(t("OSTEG", 9.0)), Some(19_000.0), &mut held), None);
        // A new altitude from the pilot is a new hold.
        assert!(held_tod(Some(t("BENUS", 6.0)), Some(17_000.0), &mut held).is_some());
    }

    use crate::nav::Fix;

    /// The crew's "next waypoint" answer while a procedure flies: distance to
    /// the follower's OWN fix, minutes from the groundspeed, no ETA when we are
    /// not really moving (a parked aircraft is not 0 minutes from anywhere).
    #[test]
    fn leg_progress_is_distance_and_minutes_to_the_followers_fix() {
        // MG430 from the point the LXGB-LEMG flight was actually at when the
        // GPS still said "PIMOS, 48 miles".
        let leg = ActiveLeg {
            fix: "MG430".into(),
            lat: 36.9985,
            lon: -4.9035,
        };
        let (dist, mins) = leg_progress(&leg, 36.95877, -4.90277, 100.0);
        assert!((dist - 2.39).abs() < 0.1, "dist {dist}");
        let mins = mins.expect("moving: an ETA");
        assert!((mins - 1.43).abs() < 0.1, "mins {mins}");
        assert_eq!(leg_progress(&leg, 36.95877, -4.90277, 3.0).1, None);
    }

    #[test]
    fn no_procedure_means_no_active_leg() {
        // Whatever a finished follower left in the slot, an inactive
        // procedure has no leg: the GPS answer applies again.
        publish_leg(Some(ActiveLeg {
            fix: "X".into(),
            lat: 0.0,
            lon: 0.0,
        }));
        if !ACTIVE.load(Ordering::Relaxed) {
            assert_eq!(active_leg(), None);
        }
        publish_leg(None);
    }

    /// EGPH AGPE1E's shape, on a line north: HAVEN 250 kt, TARTN "FL070,
    /// 230 kt" after SimBrief's guess at 9,000, then 8,000 on the far side.
    fn star_route() -> (Vec<ProcLeg>, Vec<crate::nav::StarConstraint>) {
        let at = |id: &str, nm: f64, alt: f64| {
            let mut l = leg(id, nm / 60.0, 0.0);
            l.alt1_ft = Some(alt);
            l
        };
        let legs = vec![
            at("AGPED", 0.0, 26_000.0),
            at("HAVEN", 30.0, 20_000.0),
            at("MIDDL", 55.0, 15_000.0),
            at("TARTN", 70.0, 9_000.0),
            at("AFTER", 80.0, 8_000.0),
        ];
        let c = |fix: &str, min, max, kts| crate::nav::StarConstraint {
            fix: fix.into(),
            min_ft: min,
            max_ft: max,
            max_kts: kts,
        };
        let cons = vec![
            c("AGPED", Some(26_000.0), Some(26_000.0), None),
            c("HAVEN", None, None, Some(250.0)),
            c("TARTN", None, Some(7_000.0), Some(230.0)),
        ];
        (legs, cons)
    }

    #[test]
    fn a_stars_window_is_clamped_into_the_plan_and_never_climbed_back_out_of() {
        let (mut legs, cons) = star_route();
        assert_eq!(apply_arrival_constraints(&mut legs, &cons), 2);
        let alts: Vec<f64> = legs.iter().map(|l| l.alt1_ft.unwrap()).collect();
        assert_eq!(alts, vec![26_000.0, 20_000.0, 15_000.0, 7_000.0, 7_000.0]);
        // A minimum past the first restriction still raises its own fix.
        let mut legs2 = star_route().0;
        let mut cons2 = cons.clone();
        cons2.push(crate::nav::StarConstraint {
            fix: "AFTER".into(),
            min_ft: Some(7_500.0),
            max_ft: None,
            max_kts: None,
        });
        apply_arrival_constraints(&mut legs2, &cons2);
        assert_eq!(legs2[4].alt1_ft, Some(7_500.0));
    }

    #[test]
    fn a_restriction_two_fixes_on_brings_the_descent_forward() {
        let (mut legs, cons) = star_route();
        apply_arrival_constraints(&mut legs, &cons);
        // Planned at the path's own 75 % (2026-09-26), and on the path's
        // miles: the last one before TARTN is its lead, flown level.
        let grad = arrival_ft_per_nm(2_000.0, 300.0);
        assert!((grad - 300.0).abs() < 1.0, "{grad}");
        let c = arrival_ceiling_ft(&legs, &cons, 2, grad).unwrap();
        assert!((c - (7_000.0 + 14.0 * 300.0)).abs() < 30.0, "MIDDL {c}");
        // Further out it is the same line: 39 path miles from HAVEN, under
        // the plan's 20,000 at the planning rate (above it at the old 85 %)...
        let c = arrival_ceiling_ft(&legs, &cons, 1, grad).unwrap();
        assert!((c - (7_000.0 + 39.0 * 300.0)).abs() < 30.0, "HAVEN {c}");
        // ...and a fix's own "at" stands over it.
        assert_eq!(arrival_ceiling_ft(&legs, &cons, 0, grad), Some(26_000.0));
        assert_eq!(arrival_ceiling_ft(&legs, &cons, 4, grad), None, "nothing ahead");
    }

    #[test]
    fn the_plans_own_altitudes_ahead_bring_the_descent_forward() {
        // No STAR at all: the plan says 8,000 two fixes on.
        let (mut legs, _) = star_route();
        for (l, a) in legs.iter_mut().zip([19_000.0, 19_000.0, 16_600.0, 8_000.0, 8_000.0]) {
            l.alt1_ft = Some(a);
        }
        let grad = arrival_ft_per_nm(1_300.0, 200.0);
        let c = profile_ceiling_ft(&legs, 2, grad).unwrap();
        assert!(c < 16_600.0, "the fix before must already be coming down: {c}");
        // An approach's "at or above" is no ceiling.
        legs[3].alt_desc = '+';
        legs[4].alt_desc = '+';
        assert_eq!(profile_ceiling_ft(&legs, 2, grad), None);
        // Nothing ahead, nothing to keep.
        assert_eq!(profile_ceiling_ft(&legs, 4, grad), None);
    }

    /// Fly the loop's vertical bookkeeping down the route at 300 kt and
    /// 2,000 fpm: with the ceiling TARTN is crossed at FL070, without it
    /// (each leg paced only to its own fix) it is not.
    #[test]
    fn the_arrival_crosses_its_restriction_at_the_published_altitude() {
        fn fly(with_ceiling: bool) -> f64 {
            let (mut legs, cons) = star_route();
            apply_arrival_constraints(&mut legs, &cons);
            let (gs, rate) = (300.0, 2_000.0);
            let (mut lat, mut alt, mut idx) = (-5.0 / 60.0, 26_000.0, 0usize);
            let (mut carry, mut prev) = (alt, alt);
            loop {
                let f = legs[idx].fix.clone().unwrap();
                let d = crate::nav::haversine_nm(lat, 0.0, f.lat, f.lon);
                if d < 0.3 {
                    if legs[idx].fix_ident == "TARTN" {
                        return alt;
                    }
                    idx += 1;
                    continue;
                }
                let mut l = legs[idx].clone();
                if with_ceiling {
                    if let Some(c) = arrival_ceiling_ft(&legs, &cons, idx, arrival_ft_per_nm(rate, gs)) {
                        if c < l.alt1_ft.unwrap() - 50.0 {
                            l.alt1_ft = Some((c / 100.0).floor() * 100.0);
                        }
                    }
                }
                let v = vertical_tick(&l, carry, prev, alt, Some(d), gs, None, None, rate, DescentProfile::none());
                carry = v.carry;
                prev = v.prev_cmd;
                if v.want_alt < alt - 20.0 {
                    alt = (alt - rate / 60.0).max(v.want_alt);
                }
                lat += gs / 3600.0 / 60.0;
            }
        }
        let crossed = fly(true);
        assert!(crossed <= 7_100.0, "crossed TARTN at {crossed}");
        assert!(fly(false) > 8_000.0, "the test must bite without the ceiling");
    }

    /// LEMG's UNTO2A as the 737 flew it on 2026-09-26 (LIBP-LEMG, sortie
    /// 1790429680): each fix with the plan's altitude after the STAR clamp
    /// (the `star_ceiling` records' `plan_ft`), and the groundspeed and
    /// airspeed recorded on the leg TO it (first, last).
    #[allow(clippy::type_complexity)]
    fn lemg_unto2a() -> (Vec<ProcLeg>, Vec<crate::nav::StarConstraint>, Vec<((f64, f64), (f64, f64))>) {
        let fixes: [(&str, f64, f64, f64, (f64, f64), (f64, f64)); 17] = [
            ("TOSGA", 37.625555556, -2.307222222, 37_000.0, (422.0, 435.0), (252.0, 257.0)),
            ("ROLAS", 37.415555556, -2.854444444, 37_000.0, (435.0, 410.0), (257.0, 252.0)),
            ("UNTOS", 37.250555556, -3.277694444, 29_000.0, (410.0, 383.0), (252.0, 254.0)),
            ("NESDA", 36.821472222, -3.741722222, 28_720.0, (383.0, 363.0), (254.0, 247.0)),
            ("IBACO", 36.710527778, -3.927638889, 26_007.0, (364.0, 353.0), (248.0, 252.0)),
            ("OSENE", 36.710055556, -4.051972222, 24_551.0, (351.0, 342.0), (250.0, 250.0)),
            ("PIZCU", 36.709444444, -4.176333333, 23_094.0, (342.0, 339.0), (250.0, 253.0)),
            ("MG539", 36.839777778, -4.411083333, 19_752.0, (339.0, 295.0), (253.0, 228.0)),
            ("OBLAC", 36.839055556, -4.577166667, 14_000.0, (294.0, 291.0), (227.0, 228.0)),
            ("FOFUS", 36.932777778, -4.695500000, 10_000.0, (291.0, 282.0), (228.0, 228.0)),
            ("MG529", 36.999527778, -4.696138889, 10_000.0, (282.0, 252.0), (228.0, 212.0)),
            ("MG533", 37.066277778, -4.696777778, 10_000.0, (252.0, 247.0), (212.0, 209.0)),
            ("MG537", 37.133027778, -4.697444444, 10_000.0, (247.0, 246.0), (209.0, 208.0)),
            ("MG541", 37.199805556, -4.698083333, 10_000.0, (246.0, 247.0), (207.0, 207.0)),
            ("MG437", 37.199111111, -4.802388889, 10_000.0, (247.0, 263.0), (207.0, 212.0)),
            ("MG433", 37.132333333, -4.801666667, 10_000.0, (263.0, 253.0), (212.0, 212.0)),
            ("LENHI", 37.065583333, -4.800916667, 8_800.0, (253.0, 254.0), (212.0, 218.0)),
        ];
        let legs = fixes
            .iter()
            .map(|(id, lat, lon, alt, _, _)| {
                let mut l = leg(id, *lat, *lon);
                l.alt1_ft = Some(*alt);
                l
            })
            .collect();
        let speeds = fixes.iter().map(|f| (f.4, f.5)).collect();
        // The chart's windows as `plan_audit` `star_constraints` logged them.
        let c = |fix: &str, min: Option<f64>, max: Option<f64>, kts: Option<f64>| crate::nav::StarConstraint {
            fix: fix.into(),
            min_ft: min,
            max_ft: max,
            max_kts: kts,
        };
        let cons = vec![
            c("UNTOS", Some(14_000.0), Some(29_000.0), None),
            c("NESDA", Some(13_500.0), None, None),
            c("IBACO", Some(11_000.0), None, None),
            c("OSENE", Some(10_000.0), None, None),
            c("MG539", Some(8_500.0), None, Some(230.0)),
            c("OBLAC", Some(7_000.0), Some(14_000.0), Some(220.0)),
            c("FOFUS", Some(7_000.0), Some(10_000.0), Some(220.0)),
            c("MG529", None, None, Some(210.0)),
            c("MG533", None, None, Some(210.0)),
            c("MG537", None, None, Some(210.0)),
            c("MG541", None, None, Some(210.0)),
            c("MG437", None, None, Some(210.0)),
            c("MG433", None, None, Some(210.0)),
            c("LENHI", Some(7_000.0), None, Some(210.0)),
        ];
        (legs, cons, speeds)
    }

    /// The stock 737's airspeed in a clean descent, kt/s: fitted on sortie
    /// 1790601822-full_auto (EGKK-EGBB, +1700 to +3310, 1,022 state ticks,
    /// 0.33 kt/s rms) from the lever, the flight path angle, the speedbrake
    /// and the altitude. At idle it holds its speed at about 2.9 degrees
    /// down at 8,000 ft and loses 0.9 kt/s level.
    fn b738_accel_kts_s(p: &Plant, thr: f64, vs_fpm: f64, gs_kts: f64, brake: bool, alt_ft: f64) -> f64 {
        let down_deg = -(vs_fpm / (gs_kts.max(60.0) * 101.27)).atan().to_degrees();
        -0.554 + p.drag_kts_s + 3.028 * thr + 0.299 * p.gamma_scale * down_deg - if brake { 0.432 } else { 0.0 }
            - 0.402 * alt_ft / 10_000.0
    }

    /// The fitted plant, and how far a sweep moves it.
    #[derive(Debug, Clone, Copy)]
    struct Plant {
        /// Scales the flight path angle's effect on the airspeed.
        gamma_scale: f64,
        /// Added to the idle deceleration, kt/s (negative: more drag).
        drag_kts_s: f64,
        /// The VS mode's dead time, half-second ticks.
        dead_ticks: usize,
    }

    impl Default for Plant {
        fn default() -> Self {
            Plant { gamma_scale: 1.0, drag_kts_s: 0.0, dead_ticks: 4 }
        }
    }

    /// Which laws an [`fly_arrival`] run flies.
    #[derive(Clone, Copy, PartialEq)]
    enum ArrivalLaw {
        /// The descent as flown on 2026-09-28: no speed protection, the
        /// speedbrake out at 8 over, in at 3, the rate cut to 0.75 and 0.5
        /// past 12 and 20 over.
        AsFlown,
        /// `speed_protected_vs` and the height-keyed `descent_drag`.
        Protected,
    }

    #[derive(Debug, Default)]
    struct ArrivalRun {
        /// (fix, altitude, airspeed) as each leg sequenced.
        crossings: Vec<(String, f64, f64)>,
        max_vs: f64,
        /// The most height regained above the lowest reached, ft: a
        /// level-off's overshoot coming back, or a descent climbing.
        climb_back_ft: f64,
        /// Speedbrake extensions.
        brake_outs: usize,
    }

    /// Fly the follower's real vertical and speed (the ceilings,
    /// `vertical_tick`, the VS law, the STAR's speed limits, the 250 below
    /// ten thousand, the governor's `throttle_adjust`, the speed protection
    /// and the drag, the 100 fpm dial) down `legs` from `idx` against the
    /// fitted 737 ([`b738_accel_kts_s`]): the VS mode as the autoland
    /// sorties model it (two 1.5 s lags behind 2 s of dead time), the lever
    /// spooling on a 2 s lag, the loop ticking every half second.
    /// `planned(idx)` is the cruise or profile speed on each leg, `gs(idx,
    /// frac, tas)` the groundspeed, `capture_nm` how far out a leg sequences.
    #[allow(clippy::too_many_arguments)]
    fn fly_arrival(
        legs: &[ProcLeg],
        cons: &[crate::nav::StarConstraint],
        mut idx: usize,
        (mut lat, mut lon): (f64, f64),
        mut alt: f64,
        mut ias: f64,
        planned: impl Fn(usize) -> f64,
        gs_of: impl Fn(usize, f64, f64) -> f64,
        capture_nm: f64,
        law: ArrivalLaw,
        plant: Plant,
    ) -> ArrivalRun {
        const RATE: f64 = 2_000.0;
        let dt = 0.5;
        let (mut carry, mut prev) = (alt, alt);
        let mut latch = None;
        let (mut drag_out, mut drag_held) = (false, f64::INFINITY);
        let mut begun = std::collections::HashSet::new();
        let mut airspace = crate::tools::speedhold::AirspaceLimit::default();
        let (mut lag1, mut vs) = (0.0f64, 0.0f64);
        let mut dial_queue: std::collections::VecDeque<f64> = std::collections::VecDeque::new();
        let mut run = ArrivalRun { max_vs: f64::MIN, ..Default::default() };
        let start = legs[idx - 1].fix.clone().unwrap();
        let mut leg_len = crate::nav::haversine_nm(start.lat, start.lon, legs[idx].fix.as_ref().unwrap().lat, legs[idx].fix.as_ref().unwrap().lon);
        let tas = |ias: f64, alt: f64| ias * (1.0 + 0.02 * alt / 1_000.0);
        let (mut thr, mut thrust) = (0.3f64, 0.3f64);
        let (mut trend, mut prev_ias) = (0.0f64, ias);
        let mut idle = IdleDescent::default();
        let mut low = alt;
        for _ in 0..40_000 {
            let f = legs[idx].fix.clone().unwrap();
            let d = crate::nav::haversine_nm(lat, lon, f.lat, f.lon);
            if d < capture_nm {
                run.crossings.push((legs[idx].fix_ident.clone(), alt, ias));
                idx += 1;
                if idx == legs.len() {
                    break;
                }
                let n = legs[idx].fix.clone().unwrap();
                leg_len = crate::nav::haversine_nm(lat, lon, n.lat, n.lon);
                continue;
            }
            let frac = (1.0 - d / leg_len).clamp(0.0, 1.0);
            let gs = gs_of(idx, frac, tas(ias, alt));
            // The speed the governor holds: the leg's, the STAR's limits,
            // the 250 below ten thousand.
            let mut target = planned(idx);
            if let Some((kts, _)) = arrival_speed_cap(legs, cons, idx, lat, lon, ias, gs, &mut begun) {
                target = target.min(if law == ArrivalLaw::Protected { kts - STAR_SPEED_MARGIN_KTS } else { kts });
            }
            let target = airspace.limit(alt, target);
            // The governor's own target: the speed owed, or under it at
            // idle in a descent (`IdleDescent`, the last tick's verdict).
            let governed = if law == ArrivalLaw::Protected { idle.governor_kts(target, None) } else { target };
            // The vertical.
            let mut l = legs[idx].clone();
            if let Some((a, _)) = ceilinged_leg_alt(legs, cons, idx, gs, RATE, &mut latch) {
                l.alt1_ft = Some(a);
            }
            let v = vertical_tick(&l, carry, prev, alt, Some(d), gs, None, None, RATE, DescentProfile::none());
            carry = v.carry;
            prev = v.prev_cmd;
            let want = vs_law_fpm(v.want_alt, alt, v.target_fpm, RATE, 1_500.0);
            let (cmd, out) = match law {
                ArrivalLaw::AsFlown => {
                    let over = ias - target;
                    let settled = drag_held >= DRAG_MIN_HOLD_S;
                    let out = if drag_out {
                        want < DRAG_KEEP_BELOW_FPM && (over > DRAG_IN_KTS || !settled)
                    } else {
                        want <= -DRAG_MIN_DESCENT_FPM && over > DRAG_OUT_KTS && settled
                    };
                    let scale = if want > -DRAG_MIN_DESCENT_FPM {
                        1.0
                    } else if over > 20.0 {
                        0.5
                    } else if over > 12.0 {
                        0.75
                    } else {
                        1.0
                    };
                    (want * scale, out)
                }
                ArrivalLaw::Protected => {
                    idle.step(want);
                    (
                        speed_protected_vs(want, Some(ias - target), trend),
                        descent_drag(drag_out, drag_held, ias, Some(target), want, alt - v.want_alt),
                    )
                }
            };
            if out && !drag_out {
                run.brake_outs += 1;
            }
            drag_held = if out == drag_out { drag_held + dt } else { 0.0 };
            drag_out = out;
            dial_queue.push_back(vs_dial_fpm(cmd));
            // 2 s of dead time at the half-second tick.
            let dial = if dial_queue.len() > plant.dead_ticks { dial_queue.pop_front().unwrap() } else { 0.0 };
            // The governor's tick, on the smoothed trend.
            thr = (thr + crate::tools::speedhold::throttle_adjust(governed - ias, trend)).clamp(0.0, 1.0);
            for _ in 0..10 {
                let h = dt / 10.0;
                lag1 += (dial - lag1) * h / 1.5;
                vs += (lag1 - vs) * h / 1.5;
                alt += vs * h / 60.0;
                thrust += (thr - thrust) * h / 2.0;
                ias += b738_accel_kts_s(&plant, thrust, vs, gs, drag_out, alt) * h;
            }
            trend += ((ias - prev_ias) / dt - trend) * dt / (IAS_TREND_TAU_S + dt);
            prev_ias = ias;
            run.max_vs = run.max_vs.max(vs);
            low = low.min(alt);
            run.climb_back_ft = run.climb_back_ft.max(alt - low);
            let brg = crate::nav::bearing_deg(lat, lon, f.lat, f.lon);
            (lat, lon) = crate::nav::project_latlon(lat, lon, brg, gs * dt / 3600.0);
        }
        run
    }

    /// The recorded UNTO2A from the ROLAS leg at FL370 (LIBP-LEMG,
    /// 2026-09-26), sequenced 1.2 nm out (FOFUS's recorded capture), the
    /// recorded groundspeed on each leg (the wind), flown closed loop.
    /// Returns the altitude at each fix and the highest vertical speed.
    fn fly_unto2a() -> (Vec<(String, f64)>, f64, f64) {
        let (legs, cons, speeds) = lemg_unto2a();
        let first = legs[0].fix.clone().unwrap();
        let run = fly_arrival(
            &legs,
            &cons,
            1,
            (first.lat, first.lon),
            37_000.0,
            257.0,
            |_| 255.0,
            |i, frac, tas| {
                // The recorded groundspeed less the recorded airspeed's TAS:
                // the wind, carried at the airspeed flown here.
                let ((g0, g1), (i0, i1)) = speeds[i];
                let (g, rec) = (g0 + (g1 - g0) * frac, i0 + (i1 - i0) * frac);
                let alt = legs[i].alt1_ft.unwrap_or(20_000.0);
                tas + g - rec * (1.0 + 0.02 * alt / 1_000.0)
            },
            1.2,
            ArrivalLaw::Protected,
            Plant::default(),
        );
        (run.crossings.into_iter().map(|(f, a, _)| (f, a)).collect(), run.max_vs, run.climb_back_ft)
    }

    /// EGBB's HEME1B as the 737 flew it on 2026-09-28 (sortie
    /// 1790601822-full_auto): LAM at the FL180 cruise, then the STAR's
    /// fixes (earth_fix.dat, HON the VOR), the navlog's straight line from
    /// the cruise at HEMEL to GROVE's FL070 (`plan_audit::realtitude_star`'s
    /// rule) under the windows `procedure` `star_constraints` logged.
    fn egbb_heme1b() -> (Vec<ProcLeg>, Vec<crate::nav::StarConstraint>) {
        let fixes: [(&str, f64, f64); 8] = [
            ("LAM", 51.646, 0.151667),
            ("HEMEL", 51.805555556, -0.419383333),
            ("BUZAD", 51.942244444, -0.552280556),
            ("WELIN", 52.247291667, -0.852336111),
            ("PUFAX", 52.314447222, -1.345055556),
            ("HON", 52.356677778, -1.663725000),
            ("OSKOT", 52.341655556, -2.058408333),
            ("GROVE", 52.393822222, -1.928811111),
        ];
        let mut legs: Vec<ProcLeg> = fixes.iter().map(|(id, la, lo)| leg(id, *la, *lo)).collect();
        legs[0].alt1_ft = Some(18_000.0);
        let star = crate::plan_audit::realtitude_star(legs[1..].to_vec(), Some(18_000.0), Some(7_000.0), 1);
        legs.truncate(1);
        legs.extend(star);
        let c = |fix: &str, min: Option<f64>, max: Option<f64>, kts: Option<f64>| crate::nav::StarConstraint {
            fix: fix.into(),
            min_ft: min,
            max_ft: max,
            max_kts: kts,
        };
        let cons = vec![
            c("HEMEL", Some(22_000.0), Some(22_000.0), None),
            c("PUFAX", None, None, Some(250.0)),
            c("HON", Some(9_000.0), Some(9_000.0), None),
            c("OSKOT", None, None, Some(210.0)),
            c("GROVE", Some(7_000.0), Some(7_000.0), Some(210.0)),
        ];
        (legs, cons)
    }

    /// The HEME1B from BUZAD at FL180 and 302 kt, as recorded at +1995: the
    /// OFP's cruise on WELIN (299, or the capped 296), the profile's 255
    /// after, no wind (the recorded groundspeed at FL180 was within 10 kt of
    /// the TAS). Sequenced at the recorded 0.9 nm.
    fn fly_heme1b(law: ArrivalLaw, plant: Plant) -> ArrivalRun {
        let (mut legs, cons) = egbb_heme1b();
        apply_arrival_constraints(&mut legs, &cons);
        let welin_kts = if law == ArrivalLaw::AsFlown { 299.0 } else { 296.0 };
        let buzad = legs[2].fix.clone().unwrap();
        fly_arrival(
            &legs,
            &cons,
            3,
            (buzad.lat, buzad.lon),
            18_031.0,
            302.0,
            |i| if i == 3 { welin_kts } else { 255.0 },
            |_, _, tas| tas - 10.0,
            0.9,
            law,
            plant,
        )
    }

    /// #156 (and #32's windows): PUFAX 254 against 250, OSKOT 216 and GROVE
    /// 218 against 210, six speedbrake extensions down the HEME1B, each at
    /// 8 kt over. Flown on the fitted 737, the old laws repeat the busts;
    /// the protected descent crosses every limit within 2 kt, every window
    /// within 300 ft, and needs the brake at most twice.
    #[test]
    fn the_heme1b_is_flown_at_its_speeds_with_the_brake_for_height() {
        let old = fly_heme1b(ArrivalLaw::AsFlown, Plant::default());
        let new = fly_heme1b(ArrivalLaw::Protected, Plant::default());
        let at = |run: &ArrivalRun, fix: &str| {
            run.crossings.iter().find(|c| c.0 == fix).cloned().unwrap_or_else(|| panic!("{fix}: {:?}", run.crossings))
        };
        let busts = |run: &ArrivalRun| {
            [("PUFAX", 250.0), ("OSKOT", 210.0), ("GROVE", 210.0)]
                .iter()
                .filter(|(f, kts)| at(run, f).2 > kts + 2.0)
                .count()
        };
        assert!(busts(&old) >= 1 || old.brake_outs > 2, "the test must bite: {old:?}");
        assert_eq!(busts(&new), 0, "{new:?}");
        assert!(new.brake_outs <= 2, "{} extensions: {new:?}", new.brake_outs);
        let hon = at(&new, "HON").1;
        assert!((hon - 9_000.0).abs() <= 300.0, "HON at {hon:.0}: {new:?}");
        let grove = at(&new, "GROVE").1;
        assert!((grove - 7_000.0).abs() <= 300.0, "GROVE at {grove:.0}: {new:?}");
        assert!(new.climb_back_ft < 100.0, "{:.0} ft regained: {new:?}", new.climb_back_ft);
        // Not a fit to one plant: the flight path's effect 25 % either way,
        // the idle deceleration 0.15 kt/s either way, the VS mode's dead
        // time 1 to 3 s. Every limit within 3 kt, the brake rare.
        let (mut worst, mut outs) = ((0.0f64, String::new()), 0usize);
        for gamma_scale in [0.75, 1.0, 1.25] {
            for drag_kts_s in [-0.15, 0.0, 0.15] {
                for dead_ticks in [2, 4, 6] {
                    let plant = Plant { gamma_scale, drag_kts_s, dead_ticks };
                    let run = fly_heme1b(ArrivalLaw::Protected, plant);
                    for (fix, kts) in [("PUFAX", 250.0), ("OSKOT", 210.0), ("GROVE", 210.0)] {
                        let over = at(&run, fix).2 - kts;
                        if over > worst.0 {
                            worst = (over, format!("{fix} on {plant:?}: {run:?}"));
                        }
                    }
                    outs = outs.max(run.brake_outs);
                }
            }
        }
        assert!(worst.0 <= 3.0, "{:.1} over at {}", worst.0, worst.1);
        assert!(outs <= 4, "{outs} extensions");
    }

    /// iMac 2026-09-30, sortie 1790759458-full_auto (#156): the `star_speed`
    /// record read as "OSKOT 248 against 210" is the slow-down BEGINNING,
    /// 7.86 nm along the route at 248.0 kt and 279 kt over the ground: 38 kt
    /// at the planned 0.5 kt/s is 76 s, 5.9 nm, plus the 2 nm lead. Level at
    /// idle the 737 lost it at 1.15 kt/s (248.0 to 210.9 from 7.86 to 5.56
    /// nm, 32 s), so the lead left 3.3 nm to spare before OSKOT sequenced
    /// (2.1 nm out, 208.4 kt). The planned rate stays under the measured.
    #[test]
    fn the_recorded_oskot_slow_down_begins_with_miles_to_spare() {
        let (legs, cons) = egbb_heme1b();
        let k = legs.iter().position(|l| l.fix_ident == "OSKOT").unwrap();
        let hon = legs[k - 1].fix.clone().unwrap();
        let oskot = legs[k].fix.clone().unwrap();
        let back = crate::nav::bearing_deg(oskot.lat, oskot.lon, hon.lat, hon.lon);
        let at = |nm: f64| crate::nav::project_latlon(oskot.lat, oskot.lon, back, nm);
        let cap = |nm: f64| {
            let (lat, lon) = at(nm);
            arrival_speed_cap(&legs, &cons, k, lat, lon, 248.0, 279.0, &mut Default::default())
        };
        // Short of the start only PUFAX's 250, passed, holds.
        assert_eq!(cap(8.3), Some((250.0, "PUFAX".to_string())));
        assert_eq!(cap(7.86), Some((210.0, "OSKOT".to_string())));
        // The recorded level deceleration and the miles it took.
        let (measured_kts_s, used_nm, sequenced_nm) = ((248.0 - 210.9) / 32.2, 7.86 - 5.56, 2.1);
        assert!(ARRIVAL_DECEL_KTS_S < measured_kts_s, "{measured_kts_s:.2} kt/s");
        let lead = (248.0 - 210.0) / ARRIVAL_DECEL_KTS_S * 279.0 / 3600.0 + ARRIVAL_SPEED_LEAD_NM;
        assert!(lead - used_nm - sequenced_nm > 3.0, "lead {lead:.2} nm");
        // Judged where it is flown: the crossing record, not the start's.
        let c = star_speed_crossing(&cons, &legs[k], 208.4).unwrap();
        assert!(c.met && (c.over_target_kts - 1.4).abs() < 1e-9, "{c:?}");
        assert!(!star_speed_crossing(&cons, &legs[k], 248.0).unwrap().met, "the start's airspeed is no crossing");
        assert!(star_speed_crossing(&cons, &legs[k - 2], 246.7).unwrap().met, "PUFAX 246.7 against 250");
        assert_eq!(star_speed_crossing(&cons, &legs[k - 1], 245.0), None, "HON has no speed");
    }

    /// The same sortie flown closed loop from its placement at WELIN (FL180,
    /// 293.7 kt, the profile's 255 after, 19 kt of headwind: 381.9 kt over
    /// the ground at 295), sequenced 0.8 nm out (PUFAX and HON as recorded)
    /// on the fitted plant's sweep: every limit met at its fix. The speed
    /// protection is proportional, so a descent held at idle sits over its
    /// target (OSKOT 208.4 flown against 207; 202 to 209.6 on the sweep): the
    /// 3 kt margin is what absorbs it. Moving the protection's band under the
    /// target (from 0 to -1.5 kt) held the targets but broke the HEME1B's
    /// windows and the GROVE join, so it is not done here.
    #[test]
    fn the_recorded_welin_arrival_meets_every_star_speed_at_its_fix() {
        let (mut legs, cons) = egbb_heme1b();
        apply_arrival_constraints(&mut legs, &cons);
        let welin = legs.iter().position(|l| l.fix_ident == "WELIN").unwrap();
        let start = legs[welin].fix.clone().unwrap();
        for gamma_scale in [0.75, 1.0, 1.25] {
            for drag_kts_s in [-0.15, 0.0, 0.15] {
                let plant = Plant { gamma_scale, drag_kts_s, dead_ticks: 4 };
                let run = fly_arrival(
                    &legs,
                    &cons,
                    welin + 1,
                    (start.lat, start.lon),
                    18_077.0,
                    293.7,
                    |_| 255.0,
                    |_, _, tas| tas - 19.0,
                    0.8,
                    ArrivalLaw::Protected,
                    plant,
                );
                for (fix, kts) in [("PUFAX", 250.0), ("OSKOT", 210.0), ("GROVE", 210.0)] {
                    let ias = run.crossings.iter().find(|c| c.0 == fix).map(|c| c.2).unwrap();
                    let leg = legs.iter().find(|l| l.fix_ident == fix).unwrap();
                    let c = star_speed_crossing(&cons, leg, ias).unwrap();
                    assert!(c.met && c.kts == kts, "{fix} at {ias:.1} ({plant:?}): {run:?}");
                }
                assert!(run.climb_back_ft < 100.0, "{run:?}");
            }
        }
    }

    /// FOFUS on UNTO2A is 7,000 to 10,000 ft and the 737 crossed it at 10,386
    /// (2026-09-26): every fix's target was held a few miles into its leg
    /// against the last one (never reached), the proportional VS law trailed
    /// the path by 500 ft, and MG539's ceiling rose 600 ft as the aircraft
    /// slowed and the descent climbed back. Replayed on the recorded legs,
    /// closed loop on the fitted 737 since #156 (the speed flown, not the
    /// speed recorded): every window within the card's 300 ft slack, FOFUS
    /// within 200 ft of its top as the leg sequences 1.2 nm out (the path
    /// reaches a constraint 1 nm before its fix), and nothing climbs. The
    /// old laws on the same plant climbed at 293 fpm.
    #[test]
    fn the_recorded_unto2a_crosses_fofus_inside_its_window() {
        let (crossings, max_vs, climb_back) = fly_unto2a();
        let (_, cons, _) = lemg_unto2a();
        let at = |fix: &str| crossings.iter().find(|(f, _)| f == fix).map(|c| c.1).unwrap();
        let fofus = at("FOFUS");
        assert!((7_000.0..=10_200.0).contains(&fofus), "FOFUS at {fofus:.0}: {crossings:?}");
        // A level-off's overshoot comes back; a descent never climbs (MG539
        // regained 600 ft).
        assert!(climb_back < 100.0, "{climb_back:.0} ft regained: {crossings:?}");
        for c in &cons {
            let alt = at(&c.fix);
            assert!(c.min_ft.is_none_or(|m| alt >= m - 300.0), "{} at {alt:.0}", c.fix);
            assert!(c.max_ft.is_none_or(|m| alt <= m + 300.0), "{} at {alt:.0}", c.fix);
        }
        assert!(max_vs < 250.0, "the descent climbed at {max_vs:.0} fpm: {crossings:?}");
    }

    /// EGBB I15 as the CIFP codes it (IF CF15, CF FF15 at 2,500 ft, CF RW15),
    /// resolved (earth_fix.dat; the threshold from apt.dat).
    fn egbb_i15() -> Vec<ProcLeg> {
        let rows = "APPCH:010,I,I15, ,CF15,EG,P,C,E  I, ,   ,IF, ,IBIR,EG,P,I,      ,3248,0106,    ,    ,J,02500,02500,     , ,   ,    ,   , , , , , ,0,D,S;\n\
                    APPCH:020,I,I15, ,FF15,EG,P,C,E  F, ,   ,CF, ,IBIR,EG,P,I,      ,3248,0083,1450,0023,H,02500,02500,     , ,   ,-300,   ,BHX,EG,D,B, ,0,D,S;\n\
                    APPCH:030,I,I15, ,RW15,EG,P,G,G  M, ,   ,CF, ,IBIR,EG,P,I,      ,3248,0016,1450,0067, ,00358,     ,     , ,   ,-300,   , , , , , ,0,D,S;\n";
        let raw = crate::nav::parse_proc_legs(rows.lines().map(str::trim), crate::nav::ProcKind::Appch, "I15", None);
        let mut legs = crate::nav::straight_in_legs(raw, "15").expect("straight in");
        let at = |ident: &str, lat: f64, lon: f64| Fix { ident: ident.into(), lat, lon };
        legs[0].fix = Some(at("CF15", 52.587216667, -1.896825));
        legs[1].fix = Some(at("FF15", 52.555586111, -1.861436111));
        legs[2].fix = Some(at("RW15", 52.4656497, -1.7611282));
        legs
    }

    #[derive(Debug, Default)]
    struct JoinRun {
        /// Seconds from GROVE to the FAF's capture (the autoland handoff).
        secs: f64,
        alt: f64,
        ias: f64,
        brake_outs: usize,
        /// The fastest after the first minute (the join's own, not GROVE's).
        max_ias: f64,
        /// The fastest while descending.
        max_descent_ias: f64,
        /// Miles to the FAF at its capture.
        faf_nm: f64,
        /// Miles before the FAF the final was first established (300 m, 10
        /// degrees).
        established_nm: Option<f64>,
        /// The steers in the order flown.
        steers: Vec<&'static str>,
    }

    /// The approach armed at GROVE (+2862, sortie 1790601822-full_auto):
    /// 7,022 ft, 217 kt, tracking 119 at 242 kt over the ground, abeam
    /// runway 15's threshold 7.6 nm to the right of the final. The follower's
    /// lateral (`FinalJoin`'s pattern, the course legs, the heading set as a
    /// pilot sets it, a 25 degree autopilot), its vertical paced on
    /// `pattern_nm` or, as flown, on the straight line to the IF, and the
    /// approach's 210.25 kt on the fitted 737 ([`fly_arrival`]'s plant and
    /// laws), to the FAF's capture.
    fn fly_egbb_join(law: ArrivalLaw) -> JoinRun {
        fly_egbb_join_from(&GROVE_1790601822, law, JoinGeometry::AsFlown)
    }

    /// Where the approach armed at GROVE: position, track, altitude, airspeed,
    /// and the wind's share of the groundspeed, kt.
    struct JoinStart {
        lat: f64,
        lon: f64,
        hdg: f64,
        alt: f64,
        ias: f64,
        wind_kts: f64,
    }

    /// Sortie 1790601822-full_auto, +2862.
    const GROVE_1790601822: JoinStart = JoinStart {
        lat: 52.403079551600634,
        lon: -1.9394949225789109,
        hdg: 118.6,
        alt: 7_021.5,
        ias: 217.1,
        wind_kts: 25.0,
    };

    /// Sortie 1790635335-full_auto, +790.0 (t 1790636125): 7,185 ft, 209 kt,
    /// tracking 116 at 226 kt over the ground; the procedure's first tick put
    /// it 5.9 nm past the FAF, 7.7 nm right of the final. As flown the FAF
    /// was captured 466 s later (+1256), established 6.7 nm before it.
    const GROVE_1790635335: JoinStart = JoinStart {
        lat: 52.40216798594475,
        lon: -1.946318684047718,
        hdg: 116.0,
        alt: 7_185.3,
        ias: 208.9,
        wind_kts: -13.0,
    };

    /// Sortie 1790624015-full_auto, +2215.1 (t 1790626230): 4,797 ft, 204 kt,
    /// tracking 104 at 209 kt; 6.1 nm past the FAF, 7.9 nm right. As flown
    /// the FAF was captured 429 s later (+2644) at 211.5 kt, 2,899 ft.
    const GROVE_1790624015: JoinStart = JoinStart {
        lat: 52.39743536948908,
        lon: -1.9473286705380015,
        hdg: 103.9,
        alt: 4_796.7,
        ias: 204.3,
        wind_kts: -15.0,
    };

    /// Which base and skip the join flies.
    #[derive(Clone, Copy, PartialEq, Debug)]
    enum JoinGeometry {
        /// 2026-09-28: the base two sizing radii beyond the established
        /// point, the skip on the old room.
        AsFlown,
        /// #161: the base sized on the offset and the radius.
        Sized,
    }

    fn fly_egbb_join_from(start: &JoinStart, law: ArrivalLaw, geometry: JoinGeometry) -> JoinRun {
        use crate::course_leg::{course_passed, course_track, CourseLine, FinalJoin, Join, Steer};
        const VAR: f64 = -0.9;
        const RATE: f64 = 2_000.0;
        let legs = egbb_i15();
        let plant = Plant::default();
        let (mut lat, mut lon) = (start.lat, start.lon);
        let (mut hdg, mut bank) = (start.hdg, 0.0f64);
        let (mut alt, mut ias) = (start.alt, start.ias);
        let target = crate::autoland::clean_approach_ias(145.0);
        let mut join = match geometry {
            JoinGeometry::AsFlown => FinalJoin::as_flown(),
            JoinGeometry::Sized => FinalJoin::default(),
        };
        let mut hold = HeadingHold::default();
        let mut bug_true = hdg;
        let (mut idx, mut was) = (0usize, Steer::Direct);
        let (mut carry, mut prev) = (alt, alt);
        let (mut drag_out, mut drag_held) = (false, f64::INFINITY);
        let (mut lag1, mut vs) = (-464.0f64, -464.0f64);
        let mut dial_queue: std::collections::VecDeque<f64> = std::collections::VecDeque::new();
        let (mut thr, mut thrust) = (0.0f64, 0.0f64);
        let (mut trend, mut prev_ias) = (0.0f64, ias);
        let mut idle = IdleDescent::default();
        let mut run = JoinRun::default();
        let dt = 0.5;
        for tick in 0..6_000 {
            let now = tick as f64 * dt;
            let tas = ias * (1.0 + 0.02 * alt / 1_000.0);
            let gs = tas + start.wind_kts;
            let leg = &legs[idx];
            let (mut steer, mut track) = (Steer::Direct, None);
            match join.step(&legs, idx, lat, lon, hdg, gs, VAR, alt, RATE) {
                Some(Join::Skip { .. }) => {
                    idx += 1;
                    continue;
                }
                Some(Join::Pattern { track_true, leg, .. }) => {
                    steer = Steer::Pattern(leg);
                    track = Some(track_true);
                }
                _ => {}
            }
            let line = CourseLine::of_leg(leg, VAR);
            if track.is_none() {
                if let Some((t, s)) = line.as_ref().and_then(|l| course_track(l, lat, lon, hdg, gs, was == Steer::Track)) {
                    track = Some(t);
                    steer = s;
                }
            }
            let f = leg.fix.clone().unwrap();
            let d = crate::nav::haversine_nm(lat, lon, f.lat, f.lon);
            let track = track.unwrap_or_else(|| crate::nav::bearing_deg(lat, lon, f.lat, f.lon));
            was = steer;
            if run.steers.last() != Some(&steer.word()) {
                run.steers.push(steer.word());
            }
            if run.established_nm.is_none() {
                let faf = CourseLine::of_leg(&legs[1], VAR).unwrap();
                let (along, xtk_m) = (faf.along_nm(lat, lon), faf.xtk_nm(lat, lon) * 1852.0);
                let off = crate::angles::abs_diff_deg(hdg, faf.course_true());
                if along > 0.0 && xtk_m.abs() < 300.0 && off < 10.0 {
                    run.established_nm = Some(along);
                }
            }
            let mag = |t: f64| crate::autoland::true_to_mag(t, VAR);
            if let Some(h) = hold.decide(steer.key(idx), mag(track), mag(hdg), bank, now) {
                bug_true = h - VAR;
            }
            let cap = capture_radius_nm(gs, leg_turn_deg(lat, lon, leg, legs.get(idx + 1)).unwrap_or(0.0));
            let done = leg_done(leg, Some(d), alt, cap, lat, lon)
                || line.as_ref().is_some_and(|l| course_passed(l, lat, lon, cap));
            if done {
                if idx == 1 {
                    (run.secs, run.alt, run.ias, run.faf_nm) = (now, alt, ias, d);
                    return run;
                }
                idx += 1;
                continue;
            }
            // The vertical: the pattern's miles, or (as flown) the line; a
            // skipped IF still ahead keeps its own altitude and distance.
            let owed_if = join.skipped_if_ahead_nm(&legs, idx, lat, lon, VAR);
            let dist = match (law, owed_if) {
                (ArrivalLaw::Protected, Some((_, d_if))) => Some(d_if),
                (ArrivalLaw::Protected, None) => {
                    join.pattern_to_go_nm(&legs, idx, lat, lon, gs, VAR).or(Some(d))
                }
                (ArrivalLaw::AsFlown, _) => Some(d),
            };
            let vleg = owed_if.map_or(leg, |(i, _)| &legs[i]);
            let v = vertical_tick(
                vleg, carry, prev, alt, dist, gs, None, None, RATE, DescentProfile::none(),
            );
            carry = v.carry;
            prev = v.prev_cmd;
            let want = vs_law_fpm(v.want_alt, alt, v.target_fpm, RATE, 1_500.0);
            let governed = if law == ArrivalLaw::Protected { idle.governor_kts(target, None) } else { target };
            let (cmd, out) = match law {
                ArrivalLaw::AsFlown => {
                    let over = ias - target;
                    let settled = drag_held >= DRAG_MIN_HOLD_S;
                    let out = if drag_out {
                        want < DRAG_KEEP_BELOW_FPM && (over > DRAG_IN_KTS || !settled)
                    } else {
                        want <= -DRAG_MIN_DESCENT_FPM && over > DRAG_OUT_KTS && settled
                    };
                    (want, out)
                }
                ArrivalLaw::Protected => {
                    idle.step(want);
                    (
                        speed_protected_vs(want, Some(ias - target), trend),
                        descent_drag(drag_out, drag_held, ias, Some(target), want, alt - v.want_alt),
                    )
                }
            };
            if out && !drag_out {
                run.brake_outs += 1;
            }
            drag_held = if out == drag_out { drag_held + dt } else { 0.0 };
            drag_out = out;
            dial_queue.push_back(vs_dial_fpm(cmd));
            let dial = if dial_queue.len() > plant.dead_ticks { dial_queue.pop_front().unwrap() } else { vs };
            thr = (thr + crate::tools::speedhold::throttle_adjust(governed - ias, trend)).clamp(0.0, 1.0);
            for _ in 0..10 {
                let h = dt / 10.0;
                lag1 += (dial - lag1) * h / 1.5;
                vs += (lag1 - vs) * h / 1.5;
                alt += vs * h / 60.0;
                thrust += (thr - thrust) * h / 2.0;
                ias += b738_accel_kts_s(&plant, thrust, vs, gs, drag_out, alt) * h;
            }
            trend += ((ias - prev_ias) / dt - trend) * dt / (IAS_TREND_TAU_S + dt);
            prev_ias = ias;
            if now > 60.0 {
                run.max_ias = run.max_ias.max(ias);
            }
            if vs < -500.0 {
                run.max_descent_ias = run.max_descent_ias.max(ias);
            }
            // The heading mode: a rate-limited roll to 25 degrees, the short way.
            let err = crate::angles::signed_diff_deg(bug_true, hdg);
            let want_bank = (err * 1.5).clamp(-25.0, 25.0);
            bank += (want_bank - bank).clamp(-5.0 * dt, 5.0 * dt);
            let v_ms = gs * 0.514_444;
            hdg = (hdg + (9.81 * bank.to_radians().tan() / v_ms).to_degrees() * dt).rem_euclid(360.0);
            (lat, lon) = crate::nav::project_latlon(lat, lon, hdg, gs * dt / 3600.0);
        }
        panic!("never reached the FAF: {run:?}");
    }

    /// #156, #126: the join from GROVE held a 4,011 ft plateau round the
    /// pattern (its path to the IF paced on the straight line), came down
    /// the last 4 nm at up to 2,240 fpm with the speedbrake out three times
    /// around 210.25, and was handed to autoland at 222 kt. Flown on the
    /// pattern's miles at idle with the speed on the pitch, it reaches the
    /// 2,500 ft platform before the FAF and is handed over at the approach
    /// speed, the brake needed at most once.
    #[test]
    fn the_join_from_grove_arrives_at_the_faf_at_its_speed_and_height() {
        let old = fly_egbb_join(ArrivalLaw::AsFlown);
        let new = fly_egbb_join(ArrivalLaw::Protected);
        // As flown: 218 kt at the handoff, four extensions (the flight: 222,
        // three).
        assert!(old.ias > 215.0 && old.brake_outs >= 2, "the test must bite: {old:?}");
        assert!(new.ias <= 213.0, "{new:?}");
        assert!(new.max_descent_ias <= 215.0 && old.max_descent_ias > 218.0, "{new:?} {old:?}");
        // At the FAF's capture on its 2,500 ft platform or on the way to it,
        // under the glideslope (3 degrees, 318 ft a mile, through 2,500 ft
        // at the FAF): the ILS is joined from below.
        assert!(new.alt >= 2_400.0 && new.alt <= 2_500.0 + new.faf_nm * 318.0, "{new:?}");
        assert!(new.brake_outs <= 1, "{new:?}");
        // The pattern is the pattern: the join is no longer for it.
        assert!(new.secs <= old.secs + 30.0, "{new:?} {old:?}");
    }

    /// #161: the two recorded joins from GROVE, replayed on the geometry
    /// they flew: the FAF captured within 20 s of the recorded 466 s
    /// (1790635335) and 429 s (1790624015), established 5 nm or more before
    /// it, and handed over high (the flight: 2,899 ft at 0.78 nm).
    #[test]
    fn the_recorded_joins_from_grove_replay_their_times() {
        for (start, recorded) in [(&GROVE_1790635335, 466.0), (&GROVE_1790624015, 429.0)] {
            let r = fly_egbb_join_from(start, ArrivalLaw::Protected, JoinGeometry::AsFlown);
            assert!((r.secs - recorded).abs() <= 20.0, "{recorded} s recorded: {r:?}");
            assert!(r.established_nm.is_some_and(|e| e > 5.0), "{r:?}");
            assert!(r.alt > 2_500.0 + r.faf_nm * 318.0, "over the glideslope as flown: {r:?}");
        }
    }

    /// #161: the join sized on the downwind's offset and the turn radius,
    /// the intercept's turn counted as flown, and the skipped IF's 2,500 ft
    /// still owed while it lies ahead: from each recorded GROVE state the
    /// FAF is captured sooner, still established 2 nm before it off a
    /// base, on the 2,500 ft platform and at the approach's 210 kt.
    #[test]
    fn the_join_from_grove_is_shorter_and_hands_over_on_the_platform() {
        let mut saved = 0.0;
        for start in [&GROVE_1790601822, &GROVE_1790635335, &GROVE_1790624015] {
            let old = fly_egbb_join_from(start, ArrivalLaw::Protected, JoinGeometry::AsFlown);
            let new = fly_egbb_join_from(start, ArrivalLaw::Protected, JoinGeometry::Sized);
            assert!(new.secs <= old.secs - 30.0, "{new:?} {old:?}");
            saved += old.secs - new.secs;
            let flown = ["pattern_outbound", "pattern_base", "intercept", "track"];
            assert_eq!(new.steers, flown, "{new:?}");
            let est = new.established_nm.expect("established before the FAF");
            assert!(est >= 2.0, "{new:?}");
            assert!((2_450.0..=2_600.0).contains(&new.alt), "on the platform: {new:?}");
            assert!(new.ias <= 215.0, "{new:?}");
        }
        assert!(saved >= 150.0, "{saved:.0} s saved over three joins");
    }

    /// #156 / #32: HEMEL's "at FL220" in the FL180 plan is flown at the
    /// cruise, said once at the arm; from the HEMEL leg as recorded (+1661.6:
    /// 6,350 ft, 23.4 nm, 273 kt, held at 6,000 until LAM by the clearance)
    /// even the 2,000 fpm climb cannot make 18,000, and that is said once
    /// with the number, where the flight crossed it at 14,638 unsaid.
    #[test]
    fn a_star_window_that_cannot_be_made_is_said() {
        let (mut legs, cons) = egbb_heme1b();
        apply_arrival_constraints(&mut legs, &cons);
        assert_eq!(legs[1].fix_ident, "HEMEL");
        assert_eq!(legs[1].alt1_ft, Some(18_000.0), "never climbed to FL220");
        assert!(legs.iter().all(|l| l.alt1_ft.unwrap() <= 18_000.0));
        let cruise = legs.iter().filter_map(|l| l.alt1_ft).fold(0.0, f64::max);
        assert_eq!(windows_above_cruise(&legs, &cons, cruise), vec![("HEMEL".to_string(), 22_000.0, 18_000.0)]);
        assert_eq!(
            above_cruise_line("HEMEL", 22_000.0, 18_000.0),
            "HEMEL's 22,000 feet is above our cruise. We'll cross it at 18,000 feet."
        );
        // A plan that starts on the STAR has no cruise before it: as before.
        let (legs2, cons2) = star_route();
        let mut l2 = legs2.clone();
        apply_arrival_constraints(&mut l2, &cons2);
        assert_eq!(l2[0].alt1_ft, Some(26_000.0));
        // On the LAM leg, 29 nm out: the climb still makes it.
        let hemel = legs[1].fix.clone().unwrap();
        let (lat, lon) = crate::nav::project_latlon(hemel.lat, hemel.lon, 110.0, 23.4);
        let lam = legs[0].fix.clone().unwrap();
        assert_eq!(
            star_window_miss(&legs, &cons, 0, lam.lat + 0.09, lam.lon, 6_079.0, 280.0, 2_000.0, 2_000.0, Some(cruise)),
            None
        );
        // On the HEMEL leg as recorded: 1,300 ft low even at the best.
        let miss = star_window_miss(&legs, &cons, 1, lat, lon, 6_350.0, 273.0, 2_000.0, 2_000.0, Some(cruise))
            .expect("unmakeable");
        assert!(miss.low && miss.limit_ft == 18_000.0, "{miss:?}");
        assert!((1_000.0..1_700.0).contains(&miss.by_ft), "{miss:?}");
        assert_eq!(miss.line(), format!("We'll cross HEMEL about {} feet under its 18,000 feet.",
            crate::auto_status::commas(((miss.by_ft / 50.0).round() * 50.0) as i64)));
        // An "at or below" the best descent cannot reach: said high.
        let honl = legs[5].fix.clone().unwrap();
        let (lat, lon) = crate::nav::project_latlon(honl.lat, honl.lon, 90.0, 3.0);
        let high = star_window_miss(&legs, &cons, 5, lat, lon, 12_000.0, 250.0, 2_000.0, 2_000.0, Some(cruise))
            .expect("too high for HON");
        assert!(!high.low && high.limit_ft == 9_000.0 && high.by_ft > 1_000.0, "{high:?}");
        // Within the slack, or with room: nothing.
        assert_eq!(star_window_miss(&legs, &cons, 5, lat, lon, 10_200.0, 250.0, 2_000.0, 2_000.0, Some(cruise)), None);
    }

    /// #173 (sortie 1790624015-full_auto, +1408.6): held level at 6,000 on
    /// the LAM leg, 27.4 nm from HEMEL, the book's 2,000 fpm predicted
    /// 17,668 and "about 350 feet under"; the follower's own climb was
    /// nothing, and it crossed at 14,587. Judged on the climb flown, the
    /// line is the level being held; a clearance's hold is not judged.
    #[test]
    fn a_star_window_is_judged_on_the_climb_flown() {
        let recs: Vec<serde_json::Value> =
            include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/follower/egkk_lam5p_held_6000.jsonl"))
                .lines()
                .filter_map(|l| serde_json::from_str(l).ok())
                .collect();
        let said = recs.iter().find(|r| r["kind"] == "star_window_unable").unwrap();
        let f = |k: &str| said[k].as_f64().unwrap();
        let (alt, dist, t) = (f("alt_ft"), f("dist_nm"), f("t"));
        // The groundspeed the prediction was made on, from its own numbers.
        let gs = dist / ((f("predicted_ft") - alt) / 2_000.0) * 60.0;
        assert!((250.0..320.0).contains(&gs), "{gs}");
        let tick = recs
            .iter()
            .filter(|r| r["kind"] == "leg")
            .min_by(|a, b| {
                let d = |r: &serde_json::Value| (r["t"].as_f64().unwrap() - t).abs();
                d(a).total_cmp(&d(b))
            })
            .unwrap();
        assert_eq!(tick["want_alt"].as_f64(), Some(6_000.0), "held at 6,000");
        let vs_cmd = tick["vs_cmd"].as_f64().unwrap();
        assert!(vs_cmd.abs() < 100.0, "{vs_cmd}");

        let (mut legs, cons) = egbb_heme1b();
        apply_arrival_constraints(&mut legs, &cons);
        let hemel = legs[1].fix.clone().unwrap();
        let (lat, lon) = crate::nav::project_latlon(hemel.lat, hemel.lon, 110.0, dist);
        let cruise = Some(18_000.0);
        let book = star_window_miss(&legs, &cons, 1, lat, lon, alt, gs, 2_000.0, 2_000.0, cruise)
            .expect("the log's line");
        assert!((book.by_ft - f("by_ft")).abs() < 50.0, "{book:?}");
        let climb = window_climb_fpm(vs_cmd, false).unwrap();
        let flown = star_window_miss(&legs, &cons, 1, lat, lon, alt, gs, climb, 2_000.0, cruise)
            .expect("held level, it is missed by the whole climb");
        assert!((flown.predicted_ft - alt).abs() < 100.0, "{flown:?}");
        assert_eq!(flown.line(), "We'll cross HEMEL about 12,000 feet under its 18,000 feet.");
        assert_eq!(window_climb_fpm(-300.0, false), Some(0.0));
        assert_eq!(window_climb_fpm(1_800.0, false), Some(1_800.0));
        assert_eq!(window_climb_fpm(1_800.0, true), None, "the clearance's level");
    }

    #[test]
    fn a_stars_speed_limits_start_in_time_and_hold_after() {
        let (legs, cons) = star_route();
        // 20 nm short of HAVEN at 290 kt: nothing yet.
        assert_eq!(arrival_speed_cap(&legs, &cons, 1, 10.0 / 60.0, 0.0, 290.0, 400.0, &mut Default::default()), None);
        // 40 kt to lose at 0.5 kt/s is 80 s, 8.9 nm at 400 kt, plus 2.
        let near = arrival_speed_cap(&legs, &cons, 1, 20.0 / 60.0, 0.0, 290.0, 400.0, &mut Default::default());
        assert_eq!(near, Some((250.0, "HAVEN".to_string())));
        // Past HAVEN the 250 holds; nearing TARTN its 230 takes over.
        assert_eq!(arrival_speed_cap(&legs, &cons, 2, 31.0 / 60.0, 0.0, 250.0, 300.0, &mut Default::default()).unwrap().0, 250.0);
        assert_eq!(arrival_speed_cap(&legs, &cons, 3, 66.0 / 60.0, 0.0, 250.0, 300.0, &mut Default::default()).unwrap().0, 230.0);
    }

    fn leg(fix_ident: &str, lat: f64, lon: f64) -> ProcLeg {
        ProcLeg {
            seq: 0,
            transition: String::new(),
            leg_type: "TF".to_string(),
            fix_ident: fix_ident.to_string(),
            fix_region: "LE".to_string(),
            fix: Some(Fix {
                ident: fix_ident.to_string(),
                lat,
                lon,
            }),
            course_mag: None,
            dist_nm: None,
            turn: None,
            alt_desc: ' ',
            alt1_ft: None,
            alt2_ft: None,
            is_faf: false,
            is_map: false,
            vpa_deg: None,
            navaid_ident: String::new(),
            navaid_region: String::new(),
            navaid: None,
            path: Default::default(),
        }
    }

    /// The leg to JRZ as the LEMG-LEJR plan carried it: a fix 55.9 nm out with
    /// a 2,700 ft constraint, armed at 8,000 ft in the cruise.
    fn jrz() -> ProcLeg {
        let mut l = leg("JRZ", 36.7446, -6.0601);
        l.alt1_ft = Some(2700.0);
        l
    }

    /// The loop's own tick, at the 135 kt the sortie was flown at.
    fn tick(
        leg: &ProcLeg,
        carry: f64,
        prev: f64,
        alt: f64,
        dist_nm: f64,
        pilot: Option<f64>,
        floor: Option<f64>,
    ) -> Vertical {
        vertical_tick(
            leg,
            carry,
            prev,
            alt,
            Some(dist_nm),
            135.0,
            pilot,
            floor,
            GA_VS_FPM,
            DescentProfile::none(),
        )
    }

    /// THE bug of 2026-09-13: the descent must WAIT for its top of descent on
    /// every tick, not only the first of the leg. Ticking the real bookkeeping
    /// forward the way the loop does is the whole test: `descent_target_ft`
    /// alone always passed.
    #[test]
    fn the_descent_waits_for_its_top_of_descent_tick_after_tick() {
        let leg = jrz();
        let (mut carry, mut prev) = (8000.0, 8000.0);
        // Cruise, 55.9 nm to run: 5,300 ft at 700 fpm and 135 kt needs ~18 nm.
        for _ in 0..20 {
            let v = tick(&leg, carry, prev, 8000.0, 55.9, None, None);
            assert_eq!(v.want_alt, 8000.0, "held the cruise to the top of descent");
            carry = v.carry;
            prev = v.prev_cmd;
        }
        // Inside it, down the path and staying on it: under the cruise,
        // above the constraint, lower each tick, the constraint at the fix.
        let v = tick(&leg, carry, prev, 8000.0, 17.0, None, None);
        assert!(v.want_alt < 8000.0 && v.want_alt > 2700.0, "{}", v.want_alt);
        let w = tick(&leg, v.carry, v.prev_cmd, 7000.0, 15.0, None, None);
        assert!(w.want_alt < v.want_alt && w.want_alt > 2700.0, "{}", w.want_alt);
        let x = tick(&leg, w.carry, w.prev_cmd, 2900.0, 1.0, None, None);
        assert_eq!(x.want_alt, 2700.0);
    }

    /// The ground outranks the plan. Grazalema probes at 3,861 ft under the
    /// leg that wanted 2,700: the follower flies the floor, and says so.
    #[test]
    fn terrain_ahead_floors_the_plans_constraint() {
        let floor = 3861.0 + TERRAIN_CLEARANCE_FT;
        let v = tick(&jrz(), 2700.0, 2700.0, 4000.0, 38.0, None, Some(floor));
        assert_eq!(v.want_alt, floor);
        assert_eq!(v.floored, Some(floor));
        // And the floor is what carries to the next tick, so the climb holds.
        assert_eq!(v.prev_cmd, floor);
        // Low ground under a high target changes nothing.
        let v = tick(&jrz(), 8000.0, 8000.0, 8000.0, 55.9, None, Some(1300.0));
        assert_eq!(v.want_alt, 8000.0);
        assert_eq!(v.floored, None);
    }

    /// A pilot's altitude outranks the plan's constraint: the override that
    /// was silently undone every 500 ms while the aircraft sank to 220 ft AGL.
    #[test]
    fn the_pilots_altitude_outranks_the_plan() {
        let v = tick(&jrz(), 2700.0, 2700.0, 2800.0, 38.0, Some(4000.0), None);
        assert_eq!(v.want_alt, 4000.0);
        assert!(v.pilot);
        // It survives the next tick, and the one after: the plan never takes
        // the altitude back on its own.
        let v = tick(
            &jrz(),
            v.carry,
            v.prev_cmd,
            3200.0,
            36.0,
            Some(4000.0),
            None,
        );
        assert_eq!(v.want_alt, 4000.0);
        let v = tick(
            &jrz(),
            v.carry,
            v.prev_cmd,
            4000.0,
            34.0,
            Some(4000.0),
            None,
        );
        assert_eq!(v.want_alt, 4000.0);
    }

    /// ...but the ground outranks the PILOT too. Four thousand feet was itself
    /// into the ridge; the follower holds the floor, not the number asked for.
    #[test]
    fn terrain_outranks_the_pilot_as_well() {
        let floor = 3861.0 + TERRAIN_CLEARANCE_FT;
        let v = tick(
            &jrz(),
            2700.0,
            2700.0,
            2800.0,
            38.0,
            Some(4000.0),
            Some(floor),
        );
        assert_eq!(v.want_alt, floor);
        assert_eq!(v.floored, Some(floor));
        assert!(!v.pilot, "the floor set it, not the pilot");
    }

    /// The LIEA-LEBL navlog's descent as the B738 flew it (sortie
    /// 1790494983-full_auto, 2026-09-27): each leg's plan altitude and its
    /// distance at the arm, from the `procedure` `leg` ticks.
    fn liea_lebl_descent() -> Vec<(ProcLeg, f64)> {
        [
            ("KIFMI", 31_000.0, 83.1),
            ("ELVOP", 28_000.0, 28.5),
            ("MUREN", 28_000.0, 5.7),
            ("CHELY", 29_000.0, 8.1),
            ("VERSO", 25_000.0, 34.9),
        ]
        .into_iter()
        .map(|(id, alt, d)| {
            let mut l = leg(id, 41.0, 3.0);
            l.alt1_ft = Some(alt);
            (l, d)
        })
        .collect()
    }

    /// Flies `legs` through the pure vertical the way the loop does (half a
    /// second a tick, the aircraft following the target at the airframe's
    /// rate) and returns, per leg, the highest and lowest altitude commanded
    /// and the held record, if any. `with_profile` false is the old law.
    fn fly_descent(legs: &[(ProcLeg, f64)], with_profile: bool) -> Vec<(f64, f64, Option<(f64, f64)>)> {
        const RATE: f64 = 2_000.0;
        const GS: f64 = 420.0;
        let plain: Vec<ProcLeg> = legs.iter().map(|(l, _)| l.clone()).collect();
        let cruise = plain.iter().filter_map(|l| l.alt1_ft).fold(f64::MIN, f64::max);
        let top = plan_top_index(&plain, cruise);
        let mut alt = 31_000.0;
        let (mut carry, mut prev, mut peak) = (alt, alt, alt);
        let mut out = Vec::new();
        for (idx, (l, dist)) in legs.iter().enumerate() {
            let (mut hi, mut lo, mut held) = (f64::MIN, f64::MAX, None);
            let mut d = *dist;
            while d > 0.2 {
                let profile = DescentProfile {
                    cruise_ft: with_profile.then_some(cruise),
                    peak_ft: peak,
                    past_top: top.is_some_and(|t| idx > t),
                    arrival_ceiling: false,
                };
                let v = vertical_tick(l, carry, prev, alt, Some(d), GS, None, None, RATE, profile);
                carry = v.carry;
                prev = v.prev_cmd;
                peak = v.peak_ft;
                held = held.or(v.descent_held);
                hi = hi.max(v.want_alt);
                lo = lo.min(v.want_alt);
                let step = RATE / 120.0;
                alt += (v.want_alt - alt).clamp(-step, step);
                d -= GS / 7200.0;
            }
            out.push((hi, lo, held));
        }
        out
    }

    /// THE bug of 2026-09-27: down from 31,000 and level at MUREN's 28,000,
    /// CHELY's navlog 29,000 was climbed to (28,999 by 08:28:37). It is held
    /// at 28,000 and said once; VERSO's 25,000 then descends as before.
    #[test]
    fn a_descent_never_climbs_back_to_a_navlog_altitude() {
        let legs = liea_lebl_descent();
        let old = fly_descent(&legs, false);
        assert_eq!(old[3].0, 29_000.0, "the test must bite: the old law climbed to CHELY");
        let new = fly_descent(&legs, true);
        assert_eq!(new[2].1, 28_000.0, "MUREN levelled at 28,000");
        assert_eq!(new[3].0, 28_000.0, "CHELY held at the level already commanded");
        assert_eq!(new[3].2, Some((29_000.0, 28_000.0)), "plan_ft and held_ft for the blackbox");
        assert_eq!(new[4].1, 25_000.0, "VERSO descends");
        assert!(new[4].0 <= 28_000.0, "and never above the held level: {}", new[4].0);
        assert_eq!(new[4].2, None);
    }

    /// The profile rule, one tick: past the plan's top, 28,000 commanded.
    fn in_descent(pilot: Option<f64>, floor: Option<f64>) -> Vertical {
        let mut chely = leg("CHELY", 41.0, 3.0);
        chely.alt1_ft = Some(29_000.0);
        let profile = DescentProfile { cruise_ft: Some(31_000.0), peak_ft: 31_000.0, past_top: true, ..Default::default() };
        vertical_tick(&chely, 28_000.0, 28_000.0, 28_000.0, Some(8.0), 420.0, pilot, floor, 2_000.0, profile)
    }

    /// A pilot or controller asking for a climb in the descent still gets it.
    #[test]
    fn a_pilot_handed_climb_in_the_descent_still_climbs() {
        let v = in_descent(Some(30_000.0), None);
        assert_eq!(v.want_alt, 30_000.0);
        assert!(v.pilot);
        assert_eq!(v.descent_held, None);
    }

    /// A climb for the ground is always right, descent or not.
    #[test]
    fn a_terrain_floor_above_the_held_level_still_climbs() {
        let v = in_descent(None, Some(28_600.0));
        assert_eq!(v.want_alt, 28_600.0);
        assert_eq!(v.floored, Some(28_600.0));
        assert_eq!(in_descent(None, None).want_alt, 28_000.0);
    }

    /// A step climb BEFORE the top of descent is a climb: 29,000 then 31,000.
    #[test]
    fn a_step_climb_before_the_top_of_descent_still_climbs() {
        let mut first = leg("AAAAA", 41.0, 3.0);
        first.alt1_ft = Some(29_000.0);
        let mut step = leg("BBBBB", 42.0, 3.0);
        step.alt1_ft = Some(31_000.0);
        let legs = vec![first, step.clone()];
        let top = plan_top_index(&legs, 31_000.0);
        assert_eq!(top, Some(1));
        let profile = DescentProfile {
            cruise_ft: Some(31_000.0),
            peak_ft: 29_000.0,
            past_top: top.is_some_and(|t| 1 > t),
            arrival_ceiling: false,
        };
        let v = vertical_tick(&step, 29_000.0, 29_000.0, 29_000.0, Some(60.0), 420.0, None, None, 2_000.0, profile);
        assert_eq!(v.want_alt, 31_000.0);
        assert_eq!(v.descent_held, None);
        assert_eq!(v.peak_ft, 29_000.0, "the peak is what was flown");
    }

    /// LFBD-EGKK, 2026-09-27 (sortie 1790510400-full_auto): the CNA6R from
    /// runway 29 as the follower flew it. DER29 and BD923 carry the plan's
    /// FL340, the CA leg between them its chart's "at or above 660".
    fn lfbd_cna6r() -> (ProcLeg, ProcLeg, ProcLeg) {
        let mut der = leg("DER29", 44.83, -0.72);
        der.leg_type = "DF".to_string();
        der.alt1_ft = Some(34_000.0);
        let ca = to_alt('+', 660.0);
        let mut bd923 = leg("BD923", 44.93, -0.80);
        bd923.leg_type = "DF".to_string();
        bd923.alt1_ft = Some(34_000.0);
        (der, ca, bd923)
    }

    /// THE bug of LFBD 2026-09-27: climbing through 7,638 ft on the CA leg
    /// with FL340 commanded since the runway, the terrain floor at 705 ft
    /// (the probe's ground plus 1,000). The CA's 660 dialled the aircraft
    /// down, the floor made it 705, and BD923's FL340 was then held at 705
    /// as a "descent that never climbs back". Both legs keep climbing now.
    #[test]
    fn a_climb_never_descends_to_a_minimum_it_has_passed() {
        let (der, ca, bd923) = lfbd_cna6r();
        let legs = vec![der.clone(), ca.clone(), bd923.clone()];
        let cruise = 34_000.0;
        let top = plan_top_index(&legs, cruise);
        let profile = |idx: usize, peak: f64| DescentProfile {
            cruise_ft: Some(cruise),
            peak_ft: peak,
            past_top: top.is_some_and(|t| idx > t),
            arrival_ceiling: false,
        };
        let floor = Some(705.0);
        let v0 = vertical_tick(&der, 34_000.0, 34_000.0, 7_618.0, Some(0.76), 265.0, None, floor, 2_000.0, profile(0, 7_618.0));
        assert_eq!(v0.want_alt, 34_000.0);
        let v1 = vertical_tick(&ca, v0.carry, v0.prev_cmd, 7_638.0, None, 265.0, None, floor, 2_000.0, profile(1, v0.peak_ft));
        assert_eq!(v1.want_alt, 34_000.0, "the CA's 660 is a minimum passed long ago");
        assert_eq!(v1.floored, None, "the floor is only ever a minimum");
        assert_eq!(v1.climb_held, Some((660.0, 34_000.0)), "plan_ft and held_ft for the blackbox");
        let v2 = vertical_tick(&bd923, v1.carry, v1.prev_cmd, 7_658.0, Some(7.4), 265.0, None, floor, 2_000.0, profile(2, v1.peak_ft));
        assert_eq!(v2.want_alt, 34_000.0, "BD923 climbs on to the cruise");
        assert!(v2.want_alt >= v1.prev_cmd);
        assert_eq!(v2.descent_held, None);
        assert_eq!(v2.climb_held, None);
    }

    /// The same tick without the profile: the test must bite. The CA's 660
    /// goes through and the floor is what stops the dive.
    #[test]
    fn the_lfbd_climb_bit_without_the_profile() {
        let (_, ca, _) = lfbd_cna6r();
        let v = vertical_tick(&ca, 34_000.0, 34_000.0, 7_638.0, None, 265.0, None, Some(705.0), 2_000.0, DescentProfile::none());
        assert_eq!(v.want_alt, 705.0, "without the profile the CA dials the aircraft down");
    }

    /// FL340 commanded but only 7,700 ft flown is not the cruise: not in the
    /// descent, still climbing. Flown to within a step of it, it is.
    #[test]
    fn a_cruise_commanded_but_not_flown_is_not_descending() {
        let climb = DescentProfile { cruise_ft: Some(34_000.0), peak_ft: 7_700.0, past_top: false, arrival_ceiling: false };
        assert!(!climb.descending(705.0));
        assert!(!climb.descending(34_000.0));
        assert!(climb.climbing());
        let cruised = DescentProfile { peak_ft: 33_950.0, ..climb };
        assert!(cruised.descending(31_000.0));
        assert!(!cruised.climbing());
        // Past the plan's top it is the descent whatever was flown.
        let past = DescentProfile { past_top: true, ..climb };
        assert!(past.descending(7_700.0));
        assert!(!past.climbing());
    }

    /// An arrival ceiling before the cruise was reached (a short flight, a
    /// slow climb) is the plan's descent, not a dip: it is flown.
    #[test]
    fn an_arrival_ceiling_in_the_climb_is_flown() {
        let mut fix = leg("REVTU", 51.0, 0.0);
        fix.alt1_ft = Some(26_600.0);
        let profile = DescentProfile { cruise_ft: Some(34_000.0), peak_ft: 30_000.0, past_top: false, arrival_ceiling: true };
        let v = vertical_tick(&fix, 34_000.0, 34_000.0, 30_000.0, Some(5.0), 420.0, None, None, 2_000.0, profile);
        assert_eq!(v.climb_held, None);
        assert!(v.want_alt < 30_000.0, "the ceiling is paced down to: {}", v.want_alt);
    }

    /// A follower armed mid-descent (a resume) has never commanded the
    /// cruise, but past the plan's top it is in the descent all the same.
    #[test]
    fn a_follower_armed_in_the_descent_holds_too() {
        let mut chely = leg("CHELY", 41.0, 3.0);
        chely.alt1_ft = Some(29_000.0);
        let profile = DescentProfile { cruise_ft: Some(31_000.0), peak_ft: 28_000.0, past_top: true, ..Default::default() };
        let v = vertical_tick(&chely, 28_000.0, 28_000.0, 28_000.0, Some(8.0), 420.0, None, None, 2_000.0, profile);
        assert_eq!(v.want_alt, 28_000.0);
        // Not past the top and never at the cruise: a climb.
        let early = DescentProfile { past_top: false, ..profile };
        let v = vertical_tick(&chely, 28_000.0, 28_000.0, 28_000.0, Some(8.0), 420.0, None, None, 2_000.0, early);
        assert_eq!(v.want_alt, 29_000.0);
    }

    /// The probe looks along the TRACK, not at the aircraft: the first point is
    /// ahead of us and the last is the full look-ahead out.
    #[test]
    fn look_ahead_points_walk_down_the_track() {
        let pts = look_ahead_points(36.6354, -5.2307, 285.0, 15.0, TERRAIN_POINTS);
        assert_eq!(pts.len(), TERRAIN_POINTS);
        let d = |(a, b): (f64, f64)| crate::nav::haversine_nm(36.6354, -5.2307, a, b);
        assert!(d(pts[0]) < 1.5, "first point is close ahead: {}", d(pts[0]));
        let last = d(*pts.last().unwrap());
        assert!(
            (last - 15.0).abs() < 0.5,
            "last point is the look-ahead: {last}"
        );
        // Heading 285 is west and slightly north of the aircraft.
        assert!(pts.last().unwrap().1 < -5.2307);
        assert!(pts.last().unwrap().0 > 36.6354);
    }

    fn to_alt(desc: char, alt: f64) -> ProcLeg {
        ProcLeg {
            seq: 0,
            transition: String::new(),
            leg_type: "CA".to_string(),
            fix_ident: String::new(),
            fix_region: String::new(),
            fix: None,
            course_mag: Some(315.0),
            dist_nm: None,
            turn: None,
            alt_desc: desc,
            alt1_ft: Some(alt),
            alt2_ft: None,
            is_faf: false,
            is_map: false,
            vpa_deg: None,
            navaid_ident: String::new(),
            navaid_region: String::new(),
            navaid: None,
            path: Default::default(),
        }
    }

    /// Gibraltar as apt.dat has it: 09 at the west end, 27 at the east.
    fn gibraltar() -> crate::nav::AirportInfo {
        use std::io::Write;
        let apt = std::env::temp_dir().join("xvl-proc-lxgb-apt.dat");
        let mut f = std::fs::File::create(&apt).unwrap();
        writeln!(f, "1 11 0 0 LXGB Gibraltar").unwrap();
        writeln!(f, "100 46 0 0 0 0 0 0 09 36.1509000 -5.3581000 0 0 0 0 0 0 27 36.1515000 -5.3412000 0 0 0 0 0 0").unwrap();
        f.flush().unwrap();
        crate::nav::find_airport(apt.to_str().unwrap(), "LXGB")
            .unwrap()
            .unwrap()
    }

    // The crash of 2026-09-11: 09D05 sits 3.6 km abeam the 09 centreline with
    // 5.5 km to run. Handing autoland the aircraft there is not an approach.
    #[test]
    fn a_faf_autoland_cannot_intercept_from_is_refused() {
        let airport = gibraltar();
        let mut faf = leg("09D05", 36.104666667, -5.405333333);
        faf.is_faf = true;
        let legs = vec![
            leg("09D06", 36.088, -5.404666667),
            faf,
            leg("RW09", 36.1509, -5.3581),
        ];
        let why = faf_off_course(&legs, &airport, "09").expect("refused");
        assert!(
            why.contains("09D05") && why.contains("off the runway zero nine centreline"),
            "{why}"
        );

        // A FAF ON the centreline 4 nm out is fine.
        let mut faf = leg("09D04", 36.1482, -5.4406);
        faf.is_faf = true;
        let legs = vec![faf, leg("RW09", 36.1509, -5.3581)];
        assert_eq!(faf_off_course(&legs, &airport, "09"), None);

        // Nothing to judge: no legs, or a runway the field doesn't have.
        assert_eq!(faf_off_course(&[], &airport, "09"), None);
        assert_eq!(faf_off_course(&legs, &airport, "18"), None);
    }

    // An approach plan holds the clean approach speed down the straight-in;
    // before this the throttle stayed at cruise power and the FAF handoff was
    // at 195 kt on a 100 kt approach (2026-09-11).
    #[test]
    fn an_approach_plan_holds_the_clean_approach_speed() {
        let approach = EndAction::Approach {
            apt_dat: String::new(),
            cfg: ApConfig::default(),
            approach_ias: 100.0,
            rwy_id: None,
        };
        let held = power_hold_target(&approach, None).expect("an approach holds a speed");
        assert!((140.0..=150.0).contains(&held), "1.45 x Vapp, got {held}");
        assert_eq!(
            power_hold_target(&EndAction::Enroute, Some(180.0)),
            Some(180.0)
        );
        assert_eq!(power_hold_target(&EndAction::Enroute, None), None);
        assert_eq!(power_hold_target(&EndAction::Level, Some(180.0)), None);
    }

    #[test]
    fn steers_the_great_circle_bearing_to_the_fix() {
        // A fix due east: bearing ~90.
        let l = leg("EAST", 36.0, -4.0);
        let brg = bearing_to_fix(36.0, -5.0, &l).unwrap();
        assert!((brg - 90.0).abs() < 1.0, "{brg}");
        // A to-altitude leg has no fix to steer at.
        assert!(bearing_to_fix(36.0, -5.0, &to_alt('+', 3000.0)).is_none());
    }

    #[test]
    fn a_fix_leg_captures_inside_the_radius() {
        let l = leg("F", 36.0, -5.0);
        assert!(
            !leg_captured(&l, Some(1.5), 2000.0, CAPTURE_NM),
            "far: not captured"
        );
        assert!(
            leg_captured(&l, Some(CAPTURE_NM - 0.1), 2000.0, CAPTURE_NM),
            "inside: captured"
        );
    }

    /// The EIDW departure that orbited (2026-09-20). The 737 climbing at
    /// 250 kt turns with a radius bigger than the gap between the SID's
    /// fixes, so the flat 0.8 nm capture was a circle the aircraft could
    /// never enter: it banked 25°, went round, and the distance to the fix
    /// GREW (4.5, 6.7, 7.6 nm) until the stall watchdog stood the follower
    /// down. The capture has to be at least the turn the fix demands.
    /// The BIKF arrival (2026-09-20), which the shared 700 fpm could not fly.
    /// Every leg inbound of BASLU wanted about 2,000 fpm; at 700 the aircraft
    /// would have reached the field some 17,000 ft high, and the approach
    /// would never have armed.
    /// A heading written with the WRONG variation is a wrong heading, and the
    /// roll-out capture is where it does the damage: "bug on the nose" with a
    /// stale variation is not a capture at all, it is a standing turn command.
    ///
    /// BIKF-EGPH (2026-09-20): departed Keflavik (about 11 degrees west),
    /// arrived over Scotland (under one). The variation was read once before
    /// the loop, so every tick wrote a bug 11.4 degrees right of the nose —
    /// the log shows `bug - psi` pinned at exactly 11.4 for the whole orbit
    /// while the crab beside it varied, which is how a constant offset tells
    /// itself apart from a control error.
    #[test]
    fn a_stale_magnetic_variation_turns_a_capture_into_a_turn() {
        // The roll-out branch commands the nose. With the right variation
        // that is zero error and the autopilot rolls level.
        let (psi_true, here) = (142.2, 0.78);
        let correct = true_to_mag(psi_true, here);
        assert!(
            (crate::heading_bug::bug_toward(correct, 122.2, correct, 25.0, 40.0) - correct).abs()
                < 0.01,
            "on the nose is a capture"
        );
        // With Keflavik's variation still in hand — `true_to_mag` is
        // positive-WEST, so Iceland's is the LARGER number — it is 11.4
        // degrees of standing right turn, the same way the aircraft is
        // already banked. Which is exactly what it flew.
        let stale = true_to_mag(psi_true, here + 11.4);
        let commanded = crate::heading_bug::bug_toward(stale, 122.2, stale, 25.0, 40.0);
        let error = crate::angles::signed_diff_deg(commanded, correct);
        assert!(
            (error - 11.4).abs() < 0.1,
            "a standing turn command, not a capture: {error:.1} degrees"
        );
    }

    #[test]
    fn a_jet_descent_is_paced_at_the_jets_rate_not_the_ga_one() {
        // BASLU at 27,700 to the field, 78 nm, 340 kt over the ground.
        let (drop_ft, dist_nm, gs) = (27_529.0, 78.0, 340.0);
        let reach = |fpm: f64| dist_nm / gs * 60.0 * fpm;
        assert!(
            reach(GA_VS_FPM) < drop_ft * 0.5,
            "the GA rate cannot lose half of it: {:.0} of {drop_ft:.0} ft",
            reach(GA_VS_FPM)
        );
        assert!(
            reach(2000.0) >= drop_ft,
            "the jet's rate makes it: {:.0} ft",
            reach(2000.0)
        );
        // ...and the top of descent moves with the rate, so the point and the
        // rate can never disagree: three times the rate, a third of the run.
        let ga = descent_point_nm(6_000.0, 34_000.0, 340.0, GA_VS_FPM).unwrap();
        let jet = descent_point_nm(6_000.0, 34_000.0, 340.0, 2000.0).unwrap();
        let ratio = (ga - TOD_LEAD_NM) / (jet - TOD_LEAD_NM);
        assert!(
            (ratio - 2000.0 / GA_VS_FPM).abs() < 0.01,
            "TOD scales inversely with the rate: {ga:.0} nm vs {jet:.0} nm (x{ratio:.2})"
        );
        // Concretely: the GA rate would start this jet down 228 nm out, which
        // is before the fix it is descending TO.
        // (Planned at 75% of each rate since 2026-09-26: the jet's point is
        // about 100 nm out, still well inside the GA one.)
        assert!(ga > 250.0 && jet < 110.0, "{ga:.0} nm vs {jet:.0} nm");
    }

    /// The rates are the airframe's, republished on every swap, and a profile
    /// that declares nothing keeps the GA figure it always had.
    #[test]
    fn the_vertical_rates_are_published_per_airframe() {
        let _g = crate::mode::TEST_LOCK.lock();
        let mut r = crate::aircraft::ReferenceProfile::default();
        publish_vs_limits(Some(&r));
        assert_eq!(descent_fpm(), GA_VS_FPM, "silent profile keeps the default");
        assert_eq!(climb_fpm(), GA_VS_FPM);
        r.enroute_climb_fpm = Some(2000.0);
        r.enroute_descent_fpm = Some(2000.0);
        publish_vs_limits(Some(&r));
        assert_eq!(descent_fpm(), 2000.0);
        assert_eq!(climb_fpm(), 2000.0);
        assert_eq!(vs_clamp_fpm(), 2000.0, "the spoken tool promises the truth");
        // A swap to an airframe with no figures must not inherit the jet's:
        // a process-wide fact never outlives its airframe.
        publish_vs_limits(None);
        assert_eq!(descent_fpm(), GA_VS_FPM);
        assert_eq!(climb_fpm(), GA_VS_FPM);
        // A nonsense figure is refused rather than flown.
        r.enroute_descent_fpm = Some(-50.0);
        publish_vs_limits(Some(&r));
        assert_eq!(descent_fpm(), GA_VS_FPM);
    }

    #[test]
    fn a_jet_sequences_a_fix_it_could_never_have_closed_on() {
        let gs = 280.0;
        let r = enroute_turn_radius_nm(gs);
        assert!(
            r > 2.0,
            "a jet's enroute turn is nautical miles, not the pattern clamp's 1.35: {r:.2}"
        );
        // DE28R -> DW128 -> DW991 are 2.9 and 3.9 nm apart. A 60 degree turn
        // at that radius must be started well outside the old flat radius.
        let cap = capture_radius_nm(gs, 60.0);
        assert!(
            cap > CAPTURE_NM,
            "the turn is anticipated, not chased: {cap:.2} nm"
        );
        assert!(cap <= r, "and never leads by more than the radius itself");
        let l = leg("DW128", 53.4657, -6.3523);
        // 1.5 nm out with the turn already due (the anticipation is 1.81 nm
        // here): still "not reached" under the old flat radius, which is how
        // the follower ended up chasing it round; sequenced now.
        assert!(!leg_captured(&l, Some(1.5), 6000.0, CAPTURE_NM), "the bug");
        assert!(leg_captured(&l, Some(1.5), 6000.0, cap), "the fix");
    }

    /// ...and the airframes that were already right stay byte-identical: in
    /// both cases the anticipation lands under the floor, so the floor is
    /// still the whole answer.
    #[test]
    fn turn_anticipation_never_disturbs_a_leg_that_already_worked() {
        // A 172 at 100 knots, turning 90 degrees — the sharpest thing a
        // light aircraft meets on a procedure.
        assert_eq!(capture_radius_nm(100.0, 90.0), CAPTURE_NM);
        // A jet at cruise: an 8 nm radius, but a 10 degree track change.
        assert_eq!(capture_radius_nm(450.0, 10.0), CAPTURE_NM);
        // Dead straight is always the floor, whatever the speed.
        assert_eq!(capture_radius_nm(450.0, 0.0), CAPTURE_NM);
    }

    /// The turn is measured from where we ARE into the fix, then on to the
    /// next one — so a leg with nothing after it anticipates nothing.
    #[test]
    fn the_turn_angle_comes_from_the_two_legs_it_joins() {
        let here = leg("A", 53.5, -6.0);
        let next = leg("B", 53.5, -5.0); // due east of A
                                         // Approaching A from due west: the turn onto B is straight ahead.
        let straight = leg_turn_deg(53.5, -7.0, &here, Some(&next)).unwrap();
        assert!(straight.abs() < 1.0, "no turn: {straight:.1}");
        // Approaching A from the south: a left turn onto the eastbound leg.
        let left = leg_turn_deg(52.5, -6.0, &here, Some(&next)).unwrap();
        assert!(
            (left - 90.0).abs() < 2.0,
            "a 90 degree turn, right-positive: {left:.1}"
        );
        assert_eq!(leg_turn_deg(53.5, -7.0, &here, None), None, "last leg");
        assert_eq!(
            leg_turn_deg(53.5, -7.0, &to_alt('+', 3000.0), Some(&next)),
            None,
            "a to-altitude leg has no fix to turn at"
        );
    }

    #[test]
    fn a_climb_leg_captures_at_or_above_the_altitude() {
        let climb = to_alt('+', 3000.0);
        assert!(
            !leg_captured(&climb, None, 2000.0, CAPTURE_NM),
            "below: keep climbing"
        );
        assert!(leg_captured(&climb, None, 3000.0, CAPTURE_NM), "made it");
        assert!(
            leg_captured(&climb, None, 2900.0, CAPTURE_NM),
            "within margin counts"
        );
    }

    #[test]
    fn a_descend_leg_captures_at_or_below_the_altitude() {
        let down = to_alt('-', 3000.0);
        assert!(
            !leg_captured(&down, None, 5000.0, CAPTURE_NM),
            "above: keep descending"
        );
        assert!(leg_captured(&down, None, 3000.0, CAPTURE_NM), "made it");
    }

    #[test]
    fn terrain_floor_is_converted_to_the_altimeter() {
        // Hot day: 7,012 true reads 6,838. A 6,500 true floor must be flown
        // as 6,326 indicated, not 6,500 (174 ft into the clearance).
        assert!((floor_indicated(6_500.0, 7_012.0, 6_838.0) - 6_326.0).abs() < 1e-9);
        // Cold day the other way: the indicated floor sits ABOVE the true one.
        assert!((floor_indicated(6_500.0, 6_800.0, 7_000.0) - 6_700.0).abs() < 1e-9);
        // Standard day: no change.
        assert_eq!(floor_indicated(6_500.0, 7_000.0, 7_000.0), 6_500.0);
    }

    #[test]
    fn altitude_carries_across_unconstrained_legs() {
        // A fix leg with no altitude keeps the last dialled value.
        let plain = leg("F", 36.0, -5.0);
        assert_eq!(commanded_alt_ft(&plain, 4500.0), 4500.0);
        // A constrained leg dials its own.
        let mut constrained = leg("G", 36.0, -5.0);
        constrained.alt1_ft = Some(2500.0);
        assert_eq!(commanded_alt_ft(&constrained, 4500.0), 2500.0);
    }

    #[test]
    fn handoff_is_the_faf_when_flagged() {
        let mut legs = vec![
            leg("IF", 36.0, -5.0),
            leg("FAF", 36.1, -5.0),
            leg("RW27", 36.2, -5.0),
        ];
        legs[1].is_faf = true;
        assert_eq!(handoff_index(&legs), Some(1));
    }

    #[test]
    fn handoff_falls_back_to_the_leg_before_the_runway() {
        // No FAF flag: hand off the leg before the RW threshold.
        let legs = vec![
            leg("IF", 36.0, -5.0),
            leg("STEP", 36.1, -5.0),
            leg("RW27", 36.2, -5.0),
        ];
        assert_eq!(handoff_index(&legs), Some(1));
        assert_eq!(handoff_index(&[]), None);
    }

    /// EGKK ILS 26L (sortie 1790510400-full_auto, stock 737, 2026-09-27):
    /// the real CIFP final codes its MAP on RW26L ("G  M"), and the MAP rule
    /// for LSZS handed off THERE: "At the fix. Landing us." at 1 nm and 310 ft
    /// radio, 550 m off the centreline, localizer and glideslope 2.5 dots, a
    /// go-around at 97 ft 244 m off. The handoff is the FAF, FF26L; on the
    /// sortie's tick that sequenced it (the aircraft 4.3 nm out, turning onto
    /// the final from the south) autoland takes it there, not a mile out.
    #[test]
    fn a_runway_map_hands_autoland_the_approach_at_the_faf() {
        let rows = "APPCH:010,I,I26L, ,CF26L,EG,P,C,E  I, ,   ,IF, ,IWW,EG,P,I,      ,0766,0082,    ,    ,J,02000,02000,     , ,   ,    ,   , , , , , ,0,D,S;\n\
                    APPCH:020,I,I26L, ,FF26L,EG,P,C,E  F, ,   ,CF, ,IWW,EG,P,I,      ,0766,0072,2560,0010,H,02000,02000,     , ,   ,-300,   ,EGKK,EG,P,A, ,0,D,S;\n\
                    APPCH:030,I,I26L, ,RW26L,EG,P,G,G  M, ,   ,CF, ,IWW,EG,P,I,      ,0766,0017,2560,0055, ,00248,     ,     , ,   ,-300,   , , , , , ,0,D,S;\n\
                    APPCH:040,I,I26L, , , , , ,  M , ,   ,CA, , , , , ,      ,    ,    ,2560,    ,+,02000,     ,     , ,   ,    ,   , , , , , ,0,D,S;\n\
                    APPCH:050,I,I26L, , , , , , E  , ,   ,VM, , , , , ,      ,    ,    ,1770,    , ,03000,     ,     , ,   ,    ,   , , , , , ,0,D,S;\n";
        let raw = crate::nav::parse_proc_legs(
            rows.lines().map(str::trim),
            crate::nav::ProcKind::Appch,
            "I26L",
            None,
        );
        let mut legs = crate::nav::straight_in_legs(raw, "26L").expect("straight in");
        let at = |ident: &str, lat: f64, lon: f64| Fix {
            ident: ident.into(),
            lat,
            lon,
        };
        legs[0].fix = Some(at("CF26L", 51.173697222, -0.003744444));
        legs[1].fix = Some(at("FF26L", 51.170172222, -0.029625));
        legs[2].fix = Some(at("RW26L", 51.150672, -0.171944));
        assert_eq!(
            legs.iter()
                .map(|l| (l.fix_ident.as_str(), l.is_faf, l.is_map))
                .collect::<Vec<_>>(),
            [
                ("CF26L", false, false),
                ("FF26L", true, false),
                ("RW26L", false, true)
            ]
        );
        assert_eq!(
            handoff_index(&legs),
            Some(1),
            "the FAF, never the runway's MAP"
        );

        // t 1790515473.3: FF26L sequenced, groundspeed 261 kt, track 042.
        let (lat, lon) = (51.141135858, -0.052537624);
        let d = crate::nav::haversine_nm(lat, lon, 51.170172222, -0.029625);
        let cap = capture_radius_nm(
            260.6,
            leg_turn_deg(lat, lon, &legs[1], legs.get(2)).unwrap(),
        );
        assert!(
            leg_done(&legs[1], Some(d), 2914.0, cap, lat, lon),
            "d {d:.2} cap {cap:.2}"
        );
        let thr = crate::nav::haversine_nm(lat, lon, 51.150672, -0.171944);
        assert!(
            thr > 4.0,
            "handed over {thr:.1} nm out, not the 1 nm of the sortie"
        );
    }

    fn wp(ident: &str, lat: f64, lon: f64, alt: f64) -> crate::simbrief::Waypoint {
        crate::simbrief::Waypoint {
            ident: ident.to_string(),
            lat,
            lon,
            alt,
        }
    }

    #[test]
    fn leg_detail_reads_like_a_progress_line() {
        assert_eq!(
            leg_detail("PIMOS", Some(12.34), 120.0, 4820.0, 5497.0, AltOwner::Plan),
            "to PIMOS, 12.3 nm, 6 min, climbing to 5,500 ft"
        );
        assert_eq!(
            leg_detail("", None, 0.0, 5480.0, 5500.0, AltOwner::Plan),
            "level at 5,500 ft"
        );
        assert_eq!(
            leg_detail("BLN", Some(3.0), 10.0, 6000.0, 3000.0, AltOwner::Plan),
            "to BLN, 3.0 nm, descending to 3,000 ft"
        );
        // Not the plan's number: the panel says whose it is.
        assert_eq!(
            leg_detail("JRZ", Some(38.0), 135.0, 2800.0, 4900.0, AltOwner::Terrain),
            "to JRZ, 38.0 nm, 17 min, climbing to 4,900 ft, terrain floor"
        );
        assert_eq!(
            leg_detail("JRZ", Some(38.0), 135.0, 4000.0, 4000.0, AltOwner::Pilot),
            "to JRZ, 38.0 nm, 17 min, level at 4,000 ft, your altitude"
        );
    }

    #[test]
    fn fms_goto_payload_is_ident_lat_lon() {
        assert_eq!(
            fms_goto_payload(" pimos ", 36.4, -5.1234567),
            "PIMOS,36.400000,-5.123457"
        );
    }

    #[test]
    fn enroute_legs_drop_the_airports_and_keep_the_middle() {
        // The Jerez–Gibraltar shape: origin + two fixes + destination.
        let wps = vec![
            wp("LEJR", 36.74, -6.06, 0.0),
            wp("VJF", 36.24, -5.98, 5000.0),
            wp("LINTO", 35.83, -5.95, 5000.0),
            wp("LXGB", 36.15, -5.35, 2500.0),
        ];
        let legs = enroute_legs(&wps, "LEJR", "LXGB", 5000.0);
        assert_eq!(legs.len(), 2);
        assert_eq!(legs[0].fix_ident, "VJF");
        assert_eq!(legs[1].fix_ident, "LINTO");
        assert!(legs.iter().all(|l| l.fix.is_some() && l.leg_type == "TF"));
        assert_eq!(legs[0].alt1_ft, Some(5000.0));
    }

    #[test]
    fn climb_fixes_command_cruise_but_descent_fixes_stand() {
        // A mid-climb fix at 3,000 must not stair-step the departure; the
        // post-TOD 4,000 must be honored (that's the descent).
        let wps = vec![
            wp("AAA", 36.0, -5.0, 3000.0),
            wp("BBB", 36.2, -5.0, 5000.0),
            wp("CCC", 36.4, -5.0, 5000.0),
            wp("DDD", 36.6, -5.0, 4000.0),
        ];
        let legs = enroute_legs(&wps, "LEJR", "LXGB", 5000.0);
        let alts: Vec<f64> = legs.iter().map(|l| l.alt1_ft.unwrap()).collect();
        assert_eq!(alts, vec![5000.0, 5000.0, 5000.0, 4000.0]);
    }

    #[test]
    fn a_fix_a_hair_under_cruise_is_still_cruise() {
        // LEMG-LXGB, 2026-09-11: MG834 charted 3,900 between 4,000 ft fixes.
        let wps = vec![
            wp("MG833", 36.6, -4.4, 4000.0),
            wp("MG834", 36.5, -4.5, 3900.0),
            wp("PIMOS", 36.2, -4.9, 4000.0),
            wp("XXX", 36.1, -5.1, 2500.0),
        ];
        let legs = enroute_legs(&wps, "LEMG", "LXGB", 4000.0);
        let alts: Vec<f64> = legs.iter().map(|l| l.alt1_ft.unwrap()).collect();
        assert_eq!(alts, vec![4000.0, 4000.0, 4000.0, 2500.0]);
    }

    // The Gibraltar arrival: 3,900 ft, first approach fix wanting 1,397 with
    // 25 nm to run at 200 kt over the ground. Level until the top of descent.
    #[test]
    fn a_descent_waits_for_its_top_of_descent() {
        assert_eq!(
            descent_target_ft(1397.0, 3900.0, 3888.0, Some(25.0), 200.0, GA_VS_FPM),
            3900.0
        );
        // TOD: 2,491 ft planned at 525 fpm is 4.7 min, 15.8 nm at 200 kt,
        // plus the lead: level at 17.5, on the path at 12, at the constraint
        // at the fix's lead.
        assert_eq!(
            descent_target_ft(1397.0, 3900.0, 3888.0, Some(17.5), 200.0, GA_VS_FPM),
            3900.0
        );
        let on_path = descent_target_ft(1397.0, 3900.0, 3500.0, Some(12.0), 200.0, GA_VS_FPM);
        assert!((on_path - 3130.0).abs() < 20.0, "{on_path}");
        assert_eq!(
            descent_target_ft(1397.0, 3900.0, 1500.0, Some(1.0), 200.0, GA_VS_FPM),
            1397.0
        );
        // Climbs, small steps and fix-less legs go straight to the constraint.
        assert_eq!(
            descent_target_ft(4000.0, 2000.0, 2000.0, Some(30.0), 200.0, GA_VS_FPM),
            4000.0
        );
        assert_eq!(
            descent_target_ft(3900.0, 4000.0, 3990.0, Some(30.0), 200.0, GA_VS_FPM),
            3900.0
        );
        assert_eq!(
            descent_target_ft(1000.0, 4000.0, 4000.0, None, 200.0, GA_VS_FPM),
            1000.0
        );
        // Never reached the previous target: hold where we are, rounded.
        assert_eq!(
            descent_target_ft(1000.0, 6000.0, 3460.0, Some(40.0), 200.0, GA_VS_FPM),
            3500.0
        );
    }

    #[test]
    fn a_zero_altitude_fix_commands_cruise() {
        let wps = vec![wp("AAA", 36.0, -5.0, 0.0), wp("BBB", 36.2, -5.0, 0.0)];
        let legs = enroute_legs(&wps, "LEJR", "LXGB", 4500.0);
        assert!(legs.iter().all(|l| l.alt1_ft == Some(4500.0)));
    }

    #[test]
    fn a_plan_of_just_airports_yields_no_enroute_legs() {
        let wps = vec![
            wp("LEJR", 36.74, -6.06, 0.0),
            wp("LXGB", 36.15, -5.35, 2500.0),
        ];
        assert!(enroute_legs(&wps, "LEJR", "LXGB", 5000.0).is_empty());
    }

    /// EGLF-LFOS, 2026-10-03 (#214): WAFFU was the final filed fix, 46 nm
    /// from LFOS. Arrival speed and completion must be measured at an arrival
    /// gateway, not at WAFFU; a STAR already inside the arrival area is kept.
    #[test]
    fn a_remote_last_fix_is_extended_to_the_arrival_gate() {
        let destination = wp("LFOS", 49.8335278, 0.6500255, 0.0);
        let mut legs = enroute_legs(
            &[
                wp("EGLF", 51.275, -0.776, 0.0),
                wp("WAFFU", 50.5759, 0.3317, 3000.0),
                destination.clone(),
            ],
            "EGLF",
            "LFOS",
            3000.0,
        );
        assert!(extend_to_arrival_gateway(&mut legs, &destination));
        let gate = legs.last().and_then(|l| l.fix.as_ref()).unwrap();
        assert!(
            (crate::nav::haversine_nm(gate.lat, gate.lon, destination.lat, destination.lon)
                - ARRIVAL_GATEWAY_NM)
                .abs()
                < 0.05
        );
        assert_eq!(legs.last().unwrap().alt1_ft, Some(3000.0));
        assert_eq!(legs.last().unwrap().fix_ident, "LFOS arrival");

        let n = legs.len();
        assert!(!extend_to_arrival_gateway(&mut legs, &destination));
        assert_eq!(legs.len(), n, "an arrival already at the gate is unchanged");
    }

    #[test]
    fn a_tight_circuit_is_not_flyable_but_an_arrival_is() {
        // FAF at (36.0,-5.0), crossing 2500 ft.
        let mut faf = leg("FAF", 36.0, -5.0);
        faf.is_faf = true;
        faf.alt1_ft = Some(2500.0);
        let plan = Plan {
            name: "test".into(),
            legs: vec![faf],
            end: EndAction::Level,
            vy_kts: 0.0,
            cruise_ias_kts: None,
            transition_altitude_ft: None,
            climb_minimums: Vec::new(),
            arrival_constraints: Vec::new(),
            visual_deg: None,
            missed_legs: Vec::new(),
            cruise_schedule: Default::default(),
            arrival_handoff_kts: None,
        };
        // Tight circuit: 0.3 nm from the FAF at 700 ft — NOT flyable (autoland).
        let (near_lat, near_lon) = crate::nav::project_latlon(36.0, -5.0, 0.0, 0.3);
        assert!(!approach_flyable_from(&plan, near_lat, near_lon, 700.0));
        // Arrival: 6 nm out at 3000 ft — flyable.
        let (far_lat, far_lon) = crate::nav::project_latlon(36.0, -5.0, 90.0, 6.0);
        assert!(approach_flyable_from(&plan, far_lat, far_lon, 3000.0));
        // Far out but too low (below platform − 500): not flyable.
        assert!(!approach_flyable_from(&plan, far_lat, far_lon, 1500.0));
    }
    #[test]
    fn a_probe_lost_to_a_pause_does_not_retire_the_terrain_floor() {
        // LXGB-LEAX 2026-09-15: the pause landed inside the probe's wait.
        let mut m = 0;
        assert!(!terrain_probe_fatal(&mut m, true));
        assert_eq!(m, 0, "a frozen miss is not a miss");
        // One live miss is deferred; the second in a row retires the floor.
        assert!(!terrain_probe_fatal(&mut m, false));
        assert!(terrain_probe_fatal(&mut m, false));
        // A frozen miss between two live ones neither counts nor resets.
        let mut m = 1;
        assert!(!terrain_probe_fatal(&mut m, true));
        assert_eq!(m, 1);
        assert!(terrain_probe_fatal(&mut m, false));
    }

    /// The trimmed OFP (EGKK-EGBB, B738, CI 77, FL180; the pilot's plan,
    /// 2026-09-28), its navlog as the fetcher builds the waypoints.
    fn egkk_egbb() -> (Vec<crate::simbrief::Waypoint>, crate::simbrief::CruiseSchedule) {
        let v: serde_json::Value =
            serde_json::from_str(include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/ofp/egkk-egbb-738.trimmed.json"))).unwrap();
        let n = |f: &serde_json::Value, k: &str| f[k].as_str().unwrap().parse::<f64>().unwrap();
        let mut wps: Vec<_> = v["navlog"]["fix"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|f| f["ident"] != "TOC" && f["ident"] != "TOD")
            .map(|f| wp(f["ident"].as_str().unwrap(), n(f, "pos_lat"), n(f, "pos_long"), n(f, "altitude_feet")))
            .collect();
        wps.insert(0, wp("EGKK", 51.148, -0.19, 0.0));
        (wps, crate::simbrief::parse_schedule(&v))
    }

    #[test]
    fn a_planned_mach_is_held_as_the_airspeed_it_is_at_the_level() {
        // The OFP's HEMEL: M0.628 at FL180 planned at 300 KIAS.
        let fl180 = cas_for_mach(0.628, 18_000.0);
        assert!((fl180 - 300.0).abs() < 2.0, "{fl180}");
        // The 737 profile's own reasoning (dev-history 2026-09-19): M0.78
        // at FL360 is about 255 to 260.
        let fl360 = cas_for_mach(0.78, 36_000.0);
        assert!((255.0..=262.0).contains(&fl360), "{fl360}");
        // Above the tropopause too, and at sea level Mach is TAS.
        assert!(cas_for_mach(0.78, 39_000.0) < fl360);
        assert!((cas_for_mach(0.3, 0.0) - 198.4).abs() < 1.0);
    }

    /// #101: the 737's profile flies 255 at every level, chosen to stay under
    /// Mmo at FL360. The OFP planned 300 KIAS / M0.63 for the FL180 cruise of
    /// EGKK-EGBB: the cruise legs are flown at it, the climb and descent
    /// legs are the profile's, and a level other than the plan's stays on
    /// the Mach (the lower of the two).
    #[test]
    fn the_ofp_cruise_speed_is_flown_on_its_cruise_legs() {
        let (wps, sched) = egkk_egbb();
        assert_eq!((sched.cost_index, sched.profile.as_deref()), (Some(77.0), Some("CI 77")));
        let legs = enroute_legs(&wps, "EGKK", "EGBB", 18_000.0);
        let at = |id: &str| legs.iter().find(|l| l.fix_ident == id).unwrap();
        let hemel = scheduled_cruise_kts(&sched, at("HEMEL"), 18_000.0).expect("a cruise leg");
        assert_eq!((hemel.plan_ias_kts, hemel.target_kts), (Some(300.0), 300.0));
        assert_eq!(scheduled_cruise_kts(&sched, at("D156O"), 18_000.0).unwrap().target_kts, 299.0);
        // Climb and descent legs: no schedule, the profile's speed.
        for id in ["DET31", "ACORN", "PUFAX", "HON", "OSKOT"] {
            assert_eq!(scheduled_cruise_kts(&sched, at(id), 12_000.0), None, "{id}");
        }
        // Held at FL280 (a controller's level): the Mach, not the 300.
        let high = scheduled_cruise_kts(&sched, at("HEMEL"), 28_000.0).unwrap().target_kts;
        assert!((240.0..=250.0).contains(&high), "{high}");
        // The 737 (floor about 167, ceiling Vne 340 less 15): the OFP's 300,
        // where the old law held the profile's 255.
        let pick = cruise_pick(255.0, Some(&hemel), Some(167.0), Some(325.0), None);
        assert_eq!(pick, CruisePick { kts: 300.0, source: "ofp", rejected: None, capped_from: None });
        assert_eq!(cruise_pick(255.0, None, Some(167.0), Some(325.0), None).kts, 255.0);
        // A 737 plan in a 172 is never flown at 300: the airframe's number.
        let pick = cruise_pick(108.0, Some(&hemel), Some(58.0), Some(148.0), None);
        assert_eq!(
            pick,
            CruisePick { kts: 108.0, source: "profile", rejected: Some("above_ceiling"), capped_from: None }
        );
        let slow = ScheduledSpeed { target_kts: 90.0, ..hemel.clone() };
        assert_eq!(cruise_pick(255.0, Some(&slow), Some(167.0), None, None).rejected, Some("below_floor"));
        // #156: under the overspeed caution (0.9 x 340 = 306) by the margin:
        // 296, the plan's 300 kept as `capped_from`. The FL180 cruise as
        // flown held 294.2 to 304.2 around its 299 (+2010 to +2100): the same
        // wander over the cap stays out of the caution band.
        let cap = cruise_cap_kts(Some(340.0));
        assert_eq!(cap, Some(296.0));
        let pick = cruise_pick(255.0, Some(&hemel), Some(167.0), Some(325.0), cap);
        assert_eq!(pick, CruisePick { kts: 296.0, source: "ofp", rejected: None, capped_from: Some(300.0) });
        assert!(296.0 + (304.2 - 299.2) < 306.0);
        // Under the cap, or a cap under the floor: the plan's own number.
        let under = ScheduledSpeed { target_kts: 280.0, ..hemel.clone() };
        assert_eq!(
            cruise_pick(255.0, Some(&under), Some(167.0), Some(325.0), cap),
            CruisePick { kts: 280.0, source: "ofp", rejected: None, capped_from: None }
        );
        assert_eq!(cruise_pick(255.0, Some(&hemel), Some(167.0), None, Some(150.0)).kts, 300.0);
        assert_eq!(cruise_cap_kts(None), None);
        // A leg whose ident matches but whose fix is elsewhere is not it.
        let mut far = at("HEMEL").clone();
        far.fix.as_mut().unwrap().lat += 1.0;
        assert_eq!(scheduled_cruise_kts(&sched, &far, 18_000.0), None);
        // One level, no step: the legs are left as built.
        let mut same = legs.clone();
        assert!(apply_step_climbs(&mut same, &sched).is_empty());
        let alts = |l: &[ProcLeg]| l.iter().map(|l| l.alt1_ft).collect::<Vec<_>>();
        assert_eq!(alts(&same), alts(&legs));
    }

    /// An EIDW-BIKF shaped plan (the sector the step-climb advisory's FL360
    /// test flies): FL340 initial, "ERNAN/0360" a step. The navlog shows
    /// 36,000 from the fix after ERNAN, so `enroute_legs` commanded FL360
    /// from the first leg and the step was flown as the initial cruise.
    #[test]
    fn a_planned_step_climb_is_flown_at_its_fix() {
        use crate::simbrief::{CruiseSchedule, NavlogSpeed, Stage, StepClimb};
        let route = [
            ("EIDW", 53.42, -6.27, 0.0, None),
            ("DUB", 53.50, -6.50, 12_000.0, Some(Stage::Climb)),
            ("LIFFY", 53.60, -7.00, 30_000.0, Some(Stage::Climb)),
            ("BAMLI", 54.00, -8.00, 34_000.0, Some(Stage::Cruise)),
            ("ERNAN", 55.00, -10.00, 34_000.0, Some(Stage::Cruise)),
            ("5715N", 57.00, -15.00, 36_000.0, Some(Stage::Cruise)),
            ("6120N", 61.00, -20.00, 36_000.0, Some(Stage::Cruise)),
            ("KEF", 63.90, -22.60, 11_000.0, Some(Stage::Descent)),
            ("BIKF", 63.98, -22.60, 0.0, None),
        ];
        let wps: Vec<_> = route.iter().map(|(i, la, lo, a, _)| wp(i, *la, *lo, *a)).collect();
        let sched = CruiseSchedule {
            cost_index: Some(30.0),
            profile: Some("CI 30".into()),
            fixes: route
                .iter()
                .filter_map(|(i, la, lo, a, st)| {
                    st.map(|stage| NavlogSpeed {
                        ident: i.to_string(),
                        lat: *la,
                        lon: *lo,
                        stage,
                        alt_ft: *a,
                        ias_kts: Some(270.0),
                        mach: Some(0.78),
                    })
                })
                .collect(),
            steps: crate::simbrief::parse_step_climbs("EIDW/0340 ERNAN/0360"),
            tod: None,
        };
        assert_eq!(sched.steps[1], StepClimb { fix: "ERNAN".into(), alt_ft: 36_000.0 });
        let built = enroute_legs(&wps, "EIDW", "BIKF", 34_000.0);
        let alts = |l: &[ProcLeg]| l.iter().map(|l| l.alt1_ft.unwrap()).collect::<Vec<_>>();
        // As built: FL360 from the runway.
        assert_eq!(alts(&built), vec![36_000.0, 36_000.0, 36_000.0, 36_000.0, 36_000.0, 36_000.0, 11_000.0]);
        let mut legs = built.clone();
        let placed = apply_step_climbs(&mut legs, &sched);
        assert_eq!(placed, vec![PlacedStep { fix: "ERNAN".into(), idx: 3, alt_ft: 36_000.0 }]);
        // FL340 to and over ERNAN, FL360 from the leg after it, the descent
        // untouched.
        assert_eq!(alts(&legs), vec![34_000.0, 34_000.0, 34_000.0, 34_000.0, 36_000.0, 36_000.0, 11_000.0]);
        // Said as it begins, on the leg flown from ERNAN, and nowhere else.
        assert_eq!(step_climb_at(&legs, 4, &sched), Some(("ERNAN".into(), 34_000.0, 36_000.0)));
        assert!((0..legs.len()).filter(|i| *i != 4).all(|i| step_climb_at(&legs, i, &sched).is_none()));
        // The follower's vertical: level at FL340 on the leg to ERNAN (no
        // early climb), climbing to FL360 on the leg after it; the cruise is
        // the plan's highest level, so this is a climb before the top.
        let cruise = legs.iter().filter_map(|l| l.alt1_ft).fold(f64::MIN, f64::max);
        let top = plan_top_index(&legs, cruise);
        let profile = |idx: usize| DescentProfile {
            cruise_ft: Some(cruise),
            peak_ft: 34_000.0,
            past_top: top.is_some_and(|t| idx > t),
            arrival_ceiling: false,
        };
        let v = vertical_tick(&legs[3], 34_000.0, 34_000.0, 34_000.0, Some(80.0), 460.0, None, None, 2_000.0, profile(3));
        assert_eq!(v.want_alt, 34_000.0);
        let v = vertical_tick(&legs[4], 34_000.0, 34_000.0, 34_000.0, Some(200.0), 460.0, None, None, 2_000.0, profile(4));
        assert_eq!(v.want_alt, 36_000.0);
        // A step fix the route does not carry: nothing is changed.
        let mut other = built.clone();
        let lost = CruiseSchedule {
            steps: crate::simbrief::parse_step_climbs("EIDW/0340 NOSUCH/0360"),
            ..sched.clone()
        };
        assert!(apply_step_climbs(&mut other, &lost).is_empty());
        assert_eq!(alts(&other), alts(&built));
        // A plan without stages (an old OFP) is not stepped either.
        let mut bare = built.clone();
        let nostage = CruiseSchedule { fixes: Vec::new(), ..sched };
        assert!(apply_step_climbs(&mut bare, &nostage).is_empty());
        assert_eq!(alts(&bare), alts(&built));
    }
