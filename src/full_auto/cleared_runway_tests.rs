    use super::*;

    /// ETND's two strips as apt.dat has them: 08L/26R, 446 m of grass, and
    /// 08/26, 1,240 m of asphalt beside it.
    fn etnd() -> Vec<crate::nav::Runway> {
        vec![
            crate::nav::Runway {
                ident_a: "08L".into(),
                ident_b: "26R".into(),
                lat_a: 52.5862935,
                lon_a: 8.3393953,
                lat_b: 52.5866670,
                lon_b: 8.3459906,
                width_m: 50.0,
                surface: 3,
            },
            crate::nav::Runway {
                ident_a: "08".into(),
                ident_b: "26".into(),
                lat_a: 52.5850000,
                lon_a: 8.3318780,
                lat_b: 52.5859870,
                lon_b: 8.3501930,
                width_m: 45.11,
                surface: 1,
            },
        ]
    }

    /// ETND 2026-09-24, the SF50 at the weight it flew: cleared 26R, 446 m,
    /// against 780 m needed. Refused with both numbers, the paved 26 asked
    /// for in the words the tower parses; a runway that fits is taxied as
    /// today; nothing that fits is said and asks for nothing.
    #[test]
    fn a_short_cleared_runway_is_refused_and_the_longer_one_requested() {
        let rw = etnd();
        let fit = cleared_runway_fit(&rw, "26R", 780.0);
        let ClearedFit::Short {
            cleared,
            length_m,
            needed_m,
            ask,
        } = fit.clone()
        else {
            panic!("{fit:?}");
        };
        assert_eq!(cleared, "26R");
        assert!((length_m - 446.0).abs() < 5.0, "{length_m}");
        assert_eq!(needed_m, 780.0);
        assert_eq!(ask.as_deref(), Some("26"), "the long runway, flown the cleared way");
        let line = short_runway_line(length_m, needed_m, &cleared, ask.as_deref());
        assert!(
            line.starts_with("Runway two six right is 44") && line.contains("we need 780."),
            "{line}"
        );
        assert!(line.ends_with("Asking for two six."), "{line}");
        let call = runway_request_call(&cleared, "26");
        assert_eq!(call, "unable runway two six right, request runway two six for length");
        assert_eq!(
            crate::tower::parse_intent(&call),
            crate::tower::Intent::RequestRunway
        );
        assert_eq!(crate::tower::requested_runway(&call).as_deref(), Some("26"));

        // The 1,240 m runway fits the same aircraft: taxied as today.
        assert_eq!(cleared_runway_fit(&rw, "26", 780.0), ClearedFit::Fits);
        assert_eq!(cleared_runway_fit(&rw, "08", 780.0), ClearedFit::Fits);
        // A runway the data lacks is the line-up's to judge.
        assert_eq!(cleared_runway_fit(&rw, "33", 780.0), ClearedFit::Unknown);
        // Nothing long enough: refused, nothing asked for.
        match cleared_runway_fit(&rw, "26", 2_000.0) {
            ClearedFit::Short { ask: None, .. } => {}
            other => panic!("{other:?}"),
        }
        assert!(short_runway_line(1240.0, 2000.0, "26", None).ends_with("Nothing here is long enough."));
    }

    /// #24's short-runway branch on the real apt.dat and the real profiles:
    /// the ONE rule's need at maximum weight (`profile_takeoff_roll`,
    /// `takeoff_needed_m`, the 500 m floor `judge_cleared_runway` uses)
    /// against real runways. The 737 cleared to LFPB 27 (09/27, shorter than
    /// the 2,347 m it needs) refuses it with both numbers and asks for 25 in
    /// the words the station's test grants
    /// (`real_install_short_cleared_runway_is_granted_the_longer_one`); the
    /// SF50 cleared to ETND's grass 26R asks for the paved 26; the A330 at
    /// EGKK and the 737 on LFPB 25 fit.
    /// `cargo test --release -- --ignored real_install_cleared_runway_fit --nocapture`
    #[test]
    #[ignore] // needs the real X-Plane install; run with --ignored
    fn real_install_cleared_runway_fit() {
        let apt = crate::config::Config::from_env().expect("config").apt_data_path;
        if !std::path::Path::new(&apt).exists() {
            return;
        }
        let need = |id: &str| {
            let p = crate::aircraft::load(id).expect(id);
            let (roll, mtow) = crate::aircraft::profile_takeoff_roll(&p);
            crate::aircraft::takeoff_needed_m(roll, None, mtow, 500.0)
        };
        let field = |icao: &str| crate::nav::find_runways(&apt, icao).expect(icao);
        let (b738, sf50, a330) = (need("boeing_737_800"), need("cirrus_vision_sf50"), need("airbus_a330_300"));
        let (lfpb, etnd, egkk) = (field("LFPB"), field("ETND"), field("EGKK"));
        for (icao, rw) in [("LFPB", &lfpb), ("ETND", &etnd), ("EGKK", &egkk)] {
            let lens: Vec<String> = rw
                .iter()
                .map(|r| format!("{}/{} {:.0} m", r.ident_a, r.ident_b, r.length_m()))
                .collect();
            println!("{icao}: {}", lens.join(", "));
        }
        println!("need at MTOW: 737 {b738:.0} m, SF50 {sf50:.0} m, A330 {a330:.0} m");

        let fit = cleared_runway_fit(&lfpb, "27", b738);
        println!("737 cleared LFPB 27: {fit:?}");
        let ClearedFit::Short { cleared, length_m, needed_m, ask } = fit else {
            panic!("27 is short for the 737");
        };
        assert_eq!((cleared.as_str(), ask.as_deref()), ("27", Some("25")));
        let line = short_runway_line(length_m, needed_m, &cleared, ask.as_deref());
        println!("  says: {line}");
        assert!(line.ends_with("Asking for two five."), "{line}");
        assert_eq!(
            runway_request_call(&cleared, "25"),
            "unable runway two seven, request runway two five for length"
        );
        assert_eq!(cleared_runway_fit(&lfpb, "25", b738), ClearedFit::Fits);

        let fit = cleared_runway_fit(&etnd, "26R", sf50);
        println!("SF50 cleared ETND 26R: {fit:?}");
        assert!(
            matches!(&fit, ClearedFit::Short { ask: Some(a), .. } if a == "26"),
            "{fit:?}"
        );
        for end in ["26L", "08R", "26R", "08L"] {
            let fit = cleared_runway_fit(&egkk, end, a330);
            println!("A330 cleared EGKK {end}: {fit:?}");
        }
        assert_eq!(cleared_runway_fit(&egkk, "26L", a330), ClearedFit::Fits);
    }
