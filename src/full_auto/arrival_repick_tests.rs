    use super::{repick_line, wind_runway, RepickWatch, MAX_ARRIVAL_REPICKS};

    /// #79: the tower's end is taken at once; the wind's only on the second
    /// reading in a row; nothing moves a runway the mission does not hold, a
    /// wind pick with the tower naming our end, or past the cap.
    #[test]
    fn the_arrival_is_repicked_by_the_tower_at_once_and_by_the_wind_when_it_stays() {
        let mut w = RepickWatch::default();
        assert_eq!(w.decide(None, Some("12"), None), None, "no mission runway: nothing to move");
        assert_eq!(w.decide(Some("30"), None, Some("12")), None, "one METAR is not a trend");
        assert_eq!(w.decide(Some("30"), None, None), None, "calm: the plan's end stands");
        assert_eq!(w.decide(Some("30"), None, Some("12")), None, "the calm reset it");
        assert_eq!(w.decide(Some("30"), None, Some("12")), Some(("12".into(), "wind")));
        // The tower's end, taken at once; its own end silences the wind.
        let mut w = RepickWatch::default();
        assert_eq!(w.decide(Some("12"), Some("12"), Some("30")), None);
        assert_eq!(w.decide(Some("12"), Some("12"), Some("30")), None);
        assert_eq!(w.decide(Some("12"), Some("30"), None), Some(("30".into(), "tower")));
        // A fence-sitting wind cannot swap the arrival for ever.
        let mut w = RepickWatch::default();
        let mut rwy = "30".to_string();
        let mut moves = 0;
        for _ in 0..20 {
            let other = if rwy == "30" { "12" } else { "30" };
            if let Some((r, _)) = w.decide(Some(&rwy), None, Some(other)) {
                rwy = r;
                moves += 1;
            }
        }
        assert_eq!(moves, MAX_ARRIVAL_REPICKS);
    }

    #[test]
    fn a_repick_is_said_with_its_reason_and_what_is_flown() {
        assert_eq!(
            repick_line("LIPE", "12", "wind", Some(("BAME5M", "BAMEK"))),
            "The wind at LIPE now favours runway one two, so the BAME5M arrival from BAMEK."
        );
        assert!(repick_line("LIPE", "12", "tower", None).starts_with("Tower wants runway one two, and no arrival"));
        assert_eq!(wind_runway(&[], Some((120.0, 15.0))), None);
    }

    /// LIPE in flight: armed on BAME2S for 30, the METAR swings to 120/15
    /// while the follower flies the route. The re-pick splices BAME5M for 12
    /// from BAMEK into the legs ahead, the legs flown stay, BAME5M's windows
    /// come with it, and the follower takes it while BAMEK is still ahead.
    /// Needs the X-Plane install.
    /// `cargo test -- --ignored real_install_lipe_star_repicked_in_flight --nocapture`
    #[test]
    #[ignore]
    fn real_install_lipe_star_repicked_in_flight() {
        use crate::nav::ProcKind;
        let config = crate::config::Config::from_env().expect("config");
        let apt = crate::nav::find_airport(&config.apt_data_path, "LIPE").unwrap().unwrap();
        let (alat, alon) = apt.position().unwrap();
        let mut star =
            crate::nav::load_proc_legs(&config.cifp_dir, "LIPE", ProcKind::Star, "BAME2S", Some("RW30"));
        crate::nav::resolve_proc_fixes(&config.fix_data_path, &config.nav_data_path, &apt, "LIPE", &mut star, alat, alon);
        let waypoints: Vec<crate::simbrief::Waypoint> = std::iter::once(crate::simbrief::Waypoint {
            ident: "ENRT1".into(),
            lat: 45.3,
            lon: 10.4,
            alt: 12000.0,
        })
        .chain(
            star.iter()
                .filter_map(|l| l.fix.as_ref().map(|f| (l.fix_ident.clone(), f.lat, f.lon)))
                .enumerate()
                .map(|(i, (ident, lat, lon))| crate::simbrief::Waypoint {
                    ident,
                    lat,
                    lon,
                    alt: 12000.0 - 1500.0 * i as f64,
                }),
        )
        .collect();
        let legs = crate::procedure::enroute_legs(&waypoints, "LIME", "LIPE", 12000.0);
        let (splice, ident, full) = super::arrival_splice(
            &config.cifp_dir,
            &config.fix_data_path,
            &config.nav_data_path,
            &apt,
            "LIPE",
            "12",
            Some("BAME2S"),
            &legs,
            1,
            "wind",
        )
        .expect("a STAR from the route serves 12");
        println!(
            "{ident} from {} at {}: {:?}",
            splice.entry,
            splice.from,
            full.iter().map(|l| (l.fix_ident.clone(), l.alt1_ft.map(f64::round))).collect::<Vec<_>>()
        );
        assert_eq!(ident, "BAME5M");
        assert_eq!(splice.entry, "BAMEK");
        assert!(splice.from >= 1);
        assert_eq!(
            full[..splice.from].iter().map(|l| &l.fix_ident).collect::<Vec<_>>(),
            legs[..splice.from].iter().map(|l| &l.fix_ident).collect::<Vec<_>>(),
            "the legs flown stay"
        );
        assert_eq!(full.last().map(|l| l.fix_ident.as_str()), Some("EMDUD"));
        assert!(full.iter().all(|l| l.fix.is_some() && l.alt1_ft.is_some()));
        assert!(!splice.constraints.is_empty(), "BAME5M's windows for 12 apply");
        // The follower takes it while the entry is ahead, never after.
        let applied = crate::procedure::apply_splice(&legs, splice.from, &splice).unwrap();
        assert_eq!(applied.len(), full.len());
        assert_eq!(
            crate::procedure::apply_splice(&legs, splice.from + 1, &splice).unwrap_err(),
            "past_entry"
        );
    }
