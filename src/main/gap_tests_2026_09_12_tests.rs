    //! The utterances that missed live on 2026-09-12 (LXGB to LEMG, 172SP),
    //! verbatim from the log, each now routed deterministically.
    use super::*;

    fn plan() -> simbrief::FlightPlan {
        let wp = |ident: &str, lat: f64, lon: f64| simbrief::Waypoint {
            ident: ident.into(),
            lat,
            lon,
            alt: 8000.0,
        };
        simbrief::FlightPlan {
            transition_altitude_ft: None,
            transition_level_ft: None,
            procedures: Default::default(),
            callsign: None,
            schedule: Default::default(),
            source: simbrief::PlanSource::Local,
            origin: "LXGB".into(),
            destination: "LEMG".into(),
            alternate: "LEZL".into(),
            aircraft: "C172".into(),
            cruise_alt_ft: 8000.0,
            distance_nm: 60.0,
            ete_min: 45.0,
            block_fuel_kg: 30.0 * 2.72,
            trip_fuel_kg: 8.0 * 2.72,
            reserve_kg: 6.0 * 2.72,
            min_takeoff_fuel_kg: 14.0 * 2.72,
            weights: None,
            route: "PIMOS DEKUS".into(),
            wind_comp_kts: 0.0,
            wind_dir: 0.0,
            wind_spd_kts: 0.0,
            tod_to_dest_nm: 15.0,
            waypoints: vec![
                wp("LXGB", 36.15, -5.35),
                wp("PIMOS", 36.30, -5.10),
                wp("DEKUS", 36.45, -4.80),
                wp("LEMG", 36.67, -4.50),
            ],
        }
    }

    #[test]
    fn a_fix_whisper_misspelled_is_matched_from_the_plan() {
        let fp = plan();
        // "How much left for pimmons?" -> the range to PIMOS.
        let q = fix_query("how much left for pimmons", &fp.waypoints).unwrap();
        assert_eq!((q.ident.as_str(), q.after, q.idx), ("PIMOS", false, 1));
        assert_eq!(q.next.as_deref(), Some("DEKUS"));
        // "And what's the waypoint to act at Pimos?" -> what follows PIMOS.
        let q = fix_query("and whats the waypoint to act at pimos", &fp.waypoints).unwrap();
        assert_eq!((q.ident.as_str(), q.after), ("PIMOS", true));
        let q = fix_query("whats after dekus", &fp.waypoints).unwrap();
        assert_eq!((q.ident.as_str(), q.after), ("DEKUS", true));
        assert!(fix_query("when do we reach dekus", &fp.waypoints).is_some());
        // Airports at either end are the destination ETA's job, not a fix.
        assert!(fix_query("how far to lemg", &fp.waypoints).is_none());
        // Commands and the plain progress question never match a fix.
        assert!(fix_query("set heading to 270", &fp.waypoints).is_none());
        assert!(fix_query("how long till next waypoint", &fp.waypoints).is_none());
        assert!(fix_query("how much left for next waypoint", &fp.waypoints).is_none());
        assert!(fix_query("turn to pilot heading", &fp.waypoints).is_none());
        // First letter must agree: "dimos" is not PIMOS.
        assert!(fix_query("how far to dimos", &fp.waypoints).is_none());
        assert_eq!(ident_distance("pimmons", "pimos"), 1);
    }

    #[test]
    fn arrival_weather_points_at_the_destination_never_the_air_we_are_in() {
        let fp = plan();
        let st = |icao: &str| RemoteWeather::Station {
            icao: icao.into(),
            label: icao.into(),
        };
        assert_eq!(
            remote_weather_target("what weather can we expect on arrival", Some(&fp)),
            Some(st("LEMG"))
        );
        assert_eq!(
            remote_weather_target("whats the wind at the destination", Some(&fp)),
            Some(st("LEMG"))
        );
        assert_eq!(
            remote_weather_target("weather at the alternate", Some(&fp)),
            Some(st("LEZL"))
        );
        assert_eq!(
            remote_weather_target("weather at lxgb", Some(&fp)),
            Some(st("LXGB"))
        );
        assert_eq!(
            remote_weather_target("weather on arrival", None),
            Some(RemoteWeather::NoPlan)
        );
        // Here and now stays with the sim's own weather.
        assert!(remote_weather_target("whats the weather", Some(&fp)).is_none());
        assert!(remote_weather_target("whats the wind here", Some(&fp)).is_none());
        assert!(weather_words("what weather can we expect on arrival"));
        assert!(!weather_words("how much fuel do we have"));
    }

    /// "Do we have enough runway for takeoff?" went unanswered on the A330
    /// at Edinburgh (2026-09-20, transcript verbatim). The answer is the
    /// takeoff's own fit, said as two numbers.
    #[test]
    fn the_runway_length_question_is_answered_from_the_takeoffs_own_fit() {
        use crate::answers::{takeoff_runway_line, takeoff_runway_query};
        use crate::autotakeoff::NoFit;
        for q in [
            "Do we have enough runway for takeoff?",
            "is the runway long enough",
            "how much runway do we need",
            "is this runway too short for us",
        ] {
            assert!(takeoff_runway_query(q), "{q}");
        }
        // Commands, advice and chatter stay where they were.
        for q in [
            "line us up runway two four",
            "which runway should we use",
            "runway two four cleared for takeoff",
            "do we have enough fuel",
        ] {
            assert!(!takeoff_runway_query(q), "{q}");
        }
        assert_eq!(
            takeoff_runway_line(Ok(("24".into(), 2338.0)), 2113.0),
            "Yes. Runway two four, 2,340 metres ahead of us, and we need 2,110 at this weight."
        );
        let short = takeoff_runway_line(
            Err(NoFit::RunwayTooShort {
                id: "24".into(),
                length_m: 2549.0,
                needed_m: 2560.0,
            }),
            2560.0,
        );
        assert!(
            short.starts_with("No. Runway two four is 2,550 metres and we need 2,560"),
            "{short}"
        );
        let far = takeoff_runway_line(
            Err(NoFit::TooLittleLeft {
                id: "24".into(),
                remaining_m: 141.0,
                needed_m: 2113.0,
            }),
            2113.0,
        );
        assert!(far.starts_with("Not from here. 140 metres left"), "{far}");
        assert!(takeoff_runway_line(
            Err(NoFit::NoneNear {
                id: String::new(),
                lateral_m: 0.0
            }),
            2113.0
        )
        .contains("not lined up"));
        // A length question with no runway near still cannot be measured, even
        // now that `NoneNear` knows which strip it looked for — that naming is
        // the REFUSAL's job (`autotakeoff::no_fit_line`), not this answer's.
        let away = takeoff_runway_line(
            Err(NoFit::NoneNear {
                id: "07".into(),
                lateral_m: 169.0,
            }),
            2113.0,
        );
        assert!(away.contains("not lined up"), "{away}");
    }

    #[test]
    fn speed_questions_get_a_verdict_against_the_phase() {
        let r = aircraft::ReferenceProfile {
            vy_kts: Some(74.0),
            approach_kts: Some(65.0),
            cruise_ias_kts: Some(110.0),
            ..Default::default()
        };
        let mut c = SpeedContext {
            on_ground: false,
            vvi_fpm: 500.0,
            agl_ft: 1100.0,
            flap_ratio: 0.0,
            hold_kts: None,
            floor_kts: Some(58.0),
        };
        // "Aren't we going a bit slow?" at 72 in the climb: Vy is 74.
        assert_eq!(
            speed_verdict(72.0, &c, Some(&r)).as_deref(),
            Some("we're climbing at Vy, 74, that's right.")
        );
        assert!(speed_verdict(62.0, &c, Some(&r))
            .unwrap()
            .contains("a touch slow"));
        // Below the floor outranks everything, including a governor target.
        c.hold_kts = Some(70.0);
        assert!(speed_verdict(55.0, &c, Some(&r))
            .unwrap()
            .contains("below the 58 knot floor"));
        assert_eq!(
            speed_verdict(71.0, &c, Some(&r)).as_deref(),
            Some("I'm holding 70, on target.")
        );
        c.hold_kts = None;
        // Cruise.
        c.vvi_fpm = 0.0;
        c.agl_ft = 7000.0;
        assert!(speed_verdict(90.0, &c, Some(&r))
            .unwrap()
            .contains("we're slow"));
        assert!(speed_verdict(108.0, &c, Some(&r))
            .unwrap()
            .contains("about right"));
        // Approach, flaps out.
        c.flap_ratio = 0.6;
        assert!(speed_verdict(59.0, &c, Some(&r))
            .unwrap()
            .contains("Add power"));
        // On the ground, and with no profile, just the number.
        c.on_ground = true;
        assert!(speed_verdict(20.0, &c, Some(&r)).is_none());
        c.on_ground = false;
        assert!(speed_verdict(90.0, &c, None).is_none());

        assert!(speed_assessment_words("is our speed okay"));
        assert!(speed_assessment_words("arent we going a bit slow"));
        assert!(speed_assessment_words("are we too fast"));
        assert!(!speed_assessment_words("whats our speed"));
        assert!(!speed_assessment_words("how fast are we going"));
    }

    #[test]
    fn negated_auxiliaries_and_should_we_are_questions() {
        for q in [
            "arent we going a bit slow",
            "aren't we going a bit slow",
            "isnt the gear down",
            "should we check in with atc",
            "shall we call the tower",
            "is our speed okay",
        ] {
            assert!(looks_like_question(q), "{q:?}");
        }
        // A polite request is still a command once the normaliser has peeled it.
        assert!(!looks_like_question(
            grammar::normalize("Can you fly the flight plan?").trim_end_matches('?')
        ));
    }
