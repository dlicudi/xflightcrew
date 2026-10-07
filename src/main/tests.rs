    use super::{
        classify_confidence, is_affirmative, is_undo, join_and, plan_change_line, plan_fetch_for,
        progress_line, radio_panel_line, undo_targets, wrong_plan_complaint, ConfidenceAction,
        PlanFetch, Ptt,
    };

    /// #135: a line the loop has taken off the channel and not yet routed is
    /// owed the loop as much as one still waiting on it. The crew-radio drain
    /// saw only the channel, and "my controls" (taken, so the channel was
    /// empty) waited 26.1 s behind the tower's taxi clearance at LEMG.
    #[test]
    fn a_line_in_hand_is_owed_the_loop_like_a_waiting_one() {
        assert!(super::pilot_line_pending(true));
    }

    /// #167: the records of sortie 1790611629 (LEMG, 2026-09-28). The loop
    /// spoke the setup's readback at +0.5 s, the tower stepped on it at
    /// +7.8 s, the swap was armed at +8.4 s and installed at +30.2 s, when the
    /// stale remainder was dropped. No pilot line was waiting, so only the
    /// armed swap can release the wait.
    #[test]
    fn an_armed_station_swap_releases_the_loops_speech_wait() {
        use std::sync::atomic::{AtomicBool, Ordering::SeqCst};
        // Before the swap: the readback is waited out.
        assert!(!super::loop_owed_by(false, false));
        // Armed: the loop goes to install it.
        assert!(super::loop_owed_by(false, true));
        assert!(super::loop_owed_by(true, false));
        // Through the real wait: a readback never spoken (stepped on, its
        // remainder queued behind the radio) gives way the moment the swap
        // is armed, not when the line is dropped stale 20 s later.
        let armed = std::sync::Arc::new(AtomicBool::new(false));
        let a = armed.clone();
        let t = std::thread::spawn(move || {
            std::thread::sleep(std::time::Duration::from_millis(100));
            a.store(true, SeqCst);
        });
        let t0 = std::time::Instant::now();
        let spoken =
            crate::voice::wait_unspoken_until(|| super::loop_owed_by(false, armed.load(SeqCst)));
        t.join().unwrap();
        assert!(!spoken);
        assert!(t0.elapsed() < std::time::Duration::from_secs(2), "{:?}", t0.elapsed());
    }

    fn plan(origin: &str, dest: &str, route: &str, n: usize) -> crate::simbrief::FlightPlan {
        crate::simbrief::FlightPlan {
            transition_altitude_ft: None,
            transition_level_ft: None,
            procedures: Default::default(),
            callsign: None,
            schedule: Default::default(),
            source: crate::simbrief::PlanSource::SimBrief,
            origin: origin.into(),
            destination: dest.into(),
            alternate: String::new(),
            aircraft: "C172".into(),
            cruise_alt_ft: 5500.0,
            distance_nm: 60.0,
            ete_min: 40.0,
            block_fuel_kg: 30.0 * 2.72,
            trip_fuel_kg: 10.0 * 2.72,
            reserve_kg: 8.0 * 2.72,
            min_takeoff_fuel_kg: 18.0 * 2.72,
            weights: None,
            route: route.into(),
            wind_comp_kts: 0.0,
            wind_dir: 0.0,
            wind_spd_kts: 0.0,
            tod_to_dest_nm: 0.0,
            waypoints: (0..n)
                .map(|i| crate::simbrief::Waypoint {
                    ident: format!("W{i}"),
                    lat: 36.0,
                    lon: -4.0,
                    alt: 5500.0,
                })
                .collect(),
        }
    }

    /// Live 2026-09-14: "load the flight plan" four times in two minutes
    /// after regenerating on SimBrief, each one re-sent LEMG-LEJR from the
    /// shelf. A spoken ask re-fetches; only a refresh word forces it over a
    /// local plan.
    #[test]
    fn spoken_plan_asks_refetch_simbrief() {
        for t in [
            "load the flight plan.",
            "let's load the flight plan.",
            "load the flight plane.",
            "load the flight lamp.",
            "what's our flight plan",
        ] {
            assert_eq!(plan_fetch_for(t), PlanFetch::Asked, "{t}");
        }
        for t in [
            "refresh the flight plan",
            "reload the plan",
            "update the flight plan",
        ] {
            assert_eq!(plan_fetch_for(t), PlanFetch::Forced, "{t}");
        }
    }

    #[test]
    fn wrong_plan_complaint_is_closed_shape() {
        // Live whisper output.
        assert!(wrong_plan_complaint("that's the wrong flag plan."));
        assert!(wrong_plan_complaint("that's the wrong flight plan"));
        assert!(wrong_plan_complaint("not that plan"));
        assert!(wrong_plan_complaint("wrong route"));
        // A wrongness word without a plan word is not a plan complaint.
        assert!(!wrong_plan_complaint("wrong, flaps ten"));
        assert!(!wrong_plan_complaint("that's the wrong runway"));
        assert!(!wrong_plan_complaint("load the flight plan"));
    }

    #[test]
    fn plan_change_line_says_whether_simbrief_moved() {
        let old = plan("LEMG", "LEJR", "MGA DCT", 9);
        let same = plan("LEMG", "LEJR", "MGA DCT", 9);
        let new = plan("LEJR", "LEMG", "JRZ DCT", 6);
        assert_eq!(
            plan_change_line(Some(&old), Some(&same), true),
            "SimBrief still has LEMG to LEJR. Generate the new plan there, then say load the \
             flight plan."
        );
        assert_eq!(
            plan_change_line(Some(&old), Some(&new), true),
            "Got the new plan, LEJR to LEMG, 6 waypoints. Say load it to send it to the GPS."
        );
        // Nothing on the shelf before: whatever came is new.
        assert!(plan_change_line(None, Some(&new), true).starts_with("Got the new plan"));
        assert_eq!(
            plan_change_line(Some(&old), Some(&old), false),
            "Couldn't reach SimBrief. I still have LEMG to LEJR."
        );
        assert!(plan_change_line(None, None, false).starts_with("No plan on the shelf"));
    }

    #[test]
    fn progress_line_names_fix_distance_and_time() {
        assert_eq!(
            progress_line("MG430", Some(2.39), Some(1.43)),
            "Next waypoint MG430, 2 miles, 1 minutes."
        );
        assert_eq!(
            progress_line("BUKOX", Some(33.4), None),
            "Next waypoint BUKOX, 33 miles."
        );
        assert_eq!(
            progress_line("LEMG", Some(130.0), Some(95.0)),
            "Next waypoint LEMG, 130 miles, 1 hours 35 minutes."
        );
    }

    #[test]
    fn confidence_bands_act_confirm_sayagain_ignore() {
        use ConfidenceAction::*;
        let (min, floor) = (0.35, 0.22);
        // Above the act threshold: always act, grammar/deliberate irrelevant.
        assert_eq!(classify_confidence(0.40, min, floor, true, true), Act);
        assert_eq!(classify_confidence(0.40, min, floor, false, false), Act);
        // Marginal band + deliberate + grammar recognises it → confirm.
        assert_eq!(classify_confidence(0.30, min, floor, true, true), Confirm);
        // Marginal but only the LLM could resolve it → don't guess, say again.
        assert_eq!(classify_confidence(0.30, min, floor, true, false), SayAgain);
        // Below the floor, even a recognised command is too garbled → say again.
        assert_eq!(classify_confidence(0.15, min, floor, true, true), SayAgain);
        // Not deliberate (continuous ambient / crew off): ignore, never speak.
        assert_eq!(classify_confidence(0.30, min, floor, false, true), Ignore);
        assert_eq!(classify_confidence(0.15, min, floor, false, false), Ignore);
        // Floor is inclusive; just under it is not a confirm.
        assert_eq!(classify_confidence(floor, min, floor, true, true), Confirm);
        assert_eq!(
            classify_confidence(floor - 0.01, min, floor, true, true),
            SayAgain
        );
    }

    #[test]
    fn ptt_routes_intercom_and_radio_to_separate_channels() {
        use std::sync::Arc;
        let intercom_dref = "xcopilot/ptt_active";
        let radio_dref = "xcopilot/radio_ptt_active";
        let mk = |icom: f64, radio: f64| Ptt::Dataref {
            live: Arc::new(crate::livestate::LiveState::from_pairs(&[
                (intercom_dref, icom),
                (radio_dref, radio),
            ])),
            name: intercom_dref.to_string(),
            radio: radio_dref.to_string(),
        };
        // Intercom key alone: talking to the copilot, NOT the radio.
        let p = mk(1.0, 0.0);
        assert!(p.intercom_pressed() && !p.radio_pressed());
        // Radio key alone: transmitting, NOT the intercom.
        let p = mk(0.0, 1.0);
        assert!(p.radio_pressed() && !p.intercom_pressed());
        // Neither.
        let p = mk(0.0, 0.0);
        assert!(!p.intercom_pressed() && !p.radio_pressed() && !p.is_pressed());
        // Either one starts a capture (is_pressed for the mic/routing).
        assert!(mk(1.0, 0.0).is_pressed() && mk(0.0, 1.0).is_pressed());
        // No radio dataref bound → radio is never pressed; intercom still works.
        let p = Ptt::Dataref {
            live: Arc::new(crate::livestate::LiveState::from_pairs(&[(
                intercom_dref,
                1.0,
            )])),
            name: intercom_dref.to_string(),
            radio: String::new(),
        };
        assert!(p.intercom_pressed() && !p.radio_pressed());
        // Channel completeness: with a radio key BOUND (regardless of pressed
        // state), physical routing is complete and intercom speech must never
        // content-route to ATC; without one, content routing is the only path
        // to the tower and stays. A keyboard PTT is never a radio.
        assert!(
            mk(0.0, 0.0).has_radio_channel(),
            "bound radio key, even unpressed"
        );
        assert!(!p.has_radio_channel(), "no radio dataref bound");
        assert!(!Ptt::Key(device_query::Keycode::F13).has_radio_channel());
    }

    #[test]
    fn radio_row_shows_the_station_and_the_tuned_service_only() {
        let services = vec![
            ("tower".to_string(), 131200_i64),
            ("approach".to_string(), 122800),
            ("departure".to_string(), 130400),
        ];
        // Tuned to the tower (8.33 name can sit 5 kHz off the carrier):
        // station + that ONE chip — the full plan is the ATC tab's job.
        let line = radio_panel_line("Gibraltar", &services, Some(131205));
        assert_eq!(line, "GIBRALTAR | >TWR 131.200");
        // Off every service: the raw COM1 value as the second chip.
        let line = radio_panel_line("Gibraltar", &services, Some(124730));
        assert_eq!(line, "GIBRALTAR | >COM1 124.730");
        // COM1 unreadable: just the station.
        assert_eq!(radio_panel_line("Gibraltar", &services, None), "GIBRALTAR");
        // Active station out of VHF range (empty airport): only live COM1.
        let line = radio_panel_line("", &[], Some(123500));
        assert_eq!(line, ">COM1 123.500");
        assert_eq!(radio_panel_line("", &[], None), "");
    }

    #[test]
    fn atc_payload_marks_active_and_fits_the_plugin_buffer() {
        let f = |name: &str, dist: f64, active: bool| crate::party_line::RangedField {
            name: name.to_string(),
            dist_nm: dist,
            active,
            services: vec![
                ("tower".to_string(), 118325_i64),
                ("atis".to_string(), 121800),
            ],
        };
        let payload =
            super::atc_panel_payload(&[f("Badajoz", 21.7, false), f("Gibraltar", 2.1, true)]);
        assert_eq!(
            payload,
            "BADAJOZ,22,TWR 118325,ATIS 121800|*GIBRALTAR,2,TWR 118325,ATIS 121800"
        );
        // Separators are structural — a name containing them can't corrupt
        // neighbouring fields.
        let payload = super::atc_panel_payload(&[f("El Molinillo, Almendralejo|x", 3.0, false)]);
        assert!(
            payload.starts_with("EL MOLINILLO  ALMENDRALEJO X,3,"),
            "{payload}"
        );
        // Too many fields: trailing ones drop instead of overflowing the
        // plugin's 500-byte text buffer.
        let many: Vec<_> = (0..40)
            .map(|i| f(&format!("Field Number {i:02}"), i as f64, false))
            .collect();
        let payload = super::atc_panel_payload(&many);
        assert!(payload.len() <= crate::plugin_status::TEXT_MAX);
        assert!(payload.contains("FIELD NUMBER 00"));
    }

    #[test]
    fn undo_recognizes_emergency_phrasings() {
        for w in [
            "belay that",
            "undo",
            "undo that",
            "abort",
            "scratch that",
            "as you were",
            "disregard",
            "revert",
            "reverse that",
            "take it back",
            "cancel that",
            "abort. didn't catch a command there. abort.",
            "a boat",
        ] {
            assert!(is_undo(w), "{w:?} should trigger undo");
        }
        // Not an undo (would clobber normal speech / the reminder cancel).
        for w in ["set heading 270", "cancel the reminders", "flaps up"] {
            assert!(!is_undo(w), "{w:?} should not trigger undo");
        }
    }

    #[test]
    fn value_tools_are_undoable_command_tools_are_not() {
        assert!(!undo_targets("set_heading").is_empty());
        assert!(!undo_targets("set_throttle").is_empty());
        assert!(undo_targets("start_engine").is_empty());
        assert!(undo_targets("pushback").is_empty());
    }

    const CITATION_LEVER: &str = "sim/cockpit2/engine/actuators/throttle_jet_rev_ratio_all";

    /// The Citation X obeys `throttle_jet_rev_ratio_all` (its Lua re-drives
    /// the levers from it): "belay the throttle" snapshots and restores the
    /// lever `set_throttle` wrote, never the stock one it ignores.
    #[test]
    fn throttle_undo_restores_the_airframes_lever() {
        crate::aircraft::publish_throttle_dataref(Some(CITATION_LEVER.into()));
        let t = undo_targets("set_throttle");
        crate::aircraft::publish_throttle_dataref(None);
        assert_eq!(t, vec![(CITATION_LEVER.to_string(), "throttle", false)]);
        assert_eq!(
            undo_targets("set_throttle")[0].0,
            crate::aircraft::STOCK_THROTTLE
        );
    }

    /// "What's the throttle set to?" reads the same lever.
    #[tokio::test]
    async fn the_throttle_answer_reads_the_airframes_lever() {
        crate::aircraft::publish_throttle_dataref(Some(CITATION_LEVER.into()));
        let live = crate::livestate::LiveState::from_pairs(&[
            (CITATION_LEVER, 0.62),
            (crate::aircraft::STOCK_THROTTLE, 0.0),
        ]);
        let xp = crate::xplane::XPlaneClient::new("http://127.0.0.1:9".into());
        let p = crate::aircraft::load("citation_x").expect("profile");
        let q = "what's the throttle set to";
        let a = crate::answers::answer_state_query(q, &xp, Some(&live), &p).await;
        crate::aircraft::publish_throttle_dataref(None);
        let a = a.expect("an answer");
        assert!(a.contains("throttle 62 percent"), "{a}");
    }

    #[test]
    fn reference_limits_come_from_the_profile() {
        let p = super::aircraft::load("cessna_172sp").expect("profile");
        assert_eq!(
            super::reference_limits(&p, "what's best glide").as_deref(),
            Some("best glide 68 knots.")
        );
        assert_eq!(
            super::reference_limits(&p, "rotation speed").as_deref(),
            Some("rotate at 55 knots.")
        );
        assert!(super::reference_limits(&p, "what's vy")
            .unwrap()
            .contains("74 knots"));
        // Vfe is derived from the flap detents, grouped by speed.
        assert_eq!(
            super::reference_limits(&p, "flap speed").as_deref(),
            Some("Vfe 110 at ten degrees, 85 at twenty degrees and full.")
        );
        // A non-limits utterance returns nothing.
        assert!(super::reference_limits(&p, "set flaps ten").is_none());
        // A COMMAND naming the approach is not a speed question: "fly the
        // approach" was swallowed here and answered "approach around 100
        // knots" instead of arming the Malaga arrival (2026-08-02). The
        // question keeps working; it always carries a speed word.
        assert!(super::reference_limits(&p, "fly the approach").is_none());
        assert!(super::reference_limits(&p, "shoot the approach").is_none());
        assert!(super::reference_limits(&p, "what's our approach speed")
            .unwrap()
            .contains("approach around"));

        // The SR22 carries its own card — different numbers, same machinery.
        let sr = super::aircraft::load("cirrus_sr22").expect("sr22 profile");
        assert_eq!(
            super::reference_limits(&sr, "best glide").as_deref(),
            Some("best glide 88 knots.")
        );
        assert!(super::reference_limits(&sr, "never exceed")
            .unwrap()
            .contains("204"));
        assert!(super::reference_limits(&sr, "max crosswind")
            .unwrap()
            .contains("21"));
    }

    #[test]
    fn conversation_sortie_uses_real_profile_answers_without_a_model() {
        use super::conversation::{Conversation, Response};
        let now = std::time::Instant::now();
        let profile = super::aircraft::load("cessna_172sp").unwrap();
        let mut crew = Conversation::default();
        let line = super::reference_limits(&profile, "what's best glide").unwrap();
        crew.reference(&profile.display_name, &line, now);
        let Some(Response::Say(why)) = crew.respond("why that speed?", now) else {
            panic!("must explain the reference");
        };
        assert!(why.contains("68 knots"));
        assert!(why.contains("aircraft profile"));
        let Some(Response::Reference(question)) = crew.respond("and blue line?", now) else {
            panic!("must request reference, never a command");
        };
        assert!(
            super::reference_limits(&profile, question).is_none(),
            "no twin speed invented on a single"
        );

        crew.observe("what is our altitude", "3500 feet", now);
        for words in ["and heading?", "and flaps?", "and gear?", "and fuel?"] {
            let Some(Response::Read(question)) = crew.respond(words, now) else {
                panic!("{words} must be a reading");
            };
            assert!(super::looks_like_question(question));
        }
        let registry = super::Registry::for_aircraft(&profile);
        assert!(crew.respond("flaps ten", now).is_none());
        let calls = super::grammar::try_parse("flaps ten", &registry, &profile).unwrap();
        assert_eq!(calls[0].name, "set_flaps");
        crew.report("flaps ten", "Negative, too fast for flaps.", false, now);
        let Some(Response::Say(why)) = crew.respond("why?", now) else {
            panic!("missing refusal")
        };
        assert!(why.contains("Negative, too fast"));
        assert!(
            matches!(
                crew.respond("check that again", now),
                Some(Response::Say(_))
            ),
            "cannot replay the flap command"
        );
    }

    #[test]
    fn checklist_preview_uses_loaded_items_and_clarifies_ambiguous_titles() {
        use super::conversation::{Conversation, Response};
        let now = std::time::Instant::now();
        let profile = super::aircraft::load("cessna_172sp").unwrap();
        let lists = super::checklist::parse(
            "sw_checklist:Before Takeoff\nsw_item:Brakes|SET\nsw_item:Flaps|TEN\n\
             sw_checklist:Before Landing\nsw_item:Mixture|RICH\n",
            &profile,
        );
        let mut crew = Conversation::default();
        let line = super::preview_checklist("before", &lists, &mut crew, now);
        assert!(line.contains("Which checklist"), "{line}");
        let Some(Response::Checklist(title)) = crew.respond("before takeoff", now) else {
            panic!("missing preview choice")
        };
        let line = super::preview_checklist(&title, &lists, &mut crew, now);
        assert!(line.contains("item 1 of 2"), "{line}");
        assert!(line.contains("Brakes"), "{line}");
        let Some(Response::Say(line)) = crew.respond("next item", now) else {
            panic!("missing next item")
        };
        assert!(line.contains("Flaps"));
        let line =
            super::preview_checklist("before takeoff and start engines", &lists, &mut crew, now);
        assert!(
            line.contains("Which checklist"),
            "must not silently select a partial title"
        );
        assert!(super::preview_checklist("", &[], &mut crew, now).contains("No checklists"));
    }

    #[test]
    fn conversational_fuel_questions_select_the_dedicated_reporter() {
        let mut crew = super::conversation::Conversation::default();
        for text in ["could you tell me our fuel", "tell me about endurance"] {
            let Some(super::conversation::Response::Read(query)) =
                crew.respond(text, std::time::Instant::now())
            else {
                panic!("missing read")
            };
            assert!(
                super::is_fuel_query(query),
                "{query} must use fuel_report, not the instrument-only reader"
            );
        }
    }

    /// The Baron answers the two questions only a twin gets asked, and the
    /// mapping that gets you the profile in the first place.
    #[test]
    fn the_baron_recites_the_twin_speeds() {
        let baron = "Aircraft/Laminar Research/Beechcraft Baron 58/Baron_58.acf";
        assert_eq!(
            super::detect::resolved_id("BE58", baron).as_deref(),
            Some("beechcraft_baron_58")
        );
        assert_eq!(
            super::detect::resolved_id("be58", baron).as_deref(),
            Some("beechcraft_baron_58")
        );
        let p = super::aircraft::load("beechcraft_baron_58").expect("baron profile");
        assert!(super::reference_limits(&p, "what's blue line")
            .unwrap()
            .contains("100 knots"));
        assert!(super::reference_limits(&p, "what's vmc")
            .unwrap()
            .contains("84 knots"));
        // Per-notch Vfe, spoken as two limits rather than one.
        let vfe = super::reference_limits(&p, "flap speed").unwrap();
        assert!(vfe.contains("152") && vfe.contains("122"), "{vfe}");
        // A single has neither twin speed, so it says nothing rather than
        // inventing one.
        let c172 = super::aircraft::load("cessna_172sp").expect("172 profile");
        assert!(super::reference_limits(&c172, "what's blue line").is_none());

        // The profile shapes the tool surface: retractable gear earns a gear
        // tool the 172 never gets, and a fuel-injected engine earns no carb
        // heat. This is the whole point of per-airframe profiles.
        let reg = super::tools::Registry::for_aircraft(&p);
        assert!(reg.get("gear").is_some(), "the Baron's gear retracts");
        assert!(reg.get("set_magnetos").is_some());
        assert!(reg.get("start_engine").is_some());
        assert!(
            reg.get("set_carb_heat").is_none(),
            "IO-550s are fuel injected"
        );
        assert!(super::tools::Registry::for_aircraft(&c172)
            .get("gear")
            .is_none());
    }

    // The RV-10 flew on the 172's numbers for lack of a profile (2026-09-11).
    #[test]
    fn the_rv10_has_its_own_profile_and_unknown_types_are_not_the_172() {
        assert_eq!(
            super::detect::resolved_id(
                "RV10",
                "Aircraft/Laminar Research/Van's Aircraft RV-10/RV-10.acf"
            )
            .as_deref(),
            Some("vans_rv10")
        );
        let who = super::detect::AcfIdentity {
            icao: "ZZZZ".into(),
            relative_path: "Aircraft/Somebody/Zed/Zed Plane.acf".into(),
            ..Default::default()
        };
        assert_eq!(
            super::detect::resolve(&who, &super::aircraft::profile_matchers()),
            super::detect::Resolution::UnknownType,
            "unknown is unknown, not a Cessna"
        );
        assert_eq!(
            super::detect::generic_profile_id(&who),
            "generic_zzzz_zed_plane"
        );
        let p = super::aircraft::load("vans_rv10").expect("RV-10 profile loads");
        let r = p.reference.as_ref().expect("reference card");
        assert_eq!(
            r.approach_kts,
            Some(80.0),
            "not the 172's 65, which is the RV-10's clean stall"
        );
        assert_eq!(p.vs_clean_kts, Some(65.0));
        assert_eq!(p.bus_volts_nominal, Some(12.0));
        assert!(!p.retractable_gear && !p.carb_heat && p.magnetos);
        let flaps = p.flaps.as_ref().expect("flaps");
        let vfe: Vec<Option<f64>> = flaps.detents.iter().map(|d| d.vfe_kts).collect();
        assert_eq!(vfe, vec![None, Some(96.0), Some(87.0)]);
        let sel = p.fuel_selector.as_ref().expect("xlua fuel selector");
        assert!(
            sel.readback
                .as_deref()
                .is_some_and(|r| r.contains("laminar/rv10")),
            "the stock selector is not the RV-10's"
        );
    }

    #[test]
    fn citation_x_loads_with_the_poms_own_figures() {
        let p = super::aircraft::load("citation_x").expect("citation_x profile loads");
        assert_eq!(
            super::detect::resolved_id(
                "C750",
                "Aircraft/Laminar Research/Cessna Citation X/Cessna_CitationX.acf"
            )
            .as_deref(),
            Some("citation_x")
        );
        assert!(p.autopilot && p.throttle && p.retractable_gear);
        assert!(!p.engine, "a jet declares no piston engine group");
        // POM p.80: Vr 145, Vref 132, Vs1 clean 136, Vle 210. Vmo is altitude
        // dependent (270 low / 350 high), the low figure its `vne_kts`.
        let r = p.reference.as_ref().expect("reference figures");
        assert_eq!(r.vr_kts, Some(145.0));
        assert_eq!(r.approach_kts, Some(132.0));
        assert_eq!(p.vs_clean_kts, Some(136.0));
        assert_eq!(p.vlo_kts, Some(210.0));
        assert_eq!(p.vne_kts, Some(270.0));
        // The POM's Vy of 300 KIAS is a cruise climb, and a 300 kt circuit is
        // not a circuit. The profile carries a departure speed, and the
        // pattern is capped against the approach speed regardless.
        let vy = r.vy_kts.expect("vy");
        assert!(vy <= 200.0, "vy is a climb-out speed, got {vy}");
        let pat = super::aircraft::pattern_speed_kts(vy, r.approach_kts.unwrap(), p.vs_clean_kts);
        assert!(pat <= 198.0, "pattern capped at 1.5x approach, got {pat}");
        // The runway it demands must fit the runways it flies from.
        let roll_m = r.takeoff_roll_ft.expect("ground roll") * 0.3048;
        assert!(
            roll_m * 1.4 < 2162.0,
            "must fit Luton: {:.0} m",
            roll_m * 1.4
        );
    }

    /// The SF50 is a jet whose xlua is cosmetic, so the whole profile stands on
    /// STOCK datarefs — the one custom thing is the start, which is two button
    /// presses in a fixed order rather than a crank.
    #[test]
    fn sf50_is_a_stock_systems_jet_with_a_two_press_start() {
        let p = super::aircraft::load("cirrus_vision_sf50").expect("sf50 profile loads");
        assert_eq!(
            super::detect::resolved_id(
                "SF50",
                "Aircraft/Laminar Research/Cirrus Vision SF50/CirrusSF50.acf"
            )
            .as_deref(),
            Some("cirrus_vision_sf50")
        );
        assert!(p.autopilot && p.throttle && p.retractable_gear);
        assert!(
            !p.engine && !p.magnetos && !p.carb_heat,
            "a jet declares no piston systems"
        );
        // No autothrottle on this model, so speed must be the throttle
        // governor's — declaring one withholds `hold_speed` entirely, which is
        // how "reduce speed" became a no-op on the Citation X (2026-07-28).
        let reg = super::tools::Registry::for_aircraft(&p);
        assert!(reg.get("hold_speed").is_some(), "speed is the governor's");
        assert!(reg.get("set_speed").is_none());
        assert!(reg.get("autothrottle").is_none());
        // The ground drives exist on a jet too (the same 2026-07-28 gate).
        for t in ["auto_taxi", "line_up", "taxi_to_parking"] {
            assert!(reg.get(t).is_some(), "a jet needs {t}");
        }
        // Turbine start, not the piston one-shot.
        assert!(reg.get("start_engines").is_some());
        assert!(reg.get("start_engine").is_none());
        let s = p.turboprop_start.as_ref().expect("a start sequence");
        // No APU anywhere on this airframe: nothing to bring up before cranking.
        assert!(s.apu.is_none(), "the SF50 has no APU");
        // The ignition knob is a PREREQUISITE, not a nicety: the START button's
        // handler only latches when knob_state == 1, so pressing it with the
        // knob at OFF silently sets button_state to 0 instead.
        assert_eq!(
            s.ignition_arm.first().map(String::as_str),
            Some("laminar/SF50/ignition_up")
        );
        assert_eq!(
            s.start_up.first().map(String::as_str),
            Some("laminar/SF50/eng_start_stop_toggle")
        );
        // Momentary presses, both of them — nothing to hold and nothing to
        // release (the Citation X's starter is the opposite case).
        assert!(!s.starter_held, "the SF50's controls are momentary presses");
        assert!(s.start_dn.is_empty());
        // Securing the aircraft is the KNOB, then the START/STOP button, not
        // the mixture (the script rewrites it): `ignition_down` drops the boost
        // pump and igniter, and only a button press with the knob at OFF sets
        // the button state the script's shutdown runs on. The knob alone left
        // the engine running at LSZS (2026-09-24).
        assert_eq!(
            s.shutdown_commands,
            vec![
                "laminar/SF50/ignition_down".to_string(),
                "laminar/SF50/eng_start_stop_toggle".to_string()
            ]
        );
        // ...and every airframe that has no such control keeps the old
        // lever-only shutdown untouched.
        for id in [
            "dhc6_twin_otter",
            "king_air_c90b",
            "lancair_evolution",
            "citation_x",
        ] {
            let t = crate::aircraft::load(id)
                .and_then(|a| a.turboprop_start)
                .unwrap_or_else(|| panic!("{id} start"));
            assert!(t.shutdown_commands.is_empty(), "{id} secures on the levers");
        }
        // A turbofan's idle N1 is nowhere near the PT6 default of 50, and the
        // script's own figure (N2 > 54, where it cuts its starter) says nothing
        // about N1 — too high a threshold waits for a number that never comes.
        assert!(s.n1_idle < 30.0, "{}", s.n1_idle);
        // The stall band is set so the flap-adjusted stall lands on the real
        // Vso of 67, clear of the POM's own 87 kt landing speed: the .acf's 75
        // warns at the stall itself, and the POM's Vs1 of 86 cautions on every
        // approach ever flown.
        let r = p.reference.as_ref().expect("reference figures");
        let vs_clean = p.vs_clean_kts.expect("clean stall");
        let full_flap_vs = vs_clean * (1.0 - 0.17);
        let approach = r.approach_kts.expect("approach speed");
        assert!(
            1.20 * full_flap_vs < approach,
            "the caution band must sit below the approach speed: {:.0} vs {approach}",
            1.20 * full_flap_vs
        );
        assert!(
            1.10 * full_flap_vs > r.vs0_kts.expect("vs0"),
            "the warning must come BEFORE the stall"
        );
        // Vy carries a climb-out speed, not the POM's 160 kt cruise climb.
        let vy = r.vy_kts.expect("vy");
        assert!(vy <= 140.0, "vy is a climb-out speed, got {vy}");
        // Three flap positions, takeoff at the middle one, descending Vfe
        // ladder. The labels are DEGREES (the EIS gauge is scaled UP/10/35 and
        // the checklist says "FLAPS - SET (10 Degrees)"), not the lever's
        // 50%/100% - both name the same three positions.
        let f = p.flaps.as_ref().expect("flaps");
        assert_eq!(f.detents.len(), 3);
        assert_eq!(f.takeoff_ratio, Some(0.5));
        assert_eq!(f.detents[1].label, "10");
        assert_eq!(f.detents[2].label, "35");
        assert!(f.detents[2].vfe_kts < f.detents[1].vfe_kts);
        // The gear limit is the COCKPIT PLACARD ("Vlo ret 160 KIAS", legible in
        // the POM's p.50 photograph), not the p.61 table — which gives Vlo 170
        // against a Vle of 150, an extended limit below the operating limit,
        // which no aircraft is placarded with. The placard's other half (Vle
        // 210) matches the .acf exactly, which is what makes the reading safe.
        assert_eq!(p.vlo_kts, Some(160.0));
        // Three exterior lights and no taxi light: the cockpit object's
        // complete manipulator list is nav, landing and strobe, whatever the
        // POM's before-taxi and parking checklists call for.
        let keys: Vec<&str> = p.lights.iter().map(|l| l.key.as_str()).collect();
        assert_eq!(keys, vec!["nav", "strobe", "landing"]);
        // The checklist is transcribed from the POM's image-only pages, and
        // parses into runnable lists rather than a wall of call-outs.
        let text = std::fs::read_to_string(p.checklist.as_ref().expect("a checklist"))
            .expect("the checklist file exists");
        let lists = super::checklist::parse(&text, &p);
        assert!(
            lists.len() >= 8,
            "one list per POM section: {}",
            lists.len()
        );
        // A checklist whose every line is a call-out is a transcription, not a
        // copilot. The map must actually connect: the lights, gear, flaps,
        // brake, transponder and electrical items are ours to perform.
        use super::checklist::Item;
        let acted = lists
            .iter()
            .flat_map(|c| &c.items)
            .filter(|i| matches!(i, Item::Act { .. }))
            .count();
        assert!(
            acted >= 12,
            "the map must connect: only {acted} acted items"
        );
        // ...and the "AS REQUIRED" / "CHECK" rows must stay observe-only —
        // writing a probe-heat item the POM leaves to the pilot is not ours.
        let forced = lists.iter().flat_map(|c| &c.items).any(|i| {
            matches!(i, Item::Verify { say, settable, .. }
                     if *settable && say.to_ascii_lowercase().contains("as required"))
        });
        assert!(!forced, "an 'as required' item must never be auto-set");
        // ...and no ITEM may call for a taxi light, which this aircraft has not
        // got — the two POM rows are dropped on purpose. (The `#` header
        // explains the omission, so only real item lines are checked.)
        assert!(
            !text
                .lines()
                .filter(|l| l.trim_start().starts_with("sw_item"))
                .any(|l| l.to_ascii_lowercase().contains("taxi light")),
            "the taxi-light rows must stay dropped"
        );
        // The fuel selector has no OFF position (POM p.29: AUTO/LEFT/RIGHT).
        let fs = p.fuel_selector.as_ref().expect("fuel selector");
        assert!(fs.positions.iter().all(|q| q.key != "off"));
        assert_eq!(fs.positions.len(), 3);
    }

    #[test]
    fn md82_is_a_jet_with_gear_no_piston_tools() {
        let p = super::aircraft::load("md82").expect("md82 profile loads");
        assert_eq!(p.id, "md82");
        assert!(p.autopilot && p.throttle && p.retractable_gear);
        assert!(
            !p.engine && !p.magnetos && !p.carb_heat,
            "jet has no piston systems"
        );
        assert_eq!(p.vne_kts, Some(340.0)); // Vmo
        let reg = super::tools::Registry::for_aircraft(&p);
        assert!(reg.get("gear").is_some(), "retractable gear -> gear tool");
        assert!(
            reg.get("set_throttle").is_some(),
            "throttle flag -> thrust levers"
        );
        assert!(reg.get("set_mixture").is_none(), "no mixture on a jet");
        assert!(
            reg.get("start_engine").is_none(),
            "no piston start on a jet"
        );
        assert!(reg.get("set_flaps").is_some());
        // Electrical is wired (battery, generators, avionics, "all"), so "all
        // electrical systems on" has a tool to land on.
        assert!(reg.get("set_electrical").is_some());
        // "All electrical systems on" is the procedural power_up, not a one-shot key.
        assert!(reg.get("power_up").is_some());
        // ...but only the ON direction remaps: "electrics off" must stay
        // set_electrical (the remap once powered the aircraft back up on a
        // shutdown request).
        use serde_json::json;
        assert!(super::wants_procedural_power_up(
            "set_electrical",
            &json!({"system":"all","on":true})
        ));
        assert!(super::wants_procedural_power_up(
            "set_electrical",
            &json!({"system":"all"})
        ));
        assert!(!super::wants_procedural_power_up(
            "set_electrical",
            &json!({"system":"all","on":false})
        ));
        assert!(!super::wants_procedural_power_up(
            "set_electrical",
            &json!({"system":"battery","on":true})
        ));
        // Jet start sequence, not the piston one-shot.
        assert!(reg.get("start_engines").is_some());
        assert!(reg.get("start_engine").is_none());
        // Pitot heat goes through the overhead heat knob (the stock command is a
        // no-op on this airframe), and "speed for landing" has a figure to use.
        assert_eq!(
            p.pitot_heat_dataref.as_deref(),
            Some("laminar/md82/ice/heatknob")
        );
        assert_eq!(
            p.reference.as_ref().and_then(|r| r.approach_kts),
            Some(140.0)
        );
        // The runway the automation demands must fit runways this aircraft
        // actually operates from. The recited 7,450 ft is a FACTORED FIELD
        // LENGTH at max weight; read as a ground roll it earned a x1.4 on top
        // and asked for 3,179 m — more than the whole of Luton's 2,162 m 25,
        // so the line-up backtracked for length it did not need and full auto
        // then failed its own lined-up test twice (2026-07-28).
        let roll_m = p
            .reference
            .as_ref()
            .and_then(|r| r.takeoff_roll_ft.or(r.takeoff_distance_ft))
            .expect("md82 declares a takeoff roll")
            * 0.3048;
        assert!(
            roll_m * 1.4 < 2162.0,
            "an MD-82 must be able to depart Luton: needs {:.0} m of 2162",
            roll_m * 1.4
        );
        // ...and the recited figure is still the POM one, unchanged.
        assert_eq!(
            p.reference.as_ref().and_then(|r| r.takeoff_distance_ft),
            Some(7450.0)
        );
        // This model has no autothrottle, whatever the real aircraft has: the
        // speed tool must be the throttle governor, not an AP bug with nothing
        // behind it. Declaring one also withheld `hold_speed` entirely, so
        // "reduce speed" did nothing at all (2026-07-28).
        let cit_reg = super::tools::Registry::for_aircraft(
            &super::aircraft::load("citation_x").expect("citation_x loads"),
        );
        assert!(
            cit_reg.get("hold_speed").is_some(),
            "speed is the governor's"
        );
        assert!(cit_reg.get("set_speed").is_none());
        assert!(cit_reg.get("autothrottle").is_none());
        // A toggle-only system must not be skipped by the bulk power-up: the
        // Citation's avionics master has no ON command, so "all electrical on"
        // fired nothing for it and full auto left the radios dead while
        // announcing the electrical was set (2026-07-28).
        let cit = super::aircraft::load("citation_x").expect("citation_x loads");
        let toggles = super::tools::all_electrical_toggles(&cit.electrical);
        assert!(
            toggles.iter().any(|t| t.dataref.contains("avionics")),
            "the bulk power-up must reach the avionics master"
        );
        // The ground drives exist on a jet too. They were gated behind
        // `!autothrottle` alongside the airborne speed tool, so neither jet had
        // a taxi, line-up or park tool and "taxi for me" reached nothing —
        // only full auto worked, because it builds its own (2026-07-28).
        for t in ["auto_taxi", "line_up", "taxi_to_parking"] {
            assert!(reg.get(t).is_some(), "a jet needs {t}");
        }
        // ...but the airborne speed tool is still the AP's bug on an airliner.
        assert!(
            reg.get("hold_speed").is_none(),
            "an autothrottle airframe sets the speed bug, not a throttle hold"
        );
        assert!(reg.get("set_speed").is_some());
        // A jet taxis like a jet: the GA 10/6/4 is a brake setting on this
        // airframe, which rolls past 10 knots at idle thrust on its own.
        let pace = super::autotaxi::Pace::from_profile(p.taxi.as_ref());
        assert!(
            pace.straight > 15.0 && pace.turn >= 8.0,
            "MD-82 must taxi briskly, got {pace:?}"
        );
        // ...and the piston 172 is the other way round.
        let c172 =
            super::tools::Registry::for_aircraft(&super::aircraft::load("cessna_172sp").unwrap());
        assert!(c172.get("start_engines").is_none());
        assert!(c172.get("start_engine").is_some());
        // Reference card answers from the POM.
        assert!(super::reference_limits(&p, "service ceiling")
            .unwrap()
            .contains("37,000"));
        // "What are we in" describes it as a twin-engine jet, not single-engine.
        let info = super::aircraft_info(&p, "what aircraft are we in").unwrap();
        assert!(info.contains("twin-engine jet"), "got: {info}");
        assert!(!info.contains("single-engine"));
        // The MD-82 checklist parses.
        assert!(crate::checklist::load(&p).is_ok());
    }

    #[test]
    fn join_and_reads_naturally() {
        assert_eq!(join_and(&["flaps".into()]), "flaps");
        assert_eq!(
            join_and(&["flaps".into(), "throttle".into()]),
            "flaps and throttle"
        );
        assert_eq!(
            join_and(&["flaps".into(), "throttle".into(), "mixture".into()]),
            "flaps, throttle, and mixture"
        );
    }

    #[test]
    fn abort_is_distinct_from_skip_and_yes() {
        let is_abort = |w: &str| crate::flow::classify(w) == Some(crate::flow::Reply::Abort);
        // Abort the whole checklist.
        for w in ["stop", "abort", "cancel", "belay that", "disregard"] {
            assert!(is_abort(w), "{w:?} should abort");
            assert!(!is_affirmative(w), "{w:?} is not a yes");
        }
        // A plain no/negative skips the step but isn't an abort.
        for w in ["no", "negative", "not yet"] {
            assert!(!is_abort(w), "{w:?} should not abort");
            assert!(!is_affirmative(w));
        }
        // Yes proceeds and isn't an abort.
        for w in ["affirm", "yes", "go ahead", "do it"] {
            assert!(is_affirmative(w));
            assert!(!is_abort(w));
        }
    }

    use super::looks_like_command;

    #[test]
    fn real_commands_pass_the_domain_gate() {
        for c in [
            "flaps ten",
            "gear up",
            "landing light on",
            "strobes off",
            "autopilot heading 270",
            "climb and maintain 7000",
            "descend at 500",
            "throttle 60 percent",
            "mixture full rich",
            "start the engine",
            "battery on",
            "squawk 1200",
            "align the gyro",
            "pitot heat on",
            "fuel pump on",
            "carb heat off",
            "magnetos both",
            "trim nose up",
            "nav one 109.5",
            "set the OBS to 240",
            "com two 121.9",
            "transponder standby",
            "prepare the ILS for runway 13",
            "flood lights on",
            // Phrasings that previously fell through the gate (issue: rejected
            // before reaching the LLM).
            "electrics off",
            "fix the gyro drift",
            "master switch on",
            "generator on",
            "squawk VFR",
            // Verbatim whisper output from logs/ that the gate used to reject —
            // recovered by the ignition/ascend/speed keywords.
            "Ignition on.",
            "Ascend 500 feet per minute.",
            "Physical speed 500.",
            // Whisper glued "flood lights" into one token; the compound and its
            // "blood" mishearing must still reach the LLM.
            "Floodlights on.",
            "Turn on the floodlights.",
            "Increase floodlights.",
            "Bloodlights on.",
            "landinglight on",
            // Phase-config macro invocations must reach the LLM.
            "configure for takeoff",
            "set up for takeoff",
            "takeoff config",
            "get ready for takeoff",
            "prep for departure",
            "configure for landing",
            "get ready for landing",
            "after landing",
            "clean up",
            "clear of the runway",
            "secure the aircraft",
            "secure",
            "full shutdown",
            // "checklist" carries any phase past the gate, even hyphenated STT.
            "Normal take-off checklist.",
            "starting engine checklist",
            "preflight checklist",
            // The bare rollout call — silently gated on the Twotter's first
            // reverse flight ("thrusters" is too far from "thrust" for fuzzy).
            "Reverse thrusters.",
            "reversers",
            "stow reverse",
            "cancel reverse",
            // Split compounds — swallowed with no miss chime, live 2026-07-20.
            "Take off for me.",
            "Let's take off.",
            "You have to take off.",
            "Line us up.",
            "line me up",
            "Get us lined up.",
            // Auto circuit — the whole pattern. "Do a circuit" was dropped as
            // not-a-command (circuit vocabulary missing from the gate), live
            // 2026-07-23.
            "Do a circuit.",
            "fly a circuit",
            "fly the pattern",
            "circuits and bumps",
            "make a right circuit",
            "take off and land",
        ] {
            assert!(looks_like_command(c), "{c:?} should pass the domain gate");
        }
    }

    #[test]
    fn circuit_command_survives_every_gate_for_the_172() {
        // The regression shipped 2026-07-23: "do a circuit" was GREEN in the
        // grammar unit test yet dead in the running app, because two main-loop
        // gates UPSTREAM of the parser rejected it first — the domain gate
        // (circuit not a keyword) and the question guard ("do" was a blanket
        // opener). Unit-testing the parser in isolation missed both. This runs
        // the WHOLE gate sequence a spoken command actually passes through, on
        // the real 172 registry, so the next such gap fails here.
        let prof = crate::aircraft::load("cessna_172sp").expect("172 profile");
        let reg = crate::tools::Registry::for_aircraft(&prof);
        assert!(
            reg.get("circuit").is_some(),
            "the 172 registers the circuit tool"
        );
        assert!(
            reg.get("full_auto").is_some(),
            "the 172 registers the full_auto tool"
        );
        // "do a circuit" is the whole flight to the pilot, so it routes to the
        // full-auto orchestrator (which flies the airborne circuit itself once
        // on the runway).
        for cmd in [
            "do a circuit",
            "fly a circuit",
            "take off and land",
            "circuits and bumps",
            "make a right circuit",
        ] {
            assert!(looks_like_command(cmd), "{cmd:?} must pass the domain gate");
            assert!(
                !super::looks_like_question(cmd),
                "{cmd:?} must not be withheld as a question"
            );
            let parsed = super::grammar::try_parse(cmd, &reg, &prof)
                .unwrap_or_else(|| panic!("{cmd:?} must parse to a command"));
            assert_eq!(
                parsed[0].name, "full_auto",
                "{cmd:?} must route to full_auto"
            );
        }
    }

    #[test]
    fn release_intent_catches_every_live_garble() {
        // Every phrasing the pilot actually tried while fighting to take the
        // throttle back from the taxi governor — all must release.
        for t in [
            "my throttle",
            "my thruffles",
            "mic control",
            "my throat",
            "release immediately",
            "let me go. stop touching the throttle",
            "give me command",
            "give me command now",
            "my power",
            "stop holding speed",
            "i have control",
            "im taking control",
        ] {
            assert!(super::release_intent(t), "{t:?} must release");
        }
        // Ordinary commands must not trip it (it only runs while a hold is
        // active, but even then these should pass through).
        for t in [
            "taxi at 10 knots",
            "flaps ten",
            "hold 70 knots",
            "landing light on",
        ] {
            assert!(!super::release_intent(t), "{t:?} must not release");
        }
    }

    use super::correction_value;
    use super::looks_like_followup;

    #[test]
    fn corrections_yield_the_bare_value() {
        // The live case: "Power 8 percent" executed, then the fix dead-ended.
        assert_eq!(
            correction_value("I said 80 percent."),
            Some("80 percent".into())
        );
        assert_eq!(
            correction_value("no, two five zero"),
            Some("two five zero".into())
        );
        assert_eq!(correction_value("No, I said forty"), Some("forty".into()));
        assert_eq!(
            correction_value("make it sixty percent"),
            Some("sixty percent".into())
        );
        assert_eq!(correction_value("set it to 55"), Some("55".into()));
        assert_eq!(
            correction_value("set that to two six"),
            Some("two six".into())
        );
        assert_eq!(
            correction_value("correction, one thousand five hundred"),
            Some("one thousand five hundred".into())
        );
        assert_eq!(
            correction_value("How about eighty percent?"),
            Some("eighty percent".into())
        );
        // Whisper dropping the "how" (heard live on the Twotter flight).
        assert_eq!(
            correction_value("About 40 percent?"),
            Some("40 percent".into())
        );
        assert_eq!(correction_value("about we start the descent"), None);
        // "how about" with a full command is a question for the copilot, not a
        // value correction.
        assert_eq!(correction_value("how about we start the descent"), None);
        // Full commands and non-values are NOT corrections — they take the
        // normal path ("no, flaps up" must not become "throttle flaps up").
        assert_eq!(correction_value("no, flaps up"), None);
        assert_eq!(correction_value("no"), None);
        assert_eq!(correction_value("I said start the engines"), None);
        assert_eq!(correction_value("set it to idle"), None);
        assert_eq!(correction_value("north"), None, "no prefix, no correction");
        assert_eq!(correction_value("flaps ten"), None);
    }

    #[test]
    fn followups_are_recognized_but_plain_chatter_is_not() {
        for c in [
            "switch it back to both",
            "do that again",
            "a bit more",
            "undo that",
            "the other tank",
            "seventy percent",
            "60%",
            "two thousand",
        ] {
            assert!(looks_like_followup(c), "{c:?} should look like a follow-up");
        }
        for c in [
            "nice weather today",
            "where are we",
            "hello copilot",
            // A long counting string is noise, not a bare-value follow-up.
            "one two three four five six seven eight ten",
        ] {
            assert!(!looks_like_followup(c), "{c:?} should not");
        }
    }

    use super::{command_subject, has_number};

    #[test]
    fn bare_value_followups_re_anchor_to_the_last_subject() {
        assert_eq!(command_subject("throttle eighty percent"), "throttle");
        assert_eq!(command_subject("set heading 270"), "set heading");
        assert_eq!(command_subject("flaps ten"), "flaps");
        assert_eq!(command_subject("60 percent"), ""); // starts with a number
        assert!(has_number("sixty percent") && has_number("60%") && has_number("two thousand"));
        assert!(!has_number("switch it back to both"));
        // The rewrite a bare follow-up would produce:
        assert_eq!(
            format!(
                "{} {}",
                command_subject("throttle eighty percent"),
                "sixty percent"
            ),
            "throttle sixty percent"
        );
    }

    #[test]
    fn location_answers_from_the_profile() {
        use super::location_answer;
        let p = crate::aircraft::load("king_air_c90b").expect("c90b");
        let a = location_answer("Where is the inverter?", &p).expect("known location");
        assert!(a.to_lowercase().contains("subpanel"), "{a}");
        // Aliased phrasing + longest-key preference.
        assert!(super::location_answer("where are the boost pumps", &p).is_some());
        // Unknown items and non-questions fall through.
        assert!(super::location_answer("where is the flux capacitor", &p).is_none());
        assert!(super::location_answer("inverter on", &p).is_none());
    }

    #[test]
    fn glued_light_compound_does_not_overmatch() {
        // Stripping a light/lights suffix must not wave through non-cockpit words.
        for g in [
            "moonlight",
            "highlight",
            "delight",
            "spotlight",
            "flashlight",
        ] {
            assert!(!looks_like_command(g), "{g:?} should still be rejected");
        }
    }

    #[test]
    fn one_letter_mishearings_pass_the_gate() {
        // Whisper drops/swaps a letter; these should still reach the LLM.
        for c in ["trust 80 percent", "descent 500", "altitude 3000"] {
            assert!(looks_like_command(c), "{c:?} should pass via near-miss");
        }
    }

    #[test]
    fn alias_folded_commands_pass_the_gate() {
        // The normalizer's alias table is part of the gate now: "q and h
        // 1014" (four QNH settings swallowed live, 2026-07-20) only becomes
        // the "qnh" keyword after folding, and the brake homophone rides
        // its own keywords.
        for c in [
            "Q and H 1014.",
            "Change Q and H to 1014.",
            "Release breaks.",
            "Q&H 1013",
        ] {
            assert!(looks_like_command(c), "{c:?} should pass the domain gate");
        }
    }

    /// "Stop" is a control word and must reach the router from a cold gate.
    /// It was missing from the keyword table, so in continuous mode it only
    /// ever worked through the pre-gate branch that fires while the speed
    /// governor is running. With a push under way — brakes off for the tug,
    /// no governor — a Baron rolled off its Southampton stand to 21 kt and
    /// "Stop." at 0.9996 confidence was dropped as ambient, no miss chime
    /// (2026-08-01).
    #[test]
    fn stop_reaches_the_router_with_no_governor_running() {
        for c in ["Stop.", "stop", "Stop stop stop!", "Stop the aircraft."] {
            assert!(looks_like_command(c), "{c:?} must pass the domain gate");
        }
        // Passing the GATE is not actuating: the grammar's bare-stop rule
        // still wants the whole utterance, so a sentence that merely contains
        // the word goes to the router, not to the brakes.
        let prof = crate::aircraft::load("beechcraft_baron_58").expect("Baron profile");
        let reg = crate::tools::Registry::for_aircraft(&prof);
        let stop = super::grammar::try_parse("stop", &reg, &prof)
            .expect("bare stop must parse to a command");
        assert_eq!(stop[0].name, "hold_speed");
        assert_eq!(stop[0].arguments["knots"], 0);
        assert!(
            super::grammar::try_parse("we should stop for fuel at the next field", &reg, &prof)
                .is_none(),
            "a sentence that merely contains the word must not brake"
        );
    }

    #[test]
    fn phonetic_garbage_is_rejected() {
        for g in [
            "Nautilapuku",
            "Mokuli ke maluku",
            "Reki ki panaki",
            "Tiki Maliki, WaKiPi",
            "Take a look at the video",
            "",
        ] {
            assert!(!looks_like_command(g), "{g:?} should be rejected");
        }
    }

    #[test]
    fn affirmatives_proceed() {
        for r in [
            "affirm",
            "affirmative",
            "yes",
            "confirm",
            "roger that",
            "correct",
            "go ahead",
        ] {
            assert!(is_affirmative(r), "{r:?} should confirm");
        }
    }

    #[test]
    fn negatives_and_ambiguity_cancel() {
        // explicit no, and anything not clearly yes, must NOT proceed
        for r in [
            "negative",
            "no",
            "cancel",
            "stop",
            "abort",
            "uh what",
            "descend 700",
            "",
        ] {
            assert!(!is_affirmative(r), "{r:?} must not confirm");
        }
    }

    use super::is_bare_value;

    #[test]
    fn bare_value_gates_followup_rewrite() {
        // Genuine bare follow-ups (re-anchored to the last command).
        assert!(is_bare_value("seventy percent"));
        assert!(is_bare_value("60%"));
        assert!(is_bare_value("two thousand five hundred"));
        // Mishears / real words — must NOT be force-anchored to the last tool.
        assert!(!is_bare_value("fraudal at eighty percent"));
        assert!(!is_bare_value("total thirty percent"));
        assert!(!is_bare_value("throttle eighty"));
        assert!(!is_bare_value("switch it back"));
    }

    #[test]
    fn battery_low_is_system_class_aware() {
        use super::battery_low;
        // 24V-class: the flat 20.5V we saw live is low; 27.5V charged is not.
        assert!(battery_low(20.5));
        assert!(!battery_low(27.5));
        assert!(!battery_low(24.5));
        // 12V-class must not be flagged low just for being under 23.
        assert!(!battery_low(12.6));
        assert!(!battery_low(13.8));
        assert!(battery_low(10.9));
    }

    use super::looks_like_question;

    #[test]
    fn fuel_query_detects_questions_not_system_commands() {
        use super::is_fuel_query;
        assert!(is_fuel_query("how much fuel do we have"));
        assert!(is_fuel_query("what's our fuel"));
        assert!(is_fuel_query("how much gas"));
        assert!(is_fuel_query("what's our endurance"));
        // Fuel *system* commands must NOT be hijacked — they go to their tools.
        assert!(!is_fuel_query("fuel pump on"));
        assert!(!is_fuel_query("fuel selector both"));
        assert!(!is_fuel_query("boost pump on"));
        assert!(!is_fuel_query("flaps ten"));
        // "fuel flow" is an engine-instrument read (answer_state_query), not the
        // total-fuel report.
        assert!(!is_fuel_query("what's the fuel flow"));
        // Selector commands without "tank"/"selector" said (seen live: answered
        // with the quantity report five times in one session).
        assert!(!is_fuel_query("switch fuel to right"));
        assert!(!is_fuel_query("set fuel to left"));
        assert!(!is_fuel_query("switch fuel"));
        assert!(!is_fuel_query("switch tanks"));
        // Quantity questions that mention a side or a tank stay questions.
        assert!(is_fuel_query("how much fuel in the left tank"));
        assert!(is_fuel_query("how much fuel do we have left"));
    }

    #[test]
    fn power_loss_query_detects_diagnosis_not_commands() {
        use super::is_power_loss_query;
        // The live unrecognized phrasings, plus common variants.
        assert!(is_power_loss_query("why we lost power?"));
        assert!(is_power_loss_query("why have we lost power?"));
        assert!(is_power_loss_query("we're losing power"));
        assert!(is_power_loss_query("the engine quit"));
        assert!(is_power_loss_query("engine failure"));
        assert!(is_power_loss_query("why did the engine stopped")); // STT grammar drift
        assert!(is_power_loss_query("engine cut out"));
        // Commands and unrelated mentions must not be hijacked.
        assert!(!is_power_loss_query("reduce power by ten percent"));
        assert!(!is_power_loss_query(
            "reduce power, we lost too much altitude"
        ));
        assert!(!is_power_loss_query("stop the engine"));
        assert!(!is_power_loss_query("start the engine"));
        assert!(!is_power_loss_query("full power"));
    }

    #[test]
    fn whisper_label_strips_path_prefix_and_suffix() {
        use super::whisper_model_label;
        assert_eq!(whisper_model_label("models/ggml-base.en.bin"), "base.en");
        assert_eq!(
            whisper_model_label("/a/b/ggml-large-v3-turbo.bin"),
            "large-v3-turbo"
        );
        assert_eq!(whisper_model_label("small.en"), "small.en");
    }

    #[test]
    fn questions_are_detected_commands_are_not() {
        for q in [
            "how much fuel do we have",
            "what's our heading",
            "what is our altitude?",
            "are we configured for landing",
            "how high are we",
            "tell me the airspeed",
            "do we have enough fuel to make it?",
            "where are we",
            // Keyword+number questions: same shape as a command, so they must be
            // caught here and withheld from the grammar fast-path (which would
            // otherwise *act* on them) — see the fast-path gate in run_session.
            "is the heading 270?",
            "did we set flaps to ten?",
            // Contraction openers, in raw and apostrophe-stripped forms: without
            // them "Where's the parking brake?" parsed as a command and SET the
            // brake (seen live). Never an action from a question shape.
            "where's the parking brake",
            "wheres the parking brake",
            "who's flying",
            "when's our descent",
        ] {
            assert!(looks_like_question(q), "{q:?} should be a question");
        }
        for c in [
            "flaps ten",
            "gear up",
            "set heading 270",
            "throttle 60 percent",
            "landing light on",
            "start the engine",
            // do-support imperatives: "do <article/noun>" is a COMMAND, not a
            // question. "Do a circuit" missed live because "do" was a blanket
            // question opener and swallowed the parsed command (2026-07-23).
            "do a circuit",
            "do a touch and go",
            "do a go around",
            "do another circuit",
        ] {
            assert!(!looks_like_question(c), "{c:?} should not be a question");
        }
    }

    /// Configuration-state questions ("is the gear down?") carry the same
    /// keywords as their command twins ("gear down"), so the question OPENER
    /// is what routes them to the answer path — these must all read as
    /// questions, and the loop's gate then withholds them from the grammar
    /// (which would otherwise ACT on the words inside).
    #[test]
    fn config_state_questions_route_to_the_answer_path() {
        for q in [
            "is the gear down",
            "are the flaps up",
            "is the parking brake set",
            "is the pitot heat on",
            "where are the flaps",
        ] {
            assert!(looks_like_question(q), "{q:?} should be a question");
        }
        for c in ["gear down", "flaps up", "parking brake on", "pitot heat on"] {
            assert!(!looks_like_question(c), "{c:?} must stay a command");
        }
    }

    use super::correction_text;

    #[test]
    fn correction_extracts_value_from_negative_reply() {
        assert_eq!(correction_text("negative, 500").as_deref(), Some("500"));
        assert_eq!(
            correction_text("no make it five hundred").as_deref(),
            Some("make it five hundred")
        );
    }

    #[test]
    fn plain_negation_is_not_a_correction() {
        assert_eq!(correction_text("negative"), None);
        assert_eq!(correction_text("no"), None);
        assert_eq!(correction_text("cancel."), None);
    }
