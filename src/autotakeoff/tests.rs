
    /// EGKK 2026-09-27: the reject left the tower's takeoff clearance live.
    /// The crew now reports it, in words the tower reads as the reject and
    /// never as a request, and only when the tower has us on its runway.
    #[test]
    fn a_reject_is_reported_to_the_tower_that_cleared_it() {
        use crate::tower::{parse_intent, Intent, State};
        let call = crate::circuit::radio_call(
            &Some("Whiskey Alpha Tango one seven three".into()),
            super::REJECT_CALL,
        );
        assert_eq!(
            parse_intent(&call.to_ascii_lowercase()),
            Intent::RejectingTakeoff,
            "{call}"
        );
        assert!(super::reject_call_due(Some(State::TakeoffCleared)));
        assert!(super::reject_call_due(Some(State::LinedUp)));
        for quiet in [
            None,
            Some(State::Parked),
            Some(State::TaxiCleared),
            Some(State::Inbound),
        ] {
            assert!(!super::reject_call_due(quiet), "{quiet:?}");
        }
    }

    /// The Edinburgh refusal, from the measured geometry (2026-09-20): on
    /// runway 24, aligned to 0.9 degrees, dead on the centreline, 2,408 m
    /// along a 2,549 m strip. The fit correctly refused — 141 m of runway
    /// ahead against a 500 m minimum — and then said "we're not on the
    /// runway yet" while the tower said "you are on runway two four".
    /// ETND 2026-09-24, the recorded numbers: cleared 26R (the grass strip
    /// beside 08/26), stopped at 52.585512 N 8.349835 E facing 016 true, and
    /// the refusal named 08L, the reciprocal, "152 metres away". It names
    /// the cleared end now, at the same distance; without a clearance the
    /// best-aligned strip is still what it names.
    #[test]
    fn the_line_up_refusal_names_the_cleared_runway() {
        let dir = std::env::temp_dir().join(format!("xvl_etnd_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let apt = dir.join("apt.dat");
        std::fs::write(
            &apt,
            "1    126 0 0 ETND Diepholz AB\n\
             100 50.00 3 0 0.00 0 0 0 08L 52.5862935 8.3393953 0 0 1 0 0 0 26R 52.5866670 8.3459906 0 0 1 0 0 0\n\
             100 45.11 1 0 0.25 0 2 0  08 52.5850000 8.3318780 12 0 1 0 0 0 26 52.5859870 8.3501930 0 0 1 3 0 0\n",
        )
        .unwrap();
        let (lat, lon, psi) = (52.585511622928735, 8.349835096044318, 16.0);
        let cands = crate::autoland::nearby_runways(apt.to_str().unwrap(), lat, lon, 3.0).unwrap();
        assert!(runway_fit(&cands, lat, lon, psi, 780.0).is_none());
        let old = why_no_fit(&cands, lat, lon, psi, 780.0);
        let NoFit::NoneNear { id, lateral_m } = &old else {
            panic!("{old:?}");
        };
        assert_eq!(id, "08L", "the recorded refusal");
        assert!((lateral_m - 152.0).abs() < 1.0, "{lateral_m}");

        let why = why_no_fit_cleared(&cands, lat, lon, psi, 780.0, Some("26R"));
        let NoFit::NoneNear { id, lateral_m } = &why else {
            panic!("{why:?}");
        };
        assert_eq!(id, "26R");
        assert!((lateral_m - 152.0).abs() < 1.0, "{lateral_m}");
        let (line, _) = no_fit_line(&why, false);
        assert!(line.starts_with("Runway two six right is 152 metres away"), "{line}");
        assert!(!line.contains("zero eight"), "{line}");

        // No clearance, or one the data does not know: as before.
        assert_eq!(why_no_fit_cleared(&cands, lat, lon, psi, 780.0, None), old);
        assert_eq!(why_no_fit_cleared(&cands, lat, lon, psi, 780.0, Some("33")), old);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_refused_takeoff_says_which_of_the_four_things_is_wrong() {
        // The 2,000 m north strip, and a position 1,900 m down it: aligned,
        // on the centreline, 100 m of runway left — Edinburgh's shape.
        let cands = vec![rwy_north()];
        let far = (1900.0 / 111_320.0, 0.0);
        match why_no_fit(&cands, far.0, far.1, 0.0, 500.0) {
            NoFit::TooLittleLeft {
                id, remaining_m, ..
            } => {
                assert_eq!(id, "36");
                assert!(
                    remaining_m < 200.0,
                    "a hundred metres of runway left, not a line-up problem: {remaining_m:.0}"
                );
            }
            other => panic!("expected TooLittleLeft, got {other:?}"),
        }
        // Lying ACROSS the runway — on it, and the worst place to be.
        assert!(matches!(
            why_no_fit(&cands, 0.0, 0.0, 90.0, 500.0),
            NoFit::NotAligned { .. }
        ));
        // Aligned at the right end: this one fits, so there is no diagnosis
        // to give.
        assert!(runway_fit(&cands, 0.0, 0.0, 0.0, 500.0).is_some());
        // Nothing anywhere near.
        assert_eq!(
            why_no_fit(&[], 0.0, 0.0, 0.0, 500.0),
            NoFit::NoneNear {
                id: String::new(),
                lateral_m: 0.0
            }
        );
        // EGPH 2026-09-20: at the handoff height at 203 kt against a 210 kt
        // clean floor. The handoff waits, flying, and goes once clean or fast
        // enough, and never waits past its bound or for a clean wing.
        assert!(handoff_waits_for_clean_up(false, 203.0, Some(210.0), 0.0));
        assert!(!handoff_waits_for_clean_up(false, 211.0, Some(210.0), 3.0));
        assert!(!handoff_waits_for_clean_up(true, 203.0, Some(210.0), 0.0));
        assert!(!handoff_waits_for_clean_up(false, 203.0, None, 0.0));
        assert!(!handoff_waits_for_clean_up(
            false,
            203.0,
            Some(210.0),
            CLEAN_UP_WAIT_S
        ));
        // The dial at 100 ft passing 1,150: set 2,000 up, to the thousand.
        assert_eq!(handoff_altitude_dial_ft(100.0, 1150.0), Some(4000.0));
        assert_eq!(handoff_altitude_dial_ft(1150.0, 1150.0), Some(4000.0));
        // The pilot's own level above us stands.
        assert_eq!(handoff_altitude_dial_ft(6000.0, 1150.0), None);
        assert_eq!(handoff_altitude_dial_ft(1650.0, 1150.0), None);
        // The words never deny a runway we are sitting on.
        for why in [
            NoFit::RunwayTooShort {
                id: "24".into(),
                length_m: 2549.0,
                needed_m: 2560.0,
            },
            NoFit::TooLittleLeft {
                id: "24".into(),
                remaining_m: 141.0,
                needed_m: 2560.0,
            },
            NoFit::NotAligned {
                id: "24".into(),
                err_deg: 35.0,
            },
        ] {
            for can_line_up in [true, false] {
                let (line, urgent) = no_fit_line(&why, can_line_up);
                assert!(urgent, "{line}");
                assert!(!line.contains("not on the runway"), "{line}");
            }
        }

        // THE RUNWAY ITSELF TOO SHORT. Edinburgh's shape: on the numbers,
        // aligned, full length ahead — and the aircraft needs more than the
        // strip has. No position fixes this, and the old answer sent the
        // pilot to line up and then to backtrack, twice, for eleven metres.
        match why_no_fit(&cands, 0.0, 0.0, 0.0, 2560.0) {
            NoFit::RunwayTooShort {
                length_m, needed_m, ..
            } => {
                assert!(length_m < needed_m, "{length_m} vs {needed_m}");
            }
            other => panic!("a short runway is not a position problem: {other:?}"),
        }
        // ...and it is reported even from the threshold, where "not enough
        // LEFT" would be technically true and completely useless.
        assert!(matches!(
            why_no_fit(&cands, 0.0, 0.0, 0.0, 2560.0),
            NoFit::RunwayTooShort { .. }
        ));

        // NOTHING NEAR, but we know WHAT we looked for. LFPB 2026-09-22: the
        // cleared runway 07 lay 169 m off, and both the tool and full auto
        // said only "I couldn't get us onto the runway". A strip 400 m abeam
        // the north runway is that shape.
        let abeam = why_no_fit(&cands, 0.0, 400.0 / (111_320.0 * 0.0_f64.cos()), 0.0, 500.0);
        match &abeam {
            NoFit::NoneNear { id, lateral_m } => {
                assert_eq!(id, "36");
                assert!((*lateral_m - 400.0).abs() < 5.0, "{lateral_m}");
            }
            other => panic!("expected NoneNear with the strip named, got {other:?}"),
        }
        let (line, urgent) = no_fit_line(&abeam, false);
        assert!(
            line.contains("three six") && line.contains("400 metres"),
            "{line}"
        );
        assert!(
            !urgent,
            "beside the field is not an occupied runway: {line}"
        );
        // ...and with no pavement at all in the data, the old wording stands —
        // there is nothing truthful to name.
        let (bare, _) = no_fit_line(
            &NoFit::NoneNear {
                id: String::new(),
                lateral_m: 0.0,
            },
            false,
        );
        assert!(bare.contains("not lined up on a runway"), "{bare}");
    }

    /// The BIKF departure (2026-09-20): commanded 8 degrees, flew 22.4. The
    /// law was pinned at its nose-down limit the whole way with the pitch
    /// RATE still building — it wanted about -0.38 and could ask for -0.10.
    #[test]
    fn the_rotation_can_push_hard_enough_to_hold_its_own_ceiling() {
        // The trace, replayed: 20.1 degrees against an 8 degree target with
        // the nose still coming up at 2 deg/s.
        let (theta, rate) = (20.1, 2.05);
        let mut ga = RotatePitch::with_authority(None, PITCH_MIN);
        let mut jet = RotatePitch::with_authority(Some(11.0), -0.5);
        // Walk both to the cap so the ramp is finished and the target is 8.
        for _ in 0..40 {
            ga.step(8.0, Some(0.0), 0.2);
            jet.step(8.0, Some(0.0), 0.2);
        }
        let ga_out = ga.step(theta, Some(rate), 0.2);
        let jet_out = jet.step(theta, Some(rate), 0.2);
        assert!(
            (ga_out - PITCH_MIN).abs() < 1e-6,
            "the GA limit saturates and stays there: {ga_out}"
        );
        assert!(
            jet_out < ga_out - 0.2,
            "the jet can actually push: {jet_out} vs {ga_out}"
        );
        // ...and the limit is still a limit: never past full travel, never a
        // standing pull dressed up as a minimum.
        let silly = RotatePitch::with_authority(None, 4.0);
        assert!(silly.pitch_min <= 0.0);
        let deep = RotatePitch::with_authority(None, -9.0);
        assert!(deep.pitch_min >= -1.0);
    }

    /// Flaps come up a detent at a time, each step waiting for its own
    /// manoeuvring speed. The old law took flaps 5 straight to 0 at 170 kt.
    #[test]
    fn the_flaps_walk_their_schedule_instead_of_going_straight_to_zero() {
        // The 737's declared schedule: up at 210, flaps 1 at 190, 2 at 180.
        let sched = [(0.0, 210.0), (0.125, 190.0), (0.25, 180.0)];
        // Flaps 5 at 170 kt, which is what the old rule cleaned up at.
        assert_eq!(flap_retract_step(&sched, 0.375, 170.0), None, "too slow");
        // 190 earns the first step, and ONLY the first.
        assert_eq!(flap_retract_step(&sched, 0.375, 190.0), Some(0.25));
        // From flaps 1, going clean waits for 210.
        assert_eq!(flap_retract_step(&sched, 0.125, 200.0), None);
        assert_eq!(flap_retract_step(&sched, 0.125, 210.0), Some(0.0));
        // Already clean: nothing left.
        assert_eq!(flap_retract_step(&sched, 0.0, 250.0), None);
        // Fast enough for everything still only takes ONE step: the flaps
        // have to travel through the detents either way.
        assert_eq!(flap_retract_step(&sched, 0.375, 300.0), Some(0.25));
        // An airframe with no declared speeds keeps the one-shot clean-up.
        assert_eq!(flap_retract_step(&[], 0.375, 300.0), None);
    }

    /// Held to the acceleration height, a jet is past every detent's speed
    /// at once: the schedule stepped 5 to 2 and 2 to 1 half a second apart
    /// (Alghero, 2026-09-27). The next step waits for the flaps to arrive.
    #[test]
    fn a_flap_step_waits_for_the_flaps_to_arrive() {
        let sched = [(0.0, 210.0), (0.125, 190.0), (0.25, 180.0)];
        // First step: the flaps sit at the takeoff 5.
        assert_eq!(paced_flap_step(&sched, 0.375, Some(0.375), 215.0), Some(0.25));
        // Next tick: commanded 2, still travelling from 5. No second step.
        assert_eq!(paced_flap_step(&sched, 0.25, Some(0.36), 215.0), None);
        // A failed read is no measurement.
        assert_eq!(paced_flap_step(&sched, 0.25, None, 215.0), None);
        assert_eq!(paced_flap_step(&sched, 0.25, Some(f64::NAN), 215.0), None);
        // Arrived (inside the tolerance): the next step.
        assert_eq!(paced_flap_step(&sched, 0.25, Some(0.26), 215.0), Some(0.125));
        // Arrived but too slow is still too slow.
        assert_eq!(paced_flap_step(&sched, 0.125, Some(0.125), 200.0), None);
    }

    /// #169: the stock A330's deploy ratio never came within 0.03 of CONF 1's
    /// handle, and the walk never stepped again. A reading that left the old
    /// detent and stopped counts as arrived; one still moving, one that never
    /// left the old detent (a jammed flap), or a failed read does not.
    #[test]
    fn a_stalled_deploy_reading_still_steps() {
        // The A330's schedule: CONF 1 (0.25) cleans up above 210.
        let sched = [(0.0, 210.0), (0.25, 180.0)];
        let mut pace = FlapPace::default();
        pace.stepped(0.5);
        let step = |pace: &mut FlapPace, t: f64, d: Option<f64>| {
            let judged = pace.judged(t, 0.25, d);
            paced_flap_step(&sched, 0.25, judged, 212.0)
        };
        // Travelling from CONF 2: no step.
        assert_eq!(step(&mut pace, 0.0, Some(0.45)), None);
        assert_eq!(step(&mut pace, 1.0, Some(0.40)), None);
        // Stopped short at 0.33, 0.005 of wander: not yet four seconds.
        for i in 0..8 {
            let t = 2.0 + i as f64 * 0.5;
            let d = 0.33 + if i % 2 == 0 { 0.0 } else { 0.004 };
            assert_eq!(step(&mut pace, t, Some(d)), None, "t {t}");
        }
        // Four seconds still: the step.
        assert_eq!(step(&mut pace, 6.0, Some(0.332)), Some(0.0));
        // A failed read is no measurement.
        assert_eq!(step(&mut pace, 6.5, None), None);
        // A gap in the reads starts the clock again.
        assert_eq!(step(&mut pace, 20.0, Some(0.332)), None);

        // Jammed at the detent the step left: never an arrival.
        let mut jam = FlapPace::default();
        jam.stepped(0.5);
        for i in 0..40 {
            assert_eq!(step(&mut jam, i as f64 * 0.5, Some(0.5)), None);
        }
        // Still creeping (0.02 in 4 s): not settled.
        let mut creep = FlapPace::default();
        creep.stepped(0.5);
        for i in 0..20 {
            let t = i as f64 * 0.5;
            assert_eq!(step(&mut creep, t, Some(0.40 - t * 0.005)), None, "t {t}");
        }
        // Arrived inside the tolerance needs no settling at all.
        let mut near = FlapPace::default();
        assert_eq!(step(&mut near, 0.0, Some(0.26)), Some(0.0));
        // #169, LEMG 30 (sortie 1790698132-full_auto, 2026-09-29): the
        // A330's CONF 1 reading falls past the handle (0.106, then 0.011,
        // still moving): past the detent on the way in is arrived at once.
        let mut slats = FlapPace::default();
        slats.stepped(0.5);
        assert_eq!(step(&mut slats, 0.0, Some(0.106)), Some(0.0));
        assert_eq!(step(&mut slats, 0.7, Some(0.011)), Some(0.0));
        // ...but not with no step to judge the direction by, and never a
        // reading still above the detent.
        let mut blind = FlapPace::default();
        assert_eq!(step(&mut blind, 0.0, Some(0.106)), None);
        let mut above = FlapPace::default();
        above.stepped(0.5);
        assert_eq!(step(&mut above, 0.0, Some(0.30)), None);
        // The withheld line: the first, then every ten seconds.
        let mut log = FlapPace::default();
        assert!(log.withheld_log_due(0.0));
        assert!(!log.withheld_log_due(5.0));
        assert!(log.withheld_log_due(10.0));
    }

    #[test]
    fn a_flap_step_is_called_by_its_detent() {
        assert_eq!(flap_step_line(0.125, Some("one")), "Flaps one.");
        assert_eq!(flap_step_line(0.25, Some("ten degrees")), "Flaps ten degrees.");
        assert_eq!(flap_step_line(0.0, Some("up")), "Flaps up.");
        assert_eq!(flap_step_line(0.0, None), "Flaps up.");
        // No name: the old line.
        assert_eq!(flap_step_line(0.125, None), "Flaps coming up a stage.");
        assert_eq!(flap_step_line(0.125, Some("  ")), "Flaps coming up a stage.");
    }

    /// The 737 walks 5, 1, up from its real profile, each named.
    #[test]
    fn the_737_walks_five_one_up() {
        let flaps = crate::aircraft::load("boeing_737_800").unwrap().flaps;
        let sched = crate::aircraft::flap_schedule(flaps.as_ref());
        let (mut at, mut calls) = (0.375_f64, Vec::new());
        while let Some(next) = paced_flap_step(&sched, at, Some(at), 250.0) {
            at = next;
            let name = crate::aircraft::flap_detent_spoken(flaps.as_ref(), next);
            calls.push((next, flap_step_line(next, name.as_deref())));
        }
        assert_eq!(
            calls,
            vec![(0.125, "Flaps one.".to_string()), (0.0, "Flaps up.".to_string())]
        );
    }

    #[test]
    fn the_gear_waits_for_its_own_warning_to_clear() {
        // The SF50's .acf: below 125 kt, flaps past 0.5 or thrust under 50%.
        let sf50 = crate::aircraft::gear_warning_from_acf(
            "P acf/_gear_warn_above_flap_rat 0.500000000\n\
             P acf/_gear_warn_below_kias 125.000000000\n\
             P acf/_gear_warn_below_thro_pct 50.000000000\n",
        );
        assert_eq!(
            sf50,
            Some(crate::aircraft::GearWarning {
                below_kias: 125.0,
                below_throttle_pct: Some(50.0),
                above_flap_ratio: Some(0.5),
            })
        );
        let vlo = Some(160.0);
        // ETND 26: 200 ft and 122 kt with the takeoff flap, the horn.
        assert!(!gear_up_due(203.0, 122.0, 0.5, sf50, vlo));
        assert!(!gear_up_due(340.0, 125.6, 0.5, sf50, vlo), "inside the margin");
        assert!(gear_up_due(380.0, 128.7, 0.5, sf50, vlo));
        // Clean, the flap clause is gone and full thrust is not the power one.
        assert!(gear_up_due(203.0, 110.0, 0.0, sf50, vlo));
        // Never below the floor, always by the latest height or the placard.
        assert!(!gear_up_due(150.0, 150.0, 0.0, sf50, vlo));
        assert!(gear_up_due(1000.0, 110.0, 0.5, sf50, vlo));
        assert!(gear_up_due(400.0, 151.0, 0.5, sf50, vlo));
        // An airframe with no declared warning keeps the old 200 ft rule.
        assert!(gear_up_due(201.0, 80.0, 0.5, None, None));
        assert_eq!(crate::aircraft::gear_warning_from_acf("P acf/_Vle_kts 150\n"), None);
    }
    #[tokio::test]
    async fn paused_missing_or_refused_modes_cannot_complete_the_takeoff_handoff() {
        let xp = crate::xplane::FakeXPlane::with_reads(&[
            (crate::datarefs::PAUSED, 1.0),
            (crate::datarefs::IS_IN_REPLAY, 0.0),
            (super::HDG_STATUS, 0.0),
            (super::VS_STATUS, 0.0),
            (super::SERVOS_ON, 0.0),
            (super::VS_DIAL, 700.0),
        ]);
        let mut last = None;
        assert!(!super::engage_climb(&xp, &[], &mut 0, 700.0, &mut last, &mut None).await);
        assert!(!xp.ran(super::CMD_SERVOS_ON));
        assert!(!xp.wrote(super::VS_DIAL));
        xp.set_read(crate::datarefs::PAUSED, 0.0);
        assert!(!super::engage_climb(&xp, &[], &mut 0, 700.0, &mut last, &mut None).await); // commands ignored
        assert!(xp.ran(super::CMD_VS_MODE));
        xp.set_read(super::HDG_STATUS, 1.0);
        xp.set_read(super::VS_STATUS, 1.0);
        // Lit modes with the servos out are not an autopilot flying (#160).
        assert!(!super::engage_climb(&xp, &[], &mut 0, 700.0, &mut last, &mut None).await);
        xp.set_read(super::SERVOS_ON, 1.0);
        assert!(super::engage_climb(&xp, &[], &mut 0, 700.0, &mut last, &mut None).await);
        xp.set_read(super::VS_STATUS, f64::NAN);
        assert!(!super::engage_climb(&xp, &[], &mut 0, 700.0, &mut last, &mut None).await);
        xp.set_read(super::VS_STATUS, 1.0);
        xp.set_read(super::SERVOS_ON, f64::NAN);
        assert!(!super::engage_climb(&xp, &[], &mut 0, 700.0, &mut last, &mut None).await);
    }

    /// LEMG 13 SF50 (1790607325, #160): the roll began with HDG and VS lit
    /// from the SR22 before it, and the 300 ft engage pressed nothing, so
    /// nothing flew the climb. Lit modes with the servos out now press the
    /// servos in; lit modes on the ground are taken off before the roll.
    #[tokio::test]
    async fn an_autopilot_left_on_is_never_taken_for_an_engage() {
        // The engage: HDG and VS lit, servos out. The old gate read the two
        // modes, found them on, and pressed nothing.
        let xp = crate::xplane::FakeXPlane::with_reads(&[
            (crate::datarefs::PAUSED, 0.0),
            (crate::datarefs::IS_IN_REPLAY, 0.0),
            (super::HDG_STATUS, 1.0),
            (super::VS_STATUS, 1.0),
            (super::SERVOS_ON, 0.0),
            (super::VS_DIAL, 1500.0),
        ]);
        let mut last = None;
        let _ = super::engage_climb(&xp, &[], &mut 0, 1500.0, &mut last, &mut None).await;
        assert!(xp.ran(super::CMD_SERVOS_ON), "the servos are pressed in");
        // A clean start (every other recorded takeoff: HDG and VS off) is
        // left alone on the ground.
        let clean = crate::xplane::FakeXPlane::with_reads(&[
            (super::SERVOS_ON, 0.0),
            (super::HDG_STATUS, 0.0),
            (super::VS_STATUS, 0.0),
        ]);
        assert_eq!(super::clear_inherited_autopilot(&clean).await, None);
        assert!(!clean.ran(super::CMD_AP_OFF));
        // The recorded roll: ap on, HDG and VS lit.
        let left_on = crate::xplane::FakeXPlane::with_reads(&[
            (super::SERVOS_ON, 1.0),
            (super::HDG_STATUS, 1.0),
            (super::VS_STATUS, 1.0),
        ]);
        left_on.set_on_command(super::CMD_AP_OFF, super::SERVOS_ON, 0.0);
        assert_eq!(super::clear_inherited_autopilot(&left_on).await, Some(true));
        assert!(left_on.ran(super::CMD_AP_OFF), "taken off before the roll");
        // Lit modes alone (a flight director left with modes) count too.
        assert!(super::ap_inherited(0.0, 1.0, 0.0));
        assert!(super::ap_inherited(0.0, 0.0, 1.0));
        assert!(!super::ap_inherited(0.0, 0.0, 0.0));
        // A failed read is no measurement: nothing is pressed.
        let unread = crate::xplane::FakeXPlane::with_reads(&[
            (super::HDG_STATUS, 1.0),
            (super::VS_STATUS, 1.0),
        ]);
        assert_eq!(super::clear_inherited_autopilot(&unread).await, None);
        assert!(!unread.ran(super::CMD_AP_OFF));
    }

    use super::*;
    use crate::autoland::project_latlon;

    /// The handoff line is the most important sentence this module speaks,
    /// and it was wrong in two ways at once on every circuit (2026-08-01,
    /// caught live in the cockpit): it said "your aircraft" while the circuit
    /// was taking control, and it announced "one thousand feet" at the
    /// circuit's 500 ft handoff — a number the pilot could see was wrong on
    /// the altimeter, in the same breath as the claim about who was flying.
    #[test]
    fn the_handoff_names_the_real_height() {
        assert_eq!(spoken_height_ft(500.0), "five hundred feet");
        assert_eq!(spoken_height_ft(1000.0), "one thousand feet");
        assert_eq!(spoken_height_ft(1500.0), "one thousand five hundred feet");
        // It is called on the tick that CROSSES the height, so it is never
        // exactly round: round to the hundred a pilot would say.
        assert_eq!(spoken_height_ft(1013.0), "one thousand feet");
        assert_eq!(spoken_height_ft(517.0), "five hundred feet");
        assert_eq!(spoken_height_ft(2050.0), "two thousand one hundred feet");
    }

    /// A 2,000 m runway at the equator, taking off due north from (0,0).
    fn rwy_north() -> Runway {
        Runway {
            icao: "TEST".into(),
            id: "36".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 2000.0,
            surface: 1,
            dist_nm: 0.0,
            elev_ft: 0.0,
        }
    }

    #[test]
    fn fits_the_runway_under_an_aligned_nose() {
        let cands = vec![rwy_north()];
        // On the numbers, pointing down the strip: fits, full length ahead.
        let (r, rem) = runway_fit(&cands, 0.0, 0.0, 2.0, 500.0).expect("lined up");
        assert_eq!(r.id, "36");
        assert!((rem - 2000.0).abs() < 30.0, "{rem}");
        // 300 m down the roll: fits with less left.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 300.0 / 1852.0);
        let (_, rem) = runway_fit(&cands, lat, lon, 358.0, 500.0).expect("mid-runway");
        assert!((rem - 1700.0).abs() < 40.0, "{rem}");
    }

    #[test]
    fn refuses_misaligned_offset_or_short() {
        let cands = vec![rwy_north()];
        // Facing 40 degrees off: not lined up.
        assert!(runway_fit(&cands, 0.0, 0.0, 40.0, 500.0).is_none());
        // At the hold-short point, 60 m right of the centreline: not on it.
        let (lat, lon) = project_latlon(0.0, 0.0, 90.0, 60.0 / 1852.0);
        assert!(runway_fit(&cands, lat, lon, 0.0, 500.0).is_none());
        // 1,700 m down a 2,000 m strip: not enough left to roll.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 1700.0 / 1852.0);
        assert!(runway_fit(&cands, lat, lon, 0.0, 500.0).is_none());
        // Backwards on the strip fits the RECIPROCAL end, never this one.
        assert!(runway_fit(&cands, 0.0, 0.0, 180.0, 500.0).is_none());
    }

    /// Stansted's real geometry (Global Airports apt.dat): the 1,048 m
    /// 04C/22C row is a collinear sub-segment of the 3,052 m 04/22 pavement.
    /// Cleared "22C" and lined up mid-strip, the fit names "22" (the short
    /// row fails the length gate for a jet's factored roll) and the string
    /// compare refused the takeoff — EGSS, 2026-07-29, three times.
    #[test]
    fn collinear_duplicate_designators_pass_the_crosscheck() {
        // Course 222.8 for the 22 ends: threshold at the NE, rolling SW.
        let cands = vec![
            Runway {
                icao: "EGSS".into(),
                id: "22C".into(),
                thr_lat: 51.8895556,
                thr_lon: 0.2416333,
                course_deg: 222.8,
                length_m: 1048.0,
                surface: 1,
                dist_nm: 0.0,
                elev_ft: 348.0,
            },
            Runway {
                icao: "EGSS".into(),
                id: "22".into(),
                thr_lat: 51.8951583,
                thr_lon: 0.2500444,
                course_deg: 222.8,
                length_m: 3052.0,
                surface: 1,
                dist_nm: 0.0,
                elev_ft: 348.0,
            },
        ];
        // The stop the line-up delivered: on the shared centreline, 190 m
        // down the 22C row, nose roughly down the strip.
        let (lat, lon) = (51.88799, 0.23930);
        // A factored roll longer than the 22C row's remainder: the fit
        // skips 22C and names the long row.
        let (rwy, rem) = runway_fit(&cands, lat, lon, 223.0, 900.0).expect("on the pavement");
        assert_eq!(rwy.id, "22", "the short row fails the length gate");
        assert!(rem > 1500.0, "{rem}");
        assert!(!runway_matches(&rwy.id, "22C"), "the strings alone refuse");
        // The escape hatch: the cleared designator fits where we stand.
        assert!(cleared_fits_here(&cands, "22C", lat, lon, 223.0));
        // The traps stay armed: the reciprocal fails on heading, a name
        // not under the wheels fails outright.
        assert!(!cleared_fits_here(&cands, "04C", lat, lon, 223.0));
        assert!(!cleared_fits_here(&cands, "27", lat, lon, 223.0));
        // A true parallel 200 m abeam fails on cross-track.
        let mut parallel = cands[0].clone();
        parallel.id = "22R".into();
        let (plat, plon) =
            project_latlon(parallel.thr_lat, parallel.thr_lon, 132.8, 200.0 / 1852.0);
        parallel.thr_lat = plat;
        parallel.thr_lon = plon;
        assert!(!cleared_fits_here(&[parallel], "22R", lat, lon, 223.0));
    }

    #[test]
    fn cleared_runway_crosscheck_is_forgiving_of_format_only() {
        assert!(runway_matches("09R", "09R"));
        assert!(runway_matches("9", "09"));
        assert!(runway_matches("27l", "27L"));
        assert!(
            runway_matches("", "27"),
            "no fitted ident is not a mismatch"
        );
        assert!(
            runway_matches("27", ""),
            "no cleared ident is not a mismatch"
        );
        assert!(
            !runway_matches("27", "09"),
            "the backwards line-up must be caught"
        );
        assert!(!runway_matches("27L", "27R"));
    }

    #[test]
    fn roll_target_cuts_toward_the_centreline_bounded() {
        // 10 m right of the line at taxi pace: cut left of runway heading,
        // bounded, and exactly as it always was at or below the reference.
        let tgt = roll_target_heading(0.0, 10.0, 0.0, XTK_GAIN_REF_KTS);
        assert!(tgt > 350.0 && tgt < 360.0, "cut left, got {tgt}");
        assert!(
            (360.0 - tgt - 10.0 * XTK_GAIN_DEG_PER_M).abs() < 1e-9,
            "the reference pace keeps the gain it was fitted with: {tgt}"
        );
        assert_eq!(
            roll_target_heading(0.0, 10.0, 0.0, 5.0),
            tgt,
            "below the reference nothing changes - the low-speed roll is tuned"
        );
        // 100 m off (should never happen): still clamped to the max cut.
        let tgt = roll_target_heading(0.0, 100.0, 0.0, XTK_GAIN_REF_KTS);
        assert!((360.0 - tgt - XTK_MAX_DEG).abs() < 0.01, "{tgt}");
    }

    /// The cut is sized for a CLOSURE TIME, so the lateral closure a given
    /// offset earns is the same at every speed. That is the whole fix: a
    /// fixed angle per metre winds the outer loop tighter the faster you go,
    /// until it sits on the inner loop's lag and the pair weaves.
    #[test]
    fn the_roll_cut_closes_in_the_same_time_at_any_speed() {
        let closure_m_s = |gs_kts: f64| {
            let tgt = roll_target_heading(0.0, 10.0, 0.0, gs_kts);
            let cut = (360.0 - tgt).to_radians();
            gs_kts * MS_TO_KTS.recip() * cut.sin()
        };
        let reference = closure_m_s(XTK_GAIN_REF_KTS);
        for gs in [60.0, 100.0, 140.0, 160.0] {
            let got = closure_m_s(gs);
            assert!(
                (got - reference).abs() < reference * 0.02,
                "at {gs} kt a 10 m offset must still close at {reference:.2} m/s, got {got:.2}"
            );
        }
        // And the cut itself shrinks, which is the same statement said the
        // other way round: 4 degrees at 150 kt was the weave.
        let fast = 360.0 - roll_target_heading(0.0, 10.0, 0.0, 150.0);
        assert!(fast < 1.5, "a 10 m offset at 150 kt is a small cut: {fast}");
    }

    /// LFPB 03 and EGLF 24, 2026-09-22: the same law at both ends of one
    /// sortie, weaving. The rollout wandered +3.1 / -5.7 m through four
    /// degrees of heading; the takeoff went +1.7, -4.6, +10.1 and LIFTED OFF
    /// ten metres right of the centreline.
    ///
    /// The plant is the cascade that did it, fitted to the LFPB tick log: a
    /// rudder that commands a yaw RATE through a 0.7 s lag, the loop's own
    /// half-second of transport delay, and the ground geometry closing the
    /// cross-track. Both laws fly it, because a test that only shows the fix
    /// working proves nothing about the bug.
    /// LIBP 04 (2026-09-26, 737): a steady yaw needing right rudder through
    /// the low-speed roll, a rudder whose bite grows with the square of the
    /// airspeed. The trim built up drifting left, the rudder came alive at
    /// 95 kt and the old trim swung the roll 9.5 m right by 124 kt. The roll
    /// now stays inside a few metres all the way to rotation.
    /// LEMG (2026-09-26): a 1 m/s drift read through ±0.4 m of position
    /// noise on a jittery tick. Raw, the rate swung the target heading by
    /// degrees tick to tick; smoothed, it is steady.
    #[test]
    fn the_cross_track_rate_is_smoothed_before_it_is_led() {
        let (mut raw_t, mut filt_t) = (Vec::new(), Vec::new());
        let mut f = XtkRate::default();
        let (mut x, mut prev) = (0.0_f64, None::<f64>);
        for i in 0..80 {
            let dt = if i % 3 == 0 { 0.15 } else { 0.25 };
            x += 1.0 * dt;
            let noise = if i % 2 == 0 { 0.4 } else { -0.4 };
            let seen = x + noise;
            let raw = prev.map_or(0.0, |p| (seen - p) / dt);
            prev = Some(seen);
            let smooth = f.update(raw, dt);
            if i > 20 {
                raw_t.push(roll_target_heading(40.0, seen, raw, 150.0));
                filt_t.push(roll_target_heading(40.0, seen, smooth, 150.0));
            }
        }
        let swing = |v: &[f64]| v.windows(2).map(|w| (w[1] - w[0]).abs()).fold(0.0, f64::max);
        assert!(swing(&filt_t) < swing(&raw_t) * 0.25, "{:.2} vs {:.2}", swing(&filt_t), swing(&raw_t));
        assert!(swing(&filt_t) < 0.3, "{:.2} degrees tick to tick", swing(&filt_t));
    }

    #[test]
    fn a_trim_learned_slow_does_not_swing_the_roll_fast() {
        const COURSE: f64 = 40.0;
        const TICK: f64 = 0.2;
        let mut law = RollSteer { jet: true, ..RollSteer::default() };
        let (mut psi, mut yaw, mut xtk, mut gs) = (COURSE, 0.0_f64, 0.5_f64, 0.0_f64);
        let mut prev: Option<f64> = None;
        let mut pipe = std::collections::VecDeque::from(vec![0.0; 1]);
        let (mut worst, mut worst_fast) = (0.0_f64, 0.0_f64);
        while gs < 135.0 {
            let rate = prev.map_or(0.0, |p| (xtk - p) / TICK);
            prev = Some(xtk);
            let target = roll_target_heading(COURSE, xtk, rate, gs);
            let err = crate::angles::signed_diff_deg(target, psi);
            pipe.push_back(law.step(err, TICK, gs, 1.0));
            let steer = pipe.pop_front().unwrap_or(0.0);
            for _ in 0..10 {
                let h = TICK / 10.0;
                // Nosewheel steering fades by 45 kt, BEFORE the rudder has
                // come alive: the dip the trim winds up in.
                let nose = 6.0 * (1.0 - gs / 45.0).max(0.0);
                let aero = 22.0 * (gs / 110.0).powi(2);
                // A steady left-turning disturbance (crosswind, thrust).
                let want = (nose + aero) * steer - 0.9;
                yaw += (want - yaw) * (h / 0.8).min(1.0);
                psi += yaw * h;
                xtk += gs / MS_TO_KTS * (psi - COURSE).to_radians().sin() * h;
                gs += 3.2 * h;
            }
            worst = worst.max(xtk.abs());
            if gs > 90.0 {
                worst_fast = worst_fast.max(xtk.abs());
            }
        }
        // The law before the scaling: 7.8 m, 6.0 m above 90 kt.
        assert!(worst < 5.0, "the roll wandered {worst:.1} m");
        assert!(worst_fast < 2.0, "above 90 kt it swung {worst_fast:.1} m");
    }

    /// The 737's rudder as the profile declares it (`roll_rudder:`).
    fn rudder_737() -> crate::aircraft::RollRudderProfile {
        let text = std::fs::read_to_string("profiles/boeing_737_800.yaml").unwrap();
        let p: crate::aircraft::AircraftProfile = serde_yaml::from_str(&text).unwrap();
        p.roll_rudder
            .and_then(|r| r.valid())
            .expect("the 737 declares its rudder")
    }

    /// One tick of a flown roll, as the blackbox's `roll` event carries it.
    struct RollTick {
        gs: f64,
        err: f64,
        xtk: f64,
        steer: f64,
        trim: f64,
    }

    /// A 737 takeoff roll through the plant fitted to both EGKK 26L rolls of
    /// 2026-09-27 (30 to 86 kt, three-second predictions of the heading
    /// 0.07 degrees rms): a unit of steer holds 6.45 deg/s of yaw at 60 kt,
    /// growing with v² to the 86 kt that fit reached and as v^1.25 past it
    /// (every 737 roll of the 27th and 28th refitted to Vr: never v², 12.6
    /// at 132 kt where v² reads 31), reached through a 0.47 s lag after 0.3 s of dead
    /// time (second order from steer to heading), plus the nosewheel at a
    /// walk; the crosswind from the left yaws it into the wind by
    /// 2.48 deg/s per 60 kt less 0.99 at the recorded 7.4 kt, in proportion.
    /// The loop ticks at the recorded 0.27 s with the real target law and
    /// cross-track filter. `kscale` makes the real rudder weaker or stronger
    /// than the one declared.
    fn fly_737_roll(
        law: RollSteer,
        xw_kt: f64,
        kscale: f64,
        xtk0: f64,
        psi_off: f64,
        accel_kts_s: f64,
        gs0: f64,
        vr_kts: f64,
    ) -> Vec<RollTick> {
        fly_737_roll_drifting(
            law,
            xw_kt,
            kscale,
            xtk0,
            psi_off,
            accel_kts_s,
            gs0,
            vr_kts,
            0.0,
            false,
        )
    }

    /// [`fly_737_roll`] with the tyres letting the track run `slip_deg`
    /// right of the heading (EGBB 33: 0.6 to 0.7 degrees), and the loop's
    /// [`DriftFilter`] fed into the target when `feed`.
    #[allow(clippy::too_many_arguments)]
    fn fly_737_roll_drifting(
        mut law: RollSteer,
        xw_kt: f64,
        kscale: f64,
        xtk0: f64,
        psi_off: f64,
        accel_kts_s: f64,
        gs0: f64,
        vr_kts: f64,
        slip_deg: f64,
        feed: bool,
    ) -> Vec<RollTick> {
        const COURSE: f64 = 257.6;
        const TICK: f64 = 0.27;
        const DEAD_S: f64 = 0.3;
        const LAG_S: f64 = 0.47;
        const H: f64 = 0.01;
        let (mut psi, mut yaw, mut xtk, mut gs) = (COURSE + psi_off, 0.0_f64, xtk0, gs0);
        let mut queue: std::collections::VecDeque<(f64, f64)> = Default::default();
        let mut applied = 0.0_f64;
        let mut prev_xtk: Option<f64> = None;
        let mut rate_filter = XtkRate::default();
        let mut drift_filter = DriftFilter::default();
        let mut t = 0.0_f64;
        let mut ticks = Vec::new();
        while gs < vr_kts {
            let raw = prev_xtk.map_or(0.0, |p| (xtk - p) / TICK);
            prev_xtk = Some(xtk);
            let rate = rate_filter.update(raw, TICK);
            let drift = if feed {
                drift_filter.update(psi - COURSE, rate, gs, TICK)
            } else {
                0.0
            };
            let target = roll_target_heading(COURSE, xtk, rate, gs) - drift;
            let err = crate::angles::signed_diff_deg(target, psi);
            let steer = law.step_with_yaw(err, TICK, gs, 1.0, Some(yaw));
            queue.push_back((t + DEAD_S, steer));
            ticks.push(RollTick {
                gs,
                err,
                xtk,
                steer,
                trim: law.trim(),
            });
            let next = t + TICK;
            while t < next - 1e-9 {
                while queue.front().is_some_and(|(at, _)| *at <= t) {
                    applied = queue.pop_front().map_or(applied, |(_, s)| s);
                }
                let nose = 0.23 * gs * (1.0 - gs / 40.0).max(0.0);
                let fitted = gs.min(86.0);
                let past = (gs.max(86.0) / 86.0).powf(1.25);
                let rudder = kscale * 6.45 * (fitted / 60.0).powi(2) * past + nose;
                let weathervane = xw_kt / 7.4 * (2.48 * gs / 60.0 - 0.99).max(0.0);
                yaw += (rudder * applied - weathervane - yaw) * H / LAG_S;
                psi += yaw * H;
                xtk += gs / MS_TO_KTS * (psi - COURSE + slip_deg).to_radians().sin() * H;
                gs += accel_kts_s * H;
                t += H;
            }
        }
        ticks
    }

    fn worst_xtk(ticks: &[RollTick], upto_kts: f64) -> f64 {
        ticks
            .iter()
            .filter(|k| k.gs >= 30.0 && k.gs <= upto_kts)
            .map(|k| k.xtk.abs())
            .fold(0.0, f64::max)
    }

    /// Tick-to-tick steer jumps past 0.08: the weave's signature.
    fn steer_swings(ticks: &[RollTick]) -> usize {
        ticks
            .windows(2)
            .filter(|w| w[0].gs >= 30.0 && (w[1].steer - w[0].steer).abs() > 0.08)
            .count()
    }

    fn default_737_law() -> RollSteer {
        RollSteer {
            jet: true,
            ..RollSteer::default()
        }
    }

    /// EGKK 26L, 2026-09-27, both rolls rejected `lateral_excursion`: an
    /// 8 kt crosswind from the left, the nose 5 degrees left of the target
    /// for ten seconds on 0.28 of steer, 10.3 m left at 87 kt. The default
    /// law must still do that through the fitted plant, or this test is not
    /// about EGKK any more; the law scheduled on the 737's declared rudder
    /// holds the centreline through rotation, its trim taking the crosswind
    /// within seconds of 40 kt.
    #[test]
    fn a_steady_crosswind_is_held_on_the_centreline() {
        // First roll: 7.5 kt of crosswind, on the centreline at the start.
        let old = fly_737_roll(default_737_law(), 7.5, 1.0, 0.08, 0.19, 6.2, 0.0, 145.0);
        let old_err = old
            .iter()
            .filter(|k| k.gs <= 87.0)
            .map(|k| k.err.abs())
            .fold(0.0, f64::max);
        assert!(
            old_err > 5.0,
            "the default law held only {old_err:.1} degrees off"
        );
        assert!(
            worst_xtk(&old, 87.0) > 10.0,
            "the default law reached only {:.1} m by 87 kt",
            worst_xtk(&old, 87.0)
        );
        let new = fly_737_roll(
            RollSteer::with_rudder(true, rudder_737()),
            7.5,
            1.0,
            0.08,
            0.19,
            6.2,
            0.0,
            145.0,
        );
        assert!(
            worst_xtk(&new, 145.0) < 5.0,
            "held within 5 m through rotation: {:.1} m",
            worst_xtk(&new, 145.0)
        );
        let new_err = new.iter().map(|k| k.err.abs()).fold(0.0, f64::max);
        assert!(
            new_err < 3.0,
            "the nose stays within 3 degrees: {new_err:.1}"
        );
        let trim_60 = new.iter().find(|k| k.gs >= 60.0).map_or(0.0, |k| k.trim);
        assert!(
            trim_60 > 0.1,
            "the crosswind's steady rudder is in the trim by 60 kt: {trim_60:.2}"
        );
        assert_eq!(
            steer_swings(&new),
            0,
            "and the rudder does not weave for it"
        );

        // Second roll: 6.4 kt, starting 3.1 m left with the nose 1.5 right.
        let old = fly_737_roll(default_737_law(), 6.4, 1.0, -3.1, 1.49, 6.1, 0.0, 145.0);
        let new = fly_737_roll(
            RollSteer::with_rudder(true, rudder_737()),
            6.4,
            1.0,
            -3.1,
            1.49,
            6.1,
            0.0,
            145.0,
        );
        assert!(worst_xtk(&old, 87.0) > 8.0, "{:.1}", worst_xtk(&old, 87.0));
        assert!(
            worst_xtk(&new, 145.0) < 5.0,
            "the second roll within 5 m too: {:.1}",
            worst_xtk(&new, 145.0)
        );
    }

    /// No wind (the LIBP case the trim scaling was for): the scheduled law
    /// recovers an offset without weaving and overshoots no more than the
    /// default law, with the real rudder as the plant has it, half of it or
    /// 1.75 times it; and a crosswind with a rudder not as declared is still
    /// held in half the default law's excursion or less. (Until 2026-09-28
    /// the strong case was 2.5 times a plant growing as v² to Vr: 100 deg/s
    /// per unit at 150 kt, eight times what either day's rolls measured. On
    /// the refitted plant 1.75 is still 3.4 to 5 times the measured rudder
    /// at Vr; at 2 an engage 2 m off and 1.5 degrees crooked at 80 kt swings
    /// twice, at 2.5 the loop rings near Vr.)
    #[test]
    fn the_scheduled_roll_law_does_not_weave() {
        // The far side of the centreline from where it started.
        let overshoot = |ticks: &[RollTick], xtk0: f64| {
            ticks
                .iter()
                .map(|k| -k.xtk * xtk0.signum())
                .fold(0.0, f64::max)
        };
        for kscale in [0.5, 1.0, 1.75] {
            for (gs0, xtk0, psi_off) in [(0.0, 3.0, 1.0), (40.0, 3.0, 2.0), (80.0, -2.0, -1.5)] {
                let old = fly_737_roll(
                    default_737_law(),
                    0.0,
                    kscale,
                    xtk0,
                    psi_off,
                    6.2,
                    gs0,
                    150.0,
                );
                let new = fly_737_roll(
                    RollSteer::with_rudder(true, rudder_737()),
                    0.0,
                    kscale,
                    xtk0,
                    psi_off,
                    6.2,
                    gs0,
                    150.0,
                );
                let (o, n) = (overshoot(&old, xtk0), overshoot(&new, xtk0));
                assert!(
                    n < 2.5 && n <= o + 1.0,
                    "x{kscale} from {gs0} kt: {n:.1} m through the centreline, default {o:.1}"
                );
                assert!(
                    steer_swings(&new) <= 1,
                    "x{kscale} from {gs0} kt: the rudder swung {} times",
                    steer_swings(&new)
                );
                assert!(
                    new.last().is_some_and(|k| k.xtk.abs() < 2.5),
                    "x{kscale} from {gs0} kt: rotates near the centreline"
                );
            }
            let old = fly_737_roll(default_737_law(), 7.4, kscale, 0.08, 0.2, 6.2, 0.0, 145.0);
            let new = fly_737_roll(
                RollSteer::with_rudder(true, rudder_737()),
                7.4,
                kscale,
                0.08,
                0.2,
                6.2,
                0.0,
                145.0,
            );
            assert!(
                worst_xtk(&new, 145.0) < worst_xtk(&old, 145.0) * 0.5,
                "x{kscale} in the crosswind: {:.1} against {:.1}",
                worst_xtk(&new, 145.0),
                worst_xtk(&old, 145.0)
            );
            assert_eq!(
                steer_swings(&new),
                0,
                "x{kscale} in the crosswind: the rudder weaves"
            );
        }
    }

    /// Every recorded 737 roll in the sorties (2026-09-26 and 27), replayed
    /// through the same plant with its own mean crosswind (the `state`
    /// wind across the runway), acceleration and start: the scheduled law
    /// is never worse than the default law it replaces, and every one stays
    /// within 5 m. The plant under-reads the gustier days (LIBP, LEMG flew
    /// 7 to 10 m against the plant's 2 to 6 under the default law), so this
    /// ranks the laws; it does not reproduce those rolls metre for metre.
    #[test]
    fn recorded_737_rolls_replay_no_worse() {
        // (sortie, crosswind from the left kt, accel kt/s, xtk0 m, psi0 deg, vr kt)
        const ROLLS: [(&str, f64, f64, f64, f64, f64); 10] = [
            ("1790426786 LIBP 04", 3.6, 6.2, 0.5, 0.0, 125.0),
            ("1790429680 LIBP 04", 3.9, 6.3, 0.6, 0.0, 150.0),
            ("1790441293 LEMG 13", 1.1, 6.6, 0.0, 0.0, 150.0),
            ("1790457940 LIBP 22", -0.7, 6.0, 0.0, -0.02, 150.0),
            ("1790466443 LIEA 02", 0.1, 6.0, 0.0, -0.40, 150.0),
            ("1790494983 LIEA 20", -0.1, 5.9, -0.1, -0.87, 150.0),
            ("1790504948 LEBL 24R", 0.1, 6.1, 0.0, 0.55, 150.0),
            ("1790510400 LFBD 29", 1.4, 5.9, 0.1, 0.09, 150.0),
            ("1790524628 EGKK 26L", 7.5, 6.2, 0.1, 0.19, 145.0),
            ("1790525414 EGKK 26L", 6.4, 6.1, -3.1, 1.49, 145.0),
        ];
        for (id, xw, accel, xtk0, psi0, vr) in ROLLS {
            let old = fly_737_roll(default_737_law(), xw, 1.0, xtk0, psi0, accel, 0.0, vr);
            let new = fly_737_roll(
                RollSteer::with_rudder(true, rudder_737()),
                xw,
                1.0,
                xtk0,
                psi0,
                accel,
                0.0,
                vr,
            );
            let (o, n) = (worst_xtk(&old, vr), worst_xtk(&new, vr));
            assert!(
                n <= o + 0.3,
                "{id}: {n:.1} m against the default law's {o:.1}"
            );
            assert!(n < 5.0, "{id}: {n:.1} m");
            assert_eq!(steer_swings(&new), 0, "{id}: the rudder weaves");
        }
    }

    /// EGBB 33 (1790534902, 2026-09-27): about 3 kt of crosswind from the
    /// left, the heading held within 0.3 degrees of its target, and the
    /// track 0.6 to 0.7 degrees right of the heading from 95 kt. Through the
    /// fitted plant with that drift, the cut alone lifts off metres right
    /// (the recorded roll: 5.7 m at 161 kt); fed the drift it holds the
    /// line, with no weave bought for it.
    #[test]
    fn a_steady_drift_is_fed_forward_not_held_off() {
        let fly = |feed: bool| {
            fly_737_roll_drifting(
                RollSteer::with_rudder(true, rudder_737()),
                3.1,
                1.0,
                0.13,
                0.0,
                5.85,
                0.0,
                161.0,
                0.65,
                feed,
            )
        };
        let (old, new) = (fly(false), fly(true));
        // At Vr (145 kt IAS, 147 over the ground): the recorded roll was
        // 3.7 m right there, before rotation raised the drift to 0.9.
        let at_vr = |t: &[RollTick]| t.iter().find(|k| k.gs >= 147.0).map_or(0.0, |k| k.xtk);
        println!(
            "cut alone: {:.1} m at Vr, {:.1} at liftoff; fed: worst {:.1} m",
            at_vr(&old),
            old.last().map_or(0.0, |k| k.xtk),
            worst_xtk(&new, 161.0)
        );
        assert!(
            at_vr(&old) > 3.0,
            "the cut alone must still lose the line, or this is not EGBB: {:.1} m",
            at_vr(&old)
        );
        assert!(
            worst_xtk(&new, 161.0) < 1.5,
            "fed the drift, within 1.5 m to liftoff: {:.1} m (cut alone {:.1} at Vr)",
            worst_xtk(&new, 161.0),
            at_vr(&old)
        );
        assert!(new.iter().map(|k| k.err.abs()).fold(0.0, f64::max) < 3.0);
        assert_eq!(steer_swings(&new), 0, "and the rudder does not weave for it");
    }

    /// With no drift to read, the feed changes nothing that matters: the
    /// offset recoveries, the crosswinds and every recorded 737 roll fly
    /// within a few tenths of the law without it, and never weave.
    #[test]
    fn the_drift_feed_is_inert_without_a_drift() {
        let lift = |t: &[RollTick]| t.last().map_or(0.0, |k| k.xtk);
        for kscale in [0.5, 1.0, 2.5] {
            for (xw, gs0, xtk0, psi_off) in [
                (0.0, 0.0, 3.0, 1.0),
                (0.0, 40.0, 3.0, 2.0),
                (0.0, 80.0, -2.0, -1.5),
                (7.4, 0.0, 0.08, 0.2),
            ] {
                let run = |feed: bool| {
                    fly_737_roll_drifting(
                        RollSteer::with_rudder(true, rudder_737()),
                        xw,
                        kscale,
                        xtk0,
                        psi_off,
                        6.2,
                        gs0,
                        150.0,
                        0.0,
                        feed,
                    )
                };
                let (a, b) = (run(false), run(true));
                let (wa, wb) = (worst_xtk(&a, 150.0), worst_xtk(&b, 150.0));
                assert!(
                    wb <= wa + 0.3,
                    "x{kscale} {xw} kt from {gs0} kt: {wb:.1} m against {wa:.1}"
                );
                assert!(
                    steer_swings(&b) <= steer_swings(&a).max(1),
                    "x{kscale} {xw} kt from {gs0} kt: the rudder swung {} times",
                    steer_swings(&b)
                );
                let (la, lb) = (lift(&a).abs(), lift(&b).abs());
                assert!(
                    lb < 2.5 || lb <= la + 0.3,
                    "x{kscale} {xw} kt from {gs0} kt: rotates {lb:.1} m off against {la:.1}"
                );
            }
        }
    }

    /// The filter over the recorded EGBB roll ticks (bench/sorties): the
    /// course is the logged target plus the cut it was built from. It reads
    /// the drift the roll carried (0.6 to 0.7 degrees from 95 kt) by
    /// rotation, and the position spikes in the ticks (1.3 m in one tick at
    /// 50 kt) never throw it past a degree.
    #[test]
    fn the_drift_is_read_off_the_recorded_egbb_roll() {
        const EGBB: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/sorties/1790534902-full_auto.trimmed.jsonl"));
        let ticks: Vec<serde_json::Value> = EGBB
            .lines()
            .filter_map(|l| serde_json::from_str::<serde_json::Value>(l).ok())
            .filter(|r| r["kind"] == "autotakeoff" && r["event"] == "roll")
            .collect();
        assert!(ticks.len() > 100, "{} roll ticks", ticks.len());
        let mut filter = DriftFilter::default();
        let (mut worst, mut at_vr) = (0.0_f64, None);
        for r in &ticks {
            let g = |k: &str| r[k].as_f64().unwrap_or(0.0);
            let (gs, xtk, rate) = (g("gs"), g("xtk_m"), g("xtk_rate"));
            let course = g("tgt") - roll_target_heading(0.0, xtk, rate, gs);
            let psi_off = crate::angles::signed_diff_deg(g("psi"), course);
            let d = filter.update(psi_off, rate, gs, g("dt"));
            worst = worst.max(d.abs());
            if at_vr.is_none() && g("ias") >= 145.0 {
                at_vr = Some(d);
            }
        }
        let at_vr = at_vr.expect("the roll reached Vr");
        println!("drift at Vr {at_vr:.2}, worst {worst:.2} degrees");
        assert!((0.45..=0.9).contains(&at_vr), "drift at Vr {at_vr:.2} degrees");
        assert!(worst < 1.0, "never past a degree: {worst:.2}");
    }

    /// Without a declared rudder nothing changes: the yaw rate is ignored
    /// and every other airframe flies the law it flew.
    #[test]
    fn an_undeclared_rudder_keeps_the_default_law() {
        let mut plain = default_737_law();
        let mut with_yaw = default_737_law();
        for (err, gs) in [(2.0, 30.0), (3.0, 45.0), (1.0, 70.0), (-2.0, 100.0)] {
            let a = plain.step(err, 0.27, gs, 1.0);
            let b = with_yaw.step_with_yaw(err, 0.27, gs, 1.0, Some(5.0));
            assert_eq!(a, b);
        }
        assert!(!RollSteer::default().scheduled());
        // A nonsense declaration is no declaration.
        let silly = crate::aircraft::RollRudderProfile {
            yaw_dps: 0.0,
            at_kts: 80.0,
            power_steer: None,
            power_steer_to_kts: None,
            power_steer_per_kftlb: None,
            yaw_power: None,
            close_per_s: None,
            trim_per_s: None,
        };
        assert!(!RollSteer::with_rudder(true, silly).scheduled());
    }

    /// The 737 rolls of 2026-09-28 (bench/takeoffs, every `roll` tick of
    /// EGKK 08R 1790601822 and the seven LEMG 13 circuits), one Vec per roll.
    fn rolls_0928() -> Vec<(String, Vec<serde_json::Value>)> {
        const TEXT: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/takeoffs/b738_rolls_2026-09-28.jsonl"));
        let mut rolls: Vec<(String, Vec<serde_json::Value>)> = Vec::new();
        for r in TEXT
            .lines()
            .filter_map(|l| serde_json::from_str::<serde_json::Value>(l).ok())
        {
            let id = r["sortie"].as_str().unwrap_or("").to_string();
            match rolls.last_mut() {
                Some((last, ticks)) if *last == id => ticks.push(r),
                _ => rolls.push((id, vec![r])),
            }
        }
        rolls
    }

    /// The 737's rudder on 2026-09-28, deg/s of yaw per unit of steer: the
    /// recorded steer through 0.3 s of dead time and a 0.47 s lag, fitted
    /// per speed band to three-second heading predictions of all eight
    /// rolls with a disturbance per roll (0.29 degrees rms).
    fn rudder_0928(gs: f64) -> f64 {
        const PTS: [(f64, f64); 7] = [
            (30.0, 3.5),
            (45.0, 3.5),
            (70.0, 4.0),
            (90.0, 5.4),
            (110.0, 5.8),
            (132.0, 8.5),
            (170.0, 10.5),
        ];
        if gs <= PTS[0].0 {
            return PTS[0].1;
        }
        for w in PTS.windows(2) {
            if gs <= w[1].0 {
                let f = (gs - w[0].0) / (w[1].0 - w[0].0);
                return w[0].1 + (w[1].1 - w[0].1) * f;
            }
        }
        PTS[PTS.len() - 1].1
    }

    /// The same fit over the 737 rolls of 2026-09-27 (EGKK 26L twice, EGBB,
    /// LEMG twice) to Vr: about v^1.25, 7.2 at 80 kt.
    fn rudder_0927(gs: f64) -> f64 {
        7.2 * (gs.max(0.0) / 80.0).powf(1.25)
    }

    /// One recorded roll replayed under a law: the aircraft is the recorded
    /// one plus the response to the DIFFERENCE between the law's steer and
    /// the recorded steer, through the fitted rudder, the loop's 0.3 s of
    /// dead time and the 0.47 s lag, and the ground geometry. Everything the
    /// recorded roll met (gusts, the tyres' slip, the cross-track spikes of
    /// the position read) is met again as it came. Returns per tick
    /// (gs, cross-track the law read, the TRUE cross-track (the median of
    /// five reads, which no single-tick spike passes), steer), and the
    /// largest difference between the replayed steer and the recorded one.
    fn replay_roll(
        ticks: &[serde_json::Value],
        law: RollSteer,
        cut: RollCut,
        rudder: &dyn Fn(f64) -> f64,
    ) -> (Vec<(f64, f64, f64, f64)>, f64) {
        replay_roll_braked(ticks, law, cut, rudder, None)
    }

    /// [`replay_roll`] on an airframe that brake-steers its roll
    /// ([`roll_brake_split_aligning`]): the replayed split less the split of
    /// the recorded steer yaws the aircraft through `brake` (deg/s per unit
    /// of right brake less left), on the same dead time and lag as the
    /// rudder. `None` is the rudder alone, as every earlier replay flew.
    fn replay_roll_braked(
        ticks: &[serde_json::Value],
        mut law: RollSteer,
        mut cut: RollCut,
        rudder: &dyn Fn(f64) -> f64,
        brake: Option<&dyn Fn(f64) -> f64>,
    ) -> (Vec<(f64, f64, f64, f64)>, f64) {
        const DEAD_S: f64 = 0.3;
        const LAG_S: f64 = 0.47;
        const H: f64 = 0.01;
        let g = |r: &serde_json::Value, k: &str| r[k].as_f64().unwrap_or(0.0);
        let n = ticks.len();
        let mut ts = vec![0.0; n];
        for k in 1..n {
            ts[k] = ts[k - 1] + g(&ticks[k], "dt");
        }
        let psi: Vec<f64> = ticks.iter().map(|r| g(r, "psi")).collect();
        // The sim's yaw rate the law damped on: the recorded `r` where the
        // tick carries it (the SR22's of 2026-09-29), else the heading's own
        // rate (the 737 bench carries none; before 2026-09-28 `r` was not
        // it, see the roll tick's comment).
        let yaw_rec: Vec<f64> = (0..n)
            .map(|k| {
                if let Some(r) = ticks[k]["r"].as_f64().filter(|r| r.is_finite()) {
                    return r;
                }
                let (a, b) = (k.saturating_sub(1), (k + 1).min(n - 1));
                let d = crate::angles::signed_diff_deg(psi[b], psi[a]);
                if ts[b] > ts[a] {
                    d / (ts[b] - ts[a])
                } else {
                    0.0
                }
            })
            .collect();
        let xtk_rec: Vec<f64> = ticks.iter().map(|r| g(r, "xtk_m")).collect();
        let truth: Vec<f64> = (0..n)
            .map(|k| {
                let mut w: Vec<f64> = xtk_rec[k.saturating_sub(2)..(k + 3).min(n)].to_vec();
                w.sort_by(|a, b| a.total_cmp(b));
                w[w.len() / 2]
            })
            .collect();
        // The runway course, from a tick of the law as flown.
        let r5 = &ticks[5];
        let course = g(r5, "tgt")
            + g(r5, "drift")
            + roll_cut_deg(g(r5, "xtk_m"), g(r5, "xtk_rate"), g(r5, "gs"));
        let (mut dyaw, mut dpsi, mut dx) = (0.0_f64, 0.0_f64, 0.0_f64);
        let mut queue: std::collections::VecDeque<(f64, f64, f64)> = Default::default();
        let mut applied = 0.0_f64;
        let mut applied_brake = 0.0_f64;
        let mut t = 0.0_f64;
        let mut out = Vec::with_capacity(n);
        let mut dev = 0.0_f64;
        for k in 0..n {
            let r = &ticks[k];
            let dt = if k == 0 { 0.2 } else { g(r, "dt") };
            if k > 0 {
                let (gs0, gs1) = (g(&ticks[k - 1], "gs"), g(r, "gs"));
                let end = ts[k];
                while t < end - 1e-9 {
                    while queue.front().is_some_and(|(at, _, _)| *at <= t) {
                        if let Some((_, s, b)) = queue.pop_front() {
                            applied = s;
                            applied_brake = b;
                        }
                    }
                    let gs = gs0 + (gs1 - gs0) * (1.0 - (end - t) / dt);
                    let braked = brake.map_or(0.0, |b| b(gs) * applied_brake);
                    dyaw += (rudder(gs) * applied + braked - dyaw) * H / LAG_S;
                    dpsi += dyaw * H;
                    dx += gs / MS_TO_KTS * dpsi.to_radians() * H;
                    t += H;
                }
                t = end;
            }
            let gs = g(r, "gs");
            let psi_off = crate::angles::signed_diff_deg(psi[k] + dpsi, course);
            let c = cut.update(xtk_rec[k] + dx, psi_off, gs, dt);
            let err = c.target_off_deg - psi_off;
            // The throttle as recorded (a propeller's power lead rides it);
            // the 737's ticks carry none and a jet takes no lead.
            let thr = r["thr"].as_f64().unwrap_or(1.0);
            let yaw = yaw_rec[k] + dyaw;
            // The torque as recorded, for a lead declared on it (the
            // Evolution's); a law without one ignores it.
            law.set_torque(r["trq_ftlb"].as_f64());
            let steer = law.step_with_yaw(err, dt, gs, thr, Some(yaw));
            dev = dev.max((steer - g(r, "steer")).abs());
            let dbrake = if brake.is_some() {
                let (bl, br) = roll_brake_split_aligning(steer, gs, err, yaw);
                let (bl0, br0) =
                    roll_brake_split_aligning(g(r, "steer"), gs, g(r, "err"), yaw_rec[k]);
                (br - bl) - (br0 - bl0)
            } else {
                0.0
            };
            queue.push_back((t + DEAD_S, steer - g(r, "steer"), dbrake));
            out.push((gs, c.xtk_m, truth[k] + dx, steer));
        }
        (out, dev)
    }

    /// Worst true offset from 30 kt, the true offset at liftoff (the last
    /// roll tick), and tick-to-tick steer jumps past 0.08 (the weave).
    fn roll_verdict(out: &[(f64, f64, f64, f64)]) -> (f64, f64, usize) {
        let worst = out
            .iter()
            .filter(|o| o.0 >= 30.0)
            .map(|o| o.2.abs())
            .fold(0.0, f64::max);
        let lift = out.last().map_or(0.0, |o| o.2);
        let swings = out
            .windows(2)
            .filter(|w| w[0].0 >= 30.0 && (w[1].3 - w[0].3).abs() > 0.08)
            .count();
        (worst, lift, swings)
    }

    /// The replay is the flight: the law as flown on 2026-09-28, replayed
    /// through it, writes the recorded steer tick for tick (so the replay
    /// carries everything the aircraft met), and the recorded offsets are
    /// the failures being fixed: EGKK 08R lifted off 5.4 m left (5.8 m worst,
    /// the takeoff FAIL), LEMG 1790591256 9.0 m worst and 5.9 at liftoff.
    #[test]
    fn the_recorded_737_rolls_replay_as_flown() {
        let rolls = rolls_0928();
        assert_eq!(rolls.len(), 8, "eight rolls on the bench");
        let mut failed = 0;
        for (id, ticks) in &rolls {
            let law = RollSteer::with_rudder(true, rudder_737()).as_flown();
            let (_, dev) = replay_roll(
                ticks,
                law,
                RollCut::for_law(&RollSteer::with_rudder(true, rudder_737())).as_flown(),
                &rudder_0928,
            );
            assert!(
                dev < 0.01,
                "{id}: the replayed law strays {dev:.4} from the recorded steer"
            );
            let xs: Vec<f64> = ticks
                .iter()
                .filter(|r| r["gs"].as_f64().unwrap_or(0.0) >= 30.0)
                .map(|r| r["xtk_m"].as_f64().unwrap_or(0.0))
                .collect();
            let worst = xs.iter().map(|x| x.abs()).fold(0.0, f64::max);
            let lift = xs.last().copied().unwrap_or(0.0);
            if worst > 3.0 || lift.abs() > 2.0 {
                failed += 1;
            }
            if id.starts_with("1790601822") {
                assert!(
                    (5.7..=5.9).contains(&worst) && (-5.5..=-5.3).contains(&lift),
                    "{worst} {lift}"
                );
            }
        }
        assert!(
            failed >= 4,
            "the rolls as flown: {failed} outside 3 m worst or 2 m at liftoff"
        );
    }

    /// The fix, replayed through every recorded roll of 2026-09-28: the
    /// heading loop scheduled on the declared rudder only up to the speed it
    /// was measured at, the cut held harder from 40 kt with a cross-track
    /// trim, and the cross-track read through a median of three. On the
    /// rudder fitted to these rolls and to the 27th's, each lifts off within
    /// 2 m of the centreline and never strays 3 m (the scorer's roll limit
    /// is 5), with no weave; on a rudder 0.6 or 3 times that day's it stays
    /// within 3.5 m and does not weave. And no roll is worse than it flew.
    #[test]
    fn the_737_roll_holds_the_centreline_through_liftoff() {
        let rolls = rolls_0928();
        let fly = |ticks: &[serde_json::Value], rudder: &dyn Fn(f64) -> f64| {
            let law = RollSteer::with_rudder(true, rudder_737());
            roll_verdict(
                &replay_roll(
                    ticks,
                    law,
                    RollCut::for_law(&RollSteer::with_rudder(true, rudder_737())),
                    rudder,
                )
                .0,
            )
        };
        for (id, ticks) in &rolls {
            let flown = {
                let law = RollSteer::with_rudder(true, rudder_737()).as_flown();
                roll_verdict(
                    &replay_roll(
                        ticks,
                        law,
                        RollCut::for_law(&RollSteer::with_rudder(true, rudder_737())).as_flown(),
                        &rudder_0928,
                    )
                    .0,
                )
            };
            for (name, rudder) in [
                ("09-28", &rudder_0928 as &dyn Fn(f64) -> f64),
                ("09-27", &rudder_0927),
            ] {
                let (worst, lift, swings) = fly(ticks, rudder);
                println!(
                    "{id} {name}: worst {worst:.1} m, liftoff {lift:+.1} m (flown {:.1}, {:+.1})",
                    flown.0, flown.1
                );
                assert!(
                    worst <= 3.0,
                    "{id} on the {name} rudder: {worst:.1} m worst"
                );
                assert!(
                    lift.abs() <= 2.0,
                    "{id} on the {name} rudder: {lift:+.1} m at liftoff"
                );
                assert_eq!(swings, 0, "{id} on the {name} rudder: the steer weaves");
                assert!(
                    worst <= flown.0 + 0.3,
                    "{id}: {worst:.1} m against {:.1} flown",
                    flown.0
                );
            }
            for scale in [0.6, 3.0] {
                let rudder = move |gs: f64| scale * rudder_0928(gs);
                let (worst, lift, swings) = fly(ticks, &rudder);
                assert!(
                    worst <= 3.5 && lift.abs() <= 2.5,
                    "{id} on x{scale} the rudder: {worst:.1} m worst, {lift:+.1} at liftoff"
                );
                assert_eq!(swings, 0, "{id} on x{scale} the rudder: the steer weaves");
            }
        }
    }

    /// The SR22's LEMG 13 roll of 2026-09-29 (sortie 1790677606-full_auto),
    /// every `roll` tick from brake release to the reject.
    fn sr22_roll_0929() -> Vec<serde_json::Value> {
        const TEXT: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/takeoffs/sr22_roll_lemg13_2026-09-29.jsonl"));
        TEXT.lines()
            .filter_map(|l| serde_json::from_str::<serde_json::Value>(l).ok())
            .collect()
    }

    /// The SR22's rudder fitted to that roll, deg/s of yaw per unit of steer:
    /// the sim's yaw rate `r` against the steer through the same 0.3 s of
    /// dead time and 0.47 s lag, over 15 kt to the reject (73 kt), with a
    /// standing rudder of its own (the propeller): 31 deg/s at 60 kt growing
    /// as v², 0.35 to 0.37 of steer standing from 30 kt (0.39 deg/s rms;
    /// v^1.5 fits 0.40).
    fn rudder_sr22_0929(gs: f64) -> f64 {
        31.0 * (gs.max(0.0) / 60.0).powi(2)
    }

    /// The SR22's rudder as the profile declares it (`roll_rudder:`).
    fn rudder_sr22() -> crate::aircraft::RollRudderProfile {
        let text = std::fs::read_to_string("profiles/cirrus_sr22.yaml").unwrap();
        let p: crate::aircraft::AircraftProfile = serde_yaml::from_str(&text).unwrap();
        p.roll_rudder
            .and_then(|r| r.valid())
            .expect("the SR22 declares its rudder")
    }

    /// The SR22's roll as flown: the default law replayed through the
    /// recorded roll writes the recorded steer tick for tick, and the offset
    /// is the failure being fixed (9.8 m left at the reject, 66 kt).
    #[test]
    fn the_sr22_roll_replays_as_flown() {
        let ticks = sr22_roll_0929();
        assert_eq!(ticks.len(), 63);
        let (out, dev) = replay_roll(
            &ticks,
            RollSteer::default(),
            RollCut::new(None),
            &rudder_sr22_0929,
        );
        assert!(
            dev < 0.01,
            "the replayed law strays {dev:.4} from the recorded steer"
        );
        let (worst, end, _) = roll_verdict(&out);
        assert!(worst > 9.0 && end < -9.0, "{worst:.1} {end:+.1}");
    }

    /// The fix: the SR22 declares its rudder and its propeller's standing
    /// rudder (`roll_rudder:` `power_steer`), led in with the throttle for
    /// the whole roll. Replayed through the recorded roll on the fitted
    /// rudder at half to three times, and on an aircraft needing 0.08 of
    /// steer less or more than it did that day, it never strays 3 m and
    /// does not weave; on the fitted rudder and the day's need, 1.5 m. The
    /// declared rudder alone (the generic power lead, on the closure and trim
    /// of that day) still strays over 4 m: the propeller's standing rudder is
    /// the part the default law lacked.
    #[test]
    fn the_sr22_roll_holds_the_centreline_on_its_propeller() {
        let ticks = sr22_roll_0929();
        let declared = rudder_sr22();
        assert_eq!(declared.power_steer, Some(0.35));
        let fly =
            |ticks: &[serde_json::Value], rd: crate::aircraft::RollRudderProfile, scale: f64| {
                let law = RollSteer::with_rudder(false, rd);
                let cut = RollCut::for_law(&law);
                let plant = move |gs: f64| scale * rudder_sr22_0929(gs);
                roll_verdict(&replay_roll(ticks, law, cut, &plant).0)
            };
        for need in [-0.08, 0.0, 0.08] {
            // The recorded steer relabelled: the aircraft needed `need` more.
            let shifted: Vec<serde_json::Value> = ticks
                .iter()
                .map(|r| {
                    let mut r = r.clone();
                    r["steer"] = serde_json::json!(r["steer"].as_f64().unwrap_or(0.0) - need);
                    r
                })
                .collect();
            for scale in [0.5, 1.0, 2.0, 3.0] {
                let (worst, end, swings) = fly(&shifted, declared, scale);
                println!("need {need:+} x{scale}: worst {worst:.2} m, end {end:+.2} m");
                assert!(worst <= 3.0, "need {need:+}, rudder x{scale}: {worst:.1} m");
                assert_eq!(
                    swings, 0,
                    "need {need:+}, rudder x{scale}: the steer weaves"
                );
            }
        }
        let (worst, _, _) = fly(&ticks, declared, 1.0);
        assert!(worst <= 1.5, "{worst:.2} m on the fitted rudder");
        // Against the declared rudder's law as it stood that day (the 737's
        // closure and trim): the faster closure since holds even the generic
        // lead's roll nearer.
        let generic = crate::aircraft::RollRudderProfile {
            power_steer: None,
            ..rudder_sr22_as_flown()
        };
        let (worst, _, _) = fly(&ticks, generic, 1.0);
        assert!(worst > 4.0, "{worst:.2} m without the propeller's rudder");
    }

    /// The SR22's second LEMG 13 roll of 2026-09-29 (sortie
    /// 1790682305-full_auto), flown on the declared rudder: every `roll`
    /// tick from brake release to wheels-off. The wheels left between the
    /// state records at 1790682400.9 (on the ground, 0.2 ft) and .401.9
    /// (0.5 ft); the ticks after that are the ~1.6 s the roll law kept
    /// steering in ground effect, and a replay of the tyres is no replay of
    /// them.
    fn sr22_rudder_roll_0929() -> Vec<serde_json::Value> {
        const TEXT: &str =
            include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/takeoffs/sr22_roll_lemg13_rudder_2026-09-29.jsonl"));
        TEXT.lines()
            .filter_map(|l| serde_json::from_str::<serde_json::Value>(l).ok())
            .filter(|r| r["t"].as_f64().is_some_and(|t| t <= SR22_RUDDER_WHEELS_OFF_T))
            .collect()
    }
    const SR22_RUDDER_WHEELS_OFF_T: f64 = 1790682401.0;

    /// The SR22's rudder refitted over BOTH rolls of 2026-09-29 (three-second
    /// heading predictions from the sim's `r`, 0.3 s of dead time, a standing
    /// need per roll and speed): 50 deg/s per unit at 60 kt growing as v
    /// (0.08 and 0.23 degrees rms against 0.11 and 0.28 for the declared 31
    /// as v^2; a v^2 rudder strong enough for the fast end misses the slow).
    fn rudder_sr22_0929_linear(gs: f64) -> f64 {
        50.0 * gs.max(0.0) / 60.0
    }

    /// The SR22's rudder as the profile declared it on 2026-09-29, the second
    /// roll's law: the standing rudder held to Vr, the 737's closure and trim.
    fn rudder_sr22_as_flown() -> crate::aircraft::RollRudderProfile {
        crate::aircraft::RollRudderProfile {
            power_steer_to_kts: Some(250.0),
            close_per_s: None,
            trim_per_s: None,
            ..rudder_sr22()
        }
    }

    /// The recorded steer relabelled: the aircraft needed `need` more.
    fn needing(ticks: &[serde_json::Value], need: f64) -> Vec<serde_json::Value> {
        ticks
            .iter()
            .map(|r| {
                let mut r = r.clone();
                r["steer"] = serde_json::json!(r["steer"].as_f64().unwrap_or(0.0) - need);
                r
            })
            .collect()
    }

    /// Worst true offset from 30 kt up to `to_kts` of groundspeed.
    fn worst_to(out: &[(f64, f64, f64, f64)], to_kts: f64) -> f64 {
        out.iter()
            .filter(|o| o.0 >= 30.0 && o.0 <= to_kts)
            .map(|o| o.2.abs())
            .fold(0.0, f64::max)
    }

    /// Rotation on the second roll: 73 kt indicated at 77.7 kt over the ground.
    const SR22_RUDDER_ROTATE_GS: f64 = 78.0;

    /// The second roll as flown: the declared rudder's law of 2026-09-29,
    /// replayed through it, writes the recorded steer tick for tick, and the
    /// offsets are the failure being fixed: 4.1 m left by rotation (the
    /// recorded 4.1 at 68 kt), then the swing right to 6.5 m (the median of
    /// the reads; 7.5 raw) at wheels-off, 93 kt.
    #[test]
    fn the_sr22_declared_roll_replays_as_flown() {
        let ticks = sr22_rudder_roll_0929();
        assert_eq!(ticks.len(), 84);
        let law = RollSteer::with_rudder(false, rudder_sr22_as_flown());
        let cut = RollCut::for_law(&law);
        let (out, dev) = replay_roll(&ticks, law, cut, &rudder_sr22_0929);
        assert!(
            dev < 0.01,
            "the replayed law strays {dev:.4} from the recorded steer"
        );
        let (worst, end, _) = roll_verdict(&out);
        let to_rotate = worst_to(&out, SR22_RUDDER_ROTATE_GS);
        println!(
            "as flown: {to_rotate:.2} m to rotation, {worst:.2} m to wheels-off, {end:+.2} m there"
        );
        assert!((3.9..=4.3).contains(&to_rotate), "{to_rotate:.2}");
        assert!(worst > 6.0 && end > 6.0, "{worst:.2} {end:+.2}");
    }

    /// The fix, replayed through both SR22 rolls of 2026-09-29 to wheels-off:
    /// the propeller's standing rudder believed only to the 73 kt it was
    /// measured at and falling as v^2 past it, the heading closed at 1.5 per
    /// second with the trim's corner at 0.45 (`roll_rudder:`
    /// `power_steer_to_kts`, `close_per_s`, `trim_per_s`). On either fit of the
    /// rudder at half to twice (the declared 31 as v^2, the refit 50 as v, to
    /// 1.6 times), and on an aircraft needing 0.08 of steer less or more than
    /// it did either day, no roll strays 3 m; on the fits and the day's need
    /// the second roll holds 2.5 m to wheels-off where it flew 6.5, the first
    /// 1.5 m. The standing rudder believed to Vr, or the 737's closure, still
    /// strays past 3.5 m on the second roll.
    #[test]
    fn the_sr22_roll_holds_the_centreline_past_its_measured_speed() {
        let declared = rudder_sr22();
        assert_eq!(declared.power_steer_to_kts, Some(73.0));
        let fly = |ticks: &[serde_json::Value],
                   rd: crate::aircraft::RollRudderProfile,
                   plant: &dyn Fn(f64) -> f64| {
            let law = RollSteer::with_rudder(false, rd);
            let cut = RollCut::for_law(&law);
            replay_roll(ticks, law, cut, plant).0
        };
        type Plant = Box<dyn Fn(f64) -> f64>;
        let plants: Vec<(String, Plant)> = [0.5, 1.0, 2.0]
            .into_iter()
            .map(|k| {
                let p: Plant = Box::new(move |gs: f64| k * rudder_sr22_0929(gs));
                (format!("31 v^2 x{k}"), p)
            })
            .chain([0.5, 1.0, 1.6].into_iter().map(|k| {
                let p: Plant = Box::new(move |gs: f64| k * rudder_sr22_0929_linear(gs));
                (format!("50 v x{k}"), p)
            }))
            .collect();
        for (name, ticks) in [("first", sr22_roll_0929()), ("second", sr22_rudder_roll_0929())] {
            for need in [-0.08, 0.0, 0.08] {
                let shifted = needing(&ticks, need);
                for (plant_name, plant) in &plants {
                    let out = fly(&shifted, declared, plant.as_ref());
                    let (worst, end, swings) = roll_verdict(&out);
                    println!(
                        "{name} need {need:+} {plant_name}: worst {worst:.2} m, {end:+.2} m at the end, {swings} swings"
                    );
                    assert!(
                        worst <= 3.0,
                        "{name}, need {need:+}, {plant_name}: {worst:.2} m"
                    );
                    assert!(
                        swings <= 1,
                        "{name}, need {need:+}, {plant_name}: the steer weaves"
                    );
                }
            }
        }
        let (worst, _, _) = roll_verdict(&fly(&sr22_roll_0929(), declared, &rudder_sr22_0929));
        assert!(worst <= 1.5, "the first roll: {worst:.2} m");
        for plant in [
            &rudder_sr22_0929 as &dyn Fn(f64) -> f64,
            &rudder_sr22_0929_linear,
        ] {
            let out = fly(&sr22_rudder_roll_0929(), declared, plant);
            let (worst, _, swings) = roll_verdict(&out);
            let to_rotate = worst_to(&out, SR22_RUDDER_ROTATE_GS);
            println!("the second roll: {to_rotate:.2} m to rotation, {worst:.2} m to wheels-off");
            assert!(worst <= 2.5, "the second roll: {worst:.2} m");
            assert!(to_rotate <= 2.5, "the second roll to rotation: {to_rotate:.2} m");
            assert_eq!(swings, 0);
        }
        for (why, rd) in [
            (
                "believed to Vr",
                crate::aircraft::RollRudderProfile {
                    power_steer_to_kts: Some(250.0),
                    ..declared
                },
            ),
            (
                "the 737's closure",
                crate::aircraft::RollRudderProfile {
                    close_per_s: None,
                    trim_per_s: None,
                    ..declared
                },
            ),
        ] {
            let (worst, _, _) =
                roll_verdict(&fly(&sr22_rudder_roll_0929(), rd, &rudder_sr22_0929));
            println!("{why}: {worst:.2} m");
            assert!(worst > 3.5, "{why}: {worst:.2} m");
        }
    }

    /// The Evolution's LILB 03 rolls, every `roll` tick from brake release
    /// to the reject, each with the law it flew:
    /// - 1790712974 (2026-09-29): started 24.6 degrees off before the
    ///   aligning pivot was fixed; the default law; 9 m left at 49 kt.
    /// - 1790724857 (2026-09-30): aligned at 1.9 degrees; the default law;
    ///   the torque ran to 1,735 ft-lb; 10 m left at 60 kt.
    /// - 1790736651 (2026-09-30, b6c24d5): the declared rudder of eb511f4 and
    ///   the torque limiter holding 1,093 to 1,513 ft-lb with the lever
    ///   cycling 0.55 to 0.76; from 50 kt the nose sat 4.4 to 7.9 degrees
    ///   left with the steer at 0.63 to 0.71; 9.5 m left at 78 kt.
    fn evolution_rolls() -> Vec<(&'static str, Vec<serde_json::Value>)> {
        let ld = |t: &str| -> Vec<serde_json::Value> {
            t.lines()
                .filter_map(|l| serde_json::from_str::<serde_json::Value>(l).ok())
                .collect()
        };
        vec![
            (
                "1790712974",
                ld(include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/takeoffs/evolution_roll_lilb03_1790712974.jsonl"))),
            ),
            (
                "1790724857",
                ld(include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/takeoffs/evolution_roll_lilb03_1790724857.jsonl"))),
            ),
            (
                "1790736651",
                ld(include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/takeoffs/evolution_roll_lilb03_1790736651.jsonl"))),
            ),
        ]
    }

    /// The Evolution's rudder refitted over all three rolls (three-second
    /// heading predictions from the sim's `r` through 0.3 s of dead time and
    /// the 0.47 s lag, from 12 kt, calm air, the brake split an input):
    /// 30 deg/s per unit at 60 kt as v^0.35 (1.47 degrees rms), or 33 as
    /// v^0.5 (1.52; 45 as v fits 2.19, 59 as v^2 3.72), the brake 12 deg/s
    /// per unit of split, and the standing need 0.51 of steer per 1,000
    /// ft-lb of torque (a constant need fits 3.54).
    fn rudder_evolution_flat(gs: f64) -> f64 {
        30.0 * (gs.max(0.0) / 60.0).powf(0.35)
    }
    fn rudder_evolution_sqrt(gs: f64) -> f64 {
        33.0 * (gs.max(0.0) / 60.0).sqrt()
    }
    const BRAKE_EVOLUTION: f64 = 12.0;

    /// The Evolution's rudder as the profile declares it (`roll_rudder:`).
    fn rudder_evolution() -> crate::aircraft::RollRudderProfile {
        let text = std::fs::read_to_string("profiles/lancair_evolution.yaml").unwrap();
        let p: crate::aircraft::AircraftProfile = serde_yaml::from_str(&text).unwrap();
        p.roll_rudder
            .and_then(|r| r.valid())
            .expect("the Evolution declares its rudder")
    }

    /// The Evolution's rudder as declared by eb511f4 (1790736651's law).
    fn rudder_evolution_eb511f4() -> crate::aircraft::RollRudderProfile {
        crate::aircraft::RollRudderProfile {
            yaw_dps: 30.0,
            at_kts: 60.0,
            yaw_power: None,
            power_steer: Some(0.6),
            power_steer_to_kts: Some(60.0),
            power_steer_per_kftlb: None,
            close_per_s: Some(1.0),
            trim_per_s: Some(0.3),
        }
    }

    /// Reversals of the steer from 30 kt: a jump past 0.08 against the last
    /// such jump. A steer walking up to its cap is not a weave.
    fn steer_reversals(out: &[(f64, f64, f64, f64)]) -> usize {
        let mut last = 0.0_f64;
        let mut n = 0;
        for w in out.windows(2).filter(|w| w[0].0 >= 30.0) {
            let d = w[1].3 - w[0].3;
            if d.abs() > 0.08 {
                if d * last < 0.0 {
                    n += 1;
                }
                last = d;
            }
        }
        n
    }

    /// One Evolution roll replayed under a law on a fitted rudder (scaled by
    /// `scale`) and brake.
    fn fly_evolution(
        ticks: &[serde_json::Value],
        law: RollSteer,
        cut: RollCut,
        rudder: fn(f64) -> f64,
        scale: f64,
    ) -> (Vec<(f64, f64, f64, f64)>, f64) {
        let plant = move |gs: f64| scale * rudder(gs);
        let braked = |_gs: f64| BRAKE_EVOLUTION;
        replay_roll_braked(ticks, law, cut, &plant, Some(&braked))
    }

    /// The law each roll flew.
    fn evolution_law_as_flown(name: &str) -> (RollSteer, RollCut) {
        if name == "1790736651" {
            let law = RollSteer::with_rudder(false, rudder_evolution_eb511f4());
            let cut = RollCut::for_law(&law);
            (law, cut)
        } else {
            (RollSteer::default(), RollCut::new(None))
        }
    }

    /// The Evolution's rolls as flown: each law replayed through its roll
    /// writes the recorded steer (within 0.02: the first tick's slew is
    /// replayed on the harness's 0.2 s, not the logged 0.15), and the
    /// offsets are the failures being fixed, 8.0 to 8.6 m left (the median
    /// of five reads). On the default law the steer at the reject was the
    /// authority floor, not the cap; on eb511f4's law it was 0.65 with the
    /// nose 7.6 degrees left, the lead on the LEVER faded past 60 kt.
    #[test]
    fn the_evolution_roll_replays_as_flown() {
        let lens = [32, 29, 38];
        for ((name, ticks), len) in evolution_rolls().into_iter().zip(lens) {
            assert_eq!(ticks.len(), len, "{name}");
            let (law, cut) = evolution_law_as_flown(name);
            let (out, dev) = fly_evolution(&ticks, law, cut, rudder_evolution_flat, 1.0);
            assert!(dev < 0.025, "{name}: the replayed law strays {dev:.4}");
            let (worst, end, _) = roll_verdict(&out);
            println!(
                "{name} as flown: {worst:.2} m, {end:+.2} m at the reject, {} reversals",
                steer_reversals(&out)
            );
            assert!(worst > 7.5 && end < -7.5, "{name}: {worst:.2} {end:+.2}");
            let last = ticks.last().unwrap();
            let steer = last["steer"].as_f64().unwrap();
            assert!(last["err"].as_f64().unwrap() > 7.0 && steer < 0.7, "{name}: {steer}");
        }
    }

    /// The torque lead: on the torque read, 0.51 per 1,000 ft-lb believed to
    /// 80 kt, where the lever lead gave 0.21 at 78 kt against 0.65 needed.
    #[test]
    fn the_evolution_leads_its_rudder_on_the_torque() {
        let declared = rudder_evolution();
        assert_eq!(declared.power_steer_per_kftlb, Some(0.51));
        let lead = |tq: Option<f64>, thr: f64, gs: f64| {
            let mut law = RollSteer::with_rudder(false, declared);
            law.set_torque(tq);
            // No error, no yaw: the first tick's want is the lead, slewed.
            let mut out = 0.0;
            for _ in 0..20 {
                out = law.step_with_yaw(0.0, 0.25, gs, thr, Some(0.0));
            }
            out
        };
        // 1,250 ft-lb at 78 kt: 0.64 whatever the lever.
        let a = lead(Some(1250.0), 0.55, 78.0);
        let b = lead(Some(1250.0), 0.76, 78.0);
        assert!((a - b).abs() < 1e-9, "{a} {b}");
        assert!((0.62..=0.66).contains(&a), "{a}");
        // A failed torque read leads on the lever, as power_steer does.
        let c = lead(None, 0.55, 50.0);
        assert!((c - 0.6 * 0.55).abs() < 0.02, "{c}");
        // Other airframes: no torque field, the torque is ignored.
        let mut sr22 = RollSteer::with_rudder(false, rudder_sr22());
        let mut sr22b = RollSteer::with_rudder(false, rudder_sr22());
        sr22.set_torque(Some(1500.0));
        for (err, gs) in [(2.0, 20.0), (1.0, 50.0), (-1.0, 80.0)] {
            assert_eq!(
                sr22.step_with_yaw(err, 0.25, gs, 1.0, Some(0.3)),
                sr22b.step_with_yaw(err, 0.25, gs, 1.0, Some(0.3))
            );
        }
        // The declared rudder grows as v^0.5, not v^2.
        let y = |gs| declared.yaw_dps_at(gs);
        assert!((y(20.0) / y(80.0) - 0.5).abs() < 1e-9);
    }

    /// The refit, replayed through all three rolls to the reject: the rudder
    /// declared at 32 deg/s at 80 kt growing as v^0.5 (`yaw_power`), the
    /// PT6's standing rudder led on the measured torque (0.51 per 1,000
    /// ft-lb, `power_steer_per_kftlb`) believed to 80 kt, closed at 1.5 per
    /// second with the trim's corner at 0.1 (the torque lead carries what
    /// the trim learnt slowly). On either fit, the rudder at two thirds to
    /// one and a half times (the three-roll fit bounds it tighter than the
    /// SR22's one roll did) and 0.05 of need either way:
    /// - 1790736651: 3.0 m on the fits (1.9 to 3.9 m on the neighbours),
    ///   ending within 0.9 m of the line, where it flew 8.6;
    /// - 1790724857: 3.7 to 3.8 m (4.8), where it flew 8.0, at torques the
    ///   limiter now holds off;
    /// - 1790712974: 3.6 to 3.7 m (4.9), already 2.7 m left by 30 kt from
    ///   its crooked start, where it flew 8.4.
    /// The steer reverses at most three times. eb511f4's law (the lever
    /// lead) replays 1790736651 as flown, and the torque lead on its v^2
    /// schedule and closure strays further on the fits.
    #[test]
    fn the_evolution_roll_holds_the_line_on_its_torque() {
        let declared = rudder_evolution();
        assert_eq!(declared.yaw_power, Some(0.5));
        assert_eq!(declared.power_steer_to_kts, Some(80.0));
        let fits: [(&str, fn(f64) -> f64); 2] = [
            ("30 v^0.35", rudder_evolution_flat),
            ("33 v^0.5", rudder_evolution_sqrt),
        ];
        let fly = |ticks: &[serde_json::Value],
                   rd: crate::aircraft::RollRudderProfile,
                   rudder: fn(f64) -> f64,
                   scale: f64| {
            let law = RollSteer::with_rudder(false, rd);
            let cut = RollCut::for_law(&law);
            fly_evolution(ticks, law, cut, rudder, scale).0
        };
        for ((name, ticks), (fit_max, all_max)) in
            evolution_rolls().into_iter().zip([(3.8, 5.0), (3.9, 4.9), (3.1, 4.0)])
        {
            let (law, cut) = evolution_law_as_flown(name);
            let flown = roll_verdict(&fly_evolution(&ticks, law, cut, rudder_evolution_flat, 1.0).0);
            for need in [-0.05, 0.0, 0.05] {
                let shifted = needing(&ticks, need);
                for (plant_name, rudder) in fits {
                    for scale in [0.67, 1.0, 1.5] {
                        let out = fly(&shifted, declared, rudder, scale);
                        let (worst, end, _) = roll_verdict(&out);
                        let rev = steer_reversals(&out);
                        println!(
                            "{name} need {need:+} {plant_name} x{scale}: worst {worst:.2} m, {end:+.2} m at the end, {rev} reversals"
                        );
                        assert!(
                            worst <= all_max,
                            "{name}, need {need:+}, {plant_name} x{scale}: {worst:.2} m"
                        );
                        assert!(
                            worst < flown.0 - 3.0,
                            "{name}: {worst:.2} m against {:.2} flown",
                            flown.0
                        );
                        assert!(rev <= 3, "{name}, {plant_name} x{scale}: the steer weaves");
                        if need == 0.0 && scale == 1.0 {
                            assert!(worst <= fit_max, "{name} on {plant_name}: {worst:.2} m");
                        }
                    }
                }
            }
        }
        // The torque lead alone, on eb511f4's schedule and closure.
        let torque_only = crate::aircraft::RollRudderProfile {
            power_steer_per_kftlb: Some(0.51),
            power_steer_to_kts: Some(80.0),
            ..rudder_evolution_eb511f4()
        };
        let today = &evolution_rolls()[2].1;
        let worst = |rd| {
            fits.iter()
                .map(|(_, r)| roll_verdict(&fly(today, rd, *r, 1.0)).0)
                .fold(0.0, f64::max)
        };
        let (with, without) = (worst(declared), worst(torque_only));
        println!("1790736651: {with:.2} m declared, {without:.2} m on eb511f4's schedule");
        assert!(without > with + 0.5, "{without:.2} against {with:.2}");
    }

    /// The cross-track read pairs two frames now and then (latitude and
    /// longitude are separate requests): on LEMG 13 up to 2.9 m for one tick
    /// above 120 kt. The median of three never passes a single-tick spike,
    /// follows a real movement a tick late, and the default law reads the
    /// raw value as it always has.
    #[test]
    fn a_single_tick_position_spike_never_reaches_the_cut() {
        let mut cut = RollCut::for_law(&RollSteer::with_rudder(true, rudder_737()));
        let mut plain = RollCut::new(None);
        let mut seen = Vec::new();
        for (k, x) in [0.5, 0.52, 0.54, 3.4, 0.58, 0.60, 0.62]
            .into_iter()
            .enumerate()
        {
            let c = cut.update(x, 0.0, 120.0, 0.22);
            let p = plain.update(x, 0.0, 120.0, 0.22);
            if k == 3 {
                assert_eq!(p.xtk_m, 3.4, "the default law reads the raw value");
            }
            seen.push(c.xtk_m);
        }
        assert!(
            seen.iter().all(|x| *x < 0.7),
            "a spike reached the cut: {seen:?}"
        );
        // A real step is followed, one tick late.
        let mut cut = RollCut::for_law(&RollSteer::with_rudder(true, rudder_737()));
        let steps: Vec<f64> = [0.0, 0.0, 0.0, 2.0, 2.0, 2.0]
            .into_iter()
            .map(|x| cut.update(x, 0.0, 120.0, 0.22).xtk_m)
            .collect();
        assert_eq!(steps[3], 0.0);
        assert_eq!(steps[4], 2.0);
    }

    /// Below 40 kt the scheduled roll's cut is the one it flew on what it
    /// reads: no trim, no extra gain; the heading law is unchanged below
    /// 40 kt, and past the declared speed its gain holds.
    #[test]
    fn the_slow_roll_is_the_one_it_was() {
        let mut new = RollCut::for_law(&RollSteer::with_rudder(true, rudder_737()));
        for (x, gs) in [
            (0.3, 5.0),
            (0.6, 12.0),
            (1.0, 20.0),
            (1.4, 30.0),
            (1.6, 39.0),
        ] {
            for _ in 0..3 {
                let c = new.update(x, 0.4, gs, 0.25);
                assert_eq!(
                    c.cut_deg,
                    roll_cut_deg(c.xtk_m, c.rate_m_s, gs),
                    "{x} m at {gs} kt"
                );
                assert_eq!(c.cut_i_deg, 0.0);
            }
        }
        // From 80 kt the cut is twice the closure-time cut on the same reads.
        let mut fast = RollCut::for_law(&RollSteer::with_rudder(true, rudder_737()));
        for _ in 0..3 {
            let c = fast.update(1.0, 0.0, 120.0, 0.25);
            assert!((c.cut_deg - 2.0 * roll_cut_deg(c.xtk_m, c.rate_m_s, 120.0)).abs() < 1e-12);
        }
        let mut a = RollSteer::with_rudder(true, rudder_737());
        let mut b = RollSteer::with_rudder(true, rudder_737()).as_flown();
        for (err, gs) in [(2.0, 10.0), (1.0, 25.0), (-1.5, 35.0), (0.5, 39.0)] {
            assert_eq!(
                a.step_with_yaw(err, 0.25, gs, 1.0, Some(0.2)),
                b.step_with_yaw(err, 0.25, gs, 1.0, Some(0.2))
            );
        }
        // Past the declared speed the gain holds instead of falling as 1/v².
        let kp = |gs: f64| {
            let mut l = RollSteer::with_rudder(true, rudder_737());
            l.step_with_yaw(1.0, 0.25, gs, 1.0, Some(0.0))
        };
        assert!(
            (kp(140.0) - kp(80.0)).abs() < 1e-9,
            "{} {}",
            kp(140.0),
            kp(80.0)
        );
    }

    #[test]
    fn the_roll_does_not_weave_down_the_centreline() {
        /// One run. `scaled` picks the law: false is the old fixed angle per
        /// metre, true is [`roll_target_heading`] as it now stands.
        fn fly_roll(
            scaled: bool,
            gs0: f64,
            xtk0: f64,
            psi0: f64,
            decel: f64,
            secs: f64,
        ) -> Vec<f64> {
            const COURSE: f64 = 24.76;
            const TICK: f64 = 0.42;
            const DELAY_TICKS: usize = 1;
            let mut law = RollSteer::default();
            let (mut psi, mut yaw, mut xtk, mut gs) = (psi0, 0.0_f64, xtk0, gs0);
            let mut pipe = std::collections::VecDeque::from(vec![0.0; DELAY_TICKS]);
            let mut prev: Option<f64> = None;
            let mut track = Vec::new();
            let mut t = 0.0;
            while t < secs {
                let rate = prev.map_or(0.0, |p| (xtk - p) / TICK);
                prev = Some(xtk);
                let target = if scaled {
                    roll_target_heading(COURSE, xtk, rate, gs)
                } else {
                    // The law as it was: a fixed cut per metre, no lead.
                    let corr = (xtk * XTK_GAIN_DEG_PER_M).clamp(-XTK_MAX_DEG, XTK_MAX_DEG);
                    (COURSE - corr).rem_euclid(360.0)
                };
                let err = crate::angles::signed_diff_deg(target, psi);
                pipe.push_back(law.step(err, TICK, gs, 0.0));
                let steer = pipe.pop_front().unwrap_or(0.0);
                // Sub-steps: the airframe does not tick at the loop's rate.
                for _ in 0..12 {
                    let h = TICK / 12.0;
                    // Yaw rate follows the rudder through a 0.7 s lag, with
                    // the surface biting harder the faster the air over it.
                    let want = 18.0 * steer * (gs / 120.0).min(1.0);
                    yaw += (want - yaw) * (h / 0.7).min(1.0);
                    psi += yaw * h;
                    xtk += gs / MS_TO_KTS * (psi - COURSE).to_radians().sin() * h;
                    gs = (gs - decel * h).max(8.0);
                }
                track.push(xtk);
                t += TICK;
            }
            track
        }
        let span = |t: &[f64]| {
            t.iter().cloned().fold(f64::MIN, f64::max) - t.iter().cloned().fold(f64::MAX, f64::min)
        };

        // The landing rollout: 140 kt, four metres left, decelerating hard.
        let old = fly_roll(false, 140.0, -4.2, 24.3, 4.0, 20.0);
        let new = fly_roll(true, 140.0, -4.2, 24.3, 4.0, 20.0);
        assert!(
            span(&old) > 9.0,
            "the plant must still weave under the old law, or this test is \
             not about LFPB any more: span {:.1} m",
            span(&old)
        );
        assert!(
            span(&new) < span(&old) * 0.6,
            "the scaled cut takes the weave out: {:.1} m against {:.1}",
            span(&new),
            span(&old)
        );
        assert!(
            new.last().is_some_and(|x| x.abs() < 2.0),
            "and it ends ON the centreline, not merely swinging through it: {:?}",
            new.last()
        );

        // The takeoff roll: accelerating, so the gain the old law held all
        // the way to Vr is the one that grew a 10 m offset by liftoff.
        let old = fly_roll(false, 41.0, 1.74, 24.63, -2.9, 21.0);
        let new = fly_roll(true, 41.0, 1.74, 24.63, -2.9, 21.0);
        assert!(
            new.last().unwrap().abs() < old.last().unwrap().abs(),
            "the wheels leave the ground nearer the centreline: {:.1} against {:.1}",
            new.last().unwrap(),
            old.last().unwrap()
        );
        assert!(
            span(&new) <= span(&old),
            "and no new excursion is bought for it: {:.1} against {:.1}",
            span(&new),
            span(&old)
        );
    }

    /// The lead looks at where the offset WILL be. Drifting AWAY earns more
    /// cut than the offset alone; already closing fast earns less, which is
    /// what stops the overshoot.
    #[test]
    fn the_roll_cut_leads_the_cross_track() {
        let gs = 140.0;
        let held = 360.0 - roll_target_heading(0.0, 5.0, 0.0, gs);
        let diverging = 360.0 - roll_target_heading(0.0, 5.0, 1.0, gs);
        let closing = roll_target_heading(0.0, 5.0, -3.0, gs);
        assert!(diverging > held, "drifting off earns more cut: {diverging}");
        assert!(
            closing < 1.0 || closing > 359.0,
            "closing fast enough, the cut is already given back: {closing}"
        );
        // Nothing the sim can hand us turns the target into a NaN.
        for bad in [f64::NAN, f64::INFINITY] {
            for tgt in [
                roll_target_heading(0.0, bad, 0.0, gs),
                roll_target_heading(0.0, 5.0, bad, gs),
                roll_target_heading(0.0, 5.0, 0.0, bad),
            ] {
                assert!(tgt.is_finite(), "a bad read must not steer by NaN");
            }
        }
    }

    #[test]
    fn the_integrator_trims_out_a_standing_yaw_moment() {
        // The second sortie's failure shape: a constant-ish 18 degree error
        // that pure P needed to hold ANY rudder. Feed the law a standing
        // error; the trim must grow so the total command keeps rising even
        // though P alone has plateaued — the rudder that finally straightens
        // the nose.
        let mut law = RollSteer::default();
        let first = law.step(18.0, 0.15, 25.0, 0.0);
        let mut last = first;
        for _ in 0..40 {
            last = law.step(18.0, 0.15, 25.0, 0.0);
        }
        assert!(last > first, "trim accumulates: {first} -> {last}");
        assert!(
            law.trim() > 0.2,
            "learned standing rudder, got {}",
            law.trim()
        );
        // And with the error finally at zero the trim STILL holds rudder —
        // the whole point: straight nose, rudder in.
        let centered = law.step(0.0, 0.15, 40.0, 0.0);
        assert!(
            centered > 0.15,
            "rudder held with the nose straight: {centered}"
        );
    }

    #[test]
    fn a_transient_swing_cannot_wind_the_trim() {
        // Sortie four: a 39 degree recovery overshoot wound the trim to its
        // cap in two seconds, and the stale trim fueled the NEXT overshoot.
        // The error is clamped in the update: a huge swing learns no faster
        // than a modest standing error.
        let mut big = RollSteer::default();
        let mut small = RollSteer::default();
        for _ in 0..10 {
            big.step(39.0, 0.15, 30.0, 0.0);
            small.step(STEER_TRIM_ERR_CLAMP_DEG, 0.15, 30.0, 0.0);
        }
        assert!(
            (big.trim() - small.trim()).abs() < 1e-9,
            "{} vs {}",
            big.trim(),
            small.trim()
        );
        assert!(
            big.trim() < 0.1,
            "two seconds of swing stays a nudge: {}",
            big.trim()
        );
    }

    /// The overspeed guard must OWN the airborne lever. It used to share it
    /// with an unconditional 1.0 commanded every airborne tick, so the
    /// guard's reduced write survived one tick at a time (the SF50's Vmo
    /// overspeed came straight back), and the pilot-override detector read
    /// the guard's own deep cut as a hand on the lever. March the loop's
    /// real per-tick arithmetic through one writer: the lever settles ON the
    /// guard value and stays, and the readback of our own write can never
    /// trip the override threshold.
    #[test]
    fn the_overspeed_guard_owns_the_airborne_lever() {
        let vne = Some(250.0); // SF50 Vmo, the measured live case
        let mut last_thr_cmd = 1.0_f64; // full power off the runway
        for _ in 0..40 {
            let want = tick_throttle_want(true, 60.0, 0.0, 140.0, 270.0, vne);
            let thr_cmd = throttle_slew(last_thr_cmd, want, 0.4);
            // The sim reads back what this loop wrote last tick; the
            // override detector (thr_now < last_cmd - 0.25) must never see
            // the guard's own command as a pilot pull.
            let thr_now = last_thr_cmd;
            assert!(
                thr_now >= last_thr_cmd - 0.25,
                "the guard's own write tripped the override detector"
            );
            last_thr_cmd = thr_cmd;
        }
        let guard = climb_throttle(270.0, vne);
        assert!(guard < 0.75, "the live case cuts deep: {guard}");
        assert!(
            (last_thr_cmd - guard).abs() < 1e-9,
            "the lever settles ON the guard and stays: {last_thr_cmd} vs {guard}"
        );
        // Ground ticks are untouched: the roll's ramp and swinging-nose hold
        // still own the lever there.
        assert_eq!(tick_throttle_want(false, 10.0, 2.0, 8.0, 0.0, vne), 1.0);
        assert_eq!(
            tick_throttle_want(false, 10.0, 14.0, 8.0, 0.0, vne),
            THROTTLE_HOLD_RATIO
        );
        // No declared Vne keeps the full-power climb every airframe flew.
        assert_eq!(tick_throttle_want(true, 60.0, 0.0, 140.0, 200.0, None), 1.0);
    }

    #[test]
    fn throttle_advances_gradually_and_releases_instantly() {
        // The swinging-nose hold letting go must WALK the lever up, not jump
        // half-to-full in a tick (sortie four's release kicked 19 degrees of
        // yaw at 12 knots).
        let stepped = throttle_slew(0.5, 1.0, 0.15);
        assert!(stepped < 0.55, "one tick walks, never jumps: {stepped}");
        let mut thr = 0.5;
        for _ in 0..20 {
            thr = throttle_slew(thr, 1.0, 0.15);
        }
        assert!((thr - 1.0).abs() < 1e-9, "full power arrives: {thr}");
        // A pull-back (the hold re-engaging, a pilot hand) is honored NOW.
        assert_eq!(throttle_slew(1.0, 0.5, 0.15), 0.5);
    }

    #[test]
    fn roll_law_damps_and_never_slams() {
        let mut law = RollSteer::default();
        // A swing home (error collapsing fast) commands LESS than the same
        // error steady — the lead term kills the counter-steer oscillation.
        let mut steady = RollSteer::default();
        steady.step(12.0, 0.15, 20.0, 0.0);
        let steady_out = steady.step(12.0, 0.15, 20.0, 0.0);
        law.step(20.0, 0.15, 20.0, 0.0);
        let collapsing = law.step(12.0, 0.15, 20.0, 0.0);
        assert!(collapsing < steady_out, "{collapsing} vs {steady_out}");
        // From rest, a huge demanded deflection arrives in slew steps —
        // dt-scaled, so a slow tick sweeps proportionally more, never a
        // fixed per-tick crumb (the fixed 0.25 at half-second ticks WAS
        // sortie four's weave).
        let mut fresh = RollSteer::default();
        assert!(fresh.step(90.0, 0.15, 5.0, 0.0).abs() <= STEER_SLEW_PER_S * 0.15 + 1e-9);
        let mut slow_tick = RollSteer::default();
        let one = slow_tick.step(90.0, 0.5, 20.0, 0.0).abs();
        assert!(one > 0.5, "a half-second tick sweeps half the range: {one}");
        // Output is always inside the cap.
        let mut sat = RollSteer::default();
        for _ in 0..30 {
            assert!(sat.step(90.0, 0.15, 5.0, 0.0).abs() <= MAX_ROLL_STEER + 1e-9);
        }
        // Authority at speed keeps a real floor: the same error at 50 kt
        // still commands most of what it does at 15 kt (the 0.21 starvation
        // was the second sortie's runway excursion).
        let mut slow = RollSteer::default();
        let mut fast = RollSteer::default();
        let s = (0..3)
            .map(|_| slow.step(10.0, 0.15, 10.0, 0.0))
            .last()
            .unwrap();
        let f = (0..3)
            .map(|_| fast.step(10.0, 0.15, 50.0, 0.0))
            .last()
            .unwrap();
        assert!(f >= s * STEER_AUTH_FLOOR - 1e-9, "floor holds: {f} vs {s}");
    }

    #[test]
    fn parked_under_power_the_wheel_barely_moves() {
        // Sortie three: 0.85 of deflection at brake release spot-turned
        // the aircraft at 25 deg/s (prop wash makes the tail hyper-
        // effective under power at low speed). At a standstill even a
        // huge error commands only the low-speed cap, and the trim stays
        // frozen so the swing can't wind it up.
        let mut law = RollSteer::default();
        let mut out = 0.0;
        for _ in 0..10 {
            out = law.step(-60.0, 0.15, 0.0, 0.0);
        }
        assert!(out.abs() <= LOW_SPEED_STEER_CAP + 1e-9, "{out}");
        assert_eq!(law.trim(), 0.0, "no trim learning parked");
        // The cap grows with the roll: by 15 knots the full range is back.
        let mut rolling = RollSteer::default();
        let mut r = 0.0;
        for _ in 0..10 {
            r = rolling.step(-60.0, 0.15, 20.0, 0.0);
        }
        assert!(r.abs() > 0.5, "full authority once rolling, got {r}");
    }

    #[test]
    fn rudder_leads_in_with_the_power_and_throttle_waits_for_a_straight_nose() {
        // Zero error, power coming up: the foot moves with the hand — the
        // law holds right rudder proportional to throttle, no error needed.
        let mut law = RollSteer::default();
        let with_power = (0..5)
            .map(|_| law.step(0.0, 0.15, 5.0, 1.0))
            .last()
            .unwrap();
        assert!(
            with_power > 0.1,
            "lead rides the throttle, got {with_power}"
        );
        let mut idle = RollSteer::default();
        let no_power = (0..5)
            .map(|_| idle.step(0.0, 0.15, 5.0, 0.0))
            .last()
            .unwrap();
        assert!(no_power.abs() < 0.01, "no power, no lead: {no_power}");
        // The lead fades as airflow straightens the moment out.
        assert!(power_lead(1.0, 40.0) < power_lead(1.0, 0.0));
        assert!(
            power_lead(1.0, 40.0) > 0.05,
            "never fades to nothing on the roll"
        );
        // Throttle: swinging nose at low speed holds half power...
        assert_eq!(throttle_cmd(10.0, 14.0, 8.0), THROTTLE_HOLD_RATIO);
        // ...straight nose rides the ramp to full...
        assert_eq!(throttle_cmd(10.0, 2.0, 8.0), 1.0);
        // ...and at speed a gust yaw never chops the power.
        assert_eq!(throttle_cmd(10.0, 14.0, 40.0), 1.0);
        // The ramp itself still applies early with the nose straight.
        assert!(throttle_cmd(0.0, 0.0, 0.0) < 0.4);
    }

    #[test]
    fn rotation_waits_for_a_straight_nose_unless_the_runway_ends() {
        assert!(rotate_ok(3.0, 900.0), "straight and long: rotate");
        assert!(
            !rotate_ok(25.0, 900.0),
            "yawed with room: keep straightening"
        );
        assert!(rotate_ok(25.0, 300.0), "yawed but running out: fly");
    }

    /// A rotation is a RATE. Luton 2026-07-28: the old law saw the whole 8° as
    /// error on the first tick, wound the integrator to its cap pulling for it,
    /// and the nose arrived with enough momentum to overshoot to 10.6° at 21 ft
    /// — a tail strike on a fuselage that runs out around 11, then a ±2.5°
    /// oscillation for the rest of the initial climb.
    #[test]
    fn the_rotation_walks_the_nose_up_instead_of_snatching_it() {
        let mut law = RotatePitch::with_authority(None, PITCH_MIN);
        // First tick from a nose-on-the-ground attitude: the commanded target
        // starts THERE, so the error is small and the elevator stays gentle.
        let first = law.step(0.0, None, 0.15);
        // "Gentle" is a small FRACTION of the available pull, not a number a
        // pure integrator produced for free by starting at zero. The law is
        // P+I now, so the first tick is deliberately non-zero — that is the
        // proportional path doing its job. What must never happen is the
        // snatch: jumping toward the cap.
        assert!(first > 0.0, "it must start pulling: {first}");
        assert!(
            first < 0.2 * PITCH_MAX,
            "gentle to start, not a snatch: {first} of {PITCH_MAX}"
        );
        // The real anti-snatch guarantee, unchanged: the COMMANDED attitude
        // starts at the nose and is walked up, so the law never sees the whole
        // 8 degrees as error on tick one.
        assert!(law.target_deg() < 1.0, "target starts at the nose");
        // Two seconds later the commanded attitude has walked up about
        // 2.5°/s, and no further than the 8° target.
        let mut ramp = RotatePitch::with_authority(None, PITCH_MIN);
        for _ in 0..14 {
            ramp.step(0.0, None, 0.15);
        }
        let t = ramp.target_deg();
        assert!((t - 2.1 * ROTATE_RATE_DEG_S).abs() < 1.5, "ramping: {t}");
        for _ in 0..60 {
            ramp.step(0.0, None, 0.15);
        }
        assert!(
            (ramp.target_deg() - ROTATE_PITCH_DEG).abs() < 1e-6,
            "and stops at the target: {}",
            ramp.target_deg()
        );
        // A nose FOLLOWING the ramp is doing what was asked, so the elevator
        // must not keep winding: the rate damping measures the rate ERROR, not
        // the rotation itself.
        let mut good = RotatePitch::with_authority(None, PITCH_MIN);
        let mut theta = 0.0;
        let mut peak: f64 = 0.0;
        for _ in 0..60 {
            let out = good.step(theta, None, 0.15);
            peak = peak.max(out);
            theta = (theta + ROTATE_RATE_DEG_S * 0.15).min(ROTATE_PITCH_DEG);
        }
        assert!(
            peak < 0.25,
            "a rotation on rate needs no big pull: peak {peak}"
        );
    }

    /// A long fuselage gets a ceiling: the profile's tail-strike attitude, less
    /// a margin, bounds what the rotation may ever ask for.
    #[test]
    fn a_tail_strike_limit_caps_the_commanded_attitude() {
        // 9.5° geometry, 2° margin → 7.5°, below the law's own 8° target.
        let mut md = RotatePitch::with_authority(Some(9.5), PITCH_MIN);
        for _ in 0..80 {
            md.step(0.0, None, 0.15);
        }
        assert!(
            (md.target_deg() - 7.5).abs() < 1e-6,
            "capped by the airframe: {}",
            md.target_deg()
        );
        // Past the ceiling, the law pushes back rather than holding the pull.
        let mut over = RotatePitch::with_authority(Some(9.5), PITCH_MIN);
        for _ in 0..40 {
            over.step(0.0, None, 0.15);
        }
        let pulling = over.out;
        for _ in 0..20 {
            over.step(11.0, None, 0.15);
        }
        assert!(
            over.out < pulling,
            "above the tail limit the elevator must relax: {} !< {pulling}",
            over.out
        );
        // No limit declared → the plain 8° target, unchanged.
        let mut ga = RotatePitch::with_authority(None, PITCH_MIN);
        for _ in 0..80 {
            ga.step(0.0, None, 0.15);
        }
        assert!((ga.target_deg() - ROTATE_PITCH_DEG).abs() < 1e-6);
    }

    /// A declared rotation rate walks the target up that fast; none keeps
    /// the airline 2.5 (the SR22 declares 5: LEMG 12 1790690995).
    #[test]
    fn a_declared_rotation_rate_walks_the_target_faster() {
        let target_after = |rate: Option<f64>| {
            let mut law = RotatePitch::with_authority(None, PITCH_MIN).with_rotate_rate(rate);
            for _ in 0..8 {
                law.step(-1.75, None, 0.25);
            }
            law.target_deg()
        };
        let airline = target_after(None);
        let single = target_after(Some(5.0));
        assert!((airline - (-1.75 + 2.5 * 1.75)).abs() < 0.3, "{airline}");
        assert!((single - (-1.75 + 5.0 * 1.75)).abs() < 1e-6, "{single}");
        assert!((target_after(Some(50.0)) - ROTATE_PITCH_DEG).abs() < 1e-6, "bounded at 6");
    }

    /// One rotation tick as the blackbox has it (a `roll` record from the one
    /// the rotation began on, then `climb_manual`).
    #[derive(Debug, Clone, Copy)]
    struct RotTick {
        t: f64,
        dt: f64,
        ias: f64,
        theta: f64,
        q: f64,
        /// Height, feet; zero on a roll record (it carries none).
        agl: f64,
        /// The column the law logged (climb ticks only).
        pitch: Option<f64>,
    }

    /// The three SR22 rotations of LEMG 13, 2026-09-29 (#184), by sortie,
    /// with the rotation rate each flew: 1790682305 (MacBook, 0.25 s tick)
    /// and 1790690995 at the airline 2.5, 1790694884 at the declared 5.
    fn sr22_rotations() -> Vec<(&'static str, Option<f64>, Vec<RotTick>)> {
        let recs: Vec<serde_json::Value> =
            include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/takeoffs/sr22_rotation_lemg_2026-09-29.jsonl"))
                .lines()
                .filter_map(|l| serde_json::from_str(l).ok())
                .collect();
        let f = |r: &serde_json::Value, k: &str| r[k].as_f64();
        [
            ("1790682305-full_auto", None),
            ("1790690995-full_auto", None),
            ("1790694884-full_auto", Some(5.0)),
        ]
        .into_iter()
        .map(|(sid, rate)| {
            let ticks = recs
                .iter()
                .filter(|r| r["sortie"] == sid)
                .filter(|r| r["event"] == "roll" || r["event"] == "climb_manual")
                .map(|r| RotTick {
                    t: f(r, "t").unwrap(),
                    dt: f(r, "dt").unwrap(),
                    ias: f(r, "ias").unwrap(),
                    theta: f(r, "theta").unwrap(),
                    q: f(r, "q").unwrap(),
                    agl: f(r, "agl").unwrap_or(0.0),
                    pitch: f(r, "pitch").filter(|_| r["event"] == "climb_manual"),
                })
                .collect();
            (sid, rate, ticks)
        })
        .collect()
    }

    /// The SR22's rotation law as it flew: GA authority, no ceiling, no
    /// hold, the sortie's rate.
    fn sr22_law(rate: Option<f64>) -> RotatePitch {
        RotatePitch::with_authority(None, PITCH_MIN).with_rotate_rate(rate)
    }

    /// The SR22's fitted rotation, as the profile declares it.
    fn rotate_model_sr22() -> crate::aircraft::RotateElevatorProfile {
        let text = std::fs::read_to_string("profiles/cirrus_sr22.yaml").unwrap();
        let p: crate::aircraft::AircraftProfile = serde_yaml::from_str(&text).unwrap();
        p.rotate_elevator
            .and_then(|m| m.valid())
            .expect("the SR22 declares its rotation")
    }

    /// Open loop over the recorded ticks: the column the law writes on each.
    fn replay_rotation(law: &mut RotatePitch, ticks: &[RotTick]) -> Vec<f64> {
        ticks
            .iter()
            .map(|k| {
                law.on_wheels(k.agl <= 0.0);
                law.step_climb(k.theta, Some(k.q), k.dt, Some((k.ias, k.agl)))
            })
            .collect()
    }

    /// The SR22's pitch through the rotation, fitted to those three (#184):
    /// a rate plant (the column sets a pitch rate through a lag `tau`, after
    /// the write's delay). The nose turns
    /// `k x ((v/ref)^2 x (u - per_deg x (alpha - rest)) - base)` deg/s, alpha
    /// the attitude less the flight path (level on the wheels), and never
    /// below the roll attitude on the wheels: the `per_deg` is the tail's
    /// spring, the base on the wheels the breakout (the nose's weight about the
    /// mains). Fitted by least squares on the attitude to 5 degrees on the
    /// wheels, the column replayed from the ticks: 0.35 degree rms.
    ///
    /// Flying (1790699819, #184): the base stays the breakout in ground effect
    /// and walks to `AIR_BASE` by `GROUND_EFFECT_FT` (0.31 held 4.6 degrees at
    /// 88 KIAS a few feet up; the 100 kt climbs hold 10 degrees on a 7 degree
    /// path with 0.1). The wing lifts as `v^2 x (alpha - LIFT_ZERO_DEG)`, ten
    /// percent more in ground effect, the weight balanced at 104 kt and 3.2
    /// degrees of alpha (the steady climb); the height is flown on the excess.
    /// With the elevator 1.4 and the spring 1.3 of the fit (the 1790699819
    /// airframe, whose nose came up at 4.7 deg/s on 0.38 of column), the law
    /// b2b1481 flew leaves the wheels at 80.7 KIAS and 5.0 degrees (flown
    /// 81.4, 5.0) and reaches 8 ft at 94.7 (flown 94.7).
    struct Sr22Rotation {
        theta: f64,
        q: f64,
        v: f64,
        /// Height, feet, and vertical speed, ft/s.
        h: f64,
        vs: f64,
        wheels: bool,
        /// Multipliers on the fitted elevator and spring (robustness).
        k_mult: f64,
        spring_mult: f64,
    }

    impl Sr22Rotation {
        const REF_KTS: f64 = 90.0;
        const K: f64 = 29.2;
        const BREAKOUT: f64 = 0.104;
        const PER_DEG: f64 = 0.0328;
        const TAU_S: f64 = 0.131;
        const DELAY_S: f64 = 0.26;
        const AIR_BASE: f64 = -0.059;
        const GROUND_EFFECT_FT: f64 = 20.0;
        const GROUND_EFFECT_LIFT: f64 = 0.10;
        const REST_DEG: f64 = -1.65;
        const LIFT_ZERO_DEG: f64 = -0.33;
        const CLIMB_KTS: f64 = 104.0;
        const CLIMB_ALPHA_DEG: f64 = 3.2;
        /// Airspeed gained per second on the roll (74 to 98 KIAS in 5.6 s),
        /// less half a knot a second per degree of path.
        const ACCEL_KTS_S: f64 = 4.3;

        fn new(theta: f64, v: f64) -> Self {
            Sr22Rotation {
                theta,
                q: 0.0,
                v,
                h: 0.0,
                vs: 0.0,
                wheels: true,
                k_mult: 1.0,
                spring_mult: 1.0,
            }
        }

        /// The airframe of 1790699819 (see above).
        fn flown_1790699819(v: f64) -> Self {
            Sr22Rotation {
                k_mult: 1.4,
                spring_mult: 1.3,
                ..Self::new(Self::REST_DEG, v)
            }
        }

        fn path_deg(&self) -> f64 {
            self.vs.atan2(self.v * 1.687_81).to_degrees()
        }

        fn advance(&mut self, u: f64, dur_s: f64) {
            let step_s = 0.005_f64;
            let mut left = dur_s;
            let weight = Self::CLIMB_KTS.powi(2) * (Self::CLIMB_ALPHA_DEG - Self::LIFT_ZERO_DEG);
            while left > 1e-9 {
                let step = step_s.min(left);
                let dq = (self.v / Self::REF_KTS).powi(2);
                let path = if self.wheels { 0.0 } else { self.path_deg() };
                let alpha = self.theta - path;
                let in_effect = (1.0 - self.h / Self::GROUND_EFFECT_FT).max(0.0);
                let base = if self.wheels {
                    Self::BREAKOUT
                } else {
                    Self::AIR_BASE + (Self::BREAKOUT - Self::AIR_BASE) * in_effect
                };
                let want = Self::K
                    * self.k_mult
                    * (dq * (u - Self::PER_DEG * self.spring_mult * (alpha - Self::REST_DEG))
                        - base);
                self.q += (want - self.q) / Self::TAU_S * step;
                self.theta += self.q * step;
                if self.wheels && self.theta < Self::REST_DEG {
                    self.theta = Self::REST_DEG;
                    self.q = self.q.max(0.0);
                }
                let lift = self.v.powi(2)
                    * (alpha - Self::LIFT_ZERO_DEG)
                    * (1.0 + Self::GROUND_EFFECT_LIFT * in_effect)
                    / weight;
                let accel = 32.2 * (lift - 1.0);
                if self.wheels && accel > 0.0 {
                    self.wheels = false;
                }
                if !self.wheels {
                    self.vs += accel * step;
                    self.h += self.vs * step;
                    if self.h <= 0.0 && self.vs < 0.0 {
                        self.h = 0.0;
                        self.vs = 0.0;
                        self.wheels = true;
                    }
                }
                self.v += (Self::ACCEL_KTS_S - 0.5 * path) * step;
                left -= step;
            }
        }
    }

    /// What a closed-loop rotation did, as the blackbox would say it.
    #[derive(Debug, Clone, Copy)]
    struct RotationFlown {
        /// Airspeed where the wheels left, KIAS, and the attitude there.
        wheels_off_kts: f64,
        wheels_off_deg: f64,
        /// Airspeed at 8 ft (the takeoff's `liftoff`), KIAS.
        liftoff_kts: f64,
        /// Seconds from Vr to the nose at 7 degrees.
        to_7_deg_s: f64,
        /// Highest attitude; the lowest between the wheels leaving and 8 ft
        /// (the float's sag); the lowest from a second after 8 ft on.
        max_theta: f64,
        float_min: f64,
        min_after: f64,
    }

    /// Fly `law` against the fitted SR22 from Vr, through the write's delay,
    /// at `tick` seconds with a jitter and the runway's rate noise; the law
    /// reads the wheels, the height and the vertical speed as the loop does.
    fn fly_sr22_rotation(
        mut law: RotatePitch,
        mut plant: Sr22Rotation,
        tick: f64,
        seed: u32,
    ) -> RotationFlown {
        let mut seed = seed.wrapping_mul(2_654_435_761).wrapping_add(1);
        let mut noise = move || {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            f64::from(seed >> 8) / f64::from(1_u32 << 23) - 1.0
        };
        let mut pending: Vec<(f64, f64)> = Vec::new();
        let mut applied = 0.0;
        let mut t = 0.0_f64;
        let (mut off_kts, mut off_deg) = (f64::NAN, f64::NAN);
        let (mut liftoff, mut lift_t) = (f64::NAN, f64::NAN);
        let mut to_7 = f64::NAN;
        let (mut max_theta, mut float_min, mut min_after) = (f64::MIN, f64::MAX, f64::MAX);
        while t < 14.0 {
            let dt = tick + 0.2 * tick * noise();
            let on = plant.wheels;
            let q_read = plant.q + noise() * if on { 1.0 } else { 0.1 };
            law.on_wheels(on);
            law.climb_fpm(Some(plant.vs * 60.0));
            let theta_read = plant.theta + 0.1 * noise();
            let cmd = law.step_climb(theta_read, Some(q_read), dt, Some((plant.v, plant.h)));
            pending.push((t + Sr22Rotation::DELAY_S, cmd));
            let end = t + dt;
            while t < end - 1e-9 {
                let next = pending
                    .iter()
                    .map(|&(at, _)| at)
                    .filter(|&at| at > t && at < end)
                    .fold(end, f64::min)
                    .min(t + 0.01);
                plant.advance(applied, next - t);
                t = next;
                while let Some(i) = pending.iter().position(|&(at, _)| at <= t) {
                    applied = pending.remove(i).1;
                }
                if !plant.wheels && off_kts.is_nan() {
                    off_kts = plant.v;
                    off_deg = plant.theta;
                }
                if plant.h > 8.0 && liftoff.is_nan() {
                    liftoff = plant.v;
                    lift_t = t;
                }
                if to_7.is_nan() && plant.theta >= 7.0 {
                    to_7 = t;
                }
                max_theta = max_theta.max(plant.theta);
                if off_kts.is_finite() && liftoff.is_nan() {
                    float_min = float_min.min(plant.theta);
                }
                if lift_t.is_finite() && t > lift_t + 1.0 {
                    min_after = min_after.min(plant.theta);
                }
            }
        }
        RotationFlown {
            wheels_off_kts: off_kts,
            wheels_off_deg: off_deg,
            liftoff_kts: liftoff,
            to_7_deg_s: to_7,
            max_theta,
            float_min,
            min_after,
        }
    }

    /// The replay IS the law as flown: stepped over the recorded ticks, the
    /// rotation law writes the column every climb tick logged, on all three
    /// flights. The roll records carry no column; this is how the SR22's was
    /// read: 0.03 to 0.36 through the whole rotation (PITCH_MAX 0.65).
    #[test]
    fn the_sr22_rotation_replays_as_flown() {
        for (sid, rate, ticks) in sr22_rotations() {
            let out = replay_rotation(&mut sr22_law(rate), &ticks);
            let mut climbs = 0;
            for (k, u) in ticks.iter().zip(&out) {
                if let Some(logged) = k.pitch {
                    assert!((u - logged).abs() < 1e-9, "{sid} at {}: {u} vs {logged}", k.t);
                    climbs += 1;
                }
            }
            assert!(climbs >= 10, "{sid}: {climbs} climb ticks");
            let ground_max = ticks
                .iter()
                .zip(&out)
                .filter(|(k, _)| k.agl <= 0.0)
                .map(|(_, u)| *u)
                .fold(f64::MIN, f64::max);
            assert!(ground_max < 0.37, "{sid}: the rotation never pulled past {ground_max}");
        }
    }

    /// The plant is the flown rotation: driven by the column each flight
    /// wrote (replayed), from the attitude at Vr, it keeps the recorded
    /// attitude on the wheels to 5 degrees within half a degree rms and a
    /// degree on every tick: the nose held back near 3 degrees at 88 to 94
    /// kt on the two iMac flights, 0.3 to 0.4 of column against the spring.
    #[test]
    fn the_sr22_plant_is_the_flown_rotation() {
        let mut sq = 0.0;
        let mut n = 0.0;
        for (sid, rate, ticks) in sr22_rotations() {
            let cols = replay_rotation(&mut sr22_law(rate), &ticks);
            let ground: Vec<_> = ticks
                .iter()
                .zip(&cols)
                .take_while(|(k, _)| k.agl <= 0.0 && k.theta <= 5.0)
                .collect();
            let mut plant = Sr22Rotation::new(ground[0].0.theta, ground[0].0.ias);
            let mut pending: Vec<(f64, f64)> = ground
                .iter()
                .map(|(k, u)| (k.t + Sr22Rotation::DELAY_S, **u))
                .collect();
            let mut applied = 0.0;
            let mut t = ground[0].0.t;
            for (k, _) in &ground {
                while t < k.t - 1e-9 {
                    while pending.first().is_some_and(|&(at, _)| at <= t) {
                        applied = pending.remove(0).1;
                    }
                    let step = 0.01_f64.min(k.t - t);
                    // The airspeed as recorded, not the plant's own.
                    plant.v = k.ias;
                    plant.advance(applied, step);
                    // Recorded on the wheels: the ground half of the plant.
                    (plant.wheels, plant.h, plant.vs) = (true, 0.0, 0.0);
                    t += step;
                }
                let e = plant.theta - k.theta;
                sq += e * e;
                n += 1.0;
                assert!(
                    e.abs() < 1.0,
                    "{sid} at {} kt: plant {} vs flown {}",
                    k.ias,
                    plant.theta,
                    k.theta
                );
            }
        }
        let rms = (sq / n).sqrt();
        assert!(rms < 0.5, "the fit holds: {rms:.2} degree rms over {n} ticks");
    }

    /// #184: the SR22 rotated at Vr 73 and lifted off at 98 to 100 KIAS on
    /// every flight, the nose 5 degrees behind the ramp. Against the fitted
    /// plant, at the iMac's 0.4 s tick and the MacBook's 0.25, the law as it
    /// flew then reaches 8 ft past 92 kt, the nose at 7 degrees 5 s after Vr;
    /// fed the declared model, by 88 kt, the nose at 7 inside 2.6 s, never
    /// higher than 10, no sag between the wheels and 8 ft and no drop under 7
    /// once climbing. The model is a hypothesis about the plant, so the
    /// elevator and the spring at 0.7 to 1.4 of the fit must still beat the
    /// old law, without the float's sag, and overshoot it by little.
    #[test]
    fn the_sr22_rotates_near_vr() {
        let model = rotate_model_sr22();
        let vr = 73.5;
        for tick in [0.25, 0.4] {
            for seed in 0..6 {
                let fly = |law: RotatePitch| {
                    let plant = Sr22Rotation::new(Sr22Rotation::REST_DEG, vr);
                    fly_sr22_rotation(law, plant, tick, seed)
                };
                let old = fly(sr22_law(Some(5.0)));
                assert!(old.liftoff_kts > 92.0, "the flown law, tick {tick}: {old:?}");
                assert!(old.to_7_deg_s > 5.0, "the flown law, tick {tick}: {old:?}");
                let new = fly(sr22_law(Some(5.0)).with_rotate_model(Some(model)));
                assert!(new.liftoff_kts < 88.0, "tick {tick} seed {seed}: {new:?}");
                assert!(new.to_7_deg_s < 2.6, "tick {tick} seed {seed}: {new:?}");
                assert!(new.max_theta < 10.0, "tick {tick} seed {seed}: {new:?}");
                assert!(new.float_min > new.wheels_off_deg - 0.3, "sagged: {new:?}");
                assert!(new.min_after > 7.0, "tick {tick} seed {seed}: {new:?}");
            }
        }
        for k_mult in [0.7, 1.0, 1.4] {
            for spring_mult in [0.7, 1.3] {
                let plant = || Sr22Rotation {
                    k_mult,
                    spring_mult,
                    ..Sr22Rotation::new(Sr22Rotation::REST_DEG, vr)
                };
                let old = fly_sr22_rotation(sr22_law(Some(5.0)), plant(), 0.4, 1);
                let law = sr22_law(Some(5.0)).with_rotate_model(Some(model));
                let new = fly_sr22_rotation(law, plant(), 0.4, 1);
                let case = format!("elevator x{k_mult}, spring x{spring_mult}");
                assert!(new.liftoff_kts < old.liftoff_kts - 3.0, "{case}: {new:?} vs {old:?}");
                // A softer spring than declared is an over-pull: 11.2 and 12.4
                // degrees at 0.7 of it (the old law 11.3 and 11.2).
                assert!(new.max_theta < 12.5, "{case}: {new:?}");
                assert!(new.max_theta < old.max_theta + 1.3, "{case}: {new:?} vs {old:?}");
                assert!(new.float_min > new.wheels_off_deg - 0.5, "{case}: {new:?}");
            }
        }
    }

    /// 1790699819 (LEMG 13, the iMac, 2026-09-29): the first model, flown.
    /// On the wheels it worked (Vr 74.6, wheels off 81.4 KIAS 1.4 s later at
    /// 5.0 degrees); then its flat flying hold took the column from 0.39 to
    /// 0.24, the nose sagged to 3.0, and the aircraft floated in ground
    /// effect to the 8 ft liftoff at 94.7 KIAS. Against the plant of that
    /// airframe, the law as b2b1481 flew it does the same (wheels off near
    /// 81 at 5 degrees, 8 ft past 92); the path-and-ground-effect model
    /// climbs out of the float by 89 without a sag, under 10 degrees.
    #[test]
    fn the_sr22_climbs_out_of_ground_effect() {
        let model = rotate_model_sr22();
        let mut flown_liftoffs = Vec::new();
        for tick in [0.25, 0.4] {
            for seed in 0..6 {
                let fly = |law: RotatePitch| {
                    fly_sr22_rotation(law, Sr22Rotation::flown_1790699819(74.6), tick, seed)
                };
                let law = || sr22_law(Some(5.0)).with_rotate_model(Some(model));
                let flown = fly(law().as_flown_b2b1481());
                assert!(
                    (79.0..=83.0).contains(&flown.wheels_off_kts)
                        && (4.0..=6.0).contains(&flown.wheels_off_deg),
                    "as flown, tick {tick} seed {seed}: {flown:?}"
                );
                // The float is chaotic (88.5 to 97.8 over the seeds); the
                // flight's 94.7 is its middle.
                assert!(flown.liftoff_kts > 88.0, "as flown, tick {tick} seed {seed}: {flown:?}");
                flown_liftoffs.push(flown.liftoff_kts);
                let new = fly(law());
                assert!(new.liftoff_kts < flown.liftoff_kts, "{new:?} vs {flown:?}");
                assert!(new.liftoff_kts < 89.0, "tick {tick} seed {seed}: {new:?}");
                assert!(new.float_min > new.wheels_off_deg - 0.3, "sagged: {new:?}");
                assert!(new.max_theta < 10.0, "tick {tick} seed {seed}: {new:?}");
                assert!(new.min_after > 7.0, "tick {tick} seed {seed}: {new:?}");
            }
        }
        let mean = flown_liftoffs.iter().sum::<f64>() / flown_liftoffs.len() as f64;
        assert!(mean > 92.0, "the float as flown: {mean:.1} kt at 8 ft");
    }

    /// The rotation records of 1790699819 (`rotate_tick`: the column and the
    /// model's share, the wheels).
    fn sr22_float_ticks() -> Vec<serde_json::Value> {
        include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/takeoffs/sr22_float_lemg13_1790699819.jsonl"))
            .lines()
            .filter_map(|l| serde_json::from_str::<serde_json::Value>(l).ok())
            .filter(|r| r["event"] == "rotate_tick")
            .collect()
    }

    /// Open loop over 1790699819's rotation: the law as flown (b2b1481)
    /// writes every logged column, wheels and float alike, so the replay is
    /// the flight. The model now: identical on the wheels, and at wheels-off
    /// (81.4 KIAS, 5.0 degrees) it keeps the pull, 0.35 or more where the
    /// flight dropped to 0.24; through the float it never feeds forward less
    /// than the 0.3 that held 4.6 degrees at 88 KIAS.
    #[test]
    fn the_float_of_1790699819_replays_and_keeps_its_pull() {
        let ticks = sr22_float_ticks();
        assert_eq!(ticks.len(), 12);
        let f = |r: &serde_json::Value, k: &str| r[k].as_f64().unwrap();
        let model = rotate_model_sr22();
        let mut flown = sr22_law(Some(5.0)).with_rotate_model(Some(model)).as_flown_b2b1481();
        let mut now = sr22_law(Some(5.0)).with_rotate_model(Some(model));
        let mut first_float = true;
        for r in &ticks {
            let wheels = r["wheels"] == true;
            // The float's height was not logged: a few feet, under the 8 ft.
            let air = Some((f(r, "ias"), if wheels { 0.0 } else { 3.0 }));
            let (theta, q, dt) = (f(r, "theta"), Some(f(r, "q")), f(r, "dt"));
            flown.on_wheels(wheels);
            now.on_wheels(wheels);
            let a = flown.step_climb(theta, q, dt, air);
            let b = now.step_climb(theta, q, dt, air);
            assert!((a - f(r, "pitch")).abs() < 1e-9, "replay at {}: {a}", f(r, "t"));
            assert!((flown.model_pull() - f(r, "pitch_ff")).abs() < 1e-9);
            if wheels {
                assert_eq!(a.to_bits(), b.to_bits(), "the wheels are unchanged");
            } else {
                if first_float {
                    assert!(a < 0.25 && b > 0.35, "wheels-off: flown {a}, now {b}");
                    first_float = false;
                }
                assert!(now.model_pull() > 0.3, "fed {} at {}", now.model_pull(), f(r, "ias"));
            }
        }
        assert!(!first_float, "the float is in the record");
    }

    /// No declaration, no change: the law without a model writes the same
    /// column tick for tick whether the wheels are said to be down or not,
    /// and a model that is not sane is no model.
    #[test]
    fn an_undeclared_rotation_model_changes_nothing() {
        for (_, rate, ticks) in sr22_rotations() {
            let mut a = sr22_law(rate);
            let mut b = sr22_law(rate).with_rotate_model(None);
            let insane = crate::aircraft::RotateElevatorProfile {
                breakout: 5.0,
                ..rotate_model_sr22()
            };
            let mut c = sr22_law(rate).with_rotate_model(Some(insane));
            for k in &ticks {
                b.on_wheels(true);
                c.on_wheels(k.agl <= 0.0);
                let air = Some((k.ias, k.agl));
                let x = a.step_climb(k.theta, Some(k.q), k.dt, air);
                assert_eq!(x.to_bits(), b.step_climb(k.theta, Some(k.q), k.dt, air).to_bits());
                assert_eq!(x.to_bits(), c.step_climb(k.theta, Some(k.q), k.dt, air).to_bits());
            }
        }
    }

    #[test]
    fn rotation_is_an_elevator_pull_that_holds_the_attitude() {
        // Sortie five: rotate called at 55 knots, the AP's VS mode held the
        // nose down (engage syncs to the current rate: zero), and the
        // aircraft levitated off at 98 with 250 m left. The rotation law is
        // an integrating elevator pull to the target attitude.
        let mut law = RotatePitch::default();
        let first = law.step(0.0, None, 0.15);
        assert!(first > 0.0, "nose on the ground, pull back: {first}");
        let mut out = first;
        for _ in 0..40 {
            out = law.step(0.0, None, 0.15);
        }
        assert!(out > 0.3, "an unresponsive nose earns more pull: {out}");
        assert!(out <= PITCH_MAX + 1e-9, "capped: {out}");
        // Integrating: on target and steady, the standing pull HOLDS — the
        // elevator the trim state demands at this speed.
        let mut settled = RotatePitch::default();
        for _ in 0..10 {
            settled.step(ROTATE_PITCH_DEG - 1.0, None, 0.15);
        }
        // Arriving at the target leaves a rate residue for a tick or two
        // before the pull is truly standing. Let that wash out, then it must
        // hold dead still — and it must CONVERGE on the way, never wander.
        let mut prev_gap = f64::MAX;
        for _ in 0..20 {
            let x = settled.step(ROTATE_PITCH_DEG, None, 0.15);
            let y = settled.step(ROTATE_PITCH_DEG, None, 0.15);
            let gap = (x - y).abs();
            assert!(gap <= prev_gap + 1e-9, "must converge, not ring: {gap}");
            prev_gap = gap;
        }
        let a = settled.step(ROTATE_PITCH_DEG, None, 0.15);
        let b = settled.step(ROTATE_PITCH_DEG, None, 0.15);
        assert!(
            a > 0.0 && (a - b).abs() < 1e-6,
            "holds at target: {a} vs {b}"
        );
        // A nose rising fast toward the target backs the pull off (the
        // rate lead), so the rotation doesn't slingshot past the attitude.
        let mut rising = RotatePitch::default();
        let before = rising.step(2.0, None, 0.15);
        let after = rising.step(6.0, None, 0.15);
        assert!(after < before, "fast rise backs off: {before} -> {after}");
        // And the nose-down authority is a fraction of the pull: this law
        // rotates, it never bunts.
        let mut over = RotatePitch::default();
        for _ in 0..100 {
            over.step(30.0, None, 0.15);
        }
        assert!(over.out >= PITCH_MIN - 1e-9, "floor holds: {}", over.out);
    }

    /// A second-order pitch plant: elevator commands pitch ACCELERATION,
    /// aero damping opposes the rate, static stability opposes the attitude.
    /// `m0` is a standing nose-up moment — climb power plus a nose-up
    /// stabiliser, which is what makes holding the target need nose-DOWN
    /// elevator against a floor of only [`PITCH_MIN`].
    ///
    /// This is the order the real aircraft has, and it is the whole point: the
    /// sortie test's pitch model is a first-order lag, and a single lag cannot
    /// ring at any gain, which is why the law rang through three gain changes
    /// with every test passing.
    struct PitchPlant {
        theta: f64,
        q: f64,
        wn: f64,
        zeta: f64,
        m_delta: f64,
        m0: f64,
    }

    /// A pitch plant the takeoff harnesses can fly: the LEMG-fitted
    /// [`PitchPlant`] or the EGKK-fitted [`RatePlant`].
    trait PitchModel {
        fn advance(&mut self, elevator: f64, dur_s: f64);
        fn theta(&self) -> f64;
        fn q(&self) -> f64;
        /// On the wheels: the attitude stops at the runway.
        fn hold_on_runway(&mut self);
    }

    impl PitchModel for PitchPlant {
        fn advance(&mut self, elevator: f64, dur_s: f64) {
            PitchPlant::advance(self, elevator, dur_s);
        }
        fn theta(&self) -> f64 {
            self.theta
        }
        fn q(&self) -> f64 {
            self.q
        }
        fn hold_on_runway(&mut self) {
            self.theta = 0.0;
            self.q = self.q.max(0.0);
        }
    }

    /// The 737's pitch as EGKK 08R flew it (sortie 1790601822-full_auto,
    /// 2026-09-28, #78): the column sets a pitch RATE, `k` degrees a second
    /// per unit of column past `u0`, the column that holds the attitude,
    /// through a lag `tau` (the harness adds the loop's transport delay).
    /// Second order in the attitude. Fitted to the 27 airborne ticks from
    /// liftoff to the 300 ft engage, driven by the column the law logged:
    /// `k` 12.9, `u0` -0.139, `tau` 0.1 s, delay 0.4 s, 0.23 degree RMS on
    /// the attitude. The -0.139 is the finding: the 737 pitched up at 3
    /// degrees a second on a column of 0.10 and needed a PUSH of 0.14 to stop,
    /// the nose-up trim and the thrust line the rotation rode on (it rotated
    /// at 2.9 degrees a second on a column of about zero).
    struct RatePlant {
        theta: f64,
        q: f64,
        k: f64,
        u0: f64,
        tau: f64,
    }

    impl RatePlant {
        /// As fitted.
        const K: f64 = 12.9;
        const U0: f64 = -0.139;
        const TAU_S: f64 = 0.1;
        const DELAY_S: f64 = 0.4;
    }

    impl PitchModel for RatePlant {
        fn advance(&mut self, elevator: f64, dur_s: f64) {
            let h = 0.005_f64;
            let mut left = dur_s;
            while left > 1e-9 {
                let step = h.min(left);
                self.q += (self.k * (elevator - self.u0) - self.q) / self.tau * step;
                self.theta += self.q * step;
                left -= step;
            }
        }
        fn theta(&self) -> f64 {
            self.theta
        }
        fn q(&self) -> f64 {
            self.q
        }
        fn hold_on_runway(&mut self) {
            self.theta = 0.0;
            self.q = self.q.max(0.0);
        }
    }

    impl PitchPlant {
        fn new(zeta: f64, m0: f64) -> Self {
            PitchPlant {
                theta: 0.0,
                q: 0.0,
                wn: 2.5,
                zeta,
                m_delta: 167.0,
                m0,
            }
        }
        fn advance(&mut self, elevator: f64, dur_s: f64) {
            let h = 0.005_f64;
            let mut left = dur_s;
            while left > 1e-9 {
                let step = h.min(left);
                let qd = self.m_delta * elevator + self.m0
                    - 2.0 * self.zeta * self.wn * self.q
                    - self.wn * self.wn * self.theta;
                self.q += qd * step;
                self.theta += self.q * step;
                left -= step;
            }
        }
    }

    /// Fly the rotation law against that plant through the loop's real
    /// transport delay (a REST write does not reach the aircraft the instant
    /// it is computed) and measure what the blackbox measures: the attitude
    /// spread once the ramp has arrived, and how much of the time the elevator
    /// is pinned against a stop.
    ///
    /// Returns (spread_deg, saturated_fraction).
    fn fly_rotation(zeta: f64, m0: f64, lag_s: f64, measured_rate: bool) -> (f64, f64) {
        let mut plant = PitchPlant::new(zeta, m0);
        let mut law = RotatePitch::default();
        // A deterministic stand-in for the wobble in a real attitude read —
        // the differencing estimator's whole problem is what it does to this.
        // A deterministic LCG, because the wobble must actually wobble: this
        // is the ±0.05° the differencing estimator turns into ±0.2°/s of
        // phantom rate at a 0.25 s tick, which is the whole mechanism under
        // test. (Written first as a float recurrence that silently collapsed
        // to a constant zero — a noise source stuck at zero makes the fallback
        // path look better than it is.)
        let mut seed = 1_u32;
        let mut wobble = move || {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (f64::from(seed >> 8) / f64::from(1_u32 << 23) - 1.0) * 0.05
        };
        let mut pending: Vec<(f64, f64)> = Vec::new();
        let mut applied = 0.0;
        let (mut t, dt) = (0.0_f64, 0.25_f64);
        let (mut lo, mut hi) = (f64::MAX, f64::MIN);
        let (mut ticks, mut saturated) = (0.0, 0.0);
        while t < 18.0 {
            let rate = measured_rate.then_some(plant.q);
            let cmd = law.step(plant.theta + wobble(), rate, dt);
            pending.push((t + lag_s, cmd));
            // Advance the plant across the tick, applying commands as they land.
            let (mut now, end) = (t, t + dt);
            while now < end - 1e-9 {
                let next = pending
                    .iter()
                    .map(|&(at, _)| at)
                    .filter(|&at| at > now && at < end)
                    .fold(end, f64::min);
                plant.advance(applied, next - now);
                now = next;
                while let Some(i) = pending.iter().position(|&(at, _)| at <= now) {
                    applied = pending.remove(i).1;
                }
            }
            t = end;
            // Measure once the ramp has had time to deliver the nose.
            if t > 5.0 {
                lo = lo.min(plant.theta);
                hi = hi.max(plant.theta);
                ticks += 1.0;
                if cmd <= PITCH_MIN + 1e-9 || cmd >= PITCH_MAX - 1e-9 {
                    saturated += 1.0;
                }
            }
        }
        (hi - lo, saturated / ticks)
    }

    /// The rotation must SETTLE on the target, not ring about it.
    ///
    /// Four takeoffs at EGHI on one evening, same airframe either side of the
    /// P+I change: 1.26 -> 1.89 direction reversals per second, 20% -> 48% of
    /// ticks hard against `PITCH_MIN`. Every sortie after it was the same
    /// shape (08-01: 57% and 2.42/s; 08-02: 48% and 2.27/s, ~0.9 s period,
    /// holding 7.0-10.8 about an 8 degree target). The structure was not the
    /// fault — the DAMPING term was, on both counts: a rate differenced from
    /// the attitude over a 0.25 s tick, and a 1 s lead on it.
    ///
    /// This is the test that could have caught it. The plant is second order
    /// with the loop's transport delay, so it CAN ring; the assertions are the
    /// two numbers the blackbox reports.
    #[test]
    fn the_rotation_settles_instead_of_ringing() {
        // Across the plant uncertainty — damping, standing nose-up moment, and
        // how long a REST write takes to reach the aircraft.
        for zeta in [0.35, 0.45, 0.55] {
            for m0 in [0.0, 20.0, 40.0] {
                for lag in [0.10, 0.20, 0.30] {
                    let (spread, sat) = fly_rotation(zeta, m0, lag, true);
                    assert!(
                        spread < 4.0,
                        "rings: {spread:.1} deg spread (zeta {zeta}, m0 {m0}, lag {lag})"
                    );
                    assert!(
                        sat < 0.05,
                        "camps on a stop: {:.0}% of ticks (zeta {zeta}, m0 {m0}, lag {lag})",
                        sat * 100.0
                    );
                }
            }
        }
        // The fallback — no measured rate, the law differences the attitude —
        // must cost accuracy, not stability. This is why PITCH_LEAD_S is 0.3
        // and not the 0.5 the measured path alone would allow: at 0.5 this
        // same sweep reaches a 14.6 degree spread with 31% saturation.
        for lag in [0.10, 0.20, 0.30] {
            let (spread, sat) = fly_rotation(0.45, 40.0, lag, false);
            assert!(spread < 6.0, "fallback rings: {spread:.1} deg (lag {lag})");
            assert!(sat < 0.05, "fallback camps: {:.0}%", sat * 100.0);
        }
        // And the shipped lead is on the flat part of the curve, not the edge
        // of a cliff: the whole failure was a constant sitting past one.
        let at_ship = fly_rotation(0.45, 40.0, 0.20, true).0;
        assert!(
            at_ship < 2.5,
            "the tuned point must be comfortable: {at_ship:.1} deg"
        );
    }

    /// The 737's rotation and hand-flown climb, rotate to the 300 ft engage,
    /// against `plant` (through the loop's transport delay) and a flight path
    /// with the aircraft's own excess thrust. The path and the [`PitchPlant`]
    /// were fitted to LEMG 13 (sortie 1790596335-full_auto, 2026-09-28; the
    /// hold is judged on the [`RatePlant`] refitted to EGKK 08R): the law pushing
    /// -0.37 to hold 11 degrees is a standing nose-up moment of about 130
    /// (6.25 x 11 + 167 x 0.37); 2,280 fpm at 184 kt is a 7 degree path at
    /// 11 degrees of pitch, so about 4 degrees of alpha there; 1.2 kt/s on
    /// that path is a thrust-to-weight excess of 0.185. Returns what the
    /// blackbox reports per `climb_manual` tick: the airspeed at 300 ft, the
    /// highest airspeed and attitude from liftoff to 300 ft, the highest
    /// tail margin (the attitude less the 11 degree strike plus the angle the
    /// height buys over the -800's 60 ft arm; positive is a strike), and the
    /// share of ticks the elevator
    /// sat on a stop.
    fn fly_initial_climb(
        hold: Option<ClimbHold>,
        mut plant: impl PitchModel,
        lag_s: f64,
    ) -> (f64, f64, f64, f64, f64) {
        const TW: f64 = 0.185;
        const KTS_S_PER_G: f64 = 9.81 / 0.514_444;
        let mut law = RotatePitch::with_authority(Some(11.0), -0.5).with_climb_hold(hold);
        let (mut ias, mut gamma, mut h_ft) = (145.5_f64, 0.0_f64, 0.0_f64);
        let mut airborne = false;
        let mut pending: Vec<(f64, f64)> = Vec::new();
        let mut applied = 0.0;
        let (mut t, dt) = (0.0_f64, 0.2_f64);
        let (mut max_ias, mut max_theta, mut max_theta_low) = (0.0_f64, 0.0_f64, f64::MIN);
        let (mut ticks, mut saturated) = (0.0, 0.0);
        let mut at_300 = None;
        // A tenth of a knot of read noise, deterministic.
        let mut seed = 7_u32;
        let mut noise = move || {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (f64::from(seed >> 8) / f64::from(1_u32 << 23) - 1.0) * 0.1
        };
        while t < 60.0 && at_300.is_none() {
            let cmd = law.step_climb(
                plant.theta(),
                Some(plant.q()),
                dt,
                Some((ias + noise(), h_ft)),
            );
            pending.push((t + lag_s, cmd));
            let (mut now, end) = (t, t + dt);
            while now < end - 1e-9 {
                let next = pending
                    .iter()
                    .map(|&(at, _)| at)
                    .filter(|&at| at > now && at < end)
                    .fold(end, f64::min);
                let span = next - now;
                plant.advance(applied, span);
                // On the wheels the attitude cannot go below the runway.
                if !airborne && plant.theta() < 0.0 {
                    plant.hold_on_runway();
                }
                let alpha = 4.0 * (184.0 / ias).powi(2);
                if !airborne && ias >= 160.0 && plant.theta() >= alpha {
                    airborne = true;
                }
                if airborne {
                    gamma += ((plant.theta() - alpha) - gamma) / 1.5 * span;
                    ias += (TW - gamma.to_radians().sin()) * KTS_S_PER_G * span;
                    h_ft += ias * 1.6878 * gamma.to_radians().sin() * span;
                } else {
                    // The recorded ground run: 145 to 162 in 4.4 s.
                    ias += 3.9 * span;
                }
                now = next;
                while let Some(i) = pending.iter().position(|&(at, _)| at <= now) {
                    applied = pending.remove(i).1;
                }
            }
            t = end;
            if airborne && h_ft > 8.0 {
                max_ias = max_ias.max(ias);
                max_theta = max_theta.max(plant.theta());
                let strike = 11.0 + (h_ft / 60.0).atan().to_degrees();
                max_theta_low = max_theta_low.max(plant.theta() - strike);
                ticks += 1.0;
                if cmd <= -0.5 + 1e-9 || cmd >= PITCH_MAX - 1e-9 {
                    saturated += 1.0;
                }
                if h_ft >= AP_ENGAGE_AGL_FT {
                    at_300 = Some(ias);
                }
            }
        }
        (
            at_300.expect("reaches the engage height"),
            max_ias,
            max_theta,
            max_theta_low,
            saturated / ticks,
        )
    }

    /// #78: the hand-flown segment holds V2 to V2 + 20, not the rotation
    /// attitude. The fixed 8 degrees reproduces LEMG 13 (162 at liftoff, 185
    /// at 300 ft on all five circuits); the hold keeps the 737 inside its
    /// band with the tail clear, across the plant uncertainty.
    #[test]
    fn the_initial_climb_holds_v2_plus_fifteen_by_pitch() {
        let b738 = crate::aircraft::load("boeing_737_800").expect("737 profile loads");
        let initial = b738
            .reference
            .as_ref()
            .and_then(|r| r.initial_climb_kts)
            .unwrap();
        let hold = ClimbHold::new(initial, b738.climb_pitch_max_deg);
        let h = hold.expect("the 737 declares its climb attitude");
        assert_eq!(h.hold_kts, 165.0, "V2 150 + 15");
        let (v2, top) = (initial - 20.0, initial);

        // The recorded flight, reproduced: the plant is the aircraft.
        let (at_300, ..) = fly_initial_climb(None, PitchPlant::new(0.45, 130.0), 0.2);
        assert!(at_300 > 178.0, "the old law runs away as the 737 did: {at_300:.1}");
        let fitted = RatePlant {
            theta: 0.0,
            q: 0.0,
            k: RatePlant::K,
            u0: RatePlant::U0,
            tau: RatePlant::TAU_S,
        };
        let (at_300, ..) = fly_initial_climb(None, fitted, RatePlant::DELAY_S);
        assert!(at_300 > 178.0, "and on the EGKK plant: {at_300:.1}");

        // The hold is flown on the plant refitted to its first real flight
        // (EGKK 08R, #78), across its authority, holding push and lag. The
        // LEMG plant above was fitted to a standing push alone: driven by
        // the column EGKK logged it flies 25 to 28 degrees against the
        // recorded 22.4 (6 degrees RMS), a pitch six times too quick for the
        // 737 it stands for, so it no longer judges the climb.
        let mut plants = Vec::new();
        for k in [10.0, RatePlant::K, 16.0] {
            for u0 in [-0.10, RatePlant::U0, -0.20] {
                for tau in [RatePlant::TAU_S, 0.25] {
                    plants.push((k, u0, tau));
                }
            }
        }
        for &(k, u0, tau) in &plants {
            for lag in [0.2, RatePlant::DELAY_S, 0.5] {
                let plant = RatePlant {
                    theta: 0.0,
                    q: 0.0,
                    k,
                    u0,
                    tau,
                };
                let (at_300, max_ias, max_theta, low, sat) = fly_initial_climb(hold, plant, lag);
                let tag = format!("k {k}, u0 {u0}, tau {tau}, lag {lag}");
                assert!(
                    (v2..=top).contains(&max_ias),
                    "peak {max_ias:.1} outside V2 {v2} to V2 + 20 ({tag})"
                );
                assert!(
                    (h.hold_kts - 5.0..=h.hold_kts + 5.0).contains(&at_300),
                    "at the engage {at_300:.1} vs hold {} ({tag})",
                    h.hold_kts
                );
                assert!(
                    max_theta <= h.pitch_max_deg,
                    "theta {max_theta:.2} over the ceiling ({tag})"
                );
                assert!(low < 0.0, "tail strike: {low:.1} deg past it ({tag})");
                assert!(sat < 0.05, "camps on a stop: {:.0}% ({tag})", sat * 100.0);
            }
        }
    }

    /// EGKK 08R, the first ClimbHold flight (#78), as the blackbox drew it:
    /// every `roll` tick from the rotation and every `climb_manual` tick to
    /// the 300 ft engage.
    struct EgkkTick {
        climb: bool,
        t: f64,
        theta: f64,
        q: f64,
        dt: f64,
        ias: f64,
        /// `climb_manual` only: the height and the column the law wrote.
        agl: f64,
        pitch: f64,
        /// The target as logged (a `roll` record logs the PREVIOUS tick's).
        pitch_tgt: f64,
    }

    fn egkk_ticks() -> (ClimbHold, Vec<EgkkTick>) {
        let recs: Vec<serde_json::Value> =
            include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/takeoffs/egkk08r_climb_hold.jsonl"))
                .lines()
                .filter_map(|l| serde_json::from_str(l).ok())
                .collect();
        let f = |r: &serde_json::Value, k: &str| r[k].as_f64().unwrap_or(f64::NAN);
        let h = recs
            .iter()
            .find(|r| r["event"] == "climb_hold")
            .expect("climb_hold");
        let hold = ClimbHold {
            hold_kts: f(h, "hold_kts"),
            pitch_max_deg: f(h, "pitch_max_deg"),
        };
        let rotate_t = recs
            .iter()
            .find(|r| r["event"] == "rotate")
            .map(|r| f(r, "t"))
            .unwrap();
        let ticks = recs
            .iter()
            .filter(|r| {
                (r["event"] == "roll" && f(r, "t") >= rotate_t) || r["event"] == "climb_manual"
            })
            .map(|r| EgkkTick {
                climb: r["event"] == "climb_manual",
                t: f(r, "t"),
                theta: f(r, "theta"),
                q: f(r, "q"),
                dt: f(r, "dt"),
                ias: f(r, "ias"),
                agl: f(r, "agl"),
                pitch: f(r, "pitch"),
                pitch_tgt: f(r, "pitch_tgt"),
            })
            .collect();
        (hold, ticks)
    }

    /// The law the 737 flew (tail strike 11, push authority 0.5), with the
    /// EGKK hold.
    fn egkk_law(hold: ClimbHold, guarded: bool) -> RotatePitch {
        let law = RotatePitch::with_authority(Some(11.0), -0.5).with_climb_hold(Some(hold));
        if guarded {
            law
        } else {
            law.without_ceiling_guard()
        }
    }

    /// Steps the law over the recorded rotation (the roll ticks, open loop:
    /// they carry the attitude the aircraft had) and returns the column each
    /// wrote, (t, column). A roll record carries no height, so the one the
    /// law read is given back by the tail ceiling the NEXT record's target
    /// was held under; the last, by the first climb tick's.
    fn egkk_rotation(law: &mut RotatePitch, ticks: &[EgkkTick]) -> Vec<(f64, f64)> {
        let first_climb = ticks.iter().position(|k| k.climb).unwrap();
        (0..first_climb)
            .map(|i| {
                let k = &ticks[i];
                let agl = if ticks[i + 1].climb {
                    ticks[i + 1].agl
                } else {
                    let over = (ticks[i + 1].pitch_tgt - (11.0 - TAIL_STRIKE_MARGIN_DEG)).max(0.0);
                    TAIL_ARM_FT * over.to_radians().tan()
                };
                (
                    k.t,
                    law.step_climb(k.theta, Some(k.q), k.dt, Some((k.ias, agl))),
                )
            })
            .collect()
    }

    /// Flies the law from EGKK's liftoff against a [`RatePlant`] (`k`, `u0`,
    /// `tau`) through `delay_s`: the rotation replayed as recorded, then
    /// closed loop over the recorded airspeed and height to the 300 ft
    /// engage, then `extra_s` more on the air as last read. Returns the
    /// highest attitude and (t, theta) per tick.
    fn fly_egkk(
        guarded: bool,
        plant: (f64, f64, f64),
        delay_s: f64,
        extra_s: f64,
    ) -> (f64, Vec<(f64, f64)>) {
        let (hold, ticks) = egkk_ticks();
        let mut law = egkk_law(hold, guarded);
        let mut pending: Vec<(f64, f64)> = egkk_rotation(&mut law, &ticks)
            .into_iter()
            .map(|(t, u)| (t + delay_s, u))
            .collect();
        let air: Vec<&EgkkTick> = ticks.iter().filter(|k| k.climb).collect();
        let (k, u0, tau) = plant;
        let mut p = RatePlant {
            theta: air[0].theta,
            q: air[0].q,
            k,
            u0,
            tau,
        };
        let mut t = air[0].t;
        let mut applied = pending
            .iter()
            .filter(|(at, _)| *at <= t)
            .last()
            .map_or(0.0, |x| x.1);
        pending.retain(|(at, _)| *at > t);
        let end = air.last().unwrap().t + extra_s;
        let (mut max_theta, mut log) = (p.theta, Vec::new());
        let mut i = 0;
        while t < end {
            let r = air[i.min(air.len() - 1)];
            let dt = if i < air.len() { r.dt } else { 0.22 };
            let cmd = law.step_climb(p.theta, Some(p.q), dt, Some((r.ias, r.agl)));
            log.push((t, p.theta));
            pending.push((t + delay_s, cmd));
            let stop = t + dt;
            while t < stop - 1e-9 {
                let next = pending
                    .iter()
                    .map(|&(at, _)| at)
                    .filter(|&at| at > t && at < stop)
                    .fold(stop, f64::min);
                p.advance(applied, next - t);
                t = next;
                while let Some(j) = pending.iter().position(|&(at, _)| at <= t + 1e-9) {
                    applied = pending.remove(j).1;
                }
                max_theta = max_theta.max(p.theta);
            }
            i += 1;
        }
        (max_theta, log)
    }

    /// #78, EGKK 08R (sortie 1790601822-full_auto, 2026-09-28): the hold held
    /// its speed (162.6 to 170.3 kt to the engage) and the nose flew to 22.4
    /// degrees under an 18 degree ceiling. The TARGET never passed 18 (it
    /// stopped there at 77 ft); the rotation tracked its ramp within 0.6
    /// degrees. The attitude loop overshot: the nose arrived at 18 rising
    /// 2.6 degrees a second, and four degrees past it the law was pushing
    /// 0.18 on a 737 that needs 0.14 of push just to stop rotating.
    #[test]
    fn the_egkk_climb_stays_under_its_ceiling() {
        let (hold, ticks) = egkk_ticks();
        assert_eq!((hold.hold_kts, hold.pitch_max_deg), (165.0, 18.0));
        let recorded_max = ticks.iter().map(|k| k.theta).fold(f64::MIN, f64::max);
        assert!((recorded_max - 22.38).abs() < 0.01, "{recorded_max}");

        // The replay: the law without the guard, stepped over the recorded
        // ticks, wrote the column the blackbox logged, every climb tick.
        let mut law = egkk_law(hold, false);
        let mut column = egkk_rotation(&mut law, &ticks);
        for k in ticks.iter().filter(|k| k.climb) {
            let u = law.step_climb(k.theta, Some(k.q), k.dt, Some((k.ias, k.agl)));
            assert!(
                (u - k.pitch).abs() < 0.002,
                "t {}: replayed {u:.3} vs logged {:.3}",
                k.t,
                k.pitch
            );
            assert!(
                (law.target_deg() - k.pitch_tgt).abs() < 0.01,
                "target at t {}",
                k.t
            );
            assert!(
                law.target_deg() <= hold.pitch_max_deg,
                "the target never passed the ceiling"
            );
            column.push((k.t, k.pitch));
        }

        // The plant IS the aircraft: driven by that column through the
        // fitted delay from the liftoff tick, it flies the recorded attitude.
        let air: Vec<&EgkkTick> = ticks.iter().filter(|k| k.climb).collect();
        let mut p = RatePlant {
            theta: air[0].theta,
            q: air[0].q,
            k: RatePlant::K,
            u0: RatePlant::U0,
            tau: RatePlant::TAU_S,
        };
        let applied_at = |t: f64| {
            column
                .iter()
                .filter(|(at, _)| at + RatePlant::DELAY_S <= t + 1e-9)
                .last()
                .map_or(0.0, |x| x.1)
        };
        let (mut sq, mut worst, mut peak) = (0.0, 0.0_f64, 0.0_f64);
        for w in air.windows(2) {
            let mut t = w[0].t;
            while t < w[1].t - 1e-9 {
                let step = 0.005_f64.min(w[1].t - t);
                p.advance(applied_at(t), step);
                t += step;
            }
            let e = p.theta - w[1].theta;
            sq += e * e;
            worst = worst.max(e.abs());
            peak = peak.max(p.theta);
        }
        let rms = (sq / (air.len() - 1) as f64).sqrt();
        assert!(rms < 0.3, "fit RMS {rms:.2}");
        assert!(worst < 0.6, "fit worst {worst:.2}");
        assert!(
            (peak - recorded_max).abs() < 0.5,
            "fitted peak {peak:.2} vs {recorded_max}"
        );

        // Closed loop on the fitted plant: the old law reproduces the
        // overshoot, the guarded law keeps the nose under the ceiling.
        let fitted = (RatePlant::K, RatePlant::U0, RatePlant::TAU_S);
        let (old, _) = fly_egkk(false, fitted, RatePlant::DELAY_S, 0.0);
        assert!(
            old > 21.0,
            "the old law flies past the ceiling as the 737 did: {old:.2}"
        );
        let (new, log) = fly_egkk(true, fitted, RatePlant::DELAY_S, 10.0);
        assert!(new <= hold.pitch_max_deg, "guarded: {new:.2}");
        // Still a climb attitude, not a flattened one, by the engage.
        let engage_t = air.last().unwrap().t;
        let by_engage = log
            .iter()
            .filter(|(t, _)| *t <= engage_t)
            .map(|x| x.1)
            .fold(f64::MIN, f64::max);
        assert!(by_engage > 15.0, "climbs to its attitude: {by_engage:.2}");
        // And across the plant uncertainty: authority, the holding push, the
        // lag and the transport delay.
        for k in [10.0, 12.9, 16.0] {
            for u0 in [-0.10, -0.14, -0.20] {
                for tau in [0.1, 0.25, 0.4] {
                    for delay in [0.2, 0.35, 0.5] {
                        let (max, log) = fly_egkk(true, (k, u0, tau), delay, 10.0);
                        let tag = format!("k {k}, u0 {u0}, tau {tau}, delay {delay}");
                        assert!(
                            max <= hold.pitch_max_deg,
                            "{max:.2} over the ceiling ({tag})"
                        );
                        // No ring: the last four seconds move less than a degree.
                        let end = log.last().unwrap().0;
                        let tail: Vec<f64> = log
                            .iter()
                            .filter(|(t, _)| *t > end - 4.0)
                            .map(|x| x.1)
                            .collect();
                        let spread = tail.iter().cloned().fold(f64::MIN, f64::max)
                            - tail.iter().cloned().fold(f64::MAX, f64::min);
                        assert!(spread < 1.0, "rings: {spread:.2} ({tag})");
                    }
                }
            }
        }
    }

    /// Silent profiles keep the fixed attitude: no ceiling, no hold, and the
    /// law's target never passes the rotation attitude whatever the speed.
    #[test]
    fn an_airframe_without_a_climb_attitude_keeps_the_rotation_attitude() {
        assert_eq!(ClimbHold::new(74.0, None), None);
        assert_eq!(
            ClimbHold::new(74.0, Some(ROTATE_PITCH_DEG)),
            None,
            "no room above the rotation attitude"
        );
        let c172 = crate::aircraft::builtin("cessna_172sp").unwrap();
        assert_eq!(c172.climb_pitch_max_deg, None);
        let mut law = RotatePitch::with_authority(None, PITCH_MIN);
        let mut ias = 60.0;
        for _ in 0..100 {
            ias += 1.0;
            law.step_climb(8.0, Some(0.0), 0.2, Some((ias, 200.0)));
        }
        assert_eq!(law.target_deg(), ROTATE_PITCH_DEG);
    }

    /// #91: thrust comes back to the declared climb lever at the reduction
    /// height, airborne only, at a crew's pace; every other release (the
    /// swinging-nose hold, the overspeed guard) stays instant, and a silent
    /// airframe keeps takeoff thrust as it always did.
    #[test]
    fn thrust_reduction_at_the_declared_height() {
        let r = Some(crate::takeoff_config::ThrustReduction { agl_ft: 1000.0, lever: 0.9 });
        assert_eq!(thrust_reduced(1.0, true, 999.0, r), (1.0, false));
        assert_eq!(thrust_reduced(1.0, false, 1200.0, r), (1.0, false), "on the ground never");
        assert_eq!(thrust_reduced(1.0, true, 1000.0, r), (0.9, true));
        assert_eq!(thrust_reduced(0.7, true, 1500.0, r), (0.7, false), "the guard's lower lever stands");
        assert_eq!(thrust_reduced(1.0, true, 1500.0, None), (1.0, false));
        // Paced: from takeoff thrust to 0.9 over a second, never past it.
        let mut lever = 1.0;
        let mut steps = 0;
        while lever > 0.9 + 1e-9 {
            lever = reduction_slew(throttle_slew(lever, 0.9, 0.2), lever, 0.2);
            steps += 1;
            assert!(lever >= 0.9 - 1e-9);
        }
        assert_eq!(steps, 5, "0.1 of lever a second at 0.2 s ticks");
        // The takeoff's own releases are untouched by it.
        assert_eq!(throttle_slew(1.0, 0.5, 0.2), 0.5);
    }

    /// LFLL-EGLL, 2026-09-17. The 737 lifted at V2+20 and climbed out at
    /// 174 kt. Handed the profile's enroute `vy_kts` of 210 as its climb
    /// reference, the law read that as thirty-six knots slow, took 2,150 fpm
    /// off the rate and commanded 150 fpm — a jet at takeoff thrust told to
    /// stop climbing at 300 ft. It levelled at 1,030 ft and accelerated to
    /// 220 before the follower picked it up. The reference for the INITIAL
    /// climb is the initial climb speed.
    #[test]
    fn the_initial_climb_reference_is_the_speed_the_aircraft_actually_has() {
        // The 737-800's card: V2 150, initial climb 170, enroute Vy 210, and
        // the rate the arming tool hands down (80% of best, clamped: 1500).
        let cap = 1500.0;
        let at_liftoff = climb_vs_protected(174.0, 170.0, cap, Some(340.0));
        assert_eq!(at_liftoff, cap, "at the initial climb speed, full rate");
        // Against the enroute figure the SAME aircraft is floored — the bug,
        // kept here so the two numbers can never be conflated again.
        assert_eq!(
            climb_vs_protected(174.0, 210.0, cap, Some(340.0)),
            150.0,
            "the enroute climb speed floors a jet that is climbing perfectly well"
        );
        // Genuinely slow still trades rate for speed, jet or not.
        let slow = climb_vs_protected(150.0, 170.0, cap, Some(340.0));
        assert!(slow < cap && slow >= 150.0, "{slow}");
    }

    /// A takeoff that ends with the aircraft still configured for takeoff has
    /// not finished. The in-climb schedule waits for the climb speed, which a
    /// jet does not reach before the handoff: flaps 5 rode from Lyon to
    /// FL360 and on to Heathrow's top of descent, the whole climb sitting on
    /// their 250 kt placard (2026-09-17).
    #[test]
    fn the_handoff_never_leaves_the_takeoff_flaps_out() {
        // Already clean: nothing owed, nothing said.
        assert_eq!(clean_up_at_handoff(true, 181.0, Some(167.0)), None);
        // The 737 at its handoff: 181 kt against a 167 kt clean floor.
        assert_eq!(clean_up_at_handoff(false, 181.0, Some(167.0)), Some(true));
        // Too slow to clean up: the flaps STAY, and the caller says so — a
        // retraction here is the stall the schedule exists to avoid.
        assert_eq!(clean_up_at_handoff(false, 160.0, Some(167.0)), Some(false));
        // An airframe with no declared floor has nothing to judge against and
        // cleans up, exactly as it always did.
        assert_eq!(clean_up_at_handoff(false, 70.0, None), Some(true));
    }

    #[test]
    fn slew_limits_and_throttle_ramps() {
        // A demanded hard-over arrives in steps, not a slam.
        assert_eq!(slew(0.0, 0.7, 0.25), 0.25);
        assert_eq!(slew(0.25, 0.7, 0.25), 0.5);
        assert_eq!(slew(0.5, 0.55, 0.25), 0.55, "small changes pass through");
        assert!(
            (slew(0.2, -0.7, 0.25) + 0.05).abs() < 1e-9,
            "both directions"
        );
        // Throttle: rolling at 30% immediately, full by the ramp's end,
        // never over.
        assert_eq!(throttle_ramp(0.0), 0.3);
        let mid = throttle_ramp(THROTTLE_RAMP_S / 2.0);
        assert!(mid > 0.5 && mid < 0.9, "{mid}");
        // Gradual: no more than half the travel in the first two seconds.
        assert!(throttle_ramp(2.0) < 0.6, "{}", throttle_ramp(2.0));
        assert_eq!(throttle_ramp(THROTTLE_RAMP_S), 1.0);
        assert_eq!(throttle_ramp(60.0), 1.0);
    }

    #[test]
    fn climb_law_trades_rate_for_speed_never_descends() {
        // On speed: full rate.
        assert_eq!(climb_vs_fpm(74.0, 74.0, 700.0), 700.0);
        // 10 kt slow: backed off, still climbing.
        let slow = climb_vs_fpm(64.0, 74.0, 700.0);
        assert!((150.0..700.0).contains(&slow), "{slow}");
        // Absurdly slow: pinned at the floor, never a descent.
        assert_eq!(climb_vs_fpm(40.0, 74.0, 700.0), 150.0);
        // A request already UNDER the floor is granted, never a panic:
        // f64::clamp aborts the thread on min > max, and the follower's
        // level-off asked for 145.9 (LXGB to LEMG, 2026-08-02).
        assert_eq!(climb_vs_fpm(155.0, 105.0, 145.9), 145.9);
        assert_eq!(climb_vs_fpm(100.0, 105.0, 100.0), 100.0);
        assert_eq!(climb_vs_fpm(120.0, 105.0, 0.0), 0.0);
        // Fast: capped at the profile's rate, no zoom climb.
        assert_eq!(climb_vs_fpm(95.0, 74.0, 700.0), 700.0);
    }

    /// The climb protected only the SLOW side, which is right for a trainer
    /// (it cannot outrun itself at full power) and wrong for a jet, which can.
    /// The SF50 left Luton at full thrust and accelerated to 270 KIAS against
    /// a Vmo of 250 while the copilot placidly commanded 1500 fpm and never
    /// touched the lever (measured live in the climb, 2026-07-28).
    #[test]
    fn the_climb_protects_the_fast_side_too() {
        const VNE: f64 = 250.0;
        // Well below the guard band: untouched, both rate and thrust.
        assert_eq!(climb_vs_protected(200.0, 120.0, 2000.0, Some(VNE)), 2000.0);
        assert_eq!(climb_throttle(200.0, Some(VNE)), 1.0);
        // Inside the band the aircraft climbs HARDER — excess speed traded
        // for altitude — and the lever comes back with it.
        let fast = climb_vs_protected(245.0, 120.0, 2000.0, Some(VNE));
        assert!(fast > 2000.0, "trade the speed for rate: {fast}");
        let thr = climb_throttle(245.0, Some(VNE));
        assert!(thr < 1.0, "and pull the lever: {thr}");
        // Harder the faster it gets, monotonically, on both.
        assert!(climb_vs_protected(260.0, 120.0, 2000.0, Some(VNE)) > fast);
        assert!(climb_throttle(260.0, Some(VNE)) < thr);
        // The protection can never become a pitch-up or a shutdown of its own.
        let extreme = climb_vs_protected(400.0, 120.0, 2000.0, Some(VNE));
        assert!(
            extreme <= 2000.0 + OVERSPEED_VS_MAX_FPM,
            "capped: {extreme}"
        );
        assert!(climb_throttle(400.0, Some(VNE)) >= CLIMB_THROTTLE_MIN);
        // An airframe that declares no Vne keeps exactly the climb it had.
        assert_eq!(
            climb_vs_protected(260.0, 120.0, 2000.0, None),
            climb_vs_fpm(260.0, 120.0, 2000.0)
        );
        assert_eq!(climb_throttle(260.0, None), 1.0);
        // The 172 is nowhere near its Vne in a climb, so nothing changes for it.
        assert_eq!(
            climb_vs_protected(74.0, 74.0, 700.0, Some(163.0)),
            climb_vs_fpm(74.0, 74.0, 700.0)
        );
        assert_eq!(climb_throttle(74.0, Some(163.0)), 1.0);
    }

    #[test]
    fn the_climb_guard_is_the_250_kt_rule_on_a_jet_and_vne_on_a_trainer() {
        // A330: Vne 330, the band starts at 250 (270 - 20) not 310.
        assert_eq!(climb_guard_vne(Some(330.0)), Some(270.0));
        assert!(
            climb_throttle(273.0, climb_guard_vne(Some(330.0))) < 1.0,
            "273 kt at 1,000 ft is over"
        );
        assert_eq!(climb_throttle(240.0, climb_guard_vne(Some(330.0))), 1.0);
        // 172: Vne 163 stays the reference.
        assert_eq!(climb_guard_vne(Some(163.0)), Some(163.0));
        assert_eq!(climb_guard_vne(None), None);
    }

    #[test]
    fn runway_end_reject_fires_only_slow_and_short() {
        // 200 m left at 30 kt against a 55 kt Vr: no takeoff today.
        assert!(must_reject(200.0, 30.0, 55.0));
        // The Evolution's manual torque: below it the ramp, at it the lever
        // holds, past it the lever comes back; past 150 kt, the ramp again.
        let evo = crate::aircraft::TakeoffPowerProfile { torque_ftlb: 1250.0, until_kias: 150.0 };
        assert_eq!(torque_limited(0.8, 0.5, Some(900.0), 40.0, Some(&evo)), 0.8);
        assert_eq!(torque_limited(0.8, 0.5, Some(1200.0), 40.0, Some(&evo)), 0.5, "held from 95%");
        assert!((torque_limited(0.8, 0.5, Some(1260.0), 40.0, Some(&evo)) - 0.48).abs() < 1e-9);
        let back = torque_limited(0.8, 0.5, Some(1735.0), 40.0, Some(&evo));
        assert!((back - 0.5 * 1250.0 / 1735.0 * 0.98).abs() < 1e-9, "in proportion: {back}");
        assert_eq!(torque_limited(1.0, 0.5, Some(1400.0), 155.0, Some(&evo)), 1.0);
        assert_eq!(torque_limited(1.0, 0.5, None, 40.0, Some(&evo)), 1.0, "no read: the ramp");
        assert_eq!(torque_limited(1.0, 0.5, Some(2000.0), 40.0, None), 1.0, "no limit declared");
        // LILB 03 (2026-09-25): 11 m left at 30 kt is a reject there, not
        // 129 m left at 68 kt at the far end.
        assert_eq!(lateral_excursion(-11.1, -22.3, 29.9), Some("off_centreline"));
        assert_eq!(lateral_excursion(-4.0, -21.0, 21.4), Some("off_heading"));
        assert_eq!(lateral_excursion(-1.5, 14.1, 1.2), None, "the line-up's crawl");
        assert_eq!(lateral_excursion(-3.0, 5.0, 60.0), None);
        // 200 m left but nearly at Vr: keep going, rotation is imminent.
        assert!(!must_reject(200.0, 52.0, 55.0));
        // Plenty of runway, any speed: no reject.
        assert!(!must_reject(900.0, 20.0, 55.0));
    }

    // --- End-to-end sortie ---------------------------------------------------
    //
    // The law tests above each pin one function. This one CLOSES THE LOOP: a
    // coarse airframe model, driven tick-by-tick by the SAME control laws the
    // `fly` loop calls, with reads and control writes routed through a
    // `FakeXPlane` under the real dataref names. It is the automated stand-in
    // for a hand-flown takeoff — nothing else proves the laws COMPOSE into a
    // takeoff that tracks the centreline, rotates, and reaches the AP handoff.
    //
    // What it does NOT cover (by construction): the `fly` orchestration itself
    // (its gates, yields, and reject paths have their own unit tests above),
    // the lat/lon -> cross-track geodesy (autoland's tests own that — the model
    // holds cross-track directly), and real X-Plane physics (the model is a
    // deliberately crude Newtonian sketch). Its ONE job is to fail loudly if a
    // change to the laws stops the aircraft holding the runway — e.g. flipping
    // the `power_lead` sign, or breaking the trim integrator, lets the modelled
    // P-factor walk it off the strip and the cross-track assertion fires.

    /// A deliberately crude single-engine airframe, in a runway-local frame:
    /// heading and cross-track are relative to a course of 0 degrees, right
    /// positive. Not X-Plane — just enough coupling (thrust, a speed-tapered
    /// yaw authority, a left-yawing P-factor moment under power, a first-order
    /// pitch response, and lift past Vr) for the real laws to fly against.
    struct Airframe {
        gs_ms: f64,   // groundspeed
        along_m: f64, // distance rolled from the threshold
        xtk_m: f64,   // cross-track, right positive
        psi_deg: f64, // heading, right of course positive
        theta_deg: f64,
        agl_ft: f64,
    }

    impl Airframe {
        fn lined_up_nose_right() -> Self {
            // A realistic line-up: on the centreline but the nose 6 deg right,
            // the kind of small misalignment the roll law walks out.
            Self {
                gs_ms: 0.0,
                along_m: 0.0,
                xtk_m: 0.0,
                psi_deg: 6.0,
                theta_deg: 0.0,
                agl_ft: 0.0,
            }
        }
        fn ias_kts(&self) -> f64 {
            self.gs_ms * MS_TO_KTS // calm air: IAS == groundspeed
        }
        fn on_ground(&self) -> bool {
            self.agl_ft < 1.0
        }
        /// Advance one tick from the commanded controls (all as written to the
        /// plugin channels): throttle 0..1, steer -1..1 (right positive), pitch
        /// -1..1 (nose-up positive).
        fn step(&mut self, dt: f64, thr: f64, steer: f64, pitch: f64) {
            let gs_kts = self.gs_ms * MS_TO_KTS;
            // Thrust minus a little drag; never below a standstill.
            self.gs_ms = (self.gs_ms + (3.0 * thr - 0.01 * self.gs_ms) * dt).max(0.0);
            // Yaw: steering authority grows with airflow; P-factor yaws left
            // under power and fades as the airflow straightens. The rudder
            // feed-forward and trim integrator in the roll law exist to cancel
            // exactly this — get their sign wrong and the model diverges.
            let steer_yaw = steer * 30.0;
            let pfac = -7.0 * thr * (1.0 - 0.5 * (gs_kts / 60.0).clamp(0.0, 1.0));
            self.psi_deg += (steer_yaw + pfac) * dt;
            let psi_rad = self.psi_deg.to_radians();
            self.xtk_m += self.gs_ms * psi_rad.sin() * dt;
            self.along_m += self.gs_ms * psi_rad.cos() * dt;
            // Pitch: first-order lag toward the attitude the elevator commands.
            let cmd_theta = pitch * 15.0;
            self.theta_deg += (cmd_theta - self.theta_deg) * 2.0 * dt;
            // Lift builds once the nose is up and we're at flying speed — the
            // aircraft leaves the ground on its own, which is what lets the
            // liftoff latch (AGL > 8 ft) ever fire.
            let vs_fpm = if self.theta_deg > 2.0 && self.ias_kts() >= 52.0 {
                (self.theta_deg - 2.0) * 120.0 * (self.ias_kts() / 55.0).min(1.5)
            } else {
                0.0
            };
            self.agl_ft = (self.agl_ft + vs_fpm / 60.0 * dt).max(0.0);
        }
    }

    #[tokio::test]
    async fn sortie_flies_the_roll_rotates_and_reaches_the_ap_handoff() {
        use crate::xplane::FakeXPlane;
        use sim::xplane::XPlane;

        let sim = FakeXPlane::new();
        let course_deg = 0.0;
        let length_m = 2000.0;
        let vr_kts = 55.0;
        let dt = 0.15; // fly's nominal tick

        let mut af = Airframe::lined_up_nose_right();
        let mut steer_law = RollSteer::default();
        let mut sortie_prev_xtk: Option<f64> = None;
        let mut pitch_law = RotatePitch::default();

        // Loop state mirrored from `fly`.
        let mut rotated = false;
        let mut airborne = false;
        let mut last_thr_cmd = 0.0_f64;
        let mut last_err = 0.0_f64;

        // What we assert on.
        let mut max_abs_xtk = 0.0_f64;
        let mut max_abs_xtk_after_capture = 0.0_f64;
        let mut rotate_snapshot: Option<(f64, f64, f64)> = None; // (ias, remaining, err)
        let mut hit_full_throttle = false;
        let mut ticks = 0;

        loop {
            ticks += 1;
            assert!(ticks < 1200, "takeoff never converged (ran {ticks} ticks)");
            let elapsed = ticks as f64 * dt;

            // Publish the world exactly where `fly` reads it, under the real
            // dataref names, then read the flight state back through the trait.
            sim.set_read(GS_MS, af.gs_ms);
            sim.set_read(IAS, af.ias_kts());
            sim.set_read(AGL_M, af.agl_ft / M_TO_FT);
            sim.set_read(PSI_TRUE, af.psi_deg.rem_euclid(360.0));
            sim.set_read(THETA, af.theta_deg);
            sim.set_read(crate::datarefs::ON_GROUND_ANY, if af.on_ground() { 1.0 } else { 0.0 });
            let (gs_ms, ias, agl_m, psi) = tokio::join!(
                sim.get_float(GS_MS),
                sim.get_float(IAS),
                sim.get_float(AGL_M),
                sim.get_float(PSI_TRUE),
            );
            let (gs_ms, ias, agl_ft, psi) = (
                gs_ms.unwrap(),
                ias.unwrap(),
                agl_m.unwrap() * M_TO_FT,
                psi.unwrap(),
            );
            let gs_kts = gs_ms * MS_TO_KTS;

            // Throttle (mirrors `fly`: computed from the PREVIOUS tick's error).
            let thr_want = if airborne {
                1.0
            } else {
                throttle_cmd(elapsed, last_err, gs_kts)
            };
            let thr_cmd = throttle_slew(last_thr_cmd, thr_want, dt);
            sim.set_float(&throttle_dataref(), thr_cmd).await.unwrap();
            last_thr_cmd = thr_cmd;
            hit_full_throttle |= thr_cmd >= 0.99;

            let remaining = length_m - af.along_m;
            if !airborne {
                assert!(remaining > 0.0, "rolled off the end without flying");
                assert!(
                    !must_reject(remaining, ias, vr_kts),
                    "the loop would have rejected here"
                );
            }

            // Steer the centreline (cross-track held by the model, not geodesy).
            let xtk_rate = sortie_prev_xtk.map_or(0.0, |p: f64| (af.xtk_m - p) / dt);
            sortie_prev_xtk = Some(af.xtk_m);
            let target = roll_target_heading(course_deg, af.xtk_m, xtk_rate, gs_kts);
            let err = crate::angles::signed_diff_deg(target, psi);
            last_err = err;
            let steer = steer_law.step(err, dt, gs_kts, thr_cmd);
            assert!(
                steer.abs() <= MAX_ROLL_STEER + 1e-9,
                "steer out of bounds: {steer}"
            );
            sim.set_float(TAXI_STEER, steer).await.unwrap();

            // Rotate, then hold the attitude with the elevator channel.
            if !rotated && ias >= vr_kts && rotate_ok(err, remaining) {
                rotated = true;
                rotate_snapshot = Some((ias, remaining, err));
            }
            let mut pitch = 0.0;
            if rotated {
                pitch = pitch_law.step(af.theta_deg, None, dt);
                sim.set_float(TAKEOFF_PITCH, pitch).await.unwrap();
            }

            // Liftoff latch, same gate as `fly`.
            if !airborne && !af.on_ground() && ias > vr_kts * 0.8 && agl_ft > 8.0 {
                airborne = true;
            }

            max_abs_xtk = max_abs_xtk.max(af.xtk_m.abs());
            if af.along_m > 200.0 {
                max_abs_xtk_after_capture = max_abs_xtk_after_capture.max(af.xtk_m.abs());
            }

            // The AP would take the established climb here; the sortie is done.
            if agl_ft >= AP_ENGAGE_AGL_FT {
                break;
            }

            // Advance the world from what we actually wrote to the channels.
            let steer_a = sim.last_write(TAXI_STEER).unwrap();
            let thr_a = sim.last_write(&throttle_dataref()).unwrap();
            let pitch_a = if rotated {
                sim.last_write(TAKEOFF_PITCH).unwrap()
            } else {
                0.0
            };
            let _ = pitch;
            af.step(dt, thr_a, steer_a, pitch_a);
        }

        // It rotated, at or past Vr, with runway to spare and a straight nose.
        let (r_ias, r_rem, r_err) = rotate_snapshot.expect("never rotated");
        assert!(r_ias >= vr_kts, "rotated below Vr: {r_ias}");
        assert!(
            r_rem > 300.0,
            "rotated with the runway almost gone: {r_rem} m"
        );
        assert!(r_err.abs() < 15.0, "rotated with the nose {r_err} deg off");

        // It held the centreline the whole roll and tracked runway heading to
        // the handoff — no weave, no departure off the strip.
        assert!(
            max_abs_xtk < 15.0,
            "wandered {max_abs_xtk:.1} m off the centreline"
        );
        assert!(
            max_abs_xtk_after_capture < 6.0,
            "did not settle on the centreline: {max_abs_xtk_after_capture:.1} m after capture"
        );
        assert!(
            (af.psi_deg - course_deg).abs() < 8.0,
            "handed off {:.1} deg off runway heading",
            af.psi_deg - course_deg
        );
        assert!(hit_full_throttle, "never reached full power on the roll");

        // The trim integrator learned a right-rudder standing offset — the
        // proof it recognised and cancelled the modelled P-factor rather than
        // just chasing it. Flip `power_lead`'s sign and this (and the
        // cross-track bound) go red.
        assert!(
            steer_law.trim() > 0.0,
            "trim never learned the P-factor: {}",
            steer_law.trim()
        );
    }

    /// The 737's climb-out as the blackbox drew it at EGPH 24 (2026-09-21):
    /// 4.1 kt/s of excess thrust at takeoff power, a knot a second given back
    /// per 1,000 fpm, and a VS mode that is two 1.5 s lags behind 2 s of dead
    /// time. Returns (ias at the acceleration height, flap steps as (agl, ias,
    /// ratio), handoff (agl, ias), VS command reversals below the height).
    fn fly_737_climb_out(
        climb_out: Option<ClimbOut>,
    ) -> (f64, Vec<(f64, f64, f64)>, (f64, f64), usize) {
        let schedule = crate::aircraft::flap_schedule(
            crate::aircraft::load("boeing_737_800")
                .unwrap()
                .flaps
                .as_ref(),
        );
        let (hold_kts, base_fpm, handoff_agl, dt) = (170.0, 1500.0, 1000.0, 0.4);
        let vne = climb_guard_vne(Some(340.0));
        // As the AP took it: 340 ft, 185 kt, climbing 1,600 fpm.
        let (mut agl, mut ias, mut vs1, mut vs) = (340.0_f64, 185.0_f64, 1600.0_f64, 1600.0_f64);
        let mut delay = std::collections::VecDeque::from(vec![1600.0_f64; 5]);
        let (mut flap, mut steps, mut at_accel) = (0.375_f64, Vec::new(), None);
        let mut deployed = flap;
        let (mut reversals, mut last_cmd, mut last_dir) = (0usize, None::<f64>, 0.0_f64);
        let mut waited = None::<f64>;
        for tick in 0..400 {
            let t = tick as f64 * dt;
            let gate = climb_out.map_or(FLAP_RETRACT_AGL_FT, |c| c.accel_agl_ft);
            if flap > 1e-6 && agl > gate {
                if let Some(next) = paced_flap_step(&schedule, flap, Some(deployed), ias) {
                    flap = next;
                    steps.push((agl, ias, next));
                }
            }
            // The flaps follow the handle at about a detent in four seconds.
            deployed += (flap - deployed).clamp(-0.03 * dt, 0.03 * dt);
            let cmd = climb_out_vs_fpm(ias, hold_kts, base_fpm, vne, agl, climb_out);
            if climb_out.is_some_and(|c| agl < c.accel_agl_ft) {
                if let Some(prev) = last_cmd {
                    let dir = (cmd - prev).signum();
                    if (cmd - prev).abs() > 50.0 {
                        if dir == -last_dir {
                            reversals += 1;
                        }
                        last_dir = dir;
                    }
                }
                last_cmd = Some(cmd);
            }
            if at_accel.is_none() && agl >= 1000.0 {
                at_accel = Some(ias);
            }
            if agl >= handoff_agl {
                let w = t - *waited.get_or_insert(t);
                let retracted = flap <= 1e-6;
                if !(climb_out.is_some() && handoff_waits_for_schedule(retracted, true, w)) {
                    return (at_accel.unwrap(), steps, (agl, ias), reversals);
                }
            }
            delay.push_back(cmd);
            let seen = delay.pop_front().unwrap();
            vs1 += (seen - vs1) * dt / 1.5;
            vs += (vs1 - vs) * dt / 1.5;
            ias += (4.1 - 1.06 * vs / 1000.0) * dt;
            agl += vs / 60.0 * dt;
        }
        panic!("never handed off");
    }

    #[test]
    fn the_plant_flies_the_egph_climb_out_under_the_old_law() {
        // 240 kt through 1,000 ft and the flaps gone by 740: the flight.
        let (at_accel, steps, _, _) = fly_737_climb_out(None);
        assert!((230.0..255.0).contains(&at_accel), "{at_accel}");
        assert!(steps.iter().all(|(agl, _, _)| *agl < 1000.0), "{steps:?}");
    }

    #[test]
    fn a_jet_holds_its_climb_speed_to_the_acceleration_height() {
        let c = ClimbOut {
            accel_agl_ft: 1000.0,
            max_fpm: 3000.0,
        };
        let (at_accel, steps, (h_agl, h_ias), reversals) = fly_737_climb_out(Some(c));
        // Handed 185 kt at 340 ft with 3,000 fpm to spend against 3,900 of
        // excess: not V2 + 20, but thirty-five knots under what it was, and
        // still under the flaps-up speed, so the schedule has steps to take.
        assert!(at_accel < 210.0, "{at_accel}");
        assert!(reversals <= 1, "the speed hold hunted: {reversals}");
        // Flaps 5 stays to the height, then 1 and up, each at its own speed
        // and after the flaps have run to the last.
        assert_eq!(steps.len(), 2, "{steps:?}");
        assert!(steps.iter().all(|(agl, _, _)| *agl > 1000.0), "{steps:?}");
        assert!(steps[1].1 >= 210.0 && steps[1].2 <= 1e-6, "{steps:?}");
        // The handoff is clean, inside the wait, and nowhere near 250.
        assert!(h_ias < 235.0 && h_agl < 2500.0, "{h_agl} ft {h_ias} kt");
    }

    #[test]
    fn the_speed_hold_is_a_jets_below_its_height_and_nobody_elses() {
        let c = Some(ClimbOut {
            accel_agl_ft: 1000.0,
            max_fpm: 3000.0,
        });
        // On speed: the base rate. Ten over: a thousand more. Capped at best.
        assert_eq!(
            climb_out_vs_fpm(170.0, 170.0, 1500.0, None, 500.0, c),
            1500.0
        );
        assert_eq!(
            climb_out_vs_fpm(180.0, 170.0, 1500.0, None, 500.0, c),
            2500.0
        );
        assert_eq!(
            climb_out_vs_fpm(200.0, 170.0, 1500.0, None, 500.0, c),
            3000.0
        );
        // Slow is still slow: the rate is backed off, never added to.
        assert_eq!(
            climb_out_vs_fpm(160.0, 170.0, 1500.0, None, 500.0, c),
            900.0
        );
        // At the height, and on a trainer at any height, the old climb.
        assert_eq!(
            climb_out_vs_fpm(200.0, 170.0, 1500.0, None, 1000.0, c),
            1500.0
        );
        assert_eq!(
            climb_out_vs_fpm(90.0, 74.0, 560.0, None, 500.0, None),
            560.0
        );
        // A best rate under the base never LOWERS the climb.
        let low = Some(ClimbOut {
            accel_agl_ft: 1000.0,
            max_fpm: 0.0,
        });
        assert_eq!(
            climb_out_vs_fpm(200.0, 170.0, 1500.0, None, 500.0, low),
            1500.0
        );
    }

    #[test]
    fn a_circuit_keeps_the_old_climb_out_whole() {
        let _g = CLIMB_OUT_TEST.lock().unwrap_or_else(|e| e.into_inner());
        publish_climb_out(Some(ClimbOut {
            accel_agl_ft: 1000.0,
            max_fpm: 3000.0,
        }));
        assert!(climb_out_for(DEFAULT_HANDOFF_AGL_FT).is_some());
        assert!(climb_out_for(500.0).is_none());
        publish_climb_out(None);
        assert!(climb_out_for(DEFAULT_HANDOFF_AGL_FT).is_none());
    }

    static CLIMB_OUT_TEST: std::sync::Mutex<()> = std::sync::Mutex::new(());

    #[test]
    fn the_jets_declare_a_climb_out_and_the_trainer_does_not() {
        for (id, want) in [
            ("boeing_737_800", true),
            ("airbus_a330_300", true),
            ("cessna_172sp", false),
        ] {
            let Some(p) = crate::aircraft::load(id) else {
                continue;
            };
            assert_eq!(ClimbOut::from_profile(&p).is_some(), want, "{id}");
        }
    }

    /// #150, replayed: on both recorded LEMG 13 circuits the pitch was
    /// climbing 2,300 to 2,500 fpm through 300 ft and the engage wrote 1,500.
    /// The engage now writes the climb flown, a slew step from it at most,
    /// whether the law it walks to is the circuit's or a jet's speed hold.
    #[test]
    fn the_engage_takes_the_climb_the_pitch_was_flying() {
        let recorded = climb_out_bench::load();
        assert_eq!(recorded.len(), 2);
        let jet = Some(ClimbOut {
            accel_agl_ft: 1000.0,
            max_fpm: 3000.0,
        });
        let vne = climb_guard_vne(Some(340.0));
        let dt = 0.4;
        for r in &recorded {
            let flown = climb_out_bench::flown_fpm(&r.manual);
            let (_, agl, ias, cmd) = r.engage;
            assert!(
                flown - cmd > 700.0,
                "{}: flown {flown:.0}, the engage wrote {cmd:.0}",
                r.sortie
            );
            // The circuit's takeoff (no climb-out below 500 ft): the law is
            // the old rate, the command starts at the climb flown.
            let (law, first) =
                ClimbVs::default().step(ias, 170.0, 1500.0, vne, agl, None, Some(flown), dt, false);
            assert_eq!(law, 1500.0, "{}", r.sortie);
            assert!(
                (first - flown).abs() <= CLIMB_VS_SLEW_FPM_S * dt + 1e-6,
                "{}: flown {flown:.0}, first {first:.0}",
                r.sortie
            );
            // A departure's speed hold starts where the pitch is.
            let (law, first) =
                ClimbVs::default().step(ias, 170.0, 1500.0, vne, agl, jet, Some(flown), dt, false);
            assert!((law - flown).abs() < 1.0, "{}: law {law:.0}", r.sortie);
            assert!((first - flown).abs() < 1.0, "{}", r.sortie);
            // A failed read is no measurement: the law, as it always was.
            let (law, first) =
                ClimbVs::default().step(ias, 170.0, 1500.0, vne, agl, None, None, dt, false);
            assert_eq!((law, first), (1500.0, 1500.0));
        }
    }

    /// #150 with #78: the pitch holds V2 + 15 to 300 ft (165 kt, all the
    /// excess in the path: about 3,090 fpm on #78's LEMG 13 fit). Engaged on
    /// the old proportional law the dial read 1,200 (five knots slow) and the
    /// 737 ran to the acceleration height well past V2 + 20; the speed hold
    /// seeded at the climb flown keeps the band through the 737's VS mode
    /// (2 s dead, two 1.5 s lags), then lowers the nose at the height at a
    /// crew's pace.
    #[test]
    fn a_jet_takes_the_held_climb_and_keeps_its_speed_to_the_height() {
        let (tw, dt) = (0.185, 0.4);
        let c = Some(ClimbOut {
            accel_agl_ft: 1000.0,
            max_fpm: 3000.0,
        });
        let vne = climb_guard_vne(Some(340.0));
        let fly = |new: bool| {
            let (mut agl, mut ias) = (300.0_f64, 165.0_f64);
            let flown = climb_out_bench::held_climb_fpm(tw, ias);
            let mut ap = climb_out_bench::VsMode::new(flown, dt);
            let mut law = ClimbVs::default();
            let (mut prev, mut max_step) = (flown, 0.0_f64);
            let (mut lo, mut hi, mut at_height) = (ias, ias, None);
            let mut reversals = (0usize, 0.0_f64);
            while agl < 1300.0 {
                let cmd = if new {
                    law.step(ias, 170.0, 1500.0, vne, agl, c, Some(ap.vs), dt, false).1
                } else {
                    climb_out_vs_fpm(ias, 170.0, 1500.0, vne, agl, c)
                };
                let step = cmd - prev;
                max_step = max_step.max(step.abs());
                if agl < 1000.0 && step.abs() > 20.0 {
                    if step.signum() == -reversals.1 {
                        reversals.0 += 1;
                    }
                    reversals.1 = step.signum();
                }
                prev = cmd;
                let vs = ap.step(cmd, dt);
                ias += climb_out_bench::accel_kts_s(tw, ias, vs) * dt;
                agl += vs / 60.0 * dt;
                if agl < 1000.0 {
                    lo = lo.min(ias);
                    hi = hi.max(ias);
                } else if at_height.is_none() {
                    at_height = Some(ias);
                }
            }
            (max_step, lo, hi, at_height.unwrap(), reversals.0)
        };
        let (old_step, _, old_hi, old_at, _) = fly(false);
        assert!(old_step > 1500.0, "the old engage pushes: {old_step:.0}");
        assert!(
            old_hi > 185.0 && old_at > 185.0,
            "the old law runs away: {old_hi:.1}"
        );
        let (step, lo, hi, at, reversals) = fly(true);
        assert!(
            step <= CLIMB_VS_SLEW_FPM_S * dt + 1e-6,
            "the command stepped {step:.0} fpm in a tick"
        );
        assert!(
            lo >= 160.0 && hi <= 180.0,
            "{lo:.1} to {hi:.1} below the height"
        );
        assert!((160.0..=180.0).contains(&at), "at the height {at:.1}");
        assert!(reversals <= 1, "the speed hold hunted: {reversals}");
    }

    /// #226, LEGR 2026-10-06 (sortie 1791274391-full_auto): past the 1,000
    /// ft acceleration height the 737 went back to its 3,000 fpm, gained
    /// 0.4 kt a second and stepped flaps 5 to 1 at 3,066 ft (190 kt). On
    /// the same plant (old law: 190 at 3,580 ft) the acceleration segment
    /// lowers the nose at a crew's pace and makes 190 at 1,575 ft and the
    /// 210 kt flaps-up speed at 1,743; clean, it climbs at the full rate.
    #[test]
    fn a_jet_accelerates_at_the_height_until_clean() {
        let (tw, dt) = (0.185, 0.4);
        let c = Some(ClimbOut {
            accel_agl_ft: 1000.0,
            max_fpm: 3000.0,
        });
        let vne = climb_guard_vne(Some(340.0));
        // Altitude at which each speed is first reached, flying from the
        // height at the LEGR numbers (171 kt, 3,000 fpm).
        let fly = |segment: bool| {
            let (mut agl, mut ias) = (1000.0_f64, 171.0_f64);
            let mut ap = climb_out_bench::VsMode::new(3000.0, dt);
            let mut law = ClimbVs::default();
            let (mut at190, mut at210, mut max_step, mut prev) = (None, None, 0.0_f64, 3000.0);
            while agl < 4000.0 && at210.is_none() {
                let clean = at210.is_some();
                let accel = segment && accelerating(c, agl, clean);
                let cmd = law.step(ias, 170.0, 3000.0, vne, agl, c, Some(ap.vs), dt, accel).1;
                max_step = max_step.max((cmd - prev).abs());
                prev = cmd;
                let vs = ap.step(cmd, dt);
                ias += climb_out_bench::accel_kts_s(tw, ias, vs) * dt;
                agl += vs / 60.0 * dt;
                if ias >= 190.0 && at190.is_none() {
                    at190 = Some(agl);
                }
                if ias >= 210.0 {
                    at210 = Some(agl);
                }
            }
            (at190.unwrap_or(f64::INFINITY), at210.unwrap_or(f64::INFINITY), max_step)
        };
        let (old190, _, _) = fly(false);
        assert!(old190 > 2500.0, "the old climb reaches 190 at {old190:.0} ft");
        let (at190, at210, step) = fly(true);
        assert!(at190 < 1700.0, "190 kt at {at190:.0} ft");
        assert!(at210 < 2000.0, "210 kt at {at210:.0} ft");
        assert!(step <= CLIMB_VS_SLEW_FPM_S * dt + 1e-6, "stepped {step:.0} fpm");
        // The segment is the flaps': clean, or before the height, or a
        // SID's best-rate climb (no finite height), it is the old law.
        assert!(!accelerating(c, 1200.0, true));
        assert!(!accelerating(c, 900.0, false));
        let sid = Some(ClimbOut { accel_agl_ft: f64::INFINITY, max_fpm: 3000.0 });
        assert!(!accelerating(sid, 5000.0, false));
        assert_eq!(climb_accel_fpm(c.unwrap()), 1200.0);
    }

    /// #226: the engage pressed servos and VS six times in 0.7 s while the
    /// 737's annunciators caught up; a press now settles first.
    #[test]
    fn the_engage_lets_a_press_settle() {
        let t0 = std::time::Instant::now();
        assert!(engage_may_press(None, t0));
        assert!(!engage_may_press(Some(t0), t0 + std::time::Duration::from_millis(100)));
        assert!(engage_may_press(Some(t0), t0 + std::time::Duration::from_millis(1600)));
    }

    /// #150: the circuit takes the aircraft at 500 ft and walks the schedule
    /// itself; a departure (or a clean airframe, or one with no schedule)
    /// still cleans up at the handoff as it always did. The window leaves V2
    /// for the flaps-up speed a crew bugs, only where the takeoff set V2.
    #[tokio::test]
    async fn the_circuit_takes_the_flaps_and_the_window_leaves_v2() {
        assert!(leaves_flaps_to_caller(false, true, true));
        assert!(!leaves_flaps_to_caller(true, true, true), "clean already");
        assert!(
            !leaves_flaps_to_caller(false, false, true),
            "a departure cleans up"
        );
        assert!(
            !leaves_flaps_to_caller(false, true, false),
            "no schedule to walk"
        );

        let b738 = crate::aircraft::load("boeing_737_800").expect("737 profile loads");
        let sched = crate::aircraft::flap_schedule(b738.flaps.as_ref());
        assert_eq!(
            clean_up_speed_kts(&sched, 210.0),
            210.0,
            "the 737's flaps-up speed"
        );
        assert_eq!(
            clean_up_speed_kts(&[], 74.0),
            74.0,
            "no schedule: the climb speed"
        );
        assert!(
            b738.takeoff_config
                .as_ref()
                .and_then(|t| t.v2_kts)
                .is_some(),
            "the 737's takeoff sets V2, so its window is ours to move"
        );

        use crate::xplane::FakeXPlane;
        const WINDOW: &str = crate::aircraft::TakeoffConfigProfile::SPEED_WINDOW;
        let xp = FakeXPlane::with_reads(&[(WINDOW, 150.0)]);
        // The fake never takes a write: written, and read back as not taken.
        assert!(!speed_window_after_acceleration(&xp, 210.0).await);
        assert_eq!(xp.last_write(WINDOW), Some(210.0));
        xp.set_read(WINDOW, 210.0);
        assert!(speed_window_after_acceleration(&xp, 210.0).await);
    }

    #[test]
    fn a_brake_steered_roll_uses_the_brakes_until_the_rudder_bites() {
        // EDLP 24: full left steer at 10-21 kt with nothing answering.
        let (l, r) = super::roll_brake_split(-0.85, 12.0);
        assert!(l > 0.2 && r == 0.0, "inside (left) brake: {l} {r}");
        let (l30, _) = super::roll_brake_split(-0.85, 30.0);
        assert!((l30 - 0.5 * l).abs() < 1e-9, "half by 30 kt: {l30}");
        assert_eq!(super::roll_brake_split(-0.85, 45.0), (0.0, 0.0), "the rudder's above 40");
        assert_eq!(super::roll_brake_split(0.02, 12.0), (0.0, 0.0), "no brake for a breath of steer");
    }

    #[test]
    fn a_crooked_roll_start_pivots_on_the_inside_brake() {
        // LIME 28: 21 degrees right of the nose at 1 kt, steer 0.17, no brake.
        let (l, r) = super::roll_brake_split_aligning(0.17, 0.9, 21.0, 0.0);
        // The soft aligning PD (#46): 21 of 40 degrees of led error.
        assert!(r > 0.2 && l == 0.0, "right brake, from a standstill: {l} {r}");
        // Proportional: small errors get less.
        let (_, r6) = super::roll_brake_split_aligning(0.17, 0.9, 6.0, 0.0);
        assert!(r6 < r, "{r6} < {r}");
        // Aligned: the ordinary split, nothing added.
        assert_eq!(
            super::roll_brake_split_aligning(0.17, 0.9, 2.0, 0.0),
            super::roll_brake_split(0.17, 0.9)
        );
        // Already turning right fast: the rate ceiling lets go.
        assert_eq!(super::roll_brake_split_aligning(0.17, 0.9, 21.0, 30.0).1, 0.0);
        // Rolling fast: the rudder's, not a pivot.
        assert_eq!(
            super::roll_brake_split_aligning(0.17, 16.0, 21.0, 0.0),
            super::roll_brake_split(0.17, 16.0)
        );
    }

    /// The roll's power waits for the nose on a brake-steered airframe. The
    /// first roll ticks of 2026-09-29: the SR22 at LEMG 12 10.8 degrees off
    /// at 0.07 kt (1790690995, t 1790692260.5), the lever then walked 0.3 to
    /// 0.5 through a +/-12 degree pirouette; the Evolution at LILB 03 24.6 off
    /// at 2.6 kt (1790712974, t 1790713247.3), 0.32 to 0.59 through a swing
    /// to -4.4 at 21.6 deg/s. Runway starts roll 0.3 to 0.5 off: no wait.
    #[test]
    fn a_crooked_roll_start_holds_the_power_until_the_nose_is_straight() {
        use super::{aligning_throttle, RollAlign, RollAlignState};
        let mut a = RollAlign::default();
        assert_eq!(a.step(10.8, 0.1, 0.07, 0.15), RollAlignState::Hold);
        assert_eq!(aligning_throttle(0.41, true), 0.3, "the pivot's power, not the ramp's");
        let mut b = RollAlign::default();
        assert_eq!(b.step(24.6, -4.8, 2.6, 0.15), RollAlignState::Hold);
        // Straight and steady: the power comes up, and never waits again.
        assert_eq!(a.step(1.2, 0.5, 0.3, 0.4), RollAlignState::Go);
        assert_eq!(a.step(12.0, 20.0, 9.0, 0.4), RollAlignState::Go);
        assert_eq!(aligning_throttle(0.8, false), 0.8);
        // A runway start is straight at once.
        let mut c = RollAlign::default();
        assert_eq!(c.step(0.4, 0.0, 0.0, 0.15), RollAlignState::Go);
        assert!(c.held_s() == 0.0 && c.done());
        // Rolling already (past a walking pace): the roll law's.
        let mut d = RollAlign::default();
        assert_eq!(d.step(8.0, 0.0, 7.0, 0.15), RollAlignState::Go);
        // A nose that will not come round: a reject, said, after the bound.
        let mut e = RollAlign::default();
        let mut t = 0.0;
        let end = loop {
            let s = e.step(10.8, 0.0, 0.1, 0.42);
            if s != RollAlignState::Hold {
                break s;
            }
            t += 0.42;
        };
        assert_eq!(end, RollAlignState::Failed);
        assert!((t - super::ROLL_ALIGN_MAX_S).abs() < 0.5, "{t}");
    }

    /// The roll start closed on the fitted plant ([`crate::autotaxi::yaw_plant`]):
    /// the aligning split at the pivot's power from the SR22's 10.8 and the
    /// Evolution's 24.6 degrees lets the power up with the nose straight and
    /// swings no more than 2 degrees through. As flown the SR22 swung to
    /// -11.6 in 3 s and the Evolution to -4.4.
    #[test]
    fn a_crooked_roll_start_comes_straight_before_the_power() {
        use crate::autotaxi::yaw_plant::{signed, Plant};
        use super::{RollAlign, RollAlignState};
        for e0 in [10.8, 24.6] {
            for tick in [0.3, 0.42, 0.5] {
                let mut p = Plant::new(140.0, 3.0, e0, 0.0);
                let mut a = RollAlign::default();
                let mut through = 0.0_f64;
                let mut t = 0.0;
                loop {
                    match a.step(p.err, p.yaw, 1.0, tick) {
                        RollAlignState::Hold => {}
                        RollAlignState::Go => break,
                        RollAlignState::Failed => panic!("{e0} at {tick}: never straight"),
                    }
                    let lr = super::roll_brake_split_aligning(0.1, 1.0, p.err, p.yaw);
                    p.run(signed(lr), 1.0, tick);
                    through = through.max(-p.err * e0.signum());
                    t += tick;
                }
                assert!(p.err.abs() <= crate::autotaxi::ALIGNED_DEG, "{e0} at {tick}: {}", p.err);
                assert!(through <= 2.0, "{e0} at {tick}: {through:.1} through");
                assert!(t < 12.0, "{e0} at {tick}: {t:.1} s");
            }
        }
    }

    /// LILB 03 Evolution (1790732094, 2026-09-30), the roll's first 18 ticks:
    /// (dt, err to the target, yaw deg/s, gs kt, lever, bl, br) as logged.
    /// Held at 0.3 of lever the PT6 ran it from 0.2 to 6.6 kt; the 6 kt gate
    /// let the power up at err 4.39 (`roll_aligned` held_s 5.06), and the
    /// roll rejected at 68 kt, 9.4 m off.
    #[rustfmt::skip]
    const LILB_ROLL_HOLD: &[(f64, f64, f64, f64, f64, f64, f64)] = &[
        (0.150, 16.59, -0.31, 0.16, 0.30, 0.000, 0.194), (0.329, 16.36, 1.64, 0.04, 0.30, 0.000, 0.147),
        (0.433, 15.58, 1.87, 0.08, 0.30, 0.000, 0.133), (0.338, 14.83, 2.53, 0.16, 0.30, 0.000, 0.110),
        (0.456, 13.40, 3.80, 0.29, 0.30, 0.000, 0.065), (0.323, 11.97, 4.99, 0.40, 0.30, 0.000, 0.022),
        (0.427, 9.60, 5.89, 0.72, 0.30, 0.025, 0.000), (0.301, 8.02, 4.54, 1.11, 0.30, 0.012, 0.000),
        (0.442, 6.59, 2.22, 1.86, 0.30, 0.000, 0.024), (0.327, 6.06, 1.47, 2.46, 0.30, 0.000, 0.035),
        (0.411, 5.52, 1.32, 3.23, 0.30, 0.000, 0.032), (0.315, 5.16, 1.12, 3.85, 0.30, 0.000, 0.033),
        (0.425, 4.78, 0.81, 4.74, 0.30, 0.000, 0.035), (0.383, 4.55, 0.52, 5.59, 0.30, 0.000, 0.039),
        (0.429, 4.39, 0.36, 6.56, 0.30, 0.000, 0.041), (0.313, 4.32, 0.35, 7.28, 0.38, 0.000, 0.041),
        (0.403, 4.18, 0.44, 8.24, 0.43, 0.000, 0.037), (0.326, 4.10, 0.20, 9.10, 0.47, 0.000, 0.042),
    ];

    /// The hold holds the aircraft: through the logged ticks it never lets
    /// the power up on speed; over the creep the lever is idle and both
    /// brakes on; held past a runaway it is a reject, never a roll.
    #[test]
    fn a_roll_hold_holds_the_aircraft_and_never_lets_go_on_speed() {
        use super::{RollAlign, RollAlignState};
        let mut a = RollAlign::default();
        let mut braked_from = None;
        for &(dt, err, yaw, gs, thr, _, _) in LILB_ROLL_HOLD {
            let (cap, brake) = a.creep(gs);
            let s = a.step(err, yaw, gs, dt);
            assert_eq!(s, RollAlignState::Hold, "at {gs} kt, err {err}: the flown law said Go at 6.56");
            if gs > 3.0 {
                assert_eq!(cap, 0.0, "over the creep: idle, not {thr}");
                assert!(brake >= super::ROLL_ALIGN_BRAKE_MIN, "{gs} kt: {brake}");
                braked_from.get_or_insert(gs);
            } else if gs <= 2.0 {
                assert_eq!(brake, 0.0);
            }
        }
        assert!(braked_from.unwrap() < 3.3, "{braked_from:?}");
        // Back under the creep: the pivot's power, brakes off.
        assert_eq!(a.creep(1.9), (super::ROLL_PIVOT_THR, 0.0));
        // Held past a runaway all the same: a reject on the ground.
        let mut r = RollAlign::default();
        assert_eq!(r.step(10.0, 0.0, 1.0, 0.3), RollAlignState::Hold);
        assert_eq!(r.step(10.0, 0.0, super::ROLL_ALIGN_RUNAWAY_KTS + 0.5, 0.3), RollAlignState::Failed);
        // The brake rides on the pivot's split, both wheels.
        let (l, rr) = super::with_hold_brake((0.0, 0.1), 0.2);
        assert!((l - 0.2).abs() < 1e-9 && (rr - 0.3).abs() < 1e-9);
        assert_eq!(super::aligning_throttle_capped(0.9, true, 0.0), 0.0);
        assert_eq!(super::aligning_throttle_capped(0.9, false, 0.0), 0.9);
    }

    /// Closed on the yaw plant ([`crate::autotaxi::yaw_plant`]) from the
    /// logged roll start (16.6 off, 0.16 kt), with the Evolution's speed as
    /// flown (0.3 of lever built 1.2 kt/s, idle thrust 0.2 kt/s) and the
    /// propeller's standing yaw (0.03 of brake left, what held the logged
    /// creep 4 to 5 off): the hold creeps under 3.5 kt, trims the standing
    /// yaw out and lets the power up straight inside the bound. The flown
    /// law on the same plant let it up at 6 kt still off.
    #[test]
    fn a_roll_hold_creeps_the_nose_straight_before_the_power() {
        use crate::autotaxi::yaw_plant::{signed, Plant};
        use super::{RollAlign, RollAlignState};
        const BIAS: f64 = -0.03;
        let speed = |gs: f64, lever: f64, both: f64, dt: f64| {
            (gs + (lever * 4.0 + 0.2 - both * 6.0) * dt).max(0.0)
        };
        for tick in [0.3, 0.42] {
            let mut p = Plant::new(140.0, 3.0, 16.59, -0.31);
            let mut a = RollAlign::default();
            let (mut gs, mut lever, mut t, mut peak) = (0.16_f64, 0.3_f64, 0.0_f64, 0.0_f64);
            let end = loop {
                let (cap, both) = a.creep(gs);
                match a.step(p.err, p.yaw, gs, tick) {
                    RollAlignState::Hold => {}
                    other => break other,
                }
                lever = super::throttle_slew(lever, cap.min(super::ROLL_PIVOT_THR), tick);
                let split = super::roll_brake_split_aligning(0.1, gs, p.err, p.yaw);
                let split = a.trimmed(split, p.err, p.yaw, gs, tick);
                let (l, r) = super::with_hold_brake(split, both);
                p.run(signed((l, r)) + BIAS, (gs / 2.0).clamp(0.3, 1.0), tick);
                gs = speed(gs, lever, both, tick);
                peak = peak.max(gs);
                t += tick;
            };
            assert_eq!(end, RollAlignState::Go, "{tick}: {t:.1} s, err {:.1}", p.err);
            assert!(p.err.abs() <= crate::autotaxi::ALIGNED_DEG, "{tick}: {:.1}", p.err);
            assert!(peak < 3.5, "{tick}: crept to {peak:.1} kt");
            // The flown law: 0.3 of lever to 6 kt, the pivot alone.
            let mut p = Plant::new(140.0, 3.0, 16.59, -0.31);
            let (mut gs, mut t) = (0.16_f64, 0.0_f64);
            while gs < 6.0 && t < 20.0 {
                let split = super::roll_brake_split_aligning(0.1, gs, p.err, p.yaw);
                p.run(signed(split) + BIAS, (gs / 2.0).clamp(0.3, 1.0), tick);
                gs = speed(gs, 0.3, 0.0, tick);
                t += tick;
            }
            assert!(p.err.abs() > crate::autotaxi::ALIGNED_DEG, "{tick}: flown law {:.1}", p.err);
        }
    }

    #[test]
    fn a_jet_takes_no_propeller_power_lead() {
        // LOWI 26 (2026-09-24): full power, straight, parked: the propeller
        // law leads right rudder, the jet law nothing.
        let mut prop = super::RollSteer::default();
        let mut jet = super::RollSteer { jet: true, ..Default::default() };
        let (mut p, mut j) = (0.0, 0.0);
        for _ in 0..20 {
            p = prop.step(0.0, 0.2, 5.0, 1.0);
            j = jet.step(0.0, 0.2, 5.0, 1.0);
        }
        assert!(p > 0.05, "prop leads right: {p}");
        assert!(j.abs() < 1e-9, "jet leads nothing: {j}");
    }

    /// A sim whose frames run on while the Web API answers: every request is
    /// served `frames_per_request` frames after the one before it, as the
    /// API serves each request from whichever flight-loop frame it lands in.
    /// The aircraft flies `truth` (time s, along m, cross-track m right of
    /// `course`) from `thr`. `plugin` is a plugin that publishes
    /// `xcopilot/position`, copied in the frame its request lands in.
    struct FrameSim {
        thr: (f64, f64),
        course: f64,
        truth: Vec<(f64, f64, f64)>,
        t0: std::sync::Mutex<f64>,
        frame: std::sync::atomic::AtomicU64,
        frames_per_request: u64,
        plugin: bool,
    }

    const SIM_FPS: f64 = 30.0;

    impl FrameSim {
        /// The aircraft at time `t`: (lat, lon), groundspeed m/s.
        fn at(&self, t: f64) -> ((f64, f64), f64) {
            let k = self
                .truth
                .windows(2)
                .position(|w| t < w[1].0)
                .unwrap_or(self.truth.len() - 2);
            let (a, b) = (self.truth[k], self.truth[k + 1]);
            let f = ((t - a.0) / (b.0 - a.0)).clamp(0.0, 1.5);
            let along = a.1 + f * (b.1 - a.1);
            let xtk = a.2 + f * (b.2 - a.2);
            let gs = (b.1 - a.1) / (b.0 - a.0);
            let on_line = project_latlon(self.thr.0, self.thr.1, self.course, along / M_PER_NM);
            let right = (self.course + 90.0).rem_euclid(360.0);
            (project_latlon(on_line.0, on_line.1, right, xtk / M_PER_NM), gs)
        }
        /// The time the next request is served at.
        fn next_t(&self) -> f64 {
            let f = self
                .frame
                .fetch_add(self.frames_per_request, std::sync::atomic::Ordering::SeqCst);
            *self.t0.lock().unwrap() + f as f64 / SIM_FPS
        }
        /// Start a tick's reads at `t`.
        fn tick_at(&self, t: f64) {
            *self.t0.lock().unwrap() = t;
            self.frame.store(0, std::sync::atomic::Ordering::SeqCst);
        }
    }

    #[async_trait::async_trait]
    impl crate::xplane::XPlane for FrameSim {
        async fn get_float(&self, name: &str) -> anyhow::Result<f64> {
            let ((lat, lon), gs) = self.at(self.next_t());
            Ok(match name {
                LAT => lat,
                LON => lon,
                PSI_TRUE => self.course,
                GS_MS => gs,
                _ => 0.0,
            })
        }
        async fn get_bytes(&self, name: &str) -> anyhow::Result<Vec<u8>> {
            anyhow::ensure!(
                self.plugin && name == sim::position::POSITION,
                "dataref not found: {name}"
            );
            let t = self.next_t();
            let ((lat, lon), gs) = self.at(t);
            Ok(sim::position::encode(&sim::position::Position {
                lat,
                lon,
                elev_m: 0.0,
                psi_true_deg: self.course,
                gs_ms: gs,
                vs_fpm: 0.0,
                agl_m: 0.0,
                frame: Some((t * SIM_FPS).round() as u64),
                sim_time_s: Some(t),
                source: sim::position::Source::Plugin,
            }))
        }
        async fn set_float(&self, _: &str, _: f64) -> anyhow::Result<()> {
            Ok(())
        }
        async fn set_int(&self, _: &str, _: i64) -> anyhow::Result<()> {
            Ok(())
        }
        async fn run_command(&self, _: &str) -> anyhow::Result<()> {
            Ok(())
        }
        async fn run_command_for(&self, _: &str, _: f64) -> anyhow::Result<()> {
            Ok(())
        }
        async fn get_float_array(&self, _: &str) -> anyhow::Result<Vec<f64>> {
            Ok(vec![])
        }
        async fn set_int_array_elem(&self, _: &str, _: usize, _: i64) -> anyhow::Result<()> {
            Ok(())
        }
        async fn set_float_array_elem(&self, _: &str, _: usize, _: f64) -> anyhow::Result<()> {
            Ok(())
        }
    }

    /// #157: latitude and longitude read as two requests pair two frames. The
    /// LEMG 13 roll of 1790591256 (every `roll` tick, its groundspeed and
    /// cross-track as the truth, on 13's diagonal) is flown through a sim
    /// that serves each request two frames after the last (30 fps), and each
    /// tick is read the way the roll reads it. Split, the cross-track the law
    /// is handed is off the aircraft's own by up to 3.0 m above 120 kt (the
    /// 2.9 m flown), a different error every tick; from the plugin's
    /// one-frame snapshot it is the aircraft's own, every tick. On EGKK 08R's
    /// east-west strip the same lag is under half of it.
    #[tokio::test]
    async fn one_frames_position_never_pairs_two_frames() {
        async fn worst(
            ticks: &[serde_json::Value],
            course: f64,
            plugin: bool,
        ) -> (f64, sim::position::Source) {
            // Along-track from the logged groundspeed, the logged
            // cross-track as the aircraft's.
            let mut truth = Vec::new();
            let (mut along, mut last_t) = (0.0, None::<f64>);
            for r in ticks {
                let t = r["t"].as_f64().unwrap();
                let gs_ms = r["gs"].as_f64().unwrap() / MS_TO_KTS;
                if let Some(t0) = last_t {
                    along += gs_ms * (t - t0);
                }
                last_t = Some(t);
                truth.push((t, along, r["xtk_m"].as_f64().unwrap()));
            }
            let sim = FrameSim {
                thr: (36.6893, -4.5147),
                course,
                truth: truth.clone(),
                t0: std::sync::Mutex::new(0.0),
                frame: std::sync::atomic::AtomicU64::new(0),
                frames_per_request: 2,
                plugin,
            };
            let rwy = Runway {
                icao: "TEST".into(),
                id: "13".into(),
                thr_lat: sim.thr.0,
                thr_lon: sim.thr.1,
                course_deg: course,
                length_m: 3200.0,
                surface: 1,
                dist_nm: 0.0,
                elev_ft: 0.0,
            };
            let (mut worst, mut source) = (0.0_f64, sim::position::Source::Split);
            for (r, (t, _, _)) in ticks.iter().zip(&truth) {
                if r["gs"].as_f64().unwrap() < 120.0 {
                    continue;
                }
                sim.tick_at(*t);
                let pos = sim::position::read(&sim).await.unwrap();
                source = pos.source;
                let read_m = cross_track_nm(pos.lat, pos.lon, &rwy) * M_PER_NM;
                // The aircraft's own cross-track in the frame the reading
                // began in.
                let ((lat, lon), _) = sim.at(*t);
                let own_m = cross_track_nm(lat, lon, &rwy) * M_PER_NM;
                worst = worst.max((read_m - own_m).abs());
            }
            (worst, source)
        }
        let rolls = rolls_0928();
        let (_, lemg) = rolls
            .iter()
            .find(|(id, _)| id.starts_with("1790591256"))
            .expect("the LEMG 13 roll");
        // LEMG 13 true course by the rolls' heading on the line.
        let (split, src) = worst(lemg, 132.2, false).await;
        assert_eq!(src, sim::position::Source::Split);
        assert!(
            split > 1.5,
            "split reads should pair two frames on the diagonal: {split:.2} m"
        );
        let (snap, src) = worst(lemg, 132.2, true).await;
        assert_eq!(src, sim::position::Source::Plugin);
        assert!(
            snap < 0.005,
            "one frame's position is the aircraft's: {snap:.4} m"
        );
        // The same lag on EGKK 08R's course: under half the diagonal's
        // (1.2 m against 3.0 here).
        let (east_west, _) = worst(lemg, 77.6, false).await;
        assert!(
            east_west < 0.5 * split,
            "{east_west:.2} m east-west vs {split:.2} diagonal"
        );
    }

/// A lever read below our command is a hand only when the declared hardware
/// lever moved too: the Zibo's snap to a still hardware lever handed a
/// standing aircraft to nobody at LEMG (2026-10-07).
#[test]
fn a_lever_snap_with_the_hardware_still_is_no_hand() {
    use super::pilot_moved_throttle as hand;
    assert!(hand(false, None, None), "undeclared: the lever read decides");
    assert!(!hand(true, Some(0.284), Some(0.284)));
    assert!(!hand(true, Some(0.284), Some(0.33)));
    assert!(hand(true, Some(0.284), Some(0.05)));
    assert!(!hand(true, None, Some(0.05)), "no baseline is no measurement");
    assert!(!hand(true, Some(0.284), None), "a failed read is no measurement");
}
