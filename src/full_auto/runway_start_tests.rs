    use super::*;

    /// LILB 03 Evolution (1790732094, 2026-09-30): the turn-round's drive
    /// ended `stopped_off_line` 5.6 m off and 12.4 degrees crooked on a 15 m
    /// strip, and the fit (40 m, 20 degrees) passed it: full auto said
    /// "Lined up. Flying the circuit." 2 s later and rolled. Now that is a
    /// retry, then a give-up, said. The flight before (1790724857) aligned
    /// 1.5 m off and 1.3 degrees and goes on.
    #[test]
    fn a_line_up_the_drive_did_not_finish_never_rolls() {
        let miss = judge_lined_up(true, false, -5.6, 12.4, Some(15.0));
        assert_eq!(miss, Err(LineUpMiss::NotArrived { lateral_m: 5.6, err_deg: 12.4 }));
        assert_eq!(
            line_up_miss_words(&miss.unwrap_err()),
            "Not lined up: 6 metres off the centreline and 12 degrees off the runway."
        );
        assert_eq!(judge_lined_up(true, true, -1.52, 1.3, Some(15.0)), Ok(()));
        // Arrived, but too near the edge of a narrow strip to roll from.
        assert!(matches!(
            judge_lined_up(true, true, 3.8, 1.0, Some(15.0)),
            Err(LineUpMiss::OffCentreline { max_m, .. }) if (max_m - 3.5).abs() < 1e-9
        ));
        // A wide one keeps the drive's own 5 m arrival.
        assert_eq!(judge_lined_up(true, true, 4.9, 1.0, Some(45.0)), Ok(()));
        assert_eq!(judge_lined_up(true, true, 4.9, 1.0, None), Ok(()));
        // Pointing off the runway past what the roll's hold straightens.
        assert_eq!(
            judge_lined_up(true, true, 0.5, 12.4, Some(15.0)),
            Err(LineUpMiss::Crooked { err_deg: 12.4 })
        );
        // No drive of ours to judge (the tool refused, the aircraft already
        // there): the position alone.
        assert_eq!(judge_lined_up(false, false, 0.5, 1.0, Some(15.0)), Ok(()));
    }

    /// EGKK's two strips, the rows as apt.dat has them.
    fn egkk() -> Vec<crate::runway::Runway> {
        use std::io::Write;
        let apt = std::env::temp_dir().join("xfc-runway-start-egkk-apt.dat");
        let mut f = std::fs::File::create(&apt).unwrap();
        writeln!(f, "1    203 0 0 EGKK London Gatwick").unwrap();
        writeln!(f, "100 46.00 26 1527 0.25 1 3 0 08R 51.1450993 -0.2123748 395 0 7 4 1 3 26L 51.1514994 -0.1659296 425 30 7 4 1 3").unwrap();
        writeln!(f, "100 46.00 23 853 0.25 0 1 0 08L 51.1468901 -0.2126029 321 0 7 0 0 3 26R 51.1518321 -0.1767408 415 0 7 0 0 3").unwrap();
        f.flush().unwrap();
        crate::runway::nearby_runways(apt.to_str().unwrap(), 51.1477, -0.1935, 3.0).unwrap()
    }

    /// Sortie 1790527340 (EGKK, 2026-09-27): stopped on 26L's centreline
    /// at 51.14768 N 0.19355 W, 256.3 true, 1,897 m needed. The fit fails on
    /// LENGTH on 26L (not "none near" off 26R, 200 m away and aligned to
    /// 0.005 degrees more), so the arm backtracks instead of asking for the
    /// taxi that took it off the runway to holding point Mike.
    #[test]
    fn the_egkk_runway_start_backtracks_on_26l() {
        let cands = egkk();
        let (lat, lon, psi, need) = (51.14768083914999, -0.19354614043732044, 256.3, 1897.0);
        assert!(crate::runway::runway_fit(&cands, lat, lon, psi, need).is_none());
        // The coin toss the old diagnosis made: 26R is the better-aligned.
        let best_aligned = cands
            .iter()
            .min_by(|a, b| {
                crate::angles::abs_diff_deg(psi, a.course_deg)
                    .total_cmp(&crate::angles::abs_diff_deg(psi, b.course_deg))
            })
            .unwrap();
        assert_eq!(best_aligned.id, "26R");
        let why = crate::runway::why_no_fit_cleared(&cands, lat, lon, psi, need, None);
        let crate::runway::NoFit::TooLittleLeft { id, remaining_m, .. } = &why else {
            panic!("{why:?}");
        };
        assert_eq!(id, "26L");
        assert!((1250.0..1450.0).contains(remaining_m), "{remaining_m}");
        let start = classify_runway_start(Err(Some(why.clone())), 0.0);
        assert_eq!(start, RunwayStart::Short { runway: "26L".into() });
        // A roll in progress is not a runway start.
        assert_eq!(classify_runway_start(Err(Some(why)), 40.0), RunwayStart::None);
        // Enough ahead: lined up, whatever the speed.
        assert_eq!(
            classify_runway_start(Ok("26L".into()), 0.0),
            RunwayStart::Lined { runway: "26L".into() }
        );
        // Off any runway: the taxied departure.
        assert_eq!(classify_runway_start(Err(None), 0.0), RunwayStart::None);
        let none_near = crate::runway::NoFit::NoneNear {
            id: "26L".into(),
            lateral_m: 300.0,
        };
        assert_eq!(classify_runway_start(Err(Some(none_near)), 0.0), RunwayStart::None);
    }

    /// LEMG's two strips, the rows as apt.dat has them (12/30 with 190 m
    /// displaced on 12; 13/31 with 75 m on each end).
    fn lemg() -> Vec<crate::runway::Runway> {
        use std::io::Write;
        let apt = std::env::temp_dir().join("xfc-runway-start-lemg-apt.dat");
        let mut f = std::fs::File::create(&apt).unwrap();
        writeln!(f, "1     48 0 0 LEMG Malaga").unwrap();
        writeln!(f, "100 45.11 1 1 0.00 1 3 0  12 36.6910753 -4.5078270 0 190 3 1 1 1 30 36.6795648 -4.4805125 0 489 3 0 0 1").unwrap();
        writeln!(f, "100 45.11 27 27 0.00 1 3 0  13 36.6845573 -4.5126253 0 75 3 1 0 2 31 36.6654027 -4.4858140 0 75 3 1 0 3").unwrap();
        f.flush().unwrap();
        crate::runway::nearby_runways(apt.to_str().unwrap(), 36.68, -4.50, 3.0).unwrap()
    }

    /// #134 (LEMG, 2026-09-28): the runner's runway start for 13 DID put the
    /// 737 on 13's threshold (36.684289 N 4.512250 W, 131.4 true at t
    /// 1790584459.1, sortie 1790584715's arm fit "13"). The "Short on 12,
    /// 1,375.8 m left of 2,019.4" of sortie 1790584476 was judged at
    /// 36.68532005 N 4.49416975 W: runway 12's exact midpoint, the position
    /// X-Plane reports while a flight loads (AGL -84 ft), read because a
    /// queued "put us on a 5 mile final" was reloading the sim under the
    /// arm. The classifier was right about what it read; nothing may arm
    /// during a load (`setup::new_flight_loading_refusal`).
    #[test]
    fn the_lemg_runway_start_is_on_one_three_and_the_twelve_was_a_load() {
        let cands = lemg();
        let need = 2019.3635616820702;
        let (r, remaining) =
            crate::runway::runway_fit(&cands, 36.684288506540085, -4.51224958630406, 131.4, need)
                .expect("lined up on 13");
        assert_eq!(r.id, "13");
        assert!(remaining > 3000.0, "{remaining}");
        assert_eq!(
            classify_runway_start(Ok(r.id.clone()), 0.0),
            RunwayStart::Lined { runway: "13".into() }
        );
        // The load's transient: 12's midpoint, pointing down 13's course.
        let (lat, lon): (f64, f64) = (
            (36.6910753 + 36.6795648) / 2.0,
            (-4.5078270 + -4.4805125) / 2.0,
        );
        assert!((lat - 36.68532005).abs() < 1e-7 && (lon + 4.49416975).abs() < 1e-7);
        assert!(crate::runway::runway_fit(&cands, lat, lon, 131.4, need).is_none());
        let why = crate::runway::why_no_fit_cleared(&cands, lat, lon, 131.4, need, None);
        let crate::runway::NoFit::TooLittleLeft { id, remaining_m, .. } = &why else {
            panic!("{why:?}");
        };
        assert_eq!(id, "12");
        assert!((remaining_m - 1375.8).abs() < 5.0, "{remaining_m}");
    }

    /// #134: a runway start's zone clearance names its strip (the tower
    /// gives it that runway, `station`), says "for circuits" on a circuit,
    /// and is asked again when the held one names another runway: the last
    /// case's (31, 4501) is what sent sortie 1790584715 to taxi.
    #[test]
    fn a_runway_start_asks_its_clearance_for_its_own_runway() {
        use crate::tower::{parse_intent, stated_runway, wants_circuits, Intent};
        for circuits in [false, true] {
            let call = crate::crew_radio::radio_call(
                &Some("November eight one six November November".into()),
                &strip_clearance_call("13", circuits),
            );
            assert_eq!(parse_intent(&call), Intent::RequestClearance, "{call}");
            assert_eq!(stated_runway(&call).as_deref(), Some("13"), "{call}");
            assert_eq!(wants_circuits(&call), circuits, "{call}");
        }
        assert!(!strip_clearance_held("13", Some("31"), Some(4501)), "asked again");
        assert!(!strip_clearance_held("13", None, Some(4501)));
        assert!(!strip_clearance_held("13", Some("13"), None));
        assert!(strip_clearance_held("13", Some("13"), Some(4501)));
        // The zone request from the strip goes before the under-clearance
        // judgement; an IFR flight keeps its own clearance path.
        let src = crate::test_support::module_source("full_auto");
        let strip = src
            .find("zone_clearance_from_the_strip(&xp, &ctx")
            .expect("the runway start asks from the strip");
        let judged = src.find("start = start.under_clearance(").unwrap();
        assert!(strip < judged);
    }

    /// The clearance names the runway: the one we sit on stands, a zero-
    /// padded or case-different name included; another one is taxied to.
    #[test]
    fn a_runway_start_departs_from_the_cleared_runway() {
        let short = RunwayStart::Short { runway: "26L".into() };
        assert_eq!(short.clone().under_clearance(Some("26L")), short);
        assert_eq!(short.clone().under_clearance(None), short);
        assert_eq!(short.clone().under_clearance(Some("26R")), RunwayStart::None);
        let lined = RunwayStart::Lined { runway: "09".into() };
        assert_eq!(lined.clone().under_clearance(Some("9")), lined);
        assert_eq!(lined.clone().under_clearance(Some("22")), RunwayStart::None);
        assert_eq!(RunwayStart::None.under_clearance(Some("27")), RunwayStart::None);
        // The far end of our own strip is a backtrack down it (#178).
        assert_eq!(
            lined.under_clearance(Some("27")),
            RunwayStart::Short { runway: "27".into() }
        );
    }

    /// #178, sortie 1790646817-full_auto (LEMG, 2026-09-29): lined up on
    /// 13, cleared for the circuit on 30. 30 is the OTHER strip (12/30), so
    /// the taxi was right there; cleared on 31, the far end of 13's own
    /// pavement, the start backtracks down the runway (the Short branch:
    /// "request backtrack", the line-up tool's full-length backtrack, turned
    /// round at the numbers), never a taxi off it and an own-power turn.
    #[test]
    fn a_runway_start_cleared_for_its_reciprocal_backtracks() {
        assert!(is_reciprocal("13", "31"));
        assert!(is_reciprocal("31", "13"));
        assert!(is_reciprocal("08R", "26L"));
        assert!(is_reciprocal("9", "27"));
        assert!(!is_reciprocal("13", "30"), "LEMG's 30 is the 12/30 strip");
        assert!(!is_reciprocal("08R", "26R"), "a parallel's end is another runway");
        assert!(!is_reciprocal("13", "13"));
        let on13 = RunwayStart::Lined { runway: "13".into() };
        assert_eq!(on13.clone().under_clearance(Some("30")), RunwayStart::None);
        assert_eq!(
            on13.under_clearance(Some("31")),
            RunwayStart::Short { runway: "31".into() }
        );
        assert_eq!(
            backtrack_line("13", "31"),
            "The tower wants runway three one, and we're on one three, the same strip. \
             Asking to backtrack."
        );
        // The ready call's answer for the reciprocal is flown the same way,
        // once, before the taxi fallback.
        let src = crate::test_support::module_source("full_auto");
        let ready = src.find("ClearanceWait::OtherRunway(cleared) => {").unwrap();
        let bt = src[ready..].find("backtrack_from_runway_start(").unwrap();
        let taxi = src[ready..].find("other_runway_line(&on, &cleared)").unwrap();
        assert!(bt < taxi);
    }

    /// Issue #178, the party line as the crew read it in sortie
    /// 1790631369-full_auto (LEMG, 2026-09-28), lined up on 30: +25.3
    /// Parked 30 (the circuit clearance), +47.5 Parked 13 (the field turned
    /// round, nothing said to us), +58.6 TaxiCleared 13 ("taxi to holding
    /// point Hotel November one runway one three first"), +87.3 LinedUp 13,
    /// +219.3 TakeoffCleared 13. From the taxi on, every answer is for
    /// another runway: the crew taxis, reads back nothing from 30 and never
    /// says "lined up". Its own runway, the reciprocal and a taxied departure
    /// (no start) are judged too.
    #[test]
    fn a_runway_start_never_reads_back_another_runway() {
        use crate::tower::State::{LinedUp, Parked, TakeoffCleared, TaxiCleared};
        let on = Some("30");
        let records = [
            (Parked, "30", None),
            (Parked, "13", None),
            (TaxiCleared, "13", Some("13")),
            (LinedUp, "13", Some("13")),
            (TakeoffCleared, "13", Some("13")),
        ];
        for (state, rwy, want) in records {
            assert_eq!(
                cleared_elsewhere(on, Some(state), Some(rwy)).as_deref(),
                want,
                "{state:?} {rwy}"
            );
        }
        // Our own runway, however it is spelt: the takeoff is ours.
        assert_eq!(cleared_elsewhere(on, Some(TakeoffCleared), Some("30")), None);
        assert_eq!(cleared_elsewhere(Some("09"), Some(LinedUp), Some("9")), None);
        // The reciprocal of the strip we sit on is another runway.
        assert_eq!(
            cleared_elsewhere(on, Some(LinedUp), Some("12")).as_deref(),
            Some("12")
        );
        // No start, no runway known, no state: nothing to judge.
        assert_eq!(cleared_elsewhere(None, Some(TakeoffCleared), Some("13")), None);
        assert_eq!(cleared_elsewhere(on, Some(TakeoffCleared), None), None);
        assert_eq!(cleared_elsewhere(on, None, Some("13")), None);
        // What the crew says: where it is going and that it taxis.
        let line = other_runway_line("30", "13");
        assert!(line.contains("runway one three"), "{line}");
        assert!(line.contains("Taxiing"), "{line}");
        assert!(!line.to_lowercase().contains("lined up"), "{line}");
    }

    /// The runway start's wait is the one that can see another runway, and
    /// it looks before any readback goes out; the taxied waits pass `None`.
    /// Read from the source: the wait itself needs a live party line.
    #[test]
    fn the_runway_start_waits_with_its_own_runway() {
        let src = crate::test_support::module_source("full_auto");
        let wait = src
            .find("async fn ensure_takeoff_cleared(")
            .expect("the wait");
        let body = &src[wait..];
        let check = body.find("cleared_elsewhere(").expect("the check");
        let readback = body.find("OwedReadback::due(").expect("the readback");
        assert!(check < readback, "the runway is judged before a readback");
        let call = concat!(
            "ensure_takeoff_cleared(&voice, &callsign, &mut read_back, false, ",
            "Some(&on))"
        );
        assert_eq!(src.matches(call).count(), 1);
        assert!(src.contains(concat!("request_departure(&callsign, ", "Some(&on))")));
    }

    /// The arm reads the start once, before any taxi request, and a short
    /// start asks for the backtrack by name; read from the source because
    /// the orchestration is async over the live sim.
    #[test]
    fn the_arm_asks_for_a_backtrack_before_a_taxi() {
        let src = crate::test_support::module_source("full_auto");
        let start = src.find("let mut start = runway_start(").expect("the arm classifies");
        let backtrack = src.find("backtrack_from_runway_start(&xp").expect("a short start backtracks");
        let taxi = src.find("event(\"request_taxi\"").expect("the taxi request");
        assert!(start < backtrack && backtrack < taxi, "classify, backtrack, then the taxi");
        assert!(src.contains("\"request backtrack runway {}\""));
    }
