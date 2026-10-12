    //! LXGB 09, the stock 737, wet (09011KT -RA), 2026-10-06, sortie
    //! 1791275171-full_auto (`bench/landings/lxgb09_737_float.jsonl`). The
    //! strip is 1,763 m from its apt.dat end, the threshold displaced 136 m;
    //! the POM's landing distance is 1,634 m. The arrest took the servos at
    //! 79.8 ft, 0.0745 nm short of the strip, at 145.9 kt (5.5 kt of
    //! headwind); the flare floated, and at 4.5 ft radio, 0.3535 nm past the
    //! strip's end with 1,109 m ahead, the overflight balk stood: "Going
    //! around. Full power, climbing straight ahead." toward the Rock. The
    //! wheels touched 1.6 s later on the spool-up (`balk_landed`, 995.6 m
    //! ahead) and it stopped in about 540 m.
    use super::*;

    const LENGTH_M: f64 = 1763.48;
    const BOOK_M: f64 = 5360.0 * 0.3048;
    const COURSE: f64 = 87.4;
    const HEAD_KTS: f64 = 5.5;

    fn roll_m() -> f64 {
        crate::aircraft::load("boeing_737_800")
            .expect("the 737 profile loads")
            .reference
            .and_then(|r| r.landing_roll_ft)
            .expect("the 737 declares its landing roll")
            * 0.3048
    }

    /// The gate as recorded, at `ias` (its groundspeed the wind's less).
    fn gate_call(ias: f64, length_m: f64) -> LandingCall {
        landing_call(
            0.0745 * M_PER_NM,
            length_m,
            79.8,
            ias - HEAD_KTS,
            Some(ias),
            145.0,
            wheels_need_m(Some(BOOK_M), Some(roll_m())),
        )
        .expect("every measurement read")
    }

    fn overflown() -> Guidance {
        Guidance {
            target_heading_deg: 250.0,
            target_vs_fpm: 2000.0,
            target_ias_kts: 157.0,
            dist_nm: 0.3535,
            glidepath_alt_ft: 0.0,
            phase: ApproachPhase::Intercept,
            balk: Some("overflew_the_runway"),
            arresting: false,
        }
    }

    /// The room to stop from the wheels is the declared ROLL (2,600 ft: the
    /// POM's factored 5,360 ft x 0.6 is 3,216 from 50 ft, less the 955 ft
    /// of air, x 1.15 wet), never the field length less the air (1,343 m,
    /// more than the 1,109 the balk had and twice the 540 it used).
    #[test]
    fn the_wheels_need_the_roll_not_the_field_length() {
        let roll = roll_m();
        assert!((roll - 792.5).abs() < 0.1, "{roll}");
        assert_eq!(wheels_need_m(Some(BOOK_M), Some(roll)), roll);
        let book_wheels = wheels_need_m(Some(BOOK_M), None);
        assert!((book_wheels - 1342.9).abs() < 0.5, "{book_wheels}");
        assert_eq!(book_wheels, flare_pavement_need_m(0.0, Some(BOOK_M)));
        assert_eq!(wheels_need_m(None, None), FLOAT_MIN_REMAINING_M);
        assert_eq!(wheels_need_m(Some(BOOK_M), Some(f64::NAN)), book_wheels);
    }

    /// From the recorded gate the energy predicts a touchdown with the roll
    /// still ahead: LAND, decided at 79.8 ft. On the attitude flare's
    /// round-out and hold-off that is 923 m in with 840 m left against the
    /// roll's 792 (the old law's 5.5 s exponential: 9 to 13 s of air, the
    /// touchdown 400 to 800 m in). The same gate 25 kt fast floats past the
    /// room to stop: the go-around is flown from there, with height in hand;
    /// on LEMG 13's 3,200 m the fast one still lands.
    #[test]
    fn the_lxgb_gate_calls_a_landing() {
        let c = gate_call(145.9, LENGTH_M);
        assert!(c.land, "{c:?}");
        assert!(
            c.touchdown_m > 800.0 && c.touchdown_m < 1000.0,
            "the flare from 79.8 ft: {c:?}"
        );
        assert!(c.remaining_m > c.need_m, "{c:?}");
        assert!(c.air_s > 13.0 && c.air_s < 16.0, "{c:?}");
        let fast = gate_call(170.0, LENGTH_M);
        assert!(!fast.land, "25 kt fast: {fast:?}");
        assert!(fast.touchdown_m > c.touchdown_m + 500.0);
        assert!(gate_call(170.0, 3200.0).land, "LEMG 13's 3,200 m takes it");
        // No measurement, no decision.
        assert!(landing_call(
            f64::NAN,
            LENGTH_M,
            79.8,
            140.0,
            None,
            145.0,
            800.0
        )
        .is_none());
        assert!(landing_call(
            137.0,
            LENGTH_M,
            79.8,
            f64::NAN,
            None,
            145.0,
            800.0
        )
        .is_none());
        // An unread airspeed adds no float: the flare alone.
        let unread = landing_call(
            0.0745 * M_PER_NM,
            LENGTH_M,
            79.8,
            140.4,
            None,
            145.0,
            800.0,
        )
        .unwrap();
        assert!(unread.air_s < c.air_s, "{unread:?} {c:?}");
    }

    /// The flare's time is the attitude flare's: the path's sink down to
    /// its flare height, then its round-out to the hold-off's sink.
    #[test]
    fn the_flare_time_is_the_law_it_flies() {
        use crate::pilot::attitude as af;
        let path = -(140.0 / 60.0) * gp_ft_per_nm();
        let gate = af::gate_ft(path, af::RESPONSE_S);
        let t_gate = flare_time_s(gate, 140.0).unwrap();
        let t80 = flare_time_s(80.0, 140.0).unwrap();
        assert!(
            (t80 - t_gate - (80.0 - gate) / -path * 60.0).abs() < 1e-9,
            "the path's sink above the gate: {t80} {t_gate}"
        );
        // Below the gate never faster than the path, never slower than the
        // hold-off's sink.
        assert!(t_gate > gate / -path * 60.0, "{t_gate}");
        assert!(t_gate < gate / -af::HOLDOFF_VS_FPM * 60.0, "{t_gate}");
        assert!(flare_time_s(0.0, 140.0).unwrap().abs() < 1e-9);
        assert!(flare_time_s(f64::NAN, 140.0).is_none());
        assert!(flare_time_s(20.0, 0.0).is_none());
    }

    /// The balk at 4.5 ft as flown: unlatched, and latched on the book's
    /// ground roll, the overflight balk stands (1,109 m against 1,369 and
    /// 1,343) and full power goes on toward the terrain. Committed by the
    /// gate's call on the declared roll, the flare stays a flare down the
    /// runway's course, and only a strip too short for the roll balks.
    #[test]
    fn a_landing_committed_at_the_gate_is_not_balked_at_four_feet() {
        let ahead = LENGTH_M - 0.3535 * M_PER_NM;
        assert!((ahead - 1109.0).abs() < 1.0, "{ahead}");
        let book = Some(BOOK_M);
        let as_flown = commit_flare_latched(
            overflown(),
            true,
            None,
            4.54,
            136.0,
            25.0,
            ahead,
            book,
            COURSE,
        );
        assert_eq!(as_flown.balk, Some("overflew_the_runway"), "as flown");
        let book_latch = commit_flare_latched(
            overflown(),
            true,
            Some(wheels_need_m(book, None)),
            4.54,
            136.0,
            25.0,
            ahead,
            book,
            COURSE,
        );
        assert_eq!(book_latch.balk, Some("overflew_the_runway"));
        let need = Some(wheels_need_m(book, Some(roll_m())));
        let held = commit_flare_latched(
            overflown(),
            true,
            need,
            4.54,
            136.0,
            25.0,
            ahead,
            book,
            COURSE,
        );
        assert_eq!((held.phase, held.balk), (ApproachPhase::Flare, None));
        assert_eq!(held.target_heading_deg, COURSE);
        assert!(held.arresting);
        // ...to the wheels: the touchdown left 995.6 m.
        let wheels = commit_flare_latched(
            overflown(),
            true,
            need,
            1.0,
            136.0,
            25.0,
            995.6,
            book,
            COURSE,
        );
        assert_eq!(wheels.balk, None);
        // No room to stop is still a go-around, committed or not.
        let short = commit_flare_latched(
            overflown(),
            true,
            need,
            4.54,
            136.0,
            25.0,
            700.0,
            book,
            COURSE,
        );
        assert_eq!(short.balk, Some("overflew_the_runway"));
        // And never above the band.
        let (_, band) = arrest_gates_ft(25.0, 136.0);
        let high = commit_flare_latched(
            overflown(),
            true,
            need,
            band + 1.0,
            136.0,
            25.0,
            ahead,
            book,
            COURSE,
        );
        assert_eq!(high.balk, Some("overflew_the_runway"));
    }
