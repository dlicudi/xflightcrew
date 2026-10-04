    use super::*;

    /// The stock 737's detents (`profiles/boeing_737_800.yaml`).
    const B738: [(f64, Option<f64>); 9] = [
        (0.0, None),
        (0.125, Some(250.0)),
        (0.25, Some(250.0)),
        (0.375, Some(250.0)),
        (0.5, Some(210.0)),
        (0.625, Some(200.0)),
        (0.75, Some(190.0)),
        (0.875, Some(175.0)),
        (1.0, Some(162.0)),
    ];
    /// The 737's floors: clean 1.2 x 139, landing 1.2 x 114, the half flap's
    /// between them.
    const CLEAN_FLOOR: f64 = 167.0;
    const HALF_FLOOR: f64 = 152.0;
    const LANDING_FLOOR: f64 = 137.0;

    #[test]
    fn the_clean_737s_missed_approach_flies_210_not_355() {
        // EGNX 09 (2026-09-30): clean, 1,500 ft.
        let clean = missed_approach_speed_kts(
            145.0,
            0.0,
            &B738,
            Some(HALF_FLOOR),
            Some(CLEAN_FLOOR),
            1500.0,
        );
        assert!((clean - 210.25).abs() < 0.01, "{clean}");
        // Landing flap out: under its placard, fast enough to clean up to
        // the go-around flap (the half flap's floor plus 5).
        let full = missed_approach_speed_kts(
            145.0,
            1.0,
            &B738,
            Some(HALF_FLOOR),
            Some(LANDING_FLOOR),
            1500.0,
        );
        assert_eq!(full, 152.0);
        assert!(go_around_flap_due(full, Some(HALF_FLOOR)));
        // A faster airframe's clean speed is still 250 below 10,000 ft...
        let fast = missed_approach_speed_kts(190.0, 0.0, &[], None, None, 3000.0);
        assert_eq!(fast, 250.0);
        // ...and not above it.
        let high = missed_approach_speed_kts(190.0, 0.0, &[], None, None, 11_000.0);
        assert!((high - 275.5).abs() < 0.01);
        // The floor of the configuration flown wins everything.
        assert_eq!(missed_approach_speed_kts(100.0, 0.0, &[], None, Some(180.0), 3000.0), 180.0);
    }

    #[test]
    fn go_around_thrust_until_established_then_the_speed() {
        let floor = Some(0.3);
        // The balk at 150 ft, climbing 883 fpm at 248 kt: go-around thrust.
        assert_eq!(missed_approach_throttle(0.1, 248.0, 2.0, 210.0, 150.0, Some(883.0), floor), 1.0);
        // An unread climb is not established.
        assert_eq!(missed_approach_throttle(1.0, 277.0, 3.0, 210.0, 600.0, None, floor), 1.0);
        // 426 ft, 2,476 fpm up, 277 kt: established and 67 kt fast, the
        // climb floor at once (the tick EGNX 09 was still at 1.0).
        assert_eq!(missed_approach_throttle(1.0, 277.0, 3.0, 210.0, 426.0, Some(2476.0), floor), 0.3);
        // Idle where no floor is declared.
        assert_eq!(
            missed_approach_throttle(1.0, 277.0, 3.0, 210.0, 426.0, Some(2476.0), None),
            IDLE_THROTTLE
        );
        // Near the speed, the governor's step inside the floor and full.
        let near = missed_approach_throttle(0.5, 214.0, -1.0, 210.0, 900.0, Some(1500.0), floor);
        assert!((0.3..=1.0).contains(&near) && (near - 0.5).abs() <= 0.041, "{near}");
        // Slow by more than 10: go-around thrust again.
        assert_eq!(missed_approach_throttle(0.3, 198.0, -2.0, 210.0, 900.0, Some(1500.0), floor), 1.0);
    }

    /// A crude clean 737 in a go-around climb (kt/s = 5 thr - 1.5 - 0.5 x
    /// climb per 1,000 fpm, the lever spooling at 2.5 s): TOGA gains the 3
    /// kt/s the EGNX 09 miss did at 2,400 fpm. From the balk at 150 ft and
    /// 250 kt the climb is held at the VS mode's 2,000 fpm.
    #[test]
    fn the_egnx_missed_approach_replayed_stays_under_250() {
        let (mut v, mut h, mut vs, mut thr, mut lever) =
            (250.0_f64, 150.0_f64, 0.0_f64, 1.0_f64, 1.0_f64);
        let mut prev_v = v;
        let dt = 0.5;
        let mut peak = v;
        let mut t = 0.0;
        while t < 120.0 {
            let trend = (v - prev_v) / dt;
            prev_v = v;
            let target = missed_approach_speed_kts(
                145.0,
                0.0,
                &B738,
                Some(HALF_FLOOR),
                Some(CLEAN_FLOOR),
                300.0 + h,
            );
            thr = missed_approach_throttle(thr, v, trend, target, h, Some(vs), Some(0.3));
            lever += (thr - lever) * (dt / 2.5);
            vs += (2000.0 - vs) * (dt / 4.0);
            v += (5.0 * lever - 1.5 - 0.5 * vs / 1000.0) * dt;
            h += vs / 60.0 * dt;
            peak = peak.max(v);
            t += dt;
        }
        assert!(peak < 285.0, "the peak {peak:.0} kt (355 flown)");
        assert!((v - 210.0).abs() < 15.0, "settles on the go-around speed: {v:.0}");
    }
