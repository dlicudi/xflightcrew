    use super::*;

    /// The recorded ticks after the balk (s from it, radio ft, vvi fpm, IAS
    /// kt), as the autoland loop logged them: +1,500 then 0 on the dial,
    /// half flap from the balk, the wheels at -1,568.
    const REC: [(f64, f64, f64, f64); 10] = [
        (0.1, 52.1, 860.0, 81.0),
        (0.6, 58.1, 796.0, 79.0),
        (1.1, 60.8, 649.0, 77.3),
        (1.6, 60.6, 418.0, 75.9),
        (2.1, 58.0, 151.0, 74.8),
        (2.6, 53.2, -130.0, 74.2),
        (3.1, 44.8, -463.0, 74.1),
        (3.7, 33.2, -819.0, 74.7),
        (4.2, 18.6, -1175.0, 76.0),
        (4.7, 2.7, -1487.0, 77.8),
    ];
    /// The SF50's numbers: approach 87, Vso 67 (floor 80), the half flap's
    /// stall where the recording met it (74.5, floor 89; by the straight
    /// line to a clean 82), and the best climb the loop asked (+1,500).
    const APPROACH: f64 = 87.0;
    const CLIMB: f64 = 1500.0;
    fn stall(flap: f64) -> f64 {
        67.0 + (82.0 - 67.0) * (1.0 - flap)
    }
    fn floor(flap: f64) -> f64 {
        (1.2 * stall(flap)).round()
    }

    /// The airframe as it answered, fitted to the ticks before the stall: the
    /// speed changes at -2.06 + 0.38 t kt/s (idle spooling up after the
    /// balk; capped at +2.5) less 2.44 kt/s per 1,000 fpm of climb (0.08
    /// kt/s rms; g sin of the path at 78 kt is 2.41, so the climb's cost is
    /// physics, not a fit); the VS mode follows the dial after 0.3 s through
    /// a 0.5 s lag, down at up to 700 fpm a second and UP at only 50 (the
    /// +1,500 asked at 81 kt never showed); under the stall speed a wing the
    /// autopilot keeps pulling loses 800 fpm a second, until it is 3 kt clear
    /// and the dial lets it go. The recorded radio height fell faster than
    /// the VSI (58 to 3 ft in 2.6 s at a mean of -800), so height is not
    /// fitted.
    struct Sf50 {
        t: f64,
        h: f64,
        vvi: f64,
        ias: f64,
        stalled: bool,
        ever_stalled: bool,
        dials: std::collections::VecDeque<(f64, f64)>,
        dial: f64,
    }

    impl Sf50 {
        fn at_the_balk() -> Self {
            Sf50 {
                t: 0.0,
                h: REC[0].1,
                vvi: REC[0].2,
                ias: REC[0].3,
                stalled: false,
                ever_stalled: false,
                dials: std::collections::VecDeque::new(),
                dial: REC[0].2,
            }
        }
        fn write(&mut self, dial: f64) {
            self.dials.push_back((self.t + 0.3, dial));
        }
        fn step(&mut self, flap: f64, dt: f64) {
            while self.dials.front().is_some_and(|&(t, _)| t <= self.t) {
                self.dial = self.dials.pop_front().map_or(self.dial, |(_, d)| d);
            }
            let s = stall(flap);
            if self.ias < s && self.dial >= self.vvi - 100.0 {
                self.stalled = true;
            }
            if self.stalled && (self.ias > s + 3.0 || self.dial < self.vvi - 100.0) {
                self.stalled = false;
            }
            self.ever_stalled |= self.stalled;
            let vdot = if self.stalled {
                -800.0
            } else {
                ((self.dial - self.vvi) / 0.5).clamp(-700.0, 50.0)
            };
            self.vvi += vdot * dt;
            let a = (-2.06 + 0.38 * (self.t + REC[0].0)).min(2.5);
            self.ias += (a - 2.44 * self.vvi / 1000.0) * dt;
            self.h += self.vvi / 60.0 * dt;
            self.t += dt;
        }
    }

    struct Flown {
        wheels_fpm: f64,
        stalled: bool,
        min_ias: f64,
        min_h: f64,
        trace: Vec<(f64, f64, f64, f64)>,
    }

    /// Fly from the balk to the ground (or 20 s) at the loop's 0.5 s tick.
    /// `new`: today's law and flap; else the dial and flap as flown.
    fn fly(new: bool) -> Flown {
        let mut p = Sf50::at_the_balk();
        let (mut prev_ias, mut flap) = ((83.7, -1.1), 1.0);
        let mut out = Flown {
            wheels_fpm: 0.0,
            stalled: false,
            min_ias: p.ias,
            min_h: p.h,
            trace: Vec::new(),
        };
        let (dt, tick) = (0.01, 0.5);
        let mut next = 0.0;
        while p.h > 0.0 && p.t < 20.0 {
            if p.t + 1e-9 >= next {
                next += tick;
                let trend = (p.ias - prev_ias.0) / (p.t - prev_ias.1);
                prev_ias = (p.ias, p.t);
                let dial = if new {
                    if go_around_flap_due(p.ias, Some(floor(0.5))) {
                        flap = 0.5;
                    }
                    go_around_vs_fpm(p.ias, trend, APPROACH, Some(floor(flap)), p.h, CLIMB)
                } else {
                    flap = 0.5;
                    if APPROACH - p.ias > SPEED_FLOOR_MARGIN_KTS {
                        0.0
                    } else {
                        CLIMB
                    }
                };
                p.write(dial);
                out.trace.push((p.t, p.h, p.vvi, p.ias));
            }
            p.step(flap, dt);
            out.min_ias = out.min_ias.min(p.ias);
            out.min_h = out.min_h.min(p.h);
        }
        out.wheels_fpm = p.vvi;
        out.stalled = p.ever_stalled;
        out
    }

    /// The plant is the recording: the dial and flap as flown give back the
    /// recorded ticks and the stall onto the runway.
    #[test]
    fn the_sf50_plant_replays_the_edlp_go_around() {
        let f = fly(false);
        for &(t, h, vvi, ias) in REC.iter().skip(1) {
            let k = f
                .trace
                .iter()
                .min_by(|a, b| (a.0 - t).abs().total_cmp(&(b.0 - t).abs()))
                .unwrap();
            assert!((k.3 - ias).abs() < 1.0, "{t} s: {:.1} kt against {ias}", k.3);
            assert!((k.2 - vvi).abs() < 350.0, "{t} s: {:.0} fpm against {vvi}", k.2);
            let _ = h;
        }
        assert!(f.stalled, "the go-around as flown stalls");
        assert!(f.wheels_fpm < -1200.0, "and meets the runway hard: {:.0}", f.wheels_fpm);
    }

    /// Pitched for speed: the half flap waits for its floor, the dial asks
    /// level and then a descent the height allows while the speed is short,
    /// and the SF50 never stalls: its lowest speed 75.7 kt against the full
    /// flap's 67, its lowest height 30 ft over the runway, climbing (+426) when
    /// the thrust has come, where the go-around as flown hit at -1,729.
    #[test]
    fn a_slow_low_go_around_pitches_for_speed_and_never_stalls() {
        let f = fly(true);
        assert!(!f.stalled, "{:?}", f.trace);
        assert!(f.min_ias > stall(1.0) + 3.0, "{:.1}", f.min_ias);
        assert!(f.min_h > 20.0, "{:.0} ft", f.min_h);
        assert!(f.wheels_fpm > 200.0, "climbing away: {:.0} fpm", f.wheels_fpm);
    }

    /// The law by itself: on speed, the airframe's climb; slow over the
    /// floor, level; under it, a descent no steeper than the ground ten
    /// seconds away, level on the runway; a lost trend or floor is no
    /// measurement (the approach speed's own floor, no projection).
    #[test]
    fn the_go_around_vertical_is_speed_first() {
        assert_eq!(go_around_vs_fpm(86.0, 0.0, 87.0, Some(80.0), 50.0, 1500.0), 1500.0);
        // Decaying at 3 kt/s: 86 projects to 80, slow.
        assert_eq!(go_around_vs_fpm(86.0, -4.0, 87.0, Some(78.0), 50.0, 1500.0), 0.0);
        let v = go_around_vs_fpm(79.0, -3.0, 87.0, Some(80.0), 60.0, 1500.0);
        assert!((v + 360.0).abs() < 1e-9, "{v}");
        let v = go_around_vs_fpm(79.0, -3.0, 87.0, Some(80.0), 600.0, 1500.0);
        assert!((v + 700.0).abs() < 1e-9, "{v}");
        assert_eq!(go_around_vs_fpm(70.0, -3.0, 87.0, Some(80.0), 0.0, 1500.0), 0.0);
        assert_eq!(go_around_vs_fpm(86.0, f64::NAN, 87.0, None, 50.0, 1500.0), 1500.0);
        assert!(go_around_vs_fpm(70.0, f64::NAN, 87.0, None, 50.0, 1500.0) < 0.0);
        // The flap: half at its floor, not before.
        assert!(!go_around_flap_due(81.0, Some(88.0)));
        assert!(go_around_flap_due(88.0, Some(88.0)));
        assert!(go_around_flap_due(81.0, None));
    }
