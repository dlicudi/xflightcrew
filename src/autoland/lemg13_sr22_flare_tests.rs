    use super::*;

    /// The recorded arrest (s from `arrest_hands` taken, radio ft, vvi fpm,
    /// the elevator written, IAS kt), to the last tick before the wheels.
    const REC: [(f64, f64, f64, f64, f64); 38] = [
        (0.0, 50.63, -475.8, 0.126, 92.23),
        (0.4, 46.72, -475.0, 0.139, 92.32),
        (0.9, 43.75, -447.2, 0.105, 92.48),
        (1.3, 43.15, -357.5, -0.028, 92.58),
        (1.8, 44.21, -179.0, -0.100, 92.47),
        (2.2, 45.21, -43.5, -0.100, 92.08),
        (2.7, 45.18, 9.7, -0.100, 91.43),
        (3.1, 43.88, -37.5, -0.100, 90.88),
        (3.6, 41.09, -152.1, 0.016, 90.42),
        (4.0, 37.69, -268.3, 0.114, 90.17),
        (4.5, 33.87, -404.7, 0.206, 90.09),
        (4.8, 30.61, -464.0, 0.202, 90.16),
        (5.2, 28.07, -471.1, 0.127, 90.32),
        (5.6, 27.52, -386.1, -0.035, 90.37),
        (6.0, 28.40, -214.6, -0.100, 90.20),
        (6.4, 28.87, -91.0, -0.100, 89.81),
        (6.9, 28.15, -52.1, -0.100, 89.17),
        (7.3, 25.98, -115.3, -0.045, 88.66),
        (7.8, 21.76, -261.7, 0.221, 88.22),
        (8.0, 19.03, -352.8, 0.312, 88.11),
        (8.4, 14.87, -475.2, 0.350, 88.14),
        (8.7, 12.51, -528.0, 0.350, 88.26),
        (9.0, 11.12, -503.4, 0.350, 88.30),
        (9.3, 11.41, -408.4, 0.350, 88.14),
        (9.6, 13.00, -236.7, 0.000, 87.65),
        (9.9, 15.08, -74.4, 0.000, 87.05),
        (10.2, 18.24, 123.9, 0.000, 86.08),
        (10.5, 20.25, 239.6, 0.000, 85.12),
        (10.8, 21.73, 298.4, 0.000, 83.79),
        (11.1, 22.04, 276.3, 0.000, 82.81),
        (11.4, 21.47, 189.7, 0.000, 81.66),
        (11.7, 20.14, 83.4, 0.053, 80.85),
        (12.1, 17.81, -74.6, 0.176, 80.11),
        (12.4, 15.00, -195.5, 0.269, 79.80),
        (12.7, 11.14, -349.1, 0.350, 79.65),
        (13.0, 7.55, -469.4, 0.350, 79.67),
        (13.3, 3.83, -558.4, 0.350, 79.71),
        (13.6, 1.37, -584.8, 0.350, 79.65),
    ];
    /// The lever as commanded (s, ratio): approach power, the arrest's
    /// "only back", idle at the flare gate, the balloon's approach thrust,
    /// idle again.
    const THR: [(f64, f64); 5] = [
        (-10.0, 0.318),
        (1.5, 0.309),
        (7.9, 0.0),
        (10.3, 0.309),
        (12.2, 0.0),
    ];
    const TICK_S: f64 = 0.35;
    /// The SR22's landing-configuration stall (POH, profile `vs0_kts`).
    const VSO_KTS: f64 = 60.0;
    /// The final's approach thrust and speed at the gate.
    const APPROACH_THR: f64 = 0.318;
    const APPROACH_IAS: f64 = 92.0;
    /// Groundspeed less airspeed (4.7 kt from 145 on runway 13).
    const HEADWIND_KTS: f64 = 3.0;

    /// One fit: (fpm per unit, lag 1, lag 2, dead time), the column the
    /// autopilot left in the pipe, fpm per unit of lever and its lag, fpm
    /// of sink per knot lost.
    #[derive(Debug, Clone, Copy, PartialEq)]
    struct Fit {
        el: (f64, f64, f64, f64),
        pre: f64,
        thr_fpm: f64,
        thr_lag: f64,
        per_kt: f64,
    }

    /// The whole record (39 fpm rms), then fitted to its parts alone: the
    /// first arrest (3 fpm rms there), the arrest to the flare gate (26),
    /// the flare (37, and 50 over the whole record).
    const PLANTS: [Fit; 4] = [
        Fit {
            el: (10000.0, 1.46, 1.43, 0.92),
            pre: -0.058,
            thr_fpm: 316.0,
            thr_lag: 2.70,
            per_kt: 115.0,
        },
        Fit {
            el: (7720.0, 0.95, 1.72, 0.77),
            pre: -0.062,
            thr_fpm: 1074.0,
            thr_lag: 1.10,
            per_kt: 34.0,
        },
        Fit {
            el: (6240.0, 1.03, 1.03, 1.17),
            pre: -0.132,
            thr_fpm: 1160.0,
            thr_lag: 2.78,
            per_kt: 183.0,
        },
        Fit {
            el: (9070.0, 1.42, 1.31, 0.86),
            pre: -0.038,
            thr_fpm: 1.0,
            thr_lag: 0.16,
            per_kt: 94.0,
        },
    ];
    const WHOLE: usize = 0;

    /// Gates the arrest must land from (radio ft, vvi fpm, trend fpm/s,
    /// IAS kt): the recorded one, and the same final a little steeper,
    /// shallower, slower and faster.
    const GATES: [(f64, f64, f64, f64); 5] = [
        (50.63, -475.8, 1.0, 92.23),
        (50.0, -560.0, -20.0, 92.0),
        (50.0, -400.0, 20.0, 92.0),
        (50.0, -480.0, 0.0, 86.0),
        (50.0, -480.0, 0.0, 97.0),
    ];

    struct Plant {
        fit: Fit,
        h: f64,
        v: f64,
        ias: f64,
        ias0: f64,
        base: f64,
        b1: f64,
        b2: f64,
        thr: f64,
        thr_cmd: f64,
        stall: f64,
        gust: f64,
        rng: u64,
        rough_fpm: f64,
        t: f64,
        written: Vec<(f64, f64)>,
    }

    impl Plant {
        /// As recorded: the autopilot's column settled in the pipe.
        fn recorded_gate(fit: Fit) -> Self {
            let (_, h, v, _, ias) = REC[0];
            let b = fit.el.0 * fit.pre;
            Plant {
                fit,
                h,
                v,
                ias,
                ias0: ias,
                base: v - b,
                b1: b,
                b2: b,
                thr: APPROACH_THR,
                thr_cmd: APPROACH_THR,
                stall: 0.0,
                gust: 0.0,
                rng: 0,
                rough_fpm: 0.0,
                t: 0.0,
                written: vec![(-10.0, fit.pre)],
            }
        }
        /// At a gate: sinking `v` with `trend`, the column that made the
        /// trend already in the pipe.
        fn at(fit: Fit, h: f64, v: f64, trend: f64, ias: f64) -> Self {
            let mut p = Self::recorded_gate(fit);
            let b = fit.el.0 * fit.pre;
            p.h = h;
            p.v = v;
            p.ias = ias;
            p.ias0 = ias;
            p.b1 = b + trend * fit.el.2;
            p.base = v - b;
            p
        }
        fn rough(mut self, seed: u64, rms_fpm: f64) -> Self {
            self.rng = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1;
            self.rough_fpm = rms_fpm;
            self
        }
        fn gauss(&mut self) -> f64 {
            let mut u = || {
                self.rng ^= self.rng << 13;
                self.rng ^= self.rng >> 7;
                self.rng ^= self.rng << 17;
                ((self.rng >> 11) as f64 + 0.5) / (1u64 << 53) as f64
            };
            let (a, b) = (u(), u());
            (-2.0 * a.ln()).sqrt() * (std::f64::consts::TAU * b).cos()
        }
        fn write(&mut self, e: f64) {
            self.written.push((self.t, e));
        }
        fn step(&mut self, dt: f64) {
            let (k, l1, l2, dead) = self.fit.el;
            let due = self.t - dead;
            let arriving = self
                .written
                .iter()
                .take_while(|(t, _)| *t <= due + 1e-9)
                .last()
                .map_or(0.0, |w| w.1);
            self.b1 += (k * arriving - self.b1) * dt / l1;
            self.b2 += (self.b1 - self.b2) * dt / l2;
            self.thr += (self.thr_cmd - self.thr) * dt / self.fit.thr_lag;
            if self.rough_fpm > 0.0 {
                const TC: f64 = 1.5;
                let n = self.gauss();
                self.gust += -self.gust / TC * dt + self.rough_fpm * (2.0 * dt / TC).sqrt() * n;
            }
            if self.ias < VSO_KTS {
                self.stall += 800.0 * dt;
            }
            self.v = self.base
                + self.b2
                + self.fit.thr_fpm * (self.thr - APPROACH_THR)
                + self.fit.per_kt * (self.ias - self.ias0)
                + self.gust
                - self.stall;
            self.ias += (-2.70 - 5.1 * self.v / 1000.0 + 2.74 * self.thr) * dt;
            self.h += self.v / 60.0 * dt;
            self.t += dt;
        }
        /// The recorded column and lever, written as they were, to `until`.
        fn recorded(&mut self, until: f64) {
            while self.t < until - 1e-9 {
                let e = REC
                    .iter()
                    .take_while(|r| r.0 <= self.t + 1e-9)
                    .last()
                    .map_or(0.0, |r| r.3);
                if self.written.last().map(|w| w.1) != Some(e) {
                    self.write(e);
                }
                self.thr_cmd = THR
                    .iter()
                    .take_while(|r| r.0 <= self.t + 1e-9)
                    .last()
                    .map_or(APPROACH_THR, |r| r.1);
                self.step(0.01);
            }
        }
    }

    fn sr22() -> crate::aircraft::AircraftProfile {
        crate::aircraft::load("cirrus_sr22").expect("the SR22 profile loads")
    }

    /// The SR22's flare as declared from 2026-09-28 to the hold-off of
    /// 2026-09-29 (the 90 kt finals these plants were fitted to): the model
    /// 6,000 fpm through 1.0 s lags after 0.8 s, no hold-off, 0.35 of pull,
    /// the lever held to a second from the wheels. Today's hold-off is
    /// `lemg13_sr22_bounce_tests`.
    pub(super) fn flare_0928() -> crate::aircraft::FlareElevatorProfile {
        let mut fe = sr22().flare_elevator.expect("a flare_elevator");
        fe.max = 0.35;
        fe.tau_s = None;
        let mut m = fe.model.expect("a model");
        m.lag_s = 1.0;
        m.lag2_s = None;
        m.dead_s = 0.8;
        m.ground_fpm = 0.0;
        m.hold_off_ft = None;
        m.touchdown_fpm = None;
        m.fpm_per_kt = None;
        fe.model = Some(m);
        fe
    }

    pub(super) fn power_0928() -> crate::aircraft::FlarePowerProfile {
        crate::aircraft::FlarePowerProfile {
            reduction_per_second: 0.03,
            retard_below_ft: None,
            retard_max_per_s: Some(0.03),
            retard_slow_kts: None,
            retard_at_s: Some(1.0),
            retard_fast_kts: None,
            hold_from_s: None,
        }
    }

    /// Which elevator law flies.
    enum Law {
        Model(FlareModel),
        Tracker(FlarePitch, Option<f64>),
    }

    impl Law {
        /// The 90 kt law these plants were written for ([`flare_0928`]).
        fn today() -> Self {
            Law::Model(FlareModel::from_profile(Some(flare_0928())).expect("the 09-28 model"))
        }
        /// The law the SR22 flew at LEMG 13: no `flare_elevator:`, the
        /// default tracker (the servos latched as today).
        fn as_flown() -> Self {
            Law::Tracker(FlarePitch::with_gains(FlareGains::default()), None)
        }
        fn step(&mut self, h: f64, gs: f64, v: f64, dt: f64, flare: bool, rough: bool) -> f64 {
            match self {
                Law::Model(m) => m.step(h, gs, v, dt, flare, rough).elev,
                Law::Tracker(p, prev) => {
                    let trend = prev.map_or(0.0, |p| (v - p) / dt);
                    *prev = Some(v);
                    let nominal = -(gs / 60.0) * gp_ft_per_nm();
                    let (roundout, flare_vs) = arrest_targets_fpm(nominal);
                    p.step(if flare { flare_vs } else { roundout }, v, trend, dt, flare)
                }
            }
        }
    }

    #[derive(Debug)]
    struct Landed {
        wheels_fpm: f64,
        pushed_climbing: bool,
        secs: f64,
        min_ias: f64,
    }

    /// The arrest from here to the wheels at the loop's tick, the servos
    /// kept: the flare below its gate and a flare stays a flare to the
    /// band's top (`hold_flare`), the roundout above. The lever as the loop
    /// flies it: the arrest's approach thrust, then the profile's
    /// `flare_power` (idle at the flare gate when it declares none), the
    /// approach thrust again while climbing (`balloon_power`).
    fn land(p: &mut Plant, law: &mut Law, rough: bool) -> Landed {
        land_with(p, law, rough, Some(power_0928()))
    }

    fn land_with(
        p: &mut Plant,
        law: &mut Law,
        rough: bool,
        cfg: Option<crate::aircraft::FlarePowerProfile>,
    ) -> Landed {
        let mut power = FlarePower::default();
        let start = p.t;
        let (mut next, mut flaring) = (p.t, false);
        let mut out = Landed {
            wheels_fpm: 0.0,
            pushed_climbing: false,
            secs: 0.0,
            min_ias: p.ias,
        };
        while p.h > 0.0 && p.t - start < 40.0 {
            if p.t + 1e-9 >= next {
                next += TICK_S;
                let gs = p.ias - HEADWIND_KTS;
                let (flare_ft, band_ft) = arrest_gates_ft(25.0, gs);
                flaring = p.h <= flare_ft || (flaring && p.h <= band_ft);
                let e = law.step(p.h, gs, p.v, TICK_S, flaring, rough);
                out.pushed_climbing |= p.v > BALLOON_CLIMB_FPM && e < -1e-9;
                p.write(e);
                p.thr_cmd = if !flaring {
                    power.reset();
                    APPROACH_THR.min(p.thr_cmd.max(0.0))
                } else if let Some(t) = balloon_power(
                    Some(p.v),
                    Some(APPROACH_THR),
                    p.h > band_ft,
                    cfg.is_some(),
                )
                {
                    t
                } else {
                    match cfg {
                        Some(c) => power
                            .step(
                                c,
                                p.thr_cmd,
                                p.v,
                                TICK_S,
                                p.h,
                                Some(p.ias),
                                Some(APPROACH_IAS),
                            )
                            .unwrap_or(p.thr_cmd),
                        None => 0.0,
                    }
                };
            }
            p.step(0.01);
            out.min_ias = out.min_ias.min(p.ias);
        }
        out.wheels_fpm = p.v;
        out.secs = p.t - start;
        out
    }

    /// The plant is the recording: the column and lever as written give
    /// back the sink as measured, balloon included, 39 fpm rms on the whole
    /// record's fit and the wheels at -494 against the recorded -585 (the
    /// parts' fits are other flares: 50 to 160 over the whole).
    #[test]
    fn the_sr22_plant_replays_the_lemg_flare() {
        let mut p = Plant::recorded_gate(PLANTS[WHOLE]);
        let mut sq = 0.0;
        for r in REC.iter().skip(1) {
            p.recorded(r.0);
            sq += (p.v - r.2).powi(2);
        }
        let rms = (sq / (REC.len() - 1) as f64).sqrt();
        assert!(rms < 45.0, "the whole record's fit: {rms:.1} fpm rms");
        assert!((p.v - REC[REC.len() - 1].2).abs() < 100.0, "{:.0}", p.v);
    }

    /// From every gate on every plant, calm air: the SR22's declared flare
    /// (its model, the thrust held to the last second) lands under 300 fpm
    /// (the worst 273) without pushing a climb, slowing within 5 kt of Vso
    /// or floating; the same model with the lever cut at the flare gate goes
    /// past 300 somewhere; the tracker it flew goes past 400 (the worst
    /// 1,267, a float that stalled onto the runway).
    #[test]
    fn the_sr22_flare_lands_every_gate_on_every_plant() {
        let (mut idle_worst, mut tracker_worst) = (0.0_f64, 0.0_f64);
        for (h, v, trend, ias) in GATES {
            for fit in PLANTS {
                let mut p = Plant::at(fit, h, v, trend, ias);
                let l = land(&mut p, &mut Law::today(), false);
                assert!(l.wheels_fpm > -300.0, "{fit:?} from {h} ft {v} fpm: {l:?}");
                assert!(!l.pushed_climbing, "{fit:?} from {h} ft");
                assert!(l.min_ias > VSO_KTS + 5.0, "{fit:?}: {l:?}");
                // No push on a standing plant: the model floats longer than
                // flight (the King Air's 11.3 s flown against 18 modelled).
                assert!(l.secs < 16.0, "{fit:?}: floated {l:?}");
                let mut p = Plant::at(fit, h, v, trend, ias);
                idle_worst =
                    idle_worst.min(land_with(&mut p, &mut Law::today(), false, None).wheels_fpm);
                let mut p = Plant::at(fit, h, v, trend, ias);
                tracker_worst = tracker_worst
                    .min(land_with(&mut p, &mut Law::as_flown(), false, None).wheels_fpm);
            }
        }
        assert!(idle_worst < -300.0, "idle at the gate: {idle_worst:.0}");
        assert!(tracker_worst < -400.0, "as flown: {tracker_worst:.0}");
    }

    /// Rough air (40 fpm rms, where the rough-air gate starts), every gate
    /// and plant, ten gust seeds each: under 400 fpm (the worst 353), never
    /// a push while climbing.
    #[test]
    fn in_rough_air_the_sr22_flare_still_lands() {
        let (mut worst, mut n) = (0.0_f64, 0);
        for (h, v, trend, ias) in GATES {
            for fit in PLANTS {
                for seed in 1..=10u64 {
                    let mut p = Plant::at(fit, h, v, trend, ias).rough(seed, 40.0);
                    let l = land(&mut p, &mut Law::today(), true);
                    assert!(!l.pushed_climbing, "{fit:?} seed {seed}");
                    worst = worst.min(l.wheels_fpm);
                    n += 1;
                }
            }
        }
        eprintln!("worst {worst:.0} over {n}");
        assert!(worst > -400.0, "{worst:.0} over {n}");
    }
