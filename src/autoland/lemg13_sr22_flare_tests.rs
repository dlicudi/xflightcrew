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
    /// The SR22's landing-configuration stall (POH, profile `vs0_kts`).
    const VSO_KTS: f64 = 60.0;
    /// The final's approach thrust and speed at the gate.
    const APPROACH_THR: f64 = 0.318;

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
                t: 0.0,
                written: vec![(-10.0, fit.pre)],
            }
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
            if self.ias < VSO_KTS {
                self.stall += 800.0 * dt;
            }
            self.v = self.base
                + self.b2
                + self.fit.thr_fpm * (self.thr - APPROACH_THR)
                + self.fit.per_kt * (self.ias - self.ias0)
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
