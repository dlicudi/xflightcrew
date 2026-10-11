    use super::{arrest_targets_fpm, flare_target_with, FlareModel, FlarePower, RetardKey};
    use std::collections::VecDeque;

    /// Where each recorded arrest began: the `arrest_hands` tick.
    #[derive(Clone, Copy)]
    struct Gate {
        radio_ft: f64,
        vvi_fpm: f64,
        trend_fpm_s: f64,
        ias_kts: f64,
        throttle: f64,
        /// The sink this approach's trim held with the elevator at zero
        /// (fitted; LEMG 12 arrived at -774 on a trim that held -496).
        trimmed_sink_fpm: f64,
        /// The sink at the wheels, as flown.
        #[allow(dead_code)] // the record's, read only through Debug
        flown_fpm: f64,
    }

    /// 16:29, 17:04 (its FIRST contact; the scored 278 is after a bounce),
    /// 17:17, 17:25, 17:32, 17:43Z on 13, 18:59Z on 12, and 20:03Z on 13
    /// (the first prediction flare: 277 at the wheels, the lever never
    /// retarded).
    const GATES: [Gate; 8] = [
        Gate {
            radio_ft: 79.2,
            vvi_fpm: -727.0,
            trend_fpm_s: -4.0,
            ias_kts: 145.6,
            throttle: 0.309,
            trimmed_sink_fpm: -853.0,
            flown_fpm: -445.0,
        },
        Gate {
            radio_ft: 81.2,
            vvi_fpm: -862.0,
            trend_fpm_s: -3.0,
            ias_kts: 147.8,
            throttle: 0.300,
            trimmed_sink_fpm: -1021.0,
            flown_fpm: -794.0,
        },
        Gate {
            radio_ft: 79.5,
            vvi_fpm: -748.0,
            trend_fpm_s: -26.0,
            ias_kts: 143.6,
            throttle: 0.357,
            trimmed_sink_fpm: -903.0,
            flown_fpm: -537.0,
        },
        Gate {
            radio_ft: 82.5,
            vvi_fpm: -705.0,
            trend_fpm_s: -56.0,
            ias_kts: 146.1,
            throttle: 0.372,
            trimmed_sink_fpm: -727.0,
            flown_fpm: -769.0,
        },
        Gate {
            radio_ft: 81.7,
            vvi_fpm: -752.0,
            trend_fpm_s: -72.0,
            ias_kts: 145.3,
            throttle: 0.301,
            trimmed_sink_fpm: -950.0,
            flown_fpm: -411.0,
        },
        Gate {
            radio_ft: 78.9,
            vvi_fpm: -789.0,
            trend_fpm_s: -53.0,
            ias_kts: 145.1,
            throttle: 0.301,
            trimmed_sink_fpm: -1096.0,
            flown_fpm: -408.0,
        },
        Gate {
            radio_ft: 81.5,
            vvi_fpm: -774.0,
            trend_fpm_s: 27.0,
            ias_kts: 144.7,
            throttle: 0.441,
            trimmed_sink_fpm: -496.0,
            flown_fpm: -407.0,
        },
        Gate {
            radio_ft: 79.2,
            vvi_fpm: -797.0,
            trend_fpm_s: -2.0,
            ias_kts: 143.6,
            throttle: 0.304,
            trimmed_sink_fpm: -779.0,
            flown_fpm: -277.0,
        },
    ];
    const LEMG_12: usize = 6;
    const PREDICTION_FLOWN: usize = 7;

    #[derive(Clone, Copy, Debug)]
    struct Fit {
        fpm_per_elevator: f64,
        dead_s: f64,
    }
    const LAG1_S: f64 = 0.841;
    const LAG2_S: f64 = 3.088;
    const GROUND_FPM: f64 = 405.3;
    const GROUND_FT: f64 = 43.2;
    const FPM_PER_KT: f64 = 62.7;
    /// The 737 profile's approach speed (`reference.approach_kts`), the
    /// retard's speed key's reference.
    const APPROACH_IAS: f64 = 145.0;

    /// The retard as it flew until issue #76: held while the sink (with a
    /// second of its worsening) was over 400, nothing above 20 ft, and on
    /// 2026-09-28 not before the sink had come inside 185 once or 10 ft;
    /// then timed to the wheels at 0.03 to 0.08 a second. Kept to replay
    /// the landings it flew.
    #[derive(Clone, Copy)]
    struct SinkKeyed {
        inside_fpm: Option<f64>,
        by_ft: Option<f64>,
        max_per_s: f64,
        previous_sink: Option<f64>,
        last: Option<f64>,
        inside: bool,
    }

    impl SinkKeyed {
        /// As flown on 2026-09-28.
        fn as_flown() -> Self {
            SinkKeyed {
                inside_fpm: Some(185.0),
                by_ft: Some(10.0),
                max_per_s: 0.08,
                previous_sink: None,
                last: None,
                inside: false,
            }
        }
        /// LEMG 12's (2026-09-26): by height alone, up to 0.32 a second.
        fn height_only() -> Self {
            SinkKeyed {
                inside_fpm: None,
                by_ft: None,
                max_per_s: 0.32,
                ..Self::as_flown()
            }
        }
        fn step(&mut self, measured: f64, vvi: f64, dt: f64, radio_ft: f64) -> f64 {
            let sink = -vvi;
            let worsening = self
                .previous_sink
                .map_or(0.0, |old| ((sink - old) / dt).max(0.0));
            self.previous_sink = Some(sink);
            self.inside |= self.inside_fpm.is_some_and(|f| sink <= f);
            let floor = self.by_ft.is_some_and(|h| radio_ft <= h);
            let waiting = self.inside_fpm.is_some() && !self.inside && !floor;
            let current = measured.clamp(0.0, 1.0).min(self.last.unwrap_or(1.0));
            let rate = if waiting || radio_ft > 20.0 {
                0.0
            } else {
                let to_touch_s = radio_ft.max(0.0) / sink.max(100.0) * 60.0;
                (current / to_touch_s.max(0.5)).clamp(0.03, self.max_per_s)
            };
            let next = if self.last.is_none() || sink.max(sink + worsening) > 400.0 {
                current
            } else {
                (current - rate * dt.min(1.0)).max(0.0)
            };
            self.last = Some(next);
            next
        }
    }

    /// Which retard a replay flies.
    enum Retard {
        Keyed(FlarePower, crate::aircraft::FlarePowerProfile),
        Sink(SinkKeyed),
        /// Height, time and speed as flown on 2026-09-28 after issue #76,
        /// before the float key (issue #146).
        NoFloat(NoFloatKeyed, crate::aircraft::FlarePowerProfile),
    }

    /// The keyed retard without the float key, as flown at EGBB 15
    /// (2026-09-28): nothing above the retard height, nothing while more
    /// than `retard_at_s` from the wheels (a float counted at 100 fpm),
    /// and once due, nothing while slow. Kept to replay the landing it flew.
    #[derive(Default, Clone, Copy)]
    struct NoFloatKeyed {
        last: Option<f64>,
        due: bool,
    }

    impl NoFloatKeyed {
        fn step(
            &mut self,
            cfg: crate::aircraft::FlarePowerProfile,
            measured: f64,
            vvi: f64,
            dt: f64,
            radio_ft: f64,
            ias: f64,
        ) -> f64 {
            let current = measured.clamp(0.0, 1.0).min(self.last.unwrap_or(1.0));
            let slow = ias < APPROACH_IAS - cfg.retard_slow_kts.unwrap_or(5.0);
            let above = cfg.retard_below_ft.is_some_and(|h| radio_ft > h);
            let to_touch_s = (radio_ft.max(0.0) / (-vvi).max(100.0) * 60.0).max(0.5);
            let fast = cfg.retard_fast_kts.is_some_and(|k| ias > APPROACH_IAS + k);
            self.due |= !above && (fast || cfg.retard_at_s.is_none_or(|s| to_touch_s <= s));
            let next = if self.last.is_none() || above || !self.due || slow {
                current
            } else {
                let fastest = cfg
                    .retard_max_per_s
                    .unwrap_or(cfg.reduction_per_second)
                    .max(0.03);
                let rate = (current / to_touch_s).clamp(0.03, fastest);
                (current - rate * dt.min(1.0)).max(0.0)
            };
            self.last = Some(next);
            next
        }
    }

    impl Retard {
        /// The 737 profile's, today's law.
        fn profile() -> Self {
            Retard::Keyed(
                FlarePower::default(),
                profile_737().flare_power.expect("the 737 retards"),
            )
        }
        /// ...and the retard as it flew at EGBB 15.
        fn no_float() -> Self {
            Retard::NoFloat(
                NoFloatKeyed::default(),
                profile_737().flare_power.expect("the 737 retards"),
            )
        }
        fn step(&mut self, lever: f64, vvi: f64, dt: f64, h: f64, ias: f64) -> Option<f64> {
            match self {
                Retard::Keyed(p, cfg) => p.step(*cfg, lever, vvi, dt, h, Some(ias), Some(APPROACH_IAS)),
                Retard::Sink(s) => Some(s.step(lever, vvi, dt, h)),
                Retard::NoFloat(k, cfg) => Some(k.step(*cfg, lever, vvi, dt, h, ias)),
            }
        }
    }
    /// Each flare's own gain and dead time, in the order of [`GATES`].
    const PLANTS: [Fit; 8] = [
        Fit {
            fpm_per_elevator: 4814.0,
            dead_s: 1.37,
        },
        Fit {
            fpm_per_elevator: 5097.0,
            dead_s: 1.16,
        },
        Fit {
            fpm_per_elevator: 4990.0,
            dead_s: 1.29,
        },
        Fit {
            fpm_per_elevator: 6429.0,
            dead_s: 1.43,
        },
        Fit {
            fpm_per_elevator: 4622.0,
            dead_s: 1.50,
        },
        Fit {
            fpm_per_elevator: 4093.0,
            dead_s: 2.17,
        },
        Fit {
            fpm_per_elevator: 4021.0,
            dead_s: 1.77,
        },
        Fit {
            fpm_per_elevator: 4099.0,
            dead_s: 1.63,
        },
    ];

    struct Plant {
        fit: Fit,
        trimmed: f64,
        x1: f64,
        x2: f64,
        ias: f64,
        ias0: f64,
        t: f64,
        queue: VecDeque<(f64, f64)>,
        elev: f64,
        vvi: f64,
        /// The air: sink (fpm) and speed change (kt/s) the fit does not
        /// explain, taken from a recording ([`air_of`]); zero is still air.
        air_fpm: f64,
        air_kts_s: f64,
    }

    impl Plant {
        fn at(g: Gate, fit: Fit) -> Self {
            let x2 = g.vvi_fpm - g.trimmed_sink_fpm;
            Plant {
                fit,
                trimmed: g.trimmed_sink_fpm,
                x1: x2 + g.trend_fpm_s * LAG2_S,
                x2,
                ias: g.ias_kts,
                ias0: g.ias_kts,
                t: 0.0,
                queue: VecDeque::new(),
                elev: 0.0,
                vvi: g.vvi_fpm,
                air_fpm: 0.0,
                air_kts_s: 0.0,
            }
        }
        fn write(&mut self, elev: f64) {
            self.queue.push_back((self.t + self.fit.dead_s, elev));
        }
        /// `ias` Some = the recorded speed (a replay); None = the speed
        /// model flies it.
        fn step(&mut self, lever: f64, h: f64, ias: Option<f64>, dt: f64) {
            while self.queue.front().is_some_and(|&(t, _)| t <= self.t) {
                self.elev = self.queue.pop_front().map_or(0.0, |(_, e)| e);
            }
            let ground = GROUND_FPM * (1.0 - h.max(0.0) / GROUND_FT).max(0.0);
            self.x1 += (self.fit.fpm_per_elevator * self.elev - ground - self.x1) * dt / LAG1_S;
            self.x2 += (self.x1 - self.x2) * dt / LAG2_S;
            self.ias = match ias {
                Some(v) => v,
                None => self.ias + (speed_model(lever, self.elev, self.vvi) + self.air_kts_s) * dt,
            };
            self.vvi = self.trimmed + self.x2 + FPM_PER_KT * (self.ias - self.ias0) + self.air_fpm;
            self.t += dt;
        }
    }

    /// The fitted speed model, kt/s.
    fn speed_model(lever: f64, elev: f64, vvi: f64) -> f64 {
        -1.08 + 2.99 * lever - 2.05 * elev - 0.40 * vvi / 1000.0
    }

    /// The fitted plant IS the recorded aircraft: LEMG 12's own elevator,
    /// replayed open loop through its own flare's plant, gives back its
    /// sink, the plateau at -300 and the growth to -383 over the last 15 ft
    /// included.
    #[test]
    fn the_737_plant_is_fitted_to_its_own_flares() {
        // (seconds from the gate, radio ft, sink recorded, elevator written,
        // IAS).
        const REC: [(f64, f64, f64, f64, f64); 57] = [
            (0.00, 81.5, -774.0, 0.156, 144.7),
            (0.30, 76.7, -766.0, 0.166, 145.0),
            (0.60, 73.2, -756.0, 0.141, 145.3),
            (1.00, 69.8, -739.0, 0.139, 145.6),
            (1.30, 66.8, -718.0, 0.122, 145.9),
            (1.60, 63.8, -686.0, 0.109, 145.9),
            (2.00, 61.3, -650.0, 0.088, 145.7),
            (2.30, 58.8, -605.0, 0.079, 145.3),
            (2.70, 56.7, -561.0, 0.068, 145.0),
            (3.00, 55.1, -513.0, 0.055, 144.7),
            (3.30, 53.7, -464.0, 0.037, 144.6),
            (3.70, 52.5, -410.0, 0.012, 144.6),
            (4.00, 51.6, -362.0, -0.000, 144.8),
            (4.40, 50.8, -312.0, -0.000, 145.0),
            (4.70, 50.1, -276.0, -0.000, 145.2),
            (5.10, 49.3, -248.0, -0.000, 145.4),
            (5.40, 48.6, -233.0, -0.000, 145.5),
            (5.80, 47.7, -228.0, -0.000, 145.5),
            (6.10, 46.9, -231.0, -0.000, 145.5),
            (6.60, 45.2, -248.0, -0.000, 145.3),
            (7.00, 43.9, -263.0, -0.000, 145.3),
            (7.30, 42.5, -276.0, -0.000, 145.3),
            (7.70, 40.6, -290.0, -0.000, 145.4),
            (8.00, 39.1, -301.0, -0.000, 145.5),
            (8.30, 37.0, -310.0, 0.012, 145.7),
            (8.50, 36.0, -314.0, 0.004, 145.8),
            (8.80, 34.7, -317.0, 0.012, 146.0),
            (9.00, 33.5, -319.0, 0.017, 146.2),
            (9.20, 32.4, -319.0, 0.019, 146.4),
            (9.40, 31.4, -320.0, 0.022, 146.6),
            (9.60, 30.7, -319.0, 0.022, 146.8),
            (9.80, 30.0, -319.0, 0.020, 147.0),
            (10.00, 29.1, -318.0, 0.022, 147.1),
            (10.30, 28.2, -317.0, 0.026, 147.3),
            (10.50, 27.4, -317.0, 0.027, 147.3),
            (10.70, 26.5, -316.0, 0.031, 147.4),
            (10.90, 25.7, -315.0, 0.033, 147.4),
            (11.10, 24.9, -314.0, 0.037, 147.4),
            (11.30, 24.2, -313.0, 0.039, 147.4),
            (11.60, 23.1, -311.0, 0.048, 147.5),
            (11.80, 22.2, -308.0, 0.050, 147.6),
            (12.00, 21.1, -306.0, 0.054, 147.6),
            (12.20, 20.1, -302.0, 0.061, 147.7),
            (12.40, 19.1, -299.0, 0.062, 147.8),
            (12.60, 18.2, -297.0, 0.066, 147.8),
            (12.90, 17.1, -295.0, 0.073, 147.8),
            (13.10, 16.1, -294.0, 0.068, 147.9),
            (13.30, 15.0, -295.0, 0.064, 147.8),
            (13.50, 13.7, -299.0, 0.065, 147.8),
            (13.70, 12.6, -303.0, 0.066, 147.8),
            (13.90, 11.4, -309.0, 0.070, 147.8),
            (14.10, 10.1, -317.0, 0.074, 147.8),
            (14.30, 8.8, -327.0, 0.081, 147.7),
            (14.50, 7.6, -337.0, 0.086, 147.7),
            (14.80, 6.1, -352.0, 0.095, 147.6),
            (14.90, 4.6, -366.0, 0.103, 147.5),
            (15.10, 3.1, -383.0, 0.111, 147.4),
        ];
        let g = GATES[LEMG_12];
        let mut p = Plant::at(
            Gate {
                trimmed_sink_fpm: -468.0,
                ..g
            },
            PLANTS[LEMG_12],
        );
        let dt = 0.01;
        let (mut sq, mut i) = (0.0, 0);
        let mut at_20ft = None;
        while i < REC.len() {
            if p.t + 1e-9 >= REC[i].0 {
                sq += (p.vvi - REC[i].2).powi(2);
                if at_20ft.is_none() && REC[i].1 <= 20.5 {
                    at_20ft = Some(p.vvi);
                }
                p.write(REC[i].3);
                i += 1;
            }
            let h = REC[i.saturating_sub(1)].1;
            let ias = REC[i.saturating_sub(1)].4;
            p.step(g.throttle, h, Some(ias), dt);
        }
        let rms = (sq / REC.len() as f64).sqrt();
        assert!(
            rms < 35.0,
            "the replay follows the record: {rms:.0} fpm rms"
        );
        let at_20ft = at_20ft.unwrap();
        assert!(
            (at_20ft + 302.0).abs() < 50.0,
            "the plateau at 20 ft (-302): {at_20ft:.0}"
        );
        assert!(
            p.vvi < -350.0,
            "and the sink grows over the last feet as it did (-383): {:.0}",
            p.vvi
        );
    }

    /// The first model law (2,600 fpm, two 1 s lags, 1.8 s, the exponential
    /// target 3.5 s ahead), as flown at LEMG 12, kept here to prove the
    /// plant: flown from LEMG 12's gate through LEMG 12's plant it lands
    /// where the aircraft did.
    struct FirstModel {
        t: f64,
        hist: VecDeque<(f64, f64)>,
        b1: f64,
        b2: f64,
        el: f64,
        r1: Option<f64>,
        r2: f64,
        u: Option<f64>,
    }
    impl FirstModel {
        fn new() -> Self {
            FirstModel {
                t: 0.0,
                hist: VecDeque::new(),
                b1: 0.0,
                b2: 0.0,
                el: 0.0,
                r1: None,
                r2: 0.0,
                u: None,
            }
        }
        fn step(&mut self, h: f64, gs: f64, vvi: f64, dt: f64) -> f64 {
            let (k, lag, dead, lead, lam, obs, tau) = (2600.0, 1.0, 1.8, 3.5, 0.6, 0.3, 5.0);
            let n = (dt / 0.02).round().max(1.0) as usize;
            let step = dt / n as f64;
            for _ in 0..n {
                self.t += step;
                self.hist.push_back((self.t, self.el));
                while self.hist.len() > 1 && self.hist[1].0 <= self.t - dead {
                    self.hist.pop_front();
                }
                let d = match self.hist.front() {
                    Some(&(t0, e)) if t0 <= self.t - dead => e,
                    _ => 0.0,
                };
                self.b1 += (k * d - self.b1) * step / lag;
                self.b2 += (self.b1 - self.b2) * step / lag;
            }
            let now = vvi - self.b2;
            let u = self.u.map_or(now, |u| u + (now - u) * (dt / obs).min(1.0));
            self.u = Some(u);
            let nominal = -(gs / 60.0) * super::gp_ft_per_nm();
            let (_, flare_vs) = arrest_targets_fpm(nominal);
            let ahead = (h + vvi.min(0.0) * lead / 60.0).max(0.0);
            let target = flare_target_with(ahead, flare_vs, nominal, Some(tau));
            let r1 = match self.r1 {
                Some(r) => r,
                None => {
                    self.r2 = vvi;
                    vvi
                }
            };
            let d1 = (r1 - self.r2) / lam;
            let d2 = ((target - r1) / lam - d1) / lam;
            let a = (dt / lam).min(1.0);
            self.r1 = Some(r1 + (target - r1) * a);
            self.r2 += (r1 - self.r2) * a;
            let want = self.r2 + 2.0 * lag * d1 + lag * lag * d2;
            self.el = ((want - u) / k).clamp(0.0, 0.45);
            self.el
        }
    }

    enum Law {
        First(FirstModel),
        Model(FlareModel),
        /// The model flare as flown in rough air (the column held from the
        /// flare gate).
        Rough(FlareModel),
    }

    struct Landing {
        sink_fpm: f64,
        float_m: f64,
        climbed: bool,
        max_elev: f64,
        min_elev: f64,
        lever_at_wheels: f64,
    }

    fn profile_737() -> crate::aircraft::AircraftProfile {
        crate::aircraft::load("boeing_737_800").expect("the 737 profile loads")
    }

    fn model() -> FlareModel {
        FlareModel::from_profile(profile_737().flare_elevator).expect("the 737 declares its model")
    }

    /// The model flare as flown from 2026-09-26 to 2026-10-06 (LEMG 13,
    /// EGBB 15, LXGB 09): the exponential's 6 s, before LXGB's float
    /// shortened it to 5.5. The replays of those flights fly it.
    fn model_as_flown() -> FlareModel {
        let mut p = profile_737().flare_elevator.expect("the 737's block");
        p.tau_s = Some(6.0);
        FlareModel::from_profile(Some(p)).expect("the 737 declares its model")
    }

    /// Fly one recorded gate to the wheels: the loop's 0.22 s tick (LEMG
    /// 12's), the flare gate from the loop's own function, the profile's
    /// `flare_power` below it (the model's) or LEMG 12's height-only retard
    /// (the first law's), the elevator reaching the aircraft a dead time
    /// after it is written, the speed on the fitted speed model.
    fn fly(g: Gate, fit: Fit, law: Law) -> Landing {
        let power = if matches!(law, Law::First(_)) {
            Retard::Sink(SinkKeyed::height_only())
        } else {
            Retard::profile()
        };
        fly_with(g, fit, law, power)
    }

    fn fly_with(g: Gate, fit: Fit, mut law: Law, mut power: Retard) -> Landing {
        let mut p = Plant::at(g, fit);
        let (tick, dt) = (0.22, 0.01);
        let mut h = g.radio_ft;
        let mut lever = g.throttle;
        let mut next = 0.0;
        let mut out = Landing {
            sink_fpm: 0.0,
            float_m: 0.0,
            climbed: false,
            max_elev: 0.0,
            min_elev: 0.0,
            lever_at_wheels: 0.0,
        };
        while h > 0.0 && p.t < 40.0 {
            if p.t + 1e-9 >= next {
                next += tick;
                let gs = p.ias - 6.0;
                let v = p.vvi;
                let (flare_ft, _) = super::arrest_gates_ft(25.0, gs);
                let e = match &mut law {
                    Law::First(m) => m.step(h, gs, v, tick),
                    Law::Model(m) => m.step(h, gs, v, tick, h <= flare_ft, false).elev,
                    Law::Rough(m) => m.step(h, gs, v, tick, h <= flare_ft, true).elev,
                };
                out.max_elev = out.max_elev.max(e);
                out.min_elev = out.min_elev.min(e);
                p.write(e);
                if h <= flare_ft {
                    if let Some(l) = power.step(lever, v, tick, h, p.ias) {
                        lever = l;
                    }
                }
            }
            p.step(lever, h, None, dt);
            h += p.vvi / 60.0 * dt;
            out.float_m += (p.ias - 6.0) * 0.514_444 * dt;
            out.climbed |= p.vvi > 0.0;
        }
        out.sink_fpm = -p.vvi;
        out.lever_at_wheels = lever;
        out
    }

    /// LEMG 12, closed loop: the first model law from its own gate through
    /// its own flare's plant lands at the 407 the aircraft did. A plant that
    /// landed this soft would be a plant that could not be trusted.
    #[test]
    fn the_first_model_law_lands_lemg_12_where_it_did() {
        let l = fly(
            GATES[LEMG_12],
            PLANTS[LEMG_12],
            Law::First(FirstModel::new()),
        );
        assert!(
            (l.sink_fpm - 407.0).abs() < 30.0,
            "the first model law flown at LEMG 12 lands at 407: {:.0}",
            l.sink_fpm
        );
        // And across the seven flares it was never inside the limit with
        // margin: the plant fails it as the runway did.
        let worst = GATES
            .iter()
            .zip(PLANTS.iter())
            .map(|(g, f)| fly(*g, *f, Law::First(FirstModel::new())).sink_fpm)
            .fold(0.0, f64::max);
        assert!(
            worst > crate::scorecard::TOUCHDOWN_MAX_SINK_FPM,
            "{worst:.0}"
        );
    }

    /// The prediction flare, the profile as declared, from all seven
    /// recorded gates through all seven flares' own plants (49 landings):
    /// every one well under the scorecard's 400, none climbing, the column
    /// never pushed nor past its stop, no float past 1,200 m. LEMG 12's own
    /// gate and plant, the landing flown at 407: under 300.
    #[test]
    fn the_737_model_flare_lands_every_recorded_gate_under_400() {
        // The old law's own standard: the scorecard's limit is 240 since
        // 2026-10-11, and the 737 is held to it once it flies the layered
        // flare (docs/tasks/autonomy-layers).
        let limit = 400.0;
        for (k, fit) in PLANTS.iter().enumerate() {
            for (i, g) in GATES.iter().enumerate() {
                let l = fly(*g, *fit, Law::Model(model()));
                assert!(
                    l.sink_fpm < limit - 60.0,
                    "gate {i}, plant {k} {fit:?}: {:.0} fpm",
                    l.sink_fpm
                );
                assert!(
                    l.sink_fpm > 100.0,
                    "gate {i}, plant {k}: a landing, not a float: {:.0}",
                    l.sink_fpm
                );
                assert!(!l.climbed, "gate {i}, plant {k}: ballooned");
                assert!(
                    l.min_elev >= 0.0 && l.max_elev <= 0.45 + 1e-9,
                    "gate {i}: {} {}",
                    l.min_elev,
                    l.max_elev
                );
                assert!(
                    l.float_m < 1200.0,
                    "gate {i}, plant {k}: floated {:.0} m",
                    l.float_m
                );
            }
        }
        let l = fly(GATES[LEMG_12], PLANTS[LEMG_12], Law::Model(model()));
        assert!(
            l.sink_fpm < 300.0,
            "LEMG 12 on the new law: {:.0}",
            l.sink_fpm
        );
        // The prediction flare as flown (277 at the wheels, 0.30 of lever):
        // with the height, time and speed keys the lever is coming off at the
        // wheels and the sink it rebuilds is small.
        let l = fly(
            GATES[PREDICTION_FLOWN],
            PLANTS[PREDICTION_FLOWN],
            Law::Model(model()),
        );
        assert!(l.sink_fpm < 300.0, "{:.0}", l.sink_fpm);
        assert!(
            l.lever_at_wheels < 0.30 && l.lever_at_wheels > 0.0,
            "walking off, not cut: {:.2}",
            l.lever_at_wheels
        );
    }

    /// Issue #76: the retard keyed on height, time to the wheels and speed,
    /// never on the sink. Replayed through the fitted 737 flare plant (its
    /// dead time, both lags, the ground and the speed model) and the air
    /// each recording carries: all sixteen LEMG 13 approaches of 2026-09-28
    /// (sorties 1790586843 to 1790590296), and every recorded gate of
    /// 2026-09-26 through every flare's own plant (64 landings).
    ///
    /// As flown, the sink-keyed retard started off when the ARREST brought
    /// the sink inside 185 (15.8 ft in 1790589309, mid-float) and stopped
    /// again when the sink it cost came back past 400: through the plant the
    /// sixteen land at 268 fpm mean, 300 worst. Keyed on what it is for they
    /// land softer on average and at worst, and none climbs.
    #[test]
    fn the_retard_is_keyed_on_height_time_and_speed_never_the_sink() {
        let cfg = profile_737().flare_power.expect("the 737 retards");
        assert_eq!(cfg.retard_below_ft, Some(20.0));
        assert_eq!(cfg.retard_at_s, Some(2.0));
        let (mut old_worst, mut old_sum, mut new_worst, mut new_sum) = (0.0f64, 0.0, 0.0f64, 0.0);
        for s in LEMG_13_SORTIES {
            let r = recording(s);
            let a = r.arrest();
            let rough = r.air().rough();
            let old = fly_as_flown(&a, CALM_FIT, rough, true);
            let new = fly_through_air(&a, CALM_FIT, rough, true);
            assert!(!new.climbed, "{s}: climbed");
            assert!(new.sink_fpm > 150.0, "{s}: a landing, not a float: {:.0}", new.sink_fpm);
            assert!(new.float_m < 1200.0, "{s}: floated {:.0} m", new.float_m);
            old_worst = old_worst.max(old.sink_fpm);
            new_worst = new_worst.max(new.sink_fpm);
            old_sum += old.sink_fpm;
            new_sum += new.sink_fpm;
        }
        let n = LEMG_13_SORTIES.len() as f64;
        assert!(
            (old_sum / n - 268.0).abs() < 10.0 && (old_worst - 300.0).abs() < 10.0,
            "the old retard replays as characterised: {:.0} mean, {old_worst:.0} worst",
            old_sum / n
        );
        assert!(
            new_sum / n < old_sum / n - 15.0 && new_worst < old_worst,
            "{:.0} mean, {new_worst:.0} worst against {:.0}, {old_worst:.0}",
            new_sum / n,
            old_sum / n
        );
        // The 2026-09-26 gates through every plant: under the scorecard's
        // 400 with margin, none climbing, no float past 1,200 m.
        for (k, fit) in PLANTS.iter().enumerate() {
            for (i, g) in GATES.iter().enumerate() {
                let l = fly(*g, *fit, Law::Model(model()));
                assert!(l.sink_fpm < 340.0, "gate {i}, plant {k}: {:.0}", l.sink_fpm);
                assert!(!l.climbed && l.float_m < 1200.0, "gate {i}, plant {k}");
            }
        }
    }

    /// The keys one by one on the 737's numbers: above 20 ft nothing; below
    /// it, nothing while more than 2 s from the wheels unless faster than the approach speed;
    /// then timed to the wheels at no more than 0.04 a second; and a flare
    /// 5 kt slow keeps its thrust. LEMG 12 (2026-09-26) sinking 300 at
    /// 20 ft: 4 s from the wheels, so the lever stays (it went 0.44 to 0.16
    /// by height alone and landed at 407).
    #[test]
    fn the_737_retard_keys() {
        let cfg = profile_737().flare_power.expect("the 737 retards");
        let app = Some(APPROACH_IAS);
        let on = Some(APPROACH_IAS);
        let mut p = FlarePower::default();
        assert_eq!(p.step(cfg, 0.44, -300.0, 0.22, 25.0, on, app), Some(0.44));
        assert_eq!(p.step(cfg, 0.44, -300.0, 0.22, 21.0, on, app), Some(0.44));
        assert_eq!(p.last_key, Some(RetardKey::Height));
        assert_eq!(p.step(cfg, 0.44, -300.0, 0.22, 19.0, on, app), Some(0.44));
        assert_eq!(p.last_key, Some(RetardKey::Time));
        // 10 ft at 300 fpm is 2 s: off it comes, at no more than 0.04 a second.
        let a = p.step(cfg, 0.44, -300.0, 0.22, 10.0, on, app).unwrap();
        assert!(a < 0.44 && a >= 0.44 - 0.04 * 0.22 - 1e-9, "{a}");
        assert_eq!(p.last_key, Some(RetardKey::Timed));
        // Once due it stays due: a float after it does not re-hold the lever.
        let b = p.step(cfg, 0.44, -60.0, 0.22, 9.5, on, app).unwrap();
        assert!(b < a, "{a} -> {b}");
        // Fast: 2 kt over the approach speed starts it at 19 ft.
        let mut f = FlarePower::default();
        f.step(cfg, 0.3, -300.0, 0.22, 19.0, Some(148.0), app);
        assert!(f.step(cfg, 0.3, -300.0, 0.22, 18.0, Some(148.0), app).unwrap() < 0.3);
        // Slow: 6 kt under keeps it, however close the wheels.
        let mut s = FlarePower::default();
        s.step(cfg, 0.3, -300.0, 0.22, 5.0, Some(139.0), app);
        assert_eq!(s.step(cfg, 0.3, -300.0, 0.22, 4.0, Some(139.0), app), Some(0.3));
        assert_eq!(s.last_key, Some(RetardKey::Slow));
    }

    /// Only an airframe that declares a model flies it (the 737, and the
    /// SF50 since LEMG 12, issue #163); a block without one keeps the
    /// tracker (and a bad model is dropped, never the rest of the block).
    #[test]
    fn only_a_declared_model_flies_the_model_flare() {
        assert!(FlareModel::from_profile(profile_737().flare_elevator).is_some());
        let sf50 = crate::aircraft::load("cirrus_vision_sf50").expect("sf50");
        assert!(FlareModel::from_profile(sf50.flare_elevator).is_some());
        let tracker_only = crate::aircraft::FlareElevatorProfile {
            model: None,
            ..sf50.flare_elevator.expect("the SF50's block")
        };
        assert!(FlareModel::from_profile(Some(tracker_only)).is_none());
        assert!(FlareModel::from_profile(None).is_none());
        let mut p = profile_737().flare_elevator.unwrap();
        if let Some(m) = p.model.as_mut() {
            m.dead_s = 9.0;
        }
        let v = p.valid().expect("the block stands");
        assert!(v.model.is_none(), "the bad model is dropped");
        assert_eq!(v.max, 0.45);
    }

    /// A read that fails is no measurement: the model coasts, nothing is
    /// written, and the next tick decides again.
    #[test]
    fn a_missed_read_coasts_the_model() {
        let mut m = model();
        let first = m.step(81.5, 139.0, -774.0, 0.22, false, false);
        assert!(
            first.elev > 0.1,
            "the gate kicks the column: {}",
            first.elev
        );
        m.coast(0.22, 78.0);
        let again = m.step(76.0, 139.0, -760.0, 0.22, false, false);
        assert!(again.elev >= 0.0 && again.elev <= 0.45);
        assert!(again.unexplained_fpm.is_finite() && again.ahead_ft < 76.0);
    }

    // ---- LEMG 13, 2026-09-28: the flare in rough air (issue #146) ----

    /// Three recorded LEMG 13 finals (the autoland ticks from 400 ft to the
    /// wheels, and the flight model's touchdown): 1790589309 (422 fpm) and
    /// 1790589773 (335) in gusts, 1790588609 (236) in calm air.
    const LEMG_13_FLARES: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/landings/lemg13_gusty_flares.jsonl"));
    /// The other thirteen LEMG 13 approaches of that night's build (sorties
    /// 1790586843 to 1790590296, 225 to 315 fpm), the same fields.
    const LEMG_13_CALM_FLARES: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/landings/lemg13_calm_flares.jsonl"));
    /// Every one of the sixteen, in order.
    const LEMG_13_SORTIES: [&str; 16] = [
        "1790586843",
        "1790587073",
        "1790587296",
        "1790587535",
        "1790587771",
        "1790588007",
        "1790588156",
        "1790588305",
        "1790588460",
        "1790588609",
        "1790589309",
        "1790589541",
        "1790589773",
        "1790590005",
        "1790590150",
        "1790590296",
    ];

    /// EGBB 15 (737, EGKK-EGBB full auto, 2026-09-28, sortie 1790601822):
    /// the autoland ticks from 400 ft to the wheels and the flight model's
    /// touchdown (288 fpm). Rough air (41 fpm rms); the column held 0.28
    /// through a float at 22.7 ft, eased to 0.20 at +1 fpm; the lever stayed
    /// 0.30 from the flare to 2.3 ft (retard keys height, time, slow).
    const EGBB_15_FLOAT: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/landings/egbb15_rough_float.jsonl"));

    /// EGBB 15's own flare, fitted open loop to the recorded elevator, lever
    /// and speed from the arrest gate to the wheels (36 fpm rms): a strong
    /// elevator, 6,300 fpm per unit through 1.7 s of dead time, on a trim
    /// that held -1,025. The law's 3,500 is the weakest flare's on purpose,
    /// so the pull it holds in rough air is more than this aircraft needs.
    const EGBB_FIT: Fit = Fit {
        fpm_per_elevator: 6300.0,
        dead_s: 1.7,
    };
    const EGBB_TRIM_FPM: f64 = -1025.0;

    #[test]
    fn the_egbb_plant_is_fitted_to_its_own_flare() {
        let a = recording("1790601822").arrest();
        let mut p = Plant::at(gate_of(&a, Some(EGBB_TRIM_FPM)), EGBB_FIT);
        let mut sq = 0.0;
        for w in a.windows(2) {
            let (t0, t1) = (w[0].t - a[0].t, w[1].t - a[0].t);
            p.write(w[0].elev);
            while p.t + 1e-9 < t1 {
                let f = (p.t - t0) / (t1 - t0);
                p.step(
                    w[0].lever,
                    w[0].radio,
                    Some(w[0].ias + (w[1].ias - w[0].ias) * f),
                    0.01,
                );
            }
            sq += (p.vvi - w[1].vvi).powi(2);
        }
        let rms = (sq / (a.len() - 1) as f64).sqrt();
        assert!(rms < 40.0, "the fit follows the record: {rms:.0} fpm rms");
        // The law that flew is the law in the harness: fed the recorded
        // sinks in rough air, easing on any climb, it writes what was
        // written (the 0.28 hold, the ease to 0.20 at +1 fpm, the cap).
        let mut m = model_as_flown().easing_on_any_climb();
        let mut worst: f64 = 0.0;
        for (n, k) in a.iter().enumerate() {
            let dt = if n == 0 { 0.22 } else { k.t - a[n - 1].t };
            let e = m.step(k.radio, k.ias - 4.0, k.vvi, dt, k.flare, true).elev;
            worst = worst.max((e - k.elev).abs());
        }
        assert!(worst < 0.04, "elevator {worst:.3} off the record");
    }

    /// Issues #76 and #146, EGBB 15 (sortie 1790601822): the column held
    /// 0.28 in rough air and levelled the 737 at 22.7 ft on 0.30 of lever;
    /// the retard, keyed on height (over 20 ft), then time (a float is 13 s
    /// from the wheels), then speed (the float bled it 7 kt under), never
    /// moved the lever, and the float ran on its thrust. Through the flare's
    /// own fit and the air it carried, as flown: 5.6 s under 100 fpm of sink,
    /// 940 m from the flare gate, the lever at 0.30 at the wheels. With the
    /// float key (a sink under 100 fpm now or 1.5 s ahead on its trend takes
    /// the lever off at the airframe's quickest, at any height or speed) and
    /// a column that eases only for a climb past 25 fpm: no float, 280 m
    /// shorter, the lever off at the wheels, a landing under 350 fpm.
    #[test]
    fn the_egbb_float_takes_the_thrust_off_and_lands_without_it() {
        let r = recording("1790601822");
        assert_eq!(r.touchdown_fpm, 288.0);
        assert!(r.air().rough(), "{:?}", r.air().rms_fpm());
        let a = r.arrest();
        let trim = Some(EGBB_TRIM_FPM);
        let flown = fly_through_air_with(&a, EGBB_FIT, trim, true, true, Retard::no_float(), true);
        assert!(
            flown.float_s > 4.0,
            "as flown it floats: {:.1} s",
            flown.float_s
        );
        assert!(
            (flown.lever_at_wheels - a[0].lever).abs() < 1e-9,
            "and the lever never moves: {:.3}",
            flown.lever_at_wheels
        );
        let f = fly_through_air_with(&a, EGBB_FIT, trim, true, true, Retard::profile(), false);
        assert!(f.float_s < 0.2, "no float: {:.1} s", f.float_s);
        assert!(
            f.float_m < flown.float_m - 200.0,
            "{:.0} m against {:.0}",
            f.float_m,
            flown.float_m
        );
        assert!(
            f.lever_at_wheels < 0.05,
            "the lever off: {:.3}",
            f.lever_at_wheels
        );
        assert!(
            f.sink_fpm > 150.0 && f.sink_fpm < 350.0,
            "a landing: {:.0}",
            f.sink_fpm
        );
        assert!(!f.climbed && f.eased < 1e-9, "never forward, never up");
        // Not one plant's luck: around the fit (elevator 5,700 to 6,900,
        // dead time 1.6 to 1.8 s) the worst landing is softer, the mean float
        // shorter, and the lever always comes off.
        let (mut old_worst, mut new_worst) = (0.0f64, 0.0f64);
        let (mut old_m, mut new_m, mut n) = (0.0, 0.0, 0.0);
        for k in [5700.0, 6300.0, 6900.0] {
            for d in [1.6, 1.7, 1.8] {
                let fit = Fit {
                    fpm_per_elevator: k,
                    dead_s: d,
                };
                let old = fly_through_air_with(&a, fit, trim, true, true, Retard::no_float(), true);
                let new = fly_through_air_with(&a, fit, trim, true, true, Retard::profile(), false);
                old_worst = old_worst.max(old.sink_fpm);
                new_worst = new_worst.max(new.sink_fpm);
                old_m += old.float_m;
                new_m += new.float_m;
                n += 1.0;
                assert!(
                    new.lever_at_wheels < 0.11,
                    "{k} {d}: {:.3}",
                    new.lever_at_wheels
                );
                assert!(new.sink_fpm < 430.0, "{k} {d}: {:.0}", new.sink_fpm);
            }
        }
        assert!(
            new_worst < old_worst - 100.0,
            "worst {new_worst:.0} against {old_worst:.0}"
        );
        assert!(
            new_m < old_m - 50.0 * n,
            "mean float {:.0} m against {:.0}",
            new_m / n,
            old_m / n
        );
    }

    /// The float key on the 737's numbers: a float at 22.7 ft (over the
    /// retard height), 5 kt slow and far from the wheels, walks the lever
    /// off at 0.04 a second; a sink easing fast enough to float within 1.5 s
    /// is one already; slow defers only a retard not yet begun and a sink
    /// still sinking; a +1 fpm float is no balloon for the lever.
    #[test]
    fn a_float_takes_the_thrust_off_at_any_height_and_speed() {
        let cfg = profile_737().flare_power.expect("the 737 retards");
        let app = Some(APPROACH_IAS);
        let slow = Some(APPROACH_IAS - 6.0);
        let mut p = FlarePower::default();
        assert_eq!(p.step(cfg, 0.30, -140.0, 0.25, 22.9, slow, app), Some(0.30));
        let a = p.step(cfg, 0.30, -86.0, 0.25, 22.7, slow, app).unwrap();
        assert_eq!(p.last_key, Some(RetardKey::Float));
        assert_eq!(cfg.retard_float_per_s, Some(0.08));
        assert!((a - (0.30 - 0.08 * 0.25)).abs() < 1e-9, "{a}");
        // The sink rebuilding under it: the retard goes on, timed, and the
        // speed never holds it again.
        let b = p.step(cfg, 0.30, -250.0, 0.25, 12.0, slow, app).unwrap();
        assert!(b < a, "{a} -> {b}");
        assert_eq!(p.last_key, Some(RetardKey::Timed));
        // At 24.9 ft sinking 300 and easing 140 fpm a second (EGBB, 2.1 s
        // before the float): the float ahead takes the thrust now.
        let mut q = FlarePower::default();
        q.step(cfg, 0.30, -440.0, 0.3, 26.7, app, app);
        q.step(cfg, 0.30, -398.0, 0.3, 25.8, app, app);
        let c = q.step(cfg, 0.30, -300.0, 0.3, 24.9, app, app).unwrap();
        assert_eq!(q.last_key, Some(RetardKey::Float));
        assert!(c < 0.30);
        // Still sinking 300 steadily over 20 ft: the height holds it.
        let mut h = FlarePower::default();
        h.step(cfg, 0.30, -300.0, 0.3, 26.0, app, app);
        assert_eq!(h.step(cfg, 0.30, -300.0, 0.3, 25.0, app, app), Some(0.30));
        assert_eq!(h.last_key, Some(RetardKey::Height));
        // A near-level float is not a balloon: no approach thrust back.
        assert_eq!(super::balloon_power(Some(1.0), Some(0.30), false, false), None);
        assert_eq!(super::balloon_power(Some(2.0), Some(0.30), false, false), None);
        assert_eq!(
            super::balloon_power(Some(60.0), Some(0.30), false, false),
            Some(0.30)
        );
    }

    #[derive(Clone, Copy, Debug)]
    struct Tick {
        t: f64,
        radio: f64,
        vvi: f64,
        trend: f64,
        asked: f64,
        ias: f64,
        elev: f64,
        lever: f64,
        arresting: bool,
        flare: bool,
        final_leg: bool,
        ap_hold: bool,
    }

    struct Recording {
        ticks: Vec<Tick>,
        touchdown_fpm: f64,
    }

    fn recording(sortie: &str) -> Recording {
        let mut ticks = Vec::new();
        let mut touchdown_fpm = f64::NAN;
        for line in LEMG_13_FLARES
            .lines()
            .chain(LEMG_13_CALM_FLARES.lines())
            .chain(EGBB_15_FLOAT.lines())
            .chain(LXGB_09_FLOAT.lines())
        {
            let v: serde_json::Value = serde_json::from_str(line).expect("a bench line");
            if !v["sortie"].as_str().is_some_and(|s| s.starts_with(sortie)) {
                continue;
            }
            if v["kind"] == "flight" {
                touchdown_fpm = -v["sink"].as_f64().unwrap();
                continue;
            }
            let f = |k: &str| v[k].as_f64().unwrap_or(0.0);
            ticks.push(Tick {
                t: f("t"),
                radio: f("radio"),
                vvi: f("vvi"),
                trend: f("vvi_trend"),
                asked: f("cmd_vs"),
                ias: f("ias"),
                elev: f("flare_elev"),
                lever: f("lever"),
                arresting: v["arresting"] == true,
                flare: v["phase"] == "Flare",
                final_leg: v["phase"] == "Final",
                ap_hold: v["ap_hold"] == true,
            });
        }
        assert!(!ticks.is_empty() && touchdown_fpm.is_finite(), "{sortie}");
        Recording {
            ticks,
            touchdown_fpm,
        }
    }

    impl Recording {
        /// The final's air as the loop measures it on the way to the gate.
        fn air(&self) -> super::FinalAir {
            let mut air = super::FinalAir::default();
            for k in &self.ticks {
                if k.final_leg && !k.arresting && !k.ap_hold && k.radio <= super::FINAL_AIR_FROM_FT
                {
                    air.note(k.vvi, k.asked);
                }
            }
            air
        }
        fn arrest(&self) -> Vec<Tick> {
            self.ticks.iter().copied().filter(|k| k.arresting).collect()
        }
    }

    /// Today's calm flares through their own fits: 3,700 to 4,450 fpm per
    /// unit, 1.35 to 1.8 s of dead time (ten landings, 9 to 23 fpm rms).
    /// The gusty ones fit nothing sensible (6,450 to 9,150 fpm, 49 to 96
    /// rms): the air is not in the plant.
    const CALM_FIT: Fit = Fit {
        fpm_per_elevator: 4000.0,
        dead_s: 1.55,
    };

    /// The recorded gate; `trim` is a fitted trimmed sink (fpm), else the
    /// gate's own sink (the trim folded into the air).
    fn gate_of(a: &[Tick], trim: Option<f64>) -> Gate {
        Gate {
            radio_ft: a[0].radio,
            vvi_fpm: a[0].vvi,
            trend_fpm_s: a[0].trend,
            ias_kts: a[0].ias,
            throttle: a[0].lever,
            // What the air does is taken from the recording; the trim is
            // folded into it.
            trimmed_sink_fpm: trim.unwrap_or(a[0].vvi),
            flown_fpm: f64::NAN,
        }
    }

    /// The air of a recording through a fit: the recorded elevator, speed
    /// and lever flown open loop, and per tick the sink (fpm, at the tick)
    /// and speed change (kt/s, over the interval after it) the fit does not
    /// explain.
    fn air_of(a: &[Tick], fit: Fit, trim: Option<f64>) -> (Vec<f64>, Vec<f64>) {
        let mut p = Plant::at(gate_of(a, trim), fit);
        let dt = 0.01;
        let (mut air, mut speed) = (vec![0.0], Vec::new());
        for w in a.windows(2) {
            let (t0, t1) = (w[0].t - a[0].t, w[1].t - a[0].t);
            p.write(w[0].elev);
            let (mut acc, mut n) = (0.0f64, 0.0f64);
            while p.t + 1e-9 < t1 {
                acc += speed_model(w[0].lever, p.elev, w[0].vvi);
                n += 1.0;
                let f = (p.t - t0) / (t1 - t0);
                p.step(
                    w[0].lever,
                    w[0].radio,
                    Some(w[0].ias + (w[1].ias - w[0].ias) * f),
                    dt,
                );
            }
            speed.push((w[1].ias - w[0].ias) / (t1 - t0) - acc / n.max(1.0));
            air.push(w[1].vvi - (p.trimmed + p.x2 + FPM_PER_KT * (w[1].ias - p.ias0)));
        }
        (air, speed)
    }

    struct Flown {
        sink_fpm: f64,
        /// Most the column came forward of its flare peak while not
        /// climbing.
        eased: f64,
        climbed: bool,
        float_m: f64,
        lever_at_wheels: f64,
        ias_at_wheels: f64,
        /// Seconds of the flare sinking under [`super::RETARD_FLOAT_FPM`].
        float_s: f64,
    }

    /// Fly a recorded gate through a fit and the recorded air, the law
    /// ticking at the recorded tick times (0.22 s past them), the flare from
    /// the loop's own gate, the profile's retard below it and the approach
    /// lever held in a climb, the elevator arriving a dead time after it is
    /// written through two lags.
    fn fly_through_air(a: &[Tick], fit: Fit, rough: bool, with_air: bool) -> Flown {
        fly_through_air_with(a, fit, None, rough, with_air, Retard::profile(), false)
    }

    /// ...with the retard as it flew on 2026-09-28 (the sink-keyed law) and
    /// the column and lever taking any climb for a balloon.
    fn fly_as_flown(a: &[Tick], fit: Fit, rough: bool, with_air: bool) -> Flown {
        fly_through_air_with(
            a,
            fit,
            None,
            rough,
            with_air,
            Retard::Sink(SinkKeyed::as_flown()),
            true,
        )
    }

    /// `any_climb`: the column eases and the lever takes the approach
    /// thrust back on any climb, +1 fpm included (as flown before issue
    /// #146); else only past [`super::BALLOON_CLIMB_FPM`].
    fn fly_through_air_with(
        a: &[Tick],
        fit: Fit,
        trim: Option<f64>,
        rough: bool,
        with_air: bool,
        power: Retard,
        any_climb: bool,
    ) -> Flown {
        let climb_fpm = if any_climb {
            0.0
        } else {
            super::BALLOON_CLIMB_FPM
        };
        let (air, speed) = if with_air {
            air_of(a, fit, trim)
        } else {
            (vec![0.0], Vec::new())
        };
        let law = if any_climb {
            model_as_flown().easing_on_any_climb()
        } else {
            model()
        };
        fly_through_air_law(
            a,
            fit,
            trim,
            rough,
            with_air,
            power,
            climb_fpm,
            law,
            (air, speed),
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn fly_through_air_law(
        a: &[Tick],
        fit: Fit,
        trim: Option<f64>,
        rough: bool,
        with_air: bool,
        mut power: Retard,
        climb_fpm: f64,
        mut law: FlareModel,
        (air, speed): (Vec<f64>, Vec<f64>),
    ) -> Flown {
        let times: Vec<f64> = a.iter().map(|k| k.t - a[0].t).collect();
        let mut p = Plant::at(gate_of(a, trim), fit);
        let (mut h, approach_lever) = (a[0].radio, a[0].lever);
        let mut lever = approach_lever;
        let (dt, mut i, mut next) = (0.01, 0usize, 0.0);
        let mut prev_t: Option<f64> = None;
        let (mut peak, mut flaring) = (0.0f64, false);
        let mut out = Flown {
            sink_fpm: 0.0,
            eased: 0.0,
            climbed: false,
            float_m: 0.0,
            lever_at_wheels: 0.0,
            ias_at_wheels: 0.0,
            float_s: 0.0,
        };
        while h > 0.0 && p.t < 40.0 {
            if p.t + 1e-9 >= next {
                let tick = times.get(i + 1).map_or(0.22, |t1| t1 - times[i]);
                let law_dt = prev_t.map_or(0.22, |t0| p.t - t0);
                prev_t = Some(p.t);
                let v = p.vvi;
                let gs = p.ias - 4.0;
                let (flare_ft, _) = super::arrest_gates_ft(25.0, gs);
                let flare = h <= flare_ft;
                let e = law.step(h, gs, v, law_dt, flare, rough).elev;
                p.write(e);
                if flare {
                    if !flaring {
                        peak = e;
                    }
                    flaring = true;
                    peak = peak.max(e);
                    if v <= climb_fpm {
                        out.eased = out.eased.max(peak - e);
                    }
                    lever = if v > climb_fpm {
                        approach_lever
                    } else {
                        power.step(lever, v, law_dt, h, p.ias).unwrap_or(lever)
                    };
                }
                p.air_kts_s = if with_air {
                    speed.get(i).copied().unwrap_or(0.0)
                } else {
                    0.0
                };
                i += 1;
                next += tick;
            }
            if with_air {
                let j = i - 1;
                p.air_fpm = match (air.get(j), air.get(j + 1), times.get(j + 1)) {
                    (Some(a0), Some(a1), Some(t1)) => {
                        let f = ((p.t - times[j]) / (t1 - times[j])).clamp(0.0, 1.0);
                        a0 + (a1 - a0) * f
                    }
                    _ => *air.last().unwrap(),
                };
            }
            p.step(lever, h, None, dt);
            h += p.vvi / 60.0 * dt;
            if flaring {
                out.float_m += (p.ias - 4.0) * 0.514_444 * dt;
                out.climbed |= p.vvi > 0.0;
                if p.vvi > -super::RETARD_FLOAT_FPM {
                    out.float_s += dt;
                }
            }
        }
        out.sink_fpm = -p.vvi;
        out.lever_at_wheels = lever;
        out.ias_at_wheels = p.ias;
        out
    }

    /// Every autoland tick's bug (true) from four LEMG 13 approaches of
    /// 2026-09-28, the arm to the wheels (two 5 nm finals, one with an
    /// intercept, and a full-auto circuit's final).
    const LEMG_13_FINAL_KNOBS: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/landings/lemg13_final_knobs.jsonl"));

    /// Issue #93: those approaches' bugs replayed through the knob autoland
    /// now sets (`bug_knob`): the knob moved (0.2 degrees or more, the least
    /// `drive_heading_bug_step` writes) at most half as often outside the
    /// short final and the flare (which still steer every tick), and always
    /// within a turn's steering of the law's own bug: 21 moves against 76.
    #[test]
    fn the_recorded_finals_move_the_knob_less() {
        let moved = |a: Option<f64>, b: f64| {
            a.is_none_or(|a| crate::angles::abs_diff_deg(b, a) >= 0.2)
        };
        let mut by_sortie: Vec<(String, Vec<serde_json::Value>)> = Vec::new();
        for line in LEMG_13_FINAL_KNOBS.lines() {
            let v: serde_json::Value = serde_json::from_str(line).expect("a bench line");
            let s = v["sortie"].as_str().unwrap().to_string();
            match by_sortie.last_mut() {
                Some((id, ticks)) if *id == s => ticks.push(v),
                _ => by_sortie.push((s, vec![v])),
            }
        }
        assert_eq!(by_sortie.len(), 4);
        let (mut free_all, mut held_all) = (0u32, 0u32);
        for (s, ticks) in &by_sortie {
            let mut hold = crate::heading_bug::BugHold::default();
            let (mut free, mut held) = (0u32, 0u32);
            let (mut free_at, mut held_at) = (None::<f64>, None::<f64>);
            let t0 = ticks[0]["t"].as_f64().unwrap();
            for k in ticks {
                let b = k["bug_true"].as_f64().unwrap();
                let phase = match k["phase"].as_str() {
                    Some("Flare") => super::ApproachPhase::Flare,
                    Some("Final") => super::ApproachPhase::Final,
                    _ => super::ApproachPhase::Intercept,
                };
                let short = k["short_final"] == true || k["arresting"] == true;
                let leg = match k["leg"].as_str() {
                    Some("outbound") => Some(crate::reversal::Leg::Outbound),
                    Some("base") => Some(crate::reversal::Leg::Base),
                    Some("inbound") => Some(crate::reversal::Leg::Inbound),
                    _ => None,
                };
                let knob = match super::bug_knob(phase, short, leg, None) {
                    Some((key, db)) => {
                        hold.decide(key, b, k["t"].as_f64().unwrap() - t0, db);
                        let h = hold.set().unwrap_or(b);
                        let off = crate::angles::abs_diff_deg(h, b);
                        assert!(
                            off < crate::heading_bug::BUG_STEER_DEG,
                            "{s}: the knob {h:.1} against the law's {b:.1}"
                        );
                        h
                    }
                    None => {
                        hold.reset();
                        free_at = None;
                        held_at = None;
                        continue;
                    }
                };
                if moved(free_at, b) {
                    free += 1;
                    free_at = Some(b);
                }
                if moved(held_at, knob) {
                    held += 1;
                    held_at = Some(knob);
                }
            }
            assert!(held * 2 <= free, "{s}: {held} moves against {free}");
            free_all += free;
            held_all += held;
        }
        assert!(held_all * 3 <= free_all, "{held_all} moves against {free_all}");
    }

    /// The loop's measure of the final's air tells the three apart: the
    /// gusty finals strayed 72 and 105 fpm rms from the sink asked for, the
    /// calm one 19.
    #[test]
    fn the_final_says_whether_the_air_is_rough() {
        for (sortie, rough) in [
            ("1790589309", true),
            ("1790589773", true),
            ("1790588609", false),
        ] {
            let air = recording(sortie).air();
            let rms = air.rms_fpm().expect("a whole final measured");
            assert_eq!(air.rough(), rough, "{sortie}: {rms:.0} fpm rms");
        }
        assert!(recording("1790588609").air().rms_fpm().unwrap() < 25.0);
        assert!(recording("1790589309").air().rms_fpm().unwrap() > 60.0);
        // Too little of a final is no measurement, and no measurement is
        // smooth air.
        let mut short = super::FinalAir::default();
        for _ in 0..5 {
            short.note(-700.0, -800.0);
            short.note(-900.0, -800.0);
        }
        assert!(short.rms_fpm().is_none() && !short.rough());
        short.note(f64::NAN, -800.0);
        assert!(short.rms_fpm().is_none(), "a failed read is no sample");
    }

    /// The harness is the law that flew: the law as flown that night (its
    /// 6 s exponential), fed the recorded sinks, writes the elevator that
    /// was written (the float hold changes nothing there, the model was
    /// already pulling), and the fit flown open loop through the recorded
    /// air gives back the recorded sink.
    #[test]
    fn the_recorded_flares_replay_through_the_law_that_flew() {
        for sortie in ["1790589309", "1790589773", "1790588609"] {
            let a = recording(sortie).arrest();
            let mut m = model_as_flown();
            let mut worst: f64 = 0.0;
            for (n, k) in a.iter().enumerate() {
                let dt = if n == 0 { 0.22 } else { k.t - a[n - 1].t };
                let e = m.step(k.radio, k.ias - 4.0, k.vvi, dt, k.flare, false).elev;
                worst = worst.max((e - k.elev).abs());
            }
            assert!(worst < 0.07, "{sortie}: elevator {worst:.3} off the record");
        }
    }

    /// LEMG 13, 1790589309 (issue #146): through today's calm fit and the
    /// air the recording carries, the flare as flown (smooth, the prediction
    /// free) eases the column a fifth of its travel from 25 ft while still
    /// sinking, floats, and arrives near the 422 the aircraft did. Flown for
    /// the rough air the final measured, the column holds from the flare
    /// gate and the same air lands under 300 without a climb.
    #[test]
    fn a_gusty_flare_holds_the_column_through_the_float() {
        let r = recording("1790589309");
        assert_eq!(r.touchdown_fpm, 422.0);
        assert!(r.air().rough());
        let a = r.arrest();
        let flown = fly_as_flown(&a, CALM_FIT, false, true);
        assert!(
            (flown.sink_fpm - r.touchdown_fpm).abs() < 40.0,
            "the replay lands where the aircraft did: {:.0}",
            flown.sink_fpm
        );
        assert!(flown.eased > 0.2, "and eases as it did: {:.2}", flown.eased);
        let held = fly_through_air(&a, CALM_FIT, true, true);
        assert!(held.sink_fpm < 300.0, "{:.0}", held.sink_fpm);
        assert!(
            held.sink_fpm > 150.0,
            "a landing, not a float: {:.0}",
            held.sink_fpm
        );
        assert!(held.eased < 1e-9, "never forward: {:.3}", held.eased);
        assert!(!held.climbed, "no balloon");
        assert!(held.float_m < 1200.0, "floated {:.0} m", held.float_m);
        // 1790589773 (335 fpm, the same gusts): 380 as flown here, under
        // 320 held.
        let a = recording("1790589773").arrest();
        let flown = fly_as_flown(&a, CALM_FIT, false, true);
        let held = fly_through_air(&a, CALM_FIT, true, true);
        assert!(flown.sink_fpm > 350.0, "{:.0}", flown.sink_fpm);
        assert!(
            held.sink_fpm < 320.0 && !held.climbed,
            "{:.0}",
            held.sink_fpm
        );
        // Not one plant's luck: through elevators from today's weakest to
        // past the 2026-09-26 middle, and dead times 1.35 to 1.8 s, both
        // gusty flares held land under 340 and never climb.
        for sortie in ["1790589309", "1790589773"] {
            let a = recording(sortie).arrest();
            for k in [3700.0, 4400.0, 5100.0] {
                for d in [1.35, 1.55, 1.8] {
                    let fit = Fit {
                        fpm_per_elevator: k,
                        dead_s: d,
                    };
                    let held = fly_through_air(&a, fit, true, true);
                    assert!(
                        held.sink_fpm < 340.0 && !held.climbed,
                        "{sortie} {k} {d}: {:.0}",
                        held.sink_fpm
                    );
                }
            }
        }
    }

    /// The calm final keeps the prediction: 1790588609 through its air is
    /// smooth, and flown smooth lands where it did (236).
    #[test]
    fn a_calm_flare_is_flown_as_before() {
        let r = recording("1790588609");
        assert!(!r.air().rough());
        let a = r.arrest();
        let flown = fly_as_flown(&a, CALM_FIT, false, true);
        assert!(
            (flown.sink_fpm - r.touchdown_fpm).abs() < 30.0,
            "{:.0} against {:.0}",
            flown.sink_fpm,
            r.touchdown_fpm
        );
        assert!(flown.sink_fpm > 200.0 && flown.sink_fpm < 290.0);
    }

    /// Why smooth air keeps the prediction: the 17:25 flare of 2026-09-26
    /// answered 6,429 fpm per unit, and in still air the column held from
    /// the flare gate balloons it and drops it in, where the prediction
    /// eases and lands. The gust and a strong elevator look alike for the
    /// seconds that matter; only the air tells them apart. The evidence is
    /// the 6 s exponential's, the law that set the rule (the 5.5 s one held
    /// in still air lands that plant under 300, but the prediction is still
    /// what smooth air flies).
    #[test]
    fn a_strong_elevator_in_still_air_keeps_the_prediction() {
        let strong = PLANTS[3];
        let mut worst_held: f64 = 0.0;
        let mut climbed = false;
        for g in GATES.iter() {
            let free = fly(*g, strong, Law::Model(model()));
            assert!(
                free.sink_fpm < 340.0 && !free.climbed,
                "{:.0}",
                free.sink_fpm
            );
            let held = fly(*g, strong, Law::Rough(model_as_flown()));
            worst_held = worst_held.max(held.sink_fpm);
            climbed |= held.climbed;
        }
        assert!(
            climbed && worst_held > 400.0,
            "held in still air it balloons: {worst_held:.0}"
        );
    }

    /// A near-level float never takes the column forward, smooth air or
    /// rough; a climb eases it at the airframe's rate, never a push; the
    /// roundout above the flare stays free.
    #[test]
    fn a_float_keeps_its_pull_and_only_a_climb_eases_it() {
        let mut m = model();
        // Into the flare sinking hard: the prediction pulls.
        let mut pulled: f64 = 0.0;
        for n in 0..6 {
            let h = 30.0 - n as f64 * 1.5;
            pulled = m.step(h, 140.0, -420.0, 0.22, true, false).elev;
        }
        assert!(pulled > 0.1, "{pulled}");
        // Floating at -70 with that pull in the pipe: the prediction wants
        // it forward, the column stays.
        let mut e = pulled;
        for _ in 0..8 {
            let o = m.step(16.0, 140.0, -70.0, 0.22, true, false);
            assert!(o.elev >= e - 1e-9, "forward in a float: {e} -> {}", o.elev);
            assert!(o.ask <= o.elev + 1e-9);
            e = o.elev;
        }
        // Climbing: eased at the profile's 0.15 a second, never below zero.
        let o = m.step(16.5, 140.0, 60.0, 0.22, true, false);
        assert!(
            o.elev < e && o.elev >= e - 0.15 * 0.22 - 1e-9,
            "{e} -> {}",
            o.elev
        );
        assert!(o.elev >= 0.0);
        // Above the flare the model is free both ways again.
        let mut free = model();
        let hi = free.step(60.0, 140.0, -900.0, 0.22, false, true).elev;
        let lo = free.step(58.0, 140.0, -300.0, 0.22, false, true).elev;
        assert!(lo < hi, "the roundout eases: {hi} -> {lo}");
    }

    // ---- LXGB 09, 2026-10-06: the float to 631 m and the balk at 4.5 ft ----

    /// LXGB 09 (737, wet, 09011KT -RA, sortie 1791275171-full_auto): the
    /// autoland ticks from 400 ft to the balk at 4.5 ft and the flight
    /// model's touchdown (143 fpm, 136 kt over the ground). Rough air (82
    /// fpm rms): the column held 0.30 from 20 ft while the prediction's ask
    /// fell to zero; the lever held 0.33 by height then time to 10.6 ft,
    /// then the float key walked it to 0.25 at 0.04 a second.
    const LXGB_09_FLOAT: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/bench/landings/lxgb09_737_float.jsonl"
    ));

    /// LXGB's own flare, fitted open loop to the recorded elevator, lever
    /// and speed from the arrest gate to 4.5 ft (22 fpm rms): 5,500 fpm per
    /// unit through 1.35 s of dead time, on a trim that held -850.
    const LXGB_FIT: Fit = Fit {
        fpm_per_elevator: 5500.0,
        dead_s: 1.35,
    };
    const LXGB_TRIM_FPM: f64 = -850.0;
    /// 141.5 kt indicated was 136 over the ground at the wheels.
    const LXGB_HEAD_KTS: f64 = 5.5;
    /// The arrest gate was 0.0745 nm short of the strip's end, and the
    /// threshold is displaced 136 m: metres from the gate to the threshold.
    const LXGB_GATE_TO_THRESHOLD_M: f64 = 0.0745 * 1852.0 + 136.0;
    /// The touchdown zone of the 1,627 m landing distance available: its
    /// first third (FAA: the first 3,000 ft or a third, the less).
    const LXGB_TDZ_M: f64 = 1627.0 / 3.0;

    fn lxgb_fit_rms(a: &[Tick], fit: Fit, trim: Option<f64>) -> f64 {
        let mut p = Plant::at(gate_of(a, trim), fit);
        let mut sq = 0.0;
        for w in a.windows(2) {
            let (t0, t1) = (w[0].t - a[0].t, w[1].t - a[0].t);
            p.write(w[0].elev);
            while p.t + 1e-9 < t1 {
                let f = (p.t - t0) / (t1 - t0);
                p.step(
                    w[0].lever,
                    w[0].radio,
                    Some(w[0].ias + (w[1].ias - w[0].ias) * f),
                    0.01,
                );
            }
            sq += (p.vvi - w[1].vvi).powi(2);
        }
        (sq / (a.len() - 1) as f64).sqrt()
    }

    /// The retard as flown at LXGB: the profile's keys, the float walked at
    /// the timed retard's 0.04 a second.
    fn lxgb_retard_as_flown() -> Retard {
        let cfg = profile_737().flare_power.expect("the 737 retards");
        Retard::Keyed(
            FlarePower::default(),
            crate::aircraft::FlarePowerProfile {
                retard_float_per_s: None,
                ..cfg
            },
        )
    }

    struct Run {
        sink_fpm: f64,
        /// Metres past the displaced threshold at the wheels.
        past_m: f64,
        climbed: bool,
        lever_at_wheels: f64,
        /// Seconds from the gate and metres past the threshold at the
        /// balk's 4.5 ft.
        at_balk: Option<(f64, f64)>,
    }

    /// Fly LXGB's gate through a fit and the recorded air with `law` and
    /// `power`, the loop's own flare gate, the lever held on a climb past
    /// [`super::BALLOON_CLIMB_FPM`], the distance flown over the ground.
    fn fly_lxgb(fit: Fit, rough: bool, mut law: FlareModel, mut power: Retard) -> Run {
        let a = recording("1791275171").arrest();
        let trim = Some(LXGB_TRIM_FPM);
        let (air, speed) = air_of(&a, fit, trim);
        let times: Vec<f64> = a.iter().map(|k| k.t - a[0].t).collect();
        let mut p = Plant::at(gate_of(&a, trim), fit);
        let (mut h, approach_lever) = (a[0].radio, a[0].lever);
        let mut lever = approach_lever;
        let (dt, mut i, mut next) = (0.01, 0usize, 0.0);
        let mut prev_t: Option<f64> = None;
        let mut run_m = -LXGB_GATE_TO_THRESHOLD_M;
        let mut out = Run {
            sink_fpm: 0.0,
            past_m: 0.0,
            climbed: false,
            lever_at_wheels: 0.0,
            at_balk: None,
        };
        while h > 0.0 && p.t < 40.0 {
            if p.t + 1e-9 >= next {
                let tick = times.get(i + 1).map_or(0.22, |t1| t1 - times[i]);
                let law_dt = prev_t.map_or(0.22, |t0| p.t - t0);
                prev_t = Some(p.t);
                let v = p.vvi;
                let gs = p.ias - LXGB_HEAD_KTS;
                let (flare_ft, _) = super::arrest_gates_ft(25.0, gs);
                let flare = h <= flare_ft;
                p.write(law.step(h, gs, v, law_dt, flare, rough).elev);
                if flare {
                    lever = if v > super::BALLOON_CLIMB_FPM {
                        approach_lever
                    } else {
                        power.step(lever, v, law_dt, h, p.ias).unwrap_or(lever)
                    };
                }
                p.air_kts_s = speed.get(i).copied().unwrap_or(0.0);
                i += 1;
                next += tick;
            }
            let j = i - 1;
            p.air_fpm = match (air.get(j), air.get(j + 1), times.get(j + 1)) {
                (Some(a0), Some(a1), Some(t1)) => {
                    let f = ((p.t - times[j]) / (t1 - times[j])).clamp(0.0, 1.0);
                    a0 + (a1 - a0) * f
                }
                _ => *air.last().unwrap(),
            };
            p.step(lever, h, None, dt);
            h += p.vvi / 60.0 * dt;
            run_m += (p.ias - LXGB_HEAD_KTS) * 0.514_444 * dt;
            out.climbed |= p.vvi > 0.0;
            if h < 4.54 && out.at_balk.is_none() {
                out.at_balk = Some((p.t, run_m));
            }
        }
        out.sink_fpm = -p.vvi;
        out.past_m = run_m;
        out.lever_at_wheels = lever;
        out
    }

    /// The plant is the aircraft and the harness the law that flew: the fit
    /// gives back the recorded sink open loop, the law as flown (the 6 s
    /// exponential, held in the rough air the final measured) fed the
    /// recorded sinks writes the recorded elevator, and flown closed loop
    /// it reaches the balk's 4.5 ft floating, as the aircraft did (-136
    /// fpm against -44, 10 s and 430 m in against 11.2 s and 520).
    #[test]
    fn the_lxgb_plant_is_fitted_to_its_own_flare() {
        let r = recording("1791275171");
        assert!(r.air().rough(), "{:?}", r.air().rms_fpm());
        assert_eq!(r.touchdown_fpm, 143.0);
        let a = r.arrest();
        let rms = lxgb_fit_rms(&a, LXGB_FIT, Some(LXGB_TRIM_FPM));
        assert!(rms < 25.0, "the fit follows the record: {rms:.0} fpm rms");
        let mut m = model_as_flown();
        let mut worst: f64 = 0.0;
        for (n, k) in a.iter().enumerate() {
            let dt = if n == 0 { 0.22 } else { k.t - a[n - 1].t };
            let e = m
                .step(k.radio, k.ias - LXGB_HEAD_KTS, k.vvi, dt, k.flare, true)
                .elev;
            worst = worst.max((e - k.elev).abs());
        }
        assert!(worst < 0.05, "elevator {worst:.3} off the record");
        let flown = fly_lxgb(LXGB_FIT, true, model_as_flown(), lxgb_retard_as_flown());
        let (t, past) = flown.at_balk.expect("down to the balk's height");
        assert!(t > 9.0 && t < 12.0, "{t:.1} s");
        assert!(past > 380.0 && past < 560.0, "{past:.0} m");
    }

    /// The float, issue: touchdown 604 m past the displaced threshold by the
    /// pilot's landing-rate tool (631 by the blackbox's `remaining_m`), the
    /// touchdown zone of a 1,627 m runway is its first 542. Through the
    /// flare's own fit and the air it carried, the law as flown floats past
    /// 700 m (the balk's spool-up put the real wheels down sooner). A 5.5 s
    /// exponential (the 6 s one asks 150 fpm plus 10 a foot: from 20 ft it
    /// is 5 s to the wheels) and the float's thrust off at 0.08 a second
    /// land inside the zone, under 300 fpm, the lever coming off, no climb.
    /// Around the fit (elevator 4,500 to 5,500, dead time 1.2 to 1.6 s) it
    /// is shorter every time, never firmer than 300.
    #[test]
    fn the_lxgb_float_lands_in_the_touchdown_zone() {
        let flown = fly_lxgb(LXGB_FIT, true, model_as_flown(), lxgb_retard_as_flown());
        assert!(flown.past_m > 700.0, "as flown: {:.0} m", flown.past_m);
        let f = fly_lxgb(LXGB_FIT, true, model(), Retard::profile());
        assert!(
            f.past_m < LXGB_TDZ_M,
            "in the zone: {:.0} m against {LXGB_TDZ_M:.0}",
            f.past_m
        );
        assert!(
            f.past_m < flown.past_m - 250.0,
            "{:.0} against {:.0}",
            f.past_m,
            flown.past_m
        );
        assert!(
            f.sink_fpm > 90.0 && f.sink_fpm < 300.0,
            "a landing, not firm: {:.0}",
            f.sink_fpm
        );
        assert!(!f.climbed, "no balloon");
        assert!(
            f.lever_at_wheels < flown.lever_at_wheels,
            "{:.2}",
            f.lever_at_wheels
        );
        // In smooth air too.
        let s = fly_lxgb(LXGB_FIT, false, model(), Retard::profile());
        assert!(
            s.past_m < LXGB_TDZ_M && s.sink_fpm < 300.0,
            "{:.0} m {:.0} fpm",
            s.past_m,
            s.sink_fpm
        );
        for k in [4500.0, 5500.0] {
            for d in [1.2, 1.35, 1.6] {
                let fit = Fit {
                    fpm_per_elevator: k,
                    dead_s: d,
                };
                let old = fly_lxgb(fit, true, model_as_flown(), lxgb_retard_as_flown());
                let new = fly_lxgb(fit, true, model(), Retard::profile());
                assert!(
                    new.past_m < old.past_m - 20.0,
                    "{k} {d}: {:.0} against {:.0}",
                    new.past_m,
                    old.past_m
                );
                assert!(
                    new.sink_fpm < 300.0 && !new.climbed,
                    "{k} {d}: {:.0}",
                    new.sink_fpm
                );
            }
        }
    }
