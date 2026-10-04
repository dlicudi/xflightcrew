    use super::*;

    /// The recorded arrest (s from `arrest_hands` taken, radio ft, vvi
    /// fpm, the elevator written), to the tick before the servos went back.
    const REC: [(f64, f64, f64, f64); 8] = [
        (0.0, 51.15, -529.3, 0.11),
        (0.4, 46.82, -525.2, 0.16),
        (0.8, 43.87, -512.1, 0.19),
        (1.2, 41.72, -454.1, 0.17),
        (1.5, 41.55, -323.6, 0.09),
        (1.9, 43.29, -125.7, -0.03),
        (2.3, 46.07, 62.7, -0.05),
        (2.7, 48.94, 214.5, -0.05),
    ];
    /// The trend and airspeed at the gate, and when the servos went back.
    const GATE_TREND: f64 = 8.5;
    const GATE_IAS: f64 = 86.4;
    const HANDED_BACK_S: f64 = 3.1;
    const HANDED_BACK_FT: f64 = 50.48;
    const TICK_S: f64 = 0.35;
    const VSO_KTS: f64 = 67.0;

    /// (fpm per unit, lag 1, lag 2, dead time): ETND 26, EDLP 24, LEMG 12,
    /// and all three together (the profile's).
    const PLANTS: [(f64, f64, f64, f64); 4] = [
        (7000.0, 1.2, 1.2, 1.0),
        (9000.0, 1.5, 1.2, 0.8),
        (10000.0, 1.5, 0.9, 0.7),
        (8000.0, 1.2, 1.2, 0.9),
    ];
    const LEMG_FIT: usize = 2;
    const PROFILE_FIT: usize = 3;
    /// The other recorded gates (radio ft, vvi fpm, trend fpm/s, IAS kt):
    /// ETND 26, EDLP 24's balked balloon, EDLP 24's landing.
    const OTHER_GATES: [(f64, f64, f64, f64); 3] = [
        (49.0, -456.0, -13.0, 88.0),
        (49.0, -371.0, -2.0, 88.0),
        (49.0, -571.0, 43.0, 87.0),
    ];

    struct Plant {
        fit: (f64, f64, f64, f64),
        h: f64,
        v: f64,
        ias: f64,
        base: f64,
        b1: f64,
        b2: f64,
        stall: f64,
        gust: f64,
        rng: u64,
        rough_fpm: f64,
        t: f64,
        written: Vec<(f64, f64)>,
    }

    impl Plant {
        /// At a gate: sinking `v` with `trend`, the column that made the
        /// trend already in the pipe.
        fn at(fit: (f64, f64, f64, f64), h: f64, v: f64, trend: f64, ias: f64) -> Self {
            let b1 = trend * fit.2;
            Plant {
                fit,
                h,
                v,
                ias,
                base: v,
                b1,
                b2: 0.0,
                stall: 0.0,
                gust: 0.0,
                rng: 0,
                rough_fpm: 0.0,
                t: 0.0,
                written: vec![(-10.0, b1 / fit.0)],
            }
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
            let (k, l1, l2, dead) = self.fit;
            let due = self.t - dead;
            let arriving = self
                .written
                .iter()
                .take_while(|(t, _)| *t <= due + 1e-9)
                .last()
                .map_or(0.0, |w| w.1);
            self.b1 += (k * arriving - self.b1) * dt / l1;
            self.b2 += (self.b1 - self.b2) * dt / l2;
            if self.rough_fpm > 0.0 {
                const TC: f64 = 1.5;
                let n = self.gauss();
                self.gust += -self.gust / TC * dt + self.rough_fpm * (2.0 * dt / TC).sqrt() * n;
            }
            if self.ias < VSO_KTS {
                self.stall += 800.0 * dt;
            }
            self.v = self.base + self.b2 + self.gust - self.stall;
            self.ias += (-1.71 - 1.29 * self.v / 1000.0) * dt;
            self.h += self.v / 60.0 * dt;
            self.t += dt;
        }
        /// The recorded column, written as it was, to `until`.
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
                self.step(0.01);
            }
        }
    }

    /// Which elevator law flies.
    enum Law {
        Model(FlareModel),
        Tracker(FlarePitch, Option<f64>),
    }

    impl Law {
        fn sf50_elevator() -> crate::aircraft::FlareElevatorProfile {
            crate::aircraft::load("cirrus_vision_sf50")
                .and_then(|p| p.flare_elevator)
                .expect("the SF50 declares its flare")
        }
        /// Today's: the profile's model.
        fn today() -> Self {
            Law::Model(
                FlareModel::from_profile(Some(Self::sf50_elevator()))
                    .expect("the SF50 declares its model"),
            )
        }
        /// The tracker the SF50 flew at LEMG 12 (the profile's gains, no
        /// model), still with the servos latched.
        fn as_flown() -> Self {
            Law::Tracker(
                FlarePitch::with_gains(FlareGains::from_profile(Some(Self::sf50_elevator()))),
                None,
            )
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

    struct Landed {
        wheels_fpm: f64,
        pushed_climbing: bool,
        secs: f64,
        min_ias: f64,
    }

    /// The arrest from here to the wheels at the loop's tick, the servos
    /// kept (the latch): the flare below its gate and a flare stays a flare
    /// to the band's top (`hold_flare`), the roundout above.
    fn land(p: &mut Plant, law: &mut Law, rough: bool) -> Landed {
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
                let gs = p.ias + 3.0;
                let (flare_ft, band_ft) = arrest_gates_ft(25.0, gs);
                flaring = p.h <= flare_ft || (flaring && p.h <= band_ft);
                let e = law.step(p.h, gs, p.v, TICK_S, flaring, rough);
                out.pushed_climbing |= p.v > BALLOON_CLIMB_FPM && e < -1e-9;
                p.write(e);
            }
            p.step(0.01);
            out.min_ias = out.min_ias.min(p.ias);
        }
        out.wheels_fpm = p.v;
        out.secs = p.t - start;
        out
    }

    fn at_the_gate(fit: (f64, f64, f64, f64)) -> Plant {
        Plant::at(fit, REC[0].1, REC[0].2, GATE_TREND, GATE_IAS)
    }

    /// The plant is the recording: the column as written gives back the
    /// sink as measured, the balloon included, under 12 fpm rms on its own
    /// fit and 130 on the profile's (the other flares' own plants are other
    /// flares: 200 and more here).
    #[test]
    fn the_sf50_plant_replays_the_lemg_roundout() {
        for (i, fit) in PLANTS.iter().enumerate() {
            let mut p = at_the_gate(*fit);
            let mut sq = 0.0;
            for r in REC.iter().skip(1) {
                p.recorded(r.0);
                sq += (p.v - r.2).powi(2);
            }
            let rms = (sq / (REC.len() - 1) as f64).sqrt();
            match i {
                LEMG_FIT => assert!(rms < 12.0, "its own fit: {rms:.1} fpm rms"),
                PROFILE_FIT => assert!(rms < 140.0, "the profile's: {rms:.1} fpm rms"),
                _ => {}
            }
        }
    }

    /// The tick that gave the servos back: Final at 50.48 ft, 0.48 over a
    /// band read from the radio height alone. With the servos taken it is
    /// still the arrest, down at the roundout's sink; the arrest keeps them.
    #[test]
    fn a_balloon_over_the_band_keeps_the_servos() {
        let (_, band) = arrest_gates_ft(25.0, 85.0);
        assert!(HANDED_BACK_FT > band, "{band}");
        let tick = Guidance {
            target_heading_deg: 117.0,
            target_vs_fpm: -444.6,
            target_ias_kts: 87.0,
            dist_nm: 0.05,
            glidepath_alt_ft: 30.0,
            phase: ApproachPhase::Final,
            balk: None,
            arresting: false,
        };
        // The profile's recited landing distance, 1,721 ft.
        let sf50_m = Some(1721.0 * M_PER_FT);
        let held = hold_roundout(tick, true, HANDED_BACK_FT, 85.0, 2700.0, sf50_m);
        assert!(held.arresting, "still the arrest");
        assert_eq!((held.phase, held.balk), (ApproachPhase::Final, None));
        let (roundout, _) = arrest_targets_fpm(-(85.0 / 60.0) * gp_ft_per_nm());
        assert_eq!(held.target_vs_fpm, roundout);
        assert_eq!(arrest_hands_step(true, held.arresting), ArrestHands::Keep);
        // As flown, the same tick handed them back.
        assert_eq!(arrest_hands_step(true, tick.arresting), ArrestHands::Return);
        // Above the commit height, or with no room to stop, it is a
        // balked landing: the go-around brings the autopilot back.
        let high = hold_roundout(tick, true, BALLOON_COMMIT_FT + 1.0, 85.0, 2700.0, sf50_m);
        assert_eq!(
            (high.phase, high.balk, high.arresting),
            (ApproachPhase::Intercept, Some("balloon"), false)
        );
        assert_eq!(arrest_hands_step(true, high.arresting), ArrestHands::Return);
        let short = hold_roundout(tick, true, HANDED_BACK_FT, 85.0, 100.0, sf50_m);
        assert_eq!(short.balk, Some("balloon"));
        // Servos never taken, a real balk, an arrest already: untouched.
        assert_eq!(
            hold_roundout(tick, false, HANDED_BACK_FT, 85.0, 2700.0, sf50_m).arresting,
            false
        );
        let balk = Guidance {
            phase: ApproachPhase::Intercept,
            balk: Some("off_centreline_low"),
            ..tick
        };
        assert_eq!(
            hold_roundout(balk, true, HANDED_BACK_FT, 85.0, 2700.0, sf50_m).balk,
            balk.balk
        );
        let arresting = Guidance {
            arresting: true,
            target_vs_fpm: -100.0,
            ..tick
        };
        assert_eq!(
            hold_roundout(arresting, true, 40.0, 85.0, 2700.0, sf50_m).target_vs_fpm,
            -100.0
        );
    }

    /// A balloon never pushes: the tracker's roundout wrote -0.05 at +62
    /// and +214 fpm; the model's roundout, and both flares, never below
    /// zero while climbing.
    #[test]
    fn a_roundout_balloon_never_pushes() {
        let gains = FlareGains::from_profile(Some(Law::sf50_elevator()));
        let mut law = FlarePitch::with_gains(gains);
        for &(_, _, v, _) in REC.iter().take(5) {
            law.step(-283.0, v, 0.0, TICK_S, false);
        }
        // +214 fpm, easing at 396 fpm/s against the -268 target: the
        // recorded -0.05, and now nothing.
        assert!(law.step(-268.0, 214.5, 396.0, TICK_S, false) >= 0.0);
        // Sinking, the roundout is still free both ways.
        let mut law = FlarePitch::with_gains(gains);
        assert!(law.step(-283.0, -125.7, 495.0, TICK_S, false) < 0.0);
        let mut m = FlareModel::from_profile(Some(Law::sf50_elevator())).expect("model");
        for &(_, h, v, _) in REC.iter() {
            let e = m.step(h, 86.0, v, TICK_S, false, true).elev;
            if v > BALLOON_CLIMB_FPM {
                assert!(e >= 0.0, "{v} fpm: {e}");
            }
        }
    }

    /// From the gate, on every flare's own plant and every recorded SF50
    /// gate, the model lands under 300 fpm and never pushes a climb; the
    /// tracker the SF50 flew, the servos latched and never pushing, still
    /// porpoises through the dead time to harder than 400 somewhere.
    #[test]
    fn the_sf50_flare_lands_every_gate_on_every_plant() {
        let gates = std::iter::once((REC[0].1, REC[0].2, GATE_TREND, GATE_IAS)).chain(OTHER_GATES);
        let mut tracker_worst: f64 = 0.0;
        for (h, v, trend, ias) in gates {
            for fit in PLANTS {
                let mut p = Plant::at(fit, h, v, trend, ias);
                let l = land(&mut p, &mut Law::today(), false);
                assert!(
                    l.wheels_fpm > -300.0,
                    "{fit:?} from {h} ft {v} fpm: {:.0}",
                    l.wheels_fpm
                );
                assert!(!l.pushed_climbing, "{fit:?} from {h} ft");
                assert!(l.min_ias > VSO_KTS + 5.0, "{fit:?}: {:.1} kt", l.min_ias);
                assert!(l.secs < 12.0, "{fit:?}: floated {:.1} s", l.secs);
                let mut p = Plant::at(fit, h, v, trend, ias);
                tracker_worst =
                    tracker_worst.min(land(&mut p, &mut Law::as_flown(), false).wheels_fpm);
            }
        }
        assert!(tracker_worst < -400.0, "{tracker_worst:.0}");
    }

    /// The recorded balloon, recovered: the column as written to the tick
    /// the servos went back (+263 fpm at 50.5 ft), then the arrest keeps
    /// them. The rebuilt sink is arrested: under 320 fpm on every plant in
    /// still air, under 420 in 40 fpm rms of gusts (ten seeds each), where
    /// the hand-back met the runway at -814.
    #[test]
    fn the_recorded_balloon_is_recovered_without_the_autopilot() {
        let mut worst_calm: f64 = 0.0;
        let mut worst_rough: f64 = 0.0;
        for fit in PLANTS {
            for seed in 0..=10u64 {
                let mut p = at_the_gate(fit);
                if seed > 0 {
                    p = p.rough(seed, 40.0);
                }
                p.recorded(HANDED_BACK_S);
                if seed == 0 && fit == PLANTS[LEMG_FIT] {
                    // Still climbing (recorded +263; the fit, whose column
                    // was -0.05 for a dead time already, +104).
                    assert!(p.v > BALLOON_CLIMB_FPM, "the balloon: {:.0} fpm", p.v);
                }
                let l = land(&mut p, &mut Law::today(), seed > 0);
                assert!(!l.pushed_climbing, "{fit:?} seed {seed}");
                if seed == 0 {
                    worst_calm = worst_calm.min(l.wheels_fpm);
                } else {
                    worst_rough = worst_rough.min(l.wheels_fpm);
                }
            }
        }
        assert!(worst_calm > -300.0, "{worst_calm:.0}");
        assert!(worst_rough > -400.0, "{worst_rough:.0}");
    }

    /// Rough air, every recorded gate and plant, ten gust seeds each at
    /// the 40 fpm rms the rough-air gate starts at: the SF50's prediction
    /// flown in any air lands all 160 under 400 (the worst 236). The 737's
    /// rough-air hold on the same elevator (pull only ever back) turns the
    /// pulls into floats that bleed the speed onto the runway: 57 of them
    /// past 400 fpm, the worst 1,098.
    #[test]
    fn in_rough_air_the_sf50_flies_its_prediction() {
        let held = {
            let mut p = Law::sf50_elevator();
            if let Some(m) = p.model.as_mut() {
                m.rough_hold = None;
            }
            p
        };
        let gates: Vec<_> = std::iter::once((REC[0].1, REC[0].2, GATE_TREND, GATE_IAS))
            .chain(OTHER_GATES)
            .collect();
        let (mut worst, mut n, mut held_hard, mut worst_held) = (0.0_f64, 0, 0, 0.0_f64);
        for &(h, v, trend, ias) in &gates {
            for fit in PLANTS {
                for seed in 1..=10u64 {
                    let mut p = Plant::at(fit, h, v, trend, ias).rough(seed, 40.0);
                    let l = land(&mut p, &mut Law::today(), true);
                    assert!(!l.pushed_climbing, "{fit:?} seed {seed}");
                    worst = worst.min(l.wheels_fpm);
                    n += 1;
                    let mut p = Plant::at(fit, h, v, trend, ias).rough(seed, 40.0);
                    let law = FlareModel::from_profile(Some(held)).expect("model");
                    let l = land(&mut p, &mut Law::Model(law), true);
                    held_hard += usize::from(l.wheels_fpm < -400.0);
                    worst_held = worst_held.min(l.wheels_fpm);
                }
            }
        }
        assert!(worst > -400.0, "{worst:.0} over {n}");
        assert!(
            held_hard * 5 > n,
            "held: {held_hard} of {n} past 400, worst {worst_held:.0}"
        );
    }
