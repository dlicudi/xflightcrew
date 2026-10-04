    use super::*;

    const SRC: [&str; 3] = [
        include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/landings/king_air_lemg13_landing_1790757702.jsonl")),
        include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/landings/king_air_lemg13_landing_1790758429.jsonl")),
        include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/landings/king_air_lemg13_landing_1790758604.jsonl")),
    ];
    /// The loop's tick on the iMac's rollouts.
    const TICK_S: f64 = 0.45;
    /// The rollout's braking on the recorded ticks, kt/s (86 to 60 kt in 4 s).
    const DECEL_KTS_S: f64 = 6.0;

    /// A rollout tick: (t from the first, gs, psi, track, steer written,
    /// cross-track m, cross-track rate m/s, the error the law saw).
    type Tick = (f64, f64, f64, f64, f64, f64, f64, f64);

    fn ticks(src: &str) -> Vec<Tick> {
        let mut out: Vec<Tick> = vec![];
        let mut t0 = None;
        for l in src.lines().filter(|l| !l.trim().is_empty()) {
            let r: serde_json::Value = serde_json::from_str(l).expect("a record");
            if r["event"] != "rollout" {
                continue;
            }
            let t = r["t"].as_f64().unwrap();
            let t0 = *t0.get_or_insert(t);
            let f = |k: &str| r[k].as_f64().unwrap();
            out.push((t - t0, f("gs"), f("psi"), f("trk"), f("steer"), f("xtk_m"), f("xtk_rate"), f("err")));
        }
        out
    }

    /// The course the rollout steered for: the recorded error on the first
    /// tick plus the centreline cut it held (LEMG 13's true course).
    fn course(t: &Tick) -> f64 {
        let cut = -crate::autotakeoff::roll_target_heading(0.0, t.5, t.6, t.1);
        let cut = crate::angles::wrap180(cut);
        (t.2 + t.7 + cut).rem_euclid(360.0)
    }

    /// The yaw on the wheels: the yaw rate goes toward ks x (gs/80)^2 x the
    /// steer `dead` s ago plus kb x the slip x (gs/80)^n, over `tau`; the
    /// track follows the nose at `lam` per second of slip.
    #[derive(Debug, Clone, Copy)]
    struct Yaw {
        ks: f64,
        kb: f64,
        n: f64,
        tau: f64,
        dead: f64,
        lam: f64,
        /// The contact's yaw rate, deg/s, per recorded rollout.
        r0: [f64; 3],
    }

    const A: Yaw = Yaw { ks: 62.5, kb: 1.52, n: 0.52, tau: 0.41, dead: 0.29, lam: 0.61, r0: [-16.8, -16.7, -18.8] };
    const B: Yaw = Yaw { ks: 42.6, kb: 2.26, n: 3.0, tau: 0.08, dead: 0.30, lam: 0.69, r0: [-40.0, -5.8, -24.8] };
    const C: Yaw = Yaw { ks: 219.0, kb: 0.0, n: 1.0, tau: 3.0, dead: 0.32, lam: 0.71, r0: [-6.7, -10.6, -13.3] };

    struct Sim {
        y: Yaw,
        psi: f64,
        trk: f64,
        r: f64,
        gs: f64,
        xtk: f64,
        course: f64,
        writes: Vec<(f64, f64)>,
        t: f64,
    }

    impl Sim {
        fn new(y: Yaw, first: &Tick, r0: f64, course: f64) -> Self {
            Sim {
                y,
                psi: first.2,
                trk: first.3,
                r: r0,
                gs: first.1,
                xtk: first.5,
                course,
                writes: vec![(-9.0, 0.0)],
                t: 0.0,
            }
        }

        fn step(&mut self, dt: f64, decel: Option<f64>) {
            let due = self.t - self.y.dead;
            let u = self
                .writes
                .iter()
                .take_while(|w| w.0 <= due + 1e-9)
                .last()
                .map_or(0.0, |w| w.1);
            let q = self.gs / 80.0;
            let slip = self.psi - self.trk;
            self.r += (self.y.ks * q * q * u + self.y.kb * slip * q.powf(self.y.n) - self.r) / self.y.tau
                * dt;
            self.psi += self.r * dt;
            self.trk += self.y.lam * slip * dt;
            self.xtk += self.gs / MS_TO_KTS * (self.trk - self.course).to_radians().sin() * dt;
            if let Some(a) = decel {
                self.gs -= a * dt;
            }
            self.t += dt;
        }
    }

    /// The plant is the recording: each rollout's steer as written, open
    /// loop, gives back its heading over its first three and a half seconds
    /// within 3.5 degrees rms on the slip fits and 5 on the one without
    /// (1.4 to 3.2 and 2.2 to 4.6 as fitted).
    #[test]
    fn the_king_air_yaw_plants_replay_the_rollouts() {
        for y in [A, B, C] {
            for (i, src) in SRC.iter().enumerate() {
                let tk = ticks(src);
                let mut s = Sim::new(y, &tk[0], y.r0[i], course(&tk[0]));
                let (mut se, mut n) = (0.0, 0.0);
                for w in tk.iter().filter(|w| w.0 <= 3.5) {
                    while s.t < w.0 - 1e-9 {
                        s.step(0.01, None);
                    }
                    se += (s.psi - w.2).powi(2);
                    n += 1.0;
                    s.gs = w.1;
                    s.writes.push((w.0, w.4));
                }
                let rms = (se / n).sqrt();
                eprintln!("{y:?} rollout {i}: {rms:.2} deg rms");
                let limit = if y.kb > 0.0 { 3.5 } else { 5.0 };
                assert!(rms < limit, "{y:?} rollout {i}: {rms:.2} deg rms");
            }
        }
    }

    /// One closed-loop rollout from a recorded contact down to 40 kt: the
    /// worst heading off the course and the worst cross-track.
    fn roll(y: Yaw, i: usize, mut law: impl FnMut(f64, f64, f64, f64) -> f64) -> (f64, f64) {
        let tk = ticks(SRC[i]);
        let c = course(&tk[0]);
        let mut s = Sim::new(y, &tk[0], y.r0[i], c);
        let mut rate = crate::autotakeoff::XtkRate::default();
        let mut prev = s.xtk;
        let (mut worst_h, mut worst_x) = (0.0_f64, s.xtk.abs());
        let mut next = 0.0;
        while s.gs > 40.0 {
            if s.t + 1e-9 >= next {
                let xr = if next > 0.0 {
                    rate.update((s.xtk - prev) / TICK_S, TICK_S)
                } else {
                    tk[0].6
                };
                prev = s.xtk;
                next += TICK_S;
                let target = crate::autotakeoff::roll_target_heading(c, s.xtk, xr, s.gs);
                let err = crate::angles::signed_diff_deg(target, s.psi);
                let u = law(err, TICK_S, s.gs, s.r);
                s.writes.push((s.t, u));
            }
            s.step(0.01, Some(DECEL_KTS_S));
            worst_h = worst_h.max((s.psi - c).abs());
            worst_x = worst_x.max(s.xtk.abs());
            if worst_h > 90.0 {
                break;
            }
        }
        (worst_h, worst_x)
    }

    fn king_air_law() -> RolloutSteer {
        let p = crate::aircraft::load("king_air_c90b").expect("the King Air profile loads");
        RolloutSteer::new(
            p.rollout_steer
                .and_then(crate::aircraft::RolloutSteerProfile::valid)
                .expect("the King Air declares its rollout steer"),
        )
    }

    /// The takeoff roll's default law as it flew the three rollouts swings
    /// the nose past 20 degrees from every contact on every fit, and on the
    /// best fit (A) loses it past 45 from two of the three, as the sim did;
    /// the declared law from every contact on every fit holds the nose
    /// within 15 degrees of the course and the wheels within 15 m of the
    /// centreline (LEMG 13 is 45 m wide).
    #[test]
    fn the_king_air_rollout_holds_the_runway_where_the_roll_law_lost_it() {
        let mut lost_on_a = 0;
        let mut flown_least = f64::INFINITY;
        let mut worst = (0.0_f64, 0.0_f64);
        for (name, y) in [("A", A), ("B", B), ("C", C)] {
            for i in 0..3 {
                let mut flown = crate::autotakeoff::RollSteer::default();
                let (fh, fx) = roll(y, i, |e, dt, gs, _| flown.step(e, dt, gs, 0.0));
                flown_least = flown_least.min(fh);
                if name == "A" && fh > 45.0 {
                    lost_on_a += 1;
                }
                let mut law = king_air_law();
                let (h, x) = roll(y, i, |e, dt, gs, r| law.step(e, dt, gs, Some(r)));
                eprintln!("{name} {i}: as flown {fh:.1} deg, {fx:.1} m; declared {h:.1} deg, {x:.1} m");
                assert!(h < 15.0 && x < 15.0, "{name} {i}: {h:.1} deg, {x:.1} m");
                worst = (worst.0.max(h), worst.1.max(x));
            }
        }
        assert!(flown_least > 20.0, "the roll law as flown held one within {flown_least:.1} degrees");
        assert!(lost_on_a >= 2, "the roll law as flown lost {lost_on_a} of 3 on the best fit");
        eprintln!(
            "King Air rollout: as flown at least {flown_least:.1} deg; declared worst {:.1} deg, {:.1} m",
            worst.0, worst.1
        );
    }

    /// Slow, the nosewheel's own steering (12 to 40 deg/s per unit after 0.3
    /// to 0.5 s): the default law's full authority per 12 degrees sawed
    /// +/-10 to 15 degrees at 10 to 30 kt on the recorded ticks, and saws
    /// past 10 on the 40 deg/s nosewheel here; the declared law settles
    /// within 4 degrees after four seconds on every one.
    #[test]
    fn the_king_air_rollout_settles_on_the_nosewheel() {
        fn fly(ks: f64, tau: f64, dead: f64, e0: f64, r0: f64, law: &mut dyn FnMut(f64, f64, f64, f64) -> f64) -> f64 {
            let (mut psi, mut r, mut gs, mut t, mut next) = (-e0, r0, 40.0_f64, 0.0, 0.0);
            let mut writes = vec![(-9.0, 0.0)];
            let mut late = 0.0_f64;
            while gs > 5.0 {
                if t + 1e-9 >= next {
                    next += TICK_S;
                    writes.push((t, law(-psi, TICK_S, gs, r)));
                }
                let u = writes
                    .iter()
                    .take_while(|w: &&(f64, f64)| w.0 <= t - dead + 1e-9)
                    .last()
                    .map_or(0.0, |w| w.1);
                r += (ks * u - r) / tau * 0.01;
                psi += r * 0.01;
                gs -= 3.0 * 0.01;
                t += 0.01;
                if t > 4.0 {
                    late = late.max(psi.abs());
                }
            }
            late
        }
        for (ks, tau, dead) in [(20.0, 0.2, 0.4), (40.0, 0.3, 0.5), (12.0, 0.15, 0.3)] {
            for (e0, r0) in [(10.0, 0.0), (0.0, 15.0), (-8.0, -10.0)] {
                let mut flown = crate::autotakeoff::RollSteer::default();
                let as_flown = fly(ks, tau, dead, e0, r0, &mut |e, dt, gs, _| flown.step(e, dt, gs, 0.0));
                let mut law = king_air_law();
                let declared = fly(ks, tau, dead, e0, r0, &mut |e, dt, gs, r| law.step(e, dt, gs, Some(r)));
                eprintln!("slow {ks} {tau} {dead} from {e0} {r0}: as flown {as_flown:.1}, declared {declared:.1}");
                assert!(declared < 4.0, "{ks} {tau} {dead} {e0} {r0}: {declared:.1}");
                if ks >= 40.0 {
                    assert!(as_flown > 10.0, "as flown {ks} {e0} {r0}: {as_flown:.1}");
                }
            }
        }
    }

    /// The law's shape: full authority slow, `max_fast` at speed, the gain
    /// rising as the speed falls to `min_kts`, damped on the yaw rate
    /// against the error, nothing carried through the air, and only the
    /// King Air declares it.
    #[test]
    fn rollout_steer_schedules_and_damps() {
        let mut law = king_air_law();
        assert!((law.cap_at(85.0) - law.prof.max_fast).abs() < 1e-9);
        assert!((law.cap_at(20.0) - ROLLOUT_STEER_MAX).abs() < 1e-9);
        assert!(law.per_deg_at(40.0) > law.per_deg_at(80.0));
        assert!((law.per_deg_at(120.0) - law.per_deg_at(80.0)).abs() < 1e-12);
        // The nose swinging left at 17 deg/s with no error yet: right steer.
        assert!(law.step(0.0, 0.45, 85.0, Some(-17.0)) > 0.0);
        law.in_air();
        assert_eq!(law.out, 0.0);
        for id in [
            "cirrus_sr22",
            "boeing_737_800",
            "cirrus_vision_sf50",
            "lancair_evolution",
            "beechcraft_baron_58",
            "airbus_a330_300",
        ] {
            let p = crate::aircraft::load(id).expect("the profile loads");
            assert!(p.rollout_steer.is_none(), "{id}");
        }
    }
