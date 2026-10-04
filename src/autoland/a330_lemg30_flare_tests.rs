    use super::*;

    /// The recorded arrest of the first approach (s from `arrest_hands`
    /// taken, radio ft, vvi fpm, the elevator written, IAS kt), every
    /// other tick, to the tick before the servos went back.
    const FLOAT: [(f64, f64, f64, f64, f64); 40] = [
        (0.0, 86.85, -1154.8, 0.350, 162.29),
        (0.7, 72.30, -1141.2, 0.350, 162.53),
        (1.4, 58.52, -1112.6, 0.350, 162.90),
        (2.0, 47.93, -1014.6, 0.350, 163.28),
        (2.7, 40.41, -855.3, 0.350, 163.62),
        (3.1, 36.77, -737.9, 0.350, 163.75),
        (3.6, 33.96, -611.5, 0.350, 163.84),
        (3.9, 32.18, -506.0, 0.350, 163.88),
        (4.4, 30.72, -389.1, 0.350, 163.89),
        (4.8, 29.95, -274.3, 0.350, 163.82),
        (5.2, 29.88, -195.1, 0.350, 163.75),
        (5.6, 30.41, -95.6, 0.203, 163.57),
        (6.0, 31.44, -8.8, 0.147, 163.33),
        (6.4, 32.90, 70.6, 0.093, 163.08),
        (6.8, 34.93, 154.5, 0.037, 162.72),
        (7.3, 37.37, 223.0, 0.000, 162.25),
        (7.6, 39.29, 258.9, 0.000, 161.77),
        (8.1, 41.32, 277.2, 0.000, 161.15),
        (8.5, 42.60, 273.1, 0.000, 160.56),
        (8.9, 43.41, 256.1, 0.000, 160.12),
        (9.3, 44.06, 229.4, 0.000, 159.70),
        (9.7, 45.12, 200.3, 0.000, 159.35),
        (10.1, 46.30, 175.9, 0.000, 159.08),
        (10.5, 46.89, 150.7, 0.000, 158.77),
        (11.0, 47.16, 130.6, 0.000, 158.51),
        (11.3, 47.38, 116.1, 0.000, 158.30),
        (11.8, 47.61, 102.8, 0.000, 158.10),
        (12.1, 47.59, 91.6, 0.000, 157.93),
        (12.5, 47.78, 81.3, 0.000, 157.78),
        (12.9, 47.87, 69.6, 0.000, 157.65),
        (13.4, 48.16, 57.1, 0.000, 157.50),
        (13.7, 48.28, 45.4, 0.000, 157.36),
        (14.1, 48.31, 32.7, 0.000, 157.23),
        (14.6, 48.36, 20.8, 0.000, 157.10),
        (14.9, 48.24, 10.4, 0.000, 157.00),
        (15.4, 48.09, -1.5, 0.000, 156.87),
        (15.8, 47.88, -13.5, 0.000, 156.74),
        (16.2, 47.48, -25.3, 0.000, 156.58),
        (16.6, 46.99, -39.3, 0.000, 156.34),
        (17.0, 46.40, -56.3, 0.000, 156.03),
    ];
    /// Every recorded A330 gate (radio ft, vvi fpm, trend fpm/s, IAS kt):
    /// LEMG 13 (1790621184), LEMG 30's circuit (1790633832), the floated
    /// approach and the one that landed (1790634412).
    const GATES: [(f64, f64, f64, f64); 4] = [
        (87.9, -913.9, 37.6, 162.3),
        (86.4, -942.8, -13.9, 161.2),
        (86.9, -1154.8, 20.0, 162.3),
        (90.8, -810.7, 26.3, 164.1),
    ];
    const FLOATED: usize = 2;
    /// (fpm/s per unit, lag 1, lag 2, dead s, ground fpm/s at the wheels,
    /// from ft, fpm per knot lost): each arrest's own fit, then all four.
    type Fit = (f64, f64, f64, f64, f64, f64, f64);
    const PLANTS: [Fit; 5] = [
        (824.0, 0.611, 0.602, 0.78, 164.6, 48.7, 42.9),
        (808.0, 0.235, 0.288, 1.633, 55.5, 107.3, 82.9),
        (764.0, 0.42, 0.438, 1.12, 10.95, 70.1, 75.6),
        (598.0, 0.151, 0.275, 1.38, 75.1, 41.7, 45.35),
        (777.0, 0.1, 0.465, 1.67, 60.0, 49.2, 94.1),
    ];
    const TICK_S: f64 = 0.2;

    struct Plant {
        fit: Fit,
        h: f64,
        v0: f64,
        v: f64,
        ias0: f64,
        ias: f64,
        integ: f64,
        b1: f64,
        b2: f64,
        gust: f64,
        rng: u64,
        rough_fpm: f64,
        t: f64,
        written: Vec<(f64, f64)>,
    }

    impl Plant {
        /// At a gate: sinking `v` with `trend` (the autopilot's own rate,
        /// standing in the pipe), at `ias`.
        fn at(fit: Fit, gate: (f64, f64, f64, f64)) -> Self {
            let (h, v, trend, ias) = gate;
            Plant {
                fit,
                h,
                v0: v,
                v,
                ias0: ias,
                ias,
                integ: 0.0,
                b1: trend,
                b2: trend,
                gust: 0.0,
                rng: 0,
                rough_fpm: 0.0,
                t: 0.0,
                written: vec![(-10.0, trend / fit.0)],
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
        /// One step; `ias` recorded, else the plant's own bleed.
        fn step_with(&mut self, dt: f64, ias: Option<f64>) {
            let (k, l1, l2, dead, gf, gft, spd) = self.fit;
            let due = self.t - dead;
            let arriving = self
                .written
                .iter()
                .take_while(|(t, _)| *t <= due + 1e-9)
                .last()
                .map_or(0.0, |w| w.1);
            self.b1 += (k * arriving - self.b1) * dt / l1;
            self.b2 += (self.b1 - self.b2) * dt / l2;
            let ground = gf * (1.0 - self.h.max(0.0) / gft).max(0.0);
            self.integ += (self.b2 - ground) * dt;
            if self.rough_fpm > 0.0 {
                const TC: f64 = 1.5;
                let n = self.gauss();
                self.gust += -self.gust / TC * dt + self.rough_fpm * (2.0 * dt / TC).sqrt() * n;
            }
            self.v = self.v0 + self.integ - spd * (self.ias0 - self.ias) + self.gust;
            self.ias = match ias {
                Some(i) => i,
                None => self.ias + (-1.0 - 1.3 * self.v / 1000.0) * dt,
            };
            self.h += self.v / 60.0 * dt;
            self.t += dt;
        }
    }

    enum Law {
        Model(FlareModel),
        Tracker(FlarePitch, Option<f64>),
    }

    impl Law {
        fn a330_elevator() -> crate::aircraft::FlareElevatorProfile {
            crate::aircraft::load("airbus_a330_300")
                .and_then(|p| p.flare_elevator)
                .expect("the A330 declares its flare")
        }
        fn today() -> Self {
            Law::Model(
                FlareModel::from_profile(Some(Self::a330_elevator()))
                    .expect("the A330 declares its model"),
            )
        }
        /// The tracker the A330 flew: no `flare_elevator:`, the default law.
        fn as_flown() -> Self {
            Law::Tracker(FlarePitch::with_gains(FlareGains::from_profile(None)), None)
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
        secs: f64,
        /// The steepest climb after the first three seconds, fpm.
        climb_fpm: f64,
    }

    /// The arrest from the gate to the wheels (or 25 s) at the loop's tick,
    /// the servos kept: the flare below its gate, and a flare stays a flare
    /// to the band's top.
    fn land(p: &mut Plant, law: &mut Law, rough: bool) -> Landed {
        let start = p.t;
        let (mut next, mut flaring) = (p.t, false);
        let mut climb: f64 = f64::MIN;
        while p.h > 0.0 && p.t - start < 25.0 {
            if p.t + 1e-9 >= next {
                next += TICK_S;
                let gs = p.ias + 3.0;
                let (flare_ft, band_ft) = arrest_gates_ft(25.0, gs);
                flaring = p.h <= flare_ft || (flaring && p.h <= band_ft);
                let e = law.step(p.h, gs, p.v, TICK_S, flaring, rough);
                p.write(e);
            }
            p.step_with(0.01, None);
            if p.t - start > 3.0 {
                climb = climb.max(p.v);
            }
        }
        Landed {
            wheels_fpm: if p.h <= 0.0 { p.v } else { f64::NAN },
            secs: p.t - start,
            climb_fpm: climb,
        }
    }

    /// The plant is the recording: the floated approach's column as written
    /// gives back its sink, the climb and the float included, under 25 fpm
    /// rms on its own fit (the other arrests' fits are other arrests).
    #[test]
    fn the_a330_plant_replays_the_float() {
        for (i, fit) in PLANTS.iter().enumerate() {
            // As fitted: nothing in the pipe before the first write.
            let mut p = Plant::at(*fit, (GATES[FLOATED].0, GATES[FLOATED].1, 0.0, FLOAT[0].4));
            p.written.clear();
            let mut sq = 0.0;
            for (j, r) in FLOAT.iter().enumerate().skip(1) {
                let prev = FLOAT[j - 1];
                while p.t < r.0 - 1e-9 {
                    if p.written.last().map(|w| w.1) != Some(prev.3) {
                        p.write(prev.3);
                    }
                    // The recorded speed, as the fit was made on it.
                    let f = ((p.t - prev.0) / (r.0 - prev.0)).clamp(0.0, 1.0);
                    p.step_with(0.01, Some(prev.4 + (r.4 - prev.4) * f));
                }
                sq += (p.v - r.2).powi(2);
            }
            let rms = (sq / (FLOAT.len() - 1) as f64).sqrt();
            if i == FLOATED {
                assert!(rms < 25.0, "its own fit: {rms:.1} fpm rms");
            }
        }
    }

    /// From every recorded gate on every fit, the declared model lands in
    /// still air under 250 fpm inside 13 s and never climbs; the tracker the
    /// A330 flew floats the floated approach's own plant past 15 s.
    #[test]
    fn the_a330_flare_lands_every_gate_on_every_plant() {
        for gate in GATES {
            for fit in PLANTS {
                let mut p = Plant::at(fit, gate);
                let l = land(&mut p, &mut Law::today(), false);
                assert!(l.wheels_fpm > -250.0, "{fit:?} from {gate:?}: {:.0}", l.wheels_fpm);
                assert!(l.secs < 13.0, "{fit:?} from {gate:?}: floated {:.1} s", l.secs);
                assert!(l.climb_fpm < 50.0, "{fit:?} from {gate:?}: climbed {:.0}", l.climb_fpm);
            }
        }
        let mut p = Plant::at(PLANTS[FLOATED], GATES[FLOATED]);
        let l = land(&mut p, &mut Law::as_flown(), false);
        assert!(l.secs > 15.0, "the tracker as flown: {:.1} s", l.secs);
        assert!(l.climb_fpm > 150.0, "and its balloon: {:.0} fpm", l.climb_fpm);
    }

    /// Rough air, every gate and plant, five gust seeds each at 40 fpm rms:
    /// the model lands them all, no more than one in twenty past 400 fpm,
    /// none past 560.
    #[test]
    fn in_rough_air_the_a330_lands_its_prediction() {
        let (mut worst, mut hard, mut n) = (0.0_f64, 0, 0);
        for gate in GATES {
            for fit in PLANTS {
                for seed in 1..=5u64 {
                    let mut p = Plant::at(fit, gate).rough(seed, 40.0);
                    let l = land(&mut p, &mut Law::today(), true);
                    assert!(l.wheels_fpm.is_finite(), "{fit:?} seed {seed}: never landed");
                    worst = worst.min(l.wheels_fpm);
                    hard += usize::from(l.wheels_fpm < -400.0);
                    n += 1;
                }
            }
        }
        assert!(worst > -560.0, "{worst:.0}");
        assert!(hard * 20 <= n, "{hard} of {n} past 400");
    }

    /// Only a declared rate plant flies the rate law; the 737's standing
    /// plant is untouched, and the rate law pushes a float down.
    #[test]
    fn a_rate_plant_pushes_a_float_down() {
        let mut m = FlareModel::from_profile(Some(Law::a330_elevator())).expect("model");
        // Level at 48 ft, the float's own numbers: the column comes forward.
        let mut e = 0.0;
        for _ in 0..5 {
            e = m.step(48.0, 160.0, 20.0, TICK_S, true, true).elev;
        }
        assert!(e < -0.05, "a float is pushed: {e:.3}");
        let p = Law::a330_elevator();
        assert!(p.model.is_some_and(|m| m.rate));
        let b738 = crate::aircraft::load("boeing_737_800")
            .and_then(|p| p.flare_elevator)
            .and_then(|f| f.model)
            .expect("the 737's model");
        assert!(!b738.rate);
    }

    /// The commit that lasted one tick: 46.4 ft, 1,432 m ahead of a 4,750 ft
    /// (1,448 m) book, then 46.1 ft and about 1,409 m. Judged afresh, the
    /// second tick is a go-around; latched, the landing continues while the
    /// pavement covers the book's ground roll, and not past it.
    #[test]
    fn a_committed_flare_stays_committed() {
        let book = Some(4750.0 * M_PER_FT);
        let over = Guidance {
            target_heading_deg: 120.0,
            target_vs_fpm: 1000.0,
            target_ias_kts: 150.0,
            dist_nm: 0.7,
            glidepath_alt_ft: 0.0,
            phase: ApproachPhase::Intercept,
            balk: Some("overflew_the_runway"),
            arresting: false,
        };
        let first = commit_flare(over, true, 46.4, 150.0, 25.0, 1432.0, book, 297.7);
        assert_eq!((first.phase, first.balk), (ApproachPhase::Flare, None));
        let fresh = commit_flare(over, true, 46.1, 150.0, 25.0, 1409.0, book, 297.7);
        assert_eq!(fresh.balk, Some("overflew_the_runway"), "as flown: released");
        let held = commit_flare_latched(over, true, true, 46.1, 150.0, 25.0, 1409.0, book, 297.7);
        assert_eq!((held.phase, held.balk), (ApproachPhase::Flare, None));
        assert!(held.arresting);
        // Past the ground roll (1,448 m less the 291 m air segment): around.
        let short = commit_flare_latched(over, true, true, 30.0, 150.0, 25.0, 1100.0, book, 297.7);
        assert_eq!(short.balk, Some("overflew_the_runway"));
        // Never above the band, latched or not.
        let (_, band) = arrest_gates_ft(25.0, 150.0);
        let high =
            commit_flare_latched(over, true, true, band + 1.0, 150.0, 25.0, 2000.0, book, 297.7);
        assert_eq!(high.balk, Some("overflew_the_runway"));
    }

    /// The arrest's lateral from a gate (x m, sideways m/s, bank deg,
    /// seconds to the wheels, gate ft) on a roll-rate plant (`k` deg/s per
    /// unit after `dead` s, a 0.07 s lag; the sideways push through 0.2 s):
    /// (offset at the wheels, worst overshoot, the bank's wobble about its
    /// own 2 s average, degrees rms).
    fn fly_roll(
        wings: &mut crate::autotakeoff::WingsLevel,
        gate: (f64, f64, f64, f64, f64),
        k: f64,
        dead: f64,
    ) -> (f64, f64, f64) {
        let (x0, v0, phi0, t_end, radio0) = gate;
        let (tick, h) = (0.2, 0.01);
        let (mut x, mut v, mut phi, mut p) = (x0, v0, phi0, 0.0_f64);
        let mut acc = 9.81 * phi0.to_radians().tan();
        let mut written: Vec<(f64, f64)> = Vec::new();
        let (mut t, mut next, mut worst) = (0.0, 0.0, 0.0_f64);
        let mut phis = Vec::new();
        while t < t_end {
            if t + 1e-9 >= next {
                next += tick;
                let radio = radio0 * (1.0 - t / t_end);
                let tgt = arrest_bank_target(x, v, wings.predicted(phi), radio);
                written.push((t, wings.step(phi, tgt, tick)));
            }
            let a = written
                .iter()
                .take_while(|w| w.0 <= t - dead + 1e-9)
                .last()
                .map_or(0.0, |w| w.1);
            p += (k * a - p) * h / 0.07;
            phi += p * h;
            acc += (9.81 * phi.to_radians().tan() - acc) * h / 0.2;
            v += acc * h;
            x += v * h;
            t += h;
            worst = worst.max(-x * x0.signum());
            phis.push(phi);
        }
        let w = 200;
        let (mut sq, mut n) = (0.0, 0);
        for i in (w + w / 2)..phis.len().saturating_sub(w / 2) {
            let mean: f64 = phis[i - w / 2..i + w / 2].iter().sum::<f64>() / w as f64;
            sq += (phis[i] - mean).powi(2);
            n += 1;
        }
        (x, worst, (sq / n.max(1) as f64).sqrt())
    }

    /// #180's roll: the A330's sidestick asks a roll rate (fitted 11 to 13
    /// deg/s per unit after 0.25 to 0.37 s; 9 to 20 and up to 0.6 s bound
    /// it). From every recorded gate, the stiff wings-level law rings on the
    /// stronger, later plants (the recorded +/-2 to 4 degrees at 1.5 to 2 s);
    /// the declared rate-plant law wobbles under half as much and lands
    /// within 2 m of the line. The floated approach's gate (8 m right,
    /// crossing left at 3.8 m/s, banked 3.4 the wrong way) is stopped by the
    /// 5 degree cap, not the law: no further past the line than the stiff
    /// law plus 1.5 m.
    #[test]
    fn the_a330_arrest_wings_do_not_ring() {
        let gates = [
            (0.55, 0.25, -1.47, 17.4, 87.9),
            (-7.5, 1.23, 5.19, 17.9, 86.4),
            (7.96, -3.8, -3.37, 17.3, 86.9),
            (1.07, 0.21, 0.6, 20.3, 90.8),
        ];
        let rate = crate::aircraft::load("airbus_a330_300")
            .and_then(|p| p.arrest_roll)
            .and_then(crate::aircraft::ArrestRollProfile::valid)
            .expect("the A330 declares its roll");
        let stiff = || crate::autotakeoff::WingsLevel::stiff(ARREST_FULL_AIL_BANK_DEG);
        let (mut stiff_ring, mut new_ring) = (0.0_f64, 0.0_f64);
        for (i, gate) in gates.iter().enumerate() {
            for (k, dead) in [(13.0, 0.35), (20.0, 0.35), (13.0, 0.6), (20.0, 0.6), (9.0, 0.35)] {
                let (_, s_worst, s_ring) = fly_roll(&mut stiff(), *gate, k, dead);
                let mut w = crate::autotakeoff::WingsLevel::rate_plant(rate);
                let (x, worst, ring) = fly_roll(&mut w, *gate, k, dead);
                stiff_ring = stiff_ring.max(s_ring);
                new_ring = new_ring.max(ring);
                assert!(x.abs() < 2.0, "gate {i} at {k}/{dead}: lands {x:.1} m off");
                if i == 2 {
                    assert!(worst < s_worst + 1.5, "gate {i}, {k}/{dead}: {worst:.1}/{s_worst:.1}");
                } else {
                    assert!(worst < 4.0, "gate {i} at {k}/{dead}: overshoots {worst:.1}");
                }
            }
        }
        assert!(stiff_ring > 1.5, "the stiff law rings: {stiff_ring:.2}");
        assert!(new_ring < 0.5 * stiff_ring, "{new_ring:.2} against {stiff_ring:.2}");
    }

    /// A slow tick is a slower loop: the iMac's 19 fps stretched the
    /// arrest's tick to 0.4 to 0.44 s, and the stiff wings-level law, set
    /// on 0.25 to 0.3, rang +/-5 degrees on the aileron's stops through the
    /// 737's flare (EGKK 08R 1790687142, 2026-09-29: 12 of 17 flare ticks
    /// at 0.5). On the arrest test's 737 roll plant (20 deg/s per unit,
    /// 0.3 s lag, 0.3 s dead) the gain now softens with the tick.
    #[test]
    fn a_slow_tick_does_not_ring_the_arrest_wings() {
        let fly = |tick: f64| -> (f64, usize) {
            let mut wings = arrest_wings();
            let (x0, v0, phi0, t_end, radio0): (f64, f64, f64, f64, f64) =
                (1.2, -0.3, 3.9, 9.0, 60.0);
            let (h, k, lag, dead) = (0.01, 20.0, 0.3, 0.3);
            let (mut x, mut v, mut phi, mut p) = (x0, v0, phi0, 0.0_f64);
            let mut acc = 9.81 * phi0.to_radians().tan();
            let mut written: Vec<(f64, f64)> = Vec::new();
            let (mut t, mut next) = (0.0, 0.0);
            let (mut swing, mut sat) = (0.0_f64, 0usize);
            while t < t_end {
                if t + 1e-9 >= next {
                    next += tick;
                    let radio = radio0 * (1.0 - t / t_end);
                    let tgt = arrest_bank_target(x, v, wings.predicted(phi), radio);
                    let a = wings.step(phi, tgt, tick);
                    sat += usize::from(a.abs() > 0.49);
                    written.push((t, a));
                }
                let a = written
                    .iter()
                    .take_while(|w| w.0 <= t - dead + 1e-9)
                    .last()
                    .map_or(0.0, |w| w.1);
                p += (k * a - p) * h / lag;
                phi += p * h;
                acc += (9.81 * phi.to_radians().tan() - acc) * h / 0.2;
                v += acc * h;
                x += v * h;
                t += h;
                if t > 3.0 {
                    swing = swing.max(phi.abs());
                }
            }
            (swing, sat)
        };
        let (fast, _) = fly(0.25);
        for tick in [0.4, 0.44] {
            let (swing, sat) = fly(tick);
            assert_eq!(sat, 0, "tick {tick}: {sat} ticks on the stops");
            assert!(swing < 2.0, "tick {tick}: swings {swing:.1} degrees");
        }
        assert!(fast < 2.0, "the tuned tick is the one it was: {fast:.1}");
    }
