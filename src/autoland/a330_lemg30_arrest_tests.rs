    use super::*;

    /// The commit that lasted one tick: 46.4 ft, 1,432 m ahead of a 4,750 ft
    /// (1,448 m) book, then 46.1 ft and about 1,409 m. Judged afresh, the
    /// second tick is a go-around; latched, the landing continues while the
    /// pavement covers the book's ground roll, and not past it.
    #[test]
    fn a_committed_flare_stays_committed() {
        let book = Some(4750.0 * M_PER_FT);
        let need = Some(wheels_need_m(book, None));
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
        let held = commit_flare_latched(over, true, need, 46.1, 150.0, 25.0, 1409.0, book, 297.7);
        assert_eq!((held.phase, held.balk), (ApproachPhase::Flare, None));
        assert!(held.arresting);
        // Past the ground roll (1,448 m less the 291 m air segment): around.
        let short = commit_flare_latched(over, true, need, 30.0, 150.0, 25.0, 1100.0, book, 297.7);
        assert_eq!(short.balk, Some("overflew_the_runway"));
        // Never above the band, latched or not.
        let (_, band) = arrest_gates_ft(25.0, 150.0);
        let high =
            commit_flare_latched(over, true, need, band + 1.0, 150.0, 25.0, 2000.0, book, 297.7);
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
