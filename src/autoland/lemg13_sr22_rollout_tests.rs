    use super::*;

    /// The recorded rollout ticks (s from the first, groundspeed kt, true
    /// heading, cross-track m right, the heading error, the rudder steer).
    const REC: [(f64, f64, f64, f64, f64, f64); 36] = [
        (0.0, 73.9, 131.22, 1.65, 0.14, 0.004),
        (0.4, 72.9, 132.07, 1.55, -0.67, -0.045),
        (0.8, 71.8, 133.34, 1.56, -1.96, -0.094),
        (1.2, 70.8, 133.78, 1.74, -2.52, -0.102),
        (1.6, 68.8, 136.47, 2.15, -5.44, -0.257),
        (2.0, 66.7, 137.92, 2.70, -7.19, -0.305),
        (2.5, 66.0, 136.75, 3.40, -6.34, -0.229),
        (2.8, 64.5, 134.53, 4.07, -4.43, -0.166),
        (3.3, 62.6, 132.86, 4.71, -3.03, -0.160),
        (3.6, 60.5, 132.71, 5.31, -3.15, -0.219),
        (4.1, 58.3, 133.85, 5.87, -4.51, -0.304),
        (4.4, 56.2, 135.59, 6.34, -6.47, -0.405),
        (4.8, 53.9, 136.96, 6.87, -8.11, -0.463),
        (5.2, 51.4, 137.45, 7.53, -8.98, -0.485),
        (5.6, 49.2, 136.96, 8.16, -8.89, -0.492),
        (6.1, 46.5, 135.41, 8.97, -7.89, -0.490),
        (6.5, 44.2, 134.09, 9.62, -7.04, -0.510),
        (6.9, 42.2, 133.30, 10.14, -6.59, -0.552),
        (7.3, 39.6, 132.86, 10.67, -6.61, -0.600),
        (7.7, 37.1, 132.88, 11.09, -6.67, -0.631),
        (8.1, 34.6, 133.05, 11.39, -6.84, -0.671),
        (8.5, 32.0, 133.04, 11.63, -6.80, -0.686),
        (8.9, 29.5, 132.63, 11.84, -6.36, -0.662),
        (9.3, 27.4, 132.04, 11.98, -5.72, -0.624),
        (9.7, 25.0, 131.27, 12.06, -4.85, -0.575),
        (10.1, 22.8, 130.56, 12.07, -4.03, -0.527),
        (10.6, 20.3, 129.81, 12.01, -3.12, -0.472),
        (11.0, 18.2, 129.16, 11.91, -2.32, -0.417),
        (11.4, 15.8, 128.43, 11.76, -1.41, -0.349),
        (11.8, 13.9, 127.83, 11.61, -0.69, -0.291),
        (12.2, 11.5, 127.09, 11.42, 0.21, -0.216),
        (12.6, 9.5, 126.42, 11.25, 0.99, -0.149),
        (13.0, 7.1, 125.61, 11.07, 1.90, -0.074),
        (13.4, 5.1, 124.91, 10.95, 2.65, -0.010),
        (13.9, 3.2, 124.19, 10.85, 3.39, 0.039),
        (14.2, 0.6, 123.26, 10.79, 4.32, 0.147),
    ];
    /// LEMG 13's true course: the first tick's heading plus its error plus
    /// the cut its 1.65 m asked (131.22 + 0.14 + 0.33).
    const COURSE: f64 = 131.69;
    const TICK_S: f64 = 0.4;
    const CHANNEL_DEAD_S: f64 = 0.3;
    const YAW_LAG_S: f64 = 0.5;

    /// Column `i` of the record at `t`, linear between ticks.
    fn at(t: f64, i: usize) -> f64 {
        let col = |r: &(f64, f64, f64, f64, f64, f64)| match i {
            1 => r.1,
            2 => r.2,
            _ => r.3,
        };
        if t <= REC[0].0 {
            return col(&REC[0]);
        }
        for w in REC.windows(2) {
            if (w[0].0..=w[1].0).contains(&t) {
                let f = (t - w[0].0) / (w[1].0 - w[0].0);
                return col(&w[0]) + f * (col(&w[1]) - col(&w[0]));
            }
        }
        col(&REC[REC.len() - 1])
    }

    /// The rollout from the first tick to the stop with the brakes split by
    /// the law (or not), `authority` deg/s of yaw per unit of brake
    /// difference at the wheels. The worst cross-track, metres.
    fn roll(authority: f64, split: bool) -> f64 {
        roll_with(authority, split, None)
    }

    /// [`roll`] with the free nose's one ground law on `plant` (`None` the
    /// shared split).
    pub(super) fn roll_with(
        authority: f64,
        split: bool,
        plant: Option<crate::ground_steer::GroundPlant>,
    ) -> f64 {
        let end = REC[REC.len() - 1].0;
        let dt = 0.01;
        let (mut t, mut next) = (0.0, 0.0);
        let (mut dpsi, mut yaw, mut dx) = (0.0_f64, 0.0_f64, 0.0_f64);
        let mut rate = crate::autotakeoff::XtkRate::default();
        let mut prev: Option<f64> = None;
        let mut law = plant.map_or_else(RolloutBrakeSteer::default, RolloutBrakeSteer::free);
        let mut written: Vec<(f64, f64)> = vec![(-10.0, 0.0)];
        let mut worst: f64 = 0.0;
        while t < end {
            let gs = at(t, 1);
            if t + 1e-9 >= next {
                next += TICK_S;
                let x = at(t, 3) + dx;
                let r = rate.update(prev.map_or(0.0, |p| (x - p) / TICK_S), TICK_S);
                prev = Some(x);
                let target = crate::autotakeoff::roll_target_heading(COURSE, x, r, gs);
                let psi = at(t, 2) + dpsi;
                let err = crate::angles::signed_diff_deg(target, psi);
                // The stop as flown: both brakes at 0.24.
                let brake = 0.24;
                let b = if split {
                    law.step_with(err, Some(psi), None, 0.0, TICK_S, gs, brake)
                } else {
                    RolloutBrakes {
                        sym: brake,
                        left: 0.0,
                        right: 0.0,
                    }
                };
                // The plugin: each wheel the harder of the pair.
                let diff = b.sym.max(b.right) - b.sym.max(b.left);
                written.push((t, diff));
            }
            let arriving = written
                .iter()
                .take_while(|(tw, _)| *tw <= t - CHANNEL_DEAD_S + 1e-9)
                .last()
                .map_or(0.0, |w| w.1);
            yaw += (authority * arriving - yaw) * dt / YAW_LAG_S;
            dpsi += yaw * dt;
            dx += gs * 0.514_444 * dpsi.to_radians().sin() * dt;
            worst = worst.max((at(t, 3) + dx).abs());
            t += dt;
        }
        worst
    }

    /// As flown the record is the plant: 12.1 m right. Split by the law the
    /// brakes hold the same rollout inside 2.6 m at every authority from 15
    /// to 60 deg/s per unit, and inside 3.5 m at 10 and 90 (3.1 and 3.3; the start is 1.65 m off).
    #[test]
    fn the_sr22_rollout_steers_on_its_brakes() {
        let flown = roll(30.0, false);
        assert!((flown - 12.07).abs() < 0.1, "as flown: {flown:.2}");
        for a in [15.0, 20.0, 30.0, 45.0, 60.0] {
            let w = roll(a, true);
            assert!(w < 2.6, "{a} deg/s per unit: {w:.2} m");
        }
        for a in [10.0, 90.0] {
            let w = roll(a, true);
            assert!(w < 3.5, "{a} deg/s per unit: {w:.2} m");
        }
    }

    /// The law: nose right of the target is the left wheel, the difference
    /// taken off the outside wheel so the stop is kept, capped, damped on
    /// the error's rate; nothing below 4 kt or on an unread heading.
    #[test]
    fn a_rollout_brake_split_keeps_the_stop() {
        let mut law = RolloutBrakeSteer::default();
        let b = law.step(-5.0, 0.4, 60.0, 0.24);
        assert!((b.left - 0.315).abs() < 1e-9, "{b:?}");
        assert!((b.sym - 0.165).abs() < 1e-9, "{b:?}");
        assert_eq!(b.right, 0.0);
        // The error closing at 5 deg/s leads it off.
        let b = law.step(-3.0, 0.4, 60.0, 0.24);
        assert!(b.right > 0.0 && b.left == 0.0, "{b:?}");
        // Capped at 0.3 of difference.
        let mut law = RolloutBrakeSteer::default();
        let b = law.step(40.0, 0.4, 60.0, 0.24);
        assert!((b.right - b.sym - 0.3).abs() < 1e-9, "{b:?}");
        // A light stop cannot go below zero on the outside wheel.
        let mut law = RolloutBrakeSteer::default();
        let b = law.step(40.0, 0.4, 60.0, 0.05);
        assert_eq!(b.sym, 0.0);
        assert!((b.right - 0.2).abs() < 1e-9, "{b:?}");
        // Slow, or no heading: the plain stop.
        let mut law = RolloutBrakeSteer::default();
        let plain = RolloutBrakes {
            sym: 0.24,
            left: 0.0,
            right: 0.0,
        };
        assert_eq!(law.step(-5.0, 0.4, 3.0, 0.24), plain);
        assert_eq!(law.step(f64::NAN, 0.4, 60.0, 0.24), plain);
    }
