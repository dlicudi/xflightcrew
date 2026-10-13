    use super::*;
    use crate::ground_steer::GroundPlant;

    /// The split 1790694457 flew (aa5ab0f), kept here to replay it: 0.025
    /// of difference per degree, 1.2 s of lead on the raw rate, a trim of
    /// 0.8 per second to 0.15. (The SR22's later declared schedule, the
    /// `rollout_brakes:` block, is retired for the one ground law.)
    #[derive(Default)]
    struct AsFlown2 {
        prev: Option<f64>,
        rate: f64,
        trim: f64,
    }

    impl AsFlown2 {
        fn step(&mut self, err: f64, dt: f64, gs: f64, brake: f64) -> RolloutBrakes {
            if let Some(p) = self.prev {
                self.rate = (err - p) / dt.clamp(0.05, 3.0);
            }
            self.prev = Some(err);
            if gs < 4.0 {
                return RolloutBrakes { sym: brake, left: 0.0, right: 0.0 };
            }
            self.trim = (self.trim + 0.025 * 0.8 * err * dt.clamp(0.05, 1.0)).clamp(-0.15, 0.15);
            let diff = (0.025 * (err + 1.2 * self.rate) + self.trim).clamp(-0.3, 0.3);
            let (sym, left, right) = crate::ground_steer::stop_wheels(brake, 0.0);
            let _ = (left, right);
            let half = diff.abs() / 2.0;
            let outside = (sym - half).max(0.0);
            let inside = (sym + half).min(1.0);
            if diff > 0.0 {
                RolloutBrakes { sym: outside, left: 0.0, right: inside }
            } else if diff < 0.0 {
                RolloutBrakes { sym: outside, left: inside, right: 0.0 }
            } else {
                RolloutBrakes { sym, left: 0.0, right: 0.0 }
            }
        }
    }

    /// The law a replay flies.
    #[derive(Clone, Copy, Debug)]
    enum Law {
        /// The shared split (1790689843 as flown; the SF50's today).
        Shared,
        /// 1790694457 as flown.
        AsFlown2,
        /// The free nose's one ground law on the identified SR22.
        Ground,
    }

    enum Running {
        Split(RolloutBrakeSteer),
        Flown(AsFlown2),
    }

    impl Running {
        fn new(law: Law) -> Self {
            match law {
                Law::Shared => Running::Split(RolloutBrakeSteer::default()),
                Law::AsFlown2 => Running::Flown(AsFlown2::default()),
                Law::Ground => Running::Split(RolloutBrakeSteer::free(GroundPlant::SR22)),
            }
        }

        fn step(&mut self, err: f64, psi: f64, trk: f64, dt: f64, gs: f64, brake: f64) -> RolloutBrakes {
            match self {
                Running::Split(l) => l.step_with(err, Some(psi), Some(trk), 0.0, dt, gs, brake),
                Running::Flown(l) => l.step(err, dt, gs, brake),
            }
        }
    }

    const FLOWN_1: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/landings/sr22_lemg13_rollout_1790689843.jsonl"));
    const FLOWN_2: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/landings/sr22_lemg13_rollout_1790694457.jsonl"));
    /// LEMG 13's true course, as in the 2026-09-28 record.
    const COURSE: f64 = 131.69;

    /// (s from the latch, the tick's dt, groundspeed, heading, cross-track m
    /// right, the stopping brake, the wheel difference as the plugin applied
    /// it, right positive).
    type Tick = (f64, f64, f64, f64, f64, f64, f64);

    fn ticks(text: &str) -> Vec<Tick> {
        let mut t = 0.0;
        let mut out = Vec::new();
        for (i, line) in text.lines().filter(|l| !l.trim().is_empty()).enumerate() {
            let r: serde_json::Value = serde_json::from_str(line).unwrap();
            let f = |k: &str| r[k].as_f64().unwrap();
            let dt = f("dt");
            if i > 0 {
                t += dt;
            }
            let (sym, bl, br) = (f("brake"), f("bl"), f("br"));
            let inside = bl.max(br);
            let stop = if inside > 0.0 {
                (sym + inside) / 2.0
            } else {
                sym
            };
            out.push((
                t,
                dt,
                f("gs"),
                f("psi"),
                f("xtk_m"),
                stop,
                sym.max(br) - sym.max(bl),
            ));
        }
        out
    }

    /// The wheels' plant: deg/s of yaw per unit of difference, the dead time
    /// and lag before it, the weathervane per second at 60 kt, and the rate
    /// the track follows the nose.
    #[derive(Clone, Copy)]
    struct Plant {
        a: f64,
        dead_s: f64,
        lag_s: f64,
        vane_60: f64,
        track_per_s: f64,
    }
    const FIT: Plant = Plant {
        a: 45.0,
        dead_s: 0.45,
        lag_s: 0.1,
        vane_60: 4.0,
        track_per_s: 2.5,
    };

    /// A flown rollout.
    struct Flown {
        text: &'static str,
        /// The wheels down to stay, s after the latch.
        touchdown_s: f64,
        /// The bank at the latch, degrees.
        phi0: f64,
    }
    const S1: Flown = Flown {
        text: FLOWN_1,
        touchdown_s: 6.3,
        phi0: 0.0,
    };
    const S2: Flown = Flown {
        text: FLOWN_2,
        touchdown_s: 4.4,
        phi0: -0.84,
    };

    /// The replay: `law` flown, the bounce handled or not. The worst
    /// cross-track at the ticks, and (s, heading, wheel difference) for each
    /// tick on the wheels.
    fn replay(f: &Flown, law_kind: Law, bounce_held: bool, p: Plant) -> (f64, Vec<(f64, f64, f64)>) {
        let rec = ticks(f.text);
        let n = rec.len();
        let end = rec[n - 1].0;
        // The recorded sideways velocity (central differences).
        let v_rec: Vec<f64> = (0..n)
            .map(|i| {
                let (a, b) = (i.saturating_sub(1), (i + 1).min(n - 1));
                (rec[b].4 - rec[a].4) / (rec[b].0 - rec[a].0)
            })
            .collect();
        let interp = |t: f64, col: &dyn Fn(usize) -> f64| -> f64 {
            if t <= rec[0].0 {
                return col(0);
            }
            for i in 1..n {
                if t <= rec[i].0 {
                    let fr = (t - rec[i - 1].0) / (rec[i].0 - rec[i - 1].0);
                    return col(i - 1) + fr * (col(i) - col(i - 1));
                }
            }
            col(n - 1)
        };
        let h = 0.01;
        let mut law = Running::new(law_kind);
        let mut on_wheels_seen = false;
        let (mut dpsi, mut dtrk, mut dx, mut dv, mut yaw) =
            (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
        let mut phi = f.phi0;
        let mut bank_cmds: Vec<(f64, f64)> = vec![(-10.0, f.phi0)];
        let mut written: Vec<(f64, f64)> = vec![(-10.0, 0.0)];
        let mut rate = crate::autotakeoff::XtkRate::default();
        let mut prev: Option<f64> = None;
        let (mut t, mut k) = (0.0, 0);
        let mut worst: f64 = 0.0;
        let mut ground = Vec::new();
        while t < end {
            let on = t >= f.touchdown_s;
            if k < n && t + 1e-9 >= rec[k].0 {
                let (tk, dt, gs, psi, xtk, stop, flown) = rec[k];
                let x = xtk + dx;
                let r = rate.update(prev.map_or(0.0, |q| (x - q) / dt), dt);
                prev = Some(x);
                worst = worst.max(x.abs());
                let wd = if on || !bounce_held {
                    if bounce_held && !on_wheels_seen {
                        law = Running::new(law_kind);
                        on_wheels_seen = true;
                    }
                    let target = crate::autotakeoff::roll_target_heading(COURSE, x, r, gs);
                    let err = crate::angles::signed_diff_deg(target, psi + dpsi);
                    // The track as the `hpath` read gives it: the recorded one
                    // (the course and the sideways speed) and the plant's own
                    // departure from it.
                    let trk = COURSE + (v_rec[k] / (gs * 0.514_444).max(1.0)).atan().to_degrees() + dtrk;
                    let b = law.step(err, psi + dpsi, trk, dt, gs, stop);
                    if std::env::var("RO_DEBUG").is_ok() && matches!(law_kind, Law::Ground) {
                        let (rr, w) = match &law {
                            Running::Split(l) => (l.last().map(|s| (s.r_ref, s.r_now)), l.trim()),
                            _ => (None, 0.0),
                        };
                        eprintln!("t {tk:5.2} gs {gs:5.1} x {x:6.2} r {r:5.2} psi {:7.2} trk {trk:7.2} err {err:6.2} diff {:5.2} rr {rr:?} w {w:.2}", psi + dpsi, b.sym.max(b.right) - b.sym.max(b.left));
                    }
                    if gs > 2.0 {
                        b.sym.max(b.right) - b.sym.max(b.left)
                    } else {
                        0.0
                    }
                } else {
                    let v = v_rec[k] + dv;
                    bank_cmds.push((tk, crate::pilot::line::bank_for_line(x, v, phi, 5.0, crate::pilot::line::period_s(1.0), 0.0)));
                    0.0
                };
                if on {
                    ground.push((tk, psi + dpsi, wd));
                }
                written.push((tk, wd - flown));
                k += 1;
            }
            let g = interp(t, &|i| rec[i].2);
            let v_ms = g * 0.514_444;
            if on {
                let arriving = written
                    .iter()
                    .take_while(|(tw, _)| *tw <= t - p.dead_s + 1e-9)
                    .last()
                    .map_or(0.0, |w| w.1);
                yaw += (p.a * arriving - yaw) * h / p.lag_s;
                let vane = p.vane_60 * (g / 60.0).powi(2);
                dpsi += (yaw + vane * (dtrk - dpsi)) * h;
                dtrk += p.track_per_s * (dpsi - dtrk) * h;
                dx += v_ms * dtrk.to_radians().sin() * h;
            } else if bounce_held {
                let cmd = bank_cmds
                    .iter()
                    .take_while(|(tc, _)| *tc <= t - 0.3 + 1e-9)
                    .last()
                    .map_or(f.phi0, |c| c.1);
                phi += (cmd - phi) * h / 0.8;
                dv += 9.81 * phi.to_radians().tan() * h;
                dx += dv * h;
                let vr = interp(t, &|i| v_rec[i]);
                let base = v_ms.max(1.0);
                dtrk = ((vr + dv).atan2(base) - vr.atan2(base)).to_degrees();
                dpsi = dtrk;
            }
            t += h;
        }
        (worst, ground)
    }

    /// The fitted plant and its neighbours.
    fn plants() -> Vec<Plant> {
        let mut out = Vec::new();
        for a in [40.0, 50.0] {
            for dead_s in [0.4, 0.55] {
                for vane_60 in [2.0, 6.0] {
                    for track_per_s in [2.0, 3.0] {
                        out.push(Plant {
                            a,
                            dead_s,
                            lag_s: 0.1,
                            vane_60,
                            track_per_s,
                        });
                    }
                }
            }
        }
        out
    }

    /// The span of the heading on the wheels once past the touchdown's
    /// first two seconds, degrees.
    fn heading_span(g: &[(f64, f64, f64)]) -> f64 {
        let rest = &g[4.min(g.len())..];
        let hi = rest.iter().map(|x| x.1).fold(f64::MIN, f64::max);
        let lo = rest.iter().map(|x| x.1).fold(f64::MAX, f64::min);
        hi - lo
    }

    /// As flown, each law reproduces its rollout (9.06 m, 10.97 m). With the
    /// bounce held, the free nose's one ground law (`ground_steer`, the
    /// identified SR22, no rollout numbers of its own) on this wheel plant
    /// (the #166 fit, not the law's): 1790689843 8.2 m at the fit and 9.1
    /// over the neighbouring wheel plants, WORSE than the retired declared
    /// split (4.8 and 5.2, its numbers fitted on this replay) and the shared
    /// law (7.3): its heading loop is sized for the 0.4 s loop delay the
    /// identified plant carries and follows the lateral's target 2 to 4
    /// degrees behind for 6 s (2026-09-30; the rollout is the manoeuvre to
    /// check in flight). 1790694457 inside ROLL_2_M, the drift the bounce
    /// made before the wheels (no law can touch it), where it flew 11.0 (the
    /// retired split 7.6).
    #[test]
    fn the_sr22_rollouts_replay_with_the_bounce_held() {
        let (flown_1, _) = replay(&S1, Law::Shared, false, FIT);
        assert!((flown_1 - 9.06).abs() < 0.1, "1790689843 as flown: {flown_1:.2}");
        let (flown_2, _) = replay(&S2, Law::AsFlown2, false, FIT);
        assert!((flown_2 - 10.97).abs() < 0.1, "1790694457 as flown: {flown_2:.2}");
        let (fit_1, _) = replay(&S1, Law::Ground, true, FIT);
        let (fit_2, _) = replay(&S2, Law::Ground, true, FIT);
        let (shared_1, _) = replay(&S1, Law::Shared, true, FIT);
        let worst_1 = plants().into_iter().map(|p| replay(&S1, Law::Ground, true, p).0).fold(0.0, f64::max);
        eprintln!("1790689843 {fit_1:.2} m (shared {shared_1:.2}, neighbours {worst_1:.2}), 1790694457 {fit_2:.2} m");
        assert!(fit_1 < ROLL_1_M, "1790689843 at the fit: {fit_1:.2} m");
        assert!(fit_2 < ROLL_2_M, "1790694457 at the fit: {fit_2:.2} m");
        assert!(worst_1 < ROLL_1_NEIGHBOURS_M, "1790689843 over the neighbours: {worst_1:.2} m");
    }
    const ROLL_1_M: f64 = 8.5;
    const ROLL_1_NEIGHBOURS_M: f64 = 9.5;
    const ROLL_2_M: f64 = 7.8;

    /// Below 35 kt the numbers 1790694457 flew banged the split from wheel
    /// to wheel (0.39 each tick in the air and on it) and swung the nose
    /// +/-7 degrees; replayed on its wheels the one ground law keeps the
    /// heading's span under two thirds of theirs on a wheel plant up to 75
    /// deg/s per unit and 0.6 s (3.4, 8.3, 11.5 degrees against 10.3, 13.6,
    /// 22.2; 2026-09-30).
    #[test]
    fn the_sr22_split_does_not_bang_wheel_to_wheel_slow() {
        for (a, dead_s) in [(45.0, 0.45), (60.0, 0.6), (75.0, 0.6)] {
            let p = Plant { a, dead_s, ..FIT };
            let (_, new) = replay(&S2, Law::Ground, true, p);
            let (_, old) = replay(&S2, Law::AsFlown2, true, p);
            let (sn, so) = (heading_span(&new), heading_span(&old));
            eprintln!("{a} {dead_s}: span {sn:.1} (old numbers {so:.1})");
            assert!(sn < 0.65 * so, "{a} deg/s per unit, {dead_s} s: {sn:.1} degrees against {so:.1}");
            assert!(so > sn + 2.0, "the old numbers swung {so:.1}, the new {sn:.1}");
        }
    }

    /// The bounce watch: on the wheels, off (a spell), in the air, down.
    #[test]
    fn a_bounce_is_a_spell_off_the_wheels() {
        let mut b = BounceHold::default();
        assert_eq!(b.tick(false), BounceTick::OnWheels);
        assert_eq!(b.tick(true), BounceTick::Left);
        assert_eq!(b.tick(true), BounceTick::InAir);
        assert_eq!(b.tick(false), BounceTick::Landed);
        assert_eq!(b.tick(false), BounceTick::OnWheels);
        assert_eq!(b.tick(true), BounceTick::Left);
        assert_eq!(b.spells(), 2);
    }

    /// The free law learns the standing yaw (its observer, the declared
    /// trim it replaces: in the model's own units, the crossover-matched
    /// gain's, so a third of a standing 3 deg/s^2 reads about 1.4) and holds
    /// the heading against it, and leaves the stop plain below 4 kt; the
    /// shared split learns nothing.
    #[test]
    fn the_free_split_learns_the_standing_yaw() {
        let mut law = RolloutBrakeSteer::free(GroundPlant::SR22);
        let mut psi = 131.0;
        let mut r = 0.0_f64;
        for _ in 0..300 {
            let err = 131.0 - psi;
            let b = law.step_with(err, Some(psi), None, 0.0, 0.46, 50.0, 0.22);
            let diff = b.sym.max(b.right) - b.sym.max(b.left);
            // The SR22 at 25.7 m/s with the idle propeller's right yaw.
            let p = GroundPlant::SR22;
            let v = 50.0 * 0.514_444;
            for _ in 0..46 {
                r += (p.kb(v) * diff - p.d(v) * r + 3.0) * 0.01;
                psi += r * 0.01;
            }
        }
        assert!((psi - 131.0).abs() < 0.5, "held: {psi:.2}");
        let (kb_m, _) = GroundPlant::SR22.model(50.0 * 0.514_444, 0.46);
        let want = 3.0 * kb_m / GroundPlant::SR22.kb(50.0 * 0.514_444);
        assert!((law.trim() - want).abs() < 0.3 * want, "learnt {:.2} against {want:.2}", law.trim());
        let mut shared = RolloutBrakeSteer::default();
        for _ in 0..200 {
            shared.step(-3.0, 0.46, 60.0, 0.22);
        }
        assert_eq!(shared.trim(), 0.0);
        let mut slow = RolloutBrakeSteer::free(GroundPlant::SR22);
        let b = slow.step_with(-3.0, Some(131.0), None, 0.0, 0.46, 3.0, 0.22);
        assert_eq!(b, RolloutBrakes { sym: 0.22, left: 0.0, right: 0.0 });
    }

    /// The 2026-09-28 rollout (#166, rudder alone as flown; that replay has
    /// no weathervane and the record's first seconds may be a bounce too)
    /// under the one ground law (a wheel plant of its own, not the law's):
    /// inside 6.5 m from 10 to 90 deg/s per unit (2.5 to 3.7 to 60, 6.0 at
    /// 90; the shared law 1.7 to 3.3 there, as flown 12.1).
    #[test]
    fn the_sr22_split_on_the_first_rollout() {
        for a in [10.0, 15.0, 20.0, 30.0, 45.0, 60.0, 90.0] {
            let w = super::lemg13_sr22_rollout_tests::roll_with(a, true, Some(GroundPlant::SR22));
            eprintln!("{a}: {w:.2} m");
            assert!(w < 6.5, "{a} deg/s per unit: {w:.2} m");
        }
    }
