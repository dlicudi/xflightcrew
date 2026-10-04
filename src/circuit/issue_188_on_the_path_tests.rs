    use super::*;

    // The armed record of sortie 1790697583-full_auto: a right-hand pattern
    // raised to 2,571 ft for terrain, a 922 ft platform and a 3.26 degree
    // path, radius 1,290 m.
    const COURSE: f64 = 131.685_387_32;
    const FIELD_FT: f64 = 48.0;
    const TPA_FT: f64 = 2571.13;
    const LEFT: bool = false;
    /// The SR22's pattern cap: no `enroute_descent_fpm`, the GA 700, so the
    /// reversal's 800.
    const SR22_CAP: f64 = 800.0;

    fn shape() -> DescentShape {
        DescentShape {
            platform_agl_ft: 921.96,
            gp_ft_per_nm: 3.257_647_f64.to_radians().tan() * 6076.12,
        }
    }

    fn geom() -> Geometry {
        Geometry {
            upwind_past_m: 645.0,
            width_m: 2938.24,
            downwind_past_m: 3871.13,
            // The capture the measured bank set (`final_turn` capture_m).
            final_capture_m: 1234.7,
            lead_m: 0.9 * 1290.05,
        }
    }

    /// A second-order VS plant through a dead time (the SR22's autopilot),
    /// a banked turn onto the bug, the speed held on the leg's want.
    struct Sr22 {
        t: f64,
        a_m: f64,
        x_m: f64,
        psi: f64,
        bank: f64,
        alt: f64,
        x1: f64,
        vs: f64,
        ias: f64,
        dial: f64,
        queue: VecDeque<(f64, f64)>,
    }

    impl Sr22 {
        fn set_dial(&mut self, fpm: f64) {
            self.queue.push_back((self.t + 0.8, fpm));
        }
        fn advance(&mut self, hdg: f64, want_kts: f64, secs: f64) {
            let dt = 0.05;
            let mut left = secs;
            while left > 1e-9 {
                while self.queue.front().is_some_and(|&(t, _)| t <= self.t) {
                    self.dial = self.queue.pop_front().map_or(self.dial, |(_, v)| v);
                }
                self.x1 += (self.dial - self.x1) * dt / 1.5;
                self.vs += (self.x1 - self.vs) * dt / 1.5;
                self.alt += self.vs / 60.0 * dt;
                let err = crate::angles::signed_diff_deg(hdg, self.psi);
                let want = (err * 1.5).clamp(-17.0, 17.0);
                self.bank += (want - self.bank).clamp(-5.0 * dt, 5.0 * dt);
                self.ias += (want_kts - self.ias) * dt / 10.0;
                let v = self.ias * 0.514_444;
                let rate = (9.81 * self.bank.to_radians().tan() / v).to_degrees();
                self.psi = (self.psi + rate * dt).rem_euclid(360.0);
                let rel = (self.psi - COURSE).to_radians();
                self.a_m += v * rel.cos() * dt;
                self.x_m += v * rel.sin() * dt;
                self.t += dt;
                left -= dt;
            }
        }
    }

    #[derive(Debug, Default)]
    struct Flown {
        /// Handed over: along (m, negative on the approach side), height over
        /// the path (ft).
        handoff: Option<(f64, f64)>,
        /// Flown around by the circuit for height, at this along.
        went_around: Option<f64>,
        base_turn_a_m: Option<f64>,
        min_agl_over_floor: f64,
        max_vs_down: f64,
    }

    /// From abeam the threshold as the sortie flew it (t+307: a_m 8, x_m
    /// 2,941, 2,449 ft still climbing at 274 fpm toward the 2,571 pattern,
    /// 102 kt), through the rest of the downwind, base and final to the
    /// handoff. `fixed`: the law as it flew (a standing 500 fpm authority, no
    /// extension, no height gate). `start_alt` overrides the height abeam.
    fn fly(fixed: bool, start_alt: f64, cap: f64) -> Flown {
        let g = geom();
        let s = shape();
        let mut p = Sr22 {
            t: 0.0,
            a_m: 8.0,
            x_m: 2941.0,
            psi: (COURSE + 180.0).rem_euclid(360.0),
            bank: 0.0,
            alt: start_alt,
            x1: 274.0,
            vs: 274.0,
            ias: 102.0,
            dial: 250.0,
            queue: VecDeque::new(),
        };
        let mut leg = Leg::Downwind;
        let mut turning_final = false;
        let mut descent_target = None;
        let (mut vs_est, mut prev_alt) = (p.vs, p.alt);
        let (mut closure, mut prev_along) = (0.0, p.a_m);
        let mut last_vs: Option<f64> = None;
        let mut out = Flown {
            min_agl_over_floor: f64::INFINITY,
            ..Default::default()
        };
        let tick = 0.5;
        while p.t < 900.0 {
            let above = above_path_ft(p.alt, FIELD_FT, p.a_m, s.gp_ft_per_nm);
            if leg == Leg::Final && !fixed && final_high_give_up(above, p.a_m) {
                out.went_around = Some(p.a_m);
                break;
            }
            let trk_err = crate::angles::signed_diff_deg(p.psi, COURSE);
            let judged = if fixed { None } else { above };
            if turning_final && ready_for_autoland(p.x_m, p.a_m.abs(), trk_err, p.ias, None, judged)
            {
                out.handoff = Some((p.a_m, above.unwrap()));
                break;
            }
            let hdg = match leg {
                Leg::Final => final_intercept_hdg(COURSE, p.x_m),
                _ => leg_heading(COURSE, leg, LEFT),
            };
            let target = leg_target_shaped(
                leg,
                p.a_m,
                FIELD_FT,
                TPA_FT,
                g.downwind_past_m,
                s,
                &mut descent_target,
            );
            vs_est = 0.6 * vs_est + 0.4 * (p.alt - prev_alt) / tick * 60.0;
            prev_alt = p.alt;
            let closing = (prev_along.abs() - p.a_m.abs()) / tick * 1.943_844;
            closure = (0.6 * closure + 0.4 * closing).clamp(0.0, 400.0);
            prev_along = p.a_m;
            let rate = if leg == Leg::Final {
                path_descent_fpm_on(closure, s.gp_ft_per_nm)
            } else {
                0.0
            };
            let extra = if fixed {
                ALT_MAX_EXTRA_DESCENT_FPM
            } else {
                leg_descent_authority_fpm(
                    leg, p.a_m, p.x_m, p.alt, FIELD_FT, TPA_FT, &g, LEFT, s, p.ias, cap, rate,
                )
            };
            let cmd = alt_capture_vs_on(target, rate, p.alt, vs_est, extra);
            if last_vs.is_none_or(|v| (v - cmd).abs() > 20.0) {
                p.set_dial(cmd);
                last_vs = Some(cmd);
            }
            if pattern_advance(leg, p.a_m, p.x_m, &g, 5000.0, LEFT) {
                let hold = !fixed
                    && leg == Leg::Downwind
                    && extend_downwind(
                        p.alt, FIELD_FT, TPA_FT, p.a_m, p.x_m, &g, LEFT, s, p.ias, cap,
                    );
                if !hold {
                    if let Some(n) = next_leg(leg) {
                        if n == Leg::Base {
                            out.base_turn_a_m = Some(p.a_m);
                        }
                        leg = n;
                        turning_final |= leg == Leg::Final;
                    }
                }
            }
            // The recorded speeds: 101 on the pattern, 91 on final.
            let want_kts = if leg == Leg::Final { 91.0 } else { 101.0 };
            p.advance(hdg, want_kts, tick);
            // Never under the path's own floor over the field (the base's
            // `live_floor_ft` guards the ground; the plant has none).
            if leg != Leg::Final {
                let gate = approach_leg_alt_on(
                    FIELD_FT,
                    TPA_FT,
                    p.a_m.abs().max(g.downwind_past_m),
                    s.gp_ft_per_nm,
                );
                out.min_agl_over_floor = out.min_agl_over_floor.min(p.alt - gate);
            }
            out.max_vs_down = out.max_vs_down.max(-p.vs);
        }
        out
    }

    /// The recorded sortie, both ways: the old law hands over hundreds of
    /// feet over the path (recorded 659 ft over ours at 3,007 m, 545 ft over
    /// autoland's, `late_go_around`); today's arrives on it.
    #[test]
    fn the_raised_lemg_pattern_arrives_on_the_path() {
        let old = fly(true, 2449.0, SR22_CAP);
        println!("as flown: {old:?}");
        let (a_old, above_old) = old.handoff.expect("the old law handed over");
        assert!(above_old > 400.0, "the plant reproduces the high handoff: {above_old:.0}");
        assert!(a_old.abs() > FINAL_HIGH_LAST_M);

        let new = fly(false, 2449.0, SR22_CAP);
        println!("now: {new:?}");
        assert_eq!(new.went_around, None);
        let (a, above) = new.handoff.expect("handed over");
        assert!(above <= HANDOFF_ABOVE_PATH_FT, "{above:.0} ft over the path");
        assert!(a.abs() > FINAL_HIGH_LAST_M, "handed over at {a:.0} m");
        // Within what the airframe does, never a dive.
        assert!(new.max_vs_down < SR22_CAP + 250.0, "{:.0} fpm", new.max_vs_down);
    }

    /// Higher than even the sized descent can lose (the 2,571 pattern
    /// reached before abeam, plus a floor-raised 300 ft): the downwind
    /// extends, bounded, and still arrives on the path or is flown around;
    /// never handed over high.
    #[test]
    fn too_high_for_the_base_extends_the_downwind() {
        let f = fly(false, 2900.0, SR22_CAP);
        println!("{f:?}");
        let base_at = f.base_turn_a_m.expect("turned base");
        assert!(
            base_at.abs() > geom().downwind_past_m,
            "extended: base at {base_at:.0}"
        );
        assert!(base_at.abs() <= geom().downwind_past_m + MAX_DOWNWIND_EXTEND_M + 100.0);
        match f.handoff {
            Some((_, above)) => assert!(above <= HANDOFF_ABOVE_PATH_FT, "{above:.0}"),
            None => assert!(f.went_around.is_some(), "neither handed over nor around"),
        }
    }

    /// An ordinary circuit is untouched: a 1,000 ft pattern turns base where
    /// drawn and flies the standing 500 fpm authority.
    #[test]
    fn an_ordinary_circuit_flies_as_before() {
        let g = Geometry::for_radius(1290.0);
        let s = DescentShape::default();
        let (field, tpa) = (48.0, 1048.0);
        // Abeam at pattern height, the platform 800 ft over the field.
        let a = descent_authority_fpm(
            1048.0 - approach_leg_alt_on(field, tpa, g.downwind_past_m, s.gp_ft_per_nm),
            to_rollout_m(Leg::Downwind, -1.0, 2838.0, &g, LEFT).unwrap(),
            100.0,
            SR22_CAP,
        );
        assert_eq!(a, ALT_MAX_EXTRA_DESCENT_FPM);
        assert!(!extend_downwind(
            848.0,
            field,
            tpa,
            -(g.downwind_past_m - g.lead_m),
            g.width_m,
            &g,
            LEFT,
            s,
            100.0,
            SR22_CAP
        ));
        // Level legs keep the standing authority.
        assert_eq!(
            leg_descent_authority_fpm(
                Leg::Crosswind, 2000.0, 1000.0, 1048.0, field, tpa, &g, LEFT, s, 100.0, SR22_CAP,
                0.0
            ),
            ALT_MAX_EXTRA_DESCENT_FPM
        );
    }

    /// The sized authority: the height over the track at the planned share,
    /// floored at the standing 500 and capped at the airframe's.
    #[test]
    fn the_descent_is_sized_on_the_height_to_lose() {
        // The recorded base: 2,070 ft at the turn, 771 at the gate, 2,951 m
        // across at 103 kt: 1,299 ft in 0.75 of 56 s is 1,860 fpm, capped.
        let a = descent_authority_fpm(2070.0 - 771.0, 2951.0, 103.0, SR22_CAP);
        assert_eq!(a, SR22_CAP);
        // A small loss keeps the standing authority.
        assert_eq!(descent_authority_fpm(100.0, 3000.0, 100.0, SR22_CAP), 500.0);
        // Between: sized.
        let mid = descent_authority_fpm(500.0, 3000.0, 100.0, 2000.0);
        assert!((mid - 686.0).abs() < 5.0, "{mid:.0}");
        // A jet's cap is its enroute rate, within 800 to 1,500.
        assert_eq!(pattern_descent_cap_fpm(2000.0), 1500.0);
        assert_eq!(pattern_descent_cap_fpm(700.0), 800.0);
        // Below the gate, nothing extra.
        assert_eq!(descent_authority_fpm(-50.0, 3000.0, 100.0, SR22_CAP), 500.0);
    }

    /// The handoff waits for the path; a mile out still high is flown around.
    #[test]
    fn the_handoff_waits_for_the_path() {
        // 1790697583's handoff: on the line (2 m, residual 0.26), 1.51 nm,
        // 659 ft over the circuit's path.
        assert!(ready_for_autoland(2.0, 2910.0, -4.6, 92.0, Some(15.055), None));
        assert!(!ready_for_autoland(2.0, 2910.0, -4.6, 92.0, Some(15.055), Some(659.0)));
        assert!(ready_for_autoland(2.0, 2910.0, -4.6, 92.0, Some(15.055), Some(150.0)));
        // Below the path is autoland's to level out.
        assert!(ready_for_autoland(2.0, 2910.0, -4.6, 92.0, Some(15.055), Some(-200.0)));
        // Nor does the last-ditch valve hand over high.
        assert!(!ready_for_autoland(100.0, 1200.0, 0.0, 92.0, None, Some(400.0)));
        assert!(final_high_give_up(Some(400.0), -1800.0));
        assert!(!final_high_give_up(Some(400.0), -2500.0), "still room");
        assert!(!final_high_give_up(Some(150.0), -1500.0), "on the path");
        assert!(!final_high_give_up(None, -1500.0), "no reading");
        let above = above_path_ft(1269.0, FIELD_FT, -3007.0, shape().gp_ft_per_nm).unwrap();
        assert!((above - 659.0).abs() < 10.0, "{above:.0}");
    }

    /// The crew's go-around says why.
    #[test]
    fn the_crews_go_around_says_why() {
        let line = go_around_line(crate::go_around::GoAroundBy::Crew);
        assert!(line.contains("Too high"), "{line}");
        assert_eq!(crate::go_around::GoAroundBy::Crew.label(), "crew");
        assert!(!line.contains('\u{2014}') && !line.contains('\u{2013}'));
    }
