    use super::*;
    use serde_json::Value;

    /// #181 (737 LEMG 13, sortie 1790649043): the servos-out give-up said
    /// "Your aircraft." with the aircraft as the records last saw it before
    /// the trip: 73 kt, under a 149.5 kt floor. That aircraft is never
    /// handed to nobody: the give-up finds it unflyable and recovers.
    #[test]
    fn the_servos_out_give_up_at_73_kt_is_a_recovery() {
        let src = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/sorties/1790649043-full_auto.trimmed.jsonl"));
        let rows: Vec<Value> = src.lines().filter_map(|l| serde_json::from_str(l).ok()).collect();
        let guard = rows
            .iter()
            .find(|r| r["kind"] == "circuit" && r["event"] == "pattern_guard")
            .expect("the recorded give-up");
        assert_eq!(guard["why"], "servos_off");
        let t = guard["t"].as_f64().unwrap();
        let last = rows
            .iter()
            .filter(|r| r["kind"] == "circuit" && r["event"] == "leg")
            .rfind(|r| r["t"].as_f64().unwrap() <= t)
            .unwrap();
        let ias = last["ias"].as_f64();
        let floor = last["climb_floor"].as_f64();
        assert!(ias.unwrap() < 75.0, "{ias:?}");
        assert_eq!(
            crate::tools::recover::unflyable(ias, floor, None, None, None),
            Some("slow")
        );
        // And the trip it would be now is a climb not held, never "servos".
        let trip = PatternTrip::ClimbNotHeld {
            why: crate::climb_watch::Why::NotFollowing,
            ias: 155.9,
            vs: 8066.0,
        };
        assert_eq!(trip.why(), "climb_not_held");
        assert!(trip.words().starts_with("The autopilot isn't following the climb."));
        assert!(!trip.words().contains('\u{2014}'));
    }

    const SORTIE: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/sorties/1790607325-full_auto.trimmed.jsonl"));
    /// The sortie's begin.
    const T0: f64 = 1790607325.1;
    /// As armed: LEMG 13, left hand, raised for terrain.
    const FIELD_FT: f64 = 48.0;
    const COURSE: f64 = 131.685_387_322_115_45;
    const RWY_LEN_M: f64 = 3200.0;
    /// The SF50's numbers as the profile gives them to the circuit.
    const VY: f64 = 120.0;
    const VAPP: f64 = 87.0;
    const VS_CLEAN: f64 = 80.0;

    fn recs() -> Vec<Value> {
        SORTIE
            .lines()
            .filter_map(|l| serde_json::from_str(l).ok())
            .collect()
    }

    fn is(r: &Value, kind: &str, event: &str) -> bool {
        r["kind"] == kind && r["event"] == event
    }

    fn num(r: &Value, k: &str) -> f64 {
        r[k].as_f64().unwrap_or(f64::NAN)
    }

    fn leg_of(r: &Value) -> Leg {
        match r["leg"].as_str().unwrap_or("") {
            "Upwind" => Leg::Upwind,
            "Crosswind" => Leg::Crosswind,
            "Downwind" => Leg::Downwind,
            "Base" => Leg::Base,
            _ => Leg::Final,
        }
    }

    fn armed() -> Value {
        recs()
            .into_iter()
            .find(|r| is(r, "circuit", "armed"))
            .unwrap()
    }

    fn pattern_kts() -> f64 {
        pattern_speed_kts(VY, VAPP, Some(VS_CLEAN))
    }

    /// The geometry the circuit armed, from the same functions.
    fn geometry() -> Geometry {
        let gs_est = VY * 0.514_444 * 1.1;
        Geometry::for_radii(
            turn_radius_m(gs_est),
            final_turn_radius_m(approach_leg_kts(pattern_kts(), VAPP)),
        )
    }

    #[test]
    fn the_armed_pattern_is_rebuilt_from_its_numbers() {
        let a = armed();
        let g = geometry();
        assert_eq!(pattern_kts(), 120.0);
        assert!((g.width_m - num(&a, "width_m")).abs() < 1.0, "{g:?} {a}");
        assert!((g.downwind_past_m - num(&a, "downwind_past_m")).abs() < 1.0);
        assert!((g.final_capture_m - num(&a, "radius_m")).abs() < 1.0);
        assert_eq!(a["left"], true);
        // The recorded legs changed where this geometry says they do.
        let legs: Vec<Value> = recs()
            .into_iter()
            .filter(|r| is(r, "circuit", "leg"))
            .collect();
        for w in legs.windows(2) {
            let (was, now) = (leg_of(&w[0]), leg_of(&w[1]));
            if was != now {
                assert!(
                    pattern_advance(
                        was,
                        num(&w[0], "a_m"),
                        num(&w[0], "x_m"),
                        &g,
                        RWY_LEN_M,
                        true
                    ),
                    "{was:?} -> {now:?} at {}",
                    w[0]
                );
            }
        }
        // Upwind 405 s, crosswind 405 s, downwind 669 s, at 120 kt, plus
        // the downwind's extension (#188), 60 s.
        let lim = |l| leg_limit_s(l, &g, RWY_LEN_M, pattern_kts()).round();
        assert_eq!(lim(Leg::Upwind), 348.0);
        assert_eq!(lim(Leg::Crosswind), 348.0);
        assert_eq!(lim(Leg::Downwind), 729.0);
    }

    /// Replayed through the guard, the recorded circuit is given back at
    /// 3,302 ft, 1,695 over its pattern on the crosswind, 46 s after the
    /// takeoff handed it over; it flew on to 21,882 ft for 14 more minutes. With the height guard blind the
    /// downwind is called overdue at +1,033 s (976 before the #188 extension),
    /// before the sortie's end.
    #[test]
    fn the_recorded_climb_away_is_given_back_at_the_pattern_margin() {
        let all = recs();
        let a = armed();
        let tpa = num(&a, "tpa_msl_ft");
        let g = geometry();
        let handoff = all
            .iter()
            .find(|r| is(r, "circuit", "takeoff_handoff"))
            .unwrap();
        let handoff_s = num(handoff, "t") - T0;
        let legs: Vec<&Value> = all.iter().filter(|r| is(r, "circuit", "leg")).collect();
        let fly = |blind: bool| {
            let mut guard = PatternGuard::default();
            for r in &legs {
                let (now, leg) = (num(r, "t") - T0, leg_of(r));
                let pattern = if blind {
                    f64::NAN
                } else {
                    tpa.max(num(r, "target_alt"))
                };
                let limit = leg_limit_s(leg, &g, RWY_LEN_M, pattern_kts());
                if let Some(trip) = guard.step(now, leg, num(r, "alt_msl"), pattern, limit, None) {
                    return Some((now, leg, num(r, "alt_msl"), trip));
                }
            }
            None
        };
        let (at, leg, alt, trip) = fly(false).expect("given back");
        println!(
            "tripped +{at:.1} s ({:.1} s after the handoff) on {leg:?} at {alt:.0} ft: {trip:?}",
            at - handoff_s
        );
        let PatternTrip::AbovePattern { above_ft } = trip else {
            panic!("{trip:?}")
        };
        assert!((1500.0..1750.0).contains(&above_ft), "{above_ft}");
        assert!(at - handoff_s < 50.0, "{}", at - handoff_s);
        assert!(alt < 3350.0, "{alt}");
        assert_eq!(leg, Leg::Crosswind);
        assert_eq!(
            trip.words(),
            "Circuit abandoned, 1700 feet above pattern height."
        );
        let top = all
            .iter()
            .filter(|r| r["kind"] == "state")
            .map(|r| num(r, "agl"))
            .fold(0.0, f64::max);
        assert!(top > 21_800.0, "the recording went on to {top:.0}");
        // The backstop: a downwind that never ends.
        let (at, leg, _, trip) = fly(true).expect("overdue");
        println!("blind: +{at:.1} s {leg:?} {trip:?}");
        assert_eq!(leg, Leg::Downwind);
        let PatternTrip::LegOverdue { secs, limit_s, .. } = trip else {
            panic!("{trip:?}")
        };
        assert!(secs > limit_s && secs < limit_s + 5.0, "{secs} {limit_s}");
        let end = all.iter().find(|r| is(r, "sortie", "end")).unwrap();
        assert!(at < num(end, "t") - T0, "inside the sortie");
        // +976 s before #188; the extension adds its 60 s.
        assert!((1010.0..1060.0).contains(&at), "{at}");
    }

    /// The servos out is the pilot's disconnect or a dropout: given back
    /// after [`SERVOS_OFF_S`]; a failed read starts and stops no clock; the
    /// height guard needs its margin held, not one frame of it.
    #[test]
    fn the_guard_needs_its_evidence_held() {
        let mut g = PatternGuard::default();
        let step = |g: &mut PatternGuard, t: f64, alt: f64, servos: Option<bool>| {
            g.step(t, Leg::Downwind, alt, 1600.0, 600.0, servos)
        };
        assert_eq!(step(&mut g, 0.0, 1600.0, Some(false)), None);
        assert_eq!(step(&mut g, 2.0, 1600.0, None), None);
        assert_eq!(step(&mut g, 2.9, 1600.0, Some(false)), None);
        assert!(matches!(
            step(&mut g, 3.0, 1600.0, Some(false)),
            Some(PatternTrip::ServosOff { .. })
        ));
        let mut g = PatternGuard::default();
        assert_eq!(step(&mut g, 0.0, 1600.0, Some(false)), None);
        assert_eq!(step(&mut g, 2.0, 1600.0, Some(true)), None);
        assert_eq!(
            step(&mut g, 4.0, 1600.0, Some(false)),
            None,
            "the clock restarted"
        );
        // One frame over the margin is no climb-away; a failed read neither
        // starts nor clears it.
        let mut g = PatternGuard::default();
        assert_eq!(step(&mut g, 0.0, 3200.0, Some(true)), None);
        assert_eq!(step(&mut g, 1.0, 1700.0, Some(true)), None);
        assert_eq!(step(&mut g, 2.0, 3200.0, Some(true)), None);
        assert_eq!(step(&mut g, 4.0, f64::NAN, Some(true)), None);
        assert!(matches!(
            step(&mut g, 5.0, 3200.0, Some(true)),
            Some(PatternTrip::AbovePattern { .. })
        ));
        // A leg's clock restarts on every leg, a go-around's upwind included.
        let mut g = PatternGuard::default();
        assert_eq!(g.step(0.0, Leg::Final, 800.0, 1600.0, 100.0, None), None);
        assert_eq!(g.step(90.0, Leg::Upwind, 800.0, 1600.0, 100.0, None), None);
        assert!(g
            .step(191.0, Leg::Upwind, 800.0, 1600.0, 100.0, None)
            .is_some());
    }

    /// The SF50 with its autopilot IN, fitted to its 2026-09-23 ETND circuit
    /// (VS: two lags of 1.5 s behind a dead time, 123 fpm rms over 463 s;
    /// the heading mode banks at most 27 degrees): the circuit's own laws
    /// through the plant, tick by tick as `fly` composes them.
    struct Sf50 {
        t: f64,
        a_m: f64,
        x_m: f64,
        psi: f64,
        bank: f64,
        alt: f64,
        x1: f64,
        vs: f64,
        ias: f64,
        dead_s: f64,
        dial: f64,
        queue: std::collections::VecDeque<(f64, f64)>,
    }

    const LAG1_S: f64 = 1.5;
    const LAG2_S: f64 = 1.5;
    const MAX_BANK: f64 = 27.0;

    impl Sf50 {
        fn set_dial(&mut self, fpm: f64) {
            self.queue.push_back((self.t + self.dead_s, fpm));
        }
        fn advance(&mut self, bug: f64, want_kts: f64, secs: f64) {
            let dt = 0.05;
            let mut left = secs;
            while left > 1e-9 {
                while self.queue.front().is_some_and(|&(t, _)| t <= self.t) {
                    self.dial = self.queue.pop_front().map_or(self.dial, |(_, v)| v);
                }
                self.x1 += (self.dial - self.x1) * dt / LAG1_S;
                self.vs += (self.x1 - self.vs) * dt / LAG2_S;
                self.alt += self.vs / 60.0 * dt;
                let err = crate::angles::signed_diff_deg(bug, self.psi);
                let want = (err * 1.5).clamp(-MAX_BANK, MAX_BANK);
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

    struct Flown {
        legs: Vec<(Leg, f64)>,
        max_alt: f64,
        downwind_alt: (f64, f64),
        handed_over: Option<f64>,
        turned_deg: f64,
        max_bank: f64,
        tripped: Option<PatternTrip>,
    }

    /// From this sortie's handoff (the first circuit tick's position, height,
    /// heading and speed; the climb flown, 2,778 fpm, and the dial the
    /// takeoff left, 1,500) to the handover onto final.
    fn fly_circuit(dead_s: f64) -> Flown {
        let all = recs();
        let a = armed();
        let tpa = num(&a, "tpa_msl_ft");
        let g = geometry();
        let first = all.iter().find(|r| is(r, "circuit", "leg")).unwrap();
        let climb = all
            .iter()
            .rev()
            .find(|r| is(r, "autotakeoff", "climb"))
            .unwrap();
        let handoff = all
            .iter()
            .find(|r| is(r, "circuit", "takeoff_handoff"))
            .unwrap();
        let mut p = Sf50 {
            t: num(first, "t") - T0,
            a_m: num(first, "a_m"),
            x_m: num(first, "x_m"),
            psi: num(first, "psi"),
            bank: 0.0,
            alt: num(first, "alt_msl"),
            x1: num(climb, "vs"),
            vs: num(climb, "vs"),
            ias: num(first, "ias"),
            dead_s,
            dial: num(handoff, "vs_cmd"),
            queue: Default::default(),
        };
        let kts = pattern_kts();
        let approach_kts = approach_leg_kts(kts, VAPP);
        let floor = climb_floor_kts(None, Some(VS_CLEAN));
        let mut leg = Leg::Upwind;
        let mut bug = COURSE;
        let mut knob = COURSE;
        let mut hold = BugHold::default();
        let mut carry = ClimbCarry::new(Some(num(handoff, "vs_cmd")));
        let mut descent_target = None;
        let mut guard = PatternGuard::default();
        let (mut vs_est, mut prev_alt) = (p.vs, p.alt);
        let (mut closure, mut prev_along) = (0.0, p.a_m);
        let mut last_vs: Option<f64> = None;
        let mut turning_final = false;
        let t0 = p.t;
        let mut out = Flown {
            legs: vec![(leg, 0.0)],
            max_alt: p.alt,
            downwind_alt: (f64::INFINITY, f64::NEG_INFINITY),
            handed_over: None,
            turned_deg: 0.0,
            max_bank: 0.0,
            tripped: None,
        };
        let tick = 0.6;
        let mut psi_was = p.psi;
        while p.t - t0 < 900.0 {
            let now = p.t - t0;
            let trk_err = crate::angles::signed_diff_deg(p.psi, COURSE);
            if turning_final && ready_for_autoland(p.x_m, p.a_m.abs(), trk_err, p.ias, None, None) {
                out.handed_over = Some(now);
                break;
            }
            let leg_hdg = match leg {
                Leg::Final => final_intercept_hdg(COURSE, p.x_m),
                _ => leg_heading(COURSE, leg, true),
            };
            bug = bug_toward(bug, leg_hdg, p.psi, p.bank, BUG_SLEW_PER_TICK);
            let db = if leg == Leg::Final {
                CIRCUIT_FINAL_DEADBAND
            } else {
                CIRCUIT_LEG_DEADBAND
            };
            if let Some(b) = hold.decide(leg as u32, bug, now, db) {
                knob = b.round();
            }
            if let Some(s) = hold.set() {
                bug = s;
            }
            let target = leg_target_ft(
                leg,
                p.a_m,
                FIELD_FT,
                tpa,
                g.downwind_past_m,
                &mut descent_target,
            );
            if let Some(trip) = guard.step(
                now,
                leg,
                p.alt,
                tpa.max(target),
                leg_limit_s(leg, &g, RWY_LEN_M, kts),
                Some(true),
            ) {
                out.tripped = Some(trip);
                break;
            }
            vs_est = 0.6 * vs_est + 0.4 * (p.alt - prev_alt) / tick * 60.0;
            prev_alt = p.alt;
            let closing = (prev_along.abs() - p.a_m.abs()) / tick * 1.943_844;
            closure = (0.6 * closure + 0.4 * closing).clamp(0.0, 400.0);
            prev_along = p.a_m;
            let rate = if leg == Leg::Final {
                path_descent_fpm(closure)
            } else {
                0.0
            };
            let want = alt_capture_vs(target, rate, p.alt, vs_est);
            let want_kts = if matches!(leg, Leg::Base | Leg::Final) {
                approach_kts
            } else {
                kts
            };
            let law = climb_limit_fpm(
                want,
                p.ias,
                0.0,
                climb_full_rate_kts(want_kts, VY, floor),
                floor,
            );
            let cmd = carry.step(
                law,
                tick,
                matches!(leg, Leg::Base | Leg::Final),
                p.ias <= floor,
            );
            if last_vs.is_none_or(|v| (v - cmd).abs() > 40.0) {
                p.set_dial(cmd);
                last_vs = Some(cmd);
            }
            if leg == Leg::Downwind && p.a_m >= 0.0 && now > 120.0 {
                out.downwind_alt.0 = out.downwind_alt.0.min(p.alt);
                out.downwind_alt.1 = out.downwind_alt.1.max(p.alt);
            }
            if pattern_advance(leg, p.a_m, p.x_m, &g, RWY_LEN_M, true) {
                if let Some(n) = next_leg(leg) {
                    leg = n;
                    out.legs.push((leg, now));
                    turning_final |= leg == Leg::Final;
                }
            }
            p.advance(knob, want_kts, tick);
            out.max_alt = out.max_alt.max(p.alt);
            out.max_bank = out.max_bank.max(p.bank.abs());
            out.turned_deg += crate::angles::abs_diff_deg(p.psi, psi_was);
            psi_was = p.psi;
        }
        out
    }

    /// With the autopilot flying, the same handoff levels at the pattern
    /// height, turns crosswind, downwind and base, and hands over on final:
    /// one lap of turning, never the ten the recording flew, through the
    /// fitted dead time and either side of it.
    #[test]
    fn with_the_servos_in_the_sf50_levels_and_turns_base() {
        let tpa = num(&armed(), "tpa_msl_ft");
        for dead in [1.0, 2.5, 4.0] {
            let f = fly_circuit(dead);
            println!(
                "dead {dead}: legs {:?}, max {:.0} ft (tpa {tpa:.0}), downwind {:.0}..{:.0}, \
                 handed over +{:?} s, turned {:.0} deg, bank {:.1}, tripped {:?}",
                f.legs,
                f.max_alt,
                f.downwind_alt.0,
                f.downwind_alt.1,
                f.handed_over,
                f.turned_deg,
                f.max_bank,
                f.tripped
            );
            assert_eq!(f.tripped, None, "dead {dead}");
            let names: Vec<Leg> = f.legs.iter().map(|(l, _)| *l).collect();
            assert_eq!(
                names,
                [
                    Leg::Upwind,
                    Leg::Crosswind,
                    Leg::Downwind,
                    Leg::Base,
                    Leg::Final
                ],
                "dead {dead}"
            );
            assert!(f.handed_over.is_some(), "dead {dead}");
            assert!(f.max_alt < tpa + 150.0, "dead {dead}: {:.0}", f.max_alt);
            assert!(
                (f.downwind_alt.0 - tpa).abs() < 100.0 && (f.downwind_alt.1 - tpa).abs() < 100.0,
                "dead {dead}: downwind {:?}",
                f.downwind_alt
            );
            assert!(f.turned_deg < 450.0, "dead {dead}: {:.0}", f.turned_deg);
            assert!(f.max_bank <= MAX_BANK + 0.1);
        }
    }

    /// LXGB 27, 1791723511: the second lap was as high as the first. Past
    /// MAX_HIGH_GO_AROUNDS the aircraft goes back, said with the count.
    #[test]
    fn high_finals_give_the_aircraft_back_after_two_laps() {
        assert_eq!(MAX_HIGH_GO_AROUNDS, 2);
        let trip = PatternTrip::HighFinals { laps: MAX_HIGH_GO_AROUNDS };
        assert_eq!(trip.why(), "high_finals");
        assert_eq!(
            trip.words(),
            "Circuit abandoned, still too high on final after two go arounds."
        );
        assert!(!trip.ap_has_no_pitch());
    }
