    use super::{FlarePower, RetardKey};

    /// The 737 profile's approach speed (`reference.approach_kts`), the
    /// retard's speed key's reference.
    const APPROACH_IAS: f64 = 145.0;

    fn profile_737() -> crate::aircraft::AircraftProfile {
        crate::aircraft::load("boeing_737_800").expect("the 737 profile loads")
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

    /// Every autoland tick's bug (true) from four LEMG 13 approaches of
    /// 2026-09-28, the arm to the wheels (two 5 nm finals, one with an
    /// intercept, and a full-auto circuit's final).
    const LEMG_13_FINAL_KNOBS: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/landings/lemg13_final_knobs.jsonl"));

    /// Issue #93: those approaches' bugs replayed through the knob autoland
    /// now sets (`bug_knob`): the knob moved (0.2 degrees or more, the least
    /// `drive_heading_bug_step` writes) less often than the law's bug in
    /// every approach and two thirds as often in all, down to the arrest
    /// (the arrest and the flare still steer every tick), and always within
    /// a turn's steering of the law's own bug. On the final's quarter degree
    /// a second (2026-10-12; the last 0.3 nm no longer steered every tick):
    /// 47 moves against 93, where half a degree every two seconds outside
    /// the short final made 21 against 76.
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
                let short = k["arresting"] == true;
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
            assert!(held < free, "{s}: {held} moves against {free}");
            free_all += free;
            held_all += held;
        }
        assert!(held_all * 3 <= free_all * 2, "{held_all} moves against {free_all}");
    }
