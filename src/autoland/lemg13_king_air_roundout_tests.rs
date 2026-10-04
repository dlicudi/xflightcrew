    use super::*;

    const SRC: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/landings/king_air_lemg13_roundouts.jsonl"));

    struct Tick {
        t: f64,
        radio: f64,
        vvi: f64,
        ias: f64,
        lever: f64,
        dist: f64,
        final_phase: bool,
    }

    struct Roundout {
        sortie: String,
        /// When the arrest took the servos.
        hands_t: f64,
        ticks: Vec<Tick>,
        /// (t, groundspeed kt, pitch trim) from the `state` samples.
        states: Vec<(f64, f64, f64)>,
    }

    impl Roundout {
        fn gs_at(&self, t: f64, ias: f64) -> f64 {
            self.states
                .iter()
                .take_while(|s| s.0 <= t)
                .last()
                .map_or(ias, |s| s.1)
        }
        /// The tick the arrest's release was written on.
        fn release(&self) -> &Tick {
            self.ticks.iter().find(|x| x.t >= self.hands_t).expect("a release tick")
        }
    }

    fn roundouts() -> Vec<Roundout> {
        let mut out: Vec<Roundout> = vec![];
        for line in SRC.lines().filter(|l| !l.trim().is_empty()) {
            let v: serde_json::Value = serde_json::from_str(line).expect("a record");
            let sortie = v["sortie"].as_str().expect("a sortie").to_string();
            if out.last().is_none_or(|r| r.sortie != sortie) {
                out.push(Roundout {
                    sortie: sortie.clone(),
                    hands_t: f64::NAN,
                    ticks: vec![],
                    states: vec![],
                });
            }
            let r = out.last_mut().unwrap();
            let t = v["t"].as_f64().unwrap();
            if v["event"] == "arrest_hands" {
                if r.hands_t.is_nan() {
                    r.hands_t = t;
                }
            } else if v["kind"] == "state" {
                r.states.push((
                    t,
                    v["gs"].as_f64().unwrap(),
                    v["trim"].as_f64().unwrap_or(f64::NAN),
                ));
            } else if let (Some(radio), Some(vvi), Some(ias), Some(lever)) = (
                v["radio"].as_f64(),
                v["vvi"].as_f64(),
                v["ias"].as_f64(),
                v["lever"].as_f64(),
            ) {
                r.ticks.push(Tick {
                    t,
                    radio,
                    vvi,
                    ias,
                    lever,
                    dist: v["dist"].as_f64().unwrap_or(f64::INFINITY),
                    final_phase: v["phase"] == "Final",
                });
            }
        }
        assert_eq!(out.len(), 13, "thirteen roundouts");
        assert!(out.iter().all(|r| r.hands_t.is_finite()));
        out
    }

    /// The first tick at or under 30 ft after the release: (radio, vvi), or
    /// `None` when the roundout levelled above it.
    fn at_30_ft(r: &Roundout) -> Option<(f64, f64)> {
        r.ticks
            .iter()
            .filter(|x| x.t >= r.hands_t)
            .take_while(|x| x.vvi < 0.0 || x.radio > 30.0)
            .find(|x| x.radio <= 30.0)
            .map(|x| (x.radio, x.vvi))
    }

    /// The evidence, pinned: the servos came out at 59 to 61 ft on the
    /// approach's own trim (-0.08 to -0.27; the approach swung it -0.12 to
    /// -0.40 against the lever), and what decided the roundout was the lever
    /// the speed law had raised in the seconds before. The three released
    /// at 0.47 to 0.50 came through 30 ft sinking 472 to 482 fpm (and landed
    /// at 29 to 69 fpm); the ten raised to 0.53 to 0.66 came through it at
    /// 251 to 396 fpm or levelled above it (five: two balloons, 641 fpm on
    /// the lever the float idled).
    #[test]
    fn the_raised_lever_decided_the_king_air_roundout() {
        let mut held = 0;
        let mut raised = 0;
        for r in roundouts() {
            let rel = r.release();
            assert!(
                (58.0..62.0).contains(&rel.radio),
                "{}: released at {:.1}",
                r.sortie,
                rel.radio
            );
            let trim = r
                .states
                .iter()
                .find(|s| s.0 >= r.hands_t)
                .map(|s| s.2)
                .expect("a trim after the release");
            assert!((-0.28..=-0.07).contains(&trim), "{}: trim {trim}", r.sortie);
            let at30 = at_30_ft(&r);
            if rel.lever <= 0.505 {
                held += 1;
                let (_, vvi) = at30.expect("an unraised lever comes through 30 ft");
                assert!(
                    vvi <= -470.0,
                    "{}: {vvi:.0} fpm at 30 ft on {:.3}",
                    r.sortie,
                    rel.lever
                );
            } else {
                raised += 1;
                assert!(rel.lever >= 0.53, "{}: {:.3}", r.sortie, rel.lever);
                if let Some((_, vvi)) = at30 {
                    assert!(
                        vvi > -400.0,
                        "{}: {vvi:.0} fpm at 30 ft on {:.3}",
                        r.sortie,
                        rel.lever
                    );
                }
            }
        }
        assert_eq!((held, raised), (3, 10));
    }

    /// The King Air's short final hold (`flare_power.hold_from_s`), flown
    /// open loop along each recorded final with the loop's own pieces
    /// (`short_final_power_held`, `throttle_adjust`, the flare's protected
    /// speed): it begins 5 to 7 s before the release (115 to 120 ft), never
    /// raises the lever after it, and every release is on 0.45 to 0.49, the
    /// three good landings' band, where ten were flown on 0.53 to 0.66.
    #[test]
    fn the_king_air_holds_its_short_final_lever() {
        let prof = crate::aircraft::load("king_air_c90b").expect("the King Air profile loads");
        let hold_from_s = prof.flare_power.and_then(|p| p.hold_from_s);
        assert!(hold_from_s.is_some(), "the King Air declares its short final hold");
        let reference = prof.reference.as_ref().expect("reference figures");
        let approach = reference.approach_kts.expect("an approach speed");
        let floor = reference
            .vs0_kts
            .map(|v| v * crate::aircraft::SPEED_FLOOR_RATIO);
        let protect = retard_reference_kts(approach, floor);
        assert!((95.0..100.0).contains(&protect), "protected {protect:.1}");
        for r in roundouts() {
            let mut lever: Option<f64> = None;
            let mut held_from: Option<(f64, f64)> = None;
            let mut prev: Option<&Tick> = None;
            for x in r.ticks.iter().take_while(|x| x.t < r.hands_t) {
                let trend = prev.map_or(0.0, |p| (x.ias - p.ias) / (x.t - p.t).clamp(0.1, 2.0));
                let phase = if x.final_phase {
                    ApproachPhase::Final
                } else {
                    ApproachPhase::Flare
                };
                let held = short_final_power_held(
                    phase,
                    x.dist,
                    x.radio,
                    r.gs_at(x.t, x.ias),
                    hold_from_s,
                    x.ias,
                    trend,
                    protect,
                );
                match (held_from, lever) {
                    (None, _) => {
                        // Flown as recorded until the hold begins.
                        lever = Some(x.lever);
                        if held {
                            held_from = Some((x.t, x.radio));
                        }
                    }
                    (Some(_), Some(l)) => {
                        assert!(held, "{}: the hold let go at {:.0} ft", r.sortie, x.radio);
                        let adj = crate::tools::speedhold::throttle_adjust(approach - x.ias, trend)
                            .min(0.0);
                        let next = (l + adj).clamp(IDLE_THROTTLE, 1.0);
                        assert!(next <= l, "{}: raised at {:.0} ft", r.sortie, x.radio);
                        lever = Some(next);
                    }
                    (Some(_), None) => unreachable!(),
                }
                prev = Some(x);
            }
            let (from_t, from_ft) = held_from.unwrap_or_else(|| panic!("{}: never held", r.sortie));
            let lead = r.hands_t - from_t;
            assert!(
                (5.0..7.0).contains(&lead),
                "{}: held {lead:.1} s before the release",
                r.sortie
            );
            assert!(
                (110.0..125.0).contains(&from_ft),
                "{}: held from {from_ft:.0} ft",
                r.sortie
            );
            let at_release = lever.unwrap();
            eprintln!(
                "{}: held from {from_ft:.0} ft, {lead:.1} s before; release on {at_release:.3}, flown {:.3}",
                r.sortie,
                r.release().lever
            );
            assert!(
                (0.45..0.49).contains(&at_release),
                "{}: {at_release:.3}",
                r.sortie
            );
        }
    }

    /// The hold's gates: declared, on final, near the runway, under its
    /// height at this groundspeed, and never against the protected speed.
    #[test]
    fn the_short_final_hold_is_asked_only_where_it_belongs() {
        let f = ApproachPhase::Final;
        let held = |phase, dist, radio, hold, ias, trend| {
            short_final_power_held(phase, dist, radio, 110.0, hold, ias, trend, 98.6)
        };
        // 110 kt: 9.7 ft/s of path sink, 12 s is 117 ft.
        assert!(held(f, 0.4, 110.0, Some(12.0), 108.0, -0.5));
        assert!(!held(f, 0.4, 125.0, Some(12.0), 108.0, -0.5));
        assert!(!held(f, 0.4, 110.0, None, 108.0, -0.5));
        assert!(!held(f, 1.5, 110.0, Some(12.0), 108.0, -0.5));
        assert!(!held(ApproachPhase::Intercept, 0.4, 110.0, Some(12.0), 108.0, -0.5));
        // Projected under the protected speed: the speed law may add.
        assert!(!held(f, 0.4, 110.0, Some(12.0), 101.0, -0.6));
        // A rising speed is never projected up past the protection.
        assert!(!held(f, 0.4, 110.0, Some(12.0), 98.0, 2.0));
        assert!(!held(f, 0.4, f64::NAN, Some(12.0), 108.0, 0.0));
    }

    /// Declared only where it was measured: no other profile holds its
    /// short final lever.
    #[test]
    fn only_the_king_air_declares_a_short_final_hold() {
        let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/profiles");
        for entry in std::fs::read_dir(dir).unwrap() {
            let path = entry.unwrap().path();
            if path.extension().and_then(|e| e.to_str()) != Some("yaml") {
                continue;
            }
            let id = path.file_stem().unwrap().to_str().unwrap().to_string();
            let Some(p) = crate::aircraft::load(&id) else {
                continue;
            };
            let declared = p.flare_power.and_then(|f| f.hold_from_s).is_some();
            assert_eq!(declared, id == "king_air_c90b", "{id}");
        }
    }
