    use super::*;

    /// A profile's circuit speeds and floors, as the tool registry builds
    /// them (`tools::mod`), with the configuration floor from its stall table.
    struct Airframe {
        speeds: LegSpeeds,
        vy: f64,
        climb_floor: f64,
        sched: Vec<(f64, f64)>,
        limits: Vec<(f64, Option<f64>)>,
        vs_clean: Option<f64>,
        vs0: Option<f64>,
        points: Vec<(f64, f64)>,
        takeoff_ratio: Option<f64>,
    }

    impl Airframe {
        fn load(id: &str) -> Option<Airframe> {
            let p = crate::aircraft::load(id)?;
            let r = p.reference.as_ref();
            let vs = p.vs_clean_kts.unwrap_or(50.0);
            let approach_ias = r.and_then(|r| r.approach_kts).unwrap_or_else(|| {
                crate::aircraft::fallback_approach_kts(r.and_then(|r| r.vs0_kts), vs)
            });
            let vr = r
                .and_then(|r| r.vr_kts)
                .unwrap_or_else(|| crate::aircraft::fallback_vr_kts(vs));
            let vy = r.and_then(|r| r.vy_kts).unwrap_or(vr + 15.0);
            let pattern_kts = pattern_speed_kts(vy, approach_ias, p.vs_clean_kts);
            Some(Airframe {
                speeds: LegSpeeds {
                    pattern_kts,
                    approach_kts: approach_leg_kts(pattern_kts, approach_ias),
                    approach_ias,
                },
                vy,
                climb_floor: climb_floor_kts(r.and_then(|r| r.vyse_kts), p.vs_clean_kts),
                sched: crate::aircraft::flap_schedule(p.flaps.as_ref()),
                limits: p
                    .flaps
                    .as_ref()
                    .map(|f| f.detents.iter().map(|d| (d.ratio, d.vfe_kts)).collect())
                    .unwrap_or_default(),
                vs_clean: p.vs_clean_kts,
                vs0: r.and_then(|r| r.vs0_kts),
                points: crate::aircraft::detent_stall_points(p.flaps.as_ref()),
                takeoff_ratio: p.flaps.as_ref().and_then(|f| f.takeoff_ratio),
            })
        }

        fn floor(&self, flap: f64) -> Option<f64> {
            self.vs_clean.map(|vs| {
                (crate::aircraft::SPEED_FLOOR_RATIO
                    * crate::aircraft::stall_speed_in_config(vs, self.vs0, &self.points, flap))
                .round()
            })
        }

        fn want(&self, leg: Leg, last: Option<f64>, cleaning: Option<f64>) -> f64 {
            leg_want_kts(
                leg,
                self.speeds,
                last,
                cleaning,
                &self.sched,
                &self.limits,
                |f| self.floor(f),
            )
        }

        fn climb_floor(&self, leg: Leg, cleaning: Option<f64>) -> f64 {
            leg_climb_floor_kts(leg, self.climb_floor, self.vs_clean, cleaning, |f| {
                self.floor(f)
            })
        }

        fn placard(&self, flap: f64) -> Option<f64> {
            self.limits
                .iter()
                .find(|(r, _)| (r - flap).abs() < 1e-6)
                .and_then(|(_, v)| *v)
        }
    }

    /// The recorded upwind end and crosswind, +375.6 to +420 s: CONF 1 held
    /// (the walk never stepped), `pat_kts` 227.5. The leg's one speed and
    /// the limiter's full-rate speed both sit under the 215 placard at every
    /// tick; the old full-rate speed (the pattern speed's, 220) did not, and
    /// read the recorded speeds as slow.
    #[test]
    fn the_lemg_a330_circuit_climbs_and_holds_under_conf_1s_placard() {
        let a = Airframe::load("airbus_a330_300").expect("the A330 profile");
        assert_eq!(a.speeds.pattern_kts, 227.5);
        let placard = a.placard(0.25).expect("CONF 1 placard");
        assert_eq!(placard, 215.0);
        let old = climb_full_rate_kts(a.speeds.pattern_kts, a.vy, a.climb_floor);
        assert_eq!(old, 220.0);
        // (s after the sortie start, leg, ias, recorded cmd_vs) as flown.
        let ticks = [
            (375.6, Leg::Upwind, 216.1, 652.0),
            (376.4, Leg::Crosswind, 215.5, 574.0),
            (377.8, Leg::Crosswind, 214.3, 404.0),
            (380.2, Leg::Crosswind, 211.4, 177.0),
            (380.9, Leg::Crosswind, 210.7, 140.0),
            (382.5, Leg::Crosswind, 209.9, 164.0),
            (386.3, Leg::Crosswind, 211.9, 474.0),
            (391.6, Leg::Crosswind, 223.3, 700.0),
            (395.2, Leg::Crosswind, 222.1, 700.0),
            (404.0, Leg::Crosswind, 213.2, 227.0),
            (415.7, Leg::Crosswind, 208.5, 108.0),
            (420.0, Leg::Crosswind, 210.8, 481.0),
        ];
        for (t, leg, ias, cmd_vs) in ticks {
            let want = a.want(leg, Some(0.25), Some(0.25));
            let floor = a.climb_floor(leg, Some(0.25));
            assert!(floor <= a.climb_floor, "+{t}");
            let full = climb_full_rate_kts(want, a.vy, floor);
            assert!(want < placard, "+{t}: want {want}");
            assert!(full <= want + 1e-9, "+{t}: full rate at {full}, over {want}");
            assert!(full < placard, "+{t}: full rate at {full}");
            // Wherever the recorded climb was cut, the one speed cuts less.
            let now = climb_limit_fpm(700.0, ias, 0.0, full, floor);
            let was = climb_limit_fpm(700.0, ias, 0.0, old, a.climb_floor);
            assert!(now >= was, "+{t}: {now:.0} under {was:.0}");
            if cmd_vs < 690.0 && ias < want {
                assert!(now > was + 50.0, "+{t}: {now:.0} vs {was:.0}");
            }
        }
    }

    /// Over every profile: for each detent a circuit can keep while it cleans
    /// up (the takeoff's and the go-around's half, and every one between
    /// them and clean), the leg's speed and the limiter's full-rate speed
    /// sit under THAT detent's placard.
    #[test]
    fn every_profiles_clean_up_climbs_under_the_kept_flaps_placard() {
        let mut checked = 0;
        for entry in std::fs::read_dir("profiles").expect("profiles dir") {
            let path = entry.unwrap().path();
            if path.extension().and_then(|e| e.to_str()) != Some("yaml") {
                continue;
            }
            let id = path.file_stem().unwrap().to_str().unwrap().to_string();
            let Some(a) = Airframe::load(&id) else {
                continue;
            };
            if a.limits.is_empty() || a.sched.is_empty() {
                continue;
            }
            let top = a.takeoff_ratio.unwrap_or(0.0).max(GO_AROUND_FLAP_RATIO);
            for &(flap, vfe) in &a.limits {
                let Some(vfe) = vfe else { continue };
                if flap <= 1e-6 || flap > top + 1e-6 {
                    continue;
                }
                for leg in [Leg::Upwind, Leg::Crosswind, Leg::Downwind] {
                    let want = a.want(leg, Some(flap), Some(flap));
                    let floor = a.climb_floor(leg, Some(flap));
                    let full = climb_full_rate_kts(want, a.vy, floor);
                    assert!(floor < want, "{id} flap {flap}: floor {floor} over {want}");
                    assert!(want <= vfe, "{id} flap {flap}: holds {want} over {vfe}");
                    assert!(full <= vfe, "{id} flap {flap}: full rate {full} over {vfe}");
                    checked += 1;
                }
            }
        }
        assert!(checked >= 9, "only {checked} detent-legs checked");
    }

    /// The 737 hands over clean (#150's path): no clean-up, and the leg's
    /// speed is the pattern speed, exactly as the throttle and the limiter
    /// were fed before.
    #[test]
    fn a_clean_handover_holds_the_pattern_speed_unchanged() {
        let a = Airframe::load("boeing_737_800").expect("the 737 profile");
        assert_eq!(takeoff_flaps_to_clean(Some(0.0), &a.sched), None);
        for leg in [Leg::Upwind, Leg::Crosswind, Leg::Downwind] {
            assert_eq!(a.want(leg, None, None), a.speeds.pattern_kts);
            assert_eq!(
                climb_full_rate_kts(a.want(leg, None, None), a.vy, a.climb_floor),
                climb_full_rate_kts(a.speeds.pattern_kts, a.vy, a.climb_floor)
            );
        }
        assert_eq!(a.want(Leg::Base, Some(0.5), None), a.speeds.approach_kts);
        for leg in [Leg::Upwind, Leg::Crosswind, Leg::Downwind, Leg::Base] {
            assert_eq!(a.climb_floor(leg, None), a.climb_floor);
        }
    }

    /// The recorded upwind, +355 to +364 s: CONF 2 handed over at 178 kt
    /// and `cmd_vs` 0 for ten seconds, the clean floor (201.25) over the
    /// CONF 2 placard (196). The kept flap's floor gives a climb back.
    #[test]
    fn the_a330s_conf_2_upwind_is_not_below_its_climb_floor() {
        let a = Airframe::load("airbus_a330_300").expect("the A330 profile");
        // 1.2 x the 175 kt clean stall (it was 1.15, 201.25: refactor audit #13).
        assert!((a.climb_floor - 210.0).abs() < 1e-9);
        let want = a.want(Leg::Upwind, Some(0.5), Some(0.5));
        let floor = a.climb_floor(Leg::Upwind, Some(0.5));
        assert!(floor < 170.0 && want <= 196.0, "floor {floor}, want {want}");
        let full = climb_full_rate_kts(want, a.vy, floor);
        for ias in [178.7, 179.3, 179.9] {
            let old = climb_full_rate_kts(a.speeds.pattern_kts, a.vy, a.climb_floor);
            let was = climb_limit_fpm(1500.0, ias, 0.0, old, a.climb_floor);
            let now = climb_limit_fpm(1500.0, ias, 0.0, full, floor);
            assert_eq!(was, 0.0);
            assert!(now > 500.0, "{ias}: {now:.0}");
        }
        // Base and final keep the clean floor as before.
        assert_eq!(a.climb_floor(Leg::Base, Some(0.5)), a.climb_floor);
        // A blue line does not move with the flaps.
        assert_eq!(climb_floor_in_config(101.0, Some(84.0), Some(80.0)), 101.0);
        assert_eq!(climb_floor_in_config(96.6, Some(84.0), None), 96.6);
    }

    #[test]
    fn detent_above_is_the_next_more_deployed() {
        let limits = [(0.0, None), (0.25, Some(215.0)), (0.5, Some(196.0)), (1.0, None)];
        assert_eq!(detent_above(0.25, &limits), Some(0.5));
        assert_eq!(detent_above(0.5, &limits), Some(1.0));
        assert_eq!(detent_above(1.0, &limits), None);
    }
