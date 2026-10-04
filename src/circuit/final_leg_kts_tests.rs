    use super::*;

    /// The 737's Vref 145 stands with flaps 40 out (floor 137) and is raised
    /// to 156 while the flaps are still at 5; a 172 (floor 58 under a 65 kt
    /// approach) never notices.
    #[test]
    fn the_final_leg_slows_only_as_far_as_the_flaps_allow() {
        assert_eq!(final_leg_kts(145.0, Some(137.0)), 145.0);
        assert_eq!(final_leg_kts(145.0, Some(156.0)), 156.0);
        assert_eq!(final_leg_kts(65.0, Some(58.0)), 65.0);
        assert_eq!(final_leg_kts(65.0, None), 65.0);
    }

    fn limits_of(f: Option<&crate::aircraft::FlapProfile>) -> Vec<(f64, Option<f64>)> {
        f.map(|f| f.detents.iter().map(|d| (d.ratio, d.vfe_kts)).collect())
            .unwrap_or_default()
    }

    /// LEMG 31 (sortie 1790584715-full_auto, 2026-09-28): the tower's order
    /// came on the circuit's Final at 736 ft AGL, 5,654 m out, 136.6 kt,
    /// flaps commanded full since 1790585512.0. The circuit flew on and
    /// landed. The order is the circuit's to fly on the approach legs.
    #[test]
    fn the_circuit_flies_a_go_around_order_on_its_approach_legs() {
        assert!(flies_go_around_order(Leg::Final), "the recorded leg");
        assert!(flies_go_around_order(Leg::Base));
        for leg in [Leg::Upwind, Leg::Crosswind, Leg::Downwind] {
            assert!(!flies_go_around_order(leg), "{leg:?}: no clearance to void");
        }
        // The go-around's half from full flap; a base at flaps 5 keeps 5.
        assert_eq!(go_around_flap(Some(1.0), &[]), GO_AROUND_FLAP_RATIO);
        assert_eq!(go_around_flap(Some(0.375), &[]), 0.375);
        assert_eq!(go_around_flap(None, &[]), 0.0);
        // A real detent: the MD-82's 15, where it has none at half travel.
        let md82 = crate::aircraft::load("md82").expect("the MD-82 profile");
        assert_eq!(
            go_around_flap(Some(1.0), &limits_of(md82.flaps.as_ref())),
            0.375
        );
    }

    /// The recorded go-around, cleaned up on the 737's own profile: flaps 10
    /// at the order, flown at 200 kt (Vfe 210 less the margin, past flaps
    /// one's 190), flaps one only once 10 has ARRIVED, then 215 kt (past the
    /// clean-up's 210) and up, then the pattern speed. The recorded 136.6 kt
    /// at the order takes no step.
    #[test]
    fn the_lemg_go_around_cleans_up_on_the_737s_schedule() {
        let p = crate::aircraft::load("boeing_737_800").expect("the 737 profile");
        let sched = crate::aircraft::flap_schedule(p.flaps.as_ref());
        let limits = limits_of(p.flaps.as_ref());
        let pattern = 210.0;
        let flap = go_around_flap(Some(1.0), &limits);
        assert_eq!(flap, 0.5);
        assert_eq!(
            go_around_flap_step(&sched, flap, Some(0.5), 136.6, 0.0),
            None
        );
        assert_eq!(
            go_around_speed_kts(pattern, flap, &sched, &limits, None),
            200.0
        );
        // Still travelling from 40: no step on the speed alone.
        assert_eq!(
            go_around_flap_step(&sched, flap, Some(0.8), 200.0, 0.0),
            None
        );
        assert_eq!(
            go_around_flap_step(&sched, flap, None, 200.0, 0.0),
            None,
            "no read, no step"
        );
        assert_eq!(
            go_around_flap_step(&sched, flap, Some(0.5), 200.0, 0.0),
            Some(0.125)
        );
        assert_eq!(
            go_around_speed_kts(pattern, 0.125, &sched, &limits, None),
            215.0
        );
        assert_eq!(
            go_around_flap_step(&sched, 0.125, Some(0.125), 205.0, 0.0),
            None
        );
        assert_eq!(
            go_around_flap_step(&sched, 0.125, Some(0.125), 215.0, 0.0),
            Some(0.0)
        );
        assert_eq!(
            go_around_speed_kts(pattern, 0.0, &sched, &limits, None),
            pattern
        );
        assert_eq!(
            go_around_flap_step(&sched, 0.0, Some(0.0), 250.0, 0.0),
            None
        );
        // A floor for the configuration is never flown under.
        assert_eq!(
            go_around_speed_kts(pattern, 0.5, &sched, &limits, Some(205.0)),
            205.0
        );
    }

    /// #169, LEMG 30 (sortie 1790698132-full_auto, 2026-09-29): "Flaps
    /// one." at 1,305 ft and the circuit wanted 212.5 kt, 2.5 under CONF 1's
    /// 215 placard (the old middle of the 210 retract to 215 gap), and the
    /// level-off ran 224. Now the placard margin wins: 205, and the step up
    /// is taken at 202. CONF 2 (196) keeps 186 and its step at 180, and the
    /// 737's schedule is untouched.
    #[test]
    fn the_a330_cleans_up_conf_1_inside_its_placard() {
        let p = crate::aircraft::load("airbus_a330_300").expect("the A330 profile");
        let sched = crate::aircraft::flap_schedule(p.flaps.as_ref());
        let limits = limits_of(p.flaps.as_ref());
        let pattern = 227.5;
        assert_eq!(go_around_speed_kts(pattern, 0.25, &sched, &limits, None), 205.0);
        let walk = cleanup_schedule(&sched, &limits, 0.25);
        assert_eq!(walk, vec![(0.0, 202.0), (0.25, 180.0)]);
        assert_eq!(go_around_flap_step(&walk, 0.25, Some(0.25), 201.9, 0.0), None);
        assert_eq!(go_around_flap_step(&walk, 0.25, Some(0.25), 202.0, 0.0), Some(0.0));
        // The recorded reading, past the handle: arrived, stepped at once.
        let mut pace = autotakeoff::FlapPace::default();
        pace.stepped(0.5);
        let judged = pace.judged(0.0, 0.25, Some(0.106));
        assert_eq!(go_around_flap_step(&walk, 0.25, judged, 212.1, 0.0), Some(0.0));
        // CONF 2 already fits.
        assert_eq!(go_around_speed_kts(pattern, 0.5, &sched, &limits, None), 186.0);
        assert_eq!(cleanup_schedule(&sched, &limits, 0.5), sched);
        // The leg's speed is the same number.
        let speeds = LegSpeeds {
            pattern_kts: pattern,
            approach_kts: 180.0,
            approach_ias: 150.0,
        };
        let want = leg_want_kts(
            Leg::Crosswind,
            speeds,
            Some(0.25),
            Some(0.25),
            &sched,
            &limits,
            |_| None,
        );
        assert_eq!(want, 205.0);
        // The 737 walks its own numbers from every detent.
        let b = crate::aircraft::load("boeing_737_800").expect("the 737 profile");
        let bs = crate::aircraft::flap_schedule(b.flaps.as_ref());
        let bl = limits_of(b.flaps.as_ref());
        for from in [0.125, 0.25, 0.375, 0.5] {
            assert_eq!(cleanup_schedule(&bs, &bl, from), bs, "737 from {from}");
        }
    }

    /// No airframe's go-around hovers with its flaps out: at every stage of
    /// the clean-up the speed it flies is inside the placard of what is out
    /// AND takes the next step, from the go-around's half to clean.
    #[test]
    fn every_profile_cleans_up_a_go_around_inside_its_placards() {
        let mut checked = 0;
        for entry in std::fs::read_dir("profiles").expect("profiles dir") {
            let path = entry.unwrap().path();
            if path.extension().and_then(|e| e.to_str()) != Some("yaml") {
                continue;
            }
            let id = path.file_stem().unwrap().to_str().unwrap().to_string();
            let Some(p) = crate::aircraft::load(&id) else {
                continue;
            };
            let limits = limits_of(p.flaps.as_ref());
            let sched = crate::aircraft::flap_schedule(p.flaps.as_ref());
            if limits.is_empty() || sched.is_empty() {
                continue;
            }
            // The worst case: a pattern speed that asks for nothing.
            let mut flap = go_around_flap(Some(1.0), &limits);
            let mut steps = 0;
            while flap > 1e-6 {
                let kts = go_around_speed_kts(0.0, flap, &sched, &limits, None);
                let placard = crate::autoland::flap_within_limits(flap, kts, &limits);
                assert!(
                    placard + 1e-6 >= flap,
                    "{id}: {kts} kt is over flap {flap}'s placard"
                );
                // ...and never inside the placard margin (#169).
                let vfe = placard_of(flap, &limits);
                assert!(
                    kts <= vfe - GO_AROUND_PLACARD_MARGIN_KTS + 1e-9,
                    "{id}: {kts} kt is within the margin of flap {flap}'s {vfe}"
                );
                // The walk steps on the schedule the circuit walks.
                let walk = cleanup_schedule(&sched, &limits, flap);
                let next = go_around_flap_step(&walk, flap, Some(flap), kts, 0.0);
                assert!(next.is_some(), "{id}: hovers at flap {flap}, {kts} kt");
                flap = next.unwrap();
                steps += 1;
                assert!(steps <= 10, "{id}: never cleans up");
            }
            checked += 1;
        }
        assert!(checked >= 3, "only {checked} profiles with a schedule");
    }
