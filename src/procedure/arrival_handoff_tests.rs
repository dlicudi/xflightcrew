    use super::*;

    #[test]
    fn the_egnx_arrival_slows_to_210_before_the_end_of_the_route() {
        // EGBB-EGNX (2026-09-30): 250 kt at 3,000 ft, about 260 over the
        // ground, the 737's clean approach speed 210. The slow-down needs
        // 40 kt at 0.5 kt/s: 5.8 nm, plus the 2 nm lead.
        let mut begun = false;
        assert_eq!(arrival_handoff_cap(12.0, 250.0, 260.0, 210.0, &mut begun), None);
        assert!(!begun);
        assert_eq!(arrival_handoff_cap(7.7, 250.0, 260.0, 210.0, &mut begun), Some(210.0));
        // Held once begun, whatever the speed and distance say next.
        assert_eq!(arrival_handoff_cap(9.0, 215.0, 220.0, 210.0, &mut begun), Some(210.0));
        // Already slow: only the lead before the end.
        let mut slow = false;
        assert_eq!(arrival_handoff_cap(3.0, 200.0, 200.0, 210.0, &mut slow), None);
        assert_eq!(arrival_handoff_cap(1.9, 200.0, 200.0, 210.0, &mut slow), Some(210.0));
    }

    #[test]
    fn the_handoff_speed_rides_an_enroute_plan_only() {
        assert_eq!(arrival_handoff_for(Some(210.25), &EndAction::Enroute), Some(210.25));
        assert_eq!(arrival_handoff_for(Some(f64::NAN), &EndAction::Enroute), None);
        assert_eq!(arrival_handoff_for(Some(0.0), &EndAction::Enroute), None);
        // A plan launched without one (every caller but full auto's
        // enroute) gets none.
        assert_eq!(arrival_handoff_for(None, &EndAction::Enroute), None);
        assert_eq!(
            arrival_handoff_for(Some(210.0), &EndAction::Level),
            None
        );
    }

    fn b738() -> crate::approach_energy::EnergyModel {
        let p = crate::aircraft::load("boeing_737_800").expect("737 profile");
        crate::approach_energy::EnergyModel::from_profile(&p, 3.0).expect("the 737 declares its deceleration")
    }

    fn fix_leg(ident: &str, lat: f64, lon: f64, alt: Option<f64>) -> ProcLeg {
        ProcLeg {
            seq: 0,
            transition: String::new(),
            leg_type: "TF".into(),
            fix_ident: ident.into(),
            fix_region: String::new(),
            fix: Some(crate::nav::Fix { ident: ident.into(), lat, lon }),
            course_mag: None,
            dist_nm: None,
            turn: None,
            alt_desc: ' ',
            alt1_ft: alt,
            alt2_ft: None,
            is_faf: false,
            is_map: false,
            vpa_deg: None,
            navaid_ident: String::new(),
            navaid_region: String::new(),
            navaid: None,
            path: Default::default(),
        }
    }

    /// The FAF to the threshold along the legs: the first runway leg after
    /// the FAF, none without one.
    #[test]
    fn the_faf_is_measured_to_the_runway_leg_after_it() {
        // One minute of latitude is one nautical mile.
        let legs = vec![
            fix_leg("CF26L", 51.0, 0.0, Some(3_000.0)),
            fix_leg("FF26L", 51.0 - 5.8 / 60.0, 0.0, Some(2_000.0)),
            fix_leg("RW26L", 51.0 - 11.6 / 60.0, 0.0, None),
        ];
        let d = faf_to_threshold_nm(&legs, 1).unwrap();
        assert!((d - 5.8).abs() < 0.05, "{d}");
        assert_eq!(faf_to_threshold_nm(&legs[..2], 1), None);
    }

    /// The arrival to the FAF's capture radius as the follower flies it:
    /// the governor's target `faf_speed_cap` under the clean approach
    /// speed, the flap `faf_flap` puts out; idle when over the target, the
    /// lever spooling down 4 s first; the descent given way while the speed
    /// is over (speed first, `speed_protected_vs`), gravity on the path
    /// flown. Returns (ias, alt, flap, gear, miles out the slow-down began).
    fn fly_to_faf(
        m: &crate::approach_energy::EnergyModel,
        faf: FafEnergy,
        mut to_faf: f64,
        mut alt: f64,
        faf_alt: f64,
    ) -> (f64, f64, f64, bool, Option<f64>) {
        use crate::approach_energy::{gear_with_flap, G_KTS_PER_S};
        let (descent_fpm, clean_kts, dt) = (2_000.0, 210.25, 0.5);
        let (mut ias, mut flap, mut begun, mut gear, mut since_cut) = (clean_kts, 0.0_f64, false, false, 0.0_f64);
        let mut began_at = None;
        while to_faf > CAPTURE_NM {
            let target = faf_speed_cap(m, faf, to_faf, ias, ias, alt - faf_alt, descent_fpm, &mut begun)
                .map_or(clean_kts, |c| c.min(clean_kts));
            if begun && began_at.is_none() {
                began_at = Some(to_faf);
            }
            if begun {
                flap = faf_flap(m, ias, faf.join_kts, flap);
                gear |= gear_with_flap(flap);
            }
            let vs = if alt > faf_alt + 1.0 {
                crate::procedure::speed_protected_vs(-descent_fpm, Some(ias - target), 0.0)
            } else {
                0.0
            };
            // Speed gained on the descent, kt/s.
            let gravity = G_KTS_PER_S * -vs / (ias * 6076.12 / 60.0);
            if ias > target + 0.5 {
                since_cut += dt;
                let idle = if since_cut > 4.0 { m.level_decel(flap) } else { 0.0 };
                ias = (ias + (gravity - idle) * dt).max(target);
            } else {
                // The governor holds it, adding power or easing to idle.
                since_cut = 0.0;
                ias = if gravity > m.level_decel(flap) { ias + (gravity - m.level_decel(flap)) * dt } else { target };
            }
            alt = (alt + vs * dt / 60.0).max(faf_alt);
            to_faf -= ias * dt / 3600.0;
        }
        (ias, alt, flap, gear, began_at)
    }

    /// EGKK 26L, 737 (2026-10-14, sortie 1791979030-full_auto): FF26L at
    /// 2,000 ft, 5.1 nm from the threshold (the handoff 0.8 nm before it
    /// read 5.93), the gate speed 145. The follower flew 210 kt flaps up
    /// from CF26L, 13.6 nm out at 6,400 ft (31 nm of pattern to fly to it),
    /// to the handoff, from where the energy model predicts 215 at the
    /// 1,000 ft gate (autoland's own `e_gate`, from its first tick). Planned
    /// back from the gate, the FAF is crossed no faster than the join speed
    /// with the flap for it out, and the gate is made.
    #[test]
    fn the_egkk_26l_approach_crosses_its_faf_at_a_speed_the_final_absorbs() {
        use crate::approach_energy::predicted_gate_kts;
        let m = b738();
        let vapp = 145.0;
        let faf = faf_energy(&m, vapp, 5.1, crate::stable_approach::GateKind::Instrument.gate_ft());
        // Autoland's join from the handoff was 168 kt (`e_join`, its path
        // 2.48 nm from below the beam); from the FAF a little slower.
        assert!((155.0..=168.5).contains(&faf.join_kts), "{faf:?}");
        assert!((1.9..2.5).contains(&faf.path_nm), "{faf:?}");
        // What was flown loses the gate.
        let flown = predicted_gate_kts(&m, vapp, 210.25, faf.path_nm, faf.path_nm);
        assert!(flown > 200.0, "the recorded handoff: {flown}");

        // The pattern as flown: 31 nm round to CF26L, 5.7 more to FF26L.
        let (ias, alt, flap, gear, began) = fly_to_faf(&m, faf, 37.0, 6_400.0, 2_000.0);
        let began = began.expect("the slow-down began");
        assert!(began < 20.0, "not before it was needed: {began:.1} nm");
        assert!(ias <= faf.join_kts + 1.0, "at the handoff {ias:.1} kt against {:.1}", faf.join_kts);
        assert!(alt <= 2_000.0 + 50.0, "at the FAF's altitude: {alt:.0}");
        assert!(flap >= m.flap_for(faf.join_kts) - 1e-9 && flap > 0.0, "configured: {flap}");
        assert!(gear, "the gear with the half flap");
        let gate = predicted_gate_kts(&m, vapp, ias, faf.path_nm, faf.path_nm);
        assert!(gate <= vapp + 1.0, "the gate is made: {gate:.1}");

        // Straight in from 10 nm at 6,400 ft the height cannot all come off
        // at idle with the flap out (about 1,250 ft is left), but the speed
        // still comes first: the FAF at the join speed, within the
        // speed-protected descent's own give (2 kt over as it still
        // descends), configured, the gate made on speed.
        let (ias, _, flap, _, began) = fly_to_faf(&m, faf, 10.0, 6_400.0, 2_000.0);
        assert_eq!(began, Some(10.0), "slowing at once");
        assert!(
            ias <= faf.join_kts + SPEED_PROTECT_SPAN_KTS / 2.0,
            "{ias:.1} against {:.1}",
            faf.join_kts
        );
        assert!(flap >= m.flap_for(faf.join_kts) - 1e-9, "configured: {flap}");
        assert!(predicted_gate_kts(&m, vapp, ias, faf.path_nm, faf.path_nm) <= vapp + 1.0);
    }


    /// The slow-down starts no earlier than it needs: level at the FAF's
    /// altitude and already slow, only the settling lead and the capture.
    #[test]
    fn a_slow_level_arrival_begins_only_inside_its_lead() {
        let m = b738();
        let faf = faf_energy(&m, 145.0, 5.8, 1_000.0);
        let mut begun = false;
        assert_eq!(faf_speed_cap(&m, faf, 6.0, faf.join_kts, 170.0, 0.0, 2_000.0, &mut begun), None);
        assert!(faf_speed_cap(&m, faf, 2.0, faf.join_kts, 170.0, 0.0, 2_000.0, &mut begun).is_some());
        // Held once begun.
        assert!(faf_speed_cap(&m, faf, 9.0, 200.0, 200.0, 3_000.0, 2_000.0, &mut begun).is_some());
        // Never retracts what is out.
        assert_eq!(faf_flap(&m, 210.0, faf.join_kts, 0.625), 0.625);
    }
