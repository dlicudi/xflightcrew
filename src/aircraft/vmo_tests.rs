    use super::*;

    const CITX_HIGH: VmoHigh = VmoHigh { above_ft: 8_000.0, kts: 350.0, mmo: Some(0.935) };

    #[test]
    fn a_high_vmo_applies_at_and_above_its_altitude() {
        let low = Some(270.0);
        let high = Some(VmoHigh { mmo: None, ..CITX_HIGH });
        assert_eq!(vne_at(low, high, 5_000.0), Some(270.0));
        assert_eq!(vne_at(low, high, 7_999.0), Some(270.0));
        assert_eq!(vne_at(low, high, 8_000.0), Some(350.0), "a clean threshold");
        assert_eq!(vne_at(low, high, 20_000.0), Some(350.0));
        // An altitude not read is the low limit: conservative.
        assert_eq!(vne_at(low, high, f64::NAN), Some(270.0));
        // The governor's ceilings: 233 low, 305 high.
        assert_eq!(vne_at(low, high, 5_000.0).map(ceiling_from_vne), Some(233.0));
        assert_eq!(vne_at(low, high, 20_000.0).map(ceiling_from_vne), Some(305.0));
        // A nonsense declaration is no declaration.
        let bad = Some(VmoHigh { kts: -1.0, ..CITX_HIGH });
        assert_eq!(vne_at(low, bad, 20_000.0), Some(270.0));
    }

    #[test]
    fn a_high_vmo_gives_way_to_its_mmo_up_high() {
        let low = Some(270.0);
        let high = Some(CITX_HIGH);
        // M0.935 is 442 KIAS at FL200: the 350 holds.
        assert_eq!(vne_at(low, high, 20_000.0), Some(350.0));
        // At FL430 the Mmo is the limit, about 268 KIAS.
        let fl430 = vne_at(low, high, 43_000.0).unwrap();
        assert!((260.0..275.0).contains(&fl430), "{fl430}");
        // An insane Mmo is dropped, the Vmo kept.
        let silly = Some(VmoHigh { mmo: Some(9.0), ..CITX_HIGH });
        assert_eq!(vne_at(low, silly, 43_000.0), Some(350.0));
    }

    #[test]
    fn without_a_high_vmo_the_limit_is_one_number() {
        for alt in [0.0, 5_000.0, 8_000.0, 35_000.0, f64::NAN] {
            assert_eq!(vne_at(Some(340.0), None, alt), Some(340.0));
            assert_eq!(vne_at(None, None, alt), None);
        }
        // Every shipped profile but the Citation X declares none, and its
        // limit is its `vne_kts` at every altitude.
        let b738 = load("boeing_737_800").unwrap();
        assert!(b738.vmo_high.is_none());
        assert_eq!(b738.vne_at(37_000.0), b738.vne_kts);
        let mut c = VmoCeiling::default();
        for (alt, vs) in [(5_000.0, 2_000.0), (20_000.0, -3_000.0), (9_000.0, 0.0)] {
            assert_eq!(c.ceiling_for(Some(340.0), None, alt, vs), Some(ceiling_from_vne(340.0)));
        }
    }

    #[test]
    fn the_citation_x_declares_its_high_vmo() {
        let p = load("citation_x").unwrap();
        assert_eq!(p.vne_kts, Some(270.0), "the low limit");
        assert_eq!(p.vmo_high, Some(CITX_HIGH));
        assert_eq!(p.vne_at(5_000.0), Some(270.0));
        assert_eq!(p.vne_at(20_000.0), Some(350.0));
        // The profile's cruise sits under the high ceiling.
        let cruise = p.reference.as_ref().and_then(|r| r.cruise_ias_kts).unwrap();
        assert!(cruise <= ceiling_from_vne(350.0), "{cruise}");
    }

    #[test]
    fn a_descent_slows_for_the_low_vmo_before_it_gets_there() {
        let (low, high) = (Some(270.0), Some(CITX_HIGH));
        let mut c = VmoCeiling::default();
        // Cruise at FL250: the high ceiling.
        assert_eq!(c.ceiling_for(low, high, 25_000.0, 0.0), Some(305.0));
        // Descending at 2,500 fpm: 90 s ahead is 3,750 ft lower, so the low
        // ceiling comes at 11,750 ft, well before 8,000.
        assert_eq!(c.ceiling_for(low, high, 12_000.0, -2_500.0), Some(305.0));
        assert_eq!(c.ceiling_for(low, high, 11_700.0, -2_500.0), Some(233.0));
        // The pitch shallows to bleed the speed: the ceiling HOLDS low, so
        // the speed is not given straight back.
        assert_eq!(c.ceiling_for(low, high, 11_000.0, 0.0), Some(233.0));
        assert_eq!(c.ceiling_for(low, high, 11_000.0, f64::NAN), Some(233.0));
        assert_eq!(c.ceiling_for(low, high, f64::NAN, 0.0), Some(233.0), "a lost read keeps it");
        assert_eq!(c.low(), Some(true));
        // A climb back over the altitude lifts it.
        assert_eq!(c.ceiling_for(low, high, 7_500.0, 2_000.0), Some(233.0));
        assert_eq!(c.ceiling_for(low, high, 8_100.0, 2_000.0), Some(305.0));
        // Nothing measured yet: the low ceiling stands.
        let mut fresh = VmoCeiling::default();
        assert_eq!(fresh.ceiling_for(low, high, f64::NAN, 0.0), Some(233.0));
        // Up high the Mmo's airspeed is the ceiling.
        let fl430 = fresh.ceiling_for(low, high, 43_000.0, 0.0).unwrap();
        assert!(fl430 < 240.0, "{fl430}");
    }
