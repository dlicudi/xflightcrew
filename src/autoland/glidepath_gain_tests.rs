    use super::glidepath_gain_fpm_per_ft;

    /// The glidepath correction closes an error over a fixed distance of
    /// final: the 172's gain stands, a 737 at 170 kt gets 2.6x it, and a
    /// 50 ft-high 737 over the numbers is answered with a real descent
    /// (LFLL 35L, 2026-09-17: 86 fpm was the whole correction).
    #[test]
    fn glidepath_gain_scales_with_groundspeed() {
        assert_eq!(glidepath_gain_fpm_per_ft(65.0), 1.2);
        assert_eq!(
            glidepath_gain_fpm_per_ft(40.0),
            1.2,
            "slow airframes keep the old law"
        );
        let jet = glidepath_gain_fpm_per_ft(170.0);
        // The 172's gain scaled by the groundspeed ratio, 2.6x it.
        assert!((jet - 1.2 * 170.0 / 65.0).abs() < 0.02, "{jet}");
        assert!(
            (glidepath_gain_fpm_per_ft(400.0) - 3.6).abs() < 1e-9,
            "capped"
        );
        assert!(
            50.0 * jet > 150.0,
            "50 ft high on a jet earns more than 150 fpm"
        );
    }
