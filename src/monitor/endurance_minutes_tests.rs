    use super::endurance_minutes;

    /// The 737 at takeoff thrust: 6,672 kg at 2.5 kg/s is 44 minutes by the
    /// live flow and 160 by the cruise burn the profile declares; the
    /// estimate takes the cruise figure whatever the flow (an idle descent's
    /// low flow lifted it out of the caution band and back, a line each
    /// time, LEMG 2026-10-07). No burn declared: the live flow.
    #[test]
    fn the_estimate_never_assumes_takeoff_thrust_for_the_whole_flight() {
        let takeoff = endurance_minutes(Some(6672.0), Some(2.5), Some(2500.0)).unwrap();
        assert!((takeoff - 160.1).abs() < 0.5, "{takeoff}");
        let idle = endurance_minutes(Some(6672.0), Some(0.3), Some(2500.0)).unwrap();
        assert!((idle - 160.1).abs() < 0.5, "{idle}");
        let raw = endurance_minutes(Some(6672.0), Some(2.5), None).unwrap();
        assert!((raw - 44.5).abs() < 0.5, "{raw}");
        assert_eq!(endurance_minutes(Some(6672.0), Some(0.0), None), None);
        assert_eq!(endurance_minutes(None, Some(1.0), Some(2500.0)), None);
    }
