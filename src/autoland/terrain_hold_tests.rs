    use super::*;

    /// #182: the live floor rides every leg but the final and the go-around's
    /// own climb; the level legs climb clear of ground ahead, the base and
    /// an intercept keep the floor itself. LEMG 13 (sortie
    /// 1790668215-full_auto, t+1795): 2,565 ft over 2,043 on the downwind
    /// was 522 ft, a hair outside the 500 ft trip (the corridor plan is the
    /// fix, the floor the backstop); 25 ft lower it climbs to 2,843.
    #[test]
    fn the_live_floor_rides_every_leg_but_the_final() {
        use crate::circuit::Leg as C;
        use crate::reversal::Leg as R;
        let i = ApproachPhase::Intercept;
        assert_eq!(
            live_floor_leg(i, None, Some(R::Outbound), false),
            Some(C::Downwind)
        );
        assert_eq!(live_floor_leg(i, None, Some(R::Base), false), Some(C::Base));
        assert_eq!(
            live_floor_leg(i, None, Some(R::Inbound), false),
            Some(C::Downwind)
        );
        assert_eq!(live_floor_leg(i, None, None, true), Some(C::Downwind));
        assert_eq!(
            live_floor_leg(i, Some(MissedStage::Reposition), None, false),
            Some(C::Downwind)
        );
        assert_eq!(live_floor_leg(i, None, None, false), Some(C::Base));
        assert_eq!(
            live_floor_leg(i, Some(MissedStage::Climb), None, false),
            None
        );
        assert_eq!(
            live_floor_leg(ApproachPhase::Final, None, None, false),
            None
        );
        assert_eq!(
            live_floor_leg(ApproachPhase::Flare, None, None, false),
            None
        );
        assert_eq!(
            crate::circuit::live_floor_ft(C::Downwind, 2564.7, 2043.0),
            None
        );
        let floor = crate::circuit::live_floor_ft(C::Downwind, 2540.0, 2043.0);
        assert_eq!(floor, Some(2843.0));
    }
