    use super::*;
    use crate::reversal::{hold_altitude_ft, TerrainHold, HOLD_LEVEL_TICKS};

    fn no_dashes(line: &str) {
        assert!(
            !line.contains('-') && !line.contains('\u{2013}') && !line.contains('\u{2014}'),
            "{line}"
        );
    }

    /// The refusal in the air climbs straight ahead to 1,000 ft over the
    /// ground ahead, never lower than the aircraft or the pattern, and says
    /// so with the numbers; the corridors' highest (the Rock) stands in
    /// when the probe ahead does not answer. On the runway the same reason
    /// is a plain refusal.
    #[test]
    fn a_rock_refusal_in_the_air_climbs_clear_and_says_the_numbers() {
        assert!(authored_refusal_requires_hold("LXGB"));
        assert!(!authored_refusal_requires_hold("LEMG"));
        // Ahead probed: the ridge's 1,098 plus 1,000.
        assert_eq!(
            hold_altitude_ft(Some(1098.0), Some(1398.0), 1130.0, 1650.0),
            2100.0
        );
        // Unanswered: the Rock's 1,398 plus 1,000.
        let hold = hold_altitude_ft(None, Some(1398.0), 1130.0, 1650.0);
        assert_eq!(hold, 2400.0);
        // Nothing probed at all: never under the pattern.
        assert_eq!(hold_altitude_ft(None, None, 1130.0, 1650.0), 1700.0);
        let why = rock_refused_line(
            "The standard pattern here does not clear the ground: the base needs 3.9 degrees.",
        );
        let line = terrain_hold_line(&why, hold);
        assert!(
            line.starts_with("The Rock circuit isn't clear from here."),
            "{line}"
        );
        assert!(
            line.contains("Climbing straight ahead to 2400 feet"),
            "{line}"
        );
        assert!(!line.contains("Your aircraft"), "{line}");
        no_dashes(&line);
        let level = terrain_hold_level_line(hold);
        assert!(level.contains("2400 feet") && level.ends_with("Your aircraft."));
        no_dashes(&level);
    }

    /// Handed over only once level at the hold (the autopilot holding it),
    /// never in the climb from the handoff height.
    #[test]
    fn the_rock_hold_hands_over_only_once_level() {
        let mut h = TerrainHold::refused(267.5, 2400.0);
        for alt in [1130.0, 1400.0, 1800.0, 2200.0, 2290.0] {
            assert!(!h.level_now(alt), "climbing at {alt}: still ours");
        }
        for _ in 0..HOLD_LEVEL_TICKS - 1 {
            assert!(!h.level_now(2380.0));
        }
        assert!(h.level_now(2410.0));
    }

    /// The circuit's guard still gives the hold back loudly: a climb that
    /// has not got there in its time (1,270 ft at 300 fpm plus a minute),
    /// or the servos out; a failed read changes no clock.
    #[test]
    fn the_rock_holds_guard_gives_back_a_climb_that_never_ends() {
        let limit = terrain_hold_limit_s(1130.0, 2400.0);
        assert!((limit - 314.0).abs() < 0.5, "{limit}");
        assert_eq!(terrain_hold_limit_s(f64::NAN, 2400.0), LEG_TIME_MARGIN_S);
        let mut g = PatternGuard::default();
        assert!(g
            .step(0.0, Leg::Upwind, 1130.0, 2400.0, limit, Some(true))
            .is_none());
        assert!(g
            .step(300.0, Leg::Upwind, 2100.0, 2400.0, limit, None)
            .is_none());
        assert_eq!(
            g.step(320.0, Leg::Upwind, 2200.0, 2400.0, limit, Some(true))
                .map(|t| t.why()),
            Some("leg_overdue")
        );
        let mut g = PatternGuard::default();
        assert!(g
            .step(0.0, Leg::Upwind, 1130.0, 2400.0, limit, Some(false))
            .is_none());
        assert_eq!(
            g.step(3.5, Leg::Upwind, 1200.0, 2400.0, limit, Some(false))
                .map(|t| t.why()),
            Some("servos_off")
        );
    }
