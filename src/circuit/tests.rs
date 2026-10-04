
    fn flight_plan(
        source: crate::simbrief::PlanSource,
        callsign: Option<&str>,
    ) -> crate::simbrief::FlightPlan {
        crate::simbrief::FlightPlan {
            transition_altitude_ft: None,
            transition_level_ft: None,
            procedures: Default::default(),
            callsign: callsign.map(str::to_string),
            schedule: Default::default(),
            source,
            origin: "EGLL".into(),
            destination: "EGPH".into(),
            alternate: String::new(),
            aircraft: "B738".into(),
            cruise_alt_ft: 35000.0,
            distance_nm: 300.0,
            ete_min: 60.0,
            trip_fuel_kg: 3000.0,
            reserve_kg: 1500.0,
            route: "DCT".into(),
            wind_comp_kts: 0.0,
            wind_dir: 0.0,
            wind_spd_kts: 0.0,
            tod_to_dest_nm: 100.0,
            waypoints: Vec::new(),
            block_fuel_kg: 6000.0,
            min_takeoff_fuel_kg: 5000.0,
            weights: None,
        }
    }

    /// The callsign lives with the plan: a local plan names no flight, a
    /// SimBrief plan names its own, and the plan that replaces it names the
    /// new one, with nothing stored in between.
    #[test]
    fn the_plan_callsign_is_the_plan_on_the_shelf() {
        use crate::simbrief::PlanSource;
        assert_eq!(super::plan_callsign(None), None);
        let local = flight_plan(PlanSource::Local, Some("BAW306"));
        assert_eq!(
            super::plan_callsign(Some(&local)),
            None,
            "a local plan names no flight"
        );
        let bare = flight_plan(PlanSource::SimBrief, None);
        assert_eq!(super::plan_callsign(Some(&bare)), None);

        let speak = |fp: &crate::simbrief::FlightPlan| {
            let inputs = crate::callsign::Inputs {
                override_id: None,
                plan: super::plan_callsign(Some(fp)),
            };
            crate::callsign::resolve(&inputs, Some("G-EUPT"), Some("B738"), Some(79_000.0))
                .map(|r| (r.spoken, r.source))
        };
        let first = flight_plan(PlanSource::SimBrief, Some("BAW306"));
        let p = super::plan_callsign(Some(&first)).expect("a SimBrief callsign");
        assert_eq!((p.id.as_str(), p.plan_type.as_str()), ("BAW306", "B738"));
        assert_eq!(
            speak(&first),
            Some((
                "Speedbird three zero six".into(),
                crate::callsign::Source::Simbrief
            ))
        );
        let replaced = flight_plan(PlanSource::SimBrief, Some("EZY12AB"));
        assert_eq!(
            speak(&replaced),
            Some((
                "Easy one two Alpha Bravo".into(),
                crate::callsign::Source::Simbrief
            ))
        );
        assert_eq!(
            speak(&local),
            Some((
                "Golf Echo Uniform Papa Tango".into(),
                crate::callsign::Source::Tail
            ))
        );
    }

    /// EGPF, 2026-09-21: pattern 1,796 ft, the FCU at 100. Level targets go
    /// on the dial to the hundred, a captured hold on that dial owns the leg,
    /// and the glidepath parks the dial above like autoland does.
    #[test]
    fn the_autopilot_holds_the_level_legs() {
        // Climbing to the pattern: select it.
        assert_eq!(circuit_dial_ft(0.0, 1795.8, 100.0, 600.0), Some(1800.0));
        assert_eq!(circuit_dial_ft(0.0, 1795.8, 1800.0, 1700.0), None);
        // Armed is not captured: the VS law is still flying the climb.
        assert!(!ap_holds_level(0.0, 1795.8, 1800.0, 1.0));
        assert!(ap_holds_level(0.0, 1795.8, 1800.0, 2.0));
        // The base leg's height: a new dial, and the old hold is not ours.
        assert_eq!(circuit_dial_ft(0.0, 820.0, 1800.0, 1796.0), Some(800.0));
        assert!(!ap_holds_level(0.0, 820.0, 1800.0, 2.0));
        // On the path a capture would eat the descent: parked above, once.
        let parked = circuit_dial_ft(-700.0, 700.0, 800.0, 810.0).expect("parked");
        assert!(parked >= 810.0 + 500.0, "{parked}");
        assert_eq!(circuit_dial_ft(-700.0, 600.0, parked, 700.0), None);
        assert!(!ap_holds_level(-700.0, 800.0, 800.0, 2.0));
        assert!(!vs_stuck(3.0) && vs_stuck(VS_REFUSED_SECS));
    }
    use super::*;
    use crate::heading_bug::{TURN_ENTRY_LEAD_DEG, TURN_IN_PROGRESS_BANK_DEG};

    /// The Baron's approach speed the older handover tests were flown at, and
    /// the landing limit of a runway of unknown width (the old 37 m).
    const BARON_KTS: f64 = 105.0;

    /// The base leg hands autoland an aircraft ON the path it is about to fly,
    /// whatever size the pattern is. Luton 2026-07-28: a flat 700 ft target put
    /// the MD-82 at the handover 520 ft below a path it then had one mile to
    /// catch, and it did not.
    #[test]
    fn the_base_leg_descends_the_path_not_to_a_flat_height() {
        let (elev, tpa) = (520.0, 520.0 + 1500.0);
        // 4 nm out on the centreline: the 3° path is ~1,272 ft above the field.
        let far = approach_leg_alt_ft(elev, tpa, -4.0 * M_PER_NM);
        assert!(
            (far - (elev + 4.0 * crate::autoland::GLIDEPATH_FT_PER_NM)).abs() < 1.0,
            "should target the path, got {far}"
        );
        // Closer in, the path is lower — and the leg follows it down.
        let near = approach_leg_alt_ft(elev, tpa, -2.5 * M_PER_NM);
        assert!(near < far, "{near} !< {far}");
        // The floor may not hold us ABOVE the path. It used to: a flat
        // 700 ft AGL bottom meant that on any pattern turning final inside
        // ~2.2 nm — where the 3° path is below 700 — the leg levelled off over
        // the path it was trying to join. The SF50's 1.84 nm final at EGHI met
        // a path at 585 ft AGL and the floor sat 115 ft on top of it
        // (2026-07-30). Close in, the path wins.
        let short_final = approach_leg_alt_ft(elev, tpa, -0.2 * M_PER_NM);
        let path_there = elev + 0.2 * crate::autoland::GLIDEPATH_FT_PER_NM;
        assert!(
            (short_final - path_there).abs() < 1.0,
            "close in the target IS the path, got {short_final} want {path_there}"
        );
        assert!(
            short_final < elev + HANDOFF_AGL_FT,
            "and it is allowed below the old flat floor"
        );

        // The EGHI geometry that produced the go-around: 1.84 nm final, field
        // at 41 ft, jet pattern. The handover must not be above the path.
        let (eghi, jet_tpa) = (41.0, 41.0 + 1184.0);
        let handover = approach_leg_alt_ft(eghi, jet_tpa, -1.84 * M_PER_NM);
        let eghi_path = eghi + 1.84 * crate::autoland::GLIDEPATH_FT_PER_NM;
        assert!(
            handover <= eghi_path + 1.0,
            "handed off {:.0} ft above the path",
            handover - eghi_path
        );
        // ...and never a CLIMB: far out, where the path is above the pattern
        // altitude, the leg simply holds what it has (the GA case — a 172's
        // 1,000 ft circuit is below the path at 4 nm).
        let ga_tpa = elev + 1000.0;
        assert_eq!(
            approach_leg_alt_ft(elev, ga_tpa, -4.0 * M_PER_NM),
            ga_tpa,
            "hold the pattern altitude, never climb on base"
        );
    }

    /// The descent has to START abeam the touchdown point, because base alone
    /// is not long enough to hold it.
    ///
    /// EGHI 2026-07-30: TPA 1,184 ft AGL, final turned at 1.84 nm where the
    /// path is 585 — 600 ft to lose in a 1.1 nm base, about 900 fpm. It
    /// managed 150, arrived 400 ft high, stayed high the whole final, crossed
    /// the threshold 70 ft up and went around. Splitting the descent gives
    /// base a job it can actually do.
    #[test]
    fn the_downwind_descent_leaves_base_a_reachable_job() {
        let (elev, tpa_agl) = (41.0, 1184.0);
        let tpa = elev + tpa_agl;
        let downwind_target = (elev + DOWNWIND_DESCENT_AGL_FT).min(tpa);

        // It descends, but stays a sane downwind height.
        assert!(downwind_target < tpa, "the 180 is where you start down");
        assert!(
            downwind_target - elev >= 700.0,
            "still a circuit height, not an approach: {:.0} ft AGL",
            downwind_target - elev
        );

        // What base is left with, against what it had before.
        let path_at_final = elev + 1.84 * crate::autoland::GLIDEPATH_FT_PER_NM;
        let was = tpa - path_at_final;
        let now = downwind_target - path_at_final;
        assert!(
            now < was * 0.75,
            "base's descent must shrink materially: {now:.0} ft vs {was:.0}"
        );

        // A 172's 1,000 ft circuit must not be pushed DOWN to 800 by this and
        // then find itself below its own pattern — the target is bounded by
        // the pattern altitude, so a low circuit is simply left alone.
        let ga_tpa = elev + 600.0;
        assert_eq!(
            (elev + DOWNWIND_DESCENT_AGL_FT).min(ga_tpa),
            ga_tpa,
            "never descend below a pattern that is already lower"
        );
    }

    /// The Southampton approach (2026-08-01), which is the whole reason the
    /// glidepath distance is no longer track miles.
    ///
    /// The Baron began base 3,324 m off the centreline. Track miles read 3.33
    /// nm and asked for 1,101 ft when the real distance to the threshold was
    /// 1.53 nm and the 3° path was at 529 — the target was inflated by exactly
    /// the lateral offset. The aircraft flew base and most of final level at
    /// 800 ft AGL, the target dropped below it only eleven seconds before the
    /// handoff, and autoland was handed an aircraft 206 ft high at 1.7 nm. It
    /// flew the whole final high, crossed the threshold 80 ft up at 102 knots
    /// and went around.
    #[test]
    fn the_glidepath_distance_is_down_the_course_not_across_it() {
        let (elev, tpa) = (41.0, 41.0 + 1000.0);
        let rollout = 4087.0; // geom.downwind_past_m, the live figure

        // The live base-entry geometry. Track miles (the old measure) put the
        // target above the pattern altitude itself; the real path is far
        // below. This is the bug, stated as a number.
        let (a, x): (f64, f64) = (-2840.0, 3324.0);
        let track = approach_leg_alt_ft(elev, tpa, a.abs() + x.abs());
        let along = approach_leg_alt_ft(elev, tpa, a);
        assert!(
            track - along > 500.0,
            "cross-track inflated the target by {:.0} ft",
            track - along
        );

        // ...but plain along-track is a dive to nowhere at that moment: the
        // aircraft is crossing the centreline, so its along-track distance is
        // still GROWING. Base aims at the roll-out instead.
        assert_eq!(path_dist_m(Leg::Base, a, rollout), rollout);
        let base_tgt = approach_leg_alt_ft(elev, tpa, path_dist_m(Leg::Base, a, rollout));
        assert!(
            (base_tgt - 744.0).abs() < 15.0,
            "base should aim at the path where the turn delivers us: {base_tgt:.0}"
        );

        // Flown, the roll-out came at 3,966 m — and there the target is the
        // path, not the flat 841 the old measure held.
        let rolled_out = approach_leg_alt_ft(elev, tpa, path_dist_m(Leg::Final, -3966.0, rollout));
        assert!(
            (rolled_out - 723.0).abs() < 15.0,
            "at roll-out the target is the path: {rolled_out:.0}"
        );
        assert!(rolled_out < 841.0, "and below what the old measure held");

        // Final follows the along-track distance down; at the handoff point
        // that is within a hair of what autoland will fly from.
        let handoff = approach_leg_alt_ft(elev, tpa, path_dist_m(Leg::Final, -3128.0, rollout));
        assert!(
            (handoff - 579.0).abs() < 15.0,
            "handoff on the path: {handoff:.0}"
        );

        // Still far out, the pattern altitude still caps it — a circuit does
        // not start down on the upwind.
        assert_eq!(approach_leg_alt_ft(elev, tpa, -6.0 * M_PER_NM), tpa);
    }

    /// The Southampton Baron (2026-08-01). It rolled into the crosswind turn
    /// at 127 kt and came out at 85 — one knot over its clean stall and one
    /// over Vmca — with the VS law still asking for 880 fpm and the stabilizer
    /// trimmed to the nose-up stop. The AP will keep buying rate with knots
    /// unless something stops it.
    #[test]
    fn a_climb_is_never_worth_more_than_the_airspeed_left() {
        // Baron 58: Vy 105, blue line 100, clean stall 84.
        // The floor is the higher of blue line and the 1.2 Vs automation
        // floor (refactor audit #13): 100.8 here, a hair over blue line.
        let floor = climb_floor_kts(Some(100.0), Some(84.0));
        assert!((floor - 100.8).abs() < 1e-9, "{floor}");
        assert_eq!(
            climb_floor_kts(Some(110.0), Some(84.0)),
            110.0,
            "blue line outranks the stall margin"
        );
        let want = 105.0;

        // At the pattern speed, the commanded rate is untouched.
        assert_eq!(climb_limit_fpm(880.0, 105.0, 0.0, want, floor), 880.0);
        assert_eq!(climb_limit_fpm(880.0, 130.0, 0.0, want, floor), 880.0);

        // At blue line and below, no climb at all — lower the nose, let the
        // throttle law get the speed back.
        assert_eq!(climb_limit_fpm(880.0, 100.0, 0.0, want, floor), 0.0);
        assert_eq!(
            climb_limit_fpm(880.0, 85.3, 0.0, want, floor),
            0.0,
            "the speed the live circuit actually reached"
        );

        // ...and in between, a fade rather than a switch.
        let mid = climb_limit_fpm(880.0, 102.5, 0.0, want, floor);
        assert!(mid > 0.0 && mid < 880.0, "{mid}");
        assert!(
            climb_limit_fpm(880.0, 104.0, 0.0, want, floor) > mid,
            "and it must be monotone in airspeed"
        );

        // DESCENTS are never touched: base and final descending is the point.
        assert_eq!(climb_limit_fpm(-500.0, 85.0, 0.0, want, floor), -500.0);

        // A profile with no reference speeds disables the protection outright
        // rather than inventing a floor.
        assert_eq!(climb_floor_kts(None, None), 0.0);
        assert_eq!(climb_limit_fpm(700.0, 40.0, 0.0, 74.0, 0.0), 700.0);

        // ...and it must not LIMIT-CYCLE. Live it did: 700 fpm to 0 in four
        // seconds as the speed dipped through the band, held shut while the
        // aircraft was already accelerating back, then slammed open — the
        // aeroplane porpoising thirty feet either way (2026-08-01). Judging
        // the speed we are HEADING FOR fixes both ends.
        //
        // Recovering at 2 kt/s from below the floor: give the climb back now,
        // not in five seconds' time.
        assert!(
            climb_limit_fpm(700.0, 99.0, 2.0, want, floor) > 0.0,
            "a recovering aircraft must not be held at zero climb"
        );
        // Decaying at 2 kt/s from above the pattern speed: start easing off
        // BEFORE the speed is lost, so the dip never gets deep.
        assert!(
            climb_limit_fpm(700.0, 112.0, -2.0, want, floor) < 700.0,
            "a decaying speed must ease the climb early"
        );
        // Steady on target is untouched either way.
        assert_eq!(climb_limit_fpm(700.0, 110.0, 0.0, want, floor), 700.0);

        // A 172 (Vy 74, stall 48) gets a floor of 57.6 (1.2 Vs, the
        // automation floor; it was 55.2 at 1.15), so a normally flown
        // trainer pattern never meets the limiter at all.
        let c172 = climb_floor_kts(None, Some(48.0));
        assert!((c172 - 57.6).abs() < 0.01, "{c172}");
        assert_eq!(climb_limit_fpm(700.0, 74.0, 0.0, 74.0, c172), 700.0);
    }

    /// The full-rate threshold is the speed the climb is FLOWN at, never the
    /// leg speed the throttle holds. The `want = 105.0` in the test above was
    /// always Vy; the call site passed the stall-floored pattern speed
    /// (109.2), and the four-knot gap parked the throttle's setpoint at the
    /// top of the fade band: full climb unreachable while climbing, cmd_vs
    /// porpoising 0-150 fpm, the Baron descending at 190 fpm at 670 ft AGL
    /// mid-climb-out and then a whole downwind hung 100 ft below pattern
    /// altitude (EGHI, 2026-08-02).
    #[test]
    fn full_climb_is_granted_at_the_speed_the_climb_is_flown_at() {
        // Baron 58, wired exactly as the live loop wires it: pattern 109.2
        // (1.3 x the 84 kt clean stall), Vy 105, floor 100 (blue line).
        let pattern = pattern_speed_kts(105.0, 95.0, Some(84.0));
        let floor = climb_floor_kts(Some(100.0), Some(84.0));
        let full = climb_full_rate_kts(pattern, 105.0, floor);
        assert_eq!(full, 105.0, "Vy, not the stall-floored pattern speed");

        // At Vy the climb is untouched — climbing at Vy is flying correctly.
        assert_eq!(climb_limit_fpm(700.0, 105.0, 0.0, full, floor), 700.0);
        // And at the throttle's own setpoint, with room to spare: the steady
        // operating point must sit ABOVE the fade band, not on its edge.
        assert!(
            pattern > full + 3.0,
            "no headroom left: {pattern} vs {full}"
        );
        assert_eq!(climb_limit_fpm(700.0, pattern, 0.0, full, floor), 700.0);
        // Genuine sag below Vy still fades, and blue line still shuts it.
        let sagged = climb_limit_fpm(700.0, 103.0, 0.0, full, floor);
        assert!(sagged > 0.0 && sagged < 700.0, "{sagged}");
        assert_eq!(climb_limit_fpm(700.0, 100.0, 0.0, full, floor), 0.0);

        // A jet's book Vy (a cruise-climb figure) must never RAISE the
        // threshold above the leg speed: the Citation's 300 stays capped at
        // its pattern speed.
        let citation = pattern_speed_kts(300.0, 132.0, Some(136.0));
        assert_eq!(climb_full_rate_kts(citation, 300.0, 156.4), citation);

        // A Vy at or under the floor still leaves a band, never a cliff.
        assert_eq!(climb_full_rate_kts(96.0, 94.0, 100.0), 102.0);
    }

    /// The pattern is flown CLEAN, so its speed must clear the CLEAN stall.
    /// The Baron's Vy is 105 against an 84 kt clean stall — 1.25 Vs, four
    /// knots above the airspeed caution band — so every normal variation on
    /// downwind, base and final dipped into it: six callouts in one circuit,
    /// every one correct (2026-08-01). Flap was 0.00 throughout, which is
    /// right; nothing knew that made the stall reference 84 and not 70.
    #[test]
    fn the_clean_pattern_clears_the_clean_stall() {
        // Baron 58: Vy 105, approach 95, clean stall 84.
        let baron = pattern_speed_kts(105.0, 95.0, Some(84.0));
        assert!(
            (baron - 109.2).abs() < 0.1,
            "1.3 x the clean stall: {baron:.1}"
        );
        // Clear of the caution band (1.2 Vs) with real margin, not four knots.
        assert!(baron - 1.2 * 84.0 > 7.0, "margin {:.1}", baron - 1.2 * 84.0);

        // A 172 (Vy 74, clean stall 48) is untouched — its Vy is already
        // 1.54 Vs, and this must never speed a trainer's circuit up.
        assert_eq!(pattern_speed_kts(74.0, 65.0, Some(48.0)), 74.0);
        // ...as is an MD-82 (Vy 200, stall 100) and a Citation (200, 136).
        assert_eq!(pattern_speed_kts(200.0, 140.0, Some(100.0)), 200.0);
        assert!(pattern_speed_kts(200.0, 132.0, Some(136.0)) <= 198.0);

        // No stall figure means no floor, never an invented one.
        assert_eq!(pattern_speed_kts(105.0, 95.0, None), 105.0);

        // The approach legs inherit it, so base and final are clear too.
        let app = approach_leg_kts(baron, 95.0);
        assert!(
            app - 1.2 * 84.0 > 7.0,
            "base/final margin {:.1}",
            app - 1.2 * 84.0
        );
    }

    /// Base and final are flown at an approach-setup speed, not the climb speed
    /// the rest of the pattern uses — but only ever SLOWER than the pattern.
    #[test]
    fn the_approach_legs_slow_toward_the_approach_speed() {
        // Jet: Vy 200, approach 140 → 160, comfortably clean, close enough for
        // autoland to take the last 20 kt off with flaps.
        assert_eq!(approach_leg_kts(200.0, 140.0), 160.0);
        // Trainer: Vy 74 is already below approach + margin, so nothing
        // changes — this must never speed a light aircraft UP on base.
        assert_eq!(approach_leg_kts(74.0, 65.0), 74.0);
        // A slow pattern speed is still never taken below the approach speed.
        assert_eq!(approach_leg_kts(60.0, 65.0), 65.0);
    }

    /// A proportional controller cannot track a ramp, and the glidepath is a
    /// ramp — so the final leg flew a permanent lag and handed autoland an
    /// aircraft it could not get down.
    ///
    /// This is the Malaga approach, simulated (2026-08-05). The circuit rolls
    /// out onto final 4,067 m from the threshold at 800 ft MSL, 54 ft above
    /// the path, closing at 110 kt, and flies the leg. The blackbox says what
    /// happened for real: the error GREW to 172 ft by the handoff, the
    /// commanded descent never once passed the old -500 clamp, and the
    /// commanded rate at the end (-406) was less than the rate the target
    /// itself was descending at (-510). The aircraft was not failing to close
    /// the gap; it was falling further behind every second.
    #[test]
    fn the_final_leg_stays_on_a_glidepath_it_is_flying_down() {
        let elev = 48.0;
        let tpa = elev + 1000.0;
        let closure = 110.0;
        // Where the roll-out actually put us, and the height it left us at.
        let mut along_m = 4067.0;
        let mut alt = elev + 752.0;
        let mut vs = -150.0;
        let dt = 1.0;
        let mut worst_high: f64 = 0.0;
        while along_m > 400.0 {
            let target = approach_leg_alt_ft(elev, tpa, along_m);
            let cmd = alt_capture_vs(target, path_descent_fpm(closure), alt, vs);
            // A VS-mode autopilot gets to its commanded rate quickly and then
            // holds it; a first-order lag is plenty to show the tracking.
            vs += (cmd - vs) * 0.4;
            alt += vs / 60.0 * dt;
            along_m -= closure * 1852.0 / 3600.0 * dt;
            worst_high = worst_high.max(alt - target);
        }
        assert!(
            worst_high < 50.0,
            "the leg must never get FURTHER above the path than it started: \
             {worst_high:.0} ft high"
        );
        // The handoff point itself — 1.5 nm out, where autoland took over
        // 174 ft high on the day.
        let at_handoff = alt - approach_leg_alt_ft(elev, tpa, along_m);
        assert!(
            at_handoff.abs() < 40.0,
            "autoland must inherit an aircraft on the path: {at_handoff:.0} ft off"
        );
        // And the reason the old law could not have done it even in principle:
        // its descent authority was less than the path's own rate.
        assert!(
            path_descent_fpm(closure) < -500.0,
            "a 3 degree path at approach speed descends faster than the old clamp"
        );
        // A standing target keeps exactly the authority it always had.
        assert_eq!(alt_capture_vs(0.0, 0.0, 1000.0, 0.0), -500.0);
    }

    /// The altitude capture leads on vertical speed, because a vertical speed
    /// does not stop when the command does. Luton 2026-07-28: pure proportional
    /// control climbed at 700 fpm right up to pattern altitude, sailed 100 ft
    /// through it, and hunted ±60 ft for the whole downwind.
    #[test]
    fn the_altitude_capture_levels_where_the_aircraft_will_arrive() {
        // Well below, stationary in the vertical: climb at the clamp.
        assert_eq!(alt_capture_vs(2000.0, 0.0, 1000.0, 0.0), 700.0);
        // Same error, but already climbing hard: the command backs off,
        // because most of that gap will be eaten before the aircraft responds.
        let climbing = alt_capture_vs(2000.0, 0.0, 1900.0, 700.0);
        let level = alt_capture_vs(2000.0, 0.0, 1900.0, 0.0);
        assert!(climbing < level, "{climbing} !< {level}");
        // Arriving at the target with rate on: command a descent to stop the
        // overshoot BEFORE it happens, which is the whole point.
        assert!(
            alt_capture_vs(2000.0, 0.0, 2000.0, 700.0) < 0.0,
            "must arrest an overshoot in progress"
        );
        // On target and steady: hands off.
        assert_eq!(alt_capture_vs(2000.0, 0.0, 2000.0, 0.0), 0.0);
    }

    /// A jet flies a jet's circuit: 1,500 ft, because its wider pattern turns
    /// final where a 3° path is well above a trainer's 1,000 ft.
    #[test]
    fn the_pattern_altitude_follows_the_airframe() {
        use crate::tools::circuit::pattern_alt_agl_ft;
        assert_eq!(pattern_alt_agl_ft(65.0), 1000.0, "trainer");
        assert_eq!(pattern_alt_agl_ft(90.0), 1000.0, "fast single");
        assert_eq!(pattern_alt_agl_ft(140.0), 1500.0, "jet");
    }

    #[test]
    fn left_pattern_turns_ninety_degrees_left_each_leg() {
        // Runway 27 (course 270), standard left-hand pattern.
        assert_eq!(leg_heading(270.0, Leg::Upwind, true), 270.0);
        assert_eq!(leg_heading(270.0, Leg::Crosswind, true), 180.0);
        assert_eq!(leg_heading(270.0, Leg::Downwind, true), 90.0);
        assert_eq!(leg_heading(270.0, Leg::Base, true), 0.0);
        // Final comes all the way back to the runway heading.
        assert_eq!(leg_heading(270.0, Leg::Final, true), 270.0);
    }

    #[test]
    fn right_pattern_turns_the_other_way_and_wraps() {
        // Runway 09 (course 90), right-hand pattern, checking the 360 wrap.
        assert_eq!(leg_heading(90.0, Leg::Crosswind, false), 180.0);
        assert_eq!(leg_heading(90.0, Leg::Downwind, false), 270.0);
        assert_eq!(leg_heading(90.0, Leg::Base, false), 0.0); // 90+270 = 360 -> 0
        assert_eq!(leg_heading(90.0, Leg::Final, false), 90.0);
    }

    #[test]
    fn the_downwind_is_always_the_reciprocal() {
        for course in [0.0_f64, 27.0, 90.0, 200.0, 359.0] {
            let recip = (course + 180.0).rem_euclid(360.0);
            assert!((leg_heading(course, Leg::Downwind, true) - recip).abs() < 1e-9);
            assert!((leg_heading(course, Leg::Downwind, false) - recip).abs() < 1e-9);
        }
    }

    // Position-based sequencing. A left pattern on a 1500 m runway.
    // In the runway frame: a_m along the takeoff direction, x_m right-positive;
    // a left pattern sits at NEGATIVE x_m.
    const L: f64 = 1500.0;

    /// A 172-ish geometry: ~80 kt pattern speed.
    fn g() -> Geometry {
        Geometry::for_radius(turn_radius_m(41.0))
    }

    #[test]
    fn the_bug_slews_the_short_way_across_the_north_seam() {
        // The killer case: right pattern's crosswind, dial 271.5 -> target 2.6.
        // Every step must move CLOCKWISE (increasing, mod 360) — the single
        // jump read as a counterclockwise spin and the AP turned 270° the
        // wrong way (2026-07-24).
        let mut bug = 271.5;
        let mut steps = 0;
        while (slew_bug(bug, 2.6, 40.0) - 2.6).abs() > 1e-9 || (bug - 2.6).abs() > 1e-9 {
            let next = slew_bug(bug, 2.6, 40.0);
            let d = crate::angles::signed_diff_deg(next, bug);
            assert!(d > 0.0, "always clockwise: {bug} -> {next}");
            assert!(d <= 40.0 + 1e-9, "never more than a step");
            bug = next;
            steps += 1;
            assert!(steps < 10, "converges");
        }
        // Counterclockwise across the seam works too: 2.6 -> 330 is a 32.6°
        // short-way step, done in one write; a farther target clamps.
        assert!((slew_bug(2.6, 330.0, 40.0) - 330.0).abs() < 1e-9);
        assert!(
            (slew_bug(2.6, 280.0, 40.0) - 322.6).abs() < 1e-9,
            "clamped first step"
        );
        // Small deltas go direct.
        assert!((slew_bug(100.0, 110.0, 40.0) - 110.0).abs() < 1e-9);
    }

    #[test]
    fn geometry_scales_from_the_turn_radius() {
        // ~80 kt (41 m/s) at 14° bank → radius ~700 m. A standard-rate 20° was
        // tried and reverted: the tighter pattern degenerated into a figure of
        // eight because the legs got shorter than the turns. The pattern being
        // WIDE is handled by the handover gate below, not by shrinking it.
        let r = turn_radius_m(41.0);
        assert!((550.0..900.0).contains(&r), "radius {r}");
        let geom = Geometry::for_radius(r);
        // The width must exceed the turn DIAMETER — legs shorter than the
        // diameter are what degenerated the rectangle into loops.
        assert!(
            geom.width_m > 2.0 * r,
            "width {} vs diameter {}",
            geom.width_m,
            2.0 * r
        );
        assert!(geom.downwind_past_m > geom.width_m, "room for base + final");
        assert!(geom.lead_m < r, "lead is less than the radius itself");
        // The pattern must be INTERCEPTABLE: after the base turn eats one
        // radius of the offset, what is left has to be closeable in the
        // distance that remains. A geometry that fails this cannot be flown
        // onto the centreline however well the laws behave — EGHI handed
        // autoland 2,815 m of offset with 2,421 m to run and it crossed the
        // threshold 188 m off the paint.
        let rollout_xtk = (geom.final_capture_m - r).max(0.0);
        assert!(
            interceptable(rollout_xtk, geom.downwind_past_m.min(2.0 * r + 1200.0)),
            "base rolls out {rollout_xtk:.0} m off — must be closeable"
        );
        // A bad speed read can't produce a degenerate pattern.
        assert!(turn_radius_m(0.0) >= 250.0);
        assert!(turn_radius_m(500.0) <= 2500.0);
    }

    #[test]
    fn upwind_turns_crosswind_past_the_end_while_climbing() {
        // Not yet past the departure end: hold, whatever the altitude.
        assert!(!pattern_advance(Leg::Upwind, L - 100.0, 0.0, &g(), L, true));
        // Past the end + margin: turn crosswind (the altitude no longer gates —
        // a real pattern climbs to TPA on the crosswind/downwind).
        assert!(pattern_advance(
            Leg::Upwind,
            L + g().upwind_past_m + 1.0,
            0.0,
            &g(),
            L,
            true
        ));
    }

    #[test]
    fn crosswind_turns_downwind_with_turn_anticipation() {
        let geom = g();
        // Still inside the led trigger -> hold.
        let before = geom.width_m - geom.lead_m - 50.0;
        assert!(!pattern_advance(Leg::Crosswind, L, -before, &geom, L, true));
        // Reached width - lead -> turn NOW, so the rollout lands on the line.
        let at = geom.width_m - geom.lead_m + 1.0;
        assert!(pattern_advance(Leg::Crosswind, L, -at, &geom, L, true));
        // Right pattern mirrors: the spacing is to the RIGHT (positive x).
        assert!(pattern_advance(Leg::Crosswind, L, at, &geom, L, false));
        assert!(!pattern_advance(Leg::Crosswind, L, -at, &geom, L, false));
    }

    #[test]
    fn downwind_turns_base_past_the_landing_threshold() {
        let geom = g();
        // Still abeam / not far enough back -> hold.
        assert!(!pattern_advance(
            Leg::Downwind,
            200.0,
            -geom.width_m,
            &geom,
            L,
            true
        ));
        // Past the threshold by the led base distance -> turn base.
        let at = geom.downwind_past_m - geom.lead_m + 1.0;
        assert!(pattern_advance(
            Leg::Downwind,
            -at,
            -geom.width_m,
            &geom,
            L,
            true
        ));
    }

    #[test]
    fn base_turns_final_one_radius_from_the_centreline() {
        let geom = g();
        // Still well left of the capture line -> hold.
        assert!(!pattern_advance(
            Leg::Base,
            -geom.downwind_past_m,
            -(geom.final_capture_m + 200.0),
            &geom,
            L,
            true
        ));
        // Within one radius: the 90° turn rolls out on the centreline.
        assert!(pattern_advance(
            Leg::Base,
            -geom.downwind_past_m,
            -(geom.final_capture_m - 50.0),
            &geom,
            L,
            true
        ));
    }

    fn facts(terrain: Option<f64>, air: Option<&str>) -> SideFacts {
        SideFacts {
            terrain_max_ft: terrain,
            airspace: air.map(|s| s.to_string()),
            ..Default::default()
        }
    }

    fn pick3(p: PatternPick) -> (bool, f64, &'static str) {
        assert!(p.refused.is_none(), "refused: {:?}", p.refused);
        (p.left, p.tpa_msl_ft, p.why)
    }

    #[test]
    fn terrain_pick_switches_sides_and_raises_when_boxed_in() {
        // The Gibraltar case: left downwind crosses the Rock (1,400 ft), right
        // side is the flat bay at field height: a default-left ask switches
        // right at TPA.
        let (left, tpa, why) = pick3(choose_pattern(
            true,
            &facts(Some(1400.0), None),
            &facts(Some(11.0), None),
            11.0,
            1000.0,
        ));
        assert!(!left, "switched off the Rock side");
        assert!((tpa - 1011.0).abs() < 1.0);
        assert_eq!(why, "switched_side_for_terrain");
        // Clear both sides: the asked-for side stands.
        let (left, tpa, why) = pick3(choose_pattern(
            true,
            &facts(Some(11.0), None),
            &facts(Some(5.0), None),
            11.0,
            1000.0,
        ));
        assert!(left);
        assert!((tpa - 1011.0).abs() < 1.0);
        assert_eq!(why, "default_side_clear");
        // 1,000 ft of clearance (#182): a 1,000 ft pattern over ground 19 ft
        // above the field is lifted over it, on the side asked for when the
        // other is no lower by RAISE_SIDE_MARGIN_FT.
        let (left, tpa, why) = pick3(choose_pattern(
            true,
            &facts(Some(30.0), None),
            &facts(Some(20.0), None),
            11.0,
            1000.0,
        ));
        assert!(left, "a few feet are no reason to change sides");
        assert!((tpa - 1030.0).abs() < 1.0, "{tpa}");
        assert_eq!(why, "raised_for_terrain");
        // Boxed in (a valley field): fly the LOWER side, high enough to clear.
        let (left, tpa, why) = pick3(choose_pattern(
            true,
            &facts(Some(2000.0), None),
            &facts(Some(1500.0), None),
            500.0,
            1000.0,
        ));
        assert!(!left, "the lower side");
        assert!(
            (tpa - 2500.0).abs() < 1.0,
            "raised to terrain + pattern height: {tpa}"
        );
        assert_eq!(why, "raised_for_terrain");
        // No probe data: default side, default height — the old behavior.
        let (left, tpa, why) = pick3(choose_pattern(
            true,
            &facts(None, None),
            &facts(None, None),
            11.0,
            1000.0,
        ));
        assert!(left);
        assert!((tpa - 1011.0).abs() < 1.0);
        assert_eq!(why, "default_side_clear");
    }

    #[test]
    fn airspace_fouls_a_side_like_terrain_does() {
        // Default side flat but inside R-164: switch to the other flat side.
        let (left, tpa, why) = pick3(choose_pattern(
            true,
            &facts(Some(10.0), Some("restricted area R 164")),
            &facts(Some(11.0), None),
            11.0,
            1000.0,
        ));
        assert!(!left, "switched away from the restricted side");
        assert!((tpa - 1011.0).abs() < 1.0);
        assert_eq!(why, "switched_side_for_airspace");
        // Other side blocked by airspace, default side merely high terrain:
        // stay on the default side, raised over its terrain.
        let (left, tpa, why) = pick3(choose_pattern(
            true,
            &facts(Some(1400.0), None),
            &facts(Some(20.0), Some("restricted area R 164")),
            11.0,
            1000.0,
        ));
        assert!(left, "the airspace-free side, even with terrain");
        assert!((tpa - 2400.0).abs() < 1.0, "raised over the Rock: {tpa}");
        assert_eq!(why, "raised_for_terrain");
        // Both sides inside restricted airspace: terrain safety alone picks
        // the lower side, honestly labeled.
        let (left, _, why) = pick3(choose_pattern(
            true,
            &facts(Some(1400.0), Some("restricted area R 164")),
            &facts(Some(50.0), Some("restricted area R 164")),
            11.0,
            1000.0,
        ));
        assert!(!left, "the lower-terrain side");
        assert_eq!(why, "airspace_unavoidable");
    }

    /// A descent sample `thr_nm` out on `leg`, its path judged at `path_nm`.
    fn gs(leg: DescentLeg, path_nm: f64, thr_nm: f64, ground_ft: f64) -> GroundSample {
        GroundSample {
            leg,
            path_m: path_nm * M_PER_NM,
            thr_m: thr_nm * M_PER_NM,
            ground_ft,
        }
    }

    /// The LEMG 13 737 (sortie 1790599750, #182): the left side's drawn line
    /// read 729 ft and the circuit flew it at 1,548 (48 ft field + 1,500):
    /// the downwind 1.3 km outside that line, the base turn 305 ft over a
    /// 900 ft hill. With the corridor the level legs read 1,150, and the
    /// base (judged at the 6.2 km roll-out, 3.35 nm) crosses 900 ft ground
    /// 4.6 nm from the threshold. The right side is the Sierra de Mijas.
    #[test]
    fn lemg_13_left_is_never_flown_at_1548() {
        let left = SideFacts {
            terrain_max_ft: Some(1150.0),
            line_max_ft: Some(729.0),
            descent: vec![
                gs(DescentLeg::Platform, 1.0, 3.8, 400.0),
                gs(DescentLeg::Base, 3.35, 4.6, 900.0),
                gs(DescentLeg::Final, 2.0, 2.0, 60.0),
            ],
            ..Default::default()
        };
        let right = SideFacts {
            terrain_max_ft: Some(1818.0),
            line_max_ft: Some(1818.0),
            descent: vec![gs(DescentLeg::Base, 3.35, 4.6, 1600.0)],
            ..Default::default()
        };
        let p = choose_pattern(true, &left, &right, 48.0, 1500.0);
        assert!(
            !(p.refused.is_none() && p.left && (p.tpa_msl_ft - 1548.0).abs() < 1.0),
            "flown at 1,548 over the corridor: {p:?}"
        );
        // Neither side's base can be flown 500 ft clear on a legal path: the
        // left needs (900 + 500 - 48) / 3.35 = 404 ft a mile, 3.8 degrees.
        let why = p.refused.expect("refused, never flown");
        assert!(why.contains("left") && why.contains("degree"), "{why}");
        assert!(
            !why.contains('\u{2014}') && !why.contains('\u{2013}'),
            "no dashes: {why}"
        );
        assert_eq!(p.why, "refused_for_terrain");
        // The old law's own number on that base: 848 ft platform over 900.
        assert!(p.descent_left.default_min_ft.unwrap() < 0.0);
    }

    /// A hill under the base that a steeper, still legal path clears: the
    /// platform and the path are raised, never the pattern refused.
    #[test]
    fn a_hill_under_the_base_steepens_the_path_within_the_limit() {
        let side = SideFacts {
            terrain_max_ft: Some(200.0),
            descent: vec![
                gs(DescentLeg::Platform, 1.0, 3.8, 500.0),
                gs(DescentLeg::Base, 3.35, 4.0, 750.0),
            ],
            ..Default::default()
        };
        let p = choose_pattern(true, &side, &side, 48.0, 1500.0);
        assert!(p.refused.is_none(), "{p:?}");
        assert!((p.tpa_msl_ft - 1548.0).abs() < 1.0);
        // The platform is over the base's need too (the base flies the lower
        // of it and the path): 750 + 500 = 1,250 MSL; the path
        // (1,250 - 48) / 3.35 = 359 ft a mile, 3.4 degrees.
        assert!(
            (p.shape.platform_agl_ft - 1202.0).abs() < 1.0,
            "{:?}",
            p.shape
        );
        assert!(p.shape.steep());
        let deg = p.shape.glidepath_deg();
        assert!(deg > 3.0 && deg <= STEEP_PATH_MAX_DEG, "{deg}");
        let flown = p.descent_left.flown_min_ft.unwrap();
        assert!(flown >= 499.0, "500 ft kept: {flown}");
        let line = descent_shape_line(p.shape, 48.0);
        assert!(
            line.contains("1250 feet") && line.contains("3.4 degree"),
            "{line}"
        );
    }

    /// A level field is never steepened: the floor's taper sits under the
    /// 3 degree path all the way in.
    #[test]
    fn a_level_field_keeps_the_standard_descent() {
        let flat: Vec<GroundSample> = (1..=8)
            .map(|i| gs(DescentLeg::Final, i as f64 * 0.5, i as f64 * 0.5, 48.0))
            .chain([gs(DescentLeg::Platform, 1.0, 3.0, 48.0)])
            .collect();
        let c = check_descent(&flat, 48.0, 1548.0);
        assert_eq!(c.shape, Some(DescentShape::default()));
        assert!(c.problem.is_none());
    }

    /// The live floor: inside 500 ft of the ground ahead the circuit stops
    /// descending and climbs (800 over it on a level leg, 500 on base);
    /// never on final; the upwind keeps the old guard's 400.
    #[test]
    fn the_live_floor_trips_inside_500_ft_and_never_on_final() {
        // LEMG 13's base, 1,203 ft MSL over 898 ft of ground.
        assert_eq!(live_floor_ft(Leg::Base, 1203.0, 898.0), Some(1398.0));
        // LEMG 30's downwind, 475 ft AGL.
        assert_eq!(live_floor_ft(Leg::Downwind, 848.0, 373.0), Some(1173.0));
        assert_eq!(live_floor_ft(Leg::Downwind, 1548.0, 729.0), None);
        assert_eq!(live_floor_ft(Leg::Final, 600.0, 500.0), None);
        assert_eq!(live_floor_ft(Leg::Upwind, 570.0, 120.0), None, "450 over");
        assert_eq!(live_floor_ft(Leg::Upwind, 570.0, 200.0), Some(1000.0));
        assert_eq!(
            live_floor_ft(Leg::Base, f64::NAN, 200.0),
            None,
            "no reading"
        );
    }

    /// The corridor is the rectangle and its copies, tagged with the legs
    /// the circuit flies there: the downwind until abeam level, then the
    /// platform, then (past its led turn) the base; the base's path is
    /// judged at the roll-out distance or further.
    #[test]
    fn the_corridor_probes_outside_the_drawn_line() {
        let g = Geometry::for_radius(2500.0);
        let pts = corridor_frame(&g, 3200.0, true);
        let r = 2500.0;
        let widest = pts.iter().map(|p| -p.x_m).fold(f64::MIN, f64::max);
        assert!((widest - (g.width_m + r)).abs() < 1.0, "1 r out: {widest}");
        let innermost_downwind = pts
            .iter()
            .filter(|p| p.leg == Leg::Downwind)
            .map(|p| -p.x_m)
            .fold(f64::MAX, f64::min);
        assert!((innermost_downwind - (g.width_m - 0.25 * r)).abs() < 1.0);
        assert!(pts.iter().all(|p| p.x_m <= 0.0), "left side only");
        let furthest_base = pts.iter().map(|p| p.a_m).fold(f64::MAX, f64::min);
        assert!((furthest_base + g.downwind_past_m + r).abs() < 1.0);
        // Classification.
        let at = |leg, a_m| FramePt {
            a_m,
            x_m: -g.width_m,
            leg,
            line: true,
        };
        assert_eq!(descent_leg_at(&at(Leg::Crosswind, 4000.0), &g), None);
        assert_eq!(descent_leg_at(&at(Leg::Downwind, 100.0), &g), None);
        assert_eq!(
            descent_leg_at(&at(Leg::Downwind, -1000.0), &g).map(|d| d.0),
            Some(DescentLeg::Platform)
        );
        let (leg, path) = descent_leg_at(&at(Leg::Downwind, -5341.0), &g).unwrap();
        assert_eq!(
            leg,
            DescentLeg::Base,
            "past the led turn the base has begun"
        );
        assert!((path - g.downwind_past_m).abs() < 1.0);
        // Facts: the line alone vs the corridor.
        let ground: Vec<f64> = pts
            .iter()
            .map(|p| {
                if -p.x_m > g.width_m + 1000.0 {
                    1150.0
                } else {
                    700.0
                }
            })
            .collect();
        let f = corridor_facts(&pts, &ground, &g);
        assert_eq!(f.line_max_ft, Some(700.0));
        assert_eq!(f.terrain_max_ft, Some(1150.0));
        assert!(f.descent.iter().all(|s| s.thr_m >= DESCENT_CHECK_FROM_M));
        assert!(f
            .descent
            .iter()
            .any(|s| s.leg == DescentLeg::Base && s.ground_ft == 1150.0));
    }

    /// Shortening is only offered where a final is left: a jet's drawn
    /// downwind is already the least, an SF50's has room.
    #[test]
    fn a_downwind_is_shortened_only_while_a_final_is_left() {
        let jet = Geometry::for_radii(2500.0, 2500.0);
        let least = shortest_downwind_past_m(2500.0, 140.0);
        assert!(shorter_downwinds(jet.downwind_past_m, least).is_empty());
        let sf50 = Geometry::for_radii(1885.0, 1114.0);
        let tries = shorter_downwinds(sf50.downwind_past_m, shortest_downwind_past_m(1114.0, 90.0));
        assert!(!tries.is_empty() && tries.iter().all(|d| *d < sf50.downwind_past_m));
    }

    /// Real-install dry run of the LXGB 27 side pick against the actual
    /// airspace file (terrain unknown — the probe needs the sim):
    /// `cargo test -- --ignored real_install_lxgb_side_pick --nocapture`
    #[test]
    #[ignore]
    fn real_install_lxgb_side_pick() {
        let apt =
            &crate::tools::autoland::xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
        let air = &crate::tools::autoland::xplane_path("Custom Data/airspaces/airspace.txt");
        let cands = nearby_runways(apt, 36.1515, -5.3412, 2.0).expect("apt.dat");
        let rwy = cands
            .iter()
            .find(|r| r.id == "27")
            .expect("LXGB 27")
            .clone();
        let geom = Geometry::for_radius(turn_radius_m(41.0));
        let poly_l = pattern_polyline(&rwy, &geom, rwy.length_m, true);
        let poly_r = pattern_polyline(&rwy, &geom, rwy.length_m, false);
        let spaces = crate::airspace::load_near(air, 36.1515, -5.3412, 0.5);
        println!("no-go volumes near LXGB: {}", spaces.len());
        let tpa = 11.0 + 1000.0;
        let l_air = crate::airspace::first_conflict(&spaces, &poly_l, tpa).map(|a| a.spoken_name());
        let r_air = crate::airspace::first_conflict(&spaces, &poly_r, tpa).map(|a| a.spoken_name());
        println!("left (south, Rock side) airspace: {l_air:?}");
        println!("right (north, bay side) airspace: {r_air:?}");
        // Terrain from the real probe values seen live: Rock ~1,400 under the
        // southern downwind, flat under the northern.
        let l = SideFacts {
            terrain_max_ft: Some(1400.0),
            airspace: l_air,
            ..Default::default()
        };
        let r = SideFacts {
            terrain_max_ft: Some(60.0),
            airspace: r_air,
            ..Default::default()
        };
        let pick = choose_pattern(true, &l, &r, 11.0, 1000.0);
        let (left, tpa_msl, why) = (pick.left, pick.tpa_msl_ft, pick.why);
        println!("DECISION: left={left} tpa={tpa_msl:.0} why={why}");
        assert!(
            !left || tpa_msl > 1900.0,
            "never a 1,000 ft pattern over the Rock"
        );
    }

    #[test]
    fn spells_runways_for_the_radio() {
        assert_eq!(spell_runway("27"), "two seven");
        assert_eq!(spell_runway("09L"), "zero nine left");
        assert_eq!(spell_runway("36R"), "three six right");
        assert_eq!(spell_runway("6"), "six");
        assert_eq!(spell_runway("13C"), "one three center");
    }

    /// The crew radio queue is process-wide: its tests take turns.
    static QUEUE_LOCK: Mutex<()> = Mutex::new(());

    #[test]
    fn crew_radio_queue_is_fifo_and_drains_once() {
        let _g = QUEUE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let _ = take_crew_radio(); // clear any residue
        transmit("Downwind runway two seven".to_string());
        transmit("Final runway two seven".to_string());
        assert_eq!(
            take_crew_radio()
                .into_iter()
                .map(|q| q.call)
                .collect::<Vec<_>>(),
            vec![
                CrewCall::Text("Downwind runway two seven".to_string()),
                CrewCall::Text("Final runway two seven".to_string())
            ]
        );
        assert!(take_crew_radio().is_empty(), "a second drain is empty");
    }

    const LEMG_FIVE_MILE: &str =
        "November eight one six November November, 5 mile final runway one three";

    /// LEMG, sortie 1790584300-approach (2026-09-28): four stacked approach
    /// setups queued the same "5 mile final" in the same tenth of a second
    /// (t 84315.9). One report goes on the queue; a different call after it
    /// still does, and the same words again once the first has gone are a
    /// new call.
    #[test]
    fn identical_calls_queued_back_to_back_collapse_to_one() {
        let _g = QUEUE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let _ = take_crew_radio();
        for _ in 0..4 {
            transmit(LEMG_FIVE_MILE.to_string());
        }
        transmit("November eight one six November November, going around".to_string());
        transmit(LEMG_FIVE_MILE.to_string());
        let calls: Vec<String> = take_crew_radio()
            .into_iter()
            .map(|q| call_words(&q.call))
            .collect();
        assert_eq!(
            calls,
            vec![
                LEMG_FIVE_MILE.to_string(),
                "November eight one six November November, going around".to_string(),
                LEMG_FIVE_MILE.to_string(),
            ]
        );
        transmit(LEMG_FIVE_MILE.to_string());
        assert_eq!(
            take_crew_radio().len(),
            1,
            "a drained call is not collapsed into"
        );
    }

    /// The recorded timeline: queued at t 84315.9 (4.7 nm, 1,350 ft AGL,
    /// 172 kt); sent at 84409.1 (88 ft AGL, 149 kt, the flare), 84421.2
    /// (on the wheels), 84438.9 (rollout) and 84452.7 (stopped). Every one
    /// of those four is stale; the call made on time is not.
    #[test]
    fn a_five_mile_final_report_dies_with_the_five_mile_final() {
        let holds = holds_for(&CrewCall::Text(LEMG_FIVE_MILE.to_string()));
        assert!(matches!(
            holds,
            Holds::Airborne {
                max_travel_nm: Some(_),
                before_landing: true,
                ..
            }
        ));
        let air = |agl_ft: f64, gs_kts: f64| RadioFacts {
            on_ground: Some(false),
            agl_ft: Some(agl_ft),
            gs_kts: Some(gs_kts),
        };
        let ground = |gs_kts: f64| RadioFacts {
            on_ground: Some(true),
            agl_ft: Some(0.3),
            gs_kts: Some(gs_kts),
        };
        // On time: two seconds after the queue, still at 4.6 nm.
        assert_eq!(stale_reason(holds, 2.0, &air(1335.0, 172.0)), None);
        // In the flare, 93.2 s late.
        assert_eq!(
            stale_reason(holds, 93.2, &air(87.5, 149.0)),
            Some("landing")
        );
        // The same 93 s on a long final with the height still in hand: the
        // aircraft has flown almost 4 nm past the position it reports.
        assert_eq!(
            stale_reason(holds, 93.2, &air(900.0, 149.0)),
            Some("moved_on")
        );
        // Touchdown, rollout, stopped.
        assert_eq!(
            stale_reason(holds, 105.3, &ground(122.0)),
            Some("on_ground")
        );
        assert_eq!(stale_reason(holds, 123.0, &ground(45.0)), Some("on_ground"));
        assert_eq!(stale_reason(holds, 136.8, &ground(0.0)), Some("on_ground"));
        // A queue that sat with no groundspeed read still ages out.
        let blind = RadioFacts {
            on_ground: None,
            agl_ft: None,
            gs_kts: None,
        };
        assert_eq!(stale_reason(holds, 93.2, &blind), Some("max_age"));
        // Failed reads alone never drop a call made on time.
        assert_eq!(stale_reason(holds, 2.0, &blind), None);
    }

    #[test]
    fn position_reports_carry_their_condition_and_the_rest_do_not() {
        let t = |s: &str| holds_for(&CrewCall::Text(s.to_string()));
        let cs = "November eight one six November November, ";
        for report in [
            "final runway one three",
            "final",
            "downwind runway two seven",
            "turning base",
            "going around",
            "inbound for landing, runway one three",
        ] {
            assert!(
                matches!(t(&format!("{cs}{report}")), Holds::Airborne { .. }),
                "{report}"
            );
            assert!(
                matches!(t(report), Holds::Airborne { .. }),
                "no callsign: {report}"
            );
        }
        assert!(matches!(
            t(&format!("{cs}runway vacated")),
            Holds::OnGround { .. }
        ));
        assert!(matches!(
            holds_for(&CrewCall::CheckIn {
                callsign: None,
                to: "Malaga Departure".into(),
                climb_to_ft: 5000.0
            }),
            Holds::Airborne { .. }
        ));
        // Readbacks and requests keep their own rules: a readback owed
        // stays owed, including one that repeats a controller's "final".
        for other in [
            "cleared to land runway one three",
            "cleared for takeoff runway one three",
            "lining up runway one three",
            "request taxi, runway one three",
            "ready for departure, for circuits",
            "request clearance",
            "Malaga Departure on one two one decimal zero two five",
            "join final runway one three",
            "wilco, report downwind",
            "squawking seven thousand",
        ] {
            assert_eq!(t(&format!("{cs}{other}")), Holds::Always, "{other}");
        }
    }

    #[test]
    fn runway_vacated_holds_on_the_ground_only() {
        let h = holds_for(&CrewCall::Text("runway vacated".into()));
        let facts = |g: bool| RadioFacts {
            on_ground: Some(g),
            agl_ft: None,
            gs_kts: Some(10.0),
        };
        assert_eq!(stale_reason(h, 20.0, &facts(true)), None);
        assert_eq!(stale_reason(h, 20.0, &facts(false)), Some("airborne"));
    }

    #[test]
    fn a_going_around_call_is_not_killed_by_the_low_balk_it_reports() {
        // A balk can be called from under 100 ft once it climbs; the call
        // is still true there.
        let h = holds_for(&CrewCall::Text("going around".into()));
        let low = RadioFacts {
            on_ground: Some(false),
            agl_ft: Some(60.0),
            gs_kts: Some(140.0),
        };
        assert_eq!(stale_reason(h, 3.0, &low), None);
        assert_eq!(stale_reason(h, 45.0, &low), Some("max_age"));
    }

    #[test]
    fn final_is_the_end_of_the_sequence() {
        assert_eq!(next_leg(Leg::Base), Some(Leg::Final));
        assert_eq!(next_leg(Leg::Final), None);
        // Final never self-advances (autoland owns it), whatever the position.
        assert!(!pattern_advance(Leg::Final, -800.0, 0.0, &g(), L, true));
    }

    fn lxgb_rwy() -> crate::autoland::Runway {
        crate::autoland::Runway {
            icao: "LXGB".into(),
            id: "27".into(),
            thr_lat: 36.1508,
            thr_lon: -5.3385,
            course_deg: 267.5,
            length_m: 1680.0,
            surface: 1,
            dist_nm: 0.0,
            elev_ft: 11.0,
        }
    }

    #[test]
    fn gibraltar_loops_south_over_the_sea_by_runway() {
        // 09 (east): sea is to the right → right-hand (clockwise), left = false.
        assert!(!sea_side_is_left(87.5));
        assert!(!sea_side_is_left(90.0));
        // 27 (west): sea is to the left → left-hand (anticlockwise), left = true.
        assert!(sea_side_is_left(267.5));
        assert!(sea_side_is_left(270.0));
    }

    /// #80: the LXGB route's levels are the autopilot's where the airframe
    /// declares the capture. The climb to the 1,650 ft pattern puts 1,700
    /// on a dial left at 100 (the circuit's own rounding), a captured hold
    /// at the pattern owns the downwind, the base's lower waypoint moves the
    /// dial and the old hold is no longer ours, and off (or a failed read)
    /// nothing is written.
    #[test]
    fn the_authored_route_levels_on_the_autopilots_capture() {
        let route = authored_circuit(&lxgb_rwy(), &g(), 11.0, 1650.0, false).unwrap();
        let tpa = route[2].alt_msl_ft;
        let handoff = route.last().unwrap().alt_msl_ft;
        // Climbing out with a stale dial: the level goes on it.
        let v = authored_vertical(true, tpa, 900.0, Some(100.0), Some(1.0));
        assert_eq!(v.dial_ft, Some(((tpa / 100.0).round()) * 100.0));
        assert!(!v.ap_level);
        let dial = v.dial_ft.unwrap();
        // Armed, still climbing: the VS law climbs.
        assert!(!authored_vertical(true, tpa, 1200.0, Some(dial), Some(1.0)).ap_level);
        // Captured at the pattern: the hold owns the downwind.
        let v = authored_vertical(true, tpa, tpa + 30.0, Some(dial), Some(2.0));
        assert_eq!(v, AuthoredVertical { dial_ft: None, ap_level: true });
        // The base's waypoint is lower: a new dial, the old hold not ours.
        let v = authored_vertical(true, handoff, tpa, Some(dial), Some(2.0));
        assert!(v.dial_ft.is_some() && !v.ap_level);
        let low = v.dial_ft.unwrap();
        assert!(!authored_vertical(true, handoff, tpa, Some(low), Some(2.0)).ap_level);
        // Off, or a failed read: the VS law as before, the dial untouched.
        assert_eq!(
            authored_vertical(false, tpa, tpa, Some(dial), Some(2.0)),
            AuthoredVertical { dial_ft: None, ap_level: false }
        );
        assert_eq!(
            authored_vertical(true, tpa, tpa, None, Some(2.0)),
            AuthoredVertical { dial_ft: None, ap_level: false }
        );
        assert_eq!(
            authored_vertical(true, tpa, f64::NAN, Some(dial), Some(2.0)),
            AuthoredVertical { dial_ft: None, ap_level: false }
        );
    }

    #[test]
    fn authored_circuit_only_at_lxgb() {
        let mut r = lxgb_rwy();
        assert!(authored_circuit(&r, &g(), 11.0, 1074.0, true).is_some());
        r.icao = "EGLL".into();
        assert!(authored_circuit(&r, &g(), 11.0, 1074.0, true).is_none());
    }

    #[test]
    fn authored_circuit_stays_on_the_bay_side_and_ends_on_the_centreline() {
        let route = authored_circuit(&lxgb_rwy(), &g(), 11.0, 1074.0, true).unwrap();
        // Left pattern → every offset is on the left (x_m <= 0), never crossing
        // to the Rock side.
        assert!(
            route.iter().all(|w| w.x_m <= 0.1),
            "all waypoints on the picked side"
        );
        // Upwind starts on the centreline, the last waypoint returns to it for
        // the autoland handoff.
        assert!((route.first().unwrap().x_m).abs() < 0.1);
        assert!((route.last().unwrap().x_m).abs() < 0.1);
        // The downwind is held wider than the base — the "around the Rock" bulge.
        let max_out = route.iter().map(|w| w.x_m.abs()).fold(0.0, f64::max);
        assert!(
            max_out > g().width_m,
            "downwind carried wide, {max_out} vs {}",
            g().width_m
        );
        // Descends from TPA to the handoff height by the end.
        assert!(route.first().unwrap().alt_msl_ft > route.last().unwrap().alt_msl_ft);
    }

    #[test]
    fn authored_circuit_mirrors_for_a_right_pattern() {
        let route = authored_circuit(&lxgb_rwy(), &g(), 11.0, 1074.0, false).unwrap();
        assert!(
            route.iter().all(|w| w.x_m >= -0.1),
            "right pattern → offsets to the right"
        );
    }

    /// The handover must wait for a geometry autoland can actually fly.
    ///
    /// EGHI 2026-07-30: released 2,815 m off the centreline with 2,421 m to
    /// run — the offset EXCEEDED the distance, a 49 degree angle. Autoland
    /// saturated at its 35 degree cut for the entire final and crossed the
    /// threshold 188 m off the paint. Reaching `final_capture_m` means the
    /// base turn may BEGIN, not that the aircraft is on the centreline.
    #[test]
    fn the_handover_waits_until_the_centreline_is_reachable() {
        // The exact geometry that failed.
        assert!(
            !interceptable(2815.0, 2421.0),
            "offset larger than the distance to run is never flyable"
        );
        // Sign must not matter — the other side of the centreline is the same
        // problem.
        assert!(!interceptable(-2815.0, 2421.0));

        // A rollout close to the centreline with room to run is fine.
        assert!(interceptable(400.0, 2000.0));
        assert!(interceptable(-400.0, 2000.0));

        // The boundary behaves: at the bare 35 degree geometry it is refused
        // (we insist on margin), and it is accepted once the margin is there.
        let bare = 1000.0 / MAX_INTERCEPT_CUT_DEG.to_radians().tan();
        assert!(
            !interceptable(1000.0, bare * 1.05),
            "bare geometry is not enough"
        );
        assert!(interceptable(1000.0, bare * INTERCEPT_MARGIN + 1.0));

        // On the centreline anything works, including at the threshold.
        assert!(interceptable(0.0, 0.0));
    }

    /// Being ABLE to close the offset is not being lined up, and the circuit
    /// has no intercept of its own — its final leg commands the runway heading
    /// and nothing else. So the handover waits for our own turn to roll out.
    #[test]
    fn the_handover_waits_for_the_turn_to_roll_out_not_just_for_flyable_geometry() {
        // EGHI 2026-08-01, the Baron, taken straight from the blackbox: on
        // base at 290 true, runway 02 (course 20), 1,813 m off the centreline
        // with 4,092 m to run. The geometry is a 28 degree cut, so
        // `interceptable` is happy...
        let hdg_err_on_base = crate::angles::signed_diff_deg(290.0, 20.0);
        assert!(
            interceptable(1813.0, 4092.0),
            "a 28 degree cut is closeable"
        );
        // ...but the aircraft is still 90 degrees across the centreline, and
        // handing it over there is what made it approach and land at an angle.
        assert!(
            !ready_for_autoland(1813.0, 4092.0, hdg_err_on_base, BARON_KTS, None, None),
            "still turning: fly the turn, do not hand over the diagonal"
        );

        // Rolled out on the runway course with the designed 0.3-radii trim
        // left: this is the straight-in autoland is for.
        assert!(ready_for_autoland(250.0, 3500.0, 3.0, BARON_KTS, None, None));
        assert!(ready_for_autoland(-250.0, 3500.0, -3.0, BARON_KTS, None, None));
        // Just outside the rollout tolerance, still turning.
        assert!(!ready_for_autoland(250.0, 3500.0, 25.0, BARON_KTS, None, None));

        // The live failure (2026-08-01): rolled out on HEADING — eight
        // degrees, comfortably inside the tolerance — and 776 m to the side,
        // flying parallel. The heading test alone passed it, autoland flew the
        // diagonal, and the aircraft crossed the threshold still six degrees
        // off the runway, still turning. Rolled out is not established.
        assert!(
            interceptable(776.0, 3128.0),
            "the geometry was closeable, which is exactly why heading alone let it through"
        );
        assert!(
            !ready_for_autoland(776.0, 3128.0, 8.0, BARON_KTS, None, None),
            "parallel and 776 m off is not established"
        );

        // The last-ditch valve: out of room and still not aligned, autoland's
        // intercept beats a pattern leg that has nowhere left to go.
        assert!(ready_for_autoland(300.0, 1200.0, 40.0, BARON_KTS, None, None));
        // ...but never something autoland cannot fly at all, however close.
        assert!(!ready_for_autoland(2815.0, 1200.0, 0.0, BARON_KTS, None, None));

        // A failed heading read must not read as aligned — the caller passes
        // 180, the worst case, so the handover simply waits a tick.
        assert!(!ready_for_autoland(1813.0, 4092.0, 180.0, BARON_KTS, None, None));
    }

    /// Where the 776 m came from, in arithmetic. The pattern radius estimate
    /// pads the speed by 1.1 and banks a deliberately shallow 14 degrees —
    /// both of which make it BIGGER, which is safe for every dimension it
    /// sizes except this one, where a bigger capture distance starts the turn
    /// earlier and rolls out short of the centreline.
    #[test]
    fn the_final_capture_is_sized_on_the_turn_actually_flown() {
        // Southampton, the Baron: Vy 105, approach leg also 105.
        let pattern_r = turn_radius_m(105.0 * 0.514_444 * 1.1);
        assert!(
            (pattern_r - 1443.0).abs() < 20.0,
            "the pattern estimate: {pattern_r:.0}"
        );
        // A 90-degree turn closes exactly one radius of cross-track, and the
        // aircraft closed 1,048 m (1,830 off at the turn, 782 at the handoff).
        let flown_r = 1048.0;
        let old_residual = 1.3 * pattern_r - flown_r;
        assert!(
            (old_residual - 776.0).abs() < 100.0,
            "the old model predicts the live offset: {old_residual:.0} m vs 776 flown"
        );

        // Sized on the turn actually flown, the roll-out lands near enough to
        // the centreline that the final leg's intercept is trimming, not
        // rescuing — and inside what the handoff gate will accept once the
        // intercept has had a few seconds.
        let geom = Geometry::for_radii(pattern_r, final_turn_radius_m(105.0));
        let residual = geom.final_capture_m - flown_r;
        assert!(
            (0.0..=ESTABLISHED_XTK_M + 100.0).contains(&residual),
            "roll out just short of the line, not three quarters of a radius: {residual:.0} m"
        );
        // Still EARLY, never late: with an intercept, short is a gentle
        // closure and long is an S-turn back across the centreline.
        assert!(residual > 0.0, "must not turn late");

        // The pattern itself is untouched — width and downwind length keep the
        // conservative radius that stopped the LXGB figure-eights.
        let plain = Geometry::for_radius(pattern_r);
        assert_eq!(geom.width_m, plain.width_m);
        assert_eq!(geom.downwind_past_m, plain.downwind_past_m);
        assert!(geom.final_capture_m < plain.final_capture_m);
    }

    /// The final leg has to CONVERGE on the centreline. It used to command the
    /// runway course and nothing else, which makes it a leg that flies
    /// parallel to the line it is supposed to be joining — survivable only
    /// while the turn onto final was assumed to roll out ON the line, and it
    /// does not: at Southampton the turn began 1,850 m out, consumed only
    /// ~1,050, and left the aircraft 776 m to the side holding runway heading
    /// all the way to the handoff (2026-08-01).
    #[test]
    fn the_final_leg_flies_an_intercept_not_a_parallel() {
        let course = 19.4;
        let cut =
            |x: f64| crate::angles::signed_diff_deg(final_intercept_hdg(course, x), course);

        // On the line, fly the line.
        assert!(cut(0.0).abs() < 0.01);

        // Off to the RIGHT of the course, cut LEFT to close it, and the other
        // way about. Getting this backwards flies away from the runway.
        assert!(cut(776.0) < 0.0, "right of course must cut left");
        assert!(cut(-776.0) > 0.0, "left of course must cut right");

        // The live offset earns a real intercept, not a token one.
        assert!(
            cut(776.0).abs() > 15.0,
            "776 m off deserves a proper cut, got {:.1}",
            cut(776.0)
        );

        // It fades as the offset closes — no overshoot through the centreline.
        assert!(cut(300.0).abs() < cut(776.0).abs());
        assert!(cut(80.0).abs() < cut(300.0).abs());
        assert!(cut(50.0).abs() < 5.0, "nearly on: nearly no cut");

        // ...and is capped, so a wild offset still hands over something
        // autoland's own 35-degree pursuit can fly.
        assert!(cut(9000.0).abs() <= MAX_FINAL_CUT_DEG + 0.01);
        assert!(MAX_FINAL_CUT_DEG < MAX_INTERCEPT_CUT_DEG);

        // The intercept must be able to REACH the handoff gate, or the gate
        // deadlocks into the last-ditch valve. From the live roll-out (1,830 m
        // off, 3,966 m to run) a capped cut closes at sin(30) of groundspeed
        // while the along-track closes at cos(30): check the offset is gone
        // before the runway is.
        let (mut x, mut along) = (1830.0_f64, 3966.0_f64);
        let step_m = 54.0; // ~105 kt, one second
        let mut ticks = 0;
        while x > ESTABLISHED_XTK_M && along > 0.0 && ticks < 600 {
            let c = cut(x).abs().to_radians();
            x -= step_m * c.sin();
            along -= step_m * c.cos();
            ticks += 1;
        }
        assert!(
            x <= ESTABLISHED_XTK_M && along > 900.0,
            "must be established with room to spare: {x:.0} m off, {along:.0} m to run"
        );
    }

    /// The sim's heading mode does not reverse a turn in progress (LXGB
    /// 2026-09-14): the tick that captured waypoint 2 had the aircraft 4.3
    /// degrees right for it and wanted 276 for waypoint 3 with the nose on
    /// 300.5. The bug must go on the nose until the wings are level, then
    /// the short way.
    #[test]
    fn the_bug_never_steps_against_a_turn_in_progress() {
        // Banked right, wanted turn left: the bug tracks the nose.
        let b = bug_toward(302.4, 276.5, 300.5, 4.3, BUG_SLEW_PER_TICK);
        assert!((b - 300.5).abs() < 0.01, "on the nose, got {b}");
        // Rolled out: the short way to the waypoint.
        let b = bug_toward(300.5, 276.5, 300.5, 2.0, BUG_SLEW_PER_TICK);
        assert!((b - 276.5).abs() < 0.01, "left to the waypoint, got {b}");
        // Turn wanted the SAME way as the bank: untouched pursuit.
        let b = bug_toward(100.0, 150.0, 100.0, 12.0, BUG_SLEW_PER_TICK);
        assert!((b - 140.0).abs() < 0.01, "slewed toward 150, got {b}");
        // A small reversal is the AP finishing its capture, not a wrong way.
        let b = bug_toward(100.0, 95.0, 100.0, 12.0, BUG_SLEW_PER_TICK);
        assert!((b - 95.0).abs() < 0.01, "small correction passes, got {b}");
        // Across the north seam, banked left, wanted right: nose again.
        let b = bug_toward(350.0, 30.0, 355.0, -12.0, BUG_SLEW_PER_TICK);
        assert!(
            (b - 355.0).abs() < 0.01,
            "on the nose across the seam, got {b}"
        );
    }

    /// ETND 26 (2026-09-23), the SF50's crosswind: wings level on 265, the
    /// leg wants 174. The bug used to reach 174 in three ticks and the
    /// autopilot went round to the right. Now it leads the nose by 25 until
    /// the bank is established left, then goes the whole way.
    #[test]
    fn a_big_turn_from_level_is_started_the_right_way_first() {
        let mut bug = 263.4;
        let mut heading = 264.7;
        let mut phi: f64 = 0.3;
        let mut max_lead: f64 = 0.0;
        for _ in 0..30 {
            bug = bug_toward(bug, 173.7, heading, phi, BUG_SLEW_PER_TICK);
            if (bug - 173.7).abs() < 0.01 {
                break; // committed and handed to the leg: the capture is the AP's
            }
            let lead = crate::angles::signed_diff_deg(bug, heading);
            assert!(lead <= 0.5, "never right of the nose: {lead:.1}");
            if phi > -TURN_IN_PROGRESS_BANK_DEG {
                max_lead = max_lead.max(-lead);
                assert!(-lead <= TURN_ENTRY_LEAD_DEG + 0.01, "entry lead {lead:.1}");
            }
            // A plant that banks toward the bug and turns with the bank.
            phi = (phi + (lead.clamp(-25.0, 25.0) - phi) * 0.3).clamp(-25.0, 25.0);
            heading = (heading + phi * 0.25).rem_euclid(360.0);
        }
        assert!((max_lead - TURN_ENTRY_LEAD_DEG).abs() < 0.5, "{max_lead}");
        assert!(
            (bug - 173.7).abs() < 0.01,
            "committed, the bug goes to the leg: {bug:.1}"
        );
        // Right turns the same, across the north seam.
        let b = bug_toward(355.0, 90.0, 355.0, 0.0, BUG_SLEW_PER_TICK);
        assert!((b - 20.0).abs() < 0.01, "25 right of the nose: {b}");
    }

    /// LIPK (2026-09-25), the start of the leg to ESFOK: nose 37, wanted
    /// 141, banked 2 to 5 degrees LEFT. The bug alternated between the nose
    /// (bank past 3) and 25 right of it (bank back under 3) every tick for
    /// 50 s. On the nose, it now stays on the nose until the wings are level.
    #[test]
    fn a_wrong_way_bank_is_rolled_out_to_level_before_the_lead() {
        // Past 3 degrees the other way: the nose.
        let b = bug_toward(62.0, 141.0, 37.0, -3.8, BUG_SLEW_PER_TICK);
        assert!((b - 37.0).abs() < 0.01, "{b}");
        // Rolling out through 2.5: still the nose, not the lead.
        let b = bug_toward(37.0, 141.0, 36.0, -2.5, BUG_SLEW_PER_TICK);
        assert!((b - 36.0).abs() < 0.01, "held on the nose at -2.5: {b}");
        // Level: the lead, the right way.
        let b = bug_toward(36.0, 141.0, 35.5, -0.6, BUG_SLEW_PER_TICK);
        assert!((b - 60.5).abs() < 0.01, "25 right of the nose: {b}");
        // With the lead ON, a wobble under 3 degrees does not snatch it back.
        let b = bug_toward(60.5, 141.0, 35.5, -2.0, BUG_SLEW_PER_TICK);
        assert!((b - 60.5).abs() < 0.01, "lead kept: {b}");
    }

    /// The takeoff handed off 1.8 km past the climb-out point; the follower
    /// must join at the first waypoint still ahead, never steer back.
    #[test]
    fn the_join_skips_waypoints_already_behind() {
        let route = authored_circuit(&lxgb_rwy(), &g(), 11.0, 1650.0, false).unwrap();
        let up = route[0].a_m;
        // Over the runway, climbing: the climb-out point is ahead.
        assert_eq!(join_waypoint(&route, up - 1500.0, 0.0), 0);
        // The sortie: 3,824 m along, 95 m right — past waypoint 0, short of
        // the crosswind turn's end.
        assert_eq!(join_waypoint(&route, up + 1798.0, 95.0), 1);
        // Halfway down the downwind: its end.
        assert_eq!(
            join_waypoint(&route, route[1].a_m * 0.5, route[2].x_m + 50.0),
            2
        );
        // Beyond the last waypoint along its own leg: the last one.
        let (p, q) = (route[route.len() - 2], route[route.len() - 1]);
        let (da, dx) = (q.a_m - p.a_m, q.x_m - p.x_m);
        let len = (da * da + dx * dx).sqrt();
        let idx = join_waypoint(&route, q.a_m + 100.0 * da / len, q.x_m + 100.0 * dx / len);
        assert_eq!(idx, route.len() - 1);
    }

    /// Sampling the legs, not the waypoints: every leg is covered end to end
    /// at no more than LEG_SAMPLE_M spacing, and the climb-out is left out.
    #[test]
    fn the_legs_are_sampled_end_to_end() {
        let route = authored_circuit(&lxgb_rwy(), &g(), 11.0, 1650.0, false).unwrap();
        let samples = leg_samples(&route);
        assert!(samples.iter().all(|s| s.0 >= 1), "leg 0 is the takeoff's");
        for i in 1..route.len() {
            let leg: Vec<_> = samples.iter().filter(|s| s.0 == i).collect();
            let first = leg.first().unwrap();
            let last = leg.last().unwrap();
            assert!(
                (first.1 - route[i - 1].a_m).abs() < 0.01
                    && (first.2 - route[i - 1].x_m).abs() < 0.01
            );
            assert!((last.1 - route[i].a_m).abs() < 0.01 && (last.2 - route[i].x_m).abs() < 0.01);
            for w in leg.windows(2) {
                let d = ((w[1].1 - w[0].1).powi(2) + (w[1].2 - w[0].2).powi(2)).sqrt();
                assert!(d <= LEG_SAMPLE_M + 0.01, "leg {i} sample spacing {d:.0} m");
            }
        }
    }

    /// The Rock circuit's downwind crosses the ridge (~1,130 ft) at a
    /// 1,650 ft pattern altitude: 520 ft, under the 600 the rule demands. The
    /// pattern altitude rises to clear it; the handoff waypoints stay put.
    #[test]
    fn the_pattern_altitude_rises_over_the_ridge_the_downwind_crosses() {
        let route = authored_circuit(&lxgb_rwy(), &g(), 11.0, 1650.0, false).unwrap();
        let mut legs = vec![None; route.len()];
        legs[2] = Some(1130.0);
        // A generous climb gradient: this test is about the raise alone.
        let (raised, tpa) = raise_for_terrain(&route, 1650.0, &legs, 11.0, 1.0).unwrap();
        assert!((tpa - 1730.0).abs() < 0.01, "tpa {tpa}");
        for w in &raised[..4] {
            assert!(
                (w.alt_msl_ft - 1730.0).abs() < 0.01,
                "pattern waypoints raised"
            );
        }
        assert!(
            (raised[4].alt_msl_ft - 711.0).abs() < 0.01,
            "handoff height untouched"
        );
        assert!((raised[5].alt_msl_ft - 711.0).abs() < 0.01);
        // Sea under every leg: nothing changes.
        let (same, tpa) =
            raise_for_terrain(&route, 1650.0, &vec![None; route.len()], 11.0, 1.0).unwrap();
        assert!((tpa - 1650.0).abs() < 0.01);
        assert!(same
            .iter()
            .zip(&route)
            .all(|(a, b)| (a.alt_msl_ft - b.alt_msl_ft).abs() < 0.01));
        // Terrain under a DESCENDING leg cannot be out-climbed: refused.
        let mut legs = vec![None; route.len()];
        legs[4] = Some(400.0);
        assert!(raise_for_terrain(&route, 1650.0, &legs, 11.0, 1.0).is_err());
        // A raise past the cap is refused, not flown.
        let mut legs = vec![None; route.len()];
        legs[2] = Some(2200.0);
        assert!(raise_for_terrain(&route, 1650.0, &legs, 11.0, 1.0).is_err());
    }

    /// Runway 09, 2026-09-15: the raised pattern altitude cleared the ridge
    /// under the downwind on paper, and the aircraft turned onto that leg at
    /// 1,130 ft, still climbing, and crossed the ground 454 ft up. A leg is
    /// checked against the altitude the climb can have reached at its START,
    /// not the altitude the route will eventually hold.
    #[test]
    fn a_leg_entered_below_its_terrain_in_the_climb_is_refused() {
        let route = authored_circuit(&lxgb_rwy(), &g(), 11.0, 2108.0, false).unwrap();
        let grad = authored_climb_gradient(75.0);
        // 525 fpm credited at 75 kt: about 0.23 ft per metre flown.
        assert!((0.2..0.26).contains(&grad), "gradient {grad}");
        let reach = climb_reach_ft(&route, 11.0, grad);
        // The reach grows along the route and starts from the field.
        assert!(reach.windows(2).all(|w| w[1] > w[0]));
        assert!(
            reach[0] > 11.0 && reach[0] < 1000.0,
            "reach at wpt 0 {}",
            reach[0]
        );
        // A 1,130 ft ridge under the downwind needs 1,730 ft; the climb
        // reaches less than that by the end of the crosswind, so the route
        // is refused however high the pattern altitude.
        let mut legs = vec![None; route.len()];
        legs[2] = Some(1130.0);
        assert!(reach[1] < 1730.0, "reach at wpt 1 {}", reach[1]);
        let why = raise_for_terrain(&route, 2108.0, &legs, 11.0, grad).unwrap_err();
        assert!(why.contains("in the climb"), "{why}");
        // The same ridge under the LAST pattern-altitude leg is out-climbed
        // by then (the reach at its start is past the TPA) and flown.
        let mut legs = vec![None; route.len()];
        legs[3] = Some(1098.0);
        assert!(reach[2] > 2108.0, "reach at wpt 2 {}", reach[2]);
        assert!(raise_for_terrain(&route, 2108.0, &legs, 11.0, grad).is_ok());
        // A shallow climber (a heavy twin on one engine, say) is refused
        // even there.
        assert!(raise_for_terrain(&route, 2108.0, &legs, 11.0, 0.05).is_err());
    }

    /// The loop goes AROUND the Rock: the downwind runs south of Europa
    /// Point on either side, whatever the airframe's pattern width; the base
    /// and the final handoff stay where the rectangle's would be.
    #[test]
    fn the_rock_circuit_downwind_runs_south_of_europa_point() {
        for left in [true, false] {
            let route = authored_circuit(&lxgb_rwy(), &g(), 11.0, 1074.0, left).unwrap();
            for w in &route[1..4] {
                assert!(
                    w.x_m.abs() >= LXGB_ROCK_CLEAR_M - 0.01,
                    "downwind {:.0} m out, left {left}",
                    w.x_m
                );
            }
            assert!(
                route[4].x_m.abs() < LXGB_ROCK_CLEAR_M * 0.5,
                "base inside the loop"
            );
            assert!(route[5].x_m.abs() < 0.1);
            // The crosswind (leg 1) is the climb's leg: long enough at a
            // 172's gradient to reach a raised pattern altitude over water.
            let reach = climb_reach_ft(&route, 11.0, authored_climb_gradient(75.0));
            assert!(
                reach[1] > 1600.0,
                "reach at the end of the crosswind {}",
                reach[1]
            );
        }
    }

    #[test]
    fn the_rectangle_polyline_is_dense_enough_for_a_ridge() {
        let pts = densify(&[(0.0, 0.0), (1000.0, 0.0), (1000.0, 700.0)], 150.0);
        assert_eq!(pts[0], (0.0, 0.0));
        assert_eq!(*pts.last().unwrap(), (1000.0, 700.0));
        assert!(pts.contains(&(1000.0, 0.0)), "corners kept");
        for w in pts.windows(2) {
            let d = ((w[1].0 - w[0].0).powi(2) + (w[1].1 - w[0].1).powi(2)).sqrt();
            assert!(d <= 150.01, "spacing {d}");
        }
        let poly = pattern_polyline(&lxgb_rwy(), &g(), 1680.0, false);
        assert!(
            poly.len() > 60,
            "a 12 km rectangle needs more than 20 samples: {}",
            poly.len()
        );
    }

    /// The orbit: the distance to waypoint 3 went 2,853 → 2,292 → 3,300 →
    /// 3,141 over five minutes. A watchdog on "no new low for 45 s" fires
    /// once per lap; a leg genuinely closing never trips it.
    #[test]
    fn the_watchdog_fires_on_an_orbit_and_never_on_a_closing_leg() {
        let mut c = Closure::new(2908.0, 0.0);
        assert!(!c.stalled(2853.0, 1.3));
        assert!(!c.stalled(2292.0, 22.0)); // the near side of the lap
        for t in 23..67 {
            let d = 2300.0 + (t as f64 - 22.0) * 25.0; // opening back out
            assert!(!c.stalled(d, t as f64), "not yet at t={t}");
        }
        assert!(c.stalled(3300.0, 68.0), "no new low for 45 s: stalled");
        assert!(
            !c.stalled(3290.0, 69.0),
            "fires once, then the clock restarts"
        );
        // A leg closing at 35 m/s never sets the flag.
        let mut c = Closure::new(2500.0, 0.0);
        for t in 1..70 {
            assert!(!c.stalled(2500.0 - 35.0 * t as f64, t as f64));
        }
    }

    /// LIPK FI30 (2026-09-25): a 141 degree turn onto the leg opened the
    /// distance for 45 s. With the turn's grace it is not a stall; a leg
    /// that never closes still is.
    #[test]
    fn a_leg_that_begins_with_a_turn_is_given_the_turn() {
        let mut c = Closure::with_grace(9470.0, 0.0, 70.0);
        for t in 1..60 {
            assert!(!c.stalled(9470.0 + 10.0 * t as f64, t as f64), "t={t}");
        }
        for t in 60..115 {
            let d = 10_000.0 - 45.0 * (t - 60) as f64;
            assert!(!c.stalled(d, t as f64), "closing t={t}");
        }
        let mut c = Closure::with_grace(9470.0, 0.0, 70.0);
        let fired = (1..200).any(|t| c.stalled(9470.0 + 10.0 * t as f64, t as f64));
        assert!(fired, "a leg that never closes is still caught");
    }
