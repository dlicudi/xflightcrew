    /// LEMG 31 (2026-09-28, issue #141): the rollout at 16.1 kt (state t
    /// 1790585617.7), then 0.5 s later on 12/30's centreline at 0.1 kt after
    /// the sortie runner's reload. The drive then read the 1,293 m as a join
    /// 1,033 m off its line and a stand drive 492 m off route.
    #[test]
    fn a_jump_in_position_is_a_relocation_never_a_movement() {
        let rollout = (36.67410120039185, -4.49797079757389);
        let reloaded = (36.68532005, -4.49417);
        let jump = super::relocation_jump_m(rollout, reloaded, 0.5).expect("a relocation");
        assert!((1250.0..1350.0).contains(&jump), "{jump:.0} m");
        // The second reload, rwy 13's threshold 6 s later, is one too.
        assert!(
            super::relocation_jump_m(reloaded, (36.68428873856669, -4.512249603370898), 6.0)
                .is_some()
        );
        // Real driving never is: the rollout's own 1.5 s at 37 kt (state
        // t 1790585614.3 -> .8), a 25 kt taxi over a 3 s read gap, standing still.
        assert!(super::relocation_jump_m(
            (36.67374691174619, -4.497480866034466),
            (36.67397931820287, -4.497803976111265),
            1.5
        )
        .is_none());
        let three_s_at_25_kts = (36.6741 + 38.6 / 111_320.0, -4.4980);
        assert!(super::relocation_jump_m((36.6741, -4.4980), three_s_at_25_kts, 3.0).is_none());
        assert!(super::relocation_jump_m(rollout, rollout, 0.0).is_none());
    }

    #[test]
    fn a_stuck_taxi_says_what_the_log_can_show() {
        assert!(super::stagnant_line(true, true).contains("grass"));
        assert!(super::stagnant_line(false, true).contains("hard over"));
        assert!(!super::stagnant_line(false, false).contains("nose"));
    }

    #[test]
    fn runway_spans_merge_and_the_light_leads_the_crossing() {
        use super::{on_runway_span, runway_spans};
        // Legs: taxiway 0-100, runway 100-140, runway 140-160, taxiway 160-300.
        let seg = vec![
            ("A".to_string(), false),
            ("09/27".to_string(), true),
            ("09/27".to_string(), true),
            ("B".to_string(), false),
        ];
        let cum = [0.0, 100.0, 140.0, 160.0, 300.0];
        let spans = runway_spans(&seg, &cum);
        assert_eq!(
            spans,
            vec![(100.0, 160.0)],
            "contiguous runway legs are one span"
        );
        assert!(!on_runway_span(&spans, 50.0), "well before: off");
        assert!(on_runway_span(&spans, 65.0), "40 m lead: on");
        assert!(on_runway_span(&spans, 150.0), "on the runway: on");
        assert!(!on_runway_span(&spans, 161.0), "clear: off");
        assert!(
            runway_spans(&[], &cum).is_empty(),
            "a route with no flags has no spans"
        );
    }

    use super::*;

    const LAT0: f64 = 36.0;
    const LON0: f64 = -5.0;
    /// Lat/lon from meters east/north of the origin — the inverse of
    /// [`Geo::to_m`], so routes read like a taxi diagram.
    fn ll(x_m: f64, y_m: f64) -> (f64, f64) {
        (
            LAT0 + y_m / 111_320.0,
            LON0 + x_m / (111_320.0 * LAT0.to_radians().cos()),
        )
    }

    /// East 200 m, left turn north, 300 m to the holding point — the guide's
    /// test diagram, minus the runway crossing.
    fn watch() -> TaxiWatch {
        TaxiWatch {
            path: vec![ll(0.0, 0.0), ll(200.0, 0.0), ll(200.0, 300.0)],
            via: "Whiskey, November".into(),
            seg: vec![("W".into(), false), ("N".into(), false)],
            holding_point: "N1".into(),
            lead_on: Vec::new(),
        }
    }

    /// #190's route as cleared from the LEMG gate (replayed from the install
    /// with `nav::plan_to_runway`, today's planner): the stand, 81.6 m back
    /// on 155.3 to the taxiway node, then the taxiway on 222.2 for 250 m.
    fn lemg_190_route() -> Vec<(f64, f64)> {
        let node: (f64, f64) = (36.6726900, -4.4890590);
        let m_lon = 111_320.0 * node.0.to_radians().cos();
        let (s, c) = (222.2_f64.to_radians().sin(), 222.2_f64.to_radians().cos());
        let mut path = vec![(36.67335705897913, -4.489441674810065), node];
        for k in 1..=50 {
            let d = 5.0 * k as f64;
            path.push((node.0 + d * c / 111_320.0, node.1 + d * s / m_lon));
        }
        path
    }

    /// #190, LEMG (iMac 2026-09-29, sortie 1790712526, a Citation X): the tug
    /// let go 51.5 m down the lane, nose 327.8 (`state` at the engage), and
    /// the join aimed down the stand's own lane 149 degrees left (`join`
    /// err -149.05, lateral 6.94 m, remaining 3558.8 of 3614.9). The planned
    /// turn goes left onto the taxiway ahead, meeting it within 45 degrees,
    /// and the drive's first aim after it sits ahead of the nose.
    #[test]
    fn a_join_behind_the_wing_line_is_a_planned_turn_onto_the_taxiway() {
        let (lat, lon, psi) = (36.6729659291503, -4.489131668721906, 327.8);
        let path = lemg_190_route();
        let w = TaxiWatch {
            path: path.clone(),
            via: String::new(),
            seg: vec![(String::new(), false); path.len() - 1],
            holding_point: "HW1".into(),
            lead_on: Vec::new(),
        };
        let route = Route::build(&w).expect("a route");
        // The recorded join, replayed: the plain projection onto the lane,
        // the aim 16 m on, well behind the wing line.
        let (progress, off) = route.fix_signed_facing(lat, lon, psi).expect("a fix");
        let err = route.heading_err_deg(lat, lon, psi, progress, 16.0);
        // 56 m down the route as flown (3558.8 of 3614.9), 51 m on
        // today's replay: the lane either way.
        assert!(progress > 45.0 && progress < 60.0, "{progress}");
        assert!((off.abs() - 6.9).abs() < 1.0, "{off}");
        assert!(err < -130.0, "{err}");
        assert!(join_turn_applies(err, false, Some(16_000.0), Some(false), false));
        let t = join_turn(&route.path, lat, lon, psi, join_turn_radius_m(Some(16_000.0)), |_, _| None)
            .expect("a turn fits on an open apron");
        assert_eq!(t.dir, -1.0, "left, the short way onto 222: {t:?}");
        // The taxiway lies 25 m behind the right wing and a left turn swings
        // away from it, so the turn goes past its heading and meets it at
        // the intercept: 150 degrees of arc, 97 m in all.
        assert!(t.turn_deg > 106.0 && t.turn_deg < 170.0, "{t:?}");
        assert!(t.lead_m < 120.0, "{t:?}");
        assert!(t.join_m > 81.6, "onto the taxiway, not down the lane: {t:?}");
        assert!(t.intercept_deg <= JOIN_INTERCEPT_DEG, "{t:?}");
        assert_eq!(t.lead[0], (lat, lon), "the turn starts where we stand");
        let rebuilt = route.with_lead(&t.lead, t.join_m).expect("drivable");
        let (p2, off2) = rebuilt.fix_signed(lat, lon).expect("a fix");
        assert!(p2 < 1.0 && off2.abs() < 0.5, "{p2} {off2}");
        let err2 = rebuilt.heading_err_deg(lat, lon, psi, p2, 16.0);
        assert!(err2 < 0.0 && err2 > -60.0, "the aim is round the turn ahead: {err2}");
        assert!(rebuilt.total < route.total, "the lane behind is not driven");
    }

    /// No planned turn stays on the pavement: none, and the drive refuses.
    /// Only drawn pavement refuses (a field without it is no verdict).
    #[test]
    fn a_join_turn_off_the_pavement_is_refused() {
        let (lat, lon, psi) = (36.6729659291503, -4.489131668721906, 327.8);
        let path = lemg_190_route();
        assert!(join_turn(&path, lat, lon, psi, 15.0, |_, _| Some(false)).is_none());
        assert!(join_turn(&path, lat, lon, psi, 15.0, |_, _| Some(true)).is_some());
        assert!(!JOIN_REFUSED_LINE.contains('\u{2014}') && !JOIN_REFUSED_LINE.contains('\u{2013}'));
    }

    /// Who gets a planned join: a heavy that does not pivot, off a runway,
    /// with the route behind the wing line. The SR22 and the light singles
    /// keep the pursuit's own turn; a line-up or turn-round plans its own.
    #[test]
    fn a_planned_join_is_for_a_heavy_that_does_not_pivot() {
        assert!(join_turn_applies(-149.0, false, Some(16_000.0), Some(false), false));
        assert!(join_turn_applies(149.0, false, None, None, false), "unread is heavy");
        assert!(!join_turn_applies(-60.0, false, Some(16_000.0), None, false), "a taxi turn");
        assert!(!join_turn_applies(-149.0, true, Some(16_000.0), None, false), "pivots");
        assert!(!join_turn_applies(-149.0, false, Some(1_111.0), None, false), "light");
        assert!(!join_turn_applies(-149.0, false, Some(79_000.0), Some(true), false), "runway");
        assert!(!join_turn_applies(-149.0, false, Some(79_000.0), None, true), "own geometry");
        assert_eq!(join_turn_radius_m(Some(79_000.0)), 15.0);
        assert_eq!(join_turn_radius_m(Some(230_000.0)), 25.0);
    }

    /// The same join over LEMG's drawn pavement, read from the install.
    /// `cargo test --release -- --ignored live_lemg_190_join_turn --nocapture`
    #[test]
    #[ignore] // needs the real X-Plane install
    fn live_lemg_190_join_turn_stays_on_the_pavement() {
        let apt = &crate::config::xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
        let Ok(Some(net)) = atc::nav::find_taxi_net(apt, "LEMG") else {
            return;
        };
        let (lat, lon, psi) = (36.6729659291503, -4.489131668721906, 327.8);
        let t = join_turn(&lemg_190_route(), lat, lon, psi, 15.0, |a, b| net.on_pavement(a, b))
            .expect("the apron has room for the turn");
        println!("{t:?}");
        assert_eq!(t.dir, -1.0);
    }

    /// LSZS 03 (SR22, 2026-09-24): the second line-up drive joined 27.4 m
    /// off its route, never captured, and ran the projection to the end:
    /// its last ticks read 0.4 m to go and -27.44 m off the line, and it
    /// declared the holding point. Full auto found the runway 34 degrees off
    /// and gave up. An arrival is ON the line; and full auto's own check
    /// for a hold reached by hand (a straight 60 m from the route's end,
    /// which a stop 27 m beside it passed) is along the route and on it.
    #[test]
    fn the_hold_is_reached_on_the_line_not_beside_it() {
        assert!(!arrived_on_line(-27.44), "the recorded stop is no arrival");
        assert!(!arrived_on_line(27.44));
        // LEMG 13's hold (2026-09-26) ended 0.05 m off: an arrival.
        assert!(arrived_on_line(0.05));
        let r = Route::build(&watch()).expect("route");
        // 27 m beside the hold, level with it: 27 m from the end in a
        // straight line, which the old check took for the hold.
        let (lat, lon) = ll(173.0, 297.0);
        let (to_go, lateral) = route_end_fix(&r, lat, lon).expect("a fix");
        assert!((lateral - 27.0).abs() < 1.0, "{lateral}");
        assert!(!at_the_hold(to_go, lateral), "{to_go} to go, {lateral} off");
        // Taxied by hand to 40 m short, on the paint: at the hold.
        let (lat, lon) = ll(200.5, 260.0);
        let (to_go, lateral) = route_end_fix(&r, lat, lon).expect("a fix");
        assert!(at_the_hold(to_go, lateral), "{to_go} to go, {lateral} off");
        // On the paint but 200 m short: not there yet.
        let (lat, lon) = ll(200.0, 100.0);
        let (to_go, lateral) = route_end_fix(&r, lat, lon).expect("a fix");
        assert!(!at_the_hold(to_go, lateral), "{to_go} to go");
        assert!(off_line_stop_line(-27.44).contains("27 metres off the line"));
    }

    /// The route the traffic gives way to is the drive's route still AHEAD
    /// of us, from where we are on it, and none once the drive has ended.
    #[test]
    fn the_route_ahead_starts_where_we_are_and_dies_with_the_drive() {
        let r = Route::build(&watch()).expect("route");
        let way = r.path_ahead(250.0);
        assert_eq!(way.len(), 2, "{way:?}");
        let (a, b) = (ll(200.0, 50.0), ll(200.0, 300.0));
        assert!((way[0].0 - a.0).abs() < 1e-7 && (way[0].1 - a.1).abs() < 1e-7, "{way:?}");
        assert_eq!(way[1], b);
        assert_eq!(r.path_ahead(-5.0).len(), 3, "joining: the whole route");
        assert!(r.path_ahead(1.0e6).is_empty(), "past the end: none");
        let published = RouteAheadGuard;
        published.publish(way);
        assert_eq!(ROUTE_AHEAD.lock().unwrap().len(), 2);
        drop(published);
        assert!(ROUTE_AHEAD.lock().unwrap().is_empty(), "emptied as the drive ends");
    }

    #[test]
    fn capped_aim_drives_the_final_corner_instead_of_chording_it() {
        // A short L: 60 m east, then a left corner, 18 m north to the stop —
        // the hold-short-after-a-corner shape (LXGB Echo). Aircraft 5 m
        // before the corner, heading east along the paint.
        let watch = TaxiWatch {
            path: vec![ll(0.0, 0.0), ll(60.0, 0.0), ll(60.0, 18.0)],
            via: String::new(),
            seg: vec![(String::new(), false), (String::new(), false)],
            holding_point: String::new(),
            lead_on: Vec::new(),
        };
        let r = Route::build(&watch).expect("route");
        let (lat, lon) = ll(55.0, 0.0);
        let (progress, lateral) = r.fix(lat, lon).expect("on the route");
        assert!(lateral < 1.0);
        // The old 16 m minimum aim reaches PAST the corner: the aim point is
        // on the northbound leg, so the error commands an early cut inside
        // the paint. The capped aim (remaining*0.6 min 6) stays on the
        // eastbound leg: drive straight to the corner first.
        let err_chord = r.heading_err_deg(lat, lon, 90.0, progress, 16.0);
        let remaining = r.remaining(progress);
        let capped = lookahead_m(3.0).min((remaining * 0.6).max(6.0));
        assert!(capped < 16.0, "the cap engages near the stop: {capped}");
        let err_capped = r.heading_err_deg(lat, lon, 90.0, progress, capped);
        assert!(
            err_capped.abs() < err_chord.abs() - 5.0,
            "capped aim hugs the paint: capped {err_capped:.1} vs chord {err_chord:.1}"
        );
    }

    #[test]
    fn steering_is_proportional_and_bounded() {
        assert_eq!(steer_cmd_with(0.0, STEER_FULL_DEG), 0.0);
        assert!(
            steer_cmd_with(10.0, STEER_FULL_DEG) > 0.0
                && steer_cmd_with(10.0, STEER_FULL_DEG) < 1.0,
            "right of course, steer right"
        );
        assert!(steer_cmd_with(-10.0, STEER_FULL_DEG) < 0.0);
        assert_eq!(
            steer_cmd_with(90.0, STEER_FULL_DEG),
            1.0,
            "full deflection is the cap"
        );
        assert_eq!(steer_cmd_with(-170.0, STEER_FULL_DEG), -1.0);
    }

    #[test]
    fn pure_pursuit_steers_back_to_the_centreline() {
        let r = Route::build(&watch()).expect("route");
        // On the first leg (eastbound, true bearing 090), 8 m LEFT of the
        // centreline, heading exactly east: the aim point is on the route
        // ahead, so the error must steer RIGHT (positive), back toward it.
        let (lat, lon) = ll(50.0, 8.0);
        let (progress, lateral) = r.fix(lat, lon).expect("fix");
        assert!((lateral - 8.0).abs() < 1.0, "{lateral}");
        let err = r.heading_err_deg(lat, lon, 90.0, progress, lookahead_m(8.0));
        assert!(err > 2.0, "must steer back right, got {err}");
        // Mirror: 8 m right of the line steers left.
        let (lat, lon) = ll(50.0, -8.0);
        let (progress, _) = r.fix(lat, lon).expect("fix");
        let err = r.heading_err_deg(lat, lon, 90.0, progress, lookahead_m(8.0));
        assert!(err < -2.0, "must steer back left, got {err}");
        // On the line, on heading: hands still.
        let (lat, lon) = ll(50.0, 0.0);
        let (progress, _) = r.fix(lat, lon).expect("fix");
        let err = r.heading_err_deg(lat, lon, 90.0, progress, lookahead_m(8.0));
        assert!(err.abs() < 1.0, "{err}");
    }

    #[test]
    fn the_turn_is_anticipated_through_the_aim_point() {
        let r = Route::build(&watch()).expect("route");
        // 10 m short of the left turn at 10 knots: the aim point is already
        // around the corner, so the command leads the turn.
        let (lat, lon) = ll(190.0, 0.0);
        let (progress, _) = r.fix(lat, lon).expect("fix");
        let err = r.heading_err_deg(lat, lon, 90.0, progress, lookahead_m(10.0));
        assert!(
            err < -10.0,
            "aim point should pull left into the turn, got {err}"
        );
    }

    #[test]
    fn schedule_slows_for_bends_and_ramps_to_the_stop() {
        let r = Route::build(&watch()).expect("route");
        // Mid first leg: cruise pace.
        assert_eq!(schedule_kts(10.0, &r, 50.0, 10.0), 10.0);
        // Approaching the 90° corner at 200 m: capped to the turn speed.
        let near_turn = schedule_kts(10.0, &r, 170.0, 10.0);
        assert_eq!(near_turn, TURN_KTS, "{near_turn}");
        // Mid-turn (just past the vertex): still capped.
        assert_eq!(schedule_kts(10.0, &r, 210.0, 6.0), TURN_KTS);
        // Well past it on the straight: back to pace.
        assert_eq!(schedule_kts(10.0, &r, 300.0, 10.0), 10.0);
        // 20 m from the stop point: ramping down, still moving.
        let ramp = schedule_kts(10.0, &r, r.total - STOP_SHORT_M - 20.0, 8.0);
        assert!(ramp > 2.0 && ramp < 10.0, "{ramp}");
        // At the stop point: zero.
        assert_eq!(schedule_kts(10.0, &r, r.total - STOP_SHORT_M, 3.0), 0.0);
    }

    /// LEMG's Alpha (2026-09-26, sortie 1790441293, a 737): an 80-degree
    /// corner drawn at about a 9 m radius (the route asked 38 deg/s at
    /// 11.5 kt), entered at 11.5 kt against the 10 kt turn cap that arrived
    /// 30 m out; full lock gave 14 to 22 deg/s and it swung 5.8 m wide. With
    /// the 737's yaw rate declared the corner is taken at the rate times the
    /// radius, and the approach to it is shaped from well out.
    #[test]
    fn a_tight_corner_is_taken_at_the_yaw_rate_the_airframe_has() {
        let mut path = vec![ll(0.0, 0.0), ll(200.0, 0.0)];
        let (r, turn) = (9.0_f64, 80.0_f64.to_radians());
        // Left turn: centre 9 m north of the arc's start.
        for i in 1..=20 {
            let a = turn * i as f64 / 20.0;
            path.push(ll(200.0 + r * a.sin(), r - r * a.cos()));
        }
        let end = *path.last().unwrap();
        path.push((end.0 + 100.0 / 111_320.0 * turn.sin(), end.1 + 100.0 / 90_000.0 * turn.cos()));
        let watch = TaxiWatch {
            seg: vec![(String::new(), false); path.len() - 1],
            path,
            via: String::new(),
            holding_point: String::new(),
            lead_on: Vec::new(),
        };
        let profile = |yaw: Option<f64>| crate::aircraft::TaxiProfile {
            kts: Some(18.0),
            turn_kts: Some(10.0),
            sharp_kts: Some(6.0),
            max_yaw_dps: yaw,
            ..Default::default()
        };
        let mut old = Route::build(&watch).expect("route");
        old.pace = Pace::from_profile(Some(&profile(None)));
        let mut jet = Route::build(&watch).expect("route");
        jet.pace = Pace::from_profile(Some(&profile(Some(15.0))));
        // At the corner: the angle alone says the 10 kt turn pace; the yaw
        // rate says 15 deg/s round 9 m, 4.6 kt.
        assert_eq!(schedule_kts(18.0, &old, 205.0, 10.0), 10.0);
        let at = schedule_kts(18.0, &jet, 205.0, 5.0);
        assert!(at < 5.0 && at >= CORNER_MIN_KTS, "the corner's own pace: {at:.1} kt");
        // And the approach is shaped: 60 m out (recorded 10.5 kt with a 10 kt
        // target) it is coming down, and by 30 m out (where the old 10 kt cap
        // first arrived) it is within a few metres of braking of the corner.
        let out = schedule_kts(18.0, &jet, 140.0, 9.0);
        assert!(out < 10.0, "60 m out: {out:.1} kt");
        let window = schedule_kts(18.0, &jet, 172.0, 5.0);
        assert!(window < 6.5, "30 m out: {window:.1} kt");
        let far = schedule_kts(18.0, &jet, 0.0, 18.0);
        // 16.6 since #194: the corner's slow-down is planned to where its
        // curvature begins, up to 15 m before the vertex.
        assert!(far > 16.0, "a corner 200 m on barely touches the pace: {far:.1} kt");
    }

    #[test]
    fn a_linked_nose_aims_nearer_through_a_tight_bend() {
        let pace = Pace::default();
        let linked = steer_look_m(&pace, 6.0, 10.0, 100.0, false);
        let free = steer_look_m(&pace, 6.0, 10.0, 100.0, true);
        assert_eq!(linked, LINKED_BEND_AIM_FLOOR_M);
        assert_eq!(free, BEND_AIM_FLOOR_M);
        // No curvature: both retain the ordinary speed-scaled aim.
        assert_eq!(
            steer_look_m(&pace, 6.0, 0.0, 100.0, false),
            steer_look_m(&pace, 6.0, 0.0, 100.0, true)
        );
    }

    #[test]
    fn an_airframes_own_pace_drives_the_bend_caps() {
        // The 172 default is unchanged by the profile hook existing at all.
        let ga = Route::build(&watch()).expect("route");
        assert_eq!(ga.pace, Pace::default());
        assert_eq!(schedule_kts(10.0, &ga, 170.0, 10.0), TURN_KTS);

        // A jet declares its own: 18 down the straight, 10 through the bends.
        let mut jet = Route::build(&watch()).expect("route");
        jet.pace = Pace::from_profile(Some(&crate::aircraft::TaxiProfile {
            kts: Some(18.0),
            turn_kts: Some(10.0),
            sharp_kts: Some(6.0),
            ..Default::default()
        }));
        assert_eq!(jet.pace.straight, 18.0);
        assert_eq!(schedule_kts(18.0, &jet, 0.0, 18.0), 18.0, "open taxiway");
        // 150 m out the jet is already coming down toward the bend's pace.
        let shaped = schedule_kts(18.0, &jet, 50.0, 18.0);
        assert!(shaped < 18.0 && shaped > 10.0, "{shaped}");
        assert_eq!(schedule_kts(18.0, &jet, 170.0, 18.0), 10.0, "into the bend");
        // The stop is still a stop, at any pace.
        assert_eq!(schedule_kts(18.0, &jet, jet.total - STOP_SHORT_M, 3.0), 0.0);
        // Turn-arounds stay at the sharp crawl on both airframes.
        assert_eq!(jet.pace.join_cap_kts(-170.0), 6.0);
    }

    #[test]
    fn a_partial_or_absurd_taxi_profile_keeps_safe_figures() {
        // Omitted fields keep the GA defaults, field by field.
        let partial = Pace::from_profile(Some(&crate::aircraft::TaxiProfile {
            kts: Some(15.0),
            turn_kts: None,
            sharp_kts: None,
            ..Default::default()
        }));
        assert_eq!(partial.straight, 15.0);
        assert_eq!(partial.turn, TURN_KTS);
        assert_eq!(partial.sharp, SHARP_KTS);
        // A typo can't become a takeoff roll with the tower's route as a plan.
        let absurd = Pace::from_profile(Some(&crate::aircraft::TaxiProfile {
            kts: Some(180.0),
            turn_kts: Some(90.0),
            sharp_kts: Some(0.0),
            ..Default::default()
        }));
        assert_eq!(absurd.straight, 25.0);
        assert_eq!(absurd.turn, 25.0);
        assert_eq!(absurd.sharp, 2.0);
        // No profile at all is the GA pace.
        assert_eq!(Pace::from_profile(None), Pace::default());
    }

    #[test]
    fn stop_ramp_is_gentle_and_finishes() {
        assert_eq!(stop_ramp_kts(0.0), 0.0);
        assert_eq!(stop_ramp_kts(0.4), 0.0, "inside the stop zone is stopped");
        assert!(
            stop_ramp_kts(2.0) >= CREEP_KTS,
            "creep floor carries the last meters"
        );
        let (near, far) = (stop_ramp_kts(10.0), stop_ramp_kts(40.0));
        assert!(near < far, "closer is slower: {near} vs {far}");
        assert!(far < 13.0, "40 m out needs no takeoff-roll pace: {far}");
    }

    /// The schedule must be able to STOP the aircraft inside the window the
    /// arrival test is watching, or every leg overshoots its own end.
    ///
    /// Luton 2026-07-28: a 2 kt creep held to 0.5 m to go, against arrival at
    /// `remaining <= 1 m && gs < 1.5 kt`. The schedule was still commanding
    /// 2 kt through the window that wanted under 1.5, and 2 kt cannot stop in
    /// half a metre at the ramp's own 0.5 m/s^2. Three SF50 line-up legs ran
    /// past their stop points — `remaining_m` pinned at -5, still rolling,
    /// steering already faded out by `stop_straighten` so the nose drifted —
    /// and full auto rightly called the line-up a failure.
    #[test]
    fn the_stop_ramp_can_actually_stop_before_the_arrival_window() {
        // Commanded stop no later than the arrival test's own 1 m.
        const ARRIVAL_M: f64 = 1.0;
        const ARRIVAL_KTS: f64 = 1.5;
        assert_eq!(stop_ramp_kts(ARRIVAL_M), 0.0, "zero by the arrival window");
        // The creep must be slow enough to stop within the zero distance at
        // the ramp's own deceleration, before any braking lag.
        let v = CREEP_KTS / MS_TO_KTS;
        let stopping_m = v * v / (2.0 * 0.5);
        assert!(
            stopping_m < STOP_ZERO_M,
            "creep of {CREEP_KTS} kt needs {stopping_m:.2} m but only gets {STOP_ZERO_M}"
        );
        // ...and the creep must still be under the arrival test's speed, so
        // reaching the point is enough to be judged arrived.
        const {
            assert!(CREEP_KTS < ARRIVAL_KTS, "the creep must satisfy arrival");
        }
    }

    /// The yoke-heading channel is dead on a castoring-wheel jet at taxi
    /// speed (steer −0.6 for 30 s while the offset grew to 13.8 m — Luton,
    /// 2026-07-28), so the assist brakes the inside wheel. One side only,
    /// ever: the governor's pilot-brake detector reads `min(left, right)`.
    #[test]
    fn brake_steer_brakes_the_inside_wheel_only() {
        // A breath of command is a breath of brake — on a castoring wheel
        // the brake IS the steering, so only jitter stays off it entirely.
        assert_eq!(brake_steer_split(0.04, 10.0), (0.0, 0.0));
        let (_, small) = brake_steer_split(0.15, 10.0);
        assert!(small > 0.0 && small < 0.06, "gentle, not zero: {small}");
        // Near-stopped: no asymmetric lurch, whatever the command.
        assert_eq!(brake_steer_split(0.9, 2.0), (0.0, 0.0));
        // A real correction at taxi speed: inside wheel, bounded, one side.
        let (l, r) = brake_steer_split(0.6, 10.0);
        assert_eq!(l, 0.0, "nose right brakes the RIGHT wheel only");
        assert!(r > 0.05 && r <= BRAKE_STEER_MAX, "{r}");
        let (l2, r2) = brake_steer_split(-0.6, 10.0);
        assert_eq!(r2, 0.0);
        assert!((l2 - r).abs() < 1e-12, "symmetric: {l2} vs {r}");
        // Full authority arrives INSIDE the tracking band, not at a command
        // the loop can never reach: steer sat at ~0.55 for two thirds of the
        // fourth SF50 taxi drawing only 0.07 of brake, and the aircraft held
        // a one-sided 9 m offset the whole way (2026-07-29).
        assert_eq!(r, BRAKE_STEER_MAX, "saturated by {BRAKE_STEER_SAT}: {r}");
        let (_, mid) = brake_steer_split(0.55, 10.0);
        assert!(
            mid > 0.8 * BRAKE_STEER_MAX,
            "tracking-band command near max: {mid}"
        );
        assert!(brake_steer_split(1.0, 60.0).1 <= BRAKE_STEER_MAX);
        // min(left, right) is zero in every case above — the governor can
        // never read the assist as the pilot slowing the aircraft.
        for s in [-1.0, -0.5, 0.5, 1.0] {
            let (a, b) = brake_steer_split(s, 12.0);
            assert_eq!(a.min(b), 0.0);
        }
        // The SF50 declares it; the 172 (linked nosewheel) must not.
        let sf = Pace::from_profile(
            crate::aircraft::load("cirrus_vision_sf50")
                .expect("sf50")
                .taxi
                .as_ref(),
        );
        assert!(sf.brake_steer);
        let c172 = Pace::from_profile(
            crate::aircraft::load("cessna_172sp")
                .expect("172")
                .taxi
                .as_ref(),
        );
        assert!(!c172.brake_steer);
    }

    /// The schedule has to know where the aircraft IS, not just how the paint
    /// bends ahead of it — the Southampton SF50 taxi (2026-07-30).
    #[test]
    fn a_mistracking_aircraft_is_slowed_whatever_the_route_ahead_does() {
        let sf = Pace::from_profile(
            crate::aircraft::load("cirrus_vision_sf50")
                .expect("sf50")
                .taxi
                .as_ref(),
        );
        // On the centreline and pointed along it: the open-taxiway pace,
        // untouched. The cap must never be the thing that slows a good taxi.
        assert_eq!(sf.track_cap_kts(0.5, 2.0), sf.straight);
        assert_eq!(sf.track_cap_kts(TRACK_TURN_LAT_M, 0.0), sf.straight);
        // Drifting off the line, or crossing it at an angle no intercept would
        // ask for: bend pace.
        assert_eq!(sf.track_cap_kts(8.0, 5.0), sf.turn);
        assert_eq!(sf.track_cap_kts(1.0, 25.0), sf.turn);
        // The numbers off the blackbox: 13 m off with 27 deg of crossing angle,
        // where it was doing 12.7 kt on a straight last leg. Sharp pace.
        assert_eq!(sf.track_cap_kts(13.0, 27.0), sf.sharp);
        // Which side is irrelevant.
        assert_eq!(sf.track_cap_kts(1.0, -40.0), sf.sharp);
        // It can only ever slow the schedule down.
        for lat in [0.0, 5.0, 7.0, 13.0, 40.0] {
            for err in [0.0, 10.0, 30.0, 90.0] {
                assert!(sf.track_cap_kts(lat, err) <= sf.straight);
                assert!(sf.track_cap_kts(lat, err) >= sf.sharp);
            }
        }
        // A trainer holding the centreline within a wheel's width is untouched
        // too — this is not a tax on airframes that track well.
        let c172 = Pace::from_profile(
            crate::aircraft::load("cessna_172sp")
                .expect("172")
                .taxi
                .as_ref(),
        );
        assert_eq!(c172.track_cap_kts(2.0, 8.0), c172.straight);
    }

    /// A taxiing aircraft carries people. The lever may not slam.
    #[test]
    fn the_steering_lever_is_rate_limited_when_an_airframe_asks() {
        // No limit declared: unchanged, whatever the demand.
        assert_eq!(slew_steer(0.0, 1.0, None, 0.2), 1.0);
        // Limited: 0.8/s over a 0.2 s tick is 0.16 of travel, not the lot.
        let s = slew_steer(0.0, 1.0, Some(0.8), 0.2);
        assert!((s - 0.16).abs() < 1e-9, "{s}");
        // Symmetric, and it converges rather than ringing past the target.
        assert!((slew_steer(0.0, -1.0, Some(0.8), 0.2) + 0.16).abs() < 1e-9);
        let near = slew_steer(0.5, 0.52, Some(0.8), 0.2);
        assert!(
            (near - 0.52).abs() < 1e-9,
            "small moves pass through: {near}"
        );
    }

    /// The steering law is per-airframe because the PLANT is. Measured over the
    /// blackbox at 4-12 kt, yaw per unit steer is 12.8 deg/s on the 172 and
    /// 6.9 on the SF50 — the castoring nosewheel is the laggier plant, and lag
    /// wants phase lead, not gain.
    #[test]
    fn the_sf50_taxis_more_gently_than_the_airframe_the_law_was_tuned_on() {
        let sf = Pace::from_profile(
            crate::aircraft::load("cirrus_vision_sf50")
                .expect("sf50")
                .taxi
                .as_ref(),
        );
        let c172 = Pace::from_profile(
            crate::aircraft::load("cessna_172sp")
                .expect("172")
                .taxi
                .as_ref(),
        );
        // The 172 keeps exactly the feel it was tuned with.
        assert_eq!(c172.steer_full_deg, STEER_FULL_DEG);
        assert_eq!(c172.steer_lead_s, STEER_RATE_LEAD_S);
        assert_eq!(c172.steer_slew_per_s, None);
        // The SF50 moves the lever less, and leads harder.
        assert!(sf.steer_full_deg > c172.steer_full_deg, "gentler gain");
        assert!(sf.steer_lead_s > c172.steer_lead_s, "more damping");
        let slew = sf.steer_slew_per_s.expect("a rate limit");
        // Generous on purpose: a tight slew is its own limit cycle, and the
        // aircraft's own p90 lever rate over the whole taxi was 0.18/s.
        assert!(
            slew >= 0.5,
            "a tight slew re-creates the oscillation: {slew}"
        );
        // Same error, less lever — that is the thing the cabin feels.
        let err = 18.0;
        assert!(steer_cmd_with(err, sf.steer_full_deg) < steer_cmd_with(err, c172.steer_full_deg));
        // And a longer aim: ~3.5 deg/s of yaw cannot fly the 5 s aim's
        // crossings — it met the centreline 24 degrees angled and S-weaved
        // ±10 m (2026-07-29). Same offset, shallower ask.
        assert!(sf.lookahead_s > c172.lookahead_s);
        assert_eq!(c172.lookahead_s, 5.0, "the 172 keeps its proven aim");
        assert!(
            lookahead_m_with(12.0, sf.lookahead_s) > 1.5 * lookahead_m_with(12.0, 5.0),
            "the SF50 aims well past the shared default at taxi pace"
        );
    }

    /// A drive stopped from outside claims nothing. The latch is what the
    /// taxi-in arrival watcher secures on, and leaving it set is how an
    /// aircraft got parked on a taxiway (BIKF, 2026-09-20).
    #[test]
    fn halting_a_drive_never_leaves_an_arrival_behind_it() {
        use std::sync::atomic::Ordering::Relaxed;
        let _g = crate::mode::TEST_LOCK.lock();
        ACTIVE.store(true, Relaxed);
        BACKTRACKING.store(true, Relaxed);
        LAST_ARRIVED.store(true, Relaxed);
        halt();
        assert!(!ACTIVE.load(Relaxed), "the loop is told to stop");
        assert!(!BACKTRACKING.load(Relaxed), "including between its legs");
        assert!(!driving(), "and `driving` agrees");
        assert!(
            !LAST_ARRIVED.load(Relaxed),
            "a drive someone else stopped did NOT arrive"
        );
    }

    /// The BIKF stop, measured: 140 m from the nearest stand with the nose
    /// ninety degrees across its parking heading. The copilot said "At the
    /// gate" there. The test the claim now has to pass is the tug's own.
    #[test]
    fn the_bikf_taxiway_stop_is_not_a_stand() {
        use crate::pushback::on_a_stand;
        // Stand 69, the nearest, 140 m away on heading 000; we sat on 270.
        assert!(!on_a_stand(Some((140.0, 0.0)), 269.7, true));
        // Close enough but still crosswise is no stand either.
        assert!(!on_a_stand(Some((20.0, 0.0)), 269.7, true));
        // Actually on it, nose down the stand: this is a gate.
        assert!(on_a_stand(Some((20.0, 0.0)), 5.0, true));
        // A field that publishes no stands cannot be second-guessed.
        assert!(on_a_stand(None, 269.7, false));
    }

    #[test]
    fn rate_lead_checks_the_swing() {
        // Nose swinging hard through a small error on STRAIGHT paint (the
        // route asks for nothing): the lead commands countersteer before the
        // crest — this is the SR22 snake killer, and it must survive the
        // corner credit untouched.
        assert!(damped_err_deg(5.0, 40.0, 0.0, STEER_RATE_LEAD_S) < 0.0);
        // Steady nose: pure P unchanged, the 172 keeps its proven feel.
        assert_eq!(damped_err_deg(10.0, 0.0, 0.0, STEER_RATE_LEAD_S), 10.0);
        // A gentle commanded bend (~8 deg/s) keeps most of its steer.
        assert!(damped_err_deg(15.0, 8.0, 0.0, STEER_RATE_LEAD_S) > 9.0);
    }

    #[test]
    fn the_lead_does_not_fight_the_turn_the_bend_asked_for() {
        // The EIDW 737 corner, from the trace (2026-09-20): 16.6 deg of
        // pursuit error with the nose already coming round at 10 deg/s,
        // which is the rate that bend requires. The old law damped the whole
        // 10 and commanded a third less steer than the error asked for.
        let err = 16.6;
        let yaw = 10.0;
        let old = err - yaw * STEER_RATE_LEAD_S;
        let new = damped_err_deg(err, yaw, yaw, STEER_RATE_LEAD_S);
        assert_eq!(new, err, "a rate the corner asked for is credited in full");
        assert!(
            steer_cmd_with(new, STEER_FULL_DEG) > 1.4 * steer_cmd_with(old, STEER_FULL_DEG),
            "the bend gets back the steering the derating took: {old} -> {new}"
        );
        // Credit is bounded by the measured rate: a bend asking for MORE than
        // the nose is giving never becomes drive of its own.
        assert_eq!(
            damped_err_deg(err, 4.0, 30.0, STEER_RATE_LEAD_S),
            err,
            "over-credit clamps at the measured rate, it never adds command"
        );
        // And a nose swinging the way the bend does NOT go earns nothing:
        // full damping, exactly as before.
        assert_eq!(
            damped_err_deg(err, 12.0, -9.0, STEER_RATE_LEAD_S),
            err - 12.0 * STEER_RATE_LEAD_S,
            "a swing against the bend keeps the whole lead"
        );
    }

    #[test]
    fn a_bend_asks_for_a_turn_rate_and_a_straight_asks_for_none() {
        // 300 m east, then a 90-degree LEFT onto north — the watch() diagram
        // with a long enough straight to sit well clear of the corner.
        let w = TaxiWatch {
            path: vec![ll(0.0, 0.0), ll(300.0, 0.0), ll(300.0, 300.0)],
            via: String::new(),
            seg: vec![(String::new(), false), (String::new(), false)],
            holding_point: String::new(),
            lead_on: Vec::new(),
        };
        let straight = Route::build(&w).expect("route");
        // Underfoot and aim both on the straight: nothing owed.
        assert_eq!(straight.route_turn_rate_dps(50.0, 40.0, 13.0), 0.0);
        // Aim across the corner: a real rate, signed LEFT (negative).
        let through = straight.route_turn_rate_dps(280.0, 40.0, 13.0);
        assert!(
            through < -5.0,
            "the corner inside the aim asks for a left turn rate, got {through}"
        );
        // And it scales with speed: the same corner taken faster is a harder
        // turn, which is the whole point of crediting a RATE and not an angle.
        let faster = straight.route_turn_rate_dps(280.0, 40.0, 26.0);
        assert!(
            (faster - 2.0 * through).abs() < 0.01,
            "twice the speed is twice the rate: {through} -> {faster}"
        );
    }

    #[test]
    fn lookahead_scales_with_speed_within_bounds() {
        assert_eq!(lookahead_m(0.0), 16.0);
        assert!(lookahead_m(10.0) > 16.0 && lookahead_m(10.0) < 50.0);
        assert_eq!(lookahead_m(60.0), 50.0);
    }

    #[test]
    fn steering_authority_tapers_with_speed_so_a_standstill_breaks_away() {
        assert!(steer_authority(0.0) < 0.4, "parked lock must stay partial");
        assert!(steer_authority(0.0) > 0.2, "but enough to actually turn");
        assert!(
            steer_authority(1.5) > steer_authority(0.5),
            "authority builds with the roll"
        );
        assert_eq!(steer_authority(3.0), 1.0, "full lock once rolling");
        assert_eq!(steer_authority(20.0), 1.0);
    }

    #[test]
    fn a_backwards_start_is_a_join_and_steers_hard_around() {
        let r = Route::build(&watch()).expect("route");
        // On the first leg, ON the centreline, but facing WEST against an
        // eastbound route: near-180 error, full deflection, join granted.
        let (lat, lon) = ll(50.0, 0.0);
        let (progress, lateral) = r.fix(lat, lon).expect("fix");
        let err = r.heading_err_deg(lat, lon, 270.0, progress, lookahead_m(3.0));
        assert!(err.abs() > 150.0, "backwards means a big error, got {err}");
        assert_eq!(
            steer_cmd_with(err, STEER_FULL_DEG).abs(),
            1.0,
            "turn-around wants full deflection"
        );
        assert!(
            join_allowance_m(lateral, err) > 0.0,
            "wrong-facing start must join"
        );
        assert_eq!(
            Pace::default().join_cap_kts(err),
            SHARP_KTS,
            "turn-arounds happen at the sharp pace"
        );
    }

    /// #178, the recorded drive ticks (LEMG 13, sortie
    /// 1790646817-full_auto, 2026-09-29): +877.3 lat_m 8.32 at 5.6 kt, the
    /// next tick 0.3 s on the facing pick read 149.37 m (progress 145.6) and
    /// the join's guard stopped the taxi. The aircraft moved under a metre:
    /// the plain nearest stands. A pick that shrinks the offset, or grows it
    /// no faster than the aircraft moved, is still taken.
    #[test]
    fn a_join_projection_that_jumps_is_not_a_movement() {
        let moved = 5.6 * 0.514 * 0.3;
        let (fix, jumped) =
            join_fix(Some((145.56, -149.37)), Some((5.0, 8.4)), Some(8.32), moved);
        assert!(jumped);
        assert_eq!(fix, Some((5.0, 8.4)));
        assert!(OFF_ROUTE_STOP_M + join_allowance_m(0.0027, -144.98) < 149.37, "it tripped");
        // The facing pick nearer than the last tick: a real join.
        assert_eq!(
            join_fix(Some((120.0, 3.0)), Some((5.0, 8.4)), Some(8.32), moved),
            (Some((120.0, 3.0)), false)
        );
        // Grown by what we moved: taken.
        assert_eq!(
            join_fix(Some((6.0, 9.0)), Some((5.0, 8.9)), Some(8.32), moved),
            (Some((6.0, 9.0)), false)
        );
        // The first tick has nothing to jump from.
        assert_eq!(
            join_fix(Some((145.0, 60.0)), Some((5.0, 8.4)), None, 0.0),
            (Some((145.0, 60.0)), false)
        );
    }

    #[test]
    fn an_off_route_start_is_a_join_and_aims_back_at_the_route() {
        let r = Route::build(&watch()).expect("route");
        // 60 m LEFT of the first leg, heading east (parallel to it): the aim
        // point on the route pulls strongly RIGHT, back toward the pavement.
        let (lat, lon) = ll(50.0, 60.0);
        let (progress, lateral) = r.fix(lat, lon).expect("fix");
        assert!((lateral - 60.0).abs() < 2.0, "{lateral}");
        let err = r.heading_err_deg(lat, lon, 90.0, progress, lookahead_m(5.0));
        assert!(err > 20.0, "must aim back at the route, got {err}");
        let allow = join_allowance_m(lateral, err);
        assert!(
            allow >= lateral + JOIN_GROW_M - 1.0,
            "guard covers the engage distance: {allow}"
        );
    }

    /// LEMG, 2026-09-26 (sortie 1790441293, a 737): the tug never came, the
    /// push was skipped, and the drive turned the aircraft round off its
    /// stand to join a route that started behind it (err -155). The old
    /// latch fired at 14.5 m and -42 degrees, mid-swing, and the scorer
    /// judged the taxi from there: "14.5 m off the line". Recorded `drive`
    /// ticks (seconds from the engage, lat_m, err), from 17 s into the join.
    #[test]
    fn a_turn_round_join_is_judged_once_it_is_on_the_line() {
        const TICKS: &[(f64, f64, f64)] = &[
            (17.10, 13.04, -87.95),
            (17.80, 13.70, -79.77),
            (18.50, 14.28, -70.89),
            (18.90, 14.54, -64.20),
            (19.30, 14.66, -57.44),
            (19.70, 14.67, -50.92),
            (20.00, 14.54, -42.27),
            (20.40, 14.22, -33.33),
            (20.80, 13.74, -21.68),
            (21.20, 13.13, -10.28),
            (21.60, 12.10, -4.26),
            (21.90, 11.32, -4.45),
            (22.40, 10.68, -2.25),
            (22.80, 9.82, 0.27),
            (23.10, 9.12, 2.24),
            (23.50, 8.46, 3.64),
            (23.90, 7.55, 5.50),
            (24.20, 6.88, 6.54),
            (24.60, 6.29, 7.31),
            (25.00, 5.56, 8.38),
            (25.30, 4.94, 9.09),
            (25.70, 4.41, 9.44),
            (26.10, 3.74, 10.23),
            (26.50, 3.25, 10.58),
            (26.90, 2.81, 10.69),
            (27.30, 2.36, 10.73),
            (27.60, 1.99, 10.53),
            (28.10, 1.52, 10.23),
            (28.50, 1.12, 9.60),
            (28.80, 0.82, 8.92),
            (29.20, 0.50, 8.02),
            (29.60, 0.20, 7.35),
            (30.00, -0.08, 6.52),
            (30.40, -0.32, 5.75),
            (30.80, -0.50, 5.06),
            (31.10, -0.70, 2.29),
            (31.60, -0.83, 5.63),
            (32.00, -0.91, 4.78),
            (32.40, -0.94, 3.94),
            (32.80, -0.93, 3.14),
            (33.20, -0.91, 2.50),
            (33.50, -0.88, -2.35),
            (33.90, -0.88, -8.82),
            (34.30, -0.98, -13.92),
            (34.60, -1.24, -15.82),
            (35.00, -1.71, -17.25),
            (35.40, -2.45, -19.14),
            (35.80, -2.88, -18.33),
            (36.20, -2.19, -12.96),
            (36.60, -1.67, -10.68),
            (37.00, -1.45, -8.86),
            (37.40, -0.59, -9.58),
            (37.80, 0.51, -7.09),
            (38.10, 1.32, -6.89),
            (38.50, 1.95, -4.68),
            (38.90, 2.29, -4.12),
            (39.30, 1.69, -3.14),
            (39.70, 0.82, -3.03),
            (40.10, 0.52, -5.28),
            (40.40, 0.14, -4.36),
        ];
        let score = |latch: &mut dyn FnMut(f64, f64, f64) -> bool| {
            let mut sc = crate::scorecard::Scorer::new();
            let ev = |e: &str, t: f64| serde_json::json!({"t": t, "kind": "autotaxi", "event": e});
            sc.feed(&ev("join", 0.0));
            let mut joined = false;
            for &(t, lat, err) in TICKS {
                if !joined && latch(lat.abs(), err, t) {
                    joined = true;
                    sc.feed(&ev("joined", t));
                }
                sc.feed(
                    &serde_json::json!({"t": t, "kind": "autotaxi", "event": "drive",
                    "gs": 6.0, "lat_m": lat, "err": err, "bl": 0.0, "br": 0.0, "yaw_dps": 0.0}),
                );
            }
            let out = sc.feed(&ev("holding_point", 41.0));
            out.iter()
                .find(|(k, _)| k == "stage_result")
                .map(|(_, v)| v["max_lat_m"].as_f64().unwrap())
                .unwrap()
        };
        // The old latch fired mid-swing, 14.5 m off and 42 degrees out.
        let old_at = TICKS.iter().find(|&&(_, lat, err)| lat.abs() < 15.0 && err.abs() < 45.0);
        assert!(matches!(old_at, Some(&(_, lat, _)) if lat > 14.0), "old latch: {old_at:?}");
        // The new one waits for the line (the scorer discounts a join's ticks
        // on its own since the same day, so it is scored through it too).
        let mut c = JoinCapture::default();
        let mut fired_at = None;
        let new = score(&mut |lat, err, t| {
            let done = c.step(lat, err, t);
            if done && fired_at.is_none() {
                fired_at = Some(lat);
            }
            done
        });
        assert!(matches!(fired_at, Some(lat) if lat < 3.0), "new latch fired at {fired_at:?} m");
        assert!(new < 3.0, "joined on the line, the taxi scores {new} m");
    }

    #[test]
    fn join_state_machine_grants_and_captures() {
        // On route, aligned: no join, the tight guard from tick one.
        assert_eq!(join_allowance_m(2.0, 5.0), 0.0);
        // Off route or backwards: joined with turning room.
        assert!(join_allowance_m(40.0, 0.0) > 40.0);
        assert!(join_allowance_m(2.0, 170.0) > OFF_ROUTE_STOP_M);
        // Captured only when close AND aligned.
        assert!(join_done(2.0, 10.0));
        assert!(!join_done(5.0, 10.0), "5 m off is not on the line");
        assert!(
            !join_done(2.0, 90.0),
            "close but still turning is not joined"
        );
        assert!(!join_done(20.0, 10.0), "aligned but wide is not joined");
        // A capture that has stopped closing is joined where it settled.
        let mut c = JoinCapture::default();
        assert!(!c.step(6.0, 5.0, 0.0));
        assert!(!c.step(5.9, 4.0, 5.0));
        assert!(c.step(5.8, 3.0, 8.5), "settled at 6 m for 8 s is joined");
        let mut c = JoinCapture::default();
        assert!(!c.step(6.0, 50.0, 0.0));
        assert!(!c.step(6.0, 50.0, 20.0), "settled but still turning is not");
        // Aligned rejoins ride at bend speed, not the sharp crawl.
        assert_eq!(Pace::default().join_cap_kts(20.0), TURN_KTS);
        assert_eq!(Pace::default().join_cap_kts(-170.0), SHARP_KTS);
    }

    #[test]
    fn the_wheel_straightens_into_the_stop() {
        // Stopped (or as good as): the fade owns it, so the brakes never bite
        // a deflected wheel. Well out: steering untouched. Closing: faded.
        const STOPPED: f64 = 0.0;
        assert_eq!(stop_straighten(0.8, 50.0, STOPPED), 0.8);
        let close = stop_straighten(0.8, 4.0, STOPPED);
        assert!(close > 0.0 && close < 0.5, "{close}");
        assert_eq!(stop_straighten(0.8, 1.0, STOPPED), 0.0);
        assert_eq!(stop_straighten(-0.8, 0.5, STOPPED), 0.0, "both directions");
    }

    /// The ramp must open the governor's braking gap while there is still room
    /// to stop, not a couple of metres out.
    ///
    /// `taxi_control` brakes when `(target - gs) - trend*2.5 < -3.0`, i.e. only
    /// once the aircraft is ~3 kt above target, and until the target drops
    /// below 1 kt it is capped at the 0.22 comfort effort. At EGHI, taxiing at
    /// 12 kt base, that gap did not open until 6.8 m to go: the aircraft
    /// crossed the hold line at 6.7 kt and finished five metres past it with
    /// its nose over the yellow.
    #[test]
    fn braking_starts_with_room_to_stop_at_the_hold() {
        const BASE: f64 = 12.0; // the EGHI taxi's own base speed
        const BRAKE_GAP_KTS: f64 = 3.0; // taxi_control's threshold
                                        // Measured achievable deceleration at the gentle comfort effort.
        const ACHIEVABLE_MS2: f64 = 0.6;

        // Where does the aircraft, still doing its base speed, first find
        // itself 3 kt above the commanded target?
        let mut brake_at = 0.0;
        let mut d = 0.0;
        while d < 80.0 {
            if BASE - stop_ramp_kts(d) >= BRAKE_GAP_KTS {
                brake_at = d;
            }
            d += 0.1;
        }
        // It has to be far enough out to actually stop from base speed.
        let need_m = (BASE / MS_TO_KTS).powi(2) / (2.0 * ACHIEVABLE_MS2);
        assert!(
            brake_at > need_m,
            "braking must begin with room: opens at {brake_at:.1} m but \
             stopping from {BASE} kt needs {need_m:.1} m"
        );
        // And with margin, not just barely — the governor has its own loop
        // lag on top of the physics.
        //
        // The margin is only ~1.35x, and that is the honest ceiling of this
        // approach rather than a number to tune toward. Stopping from 12 kt
        // takes ~32 m ONLY because `taxi_control` caps a planned stop at the
        // 0.22 comfort effort until the last metre, and a real aircraft stops
        // from 12 kt in a fraction of that. The deeper fix is to tell
        // `taxi_control` that a stop is planned so it can use its 0.6
        // authority earlier; that re-opens tuning three airframes have already
        // complained about, so it is deliberately not bundled with this.
        assert!(
            brake_at > need_m * 1.25,
            "want margin: {brake_at:.1} m vs {need_m:.1} m needed"
        );

        // The ramp is still a ramp: monotonic, and it still reaches a stop.
        let mut prev = 0.0;
        let mut d = 0.0;
        while d <= 60.0 {
            let v = stop_ramp_kts(d);
            assert!(v >= prev - 1e-9, "monotonic at {d} m");
            prev = v;
            d += 0.5;
        }
        assert_eq!(stop_ramp_kts(STOP_ZERO_M), 0.0, "still stops");
        assert!(stop_ramp_kts(2.0) >= CREEP_KTS, "creep survives");
    }

    /// The fade must not disarm the nose while the aircraft is still rolling.
    ///
    /// `remaining_m` saturates at `-STOP_SHORT_M` as soon as the projection
    /// reaches the end of the polyline, so the distance term pins at -5 with
    /// 2-3 kt still on the clock — and zeroing the command there also kills
    /// the yaw damping inside it. EGHI 2026-07-30: steer 0.00, yaw -2 deg/s
    /// unarrested, cross-track growing 2.45 -> 5.08 m while coasting to a
    /// stop, at the HOLDING POINT and in all three stops of that sortie.
    #[test]
    fn the_nose_is_still_held_while_the_aircraft_is_still_moving() {
        // The exact saturated value the blackbox pins at, at the speed it was
        // actually doing when the steering went to zero.
        let rolling = stop_straighten(0.8, -STOP_SHORT_M, 2.7);
        assert!(
            rolling.abs() > 0.0,
            "steering must survive past the end of the polyline while rolling"
        );
        assert!(
            rolling.abs() >= 0.8 * STRAIGHTEN_FLOOR - 1e-9,
            "and keep enough authority to arrest yaw: {rolling}"
        );
        // Sign is preserved — this only ever scales the command.
        assert!(stop_straighten(-0.8, -STOP_SHORT_M, 2.7) < 0.0);

        // ...but once genuinely stopped it goes to zero, so the original
        // carve-under-braking fix still holds.
        assert_eq!(stop_straighten(0.8, -STOP_SHORT_M, 0.4), 0.0);
        assert_eq!(
            stop_straighten(0.8, -STOP_SHORT_M, STRAIGHTEN_HOLD_KTS),
            0.0
        );

        // It is a FLOOR, never a boost: far out, full steering is unchanged
        // whether moving or not.
        assert_eq!(stop_straighten(0.8, 50.0, 12.0), 0.8);
        // And it never exceeds the command it was given.
        for gs in [0.0, 1.4, 1.6, 3.0, 10.0] {
            for rem in [-5.0, 0.0, 2.0, 8.0, 40.0] {
                assert!(
                    stop_straighten(0.8, rem, gs) <= 0.8 + 1e-9,
                    "never amplifies (rem {rem}, gs {gs})"
                );
            }
        }
    }

    #[test]
    fn line_up_enters_ahead_and_finishes_down_the_centreline() {
        use crate::runway::{along_track_nm, cross_track_nm, Runway};
        // A 2,000 m runway taking off due north from (0,0); hold-short spot
        // 40 m EAST of the centreline, abeam the numbers.
        let rwy = Runway {
            icao: "TEST".into(),
            id: "36".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 2000.0,
            surface: 1,
            dist_nm: 0.02,
            elev_ft: 0.0,
        };
        let (hlat, hlon) = crate::autoland::project_latlon(0.0, 0.0, 90.0, 40.0 / 1852.0);
        let path = line_up_path(&rwy, hlat, hlon, None).expect("feasible");
        assert_eq!(path.len(), 3);
        // The entry point is ON the centreline, ahead of abeam.
        let (elat, elon) = path[1];
        assert!(
            (cross_track_nm(elat, elon, &rwy) * 1852.0).abs() < 2.0,
            "entry on centreline"
        );
        let entry_past = -along_track_nm(elat, elon, &rwy) * 1852.0;
        assert!(
            entry_past > 20.0 && entry_past < 120.0,
            "ahead of abeam: {entry_past}"
        );
        // The stop point is further down, still leaving plenty of runway.
        let (blat, blon) = path[2];
        let stop_past = -along_track_nm(blat, blon, &rwy) * 1852.0;
        assert!(
            stop_past > entry_past + 40.0,
            "finishes aligned: {stop_past}"
        );
        // Too far off the strip is not a line-up.
        let (flat, flon) = crate::autoland::project_latlon(0.0, 0.0, 90.0, 300.0 / 1852.0);
        assert!(line_up_path(&rwy, flat, flon, None).is_none());
        // Near the far end there is no runway left to line up onto.
        let (nlat, nlon) = crate::autoland::project_latlon(0.0, 0.0, 0.0, 1800.0 / 1852.0);
        let (nlat, nlon) = crate::autoland::project_latlon(nlat, nlon, 90.0, 40.0 / 1852.0);
        assert!(line_up_path(&rwy, nlat, nlon, None).is_none());
    }

    #[test]
    fn line_up_rolls_forward_along_the_heading_not_a_diagonal() {
        use crate::autoland::bearing_deg;
        use crate::runway::{cross_track_nm, Runway};
        // Runway 36 (course 0, due north) from (0,0). Hold-short 40 m EAST of
        // the centreline, abeam the numbers.
        let rwy = Runway {
            icao: "TEST".into(),
            id: "36".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 2000.0,
            surface: 1,
            dist_nm: 0.02,
            elev_ft: 0.0,
        };
        let (hlat, hlon) = crate::autoland::project_latlon(0.0, 0.0, 90.0, 40.0 / 1852.0);
        // Nose pointed due WEST — straight at the runway. Leg one should follow
        // that heading onto the centreline (the taxiway lead-on), NOT cut a
        // ~45 degree diagonal ahead. Bearing of leg one ≈ 270.
        let path = line_up_path(&rwy, hlat, hlon, Some(270.0)).expect("feasible");
        let brg = bearing_deg(path[0].0, path[0].1, path[1].0, path[1].1);
        let off = crate::angles::abs_diff_deg(brg, 270.0);
        assert!(off < 12.0, "leg one follows the nose (270), got {brg}");
        assert!(
            (cross_track_nm(path[1].0, path[1].1, &rwy) * 1852.0).abs() < 3.0,
            "entry on centreline"
        );

        // Nose nearly aligned with the runway (5 degrees): no clean forward
        // crossing, so it falls back to the diagonal swing — clearly off the
        // near-aligned heading.
        let path2 = line_up_path(&rwy, hlat, hlon, Some(5.0)).expect("feasible");
        let brg2 = bearing_deg(path2[0].0, path2[0].1, path2[1].0, path2[1].1);
        let off2 = crate::angles::abs_diff_deg(brg2, 5.0);
        assert!(
            off2 > 20.0,
            "near-aligned nose falls back to the diagonal, got {brg2}"
        );
    }

    /// Real-install probe: what does the LXGB planner actually hand the
    /// line-up at the runway 27 hold? Prints the route tail, the lead-on,
    /// and the drive path the picker would build — evidence, not theory.
    /// `cargo test -- --ignored live_lxgb_line_up --nocapture`
    #[test]
    #[ignore]
    fn live_lxgb_line_up_lead_on() {
        let apt =
            &crate::config::xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
        let net = crate::nav::find_taxi_net(apt, "LXGB")
            .unwrap()
            .expect("LXGB network");
        let zones: Vec<String> = vec!["27".into(), "09".into()];
        // Apron, mid-field north side -> the 27 end.
        let plan = net
            .plan(
                36.1535,
                -5.3495,
                36.1515,
                -5.3412,
                crate::nav::HOLD_SHORT_STANDOFF_M,
                &zones,
            )
            .expect("a route");
        println!("holding_point: {} via {:?}", plan.holding_point, plan.via);
        let m = |p: (f64, f64)| {
            let dx = (p.1 + 5.3412) * 111_320.0 * 36.15_f64.to_radians().cos();
            let dy = (p.0 - 36.1515) * 111_320.0;
            (dx.round(), dy.round())
        };
        println!("route tail (m from 27 thr, x=E y=N):");
        for p in plan.path.iter().rev().take(4).rev() {
            println!("  {:?}", m(*p));
        }
        println!("lead_on ({} pts):", plan.lead_on.len());
        for p in &plan.lead_on {
            println!("  {:?}", m(*p));
        }
        assert!(
            !plan.lead_on.is_empty(),
            "zone-trimmed route must keep its lead-on"
        );
        // Now the picker, from the route's actual end (where the taxi stops).
        let &(hlat, hlon) = plan.path.last().unwrap();
        let cands = crate::runway::nearby_runways(apt, hlat, hlon, 1.0).unwrap();
        let picked = pick_line_up_plan(
            &cands,
            hlat,
            hlon,
            Some("27"),
            400.0,
            false,
            None,
            Some(&plan.lead_on),
            TurnRound::TEARDROP,
        );
        let (rwy, lp) = picked.expect("a line-up plan");
        println!("picked runway {}", rwy.id);
        match lp {
            LineUpPlan::Plain(p) => {
                println!("drive path ({} pts):", p.len());
                for q in &p {
                    println!("  {:?}", m(*q));
                }
                assert!(
                    p.len() > 3,
                    "authored points present, not a 3-point diagonal: {}",
                    p.len()
                );
            }
            LineUpPlan::Backtrack(_) => println!("backtrack picked"),
        }
    }

    /// Le Bourget 07, the whole chain the pilot actually heard fail.
    /// `cargo test -- --ignored live_lfpb_line_up --nocapture`
    ///
    /// Full auto taxied to the cleared holding point, stopped there, and the
    /// line-up answered "I can't find runway zero seven next to us" — the
    /// stop was 169 m off 07's centreline (the line-up's lateral gate is
    /// 150 m) and the lead-on ended 34 m off it (the authored path's gate is
    /// 20 m), because the route had been planned to runway 09's entrance
    /// (2026-09-22). Both gates are the line-up's own and both were right;
    /// the route was wrong, so this test starts at the tower's plan.
    #[test]
    #[ignore]
    fn live_lfpb_line_up_finds_runway_07() {
        let apt =
            &crate::config::xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
        let Ok(Some(net)) = crate::nav::find_taxi_net(apt, "LFPB") else {
            return; // no install, no verdict
        };
        let runways = crate::nav::find_runways(apt, "LFPB").unwrap();
        let strip = runways.iter().find(|r| r.ident_a == "07").unwrap();
        let zones: Vec<String> = vec!["07".into(), "25".into()];
        let plan = net
            .plan_to_runway(
                48.95620,
                2.43880,
                strip.lat_a,
                strip.lon_a,
                100.0,
                &zones,
                Some(strip),
            )
            .expect("a route to 07");
        // From the route's own end — where the drive stops, 48.962592,
        // 2.421745 on the day, heading 266.9 (the hold faces west).
        let &(hlat, hlon) = plan.path.last().unwrap();
        let cands = crate::runway::nearby_runways(apt, hlat, hlon, 1.0).unwrap();
        let (rwy, lp) = pick_line_up_plan(
            &cands,
            hlat,
            hlon,
            Some("07"),
            2000.0,
            false,
            Some(266.9),
            Some(&plan.lead_on),
            TurnRound::TEARDROP,
        )
        .expect("a line-up plan for runway 07");
        assert_eq!(rwy.id, "07");
        match lp {
            LineUpPlan::Plain(p) => {
                println!("lined up via {} authored points", p.len());
                assert!(
                    p.len() > 3,
                    "rides the drawn lead-on, not a synthetic diagonal: {}",
                    p.len()
                );
            }
            LineUpPlan::Backtrack(p) => panic!("a 2,991 m strip needs no backtrack: {p:?}"),
        }
    }

    /// LEMG 13 (2026-09-26, sortie 1790441293, a 737): the line-up rode the
    /// drawn lead-on and its projection jumped 6 m and 11 m forward in single
    /// ticks while the heading error went -6 to -43 degrees; the scorer read
    /// 6.0 m. `cargo test -- --ignored live_lemg_13_lead_on --nocapture`
    #[test]
    #[ignore] // needs the real X-Plane install
    fn live_lemg_13_lead_on_runs_one_way() {
        let apt = &crate::config::xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
        let Ok(Some(net)) = atc::nav::find_taxi_net(apt, "LEMG") else {
            return;
        };
        let (hlat, hlon) = (36.684854, -4.511439);
        let rwy = crate::runway::runway_by_id(apt, hlat, hlon, "13", 5.0, Some("LEMG"))
            .unwrap()
            .expect("LEMG 13");
        let runways = atc::nav::find_runways(apt, "LEMG").unwrap();
        let strip = runways.iter().find(|r| r.ident_a == "13" || r.ident_b == "13").expect("13/31");
        let zone: Vec<String> = vec!["13".into(), "31".into()];
        let plan = net
            .plan_to_runway(36.673087, -4.491381, rwy.thr_lat, rwy.thr_lon, 60.0, &zone, Some(strip))
            .expect("a route");
        println!("holding point {} via {:?}, lead-on {} pts", plan.holding_point, plan.via, plan.lead_on.len());
        // The line-up starts where the taxi stopped: the route's own end.
        let &(hlat, hlon) = plan.path.last().expect("a route end");
        let (path, stop) = line_up_path_from_lead_on(&rwy, hlat, hlon, &plan.lead_on).expect("the lead-on");
        let geo = taxi_guide::Geo::new(path[0].0);
        let pts: Vec<(f64, f64)> = path.iter().map(|&p| geo.to_m(path[0], p)).collect();
        let mut prev_brg: Option<f64> = None;
        let mut worst_turn = 0.0_f64;
        for w in pts.windows(2) {
            let d = taxi_guide::dist(w[0], w[1]);
            let b = taxi_guide::bearing(w[0], w[1]);
            let turn = prev_brg.map_or(0.0, |p| crate::angles::signed_diff_deg(b, p));
            println!("  {:7.1} {:7.1}  len {:5.1}  brg {:6.1}  turn {:+6.1}", w[1].0, w[1].1, d, b, turn);
            if d > 0.5 {
                worst_turn = worst_turn.max(turn.abs());
                prev_brg = Some(b);
            }
        }
        println!("stop {stop:.0} m, sharpest vertex {worst_turn:.0} deg");
        // Before the paint gaps were filled (`nav::fill_paint_gaps`) the
        // lead-on turned -62, +49, +43 and -63 degrees at consecutive
        // vertices where the network's chord cut the drawn curve; the curve
        // itself never turns more than about 11 degrees at one vertex.
        assert!(worst_turn < 15.0, "a lead-on runs one way: {worst_turn:.0} degrees at one vertex");
    }

    /// The corners as read before #194: a VERTEX turning [`TURN_DEG`] over
    /// the 15 m either side, its radius the circle through the route 8 m
    /// either side. Kept to compare the curvature reading against.
    fn vertex_bends(pts: &[(f64, f64)], cum: &[f64]) -> Vec<Bend> {
        let mut bends = Vec::new();
        for i in 1..pts.len().saturating_sub(1) {
            let b_in = taxi_guide::bearing(taxi_guide::point_at(pts, cum, cum[i] - 15.0), pts[i]);
            let b_out = taxi_guide::bearing(pts[i], taxi_guide::point_at(pts, cum, cum[i] + 15.0));
            let delta = crate::angles::abs_diff_deg(b_out, b_in);
            if delta >= TURN_DEG {
                let radius = circle_radius_m(
                    taxi_guide::point_at(pts, cum, cum[i] - 8.0),
                    pts[i],
                    taxi_guide::point_at(pts, cum, cum[i] + 8.0),
                );
                bends.push(Bend { start: cum[i], end: cum[i], deg: delta, radius });
            }
        }
        bends
    }

    /// The fastest the schedule's corner caps let a route be entered at
    /// `progress`, flown at `gs`: each corner's cap as `bend_cap_kts` makes it.
    fn caps_along(route: &Route, step_m: f64) -> Vec<f64> {
        let mut out = Vec::new();
        let mut s = 0.0;
        while s < route.total_m() {
            let gs = route.pace.straight;
            out.push(route.bend_cap_kts(s, gs).unwrap_or(f64::INFINITY).min(gs));
            s += step_m;
        }
        out
    }

    /// #194 at scale: every route from a sample of stands to every runway
    /// end at 24 fields, ridden as the tower rides them, for the Citation,
    /// the 737, the A330, the SR22 and the Evolution: where the curvature
    /// reading lets the schedule go faster than the vertex reading did, by
    /// how much, and where the vertex reading missed a corner it now sees.
    /// `cargo test --release -- --ignored real_install_corner_caps --nocapture`
    #[test]
    #[ignore] // needs the real X-Plane install
    fn real_install_corner_caps_are_never_faster() {
        let apt = &crate::config::xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
        if !std::path::Path::new(apt).exists() {
            return;
        }
        let fields = std::env::var("CORNER_FIELDS").unwrap_or_else(|_| {
            "LEMG EGKK EGBB EGLL EGPH LFPB LFPG EDDF EHAM LEBL LIRF LSZH EBLG LIPR LIME LILB LSZS EDLP LFBD EIDW KJFK KSFO LIEA LSGG".into()
        });
        let paces: Vec<(&str, Pace)> = ["citation_x", "boeing_737_800", "airbus_a330_300", "cirrus_sr22", "lancair_evolution"]
            .iter()
            .map(|id| (*id, Pace::from_profile(crate::aircraft::load(id).expect(id).taxi.as_ref())))
            .collect();
        let (mut routes, mut faster, mut slower, mut new_corners) = (0, 0usize, 0usize, 0usize);
        let mut worst = (0.0_f64, String::new());
        let mut time: std::collections::BTreeMap<&str, (f64, f64)> = Default::default();
        for icao in fields.split_whitespace() {
            let Ok(Some(net)) = atc::nav::find_taxi_net(apt, icao) else { continue };
            let Ok(runways) = atc::nav::find_runways(apt, icao) else { continue };
            let step = (net.stands.len() / 8).max(1);
            // The sorties' own gates too: the Citation's LEMG gate (#194)
            // and the 737's EGKK stand (1790713378, 1790726972).
            let mut starts: Vec<(f64, f64, String)> =
                net.stands.iter().step_by(step).map(|s| (s.lat, s.lon, s.name.clone())).collect();
            match icao {
                "LEMG" => starts.push((36.6733571, -4.4894417, "the #194 gate".into())),
                "EGKK" => starts.push((51.1553957, -0.1653696, "the 737's stand".into())),
                _ => {}
            }
            for st in &starts {
                let (slat, slon, sname) = (st.0, st.1, &st.2);
                for r in &runways {
                    for (id, tlat, tlon) in [(&r.ident_a, r.lat_a, r.lon_a), (&r.ident_b, r.lat_b, r.lon_b)] {
                        let zone = [r.ident_a.clone(), r.ident_b.clone()];
                        let Some(plan) = net.plan_to_runway(slat, slon, tlat, tlon, 60.0, &zone, Some(r)) else {
                            continue;
                        };
                        let w = TaxiWatch {
                            path: plan.path.clone(),
                            via: String::new(),
                            seg: plan.seg.clone(),
                            holding_point: String::new(),
                            lead_on: Vec::new(),
                        };
                        let Some(mut route) = Route::build(&w) else { continue };
                        routes += 1;
                        let curv = route.bends.clone();
                        let vert = vertex_bends(&route.pts, &route.cum);
                        new_corners += curv
                            .iter()
                            .filter(|b| !vert.iter().any(|v| v.start >= b.start - 15.0 && v.start <= b.end + 15.0))
                            .count();
                        for (name, pace) in &paces {
                            route.pace = *pace;
                            route.bends = curv.clone();
                            let new = caps_along(&route, 1.0);
                            route.bends = vert.clone();
                            let old = caps_along(&route, 1.0);
                            *time.entry(*name).or_insert((0.0, 0.0)) = {
                                let e = time.get(name).copied().unwrap_or((0.0, 0.0));
                                (e.0 + old.iter().map(|c| 1.0 / c.max(0.1)).sum::<f64>(), e.1 + new.iter().map(|c| 1.0 / c.max(0.1)).sum::<f64>())
                            };
                            for (k, (n, o)) in new.iter().zip(&old).enumerate() {
                                if n > &(o + 0.05) {
                                    faster += 1;
                                    if n - o > worst.0 {
                                        worst = (n - o, format!("{icao} {} -> {id} {name} at {k} m: {o:.1} -> {n:.1} kt", sname));
                                    }
                                } else if n + 0.05 < *o {
                                    slower += 1;
                                }
                            }
                        }
                        route.bends = curv;
                    }
                }
            }
        }
        println!("{routes} routes; stations faster {faster}, slower {slower}; corners the vertex reading missed {new_corners}");
        assert_eq!(faster, 0, "{}", worst.1);
        println!("worst faster: {:.2} kt {}", worst.0, worst.1);
        for (name, (o, n)) in &time {
            println!("{name}: time at the corner caps {:.1}% of before", 100.0 * n / o);
        }
    }

    /// #194's corner, LEMG Quebec onto Delta, from the install: the network
    /// (the Quebec node before, the node on Delta's centreline, the Delta
    /// node after) and the drawn lines near it: the fillet right, the two
    /// halves of the fillet left, Quebec, Delta.
    const LEMG_QD_RAW: [(f64, f64); 3] = [(36.683599, -4.506806), (36.687126, -4.502939), (36.688841, -4.506956)];
    #[rustfmt::skip]
    const LEMG_QD_PAINT: [&[(f64, f64)]; 5] = [
        &[(36.68680750, -4.50327940), (36.68681270, -4.50327312), (36.68682681, -4.50325461), (36.68684765, -4.50322436), (36.68687299, -4.50318286), (36.68690063, -4.50313061), (36.68692837, -4.50306808), (36.68695399, -4.50299578), (36.68697530, -4.50291420), (36.68698764, -4.50282780), (36.68698962, -4.50274279), (36.68698394, -4.50266229), (36.68697326, -4.50258944), (36.68696029, -4.50252734), (36.68694770, -4.50247912), (36.68693817, -4.50244790), (36.68693440, -4.50243680)],
        &[(36.68734910, -4.50342280), (36.68734506, -4.50341464), (36.68733313, -4.50339276), (36.68731358, -4.50336104), (36.68728668, -4.50332335), (36.68725269, -4.50328358), (36.68721191, -4.50324561), (36.68716458, -4.50321332), (36.68711100, -4.50319060)],
        &[(36.68711100, -4.50319060), (36.68704602, -4.50318040), (36.68698726, -4.50318389), (36.68693547, -4.50319725), (36.68689141, -4.50321670), (36.68685583, -4.50323844), (36.68682948, -4.50325866), (36.68681312, -4.50327358), (36.68680750, -4.50327940)],
        &[(36.68332550, -4.50710330), (36.68386960, -4.50650580), (36.68680750, -4.50327940)],
        &[(36.68320300, -4.49356340), (36.68693440, -4.50243680), (36.68734910, -4.50342280), (36.68877120, -4.50680470), (36.68932580, -4.50812360), (36.68958210, -4.50873320)],
    ];

    /// The route both sorties drove through that corner (each planned the
    /// same 3,614.9 m), as ridden before the fillets: 815.9 to 602.2 m to
    /// go, Quebec 041.4, then 36, 21, 19, 69, 114, 2, 41 and 318 degrees in
    /// seven legs, Delta 297.7.
    #[rustfmt::skip]
    const LEMG_QD_FLOWN: [(f64, f64); 44] = [
        (36.6863838, -4.5037447), (36.6864174, -4.5037078), (36.6864509, -4.5036710), (36.6864845, -4.5036341),
        (36.6865181, -4.5035972), (36.6865516, -4.5035604), (36.6865852, -4.5035235), (36.6866188, -4.5034867),
        (36.6866523, -4.5034498), (36.6866859, -4.5034129), (36.6867195, -4.5033761), (36.6867530, -4.5033392),
        (36.6867866, -4.5033023), (36.6868075, -4.5032794), (36.6868573, -4.5032336), (36.6868959, -4.5032147),
        (36.6869351, -4.5031974), (36.6869580, -4.5031231), (36.6869333, -4.5030542), (36.6870252, -4.5030495),
        (36.6870588, -4.5030127), (36.6870924, -4.5029758), (36.6871260, -4.5029390), (36.6871623, -4.5029787),
        (36.6871832, -4.5030284), (36.6872042, -4.5030782), (36.6872251, -4.5031280), (36.6872460, -4.5031777),
        (36.6872669, -4.5032275), (36.6872879, -4.5032772), (36.6873088, -4.5033270), (36.6873297, -4.5033767),
        (36.6873506, -4.5034265), (36.6873716, -4.5034762), (36.6873925, -4.5035260), (36.6874134, -4.5035757),
        (36.6874343, -4.5036255), (36.6874553, -4.5036753), (36.6874762, -4.5037250), (36.6874971, -4.5037748),
        (36.6875180, -4.5038245), (36.6875390, -4.5038743), (36.6875599, -4.5039240), (36.6875808, -4.5039738),
    ];

    /// The Citation X's nosewheel, fitted to the drive ticks of the #194
    /// sorties (iMac 2026-09-30): the yaw rate goes to `k` deg/s per m/s of
    /// groundspeed per unit of steer, capped at 12 deg/s, through a 1.3 s
    /// lag after `delay_s`. Over all three sorties k 8.5 and no delay fit
    /// best (1.00 deg/s rms against the logged, filtered rate); over the
    /// fast ticks of 1790765957's corner (over 10 kt) k 7.5 after 0.3 s
    /// (0.48 rms, where the first fits 0.86). Full lock is 12 deg/s from 3
    /// kt up: at 17 kt a 37 m radius, the Quebec-onto-Delta fillet's own.
    struct CitationNose {
        r: f64,
        k: f64,
        delay_s: f64,
        sent: std::collections::VecDeque<(f64, f64)>,
    }
    impl CitationNose {
        const MAX_DPS: f64 = 12.0;
        const TAU_S: f64 = 1.3;
        /// All three sorties' fit.
        fn slow() -> Self {
            CitationNose { r: 0.0, k: 8.5, delay_s: 0.0, sent: Default::default() }
        }
        /// The fast corner's fit.
        fn fast() -> Self {
            CitationNose { r: 0.0, k: 7.5, delay_s: 0.3, sent: Default::default() }
        }
        fn step(&mut self, t: f64, steer: f64, v_ms: f64, dt: f64) -> f64 {
            self.sent.push_back((t, steer));
            while self.sent.len() > 1 && self.sent[1].0 <= t - self.delay_s + 1e-9 {
                self.sent.pop_front();
            }
            let s = self.sent[0].1;
            let want = (self.k * v_ms * s).clamp(-Self::MAX_DPS, Self::MAX_DPS);
            self.r += (want - self.r) * (dt / Self::TAU_S).min(1.0);
            self.r
        }
    }

    /// The drive's law, tick by tick as `drive` runs it on a linked nose
    /// (no brake steering), with the Citation's own pace, through
    /// `plant`, from `start_kts` on the route's first point.
    /// Returns the scorer's verdict: (max cut inside a vertex, max off the
    /// line).
    /// The corners are read as before #194 when `vertex`.
    fn fly_citation_with(path: &[(f64, f64)], start_kts: f64, vertex: bool, mut plant: CitationNose) -> (f64, f64) {
        use crate::scorecard::{LineJudge, CORNER_TURN_DPS};
        let pace = Pace::from_profile(crate::aircraft::load("citation_x").expect("citation").taxi.as_ref());
        assert!(!pace.brake_steer);
        let watch = TaxiWatch {
            path: path.to_vec(),
            via: String::new(),
            seg: vec![(String::new(), false); path.len() - 1],
            holding_point: String::new(),
            lead_on: Vec::new(),
        };
        let mut route = Route::build(&watch).expect("route");
        route.pace = pace;
        if vertex {
            route.bends = vertex_bends(&route.pts, &route.cum);
        }
        let dt = 0.25;
        let (mut x, mut y) = (0.0_f64, 0.0_f64);
        let mut psi = taxi_guide::bearing(route.pts[0], route.pts[1]);
        let mut v = start_kts / MS_TO_KTS;
        let (mut yaw, mut prev_psi, mut prev_steer) = (0.0_f64, psi, 0.0_f64);
        let mut trim = StraightTrim::default();
        let mut turn_dir = None;
        let mut judge = LineJudge::default();
        let end_m = route.total_m() - 40.0;
        let m_lon = 111_320.0 * path[0].0.to_radians().cos();
        for i in 0..(600.0 / dt) as usize {
            let t = i as f64 * dt;
            let gs = v * MS_TO_KTS;
            let (lat, lon) = (path[0].0 + y / 111_320.0, path[0].1 + x / m_lon);
            let (progress, offset) = route.fix_signed(lat, lon).expect("a fix");
            if progress > end_m {
                break;
            }
            let remaining = route.remaining(progress);
            let look = steer_look_m(
                &pace,
                gs,
                route.route_turn_rate_dps(progress, BEND_PROBE_M, gs),
                remaining,
                false,
            );
            let err = committed_err(route.heading_err_deg(lat, lon, psi, progress, look), &mut turn_dir);
            let route_turn = route.route_turn_rate_dps(progress, look, gs);
            let raw = crate::angles::signed_diff_deg(psi, prev_psi) / dt;
            prev_psi = psi;
            yaw = yaw * 0.5 + raw.clamp(-90.0, 90.0) * 0.5;
            let tr = trim.step(offset, route_turn, gs, false, remaining, dt);
            let want = stop_straighten(
                (steer_cmd_with(damped_err_deg(err, yaw, route_turn, pace.steer_lead_s), pace.steer_full_deg) + tr)
                    .clamp(-1.0, 1.0)
                    * steer_authority(gs),
                remaining,
                gs,
            );
            let steer = slew_steer(prev_steer, want, pace.steer_slew_per_s, dt);
            prev_steer = steer;
            let asked = (route_turn.abs() >= CORNER_TURN_DPS).then(|| route_turn.signum());
            let cutting = judge.corner(t, gs, offset, Some(remaining), asked, yaw);
            judge.judge(offset, cutting);
            let sched = schedule_kts(pace.straight, &route, progress, gs).min(pace.track_cap_kts(offset.abs(), err));
            // The governor as it flies: power up to the schedule, a coast
            // over it, the brakes only 3 kt over (1790765957 held 17.4 kt
            // against the Citation's 15 down the whole of Quebec).
            let over = gs - sched;
            let accel = if over < 0.0 {
                0.5
            } else if over > 3.0 {
                -0.7
            } else {
                -0.05
            };
            v = (v + accel * dt).max(0.0);
            let r = plant.step(t, steer, v, dt);
            psi = (psi + r * dt).rem_euclid(360.0);
            x += v * psi.to_radians().sin() * dt;
            y += v * psi.to_radians().cos() * dt;
        }
        judge.flush();
        (judge.max_cut, judge.max_lat)
    }

    /// #194, LEMG (Citation X, iMac 2026-09-30), the corner three times.
    /// 1790734216 and 1790738786 drove the jagged ride (4.6 and 4.8 m off);
    /// e31f338 rode the fillet, and 1790765957 (build db43c97) went into it
    /// at 17.4 kt, because the vertex reading saw no corner in a fillet
    /// drawn 3 to 5 m at a time: full lock 3 s, err -44, 7.3 m off. The
    /// fast fit of the nose through the drive's own law and the governor as
    /// it flies (brakes only 3 kt over) fails the fillet from 17.4 kt under
    /// the vertex reading and holds it inside 3 m under the curvature
    /// reading, from any entry speed and on either fit. The jagged ride, no
    /// longer planned, replays 2.4 to 3.0 m either way (flown 4.6): the
    /// curvature reading does it no harm.
    #[test]
    fn the_citation_turns_quebec_onto_delta_on_the_line() {
        use crate::scorecard::{TAXI_MAX_CUT_M, TAXI_MAX_LAT_M};
        let paint: Vec<Vec<(f64, f64)>> = LEMG_QD_PAINT.iter().map(|l| l.to_vec()).collect();
        let seg = vec![("Q".to_string(), false), ("D".to_string(), false)];
        let (ridden, _) = atc::nav::ride_paint(&paint, &LEMG_QD_RAW, &seg);
        // From 150 m up Quebec to 150 m along Delta.
        let node = LEMG_QD_RAW[1];
        let geo = taxi_guide::Geo::new(node.0);
        let fillet: Vec<(f64, f64)> =
            ridden.iter().copied().filter(|&p| taxi_guide::dist(geo.to_m(node, p), (0.0, 0.0)) < 150.0).collect();
        let (_, as_flown) = fly_citation_with(&fillet, 17.4, true, CitationNose::fast());
        assert!(as_flown > TAXI_MAX_LAT_M, "1790765957 fails as it did: {as_flown:.2} m");
        for v0 in [10.0, 15.0, 17.4] {
            for plant in [CitationNose::slow as fn() -> CitationNose, CitationNose::fast] {
                let (cut, lat) = fly_citation_with(&fillet, v0, false, plant());
                assert!(lat <= TAXI_MAX_LAT_M && cut <= TAXI_MAX_CUT_M, "from {v0} kt: {lat:.2} off, {cut:.2} in");
                let (vc, vl) = fly_citation_with(&LEMG_QD_FLOWN, v0, true, plant());
                let (cc, cl) = fly_citation_with(&LEMG_QD_FLOWN, v0, false, plant());
                assert!(cl <= vl + 0.25 && cc <= vc + 0.4, "jagged from {v0} kt: {vl:.2}/{vc:.2} to {cl:.2}/{cc:.2}");
            }
        }
    }

    /// A path in metres: `lead` m north, then a turn of `deg` right on
    /// `radius` m sampled every `step` m (a single vertex when the radius is
    /// zero), then 100 m on.
    fn turn_route(deg: f64, radius: f64, step: f64) -> Route {
        let mut pts = vec![(0.0, 0.0), (0.0, 100.0)];
        let (cx, cy) = (radius, 100.0);
        if radius > 0.0 {
            let n = ((deg.to_radians() * radius) / step).ceil().max(1.0) as usize;
            for k in 1..=n {
                let a = (deg * k as f64 / n as f64).to_radians();
                pts.push((cx - radius * a.cos(), cy + radius * a.sin()));
            }
        }
        let last = *pts.last().unwrap();
        let out = deg.to_radians();
        pts.push((last.0 + 100.0 * out.sin(), last.1 + 100.0 * out.cos()));
        let watch = TaxiWatch {
            path: pts.iter().map(|&(x, y)| ll(x, y)).collect(),
            via: String::new(),
            seg: vec![(String::new(), false); pts.len() - 1],
            holding_point: String::new(),
            lead_on: Vec::new(),
        };
        Route::build(&watch).expect("a route")
    }

    /// #194 (sortie 1790765957, a Citation X at 17.4 kt into a fillet the
    /// vertex reading never saw): a corner is its curvature, so the same arc
    /// drawn every 1, 3 or 5 m is the same corner (its turn and radius), and
    /// the vertex reading missed the finely drawn ones entirely.
    #[test]
    fn a_corner_reads_the_same_however_finely_it_is_drawn() {
        for (deg, radius) in [(90.0, 15.0), (104.0, 20.0), (60.0, 35.0), (45.0, 50.0)] {
            let reads: Vec<Bend> = [1.0, 3.0, 5.0]
                .iter()
                .map(|&step| {
                    let r = turn_route(deg, radius, step);
                    assert_eq!(r.bends.len(), 1, "{deg} on {radius} every {step}: {:?}", r.bends);
                    r.bends[0]
                })
                .collect();
            for b in &reads {
                assert!((b.deg - deg).abs() < 3.0, "{deg} on {radius}: {b:?}");
                assert!(b.radius > 0.75 * radius && b.radius < 1.1 * radius, "{deg} on {radius}: {b:?}");
                // The slow-down is planned to the arc's start at the latest.
                assert!(b.start <= 100.5, "{deg} on {radius}: {b:?}");
            }
            let fine = turn_route(deg, radius, 1.0);
            // Chords of 15 m either side turn 30 degrees only inside 28 m.
            assert!(radius < 30.0 || vertex_bends(&fine.pts, &fine.cum).is_empty(), "the vertex reading never saw it");
        }
        // A lone vertex reads as tight as it is: 8 m over its turn, under the
        // circle the vertex reading drew through it.
        let v = turn_route(90.0, 0.0, 5.0);
        assert_eq!(v.bends.len(), 1);
        let old = vertex_bends(&v.pts, &v.cum);
        assert!((v.bends[0].radius - 8.0 / 90.0_f64.to_radians()).abs() < 0.1, "{:?}", v.bends);
        assert!(v.bends[0].radius <= old[0].radius, "{:?} vs {old:?}", v.bends);
    }

    /// The Citation's corner speed at LEMG's Quebec onto Delta as ridden
    /// since e31f338: under the vertex reading its 15 kt pace (it flew 17.4),
    /// under the curvature reading its sharp pace, 6 kt, from before the
    /// fillet begins.
    #[test]
    fn the_quebec_delta_fillet_is_a_corner() {
        let paint: Vec<Vec<(f64, f64)>> = LEMG_QD_PAINT.iter().map(|l| l.to_vec()).collect();
        let seg = vec![("Q".to_string(), false), ("D".to_string(), false)];
        let (ridden, _) = atc::nav::ride_paint(&paint, &LEMG_QD_RAW, &seg);
        let watch = TaxiWatch {
            path: ridden.clone(),
            via: String::new(),
            seg: vec![(String::new(), false); ridden.len() - 1],
            holding_point: String::new(),
            lead_on: Vec::new(),
        };
        let mut route = Route::build(&watch).expect("route");
        route.pace = Pace::from_profile(crate::aircraft::load("citation_x").expect("citation").taxi.as_ref());
        assert!(vertex_bends(&route.pts, &route.cum).is_empty(), "the vertex reading sees nothing");
        let corner = route.bends.iter().find(|b| b.deg > 90.0).copied().expect("a corner");
        println!("{corner:?}");
        assert!(corner.deg > 95.0 && corner.deg < 115.0, "{corner:?}");
        assert!(corner.radius < 25.0, "{corner:?}");
        let at_start = route.bend_cap_kts(corner.start, 15.0).expect("capped");
        assert!(at_start <= route.pace.sharp + 0.01, "{at_start:.1} kt");
    }

    /// The whole #194 route from the install: the corners the planner cut
    /// across a drawn fillet (Golf 4 onto Alpha, Bravo onto Quebec, Quebec
    /// onto Delta) are ridden round it. `cargo test -- --ignored live_lemg_194`
    #[test]
    #[ignore] // needs the real X-Plane install
    fn live_lemg_194_route_rides_its_fillets() {
        let apt = &crate::config::xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
        let Ok(Some(net)) = atc::nav::find_taxi_net(apt, "LEMG") else {
            return;
        };
        let runways = atc::nav::find_runways(apt, "LEMG").unwrap();
        let strip = runways.iter().find(|r| r.ident_a == "12" || r.ident_b == "12").expect("12/30");
        let (tlat, tlon) = if strip.ident_a == "12" { (strip.lat_a, strip.lon_a) } else { (strip.lat_b, strip.lon_b) };
        let standoff = (strip.width_m / 2.0 + 30.0).max(atc::nav::HOLD_SHORT_STANDOFF_M);
        let zone = [strip.ident_a.clone(), strip.ident_b.clone()];
        // The gate at the taxi request, both sorties.
        let plan = net
            .plan_to_runway(36.6733571, -4.4894417, tlat, tlon, standoff, &zone, Some(strip))
            .expect("a route");
        assert_eq!(plan.holding_point, "HW1");
        let geo = taxi_guide::Geo::new(plan.path[0].0);
        let pts: Vec<(f64, f64)> = plan.path.iter().map(|&p| geo.to_m(plan.path[0], p)).collect();
        let mut prev: Option<f64> = None;
        let mut sharp = 0;
        for w in pts.windows(2) {
            if taxi_guide::dist(w[0], w[1]) < 0.5 {
                continue;
            }
            let b = taxi_guide::bearing(w[0], w[1]);
            if prev.is_some_and(|p| crate::angles::abs_diff_deg(b, p) > 40.0) {
                sharp += 1;
            }
            prev = Some(b);
        }
        // Before: 12 vertices past 40 degrees (Golf 4 to Alpha -50, +84;
        // Bravo to Quebec -42, +89; Quebec to Delta +49, +45, -112, +39 near
        // enough, -83; the stand, Alpha to Golf 9, Golf 9 to Bravo). After:
        // the stand, Alpha to Golf 9 and Golf 9 to Bravo, drawn without a
        // fillet the corner turns on.
        println!("{sharp} vertices past 40 degrees");
        assert!(sharp <= 3, "{sharp}");
    }

    /// #60, LEMG 31 (sorties 1790584715 and 1790585651, 2026-09-28): the taxi
    /// rolled 7.4 m through its holding stop, and the line-up path ran from
    /// the aircraft BACK to the hold node and forward again. At rest the
    /// engage tick's joining projection read 14.8 m along (236.0 of 255.8 to
    /// go) and the next tick's plain one 0 m (250.8): `remaining_m` jumped
    /// 14.9 m with the aircraft stopped. Points behind the aircraft are
    /// dropped, so both projections of a stopped aircraft agree.
    #[test]
    fn a_line_up_from_past_the_hold_never_runs_back_to_it() {
        use crate::autoland::project_latlon;
        use crate::runway::Runway;
        let rwy = Runway {
            icao: "TEST".into(),
            id: "36".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 2000.0,
            surface: 1,
            dist_nm: 0.02,
            elev_ft: 0.0,
        };
        let at = |a_m: f64, x_m: f64| {
            let (la, lo) = project_latlon(0.0, 0.0, 0.0, a_m / 1852.0);
            project_latlon(la, lo, 90.0, x_m / 1852.0)
        };
        // The stub runs west from the hold node (40 m east) to the centreline.
        let hold = at(100.0, 40.0);
        let lead_on = vec![hold, at(100.0, 25.0), at(100.0, 10.0), at(100.0, 0.0)];
        // Stopped 7.4 m past the node, nose west down the stub.
        let (lat, lon) = at(100.0, 32.6);
        let (path, _) =
            line_up_path_from_lead_on(&rwy, lat, lon, &lead_on).expect("drives the lead-on");
        assert!(!path.contains(&hold), "the node behind the aircraft is not driven back to");
        let plain = crate::taxi_guide::project_signed(&path, lat, lon).unwrap();
        let facing = crate::taxi_guide::project_signed_facing(&path, lat, lon, 270.0).unwrap();
        assert!(
            (plain.0 - facing.0).abs() < 0.5,
            "one answer for a stopped aircraft: plain {plain:?}, facing {facing:?}"
        );
        assert!(plain.0 < 0.5, "we are at the start: {plain:?}");
        // A hold still AHEAD keeps its node, as before.
        let (blat, blon) = at(100.0, 48.0);
        let (before, _) = line_up_path_from_lead_on(&rwy, blat, blon, &lead_on).unwrap();
        assert_eq!(before[1], hold);
    }

    #[test]
    fn line_up_drives_the_authored_lead_on_when_it_is_ours() {
        use crate::autoland::project_latlon;
        use crate::runway::Runway;
        // Runway 36 from (0,0); hold 40 m east of the centreline, abeam 100 m.
        let rwy = Runway {
            icao: "TEST".into(),
            id: "36".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 2000.0,
            surface: 1,
            dist_nm: 0.02,
            elev_ft: 0.0,
        };
        let at = |a_m: f64, x_m: f64| {
            let (la, lo) = project_latlon(0.0, 0.0, 0.0, a_m / 1852.0);
            project_latlon(la, lo, 90.0, x_m / 1852.0)
        };
        let (hlat, hlon) = at(100.0, 40.0);
        // The authored yellow line: from the hold, curving onto the centreline
        // at 160 m past the threshold (two intermediate points shape the curve).
        let lead_on = vec![
            at(105.0, 38.0),
            at(125.0, 25.0),
            at(150.0, 8.0),
            at(160.0, 0.0),
        ];
        let (path, stop) =
            line_up_path_from_lead_on(&rwy, hlat, hlon, &lead_on).expect("drives the lead-on");
        // The authored points lead the drive path in order; the sharp junction
        // corner is replaced by a sampled fillet arc, then the stop continues
        // down the centreline.
        assert!(
            path.len() >= 8,
            "aircraft + authored + arc + stop: {}",
            path.len()
        );
        assert_eq!(path[1], lead_on[0]);
        assert_eq!(path[2], lead_on[1]);
        assert_eq!(path[3], lead_on[2]);
        assert!(
            (stop - 240.0).abs() < 1.0,
            "junction 160 + align 80: {stop}"
        );
        // The arc stays between the two legs: every sampled point is on the
        // hold side of the centreline and short of the stop.
        for p in &path[4..path.len() - 1] {
            let x = crate::runway::cross_track_nm(p.0, p.1, &rwy) * 1852.0;
            assert!(
                (-1.0..=40.0).contains(&x),
                "arc point off the corner: x={x}"
            );
        }
        // A lead-on from ANOTHER stub (300 m away) is refused — never drive
        // someone else's painted line.
        let foreign = vec![at(400.0, 38.0), at(460.0, 0.0)];
        assert!(line_up_path_from_lead_on(&rwy, hlat, hlon, &foreign).is_none());
        // A lead-on ending far OFF the centreline is not a runway entry.
        let sideways = vec![at(105.0, 38.0), at(160.0, 60.0)];
        assert!(line_up_path_from_lead_on(&rwy, hlat, hlon, &sideways).is_none());
        // And the picker prefers it over the synthetic diagonal for the same
        // runway.
        let cands = vec![rwy.clone()];
        let (r, plan) = pick_line_up_plan(
            &cands,
            hlat,
            hlon,
            Some("36"),
            400.0,
            false,
            Some(270.0),
            Some(&lead_on),
            TurnRound::TEARDROP,
        )
        .expect("picked");
        assert_eq!(r.id, "36");
        match plan {
            LineUpPlan::Plain(p) => assert_eq!(p[1], lead_on[0], "authored path won"),
            LineUpPlan::Backtrack(_) => panic!("no backtrack expected"),
        }
    }

    /// Luton, 2026-07-27: a full-length departure off a mid-runway holding
    /// point. The backtrack's leg one aimed at a centreline point 30-60 m
    /// nearer the threshold — 23 degrees off the stub the aircraft was
    /// standing on — and drove an MD-82 off the pavement inside fifty metres.
    /// A backtrack has to reach the runway along the DRAWN stub too; only the
    /// direction it turns at the top is ours to choose.
    #[test]
    fn a_backtrack_enters_along_the_drawn_stub_not_across_the_grass() {
        use crate::autoland::project_latlon;
        use crate::runway::{cross_track_nm, Runway};
        let rwy = Runway {
            icao: "TEST".into(),
            id: "36".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 2156.0,
            surface: 1,
            dist_nm: 0.05,
            elev_ft: 0.0,
        };
        let at = |a_m: f64, x_m: f64| {
            let (la, lo) = project_latlon(0.0, 0.0, 0.0, a_m / 1852.0);
            project_latlon(la, lo, 90.0, x_m / 1852.0)
        };
        // Hold 430 m down the strip, 45 m off to the side; the drawn stub runs
        // straight in, perpendicular, to the junction.
        let (hlat, hlon) = at(430.0, 45.0);
        let lead_on = vec![at(430.0, 0.0)];
        let path = backtrack_pocket_path_from_lead_on(&rwy, hlat, hlon, &lead_on, TurnRound::TEARDROP)
            .expect("backtrack via the stub");
        // Every point of the entry, up to the junction, stays on the stub's
        // own along-track station — it never drifts down the runway before
        // reaching the centreline, which is exactly what crossed the grass.
        let a_of = |p: (f64, f64)| -crate::runway::along_track_nm(p.0, p.1, &rwy) * 1852.0;
        let mut reached = false;
        for p in &path {
            let x = cross_track_nm(p.0, p.1, &rwy) * 1852.0;
            if x.abs() <= 1.0 {
                reached = true;
            }
            if !reached {
                assert!(
                    (a_of(*p) - 430.0).abs() < 25.0,
                    "entry wandered down the runway before reaching it: a={}",
                    a_of(*p)
                );
            }
        }
        assert!(reached, "the entry never reaches the centreline");
        // ...and it still ends in the turn pocket by the numbers.
        let last = *path.last().unwrap();
        assert!(
            a_of(last) < BACKTRACK_TURN_START_M + 5.0,
            "ends at the numbers"
        );
        // The synthetic fallback is what it replaces: that one leaves the stub
        // immediately, which is the bug.
        let synth = backtrack_pocket_path(&rwy, hlat, hlon, TurnRound::TEARDROP).expect("synthetic exists");
        assert!(
            (a_of(synth[1]) - 430.0).abs() > 25.0,
            "the old diagonal aimed down the runway from the start"
        );
    }

    #[test]
    fn line_up_pick_honors_the_cleared_runway() {
        use crate::runway::Runway;
        let mk = |id: &str, course: f64, thr: (f64, f64), dist: f64| Runway {
            icao: "TEST".into(),
            id: id.into(),
            thr_lat: thr.0,
            thr_lon: thr.1,
            course_deg: course,
            length_m: 2000.0,
            surface: 1,
            dist_nm: dist,
            elev_ft: 0.0,
        };
        // Both ends of one strip: 36 at the near threshold, 18 at the far.
        let far = crate::autoland::project_latlon(0.0, 0.0, 0.0, 2000.0 / 1852.0);
        let cands = vec![mk("36", 0.0, (0.0, 0.0), 0.02), mk("18", 180.0, far, 1.05)];
        let (hlat, hlon) = crate::autoland::project_latlon(0.0, 0.0, 90.0, 40.0 / 1852.0);
        // No clearance on record: geometry picks the near end.
        let (r, _) = pick_line_up(&cands, hlat, hlon, None, None).expect("pick");
        assert_eq!(r.id, "36");
        // The tower cleared 36: same answer, crosschecked.
        assert_eq!(
            pick_line_up(&cands, hlat, hlon, Some("36"), None)
                .unwrap()
                .0
                .id,
            "36"
        );
        // The tower cleared 18 and we're at the 36 end: nothing safe to
        // drive onto — the far end is a taxi clearance, not a line-up.
        assert!(pick_line_up(&cands, hlat, hlon, Some("18"), None).is_none());
    }

    /// LILB 03 (2026-09-25): turned round 38 m past the numbers on a 600 m
    /// strip, leg two used to drive 114 m and leave 448. Now it settles and
    /// straightens inside 60 m.
    #[test]
    fn a_backtracks_leg_two_spends_little_runway() {
        use crate::runway::{along_track_nm, Runway};
        let rwy = Runway {
            icao: "LILB".into(),
            id: "03".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 600.0,
            surface: 1,
            dist_nm: 0.0,
            elev_ft: 0.0,
        };
        let on = crate::autoland::project_latlon(0.0, 0.0, 0.0, 38.0 / 1852.0);
        let (lat, lon) = crate::autoland::project_latlon(on.0, on.1, 90.0, 3.0 / 1852.0);
        let path = backtrack_leg_two_path(&rwy, lat, lon, TurnRound::TEARDROP).expect("fits");
        let stop = path.last().unwrap();
        let stop_m = -along_track_nm(stop.0, stop.1, &rwy) * 1852.0;
        assert!(stop_m <= 38.0 + 60.0 + 0.5, "stops by {stop_m:.0} m");
        assert!(600.0 - stop_m >= 500.0, "leaves {:.0} m", 600.0 - stop_m);
        // The old hold-short geometry spent nearly twice that.
        let (_, old_stop) = line_up_stops_m(&rwy, lat, lon, None).unwrap();
        assert!(old_stop > stop_m + 40.0, "old {old_stop:.0} vs new {stop_m:.0}");
    }

    #[test]
    fn backtrack_drives_to_a_pocket_beside_the_numbers() {
        use crate::runway::{along_track_nm, cross_track_nm, Runway};
        // The LXGB shape: a 1,712 m runway, hold-short stub 1,170 m down it
        // (holding point Charlie left 580 m ahead for a 09 departure,
        // 2026-07-22). Runway 36 from (0,0) for test geometry.
        let rwy = Runway {
            icao: "TEST".into(),
            id: "36".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 1712.0,
            surface: 1,
            dist_nm: 0.6,
            elev_ft: 0.0,
        };
        let mid = crate::autoland::project_latlon(0.0, 0.0, 0.0, 1170.0 / 1852.0);
        let (hlat, hlon) = crate::autoland::project_latlon(mid.0, mid.1, 90.0, 40.0 / 1852.0);
        let path = backtrack_pocket_path(&rwy, hlat, hlon, TurnRound::TEARDROP).expect("feasible");
        // The route heads TOWARD the threshold: each point's along-track
        // shrinks, and the pocket ends up beside the numbers, off the
        // centreline (turning room, never a pivot).
        let along: Vec<f64> = path
            .iter()
            .map(|&(a, b)| -along_track_nm(a, b, &rwy) * 1852.0)
            .collect();
        assert!(
            along.windows(2).all(|w| w[1] < w[0] + 1.0),
            "monotonic toward the numbers: {along:?}"
        );
        let (plat, plon) = *path.last().unwrap();
        let pocket_along = -along_track_nm(plat, plon, &rwy) * 1852.0;
        let pocket_off = (cross_track_nm(plat, plon, &rwy) * 1852.0).abs();
        assert!(
            pocket_along < 60.0,
            "pocket beside the numbers: {pocket_along}"
        );
        assert!(
            pocket_off > 8.0 && pocket_off < 20.0,
            "turning room: {pocket_off}"
        );
        // A line-up FROM the pocket is feasible — that's leg two, the 180.
        assert!(
            line_up_path(&rwy, plat, plon, None).is_some(),
            "leg two must exist"
        );
        // Near the numbers already: no backtrack, the plain line-up owns it.
        let near = crate::autoland::project_latlon(0.0, 0.0, 0.0, 60.0 / 1852.0);
        let (nlat, nlon) = crate::autoland::project_latlon(near.0, near.1, 90.0, 40.0 / 1852.0);
        assert!(backtrack_pocket_path(&rwy, nlat, nlon, TurnRound::TEARDROP).is_none());
    }

    #[test]
    fn line_up_plan_backtracks_to_the_numbers_from_a_deep_intersection() {
        use crate::runway::Runway;
        let rwy = Runway {
            icao: "TEST".into(),
            id: "36".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 1712.0,
            surface: 1,
            dist_nm: 0.6,
            elev_ft: 0.0,
        };
        let cands = vec![rwy];
        // 1,170 m down the strip: a deep intersection on a short field. The
        // plain line-up would leave ~460 m — enough for a 172's roll — but a
        // departure uses the FULL length, so we backtrack to the numbers even
        // when it fits (the LXGB Charlie case the pilot kept flagging).
        let mid = crate::autoland::project_latlon(0.0, 0.0, 0.0, 1170.0 / 1852.0);
        let (hlat, hlon) = crate::autoland::project_latlon(mid.0, mid.1, 90.0, 40.0 / 1852.0);
        assert!(matches!(
            pick_line_up_plan(&cands, hlat, hlon, Some("36"), 400.0, false, None, None, TurnRound::TEARDROP),
            Some((_, LineUpPlan::Backtrack(_)))
        ));
        assert!(matches!(
            pick_line_up_plan(&cands, hlat, hlon, Some("36"), 700.0, false, None, None, TurnRound::TEARDROP),
            Some((_, LineUpPlan::Backtrack(_)))
        ));
        // The pilot asked for the full length in so many words: backtrack
        // even though the plain remaining would do.
        assert!(matches!(
            pick_line_up_plan(&cands, hlat, hlon, Some("36"), 400.0, true, None, None, TurnRound::TEARDROP),
            Some((_, LineUpPlan::Backtrack(_)))
        ));
        // Abeam the numbers a backtrack is impossible — the ask still gets
        // the plain line-up, never a refusal.
        let near = crate::autoland::project_latlon(0.0, 0.0, 0.0, 60.0 / 1852.0);
        let (nlat, nlon) = crate::autoland::project_latlon(near.0, near.1, 90.0, 40.0 / 1852.0);
        assert!(matches!(
            pick_line_up_plan(&cands, nlat, nlon, Some("36"), 700.0, true, None, None, TurnRound::TEARDROP),
            Some((_, LineUpPlan::Plain(_)))
        ));
    }

    #[test]
    fn short_routes_refuse_to_build() {
        let w = TaxiWatch {
            path: vec![ll(0.0, 0.0), ll(15.0, 0.0)],
            via: "A".into(),
            seg: vec![("A".into(), false)],
            holding_point: "A1".into(),
            lead_on: Vec::new(),
        };
        assert!(Route::build(&w).is_none(), "nothing to drive on 15 m");
    }

    /// Every stop turned the aircraft, and the guidance was doing it.
    ///
    /// The pursuit aim point clamps at the route's final point, so once the
    /// aircraft rolls past it the bearing to that point swings BEHIND and the
    /// law commands a hard turn toward a waypoint already passed. EGHI
    /// 2026-07-30, dead on the centreline at the stop point and then:
    ///
    ///   rem  0.04 -> err   9.5     rem -3.6 -> err  38.6
    ///   rem -4.4  -> err  70.2     rem -5.0 -> err 162.6
    ///
    /// 0.07 m off the line to 3.00 m in four seconds, steering pinned. It hit
    /// the hold, the line-up, the backtrack and the taxi-in alike, because all
    /// four drive this one loop.
    /// LIPR Alpha (2026-09-26): the planned route ended on a repeated point,
    /// the zero-length last segment read as due north, and with 8 m to run
    /// the stop steered a 220-degree aircraft full right toward it.
    #[test]
    fn a_repeated_last_point_is_not_a_final_bearing() {
        let w = TaxiWatch {
            path: vec![ll(0.0, 0.0), ll(0.0, -120.0), ll(0.0, -120.0)],
            via: String::new(),
            seg: vec![(String::new(), false), (String::new(), false)],
            holding_point: String::new(),
            lead_on: Vec::new(),
        };
        let r = Route::build(&w).expect("drives");
        let bearing = r.final_bearing_deg().expect("a real segment exists");
        assert!((bearing - 180.0).abs() < 2.0, "due south route, got {bearing:.1}");
    }

    #[test]
    fn the_stop_holds_the_routes_heading_instead_of_chasing_a_passed_point() {
        // A straight route heading due east, 120 m long.
        let w = TaxiWatch {
            path: vec![ll(0.0, 0.0), ll(120.0, 0.0)],
            via: String::new(),
            seg: vec![(String::new(), false)],
            holding_point: String::new(),
            lead_on: Vec::new(),
        };
        let r = Route::build(&w).expect("a straight two-point route drives");
        let bearing = r.final_bearing_deg().expect("a two-point route has one");
        assert!(
            (bearing - 90.0).abs() < 2.0,
            "due east route, got {bearing:.1}"
        );

        // Sitting PAST the end, pointed correctly down the route. The pursuit
        // error is nonsense here; the stop error must be ~0.
        let past = ll(125.0, 0.0);
        let (past_lat, past_lon) = (past.0, past.1);
        let progress = r.total;
        let pursuit = r.heading_err_deg(past_lat, past_lon, 90.0, progress, 6.0);
        let held = r.stop_heading_err_deg(past_lat, past_lon, 90.0, progress, 6.0);
        assert!(
            pursuit.abs() > 90.0,
            "the pursuit error should be wild past the end: {pursuit:.1}"
        );
        assert!(
            held.abs() < 2.0,
            "holding the route heading must read straight: {held:.1}"
        );

        // And it still CORRECTS a genuinely crooked nose rather than going
        // blind — 20° off reads as 20° off, with the sign that unwinds it.
        let crooked = r.stop_heading_err_deg(past_lat, past_lon, 110.0, progress, 6.0);
        assert!(
            (crooked + 20.0).abs() < 2.0,
            "20 deg right of course should command left: {crooked:.1}"
        );

        // The swap only happens near the stop; further out the pursuit still
        // owns the steering, which is what folds cross-track back in.
        assert!(
            STOP_HEADING_HOLD_M > 0.0 && STOP_HEADING_HOLD_M < 20.0,
            "the hold zone is the last few metres, not the whole leg"
        );
    }

    /// EDLP (2026-09-24): with a standing steer bias the proportional law
    /// parks off the line (the SF50 held 4 m on every straight). Flown with
    /// the SF50's own pace through a lagged yaw plant that needs 0.09 of
    /// steer to run straight: the trim takes the offset out, and it does not
    /// start a weave.
    #[test]
    fn a_straight_trim_takes_out_the_offset_a_standing_bias_leaves() {
        let pace = Pace::from_profile(
            crate::aircraft::load("cirrus_vision_sf50")
                .expect("sf50")
                .taxi
                .as_ref(),
        );
        // (final |offset| over the last 20 s, peak |offset| after 60 s)
        let fly = |trimmed: bool| -> (f64, f64) {
            let (bias, k_dps, tau, gs) = (0.09, 6.9, 0.8, 12.0);
            let v = gs * 0.514_444;
            let (mut y, mut psi, mut r, mut prev) = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
            let mut trim = StraightTrim::default();
            let dt = 0.2;
            let (mut late, mut after) = (0.0_f64, 0.0_f64);
            let look = lookahead_m_with(gs, pace.lookahead_s);
            for i in 0..900 {
                let t = i as f64 * dt;
                let err = (-y).atan2(look).to_degrees() - psi;
                let tr = if trimmed { trim.step(y, 0.0, gs, false, 1000.0, dt) } else { 0.0 };
                let want = (steer_cmd_with(
                    damped_err_deg(err, r, 0.0, pace.steer_lead_s),
                    pace.steer_full_deg,
                ) + tr)
                    .clamp(-1.0, 1.0);
                let steer = slew_steer(prev, want, pace.steer_slew_per_s, dt);
                prev = steer;
                r += (k_dps * (steer - bias) - r) * (dt / tau).min(1.0);
                psi += r * dt;
                y += v * psi.to_radians().sin() * dt;
                if t > 60.0 {
                    after = after.max(y.abs());
                }
                if t > 160.0 {
                    late = late.max(y.abs());
                }
            }
            (late, after)
        };
        let (untrimmed, _) = fly(false);
        assert!(untrimmed > 2.0, "the plant must reproduce the offset: {untrimmed:.1} m");
        let (trimmed, peak) = fly(true);
        assert!(trimmed < 0.5, "trimmed out: {trimmed:.2} m");
        assert!(peak < untrimmed, "no weave bigger than the offset it replaced: {peak:.1} m");
    }

    /// LILB (2026-09-25): the Evolution's brake pulsed with every flicker of
    /// the rate lead. Flown on its own pace through a brake-steered yaw plant
    /// that pulls left (0.05 of right brake to run straight), with the yaw
    /// rate read through the loop's own filter from a heading that carries
    /// +/-0.15 degrees of noise: the smoothed brake moves far less, and the
    /// aircraft tracks no worse.
    #[test]
    fn a_smoothed_brake_holds_a_straight_without_pulsing() {
        let pace = Pace::from_profile(
            crate::aircraft::load("lancair_evolution").expect("evot").taxi.as_ref(),
        );
        assert!(pace.brake_steer);
        // (total brake travel over the run, peak |offset| after 30 s)
        let fly = |tau: f64| -> (f64, f64) {
            let (k_dps, pull, tau_plant, gs, dt) = (60.0, 3.0, 0.6, 12.0, 0.2);
            let v = gs * 0.514_444;
            let (mut y, mut psi, mut r) = (1.5_f64, 0.0_f64, 0.0_f64);
            let (mut prev, mut bs, mut yaw, mut prev_meas) = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
            let mut trim = StraightTrim::default();
            let (mut travel, mut last_b, mut peak) = (0.0_f64, 0.0_f64, 0.0_f64);
            let mut seed = 7u64;
            let look = lookahead_m_with(gs, pace.lookahead_s);
            for i in 0..900 {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                let noise = ((seed >> 33) as f64 / (1u64 << 31) as f64 - 0.5) * 0.3;
                let meas = psi + noise;
                yaw = yaw * 0.5 + ((meas - prev_meas) / dt) * 0.5;
                prev_meas = meas;
                let err = (-y).atan2(look).to_degrees() - psi;
                let tr = trim.step(y, 0.0, gs, false, 1000.0, dt);
                let want = (steer_cmd_with(damped_err_deg(err, yaw, 0.0, pace.steer_lead_s), pace.steer_full_deg) + tr)
                    .clamp(-1.0, 1.0);
                let steer = slew_steer(prev, want, pace.steer_slew_per_s, dt);
                prev = steer;
                bs = if tau > 0.0 { smooth_toward(bs, steer, tau, dt) } else { steer };
                let (bl, br) = brake_steer_split(bs, gs);
                let b = br - bl;
                travel += (b - last_b).abs();
                last_b = b;
                r += (k_dps * b - pull - r) * (dt / tau_plant).min(1.0);
                psi += r * dt;
                y += v * psi.to_radians().sin() * dt;
                if i as f64 * dt > 30.0 {
                    peak = peak.max(y.abs());
                }
            }
            (travel, peak)
        };
        let (raw_travel, raw_peak) = fly(0.0);
        let (smooth_travel, smooth_peak) = fly(BRAKE_STEER_TAU_S);
        assert!(
            smooth_travel < raw_travel * 0.6,
            "the brake must move far less: {smooth_travel:.2} vs {raw_travel:.2}"
        );
        assert!(
            smooth_peak <= raw_peak.max(1.0),
            "and track no worse: {smooth_peak:.2} m vs {raw_peak:.2} m"
        );
    }

    /// LIME 2026-09-25: 23 kt against a 10 kt target, into a corner, off
    /// the pavement and into a wall. That is a stop, not a trim.
    #[test]
    fn a_runaway_taxi_is_stopped() {
        assert_eq!(taxi_overspeed(23.4, 10.0), Some("over_target"));
        assert_eq!(taxi_overspeed(26.0, 30.0), Some("absolute"));
        assert_eq!(taxi_overspeed(17.7, 10.0), None, "7.7 over is the brakes' to fix");
        assert_eq!(taxi_overspeed(12.0, 10.0), None);
        assert_eq!(taxi_overspeed(5.0, 0.0), None, "a stop in progress is the stop's");
    }

    #[test]
    fn a_gentle_bend_shortens_the_aim_and_a_straight_does_not() {
        // LIME: 12 kt, a 3.3 deg/s bend (radius ~107 m).
        let cap = bend_lookahead_cap_m(12.0, -3.3);
        assert!((17.0..19.5).contains(&cap), "{cap}");
        let full = lookahead_m_with(12.0, 9.0);
        assert!(full > 50.0 && full.min(cap) < 20.0);
        // The cut the capped aim leaves: under the limit.
        let r = 12.0 * 0.514_444 / 3.3_f64.to_radians();
        assert!(cap * cap / (2.0 * r) <= BEND_MAX_CUT_M + 1e-9);
        // Straight, or stopped: no cap. A sharp corner: never under 16,
        // crawling too (a corner is a radius, whatever speed reads it; #165).
        assert!(bend_lookahead_cap_m(12.0, 0.1).is_infinite());
        assert!(bend_lookahead_cap_m(0.0, 10.0).is_infinite());
        assert_eq!(bend_lookahead_cap_m(0.5, 10.0), 16.0);
        assert_eq!(bend_lookahead_cap_m(8.0, 30.0), 16.0);
        // Straight at a crawl is the same radius as straight at 0.5 m/s.
        assert!(bend_lookahead_cap_m(0.5, 0.05).is_infinite());
        assert!(bend_lookahead_cap_floor_m(0.9, 0.9, 12.0) == 12.0);
    }

    /// The aligning pivot on the fitted plant ([`super::yaw_plant`]): from
    /// the SR22's 10.8 and the Evolution's 24.6 degrees (the roll's first
    /// ticks, 1790690995 and 1790712974, 2026-09-29), at the iMac's 0.3 to
    /// 0.5 s ticks and on a softer and a stiffer plant, it comes round and
    /// STAYS within the aligned band, never swinging more than 2 degrees
    /// through. The old law (full brake from 15 degrees on the error alone,
    /// let go only past 15 deg/s) on the same plant pirouettes, as flown.
    #[test]
    fn the_aligning_pivot_catches_its_swing_on_the_imac_plant() {
        use super::yaw_plant::{signed, Plant};
        let old_law = |err: f64, yaw_diff: f64| {
            if err.abs() <= 3.0 {
                return 0.0;
            }
            let want = (err / 15.0).clamp(-1.0, 1.0);
            let (l, r) = super::pivot_brake_split(want, 2.0);
            signed((l, r)) * super::pivot_rate_scale(want, yaw_diff)
        };
        // As flown: the old law swings through by more than the start.
        let mut p = Plant::new(140.0, 3.0, 10.8, 0.0);
        let (mut prev, mut worst) = (p.err, 0.0_f64);
        for _ in 0..48 {
            let yaw_diff = -(p.err - prev) / 0.42;
            prev = p.err;
            p.run(old_law(p.err, yaw_diff), 1.0, 0.42);
            worst = worst.max(-p.err);
        }
        assert!(worst > 8.0, "the old law pirouettes on this plant: {worst:.1}");

        for (k, d) in [(140.0, 3.0), (70.0, 1.5), (100.0, 1.0), (200.0, 4.0)] {
            for tick in [0.3, 0.42, 0.5] {
                for e0 in [10.8, 24.6, -10.8, 5.0] {
                    let mut p = Plant::new(k, d, e0, 0.0);
                    let mut through = 0.0_f64;
                    let mut settled_at: Option<f64> = None;
                    let mut t = 0.0;
                    while t < 25.0 {
                        p.run(signed(super::align_pivot_brakes(p.err, p.yaw)), 1.0, tick);
                        t += tick;
                        through = through.max(-p.err * e0.signum());
                        if p.err.abs() <= super::ALIGNED_DEG {
                            settled_at.get_or_insert(t);
                        } else {
                            settled_at = None;
                        }
                    }
                    let why = format!("k {k} d {d} tick {tick} from {e0}");
                    assert!(through <= 2.5, "{why}: swung {through:.1} through");
                    let at = settled_at.unwrap_or_else(|| panic!("{why}: never settled"));
                    assert!(at < 20.0, "{why}: settled at {at:.1} s");
                }
            }
        }
    }

    #[test]
    fn the_aligning_pivot_brakes_the_inside_wheel_and_catches_a_swing() {
        // The SR22's roll start: 10.8 off, still: the right (inside) brake.
        let (l, r) = super::align_pivot_brakes(10.8, 0.0);
        assert!(l == 0.0 && r > 0.1, "{l} {r}");
        // Turning into it at 6 deg/s: the lead has already let go and holds
        // the swing on the OTHER brake (10.8 - 12 is past the course).
        let (l, r) = super::align_pivot_brakes(10.8, 6.0);
        assert!(l > 0.0 && r == 0.0, "{l} {r}");
        // Straight and still: nothing.
        assert_eq!(super::align_pivot_brakes(0.5, 0.3), (0.0, 0.0));
        // Never more than a pivot's brake.
        let (l, _) = super::align_pivot_brakes(-170.0, 0.0);
        assert!(l <= 0.45 + 1e-9);
    }

    /// The line-up's end, on the plant with the governor: the SR22 arrived
    /// at LEMG 12 5.0 degrees off its final bearing, yawing away at 1.7
    /// deg/s at 2.1 kt (1790690995, t 1790692240.7), and was declared lined
    /// up; it stopped at 107 true on a 117.7 runway. The Evolution's LILB 03
    /// turn-round arrived 11.8 off (13 at the stop) yawing away at 2.3
    /// (1790712974, t 1790713229.1). Now the drive ends only STOPPED within
    /// [`super::ALIGNED_DEG`], in bounded time and a few metres of runway.
    #[test]
    fn a_line_up_ends_stopped_and_pointing_down_the_runway() {
        use super::yaw_plant::{signed, Plant};
        use super::{AlignStep, LineUpAlign};
        for (course, psi0, yaw0, gs0) in [(117.7, 112.7, -1.7, 2.1), (24.0, 12.2, -2.3, 1.9)] {
            for (k, d) in [(140.0, 3.0), (70.0, 1.5)] {
                let mut judge = LineUpAlign::new(course);
                let e0 = judge.err_deg(psi0);
                let mut p = Plant::new(k, d, e0, yaw0);
                let (mut gs, mut rolled, mut t) = (gs0, 0.0_f64, 0.0_f64);
                let tick = 0.42;
                let end = loop {
                    let psi = course - p.err;
                    match judge.step(t, psi, gs, p.yaw, rolled) {
                        AlignStep::Align { bl, br, creep } => {
                            // The governor: toward the creep at 0.7 kt/s,
                            // or braked to a stop at 1.5 kt/s.
                            let want = if creep { super::ALIGN_CREEP_KTS } else { 0.0 };
                            gs = if gs < want { (gs + 0.7 * tick).min(want) } else { (gs - 1.5 * tick).max(want) };
                            // A braked wheel turns a castoring nose only
                            // with the aircraft moving.
                            p.run(signed((bl, br)), (gs / 1.5).min(1.0), tick);
                            rolled += gs / 1.943_844 * tick;
                        }
                        other => break other,
                    }
                    t += tick;
                };
                let why = format!("course {course} k {k}");
                match end {
                    AlignStep::Done { err_deg } => {
                        assert!(err_deg.abs() <= super::ALIGNED_DEG, "{why}: {err_deg}")
                    }
                    other => panic!("{why}: {other:?}"),
                }
                assert!(t < super::ALIGN_MAX_S, "{why}: {t:.1} s");
                assert!(rolled < 25.0, "{why}: rolled {rolled:.1} m");
            }
        }
    }

    /// LILB 03 as apt.dat draws it: 601 m of 15 m pavement.
    fn lilb_03() -> crate::runway::Runway {
        let (a, b) = ((45.7671573, 9.1605458), (45.7720916, 9.1636892));
        crate::runway::Runway {
            icao: "LILB".into(),
            id: "03".into(),
            thr_lat: a.0,
            thr_lon: a.1,
            course_deg: crate::autoland::bearing_deg(a.0, a.1, b.0, b.1),
            length_m: crate::autoland::haversine_nm(a.0, a.1, b.0, b.1) * 1852.0,
            surface: 1,
            dist_nm: 0.0,
            elev_ft: 1258.0,
        }
    }

    /// The alignment as flown at LILB 03 (1790721973, 2026-09-30), from the
    /// turn-round's stop: 12.99 off, swinging -5.56 deg/s at 1.44 kt, at
    /// 45.767857 9.160965 (85 m down a 601 m strip, 16.1 m more than the
    /// 500 m fit). The Evolution's idle thrust runs a creep up at 0.2 kt/s
    /// whatever the governor asks, and a standing yaw of 0.025 brake left
    /// held the nose 2.0 to 2.2 off against the soft law (0.02 of brake
    /// right, still, for 20 s). On that plant the old law creeps its whole
    /// 40 m 2.2 off, as flown (40.7 m, 474.8 m left); now the trim takes the
    /// standing yaw out, the creep is stopped past 3 kt, and the roll stays
    /// inside the 16.1 m the fit can spare. And on the SR22's governor, which
    /// holds its creep, the same.
    #[test]
    fn a_line_up_spends_only_the_runway_the_fit_can_spare() {
        use super::yaw_plant::{signed, Plant};
        use super::{AlignStep, LineUpAlign};
        let rwy = lilb_03();
        let (lat, lon) = (45.767_857_048, 9.160_965_298);
        let spare = super::align_spare_m(&rwy, 500.0, lat, lon);
        assert!((spare - 16.1).abs() < 0.5, "spare {spare:.1}");
        // The Evolution's governor: idle thrust accelerates any creep; a
        // stop brakes at 1.5 kt/s. The SR22's holds the creep.
        let idle_push = |gs: f64, want: f64, tick: f64| {
            if want > 0.0 {
                gs + 0.2 * tick
            } else {
                (gs - 1.5 * tick).max(0.0)
            }
        };
        let holds = |gs: f64, want: f64, tick: f64| {
            if gs < want {
                (gs + 0.7 * tick).min(want)
            } else {
                (gs - 1.5 * tick).max(want)
            }
        };
        let course = rwy.course_deg;
        let pull = -0.025;
        // The old law on the same plant: the soft brake alone, a creep the
        // governor cannot hold, the flat 40 m bound.
        {
            let mut p = Plant::new(140.0, 3.0, 12.99, -5.56);
            let (mut gs, mut rolled, mut creeping) = (1.44_f64, 0.0_f64, false);
            while rolled < super::ALIGN_MAX_ROLL_M {
                let led = p.err + p.yaw * super::ALIGN_LEAD_S * -1.0;
                creeping = if creeping {
                    !(p.err.abs() <= 1.0 && led.abs() <= 2.0)
                } else {
                    p.err.abs() > 2.0 || led.abs() > 2.0
                };
                let cmd = signed(super::align_pivot_brakes(p.err, p.yaw)) + pull;
                gs = idle_push(gs, if creeping { 2.0 } else { 0.0 }, 0.3);
                p.run(cmd, (gs / 1.5).min(1.0), 0.3);
                rolled += gs / 1.943_844 * 0.3;
            }
            eprintln!("old law: {:.2} off at {gs:.1} kt after {rolled:.1} m", p.err);
            assert!(p.err.abs() > 1.9 && gs > 5.0, "as flown: {:.1} off at {gs:.1} kt", p.err);
        }
        for (name, gov) in [("Evolution", &idle_push as &dyn Fn(f64, f64, f64) -> f64), ("SR22", &holds)] {
            for (k, d) in [(140.0, 3.0), (70.0, 1.5)] {
                for tick in [0.3, 0.45] {
                    let mut judge = LineUpAlign::with_spare(course, Some(spare));
                    let mut p = Plant::new(k, d, 12.99, -5.56);
                    let (mut gs, mut rolled, mut t) = (1.44_f64, 0.0_f64, 0.0_f64);
                    let end = loop {
                        match judge.step(t, course - p.err, gs, p.yaw, rolled) {
                            AlignStep::Align { bl, br, creep } => {
                                gs = gov(gs, if creep { judge.creep_kts } else { 0.0 }, tick);
                                // The standing yaw only while rolling.
                                p.run(signed((bl, br)) + pull, (gs / 1.5).min(1.0), tick);
                                rolled += gs / 1.943_844 * tick;
                            }
                            other => break other,
                        }
                        t += tick;
                    };
                    // The caller stops it: the governor's brake from here.
                    while gs > 0.0 {
                        gs = gov(gs, 0.0, tick);
                        rolled += gs / 1.943_844 * tick;
                    }
                    let why = format!("{name} k {k} tick {tick}");
                    eprintln!("{why}: {end:?} rolled {rolled:.1} m in {t:.1} s, trim {:.3}", judge.trim);
                    match end {
                        AlignStep::Done { err_deg } => {
                            assert!(err_deg.abs() <= super::ALIGN_ACCEPT_DEG, "{why}: {err_deg}")
                        }
                        other => panic!("{why}: {other:?}"),
                    }
                    assert!(rolled <= spare, "{why}: rolled {rolled:.1} of {spare:.1} m");
                    assert!(t < super::ALIGN_MAX_S, "{why}: {t:.1} s");
                }
            }
        }
    }

    /// The alignment closes the lateral as well as the heading: the LILB 03
    /// turn-round (1790740292, 2026-09-30) stopped its leg two 1.95 m left
    /// of the centreline, 11.7 degrees off and swinging -2.1 deg/s at 0.84
    /// kt with 61.9 m to spare, straightened the nose in 8 m and the roll
    /// began 2.3 m off (alignment FAIL at 2). On the yaw plant (a standing
    /// yaw, the Evolution's idle creep and the SR22's held one) the heading
    /// alone ends as flown, 2 m off or more; now it ends inside 1.5 m and
    /// within the aligned heading, inside the runway it may roll. With only
    /// 16 m to spare there is no room to close, and it aligns as before.
    #[test]
    fn a_line_up_closes_the_lateral_as_well_as_the_heading() {
        use super::yaw_plant::{signed, Plant};
        use super::{AlignStep, LineUpAlign};
        let course = 23.96;
        let idle_push = |gs: f64, want: f64, tick: f64| {
            if want > 0.0 {
                gs + 0.2 * tick
            } else {
                (gs - 1.5 * tick).max(0.0)
            }
        };
        let holds = |gs: f64, want: f64, tick: f64| {
            if gs < want {
                (gs + 0.7 * tick).min(want)
            } else {
                (gs - 1.5 * tick).max(want)
            }
        };
        let pull = -0.025;
        // One alignment from the logged stop; `on_line` hands it the lateral.
        let fly = |spare: f64, gov: &dyn Fn(f64, f64, f64) -> f64, k: f64, d: f64, tick: f64, on_line: bool| {
            let mut judge = LineUpAlign::with_spare(course, Some(spare));
            let mut p = Plant::new(k, d, 11.68, -2.06);
            let (mut gs, mut rolled, mut t, mut lat) = (0.84_f64, 0.0_f64, 0.0_f64, -1.95_f64);
            let end = loop {
                let step = judge.step_on_line(t, course - p.err, gs, p.yaw, rolled, on_line.then_some(lat));
                match step {
                    AlignStep::Align { bl, br, creep } => {
                        gs = gov(gs, if creep { judge.creep_kts } else { 0.0 }, tick);
                        let e0 = p.err;
                        p.run(signed((bl, br)) + pull, (gs / 1.5).min(1.0), tick);
                        let m = gs / 1.943_844 * tick;
                        // Right of the course (a negative error) moves right.
                        lat -= m * ((e0 + p.err) / 2.0).to_radians().sin();
                        rolled += m;
                    }
                    other => break other,
                }
                t += tick;
            };
            // The caller stops it: the governor's brake from here.
            while gs > 0.0 {
                gs = gov(gs, 0.0, tick);
                let m = gs / 1.943_844 * tick;
                lat -= m * p.err.to_radians().sin();
                rolled += m;
            }
            (end, lat, rolled, t, judge.closed_lateral)
        };
        for (name, gov) in [("Evolution", &idle_push as &dyn Fn(f64, f64, f64) -> f64), ("SR22", &holds)] {
            for (k, d) in [(140.0, 3.0), (70.0, 1.5)] {
                for tick in [0.3, 0.45] {
                    let why = format!("{name} k {k} tick {tick}");
                    let (_, lat, ..) = fly(61.9, gov, k, d, tick, false);
                    assert!(lat.abs() > 2.0, "{why}: the heading alone, as flown: {lat:.2}");
                    let (end, lat, rolled, t, closed) = fly(61.9, gov, k, d, tick, true);
                    eprintln!("{why}: {end:?} lat {lat:.2} rolled {rolled:.1} m in {t:.1} s");
                    assert!(closed, "{why}");
                    match end {
                        AlignStep::Done { err_deg } => {
                            assert!(err_deg.abs() <= super::ALIGNED_DEG, "{why}: {err_deg}")
                        }
                        other => panic!("{why}: {other:?}"),
                    }
                    assert!(lat.abs() <= super::ALIGN_LATERAL_M, "{why}: {lat:.2} m off");
                    assert!(rolled <= super::align_roll_limit_m(Some(61.9)), "{why}: {rolled:.1} m");
                    // No room: aligned as before, never past the spare.
                    let (end, _, rolled, ..) = fly(16.0, gov, k, d, tick, true);
                    assert!(matches!(end, AlignStep::Done { .. }), "{why}: {end:?}");
                    assert!(rolled <= 16.0, "{why}: rolled {rolled:.1} of 16");
                }
            }
        }
        // Inside 1.5 m the lateral is left alone: the heading's law alone.
        let mut a = LineUpAlign::with_spare(course, Some(61.9));
        let mut b = LineUpAlign::with_spare(course, Some(61.9));
        for i in 0..20 {
            let t = i as f64 * 0.4;
            let psi = course - 8.0 + i as f64 * 0.3;
            assert_eq!(a.step(t, psi, 1.5, 1.0, t), b.step_on_line(t, psi, 1.5, 1.0, t, Some(1.4)));
        }
        assert_eq!(super::align_intercept_deg(-2.3), 8.0);
        assert!((super::align_intercept_deg(1.0) + 4.2).abs() < 1e-9);
        assert_eq!(super::align_intercept_deg(0.2), 0.0);
    }

    /// With the fit already marginal the alignment pivots nearly in place:
    /// the tight creep, and never past the runway it can spare, whatever it
    /// then says about the heading.
    #[test]
    fn a_marginal_fit_pivots_nearly_in_place() {
        use super::yaw_plant::{signed, Plant};
        use super::{AlignStep, LineUpAlign};
        assert_eq!(super::align_roll_limit_m(None), super::ALIGN_MAX_ROLL_M);
        assert_eq!(super::align_roll_limit_m(Some(-3.0)), 0.0);
        assert_eq!(super::align_roll_limit_m(Some(100.0)), super::ALIGN_MAX_ROLL_M);
        for spare in [3.0, 6.0, 10.0] {
            let mut judge = LineUpAlign::with_spare(24.0, Some(spare));
            assert_eq!(judge.creep_kts, super::ALIGN_TIGHT_CREEP_KTS);
            let mut p = Plant::new(140.0, 3.0, 11.8, -2.3);
            let (mut gs, mut rolled, mut t, tick) = (0.5_f64, 0.0_f64, 0.0_f64, 0.42);
            let end = loop {
                match judge.step(t, 24.0 - p.err, gs, p.yaw, rolled) {
                    AlignStep::Align { bl, br, creep } => {
                        gs = if creep { gs + 0.2 * tick } else { (gs - 1.5 * tick).max(0.0) };
                        p.run(signed((bl, br)) - 0.025, (gs / 1.5).min(1.0), tick);
                        rolled += gs / 1.943_844 * tick;
                    }
                    other => break other,
                }
                t += tick;
            };
            while gs > 0.0 {
                gs = (gs - 1.5 * tick).max(0.0);
                rolled += gs / 1.943_844 * tick;
            }
            assert!(rolled <= spare, "spare {spare}: rolled {rolled:.1} m, {end:?}");
        }
    }

    /// The turn-round at LILB 03 on a pivoting airframe. The recorded one
    /// (1790721973, 2026-09-30) stopped its leg one 24.9 m down and 14.9 m
    /// right of the centreline of a 15 m strip, heading 172, turned LEFT
    /// (the short way, outward) into the grass, and leg two ended 85 m down.
    /// Now: the pocket sits on the pavement nearer the numbers, the turn is
    /// committed toward the centreline from the first tick, and leg two
    /// settles in less than half the runway, the teardrop kept for every
    /// airframe that does not pivot.
    #[test]
    fn a_pivoting_turn_round_turns_across_the_strip_near_the_numbers() {
        use crate::runway::{along_track_nm, cross_track_nm};
        let rwy = lilb_03();
        let pivot = super::TurnRound { pivots: true, width_m: Some(15.0) };
        let (start, pocket_m, off) = pivot.pocket();
        assert!(start < super::BACKTRACK_TURN_START_M && pocket_m < 30.0, "{start} {pocket_m}");
        assert!(off <= 15.0 / 2.0 - 2.0 + 1e-9 && off >= 4.0, "on the pavement: {off}");
        assert_eq!(super::TurnRound::TEARDROP.pocket(), (60.0, 30.0, 12.0));
        // Leg one from LILB's 21 hold (node 1) ends at that pocket.
        let path = super::backtrack_pocket_path(&rwy, 45.772005, 9.163885, pivot).expect("leg one");
        let end = *path.last().unwrap();
        let along = -along_track_nm(end.0, end.1, &rwy) * 1852.0;
        let ct = cross_track_nm(end.0, end.1, &rwy) * 1852.0;
        assert!((along - pocket_m).abs() < 0.5 && (ct - off).abs() < 0.5, "{along:.1} {ct:.1}");
        // Where the recorded leg one stopped: turned RIGHT, toward the
        // centreline, from the first tick, whatever the error's sign.
        let (plat, plon) = (45.767_307_48, 9.160_850_79);
        assert_eq!(super::turn_round_hint(&rwy, plat, plon), Some(1.0));
        let mut dir = super::turn_round_hint(&rwy, plat, plon);
        // The join logged -174.4 to its aim (the nose 148 off the course).
        assert!(super::committed_err(-174.4, &mut dir) > 170.0);
        assert!(super::committed_err(-120.0, &mut dir) > 0.0, "held through the turn");
        let mut dir = super::turn_round_hint(&rwy, plat, plon);
        let e0 = super::LineUpAlign::new(rwy.course_deg).err_deg(172.1);
        assert!(super::committed_err(e0, &mut dir) > 140.0, "{e0:.0}, right from the first tick");
        // Mirror side: left.
        let (llat, llon) = crate::autoland::project_latlon(plat, plon, rwy.course_deg - 90.0, 29.8 / 1852.0);
        assert_eq!(super::turn_round_hint(&rwy, llat, llon), Some(-1.0));
        // Leg two from the recorded stop: 85 m of runway as a teardrop,
        // 67 pivoting; from the new pocket, under 50.
        let stop_of = |p: Vec<(f64, f64)>| {
            let e = *p.last().unwrap();
            -along_track_nm(e.0, e.1, &rwy) * 1852.0
        };
        let old = stop_of(super::backtrack_leg_two_path(&rwy, plat, plon, super::TurnRound::TEARDROP).unwrap());
        let new = stop_of(super::backtrack_leg_two_path(&rwy, plat, plon, pivot).unwrap());
        assert!((old - 84.9).abs() < 1.0 && new < 68.0, "old {old:.1} new {new:.1}");
        let (pl, po) = (end.0, end.1);
        let from_pocket = stop_of(super::backtrack_leg_two_path(&rwy, pl, po, pivot).unwrap());
        assert!(from_pocket < 50.0, "{from_pocket:.1}");
        // The route is still a drivable one.
        let watch = TaxiWatch {
            path: super::backtrack_leg_two_path(&rwy, pl, po, pivot).unwrap(),
            via: String::new(),
            seg: Vec::new(),
            holding_point: String::new(),
            lead_on: Vec::new(),
        };
        assert!(Route::build(&watch).is_some());
    }

    /// LILB 03 Evolution (1790732094, 2026-09-30), the backtrack's turn-round:
    /// leg one stopped 15.5 m down and 6.51 m right of a 15 m strip (the
    /// pocket 5.5 off), heading 192.4, so the outer (left) main wheel stood
    /// about 8.1 m out against the 7.5 m edge. Leg two's first 8 s at 0.1 to
    /// 0.4 kt: steer +0.35 to +0.44, both brakes 0, the nose LEFT to -6.8
    /// deg/s, 33 degrees the wrong way. The flight before (1790724857)
    /// turned from 5.1 m off and worked.
    #[rustfmt::skip]
    const LILB_TURN_ROUND: &[(f64, f64, f64, f64, f64)] = &[
        // (s, steer, err, gs kt, yaw deg/s) as the drive logged them.
        (0.2, 0.160, 165.15, 0.14, 0.00), (0.4, 0.350, 165.06, 0.04, 0.19), (0.8, 0.378, 165.08, 0.13, 0.09),
        (1.0, 0.391, 165.18, 0.19, -0.15), (1.2, 0.386, 165.46, 0.16, -0.56), (1.5, 0.378, 165.88, 0.13, -1.05),
        (1.8, 0.381, 166.50, 0.14, -1.66), (2.1, 0.388, 167.31, 0.18, -2.26), (2.4, 0.397, 168.28, 0.22, -2.80),
        (2.7, 0.403, 169.32, 0.24, -3.24), (2.9, 0.407, 170.48, 0.26, -3.72), (3.2, 0.411, 171.69, 0.28, -4.11),
        (3.5, 0.415, 173.03, 0.30, -4.46), (3.8, 0.419, 174.45, 0.32, -4.85), (4.1, 0.422, 175.96, 0.33, -5.13),
        (4.4, 0.425, 177.59, 0.35, -5.36), (4.7, 0.428, 179.26, 0.36, -5.69), (4.9, 0.430, 179.26, 0.37, -5.93),
        (5.2, 0.433, 177.57, 0.38, -6.17), (5.5, 0.435, 175.84, 0.39, -6.36), (5.8, 0.437, 173.80, 0.40, -6.53),
        (6.1, 0.439, 171.99, 0.41, -6.69), (6.4, 0.437, 170.10, 0.40, -6.79),
        (13.9, 0.585, 162.42, 1.09, -1.88), (14.2, 0.607, 161.86, 1.18, -2.31), (14.5, 0.594, 161.59, 1.13, -1.98),
        (14.7, 0.599, 161.70, 1.15, -1.09), (15.0, 0.607, 162.11, 1.19, -0.17), (15.3, 0.606, 162.71, 1.18, 0.83),
        (15.5, 0.608, 163.49, 1.19, 1.87), (15.7, 0.611, 164.49, 1.21, 2.88), (16.0, 0.616, 165.91, 1.23, 3.89),
        (16.3, 0.558, 167.86, 0.96, 5.54), (16.6, 0.636, 170.27, 1.32, 6.88),
    ];

    /// The turn-round pivots as designed on a free nose: a pivot going the
    /// wrong way under a knot is braked round on the steer-side (inside)
    /// wheel at once. As flown the law gave nothing for the first 13.6 s
    /// (the taxi split and the pivot's split had no authority under a knot).
    /// The one ground law (`ground_steer`) asks the yaw rate the heading
    /// wants of the plant at any rolling speed, so the logged ticks (speed,
    /// error, the yaw read) put the right brake on from the first tick the
    /// aircraft rolls, and never the outer wheel while the nose swings the
    /// wrong way.
    #[test]
    fn a_turn_round_that_swings_the_wrong_way_under_a_knot_is_braked_round() {
        use crate::ground_steer::{Demand, GroundPlant, GroundSteer, Rudder, Yaw};
        let mut law = GroundSteer::new(GroundPlant::EVOLUTION);
        let mut dir = Some(1.0);
        let mut prev_t = LILB_TURN_ROUND[0].0 - 0.3;
        let mut first_right: Option<f64> = None;
        for &(t, _steer, err, gs, yaw) in LILB_TURN_ROUND {
            let dt = if t - prev_t > 1.0 { 0.3 } else { t - prev_t };
            prev_t = t;
            let e = super::committed_err(err, &mut dir);
            let s = law.step(Demand {
                dt,
                v_ms: gs / MS_TO_KTS,
                yaw: Yaw::Now(yaw),
                err_deg: e,
                ff_dps: 0.0,
                rudder: Rudder::Own { slew_per_s: 0.8 },
                u_max: super::PIVOT_BRAKE_MAX,
                r_max_dps: 20.0,
                io_delay_s: crate::ground_steer::IO_DELAY_S,
            });
            let (bl, br) = s.wheels();
            if yaw <= 0.0 {
                assert_eq!(bl, 0.0, "{t}: never the outer wheel while it swings the wrong way");
            }
            if br > 0.05 && first_right.is_none() {
                first_right = Some(t);
            }
            // The old law, as flown: nothing under a knot.
            if gs < 1.0 {
                assert_eq!(super::pivot_brake_split(0.4, gs), (0.0, 0.0));
            }
        }
        let first = first_right.expect("the right brake came on");
        assert!(first <= 0.8, "right brake by {first} s, the yaw at -0.15");
    }

    /// The pocket keeps the turn-round's outer wheel on the pavement. On
    /// LILB's 15 m strip the old 5.5 m pocket plus the 1.0 m leg one stopped
    /// past it (6.51, 1790732094) and half a GA single's track (about 1.6 m)
    /// is 8.1 m, off the 7.5 m edge; the new pocket with the same error is
    /// 6.6 m, and from the good flight's 5.1 m (1790724857) 6.7.
    #[test]
    fn a_turn_round_pocket_keeps_the_outer_wheel_on_the_pavement() {
        use crate::runway::cross_track_nm;
        const HALF_TRACK_M: f64 = 1.6;
        let rwy = lilb_03();
        let pivot = super::TurnRound { pivots: true, width_m: Some(15.0) };
        let (_, _, off) = pivot.pocket();
        assert!((off - 4.0).abs() < 1e-9, "{off}");
        let stood = cross_track_nm(45.767_260_9, 9.160_703_7, &rwy) * 1852.0;
        assert!((stood - 6.51).abs() < 0.1, "{stood:.2}");
        let overshoot = stood - (7.5 - 2.0);
        assert!(stood + HALF_TRACK_M > 7.5, "as flown: in the grass");
        assert!(off + overshoot + HALF_TRACK_M < 7.5 - 0.5, "now: {:.1}", off + overshoot + HALF_TRACK_M);
        // Still off the centreline enough to turn toward it.
        let on = crate::autoland::project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, 25.0 / 1852.0);
        let (plat, plon) = crate::autoland::project_latlon(on.0, on.1, rwy.course_deg + 90.0, off / 1852.0);
        assert_eq!(super::turn_round_hint(&rwy, plat, plon), Some(1.0));
    }

    /// Where the failed turn-round stopped (1790732094, 45.7675144
    /// 9.1606940, 9.7 true on 024, 41 m down and 5.6 m left), "line us up"
    /// settles onto the centreline as a backtrack's leg two, leaving the fit
    /// its 500 m; the hold-short geometry would have left less. From LILB's
    /// 21 hold, off the strip, the plan is unchanged.
    #[test]
    fn a_line_up_from_on_the_strip_settles_onto_the_centreline() {
        use crate::runway::{along_track_nm, cross_track_nm};
        let rwy = lilb_03();
        let pivot = super::TurnRound { pivots: true, width_m: Some(15.0) };
        let (lat, lon, hdg) = (45.767_514_4, 9.160_694_0, 9.7);
        let (path, stop_m) = super::on_strip_settle_path(&rwy, lat, lon, Some(hdg), pivot).expect("on the strip");
        assert!(rwy.length_m - stop_m >= 500.0, "leaves {:.1}", rwy.length_m - stop_m);
        let end = *path.last().unwrap();
        assert!((cross_track_nm(end.0, end.1, &rwy) * 1852.0).abs() < 0.5);
        let (_, old_stop) = line_up_stops_m(&rwy, lat, lon, Some(hdg)).expect("hold-short geometry");
        assert!(rwy.length_m - old_stop < 500.0, "the old retry left {:.1}", rwy.length_m - old_stop);
        let cands = vec![rwy.clone()];
        let plan = pick_line_up_plan(&cands, lat, lon, Some("03"), 500.0, false, Some(hdg), None, pivot);
        match plan {
            Some((_, LineUpPlan::Plain(p))) => assert_eq!(p, path),
            other => panic!("{:?}", other.map(|(r, _)| r.id)),
        }
        // Pointing across it, or beside it, is no settle.
        assert!(super::on_strip_settle_path(&rwy, lat, lon, Some(hdg + 90.0), pivot).is_none());
        let (blat, blon) = crate::autoland::project_latlon(lat, lon, rwy.course_deg - 90.0, 20.0 / 1852.0);
        assert!(super::on_strip_settle_path(&rwy, blat, blon, Some(hdg), pivot).is_none());
        assert!(-along_track_nm(lat, lon, &rwy) * 1852.0 > 40.0);
        // LILB's 21 hold (node 1), heading across: not on the strip.
        assert!(super::on_strip_settle_path(&rwy, 45.772005, 9.163885, Some(120.0), pivot).is_none());
    }

    #[test]
    fn a_line_up_already_straight_is_done_and_one_that_cannot_turn_says_so() {
        use super::{AlignStep, LineUpAlign};
        // A runway start's numbers: 0.4 off, stopped.
        let mut j = LineUpAlign::new(117.7);
        assert!(matches!(j.step(0.0, 117.3, 0.1, 0.0, 0.0), AlignStep::Done { .. }));
        assert_eq!(j.creeps, 0);
        // Brakes that turn nothing: the creep, then out of time, stopped.
        let mut j = LineUpAlign::new(117.7);
        let mut t = 0.0;
        let end = loop {
            match j.step(t, 107.0, 0.0, 0.0, 2.0) {
                AlignStep::Align { creep, br, .. } => assert!(creep && br > 0.0),
                other => break other,
            }
            t += 0.42;
        };
        assert!(matches!(end, AlignStep::GiveUp { why: "time", .. }), "{end:?}");
        // ...or out of runway.
        let mut j = LineUpAlign::new(117.7);
        assert!(matches!(
            j.step(0.0, 107.0, 2.0, 0.0, super::ALIGN_MAX_ROLL_M + 1.0),
            AlignStep::GiveUp { why: "rolled", .. }
        ));
        // Out of runway but inside what the roll straightens: done (LILB 03
        // Evolution 1790720549: 2.2 off, the mission gave up).
        let mut j = LineUpAlign::new(23.96);
        assert!(matches!(
            j.step(0.0, 21.76, 0.3, 0.2, super::ALIGN_MAX_ROLL_M + 1.0),
            AlignStep::Done { .. }
        ));
        assert_eq!(
            super::not_aligned_line(-10.7),
            "On the runway, but 11 degrees off its heading and I couldn't straighten us. \
             Holding position."
        );
    }

    #[test]
    fn a_pivot_lets_go_of_the_brake_as_the_turn_rate_builds() {
        assert_eq!(pivot_rate_scale(-0.7, -10.0), 1.0, "under the ceiling");
        assert!((pivot_rate_scale(-0.7, -20.0) - 0.5).abs() < 1e-9);
        assert_eq!(pivot_rate_scale(-0.7, -35.0), 0.0, "the LILB join spin");
        assert_eq!(pivot_rate_scale(-0.7, 30.0), 1.0, "turning the wrong way keeps it all");
        assert!((smooth_toward(0.0, 1.0, 0.8, 0.2) - 0.2).abs() < 1e-9);
    }

    #[test]
    fn the_trim_holds_through_joins_and_bleeds_through_bends() {
        let mut t = StraightTrim::default();
        for _ in 0..100 {
            t.step(-4.0, 0.0, 12.0, false, 500.0, 0.2);
        }
        let built = t.step(-4.0, 0.0, 12.0, false, 500.0, 0.2);
        assert!(built > 0.1 && built <= TRIM_MAX, "{built}");
        assert_eq!(t.step(-4.0, 0.0, 12.0, true, 500.0, 0.2), built, "a join is not a bias");
        assert_eq!(t.step(-20.0, 0.0, 12.0, false, 500.0, 0.2), built, "an excursion is not a bias");
        assert_eq!(t.step(-4.0, 0.0, 2.0, false, 500.0, 0.2), built, "a crawl is not a bias");
        let mut bled = built;
        for _ in 0..30 {
            bled = t.step(-4.0, 3.0, 8.0, false, 500.0, 0.2);
        }
        assert!(bled < 0.2 * built, "a bend bleeds it: {bled}");
    }

    #[test]
    fn a_reversal_is_turned_one_way() {
        // LOWI 26's backtrack turn: +177, then -179 on noise.
        let mut dir = None;
        assert_eq!(super::committed_err(177.0, &mut dir), 177.0);
        assert_eq!(super::committed_err(-179.0, &mut dir), 179.0, "kept turning right");
        assert_eq!(super::committed_err(-120.0, &mut dir), 120.0, "still the same way");
        assert_eq!(super::committed_err(-40.0, &mut dir), -40.0, "released once nearly round");
        assert_eq!(dir, None);
    }

    #[test]
    fn a_pivot_gets_brake_at_walking_pace() {
        // Full lock at 4.7 kt: the taxi split gave 0.06, a pivot 0.45.
        let (l, _) = super::brake_steer_split(-1.0, 4.7);
        let (pl, pr) = super::pivot_brake_split(-1.0, 4.7);
        assert!(l < 0.1, "{l}");
        assert!((pl - 0.45).abs() < 1e-9 && pr == 0.0, "{pl}");
        assert_eq!(super::pivot_brake_split(-1.0, 0.5), (0.0, 0.0), "not while stopped");
    }

    #[test]
    fn a_big_error_at_walking_pace_pivots_off_a_join_too() {
        // LSZS 03 lead-on: err 63 at 3.9 kt, steer 0.99, no brake.
        assert!(super::pivot_due(63.0, 3.9, false));
        assert!(super::brake_steer_split(0.99, 3.9) == (0.0, 0.0), "the starved split");
        assert!(!super::pivot_due(63.0, 9.0, false), "a taxi corner at pace keeps the taxi split");
        assert!(!super::pivot_due(30.0, 3.0, false));
        assert!(super::pivot_due(-70.0, 9.0, true), "a join as before");
    }

    /// The recorded corner (sortie 1790605551, t +434.1 to +448.0 s from the
    /// drive's engage): (s, gs kt, right minus left brake, logged yaw_dps).
    const LEMG_CORNER: &[(f64, f64, f64, f64)] = &[
        (434.1, 6.64, 0.042, -4.38), (434.3, 6.54, 0.044, -3.74), (434.5, 6.43, 0.045, -3.00), (434.7, 6.33, 0.044, -2.15),
        (434.9, 6.20, 0.044, -1.50), (435.1, 6.09, 0.045, -0.83), (435.3, 6.01, 0.044, -0.17), (435.5, 5.85, 0.047, 0.45),
        (435.7, 5.74, 0.051, 1.04), (435.9, 5.62, 0.056, 1.76), (436.1, 5.50, 0.062, 2.06), (436.3, 5.36, 0.068, 2.73),
        (436.5, 5.20, 0.075, 3.41), (436.6, 5.03, 0.082, 4.16), (436.8, 4.85, 0.092, 5.34), (437.0, 4.66, 0.103, 6.06),
        (437.2, 4.38, 0.111, 6.94), (437.4, 4.15, 0.119, 8.10), (437.6, 3.90, 0.120, 9.32), (437.9, 3.61, 0.106, 10.67),
        (438.1, 3.41, 0.088, 12.02), (438.3, 3.26, 0.064, 13.18), (438.5, 3.15, 0.039, 13.94), (438.7, 3.12, 0.021, 14.36),
        (438.9, 3.13, 0.007, 14.34), (439.1, 3.19, 0.000, 13.99), (439.3, 3.26, 0.000, 13.38), (439.5, 3.35, 0.000, 12.67),
        (439.7, 3.47, 0.000, 11.83), (439.9, 3.58, 0.000, 10.90), (440.1, 3.68, 0.000, 9.95), (440.4, 3.87, 0.000, 8.99),
        (440.6, 4.00, 0.000, 7.60), (440.8, 4.15, 0.000, 6.90), (441.0, 4.33, 0.000, 6.45), (441.2, 4.50, 0.000, 5.09),
        (441.4, 4.65, 0.017, 4.29), (441.6, 4.76, 0.067, 3.47), (441.8, 4.88, 0.099, 2.96), (442.0, 4.87, 0.130, 3.06),
        (442.2, 4.78, 0.136, 3.80), (442.4, 4.69, 0.128, 5.04), (442.6, 4.62, 0.111, 6.30), (442.9, 4.59, 0.080, 7.56),
        (443.1, 4.61, 0.053, 8.55), (443.3, 4.69, 0.027, 9.02), (443.5, 4.87, 0.000, 9.09), (443.7, 5.07, 0.000, 8.57),
        (443.9, 5.28, 0.000, 7.73), (444.1, 5.49, 0.000, 6.69), (444.3, 5.71, 0.000, 5.70), (444.5, 5.93, 0.000, 4.70),
        (444.7, 6.19, 0.000, 3.69), (444.9, 6.40, 0.013, 2.64), (445.1, 6.57, 0.031, 1.78), (445.4, 6.75, 0.052, 1.20),
        (445.6, 6.85, 0.071, 0.87), (445.7, 6.90, 0.097, 0.88), (446.0, 6.89, 0.122, 1.35), (446.2, 6.86, 0.125, 2.48),
        (446.4, 6.73, 0.132, 3.38), (446.6, 6.65, 0.136, 4.03), (446.8, 6.53, 0.135, 5.30), (446.9, 6.41, 0.117, 7.35),
        (447.1, 6.34, 0.084, 8.14), (447.3, 6.30, 0.056, 8.07), (447.5, 6.33, 0.017, 9.29), (447.8, 6.44, 0.000, 8.34),
        (448.0, 6.57, 0.000, 7.78),
    ];

    /// The three iMac corners that failed, each from about 6 s before its
    /// pivot (sortie ticks 3370 on of 1790690995, 1935 on of 1790718797,
    /// 3250 on of 1790725168; 2026-09-29/30): (s, gs kt, err, steer,
    /// yaw_dps, bl, br) as the drive logged them.
    #[rustfmt::skip]
    const IMAC_CORNERS: [&[(f64, f64, f64, f64, f64, f64, f64)]; 3] = [&[
        (0.0, 3.84, 2.4, -0.348, 14.92, 0.049, 0.0), (0.3, 4.12, -0.86, -0.363, 13.44, 0.09, 0.0), (0.6, 4.3, -3.8, -0.359, 11.25, 0.112, 0.0), (0.9, 4.39, -5.4, -0.287, 8.08, 0.11, 0.0),
        (1.2, 4.48, -5.59, -0.146, 4.1, 0.082, 0.0), (1.5, 4.63, -4.83, -0.115, 0.39, 0.061, 0.0), (1.8, 4.85, -3.47, 0.033, -2.43, 0.022, 0.0), (2.0, 5.19, -1.74, 0.141, -4.25, 0.0, 0.0),
        (2.3, 5.6, 0.4, 0.009, -5.28, 0.0, 0.0), (2.6, 6.03, 2.29, 0.054, -5.53, 0.0, 0.0), (2.9, 6.54, 4.62, 0.11, -5.71, 0.0, 0.005), (3.1, 7.01, 4.12, 0.098, -5.64, 0.0, 0.012),
        (3.4, 7.45, 0.22, 0.005, -5.21, 0.0, 0.0), (3.7, 7.84, -3.95, -0.094, -4.62, 0.0, 0.0), (3.9, 8.22, -8.99, -0.214, -4.28, 0.018, 0.0), (4.3, 8.47, -13.62, -0.324, -3.53, 0.064, 0.0),
        (4.5, 8.77, -23.45, -0.533, -5.22, 0.125, 0.0), (4.8, 8.72, -33.43, -0.762, -6.92, 0.208, 0.0), (5.1, 7.4, -42.3, -0.988, -8.73, 0.25, 0.0), (5.4, 5.95, -48.48, -1.0, -9.46, 0.25, 0.0),
        (5.7, 4.95, -63.06, -1.0, -10.92, 0.366, 0.0), (6.0, 3.78, -60.02, -0.764, -15.89, 0.41, 0.0), (6.3, 2.79, -52.47, -0.524, -22.97, 0.29, 0.0), (6.6, 2.09, -42.1, -0.279, -30.09, 0.167, 0.0),
        (6.8, 1.99, -31.25, -0.053, -35.25, 0.054, 0.0), (7.1, 2.03, -20.6, 0.171, -37.33, 0.0, 0.0), (7.4, 2.15, -9.74, 0.404, -37.72, 0.0, 0.006), (7.7, 2.25, 0.58, 0.639, -36.46, 0.0, 0.038),
        (8.0, 2.24, 10.64, 0.835, -34.84, 0.0, 0.067), (8.3, 2.11, 19.92, 0.808, -31.92, 0.0, 0.061), (8.6, 2.0, 26.88, 0.784, -28.82, 0.0, 0.05), (8.9, 1.92, 33.93, 0.765, -25.85, 0.0, 0.042),
        (9.2, 1.87, 40.31, 0.755, -23.09, 0.0, 0.037), (9.4, 1.85, 46.01, 0.752, -20.47, 0.0, 0.035), (9.7, 1.86, 51.2, 0.752, -18.03, 0.0, 0.036), (10.0, 1.87, 55.21, 0.754, -15.78, 0.0, 0.037),
        (10.2, 1.88, 58.18, 0.757, -12.46, 0.0, 0.038), (10.5, 1.91, 62.74, 0.763, -12.15, 0.0, 0.16), (10.8, 1.58, 65.19, 0.693, -9.44, 0.0, 0.262), (11.1, 0.61, 64.97, 0.482, -3.67, 0.0, 0.145),
        (11.5, 0.42, 62.92, 0.44, 1.39, 0.0, 0.008), (11.7, 0.61, 59.94, 0.481, 6.03, 0.0, 0.0),
    ], &[
        (0.0, 7.88, 2.75, 0.082, -0.23, 0.0, 0.009), (0.2, 7.87, 2.66, 0.077, -0.17, 0.0, 0.01), (0.6, 7.85, 2.49, 0.067, -0.06, 0.0, 0.009), (0.8, 7.83, 2.31, 0.06, 0.08, 0.0, 0.007),
        (1.2, 7.81, 2.02, 0.052, 0.21, 0.0, 0.004), (1.4, 7.8, 2.42, 0.062, 0.39, 0.0, 0.005), (1.7, 7.79, 4.46, 0.11, 0.41, 0.0, 0.014), (1.9, 7.76, 5.97, 0.131, 0.45, 0.0, 0.023),
        (2.3, 7.66, 8.25, 0.165, 0.72, 0.0, 0.037), (2.5, 7.57, 9.44, 0.169, 1.24, 0.0, 0.043), (2.8, 7.44, 7.75, 0.098, 1.87, 0.0, 0.035), (3.0, 7.32, 1.91, -0.077, 2.63, 0.0, 0.0),
        (3.3, 7.3, -4.8, -0.256, 3.02, 0.01, 0.0), (3.5, 7.28, -12.59, -0.436, 2.9, 0.076, 0.0), (3.8, 7.06, -21.19, -0.602, 2.08, 0.145, 0.0), (4.1, 6.59, -29.98, -0.712, -0.03, 0.209, 0.0),
        (4.4, 5.74, -38.78, -0.922, -3.99, 0.25, 0.0), (4.7, 4.89, -42.82, -0.999, -9.29, 0.25, 0.0), (5.0, 4.0, -43.01, -0.999, -15.5, 0.237, 0.0), (5.2, 3.23, -39.98, -0.787, -21.4, 0.132, 0.0),
        (5.5, 2.26, -33.75, -0.579, -27.59, 0.027, 0.0), (5.8, 2.19, -25.34, -0.369, -32.42, 0.0, 0.0), (6.0, 2.25, -16.4, -0.148, -34.08, 0.0, 0.0), (6.3, 2.33, -6.95, 0.082, -34.2, 0.0, 0.0),
        (6.6, 2.41, 2.93, 0.315, -33.67, 0.0, 0.0), (6.9, 2.49, -5.63, 0.549, -32.24, 0.0, 0.034), (7.2, 2.48, 2.08, 0.761, -30.94, 0.0, 0.071), (7.4, 2.35, 8.96, 0.859, -28.41, 0.0, 0.085),
        (7.7, 2.16, 14.97, 0.818, -25.29, 0.0, 0.066), (8.0, 2.02, 20.2, 0.788, -21.9, 0.0, 0.052), (8.3, 1.94, 24.8, 0.771, -18.74, 0.0, 0.044), (8.6, 1.91, 29.02, 0.764, -15.85, 0.0, 0.041),
        (8.8, 1.91, 32.52, 0.764, -13.31, 0.0, 0.041), (9.1, 1.93, 35.5, 0.768, -11.07, 0.0, 0.043), (9.4, 1.96, 37.77, 0.774, -8.97, 0.0, 0.046), (9.7, 1.99, 39.56, 0.782, -6.98, 0.0, 0.049),
        (9.9, 2.04, 40.89, 0.792, -5.08, 0.0, 0.054), (10.2, 2.09, 41.74, 0.802, -3.19, 0.0, 0.059), (10.5, 2.14, 42.04, 0.814, -1.25, 0.0, 0.064), (10.8, 2.19, 41.76, 0.791, 0.74, 0.0, 0.069),
    ], &[
        (0.0, 3.86, 3.17, -0.304, 13.77, 0.039, 0.0), (0.3, 4.19, 0.35, -0.33, 12.67, 0.075, 0.0), (0.7, 4.48, -2.5, -0.325, 10.35, 0.098, 0.0), (0.9, 4.61, -3.55, -0.24, 7.33, 0.094, 0.0),
        (1.2, 4.78, -3.75, -0.132, 3.84, 0.068, 0.0), (1.5, 4.98, -3.11, -0.074, 0.52, 0.047, 0.0), (1.8, 5.35, -1.46, -0.035, -2.07, 0.022, 0.0), (2.0, 5.7, 0.31, 0.008, -3.97, 0.004, 0.0),
        (2.4, 6.28, 2.84, 0.068, -4.87, 0.0, 0.0), (2.7, 6.69, 4.71, 0.112, -5.33, 0.0, 0.0), (3.0, 7.26, 2.05, 0.049, -5.27, 0.0, 0.0), (3.2, 7.64, -1.31, -0.031, -5.06, 0.0, 0.0),
        (3.6, 8.16, -7.26, -0.173, -4.83, 0.008, 0.0), (3.8, 8.49, -12.51, -0.298, -4.68, 0.047, 0.0), (4.2, 7.61, -21.08, -0.502, -4.79, 0.121, 0.0), (4.4, 6.78, -26.53, -0.632, -4.4, 0.177, 0.0),
        (4.8, 5.17, -35.8, -0.852, -5.17, 0.25, 0.0), (5.0, 4.57, -39.4, -0.938, -7.3, 0.25, 0.0), (5.4, 3.64, -40.17, -0.956, -12.19, 0.214, 0.0), (5.6, 3.06, -37.85, -0.763, -18.01, 0.117, 0.0),
        (5.9, 2.0, -30.31, -0.495, -24.5, 0.002, 0.0), (6.2, 2.04, -22.99, -0.295, -29.08, 0.0, 0.0), (6.5, 2.17, -30.46, -0.042, -30.15, 0.0, 0.0), (6.7, 2.26, -23.22, 0.157, -30.79, 0.0, 0.0),
        (7.1, 2.37, -14.86, 0.406, -29.7, 0.0, 0.009), (7.3, 2.43, -8.3, 0.604, -28.8, 0.0, 0.041), (7.7, 2.38, 0.3, 0.866, -27.13, 0.0, 0.082), (7.9, 2.23, 5.79, 0.833, -24.54, 0.0, 0.073),
        (8.2, 2.05, 11.4, 0.793, -21.0, 0.0, 0.055), (8.5, 1.95, 15.31, 0.773, -17.86, 0.0, 0.045), (8.8, 1.88, 19.08, 0.758, -15.08, 0.0, 0.038), (9.0, 1.87, 21.29, 0.755, -12.54, 0.0, 0.037),
        (9.3, 1.87, 24.07, 0.755, -10.61, 0.0, 0.037), (9.6, 1.89, 26.37, 0.759, -8.74, 0.0, 0.039), (9.9, 1.91, 28.13, 0.764, -6.93, 0.0, 0.041), (10.2, 1.94, 29.41, 0.729, -5.14, 0.0, 0.044),
        (10.5, 1.99, 30.24, 0.689, -3.41, 0.0, 0.049), (10.7, 2.01, 30.51, 0.64, -1.83, 0.0, 0.051), (11.0, 2.05, 30.52, 0.592, -0.39, 0.0, 0.055), (11.3, 2.1, 30.03, 0.534, 1.09, 0.0, 0.06),
    ]];

    /// LEMG, the SR22 on d28e8f4 (iMac 2026-09-30, sortie 1790732424, ticks
    /// 1080 on, 3,683 m to go): a right corner entered at 6.6 kt, the swing
    /// out of it at 1.8 kt, then the error 25 to 61 degrees LEFT at 1.2 to
    /// 2 kt. (s, gs kt, err, steer, yaw_dps, bl, br) as logged.
    #[rustfmt::skip]
    const LEMG_SLOW_CORNER: &[(f64, f64, f64, f64, f64, f64, f64)] = &[
        (0.0, 6.63, -3.95, -0.014, -2.32, 0.0, 0.0), (0.3, 6.59, -3.25, -0.036, -2.45, 0.0, 0.0), (0.5, 6.55, -2.33, 0.068, -2.49, 0.0, 0.0), (0.8, 6.5, -1.34, 0.089, -2.46, 0.0, 0.0),
        (1.1, 6.46, -0.4, 0.111, -2.44, 0.0, 0.008), (1.4, 6.39, 1.18, 0.143, -2.33, 0.0, 0.022), (1.6, 6.28, 1.89, 0.145, -2.04, 0.0, 0.031), (1.9, 6.16, 2.37, 0.129, -1.47, 0.0, 0.033),
        (2.1, 5.99, 7.47, 0.223, -0.89, 0.0, 0.051), (2.4, 5.83, 11.31, 0.277, -0.12, 0.0, 0.069), (2.6, 5.61, 15.25, 0.365, 0.87, 0.0, 0.096), (2.9, 5.24, 19.79, 0.473, 2.31, 0.0, 0.136),
        (3.1, 4.83, 22.96, 0.549, 4.41, 0.0, 0.169), (3.4, 4.1, 25.61, 0.611, 7.43, 0.0, 0.208), (3.6, 3.52, 25.79, 0.616, 11.14, 0.0, 0.182), (3.9, 2.89, 24.12, 0.408, 15.03, 0.0, 0.111),
        (4.1, 1.93, 20.64, 0.233, 19.78, 0.0, 0.014), (4.3, 1.81, 15.49, 0.056, 24.17, 0.0, 0.001), (4.7, 1.82, 7.73, -0.211, 25.74, 0.097, 0.0), (4.9, 1.6, 2.31, -0.406, 25.4, 0.069, 0.0),
        (5.2, 1.31, -4.51, -0.633, 23.55, 0.032, 0.0), (5.5, 1.21, -8.9, -0.61, 20.94, 0.018, 0.0), (5.8, 1.16, -14.3, -0.602, 19.13, 0.012, 0.0), (6.0, 1.16, -17.93, -0.601, 17.44, 0.01, 0.0),
        (6.3, 1.18, -22.08, -0.605, 16.01, 0.01, 0.0), (6.5, 1.2, -25.41, -0.611, 14.8, 0.009, 0.0), (6.7, 1.24, -28.7, -0.618, 13.7, 0.009, 0.0), (7.1, 1.31, -33.28, -0.633, 12.68, 0.01, 0.0),
        (7.3, 1.36, -36.29, -0.645, 11.58, 0.009, 0.0), (7.6, 1.47, -40.47, -0.667, 10.59, 0.008, 0.0), (7.9, 1.54, -43.15, -0.684, 9.59, 0.006, 0.0), (8.1, 1.66, -46.33, -0.709, 8.67, 0.016, 0.0),
        (8.4, 1.75, -49.46, -0.73, 7.79, 0.025, 0.0), (8.7, 1.83, -52.5, -0.746, 6.69, 0.033, 0.0), (8.9, 1.89, -55.46, -0.76, 5.48, 0.039, 0.0), (9.2, 1.95, -58.18, -0.772, 4.03, 0.045, 0.0),
        (9.5, 1.99, -60.48, -0.781, 2.39, 0.159, 0.0), (9.8, 1.35, -61.4, -0.643, -0.02, 0.159, 0.0), (10.0, 0.81, -34.83, -0.433, -6.72, 0.0, 0.0), (10.3, 0.94, -31.06, -0.222, -10.6, 0.0, 0.0),
        (10.6, 1.17, -52.05, -0.391, -12.39, 0.0, 0.0), (10.8, 1.42, -48.85, -0.357, -13.03, 0.0, 0.0), (11.1, 1.7, -45.68, -0.334, -13.05, 0.012, 0.0),
    ];

    /// The same vertex on b6c24d5 (1790736973, 2026-09-30): entered at 6.2
    /// kt, the swing to 24 deg/s at 1.9 kt, the nose 25 degrees past the new
    /// taxiway and the walk back. (s, gs kt, err, steer, yaw_dps, bl, br) as
    /// logged.
    #[rustfmt::skip]
    const LEMG_12_VERTEX: &[(f64, f64, f64, f64, f64, f64, f64)] = &[
        (0.0, 6.22, 1.80, 0.119, -1.51, 0.000, 0.021), (0.3, 6.08, 6.46, 0.226, -1.45, 0.000, 0.046), (0.6, 5.92, 11.03, 0.308, -0.89, 0.000, 0.074), (0.8, 5.66, 15.68, 0.376, 0.11, 0.000, 0.102),
        (1.1, 5.24, 20.82, 0.499, 1.71, 0.000, 0.146), (1.4, 4.77, 24.25, 0.580, 3.93, 0.000, 0.184), (1.8, 4.16, 26.32, 0.629, 5.36, 0.000, 0.223), (2.0, 3.20, 26.15, 0.532, 13.63, 0.000, 0.150),
        (2.3, 2.11, 22.55, 0.273, 17.18, 0.000, 0.032), (2.6, 1.88, 17.02, 0.073, 22.67, 0.000, 0.004), (3.0, 1.86, 9.03, -0.223, 24.24, 0.108, 0.000), (3.2, 1.60, 3.56, -0.427, 24.40, 0.109, 0.000),
        (3.5, 1.20, -2.09, -0.545, 22.14, 0.066, 0.000), (3.8, 0.97, 8.48, -0.330, 18.78, 0.034, 0.000), (4.1, 0.84, 4.32, -0.298, 16.23, 0.019, 0.000), (4.3, 0.80, 1.79, -0.280, 14.32, 0.013, 0.000),
        (4.7, 0.76, -2.28, -0.293, 12.91, 0.008, 0.000), (4.9, 0.77, -5.00, -0.306, 12.04, 0.007, 0.000), (5.2, 0.83, -8.56, -0.333, 11.21, 0.007, 0.000), (5.5, 0.90, -10.96, -0.341, 10.16, 0.006, 0.000),
        (5.8, 0.99, -29.37, -0.547, 9.31, 0.059, 0.000), (6.0, 0.96, -15.35, -0.371, 8.11, 0.000, 0.000), (6.4, 1.10, -33.93, -0.562, 7.12, 0.072, 0.000), (6.7, 1.03, -35.90, -0.546, 5.82, 0.064, 0.000),
        (7.0, 0.96, -20.16, -0.297, 3.73, 0.000, 0.000), (7.3, 1.08, -37.47, -0.495, 1.80, 0.070, 0.000), (7.6, 1.06, -37.59, -0.519, 0.21, 0.068, 0.000), (7.8, 1.04, -37.16, -0.462, -1.72, 0.058, 0.000),
        (8.1, 1.05, -36.39, -0.425, -2.74, 0.054, 0.000), (8.4, 1.09, -34.71, -0.339, -5.24, 0.046, 0.000), (8.7, 1.17, -32.89, -0.293, -6.23, 0.047, 0.000), (9.0, 1.27, -30.25, -0.227, -7.52, 0.046, 0.000),
        (9.3, 1.39, -27.11, -0.139, -9.08, 0.032, 0.000), (9.6, 1.58, -23.51, -0.053, -10.15, 0.001, 0.000), (9.9, 1.87, -19.66, 0.029, -10.62, 0.000, 0.017), (10.2, 2.15, -15.70, 0.107, -10.61, 0.000, 0.017),
        (10.5, 2.41, 21.39, 0.336, -10.14, 0.000, 0.015), (10.7, 2.64, 24.17, 0.540, -9.19, 0.000, 0.051), (11.0, 2.80, 27.08, 0.763, -8.20, 0.000, 0.120),
    ];

    /// The recorded brakes and rudder of every corner above through the
    /// identified plant (`ground_steer::GroundPlant::SR22`), read back
    /// through the drive's own yaw filter (the even EMA of the heading
    /// differenced), come back as the logged yaw: the plant the corner
    /// tests fly is the aircraft that flew them. It replaces the two
    /// corner-by-corner fits (0.45 s delay and 2.17 s lag, 2026-09-28; 400 -
    /// 50 v with a 1.5 + 2 / v s lag, 2026-09-30): one model for every
    /// corner, fitted on all 25,403 SR22 drive ticks, no corner's own.
    #[test]
    fn the_identified_plant_replays_the_recorded_corners() {
        use crate::ground_steer::{sim::YawSim, GroundPlant};
        // (s, gs kt, right minus left, steer, logged yaw) for each record.
        let mut corners: Vec<(&str, Vec<(f64, f64, f64, f64, f64)>)> = vec![(
            "1790605551",
            LEMG_CORNER.iter().map(|&(t, gs, b, y)| (t, gs, b, 0.0, y)).collect(),
        )];
        for (k, c) in IMAC_CORNERS.iter().enumerate() {
            corners.push((["1790690995", "1790718797", "1790725168"][k], c.iter().map(|&(t, gs, _, st, y, bl, br)| (t, gs, br - bl, st, y)).collect()));
        }
        corners.push(("1790732424", LEMG_SLOW_CORNER.iter().map(|&(t, gs, _, st, y, bl, br)| (t, gs, br - bl, st, y)).collect()));
        corners.push(("1790736973", LEMG_12_VERTEX.iter().map(|&(t, gs, _, st, y, bl, br)| (t, gs, br - bl, st, y)).collect()));
        for (id, ticks) in corners {
            // The logged EMA's first value is the rate the aircraft had.
            let mut sim = YawSim::new(GroundPlant::SR22, 0.05, -0.9, 0.0, ticks[0].4);
            let (mut ema, mut se, mut peak, mut peak_logged) = (ticks[0].4, 0.0_f64, 0.0_f64, 0.0_f64);
            for w in ticks.windows(2) {
                let ((t, gs, b, st, _), (t1, _, _, _, logged)) = (w[0], w[1]);
                let mean = sim.run(b, st, gs * 0.514_444, t1 - t);
                ema = 0.5 * ema + 0.5 * mean;
                se += (ema - logged).powi(2);
                peak = peak.max(ema.abs());
                peak_logged = peak_logged.max(logged.abs());
            }
            let rms = (se / (ticks.len() - 1) as f64).sqrt();
            eprintln!("{id}: {rms:.2} deg/s rms, peak {peak:.1} vs {peak_logged:.1}");
            assert!(rms < CORNER_REPLAY_RMS_DPS, "{id}: {rms:.2} deg/s rms");
            assert!(peak > 0.7 * peak_logged && peak < 1.4 * peak_logged, "{id}: peak {peak:.1} vs {peak_logged:.1}");
        }
    }
    /// One plant for every corner costs the worst one: 1790690995's swing
    /// comes back at 6.9 deg/s rms, its peak 51 against 37.7 (2026-09-30);
    /// the others 4.5 and under.
    const CORNER_REPLAY_RMS_DPS: f64 = 7.0;

    /// How a corner is flown by the free nose's one ground law: the tick,
    /// the speed it starts at, the governor as the iMac ran it (knots over
    /// its target, the pull under it: the SR22 entered its corners 1 to 2.7
    /// kt over and crept out of a swing at 0.1 to 0.4 m/s^2; over it the
    /// governor brakes at 0.9 m/s^2), the plant flown (the identified one,
    /// scaled for the neighbours), and where the scorer judges from.
    struct FreeRun {
        dt: f64,
        v0_kts: f64,
        over_kts: f64,
        pull: f64,
        plant: crate::ground_steer::GroundPlant,
        judge_from_m: f64,
    }

    /// The identified brake drag, m/s^2 per unit of one wheel (the SR22's).
    const BRAKE_DRAG_MS2: f64 = 6.0;

    /// One corner through the drive's free-nose path (the route's pursuit
    /// and its own turn fed forward, [`crate::ground_steer::GroundSteer`] on
    /// the sim's `R`, the rudder owned) on the plant; the scorer's verdict on the line from
    /// [`crate::scorecard::LineJudge`] on one scale: off the line, or a
    /// vertex cut past its own limit.
    fn fly_free_corner(path_m: &[(f64, f64)], start: (f64, f64), run: FreeRun) -> f64 {
        use crate::ground_steer::{path_demand, sim::YawSim, Demand, GroundPlant, GroundSteer, Rudder, Yaw};
        use crate::scorecard::{LineJudge, CORNER_TURN_DPS, TAXI_MAX_CUT_M, TAXI_MAX_LAT_M};
        let pace = Pace::from_profile(crate::aircraft::load("cirrus_sr22").expect("sr22").taxi.as_ref());
        let watch = TaxiWatch {
            path: path_m.iter().map(|&(x, y)| ll(x, y)).collect(),
            via: String::new(),
            seg: vec![(String::new(), false); path_m.len() - 1],
            holding_point: String::new(),
            lead_on: Vec::new(),
        };
        let mut route = Route::build(&watch).expect("route");
        route.pace = pace;
        let ref_path = route.ref_path();
        let mut law = GroundSteer::new(GroundPlant::SR22);
        let (mut x, mut y) = start;
        let psi0 = taxi_guide::bearing(path_m[0], path_m[1]);
        let mut sim = YawSim::new(run.plant, 0.05, -0.9, psi0, 0.0);
        let mut v = run.v0_kts / MS_TO_KTS;
        let (mut yaw_ema, mut prev_psi) = (0.0_f64, psi0);
        let mut dir = None;
        let mut judge = LineJudge::default();
        let end_m = route.total_m() - 100.0;
        let max_yaw = pace.max_yaw_dps.unwrap_or(20.0);
        for i in 0..(600.0 / run.dt) as usize {
            let t = i as f64 * run.dt;
            let gs = v * MS_TO_KTS;
            let (lat, lon) = ll(x, y);
            let psi = sim.psi.rem_euclid(360.0);
            let (progress, offset) = route.fix_signed(lat, lon).expect("a fix");
            if progress > end_m {
                break;
            }
            let remaining = route.remaining(progress);
            let look = steer_look_m(
                &pace,
                gs,
                route.route_turn_rate_dps(progress, BEND_PROBE_M, gs),
                remaining,
                true,
            );
            let err = route.heading_err_deg(lat, lon, psi, progress, look);
            let route_turn = route.route_turn_rate_dps(progress, look, gs);
            yaw_ema = 0.5 * yaw_ema + 0.5 * crate::angles::signed_diff_deg(psi, prev_psi) / run.dt;
            prev_psi = psi;
            let (e, ff) = path_demand(&ref_path, progress, route.local_m(lat, lon), psi, v, &law, max_yaw);
            let e = committed_err(e, &mut dir);
            let s = law.step(Demand {
                dt: run.dt,
                v_ms: v,
                yaw: Yaw::Now(sim.r),
                err_deg: e,
                ff_dps: ff,
                rudder: Rudder::Own { slew_per_s: pace.steer_slew_per_s.unwrap_or(0.8) },
                u_max: PIVOT_BRAKE_MAX,
                r_max_dps: law.taxi_rate_limit(v, PIVOT_BRAKE_MAX, max_yaw),
                io_delay_s: crate::ground_steer::IO_DELAY_S,
            });
            let asked = (route_turn.abs() >= CORNER_TURN_DPS).then(|| route_turn.signum());
            let cutting = judge.corner(t, gs, offset, Some(remaining), asked, yaw_ema);
            if progress >= run.judge_from_m {
                judge.judge(offset, cutting);
            }
            let sched = schedule_kts(pace.straight, &route, progress, gs).min(pace.track_cap_kts(offset.abs(), err));
            let over = gs - sched - run.over_kts;
            let gov = if over > 0.5 {
                -0.9
            } else if over < 0.0 {
                run.pull
            } else {
                0.0
            };
            let psi_a = sim.psi;
            sim.run(s.diff, s.rudder, v, run.dt);
            let mid = ((psi_a + sim.psi) / 2.0).to_radians();
            x += v * mid.sin() * run.dt;
            y += v * mid.cos() * run.dt;
            v = (v + (gov - BRAKE_DRAG_MS2 * s.diff.abs()) * run.dt).max(0.0);
        }
        judge.flush();
        judge.max_lat.max(judge.max_cut - (TAXI_MAX_CUT_M - TAXI_MAX_LAT_M))
    }

    /// The plants a corner must be flown on: the identified SR22, its
    /// brake and rudder at 0.7 and 1.3 of the fit.
    fn corner_plants() -> [(f64, crate::ground_steer::GroundPlant); 3] {
        use crate::ground_steer::GroundPlant;
        [(0.7, GroundPlant::SR22.scaled(0.7, 1.0)), (1.0, GroundPlant::SR22), (1.3, GroundPlant::SR22.scaled(1.3, 1.0))]
    }

    /// LEMG, the SR22 from the GA stand to 12 (iMac, 0.3 s ticks; sorties
    /// 1790690995, 1790718797 and 1790725168, 2026-09-29/30): 5.0, 4.7 and
    /// 5.3 m off, each at a left corner of about 100 degrees entered at 7 to
    /// 8.7 kt against a 6 kt target, the nose swung 40 to 65 degrees past the
    /// new taxiway. The corner (90 to 110 degrees, with the jog both had
    /// before it) flown by the one ground law at the iMac's ticks and 0.45
    /// and 0.6, the governor as it ran, on the identified plant and its
    /// neighbours: every one inside the 3 m line.
    #[test]
    fn a_free_nose_flies_the_lemg_corners_on_the_line() {
        use crate::scorecard::TAXI_MAX_LAT_M;
        let mut worst = (0.0_f64, String::new());
        for deg in [90.0_f64, 100.0, 110.0] {
            let (jog, th) = (10.0_f64.to_radians(), (-deg).to_radians());
            let v = (30.0 * jog.sin(), 30.0 * jog.cos());
            let path = [(0.0, -300.0), (0.0, 0.0), v, (v.0 + 250.0 * th.sin(), v.1 + 250.0 * th.cos())];
            for dt in [0.3, 0.42, 0.6] {
                for over in [1.0, 2.7] {
                    for (scale, plant) in corner_plants() {
                        let run = FreeRun { dt, v0_kts: 6.0 + over, over_kts: over, pull: 0.2, plant, judge_from_m: 280.0 };
                        let got = fly_free_corner(&path, (0.0, -150.0), run);
                        if got > worst.0 {
                            worst = (got, format!("{deg} deg, {dt} s, {over} kt over, plant x{scale}"));
                        }
                    }
                }
            }
        }
        eprintln!("the LEMG corners: worst {worst:?}");
        assert!(worst.0 <= TAXI_MAX_LAT_M, "{worst:?}");
    }

    /// The walking corner (1790732424 as [`LEMG_SLOW_CORNER`]): out of a
    /// swing at 1.2 to 2 kt with the error 25 to 61 degrees the other way,
    /// 5.1 m off as flown. Both sides, 90 to 110 degrees, the governor on its
    /// target or a knot over, creeping out at 0.05 to 0.2 m/s^2: inside the
    /// line on every plant and tick.
    #[test]
    fn a_free_nose_turns_a_walking_corner_on_the_line() {
        use crate::scorecard::TAXI_MAX_LAT_M;
        let mut worst = (0.0_f64, String::new());
        for side in [-1.0_f64, 1.0] {
            for deg in [90.0_f64, 110.0] {
                let (jog, th) = ((-10.0 * side).to_radians(), (side * deg).to_radians());
                let v = (30.0 * jog.sin(), 30.0 * jog.cos());
                let path = [(0.0, -300.0), (0.0, 0.0), v, (v.0 + 250.0 * th.sin(), v.1 + 250.0 * th.cos())];
                for dt in [0.3, 0.45, 0.6] {
                    for (over, pull) in [(0.0, 0.05), (1.0, 0.2)] {
                        for (scale, plant) in corner_plants() {
                            let run = FreeRun { dt, v0_kts: 6.0 + over, over_kts: over, pull, plant, judge_from_m: 280.0 };
                            let got = fly_free_corner(&path, (0.0, -150.0), run);
                            if got > worst.0 {
                                worst = (got, format!("{side} {deg} deg, {dt} s, {over} kt over, pull {pull}, x{scale}"));
                            }
                        }
                    }
                }
            }
        }
        eprintln!("walking corners: worst {worst:?}");
        assert!(worst.0 <= TAXI_MAX_LAT_M, "{worst:?}");
    }

    /// The route full auto flew from the GA stand to 12 at LEMG (4,782 m,
    /// `plan_to_runway` over the install's apt.dat, the drive's own
    /// `total_m`), local metres at the vertex: 222 degrees, a 2 m kink the
    /// paint left 15 m short of the vertex, then 86 degrees right onto 312.
    const LEMG_12_VERTEX_PATH: [(f64, f64); 8] =
        [(209.0, 230.4), (7.1, 8.5), (4.2, 5.7), (2.8, 5.0), (3.5, 3.1), (-5.5, -5.4), (-9.2, -2.1), (-195.8, 164.2)];

    /// That vertex (1790732424 and 1790736973, 5.1 and 4.4 m off as flown,
    /// the nose round inside the vertex and 25 to 40 degrees past), with and
    /// without the kink, from 8 and 10 kt, the governor on its cap to 2 kt
    /// over: inside the line on every plant and tick.
    #[test]
    fn a_free_nose_turns_the_lemg_12_vertex_on_the_line() {
        use crate::scorecard::TAXI_MAX_LAT_M;
        let kinked: &[(f64, f64)] = &LEMG_12_VERTEX_PATH;
        let plain: &[(f64, f64)] = &[LEMG_12_VERTEX_PATH[0], LEMG_12_VERTEX_PATH[1], LEMG_12_VERTEX_PATH[5], LEMG_12_VERTEX_PATH[6], LEMG_12_VERTEX_PATH[7]];
        let mut worst = (0.0_f64, String::new());
        for (name, path) in [("kinked", kinked), ("plain", plain)] {
            for v0 in [8.0, 10.0] {
                for dt in [0.3, 0.45, 0.6] {
                    for over in [0.0, 2.0] {
                        for (scale, plant) in corner_plants() {
                            let run = FreeRun { dt, v0_kts: v0, over_kts: over, pull: 0.2, plant, judge_from_m: 280.0 };
                            let got = fly_free_corner(path, (108.1, 119.5), run);
                            if got > worst.0 {
                                worst = (got, format!("{name}, from {v0} kt, {dt} s, {over} kt over, x{scale}"));
                            }
                        }
                    }
                }
            }
        }
        eprintln!("the LEMG 12 vertex: worst {worst:?}");
        assert!(worst.0 <= TAXI_MAX_LAT_M, "{worst:?}");
    }

    /// The corner cap is the SR22's (`taxi.max_yaw_dps`) and only a corner's:
    /// a 100 degree vertex is capped near walking pace, the straight and a
    /// gentle bend keep the GA pace.
    #[test]
    fn the_sr22_takes_a_sharp_corner_slowly_and_nothing_else() {
        let pace = Pace::from_profile(crate::aircraft::load("cirrus_sr22").expect("sr22").taxi.as_ref());
        assert_eq!(pace.max_yaw_dps, Some(20.0));
        assert_eq!((pace.straight, pace.turn, pace.sharp), (STRAIGHT_KTS, TURN_KTS, SHARP_KTS));
        let th = (-100.0_f64).to_radians();
        let path = [(0.0, -300.0), (0.0, 0.0), (250.0 * th.sin(), 250.0 * th.cos())];
        let watch = |p: &[(f64, f64)]| TaxiWatch {
            path: p.iter().map(|&(x, y)| ll(x, y)).collect(),
            via: String::new(),
            seg: vec![(String::new(), false); p.len() - 1],
            holding_point: String::new(),
            lead_on: Vec::new(),
        };
        let mut route = Route::build(&watch(&path)).expect("route");
        let uncapped = route.bend_cap_kts(295.0, 4.0).expect("the corner");
        route.pace = pace;
        let capped = route.bend_cap_kts(295.0, 4.0).expect("the corner");
        assert!((TURN_KTS - 0.01..=TURN_KTS + 0.01).contains(&uncapped), "as flown: {uncapped:.2}");
        assert!((CORNER_MIN_KTS..4.0).contains(&capped), "{capped:.2}");
        // Far down the straight, nothing.
        assert!(route.bend_cap_kts(20.0, 10.0).is_none());
        // A 60 degree bend drawn round a 60 m radius keeps the turn pace.
        let arc: Vec<(f64, f64)> = (0..=12)
            .map(|i| {
                let a = (i as f64 * 5.0).to_radians();
                (-(60.0 - 60.0 * a.cos()), 60.0 * a.sin())
            })
            .collect();
        let mut gentle = vec![(0.0, -300.0)];
        gentle.extend(arc);
        let mut r2 = Route::build(&watch(&gentle)).expect("route");
        r2.pace = pace;
        assert!(r2.bend_cap_kts(300.0, 6.0).is_none_or(|c| c >= TURN_KTS - 0.01), "{:?}", r2.bend_cap_kts(300.0, 6.0));
    }

    /// LEMG, the SR22 from the ramp (sorties 1790605551 and 1790618416,
    /// 2026-09-28): 4.2 to 5.3 m inside the corners, 5.3 m at the worst,
    /// over the scorer's 5 m. The ramp corner: 311 degrees, round through
    /// about 90 at one vertex and 20 more at the next, 6 m on; and a plain
    /// right angle. Flown by the one ground law on the plant and its
    /// neighbours: inside the line.
    #[test]
    fn a_castoring_nose_turns_the_lemg_ramp_corner_on_the_line() {
        use crate::scorecard::TAXI_MAX_LAT_M;
        let th1 = 90.0_f64.to_radians();
        let th2 = 110.0_f64.to_radians();
        let b = (6.0 * th1.sin(), 6.0 * th1.cos());
        let ramp = [(0.0, -250.0), (0.0, 0.0), b, (b.0 + 250.0 * th2.sin(), b.1 + 250.0 * th2.cos())];
        let square = [(0.0, -250.0), (0.0, 0.0), (250.0, 0.0)];
        for (name, path) in [("ramp", &ramp[..]), ("right angle", &square[..])] {
            for dt in [0.2, 0.45, 0.6] {
                for (scale, plant) in corner_plants() {
                    let run = FreeRun { dt, v0_kts: 10.0, over_kts: 0.0, pull: 0.3, plant, judge_from_m: 0.0 };
                    let got = fly_free_corner(path, (0.0, -200.0), run);
                    eprintln!("{name}, {dt} s, x{scale}: {got:.2}");
                    assert!(got <= TAXI_MAX_LAT_M, "{name}, {dt} s, x{scale}: {got:.2}");
                }
            }
        }
    }
    /// LEMG (2026-09-28): full auto gave both SR22 taxis up as
    /// "taxi_stalled" at exactly 600 s, still rolling. The drives' remaining
    /// metres every minute (sorties 1790605551, engaged at 4,605 m of a
    /// 4,661 m route, and 1790618416, 4,736 m of 4,791): never stuck by the
    /// ground they made, and each well inside its own deadline.
    #[test]
    fn a_long_taxi_is_judged_by_the_ground_it_makes() {
        const S1: &[(f64, f64)] = &[
            (0.0, 4605.4), (60.1, 4471.8), (120.1, 4201.5), (180.1, 3922.8), (240.1, 3638.2), (300.1, 3375.7),
            (360.1, 3065.3), (420.1, 2738.3), (480.2, 2537.5), (540.3, 2274.0), (599.9, 1997.7),
        ];
        const S2: &[(f64, f64)] = &[
            (0.0, 4735.9), (60.2, 4600.9), (120.2, 4328.2), (180.2, 4058.3), (240.2, 3767.1), (300.2, 3525.7),
            (360.3, 3231.3), (420.4, 2926.2), (480.4, 2624.2), (540.6, 2404.2), (600.0, 2103.9),
        ];
        for (ticks, total) in [(S1, 4660.6), (S2, 4791.1)] {
            let deadline = route_deadline_s(total, 10.0);
            let mut w = ProgressWatch::start(0.0, deadline);
            let r0 = ticks[0].1;
            for &(t, rem) in ticks {
                w.note(r0 - rem, t);
                assert!(!w.stuck(t), "t {t}: {:.0} s without {PROGRESS_STEP_M} m", w.stalled_s(t));
            }
            // At the pace it was making, the rest of the route fits.
            let (t, rem) = ticks[ticks.len() - 1];
            let finish = t + rem / ((r0 - rem) / t);
            assert!(finish < deadline, "finish {finish:.0} s against {deadline:.0} s");
            assert!(finish > 600.0, "and the flat 600 s never could: {finish:.0} s");
        }
        assert!((route_deadline_s(4660.6, 10.0) - 1932.0).abs() < 1.0);
        // A drive that stops making ground IS stuck, whatever it creeps.
        let mut w = ProgressWatch::start(0.0, 5000.0);
        w.note(2607.7, 599.9);
        for k in 0..200 {
            w.note(2607.7 + 0.05 * k as f64, 600.0 + k as f64);
        }
        assert!(w.stuck(799.0), "{:.0} s", w.stalled_s(799.0));
        // A pause, a tug or a spoken stop restarts the clock.
        w.hold(799.0);
        assert!(!w.stuck(799.0));
        // A join that turns away from the route's start is not progress,
        // and not a stall until it has had the whole window.
        let mut j = ProgressWatch::start(0.0, 5000.0);
        j.note(40.0, 0.0);
        j.note(30.0, 100.0);
        assert!(!j.stuck(100.0) && (j.stalled_s(100.0) - 100.0).abs() < 1e-9);
        j.note(61.0, 110.0);
        assert_eq!(j.stalled_s(110.0), 0.0);
        // The route's deadline, moved on by the time held.
        let mut d = ProgressWatch::start(0.0, 100.0);
        for k in 0..=100 {
            d.note(25.0 * k as f64, k as f64);
        }
        assert!(!d.stuck(100.0));
        d.hold(150.0);
        assert!(!d.stuck(149.0) && !d.stuck(150.0), "50 s held, 50 s more");
        assert!(d.stuck(151.0));
    }

    /// Queued behind traffic, every tick notes its fix THEN holds (LEMG
    /// 2026-10-13): the hold excuses the whole tick, so a wait far longer
    /// than the route's deadline is never a stall, and the deadline counts
    /// again from the moment the queue moves.
    #[test]
    fn a_hold_after_each_note_excuses_the_whole_wait() {
        let mut w = ProgressWatch::start(0.0, 100.0);
        let mut t = 0.0;
        while t < 1000.0 {
            w.note(50.0, t);
            w.hold(t + 0.001);
            t += 0.2;
        }
        assert!(!w.stuck(t), "a 1,000 s queue against a 100 s deadline");
        // Moving again: the 100 s are spent from here, on ground made.
        let t0 = t;
        while t < t0 + 99.0 {
            w.note(50.0 + 25.0 * (t - t0), t);
            t += 0.2;
        }
        assert!(!w.stuck(t));
        assert!(w.stuck(t0 + 101.0));
    }

    /// A route that doubles back (LFRG 12, 2026-10-05: down the strip, a
    /// turn-round, back up it) jumps its projection ahead at the turn and
    /// back: neither is ground, and the drive back up the strip IS.
    #[test]
    fn a_projection_jump_is_neither_progress_nor_a_stall() {
        let mut w = ProgressWatch::start(0.0, 5000.0);
        let mut t = 0.0;
        let mut p = 1340.0;
        // Rolling at 5 m/s, a reading every quarter second.
        while p < 1386.0 {
            w.note(p, t);
            p += 1.25;
            t += 0.25;
        }
        // The turn-round: 300 m ahead for 24 s, then back to the return leg.
        for _ in 0..96 {
            w.note(1688.0, t);
            t += 0.25;
        }
        let mut p = 1414.0;
        // Back up the strip at 2.5 m/s for 130 s: well past the 120 s window.
        for _ in 0..520 {
            w.note(p, t);
            assert!(!w.stuck(t), "t {t}: {:.0} s", w.stalled_s(t));
            p += 0.625;
            t += 0.25;
        }
        // Standing still after a jump is still a stall.
        let mut s = ProgressWatch::start(0.0, 5000.0);
        s.note(100.0, 0.0);
        s.note(400.0, 0.25);
        for k in 0..600 {
            s.note(400.0, 0.5 + 0.25 * k as f64);
        }
        assert!(s.stuck(150.0));
    }

    /// #288's spot on LEMG's Delta, in metres: the 737 stopped short of HW1
    /// at 36.689508452 -4.508558165 facing 297.3 (`join_refused` t
    /// 1791978916.7), the route straight back down Delta on 117.3, and the
    /// pavement as the install draws it there: 23 m to the left of the line,
    /// 29 m to the right (sampled every metre square to the nose).
    fn lemg_288() -> ((f64, f64), f64, Vec<(f64, f64)>, impl Fn(f64, f64, f64, f64) -> Option<bool>) {
        let (lat, lon, psi): (f64, f64, f64) = (36.689508452, -4.508558165, 297.3);
        let m_lon = 111_320.0 * lat.to_radians().cos();
        let back = (psi - 180.0).to_radians();
        let path: Vec<(f64, f64)> = (0..=60)
            .map(|k| {
                let d = 5.0 * k as f64;
                (lat + d * back.cos() / 111_320.0, lon + d * back.sin() / m_lon)
            })
            .collect();
        // Metres right of the line (positive) for a point, and the pavement
        // between `left` and `right` of it.
        let across = move |plat: f64, plon: f64| {
            let (e, n) = ((plon - lon) * m_lon, (plat - lat) * 111_320.0);
            let r = (psi + 90.0).to_radians();
            e * r.sin() + n * r.cos()
        };
        let paved = move |plat: f64, plon: f64, left: f64, right: f64| {
            let x = across(plat, plon);
            Some(x >= -left && x <= right)
        };
        ((lat, lon), psi, path, paved)
    }

    /// LEMG 2026-10-14 (#288): the 15 m join's 180 needed 30 m of Delta to
    /// one side and refused; the pilot turned it round by hand to the right
    /// on 6 to 9 m. The fitting join tries tighter and finds the right turn,
    /// every sample on the pavement.
    #[test]
    fn a_turn_round_on_a_taxiway_wide_enough_is_planned_tight() {
        let ((lat, lon), psi, path, paved) = lemg_288();
        let delta = |a: f64, b: f64| paved(a, b, 23.0, 29.0);
        let w = TaxiWatch {
            path: path.clone(),
            via: String::new(),
            seg: vec![(String::new(), false); path.len() - 1],
            holding_point: "HN1".into(),
            lead_on: Vec::new(),
        };
        let route = Route::build(&w).expect("a route");
        let (progress, _) = route.fix_signed_facing(lat, lon, psi).expect("a fix");
        let err = route.heading_err_deg(lat, lon, psi, progress, 20.0);
        assert!(err.abs() > 175.0, "the route is straight behind: {err}");
        assert!(join_turn_applies(err, false, Some(79_015.8), Some(false), false));
        assert!(
            join_turn(&route.path, lat, lon, psi, join_turn_radius_m(Some(79_015.8)), delta).is_none(),
            "the refusal, replayed: 15 m does not fit"
        );
        let (t, r) = join_turn_fitting(&route.path, lat, lon, psi, Some(79_015.8), delta)
            .expect("a turn round fits on Delta");
        assert_eq!(t.dir, 1.0, "right, the side with room, as the pilot turned: {t:?}");
        assert!((8.0..15.0).contains(&r), "tighter, never under the measured turn: {r}");
        assert!(t.lead.iter().all(|&(a, b)| delta(a, b) == Some(true)), "on the pavement");
        assert!(route.with_lead(&t.lead, t.join_m).is_some(), "drivable");
    }

    /// ...and where the pavement really does not fit (a taxiway 12 m either
    /// side of its line: under the 8 m turn's 16 m), it still refuses.
    #[test]
    fn a_turn_round_with_no_room_still_refuses() {
        let ((lat, lon), psi, path, paved) = lemg_288();
        let narrow = |a: f64, b: f64| paved(a, b, 12.0, 12.0);
        assert!(join_turn_fitting(&path, lat, lon, psi, Some(79_015.8), narrow).is_none());
        // A widebody keeps its one 25 m: Delta has no room for that.
        let delta = |a: f64, b: f64| paved(a, b, 23.0, 29.0);
        assert!(join_turn_fitting(&path, lat, lon, psi, Some(230_000.0), delta).is_none());
    }

    /// The same spot over LEMG's drawn pavement and the route the station
    /// plans there (the reversal, #288), read from the install.
    /// `cargo test -- --ignored live_lemg_288_turn_round --nocapture`
    #[test]
    #[ignore] // needs the real X-Plane install
    fn live_lemg_288_turn_round_fits_on_delta() {
        let apt = &crate::config::xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
        let Ok(Some(net)) = atc::nav::find_taxi_net(apt, "LEMG") else {
            return;
        };
        let runways = atc::nav::find_runways(apt, "LEMG").unwrap();
        let strip = runways.iter().find(|r| r.ident_a == "13" || r.ident_b == "13").expect("13/31");
        let (tlat, tlon) = if strip.ident_a == "13" {
            (strip.lat_a, strip.lon_a)
        } else {
            (strip.lat_b, strip.lon_b)
        };
        let zone = vec![strip.ident_a.clone(), strip.ident_b.clone()];
        let (lat, lon, psi) = (36.689508452, -4.508558165, 297.3);
        let plan = net
            .plan_to_runway_facing(lat, lon, tlat, tlon, 100.0, &zone, Some(strip), Some(psi))
            .expect("a route");
        let w = TaxiWatch {
            path: plan.path.clone(),
            via: String::new(),
            seg: plan.seg.clone(),
            holding_point: plan.holding_point.clone(),
            lead_on: Vec::new(),
        };
        let route = Route::build(&w).expect("a route");
        let pave = |a: f64, b: f64| net.on_pavement(a, b);
        assert!(join_turn(&route.path, lat, lon, psi, 15.0, pave).is_none(), "the refusal, replayed");
        let (t, r) = join_turn_fitting(&route.path, lat, lon, psi, Some(79_015.8), pave)
            .expect("a turn round fits on Delta");
        println!("radius {r} {t:?}");
        assert_eq!(t.dir, 1.0, "right, as the pilot turned");
        assert!(t.lead.iter().all(|&(a, b)| pave(a, b) == Some(true)));
    }
