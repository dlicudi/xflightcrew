    use super::*;
    use crate::autoland::lemg_replay::{
        at, fly_final, lemg, FinalLoop, HeadingFit, Jet, A330_HEADING, LEMG_WIDTH_M,
    };

    /// The A330 profile's lead.
    fn a330_lead() -> f64 {
        crate::aircraft::load("airbus_a330_300")
            .and_then(|p| p.circuit_final_lead_s)
            .expect("the A330 declares its final's lead")
    }
    use crate::autoland::{along_track_nm, cross_track_nm, landing_xtk_limit_m};

    /// The recorded airspeed from the final turn of sortie 1790710769
    /// (`circuit` leg ticks, seconds from `final_turn`).
    const A330_IAS: &[(f64, f64)] = &[
        (0.9, 177.6),
        (4.5, 172.5),
        (9.8, 162.3),
        (14.4, 156.6),
        (19.5, 158.1),
        (25.3, 155.2),
        (30.6, 150.5),
        (35.7, 155.3),
        (40.8, 156.6),
        (46.0, 150.6),
        (50.5, 148.8),
        (55.8, 157.8),
    ];

    fn ias_at(t: f64) -> f64 {
        let p = A330_IAS;
        if t <= p[0].0 {
            return p[0].1;
        }
        for w in p.windows(2) {
            if t <= w[1].0 {
                return w[0].1 + (w[1].1 - w[0].1) * (t - w[0].0) / (w[1].0 - w[0].0);
            }
        }
        p[p.len() - 1].1
    }

    #[derive(Debug)]
    struct Final {
        /// Where it let go: offset, distance to run, track error.
        x_m: f64,
        a_m: f64,
        trk_err: f64,
        /// The furthest it ran through the centreline before then.
        through_m: f64,
    }

    /// The final leg from the `final_turn` event (2,541 m right of 30's
    /// course, 7,360 m to run, heading 207.2, calm: the state records' wind
    /// is nil), at a loop tick of `dt`, the offset led when `led`.
    fn fly(fit: HeadingFit, led: bool, dt: f64) -> (Final, Jet) {
        let rwy = lemg("30");
        assert!((rwy.course_deg - 297.73).abs() < 0.1, "{}", rwy.course_deg);
        let limit = landing_xtk_limit_m(Some(LEMG_WIDTH_M));
        let (lat, lon) = at(&rwy, 7360.3, 2541.2);
        let mut jet = Jet::new(lat, lon, 207.2, ias_at(0.0));
        jet.fit = fit;
        jet.wind_mps = 0.0;
        let mut bug: f64 = 207.2;
        let mut hold = BugHold::default();
        let mut through: f64 = 0.0;
        let mut t = 0.0;
        while t < 200.0 {
            let ias = ias_at(t);
            jet.tas_mps = ias * 0.514_444 * 1.03;
            let a_m = -along_track_nm(jet.lat, jet.lon, &rwy) * M_PER_NM;
            let x_m = cross_track_nm(jet.lat, jet.lon, &rwy) * M_PER_NM;
            if x_m < 0.0 {
                through = through.max(-x_m);
            }
            let trk = jet.track();
            let trk_err = crate::angles::signed_diff_deg(trk, rwy.course_deg);
            if ready_for_autoland(x_m, a_m.abs(), trk_err, ias, Some(limit), None) {
                return (Final { x_m, a_m, trk_err, through_m: through }, jet);
            }
            let lead = if led { a330_lead() } else { 0.0 };
            let pursued = final_led_xtk_m(x_m, Some(trk), rwy.course_deg, ias, lead);
            let want = final_intercept_hdg(rwy.course_deg, pursued);
            bug = bug_toward(bug, want, jet.psi, jet.phi, BUG_SLEW_PER_TICK);
            hold.decide(Leg::Final as u32, bug, t, CIRCUIT_FINAL_DEADBAND);
            bug = hold.set().unwrap_or(bug);
            jet.step(bug.round().rem_euclid(360.0), dt);
            t += dt;
        }
        panic!("never handed over");
    }

    /// The fit is the aircraft: the bare pursuit flies the recorded final,
    /// 60 m and more through and handed over coming back 40 m off at 3 to 5
    /// degrees (recorded: 67.1 m, -43.4 m, 3.56 degrees; the 698132
    /// circuit 66.3, -40.9, 4.15).
    #[test]
    fn the_bare_pursuit_runs_the_a330_through_the_line() {
        for dt in [0.6, 0.7, 0.8] {
            let (f, _) = fly(A330_HEADING, false, dt);
            assert!((50.0..90.0).contains(&f.through_m), "dt {dt}: {f:?}");
            assert!(
                f.x_m < -30.0 && f.trk_err > 2.5,
                "handed over coming back: dt {dt}: {f:?}"
            );
        }
    }

    /// Led by the track, the A330 comes onto the line from its own side and
    /// is handed over converging, never after running through it; autoland's
    /// final then reaches the roundout gate on the line with no balk.
    #[test]
    fn the_led_pursuit_brings_the_a330_onto_the_line() {
        let rwy = lemg("30");
        let limit = landing_xtk_limit_m(Some(LEMG_WIDTH_M));
        for dt in [0.6, 0.7, 0.8] {
            let (f, mut jet) = fly(A330_HEADING, true, dt);
            assert!(f.through_m < 10.0, "dt {dt}: {f:?}");
            assert!(
                f.x_m > 0.0 && f.x_m <= 3.0 * limit && f.trk_err < 0.0,
                "converging from its own side: dt {dt}: {f:?}"
            );
            assert!(f.a_m.abs() > 3700.0, "dt {dt}: {f:?}");
            let flown = fly_final(
                &mut jet,
                &mut FinalLoop::new(Some(LEMG_WIDTH_M), 3.0).held(),
                &rwy,
                0.45,
            );
            assert!(flown.balk.is_none(), "dt {dt}: {flown:?}");
            assert!(
                flown.gate_xtk_m.abs() < 5.0 && flown.worst_low_xtk_m < limit,
                "dt {dt}: {flown:?}"
            );
        }
    }

    /// The lead is the A330's alone: every other profile is silent and flies
    /// the pursuit it has flown (the 737's LEMG 13 circuit replay,
    /// `the_lemg_13_circuit_hands_over_on_the_line`, hands over later and
    /// wanders 17 to 22 m below 500 ft when led).
    #[test]
    fn only_the_a330_leads_its_final() {
        assert!((a330_lead() - 3.5).abs() < 1e-9);
        for id in ["boeing_737_800", "cirrus_sr22", "cirrus_vision_sf50", "cessna_172sp"] {
            let p = crate::aircraft::load(id).expect(id);
            assert_eq!(p.circuit_final_lead_s, None, "{id}");
        }
        // No lead is the bare offset, whatever the track.
        assert_eq!(final_led_xtk_m(43.4, Some(280.0), 297.73, 156.0, 0.0), 43.4);
    }

    /// The handoff as it flew (43 m off, coming back at 3.6 degrees), given
    /// to autoland's final: with the old trim, which learnt the roll-out, it
    /// parks 16 m left and goes around `off_centreline_low` at 99 ft, as the
    /// sortie did (recorded: 99 ft, 15.2 m); with the trim learning only a
    /// settled track it reaches the roundout gate within 5 m at every tick.
    #[test]
    fn the_recorded_handoff_went_around_on_a_learnt_roll_out() {
        let rwy = lemg("30");
        let limit = landing_xtk_limit_m(Some(LEMG_WIDTH_M));
        let final_of = |dt: f64, gate: Option<f64>| {
            let (_, mut jet) = fly(A330_HEADING, false, dt);
            let mut lp = FinalLoop::new(Some(LEMG_WIDTH_M), 3.0).held();
            lp.trim_steady_dps = gate;
            fly_final(&mut jet, &mut lp, &rwy, 0.45)
        };
        let old = final_of(0.7, None);
        let (why, radio, xtk) = old.balk.expect("the old trim balks");
        assert_eq!(why, "off_centreline_low", "{old:?}");
        assert!((90.0..110.0).contains(&radio) && xtk < -limit, "{old:?}");
        for dt in [0.6, 0.7, 0.8] {
            let f = final_of(dt, Some(crate::autoland::TRACK_TRIM_STEADY_DPS));
            assert!(f.balk.is_none(), "dt {dt}: {f:?}");
            assert!(f.gate_xtk_m.abs() < 5.0 && f.worst_low_xtk_m < limit, "dt {dt}: {f:?}");
        }
    }

    /// The lead is the track's: toward the line it pulls the aim in, away
    /// from it it pushes it out, parallel it is nothing; no track or speed,
    /// the bare offset.
    #[test]
    fn the_lead_follows_the_track() {
        let (c, l) = (297.73, 3.5);
        // 43.4 m right, closing at 3.56 degrees at 156 kt: 17.3 m of lead.
        let v = final_led_xtk_m(43.4, Some(c - 3.56), c, 156.0, l);
        assert!((v - (43.4 - 17.3)).abs() < 0.5, "{v}");
        assert!(final_led_xtk_m(43.4, Some(c + 3.56), c, 156.0, l) > 43.4);
        assert!((final_led_xtk_m(43.4, Some(c), c, 156.0, l) - 43.4).abs() < 1e-9);
        assert_eq!(final_led_xtk_m(43.4, None, c, 156.0, l), 43.4);
        assert_eq!(final_led_xtk_m(43.4, Some(c - 3.0), c, f64::NAN, l), 43.4);
        // Across north.
        let n = final_led_xtk_m(-20.0, Some(2.0), 358.0, 150.0, l);
        assert!(n > -20.0, "{n}");
    }
