    use super::*;
    use crate::autoland::lemg_replay::{at, fly_final, lemg, FinalLoop, Flown, Jet, LEMG_WIDTH_M};
    use crate::autoland::landing_xtk_limit_m;
    use crate::runway::{along_track_nm, cross_track_nm};

    /// The recorded airspeed, by leg: from the start of the base turn
    /// (212 kt slowing to 156 through the turn, 167 by its end), and from
    /// the start of the final turn (170, flaps out to 142, then 149).
    fn ias_at(leg: Leg, t: f64) -> f64 {
        let pts: &[(f64, f64)] = match leg {
            Leg::Base => &[(0.0, 212.0), (35.0, 156.0), (52.0, 167.0)],
            _ => &[(0.0, 170.0), (17.0, 142.0), (47.0, 149.0)],
        };
        if t >= pts[pts.len() - 1].0 {
            return pts[pts.len() - 1].1;
        }
        for w in pts.windows(2) {
            if t >= w[0].0 && t <= w[1].0 {
                return w[0].1 + (w[1].1 - w[0].1) * (t - w[0].0) / (w[1].0 - w[0].0);
            }
        }
        pts[0].1
    }

    /// The circuit's handoff: where it let go and how.
    #[derive(Debug)]
    struct Handoff {
        x_m: f64,
        a_m: f64,
        trk_err: f64,
        residual_m: f64,
        capture_m: f64,
    }

    /// Fly the circuit from the recorded base-leg start (4,067 m short of 13,
    /// 6,865 m left, heading 311.8, 212 kt) to its handoff, in a wind from
    /// `wind_from` at `wind_mps`. `on_bank` sizes the turn onto final on the
    /// bank the base turn flew; `on_line` gates the handoff on the runway's
    /// landing limit. Neither: the circuit as it flew (3,250 m capture, 300 m
    /// and 12 degrees).
    fn fly_circuit(on_bank: bool, on_line: bool, wind_from: f64, wind_mps: f64) -> (Handoff, Jet) {
        let (h, jet, _) = fly_circuit_knob(on_bank, on_line, wind_from, wind_mps, false);
        (h, jet)
    }

    /// ...with the knob set as today's loop sets it (`BugHold`) when `held`;
    /// the third answer is how many times the knob moved (a whole degree).
    fn fly_circuit_knob(
        on_bank: bool,
        on_line: bool,
        wind_from: f64,
        wind_mps: f64,
        held: bool,
    ) -> (Handoff, Jet, u32) {
        let rwy = lemg("13");
        let mut geom = Geometry::for_radii(2500.0, 2500.0);
        assert_eq!(
            geom.final_capture_m, 3250.0,
            "the capture the sortie armed with"
        );
        let (lat, lon) = at(&rwy, 4067.2, -6864.6);
        let mut jet = Jet::new(lat, lon, 311.8, ias_at(Leg::Base, 0.0));
        jet.wind_to_deg = (wind_from + 180.0).rem_euclid(360.0);
        jet.wind_mps = wind_mps;
        let mut leg = Leg::Base;
        let mut leg_t = 0.0;
        let mut watch = BankWatch::default();
        let mut bug: f64 = 311.8;
        let mut hold = BugHold::default();
        let (mut moves, mut knob) = (0u32, bug.round());
        let limit = on_line.then(|| landing_xtk_limit_m(Some(LEMG_WIDTH_M)));
        let dt = 0.6;
        let mut t = 0.0;
        while t < 400.0 {
            let ias = ias_at(leg, t - leg_t);
            jet.tas_mps = ias * 0.514_444;
            let a_m = -along_track_nm(jet.lat, jet.lon, &rwy) * M_PER_NM;
            let x_m = cross_track_nm(jet.lat, jet.lon, &rwy) * M_PER_NM;
            let trk = jet.track();
            let trk_err = crate::angles::signed_diff_deg(trk, rwy.course_deg);
            if leg == Leg::Final && ready_for_autoland(x_m, a_m.abs(), trk_err, ias, limit, None) {
                return (
                    Handoff {
                        x_m,
                        a_m,
                        trk_err,
                        residual_m: handoff_residual_m(x_m, a_m, ias),
                        capture_m: geom.final_capture_m,
                    },
                    jet,
                    moves,
                );
            }
            let crab = crate::angles::signed_diff_deg(jet.psi, trk);
            let leg_hdg = match leg {
                Leg::Final => final_intercept_hdg(rwy.course_deg, x_m),
                _ => leg_heading(rwy.course_deg, leg, true),
            };
            if leg == Leg::Base && on_bank {
                watch.note(jet.phi, jet.psi - leg_hdg);
                if let Some(c) = watch
                    .steady_bank_deg()
                    .and_then(|b| final_capture_on_bank(ias, b))
                {
                    geom.final_capture_m = c;
                }
            }
            bug = bug_toward(bug, leg_hdg + crab, jet.psi, jet.phi, BUG_SLEW_PER_TICK);
            if held {
                let db = if leg == Leg::Final {
                    CIRCUIT_FINAL_DEADBAND
                } else {
                    CIRCUIT_LEG_DEADBAND
                };
                hold.decide(leg as u32, bug, t, db);
                bug = hold.set().unwrap_or(bug);
            }
            // The knob is a whole degree (`write_bug`).
            let whole = bug.round().rem_euclid(360.0);
            if whole != knob {
                moves += 1;
                knob = whole;
            }
            jet.step(whole, dt);
            if pattern_advance(leg, a_m, x_m, &geom, rwy.length_m, true) {
                if let Some(n) = next_leg(leg) {
                    leg = n;
                    leg_t = t;
                }
            }
            t += dt;
        }
        panic!("never handed over");
    }

    /// The recorded base turn of sortie 1790591256 (state records, 2 s
    /// apart, heading true against the base leg's 221.7): the 737's
    /// autopilot banks 25 degrees, not the Baron's 15.5, and the capture it
    /// sizes is two thirds of the 3,250 m the sortie armed with.
    #[test]
    fn the_base_turn_measures_the_737s_bank() {
        let base = leg_heading(131.685, Leg::Base, true);
        let rec = [
            (-2.5, 311.3),
            (-13.6, 309.2),
            (-23.2, 305.2),
            (-26.6, 300.6),
            (-26.9, 296.4),
            (-26.5, 291.0),
            (-26.1, 286.9),
            (-25.1, 283.4),
            (-24.9, 278.9),
            (-25.1, 274.0),
            (-24.6, 268.5),
            (-25.4, 265.6),
            (-24.4, 260.6),
            (-25.0, 254.9),
            (-25.3, 249.0),
            (-24.8, 242.7),
            (-25.5, 236.8),
            (-25.1, 231.1),
            (-24.9, 225.4),
            (-19.9, 220.6),
            (-10.6, 218.8),
        ];
        let mut w = BankWatch::default();
        for (phi, hdg) in rec {
            w.note(phi, hdg - base);
        }
        let bank = w.steady_bank_deg().expect("a measured bank");
        assert!((24.5..=25.5).contains(&bank), "{bank:.1}");
        // At the 167 kt the base leg ended at.
        let c = final_capture_on_bank(167.0, bank).unwrap();
        assert!((2100.0..2350.0).contains(&c), "{c:.0}");
        // A Baron's 15.8 at 105 kt keeps what it had (15.5 designed).
        let baron = final_capture_on_bank(105.0, 15.8).unwrap();
        let designed = 1.3 * final_turn_radius_m(105.0);
        assert!(
            baron >= designed && baron < designed * 1.1,
            "{baron:.0} against {designed:.0}"
        );
        // Nothing measured, nothing said; no airspeed, no capture.
        assert_eq!(BankWatch::default().steady_bank_deg(), None);
        assert_eq!(final_capture_on_bank(f64::NAN, 25.0), None);
    }

    /// The circuit as it flew hands autoland a 737 still closing at 12
    /// degrees about 100 m off at 1.5 to 2 nm (recorded: 111 m, 12 degrees,
    /// 1.9 nm), which autoland on a 45 m strip now refuses as a go-around.
    /// Sized on the measured bank and handed over only ON the line, it lets
    /// go inside 45 m and 5 degrees above 2 nm, and autoland's own final
    /// reaches the roundout gate within a few metres, with no balk.
    #[test]
    fn the_lemg_13_circuit_hands_over_on_the_line() {
        let rwy = lemg("13");
        let limit = landing_xtk_limit_m(Some(LEMG_WIDTH_M));
        let (old, mut jet) = fly_circuit(false, false, 106.0, 6.0);
        assert!(
            (80.0..130.0).contains(&old.x_m.abs())
                && old.trk_err.abs() > 10.0
                && old.residual_m > limit,
            "the replay hands over as the sortie did: {old:?}"
        );
        let f: Flown = fly_final(
            &mut jet,
            &mut FinalLoop::new(Some(LEMG_WIDTH_M)),
            &rwy,
            0.35,
        );
        assert_eq!(
            f.balk.map(|b| b.0),
            Some("unstabilised_below_floor"),
            "{f:?}"
        );

        let (new, mut jet) = fly_circuit(true, true, 106.0, 6.0);
        assert!(new.capture_m < 2400.0, "{new:?}");
        assert!(
            new.x_m.abs() <= 3.0 * limit && new.trk_err.abs() <= 5.0,
            "{new:?}"
        );
        assert!(new.residual_m <= limit && new.a_m.abs() > 3700.0, "{new:?}");
        let f = fly_final(
            &mut jet,
            &mut FinalLoop::new(Some(LEMG_WIDTH_M)),
            &rwy,
            0.35,
        );
        assert!(f.balk.is_none(), "{f:?}");
        assert!(
            f.gate_xtk_m.abs() < 5.0 && f.worst_low_xtk_m < limit,
            "{f:?}"
        );
        assert!(f.worst_short_bank_deg < 5.0, "{f:?}");
    }


    /// Issue #93: the knob set as a pilot sets it, per leg with a deadband,
    /// hands over on the line exactly as the every-tick knob did, in every
    /// wind, with the knob moved far fewer times (base and final from the
    /// recorded base-leg start: 12 to 17 moves against 31 to 43).
    #[test]
    fn the_held_knob_hands_over_on_the_line_and_moves_less() {
        let limit = landing_xtk_limit_m(Some(LEMG_WIDTH_M));
        for (from, mps) in [(0.0, 0.0), (106.0, 6.0), (106.0, 9.0), (200.0, 8.0), (40.0, 7.0)] {
            let (free, _, free_moves) = fly_circuit_knob(true, true, from, mps, false);
            let (h, mut jet, moves) = fly_circuit_knob(true, true, from, mps, true);
            assert!(
                h.x_m.abs() <= 3.0 * limit && h.residual_m <= limit && h.trk_err.abs() <= 5.0,
                "wind {from}/{mps}: {h:?}"
            );
            let f = fly_final(
                &mut jet,
                &mut FinalLoop::new(Some(LEMG_WIDTH_M)),
                &lemg("13"),
                0.35,
            );
            assert!(f.balk.is_none(), "wind {from}/{mps}: {f:?}");
            assert!(
                moves * 2 <= free_moves,
                "wind {from}/{mps}: {moves} moves against {free_moves} ({free:?})"
            );
        }
    }

    /// The hold itself: a new leg sets at once, a turn's lead is steering
    /// and goes at once, a small correction waits for its deadband and its
    /// interval, a failed write forgets.
    #[test]
    fn the_bug_is_set_once_per_leg_and_corrected_rarely() {
        let db = CIRCUIT_LEG_DEADBAND;
        let mut h = BugHold::default();
        assert_eq!(h.decide(2, 310.4, 0.0, db), Some(310.4));
        // Crab dither: nothing.
        for (i, w) in [311.2, 309.6, 311.9, 309.1].iter().enumerate() {
            assert_eq!(h.decide(2, *w, 1.0 + i as f64, db), None, "{w}");
        }
        // Two degrees off, but too soon.
        assert_eq!(h.decide(2, 312.6, 5.0, db), None);
        // ...and after the interval, once.
        assert_eq!(h.decide(2, 312.6, 11.0, db), Some(312.6));
        assert_eq!(h.decide(2, 312.9, 12.0, db), None);
        // A turn's lead: five degrees or more is steering, at once.
        assert_eq!(h.decide(2, 318.0, 12.5, db), Some(318.0));
        // Across north, the short way round counts.
        let mut n = BugHold::default();
        n.decide(1, 359.5, 0.0, db);
        assert_eq!(n.decide(1, 0.5, 1.0, db), None);
        // A new leg: at once.
        assert_eq!(h.decide(3, 220.0, 12.6, db), Some(220.0));
        // A failed write forgets: the next tick sets it.
        h.took(None);
        assert_eq!(h.decide(3, 220.0, 12.7, db), Some(220.0));
        assert_eq!(h.decide(3, f64::NAN, 13.0, db), None);
    }
    /// Not tuned to one wind: calm, the recorded one, a stronger one from the
    /// other side of the final.
    #[test]
    fn the_lemg_13_circuit_lands_on_the_line_in_other_winds() {
        let rwy = lemg("13");
        let limit = landing_xtk_limit_m(Some(LEMG_WIDTH_M));
        for (from, mps) in [(0.0, 0.0), (106.0, 9.0), (200.0, 8.0), (40.0, 7.0)] {
            let (h, mut jet) = fly_circuit(true, true, from, mps);
            assert!(
                h.x_m.abs() <= 3.0 * limit && h.residual_m <= limit,
                "wind {from}/{mps}: {h:?}"
            );
            let f = fly_final(
                &mut jet,
                &mut FinalLoop::new(Some(LEMG_WIDTH_M)),
                &rwy,
                0.35,
            );
            assert!(f.balk.is_none(), "wind {from}/{mps}: {f:?}");
            assert!(f.gate_xtk_m.abs() < limit, "wind {from}/{mps}: {f:?}");
        }
    }
