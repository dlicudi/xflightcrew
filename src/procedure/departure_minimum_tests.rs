    use super::*;
    use crate::nav::ClimbMinimum;

    const HUNT: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/follower/egkk_lam5p_minimum_hunt.jsonl"));

    fn records() -> Vec<serde_json::Value> {
        HUNT.lines().filter_map(|l| serde_json::from_str(l).ok()).collect()
    }

    fn mins() -> Vec<ClimbMinimum> {
        vec![
            ClimbMinimum { fix: "ACORN".into(), min_ft: 5000.0 },
            ClimbMinimum { fix: "D156J".into(), min_ft: 6000.0 },
        ]
    }

    /// The LAM5P as flown, on a meridian: (ident, nm from the arm, plan ft).
    const ROUTE: [(&str, f64, Option<f64>); 7] = [
        ("D075D", 0.9, Some(1210.0)),
        ("D258T", 7.5, Some(2500.0)),
        ("ACORN", 13.5, Some(5000.0)),
        ("D156O", 20.5, None),
        ("D156J", 29.0, None),
        ("LAM", 39.0, Some(8000.0)),
        ("HEMEL", 60.0, Some(22000.0)),
    ];

    fn legs() -> Vec<ProcLeg> {
        ROUTE
            .iter()
            .map(|(id, nm, alt)| ProcLeg {
                seq: 0,
                transition: String::new(),
                leg_type: "TF".into(),
                fix_ident: (*id).into(),
                fix_region: String::new(),
                fix: Some(crate::nav::Fix { ident: (*id).into(), lat: 51.0 + nm / 60.0, lon: 0.0 }),
                course_mag: None,
                dist_nm: None,
                turn: None,
                alt_desc: ' ',
                alt1_ft: *alt,
                alt2_ft: None,
                is_faf: false,
                is_map: false,
                vpa_deg: None,
                navaid_ident: String::new(),
                navaid_region: String::new(),
                navaid: None,
                path: Default::default(),
            })
            .collect()
    }

    fn changes(v: &[f64]) -> usize {
        v.windows(2).filter(|w| (w[0] - w[1]).abs() > 0.5).count()
    }

    /// The recorded ticks, replayed through the old stateless test and the
    /// new memory: the old one flips the target and the speed on the
    /// altimeter's last feet at 5,900, exactly as the sortie logged them;
    /// the new one meets D156J once and holds 6,000.
    #[test]
    fn the_recorded_climb_met_d156j_once_and_held_its_floor() {
        let recs = records();
        let ticks: Vec<&serde_json::Value> = recs.iter().filter(|r| r["kind"] == "leg").collect();
        assert!(ticks.len() > 500, "{}", ticks.len());
        // What the sortie logged: the governor's target flipping 210/250.
        let logged: Vec<f64> = recs
            .iter()
            .filter(|r| r["kind"] == "speed_limit")
            .map(|r| r["target"].as_f64().unwrap())
            .collect();
        assert!(changes(&logged) >= 18, "{logged:?}");
        let recorded_want: Vec<f64> = ticks.iter().map(|r| r["want_alt"].as_f64().unwrap()).collect();
        assert!(changes(&recorded_want) >= 18, "the target hunted 5,000/6,000: {}", changes(&recorded_want));

        let (mins, legs) = (mins(), legs());
        let idx_of = |r: &serde_json::Value| {
            let fix = r["fix"].as_str().unwrap();
            legs.iter().position(|l| l.fix_ident == fix).unwrap()
        };
        let released = recs
            .iter()
            .find(|r| r["kind"] == "cleared_ceiling" && r["ft"].is_null())
            .map(|r| r["t"].as_f64().unwrap())
            .unwrap();
        let mut memory = DepartureMinimums::default();
        let (mut old_speed, mut new_speed, mut new_want) = (Vec::new(), Vec::new(), Vec::new());
        for r in &ticks {
            let (idx, alt) = (idx_of(r), r["alt_ft"].as_f64().unwrap());
            let t = r["t"].as_f64().unwrap();
            old_speed.push(if unmet_minimum(&mins, &legs, idx, alt).is_some() { 210.0 } else { 250.0 });
            new_speed.push(if memory.unmet(&mins, &legs, idx, alt).is_some() { 210.0 } else { 250.0 });
            // The plan's own level: the logged target where no minimum
            // raised it (the navlog's 3,682 held and 5,000).
            let logged = r["want_alt"].as_f64().unwrap();
            let plan = if (logged - 6000.0).abs() < 1.0 { 5000.0 } else { logged };
            let ceiling = (t < released).then_some(5000.0);
            let floor = DepartureMinimums::floor(&mins, &legs, idx).map(|m| m.1);
            new_want.push(departure_target_ft(plan, floor, ceiling, false, None).want_ft);
        }
        assert!(changes(&old_speed) >= 18, "the old rule reproduces the log: {}", changes(&old_speed));
        assert_eq!(changes(&new_speed), 1, "Vy released once, at 5,900");
        // After the check-in: one number, 6,000, all the way to D156J (past
        // it the climb's hold on the level commanded keeps it, see the
        // plant test below).
        let after: Vec<f64> = ticks
            .iter()
            .zip(&new_want)
            .filter(|(r, _)| r["t"].as_f64().unwrap() >= released && r["fix"] != "LAM")
            .map(|(_, w)| *w)
            .collect();
        assert!(after.iter().all(|w| (*w - 6000.0).abs() < 0.5), "{after:?}");
    }

    /// The owners in order: a minimum raises the plan, the clearance caps
    /// both until it is released, the pilot and the ground outrank the cap.
    #[test]
    fn the_clearance_caps_a_minimum_until_departure_clears_higher() {
        let t = departure_target_ft(5000.0, Some(6000.0), Some(5000.0), false, None);
        assert_eq!(t.want_ft, 5000.0);
        assert!(t.raised_by_minimum && t.minimum_capped && t.cap_limited);
        let t = departure_target_ft(5000.0, Some(6000.0), None, false, None);
        assert_eq!(t.want_ft, 6000.0, "released: the minimum");
        assert!(t.raised_by_minimum && !t.minimum_capped && !t.cap_limited);
        let t = departure_target_ft(22000.0, Some(6000.0), Some(5000.0), false, None);
        assert_eq!(t.want_ft, 5000.0);
        assert!(!t.raised_by_minimum && t.cap_limited);
        assert_eq!(departure_target_ft(9000.0, Some(6000.0), Some(5000.0), true, None).want_ft, 9000.0, "the pilot's");
        assert_eq!(departure_target_ft(5000.0, Some(6000.0), Some(5000.0), false, Some(5600.0)).want_ft, 5600.0, "the ground's");
        assert_eq!(departure_target_ft(7000.0, Some(6000.0), None, false, None).want_ft, 7000.0, "under the plan: nothing");
    }

    /// A minimum met stays met, keyed by fix (a splice renumbers legs); the
    /// floor holds met minimums until their fix is behind.
    #[test]
    fn a_minimum_met_stays_met_and_floors_the_target_until_its_fix() {
        let (mins, legs) = (mins(), legs());
        let mut m = DepartureMinimums::default();
        assert_eq!(m.unmet(&mins, &legs, 1, 4000.0), Some((2, 5000.0)));
        assert_eq!(m.unmet(&mins, &legs, 1, 4900.0), Some((4, 6000.0)), "ACORN met at 4,900");
        assert_eq!(m.unmet(&mins, &legs, 2, 5900.0), None, "D156J met at 5,900");
        assert_eq!(m.unmet(&mins, &legs, 3, 5750.0), None, "and stays met");
        // Renumbered by a splice ahead: still met.
        let mut spliced = legs.clone();
        spliced.insert(0, legs[0].clone());
        assert_eq!(m.unmet(&mins, &spliced, 4, 5750.0), None);
        assert_eq!(DepartureMinimums::floor(&mins, &legs, 0), Some((4, 6000.0)), "the highest ahead");
        assert_eq!(DepartureMinimums::floor(&mins, &legs, 4), Some((4, 6000.0)), "met, not flown");
        assert_eq!(DepartureMinimums::floor(&mins, &legs, 5), None, "behind us");
    }

    /// The VS wheel only in VS mode; a hold closing on our dial is left.
    #[test]
    fn the_vs_wheel_is_written_only_in_vs_mode() {
        assert_eq!(vs_action(true, false, 5800.0, 6000.0, 6000.0, 600.0, 900.0), VsAction::Write);
        // The recorded capture: ALT HOLD 183 ft under the 6,000 dial at 962 fpm.
        assert_eq!(vs_action(false, true, 5817.0, 6000.0, 6000.0, 549.0, 962.0), VsAction::Leave);
        assert_eq!(vs_action(false, true, 5990.0, 6000.0, 6000.0, 30.0, 0.0), VsAction::Leave);
        // Holding short of the dial and not closing: not a level-off.
        assert_eq!(vs_action(false, true, 5750.0, 6000.0, 6000.0, 750.0, 0.0), VsAction::Engage);
        // A new level far off the held one: VS.
        assert_eq!(vs_action(false, true, 6000.0, 8000.0, 8000.0, 1800.0, 0.0), VsAction::Engage);
        assert_eq!(vs_action(false, false, 5000.0, 8000.0, 8000.0, 1800.0, 0.0), VsAction::Engage);
    }

    /// The 737's vertical modes: VS through a 2 s transport delay and two
    /// 1.5 s lags (#150's fit); an ALT capture that takes the dial inside
    /// 0.19 min of the rate (the recorded 183 ft at 962 fpm) and holds it
    /// through the same delay; the VS wheel reads 0 outside VS mode; a VS
    /// engage syncs the wheel to the rate flown.
    struct Ap {
        alt: f64,
        vs: f64,
        lag1: f64,
        pipe: std::collections::VecDeque<f64>,
        vs_mode: bool,
        vs_dial: f64,
        alt_dial: f64,
        hold_ft: f64,
    }

    impl Ap {
        const DT: f64 = 0.1;
        fn new(alt: f64, vs: f64) -> Self {
            Ap {
                alt,
                vs,
                lag1: vs,
                pipe: std::iter::repeat_n(vs, 20).collect(),
                vs_mode: true,
                vs_dial: vs,
                alt_dial: 5000.0,
                hold_ft: 0.0,
            }
        }
        fn step(&mut self) {
            let cmd = if self.vs_mode {
                self.vs_dial
            } else {
                ((self.hold_ft - self.alt) * 3.0).clamp(-1500.0, 1500.0)
            };
            self.pipe.push_back(cmd);
            let late = self.pipe.pop_front().unwrap();
            self.lag1 += (late - self.lag1) * Self::DT / 1.5;
            self.vs += (self.lag1 - self.vs) * Self::DT / 1.5;
            self.alt += self.vs * Self::DT / 60.0;
            let off = self.alt_dial - self.alt;
            if self.vs_mode && off * self.vs >= 0.0 && off.abs() <= (self.vs.abs() * 0.19).max(20.0) {
                self.vs_mode = false;
                self.hold_ft = self.alt_dial;
            }
        }
        fn engage_vs(&mut self) {
            self.vs_mode = true;
            self.vs_dial = (self.vs / 100.0).round() * 100.0;
        }
        /// Write then read back, as `set_confirmed`.
        fn write_vs(&mut self, v: f64) -> bool {
            if self.vs_mode {
                self.vs_dial = v;
            }
            let read = if self.vs_mode { self.vs_dial } else { 0.0 };
            (read - v).abs() <= 1.0
        }
    }

    #[derive(Default, Debug)]
    struct Flown {
        speed: Vec<f64>,
        /// (alt, vs) every tick after 5,900 ft was first reached, to D156J.
        level: Vec<(f64, f64)>,
        alt_at_d156j: Option<f64>,
        vs_writes_in_alt: u32,
        unconfirmed: u32,
        final_alt: f64,
    }

    /// The follower's vertical over the plant, old chain or new, from the
    /// arm at 3,712 ft climbing 1,500 under the clearance's 5,000 (released
    /// 4 s later by the check-in, as recorded) to 180 s past LAM.
    fn fly(new: bool) -> Flown {
        let (mins, legs) = (mins(), legs());
        let mut ap = Ap::new(3712.0, 1500.0);
        let (gs, best, climb) = (230.0, 2500.0, 1800.0);
        let (mut idx, mut pos) = (0usize, 0.0_f64);
        let (mut carry, mut prev_cmd, mut peak) = (3712.0, 3712.0, 3712.0);
        let mut memory = DepartureMinimums::default();
        let (mut last_dial, mut last_vs): (Option<f64>, Option<f64>) = (None, None);
        let mut out = Flown::default();
        let mut reached = false;
        let mut past_lam_s = 0.0;
        for tick in 0..3000 {
            for _ in 0..5 {
                ap.step();
            }
            pos += gs * 0.5 / 3600.0;
            let alt = ap.alt;
            let fix_nm = |i: usize| ROUTE[i].1;
            while idx + 1 < legs.len() && fix_nm(idx) - pos <= CAPTURE_NM {
                if legs[idx].fix_ident == "D156J" {
                    out.alt_at_d156j = Some(alt);
                }
                idx += 1;
            }
            if idx >= 6 {
                past_lam_s += 0.5;
                if past_lam_s > 180.0 {
                    break;
                }
            }
            let dist = fix_nm(idx) - pos;
            let ceiling = (tick < 8).then_some(5000.0);
            let unmet = if new {
                memory.unmet(&mins, &legs, idx, alt)
            } else {
                unmet_minimum(&mins, &legs, idx, alt)
            };
            out.speed.push(if unmet.is_some() { 210.0 } else { 250.0 });
            let profile = DescentProfile { cruise_ft: Some(22000.0), peak_ft: peak, past_top: false, arrival_ceiling: false };
            let v = vertical_tick(&legs[idx], carry, prev_cmd, alt, Some(dist), gs, None, None, 2000.0, profile);
            peak = v.peak_ft;
            let want = if new {
                let floor = DepartureMinimums::floor(&mins, &legs, idx).map(|m| m.1);
                let dep = departure_target_ft(v.want_alt, floor, ceiling, false, None);
                prev_cmd = if dep.raised_by_minimum { v.prev_cmd.max(dep.want_ft) } else { v.prev_cmd };
                dep.want_ft
            } else {
                let w = cap_at_clearance(v.want_alt, ceiling, false, None);
                prev_cmd = v.prev_cmd;
                match unmet {
                    Some((_, m)) if m > w => m,
                    _ => w,
                }
            };
            carry = v.carry;
            let path_fpm = if (want - v.want_alt).abs() < 1.0 { v.target_fpm } else { 0.0 };
            let dial = dial_target_ft(want, alt, v.carry, None, None, path_fpm);
            if last_dial.is_none_or(|d| (d - dial).abs() > 50.0) {
                ap.alt_dial = dial;
                last_dial = Some(dial);
            }
            let cap = if unmet.is_some() { best } else { climb };
            let ff = if (want - v.want_alt).abs() < 1.0 { v.target_fpm } else { 0.0 };
            let vs_cmd = vs_law_fpm(want, alt, ff, 2000.0, cap);
            let write = if new {
                let mut a = if ap.vs_mode {
                    VsAction::Write
                } else {
                    vs_action(false, true, alt, want, dial, vs_cmd, ap.vs)
                };
                if a == VsAction::Engage {
                    last_vs = None;
                    ap.engage_vs();
                    if ap.vs_mode {
                        a = VsAction::Write;
                    }
                }
                a == VsAction::Write
            } else {
                if !ap.vs_mode && !vs_mode_capturing(alt, want, dial, vs_cmd) {
                    last_vs = None;
                    ap.engage_vs();
                }
                true
            };
            let vs_cmd = vs_dial_fpm(vs_cmd);
            if write {
                if last_vs.is_none_or(|l| (l - vs_cmd).abs() > 50.0) {
                    if !ap.vs_mode {
                        out.vs_writes_in_alt += 1;
                    }
                    if ap.write_vs(vs_cmd) {
                        last_vs = Some(vs_cmd);
                    } else {
                        last_vs = None;
                        out.unconfirmed += 1;
                    }
                }
            } else {
                last_vs = None;
            }
            reached |= alt >= 5900.0;
            if reached && idx <= 4 {
                out.level.push((alt, ap.vs));
            }
        }
        out.final_alt = ap.alt;
        out
    }

    /// Replayed through the plant: the old chain hunts round 5,900 with the
    /// speed flipping and VS written under ALT HOLD; the new one climbs to
    /// 6,000 once, holds it on the autopilot's own capture, leaves Vy once,
    /// and climbs on for LAM's 8,000.
    #[test]
    fn the_follower_levels_at_the_minimum_once_on_the_737s_modes() {
        let old = fly(false);
        let old_min_vs = old.level.iter().map(|p| p.1).fold(f64::MAX, f64::min);
        eprintln!(
            "old: {} speed flips, min vs {old_min_vs:.0}, {} VS writes in ALT, {} unconfirmed",
            changes(&old.speed),
            old.vs_writes_in_alt,
            old.unconfirmed
        );
        assert!(changes(&old.speed) >= 4, "the old chain hunts: {} speed flips", changes(&old.speed));
        assert!(old_min_vs < -800.0, "and dives for the 5,000 dial: {old_min_vs:.0} fpm");

        let new = fly(true);
        let min_vs = new.level.iter().map(|p| p.1).fold(f64::MAX, f64::min);
        let min_alt = new.level.iter().map(|p| p.0).fold(f64::MAX, f64::min);
        let max_alt = new.level.iter().map(|p| p.0).fold(f64::MIN, f64::max);
        eprintln!(
            "new: {} speed flips, vs {min_vs:.0}.., alt {min_alt:.0}..{max_alt:.0}, D156J {:?}, final {:.0}",
            changes(&new.speed),
            new.alt_at_d156j,
            new.final_alt
        );
        assert_eq!(changes(&new.speed), 1, "Vy to 250 once");
        assert!(min_vs > -300.0, "no dive: {min_vs:.0} fpm");
        assert!(min_alt > 5850.0 && max_alt < 6150.0, "level at 6,000: {min_alt:.0}..{max_alt:.0}");
        assert!(new.alt_at_d156j.is_some_and(|a| a >= 5900.0), "{:?}", new.alt_at_d156j);
        assert_eq!(new.vs_writes_in_alt, 0);
        assert_eq!(new.unconfirmed, 0);
        assert!(new.final_alt > 7800.0, "on for LAM: {:.0}", new.final_alt);
    }
