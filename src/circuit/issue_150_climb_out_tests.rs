    use super::*;
    use crate::autotakeoff::climb_out_bench::{self, VsMode};
    use crate::autotakeoff::{
        climb_guard_vne, climb_out_vs_fpm, leaves_flaps_to_caller, paced_flap_step, ClimbVs,
        CLIMB_VS_SLEW_FPM_S,
    };

    fn b738() -> (Vec<(f64, f64)>, Vec<(f64, Option<f64>)>) {
        let p = crate::aircraft::load("boeing_737_800").expect("737 profile loads");
        let limits = p
            .flaps
            .as_ref()
            .map(|f| f.detents.iter().map(|d| (d.ratio, d.vfe_kts)).collect())
            .unwrap_or_default();
        (crate::aircraft::flap_schedule(p.flaps.as_ref()), limits)
    }

    /// The recorded LEMG 13 handoffs (2026-09-28): the pattern's first
    /// command came a thousand fpm under the takeoff's last, and the flaps
    /// went 5, 1, up inside a second and a half at 191 kt, under the 737's
    /// 210 kt flaps-up speed.
    #[test]
    fn the_recorded_handoffs_dropped_the_climb_and_dumped_the_flaps() {
        for r in climb_out_bench::load() {
            let last = r
                .commands
                .iter()
                .rfind(|c| c.0 <= r.handoff_t)
                .expect("a takeoff climb command");
            let first = r
                .commands
                .iter()
                .find(|c| c.0 > r.handoff_t + 0.05)
                .expect("a pattern command");
            assert!(
                last.1 - first.1 > 800.0,
                "{}: {:.0} then {:.0}",
                r.sortie,
                last.1,
                first.1
            );
            let n = r.flaps.len();
            assert!(n >= 2, "{}: {:?}", r.sortie, r.flaps);
            let (a, b) = (r.flaps[n - 2], r.flaps[n - 1]);
            assert!(b.1 == 0.0 && b.0 - a.0 < 1.5, "{}: {:?}", r.sortie, r.flaps);
        }
    }

    /// The carry walks at the slew, arrives and lets go; the approach legs
    /// and the climb floor are the law's at once; nothing handed, nothing
    /// carried.
    #[test]
    fn the_carry_walks_to_the_pattern_law_and_lets_go() {
        let step = CLIMB_VS_SLEW_FPM_S * 0.5;
        let mut c = ClimbCarry::new(Some(2600.0));
        assert_eq!(c.step(456.0, 0.5, false, false), 2600.0 - step);
        let mut n = 1;
        while c.carrying() {
            c.step(456.0, 0.5, false, false);
            n += 1;
            assert!(n < 100);
        }
        assert_eq!(n, ((2600.0_f64 - 456.0) / step).ceil() as usize);
        assert_eq!(c.step(700.0, 0.5, false, false), 700.0, "released");
        let mut c = ClimbCarry::new(Some(2600.0));
        assert_eq!(
            c.step(-600.0, 0.5, true, false),
            -600.0,
            "base descends at once"
        );
        assert!(!c.carrying());
        let mut c = ClimbCarry::new(Some(2600.0));
        assert_eq!(
            c.step(0.0, 0.5, false, true),
            0.0,
            "slow is the protection's"
        );
        let mut c = ClimbCarry::new(None);
        assert_eq!(c.step(456.0, 0.5, false, false), 456.0);
        assert_eq!(
            takeoff_flaps_to_clean(Some(0.375), &[(0.0, 210.0)]),
            Some(0.375)
        );
        assert_eq!(takeoff_flaps_to_clean(Some(0.0), &[(0.0, 210.0)]), None);
        assert_eq!(takeoff_flaps_to_clean(Some(0.375), &[]), None);
        assert_eq!(takeoff_flaps_to_clean(None, &[(0.0, 210.0)]), None);
    }

    struct Trace {
        /// Largest change between consecutive VS commands (the first from
        /// the climb flown), fpm.
        max_step: f64,
        /// (seconds after the engage, agl, ias, handle) for every flap move.
        flaps: Vec<(f64, f64, f64, f64)>,
        handoff_t: f64,
        max_agl: f64,
        min_ias: f64,
        /// Fastest with each handle out, as (handle, kts).
        dirty_max: Vec<(f64, f64)>,
    }

    /// The 737 from the 300 ft engage (`ias0`, climbing `vs0`) to 60 s past
    /// the 500 ft handoff: the takeoff's climb, then the pattern's to a
    /// 1,500 ft pattern, through the VS mode (2 s dead, two 1.5 s lags), on
    /// #78's thrust fit (0.185) with the pattern's throttle holding its speed.
    /// `new` flies #150; else the laws as flown on 2026-09-28.
    fn fly(new: bool, ias0: f64, vs0: f64) -> Trace {
        let (sched, limits) = b738();
        let vne = climb_guard_vne(Some(340.0));
        let floor = climb_floor_kts(None, Some(139.0));
        let full = climb_full_rate_kts(210.0, 210.0, floor);
        let (mut agl, mut ias, mut t) = (300.0_f64, ias0, 0.0_f64);
        let mut ap = VsMode::new(vs0, 0.1);
        let mut takeoff = ClimbVs::default();
        let (mut flap, mut deployed) = (0.375_f64, 0.375_f64);
        let mut tw = 0.185_f64;
        let (mut prev_cmd, mut max_step) = (vs0, 0.0_f64);
        let mut cmd = vs0;
        let mut handoff_t: Option<f64> = None;
        let mut carry = ClimbCarry::default();
        let mut cleaning: Option<f64> = None;
        let (mut vs_est, mut trend, mut prev_ias) = (vs0, 0.0_f64, ias0);
        let mut next_tick = 0.0_f64;
        let mut above_500: Option<f64> = None;
        let mut tr = Trace {
            max_step: 0.0,
            flaps: Vec::new(),
            handoff_t: 0.0,
            max_agl: agl,
            min_ias: ias,
            dirty_max: Vec::new(),
        };
        let sub = 0.1;
        while handoff_t.is_none_or(|h| t < h + 60.0) {
            if t >= next_tick - 1e-9 {
                let dt = if handoff_t.is_some() { 0.6 } else { 0.4 };
                next_tick = t + dt;
                vs_est = 0.6 * vs_est + 0.4 * ap.vs;
                trend = 0.6 * trend + 0.4 * (ias - prev_ias) / dt;
                prev_ias = ias;
                let mut want_kts = 210.0;
                if handoff_t.is_none() {
                    cmd = if new {
                        takeoff
                            .step(ias, 170.0, 1500.0, vne, agl, None, Some(ap.vs), dt, false)
                            .1
                    } else {
                        climb_out_vs_fpm(ias, 170.0, 1500.0, vne, agl, None)
                    };
                    if flap > 1e-6 && agl > 400.0 {
                        if let Some(n) = paced_flap_step(&sched, flap, Some(deployed), ias) {
                            flap = n;
                            tr.flaps.push((t, agl, ias, n));
                        }
                    }
                    // The old handoff waited, dirty, up to 30 s for the clean
                    // floor (`handoff_waits_for_clean_up`).
                    let old_waits = !new
                        && agl >= 500.0
                        && crate::autotakeoff::handoff_waits_for_clean_up(
                            flap <= 1e-6,
                            ias,
                            Some(167.0),
                            t - *above_500.get_or_insert(t),
                        );
                    if agl >= 500.0 && !old_waits {
                        handoff_t = Some(t);
                        tr.handoff_t = t;
                        if new && leaves_flaps_to_caller(flap <= 1e-6, true, true) {
                            cleaning = takeoff_flaps_to_clean(Some(flap), &sched);
                            carry = ClimbCarry::new(Some(cmd));
                        } else if flap > 1e-6 && ias > 167.0 {
                            flap = 0.0;
                            tr.flaps.push((t, agl, ias, 0.0));
                        }
                    }
                } else {
                    // The live loop's floor: the kept flap's while cleaning
                    // up (`leg_climb_floor_kts`, #169), on the 737's straight
                    // line from the clean 1.2 Vs (167) to the landing 1.2
                    // Vso (137). With the climb floor at the automation
                    // floor (1.2 Vs, refactor audit #13), the clean one sits
                    // over the 165 kt engage and would let the carry go.
                    let (floor, full) = if new {
                        let f = leg_climb_floor_kts(Leg::Upwind, floor, Some(139.0), cleaning, |r| {
                            Some((167.0 + (137.0 - 167.0) * r).round())
                        });
                        (f, climb_full_rate_kts(210.0, 210.0, f))
                    } else {
                        (floor, full)
                    };
                    let law = climb_limit_fpm(
                        alt_capture_vs(1500.0, 0.0, agl, vs_est),
                        ias,
                        trend,
                        full,
                        floor,
                    );
                    cmd = carry.step(law, dt, false, ias <= floor);
                    if let Some(f) = cleaning {
                        if let Some(n) = go_around_flap_step(&sched, f, Some(deployed), ias, 167.0)
                        {
                            flap = n;
                            cleaning = (n > 1e-6).then_some(n);
                            tr.flaps.push((t, agl, ias, n));
                        }
                    }
                    if let Some(f) = cleaning {
                        want_kts = go_around_speed_kts(210.0, f, &sched, &limits, None);
                    }
                    // The pattern's throttle: the path's share of the
                    // thrust, plus a hundredth per knot short of the speed.
                    let sin_g = ap.vs / (ias * 101.27);
                    let want_tw = (sin_g + 0.01 * (want_kts - ias)).clamp(0.0, 0.185);
                    tw += (want_tw - tw) * (dt / 2.0).min(1.0);
                }
                max_step = max_step.max((cmd - prev_cmd).abs());
                prev_cmd = cmd;
            }
            let vs = ap.step(cmd, sub);
            ias += climb_out_bench::accel_kts_s(tw, ias, vs) * sub;
            agl += vs / 60.0 * sub;
            deployed += (flap - deployed).clamp(-0.03 * sub, 0.03 * sub);
            t += sub;
            tr.max_agl = tr.max_agl.max(agl);
            tr.min_ias = tr.min_ias.min(ias);
            if flap > 1e-6 {
                match tr.dirty_max.iter_mut().find(|d| (d.0 - flap).abs() < 1e-6) {
                    Some(d) => d.1 = d.1.max(ias),
                    None => tr.dirty_max.push((flap, ias)),
                }
            }
        }
        tr.max_step = max_step;
        tr
    }

    /// #150 end to end: from the recorded LEMG 13 engage (186 kt, the
    /// climb flown read off the recorded heights) and from #78's (165 kt,
    /// about 3,090 fpm), the old laws step the dial by a thousand fpm or
    /// more and dump the flaps at the handoff; the new ones never move the
    /// command faster than the slew, walk the flaps up at their speeds with
    /// each one arrived, inside their placards, and still level at the
    /// pattern height with the speed above the floor.
    #[test]
    fn the_pattern_takes_the_climb_and_the_flaps_from_the_takeoff() {
        let (sched, limits) = b738();
        let rec = &climb_out_bench::load()[0];
        let starts = [
            (rec.engage.2, climb_out_bench::flown_fpm(&rec.manual)),
            (165.0, climb_out_bench::held_climb_fpm(0.185, 165.0)),
        ];
        for (ias0, vs0) in starts {
            let tag = format!("{ias0:.0} kt, {vs0:.0} fpm");
            let old = fly(false, ias0, vs0);
            assert!(
                old.max_step > 900.0,
                "old dial step {:.0} ({tag})",
                old.max_step
            );
            let dumped = old
                .flaps
                .iter()
                .any(|f| f.3 == 0.0 && (f.0 - old.handoff_t).abs() < 1e-6);
            assert!(dumped, "old flaps {:?} ({tag})", old.flaps);

            let new = fly(true, ias0, vs0);
            assert!(
                new.max_step <= CLIMB_VS_SLEW_FPM_S * 0.6 + 1e-6,
                "new dial step {:.0} ({tag})",
                new.max_step
            );
            let steps: Vec<f64> = new.flaps.iter().map(|f| f.3).collect();
            assert_eq!(steps, vec![0.125, 0.0], "5, 1, up ({tag}): {:?}", new.flaps);
            for f in &new.flaps {
                let speed = sched.iter().find(|s| (s.0 - f.3).abs() < 1e-6).unwrap().1;
                assert!(f.2 >= speed, "flaps {} at {:.1} kt ({tag})", f.3, f.2);
            }
            assert!(
                new.flaps[1].0 - new.flaps[0].0 >= 4.0,
                "one step after the other arrived ({tag}): {:?}",
                new.flaps
            );
            assert!(
                new.flaps[1].0 - new.handoff_t < 45.0,
                "clean inside the crosswind ({tag}): {:?}",
                new.flaps
            );
            for (flap, kts) in &new.dirty_max {
                let vfe = limits
                    .iter()
                    .filter(|l| l.0 <= flap + 1e-6)
                    .filter_map(|l| l.1)
                    .fold(f64::INFINITY, f64::min);
                assert!(
                    *kts < vfe - 5.0,
                    "flaps {flap} at {kts:.1} vs {vfe} ({tag})"
                );
            }
            assert!(
                new.max_agl < 1600.0,
                "overshot the pattern: {:.0} ({tag})",
                new.max_agl
            );
            // Never slower than the clean floor, or than the engage when that
            // was already under it with the flaps out (#78's 165 kt sits under
            // the 1.2 Vs clean floor of 166.8: the speed was never bled).
            assert!(
                new.min_ias >= ias0.min(climb_floor_kts(None, Some(139.0))) - 1e-9,
                "{:.1} ({tag})",
                new.min_ias
            );
        }
    }
