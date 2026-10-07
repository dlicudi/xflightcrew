    use super::{ArrivalWatch, ARRIVAL_STALL_SECS};

    /// The EGPH arrival, flown as it was flown: armed 121 nm from CI24 and
    /// closing at the 4.0 nm a minute the blackbox measured, a leg that takes
    /// half an hour. The old flat 1,800 s killed it 4 nm short; a watch on
    /// closure never fires, because the aircraft was doing exactly what it was
    /// told the whole way.
    #[test]
    fn a_long_arrival_that_keeps_closing_never_stalls() {
        let mut w = ArrivalWatch::default();
        let mut dist = 121.2;
        let mut t = 0.0;
        while dist > 0.5 {
            assert!(
                !w.stalled(Some("CI24"), Some(dist), t),
                "closing at {dist:.1} nm after {t:.0}s"
            );
            dist -= 4.0 / 60.0;
            t += 1.0;
        }
        assert!(t > 1800.0, "the leg really is longer than the old deadline");
    }

    /// An arrival that has stopped getting anywhere IS a give-up — but only
    /// after the follower's own rejoins have had their go.
    #[test]
    fn an_arrival_that_stops_closing_stalls_once() {
        let mut w = ArrivalWatch::default();
        assert!(!w.stalled(Some("CI24"), Some(12.0), 0.0));
        assert!(!w.stalled(Some("CI24"), Some(12.0), ARRIVAL_STALL_SECS - 1.0));
        assert!(w.stalled(Some("CI24"), Some(12.0), ARRIVAL_STALL_SECS + 1.0));
        // Fires once per stall: the clock restarts behind it.
        assert!(!w.stalled(Some("CI24"), Some(12.0), ARRIVAL_STALL_SECS + 2.0));
    }

    /// A sequenced leg jumps the distance UP. That is progress, not a stall.
    #[test]
    fn a_new_fix_is_judged_afresh() {
        let mut w = ArrivalWatch::default();
        assert!(!w.stalled(Some("CI24"), Some(0.4), 0.0));
        assert!(!w.stalled(Some("FI24"), Some(6.8), 1.0));
        assert!(!w.stalled(Some("FI24"), Some(6.8), ARRIVAL_STALL_SECS - 1.0));
        assert!(w.stalled(Some("FI24"), Some(6.8), ARRIVAL_STALL_SECS + 2.0));
    }

    /// A follower steering at no fix (a to-altitude leg, or between legs) is
    /// the follower's own business: hold the clock, don't start one.
    #[test]
    fn no_fix_holds_the_clock() {
        let mut w = ArrivalWatch::default();
        assert!(!w.stalled(None, None, 0.0));
        assert!(!w.stalled(None, None, ARRIVAL_STALL_SECS * 3.0));
        assert!(!w.stalled(Some("CI24"), Some(9.0), ARRIVAL_STALL_SECS * 3.0 + 1.0));
    }

    /// What full auto will fly into Innsbruck from where the aircraft is:
    /// `cargo test -- --ignored real_install_lowi_arrival --nocapture`
    #[test]
    #[ignore]
    fn real_install_lowi_arrival() {
        let apt = &crate::config::xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
        let cifp = &crate::config::xplane_path("Custom Data/CIFP");
        let fix = &crate::config::xplane_path("Custom Data/earth_fix.dat");
        let nav = &crate::config::xplane_path("Custom Data/earth_nav.dat");
        println!("MOGT3H 26 minimums: {:?}", crate::nav::sid_minimums(cifp, "LOWI", "MOGT3H", "26"));
        for wind in [Some((360.0, 17.0)), None] {
            let a = super::build_arrival(
                apt, cifp, fix, nav, "LSZS", 46.779, 10.259,
                crate::tools::autopilot::ApConfig::default(), 87.0, wind, None, None,
            );
            println!("LSZS wind {wind:?}: runway {:?} plan {:?}", a.rwy_id, a.plan.as_ref().map(|p| p.name.clone()));
            if let Some(p) = &a.plan {
                println!("   visual {:?} missed {:?}", p.visual_deg, p.missed_legs.iter().map(|l| (l.fix_ident.clone(), l.alt1_ft, l.fix.is_some())).collect::<Vec<_>>());
                for l in &p.legs {
                    println!("   {} {} {:?} map={} vpa={:?} fix={}", l.leg_type, l.fix_ident, l.alt1_ft, l.is_map, l.vpa_deg, l.fix.is_some());
                }
            }
        }
        match super::brief_arrival(apt, cifp, fix, "LSZS", None) {
            Ok(l) | Err(l) => println!("LSZS brief: {l}"),
        }
        for forced in ["08", "26"] {
            let a = super::build_arrival(
                apt, cifp, fix, nav, "LOWI", 47.77, 11.79,
                crate::tools::autopilot::ApConfig::default(), 87.0, Some((80.0, 6.0)), Some(forced), None,
            );
            println!("tower wants {forced}: plan {:?}", a.plan.as_ref().map(|p| p.name.clone()));
        }
        for wind in [Some((80.0, 6.0)), Some((110.0, 4.0)), Some((260.0, 12.0)), None] {
            let a = super::build_arrival(
                apt, cifp, fix, nav, "LOWI", 47.77, 11.79,
                crate::tools::autopilot::ApConfig::default(), 87.0, wind, None, None,
            );
            println!("wind {wind:?}: runway {:?}", a.rwy_id);
            match &a.plan {
                None => println!("  NO PLAN (visual landing)"),
                Some(p) => {
                    println!("  {}", p.name);
                    for l in &p.legs {
                        println!(
                            "  {} {} {:?} fix={}",
                            l.leg_type,
                            l.fix_ident,
                            l.alt1_ft,
                            l.fix.is_some()
                        );
                    }
                }
            }
        }
    }
