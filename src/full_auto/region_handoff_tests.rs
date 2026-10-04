    use super::region_readback;
    use crate::party_line::RegionHandoff;

    /// The readback carries a readback word the tower FSM recognises, the
    /// station and the frequency; the squawk only when one was ordered.
    #[test]
    fn readback_is_readback_shaped() {
        let cs = Some("November one seven two Sierra Papa".to_string());
        let h = RegionHandoff {
            to: "Malaga Tower".into(),
            khz: 118_100,
            squawk_vfr: true,
            ifr_code: None,
        };
        assert_eq!(
            region_readback(&cs, &h),
            "November one seven two Sierra Papa, squawking seven thousand, Malaga Tower on one one eight decimal one"
        );
        assert!(crate::tower::is_readback(&region_readback(&cs, &h)));
        let h = RegionHandoff {
            squawk_vfr: false,
            ..h
        };
        let line = region_readback(&None, &h);
        assert_eq!(line, "roger, Malaga Tower on one one eight decimal one");
        assert!(!line.contains("thousand"));
        assert!(crate::tower::is_readback(&line));
    }

    fn handoff(squawk_vfr: bool, ifr_code: Option<i64>) -> RegionHandoff {
        RegionHandoff {
            to: "Barcelona Tower".into(),
            khz: 118_105,
            squawk_vfr,
            ifr_code,
        }
    }

    /// LIEA-LEBL, 2026-09-27: the IFR 737 on 4501 was handed to Barcelona
    /// Tower. With the code kept, nothing is dialled and COM1 is retuned.
    #[tokio::test]
    async fn an_ifr_handoff_keeps_the_code() {
        use crate::xplane::FakeXPlane;
        let xp = FakeXPlane::with_reads(&[(crate::datarefs::TRANSPONDER_CODE, 4501.0)]);
        let cfg = crate::config::Config::default();
        let ctx = crate::tools::ToolCtx { xplane: &xp, config: &cfg, voice: None, status: None, reminders: None };
        super::obey_region_handoff(&xp, &ctx, &None, &None, handoff(false, Some(4501)), false).await;
        assert!(xp.writes_to(crate::datarefs::TRANSPONDER_CODE).is_empty(), "no squawk dialled");
        assert_eq!(xp.writes_to(super::COM1_FREQ_KHZ), vec![118_105.0]);
    }

    /// A VFR service ending still gives the code back: 7000 is dialled.
    #[tokio::test]
    async fn a_vfr_handoff_still_sets_seven_thousand() {
        use crate::xplane::FakeXPlane;
        let xp = FakeXPlane::with_reads(&[(crate::datarefs::TRANSPONDER_CODE, 4501.0)]);
        let cfg = crate::config::Config::default();
        let ctx = crate::tools::ToolCtx { xplane: &xp, config: &cfg, voice: None, status: None, reminders: None };
        super::obey_region_handoff(&xp, &ctx, &None, &None, handoff(true, None), false).await;
        assert_eq!(xp.writes_to(crate::datarefs::TRANSPONDER_CODE), vec![7000.0]);
    }
