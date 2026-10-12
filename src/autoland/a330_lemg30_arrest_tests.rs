    use super::*;

    /// The commit that lasted one tick: 46.4 ft, 1,432 m ahead of a 4,750 ft
    /// (1,448 m) book, then 46.1 ft and about 1,409 m. Judged afresh, the
    /// second tick is a go-around; latched, the landing continues while the
    /// pavement covers the book's ground roll, and not past it.
    #[test]
    fn a_committed_flare_stays_committed() {
        let book = Some(4750.0 * M_PER_FT);
        let need = Some(wheels_need_m(book, None));
        let over = Guidance {
            target_heading_deg: 120.0,
            target_vs_fpm: 1000.0,
            target_ias_kts: 150.0,
            dist_nm: 0.7,
            glidepath_alt_ft: 0.0,
            phase: ApproachPhase::Intercept,
            balk: Some("overflew_the_runway"),
            arresting: false,
        };
        let first = commit_flare(over, true, 46.4, 150.0, 25.0, 1432.0, book, 297.7);
        assert_eq!((first.phase, first.balk), (ApproachPhase::Flare, None));
        let fresh = commit_flare(over, true, 46.1, 150.0, 25.0, 1409.0, book, 297.7);
        assert_eq!(fresh.balk, Some("overflew_the_runway"), "as flown: released");
        let held = commit_flare_latched(over, true, need, 46.1, 150.0, 25.0, 1409.0, book, 297.7);
        assert_eq!((held.phase, held.balk), (ApproachPhase::Flare, None));
        assert!(held.arresting);
        // Past the ground roll (1,448 m less the 291 m air segment): around.
        let short = commit_flare_latched(over, true, need, 30.0, 150.0, 25.0, 1100.0, book, 297.7);
        assert_eq!(short.balk, Some("overflew_the_runway"));
        // Never above the band, latched or not.
        let (_, band) = arrest_gates_ft(25.0, 150.0);
        let high =
            commit_flare_latched(over, true, need, band + 1.0, 150.0, 25.0, 2000.0, book, 297.7);
        assert_eq!(high.balk, Some("overflew_the_runway"));
    }

