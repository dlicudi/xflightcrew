use super::*;
use crate::nav::{Fix, LegPath};

fn leg(kind: &str, ident: &str) -> ProcLeg {
    ProcLeg {
        leg_type: kind.into(),
        fix_ident: ident.into(),
        fix: Some(Fix {
            ident: ident.into(),
            lat: 40.0,
            lon: -3.0,
        }),
        ..Default::default()
    }
}

/// A tick on leg `idx` (ident `ident`, next `next`), the FMS on `fms`.
fn tick<'a>(
    now_s: f64,
    idx: usize,
    ident: &'a str,
    next: Option<&'a str>,
    fms: Option<&'a str>,
    engaged: Option<bool>,
    xtk: Option<f64>,
) -> Tick<'a> {
    Tick {
        now_s,
        idx,
        eligible: true,
        leg_ident: ident,
        next_ident: next,
        fms_ident: fms,
        engaged,
        xtk_nm: xtk,
        gs_kts: 450.0,
    }
}

/// Engaged and captured on leg 3 (BLN, next NENEM) at `t`, LNAV in.
fn captured_on_bln() -> (LnavLateral, f64) {
    let mut l = LnavLateral::default();
    let d = l.decide(tick(
        0.0,
        3,
        "BLN",
        Some("NENEM"),
        Some("BLN"),
        Some(false),
        Some(3.0),
    ));
    assert_eq!(d.act, Act::Press);
    let d = l.decide(tick(
        2.0,
        3,
        "BLN",
        Some("NENEM"),
        Some("BLN"),
        Some(true),
        Some(3.0),
    ));
    assert!(matches!(
        d.note,
        Some(Note::Engaged {
            say: true,
            reengage: false,
            presses: 1
        })
    ));
    let d = l.decide(tick(
        60.0,
        3,
        "BLN",
        Some("NENEM"),
        Some("BLN"),
        Some(true),
        Some(0.0),
    ));
    assert!(matches!(d.note, Some(Note::Captured { .. })), "{d:?}");
    (l, 60.0)
}

#[test]
fn only_a_plain_leg_to_a_fix_is_lnavs() {
    assert!(leg_flyable(&leg("TF", "BLN")));
    assert!(leg_flyable(&leg("DF", "BLN")));
    assert!(leg_flyable(&leg("IF", "BLN")));
    assert!(!leg_flyable(&leg("CF", "BLN")), "a course leg is ours");
    assert!(!leg_flyable(&leg("HM", "BLN")), "a hold is ours");
    assert!(!leg_flyable(&leg("CA", "")), "a to-altitude leg has no fix");
    let mut map = leg("TF", "RW13");
    map.is_map = true;
    assert!(!leg_flyable(&map), "a MAP is the approach's");
    let mut rf = leg("RF", "WI505");
    rf.turn = Some('R');
    rf.path = LegPath {
        arc_center: Some(Fix {
            ident: "C".into(),
            lat: 40.05,
            lon: -3.0,
        }),
        arc_radius_nm: Some(3.0),
        ..Default::default()
    };
    assert!(rf.arc().is_some());
    assert!(!leg_flyable(&rf), "an RF arc is ours");
    let mut unresolved = leg("TF", "BLN");
    unresolved.fix = None;
    assert!(!leg_flyable(&unresolved));
    assert!(!leg_flyable(&leg("TF", " ")));
}

#[test]
fn idents_compare_as_the_payload_spelt_them() {
    assert!(same_ident("BLN", "bln"));
    assert!(same_ident(" D156J ", "D156J"));
    assert!(same_ident("D156J", "D156-J"));
    assert!(!same_ident("", "BLN"), "an empty FMS never agrees");
    assert!(!same_ident("WPT", ""), "an empty ident never agrees");
    assert!(!same_ident("PIMOS", "BLN"));
}

/// The probe's 180 degree turn: after the load the FMS's active leg is the
/// plan's FIRST (PIMOS, 645 nm behind). LNAV is pressed only once the FMS
/// is on the follower's leg; a goto lost to the load is sent once more.
#[test]
fn lnav_is_pressed_only_once_the_fms_is_on_the_followers_leg() {
    let mut l = LnavLateral::default();
    for s in 0..10 {
        let t = s as f64 * 0.5;
        let d = l.decide(tick(
            t,
            3,
            "BLN",
            Some("NENEM"),
            Some("PIMOS"),
            Some(false),
            Some(180.0),
        ));
        assert_eq!(d.act, Act::Nothing, "never pressed toward PIMOS at {t}");
        assert!(l.bug_write_allowed());
    }
    let d = l.decide(tick(
        5.0,
        3,
        "BLN",
        Some("NENEM"),
        Some("PIMOS"),
        Some(false),
        None,
    ));
    assert_eq!(d.act, Act::ReGoto, "the goto again, once");
    let d = l.decide(tick(
        11.0,
        3,
        "BLN",
        Some("NENEM"),
        Some("PIMOS"),
        Some(false),
        None,
    ));
    assert_eq!(d.act, Act::Nothing, "one re-goto a leg");
    let d = l.decide(tick(
        12.0,
        3,
        "BLN",
        Some("NENEM"),
        Some("BLN"),
        Some(false),
        Some(4.0),
    ));
    assert_eq!(d.act, Act::Press);
    assert!(!l.owns_lateral(), "the bug flies until LNAV reads in");
    assert!(
        !l.heading_mode_wanted(12.0),
        "no HDG SEL pressed over the engage"
    );
    let d = l.decide(tick(
        13.0,
        3,
        "BLN",
        Some("NENEM"),
        Some("BLN"),
        Some(false),
        Some(4.0),
    ));
    assert_eq!(d.act, Act::Nothing, "a press settles before it is judged");
    let d = l.decide(tick(
        14.0,
        3,
        "BLN",
        Some("NENEM"),
        Some("BLN"),
        Some(true),
        Some(4.0),
    ));
    assert_eq!(
        d.note,
        Some(Note::Engaged {
            say: true,
            reengage: false,
            presses: 1
        })
    );
    assert!(l.owns_lateral());
}

/// setup_enroute loads nothing: `gps_nav_id` "" never agrees, so LNAV is
/// never pressed on an empty FMS.
#[test]
fn an_empty_fms_is_never_engaged() {
    let mut l = LnavLateral::default();
    let mut regotos = 0;
    for s in 0..200 {
        let t = s as f64 * 0.5;
        let idx = if t < 50.0 { 0 } else { 1 };
        let d = l.decide(tick(
            t,
            idx,
            "BLN",
            Some("NENEM"),
            Some(""),
            Some(false),
            Some(0.0),
        ));
        assert_ne!(d.act, Act::Press);
        if d.act == Act::ReGoto {
            regotos += 1;
        }
    }
    assert_eq!(regotos, 2, "one re-goto per leg");
}

/// A leg whose fix the payload did not carry (a synthetic gateway, an
/// approach leg) never agrees: LNAV hands it back to the bug, silently.
#[test]
fn a_leg_not_in_the_fms_stays_on_the_bug() {
    let (mut l, t0) = captured_on_bln();
    // The follower sequences to GATE; the goto changes nothing in the FMS,
    // which goes on to NENEM by itself.
    let mut out = None;
    for s in 0..20 {
        let t = t0 + 1.0 + s as f64 * 0.5;
        let d = l.decide(tick(
            t,
            4,
            "GATE",
            Some("LEMG"),
            Some("NENEM"),
            Some(true),
            Some(0.1),
        ));
        if d.act != Act::Nothing {
            out = Some((t, d.act));
            break;
        }
    }
    let (t, act) = out.expect("handed back");
    assert_eq!(act, Act::HandBack("leg_not_in_fms"));
    assert!(t - t0 >= IDENT_MISMATCH_S);
    assert_eq!(l.fallbacks(), 0, "not a fallback: nothing said");
    assert!(l.bug_write_allowed());
    for s in 0..40 {
        let d = l.decide(tick(
            t + 1.0 + s as f64 * 0.5,
            4,
            "GATE",
            Some("LEMG"),
            Some("NENEM"),
            Some(false),
            None,
        ));
        assert_ne!(
            d.act,
            Act::Press,
            "never pressed on a leg the FMS does not hold"
        );
        assert!(l.bug_write_allowed());
    }
}

/// The probe: from about 4 nm off at a 40 degree intercept, captured to 0.0
/// with one roll-out. Captured, and nothing more.
#[test]
fn a_capture_like_the_probes_is_captured() {
    let mut l = LnavLateral::default();
    l.decide(tick(
        0.0,
        3,
        "BLN",
        None,
        Some("BLN"),
        Some(false),
        Some(4.0),
    ));
    l.decide(tick(
        2.0,
        3,
        "BLN",
        None,
        Some("BLN"),
        Some(true),
        Some(4.0),
    ));
    let mut captured = None;
    for s in 0..400 {
        let t = 2.5 + s as f64 * 0.5;
        // The turn onto the intercept first opens it a little.
        let x = if t < 30.0 {
            4.0 + (t - 2.0) * 0.02
        } else {
            (4.5 - (t - 30.0) * 0.05).max(0.0)
        };
        let d = l.decide(tick(t, 3, "BLN", None, Some("BLN"), Some(true), Some(-x)));
        assert!(!matches!(d.act, Act::Drop(_)), "{d:?} at {t}");
        assert!(!l.bug_write_allowed(), "LNAV flies: no bug at {t}");
        if let Some(Note::Captured { secs }) = d.note {
            captured = Some(secs);
        }
    }
    let secs = captured.expect("captured");
    assert!(secs > 100.0 && secs < 130.0, "{secs}");
}

/// Captured and then 2.5 nm off: a fallback, said once; not retried on the
/// same leg; retried, and said again, on the next.
#[test]
fn a_captured_lnav_off_the_leg_falls_back_and_retries_on_the_next_leg() {
    let (mut l, t0) = captured_on_bln();
    let d = l.decide(tick(
        t0 + 1.0,
        3,
        "BLN",
        Some("NENEM"),
        Some("BLN"),
        Some(true),
        Some(1.9),
    ));
    assert_eq!(d.act, Act::Nothing);
    let d = l.decide(tick(
        t0 + 2.0,
        3,
        "BLN",
        Some("NENEM"),
        Some("BLN"),
        Some(true),
        Some(-2.5),
    ));
    assert_eq!(d.act, Act::Drop(Why::CrossTrack));
    assert!(l.bug_write_allowed());
    assert!(
        !l.heading_mode_wanted(t0 + 2.5),
        "the hand-back's HDG SEL settles"
    );
    assert!(l.heading_mode_wanted(t0 + 4.0));
    for s in 0..20 {
        let d = l.decide(tick(
            t0 + 3.0 + s as f64,
            3,
            "BLN",
            Some("NENEM"),
            Some("BLN"),
            Some(false),
            Some(2.0),
        ));
        assert_eq!(d.act, Act::Nothing, "never retried on the leg it dropped");
    }
    let d = l.decide(tick(
        t0 + 30.0,
        4,
        "NENEM",
        None,
        Some("NENEM"),
        Some(false),
        Some(2.0),
    ));
    assert_eq!(d.act, Act::Press, "retried on the next leg");
    let d = l.decide(tick(
        t0 + 32.0,
        4,
        "NENEM",
        None,
        Some("NENEM"),
        Some(true),
        Some(2.0),
    ));
    assert_eq!(
        d.note,
        Some(Note::Engaged {
            say: true,
            reengage: true,
            presses: 1
        })
    );
}

/// The lamp out for two seconds past a press's settle is lost; a flicker,
/// or an unread, is not.
#[test]
fn lnav_lost_falls_back_after_two_seconds() {
    let (mut l, t0) = captured_on_bln();
    let d = l.decide(tick(
        t0 + 0.5,
        3,
        "BLN",
        None,
        Some("BLN"),
        Some(false),
        Some(0.0),
    ));
    assert_eq!(d.act, Act::Nothing);
    let d = l.decide(tick(
        t0 + 1.0,
        3,
        "BLN",
        None,
        Some("BLN"),
        Some(true),
        Some(0.0),
    ));
    assert_eq!(d.act, Act::Nothing);
    let d = l.decide(tick(t0 + 1.5, 3, "BLN", None, Some("BLN"), None, None));
    assert_eq!(d.act, Act::Nothing);
    l.decide(tick(
        t0 + 2.0,
        3,
        "BLN",
        None,
        Some("BLN"),
        Some(false),
        Some(0.0),
    ));
    let d = l.decide(tick(
        t0 + 3.0,
        3,
        "BLN",
        None,
        Some("BLN"),
        Some(false),
        Some(0.0),
    ));
    assert_eq!(d.act, Act::Nothing);
    let d = l.decide(tick(
        t0 + 4.0,
        3,
        "BLN",
        None,
        Some("BLN"),
        Some(false),
        Some(0.0),
    ));
    assert_eq!(d.act, Act::Drop(Why::StatusLost));
}

/// A reload while LNAV flies makes the first leg active again (the probe's
/// PIMOS): a goto at once, and a drop if the FMS will not come back.
#[test]
fn a_reload_under_lnav_is_answered_with_a_goto_then_a_drop() {
    let (mut l, t0) = captured_on_bln();
    let mut acts = Vec::new();
    for s in 0..30 {
        let t = t0 + 0.5 + s as f64 * 0.5;
        let d = l.decide(tick(
            t,
            3,
            "BLN",
            Some("NENEM"),
            Some("PIMOS"),
            Some(true),
            Some(150.0),
        ));
        if d.act != Act::Nothing {
            acts.push((t - t0, d.act));
        }
    }
    assert_eq!(acts.len(), 2, "{acts:?}");
    assert_eq!(acts[0].1, Act::ReGoto);
    assert!(acts[0].0 >= IDENT_MISMATCH_S && acts[0].0 < IDENT_MISMATCH_S + 1.0);
    assert_eq!(acts[1].1, Act::Drop(Why::FmsDisagrees));
}

/// A goto that lands brings it back: no drop.
#[test]
fn a_goto_that_lands_keeps_lnav() {
    let (mut l, t0) = captured_on_bln();
    for s in 0..12 {
        l.decide(tick(
            t0 + 0.5 + s as f64 * 0.5,
            3,
            "BLN",
            None,
            Some("PIMOS"),
            Some(true),
            Some(150.0),
        ));
    }
    for s in 0..60 {
        let d = l.decide(tick(
            t0 + 7.0 + s as f64 * 0.5,
            3,
            "BLN",
            None,
            Some("BLN"),
            Some(true),
            Some(0.1),
        ));
        assert!(!matches!(d.act, Act::Drop(_) | Act::HandBack(_)), "{d:?}");
    }
    assert!(l.owns_lateral());
}

/// LNAV's own turn anticipation sequences the FMS one leg ahead of the
/// follower: agreement, and the new leg's capture is judged afresh (the
/// cross-track to the new leg at a flyby is miles).
#[test]
fn the_fms_sequencing_ahead_by_itself_is_agreement() {
    let (mut l, t0) = captured_on_bln();
    for s in 0..120 {
        let t = t0 + 0.5 + s as f64 * 0.5;
        let x = (6.0 - s as f64 * 0.1).max(0.0);
        let d = l.decide(tick(
            t,
            3,
            "BLN",
            Some("NENEM"),
            Some("NENEM"),
            Some(true),
            Some(x),
        ));
        assert!(
            !matches!(d.act, Act::Drop(_) | Act::HandBack(_) | Act::ReGoto),
            "{d:?} at {t}"
        );
    }
    // The follower catches up: no disagreement either.
    let d = l.decide(tick(
        t0 + 61.0,
        4,
        "NENEM",
        None,
        Some("NENEM"),
        Some(true),
        Some(0.0),
    ));
    assert!(!matches!(d.act, Act::Drop(_) | Act::HandBack(_)), "{d:?}");
}

/// Opening from the best seen, past the grace: diverging. Inside the grace
/// a turn may open it.
#[test]
fn a_capture_that_opens_falls_back_past_the_grace() {
    let mut l = LnavLateral::default();
    l.decide(tick(
        0.0,
        3,
        "BLN",
        None,
        Some("BLN"),
        Some(false),
        Some(3.0),
    ));
    l.decide(tick(
        2.0,
        3,
        "BLN",
        None,
        Some("BLN"),
        Some(true),
        Some(3.0),
    ));
    let d = l.decide(tick(
        60.0,
        3,
        "BLN",
        None,
        Some("BLN"),
        Some(true),
        Some(5.5),
    ));
    assert_eq!(d.act, Act::Nothing, "inside the grace");
    let d = l.decide(tick(
        130.0,
        3,
        "BLN",
        None,
        Some("BLN"),
        Some(true),
        Some(5.1),
    ));
    assert_eq!(d.act, Act::Drop(Why::Diverging));
}

/// Never captured inside its allowance: a fallback.
#[test]
fn a_capture_that_never_closes_falls_back() {
    assert!((capture_allowance_s(0.0, 450.0) - CAPTURE_MIN_S).abs() < 1e-9);
    let mut l = LnavLateral::default();
    l.decide(tick(
        0.0,
        3,
        "BLN",
        None,
        Some("BLN"),
        Some(false),
        Some(3.0),
    ));
    l.decide(tick(
        2.0,
        3,
        "BLN",
        None,
        Some("BLN"),
        Some(true),
        Some(3.0),
    ));
    let allowance = capture_allowance_s(3.0, 450.0);
    let d = l.decide(tick(
        2.0 + allowance - 1.0,
        3,
        "BLN",
        None,
        Some("BLN"),
        Some(true),
        Some(2.0),
    ));
    assert_eq!(d.act, Act::Nothing);
    let d = l.decide(tick(
        2.0 + allowance + 1.0,
        3,
        "BLN",
        None,
        Some("BLN"),
        Some(true),
        Some(2.0),
    ));
    assert_eq!(d.act, Act::Drop(Why::NoCapture));
}

/// Two presses, settled, still out: given up, the bug flies.
#[test]
fn an_lnav_that_will_not_come_in_is_given_up() {
    let mut l = LnavLateral::default();
    let mut presses = 0;
    let mut last = Act::Nothing;
    for s in 0..20 {
        let d = l.decide(tick(
            s as f64 * 0.5,
            3,
            "BLN",
            None,
            Some("BLN"),
            Some(false),
            Some(1.0),
        ));
        if d.act == Act::Press {
            presses += 1;
        }
        if d.act != Act::Nothing {
            last = d.act;
        }
    }
    assert_eq!(presses, crate::autoflight::LNAV_PRESSES);
    assert_eq!(last, Act::Drop(Why::NoEngage));
    assert!(l.bug_write_allowed());
}

/// A leg the follower steers itself (a course leg, a hold, an arc) takes
/// the lateral back from LNAV, silently.
#[test]
fn a_leg_lnav_does_not_fly_is_handed_back() {
    let (mut l, t0) = captured_on_bln();
    let mut t = tick(
        t0 + 1.0,
        4,
        "CF13",
        None,
        Some("CF13"),
        Some(true),
        Some(0.0),
    );
    t.eligible = false;
    assert_eq!(l.decide(t).act, Act::HandBack("leg_not_lnav"));
    assert_eq!(l.fallbacks(), 0);
    assert!(l.bug_write_allowed());
}

/// Three fallbacks in a run retire it for the run.
#[test]
fn three_fallbacks_retire_lnav_for_the_run() {
    let mut l = LnavLateral::default();
    let mut t = 0.0;
    for idx in 0..MAX_FALLBACKS as usize {
        l.decide(tick(
            t,
            idx,
            "BLN",
            None,
            Some("BLN"),
            Some(false),
            Some(0.0),
        ));
        l.decide(tick(
            t + 2.0,
            idx,
            "BLN",
            None,
            Some("BLN"),
            Some(true),
            Some(0.0),
        ));
        l.decide(tick(
            t + 2.5,
            idx,
            "BLN",
            None,
            Some("BLN"),
            Some(true),
            Some(0.0),
        ));
        let d = l.decide(tick(
            t + 3.0,
            idx,
            "BLN",
            None,
            Some("BLN"),
            Some(true),
            Some(3.0),
        ));
        assert!(matches!(d.act, Act::Drop(_)), "{d:?}");
        t += 10.0;
    }
    assert!(l.retired());
    let d = l.decide(tick(t, 9, "BLN", None, Some("BLN"), Some(false), Some(0.0)));
    assert_eq!(d.act, Act::Nothing);
}

/// One writer: whenever LNAV is the lateral owner the follower writes no
/// heading bug and presses no HDG SEL, through a whole route's events.
#[test]
fn the_follower_never_writes_the_bug_while_lnav_owns_the_lateral() {
    let mut l = LnavLateral::default();
    let idents = ["BLN", "NENEM", "YURZI", "ERIGA", "ARTIV", "REVTU"];
    let mut t = 0.0;
    let mut owned_ticks = 0;
    for (idx, id) in idents.iter().enumerate() {
        let next = idents.get(idx + 1).copied();
        for s in 0..240 {
            t += 0.5;
            // A lamp flicker, a stray FMS ident and a cross-track excursion
            // on the way, to walk every state.
            let engaged = Some(!(idx == 2 && (20..30).contains(&s)));
            let fms = if idx == 4 && s > 100 { "PIMOS" } else { id };
            let x = if idx == 1 && s > 200 {
                3.0
            } else {
                (4.0 - s as f64 * 0.05).max(0.0)
            };
            let d = l.decide(tick(t, idx, id, next, Some(fms), engaged, Some(x)));
            if l.owns_lateral() {
                owned_ticks += 1;
                assert!(!l.bug_write_allowed(), "bug written under LNAV at {t}");
                assert!(!l.heading_mode_wanted(t), "HDG SEL over LNAV at {t}");
                assert!(!matches!(d.act, Act::Drop(_) | Act::HandBack(_)));
            }
        }
    }
    assert!(owned_ticks > 500, "{owned_ticks}");
}

/// The follower's tick is gated as the decision says: its heading bug and
/// HDG SEL only when LNAV does not own the lateral, and a stalled leg drops
/// LNAV before the bug is written.
#[test]
fn the_followers_lateral_writes_are_gated_on_lnav() {
    let prod = crate::test_support::module_live_source("procedure");
    let lines: Vec<&str> = prod.lines().collect();
    let near = |needle: &str, guard: &str| {
        let i = lines
            .iter()
            .position(|l| l.contains(needle))
            .unwrap_or_else(|| panic!("{needle} not found"));
        let from = i.saturating_sub(4);
        assert!(
            lines[from..=i].iter().any(|l| l.contains(guard)),
            "`{needle}` is not gated by `{guard}`"
        );
    };
    near("heading_hold.decide(", "bug_write_allowed()");
    near("\"procedure_lateral\"", "heading_mode_wanted(");
    let stalled = prod.find("\"event\": \"leg_stalled\"").unwrap();
    let after = &prod[stalled..];
    let drop = after.find("lnav.drop_now(").expect("a stall drops LNAV");
    let bug = after.find("Dial::HeadingBug").unwrap();
    assert!(
        drop < bug,
        "LNAV is dropped before the stall writes the bug"
    );
}

#[test]
fn the_bug_says_why_it_flies_instead_of_lnav() {
    let tf = leg("TF", "KKW07");
    let base = BugWhy {
        owned: false,
        declared: true,
        enroute: true,
        retired: false,
        steering: None,
        leg: &tf,
        fms_ident: Some("KKW07"),
    };
    let why = |w: BugWhy| bug_reason(&w);
    assert_eq!(why(BugWhy { owned: true, ..base }), None);
    assert_eq!(why(BugWhy { declared: false, ..base }), Some("lnav_not_declared"));
    assert_eq!(why(BugWhy { enroute: false, ..base }), Some("not_enroute"));
    assert_eq!(why(BugWhy { retired: true, ..base }), Some("retired"));
    assert_eq!(why(BugWhy { steering: Some("holding"), ..base }), Some("holding"));
    let rf = leg("RF", "KKW07");
    assert_eq!(why(BugWhy { leg: &rf, ..base }), Some("leg_not_flyable"));
    assert_eq!(why(BugWhy { fms_ident: None, ..base }), Some("fms_unread"));
    assert_eq!(why(BugWhy { fms_ident: Some(""), ..base }), Some("fms_empty"));
    assert_eq!(why(BugWhy { fms_ident: Some("MID"), ..base }), Some("fms_not_on_leg"));
    assert_eq!(why(base), Some("engaging"));
}

/// The EGPH resume (737 FL380, 2026-10-16): the FMS on INPIP but the
/// aircraft about 13 nm off its RIBEL-INPIP line, where LNAV only arms. A
/// DIR TO first, once for the leg; then the press as ever.
#[test]
fn far_off_the_fms_leg_a_dir_to_comes_first_once() {
    let at = |l: &mut LnavLateral, t: f64, engaged: bool, xtk: Option<f64>| {
        l.decide(tick(
            t,
            0,
            "INPIP",
            Some("INREV"),
            Some("INPIP"),
            Some(engaged),
            xtk,
        ))
        .act
    };
    let mut l = LnavLateral::default();
    assert_eq!(at(&mut l, 0.0, false, Some(-13.0)), Act::DirectTo);
    assert!(l.bug_write_allowed(), "the bug flies through the DIR TO");
    // Done: the leg starts here, LNAV is pressed in.
    assert_eq!(at(&mut l, 4.0, false, Some(0.0)), Act::Press);
    assert_eq!(at(&mut l, 6.0, true, Some(0.0)), Act::Nothing);
    assert!(l.owns_lateral());

    // A DIR TO that did not take is never retried on the leg: the press,
    // as before.
    let mut l = LnavLateral::default();
    assert_eq!(at(&mut l, 0.0, false, Some(13.0)), Act::DirectTo);
    assert_eq!(at(&mut l, 4.0, false, Some(13.0)), Act::Press);

    // Near the line, an intercept: no DIR TO. Unread: no measurement.
    let mut l = LnavLateral::default();
    assert_eq!(at(&mut l, 0.0, false, Some(4.0)), Act::Press);
    let mut l = LnavLateral::default();
    assert_eq!(at(&mut l, 0.0, false, None), Act::Press);
}

/// The FMS not on the leg (a fix it does not hold, the synthetic gateway):
/// never a DIR TO, never a press; the goto and the bug as before.
#[test]
fn a_leg_the_fms_is_not_on_never_gets_a_dir_to() {
    let mut l = LnavLateral::default();
    for s in 0..40 {
        let t = s as f64 * 0.5;
        let d = l.decide(tick(
            t,
            0,
            "EGPH arrival",
            None,
            Some("D257B"),
            Some(false),
            Some(250.0),
        ));
        assert!(!matches!(d.act, Act::DirectTo | Act::Press), "{d:?} at {t}");
    }
}
