//! The safe hold's pure decisions.

use super::*;

/// A 737 at 2,930 ft (true 3,000), 180 kt, on track 306, over ground at 70 ft.
fn air() -> Air {
    Air {
        lat: 36.735,
        lon: -4.727,
        alt_ft: 2_930.0,
        alt_true_ft: 3_000.0,
        agl_ft: 2_930.0,
        vs_fpm: -330.0,
        ias_kts: 176.0,
        gs_kts: 190.0,
        track_true: 306.0,
        mag_var: Some(-1.0),
        bank_deg: 0.0,
        pitch_deg: 1.0,
        flap: 0.5,
    }
}

/// HDG + VS + ALT armed, servos in, the window at 0, VS -330: what the
/// restarted xvl found at 12:42:29.
fn ap_vs_down() -> ApRead {
    ApRead {
        servos: Some(true),
        state: Some(Modes(Modes::HEADING | Modes::VS | Modes::ALT_ARMED)),
        alt_dial_ft: Some(0.0),
        vs_dial_fpm: Some(-330.0),
        hdg_bug_mag: Some(305.0),
    }
}

fn ray(brg: f64, ground: &[(f64, f64)]) -> Ray {
    Ray {
        bearing_true: brg,
        ground_ft: ground.to_vec(),
    }
}

/// Ground rising along the track to 3,800 ft at 8 nm.
fn rising() -> Ray {
    ray(
        306.0,
        &[
            (1.0, 300.0),
            (3.0, 900.0),
            (5.0, 2_000.0),
            (8.0, 3_800.0),
            (12.0, 3_000.0),
            (15.0, 2_500.0),
        ],
    )
}

#[test]
fn the_autopilot_s_vertical_path_is_read_from_its_modes() {
    let a = air();
    assert_eq!(vertical(&ap_vs_down(), &a), Vertical::Captures(0.0));
    // The window above while descending is no floor.
    let mut ap = ap_vs_down();
    ap.alt_dial_ft = Some(5_600.0);
    assert_eq!(vertical(&ap, &a), Vertical::Unbounded);
    // VS without the altitude armed never captures.
    let mut ap = ap_vs_down();
    ap.state = Some(Modes(Modes::HEADING | Modes::VS));
    assert_eq!(vertical(&ap, &a), Vertical::Unbounded);
    // ALT HOLD holds here; a glideslope is an approach.
    let mut ap = ap_vs_down();
    ap.state = Some(Modes(Modes::HEADING | Modes::ALT_HOLD));
    assert_eq!(vertical(&ap, &a), Vertical::Holds(2_930.0));
    ap.state = Some(Modes(Modes::LOC_ARMED | Modes::GS));
    assert_eq!(vertical(&ap, &a), Vertical::Glideslope);
    // Level change to a window below captures it; a VS climb climbs.
    let mut ap = ap_vs_down();
    ap.state = Some(Modes(Modes::LEVEL_CHANGE));
    ap.alt_dial_ft = Some(2_000.0);
    assert_eq!(vertical(&ap, &a), Vertical::Captures(2_000.0));
    let mut ap = ap_vs_down();
    ap.vs_dial_fpm = Some(1_400.0);
    assert_eq!(vertical(&ap, &a), Vertical::Climbs);
    // No vertical mode: the aircraft's own vertical speed decides.
    let mut ap = ap_vs_down();
    ap.state = Some(Modes(Modes::HEADING));
    let mut fast_down = a;
    fast_down.vs_fpm = -900.0;
    assert_eq!(vertical(&ap, &fast_down), Vertical::Unbounded);
    let mut level = a;
    level.vs_fpm = -100.0;
    assert_eq!(vertical(&ap, &level), Vertical::Holds(2_930.0));
}

/// The bitfield is the panel owner's decoding: a -1 read (a sim with no
/// autopilot) is no state, never every mode at once (a glideslope "coupled"
/// would excuse any descent), and a float a hair under a bit rounds to it.
#[test]
fn a_negative_mode_read_is_no_state_never_every_bit() {
    let mut ap = ap_vs_down();
    ap.state = Modes::from_read(-1.0);
    assert_eq!(ap.state, None);
    let mut fast_down = air();
    fast_down.vs_fpm = -900.0;
    assert_eq!(vertical(&ap, &fast_down), Vertical::Unbounded);
    ap.state = Modes::from_read(2_047.999);
    assert_eq!(vertical(&ap, &air()), Vertical::Glideslope);
}

#[test]
fn the_floor_is_the_ground_ahead_then_the_grid_then_the_ground_under() {
    let a = air();
    let t = Terrain {
        ahead: Some(rising()),
        ..Default::default()
    };
    assert_eq!(
        floor(&a, &t),
        Floor {
            true_ft: 4_800.0,
            source: "terrain"
        }
    );
    let t = Terrain {
        ahead: None,
        fan: vec![],
        mora_ft: Some(6_100.0),
    };
    assert_eq!(
        floor(&a, &t),
        Floor {
            true_ft: 6_100.0,
            source: "mora"
        }
    );
    let t = Terrain::default();
    assert_eq!(
        floor(&a, &t),
        Floor {
            true_ft: 1_070.0,
            source: "under"
        }
    );
    // The altimeter's floor: QNH puts the aircraft 70 ft under its true height.
    assert_eq!(a.indicated(4_800.0), 4_730.0);
}

#[test]
fn a_trajectory_is_unsafe_only_when_it_ends_below_the_floor() {
    let a = air();
    let r = rising();
    assert_eq!(
        unsafe_why(Vertical::Captures(0.0), &a, 4_730.0, Some(&r)),
        Some(Why::CapturesBelow)
    );
    assert_eq!(
        unsafe_why(Vertical::Unbounded, &a, 4_730.0, Some(&r)),
        Some(Why::NoFloor)
    );
    assert_eq!(
        unsafe_why(Vertical::Holds(2_930.0), &a, 4_730.0, Some(&r)),
        Some(Why::HoldsBelow)
    );
    assert_eq!(
        unsafe_why(Vertical::Holds(5_000.0), &a, 4_730.0, Some(&r)),
        None
    );
    assert_eq!(
        unsafe_why(Vertical::Captures(4_700.0), &a, 4_730.0, Some(&r)),
        None,
        "inside the detent"
    );
    assert_eq!(
        unsafe_why(Vertical::Glideslope, &a, 4_730.0, Some(&r)),
        None
    );
    // A climb that clears the ridge is safe; a shallow one is not.
    let mut climbing = a;
    climbing.vs_fpm = 2_000.0;
    assert_eq!(
        unsafe_why(Vertical::Climbs, &climbing, 4_730.0, Some(&r)),
        None
    );
    climbing.vs_fpm = 300.0;
    assert_eq!(
        unsafe_why(Vertical::Climbs, &climbing, 4_730.0, Some(&r)),
        Some(Why::ClimbShort)
    );
}

#[test]
fn an_aligned_descent_near_a_runway_is_an_approach_and_nothing_else_is() {
    // A runway 13 threshold, course 130 true, elevation 50 ft.
    let thr = (36.68, -4.50);
    let end = End {
        ident: "LEMG 13".into(),
        thr_lat: thr.0,
        thr_lon: thr.1,
        course_true: 130.0,
        elev_ft: 50.0,
        length_m: 3_200.0,
    };
    let (lat, lon) = crate::nav::project_latlon(thr.0, thr.1, 310.0, 5.0);
    let mut a = air();
    a.lat = lat;
    a.lon = lon;
    a.track_true = 131.0;
    a.alt_true_ft = 1_650.0;
    a.vs_fpm = -800.0;
    let ends = [end.clone()];
    assert_eq!(on_approach(&ends, &a).as_deref(), Some("LEMG 13"));
    // Heading away from it, far above the path, or 3 nm off the line: no.
    let mut away = a;
    away.track_true = 306.0;
    assert_eq!(on_approach(&ends, &away), None);
    let mut high = a;
    high.alt_true_ft = 5_000.0;
    assert_eq!(on_approach(&ends, &high), None);
    let (la, lo) = crate::nav::project_latlon(lat, lon, 40.0, 3.0);
    let mut off = a;
    off.lat = la;
    off.lon = lo;
    assert_eq!(on_approach(&ends, &off), None);
    // Over the pavement in the flare is still the approach.
    let (la, lo) = crate::nav::project_latlon(thr.0, thr.1, 130.0, 0.3);
    let mut flare = a;
    flare.lat = la;
    flare.lon = lo;
    flare.alt_true_ft = 70.0;
    assert_eq!(on_approach(&ends, &flare).as_deref(), Some("LEMG 13"));
}

#[test]
fn the_escape_is_the_ground_inside_a_minute_on_the_path_flown() {
    // 12:44:49: 2,610 ft, 450 ft over the ground, -1,526 fpm at 138 kt.
    let mut a = air();
    a.alt_true_ft = 2_610.0;
    a.agl_ft = 452.0;
    a.vs_fpm = -1_526.0;
    a.gs_kts = 140.0;
    assert!(escape_due(&a, None, false), "under 500 ft and sinking");
    // Level at 3,000 toward a 3,800 ft ridge a mile ahead.
    let mut b = air();
    b.vs_fpm = 0.0;
    let ridge = ray(306.0, &[(1.0, 2_800.0)]);
    assert!(escape_due(&b, Some(&ridge), false));
    // The same ridge 8 nm out is the hold's business, not the escape's.
    let far = ray(306.0, &[(8.0, 3_800.0)]);
    assert!(!escape_due(&b, Some(&far), false));
    // Climbing away over it.
    b.vs_fpm = 2_500.0;
    assert!(!escape_due(&b, Some(&ray(306.0, &[(1.0, 2_600.0)])), false));
}

#[test]
fn the_clear_heading_is_the_nearest_one_the_climb_clears() {
    let a = air();
    let grad = gradient_ft_nm(1_500.0, a.gs_kts);
    // Straight ahead a 6,000 ft wall at 3 nm; 30 right the same; 60 left low.
    let fan = vec![
        ray(306.0, &[(2.0, 1_000.0), (4.0, 6_000.0)]),
        ray(336.0, &[(2.0, 1_000.0), (4.0, 6_000.0)]),
        ray(246.0, &[(2.0, 300.0), (4.0, 800.0), (8.0, 1_200.0)]),
        ray(126.0, &[(2.0, 0.0), (4.0, 0.0)]),
    ];
    let (brg, need) = clear_heading(&fan, &a, 4_800.0, grad).unwrap();
    assert_eq!(brg, 246.0, "the clear ray nearest the track");
    assert_eq!(need, 2_200.0);
    // Everything clear: no turn.
    let open = vec![ray(306.0, &[(4.0, 500.0)]), ray(246.0, &[(4.0, 0.0)])];
    assert_eq!(clear_heading(&open, &a, 4_800.0, grad).unwrap().0, 306.0);
    assert_eq!(clear_heading(&[], &a, 4_800.0, grad), None);
}

#[test]
fn the_hold_climbs_to_the_floor_or_bounds_the_descent_there() {
    let a = air();
    let t = Terrain {
        ahead: Some(rising()),
        ..Default::default()
    };
    let fl = floor(&a, &t);
    let c = hold_cmd(Why::CapturesBelow, &a, &ap_vs_down(), fl, &t, 2_000.0);
    assert!(c.climb);
    assert_eq!(c.alt_ft, 4_800.0, "up_100 of 4,730 indicated");
    assert_eq!(c.fpm, 2_000.0);
    assert_eq!(
        c.heading_mag, None,
        "the climb clears the ridge on this track"
    );
    // High above it, a descent with no window is bounded at the floor.
    let mut high = a;
    high.alt_ft = 9_000.0;
    high.alt_true_ft = 9_070.0;
    high.agl_ft = 9_000.0;
    let fl = floor(&high, &t);
    let c = hold_cmd(Why::NoFloor, &high, &ap_vs_down(), fl, &t, 2_000.0);
    assert!(!c.climb);
    assert_eq!(c.alt_ft, 4_800.0);
    assert_eq!(c.fpm, 330.0, "the descent's own rate");
    // A ridge the airframe cannot out-climb on this track turns the hold.
    let wall = ray(306.0, &[(2.0, 2_000.0), (3.0, 6_000.0)]);
    let t = Terrain {
        ahead: Some(wall.clone()),
        fan: vec![wall, ray(246.0, &[(2.0, 300.0), (6.0, 900.0)])],
        mora_ft: None,
    };
    let fl = floor(&a, &t);
    let c = hold_cmd(Why::HoldsBelow, &a, &ap_vs_down(), fl, &t, 700.0);
    assert_eq!(
        c.heading_mag,
        Some(245.0),
        "true 246 with 1 degree east: mag 245"
    );
    assert!(c.climb);
}

#[test]
fn only_a_hand_on_the_autopilot_is_the_pilot_never_its_own_capture() {
    let base = ap_vs_down();
    let alt = 2_930.0;
    assert_eq!(mcp_moved(&base, &base, false, alt, &Ours::default()), None);
    let mut now = base;
    now.alt_dial_ft = Some(5_000.0);
    assert_eq!(mcp_moved(&base, &now, false, alt, &Ours::default()), Some("window"));
    let mut now = base;
    now.servos = Some(false);
    assert_eq!(mcp_moved(&base, &now, false, alt, &Ours::default()), Some("servos"));
    // 12:43:41: the VS wheel to 0, nothing of ours flying.
    let mut now = base;
    now.vs_dial_fpm = Some(0.0);
    assert_eq!(mcp_moved(&base, &now, false, alt, &Ours::default()), Some("vs_wheel"));
    // ...while our own hold's speed protection moves it: ours.
    assert_eq!(mcp_moved(&base, &now, true, alt, &Ours::default()), None);
    // The capture: VS out, ALT HOLD in, at the window.
    let mut cap = base;
    cap.alt_dial_ft = Some(2_900.0);
    let mut held = cap;
    held.state = Some(Modes(Modes::HEADING | Modes::ALT_HOLD));
    assert_eq!(mcp_moved(&cap, &held, false, 2_910.0, &Ours::default()), None);
    // ALT HOLD pressed a mile from the window: a hand.
    let mut pressed = base;
    pressed.state = Some(Modes(Modes::HEADING | Modes::ALT_HOLD));
    assert_eq!(mcp_moved(&base, &pressed, false, alt, &Ours::default()), Some("alt_hold"));
    // LVL CHG, or the autothrottle: a hand, even under our hold.
    let mut flch = base;
    flch.state = Some(Modes(Modes::HEADING | Modes::LEVEL_CHANGE));
    assert_eq!(mcp_moved(&base, &flch, true, alt, &Ours::default()), Some("mode"));
    let mut at = base;
    at.state = Some(Modes(Modes::HEADING | Modes::VS | Modes::ALT_ARMED | Modes::AUTOTHROTTLE));
    assert_eq!(mcp_moved(&base, &at, true, alt, &Ours::default()), Some("mode"));
    // The heading bug in HDG mode.
    let mut hdg = base;
    hdg.hdg_bug_mag = Some(250.0);
    assert_eq!(mcp_moved(&base, &hdg, false, alt, &Ours::default()), Some("heading"));
}

/// The panel owner's ledger tells ours from the pilot's: a window, a bug, a
/// wheel or a mode OUR writes and presses explain is never the pilot; the
/// same move with nothing of ours behind it, or a mode our presses cannot
/// bring, still is.
#[test]
fn our_own_presses_and_writes_are_never_the_pilot() {
    use crate::autoflight::{Button, Dial};
    let base = ap_vs_down();
    let alt = 2_930.0;
    let ours = |c: Vec<OurChange>| Ours(c);
    let mut now = base;
    now.alt_dial_ft = Some(5_000.0);
    assert_eq!(mcp_moved(&base, &now, false, alt, &ours(vec![OurChange::Dial(Dial::Altitude, 5_000.0)])), None);
    assert_eq!(
        mcp_moved(&base, &now, false, alt, &ours(vec![OurChange::Dial(Dial::Altitude, 7_000.0)])),
        Some("window"),
        "a window of ours elsewhere does not explain this one"
    );
    let mut now = base;
    now.hdg_bug_mag = Some(250.0);
    assert_eq!(mcp_moved(&base, &now, false, alt, &ours(vec![OurChange::Dial(Dial::HeadingBug, 250.0)])), None);
    let mut now = base;
    now.vs_dial_fpm = Some(1_500.0);
    assert_eq!(mcp_moved(&base, &now, false, alt, &ours(vec![OurChange::Dial(Dial::Vs, 1_500.0)])), None);
    let mut now = base;
    now.servos = Some(false);
    assert_eq!(mcp_moved(&base, &now, false, alt, &ours(vec![OurChange::Press(Button::Disconnect)])), None);
    // Our ALT HOLD press away from the window: ours. LVL CHG: never ours.
    let mut now = base;
    now.state = Some(Modes(Modes::HEADING | Modes::ALT_HOLD));
    assert_eq!(mcp_moved(&base, &now, false, alt, &ours(vec![OurChange::Press(Button::AltitudeHold)])), None);
    let mut now = base;
    now.state = Some(Modes(Modes::HEADING | Modes::LEVEL_CHANGE));
    assert_eq!(
        mcp_moved(&base, &now, true, alt, &ours(vec![OurChange::Press(Button::AltitudeHold)])),
        Some("mode")
    );
}

/// The tick's ledger is filtered to the episode: a change of ours from
/// before the reference (less the slack) explains nothing now.
#[test]
fn only_our_changes_since_the_reference_count() {
    use crate::autoflight::Dial;
    let mut h = SafeHold::default();
    let air = Air { alt_ft: 2_930.0, alt_true_ft: 2_930.0, agl_ft: 2_000.0, ias_kts: 200.0, gs_kts: 200.0, ..Air::default() };
    let base = ap_vs_down();
    h.step(&tick(100.0, air, base));
    let mut now = base;
    now.alt_dial_ft = Some(5_000.0);
    let mut t = tick(101.0, air, now);
    t.ours = vec![(20.0, OurChange::Dial(Dial::Altitude, 5_000.0))];
    h.step(&t);
    assert_eq!(h.pilot(), Some("window"), "an old write of ours is not this one");
    let mut h = SafeHold::default();
    h.step(&tick(100.0, air, base));
    let mut t = tick(101.0, air, now);
    t.ours = vec![(100.5, OurChange::Dial(Dial::Altitude, 5_000.0))];
    h.step(&t);
    assert_eq!(h.pilot(), None);
}

fn tick(now_s: f64, air: Air, ap: ApRead) -> Tick {
    Tick {
        now_s,
        airborne: true,
        frozen: false,
        owner: None,
        pilot_words: false,
        hands: false,
        ap,
        air,
        terrain: Terrain {
            ahead: Some(rising()),
            ..Default::default()
        },
        approach: None,
        terminal: None,
        unflyable: None,
        capability_fpm: 2_000.0,
        actuate: true,
        ours: vec![],
    }
}

fn events(recs: &[Value]) -> Vec<String> {
    recs.iter()
        .map(|r| r["event"].as_str().unwrap_or("").to_string())
        .collect()
}

#[test]
fn an_unattended_descent_below_the_floor_is_held_after_confirmation() {
    let mut h = SafeHold::default();
    let (a, r) = h.step(&tick(0.0, air(), ap_vs_down()));
    assert_eq!(a, Act::Nothing);
    assert_eq!(events(&r), ["episode", "verdict"]);
    assert_eq!(r[0]["cause"], "boot");
    assert_eq!(r[1]["verdict"], "unsafe");
    assert_eq!(h.step(&tick(1.0, air(), ap_vs_down())).0, Act::Nothing);
    let (a, r) = h.step(&tick(3.0, air(), ap_vs_down()));
    let Act::Hold(cmd) = a else { panic!("{a:?}") };
    assert_eq!(cmd.alt_ft, 4_800.0);
    assert_eq!(events(&r), ["hold"]);
    // The driver set it: read back, it is the reference now.
    let mut after = ap_vs_down();
    after.alt_dial_ft = Some(4_800.0);
    after.vs_dial_fpm = Some(2_000.0);
    h.written(cmd, 3.0, 2_930.0, Some(after));
    let mut climbing = air();
    climbing.vs_fpm = 1_800.0;
    climbing.alt_ft = 3_400.0;
    climbing.alt_true_ft = 3_470.0;
    assert_eq!(h.step(&tick(10.0, climbing, after)).0, Act::Nothing);
    // The pilot turns the window: theirs from now on, never re-asserted.
    let mut theirs = after;
    theirs.alt_dial_ft = Some(3_000.0);
    let (a, r) = h.step(&tick(12.0, climbing, theirs));
    assert_eq!(a, Act::Nothing);
    assert_eq!(events(&r), ["pilot"]);
    assert_eq!(h.pilot(), Some("window"));
    for t in 13..80 {
        assert!(!matches!(
            h.step(&tick(t as f64, climbing, theirs)).0,
            Act::Hold(_)
        ));
    }
}

#[test]
fn a_hold_the_autopilot_does_not_fly_is_taken_by_the_recovery() {
    let mut h = SafeHold::default();
    for t in [0.0, 1.0, 3.0] {
        if let (Act::Hold(cmd), _) = h.step(&tick(t, air(), ap_vs_down())) {
            h.written(cmd, t, 2_930.0, Some(ap_vs_down()));
        }
    }
    assert!(h.held().is_some());
    // Still sinking 25 s later.
    let mut sinking = air();
    sinking.alt_ft = 2_800.0;
    sinking.alt_true_ft = 2_870.0;
    let (a, r) = h.step(&tick(30.0, sinking, ap_vs_down()));
    assert_eq!(
        a,
        Act::TakeOver {
            alt_ft: Some(4_800.0),
            why: "hold_not_climbing"
        }
    );
    assert_eq!(events(&r), ["take_over"]);
    assert_eq!(h.step(&tick(31.0, sinking, ap_vs_down())).0, Act::Nothing);
}

#[test]
fn a_crew_loop_or_the_ground_ends_the_episode() {
    let mut h = SafeHold::default();
    h.step(&tick(0.0, air(), ap_vs_down()));
    let mut owned = tick(1.0, air(), ap_vs_down());
    owned.owner = Some("full auto");
    let (a, r) = h.step(&owned);
    assert_eq!(a, Act::End("crew"));
    assert_eq!(events(&r), ["end"]);
    assert_eq!(h.step(&owned).0, Act::Nothing);
    // The next episode is a release, and a pilot's words name it.
    let mut t = tick(2.0, air(), ap_vs_down());
    t.pilot_words = true;
    let (_, r) = h.step(&t);
    assert_eq!(r[0]["cause"], "pilot_release");
    let mut ground = tick(3.0, air(), ap_vs_down());
    ground.airborne = false;
    assert_eq!(h.step(&ground).0, Act::End("ground"));
    // A pause judges nothing and records nothing.
    let mut frozen = tick(4.0, air(), ap_vs_down());
    frozen.frozen = true;
    assert_eq!(h.step(&frozen), (Act::Nothing, vec![]));
}

#[test]
fn a_pilot_hand_flying_hears_terrain_and_is_never_overruled() {
    let mut low = air();
    low.alt_true_ft = 2_610.0;
    low.agl_ft = 452.0;
    low.vs_fpm = -1_526.0;
    let mut off = ap_vs_down();
    off.servos = Some(false);
    let mut h = SafeHold::default();
    let (a, _) = h.step(&tick(0.0, low, off));
    assert_eq!(a, Act::Warn(TERRAIN_PULL_UP.into()));
    assert_eq!(
        h.step(&tick(5.0, low, off)).0,
        Act::Nothing,
        "not every tick"
    );
    assert_eq!(
        h.step(&tick(25.0, low, off)).0,
        Act::Warn(TERRAIN_PULL_UP.into())
    );
    // Hands on the yoke over a lit autopilot: words only, too.
    let mut h = SafeHold::default();
    let mut t = tick(0.0, low, ap_vs_down());
    t.hands = true;
    assert_eq!(h.step(&t).0, Act::Warn(TERRAIN_PULL_UP.into()));
    t.now_s = 10.0;
    assert!(!matches!(h.step(&t).0, Act::Hold(_) | Act::Escape(_)));
}

#[test]
fn a_pilot_flying_the_autopilot_is_told_once_and_escaped_only_at_the_ground() {
    let mut h = SafeHold::default();
    h.step(&tick(0.0, air(), ap_vs_down()));
    let mut theirs = ap_vs_down();
    theirs.vs_dial_fpm = Some(-800.0);
    let (a, _) = h.step(&tick(1.0, air(), theirs));
    assert!(
        matches!(a, Act::Warn(ref l) if l.contains("safe altitude")),
        "{a:?}"
    );
    for t in 2..10 {
        assert_eq!(h.step(&tick(t as f64, air(), theirs)).0, Act::Nothing);
    }
    let mut low = air();
    low.agl_ft = 400.0;
    low.vs_fpm = -800.0;
    assert!(matches!(h.step(&tick(11.0, low, theirs)).0, Act::Escape(_)));
}

#[test]
fn an_approach_is_never_held_and_without_hands_to_act_the_crew_only_speaks() {
    let mut h = SafeHold::default();
    for s in 0..10 {
        let mut t = tick(s as f64, air(), ap_vs_down());
        t.approach = Some("LEMG 13".into());
        assert_eq!(h.step(&t).0, Act::Nothing);
    }
    let mut h = SafeHold::default();
    let mut said = Vec::new();
    for s in 0..10 {
        let mut t = tick(s as f64, air(), ap_vs_down());
        t.actuate = false;
        let (a, _) = h.step(&t);
        assert!(!matches!(
            a,
            Act::Hold(_) | Act::Escape(_) | Act::TakeOver { .. }
        ));
        if let Act::Warn(l) = a {
            said.push(l);
        }
    }
    assert_eq!(said.len(), 1, "{said:?}");
}

#[test]
fn an_unattended_upset_is_recovered_never_handed_to_nobody() {
    let mut h = SafeHold::default();
    let mut t = tick(0.0, air(), ap_vs_down());
    t.unflyable = Some("stall");
    assert_eq!(
        h.step(&t).0,
        Act::Nothing,
        "a tick of stall warning is a gust"
    );
    t.now_s = 2.5;
    assert!(matches!(h.step(&t).0, Act::TakeOver { why: "stall", .. }));
}

#[test]
fn the_lines_name_the_number_and_carry_no_dashes() {
    let a = air();
    let t = Terrain {
        ahead: Some(rising()),
        ..Default::default()
    };
    let c = hold_cmd(
        Why::CapturesBelow,
        &a,
        &ap_vs_down(),
        floor(&a, &t),
        &t,
        2_000.0,
    );
    for line in [
        hold_line(&c, false, "boot"),
        hold_line(&c, true, "release"),
        below_floor_line(4_800.0),
        take_over_line(Some(4_800.0)),
    ] {
        assert!(line.contains("4,800") || line.contains("4800"), "{line}");
        assert!(
            !line.contains('\u{2014}') && !line.contains('\u{2013}'),
            "{line}"
        );
    }
    assert!(hold_line(&c, false, "boot").starts_with("Nobody was flying"));
}

#[test]
fn the_fan_answer_splits_back_into_its_rays() {
    let rays = vec![
        (10.0, vec![(1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]),
        (40.0, vec![(1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]),
    ];
    let split = split_fan(&rays, &[1.0, 2.0, 3.0, 4.0]);
    assert_eq!(split.len(), 2);
    assert_eq!(split[1].ground_ft, vec![(1.0, 3.0), (2.0, 4.0)]);
    assert_eq!(
        split_fan(&rays, &[1.0]).len(),
        0,
        "a short answer is no ray"
    );
}

#[test]
fn near_a_field_only_a_collision_course_is_an_escape() {
    let end = End {
        ident: "LEMG 13".into(),
        thr_lat: 36.68,
        thr_lon: -4.50,
        course_true: 130.0,
        elev_ft: 50.0,
        length_m: 3_200.0,
    };
    // A base turn at 600 ft, 3 nm out, sinking 700: the circuit.
    let (lat, lon) = crate::nav::project_latlon(36.68, -4.50, 40.0, 3.0);
    let mut base = air();
    base.lat = lat;
    base.lon = lon;
    base.alt_true_ft = 650.0;
    base.agl_ft = 600.0;
    base.vs_fpm = -700.0;
    base.gs_kts = 120.0;
    let ends = [end];
    assert_eq!(in_terminal(&ends, &base).as_deref(), Some("LEMG 13"));
    let flat = ray(40.0, &[(1.0, 50.0), (2.0, 60.0)]);
    assert!(
        escape_due(&base, Some(&flat), false),
        "away from a field it would be"
    );
    assert!(!escape_due(&base, Some(&flat), true));
    // A hill on the nose is still a hill.
    let hill = ray(40.0, &[(0.5, 600.0)]);
    assert!(escape_due(&base, Some(&hill), true));
    // High over the field is no circuit.
    let mut high = base;
    high.alt_true_ft = 3_000.0;
    assert_eq!(in_terminal(&ends, &high), None);
}

#[test]
fn a_coupled_glideslope_is_an_approach_without_runway_data() {
    let mut low = air();
    low.agl_ft = 300.0;
    low.vs_fpm = -800.0;
    let mut gs = ap_vs_down();
    gs.state = Some(Modes(Modes::AUTOTHROTTLE | Modes::GS | 512));
    let mut h = SafeHold::default();
    for s in 0..6 {
        let (a, r) = h.step(&tick(s as f64, low, gs));
        assert_eq!(a, Act::Nothing);
        if s == 0 {
            assert_eq!(r[1]["verdict"], "approach");
        }
    }
}

#[test]
fn a_reading_no_flight_has_is_not_judged() {
    let mut load = air();
    load.alt_ft = 0.0;
    load.agl_ft = -87.0;
    let mut h = SafeHold::default();
    assert_eq!(
        h.step(&tick(0.0, load, ap_vs_down())),
        (Act::Nothing, vec![])
    );
}

/// An escape never continues a descent: above its floor it levels at once
/// where it is (LEMD 2026-10-05 17:07: diving through 4,015 ft, floor 3,132,
/// the old escape set 3,100 at -1,000 fpm).
#[test]
fn an_escape_above_its_floor_levels_where_it_is() {
    let descent = HoldCmd {
        alt_ft: 3_100.0,
        fpm: 1_000.0,
        heading_mag: None,
        climb: false,
        why: Why::HoldsBelow,
        floor: Floor { true_ft: 3_132.0, source: "terrain" },
    };
    let e = escape_cmd(descent, 4_015.0);
    assert!(e.climb, "an escape climbs or levels, never descends");
    assert_eq!(e.alt_ft, 4_100.0);
    assert_eq!(e.fpm, ESCAPE_LEVEL_FPM);
    // Below the floor the plain hold already climbs: unchanged.
    let climb = HoldCmd { climb: true, alt_ft: 3_200.0, fpm: 2_000.0, ..descent };
    assert_eq!(escape_cmd(climb, 2_500.0), climb);
}

/// The aircraft is read as ONE frame (#157): position, heights, groundspeed
/// and vertical speed come from the plugin's snapshot, never stock reads
/// that may straddle two frames; the track rides beside it.
#[tokio::test]
async fn the_air_is_read_from_one_frame() {
    use crate::datarefs as d;
    let fake = crate::xplane::FakeXPlane::with_reads(&[
        (d::LATITUDE, 10.0),
        (d::LONGITUDE, 10.0),
        (d::ALTITUDE_FT, 2_930.0),
        (d::AIRSPEED_KTS, 176.0),
        (HPATH, 306.0),
        (MAG_VAR, -1.0),
    ]);
    let frame = sim::position::Position {
        lat: 36.735,
        lon: -4.727,
        elev_m: 914.4,
        psi_true_deg: 300.0,
        gs_ms: 100.0,
        vs_fpm: -330.0,
        agl_m: 893.0,
        frame: Some(7),
        sim_time_s: Some(1.0),
        source: sim::position::Source::Plugin,
    };
    fake.set_read_bytes(sim::position::POSITION, Some(&sim::position::encode(&frame)));
    let (air, hands, _) = read_air(&fake).await.expect("a whole reading");
    assert_eq!((air.lat, air.lon), (36.735, -4.727));
    assert!((air.alt_true_ft - 3_000.0).abs() < 0.1);
    assert_eq!(air.vs_fpm, -330.0);
    assert!((air.gs_kts - 194.4).abs() < 0.1);
    assert_eq!(air.track_true, 306.0);
    assert!(!hands);
}

/// No variation read, no turn: the bug is magnetic, and a "0" variation set
/// it off the clear bearing by the whole variation. The climb stays ahead.
#[test]
fn a_failed_variation_read_turns_nothing() {
    let mut a = air();
    a.mag_var = None;
    let wall = ray(306.0, &[(2.0, 2_000.0), (3.0, 6_000.0)]);
    let t = Terrain {
        ahead: Some(wall.clone()),
        fan: vec![wall, ray(246.0, &[(2.0, 300.0), (6.0, 900.0)])],
        ..Default::default()
    };
    let fl = floor(&a, &t);
    let c = hold_cmd(Why::HoldsBelow, &a, &ap_vs_down(), fl, &t, 700.0);
    assert_eq!(c.heading_mag, None);
    assert!(c.climb);
}
