use super::*;

fn refs172() -> Refs {
    Refs {
        vy_kts: Some(74.0),
        vr_kts: Some(55.0),
        taxi_kts: 15.0,
        approach_kts: Some(65.0),
        cruise_ias_kts: Some(108.0),
        va_kts: Some(99.0),
        slow_kts: Some(58.0),
        pattern_agl_ft: 1000.0,
        pattern_kts: Some(74.0),
    }
}

/// The instructor briefs and scores the approach autoland and the
/// circuit fly (`aircraft::approach_kts`): an airframe declaring none is
/// max(1.3 Vso, 1.2 Vs), not the 1.3 x CLEAN stall it still briefed
/// (refactor audit 2026-10-04, finding #2).
#[test]
fn the_brief_approach_is_the_aircrafts_rule() {
    let mut p = crate::aircraft::load("cirrus_sr22").unwrap();
    assert_eq!(Refs::from_profile(&p).approach_kts, Some(75.0), "declared wins");
    p.reference.as_mut().unwrap().approach_kts = None;
    // SR22: Vso 60, Vs 70: max(78, 84) = 84, not 1.3 x 70 = 91.
    assert_eq!(Refs::from_profile(&p).approach_kts, Some(84.0));
}

fn air(t: f64, alt: f64, hdg: f64, ias: f64) -> Sample {
    Sample {
        t_s: t,
        alt_ft: alt,
        agl_ft: alt - 500.0,
        hdg_deg: hdg,
        ias_kts: ias,
        bank_deg: 0.0,
        vsi_fpm: 0.0,
        on_ground: false,
        track_deg: Some(hdg),
        gs_kts: ias,
        engine_running: true,
        taxi: None,
    }
}

#[test]
fn the_brief_assigns_rounded_numbers_from_the_book() {
    let entry = air(0.0, 3470.0, 268.0, 104.0);
    let t = Task::StraightAndLevel.assign(&entry, &refs172(), Level::Private);
    assert_eq!(t.alt_ft, Some(3500.0));
    assert_eq!(t.hdg_deg, Some(270.0));
    assert_eq!(t.ias_kts, Some(108.0), "cruise IAS from the book");
    let c = Task::Climb.assign(&entry, &refs172(), Level::Private);
    assert_eq!(c.alt_ft, Some(4500.0), "next 500 at least 1,000 up");
    assert_eq!(c.ias_kts, Some(74.0), "Vy");
    let d = Task::Descent.assign(&entry, &refs172(), Level::Private);
    assert_eq!(
        d.alt_ft,
        Some(2000.0),
        "1,000 down to a 500, and ground plus 1,500 is 2,000 too"
    );
    assert_eq!(d.ias_kts, Some(105.0), "the entry speed, rounded");
    let brief = Task::Climb.brief(&c, Level::Private);
    assert!(brief.contains("4,500 feet"), "{brief}");
    assert!(brief.contains("74 knots"), "{brief}");
    assert!(brief.contains("heading 270"), "{brief}");
    assert!(brief.contains("plus 10 minus 5"), "{brief}");
    assert!(brief.ends_with("Say begin when you're ready."), "{brief}");
    assert!(!brief.contains('\u{2014}') && !brief.contains('\u{2013}'));
}

#[test]
fn a_descent_is_never_assigned_into_the_ground() {
    let mut entry = air(0.0, 2400.0, 90.0, 100.0);
    entry.agl_ft = 1600.0; // ground at 800
    let d = Task::Descent.assign(&entry, &refs172(), Level::Private);
    // 1,400 wanted, floored to ground + 1,500 = 2,300.
    assert_eq!(d.alt_ft, Some(2300.0));
}

#[test]
fn a_steady_two_minutes_passes_straight_and_level() {
    let t = Targets {
        alt_ft: Some(3500.0),
        hdg_deg: Some(270.0),
        ias_kts: Some(108.0),
        bank_deg: None,
        ..Targets::default()
    };
    let mut sc = Scorer::new(Task::StraightAndLevel, Level::Private, t);
    let mut lines = Vec::new();
    for i in 0..=125 {
        let s = air(
            i as f64,
            3500.0 + (i % 5) as f64 * 8.0,
            270.0 + ((i % 3) as f64 - 1.0) * 2.0,
            108.0 + (i % 2) as f64 * 3.0,
        );
        lines.extend(sc.tick(&s));
    }
    assert!(sc.complete());
    assert_eq!(
        lines,
        vec!["That's time.".to_string()],
        "no excursion narrated"
    );
    let g = sc.grade();
    assert!(g.pass, "{}", g.spoken());
    assert!(
        g.spoken().ends_with("That meets the standard."),
        "{}",
        g.spoken()
    );
}

#[test]
fn a_sustained_excursion_is_narrated_once_then_the_recovery() {
    let t = Targets {
        alt_ft: Some(3500.0),
        hdg_deg: Some(270.0),
        ias_kts: Some(108.0),
        bank_deg: None,
        ..Targets::default()
    };
    let mut sc = Scorer::new(Task::StraightAndLevel, Level::Private, t);
    let mut lines = Vec::new();
    for i in 0..40 {
        // 130 ft low from t=10 to t=25, then back.
        let alt = if (10..25).contains(&i) {
            3370.0
        } else {
            3500.0
        };
        lines.extend(sc.tick(&air(i as f64, alt, 270.0, 108.0)));
    }
    assert_eq!(
        lines,
        vec![
            "Altitude, 130 low. Nose up a touch.".to_string(),
            "Good, back on altitude.".to_string()
        ]
    );
    let g = sc.grade();
    let alt = g.axes.iter().find(|a| a.axis == Axis::Altitude).unwrap();
    assert!(!alt.pass, "15 s outside is not momentary");
    assert_eq!(alt.worst, -130.0);
    assert!(!g.pass);
    assert!(!g.complete, "the clock has not run");
    assert!(g.spoken().contains("Task not completed."));
}

#[test]
fn a_momentary_bump_is_forgiven_but_a_gross_one_is_not() {
    let t = Targets {
        alt_ft: Some(3500.0),
        hdg_deg: Some(270.0),
        ias_kts: Some(108.0),
        bank_deg: None,
        ..Targets::default()
    };
    let mut sc = Scorer::new(Task::StraightAndLevel, Level::Private, t);
    for i in 0..30 {
        let alt = if i == 10 || i == 11 { 3610.0 } else { 3500.0 };
        sc.tick(&air(i as f64, alt, 270.0, 108.0));
    }
    let alt = sc
        .grade()
        .axes
        .into_iter()
        .find(|a| a.axis == Axis::Altitude)
        .unwrap();
    assert!(alt.pass, "2 s outside, corrected promptly");
    let mut sc = Scorer::new(Task::StraightAndLevel, Level::Private, t);
    for i in 0..30 {
        let alt = if i == 10 { 3750.0 } else { 3500.0 };
        sc.tick(&air(i as f64, alt, 270.0, 108.0));
    }
    let alt = sc
        .grade()
        .axes
        .into_iter()
        .find(|a| a.axis == Axis::Altitude)
        .unwrap();
    assert!(!alt.pass, "250 ft is gross however brief");
}

#[test]
fn heading_deviation_wraps_through_north() {
    let t = Targets {
        alt_ft: Some(3500.0),
        hdg_deg: Some(5.0),
        ias_kts: Some(108.0),
        bank_deg: None,
        ..Targets::default()
    };
    let mut sc = Scorer::new(Task::StraightAndLevel, Level::Private, t);
    for i in 0..10 {
        sc.tick(&air(i as f64, 3500.0, 358.0, 108.0));
    }
    let h = sc
        .grade()
        .axes
        .into_iter()
        .find(|a| a.axis == Axis::Heading)
        .unwrap();
    assert_eq!(h.worst, -7.0, "358 is 7 left of 005, not 353 right");
    assert!(h.pass);
}

#[test]
fn a_climb_is_judged_on_speed_then_on_the_level_off() {
    let t = Targets {
        alt_ft: Some(4500.0),
        hdg_deg: Some(270.0),
        ias_kts: Some(74.0),
        bank_deg: None,
        ..Targets::default()
    };
    let mut sc = Scorer::new(Task::Climb, Level::Private, t);
    let mut lines = Vec::new();
    let mut alt = 3500.0;
    let mut t_s = 0.0;
    // 90 s at 700 fpm, 71 kt (3 slow: within +10/-5), then level at 4,460.
    while alt < 4460.0 {
        let mut s = air(t_s, alt, 270.0, 71.0);
        s.vsi_fpm = 700.0;
        lines.extend(sc.tick(&s));
        alt += 700.0 / 60.0;
        t_s += 1.0;
    }
    for _ in 0..12 {
        lines.extend(sc.tick(&air(t_s, 4460.0, 270.0, 90.0)));
        t_s += 1.0;
    }
    assert!(sc.complete(), "{lines:?}");
    assert!(
        lines.iter().any(|l| l == "Level. That completes the task."),
        "{lines:?}"
    );
    let g = sc.grade();
    assert!(g.pass, "{}", g.spoken());
    let ias = g.axes.iter().find(|a| a.axis == Axis::Airspeed).unwrap();
    assert!(
        ias.judged_s > 60.0 && ias.judged_s < 100.0,
        "speed judged in the climb only: {}",
        ias.judged_s
    );
    assert!(
        g.notes[0].starts_with("Levelled at 4,460 feet, 40 below"),
        "{:?}",
        g.notes
    );
}

#[test]
fn a_climb_levelled_elsewhere_names_the_assignment() {
    let t = Targets {
        alt_ft: Some(4500.0),
        hdg_deg: Some(270.0),
        ias_kts: Some(74.0),
        bank_deg: None,
        ..Targets::default()
    };
    let mut sc = Scorer::new(Task::Climb, Level::Private, t);
    let mut lines = Vec::new();
    for i in 0..10 {
        let mut s = air(i as f64, 3500.0 + i as f64 * 10.0, 270.0, 74.0);
        s.vsi_fpm = 600.0;
        lines.extend(sc.tick(&s));
    }
    for i in 10..40 {
        lines.extend(sc.tick(&air(i as f64, 4000.0, 270.0, 100.0)));
    }
    assert!(sc.complete());
    assert!(
        lines
            .iter()
            .any(|l| l.contains("levelled at 4,000 feet. The assignment was 4,500 feet")),
        "{lines:?}"
    );
    assert!(!sc.grade().pass);
}

#[test]
fn steep_turns_count_roll_outs_and_judge_the_bank() {
    let t = Targets {
        alt_ft: Some(3500.0),
        hdg_deg: Some(90.0),
        ias_kts: Some(95.0),
        bank_deg: Some(45.0),
        ..Targets::default()
    };
    let mut sc = Scorer::new(Task::SteepTurns, Level::Private, t);
    let mut lines = Vec::new();
    let mut t_s = 0.0;
    let turn = |sc: &mut Scorer,
                lines: &mut Vec<String>,
                t_s: &mut f64,
                sign: f64,
                rollout_hdg: f64| {
        for i in 0..30 {
            let mut s = air(*t_s, 3500.0, 90.0 + sign * i as f64 * 12.0, 95.0);
            s.bank_deg = sign * 45.0;
            lines.extend(sc.tick(&s));
            *t_s += 1.0;
        }
        for _ in 0..5 {
            lines.extend(sc.tick(&air(*t_s, 3500.0, rollout_hdg, 95.0)));
            *t_s += 1.0;
        }
    };
    turn(&mut sc, &mut lines, &mut t_s, 1.0, 94.0);
    assert!(
        lines.iter().any(|l| l == "One. Now the other way."),
        "{lines:?}"
    );
    turn(&mut sc, &mut lines, &mut t_s, -1.0, 82.0);
    assert!(sc.complete());
    let g = sc.grade();
    assert_eq!(g.notes.len(), 2);
    assert!(g.notes[0].contains("4 degrees right"), "{:?}", g.notes);
    assert!(g.notes[1].contains("8 degrees left"), "{:?}", g.notes);
    assert!(g.pass, "{}", g.spoken());
}

/// LFPG 09R, 2026-09-22, at the monitor's rate: -690 at 12 ft, then a
/// sample on the oleos at -221 with the flag still clear, then ground.
/// The wheels met the runway at 690, and the grade says so.
#[test]
fn the_touchdown_sink_is_the_steepest_in_the_window_not_the_last_sample() {
    let recent = [(0.0, -772.0), (1.1, -750.0), (2.1, -690.0), (3.2, -221.0)];
    assert_eq!(touchdown_sink_fpm(&recent, 4.3, -38.0), -690.0);
    // Only the window counts: a steep rate five seconds up is the approach.
    let recent = [(0.0, -900.0), (2.5, -300.0), (3.5, -250.0)];
    assert_eq!(touchdown_sink_fpm(&recent, 4.5, -156.0), -300.0);
    // The ground sample itself when it is the steepest on record.
    assert_eq!(touchdown_sink_fpm(&[(4.0, -100.0)], 4.5, -180.0), -180.0);
    assert_eq!(touchdown_sink_fpm(&[], 4.5, -180.0), -180.0);
}

#[test]
fn a_landing_reports_the_touchdown_and_a_firm_one_fails() {
    let t = Targets {
        ias_kts: Some(65.0),
        ..Targets::default()
    };
    let mut sc = Scorer::new(Task::Landing, Level::Private, t);
    let mut lines = Vec::new();
    let mut agl = 600.0;
    let mut t_s = 0.0;
    while agl > 0.0 {
        let mut s = air(t_s, 1000.0 + agl, 90.0, 66.0);
        s.agl_ft = agl;
        s.vsi_fpm = -500.0;
        lines.extend(sc.tick(&s));
        agl -= 500.0 / 60.0 * 2.0;
        t_s += 2.0;
    }
    let mut down = air(t_s, 1000.0, 92.0, 55.0);
    down.agl_ft = 0.0;
    down.on_ground = true;
    lines.extend(sc.tick(&down));
    assert!(sc.complete());
    let g = sc.grade();
    assert!(
        g.notes[0].starts_with("Touchdown at 500 feet per minute, 66 knots, no drift. Firm."),
        "{:?}",
        g.notes
    );
    assert!(!g.pass, "a firm arrival is not to standard");
    let ias = g.axes.iter().find(|a| a.axis == Axis::Airspeed).unwrap();
    assert!(ias.pass, "1 kt fast on final");
}

#[test]
fn requests_are_a_closed_vocabulary() {
    let _g = crate::mode::TEST_LOCK.lock().unwrap();
    lesson().stage = Stage::Idle;
    assert_eq!(
        request("teach me steep turns"),
        Some(Request::Start(Task::SteepTurns))
    );
    assert_eq!(
        request("let's practice slow flight"),
        Some(Request::Start(Task::SlowFlight))
    );
    assert_eq!(
        request("lesson on the traffic pattern"),
        Some(Request::Start(Task::Pattern))
    );
    assert_eq!(
        request("coach me through a normal landing"),
        Some(Request::Start(Task::Landing))
    );
    assert_eq!(
        request("practice a constant airspeed climb"),
        Some(Request::Start(Task::Climb))
    );
    assert_eq!(
        request("teach me straight and level"),
        Some(Request::Start(Task::StraightAndLevel))
    );
    assert_eq!(request("teach me something"), Some(Request::Menu));
    assert_eq!(request("what can you teach"), Some(Request::Menu));
    assert_eq!(request("end the lesson"), Some(Request::End));
    assert_eq!(request("how am I doing?"), Some(Request::Interim));
    assert_eq!(
        request("commercial standards please"),
        Some(Request::SetLevel(Level::Commercial))
    );
    assert_eq!(request("say the standards again"), Some(Request::Standards));
    // Commands and chatter are not lessons.
    assert_eq!(request("climb to four thousand"), None);
    assert_eq!(request("flaps ten"), None);
    assert_eq!(request("descend and maintain three thousand"), None);
    assert_eq!(request("begin"), None, "nothing briefed");
    assert_eq!(request("talk me through the descent checklist"), None);
}

#[test]
fn the_lesson_runs_brief_begin_tick_debrief() {
    let _g = crate::mode::TEST_LOCK.lock().unwrap();
    // Reset the process-wide lesson.
    {
        let mut l = lesson();
        l.stage = Stage::Idle;
        l.refs = refs172();
        l.level = Level::Private;
        l.latest = None;
    }
    assert!(
        handle(Request::Start(Task::StraightAndLevel), 0.0).contains("can't see the aircraft")
    );
    tick(air(0.0, 3480.0, 271.0, 106.0));
    let brief = handle(Request::Start(Task::StraightAndLevel), 0.0);
    assert!(
        brief.starts_with("Straight and level. Hold 3,500 feet, heading 270, 108 knots"),
        "{brief}"
    );
    assert_eq!(
        request("begin"),
        Some(Request::Begin),
        "a brief is standing"
    );
    assert_eq!(request("okay ready"), Some(Request::Begin));
    assert_eq!(
        status().as_deref(),
        Some("lesson: straight and level, briefed")
    );
    assert_eq!(handle(Request::Begin, 1.0), "Your aircraft. Go.");
    assert!(active());
    // A refused AP command becomes the target.
    assert_eq!(
        take_target("set_heading", &json!({ "heading": 300 })).as_deref(),
        Some("Heading 300. Your hands.")
    );
    assert_eq!(take_target("set_flaps", &json!({ "notch": 1 })), None);
    let mut said = Vec::new();
    for i in 2..130 {
        said.extend(tick(air(i as f64, 3500.0, 300.0, 108.0)));
    }
    assert!(!active(), "complete: back to idle");
    assert_eq!(said.first().map(String::as_str), Some("That's time."));
    assert!(
        said.last().unwrap().ends_with("That meets the standard."),
        "{said:?}"
    );
    assert!(handle(Request::Interim, 200.0).ends_with("That meets the standard."));
    assert_eq!(handle(Request::End, 200.0), "No lesson running.");
    assert_eq!(request("begin"), None, "nothing briefed any more");
}

#[test]
fn a_lesson_needs_height_and_a_wing() {
    let _g = crate::mode::TEST_LOCK.lock().unwrap();
    {
        let mut l = lesson();
        l.stage = Stage::Idle;
        l.refs = refs172();
        l.level = Level::Private;
    }
    let mut ground = air(0.0, 100.0, 90.0, 0.0);
    ground.on_ground = true;
    ground.agl_ft = 0.0;
    tick(ground);
    assert_eq!(
        handle(Request::Start(Task::SteepTurns), 0.0),
        "For steep turns, get airborne first."
    );
    let brief = handle(Request::Start(Task::Pattern), 0.0);
    assert!(
        brief.starts_with(
            "Traffic pattern. Pattern altitude 1,100 feet, 74 knots on the downwind"
        ),
        "{brief}"
    );
    assert_eq!(handle(Request::End, 0.0), "Lesson cancelled.");
    let mut low = air(1.0, 1200.0, 90.0, 100.0);
    low.agl_ft = 900.0;
    tick(low);
    assert_eq!(
        handle(Request::Start(Task::SlowFlight), 1.0),
        "For slow flight, climb above fifteen hundred feet first."
    );
}

#[test]
fn the_offline_grader_finds_the_level_runs_and_the_landing() {
    let mk = |t: f64, alt: f64, agl: f64, hdg: f64, ias: f64, vsi: f64, ground: bool| json!({ "t": t, "alt": alt, "agl": agl, "hdg": hdg, "ias": ias, "vsi": vsi, "ground": ground, "track": hdg, "bank": 0.0 });
    let mut states = Vec::new();
    let mut t = 1000.0;
    // 90 s level at 3,500, wandering 40 ft.
    for i in 0..90 {
        states.push(mk(
            t,
            3500.0 + (i % 4) as f64 * 10.0,
            3000.0,
            180.0,
            105.0,
            0.0,
            false,
        ));
        t += 1.0;
    }
    // 60 s descent at 500 fpm, 100 kt.
    let mut alt = 3500.0;
    for _ in 0..60 {
        states.push(mk(t, alt, alt - 500.0, 180.0, 100.0, -500.0, false));
        alt -= 500.0 / 60.0;
        t += 1.0;
    }
    // 120 s level at 3,000 but 160 ft low for 30 s.
    for i in 0..120 {
        let a = if (40..70).contains(&i) {
            2840.0
        } else {
            3000.0
        };
        states.push(mk(t, a, a - 500.0, 180.0, 105.0, 0.0, false));
        t += 1.0;
    }
    // Final at 68 kt and a soft touchdown.
    let mut agl = 700.0;
    while agl > 0.0 {
        states.push(mk(t, 500.0 + agl, agl, 180.0, 68.0, -400.0, false));
        agl -= 400.0 / 60.0;
        t += 1.0;
    }
    states.push(mk(t, 500.0, 0.0, 180.0, 60.0, -150.0, true));
    let refs: Vec<&Value> = states.iter().collect();
    let profile = crate::aircraft::load("cessna_172sp").expect("172 profile");
    let lines = grade_flight(&refs, Some(&profile), Level::Private);
    assert_eq!(lines.len(), 4, "{lines:#?}");
    assert!(
        lines[0].contains("level at 3500 ft for 89 s")
            && lines[0].contains("Within private standards"),
        "{}",
        lines[0]
    );
    assert!(
        lines[1].contains("descent from 3500 ft for 59 s") && lines[1].contains("Within"),
        "{}",
        lines[1]
    );
    assert!(
        lines[2].contains("level at 3000 ft for 2 min")
            && lines[2].contains("altitude 160 feet low (FAIL)"),
        "{}",
        lines[2]
    );
    assert!(
        lines[3].contains("final and landing")
            && lines[3].contains(
                "; Touchdown at 400 feet per minute, 68 knots, no drift. Firm. Below"
            ),
        "{}",
        lines[3]
    );
}

fn gnd(t: f64, gs: f64, hdg: f64, eng: bool) -> Sample {
    Sample {
        t_s: t,
        alt_ft: 100.0,
        agl_ft: 0.0,
        hdg_deg: hdg,
        ias_kts: gs,
        bank_deg: 0.0,
        vsi_fpm: 0.0,
        on_ground: true,
        track_deg: Some(hdg),
        gs_kts: gs,
        engine_running: eng,
        taxi: None,
    }
}

#[test]
fn a_taxi_is_graded_on_pace_and_completes_at_the_hold_short() {
    let t = Targets {
        pace_kts: Some(15.0),
        ..Targets::default()
    };
    let mut sc = Scorer::new(Task::Taxi, Level::Private, t);
    let mut lines = Vec::new();
    let mut t_s = 0.0;
    for _ in 0..60 {
        lines.extend(sc.tick(&gnd(t_s, 12.0, 90.0, true)));
        t_s += 1.0;
    }
    assert!(lines.is_empty(), "12 kt is under the pace: {lines:?}");
    for _ in 0..6 {
        lines.extend(sc.tick(&gnd(t_s, 20.0, 90.0, true)));
        t_s += 1.0;
    }
    assert_eq!(
        lines,
        vec!["Pace, 5 over. Ease the power back and hold it with the brakes.".to_string()]
    );
    for _ in 0..10 {
        lines.extend(sc.tick(&gnd(t_s, 12.0, 90.0, true)));
        t_s += 1.0;
    }
    assert_eq!(
        lines.last().map(String::as_str),
        Some("Good, back on pace.")
    );
    for _ in 0..12 {
        lines.extend(sc.tick(&gnd(t_s, 0.0, 90.0, true)));
        t_s += 1.0;
    }
    assert!(sc.complete(), "{lines:?}");
    assert_eq!(
        lines.last().map(String::as_str),
        Some("Stopped. Holding short, I take it.")
    );
    let g = sc.grade();
    assert!(!g.pass, "6 s over the pace: {}", g.spoken());
    assert!(g.notes[0].starts_with("Taxied 4"), "{:?}", g.notes);
    // A taxi that rolls straight onto the runway never held short.
    let mut sc = Scorer::new(Task::Taxi, Level::Private, t);
    for i in 0..40 {
        sc.tick(&gnd(i as f64, if i < 30 { 12.0 } else { 40.0 }, 90.0, true));
    }
    assert!(sc.complete() && !sc.grade().pass);
    assert!(sc.grade().notes[0].contains("without stopping"));
}

/// A taxi on a cleared route at 12 kt: `off(i)` is the cross-track at
/// tick `i` (1 s ticks), a right corner flown from tick 30 to 38 (the
/// route asks for it from 28; the heading swings 90 to 180), the
/// projection jumping `jump` metres along the route at tick 34 (a
/// vertex; 0 is a drawn curve), then a stop. Returns the lines said and
/// the samples fed.
fn taxi_on_route(
    level: Level,
    off: impl Fn(usize) -> f64,
    jump: f64,
) -> (Scorer, Vec<String>, Vec<Sample>) {
    let t = Targets {
        pace_kts: Some(15.0),
        ..Targets::default()
    };
    let mut sc = Scorer::new(Task::Taxi, level, t);
    let (mut lines, mut fed) = (Vec::new(), Vec::new());
    let mut rem = 800.0;
    for i in 0..90usize {
        let moving = i < 78;
        let gs = if moving { 12.0 } else { 0.0 };
        let hdg = match i {
            0..=30 => 90.0,
            31..=38 => 90.0 + (i - 30) as f64 * 11.25,
            _ => 180.0,
        };
        if moving {
            rem -= 12.0 * KTS_TO_MPS + if i == 34 { jump } else { 0.0 };
        }
        let s = Sample {
            taxi: Some(TaxiLine {
                xtk_m: off(i),
                remaining_m: rem,
                turn: (28..=34).contains(&i).then_some(1.0),
                off_route: false,
            }),
            ..gnd(i as f64, gs, hdg, true)
        };
        lines.extend(sc.tick(&s));
        fed.push(s);
    }
    (sc, lines, fed)
}

fn centreline(g: &Grade) -> &AxisGrade {
    g.axes
        .iter()
        .find(|a| a.axis == Axis::Centreline)
        .expect("a centreline axis")
}

#[test]
fn the_taxi_is_judged_on_the_centreline_and_a_vertex_cut_is_a_cut() {
    // Off the stand 20 m from the line, joined by tick 5, 0.3 m down the
    // straights, 3.8 m inside the right corner while turning through it.
    let off = |i: usize| match i {
        0..=4 => 20.0 - i as f64 * 4.9,
        32..=36 => 3.8,
        _ => 0.3,
    };
    let (sc, lines, _) = taxi_on_route(Level::Private, off, 5.0);
    assert!(sc.complete());
    assert_eq!(
        lines,
        vec!["Stopped. Holding short, I take it.".to_string()],
        "a cut inside a vertex is no excursion, the join is not judged"
    );
    let g = sc.grade();
    assert!(g.pass, "{}", g.spoken());
    assert_eq!(centreline(&g).worst.abs().round(), 0.0, "{:?}", g.axes);
    assert!(
        g.notes.iter().all(|n| !n.contains("corner")),
        "{:?}",
        g.notes
    );
    // The same offset inside a drawn CURVE (no projection jump) is off
    // the line, as the scorecard reads it: 3.8 m for 5 s, its time
    // outside counted once the corner closes as a curve.
    let (sc, lines, _) = taxi_on_route(Level::Private, off, 0.0);
    let g = sc.grade();
    assert_eq!((centreline(&g).worst * 10.0).round() / 10.0, 3.8);
    assert_eq!(centreline(&g).out_s, 5.0);
    assert!(!centreline(&g).pass);
    assert_eq!(lines.len(), 1, "never narrated in the corner: {lines:?}");
    // Commercial holds the line to 2 m: the same curve fails it.
    let (sc, _, _) = taxi_on_route(Level::Commercial, off, 0.0);
    assert!(!centreline(&sc.grade()).pass);
    // A vertex cut deeper than TAXI_MAX_CUT_M fails the task.
    let deep = |i: usize| if (33..=34).contains(&i) { 6.0 } else { 0.3 };
    let (sc, _, _) = taxi_on_route(Level::Private, deep, 5.0);
    let g = sc.grade();
    assert!(!g.pass);
    assert!(
        g.notes.contains(&"Cut a corner by 6 metres.".to_string()),
        "{:?}",
        g.notes
    );
}

#[test]
fn a_wander_off_the_line_is_narrated_once_with_its_correction() {
    let off = |i: usize| if (10..18).contains(&i) { 4.2 } else { 0.3 };
    let (sc, lines, _) = taxi_on_route(Level::Private, off, 5.0);
    assert_eq!(
        lines,
        vec![
            "Centreline, 4 metres right. A touch of left rudder, onto the line.".to_string(),
            "Good, back on centreline.".to_string(),
            "Stopped. Holding short, I take it.".to_string(),
        ]
    );
    let g = sc.grade();
    assert!(!g.pass);
    assert_eq!(
        centreline(&g).spoken(),
        "Centreline: 4 metres right at worst, outside for 8 seconds."
    );
    // The brief says the standard.
    let brief = Task::Taxi.brief(
        &Task::Taxi.assign(&gnd(0.0, 0.0, 90.0, true), &Refs::default(), Level::Private),
        Level::Private,
    );
    assert!(brief.contains("centreline within 3 metres"), "{brief}");
}

#[test]
fn the_debrief_grades_a_taxi_from_its_state_records() {
    let off = |i: usize| if (10..18).contains(&i) { 4.2 } else { 0.3 };
    let (_, _, fed) = taxi_on_route(Level::Private, off, 5.0);
    let states: Vec<Value> = fed
        .iter()
        .map(|s| {
            json!({"kind": "state", "t": 1_790_000_000.0 + s.t_s, "alt": s.alt_ft,
                "agl": 0.0, "hdg": s.hdg_deg, "ias": s.ias_kts, "vsi": 0.0,
                "ground": true, "gs": s.gs_kts, "eng": true,
                "taxi": s.taxi.map(|l| l.json())})
        })
        .collect();
    let refs: Vec<&Value> = states.iter().collect();
    let lines = grade_flight(&refs, None, Level::Private);
    assert_eq!(lines.len(), 1, "{lines:?}");
    assert!(
        lines[0]
            .contains("taxi for 89 s: pace 3 knots under, centreline 4 metres right (FAIL)"),
        "{lines:?}"
    );
    // A state record without the field grades no taxi.
    let bare: Vec<Value> = states
        .iter()
        .map(|v| {
            let mut v = v.clone();
            v.as_object_mut().unwrap().remove("taxi");
            v
        })
        .collect();
    let refs: Vec<&Value> = bare.iter().collect();
    assert!(grade_flight(&refs, None, Level::Private).is_empty());
}

#[test]
fn a_takeoff_captures_the_roll_heading_and_judges_the_rotation_and_climb() {
    let t = Targets {
        hdg_deg: Some(180.0), // the hold-short heading: a placeholder
        ias_kts: Some(74.0),
        vr_kts: Some(55.0),
        ..Targets::default()
    };
    let mut sc = Scorer::new(Task::Takeoff, Level::Private, t);
    let mut lines = Vec::new();
    let mut t_s = 0.0;
    // Lined up on 092, the roll.
    let mut gs = 0.0;
    while gs < 56.0 {
        lines.extend(sc.tick(&gnd(t_s, gs, 92.0, true)));
        gs += 3.0;
        t_s += 1.0;
    }
    assert_eq!(lines, vec!["Rolling.".to_string()]);
    // Airborne at 56, climbing at 72 on 094.
    let mut agl = 10.0;
    while agl < 1100.0 {
        let mut s = air(t_s, 100.0 + agl, 94.0, 72.0);
        s.agl_ft = agl;
        s.vsi_fpm = 700.0;
        lines.extend(sc.tick(&s));
        agl += 700.0 / 60.0;
        t_s += 1.0;
    }
    assert!(sc.complete(), "{lines:?}");
    assert!(lines.contains(&"Airborne.".to_string()));
    assert_eq!(
        lines.last().map(String::as_str),
        Some("A thousand feet. That completes the takeoff.")
    );
    let g = sc.grade();
    assert_eq!(
        g.notes[0], "Rotated at 54 knots, Vr is 55.",
        "the last sample on the wheels"
    );
    let h = g.axes.iter().find(|a| a.axis == Axis::Heading).unwrap();
    assert_eq!(h.target, 92.0, "the roll heading, not the hold-short's");
    assert_eq!(h.worst, 2.0);
    assert!(g.pass, "{}", g.spoken());
    // An early rotation fails the task.
    let mut sc = Scorer::new(Task::Takeoff, Level::Private, t);
    for i in 0..15 {
        sc.tick(&gnd(i as f64, i as f64 * 3.0, 92.0, true));
    }
    let mut s = air(15.0, 150.0, 92.0, 43.0);
    s.agl_ft = 5.0;
    sc.tick(&s);
    let g = sc.grade();
    assert_eq!(g.notes[0], "Rotated at 42 knots, Vr is 55. Early.");
    assert!(!g.pass);
}

/// A checklist flow that has some lists and runs each for two ticks.
struct FakeIo {
    have: Vec<&'static str>,
    running_left: u32,
    started: Vec<usize>,
}

impl SyllabusIo for FakeIo {
    fn find_list(&self, key: &str) -> Option<usize> {
        self.have.iter().position(|k| *k == key)
    }
    fn start_list(&mut self, idx: usize) -> bool {
        if self.running_left > 0 {
            return false;
        }
        self.started.push(idx);
        self.running_left = 2;
        true
    }
    fn list_running(&self) -> bool {
        self.running_left > 0
    }
}

impl FakeIo {
    fn tick(&mut self) {
        self.running_left = self.running_left.saturating_sub(1);
    }
}

#[test]
fn the_full_flight_runs_from_cold_and_dark_to_shutdown() {
    let _g = crate::mode::TEST_LOCK.lock().unwrap();
    {
        let mut l = lesson();
        l.stage = Stage::Idle;
        l.refs = refs172();
        l.level = Level::Private;
        l.latest = None;
    }
    let mut io = FakeIo {
        have: vec!["before taxi", "before takeoff", "engine shutdown"],
        running_left: 0,
        started: Vec::new(),
    };
    let mut t_s = 0.0;
    let mut said: Vec<String> = Vec::new();
    let run = |s: Sample, io: &mut FakeIo, said: &mut Vec<String>| {
        said.extend(tick_with(s, io));
        io.tick();
    };
    // Cold and dark on the ramp.
    run(gnd(t_s, 0.0, 180.0, false), &mut io, &mut said);
    assert!(handle(Request::Start(Task::SteepTurns), t_s).contains("get airborne first"));
    let opener = handle(Request::FullFlight, t_s);
    assert!(
        opener.starts_with("Full flight from cold and dark."),
        "{opener}"
    );
    assert_eq!(
        status().as_deref(),
        Some("lesson: full flight, preflight checklist")
    );
    assert!(syllabus_active());
    assert!(handle(Request::FullFlight, t_s).contains("already on the full flight"));
    // The three ramp lists this airframe lacks are skipped in one tick,
    // then the engine is waited for.
    t_s += 1.0;
    run(gnd(t_s, 0.0, 180.0, false), &mut io, &mut said);
    assert_eq!(
        said.iter().filter(|l| l.starts_with("No ")).count(),
        3,
        "{said:?}"
    );
    assert_eq!(
        status().as_deref(),
        Some("lesson: full flight, engine start")
    );
    // Engine comes alive: the before taxi list runs for two ticks.
    for _ in 0..5 {
        t_s += 1.0;
        run(gnd(t_s, 0.0, 180.0, true), &mut io, &mut said);
    }
    assert!(said.contains(&"Engine's running.".to_string()));
    assert!(said.contains(&"Before taxi checklist. I'll read, you do.".to_string()));
    assert_eq!(io.started, vec![0]);
    assert_eq!(
        status().as_deref(),
        Some("lesson: full flight, taxi"),
        "{said:?}"
    );
    assert!(
        said.iter()
            .any(|l| l.starts_with("Taxi. Get your clearance")),
        "{said:?}"
    );
    // A pace command during the taxi retargets the pace, never writes.
    assert_eq!(
        take_target("hold_speed", &json!({ "knots": 12 })).as_deref(),
        Some("12 knots it is. Your hands.")
    );
    // Taxi 60 s at 10 kt, stop 11 s.
    for _ in 0..60 {
        t_s += 1.0;
        run(gnd(t_s, 10.0, 180.0, true), &mut io, &mut said);
    }
    for _ in 0..11 {
        t_s += 1.0;
        run(gnd(t_s, 0.0, 180.0, true), &mut io, &mut said);
    }
    assert!(
        said.contains(&"Stopped. Holding short, I take it.".to_string()),
        "{said:?}"
    );
    // Before takeoff list, then the takeoff.
    for _ in 0..4 {
        t_s += 1.0;
        run(gnd(t_s, 0.0, 180.0, true), &mut io, &mut said);
    }
    assert_eq!(io.started, vec![0, 1]);
    assert_eq!(
        status().as_deref(),
        Some("lesson: full flight, takeoff"),
        "{said:?}"
    );
    let mut gs = 0.0;
    while gs < 58.0 {
        t_s += 1.0;
        run(gnd(t_s, gs, 92.0, true), &mut io, &mut said);
        gs += 3.0;
    }
    let mut agl = 10.0;
    while agl < 1100.0 {
        t_s += 1.0;
        let mut s = air(t_s, 100.0 + agl, 92.0, 73.0);
        s.agl_ft = agl;
        s.vsi_fpm = 700.0;
        run(s, &mut io, &mut said);
        agl += 700.0 / 60.0;
    }
    assert!(
        said.contains(&"A thousand feet. That completes the takeoff.".to_string()),
        "{said:?}"
    );
    assert!(
        said.iter().any(|l| l.starts_with("Your flight.")),
        "{said:?}"
    );
    assert_eq!(status().as_deref(), Some("lesson: full flight, enroute"));
    // Cruise, then a final: the landing is taken.
    for _ in 0..30 {
        t_s += 1.0;
        run(air(t_s, 3000.0, 92.0, 105.0), &mut io, &mut said);
    }
    let mut agl = 700.0;
    while agl > 0.0 {
        t_s += 1.0;
        let mut s = air(t_s, 100.0 + agl, 92.0, 66.0);
        s.agl_ft = agl;
        s.vsi_fpm = -250.0;
        run(s, &mut io, &mut said);
        agl -= 250.0 / 60.0;
    }
    assert!(
        said.contains(&"On final. I have the landing.".to_string()),
        "{said:?}"
    );
    t_s += 1.0;
    run(gnd(t_s, 55.0, 92.0, true), &mut io, &mut said);
    assert!(said.iter().any(|l| l.starts_with("Down.")), "{said:?}");
    // Roll out, taxi in, stop: the shutdown list, then the flight is over.
    for _ in 0..25 {
        t_s += 1.0;
        run(gnd(t_s, 0.0, 92.0, true), &mut io, &mut said);
    }
    for _ in 0..4 {
        t_s += 1.0;
        run(gnd(t_s, 0.0, 92.0, false), &mut io, &mut said);
    }
    assert_eq!(io.started, vec![0, 1, 2], "{said:?}");
    let last = said.last().unwrap();
    assert!(last.starts_with("That's the flight. Taxi: to standard. Takeoff: to standard. Landing: to standard. Lists done: before taxi, before takeoff, shutdown."), "{last}");
    assert!(!active(), "back to idle");
}

#[test]
fn the_full_flight_starts_at_before_taxi_with_the_engine_running() {
    let _g = crate::mode::TEST_LOCK.lock().unwrap();
    {
        let mut l = lesson();
        l.stage = Stage::Idle;
        l.refs = refs172();
        l.level = Level::Private;
    }
    tick(gnd(0.0, 0.0, 180.0, true));
    let opener = handle(Request::FullFlight, 0.0);
    assert!(
        opener.starts_with("Full flight from here. Engine's running"),
        "{opener}"
    );
    assert_eq!(
        status().as_deref(),
        Some("lesson: full flight, before taxi checklist")
    );
    assert_eq!(request("teach me a full flight"), Some(Request::FullFlight));
    assert_eq!(
        request("lesson from cold and dark"),
        Some(Request::FullFlight)
    );
    assert!(handle(Request::Start(Task::Taxi), 0.0).contains("on the full flight"));
    let end = handle(Request::End, 0.0);
    assert!(end.starts_with("Lesson over. Nothing graded yet."), "{end}");
    assert!(!active());
    // Airborne, the full flight is refused.
    tick(air(1.0, 3000.0, 90.0, 100.0));
    assert!(handle(Request::FullFlight, 1.0).contains("starts on the ramp"));
}

#[test]
fn the_refusal_names_the_action_and_the_way_back() {
    let first = refusal("FLAPS > 1", true);
    assert!(
        first.starts_with(
            "Instructor mode: I don't touch the controls. Flaps 1: that's your hands."
        ),
        "{first}"
    );
    assert!(first.contains("Settings"));
    assert_eq!(refusal("HEADING > 270", false), "Heading 270: your hands.");
}

#[test]
fn every_task_briefs_without_dashes_for_every_level() {
    let entry = air(0.0, 3500.0, 90.0, 100.0);
    for task in Task::ALL {
        for level in [Level::Private, Level::Commercial] {
            let t = task.assign(&entry, &refs172(), level);
            let b = task.brief(&t, level);
            assert!(
                !b.contains('\u{2014}') && !b.contains('\u{2013}') && !b.contains(" - "),
                "{b}"
            );
            assert!(b.contains("standards:"), "{b}");
        }
    }
    let c = Task::SteepTurns.assign(&entry, &refs172(), Level::Commercial);
    assert_eq!(c.bank_deg, Some(50.0));
    assert_eq!(c.ias_kts, Some(95.0), "at or below Va, rounded down to 5");
}

// ---------------------------------------------------- the demonstration

/// A clean slate for a demonstration test: instructor mode, no lesson,
/// the loop flags down. The caller holds `mode::TEST_LOCK`.
fn demo_reset() {
    use std::sync::atomic::Ordering::SeqCst;
    crate::mode::set(crate::mode::Mode::Instructor);
    crate::mode::close_demonstration();
    TEST_DEMO_RUNNING.store(false, SeqCst);
    TEST_DEMO_STOPPED.store(false, SeqCst);
    let mut l = lesson();
    l.stage = Stage::Idle;
    l.refs = refs172();
    l.level = Level::Private;
    l.latest = None;
    l.last_grade = None;
    l.pending_command = None;
}

fn demo_done() {
    use std::sync::atomic::Ordering::SeqCst;
    TEST_DEMO_RUNNING.store(false, SeqCst);
    lesson().stage = Stage::Idle;
    crate::mode::set(crate::mode::Mode::Both);
}

#[test]
fn show_me_is_a_closed_vocabulary() {
    let _g = crate::mode::TEST_LOCK.lock().unwrap();
    demo_reset();
    crate::mode::set(crate::mode::Mode::Both);
    assert_eq!(request("show me the weather"), None);
    assert_eq!(
        request("show me a takeoff"),
        None,
        "a task named outside instructor mode is not a lesson word"
    );
    crate::mode::set(crate::mode::Mode::Instructor);
    assert_eq!(
        request("show me a takeoff"),
        Some(Request::ShowMe(Some(Task::Takeoff)))
    );
    assert_eq!(
        request("can you demonstrate the landing"),
        Some(Request::ShowMe(Some(Task::Landing)))
    );
    assert_eq!(
        request("show me how to taxi"),
        Some(Request::ShowMe(Some(Task::Taxi)))
    );
    assert_eq!(
        request("show me the traffic pattern"),
        Some(Request::ShowMe(Some(Task::Pattern)))
    );
    assert_eq!(request("show me"), None, "no lesson standing, no task");
    assert_eq!(request("show me the weather"), None);
    assert_eq!(request("my controls"), None, "no demonstration running");
    lesson().stage = Stage::Briefed {
        task: Task::Landing,
    };
    assert_eq!(request("show me"), Some(Request::ShowMe(None)));
    assert_eq!(
        request("could you demonstrate it first please"),
        Some(Request::ShowMe(None))
    );
    assert_eq!(
        request("show me how it's done"),
        Some(Request::ShowMe(None))
    );
    assert_eq!(
        request("show me the fuel"),
        None,
        "a bare show-me carries nothing else"
    );
    assert_eq!(request("my controls"), None, "briefed is not demonstrating");
    demo_done();
}

/// Every demonstration walks the panel button's own words through the
/// tool door: the spoken gate passes them and the grammar picks the
/// very tool the permit opens.
#[test]
fn every_demonstration_is_the_panel_request_for_its_tool() {
    let profile = crate::aircraft::load("cessna_172sp").expect("cessna profile");
    let reg = crate::tools::Registry::for_aircraft(&profile);
    let mut shown = 0;
    for task in Task::ALL {
        let Some((tool, phrase)) = task.demonstration() else {
            continue;
        };
        shown += 1;
        assert!(crate::router::looks_like_command(phrase), "{phrase}");
        let calls = crate::grammar::try_parse(phrase, &reg, &profile)
            .unwrap_or_else(|| panic!("{phrase} does not parse"));
        assert_eq!(calls.len(), 1, "{phrase}");
        assert_eq!(calls[0].name, tool, "{phrase}");
    }
    assert_eq!(shown, 4, "taxi, takeoff, pattern, landing");
    assert_eq!(Task::SteepTurns.demonstration(), None, "air work is hands");
}

#[test]
fn a_takeoff_demonstration_flies_narrates_and_hands_back_to_the_brief() {
    use std::sync::atomic::Ordering::SeqCst;
    let _g = crate::mode::TEST_LOCK.lock().unwrap();
    demo_reset();
    tick(gnd(0.0, 0.0, 90.0, true));
    assert!(handle(Request::Start(Task::Takeoff), 0.5).starts_with("Normal takeoff"));
    assert_eq!(request("show me"), Some(Request::ShowMe(None)));
    let line = handle(Request::ShowMe(None), 1.0);
    assert!(
        line.starts_with("My controls. I'll show you a normal takeoff."),
        "{line}"
    );
    // The main loop routes the button's words, once.
    assert_eq!(take_demo_command(), Some("take off for me"));
    assert_eq!(take_demo_command(), None);
    // The hands gate: that tool once, nothing else, the duties still off.
    assert!(!crate::mode::tool_may_actuate("set_flaps", &json!({})));
    assert!(crate::mode::tool_may_actuate("auto_takeoff", &json!({})));
    assert!(!crate::mode::tool_may_actuate("auto_takeoff", &json!({})));
    assert!(!crate::mode::crew_actuates());
    assert_eq!(status().as_deref(), Some("lesson: demonstrating takeoff"));
    assert_eq!(
        request("begin"),
        None,
        "nothing briefed while demonstrating"
    );
    assert_eq!(
        handle(Request::Interim, 1.5),
        "I'm flying this one. Watch, and you'll fly it next."
    );
    // The automation flies: the roll, the rotation at 55, the climb.
    TEST_DEMO_RUNNING.store(true, SeqCst);
    let mut said = Vec::new();
    let mut t = 2.0;
    let mut gs = 0.0;
    while gs < 58.0 {
        gs += 3.0;
        let mut s = gnd(t, gs, 90.0, true);
        s.ias_kts = gs;
        said.extend(tick(s));
        t += 1.0;
    }
    let mut agl = 0.0;
    while agl < 400.0 {
        agl += 40.0;
        let mut s = air(t, 100.0 + agl, 90.0, 70.0);
        s.agl_ft = agl;
        s.vsi_fpm = 700.0;
        said.extend(tick(s));
        t += 1.0;
    }
    assert_eq!(said.len(), 4, "one line per teaching point: {said:?}");
    assert!(said[0].ends_with("Rotation at 55 knots."), "{said:?}");
    assert!(said[1].starts_with("Power set"), "{said:?}");
    assert!(said[2].starts_with("Rotate."), "{said:?}");
    assert_eq!(
        said[3],
        "Positive rate. Pitch for 74 knots, runway heading, and trim."
    );
    // The loop hands over: two quiet seconds end the demonstration.
    TEST_DEMO_RUNNING.store(false, SeqCst);
    let mut end = Vec::new();
    for _ in 0..4 {
        let mut s = air(t, 520.0, 90.0, 74.0);
        s.agl_ft = 420.0;
        end.extend(tick(s));
        t += 1.0;
    }
    assert_eq!(
        end,
        vec![
            "That's the demonstration. Your controls. Set up for it again and say begin."
                .to_string()
        ]
    );
    // The lesson stands briefed on the same task, the permit shut, and
    // nothing of the demonstration graded to the student.
    assert!(matches!(
        lesson().stage,
        Stage::Briefed {
            task: Task::Takeoff
        }
    ));
    assert!(lesson().last_grade.is_none());
    assert_eq!(crate::mode::demonstration_tool(), None);
    assert!(!crate::mode::tool_may_actuate(
        "auto_takeoff",
        &json!({ "off": true })
    ));
    demo_done();
}

#[test]
fn the_pilot_takes_a_demonstration_back_and_one_that_never_starts_resumes() {
    use std::sync::atomic::Ordering::SeqCst;
    let _g = crate::mode::TEST_LOCK.lock().unwrap();
    demo_reset();
    tick(air(0.0, 2500.0, 270.0, 90.0));
    // Straight from idle, naming the task.
    let line = handle(Request::ShowMe(Some(Task::Landing)), 1.0);
    assert!(line.contains("a normal approach and landing"), "{line}");
    assert_eq!(take_demo_command(), Some("land the aircraft"));
    TEST_DEMO_RUNNING.store(true, SeqCst);
    let said = tick(air(2.0, 2400.0, 270.0, 65.0));
    assert_eq!(
        said,
        vec![
            "A stabilised approach: the aim point fixed, 65 knots, small corrections early."
                .to_string()
        ]
    );
    assert_eq!(request("my controls"), Some(Request::TakeBack));
    assert_eq!(request("I have control"), Some(Request::TakeBack));
    assert_eq!(
        handle(Request::TakeBack, 3.0),
        "Your controls. Say begin when you're ready, and fly it the same way."
    );
    assert!(TEST_DEMO_STOPPED.load(SeqCst), "the loop was stood down");
    assert_eq!(crate::mode::demonstration_tool(), None);
    assert!(matches!(
        lesson().stage,
        Stage::Briefed {
            task: Task::Landing
        }
    ));
    // A demonstration whose tool refused: the loop never runs.
    TEST_DEMO_RUNNING.store(false, SeqCst);
    handle(Request::ShowMe(None), 10.0);
    let mut said = Vec::new();
    let mut t = 11.0;
    while t < 10.0 + DEMO_START_S + 2.0 {
        said.extend(tick(air(t, 2400.0, 270.0, 90.0)));
        t += 1.0;
    }
    assert_eq!(
        said,
        vec!["The demonstration didn't start. Your controls. Say begin when you're ready, and fly it the same way."
            .to_string()]
    );
    // The pilot's cancel at the tool door ends it in their hands.
    handle(Request::ShowMe(None), 50.0);
    TEST_DEMO_RUNNING.store(true, SeqCst);
    tick(air(51.0, 2400.0, 270.0, 65.0));
    assert!(crate::mode::tool_may_actuate(
        "autoland",
        &json!({ "cancel": true })
    ));
    TEST_DEMO_RUNNING.store(false, SeqCst);
    let mut said = Vec::new();
    for i in 0..4 {
        said.extend(tick(air(52.0 + i as f64, 2400.0, 270.0, 65.0)));
    }
    assert_eq!(
        said,
        vec![
            "Your controls. Say begin when you're ready, and fly it the same way.".to_string()
        ]
    );
    // "End the lesson" mid-demonstration stands the loop down too.
    handle(Request::ShowMe(None), 60.0);
    TEST_DEMO_STOPPED.store(false, SeqCst);
    assert_eq!(handle(Request::End, 61.0), "Lesson over. Your controls.");
    assert!(TEST_DEMO_STOPPED.load(SeqCst));
    assert!(!active());
    assert_eq!(crate::mode::demonstration_tool(), None);
    demo_done();
}

#[test]
fn only_the_crews_tasks_are_demonstrated_and_a_students_attempt_stays_theirs() {
    let _g = crate::mode::TEST_LOCK.lock().unwrap();
    demo_reset();
    tick(air(0.0, 3500.0, 90.0, 100.0));
    let no = handle(Request::ShowMe(Some(Task::SteepTurns)), 0.5);
    assert!(no.starts_with("I can't demonstrate steep turns"), "{no}");
    assert_eq!(take_demo_command(), None, "nothing routed");
    assert_eq!(crate::mode::demonstration_tool(), None, "nothing opened");
    let air_pattern = handle(Request::ShowMe(Some(Task::Pattern)), 0.6);
    assert!(
        air_pattern
            .starts_with("To show you the traffic pattern, I fly that one from the ground"),
        "{air_pattern}"
    );
    // A student flying the landing asks to see it: their attempt is
    // closed as theirs, and the demonstration opens.
    handle(Request::Start(Task::Landing), 1.0);
    handle(Request::Begin, 1.0);
    for i in 2..6 {
        tick(air(i as f64, 3400.0, 90.0, 66.0));
    }
    assert!(lesson().last_grade.is_none());
    assert!(handle(Request::ShowMe(None), 6.0).starts_with("My controls."));
    assert!(
        lesson().last_grade.is_some(),
        "the attempt so far is the student's"
    );
    assert_eq!(crate::mode::demonstration_tool(), Some("autoland"));
    assert_eq!(handle(Request::Start(Task::Climb), 6.5), DEMO_BUSY);
    // The mode changing under it retires the permit with the lesson.
    assert!(abandon("mode_change"));
    assert_eq!(crate::mode::demonstration_tool(), None);
    assert_eq!(take_demo_command(), None);
    // Outside instructor mode nothing is demonstrated.
    crate::mode::set(crate::mode::Mode::Both);
    let off = handle(Request::ShowMe(Some(Task::Takeoff)), 7.0);
    assert!(
        off.starts_with("Demonstrations run in instructor mode"),
        "{off}"
    );
    demo_done();
}
