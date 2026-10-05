//! LEMG 737, 2026-10-05 (sortie 1791187965-full_auto and the minutes
//! after): the real `state` records (`bench/replay/2026-10-05-lemg-737.jsonl`)
//! through the safe hold's decision.
//!
//! 12:42:29 xvl booted into a 737 at 2,900 ft flying autoland's last command
//! (VS -330, the window at 0, heading 306 toward the hills west of Malaga).
//! Nothing held it; at 12:44:58 it crossed a ridge 72 ft above the ground,
//! the pilot pulling up by hand. At 12:53:03 "my controls" cancelled a
//! self-picked autoland and left VS -1,500 with the window parked at 0.
//!
//! The ground ahead is what the aircraft actually flew over (the altimeter
//! less the height above the ground, from the later records along the
//! track), a lower bound on what the sim's probe would have answered.

use super::*;

const FIXTURE: &str = "bench/replay/2026-10-05-lemg-737.jsonl";
/// The boot: `flight_state` `airframe_begins` `why: boot`, 12:42:29.3.
const BOOT_T: f64 = 1_791_196_949.3;
/// "My controls" at 12:53:03.0 (autoland cancelled).
const MY_CONTROLS_T: f64 = 1_791_197_583.0;

fn records() -> Vec<Value> {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(FIXTURE);
    std::fs::read_to_string(path)
        .expect("the fixture")
        .lines()
        .map(|l| serde_json::from_str(l).expect("a json line"))
        .collect()
}

fn f(r: &Value, k: &str) -> f64 {
    r[k].as_f64().unwrap_or(f64::NAN)
}

/// One `state` record as the safe hold reads the aircraft. The altimeter is
/// taken as true altitude (QNH 30.08 against 30.12 set: under 50 ft).
fn air_of(r: &Value) -> Air {
    Air {
        lat: f(r, "lat"),
        lon: f(r, "lon"),
        alt_ft: f(r, "alt"),
        alt_true_ft: f(r, "alt"),
        agl_ft: f(r, "agl"),
        vs_fpm: f(r, "vs"),
        ias_kts: f(r, "ias"),
        gs_kts: f(r, "gs"),
        track_true: f(r, "track"),
        mag_var: -1.0,
        bank_deg: f(r, "bank"),
        pitch_deg: 0.0,
        flap: f(r, "flap"),
    }
}

/// The autopilot as recorded: the servos from the control the flight model
/// saw, the mode bits and the VS wheel; the window 0 (by hand at 12:30,
/// parked at 0 by autoland at 12:52:23).
fn ap_of(r: &Value) -> ApRead {
    let ctl = r["ctl"].as_str().unwrap_or("");
    let servos = if ctl == "Pilot" {
        Some(false)
    } else if ctl == "Autopilot" || ctl.starts_with("Automation") {
        Some(true)
    } else {
        None
    };
    ApRead {
        servos,
        state: r["in"]["ap_state"].as_i64(),
        alt_dial_ft: Some(0.0),
        vs_dial_fpm: r["in"]["vs_dial"].as_f64(),
        hdg_bug_mag: None,
    }
}

fn hands_of(r: &Value) -> bool {
    let yoke = ["yp", "yr"]
        .iter()
        .any(|k| r["in"][k].as_f64().is_some_and(|v| v.abs() > YOKE_HANDS));
    let latch = r["in"]["yield"]
        .as_array()
        .is_some_and(|a| a.iter().take(2).any(|v| v.as_bool() == Some(true)));
    yoke || latch
}

/// The ground along the track ahead, from the records flown after this one
/// inside [`LOOK_NM`] and 30 degrees of the track (the sea is at 0: the
/// altimeter less the height reads under it there).
fn ahead_of(states: &[Value], i: usize) -> Option<Ray> {
    let r = &states[i];
    let (lat, lon, trk) = (f(r, "lat"), f(r, "lon"), f(r, "track"));
    let ground: Vec<(f64, f64)> = states[i + 1..]
        .iter()
        .filter_map(|s| {
            let d = crate::nav::haversine_nm(lat, lon, f(s, "lat"), f(s, "lon"));
            let brg = crate::nav::bearing_deg(lat, lon, f(s, "lat"), f(s, "lon"));
            (d > 0.2 && d <= LOOK_NM && crate::angles::abs_diff_deg(brg, trk) <= 30.0)
                .then(|| (d, (f(s, "alt") - f(s, "agl")).max(0.0)))
        })
        .collect();
    (!ground.is_empty()).then_some(Ray {
        bearing_true: trk,
        ground_ft: ground,
    })
}

fn states(from: f64, to: f64) -> Vec<Value> {
    records()
        .into_iter()
        .filter(|r| r["kind"] == "state" && (from..=to).contains(&f(r, "t")))
        .collect()
}

/// The 737's own climb (`enroute_climb_fpm`).
fn climb_fpm() -> f64 {
    crate::aircraft::load("boeing_737_800")
        .map(|p| crate::tools::autopilot::ClimbPerf::from_profile(&p).max_climb_fpm)
        .unwrap_or(2_000.0)
}

fn tick_of(states: &[Value], i: usize, t0: f64, actuate: bool, pilot_words: bool) -> Tick {
    let r = &states[i];
    Tick {
        now_s: f(r, "t") - t0,
        airborne: r["ground"].as_bool() == Some(false),
        frozen: false,
        owner: None,
        pilot_words,
        hands: hands_of(r),
        ap: ap_of(r),
        air: air_of(r),
        terrain: Terrain {
            ahead: ahead_of(states, i),
            fan: vec![],
            mora_ft: None,
        },
        approach: None,
        terminal: None,
        unflyable: None,
        capability_fpm: climb_fpm(),
        actuate,
        ours: vec![],
    }
}

/// 12:42:29 to 12:44:50: the new code holds the aircraft within seconds of
/// the boot, climbing to the safe altitude over the ridge it was heading
/// for, on its own heading (the climb clears it), long before the ground.
#[test]
fn the_boot_into_autoland_s_last_descent_is_held_at_once() {
    let s = states(BOOT_T, 1_791_197_130.0);
    let mut h = SafeHold::default();
    let mut first: Option<(f64, HoldCmd, Value)> = None;
    for i in 0..s.len() {
        let (act, recs) = h.step(&tick_of(&s, i, BOOT_T, true, false));
        if let Act::Hold(cmd) = act {
            first = Some((f(&s[i], "t"), cmd, recs.last().cloned().unwrap()));
            break;
        }
    }
    let (t, cmd, rec) = first.expect("a hold");
    let after_boot = t - BOOT_T;
    assert!(after_boot <= 6.0, "held {after_boot:.1} s after the boot");
    assert!(cmd.climb, "{cmd:?}");
    assert_eq!(cmd.why, Why::CapturesBelow, "VS -330 to a window at 0");
    assert_eq!(
        cmd.heading_mag, None,
        "306 climbs clear of the ridge at the 737's rate"
    );
    // The ridge the 737 actually crossed at 12:44:58 stood 2,616 ft (72 ft
    // under it): the hold is a thousand feet over it at least.
    assert!(cmd.alt_ft >= 3_600.0, "{cmd:?}");
    assert_eq!(cmd.floor.source, "terrain");
    assert_eq!(rec["cause"], "boot");
    eprintln!(
        "LEMG replay: held {after_boot:.1} s after the boot, {}; climb to {:.0} ft at {:.0} fpm",
        rec["air"], cmd.alt_ft, cmd.fpm
    );
}

/// The same minutes with nothing allowed to move (instructor mode, or no
/// autopilot): the judgement as flown. Unsafe from the boot; the pilot's
/// VS wheel at 12:43:41 is the pilot's (told once); the servos out at
/// 12:44:36 leave the words only; at 450 ft and 1,500 fpm down the words
/// are "Terrain! Pull up."
#[test]
fn the_same_minutes_judged_as_flown() {
    let s = states(BOOT_T, 1_791_197_130.0);
    let mut h = SafeHold::default();
    let mut verdicts = Vec::new();
    let mut warns = Vec::new();
    let mut pilot_at = None;
    for i in 0..s.len() {
        let t = f(&s[i], "t");
        let (act, recs) = h.step(&tick_of(&s, i, BOOT_T, false, false));
        for r in &recs {
            if r["event"] == "verdict" {
                verdicts.push((
                    t,
                    r["verdict"].as_str().unwrap().to_string(),
                    r["who"].as_str().unwrap().to_string(),
                ));
            }
            if r["event"] == "pilot" && pilot_at.is_none() {
                pilot_at = Some((t, r["evidence"].as_str().unwrap().to_string()));
            }
        }
        if let Act::Warn(l) = act {
            warns.push((t, l));
        }
        assert!(!matches!(
            h.step(&tick_of(&s, i, BOOT_T, false, false)).0,
            Act::Hold(_) | Act::Escape(_)
        ));
    }
    assert_eq!(
        verdicts.first().map(|v| v.1.as_str()),
        Some("unsafe"),
        "{verdicts:?}"
    );
    let (pt, ev) = pilot_at.expect("the pilot's wheel");
    assert_eq!(ev, "vs_wheel");
    assert!((pt - 1_791_197_021.0).abs() < 4.0, "12:43:41, got {pt}");
    // The unattended warning came at the boot's confirmation, not at 12:43:41.
    assert!(warns[0].0 - BOOT_T <= 6.5, "{warns:?}");
    // "Terrain! Pull up." a minute and more before the ridge (12:44:58,
    // 72 ft), and again between the servos coming out and the ridge.
    let pull_up: Vec<f64> = warns
        .iter()
        .filter(|(_, l)| l == TERRAIN_PULL_UP)
        .map(|w| w.0)
        .collect();
    assert!(
        pull_up.first().is_some_and(|t| *t < 1_791_197_040.0),
        "{warns:?}"
    );
    assert!(
        pull_up
            .iter()
            .any(|t| (1_791_197_076.0..1_791_197_098.0).contains(t)),
        "{warns:?}"
    );
    eprintln!(
        "LEMG replay as flown: verdicts {verdicts:?}\n  warns {warns:?}\n  pilot {pt:.1} {ev}"
    );
}

/// 12:53:03: "my controls" cancels the self-picked autoland and leaves
/// VS -1,500 with the window at 0, over the sea toward the strait. The crew
/// neutralises its own leftover at once: the descent bounded at the safe
/// altitude, a pilot's release named as the cause.
#[test]
fn my_controls_leaves_no_descent_without_a_floor() {
    let s = states(MY_CONTROLS_T + 0.1, 1_791_197_640.0);
    let mut h = SafeHold::default();
    let mut held = None;
    for i in 0..s.len() {
        let words = f(&s[i], "t") - MY_CONTROLS_T <= RELEASE_WORDS_S;
        let (act, recs) = h.step(&tick_of(&s, i, MY_CONTROLS_T, true, words));
        if i == 0 {
            assert_eq!(recs[0]["cause"], "pilot_release");
        }
        if let Act::Hold(cmd) = act {
            held = Some((f(&s[i], "t"), cmd));
            break;
        }
    }
    let (t, cmd) = held.expect("a hold");
    assert!(t - MY_CONTROLS_T <= 6.0, "{}", t - MY_CONTROLS_T);
    assert!(
        !cmd.climb,
        "7,600 ft over the sea: the descent bounded, not a climb"
    );
    assert_eq!(cmd.why, Why::CapturesBelow);
    assert_eq!(cmd.alt_ft, 1_000.0, "the sea plus a thousand: {cmd:?}");
    eprintln!(
        "LEMG replay 12:53: bounded at {:.0} ft ({} {:.0}) {:.1} s after my controls",
        cmd.alt_ft,
        cmd.floor.source,
        cmd.floor.true_ft,
        t - MY_CONTROLS_T
    );
}

/// Every airborne, unowned stretch of a whole blackbox through the decision:
/// what the safe hold would have done. `XVL_SAFE_HOLD_LOG=logs/blackbox.jsonl
/// cargo test -- --ignored safe_hold_sweep --nocapture`. The owner is read
/// from the record's `ctl` (an `Automation(...)` is a crew loop).
#[test]
#[ignore]
fn safe_hold_sweep() {
    let Ok(path) = std::env::var("XVL_SAFE_HOLD_LOG") else {
        eprintln!("set XVL_SAFE_HOLD_LOG");
        return;
    };
    let text = std::fs::read_to_string(path).unwrap();
    let states: Vec<Value> = text
        .lines()
        .filter(|l| l.contains("\"kind\":\"state\"") || l.contains("\"kind\": \"state\""))
        .filter_map(|l| serde_json::from_str(l).ok())
        .collect();
    let mut h = SafeHold::default();
    let mut counts = std::collections::BTreeMap::<String, u32>::new();
    let mut last_t = 0.0;
    let mut ends: Option<(f64, f64, Vec<End>)> = None;
    for i in 0..states.len() {
        let r = &states[i];
        let t = f(r, "t");
        if t - last_t > 30.0 {
            h = SafeHold::default();
        }
        last_t = t;
        let mut tick = tick_of(&states[..(i + 80).min(states.len())], i, 0.0, true, false);
        let ctl = r["ctl"].as_str().unwrap_or("");
        if ctl.starts_with("Automation") {
            tick.owner = Some("loop");
        }
        if !tick.air.lat.is_finite() || !tick.air.alt_ft.is_finite() {
            continue;
        }
        // The runway ends round us, from the installed apt.dat, as live.
        if tick.airborne && tick.owner.is_none() {
            let (la, lo) = (tick.air.lat, tick.air.lon);
            if ends
                .as_ref()
                .is_none_or(|(a, b, _): &(f64, f64, Vec<End>)| {
                    crate::nav::haversine_nm(*a, *b, la, lo) > 6.0
                })
            {
                ends = Some((la, lo, super::ends_near(la, lo)));
            }
            if let Some((_, _, e)) = &ends {
                tick.approach = on_approach(e, &tick.air);
                tick.terminal = in_terminal(e, &tick.air);
            }
        }
        let (act, _) = h.step(&tick);
        let name = match &act {
            Act::Nothing => continue,
            Act::Hold(c) => format!("hold {}", c.why.word()),
            Act::Escape(_) => "escape".to_string(),
            Act::Warn(l) => format!("warn {}", &l[..l.len().min(20)]),
            Act::TakeOver { why, .. } => format!("take_over {why}"),
            Act::End(w) => format!("end {w}"),
        };
        if !name.starts_with("end") {
            let ts = chrono_like(t);
            eprintln!(
                "{ts} {name}: alt {:.0} agl {:.0} vs {:.0} trk {:.0} ap {:?} ctl {ctl}",
                f(r, "alt"),
                f(r, "agl"),
                f(r, "vs"),
                f(r, "track"),
                r["in"]["ap_state"]
            );
        }
        *counts
            .entry(name.split(':').next().unwrap().to_string())
            .or_default() += 1;
        if let Act::Hold(c) | Act::Escape(c) = act {
            h.written(c, tick.now_s, tick.air.alt_ft, Some(tick.ap));
        }
    }
    eprintln!("{counts:#?}");
}

fn chrono_like(t: f64) -> String {
    let s = t as i64 + 2 * 3600;
    format!("{:02}:{:02}:{:02}", (s / 3600) % 24, (s / 60) % 60, s % 60)
}
