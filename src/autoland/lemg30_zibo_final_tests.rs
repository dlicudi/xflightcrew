use super::lemg_replay::*;
use super::*;

/// The recorded LEMG 30 ticks and windows (see the file's own note).
const BENCH: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/bench/approach/lemg30_zibo_2026-10-07.jsonl"
));
/// True less magnetic at LEMG, as the loop's writes show it (the bug it
/// wanted, true, against the magnetic it wrote).
const VAR_DEG: f64 = 0.4;
/// The loop's tick on these finals, seconds.
const TICK_S: f64 = 0.6;

#[derive(Debug, Clone)]
struct Tick {
    t: f64,
    final_phase: bool,
    pattern: bool,
    dist: f64,
    psi: f64,
    phi: f64,
    ias: f64,
    arr: bool,
}

/// The ticks and the window as read back (true), in time order.
fn recorded() -> (Vec<Tick>, Vec<(f64, f64)>) {
    let mut ticks = Vec::new();
    let mut windows = Vec::new();
    for line in BENCH.lines() {
        let v: serde_json::Value = serde_json::from_str(line).expect("a bench line");
        if v.get("kind").is_some() {
            continue;
        }
        let f = |k: &str| v.get(k).and_then(|x| x.as_f64());
        match v.get("ev").and_then(|e| e.as_str()) {
            Some("heading_set") => {
                if let (Some(t), Some(read)) = (f("t"), f("read")) {
                    windows.push((t, read + VAR_DEG));
                }
            }
            Some(_) => {}
            None => {
                let (Some(t), Some(psi), Some(phi), Some(dist)) =
                    (f("t"), f("psi"), f("phi"), f("dist"))
                else {
                    continue;
                };
                let ph = v.get("ph").and_then(|p| p.as_str()).unwrap_or("");
                ticks.push(Tick {
                    t,
                    final_phase: ph == "F",
                    pattern: ph == "I" && v.get("leg").is_some(),
                    dist,
                    psi,
                    phi,
                    ias: f("ias").unwrap_or(140.0),
                    arr: v.get("arr").and_then(|a| a.as_bool()).unwrap_or(false),
                });
            }
        }
    }
    (ticks, windows)
}

/// The window (true) in force at `t`.
fn window_at(windows: &[(f64, f64)], t: f64) -> Option<f64> {
    windows
        .iter()
        .take_while(|(wt, _)| *wt <= t)
        .last()
        .map(|(_, w)| *w)
}

/// `fit` flown on the recorded windows in 20 s pieces, each from its
/// recorded heading and bank: the heading and bank errors, rms degrees,
/// over the ticks `pick` keeps (contiguous runs only).
fn replay_rms(fit: HeadingFit, pick: impl Fn(&Tick) -> bool) -> (f64, f64, usize) {
    let (ticks, windows) = recorded();
    let delay_s = fit.delay_ticks as f64 * TICK_S;
    let (mut e_psi, mut e_phi, mut n) = (0.0, 0.0, 0usize);
    let mut i = 0;
    while i < ticks.len() {
        if !pick(&ticks[i]) || ticks[i].arr {
            i += 1;
            continue;
        }
        let start = i;
        let mut j = i + 1;
        while j < ticks.len()
            && pick(&ticks[j])
            && !ticks[j].arr
            && ticks[j].t - ticks[j - 1].t < 1.5
            && ticks[j].t - ticks[start].t < 20.0
        {
            j += 1;
        }
        let (mut psi, mut phi) = (ticks[start].psi, ticks[start].phi);
        for k in start + 1..j {
            let (t0, t1) = (ticks[k - 1].t, ticks[k].t);
            let v = ticks[k - 1].ias * 0.514_444 * 1.03;
            let steps = ((t1 - t0) / 0.05).ceil().max(1.0) as usize;
            let h = (t1 - t0) / steps as f64;
            for s in 0..steps {
                if let Some(w) = window_at(&windows, t0 + s as f64 * h - delay_s) {
                    phi += fit.roll_rate(crate::angles::signed_diff_deg(w, psi), phi) * h;
                }
                psi =
                    (psi + (9.81 * phi.to_radians().tan() / v).to_degrees() * h).rem_euclid(360.0);
            }
            e_psi += crate::angles::signed_diff_deg(psi, ticks[k].psi).powi(2);
            e_phi += (phi - ticks[k].phi).powi(2);
            n += 1;
        }
        i = j.max(start + 1);
    }
    ((e_psi / n as f64).sqrt(), (e_phi / n as f64).sqrt(), n)
}

fn finals(t: &Tick) -> bool {
    t.final_phase && t.dist > 0.3
}

fn turns(t: &Tick) -> bool {
    t.pattern
}

/// The Zibo, fitted: its heading select, its whole-degree window (magnetic,
/// LEMG's variation) and the nose's yaw about the path the finals show
/// (0.9 degrees rms of heading less track after a 15 s high-pass, 6 to 13 s
/// a cycle: 1.3 degrees at 9 s), `along_m` short of 30 and `xtk_m` right,
/// at 135 kt, crabbed for a `wind_kts` wind from `wind_from`, settled.
fn zibo(rwy: &Runway, along_m: f64, xtk_m: f64, wind_from: f64, wind_kts: f64) -> Jet {
    let (lat, lon) = at(rwy, along_m, xtk_m);
    let mut jet = Jet::new(lat, lon, rwy.course_deg, 135.0);
    jet.fit = ZIBO_HEADING;
    jet.window_step = Some(1.0);
    jet.var_deg = VAR_DEG;
    jet.yaw_wobble = Some((1.3, 9.0));
    jet.wind_to_deg = (wind_from + 180.0).rem_euclid(360.0);
    jet.wind_mps = wind_kts * 0.514_444;
    // The heading whose track is the course.
    let cross = jet.wind_mps
        * crate::angles::signed_diff_deg(jet.wind_to_deg, rwy.course_deg)
            .to_radians()
            .sin();
    jet.psi = (rwy.course_deg - (cross / jet.tas_mps).asin().to_degrees()).rem_euclid(360.0);
    jet
}

/// What a Zibo final came to: the worst offset inside 1.5 nm, the worst
/// in each half mile from 3 nm in, the offset at the roundout gate, the
/// first balk, the worst bank and the knob's moves.
#[derive(Debug, Default)]
struct ZiboFinal {
    worst_in_1_5_m: f64,
    halves: Vec<f64>,
    gate_xtk_m: f64,
    balk: Option<(&'static str, f64, f64)>,
    worst_bank_deg: f64,
    moves: u32,
}

fn fly(jet: &mut Jet, rwy: &Runway) -> ZiboFinal {
    let mut lp = FinalLoop::new(Some(LEMG_WIDTH_M)).held();
    let mut out = ZiboFinal {
        halves: vec![0.0; 6],
        ..Default::default()
    };
    for _ in 0..3000 {
        let (bug, g) = lp.step(jet, rwy, TICK_S);
        let x = cross_track_nm(jet.lat, jet.lon, rwy) * M_PER_NM;
        let along = along_track_nm(jet.lat, jet.lon, rwy);
        let radio = g.glidepath_alt_ft - rwy.elev_ft;
        if let Some(why) = g.balk {
            out.balk = Some((why, radio, x));
            break;
        }
        if radio < 80.0 {
            break;
        }
        out.gate_xtk_m = x;
        if along <= 1.5 {
            out.worst_in_1_5_m = out.worst_in_1_5_m.max(x.abs());
        }
        if along < 3.0 {
            let k = ((3.0 - along) / 0.5).floor().clamp(0.0, 5.0) as usize;
            out.halves[k] = out.halves[k].max(x.abs());
        }
        out.worst_bank_deg = out.worst_bank_deg.max(jet.phi.abs());
        jet.step(bug, TICK_S);
    }
    out.moves = lp.moves;
    out
}

/// The six finals of the brief: 30 m either side at 5 nm, calm and 10 kt
/// of crosswind from either side.
fn starts(rwy: &Runway) -> Vec<(f64, f64, f64)> {
    let mut v = Vec::new();
    for (wf, wk) in [
        (0.0, 0.0),
        (rwy.course_deg - 90.0, 10.0),
        (rwy.course_deg + 90.0, 10.0),
    ] {
        for x0 in [-30.0, 30.0] {
            v.push((x0, wf, wk));
        }
    }
    v
}

/// The fit: the recorded windows flown through the Zibo's heading select
/// from the recorded heading and bank, 20 s at a time. The stock 737's fit
/// (a 1 s lag at 7 degrees a second) rolls the pattern's turns three times
/// too fast for this autopilot, and banks the finals a third too hard.
#[test]
fn the_zibo_heading_fit_flies_the_recorded_windows() {
    let (z_fin, z_turn) = (
        replay_rms(ZIBO_HEADING, finals),
        replay_rms(ZIBO_HEADING, turns),
    );
    let (b_fin, b_turn) = (
        replay_rms(B737_HEADING, finals),
        replay_rms(B737_HEADING, turns),
    );
    assert!(z_fin.2 > 800 && z_turn.2 > 1000, "{z_fin:?} {z_turn:?}");
    // Recorded: 0.81 and 0.80 degrees rms on the finals, 1.08 and 0.66 in
    // the turns; the 737's fit 0.88 and 1.07, 2.76 and 2.86.
    assert!(z_fin.0 < 0.9 && z_fin.1 < 0.9, "finals {z_fin:?}");
    assert!(z_turn.0 < 1.2 && z_turn.1 < 0.8, "turns {z_turn:?}");
    assert!(
        b_fin.1 > z_fin.1 + 0.2,
        "737 finals {b_fin:?} against {z_fin:?}"
    );
    assert!(
        b_turn.0 > 2.0 * z_turn.0 && b_turn.1 > 3.0 * z_turn.1,
        "737 turns {b_turn:?}"
    );
}

/// The final as a pilot sets it on the fitted Zibo (2026-10-12): the knob in
/// whole degrees no oftener than every ten seconds above minimums, hand
/// flown below them. The pursuit it replaced parked 3.8 to 7.0 m off inside
/// 1.5 nm with the window still and weaved 18 to 28 m with the heading mode
/// settling a degree off its window and back over 50 s, 9 of 24 going
/// around (the flights' 16 to 34 m and their go-arounds); the one law
/// after it (`FinalLateral`, a quarter degree every second) held 1.0 to 4.1
/// and 7.3 to 10.4. Here, with the window still: no balk, 2 to 7 m inside
/// 1.5 nm, the gate on the line, the knob set every 15 to 60 s. With the
/// heading wandering: 13 to 33 m inside 1.5 nm, the gate on the line, and
/// one of 24 sent round below the floor 33 m off at 319 ft, the gate's call.
#[test]
fn the_zibo_final_holds_the_line() {
    let rwy = lemg("30");
    let mut balks = 0;
    for (x0, wf, wk) in starts(&rwy) {
        for wobble in [None, Some((1.3, 9.0))] {
            let mut jet = zibo(&rwy, 5.0 * M_PER_NM, x0, wf, wk);
            jet.yaw_wobble = wobble;
            let f = fly(&mut jet, &rwy);
            let case = format!("{x0} m, {wk} kt from {wf:.0}, wobble {wobble:?}");
            assert!(f.balk.is_none(), "{case}: {f:?}");
            assert!(f.worst_in_1_5_m < 8.0, "{case}: {f:?}");
            assert!(f.gate_xtk_m.abs() < 1.0, "{case}: {f:?}");
            assert!(f.worst_bank_deg < 6.0, "{case}: {f:?}");
            assert!(f.moves <= 9, "{case}: {f:?}");
        }
        for phase in [0.0, 1.6, 3.2, 4.7] {
            let mut jet = zibo(&rwy, 5.0 * M_PER_NM, x0, wf, wk);
            jet.ap_offset = Some((1.1, 50.0, phase));
            let f = fly(&mut jet, &rwy);
            match f.balk {
                Some((why, ..)) => {
                    assert_eq!(why, "unstabilised_below_floor", "{f:?}");
                    balks += 1;
                }
                None => assert!(f.gate_xtk_m.abs() < 2.0 && f.worst_in_1_5_m < 35.0, "{f:?}"),
            }
        }
    }
    assert!(balks <= 1, "{balks} of 24 wandering finals went round");
}

/// The fourth approach as it began its final (sortie 1791381498-full_auto,
/// 14:18:27Z): 32 m left at 5.49 nm, heading 298.6 and tracking 297.2 (a
/// 3 kt wind from the right), 136 kt. On the pursuit, with the heading
/// wandering a degree, it wove 19 m inside 1.5 nm (flown: +18 at 3.5 nm,
/// -5 at 2, +16 at 1, -20 at 0.31 and the go-around). As a pilot sets it:
/// 38 m at 1.5 nm wandering, 10 m by 1 nm and the gate on the line hand
/// flown; with the heading on its window, inside 8 m.
#[test]
fn the_fourth_approach_replayed() {
    let rwy = lemg("30");
    let fly_from = |wander: bool| {
        let mut jet = zibo(&rwy, 5.49 * M_PER_NM, -32.1, rwy.course_deg + 90.0, 3.3);
        jet.psi = 298.6;
        jet.tas_mps = 136.0 * 0.514_444;
        if wander {
            jet.ap_offset = Some((1.1, 50.0, 0.0));
        }
        fly(&mut jet, &rwy)
    };
    let f = fly_from(true);
    assert!(f.balk.is_none() && f.worst_in_1_5_m < 40.0 && f.halves[4] < 12.0 && f.gate_xtk_m.abs() < 1.0, "{f:?}");
    let f = fly_from(false);
    assert!(f.balk.is_none() && f.worst_in_1_5_m < 8.0 && f.gate_xtk_m.abs() < 1.0, "{f:?}");
}
