//! The Zibo 737 from the rotation to the liftoff (LEMG 30, 2026-10-07).
//!
//! The five rolls the scheduled rudder law flew there (1791371247 to
//! 1791388851; every `roll` tick from the brake release to the liftoff in
//! bench/takeoffs/zibo738_liftoffs_lemg30_2026-10-07.jsonl) held 1.7 to 4.6 m
//! to 110 kt, and four of them lifted off 5.3 to 11.1 m right. Nothing in the
//! law changes at the rotation (the same cut, gain, trim and predictor step
//! on, the brake split is not the Zibo's, the rudder lets go only off the
//! wheels). The aircraft does: the track follows the heading through about
//! 0.75 s on the wheels, 1 to 2 s toward Vr and seconds once the nose is up,
//! and the pedals answer about their 80 kt figure to the rotation and a
//! fraction of it after. The four crossed the centreline at the rotation
//! still moving 0.9 to 2.2 m/s right, and the cut, led on the cross-track
//! rate read (the heading's answer that late), had held the nose across the
//! line to get there.
#![allow(clippy::needless_range_loop)]

use super::*;
use crate::aircraft::RollRudderProfile;
use serde_json::Value;

const LIFTOFFS: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/bench/takeoffs/zibo738_liftoffs_lemg30_2026-10-07.jsonl"
));

fn f(r: &Value, k: &str) -> f64 {
    r[k].as_f64().unwrap_or(0.0)
}

fn flag(r: &Value, k: &str) -> bool {
    r[k].as_bool().unwrap_or(false)
}

/// The five rolls, each from the brake release to the last tick before the
/// liftoff. `rotated` marks the ticks after the `rotate` event (the law's
/// own flag), `wheels` the sim's `onground_any`.
fn liftoffs() -> Vec<(String, Vec<Value>)> {
    let mut rolls: Vec<(String, Vec<Value>)> = Vec::new();
    for r in LIFTOFFS
        .lines()
        .filter_map(|l| serde_json::from_str::<Value>(l).ok())
    {
        let id = r["sortie"].as_str().unwrap_or("").to_string();
        match rolls.last_mut() {
            Some((last, ticks)) if *last == id => ticks.push(r),
            _ => rolls.push((id, vec![r])),
        }
    }
    rolls
}

/// The runway course the law steered for: the target less its cut, the
/// median over the roll from 40 kt.
fn course(ticks: &[Value]) -> f64 {
    let mut c: Vec<f64> = ticks
        .iter()
        .filter(|r| f(r, "gs") > 40.0)
        .map(|r| f(r, "tgt") + f(r, "cut") + f(r, "cut_i") + f(r, "drift"))
        .collect();
    c.sort_by(|a, b| a.total_cmp(b));
    c[c.len() / 2]
}

/// Seconds from the rotation for every tick (the sum of the logged `dt`; the
/// record's `t` is rounded to a tenth), negative before it.
fn from_rotation(ticks: &[Value]) -> Vec<f64> {
    let mut ts = vec![0.0; ticks.len()];
    for k in 1..ticks.len() {
        ts[k] = ts[k - 1] + f(&ticks[k], "dt");
    }
    let rot = ticks
        .iter()
        .position(|r| flag(r, "rotated"))
        .expect("every roll rotated");
    let at = ts[rot];
    ts.iter().map(|t| t - at).collect()
}

/// The Zibo's rudder as the profile declares it (`roll_rudder:`).
fn rudder_zibo() -> RollRudderProfile {
    let text = std::fs::read_to_string("profiles/zibo_738.yaml").unwrap();
    let p: crate::aircraft::AircraftProfile = serde_yaml::from_str(&text).unwrap();
    p.roll_rudder
        .and_then(|r| r.valid())
        .expect("the Zibo declares its rudder")
}

/// The law these rolls flew: no track lag declared, the predictor reading
/// the declared rudder past its speed.
fn law_as_flown() -> (RollSteer, RollCut) {
    let law = RollSteer {
        predictor_past_fit: true,
        ..RollSteer::with_rudder(
            true,
            RollRudderProfile {
                track_lag_s: None,
                ..rudder_zibo()
            },
        )
    };
    let cut = RollCut::for_law(&law);
    (law, cut)
}

/// The law as the profile now declares it.
fn law_declared() -> (RollSteer, RollCut) {
    let law = RollSteer::with_rudder(true, rudder_zibo());
    let cut = RollCut::for_law(&law);
    (law, cut)
}

/// The pedals' dead time and lag (`roll_rudder:` `dead_s`, `lag_s`).
const DEAD_S: f64 = 1.0;
const LAG_S: f64 = 0.2;

/// The pedals' yaw, deg/s per unit of steer, nose down: the declared figure
/// to 80 kt and flat past it, as the five rolls answered it (windowed fits
/// to the recorded yaw rate fall from 0.5 to 0.6 of the declared v^0.75 at
/// 50 to 90 KIAS to 0.3 to 0.45 at 105 KIAS and over), the nosewheel at a
/// walk below 40 kt as the declared fit has it.
fn pedal_yaw(gs: f64) -> f64 {
    let gs = gs.max(0.0);
    12.7 * (gs.min(80.0) / 80.0).powf(0.75) + 0.23 * gs * (1.0 - gs / 40.0).max(0.0)
}

/// The declared fit extrapolated (the 737 plant's crosswind is a yaw on
/// it, so [`fly`] takes it as the steer it needs there).
fn declared_yaw(gs: f64) -> f64 {
    let gs = gs.max(0.0);
    12.7 * (gs / 80.0).powf(0.75) + 0.23 * gs * (1.0 - gs / 40.0).max(0.0)
}

/// Once rotated the column has unloaded the nosewheel: the pedals' yaw is
/// this share (the windowed fits give 0.2 to 0.4 of the nose-down figure).
const ROTATED_SHARE: f64 = 0.25;

/// The track's lag behind the heading on the wheels, nose down, seconds:
/// 0.5 to 1.0 to 100 KIAS, 1 to 2 by Vr ([`the_track_lags_the_heading`]).
fn wheels_track_lag_s(ias: f64) -> f64 {
    if ias < 100.0 {
        0.75
    } else {
        (0.75 + (ias - 100.0) / 25.0 * 1.25).min(2.0)
    }
}

/// ...and once rotated (1.5 s to longer than the phase lasts).
const ROTATED_TRACK_LAG_S: f64 = 4.0;

/// One replayed or flown tick.
struct Out {
    gs: f64,
    xtk: f64,
    steer: f64,
    rotated: bool,
}

/// The steer reaching the pedals: none off the wheels once rotated (the
/// float, rudder released).
fn written(r: &Value, steer: f64) -> f64 {
    if flag(r, "rotated") && !flag(r, "wheels") {
        0.0
    } else {
        steer
    }
}

/// One recorded roll replayed under a law: the aircraft is the recorded one
/// plus the answer to the DIFFERENCE between the law's steer and the
/// recorded one, through the pedals' dead time and lag and the yaw above
/// ([`pedal_yaw`], [`ROTATED_SHARE`]), then the track's lag behind the
/// heading ([`wheels_track_lag_s`], [`ROTATED_TRACK_LAG_S`]; held off the
/// wheels, the heading no longer the track's). Everything the roll met is
/// met again as it came. Returns the ticks and the largest difference from
/// the recorded steer.
fn replay(ticks: &[Value], mut law: RollSteer, mut cut: RollCut) -> (Vec<Out>, f64) {
    const H: f64 = 0.01;
    let course = course(ticks);
    let n = ticks.len();
    let mut ts = vec![0.0; n];
    for k in 1..n {
        ts[k] = ts[k - 1] + f(&ticks[k], "dt");
    }
    let (mut dyaw, mut dpsi, mut dtrk, mut dx) = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
    let mut queue: std::collections::VecDeque<(f64, f64)> = Default::default();
    let mut applied = 0.0_f64;
    let mut t = 0.0_f64;
    let mut out = Vec::with_capacity(n);
    let mut dev = 0.0_f64;
    for k in 0..n {
        let r = &ticks[k];
        let dt = if k == 0 { 0.2 } else { f(r, "dt") };
        if k > 0 {
            let p = &ticks[k - 1];
            let (gs0, gs1) = (f(p, "gs"), f(r, "gs"));
            let (ias0, ias1) = (f(p, "ias"), f(r, "ias"));
            let rotated = flag(r, "rotated");
            let floating = rotated && !flag(r, "wheels");
            let end = ts[k];
            while t < end - 1e-9 {
                while queue.front().is_some_and(|(at, _)| *at <= t) {
                    if let Some((_, s)) = queue.pop_front() {
                        applied = s;
                    }
                }
                let frac = 1.0 - (end - t) / dt;
                let gs = gs0 + (gs1 - gs0) * frac;
                let ias = ias0 + (ias1 - ias0) * frac;
                let yaw_per = if floating {
                    0.0
                } else if rotated {
                    pedal_yaw(gs) * ROTATED_SHARE
                } else {
                    pedal_yaw(gs)
                };
                dyaw += (yaw_per * applied - dyaw) * H / LAG_S;
                dpsi += dyaw * H;
                if !floating {
                    let lag = if rotated {
                        ROTATED_TRACK_LAG_S
                    } else {
                        wheels_track_lag_s(ias)
                    };
                    dtrk += (dpsi - dtrk) * H / lag;
                }
                dx += gs / MS_TO_KTS * dtrk.to_radians() * H;
                t += H;
            }
            t = end;
        }
        let gs = f(r, "gs");
        let psi = f(r, "psi") + dpsi;
        let c = cut.update(
            f(r, "xtk_m") + dx,
            crate::angles::signed_diff_deg(psi, course),
            gs,
            dt,
        );
        let err = crate::angles::signed_diff_deg(course + c.target_off_deg, psi);
        let yaw = r["r"].as_f64().map(|v| v + dyaw);
        let steer = law.step_with_yaw(err, dt, gs, 1.0, yaw);
        dev = dev.max((steer - f(r, "steer")).abs());
        queue.push_back((
            ts[k] + DEAD_S,
            written(r, steer) - written(r, f(r, "steer")),
        ));
        out.push(Out {
            gs,
            xtk: f(r, "xtk_m") + dx,
            steer,
            rotated: flag(r, "rotated"),
        });
    }
    (out, dev)
}

/// Tick-to-tick steer jumps past 0.08 on the wheels before the rotation from
/// 30 kt: the weave's signature.
fn swings(out: &[Out]) -> usize {
    out.windows(2)
        .filter(|w| w[0].gs >= 30.0 && !w[1].rotated && (w[1].steer - w[0].steer).abs() > 0.08)
        .count()
}

/// The least-squares misfit of the cross-track over a span of ticks when
/// the track follows the heading through a first-order lag of `tau` s: the
/// offset, the track's start against the heading and a steady drift free.
fn track_misfit(ticks: &[Value], course: f64, tau: f64) -> f64 {
    const H: f64 = 0.01;
    let n = ticks.len();
    let mut ts = vec![0.0; n];
    for k in 1..n {
        ts[k] = ts[k - 1] + f(&ticks[k], "dt");
    }
    let psi: Vec<f64> = ticks
        .iter()
        .map(|r| crate::angles::signed_diff_deg(f(r, "psi"), course))
        .collect();
    let v: Vec<f64> = ticks.iter().map(|r| f(r, "gs") / MS_TO_KTS).collect();
    // Per tick: the cross-track the lagged heading makes, and the bases of
    // the free start (a decaying degree) and drift (a steady degree).
    let (mut lp, mut i_lp, mut i_e, mut i_c, mut t) = (psi[0], 0.0, 0.0, 0.0, 0.0_f64);
    let mut rows = vec![[0.0, 0.0, 0.0, 1.0]];
    for k in 1..n {
        while t < ts[k] - 1e-9 {
            let a = ((t - ts[k - 1]) / (ts[k] - ts[k - 1]).max(1e-9)).clamp(0.0, 1.0);
            let p = psi[k - 1] + (psi[k] - psi[k - 1]) * a;
            let vv = v[k - 1] + (v[k] - v[k - 1]) * a;
            lp += (p - lp) * H / tau;
            i_lp += vv * lp.to_radians() * H;
            i_e += vv * 1f64.to_radians() * (-t / tau).exp() * H;
            i_c += vv * 1f64.to_radians() * H;
            t += H;
        }
        rows.push([i_lp, i_e, i_c, 1.0]);
    }
    // Normal equations for the three free terms.
    let mut m = [[0.0_f64; 4]; 3];
    for (k, row) in rows.iter().enumerate() {
        let y = f(&ticks[k], "xtk_m") - row[0];
        let x = [row[1], row[2], row[3]];
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] += x[i] * x[j];
            }
            m[i][3] += x[i] * y;
        }
    }
    for c in 0..3 {
        let piv = (c..3)
            .max_by(|a, b| m[*a][c].abs().total_cmp(&m[*b][c].abs()))
            .unwrap();
        m.swap(c, piv);
        let d = m[c][c];
        if d.abs() < 1e-12 {
            continue;
        }
        for j in c..4 {
            m[c][j] /= d;
        }
        for i in 0..3 {
            if i != c {
                let fct = m[i][c];
                for j in c..4 {
                    m[i][j] -= fct * m[c][j];
                }
            }
        }
    }
    rows.iter()
        .enumerate()
        .map(|(k, row)| {
            let fit = row[0] + m[0][3] * row[1] + m[1][3] * row[2] + m[2][3] * row[3];
            (f(&ticks[k], "xtk_m") - fit).powi(2)
        })
        .sum()
}

/// The lag that fits the span best, from a grid.
const LAGS: [f64; 12] = [
    0.05, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0,
];

/// The fit `track_lag_s` and the plant stand on: over all five rolls, the
/// cross-track 12 to 6 s before the rotation (70 to 110 KIAS) is best made by
/// the heading through 0.5 to 1 s of lag; the last 6 s by 1 to 2.5 s; and
/// from the rotation to the wheels off by 1.5 s or more (a heading the track
/// hardly follows). With no lag at all the spans fit 7, 26 and 50 times
/// worse.
#[test]
fn the_track_lags_the_heading() {
    let mut pooled = [[0.0_f64; 12]; 3];
    for (_, ticks) in &liftoffs() {
        let c = course(ticks);
        let ts = from_rotation(ticks);
        let spans: [Vec<Value>; 3] = [
            (0..ticks.len())
                .filter(|&k| (-12.0..-6.0).contains(&ts[k]))
                .map(|k| ticks[k].clone())
                .collect(),
            (0..ticks.len())
                .filter(|&k| (-6.0..0.0).contains(&ts[k]))
                .map(|k| ticks[k].clone())
                .collect(),
            ticks
                .iter()
                .filter(|r| flag(r, "rotated") && flag(r, "wheels"))
                .cloned()
                .collect(),
        ];
        for (s, span) in spans.iter().enumerate() {
            assert!(span.len() >= 5, "a span of {} ticks", span.len());
            for (j, lag) in LAGS.iter().enumerate() {
                pooled[s][j] += track_misfit(span, c, *lag);
            }
        }
    }
    let best = |s: usize| {
        let j = (0..LAGS.len())
            .min_by(|a, b| pooled[s][*a].total_cmp(&pooled[s][*b]))
            .unwrap();
        LAGS[j]
    };
    let (early, late, rotated) = (best(0), best(1), best(2));
    println!(
        "track lag: {early} s at 70 to 110 KIAS, {late} s to Vr, {rotated} s rotated; \
         misfit with none {:.2}/{:.2}/{:.2} against {:.2}/{:.2}/{:.2}",
        pooled[0][0],
        pooled[1][0],
        pooled[2][0],
        pooled[0].iter().cloned().fold(f64::MAX, f64::min),
        pooled[1].iter().cloned().fold(f64::MAX, f64::min),
        pooled[2].iter().cloned().fold(f64::MAX, f64::min),
    );
    assert!((0.5..=1.0).contains(&early), "{early}");
    assert!((1.0..=2.5).contains(&late), "{late}");
    assert!(rotated >= 1.5, "{rotated}");
    for s in 0..3 {
        let min = pooled[s].iter().cloned().fold(f64::MAX, f64::min);
        assert!(
            pooled[s][0] > 2.0 * min,
            "span {s}: {:.3} against {min:.3}",
            pooled[s][0]
        );
    }
    assert_eq!(rudder_zibo().track_lag_s, Some(0.75));
}

/// The bench is the flights: the law as flown, replayed through each roll,
/// writes the recorded steer tick for tick, and the offsets at the liftoff
/// are the ones being fixed (1.6 m left, then 11.1, 5.3, 5.6 and 7.1 m
/// right), each roll within 4.6 m to the rotation.
#[test]
fn the_zibo_liftoffs_replay_as_flown() {
    let rolls = liftoffs();
    assert_eq!(rolls.len(), 5);
    let mut ends = Vec::new();
    for (id, ticks) in &rolls {
        let (law, cut) = law_as_flown();
        let (out, dev) = replay(ticks, law, cut);
        assert!(dev < 0.01, "{id}: the replayed law strays {dev:.4}");
        let before = out
            .iter()
            .filter(|o| !o.rotated && o.gs >= 30.0)
            .map(|o| o.xtk.abs())
            .fold(0.0, f64::max);
        let end = out.last().map_or(0.0, |o| o.xtk);
        println!("{id} as flown: {before:.2} m to the rotation, {end:+.2} m at the liftoff");
        assert!(before < 4.7, "{id}: {before:.2}");
        ends.push(end);
    }
    assert!(ends[0] < -1.0 && ends[0] > -2.0, "{ends:?}");
    assert!(ends[1..].iter().all(|e| *e > 5.0), "{ends:?}");
}

/// The fix, replayed through the five rolls: with the track lag declared
/// the cut leads on the heading's rate and the predictor believes the rudder
/// only to its speed. Every roll lifts off within 2.5 m where they flew 1.6
/// to 11.1, the five together under a third of what they flew, none worse
/// to the rotation than it flew, and the steer no more swinging than it was.
#[test]
fn the_zibo_liftoffs_replay_on_the_centreline() {
    let (mut flown_sum, mut new_sum) = (0.0, 0.0);
    for (id, ticks) in &liftoffs() {
        let (law, cut) = law_as_flown();
        let (flown, _) = replay(ticks, law, cut);
        let (law, cut) = law_declared();
        let (new, _) = replay(ticks, law, cut);
        let worst_before = |o: &[Out]| {
            o.iter()
                .filter(|o| !o.rotated && o.gs >= 30.0)
                .map(|o| o.xtk.abs())
                .fold(0.0, f64::max)
        };
        let (end_flown, end_new) = (flown.last().unwrap().xtk, new.last().unwrap().xtk);
        println!(
            "{id}: {end_new:+.2} m at the liftoff (flown {end_flown:+.2}), {:.2} m to the \
             rotation (flown {:.2}), {} swings (flown {})",
            worst_before(&new),
            worst_before(&flown),
            swings(&new),
            swings(&flown)
        );
        assert!(end_new.abs() < 2.5, "{id}: {end_new:+.2} m at the liftoff");
        assert!(worst_before(&new) <= worst_before(&flown) + 0.1, "{id}");
        assert!(swings(&new) <= swings(&flown), "{id}: the steer weaves");
        flown_sum += end_flown.abs();
        new_sum += end_new.abs();
    }
    println!("at the liftoff: {new_sum:.1} m over five rolls, flown {flown_sum:.1}");
    assert!(
        new_sum < flown_sum / 3.0,
        "{new_sum:.1} against {flown_sum:.1}"
    );
}

/// The fitted plant through the rotation: [`pedal_yaw`] nose down, the
/// track's lag behind the heading ([`wheels_track_lag_s`]); from the
/// rotation the track the aircraft has is the one it keeps (the limit of the
/// fitted 1.5 s and more: the plant gives the law nothing after the
/// rotation, and the share of rudder left and the crosswind's yaw there,
/// which the rolls do not pin, move nothing), the rudder released at Vr + 18
/// and the liftoff at Vr + 27 (the rolls: rotate 108 to 116 KIAS, wheels off
/// 15 to 20 kt later, liftoff 142 to 153); groundspeed 16 kt over the
/// airspeed as the rolls read it. The crosswind is the 737 plant's, taken as
/// the steer it needs. Returns the ticks and the offset at the liftoff.
#[allow(clippy::too_many_arguments)]
fn fly(
    mut law: RollSteer,
    mut cut: RollCut,
    xw_kt: f64,
    xtk0: f64,
    psi0: f64,
    accel_kts_s: f64,
    gust: Option<(f64, f64, f64)>,
) -> (Vec<Out>, f64) {
    const COURSE: f64 = 297.4;
    const TICK: f64 = 0.25;
    const H: f64 = 0.01;
    const VR: f64 = 115.0;
    const WHEELS_OFF: f64 = VR + 18.0;
    const LIFTOFF: f64 = VR + 27.0;
    const GS_OVER_IAS: f64 = 16.0;
    let (mut psi, mut yaw, mut xtk, mut gs) = (COURSE + psi0, 0.0_f64, xtk0, 0.0_f64);
    let mut trk = psi;
    let mut queue: std::collections::VecDeque<(f64, f64)> = Default::default();
    let mut applied = 0.0_f64;
    let mut t = 0.0_f64;
    let mut out = Vec::new();
    loop {
        let ias = gs - GS_OVER_IAS;
        if ias >= LIFTOFF {
            break;
        }
        let rotated = ias >= VR;
        let floating = ias >= WHEELS_OFF;
        let psi_off = crate::angles::signed_diff_deg(psi, COURSE);
        let c = cut.update(xtk, psi_off, gs, TICK);
        let err = c.target_off_deg - psi_off;
        let steer = law.step_with_yaw(err, TICK, gs, 1.0, Some(yaw));
        queue.push_back((t + DEAD_S, if floating { 0.0 } else { steer }));
        out.push(Out {
            gs,
            xtk,
            steer,
            rotated,
        });
        let next = t + TICK;
        while t < next - 1e-9 {
            while queue.front().is_some_and(|(at, _)| *at <= t) {
                applied = queue.pop_front().map_or(applied, |(_, s)| s);
            }
            let ias = gs - GS_OVER_IAS;
            let per = if rotated {
                pedal_yaw(gs) * ROTATED_SHARE
            } else {
                pedal_yaw(gs)
            };
            let old = xw_kt / 7.4 * (2.48 * gs / 60.0 - 0.99).max(0.0);
            let weathervane = old * pedal_yaw(gs) / declared_yaw(gs).max(1e-6);
            let g = gust.map_or(0.0, |(a, b, d)| if (a..b).contains(&ias) { d } else { 0.0 });
            yaw += (per * applied - weathervane + g - yaw) * H / LAG_S;
            psi += yaw * H;
            if !rotated {
                trk += (psi - trk) * H / wheels_track_lag_s(ias);
            }
            xtk += gs / MS_TO_KTS * (trk - COURSE).to_radians().sin() * H;
            gs += accel_kts_s * H;
            t += H;
        }
    }
    let end = xtk;
    (out, end)
}

/// The acceptance's starts: 1.4 m and 2 degrees off, either side and
/// across, at the rolls' 7 kt/s and a slower 5.
fn starts() -> Vec<(f64, f64, f64)> {
    let mut s = Vec::new();
    for (xtk0, psi0) in [(1.4, 2.0), (-1.4, -2.0), (1.4, -2.0), (-1.4, 2.0)] {
        for accel in [5.0, 7.0] {
            s.push((xtk0, psi0, accel));
        }
    }
    s
}

/// Through the fitted plant from every start: in calm air the declared law
/// lifts off within 1 m (1.2 m from 75 KIAS), in 8 kt across from either
/// side within 2.6 m (3.2 m from 75 KIAS), never weaving, where the law the
/// rolls flew lifts off up to 10.8 m off. A yaw gust of 1 deg/s for 15 kt
/// that ends 15 kt before Vr is lifted off from closer than the flown law
/// does; one held to Vr is not taken out by either before the rotation
/// (printed, the plant's limit: nothing turns the track after it).
#[test]
fn the_zibo_lifts_off_on_the_centreline_through_the_fitted_plant() {
    let (mut new_worst, mut flown_worst) = (0.0_f64, 0.0_f64);
    for xw in [0.0, 8.0, -8.0] {
        for (xtk0, psi0, accel) in starts() {
            let (law, cut) = law_declared();
            let (out, end) = fly(law, cut, xw, xtk0, psi0, accel, None);
            let fast = out
                .iter()
                .filter(|o| o.gs - 16.0 >= 75.0)
                .map(|o| o.xtk.abs())
                .fold(0.0, f64::max);
            let (law, cut) = law_as_flown();
            let (_, flown) = fly(law, cut, xw, xtk0, psi0, accel, None);
            println!(
                "xw {xw:+} from {xtk0:+} m {psi0:+} deg at {accel} kt/s: {end:+.2} m at the \
                 liftoff, {fast:.2} m from 75 KIAS (flown {flown:+.2})"
            );
            let (end_max, fast_max) = if xw == 0.0 { (1.0, 1.2) } else { (2.6, 3.2) };
            assert!(
                end.abs() < end_max,
                "xw {xw:+} {xtk0:+}/{psi0:+} at {accel}: {end:+.2}"
            );
            assert!(
                fast < fast_max,
                "xw {xw:+} {xtk0:+}/{psi0:+} at {accel}: {fast:.2}"
            );
            assert_eq!(
                swings(&out),
                0,
                "xw {xw:+} {xtk0:+}/{psi0:+} at {accel}: weaves"
            );
            new_worst = new_worst.max(end.abs());
            flown_worst = flown_worst.max(flown.abs());
        }
    }
    println!("worst at the liftoff {new_worst:.2} m, flown {flown_worst:.2} m");
    assert!(flown_worst > 8.0, "{flown_worst:.2}");
    for gust in [1.0, -1.0] {
        for (from, to) in [(85.0, 100.0), (100.0, 115.0)] {
            let (law, cut) = law_declared();
            let (_, end) = fly(law, cut, 0.0, 0.0, 0.0, 7.0, Some((from, to, gust)));
            let (law, cut) = law_as_flown();
            let (_, flown) = fly(law, cut, 0.0, 0.0, 0.0, 7.0, Some((from, to, gust)));
            println!("gust {gust:+} deg/s {from} to {to} KIAS: {end:+.2} m (flown {flown:+.2})");
            if to < 115.0 {
                assert!(end.abs() < flown.abs(), "{end:+.2} against {flown:+.2}");
            }
        }
    }
}

/// Every other airframe leads on the rate read, as before: none declares a
/// track lag, and a cut without one leads on the rate it reads.
#[test]
fn only_the_zibo_declares_a_track_lag() {
    for entry in std::fs::read_dir("profiles").unwrap() {
        let path = entry.unwrap().path();
        if path.extension().and_then(|e| e.to_str()) != Some("yaml") {
            continue;
        }
        let text = std::fs::read_to_string(&path).unwrap();
        let Ok(p) = serde_yaml::from_str::<crate::aircraft::AircraftProfile>(&text) else {
            continue;
        };
        let lag = p
            .roll_rudder
            .and_then(|r| r.valid())
            .and_then(|r| r.track_lag_s);
        if p.id == "zibo_738" {
            assert_eq!(lag, Some(0.75));
        } else {
            assert_eq!(lag, None, "{}", p.id);
        }
    }
    let law = RollSteer::with_rudder(
        true,
        RollRudderProfile {
            track_lag_s: None,
            ..rudder_zibo()
        },
    );
    let mut cut = RollCut::for_law(&law);
    for (k, x) in [0.0, 0.3, 0.7, 1.2, 1.6, 1.9].into_iter().enumerate() {
        let c = cut.update(x, 1.5, 60.0 + 10.0 * k as f64, 0.25);
        assert_eq!(c.lead_m_s, c.rate_m_s);
    }
}
