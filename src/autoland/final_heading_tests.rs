//! The final above minimums on heading modes FITTED to the 2026-10-12
//! finals (sortie tapes from 1791739000): the autopilot banks `k` per degree
//! of the knob less heading, low-passed over `tf`, plus a standing bias `b`,
//! first order over `tau` (the 172, SR22, King Air C90B, Baron 58, SF50;
//! 61 to 428 windows each, R2 0.64 to 0.96) or second order on the
//! Evolution (`wn` 1.10, `z` 0.71: R2 0.985 against the first order's
//! 0.982, 181 windows); the track turns at g tan(bank) / TAS; the heading
//! the autopilot reads carries a yaw the track does not, `ky_p` per deg/s
//! of roll rate and `ky_phi` per degree of bank (adverse yaw and the
//! sideslip in the turn: on the Evolution R2 0.54, the overshoot the first
//! fit missed).

use super::*;

const G: f64 = 9.81;
const KT: f64 = 0.514_444;

#[derive(Clone, Copy, Debug)]
struct Mode {
    name: &'static str,
    k: f64,
    tf: f64,
    b: f64,
    /// First order: the roll lag; second order: `wn`, `z` (tau 0).
    tau: f64,
    wn: f64,
    z: f64,
    ky_p: f64,
    ky_phi: f64,
    tas_kts: f64,
}

const MODES: [Mode; 6] = [
    Mode { name: "C172", k: 0.568, tf: 0.366, b: -0.095, tau: 1.43, wn: 0.0, z: 0.0, ky_p: -0.167, ky_phi: -0.093, tas_kts: 66.0 },
    Mode { name: "SR22", k: 0.764, tf: 0.948, b: -0.202, tau: 1.255, wn: 0.0, z: 0.0, ky_p: -0.125, ky_phi: -0.187, tas_kts: 77.0 },
    Mode { name: "King Air", k: 2.501, tf: 0.351, b: -0.606, tau: 1.69, wn: 0.0, z: 0.0, ky_p: -0.199, ky_phi: -0.131, tas_kts: 110.0 },
    Mode { name: "Evolution", k: 1.71, tf: 0.035, b: -0.382, tau: 0.0, wn: 1.10, z: 0.71, ky_p: -0.183, ky_phi: -0.194, tas_kts: 93.0 },
    Mode { name: "Baron", k: 1.002, tf: 0.609, b: -0.243, tau: 0.921, wn: 0.0, z: 0.0, ky_p: -0.234, ky_phi: -0.286, tas_kts: 104.0 },
    Mode { name: "SF50", k: 2.044, tf: 0.645, b: -0.558, tau: 1.193, wn: 0.0, z: 0.0, ky_p: 0.0, ky_phi: -0.295, tas_kts: 90.0 },
];

/// Who sets the knob: tonight's quarter degree every second
/// (`FinalLateral`, 2026-10-12, its response time the fast airframes' 3 s
/// floor), or the pilot's whole degree.
#[derive(Clone, Copy, PartialEq)]
enum Law {
    QuarterEverySecond,
    Pilot,
}

#[derive(Debug)]
struct Final {
    x_at_minimums_m: f64,
    worst_bank_last_min_deg: f64,
    sets_per_min: f64,
}

/// Fly `m` from 4.4 nm on course to the visual minimums (about 0.95 nm),
/// the line `x0` m away, a crosswind from the right `xw_kts`, gusts `gust_kts`
/// sd over 2 s, the heading mode settling `bias_deg` off its knob; the loop
/// ticks at 0.33 s and the knob arrives 0.1 s later.
fn fly(m: Mode, law: Law, x0: f64, xw_kts: f64, gust_kts: f64, bias_deg: f64) -> Final {
    let (h, tick) = (0.05, 0.33);
    let v = m.tas_kts * KT;
    let t_end = 3.45 * M_PER_NM / v;
    let crab_of = |xw: f64| (xw / m.tas_kts).clamp(-0.9, 0.9).asin().to_degrees();
    let b = m.b - bias_deg * m.k;
    let (mut x, mut hdg, mut phi, mut p, mut yaw) = (x0, crab_of(xw_kts), 0.0_f64, 0.0_f64, 0.0_f64);
    let mut ef = -b / m.k;
    let (mut knob, mut pending): (f64, Vec<(f64, f64)>) = (hdg, Vec::new());
    let mut crab = CrabFilter::new();
    let mut heading = FinalHeading::default();
    let mut hold = crate::heading_bug::BugHold::default();
    let (mut i_m, mut set_q, mut last_q) = (0.0_f64, None::<f64>, -1e9_f64);
    let (mut sets, mut t, mut next, mut worst_bank) = (0u32, 0.0, 0.0, 0.0_f64);
    let mut gust_state = 0x2545_f491_4f6c_dd1d_u64 ^ (x0.to_bits() ^ bias_deg.to_bits());
    let mut gust = 0.0_f64;
    while t < t_end {
        let xw = xw_kts + gust;
        let trk = hdg - crab_of(xw);
        let psi = hdg + yaw;
        if t >= next - 1e-9 {
            next += tick;
            let c = crab.update(CrabFilter::raw(Some(psi), Some(trk)), tick);
            let want = match law {
                Law::QuarterEverySecond => {
                    let tr = 3.0;
                    i_m = (i_m + x * tick / (4.0 * tr)).clamp(-8.0, 8.0);
                    let side = v * trk.to_radians().sin();
                    let cut = -((x + side * tr + i_m) / (v.max(30.0) * tr)).atan().to_degrees();
                    let want = cut.clamp(-30.0, 30.0) + c;
                    let off = set_q.map_or(f64::MAX, |s| crate::angles::abs_diff_deg(want, s));
                    if off >= 5.0 || (off >= 0.25 && t - last_q >= 1.0) {
                        set_q = Some(want);
                        last_q = t;
                        sets += 1;
                    }
                    set_q
                }
                Law::Pilot => {
                    let settled = hold.since_write_s(t).is_some_and(|s| s >= FINAL_SETTLE_S);
                    heading.learn(Some(psi), hold.set(), settled, tick);
                    let want = heading.heading_true(x, m.tas_kts, 0.0, c).round();
                    let wrote = hold.decide(2, want, t, AUTOLAND_FINAL_DEADBAND);
                    sets += u32::from(wrote.is_some());
                    hold.set().map(crate::angles::wrap180)
                }
            };
            if let Some(w) = want {
                pending.push((t + 0.1, crate::angles::wrap180(w)));
            }
        }
        while pending.first().is_some_and(|(at, _)| *at <= t + 1e-9) {
            knob = pending.remove(0).1;
        }
        let e = crate::angles::signed_diff_deg(knob, psi);
        ef += (e - ef) * (h / m.tf.max(h)).min(1.0);
        let c = (m.k * ef + b).clamp(-25.0, 25.0);
        if m.tau > 0.0 {
            p = (c - phi) / m.tau;
        } else {
            p += (m.wn * m.wn * (c - phi) - 2.0 * m.z * m.wn * p) * h;
        }
        phi += p * h;
        hdg += (G * phi.to_radians().tan() / v).to_degrees() * h;
        yaw = m.ky_p * p + m.ky_phi * phi;
        x += v * trk.to_radians().sin() * h;
        if gust_kts > 0.0 {
            gust_state ^= gust_state >> 12;
            gust_state ^= gust_state << 25;
            gust_state ^= gust_state >> 27;
            let u = (gust_state.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 11) as f64 / (1u64 << 53) as f64;
            gust += -gust * h / 2.0 + gust_kts * (3.0 * h).sqrt() * (2.0 * u - 1.0);
        }
        if t > t_end - 60.0 {
            worst_bank = worst_bank.max(phi.abs());
        }
        t += h;
    }
    Final { x_at_minimums_m: x, worst_bank_last_min_deg: worst_bank, sets_per_min: f64::from(sets) / (t_end / 60.0) }
}

/// The pilot's complaint, on the plant: tonight's knob (a quarter degree
/// every second, about 24 a minute on the tape) rocked the Evolution +-14
/// degrees of bank on a 10 s period (1791826866), and its fitted plant,
/// with the yaw the heading carries, rings the same way; the whole degree,
/// set no oftener than every ten seconds, holds the wings near level.
#[test]
fn the_quarter_degree_knob_rang_the_evolution_and_the_pilots_does_not() {
    let evo = MODES[3];
    let old = fly(evo, Law::QuarterEverySecond, 40.0, 0.0, 0.0, 0.0);
    let new = fly(evo, Law::Pilot, 40.0, 0.0, 0.0, 0.0);
    assert!(old.worst_bank_last_min_deg > 15.0 && old.sets_per_min > 30.0, "{old:?}");
    assert!(new.worst_bank_last_min_deg < 3.0 && new.sets_per_min < 3.0, "{new:?}");
}

/// Every fitted heading mode, from 40 m right, 80 m left and on the line,
/// calm, in a 15 kt crosswind and in 1 kt gusts on 10, the mode settling
/// 2 degrees either side of its knob or on it (the 172's 1.1 to 1.9 on
/// 2026-09-12): at minimums within 18 m of the line (8.5 m on the SR22 to 17
/// on the King Air: a whole degree is a metre a second at 110 kt, held ten
/// seconds), the knob set at most six times a minute.
#[test]
fn every_fitted_heading_mode_reaches_minimums_near_the_line() {
    for m in MODES {
        for (xw, gust) in [(0.0, 0.0), (15.0, 0.0), (10.0, 1.0)] {
            for x0 in [40.0, -80.0, 0.0] {
                for bias in [0.0, 2.0, -2.0] {
                    let f = fly(m, Law::Pilot, x0, xw, gust, bias);
                    assert!(
                        f.x_at_minimums_m.abs() < 18.0 && f.sets_per_min <= 6.5,
                        "{} from {x0} in {xw}/{gust} kt, bias {bias}: {f:?}",
                        m.name
                    );
                }
            }
        }
    }
}
