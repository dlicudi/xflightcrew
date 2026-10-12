//! The pilot on plants FITTED to the 2026-10-12 tapes: the hand-flown
//! segment from minimums to the wheels (`segment`, the pitch, roll and
//! speed fits together), and the flare from each recorded gate (`fly`).
//!
//! PITCH: the attitude flare on plants FITTED to six airframes' recorded flares
//! (2026-10-12, multiple shooting from the gate to the wheels on the
//! recorded column and lever, 0.11 to 0.25 deg rms on theta and gamma). One
//! shape fits all six: the column sets an angle of attack through the
//! short period, lift is the angle of attack above its zero-lift angle times
//! the dynamic pressure (more with the power's slipstream and in ground
//! effect), and the lever's drag decides the speed. The plant the law was
//! first tested on had a lift slope five times too steep and a fifth of the
//! idle deceleration, and could not show the windy evening's sag.

use super::attitude::*;
use super::bank::BankHold;
use super::energy::EnergyThrottle;
use super::line::bank_for_line;

#[test]
fn the_flare_height_is_the_sink_times_the_response() {
    assert!((gate_ft(-360.0, RESPONSE_S) - 18.0).abs() < 1e-9, "the 172 on its path");
    assert!((gate_ft(-750.0, RESPONSE_S) - 37.5).abs() < 1e-9, "a jet at 140 kt");
    assert_eq!(gate_ft(-100.0, RESPONSE_S), 6.0, "never below the floor");
}

#[test]
fn the_sink_wanted_shrinks_with_the_height_left() {
    assert_eq!(vs_target_fpm(40.0, -360.0, 12.0), -360.0, "above the gate: the path");
    assert_eq!(vs_target_fpm(12.0, -360.0, 12.0), -360.0);
    assert!((vs_target_fpm(6.0, -360.0, 12.0) + 172.08).abs() < 1e-9, "smoothstep: {}", vs_target_fpm(6.0, -360.0, 12.0));
    // No kink at either end: just inside each, the target barely moves.
    assert!((vs_target_fpm(2.1, -360.0, 12.0) - HOLDOFF_VS_FPM).abs() < 1.0);
    assert!((vs_target_fpm(11.9, -360.0, 12.0) + 360.0).abs() < 1.0);
    assert_eq!(vs_target_fpm(2.0, -360.0, 12.0), HOLDOFF_VS_FPM, "the hold-off from 2 ft");
    assert_eq!(vs_target_fpm(0.0, -360.0, 12.0), HOLDOFF_VS_FPM);
    assert_eq!(vs_target_fpm(5.0, -50.0, 6.0), HOLDOFF_VS_FPM, "never asks a climb");
}

const G: f64 = 9.806_65;
const KT: f64 = 0.514_444;

/// One airframe's fit and the gate it was recorded from.
#[derive(Clone, Copy)]
struct Fit {
    name: &'static str,
    /// Short period: stiffness (1/s^2) and damping (1/s) at the gate's speed.
    w2: f64,
    cq: f64,
    /// Angle of attack per unit of column, deg; the ground effect's column
    /// (ge) and the power's pitch (kl, per unit of lever) against it.
    ke: f64,
    ge: f64,
    kl: f64,
    /// Lift: load factor per deg above the zero-lift angle `a0` at 80 kt,
    /// more with the lever (`kp`) and in ground effect (`kg`).
    c: f64,
    a0: f64,
    kp: f64,
    kg: f64,
    /// Speed rate, kt/s: idle (`s0`) and per unit of lever (`s1`).
    s0: f64,
    s1: f64,
    /// The column's angle-of-attack offset.
    aoff: f64,
    /// The pitch trim the autopilot left at the gate, elevator: the column
    /// applied is the law's plus this.
    trim: f64,
    /// Transport delay, s.
    delay: f64,
    /// What the profile declares to the law.
    ground_pull: f64,
    pull: f64,
    push: f64,
    /// The recorded gate: height, pitch, path, speed, pitch rate.
    h0: f64,
    th0: f64,
    g0: f64,
    v0: f64,
    q0: f64,
    /// The lever as flown, by height: at 40, 30, 20, 10, 5, 0 ft.
    lever: [f64; 6],
    /// Its roll with the servos out.
    roll: Roll,
}

/// ROLL: one shape fits the 59 servos-out arrests of 2026-10-12 on six
/// airframes (R2 0.91 to 0.99, 0.10 to 0.31 deg rms over 3 s windows): the
/// roll rate follows the aileron through a first-order roll mode, `p' =
/// (kp (a(t - delay) - a0) - p) / tau`, fitted per airframe with the window's
/// starting rate free and `a0` per landing (the crosswind's dihedral).
/// Adding a spiral term (the bank returning by itself) moved kp under 20%
/// and R2 by 0.01: left out.
#[derive(Clone, Copy, Debug)]
struct Roll {
    /// Roll rate per unit of aileron, deg/s; the roll mode, s; the delay, s;
    /// the aileron that holds the wings level.
    kp: f64,
    tau: f64,
    delay: f64,
    a0: f64,
}

const FITS: [Fit; 6] = [
    // Lancair Evolution, LEMG 13 (1791801818, 1791802111, 1791815852).
    Fit { name: "Evolution", w2: 19.71, cq: 3.703, ke: 12.87, ge: 0.0222, kl: -0.1628, c: 0.0755, a0: -10.56, kp: 0.1541, kg: 0.1019,
          s0: -2.876, s1: 2.934, aoff: -1.48, trim: -0.0455, delay: 0.10, ground_pull: 0.0, pull: 0.8, push: 0.05,
          h0: 51.8, th0: -5.50, g0: -3.91, v0: 92.7, q0: -0.44, lever: [0.293, 0.293, 0.293, 0.068, 0.012, 0.0],
          // 6 arrests, 54 windows.
          roll: Roll { kp: 14.30, tau: 0.45, delay: 0.10, a0: -0.040 } },
    // Cirrus SR22, LEMG 13 (1791812680, 1791813026, 1791814190, 1791814508).
    Fit { name: "SR22", w2: 6.920, cq: 1.455, ke: 11.15, ge: 0.2165, kl: -0.1656, c: 0.0982, a0: -11.31, kp: 0.1917, kg: 0.0304,
          s0: -3.084, s1: 8.160, aoff: -2.56, trim: 0.0398, delay: 0.02, ground_pull: 0.5, pull: 0.8, push: 0.05,
          h0: 49.9, th0: -3.77, g0: -2.56, v0: 78.9, q0: 0.04, lever: [0.255, 0.255, 0.255, 0.114, 0.0, 0.0],
          // 13 arrests, 78 windows.
          roll: Roll { kp: 14.19, tau: 0.60, delay: 0.05, a0: -0.038 } },
    // King Air C90B, LEMG 13 (1791799361, 1791800024, 1791815553).
    Fit { name: "King Air", w2: 8.404, cq: 1.171, ke: 14.75, ge: -0.0454, kl: -0.2320, c: 0.0513, a0: -9.364, kp: 0.1006, kg: 0.0092,
          s0: -4.455, s1: 5.417, aoff: -0.13, trim: -0.102, delay: 0.02, ground_pull: 0.25, pull: 0.8, push: 0.10,
          h0: 68.7, th0: -4.12, g0: -4.40, v0: 113.3, q0: 0.26, lever: [0.495, 0.495, 0.444, 0.213, 0.092, 0.0],
          // 4 arrests, 29 windows.
          roll: Roll { kp: 19.24, tau: 0.60, delay: 0.10, a0: -0.061 } },
    // Baron 58, LEMG 13 (1791802802, 1791803088, 1791816184).
    Fit { name: "Baron", w2: 19.61, cq: 3.304, ke: 12.47, ge: 0.0906, kl: -0.2359, c: 0.0581, a0: -7.760, kp: 0.3633, kg: 0.0992,
          s0: -3.096, s1: 3.873, aoff: -2.22, trim: 0.0945, delay: 0.02, ground_pull: 0.3, pull: 0.8, push: 0.10,
          h0: 60.3, th0: -2.85, g0: -3.69, v0: 104.2, q0: 0.12, lever: [0.512, 0.512, 0.48, 0.359, 0.275, 0.09],
          // 6 arrests, 55 windows.
          roll: Roll { kp: 18.18, tau: 0.30, delay: 0.15, a0: -0.004 } },
    // Cessna 172 G1000, LEMG 13 (1791814879).
    Fit { name: "C172", w2: 10.31, cq: 1.502, ke: 11.34, ge: 0.0255, kl: -0.2245, c: 0.2000, a0: -7.534, kp: -0.0186, kg: -0.0896,
          s0: -2.935, s1: 4.045, aoff: 0.30, trim: -0.1572, delay: 0.02, ground_pull: 0.0, pull: 0.5, push: 0.10,
          h0: 49.7, th0: -3.28, g0: -2.84, v0: 67.4, q0: 0.34, lever: [0.515, 0.473, 0.359, 0.172, 0.068, 0.003],
          // 25 arrests, 171 windows.
          roll: Roll { kp: 18.26, tau: 0.45, delay: 0.0, a0: 0.009 } },
    // Cirrus SF50, LEMG 13 (1791805150, 1791805443, 1791809604; fitted 2026-10-12
    // as the five above, 0.21 deg rms; the gate and lever are 1791809604's).
    Fit { name: "SF50", w2: 9.074, cq: 2.864, ke: 17.65, ge: 0.0169, kl: 0.0578, c: 0.0708, a0: -10.14, kp: 0.0118, kg: 0.1085,
          s0: -2.256, s1: 5.837, aoff: 2.83, trim: -0.086, delay: 0.10, ground_pull: 0.0, pull: 0.8, push: 0.05,
          h0: 49.9, th0: -1.62, g0: -2.62, v0: 92.0, q0: 0.17, lever: [0.203, 0.203, 0.181, 0.075, 0.0, 0.0],
          // 3 arrests, 25 windows.
          roll: Roll { kp: 11.83, tau: 0.60, delay: 0.15, a0: -0.002 } },
];

fn shape(h: f64) -> f64 {
    (1.0 - h / 18.0).clamp(0.0, 1.0).powi(2)
}

impl Fit {
    fn lever_at(&self, h: f64) -> f64 {
        const AT: [f64; 6] = [40.0, 30.0, 20.0, 10.0, 5.0, 0.0];
        if h >= AT[0] {
            return self.lever[0];
        }
        for i in 1..6 {
            if h >= AT[i] {
                let k = (h - AT[i]) / (AT[i - 1] - AT[i]);
                return self.lever[i] + k * (self.lever[i - 1] - self.lever[i]);
            }
        }
        self.lever[5]
    }
}

#[derive(Debug)]
struct Landing {
    /// The sink at the wheels, and the worst in the last 2.5 s (the
    /// scorecard's window), fpm.
    wheels_fpm: f64,
    scored_fpm: f64,
    balloon_ft: f64,
}

/// A path disturbance, deg/s, at time `t`: none, or one gust that shallows
/// the path by `deg` over a second once below `below_ft`.
#[derive(Clone, Copy)]
enum Air {
    Calm,
    Gust { below_ft: f64, deg: f64 },
}

fn fly(f: Fit, air: Air) -> Landing {
    let (dt, tick) = (0.01, 0.2);
    let delay = f.delay.max(0.02);
    let mut law = AttitudeFlare::default();
    let (mut th, mut q, mut gam, mut v, mut h) = (f.th0, f.q0, f.g0, f.v0, f.h0);
    let mut queue: Vec<(f64, f64)> = Vec::new();
    let (mut t, mut next_tick, mut e_app) = (0.0, 0.0, f.trim);
    let (mut low, mut balloon) = (h, 0.0_f64);
    let mut gust_from: Option<f64> = None;
    let mut sinks: Vec<(f64, f64)> = Vec::new();
    while t < 40.0 {
        if t >= next_tick - 1e-9 {
            let o = law.step(
                In { radio_ft: h, theta_deg: th, alpha_deg: th - gam, q_dps: q, tas_kts: v, dt: tick, ground_pull: f.ground_pull, path: None },
                f.pull,
                f.push,
            );
            queue.push((t + delay, o.elev + f.trim));
            next_tick += tick;
        }
        while queue.first().is_some_and(|(at, _)| *at <= t + 1e-9) {
            e_app = queue.remove(0).1;
        }
        let lev = f.lever_at(h);
        let al = th - gam;
        let ae = f.aoff + f.ke * (e_app - f.ge * shape(h) - f.kl * lev);
        let qd = f.w2 * (v / f.v0).powi(2) * (ae - al) - f.cq * (v / f.v0) * q;
        let n = f.c * (v / 80.0).powi(2) * (al - f.a0) * (1.0 + f.kp * lev) * (1.0 + f.kg * shape(h));
        let gust = match air {
            Air::Calm => 0.0,
            Air::Gust { below_ft, deg } => {
                if gust_from.is_none() && h < below_ft {
                    gust_from = Some(t);
                }
                if gust_from.is_some_and(|t0| t - t0 < 1.0) { deg } else { 0.0 }
            }
        };
        let gd = (G / (v * KT) * (n - gam.to_radians().cos())).to_degrees() + gust;
        let vd = f.s0 + f.s1 * lev - G * gam.to_radians().sin() / KT;
        q += qd * dt;
        th += q * dt;
        gam += gd * dt;
        v += vd * dt;
        h += v * 1.687_81 * gam.to_radians().sin() * dt;
        low = low.min(h);
        balloon = balloon.max(h - low);
        let sink = -v * 101.268_591 * gam.to_radians().sin();
        sinks.push((t, sink));
        t += dt;
        if h <= 0.0 {
            let scored = sinks.iter().filter(|(ts, _)| t - ts <= 2.5).map(|s| s.1).fold(f64::MIN, f64::max);
            return Landing { wheels_fpm: sink, scored_fpm: scored, balloon_ft: balloon };
        }
    }
    panic!("{}: no landing in 40 s", f.name);
}

/// Calm air, every fitted airframe from its own recorded gate: inside the
/// 100 to 180 sweet spot's neighbourhood at the wheels, never a balloon.
#[test]
fn every_fitted_airframe_lands_soft_in_calm_air() {
    for f in FITS {
        let l = fly(f, Air::Calm);
        eprintln!("{}: {l:?}", f.name);
        assert!(
            (50.0..=180.0).contains(&l.wheels_fpm) && l.scored_fpm < 200.0 && l.balloon_ft < 1.0,
            "{}: {l:?}",
            f.name
        );
    }
}

/// A gust that shallows the path 1.5 deg for a second, above the gate and
/// below it: never pushed back down into a hard landing, never a balloon
/// (the SR22 met the runway at 441 fpm from one at 29 ft, 1791813026). The
/// same gust ending at 10 ft is a limit, not a test: the lift it lent is
/// gone with the wheels a second away (211 to 250 fpm scored, under the
/// previous law 240 on the Evolution).
#[test]
fn a_gust_that_shallows_the_path_is_never_flown_into_the_runway() {
    for f in FITS {
        for below_ft in [35.0, 20.0] {
            let l = fly(f, Air::Gust { below_ft, deg: 1.5 });
            eprintln!("{} gust below {below_ft}: {l:?}", f.name);
            assert!(
                l.scored_fpm < crate::scorecard::TOUCHDOWN_MAX_SINK_FPM && l.balloon_ft < 1.0,
                "{} gust below {below_ft}: {l:?}",
                f.name
            );
        }
    }
}

/// The fits half again either way (the column's authority, the short
/// period's damping, the ground effect's column): no fitted number is the
/// law's, so none may decide a landing.
#[test]
fn the_fits_moved_either_way_still_land() {
    for f in FITS {
        for (what, g) in [
            ("ke x0.6", Fit { ke: f.ke * 0.6, ..f }),
            ("ke x1.6", Fit { ke: f.ke * 1.6, ..f }),
            ("cq x0.6", Fit { cq: f.cq * 0.6, ..f }),
            ("ge +0.2", Fit { ge: f.ge + 0.2, ..f }),
        ] {
            let l = fly(g, Air::Calm);
            eprintln!("{} {what}: {l:?}", f.name);
            assert!(
                l.scored_fpm < crate::scorecard::TOUCHDOWN_MAX_SINK_FPM && l.wheels_fpm > 30.0 && l.balloon_ft < 1.0,
                "{} {what}: {l:?}",
                f.name
            );
        }
    }
}

/// A deterministic Gaussian source (xorshift64*, Box-Muller): the gusts.
struct Noise(u64);

impl Noise {
    fn uniform(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        ((self.0.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    fn gauss(&mut self) -> f64 {
        let (u, v) = (self.uniform(), self.uniform());
        (-2.0 * u.ln()).sqrt() * (2.0 * std::f64::consts::PI * v).cos()
    }
}

/// One hand-flown segment: from `h0` ft, `above_ft` over the 3 degree path,
/// the painted centreline `x0` m to the left (right of it positive), a
/// crosswind from the right of `xwind_kts` with gusts of `gust_kts` (sd,
/// over 2 s: the gustiest final of 2026-10-12 read 2.7, most under 1), and
/// the autopilot's trim `trim_off` of column out at the disconnect.
#[derive(Clone, Copy, Debug)]
struct Case {
    h0: f64,
    above_ft: f64,
    x0: f64,
    xwind_kts: f64,
    gust_kts: f64,
    trim_off: f64,
    seed: u64,
}

const MINIMUMS: Case =
    Case { h0: 200.0, above_ft: 0.0, x0: 0.0, xwind_kts: 0.0, gust_kts: 0.0, trim_off: 0.0, seed: 1 };

#[derive(Debug)]
#[allow(dead_code)]
struct Segment {
    wheels_fpm: f64,
    scored_fpm: f64,
    balloon_ft: f64,
    x_m: f64,
    worst_bank_deg: f64,
    kp: f64,
    slowest_kts: f64,
}

/// Fly `f` from minimums to the wheels as the loop does: the attitude law
/// on the path then the flare, the ailerons on the bank toward the
/// centreline, the energy throttle on the speed down to 40 ft and the
/// recorded retard's shape from there, each on the loop's 0.2 s tick
/// through the fit's own delay. Trimmed on the path at the gate's speed.
fn segment(f: Fit, c: Case) -> Segment {
    let (dt, tick) = (0.01, 0.2);
    let r = f.roll;
    let gp = -3.0_f64;
    let mut v = f.v0;
    let lev0 = ((G * gp.to_radians().sin() / KT - f.s0) / f.s1).clamp(0.0, 1.0);
    let al0 = f.a0 + gp.to_radians().cos() / (f.c * (v / 80.0).powi(2) * (1.0 + f.kp * lev0));
    let trim = (al0 - f.aoff) / f.ke + f.kl * lev0 + c.trim_off;
    let (mut th, mut q, mut gam, mut h) = (gp + al0, 0.0, gp, c.h0);
    let mut path_h = c.h0 - c.above_ft;
    let (mut lever, mut lever_40, mut prev_v) = (lev0, None::<f64>, v);
    let (mut law, mut energy, mut bank) =
        (AttitudeFlare::default(), EnergyThrottle::default(), BankHold::default());
    let crab = |xw: f64, v: f64| (xw / v).clamp(-0.9, 0.9).asin().to_degrees();
    let mut xw = c.xwind_kts;
    let mut psi = crab(xw, v);
    let (mut x, mut phi, mut p, mut gust) = (c.x0, 0.0_f64, 0.0_f64, 0.0);
    let mut noise = Noise(0x9e37_79b9_7f4a_7c15 ^ c.seed.wrapping_mul(0x2545_f491));
    let (mut elev_q, mut ail_q) = (Vec::<(f64, f64)>::new(), Vec::<(f64, f64)>::new());
    let (mut e_app, mut ail) = (trim, r.a0);
    let (mut t, mut next_tick) = (0.0, 0.0);
    let (mut low, mut balloon, mut worst_bank, mut slowest) = (f64::MAX, 0.0_f64, 0.0_f64, v);
    let mut sinks: Vec<(f64, f64)> = Vec::new();
    while t < 150.0 {
        if t >= next_tick - 1e-9 {
            let vs_fpm = v * 101.268_591 * gam.to_radians().sin();
            let o = law.step(
                In {
                    radio_ft: h,
                    theta_deg: th,
                    alpha_deg: th - gam,
                    q_dps: q,
                    tas_kts: v,
                    dt: tick,
                    ground_pull: f.ground_pull,
                    path: Some(Path { vs_fpm: v * 101.268_591 * gp.to_radians().sin(), above_ft: h - path_h }),
                },
                f.pull,
                f.push,
            );
            elev_q.push((t + f.delay.max(0.02), o.elev + trim));
            lever = if h > 40.0 {
                energy.step(super::energy::In {
                    ias_kts: v,
                    accel_kts_s: (v - prev_v) / tick,
                    vs_fpm,
                    target_kts: f.v0,
                    path_vs_fpm: o.vs_target_fpm,
                    thr_now: lever,
                    dt: tick,
                })
            } else {
                *lever_40.get_or_insert(lever) * f.lever_at(h) / f.lever[0].max(1e-6)
            };
            prev_v = v;
            let side = v * KT * (psi - crab(xw, v)).to_radians().sin();
            ail_q.push((t + r.delay, bank.step(phi, bank_for_line(x, side, phi, h), tick)));
            next_tick += tick;
        }
        while elev_q.first().is_some_and(|(at, _)| *at <= t + 1e-9) {
            e_app = elev_q.remove(0).1;
        }
        while ail_q.first().is_some_and(|(at, _)| *at <= t + 1e-9) {
            ail = ail_q.remove(0).1;
        }
        let al = th - gam;
        let ae = f.aoff + f.ke * (e_app - f.ge * shape(h) - f.kl * lever);
        let qd = f.w2 * (v / f.v0).powi(2) * (ae - al) - f.cq * (v / f.v0) * q;
        let n = f.c * (v / 80.0).powi(2) * (al - f.a0) * (1.0 + f.kp * lever) * (1.0 + f.kg * shape(h));
        let gd = (G / (v * KT) * (n - gam.to_radians().cos())).to_degrees();
        let vd = f.s0 + f.s1 * lever - G * gam.to_radians().sin() / KT;
        q += qd * dt;
        th += q * dt;
        gam += gd * dt;
        v += vd * dt;
        h += v * 1.687_81 * gam.to_radians().sin() * dt;
        path_h += v * 1.687_81 * gp.to_radians().sin() * dt;
        p += (r.kp * (ail - r.a0) - p) / r.tau * dt;
        phi += p * dt;
        psi += (G * phi.to_radians().tan() / (v * KT)).to_degrees() * dt;
        if c.gust_kts > 0.0 {
            gust += -gust * dt / 2.0 + c.gust_kts * dt.sqrt() * noise.gauss();
        }
        xw = c.xwind_kts + gust;
        x += v * KT * (psi - crab(xw, v)).to_radians().sin() * dt;
        if h < 50.0 {
            low = low.min(h);
            balloon = balloon.max(h - low);
        }
        worst_bank = worst_bank.max(phi.abs());
        slowest = slowest.min(v);
        let sink = -v * 101.268_591 * gam.to_radians().sin();
        sinks.push((t, sink));
        t += dt;
        if h <= 0.0 {
            let scored = sinks.iter().filter(|(ts, _)| t - ts <= 2.5).map(|s| s.1).fold(f64::MIN, f64::max);
            return Segment {
                wheels_fpm: sink,
                scored_fpm: scored,
                balloon_ft: balloon,
                x_m: x,
                worst_bank_deg: worst_bank,
                kp: bank.kp(),
                slowest_kts: slowest,
            };
        }
    }
    panic!("{}: no landing in 150 s from {c:?}", f.name);
}

/// Every fitted airframe from minimums (the coupled 172's 200 ft, the
/// visual 300), the line 8 to 23 m away either side (the coupled 172's
/// localiser at minimums, 2026-10-12), calm and in a 15 kt crosswind: on
/// the centreline at the wheels within 3 m, 60 to 200 fpm, no balloon, the
/// bank within a degree of its 5 degree cap.
#[test]
fn every_fitted_airframe_hand_flies_from_minimums_to_the_centreline() {
    for f in FITS {
        for h0 in [200.0, 300.0] {
            for x0 in [23.0, 8.0, 0.0, -8.0, -23.0] {
                for xwind_kts in [0.0, 15.0, -15.0] {
                    let c = Case { h0, x0, xwind_kts, ..MINIMUMS };
                    let s = segment(f, c);
                    eprintln!("{} {h0} {x0} {xwind_kts}: {s:?}", f.name);
                    assert!(
                        s.x_m.abs() < 3.0
                            && (60.0..=200.0).contains(&s.wheels_fpm)
                            && s.scored_fpm < crate::scorecard::TOUCHDOWN_MAX_SINK_FPM
                            && s.balloon_ft < 1.0
                            && s.worst_bank_deg < 6.0,
                        "{} {c:?}: {s:?}",
                        f.name
                    );
                }
            }
        }
    }
}

/// Gusts across the final (1 kt sd over 2 s, five seeds, on a 10 kt
/// crosswind), and off the path or out of trim at the disconnect: still on
/// the line within 3 m and inside the touchdown limit.
#[test]
fn the_segment_holds_in_gusts_off_the_path_and_out_of_trim() {
    for f in FITS {
        let mut cases = Vec::new();
        for seed in 1..=5 {
            for x0 in [23.0, -23.0] {
                cases.push(Case { x0, xwind_kts: 10.0, gust_kts: 1.0, seed, ..MINIMUMS });
            }
        }
        for above_ft in [25.0, -25.0] {
            cases.push(Case { above_ft, ..MINIMUMS });
        }
        for trim_off in [0.05, -0.05] {
            cases.push(Case { trim_off, x0: 15.0, ..MINIMUMS });
        }
        for c in cases {
            let s = segment(f, c);
            eprintln!("{} {c:?}: {s:?}", f.name);
            assert!(
                s.x_m.abs() < 3.0
                    && s.wheels_fpm > 40.0
                    && s.scored_fpm < crate::scorecard::TOUCHDOWN_MAX_SINK_FPM
                    && s.balloon_ft < 1.0,
                "{} {c:?}: {s:?}",
                f.name
            );
        }
    }
}

/// The roll response the segment measures (`bank::RollResponse`) lands near
/// each airframe's fit, and the bank holds the line on rolls half and twice
/// the fitted rate, twice the roll mode and 0.3 s late: no fitted roll
/// number is the law's.
#[test]
fn the_roll_response_is_measured_not_declared() {
    for f in FITS {
        let s = segment(f, Case { x0: 23.0, xwind_kts: 10.0, ..MINIMUMS });
        eprintln!("{}: measured {:.1} against {:.1}", f.name, s.kp, f.roll.kp);
        assert!(
            (0.5..=1.6).contains(&(s.kp / f.roll.kp)),
            "{}: measured {:.1} against the fit's {:.1}",
            f.name,
            s.kp,
            f.roll.kp
        );
        for (what, roll) in [
            ("kp x0.5", Roll { kp: f.roll.kp * 0.5, ..f.roll }),
            ("kp x2", Roll { kp: f.roll.kp * 2.0, ..f.roll }),
            ("tau x2", Roll { tau: f.roll.tau * 2.0, ..f.roll }),
            ("delay 0.3", Roll { delay: 0.3, ..f.roll }),
        ] {
            let g = Fit { roll, ..f };
            for x0 in [23.0, -8.0] {
                let s = segment(g, Case { x0, xwind_kts: 15.0, ..MINIMUMS });
                eprintln!("{} {what} from {x0}: {s:?}", f.name);
                assert!(s.x_m.abs() < 3.0 && s.worst_bank_deg < 7.0, "{} {what} from {x0}: {s:?}", f.name);
            }
        }
    }
}

/// The A330's four recorded arrest gates at LEMG 30 (#180, sorties
/// 1790633832 and 1790634412, 2026-09-28) on its sidestick's roll-rate
/// plant (fitted 11 to 13 deg/s per unit after 0.25 to 0.37 s; 9 to 20 and
/// up to 0.6 s bound it), flown by the measured bank hold where a declared
/// plant (`arrest_roll:`) flew them: (x m, sideways m/s, bank, s to the
/// wheels, ft). Within 2 m of the line at the wheels; the floated gate (8 m
/// right crossing left at 3.8 m/s, banked 3.4 the wrong way) is stopped by
/// the 5 degree cap, not the law: 8 to 13 m past the line here, where the
/// declared plant's law went 7.2 to 12.9 and the stiff law 8.3 to 13.5 on
/// the same five plants (2026-10-12, the old laws replayed).
#[test]
fn the_bank_hold_lands_the_a330s_recorded_gates() {
    let gates = [
        (0.55, 0.25, -1.47, 17.4, 87.9),
        (-7.5, 1.23, 5.19, 17.9, 86.4),
        (7.96, -3.8, -3.37, 17.3, 86.9),
        (1.07, 0.21, 0.6, 20.3, 90.8),
    ];
    for (i, (x0, v0, phi0, t_end, radio0)) in gates.into_iter().enumerate() {
        for (k, dead) in [(13.0, 0.35), (20.0, 0.35), (13.0, 0.6), (20.0, 0.6), (9.0, 0.35)] {
            let (tick, h) = (0.2, 0.01);
            let mut bank = BankHold::default();
            let (mut x, mut v, mut phi, mut p) = (x0, v0, phi0, 0.0_f64);
            let mut acc = 9.81 * f64::to_radians(phi0).tan();
            let mut written: Vec<(f64, f64)> = Vec::new();
            let (mut t, mut next, mut worst) = (0.0, 0.0, 0.0_f64);
            while t < t_end {
                if t + 1e-9 >= next {
                    next += tick;
                    let radio = radio0 * (1.0 - t / t_end);
                    written.push((t, bank.step(phi, bank_for_line(x, v, phi, radio), tick)));
                }
                let a = written.iter().take_while(|w| w.0 <= t - dead + 1e-9).last().map_or(0.0, |w| w.1);
                p += (k * a - p) * h / 0.07;
                phi += p * h;
                acc += (9.81 * phi.to_radians().tan() - acc) * h / 0.2;
                v += acc * h;
                x += v * h;
                t += h;
                worst = worst.max(-x * x0.signum());
            }
            eprintln!("gate {i} at {k}/{dead}: {x:.2} m, overshoot {worst:.2}");
            assert!(x.abs() < 2.0, "gate {i} at {k}/{dead}: lands {x:.1} m off");
            assert!(worst < if i == 2 { 13.0 } else { 2.0 }, "gate {i} at {k}/{dead}: overshoots {worst:.1}");
        }
    }
}
