//! The attitude flare on plants FITTED to five airframes' recorded flares
//! (2026-10-12, multiple shooting from the gate to the wheels on the
//! recorded column and lever, 0.11 to 0.25 deg rms on theta and gamma). One
//! shape fits all five: the column sets an angle of attack through the
//! short period, lift is the angle of attack above its zero-lift angle times
//! the dynamic pressure (more with the power's slipstream and in ground
//! effect), and the lever's drag decides the speed. The plant the law was
//! first tested on had a lift slope five times too steep and a fifth of the
//! idle deceleration, and could not show the windy evening's sag.

use super::*;

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
}

const FITS: [Fit; 5] = [
    // Lancair Evolution, LEMG 13 (1791801818, 1791802111, 1791815852).
    Fit { name: "Evolution", w2: 19.71, cq: 3.703, ke: 12.87, ge: 0.0222, kl: -0.1628, c: 0.0755, a0: -10.56, kp: 0.1541, kg: 0.1019,
          s0: -2.876, s1: 2.934, aoff: -1.48, trim: -0.0455, delay: 0.10, ground_pull: 0.0, pull: 0.8, push: 0.05,
          h0: 51.8, th0: -5.50, g0: -3.91, v0: 92.7, q0: -0.44, lever: [0.293, 0.293, 0.293, 0.068, 0.012, 0.0] },
    // Cirrus SR22, LEMG 13 (1791812680, 1791813026, 1791814190, 1791814508).
    Fit { name: "SR22", w2: 6.920, cq: 1.455, ke: 11.15, ge: 0.2165, kl: -0.1656, c: 0.0982, a0: -11.31, kp: 0.1917, kg: 0.0304,
          s0: -3.084, s1: 8.160, aoff: -2.56, trim: 0.0398, delay: 0.02, ground_pull: 0.5, pull: 0.8, push: 0.05,
          h0: 49.9, th0: -3.77, g0: -2.56, v0: 78.9, q0: 0.04, lever: [0.255, 0.255, 0.255, 0.114, 0.0, 0.0] },
    // King Air C90B, LEMG 13 (1791799361, 1791800024, 1791815553).
    Fit { name: "King Air", w2: 8.404, cq: 1.171, ke: 14.75, ge: -0.0454, kl: -0.2320, c: 0.0513, a0: -9.364, kp: 0.1006, kg: 0.0092,
          s0: -4.455, s1: 5.417, aoff: -0.13, trim: -0.102, delay: 0.02, ground_pull: 0.25, pull: 0.8, push: 0.10,
          h0: 68.7, th0: -4.12, g0: -4.40, v0: 113.3, q0: 0.26, lever: [0.495, 0.495, 0.444, 0.213, 0.092, 0.0] },
    // Baron 58, LEMG 13 (1791802802, 1791803088, 1791816184).
    Fit { name: "Baron", w2: 19.61, cq: 3.304, ke: 12.47, ge: 0.0906, kl: -0.2359, c: 0.0581, a0: -7.760, kp: 0.3633, kg: 0.0992,
          s0: -3.096, s1: 3.873, aoff: -2.22, trim: 0.0945, delay: 0.02, ground_pull: 0.3, pull: 0.8, push: 0.10,
          h0: 60.3, th0: -2.85, g0: -3.69, v0: 104.2, q0: 0.12, lever: [0.512, 0.512, 0.48, 0.359, 0.275, 0.09] },
    // Cessna 172 G1000, LEMG 13 (1791814879).
    Fit { name: "C172", w2: 10.31, cq: 1.502, ke: 11.34, ge: 0.0255, kl: -0.2245, c: 0.2000, a0: -7.534, kp: -0.0186, kg: -0.0896,
          s0: -2.935, s1: 4.045, aoff: 0.30, trim: -0.1572, delay: 0.02, ground_pull: 0.0, pull: 0.5, push: 0.10,
          h0: 49.7, th0: -3.28, g0: -2.84, v0: 67.4, q0: 0.34, lever: [0.515, 0.473, 0.359, 0.172, 0.068, 0.003] },
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
                In { radio_ft: h, theta_deg: th, alpha_deg: th - gam, q_dps: q, tas_kts: v, dt: tick, ground_pull: f.ground_pull },
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
