//! The pilot on plants FITTED to recorded flights: each airframe is first
//! IDENTIFIED as `xfc identify` would fly it (doublets at a safe height,
//! `pilot::ident`), its loops designed from that measurement by the one
//! rule (`pilot::response::design`), then flown from minimums to the wheels
//! (`segment`: pitch, roll, yaw and speed together) and through the flare
//! from each recorded gate (`fly`). No number of the law is a plant's.
//!
//! PITCH: six airframes' recorded flares (2026-10-12, multiple shooting from
//! the gate to the wheels on the recorded column and lever, 0.11 to 0.25 deg
//! rms on theta and gamma). One shape fits all six: the column sets an angle
//! of attack through the short period, lift is the angle of attack above its
//! zero-lift angle times the dynamic pressure (more with the power's
//! slipstream and in ground effect), and the lever's drag decides the speed.
//! The 737's is its identification flight's (160 kt, flaps 15, 2026-10-12:
//! effectiveness 23 deg/s^2 a unit, damping 0.5 to 0.9, alpha spring 1.7,
//! path 0.6 to 0.8), scaled to 140 kt, its lift and speed from the book.
//!
//! ROLL: the 59 servos-out arrests of 2026-10-12 (R2 0.91 to 0.99): `p' =
//! (kp (a(t - delay) - a0) - p) / tau`; the 737's from its aileron doublets
//! (47 deg/s^2 a unit, tau 0.53 s).
//!
//! YAW: the 737's rudder doublets (2026-10-12: 15 deg/s^2 a unit, the
//! weathercock 1.8 deg/s^2 a degree, the side force 0.28 /s at 167 kt;
//! `r' = b u - damping r - stiffness beta`, `beta' = r - g/V sin(bank) -
//! path beta`, X-Plane's beta). The light aircraft's are SYNTHETIC, shaped as
//! the 737's: full rudder holds 13 to 17 degrees of slip (a 172 is
//! demonstrated to 15 kt across at 65 kt, 13 degrees of crab), a side force
//! of 0.12 /s. The roll each degree of slip takes (the dihedral) is
//! synthetic everywhere: the 737's single doublet could not separate it
//! from the rudder's own roll.

use super::attitude::*;
use super::bank::BankHold;
use super::energy::{EnergyThrottle, Tecs};
use super::ident::{self, Axis, EnergySample, Sample};
use super::line::{bank_for_line, period_s};
use super::response::{attitude_tc_s, design, Response};
use super::yaw::YawHold;
use super::Airframe;

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
const FPM_PER_KT: f64 = 101.268_591;

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
    /// The recorded gate: height, pitch, path, speed, pitch rate.
    h0: f64,
    th0: f64,
    g0: f64,
    v0: f64,
    q0: f64,
    /// The lever as flown, by height: at 40, 30, 20, 10, 5, 0 ft. `None`:
    /// no recorded flare (the 737's plant is its identification's).
    lever: Option<[f64; 6]>,
    roll: Roll,
    yaw: Yaw,
}

/// The roll plant: `p' = (kp (a(t - delay) - a0) - p) / tau`, plus the
/// dihedral's roll for the slip.
#[derive(Clone, Copy, Debug)]
struct Roll {
    kp: f64,
    tau: f64,
    delay: f64,
    a0: f64,
}

/// The yaw plant at the fit's speed, the rudder through the airframe's own
/// control delay (the pitch fit's), X-Plane's beta (the nose right of the
/// airpath positive), and the roll a degree of it gives (deg/s^2).
#[derive(Clone, Copy, Debug)]
struct Yaw {
    b: f64,
    damping: f64,
    stiffness: f64,
    path: f64,
    dihedral: f64,
}

/// A light aircraft's synthetic yaw (see the module's header).
const LIGHT_YAW: Yaw = Yaw { b: 50.0, damping: 1.2, stiffness: 3.0, path: 0.12, dihedral: 0.8 };
const TWIN_YAW: Yaw = Yaw { b: 40.0, damping: 1.0, stiffness: 3.0, path: 0.12, dihedral: 0.7 };

const FITS: [Fit; 7] = [
    // Lancair Evolution, LEMG 13 (1791801818, 1791802111, 1791815852).
    Fit { name: "Evolution", w2: 19.71, cq: 3.703, ke: 12.87, ge: 0.0222, kl: -0.1628, c: 0.0755, a0: -10.56, kp: 0.1541, kg: 0.1019,
          s0: -2.876, s1: 2.934, aoff: -1.48, trim: -0.0455, delay: 0.10,
          h0: 51.8, th0: -5.50, g0: -3.91, v0: 92.7, q0: -0.44, lever: Some([0.293, 0.293, 0.293, 0.068, 0.012, 0.0]),
          // 6 arrests, 54 windows.
          roll: Roll { kp: 14.30, tau: 0.45, delay: 0.10, a0: -0.040 }, yaw: LIGHT_YAW },
    // Cirrus SR22, LEMG 13 (1791812680, 1791813026, 1791814190, 1791814508).
    Fit { name: "SR22", w2: 6.920, cq: 1.455, ke: 11.15, ge: 0.2165, kl: -0.1656, c: 0.0982, a0: -11.31, kp: 0.1917, kg: 0.0304,
          s0: -3.084, s1: 8.160, aoff: -2.56, trim: 0.0398, delay: 0.02,
          h0: 49.9, th0: -3.77, g0: -2.56, v0: 78.9, q0: 0.04, lever: Some([0.255, 0.255, 0.255, 0.114, 0.0, 0.0]),
          // 13 arrests, 78 windows.
          roll: Roll { kp: 14.19, tau: 0.60, delay: 0.05, a0: -0.038 }, yaw: LIGHT_YAW },
    // King Air C90B, LEMG 13 (1791799361, 1791800024, 1791815553).
    Fit { name: "King Air", w2: 8.404, cq: 1.171, ke: 14.75, ge: -0.0454, kl: -0.2320, c: 0.0513, a0: -9.364, kp: 0.1006, kg: 0.0092,
          s0: -4.455, s1: 5.417, aoff: -0.13, trim: -0.102, delay: 0.02,
          h0: 68.7, th0: -4.12, g0: -4.40, v0: 113.3, q0: 0.26, lever: Some([0.495, 0.495, 0.444, 0.213, 0.092, 0.0]),
          // 4 arrests, 29 windows.
          roll: Roll { kp: 19.24, tau: 0.60, delay: 0.10, a0: -0.061 }, yaw: TWIN_YAW },
    // Baron 58, LEMG 13 (1791802802, 1791803088, 1791816184).
    Fit { name: "Baron", w2: 19.61, cq: 3.304, ke: 12.47, ge: 0.0906, kl: -0.2359, c: 0.0581, a0: -7.760, kp: 0.3633, kg: 0.0992,
          s0: -3.096, s1: 3.873, aoff: -2.22, trim: 0.0945, delay: 0.02,
          h0: 60.3, th0: -2.85, g0: -3.69, v0: 104.2, q0: 0.12, lever: Some([0.512, 0.512, 0.48, 0.359, 0.275, 0.09]),
          // 6 arrests, 55 windows.
          roll: Roll { kp: 18.18, tau: 0.30, delay: 0.15, a0: -0.004 }, yaw: TWIN_YAW },
    // Cessna 172 G1000, LEMG 13 (1791814879).
    Fit { name: "C172", w2: 10.31, cq: 1.502, ke: 11.34, ge: 0.0255, kl: -0.2245, c: 0.2000, a0: -7.534, kp: -0.0186, kg: -0.0896,
          s0: -2.935, s1: 4.045, aoff: 0.30, trim: -0.1572, delay: 0.02,
          h0: 49.7, th0: -3.28, g0: -2.84, v0: 67.4, q0: 0.34, lever: Some([0.515, 0.473, 0.359, 0.172, 0.068, 0.003]),
          // 25 arrests, 171 windows.
          roll: Roll { kp: 18.26, tau: 0.45, delay: 0.0, a0: 0.009 }, yaw: LIGHT_YAW },
    // Cirrus SF50, LEMG 13 (1791805150, 1791805443, 1791809604; fitted 2026-10-12
    // as the five above, 0.21 deg rms; the gate and lever are 1791809604's).
    Fit { name: "SF50", w2: 9.074, cq: 2.864, ke: 17.65, ge: 0.0169, kl: 0.0578, c: 0.0708, a0: -10.14, kp: 0.0118, kg: 0.1085,
          s0: -2.256, s1: 5.837, aoff: 2.83, trim: -0.086, delay: 0.10,
          h0: 49.9, th0: -1.62, g0: -2.62, v0: 92.0, q0: 0.17, lever: Some([0.203, 0.203, 0.181, 0.075, 0.0, 0.0]),
          // 3 arrests, 25 windows.
          roll: Roll { kp: 11.83, tau: 0.60, delay: 0.15, a0: -0.002 }, yaw: LIGHT_YAW },
    // Boeing 737-800, its identification flight (2026-10-12, 160 kt flaps
    // 15, scaled to 140 kt): pitch b 16.9 (w2 ke), the alpha spring 1.28,
    // damping 0.58, the path's 0.58 /s (c); a0 the flaps' (an approach at 3
    // to 4 deg of alpha), the speed the book's (idle 1.4 kt/s at flaps 30,
    // the lever twice the 3,000 fpm book climb); roll 47 deg/s^2 a unit
    // over 1.87 /s; yaw as fitted, a synthetic dihedral.
    Fit { name: "737", w2: 1.28, cq: 0.58, ke: 13.2, ge: 0.02, kl: 0.0, c: 0.0243, a0: -10.0, kp: 0.0, kg: 0.1,
          s0: -1.4, s1: 8.07, aoff: 0.0, trim: 0.0, delay: 0.05,
          h0: 0.0, th0: 0.0, g0: 0.0, v0: 140.0, q0: 0.0, lever: None,
          roll: Roll { kp: 21.6, tau: 0.62, delay: 0.05, a0: 0.0 },
          yaw: Yaw { b: 11.2, damping: 0.03, stiffness: 1.35, path: 0.24, dihedral: 2.0 } },
];

/// The 737-800's own flare, EGBB 15 (1792092651, 2026-10-15): fitted from
/// the gate (48.7 ft, 154 kt, -3.8 deg, flaps 40) to the wheels on the
/// recorded column (0.7 deg rms on theta, the flare in ground effect):
/// effectiveness 12.6 deg/s^2 a unit (w2 ke), the alpha spring 1.9, no
/// damping of its own (the output-error fit, `pilot::ident`, read 11.8, 2.1
/// and 0.02 at R2 0.96), the path 0.75 /s, the command answered 0.175 s
/// after the tick that wrote it. The pilot flew it on the 116 kt
/// identification scaled to 154 kt (b 36), and the column went stop to stop
/// through the flare: 439 fpm and a bounce. The speed is the book's, the
/// lever the one that holds it.
const EGBB_737: Fit = Fit { name: "737 EGBB", w2: 1.88, cq: 0.0, ke: 6.70, ge: 0.02, kl: 0.0, c: 0.0263, a0: -9.68, kp: 0.0, kg: 0.1,
    s0: -1.4, s1: 8.07, aoff: -0.69, trim: 0.0, delay: 0.175,
    h0: 48.7, th0: -3.82, g0: -2.74, v0: 154.3, q0: -0.81, lever: Some([0.05, 0.05, 0.04, 0.02, 0.0, 0.0]),
    roll: FITS[6].roll, yaw: FITS[6].yaw };

fn shape(h: f64) -> f64 {
    (1.0 - h / 18.0).clamp(0.0, 1.0).powi(2)
}

impl Fit {
    fn lever_at(&self, h: f64) -> f64 {
        const AT: [f64; 6] = [40.0, 30.0, 20.0, 10.0, 5.0, 0.0];
        let lever = self.lever.unwrap_or([0.0; 6]);
        if h >= AT[0] {
            return lever[0];
        }
        for i in 1..6 {
            if h >= AT[i] {
                let k = (h - AT[i]) / (AT[i - 1] - AT[i]);
                return lever[i] + k * (lever[i - 1] - lever[i]);
            }
        }
        lever[5]
    }

    /// Trimmed on a path of `gp` degrees at `v` kt, clear of the ground: the
    /// lever, the angle of attack and the column.
    fn trimmed(&self, v: f64, gp: f64) -> (f64, f64, f64) {
        let lev = ((G * gp.to_radians().sin() / KT - self.s0) / self.s1).clamp(0.0, 1.0);
        let al = self.a0 + gp.to_radians().cos() / (self.c * (v / 80.0).powi(2) * (1.0 + self.kp * lev));
        let e = (al - self.aoff) / self.ke + self.kl * lev;
        (lev, al, e)
    }
}

/// The pitch plant's state.
#[derive(Clone, Copy, Debug)]
struct Pitch {
    th: f64,
    q: f64,
    gam: f64,
    v: f64,
    h: f64,
}

impl Pitch {
    /// One step of `dt` with the column `e`, the lever and a path
    /// disturbance (deg/s).
    fn step(&mut self, f: &Fit, e: f64, lev: f64, gust: f64, dt: f64) {
        let al = self.th - self.gam;
        let sh = shape(self.h);
        let ae = f.aoff + f.ke * (e - f.ge * sh - f.kl * lev);
        let qd = f.w2 * (self.v / f.v0).powi(2) * (ae - al) - f.cq * (self.v / f.v0) * self.q;
        let n = f.c * (self.v / 80.0).powi(2) * (al - f.a0) * (1.0 + f.kp * lev) * (1.0 + f.kg * sh);
        let gd = (G / (self.v * KT) * (n - self.gam.to_radians().cos())).to_degrees() + gust;
        let vd = f.s0 + f.s1 * lev - G * self.gam.to_radians().sin() / KT;
        self.q += qd * dt;
        self.th += self.q * dt;
        self.gam += gd * dt;
        self.v += vd * dt;
        self.h += self.v * 1.687_81 * self.gam.to_radians().sin() * dt;
    }

    fn sink_fpm(&self) -> f64 {
        -self.v * FPM_PER_KT * self.gam.to_radians().sin()
    }
}

/// The lateral plant's state: bank, roll rate, heading (from the course),
/// yaw rate, X-Plane's sideslip, the offset from the centreline (right
/// positive).
#[derive(Clone, Copy, Debug, Default)]
struct Lat {
    phi: f64,
    p: f64,
    psi: f64,
    r: f64,
    beta: f64,
    x: f64,
}

impl Lat {
    /// Crabbed into a crosswind from the right of `xw` kt at `v` kt.
    fn crabbed(xw: f64, v: f64, x: f64) -> Lat {
        Lat { psi: (xw / v).clamp(-0.9, 0.9).asin().to_degrees(), x, ..Lat::default() }
    }

    /// Sideways velocity over the ground, m/s right positive.
    fn side(&self, v: f64, xw: f64) -> f64 {
        v * KT * (self.psi - self.beta).to_radians().sin() - xw * KT
    }

    fn step(&mut self, f: &Fit, ail: f64, rud: f64, v: f64, xw: f64, dt: f64) {
        let (r, y) = (f.roll, f.yaw);
        let q = (v / f.v0).powi(2);
        let pd = (r.kp * (ail - r.a0) - self.p) / r.tau + y.dihedral * q * self.beta;
        let rd = y.b * q * rud - y.damping * (v / f.v0) * self.r - y.stiffness * q * self.beta;
        let turn = (G / (v * KT) * self.phi.to_radians().sin()).to_degrees();
        let bd = self.r - turn - y.path * (v / f.v0) * self.beta;
        let side = self.side(v, xw);
        self.p += pd * dt;
        self.phi += self.p * dt;
        self.r += rd * dt;
        self.psi += self.r * dt;
        self.beta += bd * dt;
        self.x += side * dt;
    }
}

/// A command queue through a transport delay.
struct Delay {
    q: Vec<(f64, f64)>,
    now: f64,
}

impl Delay {
    fn new(v: f64) -> Delay {
        Delay { q: Vec::new(), now: v }
    }
    fn push(&mut self, at: f64, v: f64) {
        self.q.push((at, v));
    }
    fn at(&mut self, t: f64) -> f64 {
        while self.q.first().is_some_and(|(at, _)| *at <= t + 1e-9) {
            self.now = self.q.remove(0).1;
        }
        self.now
    }
}

/// The identification flight on the plant, as `xfc identify` flies it: at
/// the gate's speed clear of the ground, trimmed level, each axis's input
/// twice (each sign) from trim, sampled at 8 Hz; then a power step with the
/// attitude law holding level on the fleet's priors. The airframe the
/// pilot then flies on.
fn identify(f: &Fit) -> Airframe {
    let (dt, tick) = (0.005, 0.125);
    let v0 = f.v0;
    let (lev0, al0, e0) = f.trimmed(v0, 0.0);
    // PITCH: the 3-2-1-1.
    let pitch_runs: Vec<Vec<Sample>> = [1.0, -1.0]
        .into_iter()
        .map(|sign| {
            let mut s = Pitch { th: al0, q: 0.0, gam: 0.0, v: v0, h: 3000.0 };
            let mut d = Delay::new(e0);
            let mut out = Vec::new();
            let (mut t, mut next) = (0.0, 0.0);
            while t < 7.0 {
                if t >= next - 1e-9 {
                    let u = e0 + 0.04 * sign * input_at(Axis::Pitch, t - 1.0);
                    d.push(t + f.delay.max(0.01), u);
                    out.push(Sample { t, u, rate: s.q, angle: s.th - s.gam, angle_push: 0.0, ias_kts: s.v, tas_kts: s.v });
                    next += tick;
                }
                let e = d.at(t);
                s.step(f, e, lev0, 0.0, dt);
                t += dt;
            }
            out
        })
        .collect();
    // ROLL and YAW: doublets, the other controls at trim.
    let lat_runs = |axis: Axis, amp: f64| -> Vec<(Vec<Sample>, Vec<Sample>)> {
        [1.0, -1.0]
            .into_iter()
            .map(|sign| {
                let mut s = Lat::default();
                let (mut da, mut dr) = (Delay::new(f.roll.a0), Delay::new(0.0));
                let (mut out, mut rolled) = (Vec::new(), Vec::new());
                let (mut t, mut next) = (0.0, 0.0);
                while t < 6.0 {
                    if t >= next - 1e-9 {
                        let k = amp * sign * input_at(axis, t - 1.0);
                        let (a, r) = if axis == Axis::Roll { (f.roll.a0 + k, 0.0) } else { (f.roll.a0, k) };
                        da.push(t + f.roll.delay.max(0.01), a);
                        dr.push(t + f.delay.max(0.01), r);
                        let push = -(G / (v0 * KT) * s.phi.to_radians().sin()).to_degrees();
                        out.push(match axis {
                            Axis::Roll => Sample { t, u: a, rate: s.p, angle: 0.0, angle_push: 0.0, ias_kts: v0, tas_kts: v0 },
                            _ => Sample { t, u: r, rate: s.r, angle: s.beta, angle_push: push, ias_kts: v0, tas_kts: v0 },
                        });
                        rolled.push(Sample { t, u: a, rate: s.p, angle: s.beta, angle_push: 0.0, ias_kts: v0, tas_kts: v0 });
                        next += tick;
                    }
                    let (a, r) = (da.at(t), dr.at(t));
                    s.step(f, a, r, v0, 0.0, dt);
                    t += dt;
                }
                (out, rolled)
            })
            .collect()
    };
    let (roll_runs, _): (Vec<_>, Vec<_>) = lat_runs(Axis::Roll, 0.1).into_iter().unzip();
    let (yaw_runs, slip_rolls): (Vec<_>, Vec<_>) = lat_runs(Axis::Yaw, 0.15).into_iter().unzip();
    let pitch = ident::fit_axis(&pitch_runs, true).expect("pitch identified");
    let mut roll = ident::fit_axis(&roll_runs, false).expect("roll identified");
    roll.response.cross = ident::fit_cross(&slip_rolls, &roll.response).unwrap_or(0.0);
    let yaw = ident::fit_axis(&yaw_runs, true).expect("yaw identified");
    // ENERGY: the lever stepped 0.1 either way while the attitude law holds
    // the level path on the fleet's prior pitch response.
    let mut s = Pitch { th: al0, q: 0.0, gam: 0.0, v: v0, h: 3000.0 };
    let mut law = AttitudeFlare::default();
    let mut d = Delay::new(e0);
    let mut energy = Vec::new();
    let (mut t, mut next, mut prev_v) = (0.0, 0.0, v0);
    while t < 30.0 {
        let lev = if (5.0..15.0).contains(&t) { lev0 + 0.1 } else if (15.0..25.0).contains(&t) { lev0 - 0.1 } else { lev0 };
        let lev = lev.clamp(0.0, 1.0);
        if t >= next - 1e-9 {
            let o = law.step(
                In {
                    radio_ft: s.h,
                    theta_deg: s.th,
                    alpha_deg: s.th - s.gam,
                    q_dps: s.q,
                    ias_kts: s.v,
                    tas_kts: s.v,
                    dt: 0.2,
                    path: Some(Path { vs_fpm: 0.0, above_ft: s.h - 3000.0 }),
                },
                &super::response::prior_pitch(),
                PitchLimits::default(),
            );
            d.push(t + f.delay.max(0.01), o.elev + e0);
            energy.push(EnergySample { t, thr: lev, ias_kts: s.v, accel_kts_s: (s.v - prev_v) / 0.2, vs_fpm: -s.sink_fpm() });
            prev_v = s.v;
            next += 0.2;
        }
        let e = d.at(t);
        s.step(f, e, lev, 0.0, dt);
        t += dt;
    }
    let e = ident::fit_energy(&energy).expect("the power step fits");
    Airframe {
        pitch: pitch.response,
        roll: roll.response,
        yaw: yaw.response,
        tecs: Tecs { fpm_per_unit: e.fpm_per_unit, spool_s: e.spool_s },
        limits: PitchLimits::default(),
        measured: [true; 4],
    }
}

/// The identification's input at `t` seconds from its start: 0 outside it.
fn input_at(axis: Axis, t: f64) -> f64 {
    let mut at = 0.0;
    for &(len, sign) in ident::input_for(axis) {
        if (at..at + len).contains(&t) {
            return sign;
        }
        at += len;
    }
    0.0
}

/// Each plant identified: the effectiveness within 20% of the plant's own
/// (pitch `w2 ke`, roll `kp / tau`, yaw `b`), and the throttle's energy
/// within 25% (`s1` as an energy rate).
#[test]
fn identification_recovers_each_plants_effectiveness() {
    for f in FITS {
        let a = identify(&f);
        let truth = [f.w2 * f.ke, f.roll.kp / f.roll.tau, f.yaw.b];
        let got = [a.pitch.b, a.roll.b, a.yaw.b];
        let k_true = f.v0 * 1.687_81 * f.s1 * 1.687_81 / 32.174 * 60.0;
        eprintln!(
            "{}: pitch {:.1}/{:.1} roll {:.1}/{:.1} dihedral {:.2}/{:.2} yaw {:.1}/{:.1} energy {:.0}/{:.0} spool {:.1}; {:?}",
            f.name, got[0], truth[0], got[1], truth[1], a.roll.cross, f.yaw.dihedral, got[2], truth[2], a.tecs.fpm_per_unit, k_true, a.tecs.spool_s, a.pitch
        );
        for (axis, (g, t)) in ["pitch", "roll", "yaw"].iter().zip(got.iter().zip(truth)) {
            assert!((g / t - 1.0).abs() < 0.2, "{} {axis}: {g:.1} against {t:.1}", f.name);
        }
        assert!((a.tecs.fpm_per_unit / k_true - 1.0).abs() < 0.25, "{} energy {:.0} against {k_true:.0}", f.name, a.tecs.fpm_per_unit);
    }
}

#[derive(Debug)]
struct Landing {
    /// The sink at the wheels, and the worst in the last 2.5 s (the
    /// scorecard's window), fpm.
    wheels_fpm: f64,
    scored_fpm: f64,
    balloon_ft: f64,
    /// The elevator thrown from one stop to the other (beyond 0.9 either
    /// way): the bang-bang of a loop ringing on its limits.
    reversals: usize,
}

/// A path disturbance: none, or one gust that shallows the path by `deg`
/// over a second once below `below_ft`.
#[derive(Clone, Copy)]
enum Air {
    Calm,
    Gust { below_ft: f64, deg: f64 },
}

/// The flare from the recorded gate on the recorded lever, the law on the
/// pitch response `resp`.
fn fly(f: Fit, resp: &Response, air: Air) -> Landing {
    let (dt, tick) = (0.01, 0.2);
    let mut law = AttitudeFlare::default();
    let mut s = Pitch { th: f.th0, q: f.q0, gam: f.g0, v: f.v0, h: f.h0 };
    let mut d = Delay::new(f.trim);
    let (mut t, mut next_tick) = (0.0, 0.0);
    let (mut low, mut balloon) = (s.h, 0.0_f64);
    let mut gust_from: Option<f64> = None;
    let mut sinks: Vec<(f64, f64)> = Vec::new();
    let (mut stop, mut reversals) = (0.0, 0);
    while t < 40.0 {
        if t >= next_tick - 1e-9 {
            let o = law.step(
                In { radio_ft: s.h, theta_deg: s.th, alpha_deg: s.th - s.gam, q_dps: s.q, ias_kts: s.v, tas_kts: s.v, dt: tick, path: None },
                resp,
                PitchLimits::default(),
            );
            if o.elev.abs() > 0.9 {
                reversals += usize::from(stop == -o.elev.signum());
                stop = o.elev.signum();
            }
            d.push(t + f.delay.max(0.02), o.elev + f.trim);
            next_tick += tick;
        }
        let gust = match air {
            Air::Calm => 0.0,
            Air::Gust { below_ft, deg } => {
                if gust_from.is_none() && s.h < below_ft {
                    gust_from = Some(t);
                }
                if gust_from.is_some_and(|t0| t - t0 < 1.0) { deg } else { 0.0 }
            }
        };
        let e = d.at(t);
        s.step(&f, e, f.lever_at(s.h), gust, dt);
        low = low.min(s.h);
        balloon = balloon.max(s.h - low);
        sinks.push((t, s.sink_fpm()));
        t += dt;
        if s.h <= 0.0 {
            let scored = sinks.iter().filter(|(ts, _)| t - ts <= 2.5).map(|s| s.1).fold(f64::MIN, f64::max);
            return Landing { wheels_fpm: s.sink_fpm(), scored_fpm: scored, balloon_ft: balloon, reversals };
        }
    }
    panic!("{}: no landing in 40 s", f.name);
}

/// The recorded flares (every airframe but the 737, whose plant is its
/// identification's).
fn recorded() -> impl Iterator<Item = Fit> {
    FITS.into_iter().filter(|f| f.lever.is_some())
}

/// Calm air, every fitted airframe from its own recorded gate on the
/// response its identification measured: inside the 100 to 180 sweet
/// spot's neighbourhood at the wheels, never a balloon.
#[test]
fn every_fitted_airframe_lands_soft_in_calm_air() {
    for f in recorded() {
        let a = identify(&f);
        let l = fly(f, &a.pitch, Air::Calm);
        eprintln!("{}: {l:?}", f.name);
        assert!(
            (50.0..=180.0).contains(&l.wheels_fpm) && l.scored_fpm < 200.0 && l.balloon_ft < 1.0,
            "{}: {l:?}",
            f.name
        );
    }
}

/// The 737-800's EGBB flare, identified as `xfc identify` would fly it,
/// through its own 0.175 s and a shorter 0.1: inside the touchdown limit
/// and never stop to stop. Before the feed-forward was taken at the
/// attitude crossover and the path trim held while the nose catches up,
/// the 0.175 s plant threw the column stop to stop twice and met the runway
/// at 820 fpm; the 0.1 s one rang four times.
#[test]
fn the_737s_flare_never_rings_on_its_stops() {
    for delay in [0.175, 0.1] {
        let f = Fit { delay, ..EGBB_737 };
        let l = fly(f, &identify(&f).pitch, Air::Calm);
        eprintln!("{} {delay}: {l:?}", f.name);
        assert!(
            l.reversals == 0 && l.scored_fpm < crate::scorecard::TOUCHDOWN_MAX_SINK_FPM && l.wheels_fpm > 30.0,
            "{} {delay}: {l:?}",
            f.name
        );
    }
}

/// A gust that shallows the path 1.5 deg for a second, above the gate and
/// below it: never pushed back down into a hard landing, never a balloon
/// (the SR22 met the runway at 441 fpm from one at 29 ft, 1791813026).
#[test]
fn a_gust_that_shallows_the_path_is_never_flown_into_the_runway() {
    for f in recorded() {
        let a = identify(&f);
        for below_ft in [35.0, 20.0] {
            let l = fly(f, &a.pitch, Air::Gust { below_ft, deg: 1.5 });
            eprintln!("{} gust below {below_ft}: {l:?}", f.name);
            assert!(
                l.scored_fpm < crate::scorecard::TOUCHDOWN_MAX_SINK_FPM && l.balloon_ft < 1.0,
                "{} gust below {below_ft}: {l:?}",
                f.name
            );
        }
    }
}

/// The plant half again either way (the column's authority, the short
/// period's damping, the ground effect's column) while the law flies the
/// ORIGINAL identification: a measurement that is off does not decide a
/// landing. The ground effect's column took the place of the declared
/// `ground_pull` (the SR22's 0.5, the King Air's 0.25, the Baron's 0.3):
/// the observer finds it.
#[test]
fn a_misidentified_airframe_still_lands() {
    for f in recorded() {
        let a = identify(&f);
        for (what, g) in [
            ("ke x0.6", Fit { ke: f.ke * 0.6, ..f }),
            ("ke x1.6", Fit { ke: f.ke * 1.6, ..f }),
            ("cq x0.6", Fit { cq: f.cq * 0.6, ..f }),
            ("ge +0.2", Fit { ge: f.ge + 0.2, ..f }),
        ] {
            let l = fly(g, &a.pitch, Air::Calm);
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
    /// The heading against the runway at the wheels, degrees.
    psi_deg: f64,
    worst_bank_deg: f64,
    kp: Option<f64>,
    slowest_kts: f64,
    lever_wheels: f64,
}

/// Fly `f` from minimums to the wheels as the loop does, on the airframe
/// `a` (its identification): the attitude law on the path then the flare,
/// the ailerons on the bank toward the centreline, the rudder coordinating
/// then decrabbing, TECS on the power through the flare, each on the
/// loop's 0.2 s tick through the plant's own delay. Trimmed on the path at
/// the gate's speed.
fn segment(f: Fit, a: &Airframe, c: Case) -> Segment {
    let (dt, tick) = (0.01, 0.2);
    let gp = -3.0_f64;
    let (lev0, al0, e0) = f.trimmed(f.v0, gp);
    let trim = e0 + c.trim_off;
    let mut s = Pitch { th: gp + al0, q: 0.0, gam: gp, v: f.v0, h: c.h0 };
    let mut path_h = c.h0 - c.above_ft;
    let mut lever = lev0;
    let (mut law, mut energy, mut bank, mut yaw) =
        (AttitudeFlare::default(), EnergyThrottle::default(), BankHold::default(), YawHold::default());
    let mut xw = c.xwind_kts;
    let mut lat = Lat::crabbed(xw, s.v, c.x0);
    let mut gust = 0.0;
    let mut noise = Noise(0x9e37_79b9_7f4a_7c15 ^ c.seed.wrapping_mul(0x2545_f491));
    let (mut de, mut da, mut dr) = (Delay::new(trim), Delay::new(f.roll.a0), Delay::new(0.0));
    let (mut t, mut next_tick, mut prev_v) = (0.0, 0.0, s.v);
    let (mut low, mut balloon, mut worst_bank, mut slowest) = (f64::MAX, 0.0_f64, 0.0_f64, s.v);
    let mut sinks: Vec<(f64, f64)> = Vec::new();
    while t < 150.0 {
        if t >= next_tick - 1e-9 {
            let o = law.step(
                In {
                    radio_ft: s.h,
                    theta_deg: s.th,
                    alpha_deg: s.th - s.gam,
                    q_dps: s.q,
                    ias_kts: s.v,
                    tas_kts: s.v,
                    dt: tick,
                    path: Some(Path { vs_fpm: s.v * FPM_PER_KT * gp.to_radians().sin(), above_ft: s.h - path_h }),
                },
                &a.pitch,
                a.limits,
            );
            de.push(t + f.delay.max(0.02), o.elev + trim);
            lever = energy
                .step(
                    super::energy::In {
                        ias_kts: s.v,
                        accel_kts_s: (s.v - prev_v) / tick,
                        vs_fpm: -s.sink_fpm(),
                        target_kts: f.v0,
                        floor_kts: None,
                        path_vs_fpm: o.vs_target_fpm,
                        thr_now: lever,
                        dt: tick,
                        flare_to_wheels_s: o.flaring.then_some(o.to_wheels_s),
                    },
                    a.tecs,
                )
                .lever;
            prev_v = s.v;
            let yi = super::yaw::In {
                psi_deg: lat.psi,
                beta_deg: lat.beta,
                r_dps: lat.r,
                phi_deg: lat.phi,
                theta_deg: s.th,
                ias_kts: s.v,
                tas_kts: s.v,
                dt: tick,
                course_deg: 0.0,
            };
            let mode = yaw.mode(o.flaring.then_some(o.to_wheels_s), &yi, &a.yaw);
            let rud = yaw.step(yi, mode, &a.yaw);
            dr.push(t + f.delay.max(0.01), rud);
            let side = lat.side(s.v, xw);
            let g = design(&a.roll.at(s.v, s.v), tick);
            let slip = super::line::slip_accel_m_s2(super::yaw::slip_planned_deg(&yi, mode), a.yaw.at(s.v, s.v).path, s.v);
            let target = bank_for_line(lat.x, side, lat.phi, s.h, period_s(attitude_tc_s(&g)), slip);
            da.push(t + f.roll.delay, bank.step(lat.phi, target, tick, &a.roll, s.v, s.v, lat.beta));
            next_tick += tick;
        }
        let (e, ail, rud) = (de.at(t), da.at(t), dr.at(t));
        s.step(&f, e, lever, 0.0, dt);
        path_h += s.v * 1.687_81 * gp.to_radians().sin() * dt;
        if c.gust_kts > 0.0 {
            gust += -gust * dt / 2.0 + c.gust_kts * dt.sqrt() * noise.gauss();
        }
        xw = c.xwind_kts + gust;
        lat.step(&f, ail, rud, s.v, xw, dt);
        if s.h < 50.0 {
            low = low.min(s.h);
            balloon = balloon.max(s.h - low);
        }
        worst_bank = worst_bank.max(lat.phi.abs());
        slowest = slowest.min(s.v);
        sinks.push((t, s.sink_fpm()));
        t += dt;
        if s.h <= 0.0 {
            let scored = sinks.iter().filter(|(ts, _)| t - ts <= 2.5).map(|s| s.1).fold(f64::MIN, f64::max);
            return Segment {
                wheels_fpm: s.sink_fpm(),
                scored_fpm: scored,
                balloon_ft: balloon,
                x_m: lat.x,
                psi_deg: lat.psi,
                worst_bank_deg: worst_bank,
                kp: bank.kp(),
                slowest_kts: slowest,
                lever_wheels: lever,
            };
        }
    }
    panic!("{}: no landing in 150 s from {c:?}", f.name);
}

/// Every airframe from minimums (the coupled 172's 200 ft, the visual 300),
/// the line 8 to 23 m away either side, calm and in a 15 kt crosswind
/// either way, flown on its identification: on the centreline at the wheels
/// within 3 m, 60 to 200 fpm, the crab out (the heading within 2 degrees of
/// the runway), no balloon, the bank within a degree of the wing-low limit.
#[test]
fn every_airframe_hand_flies_from_minimums_to_the_centreline_decrabbed() {
    for f in FITS {
        let a = identify(&f);
        for h0 in [200.0, 300.0] {
            for x0 in [23.0, 8.0, 0.0, -8.0, -23.0] {
                for xwind_kts in [0.0, 15.0, -15.0] {
                    let c = Case { h0, x0, xwind_kts, ..MINIMUMS };
                    let s = segment(f, &a, c);
                    eprintln!("{} {h0} {x0} {xwind_kts}: {s:?}", f.name);
                    assert!(
                        s.x_m.abs() < 3.0
                            && (60.0..=200.0).contains(&s.wheels_fpm)
                            && s.scored_fpm < crate::scorecard::TOUCHDOWN_MAX_SINK_FPM
                            && s.balloon_ft < 1.0
                            && s.psi_deg.abs() < 2.0
                            && s.worst_bank_deg < super::line::SLIP_BANK_MAX_DEG + 1.0,
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
/// the line within 4 m and inside the touchdown limit. A metre more
/// than the calm 3: the decrab's last two or three seconds of slip are
/// flown on a bank capped near the ground, and a gust's sideways step in
/// them is carried to the wheels (seed 4: the Evolution 3.3 m, the SF50 4.0,
/// its ailerons on their 0.6 stop in the gust at 10 ft; the old
/// law, which landed crabbed, held 3).
#[test]
fn the_segment_holds_in_gusts_off_the_path_and_out_of_trim() {
    for f in FITS {
        let a = identify(&f);
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
            let s = segment(f, &a, c);
            eprintln!("{} {c:?}: {s:?}", f.name);
            assert!(
                s.x_m.abs() < 4.0
                    && s.wheels_fpm > 40.0
                    && s.scored_fpm < crate::scorecard::TOUCHDOWN_MAX_SINK_FPM
                    && s.balloon_ft < 1.0,
                "{} {c:?}: {s:?}",
                f.name
            );
        }
    }
}

/// The roll response the segment refines in flight (`bank::RollResponse`)
/// lands near each airframe's plant, and the bank holds the line with the
/// roll half and twice what was identified, its mode twice as slow or 0.3 s
/// late: a measurement that is off is not a crash.
#[test]
fn the_roll_response_is_refined_in_flight() {
    for f in FITS {
        let a = identify(&f);
        let s = segment(f, &a, Case { x0: 23.0, xwind_kts: 10.0, ..MINIMUMS });
        let kp = s.kp.expect("measured");
        eprintln!("{}: measured {kp:.1} against {:.1}", f.name, f.roll.kp);
        assert!((0.5..=1.6).contains(&(kp / f.roll.kp)), "{}: measured {kp:.1} against the fit's {:.1}", f.name, f.roll.kp);
        for (what, roll) in [
            ("kp x0.5", Roll { kp: f.roll.kp * 0.5, ..f.roll }),
            ("kp x2", Roll { kp: f.roll.kp * 2.0, ..f.roll }),
            ("tau x2", Roll { tau: f.roll.tau * 2.0, ..f.roll }),
            ("delay 0.3", Roll { delay: 0.3, ..f.roll }),
        ] {
            let g = Fit { roll, ..f };
            for x0 in [23.0, -8.0] {
                let s = segment(g, &a, Case { x0, xwind_kts: 15.0, ..MINIMUMS });
                eprintln!("{} {what} from {x0}: {s:?}", f.name);
                assert!(s.x_m.abs() < 3.0 && s.worst_bank_deg < super::line::SLIP_BANK_MAX_DEG + 2.0, "{} {what} from {x0}: {s:?}", f.name);
            }
        }
    }
}

/// The A330's four recorded arrest gates at LEMG 30 (#180, sorties
/// 1790633832 and 1790634412, 2026-09-28) on its sidestick's roll-rate
/// plant (fitted 11 to 13 deg/s per unit after 0.25 to 0.37 s; 9 to 20 and
/// up to 0.6 s bound it), identified as `xfc identify` would, then flown by
/// the bank hold on the NPFG period its measured roll gives: (x m, sideways
/// m/s, bank, s to the wheels, ft). On the delays fitted (0.35 s) within
/// 2 m of the line at the wheels; at the 0.6 s bound within 4 m (the period
/// a 0.6 s bank loop allows is 18 s and the gate has 17). The floated gate
/// (8 m right crossing left at 3.8 m/s, banked 3.4 the wrong way) is stopped
/// by the 5 degree cap, not the law: 13 to 17 m past the line (the declared
/// law 7.2 to 12.9 on a hand-fitted lead). Gate 1 (7.5 m left closing at 1.2
/// m/s, banked 5 degrees toward the line) crosses it by 4.3 to 6.4 m before
/// coming back (the old law: under 2): the price of a period bounded by the
/// measured loop rather than tuned to these four gates.
#[test]
fn the_bank_hold_lands_the_a330s_recorded_gates() {
    let gates = [
        (0.55, 0.25, -1.47, 17.4, 87.9),
        (-7.5, 1.23, 5.19, 17.9, 86.4),
        (7.96, -3.8, -3.37, 17.3, 86.9),
        (1.07, 0.21, 0.6, 20.3, 90.8),
    ];
    for (k, dead) in [(13.0, 0.35), (20.0, 0.35), (13.0, 0.6), (20.0, 0.6), (9.0, 0.35)] {
        // The sidestick asks a roll rate: identified on its doublets.
        let runs: Vec<Vec<Sample>> = [1.0, -1.0]
            .into_iter()
            .map(|sign| {
                let (h, tick) = (0.005, 0.125);
                let (mut p, mut t, mut next) = (0.0_f64, 0.0, 0.0);
                let mut d = Delay::new(0.0);
                let mut out = Vec::new();
                while t < 6.0 {
                    if t >= next - 1e-9 {
                        let u = 0.1 * sign * input_at(Axis::Roll, t - 1.0);
                        d.push(t + dead, u);
                        out.push(Sample { t, u, rate: p, angle: 0.0, angle_push: 0.0, ias_kts: 140.0, tas_kts: 140.0 });
                        next += tick;
                    }
                    p += (k * d.at(t) - p) * h / 0.07;
                    t += h;
                }
                out
            })
            .collect();
        let roll = ident::fit_axis(&runs, false).expect("the sidestick identified").response;
        for (i, (x0, v0, phi0, t_end, radio0)) in gates.into_iter().enumerate() {
            let (tick, h) = (0.2, 0.01);
            let mut bank = BankHold::default();
            let (mut x, mut v, mut phi, mut p) = (x0, v0, phi0, 0.0_f64);
            let mut acc = 9.81 * f64::to_radians(phi0).tan();
            let mut written: Vec<(f64, f64)> = Vec::new();
            let (mut t, mut next, mut worst) = (0.0, 0.0, 0.0_f64);
            let period = period_s(attitude_tc_s(&design(&roll, tick)));
            while t < t_end {
                if t + 1e-9 >= next {
                    next += tick;
                    let radio = radio0 * (1.0 - t / t_end);
                    written.push((t, bank.step(phi, bank_for_line(x, v, phi, radio, period, 0.0), tick, &roll, 140.0, 140.0, 0.0)));
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
            assert!(x.abs() < if dead > 0.5 { 4.0 } else { 2.0 }, "gate {i} at {k}/{dead}: lands {x:.1} m off");
            assert!(worst < [2.0, 7.0, 17.0, 2.0][i], "gate {i} at {k}/{dead}: overshoots {worst:.1}");
        }
    }
}
