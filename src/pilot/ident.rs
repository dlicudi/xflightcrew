//! Identification: fitting an axis' [`Response`] to what a measurement
//! flight recorded (`xvl identify`, `src/identify.rs`). Pure, so the plants
//! test it (`plants_tests`).
//!
//! Output error ([`fit_segment`]): the rate simulated from the control `d`
//! seconds old, the measured airflow angle and a trim, for every decay and
//! delay on a grid, the weights by least squares; the pair that explains the
//! most is believed (the roll fits of 2026-10-12 picked their delay the same
//! way). The path's answer is a second fit: how fast the airflow angle
//! relaxes on its own. Every input is fitted on its own and the MEDIAN
//! believed (ArduPlane's autotune takes a median the same way): one gust in
//! one doublet does not set the airframe's numbers. Irregular sampling (the
//! Web API read paused 1.1 s once in the 737's flight, 2026-10-12) is
//! simulated through, never differenced.
//!
//! The throttle's energy is a third: the energy rate (climb plus speed times
//! acceleration over g) regressed on the lever through the engine's own lag,
//! the lag picked on a grid: TECS's `K_thr2STE` and the spool, measured.

pub use crate::aircraft::learned::{EnergyFit, Fit};
use super::response::Response;
use serde::{Deserialize, Serialize};

/// One recorded tick of an axis under test.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    /// Seconds, any origin.
    pub t: f64,
    /// The control commanded from this tick on, unit ratio.
    pub u: f64,
    /// The axis' rate, deg/s (q, p or r).
    pub rate: f64,
    /// The airflow angle it turns, deg: alpha for the pitch, X-Plane's beta
    /// for the yaw (nose right of the airpath positive); 0 for the roll.
    pub angle: f64,
    /// What else moves the airflow angle, deg/s: the yaw's gravity term
    /// (g / V sin bank); 0 elsewhere.
    pub angle_push: f64,
    pub ias_kts: f64,
    pub tas_kts: f64,
}

/// The delays tried, s.
const DELAY_STEP_S: f64 = 0.025;
const DELAY_MAX_S: f64 = 0.5;
/// A doublet whose fit explains less than this is not believed.
const R2_MIN: f64 = 0.6;

/// Least squares by the normal equations, small and well scaled. `None`
/// when singular.
fn solve(rows: &[(Vec<f64>, f64)]) -> Option<(Vec<f64>, f64)> {
    let n = rows.first()?.0.len();
    if rows.len() < n + 3 {
        return None;
    }
    let mut a = vec![vec![0.0; n + 1]; n];
    for (x, y) in rows {
        for i in 0..n {
            for j in 0..n {
                a[i][j] += x[i] * x[j];
            }
            a[i][n] += x[i] * y;
        }
    }
    for c in 0..n {
        let p = (c..n).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
        if a[p][c].abs() < 1e-9 {
            return None;
        }
        a.swap(c, p);
        for r in 0..n {
            if r != c {
                let f = a[r][c] / a[c][c];
                for k in c..=n {
                    a[r][k] -= f * a[c][k];
                }
            }
        }
    }
    let beta: Vec<f64> = (0..n).map(|i| a[i][n] / a[i][i]).collect();
    let mean = rows.iter().map(|r| r.1).sum::<f64>() / rows.len() as f64;
    let (mut ss_res, mut ss_tot) = (0.0, 0.0);
    for (x, y) in rows {
        let f: f64 = x.iter().zip(&beta).map(|(a, b)| a * b).sum();
        ss_res += (y - f).powi(2);
        ss_tot += (y - mean).powi(2);
    }
    let r2 = if ss_tot > 1e-12 { 1.0 - ss_res / ss_tot } else { 0.0 };
    Some((beta, r2))
}

/// The control in force at time `t`: the last command at or before it.
fn u_at(s: &[Sample], t: f64) -> f64 {
    let i = s.partition_point(|x| x.t <= t + 1e-9);
    if i == 0 { s[0].u } else { s[i - 1].u }
}

/// The interpolated value of `f` at time `t` (held at the ends).
fn lerp_at(s: &[Sample], t: f64, f: impl Fn(&Sample) -> f64) -> f64 {
    let i = s.partition_point(|x| x.t <= t);
    if i == 0 {
        return f(&s[0]);
    }
    if i >= s.len() {
        return f(&s[s.len() - 1]);
    }
    let (a, b) = (&s[i - 1], &s[i]);
    let k = ((t - a.t) / (b.t - a.t).max(1e-9)).clamp(0.0, 1.0);
    f(a) + k * (f(b) - f(a))
}

/// The response of `x' = -lam x + w(t)` from rest, at every sample's time,
/// integrated exactly over sub-steps of [`SUB_S`] with `w` taken at each
/// sub-step's middle.
fn filtered(s: &[Sample], lam: f64, w: impl Fn(f64) -> f64) -> Vec<f64> {
    let mut out = Vec::with_capacity(s.len());
    let mut x = 0.0;
    out.push(0.0);
    for k in 1..s.len() {
        let (t0, t1) = (s[k - 1].t, s[k].t);
        let n = ((t1 - t0) / SUB_S).ceil().max(1.0) as usize;
        let h = (t1 - t0) / n as f64;
        let decay = (-lam * h).exp();
        let gain = if lam > 1e-9 { (1.0 - decay) / lam } else { h };
        for j in 0..n {
            let tm = t0 + (j as f64 + 0.5) * h;
            x = x * decay + gain * w(tm);
        }
        out.push(x);
    }
    out
}

/// Sub-step of the output-error simulation, s.
const SUB_S: f64 = 0.01;
/// The rate's decays tried, 1/s: 0.02 (a yaw's weak damping) to 20 (a
/// sidestick's rate loop), geometric.
fn damping_grid() -> impl Iterator<Item = f64> {
    (0..91).map(|i| 0.02 * 1.08_f64.powi(i))
}

/// Fit one input. `has_angle`: the axis turns an airflow angle (pitch,
/// yaw). `None` when it explains too little.
///
/// OUTPUT error: for every decay and delay on their grids the rate is the
/// sum of the control's, the angle's and the trim's filtered responses and
/// the initial rate's decay, linear in their weights; the pair whose least
/// squares leaves the least is believed. Equation error (the rate's
/// derivative by differences) read a first-order roll with a 0.1 s delay
/// at 73% of its effectiveness at 8 Hz: the difference straddles every step.
pub fn fit_segment(s: &[Sample], has_angle: bool) -> Option<Fit> {
    if s.len() < 12 {
        return None;
    }
    let t0 = s[0].t;
    let angle = |t: f64| lerp_at(s, t, |x| x.angle);
    let mut best: Option<(Vec<f64>, f64, f64, f64)> = None;
    for lam in damping_grid() {
        let fa = if has_angle { filtered(s, lam, angle) } else { Vec::new() };
        let f1 = filtered(s, lam, |_| 1.0);
        let mut d = 0.0;
        while d <= DELAY_MAX_S + 1e-9 {
            let fu = filtered(s, lam, |t| u_at(s, t - d));
            let rows: Vec<(Vec<f64>, f64)> = (0..s.len())
                .map(|k| {
                    let ic = (-lam * (s[k].t - t0)).exp();
                    let v = if has_angle { vec![fu[k], fa[k], f1[k], ic] } else { vec![fu[k], f1[k], ic] };
                    (v, s[k].rate)
                })
                .collect();
            if let Some((beta, r2)) = solve(&rows) {
                if best.as_ref().is_none_or(|b| r2 > b.1 + 1e-12) {
                    best = Some((beta, r2, d, lam));
                }
            }
            d += DELAY_STEP_S;
        }
    }
    let (beta, r2, delay_s, damping) = best?;
    let b = beta[0];
    let stiffness = if has_angle { -beta[1] } else { 0.0 };
    // The path: the airflow angle relaxes as `s' = x + push - path s`; its
    // output error, on the measured rate and push, for each decay.
    let path = if has_angle {
        let mut best_p: Option<(f64, f64)> = None;
        for lam in damping_grid().filter(|l| *l <= 10.0) {
            let drive = filtered(s, lam, |t| lerp_at(s, t, |x| x.rate + x.angle_push));
            let f1 = filtered(s, lam, |_| 1.0);
            let rows: Vec<(Vec<f64>, f64)> = (0..s.len())
                .map(|k| (vec![f1[k], (-lam * (s[k].t - t0)).exp()], s[k].angle - drive[k]))
                .collect();
            if let Some((beta, _)) = solve(&rows) {
                // The target moves with the decay tried: judged on what is
                // left, not on a share of it.
                let left: f64 = rows.iter().map(|(x, y)| (y - x.iter().zip(&beta).map(|(a, b)| a * b).sum::<f64>()).powi(2)).sum();
                if best_p.is_none_or(|b| left < b.1) {
                    best_p = Some((lam, left));
                }
            }
        }
        best_p.map_or(0.0, |p| p.0)
    } else {
        0.0
    };
    let n = s.len() as f64;
    let ias_kts = s.iter().map(|x| x.ias_kts).sum::<f64>() / n;
    let tas_kts = s.iter().map(|x| x.tas_kts).sum::<f64>() / n;
    let response = Response { b, damping, stiffness, path, cross: 0.0, delay_s, ias_kts, tas_kts };
    (r2 >= R2_MIN && response.sane()).then_some(Fit { response, r2, segments: 1 })
}

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    let n = v.len();
    if n % 2 == 1 { v[n / 2] } else { 0.5 * (v[n / 2 - 1] + v[n / 2]) }
}

/// Every doublet of one axis, fitted alone, the median believed, at the
/// mean of their airspeeds (each scaled there first). `None` when no
/// doublet was believed.
pub fn fit_axis(segments: &[Vec<Sample>], has_angle: bool) -> Option<Fit> {
    let fits: Vec<Fit> = segments.iter().filter_map(|s| fit_segment(s, has_angle)).collect();
    if fits.is_empty() {
        return None;
    }
    let n = fits.len() as f64;
    let ias = fits.iter().map(|f| f.response.ias_kts).sum::<f64>() / n;
    let tas = fits.iter().map(|f| f.response.tas_kts).sum::<f64>() / n;
    let at: Vec<Response> = fits.iter().map(|f| f.response.at(ias, tas)).collect();
    let m = |g: fn(&Response) -> f64| median(at.iter().map(g).collect());
    let response = Response {
        b: m(|r| r.b),
        damping: m(|r| r.damping),
        stiffness: m(|r| r.stiffness),
        path: m(|r| r.path),
        cross: 0.0,
        delay_s: m(|r| r.delay_s),
        ias_kts: ias,
        tas_kts: tas,
    };
    Some(Fit { response, r2: median(fits.iter().map(|f| f.r2).collect()), segments: fits.len() })
}

/// The roll each degree of sideslip gives (the dihedral, deg/s^2 per
/// degree of X-Plane's beta), from the RUDDER inputs' roll: the roll rate
/// (`rate`) against the sideslip (`angle`), the ailerons held (`u`), on the
/// roll response already fitted. The aileron inputs barely slip, so they
/// cannot show it; the rudder's slip rolls the aircraft by it alone. The
/// median over the inputs; `None` when none fits.
pub fn fit_cross(segments: &[Vec<Sample>], roll: &Response) -> Option<f64> {
    let mut found: Vec<f64> = segments
        .iter()
        .filter(|s| s.len() >= 12)
        .filter_map(|s| {
            let n = s.len() as f64;
            let r = roll.at(s.iter().map(|x| x.ias_kts).sum::<f64>() / n, s.iter().map(|x| x.tas_kts).sum::<f64>() / n);
            let t0 = s[0].t;
            let lam = r.damping;
            let fu = filtered(s, lam, |t| u_at(s, t - r.delay_s));
            let fa = filtered(s, lam, |t| lerp_at(s, t, |x| x.angle));
            let f1 = filtered(s, lam, |_| 1.0);
            let rows: Vec<(Vec<f64>, f64)> = (0..s.len())
                .map(|k| (vec![fa[k], f1[k], (-lam * (s[k].t - t0)).exp()], s[k].rate - r.b * fu[k]))
                .collect();
            solve(&rows).filter(|(_, r2)| *r2 >= R2_MIN).map(|(beta, _)| beta[0] * roll.ias_kts.powi(2) / r.ias_kts.powi(2))
        })
        .filter(|c| c.is_finite())
        .collect();
    if found.is_empty() {
        return None;
    }
    found.sort_by(f64::total_cmp);
    Some(found[found.len() / 2])
}

/// One tick of a power step: the lever, the airspeed, its trend and the
/// TRUE vertical speed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EnergySample {
    pub t: f64,
    pub thr: f64,
    pub ias_kts: f64,
    pub accel_kts_s: f64,
    pub vs_fpm: f64,
}

/// The lags tried for the engine, s.
const SPOOL_GRID_S: [f64; 10] = [0.2, 0.4, 0.7, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0];

/// The energy rate regressed on the lever through a first-order lag, the
/// lag picked by fit. `None` without a lever that moved.
pub fn fit_energy(s: &[EnergySample]) -> Option<EnergyFit> {
    if s.len() < 20 {
        return None;
    }
    let span = s.iter().map(|x| x.thr).fold(f64::MIN, f64::max) - s.iter().map(|x| x.thr).fold(f64::MAX, f64::min);
    if span < 0.05 {
        return None;
    }
    let mut best: Option<EnergyFit> = None;
    for tau in SPOOL_GRID_S {
        let mut lagged = s[0].thr;
        let mut rows = Vec::with_capacity(s.len());
        for (i, x) in s.iter().enumerate() {
            if i > 0 {
                let h = (x.t - s[i - 1].t).clamp(0.0, 1.0);
                lagged += (s[i - 1].thr - lagged) * (1.0 - (-h / tau).exp());
            }
            let ste = super::energy::energy_rate_fpm(x.ias_kts, x.accel_kts_s, x.vs_fpm);
            rows.push((vec![lagged, 1.0], ste));
        }
        if let Some((beta, r2)) = solve(&rows) {
            let fit = EnergyFit { fpm_per_unit: beta[0], spool_s: tau, ias_kts: median(s.iter().map(|x| x.ias_kts).collect()), r2 };
            if best.is_none_or(|b| r2 > b.r2) {
                best = Some(fit);
            }
        }
    }
    best.filter(|f| f.r2 > 0.3 && (200.0..=20_000.0).contains(&f.fpm_per_unit))
}

/// The test inputs, in seconds and unit amplitude: a doublet (one second
/// each way) for the rates that settle (roll, yaw), the 3-2-1-1 for the
/// pitch (Koehler's input: its spectrum covers the short period without
/// leaving the phugoid's band). Each is `(seconds, sign)`.
pub fn input_for(axis: Axis) -> &'static [(f64, f64)] {
    match axis {
        Axis::Pitch => &[(1.2, 1.0), (0.8, -1.0), (0.4, 1.0), (0.4, -1.0)],
        Axis::Roll => &[(1.5, 1.0), (1.5, -1.0)],
        Axis::Yaw => &[(1.5, 1.0), (1.5, -1.0)],
    }
}

/// The three axes an identification flies.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Axis {
    Pitch,
    Roll,
    Yaw,
}

impl Axis {
    pub fn has_angle(self) -> bool {
        !matches!(self, Axis::Roll)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A first-order roll with a delay and a trim: the fit returns them.
    #[test]
    fn a_doublet_recovers_a_first_order_axis() {
        let (b, damp, delay, trim) = (40.0_f64, 2.5_f64, 0.1_f64, -0.03_f64);
        let (dt, tick) = (0.005_f64, 0.125_f64);
        let mut s = Vec::new();
        let mut p = 0.0_f64;
        // Whole steps: the command recorded is the one applied, `delay` late.
        let input = |t: f64| if (1.0..2.5).contains(&t) { 0.15 } else if (2.5..4.0).contains(&t) { -0.15 } else { 0.0 };
        let per_tick = (tick / dt).round() as usize;
        let late = (delay / dt).round() as usize;
        for k in 0..(8.0 / dt) as usize {
            let t = k as f64 * dt;
            let held = |k: usize| input((k / per_tick * per_tick) as f64 * dt);
            if k % per_tick == 0 {
                s.push(Sample { t, u: held(k), rate: p, angle: 0.0, angle_push: 0.0, ias_kts: 100.0, tas_kts: 100.0 });
            }
            let u = if k >= late { held(k - late) } else { 0.0 };
            p += (b * (u - trim) - damp * p) * dt;
        }
        let f = fit_segment(&s, false).expect("fits");
        eprintln!("{f:?}");
        assert!((f.response.b / b - 1.0).abs() < 0.1 && (f.response.damping / damp - 1.0).abs() < 0.15, "{f:?}");
        assert!((f.response.delay_s - delay).abs() <= 0.06, "{f:?}");
    }
}
