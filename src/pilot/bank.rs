//! Ailerons to bank, on the roll response measured.
//!
//! The servos-out arrests of 2026-10-12 (59 segments, six airframes,
//! `plants_tests`) fit ONE shape: the roll rate follows the aileron through
//! a first-order roll mode, `p' = (Kp (a - a0) - p) / tau`, with Kp 12 to
//! 19 deg/s per unit, tau 0.3 to 0.6 s, at most 0.15 s of delay, and a trim
//! a0 within 0.06 (R2 0.91 to 0.99). The airframe's response comes from its
//! identification ([`crate::pilot::learned`]), scaled by the air, else the
//! fleet's prior; and the segment refines Kp as it flies ([`RollResponse`]).
//! The loop is designed on it by the crossover rule
//! ([`crate::pilot::response::design`]): the bank error asks a roll RATE at
//! the attitude loop's crossover, the aileron flies that rate by its
//! feed-forward (1 / Kp) and the rate loop's gain, on top of the trim the
//! bank error integrates (a crosswind's dihedral, the torque).

use super::response::{design, Response};

/// The delay is believed from this many ticks of evidence; none before.
const DELAY_MIN_SAMPLES: f64 = 10.0;
/// The measurement is believed within this factor of the prior either way.
const KP_RANGE: f64 = 2.5;
/// The delays tried, in ticks: the aileron that best explains the rate
/// is the one this many ticks old.
const DELAY_TICKS: usize = 4;
/// The trim never takes more than this aileron (the fits' trims are 0.06
/// at most; a 15 kt slip's dihedral asks about 0.2 more).
const TRIM_MAX: f64 = 0.3;
/// The roll rate asked for, at most, deg/s.
const RATE_MAX_DPS: f64 = 10.0;
/// The aileron, at most either way: the bank is capped at 5 degrees near
/// the ground ([`crate::pilot::line`]), so more is never a need.
pub const AIL_MAX: f64 = 0.6;
/// The trim integrates only this near the bank wanted, degrees: integrated
/// through a roll-in it overshot (`plants_tests`).
const TRIM_BAND_DEG: f64 = 1.0;
/// How much the measurement's prior weighs, in squared aileron of evidence.
const KP_PRIOR_WEIGHT: f64 = 0.3;

/// The roll response measured from the ailerons held and the bank's rate:
/// `p = Kp (a_f - a0)`, `a_f` the aileron `n` ticks old low-passed over the
/// roll mode's tau, least squares on the centred sums for each `n` up to
/// [`DELAY_TICKS`], the best-correlated `n` believed, Kp drawn toward the
/// prior (the identification's, scaled to the speed flown). Pure.
#[derive(Debug, Default, Clone, Copy)]
pub struct RollResponse {
    /// The ailerons held, newest first.
    held: [Option<f64>; DELAY_TICKS],
    a_f: [Option<f64>; DELAY_TICKS],
    sums: [[f64; 5]; DELAY_TICKS],
    dt_sum: f64,
}

impl RollResponse {
    /// One tick: the aileron held over it, the bank's rate over it (deg/s),
    /// the roll mode's tau (s).
    pub fn feed(&mut self, ail_held: f64, rate_dps: f64, dt: f64, tau_s: f64) {
        if !(ail_held.is_finite() && rate_dps.is_finite() && dt > 0.0 && dt <= 1.0) {
            return;
        }
        self.held.rotate_right(1);
        self.held[0] = Some(ail_held);
        self.dt_sum += dt;
        let decay = (-dt / tau_s.clamp(0.1, 2.0)).exp();
        for n in 0..DELAY_TICKS {
            let Some(a) = self.held[n] else { continue };
            let before = self.a_f[n].unwrap_or(a);
            let after = a + (before - a) * decay;
            self.a_f[n] = Some(after);
            let x = 0.5 * (before + after);
            let s = &mut self.sums[n];
            s[0] += 1.0;
            s[1] += x;
            s[2] += rate_dps;
            s[3] += x * x;
            s[4] += x * rate_dps;
        }
    }

    /// The delay best explaining the rate, ticks, and its centred sums
    /// (variance, covariance).
    fn best(&self) -> (usize, f64, f64) {
        let mut best = (0, 0.0, 0.0, f64::MIN);
        for n in 0..DELAY_TICKS {
            let [k, sx, sy, sxx, sxy] = self.sums[n];
            if (k < DELAY_MIN_SAMPLES && n > 0) || k < 2.0 {
                continue;
            }
            let var = sxx - sx * sx / k;
            let cov = sxy - sx * sy / k;
            // The share of the rate this delay explains (up to the rate's
            // own variance, the same for every n).
            let score = if var > 1e-12 { cov * cov / var } else { 0.0 };
            if score > best.3 + 1e-12 {
                best = (n, var, cov, score);
            }
        }
        (best.0, best.1, best.2)
    }

    /// Roll rate per unit of aileron, deg/s, drawn toward `prior`.
    pub fn kp(&self, prior: f64) -> f64 {
        // Nothing is believed before the delay can be: through the first
        // roll-in the rate answers an aileron not yet known to be late.
        if self.sums[0][0] < DELAY_MIN_SAMPLES {
            return prior;
        }
        let (_, var, cov) = self.best();
        ((cov + KP_PRIOR_WEIGHT * prior) / (var + KP_PRIOR_WEIGHT)).clamp(prior / KP_RANGE, prior * KP_RANGE)
    }

    /// The delay measured, s.
    pub fn delay_s(&self) -> f64 {
        let n = self.sums[0][0];
        let tick = if n > 0.0 { self.dt_sum / n } else { 0.0 };
        self.best().0 as f64 * tick
    }
}

/// The bank hold: one per hand-flown segment. Pure.
#[derive(Debug, Default, Clone, Copy)]
pub struct BankHold {
    pub response: RollResponse,
    trim: f64,
    last_ail: Option<f64>,
    prev_phi: Option<f64>,
    kp: Option<f64>,
}

impl BankHold {
    /// One tick: the bank read and the bank wanted (degrees, right wing down
    /// positive), the tick, the airframe's roll response and the airspeeds.
    /// Returns the aileron (positive rolls right).
    #[allow(clippy::too_many_arguments)]
    pub fn step(&mut self, phi_deg: f64, target_deg: f64, dt: f64, resp: &Response, ias_kts: f64, tas_kts: f64, beta_deg: f64) -> f64 {
        let dt = dt.clamp(0.05, 1.0);
        let r = resp.at(ias_kts, tas_kts);
        let tau = 1.0 / r.damping.max(0.1);
        let rate = self.prev_phi.map_or(0.0, |p| (phi_deg - p) / dt);
        if let (Some(a), Some(_)) = (self.last_ail, self.prev_phi) {
            self.response.feed(a, rate, dt, tau);
        }
        self.prev_phi = Some(phi_deg);
        // The response flown: the identification's, its Kp refined.
        let kp = self.response.kp(r.rate_gain());
        self.kp = Some(kp);
        let flown = Response { b: kp * r.damping, delay_s: r.delay_s.max(self.response.delay_s()), ..r };
        let g = design(&flown, dt);
        let err = target_deg - phi_deg;
        let want_rate = (g.angle_w * err).clamp(-RATE_MAX_DPS, RATE_MAX_DPS);
        // The dihedral's roll for the slip flown, fed forward (measured on
        // the identification's rudder inputs): a wing held down into a
        // crosswind leans on it, and a trim finds it only seconds late.
        let dihedral = if beta_deg.is_finite() { -r.cross * beta_deg / r.b.max(1e-6) } else { 0.0 };
        let ail = self.trim + dihedral + want_rate * g.ff + g.k_rate * (want_rate - rate);
        // The trim, integrated near the bank wanted and while the aileron
        // has room, at a quarter of the attitude loop's crossover (a PI's
        // zero well below it): an observer of the rate read the aileron a
        // delay late and wound the wrong way through the roll-in (the A330's
        // floated gate, `plants_tests`); integrated through a roll-in it
        // overshot.
        if ail.abs() < AIL_MAX && err.abs() < TRIM_BAND_DEG {
            self.trim = (self.trim + err * g.angle_w * 0.25 * g.angle_w * g.ff * dt).clamp(-TRIM_MAX, TRIM_MAX);
        }
        let ail = ail.clamp(-AIL_MAX, AIL_MAX);
        // What rolled the aircraft beyond holding the slip: the response is
        // measured on it.
        self.last_ail = Some(ail - dihedral);
        ail
    }

    /// The roll response measured so far, deg/s per unit (the prior's
    /// until the segment has flown).
    pub fn kp(&self) -> Option<f64> {
        self.kp
    }
}
