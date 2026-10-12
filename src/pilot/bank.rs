//! Ailerons to bank, on the roll response measured in flight.
//!
//! The servos-out arrests of 2026-10-12 (59 segments, six airframes,
//! `plants_tests`) fit ONE shape: the roll rate follows the aileron through
//! a first-order roll mode, `p' = (Kp (a - a0) - p) / tau`, with Kp 12 to
//! 19 deg/s per unit, tau 0.3 to 0.6 s, at most 0.15 s of delay, and a trim
//! a0 within 0.06 (R2 0.91 to 0.99); the A330's sidestick reads 11 to 13
//! after 0.25 to 0.37 s (#180). So the ailerons ask for a roll RATE: the
//! bank error closed over [`BANK_CLOSE_S`], judged where the bank will be
//! once the aileron already written has arrived (the measured delay); the
//! aileron for that rate is the rate over the measured Kp, on top of the
//! trim the bank error integrates (a crosswind's dihedral, the torque).

/// The roll mode's time the measurement assumes, s: the fleet's 0.3 to 0.6.
pub const ROLL_TAU_S: f64 = 0.45;
/// Roll rate per unit of aileron before anything is measured, deg/s: the
/// middle of the six airframes' 12 to 19.
pub const KP_PRIOR: f64 = 16.0;
/// The delay is believed from this many ticks of evidence; none before.
const DELAY_MIN_SAMPLES: f64 = 10.0;
/// The measurement is believed within this factor of the prior either way.
const KP_RANGE: f64 = 2.5;
/// The delays tried, in ticks: the aileron that best explains the rate
/// is the one this many ticks old.
const DELAY_TICKS: usize = 4;
/// The trim never takes more than this aileron (the fits' trims are 0.06
/// at most).
const TRIM_MAX: f64 = 0.15;
/// The roll rate asked for, at most, deg/s.
const RATE_MAX_DPS: f64 = 10.0;
/// The aileron, at most either way.
pub const AIL_MAX: f64 = 0.6;

/// The loop's numbers, one set for every airframe, chosen on the six fitted
/// roll plants and the A330's recorded gates (`plants_tests`): over a sweep
/// of these four the gusty final moved 2.6 to 3.0 m and the bank's overshoot
/// of its 5 degree cap 0.7 to 2.8 degrees; integrated through the roll-in
/// (no band) the trim overshot most. The bank error is closed over
/// `BANK_CLOSE_S`, judged `BANK_LEAD_S` ahead on its rate beyond the delay;
/// the trim (what holds the wings level: a crosswind's dihedral, the torque)
/// is the bank error integrated over `TRIM_S`, within `TRIM_BAND_DEG` of the
/// bank wanted.
pub const BANK_CLOSE_S: f64 = 0.7;
const BANK_LEAD_S: f64 = 0.4;
const TRIM_S: f64 = 2.0;
const TRIM_BAND_DEG: f64 = 1.0;
/// How much the measurement's prior weighs, in squared aileron of evidence.
const KP_PRIOR_WEIGHT: f64 = 0.3;

/// The roll response measured from the ailerons held and the bank's rate:
/// `p = Kp (a_f - a0)`, `a_f` the aileron `n` ticks old low-passed over
/// [`ROLL_TAU_S`], least squares on the centred sums for each `n` up to
/// [`DELAY_TICKS`], the best-correlated `n` believed, Kp drawn toward
/// [`KP_PRIOR`]. Pure.
#[derive(Debug, Default, Clone, Copy)]
pub struct RollResponse {
    /// The ailerons held, newest first.
    held: [Option<f64>; DELAY_TICKS],
    a_f: [Option<f64>; DELAY_TICKS],
    sums: [[f64; 5]; DELAY_TICKS],
    dt_sum: f64,
}

impl RollResponse {
    /// One tick: the aileron held over it, the bank's rate over it (deg/s).
    pub fn feed(&mut self, ail_held: f64, rate_dps: f64, dt: f64) {
        if !(ail_held.is_finite() && rate_dps.is_finite() && dt > 0.0 && dt <= 1.0) {
            return;
        }
        self.held.rotate_right(1);
        self.held[0] = Some(ail_held);
        self.dt_sum += dt;
        let decay = (-dt / ROLL_TAU_S).exp();
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
            if k < DELAY_MIN_SAMPLES && n > 0 {
                continue;
            }
            if k < 2.0 {
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

    /// Roll rate per unit of aileron, deg/s.
    pub fn kp(&self) -> f64 {
        // Nothing is believed before the delay can be: through the first
        // roll-in the rate answers an aileron not yet known to be late.
        if self.sums[0][0] < DELAY_MIN_SAMPLES {
            return KP_PRIOR;
        }
        let (_, var, cov) = self.best();
        ((cov + KP_PRIOR_WEIGHT * KP_PRIOR) / (var + KP_PRIOR_WEIGHT))
            .clamp(KP_PRIOR / KP_RANGE, KP_PRIOR * KP_RANGE)
    }

    /// The delay measured, ticks.
    pub fn delay_ticks(&self) -> usize {
        self.best().0
    }

    /// The delay measured, s.
    pub fn delay_s(&self) -> f64 {
        let n = self.sums[0][0];
        let tick = if n > 0.0 { self.dt_sum / n } else { 0.0 };
        self.delay_ticks() as f64 * tick
    }
}

/// The bank hold: one per hand-flown segment. Pure.
#[derive(Debug, Default, Clone, Copy)]
pub struct BankHold {
    pub response: RollResponse,
    trim: f64,
    last_ail: Option<f64>,
    prev_phi: Option<f64>,
}

impl BankHold {
    /// One tick: the bank read and the bank wanted (degrees, right wing down
    /// positive), the tick. Returns the aileron (positive rolls right).
    pub fn step(&mut self, phi_deg: f64, target_deg: f64, dt: f64) -> f64 {
        let dt = dt.clamp(0.05, 1.0);
        let rate = self.prev_phi.map_or(0.0, |p| (phi_deg - p) / dt);
        if let (Some(a), Some(_)) = (self.last_ail, self.prev_phi) {
            self.response.feed(a, rate, dt);
        }
        self.prev_phi = Some(phi_deg);
        let kp = self.response.kp();
        // Where the bank goes once the aileron on its way has arrived.
        let ahead = rate * (BANK_LEAD_S + self.response.delay_s());
        let err = target_deg - phi_deg;
        let want_rate = ((err - ahead) / BANK_CLOSE_S).clamp(-RATE_MAX_DPS, RATE_MAX_DPS);
        let ail = self.trim + want_rate / kp;
        // The trim, integrated near the bank wanted and while the aileron
        // has room: an observer of the rate read the aileron a delay late
        // and wound the wrong way through the roll-in (the A330's floated
        // gate, `plants_tests`); integrated through a roll-in it overshot.
        if ail.abs() < AIL_MAX && err.abs() < TRIM_BAND_DEG {
            self.trim =
                (self.trim + err / (BANK_CLOSE_S * TRIM_S) * dt / kp).clamp(-TRIM_MAX, TRIM_MAX);
        }
        let ail = ail.clamp(-AIL_MAX, AIL_MAX);
        self.last_ail = Some(ail);
        ail
    }

    /// The roll response measured so far, deg/s per unit.
    pub fn kp(&self) -> f64 {
        self.response.kp()
    }
}
