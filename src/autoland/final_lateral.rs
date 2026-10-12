//! The final's lateral: ONE law from the start of the final to the arrest
//! (2026-10-12, designed from 60 recorded finals on seven airframes).
//!
//! One plant fits every airframe flown: the autopilot's heading mode banks
//! in proportion to a low-passed bug less heading, through a roll lag, and
//! the track turns at g tan(bank) / TAS (`final_lateral_tests`). What
//! differs is how fast: the time the heading takes to answer the bug,
//! `t_r`, V / (g K) plus the lags, 2.1 s on the SF50 to 7.5 s on the Zibo.
//! So the law is written in that one number, measured in flight
//! ([`ResponseTime`]): the offset is closed over `t_r`, judged `t_r` ahead
//! on the drift over the ground, with an integral that takes out what the
//! autopilot holds off its bug, plus the wind's crab.
//!
//! It replaced three laws that handed over to each other: the fixed-lead
//! pursuit (0.6 nm, a degree for 36 m at 135 kt), the short final's 6 s
//! closure from 0.3 nm with its bias learner, and the declared track law
//! (`final_track:`, the SR22 and the Zibo). The switch at 0.3 nm jumped the
//! bug 2 to 6 degrees and its 6 s closure ran against a 5 to 6 s response:
//! the SR22 (1791810954) and the 172 (1791808122) went round
//! `off_centreline_low`. The learner's 2.5 degree clamp had to carry the
//! whole crab: in a 15 kt crosswind 4 of the 60 finals passed. The
//! pursuit's lead with the half-degree knob deadband weaved 7 to 12 m.

/// What the in-flight estimate says when it has nothing to go on, s.
pub const TR_DEFAULT_S: f64 = 6.0;
/// The estimate's bounds, s: the fitted airframes' 2.1 to 7.5 s answer, read
/// in flight 3.2 to 7.7 (yaw noise reads as error the track never answers).
pub const TR_MIN_S: f64 = 3.0;
pub const TR_MAX_S: f64 = 10.0;
/// The bug-less-heading error is low-passed over this long before it is
/// paired with the track's rate (the autopilot's own filter and roll lag).
pub const TR_ERR_TAU_S: f64 = 1.5;
/// An error bigger than this is a turn being steered, not a correction
/// answered, degrees.
pub const TR_ERR_MAX_DEG: f64 = 10.0;
/// Ticks of evidence before the estimate is believed.
pub const TR_MIN_SAMPLES: u32 = 40;
/// The estimate is taken on the intercept and this much of the first
/// final, s; frozen after it.
pub const TR_FINAL_WINDOW_S: f64 = 90.0;
/// ...and only above this radio height, ft.
pub const TR_MIN_RADIO_FT: f64 = 300.0;

/// The integral's time is this many response times...
pub const LATERAL_I_TIMES: f64 = 4.0;
/// ...and it carries no more than this, metres: an autopilot settling off
/// its bug (the 172 1.1 to 1.9 degrees, LEMG 13, 2026-09-12; the fitted
/// roll biases, -0.1 to -0.6 degrees of bank) is metres, not tens.
pub const LATERAL_I_MAX_M: f64 = 8.0;
/// The largest cut off the course, degrees: a final handed over far off
/// closes, it is not capped into a parallel.
pub const LATERAL_MAX_CUT_DEG: f64 = 30.0;

/// The heading mode's response time measured in flight: the low-passed
/// knob less heading against the rate the track turns at, least squares,
/// `t_r = sum(e e) / sum(e rate)`. Pure.
#[derive(Debug, Default, Clone, Copy)]
pub struct ResponseTime {
    ef: Option<f64>,
    last: Option<(f64, f64)>,
    ee: f64,
    er: f64,
    n: u32,
}

impl ResponseTime {
    /// One tick: the knob as set and the heading (true), the ground track,
    /// the tick's `dt`, and whether this tick is evidence (`counts`: the
    /// loop's own lateral on the intercept or the first final's window,
    /// above [`TR_MIN_RADIO_FT`]). The filter runs on every tick.
    pub fn feed(
        &mut self,
        knob: Option<f64>,
        psi: Option<f64>,
        trk: Option<f64>,
        dt: f64,
        counts: bool,
    ) {
        let (Some(k), Some(psi), Some(trk)) = (knob, psi, trk) else {
            self.last = None;
            return;
        };
        if !(dt > 0.0 && dt <= 3.0) {
            self.last = None;
            return;
        }
        let e = crate::angles::signed_diff_deg(k, psi);
        let ef = match self.ef {
            Some(f) => f + (dt / TR_ERR_TAU_S).min(1.0) * (e - f),
            None => e,
        };
        self.ef = Some(ef);
        // The rate over the tick, paired with the error at its middle.
        if let Some((ef0, trk0)) = self.last.replace((ef, trk)) {
            let mid = 0.5 * (ef + ef0);
            let rate = crate::angles::signed_diff_deg(trk, trk0) / dt;
            if counts && mid.abs() < TR_ERR_MAX_DEG && rate.is_finite() {
                self.ee += mid * mid;
                self.er += mid * rate;
                self.n += 1;
            }
        }
    }

    /// Has the estimate enough evidence to be believed?
    pub fn measured(&self) -> bool {
        self.n >= TR_MIN_SAMPLES && self.er > 0.0
    }

    /// The ticks of evidence so far.
    pub fn samples(&self) -> u32 {
        self.n
    }

    /// The response time, s: the estimate within its bounds, else
    /// [`TR_DEFAULT_S`].
    pub fn t_r(&self) -> f64 {
        if self.measured() {
            (self.ee / self.er).clamp(TR_MIN_S, TR_MAX_S)
        } else {
            TR_DEFAULT_S
        }
    }
}

/// The final's lateral law: the bug (true) that closes the offset over the
/// response time, led on the drift, trimmed by the integral, crabbed into
/// the wind. Pure.
#[derive(Debug, Default, Clone, Copy)]
pub struct FinalLateral {
    i_m: f64,
    last: Option<f64>,
}

impl FinalLateral {
    /// One tick: `xtk_m` and `side_m_s` right positive (the offset and the
    /// drift over the ground), the course and the crab (heading less track,
    /// low-passed: `CrabFilter`) in degrees, `t_r` the heading mode's
    /// response ([`ResponseTime::t_r`]). Returns the bug, degrees true.
    #[allow(clippy::too_many_arguments)]
    pub fn bug_true(
        &mut self,
        xtk_m: f64,
        side_m_s: f64,
        gs_kts: f64,
        course_deg: f64,
        crab_deg: f64,
        t_r: f64,
        dt: f64,
    ) -> f64 {
        if !xtk_m.is_finite() || !side_m_s.is_finite() {
            return self.held(course_deg, crab_deg);
        }
        let t_r = if t_r.is_finite() {
            t_r.clamp(TR_MIN_S, TR_MAX_S)
        } else {
            TR_DEFAULT_S
        };
        let dt = if dt.is_finite() {
            dt.clamp(0.0, 3.0)
        } else {
            0.0
        };
        self.i_m = (self.i_m + xtk_m * dt / (LATERAL_I_TIMES * t_r))
            .clamp(-LATERAL_I_MAX_M, LATERAL_I_MAX_M);
        let v = (if gs_kts.is_finite() { gs_kts } else { 0.0 } * 0.514_444).max(30.0);
        let ahead = xtk_m + side_m_s * t_r + self.i_m;
        let cut = (-(ahead / (v * t_r)).atan().to_degrees())
            .clamp(-LATERAL_MAX_CUT_DEG, LATERAL_MAX_CUT_DEG);
        let bug = (course_deg + cut + crab_deg).rem_euclid(360.0);
        self.last = Some(bug);
        bug
    }

    /// A tick with no track read: the bug of the tick before, else the
    /// course crabbed.
    pub fn held(&self, course_deg: f64, crab_deg: f64) -> f64 {
        self.last
            .unwrap_or_else(|| (course_deg + crab_deg).rem_euclid(360.0))
    }

    /// The integral carried, metres.
    pub fn i_m(&self) -> f64 {
        self.i_m
    }

    /// A new final starts from nothing.
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

#[cfg(test)]
#[path = "final_lateral_tests.rs"]
mod tests;
