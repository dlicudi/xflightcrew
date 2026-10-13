//! How an axis answers its control, MEASURED, and the loop gains that
//! follow from it by rule (docs/tasks/autonomy-layers, 2026-10-13).
//!
//! The model per axis ([`Response`], stored per airframe by the
//! identification, `aircraft::learned`) is the one the literature identifies
//! and the 2026-10-12 tapes fit (`plants_tests`): the axis' rate `x` answers
//! the control `u`, `d` seconds late, as `x' = b u(t - d) - damping x -
//! stiffness s`, `s` the airflow angle the axis turns (the pitch's alpha,
//! the yaw's sideslip) and `s' = x - path s`. `b` is the control
//! EFFECTIVENESS (deg/s^2 per unit: what INDI needs, Grondman et al. 2018),
//! `damping` the rate's own decay (the roll mode's 1/tau). Scaled by the air
//! ([`Response::at`]): an identification at 160 kt serves an approach at 140.
//!
//! **Gains by McRuer's crossover rule** ([`design`]): a loop whose delay is
//! `td` crosses over at `(pi/2 - PM) / td` for a phase margin `PM`; the rate
//! loop's gain is that crossover over the effectiveness, the attitude loop
//! crosses over where the closed rate loop's own lag and the delay leave the
//! same margin. ONE number, the margin, for every airframe and every axis;
//! the rest is measured. On the 172's pitch fit it gives the attitude loop
//! 0.12 to 0.15 elevator a degree, where the hand tuning of 2026-10-11 had
//! arrived at 0.12.

pub use crate::aircraft::learned::Response;

/// The phase margin every loop is designed to, radians (50 degrees: the
/// middle of the 45 to 60 a crossover design aims for).
pub const PHASE_MARGIN_RAD: f64 = 50.0 * std::f64::consts::PI / 180.0;
/// What a command takes to reach the plugin's flight loop, s: a Web API
/// write and the next frame (14 ms median a request, 2026-10-11, and a
/// frame to apply it).
pub const TRANSPORT_S: f64 = 0.05;

/// A loop's numbers, derived by [`design`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gains {
    /// The rate loop's crossover, rad/s.
    pub rate_w: f64,
    /// The attitude loop's crossover, rad/s: the angle error asks this
    /// much rate per unit of error (1/s).
    pub angle_w: f64,
    /// Control per deg/s of rate error.
    pub k_rate: f64,
    /// Control per deg/s of rate wanted, fed forward: 1 / the rate gain.
    pub ff: f64,
    /// The loop's whole delay, s: measured, half the tick, the transport.
    pub delay_s: f64,
}

/// The gains for a response flown on a tick of `tick_s`. Pure.
pub fn design(r: &Response, tick_s: f64) -> Gains {
    let delay_s = r.delay_s.max(0.0) + tick_s.clamp(0.0, 1.0) / 2.0 + TRANSPORT_S;
    let budget = std::f64::consts::FRAC_PI_2 - PHASE_MARGIN_RAD;
    let rate_w = budget / delay_s;
    // The closed rate loop answers in about 1 / rate_w, on top of the delay.
    let angle_w = budget / (delay_s + 1.0 / rate_w);
    // |L| = k b / |jw + damping| = 1 at the crossover.
    let k_rate = (rate_w * rate_w + r.damping * r.damping).sqrt() / r.b.max(1e-6);
    Gains { rate_w, angle_w, k_rate, ff: 1.0 / r.rate_gain().max(1e-6), delay_s }
}

/// The closed attitude loop's time constant, s: what an outer loop (the
/// centreline's period, [`crate::pilot::line`]) must stay slower than.
pub fn attitude_tc_s(g: &Gains) -> f64 {
    1.0 / g.angle_w.max(1e-6) + g.delay_s
}

/// Fleet priors, flown before anything is measured: the middle of the six
/// airframes fitted on 2026-10-12 (`plants_tests`), at a light aircraft's
/// approach. An identification replaces them (`aircraft::learned`).
pub fn prior_pitch() -> Response {
    Response { b: 120.0, damping: 2.0, stiffness: 11.0, path: 1.6, cross: 0.0, delay_s: 0.05, ias_kts: 75.0, tas_kts: 75.0 }
}

/// Kp 16 deg/s per unit through a 0.45 s roll mode (the six fits' middle).
pub fn prior_roll() -> Response {
    Response { b: 16.0 / 0.45, damping: 1.0 / 0.45, stiffness: 0.0, path: 0.0, cross: 0.0, delay_s: 0.08, ias_kts: 75.0, tas_kts: 75.0 }
}

/// A light aircraft's yaw: full rudder holds about 15 degrees of slip.
pub fn prior_yaw() -> Response {
    Response { b: 45.0, damping: 1.2, stiffness: 3.0, path: 0.25, cross: 0.0, delay_s: 0.08, ias_kts: 75.0, tas_kts: 75.0 }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_air_scales_the_response() {
        let r = prior_roll();
        let fast = r.at(150.0, 150.0);
        assert!((fast.b / r.b - 4.0).abs() < 1e-9, "a moment goes as IAS squared");
        assert!((fast.rate_gain() / r.rate_gain() - 2.0).abs() < 1e-9, "the held rate as TAS");
        // Thin air: the same IAS at a higher TAS damps less.
        let high = r.at(75.0, 90.0);
        assert!(high.damping < r.damping && (high.b - r.b).abs() < 1e-9);
        assert_eq!(r.at(f64::NAN, 80.0), r, "an unread speed keeps the measurement");
    }

    /// The crossover rule on the 172's pitch fit: the attitude loop asks
    /// 0.08 to 0.16 elevator a degree (the hand tuning of 2026-10-11: 0.12),
    /// and a longer delay lowers every crossover.
    #[test]
    fn the_crossover_rule_gives_the_hand_tuned_numbers() {
        let pitch = Response { b: 117.0, damping: 1.5, stiffness: 10.3, path: 2.3, cross: 0.0, delay_s: 0.02, ias_kts: 67.0, tas_kts: 67.0 };
        let g = design(&pitch, 0.2);
        let per_deg = g.angle_w * (g.ff + g.k_rate);
        assert!((0.08..0.16).contains(&per_deg), "elevator per degree {per_deg}");
        let late = design(&Response { delay_s: 0.3, ..pitch }, 0.2);
        assert!(late.rate_w < g.rate_w && late.angle_w < g.angle_w);
    }
}
