//! The flare flown by ATTITUDE, judged by HEIGHT (docs/tasks/autonomy-layers,
//! phase 2). Nothing in it is fitted to an airframe.
//!
//! The G1000 172 on LEMG 13 (2026-10-11, 1791739518) gave the measurements
//! it rests on:
//! - the flight path is the nose's attitude less the angle of attack, read
//!   with no lag: 0.9 deg nose down at 1.8 deg of alpha is a 2.7 deg path,
//!   -300 fpm at 63 kt, where the true vertical velocity read -296 and the
//!   VSI still -177;
//! - the nose answers the elevator in 0.2 to 0.4 s (+0.08 of elevator took
//!   the pitch rate from 1.1 to 2.4 deg/s in 0.4 s);
//! - the old law flew the VSI, a second late: it pulled the 172 into a
//!   balloon, let the nose fall from 4.3 to -1 deg and touched at 463 fpm.
//!
//! So: hold the path the aircraft is on until the flare height, then shrink
//! the sink in proportion to the height left down to the touchdown sink.
//! The wanted path becomes a nose attitude (the path plus a slowly filtered
//! angle of attack), and the elevator holds that attitude through the pitch
//! rate. The flare height is the sink times the time the aircraft needs to
//! change its path ([`RESPONSE_S`]), not a fixed number of feet: 18 ft for
//! a 172 at 360 fpm, sinking less with every foot, where the old law asked
//! for -100 fpm at once from a fixed 25 ft.

/// Seconds the aircraft needs to change its flight path: the nose's answer
/// (0.2 to 0.4 s measured) plus the path following it (0.7 to 1.5 s), with
/// the margin a weak elevator needs (2 s landed a steep arrival at 300 fpm
/// on half the authority, `attitude_flare_tests`).
pub const RESPONSE_S: f64 = 3.0;
/// The sink at the wheels, fpm.
pub const TOUCHDOWN_VS_FPM: f64 = -100.0;
/// Never flare lower than this, ft (a very shallow approach).
const MIN_GATE_FT: f64 = 6.0;
/// The loop's numbers: one set for every airframe, chosen to land across
/// half to double the 172's measured elevator authority and a path lag of
/// 0.7 to 1.5 s (`attitude_flare_tests`), never fitted to one aircraft.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gains {
    /// Elevator per degree of attitude error.
    pub k_theta: f64,
    /// Elevator per deg/s of pitch rate (damping).
    pub k_q: f64,
    /// The trim found by the attitude error, elevator per degree-second:
    /// the pull the bleeding speed needs.
    pub ki_theta: f64,
    /// How long the path takes to follow the nose, seconds: the attitude
    /// asked leads the path's change by this much.
    pub path_lag_s: f64,
    /// Seconds the aircraft needs to change its path: the flare height is
    /// the sink times this ([`gate_ft`]).
    pub response_s: f64,
}

impl Default for Gains {
    fn default() -> Self {
        Gains { k_theta: 0.08, k_q: 0.02, ki_theta: 0.005, path_lag_s: 1.5, response_s: RESPONSE_S }
    }
}

/// The angle of attack is filtered this slowly, seconds: the nose's own
/// short-period swing in alpha must not feed back into the attitude asked.
const ALPHA_TAU_S: f64 = 1.5;
/// The path error is trimmed out of the attitude this slowly, deg per
/// second per deg of error.
const GAMMA_TRIM_PER_S: f64 = 0.5;
/// Knots to feet a minute.
const FPM_PER_KT: f64 = 101.268_591;

/// One tick's measurements. Attitudes in degrees, the pitch rate in deg/s.
#[derive(Debug, Clone, Copy)]
pub struct In {
    pub radio_ft: f64,
    pub theta_deg: f64,
    pub alpha_deg: f64,
    pub q_dps: f64,
    pub tas_kts: f64,
    pub dt: f64,
}

/// What the law asked for this tick (the blackbox's `att_*`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Out {
    pub elev: f64,
    pub gate_ft: f64,
    pub vs_target_fpm: f64,
    pub gamma_deg: f64,
    pub gamma_cmd_deg: f64,
    pub theta_cmd_deg: f64,
}

/// The law's memory for one approach: reset by a go-around.
#[derive(Debug, Clone, Copy, Default)]
pub struct AttitudeFlare {
    pub gains: Gains,
    entry: Option<Entry>,
    alpha_f: f64,
    gamma_trim: f64,
    integ: f64,
}

#[derive(Debug, Clone, Copy)]
struct Entry {
    vs_fpm: f64,
    gate_ft: f64,
}

/// The flare height for a sink, ft: the sink times the response time. Pure.
pub fn gate_ft(vs_fpm: f64, response_s: f64) -> f64 {
    (vs_fpm.abs() / 60.0 * response_s).max(MIN_GATE_FT)
}

/// The sink wanted at this height: the entry's path above the gate, then in
/// proportion to the height left down to the touchdown sink. Pure.
pub fn vs_target_fpm(radio_ft: f64, entry_vs_fpm: f64, gate_ft: f64) -> f64 {
    let entry = entry_vs_fpm.min(TOUCHDOWN_VS_FPM);
    let k = (radio_ft / gate_ft).clamp(0.0, 1.0);
    TOUCHDOWN_VS_FPM + (entry - TOUCHDOWN_VS_FPM) * k
}

impl AttitudeFlare {
    pub fn reset(&mut self) {
        *self = Self { gains: self.gains, ..Self::default() };
    }

    /// One tick: the elevator (yoke ratio, positive aft) within `push` and
    /// `pull`. The first tick takes the path it finds as the one to hold.
    pub fn step(&mut self, i: In, pull: f64, push: f64) -> Out {
        let g = self.gains;
        let dt = i.dt.clamp(0.02, 0.5);
        let v_fpm = i.tas_kts.max(30.0) * FPM_PER_KT;
        let gamma = i.theta_deg - i.alpha_deg;
        let entry = *self.entry.get_or_insert_with(|| {
            self.alpha_f = i.alpha_deg;
            let vs = v_fpm * gamma.to_radians().sin();
            Entry { vs_fpm: vs, gate_ft: gate_ft(vs, g.response_s) }
        });
        self.alpha_f += (i.alpha_deg - self.alpha_f) * (dt / ALPHA_TAU_S).min(1.0);
        let vs_t = vs_target_fpm(i.radio_ft, entry.vs_fpm, entry.gate_ft);
        let gamma_cmd = (vs_t / v_fpm).clamp(-1.0, 1.0).asin().to_degrees();
        // The target moves with the height, predictably: below the gate the
        // wanted sink falls by (entry - touchdown) / gate per foot, and the
        // height falls at the sink. Led, not chased (a lag chain tracking a
        // ramp arrives late by its own lag).
        let below = i.radio_ft > 0.0 && i.radio_ft < entry.gate_ft;
        let gamma_rate = if below {
            let dvs_dt = (entry.vs_fpm.min(TOUCHDOWN_VS_FPM) - TOUCHDOWN_VS_FPM) / entry.gate_ft
                * (v_fpm * gamma.to_radians().sin() / 60.0);
            (dvs_dt / v_fpm).to_degrees()
        } else {
            0.0
        };
        // The filtered alpha holds the path on the attitude; what the filter
        // misses (the speed bleeding, alpha rising) is trimmed out slowly.
        self.gamma_trim =
            (self.gamma_trim + (gamma_cmd - gamma) * GAMMA_TRIM_PER_S * dt).clamp(-3.0, 3.0);
        let theta_cmd = gamma_cmd + self.alpha_f + self.gamma_trim + g.path_lag_s * gamma_rate;
        let e_theta = theta_cmd - i.theta_deg;
        let raw = self.integ + g.k_theta * e_theta - g.k_q * (i.q_dps - gamma_rate);
        let elev = raw.clamp(-push, pull);
        // No wind-up against a limit: the trim only moves inward there.
        if raw == elev || (raw > pull && e_theta < 0.0) || (raw < -push && e_theta > 0.0) {
            self.integ = (self.integ + g.ki_theta * e_theta * dt).clamp(-push, pull);
        }
        Out {
            elev,
            gate_ft: entry.gate_ft,
            vs_target_fpm: vs_t,
            gamma_deg: gamma,
            gamma_cmd_deg: gamma_cmd,
            theta_cmd_deg: theta_cmd,
        }
    }
}

#[cfg(test)]
#[path = "attitude_flare_tests.rs"]
mod tests;
