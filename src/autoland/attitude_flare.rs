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
//! angle of attack), and the elevator holds that attitude: the command's
//! rate fed forward, on top of what a disturbance observer reads holding the
//! nose takes (ground effect, the bleeding speed). The flare height is the sink times the time the aircraft needs to
//! change its path ([`RESPONSE_S`]), not a fixed number of feet: 18 ft for
//! a 172 at 360 fpm, sinking less with every foot, where the old law asked
//! for -100 fpm at once from a fixed 25 ft.

/// Seconds the aircraft needs to change its flight path: the nose's answer
/// (0.2 to 0.4 s measured) plus the path following it (0.7 to 1.5 s), with
/// the margin a weak elevator needs (2 s landed a steep arrival at 300 fpm
/// on half the authority, `attitude_flare_tests`).
pub const RESPONSE_S: f64 = 3.0;
/// The hold-off: the sink the flare eases to by [`HOLDOFF_FT`] and holds
/// to the wheels, fpm. The flare is a round-out THEN a hold-off: the wheels
/// kept just off, the power at idle, the nose rising as the speed bleeds
/// until the aircraft settles on its mains. Easing to -100 at the wheels, the
/// 172 touched flat (1.5 to 1.7 deg nose up) at 62 kt off a 66.5 kt final:
/// a flare that looked weak (LEMG 13, 1791755237, 1791755825).
pub const HOLDOFF_VS_FPM: f64 = -70.0;
/// The height the round-out ends and the hold-off begins, ft.
pub const HOLDOFF_FT: f64 = 2.0;
/// Never flare lower than this, ft (a very shallow approach).
const MIN_GATE_FT: f64 = 6.0;
/// The loop's numbers: one set for every airframe, chosen to land across
/// half to double the 172's measured elevator authority and a path lag of
/// 0.7 to 1.5 s (`attitude_flare_tests`), never fitted to one aircraft.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gains {
    /// Elevator per degree of attitude error.
    pub k_theta: f64,
    /// Elevator per deg/s of pitch rate away from the command's own rate
    /// (damping).
    pub k_q: f64,
    /// How long the path takes to follow the nose, seconds: the attitude
    /// asked leads the path's change by this much.
    pub path_lag_s: f64,
    /// Seconds the aircraft needs to change its path: the flare height is
    /// the sink times this ([`gate_ft`]).
    pub response_s: f64,
    /// The disturbance observer's time, seconds: the elevator the aircraft
    /// needs to hold its nose (ground effect, the bleeding speed), read from
    /// what it holds and how the nose answers.
    pub dob_tau_s: f64,
    /// Pitch rate per unit of elevator, deg/s: the 172's measured 32 (LEMG
    /// 13, 1791739518), tested at half and double. The observer reads with
    /// it and the command's rate is fed forward through it; with the nose
    /// steady the observer's estimate is the elevator held, whatever it is.
    pub b_hat: f64,
}

impl Default for Gains {
    fn default() -> Self {
        Gains { k_theta: 0.12, k_q: 0.02, path_lag_s: 1.0, response_s: RESPONSE_S, dob_tau_s: 1.0, b_hat: 32.0 }
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
    last_elev: f64,
    last_theta_cmd: Option<f64>,
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
///
/// A SMOOTHSTEP in height, not a line: its slope is zero at the gate and at
/// the hold-off, so the rate fed forward never steps. As a line, the slope
/// cut to zero at 2 ft, the pull dropped 0.30 to 0.19 in one tick, the nose
/// pitched down at 1.7 deg/s and the full 0.35 could not lift it again
/// before the wheels: touched at 3.2 deg nose up against a command of 3.9
/// (LEMG 13, 1791759560).
pub fn vs_target_fpm(radio_ft: f64, entry_vs_fpm: f64, gate_ft: f64) -> f64 {
    let entry = entry_vs_fpm.min(HOLDOFF_VS_FPM);
    HOLDOFF_VS_FPM + (entry - HOLDOFF_VS_FPM) * smoothstep(rounding_out_k(radio_ft, gate_ft))
}

/// How far up the round-out the height is: 0 at the hold-off, 1 at the gate.
fn rounding_out_k(radio_ft: f64, gate_ft: f64) -> f64 {
    ((radio_ft - HOLDOFF_FT) / (gate_ft - HOLDOFF_FT).max(1.0)).clamp(0.0, 1.0)
}

fn smoothstep(k: f64) -> f64 {
    k * k * (3.0 - 2.0 * k)
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
        // The target's slope in height (the smoothstep's, zero at both
        // ends) times the height's own rate: led, not chased.
        let k = rounding_out_k(i.radio_ft, entry.gate_ft);
        let dvs_dh = (entry.vs_fpm.min(HOLDOFF_VS_FPM) - HOLDOFF_VS_FPM) * 6.0 * k * (1.0 - k)
            / (entry.gate_ft - HOLDOFF_FT).max(1.0);
        let gamma_rate = (dvs_dh * (v_fpm * gamma.to_radians().sin() / 60.0) / v_fpm).to_degrees();
        // The filtered alpha holds the path on the attitude; what the filter
        // misses (the speed bleeding, alpha rising) is trimmed out slowly.
        self.gamma_trim =
            (self.gamma_trim + (gamma_cmd - gamma) * GAMMA_TRIM_PER_S * dt).clamp(-3.0, 3.0);
        let theta_cmd = gamma_cmd + self.alpha_f + self.gamma_trim + g.path_lag_s * gamma_rate;
        let e_theta = theta_cmd - i.theta_deg;
        // The command's own rate (the speed bleeding, alpha rising, the
        // path easing): fed forward as the elevator that turns the nose at
        // that rate, so it rises WITH the command. Chased, it lagged 0.8 deg
        // below 8 ft and the 172 landed at 185 to 199 fpm on a command of
        // -110 (LEMG 13, 1791755237, 1791755825).
        let theta_rate = self.last_theta_cmd.map_or(0.0, |p| (theta_cmd - p) / dt);
        self.last_theta_cmd = Some(theta_cmd);
        // What holding the nose takes: the elevator held, less what the
        // nose's rate says it is doing beyond holding. Ground effect and the
        // bleeding speed show up here within a second; a slow trim could not
        // find them in the flare's three (226 to 287 fpm, LEMG 2026-10-11).
        let need = self.last_elev - i.q_dps / g.b_hat;
        self.integ = (self.integ + (need - self.integ) * (dt / g.dob_tau_s).min(1.0)).clamp(-push, pull);
        let elev = (self.integ + g.k_theta * e_theta + theta_rate / g.b_hat - g.k_q * (i.q_dps - theta_rate))
            .clamp(-push, pull);
        self.last_elev = elev;
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
