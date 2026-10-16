//! The rudder, on the yaw response measured (2026-10-13).
//!
//! Two jobs, the established ones:
//! - **Coordination** in the hand-flown segment: the sideslip held at zero,
//!   as a yaw damper and turn coordinator do. The yaw rate wanted is the
//!   turn's own (`g / V sin bank`) plus what takes the sideslip out at the
//!   attitude loop's crossover.
//! - **The decrab** in the flare: the nose aligned with the runway, and the
//!   aileron (the track law, [`crate::pilot::line`]) holding the track with
//!   a wing down into the wind. That is the sideslip technique the
//!   handbooks teach; X-Plane's own autoland "kicks the crab out" the same
//!   way. It begins once the wheels are the yaw loop's settling time away
//!   (three of its time constants and its delay, [`decrab_from_s`]), so the
//!   drift the slip builds before the wheels is small.
//!
//! The yaw axis is the one the identification fits as `r' = b u - damping r
//! - stiffness beta` (beta X-Plane's: the nose right of the airpath
//! positive), and the rudder is NDI on it: the weathercock's moment
//! (`stiffness beta`) fed forward, the rate flown by its damping and the
//! rate loop's gain, a slow trim on what is left (the engines' asymmetry,
//! the propeller's slipstream). The rollout's ground steering is the
//! rollout's: this law ends at the wheels.

use super::response::{attitude_tc_s, design, Response};

const G_M_S2: f64 = 9.81;
/// The yaw rate asked for, at most, deg/s: a decrab of a 15 kt crab on a
/// 65 kt 172 (13 degrees) in about two seconds.
const RATE_MAX_DPS: f64 = 8.0;
/// The rudder, at most either way: all of it.
const RUD_MAX: f64 = 1.0;
/// The trim never takes more than this rudder.
const TRIM_MAX: f64 = 0.3;

/// What the rudder is doing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// The sideslip to zero.
    Coordinate,
    /// The nose to the runway's course (true, degrees).
    Decrab,
}

/// One tick's measurements. Degrees, deg/s, kt.
#[derive(Debug, Clone, Copy)]
pub struct In {
    /// Heading, true.
    pub psi_deg: f64,
    /// X-Plane's sideslip: the nose right of the airpath positive.
    pub beta_deg: f64,
    pub r_dps: f64,
    pub phi_deg: f64,
    pub theta_deg: f64,
    pub ias_kts: f64,
    pub tas_kts: f64,
    pub dt: f64,
    /// The runway's course, true.
    pub course_deg: f64,
}

/// The seconds before the wheels the decrab begins: the crab turned out at
/// the rate the law asks at most, then the yaw loop's settling (two time
/// constants of its closed attitude loop, its delay in them: 2 settles a
/// first-order loop to 86%, and every second sooner is drift the slip builds). Pure.
pub fn decrab_from_s(resp: &Response, ias_kts: f64, tas_kts: f64, tick_s: f64, crab_deg: f64) -> f64 {
    let g = design(&resp.at(ias_kts, tas_kts), tick_s);
    (crab_deg.abs() / RATE_MAX_DPS + 2.0 * attitude_tc_s(&g)).clamp(1.5, 8.0)
}

/// The sideslip the bank must hold, degrees (X-Plane's beta): none while
/// coordinating (the rudder is taking out what there is: a wing held down
/// for it turned every slip the ailerons made into more bank, the 737 at
/// EGPH 24, 1792158539), and in the decrab the one the aligned nose WILL
/// have on the same airpath (the heading less the slip: the airpath, which
/// a wagging nose does not move), so the wing goes down WITH the rudder, as
/// a pilot crosses the controls, not after the drift has begun. Pure.
pub fn slip_planned_deg(i: &In, mode: Mode) -> f64 {
    match mode {
        Mode::Coordinate => 0.0,
        Mode::Decrab => i.beta_deg - crate::angles::signed_diff_deg(i.psi_deg, i.course_deg),
    }
}

/// The rudder law: one per hand-flown segment. Pure.
#[derive(Debug, Default, Clone, Copy)]
pub struct YawHold {
    trim: f64,
    decrabbing: bool,
}

impl YawHold {
    /// The mode for this tick: the decrab once the wheels are
    /// [`decrab_from_s`] away (the flare's planned time to them, the crab
    /// the heading's from the course), and held from then on (the crab it
    /// takes out shortens its own threshold).
    pub fn mode(&mut self, to_wheels_s: Option<f64>, i: &In, resp: &Response) -> Mode {
        let crab = crate::angles::signed_diff_deg(i.course_deg, i.psi_deg);
        self.decrabbing |= to_wheels_s
            .is_some_and(|t| t <= decrab_from_s(resp, i.ias_kts, i.tas_kts, i.dt, crab));
        if self.decrabbing { Mode::Decrab } else { Mode::Coordinate }
    }

    /// One tick: the rudder (positive yaws the nose right).
    pub fn step(&mut self, i: In, mode: Mode, resp: &Response) -> f64 {
        let dt = i.dt.clamp(0.05, 1.0);
        let r = resp.at(i.ias_kts, i.tas_kts);
        let g = design(&r, dt);
        let v = (i.tas_kts * 0.514_444).max(15.0);
        let turn = (G_M_S2 / v * i.phi_deg.to_radians().sin() * i.theta_deg.to_radians().cos()).to_degrees();
        let want = match mode {
            // beta' = r - turn - path beta: r = turn + path beta - w beta
            // takes it to zero at the attitude loop's crossover.
            Mode::Coordinate => turn + (r.path - g.angle_w) * i.beta_deg,
            Mode::Decrab => g.angle_w * crate::angles::signed_diff_deg(i.course_deg, i.psi_deg),
        }
        .clamp(-RATE_MAX_DPS, RATE_MAX_DPS);
        // NDI on the measured axis: the weathercock held, the rate flown.
        let b = r.b.max(1e-6);
        let rud = self.trim + (r.damping * want + r.stiffness * i.beta_deg) / b + g.k_rate * (want - i.r_dps);
        // A slow trim on the rate error, a quarter of the crossover.
        if rud.abs() < RUD_MAX {
            self.trim = (self.trim + (want - i.r_dps) * 0.25 * g.angle_w * r.damping.max(0.2) / b * dt)
                .clamp(-TRIM_MAX, TRIM_MAX);
        }
        rud.clamp(-RUD_MAX, RUD_MAX)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coordination_asks_the_turn_and_takes_out_the_slip() {
        let r = super::super::response::prior_yaw();
        let i = In { psi_deg: 0.0, beta_deg: 0.0, r_dps: 0.0, phi_deg: 0.0, theta_deg: 0.0, ias_kts: 75.0, tas_kts: 75.0, dt: 0.2, course_deg: 0.0 };
        assert_eq!(YawHold::default().step(i, Mode::Coordinate, &r), 0.0, "level and coordinated: nothing");
        // The nose right of the airpath: left rudder, and against the
        // weathercock a decrab to the right holds right rudder.
        assert!(YawHold::default().step(In { beta_deg: 3.0, ..i }, Mode::Coordinate, &r) < 0.0);
        assert!(YawHold::default().step(In { psi_deg: -8.0, beta_deg: 0.0, ..i }, Mode::Decrab, &r) > 0.0);
        let t = decrab_from_s(&r, 75.0, 75.0, 0.2, 10.0);
        assert!((1.5..=8.0).contains(&t), "{t}");
    }
}
