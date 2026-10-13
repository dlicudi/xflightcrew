//! The pilot: one controller that flies a fixed-wing aircraft by hand the
//! way a pilot does, for every airframe (docs/tasks/autonomy-layers,
//! 2026-10-12). Its first user is the final from minimums
//! (`autoland`: the autopilot disconnected, hand flown to the wheels); the
//! loops are written for any phase, so a recovery or a hand-flown circuit
//! can take them as they are.
//!
//! - inner loops: the elevator holds a pitch attitude ([`attitude`]), the
//!   ailerons hold a bank ([`bank`]), the rudder holds the sideslip or the
//!   runway's heading ([`yaw`]), the throttle holds the energy rate
//!   ([`energy`], TECS's throttle half);
//! - outer loops: the path and the flare as a path target ([`attitude`]),
//!   the centreline as a bank ([`line`], NPFG), the speed as energy
//!   ([`energy`]).
//!
//! Nothing in it is typed into a profile. What differs between airframes is
//! MEASURED: each axis' response by an identification flight
//! ([`ident`], `xvl identify`, stored per `.acf` in `aircraft::learned`)
//! and refined as the segment flies (the roll's [`bank::RollResponse`]),
//! the gains derived from it by one rule ([`response::design`]: McRuer's
//! crossover, scaled by the air), the nose limited by the airframe's
//! geometry read from its `.acf` ([`attitude::PitchLimits`]).

pub mod attitude;
pub mod bank;
pub mod energy;
pub mod ident;
pub mod line;
pub mod response;
pub mod yaw;

#[cfg(test)]
mod plants_tests;

use crate::aircraft::learned::{AcfModel, MAINS_FIRST_MARGIN_DEG, TAIL_MARGIN_DEG};
use response::Response;

/// What the pilot flies one airframe on: each axis' response (measured, or
/// the fleet's prior), the throttle's energy and the nose's limits.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Airframe {
    pub pitch: Response,
    pub roll: Response,
    pub yaw: Response,
    pub tecs: energy::Tecs,
    pub limits: attitude::PitchLimits,
    /// Which axes were measured on this `.acf` (pitch, roll, yaw, energy).
    pub measured: [bool; 4],
}

impl Airframe {
    /// From the `.acf`'s model, where read, and the profile's book figures:
    /// the best-rate climb (TECS's lever, until measured) and the tail
    /// strike (where the `.acf`'s fuselage did not give one). Pure.
    pub fn from(model: Option<&AcfModel>, best_climb_fpm: Option<f64>, book_tail_strike_deg: Option<f64>) -> Airframe {
        let learned = model.and_then(|m| m.learned.as_ref().ok());
        let pick = |f: Option<crate::aircraft::learned::Fit>, prior: Response| {
            f.map(|f| f.response).filter(Response::sane).map_or((prior, false), |r| (r, true))
        };
        let (pitch, p) = pick(learned.and_then(|l| l.pitch), response::prior_pitch());
        let (roll, r) = pick(learned.and_then(|l| l.roll), response::prior_roll());
        let (yaw, y) = pick(learned.and_then(|l| l.yaw), response::prior_yaw());
        let class = model.and_then(|m| m.engine).unwrap_or(crate::aircraft::learned::EngineClass::Piston);
        let book = energy::Tecs::from_book(best_climb_fpm, class);
        let measured_energy = learned.and_then(|l| l.energy);
        let tecs = measured_energy.map_or(book, |e| energy::Tecs { fpm_per_unit: e.fpm_per_unit, spool_s: e.spool_s });
        let geometry = model.map(|m| m.geometry);
        let tail = geometry.and_then(|g| g.tail_strike_deg).or(book_tail_strike_deg.filter(|t| t.is_finite()));
        let limits = attitude::PitchLimits {
            max_deg: tail.map_or(attitude::PitchLimits::default().max_deg, |t| t - TAIL_MARGIN_DEG),
            touchdown_min_deg: match geometry {
                Some(g) => g.level_deg.map(|l| l + MAINS_FIRST_MARGIN_DEG),
                None => attitude::PitchLimits::default().touchdown_min_deg,
            },
        };
        Airframe { pitch, roll, yaw, tecs, limits, measured: [p, r, y, measured_energy.is_some()] }
    }
}

/// The live airframe's.
pub fn airframe() -> Airframe {
    let p = crate::aircraft::live_profile();
    let best_climb = p.as_ref().and_then(|p| p.reference.as_ref().and_then(|r| r.best_climb_fpm));
    let tail = p.as_ref().and_then(|p| p.tail_strike_deg);
    Airframe::from(crate::aircraft::learned::acf_model().as_ref(), best_climb, tail)
}
