//! The pilot: one controller that flies a fixed-wing aircraft by hand the
//! way a pilot does, for every airframe (docs/tasks/autonomy-layers,
//! 2026-10-12). Its first user is the final from minimums
//! (`autoland`: the autopilot disconnected, hand flown to the wheels); the
//! loops are written for any phase, so a recovery or a hand-flown circuit
//! can take them as they are.
//!
//! - inner loops: the elevator holds a pitch attitude ([`attitude`]), the
//!   ailerons hold a bank ([`bank`]), the throttle holds the energy rate
//!   ([`energy`]);
//! - outer loops: the path and the flare as a path target ([`attitude`]),
//!   the centreline as a bank ([`line`]), the speed as energy ([`energy`]).
//!
//! Nothing in it is typed into a profile. What differs between airframes is
//! MEASURED in flight: the roll response the ailerons get
//! ([`bank::RollResponse`], from the first seconds of hand flying), the
//! elevator the nose needs (the attitude law's disturbance observer). One
//! shape fits every airframe flown on 2026-10-12 (`plants_tests`).
//!
//! Not yet: the rudder. Every rudder write on the tapes is on the wheels
//! (the rollout's steer), so there is no airborne rudder response to design
//! a coordination or decrab law on; the track law holds the crab to the
//! wheels and the rollout straightens it.

pub mod attitude;
pub mod bank;
pub mod energy;
pub mod line;

#[cfg(test)]
mod plants_tests;
