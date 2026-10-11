//! The approach's power on TOTAL ENERGY, for an airframe without an
//! autothrottle (docs/tasks/autonomy-layers). The throttle changes the
//! aircraft's energy; the pitch, the autopilot's, only shares it between
//! height and speed. A phugoid trades one for the other at constant energy,
//! and a speed law that leads the airspeed's trend reads every swing as a
//! coming error and pumps it.
//!
//! The G1000 172 intercepting LEMG 13's ILS level (2026-10-11) showed it:
//! in ALT hold the shared governor (`speedhold::throttle_adjust`, a 5 s
//! projection of the trend) walked the lever 0.13 to 1.0 in fifteen seconds
//! in phase with the swing, and the autopilot's altitude hold went -2,300 to
//! +750 fpm on a 30 s period. A pilot on the autopilot sets power and
//! leaves it; this law does the same thing from the physics:
//!
//! - the energy rate is the vertical speed plus speed x acceleration / g,
//!   which a phugoid holds still;
//! - it is asked for the path's own descent plus what closes the speed error
//!   in [`SPEED_CLOSE_S`];
//! - the set power is learnt slowly ([`BASE_TAU_S`]), moved at once by the
//!   energy a new path asks, and the hand moves the lever no faster than
//!   [`SLEW_PER_S`].

/// A speed error is closed, as energy, over this long, seconds.
pub const SPEED_CLOSE_S: f64 = 20.0;
/// ...and a SLOW one over this long: slow is the dangerous side. Closing
/// 10 kt slow in 20 s asked 125 fpm of energy; full flap on the glideslope
/// took the G1000 172 from 55 to 45 kt, into the stall and a wing drop
/// (LEMG 13, 1791757554).
pub const SPEED_CLOSE_SLOW_S: f64 = 6.0;
/// Slow by more than this, kt, the slow side's closing applies.
const SLOW_KTS: f64 = 3.0;
/// Slow by more than this, kt, the hand moves at [`SLEW_SLOW_PER_S`].
const URGENT_SLOW_KTS: f64 = 5.0;
/// The hand's rate when the speed is decaying away, per second.
pub const SLEW_SLOW_PER_S: f64 = 0.15;
/// The set power is learnt this slowly, seconds.
pub const BASE_TAU_S: f64 = 10.0;
/// Energy rate a whole throttle buys, fpm: idle to full is about 1,400 fpm
/// on a 172 at approach speed and more on everything faster; taken high so
/// the lever moves gently everywhere and the learnt base does the rest.
pub const FPM_PER_THROTTLE: f64 = 2_000.0;
/// The most the hand moves the lever, per second.
pub const SLEW_PER_S: f64 = 0.05;
/// Energy errors inside this are left alone, fpm (about 1.5 kt of speed
/// error at a 172's approach speed).
const DEADBAND_FPM: f64 = 25.0;
/// The speed term never asks more than this, fpm.
const SPEED_TERM_MAX_FPM: f64 = 500.0;
/// The energy rate is smoothed over this, seconds: both its terms together,
/// so a phugoid's still cancel.
const RATE_TAU_S: f64 = 1.0;
const G_FPS2: f64 = 32.174;
const FPS_PER_KT: f64 = 1.687_81;

#[derive(Debug, Clone, Copy)]
pub struct In {
    pub ias_kts: f64,
    /// The airspeed's trend, kt/s.
    pub accel_kts_s: f64,
    /// The TRUE vertical speed, fpm (not the VSI).
    pub vs_fpm: f64,
    pub target_kts: f64,
    /// The vertical speed the path asks for, fpm: 0 level, the glidepath's
    /// descent on it.
    pub path_vs_fpm: f64,
    pub thr_now: f64,
    pub dt: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct EnergyThrottle {
    base: Option<f64>,
    rate_f: Option<f64>,
    last_path_vs: Option<f64>,
}

/// The energy rate, fpm: climb plus speed x acceleration / g. Pure.
pub fn energy_rate_fpm(ias_kts: f64, accel_kts_s: f64, vs_fpm: f64) -> f64 {
    vs_fpm + ias_kts * FPS_PER_KT * accel_kts_s * FPS_PER_KT / G_FPS2 * 60.0
}

/// The energy rate that closes `err_kts` of speed (positive: slow) in
/// [`SPEED_CLOSE_S`], or [`SPEED_CLOSE_SLOW_S`] when slow, fpm. Pure.
pub fn speed_term_fpm(ias_kts: f64, err_kts: f64) -> f64 {
    let (close_s, max) = if err_kts > SLOW_KTS {
        (SPEED_CLOSE_SLOW_S, 2.0 * SPEED_TERM_MAX_FPM)
    } else {
        (SPEED_CLOSE_S, SPEED_TERM_MAX_FPM)
    };
    (ias_kts * FPS_PER_KT * err_kts * FPS_PER_KT / G_FPS2 / close_s * 60.0).clamp(-max, max)
}

impl EnergyThrottle {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// The lever for this tick, absolute, 0 to 1.
    pub fn step(&mut self, i: In) -> f64 {
        let dt = i.dt.clamp(0.05, 2.0);
        let now = energy_rate_fpm(i.ias_kts, i.accel_kts_s, i.vs_fpm);
        let f = self.rate_f.get_or_insert(now);
        *f += (now - *f) * (dt / RATE_TAU_S).min(1.0);
        let have = *f;
        let want = i.path_vs_fpm + speed_term_fpm(i.ias_kts, i.target_kts - i.ias_kts);
        let raw = want - have;
        let err = if raw.abs() < DEADBAND_FPM { 0.0 } else { raw - DEADBAND_FPM * raw.signum() };
        // The set power learns every error, however small (the deadband is
        // the hand's, not the trim's): a steady 2 kt fast is re-trimmed.
        // ...and a new path moves it at once by the energy the path asks: a
        // glideslope captured from level takes the power off as it starts down.
        let base = self.base.get_or_insert(i.thr_now);
        let path_step = self.last_path_vs.map_or(0.0, |p| i.path_vs_fpm - p);
        self.last_path_vs = Some(i.path_vs_fpm);
        *base = (*base + path_step / FPM_PER_THROTTLE + raw / FPM_PER_THROTTLE * dt / BASE_TAU_S)
            .clamp(0.0, 1.0);
        let cmd = (*base + err / FPM_PER_THROTTLE).clamp(0.0, 1.0);
        let slew = if i.target_kts - i.ias_kts > URGENT_SLOW_KTS { SLEW_SLOW_PER_S } else { SLEW_PER_S };
        let step = slew * dt;
        cmd.clamp(i.thr_now - step, i.thr_now + step).clamp(0.0, 1.0)
    }
}

#[cfg(test)]
#[path = "energy_throttle_tests.rs"]
mod tests;
