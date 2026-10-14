//! The power on TOTAL ENERGY: the throttle half of TECS (Lambregts, AIAA
//! 83-2239; flown on NASA's TSRV 737; ArduPilot's AP_TECS), for an airframe
//! without an autothrottle (docs/tasks/autonomy-layers). The throttle
//! changes the aircraft's energy; the pitch only shares it between height
//! and speed. A phugoid trades one for the other at constant energy, and a
//! speed law that leads the airspeed's trend reads every swing as a coming
//! error and pumps it (the G1000 172 in ALT hold, LEMG 13, 2026-10-11: the
//! shared governor walked the lever 0.13 to 1.0 in phase with a -2,300 to
//! +750 fpm swing).
//!
//! - The energy rate is the vertical speed plus speed x acceleration / g,
//!   which a phugoid holds still.
//! - The rate wanted is the path's own descent plus the speed error closed
//!   over [`TIME_CONST_S`] (TECS_TIME_CONST). A SLOW error's request is
//!   bounded ([`SPEED_TERM_MAX_FPM`]); a FAST one's only by idle, as TECS
//!   bounds the deceleration it asks by what idle gives: a 737 at 190 kt
//!   against 145 held the bound's 0.5 kt/s with the lever at 0.12 to 0.29
//!   down an EGKK 26L final and went around at the gate 38 kt fast
//!   (sortie 1791979030-full_auto, 2026-10-14).
//! - The trim never learns further into a lever already at its stop
//!   (TECS's integrator limits): back inside the band, the power the path
//!   needs is still there.
//! - The lever is the trim (an integrator, learnt over [`BASE_TAU_S`] and
//!   stepped at once by a new path's energy: the feed-forward
//!   `STEdot_dem / K_thr2STE`) plus the rate error over `K_thr2STE`.
//!   `K_thr2STE` ([`Tecs::fpm_per_unit`]) is MEASURED by the
//!   identification's power step, else derived from the book: twice the
//!   best-rate climb (the climb at full power and the idle glide's sink are
//!   about equal on a light aircraft: the 172's 730 fpm book climb gives
//!   1,460 against its fitted 1,440).
//! - **Underspeed protection** (TECS's): below 90% of the speed wanted, or
//!   the flap's floor, the speed takes all the weight (its error closed in
//!   a third of the time constant) at the hand's quick rate, until it is
//!   back within 3%. It replaced the slow-side close and urgent rate of
//!   2026-10-12 (`SPEED_CLOSE_SLOW_S`, `URGENT_SLOW_KTS`), which did the
//!   same thing from a typed band of knots.
//! - **Bad descent** (TECS's): sinking more than [`BAD_DESCENT_FPM`] below
//!   the path's energy for a second, the same full power.
//! - **The flare**: height has priority (the elevator holds the path the
//!   flare plans) and the lever only comes back, walking to idle so the
//!   engine has spooled down ([`Tecs::spool_s`], the engine class's, or
//!   measured) as the wheels arrive: the time to them over the lever. The
//!   per-airframe retard schedules (`flare_power`) it replaced fitted the
//!   same timing by hand.

/// A speed error is closed, as energy, over this long, seconds. TECS's
/// default is 5; 20 keeps the hand slow on an approach (the phugoid above).
pub const TIME_CONST_S: f64 = 20.0;
/// The trim is learnt this slowly, seconds.
pub const BASE_TAU_S: f64 = 10.0;
/// The most the hand moves the lever, per second.
pub const SLEW_PER_S: f64 = 0.05;
/// ...and in a protection: a hand pushing the throttle up.
pub const SLEW_PROTECT_PER_S: f64 = 0.15;
/// Energy errors inside this are left alone, fpm (about 1.5 kt of speed
/// error at a 172's approach speed).
const DEADBAND_FPM: f64 = 25.0;
/// A SLOW speed error never asks the thrust for more than this, fpm. A fast
/// one is not bounded here: the lever's idle stop is its bound.
const SPEED_TERM_MAX_FPM: f64 = 500.0;
/// The energy rate is smoothed over this, seconds: both its terms together,
/// so a phugoid's still cancel.
const RATE_TAU_S: f64 = 1.0;
/// TECS's underspeed: below this share of the speed wanted.
const UNDERSPEED_FRAC: f64 = 0.9;
/// ...and out of it once back within this share.
const UNDERSPEED_CLEAR_FRAC: f64 = 0.97;
/// ...and meanwhile the speed error is closed in this share of
/// [`TIME_CONST_S`] (6 s): full flap on the glideslope took the G1000 172
/// from 55 to 45 kt on a 20 s close, into the stall (LEMG 13, 1791757554).
const UNDERSPEED_TIME_SHARE: f64 = 0.3;
/// TECS's bad descent: the energy rate this far below the one wanted, fpm,
/// for [`BAD_DESCENT_S`].
pub const BAD_DESCENT_FPM: f64 = 500.0;
const BAD_DESCENT_S: f64 = 1.0;
const G_FPS2: f64 = 32.174;
const FPS_PER_KT: f64 = 1.687_81;

/// The throttle's numbers for this airframe: what a unit of lever buys in
/// energy rate (TECS's `K_thr2STE`, fpm per unit) and how long the engine
/// takes to follow the lever, s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tecs {
    pub fpm_per_unit: f64,
    pub spool_s: f64,
}

impl Tecs {
    /// From the book, where nothing is measured: twice the best-rate climb,
    /// and the engine class's spool.
    pub fn from_book(best_climb_fpm: Option<f64>, class: EngineClass) -> Tecs {
        let climb = best_climb_fpm.filter(|c| c.is_finite() && *c > 100.0).unwrap_or(1_000.0);
        Tecs { fpm_per_unit: 2.0 * climb, spool_s: class.spool_s() }
    }
}

pub use crate::aircraft::learned::EngineClass;

#[derive(Debug, Clone, Copy)]
pub struct In {
    pub ias_kts: f64,
    /// The airspeed's trend, kt/s.
    pub accel_kts_s: f64,
    /// The TRUE vertical speed, fpm (not the VSI).
    pub vs_fpm: f64,
    pub target_kts: f64,
    /// The flap's own floor, kt: under it is an underspeed whatever the
    /// target.
    pub floor_kts: Option<f64>,
    /// The vertical speed the path asks for, fpm: 0 level, the glidepath's
    /// descent on it.
    pub path_vs_fpm: f64,
    pub thr_now: f64,
    pub dt: f64,
    /// In the flare: the seconds to the wheels it plans.
    pub flare_to_wheels_s: Option<f64>,
}

/// What the law decided this tick (the blackbox's `tecs_*`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Out {
    pub lever: f64,
    pub underspeed: bool,
    pub bad_descent: bool,
    pub flare: bool,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct EnergyThrottle {
    base: Option<f64>,
    rate_f: Option<f64>,
    last_path_vs: Option<f64>,
    underspeed: bool,
    bad_s: f64,
}

/// The energy rate, fpm: climb plus speed x acceleration / g. Pure.
pub fn energy_rate_fpm(ias_kts: f64, accel_kts_s: f64, vs_fpm: f64) -> f64 {
    vs_fpm + ias_kts * FPS_PER_KT * accel_kts_s * FPS_PER_KT / G_FPS2 * 60.0
}

/// The energy rate that closes `err_kts` of speed (positive: slow) over
/// [`TIME_CONST_S`], fpm: at most [`SPEED_TERM_MAX_FPM`] slow, unbounded
/// fast. Pure.
pub fn speed_term_fpm(ias_kts: f64, err_kts: f64) -> f64 {
    (ias_kts * FPS_PER_KT * err_kts * FPS_PER_KT / G_FPS2 / TIME_CONST_S * 60.0).min(SPEED_TERM_MAX_FPM)
}

impl EnergyThrottle {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// The lever for this tick, absolute, 0 to 1.
    pub fn step(&mut self, i: In, tecs: Tecs) -> Out {
        let dt = i.dt.clamp(0.05, 2.0);
        let k = tecs.fpm_per_unit.max(100.0);
        let now = energy_rate_fpm(i.ias_kts, i.accel_kts_s, i.vs_fpm);
        let f = self.rate_f.get_or_insert(now);
        *f += (now - *f) * (dt / RATE_TAU_S).min(1.0);
        let have = *f;
        let base = self.base.get_or_insert(i.thr_now);
        // THE FLARE: height first, the lever only back, idle with the
        // engine spooled down as the wheels arrive.
        if let Some(t_go) = i.flare_to_wheels_s.filter(|t| t.is_finite()) {
            let left = (t_go - tecs.spool_s).max(dt);
            let lever = (i.thr_now - i.thr_now * dt / left).max(0.0);
            *base = lever;
            return Out { lever, underspeed: false, bad_descent: false, flare: true };
        }
        // The protections. An UNDERSPEED (TECS's 90% of the speed wanted, or
        // the flap's floor): the speed comes first, its error closed in a
        // share of the time constant at the quick hand (TECS gives the speed
        // all the weight). A BAD DESCENT (the energy rate far below the one
        // wanted for a second): full power at the quick hand.
        let floor = i.floor_kts.filter(|f| f.is_finite()).unwrap_or(0.0);
        let under = i.ias_kts < (UNDERSPEED_FRAC * i.target_kts).max(floor);
        let clear = i.ias_kts > (UNDERSPEED_CLEAR_FRAC * i.target_kts).max(floor);
        self.underspeed = under || (self.underspeed && !clear);
        let weight = if self.underspeed { 1.0 / UNDERSPEED_TIME_SHARE } else { 1.0 };
        let want = i.path_vs_fpm
            + (speed_term_fpm(i.ias_kts, i.target_kts - i.ias_kts) * weight).min(SPEED_TERM_MAX_FPM * weight);
        let raw = want - have;
        self.bad_s = if raw > BAD_DESCENT_FPM { self.bad_s + dt } else { 0.0 };
        let bad_descent = self.bad_s >= BAD_DESCENT_S && !self.underspeed;
        if bad_descent {
            let lever = (i.thr_now + SLEW_PROTECT_PER_S * dt).min(1.0);
            return Out { lever, underspeed: false, bad_descent, flare: false };
        }
        let err = if raw.abs() < DEADBAND_FPM { 0.0 } else { raw - DEADBAND_FPM * raw.signum() };
        // The trim learns every error, however small (the deadband is the
        // hand's), and a new path moves it at once by the energy the path
        // asks: a glideslope captured from level takes the power off as it
        // starts down. It never learns further into a lever already at its
        // stop: a final flown fast at idle would otherwise unlearn the
        // power its path needs.
        let path_step = self.last_path_vs.map_or(0.0, |p| i.path_vs_fpm - p);
        self.last_path_vs = Some(i.path_vs_fpm);
        *base = (*base + path_step / k).clamp(0.0, 1.0);
        let pushed = *base + err / k;
        let at_stop = (pushed <= 0.0 && raw < 0.0) || (pushed >= 1.0 && raw > 0.0);
        if !at_stop {
            *base = (*base + raw / k * dt / BASE_TAU_S).clamp(0.0, 1.0);
        }
        let cmd = (*base + err / k).clamp(0.0, 1.0);
        let step = if self.underspeed { SLEW_PROTECT_PER_S } else { SLEW_PER_S } * dt;
        let lever = cmd.clamp(i.thr_now - step, i.thr_now + step).clamp(0.0, 1.0);
        Out { lever, underspeed: self.underspeed, bad_descent: false, flare: false }
    }
}

#[cfg(test)]
#[path = "energy_tests.rs"]
mod tests;
