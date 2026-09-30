//! ONE ground-directional controller for a FREE castoring nose (the SR22,
//! the Lancair Evolution, the RV-10: `aircraft::nose_free`, no steering of
//! its own), used by the taxi's corners and backtracks, the line-up's
//! aligning pivot, the takeoff roll's start and the rollout. Jets and every
//! nosewheel that steers keep their own laws, untouched.
//!
//! Why one law: every flight's failing corner had grown its own mechanism
//! (the swing catch, the turn builder, the led rate ceiling, the wrong-way
//! pivot brake, the aligning PD and its trim, the rollout's scheduled split
//! and trim), each fitted to the corner that failed, and the flights still
//! failed by about a metre (iMac 2026-09-30: the SR22's LEMG taxi 3.6 m in
//! 1790764128, the Evolution's LILB backtrack 4.0 m and rollout 8.3 m in
//! 1790762771). This is the controller designed from the identified plant.
//!
//! # The plant (identified 2026-09-30 from every free-nose tick on file)
//!
//! Output error over 2.4 s windows of every drive tick of 2026-09-29/30
//! (the SR22: 20 drives, 25,403 ticks; the Evolution: 16 drives, 5,373
//! ticks; the yaw rate recovered from the logged EMA) and 2.3 s windows of
//! the rollouts' own heading (the SR22's 21, the Evolution's 2), jointly;
//! checked against the line-up alignments' own `R` reads:
//!
//! ```text
//!   rdot   = Kb(v) u + Kr v^2 d - D(v) r - Kv(v) (psi - track) + w   (deg/s^2)
//!   trackdot = (side_ms2 / v) (psi - track)
//!   Kb(v) = brake_dps2 * v / (v + brake_from_ms) * max(1 + brake_per_ms * v, 0.2)
//!   D(v)  = damp + damp_per_ms * v
//!   Kv(v) = vane_dps2 * (v / 30)^2
//! ```
//!
//! `u` the wheel difference (right minus left), `d` the rudder (the steer
//! channel), `v` groundspeed in m/s, `psi - track` the slip the tail
//! weathervanes on and the track follows the nose on (the rollouts' `trk`
//! against `psi`: 3.3 per second at 11 m/s, 1.6 at 20, 0.7 at 32 on the
//! SR22, so a side force of about 30 m/s^2 per radian), `w` a standing yaw
//! (the propeller's pull, about -0.8
//! deg/s^2 on the SR22 and -1.2 on the Evolution at taxi power; learnt in
//! flight, never declared). Per speed bin the SR22's brake is 112 to 118
//! deg/s^2 per unit from 0.35 to 4.5 m/s (half of it at 0.1 m/s) and its
//! damping 0.21 at 0.5 m/s rising to 0.64 at 6, so a swing at a walk lasts
//! seconds: that is "the swing outlasts the brake"; the Evolution's brake
//! falls from 88 at 0.5 m/s to 36 at 6. The rudder at taxi power is a few
//! deg/s^2 per unit and at idle on a rollout next to nothing. The castoring
//! nose adds no pole of its own the ticks can see (the brake lag fits at its
//! floor, 0.05 s). The drive's input-to-output delay is its tick and under
//! 0.1 s more (every fit was best at 0 to 0.1 s: the "0.45 s transport
//! delay" of the older fits was the drive's own yaw EMA, its 0.4 s brake
//! smoothing and its slews); the rollout's loop writes about 0.4 s after its
//! read (the fits' best, 1.19 degrees rms over 2.3 s against 1.72 at none).
//! A braked wheel drags the aircraft at 6.0 m/s^2 per unit (SR22), 4.0
//! (Evolution).
//!
//! At a rollout's speed the weathervane is a stiff spring (4.5 rad/s at 30
//! m/s on the SR22, damped 0.54, above what a 0.46 s loop can see), and the
//! yaw rate a steady wheel difference holds is the wheels' less what the
//! spring takes back while the track comes round: `Kb kt / (D kt + Kv)` per
//! unit, `kt = side_ms2 / v` (8.8 deg/s at 30 m/s on the SR22 where the
//! wheels alone would give 47). The law's own model is first order, fitted to
//! the plant with its spring where the rate loop closes
//! ([`GroundPlant::model`]: the rate it settles at, `(D kt + Kv) / (D + kt)`,
//! and the gain that makes the model's answer at the loop's crossover,
//! `1 / rate_tc_s`, the plant's); at taxi speed it is the wheels' own. The
//! low frequencies it leaves to the observer. Fed the slip at the tick
//! instead the law rang (the spring moves within the tick); modelled on the
//! wheels alone it asked a fifth of the brake a held turn needs (a rollout
//! replayed on #166's wheel plant ran 16.7 m off); modelled at the steady
//! gain, the Evolution (whose brake is nearly nothing against its tail at 40
//! m/s) went unstable. The observer, reading a slip-driven yaw as a standing
//! one, learns at half its rate once the spring's slow mode passes the
//! wheels' damping ([`GroundPlant::observer_share`]). The margins below are
//! on the plant WITH the spring (states yaw rate, slip, heading).
//!
//! # The controller
//!
//! A cascade, every gain from the plant and the tick:
//! - path to heading ([`path_demand`]): the reference heading is the
//!   route's chord across the aim span `L = 2 v / k` ([`aim_m`], never
//!   inside [`MIN_CLOSE_M`]) centred [`lead_s`] of travel ahead (the inner
//!   loop's lag), its rate along the route fed forward, and the lateral
//!   closed at `atan(y / L)` (the pursuit's gain: its heading gain is the
//!   heading loop's own, the lateral mode damped 0.7). A corner is the arc
//!   the chord sweeps; a kink or a scatter shorter than the aim is averaged
//!   out, never steered (fillets fitted to every vertex steered the paint's
//!   own scatter: the replayed LEMG taxis ran 10 to 40 m off on them);
//! - heading to yaw rate: `r_ref = ff + k e` with `k` = [`heading_gain`],
//!   bounded by the yaw limit (on the taxi the declared corner rate the
//!   route is sized on, or half the rate the wheels hold, to
//!   [`YAW_LIMIT_MAX_DPS`]: [`GroundSteer::taxi_rate_limit`]) and by a
//!   braking curve `sqrt(2 a e)` so no rate is asked that cannot be stopped
//!   in the error left, and slewed at half the brake's authority;
//! - yaw rate to brake: the plant discretised over the tick (zero-order
//!   hold, the tick's own length), inverted for a first-order closure in
//!   [`rate_tc_s`]; the yaw read the loop's own delay late is carried
//!   forward on the model with the command still in force (the Smith step);
//!   a disturbance observer learns `w` from the model's one-tick miss over
//!   [`DOB_S`] (the trims it replaces).
//!
//! Margins, loop broken at the wheel difference, linearised on the plant
//! with its spring (scratch analysis 2026-09-30, both airframes, the plant's
//! brake at 0.7 to 1.3 of the fit, at 0.3, 0.45 and 0.6 s ticks): the taxi
//! (0.5 to 8 m/s, 0 to 0.2 s of real delay against 0.05 modelled) gain
//! margin 2.39 and more, phase margin 39 degrees and more; the roll's start
//! (5 to 20 m/s) gain margin 2.47, phase 48; the rollout (5 to 40 m/s, 0.3
//! to 0.4 s real against 0.4) the SR22 gain margin 2.84, phase 50, the
//! Evolution 1.78 and 40 (its brake is nearly nothing against its tail at
//! 40 m/s). Pure.

use serde_json::json;

/// The yaw plant of a free castoring nose (see the module doc).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GroundPlant {
    /// Yaw acceleration per unit of wheel difference, deg/s^2 (the scale).
    pub brake_dps2: f64,
    /// The brake's authority is half at this groundspeed, m/s (a braked
    /// wheel turns the aircraft only while it rolls).
    pub brake_from_ms: f64,
    /// ...and changes by this share per m/s above it (never under a fifth).
    pub brake_per_ms: f64,
    /// Yaw damping at a standstill, 1/s...
    pub damp: f64,
    /// ...plus this per m/s of groundspeed.
    pub damp_per_ms: f64,
    /// The rudder: deg/s^2 per unit of steer per (m/s)^2.
    pub rudder: f64,
    /// The weathervane: deg/s^2 per degree of slip at 30 m/s (as v^2).
    pub vane_dps2: f64,
    /// The side force the track follows the nose on, m/s^2 per radian of
    /// slip (the track turns at this over v per second of slip).
    pub side_ms2: f64,
}

impl GroundPlant {
    /// The SR22 (LEMG, the taxis of 1790690995 to 1790764128 and the
    /// rollouts of 1790677606 to 1790765911, jointly): 0.72 deg/s rms on the
    /// taxi windows, 1.28 degrees on the rollouts'.
    pub const SR22: GroundPlant = GroundPlant {
        brake_dps2: 122.0,
        brake_from_ms: 0.1,
        brake_per_ms: 0.015,
        damp: 0.11,
        damp_per_ms: 0.127,
        rudder: 0.03,
        vane_dps2: 16.8,
        side_ms2: 30.0,
    };
    /// The Lancair Evolution (LILB, the taxis of 1790712974 to 1790762771
    /// and its two rollouts): brake 88 at 0.5 m/s falling to 36 at 6, the
    /// standstill fade fitted on the alignments' `R` (0.19 m/s). Declared in
    /// its profile (`ground_plant:`); here for the tests.
    #[cfg_attr(not(test), allow(dead_code))]
    pub const EVOLUTION: GroundPlant = GroundPlant {
        brake_dps2: 109.0,
        brake_from_ms: 0.2,
        brake_per_ms: -0.154,
        damp: 0.218,
        damp_per_ms: 0.043,
        rudder: 0.26,
        vane_dps2: 6.8,
        side_ms2: 15.0,
    };

    /// The live airframe's plant: its profile's `ground_plant:`, else the
    /// SR22's (the RV-10 flies it until a taxi of its own is on file).
    pub fn for_airframe() -> GroundPlant {
        crate::aircraft::ground_plant().map_or(Self::SR22, |p| GroundPlant {
            brake_dps2: p.brake_dps2,
            brake_from_ms: p.brake_from_ms,
            brake_per_ms: p.brake_per_ms,
            damp: p.damp,
            damp_per_ms: p.damp_per_ms,
            rudder: p.rudder_per_ms2,
            vane_dps2: p.vane_dps2,
            side_ms2: p.side_ms2,
        })
    }

    /// Yaw acceleration per unit of wheel difference at `v` m/s.
    pub fn kb(&self, v: f64) -> f64 {
        let v = v.max(0.0);
        self.brake_dps2 * v / (v + self.brake_from_ms) * (1.0 + self.brake_per_ms * v).max(0.2)
    }

    /// Yaw damping at `v` m/s, 1/s (the wheels').
    pub fn d(&self, v: f64) -> f64 {
        (self.damp + self.damp_per_ms * v.max(0.0)).max(0.02)
    }

    /// The track's rate per degree of slip at `v` m/s, 1/s.
    pub fn kt(&self, v: f64) -> f64 {
        self.side_ms2 / v.max(1.0)
    }

    /// The law's own first-order model at `v` m/s for a tick of `tick_s`:
    /// (yaw acceleration per unit of wheel difference, the rate it settles
    /// at). The settling rate is the plant's with its weathervane, and the
    /// gain makes the model's answer at the rate loop's crossover
    /// (`1 / rate_tc_s`) the plant's: `|P(jw)| |jw + settle|` with
    /// `P(s) = Kb (s + kt) / (s^2 + (D + kt) s + D kt + Kv)` (see the module
    /// doc). At taxi speed the wheels' own.
    pub fn model(&self, v: f64, tick_s: f64) -> (f64, f64) {
        let (kb, d, kv, kt) = (self.kb(v), self.d(v), self.kv(v), self.kt(v));
        let settle = (d * kt + kv) / (d + kt);
        let w = 1.0 / rate_tc_s(tick_s);
        // |P(jw)|: |kt + jw| over |(D kt + Kv - w^2) + j (D + kt) w|.
        let num = (kt * kt + w * w).sqrt();
        let den = ((d * kt + kv - w * w).powi(2) + ((d + kt) * w).powi(2)).sqrt();
        let gain = kb * num / den * (w * w + settle * settle).sqrt();
        (gain, settle)
    }

    /// The share of its rate the disturbance observer learns at at `v` m/s:
    /// whole while the weathervane's slow mode (`Kv v / side_ms2`) is under
    /// half the wheels' damping, [`OBSERVER_SHARE_MIN`] once it is their equal
    /// (see the module doc).
    pub fn observer_share(&self, v: f64) -> f64 {
        let v = v.max(0.0);
        let ratio = self.kv(v) * v / self.side_ms2.max(1.0) / self.d(v);
        (1.0 - (ratio - 0.5) / 0.5).clamp(OBSERVER_SHARE_MIN, 1.0)
    }

    /// Yaw acceleration per unit of rudder at `v` m/s.
    pub fn kr(&self, v: f64) -> f64 {
        self.rudder * v * v
    }

    /// The weathervane's yaw acceleration per degree of slip at `v` m/s.
    pub fn kv(&self, v: f64) -> f64 {
        self.vane_dps2 * (v / 30.0).powi(2)
    }

    /// The yaw acceleration a reference may ask at `v` m/s with `u_max` of
    /// wheel difference: [`ACCEL_SHARE`] of what the brake can give (the
    /// rest is the feedback's), never under [`ACCEL_MIN_DPS2`].
    pub fn ref_accel(&self, v: f64, u_max: f64) -> f64 {
        (ACCEL_SHARE * self.kb(v) * u_max).max(ACCEL_MIN_DPS2)
    }

    /// The yaw rate the taxi's feedback may ask at `v` m/s with `u_max` of
    /// wheel difference, deg/s: [`YAW_LIMIT_SHARE`] of the rate the wheels
    /// hold against their own damping (`Kb u / D`), never over
    /// [`YAW_LIMIT_MAX_DPS`]. The route's reference is sized on the
    /// airframe's declared corner rate ([`chord_m`]); this is the headroom
    /// over it that brings back a corner entered fast.
    pub fn yaw_limit_dps(&self, v: f64, u_max: f64) -> f64 {
        (YAW_LIMIT_SHARE * self.kb(v) * u_max / self.d(v).max(1e-3)).min(YAW_LIMIT_MAX_DPS)
    }


    /// The same plant with its brake and rudder scaled, for the neighbours
    /// a law must survive.
    #[cfg(test)]
    pub fn scaled(self, brake: f64, damp: f64) -> GroundPlant {
        GroundPlant {
            brake_dps2: self.brake_dps2 * brake,
            damp: self.damp * damp,
            damp_per_ms: self.damp_per_ms * damp,
            rudder: self.rudder * brake,
            ..self
        }
    }
}

/// One stretch of the plant with its input held: the yaw rate at its end
/// and the heading it turned, from rate `r` under input acceleration `a`
/// (deg/s^2) for `t` s with damping `d`. Pure.
fn advance(d: f64, r: f64, a: f64, t: f64) -> (f64, f64) {
    let inf = a / d;
    let e = (-d * t).exp();
    (inf + (r - inf) * e, inf * t + (r - inf) * (1.0 - e) / d)
}

/// The drive's yaw read's age past its tick, s: the fits' best 0 to 0.1.
pub const IO_DELAY_S: f64 = 0.05;
/// The rollout's loop writes this long after its read, s (the fits' best).
pub const ROLLOUT_IO_DELAY_S: f64 = 0.4;
/// The disturbance observer's time constant, s.
pub const DOB_S: f64 = 3.0;
/// The standing yaw it may learn, deg/s^2.
pub const W_MAX: f64 = 10.0;
/// The least share of its rate the observer learns at (a rollout's speed,
/// the weathervane's slow mode past the wheels' damping).
pub const OBSERVER_SHARE_MIN: f64 = 0.5;
/// The rate loop closes in this many ticks, never faster than
/// [`RATE_MIN_S`].
pub const RATE_TICKS: f64 = 1.5;
pub const RATE_MIN_S: f64 = 0.6;
/// The heading loop is this many times slower than the rate loop and a
/// half tick together.
pub const HEADING_SEPARATION: f64 = 2.0;
/// The lateral loop's closure time over the heading loop's (damping 0.7).
pub const LATERAL_OVER_HEADING: f64 = 2.0;
/// A lateral is never closed inside this, m (at a walk the time law would
/// ask a right angle for a metre).
pub const MIN_CLOSE_M: f64 = 8.0;
/// The taxi's feedback may ask this share of the yaw rate the wheels can
/// hold...
pub const YAW_LIMIT_SHARE: f64 = 0.5;
/// ...never more than this, deg/s: the most any recorded free-nose drive
/// turned (37.7 deg/s at 2 to 4 kt, the 2026-09-29/30 drives on file).
pub const YAW_LIMIT_MAX_DPS: f64 = 38.0;
/// The rate reference slews at this share of the brake's authority...
pub const ACCEL_SHARE: f64 = 0.5;
/// ...never under this, deg/s^2.
pub const ACCEL_MIN_DPS2: f64 = 5.0;
/// Below this groundspeed a braked wheel turns nothing: no brake-steer.
pub const STILL_MS: f64 = 0.05;
/// The observer learns only above this groundspeed (static friction).
pub const DOB_MIN_MS: f64 = 0.3;
/// An owned rudder's full deflection answers at least this, deg/s^2: at a
/// walk the rudder follows the demand proportionally, not flat out.
pub const RUDDER_FULL_MIN_DPS2: f64 = 20.0;
/// The yaw limit when the profile declares none, deg/s.
pub const DEFAULT_MAX_YAW_DPS: f64 = 20.0;

/// The rate loop's time constant for a tick of `tick_s`.
pub fn rate_tc_s(tick_s: f64) -> f64 {
    (RATE_TICKS * tick_s).max(RATE_MIN_S)
}

/// The heading loop's gain, 1/s.
pub fn heading_gain(tick_s: f64) -> f64 {
    1.0 / (HEADING_SEPARATION * (rate_tc_s(tick_s) + tick_s / 2.0))
}

/// The lateral closure time, s.
pub fn lateral_s(tick_s: f64) -> f64 {
    LATERAL_OVER_HEADING / heading_gain(tick_s)
}

/// How far ahead the path's curvature is fed forward, s: the rate loop's
/// lag, half a tick and the read's age.
pub fn lead_s(tick_s: f64) -> f64 {
    rate_tc_s(tick_s) + tick_s / 2.0 + IO_DELAY_S
}

/// How the yaw rate was read.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Yaw {
    /// The sim's `R` at the tick, deg/s.
    Now(f64),
    /// The mean over the tick just ended (a heading differenced), deg/s.
    Mean(f64),
}

/// Who owns the rudder (the steer channel).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Rudder {
    /// This law: allocated first (it costs no speed), slewed.
    Own { slew_per_s: f64 },
    /// Another law's (the takeoff and rollout rudder): echoed, its yaw left
    /// to the observer (the rudder was identified at taxi power only).
    Given(f64),
}

/// One tick's demand.
#[derive(Debug, Clone, Copy)]
pub struct Demand {
    /// Seconds since the last tick.
    pub dt: f64,
    /// Groundspeed, m/s.
    pub v_ms: f64,
    pub yaw: Yaw,
    /// The heading wanted less the heading, degrees (+ the target is right).
    pub err_deg: f64,
    /// The yaw rate the path itself asks, deg/s (+ right).
    pub ff_dps: f64,
    pub rudder: Rudder,
    /// The most wheel difference allowed.
    pub u_max: f64,
    /// The most yaw rate asked, deg/s.
    pub r_max_dps: f64,
    /// How long after its read the loop's command reaches the aircraft, s
    /// ([`IO_DELAY_S`] for the drive, [`ROLLOUT_IO_DELAY_S`] the rollout).
    pub io_delay_s: f64,
}

/// What the law wants this tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Steer {
    /// Wheel difference, right minus left.
    pub diff: f64,
    /// The rudder (the owned one, or the given one echoed).
    pub rudder: f64,
    /// The yaw rate asked, deg/s.
    pub r_ref: f64,
    /// The yaw rate now as the law reads it, deg/s.
    pub r_now: f64,
    /// The standing yaw learnt, deg/s^2.
    pub w_hat: f64,
}

impl Steer {
    /// The (left, right) brakes: one wheel, never both.
    pub fn wheels(&self) -> (f64, f64) {
        wheels(self.diff)
    }

    /// The tick's fields for the blackbox.
    pub fn json(&self) -> serde_json::Value {
        json!({"r_ref": self.r_ref, "r_now": self.r_now, "w_hat": self.w_hat, "diff": self.diff})
    }
}

/// The (left, right) pair of a wheel difference: one wheel only.
pub fn wheels(diff: f64) -> (f64, f64) {
    if diff >= 0.0 {
        (0.0, diff)
    } else {
        (-diff, 0.0)
    }
}

/// A stop's brake `sym` with a wheel difference `diff` kept: (the
/// symmetric channel, left, right). The plugin gives each wheel the harder
/// of the symmetric channel and its own, so the outside wheel's brake is the
/// channel and the inside one's its own; an inside wheel at full brake takes
/// the rest of the difference off the outside (the Evolution's 8.3 m rollout
/// asked 0.14 at 0.75 of stop and got it only by luck of the cap).
pub fn stop_wheels(sym: f64, diff: f64) -> (f64, f64, f64) {
    let sym = sym.clamp(0.0, 1.0);
    let m = diff.abs().min(1.0);
    if m < 1e-9 {
        return (sym, 0.0, 0.0);
    }
    let inside = (sym + m / 2.0).min(1.0);
    let outside = (inside - m).max(0.0);
    if diff > 0.0 {
        (outside, 0.0, inside)
    } else {
        (outside, inside, 0.0)
    }
}

#[derive(Debug, Clone, Copy)]
struct Last {
    r: f64,
    /// The command written this tick and the one before it (in force for
    /// the loop's own delay after the read).
    u: f64,
    u_before: f64,
    rudder: f64,
    rudder_before: f64,
    v: f64,
    io: f64,
}

/// The controller's state (see the module doc).
#[derive(Debug, Clone)]
pub struct GroundSteer {
    plant: GroundPlant,
    w_hat: f64,
    last: Option<Last>,
    tick_s: Option<f64>,
    /// The yaw rate asked last tick, deg/s (the reference slews from it).
    r_ref: Option<f64>,
}

impl GroundSteer {
    /// The taxi's yaw limit at `v` m/s: the airframe's declared corner rate,
    /// or the plant's headroom over it ([`GroundPlant::yaw_limit_dps`]).
    pub fn taxi_rate_limit(&self, v: f64, u_max: f64, declared_dps: f64) -> f64 {
        declared_dps.max(self.plant.yaw_limit_dps(v, u_max))
    }

    pub fn new(plant: GroundPlant) -> Self {
        GroundSteer {
            plant,
            w_hat: 0.0,
            last: None,
            tick_s: None,
            r_ref: None,
        }
    }

    /// For the live airframe.
    pub fn for_airframe() -> Self {
        Self::new(GroundPlant::for_airframe())
    }

    /// The tick it expects next, s (0.3 before the first).
    pub fn tick_s(&self) -> f64 {
        self.tick_s.unwrap_or(0.3)
    }

    /// The standing yaw learnt, deg/s^2.
    pub fn w_hat(&self) -> f64 {
        self.w_hat
    }

    /// Forget the last tick (a stop, a bounce, a pause): the observer keeps
    /// what it learnt.
    pub fn release(&mut self) {
        self.last = None;
        self.r_ref = None;
    }

    /// One tick.
    pub fn step(&mut self, d: Demand) -> Steer {
        let p = self.plant;
        let dt = if d.dt.is_finite() { d.dt.clamp(0.05, 1.5) } else { 0.3 };
        // The next tick is expected as long as the last ones; a first call
        // with no interval of its own (a law just started) expects 0.3 s.
        let tick = match self.tick_s {
            Some(t) if self.last.is_some() => 0.5 * t + 0.5 * dt,
            Some(t) => t,
            None if d.dt >= 0.1 => dt,
            None => 0.3,
        };
        self.tick_s = Some(tick);
        let io = if d.io_delay_s.is_finite() { d.io_delay_s.clamp(0.0, 1.0) } else { IO_DELAY_S };
        let v = if d.v_ms.is_finite() { d.v_ms.max(0.0) } else { 0.0 };
        // The rudder's yaw is modelled only where it was identified: at taxi
        // power and taxi speed, owned. Another law's rudder (the roll's, the
        // rollout's at idle: under 2 deg/s per unit on the SR22's rollouts,
        // #166) is left to the observer.
        let own = matches!(d.rudder, Rudder::Own { .. });
        let (kb, dmp) = p.model(v, tick);
        let kr = if own { p.kr(v) } else { 0.0 };
        let (u_prev, rud_prev) = self.last.map_or((0.0, 0.0), |l| (l.u, l.rudder));
        // The inputs over the tick just flown, as the plant felt them: the
        // command before last for the loop's delay, then the last.
        let flown = |l: &Last, w: f64| {
            let (kbl, krl) = (p.model(l.v, tick).0, if own { p.kr(l.v) } else { 0.0 });
            let t1 = l.io.min(dt);
            let a1 = kbl * l.u_before + krl * l.rudder_before + w;
            let a2 = kbl * l.u + krl * l.rudder + w;
            (t1, a1, dt - t1, a2)
        };
        // The yaw rate now: a mean over the tick is carried to its end on the
        // model with the inputs that held over it (linear in the rate the
        // tick began with, so solved for it).
        let r_now = match (d.yaw, self.last) {
            (Yaw::Now(r), _) => r,
            (Yaw::Mean(m), Some(l)) => {
                let (t1, a1, t2, a2) = flown(&l, self.w_hat);
                let run = |r0: f64| {
                    let (r1, h1) = advance(dmp, r0, a1, t1);
                    let (r2, h2) = advance(dmp, r1, a2, t2);
                    (r2, (h1 + h2) / dt)
                };
                let ((e0, m0), (e1, m1)) = (run(0.0), run(1.0));
                let r0 = if (m1 - m0).abs() > 1e-9 { (m - m0) / (m1 - m0) } else { m };
                e0 + (e1 - e0) * r0
            }
            (Yaw::Mean(m), None) => m,
        };
        let r_now = if r_now.is_finite() { r_now } else { 0.0 };
        // The observer: the model's miss over the tick just flown is the
        // standing yaw it does not know.
        if let Some(l) = self.last {
            if v >= DOB_MIN_MS && l.v >= DOB_MIN_MS {
                let dl = p.model(l.v, tick).1;
                let (t1, a1, t2, a2) = flown(&l, self.w_hat);
                let (r1, _) = advance(dl, l.r, a1, t1);
                let (r_hat, _) = advance(dl, r1, a2, t2);
                // The rate a unit of standing yaw adds over the tick.
                let c = (1.0 - (-dl * dt).exp()) / dl;
                let g = (1.0 - (-dt / DOB_S).exp()) * p.observer_share(v);
                self.w_hat = (self.w_hat + g * (r_now - r_hat) / c).clamp(-W_MAX, W_MAX);
            }
        }
        // Carried to where the new command starts (the loop's own delay,
        // the last command still in force): the Smith step.
        let a_prev = kb * u_prev + kr * rud_prev + self.w_hat;
        let (r0, turned) = advance(dmp, r_now, a_prev, io);
        let e0 = d.err_deg - turned;
        let e_mid = e0 - r0 * tick / 2.0;
        // The yaw rate asked: the path's own plus the heading's, inside the
        // yaw limit and the braking curve, slewed from the last asked.
        let alpha = p.ref_accel(v, d.u_max);
        let k = heading_gain(tick);
        let fb = e_mid.signum() * (k * e_mid.abs()).min((2.0 * alpha * e_mid.abs()).sqrt());
        let r_max = d.r_max_dps.max(1.0);
        let r_cmd = (d.ff_dps + fb).clamp(-r_max, r_max);
        let from = self.r_ref.unwrap_or(r0);
        let r_ref = from + (r_cmd - from).clamp(-alpha * tick, alpha * tick);
        self.r_ref = Some(r_ref);
        // The rate loop: the plant over the tick, inverted.
        let a = (-dmp * tick).exp();
        let c = (1.0 - a) / dmp;
        let lam = (-tick / rate_tc_s(tick)).exp();
        let r_end = r_ref + lam * (r0 - r_ref);
        let need = (r_end - a * r0) / c - self.w_hat;
        let rudder = match d.rudder {
            Rudder::Given(x) => {
                if x.is_finite() {
                    x
                } else {
                    0.0
                }
            }
            Rudder::Own { slew_per_s } => {
                let want = (need / kr.max(RUDDER_FULL_MIN_DPS2)).clamp(-1.0, 1.0);
                let step = slew_per_s.max(0.0) * dt;
                rud_prev + (want - rud_prev).clamp(-step, step)
            }
        };
        let u = if v < STILL_MS || kb < 1e-3 {
            0.0
        } else {
            ((need - kr * rudder) / kb).clamp(-d.u_max, d.u_max)
        };
        self.last = Some(Last {
            r: r_now,
            u,
            u_before: u_prev,
            rudder: if own { rudder } else { 0.0 },
            rudder_before: rud_prev,
            v,
            io,
        });
        Steer {
            diff: u,
            rudder,
            r_ref,
            r_now,
            w_hat: self.w_hat,
        }
    }
}

/// A heading-only tick (the line-up, the roll, the rollout): no path, the
/// drive's own delay, no slip.
#[allow(clippy::too_many_arguments)]
pub fn heading_demand(
    dt: f64,
    v_ms: f64,
    yaw: Yaw,
    err_deg: f64,
    rudder: Rudder,
    u_max: f64,
    r_max_dps: f64,
) -> Demand {
    Demand {
        dt,
        v_ms,
        yaw,
        err_deg,
        ff_dps: 0.0,
        rudder,
        u_max,
        r_max_dps,
        io_delay_s: IO_DELAY_S,
    }
}
// ---------------------------------------------------------------------------
// The path: the route's own geometry ahead, at the distance the heading
// loop's gain makes a pursuit of it.
// ---------------------------------------------------------------------------

fn bearing(a: (f64, f64), b: (f64, f64)) -> f64 {
    (b.0 - a.0).atan2(b.1 - a.1).to_degrees().rem_euclid(360.0)
}

fn angle_diff(a: f64, b: f64) -> f64 {
    (a - b + 540.0).rem_euclid(360.0) - 180.0
}

/// A taxi route in metres (x east, y north) with its cumulative lengths.
#[derive(Debug, Clone)]
pub struct RefPath {
    pts: Vec<(f64, f64)>,
    cum: Vec<f64>,
}

impl RefPath {
    pub fn new(pts: &[(f64, f64)], cum: &[f64]) -> RefPath {
        RefPath { pts: pts.to_vec(), cum: cum.to_vec() }
    }

    /// The point `s` metres along (the ends run on along their legs).
    fn point_at(&self, s: f64) -> (f64, f64) {
        let n = self.pts.len();
        if n < 2 {
            return self.pts.first().copied().unwrap_or((0.0, 0.0));
        }
        let i = match self.cum.iter().position(|&c| c >= s) {
            Some(0) => 1,
            Some(i) => i,
            None => n - 1,
        };
        let (a, b) = (self.pts[i - 1], self.pts[i]);
        let seg = (self.cum[i] - self.cum[i - 1]).max(1e-9);
        let t = (s - self.cum[i - 1]) / seg;
        (a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1))
    }

    /// The bearing of the route's chord across `[s - len / 2, s + len / 2]`,
    /// degrees true.
    pub fn chord_deg(&self, s: f64, len: f64) -> f64 {
        bearing(self.point_at(s - len / 2.0), self.point_at(s + len / 2.0))
    }

    /// `p`'s offset right of the route at `s`, m, against the chord across
    /// `len` centred there (a vertex has no normal of its own).
    pub fn lateral_m(&self, s: f64, p: (f64, f64), len: f64) -> f64 {
        let f = self.point_at(s);
        let h = self.chord_deg(s, len).to_radians();
        // Right of a heading h: (cos h, -sin h) in (east, north).
        (p.0 - f.0) * h.cos() - (p.1 - f.1) * h.sin()
    }

    /// A point's (along-route position, lateral right of the route
    /// positive), the nearest leg's (tests and replays; the drive has its
    /// own projection).
    #[cfg(test)]
    pub fn project(&self, p: (f64, f64)) -> (f64, f64) {
        let mut best = (f64::INFINITY, 0.0, 0.0);
        for i in 1..self.pts.len() {
            let (a, b) = (self.pts[i - 1], self.pts[i]);
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let len = (dx * dx + dy * dy).sqrt().max(1e-9);
            let t = (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len).clamp(0.0, len);
            let f = (a.0 + t * dx / len, a.1 + t * dy / len);
            let d = ((p.0 - f.0).powi(2) + (p.1 - f.1).powi(2)).sqrt();
            if d < best.0 {
                let right = ((p.0 - a.0) * dy - (p.1 - a.1) * dx) / len;
                best = (d, self.cum[i - 1] + t, right);
            }
        }
        (best.1, best.2)
    }
}


/// The pursuit's aim distance at `v_ms`, m: the lateral loop's closure
/// time's worth of travel ([`lateral_s`]: with the aim at `2 v / k` the
/// pursuit's heading gain is the heading loop's `k`, and its lateral mode is
/// damped 0.7), never inside [`MIN_CLOSE_M`].
pub fn aim_m(v_ms: f64, tick_s: f64) -> f64 {
    (v_ms.max(0.0) * lateral_s(tick_s)).max(MIN_CLOSE_M)
}

/// A right angle's chord turns fastest at its middle, `2 / L` radians per
/// metre: this many degrees over `L` at the peak.
pub const RIGHT_ANGLE_PEAK_DEG: f64 = 114.6;

/// The span the reference chord is read across at `v_ms`, m: the aim, and
/// never shorter than a right angle needs to be swept inside the airframe's
/// yaw limit (`v 114.6 / r_max`): a shorter chord asks a rate the wheels
/// cannot give and the corner is flown wide.
pub fn chord_m(v_ms: f64, tick_s: f64, r_max_dps: f64) -> f64 {
    aim_m(v_ms, tick_s).max(v_ms.max(0.0) * RIGHT_ANGLE_PEAK_DEG / r_max_dps.max(1.0))
}

/// The taxi's tick against the route: (the heading error the law flies,
/// the yaw rate the route asks). The reference heading is the route's
/// CHORD across the span `C` ([`chord_m`]: the aim, or what a right angle
/// needs inside `r_max_dps`) centred [`lead_s`] of travel ahead (the inner
/// loop's lag), and its rate along the route is fed forward (`v` times the
/// chord's turn per metre); the lateral off the route, read against the
/// chord centred on the wheels, is closed at the pursuit's own gain
/// (`atan(y / L)`, `L` the aim). The reference and its rate are one curve,
/// so a corner is flown as the arc the chord sweeps, turning from half the
/// span before the vertex to half after (inside a right angle by about a
/// quarter of `C`, a vertex cut to the scorer), and a kink or a scatter
/// shorter than the span is averaged out, never steered. A pursuit angle
/// less half the span's turn read the same corner as a turn the OTHER way
/// whenever the turn sat at the far end of the span (a replayed LEMG corner
/// asked +95 degrees against a fed-forward 70 deg/s left). `s` is the
/// drive's along-route position, `here` the aircraft in the route's frame.
/// Pure.
#[allow(clippy::too_many_arguments)]
pub fn path_demand(
    path: &RefPath,
    s: f64,
    here: (f64, f64),
    psi_true: f64,
    v_ms: f64,
    law: &GroundSteer,
    r_max_dps: f64,
) -> (f64, f64) {
    let tick = law.tick_s();
    let v = v_ms.max(0.0);
    let l = aim_m(v, tick);
    let c = chord_m(v, tick, r_max_dps);
    let ahead = s + v * lead_s(tick);
    let psi_ref = path.chord_deg(ahead, c);
    let d = c / 4.0;
    let ff = v * angle_diff(path.chord_deg(ahead + d, c), path.chord_deg(ahead - d, c)) / (2.0 * d);
    let lat = path.lateral_m(s, here, c);
    let err = angle_diff(psi_ref - (lat / l).atan().to_degrees(), psi_true);
    (err, ff)
}
#[cfg(test)]
pub(crate) mod sim {
    //! The identified plant as a simulation: the brake and rudder written at
    //! a tick reach the aircraft `delay_s` later and hold (the plugin's
    //! channels), integrated in 0.01 s steps.
    use super::GroundPlant;

    #[derive(Debug, Clone)]
    pub struct YawSim {
        pub plant: GroundPlant,
        pub delay_s: f64,
        /// Standing yaw, deg/s^2.
        pub w: f64,
        pub r: f64,
        pub psi: f64,
        /// The track, degrees true: it follows the nose at the plant's
        /// side rate, and the slip between them is what the weathervane
        /// pushes back on.
        pub trk: f64,
        t: f64,
        sent: Vec<(f64, f64, f64)>,
    }

    impl YawSim {
        pub fn new(plant: GroundPlant, delay_s: f64, w: f64, psi: f64, r: f64) -> Self {
            YawSim { plant, delay_s, w, r, psi, trk: psi, t: 0.0, sent: vec![(-1.0, 0.0, 0.0)] }
        }

        /// Write (diff, rudder) now, then fly `dt` at `v` m/s; returns the
        /// mean yaw rate over the tick.
        pub fn run(&mut self, diff: f64, rudder: f64, v: f64, dt: f64) -> f64 {
            self.sent.push((self.t, diff, rudder));
            let h = 0.01;
            let n = (dt / h).round().max(1.0) as usize;
            let h = dt / n as f64;
            let psi0 = self.psi;
            for _ in 0..n {
                let (_, u, d) = *self
                    .sent
                    .iter()
                    .rev()
                    .find(|(t, ..)| *t <= self.t - self.delay_s + 1e-9)
                    .unwrap_or(&(0.0, 0.0, 0.0));
                let p = &self.plant;
                let slip = (self.psi - self.trk + 540.0).rem_euclid(360.0) - 180.0;
                let rd = p.kb(v) * u + p.kr(v) * d - p.d(v) * self.r - p.kv(v) * slip + if v > 0.3 { self.w } else { 0.0 };
                let r1 = self.r + rd * h;
                self.psi += (self.r + r1) / 2.0 * h;
                self.r = r1;
                // The track follows the nose; at a walk it is the nose.
                self.trk += (p.kt(v) * slip * h).clamp(-slip.abs(), slip.abs());
                self.t += h;
            }
            if self.sent.len() > 64 {
                self.sent.drain(..32);
            }
            (self.psi - psi0) / dt
        }
    }
}

#[cfg(test)]
mod tests {
    use super::sim::YawSim;
    use super::*;

    /// The plants, the ticks and the delays the law is held to.
    /// (name, the law's plant, the flown plant, tick, delay, scale).
    pub(crate) fn cases() -> Vec<(&'static str, GroundPlant, GroundPlant, f64, f64, f64)> {
        let mut v = Vec::new();
        for (name, p) in [("sr22", GroundPlant::SR22), ("evolution", GroundPlant::EVOLUTION)] {
            for tick in [0.3, 0.45, 0.6] {
                for scale in [0.7, 1.0, 1.3] {
                    for delay in [0.0, 0.1, 0.2] {
                        v.push((name, p, p.scaled(scale, 1.0), tick, delay, scale));
                    }
                }
            }
        }
        v
    }

    /// A heading step at `v`: settles without ringing and never overshoots
    /// much, on every plant, tick and delay. Returns (overshoot deg, time
    /// to within 2 degrees, s).
    #[allow(clippy::too_many_arguments)]
    fn heading_step(
        nominal: GroundPlant,
        p: GroundPlant,
        tick: f64,
        delay: f64,
        v: f64,
        step_deg: f64,
        w: f64,
    ) -> (f64, f64, f64) {
        let mut law = GroundSteer::new(nominal);
        let mut sim = YawSim::new(p, delay, w, 0.0, 0.0);
        let (mut over, mut settled, mut peak_u) = (0.0_f64, None, 0.0_f64);
        let mut t = 0.0;
        while t < 30.0 {
            let err = step_deg - sim.psi;
            let s = law.step(heading_demand(tick, v, Yaw::Now(sim.r), err, Rudder::Given(0.0), 0.45, 20.0));
            peak_u = peak_u.max(s.diff.abs());
            sim.run(s.diff, 0.0, v, tick);
            t += tick;
            over = over.max((sim.psi - step_deg) * step_deg.signum());
            if (step_deg - sim.psi).abs() < 2.0 {
                settled.get_or_insert(t);
            } else {
                settled = None;
            }
        }
        (over, settled.unwrap_or(f64::INFINITY), peak_u)
    }

    #[test]
    fn a_heading_step_settles_on_every_plant_tick_and_delay() {
        for (name, nominal, p, tick, delay, scale) in cases() {
            for v in [0.5, 1.5, 3.0, 6.0] {
                let (over, settle, _) = heading_step(nominal, p, tick, delay, v, 30.0, -1.5);
                assert!(
                    over < 3.0 && settle < 12.0,
                    "{name} x{scale} tick {tick} delay {delay} at {v} m/s: overshoot {over:.1}, settled {settle:.1} s"
                );
            }
        }
    }

    #[test]
    fn a_reversal_at_a_walk_is_turned_round_and_stopped() {
        // The turn-round (LILB 03's leg two, 1790732094): 180 degrees at
        // 0.1 to 1 m/s.
        let (mut worst_over, mut worst_settle) = ((0.0_f64, String::new()), (0.0_f64, String::new()));
        for (name, nominal, p, tick, delay, scale) in cases() {
            let (over, settle, _) = heading_step(nominal, p, tick, delay, 1.0, 170.0, -1.7);
            let why = format!("{name} x{scale} tick {tick} delay {delay}");
            if over > worst_over.0 {
                worst_over = (over, why.clone());
            }
            if settle > worst_settle.0 {
                worst_settle = (settle, why);
            }
        }
        eprintln!("reversal: overshoot {worst_over:?}, settled {worst_settle:?}");
        assert!(worst_over.0 < REVERSAL_OVERSHOOT_DEG && worst_settle.0 < 25.0, "{worst_over:?} {worst_settle:?}");
    }

    const REVERSAL_OVERSHOOT_DEG: f64 = 5.0;

    #[test]
    fn the_standing_yaw_is_learnt_and_held_off() {
        // The Evolution's -1.7 deg/s^2 at 3 m/s: the heading held within half
        // a degree once learnt, where a proportional law holds a standing
        // error of w / (k D) (the LILB 03 creep held 4 to 5 degrees off).
        let p = GroundPlant::EVOLUTION;
        let mut law = GroundSteer::new(p);
        let mut sim = YawSim::new(p, 0.05, -1.7, 0.0, 0.0);
        let mut worst_late = 0.0_f64;
        for k in 0..200 {
            let s = law.step(heading_demand(0.45, 3.0, Yaw::Now(sim.r), -sim.psi, Rudder::Given(0.0), 0.45, 20.0));
            sim.run(s.diff, 0.0, 3.0, 0.45);
            if k > 60 {
                worst_late = worst_late.max(sim.psi.abs());
            }
        }
        assert!(worst_late < 0.5, "{worst_late:.2} deg");
        assert!((law.w_hat() + 1.7).abs() < 0.3, "learnt {:.2}", law.w_hat());
    }

    #[test]
    fn a_mean_rate_is_carried_to_the_tick() {
        // A heading differenced over the tick reads the rate half a tick
        // late; carried on the model the law flies as on the sim's `R`.
        let p = GroundPlant::SR22;
        for tick in [0.3, 0.45, 0.6] {
            let mut law = GroundSteer::new(p);
            let mut sim = YawSim::new(p, 0.05, -0.9, 0.0, 0.0);
            let mut mean = 0.0;
            let mut over = 0.0_f64;
            for _ in 0..80 {
                let s = law.step(heading_demand(tick, 2.0, Yaw::Mean(mean), 40.0 - sim.psi, Rudder::Given(0.0), 0.45, 20.0));
                mean = sim.run(s.diff, 0.0, 2.0, tick);
                over = over.max(sim.psi - 40.0);
            }
            assert!(over < 3.0 && (sim.psi - 40.0).abs() < 1.0, "tick {tick}: over {over:.1}, end {:.1}", sim.psi);
        }
    }

    #[test]
    fn a_stop_keeps_its_brake_and_the_difference() {
        assert_eq!(stop_wheels(0.3, 0.0), (0.3, 0.0, 0.0));
        let (s, l, r) = stop_wheels(0.3, 0.2);
        assert!((s - 0.2).abs() < 1e-9 && l == 0.0 && (r - 0.4).abs() < 1e-9);
        // The inside wheel at full brake: the rest off the outside.
        let (s, l, r) = stop_wheels(0.8, -0.5);
        assert!((l - 1.0).abs() < 1e-9 && (s - 0.5).abs() < 1e-9 && r == 0.0);
        assert_eq!(wheels(-0.2), (0.2, 0.0));
        assert_eq!(wheels(0.1), (0.0, 0.1));
    }

    fn square() -> (Vec<(f64, f64)>, Vec<f64>) {
        // North 100 m, then a right-angle right turn east 100 m.
        let pts = vec![(0.0, 0.0), (0.0, 100.0), (100.0, 100.0)];
        (pts, vec![0.0, 100.0, 200.0])
    }

    #[test]
    fn the_route_ahead_is_a_turn_and_a_pursuit() {
        let (pts, cum) = square();
        let path = RefPath::new(&pts, &cum);
        let law = GroundSteer::new(GroundPlant::SR22);
        // On the line, the route straight ahead: nothing asked.
        let (e, ff) = path_demand(&path, 50.0, (0.0, 50.0), 0.0, 4.0, &law, 60.0);
        assert!(e.abs() < 1e-6 && ff.abs() < 1e-6, "{e} {ff}");
        // A metre right of it: the lateral closed at the pursuit's gain, the
        // aim the lateral loop's closure time of travel.
        let (e, _) = path_demand(&path, 50.0, (1.0, 50.0), 0.0, 4.0, &law, 60.0);
        let l = aim_m(4.0, law.tick_s());
        assert!((l - 4.0 * lateral_s(0.3)).abs() < 1e-9);
        assert!((e + (1.0 / l).atan().to_degrees()).abs() < 1e-6, "{e}");
        // At a walk the aim is floored.
        assert_eq!(aim_m(1.0, 0.3), MIN_CLOSE_M);
        // The chord across the vertex is half the turn, and the corner ahead
        // is fed forward as a right turn with the heading asked the same way.
        assert!((path.chord_deg(100.0, 10.0) - 45.0).abs() < 1e-9);
        let (e, ff) = path_demand(&path, 98.0, (0.0, 98.0), 0.0, 4.0, &law, 60.0);
        assert!(ff > 5.0 && e > 10.0 && e < 90.0, "{e} {ff}");
        // A yaw limit lengthens the chord so a right angle is swept inside
        // it: the fed-forward rate never asks more than the limit.
        let c = chord_m(4.0, 0.3, 20.0);
        assert!((c - 4.0 * RIGHT_ANGLE_PEAK_DEG / 20.0).abs() < 1e-9 && c > l);
        let peak = (0..400)
            .map(|i| path_demand(&path, 80.0 + i as f64 * 0.1, (0.0, 80.0 + i as f64 * 0.1), 0.0, 4.0, &law, 20.0).1)
            .fold(0.0, f64::max);
        assert!(peak <= 20.5, "{peak}");
        // Projection.
        let (s, lat) = path.project((50.0, 99.0));
        assert!((s - 150.0).abs() < 1e-6 && (lat - 1.0).abs() < 1e-6, "{s} {lat}");
    }

    /// The corner flown through the whole cascade on the plant: inside the
    /// taxi limits at every tick, plant and delay.
    #[test]
    fn a_right_angle_corner_is_flown_on_the_line() {
        let (pts, cum) = square();
        let path = RefPath::new(&pts, &cum);
        let mut worst = ((0.0_f64, String::new()), (0.0_f64, String::new()));
        for (name, nominal, p, tick, delay, scale) in cases() {
            for v in [1.5, 2.5] {
                let mut law = GroundSteer::new(nominal);
                let mut sim = YawSim::new(p, delay, -1.0, 0.0, 0.0);
                let (mut x, mut y) = (0.5, 40.0);
                let mut worst_cut = 0.0_f64;
                let mut worst_out = 0.0_f64;
                while x < 60.0 {
                    let (s, lat) = path.project((x, y));
                    let (err, ff) = path_demand(&path, s, (x, y), sim.psi, v, &law, 20.0);
                    let st = law.step(Demand {
                        dt: tick,
                        v_ms: v,
                        yaw: Yaw::Now(sim.r),
                        err_deg: err,
                        ff_dps: ff,
                        rudder: Rudder::Own { slew_per_s: 0.8 },
                        u_max: 0.45,
                        r_max_dps: law.taxi_rate_limit(v, 0.45, 20.0),
                        io_delay_s: IO_DELAY_S,
                    });
                    let psi0 = sim.psi;
                    sim.run(st.diff, st.rudder, v, tick);
                    let psi_m = ((psi0 + sim.psi) / 2.0).to_radians();
                    x += v * tick * psi_m.sin();
                    y += v * tick * psi_m.cos();
                    // Inside the vertex (the cut) and outside the line.
                    if lat > 0.0 {
                        worst_cut = worst_cut.max(lat);
                    } else {
                        worst_out = worst_out.max(-lat);
                    }
                }
                let why = format!("{name} x{scale} tick {tick} delay {delay} at {v} m/s");
                if worst_cut > worst.0 .0 {
                    worst.0 = (worst_cut, why.clone());
                }
                if worst_out > worst.1 .0 {
                    worst.1 = (worst_out, why);
                }
            }
        }
        eprintln!("corner: inside {:?}, outside {:?}", worst.0, worst.1);
        assert!(worst.0 .0 < CORNER_CUT_M && worst.1 .0 < CORNER_OUT_M, "{worst:?}");
    }
    const CORNER_CUT_M: f64 = 5.0;
    /// The chord sweeps the vertex as an arc: 1.6 m outside at the slowest
    /// brake and a 0.6 s tick (2026-09-30), inside the scorer's 3 m line.
    const CORNER_OUT_M: f64 = 2.0;
}
