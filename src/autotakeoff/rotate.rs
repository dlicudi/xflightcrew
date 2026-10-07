//! The rotation and initial climb by pitch: `RotatePitch`, `ClimbHold`, the
//! tail ceiling, the climb bank and the throttle ramp.

use super::*;

/// The rotation elevator: an integrating law on pitch attitude, damped on
/// the pitch rate. Pulls the nose to [`ROTATE_PITCH_DEG`] and holds it
/// there through the initial climb, until the AP takes the established
/// climb at [`AP_ENGAGE_AGL_FT`].
#[derive(Default)]
pub struct RotatePitch {
    pub(super) out: f64,
    pub(super) prev_theta: Option<f64>,
    /// The standing pull the trim state demands — the INTEGRAL part, kept
    /// separate from `out` so the proportional path can move without
    /// disturbing it.
    ///
    /// This law was a PURE integrator and rang from liftoff to the 300 ft AP
    /// handover, where it stopped dead because the autopilot flies it from
    /// there: 5.6° to 13.2° about an 8° target, hard against `PITCH_MIN` on
    /// every down-swing (SF50 at EGHI, 2026-07-30). Elevator to pitch rate to
    /// attitude is already about a double integration; a third in the
    /// controller cannot be stabilised by one lead term at any gain.
    ///
    /// Going P+I fixed the structure and did NOT stop the ringing, which is
    /// the part worth remembering: across 49 takeoffs in the blackbox the
    /// same-airframe pair either side of that change (SF50, EGHI 20, one
    /// evening) went 1.26 -> 1.89 direction reversals per second and 20% ->
    /// 48% of ticks pinned against `PITCH_MIN`. The peak-to-peak halved and
    /// the oscillation got FASTER. What was actually driving it was the
    /// damping term's rate estimate and its 1 s lead — see
    /// [`RotatePitch::step`] and [`PITCH_LEAD_S`] (2026-08-02).
    pub(super) trim: f64,
    /// The commanded attitude, ramped at [`ROTATE_RATE_DEG_S`] from wherever
    /// the nose was on the first tick. `None` until then.
    pub(super) target: Option<f64>,
    /// Highest attitude this airframe may be commanded to, degrees — its
    /// tail-strike angle less [`TAIL_STRIKE_MARGIN_DEG`]. `None` on airframes
    /// with no geometry limit worth naming (the GA singles: a 172 runs out of
    /// elevator long before it runs out of tail clearance).
    pub(super) ceiling: Option<f64>,
    /// This airframe's nose-down limit — see [`PITCH_MIN`].
    pub(super) pitch_min: f64,
    /// The speed hold by pitch after liftoff, on an airframe that declares
    /// one (see [`ClimbHold`]); `None` holds the rotation attitude, as ever.
    pub(super) hold: Option<ClimbHold>,
    /// The last airspeed read and its low-passed trend, knots per second.
    pub(super) prev_ias: Option<f64>,
    pub(super) accel_kts_s: f64,
    /// The column the climb hold's ceiling guard allows, while it holds one
    /// ([`RotatePitch::ceiling_guard`]); `None` when the nose is inside its
    /// approach to `pitch_max_deg`.
    pub(super) ceiling_limit: Option<f64>,
    /// Tests only: the law as it flew EGKK 08R, without the guard.
    pub(super) unguarded: bool,
    /// The rotation's rate, degrees a second ([`ROTATE_RATE_DEG_S`] unless
    /// the airframe declares its own, `rotate_rate_deg_s`).
    pub(super) rotate_rate: Option<f64>,
    /// The airframe's fitted rotation elevator (`rotate_elevator:`), fed
    /// forward by [`RotatePitch::model_pull`]; `None` is the P+I alone.
    pub(super) model: Option<crate::aircraft::RotateElevatorProfile>,
    /// On the wheels, as the caller last said ([`RotatePitch::on_wheels`]).
    pub(super) wheels: bool,
    /// The attitude the rotation started from (the roll attitude).
    pub(super) rest: Option<f64>,
    /// The model's column, as last fed forward.
    pub(super) model_out: f64,
    /// The vertical speed, fpm, as last read (`vh_ind_fpm`;
    /// [`RotatePitch::climb_fpm`]): with the airspeed, the flight path.
    pub(super) vs_fpm: Option<f64>,
    /// The height's own rate, fpm, low-passed over [`HEIGHT_RATE_TAU_S`],
    /// and the height it was last taken from: the flight path of a model
    /// that declares `path_from_height`.
    pub(super) height_fpm: Option<f64>,
    pub(super) prev_agl: Option<f64>,
    /// Tests only: see [`RotatePitch::as_flown_b2b1481`].
    #[cfg(test)]
    pub(super) as_flown_b2b1481: Option<f64>,
}

/// The share of the fitted rotation model fed forward; the P+I finds the
/// rest. Under one on purpose: a column short of the need is closed from
/// below by the loop, one past it is an overshoot at liftoff. Against the
/// SR22 plant fitted to LEMG (#184) the whole model lifted off 82 kt and
/// overshot to 10.7 degrees; 0.8 lifted off 83 and held under 9.6.
pub const ROTATE_MODEL_SHARE: f64 = 0.8;
/// The flight path the model believes, degrees: a vertical speed read in a
/// bounce or a zoom is not a path to take the spring off by.
pub(super) const ROTATE_MODEL_PATH_MAX_DEG: f64 = 12.0;
/// ...and taken from the height's own rate (`path_from_height`): a jet's
/// initial climb at 17.5 degrees of attitude flies a 14 to 15 degree path
/// (the Zibo at 4,000 fpm and 155 kt), which the 12 degree cap would read as
/// three degrees of spring still to hold.
pub(super) const ROTATE_MODEL_HEIGHT_PATH_MAX_DEG: f64 = 20.0;
/// The slowest airspeed the model's `(ref / ias)^2` is taken at, knots: a
/// rotation is never below it, and a dead ASI must not ask for full column.
pub(super) const ROTATE_MODEL_MIN_KTS: f64 = 40.0;
/// The low-pass on the height's rate, seconds: a tick's difference of a
/// height read to the hundredth of a foot, a quarter of the 1.3 s the
/// flight model's vertical speed trails it by.
pub(super) const HEIGHT_RATE_TAU_S: f64 = 0.25;

/// The hand-flown initial climb's speed hold: from liftoff to the autopilot's
/// engage the rotation's attitude target is walked by the airspeed TREND, so
/// the aircraft climbs at `hold_kts` instead of accelerating away at a fixed
/// 8 degrees. The rotation attitude is its floor (a slow aircraft flies the old
/// law) and `pitch_max_deg` its ceiling, reached only as the tail clears the
/// runway ([`tail_clear_ceiling_deg`]).
///
/// LEMG 13, the 737, 2026-09-28 (five full-auto circuits, every one the same):
/// rotate 145, liftoff 162, then 185 kt by the 300 ft engage at 11 degrees,
/// the law pushing -0.37 to hold 8. EGPH 2026-09-21 the same, 157 to 185. A
/// crew rotates to about 15 degrees and holds V2 + 15 by pitch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClimbHold {
    pub hold_kts: f64,
    pub pitch_max_deg: f64,
}

impl ClimbHold {
    /// From the airframe's declaration and its initial climb speed (V2 + 20
    /// on the jets): hold [`CLIMB_HOLD_UNDER_KTS`] under it, the middle of the
    /// V2 to V2 + 20 band a crew flies. `None` without a sane ceiling above
    /// the rotation attitude.
    pub fn new(initial_climb_kts: f64, pitch_max_deg: Option<f64>) -> Option<Self> {
        let pitch_max_deg = pitch_max_deg.filter(|d| d.is_finite() && *d > ROTATE_PITCH_DEG)?;
        (initial_climb_kts.is_finite() && initial_climb_kts > CLIMB_HOLD_UNDER_KTS).then(|| {
            ClimbHold {
                hold_kts: initial_climb_kts - CLIMB_HOLD_UNDER_KTS,
                pitch_max_deg,
            }
        })
    }
}

/// The initial climb speed over V2 on an airframe whose V2 is read off its
/// card by weight (`aircraft::takeoff_speeds_now`): V2 + 20, the figure the
/// profiles' `initial_climb_kts` already carry, so the climb hold
/// ([`ClimbHold`]) flies V2 + 15.
pub const V2_CLIMB_ADD_KTS: f64 = 20.0;

/// The hold sits this far under the initial climb speed (V2 + 20): V2 + 15.
pub const CLIMB_HOLD_UNDER_KTS: f64 = 5.0;
/// The tail's arm behind the main wheels, feet, for the clearance a height
/// buys: a long one on purpose (the A330-300's is about 100, the 737-800's
/// about 60), so every airframe's ceiling grows no faster than its tail
/// clears. The sim's `y_agl` reads 0.1 ft on the 737's wheels (LEMG
/// 2026-09-28), so the height IS the wheels'.
pub const TAIL_ARM_FT: f64 = 100.0;

/// The attitude the tail allows at `agl_ft` wheel height: the rotation's own
/// ceiling on the runway, plus the angle the height buys over the tail's arm
/// (5.7 degrees at 10 ft, 11.3 at 20, 19.3 at 35). A crew keeps rotating
/// through liftoff; the tail strike is an attitude ON the runway. Pure.
pub fn tail_clear_ceiling_deg(on_runway_deg: f64, agl_ft: f64) -> f64 {
    on_runway_deg + (agl_ft.max(0.0) / TAIL_ARM_FT).atan().to_degrees()
}
/// The airspeed trend's low-pass, seconds. A two-sample difference over a
/// 0.2 s tick turns a tenth of a knot of read noise into half a knot a second.
pub(super) const CLIMB_ACCEL_TAU_S: f64 = 1.0;
/// The acceleration the hold asks for: the speed error closed over this
/// time constant, capped both ways.
pub(super) const CLIMB_SPEED_TAU_S: f64 = 3.0;
pub(super) const CLIMB_ACCEL_MAX_KTS_S: f64 = 1.5;
/// How fast the attitude target walks per knot-per-second of acceleration
/// it should not have, degrees per second. At takeoff thrust a degree of
/// flight path is about a third of a knot a second, so this closes the
/// attitude on the speed in about three seconds.
pub(super) const CLIMB_DEG_S_PER_KT_S: f64 = 1.5;
/// The target walks down no faster than this, degrees per second (up is the
/// rotation's own rate).
pub(super) const CLIMB_PITCH_DOWN_DEG_S: f64 = 1.5;

impl RotatePitch {
    /// `tail_strike_deg` is the airframe's tail-clearance attitude, from the
    /// profile; `None` leaves the law with only its own 8° target.
    ///
    /// There is deliberately no constructor that leaves the authority
    /// implicit. One existed and every caller but the tests moved off it
    /// within the hour — an argument nobody passes is an argument nobody
    /// thinks about, and this one is the difference between a ceiling and a
    /// suggestion. Pass `PITCH_MIN` to mean the GA figure and say so.
    ///
    /// A ceiling the law
    /// cannot push against is not a ceiling: the 737 declares an 11° tail
    /// strike, was commanded 8°, and flew 22.4° because the push it wanted
    /// was four times what it was allowed to ask for (2026-09-20).
    pub fn with_authority(tail_strike_deg: Option<f64>, pitch_min: f64) -> Self {
        RotatePitch {
            out: 0.0,
            prev_theta: None,
            trim: 0.0,
            target: None,
            ceiling: tail_strike_deg.map(|d| d - TAIL_STRIKE_MARGIN_DEG),
            // Never POSITIVE — a "minimum" above zero would be a standing
            // pull the law could not release — and never past full travel.
            pitch_min: pitch_min.clamp(-1.0, 0.0),
            hold: None,
            prev_ias: None,
            accel_kts_s: 0.0,
            ceiling_limit: None,
            unguarded: false,
            rotate_rate: None,
            model: None,
            wheels: true,
            rest: None,
            model_out: 0.0,
            vs_fpm: None,
            height_fpm: None,
            prev_agl: None,
            #[cfg(test)]
            as_flown_b2b1481: None,
        }
    }

    /// The airframe's fitted rotation elevator (`rotate_elevator:`); `None`
    /// (or an insane declaration) keeps the P+I alone, unchanged.
    pub fn with_rotate_model(
        mut self,
        model: Option<crate::aircraft::RotateElevatorProfile>,
    ) -> Self {
        self.model = model.and_then(crate::aircraft::RotateElevatorProfile::valid);
        self
    }

    /// Whether the wheels are on the runway this tick (`onground_any`).
    /// Read only by a declared rotation model.
    pub fn on_wheels(&mut self, on: bool) {
        self.wheels = on;
    }

    /// The flight model's vertical speed this tick (`vh_ind_fpm`), for a
    /// declared rotation model's flight path; `None` (a failed read) keeps
    /// the last.
    pub fn climb_fpm(&mut self, vs_fpm: Option<f64>) {
        if let Some(v) = vs_fpm.filter(|v| v.is_finite()) {
            self.vs_fpm = Some(v);
        }
    }

    /// The model's column right now, for the blackbox (`roll` `pitch_ff`).
    pub fn model_pull(&self) -> f64 {
        self.model_out
    }

    /// The fitted rotation column, fed forward, one law on the wheels and
    /// off them: the column that holds the TARGET attitude's angle of attack
    /// against the tail's spring (the attitude less the flight path; on the
    /// wheels the path is level), plus the breakout, plus the column that
    /// turns the nose at the ramp's rate. The breakout walks to `air_base`
    /// through the ground effect's height. A share of it
    /// ([`ROTATE_MODEL_SHARE`]).
    ///
    /// 1790699819 (LEMG 13, 2026-09-29, #184): the first model took the
    /// spring and the breakout away AT wheels-off, 0.40 to 0.18 of feed at
    /// 81.4 KIAS, as if a lifting wheel were a climbing aircraft. The nose
    /// fell from 5.0 to 3.0 degrees, the wing floated in ground effect, and
    /// the 8 ft liftoff came at 94.7 KIAS. A path is flown before the spring
    /// lets go, and ground effect holds the nose down until it is climbed out
    /// of.
    pub(super) fn model_step(
        &mut self,
        air: Option<(f64, f64)>,
        target: f64,
        want_rate: f64,
        dt: f64,
    ) -> f64 {
        let _ = dt; // read only by the b2b1481 replay (tests)
        let Some(m) = self.model else {
            return 0.0;
        };
        let Some((ias, agl_ft)) = air.filter(|(v, h)| v.is_finite() && h.is_finite()) else {
            // No airspeed: the last column stands (a failed read is no
            // measurement).
            return self.model_out;
        };
        let q = (ias.max(ROTATE_MODEL_MIN_KTS) / m.ref_kts).powi(2);
        let rest = self.rest.unwrap_or(target);
        let (path_deg, lifted) = if self.wheels {
            (0.0, 0.0)
        } else {
            let tas_fpm = ias.max(ROTATE_MODEL_MIN_KTS) * 101.269;
            let (climb, path_max) = if m.path_from_height {
                (self.height_fpm, ROTATE_MODEL_HEIGHT_PATH_MAX_DEG)
            } else {
                (self.vs_fpm, ROTATE_MODEL_PATH_MAX_DEG)
            };
            let path = climb
                .map_or(0.0, |vs| vs.atan2(tas_fpm).to_degrees())
                .clamp(-path_max, path_max);
            (path, (agl_ft / m.ground_effect_ft).clamp(0.0, 1.0))
        };
        let base = m.breakout + (m.air_base - m.breakout) * lifted;
        let per_deg = match m.air_per_deg {
            Some(air) => m.per_deg + (air - m.per_deg) * lifted,
            None => m.per_deg,
        };
        #[cfg(test)]
        if let Some(hold) = self.as_flown_b2b1481.filter(|_| !self.wheels) {
            let goal = ROTATE_MODEL_SHARE * hold / q;
            self.model_out = goal + (self.model_out - goal) * (-dt / 0.2).exp();
            return self.model_out;
        }
        self.model_out = ROTATE_MODEL_SHARE
            * (per_deg * (target - path_deg - rest).max(0.0)
                + (base + want_rate.max(0.0) / m.deg_s_per_unit) / q);
        self.model_out
    }

    /// Tests only: the model's flying half as b2b1481 flew 1790699819, a flat
    /// 0.164 hold reached over 0.2 s at wheels-off.
    #[cfg(test)]
    pub(super) fn as_flown_b2b1481(mut self) -> Self {
        self.as_flown_b2b1481 = Some(0.164);
        self
    }

    /// The law without its ceiling guard: what flew EGKK 08R (#78), for the
    /// replay that pins the guard.
    #[cfg(test)]
    pub(super) fn without_ceiling_guard(mut self) -> Self {
        self.unguarded = true;
        self
    }

    /// The column limit the ceiling guard holds right now, for the blackbox
    /// (`climb_manual` `pitch_lim`); `None` while it holds none.
    pub fn ceiling_limit(&self) -> Option<f64> {
        self.ceiling_limit
    }

    /// Hold the initial climb speed by pitch after liftoff (see
    /// [`ClimbHold`]). `None` is the fixed rotation attitude.
    pub fn with_climb_hold(mut self, hold: Option<ClimbHold>) -> Self {
        self.hold = hold;
        self
    }

    /// The airframe's own rotation rate (`rotate_rate_deg_s`), bounded to
    /// 1.5 to 6 degrees a second; `None` keeps [`ROTATE_RATE_DEG_S`].
    pub fn with_rotate_rate(mut self, deg_s: Option<f64>) -> Self {
        if let Some(r) = deg_s.filter(|r| r.is_finite()) {
            self.rotate_rate = Some(r.clamp(1.5, 6.0));
        }
        self
    }

    pub(super) fn rotate_rate(&self) -> f64 {
        self.rotate_rate.unwrap_or(ROTATE_RATE_DEG_S)
    }

    /// The low-passed airspeed trend, knots per second, for the blackbox.
    pub fn accel_kts_s(&self) -> f64 {
        self.accel_kts_s
    }

    /// The attitude being commanded right now — the ramp, for the blackbox.
    pub fn target_deg(&self) -> f64 {
        self.target.unwrap_or(0.0)
    }

    /// `rate_deg_s` is the sim's MEASURED pitch rate (`sim/flightmodel/
    /// position/Q`). `None` falls back to differencing the attitude, which is
    /// what this law did for its whole life and is the reason it rang.
    ///
    /// A two-sample difference over the loop's 0.25 s tick is a bad estimator
    /// twice over: it amplifies every wobble in the attitude read by 1/dt (a
    /// 1° tick-to-tick movement becomes 4°/s, and at the old 1 s lead that is
    /// 4° of phantom error against a real error of 1-2°), and it carries an
    /// inherent half-sample delay — so the DAMPING term arrives about 50° out
    /// of phase at the ~0.9 s period the aircraft actually rang at, which
    /// makes it drive the motion rather than oppose it. Q has neither problem:
    /// it is measured, not differentiated, so it needs no filter (and a filter
    /// was tried and measurably made things worse, EGHI 2026-07-30 — it put
    /// lag back exactly where the phase margin is thinnest).
    ///
    /// Q is the BODY-axis rate where the law wants the Euler pitch rate; they
    /// differ by cos(bank), and this climb holds the wings inside
    /// `CLIMB_BANK_MAX_DEG`, so the error is under 1%.
    #[cfg(test)]
    pub fn step(&mut self, theta_deg: f64, rate_deg_s: Option<f64>, dt_s: f64) -> f64 {
        self.step_climb(theta_deg, rate_deg_s, dt_s, None)
    }

    /// [`RotatePitch::step`] with the airspeed and height, for the climb
    /// hold: `air` is `(ias_kts, agl_ft)`. Without a declared hold, or with
    /// `air` `None`, exactly the fixed-attitude law.
    pub fn step_climb(
        &mut self,
        theta_deg: f64,
        rate_deg_s: Option<f64>,
        dt_s: f64,
        air: Option<(f64, f64)>,
    ) -> f64 {
        let dt = dt_s.clamp(0.05, 1.0);
        let rate = rate_deg_s
            .unwrap_or_else(|| self.prev_theta.map_or(0.0, |p| (theta_deg - p) / dt))
            .clamp(-30.0, 30.0);
        self.prev_theta = Some(theta_deg);
        // The commanded attitude walks up from where the nose actually was,
        // never further than the airframe's geometry allows.
        let cap = self
            .ceiling
            .map_or(ROTATE_PITCH_DEG, |c| ROTATE_PITCH_DEG.min(c));
        let prev_target = self.target;
        if prev_target.is_none() {
            self.rest = Some(theta_deg);
        }
        let ramped = match prev_target {
            None => theta_deg.min(cap),
            Some(t) if t < cap => (t + self.rotate_rate() * dt).min(cap),
            // Past the ramp: the hold's target stands (the ramp never pulls
            // it back to the rotation attitude).
            Some(t) if self.hold.is_some() => t,
            Some(t) => (t + self.rotate_rate() * dt).min(cap),
        };
        // The airspeed trend, whenever the caller has the airspeed.
        if let Some((ias, _)) = air.filter(|(v, _)| v.is_finite()) {
            let raw = self.prev_ias.map_or(0.0, |p| ((ias - p) / dt).clamp(-10.0, 10.0));
            if self.prev_ias.is_some() {
                self.accel_kts_s += (raw - self.accel_kts_s) * dt / (CLIMB_ACCEL_TAU_S + dt);
            }
            self.prev_ias = Some(ias);
        }
        let held = match (self.hold, air) {
            (Some(h), Some((ias, agl_ft))) if ramped >= cap - 1e-9 && ias.is_finite() => {
                // The climb attitude, as far as the tail has cleared.
                let tail = self
                    .ceiling
                    .map_or(f64::INFINITY, |c| tail_clear_ceiling_deg(c, agl_ft));
                // The target stops CEILING_MARGIN_DEG under the ceiling, so
                // a settled attitude is under it too (#78); the guard keeps
                // the nose from passing the ceiling on the way there.
                let margin = if self.unguarded {
                    0.0
                } else {
                    CEILING_MARGIN_DEG
                };
                let top = (h.pitch_max_deg - margin).min(tail).max(cap);
                let want_accel = ((h.hold_kts - ias) / CLIMB_SPEED_TAU_S)
                    .clamp(-CLIMB_ACCEL_MAX_KTS_S, CLIMB_ACCEL_MAX_KTS_S);
                let walk = (CLIMB_DEG_S_PER_KT_S * (self.accel_kts_s - want_accel) * dt)
                    .clamp(-CLIMB_PITCH_DOWN_DEG_S * dt, ROTATE_RATE_DEG_S * dt);
                Some((ramped + walk).clamp(cap, top))
            }
            _ => None,
        };
        let target = held.unwrap_or(ramped);
        self.target = Some(target);
        // Damp the rate ERROR: while the ramp is still climbing, the nose is
        // SUPPOSED to be moving at ROTATE_RATE_DEG_S, and subtracting that
        // motion as if it were an overshoot is what stalls a rotation. The
        // hold's walk is a commanded rate too.
        let want_rate = match (held, prev_target) {
            (Some(t), Some(p)) => (t - p) / dt,
            _ if target < cap => self.rotate_rate(),
            _ => 0.0,
        };
        let err = target - theta_deg - PITCH_LEAD_S * (rate - want_rate);
        // PROPORTIONAL + integral, not integral alone.
        //
        // Elevator -> pitch rate -> attitude is already about a double
        // integration. A pure integrator on top makes three, and one lead term
        // cannot supply the phase to stabilise that at ANY gain — it was never
        // going to settle, only ring wider or narrower. It rang 5.6-13.2 deg
        // about an 8 deg target, hard against PITCH_MIN on every down-swing.
        //
        // The proportional path answers immediately and adds no lag; the
        // integrator is left to do the one job the original comment actually
        // wanted from it — hold the standing pull the trim state demands, so
        // that on target and steady the elevator does not fall away.
        self.trim = (self.trim + err * PITCH_TRIM_PER_DEG_S * dt).clamp(self.pitch_min, PITCH_MAX);
        // The height's rate, for a model that takes its path from it.
        if self.model.is_some_and(|m| m.path_from_height) {
            match air.filter(|(_, h)| h.is_finite()) {
                Some((_, agl)) => {
                    if let Some(prev) = self.prev_agl {
                        let raw = (agl - prev) / dt * 60.0;
                        self.height_fpm = Some(self.height_fpm.map_or(raw, |f| {
                            f + (raw - f) * dt / (HEIGHT_RATE_TAU_S + dt)
                        }));
                    }
                    self.prev_agl = Some(agl);
                }
                // A failed read is no measurement: the rate stands, and the
                // next height is differenced from nothing older.
                None => self.prev_agl = None,
            }
        }
        // A declared rotation model's column, fed forward (zero without one).
        let fed = self.model_step(air, target, want_rate, dt);
        let out = (err * PITCH_P_PER_DEG + self.trim + fed).clamp(self.pitch_min, PITCH_MAX);
        self.out = match self.hold {
            Some(h) if !self.unguarded => {
                self.ceiling_guard(h.pitch_max_deg, theta_deg, rate, dt, out, err)
            }
            _ => out,
        };
        self.out
    }

    /// The climb attitude is a CEILING, not a target the law may sail
    /// through: the nose may approach `pitch_max_deg` no faster than
    /// [`CEILING_APPROACH_S`] closes the gap, and while it comes faster the
    /// column is walked forward by the excess rate (an integral, so it finds
    /// whatever push this airframe's nose-up moment needs without knowing
    /// it) plus a proportional part for the first tick. The walk lets go only
    /// once the attitude law asks for less than the limit; while it holds,
    /// the attitude law's integrator is held to the limit (it cannot wind
    /// pull the guard then has to undo).
    ///
    /// EGKK 08R, the 737, 2026-09-28 (the first ClimbHold flight): the target
    /// stopped at 18 degrees and the nose flew to 22.4. The law arrived with
    /// the nose rising 2.8 degrees a second and had 0.03 of column per degree
    /// to stop it with; four degrees over it was pushing 0.15 where the 737
    /// needs 0.14 of push just to hold its attitude (the fit in
    /// `the_egkk_climb_stays_under_its_ceiling`).
    pub(super) fn ceiling_guard(
        &mut self,
        pitch_max_deg: f64,
        theta_deg: f64,
        rate: f64,
        dt: f64,
        out: f64,
        err: f64,
    ) -> f64 {
        let excess = rate - (pitch_max_deg - theta_deg) / CEILING_APPROACH_S;
        let walked = match self.ceiling_limit {
            Some(lim) => Some(lim - CEILING_WALK_GAIN * excess * dt),
            None if excess > 0.0 => Some(self.out.min(out) - CEILING_WALK_GAIN * excess * dt),
            None => None,
        };
        // Released once the attitude law itself asks for no more than the
        // limit and the nose is inside its approach.
        self.ceiling_limit = walked
            .filter(|lim| excess > 0.0 || *lim < out)
            .map(|lim| lim.clamp(self.pitch_min, PITCH_MAX));
        let Some(lim) = self.ceiling_limit else {
            return out;
        };
        let u = out
            .min(lim - CEILING_RATE_GAIN * excess.max(0.0))
            .clamp(self.pitch_min, PITCH_MAX);
        self.trim = self.trim.min(u - err * PITCH_P_PER_DEG).max(self.pitch_min);
        u
    }
}

/// The time constant the nose may close on the climb ceiling with, seconds:
/// the rate allowed is the gap over this. Fitted with the two gains below
/// against the EGKK 08R recording across its plant uncertainty
/// (`the_egkk_climb_stays_under_its_ceiling`): 2.0 grazes the ceiling on
/// the slow plants, 3.0 holds the nose a further half degree down for
/// nothing.
pub(super) const CEILING_APPROACH_S: f64 = 2.5;
/// The climb hold's target stops this far under `climb_pitch_max_deg`: a P+I
/// law settles ON its target, grazing a ceiling the target sits on.
pub(super) const CEILING_MARGIN_DEG: f64 = 0.5;
/// The guard's integral gain: column walked forward per second per degree a
/// second of rate over the approach. A gain on the attitude law, not an
/// airframe number: it finds the airframe's push itself, and holds the
/// ceiling from 8 to 32 degrees a second per unit of column (the fit is 12.9).
pub(super) const CEILING_WALK_GAIN: f64 = 0.15;
/// The guard's proportional gain: column pushed at once per degree a second
/// over the approach.
pub(super) const CEILING_RATE_GAIN: f64 = 0.04;

/// The bank the climb-out flies: toward runway heading, capped shallow —
/// this is track-keeping on departure, not a turn.
pub fn climb_bank_target(hdg_err_deg: f64) -> f64 {
    (hdg_err_deg * BANK_PER_HDG_ERR).clamp(-CLIMB_BANK_MAX_DEG, CLIMB_BANK_MAX_DEG)
}

/// Throttle schedule on the roll: 30% immediately (the engine spools, the
/// roll starts, the yaw stays small), full power over the ramp.
pub fn throttle_ramp(elapsed_s: f64) -> f64 {
    (0.3 + 0.7 * (elapsed_s / THROTTLE_RAMP_S)).clamp(0.3, 1.0)
}
