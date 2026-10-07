//! The steering laws: the nosewheel command and its slew, the straight trim,
//! brake steering, the pivot and the line-up's alignment numbers.

use super::*;

/// Steering command for a heading error: proportional, full deflection at
/// [`STEER_FULL_DEG`]. The plant decides whether raw P is enough: a linked
/// nosewheel (172) is self-damped, a castoring one is not — feed this
/// [`damped_err_deg`], never the raw error.
pub fn steer_cmd_with(err_deg: f64, full_deg: f64) -> f64 {
    (err_deg / full_deg).clamp(-1.0, 1.0)
}

/// Rate-limit the steering command toward `want`. A taxiing aircraft is a
/// vehicle with people in it: the lever slamming half its travel in under a
/// second is felt in the cabin as a jolt, whatever it does for tracking.
///
/// Per SECOND and dt-scaled, never per tick — the auto-takeoff roll learned
/// that the hard way (a 0.25-per-tick limiter swept only 0.5/s and the rudder
/// chased the error a second behind in exact steps, a textbook slew-induced
/// limit cycle, 2026-07-21). `None` is no limit, which is every airframe that
/// has never needed one.
pub fn slew_steer(prev: f64, want: f64, per_s: Option<f64>, dt: f64) -> f64 {
    match per_s {
        Some(rate) if dt > 0.0 => {
            let step = rate * dt;
            prev + (want - prev).clamp(-step, step)
        }
        _ => want,
    }
}

/// Lead the heading error by the yaw rate. On a castoring nosewheel (SR22)
/// the steer channel is effectively RUDDER-only — a lagged, integrating
/// plant that pure P sent into a ±40° snake at three knots (heading swung
/// 137°→213°→138° on successive state ticks, 2026-07-22). Leading by where
/// the nose will be in [`STEER_RATE_LEAD_S`] checks the swing before the
/// crest. On a linked nosewheel taxi yaw rates are small and the term is
/// nearly inert, so the 172 keeps its proven feel.
///
/// But "small" was the 172's taxi, not every taxi. A derivative term fed the
/// WHOLE measured rate opposes a steady turn as hard as it opposes a swing,
/// and a bend is exactly where the rate stops being noise and becomes the
/// demand. Measured on the 737 (EIDW, 2026-09-20): 22.9 deg/s of yaw per
/// unit of steer, so in a sustained bend the 0.6 s lead subtracted 13.8 deg
/// of error per unit commanded and the declared 30 deg full-scale was really
/// 43.8 — a third of the steering quietly derated, in the one place it was
/// needed. Every corner of that taxi rounded out 6 to 8 m wide with the
/// command never past 0.55, while the straights held 0.2 m. Understeer, and
/// the 172 hid it the way it hides every jet number: at GA taxi rates the
/// same term is worth about a degree.
///
/// So the lead damps only the yaw the CORNER DID NOT ASK FOR:
/// `required_dps` is the route's own turn rate across the aim span
/// ([`Route::route_turn_rate_dps`]), and it is credited back. Straight paint
/// asks for zero and the law is byte-identical to the one the 172, the SR22
/// and the SF50 fly today — including their weave, which is a STRAIGHT-line
/// oscillation that earns no credit.
pub const STEER_RATE_LEAD_S: f64 = 0.6;
pub fn damped_err_deg(err_deg: f64, yaw_rate_dps: f64, required_dps: f64, lead_s: f64) -> f64 {
    // The credit is bounded into [0, yaw_rate]: a corner may only ever weaken
    // the lead toward zero, never reverse it into a rate DEMAND of its own,
    // and a nose swinging the way the bend does NOT go earns nothing. The
    // residual therefore always keeps the sign of the measured rate and never
    // exceeds it — this can subtract damping, never add drive.
    let credit = required_dps.clamp(yaw_rate_dps.min(0.0), yaw_rate_dps.max(0.0));
    err_deg - (yaw_rate_dps - credit) * lead_s
}

/// The standing steer a straight leg needs, learned from the offset it
/// leaves. The pursuit law is proportional, so any steady bias (a crosswind
/// weathervaning the tail, a castoring nosewheel steered by one brake) is
/// paid for with a steady ERROR: EDLP (2026-09-24) the SF50 held 0.08-0.09
/// of steer and 3-4 degrees toward the line for minutes at a time and never
/// closed 4 m of offset, every straight of the taxi. A slow integral on the
/// signed cross-track, on straights only: frozen through corners, joins,
/// big excursions, the stop and a crawl, and bled away through a bend (a
/// new heading is a new crosswind).
#[derive(Debug, Default, Clone, Copy)]
pub struct StraightTrim {
    pub(super) trim: f64,
}

/// Steer per metre of offset per second. Slow on purpose: 4 m of offset
/// builds a 0.09 trim in about fifteen seconds, longer than the SF50's
/// fourteen-second weave, so the trim cannot join an oscillation.
pub(super) const TRIM_PER_M_S: f64 = 0.0015;
/// The most standing steer the trim may hold.
pub(super) const TRIM_MAX: f64 = 0.15;
/// Offsets beyond this are a corner or a join, not a bias.
pub(super) const TRIM_MAX_OFFSET_M: f64 = 8.0;
/// The route's own turn rate above which a leg is a bend.
pub(super) const TRIM_BEND_DPS: f64 = 1.0;
/// Time constant of the bleed through a bend.
pub(super) const TRIM_BLEED_S: f64 = 3.0;

impl StraightTrim {
    /// `lat_m` right of the route positive, as the blackbox logs it; the
    /// result adds to the steer command (right positive).
    pub fn step(
        &mut self,
        lat_m: f64,
        route_turn_dps: f64,
        gs_kts: f64,
        joining: bool,
        remaining_m: f64,
        dt: f64,
    ) -> f64 {
        let dt = dt.clamp(0.0, 1.0);
        if !lat_m.is_finite() || joining {
            return self.trim;
        }
        if route_turn_dps.abs() > TRIM_BEND_DPS {
            self.trim *= (-dt / TRIM_BLEED_S).exp();
            return self.trim;
        }
        if gs_kts >= 4.0 && remaining_m > 30.0 && lat_m.abs() <= TRIM_MAX_OFFSET_M {
            self.trim = (self.trim - TRIM_PER_M_S * lat_m * dt).clamp(-TRIM_MAX, TRIM_MAX);
        }
        self.trim
    }
}

/// Steering authority vs groundspeed: a fully deflected nosewheel at a
/// standstill is a chock, not a turn — the tire scrubs sideways and the
/// taxi governor's throttle cap (well below takeoff power on purpose)
/// can't break the aircraft away against it. A 69°-off join at Gatwick
/// sat static at full lock exactly this way (2026-07-20). Partial lock
/// parked lets the roll start; full authority arrives by 3 knots, when
/// the wheel is actually steering.
pub fn steer_authority(gs_kts: f64) -> f64 {
    0.35 + 0.65 * (gs_kts / 3.0).clamp(0.0, 1.0)
}

/// The steer command below which the brakes stay out of it. Small on
/// purpose: "small corrections are the yoke channel's" was the original
/// rationale, and on the one class of airframe that carries the assist the
/// yoke channel is DEAD — below the deadband nothing steers at all. At 0.2
/// the SF50 could not make small sustained corrections, so it drifted until
/// the error bought a command past 0.5, then slow-corrected, 13 m out and
/// back for whole legs (2026-07-29, the fifth taxi). A castoring-wheel jet
/// rides a breath of inside brake down a not-quite-straight taxiway because
/// that is the only steering it has — the real aircraft does the same.
pub(super) const BRAKE_STEER_DEADBAND: f64 = 0.05;
/// The most inside-brake the assist may apply. Well under the governor's own
/// stop braking: this is a nudge, not a pivot. History: 0.25 pulled 14 m of
/// offset back in ten seconds on the assist's first sortie but read as
/// "violent, oversteering" from the cabin (2026-07-28) and was cut to 0.16 —
/// but the violence was the OLD unsaturated mapping slamming the full cap on
/// at the deadband edge, a shape that no longer exists, and the retreated cap
/// then could not fly a BEND: fifth taxi, mid-bend at 10-13 kt, steer at
/// 0.52-0.60 — effectively the cap — returned 0-0.5 deg/s of yaw against the
/// 3-4 the curve demanded, and the aircraft ran 24 m off the outside
/// (2026-07-29). Back to the value that demonstrably turned the aircraft,
/// applied smoothly by the saturated mapping and the slew.
pub(super) const BRAKE_STEER_MAX: f64 = 0.25;
/// The command at which the assist reaches that max. NOT 1.0: tracking can
/// never command full deflection — the heading error would have to exceed
/// the whole `steer_full_deg` — so a max reached only at 1.0 is a max the
/// loop can never draw on. The fourth SF50 taxi proved it (2026-07-29):
/// steer sat above 0.5 for two thirds of an 11-minute taxi, which mapped to
/// ~0.07 of brake and 0.69 deg/s of yaw against the ~3.5 the assist gives
/// flat out, and the aircraft held a one-sided 9 m offset (15 m max, the
/// error changing sign four times in 11 minutes) — starvation, not a weave.
/// Saturating at 0.6 hands the tracking band the full authority the cap was
/// sized for.
pub(super) const BRAKE_STEER_SAT: f64 = 0.6;
/// The assist's own slew, per second — pedals squeeze, they don't stab,
/// exactly the taxi governor's brake discipline.
pub(super) const BRAKE_STEER_SLEW_PER_S: f64 = 0.4;
/// Speed band the assist fades in across. Below it the nosewheel's own
/// low-speed steering (and the stop machinery) owns the aircraft — braking a
/// near-stopped aircraft asymmetrically is a lurch.
pub(super) const BRAKE_STEER_MIN_KTS: f64 = 4.0;
pub(super) const BRAKE_STEER_FULL_KTS: f64 = 7.0;

/// Differential-brake steering: the (left, right) brake ratios for a steer
/// command, for airframes flagged [`Pace::brake_steer`].
///
/// The yoke-heading channel is nosewheel + rudder, and on a castoring-wheel
/// JET both halves are dead at taxi speed — the wheel takes no input and a
/// turbofan puts no wash over the rudder. The SF50 proved it: steer −0.6 held
/// for thirty seconds while the cross-track error grew to 13.8 m and the
/// aircraft tracked the grass parallel to the taxiway (Luton, 2026-07-28,
/// the blackbox trail). A touch of inside brake is how the real aircraft is
/// steered.
///
/// One side only, ever — the governor's pilot-brake detector reads
/// `min(left, right)`, deliberately, so a single-side assist never reads as
/// the pilot slowing the aircraft. The governor's speed loop simply carries
/// the extra drag.
pub fn brake_steer_split(steer: f64, gs_kts: f64) -> (f64, f64) {
    let authority =
        ((gs_kts - BRAKE_STEER_MIN_KTS) / (BRAKE_STEER_FULL_KTS - BRAKE_STEER_MIN_KTS)).clamp(0.0, 1.0);
    let excess = ((steer.abs() - BRAKE_STEER_DEADBAND) / (BRAKE_STEER_SAT - BRAKE_STEER_DEADBAND))
        .clamp(0.0, 1.0);
    let b = excess * BRAKE_STEER_MAX * authority;
    if steer > 0.0 {
        (0.0, b) // nose right: right brake
    } else {
        (b, 0.0)
    }
}

/// A reversal is flown ONE way. Pointing almost exactly backwards the error
/// flips between +177 and -179 on noise, and the steering followed it: LOWI 26's
/// backtrack turn (2026-09-24) steered right, then left, then full left, and
/// swung 19 m off the centreline of a 45 m runway. Past [`TURN_COMMIT_DEG`] the
/// first sign is kept until the error is back under [`TURN_RELEASE_DEG`].
pub fn committed_err(err_deg: f64, dir: &mut Option<f64>) -> f64 {
    let mag = err_deg.abs();
    if mag >= TURN_COMMIT_DEG {
        let d = *dir.get_or_insert(if err_deg >= 0.0 { 1.0 } else { -1.0 });
        return d * mag;
    }
    if mag < TURN_RELEASE_DEG {
        *dir = None;
        return err_deg;
    }
    match *dir {
        Some(d) => d * mag,
        None => err_deg,
    }
}
pub(super) const TURN_COMMIT_DEG: f64 = 150.0;
pub(super) const TURN_RELEASE_DEG: f64 = 90.0;

/// The brake half of a PIVOT (a join turned through more than
/// [`PIVOT_MIN_ERR_DEG`]): a castoring nosewheel turns on its inside brake at
/// walking pace, which is exactly where the taxi split fades to nothing (0.06
/// of brake at full lock through LOWI's 180, 2026-09-24). Authority from 1 kt,
/// full by 2, capped at [`PIVOT_BRAKE_MAX`].
pub fn pivot_brake_split(steer: f64, gs_kts: f64) -> (f64, f64) {
    let authority = ((gs_kts - 1.0) / 1.0).clamp(0.0, 1.0);
    let excess = ((steer.abs() - BRAKE_STEER_DEADBAND) / (BRAKE_STEER_SAT - BRAKE_STEER_DEADBAND))
        .clamp(0.0, 1.0);
    let b = excess * PIVOT_BRAKE_MAX * authority;
    if steer > 0.0 {
        (0.0, b)
    } else {
        (b, 0.0)
    }
}
pub(super) const PIVOT_MIN_ERR_DEG: f64 = 60.0;

/// The brake half of the taxi steering follows the steer command through
/// this lag. The steer carries the rate lead's reading of a yaw rate
/// differenced at 5 Hz, and the brake copied it tick for tick: on LILB's
/// straights (2026-09-25) the Evolution's right brake pulsed 0.02 to 0.14
/// every second, the yaw swung +/-1.7 deg/s and the governor surged a knot
/// against each pulse. A foot holds a pressure; it does not tap.
///
/// 0.4 s, not the first 0.8: the longer lag let a corner's correction
/// arrive late, and the Evolution rocked +/-17 deg/s for twenty seconds
/// out of a bend at LIME (2026-09-25).
pub const BRAKE_STEER_TAU_S: f64 = 0.4;

/// One step of a first-order lag toward `want`.
pub fn smooth_toward(prev: f64, want: f64, tau_s: f64, dt: f64) -> f64 {
    prev + (want - prev) * (dt / (tau_s + dt)).clamp(0.0, 1.0)
}

/// A pivot's brake eases above this yaw rate, in the direction it is
/// turning, and is gone [`PIVOT_RATE_SPAN_DPS`] above it: the Evolution's
/// join spun at 35 deg/s on 0.36 of brake, overshot the route by 23 degrees
/// and reversed at full opposite steer (LILB 2026-09-25).
pub const PIVOT_RATE_DPS: f64 = 15.0;
pub(super) const PIVOT_RATE_SPAN_DPS: f64 = 10.0;

/// The share of a pivot's brake still wanted at `yaw_dps`. Pure.
pub fn pivot_rate_scale(steer: f64, yaw_dps: f64) -> f64 {
    if steer * yaw_dps <= 0.0 {
        return 1.0;
    }
    (1.0 - (yaw_dps.abs() - PIVOT_RATE_DPS) / PIVOT_RATE_SPAN_DPS).clamp(0.0, 1.0)
}

/// Whether a castoring nosewheel's turn is a PIVOT: a join past
/// [`PIVOT_MIN_ERR_DEG`], or ANY error that big at a pace under the taxi
/// split's full authority. The pace cap slows the aircraft exactly when the
/// error is large, and under [`BRAKE_STEER_MIN_KTS`] the taxi split is zero:
/// the SR22 swung left through the LSZS 03 lead-on, the error ran 37 to 130
/// degrees at 3 kt with full right steer and no brake at all, and it rolled
/// 25 m off the line into a give-up (2026-09-24).
pub fn pivot_due(err_deg: f64, gs_kts: f64, joining: bool) -> bool {
    err_deg.abs() > PIVOT_MIN_ERR_DEG && (joining || gs_kts < BRAKE_STEER_FULL_KTS)
}
pub(crate) const PIVOT_BRAKE_MAX: f64 = 0.45;

/// The brake half of the taxi steering on a castoring nose that STEERS
/// (the SF50's, 40 degrees of it), one tick: the taxi split on the smoothed
/// steer, the pivot's split past [`PIVOT_MIN_ERR_DEG`] at walking pace, both
/// under the turn-rate ceiling ([`pivot_rate_scale`]), squeezed on and off
/// at [`BRAKE_STEER_SLEW_PER_S`]. A nose with no steering of its own
/// (`aircraft::nose_free`) flies the one ground law instead
/// (`ground_steer`): the swing catch, the turn builder, the led rate
/// ceiling, the wrong-way pivot brake and the instant release that were
/// layered here for it (#165, #196, 2026-09-29/30) are that law's plant
/// inversion and braking curve. Pure.
#[derive(Debug, Default, Clone, Copy)]
pub struct BrakeSteer {
    /// The steer the brakes follow ([`BRAKE_STEER_TAU_S`]).
    pub(super) smoothed: f64,
    /// The last (left, right) pair written.
    pub(super) prev: (f64, f64),
}

impl BrakeSteer {
    /// `steer` right positive, `yaw_dps` the drive's smoothed yaw rate, nose
    /// right positive; returns (left, right) brake.
    pub fn step(
        &mut self,
        steer: f64,
        err_deg: f64,
        gs_kts: f64,
        yaw_dps: f64,
        joining: bool,
        dt: f64,
    ) -> (f64, f64) {
        let pivot = pivot_due(err_deg, gs_kts, joining);
        self.smoothed = smooth_toward(self.smoothed, steer, BRAKE_STEER_TAU_S, dt);
        // The turn-rate ceiling holds for every brake turn, not only a
        // pivot: out of a LIME corner at 8 to 12 kt the brake spun the
        // Evolution at 35 deg/s where the bend asked 14 (2026-09-25).
        let (wl, wr) = if pivot {
            pivot_brake_split(steer, gs_kts)
        } else {
            brake_steer_split(self.smoothed, gs_kts)
        };
        let k = pivot_rate_scale(if pivot { steer } else { self.smoothed }, yaw_dps);
        let (wl, wr) = (wl * k, wr * k);
        let ease = |prev: f64, want: f64| slew_steer(prev, want, Some(BRAKE_STEER_SLEW_PER_S), dt);
        let pair = (ease(self.prev.0, wl), ease(self.prev.1, wr));
        self.prev = pair;
        pair
    }
}

/// The brakes of an ALIGNING pivot, a castoring nose turned onto a heading
/// on its brakes at walking pace: a PD on the heading error, the derivative
/// the sim's own yaw rate (`R`, positive nose right) led by
/// [`ALIGN_LEAD_S`], soft ([`ALIGN_FULL_DEG`] of led error for the whole
/// [`PIVOT_BRAKE_MAX`]) and never pushing a turn already past
/// [`ALIGN_RATE_MAX_DPS`]. A led error of the other sign is the OPPOSITE
/// brake: the swing is caught before it overshoots, not after.
///
/// Why: the takeoff's aligning pivot ran on the error alone, at full brake
/// from 15 degrees, capped only past a 15 deg/s turn. At the iMac's 0.3 to
/// 0.45 s tick (19 fps) and the brake's own half second, the SR22 at LEMG 12
/// (1790690995, 2026-09-29) started its roll 10.8 degrees off, reached 17.5
/// deg/s, overshot to -11.6, and pirouetted +/-12 degrees from 4 to 18 kt;
/// the Evolution at LILB 03 (1790712974) swung from 25 degrees off to -4 at
/// 21.6 deg/s. Tuned on a second-order yaw plant with a 0.3 s brake delay
/// fitted to those ticks, over ticks of 0.3 to 0.5 s. Pure.
pub fn align_pivot_brakes(err_deg: f64, yaw_dps: f64) -> (f64, f64) {
    if err_deg.abs() < ALIGN_QUIET_DEG && yaw_dps.abs() < ALIGN_QUIET_DPS {
        return (0.0, 0.0);
    }
    let led = err_deg - yaw_dps * ALIGN_LEAD_S;
    let mut b = (led / ALIGN_FULL_DEG).clamp(-1.0, 1.0) * PIVOT_BRAKE_MAX;
    if b * yaw_dps > 0.0 && yaw_dps.abs() > ALIGN_RATE_MAX_DPS {
        b = 0.0;
    }
    if b >= 0.0 {
        (0.0, b)
    } else {
        (-b, 0.0)
    }
}
/// The aligning pivot's lead on the yaw rate, s: the tick, the brake's own
/// lag and the yaw's coast after the brake comes off.
pub const ALIGN_LEAD_S: f64 = 2.0;
/// Led error for the pivot's whole brake.
pub const ALIGN_FULL_DEG: f64 = 40.0;
/// The aligning pivot never pushes a turn faster than this.
pub const ALIGN_RATE_MAX_DPS: f64 = 6.0;
/// The free nose's owned rudder slews at this while aligning, per second.
pub(super) const ALIGN_RUDDER_SLEW_PER_S: f64 = 0.8;
/// Inside this error and yaw rate the pivot lets go.
pub(super) const ALIGN_QUIET_DEG: f64 = 1.0;
pub(super) const ALIGN_QUIET_DPS: f64 = 1.0;
/// A line-up is ALIGNED within this of the runway's course: the scorecard
/// judges the roll's first tick at 3.
pub const ALIGNED_DEG: f64 = 2.0;
/// ...and a line-up out of time or runway still hands over within this: the
/// roll holds its power until the nose is inside [`ALIGNED_DEG`].
pub const ALIGN_ACCEPT_DEG: f64 = 3.0;
/// Stopped, for the aligned verdict: groundspeed and yaw rate under these.
pub(super) const ALIGN_SETTLED_KTS: f64 = 0.5;
pub(super) const ALIGN_SETTLED_DPS: f64 = 0.7;
/// The creep the governor is given while the nose still has to come round:
/// a castoring nose pivots on a braked wheel only with the aircraft moving.
pub const ALIGN_CREEP_KTS: f64 = 2.0;
/// ...and with little runway to spare ([`ALIGN_TIGHT_ROLL_M`]), a slower
/// one: the same brake turns the nose through more degrees per metre.
pub const ALIGN_TIGHT_CREEP_KTS: f64 = 1.0;
/// A creep this far over its speed is stopped until it is back under it:
/// the Evolution's idle thrust ran the 2 kt creep up to 5.7 kt with the
/// governor asking 2 (LILB 03, 1790721973, 2026-09-30).
pub const ALIGN_CREEP_OVER_KTS: f64 = 1.0;
/// The alignment's bounds: time from the arrival, and ground rolled down
/// the runway doing it (the runway it can spare lowers the second).
pub const ALIGN_MAX_S: f64 = 40.0;
pub const ALIGN_MAX_ROLL_M: f64 = 40.0;
/// Runway kept back from the spare, m: the fit is judged where the aircraft
/// stops, a creep's stop is anticipated, this is the rest.
pub const ALIGN_SPARE_MARGIN_M: f64 = 2.0;
/// Under this much runway to roll, the tight creep.
pub const ALIGN_TIGHT_ROLL_M: f64 = 12.0;
/// The stop a creep needs, for the runway bound: the governor's brake takes
/// about this deceleration after this latency.
pub(super) const ALIGN_STOP_DECEL_MS2: f64 = 0.5;
pub(super) const ALIGN_STOP_LATENCY_S: f64 = 0.5;
/// The trim that holds a standing yaw: brake per degree of error per
/// second of creep, bounded, integrated only while the nose is not already
/// coming round by [`ALIGN_TRIM_TURNING_DPS`].
pub(super) const ALIGN_TRIM_PER_DEG_S: f64 = 0.006;
pub(super) const ALIGN_TRIM_MAX: f64 = 0.12;
pub(super) const ALIGN_TRIM_TURNING_DPS: f64 = 0.5;
pub(super) const ALIGN_TRIM_MIN_KTS: f64 = 0.7;
/// A line-up is on the centreline within this, m: past it the alignment
/// closes the lateral too ([`LineUpAlign::step_on_line`]). The alignment
/// stage judges the roll's first tick at 2 m (`scorecard::LINEUP_MAX_XTK_M`).
pub const ALIGN_LATERAL_M: f64 = 1.5;
/// ...and once closing, it closes to this before it straightens.
pub(super) const ALIGN_LATERAL_DONE_M: f64 = 0.75;
/// The intercept onto the centreline: degrees per metre of offset past
/// [`ALIGN_INTERCEPT_DEAD_M`], never more than [`ALIGN_INTERCEPT_MAX_DEG`].
pub(super) const ALIGN_INTERCEPT_PER_M: f64 = 6.0;
pub(super) const ALIGN_INTERCEPT_DEAD_M: f64 = 0.3;
pub(super) const ALIGN_INTERCEPT_MAX_DEG: f64 = 8.0;
/// The runway and time kept back from a lateral closure to straighten in
/// after it: past these it stops closing and only aligns.
pub(super) const ALIGN_LATERAL_RESERVE_M: f64 = 10.0;
pub(super) const ALIGN_LATERAL_RESERVE_S: f64 = 12.0;
/// A closure may take the alignment this much longer than
/// [`ALIGN_MAX_S`]: an S across 2 m at a creep is a minute's work, not forty
/// seconds'.
pub(super) const ALIGN_LATERAL_EXTRA_S: f64 = 25.0;

/// The heading offset toward the centreline for a lateral of `lat_m`
/// (right of the centreline positive), degrees, right positive: the
/// intercept a lateral closure flies. Pure.
pub fn align_intercept_deg(lat_m: f64) -> f64 {
    -lat_m.signum()
        * ((lat_m.abs() - ALIGN_INTERCEPT_DEAD_M).max(0.0) * ALIGN_INTERCEPT_PER_M)
            .min(ALIGN_INTERCEPT_MAX_DEG)
}

/// How far a line-up's alignment may roll, from the runway it can spare
/// (the runway ahead less what the takeoff fit needs), never more than
/// [`ALIGN_MAX_ROLL_M`]; `None` (no runway known) is that bound. Pure.
pub fn align_roll_limit_m(spare_m: Option<f64>) -> f64 {
    match spare_m {
        Some(s) => (s - ALIGN_SPARE_MARGIN_M).clamp(0.0, ALIGN_MAX_ROLL_M),
        None => ALIGN_MAX_ROLL_M,
    }
}

/// The runway an alignment at `(lat, lon)` can spare: what is ahead on
/// `rwy` less what the takeoff fit needs (`need_m`, floored at the fit's
/// own [`crate::runway::MIN_REMAINING_M`], as `runway_fit` judges it).
/// Negative when the fit is already lost. Pure.
pub fn align_spare_m(rwy: &crate::runway::Runway, need_m: f64, lat: f64, lon: f64) -> f64 {
    let past_m = (-crate::runway::along_track_nm(lat, lon, rwy) * M_PER_NM).max(0.0);
    rwy.length_m - past_m - need_m.max(crate::runway::MIN_REMAINING_M)
}

/// One step of the aligning pivot's standing-yaw trim (signed brake, +
/// right): integrated while rolling, off the course and not already coming
/// round, bounded. The line-up's ([`LineUpAlign`]) and the roll hold's
/// (`autotakeoff::RollAlign`). Pure.
pub fn align_trim_step(trim: f64, err_deg: f64, yaw_dps: f64, gs_kts: f64, dt: f64) -> f64 {
    if gs_kts > ALIGN_TRIM_MIN_KTS
        && err_deg.abs() > ALIGN_QUIET_DEG
        && yaw_dps * err_deg.signum() < ALIGN_TRIM_TURNING_DPS
    {
        (trim + ALIGN_TRIM_PER_DEG_S * err_deg * dt.clamp(0.0, 1.0))
            .clamp(-ALIGN_TRIM_MAX, ALIGN_TRIM_MAX)
    } else {
        trim
    }
}

/// A (left, right) pivot pair with the trim added, bounded, never pushing
/// a turn past [`ALIGN_RATE_MAX_DPS`]. Pure.
pub fn with_align_trim((bl, br): (f64, f64), trim: f64, yaw_dps: f64) -> (f64, f64) {
    let mut b = (br - bl + trim).clamp(-PIVOT_BRAKE_MAX, PIVOT_BRAKE_MAX);
    if b * yaw_dps > 0.0 && yaw_dps.abs() > ALIGN_RATE_MAX_DPS {
        b = 0.0;
    }
    if b >= 0.0 {
        (0.0, b)
    } else {
        (-b, 0.0)
    }
}
