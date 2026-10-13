//! The rollout: brakes and steering to the vacate speed, the bounce hold,
//! wheels-on, the balk call and the uncleared-landing watch.

use super::*;

/// Brake ratio per m/s² of wanted deceleration: a flat 0.45 measured about
/// 3.3 m/s² on the Baron, aero drag included, so the wheel share is a little
/// under that (EGHI, 2026-08-02).
pub(super) const BRAKE_RATIO_PER_MPS2: f64 = 0.15;
/// The rollout hands off to a following vacate at or below this groundspeed —
/// taxi pace, never a stop. Stopping dead on an active runway and then
/// re-accelerating to leave it was neither procedure nor comfortable.
pub(super) const VACATE_HANDOFF_KTS: f64 = 10.0;

/// What physics requires to be at `target_kts` with 30% of the remaining
/// runway still in hand, m/s²: the safety term, immune to any comfort
/// figure. Pure.
pub(super) fn rollout_required_decel(gs_kts: f64, target_kts: f64, remaining_m: f64) -> f64 {
    let v = (gs_kts.max(0.0)) / MS_TO_KTS;
    let vt = (target_kts.max(0.0)) / MS_TO_KTS;
    let usable = (remaining_m * 0.7).max(30.0);
    ((v * v - vt * vt) / (2.0 * usable)).max(0.0)
}

/// The deceleration a rollout aims for overall, drag and
/// brakes together, m/s²: a pilot lets the drag do the early work. The
/// G1000 172 got 0.24 of brake from the touch, 2.4 m/s² in all, and the
/// pilot called it extreme (LEMG 13, 2026-10-11).
pub(super) const LIGHT_DECEL_MPS2: f64 = 1.2;

/// The rollout's brake on the deceleration MEASURED:
/// only the brake the drag does not already give toward
/// [`LIGHT_DECEL_MPS2`], found as it goes; the runway's own requirement
/// (`rollout_required_decel`) is a floor at once. Nothing calibrated: the
/// brake's effect is read, not assumed.
#[derive(Debug, Default, Clone, Copy)]
pub struct DecelBrake {
    brake: f64,
    prev_gs: Option<f64>,
    decel_f: Option<f64>,
}

impl DecelBrake {
    /// The symmetric brake ratio for this tick.
    pub fn step(&mut self, gs_kts: f64, dt_s: f64, required_mps2: f64) -> f64 {
        let dt = if dt_s.is_finite() { dt_s.clamp(0.05, 1.0) } else { 0.4 };
        // A late tick is no measurement: a reading carried over a 1.2 s gap
        // read as no deceleration and stepped the brake 0.06 to 0.27 at
        // 40 kt (LEMG 13, 1791755237).
        if !(dt_s.is_finite() && dt_s <= 0.8) {
            self.prev_gs = None;
        }
        if let Some(prev) = self.prev_gs.filter(|_| gs_kts.is_finite()) {
            let now = (prev - gs_kts) / dt / MS_TO_KTS;
            let f = self.decel_f.get_or_insert(now);
            *f += (now - *f) * (dt / 1.0).min(1.0);
        }
        self.prev_gs = gs_kts.is_finite().then_some(gs_kts);
        let want = required_mps2.max(LIGHT_DECEL_MPS2);
        if let Some(have) = self.decel_f {
            self.brake = (self.brake + (want - have) * BRAKE_RATIO_PER_MPS2 * dt).clamp(0.0, 0.8);
        }
        let floor = if required_mps2 > LIGHT_DECEL_MPS2 { required_mps2 * BRAKE_RATIO_PER_MPS2 } else { 0.0 };
        self.brake.max(floor).min(0.8)
    }
}

/// The rollout's differential brake on an airframe that steers on its brakes
/// (a castoring nosewheel, `taxi.brake_steer`), as the real aircraft is
/// steered down the runway: the stopping brake moved from the outside wheel
/// to the inside one, the total unchanged.
///
/// LEMG 13 (issue #166, 2026-09-28, the SR22, sortie 1790619465): the
/// rollout steered on the rudder alone with both brakes at 0.24. The nose
/// swung 6 degrees right in the first two seconds on the wheels, and for ten
/// seconds from 70 kt it sat 3 to 6 degrees right of the runway while the
/// steer asked for left, growing to 0.69 and the trim to its 0.3 cap: fitted
/// over those ticks the rudder's yaw is under 2 deg/s per unit of steer, no
/// authority at all, and the aircraft rolled from 1.6 m to 12.1 m right of
/// the centreline. The takeoff's split (`autotakeoff::roll_brake_split`)
/// fades out from 20 to 40 kt because a brake on the roll costs the
/// acceleration; on the rollout the brake is wanted anyway, so it steers at
/// every speed. The heading error is the takeoff roll's target less the
/// heading (`roll_target_heading`), damped on its own rate; the SR22's taxi
/// ticks give its brakes 40 to 75 deg/s of yaw per unit of difference at
/// taxi pace, and the gain holds the recorded rollout inside 2.6 m from 15 to
/// 60 deg/s per unit.
///
/// On a FREE castoring nose (`aircraft::nose_free`: the SR22, the Evolution,
/// the RV-10) the difference is the one ground law's (`ground_steer`): the
/// identified plant inverted over the tick, the rollout's rudder a known
/// input, the standing yaw (the idle propeller's) learnt by its observer.
/// It replaces the SR22's declared split (`rollout_brakes:`, a gain
/// schedule, lead and trim fitted to LEMG 13 1790689843 and 1790694457,
/// 2026-09-29) and the shared law the Evolution flew into 8.3 m at LILB 03
/// (1790762771, 2026-09-30: 0.14 of difference at 0.75 of stop, the nose
/// 3.7 degrees right of its target for six seconds from 66 kt).
#[derive(Debug, Default, Clone)]
pub struct RolloutBrakeSteer {
    pub(super) prev_err: Option<f64>,
    pub(super) rate: f64,
    /// The one ground law on a free castoring nose, with the last heading
    /// (its yaw rate is the heading differenced over the tick).
    pub(super) free: Option<(crate::ground_steer::GroundSteer, Option<f64>)>,
    /// The free law's last tick, for the record.
    pub(super) last: Option<crate::ground_steer::Steer>,
}

/// Brake difference per degree of heading error...
pub(super) const ROLLOUT_BRAKE_STEER_PER_DEG: f64 = 0.03;
/// ...led by this many seconds of the error's rate...
pub(super) const ROLLOUT_BRAKE_STEER_LEAD_S: f64 = 0.8;
/// ...never more than this difference between the wheels...
pub(super) const ROLLOUT_BRAKE_STEER_MAX: f64 = 0.3;
/// ...and none below this groundspeed, where the stop owns the brakes.
pub(super) const ROLLOUT_BRAKE_STEER_MIN_KTS: f64 = 4.0;
/// A free nose's most wheel difference on the rollout: an inside wheel at
/// full brake takes the rest off the outside (`ground_steer::stop_wheels`).
pub(super) const ROLLOUT_FREE_DIFF_MAX: f64 = 0.5;

/// One tick of [`RolloutBrakeSteer`]: the brakes, as (the symmetric channel,
/// the left wheel, the right wheel). The plugin gives each wheel the harder
/// of the symmetric channel and its own, so the outside wheel's brake is the
/// symmetric channel and the inside one's its own.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RolloutBrakes {
    pub sym: f64,
    pub left: f64,
    pub right: f64,
}

impl RolloutBrakeSteer {
    /// The law for the live airframe: the one ground law on a free
    /// castoring nose, else the shared split.
    pub fn for_airframe() -> Self {
        if crate::aircraft::nose_free() {
            Self::free(crate::ground_steer::GroundPlant::for_airframe())
        } else {
            Self::default()
        }
    }

    /// The one ground law on this plant.
    pub fn free(plant: crate::ground_steer::GroundPlant) -> Self {
        Self {
            free: Some((crate::ground_steer::GroundSteer::new(plant), None)),
            ..Self::default()
        }
    }

    /// The learnt standing yaw (the free law's, deg/s^2), for the record.
    pub fn trim(&self) -> f64 {
        self.free.as_ref().map_or(0.0, |(law, _)| law.w_hat())
    }

    /// The free law's last tick, for the record.
    pub fn last(&self) -> Option<crate::ground_steer::Steer> {
        self.last
    }

    /// The heading error (degrees, target right of the nose positive), the
    /// tick, groundspeed and the stopping brake the rollout wants.
    #[cfg(test)]
    pub fn step(&mut self, err_deg: f64, dt_s: f64, gs_kts: f64, brake: f64) -> RolloutBrakes {
        self.step_with(err_deg, None, None, 0.0, dt_s, gs_kts, brake)
    }

    /// [`Self::step`] with the heading and the track (true degrees: the free
    /// law's yaw rate and the slip its tail weathervanes on) and the rudder
    /// written this tick (echoed; the roll law's).
    #[allow(clippy::too_many_arguments)]
    pub fn step_with(
        &mut self,
        err_deg: f64,
        psi_deg: Option<f64>,
        track_deg: Option<f64>,
        rudder: f64,
        dt_s: f64,
        gs_kts: f64,
        brake: f64,
    ) -> RolloutBrakes {
        let brake = if brake.is_finite() {
            brake.clamp(0.0, 1.0)
        } else {
            0.0
        };
        let plain = RolloutBrakes {
            sym: brake,
            left: 0.0,
            right: 0.0,
        };
        if !err_deg.is_finite() || !dt_s.is_finite() || dt_s <= 0.0 {
            // A failed read is no measurement: the plain stop.
            return plain;
        }
        if let Some((law, prev_psi)) = self.free.as_mut() {
            let mean = match (psi_deg, *prev_psi) {
                (Some(p), Some(q)) => crate::angles::signed_diff_deg(p, q) / dt_s.clamp(0.05, 3.0),
                _ => 0.0,
            };
            *prev_psi = psi_deg;
            if !gs_kts.is_finite() || gs_kts < ROLLOUT_BRAKE_STEER_MIN_KTS {
                law.release();
                return plain;
            }
            let _ = track_deg;
            let s = law.step(crate::ground_steer::Demand {
                io_delay_s: crate::ground_steer::ROLLOUT_IO_DELAY_S,
                ..crate::ground_steer::heading_demand(
                    dt_s,
                    gs_kts / MS_TO_KTS,
                    crate::ground_steer::Yaw::Mean(mean),
                    err_deg,
                    crate::ground_steer::Rudder::Given(rudder),
                    ROLLOUT_FREE_DIFF_MAX,
                    crate::aircraft::live_profile()
                        .and_then(|p| p.taxi.as_ref().and_then(|t| t.max_yaw_dps))
                        .unwrap_or(crate::ground_steer::DEFAULT_MAX_YAW_DPS),
                )
            });
            self.last = Some(s);
            let (sym, left, right) = crate::ground_steer::stop_wheels(brake, s.diff);
            return RolloutBrakes { sym, left, right };
        }
        if let Some(p) = self.prev_err {
            self.rate = (err_deg - p) / dt_s.clamp(0.05, 3.0);
        }
        self.prev_err = Some(err_deg);
        if !gs_kts.is_finite() || gs_kts < ROLLOUT_BRAKE_STEER_MIN_KTS {
            return plain;
        }
        let diff = (ROLLOUT_BRAKE_STEER_PER_DEG * (err_deg + ROLLOUT_BRAKE_STEER_LEAD_S * self.rate))
            .clamp(-ROLLOUT_BRAKE_STEER_MAX, ROLLOUT_BRAKE_STEER_MAX);
        let half = diff.abs() / 2.0;
        let outside = (brake - half).max(0.0);
        let inside = (brake + half).min(1.0);
        if diff > 0.0 {
            // Nose right: the right wheel brakes harder.
            RolloutBrakes {
                sym: outside,
                left: 0.0,
                right: inside,
            }
        } else if diff < 0.0 {
            RolloutBrakes {
                sym: outside,
                left: inside,
                right: 0.0,
            }
        } else {
            plain
        }
    }
}

/// The rollout's steer on a STEERED nosewheel that declares its own law
/// (`rollout_steer:`, [`crate::aircraft::RolloutSteerProfile`], issue #197);
/// every other airframe keeps the takeoff roll's default law. Pure.
///
/// LEMG 13 (iMac 2026-09-30, the King Air C90B): every contact yawed the
/// nose 8 to 15 deg/s left with nothing steering, and the default law, its
/// whole authority 0.35 at 80 kt, answered on a heading DIFFERENCED over
/// a 0.45 s tick: 0.35 of right steer for a second and a half, the nose
/// swung 30 to 80 degrees right, and full left steer held none of it until
/// 45 kt (1790757702 off the runway, 176 m right). Fitted over the three
/// rollouts the steer is 45 to 65 deg/s per unit at 80 kt after 0.3 s and
/// the slip between the nose and the track feeds its own yaw: the swing is
/// seen first in the yaw RATE, so the law damps on the rate the sim
/// measures (`sim/flightmodel/position/R`), holds its gain low at speed and
/// raises it as `ref_kts / gs` toward the nosewheel's own steering.
#[derive(Debug, Clone, Copy)]
pub struct RolloutSteer {
    pub(super) prof: crate::aircraft::RolloutSteerProfile,
    pub(super) prev_err: Option<f64>,
    pub(super) out: f64,
}

/// The rollout steer's slew, units per second (the takeoff roll's).
pub(super) const ROLLOUT_STEER_SLEW_PER_S: f64 = 1.5;
/// The rollout steer's authority under `min_kts` (the takeoff roll's).
pub(super) const ROLLOUT_STEER_MAX: f64 = 0.85;

impl RolloutSteer {
    pub fn new(prof: crate::aircraft::RolloutSteerProfile) -> Self {
        Self {
            prof,
            prev_err: None,
            out: 0.0,
        }
    }

    /// The gain per degree at this groundspeed.
    pub fn per_deg_at(&self, gs_kts: f64) -> f64 {
        let p = self.prof;
        let v = if gs_kts.is_finite() { gs_kts } else { p.ref_kts };
        p.per_deg * p.ref_kts / v.clamp(p.min_kts, p.ref_kts)
    }

    /// The most steer at this groundspeed: the full authority at `min_kts`
    /// and under, `max_fast` from `fast_kts`.
    pub fn cap_at(&self, gs_kts: f64) -> f64 {
        let p = self.prof;
        let v = if gs_kts.is_finite() { gs_kts } else { p.fast_kts };
        let share = ((v - p.min_kts) / (p.fast_kts - p.min_kts)).clamp(0.0, 1.0);
        ROLLOUT_STEER_MAX + (p.max_fast - ROLLOUT_STEER_MAX) * share
    }

    /// Off the wheels (a bounce): nothing is written, so the next contact
    /// slews from neutral.
    pub fn in_air(&mut self) {
        self.out = 0.0;
        self.prev_err = None;
    }

    /// The heading error (degrees, target right of the nose positive), the
    /// tick, groundspeed and the sim's yaw rate (deg/s, nose right
    /// positive; `None` on a failed read, when the error's own rate stands
    /// in).
    pub fn step(&mut self, err_deg: f64, dt_s: f64, gs_kts: f64, yaw_dps: Option<f64>) -> f64 {
        if !err_deg.is_finite() {
            return self.out;
        }
        let dt = if dt_s.is_finite() { dt_s.clamp(0.05, 1.0) } else { 0.4 };
        let differenced = self.prev_err.map_or(0.0, |p| -(err_deg - p) / dt);
        self.prev_err = Some(err_deg);
        let yaw = yaw_dps.filter(|y| y.is_finite()).unwrap_or(differenced);
        let cap = self.cap_at(gs_kts);
        let want =
            (self.per_deg_at(gs_kts) * (err_deg - self.prof.damp_s * yaw)).clamp(-cap, cap);
        let step = ROLLOUT_STEER_SLEW_PER_S * dt;
        self.out += (want - self.out).clamp(-step, step);
        self.out
    }
}

/// The rollout's view of the wheels, tick by tick: on them, off them after
/// the latch (a bounce, blackboxed `bounce` once per spell with its height),
/// or down again. Pure.
#[derive(Debug, Default, Clone, Copy)]
pub struct BounceHold {
    pub(super) in_air: bool,
    pub(super) spells: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BounceTick {
    OnWheels,
    /// The first airborne tick of a spell.
    Left,
    InAir,
    /// The first tick back on the wheels after a spell.
    Landed,
}

impl BounceHold {
    pub fn tick(&mut self, airborne: bool) -> BounceTick {
        let t = match (self.in_air, airborne) {
            (false, false) => BounceTick::OnWheels,
            (false, true) => {
                self.spells += 1;
                BounceTick::Left
            }
            (true, true) => BounceTick::InAir,
            (true, false) => BounceTick::Landed,
        };
        self.in_air = airborne;
        t
    }

    /// Spells in the air since the latch.
    pub fn spells(&self) -> u32 {
        self.spells
    }
}

/// Whether the live airframe steers on its brakes (the takeoff's own test).
pub(super) fn rollout_brake_steers() -> bool {
    crate::aircraft::nose_castors()
        || crate::aircraft::live_profile()
            .and_then(|p| p.taxi.as_ref().map(|t| t.brake_steer))
            .unwrap_or(false)
}

/// Set by full auto before its landing: a vacate drive follows this rollout,
/// so end it at taxi pace instead of a dead stop. Consumed (swapped false) by
/// the rollout that honours it and cleared on abort, so a standalone "land for
/// me" — where nobody follows — always brakes to a stop as before.
pub static VACATE_FOLLOWS: AtomicBool = AtomicBool::new(false);

/// The runway this autoland actually rolled out on, set at rollout entry.
///
/// Where we ARE, as against where a clearance says we should be — and for
/// vacating, being on the strip is the whole of the question. A landing
/// clearance can be missing for reasons that have nothing to do with the
/// pavement under the wheels: at Malaga the tower ordered a go-around, which
/// voided the clearance, the second approach was never re-cleared, and the
/// vacate — gated on a STANDING clearance — refused. The aircraft braked to
/// a stop on runway 13 and sat there through four incursion warnings with an
/// arrival closing to 1.3 nm (2026-08-05). Landing without a clearance makes
/// clearing the runway more urgent, not less. This flight's only
/// (`Scoped`): a stood-down run's late wheels-on write, landing after the
/// door forgot the last one, is born dead.
pub(super) static LANDED_RUNWAY: crate::scoped::Scoped<String> = crate::scoped::Scoped::flight();

/// A new flight forgets the last one's landing: the runway it rolled out on
/// (a hand-landed 31 after a reposition onto the same strip vacated as the
/// old 13) and its tracking (`flight_state::forget_flight`).
pub(crate) fn forget_flight() {
    LANDED_RUNWAY.clear();
    TRACKING_ERROR.clear();
}

/// The runway the last autoland rolled out on, if it has not been consumed.
pub fn landed_runway() -> Option<String> {
    LANDED_RUNWAY.get()
}

/// Wheels on: the autopilot's approach is over and its servos come off. It
/// flew the heading bug and the VS dial to the ground, and nothing ever took
/// it out: on the ground a heading servo cannot bank the aircraft, so its
/// error never closes and it winds the ailerons over (EGCC 23L, 2026-09-21:
/// the 737 taxied in with the yoke half way to the right stop, servos on,
/// the bug a quarter of a degree off the nose). Read first, read back, and
/// blackboxed `ap_disconnected`. `None` = it was already off, or no reading.
pub(super) async fn disconnect_autopilot(xp: &dyn crate::xplane::XPlane) -> Option<bool> {
    // The whole autopilot, not only the servos: the arrest takes the servos
    // out at the roundout, so `servos_on` read 0 on the wheels while the
    // master (`autopilot_on`) held HDG and VS lit and the yaw damper stayed
    // on, and nothing came out (King Air LEMG 13, 1790780099-approach and
    // 1790780311-approach, iMac 2026-09-30: `ap` true down to 32 kt; one
    // swung 70 degrees right from 91 kt under full left steer, #197).
    let on = |r: Result<f64, _>| r.is_ok_and(|v: f64| v > 0.5);
    let (servos, master, damper) = tokio::join!(
        crate::autoflight::read_status(&*xp, SERVOS_ON),
        xp.get_float(AP_MASTER_ON),
        xp.get_float(YAW_DAMPER_ON),
    );
    let (servos, master, damper) = (on(servos), on(master), on(damper));
    if !servos && !master && !damper {
        return None;
    }
    if servos || master {
        let _ = autoflight::press(xp, Button::DisconnectFd).await;
    }
    if damper {
        let _ = xp.run_command(CMD_YAW_DAMPER_OFF).await;
    }
    let (s2, m2, d2) = tokio::join!(
        crate::autoflight::read_status(&*xp, SERVOS_ON),
        xp.get_float(AP_MASTER_ON),
        xp.get_float(YAW_DAMPER_ON),
    );
    let off = |r: Result<f64, _>| r.is_ok_and(|v: f64| v < 0.5);
    let ok = off(s2) && (!master || off(m2)) && (!damper || off(d2));
    crate::blackbox::record(
        "autoland",
        serde_json::json!({"event": "ap_disconnected", "ok": ok,
            "servos": servos, "master": master, "yaw_damper": damper}),
    );
    Some(ok)
}

/// Rollout starts only after weight-on-wheels. The speed terms distinguish a
/// landing from a low pass once on the surface; they must never substitute for
/// the ground signal because a normal flare can satisfy both in the air.
///
/// ...but the wheels themselves are the best evidence there is. A 737 that
/// touches down a couple of knots over its approach speed (LEMG 12,
/// 2026-09-26: IAS 147 against 145, 146 kt over the ground) satisfied
/// neither speed term, so the loop went on flying the FLARE on the runway
/// for 2.8 s, then balked `overflew_the_runway` from the pavement and
/// firewalled the throttle. `ground_ticks` consecutive guidance ticks with
/// the wheels on and the radio under [`WHEELS_ON_RADIO_FT`] is a landing at
/// any speed; a bounce breaks the count.
pub(super) fn should_start_rollout(
    already_rolling: bool,
    on_ground: bool,
    radio_ft: f64,
    gs_kts: f64,
    ias_kts: f64,
    approach_ias: f64,
    wheels_on_s: Option<f64>,
) -> bool {
    already_rolling
        || (on_ground
            && radio_ft < WHEELS_ON_RADIO_FT
            && wheels_on_s.is_some_and(|s| s >= WHEELS_ON_S - 1e-9))
        || (on_ground && radio_ft < 5.0 && (gs_kts < 45.0 || ias_kts < approach_ias))
}

/// Seconds of wheels on, from the first read that saw them, that make a
/// landing whatever the speed. It was three consecutive TICKS, which is
/// 0.4 s from first contact at the MacBook's 0.2 s flare tick (the recorded
/// 1790448340 contact below latched 0.4 s after it) and 0.8 s at the
/// iMac's 0.39 s (2026-09-30): a count of ticks is a time only on the
/// machine it was tuned on.
pub(super) const WHEELS_ON_S: f64 = 0.4;
/// ...with the radio altimeter under this.
pub(super) const WHEELS_ON_RADIO_FT: f64 = 2.0;

/// The clock [`should_start_rollout`] reads: seconds since the first read
/// with the wheels on under [`WHEELS_ON_RADIO_FT`] (`Some(0.0)` on that
/// read, plus the tick's `dt_s` on each after), or `None` (a bounce, a
/// failed read).
pub(super) fn wheels_on_time(prev: Option<f64>, on_ground: bool, radio_ft: f64, dt_s: f64) -> Option<f64> {
    if on_ground && radio_ft.is_finite() && radio_ft < WHEELS_ON_RADIO_FT {
        Some(prev.map_or(0.0, |s| s + dt_s.max(0.0)))
    } else {
        None
    }
}

/// NO BALK FROM THE PAVEMENT. With the wheels on the runway a landing has
/// happened: a go-around from there at 145 kt with the throttle closed, the
/// spoilers out and the autobrake armed is the most dangerous thing the loop
/// could ask for. LEMG 12 (2026-09-26): 2.8 s on the wheels in the flare,
/// then `overflew_the_runway` at 0.33 ft radio, full power, "going around"
/// to the tower, and a landing clearance voided for an aircraft that then
/// stopped on the runway. On the ground any balk, and a flare turned into
/// anything else, stays a flare down the runway's course, which the rollout
/// latch takes over within ticks. Pure; the balk refused is returned.
pub fn wheels_hold_the_landing(
    g: Guidance,
    on_ground: bool,
    was_flaring: bool,
    course_deg: f64,
) -> (Guidance, Option<&'static str>) {
    if !on_ground || !(g.balk.is_some() || (was_flaring && g.phase != ApproachPhase::Flare)) {
        return (g, None);
    }
    let refused = g.balk.or(Some("phase_change"));
    (
        Guidance {
            target_heading_deg: course_deg,
            phase: ApproachPhase::Flare,
            balk: None,
            arresting: true,
            ..g
        },
        refused,
    )
}

/// A balked landing called at or above this radio height is a go-around the
/// moment it is called: from here no spool-up ends on the runway (a 737's
/// idle-to-go-around spool, 6 to 8 s at the path's 800 fpm, loses about 100
/// ft), so the tower hears it at once. Below it the call waits for a climb.
pub(super) const GO_AROUND_CALL_AGL_FT: f64 = 200.0;
/// The climb that makes a low balk real: the radio this far above its
/// lowest since the balk, with the VSI (one tick old) positive.
pub(super) const GO_AROUND_RISE_FT: f64 = 10.0;

/// A balk being flown, watched until it is a go-around or a landing: the
/// balk's own gate and height, the lowest radio since, and whether the
/// wheels have touched.
#[derive(Debug, Clone, Copy)]
pub(super) struct BalkWatch {
    pub(super) why: Option<&'static str>,
    pub(super) balk_radio_ft: f64,
    pub(super) low_ft: f64,
    pub(super) touched: bool,
}

impl BalkWatch {
    pub(super) fn new(why: Option<&'static str>, radio_ft: f64) -> Self {
        Self {
            why,
            balk_radio_ft: radio_ft,
            low_ft: radio_ft,
            touched: false,
        }
    }

    pub(super) fn note(&mut self, radio_ft: f64, on_ground: bool) {
        if radio_ft.is_finite() {
            self.low_ft = self.low_ft.min(radio_ft);
        }
        self.touched |= on_ground;
    }
}

/// The "going around" radio call goes out once per balk that was actually
/// FLOWN as a go-around, and never with the wheels on the runway. A balk
/// called low may still land: LFBD 11 (2026-09-27) the 737 balked
/// `overflew_the_runway` at 7.9 ft radio, the call went out on that tick,
/// the tower answered "roger, go around" and dropped the landing clearance,
/// and the wheels touched 1.3 s later with the engines still spooling; the
/// crew had called a go-around and then landed uncleared. So a low balk is
/// called once it CLIMBS (`GO_AROUND_RISE_FT` above its lowest radio, the
/// VSI positive when it could be read), and a balk whose wheels touched
/// only once it is clear of [`GO_AROUND_CALL_AGL_FT`] (a touch that then
/// really left); one that rolls out never calls. A balk at or above that
/// height calls at once; the first LFBD approach's `off_centreline_low` at
/// 96 ft calls as soon as it climbs. Pure.
pub(super) fn go_around_call_due(
    missed: bool,
    said: bool,
    on_ground: bool,
    watch: Option<&BalkWatch>,
    radio_ft: f64,
    vvi_fpm: Option<f64>,
) -> bool {
    if !missed || said || on_ground {
        return false;
    }
    let Some(w) = watch else { return true };
    if radio_ft >= GO_AROUND_CALL_AGL_FT {
        return true;
    }
    !w.touched && radio_ft - w.low_ft >= GO_AROUND_RISE_FT && vvi_fpm.is_none_or(|v| v > 0.0)
}

/// Radio height at which a final the tower is working but has NOT cleared
/// asks again: "final runway one five, request landing clearance" (a final
/// report, which the tower judges afresh against the strip).
pub const CLEARANCE_QUERY_FT: f64 = 1000.0;
/// Radio height at which a final still without its landing clearance goes
/// around: a crew never lands uncleared (EGKK-EGBB 15, sortie
/// 1790601822-full_auto, 2026-09-28: "expect late landing clearance" at
/// 7 nm, then nothing, and autoland landed; "you landed on runway one five
/// without clearance"). Above the flare by far: the order is the gate path's
/// (`GoAroundLatch::order`), flown from wherever we are.
pub const UNCLEARED_GO_AROUND_FT: f64 = 300.0;

/// What the crew owes a final the tower is working and has not cleared.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnclearedStep {
    /// Ask again, once per approach.
    Ask,
    /// Go around.
    GoAround,
}

/// The crew's watch on its own landing clearance. `working` is the tower's
/// book saying it has us in sequence and has not cleared us (`Inbound`: the
/// "continue approach, expect late landing clearance" answer, or a final
/// report not yet answered); without a tower working us (an uncontrolled
/// field, no two-way contact) there is nothing to be cleared by and the
/// watch is silent. `departed` is a book still holding our takeoff (a
/// circuit's tower keeps us): the crew ASKS, since the tower clears a final
/// report from any state, but never goes around on it, since a tower we
/// have left would never answer. LEMG 31, 2026-10-07: "autoland runway 31"
/// out of a circuit landed without a call and was told so. Pure.
#[derive(Debug, Default)]
pub struct UnclearedWatch {
    pub(super) asked: bool,
}

impl UnclearedWatch {
    pub fn step(
        &mut self,
        working: bool,
        departed: bool,
        radio_ft: f64,
        on_ground: bool,
        missed: bool,
        established: bool,
    ) -> Option<UnclearedStep> {
        if missed {
            // The next approach asks again.
            self.asked = false;
            return None;
        }
        if !(working || departed) || on_ground || !radio_ft.is_finite() {
            return None;
        }
        if working && radio_ft <= UNCLEARED_GO_AROUND_FT {
            return Some(UnclearedStep::GoAround);
        }
        // The ask is a FINAL report ("final runway one two, request landing
        // clearance"): made only ESTABLISHED on the final
        // (`established_on_final`, the last tick's), never going around,
        // climbing away or in a reposition pattern. LEMG 12 (sortie
        // 1791917103-full_auto, 2026-10-13): asked on the first tick of a
        // re-arm 0.46 nm out, 640 ft above the path, the tick it went round.
        if radio_ft <= CLEARANCE_QUERY_FT && !self.asked && established {
            self.asked = true;
            return Some(UnclearedStep::Ask);
        }
        None
    }
}
