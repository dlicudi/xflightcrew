//! The climb-out: throttle and thrust reduction, the climb rate and its
//! guards, the declared climb-out, `ClimbVs`, the handover and the reject's
//! gates.

use super::*;

/// Nose this far off at low speed holds the power back — full power on a
/// swinging nose is the maximum P-factor kick at the worst moment (the
/// time-only ramp reached 100% while sortie three was still pirouetting
/// nearly stationary). The hold releases with the nose straight, and it
/// only applies below [`THROTTLE_HOLD_MAX_GS_KTS`]: chopping power over a
/// gust-yaw at 40 knots mid-roll would be its own emergency.
pub(super) const THROTTLE_HOLD_ERR_DEG: f64 = 10.0;
pub(super) const THROTTLE_HOLD_RATIO: f64 = 0.5;
pub(super) const THROTTLE_HOLD_MAX_GS_KTS: f64 = 20.0;

/// The commanded throttle this tick: the time ramp, held at half power
/// while the nose is still swinging at low speed.
pub fn throttle_cmd(elapsed_s: f64, err_deg: f64, gs_kts: f64) -> f64 {
    let ramp = throttle_ramp(elapsed_s);
    if gs_kts < THROTTLE_HOLD_MAX_GS_KTS && err_deg.abs() > THROTTLE_HOLD_ERR_DEG {
        ramp.min(THROTTLE_HOLD_RATIO)
    } else {
        ramp
    }
}

/// Throttle advances are ALWAYS gradual, releases instant: when the
/// swinging-nose hold lets go, the lever walks up at the ramp's own rate
/// instead of jumping half-to-full in one tick — sortie four's release
/// kicked 19 degrees of yaw at 12 knots, exactly the swing the hold
/// existed to prevent. The power feed-forward rides the commanded value,
/// so the rudder walks up with it.
pub(super) const THROTTLE_UP_PER_S: f64 = 0.28;
pub fn throttle_slew(prev: f64, want: f64, dt_s: f64) -> f64 {
    if want <= prev {
        want
    } else {
        (prev + THROTTLE_UP_PER_S * dt_s.clamp(0.02, 1.0)).min(want)
    }
}

/// The ONE throttle command per tick, ground or air. Airborne, the climb's
/// overspeed guard IS the command: the guard used to live in a second,
/// change-gated write further down the loop while this path commanded an
/// unconditional 1.0 every airborne tick, so the guard's reduced lever
/// survived exactly one tick before full power was re-driven over it (the
/// SF50's Vmo overspeed, back again), and with a deep cut the pilot-override
/// detector then read the guard's own write as a hand pulling the lever and
/// handed the takeoff away mid-climb. Two writers on one lever is the whole
/// bug; this seam is where the loop and the test agree on the number.
pub fn tick_throttle_want(
    airborne: bool,
    elapsed_s: f64,
    err_deg: f64,
    gs_kts: f64,
    ias_kts: f64,
    vne_kts: Option<f64>,
) -> f64 {
    if airborne {
        climb_throttle(ias_kts, climb_guard_vne(vne_kts))
    } else {
        throttle_cmd(elapsed_s, err_deg, gs_kts)
    }
}

/// The thrust lever after the declared thrust reduction: above its height,
/// airborne, no more than the climb lever, and whether the reduction BINDS
/// (the lever it leaves is under the takeoff's own want). Takeoff thrust held
/// to the handoff and past it was the follower's to take back (EGPH
/// 2026-09-21); a crew sets climb thrust at the reduction height. Pure.
pub fn thrust_reduced(
    want: f64,
    airborne: bool,
    agl_ft: f64,
    reduction: Option<crate::takeoff_config::ThrustReduction>,
) -> (f64, bool) {
    match reduction {
        Some(r) if airborne && agl_ft >= r.agl_ft && want > r.lever => (r.lever, true),
        _ => (want, false),
    }
}

/// A reduction comes back at this rate, lever per second: the way a crew
/// pulls to climb thrust, not a step the pitch has to answer at once.
pub const THRUST_REDUCE_PER_S: f64 = 0.1;

/// The lever while the reduction binds: `slewed` (the takeoff's own slew,
/// whose releases are instant) no further below `last` than
/// [`THRUST_REDUCE_PER_S`] a second. Every other release stays instant. Pure.
pub fn reduction_slew(slewed: f64, last: f64, dt_s: f64) -> f64 {
    slewed.max((last - THRUST_REDUCE_PER_S * dt_s.max(0.0)).min(last))
}

/// The commanded climb rate: the profile's best rate, backed off while the
/// airspeed is below the climb speed — speed first, rate second, the same
/// energy rule autoland's go-around flies. Never below a positive floor:
/// once airborne the aircraft keeps climbing away from the ground.
pub fn climb_vs_fpm(ias_kts: f64, vy_kts: f64, max_fpm: f64) -> f64 {
    // The floor must never EXCEED the request: the procedure follower passes
    // its own shrinking level-off command as the cap, and f64::clamp PANICS
    // on min > max — clamp(150, 145.9) killed the enroute follower thread
    // dead at 7,950 ft, fifty feet under its 8,000 ft level-off, and the
    // flight continued on whatever the AP dials last held (LXGB to LEMG,
    // 2026-08-02). A request already under the floor is granted as asked:
    // near level-off, "keep climbing away from the ground" is not the
    // question any more.
    let cap = max_fpm.max(0.0);
    (cap - (vy_kts - ias_kts).max(0.0) * 60.0).clamp(150.0_f64.min(cap), cap)
}

/// What the handoff owes the takeoff flaps: `None` when they are already up,
/// `Some(true)` to retract them now, `Some(false)` when the aircraft is too
/// slow to clean up and the next pair of hands must be TOLD so.
///
/// The in-climb schedule waits for Vy, and Vy is not a speed every airframe
/// sees before this loop ends. A 737 lifts at V2+20 and hands over at 1,000 ft
/// doing 181 kt against a profile Vy of 210, so its one-shot never fired: the
/// flaps stayed at 5 from Lyon to FL360 and on to Heathrow's top of descent,
/// the entire climb sitting on their 250 kt placard (2026-09-17). A takeoff
/// that ends with the aircraft still configured for takeoff has not finished.
///
/// `clean_floor_kts` is the airframe's clean-configuration minimum (1.2 x
/// clean Vs): below it, retracting is the stall the schedule exists to avoid,
/// so the flaps stay out and the copilot says why. An airframe that declares
/// no floor has nothing to judge against and cleans up, as it always did.
pub fn clean_up_at_handoff(
    retracted: bool,
    ias_kts: f64,
    clean_floor_kts: Option<f64>,
) -> Option<bool> {
    if retracted {
        return None;
    }
    Some(clean_floor_kts.is_none_or(|floor| ias_kts > floor))
}

/// How long the handoff waits, still flying the climb, for the speed that
/// lets the flaps come up. The A330 reached its handoff height at 203 kt
/// against a 210 kt clean floor, accelerating at 2.5 kt a second: the loop
/// said "too slow to clean up", left, and the aircraft passed the 215 kt
/// placard twelve seconds later with nobody on the flap lever (EGPH,
/// 2026-09-20). Three seconds more would have cleaned it up.
pub const CLEAN_UP_WAIT_S: f64 = 30.0;

/// Whether the handoff holds off, still climbing, for the clean-up: dirty,
/// under the clean floor, and inside [`CLEAN_UP_WAIT_S`].
pub fn handoff_waits_for_clean_up(
    retracted: bool,
    ias_kts: f64,
    clean_floor_kts: Option<f64>,
    waited_s: f64,
) -> bool {
    clean_up_at_handoff(retracted, ias_kts, clean_floor_kts) == Some(false)
        && waited_s < CLEAN_UP_WAIT_S
}

/// The altitude dial a PILOT handoff leaves behind. A dial the climb can
/// never capture (at or under us: the A330's sat at 100 ft, and the VS climb
/// ran through 11,000 with "check altitude capture" said three times) is set
/// 2,000 ft up, rounded up to the thousand. A dial the pilot set above us is
/// the pilot's: `None`.
pub const HANDOFF_DIAL_CLEAR_FT: f64 = 500.0;
pub fn handoff_altitude_dial_ft(dial_ft: f64, alt_ft: f64) -> Option<f64> {
    if dial_ft >= alt_ft + HANDOFF_DIAL_CLEAR_FT {
        return None;
    }
    Some(((alt_ft + 2000.0) / 1000.0).ceil() * 1000.0)
}

/// How far below Vne the overspeed protection starts working, and how much
/// climb rate it trades in per knot of excess.
pub(super) const OVERSPEED_GUARD_KTS: f64 = 20.0;
pub(super) const OVERSPEED_VS_PER_KT: f64 = 60.0;
pub(super) const OVERSPEED_VS_MAX_FPM: f64 = 1500.0;
/// Thrust is pulled no lower than this in the climb — enough to keep climbing.
pub(super) const CLIMB_THROTTLE_MIN: f64 = 0.55;
pub(super) const OVERSPEED_THR_PER_KT: f64 = 0.02;

/// The speed the climb-out guards against: the airframe's Vne, or the 250 kt
/// rule below 10,000 ft plus the guard band, whichever is lower.
///
/// The guard was Vne alone. A 172's Vne is 163 and it cannot get there in a
/// climb; an A330's is 330, and at TOGA with a 1,500 fpm dial it reached 273
/// kt by the 1,000 ft handoff, then Vmo, and the aircraft's own high-speed
/// protection pitched it into a 5,400 fpm climb (EGPH, 2026-09-20). Every
/// takeoff is below 10,000 ft, so the band starts at 250 on any airframe
/// that could exceed it; airframes with a lower Vne are untouched.
pub fn climb_guard_vne(vne_kts: Option<f64>) -> Option<f64> {
    vne_kts.map(|v| v.min(crate::tools::speedhold::BELOW_10K_LIMIT_KTS + OVERSPEED_GUARD_KTS))
}

/// Climb rate with speed protection on BOTH sides.
///
/// [`climb_vs_fpm`] only ever protected the slow side: below Vy it backs the
/// rate off, above Vy it commands the maximum and nothing else. That is right
/// for a trainer, which cannot outrun itself at full power, and wrong for a
/// jet, which can. The SF50 left the runway at full thrust and accelerated
/// through the climb to 270 KIAS against a Vmo of 250 — measured live in the
/// climb, 2026-07-28 — with the copilot commanding a placid 1500 fpm the
/// whole way, because nothing in the law was watching the top end.
///
/// Inside the guard band the excess speed is traded for altitude: climb
/// HARDER, which is what a pilot does with thrust they cannot reduce. The
/// trade is capped so the protection can never become a pitch-up of its own.
pub fn climb_vs_protected(ias_kts: f64, vy_kts: f64, max_fpm: f64, vne_kts: Option<f64>) -> f64 {
    let base = climb_vs_fpm(ias_kts, vy_kts, max_fpm);
    let Some(vne) = vne_kts else { return base };
    let over = ias_kts - (vne - OVERSPEED_GUARD_KTS);
    if over <= 0.0 {
        return base;
    }
    base + (over * OVERSPEED_VS_PER_KT).min(OVERSPEED_VS_MAX_FPM)
}

/// Climb thrust with the same guard. Trading speed for rate is only half an
/// answer — on an airframe with this much excess thrust the aircraft simply
/// re-accelerates — so the lever comes back too, which is the other half of
/// what a pilot does. Never below [`CLIMB_THROTTLE_MIN`]: the point is to stop
/// overspeeding, not to stop climbing. Airframes with no declared Vne are
/// untouched and keep the full-power climb they have always flown.
pub fn climb_throttle(ias_kts: f64, vne_kts: Option<f64>) -> f64 {
    let Some(vne) = vne_kts else { return 1.0 };
    let over = ias_kts - (vne - OVERSPEED_GUARD_KTS);
    if over <= 0.0 {
        return 1.0;
    }
    (1.0 - over * OVERSPEED_THR_PER_KT).max(CLIMB_THROTTLE_MIN)
}

/// The live airframe's manual takeoff power limit
/// ([`crate::aircraft::TakeoffPowerProfile`]), republished on every swap.
pub(super) fn takeoff_power() -> Option<crate::aircraft::TakeoffPowerProfile> {
    crate::aircraft::airframe_facts().takeoff_power.clone()
}

/// Newton-metres to foot-pounds (`ENGN_TRQ`, `sim::state::F::Trq0`, is in N m).
pub(super) const NM_TO_FTLB: f64 = 0.737_562;
/// From this share of the limit the lever stops advancing: a turboprop
/// spooling up and gathering ram air keeps adding torque at a fixed lever.
pub(super) const TORQUE_HOLD_FRAC: f64 = 0.95;
/// Past the limit the lever comes back at least this much per tick, and in
/// proportion to the excess (torque is near proportional to the lever at a
/// given speed), aiming this share under the limit.
pub(super) const TORQUE_BACKOFF_STEP: f64 = 0.02;
pub(super) const TORQUE_AIM_FRAC: f64 = 0.98;

/// The lever under the manual's torque limit: below [`TORQUE_HOLD_FRAC`] of
/// it the law's own want; from there to the limit the lever holds; past the
/// limit it comes back in proportion to the excess. Past `until_kias`, or
/// with no torque read or no limit, the want as asked. Pure.
///
/// It stepped back 0.02 a tick only once 5% over, and a PT6 at a held lever
/// kept climbing: LILB 03 Evolution (1790724857, 2026-09-30) ran 1,735
/// ft-lb against the manual's 1,250, 39% over, by 60 kt.
pub fn torque_limited(
    want: f64,
    last: f64,
    torque_ftlb: Option<f64>,
    ias_kts: f64,
    limit: Option<&crate::aircraft::TakeoffPowerProfile>,
) -> f64 {
    let (Some(l), Some(tq)) = (limit, torque_ftlb) else {
        return want;
    };
    if ias_kts >= l.until_kias || !tq.is_finite() {
        return want;
    }
    if tq > l.torque_ftlb {
        let proportional = last * l.torque_ftlb / tq * TORQUE_AIM_FRAC;
        proportional.min(last - TORQUE_BACKOFF_STEP).max(0.0).min(want)
    } else if tq >= l.torque_ftlb * TORQUE_HOLD_FRAC {
        want.min(last)
    } else {
        want
    }
}

/// Publish the climb-out alone (the snapshot's `climb_out`, republished on
/// every swap by [`crate::flight_state::publish_airframe`]).
#[cfg(test)]
pub fn publish_climb_out(c: Option<ClimbOut>) {
    crate::aircraft::update_airframe_facts(|a| a.climb_out = c);
}

/// The declaration, for a takeoff that hands off at or above its height. A
/// circuit hands off at 500 ft into its own climb law and keeps the old
/// climb-out whole: half a profile is worse than either.
pub(super) fn climb_out_for(handoff_agl_ft: f64) -> Option<ClimbOut> {
    crate::aircraft::airframe_facts()
        .climb_out
        .filter(|c| handoff_agl_ft >= c.accel_agl_ft)
}

/// Climb rate bought per knot above the initial climb speed, below the
/// acceleration height. A 737 loses about a knot a second per 1,000 fpm, so
/// this closes the speed loop at about 0.1 rad/s: slow against the five
/// seconds of lag and dead time in its VS mode (sixty degrees of margin).
pub(super) const SPEED_HOLD_FPM_PER_KT: f64 = 100.0;

/// The climb-out's commanded rate. Below the acceleration height the speed is
/// held by climbing: every knot over `target_kts` buys rate, up to the
/// airframe's best. At and above it, and on any airframe that declares none,
/// the protected climb every takeoff has always flown.
pub fn climb_out_vs_fpm(
    ias_kts: f64,
    target_kts: f64,
    base_fpm: f64,
    vne_kts: Option<f64>,
    agl_ft: f64,
    climb_out: Option<ClimbOut>,
) -> f64 {
    let base = climb_vs_protected(ias_kts, target_kts, base_fpm, vne_kts);
    match climb_out {
        Some(c) if agl_ft < c.accel_agl_ft => {
            let hold = base + (ias_kts - target_kts).max(0.0) * SPEED_HOLD_FPM_PER_KT;
            hold.min(c.max_fpm.max(base))
        }
        _ => base,
    }
}

/// How long the handoff waits, still climbing, for the flap SCHEDULE to
/// finish on an airframe that declares a climb-out. The handoff's own
/// clean-up is one shot to zero on the clean floor: flaps 5 to up at 195 kt,
/// fifteen under the 737's flaps-up manoeuvring speed.
pub const SCHEDULE_WAIT_S: f64 = 45.0;
pub fn handoff_waits_for_schedule(retracted: bool, has_schedule: bool, waited_s: f64) -> bool {
    !retracted && has_schedule && waited_s < SCHEDULE_WAIT_S
}

/// The fastest the commanded climb moves once the autopilot has it, fpm per
/// second: about a tenth of a g of pitch change (0.1 x 9.81 m/s/s is 193 fpm
/// a second), a crew's hand lowering the nose, not a step.
///
/// LEMG 13 (the five circuits of 2026-09-28, e.g. 1790596335-full_auto): the
/// pitch was climbing 2,300 to 2,500 fpm through 300 ft and the engage wrote
/// 1,500; the circuit took it at 500 ft and wrote 456, a thousand fpm off the
/// dial in 0.8 s. After #78 the pitch climbs about 3,100 fpm at V2 + 15, so
/// the old engage would have been a 1,900 fpm push.
pub const CLIMB_VS_SLEW_FPM_S: f64 = 200.0;

/// `prev` toward `want` at [`CLIMB_VS_SLEW_FPM_S`] over `dt_s` (bounded to a
/// second, so a stalled tick cannot turn into a step). Pure.
pub fn climb_vs_slewed(prev: f64, want: f64, dt_s: f64) -> f64 {
    slew(prev, want, CLIMB_VS_SLEW_FPM_S * dt_s.clamp(0.0, 1.0))
}

/// The speed hold's integral, fpm per second per knot off the speed: the
/// proportional part ([`climb_out_vs_fpm`], 100 fpm a knot) closes near 0.1
/// rad/s against the 737's VS mode, and this puts the integral's corner a
/// quarter of that down (0.025 rad/s), about fourteen degrees of the margin.
pub(super) const CLIMB_SPEED_TRIM_FPM_S_PER_KT: f64 = 2.5;
/// A speed-holding climb never commands less than this: airborne, it climbs.
pub(super) const CLIMB_HOLD_FLOOR_FPM: f64 = 150.0;

/// The share of the airframe's best climb flown from the acceleration
/// height until the flaps are clean: the nose lowered so the speed builds
/// for the schedule. LEGR, 2026-10-06 (sortie 1791274391-full_auto, #226):
/// the 737 went back to its 3,000 fpm at 1,000 ft, gained 0.4 kt a second
/// and stepped flaps 5 to 1 at 3,066 ft; at 1,200 fpm the same excess makes
/// about 2.3 kt a second.
pub const CLIMB_ACCEL_FRACTION: f64 = 0.4;
/// Never less than this while accelerating: still a climb.
pub const CLIMB_ACCEL_FLOOR_FPM: f64 = 800.0;

/// How far the hardware lever must move from where it sat as the roll began
/// to be a pilot's hand.
pub const HW_THROTTLE_HAND: f64 = 0.1;

/// Whether a lever read below our command is the pilot's hand. With no
/// hardware axis declared, the lever read alone says so (as always). With
/// one, the axis must have moved [`HW_THROTTLE_HAND`] from its baseline: the
/// Zibo snaps its levers to the hardware lever on any twitch of it (noise
/// lock OFF), and our next write puts them back. Its takeoff at LEMG read
/// the levers at the hardware's 0.28 against 0.535 twice, 2 s into the roll,
/// and handed a standing aircraft to nobody (2026-10-07). A read that
/// failed is no measurement, so no hand. Pure.
pub fn pilot_moved_throttle(declared: bool, baseline: Option<f64>, now: Option<f64>) -> bool {
    if !declared {
        return true;
    }
    match (baseline, now) {
        (Some(b), Some(n)) => (n - b).abs() > HW_THROTTLE_HAND,
        _ => false,
    }
}

/// The acceleration segment's rate for a declared climb-out. Pure.
pub fn climb_accel_fpm(c: ClimbOut) -> f64 {
    (c.max_fpm * CLIMB_ACCEL_FRACTION).max(CLIMB_ACCEL_FLOOR_FPM)
}

/// Whether the climb is in its acceleration segment: a declared, finite
/// acceleration height passed and the flap schedule not yet finished. Pure.
pub fn accelerating(climb_out: Option<ClimbOut>, agl_ft: f64, flaps_retracted: bool) -> bool {
    climb_out.is_some_and(|c| c.accel_agl_ft.is_finite() && agl_ft >= c.accel_agl_ft)
        && !flaps_retracted
}

/// The autopilot's climb from the 300 ft engage to the handoff (#150).
///
/// The engage TAKES the climb the pitch was flying (`flown_fpm`, the flight
/// model's vertical speed) and the command moves from there toward the law
/// at [`CLIMB_VS_SLEW_FPM_S`], never a step. Below a declared acceleration
/// height ([`ClimbOut`]) the law is a speed-holding climb: the proportional
/// speed hold plus an integral trim, seeded at the engage so that the law
/// itself starts at the climb flown. A proportional hold alone settles where
/// the excess thrust is spent, 100 fpm a knot over the speed: 190 kt at the
/// 737's 3,900 fpm of excess, where the pitch had held 165.
#[derive(Debug, Clone, Copy, Default)]
pub struct ClimbVs {
    /// The last command; `None` until the first step (the engage).
    pub(super) cmd: Option<f64>,
    /// The speed hold's integral trim, fpm; `None` outside the hold.
    pub(super) trim: Option<f64>,
}

impl ClimbVs {
    /// One tick: `(law, command)`, fpm. `flown_fpm` is read only at the
    /// first step; a failed read (`None`) is no measurement, and the engage
    /// then writes the law as it always did. Pure.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        ias_kts: f64,
        target_kts: f64,
        base_fpm: f64,
        vne_kts: Option<f64>,
        agl_ft: f64,
        climb_out: Option<ClimbOut>,
        flown_fpm: Option<f64>,
        dt_s: f64,
        accelerating: bool,
    ) -> (f64, f64) {
        let flown = flown_fpm.filter(|v| v.is_finite()).map(|v| v.max(0.0));
        let p = climb_out_vs_fpm(ias_kts, target_kts, base_fpm, vne_kts, agl_ft, climb_out);
        let law = match climb_out {
            Some(c) if agl_ft < c.accel_agl_ft => {
                let hi = c
                    .max_fpm
                    .max(climb_vs_protected(ias_kts, target_kts, base_fpm, vne_kts));
                let lo = CLIMB_HOLD_FLOOR_FPM.min(hi);
                let trim = match (self.trim, self.cmd) {
                    (Some(t), Some(_)) => {
                        t + CLIMB_SPEED_TRIM_FPM_S_PER_KT
                            * (ias_kts - target_kts)
                            * dt_s.clamp(0.0, 1.0)
                    }
                    // The engage: the law starts where the pitch is.
                    (_, None) => flown.map_or(0.0, |f| f - p),
                    // Back under a height the climb had left: from zero.
                    (None, Some(_)) => 0.0,
                };
                let trim = trim.clamp(lo - p, hi - p);
                self.trim = Some(trim);
                p + trim
            }
            // From the acceleration height to clean: the nose lowered.
            Some(c) if accelerating => {
                self.trim = None;
                p.min(climb_accel_fpm(c))
            }
            _ => {
                self.trim = None;
                p
            }
        };
        let cmd = match self.cmd.or(flown) {
            Some(prev) => climb_vs_slewed(prev, law, dt_s),
            None => law,
        };
        self.cmd = Some(cmd);
        (law, cmd)
    }
}

/// Whether the handoff leaves the flaps where they are for the CALLER to
/// clean up on the schedule: an orchestrator that says it will
/// ([`Params::flaps_to_caller`], the circuit), an airframe with a schedule
/// to walk, and flaps still out. The one-shot clean-up at the circuit's 500
/// ft took the 737 from 1 to up one second after 5 to 1, at 191 kt against a
/// 210 kt flaps-up speed (LEMG 13, 2026-09-28). Pure.
pub fn leaves_flaps_to_caller(retracted: bool, flaps_to_caller: bool, has_schedule: bool) -> bool {
    !retracted && flaps_to_caller && has_schedule
}

/// What the takeoff hands its caller besides the aircraft: the climb it was
/// commanding and the flaps it left out (`None` when clean or cleaned up).
/// Set at the handoff, cleared at every start, taken once.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ClimbHandover {
    pub vs_cmd: Option<f64>,
    pub flaps_left: Option<f64>,
}

pub(super) static CLIMB_HANDOVER: std::sync::Mutex<Option<ClimbHandover>> = std::sync::Mutex::new(None);

pub(super) fn publish_climb_handover(h: Option<ClimbHandover>) {
    *CLIMB_HANDOVER.lock().unwrap_or_else(|e| e.into_inner()) = h;
}

/// The last completed takeoff's [`ClimbHandover`], once.
pub fn take_climb_handover() -> Option<ClimbHandover> {
    CLIMB_HANDOVER
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .take()
}

/// The speed the MCP window goes to once the takeoff accelerates: the flaps-up
/// manoeuvring speed a crew sets at the acceleration height (the schedule's
/// top retraction speed; the 737's 210), else the climb speed. Pure.
pub fn clean_up_speed_kts(schedule: &[(f64, f64)], vy_kts: f64) -> f64 {
    schedule
        .iter()
        .map(|(_, kts)| *kts)
        .filter(|k| k.is_finite() && *k > 0.0)
        .fold(None, |m: Option<f64>, k| Some(m.map_or(k, |m| m.max(k))))
        .unwrap_or(vy_kts)
}

/// The MCP speed window off V2 to `kts` (#150): written and read back
/// (within a knot). Only an airframe whose window the takeoff set to V2
/// (`takeoff_config: v2_kts`) is touched; the caller gates on that.
pub async fn speed_window_after_acceleration(xp: &dyn crate::xplane::XPlane, kts: f64) -> bool {
    use autoflight::Dial;
    autoflight::set_dial(xp, Dial::Airspeed, kts).await.is_ok()
        && autoflight::read_dial(xp, Dial::Airspeed)
            .await
            .is_ok_and(|v| (v - kts).abs() <= 1.0)
}

/// A roll leaving the centreline is rejected while it is still on the
/// pavement. LILB 03 (2026-09-25): the Evolution swung left at full power
/// with the steering pinned right, and the only reject in the roll was the
/// runway's END: it came at 68 kt, 129 m left of the centreline. Above
/// [`EXCURSION_MIN_KTS`], more than [`EXCURSION_XTK_M`] off the centreline
/// or [`EXCURSION_HDG_DEG`] off the runway's course is no takeoff. Pure.
pub fn lateral_excursion(xtk_m: f64, heading_off_deg: f64, gs_kts: f64) -> Option<&'static str> {
    if gs_kts < EXCURSION_MIN_KTS {
        return None;
    }
    if xtk_m.abs() > EXCURSION_XTK_M {
        Some("off_centreline")
    } else if heading_off_deg.abs() > EXCURSION_HDG_DEG {
        Some("off_heading")
    } else {
        None
    }
}
/// Below this the line-up's own straightening owns the error.
pub const EXCURSION_MIN_KTS: f64 = 10.0;
/// Half the narrowest runway anything here departs from (18 m), less a
/// wingtip's worth of margin.
pub const EXCURSION_XTK_M: f64 = 10.0;
pub const EXCURSION_HDG_DEG: f64 = 20.0;

/// Reject when the runway is running out and rotation speed is still short:
/// inside the margin with less than 90% of Vr there is no takeoff today.
/// Whether a GROUND reject (runway end, lateral excursion) may fire this
/// tick: never once rotated, and never with the wheels off the ground. The
/// light Zibo lifted itself off at 118 kt, before Vr and so before any
/// rotation; at 2 ft the excursion check rejected it, idled the thrust in the
/// air, and it zoomed to 394 ft, stalled and crashed (LEMG 30, 2026-10-07,
/// #231). A failed on-ground read (NaN) is no measurement of the ground, so
/// no reject on it. Pure.
pub fn ground_reject_allowed(rotated: bool, on_ground: f64) -> bool {
    !rotated && on_ground >= 0.5
}

pub fn must_reject(remaining_m: f64, ias_kts: f64, vr_kts: f64) -> bool {
    remaining_m < REJECT_REMAINING_M && ias_kts < vr_kts * 0.9
}
