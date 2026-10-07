//! The follower's vertical: the terrain look, the cleared ceiling, the pilot's
//! speed, the VS limits, the descent and its drag, `vertical_tick`.

use super::*;

/// How far ahead along the track the terrain probe looks, nm. Far enough that
/// a 700 fpm climb clears what it finds at light-aircraft speeds (15 nm is
/// over six minutes at 135 kt), short enough that the answer is about the leg
/// being flown and not the next county.
pub(super) const TERRAIN_LOOK_NM: f64 = 15.0;
/// Seconds between terrain probes. The ground does not move; this is a picture
/// of what is ahead, not a control input.
pub(super) const TERRAIN_PROBE_SECS: f64 = 10.0;
/// Points per probe. The plugin answers at most [`crate::terrain::MAX_POINTS`].
pub(super) const TERRAIN_POINTS: usize = 16;
/// Re-speak the terrain floor when it rises by more than this, feet.
pub(super) const TERRAIN_RESAY_FT: f64 = 200.0;

/// The altitude an IFR clearance holds the climb to until the departure
/// controller clears higher (`hold_at_cleared`): a CEILING on the plan's
/// climb, never a target of its own, set by the crew before the follower
/// starts and released at the departure check-in. A pilot's own altitude
/// outranks it; the ground ahead and an unmet SID minimum outrank both.
pub(super) static CLEARED_CEILING: std::sync::Mutex<Option<f64>> = std::sync::Mutex::new(None);

/// Hold the climb at the clearance's altitude (`Some`) or let it go on to
/// the plan (`None`). Blackboxed either way.
pub fn hold_at_cleared(ft: Option<f64>, why: &str) {
    let mut c = CLEARED_CEILING.lock().unwrap_or_else(|e| e.into_inner());
    if *c == ft {
        return;
    }
    *c = ft;
    crate::blackbox::record(
        "procedure",
        serde_json::json!({"event": "cleared_ceiling", "ft": ft, "why": why}),
    );
}

/// The clearance's ceiling on the climb, while one holds.
pub fn cleared_ceiling() -> Option<f64> {
    *CLEARED_CEILING.lock().unwrap_or_else(|e| e.into_inner())
}

/// The follower's altitude under a cleared ceiling: capped at it unless the
/// PILOT has taken the vertical, and never below the terrain floor the
/// vertical law already applied. Pure.
pub fn cap_at_clearance(
    want_ft: f64,
    ceiling_ft: Option<f64>,
    pilot_held: bool,
    floor_ft: Option<f64>,
) -> f64 {
    match ceiling_ft {
        Some(c) if !pilot_held && want_ft > c => c.max(floor_ft.unwrap_or(f64::MIN)),
        _ => want_ft,
    }
}

pub(super) fn clear_pilot_altitude() {
    crate::pilot_altitude::clear();
    *PILOT_SPEED.lock().unwrap_or_else(|e| e.into_inner()) = None;
}

/// A speed the PILOT asked for while the follower flies, the altitude's twin:
/// the follower owns the throttle (one writer), so "set speed 110" was refused
/// with "the approach has the throttle" in an enroute cruise the pilot could
/// see was slow, and there was no way to change it (LIPE-LIPK, 2026-09-25).
/// Flown as the plan's cruise speed, still floored, capped by the airframe's
/// ceiling, 250 below 10,000 and a STAR's limits; cleared with the altitude.
pub(super) static PILOT_SPEED: std::sync::Mutex<Option<f64>> = std::sync::Mutex::new(None);

/// Hand the follower a speed. `false` (nothing taken) when no procedure flies.
pub fn take_speed(kts: f64) -> bool {
    if !ACTIVE.load(Ordering::Relaxed) {
        return false;
    }
    *PILOT_SPEED.lock().unwrap_or_else(|e| e.into_inner()) = Some(kts);
    crate::blackbox::record("procedure", serde_json::json!({"event": "pilot_speed", "kts": kts}));
    true
}

/// The pilot's speed, if they have given one.
pub fn pilot_speed() -> Option<f64> {
    if !ACTIVE.load(Ordering::Relaxed) {
        return None;
    }
    *PILOT_SPEED.lock().unwrap_or_else(|e| e.into_inner())
}

/// The GA vertical rate, fpm, and the default for any airframe whose profile
/// declares nothing — the figure this law flew on every airframe until
/// 2026-09-20.
pub(super) const GA_VS_FPM: f64 = 700.0;

/// The enroute climb and descent rates of an airframe, the GA figure where
/// the profile is silent, so the last aircraft's rates can never be pointed
/// at the next one. The LIVE airframe's are the snapshot's `vs_limits`,
/// republished on every profile swap by
/// [`crate::flight_state::publish_airframe`]. Pure.
pub fn vs_limits_for(r: Option<&crate::aircraft::ReferenceProfile>) -> crate::aircraft::VsLimits {
    let pick = |v: Option<f64>| v.filter(|f| f.is_finite() && *f > 0.0).unwrap_or(GA_VS_FPM);
    crate::aircraft::VsLimits {
        climb_fpm: pick(r.and_then(|r| r.enroute_climb_fpm)),
        descent_fpm: pick(r.and_then(|r| r.enroute_descent_fpm)),
        best_climb_fpm: r
            .and_then(|r| r.best_climb_fpm)
            .filter(|f| f.is_finite() && *f > 0.0),
    }
}

/// Publish the airframe's enroute vertical rates alone.
#[cfg(test)]
pub fn publish_vs_limits(r: Option<&crate::aircraft::ReferenceProfile>) {
    let l = vs_limits_for(r);
    crate::aircraft::update_airframe_facts(|a| a.vs_limits = Some(l));
}

/// The live airframe's vertical rates, the GA figure before any airframe.
pub(super) fn vs_limits() -> crate::aircraft::VsLimits {
    crate::aircraft::airframe_facts()
        .vs_limits
        .unwrap_or_else(|| vs_limits_for(None))
}

/// The rate a climb to a departure minimum is flown at: the airframe's best
/// climb where it declares one, never less than the enroute rate.
pub fn best_climb_fpm() -> f64 {
    vs_limits().best_climb_fpm.unwrap_or(0.0).max(climb_fpm())
}

/// The live airframe's enroute climb rate, fpm.
pub fn climb_fpm() -> f64 {
    vs_limits().climb_fpm
}

/// The live airframe's enroute descent rate, fpm (magnitude). This both caps
/// the commanded rate and places the top of descent, so the two can never
/// disagree: the TOD is put exactly where this rate makes the constraint.
pub fn descent_fpm() -> f64 {
    vs_limits().descent_fpm
}

/// The rate the follower's vertical law will actually fly, fpm (magnitude):
/// what a readback must promise when the follower owns the altitude, whatever
/// rate the pilot asked for.
pub fn vs_clamp_fpm() -> f64 {
    descent_fpm()
}
/// Start down this far before the computed top of descent, nm.
pub(super) const TOD_LEAD_NM: f64 = 1.0;

/// The altitude to command NOW for a leg that wants `leg_alt` at a fix
/// `dist_nm` away: the previous target (`carry`, or where we are if we never
/// reached it) until the top of descent, then the leg's own. Dialling a
/// constraint straight in began the descent the moment the leg armed — the
/// Gibraltar arrival dialled 1,397 ft with 25 nm to run and flew 18 nm level
/// at 1,400 ft across the Strait, past Europa Point (2026-09-11). Climbs are
/// untouched (the climb law owns them), so is a small step, and a leg with no
/// fix has nothing to pace against.
#[cfg(test)]
pub fn descent_target_ft(
    leg_alt: f64,
    carry: f64,
    alt_ft: f64,
    dist_nm: Option<f64>,
    gs_kts: f64,
    descent_fpm: f64,
) -> f64 {
    descent_target(leg_alt, carry, alt_ft, dist_nm, gs_kts, descent_fpm).0
}

/// [`descent_target_ft`] and the target's own rate, fpm: the path's planned
/// descent while the target is moving down it, zero when it is a level. The
/// VS law feeds it forward ([`vs_law_fpm`]): a proportional law alone trails
/// a ramp by rate over gain, 500 ft at the 737's 1,500 fpm path, and the
/// path ends at the fix.
pub fn descent_target(
    leg_alt: f64,
    carry: f64,
    alt_ft: f64,
    dist_nm: Option<f64>,
    gs_kts: f64,
    descent_fpm: f64,
) -> (f64, f64) {
    let Some(dist_nm) = dist_nm else {
        return (leg_alt, 0.0);
    };
    if descent_point_nm(leg_alt, alt_ft, gs_kts, descent_fpm).is_none() {
        return (leg_alt, 0.0);
    }
    let held = if carry <= alt_ft + 100.0 {
        carry
    } else {
        (alt_ft / 100.0).round() * 100.0
    };
    // Down a PATH at the planning rate, not a dive at the maximum: the
    // constraint plus the distance still to run times the planned gradient,
    // never above the level held, never below the constraint. Stepping the
    // target straight to the constraint at the top of descent made the VS
    // law ask the airframe's full rate at once, and the 737 went down its
    // LEMG arrival in 2,000 fpm dives with the speedbrake out between level
    // segments (2026-09-26). A late start still catches the path at the
    // full rate; the path itself is the gentler one.
    let ft_per_nm = descent_fpm * DESCENT_PLAN_FRACTION / (gs_kts.max(60.0) / 60.0);
    let path = leg_alt + (dist_nm - TOD_LEAD_NM).max(0.0) * ft_per_nm;
    let want = path.min(held).max(leg_alt);
    // Moving only while the PATH is what is commanded (not the level held
    // before the top of descent, not the constraint after the lead).
    let rate = if path < held && path > leg_alt {
        -descent_fpm * DESCENT_PLAN_FRACTION
    } else {
        0.0
    };
    (want, rate)
}

/// The follower's vertical speed law: the target's own rate fed forward,
/// plus three fpm a foot of error, inside the airframe's rates.
pub fn vs_law_fpm(want_alt: f64, alt_ft: f64, target_fpm: f64, descent_fpm: f64, climb_cap: f64) -> f64 {
    (target_fpm + (want_alt - alt_ft) * VS_GAIN_PER_FT).clamp(-descent_fpm, climb_cap)
}

/// Fpm of commanded vertical speed per foot off the target.
pub(super) const VS_GAIN_PER_FT: f64 = 3.0;

/// The share of the airframe's descent rate a descent is PLANNED at: the
/// rest is margin to catch up with, as a crew plans 3 miles per thousand
/// feet and keeps the idle-and-speedbrake rate in hand.
pub const DESCENT_PLAN_FRACTION: f64 = 0.75;

/// Where the descent to `leg_alt` begins: miles BEFORE the fix, at the rate
/// the VS law flies and the groundspeed we have, plus the lead. `None` when
/// there is no descent to pace (the constraint is not more than 100 ft below
/// us). The one formula behind [`descent_target_ft`] and [`planned_tod`].
pub fn descent_point_nm(leg_alt: f64, alt_ft: f64, gs_kts: f64, descent_fpm: f64) -> Option<f64> {
    let drop_ft = alt_ft - leg_alt;
    if drop_ft <= 100.0 {
        return None;
    }
    Some(drop_ft / (descent_fpm * DESCENT_PLAN_FRACTION).max(1.0) * (gs_kts.max(60.0) / 60.0) + TOD_LEAD_NM)
}

/// One tick of the follower's vertical bookkeeping, pure so every rule below
/// is a plain `cargo test` and not a sortie.
///
/// `carry` is the last leg CONSTRAINT (carried across unconstrained fixes);
/// `prev_cmd` is what was actually COMMANDED last tick, which is what the
/// top-of-descent deferral paces against. Conflating the two defeated the
/// deferral after one tick per leg (2026-09-13, see [`descent_target_ft`]).
#[derive(Debug, Clone, PartialEq)]
pub struct Vertical {
    /// The altitude to command now.
    pub want_alt: f64,
    /// `carry` for the next tick.
    pub carry: f64,
    /// `prev_cmd` for the next tick.
    pub prev_cmd: f64,
    /// The terrain floor, when it is what set `want_alt`.
    pub floored: Option<f64>,
    /// True when the pilot's altitude is what set `want_alt`.
    pub pilot: bool,
    /// `want_alt`'s own rate, fpm (negative down a descent path, zero for a
    /// level), for the VS law's feed-forward.
    pub target_fpm: f64,
    /// [`DescentProfile::peak_ft`] for the next tick.
    pub peak_ft: f64,
    /// A plan altitude above the level already commanded in the descent,
    /// held rather than climbed to: `(plan_ft, held_ft)`.
    pub descent_held: Option<(f64, f64)>,
    /// A plan altitude below the level already commanded in the CLIMB,
    /// held rather than descended to: `(plan_ft, held_ft)`.
    pub climb_held: Option<(f64, f64)>,
}

/// Where the flight is on the plan's profile, for [`vertical_tick`]: once it
/// has begun its descent, a later plan altitude ABOVE the level already
/// commanded is a navlog prediction, not a step climb.
///
/// LIEA-LEBL, 2026-09-27 (sortie 1790494983-full_auto): the 737 began its
/// descent from 31,000 at 08:23:46, levelled at MUREN's 28,000, then armed
/// CHELY at 29,000 and CLIMBED to 28,999 before descending again for VERSO.
/// SimBrief's navlog altitudes down a descent are where it predicts the
/// aircraft to be, and a crew that has started down never climbs back.
///
/// And the mirror: until the cruise is REACHED, a plan altitude below the
/// level already commanded is not a descent. LFBD-EGKK, 2026-09-27 (sortie
/// 1790510400-full_auto): climbing through 7,638 ft for FL340, the CNA6R's
/// "course 296, at or above 660 ft" leg dialled 660, the terrain floor raised
/// it to 705 ("holding 800 feet") and, the cruise having been COMMANDED from
/// the first leg, the old rule called that a descent from 34,000 and held
/// BD923's 34,000 at 705: 14 minutes at 1,200 ft until the pilot dialled it.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct DescentProfile {
    /// The plan's cruise: its highest leg altitude. `None` switches the rule
    /// off (an approach's own procedure, a plan without altitudes).
    pub cruise_ft: Option<f64>,
    /// The highest altitude FLOWN this flight, carried tick to tick. Flown,
    /// not commanded: FL340 is commanded from the runway's first leg.
    pub peak_ft: f64,
    /// The leg is past the plan's last leg at its cruise altitude: whatever
    /// follows is the descent, which covers a follower armed mid-descent.
    pub past_top: bool,
    /// The arrival's ceilings lowered this leg (`ceilinged_leg_alt`): the
    /// plan's own descent begun before the cruise was reached, not a dip.
    pub arrival_ceiling: bool,
}

impl DescentProfile {
    /// The rule off.
    #[cfg(test)]
    pub fn none() -> Self {
        Self::default()
    }

    /// Whether a flight that last commanded `prev_cmd` has begun its descent
    /// on the plan's profile: it has flown the cruise and now commands below
    /// it, or it is past the plan's top and below the cruise. A step climb
    /// BEFORE the top of descent is neither: the cruise is not reached yet.
    pub fn descending(&self, prev_cmd: f64) -> bool {
        let Some(cruise) = self.cruise_ft else {
            return false;
        };
        (self.past_top || self.cruise_reached(cruise)) && prev_cmd < cruise - STEP_FT
    }

    /// Whether the flight is still climbing to the plan's cruise: never
    /// flown it, not past the plan's top, no arrival ceiling in force.
    pub fn climbing(&self) -> bool {
        self.cruise_ft
            .is_some_and(|c| !self.past_top && !self.arrival_ceiling && !self.cruise_reached(c))
    }

    pub(super) fn cruise_reached(&self, cruise: f64) -> bool {
        self.peak_ft >= cruise - STEP_FT
    }
}

/// Feet a level must differ by to count as a step (the same 100 ft the
/// descent pacing treats as no descent at all).
pub(super) const STEP_FT: f64 = 100.0;

/// The index of the plan's last leg at its cruise altitude (within a step):
/// every leg after it is the descent.
pub fn plan_top_index(legs: &[ProcLeg], cruise_ft: f64) -> Option<usize> {
    legs.iter()
        .rposition(|l| l.alt1_ft.is_some_and(|a| a >= cruise_ft - STEP_FT))
}

/// Order of precedence, each one a rule learned the hard way: the plan's
/// constraint paced to its top of descent, then the PILOT's altitude if they
/// have taken it, then the ground AHEAD, which outranks everyone: no target
/// the follower commands may be into terrain, whoever asked for it.
/// The terrain floor is TRUE feet (probed ground plus clearance); the dial is
/// indicated. The live true-minus-indicated offset (temperature and QNH error
/// together) converts it, so a hot day never eats into the clearance.
pub fn floor_indicated(floor_true_ft: f64, true_alt_ft: f64, indicated_alt_ft: f64) -> f64 {
    floor_true_ft - (true_alt_ft - indicated_alt_ft)
}

/// The flight detent: half. Full is a ground spoiler setting on most jets.
pub(super) const SPEEDBRAKE_FLIGHT: f64 = 0.5;
/// The follower put the speedbrake out, so the follower puts it away. A
/// lever the pilot pulled is the pilot's.
pub(super) static DRAG_OURS: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
/// Over the governor's target by this much in a descent: brakes out...
pub(super) const DRAG_OUT_KTS: f64 = 8.0;
/// ...and back in this near it (hysteresis: a comparator on airspeed cycles).
pub(super) const DRAG_IN_KTS: f64 = 3.0;
/// A commanded descent gentler than this is not one the brakes are for.
pub(super) const DRAG_MIN_DESCENT_FPM: f64 = 500.0;
/// A brake already out stays out through a level-off or a shallowing path
/// (the speed is still owed); only a commanded CLIMB past this puts it in.
/// The descent gate alone put it in 5 s after it went out at 216 kt when the
/// VS law eased to +131 fpm on a level-off (EGKK-EGBB +2739, 2026-09-27).
pub(super) const DRAG_KEEP_BELOW_FPM: f64 = 300.0;
/// Seconds the brake keeps a state before the speed rule may change it
/// (the configuration bar and a lost target still act at once).
pub(super) const DRAG_MIN_HOLD_S: f64 = 15.0;
/// Seconds between spoken speedbrake calls: a change sooner than this is
/// flown and blackboxed, not said.
pub(super) const DRAG_SPEAK_GAP_S: f64 = 120.0;
/// Feet above the level commanded now at which a descent held back for its
/// speed ([`speed_protected_vs`]) gets the speedbrake: the height is owed and
/// idle alone cannot lose it at the speed allowed...
pub(super) const DRAG_HIGH_FT: f64 = 400.0;
/// ...and back within this of it, the brake goes in.
pub(super) const DRAG_ON_PATH_FT: f64 = 150.0;
/// Slower than the target by this, a high descent needs no brake: the
/// protection lets the whole rate through.
pub(super) const DRAG_SLOW_KTS: f64 = 4.0;

/// The airspeed trend's smoothing for the speed protection, seconds.
pub(super) const IAS_TREND_TAU_S: f64 = 2.0;

/// A descent is flown at IDLE with the speed on the pitch, as a crew flies
/// one (level change, VNAV SPD): while the vertical law wants down, the
/// governor's target sits [`IDLE_DESCENT_MARGIN_KTS`] under the speed owed,
/// so its lever stays closed at the speed, and [`speed_protected_vs`] holds
/// the speed owed by giving way. Power comes back only past the margin.
///
/// EGKK-EGBB (2026-09-28, sortie 1790601822-full_auto): OSKOT's 210 was made
/// level, the governor overshot to 204 and was adding power (lever 0.3) as
/// the descent to GROVE began: 216 at OSKOT, 218 at GROVE, with the
/// speedbrake out and in three times in the join after. With the governor
/// at the speed owed, every start of a descent was a lever still coming back.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct IdleDescent {
    pub(super) on: bool,
}

/// The vertical law wanting this much descent starts an idle descent...
pub(super) const IDLE_DESCENT_ON_FPM: f64 = -800.0;
/// ...and wanting less than this ends it (a level-off's last feet).
pub(super) const IDLE_DESCENT_OFF_FPM: f64 = -300.0;
/// Knots under the speed owed the governor holds in an idle descent.
pub const IDLE_DESCENT_MARGIN_KTS: f64 = 5.0;

impl IdleDescent {
    /// Update from the descent the vertical law wants, fpm; whether the
    /// descent is flown at idle.
    pub fn step(&mut self, vs_want_fpm: f64) -> bool {
        if self.on {
            if vs_want_fpm >= IDLE_DESCENT_OFF_FPM {
                self.on = false;
            }
        } else if vs_want_fpm <= IDLE_DESCENT_ON_FPM {
            self.on = true;
        }
        self.on
    }

    pub fn on(&self) -> bool {
        self.on
    }

    /// The governor's target for the speed owed `kts`, never under `floor`.
    pub fn governor_kts(&self, kts: f64, floor: Option<f64>) -> f64 {
        if self.on {
            crate::aircraft::floored_target(kts - IDLE_DESCENT_MARGIN_KTS, floor).min(kts)
        } else {
            kts
        }
    }
}
/// Knots over the governor's target, projected [`SPEED_PROTECT_LEAD_S`]
/// ahead on the airspeed trend, at which a descent starts giving way...
pub(super) const SPEED_PROTECT_FROM_KTS: f64 = 0.0;
/// ...and this many knots more flies it level, where a jet at idle loses
/// speed (the 737 about 0.9 kt/s at 8,000 ft, fitted on sortie
/// 1790601822-full_auto).
pub(super) const SPEED_PROTECT_SPAN_KTS: f64 = 6.0;
/// The airspeed trend's lead, seconds.
pub(super) const SPEED_PROTECT_LEAD_S: f64 = 3.0;

/// The descent a speed over its target may keep, fpm: the commanded descent
/// given way in proportion to the projected excess, level at
/// [`SPEED_PROTECT_FROM_KTS`] + [`SPEED_PROTECT_SPAN_KTS`] over. A crew
/// descending at idle on a speed limit flies the SPEED and lets the path
/// go (the speed is owed to the fix and the approach; the height can be
/// lost with the speedbrake), where the VS law alone flew the path and let
/// the speed go. Climbs, level flight and an unknown target pass through.
///
/// EGKK-EGBB 737 (2026-09-28, sortie 1790601822-full_auto, #156, #32):
/// 2,000 fpm at idle down the HEME1B, PUFAX crossed at 254 against 250,
/// OSKOT 216 and GROVE 218 against 210, and six speedbrake extensions
/// each started at 8 kt over. The fitted plant descends at constant speed
/// at idle at about 2.9 degrees; a 2,000 fpm catch-up is four and gains
/// 0.3 kt/s. Pure.
pub fn speed_protected_vs(vs_cmd: f64, over_kts: Option<f64>, trend_kts_s: f64) -> f64 {
    let Some(over) = over_kts.filter(|o| o.is_finite()) else {
        return vs_cmd;
    };
    if vs_cmd >= 0.0 {
        return vs_cmd;
    }
    let proj = over + trend_kts_s.clamp(-3.0, 3.0) * SPEED_PROTECT_LEAD_S;
    let keep = 1.0 - ((proj - SPEED_PROTECT_FROM_KTS) / SPEED_PROTECT_SPAN_KTS).clamp(0.0, 1.0);
    vs_cmd * keep
}

/// What a descent that is running away with the speed gets: whether the
/// speedbrake should be out, and the fraction of the commanded rate to fly.
///
/// EGPH-EGTE, 2026-09-21: 2,000 fpm down with the governor's lever already at
/// idle and nothing else to slow with. 258 kt through ten thousand against a
/// 250 target, 267 at six, 278 at the last fix, where the approach armed. A
/// governor with no drag device can only watch a descent accelerate. A crew
/// pulls the speedbrake, and if that is not enough it shallows the descent:
/// the speed is owed to the approach that starts at the bottom, the height
/// can be lost later. Pure; `out` is the brake's state going in, `held_s`
/// how long it has had it (`f64::INFINITY` when it never changed).
///
/// EGKK-EGBB, 2026-09-27 (sortie 1790541304-full_auto, #126): out and in
/// around the approach's 210.25 at +2621/+2691, +2735/+2739 and +2962/+3004,
/// every change spoken. Two of the three "in"s were the descent gate, not
/// the speed: the VS law easing through -500 fpm at a level-off with the
/// aircraft still 6 and 13 kt fast. Out, the brake now stays out through a
/// level-off until the speed is in or a climb is commanded, and neither
/// state is left inside [`DRAG_MIN_HOLD_S`].
///
/// EGKK-EGBB, 2026-09-28 (sortie 1790601822-full_auto, #156, #126): six
/// extensions down the HEME1B and three more in the join, every one at
/// 8 kt over. The shallower descent is now [`speed_protected_vs`]'s, flown
/// before the speed runs away, so the brake is for what a crew uses it
/// for: HEIGHT the protected descent cannot lose. Out when the aircraft is
/// [`DRAG_HIGH_FT`] above the level commanded (`high_ft`, the target's
/// shortfall) with the speed not slow, or a runaway past
/// [`DRAG_OUT_KTS`]; in once back within [`DRAG_ON_PATH_FT`] with the speed
/// in, or [`DRAG_SLOW_KTS`] slow, or a climb. `vs_want_fpm` is the descent
/// the vertical law WANTS, before the speed protection held it back. Pure.
pub fn descent_drag(
    out: bool,
    held_s: f64,
    ias_kts: f64,
    target_kts: Option<f64>,
    vs_want_fpm: f64,
    high_ft: f64,
) -> bool {
    let Some(target) = target_kts else {
        return false;
    };
    let over = ias_kts - target;
    let settled = held_s >= DRAG_MIN_HOLD_S;
    if out {
        let done = over < -DRAG_SLOW_KTS || (over <= DRAG_IN_KTS && high_ft <= DRAG_ON_PATH_FT);
        vs_want_fpm < DRAG_KEEP_BELOW_FPM && (!done || !settled)
    } else {
        let high = high_ft > DRAG_HIGH_FT && over > -DRAG_SLOW_KTS / 2.0;
        vs_want_fpm <= -DRAG_MIN_DESCENT_FPM && (over > DRAG_OUT_KTS || high) && settled
    }
}

/// Minutes of the commanded rate inside which a VS mode that drops heading
/// for the dial is the autopilot's own altitude capture.
pub(super) const CAPTURE_LEAD_MIN: f64 = 0.2;

/// Whether a VS mode found OFF is the autopilot capturing, to be left alone:
/// near the follower's target, or inside the capture band of the DIAL (the
/// level this movement ends at) while heading for it. EGKK-EGBB (2026-09-27,
/// sortie 1790541304-full_auto +2295 and +3001): the 737 began capturing
/// HON's 9,000 202 ft above it at 1,960 fpm down and FI33's 1,800 246 ft
/// above it at 1,500; the path target was 134 and 148 ft below, so the old
/// 100 ft test called it a dropped mode, re-engaged VS, and the dial write
/// read back 0 (`write_unconfirmed vertical_velocity want -1900 actual 0`),
/// one tick lost and the capture undone. Pure.
pub fn vs_mode_capturing(alt_ft: f64, want_alt: f64, dial_ft: f64, vs_cmd_fpm: f64) -> bool {
    if (want_alt - alt_ft).abs() <= 100.0 {
        return true;
    }
    let band = (vs_cmd_fpm.abs() * CAPTURE_LEAD_MIN).max(100.0);
    (dial_ft - alt_ft).abs() <= band && (dial_ft - alt_ft) * vs_cmd_fpm >= 0.0
}

/// Which speedbrake changes are SAID: an extension when nothing about the
/// brake was said for [`DRAG_SPEAK_GAP_S`], and the stow of an extension
/// that was said (never an orphan "Speedbrake in."). Every change is still
/// flown and blackboxed. EGKK-EGBB (2026-09-27, #126) said "Speedbrake." /
/// "Speedbrake in." thirteen times on one arrival, seven of them in 46 s.
/// Pure: `t` is seconds on any clock.
#[derive(Debug, Default, Clone)]
pub struct DragVoice {
    pub(super) last_said: Option<f64>,
    pub(super) out_said: bool,
}

impl DragVoice {
    /// Whether the change to `out` at `t` is spoken.
    pub fn say(&mut self, out: bool, t: f64) -> bool {
        let due = if out {
            let due = self.last_said.is_none_or(|l| t - l >= DRAG_SPEAK_GAP_S);
            self.out_said = due;
            due
        } else {
            std::mem::take(&mut self.out_said)
        };
        if due {
            self.last_said = Some(t);
        }
        due
    }
}

/// Why the speedbrake may not be out in this configuration, if it may not:
/// `"gear"` with the gear handle down, `"flaps"` with the flap handle past the
/// airframe's first detent (`flap_limits`, retracted first; any flap at all
/// when none is declared). A crew rule, not a speed rule: the drag is for a
/// clean descent, and the approach slows on flap and gear.
///
/// EGKK 26L, 2026-09-27 (sortie 1790510400-full_auto, stock 737): the
/// follower put the speedbrake out at 218 kt against a 210 target and
/// 2,000 fpm down, then configured for the final with it still out: gear
/// down and flaps 10, 25, 30, 40 before the speed let it in, and out again
/// for five seconds at flaps 40. The SPEEDBRAKE EXTENDED caution kept
/// tripping the master caution through the approach. `descent_drag`'s
/// shallower rate is what the descent keeps.
pub fn drag_barred(
    flap_handle: f64,
    gear_handle_down: bool,
    flap_limits: &[(f64, Option<f64>)],
) -> Option<&'static str> {
    if gear_handle_down {
        return Some("gear");
    }
    let first_detent = flap_limits
        .iter()
        .map(|(r, _)| *r)
        .filter(|r| r.is_finite() && *r > 0.01)
        .min_by(f64::total_cmp)
        .unwrap_or(0.0);
    if flap_handle > first_detent + 0.01 {
        return Some("flaps");
    }
    None
}

pub fn vertical_tick(
    leg: &ProcLeg,
    carry: f64,
    prev_cmd: f64,
    alt_ft: f64,
    dist_nm: Option<f64>,
    gs_kts: f64,
    pilot: Option<f64>,
    terrain_floor: Option<f64>,
    descent_fpm: f64,
    profile: DescentProfile,
) -> Vertical {
    let mut leg_alt = commanded_alt_ft(leg, carry);
    // In the descent a plan altitude above the level already commanded is
    // held at that level until a lower one takes over. The pilot's altitude
    // still wins (a controller may ask for a climb) and so does the ground:
    // both are applied after this, over it.
    let mut descent_held = None;
    if pilot.is_none() && profile.descending(prev_cmd) && leg_alt > prev_cmd + 50.0 {
        descent_held = Some((leg_alt, prev_cmd));
        leg_alt = prev_cmd;
    }
    // In the climb a plan altitude below the level already commanded (a
    // CA leg's "at or above", a navlog prediction) is held at that level:
    // a crew climbing to its cruise never descends for a minimum it passed.
    let mut climb_held = None;
    if pilot.is_none() && profile.climbing() && leg_alt < prev_cmd - 50.0 {
        climb_held = Some((leg_alt, prev_cmd));
        leg_alt = prev_cmd;
    }
    let (mut want_alt, mut target_fpm) =
        descent_target(leg_alt, prev_cmd, alt_ft, dist_nm, gs_kts, descent_fpm);
    // The cruise is reached by FLYING it: commanded from the first leg, it
    // made a climb through 7,600 ft look like a descent (LFBD 2026-09-27).
    let peak_ft = profile.peak_ft.max(alt_ft);
    let from_pilot = pilot.is_some();
    if let Some(p) = pilot {
        want_alt = p;
        target_fpm = 0.0;
    }
    let mut floored = None;
    if let Some(floor) = terrain_floor {
        if want_alt < floor {
            want_alt = floor;
            floored = Some(floor);
            target_fpm = 0.0;
        }
    }
    Vertical {
        want_alt,
        carry: leg_alt,
        prev_cmd: want_alt,
        floored,
        pilot: from_pilot && floored.is_none(),
        target_fpm,
        peak_ft,
        descent_held,
        climb_held,
    }
}

/// The leg to hand off to autoland on an approach: the MAP of an approach
/// that ends short of the runway, else the FAF if the data flags one,
/// else the last fix leg before the runway threshold, else the last leg. `None`
/// only when there are no legs. Handing off AT the FAF lets autoland own the
/// glideslope from platform to touchdown, exactly where it's proven.
pub fn handoff_index(legs: &[ProcLeg]) -> Option<usize> {
    if legs.is_empty() {
        return None;
    }
    // An approach to a MAP short of the runway is flown TO the MAP: its final
    // is the published valley track, and the runway's extended centreline
    // from the FAF lies across the slopes beside it (LSZS 03: two miles off,
    // toward Piz Corvatsch). A MAP coded ON the runway fix is not one: the
    // ILS finals in the CIFP carry the MAP flag on the threshold ("G  M"),
    // and handing off there gave autoland EGKK 26L at 1 nm, 244 m off the
    // centreline, and a go-around at 97 ft (2026-09-27).
    if legs.last().is_some_and(map_short_of_runway) {
        return Some(legs.len() - 1);
    }
    if let Some(i) = legs.iter().position(|l| l.is_faf) {
        return Some(i);
    }
    // The leg just before the first runway-threshold ("RW..") leg.
    if let Some(rw) = legs.iter().position(|l| l.fix_ident.starts_with("RW")) {
        return Some(rw.saturating_sub(1));
    }
    Some(legs.len() - 1)
}
