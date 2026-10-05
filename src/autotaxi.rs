//! Auto-taxi — the copilot steers the cleared taxi route to the holding point.
//!
//! Composes the two halves that already exist: the taxi-speed governor
//! ([`crate::tools::speedhold`], throttle + brakes) and the tower's published
//! route ([`crate::party_line::with_taxi_route`], the same polyline the
//! progressive taxi guide narrates). This module adds the missing piece — a
//! ground steering law — and a speed schedule that slows for bends and ramps
//! to a stop short of the holding point. The guide keeps talking over it, so
//! the pilot hears the same "next left onto Whiskey" narration whether they
//! or the copilot are steering.
//!
//! Steering goes through the plugin's `xcopilot/taxi_steer` channel (the
//! proportional-brake pattern: per-frame writes under
//! `override_joystick_heading`, expiring within a second of writes stopping),
//! because REST-rate writes to `yoke_heading_ratio` lose the re-drive fight
//! with the pilot's hardware exactly like the brake ratios did.
//!
//! Handback, every path instant:
//! - The pilot's FEET win in-plugin: real yaw-axis deflection latches
//!   `xcopilot/steer_yield` and drops the override that frame — this loop
//!   polls the latch and stands down, but the pedals were live already.
//! - "My controls" (voice or panel button) releases steering and throttle
//!   together via the main loop's release path; "my steering" releases just
//!   the steering, leaving the speed hold — the pre-existing "you steer, I
//!   have the throttle" division.
//! - The governor standing down for ANY reason (park handle, throttle push,
//!   airborne, release words) takes the steering with it: speed authority
//!   gone means nobody is stopping the aircraft, so steering alone must not
//!   keep driving it.
//! - An engage that starts off the route or facing against it is a JOIN, not
//!   a fault: the law turns toward the route at bend speed and the off-route
//!   guard is widened to the engage distance plus [`JOIN_GROW_M`] until the
//!   aircraft is captured (close and aligned) — then the tight guard resumes.
//!   Once ESTABLISHED, drifting past [`OFF_ROUTE_STOP_M`] still STOPS the
//!   aircraft and releases steering: a law that held meters for a while and
//!   then walked off means something is broken, and a driven "recovery" on
//!   pavement it was never near is nobody's clearance.
//!
//! Everything here is pure geometry over the published polyline; the laws
//! battle-test as plain `cargo test`, no sim.

use std::sync::atomic::{AtomicBool, Ordering};

use crate::station::TaxiWatch;
use crate::taxi_guide;
use crate::tools::speedhold;
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

pub(crate) const LAT: &str = "sim/flightmodel/position/latitude";
pub(crate) const LON: &str = "sim/flightmodel/position/longitude";
/// TRUE heading, matching the route bearings computed in the local
/// east/north frame — never the magnetic compass.
pub(crate) const PSI_TRUE: &str = "sim/flightmodel/position/psi";
pub(crate) const GS_MS: &str = "sim/flightmodel/position/groundspeed";
/// The thrust lever THIS airframe obeys — not always the stock one (the
/// Citation X tracks `throttle_jet_rev_ratio`, so every write to
/// `throttle_ratio_all` went nowhere and it flew a whole pattern at takeoff
/// thrust). Resolved per call, so a profile swap takes effect immediately.
fn throttle_dref() -> String {
    crate::aircraft::throttle_dataref()
}
/// The plugin's proportional steering channel (-1..1, right positive).
const TAXI_STEER: &str = "xcopilot/taxi_steer";
/// The sim's own toe-brake inputs, for the differential-brake steering assist
/// ([`brake_steer_split`]). Written one side at a time and zeroed on every
/// stand-down; pilot toe input on the SAME datarefs wins by the pedal/steer
/// yield machinery, not by arbitration here.
/// The differential-brake steering channels. NOT the stock ratios: XP12
/// re-drives `cockpit2/controls/*_brake_ratio` from the pilot's assigned
/// toe-brake axes every frame, so a REST write is gone before the next read
/// (probed live 2026-07-30 — a PATCH of 0.25 read back 0.0000 within 400 ms,
/// and held 0.2500 under `override_toe_brakes`). Only the plugin can hold that
/// override at frame rate, and it arbitrates these against the governor's
/// stopping brake per wheel. The channels expire a second after writes stop,
/// so a dead app frees the pedals by silence.
pub(crate) const LEFT_BRAKE: &str = "xcopilot/taxi_brake_l";
pub(crate) const RIGHT_BRAKE: &str = "xcopilot/taxi_brake_r";
/// The drive's and the line-up alignment's tick period, ms
/// (`sim::state::Pacer`): the old 150 ms sleep plus the MacBook's three 20
/// ms rounds (no MacBook record of the drive's dt; the iMac's drive ticked
/// about 0.3 s, 2026-09-30).
const DRIVE_TICK_MS: u64 = 210;
/// The drive's yaw-rate filter, seconds: half the raw rate a tick at the
/// MacBook's 0.21 s tick is a time constant of 0.21 / ln 2.
const YAW_RATE_TAU_S: f64 = 0.303;
/// What the drive's tick reads past the position (`sim::state`).
const DRIVE_WANT: &[sim::state::F] = &[sim::state::F::SteerYield, sim::state::F::Tug];
/// ...and a free castoring nose the body yaw rate too (`position/R`, its
/// ground law's measurement, `ground_steer`).
const DRIVE_WANT_FREE: &[sim::state::F] = &[sim::state::F::SteerYield, sim::state::F::Tug, sim::state::F::R];
/// What the line-up alignment's tick reads past the position: the pedals
/// (`xcopilot/steer_yield`, 1 = the yaw axis moved, override dropped) and
/// the body yaw rate (`position/R`, nose right positive, the aligning
/// pivot's derivative, [`align_pivot_brakes`]).
const ALIGN_WANT: &[sim::state::F] = &[sim::state::F::SteerYield, sim::state::F::R];
/// The tug on the bar, blackboxed per tick as EVIDENCE only: the taxi that
/// followed an early tug release could not turn right for fifteen seconds
/// and the record could not say whether a re-summoned tug had hooked back
/// on (LEMG, 2026-09-12). A slow dataref, so the read is nearly free.
#[cfg(test)]
pub(crate) const TUG_ATTACHED: &str = "sim/aircraft/overflow/pushback_attached";
/// A tug on the bar owns the aircraft: the drive holds (target zero, no
/// steer heartbeat, the stagnation clock parked) until the bar comes off,
/// and stands down after this long if it never does. The taxi that engaged
/// with the tug still turning the nose at LXGB "drove" 34 s at half a knot
/// and stood the mission down as stagnant (08:15Z, 2026-09-15).
const TUG_WAIT_SECS: f64 = 180.0;
const MS_TO_KTS: f64 = 1.943_844;
const M_PER_NM: f64 = 1852.0;

/// Stop this many meters short of the route's end. The planner already ends
/// the route at the hold-short standoff (40 m back from a runway-junction
/// tail — nav.rs `plan`), so this is only the nose-ahead-of-CG margin, not
/// the hold line's position. It was 12 m back when the route ended ON the
/// runway centreline, and the aircraft still stopped past the hold line
/// (2026-07-20): the standoff belongs to the planner, not this margin.
const STOP_SHORT_M: f64 = 5.0;
/// Inside this many meters of the stop, steer to the route's FINAL BEARING
/// rather than to a pursuit aim point. Chosen from the live trace: the
/// pursuit error was still honest at 8 m to run (-8°) and had begun walking
/// by 4 m (+1°), so the swap happens before it degrades rather than after.
const STOP_HEADING_HOLD_M: f64 = 8.0;
/// Lateral drift at which the loop stops the aircraft and gives steering
/// back, once ESTABLISHED on the route. Deliberately much tighter than the
/// guide/compliance 60 m: the law holds a couple of meters in normal work,
/// so 25 m off means something is wrong (bad fix, blocked nosewheel, a
/// route the geometry can't follow).
pub(crate) const OFF_ROUTE_STOP_M: f64 = 25.0;
/// How far off the route's line a stop at its end may sit and still be an
/// ARRIVAL. The law holds a couple of metres; the scorer calls 3 m off the
/// line and 5 m inside a corner a miss (`scorecard` `TAXI_MAX_CUT_M`), so
/// past this the aircraft is beside the holding point, not at it.
pub(crate) const ARRIVED_ON_LINE_M: f64 = 5.0;

/// Whether a stop at the route's end, `lateral_m` off its line (either
/// side), is an arrival.
pub(crate) fn arrived_on_line(lateral_m: f64) -> bool {
    lateral_m.abs() <= ARRIVED_ON_LINE_M
}

/// How far short of the route's end, along it, an aircraft taxied there by
/// hand still counts as at the holding point.
pub(crate) const HAND_TAXIED_END_M: f64 = 60.0;

/// Whether a position `to_go_m` short of the route's end (along it) and
/// `lateral_m` off its line is at the holding point, for a drive that did
/// not arrive by itself: near along the route AND on its line.
pub fn at_the_hold(to_go_m: f64, lateral_m: f64) -> bool {
    to_go_m <= HAND_TAXIED_END_M && arrived_on_line(lateral_m)
}

/// What the crew says stopping at the route's end off its line.
fn off_line_stop_line(lateral_m: f64) -> String {
    format!(
        "We're {:.0} metres off the line at the end of the route, not at the holding point. Holding here. Your steering.",
        lateral_m.abs()
    )
}

/// Where a position sits against a route's END: metres still to run along
/// it and metres off its line. The arrival judgement for a caller that was
/// not driving (full auto's check after a pilot-taxied hold).
pub fn route_end_fix(route: &Route, lat: f64, lon: f64) -> Option<(f64, f64)> {
    let (progress, offset) = route.fix_signed(lat, lon)?;
    Some((route.remaining(progress), offset.abs()))
}
/// Farther off than this at engage is a refusal: the projection onto the
/// route stops meaning anything, and a blind cross-country drive to it
/// isn't a join, it's a new route the tower never gave us.
pub(crate) const MAX_JOIN_M: f64 = 150.0;
/// Extra lateral allowance granted while joining, on top of the engage
/// distance: room for the turning circle of a turn-around without ever
/// letting a broken law wander further from the route than it started.
pub(crate) const JOIN_GROW_M: f64 = 40.0;
/// An engage further off than [`OFF_ROUTE_STOP_M`] or with the nose more than
/// this far off the aim is a JOIN (the widened guard, the join pace).
const JOIN_ALLOW_ERR_DEG: f64 = 45.0;
/// Joined when inside this lateral AND aligned within [`JOIN_DONE_ERR_DEG`]:
/// ON the line, which is what "joined" means to everything that reads it.
///
/// It was 15 m and 45 degrees, loose enough to latch in the middle of a
/// capture that had not happened (Southampton, 2026-07-30), and the sortie
/// scorer judges the taxi's lateral from the latch on, so every turn-round
/// off a stand scored the tail of its own U-turn as "off the line": 14.5,
/// 14.9 and 15.3 m (LEMG, LIBP, LIPR, 2026-09-26), the latch threshold
/// itself, with the aircraft still swinging at 42 degrees of error.
const JOIN_DONE_LAT_M: f64 = 3.0;
const JOIN_DONE_ERR_DEG: f64 = 15.0;
/// A capture that has stopped closing is joined where it SETTLED: within the
/// old 15 m, aligned, and no nearer by [`JOIN_PROGRESS_M`] for
/// [`JOIN_SETTLE_S`]. Without it a law holding a steady offset would stay
/// "joining" for the whole taxi, the wide guard on, the trim frozen and the
/// offset never judged.
const JOIN_SETTLE_MAX_LAT_M: f64 = 15.0;
const JOIN_PROGRESS_M: f64 = 0.5;
const JOIN_SETTLE_S: f64 = 8.0;
/// While joining with the nose still this far off the aim, hold the
/// sharp-turn pace; better aligned joins ride at ordinary bend speed.
const JOIN_ERR_SLOW_DEG: f64 = 60.0;
/// Cross-track error that has a wheel off the centreline but is still well
/// inside the paint (a code-C taxiway is ~23 m wide): back off to the bend
/// pace. Past [`TRACK_SHARP_LAT_M`] the aircraft is at or beyond the edge and
/// only the sharp-corner pace is honest. See [`Pace::track_cap_kts`].
const TRACK_TURN_LAT_M: f64 = 6.0;
const TRACK_SHARP_LAT_M: f64 = 12.0;
/// The same two bands in heading. Deliberately looser than the lateral pair: a
/// healthy pure-pursuit intercept carries a real crossing angle by design, so
/// only an angle no intercept would ask for should slow the schedule.
const TRACK_TURN_ERR_DEG: f64 = 20.0;
const TRACK_SHARP_ERR_DEG: f64 = 35.0;
/// Full steering deflection at this much heading error.
const STEER_FULL_DEG: f64 = 30.0;
/// Route bends at/above this (smoothed like the guide's cues) slow the
/// schedule to the pace's turn cap; at/above `SHARP_DEG`, to its sharp cap.
const TURN_DEG: f64 = 30.0;
const SHARP_DEG: f64 = 100.0;
/// The deceleration the bend caps are anticipated at: the stop ramp's own.
const BEND_DECEL_MS2: f64 = STOP_DECEL_MS2;
/// How far before a corner's own window its approach is shaped: enough for
/// 18 kt down to 6 at [`BEND_DECEL_MS2`].
const BEND_ENVELOPE_M: f64 = 160.0;
/// No corner cap below a walking pace: the governor needs a speed to hold.
const CORNER_MIN_KTS: f64 = 3.0;

/// The radius of the circle through three points (metres), or infinity when
/// they lie on a line. The corner radius before #194, kept for the tests'
/// comparison with it.
#[cfg(test)]
fn circle_radius_m(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> f64 {
    let (ab, bc, ca) = (taxi_guide::dist(a, b), taxi_guide::dist(b, c), taxi_guide::dist(c, a));
    let cross = ((b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)).abs();
    if cross < 1e-6 {
        return f64::INFINITY;
    }
    ab * bc * ca / (2.0 * cross)
}
/// The GA pace, tuned on the 172 and the default for any airframe whose
/// profile declares nothing. See [`Pace`].
const STRAIGHT_KTS: f64 = 10.0;
const TURN_KTS: f64 = 6.0;
const SHARP_KTS: f64 = 4.0;

/// How briskly this airframe taxis: the straight-leg pace and the two bend
/// caps. Carried BY THE ROUTE, not a process-wide static, so a profile swap
/// can't leave the last aircraft's pace pointed at the next one — the same
/// trap the radio-power latch and the transponder knob selector fell into.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pace {
    pub straight: f64,
    pub turn: f64,
    pub sharp: f64,
    /// Steering feel, per airframe — see [`crate::aircraft::TaxiProfile`].
    pub steer_full_deg: f64,
    pub steer_lead_s: f64,
    pub steer_slew_per_s: Option<f64>,
    /// Differential-brake steering assist (castoring nosewheel + no prop
    /// wash) — see [`brake_steer_split`].
    pub brake_steer: bool,
    /// Pure-pursuit aim time, seconds — see [`lookahead_m_with`].
    pub lookahead_s: f64,
    /// The yaw rate a corner may ask, deg/s, when the profile declares one
    /// (`taxi.max_yaw_dps`); see [`Route::bend_cap_kts`].
    pub max_yaw_dps: Option<f64>,
    /// A full-length backtrack's pace down the strip, knots, when the
    /// profile declares one (`taxi.backtrack_kts`); else the straight pace
    /// floored at the line-up tool's 14 kt.
    pub backtrack: Option<f64>,
}

impl Default for Pace {
    fn default() -> Self {
        Pace {
            straight: STRAIGHT_KTS,
            turn: TURN_KTS,
            sharp: SHARP_KTS,
            steer_full_deg: STEER_FULL_DEG,
            steer_lead_s: STEER_RATE_LEAD_S,
            steer_slew_per_s: None,
            brake_steer: false,
            lookahead_s: 5.0,
            max_yaw_dps: None,
            backtrack: None,
        }
    }
}

impl Pace {
    /// Resolve an airframe's declared pacing, field by field — anything the
    /// profile leaves out keeps the GA figure.
    pub fn from_profile(t: Option<&crate::aircraft::TaxiProfile>) -> Pace {
        let d = Pace::default();
        let Some(t) = t else { return d };
        // Clamped to the governor's own taxi ceiling: a profile typo must not
        // become a takeoff roll with the tower's route as its plan.
        Pace {
            straight: t
                .kts
                .map_or(d.straight, crate::tools::speedhold::clamp_taxi_target),
            turn: t.turn_kts.map_or(d.turn, |k| k.clamp(2.0, 25.0)),
            sharp: t.sharp_kts.map_or(d.sharp, |k| k.clamp(2.0, 25.0)),
            // Clamped the same way the speeds are: a profile typo must not be
            // able to make the steering either inert or a bang-bang switch.
            steer_full_deg: t
                .steer_full_deg
                .map_or(d.steer_full_deg, |v| v.clamp(10.0, 90.0)),
            steer_lead_s: t.steer_lead_s.map_or(d.steer_lead_s, |v| v.clamp(0.0, 3.0)),
            // A slew below 0.5/s is the auto-takeoff limit-cycle trap; refuse to
            // configure one that tight rather than ship a new oscillation.
            steer_slew_per_s: t.steer_slew_per_s.map(|v| v.clamp(0.5, 10.0)),
            brake_steer: t.brake_steer,
            lookahead_s: t.lookahead_s.map_or(5.0, |v| v.clamp(3.0, 12.0)),
            max_yaw_dps: t.max_yaw_dps.map(|v| v.clamp(5.0, 60.0)),
            backtrack: t.backtrack_kts.map(crate::tools::speedhold::clamp_taxi_target),
        }
    }

    /// The speed cap while joining: a turn-around happens at the sharp-corner
    /// pace, an aligned rejoin at ordinary bend speed. Never the open-taxiway
    /// pace — the aircraft is off the pavement it was cleared on.
    pub fn join_cap_kts(&self, err_deg: f64) -> f64 {
        if err_deg.abs() > JOIN_ERR_SLOW_DEG {
            self.sharp
        } else {
            self.turn
        }
    }

    /// The speed cap for how the aircraft is actually TRACKING, as opposed to
    /// how the route ahead bends.
    ///
    /// The bend caps read the paint and nothing else: they answer "how sharp is
    /// what's coming", never "are we on it". Between them and `join_cap_kts`
    /// there was a hole: the join cap stops at the `join_done` latch (then 15 m,
    /// 45 deg), which is loose enough to fire in the middle of a capture that
    /// has not happened, and past it the schedule went straight back to the
    /// open-taxiway pace. Southampton, 2026-07-30: the SF50 latched joined at
    /// 14.8 m and -28 deg, the last leg was straight so the cap returned to
    /// 12 kt, and it accelerated 6.5 -> 12.7 kt while still 13 m off with 27
    /// deg of crossing angle and 33 m to run. On a castoring-wheel jet whose
    /// entire steering authority is a quarter of one-sided brake (~3.5 deg/s
    /// flat out) that is unrecoverable: it simply floats wide.
    ///
    /// So speed backs off for the error itself, and the cap outlives the join
    /// latch. Slowing down is what a pilot does when the centreline is drifting
    /// away from under the nose; it buys the steering more degrees per metre,
    /// which is the only currency a weak plant has.
    pub fn track_cap_kts(&self, lateral_m: f64, err_deg: f64) -> f64 {
        if lateral_m > TRACK_SHARP_LAT_M || err_deg.abs() > TRACK_SHARP_ERR_DEG {
            self.sharp
        } else if lateral_m > TRACK_TURN_LAT_M || err_deg.abs() > TRACK_TURN_ERR_DEG {
            self.turn
        } else {
            self.straight
        }
    }
}

/// True while the steering loop runs, mirroring `speedhold::ACTIVE` /
/// `autoland::ACTIVE`. Written only by the drive's run ([`RUN`]): the loop
/// lowers it, [`halt`] (the release path in main, the `auto_taxi` tool's off
/// switch, a stand-down) cancels it.
pub static ACTIVE: AtomicBool = AtomicBool::new(false);
/// The drive's run. A drive that is finishing lowers its flag and can no
/// longer clear the flag of the drive after it: a backtrack's leg one
/// finished its exit AFTER the supervisor had started leg two, its late
/// clear cancelled leg two 40 ms into the turn, and full auto then lined up
/// from the wrong heading and 136 m down a 600 m strip (LILB 03,
/// 2026-09-25).
pub(crate) static RUN: sim::run::Slot = sim::run::Slot::flagged(&ACTIVE);
/// True while a BACKTRACK is in progress, including the gap between its two
/// legs. `ACTIVE` alone drops false in that gap — leg one ends, the supervisor
/// builds leg two, and a caller polling `ACTIVE` sees "the drive is finished"
/// for a moment that isn't. Full auto did exactly that and called a running
/// line-up a failure one second after leg two engaged (Luton, 2026-07-28).
pub static BACKTRACKING: AtomicBool = AtomicBool::new(false);

/// True while ANY auto drive owns the aircraft — the drive itself, or a
/// backtrack between its legs. What a caller waiting for "are we there yet"
/// should poll.
pub fn driving() -> bool {
    ACTIVE.load(Ordering::Relaxed) || BACKTRACKING.load(Ordering::Relaxed)
}

/// How many drives have started: a caller compares it across a tool call to
/// know whether [`LAST_ARRIVED`] is about a drive it asked for.
pub fn drive_gen() -> u64 {
    DRIVES_STARTED.load(Ordering::SeqCst)
}

/// Whether the LAST drive ended by ARRIVING at its stop (vs a handback,
/// arrest, or off-route stand-down). The backtrack supervisor reads this to
/// decide whether leg two may begin — any other ending already spoke and
/// must not be driven over.
pub static LAST_ARRIVED: AtomicBool = AtomicBool::new(false);

/// Whether the LAST drive stood down because the aircraft was MOVED under
/// it ([`relocation_jump_m`]): a reposition, a sim reload, a replay scrub.
/// Set by the loop, cleared by every [`start`]. The taxi-in reads it: the
/// drive it was running belongs to a place the aircraft no longer is.
pub static RELOCATED: AtomicBool = AtomicBool::new(false);

/// No aircraft on its wheels covers ground faster than this; the slack
/// absorbs a stale read and the tick's own jitter.
pub(crate) const RELOCATE_MAX_MPS: f64 = 60.0;
pub(crate) const RELOCATE_SLACK_M: f64 = 100.0;

/// A position JUMP between two reads `dt_s` apart: `Some(metres)` when the
/// aircraft moved further than it could have on its wheels, else `None`.
/// A jump is never a movement (the projection rule's sibling): LEMG 31
/// (2026-09-28, sorties 1790584715 and 1790585651) the sortie runner's next
/// setup reloaded the aircraft 1,293 m from its rollout 0.3 s after the
/// landing verdict, the vacate "joined" 1,033 m off its route, and the
/// taxi-in went on to ask for and drive a stand route from the new place.
pub fn relocation_jump_m(from: (f64, f64), to: (f64, f64), dt_s: f64) -> Option<f64> {
    let d = crate::autoland::haversine_nm(from.0, from.1, to.0, to.1) * M_PER_NM;
    (d > RELOCATE_SLACK_M + RELOCATE_MAX_MPS * dt_s.max(0.0)).then_some(d)
}

/// Stop a drive from OUTSIDE its loop, claiming nothing.
///
/// The arrival latch is cleared with the flags, and that is the whole point:
/// a drive somebody else stopped did NOT arrive, and a caller waiting on
/// [`LAST_ARRIVED`] must never read a cancelled drive as a parked aircraft.
/// At BIKF (2026-09-20) a vacate that over-ran its budget was left running,
/// the stand drive was never started on top of it, and when the VACATE route
/// finally ended it set the latch — so the arrival watcher secured the
/// aircraft, transponder to standby and lights out, 140 m from the nearest
/// stand and ninety degrees across it, having never gone to the stand it was
/// cleared to.
///
/// This stops the loop; it does not stop the AIRCRAFT. The caller holds the
/// sim client and owes the brakes — a stage that is not moving the aircraft
/// is holding it.
///
/// Returns whether a drive was running. Every stop from outside comes
/// through here (it was seven bare stores of the flag, each forgetting the
/// backtrack and the arrival latch).
pub fn halt() -> bool {
    let was = RUN.cancel();
    BACKTRACKING.store(false, Ordering::Relaxed);
    LAST_ARRIVED.store(false, Ordering::Relaxed);
    was
}

/// A caller about to start a drive of its own forgets the last one's
/// arrival (the taxi-in, before its stand drive).
pub fn forget_arrival() {
    LAST_ARRIVED.store(false, Ordering::Relaxed);
}

/// A drive is judged STALLED by its progress along the route, never by a
/// flat clock. Full auto waited 600 s for every taxi, and at LEMG (sorties
/// 1790605551 and 1790618416, 2026-09-28) the SR22's 4,661 m and 4,791 m
/// routes from the ramp to Hotel Echo one were given up as "taxi_stalled"
/// at exactly 600 s, still rolling at 4 to 9 kt with 1,998 m and 2,104 m to
/// go: 4.7 km at the 10 kt pace is 906 s of straight taxiway alone. The
/// drive's own guards stand a real stall down (stagnant 30 s, off route, a
/// tug that never lets go at 180 s); this is the backstop behind them: no
/// [`PROGRESS_STEP_M`] gained along the route for this long while the sim
/// is live, no tug holds it and nobody said "stop". The longest those two
/// taxis went without 20 m was the turn-round join, 37.8 s.
pub const DRIVE_NO_PROGRESS_S: f64 = 120.0;
/// What counts as ground gained along the route, m.
pub const PROGRESS_STEP_M: f64 = 20.0;
/// A change of along-route progress past this, plus [`PROJECTION_JUMP_MPS`]
/// for the time between the two readings, is a re-projection (a route that
/// doubles back on itself, a facing pick), never ground made or lost.
pub const PROJECTION_JUMP_M: f64 = 40.0;
/// No taxi makes ground faster than this, m/s (about 58 kt).
pub const PROJECTION_JUMP_MPS: f64 = 30.0;
/// The route's own deadline: the whole length at half the pace, plus two
/// minutes, with every excused hold added on. 4,661 m at 10 kt: 1,932 s.
pub fn route_deadline_s(total_m: f64, base_kts: f64) -> f64 {
    total_m.max(0.0) / (0.5 * base_kts.max(2.0) / MS_TO_KTS) + 120.0
}

/// The progress watch, pure: `note` each fix's along-route progress with
/// a monotonic time; `hold` while a pause, a tug or a spoken stop excuses
/// standing still.
#[derive(Debug, Default, Clone, Copy)]
pub struct ProgressWatch {
    /// (best progress m, when it last advanced a step).
    best: Option<(f64, f64)>,
    /// When the route's deadline falls, and the last time seen.
    deadline: Option<f64>,
    last_t: Option<f64>,
    /// The last progress noted and when, to tell a re-projection from a
    /// movement.
    last_p: Option<(f64, f64)>,
}

impl ProgressWatch {
    /// A drive starting at `t_s` with this much time to finish.
    pub fn start(t_s: f64, deadline_s: f64) -> Self {
        ProgressWatch {
            best: Some((f64::NEG_INFINITY, t_s)),
            deadline: Some(t_s + deadline_s),
            last_t: Some(t_s),
            last_p: None,
        }
    }
    pub fn note(&mut self, progress_m: f64, t_s: f64) {
        self.last_t = Some(t_s);
        let jumped = self.last_p.is_some_and(|(p, t)| {
            (progress_m - p).abs() > PROJECTION_JUMP_M + PROJECTION_JUMP_MPS * (t_s - t).max(0.0)
        });
        self.last_p = Some((progress_m, t_s));
        if jumped {
            // The projection moved, not the aircraft: measure on from the
            // new answer, the clock NOT restarted (it is no ground made).
            // LFRG 12 (2026-10-05): the line-up's turn-round at the end of
            // a down-and-back route jumped 300 m ahead and back, the best
            // stayed 300 m ahead, and the drive back up the strip at 10 kt
            // read "no progress" until full auto gave it up 25 m short.
            if let Some((b, _)) = self.best.as_mut() {
                *b = progress_m;
            }
            return;
        }
        match self.best {
            Some((b, _)) if progress_m < b + PROGRESS_STEP_M => {}
            _ => self.best = Some((progress_m, t_s)),
        }
    }
    /// Standing still for a reason that is not a stall: the clock restarts
    /// and the deadline moves on by the time held.
    pub fn hold(&mut self, t_s: f64) {
        if let (Some(d), Some(l)) = (self.deadline.as_mut(), self.last_t) {
            *d += (t_s - l).max(0.0);
        }
        self.last_t = Some(t_s);
        if let Some((_, t)) = self.best.as_mut() {
            *t = t_s;
        }
    }
    /// Seconds since the route last gave up a step, 0 before the first fix.
    pub fn stalled_s(&self, t_s: f64) -> f64 {
        self.best.map_or(0.0, |(_, t)| (t_s - t).max(0.0))
    }
    /// No step for [`DRIVE_NO_PROGRESS_S`], or past the route's deadline.
    pub fn stuck(&self, t_s: f64) -> bool {
        self.stalled_s(t_s) > DRIVE_NO_PROGRESS_S || self.deadline.is_some_and(|d| t_s > d)
    }
}

/// The running drive's watch, restarted by every [`start`]; its clock is
/// seconds since [`EPOCH`].
static PROGRESS: std::sync::Mutex<ProgressWatch> = std::sync::Mutex::new(ProgressWatch {
    best: None,
    deadline: None,
    last_t: None,
    last_p: None,
});
static EPOCH: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();

fn clock_s() -> f64 {
    EPOCH.get_or_init(std::time::Instant::now).elapsed().as_secs_f64()
}

fn progress_note(progress_m: f64) {
    if let Ok(mut w) = PROGRESS.lock() {
        w.note(progress_m, clock_s());
    }
}

/// The route was rebuilt under the drive ([`Route::with_lead`]): progress is
/// measured afresh from here, the deadline kept.
fn progress_rebase() {
    if let Ok(mut w) = PROGRESS.lock() {
        let t = clock_s();
        w.best = Some((f64::NEG_INFINITY, t));
        w.last_t = Some(t);
        w.last_p = None;
    }
}

fn progress_hold() {
    if let Ok(mut w) = PROGRESS.lock() {
        w.hold(clock_s());
    }
}

/// A drive is running and is stuck by [`ProgressWatch::stuck`]: what a
/// caller waiting for the drive gives up on.
pub fn drive_stalled() -> bool {
    driving() && PROGRESS.lock().is_ok_and(|w| w.stuck(clock_s()))
}

/// The backstop behind [`drive_stalled`] for a caller's own wait, s.
pub const DRIVE_CEILING_S: f64 = 7200.0;

/// One-line status for the dashboard's "managing" panel.
/// The taxi's live line for the panel: metres left to the stop point,
/// rounded to ten so the feed is not re-sent every tick.
pub static DETAIL: crate::auto_status::Detail = crate::auto_status::Detail::new();

pub fn status() -> Option<String> {
    ACTIVE
        .load(Ordering::Relaxed)
        .then(|| DETAIL.line("auto taxi"))
}

/// The cleared route, pre-digested into the local flat-earth frame the
/// steering law works in.
pub struct Route {
    /// Lat/lon polyline, for projecting fixes (same code path as the guide).
    path: Vec<(f64, f64)>,
    /// The polyline in meters (x east, y north) from `path[0]`.
    pts: Vec<(f64, f64)>,
    /// Cumulative along-route distance per vertex.
    cum: Vec<f64>,
    total: f64,
    /// The corners, for the slow-down ([`route_bends`]).
    bends: Vec<Bend>,
    /// Spoken holding point ("November five Whiskey"), may be empty.
    pub holding_point: String,
    /// Overrides the arrival line when set — a LINE-UP drive ends with
    /// "Lined up runway two seven", not "Holding point".
    pub done_line: Option<String>,
    /// This airframe's pacing. `build` defaults it to the GA figures; the
    /// arming tool sets the flying aircraft's own (see [`Pace`]).
    pub pace: Pace,
    /// Along-route spans (start m, end m) that run ON a runway: crossings,
    /// backtracks, a line-up's last leg. From the route's own segment flags,
    /// merged where contiguous. The drive lights the landing light over them.
    pub runway_spans: Vec<(f64, f64)>,
    /// A LINE-UP's runway course, true degrees: on a castoring nose the drive
    /// does not end until the nose points down it ([`LineUpAlign`]). `None`
    /// for every other drive.
    pub align_course_deg: Option<f64>,
    /// ...and the runway it lines up on with the takeoff fit's need, m: the
    /// alignment rolls only the runway the fit can spare
    /// ([`LineUpAlign::with_spare`]). `None` keeps [`ALIGN_MAX_ROLL_M`].
    pub align_runway: Option<(crate::autoland::Runway, f64)>,
    /// The way a reversal at the route's start is turned (+1 right, -1
    /// left), when the geometry knows it: a backtrack's turn-round turns
    /// TOWARD the centreline ([`turn_round_hint`]). `None` leaves the
    /// first sign of the error to decide ([`committed_err`]).
    pub turn_hint: Option<f64>,
}

/// How far before a runway span the landing light comes on.
const CROSSING_LEAD_M: f64 = 40.0;

/// Is `progress` on (or within [`CROSSING_LEAD_M`] of) a runway span?
pub fn on_runway_span(spans: &[(f64, f64)], progress: f64) -> bool {
    spans
        .iter()
        .any(|&(a, b)| progress >= a - CROSSING_LEAD_M && progress <= b)
}

/// The runway spans of a route from its per-segment flags (`seg[i]` is the
/// leg `path[i] -> path[i+1]`), contiguous runway legs merged into one.
pub fn runway_spans(seg: &[(String, bool)], cum: &[f64]) -> Vec<(f64, f64)> {
    let mut spans: Vec<(f64, f64)> = Vec::new();
    for (i, (_, is_rwy)) in seg.iter().enumerate() {
        if !*is_rwy || i + 1 >= cum.len() {
            continue;
        }
        match spans.last_mut() {
            Some(last) if (last.1 - cum[i]).abs() < 1e-6 => last.1 = cum[i + 1],
            _ => spans.push((cum[i], cum[i + 1])),
        }
    }
    spans
}

/// One corner of a digested polyline, for the slow-down: where its turning
/// starts and ends along the route (m), the heading it turns through
/// (degrees, unsigned) and the tightest radius flown inside it (m).
#[derive(Debug, Clone, Copy, PartialEq)]
struct Bend {
    start: f64,
    end: f64,
    deg: f64,
    radius: f64,
}

/// The heading change a corner is found by, over [`BEND_FIND_SPAN_M`] of
/// route. Smoothed like the guide's cues (densely-noded fillet arcs must not
/// read as hairpins), and on an arc the same 30 degrees in 30 m whether it
/// is drawn in one vertex or twelve: a 57 m radius or tighter.
const BEND_FIND_SPAN_M: f64 = 30.0;
/// The span a corner's radius is read over: the route's length over the
/// heading it changes by across this much of it. On a drawn arc that IS its
/// radius; at a lone vertex of D it is 8 / D (in radians), a little under
/// the circle through the route 8 m either side (4 / sin(D/2)), which the
/// cap read before #194.
const BEND_RADIUS_SPAN_M: f64 = 8.0;
/// Stations the route's turning is read at, m.
const BEND_STEP_M: f64 = 1.0;
/// A vertex turning less than this is no part of a corner's extent.
const BEND_VERTEX_MIN_DEG: f64 = 0.5;

/// The corners of a digested polyline, read from its CURVATURE, the heading
/// it changes by per metre, so a corner is the same corner however finely
/// it is drawn. Before #194 a corner was a VERTEX turning 30 degrees over
/// the 15 m either side, and its radius the circle through the route 8 m
/// either side of that vertex: a fillet drawn in 3 to 5 m segments never
/// turns 30 degrees at one vertex, so LEMG's Quebec onto Delta, 104 degrees
/// on about a 16 m radius once the ride took the fillet (e31f338), had no
/// corner at all and the Citation X went in at 17.4 kt (sortie
/// 1790765957, iMac 2026-09-30): full lock for 3 s, 7.3 m off the line.
///
/// A corner is the stretch of stations where the route turns [`TURN_DEG`]
/// or more within [`BEND_FIND_SPAN_M`] centred on them, one way (so it opens
/// up to 15 m before the turning starts, as the vertex reading's
/// neighbouring vertices did); it turns through the sum of its turning
/// vertices, and its radius is the tightest [`BEND_RADIUS_SPAN_M`] inside.
fn route_bends(pts: &[(f64, f64)], cum: &[f64]) -> Vec<Bend> {
    // Each vertex's signed turn, over segments of half a metre or more.
    let mut vt: Vec<(f64, f64)> = Vec::new();
    let mut last_brg: Option<f64> = None;
    for i in 0..pts.len().saturating_sub(1) {
        if cum[i + 1] - cum[i] < 0.5 {
            continue;
        }
        let b = taxi_guide::bearing(pts[i], pts[i + 1]);
        if let Some(p) = last_brg {
            let t = crate::angles::signed_diff_deg(b, p);
            if t != 0.0 {
                vt.push((cum[i], t));
            }
        }
        last_brg = Some(b);
    }
    if vt.is_empty() {
        return Vec::new();
    }
    // The heading turned from the start to `s`, and across (a, b].
    let mut prefix = Vec::with_capacity(vt.len() + 1);
    prefix.push(0.0);
    for &(_, t) in &vt {
        prefix.push(prefix.last().unwrap() + t);
    }
    let turned_to = |s: f64| prefix[vt.partition_point(|&(at, _)| at <= s)];
    let turned = |a: f64, b: f64| turned_to(b) - turned_to(a);
    let total = *cum.last().unwrap_or(&0.0);
    let half = BEND_FIND_SPAN_M / 2.0;
    let r_half = BEND_RADIUS_SPAN_M / 2.0;
    let mut bends: Vec<Bend> = Vec::new();
    let mut group: Option<(f64, f64, f64, f64)> = None; // (first, last, side, most)
    let flush = |g: (f64, f64, f64, f64), bends: &mut Vec<Bend>| {
        let (g0, g1, side, most) = g;
        let turning: Vec<f64> = vt
            .iter()
            .filter(|&&(at, t)| at > g0 - half && at <= g1 + half && t * side >= BEND_VERTEX_MIN_DEG)
            .map(|&(at, _)| at)
            .collect();
        let (Some(&first), Some(&last)) = (turning.first(), turning.last()) else {
            return;
        };
        let through = turned(first - 1e-6, last).abs().max(most);
        let (start, end) = (g0.min(first), g1.max(last));
        let mut radius = f64::INFINITY;
        let mut s = start - r_half;
        while s <= end + r_half {
            let d = turned(s - r_half, s + r_half).abs().to_radians();
            if d > 1e-6 {
                radius = radius.min(BEND_RADIUS_SPAN_M / d);
            }
            s += BEND_STEP_M / 2.0;
        }
        bends.push(Bend { start, end, deg: through, radius });
    };
    let mut s = 0.0;
    while s <= total {
        // Either a corner's worth of heading in the find span, or a turn
        // that tight in the radius span (a jog: +27 then -47 degrees in 9 m
        // turns the aircraft hard and nets little, LIRF).
        let wide = turned(s - half, s + half);
        let tight = turned(s - r_half, s + r_half);
        let w = if tight.abs() >= TURN_DEG && tight.abs() > wide.abs() { tight } else { wide };
        let side = w.signum();
        if w.abs() >= TURN_DEG {
            group = match group {
                Some((g0, _, gs, most)) if gs == side => Some((g0, s, side, most.max(w.abs()))),
                Some(g) => {
                    flush(g, &mut bends);
                    Some((s, s, side, w.abs()))
                }
                None => Some((s, s, side, w.abs())),
            };
        } else if let Some(g) = group.take() {
            flush(g, &mut bends);
        }
        s += BEND_STEP_M;
    }
    if let Some(g) = group {
        flush(g, &mut bends);
    }
    bends
}

impl Route {
    /// The route's whole length, metres.
    pub fn total_m(&self) -> f64 {
        self.total
    }

    /// This route joined by a planned turn ([`join_turn`]): `lead` (lat/lon,
    /// from the aircraft round the turn to the join point) then the route
    /// beyond `join_m`. Pace, holding point and the rest are kept; the runway
    /// spans move with the join. `None` if the result is too short to drive.
    pub(crate) fn with_lead(&self, lead: &[(f64, f64)], join_m: f64) -> Option<Route> {
        let mut path: Vec<(f64, f64)> = lead.to_vec();
        path.extend(
            self.path
                .iter()
                .zip(self.cum.iter())
                .filter(|(_, &c)| c > join_m + 0.5)
                .map(|(p, _)| *p),
        );
        if path.len() < 2 {
            return None;
        }
        let geo = taxi_guide::Geo::new(path[0].0);
        let pts: Vec<(f64, f64)> = path.iter().map(|&p| geo.to_m(path[0], p)).collect();
        let mut cum = vec![0.0_f64];
        for w in pts.windows(2) {
            cum.push(cum.last().unwrap() + taxi_guide::dist(w[0], w[1]));
        }
        let total = *cum.last().unwrap();
        if total < STOP_SHORT_M + 15.0 {
            return None;
        }
        let lead_m: f64 = lead
            .windows(2)
            .map(|w| taxi_guide::dist(geo.to_m(path[0], w[0]), geo.to_m(path[0], w[1])))
            .sum();
        let shift = lead_m - join_m;
        let runway_spans = self
            .runway_spans
            .iter()
            .filter(|&&(_, b)| b > join_m)
            .map(|&(a, b)| ((a.max(join_m) + shift).max(0.0), b + shift))
            .collect();
        Some(Route {
            bends: route_bends(&pts, &cum),
            path,
            pts,
            cum,
            total,
            holding_point: self.holding_point.clone(),
            done_line: self.done_line.clone(),
            pace: self.pace,
            runway_spans,
            align_course_deg: self.align_course_deg,
            align_runway: self.align_runway.clone(),
            turn_hint: self.turn_hint,
        })
    }

    /// Digest a published route. `None` when it's too short to drive.
    pub fn build(watch: &TaxiWatch) -> Option<Route> {
        if watch.path.len() < 2 {
            return None;
        }
        let geo = taxi_guide::Geo::new(watch.path[0].0);
        let pts: Vec<(f64, f64)> = watch
            .path
            .iter()
            .map(|&p| geo.to_m(watch.path[0], p))
            .collect();
        let mut cum = vec![0.0_f64];
        for w in pts.windows(2) {
            cum.push(cum.last().unwrap() + taxi_guide::dist(w[0], w[1]));
        }
        let total = *cum.last().unwrap();
        // Shorter than the stop-short margin plus a rolling start there is
        // nothing to drive — the tool answers "we're already there".
        if total < STOP_SHORT_M + 15.0 {
            return None;
        }
        let bends = route_bends(&pts, &cum);
        let runway_spans = runway_spans(&watch.seg, &cum);
        Some(Route {
            path: watch.path.clone(),
            pts,
            cum,
            total,
            bends,
            holding_point: crate::station::spoken_taxiway(&watch.holding_point),
            done_line: None,
            pace: Pace::default(),
            runway_spans,
            align_course_deg: None,
            align_runway: None,
            turn_hint: None,
        })
    }

    /// Project a lat/lon fix: (along-route progress m, DISTANCE off the route
    /// in m). Never negative — every gate here asks "how far off", which is
    /// the same question on both sides of the line.
    pub fn fix(&self, lat: f64, lon: f64) -> Option<(f64, f64)> {
        taxi_guide::project(&self.path, lat, lon)
    }

    /// [`fix`](Self::fix) keeping the SIDE: positive is right of the route.
    /// What the drive records, so a trail can be read against the paint
    /// afterwards instead of only saying how far off it was.
    pub fn fix_signed(&self, lat: f64, lon: f64) -> Option<(f64, f64)> {
        taxi_guide::project_signed(&self.path, lat, lon)
    }

    /// [`fix_signed`](Self::fix_signed) onto the legs that run the way the
    /// nose points — the projection a JOIN uses, so the nearest paint behind
    /// the tail never becomes the target (see `taxi_guide::project_signed_facing`).
    pub fn fix_signed_facing(&self, lat: f64, lon: f64, heading_true: f64) -> Option<(f64, f64)> {
        taxi_guide::project_signed_facing(&self.path, lat, lon, heading_true)
    }

    /// Heading error (signed degrees, right positive) from the aircraft's
    /// true heading to the pure-pursuit aim point `lookahead` meters further
    /// along the route. Aiming at a point ON the route (not the tangent) is
    /// what folds the cross-track error back to the centreline.
    pub fn heading_err_deg(
        &self,
        lat: f64,
        lon: f64,
        heading_true: f64,
        progress: f64,
        lookahead: f64,
    ) -> f64 {
        let geo = taxi_guide::Geo::new(self.path[0].0);
        let here = geo.to_m(self.path[0], (lat, lon));
        let aim = taxi_guide::point_at(&self.pts, &self.cum, progress + lookahead);
        crate::angles::signed_diff_deg(taxi_guide::bearing(here, aim), heading_true)
    }

    /// The yaw rate the ROUTE ITSELF asks for across the aim span, degrees
    /// per second at this groundspeed: the route's direction at the aim point
    /// minus its direction underfoot, over the time the aircraft takes to
    /// cover the span.
    ///
    /// On a circular arc of radius R this is exactly v/R — the span subtends
    /// L/R radians and is covered in L/v seconds — and on the polyline a
    /// taxiway really is, it is that same measure sampled across whatever
    /// corner lies inside the aim. Straight paint gives zero.
    ///
    /// This is the turn a correctly flown bend MUST carry, which is what
    /// [`damped_err_deg`] credits back so the rate lead stops fighting it.
    pub fn route_turn_rate_dps(&self, progress: f64, lookahead: f64, gs_kts: f64) -> f64 {
        // Sampled over a short step either end rather than read off a leg
        // index: `point_at` already walks the polyline, and a corner landing
        // between two samples is a corner the aim is flying through anyway.
        const STEP_M: f64 = 4.0;
        if lookahead <= STEP_M || gs_kts <= 0.0 {
            return 0.0;
        }
        let dir = |s: f64| {
            taxi_guide::bearing(
                taxi_guide::point_at(&self.pts, &self.cum, s),
                taxi_guide::point_at(&self.pts, &self.cum, s + STEP_M),
            )
        };
        let turn = crate::angles::signed_diff_deg(dir(progress + lookahead), dir(progress));
        turn * (gs_kts * 0.514) / lookahead
    }

    /// The direction the route's LAST segment runs, degrees true.
    ///
    /// Near the stop this replaces the pure-pursuit aim, because the aim point
    /// saturates: `point_at` clamps at the final point, so once the aircraft
    /// rolls past it the bearing to it swings round BEHIND and the law
    /// commands a hard turn toward a waypoint already passed. Measured at
    /// EGHI (2026-07-30), the error ran away with the aircraft dead on the
    /// centreline:
    ///
    ///   rem  0.04 m -> err   9.5°     rem -3.6 m -> err  38.6°
    ///   rem -4.4  m -> err  70.2°     rem -5.0 m -> err 162.6°
    ///
    /// Steering pinned at its limit and the nose came round: 0.07 m off the
    /// line to 3.00 m in four seconds. Every stop did it — hold, line-up,
    /// backtrack, taxi-in — because they all drive this one loop.
    /// The direction of the route's last REAL segment. The planner can end a
    /// route on a repeated point (LIPR Alpha, 2026-09-26: the last segment
    /// was 0.00 m), and the bearing of a zero-length segment is 0, due north:
    /// with 8 m to run the taxi steered a 220-degree aircraft 140 degrees
    /// toward it, full right, and it swung to 255 as it braked. Ten such
    /// jumps at 8 m to go are in the logs. Segments under a metre are no
    /// direction at all.
    pub fn final_bearing_deg(&self) -> Option<f64> {
        const MIN_SEG_M: f64 = 1.0;
        self.pts
            .windows(2)
            .rev()
            .find(|w| taxi_guide::dist(w[0], w[1]) >= MIN_SEG_M)
            .map(|w| taxi_guide::bearing(w[0], w[1]))
    }

    /// Heading error against the route's final bearing — what to fly once the
    /// pursuit aim can no longer be trusted. Falls back to the pursuit error
    /// when the route is too short to have a final segment.
    pub fn stop_heading_err_deg(
        &self,
        lat: f64,
        lon: f64,
        heading_true: f64,
        progress: f64,
        lookahead: f64,
    ) -> f64 {
        match self.final_bearing_deg() {
            Some(b) => crate::angles::signed_diff_deg(b, heading_true),
            None => self.heading_err_deg(lat, lon, heading_true, progress, lookahead),
        }
    }

    /// Meters left before the stop point short of the holding point.
    pub fn remaining(&self, progress: f64) -> f64 {
        self.total - STOP_SHORT_M - progress
    }

    /// The bend-imposed speed cap near `progress`, if a corner is close
    /// enough to matter: inside a speed-scaled lead distance before the
    /// corner, and until 15 m past it (mid-turn is no place to accelerate).
    ///
    /// Before that window the cap is the corner's speed plus what
    /// [`BEND_DECEL_MS2`] can take off in the distance left to the window,
    /// so the governor arrives AT the corner's pace instead of being handed
    /// it at the corner: on a 737 the cap arrived about 30 m out and it went
    /// into LEMG's Alpha at 11.5 kt against 10 (2026-09-26). And where the
    /// airframe declares the yaw rate it can turn at (`Pace::max_yaw_dps`),
    /// no corner is taken faster than that rate times its radius: the same
    /// corner was drawn at 9 m and asked 38 deg/s of an aircraft that turns
    /// at 20, and it swung 5.8 m wide.
    ///
    /// A corner is read from the route's curvature ([`route_bends`]): the
    /// window opens before its FIRST turning vertex and holds to 15 m past
    /// its last, so a drawn arc is slowed for where it starts to turn.
    pub fn bend_cap_kts(&self, progress: f64, gs_kts: f64) -> Option<f64> {
        let lead = (gs_kts * 0.514 * 6.0).clamp(25.0, 90.0);
        self.bends
            .iter()
            .filter(|b| progress < b.end + 15.0 && b.start - progress < lead + BEND_ENVELOPE_M)
            .map(|&Bend { start: at, deg, radius, .. }| {
                let class = if deg >= SHARP_DEG {
                    self.pace.sharp
                } else {
                    self.pace.turn
                };
                let corner = self
                    .pace
                    .max_yaw_dps
                    .map_or(class, |w| class.min(w.to_radians() * radius * MS_TO_KTS).max(CORNER_MIN_KTS));
                let before = (at - progress - lead).max(0.0);
                let v = corner / MS_TO_KTS;
                MS_TO_KTS * (v * v + 2.0 * BEND_DECEL_MS2 * before).sqrt()
            })
            .fold(None, |acc: Option<f64>, cap| {
                Some(acc.map_or(cap, |a| a.min(cap)))
            })
    }

    /// A fix in the route's own frame, metres (x east, y north).
    pub fn local_m(&self, lat: f64, lon: f64) -> (f64, f64) {
        taxi_guide::Geo::new(self.path[0].0).to_m(self.path[0], (lat, lon))
    }

    /// The route in metres for the free nose's one ground law
    /// ([`crate::ground_steer::RefPath`]).
    pub fn ref_path(&self) -> crate::ground_steer::RefPath {
        crate::ground_steer::RefPath::new(&self.pts, &self.cum)
    }
}

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
    trim: f64,
}

/// Steer per metre of offset per second. Slow on purpose: 4 m of offset
/// builds a 0.09 trim in about fifteen seconds, longer than the SF50's
/// fourteen-second weave, so the trim cannot join an oscillation.
const TRIM_PER_M_S: f64 = 0.0015;
/// The most standing steer the trim may hold.
const TRIM_MAX: f64 = 0.15;
/// Offsets beyond this are a corner or a join, not a bias.
const TRIM_MAX_OFFSET_M: f64 = 8.0;
/// The route's own turn rate above which a leg is a bend.
const TRIM_BEND_DPS: f64 = 1.0;
/// Time constant of the bleed through a bend.
const TRIM_BLEED_S: f64 = 3.0;

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
const BRAKE_STEER_DEADBAND: f64 = 0.05;
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
const BRAKE_STEER_MAX: f64 = 0.25;
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
const BRAKE_STEER_SAT: f64 = 0.6;
/// The assist's own slew, per second — pedals squeeze, they don't stab,
/// exactly the taxi governor's brake discipline.
const BRAKE_STEER_SLEW_PER_S: f64 = 0.4;
/// Speed band the assist fades in across. Below it the nosewheel's own
/// low-speed steering (and the stop machinery) owns the aircraft — braking a
/// near-stopped aircraft asymmetrically is a lurch.
const BRAKE_STEER_MIN_KTS: f64 = 4.0;
const BRAKE_STEER_FULL_KTS: f64 = 7.0;

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
const TURN_COMMIT_DEG: f64 = 150.0;
const TURN_RELEASE_DEG: f64 = 90.0;

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
const PIVOT_MIN_ERR_DEG: f64 = 60.0;

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
const PIVOT_RATE_SPAN_DPS: f64 = 10.0;

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
    smoothed: f64,
    /// The last (left, right) pair written.
    prev: (f64, f64),
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
const ALIGN_RUDDER_SLEW_PER_S: f64 = 0.8;
/// Inside this error and yaw rate the pivot lets go.
const ALIGN_QUIET_DEG: f64 = 1.0;
const ALIGN_QUIET_DPS: f64 = 1.0;
/// A line-up is ALIGNED within this of the runway's course: the scorecard
/// judges the roll's first tick at 3.
pub const ALIGNED_DEG: f64 = 2.0;
/// ...and a line-up out of time or runway still hands over within this: the
/// roll holds its power until the nose is inside [`ALIGNED_DEG`].
pub const ALIGN_ACCEPT_DEG: f64 = 3.0;
/// Stopped, for the aligned verdict: groundspeed and yaw rate under these.
const ALIGN_SETTLED_KTS: f64 = 0.5;
const ALIGN_SETTLED_DPS: f64 = 0.7;
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
const ALIGN_STOP_DECEL_MS2: f64 = 0.5;
const ALIGN_STOP_LATENCY_S: f64 = 0.5;
/// The trim that holds a standing yaw: brake per degree of error per
/// second of creep, bounded, integrated only while the nose is not already
/// coming round by [`ALIGN_TRIM_TURNING_DPS`].
const ALIGN_TRIM_PER_DEG_S: f64 = 0.006;
const ALIGN_TRIM_MAX: f64 = 0.12;
const ALIGN_TRIM_TURNING_DPS: f64 = 0.5;
const ALIGN_TRIM_MIN_KTS: f64 = 0.7;
/// A line-up is on the centreline within this, m: past it the alignment
/// closes the lateral too ([`LineUpAlign::step_on_line`]). The alignment
/// stage judges the roll's first tick at 2 m (`scorecard::LINEUP_MAX_XTK_M`).
pub const ALIGN_LATERAL_M: f64 = 1.5;
/// ...and once closing, it closes to this before it straightens.
const ALIGN_LATERAL_DONE_M: f64 = 0.75;
/// The intercept onto the centreline: degrees per metre of offset past
/// [`ALIGN_INTERCEPT_DEAD_M`], never more than [`ALIGN_INTERCEPT_MAX_DEG`].
const ALIGN_INTERCEPT_PER_M: f64 = 6.0;
const ALIGN_INTERCEPT_DEAD_M: f64 = 0.3;
const ALIGN_INTERCEPT_MAX_DEG: f64 = 8.0;
/// The runway and time kept back from a lateral closure to straighten in
/// after it: past these it stops closing and only aligns.
const ALIGN_LATERAL_RESERVE_M: f64 = 10.0;
const ALIGN_LATERAL_RESERVE_S: f64 = 12.0;
/// A closure may take the alignment this much longer than
/// [`ALIGN_MAX_S`]: an S across 2 m at a creep is a minute's work, not forty
/// seconds'.
const ALIGN_LATERAL_EXTRA_S: f64 = 25.0;

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
/// own [`crate::autotakeoff::MIN_REMAINING_M`], as `runway_fit` judges it).
/// Negative when the fit is already lost. Pure.
pub fn align_spare_m(rwy: &crate::autoland::Runway, need_m: f64, lat: f64, lon: f64) -> f64 {
    let past_m = (-crate::autoland::along_track_nm(lat, lon, rwy) * M_PER_NM).max(0.0);
    rwy.length_m - past_m - need_m.max(crate::autotakeoff::MIN_REMAINING_M)
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

/// What the line-up's alignment wants this tick ([`LineUpAlign::step`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AlignStep {
    /// The differential brakes, and whether the governor creeps (`true`) or
    /// stops (`false`, its symmetric brake then outweighs these).
    Align { bl: f64, br: f64, creep: bool },
    /// Stopped with the nose within [`ALIGNED_DEG`] of the course.
    Done { err_deg: f64 },
    /// Out of time or of runway rolled: stop, `err_deg` off.
    GiveUp { err_deg: f64, why: &'static str },
}

/// A line-up does not end until the nose points down the runway, on a
/// castoring nose: its position can be right and its heading not. LEMG 12
/// SR22 (1790690995, 2026-09-29): the drive ended 5.0 degrees off its final
/// bearing, still yawing left at 1.7 deg/s, and stopped at 107 true on a
/// 117.7 runway: the roll began 10.8 off (alignment FAIL). LILB 03 Evolution
/// (1790712974): the backtrack's turn-round ended 11.8 off (13 at the
/// stop) and the roll began 24.6 off. Runway STARTS, set on the numbers,
/// roll 0.3 to 0.5 off. So after the arrival the nose is brought round on
/// the brakes ([`align_pivot_brakes`]) at a creep, stopped, and judged
/// stopped; again if the stop swung it; bounded by [`ALIGN_MAX_S`] and
/// the runway it can roll ([`align_roll_limit_m`]). Pure: the drive feeds it.
///
/// It spends as little runway as it can. LILB 03 Evolution (1790721973,
/// 2026-09-30): the pivot took 13 degrees off in the first 2 m, then crept
/// 38 m more with the nose held 2.0 to 2.2 off by a standing yaw the 0.02
/// of brake its soft law gave could not beat, the idle thrust running the
/// creep from 2 to 5.7 kt, and stopped with 474.8 m ahead of a 500 m fit
/// (516.1 at the turn-round's stop). So: a slow trim holds the standing
/// yaw, a creep over its speed is stopped, and the roll is bounded by the
/// runway the fit can spare, the stop anticipated; with little to spare the
/// creep is halved and the nose pivots nearly in place.
#[derive(Debug, Clone)]
pub struct LineUpAlign {
    pub course_deg: f64,
    t0: Option<f64>,
    creeping: bool,
    /// How many times the governor was asked to creep (a stop that swung
    /// the nose is a second).
    pub creeps: u32,
    /// Ground it may roll, m ([`align_roll_limit_m`]).
    pub roll_limit_m: f64,
    /// The creep it asks the governor for.
    pub creep_kts: f64,
    /// The standing-yaw trim, signed brake (+ right).
    pub trim: f64,
    over: bool,
    last_t: Option<f64>,
    /// Closing a lateral past [`ALIGN_LATERAL_M`] ([`Self::step_on_line`]).
    pub closing: bool,
    /// Whether a lateral closure ever began (the time bound grows by
    /// [`ALIGN_LATERAL_EXTRA_S`]).
    pub closed_lateral: bool,
    /// A free castoring nose's one ground law (`ground_steer`): its brakes
    /// replace the aligning PD and its trim ([`Self::with_ground`]).
    ground: Option<crate::ground_steer::GroundSteer>,
    /// ...and the rudder it owns, for the drive to write.
    pub rudder: f64,
}

impl LineUpAlign {
    #[cfg(test)]
    pub fn new(course_deg: f64) -> Self {
        Self::with_spare(course_deg, None)
    }

    /// An alignment that may roll only the runway `spare_m` the takeoff fit
    /// does not need.
    pub fn with_spare(course_deg: f64, spare_m: Option<f64>) -> Self {
        let roll_limit_m = align_roll_limit_m(spare_m);
        let creep_kts = if roll_limit_m < ALIGN_TIGHT_ROLL_M {
            ALIGN_TIGHT_CREEP_KTS
        } else {
            ALIGN_CREEP_KTS
        };
        LineUpAlign {
            course_deg,
            t0: None,
            creeping: false,
            creeps: 0,
            roll_limit_m,
            creep_kts,
            trim: 0.0,
            over: false,
            last_t: None,
            closing: false,
            closed_lateral: false,
            ground: None,
            rudder: 0.0,
        }
    }

    /// The alignment flown by the free nose's one ground law on `plant`:
    /// the heading (or the lateral's intercept) held by the plant inverted,
    /// the standing yaw its observer's, never faster than
    /// [`ALIGN_RATE_MAX_DPS`]. The creep, the stops and the bounds are this
    /// judge's, as before.
    pub fn with_ground(mut self, plant: crate::ground_steer::GroundPlant) -> Self {
        self.ground = Some(crate::ground_steer::GroundSteer::new(plant));
        self
    }

    /// Whether the one ground law flies it.
    pub fn free(&self) -> bool {
        self.ground.is_some()
    }

    /// Signed error, positive = the runway is right of the nose.
    pub fn err_deg(&self, psi_true: f64) -> f64 {
        crate::angles::signed_diff_deg(self.course_deg, psi_true)
    }

    #[cfg(test)]
    pub fn step(
        &mut self,
        t_s: f64,
        psi_true: f64,
        gs_kts: f64,
        yaw_dps: f64,
        rolled_m: f64,
    ) -> AlignStep {
        self.step_on_line(t_s, psi_true, gs_kts, yaw_dps, rolled_m, None)
    }

    /// [`Self::step`] knowing the lateral off the runway's centreline
    /// (`lat_m`, right positive): past [`ALIGN_LATERAL_M`] the creep flies
    /// an intercept back onto the line ([`align_intercept_deg`]) to
    /// [`ALIGN_LATERAL_DONE_M`] and then straightens, while the runway it
    /// may roll and the time keep [`ALIGN_LATERAL_RESERVE_M`] and
    /// [`ALIGN_LATERAL_RESERVE_S`] back to straighten in; past them it only
    /// aligns, as before. LILB 03 Evolution (1790740292, 2026-09-30): the
    /// turn-round's leg two stopped 1.9 m left and 11.7 degrees off, the
    /// alignment straightened the nose in 8 m and the roll began 2.3 m off
    /// (the alignment stage FAIL at 2), 61.9 m of runway to spare. A
    /// lateral inside [`ALIGN_LATERAL_M`] is left alone: the law is the
    /// heading's alone, as before.
    pub fn step_on_line(
        &mut self,
        t_s: f64,
        psi_true: f64,
        gs_kts: f64,
        yaw_dps: f64,
        rolled_m: f64,
        lat_m: Option<f64>,
    ) -> AlignStep {
        let t0 = *self.t0.get_or_insert(t_s);
        let dt = self.last_t.map_or(0.0, |l| (t_s - l).clamp(0.0, 1.0));
        self.last_t = Some(t_s);
        let course_err = self.err_deg(psi_true);
        // The stop this speed still needs, counted against the runway.
        let v = gs_kts / MS_TO_KTS;
        let stop_m = v * ALIGN_STOP_LATENCY_S + v * v / (2.0 * ALIGN_STOP_DECEL_MS2);
        let max_s = if self.closed_lateral {
            ALIGN_MAX_S + ALIGN_LATERAL_EXTRA_S
        } else {
            ALIGN_MAX_S
        };
        // A lateral closure while the runway and the time still leave room
        // to straighten after it.
        let room = rolled_m + stop_m + ALIGN_LATERAL_RESERVE_M < self.roll_limit_m
            && t_s - t0 + ALIGN_LATERAL_RESERVE_S < ALIGN_MAX_S + ALIGN_LATERAL_EXTRA_S;
        self.closing = match lat_m.filter(|l| l.is_finite()) {
            Some(l) if room => {
                if self.closing {
                    l.abs() > ALIGN_LATERAL_DONE_M
                } else {
                    l.abs() > ALIGN_LATERAL_M
                }
            }
            _ => false,
        };
        self.closed_lateral |= self.closing;
        // Closing, the heading wanted is the intercept; else the course.
        let err = if self.closing {
            course_err + align_intercept_deg(lat_m.unwrap_or(0.0))
        } else {
            course_err
        };
        let led = err - yaw_dps * ALIGN_LEAD_S;
        let settled = gs_kts < ALIGN_SETTLED_KTS && yaw_dps.abs() < ALIGN_SETTLED_DPS;
        if settled && !self.closing && err.abs() <= ALIGNED_DEG {
            self.creeping = false;
            return AlignStep::Done { err_deg: err };
        }
        let out_of_bounds = if t_s - t0 > max_s {
            Some("time")
        } else if rolled_m + stop_m >= self.roll_limit_m {
            Some("rolled")
        } else {
            None
        };
        if let Some(why) = out_of_bounds {
            // Out of bounds but inside what the roll's own hold straightens
            // (`autotakeoff::RollAlign`) and the alignment stage passes: done,
            // not a give-up (LILB 03 Evolution 1790720549, 2026-09-30: the
            // turn-round settled 2.2 off in 24.6 s and the mission gave up).
            self.creeping = false;
            if err.abs() <= ALIGN_ACCEPT_DEG {
                return AlignStep::Done { err_deg: err };
            }
            return AlignStep::GiveUp { err_deg: err, why };
        }
        // Creep while the nose still has to come round; stop once it is
        // close and not carried past (half the band, with hysteresis).
        let off = err.abs() > ALIGNED_DEG || led.abs() > ALIGNED_DEG;
        let close = err.abs() <= ALIGNED_DEG * 0.5 && led.abs() <= ALIGNED_DEG;
        let want = self.closing || if self.creeping { !close } else { off };
        // ...never faster than the creep: over it, stopped until back under.
        if gs_kts > self.creep_kts + ALIGN_CREEP_OVER_KTS {
            self.over = true;
        } else if gs_kts <= self.creep_kts {
            self.over = false;
        }
        if want && !self.creeping {
            self.creeps += 1;
        }
        self.creeping = want;
        let creep = want && !self.over;
        // The standing yaw (the propeller, a slope, a crosswind) is what a
        // soft law settles against: trimmed out while rolling and not
        // already coming round.
        let (bl, br) = match self.ground.as_mut() {
            Some(g) => {
                let s = g.step(crate::ground_steer::heading_demand(
                    dt,
                    gs_kts / MS_TO_KTS,
                    crate::ground_steer::Yaw::Now(yaw_dps),
                    err,
                    crate::ground_steer::Rudder::Own { slew_per_s: ALIGN_RUDDER_SLEW_PER_S },
                    PIVOT_BRAKE_MAX,
                    ALIGN_RATE_MAX_DPS,
                ));
                self.rudder = s.rudder;
                s.wheels()
            }
            None => {
                if want {
                    self.trim = align_trim_step(self.trim, err, yaw_dps, gs_kts, dt);
                }
                with_align_trim(align_pivot_brakes(err, yaw_dps), self.trim, yaw_dps)
            }
        };
        AlignStep::Align { bl, br, creep }
    }
}

/// Steering held while still rolling, even after the distance fade has run
/// out. Enough for the yaw damper to arrest a rotation, far short of the
/// full deflection that carves.
const STRAIGHTEN_FLOOR: f64 = 0.35;
/// Below this groundspeed the aircraft has genuinely stopped and the fade may
/// go to zero. Matches the arrival test's own 1.5 kt so the two agree.
const STRAIGHTEN_HOLD_KTS: f64 = 1.5;

/// Straighten the nosewheel as a stop closes: braking shifts weight onto
/// the nosewheel, and a deflected one carves under it — every stop at a
/// bendy route end turned into a swerve-and-lurch ("when you brake, the
/// aircraft steers", 2026-07-20). Full steering beyond 8 m to go, faded
/// across the last meters.
///
/// The fade may NOT reach zero while the aircraft is still moving.
/// `remaining_m` SATURATES at `-STOP_SHORT_M` the instant the projection
/// reaches the end of the polyline (`remaining = total - STOP_SHORT_M -
/// progress`, and progress cannot exceed total), so the distance term pins at
/// -5 and the fade hits zero with 2-3 kt still on the clock. Zeroing the
/// command also kills the yaw-rate damping *inside* it, so a residual rotation
/// goes unarrested through the last metres.
///
/// EGHI 2026-07-30, at the HOLDING POINT and repeatably: steer pinned at 0.00,
/// yaw −2 deg/s unarrested, cross-track growing 2.45 → 5.08 m while coasting
/// 2.7 → 1.6 kt. The aircraft stopped several metres off the paint and visibly
/// crooked. The same signature is in all three stops of that sortie.
///
/// The carve this exists to prevent needs SPEED; by the arrival threshold
/// there is none left to carve with, so a floor below it is free.
pub fn stop_straighten(steer: f64, remaining_m: f64, gs_kts: f64) -> f64 {
    let fade = ((remaining_m - 1.0) / 7.0).clamp(0.0, 1.0);
    let floor = if gs_kts > STRAIGHTEN_HOLD_KTS {
        STRAIGHTEN_FLOOR
    } else {
        0.0
    };
    steer * fade.max(floor)
}

/// The lateral allowance for an engage at `lateral` meters off the route
/// with `err_deg` of heading error: zero for an on-route, along-route start
/// (the tight established guard applies from the first tick), otherwise the
/// engage distance plus turning-circle room. The guard while joining is
/// [`OFF_ROUTE_STOP_M`] plus this.
pub fn join_allowance_m(lateral: f64, err_deg: f64) -> f64 {
    if lateral > OFF_ROUTE_STOP_M || err_deg.abs() > JOIN_ALLOW_ERR_DEG {
        lateral.max(OFF_ROUTE_STOP_M) + JOIN_GROW_M
    } else {
        0.0
    }
}

/// A JUMP IN A PROJECTION IS NEVER A MOVEMENT, tick to tick. While joining,
/// the facing pick (`taxi_guide::project_signed_facing`) can move to another
/// leg between two ticks as the nose swings; an offset that grew by more than
/// the aircraft moved (plus [`PROJECTION_JUMP_SLACK_M`]) is that jump, and
/// the plain nearest paint stands for the tick. LEMG 13 (sortie
/// 1790646817-full_auto, 2026-09-29, #178): a turn-round join 8.3 m off read
/// 149.4 m the next tick at 5.6 kt, and the off-route guard stopped the
/// taxi. `(fix, jumped)`. Pure.
pub(crate) fn join_fix(
    facing: Option<(f64, f64)>,
    plain: Option<(f64, f64)>,
    prev_lateral: Option<f64>,
    moved_m: f64,
) -> (Option<(f64, f64)>, bool) {
    match (facing, plain, prev_lateral) {
        (Some(f), Some(p), Some(prev))
            if f.1.abs() > prev + moved_m.max(0.0) + PROJECTION_JUMP_SLACK_M
                && p.1.abs() < f.1.abs() =>
        {
            (Some(p), true)
        }
        _ => (facing, false),
    }
}

/// A JOIN THE NOSE CANNOT POINT AT IS A PLANNED TURN, or no join at all.
/// Pure pursuit joins a route by aiming a few metres down it, and with the
/// route behind the wing line that aim is behind the aircraft: a jet pivots
/// toward it on the apron. LEMG (iMac 2026-09-29, sortie 1790712526, a
/// Citation X): the tug let go 51.5 m down the stand's lane with the nose
/// still on the stand (327.8), the cleared route's first leg was that lane
/// (155.3, the stand to the taxiway) and the taxiway beyond it left on 222.2.
/// No leg ran within 90 degrees of the nose, so the join aimed 16 m down
/// the lane at a point 149 degrees left (`join` err -149, lateral 6.9 m),
/// swung 188 degrees left through the aim and 27 m off the route before it
/// turned back (#190). A heavy airframe that does not pivot (neither
/// brake-steered nor castoring, over [`JOIN_TURN_LIGHT_KG`]) joining with
/// more than [`JOIN_TURN_MIN_DEG`] to turn, off a runway, flies a turn of
/// [`join_turn_radius_m`] from where it stands onto the route AHEAD of it:
/// the arc, then a straight meeting the route within [`JOIN_INTERCEPT_DEG`],
/// the shortest such turn (never the longest cut across an apron), every
/// sample on the drawn
/// pavement (`None` from the field is no verdict). No such turn is a
/// refusal, said. Pure.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct JoinTurn {
    /// Lat/lon from the aircraft round the arc and down the straight to the
    /// join point on the route.
    pub lead: Vec<(f64, f64)>,
    /// +1 a right turn, -1 left.
    pub dir: f64,
    /// The arc, degrees.
    pub turn_deg: f64,
    /// Along the route to the join point, m.
    pub join_m: f64,
    /// The arc plus the straight, m.
    pub lead_m: f64,
    /// The straight's angle onto the route, degrees.
    pub intercept_deg: f64,
}

/// A join needing more than this in heading is planned ([`JoinTurn`]).
pub(crate) const JOIN_TURN_MIN_DEG: f64 = 90.0;
/// At or under this `acf_m_max` an airframe turns on its brakes and nose
/// (the tug's line, `pushback::tug_for_weight`): the pursuit's own join.
pub(crate) const JOIN_TURN_LIGHT_KG: f64 = 5_700.0;
/// The straight meets the route at no more than this.
const JOIN_INTERCEPT_DEG: f64 = 45.0;
/// ...after at least this much straight to settle on.
const JOIN_STRAIGHT_MIN_M: f64 = 8.0;
/// The join point is sought this far along the route past the aircraft.
const JOIN_TURN_REACH_M: f64 = 250.0;
/// An arc past this is a loop, not a turn onto a route.
const JOIN_TURN_MAX_ARC_DEG: f64 = 240.0;
/// Pavement is sampled this often along the planned turn.
const JOIN_SAMPLE_M: f64 = 3.0;
/// No planned turn fits: said, and the drive stands down holding.
const JOIN_REFUSED_LINE: &str = "The route runs behind us and there's no room to turn round \
     onto it from here. Holding. Your aircraft.";

/// The planned join's radius by `acf_m_max`, m: the Citation X turned 27
/// deg/s at 8 kt in that LEMG join (8.7 m) and the 737 turns about 20 deg/s
/// at full lock under 7 kt (about 9 m at 6 kt; the 737 profile's taxi note),
/// so 15 m is a taxi turn with lock to spare; a widebody gets 25 m. Unread
/// reads as the medium jet.
pub(crate) fn join_turn_radius_m(mtow_kg: Option<f64>) -> f64 {
    match mtow_kg {
        Some(m) if m > 100_000.0 => 25.0,
        _ => 15.0,
    }
}

/// Does a join this far off the nose get a planned turn ([`JoinTurn`])?
/// Heavy (or unread), not pivoting, off a runway, a drive that is not a
/// line-up or turn-round (those plan their own geometry). Pure.
pub(crate) fn join_turn_applies(
    err_deg: f64,
    pivots: bool,
    mtow_kg: Option<f64>,
    on_runway: Option<bool>,
    own_geometry: bool,
) -> bool {
    err_deg.abs() > JOIN_TURN_MIN_DEG
        && !pivots
        && mtow_kg.is_none_or(|m| m > JOIN_TURN_LIGHT_KG)
        && on_runway != Some(true)
        && !own_geometry
}

pub(crate) fn join_turn(
    route: &[(f64, f64)],
    lat: f64,
    lon: f64,
    psi: f64,
    radius_m: f64,
    on_pavement: impl Fn(f64, f64) -> Option<bool>,
) -> Option<JoinTurn> {
    if route.len() < 2 || radius_m <= 0.0 {
        return None;
    }
    let geo = taxi_guide::Geo::new(lat);
    let here = (lat, lon);
    let pts: Vec<(f64, f64)> = route.iter().map(|&p| geo.to_m(here, p)).collect();
    let mut cum = vec![0.0_f64];
    for w in pts.windows(2) {
        cum.push(cum.last().unwrap() + taxi_guide::dist(w[0], w[1]));
    }
    let total = *cum.last().unwrap();
    // Where the aircraft projects onto the route (the nearest paint).
    let mut s0 = 0.0;
    let mut best_d = f64::INFINITY;
    for (i, w) in pts.windows(2).enumerate() {
        let (a, b) = (w[0], w[1]);
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let len2 = dx * dx + dy * dy;
        let t = if len2 < 1e-9 {
            0.0
        } else {
            ((-a.0 * dx - a.1 * dy) / len2).clamp(0.0, 1.0)
        };
        let c = (a.0 + t * dx, a.1 + t * dy);
        let d = (c.0 * c.0 + c.1 * c.1).sqrt();
        if d < best_d {
            best_d = d;
            s0 = cum[i] + t * len2.sqrt();
        }
    }
    let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
    let to_ll = |p: (f64, f64)| (lat + p.1 / 111_320.0, lon + p.0 / m_per_deg_lon);
    let (sin_p, cos_p) = (psi.to_radians().sin(), psi.to_radians().cos());
    let r = radius_m;
    let mut best: Option<(f64, JoinTurn)> = None;
    let mut s = s0 + JOIN_STRAIGHT_MIN_M;
    while s <= (s0 + JOIN_TURN_REACH_M).min(total - 1.0) {
        let here_s = s;
        s += 2.0;
        let q = taxi_guide::point_at(&pts, &cum, here_s);
        let i = (1..pts.len()).find(|&i| here_s <= cum[i]).unwrap_or(pts.len() - 1);
        let (a, b) = (pts[i - 1], pts[i]);
        if taxi_guide::dist(a, b) < 0.5 {
            continue;
        }
        let leg = taxi_guide::bearing(a, b);
        for dir in [-1.0_f64, 1.0] {
            // The turn's centre, abeam the nose on the turning side.
            let c = (dir * r * cos_p, -dir * r * sin_p);
            let d = taxi_guide::dist(c, q);
            if d < r * 1.2 {
                continue;
            }
            let bq = taxi_guide::bearing(c, q);
            let alpha = (r / d).acos().to_degrees();
            let b0 = taxi_guide::bearing(c, (0.0, 0.0));
            // Clockwise round the centre the heading is the bearing from it
            // plus 90; anticlockwise, less 90.
            let (arc, hs) = if dir > 0.0 {
                let beta = bq - alpha;
                ((beta - b0).rem_euclid(360.0), beta + 90.0)
            } else {
                let beta = bq + alpha;
                ((b0 - beta).rem_euclid(360.0), beta - 90.0)
            };
            let arc = if arc > 359.0 { 0.0 } else { arc };
            if arc > JOIN_TURN_MAX_ARC_DEG {
                continue;
            }
            let straight = (d * d - r * r).max(0.0).sqrt();
            if straight < JOIN_STRAIGHT_MIN_M {
                continue;
            }
            let intercept = crate::angles::abs_diff_deg(hs.rem_euclid(360.0), leg);
            if intercept > JOIN_INTERCEPT_DEG {
                continue;
            }
            let arc_m = r * arc.to_radians();
            let cost = arc_m + straight;
            if best.as_ref().is_some_and(|(c0, _)| *c0 <= cost) {
                continue;
            }
            // The turn, sampled: round the arc, then down the straight.
            let mut lead = vec![(0.0, 0.0)];
            let n_arc = (arc_m / JOIN_SAMPLE_M).ceil().max(1.0) as usize;
            for k in 1..=n_arc {
                let th = (b0 + dir * arc * k as f64 / n_arc as f64).to_radians();
                lead.push((c.0 + r * th.sin(), c.1 + r * th.cos()));
            }
            let t = *lead.last().unwrap();
            let n_st = (straight / JOIN_SAMPLE_M).ceil().max(1.0) as usize;
            for k in 1..=n_st {
                let f = k as f64 / n_st as f64;
                lead.push((t.0 + (q.0 - t.0) * f, t.1 + (q.1 - t.1) * f));
            }
            if lead.iter().any(|&p| {
                let (plat, plon) = to_ll(p);
                on_pavement(plat, plon) == Some(false)
            }) {
                continue;
            }
            let mut lead_ll: Vec<(f64, f64)> = lead.iter().map(|&p| to_ll(p)).collect();
            lead_ll[0] = here;
            best = Some((
                cost,
                JoinTurn {
                    lead: lead_ll,
                    dir,
                    turn_deg: arc,
                    join_m: here_s,
                    lead_m: arc_m + straight,
                    intercept_deg: intercept,
                },
            ));
        }
    }
    best.map(|(_, j)| j)
}

/// Measurement noise a join's offset may grow by, beyond the distance
/// actually moved, in one tick.
const PROJECTION_JUMP_SLACK_M: f64 = 5.0;

/// True when a join has captured the route: on the line and pointed along it.
pub fn join_done(lateral: f64, err_deg: f64) -> bool {
    lateral < JOIN_DONE_LAT_M && err_deg.abs() < JOIN_DONE_ERR_DEG
}

/// The join's capture latch: [`join_done`], or a capture that has SETTLED
/// (aligned, inside the old 15 m, and no longer closing). Pure; `t_s` is any
/// monotonic clock.
#[derive(Debug, Default, Clone, Copy)]
pub struct JoinCapture {
    best: Option<(f64, f64)>,
}

impl JoinCapture {
    pub fn step(&mut self, lateral: f64, err_deg: f64, t_s: f64) -> bool {
        if join_done(lateral, err_deg) {
            return true;
        }
        match self.best {
            Some((b, _)) if lateral > b - JOIN_PROGRESS_M => {}
            _ => self.best = Some((lateral, t_s)),
        }
        let since = self.best.map_or(t_s, |(_, t)| t);
        lateral < JOIN_SETTLE_MAX_LAT_M
            && err_deg.abs() < JOIN_DONE_ERR_DEG
            && t_s - since >= JOIN_SETTLE_S
    }
}

// ---- Line-up: from the hold-short position onto the runway centreline ----

/// Farther than this from the centreline is not a hold-short position —
/// the same reasoning as [`MAX_JOIN_M`]: the geometry stops meaning
/// "line up" and starts meaning "drive somewhere".
pub(crate) const LINE_UP_MAX_LATERAL_M: f64 = 150.0;
/// Runway that must remain beyond the alignment stop — lined up with less
/// than this ahead is a line-up onto the wrong end.
const LINE_UP_MIN_REMAINING_M: f64 = 400.0;
/// The drive continues this far down the centreline past the entry point,
/// so the nose is pointing down the runway when it stops, not across it.
/// The arrival check is distance-based, so this leg is what buys the
/// alignment — 60 m left the nose 11 degrees off at the stop (2026-07-20).
const LINE_UP_ALIGN_M: f64 = 80.0;

/// The synthetic route of a line-up: from where the aircraft sits, onto the
/// centreline ahead, and down it far enough to finish aligned. `None` when
/// the geometry refuses (too far off, or not enough runway past the stop).
pub fn line_up_path(
    rwy: &crate::autoland::Runway,
    lat: f64,
    lon: f64,
    heading_deg: Option<f64>,
) -> Option<Vec<(f64, f64)>> {
    use crate::autoland::project_latlon;
    let (entry_m, stop_m) = line_up_stops_m(rwy, lat, lon, heading_deg)?;
    if rwy.length_m - stop_m < LINE_UP_MIN_REMAINING_M {
        return None;
    }
    let e = project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, entry_m / M_PER_NM);
    let b = project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, stop_m / M_PER_NM);
    Some(vec![(lat, lon), e, b])
}

/// A backtrack's leg two: the aircraft has turned round ON the strip and
/// points down it already, so it only settles its offset and straightens.
/// It used the hold-short geometry (an entry 30 to 60 m ahead, then
/// [`LINE_UP_ALIGN_M`]) and the Evolution drove 114 m of LILB's 600 m
/// runway after its turn; the takeoff then found 448 m left against 599
/// needed (2026-09-25). At most [`BACKTRACK_SETTLE_MAX_M`] plus
/// [`BACKTRACK_ALIGN_M`].
///
/// An airframe that PIVOTS (`turn.pivots`: a castoring nose or brake
/// steering, whose line-up then ends in [`LineUpAlign`]) settles in less:
/// the entry at twice the offset, then [`PIVOT_SETTLE_M`], the alignment
/// finishing the heading. LILB 03 Evolution (1790721973, 2026-09-30): from
/// the pocket 24.9 m past the numbers and 14.9 m off, the entry was 55 m
/// down and the drive stopped 84 m down, 516.1 m left against 500 (the
/// alignment then crept 40 m more and lost the fit).
pub fn backtrack_leg_two_path(
    rwy: &crate::autoland::Runway,
    lat: f64,
    lon: f64,
    turn: TurnRound,
) -> Option<Vec<(f64, f64)>> {
    use crate::autoland::{along_track_nm, cross_track_nm, project_latlon};
    let lateral = (cross_track_nm(lat, lon, rwy) * M_PER_NM).abs();
    if lateral > LINE_UP_MAX_LATERAL_M {
        return None;
    }
    let a0 = (-along_track_nm(lat, lon, rwy) * M_PER_NM).max(0.0);
    let (entry_m, stop_m) = if turn.pivots {
        let entry = a0 + (lateral * 2.0).clamp(PIVOT_ENTRY_MIN_M, BACKTRACK_SETTLE_MAX_M);
        (entry, entry + PIVOT_SETTLE_M)
    } else {
        let entry = a0 + (lateral * 4.0).clamp(BACKTRACK_SETTLE_MIN_M, BACKTRACK_SETTLE_MAX_M);
        (entry, entry + BACKTRACK_ALIGN_M)
    };
    if rwy.length_m - stop_m < LINE_UP_MIN_REMAINING_M.min(rwy.length_m * 0.5) {
        return None;
    }
    let e = project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, entry_m / M_PER_NM);
    let b = project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, stop_m / M_PER_NM);
    Some(vec![(lat, lon), e, b])
}
const BACKTRACK_SETTLE_MIN_M: f64 = 10.0;
const BACKTRACK_SETTLE_MAX_M: f64 = 30.0;
const BACKTRACK_ALIGN_M: f64 = 30.0;
/// A pivoting airframe's leg two: the entry at least this far down from
/// where it turned, and this much centreline after it (the route's end:
/// the drive stops [`STOP_SHORT_M`] before it).
const PIVOT_ENTRY_MIN_M: f64 = 10.0;
const PIVOT_SETTLE_M: f64 = 12.0;

/// The way a backtrack's turn-round turns: TOWARD the centreline, across
/// the strip, the way the teardrop pocket was laid out for (+1 right, -1
/// left); `None` on the centreline itself. The pocket leg arrives pointing
/// a little OUTWARD, so the shorter way round is outward, and the drive
/// took it: LILB 03 Evolution (1790721973, 2026-09-30) stood 14.9 m right
/// of the centreline heading 172 for a 024 runway, turned left through
/// 113 and 44 with the aircraft carried out to 21.6 m, in the grass of a
/// 15 m strip, and crossed back 58 degrees off. Pure.
pub fn turn_round_hint(rwy: &crate::autoland::Runway, lat: f64, lon: f64) -> Option<f64> {
    let ct_m = crate::autoland::cross_track_nm(lat, lon, rwy) * M_PER_NM;
    (ct_m.abs() >= TURN_HINT_MIN_OFFSET_M).then(|| ct_m.signum())
}
const TURN_HINT_MIN_OFFSET_M: f64 = 1.0;

/// How a backtrack turns round at the numbers ([`backtrack_pocket_path`],
/// [`backtrack_leg_two_path`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TurnRound {
    /// The airframe pivots on a braked wheel: a castoring nose or brake
    /// steering (the line-up then ends in [`LineUpAlign`]).
    pub pivots: bool,
    /// The strip's pavement width, m, when apt.dat gave one.
    pub width_m: Option<f64>,
}

impl TurnRound {
    /// The teardrop every airframe flew before: room for a steered nose.
    #[cfg(test)]
    pub const TEARDROP: TurnRound = TurnRound { pivots: false, width_m: None };

    /// Where the turn begins down the strip, where the pocket sits down it
    /// and how far off the centreline, m. A pivoting airframe turns nearer
    /// the numbers and on the PAVEMENT: the teardrop's 12 m off put the
    /// Evolution's pocket in the grass beside LILB's 15 m strip, 30 m down
    /// (1790721973, 2026-09-30).
    pub fn pocket(&self) -> (f64, f64, f64) {
        if !self.pivots {
            return (
                BACKTRACK_TURN_START_M,
                BACKTRACK_TURN_START_M / 2.0,
                BACKTRACK_POCKET_OFFSET_M,
            );
        }
        let offset = self.width_m.map_or(PIVOT_POCKET_OFFSET_M, |w| {
            (w / 2.0 - PIVOT_POCKET_EDGE_M)
                .clamp(PIVOT_POCKET_MIN_OFFSET_M, BACKTRACK_POCKET_OFFSET_M)
        });
        (PIVOT_TURN_START_M, PIVOT_POCKET_ALONG_M, offset)
    }
}
/// A pivoting airframe's pocket: the turn begins this far down, the pocket
/// this far down (the drive has been seen to run 5 to 10 m past it at the
/// backtrack's pace), this far off the centreline with no width known, and
/// this far inside the pavement's edge with one, never nearer the
/// centreline than the minimum.
///
/// The edge is the AIRCRAFT's centre: the outer main wheel sits half the
/// track further out (about 1.6 m on a GA single), and the pocket leg stops
/// a metre off its line. At 2 m the Evolution's pocket on LILB's 15 m strip
/// was 5.5 m off; leg one stopped at 6.5 (1790732094, 2026-09-30), the
/// outer wheel about 8.1 m out against the 7.5 m edge, and the turn-round
/// swung the wrong way about it, stuck, and broke away at 9.5 kt. From 5.1 m
/// (1790724857) the same turn worked. So 3.5 m: the minimum's 4 m there.
const PIVOT_TURN_START_M: f64 = 50.0;
const PIVOT_POCKET_ALONG_M: f64 = 25.0;
const PIVOT_POCKET_OFFSET_M: f64 = 6.0;
const PIVOT_POCKET_EDGE_M: f64 = 3.5;
const PIVOT_POCKET_MIN_OFFSET_M: f64 = 4.0;

/// Where a plain line-up from this position would enter and stop (meters
/// past the threshold), or `None` when the position isn't a hold-short
/// geometry at all. Shared by the path builder and the full-length check.
/// `heading_deg` (the aircraft's current heading) makes leg one roll STRAIGHT
/// FORWARD along the taxiway lead-on onto the centreline instead of cutting a
/// diagonal across to it (the "beeline" the pilot saw, 2026-07-23); `None`
/// keeps the old diagonal (backtrack leg two, tests).
pub fn line_up_stops_m(
    rwy: &crate::autoland::Runway,
    lat: f64,
    lon: f64,
    heading_deg: Option<f64>,
) -> Option<(f64, f64)> {
    use crate::autoland::{along_track_nm, cross_track_nm};
    let x0 = cross_track_nm(lat, lon, rwy) * M_PER_NM; // signed, right of course +
    let lateral = x0.abs();
    if lateral > LINE_UP_MAX_LATERAL_M {
        return None;
    }
    // Signed distance past the threshold (positive = down the runway).
    let a0 = -along_track_nm(lat, lon, rwy) * M_PER_NM;
    // Roll straight forward along the current heading onto the centreline —
    // the taxiway lead-on line — when the heading gives a clean crossing.
    // Otherwise the old fallback: enter ahead of abeam by ~the lateral offset
    // (a ~45 degree swing), floored at one turn's worth and capped so a far-set
    // hold (LXGB's Echo ~90 m off the strip) gets a sharper swing, not a 90 m lead.
    let entry_m = roll_forward_entry_m(rwy.course_deg, a0, x0, heading_deg)
        .unwrap_or_else(|| a0.max(0.0) + lateral.clamp(30.0, 60.0));
    Some((entry_m, entry_m + LINE_UP_ALIGN_M))
}

/// Distance past the threshold where the aircraft's heading, rolled straight
/// forward, crosses the centreline — following the taxiway lead-on onto the
/// runway. `None` (use the diagonal fallback) when the heading is within ~15
/// degrees of the runway (no clean crossing), points away from the centreline,
/// crosses behind the threshold, or would run implausibly far.
fn roll_forward_entry_m(
    course_deg: f64,
    a0: f64,
    x0: f64,
    heading_deg: Option<f64>,
) -> Option<f64> {
    let psi = heading_deg?;
    let rel = crate::angles::signed_diff_deg(psi, course_deg).to_radians();
    let s = rel.sin();
    if s.abs() < 0.26 {
        return None; // within ~15 degrees of the runway: no clean forward crossing
    }
    let fwd = -x0 / s; // meters forward to reach cross-track zero
    if fwd <= 0.0 || fwd > 150.0 {
        return None; // heading away from the centreline, or too shallow / far
    }
    let entry = a0 + fwd * rel.cos();
    (entry >= 0.0).then_some(entry)
}

/// A line-up riding the AUTHORED lead-on: the trimmed tail of the cleared taxi
/// route — the painted yellow line from the holding point onto the runway
/// centreline — instead of a synthetic diagonal (the aircraft cut the Echo
/// stub's curve and stopped crooked, LXGB 2026-07-24). Returns the path and
/// its centreline stop (m past the threshold). `None` when the lead-on isn't
/// THIS hold or THIS runway: it must start near the aircraft and end on the
/// centreline with runway enough beyond the stop.
/// How far from the centreline a lead-on may end and still be on the runway
/// (half the widest strip, 60 m, plus slack), and how far before the end.
const LEAD_ON_JUNCTION_XT_M: f64 = 35.0;
const LEAD_ON_BEHIND_END_M: f64 = 30.0;

pub fn line_up_path_from_lead_on(
    rwy: &crate::autoland::Runway,
    lat: f64,
    lon: f64,
    lead_on: &[(f64, f64)],
) -> Option<(Vec<(f64, f64)>, f64)> {
    use crate::autoland::{along_track_nm, cross_track_nm, project_latlon};
    let (&first, &last) = (lead_on.first()?, lead_on.last()?);
    // OURS, not another stub's: compare ALONG-TRACK positions, not straight-
    // line distance — the stub runs perpendicular to the runway, so a legit
    // hold sits ~90 m from its own junction (LXGB Echo: 92 m; the old 80 m
    // straight-line guard silently rejected it and the diagonal came back,
    // 2026-07-24, probed). A foreign stub is displaced ALONG the runway.
    // Every rejection says which test and the numbers: a remote hold at
    // LIME 28 (200 m off the centreline) was refused as "can't find runway
    // 28" with nothing to say whether the lead-on had failed and why
    // (2026-09-25).
    let reject = |why: &str, v: f64| {
        crate::blackbox::record(
            "autotaxi",
            serde_json::json!({"event": "lead_on_rejected", "rwy": rwy.id, "why": why, "value": v, "pts": lead_on.len()}),
        );
        None
    };
    let a_air = -along_track_nm(lat, lon, rwy) * M_PER_NM;
    let a_first = -along_track_nm(first.0, first.1, rwy) * M_PER_NM;
    if (a_first - a_air).abs() > 80.0 {
        return reject("not_this_hold", a_first - a_air);
    }
    // ...and ends ON this runway's pavement (the junction node): a
    // connector that meets the runway at its end can stop at the edge of a
    // 45 m strip, not on the centreline; the fillet below finishes it.
    let junction_xt = (cross_track_nm(last.0, last.1, rwy) * M_PER_NM).abs();
    if junction_xt > LEAD_ON_JUNCTION_XT_M {
        return reject("junction_off_the_runway", junction_xt);
    }
    // A junction a little before the end is the end: a hold beside the
    // runway's first metres (and a displaced threshold is usable for
    // takeoff).
    let junction_a = -along_track_nm(last.0, last.1, rwy) * M_PER_NM;
    if junction_a < -LEAD_ON_BEHIND_END_M {
        return reject("junction_behind_the_end", junction_a);
    }
    let junction_a = junction_a.max(0.0);
    let stop_m = junction_a + LINE_UP_ALIGN_M;
    if rwy.length_m - stop_m < LINE_UP_MIN_REMAINING_M {
        return reject("too_little_runway_left", rwy.length_m - stop_m);
    }
    // Aircraft, then the authored points (skipping any we're already on top
    // of), then a FILLET ARC around the junction corner onto the centreline,
    // then the alignment stop. The network models the entry as a right angle;
    // the paint curves — a sampled arc where the sharp corner was is what the
    // pursuit can actually follow (it chorded the raw corner, probed
    // 2026-07-24).
    let mut path = vec![(lat, lon)];
    path.extend(lead_on_ahead(lead_on, lat, lon));
    let stop = project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, stop_m / M_PER_NM);
    let corner = path.pop()?; // the junction (or the aircraft itself — refused below)
    if path.is_empty() {
        return reject("already_at_the_junction", 0.0);
    }
    path.extend(fillet_corner(*path.last()?, corner, stop, 18.0));
    path.push(stop);
    (path.len() >= 3).then_some((path, stop_m))
}

/// The lead-on's points still AHEAD of the aircraft and more than 5 m from
/// it (one we are on top of is not a point to drive to), for the line-up
/// and the backtrack. A taxi that rolls through its holding stop (LEMG 31,
/// 2026-09-28: `past_end_ticks` 36, stopped 7.4 m past the hold node) left
/// the node BEHIND the aircraft, and the path ran back to it and forward
/// again through the aircraft: at rest the joining projection read 14.8 m
/// along (the forward pass) and the plain one 0 m (the leg back), and
/// `remaining_m` jumped 14.9 m between two ticks with nothing moving.
fn lead_on_ahead(lead_on: &[(f64, f64)], lat: f64, lon: f64) -> Vec<(f64, f64)> {
    let dist_m = |a: (f64, f64), b: (f64, f64)| {
        crate::autoland::haversine_nm(a.0, a.1, b.0, b.1) * M_PER_NM
    };
    let passed_m = crate::taxi_guide::project_signed(lead_on, lat, lon).map_or(0.0, |(a, _)| a);
    let mut along = 0.0;
    let mut prev = lead_on.first().copied();
    lead_on
        .iter()
        .copied()
        .filter(|&p| {
            along += prev.map_or(0.0, |q| dist_m(q, p));
            prev = Some(p);
            along > passed_m - 1.0 && dist_m((lat, lon), p) > 5.0
        })
        .collect()
}

/// A sampled quadratic-Bezier fillet replacing the sharp vertex at `corner`:
/// starts `r_m` back along the inbound leg, ends `r_m` down the outbound leg,
/// bulging toward the corner — the painted lead-on curve the taxi networks
/// don't model. Radii clamp to half of each leg so short legs stay sane.
fn fillet_corner(
    prev: (f64, f64),
    corner: (f64, f64),
    next: (f64, f64),
    r_m: f64,
) -> Vec<(f64, f64)> {
    let seg = |a: (f64, f64), b: (f64, f64), t: f64| (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t);
    let len =
        |a: (f64, f64), b: (f64, f64)| crate::autoland::haversine_nm(a.0, a.1, b.0, b.1) * M_PER_NM;
    let l_in = len(prev, corner);
    let l_out = len(corner, next);
    if l_in < 1.0 || l_out < 1.0 {
        return vec![corner];
    }
    let a = seg(corner, prev, (r_m.min(l_in * 0.5)) / l_in); // arc entry, on the inbound leg
    let b = seg(corner, next, (r_m.min(l_out * 0.5)) / l_out); // arc exit, on the outbound leg
                                                               // Quadratic Bezier a -> corner(control) -> b, sampled.
    let bez = |t: f64| {
        let u = 1.0 - t;
        (
            u * u * a.0 + 2.0 * u * t * corner.0 + t * t * b.0,
            u * u * a.1 + 2.0 * u * t * corner.1 + t * t * b.1,
        )
    };
    vec![a, bez(0.25), bez(0.5), bez(0.75), b]
}

/// A backtrack pocket sits this far past the threshold along the runway —
/// room for the turnaround to finish at the numbers.
const BACKTRACK_TURN_START_M: f64 = 60.0;
/// ...and this far off the centreline (right of the backtrack direction),
/// so the 180 is a teardrop with turning room, never a pivot on the spot.
const BACKTRACK_POCKET_OFFSET_M: f64 = 12.0;
/// Closer to the threshold than this, a backtrack is pointless — the plain
/// line-up already stops within a couple of plane-lengths of the numbers.
const BACKTRACK_MIN_M: f64 = 150.0;

/// Leg one of a backtrack: from the hold, onto the centreline heading TOWARD
/// the departure threshold, down the strip, ending in a turn pocket beside
/// the numbers. Leg two is a plain [`line_up_path`] from the pocket — the
/// join machinery already handles "facing the wrong way", and splitting the
/// maneuver keeps each route free of overlapping legs (a single out-and-back
/// polyline is ambiguous to project onto). Real Gibraltar: the taxiways only
/// reach the east half, so a full-length 09 departure backtracks ~1.2 km
/// (2026-07-22, held at Charlie with 580 m ahead).
pub fn backtrack_pocket_path(
    rwy: &crate::autoland::Runway,
    lat: f64,
    lon: f64,
    turn: TurnRound,
) -> Option<Vec<(f64, f64)>> {
    let (turn_start_m, pocket_m, pocket_off_m) = turn.pocket();
    use crate::autoland::{along_track_nm, cross_track_nm, project_latlon};
    let lateral = (cross_track_nm(lat, lon, rwy) * M_PER_NM).abs();
    if lateral > LINE_UP_MAX_LATERAL_M {
        return None;
    }
    let past_m = (-along_track_nm(lat, lon, rwy) * M_PER_NM).max(0.0);
    if past_m < BACKTRACK_MIN_M {
        return None;
    }
    let c = |d_m: f64| project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, d_m / M_PER_NM);
    // Enter the runway swinging toward the threshold (the mirror of the
    // line-up's entry lead), never inside the turn pocket itself.
    let entry_m = (past_m - lateral.clamp(30.0, 60.0)).max(turn_start_m + 30.0);
    let entry = c(entry_m);
    let turn_start = c(turn_start_m);
    // Swing the teardrop toward the side the aircraft CAME IN on — the taxiway/
    // apron side, which is the way the painted turn-pad leads. A fixed hand
    // turned the wrong way against the yellow line at LXGB 09 (2026-07-26).
    // `cross_track` is signed (+ = right of the runway course); default to the
    // right of travel when we're dead on the centreline.
    let entry_ct_m = cross_track_nm(lat, lon, rwy) * M_PER_NM;
    let pocket_bearing = if entry_ct_m >= 0.0 {
        rwy.course_deg + 90.0
    } else {
        rwy.course_deg - 90.0
    };
    let mid = c(pocket_m);
    let pocket = project_latlon(mid.0, mid.1, pocket_bearing, pocket_off_m / M_PER_NM);
    Some(vec![(lat, lon), entry, turn_start, pocket])
}

/// The same backtrack, entering the runway along the AUTHORED lead-on — the
/// painted line from the holding point onto the centreline — instead of a
/// diagonal aimed at a point down the strip.
///
/// The plain line-up learned this in 2026-07-24 ([`line_up_path_from_lead_on`]);
/// the backtrack kept its diagonal and it put an MD-82 in the grass at Luton
/// (2026-07-27). Its leg one ran from the holding point to a centreline point
/// 30 to 60 m nearer the threshold — 23 degrees off the stub the aircraft was
/// standing on — so the drive left the pavement within fifty metres, bogged,
/// and the governor called it stagnant with the aircraft off the taxiway. The
/// entry is not ours to invent: the stub is drawn, and the only question is
/// which way we turn at the top of it.
pub fn backtrack_pocket_path_from_lead_on(
    rwy: &crate::autoland::Runway,
    lat: f64,
    lon: f64,
    lead_on: &[(f64, f64)],
    turn: TurnRound,
) -> Option<Vec<(f64, f64)>> {
    use crate::autoland::{along_track_nm, cross_track_nm, project_latlon};
    let (turn_start_m, pocket_m, pocket_off_m) = turn.pocket();
    let (&first, &last) = (lead_on.first()?, lead_on.last()?);
    // OURS, and ending on THIS centreline — the same two tests the plain
    // lead-on line-up applies, for the same reason: a foreign stub is
    // displaced along the runway, not merely far away.
    let a_air = -along_track_nm(lat, lon, rwy) * M_PER_NM;
    if (-along_track_nm(first.0, first.1, rwy) * M_PER_NM - a_air).abs() > 80.0 {
        return None;
    }
    if (cross_track_nm(last.0, last.1, rwy) * M_PER_NM).abs() > 20.0 {
        return None;
    }
    // Same "is a backtrack worth it" test as the synthetic version — measured
    // from the AIRCRAFT, not the junction: near the numbers the plain line-up
    // already stops within a plane-length or two of them. And the junction has
    // to sit beyond the turn pocket, or there is nothing to drive back along.
    if a_air < BACKTRACK_MIN_M {
        return None;
    }
    let junction_a = -along_track_nm(last.0, last.1, rwy) * M_PER_NM;
    if junction_a < turn_start_m + 30.0 {
        return None;
    }
    let c = |d_m: f64| project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, d_m / M_PER_NM);
    let turn_start = c(turn_start_m);
    let entry_ct_m = cross_track_nm(lat, lon, rwy) * M_PER_NM;
    let pocket_bearing = if entry_ct_m >= 0.0 {
        rwy.course_deg + 90.0
    } else {
        rwy.course_deg - 90.0
    };
    let mid = c(pocket_m);
    let pocket = project_latlon(mid.0, mid.1, pocket_bearing, pocket_off_m / M_PER_NM);
    // Aircraft, up the drawn stub, a fillet around the junction (the network
    // draws a right angle, the paint curves), then down the strip to the
    // pocket.
    let mut path = vec![(lat, lon)];
    path.extend(lead_on_ahead(lead_on, lat, lon));
    let corner = path.pop()?;
    if path.is_empty() {
        return None;
    }
    path.extend(fillet_corner(*path.last()?, corner, turn_start, 18.0));
    path.push(turn_start);
    path.push(pocket);
    (path.len() >= 4).then_some(path)
}

/// How a line-up will be driven: straight on, or the full-length backtrack.
pub enum LineUpPlan {
    Plain(Vec<(f64, f64)>),
    Backtrack(Vec<(f64, f64)>),
}

/// Pick the runway AND the maneuver: a plain line-up when the runway ahead
/// of its stop covers `needed_m`, otherwise a backtrack to the threshold
/// when the geometry offers one (`force_backtrack` skips straight to it —
/// the pilot asked for the full length in so many words).
#[allow(clippy::too_many_arguments)]
pub fn pick_line_up_plan(
    cands: &[crate::autoland::Runway],
    lat: f64,
    lon: f64,
    cleared: Option<&str>,
    needed_m: f64,
    force_backtrack: bool,
    heading_deg: Option<f64>,
    lead_on: Option<&[(f64, f64)]>,
    turn: TurnRound,
) -> Option<(crate::autoland::Runway, LineUpPlan)> {
    let mut best: Option<(crate::autoland::Runway, LineUpPlan, f64)> = None;
    for r in cands {
        let turn = TurnRound {
            width_m: turn.width_m.or_else(|| crate::autoland::runway_width_m(r)),
            ..turn
        };
        if let Some(c) = cleared {
            if !crate::autotakeoff::runway_matches(&r.id, c) {
                continue;
            }
        }
        // Already ON the strip and pointing down it (a turn-round or a
        // line-up that stopped off the line): settle onto the centreline in
        // the least runway, as a backtrack's leg two does.
        if !force_backtrack {
            if let Some((p, stop_m)) = on_strip_settle_path(r, lat, lon, heading_deg, turn) {
                if r.length_m - stop_m >= needed_m {
                    if best.as_ref().is_none_or(|(_, _, d)| r.dist_nm < *d) {
                        best = Some((r.clone(), LineUpPlan::Plain(p), r.dist_nm));
                    }
                    continue;
                }
            }
        }
        // The authored lead-on wins when it belongs to this runway and this
        // hold — the painted line IS the maneuver. Synthetic geometry is the
        // fallback (no published route, wrong stub, backtracks).
        let via_lead = if force_backtrack {
            None
        } else {
            lead_on.and_then(|lo| line_up_path_from_lead_on(r, lat, lon, lo))
        };
        let (plain, stop_m) = match via_lead {
            Some((p, s)) => (Some(p), Some(s)),
            None => (
                line_up_path(r, lat, lon, heading_deg),
                line_up_stops_m(r, lat, lon, heading_deg).map(|(_, stop)| stop),
            ),
        };
        let ahead = stop_m.map(|s| r.length_m - s);
        // Use the FULL length for a departure: if the line-up entry sits well
        // past the numbers (a deep intersection like LXGB's Charlie, which
        // enters the middle of 09) on a short/medium runway, backtrack to the
        // threshold rather than accept the reduced length — even when the roll
        // would fit. A departure lines up at the START of the runway; the pilot
        // shouldn't have to ask (2026-07-26). Long runways keep intersection
        // departures (backtracking 3 km of active runway is not the move).
        const DEEP_INTERSECTION_M: f64 = 200.0;
        const FULL_LENGTH_MATTERS_M: f64 = 2500.0;
        let deep_intersection =
            stop_m.is_some_and(|s| s > DEEP_INTERSECTION_M) && r.length_m <= FULL_LENGTH_MATTERS_M;
        let wants_backtrack = force_backtrack
            || ahead.is_none_or(|a| a < needed_m)
            || plain.is_none()
            || deep_intersection;
        let plan = if wants_backtrack {
            // The drawn stub first, here too — a backtrack still has to get
            // ONTO the runway, and inventing that entry is what put an
            // aircraft in the grass (see `backtrack_pocket_path_from_lead_on`).
            let via_lead_bt = lead_on
                .and_then(|lo| backtrack_pocket_path_from_lead_on(r, lat, lon, lo, turn))
                .or_else(|| backtrack_pocket_path(r, lat, lon, turn));
            match via_lead_bt {
                Some(p) => Some(LineUpPlan::Backtrack(p)),
                None => plain.map(LineUpPlan::Plain),
            }
        } else {
            plain.map(LineUpPlan::Plain)
        };
        let Some(plan) = plan else { continue };
        if best.as_ref().is_none_or(|(_, _, d)| r.dist_nm < *d) {
            best = Some((r.clone(), plan, r.dist_nm));
        }
    }
    best.map(|(r, p, _)| (r, p))
}

/// A line-up asked for from ON the strip, the nose within
/// [`ON_STRIP_MAX_ERR_DEG`] of its course: the route of a backtrack's leg two
/// from here ([`backtrack_leg_two_path`]) and where it stops, m past the
/// threshold. `None` off the pavement (half the width, else
/// [`ON_STRIP_HALF_WIDTH_M`]), short of the threshold or pointing elsewhere.
/// LILB 03 Evolution (1790732094, 2026-09-30): the turn-round stopped 41 m
/// down, 5.6 m off and 14 degrees crooked; the hold-short geometry from
/// there enters 30 to 60 m further on and leaves less than the 500 m fit,
/// so the one retry a failed line-up gets would have been hopeless. Pure.
pub fn on_strip_settle_path(
    rwy: &crate::autoland::Runway,
    lat: f64,
    lon: f64,
    heading_deg: Option<f64>,
    turn: TurnRound,
) -> Option<(Vec<(f64, f64)>, f64)> {
    use crate::autoland::{along_track_nm, cross_track_nm};
    let hdg = heading_deg?;
    let err = crate::angles::abs_diff_deg(hdg, rwy.course_deg);
    if err > ON_STRIP_MAX_ERR_DEG {
        return None;
    }
    let half = turn.width_m.map_or(ON_STRIP_HALF_WIDTH_M, |w| w / 2.0);
    let ct = (cross_track_nm(lat, lon, rwy) * M_PER_NM).abs();
    let a0 = -along_track_nm(lat, lon, rwy) * M_PER_NM;
    if ct > half || a0 < 0.0 || a0 > rwy.length_m {
        return None;
    }
    let path = backtrack_leg_two_path(rwy, lat, lon, turn)?;
    let end = *path.last()?;
    let stop_m = -along_track_nm(end.0, end.1, rwy) * M_PER_NM;
    Some((path, stop_m))
}
/// Pointing down the strip, for [`on_strip_settle_path`].
pub const ON_STRIP_MAX_ERR_DEG: f64 = 45.0;
/// On it, with no width known.
const ON_STRIP_HALF_WIDTH_M: f64 = 10.0;

/// Pick the runway end to line up on: feasible geometry, matching the
/// cleared runway when the tower named one (lining up on 09 with a 27
/// clearance is the backwards trap, refused upstream by returning the
/// match only). Nearest threshold among the feasible wins.
pub fn pick_line_up(
    cands: &[crate::autoland::Runway],
    lat: f64,
    lon: f64,
    cleared: Option<&str>,
    heading_deg: Option<f64>,
) -> Option<(crate::autoland::Runway, Vec<(f64, f64)>)> {
    type LineUpCandidate = (crate::autoland::Runway, Vec<(f64, f64)>, f64);
    let mut best: Option<LineUpCandidate> = None;
    for r in cands {
        if let Some(c) = cleared {
            if !crate::autotakeoff::runway_matches(&r.id, c) {
                continue;
            }
        }
        let Some(path) = line_up_path(r, lat, lon, heading_deg) else {
            continue;
        };
        if best.as_ref().is_none_or(|(_, _, d)| r.dist_nm < *d) {
            best = Some((r.clone(), path, r.dist_nm));
        }
    }
    best.map(|(r, p, _)| (r, p))
}

/// Pure-pursuit lookahead: seconds of taxi ahead, floored so creep-speed
/// taxiing still aims somewhere and capped so the aim point can't cut a
/// corner two junctions away. 5 s, not 3.5: at 14 m of lookahead a 3 m
/// centreline offset read as 12° of error and half-lock steer — the SR22
/// S-weaved ±4 m for whole legs even with the rate lead (2026-07-22).
/// Longer aim turns the same offset into ~8° and a quarter-lock nudge.
/// Test-only since the live loop took the aim from the profile
/// ([`Pace::lookahead_s`]); the tests pin this proven 5 s baseline.
#[cfg(test)]
pub fn lookahead_m(gs_kts: f64) -> f64 {
    lookahead_m_with(gs_kts, 5.0)
}

/// [`lookahead_m`] with the airframe's own aim time. A slow plant needs a
/// LONG aim: the SF50 (castoring wheel, brake-assisted, ~3.5 deg/s of yaw at
/// full command) crossed the centreline 24 degrees ANGLED at 12 kt with the
/// 5 s aim — it cannot turn fast enough to roll out in 31 m, so every
/// crossing overshot ~10 m the other way, a slow drunk S down the taxiway
/// (2026-07-29). Aim further out and the same offset asks for a shallower
/// crossing the plant can actually fly. The cap scales with the aim so a
/// long-aim airframe keeps it at speed.
pub fn lookahead_m_with(gs_kts: f64, aim_s: f64) -> f64 {
    (gs_kts * 0.514 * aim_s).clamp(16.0, 10.0 * aim_s)
}

/// Pure pursuit on a curve cuts inside it by about L^2 / 2R. A long aim in a
/// GENTLE bend is where that bites: the Evolution's 9 s aim at 12 kt is 55 m,
/// and a 107 m radius bend at LIME put it 8.5 m inside the line with the
/// steering near zero, 13 m on another (2026-09-25). The aim is capped so the
/// geometric cut stays under [`BEND_MAX_CUT_M`]; never under the corner
/// floor ([`BEND_AIM_FLOOR_M`], 16 m; a free castoring nose flies `ground_steer`), and
/// straights are exactly as they were.
/// Test-only since the live loop takes the floor per nose
/// ([`steer_look_m`]); the tests pin the 16 m baseline.
#[cfg(test)]
pub fn bend_lookahead_cap_m(gs_kts: f64, turn_dps: f64) -> f64 {
    bend_lookahead_cap_floor_m(gs_kts, turn_dps, BEND_AIM_FLOOR_M)
}

/// [`bend_lookahead_cap_m`] with the corner's own floor.
///
/// The corner is its radius, v over the route's turn rate, and that is the
/// same at any speed the rate was read at: under 0.5 m/s the "straight"
/// threshold scales with the speed (the same 143 m radius) instead of the
/// cap vanishing. It vanished under a knot, and a free nose recovering from
/// a swing at 0.8 to 1.1 kt aimed 12 m and 16 m on alternate ticks: the
/// error read -29 to -38 degrees, then -15 to -20, and the turn builder's
/// brake went 0.06 to 0.07, nothing, 0.07 (LEMG, 1790736973, 2026-09-30).
/// Only a free nose notices: every other nose's crawl aim is its 16 m floor
/// either way.
pub fn bend_lookahead_cap_floor_m(gs_kts: f64, turn_dps: f64, floor_m: f64) -> f64 {
    let v = gs_kts * 0.514_444;
    if v <= 0.0 || turn_dps.abs() < BEND_STRAIGHT_DPS * (v / BEND_STRAIGHT_FROM_MS).min(1.0) {
        return f64::INFINITY;
    }
    let radius_m = v / turn_dps.abs().to_radians();
    (2.0 * radius_m * BEND_MAX_CUT_M).sqrt().max(floor_m)
}
/// Under this turn rate the route is straight, from this speed up; below
/// it the rate that is the same radius.
const BEND_STRAIGHT_DPS: f64 = 0.2;
const BEND_STRAIGHT_FROM_MS: f64 = 0.5;
/// The most a gentle bend may be cut, m.
pub const BEND_MAX_CUT_M: f64 = 1.5;
/// The shortest aim a corner is flown on: a shorter one asks for a turn
/// tighter than a nosewheel's lock can fly, and the aircraft runs wide.
pub const BEND_AIM_FLOOR_M: f64 = 16.0;
/// Candidate shorter aim for linked noses through tight bends. EGBS Bravo
/// recorded a 7.6 m inside-corner cut; the fixed 16 m floor is a suspected
/// contributor. Exact-route replay and live confirmation remain necessary.
pub const LINKED_BEND_AIM_FLOOR_M: f64 = 8.0;
/// The pure-pursuit aim for one tick, metres: the airframe's aim time,
/// capped in a bend ([`bend_lookahead_cap_floor_m`] over
/// [`BEND_PROBE_M`]) and by what is left of the route. `turn_probe_dps` is
/// `Route::route_turn_rate_dps(progress, BEND_PROBE_M, gs_kts)`.
///
/// The route cap: near the stop the pursuit point must walk the path
/// through the final corner, not chord across it. The 16 m minimum aim
/// against a corner ~20 m before the hold parked the aircraft off the paint,
/// and stop_straighten then froze the leftover angle (LXGB Echo,
/// 2026-07-24, twice). Only bites close-in, where the stop ramp has already
/// slowed us below weave speeds.
///
/// A free castoring nose no longer steers on this aim (`ground_steer`
/// flies its filleted path); the aim still gives its drive the error the
/// join and the guards read, and the route's turn rate the scorer reads.
pub fn steer_look_m(
    pace: &Pace,
    gs_kts: f64,
    turn_probe_dps: f64,
    remaining_m: f64,
    free_nose: bool,
) -> f64 {
    let bend_floor = if free_nose || pace.brake_steer {
        BEND_AIM_FLOOR_M
    } else {
        LINKED_BEND_AIM_FLOOR_M
    };
    lookahead_m_with(gs_kts, pace.lookahead_s)
        .min(bend_lookahead_cap_floor_m(gs_kts, turn_probe_dps, bend_floor))
        .min((remaining_m * 0.6).max(6.0))
}
/// The route span the bend's curvature is read over, m.
pub const BEND_PROBE_M: f64 = 25.0;

/// Deceleration the stop ramp plans for, m/s².
///
/// NOT the deceleration the brakes can achieve — that is about 0.7 m/s² at the
/// gentle end (measured: 0.14-0.18 brake gave a -1.36 kt/s trend at EGHI). It
/// is deliberately far below it, because the ramp has to command a speed the
/// governor will actually ACT on, and `taxi_control` only brakes once the
/// aircraft is 3 kt above target. A √ curve is flat far out and steep near
/// zero, so at 0.5 that gap did not open until ~7 m to go:
///
///   rem 22.9 m -> target 9.3 kt vs 9.54 actual   no brake
///   rem 11.2 m -> target 6.5 kt vs 9.24 actual   no brake
///   rem  6.8 m -> target 5.1 kt vs 9.14 actual   first brake, far too late
///
/// The aircraft then crossed the hold line at 6.7 kt and coasted five metres
/// past it, nose over the yellow — a runway incursion, and with the steering
/// already released it stopped skewed as well (EGHI, photographed 2026-07-30).
///
/// Worse, `taxi_control` cannot tell a planned STOP from a bend trim: both are
/// just a target above 1 kt, so both get the 0.22 comfort cap that three
/// airframes' jerkiness complaints put there. Full 0.6 authority arrives only
/// inside the last metre. Rather than re-open that tuning, this starts the
/// deceleration early enough that comfort braking is enough: at 0.25 the same
/// 22.9 m commands 6.6 kt, the 3 kt gap opens around 15 m out, and there is
/// then twice the distance to stop in with ~2.8x the needed authority.
const STOP_DECEL_MS2: f64 = 0.25;

/// The braking-distance speed cap approaching the stop point: v = √(2·a·d) at
/// [`STOP_DECEL_MS2`], floored at a creep so the last meters actually finish.
/// At/inside [`STOP_ZERO_M`] it's a stop.
pub fn stop_ramp_kts(remaining_m: f64) -> f64 {
    if remaining_m <= STOP_ZERO_M {
        return 0.0;
    }
    (MS_TO_KTS * (2.0 * STOP_DECEL_MS2 * remaining_m).sqrt()).max(CREEP_KTS)
}

/// Where the schedule commands a full stop, and the creep it will not go below
/// on the way in. These two are a PAIR, and getting them out of step is what
/// made the SF50 unable to line up at Luton (2026-07-28).
///
/// The old figures were a 2 kt creep held until half a metre to go, against an
/// arrival test of `remaining <= 1 m && gs < 1.5 kt`. So the schedule was still
/// DRIVING at 2 kt through the window the arrival test needed it under 1.5, and
/// 2 kt needs `v^2/2a` = 1.06 m to stop at the ramp's own 0.5 m/s^2 — more than
/// the distance left — before any braking lag. Every leg therefore sailed past
/// its stop point: the blackbox shows `remaining_m` pinned at -5 with the
/// aircraft still rolling, and because `stop_straighten` has faded the steering
/// out by then, it drifted while nobody was holding the nose. Three line-up
/// legs in a row finished 2.5 to 8.6 m off with the aim point behind them, and
/// full auto's `lined_up` check — correctly — called it a failure.
///
/// 1.2 kt needs 0.38 m, comfortably inside the 1 m zero. The creep still moves
/// (the reason it exists at all: too low and the last metres never finish), and
/// the zero now matches the arrival test's own 1 m so the two agree about where
/// the route ends.
const CREEP_KTS: f64 = 1.2;
const STOP_ZERO_M: f64 = 1.0;

/// The scheduled taxi target: the pilot's pace, capped by any nearby bend
/// and by the stop ramp.
pub fn schedule_kts(base: f64, route: &Route, progress: f64, gs_kts: f64) -> f64 {
    let mut v = base.min(stop_ramp_kts(route.remaining(progress)));
    if let Some(cap) = route.bend_cap_kts(progress, gs_kts) {
        v = v.min(cap);
    }
    v
}

/// True when the plugin's steering channel is live — a zero write is a
/// no-op steer, so probing costs nothing.
pub async fn steer_channel_ok(xp: &dyn crate::xplane::XPlane) -> bool {
    xp.set_float(TAXI_STEER, 0.0).await.is_ok()
}

/// Spawn the steering loop. The caller (the `auto_taxi` tool) has already
/// validated the route, armed the taxi governor, and proved the plugin's
/// steer channel writable.
pub fn start(base_url: String, voice: Option<Voice>, route: Route, base_kts: f64) {
    let Some(gen) = RUN.claim() else {
        return; // already driving — the tool retargets instead
    };
    DRIVES_STARTED.fetch_add(1, Ordering::SeqCst);
    LAST_ARRIVED.store(false, Ordering::Relaxed);
    RELOCATED.store(false, Ordering::Relaxed);
    if let Ok(mut w) = PROGRESS.lock() {
        *w = ProgressWatch::start(clock_s(), route_deadline_s(route.total, base_kts));
    }
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            return;
        };
        rt.block_on(drive(XPlaneClient::new(base_url), voice, route, base_kts, &gen));
    });
}

/// Past this groundspeed a drive is no taxi at all, whatever the target.
pub const OVERSPEED_ABS_KTS: f64 = 25.0;
/// This far over the target, held [`OVERSPEED_HOLD_S`], stops the drive.
pub const OVERSPEED_OVER_KTS: f64 = 8.0;
pub const OVERSPEED_HOLD_S: f64 = 3.0;

/// Whether the taxi is running away: `Some("absolute")` past
/// [`OVERSPEED_ABS_KTS`] (stop now), `Some("over_target")` past the target
/// by [`OVERSPEED_OVER_KTS`] (stop once held). Pure.
pub fn taxi_overspeed(gs_kts: f64, target_kts: f64) -> Option<&'static str> {
    if gs_kts > OVERSPEED_ABS_KTS {
        Some("absolute")
    } else if target_kts >= 1.0 && gs_kts > target_kts + OVERSPEED_OVER_KTS {
        Some("over_target")
    } else {
        None
    }
}

/// How many drives have started ([`drive_gen`]); ownership of the flag is
/// the run's ([`RUN`]).
static DRIVES_STARTED: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Drive a backtrack: leg one (down the strip to the turn pocket) now, and
/// when it ARRIVES, leg two — a plain line-up from wherever the aircraft
/// actually stopped, which the join machinery turns into the 180. Any other
/// ending of leg one (pedals, "my controls", park handle, off-route) stands
/// the whole maneuver down: those paths already spoke, and the pilot who
/// took over mid-backtrack does not want a second drive starting itself.
pub fn start_backtrack(
    base_url: String,
    voice: Option<Voice>,
    route: Route,
    rwy: crate::autoland::Runway,
    base_kts: f64,
    need_m: f64,
) {
    // Leg two is built here, far from the tool that knows the airframe — carry
    // the pace across rather than letting the fresh route fall back to GA.
    let pace = route.pace;
    let turn = TurnRound {
        pivots: pace.brake_steer || crate::aircraft::nose_castors(),
        width_m: crate::autoland::runway_width_m(&rwy),
    };
    BACKTRACKING.store(true, Ordering::Relaxed);
    start(base_url.clone(), voice.clone(), route, base_kts);
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            return;
        };
        rt.block_on(async move {
            // Wait out leg one — bounded, so a stuck drive can't leak this
            // thread forever.
            for _ in 0..2400 {
                if !ACTIVE.load(Ordering::Relaxed) {
                    break;
                }
                tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            }
            if ACTIVE.load(Ordering::Relaxed) || !LAST_ARRIVED.load(Ordering::Relaxed) {
                BACKTRACKING.store(false, Ordering::Relaxed);
                return; // taken over, arrested, or still (impossibly) driving
            }
            if !crate::tools::speedhold::taxi_hold_active() {
                BACKTRACKING.store(false, Ordering::Relaxed);
                return; // speed authority gone with leg one — nobody drives
            }
            let xp = XPlaneClient::new(base_url.clone());
            let (Ok(lat), Ok(lon)) = (xp.get_float(LAT).await, xp.get_float(LON).await) else {
                BACKTRACKING.store(false, Ordering::Relaxed);
                return;
            };
            // Leg two of a backtrack: the aircraft has turned around and is
            // already on the strip pointing down it; settle and straighten
            // in the least runway that does it.
            let Some(path) = backtrack_leg_two_path(&rwy, lat, lon, turn) else {
                BACKTRACKING.store(false, Ordering::Relaxed);
                alert(
                    &voice,
                    Priority::Caution,
                    "I couldn't set up the turn. The line-up is yours.",
                );
                return;
            };
            let watch = crate::station::TaxiWatch {
                path,
                via: String::new(),
                seg: Vec::new(),
                holding_point: String::new(),
                lead_on: Vec::new(),
            };
            let Some(mut route2) = Route::build(&watch) else {
                BACKTRACKING.store(false, Ordering::Relaxed);
                alert(
                    &voice,
                    Priority::Caution,
                    "I couldn't set up the turn. The line-up is yours.",
                );
                return;
            };
            let spoken = crate::atis::spoken_runway(&rwy.id);
            route2.pace = pace;
            route2.align_course_deg = Some(rwy.course_deg);
            route2.align_runway = Some((rwy.clone(), need_m));
            route2.turn_hint = turn_round_hint(&rwy, lat, lon);
            route2.done_line = Some(format!("Lined up runway {spoken}. Holding position."));
            crate::blackbox::record(
                "autotaxi",
                serde_json::json!({"event": "backtrack_turn", "rwy": rwy.id,
                                   "pivots": turn.pivots, "width_m": turn.width_m,
                                   "turn_hint": route2.turn_hint,
                                   "from_along_m": -crate::autoland::along_track_nm(lat, lon, &rwy) * M_PER_NM,
                                   "from_ct_m": crate::autoland::cross_track_nm(lat, lon, &rwy) * M_PER_NM}),
            );
            crate::tools::speedhold::set_taxi_target(base_kts);
            start(base_url, voice, route2, base_kts);
            // Leg two owns ACTIVE from here.
            BACKTRACKING.store(false, Ordering::Relaxed);
        });
    });
}

/// How a line-up's alignment ended ([`align_on_the_runway`]).
enum AlignEnd {
    Aligned,
    /// Stopped this many degrees off the course (signed).
    NotAligned(f64),
    /// Stood down under it (pedals, "my controls", the hold gone, a newer
    /// drive): no arrival.
    Interrupted,
}

/// What a line-up that could not straighten says instead of "lined up".
pub fn not_aligned_line(err_deg: f64) -> String {
    format!(
        "On the runway, but {:.0} degrees off its heading and I couldn't straighten us. \
         Holding position.",
        err_deg.abs()
    )
}

/// Bring a castoring nose round onto the runway's course after a line-up's
/// arrival ([`LineUpAlign`]): the differential brakes on the drive's own
/// channels, the governor creeping or stopping. Every tick is blackboxed
/// `line_up_align`, the verdict `line_up_aligned`.
async fn align_on_the_runway(
    xp: &XPlaneClient,
    voice: &Option<Voice>,
    course: f64,
    fit: Option<&(crate::autoland::Runway, f64)>,
    gen: &sim::run::Lease<'_>,
) -> AlignEnd {
    // The runway it may roll is judged from where the drive stopped.
    let mut judge: Option<LineUpAlign> = None;
    let t0 = std::time::Instant::now();
    let mut frozen_s = 0.0_f64;
    let mut start: Option<(f64, f64)> = None;
    let mut last_creep: Option<bool> = None;
    let mut last_err = 0.0_f64;
    let mut last_lat: Option<f64> = None;
    let end = loop {
        // The tick runs to its period, not sleep + work (`DRIVE_TICK_MS`).
        let pacer = sim::state::Pacer::start();
        if !gen.live()
            || !speedhold::taxi_hold_active()
        {
            break AlignEnd::Interrupted;
        }
        // The freeze flags, the pedals, the position and the yaw rate in
        // ONE read (`sim::state`): a round of requests is a sim frame, and
        // the freeze check alone was one or two more (iMac 2026-09-30).
        let st = sim::state::read(xp, sim::state::Want(ALIGN_WANT)).await;
        if st.frozen() {
            progress_hold();
            frozen_s += 0.5;
            tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            continue;
        }
        let (yield_r, pos, yaw_r) = (
            st.get(sim::state::F::SteerYield),
            st.position(),
            st.get(sim::state::F::R),
        );
        if yield_r.is_ok_and(|v| v > 0.5) {
            crate::blackbox::record("autotaxi", serde_json::json!({"event": "pedal_yield"}));
            crate::flight::note_pilot_takeover("taxi: pedals");
            alert(
                voice,
                Priority::Advisory,
                "Your steering. I still have the throttle and brakes.",
            );
            gen.lower_flag();
            break AlignEnd::Interrupted;
        }
        let (lat_r, lon_r) = sim::position::lat_lon(&pos);
        let psi_r = sim::position::field(&pos, |p| p.psi_true_deg);
        let gs_r = sim::position::field(&pos, |p| p.gs_ms);
        // A failed read is no measurement: nothing written, the channels
        // expire, the next tick asks again.
        let (Ok(lat), Ok(lon), Ok(psi), Ok(gs_ms), Ok(yaw)) = (lat_r, lon_r, psi_r, gs_r, yaw_r)
        else {
            tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            continue;
        };
        progress_hold();
        let gs_kts = gs_ms * MS_TO_KTS;
        let from = *start.get_or_insert((lat, lon));
        let judge = judge.get_or_insert_with(|| {
            let spare = fit.map(|(rwy, need)| align_spare_m(rwy, *need, lat, lon));
            let j = LineUpAlign::with_spare(course, spare);
            let j = if crate::aircraft::nose_free() {
                j.with_ground(crate::ground_steer::GroundPlant::for_airframe())
            } else {
                j
            };
            crate::blackbox::record(
                "autotaxi",
                serde_json::json!({"event": "line_up_align_budget", "spare_m": spare,
                                   "roll_limit_m": j.roll_limit_m, "creep_kts": j.creep_kts}),
            );
            j
        });
        let rolled_m = crate::autoland::haversine_nm(from.0, from.1, lat, lon) * M_PER_NM;
        let t_s = t0.elapsed().as_secs_f64() - frozen_s;
        last_err = judge.err_deg(psi);
        // The lateral off the runway's centreline, when the runway is known.
        let lat_m = fit.map(|(rwy, _)| crate::autoland::cross_track_nm(lat, lon, rwy) * M_PER_NM);
        last_lat = lat_m;
        let step = judge.step_on_line(t_s, psi, gs_kts, yaw, rolled_m, lat_m);
        let (bl, br, creep) = match step {
            AlignStep::Align { bl, br, creep } => (bl, br, creep),
            _ => (0.0, 0.0, false),
        };
        crate::blackbox::record(
            "autotaxi",
            serde_json::json!({"event": "line_up_align", "err": last_err, "yaw_dps": yaw,
                               "gs": gs_kts, "bl": bl, "br": br, "creep": creep,
                               "trim": judge.trim, "rolled_m": rolled_m, "t_s": t_s,
                               "lat_m": lat_m, "closing": judge.closing}),
        );
        match step {
            AlignStep::Done { .. } => break AlignEnd::Aligned,
            AlignStep::GiveUp { err_deg, .. } => break AlignEnd::NotAligned(err_deg),
            AlignStep::Align { .. } => {}
        }
        if last_creep != Some(creep) {
            speedhold::set_taxi_target(if creep { judge.creep_kts } else { 0.0 });
            last_creep = Some(creep);
        }
        // The rudder helps what it can (a steerable castoring nose, the
        // SF50's); the brakes do the turning.
        // On a free nose the rudder is the one ground law's.
        let steer = if judge.free() {
            judge.rudder
        } else {
            ((last_err - yaw * ALIGN_LEAD_S) / ALIGN_FULL_DEG).clamp(-1.0, 1.0)
        };
        let _ = tokio::join!(
            xp.set_float(TAXI_STEER, steer),
            xp.set_float(LEFT_BRAKE, bl),
            xp.set_float(RIGHT_BRAKE, br),
        );
        pacer.wait(std::time::Duration::from_millis(DRIVE_TICK_MS)).await;
    };
    speedhold::set_taxi_target(0.0);
    let _ = tokio::join!(
        xp.set_float(TAXI_STEER, 0.0),
        xp.set_float(LEFT_BRAKE, 0.0),
        xp.set_float(RIGHT_BRAKE, 0.0),
    );
    let ok = match &end {
        AlignEnd::Aligned => Some(true),
        AlignEnd::NotAligned(_) => Some(false),
        AlignEnd::Interrupted => None,
    };
    let (creeps, rolled) = match (&judge, start) {
        (Some(j), Some(from)) => {
            let here = sim::position::read(xp).await;
            let (la, lo) = sim::position::lat_lon(&here);
            let rolled = la.ok().zip(lo.ok()).map(|(la, lo)| {
                crate::autoland::haversine_nm(from.0, from.1, la, lo) * M_PER_NM
            });
            (j.creeps, rolled)
        }
        _ => (0, None),
    };
    crate::blackbox::record(
        "autotaxi",
        serde_json::json!({"event": "line_up_aligned", "ok": ok, "err_deg": last_err,
                           "course": course, "creeps": creeps, "rolled_m": rolled,
                           "lat_m": last_lat,
                           "closed_lateral": judge.as_ref().is_some_and(|j| j.closed_lateral),
                           "secs": t0.elapsed().as_secs_f64() - frozen_s}),
    );
    end
}

/// Say a line without blocking the loop (queued; gated off in ATC-only mode
/// like every crew alert).
/// How long a drive may command motion without moving before it stands down.
const STAGNANT_SECS: f64 = 30.0;

/// What a stuck drive says. Each clause is what the log can actually show:
/// "the nose is pointing away" used to be said for every stall with a big
/// heading error, including a 172 sat on the grass at breakaway power that
/// no heading could have moved (LIPK 2026-09-26).
fn stagnant_line(off_pavement: bool, facing_away: bool) -> &'static str {
    match (off_pavement, facing_away) {
        (true, _) => {
            "I can't get us moving, we're off the pavement and on the grass. \
             Your aircraft. Get us back onto the taxiway, then ask me to taxi again."
        }
        (false, true) => {
            "I can't get us moving with the nosewheel hard over. Your aircraft. \
             Turn us toward the taxiway, then ask me to taxi again."
        }
        (false, false) => "I can't get us moving. Standing down. Your aircraft.",
    }
}

fn alert(voice: &Option<Voice>, priority: Priority, line: &str) {
    if let Some(v) = voice {
        v.alert(priority, line);
    }
}

/// The steering loop. Steering writes ride the plugin's expiring channel, so
/// every exit path — including a panic or kill -9 — releases the override
/// within a second by silence alone; the explicit zero write on the way out
/// is just politeness.
async fn drive(
    xp: XPlaneClient,
    voice: Option<Voice>,
    mut route: Route,
    base_kts: f64,
    gen: &sim::run::Lease<'_>,
) {
    let _detail = crate::auto_status::DetailGuard::hold(&DETAIL);
    // The landing light over runway spans, decided once for the drive: none
    // when the airframe's landing lamp is already its taxi light tonight.
    let crossing_light = if route.runway_spans.is_empty() {
        None
    } else {
        crate::tools::lights::crossing_light(crate::tools::lights::dark_now(&xp).await)
    };
    let mut crossing_lit = false;
    tracing::info!("auto taxi started");
    crate::blackbox::record(
        "autotaxi",
        serde_json::json!({"event": "engaged", "total_m": route.total, "base_kts": base_kts}),
    );
    // The pilot's pace: adopted from a spoken "taxi at N" mid-drive.
    let mut base = base_kts;
    // What the schedule last wrote into the governor — any OTHER value read
    // back is an external retarget (spoken stop/go/speed) to fold in.
    let mut last_sched = base_kts;
    // External stop ("stop" / both pedals park the target at 0): the
    // schedule stands aside until a "go" restores a rolling target.
    let mut ext_stopped = false;
    // A tug on the bar: held since, and whether the hold has been announced.
    let mut tug_since: Option<std::time::Instant> = None;
    let mut tug_held = false;
    // Seeded at the first fix: >0 means we engaged off the route or facing
    // against it and are JOINING — widened guard, bend-speed cap — until
    // captured, when it drops to 0 and the established guard takes over.
    let mut join_allow: Option<f64> = None;
    let mut join_capture = JoinCapture::default();
    let t_join = std::time::Instant::now();
    // Yaw rate for the steering lead, EMA-smoothed: raw tick-to-tick heading
    // differences of an HTTP read are noise the lead would amplify.
    let mut prev_psi: Option<(f64, std::time::Instant)> = None;
    let mut yaw_rate = 0.0f64;
    // Last steering command, so the slew limiter has something to move FROM.
    let mut prev_steer = 0.0f64;
    let mut straight_trim = StraightTrim::default();
    // A turn-round the geometry knows the way of is turned that way from
    // the first tick (`Route.turn_hint`).
    let mut turn_dir: Option<f64> = route.turn_hint;
    // Whether the brake-steer assist currently has a wheel braked, so every
    // exit path can release it exactly once — and the last commanded pair,
    // so the assist's own slew has something to move from.
    let mut bs_on = false;
    let mut brakes = BrakeSteer::default();
    // A nose with no steering of its own flies the one ground law on the
    // route (`ground_steer`), on the sim's own yaw rate.
    let free_nose = crate::aircraft::nose_free();
    let mut ground = crate::ground_steer::GroundSteer::for_airframe();
    let mut ref_path = route.ref_path();
    let mut ref_dir: Option<f64> = route.turn_hint;
    let max_yaw_dps = route.pace.max_yaw_dps.unwrap_or(crate::ground_steer::DEFAULT_MAX_YAW_DPS);
    // Stagnation watch: commanding motion but not moving (see below).
    let mut stalled_since: Option<std::time::Instant> = None;
    // Over the taxi pace since (see [`taxi_overspeed`]).
    let mut overspeed_since: Option<std::time::Instant> = None;
    // The last good fix, for the relocation guard ([`relocation_jump_m`]).
    let mut last_fix: Option<((f64, f64), std::time::Instant)> = None;
    // The last tick's lateral, for the join's jump test (`join_fix`).
    let mut last_lateral: Option<f64> = None;
    let mut last_jumped = false;
    // A join the nose cannot point at is judged once, at the first fix
    // ([`JoinTurn`]).
    let mut join_turn_judged = false;
    loop {
        // The tick runs to its period, not sleep + work (`DRIVE_TICK_MS`).
        let pacer = sim::state::Pacer::start();
        if !gen.live() {
            crate::blackbox::record(
                "autotaxi",
                serde_json::json!({"event": "stopped_from_outside", "superseded": !gen.current()}),
            );
            break;
        }
        // Speed authority gone — steering alone must not keep driving. This
        // is NOT only the deliberate paths (park handle, spoken release):
        // the governor's throttle-override check can't tell a pilot's hand
        // from a hardware-axis bump the sim re-drives, and mid-drive that
        // left an aircraft rolling at taxi power with NOBODY on the
        // throttle — the 2026-07-20 Gatwick runaway ran 10 to 50 knots
        // through the grass in silence. On the ground the safe reading is
        // stopping: throttle to idle plus ONE expiring brake pulse (the
        // pilot's own brake key, frees itself within a second, can't
        // stick), and say it out loud — a pilot who truly wants the power
        // pushes again with everything already released. Airborne (a
        // runaway that got that far), touch nothing.
        if !speedhold::taxi_hold_active() {
            let on_ground = xp.get_float(crate::datarefs::ON_GROUND_ANY).await.unwrap_or(0.0) > 0.5;
            tracing::info!(
                on_ground,
                "auto taxi: taxi hold gone, arresting and releasing"
            );
            if on_ground {
                let _ = xp.set_float(&throttle_dref(), 0.0).await;
                if xp
                    .run_command_for(crate::tools::BRAKES_REGULAR, crate::tools::BRAKE_HOLD_SECS)
                    .await
                    .is_ok()
                {
                    crate::tools::note_brake_cmd();
                }
                alert(
                    &voice,
                    Priority::Caution,
                    "I've lost the speed hold. Throttle's idle. Your aircraft.",
                );
            } else {
                alert(
                    &voice,
                    Priority::Caution,
                    "I've lost the speed hold. Your aircraft.",
                );
            }
            crate::blackbox::record(
                "autotaxi",
                serde_json::json!({"event": "hold_gone", "arrested": on_ground}),
            );
            break;
        }
        // Paused or replay: stop writing — the steer channel expires and the
        // pedals are the pilot's until the sim is live again.
        //
        // One read, not a chain: sequential HTTP awaits stretched the
        // takeoff tick to half a second and bought a slew-limit weave
        // (sortie four); the same medicine here keeps the steer tick honest.
        // Position, heading and groundspeed as ONE frame saw them (#157),
        // and since 2026-09-30 the freeze flags, the pedals and the tug in
        // the same request (`sim::state`): every round of requests is a sim
        // frame (50 ms on the iMac at 20 fps), and the freeze check was one
        // or two rounds of its own ahead of the read.
        let st = sim::state::read(&xp, sim::state::Want(if free_nose { DRIVE_WANT_FREE } else { DRIVE_WANT })).await;
        if st.frozen() {
            stalled_since = None; // a pause is not a stall
            progress_hold();
            tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            continue;
        }
        let (yield_r, pos, tug_r) = (
            st.get(sim::state::F::SteerYield),
            st.position(),
            st.get(sim::state::F::Tug),
        );
        // A free nose reads the sim's yaw rate too (its law's measurement).
        let yaw_now = if free_nose { st.opt(sim::state::F::R) } else { None };
        let (lat_r, lon_r) = sim::position::lat_lon(&pos);
        let psi_r = sim::position::field(&pos, |p| p.psi_true_deg);
        let gs_r = sim::position::field(&pos, |p| p.gs_ms);
        let tug_on = tug_r.is_ok_and(|v| v > 0.5);
        if tug_on {
            let t0 = *tug_since.get_or_insert_with(std::time::Instant::now);
            if !tug_held {
                tug_held = true;
                tracing::info!("auto taxi: tug on the bar, holding until it lets go");
                crate::blackbox::record("autotaxi", serde_json::json!({"event": "tug_hold"}));
                speedhold::set_taxi_target(0.0);
                last_sched = 0.0;
                // Once per tug: the push may have just said it.
                if !crate::pushback::take_tug_warned() {
                    alert(
                        &voice,
                        Priority::Advisory,
                        "The tug's still on the nose. Holding until it lets go.",
                    );
                }
            }
            if t0.elapsed().as_secs_f64() > TUG_WAIT_SECS {
                tracing::warn!("auto taxi: the tug never let go, standing down");
                crate::blackbox::record(
                    "autotaxi",
                    serde_json::json!({"event": "tug_never_released"}),
                );
                alert(
                    &voice,
                    Priority::Caution,
                    "The tug hasn't let go. Standing down, brakes set. Your aircraft.",
                );
                gen.lower_flag();
                break;
            }
            // No steer heartbeat: the nosewheel is the tug's, and the
            // channel expires on its own. Nothing here is a stall.
            stalled_since = None;
            progress_hold();
            tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            continue;
        } else if tug_held {
            tug_held = false;
            tug_since = None;
            // Not said: the monitor's "Pushback done, tug's clear" is the
            // one voice for the bar coming off (both were heard, LIPK
            // 2026-09-26).
            crate::blackbox::record("autotaxi", serde_json::json!({"event": "tug_released"}));
        }
        // The pilot's feet moved the yaw axis: the plugin already dropped
        // the override — stand the loop down and keep the speed hold, which
        // is the pre-existing "you steer, I have the throttle" division.
        if yield_r.is_ok_and(|v| v > 0.5) {
            tracing::info!("auto taxi: pilot on the pedals, steering released");
            crate::blackbox::record("autotaxi", serde_json::json!({"event": "pedal_yield"}));
            crate::flight::note_pilot_takeover("taxi: pedals");
            alert(
                &voice,
                Priority::Advisory,
                "Your steering. I still have the throttle and brakes.",
            );
            gen.lower_flag();
            break;
        }
        let (Ok(lat), Ok(lon), Ok(psi), Ok(gs_ms)) = (lat_r, lon_r, psi_r, gs_r) else {
            // Sim hiccup: no writes, the channel expires, retry.
            tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
            continue;
        };
        let gs_kts = gs_ms * MS_TO_KTS;
        // Moved under us: whatever this route was, it is for a place the
        // aircraft no longer is. Stand down BEFORE the join or the off-route
        // guard reads the jump as a drive gone wrong (LEMG 31, 2026-09-28).
        let now = std::time::Instant::now();
        if let Some((from, t0)) = last_fix {
            if let Some(jump_m) =
                relocation_jump_m(from, (lat, lon), now.duration_since(t0).as_secs_f64())
            {
                tracing::warn!(jump_m, "auto taxi: the aircraft was moved, standing down");
                crate::blackbox::record(
                    "autotaxi",
                    serde_json::json!({"event": "relocated", "jump_m": jump_m,
                                       "dt_s": now.duration_since(t0).as_secs_f64()}),
                );
                speedhold::set_taxi_target(0.0);
                let _ = xp.set_float(TAXI_STEER, 0.0).await;
                RELOCATED.store(true, Ordering::Relaxed);
                alert(
                    &voice,
                    Priority::Caution,
                    "We've been moved. Taxi stopped. Your aircraft.",
                );
                gen.lower_flag();
                break;
            }
        }
        let moved_m = last_fix.map_or(0.0, |(from, _)| {
            crate::autoland::haversine_nm(from.0, from.1, lat, lon) * M_PER_NM
        });
        last_fix = Some(((lat, lon), now));
        // Signed here, magnitude everywhere below: the gates ask how far off
        // the route we are, and the record wants to know which side. While
        // still JOINING, project along the nose: the nearest paint can be a
        // leg already behind us (the pushback that let go 60 m short of its
        // node, Malaga 2026-09-11 — the join drove a loop back to the node).
        // ...never onto a leg the aircraft did not move to (`join_fix`).
        let joining = join_allow.is_none_or(|a| a > 0.0);
        let fixed = if joining {
            let (fix, jumped) = join_fix(
                route.fix_signed_facing(lat, lon, psi),
                route.fix_signed(lat, lon),
                last_lateral,
                moved_m,
            );
            if jumped && !last_jumped {
                crate::blackbox::record(
                    "autotaxi",
                    serde_json::json!({"event": "projection_jump", "prev_lat_m": last_lateral,
                                       "moved_m": moved_m,
                                       "facing": route.fix_signed_facing(lat, lon, psi)}),
                );
            }
            last_jumped = jumped;
            fix
        } else {
            route.fix_signed(lat, lon)
        };
        let Some((progress, offset)) = fixed else {
            tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
            continue;
        };
        let lateral = offset.abs();
        last_lateral = Some(lateral);
        let remaining = route.remaining(progress);
        progress_note(progress);
        if ext_stopped {
            // Held by a spoken "stop" (or both pedals): a stop somebody
            // asked for is not a stall.
            progress_hold();
        }
        DETAIL.set(format!(
            "{} m to go",
            (remaining.max(0.0) / 10.0).round() as i64 * 10
        ));
        if let Some(l) = crossing_light.as_ref() {
            let want = on_runway_span(&route.runway_spans, progress);
            if want != crossing_lit {
                crossing_lit = want;
                let ok = crate::tools::lights::apply(&xp, l, want).await.is_ok();
                crate::blackbox::record(
                    "autotaxi",
                    serde_json::json!({"event": "crossing_lights", "on": want, "ok": ok, "progress_m": progress}),
                );
            }
        }
        // Steering error first: the join state and the guard both need it.
        // The aim: the airframe's, capped in a bend and near the stop
        // (see [`steer_look_m`]).
        let look = steer_look_m(
            &route.pace,
            gs_kts,
            route.route_turn_rate_dps(progress, BEND_PROBE_M, gs_kts),
            remaining,
            free_nose,
        );
        // Inside the stop zone, fly the route's FINAL BEARING instead of
        // pursuing an aim point that has stopped moving. See
        // `final_bearing_deg`: past the end the pursuit error runs to 160°+
        // and steers the aircraft into a turn toward a waypoint behind it.
        // Holding the last segment's direction is what a pilot does — nose
        // straight down the paint, roll to a halt.
        let err = if remaining <= STOP_HEADING_HOLD_M {
            route.stop_heading_err_deg(lat, lon, psi, progress, look)
        } else {
            route.heading_err_deg(lat, lon, psi, progress, look)
        };
        // The first fix: a join the nose cannot point at is flown as a
        // planned turn onto the route ahead, or refused and said (#190).
        if !join_turn_judged {
            join_turn_judged = true;
            let pivots = route.pace.brake_steer || crate::aircraft::nose_castors();
            let own_geometry = route.align_course_deg.is_some() || route.turn_hint.is_some();
            let quick = err.abs() > JOIN_TURN_MIN_DEG && !pivots && !own_geometry;
            let mtow_kg = if quick {
                xp.get_float(crate::datarefs::ACF_M_MAX_KG).await.ok().map(f64::from)
            } else {
                None
            };
            let on_rwy = crate::party_line::on_runway(lat, lon);
            if joining && join_turn_applies(err, pivots, mtow_kg, on_rwy, own_geometry) {
                let radius_m = join_turn_radius_m(mtow_kg);
                let turn = join_turn(&route.path, lat, lon, psi, radius_m, |a, b| {
                    crate::party_line::on_pavement(a, b)
                });
                match turn.and_then(|t| route.with_lead(&t.lead, t.join_m).map(|r| (t, r))) {
                    Some((t, rebuilt)) => {
                        tracing::info!(err, turn = t.turn_deg, "auto taxi: planned turn onto the route");
                        crate::blackbox::record(
                            "autotaxi",
                            serde_json::json!({"event": "join_turn", "err_deg": err, "lateral_m": offset,
                                               "dir": t.dir, "turn_deg": t.turn_deg, "radius_m": radius_m,
                                               "join_m": t.join_m, "lead_m": t.lead_m,
                                               "intercept_deg": t.intercept_deg,
                                               "total_m": rebuilt.total, "mtow_kg": mtow_kg}),
                        );
                        route = rebuilt;
                        ref_path = route.ref_path();
                        turn_dir = Some(t.dir);
                        ref_dir = Some(t.dir);
                        last_lateral = None;
                        progress_rebase();
                        continue;
                    }
                    None => {
                        tracing::warn!(err, "auto taxi: no turn onto the route fits, refusing");
                        crate::blackbox::record(
                            "autotaxi",
                            serde_json::json!({"event": "join_refused", "err_deg": err, "lateral_m": offset,
                                               "radius_m": radius_m, "mtow_kg": mtow_kg}),
                        );
                        speedhold::set_taxi_target(0.0);
                        let _ = xp.set_float(TAXI_STEER, 0.0).await;
                        alert(&voice, Priority::Caution, JOIN_REFUSED_LINE);
                        gen.lower_flag();
                        break;
                    }
                }
            }
        }
        let err = committed_err(err, &mut turn_dir);
        // The turn the route asks for over that same aim span, for the rate
        // lead to credit. Zero inside the stop zone: there the error is held
        // against a FIXED final bearing, which is a straight line by
        // construction, so any yaw there really is a swing to be checked.
        let route_turn_dps = if remaining <= STOP_HEADING_HOLD_M {
            0.0
        } else {
            route.route_turn_rate_dps(progress, look, gs_kts)
        };
        let allow = *join_allow.get_or_insert_with(|| {
            let a = join_allowance_m(lateral, err);
            if a > 0.0 {
                tracing::info!(lateral, err, "auto taxi: joining the route");
                crate::blackbox::record(
                    "autotaxi",
                    serde_json::json!({"event": "join", "lateral_m": lateral, "err_deg": err}),
                );
            }
            a
        });
        if allow > 0.0 && join_capture.step(lateral, err, t_join.elapsed().as_secs_f64()) {
            tracing::info!(lateral, "auto taxi: joined the route");
            crate::blackbox::record("autotaxi", serde_json::json!({"event": "joined"}));
            join_allow = Some(0.0);
        }
        // Too far off the pavement we were cleared on: stop, hand steering
        // back, and say so. Established that's the tight guard; a join gets
        // its engage distance plus turning room, never more — the one drive
        // this loop makes is TOWARD the route.
        if lateral > OFF_ROUTE_STOP_M + join_allow.unwrap_or(0.0) {
            tracing::warn!(lateral, "auto taxi: off route, stopping");
            crate::blackbox::record(
                "autotaxi",
                serde_json::json!({"event": "off_route", "lateral_m": lateral}),
            );
            speedhold::set_taxi_target(0.0);
            let _ = xp.set_float(TAXI_STEER, 0.0).await;
            alert(
                &voice,
                Priority::Caution,
                "We're off the cleared route. Stopping here. Your steering.",
            );
            gen.lower_flag();
            break;
        }
        // Stopped at the end of the route but NOT on it: that is no arrival.
        // A projection runs along the route however far off it the aircraft
        // is, so the end can be "reached" from the grass beside it: LSZS 03
        // (SR22, 2026-09-24) the second line-up joined 27 m off, never
        // captured, and declared the holding point 27.4 m off the line; full
        // auto then found the runway 34 degrees off. Hold it where it is and
        // hand the steering back, with the number.
        if remaining <= 1.0 && gs_kts < 1.5 && !arrived_on_line(lateral) {
            tracing::warn!(lateral, "auto taxi: route end reached off the line");
            crate::blackbox::record(
                "autotaxi",
                serde_json::json!({"event": "stopped_off_line", "lateral_m": lateral,
                                   "joining": join_allow.is_none_or(|a| a > 0.0)}),
            );
            speedhold::set_taxi_target(0.0);
            let _ = xp.set_float(TAXI_STEER, 0.0).await;
            alert(&voice, Priority::Caution, &off_line_stop_line(lateral));
            gen.lower_flag();
            break;
        }
        // Arrived: stopped at the hold-short line, ON it. The governor stays,
        // holding position on the brakes, exactly as a spoken "stop" —
        // "go" past this point is the pilot's call (a crossing clearance).
        if remaining <= 1.0 && gs_kts < 1.5 {
            // A line-up on a castoring nose ends POINTING down the runway,
            // not merely on it ([`LineUpAlign`]).
            let mut crooked: Option<f64> = None;
            if let Some(course) = route.align_course_deg.filter(|_| {
                route.pace.brake_steer || crate::aircraft::nose_castors()
            }) {
                match align_on_the_runway(&xp, &voice, course, route.align_runway.as_ref(), gen)
                    .await
                {
                    AlignEnd::Aligned => {}
                    AlignEnd::NotAligned(err) => crooked = Some(err),
                    AlignEnd::Interrupted => break,
                }
            }
            tracing::info!("auto taxi: holding point, done");
            LAST_ARRIVED.store(true, Ordering::Relaxed);
            crate::blackbox::record("autotaxi", serde_json::json!({"event": "holding_point"}));
            speedhold::set_taxi_target(0.0);
            let _ = xp.set_float(TAXI_STEER, 0.0).await;
            let line = if let Some(err) = crooked {
                not_aligned_line(err)
            } else if let Some(done) = &route.done_line {
                done.clone()
            } else if route.holding_point.is_empty() {
                "Holding point. Holding position, brakes set.".to_string()
            } else {
                format!(
                    "Holding point {}. Holding position, brakes set.",
                    route.holding_point
                )
            };
            alert(&voice, Priority::Advisory, &line);
            gen.lower_flag();
            break;
        }
        // A drive that commands motion but never MOVES is stuck. A backwards
        // join's near-full nosewheel lock is a chock the governor's gentle
        // taxi throttle can't break away against: a 169°-off engage at a
        // Jerez stand crept at millimetres per second for half a minute, no
        // voice line, until the pilot's pedals rescued it (16:48Z,
        // 2026-07-24). Twenty seconds of commanded-but-stationary says so out
        // loud and hands back, instead of pirouetting in place.
        // The window covers the governor's ramp to its BREAKAWAY ceiling
        // (`speedhold::taxi_throttle_cap`), about 20 s from idle at its
        // usual step: judged at 20 s the 172 at LIPK was still at 0.35.
        if !ext_stopped && gs_kts < 0.7 {
            let t0 = *stalled_since.get_or_insert_with(std::time::Instant::now);
            if t0.elapsed().as_secs_f64() > STAGNANT_SECS {
                let thr_dref = throttle_dref();
                let (thr, la, lo) = tokio::join!(
                    xp.get_float(&thr_dref),
                    xp.get_float(LAT),
                    xp.get_float(LON),
                );
                let (thr, here) = (thr.ok(), la.ok().zip(lo.ok()));
                let off_pavement = here
                    .and_then(|(la, lo)| crate::party_line::on_pavement(la, lo))
                    == Some(false);
                tracing::warn!(err, ?thr, off_pavement, "auto taxi: stagnant, standing down");
                crate::blackbox::record(
                    "autotaxi",
                    serde_json::json!({"event": "stagnant", "err_deg": err, "remaining_m": remaining,
                                       "throttle": thr, "off_pavement": off_pavement}),
                );
                speedhold::set_taxi_target(0.0);
                let _ = xp.set_float(TAXI_STEER, 0.0).await;
                let line = stagnant_line(off_pavement, err.abs() > JOIN_ERR_SLOW_DEG);
                alert(&voice, Priority::Caution, line);
                gen.lower_flag();
                break;
            }
        } else {
            stalled_since = None;
        }
        // Too fast to be a taxi: stop, never press on into a corner. The
        // Evolution taxied at 17 to 23 kt against a 10 kt target, took a
        // corner at that pace, ground-looped off the pavement and hit a wall
        // (LIME 2026-09-25). Sustained over the target by
        // [`OVERSPEED_OVER_KTS`], or ever past [`OVERSPEED_ABS_KTS`].
        // Against the drive's PACE, never a bend's momentary target: slowing
        // from 14 kt toward a 6 kt bend is the governor working, and judged
        // against the 6 it stopped a good taxi at LIME (2026-09-25).
        let target_now = speedhold::taxi_target().unwrap_or(base_kts).max(base);
        let over = taxi_overspeed(gs_kts, target_now);
        let over_held = match over {
            Some("absolute") => true,
            Some(_) => overspeed_since
                .get_or_insert_with(std::time::Instant::now)
                .elapsed()
                .as_secs_f64()
                >= OVERSPEED_HOLD_S,
            None => {
                overspeed_since = None;
                false
            }
        };
        if over_held {
            tracing::warn!(gs_kts, target_now, "auto taxi: too fast, stopping");
            crate::blackbox::record(
                "autotaxi",
                serde_json::json!({"event": "overspeed_stop", "gs": gs_kts, "target": target_now, "why": over}),
            );
            speedhold::set_taxi_target(0.0);
            let _ = xp.set_float(TAXI_STEER, 0.0).await;
            alert(
                &voice,
                Priority::Warning,
                &format!("Too fast for the taxi, {gs_kts:.0} knots. Stopping. Your aircraft."),
            );
            gen.lower_flag();
            break;
        }
        // Steering: aim at the route ahead, command the error down — led by
        // the yaw rate so a laggy plant (castoring nosewheel steering by
        // rudder) can't snake — with the lock tapered by groundspeed so a
        // standstill can break away. From a backwards or off-route start
        // this is the whole join — the deflection tightens as the roll
        // builds. Approaching the stop (route end, or a spoken "stop"
        // bleeding speed off) the wheel straightens so the brakes never
        // bite onto a deflected nosewheel.
        let now = std::time::Instant::now();
        let dt = prev_psi.map_or(0.2, |(_, t0)| {
            now.duration_since(t0).as_secs_f64().clamp(0.05, 1.0)
        });
        let raw_rate = prev_psi.map_or(0.0, |(p, t0)| {
            let dt = now.duration_since(t0).as_secs_f64();
            if (0.05..=1.0).contains(&dt) {
                crate::angles::signed_diff_deg(psi, p) / dt
            } else {
                0.0
            }
        });
        prev_psi = Some((psi, now));
        // Low-passed on TIME (`YAW_RATE_TAU_S`), not half per tick.
        yaw_rate += (raw_rate.clamp(-90.0, 90.0) - yaw_rate) * (1.0 - (-dt / YAW_RATE_TAU_S).exp());
        // A free castoring nose: the one ground law on the route
        // (`ground_steer`), the rudder and the brakes allocated together.
        let mut ground_rec: Option<serde_json::Value> = None;
        let (steer, bl, br, trim) = if free_nose {
            let v = gs_ms;
            let (e, ff) = if remaining <= STOP_HEADING_HOLD_M {
                // The stop zone: the route's final bearing, as every nose.
                (err, 0.0)
            } else {
                crate::ground_steer::path_demand(&ref_path, progress, route.local_m(lat, lon), psi, v, &ground, max_yaw_dps)
            };
            let e = committed_err(e, &mut ref_dir);
            let s = ground.step(crate::ground_steer::Demand {
                dt,
                v_ms: v,
                yaw: yaw_now.map_or(crate::ground_steer::Yaw::Mean(raw_rate), crate::ground_steer::Yaw::Now),
                err_deg: e,
                ff_dps: ff,
                rudder: crate::ground_steer::Rudder::Own {
                    slew_per_s: route.pace.steer_slew_per_s.unwrap_or(0.8),
                },
                u_max: PIVOT_BRAKE_MAX,
                r_max_dps: ground.taxi_rate_limit(v, PIVOT_BRAKE_MAX, max_yaw_dps),
                io_delay_s: crate::ground_steer::IO_DELAY_S,
            });
            let (bl, br) = s.wheels();
            bs_on = bl > 0.0 || br > 0.0;
            prev_steer = s.rudder;
            let mut rec = s.json();
            rec["e_ref"] = serde_json::json!(e);
            rec["ff"] = serde_json::json!(ff);
            rec["r"] = serde_json::json!(yaw_now);
            ground_rec = Some(rec);
            (s.rudder, bl, br, 0.0)
        } else {
            let trim = straight_trim.step(
                offset,
                route_turn_dps,
                gs_kts,
                join_allow.unwrap_or(0.0) > 0.0,
                remaining,
                dt,
            );
            let want = stop_straighten(
                (steer_cmd_with(
                    damped_err_deg(err, yaw_rate, route_turn_dps, route.pace.steer_lead_s),
                    route.pace.steer_full_deg,
                ) + trim)
                    .clamp(-1.0, 1.0)
                    * steer_authority(gs_kts),
                remaining,
                gs_kts,
            );
            let mut steer = slew_steer(prev_steer, want, route.pace.steer_slew_per_s, dt);
            prev_steer = steer;
            if ext_stopped {
                steer *= (gs_kts / 3.0).clamp(0.0, 1.0);
            }
            // The brake half of the steering, where the airframe needs it. Only
            // ever one side, slewed on like a foot (BRAKE_STEER_SLEW_PER_S),
            // released the moment the command shrinks back into the deadband.
            //
            // Written EVERY tick, never edge-gated on change. The plugin channel
            // is a heartbeat that expires a second after writes stop, so skipping
            // a steady command doesn't leave the last one standing, it drops the
            // brake mid-turn, and a turn is exactly where the command sits still
            // for seconds. (The old gate existed to keep HTTP off the tick, which
            // the join! below already buys: all three control writes go out
            // together, so the whole steering output costs ONE round trip, not
            // three. The 2026-07-28 lesson was about sequential awaits dragging
            // the loop to a second, not about the number of writes.)
            let (bl, br) = if route.pace.brake_steer || crate::aircraft::nose_castors() {
                let (bl, br) = brakes.step(
                    steer,
                    err,
                    gs_kts,
                    yaw_rate,
                    join_allow.unwrap_or(0.0) > 0.0,
                    dt,
                );
                bs_on = bl > 0.0 || br > 0.0;
                (bl, br)
            } else {
                (0.0, 0.0)
            };
            (steer, bl, br, trim)
        };
        let mut drive_rec = serde_json::json!({
            // `lat_m` is SIGNED: right of the route is positive. It was
            // a distance until 2026-08-05, which made every taxi trail in
            // the log unreadable against the paint: "two metres off" says
            // nothing about which side, and the side is the whole
            // question when the pilot can see the line.
            // `turn_dps` is the bend's OWN rate, the credit the lead
            // gives back: `yaw_dps` minus it is what the law actually
            // damps, and the pair is how a wide corner gets told apart
            // from a swing in the trace.
            "event": "drive", "gs": gs_kts, "err": err, "yaw_dps": yaw_rate,
            "turn_dps": route_turn_dps, "look_m": look,
            "steer": steer, "lat_m": offset, "remaining_m": remaining, "trim": trim,
            "bs": bs_on, "bl": bl, "br": br, "tug": tug_on,
            // "plugin": one frame's position; "split": separate reads (#157).
            "pos": sim::position::source_word(&pos),
        });
        // The free nose's law, tick by tick: the yaw rate asked and read,
        // the standing yaw learnt, the heading error flown and the route's
        // own rate fed forward.
        if let Some(g) = ground_rec {
            drive_rec["gsteer"] = g;
        }
        crate::blackbox::record("autotaxi", drive_rec);
        let steer_write = if route.pace.brake_steer || crate::aircraft::nose_castors() {
            let (s, a, b) = tokio::join!(
                xp.set_float(TAXI_STEER, steer),
                xp.set_float(LEFT_BRAKE, bl),
                xp.set_float(RIGHT_BRAKE, br),
            );
            let _ = (a, b);
            s
        } else {
            xp.set_float(TAXI_STEER, steer).await
        };
        if steer_write.is_err() {
            // The channel vanished (plugin reload): no steering, no drive.
            tracing::warn!("auto taxi: steer channel lost, stopping");
            speedhold::set_taxi_target(0.0);
            alert(
                &voice,
                Priority::Caution,
                "I've lost the steering. Stopping. Your aircraft.",
            );
            gen.lower_flag();
            break;
        }
        // Speed: fold in any external retarget, then write the schedule.
        match speedhold::taxi_target() {
            Some(t) if (t - last_sched).abs() > 0.01 => {
                // Someone else moved the target: a spoken "stop" parks the
                // schedule; a spoken pace ("go", "taxi at 12") becomes the
                // new base.
                if t < 1.0 {
                    ext_stopped = true;
                } else {
                    base = t;
                    ext_stopped = false;
                }
                last_sched = t;
            }
            _ => {}
        }
        if !ext_stopped {
            let mut sched = schedule_kts(base, &route, progress, gs_kts);
            if join_allow.unwrap_or(0.0) > 0.0 {
                sched = sched.min(route.pace.join_cap_kts(err));
            }
            // Applied every tick, joined or not: the route's bends know nothing
            // about where the aircraft actually is, and the join cap lets go
            // too early to cover it.
            sched = sched.min(route.pace.track_cap_kts(lateral, err));
            if (sched - last_sched).abs() > 0.01 {
                speedhold::set_taxi_target(sched);
                last_sched = sched;
            }
        }
        pacer.wait(std::time::Duration::from_millis(DRIVE_TICK_MS)).await;
    }
    // Politeness on the way out — expiry would do it within the second.
    let _ = xp.set_float(TAXI_STEER, 0.0).await;
    // The brake-steer datarefs do NOT expire like the plugin channel does, so
    // a braked wheel left behind would drag forever. Every exit is a `break`
    // to here (checked — no returns inside the loop), so one release covers
    // them all.
    if bs_on {
        let _ = xp.set_float(LEFT_BRAKE, 0.0).await;
        let _ = xp.set_float(RIGHT_BRAKE, 0.0).await;
    }
    // Only the newest drive may clear the flag (the run's rule, [`RUN`]).
    gen.lower_flag();
    tracing::info!(gen = gen.id, "auto taxi stopped");
}

/// A castoring nose's yaw on its brakes, for the aligning pivot's tests: the
/// brake command (signed, + right) reaches the wheel after `delay_s` through a
/// `tau_s` lag, the yaw accelerates `k` deg/s^2 per unit of brake (times
/// `authority`, the share the rolling speed gives) and decays at `d` per
/// second. Fitted to the iMac's pirouettes of 2026-09-29 (LEMG 12 SR22
/// 1790690995, LILB 03 Evolution 1790712974): 0.45 of brake built the yaw
/// 25 to 40 deg/s per second, a brake let go turned the yaw 0.4 to 0.8 s
/// later, and the old law on this plant at the 0.42 s tick swings
/// +/-13 degrees at 19 deg/s where the SR22 swung +/-12 at up to 25.
#[path = "autotaxi/yaw_plant_tests.rs"]
#[cfg(test)]
pub(crate) mod yaw_plant;

#[cfg(test)]
mod tests;
