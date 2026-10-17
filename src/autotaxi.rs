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

use crate::aircraft::throttle_dataref;
use crate::party_line::TaxiWatch;
use crate::taxi_guide;
use crate::tools::speedhold;
use crate::voice::say_opt as alert;
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

pub(crate) const LAT: &str = crate::datarefs::LATITUDE;
pub(crate) const LON: &str = crate::datarefs::LONGITUDE;
/// TRUE heading, matching the route bearings computed in the local
/// east/north frame — never the magnetic compass.
pub(crate) const PSI_TRUE: &str = crate::datarefs::TRUE_HEADING_DEG;
pub(crate) const GS_MS: &str = crate::datarefs::GROUNDSPEED_MS;
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
pub(crate) const TUG_ATTACHED: &str = crate::datarefs::PUSHBACK_ATTACHED;
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
/// How far behind a hold line the nose stops ([`Route::stop_nose_behind_end`]).
pub const NOSE_BEHIND_LINE_M: f64 = 3.0;
/// Inside this many meters of the stop, steer to the route's FINAL BEARING
/// rather than to a pursuit aim point. Chosen from the live trace: the
/// pursuit error was still honest at 8 m to run (-8°) and had begun walking
/// by 4 m (+1°), so the swap happens before it degrades rather than after.
const STOP_HEADING_HOLD_M: f64 = 8.0;
// The drive's route limits (the established stop, the engage limit, the
// join's room) live with the route geometry in `taxi_guide`, which judges
// the same joins (pillars audit 2026-10-07, stage A).
use crate::taxi_guide::{JOIN_GROW_M, OFF_ROUTE_STOP_M};
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

/// The running drive's route still ahead of us, lat/lon from where we
/// are: the traffic's ground spacing gives way to it as to one of its own
/// movers (`traffic::OwnTarget::route`; EGKK 2026-10-14, a 737 drove onto
/// our stretch unseen and stood nose to nose with us for good). Written by
/// the drive each fix, emptied as it ends ([`RouteAheadGuard`]).
static ROUTE_AHEAD: std::sync::Mutex<Vec<(f64, f64)>> = std::sync::Mutex::new(Vec::new());

/// The route ahead the running drive published; empty with no drive.
pub(crate) fn route_ahead() -> Vec<(f64, f64)> {
    if !ACTIVE.load(Ordering::Relaxed) {
        return Vec::new();
    }
    ROUTE_AHEAD.lock().map(|r| r.clone()).unwrap_or_default()
}

fn publish_route_ahead(way: Vec<(f64, f64)>) {
    if let Ok(mut r) = ROUTE_AHEAD.lock() {
        *r = way;
    }
}

/// Empties [`ROUTE_AHEAD`] however the drive ends.
pub(super) struct RouteAheadGuard;

impl RouteAheadGuard {
    pub(super) fn publish(&self, way: Vec<(f64, f64)>) {
        publish_route_ahead(way);
    }
}

impl Drop for RouteAheadGuard {
    fn drop(&mut self) {
        publish_route_ahead(Vec::new());
    }
}

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
    /// The last tick was held: the time to the next one is excused too. A
    /// tick notes, THEN holds, so the note had already moved `last_t` to
    /// now and the hold alone excused nothing (LEMG 2026-10-13: queued
    /// behind traffic, the deadline gave the taxi up at 656 s).
    held: bool,
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
            held: false,
            last_p: None,
        }
    }
    pub fn note(&mut self, progress_m: f64, t_s: f64) {
        if std::mem::take(&mut self.held) {
            self.excuse_to(t_s);
        }
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
        self.excuse_to(t_s);
        self.held = true;
        self.last_t = Some(t_s);
        if let Some((_, t)) = self.best.as_mut() {
            *t = t_s;
        }
    }
    /// The deadline moved on by the time since the last tick seen.
    fn excuse_to(&mut self, t_s: f64) {
        if let (Some(d), Some(l)) = (self.deadline.as_mut(), self.last_t) {
            *d += (t_s - l).max(0.0);
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
    held: false,
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

// The drive's laws and its loop in their own files, whole and unchanged
// (refactor phase 4); every path stays `crate::autotaxi::...` through these
// re-exports.
mod route;
pub use route::*;
mod steer;
pub use steer::*;
mod align;
pub use align::*;
mod line_up;
pub use line_up::*;
mod run;
pub use run::*;
mod traffic_guard;
pub use traffic_guard::*;

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
