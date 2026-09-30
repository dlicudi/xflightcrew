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
const ON_GROUND: &str = "sim/flightmodel/failures/onground_any";
/// The sim's body yaw rate, deg/s, nose right positive: the aligning pivot's
/// derivative ([`align_pivot_brakes`]).
const YAW_RATE: &str = "sim/flightmodel/position/R";
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
/// The plugin's pilot-feet latch: 1 = the yaw axis moved, override dropped.
const STEER_YIELD: &str = "xcopilot/steer_yield";
/// The tug on the bar, blackboxed per tick as EVIDENCE only: the taxi that
/// followed an early tug release could not turn right for fifteen seconds
/// and the record could not say whether a re-summoned tug had hooked back
/// on (LEMG, 2026-09-12). A slow dataref, so the read is nearly free.
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
/// A corner's radius is the circle through the route this far either side
/// of its vertex.
const BEND_RADIUS_SPAN_M: f64 = 8.0;
/// The deceleration the bend caps are anticipated at: the stop ramp's own.
const BEND_DECEL_MS2: f64 = STOP_DECEL_MS2;
/// How far before a corner's own window its approach is shaped: enough for
/// 18 kt down to 6 at [`BEND_DECEL_MS2`].
const BEND_ENVELOPE_M: f64 = 160.0;
/// No corner cap below a walking pace: the governor needs a speed to hold.
const CORNER_MIN_KTS: f64 = 3.0;

/// The radius of the circle through three points (metres), or infinity when
/// they lie on a line.
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
/// Bearing smoothing window for bend detection, meters (the guide's value:
/// densely-noded fillet arcs must not read as hairpins).
const WINDOW_M: f64 = 15.0;

/// True while the steering loop runs, mirroring `speedhold::ACTIVE` /
/// `autoland::ACTIVE`. Cleared by the loop itself, the release path in
/// main, or the `auto_taxi` tool's off switch.
pub static ACTIVE: AtomicBool = AtomicBool::new(false);
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
pub fn halt() {
    ACTIVE.store(false, Ordering::Relaxed);
    BACKTRACKING.store(false, Ordering::Relaxed);
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
}

impl ProgressWatch {
    /// A drive starting at `t_s` with this much time to finish.
    pub fn start(t_s: f64, deadline_s: f64) -> Self {
        ProgressWatch {
            best: Some((f64::NEG_INFINITY, t_s)),
            deadline: Some(t_s + deadline_s),
            last_t: Some(t_s),
        }
    }
    pub fn note(&mut self, progress_m: f64, t_s: f64) {
        self.last_t = Some(t_s);
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
    /// (along-route m, |bend| degrees, radius m) per real corner, for the
    /// slow-down.
    bends: Vec<(f64, f64, f64)>,
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

/// (along-route m, |bend| degrees, radius m) per real corner of a digested
/// polyline, for the slow-down.
fn route_bends(pts: &[(f64, f64)], cum: &[f64]) -> Vec<(f64, f64, f64)> {
    let mut bends: Vec<(f64, f64, f64)> = Vec::new();
    for i in 1..pts.len().saturating_sub(1) {
        let b_in = taxi_guide::bearing(taxi_guide::point_at(pts, cum, cum[i] - WINDOW_M), pts[i]);
        let b_out = taxi_guide::bearing(pts[i], taxi_guide::point_at(pts, cum, cum[i] + WINDOW_M));
        let delta = taxi_guide::angle_diff(b_out, b_in).abs();
        if delta >= TURN_DEG {
            let radius = circle_radius_m(
                taxi_guide::point_at(pts, cum, cum[i] - BEND_RADIUS_SPAN_M),
                pts[i],
                taxi_guide::point_at(pts, cum, cum[i] + BEND_RADIUS_SPAN_M),
            );
            bends.push((cum[i], delta, radius));
        }
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
        taxi_guide::angle_diff(taxi_guide::bearing(here, aim), heading_true)
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
        let turn = taxi_guide::angle_diff(dir(progress + lookahead), dir(progress));
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
            Some(b) => taxi_guide::angle_diff(b, heading_true),
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
    pub fn bend_cap_kts(&self, progress: f64, gs_kts: f64) -> Option<f64> {
        let lead = (gs_kts * 0.514 * 6.0).clamp(25.0, 90.0);
        self.bends
            .iter()
            .filter(|(at, ..)| progress < at + 15.0 && at - progress < lead + BEND_ENVELOPE_M)
            .map(|&(at, deg, radius)| {
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
/// The band on a nose with NO steering of its own (`aircraft::nose_free`):
/// under 4 kt nothing turned the Evolution, and the propeller's pull swung
/// it 35 degrees off LILB 03's centreline at 3 kt after the line-up
/// (2026-09-25).
const FREE_NOSE_MIN_KTS: f64 = 1.5;
const FREE_NOSE_FULL_KTS: f64 = 4.0;

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
    brake_steer_split_band(steer, gs_kts, false)
}

/// [`brake_steer_split`], with the low-speed band of a nose that cannot
/// steer itself when `free_nose`.
pub fn brake_steer_split_band(steer: f64, gs_kts: f64, free_nose: bool) -> (f64, f64) {
    let (lo, hi) = if free_nose {
        (FREE_NOSE_MIN_KTS, FREE_NOSE_FULL_KTS)
    } else {
        (BRAKE_STEER_MIN_KTS, BRAKE_STEER_FULL_KTS)
    };
    let authority = ((gs_kts - lo) / (hi - lo)).clamp(0.0, 1.0);
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
const PIVOT_BRAKE_MAX: f64 = 0.45;

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
        }
    }

    /// Signed error, positive = the runway is right of the nose.
    pub fn err_deg(&self, psi_true: f64) -> f64 {
        ((self.course_deg - psi_true + 540.0).rem_euclid(360.0)) - 180.0
    }

    pub fn step(
        &mut self,
        t_s: f64,
        psi_true: f64,
        gs_kts: f64,
        yaw_dps: f64,
        rolled_m: f64,
    ) -> AlignStep {
        let t0 = *self.t0.get_or_insert(t_s);
        let dt = self.last_t.map_or(0.0, |l| (t_s - l).clamp(0.0, 1.0));
        self.last_t = Some(t_s);
        let err = self.err_deg(psi_true);
        let led = err - yaw_dps * ALIGN_LEAD_S;
        let settled = gs_kts < ALIGN_SETTLED_KTS && yaw_dps.abs() < ALIGN_SETTLED_DPS;
        if settled && err.abs() <= ALIGNED_DEG {
            self.creeping = false;
            return AlignStep::Done { err_deg: err };
        }
        // The stop this speed still needs, counted against the runway.
        let v = gs_kts / MS_TO_KTS;
        let stop_m = v * ALIGN_STOP_LATENCY_S + v * v / (2.0 * ALIGN_STOP_DECEL_MS2);
        let out_of_bounds = if t_s - t0 > ALIGN_MAX_S {
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
        let want = if self.creeping { !close } else { off };
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
        if want
            && gs_kts > ALIGN_TRIM_MIN_KTS
            && err.abs() > ALIGN_QUIET_DEG
            && yaw_dps * err.signum() < ALIGN_TRIM_TURNING_DPS
        {
            self.trim = (self.trim + ALIGN_TRIM_PER_DEG_S * err * dt)
                .clamp(-ALIGN_TRIM_MAX, ALIGN_TRIM_MAX);
        }
        let (bl, br) = align_pivot_brakes(err, yaw_dps);
        let mut b = (br - bl + self.trim).clamp(-PIVOT_BRAKE_MAX, PIVOT_BRAKE_MAX);
        if b * yaw_dps > 0.0 && yaw_dps.abs() > ALIGN_RATE_MAX_DPS {
            b = 0.0;
        }
        let (bl, br) = if b >= 0.0 { (0.0, b) } else { (-b, 0.0) };
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
            let intercept = taxi_guide::angle_diff(hs.rem_euclid(360.0), leg).abs();
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
/// this far down (the drive has been seen to run 5 m past it at the
/// backtrack's pace), this far off the centreline with no width known, and
/// this far inside the pavement's edge with one, never nearer the
/// centreline than the minimum.
const PIVOT_TURN_START_M: f64 = 50.0;
const PIVOT_POCKET_ALONG_M: f64 = 25.0;
const PIVOT_POCKET_OFFSET_M: f64 = 6.0;
const PIVOT_POCKET_EDGE_M: f64 = 2.0;
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
    let rel = ((psi - course_deg + 540.0).rem_euclid(360.0) - 180.0).to_radians();
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
/// floor ([`corner_aim_floor_m`]: 16 m, a free castoring nose's 12 m), and
/// straights are exactly as they were.
/// Test-only since the live loop takes the floor per nose
/// ([`steer_look_m`]); the tests pin the 16 m baseline.
#[cfg(test)]
pub fn bend_lookahead_cap_m(gs_kts: f64, turn_dps: f64) -> f64 {
    bend_lookahead_cap_floor_m(gs_kts, turn_dps, BEND_AIM_FLOOR_M)
}

/// [`bend_lookahead_cap_m`] with the corner's own floor: see
/// [`corner_aim_floor_m`].
pub fn bend_lookahead_cap_floor_m(gs_kts: f64, turn_dps: f64, floor_m: f64) -> f64 {
    let v = gs_kts * 0.514_444;
    if v < 0.5 || turn_dps.abs() < 0.2 {
        return f64::INFINITY;
    }
    let radius_m = v / turn_dps.abs().to_radians();
    (2.0 * radius_m * BEND_MAX_CUT_M).sqrt().max(floor_m)
}
/// The most a gentle bend may be cut, m.
pub const BEND_MAX_CUT_M: f64 = 1.5;
/// The shortest aim a corner is flown on: a shorter one asks for a turn
/// tighter than a nosewheel's lock can fly, and the aircraft runs wide.
pub const BEND_AIM_FLOOR_M: f64 = 16.0;
/// The shortest aim a castoring nose with no steering of its own
/// (`aircraft::nose_free`) flies a corner on. It turns on a braked wheel,
/// with no lock to run out of: the SR22 turned LEMG's ramp corners at 14
/// deg/s at 3 kt, a 6.5 m radius, and still ran 4.2 to 5.3 m INSIDE them
/// (sorties 1790605551 and 1790618416, 2026-09-28), because the 16 m aim
/// reached round the vertex and the pursuit flew the chord. Pure pursuit
/// cuts a 90 degree vertex by about a fifth of its aim, whatever flies it.
pub const FREE_NOSE_AIM_FLOOR_M: f64 = 12.0;

/// The corner aim floor for this nose. Pure.
pub fn corner_aim_floor_m(free_nose: bool) -> f64 {
    if free_nose {
        FREE_NOSE_AIM_FLOOR_M
    } else {
        BEND_AIM_FLOOR_M
    }
}

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
pub fn steer_look_m(
    pace: &Pace,
    gs_kts: f64,
    turn_probe_dps: f64,
    remaining_m: f64,
    free_nose: bool,
) -> f64 {
    lookahead_m_with(gs_kts, pace.lookahead_s)
        .min(bend_lookahead_cap_floor_m(gs_kts, turn_probe_dps, corner_aim_floor_m(free_nose)))
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
    if ACTIVE.swap(true, Ordering::Relaxed) {
        return; // already driving — the tool retargets instead
    }
    let gen = DRIVE_GEN.fetch_add(1, Ordering::SeqCst) + 1;
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
            ACTIVE.store(false, Ordering::Relaxed);
            return;
        };
        rt.block_on(drive(XPlaneClient::new(base_url), voice, route, base_kts, gen));
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

/// Which drive owns [`ACTIVE`]: bumped by every [`start`]. A drive that is
/// finishing clears the flag only while it is still the newest: a
/// backtrack's leg one finished its exit AFTER the supervisor had started
/// leg two, its late clear cancelled leg two 40 ms into the turn, and full
/// auto then lined up from the wrong heading and 136 m down a 600 m strip
/// (LILB 03, 2026-09-25).
static DRIVE_GEN: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

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
    gen: u64,
) -> AlignEnd {
    // The runway it may roll is judged from where the drive stopped.
    let mut judge: Option<LineUpAlign> = None;
    let t0 = std::time::Instant::now();
    let mut frozen_s = 0.0_f64;
    let mut start: Option<(f64, f64)> = None;
    let mut last_creep: Option<bool> = None;
    let mut last_err = 0.0_f64;
    let end = loop {
        if !ACTIVE.load(Ordering::Relaxed)
            || DRIVE_GEN.load(Ordering::SeqCst) != gen
            || !speedhold::taxi_hold_active()
        {
            break AlignEnd::Interrupted;
        }
        if crate::xplane::sim_frozen(xp).await {
            progress_hold();
            frozen_s += 0.5;
            tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            continue;
        }
        let (yield_r, pos, yaw_r) = tokio::join!(
            xp.get_float(STEER_YIELD),
            sim::position::read(xp),
            xp.get_float(YAW_RATE),
        );
        if yield_r.is_ok_and(|v| v > 0.5) {
            crate::blackbox::record("autotaxi", serde_json::json!({"event": "pedal_yield"}));
            crate::flight::note_pilot_takeover("taxi: pedals");
            alert(
                voice,
                Priority::Advisory,
                "Your steering. I still have the throttle and brakes.",
            );
            ACTIVE.store(false, Ordering::Relaxed);
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
        let step = judge.step(t_s, psi, gs_kts, yaw, rolled_m);
        let (bl, br, creep) = match step {
            AlignStep::Align { bl, br, creep } => (bl, br, creep),
            _ => (0.0, 0.0, false),
        };
        crate::blackbox::record(
            "autotaxi",
            serde_json::json!({"event": "line_up_align", "err": last_err, "yaw_dps": yaw,
                               "gs": gs_kts, "bl": bl, "br": br, "creep": creep,
                               "trim": judge.trim, "rolled_m": rolled_m, "t_s": t_s}),
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
        let steer = ((last_err - yaw * ALIGN_LEAD_S) / ALIGN_FULL_DEG).clamp(-1.0, 1.0);
        let _ = tokio::join!(
            xp.set_float(TAXI_STEER, steer),
            xp.set_float(LEFT_BRAKE, bl),
            xp.set_float(RIGHT_BRAKE, br),
        );
        tokio::time::sleep(std::time::Duration::from_millis(150)).await;
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
async fn drive(xp: XPlaneClient, voice: Option<Voice>, mut route: Route, base_kts: f64, gen: u64) {
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
    let mut bs_prev = (0.0f64, 0.0f64);
    // The steer the BRAKES follow ([`BRAKE_STEER_TAU_S`]).
    let mut bs_steer = 0.0f64;
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
        if !ACTIVE.load(Ordering::Relaxed) || DRIVE_GEN.load(Ordering::SeqCst) != gen {
            crate::blackbox::record(
                "autotaxi",
                serde_json::json!({"event": "stopped_from_outside", "superseded": DRIVE_GEN.load(Ordering::SeqCst) != gen}),
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
            let on_ground = xp.get_float(ON_GROUND).await.unwrap_or(0.0) > 0.5;
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
        if crate::xplane::sim_frozen(&xp).await {
            stalled_since = None; // a pause is not a stall
            progress_hold();
            tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            continue;
        }
        // One read burst, not a chain: sequential HTTP awaits stretched the
        // takeoff tick to half a second and bought a slew-limit weave
        // (sortie four) — the same medicine here keeps the steer tick honest.
        // Position, heading and groundspeed as ONE frame saw them (#157).
        let (yield_r, pos, tug_r) = tokio::join!(
            xp.get_float(STEER_YIELD),
            sim::position::read(&xp),
            xp.get_float(TUG_ATTACHED),
        );
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
                ACTIVE.store(false, Ordering::Relaxed);
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
            ACTIVE.store(false, Ordering::Relaxed);
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
                ACTIVE.store(false, Ordering::Relaxed);
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
            crate::aircraft::nose_free(),
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
                        turn_dir = Some(t.dir);
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
                        ACTIVE.store(false, Ordering::Relaxed);
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
            ACTIVE.store(false, Ordering::Relaxed);
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
            ACTIVE.store(false, Ordering::Relaxed);
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
            ACTIVE.store(false, Ordering::Relaxed);
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
                ACTIVE.store(false, Ordering::Relaxed);
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
            ACTIVE.store(false, Ordering::Relaxed);
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
                (((psi - p + 540.0).rem_euclid(360.0)) - 180.0) / dt
            } else {
                0.0
            }
        });
        prev_psi = Some((psi, now));
        yaw_rate = yaw_rate * 0.5 + raw_rate.clamp(-90.0, 90.0) * 0.5;
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
        // brake mid-turn — and a turn is exactly where the command sits still
        // for seconds. (The old gate existed to keep HTTP off the tick, which
        // the join! below already buys: all three control writes go out
        // together, so the whole steering output costs ONE round trip, not
        // three. The 2026-07-28 lesson was about sequential awaits dragging
        // the loop to a second, not about the number of writes.)
        let (bl, br) = if route.pace.brake_steer || crate::aircraft::nose_castors() {
            let pivot = pivot_due(err, gs_kts, join_allow.unwrap_or(0.0) > 0.0);
            bs_steer = smooth_toward(bs_steer, steer, BRAKE_STEER_TAU_S, dt);
            // The turn-rate ceiling holds for every brake turn, not only a
            // pivot: out of a LIME corner at 8 to 12 kt the brake spun the
            // Evolution at 35 deg/s where the bend asked 14 (2026-09-25).
            let (wl, wr) = if pivot {
                pivot_brake_split(steer, gs_kts)
            } else {
                brake_steer_split_band(bs_steer, gs_kts, crate::aircraft::nose_free())
            };
            let k = pivot_rate_scale(if pivot { steer } else { bs_steer }, yaw_rate);
            let (wl, wr) = (wl * k, wr * k);
            let bl = slew_steer(bs_prev.0, wl, Some(BRAKE_STEER_SLEW_PER_S), dt);
            let br = slew_steer(bs_prev.1, wr, Some(BRAKE_STEER_SLEW_PER_S), dt);
            bs_prev = (bl, br);
            bs_on = bl > 0.0 || br > 0.0;
            (bl, br)
        } else {
            (0.0, 0.0)
        };
        crate::blackbox::record(
            "autotaxi",
            serde_json::json!({
                // `lat_m` is SIGNED — right of the route is positive. It was
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
            }),
        );
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
            ACTIVE.store(false, Ordering::Relaxed);
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
        tokio::time::sleep(std::time::Duration::from_millis(150)).await;
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
    // Only the newest drive may clear the flag (see [`DRIVE_GEN`]).
    if DRIVE_GEN.load(Ordering::SeqCst) == gen {
        ACTIVE.store(false, Ordering::Relaxed);
    }
    tracing::info!(gen, "auto taxi stopped");
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
#[cfg(test)]
pub(crate) mod yaw_plant {
    pub struct Plant {
        pub k: f64,
        pub d: f64,
        pub delay_s: f64,
        pub tau_s: f64,
        /// Heading error, degrees (+ = the course is right of the nose).
        pub err: f64,
        /// Yaw rate, deg/s (+ = nose right, reducing a + error).
        pub yaw: f64,
        brake: f64,
        t: f64,
        hist: Vec<(f64, f64)>,
    }

    impl Plant {
        pub fn new(k: f64, d: f64, err: f64, yaw: f64) -> Self {
            Plant { k, d, delay_s: 0.3, tau_s: 0.3, err, yaw, brake: 0.0, t: 0.0, hist: Vec::new() }
        }

        /// Hold the signed brake command `cmd` for `dt` seconds.
        pub fn run(&mut self, cmd: f64, authority: f64, dt: f64) {
            self.hist.push((self.t, cmd));
            let h = 0.01;
            let end = self.t + dt;
            while self.t < end - 1e-9 {
                let delayed = self
                    .hist
                    .iter()
                    .rev()
                    .find(|(tt, _)| *tt <= self.t - self.delay_s)
                    .map_or(0.0, |(_, c)| *c);
                self.brake += (delayed - self.brake) * h / self.tau_s;
                self.yaw += (self.k * self.brake * authority - self.d * self.yaw) * h;
                self.err -= self.yaw * h;
                self.t += h;
            }
        }
    }

    /// The signed command of a (left, right) pair.
    pub fn signed((l, r): (f64, f64)) -> f64 {
        r - l
    }
}

#[cfg(test)]
mod tests {
    /// LEMG 31 (2026-09-28, issue #141): the rollout at 16.1 kt (state t
    /// 1790585617.7), then 0.5 s later on 12/30's centreline at 0.1 kt after
    /// the sortie runner's reload. The drive then read the 1,293 m as a join
    /// 1,033 m off its line and a stand drive 492 m off route.
    #[test]
    fn a_jump_in_position_is_a_relocation_never_a_movement() {
        let rollout = (36.67410120039185, -4.49797079757389);
        let reloaded = (36.68532005, -4.49417);
        let jump = super::relocation_jump_m(rollout, reloaded, 0.5).expect("a relocation");
        assert!((1250.0..1350.0).contains(&jump), "{jump:.0} m");
        // The second reload, rwy 13's threshold 6 s later, is one too.
        assert!(
            super::relocation_jump_m(reloaded, (36.68428873856669, -4.512249603370898), 6.0)
                .is_some()
        );
        // Real driving never is: the rollout's own 1.5 s at 37 kt (state
        // t 1790585614.3 -> .8), a 25 kt taxi over a 3 s read gap, standing still.
        assert!(super::relocation_jump_m(
            (36.67374691174619, -4.497480866034466),
            (36.67397931820287, -4.497803976111265),
            1.5
        )
        .is_none());
        let three_s_at_25_kts = (36.6741 + 38.6 / 111_320.0, -4.4980);
        assert!(super::relocation_jump_m((36.6741, -4.4980), three_s_at_25_kts, 3.0).is_none());
        assert!(super::relocation_jump_m(rollout, rollout, 0.0).is_none());
    }

    #[test]
    fn a_stuck_taxi_says_what_the_log_can_show() {
        assert!(super::stagnant_line(true, true).contains("grass"));
        assert!(super::stagnant_line(false, true).contains("hard over"));
        assert!(!super::stagnant_line(false, false).contains("nose"));
    }

    #[test]
    fn runway_spans_merge_and_the_light_leads_the_crossing() {
        use super::{on_runway_span, runway_spans};
        // Legs: taxiway 0-100, runway 100-140, runway 140-160, taxiway 160-300.
        let seg = vec![
            ("A".to_string(), false),
            ("09/27".to_string(), true),
            ("09/27".to_string(), true),
            ("B".to_string(), false),
        ];
        let cum = [0.0, 100.0, 140.0, 160.0, 300.0];
        let spans = runway_spans(&seg, &cum);
        assert_eq!(
            spans,
            vec![(100.0, 160.0)],
            "contiguous runway legs are one span"
        );
        assert!(!on_runway_span(&spans, 50.0), "well before: off");
        assert!(on_runway_span(&spans, 65.0), "40 m lead: on");
        assert!(on_runway_span(&spans, 150.0), "on the runway: on");
        assert!(!on_runway_span(&spans, 161.0), "clear: off");
        assert!(
            runway_spans(&[], &cum).is_empty(),
            "a route with no flags has no spans"
        );
    }

    use super::*;

    const LAT0: f64 = 36.0;
    const LON0: f64 = -5.0;
    /// Lat/lon from meters east/north of the origin — the inverse of
    /// [`Geo::to_m`], so routes read like a taxi diagram.
    fn ll(x_m: f64, y_m: f64) -> (f64, f64) {
        (
            LAT0 + y_m / 111_320.0,
            LON0 + x_m / (111_320.0 * LAT0.to_radians().cos()),
        )
    }

    /// East 200 m, left turn north, 300 m to the holding point — the guide's
    /// test diagram, minus the runway crossing.
    fn watch() -> TaxiWatch {
        TaxiWatch {
            path: vec![ll(0.0, 0.0), ll(200.0, 0.0), ll(200.0, 300.0)],
            via: "Whiskey, November".into(),
            seg: vec![("W".into(), false), ("N".into(), false)],
            holding_point: "N1".into(),
            lead_on: Vec::new(),
        }
    }

    /// #190's route as cleared from the LEMG gate (replayed from the install
    /// with `nav::plan_to_runway`, today's planner): the stand, 81.6 m back
    /// on 155.3 to the taxiway node, then the taxiway on 222.2 for 250 m.
    fn lemg_190_route() -> Vec<(f64, f64)> {
        let node: (f64, f64) = (36.6726900, -4.4890590);
        let m_lon = 111_320.0 * node.0.to_radians().cos();
        let (s, c) = (222.2_f64.to_radians().sin(), 222.2_f64.to_radians().cos());
        let mut path = vec![(36.67335705897913, -4.489441674810065), node];
        for k in 1..=50 {
            let d = 5.0 * k as f64;
            path.push((node.0 + d * c / 111_320.0, node.1 + d * s / m_lon));
        }
        path
    }

    /// #190, LEMG (iMac 2026-09-29, sortie 1790712526, a Citation X): the tug
    /// let go 51.5 m down the lane, nose 327.8 (`state` at the engage), and
    /// the join aimed down the stand's own lane 149 degrees left (`join`
    /// err -149.05, lateral 6.94 m, remaining 3558.8 of 3614.9). The planned
    /// turn goes left onto the taxiway ahead, meeting it within 45 degrees,
    /// and the drive's first aim after it sits ahead of the nose.
    #[test]
    fn a_join_behind_the_wing_line_is_a_planned_turn_onto_the_taxiway() {
        let (lat, lon, psi) = (36.6729659291503, -4.489131668721906, 327.8);
        let path = lemg_190_route();
        let w = TaxiWatch {
            path: path.clone(),
            via: String::new(),
            seg: vec![(String::new(), false); path.len() - 1],
            holding_point: "HW1".into(),
            lead_on: Vec::new(),
        };
        let route = Route::build(&w).expect("a route");
        // The recorded join, replayed: the plain projection onto the lane,
        // the aim 16 m on, well behind the wing line.
        let (progress, off) = route.fix_signed_facing(lat, lon, psi).expect("a fix");
        let err = route.heading_err_deg(lat, lon, psi, progress, 16.0);
        // 56 m down the route as flown (3558.8 of 3614.9), 51 m on
        // today's replay: the lane either way.
        assert!(progress > 45.0 && progress < 60.0, "{progress}");
        assert!((off.abs() - 6.9).abs() < 1.0, "{off}");
        assert!(err < -130.0, "{err}");
        assert!(join_turn_applies(err, false, Some(16_000.0), Some(false), false));
        let t = join_turn(&route.path, lat, lon, psi, join_turn_radius_m(Some(16_000.0)), |_, _| None)
            .expect("a turn fits on an open apron");
        assert_eq!(t.dir, -1.0, "left, the short way onto 222: {t:?}");
        // The taxiway lies 25 m behind the right wing and a left turn swings
        // away from it, so the turn goes past its heading and meets it at
        // the intercept: 150 degrees of arc, 97 m in all.
        assert!(t.turn_deg > 106.0 && t.turn_deg < 170.0, "{t:?}");
        assert!(t.lead_m < 120.0, "{t:?}");
        assert!(t.join_m > 81.6, "onto the taxiway, not down the lane: {t:?}");
        assert!(t.intercept_deg <= JOIN_INTERCEPT_DEG, "{t:?}");
        assert_eq!(t.lead[0], (lat, lon), "the turn starts where we stand");
        let rebuilt = route.with_lead(&t.lead, t.join_m).expect("drivable");
        let (p2, off2) = rebuilt.fix_signed(lat, lon).expect("a fix");
        assert!(p2 < 1.0 && off2.abs() < 0.5, "{p2} {off2}");
        let err2 = rebuilt.heading_err_deg(lat, lon, psi, p2, 16.0);
        assert!(err2 < 0.0 && err2 > -60.0, "the aim is round the turn ahead: {err2}");
        assert!(rebuilt.total < route.total, "the lane behind is not driven");
    }

    /// No planned turn stays on the pavement: none, and the drive refuses.
    /// Only drawn pavement refuses (a field without it is no verdict).
    #[test]
    fn a_join_turn_off_the_pavement_is_refused() {
        let (lat, lon, psi) = (36.6729659291503, -4.489131668721906, 327.8);
        let path = lemg_190_route();
        assert!(join_turn(&path, lat, lon, psi, 15.0, |_, _| Some(false)).is_none());
        assert!(join_turn(&path, lat, lon, psi, 15.0, |_, _| Some(true)).is_some());
        assert!(!JOIN_REFUSED_LINE.contains('\u{2014}') && !JOIN_REFUSED_LINE.contains('\u{2013}'));
    }

    /// Who gets a planned join: a heavy that does not pivot, off a runway,
    /// with the route behind the wing line. The SR22 and the light singles
    /// keep the pursuit's own turn; a line-up or turn-round plans its own.
    #[test]
    fn a_planned_join_is_for_a_heavy_that_does_not_pivot() {
        assert!(join_turn_applies(-149.0, false, Some(16_000.0), Some(false), false));
        assert!(join_turn_applies(149.0, false, None, None, false), "unread is heavy");
        assert!(!join_turn_applies(-60.0, false, Some(16_000.0), None, false), "a taxi turn");
        assert!(!join_turn_applies(-149.0, true, Some(16_000.0), None, false), "pivots");
        assert!(!join_turn_applies(-149.0, false, Some(1_111.0), None, false), "light");
        assert!(!join_turn_applies(-149.0, false, Some(79_000.0), Some(true), false), "runway");
        assert!(!join_turn_applies(-149.0, false, Some(79_000.0), None, true), "own geometry");
        assert_eq!(join_turn_radius_m(Some(79_000.0)), 15.0);
        assert_eq!(join_turn_radius_m(Some(230_000.0)), 25.0);
    }

    /// The same join over LEMG's drawn pavement, read from the install.
    /// `cargo test --release -- --ignored live_lemg_190_join_turn --nocapture`
    #[test]
    #[ignore] // needs the real X-Plane install
    fn live_lemg_190_join_turn_stays_on_the_pavement() {
        let apt = &crate::tools::autoland::xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
        let Ok(Some(net)) = atc::nav::find_taxi_net(apt, "LEMG") else {
            return;
        };
        let (lat, lon, psi) = (36.6729659291503, -4.489131668721906, 327.8);
        let t = join_turn(&lemg_190_route(), lat, lon, psi, 15.0, |a, b| net.on_pavement(a, b))
            .expect("the apron has room for the turn");
        println!("{t:?}");
        assert_eq!(t.dir, -1.0);
    }

    /// LSZS 03 (SR22, 2026-09-24): the second line-up drive joined 27.4 m
    /// off its route, never captured, and ran the projection to the end:
    /// its last ticks read 0.4 m to go and -27.44 m off the line, and it
    /// declared the holding point. Full auto found the runway 34 degrees off
    /// and gave up. An arrival is ON the line; and full auto's own check
    /// for a hold reached by hand (a straight 60 m from the route's end,
    /// which a stop 27 m beside it passed) is along the route and on it.
    #[test]
    fn the_hold_is_reached_on_the_line_not_beside_it() {
        assert!(!arrived_on_line(-27.44), "the recorded stop is no arrival");
        assert!(!arrived_on_line(27.44));
        // LEMG 13's hold (2026-09-26) ended 0.05 m off: an arrival.
        assert!(arrived_on_line(0.05));
        let r = Route::build(&watch()).expect("route");
        // 27 m beside the hold, level with it: 27 m from the end in a
        // straight line, which the old check took for the hold.
        let (lat, lon) = ll(173.0, 297.0);
        let (to_go, lateral) = route_end_fix(&r, lat, lon).expect("a fix");
        assert!((lateral - 27.0).abs() < 1.0, "{lateral}");
        assert!(!at_the_hold(to_go, lateral), "{to_go} to go, {lateral} off");
        // Taxied by hand to 40 m short, on the paint: at the hold.
        let (lat, lon) = ll(200.5, 260.0);
        let (to_go, lateral) = route_end_fix(&r, lat, lon).expect("a fix");
        assert!(at_the_hold(to_go, lateral), "{to_go} to go, {lateral} off");
        // On the paint but 200 m short: not there yet.
        let (lat, lon) = ll(200.0, 100.0);
        let (to_go, lateral) = route_end_fix(&r, lat, lon).expect("a fix");
        assert!(!at_the_hold(to_go, lateral), "{to_go} to go");
        assert!(off_line_stop_line(-27.44).contains("27 metres off the line"));
    }

    #[test]
    fn capped_aim_drives_the_final_corner_instead_of_chording_it() {
        // A short L: 60 m east, then a left corner, 18 m north to the stop —
        // the hold-short-after-a-corner shape (LXGB Echo). Aircraft 5 m
        // before the corner, heading east along the paint.
        let watch = TaxiWatch {
            path: vec![ll(0.0, 0.0), ll(60.0, 0.0), ll(60.0, 18.0)],
            via: String::new(),
            seg: vec![(String::new(), false), (String::new(), false)],
            holding_point: String::new(),
            lead_on: Vec::new(),
        };
        let r = Route::build(&watch).expect("route");
        let (lat, lon) = ll(55.0, 0.0);
        let (progress, lateral) = r.fix(lat, lon).expect("on the route");
        assert!(lateral < 1.0);
        // The old 16 m minimum aim reaches PAST the corner: the aim point is
        // on the northbound leg, so the error commands an early cut inside
        // the paint. The capped aim (remaining*0.6 min 6) stays on the
        // eastbound leg: drive straight to the corner first.
        let err_chord = r.heading_err_deg(lat, lon, 90.0, progress, 16.0);
        let remaining = r.remaining(progress);
        let capped = lookahead_m(3.0).min((remaining * 0.6).max(6.0));
        assert!(capped < 16.0, "the cap engages near the stop: {capped}");
        let err_capped = r.heading_err_deg(lat, lon, 90.0, progress, capped);
        assert!(
            err_capped.abs() < err_chord.abs() - 5.0,
            "capped aim hugs the paint: capped {err_capped:.1} vs chord {err_chord:.1}"
        );
    }

    #[test]
    fn steering_is_proportional_and_bounded() {
        assert_eq!(steer_cmd_with(0.0, STEER_FULL_DEG), 0.0);
        assert!(
            steer_cmd_with(10.0, STEER_FULL_DEG) > 0.0
                && steer_cmd_with(10.0, STEER_FULL_DEG) < 1.0,
            "right of course, steer right"
        );
        assert!(steer_cmd_with(-10.0, STEER_FULL_DEG) < 0.0);
        assert_eq!(
            steer_cmd_with(90.0, STEER_FULL_DEG),
            1.0,
            "full deflection is the cap"
        );
        assert_eq!(steer_cmd_with(-170.0, STEER_FULL_DEG), -1.0);
    }

    #[test]
    fn pure_pursuit_steers_back_to_the_centreline() {
        let r = Route::build(&watch()).expect("route");
        // On the first leg (eastbound, true bearing 090), 8 m LEFT of the
        // centreline, heading exactly east: the aim point is on the route
        // ahead, so the error must steer RIGHT (positive), back toward it.
        let (lat, lon) = ll(50.0, 8.0);
        let (progress, lateral) = r.fix(lat, lon).expect("fix");
        assert!((lateral - 8.0).abs() < 1.0, "{lateral}");
        let err = r.heading_err_deg(lat, lon, 90.0, progress, lookahead_m(8.0));
        assert!(err > 2.0, "must steer back right, got {err}");
        // Mirror: 8 m right of the line steers left.
        let (lat, lon) = ll(50.0, -8.0);
        let (progress, _) = r.fix(lat, lon).expect("fix");
        let err = r.heading_err_deg(lat, lon, 90.0, progress, lookahead_m(8.0));
        assert!(err < -2.0, "must steer back left, got {err}");
        // On the line, on heading: hands still.
        let (lat, lon) = ll(50.0, 0.0);
        let (progress, _) = r.fix(lat, lon).expect("fix");
        let err = r.heading_err_deg(lat, lon, 90.0, progress, lookahead_m(8.0));
        assert!(err.abs() < 1.0, "{err}");
    }

    #[test]
    fn the_turn_is_anticipated_through_the_aim_point() {
        let r = Route::build(&watch()).expect("route");
        // 10 m short of the left turn at 10 knots: the aim point is already
        // around the corner, so the command leads the turn.
        let (lat, lon) = ll(190.0, 0.0);
        let (progress, _) = r.fix(lat, lon).expect("fix");
        let err = r.heading_err_deg(lat, lon, 90.0, progress, lookahead_m(10.0));
        assert!(
            err < -10.0,
            "aim point should pull left into the turn, got {err}"
        );
    }

    #[test]
    fn schedule_slows_for_bends_and_ramps_to_the_stop() {
        let r = Route::build(&watch()).expect("route");
        // Mid first leg: cruise pace.
        assert_eq!(schedule_kts(10.0, &r, 50.0, 10.0), 10.0);
        // Approaching the 90° corner at 200 m: capped to the turn speed.
        let near_turn = schedule_kts(10.0, &r, 170.0, 10.0);
        assert_eq!(near_turn, TURN_KTS, "{near_turn}");
        // Mid-turn (just past the vertex): still capped.
        assert_eq!(schedule_kts(10.0, &r, 210.0, 6.0), TURN_KTS);
        // Well past it on the straight: back to pace.
        assert_eq!(schedule_kts(10.0, &r, 300.0, 10.0), 10.0);
        // 20 m from the stop point: ramping down, still moving.
        let ramp = schedule_kts(10.0, &r, r.total - STOP_SHORT_M - 20.0, 8.0);
        assert!(ramp > 2.0 && ramp < 10.0, "{ramp}");
        // At the stop point: zero.
        assert_eq!(schedule_kts(10.0, &r, r.total - STOP_SHORT_M, 3.0), 0.0);
    }

    /// LEMG's Alpha (2026-09-26, sortie 1790441293, a 737): an 80-degree
    /// corner drawn at about a 9 m radius (the route asked 38 deg/s at
    /// 11.5 kt), entered at 11.5 kt against the 10 kt turn cap that arrived
    /// 30 m out; full lock gave 14 to 22 deg/s and it swung 5.8 m wide. With
    /// the 737's yaw rate declared the corner is taken at the rate times the
    /// radius, and the approach to it is shaped from well out.
    #[test]
    fn a_tight_corner_is_taken_at_the_yaw_rate_the_airframe_has() {
        let mut path = vec![ll(0.0, 0.0), ll(200.0, 0.0)];
        let (r, turn) = (9.0_f64, 80.0_f64.to_radians());
        // Left turn: centre 9 m north of the arc's start.
        for i in 1..=20 {
            let a = turn * i as f64 / 20.0;
            path.push(ll(200.0 + r * a.sin(), r - r * a.cos()));
        }
        let end = *path.last().unwrap();
        path.push((end.0 + 100.0 / 111_320.0 * turn.sin(), end.1 + 100.0 / 90_000.0 * turn.cos()));
        let watch = TaxiWatch {
            seg: vec![(String::new(), false); path.len() - 1],
            path,
            via: String::new(),
            holding_point: String::new(),
            lead_on: Vec::new(),
        };
        let profile = |yaw: Option<f64>| crate::aircraft::TaxiProfile {
            kts: Some(18.0),
            turn_kts: Some(10.0),
            sharp_kts: Some(6.0),
            max_yaw_dps: yaw,
            ..Default::default()
        };
        let mut old = Route::build(&watch).expect("route");
        old.pace = Pace::from_profile(Some(&profile(None)));
        let mut jet = Route::build(&watch).expect("route");
        jet.pace = Pace::from_profile(Some(&profile(Some(15.0))));
        // At the corner: the angle alone says the 10 kt turn pace; the yaw
        // rate says 15 deg/s round 9 m, 4.6 kt.
        assert_eq!(schedule_kts(18.0, &old, 205.0, 10.0), 10.0);
        let at = schedule_kts(18.0, &jet, 205.0, 5.0);
        assert!(at < 5.0 && at >= CORNER_MIN_KTS, "the corner's own pace: {at:.1} kt");
        // And the approach is shaped: 60 m out (recorded 10.5 kt with a 10 kt
        // target) it is coming down, and by 30 m out (where the old 10 kt cap
        // first arrived) it is within a few metres of braking of the corner.
        let out = schedule_kts(18.0, &jet, 140.0, 9.0);
        assert!(out < 10.0, "60 m out: {out:.1} kt");
        let window = schedule_kts(18.0, &jet, 172.0, 5.0);
        assert!(window < 6.5, "30 m out: {window:.1} kt");
        let far = schedule_kts(18.0, &jet, 0.0, 18.0);
        assert!(far > 17.0, "a corner 200 m on barely touches the pace: {far:.1} kt");
    }

    #[test]
    fn an_airframes_own_pace_drives_the_bend_caps() {
        // The 172 default is unchanged by the profile hook existing at all.
        let ga = Route::build(&watch()).expect("route");
        assert_eq!(ga.pace, Pace::default());
        assert_eq!(schedule_kts(10.0, &ga, 170.0, 10.0), TURN_KTS);

        // A jet declares its own: 18 down the straight, 10 through the bends.
        let mut jet = Route::build(&watch()).expect("route");
        jet.pace = Pace::from_profile(Some(&crate::aircraft::TaxiProfile {
            kts: Some(18.0),
            turn_kts: Some(10.0),
            sharp_kts: Some(6.0),
            ..Default::default()
        }));
        assert_eq!(jet.pace.straight, 18.0);
        assert_eq!(schedule_kts(18.0, &jet, 0.0, 18.0), 18.0, "open taxiway");
        // 150 m out the jet is already coming down toward the bend's pace.
        let shaped = schedule_kts(18.0, &jet, 50.0, 18.0);
        assert!(shaped < 18.0 && shaped > 10.0, "{shaped}");
        assert_eq!(schedule_kts(18.0, &jet, 170.0, 18.0), 10.0, "into the bend");
        // The stop is still a stop, at any pace.
        assert_eq!(schedule_kts(18.0, &jet, jet.total - STOP_SHORT_M, 3.0), 0.0);
        // Turn-arounds stay at the sharp crawl on both airframes.
        assert_eq!(jet.pace.join_cap_kts(-170.0), 6.0);
    }

    #[test]
    fn a_partial_or_absurd_taxi_profile_keeps_safe_figures() {
        // Omitted fields keep the GA defaults, field by field.
        let partial = Pace::from_profile(Some(&crate::aircraft::TaxiProfile {
            kts: Some(15.0),
            turn_kts: None,
            sharp_kts: None,
            ..Default::default()
        }));
        assert_eq!(partial.straight, 15.0);
        assert_eq!(partial.turn, TURN_KTS);
        assert_eq!(partial.sharp, SHARP_KTS);
        // A typo can't become a takeoff roll with the tower's route as a plan.
        let absurd = Pace::from_profile(Some(&crate::aircraft::TaxiProfile {
            kts: Some(180.0),
            turn_kts: Some(90.0),
            sharp_kts: Some(0.0),
            ..Default::default()
        }));
        assert_eq!(absurd.straight, 25.0);
        assert_eq!(absurd.turn, 25.0);
        assert_eq!(absurd.sharp, 2.0);
        // No profile at all is the GA pace.
        assert_eq!(Pace::from_profile(None), Pace::default());
    }

    #[test]
    fn stop_ramp_is_gentle_and_finishes() {
        assert_eq!(stop_ramp_kts(0.0), 0.0);
        assert_eq!(stop_ramp_kts(0.4), 0.0, "inside the stop zone is stopped");
        assert!(
            stop_ramp_kts(2.0) >= CREEP_KTS,
            "creep floor carries the last meters"
        );
        let (near, far) = (stop_ramp_kts(10.0), stop_ramp_kts(40.0));
        assert!(near < far, "closer is slower: {near} vs {far}");
        assert!(far < 13.0, "40 m out needs no takeoff-roll pace: {far}");
    }

    /// The schedule must be able to STOP the aircraft inside the window the
    /// arrival test is watching, or every leg overshoots its own end.
    ///
    /// Luton 2026-07-28: a 2 kt creep held to 0.5 m to go, against arrival at
    /// `remaining <= 1 m && gs < 1.5 kt`. The schedule was still commanding
    /// 2 kt through the window that wanted under 1.5, and 2 kt cannot stop in
    /// half a metre at the ramp's own 0.5 m/s^2. Three SF50 line-up legs ran
    /// past their stop points — `remaining_m` pinned at -5, still rolling,
    /// steering already faded out by `stop_straighten` so the nose drifted —
    /// and full auto rightly called the line-up a failure.
    #[test]
    fn the_stop_ramp_can_actually_stop_before_the_arrival_window() {
        // Commanded stop no later than the arrival test's own 1 m.
        const ARRIVAL_M: f64 = 1.0;
        const ARRIVAL_KTS: f64 = 1.5;
        assert_eq!(stop_ramp_kts(ARRIVAL_M), 0.0, "zero by the arrival window");
        // The creep must be slow enough to stop within the zero distance at
        // the ramp's own deceleration, before any braking lag.
        let v = CREEP_KTS / MS_TO_KTS;
        let stopping_m = v * v / (2.0 * 0.5);
        assert!(
            stopping_m < STOP_ZERO_M,
            "creep of {CREEP_KTS} kt needs {stopping_m:.2} m but only gets {STOP_ZERO_M}"
        );
        // ...and the creep must still be under the arrival test's speed, so
        // reaching the point is enough to be judged arrived.
        const {
            assert!(CREEP_KTS < ARRIVAL_KTS, "the creep must satisfy arrival");
        }
    }

    /// The yoke-heading channel is dead on a castoring-wheel jet at taxi
    /// speed (steer −0.6 for 30 s while the offset grew to 13.8 m — Luton,
    /// 2026-07-28), so the assist brakes the inside wheel. One side only,
    /// ever: the governor's pilot-brake detector reads `min(left, right)`.
    #[test]
    fn brake_steer_brakes_the_inside_wheel_only() {
        // A breath of command is a breath of brake — on a castoring wheel
        // the brake IS the steering, so only jitter stays off it entirely.
        assert_eq!(brake_steer_split(0.04, 10.0), (0.0, 0.0));
        let (_, small) = brake_steer_split(0.15, 10.0);
        assert!(small > 0.0 && small < 0.06, "gentle, not zero: {small}");
        // Near-stopped: no asymmetric lurch, whatever the command.
        assert_eq!(brake_steer_split(0.9, 2.0), (0.0, 0.0));
        // A real correction at taxi speed: inside wheel, bounded, one side.
        let (l, r) = brake_steer_split(0.6, 10.0);
        assert_eq!(l, 0.0, "nose right brakes the RIGHT wheel only");
        assert!(r > 0.05 && r <= BRAKE_STEER_MAX, "{r}");
        let (l2, r2) = brake_steer_split(-0.6, 10.0);
        assert_eq!(r2, 0.0);
        assert!((l2 - r).abs() < 1e-12, "symmetric: {l2} vs {r}");
        // Full authority arrives INSIDE the tracking band, not at a command
        // the loop can never reach: steer sat at ~0.55 for two thirds of the
        // fourth SF50 taxi drawing only 0.07 of brake, and the aircraft held
        // a one-sided 9 m offset the whole way (2026-07-29).
        assert_eq!(r, BRAKE_STEER_MAX, "saturated by {BRAKE_STEER_SAT}: {r}");
        let (_, mid) = brake_steer_split(0.55, 10.0);
        assert!(
            mid > 0.8 * BRAKE_STEER_MAX,
            "tracking-band command near max: {mid}"
        );
        assert!(brake_steer_split(1.0, 60.0).1 <= BRAKE_STEER_MAX);
        // min(left, right) is zero in every case above — the governor can
        // never read the assist as the pilot slowing the aircraft.
        for s in [-1.0, -0.5, 0.5, 1.0] {
            let (a, b) = brake_steer_split(s, 12.0);
            assert_eq!(a.min(b), 0.0);
        }
        // The SF50 declares it; the 172 (linked nosewheel) must not.
        let sf = Pace::from_profile(
            crate::aircraft::load("cirrus_vision_sf50")
                .expect("sf50")
                .taxi
                .as_ref(),
        );
        assert!(sf.brake_steer);
        let c172 = Pace::from_profile(
            crate::aircraft::load("cessna_172sp")
                .expect("172")
                .taxi
                .as_ref(),
        );
        assert!(!c172.brake_steer);
    }

    /// The schedule has to know where the aircraft IS, not just how the paint
    /// bends ahead of it — the Southampton SF50 taxi (2026-07-30).
    #[test]
    fn a_mistracking_aircraft_is_slowed_whatever_the_route_ahead_does() {
        let sf = Pace::from_profile(
            crate::aircraft::load("cirrus_vision_sf50")
                .expect("sf50")
                .taxi
                .as_ref(),
        );
        // On the centreline and pointed along it: the open-taxiway pace,
        // untouched. The cap must never be the thing that slows a good taxi.
        assert_eq!(sf.track_cap_kts(0.5, 2.0), sf.straight);
        assert_eq!(sf.track_cap_kts(TRACK_TURN_LAT_M, 0.0), sf.straight);
        // Drifting off the line, or crossing it at an angle no intercept would
        // ask for: bend pace.
        assert_eq!(sf.track_cap_kts(8.0, 5.0), sf.turn);
        assert_eq!(sf.track_cap_kts(1.0, 25.0), sf.turn);
        // The numbers off the blackbox: 13 m off with 27 deg of crossing angle,
        // where it was doing 12.7 kt on a straight last leg. Sharp pace.
        assert_eq!(sf.track_cap_kts(13.0, 27.0), sf.sharp);
        // Which side is irrelevant.
        assert_eq!(sf.track_cap_kts(1.0, -40.0), sf.sharp);
        // It can only ever slow the schedule down.
        for lat in [0.0, 5.0, 7.0, 13.0, 40.0] {
            for err in [0.0, 10.0, 30.0, 90.0] {
                assert!(sf.track_cap_kts(lat, err) <= sf.straight);
                assert!(sf.track_cap_kts(lat, err) >= sf.sharp);
            }
        }
        // A trainer holding the centreline within a wheel's width is untouched
        // too — this is not a tax on airframes that track well.
        let c172 = Pace::from_profile(
            crate::aircraft::load("cessna_172sp")
                .expect("172")
                .taxi
                .as_ref(),
        );
        assert_eq!(c172.track_cap_kts(2.0, 8.0), c172.straight);
    }

    /// A taxiing aircraft carries people. The lever may not slam.
    #[test]
    fn the_steering_lever_is_rate_limited_when_an_airframe_asks() {
        // No limit declared: unchanged, whatever the demand.
        assert_eq!(slew_steer(0.0, 1.0, None, 0.2), 1.0);
        // Limited: 0.8/s over a 0.2 s tick is 0.16 of travel, not the lot.
        let s = slew_steer(0.0, 1.0, Some(0.8), 0.2);
        assert!((s - 0.16).abs() < 1e-9, "{s}");
        // Symmetric, and it converges rather than ringing past the target.
        assert!((slew_steer(0.0, -1.0, Some(0.8), 0.2) + 0.16).abs() < 1e-9);
        let near = slew_steer(0.5, 0.52, Some(0.8), 0.2);
        assert!(
            (near - 0.52).abs() < 1e-9,
            "small moves pass through: {near}"
        );
    }

    /// The steering law is per-airframe because the PLANT is. Measured over the
    /// blackbox at 4-12 kt, yaw per unit steer is 12.8 deg/s on the 172 and
    /// 6.9 on the SF50 — the castoring nosewheel is the laggier plant, and lag
    /// wants phase lead, not gain.
    #[test]
    fn the_sf50_taxis_more_gently_than_the_airframe_the_law_was_tuned_on() {
        let sf = Pace::from_profile(
            crate::aircraft::load("cirrus_vision_sf50")
                .expect("sf50")
                .taxi
                .as_ref(),
        );
        let c172 = Pace::from_profile(
            crate::aircraft::load("cessna_172sp")
                .expect("172")
                .taxi
                .as_ref(),
        );
        // The 172 keeps exactly the feel it was tuned with.
        assert_eq!(c172.steer_full_deg, STEER_FULL_DEG);
        assert_eq!(c172.steer_lead_s, STEER_RATE_LEAD_S);
        assert_eq!(c172.steer_slew_per_s, None);
        // The SF50 moves the lever less, and leads harder.
        assert!(sf.steer_full_deg > c172.steer_full_deg, "gentler gain");
        assert!(sf.steer_lead_s > c172.steer_lead_s, "more damping");
        let slew = sf.steer_slew_per_s.expect("a rate limit");
        // Generous on purpose: a tight slew is its own limit cycle, and the
        // aircraft's own p90 lever rate over the whole taxi was 0.18/s.
        assert!(
            slew >= 0.5,
            "a tight slew re-creates the oscillation: {slew}"
        );
        // Same error, less lever — that is the thing the cabin feels.
        let err = 18.0;
        assert!(steer_cmd_with(err, sf.steer_full_deg) < steer_cmd_with(err, c172.steer_full_deg));
        // And a longer aim: ~3.5 deg/s of yaw cannot fly the 5 s aim's
        // crossings — it met the centreline 24 degrees angled and S-weaved
        // ±10 m (2026-07-29). Same offset, shallower ask.
        assert!(sf.lookahead_s > c172.lookahead_s);
        assert_eq!(c172.lookahead_s, 5.0, "the 172 keeps its proven aim");
        assert!(
            lookahead_m_with(12.0, sf.lookahead_s) > 1.5 * lookahead_m_with(12.0, 5.0),
            "the SF50 aims well past the shared default at taxi pace"
        );
    }

    /// A drive stopped from outside claims nothing. The latch is what the
    /// taxi-in arrival watcher secures on, and leaving it set is how an
    /// aircraft got parked on a taxiway (BIKF, 2026-09-20).
    #[test]
    fn halting_a_drive_never_leaves_an_arrival_behind_it() {
        use std::sync::atomic::Ordering::Relaxed;
        let _g = crate::mode::TEST_LOCK.lock();
        ACTIVE.store(true, Relaxed);
        BACKTRACKING.store(true, Relaxed);
        LAST_ARRIVED.store(true, Relaxed);
        halt();
        assert!(!ACTIVE.load(Relaxed), "the loop is told to stop");
        assert!(!BACKTRACKING.load(Relaxed), "including between its legs");
        assert!(!driving(), "and `driving` agrees");
        assert!(
            !LAST_ARRIVED.load(Relaxed),
            "a drive someone else stopped did NOT arrive"
        );
    }

    /// The BIKF stop, measured: 140 m from the nearest stand with the nose
    /// ninety degrees across its parking heading. The copilot said "At the
    /// gate" there. The test the claim now has to pass is the tug's own.
    #[test]
    fn the_bikf_taxiway_stop_is_not_a_stand() {
        use crate::pushback::on_a_stand;
        // Stand 69, the nearest, 140 m away on heading 000; we sat on 270.
        assert!(!on_a_stand(Some((140.0, 0.0)), 269.7, true));
        // Close enough but still crosswise is no stand either.
        assert!(!on_a_stand(Some((20.0, 0.0)), 269.7, true));
        // Actually on it, nose down the stand: this is a gate.
        assert!(on_a_stand(Some((20.0, 0.0)), 5.0, true));
        // A field that publishes no stands cannot be second-guessed.
        assert!(on_a_stand(None, 269.7, false));
    }

    #[test]
    fn rate_lead_checks_the_swing() {
        // Nose swinging hard through a small error on STRAIGHT paint (the
        // route asks for nothing): the lead commands countersteer before the
        // crest — this is the SR22 snake killer, and it must survive the
        // corner credit untouched.
        assert!(damped_err_deg(5.0, 40.0, 0.0, STEER_RATE_LEAD_S) < 0.0);
        // Steady nose: pure P unchanged, the 172 keeps its proven feel.
        assert_eq!(damped_err_deg(10.0, 0.0, 0.0, STEER_RATE_LEAD_S), 10.0);
        // A gentle commanded bend (~8 deg/s) keeps most of its steer.
        assert!(damped_err_deg(15.0, 8.0, 0.0, STEER_RATE_LEAD_S) > 9.0);
    }

    #[test]
    fn the_lead_does_not_fight_the_turn_the_bend_asked_for() {
        // The EIDW 737 corner, from the trace (2026-09-20): 16.6 deg of
        // pursuit error with the nose already coming round at 10 deg/s,
        // which is the rate that bend requires. The old law damped the whole
        // 10 and commanded a third less steer than the error asked for.
        let err = 16.6;
        let yaw = 10.0;
        let old = err - yaw * STEER_RATE_LEAD_S;
        let new = damped_err_deg(err, yaw, yaw, STEER_RATE_LEAD_S);
        assert_eq!(new, err, "a rate the corner asked for is credited in full");
        assert!(
            steer_cmd_with(new, STEER_FULL_DEG) > 1.4 * steer_cmd_with(old, STEER_FULL_DEG),
            "the bend gets back the steering the derating took: {old} -> {new}"
        );
        // Credit is bounded by the measured rate: a bend asking for MORE than
        // the nose is giving never becomes drive of its own.
        assert_eq!(
            damped_err_deg(err, 4.0, 30.0, STEER_RATE_LEAD_S),
            err,
            "over-credit clamps at the measured rate, it never adds command"
        );
        // And a nose swinging the way the bend does NOT go earns nothing:
        // full damping, exactly as before.
        assert_eq!(
            damped_err_deg(err, 12.0, -9.0, STEER_RATE_LEAD_S),
            err - 12.0 * STEER_RATE_LEAD_S,
            "a swing against the bend keeps the whole lead"
        );
    }

    #[test]
    fn a_bend_asks_for_a_turn_rate_and_a_straight_asks_for_none() {
        // 300 m east, then a 90-degree LEFT onto north — the watch() diagram
        // with a long enough straight to sit well clear of the corner.
        let w = TaxiWatch {
            path: vec![ll(0.0, 0.0), ll(300.0, 0.0), ll(300.0, 300.0)],
            via: String::new(),
            seg: vec![(String::new(), false), (String::new(), false)],
            holding_point: String::new(),
            lead_on: Vec::new(),
        };
        let straight = Route::build(&w).expect("route");
        // Underfoot and aim both on the straight: nothing owed.
        assert_eq!(straight.route_turn_rate_dps(50.0, 40.0, 13.0), 0.0);
        // Aim across the corner: a real rate, signed LEFT (negative).
        let through = straight.route_turn_rate_dps(280.0, 40.0, 13.0);
        assert!(
            through < -5.0,
            "the corner inside the aim asks for a left turn rate, got {through}"
        );
        // And it scales with speed: the same corner taken faster is a harder
        // turn, which is the whole point of crediting a RATE and not an angle.
        let faster = straight.route_turn_rate_dps(280.0, 40.0, 26.0);
        assert!(
            (faster - 2.0 * through).abs() < 0.01,
            "twice the speed is twice the rate: {through} -> {faster}"
        );
    }

    #[test]
    fn lookahead_scales_with_speed_within_bounds() {
        assert_eq!(lookahead_m(0.0), 16.0);
        assert!(lookahead_m(10.0) > 16.0 && lookahead_m(10.0) < 50.0);
        assert_eq!(lookahead_m(60.0), 50.0);
    }

    #[test]
    fn steering_authority_tapers_with_speed_so_a_standstill_breaks_away() {
        assert!(steer_authority(0.0) < 0.4, "parked lock must stay partial");
        assert!(steer_authority(0.0) > 0.2, "but enough to actually turn");
        assert!(
            steer_authority(1.5) > steer_authority(0.5),
            "authority builds with the roll"
        );
        assert_eq!(steer_authority(3.0), 1.0, "full lock once rolling");
        assert_eq!(steer_authority(20.0), 1.0);
    }

    #[test]
    fn a_backwards_start_is_a_join_and_steers_hard_around() {
        let r = Route::build(&watch()).expect("route");
        // On the first leg, ON the centreline, but facing WEST against an
        // eastbound route: near-180 error, full deflection, join granted.
        let (lat, lon) = ll(50.0, 0.0);
        let (progress, lateral) = r.fix(lat, lon).expect("fix");
        let err = r.heading_err_deg(lat, lon, 270.0, progress, lookahead_m(3.0));
        assert!(err.abs() > 150.0, "backwards means a big error, got {err}");
        assert_eq!(
            steer_cmd_with(err, STEER_FULL_DEG).abs(),
            1.0,
            "turn-around wants full deflection"
        );
        assert!(
            join_allowance_m(lateral, err) > 0.0,
            "wrong-facing start must join"
        );
        assert_eq!(
            Pace::default().join_cap_kts(err),
            SHARP_KTS,
            "turn-arounds happen at the sharp pace"
        );
    }

    /// #178, the recorded drive ticks (LEMG 13, sortie
    /// 1790646817-full_auto, 2026-09-29): +877.3 lat_m 8.32 at 5.6 kt, the
    /// next tick 0.3 s on the facing pick read 149.37 m (progress 145.6) and
    /// the join's guard stopped the taxi. The aircraft moved under a metre:
    /// the plain nearest stands. A pick that shrinks the offset, or grows it
    /// no faster than the aircraft moved, is still taken.
    #[test]
    fn a_join_projection_that_jumps_is_not_a_movement() {
        let moved = 5.6 * 0.514 * 0.3;
        let (fix, jumped) =
            join_fix(Some((145.56, -149.37)), Some((5.0, 8.4)), Some(8.32), moved);
        assert!(jumped);
        assert_eq!(fix, Some((5.0, 8.4)));
        assert!(OFF_ROUTE_STOP_M + join_allowance_m(0.0027, -144.98) < 149.37, "it tripped");
        // The facing pick nearer than the last tick: a real join.
        assert_eq!(
            join_fix(Some((120.0, 3.0)), Some((5.0, 8.4)), Some(8.32), moved),
            (Some((120.0, 3.0)), false)
        );
        // Grown by what we moved: taken.
        assert_eq!(
            join_fix(Some((6.0, 9.0)), Some((5.0, 8.9)), Some(8.32), moved),
            (Some((6.0, 9.0)), false)
        );
        // The first tick has nothing to jump from.
        assert_eq!(
            join_fix(Some((145.0, 60.0)), Some((5.0, 8.4)), None, 0.0),
            (Some((145.0, 60.0)), false)
        );
    }

    #[test]
    fn an_off_route_start_is_a_join_and_aims_back_at_the_route() {
        let r = Route::build(&watch()).expect("route");
        // 60 m LEFT of the first leg, heading east (parallel to it): the aim
        // point on the route pulls strongly RIGHT, back toward the pavement.
        let (lat, lon) = ll(50.0, 60.0);
        let (progress, lateral) = r.fix(lat, lon).expect("fix");
        assert!((lateral - 60.0).abs() < 2.0, "{lateral}");
        let err = r.heading_err_deg(lat, lon, 90.0, progress, lookahead_m(5.0));
        assert!(err > 20.0, "must aim back at the route, got {err}");
        let allow = join_allowance_m(lateral, err);
        assert!(
            allow >= lateral + JOIN_GROW_M - 1.0,
            "guard covers the engage distance: {allow}"
        );
    }

    /// LEMG, 2026-09-26 (sortie 1790441293, a 737): the tug never came, the
    /// push was skipped, and the drive turned the aircraft round off its
    /// stand to join a route that started behind it (err -155). The old
    /// latch fired at 14.5 m and -42 degrees, mid-swing, and the scorer
    /// judged the taxi from there: "14.5 m off the line". Recorded `drive`
    /// ticks (seconds from the engage, lat_m, err), from 17 s into the join.
    #[test]
    fn a_turn_round_join_is_judged_once_it_is_on_the_line() {
        const TICKS: &[(f64, f64, f64)] = &[
            (17.10, 13.04, -87.95),
            (17.80, 13.70, -79.77),
            (18.50, 14.28, -70.89),
            (18.90, 14.54, -64.20),
            (19.30, 14.66, -57.44),
            (19.70, 14.67, -50.92),
            (20.00, 14.54, -42.27),
            (20.40, 14.22, -33.33),
            (20.80, 13.74, -21.68),
            (21.20, 13.13, -10.28),
            (21.60, 12.10, -4.26),
            (21.90, 11.32, -4.45),
            (22.40, 10.68, -2.25),
            (22.80, 9.82, 0.27),
            (23.10, 9.12, 2.24),
            (23.50, 8.46, 3.64),
            (23.90, 7.55, 5.50),
            (24.20, 6.88, 6.54),
            (24.60, 6.29, 7.31),
            (25.00, 5.56, 8.38),
            (25.30, 4.94, 9.09),
            (25.70, 4.41, 9.44),
            (26.10, 3.74, 10.23),
            (26.50, 3.25, 10.58),
            (26.90, 2.81, 10.69),
            (27.30, 2.36, 10.73),
            (27.60, 1.99, 10.53),
            (28.10, 1.52, 10.23),
            (28.50, 1.12, 9.60),
            (28.80, 0.82, 8.92),
            (29.20, 0.50, 8.02),
            (29.60, 0.20, 7.35),
            (30.00, -0.08, 6.52),
            (30.40, -0.32, 5.75),
            (30.80, -0.50, 5.06),
            (31.10, -0.70, 2.29),
            (31.60, -0.83, 5.63),
            (32.00, -0.91, 4.78),
            (32.40, -0.94, 3.94),
            (32.80, -0.93, 3.14),
            (33.20, -0.91, 2.50),
            (33.50, -0.88, -2.35),
            (33.90, -0.88, -8.82),
            (34.30, -0.98, -13.92),
            (34.60, -1.24, -15.82),
            (35.00, -1.71, -17.25),
            (35.40, -2.45, -19.14),
            (35.80, -2.88, -18.33),
            (36.20, -2.19, -12.96),
            (36.60, -1.67, -10.68),
            (37.00, -1.45, -8.86),
            (37.40, -0.59, -9.58),
            (37.80, 0.51, -7.09),
            (38.10, 1.32, -6.89),
            (38.50, 1.95, -4.68),
            (38.90, 2.29, -4.12),
            (39.30, 1.69, -3.14),
            (39.70, 0.82, -3.03),
            (40.10, 0.52, -5.28),
            (40.40, 0.14, -4.36),
        ];
        let score = |latch: &mut dyn FnMut(f64, f64, f64) -> bool| {
            let mut sc = crate::scorecard::Scorer::new();
            let ev = |e: &str, t: f64| serde_json::json!({"t": t, "kind": "autotaxi", "event": e});
            sc.feed(&ev("join", 0.0));
            let mut joined = false;
            for &(t, lat, err) in TICKS {
                if !joined && latch(lat.abs(), err, t) {
                    joined = true;
                    sc.feed(&ev("joined", t));
                }
                sc.feed(
                    &serde_json::json!({"t": t, "kind": "autotaxi", "event": "drive",
                    "gs": 6.0, "lat_m": lat, "err": err, "bl": 0.0, "br": 0.0, "yaw_dps": 0.0}),
                );
            }
            let out = sc.feed(&ev("holding_point", 41.0));
            out.iter()
                .find(|(k, _)| k == "stage_result")
                .map(|(_, v)| v["max_lat_m"].as_f64().unwrap())
                .unwrap()
        };
        // The old latch fired mid-swing, 14.5 m off and 42 degrees out.
        let old_at = TICKS.iter().find(|&&(_, lat, err)| lat.abs() < 15.0 && err.abs() < 45.0);
        assert!(matches!(old_at, Some(&(_, lat, _)) if lat > 14.0), "old latch: {old_at:?}");
        // The new one waits for the line (the scorer discounts a join's ticks
        // on its own since the same day, so it is scored through it too).
        let mut c = JoinCapture::default();
        let mut fired_at = None;
        let new = score(&mut |lat, err, t| {
            let done = c.step(lat, err, t);
            if done && fired_at.is_none() {
                fired_at = Some(lat);
            }
            done
        });
        assert!(matches!(fired_at, Some(lat) if lat < 3.0), "new latch fired at {fired_at:?} m");
        assert!(new < 3.0, "joined on the line, the taxi scores {new} m");
    }

    #[test]
    fn join_state_machine_grants_and_captures() {
        // On route, aligned: no join, the tight guard from tick one.
        assert_eq!(join_allowance_m(2.0, 5.0), 0.0);
        // Off route or backwards: joined with turning room.
        assert!(join_allowance_m(40.0, 0.0) > 40.0);
        assert!(join_allowance_m(2.0, 170.0) > OFF_ROUTE_STOP_M);
        // Captured only when close AND aligned.
        assert!(join_done(2.0, 10.0));
        assert!(!join_done(5.0, 10.0), "5 m off is not on the line");
        assert!(
            !join_done(2.0, 90.0),
            "close but still turning is not joined"
        );
        assert!(!join_done(20.0, 10.0), "aligned but wide is not joined");
        // A capture that has stopped closing is joined where it settled.
        let mut c = JoinCapture::default();
        assert!(!c.step(6.0, 5.0, 0.0));
        assert!(!c.step(5.9, 4.0, 5.0));
        assert!(c.step(5.8, 3.0, 8.5), "settled at 6 m for 8 s is joined");
        let mut c = JoinCapture::default();
        assert!(!c.step(6.0, 50.0, 0.0));
        assert!(!c.step(6.0, 50.0, 20.0), "settled but still turning is not");
        // Aligned rejoins ride at bend speed, not the sharp crawl.
        assert_eq!(Pace::default().join_cap_kts(20.0), TURN_KTS);
        assert_eq!(Pace::default().join_cap_kts(-170.0), SHARP_KTS);
    }

    #[test]
    fn the_wheel_straightens_into_the_stop() {
        // Stopped (or as good as): the fade owns it, so the brakes never bite
        // a deflected wheel. Well out: steering untouched. Closing: faded.
        const STOPPED: f64 = 0.0;
        assert_eq!(stop_straighten(0.8, 50.0, STOPPED), 0.8);
        let close = stop_straighten(0.8, 4.0, STOPPED);
        assert!(close > 0.0 && close < 0.5, "{close}");
        assert_eq!(stop_straighten(0.8, 1.0, STOPPED), 0.0);
        assert_eq!(stop_straighten(-0.8, 0.5, STOPPED), 0.0, "both directions");
    }

    /// The ramp must open the governor's braking gap while there is still room
    /// to stop, not a couple of metres out.
    ///
    /// `taxi_control` brakes when `(target - gs) - trend*2.5 < -3.0`, i.e. only
    /// once the aircraft is ~3 kt above target, and until the target drops
    /// below 1 kt it is capped at the 0.22 comfort effort. At EGHI, taxiing at
    /// 12 kt base, that gap did not open until 6.8 m to go: the aircraft
    /// crossed the hold line at 6.7 kt and finished five metres past it with
    /// its nose over the yellow.
    #[test]
    fn braking_starts_with_room_to_stop_at_the_hold() {
        const BASE: f64 = 12.0; // the EGHI taxi's own base speed
        const BRAKE_GAP_KTS: f64 = 3.0; // taxi_control's threshold
                                        // Measured achievable deceleration at the gentle comfort effort.
        const ACHIEVABLE_MS2: f64 = 0.6;

        // Where does the aircraft, still doing its base speed, first find
        // itself 3 kt above the commanded target?
        let mut brake_at = 0.0;
        let mut d = 0.0;
        while d < 80.0 {
            if BASE - stop_ramp_kts(d) >= BRAKE_GAP_KTS {
                brake_at = d;
            }
            d += 0.1;
        }
        // It has to be far enough out to actually stop from base speed.
        let need_m = (BASE / MS_TO_KTS).powi(2) / (2.0 * ACHIEVABLE_MS2);
        assert!(
            brake_at > need_m,
            "braking must begin with room: opens at {brake_at:.1} m but \
             stopping from {BASE} kt needs {need_m:.1} m"
        );
        // And with margin, not just barely — the governor has its own loop
        // lag on top of the physics.
        //
        // The margin is only ~1.35x, and that is the honest ceiling of this
        // approach rather than a number to tune toward. Stopping from 12 kt
        // takes ~32 m ONLY because `taxi_control` caps a planned stop at the
        // 0.22 comfort effort until the last metre, and a real aircraft stops
        // from 12 kt in a fraction of that. The deeper fix is to tell
        // `taxi_control` that a stop is planned so it can use its 0.6
        // authority earlier; that re-opens tuning three airframes have already
        // complained about, so it is deliberately not bundled with this.
        assert!(
            brake_at > need_m * 1.25,
            "want margin: {brake_at:.1} m vs {need_m:.1} m needed"
        );

        // The ramp is still a ramp: monotonic, and it still reaches a stop.
        let mut prev = 0.0;
        let mut d = 0.0;
        while d <= 60.0 {
            let v = stop_ramp_kts(d);
            assert!(v >= prev - 1e-9, "monotonic at {d} m");
            prev = v;
            d += 0.5;
        }
        assert_eq!(stop_ramp_kts(STOP_ZERO_M), 0.0, "still stops");
        assert!(stop_ramp_kts(2.0) >= CREEP_KTS, "creep survives");
    }

    /// The fade must not disarm the nose while the aircraft is still rolling.
    ///
    /// `remaining_m` saturates at `-STOP_SHORT_M` as soon as the projection
    /// reaches the end of the polyline, so the distance term pins at -5 with
    /// 2-3 kt still on the clock — and zeroing the command there also kills
    /// the yaw damping inside it. EGHI 2026-07-30: steer 0.00, yaw -2 deg/s
    /// unarrested, cross-track growing 2.45 -> 5.08 m while coasting to a
    /// stop, at the HOLDING POINT and in all three stops of that sortie.
    #[test]
    fn the_nose_is_still_held_while_the_aircraft_is_still_moving() {
        // The exact saturated value the blackbox pins at, at the speed it was
        // actually doing when the steering went to zero.
        let rolling = stop_straighten(0.8, -STOP_SHORT_M, 2.7);
        assert!(
            rolling.abs() > 0.0,
            "steering must survive past the end of the polyline while rolling"
        );
        assert!(
            rolling.abs() >= 0.8 * STRAIGHTEN_FLOOR - 1e-9,
            "and keep enough authority to arrest yaw: {rolling}"
        );
        // Sign is preserved — this only ever scales the command.
        assert!(stop_straighten(-0.8, -STOP_SHORT_M, 2.7) < 0.0);

        // ...but once genuinely stopped it goes to zero, so the original
        // carve-under-braking fix still holds.
        assert_eq!(stop_straighten(0.8, -STOP_SHORT_M, 0.4), 0.0);
        assert_eq!(
            stop_straighten(0.8, -STOP_SHORT_M, STRAIGHTEN_HOLD_KTS),
            0.0
        );

        // It is a FLOOR, never a boost: far out, full steering is unchanged
        // whether moving or not.
        assert_eq!(stop_straighten(0.8, 50.0, 12.0), 0.8);
        // And it never exceeds the command it was given.
        for gs in [0.0, 1.4, 1.6, 3.0, 10.0] {
            for rem in [-5.0, 0.0, 2.0, 8.0, 40.0] {
                assert!(
                    stop_straighten(0.8, rem, gs) <= 0.8 + 1e-9,
                    "never amplifies (rem {rem}, gs {gs})"
                );
            }
        }
    }

    #[test]
    fn line_up_enters_ahead_and_finishes_down_the_centreline() {
        use crate::autoland::{along_track_nm, cross_track_nm, Runway};
        // A 2,000 m runway taking off due north from (0,0); hold-short spot
        // 40 m EAST of the centreline, abeam the numbers.
        let rwy = Runway {
            icao: "TEST".into(),
            id: "36".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 2000.0,
            surface: 1,
            dist_nm: 0.02,
            elev_ft: 0.0,
        };
        let (hlat, hlon) = crate::autoland::project_latlon(0.0, 0.0, 90.0, 40.0 / 1852.0);
        let path = line_up_path(&rwy, hlat, hlon, None).expect("feasible");
        assert_eq!(path.len(), 3);
        // The entry point is ON the centreline, ahead of abeam.
        let (elat, elon) = path[1];
        assert!(
            (cross_track_nm(elat, elon, &rwy) * 1852.0).abs() < 2.0,
            "entry on centreline"
        );
        let entry_past = -along_track_nm(elat, elon, &rwy) * 1852.0;
        assert!(
            entry_past > 20.0 && entry_past < 120.0,
            "ahead of abeam: {entry_past}"
        );
        // The stop point is further down, still leaving plenty of runway.
        let (blat, blon) = path[2];
        let stop_past = -along_track_nm(blat, blon, &rwy) * 1852.0;
        assert!(
            stop_past > entry_past + 40.0,
            "finishes aligned: {stop_past}"
        );
        // Too far off the strip is not a line-up.
        let (flat, flon) = crate::autoland::project_latlon(0.0, 0.0, 90.0, 300.0 / 1852.0);
        assert!(line_up_path(&rwy, flat, flon, None).is_none());
        // Near the far end there is no runway left to line up onto.
        let (nlat, nlon) = crate::autoland::project_latlon(0.0, 0.0, 0.0, 1800.0 / 1852.0);
        let (nlat, nlon) = crate::autoland::project_latlon(nlat, nlon, 90.0, 40.0 / 1852.0);
        assert!(line_up_path(&rwy, nlat, nlon, None).is_none());
    }

    #[test]
    fn line_up_rolls_forward_along_the_heading_not_a_diagonal() {
        use crate::autoland::{bearing_deg, cross_track_nm, Runway};
        // Runway 36 (course 0, due north) from (0,0). Hold-short 40 m EAST of
        // the centreline, abeam the numbers.
        let rwy = Runway {
            icao: "TEST".into(),
            id: "36".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 2000.0,
            surface: 1,
            dist_nm: 0.02,
            elev_ft: 0.0,
        };
        let (hlat, hlon) = crate::autoland::project_latlon(0.0, 0.0, 90.0, 40.0 / 1852.0);
        // Nose pointed due WEST — straight at the runway. Leg one should follow
        // that heading onto the centreline (the taxiway lead-on), NOT cut a
        // ~45 degree diagonal ahead. Bearing of leg one ≈ 270.
        let path = line_up_path(&rwy, hlat, hlon, Some(270.0)).expect("feasible");
        let brg = bearing_deg(path[0].0, path[0].1, path[1].0, path[1].1);
        let off = ((brg - 270.0 + 540.0).rem_euclid(360.0) - 180.0).abs();
        assert!(off < 12.0, "leg one follows the nose (270), got {brg}");
        assert!(
            (cross_track_nm(path[1].0, path[1].1, &rwy) * 1852.0).abs() < 3.0,
            "entry on centreline"
        );

        // Nose nearly aligned with the runway (5 degrees): no clean forward
        // crossing, so it falls back to the diagonal swing — clearly off the
        // near-aligned heading.
        let path2 = line_up_path(&rwy, hlat, hlon, Some(5.0)).expect("feasible");
        let brg2 = bearing_deg(path2[0].0, path2[0].1, path2[1].0, path2[1].1);
        let off2 = ((brg2 - 5.0 + 540.0).rem_euclid(360.0) - 180.0).abs();
        assert!(
            off2 > 20.0,
            "near-aligned nose falls back to the diagonal, got {brg2}"
        );
    }

    /// Real-install probe: what does the LXGB planner actually hand the
    /// line-up at the runway 27 hold? Prints the route tail, the lead-on,
    /// and the drive path the picker would build — evidence, not theory.
    /// `cargo test -- --ignored live_lxgb_line_up --nocapture`
    #[test]
    #[ignore]
    fn live_lxgb_line_up_lead_on() {
        let apt =
            &crate::tools::autoland::xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
        let net = crate::nav::find_taxi_net(apt, "LXGB")
            .unwrap()
            .expect("LXGB network");
        let zones: Vec<String> = vec!["27".into(), "09".into()];
        // Apron, mid-field north side -> the 27 end.
        let plan = net
            .plan(
                36.1535,
                -5.3495,
                36.1515,
                -5.3412,
                crate::nav::HOLD_SHORT_STANDOFF_M,
                &zones,
            )
            .expect("a route");
        println!("holding_point: {} via {:?}", plan.holding_point, plan.via);
        let m = |p: (f64, f64)| {
            let dx = (p.1 + 5.3412) * 111_320.0 * 36.15_f64.to_radians().cos();
            let dy = (p.0 - 36.1515) * 111_320.0;
            (dx.round(), dy.round())
        };
        println!("route tail (m from 27 thr, x=E y=N):");
        for p in plan.path.iter().rev().take(4).rev() {
            println!("  {:?}", m(*p));
        }
        println!("lead_on ({} pts):", plan.lead_on.len());
        for p in &plan.lead_on {
            println!("  {:?}", m(*p));
        }
        assert!(
            !plan.lead_on.is_empty(),
            "zone-trimmed route must keep its lead-on"
        );
        // Now the picker, from the route's actual end (where the taxi stops).
        let &(hlat, hlon) = plan.path.last().unwrap();
        let cands = crate::autoland::nearby_runways(apt, hlat, hlon, 1.0).unwrap();
        let picked = pick_line_up_plan(
            &cands,
            hlat,
            hlon,
            Some("27"),
            400.0,
            false,
            None,
            Some(&plan.lead_on),
            TurnRound::TEARDROP,
        );
        let (rwy, lp) = picked.expect("a line-up plan");
        println!("picked runway {}", rwy.id);
        match lp {
            LineUpPlan::Plain(p) => {
                println!("drive path ({} pts):", p.len());
                for q in &p {
                    println!("  {:?}", m(*q));
                }
                assert!(
                    p.len() > 3,
                    "authored points present, not a 3-point diagonal: {}",
                    p.len()
                );
            }
            LineUpPlan::Backtrack(_) => println!("backtrack picked"),
        }
    }

    /// Le Bourget 07, the whole chain the pilot actually heard fail.
    /// `cargo test -- --ignored live_lfpb_line_up --nocapture`
    ///
    /// Full auto taxied to the cleared holding point, stopped there, and the
    /// line-up answered "I can't find runway zero seven next to us" — the
    /// stop was 169 m off 07's centreline (the line-up's lateral gate is
    /// 150 m) and the lead-on ended 34 m off it (the authored path's gate is
    /// 20 m), because the route had been planned to runway 09's entrance
    /// (2026-09-22). Both gates are the line-up's own and both were right;
    /// the route was wrong, so this test starts at the tower's plan.
    #[test]
    #[ignore]
    fn live_lfpb_line_up_finds_runway_07() {
        let apt =
            &crate::tools::autoland::xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
        let Ok(Some(net)) = crate::nav::find_taxi_net(apt, "LFPB") else {
            return; // no install, no verdict
        };
        let runways = crate::nav::find_runways(apt, "LFPB").unwrap();
        let strip = runways.iter().find(|r| r.ident_a == "07").unwrap();
        let zones: Vec<String> = vec!["07".into(), "25".into()];
        let plan = net
            .plan_to_runway(
                48.95620,
                2.43880,
                strip.lat_a,
                strip.lon_a,
                100.0,
                &zones,
                Some(strip),
            )
            .expect("a route to 07");
        // From the route's own end — where the drive stops, 48.962592,
        // 2.421745 on the day, heading 266.9 (the hold faces west).
        let &(hlat, hlon) = plan.path.last().unwrap();
        let cands = crate::autoland::nearby_runways(apt, hlat, hlon, 1.0).unwrap();
        let (rwy, lp) = pick_line_up_plan(
            &cands,
            hlat,
            hlon,
            Some("07"),
            2000.0,
            false,
            Some(266.9),
            Some(&plan.lead_on),
            TurnRound::TEARDROP,
        )
        .expect("a line-up plan for runway 07");
        assert_eq!(rwy.id, "07");
        match lp {
            LineUpPlan::Plain(p) => {
                println!("lined up via {} authored points", p.len());
                assert!(
                    p.len() > 3,
                    "rides the drawn lead-on, not a synthetic diagonal: {}",
                    p.len()
                );
            }
            LineUpPlan::Backtrack(p) => panic!("a 2,991 m strip needs no backtrack: {p:?}"),
        }
    }

    /// LEMG 13 (2026-09-26, sortie 1790441293, a 737): the line-up rode the
    /// drawn lead-on and its projection jumped 6 m and 11 m forward in single
    /// ticks while the heading error went -6 to -43 degrees; the scorer read
    /// 6.0 m. `cargo test -- --ignored live_lemg_13_lead_on --nocapture`
    #[test]
    #[ignore] // needs the real X-Plane install
    fn live_lemg_13_lead_on_runs_one_way() {
        let apt = &crate::tools::autoland::xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
        let Ok(Some(net)) = atc::nav::find_taxi_net(apt, "LEMG") else {
            return;
        };
        let (hlat, hlon) = (36.684854, -4.511439);
        let rwy = crate::autoland::runway_by_id(apt, hlat, hlon, "13", 5.0, Some("LEMG"))
            .unwrap()
            .expect("LEMG 13");
        let runways = atc::nav::find_runways(apt, "LEMG").unwrap();
        let strip = runways.iter().find(|r| r.ident_a == "13" || r.ident_b == "13").expect("13/31");
        let zone: Vec<String> = vec!["13".into(), "31".into()];
        let plan = net
            .plan_to_runway(36.673087, -4.491381, rwy.thr_lat, rwy.thr_lon, 60.0, &zone, Some(strip))
            .expect("a route");
        println!("holding point {} via {:?}, lead-on {} pts", plan.holding_point, plan.via, plan.lead_on.len());
        // The line-up starts where the taxi stopped: the route's own end.
        let &(hlat, hlon) = plan.path.last().expect("a route end");
        let (path, stop) = line_up_path_from_lead_on(&rwy, hlat, hlon, &plan.lead_on).expect("the lead-on");
        let geo = taxi_guide::Geo::new(path[0].0);
        let pts: Vec<(f64, f64)> = path.iter().map(|&p| geo.to_m(path[0], p)).collect();
        let mut prev_brg: Option<f64> = None;
        let mut worst_turn = 0.0_f64;
        for w in pts.windows(2) {
            let d = taxi_guide::dist(w[0], w[1]);
            let b = taxi_guide::bearing(w[0], w[1]);
            let turn = prev_brg.map_or(0.0, |p| taxi_guide::angle_diff(b, p));
            println!("  {:7.1} {:7.1}  len {:5.1}  brg {:6.1}  turn {:+6.1}", w[1].0, w[1].1, d, b, turn);
            if d > 0.5 {
                worst_turn = worst_turn.max(turn.abs());
                prev_brg = Some(b);
            }
        }
        println!("stop {stop:.0} m, sharpest vertex {worst_turn:.0} deg");
        // Before the paint gaps were filled (`nav::fill_paint_gaps`) the
        // lead-on turned -62, +49, +43 and -63 degrees at consecutive
        // vertices where the network's chord cut the drawn curve; the curve
        // itself never turns more than about 11 degrees at one vertex.
        assert!(worst_turn < 15.0, "a lead-on runs one way: {worst_turn:.0} degrees at one vertex");
    }

    /// #60, LEMG 31 (sorties 1790584715 and 1790585651, 2026-09-28): the taxi
    /// rolled 7.4 m through its holding stop, and the line-up path ran from
    /// the aircraft BACK to the hold node and forward again. At rest the
    /// engage tick's joining projection read 14.8 m along (236.0 of 255.8 to
    /// go) and the next tick's plain one 0 m (250.8): `remaining_m` jumped
    /// 14.9 m with the aircraft stopped. Points behind the aircraft are
    /// dropped, so both projections of a stopped aircraft agree.
    #[test]
    fn a_line_up_from_past_the_hold_never_runs_back_to_it() {
        use crate::autoland::{project_latlon, Runway};
        let rwy = Runway {
            icao: "TEST".into(),
            id: "36".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 2000.0,
            surface: 1,
            dist_nm: 0.02,
            elev_ft: 0.0,
        };
        let at = |a_m: f64, x_m: f64| {
            let (la, lo) = project_latlon(0.0, 0.0, 0.0, a_m / 1852.0);
            project_latlon(la, lo, 90.0, x_m / 1852.0)
        };
        // The stub runs west from the hold node (40 m east) to the centreline.
        let hold = at(100.0, 40.0);
        let lead_on = vec![hold, at(100.0, 25.0), at(100.0, 10.0), at(100.0, 0.0)];
        // Stopped 7.4 m past the node, nose west down the stub.
        let (lat, lon) = at(100.0, 32.6);
        let (path, _) =
            line_up_path_from_lead_on(&rwy, lat, lon, &lead_on).expect("drives the lead-on");
        assert!(!path.contains(&hold), "the node behind the aircraft is not driven back to");
        let plain = crate::taxi_guide::project_signed(&path, lat, lon).unwrap();
        let facing = crate::taxi_guide::project_signed_facing(&path, lat, lon, 270.0).unwrap();
        assert!(
            (plain.0 - facing.0).abs() < 0.5,
            "one answer for a stopped aircraft: plain {plain:?}, facing {facing:?}"
        );
        assert!(plain.0 < 0.5, "we are at the start: {plain:?}");
        // A hold still AHEAD keeps its node, as before.
        let (blat, blon) = at(100.0, 48.0);
        let (before, _) = line_up_path_from_lead_on(&rwy, blat, blon, &lead_on).unwrap();
        assert_eq!(before[1], hold);
    }

    #[test]
    fn line_up_drives_the_authored_lead_on_when_it_is_ours() {
        use crate::autoland::{project_latlon, Runway};
        // Runway 36 from (0,0); hold 40 m east of the centreline, abeam 100 m.
        let rwy = Runway {
            icao: "TEST".into(),
            id: "36".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 2000.0,
            surface: 1,
            dist_nm: 0.02,
            elev_ft: 0.0,
        };
        let at = |a_m: f64, x_m: f64| {
            let (la, lo) = project_latlon(0.0, 0.0, 0.0, a_m / 1852.0);
            project_latlon(la, lo, 90.0, x_m / 1852.0)
        };
        let (hlat, hlon) = at(100.0, 40.0);
        // The authored yellow line: from the hold, curving onto the centreline
        // at 160 m past the threshold (two intermediate points shape the curve).
        let lead_on = vec![
            at(105.0, 38.0),
            at(125.0, 25.0),
            at(150.0, 8.0),
            at(160.0, 0.0),
        ];
        let (path, stop) =
            line_up_path_from_lead_on(&rwy, hlat, hlon, &lead_on).expect("drives the lead-on");
        // The authored points lead the drive path in order; the sharp junction
        // corner is replaced by a sampled fillet arc, then the stop continues
        // down the centreline.
        assert!(
            path.len() >= 8,
            "aircraft + authored + arc + stop: {}",
            path.len()
        );
        assert_eq!(path[1], lead_on[0]);
        assert_eq!(path[2], lead_on[1]);
        assert_eq!(path[3], lead_on[2]);
        assert!(
            (stop - 240.0).abs() < 1.0,
            "junction 160 + align 80: {stop}"
        );
        // The arc stays between the two legs: every sampled point is on the
        // hold side of the centreline and short of the stop.
        for p in &path[4..path.len() - 1] {
            let x = crate::autoland::cross_track_nm(p.0, p.1, &rwy) * 1852.0;
            assert!(
                (-1.0..=40.0).contains(&x),
                "arc point off the corner: x={x}"
            );
        }
        // A lead-on from ANOTHER stub (300 m away) is refused — never drive
        // someone else's painted line.
        let foreign = vec![at(400.0, 38.0), at(460.0, 0.0)];
        assert!(line_up_path_from_lead_on(&rwy, hlat, hlon, &foreign).is_none());
        // A lead-on ending far OFF the centreline is not a runway entry.
        let sideways = vec![at(105.0, 38.0), at(160.0, 60.0)];
        assert!(line_up_path_from_lead_on(&rwy, hlat, hlon, &sideways).is_none());
        // And the picker prefers it over the synthetic diagonal for the same
        // runway.
        let cands = vec![rwy.clone()];
        let (r, plan) = pick_line_up_plan(
            &cands,
            hlat,
            hlon,
            Some("36"),
            400.0,
            false,
            Some(270.0),
            Some(&lead_on),
            TurnRound::TEARDROP,
        )
        .expect("picked");
        assert_eq!(r.id, "36");
        match plan {
            LineUpPlan::Plain(p) => assert_eq!(p[1], lead_on[0], "authored path won"),
            LineUpPlan::Backtrack(_) => panic!("no backtrack expected"),
        }
    }

    /// Luton, 2026-07-27: a full-length departure off a mid-runway holding
    /// point. The backtrack's leg one aimed at a centreline point 30-60 m
    /// nearer the threshold — 23 degrees off the stub the aircraft was
    /// standing on — and drove an MD-82 off the pavement inside fifty metres.
    /// A backtrack has to reach the runway along the DRAWN stub too; only the
    /// direction it turns at the top is ours to choose.
    #[test]
    fn a_backtrack_enters_along_the_drawn_stub_not_across_the_grass() {
        use crate::autoland::{cross_track_nm, project_latlon, Runway};
        let rwy = Runway {
            icao: "TEST".into(),
            id: "36".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 2156.0,
            surface: 1,
            dist_nm: 0.05,
            elev_ft: 0.0,
        };
        let at = |a_m: f64, x_m: f64| {
            let (la, lo) = project_latlon(0.0, 0.0, 0.0, a_m / 1852.0);
            project_latlon(la, lo, 90.0, x_m / 1852.0)
        };
        // Hold 430 m down the strip, 45 m off to the side; the drawn stub runs
        // straight in, perpendicular, to the junction.
        let (hlat, hlon) = at(430.0, 45.0);
        let lead_on = vec![at(430.0, 0.0)];
        let path = backtrack_pocket_path_from_lead_on(&rwy, hlat, hlon, &lead_on, TurnRound::TEARDROP)
            .expect("backtrack via the stub");
        // Every point of the entry, up to the junction, stays on the stub's
        // own along-track station — it never drifts down the runway before
        // reaching the centreline, which is exactly what crossed the grass.
        let a_of = |p: (f64, f64)| -crate::autoland::along_track_nm(p.0, p.1, &rwy) * 1852.0;
        let mut reached = false;
        for p in &path {
            let x = cross_track_nm(p.0, p.1, &rwy) * 1852.0;
            if x.abs() <= 1.0 {
                reached = true;
            }
            if !reached {
                assert!(
                    (a_of(*p) - 430.0).abs() < 25.0,
                    "entry wandered down the runway before reaching it: a={}",
                    a_of(*p)
                );
            }
        }
        assert!(reached, "the entry never reaches the centreline");
        // ...and it still ends in the turn pocket by the numbers.
        let last = *path.last().unwrap();
        assert!(
            a_of(last) < BACKTRACK_TURN_START_M + 5.0,
            "ends at the numbers"
        );
        // The synthetic fallback is what it replaces: that one leaves the stub
        // immediately, which is the bug.
        let synth = backtrack_pocket_path(&rwy, hlat, hlon, TurnRound::TEARDROP).expect("synthetic exists");
        assert!(
            (a_of(synth[1]) - 430.0).abs() > 25.0,
            "the old diagonal aimed down the runway from the start"
        );
    }

    #[test]
    fn line_up_pick_honors_the_cleared_runway() {
        use crate::autoland::Runway;
        let mk = |id: &str, course: f64, thr: (f64, f64), dist: f64| Runway {
            icao: "TEST".into(),
            id: id.into(),
            thr_lat: thr.0,
            thr_lon: thr.1,
            course_deg: course,
            length_m: 2000.0,
            surface: 1,
            dist_nm: dist,
            elev_ft: 0.0,
        };
        // Both ends of one strip: 36 at the near threshold, 18 at the far.
        let far = crate::autoland::project_latlon(0.0, 0.0, 0.0, 2000.0 / 1852.0);
        let cands = vec![mk("36", 0.0, (0.0, 0.0), 0.02), mk("18", 180.0, far, 1.05)];
        let (hlat, hlon) = crate::autoland::project_latlon(0.0, 0.0, 90.0, 40.0 / 1852.0);
        // No clearance on record: geometry picks the near end.
        let (r, _) = pick_line_up(&cands, hlat, hlon, None, None).expect("pick");
        assert_eq!(r.id, "36");
        // The tower cleared 36: same answer, crosschecked.
        assert_eq!(
            pick_line_up(&cands, hlat, hlon, Some("36"), None)
                .unwrap()
                .0
                .id,
            "36"
        );
        // The tower cleared 18 and we're at the 36 end: nothing safe to
        // drive onto — the far end is a taxi clearance, not a line-up.
        assert!(pick_line_up(&cands, hlat, hlon, Some("18"), None).is_none());
    }

    /// LILB 03 (2026-09-25): turned round 38 m past the numbers on a 600 m
    /// strip, leg two used to drive 114 m and leave 448. Now it settles and
    /// straightens inside 60 m.
    #[test]
    fn a_backtracks_leg_two_spends_little_runway() {
        use crate::autoland::{along_track_nm, Runway};
        let rwy = Runway {
            icao: "LILB".into(),
            id: "03".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 600.0,
            surface: 1,
            dist_nm: 0.0,
            elev_ft: 0.0,
        };
        let on = crate::autoland::project_latlon(0.0, 0.0, 0.0, 38.0 / 1852.0);
        let (lat, lon) = crate::autoland::project_latlon(on.0, on.1, 90.0, 3.0 / 1852.0);
        let path = backtrack_leg_two_path(&rwy, lat, lon, TurnRound::TEARDROP).expect("fits");
        let stop = path.last().unwrap();
        let stop_m = -along_track_nm(stop.0, stop.1, &rwy) * 1852.0;
        assert!(stop_m <= 38.0 + 60.0 + 0.5, "stops by {stop_m:.0} m");
        assert!(600.0 - stop_m >= 500.0, "leaves {:.0} m", 600.0 - stop_m);
        // The old hold-short geometry spent nearly twice that.
        let (_, old_stop) = line_up_stops_m(&rwy, lat, lon, None).unwrap();
        assert!(old_stop > stop_m + 40.0, "old {old_stop:.0} vs new {stop_m:.0}");
    }

    #[test]
    fn backtrack_drives_to_a_pocket_beside_the_numbers() {
        use crate::autoland::{along_track_nm, cross_track_nm, Runway};
        // The LXGB shape: a 1,712 m runway, hold-short stub 1,170 m down it
        // (holding point Charlie left 580 m ahead for a 09 departure,
        // 2026-07-22). Runway 36 from (0,0) for test geometry.
        let rwy = Runway {
            icao: "TEST".into(),
            id: "36".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 1712.0,
            surface: 1,
            dist_nm: 0.6,
            elev_ft: 0.0,
        };
        let mid = crate::autoland::project_latlon(0.0, 0.0, 0.0, 1170.0 / 1852.0);
        let (hlat, hlon) = crate::autoland::project_latlon(mid.0, mid.1, 90.0, 40.0 / 1852.0);
        let path = backtrack_pocket_path(&rwy, hlat, hlon, TurnRound::TEARDROP).expect("feasible");
        // The route heads TOWARD the threshold: each point's along-track
        // shrinks, and the pocket ends up beside the numbers, off the
        // centreline (turning room, never a pivot).
        let along: Vec<f64> = path
            .iter()
            .map(|&(a, b)| -along_track_nm(a, b, &rwy) * 1852.0)
            .collect();
        assert!(
            along.windows(2).all(|w| w[1] < w[0] + 1.0),
            "monotonic toward the numbers: {along:?}"
        );
        let (plat, plon) = *path.last().unwrap();
        let pocket_along = -along_track_nm(plat, plon, &rwy) * 1852.0;
        let pocket_off = (cross_track_nm(plat, plon, &rwy) * 1852.0).abs();
        assert!(
            pocket_along < 60.0,
            "pocket beside the numbers: {pocket_along}"
        );
        assert!(
            pocket_off > 8.0 && pocket_off < 20.0,
            "turning room: {pocket_off}"
        );
        // A line-up FROM the pocket is feasible — that's leg two, the 180.
        assert!(
            line_up_path(&rwy, plat, plon, None).is_some(),
            "leg two must exist"
        );
        // Near the numbers already: no backtrack, the plain line-up owns it.
        let near = crate::autoland::project_latlon(0.0, 0.0, 0.0, 60.0 / 1852.0);
        let (nlat, nlon) = crate::autoland::project_latlon(near.0, near.1, 90.0, 40.0 / 1852.0);
        assert!(backtrack_pocket_path(&rwy, nlat, nlon, TurnRound::TEARDROP).is_none());
    }

    #[test]
    fn line_up_plan_backtracks_to_the_numbers_from_a_deep_intersection() {
        use crate::autoland::Runway;
        let rwy = Runway {
            icao: "TEST".into(),
            id: "36".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 1712.0,
            surface: 1,
            dist_nm: 0.6,
            elev_ft: 0.0,
        };
        let cands = vec![rwy];
        // 1,170 m down the strip: a deep intersection on a short field. The
        // plain line-up would leave ~460 m — enough for a 172's roll — but a
        // departure uses the FULL length, so we backtrack to the numbers even
        // when it fits (the LXGB Charlie case the pilot kept flagging).
        let mid = crate::autoland::project_latlon(0.0, 0.0, 0.0, 1170.0 / 1852.0);
        let (hlat, hlon) = crate::autoland::project_latlon(mid.0, mid.1, 90.0, 40.0 / 1852.0);
        assert!(matches!(
            pick_line_up_plan(&cands, hlat, hlon, Some("36"), 400.0, false, None, None, TurnRound::TEARDROP),
            Some((_, LineUpPlan::Backtrack(_)))
        ));
        assert!(matches!(
            pick_line_up_plan(&cands, hlat, hlon, Some("36"), 700.0, false, None, None, TurnRound::TEARDROP),
            Some((_, LineUpPlan::Backtrack(_)))
        ));
        // The pilot asked for the full length in so many words: backtrack
        // even though the plain remaining would do.
        assert!(matches!(
            pick_line_up_plan(&cands, hlat, hlon, Some("36"), 400.0, true, None, None, TurnRound::TEARDROP),
            Some((_, LineUpPlan::Backtrack(_)))
        ));
        // Abeam the numbers a backtrack is impossible — the ask still gets
        // the plain line-up, never a refusal.
        let near = crate::autoland::project_latlon(0.0, 0.0, 0.0, 60.0 / 1852.0);
        let (nlat, nlon) = crate::autoland::project_latlon(near.0, near.1, 90.0, 40.0 / 1852.0);
        assert!(matches!(
            pick_line_up_plan(&cands, nlat, nlon, Some("36"), 700.0, true, None, None, TurnRound::TEARDROP),
            Some((_, LineUpPlan::Plain(_)))
        ));
    }

    #[test]
    fn short_routes_refuse_to_build() {
        let w = TaxiWatch {
            path: vec![ll(0.0, 0.0), ll(15.0, 0.0)],
            via: "A".into(),
            seg: vec![("A".into(), false)],
            holding_point: "A1".into(),
            lead_on: Vec::new(),
        };
        assert!(Route::build(&w).is_none(), "nothing to drive on 15 m");
    }

    /// Every stop turned the aircraft, and the guidance was doing it.
    ///
    /// The pursuit aim point clamps at the route's final point, so once the
    /// aircraft rolls past it the bearing to that point swings BEHIND and the
    /// law commands a hard turn toward a waypoint already passed. EGHI
    /// 2026-07-30, dead on the centreline at the stop point and then:
    ///
    ///   rem  0.04 -> err   9.5     rem -3.6 -> err  38.6
    ///   rem -4.4  -> err  70.2     rem -5.0 -> err 162.6
    ///
    /// 0.07 m off the line to 3.00 m in four seconds, steering pinned. It hit
    /// the hold, the line-up, the backtrack and the taxi-in alike, because all
    /// four drive this one loop.
    /// LIPR Alpha (2026-09-26): the planned route ended on a repeated point,
    /// the zero-length last segment read as due north, and with 8 m to run
    /// the stop steered a 220-degree aircraft full right toward it.
    #[test]
    fn a_repeated_last_point_is_not_a_final_bearing() {
        let w = TaxiWatch {
            path: vec![ll(0.0, 0.0), ll(0.0, -120.0), ll(0.0, -120.0)],
            via: String::new(),
            seg: vec![(String::new(), false), (String::new(), false)],
            holding_point: String::new(),
            lead_on: Vec::new(),
        };
        let r = Route::build(&w).expect("drives");
        let bearing = r.final_bearing_deg().expect("a real segment exists");
        assert!((bearing - 180.0).abs() < 2.0, "due south route, got {bearing:.1}");
    }

    #[test]
    fn the_stop_holds_the_routes_heading_instead_of_chasing_a_passed_point() {
        // A straight route heading due east, 120 m long.
        let w = TaxiWatch {
            path: vec![ll(0.0, 0.0), ll(120.0, 0.0)],
            via: String::new(),
            seg: vec![(String::new(), false)],
            holding_point: String::new(),
            lead_on: Vec::new(),
        };
        let r = Route::build(&w).expect("a straight two-point route drives");
        let bearing = r.final_bearing_deg().expect("a two-point route has one");
        assert!(
            (bearing - 90.0).abs() < 2.0,
            "due east route, got {bearing:.1}"
        );

        // Sitting PAST the end, pointed correctly down the route. The pursuit
        // error is nonsense here; the stop error must be ~0.
        let past = ll(125.0, 0.0);
        let (past_lat, past_lon) = (past.0, past.1);
        let progress = r.total;
        let pursuit = r.heading_err_deg(past_lat, past_lon, 90.0, progress, 6.0);
        let held = r.stop_heading_err_deg(past_lat, past_lon, 90.0, progress, 6.0);
        assert!(
            pursuit.abs() > 90.0,
            "the pursuit error should be wild past the end: {pursuit:.1}"
        );
        assert!(
            held.abs() < 2.0,
            "holding the route heading must read straight: {held:.1}"
        );

        // And it still CORRECTS a genuinely crooked nose rather than going
        // blind — 20° off reads as 20° off, with the sign that unwinds it.
        let crooked = r.stop_heading_err_deg(past_lat, past_lon, 110.0, progress, 6.0);
        assert!(
            (crooked + 20.0).abs() < 2.0,
            "20 deg right of course should command left: {crooked:.1}"
        );

        // The swap only happens near the stop; further out the pursuit still
        // owns the steering, which is what folds cross-track back in.
        assert!(
            STOP_HEADING_HOLD_M > 0.0 && STOP_HEADING_HOLD_M < 20.0,
            "the hold zone is the last few metres, not the whole leg"
        );
    }

    /// EDLP (2026-09-24): with a standing steer bias the proportional law
    /// parks off the line (the SF50 held 4 m on every straight). Flown with
    /// the SF50's own pace through a lagged yaw plant that needs 0.09 of
    /// steer to run straight: the trim takes the offset out, and it does not
    /// start a weave.
    #[test]
    fn a_straight_trim_takes_out_the_offset_a_standing_bias_leaves() {
        let pace = Pace::from_profile(
            crate::aircraft::load("cirrus_vision_sf50")
                .expect("sf50")
                .taxi
                .as_ref(),
        );
        // (final |offset| over the last 20 s, peak |offset| after 60 s)
        let fly = |trimmed: bool| -> (f64, f64) {
            let (bias, k_dps, tau, gs) = (0.09, 6.9, 0.8, 12.0);
            let v = gs * 0.514_444;
            let (mut y, mut psi, mut r, mut prev) = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
            let mut trim = StraightTrim::default();
            let dt = 0.2;
            let (mut late, mut after) = (0.0_f64, 0.0_f64);
            let look = lookahead_m_with(gs, pace.lookahead_s);
            for i in 0..900 {
                let t = i as f64 * dt;
                let err = (-y).atan2(look).to_degrees() - psi;
                let tr = if trimmed { trim.step(y, 0.0, gs, false, 1000.0, dt) } else { 0.0 };
                let want = (steer_cmd_with(
                    damped_err_deg(err, r, 0.0, pace.steer_lead_s),
                    pace.steer_full_deg,
                ) + tr)
                    .clamp(-1.0, 1.0);
                let steer = slew_steer(prev, want, pace.steer_slew_per_s, dt);
                prev = steer;
                r += (k_dps * (steer - bias) - r) * (dt / tau).min(1.0);
                psi += r * dt;
                y += v * psi.to_radians().sin() * dt;
                if t > 60.0 {
                    after = after.max(y.abs());
                }
                if t > 160.0 {
                    late = late.max(y.abs());
                }
            }
            (late, after)
        };
        let (untrimmed, _) = fly(false);
        assert!(untrimmed > 2.0, "the plant must reproduce the offset: {untrimmed:.1} m");
        let (trimmed, peak) = fly(true);
        assert!(trimmed < 0.5, "trimmed out: {trimmed:.2} m");
        assert!(peak < untrimmed, "no weave bigger than the offset it replaced: {peak:.1} m");
    }

    /// LILB (2026-09-25): the Evolution's brake pulsed with every flicker of
    /// the rate lead. Flown on its own pace through a brake-steered yaw plant
    /// that pulls left (0.05 of right brake to run straight), with the yaw
    /// rate read through the loop's own filter from a heading that carries
    /// +/-0.15 degrees of noise: the smoothed brake moves far less, and the
    /// aircraft tracks no worse.
    #[test]
    fn a_smoothed_brake_holds_a_straight_without_pulsing() {
        let pace = Pace::from_profile(
            crate::aircraft::load("lancair_evolution").expect("evot").taxi.as_ref(),
        );
        assert!(pace.brake_steer);
        // (total brake travel over the run, peak |offset| after 30 s)
        let fly = |tau: f64| -> (f64, f64) {
            let (k_dps, pull, tau_plant, gs, dt) = (60.0, 3.0, 0.6, 12.0, 0.2);
            let v = gs * 0.514_444;
            let (mut y, mut psi, mut r) = (1.5_f64, 0.0_f64, 0.0_f64);
            let (mut prev, mut bs, mut yaw, mut prev_meas) = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
            let mut trim = StraightTrim::default();
            let (mut travel, mut last_b, mut peak) = (0.0_f64, 0.0_f64, 0.0_f64);
            let mut seed = 7u64;
            let look = lookahead_m_with(gs, pace.lookahead_s);
            for i in 0..900 {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                let noise = ((seed >> 33) as f64 / (1u64 << 31) as f64 - 0.5) * 0.3;
                let meas = psi + noise;
                yaw = yaw * 0.5 + ((meas - prev_meas) / dt) * 0.5;
                prev_meas = meas;
                let err = (-y).atan2(look).to_degrees() - psi;
                let tr = trim.step(y, 0.0, gs, false, 1000.0, dt);
                let want = (steer_cmd_with(damped_err_deg(err, yaw, 0.0, pace.steer_lead_s), pace.steer_full_deg) + tr)
                    .clamp(-1.0, 1.0);
                let steer = slew_steer(prev, want, pace.steer_slew_per_s, dt);
                prev = steer;
                bs = if tau > 0.0 { smooth_toward(bs, steer, tau, dt) } else { steer };
                let (bl, br) = brake_steer_split(bs, gs);
                let b = br - bl;
                travel += (b - last_b).abs();
                last_b = b;
                r += (k_dps * b - pull - r) * (dt / tau_plant).min(1.0);
                psi += r * dt;
                y += v * psi.to_radians().sin() * dt;
                if i as f64 * dt > 30.0 {
                    peak = peak.max(y.abs());
                }
            }
            (travel, peak)
        };
        let (raw_travel, raw_peak) = fly(0.0);
        let (smooth_travel, smooth_peak) = fly(BRAKE_STEER_TAU_S);
        assert!(
            smooth_travel < raw_travel * 0.6,
            "the brake must move far less: {smooth_travel:.2} vs {raw_travel:.2}"
        );
        assert!(
            smooth_peak <= raw_peak.max(1.0),
            "and track no worse: {smooth_peak:.2} m vs {raw_peak:.2} m"
        );
    }

    #[test]
    fn a_nose_that_cannot_steer_is_brake_steered_down_to_walking_pace() {
        // The LILB line-up's last metres: 3 kt, 0.3 of steer, 30 degrees off.
        assert_eq!(brake_steer_split_band(0.3, 3.0, false), (0.0, 0.0), "the old band: nothing");
        let (_, r) = brake_steer_split_band(0.3, 3.0, true);
        assert!(r > 0.05, "a free nose gets its brake: {r:.3}");
        assert_eq!(brake_steer_split_band(0.3, 1.0, true), (0.0, 0.0), "stopped is the stop's");
    }

    /// LIME 2026-09-25: 23 kt against a 10 kt target, into a corner, off
    /// the pavement and into a wall. That is a stop, not a trim.
    #[test]
    fn a_runaway_taxi_is_stopped() {
        assert_eq!(taxi_overspeed(23.4, 10.0), Some("over_target"));
        assert_eq!(taxi_overspeed(26.0, 30.0), Some("absolute"));
        assert_eq!(taxi_overspeed(17.7, 10.0), None, "7.7 over is the brakes' to fix");
        assert_eq!(taxi_overspeed(12.0, 10.0), None);
        assert_eq!(taxi_overspeed(5.0, 0.0), None, "a stop in progress is the stop's");
    }

    #[test]
    fn a_gentle_bend_shortens_the_aim_and_a_straight_does_not() {
        // LIME: 12 kt, a 3.3 deg/s bend (radius ~107 m).
        let cap = bend_lookahead_cap_m(12.0, -3.3);
        assert!((17.0..19.5).contains(&cap), "{cap}");
        let full = lookahead_m_with(12.0, 9.0);
        assert!(full > 50.0 && full.min(cap) < 20.0);
        // The cut the capped aim leaves: under the limit.
        let r = 12.0 * 0.514_444 / 3.3_f64.to_radians();
        assert!(cap * cap / (2.0 * r) <= BEND_MAX_CUT_M + 1e-9);
        // Straight, or crawling: no cap. A sharp corner: never under 16.
        assert!(bend_lookahead_cap_m(12.0, 0.1).is_infinite());
        assert!(bend_lookahead_cap_m(0.5, 10.0).is_infinite());
        assert_eq!(bend_lookahead_cap_m(8.0, 30.0), 16.0);
    }

    /// The aligning pivot on the fitted plant ([`super::yaw_plant`]): from
    /// the SR22's 10.8 and the Evolution's 24.6 degrees (the roll's first
    /// ticks, 1790690995 and 1790712974, 2026-09-29), at the iMac's 0.3 to
    /// 0.5 s ticks and on a softer and a stiffer plant, it comes round and
    /// STAYS within the aligned band, never swinging more than 2 degrees
    /// through. The old law (full brake from 15 degrees on the error alone,
    /// let go only past 15 deg/s) on the same plant pirouettes, as flown.
    #[test]
    fn the_aligning_pivot_catches_its_swing_on_the_imac_plant() {
        use super::yaw_plant::{signed, Plant};
        let old_law = |err: f64, yaw_diff: f64| {
            if err.abs() <= 3.0 {
                return 0.0;
            }
            let want = (err / 15.0).clamp(-1.0, 1.0);
            let (l, r) = super::pivot_brake_split(want, 2.0);
            signed((l, r)) * super::pivot_rate_scale(want, yaw_diff)
        };
        // As flown: the old law swings through by more than the start.
        let mut p = Plant::new(140.0, 3.0, 10.8, 0.0);
        let (mut prev, mut worst) = (p.err, 0.0_f64);
        for _ in 0..48 {
            let yaw_diff = -(p.err - prev) / 0.42;
            prev = p.err;
            p.run(old_law(p.err, yaw_diff), 1.0, 0.42);
            worst = worst.max(-p.err);
        }
        assert!(worst > 8.0, "the old law pirouettes on this plant: {worst:.1}");

        for (k, d) in [(140.0, 3.0), (70.0, 1.5), (100.0, 1.0), (200.0, 4.0)] {
            for tick in [0.3, 0.42, 0.5] {
                for e0 in [10.8, 24.6, -10.8, 5.0] {
                    let mut p = Plant::new(k, d, e0, 0.0);
                    let mut through = 0.0_f64;
                    let mut settled_at: Option<f64> = None;
                    let mut t = 0.0;
                    while t < 25.0 {
                        p.run(signed(super::align_pivot_brakes(p.err, p.yaw)), 1.0, tick);
                        t += tick;
                        through = through.max(-p.err * e0.signum());
                        if p.err.abs() <= super::ALIGNED_DEG {
                            settled_at.get_or_insert(t);
                        } else {
                            settled_at = None;
                        }
                    }
                    let why = format!("k {k} d {d} tick {tick} from {e0}");
                    assert!(through <= 2.5, "{why}: swung {through:.1} through");
                    let at = settled_at.unwrap_or_else(|| panic!("{why}: never settled"));
                    assert!(at < 20.0, "{why}: settled at {at:.1} s");
                }
            }
        }
    }

    #[test]
    fn the_aligning_pivot_brakes_the_inside_wheel_and_catches_a_swing() {
        // The SR22's roll start: 10.8 off, still: the right (inside) brake.
        let (l, r) = super::align_pivot_brakes(10.8, 0.0);
        assert!(l == 0.0 && r > 0.1, "{l} {r}");
        // Turning into it at 6 deg/s: the lead has already let go and holds
        // the swing on the OTHER brake (10.8 - 12 is past the course).
        let (l, r) = super::align_pivot_brakes(10.8, 6.0);
        assert!(l > 0.0 && r == 0.0, "{l} {r}");
        // Straight and still: nothing.
        assert_eq!(super::align_pivot_brakes(0.5, 0.3), (0.0, 0.0));
        // Never more than a pivot's brake.
        let (l, _) = super::align_pivot_brakes(-170.0, 0.0);
        assert!(l <= 0.45 + 1e-9);
    }

    /// The line-up's end, on the plant with the governor: the SR22 arrived
    /// at LEMG 12 5.0 degrees off its final bearing, yawing away at 1.7
    /// deg/s at 2.1 kt (1790690995, t 1790692240.7), and was declared lined
    /// up; it stopped at 107 true on a 117.7 runway. The Evolution's LILB 03
    /// turn-round arrived 11.8 off (13 at the stop) yawing away at 2.3
    /// (1790712974, t 1790713229.1). Now the drive ends only STOPPED within
    /// [`super::ALIGNED_DEG`], in bounded time and a few metres of runway.
    #[test]
    fn a_line_up_ends_stopped_and_pointing_down_the_runway() {
        use super::yaw_plant::{signed, Plant};
        use super::{AlignStep, LineUpAlign};
        for (course, psi0, yaw0, gs0) in [(117.7, 112.7, -1.7, 2.1), (24.0, 12.2, -2.3, 1.9)] {
            for (k, d) in [(140.0, 3.0), (70.0, 1.5)] {
                let mut judge = LineUpAlign::new(course);
                let e0 = judge.err_deg(psi0);
                let mut p = Plant::new(k, d, e0, yaw0);
                let (mut gs, mut rolled, mut t) = (gs0, 0.0_f64, 0.0_f64);
                let tick = 0.42;
                let end = loop {
                    let psi = course - p.err;
                    match judge.step(t, psi, gs, p.yaw, rolled) {
                        AlignStep::Align { bl, br, creep } => {
                            // The governor: toward the creep at 0.7 kt/s,
                            // or braked to a stop at 1.5 kt/s.
                            let want = if creep { super::ALIGN_CREEP_KTS } else { 0.0 };
                            gs = if gs < want { (gs + 0.7 * tick).min(want) } else { (gs - 1.5 * tick).max(want) };
                            // A braked wheel turns a castoring nose only
                            // with the aircraft moving.
                            p.run(signed((bl, br)), (gs / 1.5).min(1.0), tick);
                            rolled += gs / 1.943_844 * tick;
                        }
                        other => break other,
                    }
                    t += tick;
                };
                let why = format!("course {course} k {k}");
                match end {
                    AlignStep::Done { err_deg } => {
                        assert!(err_deg.abs() <= super::ALIGNED_DEG, "{why}: {err_deg}")
                    }
                    other => panic!("{why}: {other:?}"),
                }
                assert!(t < super::ALIGN_MAX_S, "{why}: {t:.1} s");
                assert!(rolled < 25.0, "{why}: rolled {rolled:.1} m");
            }
        }
    }

    /// LILB 03 as apt.dat draws it: 601 m of 15 m pavement.
    fn lilb_03() -> crate::autoland::Runway {
        let (a, b) = ((45.7671573, 9.1605458), (45.7720916, 9.1636892));
        crate::autoland::Runway {
            icao: "LILB".into(),
            id: "03".into(),
            thr_lat: a.0,
            thr_lon: a.1,
            course_deg: crate::autoland::bearing_deg(a.0, a.1, b.0, b.1),
            length_m: crate::autoland::haversine_nm(a.0, a.1, b.0, b.1) * 1852.0,
            surface: 1,
            dist_nm: 0.0,
            elev_ft: 1258.0,
        }
    }

    /// The alignment as flown at LILB 03 (1790721973, 2026-09-30), from the
    /// turn-round's stop: 12.99 off, swinging -5.56 deg/s at 1.44 kt, at
    /// 45.767857 9.160965 (85 m down a 601 m strip, 16.1 m more than the
    /// 500 m fit). The Evolution's idle thrust runs a creep up at 0.2 kt/s
    /// whatever the governor asks, and a standing yaw of 0.025 brake left
    /// held the nose 2.0 to 2.2 off against the soft law (0.02 of brake
    /// right, still, for 20 s). On that plant the old law creeps its whole
    /// 40 m 2.2 off, as flown (40.7 m, 474.8 m left); now the trim takes the
    /// standing yaw out, the creep is stopped past 3 kt, and the roll stays
    /// inside the 16.1 m the fit can spare. And on the SR22's governor, which
    /// holds its creep, the same.
    #[test]
    fn a_line_up_spends_only_the_runway_the_fit_can_spare() {
        use super::yaw_plant::{signed, Plant};
        use super::{AlignStep, LineUpAlign};
        let rwy = lilb_03();
        let (lat, lon) = (45.767_857_048, 9.160_965_298);
        let spare = super::align_spare_m(&rwy, 500.0, lat, lon);
        assert!((spare - 16.1).abs() < 0.5, "spare {spare:.1}");
        // The Evolution's governor: idle thrust accelerates any creep; a
        // stop brakes at 1.5 kt/s. The SR22's holds the creep.
        let idle_push = |gs: f64, want: f64, tick: f64| {
            if want > 0.0 {
                gs + 0.2 * tick
            } else {
                (gs - 1.5 * tick).max(0.0)
            }
        };
        let holds = |gs: f64, want: f64, tick: f64| {
            if gs < want {
                (gs + 0.7 * tick).min(want)
            } else {
                (gs - 1.5 * tick).max(want)
            }
        };
        let course = rwy.course_deg;
        let pull = -0.025;
        // The old law on the same plant: the soft brake alone, a creep the
        // governor cannot hold, the flat 40 m bound.
        {
            let mut p = Plant::new(140.0, 3.0, 12.99, -5.56);
            let (mut gs, mut rolled, mut creeping) = (1.44_f64, 0.0_f64, false);
            while rolled < super::ALIGN_MAX_ROLL_M {
                let led = p.err + p.yaw * super::ALIGN_LEAD_S * -1.0;
                creeping = if creeping {
                    !(p.err.abs() <= 1.0 && led.abs() <= 2.0)
                } else {
                    p.err.abs() > 2.0 || led.abs() > 2.0
                };
                let cmd = signed(super::align_pivot_brakes(p.err, p.yaw)) + pull;
                gs = idle_push(gs, if creeping { 2.0 } else { 0.0 }, 0.3);
                p.run(cmd, (gs / 1.5).min(1.0), 0.3);
                rolled += gs / 1.943_844 * 0.3;
            }
            eprintln!("old law: {:.2} off at {gs:.1} kt after {rolled:.1} m", p.err);
            assert!(p.err.abs() > 1.9 && gs > 5.0, "as flown: {:.1} off at {gs:.1} kt", p.err);
        }
        for (name, gov) in [("Evolution", &idle_push as &dyn Fn(f64, f64, f64) -> f64), ("SR22", &holds)] {
            for (k, d) in [(140.0, 3.0), (70.0, 1.5)] {
                for tick in [0.3, 0.45] {
                    let mut judge = LineUpAlign::with_spare(course, Some(spare));
                    let mut p = Plant::new(k, d, 12.99, -5.56);
                    let (mut gs, mut rolled, mut t) = (1.44_f64, 0.0_f64, 0.0_f64);
                    let end = loop {
                        match judge.step(t, course - p.err, gs, p.yaw, rolled) {
                            AlignStep::Align { bl, br, creep } => {
                                gs = gov(gs, if creep { judge.creep_kts } else { 0.0 }, tick);
                                // The standing yaw only while rolling.
                                p.run(signed((bl, br)) + pull, (gs / 1.5).min(1.0), tick);
                                rolled += gs / 1.943_844 * tick;
                            }
                            other => break other,
                        }
                        t += tick;
                    };
                    // The caller stops it: the governor's brake from here.
                    while gs > 0.0 {
                        gs = gov(gs, 0.0, tick);
                        rolled += gs / 1.943_844 * tick;
                    }
                    let why = format!("{name} k {k} tick {tick}");
                    eprintln!("{why}: {end:?} rolled {rolled:.1} m in {t:.1} s, trim {:.3}", judge.trim);
                    match end {
                        AlignStep::Done { err_deg } => {
                            assert!(err_deg.abs() <= super::ALIGN_ACCEPT_DEG, "{why}: {err_deg}")
                        }
                        other => panic!("{why}: {other:?}"),
                    }
                    assert!(rolled <= spare, "{why}: rolled {rolled:.1} of {spare:.1} m");
                    assert!(t < super::ALIGN_MAX_S, "{why}: {t:.1} s");
                }
            }
        }
    }

    /// With the fit already marginal the alignment pivots nearly in place:
    /// the tight creep, and never past the runway it can spare, whatever it
    /// then says about the heading.
    #[test]
    fn a_marginal_fit_pivots_nearly_in_place() {
        use super::yaw_plant::{signed, Plant};
        use super::{AlignStep, LineUpAlign};
        assert_eq!(super::align_roll_limit_m(None), super::ALIGN_MAX_ROLL_M);
        assert_eq!(super::align_roll_limit_m(Some(-3.0)), 0.0);
        assert_eq!(super::align_roll_limit_m(Some(100.0)), super::ALIGN_MAX_ROLL_M);
        for spare in [3.0, 6.0, 10.0] {
            let mut judge = LineUpAlign::with_spare(24.0, Some(spare));
            assert_eq!(judge.creep_kts, super::ALIGN_TIGHT_CREEP_KTS);
            let mut p = Plant::new(140.0, 3.0, 11.8, -2.3);
            let (mut gs, mut rolled, mut t, tick) = (0.5_f64, 0.0_f64, 0.0_f64, 0.42);
            let end = loop {
                match judge.step(t, 24.0 - p.err, gs, p.yaw, rolled) {
                    AlignStep::Align { bl, br, creep } => {
                        gs = if creep { gs + 0.2 * tick } else { (gs - 1.5 * tick).max(0.0) };
                        p.run(signed((bl, br)) - 0.025, (gs / 1.5).min(1.0), tick);
                        rolled += gs / 1.943_844 * tick;
                    }
                    other => break other,
                }
                t += tick;
            };
            while gs > 0.0 {
                gs = (gs - 1.5 * tick).max(0.0);
                rolled += gs / 1.943_844 * tick;
            }
            assert!(rolled <= spare, "spare {spare}: rolled {rolled:.1} m, {end:?}");
        }
    }

    /// The turn-round at LILB 03 on a pivoting airframe. The recorded one
    /// (1790721973, 2026-09-30) stopped its leg one 24.9 m down and 14.9 m
    /// right of the centreline of a 15 m strip, heading 172, turned LEFT
    /// (the short way, outward) into the grass, and leg two ended 85 m down.
    /// Now: the pocket sits on the pavement nearer the numbers, the turn is
    /// committed toward the centreline from the first tick, and leg two
    /// settles in less than half the runway, the teardrop kept for every
    /// airframe that does not pivot.
    #[test]
    fn a_pivoting_turn_round_turns_across_the_strip_near_the_numbers() {
        use crate::autoland::{along_track_nm, cross_track_nm};
        let rwy = lilb_03();
        let pivot = super::TurnRound { pivots: true, width_m: Some(15.0) };
        let (start, pocket_m, off) = pivot.pocket();
        assert!(start < super::BACKTRACK_TURN_START_M && pocket_m < 30.0, "{start} {pocket_m}");
        assert!(off <= 15.0 / 2.0 - 2.0 + 1e-9 && off >= 4.0, "on the pavement: {off}");
        assert_eq!(super::TurnRound::TEARDROP.pocket(), (60.0, 30.0, 12.0));
        // Leg one from LILB's 21 hold (node 1) ends at that pocket.
        let path = super::backtrack_pocket_path(&rwy, 45.772005, 9.163885, pivot).expect("leg one");
        let end = *path.last().unwrap();
        let along = -along_track_nm(end.0, end.1, &rwy) * 1852.0;
        let ct = cross_track_nm(end.0, end.1, &rwy) * 1852.0;
        assert!((along - pocket_m).abs() < 0.5 && (ct - off).abs() < 0.5, "{along:.1} {ct:.1}");
        // Where the recorded leg one stopped: turned RIGHT, toward the
        // centreline, from the first tick, whatever the error's sign.
        let (plat, plon) = (45.767_307_48, 9.160_850_79);
        assert_eq!(super::turn_round_hint(&rwy, plat, plon), Some(1.0));
        let mut dir = super::turn_round_hint(&rwy, plat, plon);
        // The join logged -174.4 to its aim (the nose 148 off the course).
        assert!(super::committed_err(-174.4, &mut dir) > 170.0);
        assert!(super::committed_err(-120.0, &mut dir) > 0.0, "held through the turn");
        let mut dir = super::turn_round_hint(&rwy, plat, plon);
        let e0 = super::LineUpAlign::new(rwy.course_deg).err_deg(172.1);
        assert!(super::committed_err(e0, &mut dir) > 140.0, "{e0:.0}, right from the first tick");
        // Mirror side: left.
        let (llat, llon) = crate::autoland::project_latlon(plat, plon, rwy.course_deg - 90.0, 29.8 / 1852.0);
        assert_eq!(super::turn_round_hint(&rwy, llat, llon), Some(-1.0));
        // Leg two from the recorded stop: 85 m of runway as a teardrop,
        // 67 pivoting; from the new pocket, under 50.
        let stop_of = |p: Vec<(f64, f64)>| {
            let e = *p.last().unwrap();
            -along_track_nm(e.0, e.1, &rwy) * 1852.0
        };
        let old = stop_of(super::backtrack_leg_two_path(&rwy, plat, plon, super::TurnRound::TEARDROP).unwrap());
        let new = stop_of(super::backtrack_leg_two_path(&rwy, plat, plon, pivot).unwrap());
        assert!((old - 84.9).abs() < 1.0 && new < 68.0, "old {old:.1} new {new:.1}");
        let (pl, po) = (end.0, end.1);
        let from_pocket = stop_of(super::backtrack_leg_two_path(&rwy, pl, po, pivot).unwrap());
        assert!(from_pocket < 50.0, "{from_pocket:.1}");
        // The route is still a drivable one.
        let watch = TaxiWatch {
            path: super::backtrack_leg_two_path(&rwy, pl, po, pivot).unwrap(),
            via: String::new(),
            seg: Vec::new(),
            holding_point: String::new(),
            lead_on: Vec::new(),
        };
        assert!(Route::build(&watch).is_some());
    }

    #[test]
    fn a_line_up_already_straight_is_done_and_one_that_cannot_turn_says_so() {
        use super::{AlignStep, LineUpAlign};
        // A runway start's numbers: 0.4 off, stopped.
        let mut j = LineUpAlign::new(117.7);
        assert!(matches!(j.step(0.0, 117.3, 0.1, 0.0, 0.0), AlignStep::Done { .. }));
        assert_eq!(j.creeps, 0);
        // Brakes that turn nothing: the creep, then out of time, stopped.
        let mut j = LineUpAlign::new(117.7);
        let mut t = 0.0;
        let end = loop {
            match j.step(t, 107.0, 0.0, 0.0, 2.0) {
                AlignStep::Align { creep, br, .. } => assert!(creep && br > 0.0),
                other => break other,
            }
            t += 0.42;
        };
        assert!(matches!(end, AlignStep::GiveUp { why: "time", .. }), "{end:?}");
        // ...or out of runway.
        let mut j = LineUpAlign::new(117.7);
        assert!(matches!(
            j.step(0.0, 107.0, 2.0, 0.0, super::ALIGN_MAX_ROLL_M + 1.0),
            AlignStep::GiveUp { why: "rolled", .. }
        ));
        // Out of runway but inside what the roll straightens: done (LILB 03
        // Evolution 1790720549: 2.2 off, the mission gave up).
        let mut j = LineUpAlign::new(23.96);
        assert!(matches!(
            j.step(0.0, 21.76, 0.3, 0.2, super::ALIGN_MAX_ROLL_M + 1.0),
            AlignStep::Done { .. }
        ));
        assert_eq!(
            super::not_aligned_line(-10.7),
            "On the runway, but 11 degrees off its heading and I couldn't straighten us. \
             Holding position."
        );
    }

    #[test]
    fn a_pivot_lets_go_of_the_brake_as_the_turn_rate_builds() {
        assert_eq!(pivot_rate_scale(-0.7, -10.0), 1.0, "under the ceiling");
        assert!((pivot_rate_scale(-0.7, -20.0) - 0.5).abs() < 1e-9);
        assert_eq!(pivot_rate_scale(-0.7, -35.0), 0.0, "the LILB join spin");
        assert_eq!(pivot_rate_scale(-0.7, 30.0), 1.0, "turning the wrong way keeps it all");
        assert!((smooth_toward(0.0, 1.0, 0.8, 0.2) - 0.2).abs() < 1e-9);
    }

    #[test]
    fn the_trim_holds_through_joins_and_bleeds_through_bends() {
        let mut t = StraightTrim::default();
        for _ in 0..100 {
            t.step(-4.0, 0.0, 12.0, false, 500.0, 0.2);
        }
        let built = t.step(-4.0, 0.0, 12.0, false, 500.0, 0.2);
        assert!(built > 0.1 && built <= TRIM_MAX, "{built}");
        assert_eq!(t.step(-4.0, 0.0, 12.0, true, 500.0, 0.2), built, "a join is not a bias");
        assert_eq!(t.step(-20.0, 0.0, 12.0, false, 500.0, 0.2), built, "an excursion is not a bias");
        assert_eq!(t.step(-4.0, 0.0, 2.0, false, 500.0, 0.2), built, "a crawl is not a bias");
        let mut bled = built;
        for _ in 0..30 {
            bled = t.step(-4.0, 3.0, 8.0, false, 500.0, 0.2);
        }
        assert!(bled < 0.2 * built, "a bend bleeds it: {bled}");
    }

    #[test]
    fn a_reversal_is_turned_one_way() {
        // LOWI 26's backtrack turn: +177, then -179 on noise.
        let mut dir = None;
        assert_eq!(super::committed_err(177.0, &mut dir), 177.0);
        assert_eq!(super::committed_err(-179.0, &mut dir), 179.0, "kept turning right");
        assert_eq!(super::committed_err(-120.0, &mut dir), 120.0, "still the same way");
        assert_eq!(super::committed_err(-40.0, &mut dir), -40.0, "released once nearly round");
        assert_eq!(dir, None);
    }

    #[test]
    fn a_pivot_gets_brake_at_walking_pace() {
        // Full lock at 4.7 kt: the taxi split gave 0.06, a pivot 0.45.
        let (l, _) = super::brake_steer_split(-1.0, 4.7);
        let (pl, pr) = super::pivot_brake_split(-1.0, 4.7);
        assert!(l < 0.1, "{l}");
        assert!((pl - 0.45).abs() < 1e-9 && pr == 0.0, "{pl}");
        assert_eq!(super::pivot_brake_split(-1.0, 0.5), (0.0, 0.0), "not while stopped");
    }

    #[test]
    fn a_big_error_at_walking_pace_pivots_off_a_join_too() {
        // LSZS 03 lead-on: err 63 at 3.9 kt, steer 0.99, no brake.
        assert!(super::pivot_due(63.0, 3.9, false));
        assert!(super::brake_steer_split(0.99, 3.9) == (0.0, 0.0), "the starved split");
        assert!(!super::pivot_due(63.0, 9.0, false), "a taxi corner at pace keeps the taxi split");
        assert!(!super::pivot_due(30.0, 3.0, false));
        assert!(super::pivot_due(-70.0, 9.0, true), "a join as before");
    }

    /// The SR22's brake-steer yaw plant, fitted over both LEMG taxis (sorties
    /// 1790605551 and 1790618416, 2026-09-28; output error over 5 s windows,
    /// 1.7 deg/s rms): yaw follows the differential brake (right minus left)
    /// through a 0.45 s transport delay and a 2.17 s lag, at
    /// 299 - 35.8 v deg/s per unit of brake (v in m/s: a braked wheel swings
    /// a slow aircraft harder), with a 1.7 deg/s pull to the left. The
    /// rudder half of the steer adds nothing measurable at taxi speed.
    struct Sr22Plant {
        r: f64,
        /// (time, brake difference, groundspeed m/s) commanded.
        sent: std::collections::VecDeque<(f64, f64, f64)>,
    }

    impl Sr22Plant {
        const DELAY_S: f64 = 0.45;
        const TAU_S: f64 = 2.17;
        const PULL_DPS: f64 = -1.7;
        fn gain(v_ms: f64) -> f64 {
            (299.0 - 35.8 * v_ms).max(10.0)
        }
        fn new(r: f64) -> Self {
            Sr22Plant { r, sent: std::collections::VecDeque::new() }
        }
        /// Command at `t`, then advance `dt`: the yaw rate, deg/s.
        fn step(&mut self, t: f64, brake_diff: f64, v_ms: f64, dt: f64) -> f64 {
            self.sent.push_back((t, brake_diff, v_ms));
            // Sample and hold: the newest command at least DELAY_S old.
            while self.sent.len() > 1 && self.sent[1].0 <= t - Self::DELAY_S + 1e-9 {
                self.sent.pop_front();
            }
            let (t0, b, v) = self.sent[0];
            let (b, v) = if t0 <= t - Self::DELAY_S + 1e-9 { (b, v) } else { (0.0, v_ms) };
            let pull = if v_ms > 0.5 { Self::PULL_DPS } else { 0.0 };
            let want = b * Self::gain(v) + pull;
            self.r += (want - self.r) * (dt / Self::TAU_S).min(1.0);
            self.r
        }
    }

    /// The recorded corner (sortie 1790605551, t +434.1 to +448.0 s from the
    /// drive's engage): (s, gs kt, right minus left brake, logged yaw_dps).
    const LEMG_CORNER: &[(f64, f64, f64, f64)] = &[
        (434.1, 6.64, 0.042, -4.38), (434.3, 6.54, 0.044, -3.74), (434.5, 6.43, 0.045, -3.00), (434.7, 6.33, 0.044, -2.15),
        (434.9, 6.20, 0.044, -1.50), (435.1, 6.09, 0.045, -0.83), (435.3, 6.01, 0.044, -0.17), (435.5, 5.85, 0.047, 0.45),
        (435.7, 5.74, 0.051, 1.04), (435.9, 5.62, 0.056, 1.76), (436.1, 5.50, 0.062, 2.06), (436.3, 5.36, 0.068, 2.73),
        (436.5, 5.20, 0.075, 3.41), (436.6, 5.03, 0.082, 4.16), (436.8, 4.85, 0.092, 5.34), (437.0, 4.66, 0.103, 6.06),
        (437.2, 4.38, 0.111, 6.94), (437.4, 4.15, 0.119, 8.10), (437.6, 3.90, 0.120, 9.32), (437.9, 3.61, 0.106, 10.67),
        (438.1, 3.41, 0.088, 12.02), (438.3, 3.26, 0.064, 13.18), (438.5, 3.15, 0.039, 13.94), (438.7, 3.12, 0.021, 14.36),
        (438.9, 3.13, 0.007, 14.34), (439.1, 3.19, 0.000, 13.99), (439.3, 3.26, 0.000, 13.38), (439.5, 3.35, 0.000, 12.67),
        (439.7, 3.47, 0.000, 11.83), (439.9, 3.58, 0.000, 10.90), (440.1, 3.68, 0.000, 9.95), (440.4, 3.87, 0.000, 8.99),
        (440.6, 4.00, 0.000, 7.60), (440.8, 4.15, 0.000, 6.90), (441.0, 4.33, 0.000, 6.45), (441.2, 4.50, 0.000, 5.09),
        (441.4, 4.65, 0.017, 4.29), (441.6, 4.76, 0.067, 3.47), (441.8, 4.88, 0.099, 2.96), (442.0, 4.87, 0.130, 3.06),
        (442.2, 4.78, 0.136, 3.80), (442.4, 4.69, 0.128, 5.04), (442.6, 4.62, 0.111, 6.30), (442.9, 4.59, 0.080, 7.56),
        (443.1, 4.61, 0.053, 8.55), (443.3, 4.69, 0.027, 9.02), (443.5, 4.87, 0.000, 9.09), (443.7, 5.07, 0.000, 8.57),
        (443.9, 5.28, 0.000, 7.73), (444.1, 5.49, 0.000, 6.69), (444.3, 5.71, 0.000, 5.70), (444.5, 5.93, 0.000, 4.70),
        (444.7, 6.19, 0.000, 3.69), (444.9, 6.40, 0.013, 2.64), (445.1, 6.57, 0.031, 1.78), (445.4, 6.75, 0.052, 1.20),
        (445.6, 6.85, 0.071, 0.87), (445.7, 6.90, 0.097, 0.88), (446.0, 6.89, 0.122, 1.35), (446.2, 6.86, 0.125, 2.48),
        (446.4, 6.73, 0.132, 3.38), (446.6, 6.65, 0.136, 4.03), (446.8, 6.53, 0.135, 5.30), (446.9, 6.41, 0.117, 7.35),
        (447.1, 6.34, 0.084, 8.14), (447.3, 6.30, 0.056, 8.07), (447.5, 6.33, 0.017, 9.29), (447.8, 6.44, 0.000, 8.34),
        (448.0, 6.57, 0.000, 7.78),
    ];

    /// The recorded brakes through the plant, with the loop's own yaw filter
    /// (an even EMA of the tick's rate), come back as the recorded yaw: the
    /// plant the corner test flies is the aircraft. A plant without the lag
    /// swings nearly twice as hard.
    #[test]
    fn the_sr22_brake_plant_replays_the_recorded_corner() {
        let replay = |tau: f64| -> (f64, f64) {
            let mut p = Sr22Plant::new(LEMG_CORNER[0].3);
            let (mut ema, mut se, mut peak) = (LEMG_CORNER[0].3, 0.0_f64, 0.0_f64);
            for w in LEMG_CORNER.windows(2) {
                let ((t, gs, b, _), (t1, _, _, logged)) = (w[0], w[1]);
                let dt = t1 - t;
                p.sent.push_back((t, b, gs * 0.514_444));
                while p.sent.len() > 1 && p.sent[1].0 <= t - Sr22Plant::DELAY_S + 1e-9 {
                    p.sent.pop_front();
                }
                let (_, bd, v) = p.sent[0];
                let want = bd * Sr22Plant::gain(v) + Sr22Plant::PULL_DPS;
                p.r += (want - p.r) * (dt / tau).min(1.0);
                ema = 0.5 * ema + 0.5 * p.r;
                se += (ema - logged).powi(2);
                peak = peak.max(ema);
            }
            ((se / (LEMG_CORNER.len() - 1) as f64).sqrt(), peak)
        };
        let (rms, peak) = replay(Sr22Plant::TAU_S);
        assert!(rms < 3.0, "the plant tracks the recorded yaw: {rms:.2} deg/s rms");
        assert!((peak - 14.36).abs() < 2.0, "and its peak: {peak:.1} vs 14.4 deg/s");
        let (fast, fast_peak) = replay(0.3);
        assert!(fast > 2.5 * rms && fast_peak > 20.0, "no lag is not this aircraft: {fast:.1} rms, {fast_peak:.1} peak");
    }

    /// One drive tick of the law, as `drive` runs it, through the SR22 plant.
    /// Returns the scorer's own verdict on the line: (max cut inside a
    /// vertex, max off the line), from [`crate::scorecard::LineJudge`].
    fn fly_sr22_corner(path_m: &[(f64, f64)], start: (f64, f64), aim_floor_free: bool) -> (f64, f64) {
        use crate::scorecard::{LineJudge, CORNER_TURN_DPS};
        let pace = Pace::from_profile(crate::aircraft::load("cirrus_sr22").expect("sr22").taxi.as_ref());
        assert!(pace.brake_steer);
        let watch = TaxiWatch {
            path: path_m.iter().map(|&(x, y)| ll(x, y)).collect(),
            via: String::new(),
            seg: vec![(String::new(), false); path_m.len() - 1],
            holding_point: String::new(),
            lead_on: Vec::new(),
        };
        let route = Route::build(&watch).expect("route");
        let dt = 0.2;
        let (mut x, mut y) = start;
        let mut psi = taxi_guide::bearing(path_m[0], path_m[1]);
        let mut v = 10.0 / MS_TO_KTS;
        let mut plant = Sr22Plant::new(0.0);
        let (mut yaw, mut prev_psi, mut prev_steer, mut bs, mut bs_prev) = (0.0_f64, psi, 0.0_f64, 0.0_f64, (0.0_f64, 0.0_f64));
        let mut trim = StraightTrim::default();
        let mut turn_dir = None;
        let mut judge = LineJudge::default();
        let end_m = route.total_m() - 100.0;
        for i in 0..3000 {
            let t = i as f64 * dt;
            let gs = v * MS_TO_KTS;
            let (lat, lon) = ll(x, y);
            let (progress, offset) = route.fix_signed(lat, lon).expect("a fix");
            if progress > end_m {
                break;
            }
            let remaining = route.remaining(progress);
            // The floor under test; the brakes are the free nose's either way.
            let look = lookahead_m_with(gs, pace.lookahead_s)
                .min(bend_lookahead_cap_floor_m(
                    gs,
                    route.route_turn_rate_dps(progress, BEND_PROBE_M, gs),
                    corner_aim_floor_m(aim_floor_free),
                ))
                .min((remaining * 0.6).max(6.0));
            let err = committed_err(route.heading_err_deg(lat, lon, psi, progress, look), &mut turn_dir);
            let route_turn = route.route_turn_rate_dps(progress, look, gs);
            let raw = taxi_guide::angle_diff(psi, prev_psi) / dt;
            prev_psi = psi;
            yaw = yaw * 0.5 + raw.clamp(-90.0, 90.0) * 0.5;
            let tr = trim.step(offset, route_turn, gs, false, remaining, dt);
            let want = stop_straighten(
                (steer_cmd_with(damped_err_deg(err, yaw, route_turn, pace.steer_lead_s), pace.steer_full_deg) + tr)
                    .clamp(-1.0, 1.0)
                    * steer_authority(gs),
                remaining,
                gs,
            );
            let steer = slew_steer(prev_steer, want, pace.steer_slew_per_s, dt);
            prev_steer = steer;
            bs = smooth_toward(bs, steer, BRAKE_STEER_TAU_S, dt);
            let pivot = pivot_due(err, gs, false);
            let (wl, wr) = if pivot { pivot_brake_split(steer, gs) } else { brake_steer_split_band(bs, gs, true) };
            let k = pivot_rate_scale(if pivot { steer } else { bs }, yaw);
            let bl = slew_steer(bs_prev.0, wl * k, Some(BRAKE_STEER_SLEW_PER_S), dt);
            let br = slew_steer(bs_prev.1, wr * k, Some(BRAKE_STEER_SLEW_PER_S), dt);
            bs_prev = (bl, br);
            // What the blackbox would log, judged as the scorer judges it.
            let asked = (route_turn.abs() >= CORNER_TURN_DPS).then(|| route_turn.signum());
            let cutting = judge.corner(t, gs, offset, Some(remaining), asked, yaw);
            judge.judge(offset, cutting);
            // The governor holds the schedule at the SR22's measured pace.
            let sched = schedule_kts(pace.straight, &route, progress, gs).min(pace.track_cap_kts(offset.abs(), err));
            v = (v + (sched / MS_TO_KTS - v).clamp(-0.7 * dt, 0.5 * dt)).max(0.0);
            let r = plant.step(t, br - bl, v, dt);
            psi = (psi + r * dt).rem_euclid(360.0);
            x += v * psi.to_radians().sin() * dt;
            y += v * psi.to_radians().cos() * dt;
        }
        judge.flush();
        (judge.max_cut, judge.max_lat)
    }

    /// LEMG, the SR22 from the ramp (sorties 1790605551 and 1790618416,
    /// 2026-09-28): 4.2 to 5.3 m inside the corners, 5.3 m at the worst,
    /// over the scorer's 5 m. The ramp corner at +436 s: 311 degrees, round
    /// through about 90 at one vertex and 20 more at the next, 6 m on. The
    /// 16 m aim reaches round the vertex, the nose comes round early at 14
    /// deg/s, the rate lead takes it off and the aircraft rolls the chord
    /// across the inside. Flown through the fitted plant: the free nose's
    /// shorter corner aim keeps it inside the lateral limit, and the swing
    /// out of the corner stays inside it too.
    #[test]
    fn a_castoring_nose_turns_the_lemg_ramp_corner_on_the_line() {
        let th1 = 90.0_f64.to_radians();
        let th2 = 110.0_f64.to_radians();
        let b = (6.0 * th1.sin(), 6.0 * th1.cos());
        let path = [(0.0, -250.0), (0.0, 0.0), b, (b.0 + 250.0 * th2.sin(), b.1 + 250.0 * th2.cos())];
        let (old_cut, old_lat) = fly_sr22_corner(&path, (0.0, -200.0), false);
        let (cut, lat) = fly_sr22_corner(&path, (0.0, -200.0), true);
        eprintln!("16 m floor: cut {old_cut:.2} lat {old_lat:.2}; free nose: cut {cut:.2} lat {lat:.2}");
        // The scorer's worst: a cut the projection jumped across, or the same
        // chord judged as the line when the nose stopped turning first.
        let (old, new) = (old_cut.max(old_lat), cut.max(lat));
        assert!(old > 4.5, "the plant must reproduce the recorded 5.3 m: {old:.2} m");
        assert!(new <= crate::scorecard::TAXI_MAX_LAT_M, "inside the line's own limit: {new:.2} m");
        assert!(new < old - 1.5, "{new:.2} vs {old:.2} m");
        // A plain right angle the same way.
        let path = [(0.0, -250.0), (0.0, 0.0), (250.0, 0.0)];
        let (old_cut, old_lat) = fly_sr22_corner(&path, (0.0, -200.0), false);
        let (cut, lat) = fly_sr22_corner(&path, (0.0, -200.0), true);
        eprintln!("90 degrees: 16 m floor cut {old_cut:.2} lat {old_lat:.2}; free nose cut {cut:.2} lat {lat:.2}");
        let (old, new) = (old_cut.max(old_lat), cut.max(lat));
        assert!(new < old && new <= crate::scorecard::TAXI_MAX_LAT_M, "{new:.2} vs {old:.2}");
    }

    /// The aim is only shorter where a free nose turns a corner: a straight,
    /// a gentle bend and every other nose fly exactly the old aim.
    #[test]
    fn only_a_free_nose_in_a_corner_gets_the_shorter_aim() {
        let pace = Pace::from_profile(crate::aircraft::load("cirrus_sr22").expect("sr22").taxi.as_ref());
        // Straight: no bend cap at all.
        assert_eq!(steer_look_m(&pace, 10.0, 0.0, 1000.0, true), steer_look_m(&pace, 10.0, 0.0, 1000.0, false));
        // The LEMG vertex at 5 kt (the recorded 14 deg/s of route turn).
        assert_eq!(steer_look_m(&pace, 5.0, 14.0, 1000.0, false), 16.0);
        assert_eq!(steer_look_m(&pace, 5.0, 14.0, 1000.0, true), FREE_NOSE_AIM_FLOOR_M);
        // A gentle bend's own cap is longer than either floor: unchanged.
        assert_eq!(bend_lookahead_cap_floor_m(12.0, -3.3, 12.0), bend_lookahead_cap_m(12.0, -3.3));
        assert_eq!(corner_aim_floor_m(false), BEND_AIM_FLOOR_M);
    }

    /// LEMG (2026-09-28): full auto gave both SR22 taxis up as
    /// "taxi_stalled" at exactly 600 s, still rolling. The drives' remaining
    /// metres every minute (sorties 1790605551, engaged at 4,605 m of a
    /// 4,661 m route, and 1790618416, 4,736 m of 4,791): never stuck by the
    /// ground they made, and each well inside its own deadline.
    #[test]
    fn a_long_taxi_is_judged_by_the_ground_it_makes() {
        const S1: &[(f64, f64)] = &[
            (0.0, 4605.4), (60.1, 4471.8), (120.1, 4201.5), (180.1, 3922.8), (240.1, 3638.2), (300.1, 3375.7),
            (360.1, 3065.3), (420.1, 2738.3), (480.2, 2537.5), (540.3, 2274.0), (599.9, 1997.7),
        ];
        const S2: &[(f64, f64)] = &[
            (0.0, 4735.9), (60.2, 4600.9), (120.2, 4328.2), (180.2, 4058.3), (240.2, 3767.1), (300.2, 3525.7),
            (360.3, 3231.3), (420.4, 2926.2), (480.4, 2624.2), (540.6, 2404.2), (600.0, 2103.9),
        ];
        for (ticks, total) in [(S1, 4660.6), (S2, 4791.1)] {
            let deadline = route_deadline_s(total, 10.0);
            let mut w = ProgressWatch::start(0.0, deadline);
            let r0 = ticks[0].1;
            for &(t, rem) in ticks {
                w.note(r0 - rem, t);
                assert!(!w.stuck(t), "t {t}: {:.0} s without {PROGRESS_STEP_M} m", w.stalled_s(t));
            }
            // At the pace it was making, the rest of the route fits.
            let (t, rem) = ticks[ticks.len() - 1];
            let finish = t + rem / ((r0 - rem) / t);
            assert!(finish < deadline, "finish {finish:.0} s against {deadline:.0} s");
            assert!(finish > 600.0, "and the flat 600 s never could: {finish:.0} s");
        }
        assert!((route_deadline_s(4660.6, 10.0) - 1932.0).abs() < 1.0);
        // A drive that stops making ground IS stuck, whatever it creeps.
        let mut w = ProgressWatch::start(0.0, 5000.0);
        w.note(2607.7, 599.9);
        for k in 0..200 {
            w.note(2607.7 + 0.05 * k as f64, 600.0 + k as f64);
        }
        assert!(w.stuck(799.0), "{:.0} s", w.stalled_s(799.0));
        // A pause, a tug or a spoken stop restarts the clock.
        w.hold(799.0);
        assert!(!w.stuck(799.0));
        // A join that turns away from the route's start is not progress,
        // and not a stall until it has had the whole window.
        let mut j = ProgressWatch::start(0.0, 5000.0);
        j.note(40.0, 0.0);
        j.note(30.0, 100.0);
        assert!(!j.stuck(100.0) && (j.stalled_s(100.0) - 100.0).abs() < 1e-9);
        j.note(61.0, 110.0);
        assert_eq!(j.stalled_s(110.0), 0.0);
        // The route's deadline, moved on by the time held.
        let mut d = ProgressWatch::start(0.0, 100.0);
        for k in 0..=100 {
            d.note(25.0 * k as f64, k as f64);
        }
        assert!(!d.stuck(100.0));
        d.hold(150.0);
        assert!(!d.stuck(149.0) && !d.stuck(150.0), "50 s held, 50 s more");
        assert!(d.stuck(151.0));
    }
}
