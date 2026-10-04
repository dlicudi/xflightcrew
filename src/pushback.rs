//! Pushback off the stand — the first metres of the gate-to-gate chain.
//!
//! A nose-in stand's way out runs BEHIND the aircraft: the cleared taxi
//! route's opening leg points back past the wingtip, and driving it forward
//! is driving into the terminal. This stage reads that geometry off the
//! ROUTE itself — no stand database, no per-airframe flag: if the way out is
//! behind the wing line we are nose-in, and that is what a pushback is for.
//! A tie-down you can simply taxi out of returns [`plan`] `None` and never
//! calls a tug.
//!
//! The push is steered, not just straight: the tug stops with the aircraft
//! ON the taxi lane and POINTING DOWN IT, so an airliner never has to turn
//! itself around on the apron (the auto-taxi join could, but a 180 on a
//! stand is not a thing real crews do). Which of X-Plane's `pushback_left` /
//! `pushback_right` swings the nose which way under tow is LEARNED in the
//! loop rather than assumed — the first sustained turn that makes the error
//! worse flips the sense once, so a wrong guess costs a few degrees, not the
//! whole push. That lesson is only worth anything if it is judged against a
//! MOVING aircraft: see [`sense_is_wrong`].
//!
//! Structurally this is an inline stage of [`crate::full_auto`], not a
//! spawned sub-loop like the taxi governor: nothing else can arm a pushback,
//! so there is no second owner to arbitrate with. The narration is the
//! MONITOR's — its pushback watch already calls the tug connecting, the push
//! starting, and the tug clearing off the transition datarefs (so this
//! module speaks only what that watch cannot know: the ask, and the
//! failures). Laws are pure and battle-test as plain `cargo test`; events
//! blackbox as `"kind":"pushback"`.

use std::sync::atomic::{AtomicBool, AtomicI8, Ordering::SeqCst};
use std::time::{Duration, Instant};

use serde_json::json;

use crate::autotaxi::PSI_TRUE;
use crate::nav;
use crate::tools::{Tool, ToolCtx};
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

const LAT: &str = "sim/flightmodel/position/latitude";
const LON: &str = "sim/flightmodel/position/longitude";
/// The tug on the bar. This is the ONE dataref a pushback actually moves:
/// probed live at a stand while the aircraft was rolling backwards under a
/// tug, `pushback_attached` was 1 and `tow_in_progress` stayed 0 the whole
/// time (2026-07-27). Whatever "tow operation" the latter reports, it is not
/// this one, so nothing here may be gated on it — treating it as the
/// under-tow signal meant the push never believed it was moving.
const PUSH_ATTACHED: &str = "sim/aircraft/overflow/pushback_attached";
/// Read alongside, never relied on: if some airframe or a future build does
/// raise it, it only ever ADDS evidence (and the per-tick events carry both,
/// so the next sortie can settle it again).
const TOW_IN_PROGRESS: &str = "sim/flightmodel2/misc/tow_in_progress";
const M_PER_NM: f64 = 1852.0;

/// Route points closer than this to the aircraft are the stand itself, not
/// the way out — the planner's polyline starts AT the nose.
const MIN_NODE_M: f64 = 10.0;
/// ...and a "way out" farther than this is not OUR way out at all: the route
/// was planned from somewhere the aircraft no longer is. A stale clearance
/// (planned from runway 22C, aircraft since repositioned to a gate 1 km
/// away) read as "nose in" and called a tug to push toward a lane a
/// kilometre off (EGSS, 2026-07-29). The stand's own lane is tens of
/// metres away, never hundreds.
const MAX_NODE_M: f64 = 250.0;
/// The way out must be at least this far off the nose before we call a tug.
/// Past the wing line, not merely off to one side: a sixty-degree turn out of
/// a tie-down is a taxi, and asking for a pushback to make it would be the
/// copilot inventing ground handling nobody ordered.
const BEHIND_DEG: f64 = 100.0;
/// Bounds on how far back to push. The floor keeps a rounding-error route
/// from calling a tug for nothing; the ceiling is the point past which we are
/// no longer pushing off a stand but taxiing backwards down an apron.
const MIN_BACK_M: f64 = 12.0;
const MAX_BACK_M: f64 = 120.0;
/// Heading band inside which the tug pushes STRAIGHT back. A swing pattern
/// is a whole manoeuvre of about 90 degrees whatever was wanted, so asked for
/// 45 it overshoots by as much as it was meant to fix, and it swings the
/// tail sideways across whatever is beside the stand: LIBP (2026-09-26)
/// asked "tail left" for a 46-degree change and swung a 737 toward the
/// buildings. Under 60 degrees the push goes straight back and the taxi
/// turns the rest under its own steering, which it does from any angle.
const DEAD_DEG: f64 = 60.0;
/// Close enough to "pointing down the lane" to let the tug go.
const ALIGN_DEG: f64 = 20.0;
/// What ONE pattern delivers. X-Plane's commands are whole manoeuvres named
/// "Nose 90 left" / "Nose 90 right", and every measured trail agrees: a clean
/// ~90-degree swing, then the tug stops and lets go. This is a fact about the
/// SIM, and it belongs in the arithmetic — a plan wanting 135 degrees cannot
/// be satisfied by one pattern no matter how patiently we wait for it.
const PATTERN_DEG: f64 = 90.0;
/// Meaningfully short of the planned distance — enough that another straight
/// segment is worth an ask. Under this the remaining metres are the taxi
/// drive's to cover in its first seconds.
const SHORT_PUSH_M: f64 = 10.0;
/// Extra distance the turn may use past the planned stop before we stop
/// anyway — the push ENDS, aligned or not, rather than reversing across the
/// apron in search of the last twenty degrees.
const CHASE_M: f64 = 25.0;
/// The error must grow by more than this under a sustained turn before we
/// believe we are turning the wrong way. The target heading is fixed for the
/// whole push, so this is really a test that the NOSE swung this far the wrong
/// way — which is what a reversed command looks like (the 2026-07-27 Luton push
/// swung 90 degrees). At 3 degrees it convicted on noise instead: a tug pushing
/// dead straight back drifted 86 to 89 degrees over 39 metres, the sense was
/// called wrong, and the correction that followed ended the push (2026-07-28).
const SENSE_TOL_DEG: f64 = 15.0;
/// How long a turn command runs before its effect is judged.
const CAL_SECS: f64 = 4.0;

const TICK_MS: u64 = 500;
/// How long to wait for the tug to drive up and take hold. Minutes, not
/// seconds: it is a vehicle crossing an airport, not a dataref flipping (see
/// the wait loop in [`run`]) — X-Plane searches 1500 m for one and lets it
/// drive up to 6 km, and there is no dataref that says "it's on its way", so
/// this deadline is a blind bet against an invisible vehicle.
///
/// It has now lost that bet TWICE, in the same direction: 30 s gave up on the
/// first live push with the tug still driving (Gatwick, 2026-07-27), was
/// raised to 180 — and 180 then gave up at Luton with the tug visibly almost
/// at the nose (2026-07-28; the previous push at the SAME field had hooked at
/// 125 s, so arrivals genuinely spread past that ceiling). The cost of the
/// two outcomes is not symmetric: waiting longer at a tug-less field costs a
/// few quiet minutes before the same fallback, while giving up early costs
/// the whole push AND the apron turn-around that follows, with the pilot
/// watching the tug arrive at an aircraft that just drove off. So: six
/// minutes, and the crew narrates the wait so it never reads as a hang.
const TUG_WAIT_SECS: f64 = 360.0;
/// Past this the wait stops looking normal, so the crew says what we're
/// waiting for — an aircraft sitting silently on a stand reads as a hang.
const TUG_PATIENCE_SECS: f64 = 30.0;
/// A second crew line deep into the wait, so the long ceiling stays narrated
/// rather than silent ("still coming" beats three quiet minutes).
const TUG_PATIENCE2_SECS: f64 = 150.0;
/// The tug hooked on, let go WITHOUT pushing, and has been gone this long:
/// summon it again, once. Never sent to a tug on the bar — a command to an
/// attached tug drops it off the bar (five pushes for five, 2026-09-12).
const NUDGE_SECS: f64 = 30.0;
/// Rolled back this far and the push is genuinely under way — the "are we
/// moving" test is our OWN position, not a dataref, because the sim raises
/// none while a tug pushes (see [`PUSH_ATTACHED`]). Comfortably past GPS
/// jitter at a parked stand.
const MOVED_M: f64 = 1.5;
/// Motionless this long once the push has started: the tug has finished the
/// manoeuvre it was given (they run one pattern and stop), so this ends the
/// push rather than waiting out `MAX_SECS`. Generous, because the tug's own
/// acceleration is slow and a real push has slack in it.
const STALL_SECS: f64 = 12.0;
/// Whole-push timeout: something is wrong (a tug pushing us nowhere, a stuck
/// dataref) and the sequence must not sit at the gate forever.
const MAX_SECS: f64 = 300.0;
/// How long to wait for the tug to unhook and clear before taxiing, counted
/// from the last moment the tug MOVED the aircraft, not from our "stop": a
/// tug mid-swing can ignore the stop and finish its pattern. At 13:39Z
/// 2026-09-26 the push loop stopped at 60.9 m with the nose at 204, the tug
/// swung it on to 262 over the next 31 s, stood still, and let go 3 s
/// later; a wait counted from the stop had timed out 4 s before that and
/// said "the tug's still showing connected".
const TUG_CLEAR_SECS: f64 = 30.0;
/// The whole tug wait, however long the tug keeps moving us.
const TUG_CLEAR_MAX_SECS: f64 = 120.0;
/// The nose turning by this much over `SWING_WINDOW_S` is the tug SWINGING
/// us: a manoeuvre in progress, which no distance test may cut short. A
/// Nose 90 at LXGB stand 1 ran 45 m straight before it began to turn, so
/// the plan's 40 m plus its chase margin was spent while the nose was still
/// 140 degrees off; `done` fired on the distance, "stop" went to a tug that
/// finishes what it was given regardless, and the taxi then "drove" for
/// 34 s at half a knot while the tug turned it, until the stagnation clock
/// stood the whole mission down (08:15Z, 2026-09-15).
const SWING_DEG: f64 = 2.0;
const SWING_WINDOW_S: f64 = 2.0;
/// A swing, once seen, is over only when the nose has turned less than
/// `SWING_STILL_DEG` over `SWING_SETTLE_S`: STILL, not merely slower. The
/// rate test alone stood in for "the swing is over" and ended a live one.
/// At 13:38:44Z 2026-09-26 (LIBP, sortie 1790429680, a Nose 90 left) the
/// tug was swinging the nose at 1.2 deg/s; its window, trimmed to samples
/// NEWER than two seconds, spanned three 0.55 s ticks, so it asked for two
/// degrees in 1.7 s, read 14.6 to 12.6 as under it, and `done` sent "stop"
/// at 60.9 m with the nose at 204. The tug took no notice: over the next
/// 31 s it swung us on to 262, dipping to 0.7 deg/s on the way (never less
/// than 3.2 degrees in any four seconds), and stood still 4 s before it
/// let go. A tug at the end of its pattern turns under a tenth of a degree
/// in four seconds.
const SWING_SETTLE_S: f64 = 4.0;
const SWING_STILL_DEG: f64 = 1.0;
/// A swing that BUILDS slowly is still a swing: `SWING_DEG` over the
/// longer `SWING_SETTLE_S` starts one too (0.5 deg/s). The two-second test
/// alone asks for 1 deg/s, and a tug easing into its turn stays under that
/// for seconds while the nose is already going: at EGBB (sortie
/// 1790534902, 2026-09-27, a Nose 90 on the 737) the rate climbed 0.1 to
/// 0.8 deg/s over 13 s and the watch still read `swinging` false at 41.2 m,
/// when `pavement_edge` stopped the push with the nose 82 degrees off; the
/// tug swung on to 235 regardless (`stop_ignored`, 85 degrees). Two degrees
/// in four seconds is past the ending test's one, so the hysteresis stays.
/// From the push tick that reads the heading to the tug's nose standing
/// still, once "stop" is sent mid-swing: the stop is LED by the swing rate
/// times this. Measured on the one obeyed mid-swing stop in the log: LIPK
/// (sortie 1790408535, 2026-09-26) sent it at err -11.8 with the nose
/// turning 3.4 deg/s; the `state` heading moved 2.8 degrees more and was
/// still 1.3 s later (0.8 s at the entry rate). The LIBP tug ignored its stop
/// outright, so this is one sample, not a law.
const STOP_LATENCY_S: f64 = 0.8;
/// A swing closing on the lane is stopped when the heading it will come to
/// rest on (see `STOP_LATENCY_S`) is this close to the lane, or past it.
const ON_LANE_DEG: f64 = 2.0;
/// The heading still moving this far after "stop" went out: the tug ignored
/// it (LIBP 2026-09-26: 58 degrees). The obeyed LIPK stop moved 2.8.
const STOP_IGNORED_DEG: f64 = 5.0;
// There is deliberately no re-hook deadline. A released tug is never asked
// back: `reissued_after_release` fired four times across the whole blackbox
// and `rehooked` never once answered it, so the wait was a pure four-for-four
// tax on the give-up. See the release branch in `run_inner`.
/// Rolled FORWARD this far from the stand and no tug is pushing us — we are
/// creeping under our own idle thrust with the brakes off. Ends the push,
/// brakes on. Past `MOVED_M` so the same jitter both tests tolerate cannot
/// trip it, and small enough to catch the creep while it is still slow.
///
/// A BACKSTOP now, not the first line: the hold below arrests the creep long
/// before this fires, so reaching it means the brake channel isn't working
/// (an old plugin) — only then is giving up right.
const CREEP_M: f64 = 2.5;
/// The proportional brake channel (1 s expiry — the same one autoland's
/// rollout squeezes), and how hard the push loop holds against idle creep.
const TAXI_BRAKE: &str = "xcopilot/taxi_brake";
const HOLD_BRAKE: f64 = 0.4;
/// Forward drift that starts the hold. Under `MOVED_M` on purpose: the hold
/// must catch the creep while the creep failure above must never fire from
/// jitter, and they are different jobs.
const HOLD_AT_M: f64 = 0.3;
/// The post-push hold's own thresholds: no backward progress for this long,
/// AND this far forward of the push's high-water mark, before a released
/// tug's dead air is held against. Both much tighter than the 12 s stall
/// clock — the Baron reached 21 kt inside one of these windows — and the
/// regression term keeps a swing's sideways plateau off the brakes.
const HOLD_STALL_SECS: f64 = 3.0;
const HOLD_REGRESS_M: f64 = 1.0;
/// Consecutive polls the attach/tow flag must hold before the tug counts as
/// CONNECTED. One blip is the tug positioning, and releasing the brakes on it
/// is how three pushes died in the dead air (2026-08-02).
const HOOK_SOLID_POLLS: u32 = 3;

/// What the push is aiming at, read off the cleared taxi route.
#[derive(Debug, Clone, PartialEq)]
pub struct Plan {
    /// How far back to push, straight-line metres from where we start.
    pub back_m: f64,
    /// The heading to be pointing when the tug lets go — down the outbound
    /// taxi lane, so the taxi drive starts by rolling forward.
    pub target_hdg: f64,
    /// The route node the push aims at (the lane), for the log.
    pub node: (f64, f64),
    /// Planned off the stand's PAINTED lane ([`lane_plan`]): the push ends at
    /// the lane's length, whatever the heading, and the taxi turns the rest.
    pub lane: bool,
}

/// Which way the tug is being asked to swing us.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Steer {
    Straight,
    Left,
    Right,
}

impl Steer {
    /// The `pushback` tool's action argument.
    fn action(self) -> &'static str {
        match self {
            Steer::Straight => "straight",
            Steer::Left => "left",
            Steer::Right => "right",
        }
    }
}

/// Is a push owed here, and where does it end? `None` means the way out is
/// ahead of us (or there is no route to read) — taxi straight off the stand.
///
/// The trigger is pure geometry against the cleared route: the first route
/// point that isn't the stand itself is the taxi lane, and the bearing to it
/// being behind the wing line is what "nose-in" means. The finish heading is
/// the lane's OWN direction (that node to the next one), not the reciprocal
/// of our push — a stand entered at an angle would otherwise leave us aimed
/// across the taxiway.
pub fn plan(lat: f64, lon: f64, psi: f64, path: &[(f64, f64)]) -> Option<Plan> {
    let from_us = |p: &(f64, f64)| nav::haversine_nm(lat, lon, p.0, p.1) * M_PER_NM;
    let (idx, node) = path
        .iter()
        .enumerate()
        .find(|(_, p)| from_us(p) >= MIN_NODE_M)?;
    if from_us(node) > MAX_NODE_M {
        return None; // the route doesn't start here — see MAX_NODE_M
    }
    let out_bearing = nav::bearing_deg(lat, lon, node.0, node.1);
    if crate::angles::abs_diff_deg(out_bearing, psi) < BEHIND_DEG {
        return None;
    }
    // Where we point when the tug lets go: down the lane. With no node beyond
    // it (a two-point route) the direction we pushed IS the only way out.
    let next = path[idx + 1..]
        .iter()
        .find(|p| nav::haversine_nm(node.0, node.1, p.0, p.1) * M_PER_NM >= MIN_NODE_M);
    let target_hdg = match next {
        Some(n) => nav::bearing_deg(node.0, node.1, n.0, n.1),
        None => out_bearing,
    };
    Some(Plan {
        back_m: from_us(node).clamp(MIN_BACK_M, MAX_BACK_M),
        target_hdg,
        node: *node,
        lane: false,
    })
}

/// The push planned off the stand's painted lane (`party_line::push_lane`,
/// the lead-in read backwards): as far as the lane runs, and the heading the
/// nose has at its end (the reverse of the lane's last direction). The route
/// plan above aims at the cleared route's first leg: at LIBP stand 1 that was
/// 217, a swing to the RIGHT that took a 737's tail into the buildings, where
/// the painted lane says the nose ends at 127, a turn to the LEFT
/// (2026-09-26). Pure.
pub fn lane_plan(lane: &[(f64, f64)]) -> Option<Plan> {
    if lane.len() < 2 {
        return None;
    }
    let seg = |a: (f64, f64), b: (f64, f64)| nav::haversine_nm(a.0, a.1, b.0, b.1) * M_PER_NM;
    let length: f64 = lane.windows(2).map(|w| seg(w[0], w[1])).sum();
    let (a, b) = lane
        .windows(2)
        .rev()
        .find(|w| seg(w[0], w[1]) >= 1.0)
        .map(|w| (w[0], w[1]))?;
    let travel = nav::bearing_deg(a.0, a.1, b.0, b.1);
    Some(Plan {
        back_m: length.clamp(MIN_BACK_M, MAX_BACK_M),
        target_hdg: (travel + 180.0).rem_euclid(360.0),
        node: *lane.last()?,
        lane: true,
    })
}

/// The push flown: the painted lane's, but only when the heading it ends on
/// leaves the cleared route's first leg AHEAD of the wing line (within
/// [`BEHIND_DEG`]); else the route's own plan, which swings the nose onto
/// that leg. A lead-in is painted for the traffic that arrives by it, so
/// read backwards it can end facing the way the arrivals came from. LEMG
/// (iMac 2026-09-29, sortie 1790712526, a Citation X): the lane ended on
/// 26.3 while the route left on 222.2, 164 degrees apart; 26.3 was inside
/// the straight band of the 327.7 nose, the tug pushed 51.5 m straight and
/// let go, and the taxi was left facing the stand with the way out behind
/// (#190). At LIBP stand 1 the two were 90 degrees apart (127 against 217)
/// and the lane was right (2026-09-26): a quarter turn is a taxi's. Returns
/// the plan and whether the lane was taken. Pure.
pub fn choose_plan(route: Plan, lane: Option<Plan>) -> (Plan, bool) {
    match lane {
        Some(l) if crate::angles::abs_diff_deg(l.target_hdg, route.target_hdg) <= BEHIND_DEG => (l, true),
        _ => (route, false),
    }
}

/// The tug command for a heading error, given which way `Right` actually
/// swings the nose under tow (`sense >= 0` = clockwise).
pub fn steer_for(err_deg: f64, sense: i8) -> Steer {
    if err_deg.abs() <= DEAD_DEG {
        return Steer::Straight;
    }
    // We want clockwise when the target is to the right of the nose.
    if (err_deg > 0.0) == (sense >= 0) {
        Steer::Right
    } else {
        Steer::Left
    }
}

/// Believe the observation over the guess: a sustained turn that made the
/// error WORSE means the commands are the other way round on this build.
/// Acted on once per push, with a tolerance — under tow the nose swings slowly
/// and the first seconds of a turn read as noise.
pub fn learn_sense(sense: i8, err_before: f64, err_now: f64) -> i8 {
    if err_now.abs() > err_before.abs() + SENSE_TOL_DEG {
        -sense
    } else {
        sense
    }
}

/// Has the running turn proved the sense wrong? Three things must hold: we are
/// actually MOVING (a turn commanded to a stationary aircraft proves nothing —
/// the clock used to start at the ask and expire in the dead air before the tug
/// even hooked on), a turn is what's commanded, and it has had `CAL_SECS` of
/// travel to show itself.
pub fn sense_is_wrong(
    towed: bool,
    turning: bool,
    secs_moving: f64,
    err_at: f64,
    err_now: f64,
    sense: i8,
) -> bool {
    towed && turning && secs_moving >= CAL_SECS && learn_sense(sense, err_at, err_now) != sense
}

/// Is the push finished? Far enough back AND pointing down the lane — or far
/// enough past that to stop chasing the alignment across the apron.
pub fn done(back_m: f64, err_deg: f64, plan: &Plan) -> bool {
    back_m >= plan.back_m + CHASE_M || (back_m >= plan.back_m && err_deg.abs() <= ALIGN_DEG)
}

/// Is the tug still turning the nose? Pure over (time, heading error, bar),
/// so the recorded pushes replay through it. A swing STARTS when the error
/// moves `SWING_DEG` over a span of at least `SWING_WINDOW_S` (the span is
/// measured, never assumed from the trim: the live window spanned 1.7 s) or
/// of `SWING_SETTLE_S` (a swing that builds slowly, see `SWING_SETTLE_S`);
/// it ENDS only when the nose has turned under `SWING_STILL_DEG` over
/// `SWING_SETTLE_S`, so a dip in the rate mid-swing is noise, not the end.
/// While the bar is on and this says swinging, the stall clock may not end
/// the push (a nose being swung is the tug moving us) and a rate dip is not
/// the end; the push is stopped mid-swing only ON the lane
/// ([`stop_on_the_lane`]).
#[derive(Debug, Clone, Default)]
pub struct SwingWatch {
    hist: std::collections::VecDeque<(f64, f64)>,
    swinging: bool,
    rate: f64,
}

impl SwingWatch {
    /// How far the error has moved since the newest sample at least `span`
    /// old; `None` until the history reaches back that far.
    fn turned_over(&self, t: f64, err: f64, span: f64) -> Option<f64> {
        self.hist
            .iter()
            .rev()
            .find(|(ts, _)| t - ts >= span)
            .map(|(_, e)| crate::angles::abs_diff_deg(err, *e))
    }

    pub fn step(&mut self, t: f64, err: f64, attached: bool) -> bool {
        // The error's rate over the measured window (signed, deg/s), before
        // this sample joins the history.
        self.rate = self
            .hist
            .iter()
            .rev()
            .find(|(ts, _)| t - ts >= SWING_WINDOW_S)
            .map_or(0.0, |(ts, e)| crate::angles::signed_diff_deg(err, *e) / (t - ts));
        self.hist.push_back((t, err));
        // Keep one sample at or beyond the longest span.
        while self.hist.len() > 2 && t - self.hist[1].0 >= SWING_SETTLE_S {
            self.hist.pop_front();
        }
        if self
            .turned_over(t, err, SWING_WINDOW_S)
            .is_some_and(|d| d >= SWING_DEG)
            || self
                .turned_over(t, err, SWING_SETTLE_S)
                .is_some_and(|d| d >= SWING_DEG)
        {
            self.swinging = true;
        } else if self
            .turned_over(t, err, SWING_SETTLE_S)
            .is_some_and(|d| d < SWING_STILL_DEG)
        {
            self.swinging = false;
        }
        self.in_progress(attached)
    }

    /// The last judgement, with the bar as read now: a swing with the bar
    /// OFF is not the tug's doing.
    pub fn in_progress(&self, attached: bool) -> bool {
        attached && self.swinging
    }

    /// The heading error's rate over the last `SWING_WINDOW_S`, deg/s,
    /// signed (0 until the history spans it).
    pub fn rate(&self) -> f64 {
        self.rate
    }
}

/// Where the nose comes to rest if "stop" goes out on this tick: the swing
/// carries on for `STOP_LATENCY_S`.
pub fn predicted_rest_err(err: f64, rate: f64) -> f64 {
    err + rate * STOP_LATENCY_S
}

/// Is this the tick to stop a tug mid-swing, the distance already made? We
/// CONTROL the push (`pushback_stop` = stop and let go): a nose swinging
/// onto the lane is stopped so it comes to rest ON it, led by the rate
/// ([`predicted_rest_err`]); a nose swinging away from the lane, or not
/// closing, is stopped at once while it is still inside `ALIGN_DEG`. Short
/// of the band the tug is left to bring it round.
pub fn stop_on_the_lane(err: f64, rate: f64) -> bool {
    let rest = predicted_rest_err(err, rate);
    if err * rate < 0.0 {
        // Closing: at the lane, or it would be past it by the stop.
        rest.abs() <= ON_LANE_DEG || rest.signum() != err.signum()
    } else {
        err.abs() <= ALIGN_DEG
    }
}

/// The heading after "stop" went out: a tug that ignores it keeps swinging
/// us (LIBP 2026-09-26: 204 to 262 over 31 s). Watched, never answered: a
/// command to a tug on the bar drops it off, so the travel is only logged
/// (`stop_ignored`) and the taxi takes the residual.
#[derive(Debug, Clone)]
pub struct AfterStop {
    at: Option<f64>,
    travel: f64,
    moved_s: f64,
}

impl AfterStop {
    /// `hdg`: the heading on the tick "stop" was sent.
    pub fn new(hdg: Option<f64>) -> Self {
        Self {
            at: hdg,
            travel: 0.0,
            moved_s: 0.0,
        }
    }

    /// `t` seconds since the stop; a failed read is no measurement.
    pub fn step(&mut self, t: f64, hdg: Option<f64>) {
        if let (Some(a), Some(h)) = (self.at, hdg) {
            let d = crate::angles::signed_diff_deg(h, a);
            if d.abs() > self.travel.abs() {
                self.travel = d;
                self.moved_s = t;
            }
        }
    }

    /// Signed degrees the nose moved past the stop's heading, at most.
    pub fn travel_deg(&self) -> f64 {
        self.travel
    }

    /// Seconds after the stop the nose was last moving further.
    pub fn moved_s(&self) -> f64 {
        self.moved_s
    }

    pub fn ignored(&self) -> bool {
        self.travel.abs() > STOP_IGNORED_DEG
    }
}

/// What this tick of the push says. [`PushJudge`] is the whole ending rule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PushVerdict {
    Pushing,
    /// Finished by the numbers but the tug is mid-swing: waiting for it
    /// (returned on the first such tick only, for the log).
    SwingWait,
    /// Finished: stop and let go.
    Complete,
    /// Finished mid-swing, the nose coming to rest on the lane: stop and
    /// let go now, swing or no swing ([`stop_on_the_lane`]).
    Aligned,
    /// Motionless, nose included, for `STALL_SECS` after moving: the tug
    /// has run its pattern and stopped.
    Stalled,
}

/// The push's ending rule, pure over seconds since the brakes came off, so
/// the recorded ticks replay through exactly what the loop runs. We control
/// the push: once the distance is made and the nose is (or is coming to
/// rest) on the lane, it is stopped, swing or no swing. The swing
/// ([`SwingWatch`]) only guards the FALSE stop: the stall clock never runs
/// while the nose is being swung, and a distance-only finish (the chase
/// margin, a straight or lane push) waits for the swing to end.
#[derive(Debug, Clone)]
pub struct PushJudge {
    swing: SwingWatch,
    swinging: bool,
    moved_at: f64,
    moved_m: f64,
    swing_waited: bool,
}

impl PushJudge {
    pub fn new(t: f64) -> Self {
        Self {
            swing: SwingWatch::default(),
            swinging: false,
            moved_at: t,
            moved_m: 0.0,
            swing_waited: false,
        }
    }

    /// Stalled for a beat AND regressing forward off the high-water mark:
    /// idle creep, not a swing's sideways plateau (see [`should_hold_creep`]).
    pub fn sliding(&self, t: f64, back_m: f64) -> bool {
        t - self.moved_at >= HOLD_STALL_SECS && back_m < self.moved_m - HOLD_REGRESS_M
    }

    /// The tug swinging us, as of the last step.
    pub fn swinging(&self) -> bool {
        self.swinging
    }

    /// The heading error's rate as of the last step, deg/s.
    pub fn swing_rate(&self) -> f64 {
        self.swing.rate()
    }

    /// `straight`: the pattern asked was straight back (or the plan is a
    /// painted lane), which ends at the planned distance.
    pub fn step(
        &mut self,
        t: f64,
        back_m: f64,
        err: f64,
        attached: bool,
        towed: bool,
        plan: &Plan,
        straight: bool,
    ) -> PushVerdict {
        let swinging = self.swing.step(t, err, attached);
        self.swinging = swinging;
        let stalled = if back_m > self.moved_m + MOVED_M {
            self.moved_m = back_m;
            self.moved_at = t;
            false
        } else if swinging {
            self.moved_at = t;
            false
        } else {
            towed && t - self.moved_at >= STALL_SECS
        };
        let reached = back_m >= plan.back_m;
        // A straight push never aligns the nose (by design: the taxi turns
        // it), so it ends at the planned distance, not 25 m past it.
        let by_distance = (straight || plan.lane) && reached;
        if !swinging {
            if by_distance || done(back_m, err, plan) {
                return PushVerdict::Complete;
            }
        } else if reached && stop_on_the_lane(err, self.swing.rate()) {
            // Delivered: far enough, and the nose comes to rest on the lane
            // if we stop it now. The tug is not left to swing us past it.
            return PushVerdict::Aligned;
        } else if by_distance || done(back_m, err, plan) {
            // Finished by distance alone while the tug swings us: stopping
            // now would leave the nose wherever the swing had got to (LXGB
            // 2026-09-15: 140 degrees off). Wait for the lane or the stop.
            if !self.swing_waited {
                self.swing_waited = true;
                return PushVerdict::SwingWait;
            }
        }
        if stalled {
            PushVerdict::Stalled
        } else {
            PushVerdict::Pushing
        }
    }
}

/// Would asking for ANOTHER pattern actually improve things?
///
/// Two ways it can, and only two. Real DISTANCE is still owed, which a
/// straight segment earns. Or at least half a SWING is — and half is the bar
/// because a pattern is 90 degrees whether we want 90 or 10, so below 45 the
/// next one overshoots by more than the error it was sent to remove.
///
/// At Malaga the tug ran a textbook Nose 90: 55 m back, the nose swung 041 to
/// 311, and it stopped and let go exactly as the command says it will. The
/// plan had wanted 135 degrees, which one pattern can never deliver, so the
/// loop found a 44.6-degree residual and asked for another 90 — a swing that
/// would have left us 45 degrees off the OTHER way. Twice, on two sorties at
/// the same stand (2026-08-03).
pub fn worth_another_pattern(back_m: f64, err_deg: f64, plan: &Plan) -> bool {
    if back_m >= plan.back_m + CHASE_M {
        return false; // past the plan and its chase margin: stop spending apron
    }
    back_m < plan.back_m - SHORT_PUSH_M || err_deg.abs() >= PATTERN_DEG / 2.0
}

/// Has the tug delivered everything a tug CAN deliver? [`done`] is the PLANNED
/// finish; this is the achievable one, and the difference is what failed two
/// perfectly executed pushes.
///
/// The residual belongs to the taxi drive, which is built for it: the join
/// pure-pursuits onto the route from any angle and has turned a nose-in Twin
/// Otter through 141 degrees under its own steam (Gatwick, 2026-07-27). Forty
/// degrees is a taxi turn, not a turn-around, and refusing to hand it over
/// costs the pilot a "the tug let go early" for a manoeuvre they watched go
/// perfectly out of the window — the same wrong verb the 2026-07-28 Stansted
/// fix removed one state earlier.
/// How much of the planned distance a push with the NOSE where we asked for
/// it has to have covered to count as delivered. The straight-metres test
/// below it ([`worth_another_pattern`]) is a fixed ten, which is the right
/// question for a fifty-metre GA push and a cliff for an airliner's ninety:
/// a 737 asked for 88.7 m, got 77.4 with the heading 1.1 degrees off — a
/// textbook push anyone watching would call finished — and failed by 1.3 m
/// (LFLL, 2026-09-17). Those metres are three seconds of taxi.
const DELIVERED_FRACTION: f64 = 0.8;

pub fn push_delivered(back_m: f64, err_deg: f64, plan: &Plan) -> bool {
    done(back_m, err_deg, plan)
        || (back_m >= MIN_BACK_M
            && (!worth_another_pattern(back_m, err_deg, plan)
                // Or: the tug ran its pattern, the nose came round to the lane,
                // and only the last few metres of straight are owed. Both
                // callers are TERMINAL — the bar is off, or a second command
                // would knock it off — so nothing is going to earn those
                // metres but the taxi drive, which is built for them.
                || (err_deg.abs() <= ALIGN_DEG
                    && back_m >= plan.back_m * DELIVERED_FRACTION)))
}

/// What a push that fell short actually fell short OF — the distance, or the
/// turn. One label covered both, so a tug that pushed a 737 twelve metres
/// PAST the plan and left the nose 71 degrees off the lane was blackboxed as
/// "short" (LFLL, 2026-09-17); a residual you can read the wrong way is worse
/// than no label.
pub fn push_residual(back_m: f64, err_deg: f64, plan: &Plan) -> &'static str {
    if err_deg.abs() > ALIGN_DEG && back_m >= plan.back_m * DELIVERED_FRACTION {
        "unturned"
    } else {
        "short"
    }
}

/// How near a published parking position the aircraft must be, and how
/// close to its parking heading the nose must point, to count as ON it.
/// Stands are tens of metres apart and a nose-in aircraft sits within a few
/// degrees of the painted heading; a taxiway node is neither.
const STAND_M: f64 = 35.0;
const STAND_ALIGN_DEG: f64 = 60.0;

/// Is the aircraft parked on a stand, so that a tug is the right tool?
///
/// A pushback belongs to a stand. The trigger in [`plan`] only asks whether
/// the route's first node is behind us, and after an off-route stop it is:
/// every full-auto restart from the middle of LEMG's Bravo called a tug,
/// waited minutes for it and pushed the aircraft back down a taxiway
/// (four times, 2026-09-12). `stand` is the nearest published position
/// (metres, parking heading); `known` whether the field publishes any. A
/// field with no stand rows keeps the bearing-only rule (nothing to check
/// against); one WITH them says no unless we are on one, nose aligned.
pub fn on_a_stand(stand: Option<(f64, f64)>, psi: f64, known: bool) -> bool {
    if !known {
        return true;
    }
    match stand {
        Some((dist_m, heading)) => {
            dist_m <= STAND_M && crate::angles::abs_diff_deg(heading, psi) <= STAND_ALIGN_DEG
        }
        None => false,
    }
}

/// The heaviest aircraft that turns itself off a stand instead of calling a
/// tug: ICAO's light class (5,700 kg maximum takeoff mass).
pub const LIGHT_MTOW_KG: f64 = 5_700.0;

/// Does this airframe call a tug at all? `mtow_kg` is `acf_m_max`; a failed
/// read is no measurement and keeps the tug. A light aircraft turns round on
/// the apron under its own power, as it would in life, instead of waiting
/// minutes for a vehicle: LEMG's GA tie-downs (100 to 107) kept an SR22
/// waiting 360 s twice for a tug that never came (sorties 1790605551 and
/// 1790618416, 2026-09-28), and the Baron's pushes there hooked at 261 to
/// 289 s when one came; the turn-round that followed joined in 14 s.
pub fn tug_for_weight(mtow_kg: Option<f64>) -> bool {
    match mtow_kg {
        Some(w) if w.is_finite() && w > 0.0 => w > LIGHT_MTOW_KG,
        _ => true,
    }
}

/// Is the aircraft PARKED at a stand: within [`STAND_M`] of one, whichever
/// way the nose points. The heading is the tug's question (which way to
/// push), not the securing's: EDLP 7C was reached 2 m from its spot and 147
/// degrees across it, and "stopped short of the stand" left the aircraft
/// unsecured (2026-09-24). A field with no stand rows counts as parked.
pub fn parked_at_stand(stand: Option<(f64, f64)>, known: bool) -> bool {
    !known || stand.is_some_and(|(dist_m, _)| dist_m <= STAND_M)
}

/// Where the wait for the tug to let go stands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TugWait {
    /// Off the bar: the parking brake may go on.
    Clear,
    Waiting,
    /// Still on the bar at the deadline.
    TimedOut,
}

/// The wait for the tug to unhook, pure over (time, reads) so the recorded
/// push replays through it. The deadline runs from the last moment the nose
/// was still being swung (the push loop's slow swing rate, `SWING_DEG` per
/// `SWING_SETTLE_S`: EGBB's tug turned us at 0.7 deg/s after its stop),
/// capped at `TUG_CLEAR_MAX_SECS` from the start. A failed
/// read of the bar is no measurement, never "clear": the old wait read a
/// failure as 0.0, off the bar. `tow_in_progress` is dead on this sim (see
/// [`TOW_IN_PROGRESS`]) and only blocks when it actually reads on.
#[derive(Debug, Clone)]
pub struct TugClearWatch {
    began: f64,
    last_moved: f64,
    hdg_hist: std::collections::VecDeque<(f64, f64)>,
}

impl TugClearWatch {
    pub fn new(t: f64) -> Self {
        Self {
            began: t,
            last_moved: t,
            hdg_hist: Default::default(),
        }
    }

    /// Is the tug still turning us? The rate over the last window.
    pub fn swinging(&self) -> bool {
        match (self.hdg_hist.front(), self.hdg_hist.back()) {
            (Some(&(t0, h0)), Some(&(t1, h1))) if t1 - t0 >= 0.5 * SWING_WINDOW_S => {
                crate::angles::abs_diff_deg(h1, h0) / (t1 - t0) >= SWING_DEG / SWING_SETTLE_S
            }
            _ => false,
        }
    }

    pub fn step(
        &mut self,
        t: f64,
        attached: Option<f64>,
        tow: Option<f64>,
        hdg: Option<f64>,
    ) -> TugWait {
        if let Some(h) = hdg {
            self.hdg_hist.push_back((t, h));
            // Keep one sample at or beyond the window so the rate always
            // spans it (the poll is 0.5 s, the recorded states 1.3 s).
            while self.hdg_hist.len() > 2 && t - self.hdg_hist[1].0 >= SWING_WINDOW_S {
                self.hdg_hist.pop_front();
            }
            if self.swinging() {
                self.last_moved = t;
            }
        }
        let off_bar = attached.is_some_and(|a| a <= 0.5) && !tow.is_some_and(|w| w > 0.5);
        if off_bar {
            TugWait::Clear
        } else if t - self.last_moved >= TUG_CLEAR_SECS || t - self.began >= TUG_CLEAR_MAX_SECS {
            TugWait::TimedOut
        } else {
            TugWait::Waiting
        }
    }
}

/// How far BACK we have come: displacement from the stand projected onto the
/// reverse of the heading we started on. Signed — negative means we have
/// rolled FORWARD off the stand.
///
/// This has to be signed, and the reason is a whole sortie. It was the plain
/// haversine distance from the start point, which is a MAGNITUDE: a metre
/// forward and a metre back read identically, exactly the trap the taxi guide
/// hit when four knots backwards narrated as "next left" (2026-07-28). At
/// Southampton the tug hooked on, never pushed, and the Baron — brakes off for
/// the hook-up, both engines idling — crept FORWARD off the stand. The creep
/// counted as pushback progress, crossed `MOVED_M`, latched `towed`, and with
/// the attach flag reading false the loop concluded the tug had let go after
/// pushing us 1.2 m. It then waited out a re-hook that was never coming while
/// the aircraft accelerated to 21 kt across the apron (2026-08-01).
///
/// Projected onto the START heading, not the current one, because a "Nose 90"
/// pattern swings the aircraft up to a right angle mid-push: the nose ends up
/// across the displacement, but the displacement is still behind where the
/// nose BEGAN, which is what "how far back have we come" means.
pub fn back_along(start_lat: f64, start_lon: f64, start_psi: f64, lat: f64, lon: f64) -> f64 {
    let d_m = nav::haversine_nm(start_lat, start_lon, lat, lon) * M_PER_NM;
    if d_m <= 0.0 {
        return 0.0;
    }
    let brg = nav::bearing_deg(start_lat, start_lon, lat, lon);
    // Angle between where we went and straight back up the original tail.
    let off = crate::angles::signed_diff_deg(brg, start_psi + 180.0).to_radians();
    d_m * off.cos()
}

/// How far behind the aircraft the pavement is looked for. A tug honours
/// "stop" on its straight segment, not mid-swing, so the edge has to be seen
/// before the swing that carries the aircraft over it: at LIPK (2026-09-26)
/// the 172 left the apron at 102 m, the second its swing began, and this
/// much look-behind finds that edge at about 92 m, still straight.
pub const PAVEMENT_LOOK_M: f64 = 10.0;

/// Is the push at the pavement's edge? Only for a push that STARTED on drawn
/// pavement (a grass parking row is left alone), and never on no data.
///
/// The look-behind is a STRAIGHT push's geometry: it is where the tail goes
/// only while the tug pushes us straight back. Once the tug swings us
/// (`swinging`, [`SwingWatch`]) the reference point runs round an arc that
/// turns away from that line, so only being OFF the pavement stops a swing,
/// never a point 10 m down the line it is leaving. At EGBB (sortie
/// 1790534902, 2026-09-27) the edge was read behind at 41.2 of 49.6 m with
/// the aircraft on the pavement and the nose 82 degrees off, a second into
/// the Nose 90's swing; "stop" went to a tug mid-swing, which ignored it and
/// finished the turn to 235 against 238.7 on the pavement.
pub fn at_pavement_edge(
    started_on: Option<bool>,
    here: Option<bool>,
    behind: Option<bool>,
    swinging: bool,
) -> bool {
    started_on == Some(true) && (here == Some(false) || (!swinging && behind == Some(false)))
}

/// Should this tick squeeze the brake channel against forward creep? Only
/// while NOBODY is pushing (a tug mid-arc can carry the reference point
/// forward, and a push already under way must never be braked against) and
/// the aircraft has drifted forward of the stand — the idle-thrust walk that
/// killed three EGHI pushes in the tug's own dead air (2026-08-02).
pub fn should_hold_creep(
    towed: bool,
    towing: bool,
    attached: bool,
    back_m: f64,
    sliding: bool,
) -> bool {
    if towing || attached {
        return false;
    }
    if !towed {
        return back_m < -HOLD_AT_M;
    }
    // After the push has moved us the start line is metres behind, so the
    // pre-push test above can never fire again — and `towed` is a latch that
    // never clears, so on the airframe whose attach flag only ever blips, a
    // tug that let go mid-plan left the aircraft brakeless at idle through
    // the whole re-issue window: 25 m of push, release, and the same
    // idle-thrust walk the hold exists for, now measured from a high-water
    // mark instead of the stand (audit, 2026-08-02). SLIDING — stalled for a
    // beat AND regressing forward off that mark — is what tells idle creep
    // from a live swing's sideways plateau, which must never meet a brake.
    sliding
}

fn event(detail: serde_json::Value) {
    crate::blackbox::record("pushback", detail);
}

/// Drive the push: call the tug, steer it onto the lane, stop, and wait for
/// the tug to clear. Returns how far back we ended up, or why it didn't
/// happen — the caller decides what a failure means (full auto falls back to
/// taxiing off the stand under its own steering rather than standing down).
///
/// `abort` is the caller's cancel flag ("my controls"), checked every tick;
/// an abort always stops the tug before returning.
/// Which way `pushback_right` actually swings the nose on THIS sim build:
/// +1 = clockwise (the naive reading), -1 = counter-clockwise, 0 = not yet
/// observed. Learned once under tow and kept for the SESSION, not per push:
/// re-learning it every push is how both Luton pushes died on 2026-07-28 —
/// each started from the same wrong guess, each convicted it ~46 m in, and
/// the mid-push correction made the tug let go (see the flip site). Never
/// persisted to disk — a sim update may change it, and one observation per
/// session is cheap.
static LEARNED_SENSE: AtomicI8 = AtomicI8::new(0);

/// True while a tug is pushing us. The taxi guide reads it and goes quiet:
/// groundspeed is a MAGNITUDE, so being pushed back at four knots looks exactly
/// like taxiing forward at four, and the stand is the cleared route's first
/// point — so the guide happily called the turns of a route we were being
/// pushed away from ("Rolling again. Sharp left next, onto Bravo", Luton
/// 2026-07-28, with the tug still connected).
pub static ACTIVE: AtomicBool = AtomicBool::new(false);

/// True while the STEERED push loop ([`run`]) is working the manoeuvre — the
/// one consumer of [`ABORT`]. The spoken tool's native one-shot sets only
/// [`ACTIVE`], and the distinction is what tells the command loop whether a
/// stop word has a listener or must stop the tug itself: ABORT against a
/// native push was a flag nobody read, and every stop-phrase — the panel
/// CANCEL's "stop pushback" included — was consumed with the tug still
/// pushing (audit, 2026-08-02).
static STEERED: AtomicBool = AtomicBool::new(false);

/// Is the steered loop the thing holding [`ACTIVE`] right now?
pub fn steered() -> bool {
    STEERED.load(SeqCst)
}

/// The pilot's own stop, set from the command loop. Separate from the caller's
/// abort flag (full auto's), because "stop" during a push is about THIS
/// manoeuvre and must not depend on who armed it: a push holds the aircraft
/// with the brakes off and no governor running, so it is the one stage where
/// the word has to land somewhere. Cleared at the start of every run.
pub static ABORT: AtomicBool = AtomicBool::new(false);
/// "Skip the push": end the steered push (tug stopped, brakes on) and let
/// full auto turn the aircraft round under its own steam. Set by the
/// pushback tool's stop action while a steered push runs; consumed by the
/// loop, which returns `Err("skipped")`. Distinct from [`ABORT`], which is
/// full auto's own stop and ends the whole mission.
pub static SKIP: AtomicBool = AtomicBool::new(false);

/// Either flag — the caller's, or the pilot's.
fn stopping(abort: &AtomicBool) -> bool {
    abort.load(SeqCst) || ABORT.load(SeqCst)
}

/// One-line status for the dashboard/panel; the stage detail (tug wait, under
/// tow) is full auto's to narrate — this only says a push is running.
pub fn status() -> Option<String> {
    ACTIVE.load(SeqCst).then(|| "pushback".to_string())
}

/// Run the push, holding [`ACTIVE`] for its duration however it ends.
pub async fn run(
    xp: &XPlaneClient,
    ctx: &ToolCtx<'_>,
    voice: &Option<Voice>,
    plan: &Plan,
    abort: &AtomicBool,
    valve_set: Option<f64>,
) -> Result<f64, String> {
    ACTIVE.store(true, SeqCst);
    STEERED.store(true, SeqCst);
    ABORT.store(false, SeqCst);
    SKIP.store(false, SeqCst);
    let mut out = run_inner(xp, ctx, voice, plan, abort, valve_set).await;
    // The pilot's stop that landed after the loop's last look (it returned
    // on its own a tick later): still answered, brakes first, from their
    // read-back, never left to a line said before anything was set (#110).
    if out.is_ok() && ABORT.load(SeqCst) {
        send(ctx, "stop").await;
        abort_brakes(&xp.unscoped(), voice, valve_set, "ended").await;
        out = Err("abort".into());
    }
    // Stop-and-let-go on EVERY exit, unconditionally. A pushback command is a
    // whole manoeuvre the tug runs on its own, and a re-issued continuation
    // outlives any return path that forgets this line: the MAX_ISSUES exit
    // returned mid-manoeuvre and the tug kept working the aircraft in stages
    // — through full auto's taxi attempt and past its stand-down — until the
    // pilot asked what on earth was going on and the command had to be fired
    // by hand over the Web API (2026-07-29). Idempotent when already stopped.
    send(ctx, "stop").await;
    STEERED.store(false, SeqCst);
    ACTIVE.store(false, SeqCst);
    out
}

async fn run_inner(
    xp: &XPlaneClient,
    ctx: &ToolCtx<'_>,
    voice: &Option<Voice>,
    plan: &Plan,
    abort: &AtomicBool,
    // The airframe's park-brake valve SET sense — brakes are held for the
    // tug wait and re-set at disconnect through the same valve-aware routine
    // the parking-brake tool uses.
    valve_set: Option<f64>,
) -> Result<f64, String> {
    let (Ok(start_lat), Ok(start_lon), Ok(start_psi)) =
        tokio::join!(xp.get_float(LAT), xp.get_float(LON), xp.get_float(PSI_TRUE))
    else {
        return Err("no_position".into());
    };
    // Trusted wherever the stand itself is drawn. A "coverage" gate (most
    // nearby taxi nodes inside the polygons) was tried at LIBP, where 25 of
    // 66 nodes are: it switched the guard off, and the tug's straight leg
    // ran a 737 into the buildings behind the stand the guard had stopped
    // it short of one flight earlier (2026-09-26). An edge in the data is an
    // edge: the taxi turns the aircraft from there.
    let started_on_pavement = crate::party_line::on_pavement(start_lat, start_lon);
    // The MANOEUVRE, picked once and up front — X-Plane's three pushback
    // commands are not a steering channel, they are whole-push patterns the
    // tug executes and finishes ("Nose 90 left" / "Straight back" / "Nose 90
    // right", per Commands.txt), and `pushback_stop` is "stop and LET GO".
    // Steering it per tick like a rudder is what the first version did: the
    // loop picked `left` correctly, then its own twelve-second nudge re-sent
    // `straight`, which RE-TARGETED the tug — and because the tracked command
    // still read Left, nothing ever re-sent Left. The aircraft was pushed 51 m
    // dead straight with its heading unchanged to a tenth of a degree, 115°
    // off the lane, and the taxi join had to turn a Gatwick MD-82 round on the
    // apron (2026-07-27).
    let err0 = crate::angles::signed_diff_deg(plan.target_hdg, start_psi);
    // Start from what this session has already OBSERVED — and when nothing
    // has been observed yet, from what the BUILD has been measured to do:
    // three pushes on 2026-07-28 (two at Luton, one at Stansted), three
    // identical verdicts, full turn trails in the blackbox — X-Plane 12.4.3's
    // `pushback_left` swings the NOSE clockwise, the opposite of its
    // "Nose 90 left" description. So the naive reading is the wrong default
    // on the one build we can measure. Still learned per session: a future
    // build that fixes the command names flips one push's first turn, not
    // every push forever.
    let sense0: i8 = match LEARNED_SENSE.load(SeqCst) {
        0 => -1,
        v => v,
    };
    let cmd_now = steer_for(err0, sense0);
    // The arm log carries the GEOMETRY, not just the plan: "no tug came" is
    // undiagnosable afterwards without knowing which stand we sat on and how
    // far the lane was (the Heathrow lesson — name every gate in the log).
    event(json!({
        "event": "engaged",
        "back_m": r1(plan.back_m),
        "target_hdg": r1(plan.target_hdg),
        "from": [start_lat, start_lon],
        "node": [plan.node.0, plan.node.1],
        "err": r1(err0),
        "pattern": cmd_now.action(),
    }));
    // Issuing a pushback command with no tug about is what SUMMONS one.
    send(ctx, cmd_now.action()).await;

    // Brakes SET for the wait — the real call is "parking brake set, expecting
    // the tug", and the wait is minutes long. This is not optional dressing:
    // full auto used to release the brakes right after engine start, and an
    // idle jet on a flat apron rolls — the SF50 crept to 2.8 kt brakeless
    // while this very wait was running and went into the grass (Luton,
    // 2026-07-28). Valve-aware, same routine the parking-brake tool uses; a
    // failure here is not fatal (the pilot may be holding the toe brakes).
    let _ = crate::tools::controls::drive_park_brake(ctx.xplane, true, valve_set).await;

    // The tug drives up and hooks on BEFORE anything moves — two datarefs,
    // two moments: `pushback_attached` when it's on the bar, `tow_in_progress`
    // once we're actually rolling back. Either one means a tug came; waiting
    // on the tow alone would read the hook-up pause as no tug at all.
    //
    // The wait is MINUTES, not seconds, because the tug is a vehicle that has
    // to drive to us: X-Plane searches 1500 m for one and lets it drive up to
    // 6 km (`sim/private/controls/trucks/max_search_radius_m` /
    // `max_drive_distance_m`), so at a field the size of Gatwick the trip is
    // long. A 30 s limit gave up on the FIRST live push (2026-07-27) and fell
    // back to turning the aircraft round on the stand — the tug wasn't
    // missing, it was still driving. Waiting silently would be its own bug, so
    // the crew says what we're waiting for once the wait stops looking normal.
    let hooked = {
        let began = Instant::now();
        let mut said = false;
        let mut said2 = false;
        let mut logged = began;
        // CONNECTED means connected, not glimpsed. All three dead EGHI pushes
        // show the attach flag up for exactly ONE poll as the tug positioned,
        // false again by the first push tick — and the brakes came off on that
        // blip, four seconds before the creep guard killed the push the tug
        // had not started. The pilot's rule is the right one: brakes stay ON
        // until the tug is solidly on the bar, THEN release (2026-08-02).
        let mut solid = 0u32;
        loop {
            if stopping(abort) || SKIP.load(SeqCst) {
                break false;
            }
            let (a, t) = tokio::join!(xp.get_float(PUSH_ATTACHED), xp.get_float(TOW_IN_PROGRESS));
            if a.unwrap_or(0.0) > 0.5 || t.unwrap_or(0.0) > 0.5 {
                solid += 1;
                if solid >= HOOK_SOLID_POLLS {
                    event(
                        json!({ "event": "hooked", "waited_s": r1(began.elapsed().as_secs_f64()) }),
                    );
                    break true;
                }
                tokio::time::sleep(Duration::from_millis(TICK_MS)).await;
                continue;
            }
            solid = 0;
            let waited = began.elapsed().as_secs_f64();
            if waited >= TUG_WAIT_SECS {
                break false;
            }
            if !said && waited >= TUG_PATIENCE_SECS {
                said = true;
                if let Some(v) = voice {
                    v.alert(Priority::Advisory, "Still waiting on the tug.".to_string());
                }
            }
            if !said2 && waited >= TUG_PATIENCE2_SECS {
                said2 = true;
                if let Some(v) = voice {
                    v.alert(
                        Priority::Advisory,
                        "Tug's taking its time — I'll give it a few more minutes.".to_string(),
                    );
                }
            }
            if logged.elapsed().as_secs_f64() >= 15.0 {
                logged = Instant::now();
                event(json!({ "event": "waiting", "waited_s": r1(waited) }));
            }
            tokio::time::sleep(Duration::from_millis(TICK_MS)).await;
        }
    };
    if !hooked {
        // Cancel the request as well as the wait: a tug still driving toward
        // us must not hook onto an aircraft that has given up and started
        // taxiing.
        send(ctx, "stop").await;
        if SKIP.swap(false, SeqCst) {
            event(json!({ "event": "skipped", "stage": "waiting" }));
            return Err("skipped".into());
        }
        let aborted = stopping(abort);
        if aborted {
            // Set for the wait already; driven again for the read-back the
            // pilot's "stop" is answered from.
            abort_brakes(&xp.unscoped(), voice, valve_set, "waiting").await;
        }
        event(json!({ "event": "no_tug", "waited_s": TUG_WAIT_SECS }));
        return Err(if aborted { "abort" } else { "no_tug" }.into());
    }

    // NOTHING is sent to a tug on the bar. The summon carried the pattern,
    // and a command to an attached tug drops it off the bar: every push in
    // the log (five for five, 2026-09-12) shows `hooked`, the old re-issue,
    // and the attach flag false on the very next poll, the tug driving off
    // and coming back ninety seconds later to push. The 2026-07-27 premise
    // ("nothing moved until the command was sent again") was that cycle seen
    // from the inside: the re-send was what made it leave, and the push that
    // "began a minute later" was its return. The tug pushes when the brakes
    // come off.
    event(json!({ "event": "on_bar", "pattern": cmd_now.action() }));

    // Hooked, instructed: NOW the brakes come off — the crew's "brakes
    // released" answer to the tug's "brakes off please", and the first moment
    // releasing them is safe (something is holding the aircraft).
    let _ = crate::tools::controls::drive_park_brake(ctx.xplane, false, valve_set).await;
    event(json!({ "event": "brakes_released" }));

    let began = Instant::now();
    // The guess (or the session's learned answer). Still checked against the
    // aircraft's own behaviour on the first sustained turn, once.
    let mut sense: i8 = sense0;
    let mut flipped = false;
    let cmd = cmd_now;
    // When the pattern went on, and the error it started from — the pair the
    // sense is learned from.
    let mut turn_since: Option<(Instant, f64)> = (cmd != Steer::Straight).then_some((began, err0));
    let mut back_m = 0.0;
    // The ending rule: last time we actually moved (a swing counts), how far
    // we'd got then, and the swing watch. The tug runs its chosen manoeuvre
    // and STOPS, so "stopped after moving" is completion, not a hang, and the
    // loop must not sit out MAX_SECS waiting for more.
    let mut judge = PushJudge::new(0.0);
    // Whether we've ever actually been under tow, and whether the "push" has
    // been repeated to a tug that hooked on but never moved.
    let mut towed = false;
    let mut nudged = false;
    // Whether the attach flag has EVER read true inside this loop. On the
    // Baron at EGHI it blips once during the hook wait and reads false from
    // the first push tick on — three pushes in a row, 2026-08-02 — so "the
    // bar came off" can only mean something on an airframe where the bar was
    // ever seen ON. Without this gate, a real push on that airframe would be
    // declared released `MOVED_M` metres in, and the re-issue would re-aim
    // the tug mid-manoeuvre.
    let mut saw_attached = false;
    // The heading and error on the last tick read, for the stop's record and
    // the watch after it (`AfterStop`).
    let mut last_psi: Option<f64> = None;
    let mut last_err: Option<f64> = None;
    loop {
        if stopping(abort) {
            send(ctx, "stop").await;
            // Brakes on. The bar may be off and the engines are running, so an
            // abort that only stops the TUG leaves the aircraft rolling —
            // the same hole the re-hook wait had. Answered from the brake's
            // read-back, never before it (#110).
            abort_brakes(&xp.unscoped(), voice, valve_set, "pushing").await;
            event(json!({ "event": "abort", "back_m": r1(back_m) }));
            return Err("abort".into());
        }
        if began.elapsed().as_secs_f64() > MAX_SECS {
            send(ctx, "stop").await;
            // Mid-apron with no tug and no drive coming for seconds yet:
            // brakes on. The taxi join's governor releases them when it arms.
            let _ = crate::tools::controls::drive_park_brake(ctx.xplane, true, valve_set).await;
            event(json!({ "event": "timeout", "back_m": r1(back_m) }));
            return Err("timeout".into());
        }
        // Paused or in replay: the tug is frozen with everything else, and a
        // push measured across the freeze would read as a stall.
        if crate::xplane::sim_frozen(xp).await {
            tokio::time::sleep(Duration::from_millis(TICK_MS)).await;
            continue;
        }
        if SKIP.swap(false, SeqCst) {
            send(ctx, "stop").await;
            let _ = crate::tools::controls::drive_park_brake(ctx.xplane, true, valve_set).await;
            event(json!({ "event": "skipped", "stage": "pushing", "back_m": r1(back_m) }));
            return Err("skipped".into());
        }
        let (lat_r, lon_r, psi_r, tow_r, att_r) = tokio::join!(
            xp.get_float(LAT),
            xp.get_float(LON),
            xp.get_float(PSI_TRUE),
            xp.get_float(TOW_IN_PROGRESS),
            xp.get_float(PUSH_ATTACHED),
        );
        let (Ok(lat), Ok(lon), Ok(psi)) = (lat_r, lon_r, psi_r) else {
            tokio::time::sleep(Duration::from_millis(TICK_MS)).await;
            continue;
        };
        let towing = tow_r.is_ok_and(|v| v > 0.5);
        let attached = att_r.is_ok_and(|v| v > 0.5);
        saw_attached |= attached;
        back_m = back_along(start_lat, start_lon, start_psi, lat, lon);
        // HOLD against idle creep — don't fail on it. The dead air between the
        // hook-up and the tug actually rolling is MINUTES on this sim (Luton:
        // 74 s; the nudge era: "the push began a minute later"), the brakes
        // are off for the tug, and a Baron at idle walks forward in seconds.
        // Three EGHI pushes died exactly there on 2026-08-02: hooked, brakes
        // released, forward 3 m in four seconds, `creeping`, every time — the
        // creep guard was killing pushes the tug had simply not started yet.
        // So while nobody is pushing and we are drifting FORWARD, squeeze the
        // expiring proportional brake channel instead: it holds the aircraft
        // this tick and frees itself in a second, so a tug that begins its
        // manoeuvre is never fighting a latched brake.
        let sliding = judge.sliding(began.elapsed().as_secs_f64(), back_m);
        if should_hold_creep(towed, towing, attached, back_m, sliding) {
            let _ = xp.set_float(TAXI_BRAKE, HOLD_BRAKE).await;
        }
        // Going the WRONG WAY with nobody pushing: the brakes are off for the
        // tug and the engines are running, so this is our own idle thrust
        // walking us off the stand. Nothing downstream can recover from it —
        // the taxi fallback would join a route from an aircraft already
        // rolling — so it ends the push here, brakes on. Gated on not being
        // under tow, because a tug swinging a long nose CAN carry the
        // reference point forward for a moment mid-arc. With the hold above
        // this is the backstop for a missing brake channel, not the norm.
        if !towing && !attached && back_m < -CREEP_M {
            send(ctx, "stop").await;
            let _ = crate::tools::controls::drive_park_brake(ctx.xplane, true, valve_set).await;
            event(json!({ "event": "creeping", "back_m": r1(back_m) }));
            return Err("creeping".into());
        }
        // "Under tow" is measured off our OWN movement. The dataref that
        // sounds like it says this doesn't (see PUSH_ATTACHED): live, it read
        // 0 through an entire push, which left `towed` false forever — the
        // release check below could never fire, and the nudge fired at twelve
        // seconds on every push that was already rolling normally.
        towed |= towing || back_m > MOVED_M;
        // The bar came off after we'd rolled. This is NOT a failure — it is
        // how a pushback ENDS on this sim: each command is one complete
        // manoeuvre ("Nose 90 left" = straight segment, ~90° swing, stop,
        // RELEASE), and the Stansted trail proved it — err pinned at −98.5
        // for 30 m of straight push, a clean 90° arc, and the bar off with
        // nothing sent after the initial issue (2026-07-28). Treating it as
        // "tug let go early" failed a push that was executing perfectly.
        //
        // So the bar coming off ENDS the push, either way — the only question
        // is whether what the tug delivered is usable (`push_delivered`), and
        // there is no asking it back once it has let go.
        //
        // Gated on the bar having ever been SEEN on (`saw_attached`): on an
        // airframe where the attach flag reads false through a real push, the
        // bar "coming off" is a fact about the dataref, not the tug, and a
        // completed manoeuvre still ends through the stall path below —
        // "stopped after moving is completion" needs no flag to say so.
        if towed && !towing && !attached && saw_attached {
            let err_now =
                crate::angles::signed_diff_deg(plan.target_hdg, xp.get_float(PSI_TRUE).await.unwrap_or(0.0));
            let delivered = push_delivered(back_m, err_now, plan);
            event(json!({ "event": "tug_finished", "back_m": r1(back_m),
                          "err": r1(err_now), "delivered": delivered }));
            if delivered {
                break;
            }
            // Short, and the bar is OFF: that is the end of the push. There is
            // no summoning it back — `reissued_after_release` fired four times
            // across the whole blackbox and `rehooked` has never once fired
            // against it, four for four, each costing the full REHOOK_SECS
            // before the same give-up. The premise was that the tug "is
            // standing at the nose"; measured, the hook takes minutes (262 s
            // and 268 s at Malaga, 90 s at Luton) because it is a vehicle
            // driving across an airport, and a released one drives away.
            // Failing here immediately hands the fallback taxi four hundred
            // wasted seconds back.
            let _ = crate::tools::controls::drive_park_brake(ctx.xplane, true, valve_set).await;
            // Nothing left PENDING in the sim: the stall clock can issue a
            // second pattern in the very tick before this branch sees the
            // bar off (0.6 s apart at Malaga, 2026-08-03 and again
            // 2026-09-12), and a pushback request the tug has not yet
            // answered summons it back. Standing at the nose it re-hooks in
            // seconds, and X-Plane then owns the nosewheel: the fallback
            // taxi that followed the 2026-09-12 release could not turn
            // right at all and ran 25 m off the route. Stop is the only
            // word that cancels a request; the completion path sends it too.
            send(ctx, "stop").await;
            return Err("tug_released".into());
        }
        // Hooked on, let go WITHOUT pushing, and gone: summon it again, once,
        // with THE SAME pattern (a "straight" nudge once re-aimed a tug that
        // had been correctly told to swing the nose left). Never while it is
        // on the bar — that command is what makes it let go.
        if !towed && !attached && !nudged && began.elapsed().as_secs_f64() >= NUDGE_SECS {
            nudged = true;
            tracing::info!(
                pattern = cmd.action(),
                "pushback: tug left without pushing, asking again"
            );
            event(json!({ "event": "nudge", "pattern": cmd.action() }));
            send(ctx, cmd.action()).await;
        }
        let err = crate::angles::signed_diff_deg(plan.target_hdg, psi);
        last_psi = Some(psi);
        last_err = Some(err);
        // The ending rule, one judgement for the finish and the stall
        // clock: stopped on the lane mid-swing, never stalled mid-swing
        // (see `PushJudge`).
        let verdict = judge.step(
            began.elapsed().as_secs_f64(),
            back_m,
            err,
            attached,
            towed,
            plan,
            cmd == Steer::Straight,
        );
        event(json!({
            "event": "push",
            "back_m": r1(back_m),
            "err": r1(err),
            "steer": cmd.action(),
            "sense": sense,
            "att": attached,
            "tow": towing,
            "swinging": judge.swinging(),
            "rate": r1(judge.swing_rate()),
        }));
        // The tug's pattern is the sim's geometry: stop it before it takes
        // us off the pavement, whatever the plan still owed. The residual is
        // the taxi's, and a taxi can turn on pavement; on grass it could not
        // move at all (LIPK 2026-09-26).
        if towed && started_on_pavement == Some(true) {
            let (blat, blon) =
                nav::project_latlon(lat, lon, psi + 180.0, PAVEMENT_LOOK_M / M_PER_NM);
            let here = crate::party_line::on_pavement(lat, lon);
            let behind = crate::party_line::on_pavement(blat, blon);
            if at_pavement_edge(started_on_pavement, here, behind, judge.swinging()) {
                event(
                    json!({ "event": "pavement_edge", "back_m": r1(back_m), "err": r1(err),
                              "here": here, "behind": behind,
                              "swinging": judge.swinging(),
                              "rate": r1(judge.swing_rate()) }),
                );
                break;
            }
        }
        match verdict {
            PushVerdict::Complete => break,
            PushVerdict::Aligned => {
                let rate = judge.swing_rate();
                event(
                    json!({ "event": "aligned", "back_m": r1(back_m), "err": r1(err),
                              "rate": r1(rate), "rest": r1(predicted_rest_err(err, rate)) }),
                );
                break;
            }
            PushVerdict::SwingWait => {
                event(json!({ "event": "swing_wait", "back_m": r1(back_m), "err": r1(err) }));
            }
            PushVerdict::Pushing | PushVerdict::Stalled => {}
        }
        // The sense clock only runs while the aircraft is actually MOVING.
        // Standing still, a turn proves nothing: at Luton the ask went out at
        // 12:23:38 and the wheels did not turn until 12:24:52, so the window
        // closed seventy seconds into that dead air comparing 64.6 degrees
        // against an unchanged 64.6, called the sense good and latched it. The
        // push then swung the nose ninety degrees the WRONG way with no
        // recourse and ended 155 degrees off the lane (2026-07-28).
        if !towed {
            turn_since = (cmd != Steer::Straight).then_some((Instant::now(), err));
        }
        // Learn the turn sense from the turn that's been running: if it made
        // the error worse, the commands are the other way round on this build.
        // Acted on by re-issuing the OPPOSITE pattern, because there is no
        // per-tick steer to correct — the tug is already running the wrong one.
        // Judged EVERY tick, acted on once: a verdict of "fine" no longer ends
        // the watch, because a tug that reverses us straight back for twenty
        // seconds before it starts turning would spend the whole window
        // showing an unchanged error and buy the wrong sense a free pass.
        if !flipped {
            if let Some((since, err_at)) = turn_since {
                if sense_is_wrong(
                    towed,
                    cmd != Steer::Straight,
                    since.elapsed().as_secs_f64(),
                    err_at,
                    err,
                    sense,
                ) {
                    sense = -sense;
                    flipped = true;
                    LEARNED_SENSE.store(sense, SeqCst);
                    // Learn it, remember it for the session — and DO NOT
                    // re-issue the corrected pattern mid-push. A different
                    // pattern sent to a tug mid-manoeuvre makes it LET GO:
                    // both Luton pushes on 2026-07-28 show `sense_learned`
                    // followed by `tug_released` one second later, ~46 m into
                    // the plan. The tug is running a whole manoeuvre it will
                    // finish and stop; the learned sense serves the NEXT push
                    // (nothing is ever re-issued to a tug on the bar).
                    tracing::info!(
                        sense,
                        "pushback: turn sense inverted, correcting at the tug's next stop"
                    );
                    event(json!({ "event": "sense_learned", "sense": sense }));
                    turn_since = None;
                }
            }
        }
        // The tug finished. It runs the pattern it was given and stops, so a
        // stall AFTER movement is the end of that manoeuvre, not a hang — the
        // first live push sat out 203 more seconds to MAX_SECS with the
        // aircraft motionless, and the copilot then reported the whole stage
        // as "no tug came". Short of the plan, ask for one more push; the
        // count is bounded so a tug that can't finish can't loop us forever.
        if verdict == PushVerdict::Stalled {
            // The tug has run its pattern and stopped. It used to be asked
            // for another; a command to a tug ON THE BAR drops it off the bar
            // (five for five, 2026-09-12) and it comes back ninety seconds
            // later, so the push ends here: let go, and judge what it
            // delivered exactly as at a release. The residual is the taxi's.
            let delivered = push_delivered(back_m, err, plan);
            event(json!({ "event": "tug_finished", "back_m": r1(back_m),
                          "err": r1(err), "delivered": delivered }));
            if delivered {
                break;
            }
            send(ctx, "stop").await;
            let _ = crate::tools::controls::drive_park_brake(ctx.xplane, true, valve_set).await;
            return Err(push_residual(back_m, err, plan).into());
        }
        tokio::time::sleep(Duration::from_millis(TICK_MS)).await;
    }

    send(ctx, "stop").await;
    event(
        json!({ "event": "stopped", "back_m": r1(back_m), "err": last_err.map(r1),
                  "hdg": last_psi.map(r1), "swinging": judge.swinging() }),
    );
    // Wait for the tug to unhook and drive clear before any power goes on —
    // taxiing into your own tug is a way to end a flight on the stand. Watch
    // the BAR: gating this on tow_in_progress alone made it a no-op, because
    // that dataref is 0 for the whole push (see PUSH_ATTACHED), so the "wait"
    // returned true on its first read with the tug still hooked on.
    //
    // Held on the TOE brakes while it unhooks, the parking brake only once
    // it has: with the handle set the sim's tug does not let go. Nine pushes
    // in the logs set it first and waited out the whole 30 s with the tug on
    // the bar, and every one was released 5 to 10 s after the taxi drive let
    // the handle off (LIPK 2026-09-26 among them). The toe channel is an
    // expiring heartbeat, so it is written every poll.
    let cleared = wait_for_tug(xp, abort, last_psi).await;
    // Off the toe brakes we put on: the channel's override expires but the
    // ratios keep their last value on an airframe with no toe-brake axis,
    // and the taxi governor then read our 0.4 as the PILOT braking and
    // held the throttle at idle for 25 s (LIBP 737, 2026-09-26). A fresh
    // zero makes the plugin write the ratios back to zero.
    let _ = xp.set_float(TAXI_BRAKE, 0.0).await;
    if stopping(abort) {
        // Stopped while waiting for the tug to let go: the pilot's word, so
        // the parking brake, on the bar or not, and the line from what the
        // brake reads.
        abort_brakes(&xp.unscoped(), voice, valve_set, "tug_wait").await;
        event(json!({ "event": "abort", "back_m": r1(back_m), "stage": "tug_wait" }));
        return Err("abort".into());
    }
    if cleared {
        // Brakes SET now, and set they STAY until the taxi drive arms and its
        // governor releases them itself. The gap between the tug letting go
        // and the taxi engaging is exactly the window the stand creep
        // happened in.
        let _ = crate::tools::controls::drive_park_brake(ctx.xplane, true, valve_set).await;
        event(json!({ "event": "brakes_set", "tug_clear": true, "held_on": "park" }));
    } else {
        // Still on the bar: NEVER the parking brake against a tug (with the
        // handle set the sim's tug does not let go), and no last squeeze on
        // the toe channel either (the ratios would keep it, the LIBP lesson
        // above). The tug on the bar holds us, and the taxi drive's tug hold
        // (`tug_hold`) brakes on its governor from its first tick until the
        // bar comes off.
        event(json!({ "event": "brakes_set", "tug_clear": false, "held_on": "toe" }));
        tracing::warn!("pushback: the tug never showed clear");
        event(json!({ "event": "tug_not_clear" }));
        if let Some(v) = voice {
            v.alert(
                Priority::Advisory,
                "The tug's still on the bar, so I'm holding us on the brakes, not the \
                 parking brake. Watch for it before we roll."
                    .to_string(),
            );
        }
        // Said once: the taxi drive's own tug hold, a second later, stays
        // silent about the same tug.
        *TUG_WARNED.lock().unwrap_or_else(|e| e.into_inner()) = Some(Instant::now());
    }
    Ok(back_m)
}

/// The answer to the pilot's "stop" during a push, chosen from the parking
/// brake's READ-BACK: full auto's steered push used to hear "Brakes on, your
/// aircraft." from the command loop the moment the word landed, before the
/// push loop had set anything (#110, from #65). Shared with the native push's
/// stop in `main.rs`, so the two can never say different things.
pub fn stop_line(held: bool) -> &'static str {
    if held {
        "Stopping the push. Brakes on, your aircraft."
    } else {
        "Stopping the push. The parking brake did not set, hold the brakes. Your aircraft."
    }
}

/// A push that ends on a stop: the parking brake through the confirmed seam
/// (tried three times, read back). The line is the pilot's answer only when
/// the pilot's stop word ([`ABORT`]) ended it, taken once; a full auto abort
/// from elsewhere is spoken for by full auto. Blackbox `abort_brakes`.
///
/// Callers pass an UNSCOPED client: the stop has already cancelled the
/// mission's, and scoped the brake was refused three times as "automation
/// cancelled" (LEMG 737 push, 1790693535, 2026-09-29: "The parking brake did
/// not set, hold the brakes.", #110), the hand-back's bug of 2026-09-26.
async fn abort_brakes(
    xp: &dyn crate::xplane::XPlane,
    voice: &Option<Voice>,
    valve_set: Option<f64>,
    stage: &str,
) -> bool {
    let (held, line) = answer_stop(xp, valve_set, stage, ABORT.swap(false, SeqCst)).await;
    if let (Some(line), Some(v)) = (line, voice) {
        v.alert(Priority::Readback, line.to_string());
    }
    held
}

/// [`abort_brakes`] without the voice: whether the brake reads set, and the
/// line owed to the pilot (`None` when the stop was not theirs, `pilot`).
async fn answer_stop(
    xp: &dyn crate::xplane::XPlane,
    valve_set: Option<f64>,
    stage: &str,
    pilot: bool,
) -> (bool, Option<&'static str>) {
    let held = park_after_stop(xp, valve_set).await;
    event(json!({ "event": "abort_brakes", "stage": stage, "held": held, "pilot_stop": pilot }));
    (held, pilot.then(|| stop_line(held)))
}

/// A stopped push held: the one stop-and-park (`hand_back_hold`), so a push
/// still rolling is braked to a stop before the parking brake, the held
/// command's tail waited out, the brake read back. Both stops, the steered
/// push's and the native one's in `main.rs`, set the parking brake at once
/// at any speed (refactor audit 2026-10-04, duplicates #8). The tug has been
/// told to stop, so the brakes go on with it on the bar; and we are on the
/// ground, so an unread speed still parks (fail closed). `true` = it reads set.
pub(crate) async fn park_after_stop(
    xp: &dyn crate::xplane::XPlane,
    valve_set: Option<f64>,
) -> bool {
    let hold = crate::tools::controls::HoldFor::by("pushback")
        .on_the_bar()
        .fail_closed();
    crate::tools::controls::hand_back_hold(xp, valve_set, hold)
        .await
        .unwrap_or(false)
}

/// Every tug instruction goes through the SAME `pushback` tool the pilot's
/// own "push back left" reaches — one owner for the ground-ops commands, and
/// a spoken push and an automatic one can never drift apart. The tool's
/// readback string is the pilot's, not ours: the monitor narrates this stage.
async fn send(ctx: &ToolCtx<'_>, action: &str) {
    let _ = crate::tools::ground::Pushback
        .execute(&json!({ "action": action }), ctx)
        .await;
}

/// The tug wait itself: the toe brakes written every poll (an expiring
/// heartbeat), the bar, the dead tow flag and the heading read, and the
/// deadline [`TugClearWatch`]'s. `true` when the bar came off. The heading
/// is also held against the one at the stop (`hdg_at_stop`): a tug that
/// swings on regardless is logged `stop_ignored`, never told again.
async fn wait_for_tug(xp: &XPlaneClient, abort: &AtomicBool, hdg_at_stop: Option<f64>) -> bool {
    let t0 = Instant::now();
    let mut watch = TugClearWatch::new(0.0);
    let mut after = AfterStop::new(hdg_at_stop);
    let mut logged_at = -TUG_LOG_SECS;
    loop {
        if stopping(abort) {
            return false;
        }
        let _ = xp.set_float(TAXI_BRAKE, HOLD_BRAKE).await;
        let (att, tow, psi) = tokio::join!(
            xp.get_float(PUSH_ATTACHED),
            xp.get_float(TOW_IN_PROGRESS),
            xp.get_float(PSI_TRUE),
        );
        let t = t0.elapsed().as_secs_f64();
        let (att, tow, psi) = (att.ok(), tow.ok(), psi.ok());
        let verdict = watch.step(t, att, tow, psi);
        after.step(t, psi);
        if t - logged_at >= TUG_LOG_SECS || verdict != TugWait::Waiting {
            logged_at = t;
            let rec = json!({ "event": "tug_wait", "s": r1(t), "att": att, "tow": tow,
                              "hdg": psi.map(r1), "swinging": watch.swinging(),
                              "verdict": format!("{verdict:?}") });
            event(rec);
        }
        if verdict != TugWait::Waiting && after.ignored() {
            tracing::warn!(
                travel = after.travel_deg(),
                "pushback: the tug swung on after the stop"
            );
            event(
                json!({ "event": "stop_ignored", "travel_deg": r1(after.travel_deg()),
                          "hdg_at_stop": hdg_at_stop.map(r1), "hdg": psi.map(r1),
                          "moved_s": r1(after.moved_s()) }),
            );
        }
        match verdict {
            TugWait::Clear => return true,
            TugWait::TimedOut => return false,
            TugWait::Waiting => tokio::time::sleep(Duration::from_millis(TICK_MS)).await,
        }
    }
}

/// When the push last said the tug was still on the bar, so the taxi's tug
/// hold does not say it again (13:39Z 2026-09-26 would have heard it twice).
static TUG_WARNED: std::sync::Mutex<Option<Instant>> = std::sync::Mutex::new(None);

/// Did the push just say the tug is still on? Consumed, and only fresh
/// within a minute, so a later push's tug is always announced.
pub fn take_tug_warned() -> bool {
    TUG_WARNED
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .take()
        .is_some_and(|at| at.elapsed().as_secs_f64() < 60.0)
}

/// How often the tug wait leaves a trace (the 13:39Z wait left none, and its
/// story had to be read off the 1 Hz state records).
const TUG_LOG_SECS: f64 = 2.0;

/// Round to 0.1 for compact black-box records.
fn r1(v: f64) -> f64 {
    (v * 10.0).round() / 10.0
}

#[cfg(test)]
mod tests {
    #[test]
    fn the_push_stops_at_the_pavement_edge_only_from_pavement() {
        use super::at_pavement_edge;
        assert!(
            at_pavement_edge(Some(true), Some(true), Some(false), false),
            "edge behind"
        );
        assert!(
            at_pavement_edge(Some(true), Some(false), Some(false), false),
            "already over"
        );
        assert!(!at_pavement_edge(Some(true), Some(true), Some(true), false));
        assert!(
            !at_pavement_edge(Some(false), Some(false), Some(false), false),
            "a grass stand"
        );
        assert!(
            !at_pavement_edge(None, None, None, false),
            "no data is no verdict"
        );
        // Mid-swing only the aircraft itself leaving the pavement counts.
        assert!(
            !at_pavement_edge(Some(true), Some(true), Some(false), true),
            "a swing"
        );
        assert!(
            at_pavement_edge(Some(true), Some(false), Some(true), true),
            "over, swinging"
        );
    }

    /// LIBP stand 1's painted lane, as read from the install: back north
    /// from the stand, then north-west to the taxiway. The nose ends at 127
    /// (a turn LEFT), never the route's 217.
    #[test]
    fn a_painted_lane_plans_the_push() {
        let lane = [(42.431840, 14.188786), (42.431840, 14.188786), (42.431984, 14.188531)];
        let p = super::lane_plan(&lane).expect("a plan");
        assert!(p.lane);
        assert!((p.target_hdg - 127.4).abs() < 1.5, "{}", p.target_hdg);
        assert!((p.back_m - 26.3).abs() < 1.0, "{}", p.back_m);
        assert_eq!(super::steer_for(crate::angles::signed_diff_deg(p.target_hdg, 171.8), 1), super::Steer::Straight);
    }

    /// #190, LEMG (iMac 2026-09-29, sortie 1790712526, a Citation X at the
    /// gate, nose 327.7): the painted lane read from the install ends on
    /// 26.3 (`push_lane` target_hdg 26.3, back_m 98.6) while the cleared
    /// route leaves on 222.2 (`route_target_hdg`). The lane was flown: 58.6
    /// degrees, inside the straight band, and the tug pushed 51.5 m straight.
    /// Now the route's plan is flown, a swing onto 222; the LIBP lane, a
    /// quarter turn from its route, is still the lane's.
    #[test]
    fn a_lane_that_ends_facing_away_from_the_route_is_not_flown() {
        let lane = super::lane_plan(&[
            (36.67335699385795, -4.489441801082622),
            (36.672821, -4.4890121),
            (36.67281772148437, -4.4890096037109375),
            (36.672808571875, -4.489002910937501),
            (36.67279458007812, -4.4889932158203125),
            (36.672776775, -4.4889817125),
            (36.672756185546874, -4.4889695951171875),
            (36.672733840625, -4.4889580578125),
            (36.67271076914062, -4.488948294726563),
            (36.672688, -4.4889415),
            (36.67266665742188, -4.488938740039062),
            (36.672647346875, -4.4889396140625),
            (36.67263031914063, -4.488943115429687),
            (36.672615825, -4.4889482375),
            (36.67260411523438, -4.488953973632812),
            (36.672595440625, -4.4889593171875),
            (36.67259005195312, -4.488963261523438),
            (36.6725882, -4.4889648),
        ])
        .expect("the LEMG lane");
        assert!((lane.target_hdg - 26.3).abs() < 1.0, "{}", lane.target_hdg);
        let psi = 327.7;
        assert_eq!(super::steer_for(crate::angles::signed_diff_deg(lane.target_hdg, psi), 1), super::Steer::Straight);
        // The route as planned from the stand (replayed from the install):
        // the stand, 81.6 m back on 155.3 to the taxiway, then 222.2.
        let path = [
            (36.67335705897913, -4.489441674810065),
            (36.6726900, -4.4890590),
            (36.6726569, -4.4890964),
            (36.6726238, -4.4891338),
            (36.6725907, -4.4891712),
            (36.6725576, -4.4892086),
        ];
        let route = super::plan(36.67335705897913, -4.489441674810065, psi, &path).expect("nose in");
        assert!((route.target_hdg - 222.2).abs() < 0.5, "{}", route.target_hdg);
        let (p, used) = super::choose_plan(route.clone(), Some(lane));
        assert!(!used, "a lane ending 164 degrees from the route is not the way out");
        assert_eq!(p, route);
        assert!((p.back_m - 81.6).abs() < 1.0, "{}", p.back_m);
        let err = crate::angles::signed_diff_deg(p.target_hdg, psi);
        assert!((err + 105.5).abs() < 1.0, "{err}");
        assert_ne!(super::steer_for(err, 1), super::Steer::Straight, "the tug swings the nose");
        // LIBP stand 1: lane 127.4, route 217, the lane stands.
        let libp = super::lane_plan(&[(42.431840, 14.188786), (42.431840, 14.188786), (42.431984, 14.188531)])
            .expect("the LIBP lane");
        let libp_route = super::Plan { target_hdg: 217.0, ..libp.clone() };
        let libp_route = super::Plan { lane: false, ..libp_route };
        let (p, used) = super::choose_plan(libp_route, Some(libp.clone()));
        assert!(used);
        assert_eq!(p, libp);
        // No lane painted: the route's plan, as before.
        assert_eq!(super::choose_plan(route.clone(), None), (route, false));
    }

    #[test]
    fn parked_is_near_the_stand_whatever_the_heading() {
        // EDLP 7C: 2 m from the spot, 147 degrees across it. Parked; no tug.
        assert!(super::parked_at_stand(Some((2.0, 237.5)), true));
        assert!(!super::on_a_stand(Some((2.0, 237.5)), 91.0, true));
        assert!(!super::parked_at_stand(Some((140.0, 90.0)), true));
        assert!(!super::parked_at_stand(None, true));
        assert!(super::parked_at_stand(None, false));
    }

    #[test]
    fn a_tug_is_only_for_a_stand() {
        use super::on_a_stand;
        // On the stand, nose along its heading: push.
        assert!(on_a_stand(Some((12.0, 90.0)), 95.0, true));
        // Mid-taxiway, nearest stand 120 m off: taxi out instead.
        assert!(!on_a_stand(Some((120.0, 90.0)), 95.0, true));
        // Beside a stand but pointing out of it (already turned): no tug.
        assert!(!on_a_stand(Some((12.0, 90.0)), 260.0, true));
        // A field with no stand rows keeps the bearing-only rule.
        assert!(on_a_stand(None, 95.0, false));
        assert!(!on_a_stand(None, 95.0, true));
    }

    #[test]
    fn a_light_aircraft_turns_itself_off_the_stand() {
        use super::tug_for_weight;
        // SR22 (1,633 kg), SF50 (2,727 kg), Baron (2,495 kg): no tug.
        for kg in [1_633.0, 2_727.0, 2_495.0, 5_700.0] {
            assert!(!tug_for_weight(Some(kg)), "{kg}");
        }
        // Citation X, 737-800: the tug.
        assert!(tug_for_weight(Some(16_375.0)));
        assert!(tug_for_weight(Some(79_016.0)));
        // Unread or nonsense: no measurement, the tug as before.
        assert!(tug_for_weight(None));
        assert!(tug_for_weight(Some(0.0)));
        assert!(tug_for_weight(Some(f64::NAN)));
    }

    use super::*;

    /// Metres east/north of a reference point, as a lat/lon — a flat-earth
    /// approximation good to well under a metre at apron scale.
    fn at(lat0: f64, lon0: f64, east_m: f64, north_m: f64) -> (f64, f64) {
        let dlat = north_m / 111_320.0;
        let dlon = east_m / (111_320.0 * lat0.to_radians().cos());
        (lat0 + dlat, lon0 + dlon)
    }

    const LAT0: f64 = 51.15;
    const LON0: f64 = -0.16;

    /// A jet parked nose-in facing east, with the taxi lane 40 m behind it
    /// running north. The push is owed, it ends on the lane, and it ends
    /// pointing DOWN the lane — not merely reversed.
    #[test]
    fn a_nose_in_stand_earns_a_steered_push() {
        let me = (LAT0, LON0);
        let lane = at(LAT0, LON0, -40.0, 0.0); // 40 m west = behind us
        let path = vec![
            me,
            lane,
            at(LAT0, LON0, -40.0, 120.0), // the lane runs north from there
        ];
        let p = plan(me.0, me.1, 90.0, &path).expect("nose-in: a push is owed");
        assert!((p.back_m - 40.0).abs() < 2.0, "back {}", p.back_m);
        assert!(
            crate::angles::abs_diff_deg(p.target_hdg, 0.0) < 3.0,
            "finish pointing north up the lane, not 270: {}",
            p.target_hdg
        );
    }

    /// The same stand, but parked facing OUT: the way to the lane is straight
    /// ahead, so no tug is called. A GA tie-down must never earn a pushback
    /// just for being a parking spot.
    #[test]
    fn a_stand_you_can_taxi_out_of_is_left_alone() {
        let me = (LAT0, LON0);
        let path = vec![me, at(LAT0, LON0, -40.0, 0.0), at(LAT0, LON0, -40.0, 120.0)];
        // Pointing west — the lane is dead ahead.
        assert_eq!(plan(me.0, me.1, 270.0, &path), None);
        // Angled out of the spot: a taxi turn, not a push.
        assert_eq!(plan(me.0, me.1, 315.0, &path), None);
        // No route at all is nothing to plan against.
        assert_eq!(plan(me.0, me.1, 90.0, &[]), None);
        // A route whose points are all still on the stand: no way out to read.
        assert_eq!(
            plan(me.0, me.1, 90.0, &[me, at(LAT0, LON0, 2.0, 1.0)]),
            None
        );
    }

    /// The push distance is bounded at both ends — and a "way out" hundreds
    /// of metres off is not a long stand, it is a route that doesn't start
    /// here. A stale clearance (planned from runway 22C before a reposition
    /// to a gate 1 km away) read as nose-in and called a tug to push toward
    /// a lane a kilometre off (EGSS, 2026-07-29); the old clamp would have
    /// pushed 120 m toward it, which is a reverse taxi to nowhere.
    #[test]
    fn the_push_distance_stays_within_stand_scale() {
        let me = (LAT0, LON0);
        // A lane at the edge of stand scale still clamps the push distance.
        let long = vec![
            me,
            at(LAT0, LON0, -200.0, 0.0),
            at(LAT0, LON0, -200.0, 90.0),
        ];
        assert_eq!(plan(me.0, me.1, 90.0, &long).unwrap().back_m, MAX_BACK_M);
        // Past MAX_NODE_M the route was planned from somewhere else: refuse.
        let stale = vec![
            me,
            at(LAT0, LON0, -400.0, 0.0),
            at(LAT0, LON0, -400.0, 90.0),
        ];
        assert_eq!(plan(me.0, me.1, 90.0, &stale), None);
        let near = vec![me, at(LAT0, LON0, -11.0, 0.0), at(LAT0, LON0, -11.0, 90.0)];
        assert_eq!(plan(me.0, me.1, 90.0, &near).unwrap().back_m, MIN_BACK_M);
    }

    /// Which command swings the nose which way under tow is not something to
    /// assume: a sustained turn that makes the error WORSE flips the sense,
    /// and the same turn that helps leaves it alone.
    #[test]
    fn the_turn_sense_is_learned_not_assumed() {
        // Wanted 20 degrees right, and the nose has swung right around the
        // other way instead: that command turns the other way on this build.
        assert_eq!(learn_sense(1, 20.0, 60.0), -1);
        // The same turn working keeps the sense.
        assert_eq!(learn_sense(1, 20.0, 8.0), 1);
        // Inside the tolerance nothing is believed either way — a nose under
        // tow swings slowly, a straight push drifts a few degrees, and neither
        // is evidence of a reversed command.
        assert_eq!(learn_sense(1, 20.0, 22.0), 1);
        assert_eq!(learn_sense(1, 20.0, 20.0 + SENSE_TOL_DEG), 1);
        assert_eq!(learn_sense(-1, 20.0, 60.0), 1);
        // And the learned sense is what steers afterwards.
        assert_eq!(steer_for(80.0, 1), Steer::Right);
        assert_eq!(steer_for(80.0, -1), Steer::Left);
    }

    /// The lesson is only worth anything against a MOVING aircraft. Luton
    /// 2026-07-28: the tug was asked at 12:23:38, hooked on at 12:23:38 and did
    /// not roll until 12:24:52 — the four-second window expired in that dead
    /// air, compared 64.6 degrees against an unchanged 64.6, called the sense
    /// good and latched it. The push then swung the nose the wrong way for
    /// fifty-five metres and finished 155 degrees off the lane.
    /// The verdict is bookkeeping, not an interruption, and it needs real
    /// evidence. Luton 2026-07-28: a tug pushing dead straight back drifted
    /// three degrees over 39 metres, that was enough to convict the sense, and
    /// the corrective pattern sent mid-manoeuvre made the tug let go — the push
    /// stopped at 39 m of a 74 m plan in the same second.
    #[test]
    fn a_straight_push_that_drifts_is_not_a_reversed_command() {
        // The exact numbers from that push: 86.3 degrees off, 89.3 after.
        assert!(
            !sense_is_wrong(true, true, 90.0, 86.3, 89.3, 1),
            "three degrees of drift is noise, not a reversed tug"
        );
        // A genuinely reversed command swings the nose right around — the
        // 2026-07-27 push went from 64.6 to 154.8 degrees off.
        assert!(sense_is_wrong(true, true, 90.0, 64.6, 154.8, 1));
        // The threshold sits between the two, well clear of both.
        assert!(!sense_is_wrong(
            true,
            true,
            90.0,
            20.0,
            20.0 + SENSE_TOL_DEG,
            1
        ));
        assert!(sense_is_wrong(
            true,
            true,
            90.0,
            20.0,
            20.0 + SENSE_TOL_DEG + 1.0,
            1
        ));
    }

    #[test]
    fn the_sense_is_judged_against_a_moving_aircraft() {
        // Standing on the stand with the tug still driving over: a turn command
        // and all the time in the world prove exactly nothing.
        assert!(!sense_is_wrong(false, true, 70.0, 64.6, 64.6, 1));
        assert!(!sense_is_wrong(false, true, 70.0, 64.6, 154.8, 1));
        // Rolling, and the error unchanged: still no verdict — this is the tug
        // reversing straight back before it starts the swing.
        assert!(!sense_is_wrong(true, true, 10.0, 64.6, 64.6, 1));
        // Rolling, turning, and the error growing hard: that is the wrong
        // pattern.
        assert!(sense_is_wrong(true, true, CAL_SECS, 64.6, 100.0, 1));
        // ...but not before the turn has had time to show itself.
        assert!(!sense_is_wrong(true, true, CAL_SECS - 0.1, 64.6, 100.0, 1));
        // A straight push is never evidence about a turn.
        assert!(!sense_is_wrong(true, false, 30.0, 64.6, 154.8, 1));
        // A turn that's working is left alone, however long it runs.
        assert!(!sense_is_wrong(true, true, 60.0, 64.6, 20.0, 1));
    }

    /// A pattern is 90 degrees, so a plan wanting 135 cannot be finished by
    /// one — and the residual it leaves is a taxi turn, not a failure. The
    /// exact Malaga numbers, both sorties (2026-08-03): stand heading 041,
    /// lane 265.9, so 135.4 degrees owed; the tug pushed 55 m and swung the
    /// nose to 311, textbook, and both pushes were reported to the pilot as
    /// "the tug let go early".
    #[test]
    fn a_completed_ninety_is_a_delivered_push_not_a_failure() {
        let plan = Plan {
            back_m: 56.35,
            target_hdg: 265.9,
            node: (LAT0, LON0),
            lane: false,
        };
        // What the tug actually delivered, and what the loop asked next.
        assert!(
            !worth_another_pattern(53.9, -44.6, &plan),
            "another 90 from 44.6 degrees off overshoots 45 the other way"
        );
        assert!(
            push_delivered(53.9, -44.6, &plan),
            "55 m back and 45 degrees off is the taxi join's problem, not the tug's"
        );
        // It is NOT the planned finish — that is the whole point of the split.
        assert!(!done(53.9, -44.6, &plan));

        // Still owed half a swing: worth the ask, and not yet delivered.
        assert!(worth_another_pattern(53.9, -95.0, &plan));
        assert!(!push_delivered(53.9, -95.0, &plan));
        // Still owed real distance, however well aligned: a straight segment
        // earns its ask.
        assert!(worth_another_pattern(20.0, 2.0, &plan));
        assert!(!push_delivered(20.0, 2.0, &plan));
        // Past the plan and its chase margin: stop spending apron either way.
        assert!(!worth_another_pattern(90.0, -95.0, &plan));

        // A tug that let go after a metre is still a genuine failure — the
        // Southampton push (2026-08-01) must not be waved through as
        // "delivered" by the new leniency.
        assert!(!push_delivered(1.2, -130.0, &plan));
    }

    /// The airliner's ninety metres against a ten-metre threshold, and the
    /// label the failure wore. Both LFLL, 2026-09-17.
    #[test]
    fn a_long_push_is_judged_on_what_is_left_not_on_ten_flat_metres() {
        // Asked for 88.7 m, delivered 77.4 with the nose 1.1 degrees off the
        // lane. Anyone at the window watched that push finish; it failed by
        // 1.3 m, and full auto announced the tug had let us down.
        let long = Plan {
            back_m: 88.7,
            target_hdg: 85.1,
            node: (LAT0, LON0),
            lane: false,
        };
        assert!(
            push_delivered(77.4, 1.1, &long),
            "eleven metres behind a completed swing is the taxi drive's"
        );
        // The proportion is what makes it safe on a short push: a THIRD of
        // the way back is still in the stand, however straight the nose.
        let short = Plan {
            back_m: 56.35,
            target_hdg: 265.9,
            node: (LAT0, LON0),
            lane: false,
        };
        assert!(!push_delivered(20.0, 2.0, &short), "still in the stand");
        // And the other push that day: twelve metres PAST the plan with the
        // nose 71 degrees off. Not delivered, and not "short" either.
        let stand = Plan {
            back_m: 65.7,
            target_hdg: 157.2,
            node: (LAT0, LON0),
            lane: false,
        };
        assert!(
            !push_delivered(78.0, 71.3, &stand),
            "the turn never finished"
        );
        assert_eq!(push_residual(78.0, 71.3, &stand), "unturned");
        assert_eq!(push_residual(30.0, 2.0, &stand), "short");
    }

    /// The tug's dead air is survived by HOLDING, not by failing. Three EGHI
    /// pushes on 2026-08-02: hooked, brakes released, the Baron walked
    /// forward 3 m at idle in four seconds, and `creeping` killed a push the
    /// tug had simply not started yet — every time.
    #[test]
    fn creep_is_held_against_never_braked_into_a_push() {
        // Drifting forward with nobody pushing: hold.
        assert!(should_hold_creep(false, false, false, -0.5, false));
        // Inside the drift deadband: leave the brakes alone.
        assert!(!should_hold_creep(false, false, false, -0.2, false));
        // Moving BACK is the tug working, whatever the flags say.
        assert!(!should_hold_creep(false, false, false, 1.0, false));
        // A tug on the bar, mid-tow, or a push already under way is never
        // braked against — a "Nose 90" arc can carry the reference point
        // forward for a moment and must not meet a brake when it does.
        assert!(!should_hold_creep(false, false, true, -0.5, false));
        assert!(!should_hold_creep(false, true, false, -0.5, false));
        // After the push has moved us, the `towed` latch alone no longer
        // disarms the hold: a released tug plus a forward SLIDE off the
        // high-water mark is the same idle-thrust walk, 25 m further back —
        // the latch never clears, and it disarmed the hold for good after
        // the first 1.5 m of every push (audit, 2026-08-02).
        assert!(should_hold_creep(true, false, false, 24.0, true));
        // ...but a swing's plateau (no slide) is not creep, and the flags up
        // still win over everything.
        assert!(!should_hold_creep(true, false, false, 24.0, false));
        assert!(!should_hold_creep(true, true, false, 24.0, true));
        assert!(!should_hold_creep(true, false, true, 24.0, true));
        // The hold engages well before the creep FAILURE would: the backstop
        // stays a backstop.
        assert!(HOLD_AT_M < CREEP_M);
    }

    /// The deadband: inside it the tug pushes straight, because chasing the
    /// last few degrees costs apron the taxi's own steering owns.
    #[test]
    fn small_errors_push_straight() {
        assert_eq!(steer_for(0.0, 1), Steer::Straight);
        assert_eq!(steer_for(DEAD_DEG - 0.1, 1), Steer::Straight);
        assert_eq!(steer_for(-(DEAD_DEG - 0.1), 1), Steer::Straight);
        assert_eq!(steer_for(-46.0, 1), Steer::Straight, "LIBP: straight back, the taxi turns");
        assert_eq!(steer_for(-70.0, 1), Steer::Left);
    }

    /// The push ends when it's back far enough AND aligned — but it never
    /// reverses across the apron hunting the last twenty degrees.
    #[test]
    fn the_push_ends_aligned_or_it_ends_anyway() {
        let p = Plan {
            back_m: 40.0,
            target_hdg: 0.0,
            node: (LAT0, LON0),
            lane: false,
        };
        assert!(!done(20.0, 2.0, &p), "not back far enough yet");
        assert!(!done(41.0, 45.0, &p), "back far enough but pointing across");
        assert!(done(41.0, 5.0, &p), "back and aligned: let the tug go");
        assert!(
            done(40.0 + CHASE_M + 1.0, 90.0, &p),
            "past the chase limit the push STOPS, aligned or not"
        );
    }

    /// LXGB stand 1, 2026-09-15: the Nose 90 ran 45 m straight before it
    /// turned, so `done` fired on distance with the nose 140 degrees off
    /// and still swinging. A swing in progress outranks the distance test;
    /// a nose that has stopped moving does not.
    #[test]
    fn a_swing_in_progress_is_never_cut_short_by_the_distance_test() {
        let p = Plan {
            back_m: 40.1,
            target_hdg: 176.6,
            node: (LAT0, LON0),
            lane: false,
        };
        assert!(
            done(65.3, 139.6, &p),
            "past the chase margin by the numbers"
        );
        // Two seconds ago the error was 146.5: the tug is turning us.
        let mut w = SwingWatch::default();
        w.step(0.0, 146.5, true);
        assert!(w.step(2.0, 139.6, true));
        // The same swing with the bar OFF is not the tug's doing.
        assert!(!w.in_progress(false));
        // Error holding still: no swing, the distance test stands.
        let mut w = SwingWatch::default();
        w.step(0.0, 139.9, true);
        assert!(!w.step(2.0, 139.6, true));
        let mut j = PushJudge::new(0.0);
        assert_eq!(
            j.step(0.0, 65.3, 139.6, true, true, &p, false),
            PushVerdict::Complete
        );
    }

    /// LIBP, sortie 1790429680, 13:38:44Z 2026-09-26: a Nose 90 left, plan
    /// 55.3 m to 217.4. Every `push` tick (t from the live "stopped", back_m,
    /// err, the bar on throughout), as blackboxed.
    #[rustfmt::skip]
    const LIBP_PUSH: &[(f64, f64, f64)] = &[
        (-48.4, -0.0, 45.5), (-47.8, -0.0, 45.5), (-47.3, -0.0, 45.5), (-46.8, 0.1, 45.5),
        (-46.2, 0.4, 45.5), (-45.7, 0.9, 45.5), (-45.1, 1.6, 45.5), (-44.6, 2.5, 45.5),
        (-44.0, 3.4, 45.5), (-43.5, 4.5, 45.5), (-42.9, 5.8, 45.5), (-42.4, 7.0, 45.5),
        (-41.8, 8.2, 45.5), (-41.3, 9.4, 45.5), (-40.8, 10.6, 45.5), (-40.2, 11.8, 45.6),
        (-39.7, 13.1, 45.6), (-39.1, 14.4, 45.6), (-38.6, 15.6, 45.6), (-38.1, 16.8, 45.6),
        (-37.5, 18.0, 45.6), (-37.0, 19.3, 45.6), (-36.4, 20.4, 45.6), (-35.9, 21.7, 45.6),
        (-35.3, 22.9, 45.6), (-34.8, 24.2, 45.6), (-34.2, 25.4, 45.6), (-33.7, 26.3, 45.5),
        (-33.2, 26.9, 45.5), (-32.6, 27.4, 45.4), (-32.1, 27.9, 45.4), (-31.5, 28.6, 45.3),
        (-31.0, 29.2, 45.2), (-30.4, 29.9, 45.1), (-29.9, 30.6, 44.9), (-29.3, 31.4, 44.8),
        (-28.8, 32.1, 44.7), (-28.3, 32.9, 44.5), (-27.7, 33.7, 44.3), (-27.2, 34.6, 44.1),
        (-26.6, 35.4, 43.8), (-26.1, 36.2, 43.6), (-25.5, 37.1, 43.3), (-25.0, 37.9, 43.0),
        (-24.4, 38.8, 42.6), (-23.9, 39.7, 42.2), (-23.4, 40.5, 41.8), (-22.8, 41.3, 41.4),
        (-22.3, 41.9, 41.0), (-21.7, 42.6, 40.5), (-21.2, 43.2, 40.0), (-20.7, 43.8, 39.6),
        (-20.1, 44.4, 39.0), (-19.6, 45.0, 38.6), (-19.0, 45.5, 38.1), (-18.5, 46.0, 37.5),
        (-18.0, 46.5, 37.0), (-17.4, 47.0, 36.3), (-16.9, 47.5, 35.7), (-16.3, 47.9, 35.1),
        (-15.8, 48.4, 34.5), (-15.2, 48.7, 33.9), (-14.7, 49.1, 33.3), (-14.1, 49.6, 32.7),
        (-13.6, 50.0, 32.0), (-13.1, 50.4, 31.3), (-12.5, 50.8, 30.6), (-12.0, 51.3, 29.8),
        (-11.4, 51.7, 29.1), (-10.9, 52.2, 28.3), (-10.3, 52.7, 27.4), (-9.8, 53.2, 26.6),
        (-9.2, 53.7, 25.6), (-8.7, 54.4, 24.3), (-8.1, 55.1, 22.8), (-7.6, 55.7, 21.9),
        (-7.1, 56.0, 21.4), (-6.5, 56.4, 20.8), (-6.0, 56.8, 20.0), (-5.4, 57.2, 19.4),
        (-4.9, 57.6, 18.7), (-4.4, 57.9, 17.9), (-3.8, 58.4, 17.2), (-3.3, 58.7, 16.6),
        (-2.7, 59.1, 15.9), (-2.2, 59.5, 15.2), (-1.7, 59.8, 14.6), (-1.1, 60.2, 13.9),
        (-0.6, 60.5, 13.2), (0.0, 60.9, 12.6),
    ];
    /// The same push after the stop: true heading from the `state` records
    /// (t from the stop). The tug ignored "stop" and swung on to 262.
    #[rustfmt::skip]
    const LIBP_AFTER_STOP: &[(f64, f64)] = &[
        (1.0, 205.9), (2.2, 208.6), (3.4, 209.4), (4.8, 210.4), (6.0, 211.8),
        (7.2, 213.0), (8.6, 214.5), (9.9, 216.4), (11.3, 218.8), (12.6, 221.8),
        (14.0, 225.1), (15.3, 229.2), (16.5, 231.6), (17.8, 235.2), (19.2, 238.6),
        (20.5, 241.8), (21.8, 244.7), (23.1, 247.6), (24.4, 250.1), (25.7, 253.3),
        (26.9, 255.8), (28.2, 258.1), (29.5, 259.9), (30.9, 262.1), (32.2, 262.2),
        (33.5, 262.2), (34.6, 262.2), (35.9, 262.2), (37.2, 262.0),
    ];
    /// The bar off after the stop (`autotaxi` `tug_released`).
    const LIBP_RELEASED_S: f64 = 34.0;

    /// Replay a push's ticks through the judge, the bar on until `released`:
    /// how it ended, when, the error then and the judge's rate.
    fn replay(
        ticks: &[(f64, f64, f64)],
        plan: &Plan,
        released: f64,
    ) -> (&'static str, f64, f64, f64, bool) {
        let mut j = PushJudge::new(ticks[0].0);
        let mut towed = false;
        for &(t, back, err) in ticks {
            let attached = t < released;
            towed |= back > MOVED_M;
            if towed && !attached {
                return ("released", t, err, j.swing_rate(), j.swinging());
            }
            let how = match j.step(t, back, err, attached, towed, plan, false) {
                PushVerdict::Complete => "complete",
                PushVerdict::Aligned => "aligned",
                PushVerdict::Stalled => "stalled",
                _ => continue,
            };
            return (how, t, err, j.swing_rate(), j.swinging());
        }
        ("pushing", f64::NAN, f64::NAN, 0.0, false)
    }

    /// Issue #62. The recorded LIBP ticks through the old rule and the new.
    /// The old window (samples newer than two seconds, two degrees) read the
    /// 1.2 deg/s swing as over and said "stop" at err 12.6 (live t 0.0; the
    /// log's rounding trips it a few ticks earlier), a rate dip read as the
    /// end. The new judge rides the dip: the nose is still closing on the
    /// lane and would come to rest 11.7 off it, so the tug is left to bring
    /// it round and stopped ON the lane (err 2.9, +8.8 s, resting ~1.9 off);
    /// the headings that followed (the tug ignored the live stop and swung
    /// on to 262) are then read as `stop_ignored`, never answered. With a
    /// lane the swing never reaches, the stall clock never fires mid-swing:
    /// only once the nose has stood still.
    #[test]
    fn a_dip_in_the_swing_rate_never_stops_the_push() {
        let plan = Plan {
            back_m: 55.3,
            target_hdg: 217.4,
            node: (LAT0, LON0),
            lane: false,
        };
        // The continuation at the loop's 0.55 s tick, holding the last
        // recorded back_m (the aircraft crept at 0.1-0.2 kt: no progress).
        let heading_at = |t: f64| {
            LIBP_AFTER_STOP
                .iter()
                .rev()
                .find(|(ts, _)| *ts <= t)
                .map_or(204.8, |(_, h)| *h)
        };
        let ticks_for = |target: f64| {
            let mut ticks: Vec<(f64, f64, f64)> = LIBP_PUSH
                .iter()
                .map(|&(t, b, e)| (t, b, crate::angles::signed_diff_deg(target, plan.target_hdg - e)))
                .collect();
            let mut t = 0.55;
            while t <= 50.0 {
                ticks.push((t, 60.9, crate::angles::signed_diff_deg(target, heading_at(t))));
                t += 0.55;
            }
            ticks
        };
        let ticks = ticks_for(plan.target_hdg);

        // The old rule, as it ran.
        let old_stop = {
            let mut hist: std::collections::VecDeque<(f64, f64)> = Default::default();
            ticks.iter().find_map(|&(t, back, err)| {
                hist.push_back((t, err));
                while hist.front().is_some_and(|(ts, _)| t - ts > SWING_WINDOW_S) {
                    hist.pop_front();
                }
                let then = hist.front().map_or(err, |(_, e)| *e);
                let swinging = (err - then).abs() >= SWING_DEG;
                (done(back, err, &plan) && !swinging).then_some((t, err))
            })
        };
        let (t_old, err_old) = old_stop.expect("the old rule stopped the push");
        assert!(t_old <= 0.0 && err_old > 12.0, "{t_old} {err_old}");

        // The new rule: stopped on the lane, mid-swing, long before the
        // tug's own release at +34 s.
        let (how, at, err, rate, swinging) = replay(&ticks, &plan, LIBP_RELEASED_S);
        assert_eq!(how, "aligned", "at {at}");
        assert!(swinging);
        assert!((8.0..10.5).contains(&at), "{at}");
        assert!(err.abs() < 4.0, "{err}");
        let rest = predicted_rest_err(err, rate);
        assert!(rest.abs() <= ON_LANE_DEG, "{rest}");

        // What the tug did after a stop it ignored: the heading at the stop,
        // then every `state` heading after it.
        let mut after = AfterStop::new(Some(heading_at(at)));
        for &(ts, h) in LIBP_AFTER_STOP.iter().filter(|(ts, _)| *ts > at) {
            after.step(ts - at, Some(h));
        }
        assert!(after.ignored());
        assert!(after.travel_deg() > 40.0, "{}", after.travel_deg());
        // A failed read is no measurement.
        let mut blind = AfterStop::new(Some(heading_at(at)));
        blind.step(1.0, None);
        assert!(!blind.ignored());

        // A lane the swing never reaches (target 300): nothing stops the
        // push while the nose swings, the stall clock only once it has
        // stood still (262.1 at +30.9; the state records are 1.3 s apart,
        // so allow a sample).
        let far = Plan {
            target_hdg: 300.0,
            ..plan.clone()
        };
        let (how, at, _, _, swinging) = replay(&ticks_for(300.0), &far, f64::INFINITY);
        assert_eq!(how, "stalled", "{at}");
        assert!(!swinging);
        assert!(
            at > 30.9 + STALL_SECS && at < 30.9 + 1.3 + SWING_SETTLE_S + STALL_SECS,
            "{at}"
        );
    }

    /// LIPK, sortie 1790408535, 2026-09-26: a Nose 90 left, plan 83.5 m to
    /// 135.6. Every `push` tick from the swing's start (t from the live
    /// "stopped"), as blackboxed.
    #[rustfmt::skip]
    const LIPK_PUSH: &[(f64, f64, f64)] = &[
        (-39.7, 93.9, 109.1), (-39.2, 95.0, 109.1), (-38.6, 96.1, 109.1), (-38.1, 97.1, 109.1),
        (-37.5, 98.2, 109.1), (-37.0, 99.2, 109.1), (-36.4, 100.4, 109.0), (-35.9, 101.3, 108.1),
        (-35.4, 101.8, 107.2), (-34.8, 102.2, 106.7), (-34.3, 102.6, 106.3), (-33.7, 103.2, 105.9),
        (-33.2, 103.9, 105.5), (-32.6, 104.5, 105.0), (-32.1, 105.1, 104.5), (-31.5, 105.7, 103.9),
        (-31.0, 106.3, 103.3), (-30.4, 106.8, 102.7), (-29.9, 107.4, 102.0), (-29.4, 108.0, 100.9),
        (-28.8, 108.6, 99.9), (-28.3, 109.2, 98.7), (-27.7, 109.8, 97.3), (-27.2, 110.4, 95.6),
        (-26.7, 111.1, 93.9), (-26.1, 111.7, 91.9), (-25.6, 112.3, 89.8), (-25.0, 112.7, 88.0),
        (-24.5, 113.2, 86.2), (-24.0, 113.5, 84.5), (-23.4, 113.9, 82.9), (-22.9, 114.2, 81.4),
        (-22.3, 114.6, 79.6), (-21.8, 114.9, 77.8), (-21.2, 115.2, 76.1), (-20.7, 115.5, 74.3),
        (-20.1, 115.9, 72.5), (-19.6, 116.1, 70.6), (-19.0, 116.4, 68.7), (-18.5, 116.7, 66.9),
        (-18.0, 116.9, 64.9), (-17.4, 117.2, 63.0), (-16.9, 117.4, 61.0), (-16.3, 117.6, 59.0),
        (-15.8, 117.8, 56.7), (-15.2, 118.0, 53.5), (-14.7, 118.2, 50.3), (-14.2, 118.4, 47.0),
        (-13.6, 118.5, 43.6), (-13.1, 118.7, 40.2), (-12.5, 118.8, 36.7), (-12.0, 118.9, 33.2),
        (-11.4, 118.9, 29.9), (-10.9, 119.0, 27.0), (-10.4, 119.0, 24.6), (-9.8, 119.1, 22.2),
        (-9.3, 119.1, 19.9), (-8.7, 119.1, 17.6), (-8.2, 119.1, 15.5), (-7.6, 119.1, 13.5),
        (-7.1, 119.1, 11.7), (-6.6, 119.1, 10.0), (-6.0, 119.1, 8.2), (-5.5, 119.1, 6.6),
        (-4.9, 119.0, 4.8), (-4.4, 119.0, 3.0), (-3.8, 119.0, 1.1), (-3.3, 119.0, -0.7),
        (-2.7, 118.9, -2.5), (-2.2, 118.9, -4.3), (-1.7, 118.8, -6.2), (-1.1, 118.8, -8.1),
        (-0.6, 118.8, -9.9), (0.0, 118.7, -11.8),
    ];

    /// The LIPK tug swung the nose THROUGH the lane at 3.4 deg/s and the
    /// stall clock said "stop" 11.8 degrees past it; that stop was obeyed:
    /// the `state` heading moved 145.5, 148.3, 148.3 (2.8 degrees, still
    /// within 1.3 s), which is `STOP_LATENCY_S` and a final err near -14.6.
    /// The new judge stops it 4.4 s earlier, at err 3.0, led by the rate, so
    /// it comes to rest within `ON_LANE_DEG` of the lane; the obeyed stop's
    /// travel is not `stop_ignored`.
    #[test]
    fn a_swing_onto_the_lane_is_stopped_on_it() {
        let plan = Plan {
            back_m: 83.5,
            target_hdg: 135.6,
            node: (LAT0, LON0),
            lane: false,
        };
        let (how, at, err, rate, _) = replay(LIPK_PUSH, &plan, f64::INFINITY);
        assert_eq!(how, "aligned", "at {at}");
        assert!((at + 4.4).abs() < 0.05, "{at}");
        assert!((err - 3.0).abs() < 0.05, "{err}");
        assert!((-3.6..-2.8).contains(&rate), "{rate}");
        let rest = predicted_rest_err(err, rate);
        assert!(rest.abs() <= ON_LANE_DEG, "{rest}");
        // The live stop's own numbers, through the same prediction: -11.8 at
        // -3.4 deg/s rests at -14.5, what the state headings showed.
        assert!((predicted_rest_err(-11.8, -3.4) + 14.5).abs() < 0.1);

        // The obeyed stop, read after it: 2.8 degrees, not ignored.
        let mut after = AfterStop::new(Some(145.5));
        for (ts, h) in [(1.3, 148.3), (2.5, 148.3), (3.7, 148.7), (4.9, 148.2)] {
            after.step(ts, Some(h));
        }
        assert!(!after.ignored(), "{}", after.travel_deg());
    }

    /// A push that is really over still ends at once. The LIEA straight push
    /// (sortie 1790494983, 2026-09-27, plan 34.3 m, the nose steady at 0.4)
    /// stops on the tick it passed the plan, as it did live; a turned push
    /// that comes to rest short of the lane (but inside `ALIGN_DEG`) ends
    /// within `SWING_SETTLE_S` of the nose standing still, not at the 12 s
    /// stall; one swinging AWAY from the lane is stopped at once.
    #[test]
    fn a_finished_push_still_stops_promptly() {
        let plan = Plan {
            back_m: 34.3,
            target_hdg: 114.6,
            node: (LAT0, LON0),
            lane: false,
        };
        #[rustfmt::skip]
        let ticks = [
            (-6.5, 22.1, 0.4), (-6.0, 23.1, 0.4), (-5.5, 24.2, 0.4), (-4.9, 25.1, 0.4),
            (-4.4, 26.3, 0.4), (-3.8, 27.3, 0.4), (-3.3, 28.4, 0.4), (-2.7, 29.5, 0.4),
            (-2.2, 30.5, 0.4), (-1.6, 31.6, 0.4), (-1.1, 32.7, 0.4), (-0.5, 33.7, 0.4),
            (0.0, 34.8, 0.4),
        ];
        let mut j = PushJudge::new(-40.0);
        let end = ticks
            .iter()
            .find(|&&(t, back, err)| {
                j.step(t, back, err, true, true, &plan, true) == PushVerdict::Complete
            })
            .map(|(t, _, _)| *t);
        assert_eq!(end, Some(0.0));

        // A Nose 90 swinging 3 deg/s toward the lane, still at 5 off from
        // t = 10: never "on the lane", so it ends once the nose is still.
        let plan = Plan {
            back_m: 40.0,
            target_hdg: 0.0,
            node: (LAT0, LON0),
            lane: false,
        };
        let mut j = PushJudge::new(0.0);
        let mut t: f64 = 0.0;
        let end = loop {
            let err = (35.0 - 3.0 * t).max(5.0);
            let v = j.step(t, 45.0, err, true, true, &plan, false);
            if t < 10.0 {
                assert_ne!(v, PushVerdict::Complete, "mid-swing at {t}");
                assert_ne!(v, PushVerdict::Aligned, "short of the lane at {t}");
            }
            if v == PushVerdict::Complete {
                break t;
            }
            t += 0.5;
            assert!(t < 30.0);
        };
        assert!((10.0..=10.0 + SWING_SETTLE_S + 0.5).contains(&end), "{end}");

        // Swinging AWAY from the lane, inside the band: stopped at once.
        // The swing is under way before the distance is made (t = 3.5).
        let mut j = PushJudge::new(0.0);
        let mut t: f64 = 0.0;
        let end = loop {
            let v = j.step(t, 30.0 + 3.0 * t, 5.0 + 2.0 * t, true, true, &plan, false);
            assert_ne!(v, PushVerdict::Complete, "at {t}");
            if v == PushVerdict::Aligned {
                break t;
            }
            t += 0.5;
            assert!(t < 10.0);
        };
        assert_eq!(end, 3.5);
    }

    /// EGBB, sortie 1790534902, 2026-09-27: the 737's Nose 90, plan 49.6 m
    /// to 238.7. Every `push` tick (t from the live "stopped", back_m, err,
    /// the bar on throughout), as blackboxed.
    #[rustfmt::skip]
    const EGBB_PUSH: &[(f64, f64, f64)] = &[
        (-30.6, 0.0, -87.4), (-30.0, 0.0, -87.4), (-29.5, 0.0, -87.4), (-28.9, 0.0, -87.4),
        (-28.3, 0.1, -87.4), (-27.7, 0.4, -87.4), (-27.1, 0.8, -87.4), (-26.6, 1.4, -87.4),
        (-26.0, 2.1, -87.4), (-25.4, 2.9, -87.4), (-24.8, 4.0, -87.4), (-24.2, 5.0, -87.4),
        (-23.7, 6.1, -87.4), (-23.1, 7.3, -87.4), (-22.5, 8.5, -87.3), (-21.9, 9.6, -87.3),
        (-21.3, 10.8, -87.3), (-20.8, 11.9, -87.3), (-20.2, 13.0, -87.3), (-19.6, 14.2, -87.3),
        (-19.0, 15.3, -87.3), (-18.4, 16.5, -87.3), (-17.9, 17.7, -87.3), (-17.3, 18.9, -87.3),
        (-16.7, 20.1, -87.3), (-16.1, 21.1, -87.3), (-15.6, 22.3, -87.3), (-15.0, 23.5, -87.3),
        (-14.4, 24.5, -87.3), (-13.8, 25.5, -87.3), (-13.3, 26.4, -87.2), (-12.7, 26.9, -87.2),
        (-12.2, 27.3, -87.1), (-11.6, 27.8, -87.0), (-11.0, 28.4, -87.0), (-10.4, 29.0, -86.9),
        (-9.8, 29.5, -86.8), (-9.3, 30.1, -86.7), (-8.7, 30.7, -86.5), (-8.1, 31.4, -86.4),
        (-7.5, 32.1, -86.2), (-7.0, 32.7, -86.0), (-6.4, 33.4, -85.8), (-5.8, 34.2, -85.6),
        (-5.2, 34.9, -85.4), (-4.7, 35.6, -85.1), (-4.1, 36.3, -84.8), (-3.5, 37.0, -84.5),
        (-3.0, 37.7, -84.2), (-2.4, 38.5, -83.8), (-1.8, 39.2, -83.4), (-1.2, 39.9, -83.0),
        (-0.7, 40.5, -82.6), (-0.1, 41.2, -82.1),
    ];
    /// The same push after the stop: the `tug_wait` headings (s from the
    /// stop). The tug ignored "stop" and swung on to 235.4, the bar off at
    /// +59.2 (`stop_ignored`, 85.4 degrees).
    #[rustfmt::skip]
    const EGBB_AFTER_STOP: &[(f64, f64)] = &[
        (0.0, 320.8), (2.2, 316.5), (4.4, 313.5), (6.6, 311.0), (8.8, 308.9),
        (11.0, 307.3), (13.2, 305.7), (15.4, 303.8), (17.6, 301.7), (19.7, 299.1),
        (21.9, 296.2), (24.1, 292.9), (26.3, 289.0), (28.5, 284.8), (30.7, 280.4),
        (32.9, 275.1), (35.1, 271.1), (37.3, 267.1), (39.5, 263.1), (41.7, 258.9),
        (43.9, 254.7), (46.0, 250.8), (48.2, 247.0), (50.4, 243.2), (52.6, 239.9),
        (54.8, 236.7), (57.0, 235.6), (59.2, 235.4),
    ];

    /// Issue #125. The EGBB swing built from 0.1 to 0.8 deg/s over 13 s and
    /// the two-second test (1 deg/s) never saw it: `swinging` false at
    /// 41.2 m, so the edge 10 m behind stopped the push with the nose 82
    /// degrees off, a second into the swing. A swing that builds slowly is
    /// seen by 38.5 m (two degrees in four seconds), and mid-swing the
    /// look-behind no longer stops it: the push rides on to the lane or the
    /// aircraft actually leaving the pavement. After the stop the tug wait
    /// reads the same slow swing as moving (it read 0.7 to 0.95 deg/s as
    /// still for 9 s).
    #[test]
    fn a_swing_that_builds_slowly_is_seen_and_the_edge_behind_waits_for_it() {
        let plan = Plan {
            back_m: 49.6,
            target_hdg: 238.7,
            node: (LAT0, LON0),
            lane: false,
        };
        // The old start rule, as it ran: two degrees over two seconds only.
        let old_swinging = |upto: usize| {
            let (t, _, err) = EGBB_PUSH[upto];
            EGBB_PUSH[..upto]
                .iter()
                .rev()
                .find(|(ts, _, _)| t - ts >= SWING_WINDOW_S)
                .is_some_and(|(_, _, e)| crate::angles::abs_diff_deg(err, *e) >= SWING_DEG)
        };
        let last = EGBB_PUSH.len() - 1;
        assert!(!old_swinging(last), "the recorded failure");
        assert!(at_pavement_edge(Some(true), Some(true), Some(false), false));

        let mut j = PushJudge::new(EGBB_PUSH[0].0);
        let mut first_swing: Option<(f64, f64)> = None;
        for &(t, back, err) in EGBB_PUSH {
            let v = j.step(t, back, err, true, back > MOVED_M, &plan, false);
            assert_eq!(
                v,
                PushVerdict::Pushing,
                "at {t}: short of the plan and the lane"
            );
            if j.swinging() && first_swing.is_none() {
                first_swing = Some((t, back));
            }
        }
        let (t_swing, back_swing) = first_swing.expect("the swing is seen");
        assert!((-2.5..=-2.3).contains(&t_swing), "{t_swing}");
        assert!((back_swing - 38.5).abs() < 0.05, "{back_swing}");
        assert!(j.swinging(), "still swinging at the edge's tick");
        assert!((j.swing_rate() - 0.8).abs() < 0.1, "{}", j.swing_rate());
        // The edge behind, with the aircraft on the pavement: no stop.
        assert!(!at_pavement_edge(
            Some(true),
            Some(true),
            Some(false),
            j.swinging()
        ));

        // The tug wait over the headings after the stop: moving throughout
        // the swing, still from 57 s, the bar off at 59.2 is Clear.
        let mut w = TugClearWatch::new(0.0);
        for &(t, h) in EGBB_AFTER_STOP {
            let att = if t >= 59.2 { 0.0 } else { 1.0 };
            let v = w.step(t, Some(att), Some(0.0), Some(h));
            if (2.2..=54.8).contains(&t) {
                assert!(w.swinging(), "at {t}");
            }
            assert_eq!(v == TugWait::Clear, t >= 59.2, "at {t}");
        }
    }

    /// The Gatwick stand that convicted the per-tick steering (2026-07-27).
    /// The nose had to come 115 degrees LEFT to face down the lane, and the
    /// pattern picked from that error is the one command the tug gets — a
    /// whole manoeuvre, not a rudder input. The live failure was the loop
    /// picking this correctly and then overwriting it with `straight`.
    #[test]
    fn the_pattern_is_picked_from_the_error_and_is_a_whole_manoeuvre() {
        assert_eq!(steer_for(-114.8, 1), Steer::Left);
        assert_eq!(steer_for(114.8, 1), Steer::Right);
        // X-Plane offers exactly three patterns, so a 115 degree turn and a
        // 90 degree one both ask for the same thing: the tug swings the nose
        // ninety and the taxi join owns whatever is left.
        assert_eq!(steer_for(-90.0, 1), steer_for(-114.8, 1));
        // "straight" is a PATTERN too, not the absence of one — sending it to
        // re-aim a tug that was told to turn is what broke the live push.
        assert_eq!(Steer::Straight.action(), "straight");
        assert_eq!(Steer::Left.action(), "left");
        assert_eq!(Steer::Right.action(), "right");
    }

    /// The Southampton creep (2026-08-01). Progress along the push is a
    /// SIGNED quantity: the tug hooked on, never pushed, and the aircraft
    /// rolled forward off the stand at idle with the brakes off for the
    /// hook-up. Measured as a plain distance from the stand that forward roll
    /// read as pushback progress, latched "under tow", and the loop then
    /// waited out a re-hook that was never coming while the aircraft
    /// accelerated across the apron.
    #[test]
    fn rolling_forward_is_never_pushback_progress() {
        // Facing east (090). Behind us is west.
        let psi = 90.0;
        let back = at(LAT0, LON0, -10.0, 0.0); // 10 m west = 10 m back
        assert!((back_along(LAT0, LON0, psi, back.0, back.1) - 10.0).abs() < 0.2);

        // The live case: 1.5 m FORWARD, which the old magnitude read as 1.5 m
        // of push — enough to cross MOVED_M and latch the tow.
        let fwd = at(LAT0, LON0, 1.6, 0.0);
        let crept = back_along(LAT0, LON0, psi, fwd.0, fwd.1);
        assert!(crept < 0.0, "forward roll must be negative, got {crept}");
        assert!(crept < MOVED_M, "and must never latch the tow, got {crept}");

        // Sitting still is neither.
        assert!(back_along(LAT0, LON0, psi, LAT0, LON0).abs() < 0.01);

        // A "Nose 90" push swings the aircraft up to a right angle, so the
        // measurement is against the heading we STARTED on: 30 m back and 12 m
        // to one side is still ~30 m of push, whatever the nose is doing now.
        let arc = at(LAT0, LON0, -30.0, 12.0);
        let along = back_along(LAT0, LON0, psi, arc.0, arc.1);
        assert!((along - 30.0).abs() < 0.5, "arc measured {along}");

        // And the creep guard fires before the aircraft can get going.
        const { assert!(CREEP_M > MOVED_M) } // jitter must not trip the creep guard
    }

    /// 13:39Z 2026-09-26 (sortie 1790429680): "stopped" at 60.9 m with the
    /// nose at 204, the heading from the 1 Hz `state` records after it, and
    /// the bar off at +34.0 s (`autotaxi` `tug_released`). The old wait,
    /// counted from the stop, timed out at +30 s with the tug still swinging
    /// us and set the parking brake against it; counted from the swing's end
    /// the same ticks see the tug let go.
    #[test]
    fn the_tug_wait_runs_from_the_end_of_the_swing() {
        let hdg = LIBP_AFTER_STOP;
        const RELEASED_S: f64 = LIBP_RELEASED_S;
        let replay = |with_heading: bool| {
            let mut w = TugClearWatch::new(0.0);
            // The live wait polls every 0.5 s; the heading is the nearest
            // recorded sample at or before the poll.
            let mut t = 0.0;
            while t <= 40.0 {
                let h = hdg.iter().rev().find(|(ts, _)| *ts <= t).map(|(_, h)| *h);
                let att = if t >= RELEASED_S { 0.0 } else { 1.0 };
                let v = w.step(t, Some(att), Some(0.0), h.filter(|_| with_heading));
                if v != TugWait::Waiting {
                    return (v, t);
                }
                t += 0.5;
            }
            (TugWait::Waiting, t)
        };
        let (old, at) = replay(false);
        assert_eq!((old, at), (TugWait::TimedOut, 30.0), "the recorded failure");
        let (new, at) = replay(true);
        assert_eq!(new, TugWait::Clear);
        assert_eq!(at, RELEASED_S);
    }

    /// A bar that cannot be read is not off the bar, and a tug that sits
    /// still on it times out 30 s after it stopped moving us; one that keeps
    /// swinging is capped at the whole wait's two minutes.
    #[test]
    fn the_tug_wait_never_reads_a_failure_as_clear() {
        let mut w = TugClearWatch::new(0.0);
        assert_eq!(w.step(0.5, None, None, Some(90.0)), TugWait::Waiting);
        assert_eq!(w.step(29.5, None, Some(0.0), Some(90.0)), TugWait::Waiting);
        assert_eq!(w.step(30.0, None, Some(0.0), Some(90.0)), TugWait::TimedOut);
        // The dead tow flag blocks only when it actually reads on.
        let mut w = TugClearWatch::new(0.0);
        assert_eq!(w.step(1.0, Some(0.0), Some(1.0), None), TugWait::Waiting);
        assert_eq!(w.step(1.5, Some(0.0), None, None), TugWait::Clear);
        // Swinging forever: the cap.
        let mut w = TugClearWatch::new(0.0);
        let mut t = 0.0;
        let v = loop {
            let v = w.step(t, Some(1.0), Some(0.0), Some((t * 2.0) % 360.0));
            if v != TugWait::Waiting {
                break v;
            }
            t += 0.5;
        };
        assert_eq!((v, t), (TugWait::TimedOut, TUG_CLEAR_MAX_SECS));
    }

    /// Issue #110: the steered push's stop was answered "Brakes on, your
    /// aircraft." by the command loop before the push loop had set anything.
    /// The answer now comes from the brake's read-back: a brake the command
    /// sets is "Brakes on"; one that stays at 0.03 through three drives (the
    /// EGSS 737's read-back, #65) says it did not set; a stop that was not
    /// the pilot's (full auto's own abort) sets the brake and says nothing.
    #[tokio::test]
    async fn the_pilots_stop_is_answered_from_the_brake_read_back() {
        use crate::tools::PARK_BRAKE;
        use crate::xplane::FakeXPlane;
        const SET: &str = "sim/flight_controls/park_brake_set";
        let takes = FakeXPlane::with_reads(&[(PARK_BRAKE, 0.0)]);
        takes.step_on_command(SET, PARK_BRAKE, 1.0);
        let (held, line) = answer_stop(&takes, None, "pushing", true).await;
        assert!(held);
        assert_eq!(line, Some("Stopping the push. Brakes on, your aircraft."));

        let stuck = FakeXPlane::with_reads(&[(PARK_BRAKE, 0.03)]);
        let (held, line) = answer_stop(&stuck, None, "pushing", true).await;
        assert!(!held);
        let line = line.expect("the pilot is answered");
        assert!(
            line.contains("did not set") && !line.contains("Brakes on"),
            "{line}"
        );

        let takes = FakeXPlane::with_reads(&[(PARK_BRAKE, 0.0)]);
        takes.step_on_command(SET, PARK_BRAKE, 1.0);
        let (held, line) = answer_stop(&takes, None, "pushing", false).await;
        assert!(held, "the brake is set whoever stopped the push");
        assert_eq!(line, None);

        // The command loop no longer claims the brake for a steered push:
        // its stop lines are the native push's, chosen from its own
        // read-back through the same `stop_line`.
        let main = crate::test_support::module_source("main");
        assert!(
            !main.contains(r#""Stopping the push."#),
            "a literal stop line in main.rs"
        );
        assert!(main.contains("pushback::stop_line(held)"));
    }
}
