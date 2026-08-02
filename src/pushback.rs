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
/// Heading deadband — inside it the tug pushes straight. Chasing the last few
/// degrees costs metres of apron and the taxi's own steering owns them.
const DEAD_DEG: f64 = 6.0;
/// Close enough to "pointing down the lane" to let the tug go.
const ALIGN_DEG: f64 = 20.0;
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
/// Hooked on but not rolling this long after the ask: say it again, once.
const NUDGE_SECS: f64 = 12.0;
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
/// How many patterns we may ask for in one push. One is the plan; the rest
/// are continuations when the tug stopped short. Bounded so a tug that cannot
/// finish cannot keep us at the stand.
const MAX_ISSUES: u32 = 3;
/// Whole-push timeout: something is wrong (a tug pushing us nowhere, a stuck
/// dataref) and the sequence must not sit at the gate forever.
const MAX_SECS: f64 = 300.0;
/// How long to wait for the tug to unhook and clear before taxiing.
const TUG_CLEAR_SECS: f64 = 30.0;
/// How long to wait for the tug to take the bar AGAIN for a continuation
/// segment. It is standing at the nose, not driving across the airport —
/// the Stansted re-hook after a completed segment took seconds.
const REHOOK_SECS: f64 = 60.0;
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
    if angle_diff_deg(out_bearing, psi).abs() < BEHIND_DEG {
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
    })
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

/// Smallest signed angle a−b, degrees, in −180..180.
fn angle_diff_deg(a: f64, b: f64) -> f64 {
    (a - b + 540.0).rem_euclid(360.0) - 180.0
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
    let off = angle_diff_deg(brg, start_psi + 180.0).to_radians();
    d_m * off.cos()
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
    let out = run_inner(xp, ctx, voice, plan, abort, valve_set).await;
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
    let err0 = angle_diff_deg(plan.target_hdg, start_psi);
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
            if stopping(abort) {
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
        event(json!({ "event": "no_tug", "waited_s": TUG_WAIT_SECS }));
        return Err(if stopping(abort) { "abort" } else { "no_tug" }.into());
    }

    // The tug ignores a command issued before it was on the bar: the summon
    // above brought it, and nothing moved until the command was sent AGAIN
    // once hooked (live, the re-send was the twelve-second "nudge" and the
    // push began a minute later, 2026-07-27). So re-issue here, and treat the
    // pre-hook command as having done its one job — calling the truck.
    // Issued BEFORE the brakes come off: the aircraft is never brakeless with
    // a tug that has no instruction yet.
    send(ctx, cmd_now.action()).await;
    event(json!({ "event": "issued", "pattern": cmd_now.action(), "issue": 1 }));

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
    let mut cmd = cmd_now;
    let mut issues = 1u32;
    // When the pattern went on, and the error it started from — the pair the
    // sense is learned from.
    let mut turn_since: Option<(Instant, f64)> = (cmd != Steer::Straight).then_some((began, err0));
    let mut back_m = 0.0;
    // Last time we actually moved, and how far we'd got then: the tug runs its
    // chosen manoeuvre and STOPS, so "stopped after moving" is completion, not
    // a hang, and the loop must not sit out MAX_SECS waiting for more.
    let mut moved_at = Instant::now();
    let mut moved_m = 0.0;
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
    loop {
        if stopping(abort) {
            send(ctx, "stop").await;
            // Brakes on. The bar may be off and the engines are running, so an
            // abort that only stops the TUG leaves the aircraft rolling —
            // the same hole the re-hook wait had.
            let _ = crate::tools::controls::drive_park_brake(ctx.xplane, true, valve_set).await;
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
        let sliding = moved_at.elapsed().as_secs_f64() >= HOLD_STALL_SECS
            && back_m < moved_m - HOLD_REGRESS_M;
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
        // So: aligned enough, or out of asks — the push is complete. Short of
        // the plan with asks left — issue the next pattern, which SUMMONS the
        // tug back to the bar (it is standing at the nose), and wait for the
        // re-hook like the first hook, bounded much tighter.
        //
        // Gated on the bar having ever been SEEN on (`saw_attached`): on an
        // airframe where the attach flag reads false through a real push, the
        // bar "coming off" is a fact about the dataref, not the tug, and a
        // completed manoeuvre still ends through the stall path below —
        // "stopped after moving is completion" needs no flag to say so.
        if towed && !towing && !attached && saw_attached {
            let err_now =
                angle_diff_deg(plan.target_hdg, xp.get_float(PSI_TRUE).await.unwrap_or(0.0));
            let complete =
                done(back_m, err_now, plan) || (back_m >= MIN_BACK_M && err_now.abs() <= ALIGN_DEG);
            if complete || issues >= MAX_ISSUES {
                event(json!({ "event": "tug_finished", "back_m": r1(back_m),
                              "err": r1(err_now), "complete": complete }));
                if complete {
                    break;
                }
                let _ = crate::tools::controls::drive_park_brake(ctx.xplane, true, valve_set).await;
                return Err("tug_released".into());
            }
            cmd = steer_for(err_now, sense);
            issues += 1;
            event(
                json!({ "event": "reissued_after_release", "pattern": cmd.action(),
                          "issue": issues, "err": r1(err_now) }),
            );
            // BRAKES ON FIRST. The bar is off, so nothing is holding us — and
            // the engines are running, which is the whole of the rule at the
            // hook-up release above ("the first moment releasing them is safe
            // is when something is holding the aircraft"). Waiting out the
            // re-hook brakeless is the same hole the tug WAIT had before
            // 2026-07-27, one state later: at Southampton the tug let go after
            // 1.2 m, this wait ran its full minute, and the Baron rolled
            // FORWARD off the stand at idle thrust to 21 kt while the module
            // believed it was standing still — the tower berated us for being
            // off the cleared route and the pilot had to stop it by voice
            // (2026-08-01).
            let _ = crate::tools::controls::drive_park_brake(ctx.xplane, true, valve_set).await;
            send(ctx, cmd.action()).await;
            let rehooked = wait_for(REHOOK_SECS, abort, || async {
                let (a, t) =
                    tokio::join!(xp.get_float(PUSH_ATTACHED), xp.get_float(TOW_IN_PROGRESS));
                a.unwrap_or(0.0) > 0.5 || t.unwrap_or(0.0) > 0.5
            })
            .await;
            if !rehooked {
                send(ctx, "stop").await;
                // Already set above; re-assert, since the give-up path is the
                // one a fallback taxi picks up from.
                let _ = crate::tools::controls::drive_park_brake(ctx.xplane, true, valve_set).await;
                event(json!({ "event": "no_rehook", "back_m": r1(back_m) }));
                return Err("tug_released".into());
            }
            // On the bar again: same handshake as the first hook-up.
            let _ = crate::tools::controls::drive_park_brake(ctx.xplane, false, valve_set).await;
            event(json!({ "event": "rehooked" }));
            turn_since = (cmd != Steer::Straight).then_some((Instant::now(), err_now));
            moved_at = Instant::now();
            continue;
        }
        // On the bar but going nowhere: ask again, once. The first command
        // only summoned the truck. Re-issue THE SAME pattern — the nudge used
        // to send "straight" unconditionally, which re-aimed a tug that had
        // been correctly told to swing the nose left.
        if !towed && !nudged && began.elapsed().as_secs_f64() >= NUDGE_SECS {
            nudged = true;
            tracing::info!(
                pattern = cmd.action(),
                "pushback: hooked but not moving, asking again"
            );
            event(json!({ "event": "nudge", "pattern": cmd.action() }));
            send(ctx, cmd.action()).await;
        }
        let err = angle_diff_deg(plan.target_hdg, psi);
        event(json!({
            "event": "push",
            "back_m": r1(back_m),
            "err": r1(err),
            "steer": cmd.action(),
            "sense": sense,
            "att": attached,
            "tow": towing,
        }));
        if done(back_m, err, plan) {
            break;
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
                    // finish and stop; the stall-continuation below re-issues
                    // with the CORRECTED sense once it has stopped, which is
                    // the one moment a new pattern is known to take (the
                    // hook-up re-issue is the same moment).
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
        if back_m > moved_m + MOVED_M {
            moved_m = back_m;
            moved_at = Instant::now();
        } else if towed && moved_at.elapsed().as_secs_f64() >= STALL_SECS {
            if issues >= MAX_ISSUES || back_m >= plan.back_m {
                event(json!({ "event": "tug_finished", "back_m": r1(back_m), "err": r1(err) }));
                break;
            }
            cmd = steer_for(err, sense);
            issues += 1;
            event(json!({ "event": "issued", "pattern": cmd.action(), "issue": issues }));
            send(ctx, cmd.action()).await;
            turn_since = (cmd != Steer::Straight).then_some((Instant::now(), err));
            moved_at = Instant::now();
        }
        tokio::time::sleep(Duration::from_millis(TICK_MS)).await;
    }

    send(ctx, "stop").await;
    event(json!({ "event": "stopped", "back_m": r1(back_m) }));
    // Push complete: brakes SET while the tug unhooks — "parking brake set,
    // disconnect" — and set they STAY until the taxi drive arms and its
    // governor releases them itself. The gap between the tug letting go and
    // the taxi engaging is exactly the window the stand creep happened in.
    let _ = crate::tools::controls::drive_park_brake(ctx.xplane, true, valve_set).await;
    event(json!({ "event": "brakes_set" }));
    // Wait for the tug to unhook and drive clear before any power goes on —
    // taxiing into your own tug is a way to end a flight on the stand. Watch
    // the BAR: gating this on tow_in_progress alone made it a no-op, because
    // that dataref is 0 for the whole push (see PUSH_ATTACHED), so the "wait"
    // returned true on its first read with the tug still hooked on.
    if !wait_for(TUG_CLEAR_SECS, abort, || async {
        let (att, tow) = tokio::join!(xp.get_float(PUSH_ATTACHED), xp.get_float(TOW_IN_PROGRESS));
        att.unwrap_or(0.0) <= 0.5 && tow.unwrap_or(0.0) <= 0.5
    })
    .await
    {
        // It never cleared: say so, but the push itself happened.
        tracing::warn!("pushback: the tug never showed clear");
        event(json!({ "event": "tug_not_clear" }));
        if let Some(v) = voice {
            v.alert(
                Priority::Advisory,
                "The tug's still showing connected. Watch for it before we roll.".to_string(),
            );
        }
    }
    Ok(back_m)
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

/// Poll `cond` until it's true, the deadline passes, or the caller aborts.
async fn wait_for<F, Fut>(secs: f64, abort: &AtomicBool, mut cond: F) -> bool
where
    F: FnMut() -> Fut,
    Fut: std::future::Future<Output = bool>,
{
    let deadline = Instant::now() + Duration::from_secs_f64(secs);
    loop {
        if stopping(abort) {
            return false;
        }
        if cond().await {
            return true;
        }
        if Instant::now() >= deadline {
            return false;
        }
        tokio::time::sleep(Duration::from_millis(TICK_MS)).await;
    }
}

/// Round to 0.1 for compact black-box records.
fn r1(v: f64) -> f64 {
    (v * 10.0).round() / 10.0
}

#[cfg(test)]
mod tests {
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
            angle_diff_deg(p.target_hdg, 0.0).abs() < 3.0,
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
        assert_eq!(steer_for(20.0, 1), Steer::Right);
        assert_eq!(steer_for(20.0, -1), Steer::Left);
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
        assert_eq!(steer_for(-12.0, 1), Steer::Left);
    }

    /// The push ends when it's back far enough AND aligned — but it never
    /// reverses across the apron hunting the last twenty degrees.
    #[test]
    fn the_push_ends_aligned_or_it_ends_anyway() {
        let p = Plan {
            back_m: 40.0,
            target_hdg: 0.0,
            node: (LAT0, LON0),
        };
        assert!(!done(20.0, 2.0, &p), "not back far enough yet");
        assert!(!done(41.0, 45.0, &p), "back far enough but pointing across");
        assert!(done(41.0, 5.0, &p), "back and aligned: let the tug go");
        assert!(
            done(40.0 + CHASE_M + 1.0, 90.0, &p),
            "past the chase limit the push STOPS, aligned or not"
        );
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
}
