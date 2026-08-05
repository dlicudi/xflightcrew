//! Auto circuit — the copilot flies a full traffic pattern: takeoff, the four
//! pattern legs, and a landing back at the departure runway.
//!
//! This is an ORCHESTRATOR, not a new flight law. It stitches together the
//! three proven pieces so "do a circuit" is one command:
//!
//! 1. [`crate::autotakeoff`] flies the takeoff and climb-out, handing off with
//!    the autopilot engaged (servos + HDG + VS) at a thousand feet.
//! 2. This loop then flies the pattern on the autopilot's heading bug —
//!    crosswind, downwind, base — levelling at pattern altitude and starting
//!    the descent on base.
//! 3. [`crate::autoland::run_autoland`] takes the turn to final and lands.
//!
//! Trust model matches its parts: every actuation goes through the same AP the
//! copilot tools drive, and "my controls" / "cancel circuit" hands back at any
//! point. EXPERIMENTAL and untuned — the leg timings and altitudes are coarse
//! constants meant to be tuned from `logs/blackbox.jsonl` (`"kind":"circuit"`)
//! after in-sim flights, exactly like autoland. Fly it VMC, over the field,
//! with a hand ready.
//!
//! What it deliberately does NOT do: hold precise pattern spacing (the legs are
//! time-based, not abeam-the-numbers geometry), fly a right-hand pattern with
//! terrain awareness, or guarantee autoland re-selects the departure runway
//! (it scores by position and heading; turning final onto the departure course
//! from close in is what biases it back — see the handoff below).

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;

use crate::autoland::{along_track_nm, cross_track_nm, nearby_runways, true_to_mag};
use crate::autotakeoff;
use crate::tools::autopilot::ApConfig;
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

const LAT: &str = "sim/flightmodel/position/latitude";
const LON: &str = "sim/flightmodel/position/longitude";
const PSI_TRUE: &str = "sim/flightmodel/position/psi";
const HPATH: &str = "sim/flightmodel/position/hpath";
const AGL_M: &str = "sim/flightmodel/position/y_agl";
const ELEV_M: &str = "sim/flightmodel/position/elevation";
const MAG_VAR: &str = "sim/flightmodel/position/magnetic_variation";
// AP surface (mirror of autotakeoff.rs — the recipe PROVEN to climb and
// steer; `sim/autopilot/altitude_hold` is deliberately absent: it holds the
// CURRENT altitude, not the dial, and locked every pattern at handoff height
// (2026-07-24)).
const CMD_SERVOS_ON: &str = "sim/autopilot/servos_on";
const CMD_HDG_MODE: &str = "sim/autopilot/heading";
const CMD_VS_MODE: &str = "sim/autopilot/vertical_speed";
const VS_DIAL: &str = "sim/cockpit/autopilot/vertical_velocity";
const HDG_BUG: &str = "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot";
const HDG_STATUS: &str = "sim/cockpit2/autopilot/heading_status";
const VS_STATUS: &str = "sim/cockpit2/autopilot/vvi_status";
const IAS: &str = "sim/flightmodel/position/indicated_airspeed";
/// Gear handle — the circuit lowers it turning base (autoland's own
/// extension only fires inside 6 nm, which this hands over well inside of).
const GEAR_HANDLE: &str = "sim/cockpit2/controls/gear_handle_down";
/// The thrust lever THIS airframe obeys — not always the stock one (the
/// Citation X tracks `throttle_jet_rev_ratio`, so every write to
/// `throttle_ratio_all` went nowhere and it flew a whole pattern at takeoff
/// thrust). Resolved per call, so a profile swap takes effect immediately.
fn throttle_dref() -> String {
    crate::aircraft::throttle_dataref()
}

const M_TO_FT: f64 = 3.280_84;
const M_PER_NM: f64 = 1852.0;

// Pattern GEOMETRY, meters, in the departure-runway frame (along = the takeoff
// direction from the departure threshold; cross = right of that course). Timed
// legs looped the aircraft — the 172's autopilot turns ~3 deg/s, so a 90-degree
// turn takes ~30 s, longer than any sane dwell; every leg turning the same way
// with no straight segment spiralled it (LXGB, 2026-07-23). Position triggers
// don't care how slow a turn is: each leg flies until the aircraft REACHES
// the geometric point, so the pattern is a rectangle whatever the turn rate.
//
// And the geometry must be SCALED TO THE TURN RADIUS, not fixed: the AP banks
// gently (~14°), so at ~80 kt the radius is ~700-900 m — against the old fixed
// 900 m width, every leg trigger fired while the aircraft was still banked from
// the previous turn and the rectangle degenerated into continuous circles (the
// LXGB loops, 2026-07-24). All distances now derive from the radius, and legs
// advance EARLY by a turn-anticipation lead so the rollout lands ON the target
// line instead of overshooting it by a full radius.

/// The bank the AP actually flies in a heading capture, conservative
/// (shallower = wider radius = wider pattern).
///
/// 20° was tried — a standard-rate turn, which is what an autopilot nominally
/// does — and it broke the pattern: at EGHI the legs became short enough that
/// each trigger fired while the aircraft was still banked from the previous
/// turn, and the rectangle degenerated into a figure of eight (2026-07-30).
/// That is the exact failure this constant's conservatism exists to prevent,
/// and it is documented at the top of this file from the LXGB loops.
///
/// The lesson is that the number must match what the AP DOES, not what a
/// standard-rate turn implies — if it rolls into 14°, sizing the pattern for
/// 20° asks for corners it cannot make. Oversized is survivable; undersized
/// is not. The approach being too wide to intercept was a real problem, but
/// the fix for that is the handover gate (`interceptable`), not a tighter
/// pattern the aircraft cannot fly.
const AP_BANK_DEG: f64 = 14.0;
/// Max heading-bug movement per 500 ms tick, degrees — see [`slew_bug`].
const BUG_SLEW_PER_TICK: f64 = 40.0;

/// The bank the AP was MEASURED to fly, for the one dimension where being
/// conservative is the error rather than the safeguard.
///
/// `AP_BANK_DEG` above is deliberately shallow because everything it sizes —
/// pattern width, downwind length — is safe when it comes out too big and
/// broke the pattern into figure-eights when it came out too small. The
/// base-to-final capture is the exception: it is a distance the aircraft turns
/// IN, so over-estimating it starts the turn early and rolls out SHORT of the
/// centreline. Southampton, the Baron: the model wanted 1.3 x 1,443 = 1,876 m,
/// the aircraft actually turned in 1,048, and it rolled out 776 m to the side
/// (the arithmetic predicts 828 — see the test). Back-solved from that turn at
/// the real approach groundspeed the AP was banking 15.8 degrees, not 14.
/// Rounded a little shallow so the turn still errs EARLY, because with the
/// final leg's intercept an early roll-out is a gentle closure while a late
/// one is an S-turn across the centreline.
const AP_BANK_FINAL_DEG: f64 = 15.5;

/// Coordinated-turn radius at `gs_mps` and `bank_deg`, meters: v²/(g·tan φ).
/// Clamped to sane pattern bounds so a bad speed read can't produce a
/// degenerate geometry.
pub fn turn_radius_at(gs_mps: f64, bank_deg: f64) -> f64 {
    (gs_mps * gs_mps / (9.81 * bank_deg.to_radians().tan())).clamp(250.0, 2500.0)
}

/// Pattern turn radius — the conservative one, sizing every leg but the
/// base-to-final capture.
pub fn turn_radius_m(gs_mps: f64) -> f64 {
    turn_radius_at(gs_mps, AP_BANK_DEG)
}

/// The radius the BASE-TO-FINAL turn is actually flown at: the approach-leg
/// speed with no padding, at the measured bank.
///
/// Both of the pattern estimate's safety margins point the wrong way here. It
/// pads the speed by 1.1 (59.4 m/s against the 54.0 the aircraft flew) and it
/// banks shallow, and each one makes the radius BIGGER, which starts the turn
/// EARLIER, which is what left three quarters of a radius of offset for the
/// final leg to deal with.
pub fn final_turn_radius_m(approach_kts: f64) -> f64 {
    turn_radius_at(approach_kts.max(60.0) * 0.514_444, AP_BANK_FINAL_DEG)
}

/// The pattern's dimensions, all derived from one turn radius.
#[derive(Debug, Clone, Copy)]
pub struct Geometry {
    /// Climb past the departure end by this before turning crosswind.
    pub upwind_past_m: f64,
    /// Downwind spacing abeam the runway — how far out the pattern sits.
    pub width_m: f64,
    /// Fly the downwind until this far PAST the landing threshold (approach
    /// side), so the two 90° turns still leave a normal final.
    pub downwind_past_m: f64,
    /// Turn final once base is within this of the extended centreline — one
    /// radius, so the 90° turn rolls out ON the centreline.
    pub final_capture_m: f64,
    /// Turn anticipation: crosswind/downwind advance this EARLY, so the slow
    /// AP turn carries the aircraft TO the line, not a radius past it.
    pub lead_m: f64,
}

impl Geometry {
    /// One radius for the whole pattern — the shape tests use, and the right
    /// answer whenever the pattern and approach speeds are the same.
    pub fn for_radius(r: f64) -> Geometry {
        Geometry::for_radii(r, r)
    }

    /// `r` sizes the pattern; `final_r` sizes only the base-to-final capture,
    /// which is turned at the approach speed and is the one dimension where
    /// an over-estimate rolls out SHORT of the centreline instead of merely
    /// flying a wider box. See [`final_turn_radius_m`].
    pub fn for_radii(r: f64, final_r: f64) -> Geometry {
        Geometry {
            upwind_past_m: (0.5 * r).max(300.0),
            width_m: (2.2 * r).max(900.0),
            downwind_past_m: (2.0 * r + 1200.0).max(2000.0),
            // 1.3 radii, not one: the estimate is a still-air radius, and a
            // base leg ridden by a tailwind turns wider — one radius rolled
            // out south of the centreline (the overshoot, 2026-07-24).
            // Rolling out slightly EARLY is free; autoland tracks in.
            final_capture_m: (1.3 * final_r).max(250.0),
            lead_m: 0.9 * r,
        }
    }
}

/// The steepest cut autoland will fly to regain the centreline (its final
/// pursuit saturates here), and the margin we insist on over the bare
/// geometry before handing an approach to it.
const MAX_INTERCEPT_CUT_DEG: f64 = 35.0;
const INTERCEPT_MARGIN: f64 = 1.3;

/// Can an aircraft `xtk_m` off the centreline with `along_m` still to run
/// actually reach it, at the steepest cut autoland flies?
///
/// EGHI 2026-07-30: handed off 2,815 m abeam with 2,421 m to run — the offset
/// EXCEEDED the distance, a 49° angle to the runway. No cut closes that, so
/// autoland flew the whole final saturated and crossed the threshold 188 m
/// off. The circuit must fly its own turn onto final and only let go once the
/// geometry is actually flyable.
pub fn interceptable(xtk_m: f64, along_m: f64) -> bool {
    let need = xtk_m.abs() / MAX_INTERCEPT_CUT_DEG.to_radians().tan();
    along_m >= need * INTERCEPT_MARGIN
}

/// How near the runway course counts as "our turn has rolled out".
const ROLLED_OUT_HDG_DEG: f64 = 12.0;
/// Distance still to run below which we hand over regardless of alignment: a
/// turn that has gone wrong is still better flown by autoland's intercept than
/// by a pattern leg that has run out of room.
const LAST_DITCH_ALONG_M: f64 = 1500.0;
/// ...and how far off the centreline the valve will still accept. Twice the
/// established tolerance: enough that a turn which merely finished untidily
/// still gets handed over rather than flown into the ground by a pattern leg,
/// and tight enough that the 776 m diagonal never qualifies.
const LAST_DITCH_XTK_M: f64 = 2.0 * ESTABLISHED_XTK_M;

/// Is the aircraft ready to be handed to autoland?
///
/// Three conditions, and the middle one is the one that was missing.
/// `interceptable` asks whether autoland COULD close the offset; being able to
/// close it is not the same as being lined up. The circuit's own final leg
/// commands the runway heading and nothing else — it has no intercept — so the
/// turn onto final is the only thing that ever puts the aircraft on the
/// centreline, and releasing at the instant that turn is ARMED means it is
/// never flown at all.
///
/// EGHI, 2026-08-01, a Baron: base was crossing at 290° with 1,813 m still to
/// go to the centreline and 4,092 m to run. `interceptable` said yes (a 28°
/// cut), so autoland took it there and flew the diagonal in — the aircraft
/// approached and touched down at an angle instead of rolling out on the
/// centreline and flying a straight final. The turn was armed and then
/// immediately handed away.
///
/// So: wait until our own turn has actually rolled out onto the runway course
/// (by which point the geometry leaves ~0.3 radii to trim, which is what
/// autoland's final pursuit is for), unless we are running out of distance.
///
/// `hdg_err_deg` is the ground TRACK against the runway course, never the
/// nose: the final leg commands intercept-plus-crab, so on the centreline in
/// any crosswind the heading sits a whole wind-correction angle off the
/// course — 12 degrees at 16 kt across a 172's 74 kt approach — and a
/// heading-based gate can never pass while the aircraft tracks the
/// centreline perfectly. Every handover would then fall through the
/// last-ditch valve, which is the exact bypass this test exists to guard
/// (audit, 2026-08-02). The track is also the honest question: it is what
/// autoland inherits.
pub fn ready_for_autoland(xtk_m: f64, along_m: f64, hdg_err_deg: f64) -> bool {
    if !interceptable(xtk_m, along_m) {
        return false;
    }
    // The last-ditch valve: out of room, and autoland's intercept beats a
    // pattern leg with nowhere left to go. It bypasses the alignment test
    // below, so it must not become the normal way we hand over — it existed
    // because the final leg had no intercept of its own and genuinely could
    // not close an offset. It has one now, so the valve is for a turn that
    // went WRONG, not for one that simply hasn't finished: a big offset this
    // close in is exactly the angled arrival the test below exists to
    // prevent, and handing it over is not better than flying the intercept.
    if along_m.abs() <= LAST_DITCH_ALONG_M && xtk_m.abs() <= LAST_DITCH_XTK_M {
        return true;
    }
    // Rolled out on the runway HEADING is not established on the runway
    // CENTRELINE, and the difference is a landing at an angle. The 2026-07-30
    // fix added the heading test after a handover on base flew the diagonal
    // in; at Southampton the heading test passed — 8 degrees, comfortably
    // rolled out — with the aircraft 776 m to the side, flying parallel.
    // Autoland's pursuit then had to close 776 m in 1.7 nm on top of an
    // approach it was already high on, and the aircraft crossed the threshold
    // still 6 degrees off the runway heading, still turning (2026-08-01).
    // Established means BOTH, and the final leg's own intercept is what gets
    // us there (see `final_intercept_hdg`).
    hdg_err_deg.abs() <= ROLLED_OUT_HDG_DEG && xtk_m.abs() <= ESTABLISHED_XTK_M
}

/// How close to the centreline counts as ESTABLISHED, metres.
///
/// This is the 0.3 radii of trim the pattern geometry was always designed to
/// leave, and it is what autoland's final pursuit is for. What failed at
/// Southampton was not the existence of a residual but its SIZE: 776 m, most
/// of a radius, handed over on top of an approach already 206 ft high, and
/// autoland was still turning as it crossed the threshold. Half that, on a
/// path, is a trim; three quarters of a radius is a diagonal.
///
/// Not tighter, deliberately: the final leg's intercept has to be able to
/// actually REACH this before the handoff, and a gate the leg cannot satisfy
/// deadlocks into the last-ditch valve, which hands over the very thing this
/// exists to prevent.
const ESTABLISHED_XTK_M: f64 = 300.0;

/// The steepest cut the final leg will take toward the centreline. The
/// standard intercept angle for an instrument approach, and comfortably inside
/// autoland's own 35-degree pursuit limit, so a leg flown at the cap still
/// hands over something interceptable.
const MAX_FINAL_CUT_DEG: f64 = 30.0;

/// How far ahead the final leg aims when closing the centreline.
///
/// Sets how long the cut stays at the cap: the full 30 degrees holds until the
/// offset is inside `FINAL_AIM_M * tan(30)`, about 520 m, and fades from
/// there. A gentler aim reads better on paper and does not work — at 2,000 m
/// the cut had faded to 8 degrees by the time the offset was down to 300, and
/// the simulated intercept from the live roll-out only just made the
/// centreline before it ran out of runway. Hold the intercept like an
/// instrument approach does, then wash it out.
const FINAL_AIM_M: f64 = 900.0;

/// The heading the FINAL leg flies: the runway course, cut toward the
/// centreline in proportion to how far off it we are.
///
/// The final leg used to command the runway course and nothing else, which
/// makes it a leg that flies PARALLEL to the centreline forever. That was
/// survivable only while the turn onto final was assumed to roll out on the
/// line — and it does not. `final_capture_m` is 1.3 turn radii deliberately
/// ("rolling out slightly EARLY is free; autoland tracks in"), but the radius
/// model over-estimates what the autopilot actually flies: at Southampton the
/// turn began 1,850 m out and consumed only ~1,050 m, so it rolled out 776 m
/// short of the centreline and then held runway heading all the way to the
/// handoff, closing almost nothing (2026-08-01). "Slightly early" was three
/// quarters of a radius, and free it was not.
///
/// Pure pursuit at a point down the centreline, capped — the same shape the
/// taxi steering uses, for the same reason: it is a turn a real aeroplane can
/// fly, and it fades to zero as the offset closes instead of overshooting.
pub fn final_intercept_hdg(course_deg: f64, xtk_m: f64) -> f64 {
    let cut = (xtk_m.abs() / FINAL_AIM_M)
        .atan()
        .to_degrees()
        .min(MAX_FINAL_CUT_DEG);
    // Positive cross-track is right of the course, so close it by cutting
    // LEFT, and the other way about.
    (course_deg - xtk_m.signum() * cut).rem_euclid(360.0)
}

/// Floor for the base/final descent, feet above the field — the lowest the
/// pattern will put the aircraft before autoland takes it. It is a FLOOR now,
/// not a target: the leg descends the 3° path and stops here (see the vertical
/// law). Flown as a flat target it delivered a jet to the handoff 520 ft BELOW
/// the path it was then expected to fly down (Luton, 2026-07-28).
const HANDOFF_AGL_FT: f64 = 700.0;
/// How far above the approach speed the base and final legs are flown. Enough
/// margin to stay comfortably clean-configured, close enough that autoland has
/// only the last few knots to take off with flaps.
const APPROACH_LEG_MARGIN_KTS: f64 = 20.0;
/// Ceiling on the pattern speed, as a multiple of the approach speed.
const PATTERN_MAX_APPROACH_MULT: f64 = 1.5;
/// Altitude-capture gain: commanded fpm per foot of error.
const ALT_GAIN_FPM_PER_FT: f64 = 3.0;
/// ...and how far AHEAD the capture looks, seconds. A vertical speed does not
/// stop when the command does: an airliner rolling out of a 700 fpm climb
/// carries on for the best part of ten seconds. Pure proportional control on
/// altitude therefore commanded a climb right up to the target, sailed 100 ft
/// through it, commanded a descent, sailed back — the MD-82 hunted ±60 ft
/// around pattern altitude for the whole downwind with a period near forty
/// seconds (Luton, 2026-07-28). Feeding the CURRENT vertical speed forward
/// levels the aircraft where it will actually arrive.
const ALT_LEAD_S: f64 = 8.0;

/// How much descent the capture may command BEYOND the target's own rate.
/// For a target standing still this is the whole authority and reproduces the
/// law's original 500 fpm floor exactly; on a glidepath it is what closes an
/// error on top of simply keeping up.
const ALT_MAX_EXTRA_DESCENT_FPM: f64 = 500.0;

/// The rate a 3° path descends at, fpm, closing on the threshold at
/// `closure_kts` — the feed-forward for [`alt_capture_vs`].
///
/// 318 ft per nautical mile is a slope, not a rate: it becomes a rate only
/// when something is flying down it, and how fast is the aircraft's business.
/// Zero when nothing is closing (a base leg aims at a FIXED point, so its
/// target does not move and there is nothing to feed forward).
pub fn path_descent_fpm(closure_kts: f64) -> f64 {
    if closure_kts <= 0.0 {
        return 0.0;
    }
    -crate::autoland::GLIDEPATH_FT_PER_NM * closure_kts / 60.0
}

/// The commanded vertical speed for an altitude capture: the target's own
/// rate, plus proportional on the error, led by where the current vertical
/// speed is taking us.
///
/// `target_rate_fpm` is the feed-forward and it is the whole of the fix from
/// Malaga (2026-08-05). Without it this is a proportional controller chasing
/// a RAMP, and a proportional controller cannot track a ramp: it settles at a
/// constant lag of exactly ramp rate over gain. On final the target is the 3°
/// path walking down at 510 fpm as we close, the gain is 3 fpm per foot, and
/// 510/3 is 170 ft. The Baron was handed to autoland 174 ft high — the error
/// did not merely fail to close, it GREW from 54 ft at the roll-out to 172 ft
/// at the handoff, monotonically, exactly as the algebra says it must.
///
/// The old descent clamp was the same fault twice over: a flat -500 fpm is
/// LESS than a 3° path's own rate at any approach speed above 94 kt, so the
/// aircraft could not have stayed on the path even with zero error and
/// perfect control. It is now the target's rate plus a fixed correction
/// budget, which leaves a standing target's authority exactly where it was.
pub fn alt_capture_vs(target_ft: f64, target_rate_fpm: f64, alt_ft: f64, vs_fpm: f64) -> f64 {
    // The lead compares two FUTURES. Projecting the aircraft forward while
    // comparing it against where the target is NOW is the same mistake in a
    // second coat: in a steady descent it reads the aircraft as lower than it
    // is by rate times lead, so the law settles there — 77 ft high at 580 fpm
    // with an 8 s lead, on top of the ramp lag above. Projecting both leaves
    // a fixed target's behaviour bit-for-bit unchanged (its future is its
    // present) and lets a tracked path settle on zero error.
    let lead = ALT_LEAD_S / 60.0;
    let projected = alt_ft + vs_fpm * lead;
    let projected_target = target_ft + target_rate_fpm * lead;
    let correction = (projected_target - projected) * ALT_GAIN_FPM_PER_FT;
    (target_rate_fpm + correction).clamp(
        target_rate_fpm.min(0.0) - ALT_MAX_EXTRA_DESCENT_FPM,
        700.0,
    )
}

/// The altitude the base/final legs fly to, ft MSL: the 3° path at the
/// aircraft's TRACK distance from the threshold, bounded below by the handoff
/// floor and above by the pattern altitude.
///
/// Track distance (along the axis plus across it, the L the aircraft will
/// actually fly) rather than straight-line: on a wide base the two disagree
/// badly. A jet swinging round a 2.5 km turn radius is still moving AWAY down
/// the runway axis faster than it closes laterally, so the straight-line
/// distance GROWS through the first half of the turn — and a path computed
/// from it grows with it. Flown live that lifted the target 170 ft mid-base,
/// the commanded descent collapsed from 500 fpm to 145, and the aircraft
/// levelled off and sat there until the geometry came back round
/// (2026-07-28). Track miles are also simply the right measure: they are what
/// a 3° profile is flown against.
///
/// The pattern exists to deliver the aircraft to autoland, and autoland flies
/// that path from the handoff — so the base leg should arrive ON it. A flat
/// target made the handover height depend only on how wide the pattern was: the
/// 172 lands near the path by luck of its small one, while the MD-82's reaches
/// the centreline around 4 nm, where the path is 1,340 ft up and the old 700 ft
/// target left 520 ft to make up. That became a dive, twelve knots of extra
/// speed, and an overflight go-around (Luton, 2026-07-28).
///
/// Bounded above by the pattern altitude, so this can only ever DELAY a
/// descent, never command a climb on base.
/// The floor may not hold us ABOVE the path we are trying to join. A flat
/// 700 ft AGL did exactly that on a tight pattern: an SF50 turning final at
/// 1.84 nm meets a 3° path at 585 ft AGL, so the floor sat 115 ft over it and
/// even a perfectly flown base handed autoland an aircraft already high
/// (EGHI, 2026-07-30 — it then flew the whole final 400 ft high, crossed the
/// threshold 70 ft up and went around). Where the path is lower than the
/// floor, the path is the answer: it is converging on the runway, which is
/// the one place we are allowed to be low.
pub fn approach_leg_alt_ft(field_elev_ft: f64, tpa_msl_ft: f64, dist_m: f64) -> f64 {
    let dist_nm = dist_m.abs() / M_PER_NM;
    let path = field_elev_ft + dist_nm * crate::autoland::GLIDEPATH_FT_PER_NM;
    let floor = (field_elev_ft + HANDOFF_AGL_FT).min(path);
    path.clamp(floor, tpa_msl_ft.max(floor))
}

/// The distance a leg's glidepath target is computed against: how far the
/// aircraft still has to go DOWN THE APPROACH COURSE, which is not the same as
/// how far away it is.
///
/// This used to be track miles — along-track plus cross-track — and the
/// cross-track term is what wrecked the approach. A 3° path is flown against
/// distance-to-threshold along the FINAL APPROACH COURSE; adding the lateral
/// offset inflates the target by exactly however far off the centreline you
/// are, and on a base leg that is the whole leg. At Southampton the Baron
/// began base 3,324 m off the centreline, so track miles read 3.33 nm and
/// asked for 1,101 ft when the real distance to the threshold was 1.53 nm and
/// the path was at 529. The aircraft flew base and most of final level at
/// 800 ft AGL; the target only fell below it eleven seconds before the
/// handoff; autoland inherited an aircraft 206 ft high at 1.7 nm, flew the
/// whole final high, crossed the threshold 80 ft up at 102 knots and went
/// around (2026-08-01).
///
/// But plain along-track is wrong too, and wrong in the dangerous direction.
/// On base the aircraft is flying ACROSS the extended centreline, so its
/// along-track distance GROWS to a peak at the roll-out and only then starts
/// shrinking. Aiming at the path for the distance we have *now* means aiming
/// at a path for a point we are about to move further away from — at the same
/// stand that reads 529 ft with 3.3 km still to fly, which is a dive to
/// nowhere.
///
/// So base aims at the roll-out: the path altitude at the distance the turn
/// onto final will actually deliver us to, which the pattern geometry already
/// knows as `downwind_past_m` (predicted 4,087 m, flown 3,966 — close enough
/// to fly). Final then follows the real along-track distance down. That is
/// also simply how an approach works: platform until the path meets you, then
/// down the path.
pub fn path_dist_m(leg: Leg, a_m: f64, rollout_m: f64) -> f64 {
    match leg {
        // Never nearer than where the roll-out puts us.
        Leg::Base => a_m.abs().max(rollout_m),
        _ => a_m.abs(),
    }
}

/// Height to be down to by the base turn, AGL. The pattern used to hold TPA
/// until base and then try to lose the lot in one leg, which does not fit: at
/// EGHI the SF50 had to shed 600 ft in a 1.1 nm base (~900 fpm) and managed
/// 150. A real circuit starts down abeam the touchdown point, on the 180, and
/// this is that descent's target — low enough to give base a reachable job,
/// high enough to stay a sane downwind height.
const DOWNWIND_DESCENT_AGL_FT: f64 = 800.0;

/// The speed the pattern legs are flown at: the airframe's climb speed, but
/// never more than [`PATTERN_MAX_APPROACH_MULT`] times its approach speed.
///
/// Vy is not a pattern speed on a jet. The Citation X's published best-rate
/// figure is 300 KIAS — a cruise-climb number — and flown as a circuit speed it
/// would ask for a turn radius of about ten kilometres from geometry that
/// clamps at 2.5. The MD-82's 200 kt was the same mistake in a milder form.
/// A pattern at 1.5x the approach speed is what these aircraft actually fly,
/// and for every GA profile Vy already sits below that, so nothing changes.
pub fn pattern_speed_kts(vy_kts: f64, approach_ias: f64, vs_clean_kts: Option<f64>) -> f64 {
    let base = vy_kts
        .max(60.0)
        .min((approach_ias * PATTERN_MAX_APPROACH_MULT).max(60.0));
    // ...and never near the CLEAN stall, because the pattern is flown clean.
    // The Baron's Vy is 105 against an 84 kt clean stall — 1.25 Vs, which is
    // four knots above the airspeed caution band (1.2 Vs = 100.8) and so every
    // normal variation on downwind, base and final dipped into it. Six
    // callouts in one circuit, and every one of them was correct: the aeroplane
    // really was that close to the band. A flapped approach speed is not a
    // clean pattern speed, and nothing here knew the difference.
    let floor = vs_clean_kts.map_or(0.0, |v| v * PATTERN_MIN_STALL_MULT);
    base.max(floor)
}

/// The clean pattern is flown no slower than this multiple of the CLEAN stall.
/// 1.3 Vs is the usual approach minimum, and it is applied to the clean figure
/// here because the circuit never lowers flap before the handoff. Only bites on
/// an airframe whose Vy sits close to its clean stall — a 172 (Vy 74, stall 48)
/// and an MD-82 (Vy 200, stall 100) are both untouched.
const PATTERN_MIN_STALL_MULT: f64 = 1.3;

/// The speed the base and final legs are flown at. Vy is a CLIMB speed: an
/// MD-82 arrived on base at 226 KIAS against a 140 kt approach speed and handed
/// autoland an aircraft it could not slow down in the mile that was left
/// (Luton, 2026-07-28). Slow toward the approach speed, but never below a
/// margin over it — the circuit flies clean, and a jet at its flapped approach
/// speed with the flaps still up is not slow, it is near the stall. And never
/// FASTER than the pattern speed, so a GA airframe (whose Vy already sits below
/// approach + margin) keeps exactly what it flew before.
pub fn approach_leg_kts(pattern_kts: f64, approach_ias: f64) -> f64 {
    pattern_kts
        .min(approach_ias + APPROACH_LEG_MARGIN_KTS)
        .max(approach_ias)
}

/// How far ahead the climb limiter judges the airspeed, seconds. Long enough
/// to lead the airframe's own response, short enough not to invent a speed:
/// the same order as the altitude capture's lead.
const CLIMB_SPEED_LEAD_S: f64 = 5.0;

/// The slowest the pattern will climb toward: a floor under the speed
/// protection below. Blue line where the airframe has one, else a margin over
/// the clean stall.
///
/// A twin's Vyse is the honest number here and it is not a comfort figure —
/// below blue line a light twin cannot climb on one engine at all, and the
/// margin to Vmca is what is left of the pilot's options.
pub fn climb_floor_kts(vyse_kts: Option<f64>, vs_clean_kts: Option<f64>) -> f64 {
    let from_stall = vs_clean_kts.map(|v| v * 1.15);
    match (vyse_kts, from_stall) {
        (Some(a), Some(b)) => a.max(b),
        (Some(a), None) => a,
        (None, Some(b)) => b,
        (None, None) => 0.0,
    }
}

/// How much of a commanded CLIMB the airspeed will actually support. Scales
/// the rate to zero as the speed falls from the pattern speed to the floor.
///
/// The autopilot's vertical mode buys rate with airspeed, and it will keep
/// buying: a Baron 58 rolled into the crosswind turn at 127 kt and came out at
/// 85 — one knot over its clean stall and one over Vmca — with the VS law
/// still asking for 880 fpm and the stabilizer trimmed to the nose-up stop.
/// Six airspeed callouts fired across one Southampton circuit and every one of
/// them was right (2026-08-01). The throttle law alone cannot fix this: it is
/// already commanding more power, and in a climbing turn on a piston twin
/// there is none left to give. Lowering the nose is the other half, and it is
/// the half a pilot would do first.
///
/// Only ever REDUCES a climb — descents (negative) pass through untouched, and
/// an aircraft at or above its full-rate speed (see [`climb_full_rate_kts`])
/// keeps the rate it asked for, so every airframe with power to spare is
/// unchanged.
pub fn climb_limit_fpm(
    vs_cmd: f64,
    ias: f64,
    ias_trend_kts_per_s: f64,
    want_kts: f64,
    floor_kts: f64,
) -> f64 {
    if vs_cmd <= 0.0 || !ias.is_finite() || floor_kts <= 0.0 {
        return vs_cmd;
    }
    // Judge the speed we are HEADING FOR, not the one on the dial. Reacting to
    // the instantaneous reading made this a bang-bang gate and it limit-cycled
    // live: the commanded rate went 700 to 0 in four seconds as the speed dipped
    // through the band, HELD there while the aircraft was already accelerating
    // back, then slammed open again — the aeroplane porpoising about thirty feet
    // either side with a period near twenty-five seconds (2026-08-01, felt in the
    // cockpit before it was seen in the log). The projection fixes both ends: it
    // starts easing the climb off before the speed is lost, so the dip is
    // shallower, and it gives the rate back as soon as the speed is genuinely
    // recovering instead of waiting for it to arrive. Same lead trick
    // `alt_capture_vs` uses on altitude and the stall callout uses on airspeed.
    let ias = ias + ias_trend_kts_per_s * CLIMB_SPEED_LEAD_S;
    // Full rate AT the pattern speed — that is the speed the airframe is meant
    // to be climbing at, and an aircraft flying it correctly must never be
    // penalised. The fade runs downward from there to the floor.
    if ias >= want_kts {
        return vs_cmd;
    }
    if ias <= floor_kts {
        return 0.0;
    }
    // A pattern speed at or below the floor leaves nothing to fade across
    // (the two checks above have already covered it).
    vs_cmd * ((ias - floor_kts) / (want_kts - floor_kts))
}

/// The speed at which [`climb_limit_fpm`] grants the FULL commanded rate: the
/// speed the climb is actually FLOWN at, never the leg target the throttle
/// holds.
///
/// Those were the same number until the clean-stall floor raised the Baron's
/// pattern speed to 109.2 kt — four knots ABOVE its Vy of 105 — and the two
/// changes shipped the same evening without meeting. The throttle law holds
/// the leg speed, so wiring that same speed in as the limiter's full-rate
/// threshold parks the aircraft at the exact top of a nine-knot fade band:
/// full climb is unreachable WHILE climbing (climbing at Vy is what the
/// airframe does), and every knot of normal jitter swings the granted rate
/// across half its range. Live it porpoised exactly as the projection was
/// supposed to prevent — cmd_vs 700 to 0 on crosswind with the Baron
/// DESCENDING at 190 fpm at 670 ft AGL mid-climb-out, then the whole downwind
/// hung 100 ft below pattern altitude with the capture choked to 0-150 fpm
/// and the speed see-sawing 104 to 113 on a 25 s cycle (EGHI, 2026-08-02).
/// The unit test had pinned Vy as `want` all along; the call site passed the
/// pattern speed, and the four-knot difference was the whole failure.
///
/// Capped below the leg speed (a jet's book Vy is a cruise-climb figure far
/// above its pattern speed, and must not RAISE the threshold), and held a
/// margin above the floor so a profile whose Vy sits at or under the floor
/// still leaves a band to fade across instead of a cliff.
pub fn climb_full_rate_kts(leg_kts: f64, vy_kts: f64, floor_kts: f64) -> f64 {
    leg_kts.min(vy_kts).max(floor_kts + 2.0)
}

/// An airborne AGL below this after the takeoff loop exits means it REJECTED
/// (never flew) — the circuit is off, not into the pattern.
const AIRBORNE_MIN_AGL_FT: f64 = 300.0;
/// Terrain clearance a pattern side must have at TPA to be flyable; below it
/// the circuit switches sides, and with BOTH sides fouled it raises the TPA.
const TERRAIN_CLEAR_FT: f64 = 600.0;

/// True while the circuit orchestrator runs.
pub static ACTIVE: AtomicBool = AtomicBool::new(false);
/// Set true to cancel ("cancel circuit" / "my controls").
pub static ABORT: AtomicBool = AtomicBool::new(false);

/// One-line status for the dashboard's "managing" panel.
pub fn status() -> Option<String> {
    ACTIVE
        .load(Ordering::Relaxed)
        .then(|| "auto circuit".to_string())
}

/// Radio calls the copilot makes on our behalf during the circuit — the ONE
/// place the crew talks to the world. The circuit thread (which doesn't own the
/// station) pushes a position report here; the main loop (which does) drains it
/// each tick, routes it through the tuned tower like a keyed pilot call, and
/// speaks both the copilot's transmission and the tower's reply. Empty in
/// copilot-only mode or with no station — the calls are simply discarded, since
/// real/external ATC owns the radio then.
static CREW_RADIO: Mutex<VecDeque<String>> = Mutex::new(VecDeque::new());

/// Drain the pending crew radio calls (main loop only).
pub fn take_crew_radio() -> Vec<String> {
    let mut q = CREW_RADIO.lock().unwrap_or_else(|e| e.into_inner());
    q.drain(..).collect()
}

/// Queue a radio call for the main loop to transmit. Public so the full-auto
/// orchestrator can request taxi through the same channel.
pub fn transmit(call: String) {
    CREW_RADIO
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .push_back(call);
}

/// The aircraft's callsign, spoken the way the tower says it ("November one
/// seven two Sierra Papa"), so the copilot's calls sound like a real pilot's.
/// `None` if the tail number can't be read.
pub async fn crew_callsign(xp: &XPlaneClient) -> Option<String> {
    xp.get_string(crate::datarefs::TAILNUM)
        .await
        .ok()
        .map(|t| crate::tower::spoken_callsign(&t))
        .filter(|s| !s.is_empty())
}

/// Format a radio call with the callsign in front, pilot-style: "November one
/// seven two Sierra Papa, downwind runway two seven". No callsign → bare.
pub fn radio_call(callsign: &Option<String>, msg: &str) -> String {
    match callsign {
        Some(cs) => format!("{cs}, {msg}"),
        None => msg.to_string(),
    }
}

/// Spell a runway identifier for the radio: "27" -> "two seven", "09L" -> "zero
/// nine left". Digit by digit, with the L/R/C suffix spoken.
pub fn spell_runway(id: &str) -> String {
    const DIGITS: [&str; 10] = [
        "zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine",
    ];
    let mut out: Vec<String> = Vec::new();
    for c in id.chars() {
        match c {
            '0'..='9' => out.push(DIGITS[c as usize - '0' as usize].to_string()),
            'L' | 'l' => out.push("left".to_string()),
            'R' | 'r' => out.push("right".to_string()),
            'C' | 'c' => out.push("center".to_string()),
            _ => {}
        }
    }
    out.join(" ")
}

/// Cancel an active circuit and stand down whichever sub-loop is flying;
/// returns whether one was running.
pub fn abort() -> bool {
    ABORT.store(true, Ordering::SeqCst);
    // Stand down whatever phase is live. The takeoff hands off silently (power
    // stays set); the procedure/autoland arrival aborts to the pilot.
    autotakeoff::ACTIVE.store(false, Ordering::SeqCst);
    crate::procedure::abort();
    crate::autoland::abort();
    ACTIVE.swap(false, Ordering::SeqCst)
}

/// Everything the orchestrator needs, resolved by the arming tool.
pub struct Params {
    pub apt_dat: String,
    /// Nav-data paths for the published-approach arrival (the terrain-safe
    /// alternative to a naive base-to-final): CIFP procedure dir, `earth_fix.dat`,
    /// `earth_nav.dat`. Empty-string-safe — a missing file just means the
    /// approach isn't found and the arrival falls back to autoland.
    pub cifp_dir: String,
    pub fix_data_path: String,
    pub nav_data_path: String,
    /// OpenAir airspace file — the pattern's restricted/prohibited/danger
    /// avoidance. Empty/missing file = no airspace data, no avoidance.
    pub airspace_path: String,
    pub cfg: ApConfig,
    /// A fully-built takeoff config — the circuit arms the same takeoff loop.
    pub takeoff: autotakeoff::Params,
    /// Approach speed handed to autoland (KIAS).
    pub approach_ias: f64,
    /// Pattern altitude above the field, feet.
    pub tpa_agl_ft: f64,
    /// Standard left-hand pattern unless the pilot asked for right.
    pub left_pattern: bool,
    /// Clean stall speed, KIAS — the floor under the pattern speed, since the
    /// circuit is flown clean. `None` leaves the speed as Vy alone.
    pub vs_clean_kts: Option<f64>,
    /// The slowest the pattern may climb toward, KIAS. See
    /// [`climb_limit_fpm`]; 0 disables the protection.
    pub climb_floor_kts: f64,
}

/// The four legs after the departure runway; `Final` is the handoff to autoland.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Leg {
    Upwind,
    Crosswind,
    Downwind,
    Base,
    Final,
}

/// The heading to fly on a given leg: the runway course plus 90 degrees per leg,
/// left (subtract) for a standard pattern, right (add) otherwise. Upwind is the
/// runway course; Final comes back to it (four 90-degree turns = 360).
pub fn leg_heading(rwy_course_deg: f64, leg: Leg, left: bool) -> f64 {
    let n = match leg {
        Leg::Upwind => 0.0,
        Leg::Crosswind => 1.0,
        Leg::Downwind => 2.0,
        Leg::Base => 3.0,
        Leg::Final => 4.0,
    };
    let turn = if left { -90.0 } else { 90.0 };
    (rwy_course_deg + turn * n).rem_euclid(360.0)
}

/// Move the heading bug toward `target` by at most `max_step` degrees, the
/// SHORT way around, returning the new dial value. A single 90-degree write
/// across the 0/360 seam (271 -> 2 for the right pattern's crosswind) reads
/// to the sim's AP as a 270-degree counterclockwise dial spin, and it turned
/// the LONG way — the 1.8 km loop south over the bay with an AGL dip to
/// 126 ft (LXGB, 2026-07-24). Stepping the dial keeps every write's turn
/// direction unambiguous; at 40 degrees per 500 ms tick a 90-degree leg
/// change completes in ~1.5 s, faster than the AP banks anyway.
pub fn slew_bug(current: f64, target: f64, max_step: f64) -> f64 {
    let d = (target - current + 540.0).rem_euclid(360.0) - 180.0;
    (current + d.clamp(-max_step, max_step)).rem_euclid(360.0)
}

/// The next leg in sequence, or `None` past Final.
pub fn next_leg(leg: Leg) -> Option<Leg> {
    match leg {
        Leg::Upwind => Some(Leg::Crosswind),
        Leg::Crosswind => Some(Leg::Downwind),
        Leg::Downwind => Some(Leg::Base),
        Leg::Base => Some(Leg::Final),
        Leg::Final => None,
    }
}

/// The pure leg-advance decision — POSITION-based, so the sequencing is
/// testable without a sim and immune to how long the autopilot's turns take.
/// `a_m` is distance along the takeoff direction from the departure threshold
/// (negative = on the approach side, before the threshold); `x_m` is cross-
/// track, right of the takeoff course positive. `left` selects the pattern
/// side. Each leg advances when the aircraft reaches the geometric point LESS
/// the turn-anticipation lead, so the slow AP turn rolls out on the line.
pub fn pattern_advance(
    leg: Leg,
    a_m: f64,
    x_m: f64,
    g: &Geometry,
    runway_len_m: f64,
    left: bool,
) -> bool {
    // The pattern sits on the LEFT (negative cross-track) for a standard
    // circuit; `side * x_m` is then "distance out to the pattern side", always
    // positive as we fly away from the runway on crosswind.
    let side = if left { -1.0 } else { 1.0 };
    match leg {
        // Past the departure end — turn crosswind while still climbing to
        // pattern altitude (a real pattern climbs on the crosswind/downwind,
        // not four kilometres straight ahead waiting to reach TPA first).
        Leg::Upwind => a_m >= runway_len_m + g.upwind_past_m,
        // Reaching the downwind spacing — led, so the rollout lands AT it.
        Leg::Crosswind => side * x_m >= g.width_m - g.lead_m,
        // Flown back down the downwind until past the landing threshold by
        // enough to build a normal final — led for the base turn.
        Leg::Downwind => a_m <= -(g.downwind_past_m - g.lead_m),
        // Base has brought us back within one turn radius of the extended
        // centreline: the 90° turn to final rolls out on it.
        Leg::Base => side * x_m <= g.final_capture_m,
        Leg::Final => false,
    }
}

/// What a candidate pattern side knows about the world under and around it:
/// the highest sampled terrain (`None` = no probe data) and the first no-go
/// airspace (prohibited/restricted/danger) its polyline enters at pattern
/// height (`None` = clear or no data).
#[derive(Debug, Clone, Default)]
pub struct SideFacts {
    pub terrain_max_ft: Option<f64>,
    pub airspace: Option<String>,
}

/// Decide the pattern side and altitude — the generic rule, no per-airport
/// exceptions: fly the side clear of BOTH terrain and no-go airspace; switch
/// sides when only the other one is; when neither is clean, prefer the side
/// without an airspace block, raised over its terrain; and when airspace is
/// unavoidable, fall back to terrain-safety alone (terrain kills, paper
/// doesn't) and say so. Missing data degrades gracefully: `None` terrain =
/// assume clear (the old behavior), `None` airspace = clear.
/// Returns (left, tpa_msl_ft, why).
pub fn choose_pattern(
    default_left: bool,
    l: &SideFacts,
    r: &SideFacts,
    field_elev_ft: f64,
    tpa_agl_ft: f64,
) -> (bool, f64, &'static str) {
    let tpa = field_elev_ft + tpa_agl_ft;
    let terrain_ok = |f: &SideFacts| f.terrain_max_ft.is_none_or(|m| tpa - m >= TERRAIN_CLEAR_FT);
    let clean = |f: &SideFacts| terrain_ok(f) && f.airspace.is_none();
    let raise = |f: &SideFacts| (f.terrain_max_ft.unwrap_or(field_elev_ft) + tpa_agl_ft).max(tpa);
    let (df, of) = if default_left { (l, r) } else { (r, l) };
    if clean(df) {
        return (default_left, tpa, "default_side_clear");
    }
    if clean(of) {
        let why = if df.airspace.is_some() {
            "switched_side_for_airspace"
        } else {
            "switched_side_for_terrain"
        };
        return (!default_left, tpa, why);
    }
    // Neither side is clean. Airspace can't be out-climbed at pattern heights
    // (R-164 runs to 30,000), so prefer the side WITHOUT a block, raised over
    // its terrain when needed.
    match (df.airspace.is_some(), of.airspace.is_some()) {
        (true, false) => (!default_left, raise(of), "raised_for_terrain"),
        (false, true) => (default_left, raise(df), "raised_for_terrain"),
        _ => {
            // Both blocked (or both merely terrain-fouled): terrain safety
            // alone picks — the LOWER side, high enough to clear it.
            let (side_left, facts) =
                if l.terrain_max_ft.unwrap_or(0.0) <= r.terrain_max_ft.unwrap_or(0.0) {
                    (true, l)
                } else {
                    (false, r)
                };
            let why = if facts.airspace.is_some() {
                "airspace_unavoidable"
            } else {
                "raised_for_terrain"
            };
            (side_left, raise(facts), why)
        }
    }
}

/// Which pattern side keeps the downwind SOUTH (over the sea, around the Rock)
/// for a runway of the given true course — returns `left` (true = left-hand).
/// At Gibraltar south is the only flyable side (north is Spain); the runway
/// sets the turn: departing 09 (east) the sea is to the RIGHT → right-hand
/// (clockwise); departing 27 (west) it's to the LEFT → left-hand.
pub fn sea_side_is_left(course_deg: f64) -> bool {
    let right_normal = (course_deg + 90.0).rem_euclid(360.0);
    // Right of course points south (bearing 90..270) → fly right-hand (not left).
    !(90.0..270.0).contains(&right_normal)
}

/// A point in the runway frame (`a_m` along the takeoff course from the
/// departure threshold, `x_m` right of it) → lat/lon.
fn frame_to_latlon(rwy: &crate::autoland::Runway, a_m: f64, x_m: f64) -> (f64, f64) {
    let (lat, lon) =
        crate::nav::project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, a_m / M_PER_NM);
    crate::nav::project_latlon(
        lat,
        lon,
        (rwy.course_deg + 90.0).rem_euclid(360.0),
        x_m / M_PER_NM,
    )
}

/// The sampling polyline for one pattern side: points along upwind-extension,
/// crosswind, downwind, base, and final, in the runway frame. ~20 points —
/// inside the plugin probe channel's per-query cap.
fn pattern_polyline(
    rwy: &crate::autoland::Runway,
    g: &Geometry,
    runway_len_m: f64,
    left: bool,
) -> Vec<(f64, f64)> {
    let side = if left { -1.0 } else { 1.0 };
    let x_out = side * g.width_m;
    let a_up = runway_len_m + g.upwind_past_m;
    let a_dn = -g.downwind_past_m;
    let mut pts = Vec::with_capacity(20);
    // Crosswind line (includes the upwind end at x=0).
    for i in 0..4 {
        pts.push((a_up, x_out * i as f64 / 3.0));
    }
    // Downwind line.
    for i in 1..=8 {
        pts.push((a_up + (a_dn - a_up) * i as f64 / 8.0, x_out));
    }
    // Base line back to the centreline.
    for i in 1..=3 {
        pts.push((a_dn, x_out * (3 - i) as f64 / 3.0));
    }
    // Final, back toward the threshold.
    for i in 1..=4 {
        pts.push((a_dn + (0.0 - a_dn) * i as f64 / 4.0, 0.0));
    }
    pts.into_iter()
        .map(|(a, x)| frame_to_latlon(rwy, a, x))
        .collect()
}

/// How close (metres) to an authored waypoint counts as reaching it. Generous,
/// so the slow AP turn rolls through the corner instead of circling the point.
const AUTHORED_CAPTURE_M: f64 = 350.0;

/// One authored circuit waypoint, in the departure-runway frame (see
/// `frame_to_latlon`): `a_m` along the takeoff course from the threshold, `x_m`
/// right of it, and the target altitude (MSL) to be at approaching it.
#[derive(Debug, Clone, Copy)]
struct AuthoredWpt {
    a_m: f64,
    x_m: f64,
    alt_msl_ft: f64,
}

/// A hand-authored circuit for a field where the parametric rectangle can't be
/// flown clear — Gibraltar (LXGB), where R-164 (Spanish restricted, GND-FL300)
/// blankets both sides of any standard pattern (`choose_pattern` returns
/// `airspace_unavoidable`). The airspace can't be dodged — nothing at LXGB can,
/// it's the real political geometry — but the ROCK'S TERRAIN can, and a tight,
/// sane track over the bay beats flying the rectangle through the granite. The
/// route stays on the low-terrain side `choose_pattern` picked, holds the
/// downwind WIDE to carry the ground track around the Rock's flank, and rejoins
/// a straight final where autoland takes it. Scaled to the same geometry, so
/// the pattern speed-hold keeps it this size (at climb power it balloons back
/// over the Rock). `None` at every other field. DRAFT coordinates — the caller
/// validates every waypoint against the live terrain probe before flying it.
fn authored_circuit(
    rwy: &crate::autoland::Runway,
    geom: &Geometry,
    field_elev_ft: f64,
    tpa_msl_ft: f64,
    left: bool,
) -> Option<Vec<AuthoredWpt>> {
    if rwy.icao != "LXGB" {
        return None;
    }
    let side = if left { -1.0 } else { 1.0 };
    let handoff = field_elev_ft + HANDOFF_AGL_FT;
    let w = geom.width_m;
    let up = rwy.length_m + geom.upwind_past_m;
    let dn = geom.downwind_past_m;
    Some(vec![
        // Climb out on the runway heading.
        AuthoredWpt {
            a_m: up,
            x_m: 0.0,
            alt_msl_ft: tpa_msl_ft,
        },
        // Turn to the bay side; downwind held wide to clear the Rock's flank.
        AuthoredWpt {
            a_m: up,
            x_m: side * w * 1.3,
            alt_msl_ft: tpa_msl_ft,
        },
        AuthoredWpt {
            a_m: 0.0,
            x_m: side * w * 1.3,
            alt_msl_ft: tpa_msl_ft,
        },
        AuthoredWpt {
            a_m: -dn,
            x_m: side * w * 1.3,
            alt_msl_ft: tpa_msl_ft,
        },
        // Base back over the water, descending to the handoff height.
        AuthoredWpt {
            a_m: -dn * 1.2,
            x_m: side * w * 0.6,
            alt_msl_ft: handoff,
        },
        // Rejoin the centreline and hand to autoland on a NORMAL final: ~2 nm
        // out (`-dn`, not `-dn * 0.5`), so the handoff height sits on a ~3°
        // glidepath, not a 6.5° dive autoland can't capture. Handing off at 1 nm
        // / 700 ft looped the LXGB approach into a permanent go-around
        // (2026-07-26).
        AuthoredWpt {
            a_m: -dn,
            x_m: 0.0,
            alt_msl_ft: handoff,
        },
    ])
}

/// Probe an authored route against the terrain it's flown at: every waypoint
/// must clear the ground by [`TERRAIN_CLEAR_FT`]. A missing probe (no plugin)
/// reads as clear — the old fail-open behavior. Returns false if any sampled
/// point is too low, so the caller falls back to the rectangle rather than fly
/// hand-authored coordinates into a hill.
async fn authored_route_clears_terrain(
    xp: &XPlaneClient,
    rwy: &crate::autoland::Runway,
    route: &[AuthoredWpt],
) -> bool {
    for wp in route {
        let (lat, lon) = frame_to_latlon(rwy, wp.a_m, wp.x_m);
        if let Some(e) = crate::terrain::max_elevation_ft(xp, &[(lat, lon)]).await {
            if wp.alt_msl_ft - e < TERRAIN_CLEAR_FT {
                return false;
            }
        }
    }
    true
}

/// Fly an authored waypoint route (the LXGB Rock circuit), then hand the final
/// to autoland. Same control surface as the rectangle loop — HDG bug + VS + the
/// pattern speed-hold — but steering great-circle to each waypoint instead of
/// the leg geometry. The last waypoint sits on short final.
#[allow(clippy::too_many_arguments)]
async fn fly_authored_route(
    xp: XPlaneClient,
    base_url: String,
    voice: Option<Voice>,
    route: Vec<AuthoredWpt>,
    rwy: crate::autoland::Runway,
    mag_var: f64,
    pattern_kts: f64,
    callsign: Option<String>,
    apt_dat: String,
    cfg: crate::tools::autopilot::ApConfig,
    approach_ias: f64,
) {
    let rwy_id = rwy.id.clone();
    let mut bug_mag = xp.get_float(HDG_BUG).await.unwrap_or(rwy.course_deg);
    let mut last_vs_cmd: Option<f64> = None;
    let mut prev_ias: Option<(f64, std::time::Instant)> = None;
    let mut ias_trend = 0.0_f64;
    let mut wpt_idx = 0usize;
    loop {
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        if ABORT.load(Ordering::SeqCst) {
            stand_down(
                &voice,
                "Circuit cancelled. Your aircraft.",
                "abort_authored",
            );
            return;
        }
        if crate::xplane::sim_frozen(&xp).await {
            continue;
        }
        let thr_dref = throttle_dref();
        let (lat_r, lon_r, elev_r, psi_r, hp_r, hdg_st, vs_st, ias_r, thr_r) = tokio::join!(
            xp.get_float(LAT),
            xp.get_float(LON),
            xp.get_float(ELEV_M),
            xp.get_float(PSI_TRUE),
            xp.get_float(HPATH),
            xp.get_float(HDG_STATUS),
            xp.get_float(VS_STATUS),
            xp.get_float(IAS),
            xp.get_float(&thr_dref),
        );
        let (Ok(lat), Ok(lon)) = (lat_r, lon_r) else {
            continue;
        };
        let alt_msl_ft = elev_r.map(|m| m * M_TO_FT).unwrap_or(f64::NAN);
        let (hdg_on, vs_on) = (hdg_st.unwrap_or(1.0) > 0.5, vs_st.unwrap_or(1.0) > 0.5);

        let wp = route[wpt_idx];
        let (wlat, wlon) = frame_to_latlon(&rwy, wp.a_m, wp.x_m);
        // Crab so the AP holds the ground TRACK to the waypoint, like the legs.
        let crab = match (psi_r, hp_r) {
            (Ok(psi), Ok(hp)) => {
                (((psi - hp + 540.0).rem_euclid(360.0)) - 180.0).clamp(-25.0, 25.0)
            }
            _ => 0.0,
        };
        let want_hdg = crate::nav::bearing_deg(lat, lon, wlat, wlon) + crab;
        bug_mag = slew_bug(bug_mag, true_to_mag(want_hdg, mag_var), BUG_SLEW_PER_TICK);
        let _ = xp.set_float(HDG_BUG, bug_mag).await;
        if !hdg_on {
            let _ = xp.run_command(CMD_SERVOS_ON).await;
            let _ = xp.run_command(CMD_HDG_MODE).await;
        }

        // Vertical: VS toward the waypoint altitude (the takeoff's proven recipe).
        let vs_cmd = if alt_msl_ft.is_finite() {
            ((wp.alt_msl_ft - alt_msl_ft) * 3.0).clamp(-500.0, 700.0)
        } else {
            0.0
        };
        if !vs_on {
            let _ = xp.run_command(CMD_VS_MODE).await;
            last_vs_cmd = None;
        }
        if last_vs_cmd.is_none_or(|v| (v - vs_cmd).abs() > 40.0) {
            let _ = xp.set_float(VS_DIAL, vs_cmd).await;
            last_vs_cmd = Some(vs_cmd);
        }

        // Power holds the pattern speed — the same law the rectangle loop uses.
        if let (Ok(ias), Ok(thr)) = (ias_r, thr_r) {
            let t_now = std::time::Instant::now();
            if let Some((prev, t_prev)) = prev_ias {
                let dt = t_now.duration_since(t_prev).as_secs_f64();
                if (0.1..=2.0).contains(&dt) {
                    ias_trend = 0.6 * ias_trend + 0.4 * (ias - prev) / dt;
                }
            }
            prev_ias = Some((ias, t_now));
            let delta = crate::tools::speedhold::throttle_adjust(pattern_kts - ias, ias_trend);
            if delta != 0.0 {
                let _ = xp
                    .set_float(&throttle_dref(), (thr + delta).clamp(0.0, 1.0))
                    .await;
            }
        }

        let d_m = crate::nav::haversine_nm(lat, lon, wlat, wlon) * M_PER_NM;
        crate::blackbox::record(
            "circuit",
            serde_json::json!({
                "event": "authored_leg", "wpt": wpt_idx, "d_m": d_m, "bug": bug_mag,
                "alt_msl": alt_msl_ft, "tgt_alt": wp.alt_msl_ft, "cmd_vs": vs_cmd,
            }),
        );

        if d_m < AUTHORED_CAPTURE_M {
            wpt_idx += 1;
            if wpt_idx >= route.len() {
                break;
            }
        }
    }
    // The last waypoint sits on short final — hand the landing to autoland.
    alert(
        &voice,
        Priority::Advisory,
        crate::readback::one_of(&[
            "Turning final. Landing us.",
            "On final now. I'll put her down.",
            "Turning final. Bringing us in.",
        ]),
    );
    transmit(radio_call(
        &callsign,
        &format!("final runway {}", spell_runway(&rwy_id)),
    ));
    crate::blackbox::record(
        "circuit",
        serde_json::json!({"event": "handoff_autoland_authored", "rwy": rwy_id}),
    );
    crate::autoland::ABORT.store(false, Ordering::SeqCst);
    // FORCE the pattern's own runway. Bare `None` sends autoland to its
    // EMERGENCY behaviour — search the whole apt.dat for the best end by track
    // — which is right for an engine failure and catastrophically wrong here:
    // we are on short final to a runway we just took off from and flew a
    // pattern around, and the search does not know that.
    crate::autoland::run_autoland(
        base_url,
        apt_dat,
        cfg,
        approach_ias,
        voice,
        None,
        Some(rwy_id.clone()),
    );
    ACTIVE.store(false, Ordering::SeqCst);
}

/// Spawn the circuit orchestrator on its own thread + runtime + client, the
/// same shape as the takeoff and autoland loops.
pub fn start(base_url: String, voice: Option<Voice>, p: Params) {
    if ACTIVE.swap(true, Ordering::SeqCst) {
        return;
    }
    ABORT.store(false, Ordering::SeqCst);
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            ACTIVE.store(false, Ordering::SeqCst);
            return;
        };
        rt.block_on(fly(XPlaneClient::new(base_url.clone()), base_url, voice, p));
        ACTIVE.store(false, Ordering::SeqCst);
        // (base_url is cloned into the client and passed on for the sub-loops.)
    });
}

fn alert(voice: &Option<Voice>, priority: Priority, line: &str) {
    // Log every spoken line so the auto-flight narration shows up in the app
    // log, not just through the speakers (background loops have no reporter).
    tracing::info!(target: "xvoicelayer::circuit", spoke = %line, "copilot");
    if let Some(v) = voice {
        v.alert(priority, line.to_string());
    }
}

/// Bail out cleanly: clear our flag and let whatever sub-loop is flying stand
/// down. Never leaves the aircraft with nobody watching the loop.
fn stand_down(voice: &Option<Voice>, line: &str, reason: &str) {
    ACTIVE.store(false, Ordering::SeqCst);
    crate::blackbox::record(
        "circuit",
        serde_json::json!({"event": "stand_down", "reason": reason}),
    );
    alert(voice, Priority::Caution, line);
}

async fn fly(xp: XPlaneClient, base_url: String, voice: Option<Voice>, p: Params) {
    // `base_url` is kept owned so each sub-loop gets its own client for its own
    // thread (the takeoff loop and the autoland loop each run on one).
    // The departure runway — its course drives every leg heading. Fit it from
    // where we're pointing right now (lined up before the roll), the same way
    // the takeoff loop does.
    let (Ok(lat), Ok(lon), Ok(psi)) = (
        xp.get_float(LAT).await,
        xp.get_float(LON).await,
        xp.get_float(PSI_TRUE).await,
    ) else {
        stand_down(
            &voice,
            "I can't read our position. The circuit is off, your aircraft.",
            "no_position",
        );
        return;
    };
    let cands = nearby_runways(&p.apt_dat, lat, lon, 3.0).unwrap_or_default();
    let Some((rwy, _)) = autotakeoff::runway_fit(&cands, lat, lon, psi, p.takeoff.needed_m) else {
        stand_down(
            &voice,
            "We're not on a runway yet. I can only fly the circuit from the runway itself. \
             Taxi us over and line us up first, then say do a circuit.",
            "not_lined_up",
        );
        return;
    };
    let rwy_course = rwy.course_deg;
    let rwy_id = rwy.id.clone();
    let callsign = crew_callsign(&xp).await;
    let field_elev_ft = rwy.elev_ft;

    // Geometry scaled to the AP's real turn radius at pattern speed (~Vy plus
    // a bit) — fixed constants smaller than the turn diameter degenerated the
    // rectangle into loops (LXGB, 2026-07-24).
    let gs_est_mps = p.takeoff.vy_kts.max(60.0) * 0.514_444 * 1.1;
    // The pattern SPEED the throttle holds in the loop (~Vy), so the flown turn
    // radius matches the geometry above — a pattern flown at climb power runs
    // away to Vne and balloons ~3x wider (LXGB, 2026-07-26). Captured here,
    // before `p.takeoff` is moved into the takeoff loop.
    let pattern_kts = pattern_speed_kts(p.takeoff.vy_kts, p.approach_ias, p.vs_clean_kts);
    // ...and the speed the approach-setup legs are flown at.
    let approach_kts = approach_leg_kts(pattern_kts, p.approach_ias);
    // Two radii, because the base-to-final turn is flown slower than the
    // pattern and is the one dimension where an over-estimate rolls us out
    // short of the centreline rather than merely wide.
    let geom = Geometry::for_radii(turn_radius_m(gs_est_mps), final_turn_radius_m(approach_kts));
    // The floor under the climb's speed protection, captured before `p` moves.
    let p_climb_floor_kts = p.climb_floor_kts;
    // ...and the speed the climb is flown at, for the limiter's full-rate
    // threshold — NOT the leg speed the throttle holds (see
    // `climb_full_rate_kts` for the four knots that cost a circuit).
    let p_vy_kts = p.takeoff.vy_kts;

    // The world under BOTH candidate patterns: terrain probed from the sim
    // itself, and the no-go airspace polygons the G1000 draws (OpenAir file).
    // One generic rule picks the side and height; missing data degrades to
    // the old behavior, never to a guess.
    let poly_l = pattern_polyline(&rwy, &geom, rwy.length_m, true);
    let poly_r = pattern_polyline(&rwy, &geom, rwy.length_m, false);
    let l_max = crate::terrain::max_elevation_ft(&xp, &poly_l).await;
    let r_max = crate::terrain::max_elevation_ft(&xp, &poly_r).await;
    let spaces = {
        let path = p.airspace_path.clone();
        tokio::task::spawn_blocking(move || crate::airspace::load_near(&path, lat, lon, 0.5))
            .await
            .unwrap_or_default()
    };
    let tpa_probe_ft = field_elev_ft + p.tpa_agl_ft;
    let l_facts = SideFacts {
        terrain_max_ft: l_max,
        airspace: crate::airspace::first_conflict(&spaces, &poly_l, tpa_probe_ft)
            .map(|a| a.spoken_name()),
    };
    let r_facts = SideFacts {
        terrain_max_ft: r_max,
        airspace: crate::airspace::first_conflict(&spaces, &poly_r, tpa_probe_ft)
            .map(|a| a.spoken_name()),
    };
    let (left, tpa_msl_ft, side_pick) = choose_pattern(
        p.left_pattern,
        &l_facts,
        &r_facts,
        field_elev_ft,
        p.tpa_agl_ft,
    );
    // Gibraltar: the ONLY safe side is SOUTH — over the sea, looping around the
    // Rock. NORTH is Spain (restricted, foreign soil). choose_pattern picks the
    // LOWER-TERRAIN side, which at LXGB is the flat isthmus to the north, and it
    // flew the pattern straight into Spanish airspace (2026-07-26). Override to
    // the sea side; the runway sets the turn direction — 09 loops right
    // (clockwise), 27 loops left (anticlockwise), both staying over the water.
    let (left, side_pick) = if rwy.icao == "LXGB" {
        (sea_side_is_left(rwy_course), "lxgb_south_over_the_sea")
    } else {
        (left, side_pick)
    };
    crate::blackbox::record(
        "circuit",
        serde_json::json!({
            "event": "armed", "runway": rwy.id, "icao": rwy.icao,
            "course": rwy_course, "field_elev_ft": field_elev_ft,
            "tpa_msl_ft": tpa_msl_ft, "left": left, "side_pick": side_pick,
            "left_max_ft": l_max, "right_max_ft": r_max,
            "left_airspace": l_facts.airspace, "right_airspace": r_facts.airspace,
            "airspaces_near": spaces.len(),
            "radius_m": geom.final_capture_m, "width_m": geom.width_m,
            "downwind_past_m": geom.downwind_past_m,
        }),
    );
    let side_word = |l: bool| if l { "left" } else { "right" };
    match side_pick {
        "switched_side_for_terrain" => alert(
            &voice,
            Priority::Caution,
            &format!(
                "Terrain on the {} side, flying a {}-hand circuit instead.",
                side_word(p.left_pattern),
                side_word(left)
            ),
        ),
        "switched_side_for_airspace" => {
            let name = if p.left_pattern {
                &l_facts.airspace
            } else {
                &r_facts.airspace
            };
            alert(
                &voice,
                Priority::Caution,
                &format!(
                    "The {} side runs into {}, flying a {}-hand circuit instead.",
                    side_word(p.left_pattern),
                    name.as_deref().unwrap_or("restricted airspace"),
                    side_word(left)
                ),
            );
        }
        "raised_for_terrain" => alert(
            &voice,
            Priority::Caution,
            &format!("Terrain in the pattern, flying it at {tpa_msl_ft:.0} feet."),
        ),
        "airspace_unavoidable" => {
            let name = if left {
                &l_facts.airspace
            } else {
                &r_facts.airspace
            };
            alert(
                &voice,
                Priority::Caution,
                &format!(
                    "No pattern side stays clear of {} here, flying the {} side on terrain alone.",
                    name.as_deref().unwrap_or("restricted airspace"),
                    side_word(left)
                ),
            );
        }
        _ => {}
    }
    alert(
        &voice,
        Priority::Advisory,
        crate::readback::one_of(&[
            "Flying a full circuit. My takeoff first.",
            "Full circuit coming up. I'll take us off first.",
            "Let's fly the circuit. My takeoff to start.",
        ]),
    );

    // --- Phase 1: the takeoff (the proven loop). Arm it, then wait for it to
    // hand off. It clears its own ACTIVE at the thousand-foot handoff — or on a
    // reject, which we tell apart by how high we ended up.
    // Read before the move: `p.takeoff` goes into the takeoff loop below, and
    // the base leg needs to know whether this airframe HAS gear to lower.
    let retractable_gear = p.takeoff.retract_gear;
    autotakeoff::start(base_url.clone(), voice.clone(), p.takeoff);
    loop {
        tokio::time::sleep(std::time::Duration::from_millis(300)).await;
        if ABORT.load(Ordering::SeqCst) {
            stand_down(
                &voice,
                "Circuit cancelled. Your aircraft.",
                "abort_takeoff_phase",
            );
            return;
        }
        if !autotakeoff::ACTIVE.load(Ordering::SeqCst) {
            break;
        }
    }
    let agl_ft = xp.get_float(AGL_M).await.unwrap_or(0.0) * M_TO_FT;
    if agl_ft < AIRBORNE_MIN_AGL_FT {
        stand_down(
            &voice,
            "The takeoff didn't get us airborne, so the circuit is off.",
            "takeoff_rejected",
        );
        return;
    }
    crate::blackbox::record(
        "circuit",
        serde_json::json!({"event": "takeoff_handoff", "agl_ft": agl_ft}),
    );

    // --- Phase 2a: an AUTHORED track, where the parametric rectangle can't be
    // flown clear (LXGB — R-164 blankets both sides, so the rectangle flies
    // through Spanish airspace AND the Rock). The airspace is unavoidable here,
    // but the route keeps the ground track over the bay and around the Rock's
    // terrain. Validated against the live terrain probe first; if any waypoint
    // isn't clear (a wrong hand-authored point, an off-nominal start), fall
    // back to the standard pattern rather than fly it into a hill.
    // The authored around-the-Rock loop, once the sea-side fix pointed it the
    // right way (clockwise on 09), flew a clean circuit south around the Rock and
    // handed to autoland — confirmed in the sim (2026-07-26). It runs whenever
    // it's available and terrain-clear.
    if let Some(route) = authored_circuit(&rwy, &geom, field_elev_ft, tpa_msl_ft, left) {
        if authored_route_clears_terrain(&xp, &rwy, &route).await {
            crate::blackbox::record(
                "circuit",
                serde_json::json!({"event": "authored_route", "wpts": route.len()}),
            );
            alert(
                &voice,
                Priority::Advisory,
                crate::readback::one_of(&[
                    "Airborne. Flying the bay circuit, around the Rock.",
                    "Off the ground. Taking the bay circuit around the Rock.",
                    "Airborne. Around the Rock we go, over the bay.",
                ]),
            );
            let mag_var = xp.get_float(MAG_VAR).await.unwrap_or(0.0);
            fly_authored_route(
                XPlaneClient::new(base_url.clone()),
                base_url.clone(),
                voice.clone(),
                route,
                rwy.clone(),
                mag_var,
                pattern_kts,
                callsign.clone(),
                p.apt_dat.clone(),
                p.cfg.clone(),
                p.approach_ias,
            )
            .await;
            return;
        }
        crate::blackbox::record(
            "circuit",
            serde_json::json!({"event": "authored_route_unsafe"}),
        );
        alert(
            &voice,
            Priority::Caution,
            "The Rock circuit isn't clear from here. Flying the standard pattern.",
        );
    }
    alert(
        &voice,
        Priority::Advisory,
        crate::readback::one_of(&[
            "Airborne. Flying the pattern.",
            "Off the ground. Into the pattern.",
            "Airborne. Taking us round the pattern.",
        ]),
    );

    // --- Phase 2: the pattern rides the AP EXACTLY as the takeoff left it —
    // servos + HDG + VS engaged, VS dial at the climb rate. NOTHING is
    // re-pressed wholesale here: the old pattern-start recipe (servos_on +
    // HDG press + ALT_HOLD) stomped those working modes — X-Plane's
    // `altitude_hold` holds the CURRENT altitude, which locked every pattern
    // at the ~520 ft handoff height instead of climbing to TPA, left the
    // terrain guard's "climb" a NO-OP, and the aircraft flew straight through
    // its commanded crosswind turn into terrain SW of LXGB (2026-07-24,
    // blackboxed: flat AGL all flights, 6 s of no-turn on crosswind).
    // Vertical is VS-mode + dial (the takeoff's proven recipe); modes are
    // re-engaged ONLY when their status reads off, like the takeoff's guard.
    let mag_var = xp.get_float(MAG_VAR).await.unwrap_or(0.0);

    let mut leg = Leg::Upwind;
    // The dial's CURRENT value seeds the slew (the takeoff left it near the
    // runway heading); every subsequent write moves it the short way only.
    let mut bug_mag = match xp.get_float(HDG_BUG).await {
        Ok(v) => v,
        Err(_) => true_to_mag(rwy_course, mag_var),
    };
    bug_mag = slew_bug(
        bug_mag,
        true_to_mag(leg_heading(rwy_course, Leg::Upwind, left), mag_var),
        BUG_SLEW_PER_TICK,
    );
    let _ = xp.set_float(HDG_BUG, bug_mag).await;
    // Forward terrain guard cadence + throttle; a fired guard raises the
    // vertical target floor for a while instead of pressing dead buttons.
    let mut tick: u32 = 0;
    let mut guard_warned: Option<std::time::Instant> = None;
    let mut guard_floor_ft: Option<(f64, std::time::Instant)> = None;
    // VS-dial bookkeeping (the takeoff's recipe: engage syncs the dial away,
    // so rewrite after any re-engage; skip no-op writes otherwise).
    let mut last_vs_cmd: Option<f64> = None;
    // Speed-hold bookkeeping: power works the throttle to hold `pattern_kts`,
    // leading on the airspeed trend EMA'd tick to tick (the flight law's input).
    let mut prev_ias: Option<(f64, std::time::Instant)> = None;
    let mut ias_trend = 0.0_f64;
    // Vertical speed, derived the same way, for the altitude capture's lead.
    let mut prev_alt: Option<(f64, std::time::Instant)> = None;
    let mut vs_est = 0.0_f64;
    // ...and the rate we are closing the threshold at, derived the same way
    // again, for the glidepath's feed-forward. GROUND closure, not airspeed:
    // the path descends at 318 ft for every mile actually covered, so a
    // headwind genuinely slows the target down and an IAS proxy would
    // over-command by the whole wind (see `path_descent_fpm`).
    let mut prev_along: Option<(f64, std::time::Instant)> = None;
    let mut closure_kts = 0.0_f64;
    // The ratcheted base/final descent target — see the vertical law below.
    let mut descent_target: Option<f64> = None;
    // Gear goes down ONCE, at the 180, and the latch is what makes it once.
    let mut gear_lowered = !retractable_gear;
    // Set when the base turn onto final may BEGIN. The handover waits for the
    // geometry to become flyable, not merely for the leg to change.
    let mut turning_final = false;

    loop {
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        if ABORT.load(Ordering::SeqCst) {
            stand_down(
                &voice,
                "Circuit cancelled. Your aircraft.",
                "abort_pattern_phase",
            );
            return;
        }
        // A frozen sim (pause/replay) suspends the sequence; the AP holds.
        if crate::xplane::sim_frozen(&xp).await {
            continue;
        }

        let (lat, lon, agl_m, elev_m, psi_r, hpath_r, hdg_st, vs_st) = tokio::join!(
            xp.get_float(LAT),
            xp.get_float(LON),
            xp.get_float(AGL_M),
            xp.get_float(ELEV_M),
            xp.get_float(PSI_TRUE),
            xp.get_float(HPATH),
            xp.get_float(HDG_STATUS),
            xp.get_float(VS_STATUS),
        );
        let thr_dref = throttle_dref();
        let (ias_r, thr_r) = tokio::join!(xp.get_float(IAS), xp.get_float(&thr_dref));
        let (Ok(lat), Ok(lon)) = (lat, lon) else {
            continue;
        };
        let agl_ft = agl_m.unwrap_or(0.0) * M_TO_FT;
        let alt_msl_ft = elev_m.map(|m| m * M_TO_FT).unwrap_or(f64::NAN);
        let (hdg_on, vs_on) = (hdg_st.unwrap_or(1.0) > 0.5, vs_st.unwrap_or(1.0) > 0.5);
        // Position in the departure-runway frame: along the takeoff direction
        // (negative on the approach side) and cross-track (right positive).
        let a_m = -along_track_nm(lat, lon, &rwy) * M_PER_NM;
        let x_m = cross_track_nm(lat, lon, &rwy) * M_PER_NM;

        // --- Hand over only once our own turn onto final has rolled out. ---
        // `turning_final` means the turn may BEGIN; the aircraft is still up
        // to 1.3 radii abeam at that instant, pointing across the centreline.
        // See `ready_for_autoland` for why "autoland could close this" is not
        // the same question as "we are lined up".
        // Rolled out is judged on the ground TRACK, not the nose: the leg
        // commands intercept-plus-crab, so on the centreline in a crosswind
        // the heading carries the whole wind-correction angle and a
        // heading-based gate never passes (see `ready_for_autoland`). A
        // failed track read reads as "not rolled out yet": it can only delay
        // the handoff by a tick, where treating it as aligned would hand over
        // the very thing this guard exists to prevent.
        let hdg_err = hpath_r
            .as_ref()
            .ok()
            .map(|hp| ((hp - rwy_course + 540.0).rem_euclid(360.0)) - 180.0)
            .unwrap_or(180.0);
        if turning_final && ready_for_autoland(x_m, a_m.abs(), hdg_err) {
            let alt_msl = field_elev_ft + agl_ft;
            let arr = ArrivalCtx {
                apt_dat: &p.apt_dat,
                cifp_dir: &p.cifp_dir,
                fix_data_path: &p.fix_data_path,
                nav_data_path: &p.nav_data_path,
                cfg: &p.cfg,
                approach_ias: p.approach_ias,
            };
            if try_published_arrival(
                arr, &rwy.icao, &rwy_id, &base_url, &voice, lat, lon, alt_msl,
            )
            .await
            {
                ACTIVE.store(false, Ordering::SeqCst);
                return;
            }
            crate::blackbox::record(
                "circuit",
                serde_json::json!({
                    "event": "handoff_autoland",
                    "agl_ft": agl_ft,
                    "icao": rwy.icao,
                    "rwy": rwy_id,
                    "xtk_m": x_m,
                    "along_m": a_m,
                }),
            );
            crate::autoland::ABORT.store(false, Ordering::SeqCst);
            crate::autoland::run_autoland(
                base_url.clone(),
                p.apt_dat.clone(),
                p.cfg.clone(),
                p.approach_ias,
                voice.clone(),
                None,
                Some(rwy_id.clone()),
            );
            ACTIVE.store(false, Ordering::SeqCst);
            return;
        }

        // Fly each leg as a TRACK, not a bare heading: crab into the wind so the
        // AP holds the ground track. Without this a 12 kt crosswind blew the
        // downwind 900 m back into the field (the teardrop, 2026-07-23).
        // crab = heading - track; command heading = leg heading + crab.
        let (psi_v, hp_v) = (psi_r.ok(), hpath_r.ok());
        let crab = match (psi_v, hp_v) {
            (Some(psi), Some(hp)) => {
                (((psi - hp + 540.0).rem_euclid(360.0)) - 180.0).clamp(-25.0, 25.0)
            }
            _ => 0.0,
        };
        // Every leg but the last flies its own heading. FINAL flies an
        // intercept: it is the leg whose whole job is to be ON the centreline
        // by the handoff, and commanding the bare runway course makes it a leg
        // that flies parallel to the line it is supposed to join.
        let leg_hdg = match leg {
            Leg::Final => final_intercept_hdg(rwy_course, x_m),
            _ => leg_heading(rwy_course, leg, left),
        };
        let want_hdg = leg_hdg + crab;
        let target_mag = true_to_mag(want_hdg, mag_var);
        bug_mag = slew_bug(bug_mag, target_mag, BUG_SLEW_PER_TICK);
        let _ = xp.set_float(HDG_BUG, bug_mag).await;
        // The lateral mode must actually be ON for the bug to mean anything —
        // re-engage like the takeoff's VS guard, never press blind.
        if !hdg_on {
            let _ = xp.run_command(CMD_SERVOS_ON).await;
            let _ = xp.run_command(CMD_HDG_MODE).await;
        }

        // Forward terrain guard — generic TAWS-lite, every ~2 s: probe the
        // ground 1.2 nm ahead along the ground track; a hit raises the
        // vertical target floor (flown by the VS law below) for a minute.
        // Silently inactive without the plugin probe (the old behavior).
        tick = tick.wrapping_add(1);
        if tick.is_multiple_of(4) && alt_msl_ft.is_finite() {
            if let Some(hp) = hp_v {
                let (glat, glon) = crate::nav::project_latlon(lat, lon, hp, 1.2);
                if let Some(e) = crate::terrain::max_elevation_ft(&xp, &[(glat, glon)]).await {
                    if alt_msl_ft - e < 400.0 {
                        guard_floor_ft = Some((e + 800.0, std::time::Instant::now()));
                        if guard_warned.is_none_or(|t| t.elapsed().as_secs() >= 30) {
                            guard_warned = Some(std::time::Instant::now());
                            crate::blackbox::record(
                                "circuit",
                                serde_json::json!({"event": "terrain_guard_climb", "elev_ft": e, "to_ft": e + 800.0}),
                            );
                            alert(&voice, Priority::Warning, "Terrain ahead. Climbing.");
                        }
                    }
                }
            }
        }

        // --- Vertical: the takeoff's proven VS recipe, never ALT_HOLD. The
        // target is the leg's altitude (TPA; the handoff height from base),
        // floored by an active terrain-guard demand. The dial is written on
        // change; a dropped VS mode is re-engaged and the dial rewritten
        // (engage syncs it away — the takeoff's lesson).
        guard_floor_ft = guard_floor_ft.filter(|(_, t)| t.elapsed().as_secs() < 60);
        // GEAR DOWN abeam the touchdown point — the same moment the descent
        // starts, which is how a real pattern does it: power back, gear down,
        // start down. Putting it a leg later (on base) while the descent had
        // already moved to the 180 left the aircraft descending, slowing and
        // gear-up for the whole second half of downwind, which is exactly the
        // configuration an airframe's gear warning exists to shout about — and
        // it did, all the way round (SF50 at EGHI, twice).
        //
        // Base is kept as the backstop for any path that reaches it without
        // passing the abeam test; the latch means it can only fire once.
        if !gear_lowered
            && (matches!(leg, Leg::Base) || (matches!(leg, Leg::Downwind) && a_m < 0.0))
        {
            let _ = xp.set_float(GEAR_HANDLE, 1.0).await;
            alert(&voice, Priority::Advisory, "Gear down.");
            gear_lowered = true;
        }
        let leg_target = match leg {
            // Abeam the touchdown point and later, START DOWN — the 180, where
            // a real pattern reduces power. Holding TPA to the base turn left
            // the whole descent to one short leg it could not fit (see
            // DOWNWIND_DESCENT_AGL_FT). Deliberately a FLAT target and not the
            // path: the path's distance term here is the Manhattan
            // |along| + |across|, which on an OUTBOUND downwind shrinks as the
            // aircraft flies away from the runway and would ratchet us down to
            // a few hundred feet while still abeam. Ratcheted like base, so it
            // can only ever descend.
            Leg::Downwind if a_m < 0.0 => {
                let want = (field_elev_ft + DOWNWIND_DESCENT_AGL_FT).min(tpa_msl_ft);
                let t = descent_target.map_or(want, |d: f64| d.min(want));
                descent_target = Some(t);
                t
            }
            // Base and final descend the 3° PATH, not to a flat height.
            Leg::Base | Leg::Final => {
                // ...and RATCHETED: an approach descends. Whatever the
                // geometry does mid-turn, the target may fall and hold, never
                // climb back — a target that rises is a level-off with the
                // runway ahead, and the aircraft chases it every time.
                let want = approach_leg_alt_ft(
                    field_elev_ft,
                    tpa_msl_ft,
                    path_dist_m(leg, a_m, geom.downwind_past_m),
                );
                let t = descent_target.map_or(want, |d: f64| d.min(want));
                descent_target = Some(t);
                t
            }
            _ => {
                descent_target = None;
                tpa_msl_ft
            }
        };
        // The guard floor never applies on base/final: descending toward the
        // runway is the POINT there, and autoland brings its own terrain
        // floor. A ridge-probe floor that lingered into base handed autoland
        // the approach 190 ft high and it balked over the numbers
        // (2026-07-24).
        let target_alt = if matches!(leg, Leg::Base | Leg::Final) {
            leg_target
        } else {
            guard_floor_ft.map_or(leg_target, |(f, _)| leg_target.max(f))
        };
        // Vertical speed, derived from successive altitude reads (the same
        // trick the speed law uses on IAS) — the lead term the capture needs.
        if alt_msl_ft.is_finite() {
            let t_now = std::time::Instant::now();
            if let Some((prev, t_prev)) = prev_alt {
                let dt = t_now.duration_since(t_prev).as_secs_f64();
                if (0.1..=2.0).contains(&dt) {
                    vs_est = 0.6 * vs_est + 0.4 * (alt_msl_ft - prev) / dt * 60.0;
                }
            }
            prev_alt = Some((alt_msl_ft, t_now));
        }
        // The target MOVES on final: it is the glidepath at our distance to
        // run, so it walks down as we close, and the capture has to be told
        // that or it flies a permanent lag (see `alt_capture_vs`). Only on
        // final — base aims at the fixed roll-out point, and downwind at a
        // level altitude, so for those the target really is standing still.
        if a_m.is_finite() {
            let t_now = std::time::Instant::now();
            if let Some((prev, t_prev)) = prev_along {
                let dt = t_now.duration_since(t_prev).as_secs_f64();
                if (0.1..=2.0).contains(&dt) {
                    let closing = (prev.abs() - a_m.abs()) / dt * 1.943_844;
                    closure_kts = (0.6 * closure_kts + 0.4 * closing).clamp(0.0, 400.0);
                }
            }
            prev_along = Some((a_m, t_now));
        }
        let target_rate = if leg == Leg::Final {
            path_descent_fpm(closure_kts)
        } else {
            0.0
        };
        let vs_want = if alt_msl_ft.is_finite() {
            alt_capture_vs(target_alt, target_rate, alt_msl_ft, vs_est)
        } else {
            0.0
        };
        // ...and only as much of it as the airspeed will pay for. The AP buys
        // rate with knots, and on a piston twin in a climbing turn there is no
        // power left to buy them back — see `climb_limit_fpm`.
        let ias_read = match &ias_r {
            Ok(v) => *v,
            Err(_) => f64::NAN,
        };
        let want_now = match leg {
            Leg::Base | Leg::Final => approach_kts,
            _ => pattern_kts,
        };
        let full_kts = climb_full_rate_kts(want_now, p_vy_kts, p_climb_floor_kts);
        let vs_cmd = climb_limit_fpm(vs_want, ias_read, ias_trend, full_kts, p_climb_floor_kts);
        if !vs_on {
            let _ = xp.run_command(CMD_VS_MODE).await;
            last_vs_cmd = None; // engage synced the dial; force a rewrite
        }
        if last_vs_cmd.is_none_or(|v| (v - vs_cmd).abs() > 40.0) {
            let _ = xp.set_float(VS_DIAL, vs_cmd).await;
            last_vs_cmd = Some(vs_cmd);
        }

        // --- Power for speed: the throttle holds the pattern speed (~Vy) while
        // the VS law above flies altitude. Left at climb power the Twin Otter
        // ran the whole pattern up to Vne (202 KIAS), ~3x the planned turn
        // radius, straight through the Spanish airspace choose_pattern had tried
        // to avoid (LXGB, 2026-07-26). The flight speed-hold law, folded into
        // this tick; autoland re-takes the throttle at the handoff below.
        let mut ias_now = f64::NAN;
        if let (Ok(ias), Ok(thr)) = (ias_r, thr_r) {
            ias_now = ias;
            let t_now = std::time::Instant::now();
            if let Some((prev, t_prev)) = prev_ias {
                let dt = t_now.duration_since(t_prev).as_secs_f64();
                if (0.1..=2.0).contains(&dt) {
                    ias_trend = 0.6 * ias_trend + 0.4 * (ias - prev) / dt;
                }
            }
            prev_ias = Some((ias, t_now));
            // Base and final fly the approach-setup speed; the rest of the
            // pattern flies Vy (which is also what the geometry was sized on).
            let want_kts = match leg {
                Leg::Base | Leg::Final => approach_kts,
                _ => pattern_kts,
            };
            let delta = crate::tools::speedhold::throttle_adjust(want_kts - ias, ias_trend);
            if delta != 0.0 {
                let _ = xp
                    .set_float(&throttle_dref(), (thr + delta).clamp(0.0, 1.0))
                    .await;
            }
        }

        crate::blackbox::record(
            "circuit",
            serde_json::json!({
                "event": "leg", "leg": format!("{leg:?}"), "agl_ft": agl_ft,
                "a_m": a_m, "x_m": x_m, "crab": crab,
                "bug": bug_mag, "tgt": target_mag, "psi": psi_v,
                "hdg_on": hdg_on, "vs_on": vs_on, "cmd_vs": vs_cmd,
                "alt_msl": alt_msl_ft, "target_alt": target_alt,
                "ias": ias_now, "pat_kts": pattern_kts,
            }),
        );

        if !pattern_advance(leg, a_m, x_m, &geom, rwy.length_m, left) {
            continue;
        }
        let Some(nxt) = next_leg(leg) else { continue };
        leg = nxt;

        // The wind-corrected bug is written every tick above; the transitions
        // just announce, report, and change altitude / hand off.
        match leg {
            Leg::Crosswind => {
                alert(&voice, Priority::Advisory, "Crosswind.");
            }
            Leg::Downwind => {
                alert(&voice, Priority::Advisory, "Downwind.");
                // The copilot reports the pattern position to the tower.
                transmit(radio_call(
                    &callsign,
                    &format!("downwind runway {}", spell_runway(&rwy_id)),
                ));
            }
            Leg::Base => {
                // Turn base and start down — the VS law above flies the
                // descent once the leg target drops to the handoff height.
                alert(&voice, Priority::Advisory, "Base. Descending.");
                transmit(radio_call(&callsign, "turning base"));
            }
            Leg::Final => {
                // ARM the turn onto final; do NOT hand over here. Reaching
                // `final_capture_m` means the base turn may BEGIN, not that
                // the aircraft is on the centreline — it is still 1.3 radii
                // abeam at that instant. Handing autoland an aircraft 2,815 m
                // off with 2,421 m to run gave it a 49° angle it could not
                // close, and it crossed the threshold 188 m off the paint
                // (EGHI, 2026-07-30). The circuit flies its own turn now and
                // lets go when the geometry is actually flyable.
                alert(
                    &voice,
                    Priority::Advisory,
                    crate::readback::one_of(&[
                        "Turning final. Landing us.",
                        "On final now. I'll put her down.",
                        "Turning final. Bringing us in.",
                    ]),
                );
                // The final report — this is what earns the landing clearance.
                transmit(radio_call(
                    &callsign,
                    &format!("final runway {}", spell_runway(&rwy_id)),
                ));
                turning_final = true;
            }
            Leg::Upwind => {}
        }
    }
}

/// The arrival-relevant slice of [`Params`] — passed as fields because by the
/// time the arrival runs, `p.takeoff` has been moved into the takeoff loop and
/// `&p` can no longer be borrowed whole.
struct ArrivalCtx<'a> {
    apt_dat: &'a str,
    cifp_dir: &'a str,
    fix_data_path: &'a str,
    nav_data_path: &'a str,
    cfg: &'a ApConfig,
    approach_ias: f64,
}

/// Try to hand the arrival to the airport's PUBLISHED approach (the terrain-safe
/// path) instead of a raw autoland. Builds the approach plan off the realtime
/// loop, and only takes it when [`crate::procedure::approach_flyable_from`] says
/// we're positioned to fly it — outside and above the FAF. Returns whether the
/// procedure follower took over. A tight circuit (inside the FAF, at pattern
/// height) always returns false, so autoland flies the short final unchanged.
#[allow(clippy::too_many_arguments)]
async fn try_published_arrival(
    arr: ArrivalCtx<'_>,
    icao: &str,
    rwy_id: &str,
    base_url: &str,
    voice: &Option<Voice>,
    lat: f64,
    lon: f64,
    alt_msl: f64,
) -> bool {
    if arr.cifp_dir.is_empty() {
        return false;
    }
    let (apt_dat, cifp, fixp, navp) = (
        arr.apt_dat.to_string(),
        arr.cifp_dir.to_string(),
        arr.fix_data_path.to_string(),
        arr.nav_data_path.to_string(),
    );
    let (icao, rwy_id, cfg, ias) = (
        icao.to_string(),
        rwy_id.to_string(),
        arr.cfg.clone(),
        arr.approach_ias,
    );
    // find_airport + procedure load/resolve read big files — off the reactor.
    let plan = tokio::task::spawn_blocking(move || {
        let airport = crate::nav::find_airport(&apt_dat, &icao).ok().flatten()?;
        let end = crate::procedure::EndAction::Approach {
            apt_dat: apt_dat.clone(),
            cfg,
            approach_ias: ias,
        };
        crate::procedure::approach_plan_for(
            &cifp, &fixp, &navp, &airport, &icao, &rwy_id, lat, lon, end,
        )
        .ok()
    })
    .await
    .ok()
    .flatten();
    let Some(plan) = plan else {
        return false;
    };
    if !crate::procedure::approach_flyable_from(&plan, lat, lon, alt_msl) {
        crate::blackbox::record(
            "circuit",
            serde_json::json!({"event": "approach_declined", "reason": "inside_or_below_faf"}),
        );
        return false;
    }
    crate::blackbox::record(
        "circuit",
        serde_json::json!({"event": "handoff_procedure", "name": plan.name}),
    );
    alert(
        voice,
        Priority::Advisory,
        &format!("Joining {}.", plan.name),
    );
    crate::procedure::ABORT.store(false, Ordering::SeqCst);
    crate::procedure::start(base_url.to_string(), voice.clone(), plan);
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The base leg hands autoland an aircraft ON the path it is about to fly,
    /// whatever size the pattern is. Luton 2026-07-28: a flat 700 ft target put
    /// the MD-82 at the handover 520 ft below a path it then had one mile to
    /// catch, and it did not.
    #[test]
    fn the_base_leg_descends_the_path_not_to_a_flat_height() {
        let (elev, tpa) = (520.0, 520.0 + 1500.0);
        // 4 nm out on the centreline: the 3° path is ~1,272 ft above the field.
        let far = approach_leg_alt_ft(elev, tpa, -4.0 * M_PER_NM);
        assert!(
            (far - (elev + 4.0 * crate::autoland::GLIDEPATH_FT_PER_NM)).abs() < 1.0,
            "should target the path, got {far}"
        );
        // Closer in, the path is lower — and the leg follows it down.
        let near = approach_leg_alt_ft(elev, tpa, -2.5 * M_PER_NM);
        assert!(near < far, "{near} !< {far}");
        // The floor may not hold us ABOVE the path. It used to: a flat
        // 700 ft AGL bottom meant that on any pattern turning final inside
        // ~2.2 nm — where the 3° path is below 700 — the leg levelled off over
        // the path it was trying to join. The SF50's 1.84 nm final at EGHI met
        // a path at 585 ft AGL and the floor sat 115 ft on top of it
        // (2026-07-30). Close in, the path wins.
        let short_final = approach_leg_alt_ft(elev, tpa, -0.2 * M_PER_NM);
        let path_there = elev + 0.2 * crate::autoland::GLIDEPATH_FT_PER_NM;
        assert!(
            (short_final - path_there).abs() < 1.0,
            "close in the target IS the path, got {short_final} want {path_there}"
        );
        assert!(
            short_final < elev + HANDOFF_AGL_FT,
            "and it is allowed below the old flat floor"
        );

        // The EGHI geometry that produced the go-around: 1.84 nm final, field
        // at 41 ft, jet pattern. The handover must not be above the path.
        let (eghi, jet_tpa) = (41.0, 41.0 + 1184.0);
        let handover = approach_leg_alt_ft(eghi, jet_tpa, -1.84 * M_PER_NM);
        let eghi_path = eghi + 1.84 * crate::autoland::GLIDEPATH_FT_PER_NM;
        assert!(
            handover <= eghi_path + 1.0,
            "handed off {:.0} ft above the path",
            handover - eghi_path
        );
        // ...and never a CLIMB: far out, where the path is above the pattern
        // altitude, the leg simply holds what it has (the GA case — a 172's
        // 1,000 ft circuit is below the path at 4 nm).
        let ga_tpa = elev + 1000.0;
        assert_eq!(
            approach_leg_alt_ft(elev, ga_tpa, -4.0 * M_PER_NM),
            ga_tpa,
            "hold the pattern altitude, never climb on base"
        );
    }

    /// The descent has to START abeam the touchdown point, because base alone
    /// is not long enough to hold it.
    ///
    /// EGHI 2026-07-30: TPA 1,184 ft AGL, final turned at 1.84 nm where the
    /// path is 585 — 600 ft to lose in a 1.1 nm base, about 900 fpm. It
    /// managed 150, arrived 400 ft high, stayed high the whole final, crossed
    /// the threshold 70 ft up and went around. Splitting the descent gives
    /// base a job it can actually do.
    #[test]
    fn the_downwind_descent_leaves_base_a_reachable_job() {
        let (elev, tpa_agl) = (41.0, 1184.0);
        let tpa = elev + tpa_agl;
        let downwind_target = (elev + DOWNWIND_DESCENT_AGL_FT).min(tpa);

        // It descends, but stays a sane downwind height.
        assert!(downwind_target < tpa, "the 180 is where you start down");
        assert!(
            downwind_target - elev >= 700.0,
            "still a circuit height, not an approach: {:.0} ft AGL",
            downwind_target - elev
        );

        // What base is left with, against what it had before.
        let path_at_final = elev + 1.84 * crate::autoland::GLIDEPATH_FT_PER_NM;
        let was = tpa - path_at_final;
        let now = downwind_target - path_at_final;
        assert!(
            now < was * 0.75,
            "base's descent must shrink materially: {now:.0} ft vs {was:.0}"
        );

        // A 172's 1,000 ft circuit must not be pushed DOWN to 800 by this and
        // then find itself below its own pattern — the target is bounded by
        // the pattern altitude, so a low circuit is simply left alone.
        let ga_tpa = elev + 600.0;
        assert_eq!(
            (elev + DOWNWIND_DESCENT_AGL_FT).min(ga_tpa),
            ga_tpa,
            "never descend below a pattern that is already lower"
        );
    }

    /// The Southampton approach (2026-08-01), which is the whole reason the
    /// glidepath distance is no longer track miles.
    ///
    /// The Baron began base 3,324 m off the centreline. Track miles read 3.33
    /// nm and asked for 1,101 ft when the real distance to the threshold was
    /// 1.53 nm and the 3° path was at 529 — the target was inflated by exactly
    /// the lateral offset. The aircraft flew base and most of final level at
    /// 800 ft AGL, the target dropped below it only eleven seconds before the
    /// handoff, and autoland was handed an aircraft 206 ft high at 1.7 nm. It
    /// flew the whole final high, crossed the threshold 80 ft up at 102 knots
    /// and went around.
    #[test]
    fn the_glidepath_distance_is_down_the_course_not_across_it() {
        let (elev, tpa) = (41.0, 41.0 + 1000.0);
        let rollout = 4087.0; // geom.downwind_past_m, the live figure

        // The live base-entry geometry. Track miles (the old measure) put the
        // target above the pattern altitude itself; the real path is far
        // below. This is the bug, stated as a number.
        let (a, x): (f64, f64) = (-2840.0, 3324.0);
        let track = approach_leg_alt_ft(elev, tpa, a.abs() + x.abs());
        let along = approach_leg_alt_ft(elev, tpa, a);
        assert!(
            track - along > 500.0,
            "cross-track inflated the target by {:.0} ft",
            track - along
        );

        // ...but plain along-track is a dive to nowhere at that moment: the
        // aircraft is crossing the centreline, so its along-track distance is
        // still GROWING. Base aims at the roll-out instead.
        assert_eq!(path_dist_m(Leg::Base, a, rollout), rollout);
        let base_tgt = approach_leg_alt_ft(elev, tpa, path_dist_m(Leg::Base, a, rollout));
        assert!(
            (base_tgt - 744.0).abs() < 15.0,
            "base should aim at the path where the turn delivers us: {base_tgt:.0}"
        );

        // Flown, the roll-out came at 3,966 m — and there the target is the
        // path, not the flat 841 the old measure held.
        let rolled_out = approach_leg_alt_ft(elev, tpa, path_dist_m(Leg::Final, -3966.0, rollout));
        assert!(
            (rolled_out - 723.0).abs() < 15.0,
            "at roll-out the target is the path: {rolled_out:.0}"
        );
        assert!(rolled_out < 841.0, "and below what the old measure held");

        // Final follows the along-track distance down; at the handoff point
        // that is within a hair of what autoland will fly from.
        let handoff = approach_leg_alt_ft(elev, tpa, path_dist_m(Leg::Final, -3128.0, rollout));
        assert!(
            (handoff - 579.0).abs() < 15.0,
            "handoff on the path: {handoff:.0}"
        );

        // Still far out, the pattern altitude still caps it — a circuit does
        // not start down on the upwind.
        assert_eq!(approach_leg_alt_ft(elev, tpa, -6.0 * M_PER_NM), tpa);
    }

    /// The Southampton Baron (2026-08-01). It rolled into the crosswind turn
    /// at 127 kt and came out at 85 — one knot over its clean stall and one
    /// over Vmca — with the VS law still asking for 880 fpm and the stabilizer
    /// trimmed to the nose-up stop. The AP will keep buying rate with knots
    /// unless something stops it.
    #[test]
    fn a_climb_is_never_worth_more_than_the_airspeed_left() {
        // Baron 58: Vy 105, blue line 100, clean stall 84.
        let floor = climb_floor_kts(Some(100.0), Some(84.0));
        assert_eq!(floor, 100.0, "blue line outranks the stall margin");
        let want = 105.0;

        // At the pattern speed, the commanded rate is untouched.
        assert_eq!(climb_limit_fpm(880.0, 105.0, 0.0, want, floor), 880.0);
        assert_eq!(climb_limit_fpm(880.0, 130.0, 0.0, want, floor), 880.0);

        // At blue line and below, no climb at all — lower the nose, let the
        // throttle law get the speed back.
        assert_eq!(climb_limit_fpm(880.0, 100.0, 0.0, want, floor), 0.0);
        assert_eq!(
            climb_limit_fpm(880.0, 85.3, 0.0, want, floor),
            0.0,
            "the speed the live circuit actually reached"
        );

        // ...and in between, a fade rather than a switch.
        let mid = climb_limit_fpm(880.0, 102.5, 0.0, want, floor);
        assert!(mid > 0.0 && mid < 880.0, "{mid}");
        assert!(
            climb_limit_fpm(880.0, 104.0, 0.0, want, floor) > mid,
            "and it must be monotone in airspeed"
        );

        // DESCENTS are never touched: base and final descending is the point.
        assert_eq!(climb_limit_fpm(-500.0, 85.0, 0.0, want, floor), -500.0);

        // A profile with no reference speeds disables the protection outright
        // rather than inventing a floor.
        assert_eq!(climb_floor_kts(None, None), 0.0);
        assert_eq!(climb_limit_fpm(700.0, 40.0, 0.0, 74.0, 0.0), 700.0);

        // ...and it must not LIMIT-CYCLE. Live it did: 700 fpm to 0 in four
        // seconds as the speed dipped through the band, held shut while the
        // aircraft was already accelerating back, then slammed open — the
        // aeroplane porpoising thirty feet either way (2026-08-01). Judging
        // the speed we are HEADING FOR fixes both ends.
        //
        // Recovering at 2 kt/s from below the floor: give the climb back now,
        // not in five seconds' time.
        assert!(
            climb_limit_fpm(700.0, 99.0, 2.0, want, floor) > 0.0,
            "a recovering aircraft must not be held at zero climb"
        );
        // Decaying at 2 kt/s from above the pattern speed: start easing off
        // BEFORE the speed is lost, so the dip never gets deep.
        assert!(
            climb_limit_fpm(700.0, 112.0, -2.0, want, floor) < 700.0,
            "a decaying speed must ease the climb early"
        );
        // Steady on target is untouched either way.
        assert_eq!(climb_limit_fpm(700.0, 110.0, 0.0, want, floor), 700.0);

        // A 172 (Vy 74, stall 48) gets a floor of 55 — so a normally flown
        // trainer pattern never meets the limiter at all.
        let c172 = climb_floor_kts(None, Some(48.0));
        assert!((c172 - 55.2).abs() < 0.01, "{c172}");
        assert_eq!(climb_limit_fpm(700.0, 74.0, 0.0, 74.0, c172), 700.0);
    }

    /// The full-rate threshold is the speed the climb is FLOWN at, never the
    /// leg speed the throttle holds. The `want = 105.0` in the test above was
    /// always Vy; the call site passed the stall-floored pattern speed
    /// (109.2), and the four-knot gap parked the throttle's setpoint at the
    /// top of the fade band: full climb unreachable while climbing, cmd_vs
    /// porpoising 0-150 fpm, the Baron descending at 190 fpm at 670 ft AGL
    /// mid-climb-out and then a whole downwind hung 100 ft below pattern
    /// altitude (EGHI, 2026-08-02).
    #[test]
    fn full_climb_is_granted_at_the_speed_the_climb_is_flown_at() {
        // Baron 58, wired exactly as the live loop wires it: pattern 109.2
        // (1.3 x the 84 kt clean stall), Vy 105, floor 100 (blue line).
        let pattern = pattern_speed_kts(105.0, 95.0, Some(84.0));
        let floor = climb_floor_kts(Some(100.0), Some(84.0));
        let full = climb_full_rate_kts(pattern, 105.0, floor);
        assert_eq!(full, 105.0, "Vy, not the stall-floored pattern speed");

        // At Vy the climb is untouched — climbing at Vy is flying correctly.
        assert_eq!(climb_limit_fpm(700.0, 105.0, 0.0, full, floor), 700.0);
        // And at the throttle's own setpoint, with room to spare: the steady
        // operating point must sit ABOVE the fade band, not on its edge.
        assert!(
            pattern > full + 3.0,
            "no headroom left: {pattern} vs {full}"
        );
        assert_eq!(climb_limit_fpm(700.0, pattern, 0.0, full, floor), 700.0);
        // Genuine sag below Vy still fades, and blue line still shuts it.
        let sagged = climb_limit_fpm(700.0, 103.0, 0.0, full, floor);
        assert!(sagged > 0.0 && sagged < 700.0, "{sagged}");
        assert_eq!(climb_limit_fpm(700.0, 100.0, 0.0, full, floor), 0.0);

        // A jet's book Vy (a cruise-climb figure) must never RAISE the
        // threshold above the leg speed: the Citation's 300 stays capped at
        // its pattern speed.
        let citation = pattern_speed_kts(300.0, 132.0, Some(136.0));
        assert_eq!(climb_full_rate_kts(citation, 300.0, 156.4), citation);

        // A Vy at or under the floor still leaves a band, never a cliff.
        assert_eq!(climb_full_rate_kts(96.0, 94.0, 100.0), 102.0);
    }

    /// The pattern is flown CLEAN, so its speed must clear the CLEAN stall.
    /// The Baron's Vy is 105 against an 84 kt clean stall — 1.25 Vs, four
    /// knots above the airspeed caution band — so every normal variation on
    /// downwind, base and final dipped into it: six callouts in one circuit,
    /// every one correct (2026-08-01). Flap was 0.00 throughout, which is
    /// right; nothing knew that made the stall reference 84 and not 70.
    #[test]
    fn the_clean_pattern_clears_the_clean_stall() {
        // Baron 58: Vy 105, approach 95, clean stall 84.
        let baron = pattern_speed_kts(105.0, 95.0, Some(84.0));
        assert!(
            (baron - 109.2).abs() < 0.1,
            "1.3 x the clean stall: {baron:.1}"
        );
        // Clear of the caution band (1.2 Vs) with real margin, not four knots.
        assert!(baron - 1.2 * 84.0 > 7.0, "margin {:.1}", baron - 1.2 * 84.0);

        // A 172 (Vy 74, clean stall 48) is untouched — its Vy is already
        // 1.54 Vs, and this must never speed a trainer's circuit up.
        assert_eq!(pattern_speed_kts(74.0, 65.0, Some(48.0)), 74.0);
        // ...as is an MD-82 (Vy 200, stall 100) and a Citation (200, 136).
        assert_eq!(pattern_speed_kts(200.0, 140.0, Some(100.0)), 200.0);
        assert!(pattern_speed_kts(200.0, 132.0, Some(136.0)) <= 198.0);

        // No stall figure means no floor, never an invented one.
        assert_eq!(pattern_speed_kts(105.0, 95.0, None), 105.0);

        // The approach legs inherit it, so base and final are clear too.
        let app = approach_leg_kts(baron, 95.0);
        assert!(
            app - 1.2 * 84.0 > 7.0,
            "base/final margin {:.1}",
            app - 1.2 * 84.0
        );
    }

    /// Base and final are flown at an approach-setup speed, not the climb speed
    /// the rest of the pattern uses — but only ever SLOWER than the pattern.
    #[test]
    fn the_approach_legs_slow_toward_the_approach_speed() {
        // Jet: Vy 200, approach 140 → 160, comfortably clean, close enough for
        // autoland to take the last 20 kt off with flaps.
        assert_eq!(approach_leg_kts(200.0, 140.0), 160.0);
        // Trainer: Vy 74 is already below approach + margin, so nothing
        // changes — this must never speed a light aircraft UP on base.
        assert_eq!(approach_leg_kts(74.0, 65.0), 74.0);
        // A slow pattern speed is still never taken below the approach speed.
        assert_eq!(approach_leg_kts(60.0, 65.0), 65.0);
    }

    /// A proportional controller cannot track a ramp, and the glidepath is a
    /// ramp — so the final leg flew a permanent lag and handed autoland an
    /// aircraft it could not get down.
    ///
    /// This is the Malaga approach, simulated (2026-08-05). The circuit rolls
    /// out onto final 4,067 m from the threshold at 800 ft MSL, 54 ft above
    /// the path, closing at 110 kt, and flies the leg. The blackbox says what
    /// happened for real: the error GREW to 172 ft by the handoff, the
    /// commanded descent never once passed the old -500 clamp, and the
    /// commanded rate at the end (-406) was less than the rate the target
    /// itself was descending at (-510). The aircraft was not failing to close
    /// the gap; it was falling further behind every second.
    #[test]
    fn the_final_leg_stays_on_a_glidepath_it_is_flying_down() {
        let elev = 48.0;
        let tpa = elev + 1000.0;
        let closure = 110.0;
        // Where the roll-out actually put us, and the height it left us at.
        let mut along_m = 4067.0;
        let mut alt = elev + 752.0;
        let mut vs = -150.0;
        let dt = 1.0;
        let mut worst_high: f64 = 0.0;
        while along_m > 400.0 {
            let target = approach_leg_alt_ft(elev, tpa, along_m);
            let cmd = alt_capture_vs(target, path_descent_fpm(closure), alt, vs);
            // A VS-mode autopilot gets to its commanded rate quickly and then
            // holds it; a first-order lag is plenty to show the tracking.
            vs += (cmd - vs) * 0.4;
            alt += vs / 60.0 * dt;
            along_m -= closure * 1852.0 / 3600.0 * dt;
            worst_high = worst_high.max(alt - target);
        }
        assert!(
            worst_high < 50.0,
            "the leg must never get FURTHER above the path than it started: \
             {worst_high:.0} ft high"
        );
        // The handoff point itself — 1.5 nm out, where autoland took over
        // 174 ft high on the day.
        let at_handoff = alt - approach_leg_alt_ft(elev, tpa, along_m);
        assert!(
            at_handoff.abs() < 40.0,
            "autoland must inherit an aircraft on the path: {at_handoff:.0} ft off"
        );
        // And the reason the old law could not have done it even in principle:
        // its descent authority was less than the path's own rate.
        assert!(
            path_descent_fpm(closure) < -500.0,
            "a 3 degree path at approach speed descends faster than the old clamp"
        );
        // A standing target keeps exactly the authority it always had.
        assert_eq!(alt_capture_vs(0.0, 0.0, 1000.0, 0.0), -500.0);
    }

    /// The altitude capture leads on vertical speed, because a vertical speed
    /// does not stop when the command does. Luton 2026-07-28: pure proportional
    /// control climbed at 700 fpm right up to pattern altitude, sailed 100 ft
    /// through it, and hunted ±60 ft for the whole downwind.
    #[test]
    fn the_altitude_capture_levels_where_the_aircraft_will_arrive() {
        // Well below, stationary in the vertical: climb at the clamp.
        assert_eq!(alt_capture_vs(2000.0, 0.0, 1000.0, 0.0), 700.0);
        // Same error, but already climbing hard: the command backs off,
        // because most of that gap will be eaten before the aircraft responds.
        let climbing = alt_capture_vs(2000.0, 0.0, 1900.0, 700.0);
        let level = alt_capture_vs(2000.0, 0.0, 1900.0, 0.0);
        assert!(climbing < level, "{climbing} !< {level}");
        // Arriving at the target with rate on: command a descent to stop the
        // overshoot BEFORE it happens, which is the whole point.
        assert!(
            alt_capture_vs(2000.0, 0.0, 2000.0, 700.0) < 0.0,
            "must arrest an overshoot in progress"
        );
        // On target and steady: hands off.
        assert_eq!(alt_capture_vs(2000.0, 0.0, 2000.0, 0.0), 0.0);
    }

    /// A jet flies a jet's circuit: 1,500 ft, because its wider pattern turns
    /// final where a 3° path is well above a trainer's 1,000 ft.
    #[test]
    fn the_pattern_altitude_follows_the_airframe() {
        use crate::tools::circuit::pattern_alt_agl_ft;
        assert_eq!(pattern_alt_agl_ft(65.0), 1000.0, "trainer");
        assert_eq!(pattern_alt_agl_ft(90.0), 1000.0, "fast single");
        assert_eq!(pattern_alt_agl_ft(140.0), 1500.0, "jet");
    }

    #[test]
    fn left_pattern_turns_ninety_degrees_left_each_leg() {
        // Runway 27 (course 270), standard left-hand pattern.
        assert_eq!(leg_heading(270.0, Leg::Upwind, true), 270.0);
        assert_eq!(leg_heading(270.0, Leg::Crosswind, true), 180.0);
        assert_eq!(leg_heading(270.0, Leg::Downwind, true), 90.0);
        assert_eq!(leg_heading(270.0, Leg::Base, true), 0.0);
        // Final comes all the way back to the runway heading.
        assert_eq!(leg_heading(270.0, Leg::Final, true), 270.0);
    }

    #[test]
    fn right_pattern_turns_the_other_way_and_wraps() {
        // Runway 09 (course 90), right-hand pattern, checking the 360 wrap.
        assert_eq!(leg_heading(90.0, Leg::Crosswind, false), 180.0);
        assert_eq!(leg_heading(90.0, Leg::Downwind, false), 270.0);
        assert_eq!(leg_heading(90.0, Leg::Base, false), 0.0); // 90+270 = 360 -> 0
        assert_eq!(leg_heading(90.0, Leg::Final, false), 90.0);
    }

    #[test]
    fn the_downwind_is_always_the_reciprocal() {
        for course in [0.0_f64, 27.0, 90.0, 200.0, 359.0] {
            let recip = (course + 180.0).rem_euclid(360.0);
            assert!((leg_heading(course, Leg::Downwind, true) - recip).abs() < 1e-9);
            assert!((leg_heading(course, Leg::Downwind, false) - recip).abs() < 1e-9);
        }
    }

    // Position-based sequencing. A left pattern on a 1500 m runway.
    // In the runway frame: a_m along the takeoff direction, x_m right-positive;
    // a left pattern sits at NEGATIVE x_m.
    const L: f64 = 1500.0;

    /// A 172-ish geometry: ~80 kt pattern speed.
    fn g() -> Geometry {
        Geometry::for_radius(turn_radius_m(41.0))
    }

    #[test]
    fn the_bug_slews_the_short_way_across_the_north_seam() {
        // The killer case: right pattern's crosswind, dial 271.5 -> target 2.6.
        // Every step must move CLOCKWISE (increasing, mod 360) — the single
        // jump read as a counterclockwise spin and the AP turned 270° the
        // wrong way (2026-07-24).
        let mut bug = 271.5;
        let mut steps = 0;
        while (slew_bug(bug, 2.6, 40.0) - 2.6).abs() > 1e-9 || (bug - 2.6).abs() > 1e-9 {
            let next = slew_bug(bug, 2.6, 40.0);
            let d = (next - bug + 540.0).rem_euclid(360.0) - 180.0;
            assert!(d > 0.0, "always clockwise: {bug} -> {next}");
            assert!(d <= 40.0 + 1e-9, "never more than a step");
            bug = next;
            steps += 1;
            assert!(steps < 10, "converges");
        }
        // Counterclockwise across the seam works too: 2.6 -> 330 is a 32.6°
        // short-way step, done in one write; a farther target clamps.
        assert!((slew_bug(2.6, 330.0, 40.0) - 330.0).abs() < 1e-9);
        assert!(
            (slew_bug(2.6, 280.0, 40.0) - 322.6).abs() < 1e-9,
            "clamped first step"
        );
        // Small deltas go direct.
        assert!((slew_bug(100.0, 110.0, 40.0) - 110.0).abs() < 1e-9);
    }

    #[test]
    fn geometry_scales_from_the_turn_radius() {
        // ~80 kt (41 m/s) at 14° bank → radius ~700 m. A standard-rate 20° was
        // tried and reverted: the tighter pattern degenerated into a figure of
        // eight because the legs got shorter than the turns. The pattern being
        // WIDE is handled by the handover gate below, not by shrinking it.
        let r = turn_radius_m(41.0);
        assert!((550.0..900.0).contains(&r), "radius {r}");
        let geom = Geometry::for_radius(r);
        // The width must exceed the turn DIAMETER — legs shorter than the
        // diameter are what degenerated the rectangle into loops.
        assert!(
            geom.width_m > 2.0 * r,
            "width {} vs diameter {}",
            geom.width_m,
            2.0 * r
        );
        assert!(geom.downwind_past_m > geom.width_m, "room for base + final");
        assert!(geom.lead_m < r, "lead is less than the radius itself");
        // The pattern must be INTERCEPTABLE: after the base turn eats one
        // radius of the offset, what is left has to be closeable in the
        // distance that remains. A geometry that fails this cannot be flown
        // onto the centreline however well the laws behave — EGHI handed
        // autoland 2,815 m of offset with 2,421 m to run and it crossed the
        // threshold 188 m off the paint.
        let rollout_xtk = (geom.final_capture_m - r).max(0.0);
        assert!(
            interceptable(rollout_xtk, geom.downwind_past_m.min(2.0 * r + 1200.0)),
            "base rolls out {rollout_xtk:.0} m off — must be closeable"
        );
        // A bad speed read can't produce a degenerate pattern.
        assert!(turn_radius_m(0.0) >= 250.0);
        assert!(turn_radius_m(500.0) <= 2500.0);
    }

    #[test]
    fn upwind_turns_crosswind_past_the_end_while_climbing() {
        // Not yet past the departure end: hold, whatever the altitude.
        assert!(!pattern_advance(Leg::Upwind, L - 100.0, 0.0, &g(), L, true));
        // Past the end + margin: turn crosswind (the altitude no longer gates —
        // a real pattern climbs to TPA on the crosswind/downwind).
        assert!(pattern_advance(
            Leg::Upwind,
            L + g().upwind_past_m + 1.0,
            0.0,
            &g(),
            L,
            true
        ));
    }

    #[test]
    fn crosswind_turns_downwind_with_turn_anticipation() {
        let geom = g();
        // Still inside the led trigger -> hold.
        let before = geom.width_m - geom.lead_m - 50.0;
        assert!(!pattern_advance(Leg::Crosswind, L, -before, &geom, L, true));
        // Reached width - lead -> turn NOW, so the rollout lands on the line.
        let at = geom.width_m - geom.lead_m + 1.0;
        assert!(pattern_advance(Leg::Crosswind, L, -at, &geom, L, true));
        // Right pattern mirrors: the spacing is to the RIGHT (positive x).
        assert!(pattern_advance(Leg::Crosswind, L, at, &geom, L, false));
        assert!(!pattern_advance(Leg::Crosswind, L, -at, &geom, L, false));
    }

    #[test]
    fn downwind_turns_base_past_the_landing_threshold() {
        let geom = g();
        // Still abeam / not far enough back -> hold.
        assert!(!pattern_advance(
            Leg::Downwind,
            200.0,
            -geom.width_m,
            &geom,
            L,
            true
        ));
        // Past the threshold by the led base distance -> turn base.
        let at = geom.downwind_past_m - geom.lead_m + 1.0;
        assert!(pattern_advance(
            Leg::Downwind,
            -at,
            -geom.width_m,
            &geom,
            L,
            true
        ));
    }

    #[test]
    fn base_turns_final_one_radius_from_the_centreline() {
        let geom = g();
        // Still well left of the capture line -> hold.
        assert!(!pattern_advance(
            Leg::Base,
            -geom.downwind_past_m,
            -(geom.final_capture_m + 200.0),
            &geom,
            L,
            true
        ));
        // Within one radius: the 90° turn rolls out on the centreline.
        assert!(pattern_advance(
            Leg::Base,
            -geom.downwind_past_m,
            -(geom.final_capture_m - 50.0),
            &geom,
            L,
            true
        ));
    }

    fn facts(terrain: Option<f64>, air: Option<&str>) -> SideFacts {
        SideFacts {
            terrain_max_ft: terrain,
            airspace: air.map(|s| s.to_string()),
        }
    }

    #[test]
    fn terrain_pick_switches_sides_and_raises_when_boxed_in() {
        // The Gibraltar case: left downwind crosses the Rock (1,400 ft), right
        // side is the flat bay — a default-left ask switches right at TPA.
        let (left, tpa, why) = choose_pattern(
            true,
            &facts(Some(1400.0), None),
            &facts(Some(50.0), None),
            11.0,
            1000.0,
        );
        assert!(!left, "switched off the Rock side");
        assert!((tpa - 1011.0).abs() < 1.0);
        assert_eq!(why, "switched_side_for_terrain");
        // Clear both sides: the asked-for side stands.
        let (left, _, why) = choose_pattern(
            true,
            &facts(Some(30.0), None),
            &facts(Some(20.0), None),
            11.0,
            1000.0,
        );
        assert!(left);
        assert_eq!(why, "default_side_clear");
        // Boxed in (a valley field): fly the LOWER side, high enough to clear.
        let (left, tpa, why) = choose_pattern(
            true,
            &facts(Some(2000.0), None),
            &facts(Some(1500.0), None),
            500.0,
            1000.0,
        );
        assert!(!left, "the lower side");
        assert!(
            (tpa - 2500.0).abs() < 1.0,
            "raised to terrain + pattern height: {tpa}"
        );
        assert_eq!(why, "raised_for_terrain");
        // No probe data: default side, default height — the old behavior.
        let (left, tpa, why) =
            choose_pattern(true, &facts(None, None), &facts(None, None), 11.0, 1000.0);
        assert!(left);
        assert!((tpa - 1011.0).abs() < 1.0);
        assert_eq!(why, "default_side_clear");
    }

    #[test]
    fn airspace_fouls_a_side_like_terrain_does() {
        // Default side flat but inside R-164: switch to the other flat side.
        let (left, tpa, why) = choose_pattern(
            true,
            &facts(Some(20.0), Some("restricted area R 164")),
            &facts(Some(30.0), None),
            11.0,
            1000.0,
        );
        assert!(!left, "switched away from the restricted side");
        assert!((tpa - 1011.0).abs() < 1.0);
        assert_eq!(why, "switched_side_for_airspace");
        // Other side blocked by airspace, default side merely high terrain:
        // stay on the default side, raised over its terrain.
        let (left, tpa, why) = choose_pattern(
            true,
            &facts(Some(1400.0), None),
            &facts(Some(20.0), Some("restricted area R 164")),
            11.0,
            1000.0,
        );
        assert!(left, "the airspace-free side, even with terrain");
        assert!((tpa - 2400.0).abs() < 1.0, "raised over the Rock: {tpa}");
        assert_eq!(why, "raised_for_terrain");
        // Both sides inside restricted airspace: terrain safety alone picks
        // the lower side, honestly labeled.
        let (left, _, why) = choose_pattern(
            true,
            &facts(Some(1400.0), Some("restricted area R 164")),
            &facts(Some(50.0), Some("restricted area R 164")),
            11.0,
            1000.0,
        );
        assert!(!left, "the lower-terrain side");
        assert_eq!(why, "airspace_unavoidable");
    }

    /// Real-install dry run of the LXGB 27 side pick against the actual
    /// airspace file (terrain unknown — the probe needs the sim):
    /// `cargo test -- --ignored real_install_lxgb_side_pick --nocapture`
    #[test]
    #[ignore]
    fn real_install_lxgb_side_pick() {
        let apt =
            "/Users/duanelicudi/X-Plane 12/Global Scenery/Global Airports/Earth nav data/apt.dat";
        let air = "/Users/duanelicudi/X-Plane 12/Custom Data/airspaces/airspace.txt";
        let cands = nearby_runways(apt, 36.1515, -5.3412, 2.0).expect("apt.dat");
        let rwy = cands
            .iter()
            .find(|r| r.id == "27")
            .expect("LXGB 27")
            .clone();
        let geom = Geometry::for_radius(turn_radius_m(41.0));
        let poly_l = pattern_polyline(&rwy, &geom, rwy.length_m, true);
        let poly_r = pattern_polyline(&rwy, &geom, rwy.length_m, false);
        let spaces = crate::airspace::load_near(air, 36.1515, -5.3412, 0.5);
        println!("no-go volumes near LXGB: {}", spaces.len());
        let tpa = 11.0 + 1000.0;
        let l_air = crate::airspace::first_conflict(&spaces, &poly_l, tpa).map(|a| a.spoken_name());
        let r_air = crate::airspace::first_conflict(&spaces, &poly_r, tpa).map(|a| a.spoken_name());
        println!("left (south, Rock side) airspace: {l_air:?}");
        println!("right (north, bay side) airspace: {r_air:?}");
        // Terrain from the real probe values seen live: Rock ~1,400 under the
        // southern downwind, flat under the northern.
        let l = SideFacts {
            terrain_max_ft: Some(1400.0),
            airspace: l_air,
        };
        let r = SideFacts {
            terrain_max_ft: Some(60.0),
            airspace: r_air,
        };
        let (left, tpa_msl, why) = choose_pattern(true, &l, &r, 11.0, 1000.0);
        println!("DECISION: left={left} tpa={tpa_msl:.0} why={why}");
        assert!(
            !left || tpa_msl > 1900.0,
            "never a 1,000 ft pattern over the Rock"
        );
    }

    #[test]
    fn spells_runways_for_the_radio() {
        assert_eq!(spell_runway("27"), "two seven");
        assert_eq!(spell_runway("09L"), "zero nine left");
        assert_eq!(spell_runway("36R"), "three six right");
        assert_eq!(spell_runway("6"), "six");
        assert_eq!(spell_runway("13C"), "one three center");
    }

    #[test]
    fn crew_radio_queue_is_fifo_and_drains_once() {
        let _ = take_crew_radio(); // clear any residue
        transmit("Downwind runway two seven".to_string());
        transmit("Final runway two seven".to_string());
        assert_eq!(
            take_crew_radio(),
            vec![
                "Downwind runway two seven".to_string(),
                "Final runway two seven".to_string()
            ]
        );
        assert!(take_crew_radio().is_empty(), "a second drain is empty");
    }

    #[test]
    fn final_is_the_end_of_the_sequence() {
        assert_eq!(next_leg(Leg::Base), Some(Leg::Final));
        assert_eq!(next_leg(Leg::Final), None);
        // Final never self-advances (autoland owns it), whatever the position.
        assert!(!pattern_advance(Leg::Final, -800.0, 0.0, &g(), L, true));
    }

    fn lxgb_rwy() -> crate::autoland::Runway {
        crate::autoland::Runway {
            icao: "LXGB".into(),
            id: "27".into(),
            thr_lat: 36.1508,
            thr_lon: -5.3385,
            course_deg: 267.5,
            length_m: 1680.0,
            surface: 1,
            dist_nm: 0.0,
            elev_ft: 11.0,
        }
    }

    #[test]
    fn gibraltar_loops_south_over_the_sea_by_runway() {
        // 09 (east): sea is to the right → right-hand (clockwise), left = false.
        assert!(!sea_side_is_left(87.5));
        assert!(!sea_side_is_left(90.0));
        // 27 (west): sea is to the left → left-hand (anticlockwise), left = true.
        assert!(sea_side_is_left(267.5));
        assert!(sea_side_is_left(270.0));
    }

    #[test]
    fn authored_circuit_only_at_lxgb() {
        let mut r = lxgb_rwy();
        assert!(authored_circuit(&r, &g(), 11.0, 1074.0, true).is_some());
        r.icao = "EGLL".into();
        assert!(authored_circuit(&r, &g(), 11.0, 1074.0, true).is_none());
    }

    #[test]
    fn authored_circuit_stays_on_the_bay_side_and_ends_on_the_centreline() {
        let route = authored_circuit(&lxgb_rwy(), &g(), 11.0, 1074.0, true).unwrap();
        // Left pattern → every offset is on the left (x_m <= 0), never crossing
        // to the Rock side.
        assert!(
            route.iter().all(|w| w.x_m <= 0.1),
            "all waypoints on the picked side"
        );
        // Upwind starts on the centreline, the last waypoint returns to it for
        // the autoland handoff.
        assert!((route.first().unwrap().x_m).abs() < 0.1);
        assert!((route.last().unwrap().x_m).abs() < 0.1);
        // The downwind is held wider than the base — the "around the Rock" bulge.
        let max_out = route.iter().map(|w| w.x_m.abs()).fold(0.0, f64::max);
        assert!(
            max_out > g().width_m,
            "downwind carried wide, {max_out} vs {}",
            g().width_m
        );
        // Descends from TPA to the handoff height by the end.
        assert!(route.first().unwrap().alt_msl_ft > route.last().unwrap().alt_msl_ft);
    }

    #[test]
    fn authored_circuit_mirrors_for_a_right_pattern() {
        let route = authored_circuit(&lxgb_rwy(), &g(), 11.0, 1074.0, false).unwrap();
        assert!(
            route.iter().all(|w| w.x_m >= -0.1),
            "right pattern → offsets to the right"
        );
    }

    /// The handover must wait for a geometry autoland can actually fly.
    ///
    /// EGHI 2026-07-30: released 2,815 m off the centreline with 2,421 m to
    /// run — the offset EXCEEDED the distance, a 49 degree angle. Autoland
    /// saturated at its 35 degree cut for the entire final and crossed the
    /// threshold 188 m off the paint. Reaching `final_capture_m` means the
    /// base turn may BEGIN, not that the aircraft is on the centreline.
    #[test]
    fn the_handover_waits_until_the_centreline_is_reachable() {
        // The exact geometry that failed.
        assert!(
            !interceptable(2815.0, 2421.0),
            "offset larger than the distance to run is never flyable"
        );
        // Sign must not matter — the other side of the centreline is the same
        // problem.
        assert!(!interceptable(-2815.0, 2421.0));

        // A rollout close to the centreline with room to run is fine.
        assert!(interceptable(400.0, 2000.0));
        assert!(interceptable(-400.0, 2000.0));

        // The boundary behaves: at the bare 35 degree geometry it is refused
        // (we insist on margin), and it is accepted once the margin is there.
        let bare = 1000.0 / MAX_INTERCEPT_CUT_DEG.to_radians().tan();
        assert!(
            !interceptable(1000.0, bare * 1.05),
            "bare geometry is not enough"
        );
        assert!(interceptable(1000.0, bare * INTERCEPT_MARGIN + 1.0));

        // On the centreline anything works, including at the threshold.
        assert!(interceptable(0.0, 0.0));
    }

    /// Being ABLE to close the offset is not being lined up, and the circuit
    /// has no intercept of its own — its final leg commands the runway heading
    /// and nothing else. So the handover waits for our own turn to roll out.
    #[test]
    fn the_handover_waits_for_the_turn_to_roll_out_not_just_for_flyable_geometry() {
        // EGHI 2026-08-01, the Baron, taken straight from the blackbox: on
        // base at 290 true, runway 02 (course 20), 1,813 m off the centreline
        // with 4,092 m to run. The geometry is a 28 degree cut, so
        // `interceptable` is happy...
        let hdg_err_on_base = ((290.0 - 20.0 + 540.0_f64).rem_euclid(360.0)) - 180.0;
        assert!(
            interceptable(1813.0, 4092.0),
            "a 28 degree cut is closeable"
        );
        // ...but the aircraft is still 90 degrees across the centreline, and
        // handing it over there is what made it approach and land at an angle.
        assert!(
            !ready_for_autoland(1813.0, 4092.0, hdg_err_on_base),
            "still turning: fly the turn, do not hand over the diagonal"
        );

        // Rolled out on the runway course with the designed 0.3-radii trim
        // left: this is the straight-in autoland is for.
        assert!(ready_for_autoland(250.0, 3500.0, 3.0));
        assert!(ready_for_autoland(-250.0, 3500.0, -3.0));
        // Just outside the rollout tolerance, still turning.
        assert!(!ready_for_autoland(250.0, 3500.0, 25.0));

        // The live failure (2026-08-01): rolled out on HEADING — eight
        // degrees, comfortably inside the tolerance — and 776 m to the side,
        // flying parallel. The heading test alone passed it, autoland flew the
        // diagonal, and the aircraft crossed the threshold still six degrees
        // off the runway, still turning. Rolled out is not established.
        assert!(
            interceptable(776.0, 3128.0),
            "the geometry was closeable, which is exactly why heading alone let it through"
        );
        assert!(
            !ready_for_autoland(776.0, 3128.0, 8.0),
            "parallel and 776 m off is not established"
        );

        // The last-ditch valve: out of room and still not aligned, autoland's
        // intercept beats a pattern leg that has nowhere left to go.
        assert!(ready_for_autoland(300.0, 1200.0, 40.0));
        // ...but never something autoland cannot fly at all, however close.
        assert!(!ready_for_autoland(2815.0, 1200.0, 0.0));

        // A failed heading read must not read as aligned — the caller passes
        // 180, the worst case, so the handover simply waits a tick.
        assert!(!ready_for_autoland(1813.0, 4092.0, 180.0));
    }

    /// Where the 776 m came from, in arithmetic. The pattern radius estimate
    /// pads the speed by 1.1 and banks a deliberately shallow 14 degrees —
    /// both of which make it BIGGER, which is safe for every dimension it
    /// sizes except this one, where a bigger capture distance starts the turn
    /// earlier and rolls out short of the centreline.
    #[test]
    fn the_final_capture_is_sized_on_the_turn_actually_flown() {
        // Southampton, the Baron: Vy 105, approach leg also 105.
        let pattern_r = turn_radius_m(105.0 * 0.514_444 * 1.1);
        assert!(
            (pattern_r - 1443.0).abs() < 20.0,
            "the pattern estimate: {pattern_r:.0}"
        );
        // A 90-degree turn closes exactly one radius of cross-track, and the
        // aircraft closed 1,048 m (1,830 off at the turn, 782 at the handoff).
        let flown_r = 1048.0;
        let old_residual = 1.3 * pattern_r - flown_r;
        assert!(
            (old_residual - 776.0).abs() < 100.0,
            "the old model predicts the live offset: {old_residual:.0} m vs 776 flown"
        );

        // Sized on the turn actually flown, the roll-out lands near enough to
        // the centreline that the final leg's intercept is trimming, not
        // rescuing — and inside what the handoff gate will accept once the
        // intercept has had a few seconds.
        let geom = Geometry::for_radii(pattern_r, final_turn_radius_m(105.0));
        let residual = geom.final_capture_m - flown_r;
        assert!(
            (0.0..=ESTABLISHED_XTK_M + 100.0).contains(&residual),
            "roll out just short of the line, not three quarters of a radius: {residual:.0} m"
        );
        // Still EARLY, never late: with an intercept, short is a gentle
        // closure and long is an S-turn back across the centreline.
        assert!(residual > 0.0, "must not turn late");

        // The pattern itself is untouched — width and downwind length keep the
        // conservative radius that stopped the LXGB figure-eights.
        let plain = Geometry::for_radius(pattern_r);
        assert_eq!(geom.width_m, plain.width_m);
        assert_eq!(geom.downwind_past_m, plain.downwind_past_m);
        assert!(geom.final_capture_m < plain.final_capture_m);
    }

    /// The final leg has to CONVERGE on the centreline. It used to command the
    /// runway course and nothing else, which makes it a leg that flies
    /// parallel to the line it is supposed to be joining — survivable only
    /// while the turn onto final was assumed to roll out ON the line, and it
    /// does not: at Southampton the turn began 1,850 m out, consumed only
    /// ~1,050, and left the aircraft 776 m to the side holding runway heading
    /// all the way to the handoff (2026-08-01).
    #[test]
    fn the_final_leg_flies_an_intercept_not_a_parallel() {
        let course = 19.4;
        let cut =
            |x: f64| ((final_intercept_hdg(course, x) - course + 540.0).rem_euclid(360.0)) - 180.0;

        // On the line, fly the line.
        assert!(cut(0.0).abs() < 0.01);

        // Off to the RIGHT of the course, cut LEFT to close it, and the other
        // way about. Getting this backwards flies away from the runway.
        assert!(cut(776.0) < 0.0, "right of course must cut left");
        assert!(cut(-776.0) > 0.0, "left of course must cut right");

        // The live offset earns a real intercept, not a token one.
        assert!(
            cut(776.0).abs() > 15.0,
            "776 m off deserves a proper cut, got {:.1}",
            cut(776.0)
        );

        // It fades as the offset closes — no overshoot through the centreline.
        assert!(cut(300.0).abs() < cut(776.0).abs());
        assert!(cut(80.0).abs() < cut(300.0).abs());
        assert!(cut(50.0).abs() < 5.0, "nearly on: nearly no cut");

        // ...and is capped, so a wild offset still hands over something
        // autoland's own 35-degree pursuit can fly.
        assert!(cut(9000.0).abs() <= MAX_FINAL_CUT_DEG + 0.01);
        assert!(MAX_FINAL_CUT_DEG < MAX_INTERCEPT_CUT_DEG);

        // The intercept must be able to REACH the handoff gate, or the gate
        // deadlocks into the last-ditch valve. From the live roll-out (1,830 m
        // off, 3,966 m to run) a capped cut closes at sin(30) of groundspeed
        // while the along-track closes at cos(30): check the offset is gone
        // before the runway is.
        let (mut x, mut along) = (1830.0_f64, 3966.0_f64);
        let step_m = 54.0; // ~105 kt, one second
        let mut ticks = 0;
        while x > ESTABLISHED_XTK_M && along > 0.0 && ticks < 600 {
            let c = cut(x).abs().to_radians();
            x -= step_m * c.sin();
            along -= step_m * c.cos();
            ticks += 1;
        }
        assert!(
            x <= ESTABLISHED_XTK_M && along > 900.0,
            "must be established with room to spare: {x:.0} m off, {along:.0} m to run"
        );
    }
}
