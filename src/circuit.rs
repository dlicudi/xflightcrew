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

// The tests reach these through `use super::*` (the queue that used them
// moved to `crew_radio.rs`).
use crate::autoflight::{self, Button, Dial};
#[cfg_attr(not(test), allow(unused_imports))]
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
#[cfg_attr(not(test), allow(unused_imports))]
use std::sync::Mutex;

use crate::autoland::{along_track_nm, cross_track_nm, nearby_runways, true_to_mag};
use crate::autotakeoff;
use crate::heading_bug::{bug_toward, slew_bug, BugDeadband, BugHold};
use crate::tools::autopilot::ApConfig;
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

const LAT: &str = "sim/flightmodel/position/latitude";
const LON: &str = "sim/flightmodel/position/longitude";
const PSI_TRUE: &str = "sim/flightmodel/position/psi";
const HPATH: &str = "sim/flightmodel/position/hpath";
/// Bank angle, degrees, right wing down positive — the authored follower
/// reads it to know which way the AP is turning (see `bug_toward`).
const PHI: &str = "sim/flightmodel/position/phi";
const AGL_M: &str = "sim/flightmodel/position/y_agl";
const MAG_VAR: &str = "sim/flightmodel/position/magnetic_variation";
// AP surface (mirror of autotakeoff.rs — the recipe PROVEN to climb and
// steer; `sim/autopilot/altitude_hold` is deliberately absent: it holds the
// CURRENT altitude, not the dial, and locked every pattern at handoff height
// (2026-07-24)).
const HDG_STATUS: &str = autoflight::names::HDG_STATUS;
const SERVOS_ON: &str = sim::datarefs::SERVOS_ON;
const VS_STATUS: &str = autoflight::names::VS_STATUS;
const IAS: &str = "sim/flightmodel/position/indicated_airspeed";
/// Gear handle — the circuit lowers it turning base (autoland's own
/// extension only fires inside 6 nm, which this hands over well inside of).
const GEAR_HANDLE: &str = "sim/cockpit2/controls/gear_handle_down";
/// Flap handle: base and final extend the landing flaps by the placard
/// gates as the speed allows, the same law autoland runs inside.
const FLAP_RATIO: &str = "sim/cockpit2/controls/flap_ratio";
/// The flap the base leg configures toward (the handle ratio): about half
/// travel, the approach setting on every airframe here; final goes to full.
const BASE_FLAP_RATIO: f64 = 0.5;
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

/// A heading error beyond this is a turn the autopilot flies at its own bank
/// limit, degrees: the samples [`BankWatch`] keeps.
const BANK_SAMPLE_ERR_DEG: f64 = 20.0;
/// ...of which it needs this many before it says anything.
const BANK_MIN_SAMPLES: usize = 4;
/// The measured bank is rounded shallow by this, as `AP_BANK_FINAL_DEG` was
/// (15.8 measured, 15.5 used): the turn should still err EARLY.
const MEASURED_BANK_SHALLOW: f64 = 0.95;

/// The bank the autopilot flew through the turn onto BASE, measured from the
/// roll angle while the heading was still well off the leg's.
///
/// `AP_BANK_FINAL_DEG` is a Baron's autopilot (15.8 degrees back-solved at
/// Southampton), and the 737's banks 25: LEMG 13 (sortie
/// 1790591256-full_auto, 2026-09-28) the base turn held 23 to 27 degrees,
/// the capture was 1.3 x the 2,500 m radius clamp = 3,250 m where the
/// aircraft turns in about 1,800, the turn onto final began 3.1 km out and
/// rolled onto the 30 degree intercept 1.3 km short of the centreline, and
/// the final leg was still closing at 12 degrees, 111 m off, when it let go
/// at 1.9 nm.
#[derive(Debug, Default, Clone)]
pub struct BankWatch {
    samples: Vec<f64>,
}

impl BankWatch {
    /// One tick on base: the bank read and the heading error to the leg.
    pub fn note(&mut self, phi_deg: f64, hdg_err_deg: f64) {
        let err = crate::angles::wrap180(hdg_err_deg);
        if phi_deg.is_finite() && err.abs() > BANK_SAMPLE_ERR_DEG && phi_deg.abs() > 5.0 {
            self.samples.push(phi_deg.abs());
        }
    }

    /// The steady bank of that turn (the median of the samples), once there
    /// are enough of them.
    pub fn steady_bank_deg(&self) -> Option<f64> {
        if self.samples.len() < BANK_MIN_SAMPLES {
            return None;
        }
        let mut s = self.samples.clone();
        s.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        Some(s[s.len() / 2])
    }
}

/// The base-to-final capture sized on the bank the autopilot really flies
/// (`bank_deg`, from [`BankWatch`]) at the base leg's airspeed: the same
/// 1.3 radii [`Geometry::for_radii`] keeps, on the measured radius. Never a
/// bank shallower than the pattern's own 14 degrees or steeper than 30.
/// `None` without a usable airspeed.
pub fn final_capture_on_bank(ias_kts: f64, bank_deg: f64) -> Option<f64> {
    if !(ias_kts.is_finite() && ias_kts > 40.0 && bank_deg.is_finite()) {
        return None;
    }
    let bank = (bank_deg * MEASURED_BANK_SHALLOW).clamp(AP_BANK_DEG, 30.0);
    Some((1.3 * turn_radius_at(ias_kts * 0.514_444, bank)).max(250.0))
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
    #[cfg(test)]
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
///
/// And established is judged by what autoland will make of it. 300 m and 12
/// degrees let go of a 737 at 1.9 nm still 111 m left and closing at 12
/// degrees (LEMG 13, sortie 1790591256-full_auto, 2026-09-28): autoland's
/// pursuit is a thirty-second law, it had 46 s of final, and the time model
/// it is gated on says 24 m at the threshold on a runway 45 m wide. It flew
/// the final 30 to 50 m off and landed 37 m right, off the pavement. So the
/// handoff, on a runway of known width (`limit_m`, its landing limit,
/// [`crate::autoland::landing_xtk_limit_m`]), waits until the aircraft is ON
/// the line: within [`ON_LINE_LIMITS`] landing limits of it, tracking within
/// [`ON_LINE_TRACK_DEG`] of the course, and with the offset autoland's own
/// pursuit leaves at the threshold ([`crate::reversal::predicted_residual_m`]
/// at `gs_kts`) inside the limit. The circuit's intercept closes on a 900 m
/// aim, three times quicker than autoland's, and keeps flying it until then.
/// `None`: the old test alone.
///
/// And ON THE PATH, not only on the line (#188): `above_path_ft`, the height
/// over the circuit's own glidepath at this distance ([`above_path_ft`]),
/// must be inside [`HANDOFF_ABOVE_PATH_FT`], the last-ditch valve included.
/// LEMG 13 SR22 (sortie 1790697583-full_auto, 2026-09-29): handed over on
/// the line at 1.51 nm but 545 ft above autoland's path, which went around at
/// once (`late_go_around`). The final keeps flying its own descent until then
/// (see [`final_high_give_up`]). Below the path is autoland's to level out.
/// `None`: not judged (a failed altitude read).
pub fn ready_for_autoland(
    xtk_m: f64,
    along_m: f64,
    hdg_err_deg: f64,
    gs_kts: f64,
    limit_m: Option<f64>,
    above_path_ft: Option<f64>,
) -> bool {
    if above_path_ft.is_some_and(|h| h > HANDOFF_ABOVE_PATH_FT) {
        return false;
    }
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
    let rolled_out = hdg_err_deg.abs() <= ROLLED_OUT_HDG_DEG && xtk_m.abs() <= ESTABLISHED_XTK_M;
    match limit_m {
        None => rolled_out,
        Some(limit) => {
            rolled_out
                && hdg_err_deg.abs() <= ON_LINE_TRACK_DEG
                && xtk_m.abs() <= ON_LINE_LIMITS * limit
                && handoff_residual_m(xtk_m, along_m, gs_kts) <= limit
        }
    }
}

/// ON the line, for a handoff on a runway of known width: within this many
/// landing limits of the centreline (45 m on LEMG's 45 m strip, about half a
/// dot of localiser three miles out)...
const ON_LINE_LIMITS: f64 = 3.0;
/// ...and tracking within this of the course: rolled out, not still turning
/// off the intercept (a handoff mid-roll-out teaches the final's crab filter
/// and track trim the turn).
const ON_LINE_TRACK_DEG: f64 = 5.0;

/// The offset autoland's final pursuit will still carry at the threshold if
/// it takes the aircraft here, metres (its own time model).
pub fn handoff_residual_m(xtk_m: f64, along_m: f64, gs_kts: f64) -> f64 {
    crate::reversal::predicted_residual_m(xtk_m / M_PER_NM, along_m.abs() / M_PER_NM, gs_kts)
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

/// How far ahead the final leg leads the offset it pursues, s: the pursuit
/// aims from where the aircraft's ground track will have carried it this
/// long from now, not from where it is.
///
/// Pure pursuit on the offset alone is a lag-free law, and an autopilot's
/// heading answer is not lag-free. The A330's rolls toward its bug at about
/// 3 degrees a second through a 2 s lag and flies each bug two ticks late
/// (fitted to the LEMG 30 circuits of 2026-09-29, sorties 1790710769 and
/// 1790698132-full_auto, 1.0 to 1.4 degrees rms over each final): its
/// heading sat 12 to 14 degrees behind a bug already back on the course, it
/// crossed the centreline cutting 13 degrees and ran 66 to 67 m through it,
/// both times, and the handoff took it coming back at 3.6 to 4.2 degrees,
/// 41 to 43 m off. The 737 ran 20 to 28 m through, the SR22 never crossed.
/// Led by 3.5 s the fitted A330 stops inside 8 m of the line in every tick
/// rate from 0.6 to 0.8 s (the pursuit's damping: the lead is the
/// derivative term the heading lag takes away). Per airframe
/// (`circuit_final_lead_s`), silent none: on the 737's fit the same lead
/// hands over later (3.5 against 4.2 km out from the LEMG 13 replay) and
/// autoland's final wanders 17 to 22 m below 500 ft, where the bare pursuit
/// keeps it inside 13.
pub fn final_lead_s() -> f64 {
    crate::aircraft::live_profile()
        .and_then(|p| p.circuit_final_lead_s)
        .filter(|s| s.is_finite() && *s > 0.0)
        .map_or(0.0, |s| s.min(MAX_FINAL_LEAD_S))
}
/// A lead past this is a typo, not an autopilot.
const MAX_FINAL_LEAD_S: f64 = 8.0;

/// The offset the final leg pursues, metres (right positive): `xtk_m`
/// carried `lead_s` ahead along the ground track (`track_deg`, true) at
/// `speed_kts` against the runway's `course_deg`. No lead, a failed track
/// read or airspeed: the bare offset, the old law.
pub fn final_led_xtk_m(
    xtk_m: f64,
    track_deg: Option<f64>,
    course_deg: f64,
    speed_kts: f64,
    lead_s: f64,
) -> f64 {
    let Some(trk) = track_deg.filter(|t| t.is_finite()) else {
        return xtk_m;
    };
    if !(speed_kts.is_finite() && speed_kts > 0.0 && lead_s > 0.0) {
        return xtk_m;
    }
    let off = crate::angles::signed_diff_deg(trk, course_deg).to_radians();
    xtk_m + lead_s * speed_kts * 0.514_444 * off.sin()
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
#[cfg(test)]
pub fn path_descent_fpm(closure_kts: f64) -> f64 {
    path_descent_fpm_on(closure_kts, crate::autoland::GLIDEPATH_FT_PER_NM)
}

/// [`path_descent_fpm`] on a path of `gp_ft_per_nm`.
pub fn path_descent_fpm_on(closure_kts: f64, gp_ft_per_nm: f64) -> f64 {
    if closure_kts <= 0.0 {
        return 0.0;
    }
    -gp_ft_per_nm * closure_kts / 60.0
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
#[cfg(test)]
pub fn alt_capture_vs(target_ft: f64, target_rate_fpm: f64, alt_ft: f64, vs_fpm: f64) -> f64 {
    alt_capture_vs_on(
        target_ft,
        target_rate_fpm,
        alt_ft,
        vs_fpm,
        ALT_MAX_EXTRA_DESCENT_FPM,
    )
}

/// [`alt_capture_vs`] with `extra_descent_fpm` of descent authority beyond
/// the target's own rate, sized on the height to lose (#188,
/// [`leg_descent_authority_fpm`]).
pub fn alt_capture_vs_on(
    target_ft: f64,
    target_rate_fpm: f64,
    alt_ft: f64,
    vs_fpm: f64,
    extra_descent_fpm: f64,
) -> f64 {
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
    (target_rate_fpm + correction)
        .clamp(target_rate_fpm.min(0.0) - extra_descent_fpm.max(0.0), 700.0)
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
#[cfg(test)]
pub fn approach_leg_alt_ft(field_elev_ft: f64, tpa_msl_ft: f64, dist_m: f64) -> f64 {
    approach_leg_alt_on(
        field_elev_ft,
        tpa_msl_ft,
        dist_m,
        crate::autoland::GLIDEPATH_FT_PER_NM,
    )
}

/// [`approach_leg_alt_ft`] on a path of `gp_ft_per_nm` (318 at 3 degrees;
/// steeper only where the terrain under the pattern asks for it,
/// [`DescentShape`]).
pub fn approach_leg_alt_on(
    field_elev_ft: f64,
    tpa_msl_ft: f64,
    dist_m: f64,
    gp_ft_per_nm: f64,
) -> f64 {
    let dist_nm = dist_m.abs() / M_PER_NM;
    let path = field_elev_ft + dist_nm * gp_ft_per_nm;
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

/// The live floor over the ground under and ahead of the aircraft (#182): the
/// same 500 ft the descending legs are planned to keep ([`DESCENT_FLOOR_FT`]).
const LIVE_FLOOR_FT: f64 = 500.0;
/// The upwind's trip, the old forward guard's figure.
const UPWIND_FLOOR_FT: f64 = 400.0;
/// ...and how far above the ground a level leg climbs once it trips, so a
/// floor that trips does not sit on its own trigger (the old guard's 800).
const LIVE_FLOOR_CLIMB_FT: f64 = 800.0;
/// How far ahead along the track the live floor looks, nm, and in how many
/// points: 1.5 nm is 25 s at a jet's 210 kt pattern speed, time to arrest a
/// 1,500 fpm descent; four points and the one under us are one probe query.
pub const LIVE_LOOK_NM: f64 = 1.5;
pub const LIVE_LOOK_POINTS: usize = 4;

/// The floor the circuit holds over `ground_ft` (the highest ground under
/// and ahead), `None` while `alt_msl_ft` keeps [`LIVE_FLOOR_FT`] over it
/// ([`UPWIND_FLOOR_FT`] on the upwind). On
/// the level legs it climbs to [`LIVE_FLOOR_CLIMB_FT`]; on base only to the
/// floor itself (the approach, handed over high only for the ground). Never
/// on final (autoland's). Pure.
pub fn live_floor_ft(leg: Leg, alt_msl_ft: f64, ground_ft: f64) -> Option<f64> {
    if leg == Leg::Final || !alt_msl_ft.is_finite() || !ground_ft.is_finite() {
        return None;
    }
    // The upwind is the takeoff's climb, handed over at about 500 ft: it
    // keeps the old guard's 400 so level ground ahead is not a warning.
    let trip = if leg == Leg::Upwind {
        UPWIND_FLOOR_FT
    } else {
        LIVE_FLOOR_FT
    };
    if alt_msl_ft - ground_ft >= trip {
        return None;
    }
    let over = if leg == Leg::Base {
        LIVE_FLOOR_FT
    } else {
        LIVE_FLOOR_CLIMB_FT
    };
    Some(ground_ft + over)
}

// ---- Arriving ON the path (#188) ----
//
// LEMG 13 SR22, 2026-09-29 (sorties 1790692781, 1790694884 and 1790697583,
// all `full_auto`): terrain put the pattern at 2,571 ft (2,523 ft over the
// field), the platform at 970 and the base's path target at 771. From abeam
// at 2,450 ft the downwind, base and final were flown on the capture's
// standing-target authority, a flat -500 fpm (`ALT_MAX_EXTRA_DESCENT_FPM`
// over a rate of 0): 2,070 ft at the base turn, 1,742 at the final turn,
// 1,269 at the handoff at 1.51 nm against a 610 ft path, 545 ft above
// autoland's path, which went around at once (`late_go_around`). Losing
// 1,850 ft in the 144 s from abeam to that handoff wanted 770 fpm, which
// the airframe had. So the descent is SIZED on the height to lose over the
// track left, bounded by what the airframe descends at in a pattern; the
// downwind extends where even that cannot make the path; and the handoff
// waits for the path.

/// Handed to autoland no higher than this over the circuit's path, ft. Its
/// path (touchdown aim) sits about 55 ft over ours; its straight-in dives
/// from inside 500 ft above ([`crate::autoland::aims_at_join_point`]), so
/// this leaves it a catch-up, never a dive.
pub const HANDOFF_ABOVE_PATH_FT: f64 = 200.0;
/// A final still higher than [`HANDOFF_ABOVE_PATH_FT`] this close in is
/// flown around by the circuit itself, never handed over high: a mile, about
/// 300 ft on the path, where the normal handoff has long happened (1.5 to 2.2
/// nm in the LEMG circuits).
pub const FINAL_HIGH_LAST_M: f64 = M_PER_NM;
/// The share of the track left a descent is planned on: arrive with a
/// quarter in hand for the capture's lag and a turn flown wider than drawn.
const DESCENT_PLAN_SHARE: f64 = 0.75;
/// The downwind is extended at most this far past its base turn: the
/// corridor was probed to `downwind_past_m` and the live floor
/// ([`live_floor_ft`]) is all that guards past it.
pub const MAX_DOWNWIND_EXTEND_M: f64 = 2.0 * M_PER_NM;

/// Height over the circuit's glidepath at `along_m` from the threshold, ft
/// (the path [`approach_leg_alt_on`] flies, unfloored). `None` without a
/// reading. Pure.
pub fn above_path_ft(
    alt_msl_ft: f64,
    field_elev_ft: f64,
    along_m: f64,
    gp_ft_per_nm: f64,
) -> Option<f64> {
    (alt_msl_ft.is_finite() && along_m.is_finite())
        .then(|| alt_msl_ft - (field_elev_ft + along_m.abs() / M_PER_NM * gp_ft_per_nm))
}

/// The steepest descent the pattern plans on, fpm (positive): the
/// reversal's figure for a high pattern, the airframe's enroute rate within
/// 800 to 1,500 ([`crate::reversal::pattern_descent_fpm`]).
pub fn pattern_descent_cap_fpm(enroute_descent_fpm: f64) -> f64 {
    crate::reversal::pattern_descent_fpm(f64::INFINITY, enroute_descent_fpm)
}

/// The descent authority the capture is given, fpm (positive): the rate that
/// loses `above_ft` over [`DESCENT_PLAN_SHARE`] of `to_go_m` at `gs_kts`,
/// never less than the old standing authority
/// ([`ALT_MAX_EXTRA_DESCENT_FPM`], so an ordinary circuit flies exactly as it
/// did) and never more than `cap_fpm`. Pure.
pub fn descent_authority_fpm(above_ft: f64, to_go_m: f64, gs_kts: f64, cap_fpm: f64) -> f64 {
    let top = cap_fpm.max(ALT_MAX_EXTRA_DESCENT_FPM);
    if !above_ft.is_finite() || above_ft <= 0.0 || !to_go_m.is_finite() {
        return ALT_MAX_EXTRA_DESCENT_FPM;
    }
    let v_mps = gs_kts.max(50.0) * 0.514_444;
    let minutes = (to_go_m * DESCENT_PLAN_SHARE).max(1.0) / v_mps / 60.0;
    (above_ft / minutes).clamp(ALT_MAX_EXTRA_DESCENT_FPM, top)
}

/// The track still to fly to the roll-out on final, m: the rest of the
/// downwind past abeam to its base turn, then the base across to the
/// centreline. `None` on the other legs. Pure.
pub fn to_rollout_m(leg: Leg, a_m: f64, x_m: f64, g: &Geometry, left: bool) -> Option<f64> {
    let side = if left { -1.0 } else { 1.0 };
    let across = (side * x_m).max(0.0);
    match leg {
        Leg::Downwind if a_m < 0.0 => Some((g.downwind_past_m - a_m.abs()).max(0.0) + across),
        Leg::Base => Some(across),
        _ => None,
    }
}

/// The capture's descent authority on `leg`, fpm (positive; see
/// [`alt_capture_vs_on`]). From abeam and on base: the height over the
/// path's roll-out gate (the base's own target) over the track to it. On
/// final: the height over the path over the run to [`FINAL_HIGH_LAST_M`],
/// on top of the path's own rate `path_rate_fpm`, the whole within the cap.
/// Elsewhere the standing authority. Pure.
#[allow(clippy::too_many_arguments)]
pub fn leg_descent_authority_fpm(
    leg: Leg,
    a_m: f64,
    x_m: f64,
    alt_msl_ft: f64,
    field_elev_ft: f64,
    tpa_msl_ft: f64,
    g: &Geometry,
    left: bool,
    shape: DescentShape,
    gs_kts: f64,
    cap_fpm: f64,
    path_rate_fpm: f64,
) -> f64 {
    match leg {
        Leg::Final => {
            let above = above_path_ft(alt_msl_ft, field_elev_ft, a_m, shape.gp_ft_per_nm);
            let to_go = (a_m.abs() - FINAL_HIGH_LAST_M).max(0.0);
            descent_authority_fpm(
                above.unwrap_or(0.0),
                to_go,
                gs_kts,
                cap_fpm - path_rate_fpm.abs(),
            )
        }
        _ => match to_rollout_m(leg, a_m, x_m, g, left) {
            Some(to_go) => {
                let gate = approach_leg_alt_on(
                    field_elev_ft,
                    tpa_msl_ft,
                    path_dist_m(Leg::Base, a_m, g.downwind_past_m),
                    shape.gp_ft_per_nm,
                );
                descent_authority_fpm(alt_msl_ft - gate, to_go, gs_kts, cap_fpm)
            }
            None => ALT_MAX_EXTRA_DESCENT_FPM,
        },
    }
}

/// Hold the downwind past its base turn: too high for the rest of the
/// pattern to put the aircraft on the path (the reversal's
/// [`crate::reversal::too_high_to_turn`], for the circuit). Turning base at
/// `a_m` rolls out there, where the path is `approach_leg_alt_on`; what can
/// come off on the way is the cap across the base plus the final's catch-up
/// over the path's own rate (at least the standing
/// [`ALT_MAX_EXTRA_DESCENT_FPM`]) down to [`FINAL_HIGH_LAST_M`], planned on
/// [`DESCENT_PLAN_SHARE`] of it. Each metre flown on gains a little: the
/// descent is steeper than the path. Never past [`MAX_DOWNWIND_EXTEND_M`]
/// beyond the base point. Pure.
#[allow(clippy::too_many_arguments)]
pub fn extend_downwind(
    alt_msl_ft: f64,
    field_elev_ft: f64,
    tpa_msl_ft: f64,
    a_m: f64,
    x_m: f64,
    g: &Geometry,
    left: bool,
    shape: DescentShape,
    gs_kts: f64,
    cap_fpm: f64,
) -> bool {
    let along = a_m.abs();
    if a_m >= 0.0
        || !alt_msl_ft.is_finite()
        || !x_m.is_finite()
        || along >= g.downwind_past_m + MAX_DOWNWIND_EXTEND_M
    {
        return false;
    }
    let side = if left { -1.0 } else { 1.0 };
    let across = (side * x_m).max(0.0);
    let rollout = along.max(g.downwind_past_m);
    let gate = approach_leg_alt_on(field_elev_ft, tpa_msl_ft, rollout, shape.gp_ft_per_nm);
    let gs = gs_kts.max(50.0);
    let v_mps = gs * 0.514_444;
    let base_min = across / v_mps / 60.0;
    let final_min = (rollout - FINAL_HIGH_LAST_M).max(0.0) / v_mps / 60.0;
    let path_fpm = shape.gp_ft_per_nm * gs / 60.0;
    let catch_up = (cap_fpm - path_fpm).max(ALT_MAX_EXTRA_DESCENT_FPM);
    let can = (cap_fpm.max(ALT_MAX_EXTRA_DESCENT_FPM) * base_min + catch_up * final_min)
        * DESCENT_PLAN_SHARE;
    alt_msl_ft - gate > can
}

/// A final still above [`HANDOFF_ABOVE_PATH_FT`] inside
/// [`FINAL_HIGH_LAST_M`]: never handed over, flown around. Pure.
pub fn final_high_give_up(above_path_ft: Option<f64>, along_m: f64) -> bool {
    above_path_ft.is_some_and(|h| h > HANDOFF_ABOVE_PATH_FT) && along_m.abs() <= FINAL_HIGH_LAST_M
}

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
const PATTERN_MIN_STALL_MULT: f64 = crate::aircraft::PATTERN_MIN_RATIO;

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

/// The speed the FINAL leg slows toward: the approach speed, floored by the
/// configuration the flaps are actually in (`aircraft::config_floor_kts`).
/// Base flies the approach-setup margin; final hands autoland an aircraft
/// already at Vref with the landing flaps out, instead of 20 kt fast at
/// flaps 5 with a mile to fix both (the 737 at LFLL, 2026-09-17: the
/// autoland idled the whole last mile and touched down 17 kt slow).
pub fn final_leg_kts(approach_ias: f64, config_floor: Option<f64>) -> f64 {
    crate::aircraft::floored_target(approach_ias, config_floor)
}

/// The speeds a pattern's legs are built from, fixed for the circuit.
#[derive(Debug, Clone, Copy)]
pub struct LegSpeeds {
    /// The clean pattern speed ([`pattern_speed_kts`]).
    pub pattern_kts: f64,
    /// The approach-setup speed base flies ([`approach_leg_kts`]).
    pub approach_kts: f64,
    /// The profile's approach speed, final's target before its floor.
    pub approach_ias: f64,
}

/// The ONE speed a pattern leg holds, fed to BOTH the throttle and the
/// climb limiter's full-rate speed (#169). Base flies the approach-setup
/// speed, final the approach speed floored by the flaps actually out; the
/// rest of the pattern flies the pattern speed (which the geometry was
/// sized on), and a clean-up in progress (the takeoff's or a go-around's)
/// flies inside the placard of what is out ([`go_around_speed_kts`]).
/// `floor_of` is the configuration floor for a flap ratio
/// ([`crate::aircraft::config_floor_kts`] live).
///
/// The A330 at LEMG 13 (sortie 1790621184-full_auto, 2026-09-28): the
/// throttle held 212.5 for CONF 1 while the limiter granted full rate only
/// at 220 (the pattern speed's), read 210 as slow, faded the climb to 140
/// fpm while the throttle was still easing off TOGA, and the climb given up
/// went into speed: 224 kt on the crosswind, over the 215 placard. Pure.
#[allow(clippy::too_many_arguments)]
pub fn leg_want_kts(
    leg: Leg,
    speeds: LegSpeeds,
    last_flap: Option<f64>,
    cleaning_up: Option<f64>,
    schedule: &[(f64, f64)],
    limits: &[(f64, Option<f64>)],
    floor_of: impl Fn(f64) -> Option<f64>,
) -> f64 {
    match leg {
        Leg::Final => final_leg_kts(speeds.approach_ias, floor_of(last_flap.unwrap_or(0.0))),
        Leg::Base => speeds.approach_kts,
        _ => match cleaning_up {
            Some(flap) => {
                go_around_speed_kts(speeds.pattern_kts, flap, schedule, limits, floor_of(flap))
            }
            None => speeds.pattern_kts,
        },
    }
}

/// The detent next more deployed than `flap` among `limits`, if any. Pure.
pub fn detent_above(flap: f64, limits: &[(f64, Option<f64>)]) -> Option<f64> {
    limits
        .iter()
        .map(|(ratio, _)| *ratio)
        .filter(|ratio| *ratio > flap + 1e-6)
        .min_by(f64::total_cmp)
}

/// The flap a go-around selects at once: the go-around's half (autoland's
/// own), never more than is already out.
pub const GO_AROUND_FLAP_RATIO: f64 = 0.5;
/// While cleaning up, the speed stays this far under the placard of the flap
/// that is out.
const GO_AROUND_PLACARD_MARGIN_KTS: f64 = 10.0;
/// ...and reaches this far past the next retraction speed, so the step is
/// taken rather than hovered at.
const GO_AROUND_RETRACT_MARGIN_KTS: f64 = 5.0;
/// Where a retraction speed sits inside the placard margin of the flap being
/// left, the step is taken this far under the placard less its margin.
const CLEANUP_STEP_UNDER_CAP_KTS: f64 = 3.0;

/// The placard of the flap `flap` out ([`crate::aircraft::flap_placard_kts`],
/// the one lookup). Infinite when none is. Pure.
fn placard_of(flap: f64, limits: &[(f64, Option<f64>)]) -> f64 {
    crate::aircraft::flap_placard_kts(flap, limits).unwrap_or(f64::INFINITY)
}

/// The retraction schedule a clean-up walks from `from` out (#169): the
/// next step's speed held under the placard of `from`, less the placard
/// margin and [`CLEANUP_STEP_UNDER_CAP_KTS`], so the speed that takes the
/// step can be flown inside the placard. Every other step, and a step that
/// already fits (every profile but the A330's CONF 1), is unchanged. The
/// A330 retracts CONF 1 at 210 under a 215 placard: the clean-up flew
/// 212.5, 2.5 kt under it, and the level-off ran 224 (LEMG 30, sortie
/// 1790698132-full_auto, 2026-09-29); it now steps at 202 and flies 205.
/// Pure.
pub fn cleanup_schedule(
    schedule: &[(f64, f64)],
    limits: &[(f64, Option<f64>)],
    from: f64,
) -> Vec<(f64, f64)> {
    let next = schedule
        .iter()
        .map(|(r, _)| *r)
        .filter(|r| *r < from - 1e-6)
        .max_by(f64::total_cmp);
    let cap = placard_of(from, limits) - GO_AROUND_PLACARD_MARGIN_KTS - CLEANUP_STEP_UNDER_CAP_KTS;
    schedule
        .iter()
        .map(|&(ratio, kts)| {
            if next.is_some_and(|n| (n - ratio).abs() < 1e-6) {
                (ratio, kts.min(cap))
            } else {
                (ratio, kts)
            }
        })
        .collect()
}

/// Whether the circuit flies a tower's go-around order on this leg: the
/// approach legs, the only ones a landing clearance stands on. The order
/// outranks everything there; elsewhere there is nothing to go around from,
/// and it is logged, not flown. Pure.
pub fn flies_go_around_order(leg: Leg) -> bool {
    matches!(leg, Leg::Base | Leg::Final)
}

/// The intercom line for a go-around the circuit flies on an order: the
/// tower's is named as its instruction, the pilot's is the readback. Pure.
pub fn go_around_line(by: crate::go_around::GoAroundBy) -> &'static str {
    match by {
        crate::go_around::GoAroundBy::Tower => "Going around. Tower's instruction.",
        crate::go_around::GoAroundBy::Pilot => "Going around.",
        crate::go_around::GoAroundBy::Crew => "Too high to land from here. Going around.",
    }
}

/// The flap the go-around selects at the order: the deepest of the
/// airframe's detents (`limits`, as [`crate::aircraft::flap_limits`]) at or
/// under the go-around's half, never more than `current` (a base at flaps 5
/// keeps 5). A real detent: the MD-82 has none at half travel, and its 15
/// (0.375) is the go-around flap there. No detents: the ratio itself. Pure.
pub fn go_around_flap(current: Option<f64>, limits: &[(f64, Option<f64>)]) -> f64 {
    let want = current.unwrap_or(0.0).clamp(0.0, GO_AROUND_FLAP_RATIO);
    if limits.is_empty() {
        return want;
    }
    limits
        .iter()
        .map(|(ratio, _)| *ratio)
        .filter(|ratio| *ratio <= want + 1e-6)
        .fold(0.0, f64::max)
}

/// The next flap step while a go-around cleans up: the airframe's retraction
/// schedule, one detent at a time and paced on the flaps arriving
/// ([`autotakeoff::paced_flap_step`]); with no schedule, up in one step once
/// the speed is at `clean_ok_kts`. `None` when clean or not yet. Pure.
pub fn go_around_flap_step(
    schedule: &[(f64, f64)],
    commanded: f64,
    deployed: Option<f64>,
    ias: f64,
    clean_ok_kts: f64,
) -> Option<f64> {
    if commanded <= 1e-6 {
        return None;
    }
    if schedule.is_empty() {
        return (ias >= clean_ok_kts).then_some(0.0);
    }
    autotakeoff::paced_flap_step(schedule, commanded, deployed, ias)
}

/// The speed a go-around cleaning up flies with `flap` out: the pattern
/// speed, raised past the next retraction speed so the step comes, held
/// under the placard of what is out, never under `floor_kts` (the floor for
/// that configuration). Clean, it is the pattern speed. Flying the pattern
/// speed at once would put the 737's go-around flaps 10 (Vfe 210) at 210.
/// Pure.
pub fn go_around_speed_kts(
    pattern_kts: f64,
    flap: f64,
    schedule: &[(f64, f64)],
    limits: &[(f64, Option<f64>)],
    floor_kts: Option<f64>,
) -> f64 {
    if flap <= 1e-6 {
        return pattern_kts;
    }
    let placard = placard_of(flap, limits);
    let cap = placard - GO_AROUND_PLACARD_MARGIN_KTS;
    // The step's speed as the walk takes it ([`cleanup_schedule`]): never
    // inside the placard margin, so the margin always wins (#169).
    let next_retract = cleanup_schedule(schedule, limits, flap)
        .into_iter()
        .filter(|(ratio, _)| *ratio < flap - 1e-6)
        .max_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, kts)| kts);
    let target = pattern_kts
        .max(next_retract.map_or(0.0, |r| r + GO_AROUND_RETRACT_MARGIN_KTS))
        .min(cap);
    target.max(floor_kts.unwrap_or(0.0))
}

/// How far ahead the climb limiter judges the airspeed, seconds. Long enough
/// to lead the airframe's own response, short enough not to invent a speed:
/// the same order as the altitude capture's lead.
const CLIMB_SPEED_LEAD_S: f64 = 5.0;

/// The slowest the pattern will climb toward: a floor under the speed
/// protection below. Blue line where the airframe has one, else a margin over
/// the clean stall: [`crate::aircraft::CLIMB_FLOOR_RATIO`], the automation
/// floor itself. It was 1.15 Vs, under the 1.2 Vs no automation target may
/// go below, so a climb could bleed to a speed the governor would have
/// refused as a target (refactor audit 2026-10-04, finding #13; the follower
/// already faded to `speed_floor_kts`).
///
/// A twin's Vyse is the honest number here and it is not a comfort figure —
/// below blue line a light twin cannot climb on one engine at all, and the
/// margin to Vmca is what is left of the pilot's options.
pub fn climb_floor_kts(vyse_kts: Option<f64>, vs_clean_kts: Option<f64>) -> f64 {
    let from_stall = vs_clean_kts.map(|v| v * crate::aircraft::CLIMB_FLOOR_RATIO);
    match (vyse_kts, from_stall) {
        (Some(a), Some(b)) => a.max(b),
        (Some(a), None) => a,
        (None, Some(b)) => b,
        (None, None) => 0.0,
    }
}

/// The climb floor while a clean-up keeps flap out (#169): the clean floor's
/// stall margin taken on the stall IN that configuration (`config_floor`,
/// 1.2 Vs there, from [`crate::aircraft::config_floor_kts`]); a floor that
/// is the airframe's blue line stays. Clean, or with nothing known, the
/// clean floor.
///
/// The A330 handed the circuit CONF 2 (placard 196) at 178 kt: the clean
/// floor 201.25 sat over the placard, so the limiter read every legal speed
/// as below the floor and commanded no climb at all for ten seconds of the
/// upwind (LEMG 13, sortie 1790621184-full_auto, 2026-09-28). Pure.
pub fn climb_floor_in_config(
    clean_floor: f64,
    vs_clean_kts: Option<f64>,
    config_floor: Option<f64>,
) -> f64 {
    let (Some(vs), Some(cfg)) = (vs_clean_kts, config_floor) else {
        return clean_floor;
    };
    let from_stall = vs * crate::aircraft::CLIMB_FLOOR_RATIO;
    if clean_floor > from_stall + 1e-6 {
        // Blue line, not the stall: it does not move with the flaps.
        return clean_floor;
    }
    let vs_config = cfg / crate::aircraft::SPEED_FLOOR_RATIO;
    clean_floor.min(vs_config * crate::aircraft::CLIMB_FLOOR_RATIO)
}

/// The climb floor a pattern leg is judged against: the clean-up's
/// ([`climb_floor_in_config`]) on the climbing legs while flap is kept, else
/// the clean floor. Pure.
pub fn leg_climb_floor_kts(
    leg: Leg,
    clean_floor: f64,
    vs_clean_kts: Option<f64>,
    cleaning_up: Option<f64>,
    floor_of: impl Fn(f64) -> Option<f64>,
) -> f64 {
    match (leg, cleaning_up) {
        (Leg::Base | Leg::Final, _) | (_, None) => clean_floor,
        (_, Some(flap)) => climb_floor_in_config(clean_floor, vs_clean_kts, floor_of(flap)),
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

/// The climb the takeoff handed over, carried into the pattern's own law
/// (#150). LEMG 13 (1790596335-full_auto, 2026-09-28): the takeoff's dial
/// read 1,500 at the 500 ft handoff with the 737 still climbing 2,150, and
/// the pattern's first tick wrote 456; the aircraft was down to 870 fpm eight
/// seconds later. The command now walks from the handed-over dial to the
/// pattern's rate at [`autotakeoff::CLIMB_VS_SLEW_FPM_S`], and lets go once it
/// arrives, on the approach legs (whose descent is the point), or at the
/// climb floor (the speed protection's answer is not paced).
#[derive(Debug, Clone, Copy, Default)]
pub struct ClimbCarry {
    cmd: Option<f64>,
}

impl ClimbCarry {
    pub fn new(handed_fpm: Option<f64>) -> Self {
        ClimbCarry {
            cmd: handed_fpm.filter(|v| v.is_finite()),
        }
    }

    /// The command for this tick given the pattern's own `law_fpm`. `slow`
    /// is the airspeed at or under the climb floor (a failed read is not
    /// slow). Pure.
    pub fn step(&mut self, law_fpm: f64, dt_s: f64, approach_leg: bool, slow: bool) -> f64 {
        let Some(prev) = self.cmd.filter(|_| !approach_leg && !slow) else {
            self.cmd = None;
            return law_fpm;
        };
        let cmd = autotakeoff::climb_vs_slewed(prev, law_fpm, dt_s);
        self.cmd = ((cmd - law_fpm).abs() > 1.0).then_some(cmd);
        cmd
    }

    /// Still walking toward the pattern's law.
    pub fn carrying(&self) -> bool {
        self.cmd.is_some()
    }
}

/// The flap the takeoff left for the pattern to clean up: out, and an
/// airframe schedule to walk it on. Pure.
pub fn takeoff_flaps_to_clean(flaps_left: Option<f64>, schedule: &[(f64, f64)]) -> Option<f64> {
    flaps_left.filter(|f| f.is_finite() && *f > 1e-6 && !schedule.is_empty())
}

/// An airborne AGL below this after the takeoff loop exits means it REJECTED
/// (never flew) — the circuit is off, not into the pattern.
const AIRBORNE_MIN_AGL_FT: f64 = 300.0;
/// Terrain clearance the GENERIC rectangle must have at pattern altitude over
/// every leg it flies level (the crosswind, and the downwind until abeam the
/// threshold), over the whole corridor the autopilot really flies
/// ([`corridor_frame`]); below it the circuit switches sides, and with BOTH
/// sides fouled it raises the pattern altitude. 1,000 ft: SERA.5005(f)(1)'s
/// figure over congested areas, and the standard for an automation flying a
/// leg nobody is looking out of (#182). It was 600, checked on the drawn line
/// alone: the 737's LEMG 13 left side read 729 ft and was flown at 1,548
/// (sortie 1790599750), the downwind 1.3 km outside that line.
pub const PATTERN_TERRAIN_CLEAR_FT: f64 = 1000.0;
/// The authored LXGB route's own figure ([`raise_for_terrain`]), kept at the
/// 600 ft it was flown and checked on (2026-07-26, 2026-09-15): its legs are
/// the drawn lines themselves, joined and captured within
/// [`AUTHORED_CAPTURE_M`] by a waypoint follower rather than a rectangle whose
/// autopilot turns overshoot by a kilometre, and its climb model already
/// refuses a leg entered low. The rectangle's 1,000 would lift or refuse the
/// Rock circuit on a figure no sortie has tested (#182).
const AUTHORED_TERRAIN_CLEAR_FT: f64 = 600.0;

/// True while the circuit orchestrator runs. Written only by the run
/// ([`RUN`]).
pub static ACTIVE: AtomicBool = AtomicBool::new(false);
/// The circuit's run: claimed by [`start`], cancelled by [`abort`] ("cancel
/// circuit" / "my controls", a stand-down). The circuit's client carries the
/// run's token, so a cancel is seen by every loop through
/// `xp.cancelled()` (the old shared ABORT latch was cleared by the next
/// circuit's start, under a circuit still winding down) and refuses its
/// writes; the sub-loops it launches take the token as their parent.
pub(crate) static RUN: sim::run::Slot = sim::run::Slot::flagged(&ACTIVE);

/// The circuit run that owns `xp` is over: its flag drops now (the rest of
/// the run, a give-up's hand-back, still to come). Never a newer run's.
fn lower_flag(xp: &XPlaneClient) {
    for t in xp.cancellation() {
        RUN.lower_flag_for(&t);
    }
}

/// The circuit run that owns `xp` handed the aircraft to a loop it launched
/// with its token as the parent (autoland, the published arrival): its exit
/// must not cancel that loop.
fn handed_off(xp: &XPlaneClient) {
    for t in xp.cancellation() {
        RUN.record_for(&t, sim::run::Outcome::HandedOff);
    }
}

/// One-line status for the dashboard's "managing" panel.
/// The circuit's live line for the panel: the leg, the runway, the altitude
/// against the pattern target and the speed. Set every tick, cleared with
/// the loop.
pub static DETAIL: crate::auto_status::Detail = crate::auto_status::Detail::new();

pub fn status() -> Option<String> {
    ACTIVE
        .load(Ordering::Relaxed)
        .then(|| DETAIL.line("auto circuit"))
}

/// "downwind runway 27, 1,200 ft to 1,500, 75 kt"; the speed is left out
/// while the read has not come in.
pub fn circuit_detail(leg: Leg, rwy_id: &str, alt_ft: f64, target_alt_ft: f64, ias: f64) -> String {
    let leg = format!("{leg:?}").to_lowercase();
    let alt = crate::auto_status::commas((alt_ft / 10.0).round() as i64 * 10);
    let tgt = crate::auto_status::commas((target_alt_ft / 10.0).round() as i64 * 10);
    let mut s = format!("{leg} runway {rwy_id}, {alt} ft to {tgt}");
    if ias.is_finite() {
        s.push_str(&format!(", {} kt", (ias / 5.0).round() * 5.0));
    }
    s
}

// The crew radio queue moved to `crew_radio.rs` (phase 2); re-exported so
// every `circuit::` path keeps compiling.
pub use crate::crew_radio::*;


/// Cancel an active circuit and stand down whichever sub-loop is flying;
/// returns whether one was running.
pub fn abort() -> bool {
    let was = RUN.cancel();
    // Stand down whatever phase is live. The takeoff hands off silently (power
    // stays set); the procedure/autoland arrival aborts to the pilot.
    autotakeoff::cancel();
    crate::procedure::abort();
    crate::autoland::abort();
    was
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

/// The profile's `ap_altitude_capture`: the pattern altitude goes on the
/// altitude dial and the autopilot's own capture holds the level legs (see
/// [`ap_holds_level`]). Off, the VS law flies altitude as it always has and
/// the dial is only lifted out of the way. Republished on every swap
/// ([`crate::flight_state::publish_airframe`]).
static AP_ALTITUDE_CAPTURE: AtomicBool = AtomicBool::new(false);

pub fn publish_ap_altitude_capture(on: bool) {
    AP_ALTITUDE_CAPTURE.store(on, Ordering::SeqCst);
}

const ALT_HOLD_STATUS: &str = autoflight::names::ALT_HOLD_STATUS;

/// The altitude the dial should read for this tick's vertical target. A LEVEL
/// target (pattern altitude, the base leg's height) is selected to the hundred
/// so the autopilot captures it; on the glidepath the dial is parked above,
/// autoland's own rule, because a capture under a descent eats the descent
/// (LEAX, 2026-09-15). `None` = leave it.
pub fn circuit_dial_ft(
    target_rate_fpm: f64,
    target_alt_ft: f64,
    dial_ft: f64,
    alt_ft: f64,
) -> Option<f64> {
    if target_rate_fpm != 0.0 {
        return crate::autoland::parked_altitude_dial_ft(dial_ft, alt_ft);
    }
    let want = (target_alt_ft / 100.0).round() * 100.0;
    ((dial_ft - want).abs() > 50.0).then_some(want)
}

/// How long VS mode may stay off under a command for it before it is said.
pub const VS_REFUSED_SECS: f64 = 8.0;
pub fn vs_stuck(off_for_s: f64) -> bool {
    off_for_s >= VS_REFUSED_SECS
}

/// Whether the autopilot's altitude hold has the level leg, so the VS law
/// stands off. CAPTURED (status 2) on a dial that IS this tick's level target:
/// re-pressing VS mode then would drop the hold it just made, every tick. The
/// A330 wandered 1,796 to 1,840 ft down the EGPF downwind on the VS law
/// (2026-09-21) with an altitude hold sitting unused behind a dial at 100 ft.
pub fn ap_holds_level(
    target_rate_fpm: f64,
    target_alt_ft: f64,
    dial_ft: f64,
    alt_hold_status: f64,
) -> bool {
    target_rate_fpm == 0.0
        && alt_hold_status.round() as i64 == 2
        && (dial_ft - (target_alt_ft / 100.0).round() * 100.0).abs() <= 50.0
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

/// Put the bug on a WHOLE degree, and only when that degree changes. The
/// laws above keep their fine value for their own arithmetic; the knob a
/// pilot sees moves in the steps a pilot's hand makes, not a tenth of a
/// degree every tick (the pilot, 2026-09-26: "a human pilot would never use
/// the heading bug like that"). Half a degree of rounding is inside what the
/// autopilot's own heading hold settles to.
async fn write_bug(xp: &dyn crate::xplane::XPlane, bug_mag: f64) {
    let whole = bug_mag.round().rem_euclid(360.0);
    let bits = whole.to_bits();
    if LAST_BUG.swap(bits, std::sync::atomic::Ordering::Relaxed) == bits {
        return;
    }
    if autoflight::set_dial(xp, Dial::HeadingBug, whole).await.is_err() {
        LAST_BUG.store(u64::MAX, std::sync::atomic::Ordering::Relaxed);
    }
}

/// The circuit's legs: a two-degree correction at most every ten seconds.
pub const CIRCUIT_LEG_DEADBAND: BugDeadband = BugDeadband { deg: 2.0, every_s: 10.0 };
/// The circuit's final, closing the centreline for the handover: a degree
/// at most every three seconds.
pub const CIRCUIT_FINAL_DEADBAND: BugDeadband = BugDeadband { deg: 1.0, every_s: 3.0 };

/// The laws' bug through the [`BugHold`]: the knob is set only when the hold
/// says so, and the laws carry on from what the knob holds.
async fn hold_bug(
    xp: &dyn crate::xplane::XPlane,
    hold: &mut BugHold,
    key: u32,
    bug_mag: &mut f64,
    now_s: f64,
    db: BugDeadband,
) {
    if let Some(b) = hold.decide(key, *bug_mag, now_s, db) {
        write_bug(xp, b).await;
    }
    if let Some(s) = hold.set() {
        *bug_mag = s;
    }
}

/// The last whole degree written, as f64 bits (`u64::MAX`: none).
static LAST_BUG: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(u64::MAX);

/// The authored follower's progress watchdog fires when the distance to the
/// current waypoint has set no new low for this long: the loop is orbiting or
/// being blown away, not flying the leg.
const STALL_S: f64 = 45.0;
/// A new low has to beat the old one by this much to count as progress
/// (an orbit's near side re-sets the same distance every lap).
const STALL_PROGRESS_M: f64 = 25.0;
/// Stalls the authored follower tolerates before handing the aircraft back.
const MAX_STALLS: u32 = 3;

/// Distance-closure watchdog for one waypoint (see [`STALL_S`]).
#[derive(Debug, Clone, Copy)]
pub struct Closure {
    best_m: f64,
    best_at_s: f64,
}

impl Closure {
    pub fn new(d_m: f64, now_s: f64) -> Self {
        Closure {
            best_m: d_m,
            best_at_s: now_s,
        }
    }

    /// A watchdog for a leg that BEGINS with a turn: no verdict for
    /// `grace_s`, because an aircraft turning to the fix opens the distance
    /// before it closes it. The follower's leg to FI30 at LIPK started with a
    /// 141 degree turn, was judged "not closing" 45 s in, still turning, and
    /// the whole approach stood down 5 nm from the fix (2026-09-25).
    pub fn with_grace(d_m: f64, now_s: f64, grace_s: f64) -> Self {
        Closure {
            best_m: d_m,
            best_at_s: now_s + grace_s.max(0.0),
        }
    }

    /// Feed this tick's distance. True once per stall: no new low for
    /// [`STALL_S`]. The clock restarts so the next stall is judged afresh.
    pub fn stalled(&mut self, d_m: f64, now_s: f64) -> bool {
        if d_m < self.best_m - STALL_PROGRESS_M {
            self.best_m = d_m;
            self.best_at_s = now_s;
            return false;
        }
        if now_s - self.best_at_s > STALL_S {
            self.best_m = d_m;
            self.best_at_s = now_s;
            return true;
        }
        false
    }
}

/// The leg's altitude target (MSL): the pattern altitude, the downwind's
/// descent from abeam the touchdown point, base and final's path. Pure; the
/// descent is ratcheted through `descent_target`.
#[cfg(test)]
pub fn leg_target_ft(
    leg: Leg,
    a_m: f64,
    field_elev_ft: f64,
    tpa_msl_ft: f64,
    downwind_past_m: f64,
    descent_target: &mut Option<f64>,
) -> f64 {
    leg_target_shaped(
        leg,
        a_m,
        field_elev_ft,
        tpa_msl_ft,
        downwind_past_m,
        DescentShape::default(),
        descent_target,
    )
}

/// [`leg_target_ft`] with the descent the pattern pick settled on
/// ([`DescentShape`]: the downwind's platform and the path's slope, raised
/// only where the ground under the pattern asks for it, #182).
pub fn leg_target_shaped(
    leg: Leg,
    a_m: f64,
    field_elev_ft: f64,
    tpa_msl_ft: f64,
    downwind_past_m: f64,
    shape: DescentShape,
    descent_target: &mut Option<f64>,
) -> f64 {
    match leg {
        // Abeam the touchdown point and later, START DOWN: the 180, where
        // a real pattern reduces power. Holding TPA to the base turn left
        // the whole descent to one short leg it could not fit (see
        // DOWNWIND_DESCENT_AGL_FT). Deliberately a FLAT target and not the
        // path: the path's distance term here is the Manhattan
        // |along| + |across|, which on an OUTBOUND downwind shrinks as the
        // aircraft flies away from the runway and would ratchet us down to
        // a few hundred feet while still abeam. Ratcheted like base, so it
        // can only ever descend.
        Leg::Downwind if a_m < 0.0 => {
            let want = (field_elev_ft + shape.platform_agl_ft).min(tpa_msl_ft);
            let t = descent_target.map_or(want, |d: f64| d.min(want));
            *descent_target = Some(t);
            t
        }
        // Base and final descend the 3 degree PATH, not to a flat height.
        Leg::Base | Leg::Final => {
            // ...and RATCHETED: an approach descends. Whatever the geometry
            // does mid-turn, the target may fall and hold, never climb back:
            // a target that rises is a level-off with the runway ahead, and
            // the aircraft chases it every time.
            let want = approach_leg_alt_on(
                field_elev_ft,
                tpa_msl_ft,
                path_dist_m(leg, a_m, downwind_past_m),
                shape.gp_ft_per_nm,
            );
            let t = descent_target.map_or(want, |d: f64| d.min(want));
            *descent_target = Some(t);
            t
        }
        _ => {
            *descent_target = None;
            tpa_msl_ft
        }
    }
}

/// Higher than this over the pattern altitude is not a pattern (#160).
/// LEMG 13 SF50 (sortie 1790607325): nothing was flying the aircraft, and
/// the circuit rode it from the 1,606 ft pattern to 21,900 ft in circles for
/// 14 minutes, the leg ticks writing VS -500 and a bug on the nose the whole
/// way. At this margin it gives up at 3,100 ft, 42 s after the takeoff.
pub const PATTERN_CEILING_MARGIN_FT: f64 = 1500.0;
/// The margin must hold this long: one frame's glitch is not a climb-away.
pub const PATTERN_GUARD_SUSTAIN_S: f64 = 3.0;
/// A leg lasting this many times its flown length at the pattern speed, plus
/// [`LEG_TIME_MARGIN_S`], is a pattern that is not being flown: that downwind
/// ran 778 s, until the sortie ended, against its 244 s (limit 669).
pub const LEG_TIME_FACTOR: f64 = 2.5;
pub const LEG_TIME_MARGIN_S: f64 = 60.0;
/// The servos out this long: someone took the autopilot out, or it dropped
/// out. The circuit's knobs mean nothing without it.
pub const SERVOS_OFF_S: f64 = 3.0;

/// The margin over the configuration's floor a recovery holds (#181).
pub const RECOVERY_MARGIN_KTS: f64 = 10.0;

/// Why the circuit gave the aircraft back (blackbox `pattern_guard`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PatternTrip {
    /// Above the pattern altitude by `above_ft` for [`PATTERN_GUARD_SUSTAIN_S`].
    AbovePattern { above_ft: f64 },
    /// On one leg `secs` against a `limit_s`.
    LegOverdue { leg: Leg, secs: f64, limit_s: f64 },
    /// The servos read out for `secs`.
    ServosOff { secs: f64 },
    /// The autopilot did not hold the climb (#181, `climb_watch`): `ias`
    /// and the climb `vs` when it was taken.
    ClimbNotHeld {
        why: crate::climb_watch::Why,
        ias: f64,
        vs: f64,
    },
}

impl PatternTrip {
    /// The blackbox's `why`.
    pub fn why(&self) -> &'static str {
        match self {
            PatternTrip::AbovePattern { .. } => "above_pattern",
            PatternTrip::LegOverdue { .. } => "leg_overdue",
            PatternTrip::ServosOff { .. } => "servos_off",
            PatternTrip::ClimbNotHeld { .. } => "climb_not_held",
        }
    }

    /// The first half of the spoken give-up. No dashes (TTS).
    pub fn words(&self) -> String {
        match self {
            PatternTrip::AbovePattern { above_ft } => format!(
                "Circuit abandoned, {:.0} feet above pattern height.",
                (above_ft / 100.0).round() * 100.0
            ),
            PatternTrip::LegOverdue { leg, secs, .. } => format!(
                "Circuit abandoned, {} for {:.0} minutes.",
                format!("{leg:?}").to_lowercase(),
                (secs / 60.0).max(1.0).round()
            ),
            PatternTrip::ServosOff { .. } => "The autopilot is off. Circuit cancelled.".into(),
            PatternTrip::ClimbNotHeld { why, .. } => format!("{} Circuit cancelled.", why.line()),
        }
    }

    /// The autopilot's pitch moves nothing (#181): hand flown, never given
    /// back to it.
    pub fn ap_has_no_pitch(&self) -> bool {
        matches!(self, PatternTrip::ClimbNotHeld { why, .. } if why.ap_has_no_pitch())
    }
}

/// How long `leg` may take before [`PatternGuard`] calls it overdue: its
/// length in the pattern's own geometry plus its turn, flown at the pattern
/// speed, times [`LEG_TIME_FACTOR`], plus [`LEG_TIME_MARGIN_S`]. Pure.
pub fn leg_limit_s(leg: Leg, g: &Geometry, runway_len_m: f64, pattern_kts: f64) -> f64 {
    let radius = g.lead_m / 0.9;
    let turn = std::f64::consts::FRAC_PI_2 * radius;
    let upwind = runway_len_m.max(0.0) + g.upwind_past_m;
    let dist = match leg {
        Leg::Upwind => upwind + turn,
        Leg::Crosswind | Leg::Base => g.width_m + turn,
        Leg::Downwind => upwind + g.downwind_past_m + 2.0 * turn,
        Leg::Final => g.downwind_past_m + turn,
    };
    let mps = pattern_kts.max(40.0) * 0.514_444;
    // An extended downwind (#188) is still the pattern, and so is its longer
    // final: the extension's own time, once (not factored, so the #160
    // downwind that never ended still trips before its sortie did).
    let extension = match leg {
        Leg::Downwind | Leg::Final => MAX_DOWNWIND_EXTEND_M / mps,
        _ => 0.0,
    };
    LEG_TIME_FACTOR * dist / mps + LEG_TIME_MARGIN_S + extension
}

/// The circuit's own give-up (#160): the aircraft far above the pattern, a
/// leg that never ends, or the servos out. Fed every tick; a failed read is
/// no measurement and changes no clock. Pure.
#[derive(Debug, Default, Clone)]
pub struct PatternGuard {
    leg: Option<Leg>,
    leg_t0: f64,
    above_since: Option<f64>,
    servos_off_since: Option<f64>,
}

impl PatternGuard {
    /// `pattern_ft` is the height the pattern is flying to (the pattern
    /// altitude, or a terrain floor over it); `servos` the servos' reading.
    pub fn step(
        &mut self,
        now_s: f64,
        leg: Leg,
        alt_msl_ft: f64,
        pattern_ft: f64,
        leg_limit_s: f64,
        servos: Option<bool>,
    ) -> Option<PatternTrip> {
        if self.leg != Some(leg) {
            self.leg = Some(leg);
            self.leg_t0 = now_s;
        }
        if alt_msl_ft.is_finite() && pattern_ft.is_finite() {
            let above_ft = alt_msl_ft - pattern_ft;
            if above_ft > PATTERN_CEILING_MARGIN_FT {
                let since = *self.above_since.get_or_insert(now_s);
                if now_s - since >= PATTERN_GUARD_SUSTAIN_S {
                    return Some(PatternTrip::AbovePattern { above_ft });
                }
            } else {
                self.above_since = None;
            }
        }
        match servos {
            Some(false) => {
                let since = *self.servos_off_since.get_or_insert(now_s);
                if now_s - since >= SERVOS_OFF_S {
                    return Some(PatternTrip::ServosOff {
                        secs: now_s - since,
                    });
                }
            }
            Some(true) => self.servos_off_since = None,
            None => {}
        }
        let secs = now_s - self.leg_t0;
        (leg_limit_s.is_finite() && secs > leg_limit_s).then_some(PatternTrip::LegOverdue {
            leg,
            secs,
            limit_s: leg_limit_s,
        })
    }
}

/// Hand the aircraft back FLYABLE and say so loudly. Wings to the heading
/// flown and VS 0 on an autopilot we own (the servos in if they read out),
/// the speed to the governor at `hold_kts`; a pilot's disconnect is left
/// alone. Only what reads back is claimed.
#[allow(clippy::too_many_arguments)]
/// The circuit's lateral re-engage, one tick: the servos pressed through
/// [`crate::ap_engage::ServoWatch`] (flight director first, re-pressed until
/// they read in, `ap_engage` / `ap_engaged` on the tape under `site`) while
/// the heading mode reads off (`wanted`). It used to press `servos_on` alone,
/// every tick, blind: on an FD airframe a press that cannot hold (refactor
/// audit 2026-10-04 #5). The servos read OUT with the mode on is the
/// `PatternGuard`'s, never re-pressed here.
#[allow(clippy::too_many_arguments)]
async fn reengage_lateral(
    xp: &dyn crate::xplane::XPlane,
    watch: &mut crate::ap_engage::ServoWatch,
    fd: &[String],
    now_s: f64,
    wanted: bool,
    servos: Option<f64>,
    ias_kts: f64,
    site: &str,
) {
    let st = watch.step(now_s, wanted, servos, ias_kts);
    if st.fire {
        crate::ap_engage::press_logged(xp, fd, "circuit", site, st.try_n).await;
    }
    if st.confirmed_now {
        crate::ap_engage::record_engaged("circuit", site, st.try_n);
    }
}

async fn give_up_pattern(
    xp: &XPlaneClient,
    base_url: &str,
    voice: &Option<Voice>,
    trip: PatternTrip,
    leg: Leg,
    alt_msl_ft: f64,
    pattern_ft: f64,
    mag_var: f64,
    hold_kts: f64,
    floor_kts: Option<f64>,
) {
    lower_flag(xp);
    // Never "your aircraft" to an aircraft that is slow, stalling or upset,
    // nor to one whose autopilot would not hold the climb (#181: handed
    // back at 73 kt, the 737 rolled past 160 degrees with nobody flying).
    // The recovery takes it: servos out, flown level, the autopilot given
    // it back only once it is seen to hold.
    let upset = match trip {
        PatternTrip::ClimbNotHeld { why, .. } if why.ap_has_no_pitch() => {
            Some("no_pitch_authority")
        }
        PatternTrip::ClimbNotHeld { .. } => Some("climb_not_held"),
        _ => crate::tools::recover::read_unflyable(xp, floor_kts).await,
    };
    if let Some(what) = upset {
        let no_pitch = trip.ap_has_no_pitch();
        let started = crate::tools::recover::take_over(
            base_url.to_string(),
            voice.clone(),
            crate::tools::recover::TakeOver {
                why: format!("circuit_{what}"),
                min_safe_ias: floor_kts.map(|f| f + RECOVERY_MARGIN_KTS),
                hold_kts: Some(hold_kts),
                ap_no_pitch: no_pitch,
                // The pattern's height: a climb the takeoff began is kept.
                hold_ft: no_pitch.then_some(pattern_ft).filter(|h| h.is_finite()),
            },
        );
        let (watch, ias, vs) = match trip {
            PatternTrip::ClimbNotHeld { why, ias, vs } => (Some(why.word()), Some(ias), Some(vs)),
            _ => (None, None, None),
        };
        crate::blackbox::record(
            "circuit",
            serde_json::json!({
                "event": "pattern_guard", "why": trip.why(), "leg": format!("{leg:?}"),
                "alt_msl": alt_msl_ft, "pattern_ft": pattern_ft, "hold_kts": hold_kts,
                "recovering": what, "recovery_started": started, "floor_kts": floor_kts,
                "watch": watch, "ias": ias, "vs": vs,
            }),
        );
        // The recovery says the missing pitch itself, first thing.
        let line = if no_pitch {
            "Circuit cancelled.".to_string()
        } else {
            format!("{} I have control, recovering.", trip.words())
        };
        alert(voice, Priority::Warning, &line);
        return;
    }
    let psi = xp.get_float(PSI_TRUE).await.ok().filter(|v| v.is_finite());
    let hdg_mag = psi.map(|p| true_to_mag(p, mag_var).round().rem_euclid(360.0));
    let ours = !matches!(trip, PatternTrip::ServosOff { .. });
    if ours {
        let servos = xp.get_float(SERVOS_ON).await.ok();
        if servos.is_some_and(|v| v < 0.5) {
            // FD first, on the tape (`ap_engage`); the read-back below
            // judges whether it is holding.
            crate::ap_engage::press_logged(
                xp, &crate::ap_engage::fd_commands(), "circuit", "pattern_guard", 1,
            )
            .await;
        }
        if let Some(h) = hdg_mag {
            write_bug(xp, h).await;
        }
        if xp.get_float(VS_STATUS).await.is_ok_and(|v| v <= 0.5) {
            let _ = autoflight::press(xp, Button::VerticalSpeed).await;
        }
        let _ = autoflight::set_dial(xp, Dial::Vs, 0.0).await;
    }
    let (servos_after, hdg_st, vs_st) = tokio::join!(
        xp.get_float(SERVOS_ON),
        xp.get_float(HDG_STATUS),
        xp.get_float(VS_STATUS)
    );
    let holding = ours
        && [&servos_after, &hdg_st, &vs_st]
            .iter()
            .all(|r| r.as_ref().is_ok_and(|v| *v > 0.5));
    let (leg_secs, limit_s, above_ft) = match trip {
        PatternTrip::LegOverdue { secs, limit_s, .. } => (Some(secs), Some(limit_s), None),
        PatternTrip::AbovePattern { above_ft } => (None, None, Some(above_ft)),
        PatternTrip::ServosOff { secs } => (Some(secs), None, None),
        PatternTrip::ClimbNotHeld { .. } => (None, None, None),
    };
    crate::blackbox::record(
        "circuit",
        serde_json::json!({
            "event": "pattern_guard", "why": trip.why(), "leg": format!("{leg:?}"),
            "alt_msl": alt_msl_ft, "pattern_ft": pattern_ft, "above_ft": above_ft,
            "secs": leg_secs, "limit_s": limit_s, "hdg_mag": hdg_mag,
            "servos_after": servos_after.ok(), "holding": holding, "hold_kts": hold_kts,
        }),
    );
    let tail = match (holding, hdg_mag) {
        (true, Some(h)) => format!("Autopilot on heading {h:03.0}, levelling off. Your aircraft."),
        (true, None) => "Autopilot levelling off. Your aircraft.".to_string(),
        (false, _) if ours => "The autopilot is not flying it. Your aircraft.".to_string(),
        (false, _) => "Your aircraft.".to_string(),
    };
    alert(
        voice,
        Priority::Warning,
        &format!("{} {tail}", trip.words()),
    );
    crate::tools::speedhold::arm_flight(base_url.to_string(), voice.clone(), hold_kts);
}

/// The climb rate a terrain hold's give-up clock credits, feet per minute:
/// a laden light single's, so the clock never runs out on a climb that is
/// going.
const TERRAIN_HOLD_CREDIT_FPM: f64 = 300.0;

/// How long a terrain hold's climb may take before the circuit's guard
/// ([`PatternGuard`]) gives it back: the climb at
/// [`TERRAIN_HOLD_CREDIT_FPM`] plus [`LEG_TIME_MARGIN_S`]. Pure.
pub fn terrain_hold_limit_s(alt_ft: f64, hold_ft: f64) -> f64 {
    let climb = if alt_ft.is_finite() && hold_ft.is_finite() {
        (hold_ft - alt_ft).max(0.0)
    } else {
        0.0
    };
    climb / TERRAIN_HOLD_CREDIT_FPM * 60.0 + LEG_TIME_MARGIN_S
}

/// The spoken airborne refusal (#182): the reason with its numbers, then
/// what the autopilot is doing about it. No dashes (TTS). Pure.
pub fn terrain_hold_line(why: &str, hold_ft: f64) -> String {
    format!("{why} Climbing straight ahead to {hold_ft:.0} feet and holding it.")
}

/// ...and the hand-over once level there: autoland's words. Pure.
pub fn terrain_hold_level_line(hold_ft: f64) -> String {
    format!(
        "Level at {hold_ft:.0} feet, straight ahead, the autopilot holding heading and \
         altitude. Your aircraft."
    )
}

/// Airborne and no circuit clears the ground (#182; LXGB, the Rock route
/// and the rectangle both refused after the takeoff): the reversal's
/// refusal ([`crate::reversal::TerrainHold`]), never "your aircraft" low
/// over the hills. Climb straight ahead on the autopilot to 1,000 ft over
/// the highest ground under and 12 nm ahead
/// ([`crate::reversal::hold_altitude_ahead_ft`], the corridors' highest
/// standing in for an unanswered probe), the altitude dial on it and the
/// governor on the configuration's speed, said with the numbers; handed
/// over only once level there. The circuit's guard still gives it back
/// loudly (servos out, a climb that never ends), through the recovery
/// when slow or upset.
#[allow(clippy::too_many_arguments)]
async fn hold_clear_of_terrain(
    xp: &XPlaneClient,
    base_url: &str,
    voice: &Option<Voice>,
    why: &str,
    corridor_max_ft: Option<f64>,
    default_platform_ft: f64,
    fallback_hdg_true: f64,
    mag_var: f64,
    hold_kts: f64,
    floor_kts: Option<f64>,
) {
    let (pos, hpath_r) = tokio::join!(sim::position::read(xp), xp.get_float(HPATH));
    let (lat, lon) = sim::position::lat_lon(&pos);
    let alt_ft = sim::position::field(&pos, |p| p.elev_m)
        .map(|m| m * M_TO_FT)
        .unwrap_or(f64::NAN);
    let psi = sim::position::field(&pos, |p| p.psi_true_deg)
        .ok()
        .filter(|v| v.is_finite());
    let track = hpath_r.ok().filter(|v| v.is_finite()).or(psi);
    let hdg_true = psi.unwrap_or(fallback_hdg_true);
    let (ahead_ft, hold_ft) = match (lat, lon) {
        (Ok(lat), Ok(lon)) => {
            crate::reversal::hold_altitude_ahead_ft(
                xp,
                (lat, lon, track.unwrap_or(hdg_true), alt_ft),
                corridor_max_ft,
                default_platform_ft,
            )
            .await
        }
        _ => (
            None,
            crate::reversal::hold_altitude_ft(None, corridor_max_ft, alt_ft, default_platform_ft),
        ),
    };
    let mut hold = crate::reversal::TerrainHold::refused(hdg_true, hold_ft);
    crate::blackbox::record(
        "circuit",
        serde_json::json!({
            "event": "terrain_hold", "hold_ft": hold_ft, "ahead_ft": ahead_ft,
            "corridor_max_ft": corridor_max_ft, "alt": alt_ft, "hdg_true": hdg_true,
            "hold_kts": hold_kts, "why": why,
        }),
    );
    alert(voice, Priority::Warning, &terrain_hold_line(why, hold_ft));
    // The autopilot's own capture holds the level: the dial on the hold.
    let bug_mag = true_to_mag(hdg_true, mag_var).rem_euclid(360.0);
    if xp.get_float(SERVOS_ON).await.is_ok_and(|v| v < 0.5) {
        // FD first, on the tape (`ap_engage`); the hold's own guard watches
        // the servos from here.
        crate::ap_engage::press_logged(
            xp, &crate::ap_engage::fd_commands(), "circuit", "terrain_hold", 1,
        )
        .await;
    }
    let _ = autoflight::set_dial(xp, Dial::Altitude, hold_ft).await;
    write_bug(xp, bug_mag).await;
    crate::tools::speedhold::arm_flight(base_url.to_string(), voice.clone(), hold_kts);
    let limit_s = terrain_hold_limit_s(alt_ft, hold_ft);
    let mut guard = PatternGuard::default();
    let t0 = std::time::Instant::now();
    let mut last_vs: Option<f64> = None;
    let mut tick: u32 = 0;
    loop {
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        if xp.cancelled() {
            stand_down(&xp, voice, "Circuit cancelled. Your aircraft.", "abort_terrain_hold");
            return;
        }
        if crate::xplane::sim_frozen(xp).await {
            continue;
        }
        let (pos, ias_r, servos_r, hdg_st, vs_st) = tokio::join!(
            sim::position::read(xp),
            xp.get_float(IAS),
            xp.get_float(SERVOS_ON),
            xp.get_float(HDG_STATUS),
            xp.get_float(VS_STATUS),
        );
        // A failed read is no measurement: the tick is skipped.
        let Some(alt) = sim::position::field(&pos, |p| p.elev_m)
            .ok()
            .map(|m| m * M_TO_FT)
            .filter(|v| v.is_finite())
        else {
            continue;
        };
        tick += 1;
        if hold.level_now(alt) {
            let _ = autoflight::set_dial(xp, Dial::Vs, 0.0).await;
            crate::blackbox::record(
                "circuit",
                serde_json::json!({"event": "terrain_hold_level", "hold_ft": hold.alt_ft,
                                   "alt": alt, "secs": t0.elapsed().as_secs_f64()}),
            );
            stand_down(&xp, voice, &terrain_hold_level_line(hold.alt_ft), TERRAIN_REFUSED);
            return;
        }
        let servos = servos_r
            .as_ref()
            .ok()
            .filter(|v| v.is_finite())
            .map(|v| *v > 0.5);
        if let Some(trip) = guard.step(
            t0.elapsed().as_secs_f64(),
            Leg::Upwind,
            alt,
            hold.alt_ft,
            limit_s,
            servos,
        ) {
            crate::blackbox::record(
                "circuit",
                serde_json::json!({"event": "terrain_hold_given_back", "why": trip.why(),
                                   "hold_ft": hold.alt_ft, "alt": alt}),
            );
            *STOOD_DOWN.lock().unwrap_or_else(|e| e.into_inner()) = Some(TERRAIN_REFUSED);
            give_up_pattern(
                xp,
                base_url,
                voice,
                trip,
                Leg::Upwind,
                alt,
                hold.alt_ft,
                mag_var,
                hold_kts,
                floor_kts,
            )
            .await;
            return;
        }
        // Modes re-engaged only when their status reads off, the dial
        // rewritten after (an engage syncs it away). The capture at the
        // hold is ALT HOLD, not a dropped VS mode.
        let hdg = autoflight::bring_heading(xp, hdg_st.ok(), "circuit_terrain_hold").await;
        if hdg.pressed(Button::Heading) {
            write_bug(xp, bug_mag).await;
        }
        if hold.alt_ft - alt > crate::reversal::HOLD_LEVEL_FT
            && autoflight::bring_vs(xp, vs_st.ok(), "circuit_terrain_hold")
                .await
                .pressed(Button::VerticalSpeed)
        {
            last_vs = None;
        }
        let ias = ias_r.ok().filter(|v| v.is_finite());
        let slow = matches!((ias, floor_kts), (Some(i), Some(f)) if i < f);
        let vs = crate::reversal::platform_vs_fpm(
            hold.alt_ft,
            alt,
            slow,
            crate::autoland::max_climb_fpm(),
            crate::procedure::descent_fpm(),
        );
        if last_vs.is_none_or(|v| (v - vs).abs() >= 10.0) {
            let _ = autoflight::set_dial(xp, Dial::Vs, vs).await;
            last_vs = Some(vs);
        }
        if tick % 10 == 1 {
            crate::blackbox::record(
                "circuit",
                serde_json::json!({"event": "terrain_hold_tick", "alt": alt, "hold_ft": hold.alt_ft,
                                   "vs_cmd": vs, "ias": ias, "slow": slow, "servos": servos}),
            );
        }
    }
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

/// How far outside (and inside) the drawn rectangle the pattern is probed, in
/// turn radii: the autopilot does not fly the line. Measured from every
/// jet circuit logged (`circuit` `leg` records against their `armed`
/// geometry, 17 circuits of the 737 and A330 at LEMG and LIEA, r 2,500 m,
/// 2026-09-26 to 09-29): the downwind was flown 930 to 2,200 m OUTSIDE its
/// line (median 1,330, 0.53 r), once 3,170 (LEMG 30, sortie 1790633832,
/// 1.27 r), and never inside it; the base ran 320 to 840 m past
/// `downwind_past_m` (1,525 on that 30). The SF50 (r 1,885) flew 250 m
/// INSIDE (0.13 r). So copies at 0.5 and 1.0 radii out cover the median and
/// all but the worst, and 0.25 in covers the tight airframe. The LEMG 13
/// base turn that went 305 ft over the ground (1790599750) was 1,200 m
/// outside the corner, between the two outer copies.
pub const CORRIDOR_OUT_RADII: [f64; 2] = [0.5, 1.0];
pub const CORRIDOR_IN_RADII: f64 = 0.25;
/// Sample spacing on the corridor's copies, metres: twice the line's
/// [`LEG_SAMPLE_M`], so the probe stays a few seconds on the runway. The
/// line itself keeps its 150 m (the Rock's 300 m ridge).
const CORRIDOR_SAMPLE_M: f64 = 300.0;

/// The least height over the ground a DESCENDING leg (the downwind from abeam
/// the threshold, base, final) may be commanded to, from
/// [`DESCENT_FLOOR_FULL_NM`] out, tapering in a straight line to nothing at
/// the threshold (#182). 500 ft is SERA.5005(f)(2)'s minimum height away
/// from congested areas; the taper is because the 3 degree path itself is
/// only 636 ft over a level field at 2 nm (250 ft a mile against the path's
/// 318, so a level field always passes and a hill on final does not). The
/// old law descended to 800 ft over the FIELD from abeam the threshold and
/// never looked at the ground: LEMG 13's base went 305 ft over a hill 900 ft
/// high (sortie 1790599750), the LEMG 30 downwind 475 (1790633832).
const DESCENT_FLOOR_FT: f64 = 500.0;
const DESCENT_FLOOR_FULL_NM: f64 = 2.0;
/// The steepest path the circuit will fly to clear the ground on its base and
/// final: PANS-OPS's upper limit for a procedure that is not a steep approach
/// (anything past it needs an approval no profile declares). Autoland is
/// handed the same angle (`autoland::HandDown::circuit`).
pub const STEEP_PATH_MAX_DEG: f64 = 3.5;
/// A raise smaller than this is carried out but not said: with 1,000 ft of
/// clearance a 1,000 ft pattern lifts over any ground above the field, and
/// "flying it at 1,030 feet" is noise.
const RAISE_SAID_FT: f64 = 100.0;
/// With both sides needing a raise, the asked-for side is kept unless the
/// other one is lower by more than this.
const RAISE_SIDE_MARGIN_FT: f64 = 100.0;

/// Feet of path per nautical mile at `deg`.
fn gp_ft_per_nm_at(deg: f64) -> f64 {
    6076.12 * deg.to_radians().tan()
}

/// The descent the pattern flies from abeam the threshold: the downwind's
/// platform (feet over the field) and the path base and final follow.
/// The default is the law as it always flew (800 ft, 3 degrees); the pick
/// raises either only where the ground under the pattern asks for it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DescentShape {
    pub platform_agl_ft: f64,
    pub gp_ft_per_nm: f64,
}

impl Default for DescentShape {
    fn default() -> Self {
        DescentShape {
            platform_agl_ft: DOWNWIND_DESCENT_AGL_FT,
            gp_ft_per_nm: crate::autoland::GLIDEPATH_FT_PER_NM,
        }
    }
}

impl DescentShape {
    /// The path's angle, degrees.
    pub fn glidepath_deg(&self) -> f64 {
        (self.gp_ft_per_nm / 6076.12).atan().to_degrees()
    }

    /// Steeper than the standard 3 degrees.
    pub fn steep(&self) -> bool {
        self.gp_ft_per_nm > crate::autoland::GLIDEPATH_FT_PER_NM + 0.5
    }
}

/// Which part of the descent a ground sample sits under.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DescentLeg {
    /// The downwind from abeam the threshold: the flat platform.
    Platform,
    /// Base, the turn onto it included (the leg starts at the downwind's
    /// led turn), and final: the lower of the platform and the path at
    /// `path_m`.
    Base,
    Final,
}

impl DescentLeg {
    fn on_path(self) -> bool {
        !matches!(self, DescentLeg::Platform)
    }

    fn words(self) -> &'static str {
        match self {
            DescentLeg::Platform => "the downwind",
            DescentLeg::Base => "the base",
            DescentLeg::Final => "the final",
        }
    }
}

/// The ground under one point of a descending leg, with the distances its
/// target and its floor are judged on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GroundSample {
    pub leg: DescentLeg,
    /// The distance the leg's path target is computed on ([`path_dist_m`]).
    pub path_m: f64,
    /// Straight-line distance to the threshold, for the floor's taper.
    pub thr_m: f64,
    pub ground_ft: f64,
}

/// The floor a descending leg keeps over the ground `thr_m` from the
/// threshold ([`DESCENT_FLOOR_FT`]). Pure.
pub fn descent_floor_ft(thr_m: f64) -> f64 {
    DESCENT_FLOOR_FT * (thr_m.abs() / M_PER_NM / DESCENT_FLOOR_FULL_NM).clamp(0.0, 1.0)
}

/// The altitude the pattern commands over a descent sample, feet MSL: what
/// [`leg_target_shaped`] ratchets to there (the platform, then the lower of
/// it and the path). Pure.
pub fn descent_target_ft(
    s: &GroundSample,
    field_elev_ft: f64,
    tpa_msl_ft: f64,
    shape: DescentShape,
) -> f64 {
    let platform = (field_elev_ft + shape.platform_agl_ft).min(tpa_msl_ft);
    match s.leg {
        DescentLeg::Platform => platform,
        DescentLeg::Base | DescentLeg::Final => platform.min(approach_leg_alt_on(
            field_elev_ft,
            tpa_msl_ft,
            s.path_m,
            shape.gp_ft_per_nm,
        )),
    }
}

/// The lowest point of a descent flown with `shape`: (clearance over the
/// ground, feet, less its floor; the sample). `None` without samples. Pure.
fn worst_descent_sample(
    samples: &[GroundSample],
    field_elev_ft: f64,
    tpa_msl_ft: f64,
    shape: DescentShape,
) -> Option<(f64, GroundSample)> {
    samples
        .iter()
        .map(|s| {
            let over = descent_target_ft(s, field_elev_ft, tpa_msl_ft, shape) - s.ground_ft;
            (over - descent_floor_ft(s.thr_m), *s)
        })
        .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal))
}

/// How a side's descent from abeam the threshold clears its ground.
#[derive(Debug, Clone, PartialEq)]
pub struct DescentCheck {
    /// The shape to fly: the default, or raised (platform, then a steeper
    /// path up to [`STEEP_PATH_MAX_DEG`]) to keep every sample over its
    /// floor. `None`: nothing legal clears it.
    pub shape: Option<DescentShape>,
    /// The least height over the ground the DEFAULT descent would have
    /// flown, feet (the number the old law never looked at), and where.
    pub default_min_ft: Option<f64>,
    /// ...and with the shape flown.
    pub flown_min_ft: Option<f64>,
    /// Spoken when refused: the worst point, no dashes.
    pub problem: Option<String>,
}

/// Check a side's descending legs at pattern altitude `tpa_msl_ft` and find
/// the least change that clears them: the downwind platform raised to the
/// highest sample's need (never over the pattern altitude), the path steepened
/// to the need of every base and final sample (never past
/// [`STEEP_PATH_MAX_DEG`]). No samples: the default, the old behaviour (no
/// probe, no verdict). Pure.
pub fn check_descent(
    samples: &[GroundSample],
    field_elev_ft: f64,
    tpa_msl_ft: f64,
) -> DescentCheck {
    check_descent_from(samples, field_elev_ft, tpa_msl_ft, DescentShape::default())
}

/// [`check_descent`] from a `default` descent other than the circuit's (its
/// 800 ft downwind platform): autoland's reversal holds its pattern altitude
/// until the path meets it (`reversal::judge`, #182). Pure.
pub fn check_descent_from(
    samples: &[GroundSample],
    field_elev_ft: f64,
    tpa_msl_ft: f64,
    default: DescentShape,
) -> DescentCheck {
    let over_ground = |shape: DescentShape| {
        samples
            .iter()
            .map(|s| descent_target_ft(s, field_elev_ft, tpa_msl_ft, shape) - s.ground_ft)
            .fold(None, |m: Option<f64>, v| Some(m.map_or(v, |m| m.min(v))))
    };
    let default_min_ft = over_ground(default);
    let Some((worst, at)) = worst_descent_sample(samples, field_elev_ft, tpa_msl_ft, default)
    else {
        return DescentCheck {
            shape: Some(default),
            default_min_ft: None,
            flown_min_ft: None,
            problem: None,
        };
    };
    if worst >= 0.0 {
        return DescentCheck {
            shape: Some(default),
            default_min_ft,
            flown_min_ft: default_min_ft,
            problem: None,
        };
    }
    let need = |s: &GroundSample| s.ground_ft + descent_floor_ft(s.thr_m);
    let platform_need = samples
        .iter()
        .map(need)
        .fold(field_elev_ft + default.platform_agl_ft, f64::max);
    let gp_need = samples
        .iter()
        .filter(|s| s.leg.on_path() && s.path_m > 1.0)
        .map(|s| (need(s) - field_elev_ft) / (s.path_m / M_PER_NM))
        .fold(default.gp_ft_per_nm, f64::max);
    let gp_max = gp_ft_per_nm_at(STEEP_PATH_MAX_DEG);
    if platform_need > tpa_msl_ft + 1.0 || gp_need > gp_max + 1e-6 {
        let over = descent_target_ft(&at, field_elev_ft, tpa_msl_ft, default) - at.ground_ft;
        let tail = if platform_need > tpa_msl_ft + 1.0 {
            "clearing it needs more than the pattern altitude".to_string()
        } else {
            format!(
                "clearing it needs a {:.1} degree path",
                (gp_need / 6076.12).atan().to_degrees()
            )
        };
        let problem = format!(
            "{} would pass {:.0} feet over {:.0} feet of ground {:.1} miles out, {tail}",
            at.leg.words(),
            (over.max(0.0) / 10.0).round() * 10.0,
            (at.ground_ft / 10.0).round() * 10.0,
            at.thr_m / M_PER_NM,
        );
        return DescentCheck {
            shape: None,
            default_min_ft,
            flown_min_ft: None,
            problem: Some(problem),
        };
    }
    let shape = DescentShape {
        platform_agl_ft: (platform_need - field_elev_ft).max(default.platform_agl_ft),
        gp_ft_per_nm: gp_need,
    };
    DescentCheck {
        shape: Some(shape),
        default_min_ft,
        flown_min_ft: over_ground(shape),
        problem: None,
    }
}

/// What a candidate pattern side knows about the world under and around it:
/// the highest sampled terrain under the legs flown AT pattern altitude,
/// corridor included (`None` = no probe data), the first no-go airspace
/// (prohibited/restricted/danger) its polyline enters at pattern height
/// (`None` = clear or no data), and the ground under its descending legs
/// (empty = no probe data, the old behaviour). `line_max_ft` is the drawn
/// rectangle alone, the number the old check used, for the record.
#[derive(Debug, Clone, Default)]
pub struct SideFacts {
    pub terrain_max_ft: Option<f64>,
    pub airspace: Option<String>,
    pub line_max_ft: Option<f64>,
    pub descent: Vec<GroundSample>,
}

/// The pattern the pick settled on.
#[derive(Debug, Clone, PartialEq)]
pub struct PatternPick {
    pub left: bool,
    pub tpa_msl_ft: f64,
    pub why: &'static str,
    /// The descent flown from abeam the threshold.
    pub shape: DescentShape,
    /// Each side's descent check at the altitude it was judged at (left,
    /// right), for the record.
    pub descent_left: DescentCheck,
    pub descent_right: DescentCheck,
    /// `Some`: no side can be flown clear of the ground. The spoken reason,
    /// with the numbers; the circuit is never flown.
    pub refused: Option<String>,
}

/// Decide the pattern side, altitude and descent — the generic rule, no
/// per-airport exceptions: fly the side clear of terrain (level legs by
/// [`PATTERN_TERRAIN_CLEAR_FT`], the descent by [`check_descent`]) and of
/// no-go airspace; switch sides when only the other one is; when neither is
/// clean, prefer the side without an airspace block, raised over its terrain;
/// when airspace is unavoidable, fall back to terrain-safety alone (terrain
/// kills, paper doesn't) and say so; and when no side's descent can be flown
/// clear, REFUSE with the numbers (#182). Missing data degrades gracefully:
/// `None` terrain = assume clear (the old behavior), `None` airspace = clear,
/// no descent samples = the default descent. Pure.
pub fn choose_pattern(
    default_left: bool,
    l: &SideFacts,
    r: &SideFacts,
    field_elev_ft: f64,
    tpa_agl_ft: f64,
) -> PatternPick {
    choose_pattern_with(
        default_left,
        l,
        r,
        field_elev_ft,
        tpa_agl_ft,
        PickRules::circuit(tpa_agl_ft),
    )
}

/// How [`choose_pattern_with`] raises a side and where its descent starts:
/// the circuit's rectangle, or autoland's reversal (`reversal::judge`, #182),
/// one rule behind both.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PickRules {
    /// A raised side flies this far over the highest ground under its level
    /// legs, at least [`PATTERN_TERRAIN_CLEAR_FT`].
    pub raise_over_ft: f64,
    /// The descent starts from the pattern altitude itself (the reversal's
    /// inbound holds its platform until the 3 degree path comes down to it),
    /// not from the circuit's 800 ft downwind platform.
    pub descend_from_pattern: bool,
}

impl PickRules {
    /// The circuit: a raised pattern keeps a pattern height over the ground.
    pub fn circuit(tpa_agl_ft: f64) -> Self {
        PickRules {
            raise_over_ft: tpa_agl_ft.max(PATTERN_TERRAIN_CLEAR_FT),
            descend_from_pattern: false,
        }
    }

    /// Autoland's reversal: the lowest legal platform (its join point moves
    /// out with every foot of it), descending on the path from it.
    pub fn reversal() -> Self {
        PickRules {
            raise_over_ft: PATTERN_TERRAIN_CLEAR_FT,
            descend_from_pattern: true,
        }
    }
}

/// [`choose_pattern`] under `rules`. Pure.
pub fn choose_pattern_with(
    default_left: bool,
    l: &SideFacts,
    r: &SideFacts,
    field_elev_ft: f64,
    tpa_agl_ft: f64,
    rules: PickRules,
) -> PatternPick {
    let tpa = field_elev_ft + tpa_agl_ft;
    let level_ok = |f: &SideFacts, at: f64| {
        f.terrain_max_ft
            .is_none_or(|m| at - m >= PATTERN_TERRAIN_CLEAR_FT - 0.5)
    };
    // Only as high as the level legs need, and at least a pattern height
    // over the ground: a 1,000 ft clearance under an 800 ft pattern would
    // otherwise leave it lower than the rule.
    let raise = |f: &SideFacts| match f.terrain_max_ft {
        Some(m) => (m + rules.raise_over_ft.max(PATTERN_TERRAIN_CLEAR_FT)).max(tpa),
        None => tpa,
    };
    let side_facts = |left: bool| if left { l } else { r };
    let from = |at: f64| {
        if rules.descend_from_pattern {
            DescentShape {
                platform_agl_ft: at - field_elev_ft,
                ..DescentShape::default()
            }
        } else {
            DescentShape::default()
        }
    };
    let descent = |left: bool, at: f64| {
        check_descent_from(&side_facts(left).descent, field_elev_ft, at, from(at))
    };
    let pick = |left: bool, at: f64, why: &'static str, shape: DescentShape| PatternPick {
        left,
        tpa_msl_ft: at,
        why,
        shape,
        descent_left: descent(true, if left { at } else { raise(l) }),
        descent_right: descent(false, if left { raise(r) } else { at }),
        refused: None,
    };
    let clean = |left: bool| {
        let f = side_facts(left);
        level_ok(f, tpa) && f.airspace.is_none()
    };
    let df = side_facts(default_left);
    let of = side_facts(!default_left);
    if clean(default_left) {
        if let Some(shape) = descent(default_left, tpa).shape {
            return pick(default_left, tpa, "default_side_clear", shape);
        }
    }
    if clean(!default_left) {
        if let Some(shape) = descent(!default_left, tpa).shape {
            let why = if df.airspace.is_some() {
                "switched_side_for_airspace"
            } else {
                "switched_side_for_terrain"
            };
            return pick(!default_left, tpa, why, shape);
        }
    }
    // Neither side is clean. Airspace can't be out-climbed at pattern heights
    // (R-164 runs to 30,000), so prefer the side WITHOUT a block, raised over
    // its terrain when needed; the other side, raised, is the second choice.
    let lower_left = {
        let (dr, or) = (raise(df), raise(of));
        if dr <= or + RAISE_SIDE_MARGIN_FT {
            default_left
        } else {
            !default_left
        }
    };
    let first = match (df.airspace.is_some(), of.airspace.is_some()) {
        (true, false) => !default_left,
        (false, true) => default_left,
        _ => lower_left,
    };
    let why_for = |left: bool| {
        let f = side_facts(left);
        if f.airspace.is_some() {
            "airspace_unavoidable"
        } else if raise(f) > tpa + 0.5 {
            "raised_for_terrain"
        } else if left != default_left {
            "switched_side_for_terrain"
        } else {
            "default_side_clear"
        }
    };
    for left in [first, !first] {
        let at = raise(side_facts(left));
        if let Some(shape) = descent(left, at).shape {
            return pick(left, at, why_for(left), shape);
        }
    }
    // Nothing legal clears the ground: never fly it.
    let side_word = |left: bool| if left { "left" } else { "right" };
    let problem = |left: bool| {
        descent(left, raise(side_facts(left)))
            .problem
            .unwrap_or_else(|| "its descent does not clear the ground".into())
    };
    let words = format!(
        "No circuit here clears the ground. On the {} side {}; on the {} side {}.",
        side_word(first),
        problem(first),
        side_word(!first),
        problem(!first)
    );
    PatternPick {
        refused: Some(words),
        ..pick(
            first,
            raise(side_facts(first)),
            "refused_for_terrain",
            DescentShape::default(),
        )
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
    densify(&pts, LEG_SAMPLE_M)
        .into_iter()
        .map(|(a, x)| frame_to_latlon(rwy, a, x))
        .collect()
}

/// One probe point of a pattern side, in the runway frame: the leg the
/// circuit flies there and whether it is on the drawn line.
#[derive(Debug, Clone, Copy, PartialEq)]
struct FramePt {
    a_m: f64,
    x_m: f64,
    leg: Leg,
    line: bool,
}

/// The rectangle moved `d_m` outward (negative: inward) from its drawn line on
/// every side, sampled every `step_m` and tagged by leg: the crosswind
/// further along, the downwind further out, the base further back, and the
/// final longer.
fn rectangle_offset(
    g: &Geometry,
    runway_len_m: f64,
    left: bool,
    d_m: f64,
    step_m: f64,
) -> Vec<FramePt> {
    let side = if left { -1.0 } else { 1.0 };
    let a_up = runway_len_m + g.upwind_past_m + d_m;
    let a_dn = -(g.downwind_past_m + d_m);
    let x_out = side * (g.width_m + d_m);
    let legs = [
        (Leg::Crosswind, (a_up, 0.0), (a_up, x_out)),
        (Leg::Downwind, (a_up, x_out), (a_dn, x_out)),
        (Leg::Base, (a_dn, x_out), (a_dn, 0.0)),
        (Leg::Final, (a_dn, 0.0), (0.0, 0.0)),
    ];
    let mut out = Vec::new();
    for (leg, from, to) in legs {
        for (a_m, x_m) in densify(&[from, to], step_m) {
            out.push(FramePt {
                a_m,
                x_m,
                leg,
                line: d_m == 0.0,
            });
        }
    }
    out
}

/// Every point probed for one pattern side: the drawn line every
/// [`LEG_SAMPLE_M`], and the corridor the autopilot actually flies
/// ([`CORRIDOR_OUT_RADII`], [`CORRIDOR_IN_RADII`]) every
/// [`CORRIDOR_SAMPLE_M`]. Pure.
fn corridor_frame(g: &Geometry, runway_len_m: f64, left: bool) -> Vec<FramePt> {
    let r = g.lead_m / 0.9;
    let mut out = rectangle_offset(g, runway_len_m, left, 0.0, LEG_SAMPLE_M);
    for k in CORRIDOR_OUT_RADII {
        out.extend(rectangle_offset(
            g,
            runway_len_m,
            left,
            k * r,
            CORRIDOR_SAMPLE_M,
        ));
    }
    out.extend(rectangle_offset(
        g,
        runway_len_m,
        left,
        -CORRIDOR_IN_RADII * r,
        CORRIDOR_SAMPLE_M,
    ));
    out
}

/// Inside this of the threshold the descent is autoland's (the circuit hands
/// over further out, and the last mile's ground is the runway's own slope,
/// which the path's floor would read as a hill).
pub const DESCENT_CHECK_FROM_M: f64 = M_PER_NM;

/// The part of the descent a probe point is flown on, as [`leg_target_ft`]
/// flies it: `None` for the legs flown level (the crosswind, the downwind
/// until abeam the threshold); the downwind after it is the platform until
/// its led turn, where the BASE leg begins; the base's path is judged at
/// the roll-out distance or further ([`path_dist_m`]). Pure.
fn descent_leg_at(p: &FramePt, g: &Geometry) -> Option<(DescentLeg, f64)> {
    match p.leg {
        Leg::Upwind | Leg::Crosswind => None,
        Leg::Downwind if p.a_m >= 0.0 => None,
        Leg::Downwind if p.a_m > -(g.downwind_past_m - g.lead_m) => {
            Some((DescentLeg::Platform, p.a_m.abs()))
        }
        Leg::Downwind | Leg::Base => Some((
            DescentLeg::Base,
            path_dist_m(Leg::Base, p.a_m, g.downwind_past_m),
        )),
        Leg::Final => Some((DescentLeg::Final, p.a_m.abs())),
    }
}

/// A side's terrain facts from its probe points and the ground under each
/// (feet MSL, same order): the drawn line's highest (the old number), the
/// highest under the legs flown level, corridor included, and the descent's
/// samples. Pure.
fn corridor_facts(pts: &[FramePt], ground_ft: &[f64], g: &Geometry) -> SideFacts {
    let mut f = SideFacts::default();
    let max = |m: Option<f64>, v: f64| Some(m.map_or(v, |m: f64| m.max(v)));
    for (p, &e) in pts.iter().zip(ground_ft) {
        if p.line {
            f.line_max_ft = max(f.line_max_ft, e);
        }
        match descent_leg_at(p, g) {
            None => f.terrain_max_ft = max(f.terrain_max_ft, e),
            Some((leg, path_m)) => {
                let thr_m = p.a_m.hypot(p.x_m);
                if thr_m >= DESCENT_CHECK_FROM_M {
                    f.descent.push(GroundSample {
                        leg,
                        path_m,
                        thr_m,
                        ground_ft: e,
                    });
                }
            }
        }
    }
    f
}

/// Probe one pattern side's corridor ([`corridor_frame`]) through the plugin
/// and read its facts ([`corridor_facts`]); airspace is the caller's. Any
/// unanswered chunk is no data at all (`SideFacts::default()`), never a
/// partial picture read as clear ground.
async fn probe_side(
    xp: &XPlaneClient,
    rwy: &crate::autoland::Runway,
    g: &Geometry,
    left: bool,
) -> SideFacts {
    let pts = corridor_frame(g, rwy.length_m, left);
    let lls: Vec<(f64, f64)> = pts
        .iter()
        .map(|p| frame_to_latlon(rwy, p.a_m, p.x_m))
        .collect();
    let mut ground = Vec::with_capacity(lls.len());
    for chunk in lls.chunks(crate::terrain::MAX_POINTS) {
        match crate::terrain::sample_ft(xp, chunk).await {
            Some(v) => ground.extend(v),
            None => return SideFacts::default(),
        }
    }
    corridor_facts(&pts, &ground, g)
}

/// The shortest `downwind_past_m` that still leaves autoland a final: the
/// base-to-final turn (`final_r_m`) plus [`FINAL_STRAIGHT_MIN_S`] of straight
/// final at the approach speed. Pure.
fn shortest_downwind_past_m(final_r_m: f64, approach_kts: f64) -> f64 {
    (final_r_m + FINAL_STRAIGHT_MIN_S * approach_kts.max(40.0) * 0.514_444).max(2000.0)
}

/// The least straight final a shortened pattern may leave, seconds: the 737
/// handed over at LEMG 13 with 46 s of final flew it 30 to 50 m off and
/// landed off the pavement (sortie 1790591256-full_auto, 2026-09-28).
const FINAL_STRAIGHT_MIN_S: f64 = 60.0;
/// Each shortening of the downwind tried when no side's descent clears.
const DOWNWIND_SHORTEN_STEP_M: f64 = 500.0;

/// The shorter downwinds to try, longest first, never under `min_m`. For a
/// jet the drawn `2 r + 1,200 m` is already under it and there are none. Pure.
fn shorter_downwinds(dn_m: f64, min_m: f64) -> Vec<f64> {
    let mut out = Vec::new();
    let mut d = dn_m - DOWNWIND_SHORTEN_STEP_M;
    while d >= min_m && out.len() < 4 {
        out.push(d);
        d -= DOWNWIND_SHORTEN_STEP_M;
    }
    out
}

/// Fill a frame polyline so no two consecutive samples are further apart
/// than `step_m`. The Rock's ridge is ~300 m wide and the rectangle's eight
/// downwind samples were 580 m apart: the probe reported 650 ft of terrain
/// under a side whose ridge stands at ~1,150 (2026-09-14).
pub fn densify(pts: &[(f64, f64)], step_m: f64) -> Vec<(f64, f64)> {
    let mut out = Vec::with_capacity(pts.len() * 4);
    for (i, &(a, x)) in pts.iter().enumerate() {
        if i > 0 {
            let (pa, px) = pts[i - 1];
            let len = ((a - pa).powi(2) + (x - px).powi(2)).sqrt();
            let n = (len / step_m).ceil() as usize;
            for k in 1..n {
                let f = k as f64 / n as f64;
                out.push((pa + (a - pa) * f, px + (x - px) * f));
            }
        }
        out.push((a, x));
    }
    out
}

/// How close (metres) to an authored waypoint counts as reaching it. Generous,
/// so the slow AP turn rolls through the corner instead of circling the point.
const AUTHORED_CAPTURE_M: f64 = 350.0;

/// How far the Rock circuit's downwind runs from the runway line, metres:
/// SOUTH of Europa Point (4,500 m from the centreline at the lighthouse),
/// so the loop goes AROUND the Rock, over the Strait. The old 1.3 x pattern
/// width put it 2,050 m out, over Middle Hill (2026-09-15, runway 09): the
/// probe read 1,098 ft under the leg, the raised pattern altitude cleared
/// that on paper, and the aircraft crossed the ridge at 1,412 ft STILL
/// CLIMBING, 454 ft over the ground, with the terrain check satisfied.
const LXGB_ROCK_CLEAR_M: f64 = 5_400.0;

/// The most climb the authored follower commands, feet per minute (the
/// `vs_raw` clamp), and the fraction of it the pre-flight climb model
/// credits: measured 550 to 650 fpm at 73 kt on the 172 against a 700 clamp.
const AUTHORED_CLIMB_FPM: f64 = 700.0;

/// ...or the airframe's own, when it climbs harder than a 172 does. The
/// figure above is a measurement OF a 172 ("550 to 650 fpm at 73 kt"), which
/// is exactly the kind of number that has no business capping a jet.
fn authored_climb_fpm() -> f64 {
    crate::procedure::climb_fpm().max(AUTHORED_CLIMB_FPM)
}
const AUTHORED_CLIMB_CREDIT: f64 = 0.75;

/// Feet gained per metre flown at the pattern speed, for the climb model.
fn authored_climb_gradient(pattern_kts: f64) -> f64 {
    let m_per_min = pattern_kts.max(40.0) * M_PER_NM / 60.0;
    AUTHORED_CLIMB_FPM * AUTHORED_CLIMB_CREDIT / m_per_min
}

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
    // The downwind runs south of Europa Point, whatever the airframe's
    // pattern width: the Rock is a fixed piece of granite, not a fraction
    // of the turn radius.
    let out = (w * 1.3).max(LXGB_ROCK_CLEAR_M);
    Some(vec![
        // Climb out on the runway heading.
        AuthoredWpt {
            a_m: up,
            x_m: 0.0,
            alt_msl_ft: tpa_msl_ft,
        },
        // Crosswind south over the sea off the Rock's east face, the length
        // of the Rock: the climb to pattern altitude happens here, over
        // water, before any leg turns back toward the granite.
        AuthoredWpt {
            a_m: up,
            x_m: side * out,
            alt_msl_ft: tpa_msl_ft,
        },
        AuthoredWpt {
            a_m: 0.0,
            x_m: side * out,
            alt_msl_ft: tpa_msl_ft,
        },
        AuthoredWpt {
            a_m: -dn,
            x_m: side * out,
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

/// How the authored route flies this tick's waypoint altitude (#80).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AuthoredVertical {
    /// A dial write owed: the waypoint's level to the hundred.
    pub dial_ft: Option<f64>,
    /// The autopilot's altitude hold has the level: the VS law stands off.
    pub ap_level: bool,
}

/// A captured hold this far from the level is no hold of ours: the dial was
/// just moved and the autopilot is still holding the old level.
pub const AUTHORED_HOLD_BAND_FT: f64 = 200.0;

/// The authored route's vertical where the airframe declares
/// `ap_altitude_capture`: each waypoint's altitude is a level target, so it
/// goes on the altitude dial (`circuit_dial_ft` with no rate: the route has
/// no glidepath, autoland parks the dial at the handoff), and a CAPTURED hold
/// on that dial within [`AUTHORED_HOLD_BAND_FT`] owns the leg
/// (`ap_holds_level`). The LXGB route flew its levels on the VS law behind
/// whatever the dial held (dev-history 2026-09-21: "the authored LXGB route
/// has no altitude capture"). Off, or with either read failed (no
/// measurement), nothing is written and the VS law flies it as before. Pure.
pub fn authored_vertical(
    ap_capture: bool,
    wp_alt_ft: f64,
    alt_ft: f64,
    dial_ft: Option<f64>,
    alt_hold_status: Option<f64>,
) -> AuthoredVertical {
    let none = AuthoredVertical { dial_ft: None, ap_level: false };
    if !ap_capture || !alt_ft.is_finite() || !wp_alt_ft.is_finite() {
        return none;
    }
    let (Some(dial), Some(hold)) = (
        dial_ft.filter(|v| v.is_finite()),
        alt_hold_status.filter(|v| v.is_finite()),
    ) else {
        return none;
    };
    match circuit_dial_ft(0.0, wp_alt_ft, dial, alt_ft) {
        Some(want) => AuthoredVertical { dial_ft: Some(want), ap_level: false },
        None => AuthoredVertical {
            dial_ft: None,
            ap_level: ap_holds_level(0.0, wp_alt_ft, dial, hold)
                && (alt_ft - wp_alt_ft).abs() <= AUTHORED_HOLD_BAND_FT,
        },
    }
}

/// The waypoint to fly to from `(a_m, x_m)` (runway frame): the end of the
/// route leg the aircraft is nearest to, or the one after it when the
/// aircraft is already past that leg's end. Leg 0 runs from the threshold
/// along the course to waypoint 0. The follower joins there. At LXGB
/// (2026-09-14) the takeoff sank at the AP engage and handed off 1.8 km PAST
/// the climb-out point, and the follower steered back to it: a 270-degree
/// turn at 60 kt, the wrong way round the first leg. Past the last leg →
/// the last waypoint (the autoland handoff).
fn join_waypoint(route: &[AuthoredWpt], a_m: f64, x_m: f64) -> usize {
    let mut best: Option<(f64, usize)> = None;
    for (i, wp) in route.iter().enumerate() {
        let (sa, sx) = if i == 0 {
            (0.0, 0.0)
        } else {
            (route[i - 1].a_m, route[i - 1].x_m)
        };
        let (da, dx) = (wp.a_m - sa, wp.x_m - sx);
        let len2 = da * da + dx * dx;
        if len2 < 1.0 {
            continue;
        }
        let t = (((a_m - sa) * da + (x_m - sx) * dx) / len2).clamp(0.0, 1.0);
        let (pa, px) = (sa + t * da, sx + t * dx);
        let dist = ((a_m - pa).powi(2) + (x_m - px).powi(2)).sqrt();
        let target = if t >= 0.999 { i + 1 } else { i };
        if best.is_none_or(|(d, _)| dist < d) {
            best = Some((dist, target));
        }
    }
    best.map_or(0, |(_, t)| t.min(route.len().saturating_sub(1)))
}

/// Spacing of the terrain samples along an authored leg, metres. The Rock's
/// ridge is ~300 m wide; six waypoints probed alone (2026-09-14) sat on sea
/// either side of it while the downwind between them crossed it 523 ft up.
const LEG_SAMPLE_M: f64 = 150.0;
/// The most the authored pattern altitude may be raised over its terrain
/// before the route is refused instead.
const MAX_TERRAIN_RAISE_FT: f64 = 1000.0;

/// Sample points along every leg of the route in the runway frame, tagged
/// with the index of the waypoint the leg runs INTO (legs 1..). Leg 0, the
/// climb-out from the runway to the first waypoint, is the takeoff's and is
/// not sampled.
fn leg_samples(route: &[AuthoredWpt]) -> Vec<(usize, f64, f64)> {
    let mut out = Vec::new();
    for i in 1..route.len() {
        let (p, q) = (route[i - 1], route[i]);
        let len = ((q.a_m - p.a_m).powi(2) + (q.x_m - p.x_m).powi(2)).sqrt();
        let n = ((len / LEG_SAMPLE_M).ceil() as usize).max(1);
        for k in 0..=n {
            let f = k as f64 / n as f64;
            out.push((i, p.a_m + (q.a_m - p.a_m) * f, p.x_m + (q.x_m - p.x_m) * f));
        }
    }
    out
}

/// The highest ground (feet MSL) under each leg of the route, indexed like
/// the waypoints (entry 0 unused). `None` for a leg = the probe did not
/// answer (no plugin): read as clear, the old fail-open behaviour.
async fn leg_terrain_ft(
    xp: &XPlaneClient,
    rwy: &crate::autoland::Runway,
    route: &[AuthoredWpt],
) -> Vec<Option<f64>> {
    let samples = leg_samples(route);
    let pts: Vec<(f64, f64)> = samples
        .iter()
        .map(|&(_, a, x)| frame_to_latlon(rwy, a, x))
        .collect();
    let mut out = vec![None; route.len()];
    let mut at = 0;
    for chunk in pts.chunks(crate::terrain::MAX_POINTS) {
        if let Some(elev) = crate::terrain::sample_ft(xp, chunk).await {
            for (k, e) in elev.into_iter().enumerate() {
                let leg = samples[at + k].0;
                out[leg] = Some(out[leg].map_or(e, |m: f64| m.max(e)));
            }
        }
        at += chunk.len();
    }
    out
}

/// The altitude (feet MSL) the climb can have reached at each waypoint:
/// the field elevation plus `gradient_ft_per_m` over the path flown from
/// the threshold through every waypoint before it. Uncapped; the caller
/// caps it at the altitude actually asked for.
fn climb_reach_ft(route: &[AuthoredWpt], field_elev_ft: f64, gradient_ft_per_m: f64) -> Vec<f64> {
    let mut out = Vec::with_capacity(route.len());
    let (mut pa, mut px, mut len) = (0.0_f64, 0.0_f64, 0.0_f64);
    for w in route {
        len += ((w.a_m - pa).powi(2) + (w.x_m - px).powi(2)).sqrt();
        (pa, px) = (w.a_m, w.x_m);
        out.push(field_elev_ft + gradient_ft_per_m * len);
    }
    out
}

/// Raise the route's pattern altitude until every leg flown AT pattern
/// altitude clears its terrain by [`AUTHORED_TERRAIN_CLEAR_FT`], or refuse. Legs that
/// descend to the handoff height (into a waypoint below the TPA) cannot be
/// raised and are refused when their terrain needs more than the height
/// they end at; a raise past [`MAX_TERRAIN_RAISE_FT`] is refused too.
///
/// "Flown at pattern altitude" is only true once the climb has got there:
/// the follower leaves the takeoff's handoff at ~500 ft and climbs along
/// the route, so a leg with terrain under it is also checked against the
/// altitude the climb ([`climb_reach_ft`], `gradient_ft_per_m`) can have
/// reached at the leg's START, and refused when the terrain needs more.
/// Runway 09 at LXGB (2026-09-15): a 2,108 ft pattern altitude cleared the
/// 1,098 ft under the downwind on paper, and the aircraft entered that leg
/// at 1,130 ft.
/// Returns the route to fly and the pattern altitude it flies.
fn raise_for_terrain(
    route: &[AuthoredWpt],
    tpa_msl_ft: f64,
    leg_max_ft: &[Option<f64>],
    field_elev_ft: f64,
    gradient_ft_per_m: f64,
) -> Result<(Vec<AuthoredWpt>, f64), String> {
    let at_tpa = |w: &AuthoredWpt| w.alt_msl_ft >= tpa_msl_ft - 1.0;
    let reach = climb_reach_ft(route, field_elev_ft, gradient_ft_per_m);
    let mut tpa_needed = tpa_msl_ft;
    for i in 1..route.len() {
        let Some(Some(ground)) = leg_max_ft.get(i) else {
            continue;
        };
        let need = ground + AUTHORED_TERRAIN_CLEAR_FT;
        if at_tpa(&route[i - 1]) && at_tpa(&route[i]) {
            tpa_needed = tpa_needed.max(need);
            let entry = reach[i - 1].min(tpa_needed);
            if need > entry {
                return Err(format!(
                    "leg {i} needs {need:.0} ft over {ground:.0} ft ground, entered at {entry:.0} ft in the climb"
                ));
            }
        } else {
            let floor = route[i - 1].alt_msl_ft.min(route[i].alt_msl_ft);
            if need > floor {
                return Err(format!(
                    "leg {i} needs {need:.0} ft over {ground:.0} ft ground, flown down to {floor:.0}"
                ));
            }
        }
    }
    if tpa_needed > tpa_msl_ft + MAX_TERRAIN_RAISE_FT {
        return Err(format!(
            "pattern altitude would need {tpa_needed:.0} ft, {:.0} over the TPA",
            tpa_needed - tpa_msl_ft
        ));
    }
    let raised = route
        .iter()
        .map(|w| {
            let mut w = *w;
            if at_tpa(&w) {
                w.alt_msl_ft = tpa_needed;
            }
            w
        })
        .collect();
    Ok((raised, tpa_needed))
}

/// The authored route checked over its legs' ground ([`leg_terrain_ft`],
/// [`raise_for_terrain`]): the ground per leg and the route to fly at its
/// pattern altitude, or why not. The same answer on the runway and in the
/// air (the route is fixed to the runway, not to the aircraft).
async fn check_authored_route(
    xp: &XPlaneClient,
    rwy: &crate::autoland::Runway,
    route: &[AuthoredWpt],
    tpa_msl_ft: f64,
    field_elev_ft: f64,
    pattern_kts: f64,
) -> (Vec<Option<f64>>, Result<(Vec<AuthoredWpt>, f64), String>) {
    let leg_max = leg_terrain_ft(xp, rwy, route).await;
    let verdict = raise_for_terrain(
        route,
        tpa_msl_ft,
        &leg_max,
        field_elev_ft,
        authored_climb_gradient(pattern_kts),
    );
    (leg_max, verdict)
}

/// The Rock refused both ways (#182): the spoken line, the rectangle's
/// numbers in it. Pure.
pub fn rock_refused_line(rectangle_line: &str) -> String {
    format!("The Rock circuit isn't clear from here. {rectangle_line}")
}

/// Height above the field at which the authored circuit switches the landing
/// and taxi lights off (the rectangle and full auto's flight do it through
/// the same `lights::after_climb`).
const LIGHTS_OFF_AGL_FT: f64 = 1000.0;

/// Fly an authored waypoint route (the LXGB Rock circuit), then hand the final
/// to autoland. Same control surface as the rectangle loop — HDG bug + VS + the
/// pattern speed-hold — but steering great-circle to each waypoint instead of
/// the leg geometry. The last waypoint sits on short final.
///
/// Joins at the nearest leg (`join_waypoint`), never
/// steps the bug against a turn in progress (`bug_toward`), climbs only as
/// fast as the airspeed pays for (`climb_limit_fpm`, the rectangle's own
/// law: this loop clamped a flat 700 fpm and flew the first turn at 60 kt),
/// and gives the aircraft back after [`MAX_STALLS`] legs it could not close.
#[allow(clippy::too_many_arguments)]
async fn fly_authored_route(
    xp: XPlaneClient,
    base_url: String,
    voice: Option<Voice>,
    route: Vec<AuthoredWpt>,
    rwy: crate::autoland::Runway,
    mag_var: f64,
    pattern_kts: f64,
    full_kts: f64,
    floor_kts: f64,
    callsign: Option<String>,
    apt_dat: String,
    cfg: crate::tools::autopilot::ApConfig,
    approach_ias: f64,
) {
    let rwy_id = rwy.id.clone();
    let mut bug_mag = autoflight::read_dial(&xp, Dial::HeadingBug).await.unwrap_or(rwy.course_deg);
    // A new flight of the laws: whatever the knob holds now is unknown.
    LAST_BUG.store(u64::MAX, std::sync::atomic::Ordering::Relaxed);
    let mut last_vs_cmd: Option<f64> = None;
    let mut prev_ias: Option<(f64, std::time::Instant)> = None;
    let mut ias_trend = 0.0_f64;
    let t_start = std::time::Instant::now();
    let mut bug_hold = BugHold::default();
    // The lateral re-engage (`ap_engage::ServoWatch`): FD first, confirmed.
    let mut servo_watch = crate::ap_engage::ServoWatch::default();
    // Picked from the first good position read, not assumed to be 0.
    let mut wpt_idx: Option<usize> = None;
    let mut closure: Option<Closure> = None;
    let mut stalls = 0u32;
    let mut lights_done = false;
    // The autopilot's own capture on the level legs, where the airframe
    // declares it (`ap_altitude_capture`), as the rectangle does (#80).
    let ap_capture = AP_ALTITUDE_CAPTURE.load(Ordering::SeqCst);
    let mut vs_refused: Option<std::time::Instant> = None;
    let mut vs_refused_said = false;
    loop {
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        if xp.cancelled() {
            stand_down(
                &xp,
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
        // Position, elevation and heading as ONE frame saw them (#157).
        let (pos, hp_r, phi_r, hdg_st, vs_st, ias_r, thr_r, servos_r) = tokio::join!(
            sim::position::read(&xp),
            xp.get_float(HPATH),
            xp.get_float(PHI),
            xp.get_float(HDG_STATUS),
            xp.get_float(VS_STATUS),
            xp.get_float(IAS),
            xp.get_float(&thr_dref),
            xp.get_float(SERVOS_ON),
        );
        let (lat_r, lon_r) = sim::position::lat_lon(&pos);
        let elev_r = sim::position::field(&pos, |p| p.elev_m);
        let psi_r = sim::position::field(&pos, |p| p.psi_true_deg);
        // Read only where the capture is declared: nothing else uses them.
        let (dial_r, alt_hold_r) = if ap_capture {
            let (d, h) =
                tokio::join!(autoflight::read_dial(&xp, Dial::Altitude), xp.get_float(ALT_HOLD_STATUS));
            (d.ok(), h.ok())
        } else {
            (None, None)
        };
        let (Ok(lat), Ok(lon)) = (lat_r, lon_r) else {
            continue;
        };
        let alt_msl_ft = elev_r.map(|m| m * M_TO_FT).unwrap_or(f64::NAN);
        let (hdg_read, vs_read) = (hdg_st.ok(), vs_st.ok());
        let (hdg_on, vs_on) = (hdg_read.unwrap_or(1.0) > 0.5, vs_read.unwrap_or(1.0) > 0.5);
        // Position in the departure-runway frame (along the takeoff course
        // from the threshold, right positive) — the authored waypoints' frame.
        let a_m = -along_track_nm(lat, lon, &rwy) * M_PER_NM;
        let x_m = cross_track_nm(lat, lon, &rwy) * M_PER_NM;
        let idx = match wpt_idx {
            Some(i) => i,
            None => {
                let i = join_waypoint(&route, a_m, x_m);
                crate::blackbox::record(
                    "circuit",
                    serde_json::json!({
                        "event": "authored_join", "wpt": i, "skipped": i, "a_m": a_m, "x_m": x_m,
                    }),
                );
                wpt_idx = Some(i);
                i
            }
        };
        let wp = route[idx];
        let (wlat, wlon) = frame_to_latlon(&rwy, wp.a_m, wp.x_m);
        // Crab so the AP holds the ground TRACK to the waypoint, like the legs.
        let psi_r = psi_r.ok();
        let crab = match (psi_r, hp_r) {
            (Some(psi), Ok(hp)) => {
                crate::angles::signed_diff_deg(psi, hp).clamp(-25.0, 25.0)
            }
            _ => 0.0,
        };
        let want_mag = true_to_mag(
            crate::nav::bearing_deg(lat, lon, wlat, wlon) + crab,
            mag_var,
        );
        let phi = phi_r.unwrap_or(0.0);
        bug_mag = match psi_r {
            Some(psi) => bug_toward(
                bug_mag,
                want_mag,
                true_to_mag(psi, mag_var),
                phi,
                BUG_SLEW_PER_TICK,
            ),
            None => slew_bug(bug_mag, want_mag, BUG_SLEW_PER_TICK),
        };
        // Set, not rewritten: per waypoint, a correction past the deadband
        // (`BugHold`, issue #93).
        hold_bug(
            &xp,
            &mut bug_hold,
            idx as u32,
            &mut bug_mag,
            t_start.elapsed().as_secs_f64(),
            CIRCUIT_LEG_DEADBAND,
        )
        .await;
        reengage_lateral(
            &xp,
            &mut servo_watch,
            &cfg.fd_commands,
            t_start.elapsed().as_secs_f64(),
            !hdg_on,
            servos_r.as_ref().ok().copied(),
            ias_r.as_ref().ok().copied().unwrap_or(f64::NAN),
            "authored",
        )
        .await;
        autoflight::bring_heading(&xp, hdg_read, "circuit_authored").await;

        // Airspeed trend first: both the climb limiter and the throttle law
        // judge the speed they are heading for.
        let ias = ias_r.as_ref().ok().copied().unwrap_or(f64::NAN);
        if ias.is_finite() {
            let t_now = std::time::Instant::now();
            if let Some((prev, t_prev)) = prev_ias {
                let dt = t_now.duration_since(t_prev).as_secs_f64();
                if (0.1..=2.0).contains(&dt) {
                    ias_trend = 0.6 * ias_trend + 0.4 * (ias - prev) / dt;
                }
            }
            prev_ias = Some((ias, t_now));
        }

        // Vertical: VS toward the waypoint altitude (the takeoff's proven
        // recipe), and only as much climb as the airspeed will pay for.
        let vs_raw = if alt_msl_ft.is_finite() {
            ((wp.alt_msl_ft - alt_msl_ft) * 3.0).clamp(-500.0, authored_climb_fpm())
        } else {
            0.0
        };
        let vs_cmd = climb_limit_fpm(vs_raw, ias, ias_trend, full_kts, floor_kts);
        // Each waypoint's altitude is a LEVEL target: with the capture
        // declared it goes on the dial, and a captured hold on it owns the
        // leg while the VS law stands off (`authored_vertical`).
        let vert = authored_vertical(ap_capture, wp.alt_msl_ft, alt_msl_ft, dial_r, alt_hold_r);
        if let Some(want) = vert.dial_ft {
            let ok = autoflight::set_dial(&xp, Dial::Altitude, want).await.is_ok();
            crate::blackbox::record(
                "circuit",
                serde_json::json!({"event": "alt_dial", "was": dial_r, "now": want,
                                   "wpt": idx, "ok": ok, "authored": true}),
            );
        }
        if vert.ap_level {
            // Whatever the VS dial says is stale the moment VS re-engages.
            last_vs_cmd = None;
            vs_refused = None;
        } else {
            if autoflight::bring_vs(&xp, vs_read, "circuit_authored")
                .await
                .pressed(Button::VerticalSpeed)
            {
                last_vs_cmd = None;
            }
            if last_vs_cmd.is_none_or(|v| (v - vs_cmd).abs() > 40.0) {
                let _ = autoflight::set_dial(&xp, Dial::Vs, vs_cmd).await;
                last_vs_cmd = Some(vs_cmd);
            }
            // The capture rests on VS taking the aircraft back OUT of the
            // hold for the next level: an autopilot that will not is said
            // once, with a base leg's height still to act on.
            if ap_capture && !vs_on {
                let since = *vs_refused.get_or_insert_with(std::time::Instant::now);
                if vs_stuck(since.elapsed().as_secs_f64()) && !vs_refused_said {
                    vs_refused_said = true;
                    crate::blackbox::record(
                        "circuit",
                        serde_json::json!({"event": "vs_refused", "wpt": idx, "cmd_vs": vs_cmd, "authored": true}),
                    );
                    alert(
                        &voice,
                        Priority::Warning,
                        "The autopilot won't take vertical speed. Check the vertical mode.",
                    );
                }
            } else {
                vs_refused = None;
            }
        }

        // Power holds the pattern speed — the same law the rectangle loop uses.
        if let (true, Ok(thr)) = (ias.is_finite(), thr_r) {
            let delta = crate::tools::speedhold::throttle_adjust_live(pattern_kts - ias, ias_trend);
            if delta != 0.0 {
                let _ = xp
                    .set_float(&throttle_dref(), (thr + delta).clamp(0.0, 1.0))
                    .await;
            }
        }

        // Landing and taxi lights off through 1,000 ft, as the rectangle does.
        if !lights_done && alt_msl_ft.is_finite() && alt_msl_ft - rwy.elev_ft > LIGHTS_OFF_AGL_FT {
            lights_done = true;
            let did = crate::tools::lights::after_climb(&xp).await;
            crate::blackbox::record(
                "circuit",
                serde_json::json!({"event": "lights_climb", "off": did}),
            );
        }

        let d_m = crate::nav::haversine_nm(lat, lon, wlat, wlon) * M_PER_NM;
        crate::blackbox::record(
            "circuit",
            serde_json::json!({
                "event": "authored_leg", "wpt": idx, "d_m": d_m, "bug": bug_mag,
                "alt_msl": alt_msl_ft, "tgt_alt": wp.alt_msl_ft, "cmd_vs": vs_cmd,
                "vs_raw": vs_raw, "ias": ias, "phi": phi, "ap_level": vert.ap_level,
            }),
        );

        if d_m < AUTHORED_CAPTURE_M {
            wpt_idx = Some(idx + 1);
            closure = None;
            if idx + 1 >= route.len() {
                break;
            }
            continue;
        }
        // Not closing on the waypoint for a whole [`STALL_S`]: orbiting, or
        // blown off the leg. Level off (bug on the nose) and let the next
        // ticks re-steer; three of those and the aircraft goes back.
        let now_s = t_start.elapsed().as_secs_f64();
        let stalled = closure
            .get_or_insert_with(|| Closure::new(d_m, now_s))
            .stalled(d_m, now_s);
        if stalled {
            stalls += 1;
            crate::blackbox::record(
                "circuit",
                serde_json::json!({
                    "event": "authored_stalled", "wpt": idx, "d_m": d_m, "stalls": stalls,
                }),
            );
            if stalls >= MAX_STALLS {
                let _ = autoflight::set_dial(&xp, Dial::Vs, 0.0).await;
                stand_down(
                    &xp,
                    &voice,
                    "I can't make the circuit's next point. Holding heading and altitude. Your aircraft.",
                    "authored_stalled_out",
                );
                return;
            }
            alert(
                &voice,
                Priority::Caution,
                "Not closing on the next point. Levelling off and re-joining.",
            );
            if let Some(psi) = psi_r {
                bug_mag = true_to_mag(psi, mag_var);
                bug_hold.reset();
                write_bug(&xp, bug_mag).await;
                bug_hold.decide(idx as u32, bug_mag, t_start.elapsed().as_secs_f64(), CIRCUIT_LEG_DEADBAND);
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
    // FORCE the pattern's own runway. Bare `None` sends autoland to its
    // EMERGENCY behaviour — search the whole apt.dat for the best end by track
    // — which is right for an engine failure and catastrophically wrong here:
    // we are on short final to a runway we just took off from and flew a
    // pattern around, and the search does not know that.
    // The circuit's own token is the autoland's parent: a stand-down landing
    // around the launch cancels both (`handed_off` keeps the circuit's
    // normal exit from doing it).
    crate::autoland::run_autoland_scoped(
        base_url,
        apt_dat,
        cfg,
        approach_ias,
        voice,
        None,
        // No ICAO needed: this is the runway we just took off from and have
        // been circling for minutes, a mile away. Proximity IS the right
        // answer here — unlike an arrival, which knows its destination and
        // must say so (see `PlannedRunway`).
        Some(crate::autoland::PlannedRunway::anywhere(&rwy_id)),
        xp.cancellation(),
        // A circuit final is judged at the visual gate, 500 ft (#199).
        crate::autoland::HandDown::circuit(None),
    );
    handed_off(&xp);
}

/// Spawn the circuit orchestrator on its own thread + runtime + client, the
/// same shape as the takeoff and autoland loops.
pub fn start(base_url: String, voice: Option<Voice>, p: Params) {
    let Some(run) = RUN.claim() else {
        return;
    };
    *STOOD_DOWN.lock().unwrap_or_else(|e| e.into_inner()) = None;
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            return;
        };
        // The run's token scopes the circuit's client: its cancel test, its
        // writes, and the parent of every loop it launches.
        let xp = XPlaneClient::new(base_url.clone()).with_cancel(run.cancel.clone());
        rt.block_on(fly(xp, base_url, voice, p));
        // (base_url is cloned into the client and passed on for the sub-loops.)
        drop(run);
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

/// Why the running (or last) circuit stood down, `None` while it has not:
/// full auto's circuit mission ends as the takeoff's give-up when the
/// takeoff never completed, instead of a silent "ended" (LEMG 13,
/// 2026-09-28, sortie 1790627884-full_auto: a rejected takeoff ended the
/// mission as `no_autoland`).
static STOOD_DOWN: std::sync::Mutex<Option<&'static str>> = std::sync::Mutex::new(None);

/// The reason the last circuit stood down, if it did.
pub fn stood_down() -> Option<&'static str> {
    *STOOD_DOWN.lock().unwrap_or_else(|e| e.into_inner())
}

/// The circuit's takeoff never completed: a reject, a pilot taking the
/// roll, or a takeoff that handed back on the ground.
pub const TAKEOFF_NOT_COMPLETED: &str = "takeoff_not_completed";
/// The circuit was refused on the runway: no side of it clears the ground
/// (#182). Never flown.
pub const TERRAIN_REFUSED: &str = "terrain_refused";

/// The spoken line for a descent raised over the ground (#182). No dashes.
/// Pure.
pub fn descent_shape_line(shape: DescentShape, field_elev_ft: f64) -> String {
    let mut parts = Vec::new();
    if shape.platform_agl_ft > DOWNWIND_DESCENT_AGL_FT + 1.0 {
        parts.push(format!(
            "holding {:.0} feet on the downwind",
            ((field_elev_ft + shape.platform_agl_ft) / 50.0).ceil() * 50.0
        ));
    }
    if shape.steep() {
        parts.push(format!(
            "a {:.1} degree approach path",
            shape.glidepath_deg()
        ));
    }
    format!("High ground under the approach, {}.", parts.join(", then "))
}

/// The side the circuit asks the terrain pick for (#182): the runway's
/// published pattern side (apt.dat `1101`; LEMG 13 and 31 left) over the
/// one asked for, which only a runway without one keeps. Terrain and
/// airspace still veto it in [`choose_pattern`], and say so. Pure.
pub fn pattern_side_wanted(asked_left: bool, published_left: Option<bool>) -> bool {
    published_left.unwrap_or(asked_left)
}

/// What the crew says about the field's circuit data before the takeoff
/// (#182): a published side that is not the one asked for, and a field
/// whose data says no circuits (`1302 allows_circuits 0`; LEMG, EGKK),
/// flown as asked. `None` when there is nothing to say. No dashes. Pure.
pub fn field_circuit_line(
    field: &str,
    runway: &str,
    asked_left: bool,
    published_left: Option<bool>,
    allows_circuits: Option<bool>,
) -> Option<String> {
    let mut parts = Vec::new();
    if allows_circuits == Some(false) {
        parts.push(format!(
            "The airport data says no circuits at {field}, flying this one as asked."
        ));
    }
    if let Some(left) = published_left.filter(|l| *l != asked_left) {
        parts.push(format!(
            "Runway {} has a published {} hand circuit, flying it {}.",
            spell_runway(runway),
            if left { "left" } else { "right" },
            if left { "left" } else { "right" },
        ));
    }
    (!parts.is_empty()).then(|| parts.join(" "))
}

/// Bail out cleanly: clear our flag and let whatever sub-loop is flying stand
/// down. Never leaves the aircraft with nobody watching the loop.
fn stand_down(xp: &XPlaneClient, voice: &Option<Voice>, line: &str, reason: &'static str) {
    *STOOD_DOWN.lock().unwrap_or_else(|e| e.into_inner()) = Some(reason);
    lower_flag(xp);
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
            &xp,
            &voice,
            "I can't read our position. The circuit is off, your aircraft.",
            "no_position",
        );
        return;
    };
    let cands = nearby_runways(&p.apt_dat, lat, lon, 3.0).unwrap_or_default();
    let Some((rwy, _)) = autotakeoff::runway_fit(&cands, lat, lon, psi, p.takeoff.needed_m) else {
        stand_down(
            &xp,
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
    // The field's own circuit data (#182): apt.dat's `1302 allows_circuits`
    // and the runway's `1101` pattern side. LEMG says no circuits and 13
    // left, and the 737 has flown circuits there nightly on whatever side
    // the terrain picked.
    let field = {
        let (path, icao) = (p.apt_dat.clone(), rwy.icao.clone());
        tokio::task::spawn_blocking(move || crate::nav::find_airport(&path, &icao).ok().flatten())
            .await
            .ok()
            .flatten()
    };
    let published_left = field.as_ref().and_then(|a| a.pattern_left(&rwy.id));
    let allows_circuits = field.as_ref().and_then(|a| a.allows_circuits);
    let want_left = pattern_side_wanted(p.left_pattern, published_left);

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
    let p_approach_ias = p.approach_ias;
    // Two radii, because the base-to-final turn is flown slower than the
    // pattern and is the one dimension where an over-estimate rolls us out
    // short of the centreline rather than merely wide.
    let mut geom =
        Geometry::for_radii(turn_radius_m(gs_est_mps), final_turn_radius_m(approach_kts));
    // The floor under the climb's speed protection, captured before `p` moves.
    let p_climb_floor_kts = p.climb_floor_kts;
    let p_vs_clean_kts = p.vs_clean_kts;
    // ...and the speed the climb is flown at, for the limiter's full-rate
    // threshold — NOT the leg speed the throttle holds (see
    // `climb_full_rate_kts` for the four knots that cost a circuit).
    let p_vy_kts = p.takeoff.vy_kts;

    // The world under BOTH candidate patterns: terrain probed from the sim
    // itself over the corridor the autopilot really flies (`corridor_frame`),
    // and the no-go airspace polygons the G1000 draws (OpenAir file). One
    // generic rule picks the side, height and descent; missing data degrades
    // to the old behavior, never to a guess.
    let poly_l = pattern_polyline(&rwy, &geom, rwy.length_m, true);
    let poly_r = pattern_polyline(&rwy, &geom, rwy.length_m, false);
    let spaces = {
        let path = p.airspace_path.clone();
        tokio::task::spawn_blocking(move || crate::airspace::load_near(&path, lat, lon, 0.5))
            .await
            .unwrap_or_default()
    };
    let tpa_probe_ft = field_elev_ft + p.tpa_agl_ft;
    let l_air =
        crate::airspace::first_conflict(&spaces, &poly_l, tpa_probe_ft).map(|a| a.spoken_name());
    let r_air =
        crate::airspace::first_conflict(&spaces, &poly_r, tpa_probe_ft).map(|a| a.spoken_name());
    let probe = |g: Geometry| {
        let (xp, rwy, l_air, r_air) = (&xp, &rwy, l_air.clone(), r_air.clone());
        async move {
            let l = SideFacts {
                airspace: l_air,
                ..probe_side(xp, rwy, &g, true).await
            };
            let r = SideFacts {
                airspace: r_air,
                ..probe_side(xp, rwy, &g, false).await
            };
            (l, r)
        }
    };
    let (mut l_facts, mut r_facts) = probe(geom).await;
    let mut pick = choose_pattern(
        want_left,
        &l_facts,
        &r_facts,
        field_elev_ft,
        p.tpa_agl_ft,
    );
    let lxgb = rwy.icao == "LXGB";
    // No side's descent clears the ground: a shorter downwind brings the base
    // in over other ground, where the geometry still leaves autoland a final
    // (`shortest_downwind_past_m`; a jet's drawn one is already the least).
    if pick.refused.is_some() && !lxgb {
        let least = shortest_downwind_past_m(geom.final_capture_m / 1.3, p_approach_ias);
        for dn in shorter_downwinds(geom.downwind_past_m, least) {
            let g = Geometry {
                downwind_past_m: dn,
                ..geom
            };
            let (l, r) = probe(g).await;
            let again = choose_pattern(want_left, &l, &r, field_elev_ft, p.tpa_agl_ft);
            crate::blackbox::record(
                "circuit",
                serde_json::json!({"event": "downwind_shortened", "downwind_past_m": dn,
                                   "refused": again.refused.is_some()}),
            );
            if again.refused.is_none() {
                (geom, l_facts, r_facts, pick) = (g, l, r, again);
                break;
            }
        }
    }
    let tpa_msl_ft = pick.tpa_msl_ft;
    // Gibraltar: the ONLY safe side is SOUTH — over the sea, looping around the
    // Rock. NORTH is Spain (restricted, foreign soil). choose_pattern picks the
    // LOWER-TERRAIN side, which at LXGB is the flat isthmus to the north, and it
    // flew the pattern straight into Spanish airspace (2026-07-26). Override to
    // the sea side; the runway sets the turn direction — 09 loops right
    // (clockwise), 27 loops left (anticlockwise), both staying over the water.
    // The authored route checks its own legs (`raise_for_terrain`); the
    // rectangle it may fall back to flies the forced side's own descent.
    let (left, side_pick, descent_shape, refused) = if lxgb {
        let left = sea_side_is_left(rwy_course);
        let forced = check_descent(
            &if left { &l_facts } else { &r_facts }.descent,
            field_elev_ft,
            tpa_msl_ft,
        );
        let refused = forced.shape.is_none().then(|| {
            format!(
                "The standard pattern here does not clear the ground: {}.",
                forced.problem.clone().unwrap_or_default()
            )
        });
        (
            left,
            "lxgb_south_over_the_sea",
            forced.shape.unwrap_or_default(),
            refused,
        )
    } else {
        (pick.left, pick.why, pick.shape, pick.refused.clone())
    };
    let check_json = |c: &DescentCheck| {
        serde_json::json!({
            "default_min_ft": c.default_min_ft, "flown_min_ft": c.flown_min_ft,
            "platform_agl_ft": c.shape.map(|s| s.platform_agl_ft),
            "glidepath_deg": c.shape.map(|s| s.glidepath_deg()),
            "problem": c.problem,
        })
    };
    crate::blackbox::record(
        "circuit",
        serde_json::json!({
            "event": "armed", "runway": rwy.id, "icao": rwy.icao,
            "course": rwy_course, "field_elev_ft": field_elev_ft,
            "tpa_msl_ft": tpa_msl_ft, "left": left, "side_pick": side_pick,
            "left_max_ft": l_facts.line_max_ft, "right_max_ft": r_facts.line_max_ft,
            "left_corridor_max_ft": l_facts.terrain_max_ft,
            "right_corridor_max_ft": r_facts.terrain_max_ft,
            "left_descent": check_json(&pick.descent_left),
            "right_descent": check_json(&pick.descent_right),
            "descent_platform_agl_ft": descent_shape.platform_agl_ft,
            "glidepath_deg": descent_shape.glidepath_deg(),
            "refused": refused,
            "left_airspace": l_facts.airspace, "right_airspace": r_facts.airspace,
            "airspaces_near": spaces.len(),
            "radius_m": geom.final_capture_m, "width_m": geom.width_m,
            "downwind_past_m": geom.downwind_past_m,
            "asked_left": p.left_pattern, "published_left": published_left,
            "allows_circuits": allows_circuits,
        }),
    );
    // Never flown into the ground (#182): refused on the runway, with the
    // numbers. The Rock's authored route is checked after the takeoff, and
    // its own refusal hands back there.
    if let (Some(line), false) = (refused.as_deref(), lxgb) {
        stand_down(&xp, &voice, &format!("{line} Your aircraft."), TERRAIN_REFUSED);
        return;
    }
    // ...and at the Rock, where the rectangle is refused, the authored route
    // is judged HERE too, on the runway, before anything flies: its legs are
    // fixed to the runway, so the answer after the takeoff is the same one.
    // Refused both ways in the air, the circuit handed an airborne aircraft
    // to nobody with "Your aircraft" (#182 review of 528b7d1, 2026-09-29).
    if let (Some(line), true) = (refused.as_deref(), lxgb) {
        let route = authored_circuit(&rwy, &geom, field_elev_ft, tpa_msl_ft, left);
        let why = match &route {
            Some(route) => {
                check_authored_route(&xp, &rwy, route, tpa_msl_ft, field_elev_ft, pattern_kts)
                    .await
                    .1
                    .err()
            }
            None => Some("no authored route".to_string()),
        };
        if let Some(why) = why {
            crate::blackbox::record(
                "circuit",
                serde_json::json!({"event": "authored_route_unsafe", "why": why, "on": "runway"}),
            );
            stand_down(
                &xp,
                &voice,
                &format!("{} Your aircraft.", rock_refused_line(line)),
                TERRAIN_REFUSED,
            );
            return;
        }
    }
    let side_word = |l: bool| if l { "left" } else { "right" };
    match side_pick {
        "switched_side_for_terrain" => alert(
            &voice,
            Priority::Caution,
            &format!(
                "Terrain on the {} side, flying a {}-hand circuit instead.",
                side_word(want_left),
                side_word(left)
            ),
        ),
        "switched_side_for_airspace" => {
            let name = if want_left {
                &l_facts.airspace
            } else {
                &r_facts.airspace
            };
            alert(
                &voice,
                Priority::Caution,
                &format!(
                    "The {} side runs into {}, flying a {}-hand circuit instead.",
                    side_word(want_left),
                    name.as_deref().unwrap_or("restricted airspace"),
                    side_word(left)
                ),
            );
        }
        "raised_for_terrain" if tpa_msl_ft - tpa_probe_ft >= RAISE_SAID_FT => alert(
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
    // The field's data, said before the takeoff (#182). Not at the Rock,
    // whose side is the sea's.
    if let Some(line) = field
        .as_ref()
        .filter(|_| !lxgb)
        .and_then(|a| {
            field_circuit_line(&a.name, &rwy.id, p.left_pattern, published_left, allows_circuits)
        })
    {
        alert(&voice, Priority::Advisory, &line);
    }
    if !lxgb && descent_shape != DescentShape::default() {
        alert(
            &voice,
            Priority::Caution,
            &descent_shape_line(descent_shape, field_elev_ft),
        );
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
    // ...and the retraction schedule, for a go-around the tower orders.
    let flap_schedule = p.takeoff.flap_schedule.clone();
    // Where the airframe declares it, the pattern altitude goes on the
    // altitude dial before the roll and the autopilot's own capture holds it
    // (see `ap_holds_level`). Otherwise the takeoff only lifts a dial the
    // climb would bust through, well clear of the pattern.
    let p_ap_capture = AP_ALTITUDE_CAPTURE.load(Ordering::SeqCst);
    let mut takeoff = p.takeoff;
    if p_ap_capture {
        takeoff.level_ft = Some(tpa_msl_ft);
    }
    // The rectangle walks the flap schedule after the handoff (#150); an
    // authored route does not, so its takeoff cleans up as it always did.
    if authored_circuit(&rwy, &geom, field_elev_ft, tpa_msl_ft, left).is_some() {
        takeoff.flaps_to_caller = false;
    }
    if !autotakeoff::start_scoped(base_url.clone(), voice.clone(), takeoff, xp.cancellation()) {
        stand_down(
            &xp,
            &voice,
            "The previous takeoff is still stopping. Your aircraft.",
            "takeoff_busy",
        );
        return;
    }
    loop {
        tokio::time::sleep(std::time::Duration::from_millis(300)).await;
        if xp.cancelled() {
            stand_down(
                &xp,
                &voice,
                "Circuit cancelled. Your aircraft.",
                "abort_takeoff_phase",
            );
            return;
        }
        if !autotakeoff::running() {
            break;
        }
    }
    let outcome = autotakeoff::outcome();
    if outcome != Some(sim::run::Outcome::Completed) {
        // A reject has already stopped, parked and said "your aircraft".
        let line = match outcome {
            // The recovery has the aircraft and has said why (#181).
            Some(sim::run::Outcome::Failed(r)) if r == autotakeoff::AP_NO_PITCH => {
                "The circuit is off. I have the aircraft until you say my controls."
            }
            Some(sim::run::Outcome::Failed(_)) => {
                "The takeoff was rejected, so the circuit is off."
            }
            _ => "The takeoff handed control back. Your aircraft.",
        };
        stand_down(&xp, &voice, line, TAKEOFF_NOT_COMPLETED);
        return;
    }
    let agl_ft = xp.get_float(AGL_M).await.unwrap_or(0.0) * M_TO_FT;
    if agl_ft < AIRBORNE_MIN_AGL_FT {
        stand_down(
            &xp,
            &voice,
            "The takeoff didn't get us airborne, so the circuit is off.",
            "takeoff_rejected",
        );
        return;
    }
    // What the takeoff left us besides the aircraft: the climb it was
    // commanding and the flaps it left out for this loop's schedule (#150).
    let handover = autotakeoff::take_climb_handover().unwrap_or_default();
    crate::blackbox::record(
        "circuit",
        serde_json::json!({"event": "takeoff_handoff", "agl_ft": agl_ft,
                           "vs_cmd": handover.vs_cmd, "flaps_left": handover.flaps_left}),
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
        // Probed along the LEGS, not at the waypoints: the six points sat on
        // sea either side of the Rock while the downwind between them crossed
        // the ridge 523 ft up (2026-09-14). Terrain under a pattern-altitude
        // leg raises the pattern altitude; terrain under a descending leg,
        // or a raise past MAX_TERRAIN_RAISE_FT, refuses the route.
        let (leg_max, verdict) =
            check_authored_route(&xp, &rwy, &route, tpa_msl_ft, field_elev_ft, pattern_kts).await;
        let authored = match verdict {
            Ok(v) => Some(v),
            Err(why) => {
                crate::blackbox::record(
                    "circuit",
                    serde_json::json!({"event": "authored_route_unsafe", "why": why,
                                       "on": "airborne"}),
                );
                None
            }
        };
        if let Some((route, tpa_used)) = authored {
            crate::blackbox::record(
                "circuit",
                serde_json::json!({
                    "event": "authored_route", "wpts": route.len(),
                    "tpa_msl_ft": tpa_used, "raised_ft": tpa_used - tpa_msl_ft,
                    "leg_max_ft": leg_max,
                }),
            );
            if tpa_used > tpa_msl_ft + 1.0 {
                alert(
                    &voice,
                    Priority::Advisory,
                    &format!(
                        "Pattern altitude {} feet, to clear the Rock.",
                        (tpa_used / 50.0).round() * 50.0
                    ),
                );
            }
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
                climb_full_rate_kts(pattern_kts, p_vy_kts, p_climb_floor_kts),
                p_climb_floor_kts,
                callsign.clone(),
                p.apt_dat.clone(),
                p.cfg.clone(),
                p.approach_ias,
            )
            .await;
            return;
        }
        // ...and the standard pattern is flown only where it clears the
        // ground itself (#182). Refused both ways in the AIR (the runway's
        // judgement passed, the probe answered otherwise after the takeoff):
        // never "your aircraft" at the handoff height, the reversal's hold
        // instead, climbing clear of the ground ahead on the autopilot.
        if let Some(line) = refused.as_deref() {
            let corridor = [l_facts.terrain_max_ft, r_facts.terrain_max_ft]
                .into_iter()
                .chain(leg_max.iter().copied())
                .flatten()
                .reduce(f64::max);
            let mag_var = xp.get_float(MAG_VAR).await.unwrap_or(0.0);
            let hold_kts = crate::autoland::hand_back_speed_kts(
                p_approach_ias,
                handover.flaps_left.unwrap_or(0.0),
                &crate::aircraft::flap_limits(),
            );
            let floor = crate::aircraft::config_floor_kts(handover.flaps_left.unwrap_or(0.0));
            hold_clear_of_terrain(
                &xp,
                &base_url,
                &voice,
                &rock_refused_line(line),
                corridor,
                tpa_msl_ft,
                rwy_course,
                mag_var,
                hold_kts,
                floor,
            )
            .await;
            return;
        }
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
    let _detail = crate::auto_status::DetailGuard::hold(&DETAIL);
    // The dial's CURRENT value seeds the slew (the takeoff left it near the
    // runway heading); every subsequent write moves it the short way only.
    LAST_BUG.store(u64::MAX, std::sync::atomic::Ordering::Relaxed);
    let mut bug_mag = match autoflight::read_dial(&xp, Dial::HeadingBug).await {
        Ok(v) => v,
        Err(_) => true_to_mag(rwy_course, mag_var),
    };
    bug_mag = slew_bug(
        bug_mag,
        true_to_mag(leg_heading(rwy_course, Leg::Upwind, left), mag_var),
        BUG_SLEW_PER_TICK,
    );
    write_bug(&xp, bug_mag).await;
    // The pattern's knob from here on (`BugHold`, issue #93).
    let mut bug_hold = BugHold::default();
    let loop_t0 = std::time::Instant::now();
    // The lateral re-engage (`ap_engage::ServoWatch`): FD first, confirmed.
    let mut servo_watch = crate::ap_engage::ServoWatch::default();
    // The pattern's own give-up (#160).
    let mut pattern_guard = PatternGuard::default();
    // The autopilot's answer to the dial (#181).
    let mut climb_watch = crate::climb_watch::ClimbWatch::default();
    let mut last_verdict = crate::climb_watch::Verdict::Flying;
    // Forward terrain guard cadence + throttle; a fired guard raises the
    // vertical target floor for a while instead of pressing dead buttons.
    let mut tick: u32 = 0;
    let mut guard_warned: Option<std::time::Instant> = None;
    let mut guard_floor_ft: Option<(f64, std::time::Instant)> = None;
    // VS-dial bookkeeping (the takeoff's recipe: engage syncs the dial away,
    // so rewrite after any re-engage; skip no-op writes otherwise).
    let mut last_vs_cmd: Option<f64> = None;
    let mut vs_refused: Option<std::time::Instant> = None;
    let mut vs_refused_said = false;
    let mut lights_done = false;
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
    // The flap handle as this loop last commanded it; `None` until base.
    let mut last_flap: Option<f64> = None;
    // Set when the base turn onto final may BEGIN. The handover waits for the
    // geometry to become flyable, not merely for the leg to change.
    let mut turning_final = false;
    // The tower's go-around orders, watched from the pattern on: a landing
    // clearance only stands on the approach legs, and there the order
    // outranks everything (see `flies_go_around_order`).
    // The pilot's order rides the same watch and is flown the same way
    // (`go_around`, #77).
    let mut orders = crate::go_around::GoAroundWatch::arm();
    // An order seen while handing over, flown on the next tick.
    let mut order_owed: Option<crate::go_around::GoAroundBy> = None;
    // A clean-up in progress, the takeoff's (#150) or a go-around's: the flap
    // it has commanded, until clean.
    let mut cleaning_up: Option<f64> = takeoff_flaps_to_clean(handover.flaps_left, &flap_schedule);
    if cleaning_up.is_some() {
        last_flap = cleaning_up;
    }
    // Whose clean-up it is, for the record.
    let mut cleanup_event = "climb_flap_step";
    // The flaps' own pace for the clean-up (#169), from the detent above the
    // one handed over: the takeoff may have stepped it on the way out.
    let mut flap_pace = autotakeoff::FlapPace::default();
    let handed_from = cleaning_up.and_then(|f| detent_above(f, &crate::aircraft::flap_limits()));
    if let Some(above) = handed_from {
        flap_pace.stepped(above);
    }
    let pace_t0 = std::time::Instant::now();
    // The takeoff's climb, carried into the pattern's law (#150).
    let mut climb_carry = ClimbCarry::new(handover.vs_cmd);
    let mut last_tick_at: Option<std::time::Instant> = None;
    // The gear the go-around still owes raising.
    let mut gear_up_owed = false;
    let mut go_arounds: u32 = 0;
    // The circuit's own go-arounds for a final too high to hand over (#188).
    let mut high_go_arounds: u32 = 0;
    // The steepest the pattern descends at, this airframe (#188).
    let descent_cap = pattern_descent_cap_fpm(crate::procedure::descent_fpm());
    // The downwind held past its base turn for height, and where it began.
    let mut extending: Option<f64> = None;
    // The bank flown through the turn onto base.
    let mut base_bank = BankWatch::default();

    loop {
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        if xp.cancelled() {
            stand_down(
                &xp,
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

        // Position, height and heading as ONE frame saw them (#157).
        let (pos, hpath_r, hdg_st, vs_st) = tokio::join!(
            sim::position::read(&xp),
            xp.get_float(HPATH),
            xp.get_float(HDG_STATUS),
            xp.get_float(VS_STATUS),
        );
        let (lat, lon) = sim::position::lat_lon(&pos);
        let agl_m = sim::position::field(&pos, |p| p.agl_m);
        let elev_m = sim::position::field(&pos, |p| p.elev_m);
        let psi_r = sim::position::field(&pos, |p| p.psi_true_deg);
        let thr_dref = throttle_dref();
        let (ias_r, thr_r, dial_r, alt_hold_r, phi_r, servos_r) = tokio::join!(
            xp.get_float(IAS),
            xp.get_float(&thr_dref),
            autoflight::read_dial(&xp, Dial::Altitude),
            xp.get_float(ALT_HOLD_STATUS),
            xp.get_float(PHI),
            xp.get_float(SERVOS_ON),
        );
        let (Ok(lat), Ok(lon)) = (lat, lon) else {
            continue;
        };
        let agl_ft = agl_m.unwrap_or(0.0) * M_TO_FT;
        let alt_msl_ft = elev_m.map(|m| m * M_TO_FT).unwrap_or(f64::NAN);
        let (hdg_read, vs_read) = (hdg_st.ok(), vs_st.ok());
        let (hdg_on, vs_on) = (hdg_read.unwrap_or(1.0) > 0.5, vs_read.unwrap_or(1.0) > 0.5);
        // Position in the departure-runway frame: along the takeoff direction
        // (negative on the approach side) and cross-track (right positive).
        let a_m = -along_track_nm(lat, lon, &rwy) * M_PER_NM;
        let x_m = cross_track_nm(lat, lon, &rwy) * M_PER_NM;
        // Height over the circuit's own path here (#188): the handoff waits
        // for it, and a final still high a mile out is flown around.
        let above_path = above_path_ft(alt_msl_ft, field_elev_ft, a_m, descent_shape.gp_ft_per_nm);
        // Once only: a second lap as high as the first is the same geometry,
        // and autoland's own late go-around and reversal take it from there
        // (blackbox `high_handoff`).
        let high_final = leg == Leg::Final
            && turning_final
            && high_go_arounds == 0
            && final_high_give_up(above_path, a_m);
        if high_final {
            high_go_arounds += 1;
        }

        // --- The tower's go-around order, before anything else: a landing
        // clearance voided while the circuit flies its own final is flown
        // HERE, not left to autoland (which arms at 2 nm and only watches
        // from then on). LEMG 31 (sortie 1790584715-full_auto, 2026-09-28):
        // the order came at 736 ft, 5.7 km out, and the circuit flew on,
        // handed over at 2 nm and landed "without clearance".
        let fresh = orders.take();
        let crew = high_final.then_some(crate::go_around::GoAroundBy::Crew);
        if let Some(by) = order_owed.take().or(fresh).or(crew) {
            let ias_now = ias_r.as_ref().ok().copied();
            // The crew's own go-around answers no order.
            if by != crate::go_around::GoAroundBy::Crew {
                crate::go_around::answer(if flies_go_around_order(leg) {
                    crate::go_around::Answer::Flying
                } else {
                    crate::go_around::Answer::NotOnApproach
                });
            }
            if flies_go_around_order(leg) {
                go_arounds += 1;
                let limits = crate::aircraft::flap_limits();
                let flap = go_around_flap(last_flap, &limits);
                let (thr_ok, flap_ok) = tokio::join!(xp.set_float(&thr_dref, 1.0), async {
                    if limits.is_empty() || last_flap.is_none_or(|f| f <= flap + 1e-6) {
                        None
                    } else {
                        Some(xp.set_float(FLAP_RATIO, flap).await.is_ok())
                    }
                },);
                crate::blackbox::record(
                    "circuit",
                    serde_json::json!({
                        "event": "ordered_go_around", "leg": format!("{leg:?}"),
                        "agl_ft": agl_ft, "a_m": a_m, "x_m": x_m, "ias": ias_now,
                        "flap_was": last_flap, "flap": flap, "flap_ok": flap_ok,
                        "thr_ok": thr_ok.is_ok(), "n": go_arounds, "by": by.label(),
                        "above_path_ft": above_path,
                    }),
                );
                if let Some(was) = last_flap {
                    flap_pace.stepped(was);
                }
                if !limits.is_empty() {
                    last_flap = Some(flap);
                }
                cleaning_up = (!limits.is_empty() && flap > 1e-6).then_some(flap);
                cleanup_event = "go_around_flap_step";
                gear_up_owed = retractable_gear && gear_lowered;
                leg = Leg::Upwind;
                turning_final = false;
                descent_target = None;
                last_vs_cmd = None;
                alert(&voice, Priority::Warning, go_around_line(by));
                // The tower voided the clearance, or the pilot did: it is owed
                // the call, and the downwind report comes round with the
                // pattern.
                transmit(radio_call(&callsign, "going around"));
            } else {
                crate::blackbox::record(
                    "circuit",
                    serde_json::json!({"event": "go_around_order_ignored",
                                       "leg": format!("{leg:?}"), "agl_ft": agl_ft,
                                       "by": by.label()}),
                );
            }
        }

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
            .map(|hp| crate::angles::signed_diff_deg(*hp, rwy_course))
            .unwrap_or(180.0);
        // Established by autoland's own measure: its pursuit's residual at
        // the threshold inside the runway's landing limit. The IAS stands in
        // for the groundspeed (a failed read: the approach speed).
        let handoff_gs = ias_r
            .as_ref()
            .ok()
            .copied()
            .filter(|v| v.is_finite() && *v > 40.0)
            .unwrap_or(p_approach_ias);
        let rwy_width_m = crate::autoland::runway_width_m(&rwy);
        let landing_limit_m = crate::autoland::landing_xtk_limit_m(rwy_width_m);
        // On the path too (#188), until the one lap flown around for height
        // comes back as high: then autoland's own go-around takes it.
        let high_handoff = high_go_arounds > 0 && final_high_give_up(above_path, a_m);
        if turning_final
            && ready_for_autoland(
                x_m,
                a_m.abs(),
                hdg_err,
                handoff_gs,
                rwy_width_m.map(|_| landing_limit_m),
                if high_handoff { None } else { above_path },
            )
        {
            let alt_msl = field_elev_ft + agl_ft;
            let arr = ArrivalCtx {
                xp: &xp,
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
                // The procedure flies the arrival, its parent our token.
                handed_off(&xp);
                return;
            }
            // The arrival lookup takes a while off the reactor: an order
            // given meanwhile is still this leg's, never autoland's to miss.
            if let Some(by) = orders.take() {
                order_owed = Some(by);
                continue;
            }
            // A stand-down (swap, instructor, a new flight) that landed during
            // that await set our ABORT and autoland's: clearing autoland's
            // latch below would launch an autoland nobody can cancel.
            if xp.cancelled() {
                stand_down(
                    &xp,
                    &voice,
                    "Circuit cancelled. Your aircraft.",
                    "abort_pattern_phase",
                );
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
                    "trk_err": hdg_err,
                    "residual_m": handoff_residual_m(x_m, a_m, handoff_gs),
                    "limit_m": landing_limit_m,
                    "glidepath_deg": descent_shape.glidepath_deg(),
                    "above_path_ft": above_path,
                    "high_handoff": high_handoff,
                }),
            );
            // A path steepened over the ground is autoland's path too (#182),
            // else it would take the aircraft handed over and dive for 3 degrees.
            // A circuit final is judged at the visual gate, 500 ft (#199).
            let hand_down = crate::autoland::HandDown::circuit(
                descent_shape.steep().then(|| descent_shape.glidepath_deg()),
            );
            // The circuit's token is its parent: a stand-down that lands
            // after the recheck above still cancels this autoland.
            crate::autoland::run_autoland_scoped(
                base_url.clone(),
                p.apt_dat.clone(),
                p.cfg.clone(),
                p.approach_ias,
                voice.clone(),
                None,
                Some(crate::autoland::PlannedRunway::anywhere(&rwy_id)),
                xp.cancellation(),
                hand_down,
            );
            handed_off(&xp);
            return;
        }

        // Fly each leg as a TRACK, not a bare heading: crab into the wind so the
        // AP holds the ground track. Without this a 12 kt crosswind blew the
        // downwind 900 m back into the field (the teardrop, 2026-07-23).
        // crab = heading - track; command heading = leg heading + crab.
        let (psi_v, hp_v) = (psi_r.ok(), hpath_r.ok());
        let crab = match (psi_v, hp_v) {
            (Some(psi), Some(hp)) => {
                crate::angles::signed_diff_deg(psi, hp).clamp(-25.0, 25.0)
            }
            _ => 0.0,
        };
        // Every leg but the last flies its own heading. FINAL flies an
        // intercept: it is the leg whose whole job is to be ON the centreline
        // by the handoff, and commanding the bare runway course makes it a leg
        // that flies parallel to the line it is supposed to join.
        let leg_hdg = match leg {
            Leg::Final => final_intercept_hdg(
                rwy_course,
                final_led_xtk_m(x_m, hp_v, rwy_course, handoff_gs, final_lead_s()),
            ),
            _ => leg_heading(rwy_course, leg, left),
        };
        // The bank the autopilot really flies, from the turn onto base, sizes
        // the turn onto final (see `final_capture_on_bank`).
        if leg == Leg::Base {
            if let (Some(psi), Ok(phi)) = (psi_v, phi_r.as_ref()) {
                base_bank.note(*phi, psi - leg_hdg);
            }
            if let (Some(bank), Ok(ias)) = (base_bank.steady_bank_deg(), ias_r.as_ref()) {
                if let Some(c) = final_capture_on_bank(*ias, bank) {
                    geom.final_capture_m = c;
                }
            }
        }
        let want_hdg = leg_hdg + crab;
        let target_mag = true_to_mag(want_hdg, mag_var);
        // Through the one door every pursuit steer takes: the bug leads the
        // nose gently until the autopilot is banked the right way. A bare
        // slew put it 90 degrees off from wings level in 1.5 s, and the SF50
        // flew 13 of its 30 circuit turns the long way round (ETND 26,
        // 2026-09-23: crosswind, downwind and base each a 270 right).
        bug_mag = match (psi_v, phi_r.as_ref().ok()) {
            (Some(psi), Some(phi)) => bug_toward(
                bug_mag,
                target_mag,
                true_to_mag(psi, mag_var),
                *phi,
                BUG_SLEW_PER_TICK,
            ),
            _ => slew_bug(bug_mag, target_mag, BUG_SLEW_PER_TICK),
        };
        // Set, not rewritten: per leg, a correction past the leg's deadband,
        // the final's finer (`BugHold`, issue #93).
        hold_bug(
            &xp,
            &mut bug_hold,
            leg as u32,
            &mut bug_mag,
            loop_t0.elapsed().as_secs_f64(),
            if leg == Leg::Final {
                CIRCUIT_FINAL_DEADBAND
            } else {
                CIRCUIT_LEG_DEADBAND
            },
        )
        .await;
        // The lateral mode must actually be ON for the bug to mean anything —
        // re-engage like the takeoff's VS guard, never press blind: the
        // servos through the one engage (`ap_engage::ServoWatch`, FD first,
        // confirmed), the mode pressed while it reads off.
        reengage_lateral(
            &xp,
            &mut servo_watch,
            &p.cfg.fd_commands,
            loop_t0.elapsed().as_secs_f64(),
            !hdg_on,
            servos_r.as_ref().ok().copied(),
            ias_r.as_ref().ok().copied().unwrap_or(f64::NAN),
            "rectangle",
        )
        .await;
        autoflight::bring_heading(&xp, hdg_read, "circuit_rectangle").await;

        // Forward terrain floor, every ~2 s: the ground under us and along
        // the ground track ahead (the follower's own probe geometry,
        // `procedure::look_ahead_points`). Inside LIVE_FLOOR_FT of it on the
        // upwind, crosswind, downwind or base, the circuit stops descending
        // and climbs to its floor (`live_floor_ft`), said once a spell,
        // blackbox `terrain_floor` (#182: LEMG 13's base at 305 ft AGL).
        // Silently inactive without the plugin probe (the old behavior).
        tick = tick.wrapping_add(1);
        if tick.is_multiple_of(4) && alt_msl_ft.is_finite() && leg != Leg::Final {
            if let Some(hp) = hp_v {
                let mut pts = vec![(lat, lon)];
                pts.extend(crate::procedure::look_ahead_points(
                    lat,
                    lon,
                    hp,
                    LIVE_LOOK_NM,
                    LIVE_LOOK_POINTS,
                ));
                if let Some(e) = crate::terrain::max_elevation_ft(&xp, &pts).await {
                    if let Some(floor) = live_floor_ft(leg, alt_msl_ft, e) {
                        guard_floor_ft = Some((floor, std::time::Instant::now()));
                        if guard_warned.is_none() {
                            guard_warned = Some(std::time::Instant::now());
                            crate::blackbox::record(
                                "circuit",
                                serde_json::json!({"event": "terrain_floor", "leg": format!("{leg:?}"),
                                                   "ground_ft": e, "alt_msl": alt_msl_ft,
                                                   "agl_ft": agl_ft, "floor_ft": floor}),
                            );
                            alert(
                                &voice,
                                Priority::Warning,
                                &format!(
                                    "Terrain ahead. Holding {:.0} feet.",
                                    (floor / 50.0).ceil() * 50.0
                                ),
                            );
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
        // A spell ends with its floor: the next one is said again.
        if guard_floor_ft.is_none() {
            guard_warned = None;
        }
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
        let leg_target = leg_target_shaped(
            leg,
            a_m,
            field_elev_ft,
            tpa_msl_ft,
            geom.downwind_past_m,
            descent_shape,
            &mut descent_target,
        );
        // The floor never applies on final: descending toward the runway is
        // the POINT there, and autoland brings its own terrain floor. A
        // ridge-probe floor that lingered into base handed autoland the
        // approach 190 ft high and it balked over the numbers (2026-07-24),
        // so on base it is only the ground's own LIVE_FLOOR_FT (`live_floor_ft`)
        // and only while that ground is there: a balk is better than the
        // hill LEMG 13's base passed 305 ft over (#182).
        let target_alt = if leg == Leg::Final {
            leg_target
        } else {
            guard_floor_ft.map_or(leg_target, |(f, _)| leg_target.max(f))
        };
        // Still a pattern (#160): not far above it, no leg without end, an
        // autopilot in. Else the aircraft goes back, flyable and said loudly.
        let pattern_ft = tpa_msl_ft.max(target_alt);
        let servos = servos_r
            .as_ref()
            .ok()
            .filter(|v| v.is_finite())
            .map(|v| *v > 0.5);
        if let Some(trip) = pattern_guard.step(
            loop_t0.elapsed().as_secs_f64(),
            leg,
            alt_msl_ft,
            pattern_ft,
            leg_limit_s(leg, &geom, rwy.length_m, pattern_kts),
            servos,
        ) {
            let hold_kts = crate::autoland::hand_back_speed_kts(
                p_approach_ias,
                last_flap.unwrap_or(0.0),
                &crate::aircraft::flap_limits(),
            );
            let floor = crate::aircraft::config_floor_kts(last_flap.unwrap_or(0.0));
            give_up_pattern(
                &xp, &base_url, &voice, trip, leg, alt_msl_ft, pattern_ft, mag_var, hold_kts,
                floor,
            )
            .await;
            return;
        }
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
            path_descent_fpm_on(closure_kts, descent_shape.gp_ft_per_nm)
        } else {
            0.0
        };
        // The descent sized on the height to lose over the track left, within
        // the airframe's (#188): a flat 500 fpm from a pattern raised for the
        // ground handed autoland the final 545 ft high. A terrain floor above
        // the leg's target is still the target: never below it.
        let descent_fpm = leg_descent_authority_fpm(
            leg,
            a_m,
            x_m,
            alt_msl_ft,
            field_elev_ft,
            tpa_msl_ft,
            &geom,
            left,
            descent_shape,
            handoff_gs,
            descent_cap,
            target_rate,
        );
        let vs_want = if alt_msl_ft.is_finite() {
            alt_capture_vs_on(target_alt, target_rate, alt_msl_ft, vs_est, descent_fpm)
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
        // The ONE speed this leg holds: the throttle below holds it and the
        // limiter grants full rate at it (#169).
        let want_kts = leg_want_kts(
            leg,
            LegSpeeds {
                pattern_kts,
                approach_kts,
                approach_ias: p_approach_ias,
            },
            last_flap,
            cleaning_up,
            &flap_schedule,
            &crate::aircraft::flap_limits(),
            crate::aircraft::config_floor_kts,
        );
        // ...and the floor it fades to, the kept flap's while cleaning up.
        let climb_floor = leg_climb_floor_kts(
            leg,
            p_climb_floor_kts,
            p_vs_clean_kts,
            cleaning_up,
            crate::aircraft::config_floor_kts,
        );
        let full_kts = climb_full_rate_kts(want_kts, p_vy_kts, climb_floor);
        let law_vs = climb_limit_fpm(vs_want, ias_read, ias_trend, full_kts, climb_floor);
        // ...walked to from the climb the takeoff handed over (#150).
        let tick_dt = last_tick_at.map_or(0.5, |t| t.elapsed().as_secs_f64());
        last_tick_at = Some(std::time::Instant::now());
        let mut vs_cmd = climb_carry.step(
            law_vs,
            tick_dt,
            matches!(leg, Leg::Base | Leg::Final),
            ias_read.is_finite() && ias_read <= climb_floor,
        );
        // Landing and taxi lights off through 1,000 ft. The authored route's
        // comment said "as the rectangle does" and the rectangle never did:
        // the A330's climb list named "taxi light, off" (EGPF, 2026-09-21).
        if !lights_done && alt_msl_ft.is_finite() && alt_msl_ft - field_elev_ft > LIGHTS_OFF_AGL_FT
        {
            lights_done = true;
            let did = crate::tools::lights::after_climb(&xp).await;
            crate::blackbox::record(
                "circuit",
                serde_json::json!({"event": "lights_climb", "off": did}),
            );
        }

        // The autopilot's own capture flies the level legs where the airframe
        // declares it. Failed reads are no measurement: no dial write, no
        // hold believed, and the VS law below carries on as it always has.
        let mut ap_level = false;
        if p_ap_capture && alt_msl_ft.is_finite() {
            if let (Ok(dial), Ok(hold)) = (&dial_r, &alt_hold_r) {
                match circuit_dial_ft(target_rate, target_alt, *dial, alt_msl_ft) {
                    Some(want) => {
                        let ok = autoflight::set_dial(&xp, Dial::Altitude, want).await.is_ok();
                        crate::blackbox::record(
                            "circuit",
                            serde_json::json!({"event": "alt_dial", "was": dial, "now": want,
                                               "leg": format!("{leg:?}"), "ok": ok}),
                        );
                    }
                    None => ap_level = ap_holds_level(target_rate, target_alt, *dial, *hold),
                }
            }
        }
        // The autopilot's answer, WATCHED (#181): a climb far over the dial
        // with the speed going, or one still on under the floor, is sent
        // down, and taken if it does not come down. The floor is the kept
        // flap's while cleaning up (`climb_floor`), the landing flap's on
        // base and final.
        let watch_floor = match leg {
            Leg::Base | Leg::Final => {
                crate::aircraft::config_floor_kts(last_flap.unwrap_or(0.0)).or(Some(climb_floor))
            }
            _ => Some(climb_floor),
        };
        let verdict = climb_watch.step(
            loop_t0.elapsed().as_secs_f64(),
            alt_msl_ft,
            ias_read,
            watch_floor,
            (!ap_level).then_some(vs_cmd),
        );
        if verdict != last_verdict {
            last_verdict = verdict;
            crate::blackbox::record(
                "circuit",
                serde_json::json!({"event": "climb_watch", "verdict": format!("{verdict:?}"),
                                   "leg": format!("{leg:?}"), "ias": ias_read,
                                   "vs": climb_watch.vs_fpm(), "cmd_vs": vs_cmd,
                                   "floor": watch_floor, "agl_ft": agl_ft}),
            );
        }
        if let crate::climb_watch::Verdict::TakeOver(why) = verdict {
            let hold_kts = crate::autoland::hand_back_speed_kts(
                p_approach_ias,
                last_flap.unwrap_or(0.0),
                &crate::aircraft::flap_limits(),
            );
            let trip = PatternTrip::ClimbNotHeld {
                why,
                ias: ias_read,
                vs: climb_watch.vs_fpm().unwrap_or(f64::NAN),
            };
            give_up_pattern(
                &xp, &base_url, &voice, trip, leg, alt_msl_ft, pattern_ft, mag_var, hold_kts,
                watch_floor,
            )
            .await;
            return;
        }
        vs_cmd = crate::climb_watch::dial_for(verdict, vs_cmd);
        // The autopilot's pitch AUTHORITY after an engage (#181), the
        // takeoff's carried here: the dial this loop flies, 0 while the
        // autopilot's capture holds the level.
        let auth_dial = if ap_level { 0.0 } else { vs_cmd };
        if let Some(crate::climb_watch::Authority::NoAuthority(_)) =
            crate::climb_watch::feed_authority(climb_watch.vs_fpm(), Some(auth_dial))
        {
            let hold_kts = crate::autoland::hand_back_speed_kts(
                p_approach_ias,
                last_flap.unwrap_or(0.0),
                &crate::aircraft::flap_limits(),
            );
            let trip = PatternTrip::ClimbNotHeld {
                why: crate::climb_watch::Why::NoAuthority,
                ias: ias_read,
                vs: climb_watch.vs_fpm().unwrap_or(f64::NAN),
            };
            give_up_pattern(
                &xp, &base_url, &voice, trip, leg, alt_msl_ft, pattern_ft, mag_var, hold_kts,
                watch_floor,
            )
            .await;
            return;
        }
        if ap_level {
            // Whatever the VS dial says now is stale the moment VS re-engages.
            last_vs_cmd = None;
            vs_refused = None;
        } else {
            if autoflight::bring_vs(&xp, vs_read, "circuit_rectangle")
                .await
                .pressed(Button::VerticalSpeed)
            {
                // The engage synced the dial: a rewrite is forced.
                last_vs_cmd = None;
                // A vertical mode engaged: its authority is checked afresh.
                if let Some(vs) = climb_watch.vs_fpm() {
                    crate::climb_watch::arm_authority("circuit", vs, vs_cmd);
                }
            }
            if last_vs_cmd.is_none_or(|v| (v - vs_cmd).abs() > 40.0) {
                let _ = autoflight::set_dial(&xp, Dial::Vs, vs_cmd).await;
                last_vs_cmd = Some(vs_cmd);
            }
            // The one thing the capture rests on: VS taking the aircraft back
            // OUT of the hold. An autopilot that will not is said, once, while
            // there is still a downwind's worth of height to act on.
            if p_ap_capture && !vs_on {
                let since = *vs_refused.get_or_insert_with(std::time::Instant::now);
                if vs_stuck(since.elapsed().as_secs_f64()) && !vs_refused_said {
                    vs_refused_said = true;
                    crate::blackbox::record(
                        "circuit",
                        serde_json::json!({"event": "vs_refused", "leg": format!("{leg:?}"),
                                           "cmd_vs": vs_cmd}),
                    );
                    alert(
                        &voice,
                        Priority::Warning,
                        "The autopilot won't take vertical speed. Check the vertical mode.",
                    );
                }
            } else {
                vs_refused = None;
            }
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
            // Configure for landing: base toward the approach flap, final to
            // full, each notch only once the speed is under its placard
            // (`flap_within_limits`, autoland's own law). Airframes with no
            // flap profile are left alone.
            if matches!(leg, Leg::Base | Leg::Final) {
                // A go-around's clean-up not finished by base: base owns the
                // flaps from here.
                cleaning_up = None;
                let limits = crate::aircraft::flap_limits();
                if !limits.is_empty() {
                    let wanted = if matches!(leg, Leg::Final) {
                        1.0
                    } else {
                        BASE_FLAP_RATIO
                    };
                    let flap = crate::autoland::flap_within_limits(wanted, ias, &limits);
                    if last_flap != Some(flap) {
                        last_flap = Some(flap);
                        let _ = xp.set_float(FLAP_RATIO, flap).await;
                        crate::blackbox::record(
                            "circuit",
                            serde_json::json!({"event": "flaps", "leg": format!("{leg:?}"), "ratio": flap, "ias": ias}),
                        );
                    }
                }
            }
            // A go-around cleans up on the airframe's retraction schedule,
            // one detent at a time, paced on the flaps arriving.
            if let Some(cmd) = cleaning_up {
                let clean_ok = crate::aircraft::config_floor_kts(0.0).unwrap_or(pattern_kts * 0.9);
                // The step's speed inside the placard of what is out (#169).
                let flap_schedule =
                    cleanup_schedule(&flap_schedule, &crate::aircraft::flap_limits(), cmd);
                let due = if flap_schedule.is_empty() {
                    go_around_flap_step(&flap_schedule, cmd, None, ias, clean_ok)
                } else {
                    autotakeoff::flap_retract_step(&flap_schedule, cmd, ias)
                };
                if due.is_some() {
                    // Read only while a step is due; a failed read is no
                    // measurement, and no step is taken on it.
                    let deployed = xp.get_float(crate::takeoff_config::FLAP_DEPLOY).await.ok();
                    let t_s = pace_t0.elapsed().as_secs_f64();
                    let judged = flap_pace.judged(t_s, cmd, deployed);
                    let step = go_around_flap_step(&flap_schedule, cmd, judged, ias, clean_ok);
                    if step.is_none() && flap_pace.withheld_log_due(t_s) {
                        crate::blackbox::record(
                            "circuit",
                            serde_json::json!({"event": "flap_step_withheld", "at": cmd,
                                               "ias": ias, "agl_ft": agl_ft,
                                               "deployed": deployed}),
                        );
                    }
                    if let Some(next) = step {
                        flap_pace.stepped(cmd);
                        let ok = xp.set_float(FLAP_RATIO, next).await.is_ok();
                        last_flap = Some(next);
                        cleaning_up = (next > 1e-6).then_some(next);
                        let detent = crate::aircraft::live_profile().and_then(|lp| {
                            crate::aircraft::flap_detent_spoken(lp.flaps.as_ref(), next)
                        });
                        crate::blackbox::record(
                            "circuit",
                            serde_json::json!({"event": cleanup_event, "to": next,
                                               "ias": ias, "agl_ft": agl_ft, "deployed": deployed,
                                               "ok": ok}),
                        );
                        alert(
                            &voice,
                            Priority::Advisory,
                            &autotakeoff::flap_step_line(next, detent.as_deref()),
                        );
                    }
                }
            }
            // The go-around's gear, once climbing and clear of its own
            // warning (the takeoff's rule). The downwind lowers it again.
            if gear_up_owed
                && vs_est > 0.0
                && autotakeoff::gear_up_due(
                    agl_ft,
                    ias,
                    last_flap.unwrap_or(0.0),
                    autotakeoff::gear_warning(),
                    crate::aircraft::gear_limit_kts(),
                )
            {
                gear_up_owed = false;
                let ok = xp.set_float(GEAR_HANDLE, 0.0).await.is_ok();
                if ok {
                    gear_lowered = false;
                }
                crate::blackbox::record(
                    "circuit",
                    serde_json::json!({"event": "go_around_gear_up", "ias": ias,
                                       "agl_ft": agl_ft, "ok": ok}),
                );
                alert(&voice, Priority::Advisory, "Gear up.");
            }
            // The leg's speed, the same number the climb limiter was fed.
            let delta = crate::tools::speedhold::throttle_adjust_live(want_kts - ias, ias_trend);
            if delta != 0.0 {
                let _ = xp
                    .set_float(&throttle_dref(), (thr + delta).clamp(0.0, 1.0))
                    .await;
            }
        }

        DETAIL.set(circuit_detail(
            leg, &rwy_id, alt_msl_ft, target_alt, ias_now,
        ));
        crate::auto_status::set_ap_targets(Some(bug_mag), Some(target_alt), Some(vs_cmd));
        crate::blackbox::record(
            "circuit",
            serde_json::json!({
                "event": "leg", "leg": format!("{leg:?}"), "agl_ft": agl_ft,
                "a_m": a_m, "x_m": x_m, "crab": crab,
                "bug": bug_mag, "tgt": target_mag, "psi": psi_v,
                "hdg_on": hdg_on, "vs_on": vs_on, "ap_level": ap_level, "cmd_vs": vs_cmd,
                "law_vs": law_vs, "carry": climb_carry.carrying(), "flap": last_flap,
                "alt_msl": alt_msl_ft, "target_alt": target_alt,
                "ias": ias_now, "pat_kts": pattern_kts, "want_kts": want_kts,
                "climb_floor": climb_floor,
                // The climb the watch judges on (#181).
                "vs": climb_watch.vs_fpm(),
                // The descent authority and the height over the path (#188).
                "descent_fpm": descent_fpm, "above_path": above_path,
            }),
        );

        if !pattern_advance(leg, a_m, x_m, &geom, rwy.length_m, left) {
            continue;
        }
        // Too high for the rest of the pattern to meet the path: the
        // downwind carries on (#188), said once, bounded by
        // MAX_DOWNWIND_EXTEND_M.
        if leg == Leg::Downwind
            && extend_downwind(
                alt_msl_ft,
                field_elev_ft,
                tpa_msl_ft,
                a_m,
                x_m,
                &geom,
                left,
                descent_shape,
                handoff_gs,
                descent_cap,
            )
        {
            if extending.is_none() {
                extending = Some(a_m);
                crate::blackbox::record(
                    "circuit",
                    serde_json::json!({"event": "downwind_extended", "a_m": a_m, "x_m": x_m,
                                       "alt_msl": alt_msl_ft, "above_path": above_path,
                                       "descent_cap_fpm": descent_cap,
                                       "max_extend_m": MAX_DOWNWIND_EXTEND_M}),
                );
                alert(
                    &voice,
                    Priority::Advisory,
                    "High for the approach. Extending the downwind.",
                );
            }
            continue;
        }
        if leg == Leg::Downwind {
            if let Some(from) = extending.take() {
                crate::blackbox::record(
                    "circuit",
                    serde_json::json!({"event": "downwind_extension_end",
                                       "extended_m": (a_m.abs() - from.abs()).max(0.0),
                                       "alt_msl": alt_msl_ft, "above_path": above_path}),
                );
            }
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
                crate::blackbox::record(
                    "circuit",
                    serde_json::json!({
                        "event": "final_turn", "capture_m": geom.final_capture_m,
                        "bank_measured": base_bank.steady_bank_deg(),
                        "x_m": x_m, "a_m": a_m,
                    }),
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
    xp: &'a XPlaneClient,
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
            rwy_id: Some(crate::autoland::PlannedRunway::at(&icao, &rwy_id)),
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
    crate::procedure::start_scoped(
        base_url.to_string(),
        voice.clone(),
        plan,
        arr.xp.cancellation(),
    )
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod final_leg_kts_tests;

/// Issue #147: the LEMG 13 circuit's final and handoff, flown from the
/// recorded base leg of sortie 1790591256-full_auto (2026-09-28) against
/// the real apt.dat geometry and a 737 fitted to the recorded turns.
#[cfg(test)]
mod issue_147_circuit_tests;

/// Issue #189: the A330's LEMG 30 circuits of 2026-09-29 (sorties
/// 1790710769 and 1790698132-full_auto) ran 66 to 67 m through the
/// centreline off the final turn and were handed over coming back, 41 to
/// 43 m off. Flown from the recorded final-turn point against the real
/// 12/30 row with the A330's fitted heading mode and the knob as the loop
/// sets it.
#[cfg(test)]
mod issue_189_tests;

#[cfg(test)]
mod issue_150_climb_out_tests;

/// The SF50's LEMG 13 circuit (sortie 1790607325, 2026-09-28, issue #160):
/// the recorded ticks through the circuit's give-up, and this sortie's
/// handoff flown through a fitted autopilot plant.
#[cfg(test)]
mod pattern_guard_tests;

/// Issue #169: the A330's LEMG 13 circuit (sortie 1790621184-full_auto,
/// 2026-09-28) flew 224 kt on CONF 1 (placard 215): the throttle held the
/// clean-up's 212.5 while the climb limiter granted full rate only at 220.
#[cfg(test)]
mod issue_169_tests;

/// #172, LEMG 13 (2026-09-28, sortie 1790627884-full_auto): the circuit
/// clearance's readback, queued at +13.7 and again at +34.0, went out at
/// +36.1 (the tower's silence: heard) and again at +63.2 in the takeoff roll.
#[cfg(test)]
mod readback_tests;

/// #182 (review of 528b7d1, 2026-09-29): at LXGB the Rock route and the
/// rectangle refused in the AIR stood the circuit down with "Your aircraft"
/// at the takeoff's handoff height. The numbers are the Rock's: 1,398 ft of
/// granite, the 1,098 ft ridge under 09's downwind entered at 1,130 ft in
/// the climb (2026-09-15), the 1,650 ft pattern.
#[cfg(test)]
mod terrain_hold_tests;

/// #182 layer 1: the field's own circuit data. LEMG's apt.dat (2026-09-29)
/// says `1302 allows_circuits 0` and `1101 13 left`, `1101 31 left`.
#[cfg(test)]
mod field_circuit_tests;

/// Issue #188: the circuit handed autoland the final far above the path
/// after a pattern raised for the ground. LEMG 13, SR22, 2026-09-29.
#[cfg(test)]
mod issue_188_on_the_path_tests;
