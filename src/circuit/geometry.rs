//! The pattern's geometry and laws: turn radii, the final intercept and
//! handoff, the altitude capture, the path and its descent, the live floor,
//! the leg speeds, the go-around flap and the climb floors.

use super::*;

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
pub(super) const AP_BANK_DEG: f64 = 14.0;
/// Max heading-bug movement per 500 ms tick, degrees — see [`slew_bug`].
pub(super) const BUG_SLEW_PER_TICK: f64 = 40.0;

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
pub(super) const AP_BANK_FINAL_DEG: f64 = 15.5;

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
pub(super) const BANK_SAMPLE_ERR_DEG: f64 = 20.0;
/// ...of which it needs this many before it says anything.
pub(super) const BANK_MIN_SAMPLES: usize = 4;
/// The measured bank is rounded shallow by this, as `AP_BANK_FINAL_DEG` was
/// (15.8 measured, 15.5 used): the turn should still err EARLY.
pub(super) const MEASURED_BANK_SHALLOW: f64 = 0.95;

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
    pub(super) samples: Vec<f64>,
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
pub(super) const MAX_INTERCEPT_CUT_DEG: f64 = 35.0;
pub(super) const INTERCEPT_MARGIN: f64 = 1.3;

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
pub(super) const ROLLED_OUT_HDG_DEG: f64 = 12.0;
/// Distance still to run below which we hand over regardless of alignment: a
/// turn that has gone wrong is still better flown by autoland's intercept than
/// by a pattern leg that has run out of room.
pub(super) const LAST_DITCH_ALONG_M: f64 = 1500.0;
/// ...and how far off the centreline the valve will still accept. Twice the
/// established tolerance: enough that a turn which merely finished untidily
/// still gets handed over rather than flown into the ground by a pattern leg,
/// and tight enough that the 776 m diagonal never qualifies.
pub(super) const LAST_DITCH_XTK_M: f64 = 2.0 * ESTABLISHED_XTK_M;

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
pub(super) const ON_LINE_LIMITS: f64 = 3.0;
/// ...and tracking within this of the course: rolled out, not still turning
/// off the intercept (a handoff mid-roll-out teaches the final's crab filter
/// and track trim the turn).
pub(super) const ON_LINE_TRACK_DEG: f64 = 5.0;

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
pub(super) const ESTABLISHED_XTK_M: f64 = 300.0;

/// The steepest cut the final leg will take toward the centreline. The
/// standard intercept angle for an instrument approach, and comfortably inside
/// autoland's own 35-degree pursuit limit, so a leg flown at the cap still
/// hands over something interceptable.
pub(super) const MAX_FINAL_CUT_DEG: f64 = 30.0;

/// How far ahead the final leg aims when closing the centreline.
///
/// Sets how long the cut stays at the cap: the full 30 degrees holds until the
/// offset is inside `FINAL_AIM_M * tan(30)`, about 520 m, and fades from
/// there. A gentler aim reads better on paper and does not work — at 2,000 m
/// the cut had faded to 8 degrees by the time the offset was down to 300, and
/// the simulated intercept from the live roll-out only just made the
/// centreline before it ran out of runway. Hold the intercept like an
/// instrument approach does, then wash it out.
pub(super) const FINAL_AIM_M: f64 = 900.0;

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
pub(super) const MAX_FINAL_LEAD_S: f64 = 8.0;

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
pub(super) const HANDOFF_AGL_FT: f64 = 700.0;
/// How far above the approach speed the base and final legs are flown. Enough
/// margin to stay comfortably clean-configured, close enough that autoland has
/// only the last few knots to take off with flaps.
pub(super) const APPROACH_LEG_MARGIN_KTS: f64 = 20.0;
/// Altitude-capture gain: commanded fpm per foot of error.
pub(super) const ALT_GAIN_FPM_PER_FT: f64 = 3.0;
/// ...and how far AHEAD the capture looks, seconds. A vertical speed does not
/// stop when the command does: an airliner rolling out of a 700 fpm climb
/// carries on for the best part of ten seconds. Pure proportional control on
/// altitude therefore commanded a climb right up to the target, sailed 100 ft
/// through it, commanded a descent, sailed back — the MD-82 hunted ±60 ft
/// around pattern altitude for the whole downwind with a period near forty
/// seconds (Luton, 2026-07-28). Feeding the CURRENT vertical speed forward
/// levels the aircraft where it will actually arrive.
pub(super) const ALT_LEAD_S: f64 = 8.0;

/// How much descent the capture may command BEYOND the target's own rate.
/// For a target standing still this is the whole authority and reproduces the
/// law's original 500 fpm floor exactly; on a glidepath it is what closes an
/// error on top of simply keeping up.
pub(super) const ALT_MAX_EXTRA_DESCENT_FPM: f64 = 500.0;

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
pub(super) const DOWNWIND_DESCENT_AGL_FT: f64 = 800.0;

/// The live floor over the ground under and ahead of the aircraft (#182): the
/// same 500 ft the descending legs are planned to keep ([`DESCENT_FLOOR_FT`]).
pub(super) const LIVE_FLOOR_FT: f64 = 500.0;
/// The upwind's trip, the old forward guard's figure.
pub(super) const UPWIND_FLOOR_FT: f64 = 400.0;
/// ...and how far above the ground a level leg climbs once it trips, so a
/// floor that trips does not sit on its own trigger (the old guard's 800).
pub(super) const LIVE_FLOOR_CLIMB_FT: f64 = 800.0;
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
pub(super) const DESCENT_PLAN_SHARE: f64 = 0.75;
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
pub(super) const GO_AROUND_PLACARD_MARGIN_KTS: f64 = 10.0;
/// ...and reaches this far past the next retraction speed, so the step is
/// taken rather than hovered at.
pub(super) const GO_AROUND_RETRACT_MARGIN_KTS: f64 = 5.0;
/// Where a retraction speed sits inside the placard margin of the flap being
/// left, the step is taken this far under the placard less its margin.
pub(super) const CLEANUP_STEP_UNDER_CAP_KTS: f64 = 3.0;

/// The placard of the flap `flap` out ([`crate::aircraft::flap_placard_kts`],
/// the one lookup). Infinite when none is. Pure.
pub(super) fn placard_of(flap: f64, limits: &[(f64, Option<f64>)]) -> f64 {
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
    pub(super) cmd: Option<f64>,
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
