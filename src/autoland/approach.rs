//! The approach laws: clearance helpers, the crab filter, the
//! glidepath, speed, flap and power on the approach, the live terrain floor.

use super::*;

/// The runway END a clearance heard DURING the approach moves us to, if any:
/// an arrival-shaped clearance (told to join, or cleared to land) naming a
/// different end than the one being flown, not already looked up. A
/// departure state's runway belongs to the departure; a blank or matching
/// ident changes nothing. The caller still checks the end is at THIS field.
///
/// Never during a missed approach: the go-around itself moves the tower
/// from `ClearedToLand` to `Inbound`, and the ident published with that state
/// is the runway IN USE, which on a wind dead across the strip had just
/// flipped. Five seconds into the EGTE go-around, at 60 ft, "Tower cleared us
/// for runway two six" replaced the climb-away with a fresh approach to the
/// other end; nobody had cleared anything (2026-09-21). A missed approach is
/// flown to its end, and the tower's answer to the next final report is where
/// a new end comes from.
///
/// ...unless the tower's go-around ORDER itself named the end
/// (`expected`, "expect runway one three", `GoAroundWatch::expected_runway`),
/// and the published clearance agrees: that is a re-sequence, taken once the
/// climb away is flown (the caller passes `expected` only then). LEMG,
/// 2026-09-29 (sortie 1790668215-full_auto): sent round off 30 and
/// re-sequenced to 13 at t+649, the reposition flew a right-hand pattern for
/// 30 from t+661 and learnt of 13 only at t+984, 5.5 minutes later. Pure.
pub fn clearance_switch(
    flying: &str,
    state: Option<crate::tower::State>,
    cleared: Option<&str>,
    already_checked: Option<&str>,
    going_around: bool,
    expected: Option<&str>,
) -> Option<String> {
    use crate::tower::State::{ClearedToLand, Inbound};
    if !matches!(state, Some(ClearedToLand | Inbound)) {
        return None;
    }
    let c = cleared?.trim();
    if going_around && !expected.is_some_and(|e| e.trim().eq_ignore_ascii_case(c)) {
        return None;
    }
    if !clearance_differs(flying, c) {
        return None;
    }
    if already_checked.is_some_and(|a| a.trim().eq_ignore_ascii_case(c)) {
        return None;
    }
    Some(c.to_string())
}

/// Why the runway autoland arms on is (or is not) a change of the mission's
/// arrival runway, for `full_auto::retarget_arrival_runway`: the tower's
/// clearance moved it at the arm ("clearance"), the forced end was flown as
/// handed down ("arm", no change), or nothing forced it and the track picked
/// ("self_pick"). Pure. A missing forced end fails the arm before this runs.
pub fn armed_runway_why(forced: Option<&str>, flying: &str, from_clearance: bool) -> &'static str {
    if from_clearance {
        "clearance"
    } else if forced.is_some_and(|f| f.trim().eq_ignore_ascii_case(flying.trim())) {
        "arm"
    } else {
        "self_pick"
    }
}

/// Is an owed readback the LANDING one? The tower phrases what it waits for
/// (`Tower::suggested_readback`); a hold or line-up echo is never an
/// approach's to make.
pub fn landing_readback(owed: &str) -> bool {
    let o = owed.trim_start().to_ascii_lowercase();
    o.starts_with("cleared to land") || o.starts_with("cleared touch and go")
}

/// Does the standing clearance name a different runway END than the plan?
/// Idents compare trimmed and case-blind ("12" vs "12 "), a missing or blank
/// clearance never differs.
pub fn clearance_differs(planned_id: &str, cleared_id: &str) -> bool {
    let c = cleared_id.trim();
    !c.is_empty() && !c.eq_ignore_ascii_case(planned_id.trim())
}

/// Time constant of the wind-crab estimate the final's lateral law feeds
/// forward. A wind-correction angle changes over minutes (wind shear down the
/// last thousand feet); the RAW heading-minus-track read changes every tick.
/// LEMG 13, 2026-09-12: the raw value was added straight to the heading bug
/// on final, so every nose swing the autopilot made came back as a bug move
/// the same tick, unity gain, and the loop hunted. The blackbox has the
/// signature: crab standard deviation 0.5° on the intercept (where nothing
/// fed it back), then 1.2, 1.8, 2.5, 3.1, 3.7, 4.0° down the final, a ±9°
/// swing with a ~13 s period at the numbers, a standing 35 m offset the
/// short-final law could not close, and the lateral balk gate at 74 ft.
/// At 15 s the 13 s swing comes through at about a seventh of its amplitude.
/// 5 s since 2026-10-12 (for `FinalLateral`, a lead on the drift and an
/// integral answering what the crab misses; a 15 s crab held the wind of
/// half a minute before through a shear). The final's heading now takes it
/// in whole degrees (`FinalHeading`), its standing error learnt. On the
/// fitted plants under the 60 recorded finals, 15 s to 5 s took the replays
/// within 5 m at 50 ft from 56 to 59 and a 15 to 0 kt shear from 46 to 49.
pub const CRAB_TAU_S: f64 = 5.0;

/// The wind crab actually commanded: heading minus ground track, low-passed
/// with [`CRAB_TAU_S`]. Fed EVERY tick (intercept included) so it is settled
/// when the final starts. A missing read holds the last estimate rather than
/// yanking the nose by the whole angle; a filter that has never seen a read
/// says zero.
#[derive(Debug, Default, Clone, Copy)]
pub struct CrabFilter {
    pub(super) est: Option<f64>,
}

impl CrabFilter {
    pub const fn new() -> Self {
        Self { est: None }
    }

    /// Raw heading-minus-track (degrees, positive = nose right of track),
    /// `None` when either read is missing. Clamped to ±15°: beyond that it
    /// is a turn or a slip, not a wind angle.
    pub fn raw(psi_true: Option<f64>, hpath_true: Option<f64>) -> Option<f64> {
        let (psi, hpath) = (psi_true?, hpath_true?);
        Some(crate::angles::signed_diff_deg(psi, hpath).clamp(-15.0, 15.0))
    }

    /// Advance by one tick of `dt_s` seconds and return the crab to command.
    pub fn update(&mut self, raw: Option<f64>, dt_s: f64) -> f64 {
        match (raw, self.est) {
            (Some(r), None) => {
                self.est = Some(r);
                r
            }
            (Some(r), Some(e)) => {
                let a = (dt_s / CRAB_TAU_S).clamp(0.0, 1.0);
                let e = e + a * (r - e);
                self.est = Some(e);
                e
            }
            (None, e) => e.unwrap_or(0.0),
        }
    }
}

/// Feet of altitude per nm along a 3° glidepath (tan 3° × 6076 ft/nm ≈ 318).
/// Public because the circuit descends its base leg onto the same path — a
/// pattern that hands over 500 ft below it has already lost the approach.
pub const GLIDEPATH_FT_PER_NM: f64 = 318.0;

/// This approach's glidepath angle, degrees: 3 unless an approach handed a
/// steeper one down for its visual segment (`HandDown::visual_segment`; LSZS
/// RNAV 03's MAP is 996 ft above the threshold two miles out, 4.7 degrees).
/// Set at every arm, so no approach inherits another's.
/// In a test build it lives per THREAD: the LSZS visual-segment test arms a
/// 4.5 degree path, and every other test landing at the same moment flew it
/// (the balloon test failed one run in three, 2026-09-24). One landing per
/// process is the app's truth; one per test is the tests'.
/// The angle carries WHO handed it down: `true` for a published visual
/// segment (the follower at a MAP, with its missed approach stashed), `false`
/// for a circuit's path steepened over the ground (#182). Only the first is
/// a valley our own pattern must never fly into; the second is an ordinary
/// field whose go-around the terrain-probed reversal flies (LEMG 13 SR22
/// circuit 2026-09-29: a 3.26 degree circuit path was taken for LSZS, the
/// go-around found no published missed approach and said "Your aircraft"
/// at 805 ft to nobody).
#[cfg(not(test))]
pub(super) static GLIDEPATH_DEG: std::sync::Mutex<(f64, bool)> = std::sync::Mutex::new((3.0, false));
/// The stabilised gate's blackbox record (`stable_gate` at the crossing,
/// `unstabilised` when it orders a go-around): every number the rule read.
pub(super) fn stable_record(
    event: &str,
    gate: crate::stable_approach::GateKind,
    s: &crate::stable_approach::Sample,
    now: &[crate::stable_approach::Breach],
    tripped: Option<crate::stable_approach::Breach>,
) -> serde_json::Value {
    use crate::stable_approach as sa;
    serde_json::json!({
        "event": event, "gate": gate.word(), "gate_ft": gate.gate_ft(),
        "height_aal": s.height_aal_ft, "dist_aim_nm": s.dist_to_aim_nm,
        "ias": s.ias_kts, "target": s.target_kts, "gs": s.gs_kts,
        "vvi": s.vvi_fpm,
        "sink_limit": sa::sink_limit_fpm(s.target_kts, s.ias_kts, s.gs_kts, s.path_ft_per_nm),
        "above_path_ft": s.above_path_ft,
        "path_tol_ft": sa::path_tolerance_ft(s.dist_to_aim_nm),
        "gear": s.gear_down, "landing_flap": s.landing_flap,
        "breaches": now.iter().map(|b| b.word()).collect::<Vec<_>>(),
        "why": tripped.map(|b| b.word()),
        "verdict": if now.is_empty() { "stable" } else { "unstable" },
    })
}

/// What a launcher hands its autoland for this arm, carried in the launch
/// itself so no arm inherits another's: the stabilised gate its final is
/// judged at (a circuit's is visual, every other the instrument gate, #199)
/// and a path angle, `None` for 3 degrees.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HandDown {
    pub gate: crate::stable_approach::GateKind,
    /// The angle and whether it is a PUBLISHED visual segment
    /// ([`published_steep`]).
    pub glidepath: Option<(f64, bool)>,
}

impl HandDown {
    /// A circuit's final: the visual gate, and its path steepened over the
    /// ground (#182) flown at its angle, but not a valley.
    pub fn circuit(steep_deg: Option<f64>) -> Self {
        Self {
            gate: crate::stable_approach::GateKind::Visual,
            glidepath: steep_deg.map(|d| (d, false)),
        }
    }

    /// A PUBLISHED visual segment's angle (the follower, at a MAP).
    pub fn visual_segment(deg: Option<f64>) -> Self {
        Self {
            glidepath: deg.map(|d| (d, true)),
            ..Self::default()
        }
    }
}

#[cfg(test)]
thread_local! {
    pub(super) static GLIDEPATH_DEG_T: std::cell::Cell<(f64, bool)> = const { std::cell::Cell::new((3.0, false)) };
}
/// The steepest path the landing will fly.
pub const MAX_GLIDEPATH_DEG: f64 = 5.5;

#[cfg(not(test))]
pub(super) fn live_glidepath(set: Option<(f64, bool)>) -> (f64, bool) {
    let mut g = GLIDEPATH_DEG.lock().unwrap_or_else(|e| e.into_inner());
    if let Some(v) = set {
        *g = v;
    }
    *g
}
#[cfg(test)]
pub(super) fn live_glidepath(set: Option<(f64, bool)>) -> (f64, bool) {
    GLIDEPATH_DEG_T.with(|c| {
        if let Some(v) = set {
            c.set(v);
        }
        c.get()
    })
}

/// Arm this approach's angle from its hand-down (3 without one), clamped.
pub(super) fn arm_glidepath(handed: Option<(f64, bool)>) -> f64 {
    let (deg, published) = handed
        .filter(|(d, _)| d.is_finite())
        .map_or((3.0, false), |(d, p)| (d.clamp(3.0, MAX_GLIDEPATH_DEG), p));
    live_glidepath(Some((deg, published && deg > 3.0 + 1e-9))).0
}

pub(super) fn glidepath_deg() -> f64 {
    live_glidepath(None).0
}

/// Is this approach a PUBLISHED steep visual segment (LSZS), the one kind no
/// pattern of our own may reposition? A circuit's steepened path is not.
pub(super) fn published_steep() -> bool {
    let (deg, published) = live_glidepath(None);
    published && deg > 3.0 + 1e-9
}

/// Feet of path per nautical mile for this approach (318 at 3 degrees, the
/// constant every law was tuned on, returned exactly).
pub fn gp_ft_per_nm() -> f64 {
    let deg = glidepath_deg();
    if (deg - 3.0).abs() < 1e-9 {
        GLIDEPATH_FT_PER_NM
    } else {
        6076.12 * deg.to_radians().tan()
    }
}

/// tan of this approach's path angle (the 3 degree constant returned exactly).
pub(super) fn gp_tan() -> f64 {
    let deg = glidepath_deg();
    if (deg - 3.0).abs() < 1e-9 {
        GLIDEPATH_TAN
    } else {
        deg.to_radians().tan()
    }
}

/// The glidepath correction, fpm per foot of path error, sized to close the
/// error over a fixed DISTANCE of final rather than a fixed time. 1.2 fpm/ft
/// is a 50 s time constant: 0.9 nm on a 172 at 65 kt, but 2.4 nm on a 737 at
/// 170 kt, whose last mile is 22 s long. Captured from below at 1.6 nm, the
/// 737 levelled, started down late, and arrived over LFLL 35L's numbers 51 ft
/// high with a correction of 86 fpm to fix it (2026-09-17). Scaled by
/// groundspeed over the 172's, floored at the old gain so every slow airframe
/// flies the law it always did, capped so the gain never outruns the VS
/// mode's own lag.
pub fn glidepath_gain_fpm_per_ft(gs_kts: f64) -> f64 {
    GLIDEPATH_GAIN_FPM_PER_FT
        * (gs_kts / GLIDEPATH_GAIN_REF_KTS).clamp(1.0, GLIDEPATH_GAIN_MAX_MULT)
}
pub(super) const GLIDEPATH_GAIN_FPM_PER_FT: f64 = 1.2;
pub(super) const GLIDEPATH_GAIN_REF_KTS: f64 = 65.0;
pub(super) const GLIDEPATH_GAIN_MAX_MULT: f64 = 3.0;
/// Aim the glidepath at a touchdown point ~300 m (≈1,000 ft) PAST the threshold,
/// so the path crosses the numbers at a standard ~50 ft threshold-crossing
/// height and touches in the touchdown zone — not planted on the leading edge.
/// The aim point was the threshold itself, so the 3° path targeted the runway
/// EDGE at zero height (2026-07-24). Lateral tracking still anchors on the
/// threshold — it's the same centreline, just a farther aim along it — and the
/// flare still triggers off actual radio height, so it simply engages a little
/// farther in, over the touchdown zone.
pub const TOUCHDOWN_AIM_NM: f64 = 300.0 / M_PER_NM;
/// Final approach point: intercept the extended centreline this far out (nm).
pub(super) const FAP_NM: f64 = 5.0;
/// Above the path by this much the guidance still flies the straight-in and
/// dives onto the path; a join point behind was the only answer past it.
pub(super) const CAPTURE_ABOVE_FT: f64 = 500.0;
/// The share of the path law's catch-up rate a straight-in is planned on.
pub(super) const STRAIGHT_IN_CATCH_UP_SHARE: f64 = 0.8;

/// Can the height above the path come off on the way in, straight in? The
/// catch-up the path law flies over its path's own sink
/// ([`path_descent_cap_fpm`]) for the minutes to the stabilised gate, on a
/// final the pursuit can intercept from here (`circuit::interceptable`).
/// LEMG 12 (sortie 1790583663-approach, #131): re-armed from 13's final
/// 3.5 km beside 12's, the aircraft was 513 ft over the path at 7.6 nm; the
/// 500 ft rule put the join point 10.8 nm out behind it, and a ten minute
/// reversal began where a steeper descent onto an intercept was a minute
/// and a half. Pure.
pub fn straight_in_losable(above_path_ft: f64, along_nm: f64, xtk_nm: f64, gs_kts: f64) -> bool {
    let gate_nm = STABILISED_GATE_AGL_FT / gp_ft_per_nm() - TOUCHDOWN_AIM_NM;
    let run_nm = along_nm - gate_nm;
    if run_nm <= 0.0 || !crate::circuit::interceptable(xtk_nm * M_PER_NM, along_nm * M_PER_NM) {
        return false;
    }
    let gs = gs_kts.max(60.0);
    let nominal = -(gs / 60.0) * gp_ft_per_nm();
    let catch_up_fpm = nominal - path_descent_cap_fpm(nominal);
    let minutes = run_nm / gs * 60.0;
    above_path_ft <= CAPTURE_ABOVE_FT + catch_up_fpm * minutes * STRAIGHT_IN_CATCH_UP_SHARE
}

/// Does the guidance aim at the join point (out along the centreline, behind
/// an aircraft inside it) rather than fly the straight-in? Only when the
/// height above the path cannot come off on the way in
/// ([`straight_in_losable`]). The reversal's own aim asks the same, so a
/// pattern is flown only when a re-intercept is not flyable. Pure.
pub fn aims_at_join_point(above_path_ft: f64, along_nm: f64, xtk_nm: f64, gs_kts: f64) -> bool {
    above_path_ft > CAPTURE_ABOVE_FT
        && !straight_in_losable(above_path_ft, along_nm, xtk_nm, gs_kts)
}
/// Can a final still be made from `xtk_nm` off the centreline with `along_nm`
/// to run — will the pursuit's steepest cut close the offset, with margin,
/// before the threshold? The circuit asks exactly this before it hands over
/// (`circuit::interceptable`); below the terrain floor autoland asks it of
/// itself. A 370 m offset at 0.8 nm passes (the pursuit flies that out); the
/// Gibraltar handoff's 1,660 m at 1.5 nm does not. See the balk in [`guidance`].
/// The gate applies on the approach side with this much still to run: closer
/// in, the flare gate and the pursuit's own short-final law own the question
/// (and past the threshold the along-track is negative, which the intercept
/// sum would read as "can't", over a landing that is already happening).
pub(super) const STABILISED_MIN_ALONG_NM: f64 = 0.3;
/// Below this on final the approach is stabilised laterally or it is a
/// go-around (see the gate in [`guidance`]).
pub const STABILISED_GATE_AGL_FT: f64 = 1000.0;

pub fn stabilised_xtk(xtk_nm: f64, along_nm: f64) -> bool {
    crate::circuit::interceptable(xtk_nm * M_PER_NM, along_nm * M_PER_NM)
}

/// Will the final have the offset inside `limit_m` by the threshold? The
/// question below the terrain floor, where "interceptable" is true of any
/// offset a landing could still be saved from and of many it could not.
/// Above the visual minimums the time model is the stabilised gate's own
/// ([`crate::reversal::predicted_residual_m`]): 98 m off at 470 ft is a
/// go-around however the last 300 ft could be flown. Below them the final
/// is hand flown (2026-10-12), closing at up to 2.5 m/s with the last 10 m
/// over 4 s (`pilot::line`): on the gate's model a whole-degree final 31 m
/// off at 297 ft balked that the bank lands on the line
/// (`lemg30_zibo_final_tests`). Pure.
pub fn lands_on_the_pavement(xtk_nm: f64, along_nm: f64, gs_kts: f64, radio_ft: f64, limit_m: f64) -> bool {
    if radio_ft > VISUAL_MINIMUMS_FT {
        return crate::reversal::predicted_residual_m(xtk_nm, along_nm, gs_kts) <= limit_m;
    }
    let t = along_nm.max(0.0) * M_PER_NM / (gs_kts * 0.514_444).max(20.0);
    let x = xtk_nm.abs() * M_PER_NM;
    let linear_s = (x - 10.0).max(0.0) / 2.5;
    let residual = if t < linear_s { x - 2.5 * t } else { x.min(10.0) * (-(t - linear_s) / 4.0).exp() };
    residual <= limit_m
}
/// Airspeed this far below the approach speed triggers speed protection (nose
/// down to regain it), before the decay reaches a stall and disconnects the AP.
/// Approach speed is already ~1.3×Vs, so this margin still stays well above stall.
pub(super) const SPEED_FLOOR_MARGIN_KTS: f64 = 7.0;

/// Clean manoeuvring speed for the intercept, as a multiple of the airframe's
/// approach speed. The two scale together across every airframe we fly, which
/// is why this is a RATIO and not another profile field to thread through five
/// callers: the SF50's 87 kt approach gives 126 (it was flying 125 when the
/// circuit handed off), the 172's 65 gives 94, and the MD-82's 140 gives 203 —
/// within a few knots of the 200 its profile already declares as the pattern
/// speed. A profile figure can always override this later.
pub(super) const CLEAN_APPROACH_RATIO: f64 = 1.45;
/// Start slowing to the approach speed here (nm), reaching it at the FAP.
/// Commanding the approach speed from the moment autoland arms is what flew an
/// SF50 at 92 kt NINE MILES OUT with the gear up (Southampton, 2026-07-30) —
/// the airframe's own gear warning was right and we were wrong. Gear comes
/// down at 6 nm, inside this ramp, so it extends well below Vlo.
pub(super) const DECEL_START_NM: f64 = 10.0;

/// The clean manoeuvring speed for an airframe with this approach speed — what
/// autoland flies outside the deceleration ramp, and what the procedure
/// follower holds down a straight-in so the FAF handoff arrives at a speed
/// autoland expects.
pub fn clean_approach_ias(approach_ias: f64) -> f64 {
    approach_ias * CLEAN_APPROACH_RATIO
}

/// May the gear come down at this speed? The profile's VLO/VLE gates it; a
/// profile with no figure never gates. Autoland used to lower the gear the
/// moment it was inside 6 nm whatever the speed: 195 kt on a Baron placarded
/// 152 (2026-09-11).
pub fn gear_may_extend(ias_kts: f64, vlo_kts: Option<f64>) -> bool {
    vlo_kts.is_none_or(|v| ias_kts <= v)
}

/// The most flap the schedule's `wanted` ratio allows at this speed: the
/// deepest detent at or below `wanted` whose placard Vfe the speed respects.
/// `limits` is the profile's detents as `(ratio, Vfe)`, retracted first; a
/// detent with no Vfe never gates, and an empty list (no flap profile) hands
/// `wanted` straight back. The setting is held back, not skipped: as the speed
/// bleeds off, the schedule's target becomes legal and goes out then.
pub fn flap_within_limits(wanted: f64, ias_kts: f64, limits: &[(f64, Option<f64>)]) -> f64 {
    if limits.is_empty() {
        return wanted;
    }
    limits
        .iter()
        .filter(|(ratio, vfe)| *ratio <= wanted + 1e-6 && vfe.is_none_or(|v| ias_kts <= v))
        .map(|(ratio, _)| *ratio)
        .fold(0.0, f64::max)
}

/// A detent goes out only with this much room under its placard...
pub(crate) const FLAP_EXTEND_MARGIN_KTS: f64 = 5.0;
/// ...and one already out comes back in only this far over it.
pub(super) const FLAP_RETRACT_MARGIN_KTS: f64 = 8.0;

/// [`flap_within_limits`] with a memory. The bare rule is a comparator on a
/// noisy airspeed: the 737's flap 40 went out at 161 kt under a 162 placard,
/// came back to 10 at 162, and went out again at 900 ft on final (EGTE,
/// 2026-09-21). A crew that touches the placard slows down; it does not cycle
/// the flaps. Extending wants margin under the limit, a detent already out is
/// held until the speed is properly over it, and a schedule that asks for
/// LESS (the go-around's half) is obeyed at once.
pub fn flap_held_within_limits(
    wanted: f64,
    ias_kts: f64,
    limits: &[(f64, Option<f64>)],
    current: Option<f64>,
) -> f64 {
    let cur = current.unwrap_or(0.0);
    if wanted + 1e-6 < cur {
        return flap_within_limits(wanted, ias_kts, limits);
    }
    let extend = flap_within_limits(wanted, ias_kts + FLAP_EXTEND_MARGIN_KTS, limits);
    if extend > cur {
        return extend;
    }
    flap_within_limits(cur, ias_kts - FLAP_RETRACT_MARGIN_KTS, limits)
}

/// An APPROACH never takes away the flap the wing is flying on. A detent comes
/// in only at the speed that flies what is left (`retract_kts`: the cleaner
/// configuration's manoeuvring speed where the profile declares one, else its
/// speed floor), unless a placard forces it (over the current detent's Vfe,
/// [`flap_within_limits`]) or the go-around asks for its half (`go_around`).
/// LEMG 13, 2026-09-28: every sim-started 10 nm final handed autoland the 737
/// with flaps 2 (0.25) at idle, the distance schedule wanted it clean, and
/// the flaps came up on the first tick at a 58 kt airspeed read (the sim's
/// ASI still waking) and 146 kt a second later, under the 139 kt clean
/// stall; with the lever at idle and a climb on the dial the aircraft sank
/// at up to 4,400 fpm (sortie 1790583434-approach bottomed at 200 ft AGL,
/// 1790584266-approach turned onto 12's line over rising ground and hit).
/// `current` is `None` only when the flap position was never read. Pure.
pub fn flap_kept_flying(
    flap: f64,
    current: Option<f64>,
    ias_kts: f64,
    limits: &[(f64, Option<f64>)],
    go_around: bool,
    retract_kts: impl Fn(f64) -> Option<f64>,
) -> f64 {
    let Some(cur) = current else {
        return flap;
    };
    if go_around || flap + 1e-6 >= cur {
        return flap;
    }
    // Over the placard of what is out: the placard wins, it is the other
    // edge of the same envelope.
    if flap_within_limits(cur, ias_kts, limits) + 1e-6 < cur {
        return flap;
    }
    match retract_kts(flap) {
        Some(need) if !(ias_kts >= need) => cur,
        _ => flap,
    }
}

/// The speed a flap setting may be retracted TO for this airframe: the
/// detent's declared manoeuvring speed (`retract_above_kts`), else its speed
/// floor (1.2 x the stall in it), else nothing known.
pub(super) fn retract_speed_kts(ratio: f64) -> Option<f64> {
    crate::aircraft::live_profile()
        .and_then(|p| {
            crate::aircraft::flap_schedule(p.flaps.as_ref())
                .into_iter()
                .find(|(r, _)| (r - ratio).abs() < 1e-3)
                .map(|(_, k)| k)
        })
        .or_else(|| crate::aircraft::config_floor_kts(ratio))
}

/// Speed to command at this distance: clean far out, approach speed by the FAP,
/// linear between. Pure, so the schedule is testable without a sim.
pub fn scheduled_ias(dist_nm: f64, approach_ias: f64) -> f64 {
    let clean = clean_approach_ias(approach_ias);
    if dist_nm >= DECEL_START_NM {
        clean
    } else if dist_nm <= FAP_NM {
        approach_ias
    } else {
        let frac = (dist_nm - FAP_NM) / (DECEL_START_NM - FAP_NM);
        approach_ias + (clean - approach_ias) * frac
    }
}

/// The speed a reposition pattern's downwind and base are flown at: midway
/// down the deceleration schedule, and well inside the half-flap placard it is
/// flown with. The approach speed itself belongs to the final: at it, with the
/// reposition's climb bonus, the 737 flew its downwind at 139 to 162 kt under
/// flap 40's 162 kt limit.
pub fn pattern_ias(approach_ias: f64, flap_limits: &[(f64, Option<f64>)]) -> f64 {
    let mid = scheduled_ias((FAP_NM + DECEL_START_NM) / 2.0, approach_ias);
    let half_vfe = flap_limits
        .iter()
        .filter(|(ratio, _)| *ratio <= 0.5 + 1e-6)
        .filter_map(|(_, vfe)| *vfe)
        .fold(f64::INFINITY, f64::min);
    mid.min(half_vfe - 15.0).max(approach_ias)
}

/// What the copilot says when a final drops back to an intercept without a
/// go-around, or `None` when another line already says it. "Overflew the
/// runway" only with the runway BEHIND (`along_nm` negative): it was said at
/// 5.2 nm on the tick a pattern began, over the pattern's own "Repositioning"
/// line (LEMG 13, sortie 1790622093-approach, #171). Pure.
pub fn dropped_final_line(
    along_nm: f64,
    reversal_began: bool,
    above_path_ft: f64,
) -> Option<&'static str> {
    if along_nm < 0.0 {
        Some("Overflew the runway. Going around, climbing to re-intercept.")
    } else if reversal_began {
        None
    } else if above_path_ft > 0.0 {
        Some("High on the approach. Re-intercepting the glidepath.")
    } else {
        Some("Off the final. Re-intercepting.")
    }
}

/// Room past the path's own sink the glidepath law may command to get back
/// down to it, fpm.
pub(super) const PATH_CATCH_UP_FPM: f64 = 300.0;

/// The steepest descent the glidepath law commands, fpm (negative): the old
/// flat 1,200, or the path's own sink at this groundspeed plus
/// [`PATH_CATCH_UP_FPM`] when that is steeper. A flat 1,200 is the 3 degree
/// path at 226 kt over the ground: faster, the law could not even HOLD the
/// path, the error grew, and past 500 ft high the guidance aimed at a join
/// point behind the aircraft, which the reversal read as a pattern (the A330
/// at 239 kt, 491 ft high at 5.3 nm on a -1,200 dial, then a downwind from
/// 5.2 nm; LEMG 13, sortie 1790622093-approach, #171). Pure.
pub fn path_descent_cap_fpm(nominal_vs_fpm: f64) -> f64 {
    (nominal_vs_fpm - PATH_CATCH_UP_FPM).min(-1200.0)
}

/// The placard of the flap that is out: the one lookup,
/// [`crate::aircraft::flap_placard_kts`]. Pure.
pub fn flap_placard_kts(flap_ratio: f64, flap_limits: &[(f64, Option<f64>)]) -> Option<f64> {
    crate::aircraft::flap_placard_kts(flap_ratio, flap_limits)
}

/// The approach's speed target sits this far under the placard of the flap out.
pub(super) const PLACARD_TARGET_MARGIN_KTS: f64 = 10.0;

/// The speed the approach's governor holds: the schedule, never closer to the
/// placard of the flap actually out than [`PLACARD_TARGET_MARGIN_KTS`], and
/// never under that configuration's floor (the floor wins a conflict). The
/// schedule's clean speed is 1.45 x the approach speed, 223 kt on the A330's
/// 154: with CONF 1 out (Vfe 215) the governor added power to reach it, the
/// placard took the flap, the clean floor (1.2 x 175 = 210) then held the
/// lever up, and a 10 nm final ran at 200 to 241 kt (LEMG 13, sortie
/// 1790622093-approach, 2026-09-28, #171). Pure.
pub fn approach_target_kts(
    scheduled_kts: f64,
    flap_ratio: f64,
    flap_limits: &[(f64, Option<f64>)],
    floor_kts: Option<f64>,
) -> f64 {
    let ceiling = flap_placard_kts(flap_ratio, flap_limits)
        .map_or(f64::INFINITY, |v| v - PLACARD_TARGET_MARGIN_KTS);
    crate::aircraft::floored_target(scheduled_kts.min(ceiling), floor_kts)
}

/// The flap an approach asks for: the distance schedule, never LESS than is
/// already out. A flap out is drag and a lower floor, both what an approach
/// needs; only a placard (`flap_held_within_limits`) or the go-around's half
/// takes one away. The schedule wanted the A330 clean beyond 6 nm, CONF 1
/// came up at 211 kt 7.9 nm out (the clean retract speed, 210), and the clean
/// floor then kept the final at 210 and over (#171). `current` is the last
/// flap written or read. Pure.
pub fn approach_flap_wanted(scheduled: f64, current: Option<f64>) -> f64 {
    scheduled.max(current.unwrap_or(0.0))
}

/// The speed a give-up leaves the governor holding: the pattern speed, inside
/// the placard of whatever flap is actually out, never under the approach
/// speed. With the flaps up it is the clean approach speed.
pub fn hand_back_speed_kts(
    approach_ias: f64,
    flap_ratio: f64,
    flap_limits: &[(f64, Option<f64>)],
) -> f64 {
    if flap_ratio < 0.05 {
        return clean_approach_ias(approach_ias);
    }
    let placard = flap_placard_kts(flap_ratio, flap_limits).unwrap_or(f64::INFINITY);
    pattern_ias(approach_ias, flap_limits)
        .min(placard - 15.0)
        .max(approach_ias)
}

/// Past the threshold by more than this (nm along the centreline, departure side)
/// with no landing = an overflight. Go around: fly back to the FAP instead of
/// tracking the runway heading straight out. Seen live 2026-07-06: overflew the
/// numbers at 167 ft, then departed the far end on a fixed course to 5.4 nm at
/// 60 kt, never turning back. The margin ignores a normal float/rollout past the
/// threshold (the flare branch, checked first, still owns the touchdown).
pub(super) const OVERFLIGHT_GO_AROUND_NM: f64 = 0.15;
/// The float must leave at least this much pavement to land and stop on before
/// the overflight rule lets it continue — the Baron's measured comfortable
/// rollout from 79 kt used ~550 m.
pub(super) const FLOAT_MIN_REMAINING_M: f64 = 900.0;

/// How far past the threshold the aircraft may fly before "overflight" is the
/// verdict. The fixed 0.15 nm was tuned when the descent ran at path rate all
/// the way to the flare gate; the roundout legitimately floats further, and at
/// EGHI it balked a perfectly good landing at 26.3 ft radio — ONE FOOT above
/// the 25 ft flare gate — with 1,400 m of runway still ahead (2026-08-02).
/// Low over the pavement, in the landing float, the question is not "how far
/// past the numbers" but "is there still room to land and stop": allow the
/// float as long as [`FLOAT_MIN_REMAINING_M`] remains, capped at half the
/// runway (not down by halfway is a go-around on ANY runway), floored at the
/// old margin so a short strip keeps exactly the old rule. HIGH past the
/// threshold (above the roundout band) the question is geometric: where a
/// 3 degree descent from HERE reaches the band, is the float allowance still
/// ahead? The fixed 0.15 nm was a 172 figure twice over: at 60 kt a 172 that
/// crosses the numbers 50 ft high is in the band 100 m later, a 737 at 140 kt
/// is 290 m in, and EGCC 23L balked it at 57 ft and 322 m with 2,700 m of
/// runway ahead (2026-09-19). The band's own allowance less the run to the
/// band, floored at the old margin, so the 167 ft overflight over a short
/// strip is still caught and a long runway is judged by its length. The
/// pavement a landing needs is the airframe's recited landing distance when
/// it has one (the MD-82 would float Luton away on the Baron's 900 m).
pub fn overflight_allowance_nm(
    length_m: f64,
    radio_alt_ft: f64,
    band_ft: f64,
    landing_m: Option<f64>,
) -> f64 {
    let need_m = landing_m
        .filter(|v| v.is_finite() && *v > 0.0)
        .unwrap_or(FLOAT_MIN_REMAINING_M)
        .max(FLOAT_MIN_REMAINING_M);
    let low_m = (length_m - need_m).min(0.5 * length_m);
    let run_to_band_m = (radio_alt_ft - band_ft).max(0.0) * M_PER_FT / gp_tan();
    ((low_m - run_to_band_m) / M_PER_NM).max(OVERFLIGHT_GO_AROUND_NM)
}
pub(super) const M_PER_FT: f64 = 0.3048;
/// tan(3 degrees): the metres a 3 degree path runs per metre of height.
pub(super) const GLIDEPATH_TAN: f64 = 0.052_408;

/// A latched go-around climbs straight ahead until this height before the
/// intercept guidance may turn it back toward the FAP (see GoAroundLatch).
pub(super) const GO_AROUND_TURN_AGL_FT: f64 = 800.0;
/// How near the extended centreline the reposition leg must be before it
/// hands back to the ordinary approach (see [`GoAroundLatch::update`]). Half
/// a mile leaves the pursuit ~17 m of offset after four time constants on a
/// 737's 118 s final, inside the flare gate; a 172's 280 s final never
/// needed the rule.
pub(super) const REPOSITION_RELEASE_XTK_NM: f64 = 0.5;
/// Terrain floor (ft AGL): below this and outside the flare zone, low radio
/// altitude is rising terrain under the flight path, not the runway — climb away
/// from it rather than hold level into it. Seen live 2026-07-06: held level at
/// ~50 ft AGL 5 nm out over rising ground, no climb authority anywhere in the loop.
pub(super) const TERRAIN_FLOOR_AGL_FT: f64 = 500.0;

/// The flap ratio that is the landing configuration (the schedule's full).
pub(crate) const LANDING_FLAP_RATIO: f64 = 0.999;
/// The lever's idle stop in the speed law.
pub(super) const IDLE_THROTTLE: f64 = 0.1;

/// The least the speed law may pull the lever to. On FINAL, at or below the
/// target, it is the airframe's approach thrust floor (jets fly the approach
/// on thrust: the 737 idled its last mile and touched down 17 kt under
/// Vref, LFLL 2026-09-17) - but only in the LANDING configuration
/// (`landing_flaps`), the one the floor was measured in (0.4 at Vref with
/// flaps 40). With flaps 10 the same 0.3 is more than the path needs: from
/// the FAP to 3 nm every LEMG 13 final of 2026-09-26 sat at 161 to 164 kt.
/// FAST by any amount the lever is TECS's down to idle (`pilot::energy`):
/// light, 0.3 is the path's own power at the final's speed, and a floor
/// standing up to ten knots fast held the 737 at 137 to 139 kt against 129
/// from 1,000 ft to the flare and floated it (EGPH 06, 2026-10-17, sortie
/// 1792238650-approach, 51.9 t; #238 the Zibo's LEMG 30). In a REPOSITION below the
/// terrain floor it is full power: excess speed there is spent on the
/// climb, never on the lever ("land the aircraft" from 140 ft at 237 kt
/// pulled the throttle to idle for its 179 kt target and the aircraft
/// settled into the fields with +800 fpm on the dial, same day). Everywhere
/// else, idle.
pub fn approach_power_floor(
    phase: ApproachPhase,
    radio_alt_ft: f64,
    err_kts: f64,
    approach_thrust_min: Option<f64>,
    landing_flaps: bool,
) -> f64 {
    match phase {
        ApproachPhase::Intercept if radio_alt_ft < TERRAIN_FLOOR_AGL_FT => 1.0,
        ApproachPhase::Final if err_kts >= 0.0 && landing_flaps => {
            approach_thrust_min
                .unwrap_or(IDLE_THROTTLE)
                .max(IDLE_THROTTLE)
        }
        _ => IDLE_THROTTLE,
    }
}
/// A climb is flown on thrust. Repositioning (Intercept) with the vertical
/// law asking to CLIMB, the lever never sits below the airframe's approach
/// thrust floor (idle when none is declared): the speed law alone reads a
/// dive's rising airspeed as "fast" and idles. LEMG 13, 2026-09-28 (sortie
/// 1790584266-approach): +100 to +2,000 fpm on the dial under the reposition
/// floor, the lever at 0.10 from the arm until 467 ft radio, the 737 sinking
/// 1,200 to 3,700 fpm. Final and the flare keep their own rules. Pure.
pub fn climb_power_floor(
    phase: ApproachPhase,
    target_vs_fpm: f64,
    approach_thrust_min: Option<f64>,
) -> f64 {
    if phase == ApproachPhase::Intercept && target_vs_fpm > 0.0 {
        approach_thrust_min.unwrap_or(IDLE_THROTTLE).max(IDLE_THROTTLE)
    } else {
        IDLE_THROTTLE
    }
}

/// Don't enforce the terrain floor inside this range ON FINAL — here a low
/// radio altitude IS the approach (short final), not terrain to climb over.
/// The gate applies to Final only: in Intercept the aircraft is repositioning,
/// not landing, so proximity to the threshold means nothing — the KPDK 16
/// rejoin (2026-07-14) armed 0.75 nm out at 300 AGL, the gate held the floor
/// off, and the vertical law tracked the (distance-symmetric) glidepath into
/// the ground 0.35 nm short of the runway.
pub(super) const FLOOR_ENFORCE_MIN_DIST_NM: f64 = 1.0;
/// ...and on Final the floor binds only this far BELOW the glidepath. A 3°
/// path crosses 500 AGL at ~1.6 nm, so at any flat field the floor used to
/// capture a perfectly on-path approach between 1.6 and 1.0 nm — held it
/// level with the climb-energy speed bonus, then released it 200 ft high
/// and 15 kt hot at the numbers for a guaranteed overflight go-around
/// (LXGB 09, 2026-07-22). Every scenario the floor exists for (dragged in
/// low over rising ground) is deeply below the path; on-path with low
/// radio near the field IS the approach.
pub(super) const FLOOR_BELOW_PATH_FT: f64 = -100.0;
/// Repositioning (Intercept) wants PLATFORM height — the ~1,600 ft AGL the
/// aircraft must be back at by the FAP to fly the path down anyway. Not 500
/// ("flying around 500 feet, is that a bit too low?" — it was), and not
/// 1,000 either: the LXGB 27 go-around turned back toward a FAP east of the
/// field and flew level at ~650 ft AGL into the Rock of Gibraltar (1,400 ft;
/// radio collapsed 680 → 3 in six seconds, CFIT, 2026-07-22). Terrain-aware
/// routing is the real fix; until then, climb to the altitude the rejoin
/// needs and the highest terrain a sane approach corridor hides.
pub(super) const REPOSITION_MIN_AGL_FT: f64 = FAP_NM * GLIDEPATH_FT_PER_NM;
/// A pattern plan older than this is stale: the aircraft has flown on from
/// where its downwind was probed from.
pub(super) const REVERSAL_PLAN_FRESH_S: f64 = 60.0;

/// The live floor is probed this often, and a tripped floor stands this
/// long after its last trip (the circuit's 2 s and 60 s).
pub(super) const LIVE_FLOOR_EVERY_S: f64 = 2.0;
pub(super) const LIVE_FLOOR_HOLD_S: f64 = 60.0;

/// The circuit's leg whose live floor (`circuit::live_floor_ft`) a reversal
/// leg or a hold keeps: the level legs climb clear of ground ahead, the base
/// (and the ordinary intercept, descending to its path) only keeps the floor
/// itself. `None` on the final and in the go-around's own climb. Pure.
pub(super) fn live_floor_leg(
    phase: ApproachPhase,
    missed: Option<MissedStage>,
    pattern_leg: Option<crate::reversal::Leg>,
    holding: bool,
) -> Option<crate::circuit::Leg> {
    use crate::circuit::Leg as C;
    if phase != ApproachPhase::Intercept || missed == Some(MissedStage::Climb) {
        return None;
    }
    Some(match pattern_leg {
        Some(crate::reversal::Leg::Base) => C::Base,
        Some(_) => C::Downwind,
        None if holding || missed.is_some() => C::Downwind,
        None => C::Base,
    })
}

/// An ILS is intercepted LEVEL: with the localiser CAPTURED, the glideslope
/// not yet and a live beam above us, ALT holds and the glideslope captures
/// from below. Only once the localiser is captured: merely armed, a "valid"
/// glideslope is only the display's (EGBB 15, 2026-10-15: levelled at 5,150
/// ft 15 nm out, 10 km off the localiser, `vdef` a frozen -5.6e-45). Pure.
pub fn level_for_glideslope(
    coupling: crate::ils::Coupling,
    modes: Option<autoflight::Modes>,
    needles: Option<crate::ils::Needles>,
) -> bool {
    coupling == crate::ils::Coupling::LocCaptured
        && !modes.is_some_and(|m| m.gs())
        && needles.is_some_and(|n| n.gs_valid && n.vdef.is_some_and(|v| v <= 0.0))
}

/// The pilot taking the aircraft from autoland by hand: the yoke past the
/// monitor's hands figure ([`crate::safe_hold::YOKE_HANDS`], not our own
/// channel), or the autopilot in control wheel steering
/// ([`autoflight::Modes::cws`]) once its engage is confirmed, either HELD
/// for [`crate::stable_approach::PERSIST_S`]. EGBB 15 (2026-10-15): the
/// pilot pushed at 17:26:37, the 737 went to CWS, and autoland pressed V/S
/// and HDG and drove the lever to TOGA for 15 s against the hand.
#[derive(Debug, Default, Clone, Copy)]
pub struct YokeTakeover {
    since_s: Option<f64>,
}

impl YokeTakeover {
    /// One tick at `now_s`; true once the hand has held.
    pub fn step(&mut self, now_s: f64, hands: bool, cws: bool) -> bool {
        if !(hands || cws) {
            self.since_s = None;
            return false;
        }
        let since = *self.since_s.get_or_insert(now_s);
        now_s - since >= crate::stable_approach::PERSIST_S
    }
}

/// Whether the yoke reads a hand: either axis past
/// [`crate::safe_hold::YOKE_HANDS`]; a failed read is no hand. Pure.
pub fn yoke_hands(pitch: Option<f64>, roll: Option<f64>) -> bool {
    [pitch, roll]
        .into_iter()
        .flatten()
        .any(|v| v.is_finite() && v.abs() > crate::safe_hold::YOKE_HANDS)
}
