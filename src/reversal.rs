//! The course reversal: how autoland gets from the wrong side of a runway onto
//! its final, as a PATTERN (outbound, base, inbound) and never as a point.
//!
//! EGTE, 2026-09-21, a 737 re-armed from the 26 final onto 08: every
//! reposition autoland knew aimed AT a point on the extended centreline (the
//! FAP, the join point), so the aircraft arrived there flying OUTBOUND, on or
//! converging with the line it then had to turn 170 degrees onto. At 160 kt
//! that turn is three kilometres wide. "Final" was called at 3.3 nm with the
//! nose pointing away from the runway, the turn overshot to 1,880 m on the far
//! side, the pursuit closed what it could and the landing balked at 94 ft,
//! 207 m off. The second try overshot to 2,300 m and balked at 143 ft; the
//! third was killed by the receding guard on the tick its turn began.
//!
//! A reversal needs room BESIDE the centreline, so the pattern is flown there:
//!
//! - **Outbound**: a downwind PARALLEL to the centreline, offset by three turn
//!   radii on one side, tracked with a pursuit so it is a line and not a
//!   converging homing run;
//! - **Base**: perpendicular, toward the centreline, begun a radius before the
//!   base leg's own position beyond the join point;
//! - **Inbound**: the final's own fixed-lead pursuit of the centreline, begun a
//!   radius before it, flown until the ordinary guidance calls Final with the
//!   nose actually pointing at the runway.
//!
//! Every heading change is about ninety degrees, so the autopilot's heading
//! mode turns the short way each time (see `circuit::bug_toward` for what a
//! 180 degree bug step does). The side is sticky: a pattern that changes its
//! mind crosses the final it is trying to join.
//!
//! All pure but [`plan`], which probes the ground under both sides' corridors
//! (#182); the autoland loop runs it beside itself, owns the one `Reversal`
//! and applies it.

use crate::nav::{bearing_deg, project_latlon};
use crate::runway::Runway;

const M_PER_NM: f64 = 1852.0;
/// The bank the pattern is SIZED for. Shallower than any autopilot flies
/// (the 737's heading mode measured about 24 degrees on the EGTE overshoot, a
/// 172's 15.5), because the two errors are not alike: an over-estimated radius
/// flies a wider downwind and turns final early, which the inbound pursuit
/// closes gently from beyond the join point; an under-estimated one is the
/// overshoot this module exists to end.
const SIZING_BANK_DEG: f64 = 18.0;
/// Downwind offset, in turn radii: two 90 degree turns and a base leg between.
const OFFSET_RADII: f64 = 3.0;
/// The base leg sits this far beyond the join point, so the inbound leg has a
/// straight run before the path comes down to it.
pub const BASE_BEYOND_JOIN_NM: f64 = 0.5;
/// Nearer the centreline than this the aircraft has no side of its own yet (a
/// go-around climbing straight ahead) and the caller's default stands.
const OWN_SIDE_XTK_NM: f64 = 0.5;
/// The ordinary guidance may call it a final only with the nose within this of
/// the runway course. Wider than any intercept cut it commands (35 degrees)
/// plus a crab; far narrower than the 170 degrees it was called at.
pub const FINAL_MAX_TRACK_OFF_DEG: f64 = 75.0;
/// ...and a reversal is asked for when getting to the point the guidance aims
/// at means arriving there flying away from the runway.
const ARRIVAL_MAX_OFF_DEG: f64 = 90.0;
/// Patterns flown for one landing before the loop stops trying.
pub const MAX_REVERSALS: u32 = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Leg {
    Outbound,
    Base,
    Inbound,
}

impl Leg {
    pub fn word(self) -> &'static str {
        match self {
            Leg::Outbound => "outbound",
            Leg::Base => "base",
            Leg::Inbound => "inbound",
        }
    }
}

/// One pattern in progress. `side` is the sign of the cross-track the downwind
/// is flown on: +1 right of the inbound course, -1 left.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Reversal {
    pub side: f64,
    pub leg: Leg,
    /// The altitude the pattern holds, ft MSL. Fixed at the start: a platform
    /// read off the radio altimeter every tick chases the hills under it
    /// (+330 to -360 fpm commanded in "level" flight over Devon).
    pub platform_ft: f64,
    /// Too high to turn base yet ([`too_high_to_turn`], set by the caller
    /// every tick from the altitude it already has): the downwind extends.
    pub high: bool,
}

/// The gentle platform law's descent, fpm.
const PLATFORM_DESCENT_FPM: f64 = 800.0;
/// Above the platform by more than this the pattern descends at the
/// airframe's enroute rate instead.
const HIGH_ABOVE_PLATFORM_FT: f64 = 1500.0;
/// The enroute rate is capped here in the pattern: half flap, pattern speed.
const HIGH_DESCENT_CAP_FPM: f64 = 1500.0;
/// The downwind is never extended past this, nm along. Past it the turn is
/// made anyway and the stabilised gate has the last word.
const MAX_OUTBOUND_NM: f64 = 25.0;

/// The descent the pattern commands from `above_ft` over its platform, fpm
/// (positive). Pure.
pub fn pattern_descent_fpm(above_ft: f64, enroute_descent_fpm: f64) -> f64 {
    if above_ft > HIGH_ABOVE_PLATFORM_FT {
        enroute_descent_fpm.clamp(PLATFORM_DESCENT_FPM, HIGH_DESCENT_CAP_FPM)
    } else {
        PLATFORM_DESCENT_FPM
    }
}

/// Is the aircraft higher than the REST of the pattern can lose? Turning base
/// here leaves the crossing and the run back in to the join; what that takes
/// at the pattern's descent is all the height there is to give. EGSS,
/// 2026-09-22: a 737 asked to land from 16,700 ft, 13 nm out, flew the
/// pattern at 800 fpm and turned final 12,600 ft above the field. Pure.
pub fn too_high_to_turn(
    alt_ft: f64,
    platform_ft: f64,
    along_nm: f64,
    join_nm: f64,
    gs_kts: f64,
    enroute_descent_fpm: f64,
) -> bool {
    if along_nm >= MAX_OUTBOUND_NM {
        return false;
    }
    let above = alt_ft - platform_ft;
    let track_nm = OFFSET_RADII * sizing_radius_nm(gs_kts) + (along_nm - join_nm).max(0.0);
    let minutes = track_nm / gs_kts.max(50.0) * 60.0;
    above > pattern_descent_fpm(above, enroute_descent_fpm) * minutes + 300.0
}

/// An inbound leg that has passed the threshold without ever being called a
/// final has FAILED, and has no way back: the pursuit's aim is behind it, the
/// capped cut flips sign with every crossing of the centreline, and the
/// aircraft flies away on the runway heading. EGSS, 2026-09-22: inbound
/// 12,600 ft over the field, then 258/188/258 for 52 nm. The caller drops
/// the pattern; the ordinary `needed` test starts the next one or gives up.
pub fn overflown(rev: &Reversal, along_nm: f64) -> bool {
    rev.leg == Leg::Inbound && along_nm < 0.0
}

/// Why a pattern began, for its blackbox record: the missed approach's
/// reposition stage, a final called with the nose pointing away, or the
/// guidance's aim (the join point when over 500 ft above the path, else the
/// FAP) lying behind the aircraft. The record carried no reason, and the A330
/// began one at 5.2 nm on the centreline (LEMG 13, #171): 505 ft high, the
/// join point 6.9 nm out behind it. Pure.
pub fn why(missed: bool, final_called: bool, above_path_ft: f64) -> &'static str {
    if missed {
        "missed_approach"
    } else if final_called {
        "final_pointing_away"
    } else if above_path_ft > 500.0 {
        "join_point_behind"
    } else {
        "fap_behind"
    }
}

/// Turn radius the pattern is sized for, nm.
pub fn sizing_radius_nm(gs_kts: f64) -> f64 {
    let v = gs_kts.max(50.0) * 0.514_444;
    (v * v / (9.81 * SIZING_BANK_DEG.to_radians().tan())).clamp(250.0, 3000.0) / M_PER_NM
}

/// Which side to fly the downwind on. An aircraft already beside the runway
/// keeps its side (crossing the field to fly the other one crosses the final);
/// on the centreline the lower ground wins when both sides were probed, and
/// the standard left-hand pattern (left of the inbound course) otherwise.
pub fn pick_side(xtk_nm: f64, left_max_ft: Option<f64>, right_max_ft: Option<f64>) -> f64 {
    if xtk_nm.abs() >= OWN_SIDE_XTK_NM {
        return xtk_nm.signum();
    }
    match (left_max_ft, right_max_ft) {
        (Some(l), Some(r)) if r + 200.0 < l => 1.0,
        _ => -1.0,
    }
}

/// Does getting onto this final need a pattern? `aim_along_nm` is where the
/// ordinary guidance is steering (the join point or the FAP, out along the
/// centreline); `final_called` is whether it already calls this a final.
///
/// Two shapes of the same mistake. Called a final with the nose pointing away
/// (3.3 nm out, 1,250 m off, heading 271 for runway 08): position alone said
/// interceptable. Or steering to a centreline point from inside or behind it,
/// which arrives there outbound.
pub fn needed(
    rwy: &Runway,
    lat: f64,
    lon: f64,
    track_deg: f64,
    aim_along_nm: f64,
    final_called: bool,
) -> bool {
    if final_called {
        return crate::angles::abs_diff_deg(track_deg, rwy.course_deg) > FINAL_MAX_TRACK_OFF_DEG;
    }
    let recip = (rwy.course_deg + 180.0) % 360.0;
    let (alat, alon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, aim_along_nm);
    let arrival = bearing_deg(lat, lon, alat, alon);
    crate::angles::abs_diff_deg(arrival, rwy.course_deg) > ARRIVAL_MAX_OFF_DEG
}

/// Inside this of the centreline an inbound leg is ON it.
const PATH_JOIN_XTK_NM: f64 = 0.2;
/// ...and tracking within this of the runway course.
const PATH_JOIN_TRACK_DEG: f64 = 30.0;
/// The glidepath has come down to the aircraft: no more than this below it.
const PATH_JOIN_BELOW_FT: f64 = 50.0;
/// ...and no higher above it than the ordinary guidance flies straight down
/// (its `CAPTURE_ABOVE_FT`; above that it would aim at a join point behind).
const PATH_JOIN_ABOVE_FT: f64 = 500.0;

/// The inbound leg has JOINED the approach: on the centreline, pointing down
/// it, on the approach side, with the glidepath come down to it. The pattern
/// is done there, and the ordinary guidance takes the descent. Holding the
/// platform past this point is how an inbound leg overflies the field.
/// LEMG 12, 2026-09-28 (sortie 1790583663-approach): the pattern's inbound
/// leg held its 2,921 ft platform along 12's centreline from 12.4 nm, crossed
/// the glidepath at 8.9 nm (2,942 ft against 2,938, 58 m off), and flew on
/// level because only a Final (inside 5.5 nm, within 500 ft of the path)
/// could end it; at 5.5 nm it was 1,060 ft high, never a final, and went
/// over the threshold at 2,900 ft ten minutes after the retarget. Pure.
pub fn path_joined(
    rev: &Reversal,
    along_nm: f64,
    xtk_nm: f64,
    track_deg: f64,
    course_deg: f64,
    above_path_ft: f64,
) -> bool {
    rev.leg == Leg::Inbound
        && along_nm > 0.0
        && xtk_nm.abs() <= PATH_JOIN_XTK_NM
        && crate::angles::abs_diff_deg(track_deg, course_deg) <= PATH_JOIN_TRACK_DEG
        && (-PATH_JOIN_BELOW_FT..=PATH_JOIN_ABOVE_FT).contains(&above_path_ft)
}

/// A point `along_nm` out on the approach side and `lateral_nm` to the right
/// of the inbound course.
pub fn pattern_point(rwy: &Runway, along_nm: f64, lateral_nm: f64) -> (f64, f64) {
    let recip = (rwy.course_deg + 180.0) % 360.0;
    let (clat, clon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, along_nm);
    // Right of the INBOUND course is course + 90.
    let brg = (rwy.course_deg + if lateral_nm >= 0.0 { 90.0 } else { 270.0 }) % 360.0;
    project_latlon(clat, clon, brg, lateral_nm.abs())
}

/// Advance the pattern one tick and return the TRUE heading to fly, or `None`
/// when it is complete and the ordinary guidance has the final.
///
/// `along_nm` and `xtk_nm` are the aircraft's position on the runway's own
/// axes, `join_nm` the join point for the platform altitude, `final_called`
/// whether the ordinary guidance calls this tick a final.
#[allow(clippy::too_many_arguments)]
pub fn step(
    rev: &mut Reversal,
    rwy: &Runway,
    lat: f64,
    lon: f64,
    along_nm: f64,
    xtk_nm: f64,
    track_deg: f64,
    gs_kts: f64,
    join_nm: f64,
    final_called: bool,
) -> Option<f64> {
    let r = sizing_radius_nm(gs_kts);
    let offset = OFFSET_RADII * r;
    let base_along = join_nm + BASE_BEYOND_JOIN_NM + r;
    let recip = (rwy.course_deg + 180.0) % 360.0;
    let toward_line = (rwy.course_deg + if rev.side > 0.0 { 270.0 } else { 90.0 }) % 360.0;

    if rev.leg == Leg::Outbound && along_nm >= base_along - r && !rev.high {
        rev.leg = Leg::Base;
    }
    // Turn inbound a radius before the line; an aircraft that began the
    // pattern already across it (the wrong side of its own side) turns at once.
    if rev.leg == Leg::Base && xtk_nm * rev.side <= r {
        rev.leg = Leg::Inbound;
    }
    match rev.leg {
        Leg::Outbound => {
            // Pursue the offset line, never home on its far end: a homing run
            // from beside the runway converges on the point and arrives with
            // whatever spacing is left, which is how 5.3 km became 1.25.
            // An extended downwind keeps pursuing the line past the base point.
            let ahead = if rev.high {
                along_nm + 2.0 * r
            } else {
                (along_nm + 2.0 * r).min(base_along + r)
            };
            let (plat, plon) = pattern_point(rwy, ahead, rev.side * offset);
            let brg = bearing_deg(lat, lon, plat, plon);
            // Never steeper across the downwind line than 45 degrees, and
            // never back toward the field.
            let off = crate::angles::signed_diff_deg(brg, recip).clamp(-45.0, 45.0);
            Some((recip + off).rem_euclid(360.0))
        }
        Leg::Base => Some(toward_line),
        Leg::Inbound => {
            if final_called
                && crate::angles::abs_diff_deg(track_deg, rwy.course_deg) <= FINAL_MAX_TRACK_OFF_DEG
            {
                return None;
            }
            // The final's own law, from beyond the FAP: fixed lead, cut
            // capped at 35 degrees, exponential onto the line.
            let aim = crate::autoland::pursuit_aim_nm(along_nm, gs_kts, rwy.length_m);
            let (alat, alon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, aim);
            let brg = bearing_deg(lat, lon, alat, alon);
            let cut = crate::angles::signed_diff_deg(brg, rwy.course_deg).clamp(-35.0, 35.0);
            Some((rwy.course_deg + cut).rem_euclid(360.0))
        }
    }
}

/// Vertical speed that holds the pattern's platform: a deadband, a gentle
/// proportional law, never a climb without the speed for it.
pub fn platform_vs_fpm(
    platform_ft: f64,
    alt_ft: f64,
    slow: bool,
    max_climb_fpm: f64,
    enroute_descent_fpm: f64,
) -> f64 {
    let err = platform_ft - alt_ft;
    if err.abs() < 50.0 {
        return 0.0;
    }
    let vs = (err * 4.0).clamp(
        -pattern_descent_fpm(-err, enroute_descent_fpm),
        max_climb_fpm,
    );
    if slow && vs > 0.0 {
        0.0
    } else {
        vs
    }
}

/// The cross-track the final's pursuit will still carry at the threshold, m.
///
/// The pursuit converges exponentially in TIME (lead over groundspeed, about
/// thirty seconds), after a saturated first stretch at the 35 degree cut, so
/// what matters is the seconds of final left, not the geometry alone: a 35
/// degree cut "reaches" 1,880 m from 3.3 nm on paper, and a 737 with 74 s to
/// run arrived 207 m off (this model says 162).
pub fn predicted_residual_m(xtk_nm: f64, along_nm: f64, gs_kts: f64) -> f64 {
    let gs = gs_kts.max(40.0) * 0.514_444;
    let lead_m = (gs_kts / 120.0).clamp(0.6, 1.5) * M_PER_NM;
    let tau = lead_m / gs;
    let x = xtk_nm.abs() * M_PER_NM;
    let t = along_nm.max(0.0) * M_PER_NM / gs;
    let cut = 35.0_f64.to_radians();
    let sat = lead_m * cut.tan();
    let t1 = (x - sat).max(0.0) / (gs * cut.sin());
    if t1 >= t {
        return x - t * gs * cut.sin();
    }
    x.min(sat) * (-(t - t1) / tau).exp()
}

/// More than this predicted at the threshold and the final is not stabilised.
/// Three flare gates (`FLARE_MAX_XTK_NM` is 37 m): the pursuit keeps closing
/// over the pavement, and a model is owed a margin before it may balk a
/// landing that would have made it.
pub const RESIDUAL_MAX_M: f64 = 120.0;

pub fn stabilisable(xtk_nm: f64, along_nm: f64, gs_kts: f64) -> bool {
    predicted_residual_m(xtk_nm, along_nm, gs_kts) <= RESIDUAL_MAX_M
}

// ---------------------------------------------------------------------------
// Terrain (#182): the pattern is judged on the corridor it is flown in, by
// the circuit's own rule (`circuit::choose_pattern_with`).
//
// LEMG 13, 2026-09-29 (sortie 1790668215-full_auto, 737): the old rule
// probed eight points on the drawn downwind line, 1.9 nm apart, and flew
// its platform 1,000 ft over the highest. The left-hand downwind at 2,600
// ft (probed 1,644) was flown 0.4 to 0.6 nm outside the line and passed
// 522 ft over 2,043 ft of hill behind Malaga (t+1795), 633 ft on the
// pattern before it (t+1320). Neither base nor inbound was probed at all.
// ---------------------------------------------------------------------------

/// Sample spacing on the drawn pattern and on its corridor copies, nm. The
/// circuit's 150 / 300 m are for a two-mile rectangle; a reversal is thirty
/// miles of legs and is probed while flying, so a little coarser (the hills
/// behind Malaga are kilometres wide).
const LINE_STEP_NM: f64 = 0.2;
const CORRIDOR_STEP_NM: f64 = 0.3;
/// The downwind is probed from where the aircraft is, or a mile past the
/// threshold, and never from further back than this.
const PROBE_BEHIND_NM: f64 = 8.0;
/// A platform whose join point lies further out than this is no pattern to
/// fly round a runway: refused, with the number. 4,150 ft over the field.
const MAX_JOIN_NM: f64 = 15.0;
/// Corridor probes before the platform must have settled: a raise moves the
/// join point, and the base, over other ground.
const MAX_PROBE_ROUNDS: u32 = 4;
/// ...settled within this.
const SETTLED_FT: f64 = 50.0;
/// The refusal's hold looks this far along the track it holds.
const HOLD_LOOK_NM: f64 = 12.0;

/// One probe point of a pattern, on the runway's axes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProbePt {
    /// Out along the extended centreline, nm (positive on the approach side).
    pub along_nm: f64,
    /// Right of the inbound course, nm.
    pub lateral_nm: f64,
    pub leg: Leg,
    /// On the drawn line (the old check's points), not a corridor copy.
    pub line: bool,
    /// Flown at the platform; else on the path down from it.
    pub level: bool,
}

/// The legs one side's pattern flies at `platform_ft`: the downwind (from
/// where the aircraft is, `along_now`, to the base) and the base, drawn and
/// in the corridor the autopilot really flies, `circuit::CORRIDOR_OUT_RADII`
/// outside and `circuit::CORRIDOR_IN_RADII` inside (on the LEMG 13 downwind
/// it flew 0.3 to 0.45 radii outside). The inbound is both sides' and is
/// [`inbound_frame`]'s. Pure.
pub fn side_frame(
    side: f64,
    platform_ft: f64,
    elev_ft: f64,
    gs_kts: f64,
    along_now: f64,
) -> Vec<ProbePt> {
    let r = sizing_radius_nm(gs_kts);
    let offset = OFFSET_RADII * r;
    let base_along = crate::autoland::join_point_nm(platform_ft, elev_ft) + BASE_BEYOND_JOIN_NM + r;
    let a0 = along_now.clamp(-PROBE_BEHIND_NM, -1.0);
    let mut copies = vec![(0.0, true, LINE_STEP_NM)];
    copies.extend(
        crate::circuit::CORRIDOR_OUT_RADII
            .iter()
            .map(|k| (k * r, false, CORRIDOR_STEP_NM)),
    );
    copies.push((
        -crate::circuit::CORRIDOR_IN_RADII * r,
        false,
        CORRIDOR_STEP_NM,
    ));
    let mut out = Vec::new();
    for (d, line, step) in copies {
        let x = side * (offset + d);
        let a1 = base_along + d;
        for (leg, from, to) in [
            (Leg::Outbound, (a0, x), (a1, x)),
            (Leg::Base, (a1, x), (a1, 0.0)),
        ] {
            for (along_nm, lateral_nm) in crate::circuit::densify(&[from, to], step) {
                out.push(ProbePt {
                    along_nm,
                    lateral_nm,
                    leg,
                    line,
                    level: true,
                });
            }
        }
    }
    out
}

/// The inbound leg on the centreline, from beyond the widest base in to
/// `circuit::DESCENT_CHECK_FROM_M` (inside it the descent is the landing's):
/// level at the platform until the 3 degree path comes down to it
/// ([`path_joined`]), then on the path. Pure.
pub fn inbound_frame(platform_ft: f64, elev_ft: f64, gs_kts: f64) -> Vec<ProbePt> {
    let r = sizing_radius_nm(gs_kts);
    let base_along = crate::autoland::join_point_nm(platform_ft, elev_ft) + BASE_BEYOND_JOIN_NM + r;
    let top = (platform_ft - elev_ft) / crate::autoland::GLIDEPATH_FT_PER_NM;
    let from = base_along + r;
    let to = crate::circuit::DESCENT_CHECK_FROM_M / M_PER_NM;
    crate::circuit::densify(&[(from, 0.0), (to, 0.0)], LINE_STEP_NM)
        .into_iter()
        .map(|(along_nm, _)| ProbePt {
            along_nm,
            lateral_nm: 0.0,
            leg: Leg::Inbound,
            line: true,
            level: along_nm > top,
        })
        .collect()
}

/// A side's terrain facts from its probe points and the ground under each
/// (feet MSL, same order), in the circuit's terms: the highest ground under
/// the legs flown at the platform, the drawn line's highest (the old
/// number), and the path's samples as the circuit's final. Pure.
pub fn side_facts(pts: &[ProbePt], ground_ft: &[f64]) -> crate::circuit::SideFacts {
    use crate::circuit::{DescentLeg, GroundSample};
    let mut f = crate::circuit::SideFacts::default();
    let max = |m: Option<f64>, v: f64| Some(m.map_or(v, |m: f64| m.max(v)));
    for (p, &e) in pts.iter().zip(ground_ft) {
        if p.line && p.leg != Leg::Inbound {
            f.line_max_ft = max(f.line_max_ft, e);
        }
        if p.level {
            f.terrain_max_ft = max(f.terrain_max_ft, e);
        } else {
            let m = p.along_nm * M_PER_NM;
            f.descent.push(GroundSample {
                leg: DescentLeg::Final,
                path_m: m,
                thr_m: m,
                ground_ft: e,
            });
        }
    }
    f
}

/// What the corridor says about a pattern.
#[derive(Debug, Clone, PartialEq)]
pub struct Judgement {
    /// The side to fly: +1 right of the inbound course, -1 left.
    pub side: f64,
    pub platform_ft: f64,
    /// The circuit's word for the pick (`default_side_clear`,
    /// `switched_side_for_terrain`, `raised_for_terrain`, ...).
    pub why: &'static str,
    pub pick: crate::circuit::PatternPick,
    /// `Some`: no pattern here clears the ground; spoken, with the numbers.
    pub refused: Option<String>,
}

/// Judge both sides at the default platform by the circuit's rule
/// ([`crate::circuit::choose_pattern_with`], `PickRules::reversal`): 1,000 ft
/// over the level legs' corridor, the other side if it is clean, else the
/// lower side raised; the path down from the platform keeps the circuit's
/// 500 ft (tapering inside 2 nm). The loop flies no steepened path and no
/// join point past [`MAX_JOIN_NM`]: those are refusals. Pure.
pub fn judge(
    default_side: f64,
    l: &crate::circuit::SideFacts,
    r: &crate::circuit::SideFacts,
    elev_ft: f64,
    default_platform_ft: f64,
) -> Judgement {
    let pick = crate::circuit::choose_pattern_with(
        default_side < 0.0,
        l,
        r,
        elev_ft,
        default_platform_ft - elev_ft,
        crate::circuit::PickRules::reversal(),
    );
    let check = if pick.left {
        &pick.descent_left
    } else {
        &pick.descent_right
    };
    let join = crate::autoland::join_point_nm(pick.tpa_msl_ft, elev_ft);
    let refused = if pick.refused.is_some() {
        Some(
            check
                .problem
                .clone()
                .or_else(|| pick.descent_left.problem.clone())
                .unwrap_or_else(|| "the final does not clear the ground".into()),
        )
    } else if pick.shape.steep() {
        Some(format!(
            "the final would pass {:.0} feet over the ground, clearing it needs a {:.1} degree path",
            (check.default_min_ft.unwrap_or(0.0).max(0.0) / 10.0).round() * 10.0,
            pick.shape.glidepath_deg()
        ))
    } else if join > MAX_JOIN_NM {
        Some(format!(
            "clearing the hills needs {:.0} feet, joining the final {:.0} miles out",
            (pick.tpa_msl_ft / 100.0).ceil() * 100.0,
            join
        ))
    } else {
        None
    };
    Judgement {
        side: if pick.left { -1.0 } else { 1.0 },
        platform_ft: pick.tpa_msl_ft,
        why: pick.why,
        pick,
        refused,
    }
}

/// The next corridor to probe after a judgement at `built_for_ft`: `Some`
/// (the raised platform) while a raise moves the pattern past the corridor
/// it was judged on, `None` once it has settled. Pure.
pub fn reprobe_at(built_for_ft: f64, j: &Judgement) -> Option<f64> {
    (j.refused.is_none() && j.platform_ft > built_for_ft + SETTLED_FT).then_some(j.platform_ft)
}

/// The altitude a refusal climbs to and holds, feet MSL: 1,000 ft over the
/// highest ground under and ahead along the track it holds (`ahead_max_ft`),
/// never below where the aircraft is or the default platform, in hundreds.
/// No ground data: the highest ground the corridor found instead. Pure.
pub fn hold_altitude_ft(
    ahead_max_ft: Option<f64>,
    corridor_max_ft: Option<f64>,
    alt_ft: f64,
    default_platform_ft: f64,
) -> f64 {
    let ground = ahead_max_ft
        .or(corridor_max_ft)
        .unwrap_or(f64::NEG_INFINITY);
    let want = (ground + crate::circuit::PATTERN_TERRAIN_CLEAR_FT)
        .max(alt_ft)
        .max(default_platform_ft);
    (want / 100.0).ceil() * 100.0
}

/// Straight ahead at a held altitude (#182): autoland's while the
/// reversal's plan is probed (`waiting`), or because no pattern clears the
/// ground (`refused`), which climbs to a height clear of the ground ahead and
/// hands the aircraft over only once it holds it there. The circuit flies
/// the same refusal when LXGB's Rock route and rectangle are both refused in
/// the air (`circuit::hold_clear_of_terrain`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TerrainHold {
    pub hdg_true: f64,
    pub alt_ft: f64,
    pub refused: bool,
    /// Consecutive ticks level at the hold's altitude.
    level_ticks: u32,
}

/// Ticks level at a refusal's altitude before the aircraft is handed over.
pub const HOLD_LEVEL_TICKS: u32 = 8;
/// ...level within this.
pub const HOLD_LEVEL_FT: f64 = 100.0;

impl TerrainHold {
    pub fn waiting(hdg_true: f64, alt_ft: f64) -> Self {
        TerrainHold {
            hdg_true,
            alt_ft,
            refused: false,
            level_ticks: 0,
        }
    }

    pub fn refused(hdg_true: f64, alt_ft: f64) -> Self {
        TerrainHold {
            refused: true,
            ..Self::waiting(hdg_true, alt_ft)
        }
    }

    /// One tick at `alt_ft`: true once a refusal has held its altitude long
    /// enough to be handed over (the autopilot holding it). Pure.
    pub fn level_now(&mut self, alt_ft: f64) -> bool {
        if (alt_ft - self.alt_ft).abs() <= HOLD_LEVEL_FT {
            self.level_ticks += 1;
        } else {
            self.level_ticks = 0;
        }
        self.refused && self.level_ticks >= HOLD_LEVEL_TICKS
    }
}

/// The highest ground under the aircraft and [`HOLD_LOOK_NM`] along
/// `track_deg`, feet MSL, and the altitude a refusal holds over it
/// ([`hold_altitude_ft`], `corridor_max_ft` standing in when the probe does
/// not answer). One probe for autoland's reversal and the circuit.
pub async fn hold_altitude_ahead_ft(
    xp: &crate::xplane::XPlaneClient,
    (lat, lon, track_deg, alt_ft): (f64, f64, f64, f64),
    corridor_max_ft: Option<f64>,
    default_platform_ft: f64,
) -> (Option<f64>, f64) {
    let mut pts = vec![(lat, lon)];
    pts.extend(crate::procedure::look_ahead_points(
        lat,
        lon,
        track_deg,
        HOLD_LOOK_NM,
        crate::terrain::MAX_POINTS - 1,
    ));
    let ahead = crate::terrain::max_elevation_ft(xp, &pts).await;
    (
        ahead,
        hold_altitude_ft(ahead, corridor_max_ft, alt_ft, default_platform_ft),
    )
}

// ---------------------------------------------------------------------------
// No-go airspace: the pattern's legs are judged against the P/R/Q volumes the
// sim ships (`airspace.txt`, the circuit's own `SideFacts.airspace`), and the
// downwind is never EXTENDED into one: height is lost in a hold short of it.
//
// LEGR-LXGB 09, 2026-10-06 (sortie 1791275171-full_auto, 737): the
// reposition began 12 nm east of the field 10,100 ft above its platform,
// went round Europa Point at 7,700 ft, and flew its downwind west along
// 36.07 N to 5.61 W, 12 nm past the threshold over Spanish land between
// Algeciras and Tarifa, then a base north over land and a 12 nm final over
// Los Barrios: R-164 ALGECIRAS (GND to 30,000 ft) under most of it.
// ---------------------------------------------------------------------------

/// The hold's straight legs, nm: back toward the field on a line two radii
/// outside the downwind, then the downwind again. With two half turns a lap
/// is about five minutes for a jet, 7,000 ft at the pattern's descent.
pub const HOLD_LEG_NM: f64 = 3.0;
/// The hold's turn begins this far short of the first downwind sample in a
/// volume, nm, on top of the turn's own radius (a 180 away overshoots by it).
const AIRSPACE_MARGIN_NM: f64 = 0.5;

/// Where an extended downwind must stop for no-go airspace, and whether it
/// may lose its height in a hold there.
#[derive(Debug, Clone, PartialEq)]
pub struct AirspaceLimit {
    /// The downwind is never flown past this, nm along, while high: a radius
    /// and a margin short of the first sample inside a volume.
    pub extend_to_nm: f64,
    /// The first sample inside it, nm along.
    pub conflict_nm: f64,
    /// The volume, spoken ("restricted area R 164").
    pub name: String,
    /// The hold short of it is clear of every volume and of the ground (the
    /// pattern's 1,000 ft): lose the height there. Else the downwind turns in
    /// at the limit, high, and the stabilised gate has the last word.
    pub hold: bool,
}

/// The first volume a pattern side's LEVEL legs (the drawn downwind and base,
/// the line points of [`side_frame`]) enter at `platform_ft`, spoken. The
/// inbound is the runway's own final, flown wherever the runway is. Pure.
pub fn side_airspace(
    rwy: &Runway,
    pts: &[ProbePt],
    platform_ft: f64,
    spaces: &[crate::airspace::Airspace],
) -> Option<String> {
    let lls: Vec<(f64, f64)> = pts
        .iter()
        .filter(|p| p.line && p.leg != Leg::Inbound)
        .map(|p| pattern_point(rwy, p.along_nm, p.lateral_nm))
        .collect();
    crate::airspace::first_conflict(spaces, &lls, platform_ft).map(|a| a.spoken_name())
}

/// The first sample of the downwind line (`side`, `offset_nm` out) from
/// `from_nm` to [`MAX_OUTBOUND_NM`] inside a volume at `platform_ft` or at
/// `alt_ft` (the height it will be flown down from): `(along, name)`, or
/// `None` when the whole extension is clear. Pure.
pub fn downwind_conflict(
    rwy: &Runway,
    side: f64,
    offset_nm: f64,
    from_nm: f64,
    platform_ft: f64,
    alt_ft: f64,
    spaces: &[crate::airspace::Airspace],
) -> Option<(f64, String)> {
    let from = from_nm.min(MAX_OUTBOUND_NM);
    let x = side * offset_nm;
    crate::circuit::densify(&[(from, x), (MAX_OUTBOUND_NM, x)], LINE_STEP_NM)
        .into_iter()
        .find_map(|(a, x)| {
            let (lat, lon) = pattern_point(rwy, a, x);
            spaces
                .iter()
                .find(|s| s.blocks(lat, lon, platform_ft) || s.blocks(lat, lon, alt_ft))
                .map(|s| (a, s.spoken_name()))
        })
}

/// The hold short of `extend_to_nm`: the racetrack's outer leg, two radii
/// outside the downwind, from [`HOLD_LEG_NM`] and a radius back to a radius
/// past the turn, and the two half turns across. Level points, for the
/// ground probe and the airspace check. Pure.
pub fn hold_frame(side: f64, offset_nm: f64, r_nm: f64, extend_to_nm: f64) -> Vec<ProbePt> {
    let outer = side * (offset_nm + 2.0 * r_nm);
    let inner = side * offset_nm;
    let (near, far) = (extend_to_nm - HOLD_LEG_NM - r_nm, extend_to_nm + r_nm);
    crate::circuit::densify(
        &[(near, inner), (near, outer), (far, outer), (far, inner)],
        LINE_STEP_NM,
    )
    .into_iter()
    .map(|(along_nm, lateral_nm)| ProbePt {
        along_nm,
        lateral_nm,
        leg: Leg::Outbound,
        line: false,
        level: true,
    })
    .collect()
}

/// The limit a downwind conflict at `conflict_nm` sets, nm along: a radius
/// and the margin short of it, never behind where the downwind begins. Pure.
pub fn extend_to_nm(conflict_nm: f64, r_nm: f64, from_nm: f64) -> f64 {
    (conflict_nm - r_nm - AIRSPACE_MARGIN_NM).max(from_nm)
}

/// One tick of the downwind's height rule under an [`AirspaceLimit`]:
/// `high` is [`too_high_to_turn`]; `back` the hold's outer leg in hand,
/// carried tick to tick. Returns what the pattern flies: `high` as the
/// pattern sees it (an extension stopped at the limit turns in), and whether
/// this tick is the hold's way back toward the field. No limit: today's
/// rule, the downwind extends to [`MAX_OUTBOUND_NM`]. Pure.
pub fn limit_step(
    high: bool,
    along_nm: f64,
    limit: Option<&AirspaceLimit>,
    back: &mut bool,
) -> LimitStep {
    let Some(l) = limit.filter(|_| high) else {
        *back = false;
        return LimitStep {
            high,
            back: false,
            capped: false,
        };
    };
    if !l.hold {
        let capped = along_nm >= l.extend_to_nm;
        *back = false;
        return LimitStep {
            high: !capped,
            back: false,
            capped,
        };
    }
    if *back {
        if along_nm <= l.extend_to_nm - HOLD_LEG_NM {
            *back = false;
        }
    } else if along_nm >= l.extend_to_nm {
        *back = true;
    }
    LimitStep {
        high: true,
        back: *back,
        capped: false,
    }
}

/// What [`limit_step`] decided.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LimitStep {
    /// The pattern is still too high to turn in.
    pub high: bool,
    /// Flying the hold's outer leg back toward the field.
    pub back: bool,
    /// An extension stopped at the airspace with no hold: turning in high.
    pub capped: bool,
}

/// The TRUE heading of the hold's outer leg: back toward the field on the
/// line two radii outside the downwind, pursued like the downwind itself
/// (never steeper across it than 45 degrees). From the downwind the turn
/// onto it is the short way, AWAY from the runway; from it back onto the
/// downwind ([`step`]'s outbound pursuit), toward it. Pure.
pub fn hold_heading(
    rwy: &Runway,
    side: f64,
    lat: f64,
    lon: f64,
    along_nm: f64,
    gs_kts: f64,
) -> f64 {
    let r = sizing_radius_nm(gs_kts);
    let outer = side * (OFFSET_RADII * r + 2.0 * r);
    let (plat, plon) = pattern_point(rwy, along_nm - 2.0 * r, outer);
    let brg = bearing_deg(lat, lon, plat, plon);
    let off = crate::angles::signed_diff_deg(brg, rwy.course_deg).clamp(-45.0, 45.0);
    (rwy.course_deg + off).rem_euclid(360.0)
}

/// What the crew says as a pattern begins: the runway, the side, the
/// platform, and why it is where it is (raised for the ground, the other
/// side, or a volume no side was clear of). Pure.
pub fn reposition_line(
    runway_spoken: &str,
    side: f64,
    platform_ft: f64,
    why: &str,
    raised: bool,
    airspace: Option<&str>,
) -> String {
    let because = match (why, airspace) {
        ("switched_side_for_terrain", _) => ", the other side is high ground".to_string(),
        ("switched_side_for_airspace", _) => ", the other side is restricted airspace".to_string(),
        (_, Some(name)) if raised => {
            format!(", raised for the high ground. No side is clear of {name}")
        }
        (_, Some(name)) => format!(". No side is clear of {name}"),
        ("raised_for_terrain", None) => ", raised for the high ground".to_string(),
        _ => String::new(),
    };
    format!(
        "Repositioning for runway {runway_spoken}: {} downwind at {:.0} feet{because}.",
        if side < 0.0 {
            "left-hand"
        } else {
            "right-hand"
        },
        (platform_ft / 100.0).round() * 100.0,
    )
}

/// What the crew says as the downwind begins to lose height it has too much
/// of: extended, or held short of a volume. Pure.
pub fn extend_line(above_ft: f64, limit: Option<&AirspaceLimit>) -> String {
    match limit {
        Some(l) if l.hold => format!(
            "We're {above_ft:.0} feet above the pattern. Holding clear of {} to lose it.",
            l.name
        ),
        _ => format!(
            "We're {above_ft:.0} feet above the pattern. Extending the downwind to lose it."
        ),
    }
}

/// ...and when there is no legal room to lose it before the volume. Pure.
pub fn capped_line(above_ft: f64, name: &str) -> String {
    format!("No room to lose the height clear of {name}. Turning in {above_ft:.0} feet high.")
}

/// The no-go volumes near the threshold, or `None` when the file cannot be
/// read (no data: the old behaviour, said in the record). Blocking.
pub fn load_spaces_near(path: &str, lat: f64, lon: f64) -> Option<Vec<crate::airspace::Airspace>> {
    std::fs::File::open(path).ok()?;
    Some(crate::airspace::load_near(path, lat, lon, 0.5))
}

/// A pattern planned over the probed ground, for the loop to fly.
#[derive(Debug, Clone)]
pub struct PatternPlan {
    pub side: f64,
    pub platform_ft: f64,
    pub why: &'static str,
    /// No pattern clears the ground: why, with the numbers.
    pub refused: Option<String>,
    /// ...and the altitude the refusal climbs to and holds.
    pub hold_ft: Option<f64>,
    /// The flown side's level legs enter this no-go volume (spoken): no side
    /// was clear of it (`airspace_unavoidable`), flown for the ground's sake
    /// and said.
    pub airspace: Option<String>,
    /// Where the downwind must stop for airspace while high.
    pub limit: Option<AirspaceLimit>,
    /// The facts for the `reversal` record.
    pub record: serde_json::Value,
}

/// Ground under `pts` (runway axes), feet MSL, in probe-sized chunks; any
/// unanswered chunk is no data at all, never a partial picture read as clear.
async fn ground_under(
    xp: &crate::xplane::XPlaneClient,
    rwy: &Runway,
    pts: &[ProbePt],
) -> Option<Vec<f64>> {
    let lls: Vec<(f64, f64)> = pts
        .iter()
        .map(|p| pattern_point(rwy, p.along_nm, p.lateral_nm))
        .collect();
    let mut out = Vec::with_capacity(lls.len());
    for chunk in lls.chunks(crate::terrain::MAX_POINTS) {
        out.extend(crate::terrain::sample_ft(xp, chunk).await?);
    }
    Some(out)
}

fn check_json(c: &crate::circuit::DescentCheck) -> serde_json::Value {
    serde_json::json!({"default_min_ft": c.default_min_ft, "flown_min_ft": c.flown_min_ft,
                       "glidepath_deg": c.shape.map(|s| s.glidepath_deg()),
                       "problem": c.problem})
}

/// Plan the pattern over the ground the sim reports (#182): probe both
/// sides' corridors at the default platform, judge them ([`judge`]), and
/// while a raise moves the pattern out, probe again where it moved to
/// ([`reprobe_at`]). A refusal also probes the track ahead for the height
/// its hold climbs to ([`hold_altitude_ft`]). No probe data: the old
/// behaviour, the default platform on [`pick_side`]'s side. Runs beside the
/// loop, which keeps flying while the plugin answers (a few seconds).
#[allow(clippy::too_many_arguments)]
pub async fn plan(
    xp: crate::xplane::XPlaneClient,
    rwy: Runway,
    along_now: f64,
    xtk_now: f64,
    gs_kts: f64,
    default_platform_ft: f64,
    (lat, lon, track_deg, alt_ft): (f64, f64, f64, f64),
) -> PatternPlan {
    let mut built_for = default_platform_ft;
    let mut rounds = 0u32;
    let mut first_lines: Option<(Option<f64>, Option<f64>)> = None;
    // The no-go volumes near the field (off the loop's thread: the file is
    // tens of MB). Unreadable: no airspace rule, the old behaviour, recorded.
    let spaces: Option<Vec<crate::airspace::Airspace>> = {
        let path = crate::settings::live().airspace_path.clone();
        let (lat, lon) = (rwy.thr_lat, rwy.thr_lon);
        tokio::task::spawn_blocking(move || load_spaces_near(&path, lat, lon))
            .await
            .ok()
            .flatten()
    };
    let airspace_word = if spaces.is_some() { "read" } else { "unread" };
    let no_data = |rounds: u32| {
        let side = pick_side(xtk_now, None, None);
        PatternPlan {
            side,
            platform_ft: default_platform_ft,
            why: "no_terrain_data",
            refused: None,
            hold_ft: None,
            airspace: None,
            limit: None,
            record: serde_json::json!({"terrain": "none", "rounds": rounds,
                                       "airspace": airspace_word}),
        }
    };
    loop {
        rounds += 1;
        let (lp, rp) = (
            side_frame(-1.0, built_for, rwy.elev_ft, gs_kts, along_now),
            side_frame(1.0, built_for, rwy.elev_ft, gs_kts, along_now),
        );
        let ip = inbound_frame(built_for, rwy.elev_ft, gs_kts);
        let (Some(lg), Some(rg), Some(ig)) = (
            ground_under(&xp, &rwy, &lp).await,
            ground_under(&xp, &rwy, &rp).await,
            ground_under(&xp, &rwy, &ip).await,
        ) else {
            return no_data(rounds);
        };
        let facts = |pts: &[ProbePt], g: &[f64]| {
            let all: Vec<ProbePt> = pts.iter().chain(ip.iter()).copied().collect();
            let ground: Vec<f64> = g.iter().chain(ig.iter()).copied().collect();
            side_facts(&all, &ground)
        };
        let (mut l, mut r) = (facts(&lp, &lg), facts(&rp, &rg));
        if let Some(sp) = spaces.as_deref() {
            l.airspace = side_airspace(&rwy, &lp, built_for, sp);
            r.airspace = side_airspace(&rwy, &rp, built_for, sp);
        }
        let lines = *first_lines.get_or_insert((l.line_max_ft, r.line_max_ft));
        let default_side = pick_side(xtk_now, lines.0, lines.1);
        let j = judge(default_side, &l, &r, rwy.elev_ft, default_platform_ft);
        let next = reprobe_at(built_for, &j);
        let unsettled = next.is_some() && rounds >= MAX_PROBE_ROUNDS;
        if next.is_some() && !unsettled {
            built_for = next.unwrap_or(built_for);
            continue;
        }
        // Flown at the corridor it was judged on, never under the pick.
        let platform_ft = built_for.max(j.platform_ft);
        let refused = if unsettled {
            Some(format!(
                "the ground keeps rising under a longer pattern, {:.0} feet and climbing",
                (j.platform_ft / 100.0).ceil() * 100.0
            ))
        } else {
            j.refused.clone()
        };
        let hold_ft = match refused {
            Some(_) => {
                let corridor = [l.terrain_max_ft, r.terrain_max_ft]
                    .into_iter()
                    .flatten()
                    .reduce(f64::max);
                let (_, hold) = hold_altitude_ahead_ft(
                    &xp,
                    (lat, lon, track_deg, alt_ft),
                    corridor,
                    default_platform_ft,
                )
                .await;
                Some(hold)
            }
            None => None,
        };
        // The downwind's limit for no-go airspace on the side flown, and the
        // hold short of it, probed: clear of every volume and 1,000 ft over
        // its ground at the platform it descends to.
        let flown_airspace = if j.side < 0.0 {
            l.airspace.clone()
        } else {
            r.airspace.clone()
        };
        let limit = match (&refused, spaces.as_deref()) {
            (None, Some(sp)) => {
                let rr = sizing_radius_nm(gs_kts);
                let offset = OFFSET_RADII * rr;
                match downwind_conflict(&rwy, j.side, offset, along_now, platform_ft, alt_ft, sp) {
                    Some((conflict_nm, name)) => {
                        let to = extend_to_nm(conflict_nm, rr, along_now);
                        let pts = hold_frame(j.side, offset, rr, to);
                        let lls: Vec<(f64, f64)> = pts
                            .iter()
                            .map(|p| pattern_point(&rwy, p.along_nm, p.lateral_nm))
                            .collect();
                        let clear_air = sp.iter().all(|s| {
                            !lls.iter().any(|&(la, lo)| {
                                s.blocks(la, lo, platform_ft) || s.blocks(la, lo, alt_ft)
                            })
                        });
                        let ground = if clear_air {
                            ground_under(&xp, &rwy, &pts).await
                        } else {
                            None
                        };
                        let hold = clear_air
                            && ground.as_ref().is_some_and(|g| {
                                g.iter().copied().fold(f64::NEG_INFINITY, f64::max)
                                    + crate::circuit::PATTERN_TERRAIN_CLEAR_FT
                                    <= platform_ft + 0.5
                            });
                        Some(AirspaceLimit {
                            extend_to_nm: to,
                            conflict_nm,
                            name,
                            hold,
                        })
                    }
                    None => None,
                }
            }
            _ => None,
        };
        let record = serde_json::json!({
            "airspace": airspace_word,
            "left_airspace": l.airspace, "right_airspace": r.airspace,
            "limit": limit.as_ref().map(|l| serde_json::json!({"extend_to_nm": l.extend_to_nm,
                "conflict_nm": l.conflict_nm, "name": l.name, "hold": l.hold})),
            "terrain": "probed", "rounds": rounds, "side_pick": j.why,
            "left_line_ft": l.line_max_ft, "right_line_ft": r.line_max_ft,
            "left_corridor_max_ft": l.terrain_max_ft, "right_corridor_max_ft": r.terrain_max_ft,
            "judged_at_ft": built_for, "picked_ft": j.platform_ft,
            "left_descent": check_json(&j.pick.descent_left),
            "right_descent": check_json(&j.pick.descent_right),
            "refused": refused, "hold_ft": hold_ft,
        });
        return PatternPlan {
            side: j.side,
            platform_ft,
            why: j.why,
            refused,
            hold_ft,
            airspace: flown_airspace,
            limit,
            record,
        };
    }
}

// ---------------------------------------------------------------------------
// Progress (2026-09-30): a missed approach and its pattern are judged by the
// ground they make, never a flat clock, as the taxi is
// (`autotaxi::ProgressWatch`).
//
// LEMG 13, iMac 2026-09-30 (sortie 1790775809-approach, SR22): the
// stabilised gate sent it round at 734 ft, the reversal planned the
// terrain-raised right-hand pattern at 2,895 ft (join point 10.3 nm), and the
// SR22 flew it as planned: 10 nm of downwind at 90 kt, base, inbound on the
// line at 9.2 nm, 90 ft under the path it was about to join. The flat 600 s
// (`MAX_MISSED_APPROACH_SECS`) expired there, "missed_approach_timeout", and
// handed a level aircraft to nobody in an unattended run. A slow single on a
// big pattern cannot finish inside a flat clock; a jet could loiter
// uselessly in it for ten minutes.
// ---------------------------------------------------------------------------

/// A pattern is stuck when its track to go has not shrunk by a step in this
/// long, s. Two minutes: the 180 degree turn onto the downwind trades along
/// track for lateral and nets little (the SR22's first step took 60 s).
pub const PATTERN_STALL_S: f64 = 120.0;
/// ...a step being this fraction of what the pattern's speed flies in that
/// time: a quarter, so a 40 kt headwind on a 90 kt downwind still progresses.
pub const PATTERN_PROGRESS_FRAC: f64 = 0.25;
/// A climb (the go-around's, or a refused pattern's hold) or an extended
/// downwind's descent is stuck with no 100 ft gained or lost in 90 s.
pub const HEIGHT_STEP_FT: f64 = 100.0;
pub const HEIGHT_STALL_S: f64 = 90.0;
/// Waiting for the pattern's plan (the probe answers in seconds; a stale
/// one is asked again after a minute): two tries.
pub const PLAN_WAIT_S: f64 = 150.0;
/// A pattern's outer bound: its planned track at the speed it began at, plus
/// the height above its platform at the pattern's descent, this many times
/// over, plus [`PATTERN_BUDGET_SLACK_S`].
pub const PATTERN_BUDGET_MARGIN: f64 = 1.5;
pub const PATTERN_BUDGET_SLACK_S: f64 = 180.0;
/// A climb's outer bound is its height at this rate, the margin and slack.
pub const CLIMB_BUDGET_FPM: f64 = 300.0;
/// A gap this long between two notes is a frozen sim (pause, replay: the
/// loop skips its tick): the clocks stand still through it.
pub const FROZEN_GAP_S: f64 = 5.0;

/// Where the pattern ends along the centreline, nm: where the glidepath
/// comes down to the platform ([`path_joined`]'s lower edge), never beyond
/// the join point. Pure.
pub fn pattern_end_nm(platform_ft: f64, elev_ft: f64, gp_ft_per_nm: f64, join_nm: f64) -> f64 {
    ((platform_ft + PATH_JOIN_BELOW_FT - elev_ft) / gp_ft_per_nm.max(1.0))
        .min(join_nm)
        .max(0.0)
}

/// The track still to fly round the pattern to its end, nm: the rest of the
/// downwind out to the base and back in along the final to `end_nm`, and the
/// lateral legs out to the downwind line and back to the centreline. Sized
/// at `sized_gs_kts`, the speed the pattern began at (the live speed moves
/// the radius a little every tick; the pattern's size does not). An
/// extended downwind grows this by what it flies, so its progress is the
/// height it loses instead ([`Progress::Pattern`] `high`). Pure.
pub fn to_go_nm(
    rev: &Reversal,
    along_nm: f64,
    xtk_nm: f64,
    sized_gs_kts: f64,
    join_nm: f64,
    end_nm: f64,
) -> f64 {
    let r = sizing_radius_nm(sized_gs_kts);
    let offset = OFFSET_RADII * r;
    let base_along = join_nm + BASE_BEYOND_JOIN_NM + r;
    let own = xtk_nm * rev.side;
    match rev.leg {
        Leg::Outbound => {
            (base_along - along_nm).max(0.0)
                + (base_along.max(along_nm) - end_nm).max(0.0)
                + (offset - own).abs()
                + offset
        }
        Leg::Base => own.max(0.0) + (along_nm - end_nm).max(0.0),
        // Not clamped at the end: an inbound leg flown on past where the
        // path should have met it (flown high) still closes on the runway,
        // and that is still progress.
        Leg::Inbound => xtk_nm.abs() + (along_nm.max(0.0) - end_nm),
    }
}

/// A pattern's outer bound, s: `to_go_nm` at `gs_kts` (floored at 60 kt)
/// plus the height above the platform at the pattern's descent (an extended
/// downwind), times [`PATTERN_BUDGET_MARGIN`], plus the slack. Pure.
pub fn pattern_budget_s(
    to_go_nm: f64,
    gs_kts: f64,
    above_platform_ft: f64,
    enroute_descent_fpm: f64,
) -> f64 {
    let track_s = to_go_nm.max(0.0) / gs_kts.max(60.0) * 3600.0;
    let above = above_platform_ft.max(0.0);
    let height_s = above / pattern_descent_fpm(above, enroute_descent_fpm) * 60.0;
    (track_s + height_s) * PATTERN_BUDGET_MARGIN + PATTERN_BUDGET_SLACK_S
}

/// A climb's outer bound, s. Pure.
pub fn climb_budget_s(to_go_ft: f64) -> f64 {
    to_go_ft.max(0.0) / CLIMB_BUDGET_FPM * 60.0 * PATTERN_BUDGET_MARGIN + PATTERN_BUDGET_SLACK_S
}

/// What the missed approach is doing this tick, and the measure of how far
/// it still has to go.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Progress {
    /// The go-around's straight-ahead climb: feet still to climb before the
    /// turn.
    Climb { to_go_ft: f64 },
    /// A refused pattern's hold: feet still to climb to it.
    Hold { to_go_ft: f64 },
    /// Straight ahead while the pattern is planned.
    Waiting,
    /// Pattern `n` (the loop's count) on `leg`: the track to go, or while
    /// `high` (an extended downwind) the height above its platform. Its
    /// budget and the speed it began at are read on its first tick.
    Pattern {
        n: u32,
        leg: Leg,
        high: bool,
        to_go_nm: f64,
        above_ft: f64,
        budget_s: f64,
        gs_kts: f64,
    },
}

impl Progress {
    pub fn word(&self) -> &'static str {
        match self {
            Progress::Climb { .. } => "climb",
            Progress::Hold { .. } => "hold",
            Progress::Waiting => "waiting",
            Progress::Pattern { .. } => "pattern",
        }
    }
}

fn leg_rank(leg: Leg) -> u8 {
    match leg {
        Leg::Outbound => 0,
        Leg::Base => 1,
        Leg::Inbound => 2,
    }
}

/// Why a missed approach was given up.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    /// No progress for its stage's stall time.
    Stalled,
    /// Past its stage's outer bound.
    OverBudget,
}

impl Verdict {
    pub fn reason(self) -> &'static str {
        match self {
            Verdict::Stalled => "missed_approach_stalled",
            Verdict::OverBudget => "missed_approach_over_budget",
        }
    }
}

/// The missed approach's (and a pattern's) progress watch. Progress is a
/// step off the stage's measure (feet to climb, track to go, height above an
/// extended downwind's platform), a leg sequenced, or a new stage; each
/// stage sets its own outer bound as it begins. Clock injected, seconds.
#[derive(Debug, Clone)]
pub struct MissedWatch {
    stage: Option<&'static str>,
    pattern_n: Option<u32>,
    leg: Option<Leg>,
    high: bool,
    /// The best measure in the current stage (feet or nm).
    best: Option<f64>,
    step_nm: f64,
    last_progress_s: f64,
    deadline_s: f64,
    last_t: f64,
}

impl MissedWatch {
    pub fn start(t_s: f64) -> Self {
        MissedWatch {
            stage: None,
            pattern_n: None,
            leg: None,
            high: false,
            best: None,
            step_nm: 0.0,
            last_progress_s: t_s,
            deadline_s: t_s + PLAN_WAIT_S,
            last_t: t_s,
        }
    }

    /// One tick. Returns the stage's word on the tick a stage begins (a new
    /// pattern included), for the record.
    pub fn note(&mut self, t_s: f64, p: Progress) -> Option<&'static str> {
        let gap = t_s - self.last_t;
        if gap > FROZEN_GAP_S {
            self.last_progress_s += gap;
            self.deadline_s += gap;
        }
        self.last_t = t_s;
        let word = p.word();
        let new_pattern = matches!(p, Progress::Pattern { n, .. } if self.pattern_n != Some(n));
        let began = self.stage != Some(word) || new_pattern;
        if began {
            self.stage = Some(word);
            self.best = None;
            self.leg = None;
            self.last_progress_s = t_s;
            self.deadline_s = t_s
                + match p {
                    Progress::Climb { to_go_ft } | Progress::Hold { to_go_ft } => {
                        climb_budget_s(to_go_ft)
                    }
                    Progress::Waiting => PLAN_WAIT_S,
                    Progress::Pattern { budget_s, .. } => budget_s,
                };
        }
        match p {
            Progress::Climb { to_go_ft } | Progress::Hold { to_go_ft } => {
                if to_go_ft <= HEIGHT_STEP_FT {
                    // There: nothing left to gain.
                    self.last_progress_s = t_s;
                } else if self.best.is_none_or(|b| to_go_ft < b - HEIGHT_STEP_FT) {
                    if self.best.is_some() {
                        self.last_progress_s = t_s;
                    }
                    self.best = Some(to_go_ft);
                }
            }
            Progress::Waiting => {}
            Progress::Pattern {
                n,
                leg,
                high,
                to_go_nm,
                above_ft,
                gs_kts,
                ..
            } => {
                if new_pattern {
                    self.pattern_n = Some(n);
                    self.step_nm =
                        PATTERN_PROGRESS_FRAC * gs_kts.max(60.0) * PATTERN_STALL_S / 3600.0;
                    self.high = high;
                }
                if self.leg.is_some_and(|l| leg_rank(leg) > leg_rank(l)) {
                    self.last_progress_s = t_s;
                }
                self.leg = Some(leg);
                // Into or out of an extended downwind the measure changes
                // (height, track), so its best starts afresh; the flip itself
                // is not progress (a flip every tick never makes a step).
                if high != self.high {
                    self.high = high;
                    self.best = None;
                }
                let (value, step) = if high {
                    (above_ft, HEIGHT_STEP_FT)
                } else {
                    (to_go_nm, self.step_nm)
                };
                if self.best.is_none_or(|b| value < b - step) {
                    if self.best.is_some() {
                        self.last_progress_s = t_s;
                    }
                    self.best = Some(value);
                }
            }
        }
        began.then_some(word)
    }

    /// Seconds since the last progress.
    pub fn stalled_s(&self, t_s: f64) -> f64 {
        (t_s - self.last_progress_s).max(0.0)
    }

    /// Seconds left of the stage's outer bound.
    pub fn budget_left_s(&self, t_s: f64) -> f64 {
        self.deadline_s - t_s
    }

    fn stall_limit_s(&self) -> f64 {
        match self.stage {
            Some("waiting") => PLAN_WAIT_S,
            Some("pattern") => PATTERN_STALL_S,
            _ => HEIGHT_STALL_S,
        }
    }

    /// Give up? Past the stage's outer bound, or no progress for its stall
    /// time.
    pub fn verdict(&self, t_s: f64) -> Option<Verdict> {
        if t_s > self.deadline_s {
            Some(Verdict::OverBudget)
        } else if self.stalled_s(t_s) > self.stall_limit_s() {
            Some(Verdict::Stalled)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::runway::{along_track_nm, cross_track_nm};

    /// LEMG 12, 2026-09-28 (sortie 1790583663-approach): the inbound leg at
    /// its 2,921 ft platform, recorded. At 9.9 nm the path is still 305 ft
    /// above: keep the platform. At 8.9 nm it has come down to us (2,942 ft
    /// against 2,938, 58 m off, tracking 116.8 for a 117 course): the
    /// pattern is done. The old rule held the platform to the threshold.
    #[test]
    fn an_inbound_leg_the_glidepath_reaches_is_an_approach() {
        let rev = Reversal {
            side: -1.0,
            leg: Leg::Inbound,
            platform_ft: 2921.9,
            high: false,
        };
        let joined = |along: f64, xtk_m: f64, trk: f64, above: f64| {
            path_joined(&rev, along, xtk_m / M_PER_NM, trk, 117.0, above)
        };
        assert!(!joined(9.9, 95.9, 115.7, 2944.7 - 3249.0), "still under the path");
        assert!(joined(8.93, 58.1, 116.8, 2942.3 - 2938.1), "the path came down to it");
        assert!(joined(7.97, 49.4, 117.6, 2900.7 - 2635.7), "265 ft high: flown down");
        // Not on the line, not pointing down it, or too high: still a pattern.
        assert!(!joined(8.93, 800.0, 116.8, 4.0));
        assert!(!joined(8.93, 58.1, 160.0, 4.0));
        assert!(!joined(7.07, 48.8, 117.4, 2863.8 - 2349.0));
        // Only the inbound leg, and only short of the field.
        assert!(!path_joined(&Reversal { leg: Leg::Base, ..rev }, 8.93, 0.03, 116.8, 117.0, 4.0));
        assert!(!joined(-0.2, 48.0, 113.0, 4.0));
    }

    fn egte_08() -> Runway {
        Runway {
            icao: "EGTE".into(),
            id: "08".into(),
            thr_lat: 50.7331,
            thr_lon: -3.4290,
            course_deg: 79.0,
            length_m: 2037.0,
            surface: 1,
            dist_nm: 0.0,
            elev_ft: 102.0,
        }
    }

    /// A kinematic aircraft on a heading-mode autopilot: constant speed, a
    /// rate-limited roll into the bank that turns it the SHORT way to the bug.
    struct Plant {
        lat: f64,
        lon: f64,
        hdg: f64,
        bank: f64,
        gs_kts: f64,
        max_bank: f64,
    }

    impl Plant {
        fn fly(&mut self, bug: f64, dt: f64) {
            let err = crate::angles::signed_diff_deg(bug, self.hdg);
            let want = (err * 1.5).clamp(-self.max_bank, self.max_bank);
            let roll = (want - self.bank).clamp(-5.0 * dt, 5.0 * dt);
            self.bank += roll;
            let v = self.gs_kts * 0.514_444;
            let rate = (9.81 * self.bank.to_radians().tan() / v).to_degrees();
            self.hdg = (self.hdg + rate * dt).rem_euclid(360.0);
            let (lat, lon) =
                project_latlon(self.lat, self.lon, self.hdg, self.gs_kts * dt / 3600.0);
            self.lat = lat;
            self.lon = lon;
        }
    }

    /// Fly the pattern, then the final's pursuit, to the threshold; returns
    /// the worst cross-track inside 3 nm and the legs seen.
    fn fly_pattern(p: Plant, rwy: &Runway, side: f64) -> (f64, Vec<Leg>) {
        let (worst, legs, _) = fly_pattern_knob(p, rwy, side, false);
        (worst, legs)
    }

    /// ...with the knob set as autoland sets it (`autoland::bug_knob`) when
    /// `held`; the third answer is how often the knob moved (0.2 degrees).
    fn fly_pattern_knob(mut p: Plant, rwy: &Runway, side: f64, held: bool) -> (f64, Vec<Leg>, u32) {
        let join = 5.75;
        let mut hold = crate::heading_bug::BugHold::default();
        let (mut t, mut moves, mut knob_at) = (0.0, 0u32, None::<f64>);
        let mut rev = Some(Reversal {
            side,
            leg: Leg::Outbound,
            platform_ft: 1700.0,
            high: false,
        });
        let mut legs = vec![Leg::Outbound];
        let mut worst_inside_3: f64 = 0.0;
        for _ in 0..6000 {
            let along = along_track_nm(p.lat, p.lon, rwy);
            let xtk = cross_track_nm(p.lat, p.lon, rwy);
            let final_called =
                rev.is_none() || (along < 5.5 && crate::autoland::stabilised_xtk(xtk, along));
            let bug = match rev.as_mut() {
                Some(r) => {
                    let h = step(
                        r,
                        rwy,
                        p.lat,
                        p.lon,
                        along,
                        xtk,
                        p.hdg,
                        p.gs_kts,
                        join,
                        final_called,
                    );
                    if legs.last() != Some(&r.leg) {
                        legs.push(r.leg);
                    }
                    h
                }
                None => None,
            };
            let bug = match bug {
                Some(b) => b,
                None => {
                    rev = None;
                    let recip = (rwy.course_deg + 180.0) % 360.0;
                    let aim = crate::autoland::pursuit_aim_nm(along, p.gs_kts, rwy.length_m);
                    let (alat, alon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, aim);
                    let cut = crate::angles::signed_diff_deg(bearing_deg(p.lat, p.lon, alat, alon), rwy.course_deg)
                        .clamp(-35.0, 35.0);
                    (rwy.course_deg + cut).rem_euclid(360.0)
                }
            };
            if rev.is_none() && along < 3.0 {
                worst_inside_3 = worst_inside_3.max(xtk.abs() * M_PER_NM);
            }
            if rev.is_none() && along <= 0.05 {
                return (worst_inside_3, legs, moves);
            }
            let mut bug = bug;
            if held {
                let (phase, leg) = match rev {
                    Some(r) => (crate::autoland::ApproachPhase::Intercept, Some(r.leg)),
                    None => (crate::autoland::ApproachPhase::Final, None),
                };
                let short = rev.is_none() && along <= crate::autoland::SHORT_FINAL_NM;
                match crate::autoland::bug_knob(phase, short, leg, None) {
                    Some((key, db)) => {
                        hold.decide(key, bug, t, db);
                        bug = hold.set().unwrap_or(bug);
                    }
                    None => hold.reset(),
                }
            }
            if knob_at.is_none_or(|a| crate::angles::abs_diff_deg(bug, a) >= 0.2) {
                moves += 1;
                knob_at = Some(bug);
            }
            t += 0.5;
            p.fly(bug, 0.5);
        }
        panic!("never reached the threshold: legs {legs:?}");
    }

    #[test]
    fn egte_737_from_the_wrong_final_joins_instead_of_overshooting() {
        // 2026-09-21 12:10:15Z: re-armed onto 08 from the 26 final, 6.5 nm
        // east of the 08 threshold, heading west at 236 kt slowing to 160.
        let rwy = egte_08();
        let (lat, lon) = project_latlon(rwy.thr_lat, rwy.thr_lon, 80.0, 6.5);
        let p = Plant {
            lat,
            lon,
            hdg: 259.0,
            bank: 0.0,
            gs_kts: 165.0,
            max_bank: 24.0,
        };
        let xtk0 = cross_track_nm(lat, lon, &rwy);
        let side = pick_side(xtk0, None, None);
        let (worst, legs) = fly_pattern(p, &rwy, side);
        assert_eq!(legs, vec![Leg::Outbound, Leg::Base, Leg::Inbound]);
        // Flown: 1,880 m and 2,300 m overshoots, 430 m off at 1 nm.
        assert!(worst < 60.0, "inside 3 nm the final was {worst:.0} m off");
    }

    #[test]
    fn a_go_around_straight_ahead_flies_a_left_hand_pattern_back() {
        // Climbing out over the far end on runway heading, on the centreline.
        let rwy = egte_08();
        let (lat, lon) = project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, 1.5);
        for (gs, bank) in [(160.0, 24.0), (75.0, 15.5)] {
            let p = Plant {
                lat,
                lon,
                hdg: rwy.course_deg,
                bank: 0.0,
                gs_kts: gs,
                max_bank: bank,
            };
            let side = pick_side(0.0, None, None);
            assert_eq!(side, -1.0);
            let (worst, legs) = fly_pattern(p, &rwy, side);
            assert_eq!(legs.last(), Some(&Leg::Inbound));
            assert!(worst < 60.0, "{gs} kt: {worst:.0} m off inside 3 nm");
        }
    }


    /// Issue #93: the patterns flown on the knob autoland now sets (each leg
    /// once, corrections past two degrees at most every five seconds, the
    /// final past half a degree every two) fly the same legs onto the same
    /// line, with the knob moved far less (EGTE 08: 30 to 39 moves against
    /// 188 to 272; worst inside 3 nm 5 to 7 m against 3 to 15).
    #[test]
    fn a_pattern_on_a_held_knob_flies_the_same_legs() {
        let rwy = egte_08();
        let (lat, lon) = project_latlon(rwy.thr_lat, rwy.thr_lon, 80.0, 6.5);
        let wrong_final = Plant {
            lat,
            lon,
            hdg: 259.0,
            bank: 0.0,
            gs_kts: 165.0,
            max_bank: 24.0,
        };
        let side = pick_side(cross_track_nm(lat, lon, &rwy), None, None);
        let (lat, lon) = project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, 1.5);
        let go_around = |gs: f64, bank: f64| Plant {
            lat,
            lon,
            hdg: rwy.course_deg,
            bank: 0.0,
            gs_kts: gs,
            max_bank: bank,
        };
        let copy = |p: &Plant| Plant { ..*p };
        for (p, side) in [
            (wrong_final, side),
            (go_around(160.0, 24.0), -1.0),
            (go_around(75.0, 15.5), -1.0),
        ] {
            let (free_worst, free_legs, free_moves) = fly_pattern_knob(copy(&p), &rwy, side, false);
            let (worst, legs, moves) = fly_pattern_knob(p, &rwy, side, true);
            assert_eq!(legs, free_legs);
            assert!(worst < 60.0, "{worst:.0} m inside 3 nm (every tick {free_worst:.0})");
            assert!(moves * 2 <= free_moves, "{moves} moves against {free_moves}");
        }
    }

    #[test]
    fn the_downwind_is_parallel_not_converging() {
        // Established on the offset line the outbound heading is the
        // reciprocal, not a bearing at a centreline point (271 flown against
        // a 259 reciprocal, closing 5.3 km of spacing to 1.25).
        let rwy = egte_08();
        let r = sizing_radius_nm(160.0);
        let (lat, lon) = pattern_point(&rwy, 1.0, -OFFSET_RADII * r);
        let mut rev = Reversal {
            side: -1.0,
            leg: Leg::Outbound,
            platform_ft: 1700.0,
            high: false,
        };
        let h = step(
            &mut rev,
            &rwy,
            lat,
            lon,
            1.0,
            -OFFSET_RADII * r,
            259.0,
            160.0,
            5.75,
            false,
        )
        .unwrap();
        assert!(
            crate::angles::abs_diff_deg(h, 259.0) < 2.0,
            "downwind heading {h:.0}"
        );
    }

    #[test]
    fn a_final_called_flying_away_needs_a_pattern_and_a_real_one_does_not() {
        let rwy = egte_08();
        // 12:15:01Z: 3.3 nm out, 1,253 m right, tracking 271.
        let (lat, lon) = pattern_point(&rwy, 3.3, 1253.0 / M_PER_NM);
        assert!(needed(&rwy, lat, lon, 271.0, 5.0, true));
        // The published approach's handoff: same place, nose at the runway.
        assert!(!needed(&rwy, lat, lon, 60.0, 5.0, true));
        // Steering at the FAP from the far side of the field arrives outbound.
        let (lat, lon) = project_latlon(rwy.thr_lat, rwy.thr_lon, 80.0, 6.5);
        assert!(needed(&rwy, lat, lon, 187.0, 5.0, false));
        // From well out on the approach side it is an ordinary intercept.
        let (lat, lon) = pattern_point(&rwy, 12.0, 3.0);
        assert!(!needed(&rwy, lat, lon, 187.0, 5.0, false));
    }

    #[test]
    fn side_is_the_aircrafts_own_then_the_lower_ground_then_left() {
        assert_eq!(pick_side(0.8, Some(100.0), Some(2000.0)), 1.0);
        assert_eq!(pick_side(-0.8, Some(2000.0), Some(100.0)), -1.0);
        assert_eq!(
            pick_side(0.1, Some(1400.0), Some(50.0)),
            1.0,
            "the Rock on the left"
        );
        assert_eq!(pick_side(0.1, Some(300.0), Some(250.0)), -1.0);
        assert_eq!(pick_side(0.1, None, None), -1.0);
    }

    #[test]
    fn the_residual_model_matches_the_two_finals_flown() {
        // First final: 1,880 m off at 3.3 nm, 160 kt. Arrived 207 m off.
        let first = predicted_residual_m(1880.0 / M_PER_NM, 3.3, 160.0);
        assert!((120.0..260.0).contains(&first), "{first:.0}");
        assert!(!stabilisable(1880.0 / M_PER_NM, 3.3, 160.0));
        // Second: 2,307 m at 3.09 nm. Arrived 321 m off at 0.36 nm.
        assert!(!stabilisable(2307.0 / M_PER_NM, 3.09, 160.0));
        // An ordinary jet final, 300 m off at 5 nm, is fine...
        assert!(stabilisable(300.0 / M_PER_NM, 5.0, 150.0));
        // ...and so is the light single's 370 m at 0.8 nm that the pursuit
        // is documented to fly out over the pavement.
        assert!(stabilisable(0.2, 0.8, 65.0));
    }

    #[test]
    fn an_inbound_leg_past_the_threshold_is_over() {
        let mut rev = Reversal {
            side: 1.0,
            leg: Leg::Inbound,
            platform_ft: 1938.0,
            high: false,
        };
        assert!(!overflown(&rev, 3.0));
        assert!(overflown(&rev, -0.5));
        rev.leg = Leg::Outbound;
        assert!(
            !overflown(&rev, -0.5),
            "the downwind starts beside the runway"
        );
    }

    #[test]
    fn a_pattern_begun_far_too_high_extends_its_downwind_and_descends_harder() {
        // EGSS 22, 2026-09-22: 16,700 ft at the base point, platform 1,938.
        assert!(too_high_to_turn(16_700.0, 1_938.0, 7.0, 5.0, 230.0, 2000.0));
        assert_eq!(pattern_descent_fpm(14_700.0, 2000.0), 1500.0);
        // A 172 a few hundred feet over its platform turns where it always did.
        assert!(!too_high_to_turn(2_100.0, 1_700.0, 4.0, 3.0, 90.0, 500.0));
        assert_eq!(pattern_descent_fpm(400.0, 500.0), 800.0);
        // The extension is bounded: past the cap the turn is made.
        assert!(!too_high_to_turn(
            16_700.0, 1_938.0, 25.0, 5.0, 230.0, 2000.0
        ));
        // A high downwind does not turn base at the base point...
        let rwy = egte_08();
        let r = sizing_radius_nm(200.0);
        let (lat, lon) = pattern_point(&rwy, 12.0, -OFFSET_RADII * r);
        let mut rev = Reversal {
            side: -1.0,
            leg: Leg::Outbound,
            platform_ft: 1700.0,
            high: true,
        };
        step(
            &mut rev,
            &rwy,
            lat,
            lon,
            12.0,
            -OFFSET_RADII * r,
            259.0,
            200.0,
            5.75,
            false,
        );
        assert_eq!(rev.leg, Leg::Outbound);
        // ...and turns the tick the height is in hand.
        rev.high = false;
        step(
            &mut rev,
            &rwy,
            lat,
            lon,
            12.0,
            -OFFSET_RADII * r,
            259.0,
            200.0,
            5.75,
            false,
        );
        assert_eq!(rev.leg, Leg::Base);
        assert_eq!(
            platform_vs_fpm(1700.0, 9700.0, false, 800.0, 2000.0),
            -1500.0
        );
    }

    #[test]
    fn the_platform_is_held_with_a_deadband_and_never_climbed_slow() {
        assert_eq!(platform_vs_fpm(1700.0, 1730.0, false, 800.0, 2000.0), 0.0);
        assert_eq!(platform_vs_fpm(1700.0, 1500.0, false, 800.0, 2000.0), 800.0);
        assert_eq!(platform_vs_fpm(1700.0, 1500.0, true, 800.0, 2000.0), 0.0);
        assert_eq!(
            platform_vs_fpm(1700.0, 2700.0, false, 800.0, 2000.0),
            -800.0
        );
        assert_eq!(platform_vs_fpm(1700.0, 1800.0, true, 800.0, 2000.0), -400.0);
    }

    // ---- Terrain (#182) ----

    fn interp(pts: &[(f64, f64)], x: f64) -> f64 {
        if x <= pts[0].0 {
            return pts[0].1;
        }
        for w in pts.windows(2) {
            let ((x0, y0), (x1, y1)) = (w[0], w[1]);
            if x <= x1 {
                return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
            }
        }
        pts[pts.len() - 1].1
    }

    /// LEMG 13's ground on the runway's axes (along, lateral right of the
    /// inbound course, nm), from sortie 1790668215-full_auto's `state`
    /// records (alt less agl) and its `reversal` records. The centreline as
    /// the 737 flew it in (t+1422 to t+1585). Left (north east, the hills
    /// behind Malaga): the drawn downwind 3.9 nm out read 1,644 ft; flown 0.4
    /// to 0.6 nm further out it crossed 1,966 and 2,043 ft at 8.5 and 8.7 nm
    /// (t+1791 to t+1795, 522 ft under the 2,644 ft platform). Right (the
    /// Guadalhorce valley): 1,653 at most, the old probe's number.
    fn lemg13_ground(along: f64, lat: f64) -> f64 {
        if lat.abs() < 0.6 {
            return interp(
                &[
                    (1.0, 60.0),
                    (2.1, 72.0),
                    (3.6, 137.0),
                    (5.15, 168.0),
                    (5.9, 265.0),
                    (6.3, 327.0),
                    (6.7, 478.0),
                    (7.15, 611.0),
                    (7.6, 817.0),
                    (8.0, 959.0),
                    (8.8, 1110.0),
                    (9.7, 885.0),
                    (10.6, 760.0),
                    (11.1, 1063.0),
                    (12.0, 900.0),
                ],
                along,
            );
        }
        if lat < 0.0 {
            if (6.5..=10.5).contains(&along) && lat <= -4.3 {
                return 2043.0;
            }
            if (6.5..=10.5).contains(&along) && lat <= -3.0 {
                return 1644.0;
            }
            return if along > 0.0 { 900.0 } else { 100.0 };
        }
        if (3.0..=10.0).contains(&along) && lat >= 3.0 {
            1653.0
        } else {
            300.0
        }
    }

    /// The loop of [`plan`] with the ground from `g` instead of the sim.
    fn plan_over(
        g: impl Fn(f64, f64) -> f64,
        along_now: f64,
        xtk_now: f64,
        gs: f64,
        elev: f64,
        default_platform: f64,
    ) -> (Judgement, f64, u32) {
        let under = |pts: &[ProbePt]| -> Vec<f64> {
            pts.iter().map(|p| g(p.along_nm, p.lateral_nm)).collect()
        };
        let mut built_for = default_platform;
        let mut rounds = 0;
        loop {
            rounds += 1;
            let ip = inbound_frame(built_for, elev, gs);
            let facts = |side: f64| {
                let pts: Vec<ProbePt> = side_frame(side, built_for, elev, gs, along_now)
                    .into_iter()
                    .chain(ip.iter().copied())
                    .collect();
                side_facts(&pts, &under(&pts))
            };
            let (l, r) = (facts(-1.0), facts(1.0));
            let side = pick_side(xtk_now, l.line_max_ft, r.line_max_ft);
            let j = judge(side, &l, &r, elev, default_platform);
            match reprobe_at(built_for, &j) {
                Some(p) if rounds < MAX_PROBE_ROUNDS => built_for = p,
                _ => {
                    let flown = built_for.max(j.platform_ft);
                    return (j, flown, rounds);
                }
            }
        }
    }

    /// The least height over the ground of every level point of a side's
    /// pattern flown at `platform_ft`, corridor included.
    fn least_over(
        g: impl Fn(f64, f64) -> f64,
        side: f64,
        platform_ft: f64,
        gs: f64,
        along_now: f64,
    ) -> f64 {
        side_frame(side, platform_ft, 48.0, gs, along_now)
            .into_iter()
            .chain(inbound_frame(platform_ft, 48.0, gs))
            .filter(|p| p.level)
            .map(|p| platform_ft - g(p.along_nm, p.lateral_nm))
            .fold(f64::INFINITY, f64::min)
    }

    /// #182, the recorded LEMG 13 reversal at t+1589.3 (sortie
    /// 1790668215-full_auto): 1.88 nm out on the centreline after a go-around,
    /// 170 kt. The old rule flew the left-hand downwind at 2,644 ft (the drawn
    /// line's 1,644 plus 1,000) and passed 522 ft over the hill outside it.
    /// Judged on the corridor, the left needs 3,043 and the right 2,653: the
    /// right is flown, 1,000 ft over everything under its level legs.
    #[test]
    fn lemg_13_reversal_is_flown_over_its_corridor_not_its_line() {
        let (gs, along, xtk) = (170.4, 1.884, 0.0077);
        // The old rule's number, and what its corridor held.
        assert!(least_over(lemg13_ground, -1.0, 2644.0, gs, along) < 700.0);
        let (j, flown, rounds) = plan_over(lemg13_ground, along, xtk, gs, 48.0, 48.0 + 1590.0);
        assert!(j.refused.is_none(), "{:?}", j.refused);
        assert_eq!(j.side, 1.0, "the lower side, raised: {j:?}");
        assert_eq!(j.why, "raised_for_terrain");
        assert!((flown - 2653.0).abs() < 1.0, "{flown}");
        assert!(rounds >= 2, "the raise moved the base out: probed again");
        let least = least_over(lemg13_ground, j.side, flown, gs, along);
        assert!(least >= 999.0, "1,000 ft over the level legs: {least}");
        // The descent from the platform keeps its floor all the way in (500
        // ft, tapering inside 2 nm: 306 ft at the 1 nm sample), on the
        // standard path.
        let check = if j.pick.left {
            &j.pick.descent_left
        } else {
            &j.pick.descent_right
        };
        assert!(
            check.problem.is_none() && !j.pick.shape.steep(),
            "{check:?}"
        );
        assert!(check.default_min_ft.unwrap() > 300.0, "{check:?}");
    }

    /// Over level ground the pattern is the one it always was: the standard
    /// left-hand side off the centreline, at the default platform, probed
    /// once.
    #[test]
    fn a_level_field_keeps_the_old_pattern() {
        let flat = |_: f64, _: f64| 20.0;
        let (j, flown, rounds) = plan_over(flat, -1.0, 0.0, 160.0, 48.0, 1638.0);
        assert_eq!((j.side, j.why, rounds), (-1.0, "default_side_clear", 1));
        assert!((flown - 1638.0).abs() < 1.0);
        assert!(j.refused.is_none());
        // Beside the runway, its own side stands.
        let (j, _, _) = plan_over(flat, -1.0, 1.2, 160.0, 48.0, 1638.0);
        assert_eq!(j.side, 1.0);
    }

    /// The corridor is the autopilot's, not the line: a hill half a radius
    /// outside the drawn downwind fouls the side the line alone called clear.
    #[test]
    fn a_hill_outside_the_drawn_downwind_switches_sides() {
        let gs = 170.0;
        let offset = OFFSET_RADII * sizing_radius_nm(gs);
        let hill = move |along: f64, lat: f64| {
            if lat < -(offset + 0.3) && (3.0..=8.0).contains(&along) {
                1200.0
            } else {
                20.0
            }
        };
        let line = side_frame(-1.0, 1638.0, 48.0, gs, -1.0);
        assert!(line
            .iter()
            .filter(|p| p.line)
            .all(|p| hill(p.along_nm, p.lateral_nm) < 100.0));
        let (j, flown, _) = plan_over(hill, -1.0, 0.0, gs, 48.0, 1638.0);
        assert_eq!((j.side, j.why), (1.0, "switched_side_for_terrain"));
        assert!((flown - 1638.0).abs() < 1.0);
    }

    /// A final in the hills is no pattern to fly: refused with the numbers,
    /// never steepened (the loop flies autoland's own path).
    #[test]
    fn a_final_the_path_cannot_clear_is_refused_with_the_numbers() {
        let ridge = |h: f64| {
            move |along: f64, lat: f64| {
                if lat.abs() < 0.6 && (3.8..=4.2).contains(&along) {
                    h
                } else {
                    20.0
                }
            }
        };
        let (j, _, _) = plan_over(ridge(1500.0), -1.0, 0.0, 160.0, 48.0, 1638.0);
        let why = j.refused.expect("refused");
        assert!(
            why.contains("the final would pass 0 feet over 1500 feet of ground 4.0 miles out"),
            "{why}"
        );
        assert!(
            !why.contains('\u{2014}') && !why.contains('\u{2013}'),
            "no dashes: {why}"
        );
        // A lower ridge a steeper, legal path would clear is still refused:
        // autoland flies its approach's own path.
        let (j, _, _) = plan_over(ridge(900.0), -1.0, 0.0, 160.0, 48.0, 1638.0);
        let why = j.refused.expect("refused, never steepened");
        assert!(why.contains("degree path"), "{why}");
    }

    /// Mountains all round: a platform whose join point is past 15 nm is
    /// refused.
    #[test]
    fn a_pattern_raised_past_its_join_limit_is_refused() {
        let high = |_: f64, lat: f64| if lat.abs() > 1.0 { 4200.0 } else { 20.0 };
        let (j, _, _) = plan_over(high, -1.0, 0.0, 160.0, 48.0, 1638.0);
        let why = j.refused.expect("refused");
        assert!(
            why.contains("5200 feet") && why.contains("miles out"),
            "{why}"
        );
    }

    /// A refusal hands over only once level at its altitude, the autopilot
    /// holding it; a hold waiting on the plan never does.
    #[test]
    fn a_refusal_hands_over_only_once_level_at_its_altitude() {
        let mut h = TerrainHold::refused(133.0, 3100.0);
        for _ in 0..20 {
            assert!(!h.level_now(2600.0), "climbing: still ours");
        }
        for _ in 0..HOLD_LEVEL_TICKS - 1 {
            assert!(!h.level_now(3080.0));
        }
        assert!(h.level_now(3060.0));
        let mut w = TerrainHold::waiting(133.0, 1700.0);
        for _ in 0..50 {
            assert!(!w.level_now(1700.0));
        }
    }

    /// The refusal's hold: 1,000 ft over the ground ahead, never lower than
    /// the aircraft or the platform, in hundreds.
    #[test]
    fn the_refusals_hold_clears_the_ground_ahead() {
        assert_eq!(
            hold_altitude_ft(Some(2043.0), Some(900.0), 2500.0, 1638.0),
            3100.0
        );
        assert_eq!(hold_altitude_ft(Some(100.0), None, 2500.0, 1638.0), 2500.0);
        assert_eq!(hold_altitude_ft(None, Some(2200.0), 800.0, 1638.0), 3200.0);
        assert_eq!(hold_altitude_ft(None, None, 800.0, 1638.0), 1700.0);
    }

    // --- Progress (2026-09-30) -------------------------------------------

    /// LEMG 13, iMac 2026-09-30 (sortie 1790775809-approach, SR22): the
    /// pattern as recorded, every 15 s from the `reversal` record (t+29.5,
    /// 104 kt, right-hand, platform 2,895 ft, field 48 ft) to the flat
    /// clock's give-up at t+613: (t, leg, along nm, xtk m, alt ft).
    const LEMG13_SR22: &[(f64, char, f64, f64, f64)] = &[
        (29.5, 'O', 1.06, 7.0, 915.0),
        (44.6, 'O', 0.73, 171.0, 998.0),
        (59.7, 'O', 0.57, 716.0, 1225.0),
        (75.0, 'O', 0.68, 1388.0, 1453.0),
        (90.2, 'O', 1.01, 1840.0, 1656.0),
        (105.3, 'O', 1.38, 2047.0, 1833.0),
        (120.5, 'O', 1.76, 2095.0, 2041.0),
        (135.7, 'O', 2.15, 2161.0, 2243.0),
        (150.8, 'O', 2.53, 2209.0, 2428.0),
        (166.0, 'O', 2.90, 2211.0, 2617.0),
        (181.2, 'O', 3.29, 2216.0, 2803.0),
        (196.3, 'O', 3.68, 2280.0, 2852.0),
        (211.7, 'O', 4.05, 2325.0, 2847.0),
        (226.8, 'O', 4.42, 2280.0, 2854.0),
        (242.0, 'O', 4.80, 2285.0, 2859.0),
        (257.3, 'O', 5.19, 2301.0, 2851.0),
        (272.5, 'O', 5.57, 2274.0, 2850.0),
        (287.7, 'O', 5.95, 2250.0, 2855.0),
        (302.9, 'O', 6.38, 2264.0, 2860.0),
        (318.0, 'O', 6.78, 2293.0, 2854.0),
        (333.2, 'O', 7.17, 2285.0, 2849.0),
        (348.3, 'O', 7.57, 2250.0, 2851.0),
        (363.5, 'O', 7.97, 2238.0, 2856.0),
        (378.7, 'O', 8.37, 2254.0, 2860.0),
        (393.9, 'O', 8.75, 2284.0, 2855.0),
        (409.0, 'O', 9.13, 2286.0, 2852.0),
        (424.2, 'O', 9.50, 2264.0, 2849.0),
        (439.3, 'O', 9.88, 2232.0, 2853.0),
        (454.5, 'O', 10.25, 2228.0, 2857.0),
        (469.7, 'O', 10.65, 2243.0, 2860.0),
        (484.9, 'B', 11.06, 2207.0, 2850.0),
        (500.1, 'B', 11.33, 1755.0, 2847.0),
        (515.4, 'B', 11.40, 1055.0, 2857.0),
        (530.8, 'I', 11.36, 308.0, 2870.0),
        (545.8, 'I', 11.06, -270.0, 2901.0),
        (560.9, 'I', 10.59, -344.0, 2895.0),
        (576.1, 'I', 10.13, -202.0, 2893.0),
        (591.3, 'I', 9.66, -98.0, 2898.0),
        (606.6, 'I', 9.21, -45.0, 2888.0),
        (613.1, 'I', 9.04, -40.0, 2884.0),
    ];
    const LEMG13_PLATFORM_FT: f64 = 2895.1;
    const LEMG13_ELEV_FT: f64 = 48.0;
    const LEMG13_GS_KTS: f64 = 104.1;

    fn leg_of(c: char) -> Leg {
        match c {
            'O' => Leg::Outbound,
            'B' => Leg::Base,
            _ => Leg::Inbound,
        }
    }

    /// The loop's `Progress::Pattern` for one position.
    fn pattern_progress(
        rev: &Reversal,
        along: f64,
        xtk_nm: f64,
        alt: f64,
        gs: f64,
        elev: f64,
        n: u32,
    ) -> Progress {
        let join = crate::autoland::join_point_nm(rev.platform_ft, elev);
        let end = pattern_end_nm(rev.platform_ft, elev, crate::autoland::gp_ft_per_nm(), join);
        let to_go = to_go_nm(rev, along, xtk_nm, gs, join, end);
        let above = alt - rev.platform_ft;
        Progress::Pattern {
            n,
            leg: rev.leg,
            high: rev.high,
            to_go_nm: to_go,
            above_ft: above,
            budget_s: pattern_budget_s(to_go, gs, above, 1500.0),
            gs_kts: gs,
        }
    }

    /// The record replayed at 1 s ticks (interpolated between its samples),
    /// after the go-around's climb from 734 ft: the pattern made its ground
    /// all the way and is never given up; its budget from the plan covers
    /// the 584 s it flew and then some. The flat 600 s gave it up at t+613.
    #[test]
    fn lemg_13_sr22_pattern_makes_its_ground_and_is_never_given_up() {
        let mut w = MissedWatch::start(12.7);
        // The go-around's climb, 734 ft radio to the turn at 800.
        for (i, t) in (13..29).enumerate() {
            let radio = 734.0 + i as f64 * 10.0;
            w.note(t as f64, Progress::Climb { to_go_ft: 800.0 - radio });
            assert_eq!(w.verdict(t as f64), None);
        }
        let mut rev = Reversal {
            side: 1.0,
            leg: Leg::Outbound,
            platform_ft: LEMG13_PLATFORM_FT,
            high: false,
        };
        let first = LEMG13_SR22[0];
        let p0 = pattern_progress(
            &rev,
            first.2,
            first.3 / M_PER_NM,
            first.4,
            LEMG13_GS_KTS,
            LEMG13_ELEV_FT,
            1,
        );
        let Progress::Pattern { to_go_nm, budget_s, .. } = p0 else { unreachable!() };
        // Planned: 10.2 nm of downwind to the base, back in to the path at
        // 9.1 nm, 1.5 nm out and back across: about 15 nm.
        assert!((14.0..16.5).contains(&to_go_nm), "to go {to_go_nm:.1} nm");
        assert!(budget_s > 613.1 - 29.5 + 120.0, "budget {budget_s:.0} s");
        assert!(budget_s < 1500.0, "budget {budget_s:.0} s");
        assert_eq!(w.note(29.5, p0), Some("pattern"));
        let mut worst_stall: f64 = 0.0;
        for pair in LEMG13_SR22.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            let mut t = a.0.ceil();
            while t < b.0 {
                let f = (t - a.0) / (b.0 - a.0);
                let lerp = |x: f64, y: f64| x + (y - x) * f;
                rev.leg = leg_of(a.1);
                let p = pattern_progress(
                    &rev,
                    lerp(a.2, b.2),
                    lerp(a.3, b.3) / M_PER_NM,
                    lerp(a.4, b.4),
                    LEMG13_GS_KTS,
                    LEMG13_ELEV_FT,
                    1,
                );
                w.note(t, p);
                assert_eq!(w.verdict(t), None, "given up at t+{t:.0}");
                worst_stall = worst_stall.max(w.stalled_s(t));
                t += 1.0;
            }
        }
        assert!(worst_stall < PATTERN_STALL_S * 0.75, "stood {worst_stall:.0} s");
        assert!(w.budget_left_s(613.1) > 120.0);
    }

    /// The same SR22 circling where the downwind began (a lost pattern): its
    /// track to go swings with the orbit and never steps, so it is given up
    /// about two minutes after the last ground it made, not at ten.
    #[test]
    fn a_pattern_that_goes_nowhere_is_given_up_in_minutes() {
        let rev = Reversal {
            side: 1.0,
            leg: Leg::Outbound,
            platform_ft: LEMG13_PLATFORM_FT,
            high: false,
        };
        let mut w = MissedWatch::start(0.0);
        let mut gave_up = None;
        for i in 0..900 {
            let t = i as f64;
            // A 0.4 nm orbit round 2 nm out, 1.2 nm right, every 100 s.
            let a = t / 100.0 * std::f64::consts::TAU;
            let p = pattern_progress(
                &rev,
                2.0 + 0.4 * a.cos(),
                1.2 + 0.4 * a.sin(),
                2890.0,
                LEMG13_GS_KTS,
                LEMG13_ELEV_FT,
                1,
            );
            w.note(t, p);
            if let Some(v) = w.verdict(t) {
                gave_up = Some((t, v));
                break;
            }
        }
        let (t, v) = gave_up.expect("an orbit is never progress");
        assert_eq!(v, Verdict::Stalled);
        assert!(t < 250.0, "given up at {t:.0} s");
    }

    /// A jet flies its pattern on the kinematic autopilot (737, 160 kt,
    /// 24 degrees, EGTE 08 at 3,000 ft from a go-around 1.5 nm past the far
    /// end): never stalled, done inside its own budget. The same jet in an
    /// orbit is given up within minutes: a fast aircraft is not given ten of
    /// them to loiter in.
    #[test]
    fn a_jet_pattern_is_judged_on_its_own_size_and_speed() {
        let rwy = egte_08();
        let (lat, lon) = project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, 1.5);
        let gs = 160.0;
        let platform = 3000.0;
        let join = crate::autoland::join_point_nm(platform, rwy.elev_ft);
        for orbit in [false, true] {
            let mut p = Plant {
                lat,
                lon,
                hdg: rwy.course_deg,
                bank: 0.0,
                gs_kts: gs,
                max_bank: 24.0,
            };
            let mut rev = Reversal {
                side: -1.0,
                leg: Leg::Outbound,
                platform_ft: platform,
                high: false,
            };
            let mut w = MissedWatch::start(0.0);
            let mut t = 0.0;
            let mut first_budget = None;
            let outcome = loop {
                let along = along_track_nm(p.lat, p.lon, &rwy);
                let xtk = cross_track_nm(p.lat, p.lon, &rwy);
                let final_called = along < 5.5 && crate::autoland::stabilised_xtk(xtk, along);
                let bug = step(
                    &mut rev, &rwy, p.lat, p.lon, along, xtk, p.hdg, gs, join, final_called,
                );
                let Some(bug) = bug else { break Ok(t) };
                let prog = pattern_progress(&rev, along, xtk, platform, gs, rwy.elev_ft, 1);
                if let Progress::Pattern { budget_s, .. } = prog {
                    first_budget.get_or_insert(budget_s);
                }
                w.note(t, prog);
                if let Some(v) = w.verdict(t) {
                    break Err((t, v));
                }
                // A lost pattern: the heading bug always 90 degrees round.
                let bug = if orbit && t > 60.0 { p.hdg + 90.0 } else { bug };
                p.fly(bug, 0.5);
                t += 0.5;
                assert!(t < 3000.0, "never ended");
            };
            let budget = first_budget.unwrap();
            if orbit {
                let (at, v) = outcome.expect_err("an orbit is given up");
                assert_eq!(v, Verdict::Stalled);
                assert!(at < 400.0, "given up at {at:.0} s");
            } else {
                let done = outcome.unwrap_or_else(|(at, v)| panic!("{v:?} at {at:.0} s"));
                assert!(done < budget, "flown in {done:.0} s against {budget:.0}");
                // Sized by the plan: a jet's bigger pattern at twice the
                // speed still earns a bound of its own, not a flat ten minutes.
                assert!((300.0..1200.0).contains(&budget), "budget {budget:.0} s");
            }
        }
    }

    /// An extended downwind makes its progress in height: descending, it is
    /// never stalled though its track to go grows; held level above the
    /// platform, it is. Flipping in and out of it every tick is not progress.
    #[test]
    fn an_extended_downwind_progresses_by_the_height_it_loses() {
        let pat = |high: bool, to_go: f64, above: f64| Progress::Pattern {
            n: 1,
            leg: Leg::Outbound,
            high,
            to_go_nm: to_go,
            above_ft: above,
            budget_s: 3000.0,
            gs_kts: 200.0,
        };
        let mut w = MissedWatch::start(0.0);
        for i in 0..600 {
            let t = i as f64;
            // 1,200 fpm down from 8,000 ft above, the track growing.
            w.note(t, pat(true, 20.0 + t * 0.05, 8000.0 - 20.0 * t));
            assert_eq!(w.verdict(t), None, "stalled descending at {t} s");
        }
        let mut w = MissedWatch::start(0.0);
        let stalled = (0..600).map(|i| i as f64).find(|&t| {
            w.note(t, pat(true, 20.0 + t * 0.05, 8000.0));
            w.verdict(t).is_some()
        });
        assert!(stalled.is_some_and(|t| t <= PATTERN_STALL_S + 1.0), "{stalled:?}");
        let mut w = MissedWatch::start(0.0);
        let flipping = (0..600).map(|i| i as f64).find(|&t| {
            let high = (t as u32) % 2 == 0;
            w.note(t, pat(high, 20.0 - t * 0.001, 8000.0 - t));
            w.verdict(t).is_some()
        });
        assert!(flipping.is_some_and(|t| t <= PATTERN_STALL_S + 1.0), "{flipping:?}");
    }

    /// A climb that stops (the go-around's, or a refused pattern's hold) is
    /// given up at 90 s; a pause is not a stall; the plan not coming is.
    #[test]
    fn a_climb_that_stops_or_a_plan_that_never_comes_is_given_up() {
        let mut w = MissedWatch::start(0.0);
        w.note(0.0, Progress::Hold { to_go_ft: 2000.0 });
        for t in 1..=90 {
            w.note(t as f64, Progress::Hold { to_go_ft: 2000.0 - 10.0 * t as f64 });
            assert_eq!(w.verdict(t as f64), None);
        }
        // Stopped 1,100 ft short.
        let stop = (91..400).map(|t| t as f64).find(|&t| {
            w.note(t, Progress::Hold { to_go_ft: 1100.0 });
            w.verdict(t).is_some()
        });
        assert!(stop.is_some_and(|t| t > 150.0 && t < 200.0), "{stop:?}");
        // A pause of five minutes mid-climb holds the clocks.
        let mut w = MissedWatch::start(0.0);
        w.note(0.0, Progress::Climb { to_go_ft: 500.0 });
        w.note(10.0, Progress::Climb { to_go_ft: 300.0 });
        w.note(310.0, Progress::Climb { to_go_ft: 300.0 });
        assert_eq!(w.verdict(310.0), None);
        // The plan never answering.
        let mut w = MissedWatch::start(0.0);
        let waited = (0..400).map(|t| t as f64).find(|&t| {
            w.note(t, Progress::Waiting);
            w.verdict(t).is_some()
        });
        assert!(waited.is_some_and(|t| t > PLAN_WAIT_S && t < PLAN_WAIT_S + 2.0), "{waited:?}");
    }

    // --- No-go airspace (LEGR-LXGB 09, 2026-10-06) ---------------------------

    /// LXGB 09 from apt.dat: the 09 threshold and the 27 threshold.
    fn lxgb_09() -> Runway {
        let (lat, lon) = (36.1508156, -5.3596312);
        Runway {
            icao: "LXGB".into(),
            id: "09".into(),
            thr_lat: lat,
            thr_lon: lon,
            course_deg: bearing_deg(lat, lon, 36.1506, -5.3336),
            length_m: 1680.0,
            surface: 1,
            dist_nm: 0.0,
            elev_ft: 15.0,
        }
    }

    /// R-164 ALGECIRAS (CADIZ), GND to 30,000 ft, from the pilot's
    /// `Custom Data/airspaces/airspace.txt`: its vertices, the arcs round
    /// Algeciras and Tarifa thinned to their ends.
    fn r164() -> crate::airspace::Airspace {
        crate::airspace::Airspace {
            class: 'R',
            name: "R-164        (ALGECIRAS (CADIZ))".into(),
            floor_ft: 0.0,
            ceiling_ft: 30_000.0,
            poly: vec![
                (36.4433, -5.2719),
                (36.4164, -5.1544),
                (36.3742, -5.1183),
                (36.1814, -5.2694),
                (36.1725, -5.2722),
                (36.1489, -5.2764),
                (36.1536, -5.3375),
                (36.1528, -5.3389),
                (36.1550, -5.3453),
                (36.1547, -5.3519),
                (36.1558, -5.3506),
                (36.1558, -5.4019),
                (36.0944, -5.3842),
                (36.0792, -5.3842),
                (36.0406, -5.4092),
                (36.0286, -5.4256),
                (36.0203, -5.4458),
                (36.0167, -5.4650),
                (36.0175, -5.4747),
                (35.9833, -5.5608),
                (35.9667, -5.6089),
                (35.9994, -5.6517),
                (36.0225, -5.6719),
                (36.0239, -5.6900),
                (36.0558, -5.6647),
                (36.2056, -5.6583),
            ],
        }
    }

    /// The pattern the sortie flew (12 nm east of the field, 13,283 ft, a
    /// 3,210 ft platform, 212 kt) met R-164 on BOTH sides: the right-hand
    /// downwind south of the Rock enters it over the bay, 1.5 to 2 nm past
    /// the threshold, and the left one over La Linea. The downwind may be
    /// extended only to a radius and a half mile short of it, and the hold
    /// there (over the Strait south of Europa Point) is clear of it.
    #[test]
    fn lxgb_09_downwind_stops_short_of_r164_and_holds_over_the_strait() {
        let rwy = lxgb_09();
        let sp = vec![r164()];
        let (gs, platform, alt, along_now) = (212.0, 3_210.0, 13_283.0, -11.98);
        // Both sides' level legs are in it: no side is clear.
        let l = side_frame(-1.0, platform, rwy.elev_ft, gs, along_now);
        let r = side_frame(1.0, platform, rwy.elev_ft, gs, along_now);
        let r164_name = Some("restricted area R 164");
        assert_eq!(side_airspace(&rwy, &l, platform, &sp).as_deref(), r164_name);
        assert_eq!(side_airspace(&rwy, &r, platform, &sp).as_deref(), r164_name);
        // The flown base 12 nm out (5.61 W) is deep inside it.
        let (blat, blon) = pattern_point(&rwy, 12.0, 4.9);
        assert!(sp[0].contains(blat, blon));
        // The right-hand downwind meets it over the bay.
        let rr = sizing_radius_nm(gs);
        let offset = OFFSET_RADII * rr;
        let (conflict, name) = downwind_conflict(&rwy, 1.0, offset, along_now, platform, alt, &sp)
            .expect("R-164 ahead");
        assert_eq!(name, "restricted area R 164");
        assert!((1.0..2.5).contains(&conflict), "{conflict}");
        let to = extend_to_nm(conflict, rr, along_now);
        assert!(to < conflict - rr, "{to}");
        // The racetrack short of it is clear of it, at both heights.
        for p in hold_frame(1.0, offset, rr, to) {
            let (la, lo) = pattern_point(&rwy, p.along_nm, p.lateral_nm);
            assert!(
                !sp[0].blocks(la, lo, platform) && !sp[0].blocks(la, lo, alt),
                "{p:?}"
            );
        }
        // No volume: no limit, today's rule.
        assert!(downwind_conflict(&rwy, 1.0, offset, along_now, platform, alt, &[]).is_none());
    }

    /// The rule per tick: not high, nothing; high with a hold, out to the
    /// limit, back along the outer leg for HOLD_LEG_NM, out again; high
    /// without one, the downwind turns in at the limit.
    #[test]
    fn a_high_downwind_holds_at_the_airspace_limit_and_never_extends_past_it() {
        let held = AirspaceLimit {
            extend_to_nm: -0.6,
            conflict_nm: 1.6,
            name: "restricted area R 164".into(),
            hold: true,
        };
        let mut back = false;
        // Not high: the limit is nothing.
        let s = limit_step(false, 0.0, Some(&held), &mut back);
        let quiet = LimitStep {
            high: false,
            back: false,
            capped: false,
        };
        assert_eq!(s, quiet);
        // High, short of the limit: the downwind is flown.
        let s = limit_step(true, -3.0, Some(&held), &mut back);
        assert_eq!(
            s,
            LimitStep {
                high: true,
                ..quiet
            }
        );
        // At it: the hold's way back.
        let s = limit_step(true, -0.6, Some(&held), &mut back);
        assert!(s.high && s.back);
        // Still back until HOLD_LEG_NM short of it.
        assert!(limit_step(true, -2.0, Some(&held), &mut back).back);
        assert!(!limit_step(true, -0.6 - HOLD_LEG_NM, Some(&held), &mut back).back);
        // Out again on the downwind, and back once more at the limit.
        let s = limit_step(true, -1.0, Some(&held), &mut back);
        assert!(!s.back);
        let _ = limit_step(true, -0.5, Some(&held), &mut back);
        assert!(back);
        // Height lost: the pattern goes on, the hold forgotten.
        let s = limit_step(false, -0.8, Some(&held), &mut back);
        assert_eq!(s, quiet);
        // No legal hold: turned in at the limit, high.
        let capped = AirspaceLimit {
            hold: false,
            ..held.clone()
        };
        assert!(limit_step(true, -1.0, Some(&capped), &mut back).high);
        let s = limit_step(true, -0.6, Some(&capped), &mut back);
        assert_eq!(
            s,
            LimitStep {
                capped: true,
                ..quiet
            }
        );
        // No limit at all: today's extension.
        let s = limit_step(true, 20.0, None, &mut back);
        assert_eq!(
            s,
            LimitStep {
                high: true,
                ..quiet
            }
        );
    }

    /// Flown: the LXGB 09 sortie's start, 12 nm east at 13,283 ft over a
    /// 3,210 ft platform, but with the limit. The pattern loses the height
    /// round Europa Point and in the hold over the Strait, and is never in
    /// R-164 while it is high; then it goes on to its base.
    #[test]
    fn the_lxgb_09_pattern_loses_its_height_clear_of_r164() {
        let rwy = lxgb_09();
        let sp = vec![r164()];
        let gs = 212.0;
        let platform = 3_210.0;
        let rr = sizing_radius_nm(gs);
        let offset = OFFSET_RADII * rr;
        let along0 = -11.98;
        let (conflict, name) =
            downwind_conflict(&rwy, 1.0, offset, along0, platform, 13_283.0, &sp).unwrap();
        let limit = AirspaceLimit {
            extend_to_nm: extend_to_nm(conflict, rr, along0),
            conflict_nm: conflict,
            name,
            hold: true,
        };
        let recip = (rwy.course_deg + 180.0) % 360.0;
        let (lat, lon) = pattern_point(&rwy, along0, 0.35);
        let mut p = Plant {
            lat,
            lon,
            hdg: 281.0,
            bank: 0.0,
            gs_kts: gs,
            max_bank: 25.0,
        };
        let mut rev = Reversal {
            side: 1.0,
            leg: Leg::Outbound,
            platform_ft: platform,
            high: false,
        };
        let join = crate::autoland::join_point_nm(platform, rwy.elev_ft);
        let (mut alt, mut back, mut holds, mut was_back) = (13_283.0, false, 0, false);
        let dt = 1.0;
        let mut base_at = None;
        for _ in 0..6000 {
            let along = along_track_nm(p.lat, p.lon, &rwy);
            let xtk = cross_track_nm(p.lat, p.lon, &rwy);
            let too_high = rev.leg == Leg::Outbound
                && too_high_to_turn(alt, platform, along, join, gs, 2_000.0);
            let lim = limit_step(too_high, along, Some(&limit), &mut back);
            rev.high = lim.high;
            if lim.high {
                assert!(
                    !sp[0].blocks(p.lat, p.lon, alt),
                    "high in R-164 at {along:.2} nm, {alt:.0} ft"
                );
            }
            if lim.back && !was_back {
                holds += 1;
            }
            was_back = lim.back;
            let h = step(
                &mut rev, &rwy, p.lat, p.lon, along, xtk, p.hdg, gs, join, false,
            );
            let bug = if lim.back {
                hold_heading(&rwy, rev.side, p.lat, p.lon, along, gs)
            } else {
                h.unwrap_or(recip)
            };
            p.fly(bug, dt);
            let vs = platform_vs_fpm(platform, alt, false, 1_500.0, 2_000.0);
            alt += vs * dt / 60.0;
            if rev.leg != Leg::Outbound {
                base_at = Some((along, alt));
                break;
            }
        }
        let (along_base, alt_base) = base_at.expect("the pattern reached its base");
        assert!(holds >= 1, "the height was lost in a hold");
        assert!(
            (alt_base - platform).abs() < 300.0,
            "at the platform by the base: {alt_base:.0}"
        );
        assert!(
            along_base > conflict,
            "the pattern goes on to its base: {along_base:.1}"
        );
    }

    #[test]
    fn the_reposition_names_the_airspace_it_cannot_avoid() {
        let line = reposition_line(
            "zero nine",
            1.0,
            3_210.0,
            "airspace_unavoidable",
            true,
            Some("restricted area R 164"),
        );
        assert_eq!(
            line,
            "Repositioning for runway zero nine: right-hand downwind at 3200 feet, raised for the \
             high ground. No side is clear of restricted area R 164."
        );
        assert_eq!(
            reposition_line(
                "two seven",
                -1.0,
                1_605.0,
                "default_side_clear",
                false,
                None
            ),
            "Repositioning for runway two seven: left-hand downwind at 1600 feet."
        );
        let held = AirspaceLimit {
            extend_to_nm: -0.6,
            conflict_nm: 1.6,
            name: "restricted area R 164".into(),
            hold: true,
        };
        assert_eq!(
            extend_line(2_000.0, Some(&held)),
            "We're 2000 feet above the pattern. Holding clear of restricted area R 164 to lose it."
        );
        assert_eq!(
            extend_line(2_000.0, None),
            "We're 2000 feet above the pattern. Extending the downwind to lose it."
        );
        for l in [line, capped_line(900.0, "restricted area R 164")] {
            assert!(!l.contains('\u{2014}') && !l.contains('\u{2013}'), "{l}");
        }
    }
}
