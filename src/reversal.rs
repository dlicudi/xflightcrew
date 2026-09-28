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
//! All pure; the autoland loop owns the one `Reversal` and applies it.

use crate::autoland::Runway;
use crate::nav::{bearing_deg, project_latlon};

/// Signed smallest turn from heading `b` to heading `a`, degrees (-180..180,
/// positive clockwise).
fn heading_diff(a: f64, b: f64) -> f64 {
    (a - b + 540.0).rem_euclid(360.0) - 180.0
}

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
const BASE_BEYOND_JOIN_NM: f64 = 0.5;
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
        return heading_diff(track_deg, rwy.course_deg).abs() > FINAL_MAX_TRACK_OFF_DEG;
    }
    let recip = (rwy.course_deg + 180.0) % 360.0;
    let (alat, alon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, aim_along_nm);
    let arrival = bearing_deg(lat, lon, alat, alon);
    heading_diff(arrival, rwy.course_deg).abs() > ARRIVAL_MAX_OFF_DEG
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
        && heading_diff(track_deg, course_deg).abs() <= PATH_JOIN_TRACK_DEG
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
            let off = heading_diff(brg, recip).clamp(-45.0, 45.0);
            Some((recip + off).rem_euclid(360.0))
        }
        Leg::Base => Some(toward_line),
        Leg::Inbound => {
            if final_called
                && heading_diff(track_deg, rwy.course_deg).abs() <= FINAL_MAX_TRACK_OFF_DEG
            {
                return None;
            }
            // The final's own law, from beyond the FAP: fixed lead, cut
            // capped at 35 degrees, exponential onto the line.
            let aim = crate::autoland::pursuit_aim_nm(along_nm, gs_kts, rwy.length_m);
            let (alat, alon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, aim);
            let brg = bearing_deg(lat, lon, alat, alon);
            let cut = heading_diff(brg, rwy.course_deg).clamp(-35.0, 35.0);
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autoland::{along_track_nm, cross_track_nm};

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
            let err = heading_diff(bug, self.hdg);
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
        let mut hold = crate::circuit::BugHold::default();
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
                    let cut = heading_diff(bearing_deg(p.lat, p.lon, alat, alon), rwy.course_deg)
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
            if knob_at.is_none_or(|a| heading_diff(bug, a).abs() >= 0.2) {
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
            heading_diff(h, 259.0).abs() < 2.0,
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
}
