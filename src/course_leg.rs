//! Course legs and the join onto a final: how the procedure follower flies a
//! CF leg ON its published course, and how it joins an approach whose IF lies
//! behind it.
//!
//! EGKK 26L, 2026-09-27 (sorties `1790510400-full_auto` and
//! `1790534902-full_auto`, issue #106): the follower flew every fix leg
//! DIRECT to its fix, a CF's course included. From WILLO, south of the
//! field, the IF (CF26L, east of it and 1 nm before the FAF) was a 026 bug
//! and a 150 degree turn onto the 256 final; FF26L then came in the same
//! tick, and autoland took the aircraft 4.6 nm out, 1,965 m off the course,
//! tracking 044, 600 ft high. Its reversal ended 3.7 km off and the
//! localizer captured at 1 nm.
//!
//! A pilot never flies a final that way, and nothing here is a new flight
//! law:
//!
//! - **A CF leg is a course intercept, then the course** ([`course_track`]):
//!   30 degrees toward the line (the standard intercept, and
//!   `circuit::MAX_FINAL_CUT_DEG`), the roll-out begun at the fly-by lead
//!   `R(1 - cos 30)` for the enroute radius, then the course with a gentle
//!   pursuit. Only when the line can be joined before the fix; else the leg
//!   is flown to its fix as before.
//! - **An IF behind us is joined by a pattern** ([`FinalJoin`]): the turn at
//!   an IF is at most 90 degrees by design, so a bigger one means the
//!   aircraft is on the far side. When the final course can be intercepted
//!   with [`FAF_ESTABLISHED_NM`] to spare, the IF is skipped for the
//!   intercept (vectors to final); otherwise autoland's own reversal pattern
//!   ([`crate::reversal`], outbound, base, inbound) is flown against the
//!   FAF's course line until the intercept fits.
//!
//! Every output is a TRACK; the follower adds the crab and sets the bug
//! through its `HeadingHold`, keyed by [`Steer::key`] so a new phase is set
//! at once and a correction inside one waits as a pilot's would (#93).
//! All pure.

use crate::autoland::{along_track_nm, cross_track_nm, Runway};
use crate::nav::ProcLeg;
use crate::reversal::{self, Reversal};

/// The intercept angle, degrees: the standard one for an instrument final,
/// inside autoland's 35 degree pursuit limit.
pub const INTERCEPT_DEG: f64 = 30.0;
/// On the course, the correction back to it aims this far ahead, nm. Longer
/// than the ground covered between two corrections the heading hold allows
/// (30 s at 210 kt is 1.75 nm), so a correction never overshoots the line.
const TRACK_AIM_NM: f64 = 2.0;
/// An intercept must be complete this far before the FAF: ICAO's minimum
/// for an intercept of the final approach track, and where autoland expects
/// to find the aircraft established.
pub const FAF_ESTABLISHED_NM: f64 = 2.0;
/// A course leg's intercept must be complete this far before its fix, nm
/// (the follower's own capture floor): else the fix is flown direct.
const FIX_MARGIN_NM: f64 = 0.8;
/// The turn at an IF above which the aircraft is on the far side of it. A
/// straight-in area allows 90 at most.
const IF_MAX_TURN_DEG: f64 = 90.0;

/// A published course through a fix, flown toward the fix.
#[derive(Debug, Clone)]
pub struct CourseLine(Runway);

impl CourseLine {
    /// The course line of a CF leg: its fix and its course, magnetic in the
    /// CIFP, made true with the variation HERE (`mag = true + var`, the
    /// X-Plane sense of `autoland::true_to_mag`). `None` for any other leg.
    pub fn of_leg(leg: &ProcLeg, mag_var: f64) -> Option<Self> {
        if leg.leg_type != "CF" {
            return None;
        }
        let fix = leg.fix.as_ref()?;
        let course = leg.course_mag?;
        Some(Self::through(
            fix.lat,
            fix.lon,
            crate::runway::mag_to_true(course, mag_var),
        ))
    }

    /// A true course through a point.
    pub fn through(lat: f64, lon: f64, course_true: f64) -> Self {
        CourseLine(Runway {
            icao: String::new(),
            id: String::new(),
            thr_lat: lat,
            thr_lon: lon,
            course_deg: course_true,
            length_m: 0.0,
            surface: 1,
            dist_nm: 0.0,
            elev_ft: 0.0,
        })
    }

    pub fn course_true(&self) -> f64 {
        self.0.course_deg
    }

    /// Distance still to fly along the course to the fix, nm (negative past it).
    pub fn along_nm(&self, lat: f64, lon: f64) -> f64 {
        along_track_nm(lat, lon, &self.0)
    }

    /// Cross-track, nm, positive right of the course.
    pub fn xtk_nm(&self, lat: f64, lon: f64) -> f64 {
        cross_track_nm(lat, lon, &self.0)
    }
}

/// How the lateral is being flown this tick. Logged as the `leg` tick's
/// `steer`, and the heading hold's key: a new phase is set at once.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Steer {
    /// Straight at the fix (every leg that is not a course, and a course that
    /// cannot be joined before its fix).
    Direct,
    Intercept,
    Track,
    Pattern(reversal::Leg),
    /// An RF leg flown round its arc (#101).
    Arc,
    /// A holding pattern's phase (#101).
    Hold(HoldPhase),
}

impl Steer {
    pub fn word(self) -> &'static str {
        match self {
            Steer::Direct => "direct",
            Steer::Intercept => "intercept",
            Steer::Track => "track",
            Steer::Pattern(reversal::Leg::Outbound) => "pattern_outbound",
            Steer::Pattern(reversal::Leg::Base) => "pattern_base",
            Steer::Pattern(reversal::Leg::Inbound) => "pattern_inbound",
            Steer::Arc => "arc",
            Steer::Hold(p) => p.word(),
        }
    }

    /// The heading hold's key for leg `idx` in this phase.
    pub fn key(self, idx: usize) -> usize {
        let k = match self {
            Steer::Direct => 0,
            Steer::Intercept => 1,
            Steer::Track => 2,
            Steer::Pattern(reversal::Leg::Outbound) => 3,
            Steer::Pattern(reversal::Leg::Base) => 4,
            Steer::Pattern(reversal::Leg::Inbound) => 5,
            Steer::Arc => 6,
            Steer::Hold(HoldPhase::ToFix) => 7,
            Steer::Hold(HoldPhase::TurnOut) => 8,
            Steer::Hold(HoldPhase::Outbound) => 9,
            Steer::Hold(HoldPhase::TurnIn) => 10,
            Steer::Hold(HoldPhase::Inbound) => 11,
        };
        idx * 16 + k
    }
}

/// The lateral distance off the course at which a 30 degree cut rolls out
/// onto it: the fly-by lead for radius `r_nm`.
fn roll_out_lead_nm(r_nm: f64) -> f64 {
    r_nm * (1.0 - INTERCEPT_DEG.to_radians().cos())
}

/// Seconds the heading mode takes to roll into a turn before it turns at
/// its radius: the bank ramps in at about 5 degrees a second.
const ROLL_LAG_S: f64 = 5.0;

/// The cross-track a 30 degree cut still closes while the heading mode rolls
/// into the roll-out, nm, added to the geometric lead. Without it the
/// intercept rolled out about 400 m through the line and the pursuit was
/// still bringing it back at the FAF (EGKK I26L, the #161 join). Pure.
fn roll_lag_lead_nm(gs_kts: f64) -> f64 {
    gs_kts.max(60.0) / 3600.0 * ROLL_LAG_S * INTERCEPT_DEG.to_radians().sin()
}

/// A turn at radius `r_nm` from `from_deg` to `to_deg`, both the track's
/// angle TOWARD a line (0 along its course, 90 straight at it), the short
/// way: the miles it runs along the course and the miles it closes on the
/// line (either negative: back, or away). Pure.
fn turn_run_close_nm(from_deg: f64, to_deg: f64, r_nm: f64) -> (f64, f64) {
    let d = crate::angles::signed_diff_deg(from_deg, to_deg);
    let (a, b) = ((to_deg + d).to_radians(), to_deg.to_radians());
    if d >= 0.0 {
        (r_nm * (a.sin() - b.sin()), r_nm * (b.cos() - a.cos()))
    } else {
        (r_nm * (b.sin() - a.sin()), r_nm * (a.cos() - b.cos()))
    }
}

/// The distance along the course an intercept from here consumes, nm: the
/// turn onto the intercept heading, the cut, the roll-out.
///
/// The turn is counted as flown: it runs `r (sin a - sin b)` along the course
/// and CLOSES `r (cos b - cos a)` on the line. Counted as `r sin(turn)` of
/// along and no closing, a base leg's 60 degree turn onto the intercept was
/// charged 1.9 radii it never used: at EGBB 15 (#161) the join skipped the IF
/// at 8.4 nm before the FAF and was established at 6.7, where 2 was asked.
pub fn intercept_room_nm(xtk_nm: f64, track_true: f64, course_true: f64, r_nm: f64) -> f64 {
    let toward = crate::angles::signed_diff_deg(track_true, course_true) * -xtk_nm.signum();
    room_from_nm(toward, xtk_nm.abs(), r_nm)
}

/// [`intercept_room_nm`] from a track `toward_deg` at the line, `off_nm`
/// beside it.
fn room_from_nm(toward_deg: f64, off_nm: f64, r_nm: f64) -> f64 {
    let (run_in, close_in) = turn_run_close_nm(toward_deg, INTERCEPT_DEG, r_nm);
    let (run_out, close_out) = turn_run_close_nm(INTERCEPT_DEG, 0.0, r_nm);
    run_in + (off_nm - close_in - close_out).max(0.0) / INTERCEPT_DEG.to_radians().tan() + run_out
}

/// The miles of final the join leaves beyond [`FAF_ESTABLISHED_NM`], for the
/// heading mode's roll in and out, which the geometry does not count.
const JOIN_SLACK_NM: f64 = 0.5;

/// Where the join's base turn begins, nm along the FAF's course line beyond
/// the FAF, from a downwind `off_nm` beside it: the turn onto base (it runs a
/// radius further out and closes one), a 30 degree intercept off the base and
/// its roll-out leave [`FAF_ESTABLISHED_NM`] of final and the slack. Sized
/// as a flat two radii beyond the established point, the base came at 5.7 nm
/// whatever the offset, and from GROVE the join took 430 s (#161). Pure.
pub fn base_turn_along_nm(off_nm: f64, r_nm: f64) -> f64 {
    let (run_base, close_base) = turn_run_close_nm(180.0, 90.0, r_nm);
    let room = room_from_nm(90.0, (off_nm - close_base).max(0.0), r_nm);
    FAF_ESTABLISHED_NM + JOIN_SLACK_NM + BASE_MARGIN_NM + room + run_base
}

/// The base is sized this much longer than the skip asks, so the skip
/// always comes off the base: sized exactly, a base turn flown a few seconds
/// slow left the intercept 0.15 nm short, the base ran on into the pattern's
/// own inbound turn and the final was crossed 900 m (the GROVE replay).
const BASE_MARGIN_NM: f64 = 0.5;

/// The room as the join flew it on 2026-09-28 (turn charged `r sin(turn)`,
/// closing nothing), for the replays of that geometry.
#[cfg(test)]
pub fn intercept_room_as_flown_nm(
    xtk_nm: f64,
    track_true: f64,
    course_true: f64,
    r_nm: f64,
) -> f64 {
    let heading = course_true - xtk_nm.signum() * INTERCEPT_DEG;
    let turn = crate::angles::abs_diff_deg(heading, track_true).min(90.0);
    let lead = roll_out_lead_nm(r_nm);
    r_nm * turn.to_radians().sin()
        + (xtk_nm.abs() - lead).max(0.0) / INTERCEPT_DEG.to_radians().tan()
        + r_nm * INTERCEPT_DEG.to_radians().sin()
}

/// The TRACK (true) that flies a course leg, and the phase: the 30 degree
/// intercept until the roll-out lead, then the course with a pursuit that
/// aims [`TRACK_AIM_NM`] ahead. `was_track` is hysteresis: once on the
/// course it stays tracking until twice the lead off. `None` when the line
/// cannot be joined before the fix: the leg is flown to its fix.
pub fn course_track(
    line: &CourseLine,
    lat: f64,
    lon: f64,
    track_true: f64,
    gs_kts: f64,
    was_track: bool,
) -> Option<(f64, Steer)> {
    let along = line.along_nm(lat, lon);
    let xtk = line.xtk_nm(lat, lon);
    let c = line.course_true();
    let r = crate::procedure::enroute_turn_radius_nm(gs_kts);
    let lead = roll_out_lead_nm(r);
    let lag = roll_lag_lead_nm(gs_kts);
    let tracking_band = if was_track { 2.0 * lead + lag } else { lead + lag };
    if xtk.abs() <= tracking_band {
        // The heading is SET, then left for up to 30 s (`HeadingHold`), so
        // it is the cut for the offset left once the turn onto it is
        // flown: from a 30 degree intercept that is the course itself. The
        // bare pursuit set 14 degrees at the roll-out and crossed the line
        // 340 m before the next correction (the #161 join at EGKK).
        let toward = (crate::angles::signed_diff_deg(track_true, c) * -xtk.signum()).clamp(0.0, 90.0);
        let closing = r * (1.0 - toward.to_radians().cos())
            + gs_kts.max(60.0) / 3600.0 * ROLL_LAG_S * toward.to_radians().sin();
        let cut = ((xtk.abs() - closing).max(0.0) / TRACK_AIM_NM)
            .atan()
            .to_degrees()
            .min(INTERCEPT_DEG);
        return Some(((c - xtk.signum() * cut).rem_euclid(360.0), Steer::Track));
    }
    if along < intercept_room_nm(xtk, track_true, c, r) + FIX_MARGIN_NM {
        return None;
    }
    Some((
        (c - xtk.signum() * INTERCEPT_DEG).rem_euclid(360.0),
        Steer::Intercept,
    ))
}

/// Has a course leg been flown past its fix? Abeam it within the capture
/// distance counts, as a fly-by does: an intercept that is still closing the
/// line never passes within the capture radius of the point itself.
pub fn course_passed(line: &CourseLine, lat: f64, lon: f64, capture_nm: f64) -> bool {
    line.along_nm(lat, lon) <= capture_nm
}

/// What the IF join decided this tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Join {
    /// Near side: the IF is flown to, as published.
    Direct,
    /// The IF is behind: skip it and intercept the final course. `why` is
    /// `intercept` (room to spare) or `out_of_room` (a pattern that ran out
    /// of final; the course leg or the FAF's own direct-to takes over).
    Skip { why: &'static str },
    /// Fly this TRACK (true), a leg of the reversal pattern. `started` on
    /// the tick the pattern began.
    Pattern {
        track_true: f64,
        leg: reversal::Leg,
        started: bool,
        side: f64,
    },
}

/// The join onto an approach's final from its IF, one per flight.
#[derive(Debug, Default)]
pub struct FinalJoin {
    idx: Option<usize>,
    rev: Option<Reversal>,
    /// The base turn as last sized ([`base_turn_along_nm`]), nm along.
    base_at_nm: Option<f64>,
    /// The IF skipped for an intercept, whose altitude is still owed while
    /// it lies ahead along the final ([`FinalJoin::skipped_if_ahead_nm`]).
    skipped_if: Option<usize>,
    /// Fly the 2026-09-28 geometry (the replays' "as flown").
    #[cfg(test)]
    as_flown: bool,
}

impl FinalJoin {
    pub fn flying_pattern(&self) -> bool {
        self.rev.is_some()
    }

    /// The join as flown on 2026-09-28: the base two sizing radii beyond
    /// the established point, the skip on the old room.
    #[cfg(test)]
    pub fn as_flown() -> Self {
        FinalJoin { as_flown: true, ..Default::default() }
    }

    #[cfg(test)]
    fn old_geometry(&self) -> bool {
        self.as_flown
    }

    #[cfg(not(test))]
    fn old_geometry(&self) -> bool {
        false
    }

    /// The room an intercept from here needs, by this join's geometry.
    fn room_nm(&self, xtk_nm: f64, track_true: f64, course_true: f64, r_nm: f64) -> f64 {
        #[cfg(test)]
        if self.as_flown {
            return intercept_room_as_flown_nm(xtk_nm, track_true, course_true, r_nm);
        }
        intercept_room_nm(xtk_nm, track_true, course_true, r_nm)
    }

    /// Where the base turn begins, nm along the FAF's course: sized on the
    /// downwind's offset and the turn radius, or the old flat two radii.
    fn base_turn_nm(&self, xtk_nm: f64, gs_kts: f64) -> f64 {
        let rs = reversal::sizing_radius_nm(gs_kts);
        if self.old_geometry() {
            return FAF_ESTABLISHED_NM + 2.0 * rs + reversal::BASE_BEYOND_JOIN_NM;
        }
        let r = crate::procedure::enroute_turn_radius_nm(gs_kts);
        base_turn_along_nm(xtk_nm.abs(), r)
    }

    /// The miles still to fly round the pattern to the IF (leg `idx`), on
    /// the FAF's course line: the rest of the outbound leg to the base
    /// turn, the base across, and the inbound back to the IF. `None` when
    /// no pattern is being flown. The descent to the IF's altitude is paced
    /// on THESE miles, never on the straight line to a fix the pattern is
    /// flying away from.
    ///
    /// EGBB 15 (2026-09-28, sortie 1790601822-full_auto, #156): from GROVE,
    /// abeam the threshold 7.6 nm off, the pattern flew outbound for 250 s
    /// with the IF never more than about 5 nm away, so the path to its
    /// 2,500 ft held a 4,011 ft plateau the whole downwind and base; the
    /// descent came in the last 4 nm before the FAF at up to 2,240 fpm, the
    /// speedbrake out again at 218 kt, handed to autoland at 222 kt and
    /// 2,111 ft AGL. Pure.
    pub fn pattern_to_go_nm(
        &self,
        legs: &[ProcLeg],
        idx: usize,
        lat: f64,
        lon: f64,
        gs_kts: f64,
        mag_var: f64,
    ) -> Option<f64> {
        let rev = self.rev.as_ref().filter(|_| self.idx == Some(idx))?;
        let at_if = legs.get(idx)?.fix.as_ref()?;
        let line = CourseLine::of_leg(legs.get(idx + 1)?, mag_var)?;
        let along = line.along_nm(lat, lon);
        let xtk = line.xtk_nm(lat, lon).abs();
        let if_along = line.along_nm(at_if.lat, at_if.lon).max(0.0);
        let turn_along = if self.old_geometry() {
            FAF_ESTABLISHED_NM + 2.0 * reversal::sizing_radius_nm(gs_kts)
        } else {
            self.base_at_nm.unwrap_or_else(|| self.base_turn_nm(xtk, gs_kts))
        };
        if self.old_geometry() {
            return Some(match rev.leg {
                reversal::Leg::Outbound => {
                    (turn_along - along).max(0.0)
                        + xtk
                        + (turn_along.max(along) - if_along).max(0.0)
                }
                reversal::Leg::Base | reversal::Leg::Inbound => xtk + (along - if_along).max(0.0),
            });
        }
        // The IF's altitude by the end of the base, as a crew has the
        // platform before it turns onto the intercept: the base turn runs a
        // radius further out, and counted to the IF along the line the
        // miles grew on the base and held the descent level there, handing
        // over 300 ft over the glideslope (#161).
        Some(match rev.leg {
            reversal::Leg::Outbound => (turn_along - along).max(0.0) + xtk,
            reversal::Leg::Base => xtk,
            reversal::Leg::Inbound => xtk + (along - if_along).max(0.0),
        })
    }

    /// On the leg after an IF skipped for the intercept (leg `idx`): the IF
    /// (its index) and the miles along the final still to its abeam point,
    /// while it lies ahead. A join from a base beyond the IF meets the final
    /// OUTSIDE it, so the IF's altitude is still owed there; paced on the
    /// FAF's distance instead, the path held level after the skip (at
    /// EGBB 15 from 4,796 ft for 38 s, 1790624015) and handed over 400 ft
    /// over the glideslope (#161). Pure.
    pub fn skipped_if_ahead_nm(
        &self,
        legs: &[ProcLeg],
        idx: usize,
        lat: f64,
        lon: f64,
        mag_var: f64,
    ) -> Option<(usize, f64)> {
        if self.old_geometry() {
            return None;
        }
        let i = self.skipped_if.filter(|i| i + 1 == idx)?;
        let at_if = legs.get(i)?.fix.as_ref()?;
        let line = CourseLine::of_leg(legs.get(idx)?, mag_var)?;
        let d = line.along_nm(lat, lon) - line.along_nm(at_if.lat, at_if.lon);
        (d > 0.0).then_some((i, d))
    }

    /// Advance the join for leg `idx`. `None` unless that leg is an IF
    /// followed by a course leg (a CF with a course and a fix): the only
    /// shape this decides. `track_true` is the ground track, `platform_ft`
    /// the altitude the join holds (the IF's constraint), `descent_fpm` the
    /// airframe's enroute descent, for the pattern's height rule.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        legs: &[ProcLeg],
        idx: usize,
        lat: f64,
        lon: f64,
        track_true: f64,
        gs_kts: f64,
        mag_var: f64,
        alt_ft: f64,
        descent_fpm: f64,
    ) -> Option<Join> {
        if self.idx != Some(idx) {
            if self.skipped_if.is_some_and(|i| i + 1 != idx) {
                self.skipped_if = None;
            }
            self.idx = Some(idx);
            self.rev = None;
            self.base_at_nm = None;
        }
        let leg = legs.get(idx)?;
        if leg.leg_type != "IF" {
            return None;
        }
        let at_if = leg.fix.as_ref()?;
        let line = CourseLine::of_leg(legs.get(idx + 1)?, mag_var)?;
        let c = line.course_true();
        let along = line.along_nm(lat, lon);
        let xtk = line.xtk_nm(lat, lon);
        let r = crate::procedure::enroute_turn_radius_nm(gs_kts);
        if self.rev.is_none() {
            let to_if = crate::nav::bearing_deg(lat, lon, at_if.lat, at_if.lon);
            if crate::angles::abs_diff_deg(c, to_if) <= IF_MAX_TURN_DEG {
                return Some(Join::Direct);
            }
        }
        // Never off the downwind: from there the intercept is a teardrop
        // the heading mode flies wide, and the base turn is sized for it.
        let slack = if self.old_geometry() { 0.0 } else { JOIN_SLACK_NM };
        let downwind = self.rev.is_some_and(|r| r.leg == reversal::Leg::Outbound);
        let turn_ok = self.old_geometry() || !downwind;
        if turn_ok && along - self.room_nm(xtk, track_true, c, r) >= FAF_ESTABLISHED_NM + slack {
            self.rev = None;
            self.base_at_nm = None;
            self.skipped_if = Some(idx);
            return Some(Join::Skip { why: "intercept" });
        }
        if self
            .rev
            .is_some_and(|rev| rev.leg == reversal::Leg::Inbound && along < FAF_ESTABLISHED_NM)
        {
            self.rev = None;
            return Some(Join::Skip { why: "out_of_room" });
        }
        // The pattern against the FAF's course line: the base turn where the
        // turn onto base, the 60 degree turn off it onto the intercept and
        // the cut leave FAF_ESTABLISHED_NM and the slack, sized on the
        // downwind's offset while it is flown and then held (#161).
        let on_downwind = self.rev.is_none_or(|r| r.leg == reversal::Leg::Outbound);
        let base_at = match self.base_at_nm {
            Some(b) if !on_downwind => b,
            _ => self.base_turn_nm(xtk, gs_kts),
        };
        self.base_at_nm = Some(base_at);
        let join_nm = base_at - reversal::BASE_BEYOND_JOIN_NM;
        let platform_ft = leg.alt1_ft.unwrap_or(alt_ft);
        let started = self.rev.is_none();
        let rev = self.rev.get_or_insert(Reversal {
            side: reversal::pick_side(xtk, None, None),
            leg: reversal::Leg::Outbound,
            platform_ft,
            high: false,
        });
        rev.high = reversal::too_high_to_turn(
            alt_ft,
            rev.platform_ft,
            along,
            join_nm,
            gs_kts,
            descent_fpm,
        );
        let track = reversal::step(
            rev, &line.0, lat, lon, along, xtk, track_true, gs_kts, join_nm, false,
        )
        .unwrap_or(c);
        Some(Join::Pattern {
            track_true: track,
            leg: rev.leg,
            started,
            side: rev.side,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nav::{project_latlon, Fix};
    use crate::heading_bug::HeadingHold;
    use crate::procedure::{capture_radius_nm, leg_done, leg_turn_deg};

    /// EGKK I26L as the CIFP codes it (the rows of
    /// `procedure::a_runway_map_hands_autoland_the_approach_at_the_faf`),
    /// resolved: IF CF26L, CF FF26L (FAF, course 256), CF RW26L (MAP).
    fn egkk_i26l() -> Vec<ProcLeg> {
        let rows = "APPCH:010,I,I26L, ,CF26L,EG,P,C,E  I, ,   ,IF, ,IWW,EG,P,I,      ,0766,0082,    ,    ,J,02000,02000,     , ,   ,    ,   , , , , , ,0,D,S;\n\
                    APPCH:020,I,I26L, ,FF26L,EG,P,C,E  F, ,   ,CF, ,IWW,EG,P,I,      ,0766,0072,2560,0010,H,02000,02000,     , ,   ,-300,   ,EGKK,EG,P,A, ,0,D,S;\n\
                    APPCH:030,I,I26L, ,RW26L,EG,P,G,G  M, ,   ,CF, ,IWW,EG,P,I,      ,0766,0017,2560,0055, ,00248,     ,     , ,   ,-300,   , , , , , ,0,D,S;\n\
                    APPCH:040,I,I26L, , , , , ,  M , ,   ,CA, , , , , ,      ,    ,    ,2560,    ,+,02000,     ,     , ,   ,    ,   , , , , , ,0,D,S;\n";
        let raw = crate::nav::parse_proc_legs(
            rows.lines().map(str::trim),
            crate::nav::ProcKind::Appch,
            "I26L",
            None,
        );
        let mut legs = crate::nav::straight_in_legs(raw, "26L").expect("straight in");
        let at = |ident: &str, lat: f64, lon: f64| Fix {
            ident: ident.into(),
            lat,
            lon,
        };
        legs[0].fix = Some(at("CF26L", 51.173697222, -0.003744444));
        legs[1].fix = Some(at("FF26L", 51.170172222, -0.029625));
        legs[2].fix = Some(at("RW26L", 51.150672, -0.171944));
        legs
    }

    /// X-Plane's variation at EGKK: the CIFP's 256 magnetic is the 257.75
    /// true the fixes lie on.
    const EGKK_VAR: f64 = -1.75;

    /// The heading-mode autopilot of the reversal tests: a rate-limited roll
    /// to at most `max_bank`, turning the short way to the bug.
    struct Plant {
        lat: f64,
        lon: f64,
        hdg: f64,
        bank: f64,
        gs_kts: f64,
        max_bank: f64,
    }

    impl Plant {
        fn fly(&mut self, bug_true: f64, dt: f64) {
            let err = crate::angles::signed_diff_deg(bug_true, self.hdg);
            let want = (err * 1.5).clamp(-self.max_bank, self.max_bank);
            self.bank += (want - self.bank).clamp(-5.0 * dt, 5.0 * dt);
            let v = self.gs_kts * 0.514_444;
            let rate = (9.81 * self.bank.to_radians().tan() / v).to_degrees();
            self.hdg = (self.hdg + rate * dt).rem_euclid(360.0);
            let (lat, lon) =
                project_latlon(self.lat, self.lon, self.hdg, self.gs_kts * dt / 3600.0);
            self.lat = lat;
            self.lon = lon;
        }
    }

    struct Flown {
        steers: Vec<&'static str>,
        skipped: Option<&'static str>,
        /// At the FAF's capture (the autoland handoff): along to go, nm,
        /// cross-track, m, and the track against the course, degrees.
        handoff: (f64, f64, f64),
        /// The along distance before the FAF at which the aircraft was first
        /// established (within 300 m and 10 degrees).
        established_nm: Option<f64>,
        bug_writes: usize,
    }

    /// The follower's lateral, tick by tick, as `procedure::fly` composes
    /// it, from `p` to the handoff at the FAF.
    fn fly_approach(mut p: Plant, legs: &[ProcLeg]) -> Flown {
        let mut join = FinalJoin::default();
        let mut hold = HeadingHold::default();
        let mut bug_true = p.hdg;
        let mut idx = 0usize;
        let mut was = Steer::Direct;
        let mut out = Flown {
            steers: vec![],
            skipped: None,
            handoff: (0.0, 0.0, 0.0),
            established_nm: None,
            bug_writes: 0,
        };
        let faf = CourseLine::of_leg(&legs[1], EGKK_VAR).unwrap();
        for tick in 0..8000 {
            let now = tick as f64 * 0.5;
            let leg = &legs[idx];
            let mut steer = Steer::Direct;
            let mut track = None;
            match join.step(
                legs, idx, p.lat, p.lon, p.hdg, p.gs_kts, EGKK_VAR, 2000.0, 1500.0,
            ) {
                Some(Join::Skip { why }) => {
                    out.skipped = Some(why);
                    idx += 1;
                    continue;
                }
                Some(Join::Pattern {
                    track_true, leg, ..
                }) => {
                    steer = Steer::Pattern(leg);
                    track = Some(track_true);
                }
                _ => {}
            }
            let line = CourseLine::of_leg(leg, EGKK_VAR);
            if track.is_none() {
                if let Some((t, s)) = line.as_ref().and_then(|l| {
                    course_track(l, p.lat, p.lon, p.hdg, p.gs_kts, was == Steer::Track)
                }) {
                    track = Some(t);
                    steer = s;
                }
            }
            let f = leg.fix.as_ref().unwrap();
            let track =
                track.unwrap_or_else(|| crate::nav::bearing_deg(p.lat, p.lon, f.lat, f.lon));
            if out.steers.last() != Some(&steer.word()) {
                out.steers.push(steer.word());
            }
            was = steer;
            let mag = |t: f64| crate::autoland::true_to_mag(t, EGKK_VAR);
            if let Some(h) = hold.decide(steer.key(idx), mag(track), mag(p.hdg), p.bank, now) {
                bug_true = h - EGKK_VAR;
                out.bug_writes += 1;
            }
            let along = faf.along_nm(p.lat, p.lon);
            let xtk_m = faf.xtk_nm(p.lat, p.lon) * 1852.0;
            let off = crate::angles::signed_diff_deg(p.hdg, faf.course_true());
            if out.established_nm.is_none()
                && xtk_m.abs() < 300.0
                && off.abs() < 10.0
                && along > 0.0
            {
                out.established_nm = Some(along);
            }
            let d = crate::nav::haversine_nm(p.lat, p.lon, f.lat, f.lon);
            let cap = capture_radius_nm(
                p.gs_kts,
                leg_turn_deg(p.lat, p.lon, leg, legs.get(idx + 1)).unwrap_or(0.0),
            );
            let done = leg_done(leg, Some(d), 2000.0, cap, p.lat, p.lon)
                || line
                    .as_ref()
                    .is_some_and(|l| course_passed(l, p.lat, p.lon, cap));
            if done {
                if idx == 1 {
                    out.handoff = (along, xtk_m, off);
                    return out;
                }
                idx += 1;
            }
            p.fly(bug_true, 0.5);
        }
        panic!("never reached the FAF: {:?}", out.steers);
    }

    /// Sortie 1790534902: the approach armed at t 1790537411.8 just past
    /// WILLO, 9.6 nm south of the 26L final and 7.8 nm downwind of FF26L,
    /// tracking 264 at 7,066 ft, 218 kt. Flown direct, it was handed to
    /// autoland 4.57 nm out, 1,965 m off, tracking 044.
    #[test]
    fn a_far_side_join_at_egkk_flies_a_pattern_and_reaches_the_faf_established() {
        let legs = egkk_i26l();
        let (lat, lon) = (50.98614, -0.17858);
        let line = CourseLine::of_leg(&legs[1], EGKK_VAR).unwrap();
        assert!((line.course_true() - 257.75).abs() < 0.1);
        assert!(line.along_nm(lat, lon) < -7.0, "downwind of the FAF");
        assert!(line.xtk_nm(lat, lon) < -9.0, "left of the course");
        for (gs, bank) in [(220.0, 24.0), (220.0, 20.0), (180.0, 25.0)] {
            let p = Plant {
                lat,
                lon,
                hdg: 264.0,
                bank: 0.0,
                gs_kts: gs,
                max_bank: bank,
            };
            let flown = fly_approach(p, &legs);
            assert_eq!(
                &flown.steers[..4],
                ["pattern_outbound", "pattern_base", "intercept", "track"],
                "{gs} kt, {bank} deg: {:?}",
                flown.steers
            );
            assert_eq!(
                flown.skipped,
                Some("intercept"),
                "the IF skipped with room to spare"
            );
            let (along, xtk_m, off) = flown.handoff;
            assert!(along < 1.0, "handed over at the FAF, {along:.2} nm to go");
            assert!(
                xtk_m.abs() < 150.0,
                "{gs} kt, {bank} deg: {xtk_m:.0} m off at the FAF"
            );
            assert!(
                off.abs() < 10.0,
                "tracking {off:.1} degrees off the course at the FAF"
            );
            let est = flown.established_nm.expect("established before the FAF");
            assert!(
                est >= FAF_ESTABLISHED_NM,
                "{gs} kt, {bank} deg: established only {est:.2} nm before the FAF"
            );
            // Set, not steered: a handful of heading changes, never a stream.
            assert!(flown.bug_writes < 20, "{} bug writes", flown.bug_writes);
        }
    }

    /// The same final from the NEAR side (straight in from the east, 3 nm
    /// south of the extended course, 12 nm out): the IF is flown to as
    /// published, and the FAF's CF leg is intercepted and tracked instead of
    /// homed on.
    #[test]
    fn a_near_side_join_flies_the_if_then_tracks_the_faf_course() {
        let legs = egkk_i26l();
        let line = CourseLine::of_leg(&legs[1], EGKK_VAR).unwrap();
        let recip = (line.course_true() + 180.0) % 360.0;
        let (clat, clon) = project_latlon(51.170172222, -0.029625, recip, 12.0);
        let (lat, lon) = project_latlon(clat, clon, (line.course_true() + 270.0) % 360.0, 3.0);
        let mut j = FinalJoin::default();
        assert_eq!(
            j.step(&legs, 0, lat, lon, 280.0, 210.0, EGKK_VAR, 3000.0, 1500.0),
            Some(Join::Direct)
        );
        let p = Plant {
            lat,
            lon,
            hdg: 280.0,
            bank: 0.0,
            gs_kts: 210.0,
            max_bank: 24.0,
        };
        let flown = fly_approach(p, &legs);
        assert_eq!(flown.skipped, None, "the IF is flown: {:?}", flown.steers);
        assert_eq!(flown.steers[0], "direct");
        let (_, xtk_m, off) = flown.handoff;
        assert!(xtk_m.abs() < 150.0, "{xtk_m:.0} m off at the FAF");
        assert!(off.abs() < 10.0, "{off:.1} degrees off at the FAF");
    }

    /// #161: the EGBB 15 join (sortie 1790635335) skipped CF15 off its
    /// base at 8.44 nm before FF15, 2.34 nm right, tracking 051.5 at 229 kt
    /// over the ground, and was on the course by about 6.3 nm: 2.1 nm of
    /// final used. The turn counted as flown says so; charged `r sin(turn)`
    /// and no closing, the room was 6.5 nm and the skip came 4 nm late.
    #[test]
    fn the_intercept_room_counts_the_turn_it_flies() {
        let course = 145.0 + 0.9;
        let r = crate::procedure::enroute_turn_radius_nm(229.0);
        let new = intercept_room_nm(2.34, 51.5, course, r);
        let old = intercept_room_as_flown_nm(2.34, 51.5, course, r);
        assert!((1.8..=2.6).contains(&new), "{new:.2}");
        assert!(old > 6.0, "{old:.2}");
        // Along the course 2 nm off: the 30 degree turn in, the cut, the
        // roll-out.
        let flat = intercept_room_nm(2.0, course, course, r);
        let c30 = INTERCEPT_DEG.to_radians();
        let want = r + (2.0 - 2.0 * r * (1.0 - c30.cos())) / c30.tan();
        assert!((flat - want).abs() < 1e-9, "{flat} {want}");
        // A downwind 4.9 nm out: the base turn 4.2 nm beyond the FAF (the
        // old flat 5.7) leaves the intercept its room, the final its 2 nm.
        let b = base_turn_along_nm(4.9, r);
        assert!((4.0..=4.4).contains(&b), "{b:.2}");
    }

    /// The course law alone: 30 degrees toward the line from either side,
    /// the course inside the roll-out lead, a correction that never cuts
    /// more than the intercept, and no course at all when the line cannot be
    /// joined before the fix.
    #[test]
    fn a_cf_leg_is_a_course_intercept_not_a_direct_to() {
        let legs = egkk_i26l();
        let line = CourseLine::of_leg(&legs[1], EGKK_VAR).unwrap();
        let c = line.course_true();
        let recip = (c + 180.0) % 360.0;
        let at = |along: f64, right: f64| {
            let (a, b) = project_latlon(51.170172222, -0.029625, recip, along);
            project_latlon(
                a,
                b,
                (c + if right >= 0.0 { 90.0 } else { 270.0 }) % 360.0,
                right.abs(),
            )
        };
        // Sortie 1790534902's own bug on this leg was 024: straight at the
        // FAF from 1.2 nm left of the course.
        let (lat, lon) = at(10.0, -2.0);
        let (t, s) = course_track(&line, lat, lon, c, 210.0, false).unwrap();
        assert_eq!(s, Steer::Intercept);
        assert!(
            (crate::angles::signed_diff_deg(t, c) - 30.0).abs() < 0.01,
            "from the left: course + 30, got {t:.1}"
        );
        let (lat, lon) = at(10.0, 2.0);
        let (t, _) = course_track(&line, lat, lon, c, 210.0, false).unwrap();
        assert!(
            (crate::angles::signed_diff_deg(t, c) + 30.0).abs() < 0.01,
            "from the right: course - 30"
        );
        // Inside the lead (0.24 nm at 210 kt, and 0.15 for the roll-in):
        // the course, nearly.
        let (lat, lon) = at(6.0, -0.1);
        let (t, s) = course_track(&line, lat, lon, c, 210.0, false).unwrap();
        assert_eq!(s, Steer::Track);
        assert!(
            crate::angles::signed_diff_deg(t, c) > 0.0 && crate::angles::signed_diff_deg(t, c) < 5.0,
            "a gentle correction right"
        );
        // Rolling out of the 30 degree cut at the lead: the course itself,
        // since the turn onto it closes the rest (#161).
        let (lat, lon) = at(6.0, -0.35);
        let (t, s) = course_track(&line, lat, lon, c + 30.0, 210.0, false).unwrap();
        assert_eq!(s, Steer::Track);
        assert!(crate::angles::abs_diff_deg(t, c) < 0.01, "the course, got {:.1}", crate::angles::signed_diff_deg(t, c));
        // Tracking holds to twice the geometric lead.
        let (lat, lon) = at(6.0, -0.45);
        assert_eq!(
            course_track(&line, lat, lon, c, 210.0, true).unwrap().1,
            Steer::Track
        );
        assert_eq!(
            course_track(&line, lat, lon, c, 210.0, false).unwrap().1,
            Steer::Intercept
        );
        // 2 nm off with 2 nm to run: no intercept fits, the fix is flown to.
        let (lat, lon) = at(2.0, -2.0);
        assert_eq!(course_track(&line, lat, lon, c, 210.0, false), None);
        // An IF is not a course leg.
        assert!(CourseLine::of_leg(&legs[0], EGKK_VAR).is_none());
        // Abeam the fix counts as passed.
        let (lat, lon) = at(0.5, -1.0);
        assert!(course_passed(&line, lat, lon, 0.8));
        assert!(!course_passed(&line, lat, lon, 0.3));
    }
}

/// On an RF arc, the correction back onto it aims this far ahead, nm.
const ARC_AIM_NM: f64 = 1.0;
/// An arc whose end radial lies this far or more round the turn is behind
/// us (no coded RF turns through more than this).
const ARC_PASSED_DEG: f64 = 330.0;

/// Degrees of the arc still to fly, measured round the turn from the
/// aircraft's radial to the end fix's radial.
pub fn arc_remaining_deg(arc: &crate::nav::Arc, lat: f64, lon: f64) -> f64 {
    let here = crate::nav::bearing_deg(arc.center_lat, arc.center_lon, lat, lon);
    let end = crate::nav::bearing_deg(arc.center_lat, arc.center_lon, arc.end_lat, arc.end_lon);
    if arc.right {
        (end - here).rem_euclid(360.0)
    } else {
        (here - end).rem_euclid(360.0)
    }
}

/// The TRACK (true) that flies an RF arc (#101): the tangent at the
/// aircraft's radial, cut toward the arc by the radial error over
/// [`ARC_AIM_NM`], at most the standard intercept. Flown direct to its end
/// fix, an RF is a chord across the airspace the arc was drawn round.
pub fn arc_track(arc: &crate::nav::Arc, lat: f64, lon: f64) -> f64 {
    let radial = crate::nav::bearing_deg(arc.center_lat, arc.center_lon, lat, lon);
    let dir = if arc.right { 1.0 } else { -1.0 };
    let tangent = radial + dir * 90.0;
    let err = crate::nav::haversine_nm(arc.center_lat, arc.center_lon, lat, lon) - arc.radius_nm;
    let cut = (err.abs() / ARC_AIM_NM).atan().to_degrees().min(INTERCEPT_DEG);
    // Outside the arc the correction turns toward the centre: the turn's
    // own direction.
    (tangent + dir * err.signum() * cut).rem_euclid(360.0)
}

/// Has the arc been flown? Its end radial reached, or within the capture
/// distance of it round the arc.
pub fn arc_done(arc: &crate::nav::Arc, lat: f64, lon: f64, capture_nm: f64) -> bool {
    let rem = arc_remaining_deg(arc, lat, lon);
    rem >= ARC_PASSED_DEG || rem.to_radians() * arc.radius_nm <= capture_nm
}

/// A holding pattern's phase: to the fix, the turn outbound, the timed
/// outbound leg, the turn back, the inbound course to the fix.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HoldPhase {
    ToFix,
    TurnOut,
    Outbound,
    TurnIn,
    Inbound,
}

impl HoldPhase {
    pub fn word(self) -> &'static str {
        match self {
            HoldPhase::ToFix => "hold_to_fix",
            HoldPhase::TurnOut => "hold_turn_out",
            HoldPhase::Outbound => "hold_outbound",
            HoldPhase::TurnIn => "hold_turn_in",
            HoldPhase::Inbound => "hold_inbound",
        }
    }
}

/// A hold's 180 degree turns are flown in two halves (90 degrees, then the
/// rest), so the heading mode never meets a 180 it may take the short way.
const HOLD_HALF_TURN_DONE_DEG: f64 = 20.0;
/// The outbound leg is timed from this close to the outbound track.
const HOLD_OUTBOUND_BAND_DEG: f64 = 10.0;
/// A hold to an altitude that has not made it after this many circuits is
/// left, and said.
pub const HOLD_MAX_CIRCUITS: u32 = 6;
/// A hold's altitude is made within this, feet (the follower's own margin).
const HOLD_ALT_MARGIN_FT: f64 = 100.0;

/// How a hold ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HoldEnd {
    /// HF: the one circuit flown; HA: the altitude made at the fix.
    Flown,
    /// HA past [`HOLD_MAX_CIRCUITS`].
    GaveUp,
}

/// One holding leg flown as a racetrack (#101): HF once round, HA until its
/// altitude is made at the fix. Direct entry (to the fix, then outbound);
/// the outbound leg is the chart's time (else 1 minute at or below 14,000
/// ft, 1.5 above) or distance. Pure; the caller supplies the clock.
#[derive(Debug, Clone)]
pub struct HoldFlight {
    pub phase: HoldPhase,
    pub circuits: u32,
    /// The inbound course, true.
    inbound_true: f64,
    right: bool,
    fix: (f64, f64),
    leg_min: Option<f64>,
    leg_nm: Option<f64>,
    /// Inside a turn: its first 90 degrees are done.
    half: bool,
    outbound_from: Option<(f64, f64, f64)>,
}

/// One hold tick's answer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HoldStep {
    pub track_true: f64,
    pub steer: Steer,
    pub end: Option<HoldEnd>,
    /// The phase changed this tick (for the log).
    pub moved: bool,
}

impl HoldFlight {
    /// A hold for `leg` when it is HF or HA with a fix and an inbound
    /// course. HM (hold until told) is `None`: the follower flies its fix
    /// and goes on, as before, since nobody here can release it.
    pub fn for_leg(leg: &ProcLeg, mag_var: f64) -> Option<Self> {
        if !matches!(leg.leg_type.as_str(), "HF" | "HA") {
            return None;
        }
        let f = leg.fix.as_ref()?;
        let course = leg.course_mag?;
        Some(HoldFlight {
            phase: HoldPhase::ToFix,
            circuits: 0,
            inbound_true: crate::runway::mag_to_true(course, mag_var),
            right: leg.turn != Some('L'),
            fix: (f.lat, f.lon),
            leg_min: leg.path.hold_leg_min.filter(|m| *m > 0.0),
            leg_nm: leg.dist_nm.filter(|d| *d > 0.0),
            half: false,
            outbound_from: None,
        })
    }

    /// At the fix: is the hold over? HF after one circuit; HA once its
    /// altitude is made, or past the cap.
    fn exit_due(&self, leg: &ProcLeg, alt_ft: f64) -> Option<HoldEnd> {
        if leg.leg_type == "HF" {
            return (self.circuits >= 1).then_some(HoldEnd::Flown);
        }
        let made = match (leg.alt1_ft, leg.alt_desc) {
            (None, _) => true,
            (Some(t), '+') => alt_ft >= t - HOLD_ALT_MARGIN_FT,
            (Some(t), '-') => alt_ft <= t + HOLD_ALT_MARGIN_FT,
            (Some(t), _) => (alt_ft - t).abs() <= HOLD_ALT_MARGIN_FT,
        };
        if made {
            Some(HoldEnd::Flown)
        } else if self.circuits >= HOLD_MAX_CIRCUITS {
            Some(HoldEnd::GaveUp)
        } else {
            None
        }
    }

    /// Steer the hold one tick. `capture_nm` is the fix's capture distance,
    /// `now_s` any monotonic clock.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        leg: &ProcLeg,
        lat: f64,
        lon: f64,
        track_true: f64,
        gs_kts: f64,
        alt_ft: f64,
        capture_nm: f64,
        now_s: f64,
    ) -> HoldStep {
        let dir = if self.right { 1.0 } else { -1.0 };
        let outbound = (self.inbound_true + 180.0).rem_euclid(360.0);
        let abeam_out = self.inbound_true + dir * 90.0;
        let abeam_in = outbound + dir * 90.0;
        let line = CourseLine::through(self.fix.0, self.fix.1, self.inbound_true);
        let at_fix = crate::nav::haversine_nm(lat, lon, self.fix.0, self.fix.1) <= capture_nm;
        let was = self.phase;
        let mut end = None;
        match self.phase {
            HoldPhase::ToFix if at_fix => {
                // A hold to an altitude already made is flown through.
                if leg.leg_type == "HA" {
                    end = self.exit_due(leg, alt_ft).filter(|e| *e == HoldEnd::Flown);
                }
                if end.is_none() {
                    self.phase = HoldPhase::TurnOut;
                    self.half = false;
                }
            }
            HoldPhase::TurnOut => {
                if !self.half && crate::angles::abs_diff_deg(abeam_out, track_true) <= HOLD_HALF_TURN_DONE_DEG {
                    self.half = true;
                }
                if self.half && crate::angles::abs_diff_deg(outbound, track_true) <= HOLD_OUTBOUND_BAND_DEG {
                    self.phase = HoldPhase::Outbound;
                    self.outbound_from = Some((lat, lon, now_s));
                }
            }
            HoldPhase::Outbound => {
                let (la, lo, t0) = self.outbound_from.unwrap_or((lat, lon, now_s));
                let done = match self.leg_nm {
                    Some(nm) => crate::nav::haversine_nm(la, lo, lat, lon) >= nm,
                    None => {
                        let min = self.leg_min.unwrap_or(if alt_ft > 14_000.0 { 1.5 } else { 1.0 });
                        now_s - t0 >= min * 60.0
                    }
                };
                if done {
                    self.phase = HoldPhase::TurnIn;
                    self.half = false;
                }
            }
            HoldPhase::TurnIn => {
                if crate::angles::abs_diff_deg(abeam_in, track_true) <= HOLD_HALF_TURN_DONE_DEG {
                    self.phase = HoldPhase::Inbound;
                }
            }
            HoldPhase::Inbound => {
                if at_fix || course_passed(&line, lat, lon, capture_nm) {
                    self.circuits += 1;
                    end = self.exit_due(leg, alt_ft);
                    if end.is_none() {
                        self.phase = HoldPhase::TurnOut;
                        self.half = false;
                    }
                }
            }
            _ => {}
        }
        let track = match self.phase {
            HoldPhase::ToFix => crate::nav::bearing_deg(lat, lon, self.fix.0, self.fix.1),
            HoldPhase::TurnOut if !self.half => abeam_out,
            HoldPhase::TurnOut | HoldPhase::Outbound => outbound,
            HoldPhase::TurnIn => abeam_in,
            HoldPhase::Inbound => course_track(&line, lat, lon, track_true, gs_kts, true)
                .map(|(t, _)| t)
                .unwrap_or_else(|| crate::nav::bearing_deg(lat, lon, self.fix.0, self.fix.1)),
        };
        HoldStep {
            track_true: track.rem_euclid(360.0),
            steer: Steer::Hold(self.phase),
            end,
            moved: self.phase != was,
        }
    }
}

#[cfg(test)]
mod arc_hold_tests {
    use super::*;
    use crate::nav::{project_latlon, Arc, Fix, ProcLeg};

    fn arc() -> Arc {
        // Centre 47N 11E, 3 nm, clockwise, ending due east of it.
        let (end_lat, end_lon) = project_latlon(47.0, 11.0, 90.0, 3.0);
        Arc { center_lat: 47.0, center_lon: 11.0, radius_nm: 3.0, right: true, end_lat, end_lon }
    }

    /// #101: on the arc the track is its tangent; outside it cuts in,
    /// inside it cuts out, never past the intercept; the end radial ends it.
    #[test]
    fn an_rf_leg_is_flown_round_its_arc_not_across_its_chord() {
        let a = arc();
        let (lat, lon) = project_latlon(47.0, 11.0, 0.0, 3.0);
        assert!(crate::angles::abs_diff_deg(arc_track(&a, lat, lon), 90.0) < 1.0, "north of the centre, clockwise: east");
        assert!((arc_remaining_deg(&a, lat, lon) - 90.0).abs() < 1.0);
        assert!(!arc_done(&a, lat, lon, 0.5));
        let (lat, lon) = project_latlon(47.0, 11.0, 0.0, 3.5);
        assert!(crate::angles::signed_diff_deg(arc_track(&a, lat, lon), 90.0) > 10.0, "outside: toward the centre");
        let (lat, lon) = project_latlon(47.0, 11.0, 0.0, 2.5);
        let cut = crate::angles::signed_diff_deg(arc_track(&a, lat, lon), 90.0);
        assert!(cut < -10.0 && cut >= -INTERCEPT_DEG - 0.01, "inside: outward, {cut}");
        let (lat, lon) = project_latlon(47.0, 11.0, 85.0, 3.0);
        assert!(arc_done(&a, lat, lon, 0.5), "within the capture of the end");
        let (lat, lon) = project_latlon(47.0, 11.0, 100.0, 3.0);
        assert!(arc_done(&a, lat, lon, 0.5), "past the end radial");
        // Anticlockwise mirrors it.
        let l = Arc { right: false, ..a };
        let (lat, lon) = project_latlon(47.0, 11.0, 180.0, 3.0);
        assert!(crate::angles::abs_diff_deg(arc_track(&l, lat, lon), 90.0) < 1.0, "south, anticlockwise: east");
    }

    #[test]
    fn a_parsed_rf_leg_is_an_arc_only_with_its_centre_turn_and_fix() {
        let mut leg = ProcLeg { leg_type: "RF".into(), turn: Some('R'), ..Default::default() };
        assert!(leg.arc().is_none());
        leg.fix = Some(Fix { ident: "END".into(), lat: 47.0, lon: 11.05 });
        assert!(leg.arc().is_none(), "no centre");
        leg.path.arc_center = Some(Fix { ident: "CTR".into(), lat: 47.0, lon: 11.0 });
        let a = leg.arc().expect("arc");
        assert!((a.radius_nm - crate::nav::haversine_nm(47.0, 11.0, 47.0, 11.05)).abs() < 0.01, "from geometry");
        leg.path.arc_radius_nm = Some(2.0);
        assert_eq!(leg.arc().unwrap().radius_nm, 2.0, "the chart's radius first");
        leg.leg_type = "TF".into();
        assert!(leg.arc().is_none());
    }

    fn hold(kind: &str) -> ProcLeg {
        let mut l = ProcLeg {
            leg_type: kind.into(),
            fix_ident: "HLD".into(),
            fix: Some(Fix { ident: "HLD".into(), lat: 47.0, lon: 11.0 }),
            course_mag: Some(360.0),
            turn: Some('R'),
            ..Default::default()
        };
        l.path.hold_leg_min = Some(1.0);
        l
    }

    /// A kinematic aircraft turning at 3 deg/s toward the commanded track,
    /// from 5 nm south of the fix. Returns the end, the circuits, the
    /// distance from the fix at the end and the phases in order.
    fn fly(leg: &ProcLeg, climb_fpm: f64) -> (HoldEnd, u32, f64, Vec<HoldPhase>) {
        let mut h = HoldFlight::for_leg(leg, 0.0).expect("a hold");
        let (mut lat, mut lon) = project_latlon(47.0, 11.0, 180.0, 5.0);
        let (mut trk, mut alt, gs) = (0.0_f64, 5000.0, 180.0);
        let mut phases = vec![h.phase];
        for i in 0..20_000 {
            let s = h.step(leg, lat, lon, trk, gs, alt, 0.5, i as f64);
            if s.moved {
                phases.push(h.phase);
            }
            if let Some(e) = s.end {
                return (e, h.circuits, crate::nav::haversine_nm(lat, lon, 47.0, 11.0), phases);
            }
            trk = (trk + crate::angles::signed_diff_deg(s.track_true, trk).clamp(-3.0, 3.0)).rem_euclid(360.0);
            (lat, lon) = project_latlon(lat, lon, trk, gs / 3600.0);
            alt += climb_fpm / 60.0;
        }
        panic!("the hold never ended: {:?} after {} circuits", h.phase, h.circuits);
    }

    #[test]
    fn a_hold_is_flown_as_a_racetrack_and_left_at_its_fix() {
        let (e, n, d, phases) = fly(&hold("HF"), 0.0);
        assert_eq!((e, n), (HoldEnd::Flown, 1));
        assert!(d < 1.0, "left at the fix, {d} nm");
        use HoldPhase::*;
        assert_eq!(phases, vec![ToFix, TurnOut, Outbound, TurnIn, Inbound]);
        // HA: 500 fpm from 5,000 to 8,000 is six minutes, a circuit or two
        // of about four; left at the fix once made.
        let mut ha = hold("HA");
        ha.alt1_ft = Some(8000.0);
        ha.alt_desc = '+';
        let (e, n, d, _) = fly(&ha, 500.0);
        assert_eq!(e, HoldEnd::Flown);
        assert!((1..=3).contains(&n), "{n} circuits");
        assert!(d < 1.0);
        // One that never makes it gives up at the cap.
        let (e, n, _, _) = fly(&ha, 0.0);
        assert_eq!((e, n), (HoldEnd::GaveUp, HOLD_MAX_CIRCUITS));
        // HM is not flown: the follower goes on at its fix.
        assert!(HoldFlight::for_leg(&hold("HM"), 0.0).is_none());
    }

    /// LOWI's RTT3X from 26: the RF to WI008 round WI009 at 1.4 nm, parsed
    /// and resolved into an arc the follower flies. Needs the X-Plane install.
    /// `cargo test -- --ignored real_install_lowi_rf_arc --nocapture`
    #[test]
    #[ignore]
    fn real_install_lowi_rf_arc() {
        let config = crate::config::Config::from_env().expect("config");
        let apt = crate::nav::find_airport(&config.apt_data_path, "LOWI").unwrap().unwrap();
        let (alat, alon) = apt.position().unwrap();
        let mut legs =
            crate::nav::load_proc_legs(&config.cifp_dir, "LOWI", crate::nav::ProcKind::Sid, "RTT3X", Some("RW26"));
        crate::nav::resolve_proc_fixes(&config.fix_data_path, &config.nav_data_path, &apt, "LOWI", &mut legs, alat, alon);
        let rf = legs.iter().find(|l| l.leg_type == "RF").expect("an RF leg");
        println!("{} centre {:?} r {:?} turn {:?}", rf.fix_ident, rf.path.arc_center, rf.path.arc_radius_nm, rf.turn);
        assert_eq!((rf.fix_ident.as_str(), rf.path.arc_center_ident.as_str()), ("WI008", "WI009"));
        let a = rf.arc().expect("a flyable arc");
        assert!((a.radius_nm - 1.4).abs() < 0.15, "{}", a.radius_nm);
        assert!(!a.right, "a left turn");
        let geo = crate::nav::haversine_nm(a.center_lat, a.center_lon, a.end_lat, a.end_lon);
        assert!((geo - a.radius_nm).abs() < 0.2, "the end fix is on the arc: {geo}");
    }
}
