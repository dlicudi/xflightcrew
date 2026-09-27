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

fn heading_diff(a: f64, b: f64) -> f64 {
    (a - b + 540.0).rem_euclid(360.0) - 180.0
}

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
            (course - mag_var).rem_euclid(360.0),
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
        };
        idx * 8 + k
    }
}

/// The lateral distance off the course at which a 30 degree cut rolls out
/// onto it: the fly-by lead for radius `r_nm`.
fn roll_out_lead_nm(r_nm: f64) -> f64 {
    r_nm * (1.0 - INTERCEPT_DEG.to_radians().cos())
}

/// The distance along the course an intercept from here consumes, nm: the
/// turn onto the intercept heading, the cut, the roll-out.
pub fn intercept_room_nm(xtk_nm: f64, track_true: f64, course_true: f64, r_nm: f64) -> f64 {
    let heading = course_true - xtk_nm.signum() * INTERCEPT_DEG;
    let turn = heading_diff(heading, track_true).abs().min(90.0);
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
    let tracking_band = if was_track { 2.0 * lead } else { lead };
    if xtk.abs() <= tracking_band {
        let cut = (xtk.abs() / TRACK_AIM_NM)
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
}

impl FinalJoin {
    pub fn flying_pattern(&self) -> bool {
        self.rev.is_some()
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
            self.idx = Some(idx);
            self.rev = None;
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
            if heading_diff(c, to_if).abs() <= IF_MAX_TURN_DEG {
                return Some(Join::Direct);
            }
        }
        if along - intercept_room_nm(xtk, track_true, c, r) >= FAF_ESTABLISHED_NM {
            self.rev = None;
            return Some(Join::Skip { why: "intercept" });
        }
        if self
            .rev
            .is_some_and(|rev| rev.leg == reversal::Leg::Inbound && along < FAF_ESTABLISHED_NM)
        {
            self.rev = None;
            return Some(Join::Skip { why: "out_of_room" });
        }
        // The pattern against the FAF's course line: the base sits two
        // sizing radii beyond the established point, so the turn off base
        // onto the intercept (60 degrees, not the pattern's own 90 onto the
        // line) and the cut fit with FAF_ESTABLISHED_NM to spare.
        let rs = reversal::sizing_radius_nm(gs_kts);
        let join_nm = FAF_ESTABLISHED_NM + 2.0 * rs;
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
    use crate::procedure::{capture_radius_nm, leg_done, leg_turn_deg, HeadingHold};

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
            let err = heading_diff(bug_true, self.hdg);
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
            let off = heading_diff(p.hdg, faf.course_true());
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
            (heading_diff(t, c) - 30.0).abs() < 0.01,
            "from the left: course + 30, got {t:.1}"
        );
        let (lat, lon) = at(10.0, 2.0);
        let (t, _) = course_track(&line, lat, lon, c, 210.0, false).unwrap();
        assert!(
            (heading_diff(t, c) + 30.0).abs() < 0.01,
            "from the right: course - 30"
        );
        // Inside the lead (0.24 nm at 210 kt): the course, nearly.
        let (lat, lon) = at(6.0, -0.1);
        let (t, s) = course_track(&line, lat, lon, c, 210.0, false).unwrap();
        assert_eq!(s, Steer::Track);
        assert!(
            heading_diff(t, c) > 0.0 && heading_diff(t, c) < 5.0,
            "a gentle correction right"
        );
        // Tracking holds to twice the lead.
        let (lat, lon) = at(6.0, -0.35);
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
