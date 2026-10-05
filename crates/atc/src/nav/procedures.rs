//! Terminal procedures (SID / STAR / approach) from X-Plane's CIFP.

use super::*;

// ===========================================================================
// Terminal procedures (SID / STAR / approach) from X-Plane's CIFP.
//
// Each `CIFP/<ICAO>.dat` holds ARINC-424 procedure records, one leg per line:
//   SID:seq,rtype,IDENT,TRANS,FIX,rgn,sec,sub,desc,...,PT,turn,recnav,...,
//        theta,rho,COURSE,DIST, altdesc,alt1,alt2, ...
// Comma-split field indices, verified against real EGLL/LEMG records (and the
// same 22/23/24 altitude columns `ils_platform_alt_ft` already relies on):
//   [0]  record type + sequence ("APPCH:020")
//   [2]  procedure ident       (approach "I27R"/"R27R", SID/STAR name "BLN4L")
//   [3]  transition ident      ("RW30" runway transition, a fix for an enroute
//                               transition, or BLANK for the common/final part)
//   [4]  fix ident             ("MG930", "RW27R", "" for the to-altitude legs)
//   [5]  fix ICAO region       ("LE", "EG") — disambiguates duplicate idents
//   [8]  description code      (4th char 'F' = final approach fix)
//   [11] path & terminator     (IF/TF/CF/DF/RF fix legs; CA/VA/FA to-altitude;
//                               FM/VM manual; HM/HA/HF holds)
//   [12] turn direction        (L/R)
//   [20] magnetic course ×10   (2680 → 268.0°) — set on course/heading legs
//   [21] distance ×10, nm      (0025 → 2.5)
//   [22] altitude descriptor   (+ at-or-above, - at-or-below, blank/@ at, B/G/I ..)
//   [23] altitude 1, ft (or FLxxx)
//   [24] altitude 2, ft
// Speed limits live further right at an index that shifts between record shapes,
// so they are deliberately NOT parsed — the follower flies a fixed profile speed.
// ===========================================================================

/// Which family of terminal procedure — the CIFP record prefix.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProcKind {
    /// Standard Instrument Departure.
    Sid,
    /// Standard Terminal Arrival Route.
    Star,
    /// Instrument approach.
    Appch,
}

impl ProcKind {
    fn prefix(self) -> &'static str {
        match self {
            ProcKind::Sid => "SID:",
            ProcKind::Star => "STAR:",
            ProcKind::Appch => "APPCH:",
        }
    }
}

/// A navigation fix resolved to a position.
#[derive(Debug, Clone, PartialEq)]
pub struct Fix {
    pub ident: String,
    pub lat: f64,
    pub lon: f64,
}

/// One leg of a terminal procedure, parsed from a CIFP record. `fix` is `None`
/// until [`resolve_proc_fixes`] fills it (and stays `None` for to-altitude legs
/// that have no fix at all).
#[derive(Debug, Clone, Default)]
pub struct ProcLeg {
    pub seq: u32,
    /// Transition/runway this leg belongs to (field 3): "RW30", a transition
    /// fix name, or empty for the common/final portion.
    pub transition: String,
    /// Path & terminator (field 11): IF, TF, CF, DF, CA, VA, FM, RF, HM, ...
    pub leg_type: String,
    /// Fix ident (field 4): a waypoint/navaid, "RW27R" for the threshold, or
    /// empty on a to-altitude leg.
    pub fix_ident: String,
    /// Fix ICAO region (field 5) — narrows duplicate idents at resolve time.
    pub fix_region: String,
    pub fix: Option<Fix>,
    /// Magnetic course for course/heading legs (field 20), degrees; `None` on
    /// pure track/direct legs.
    pub course_mag: Option<f64>,
    /// Leg distance in nm (field 21), where the record carries one.
    pub dist_nm: Option<f64>,
    pub turn: Option<char>,
    /// Altitude descriptor (field 22): '+' at-or-above, '-' at-or-below, else at.
    pub alt_desc: char,
    pub alt1_ft: Option<f64>,
    pub alt2_ft: Option<f64>,
    /// True when this leg's fix is the final approach fix.
    pub is_faf: bool,
    /// The missed approach point (description code M): an approach that ends
    /// here, short of the runway, has a visual segment to fly (LSZS).
    pub is_map: bool,
    /// The published vertical path angle, degrees (field 28: "-449" = 4.49).
    pub vpa_deg: Option<f64>,
    /// The recommended navaid (field 13): what a DME-terminated leg (CD, VD,
    /// FD) measures its distance from. Empty when the record names none.
    pub navaid_ident: String,
    /// Its ICAO region (field 14), which narrows namesakes at resolve time.
    pub navaid_region: String,
    /// That navaid resolved, once [`resolve_proc_fixes`] has run.
    pub navaid: Option<Fix>,
    /// The leg's shape beyond its fix: an RF arc's centre and radius, a
    /// holding leg's length.
    pub path: LegPath,
}

/// What an RF or holding leg carries besides its fix (issue #101).
#[derive(Debug, Clone, Default)]
pub struct LegPath {
    /// RF: the arc's centre fix (fields 30 and 31) and radius, nm (field 17,
    /// thousandths: LOWI's "001400" is 1.4 nm).
    pub arc_center_ident: String,
    pub arc_center_region: String,
    /// The centre resolved, once [`resolve_proc_fixes`] has run.
    pub arc_center: Option<Fix>,
    pub arc_radius_nm: Option<f64>,
    /// HM / HA / HF: the inbound leg's time, minutes (field 21, "T010" = 1.0
    /// min); a leg coded by distance keeps it in `dist_nm`.
    pub hold_leg_min: Option<f64>,
}

/// An RF leg the follower can fly: its fix, its centre and its radius known.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Arc {
    pub center_lat: f64,
    pub center_lon: f64,
    pub radius_nm: f64,
    /// 'R' clockwise, 'L' anticlockwise, seen from above.
    pub right: bool,
    pub end_lat: f64,
    pub end_lon: f64,
}

impl ProcLeg {
    /// This leg as a flyable RF arc: `None` for any other leg, or an RF whose
    /// centre, radius, turn or end fix is unknown (flown as before).
    pub fn arc(&self) -> Option<Arc> {
        if self.leg_type != "RF" {
            return None;
        }
        let c = self.path.arc_center.as_ref()?;
        let f = self.fix.as_ref()?;
        let right = match self.turn? {
            'R' => true,
            'L' => false,
            _ => return None,
        };
        // The radius from the chart, else from the geometry (the end fix is
        // on the arc by construction).
        let radius_nm = self
            .path
            .arc_radius_nm
            .filter(|r| *r > 0.05)
            .unwrap_or_else(|| haversine_nm(c.lat, c.lon, f.lat, f.lon));
        (radius_nm > 0.05).then_some(Arc {
            center_lat: c.lat,
            center_lon: c.lon,
            radius_nm,
            right,
            end_lat: f.lat,
            end_lon: f.lon,
        })
    }

    /// A holding leg (HM manual, HA to an altitude, HF one circuit).
    pub fn is_hold(&self) -> bool {
        matches!(self.leg_type.as_str(), "HM" | "HA" | "HF")
    }
}

/// A CIFP distance/time cell: "T010" is 1.0 minute (a hold's leg time),
/// anything else tenths of a mile. Returns (nm, minutes).
pub(super) fn arinc_dist_or_time(s: &str) -> (Option<f64>, Option<f64>) {
    let s = s.trim();
    match s.strip_prefix('T') {
        Some(t) => (None, arinc_tenths(t)),
        None => (arinc_tenths(s), None),
    }
}

pub(super) fn cifp_field<'a>(f: &[&'a str], i: usize) -> &'a str {
    f.get(i).map(|s| s.trim()).unwrap_or("")
}

/// ARINC altitude cell → feet MSL. Handles "FLxxx" and zero-padded numbers;
/// blank / zero → `None`.
pub(super) fn arinc_alt_ft(s: &str) -> Option<f64> {
    let s = s.trim();
    if let Some(fl) = s.strip_prefix("FL") {
        return fl.parse::<f64>().ok().map(|f| f * 100.0);
    }
    s.parse::<f64>().ok().filter(|a| *a > 0.0)
}

/// A CIFP "×10" cell (course, distance) → its real value; blank → `None`.
pub(super) fn arinc_tenths(s: &str) -> Option<f64> {
    let s = s.trim();
    (!s.is_empty())
        .then(|| s.parse::<f64>().ok())
        .flatten()
        .map(|v| v / 10.0)
}

/// Parse the legs of one procedure from CIFP lines. `transition` selects which
/// legs to keep: `None` keeps only the common/final portion (blank field 3) —
/// the right choice for an approach's final segment; `Some(t)` also keeps legs
/// whose transition is `t` — e.g. `Some("RW30")` for a SID off runway 30.
/// Legs come back transition-first, then common, each ordered by sequence.
pub fn parse_proc_legs<'a>(
    lines: impl Iterator<Item = &'a str>,
    kind: ProcKind,
    ident: &str,
    transition: Option<&str>,
) -> Vec<ProcLeg> {
    let prefix = kind.prefix();
    let mut out: Vec<ProcLeg> = Vec::new();
    for line in lines {
        if !line.starts_with(prefix) {
            continue;
        }
        let f: Vec<&str> = line.split(',').collect();
        if f.len() < 12 || cifp_field(&f, 2) != ident {
            continue;
        }
        let tr = cifp_field(&f, 3).to_string();
        match transition {
            None if !tr.is_empty() => continue,
            Some(want) if !tr.is_empty() && tr != want => continue,
            _ => {}
        }
        let seq = f[0]
            .split(':')
            .nth(1)
            .and_then(|s| s.trim().parse().ok())
            .unwrap_or(0);
        let leg_type = cifp_field(&f, 11).to_string();
        let (dist_nm, leg_min) = arinc_dist_or_time(cifp_field(&f, 21));
        let rf = leg_type == "RF";
        out.push(ProcLeg {
            seq,
            transition: tr,
            path: LegPath {
                arc_center_ident: if rf { cifp_field(&f, 30).to_string() } else { String::new() },
                arc_center_region: if rf { cifp_field(&f, 31).to_string() } else { String::new() },
                arc_center: None,
                arc_radius_nm: if rf {
                    cifp_field(&f, 17).parse::<f64>().ok().map(|r| r / 1000.0).filter(|r| *r > 0.0)
                } else {
                    None
                },
                hold_leg_min: leg_min,
            },
            leg_type,
            fix_ident: cifp_field(&f, 4).to_string(),
            fix_region: cifp_field(&f, 5).to_string(),
            fix: None,
            course_mag: arinc_tenths(cifp_field(&f, 20)),
            dist_nm,
            // The turn direction is field 9; 12 is only its "valid" flag
            // (LOWI R26-Z's RF leg: "L" in 9, blank in 12).
            turn: cifp_field(&f, 9)
                .chars()
                .next()
                .filter(|c| *c == 'L' || *c == 'R'),
            alt_desc: cifp_field(&f, 22).chars().next().unwrap_or(' '),
            alt1_ft: arinc_alt_ft(cifp_field(&f, 23)),
            alt2_ft: arinc_alt_ft(cifp_field(&f, 24)),
            is_faf: cifp_field(&f, 8).chars().nth(3) == Some('F'),
            is_map: cifp_field(&f, 8).chars().nth(3) == Some('M'),
            vpa_deg: cifp_field(&f, 28)
                .parse::<f64>()
                .ok()
                .map(|v| v.abs() / 100.0)
                .filter(|v| (1.0..=8.0).contains(v)),
            navaid_ident: cifp_field(&f, 13).to_string(),
            navaid_region: cifp_field(&f, 14).to_string(),
            navaid: None,
        });
    }
    // Transition (runway) legs first, then the common portion, each by sequence.
    out.sort_by_key(|l| (l.transition.is_empty(), l.seq));
    out
}

/// Read one procedure's legs from `CIFP/<ICAO>.dat`. See [`parse_proc_legs`]
/// for `transition` semantics. Empty vec = no such file/procedure.
pub fn load_proc_legs(
    cifp_dir: &str,
    icao: &str,
    kind: ProcKind,
    ident: &str,
    transition: Option<&str>,
) -> Vec<ProcLeg> {
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let Ok(text) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    parse_proc_legs(text.lines(), kind, ident, transition)
}

/// Every distinct procedure ident of `kind` at `icao`, in first-seen order.
pub fn list_procedures(cifp_dir: &str, icao: &str, kind: ProcKind) -> Vec<String> {
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let Ok(text) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    let prefix = kind.prefix();
    let mut out: Vec<String> = Vec::new();
    for line in text.lines() {
        if !line.starts_with(prefix) {
            continue;
        }
        let f: Vec<&str> = line.split(',').collect();
        let id = cifp_field(&f, 2);
        if !id.is_empty() && !out.iter().any(|s| s == id) {
            out.push(id.to_string());
        }
    }
    out
}

/// The runway an approach ident serves: strip the leading type letter(s) and any
/// "-Y"/"-Z" variant suffix. "I27R" → "27R", "R27R-Z" → "27R", "I20-Y" → "20".
/// `None` if there are no runway digits.
pub fn approach_runway(ident: &str) -> Option<String> {
    let body: String = ident
        .chars()
        .skip_while(|c| c.is_ascii_alphabetic())
        .collect();
    let body = body.split('-').next().unwrap_or("");
    let digits: String = body.chars().take_while(|c| c.is_ascii_digit()).collect();
    if digits.is_empty() {
        return None;
    }
    let suffix: String = body
        .chars()
        .skip_while(|c| c.is_ascii_digit())
        .take(1)
        .filter(|c| matches!(c, 'L' | 'R' | 'C'))
        .collect();
    Some(format!("{digits}{suffix}"))
}

/// Whether an approach/runway designator matches `runway` (e.g. "27R"): same
/// leading number, and same L/R/C side when both name one.
pub(super) fn runway_matches(a: &str, b: &str) -> bool {
    if runway_number(a) != runway_number(b) {
        return false;
    }
    let side = |s: &str| {
        s.chars()
            .find(|c| matches!(c, 'L' | 'R' | 'C' | 'l' | 'r' | 'c'))
    };
    match (side(a), side(b)) {
        (Some(x), Some(y)) => x.eq_ignore_ascii_case(&y),
        _ => true, // one side unspecified → the number match is enough
    }
}

/// The best instrument approach ident for `runway` at `icao`: prefer ILS ('I'),
/// then RNAV/GPS ('R'/'X'/'Q'), then LOC ('L'), then anything else. `None` if the
/// field publishes no approach to that runway.
pub fn best_approach(cifp_dir: &str, icao: &str, runway: &str) -> Option<String> {
    approach_candidates(cifp_dir, icao, runway).into_iter().next()
}

/// Every approach to `runway` the follower can fly, best first: by class
/// (ILS, RNAV, LOC, VOR), a straight-in to the threshold ahead of one that
/// ends at a missed approach point with a visual segment (`map_final_legs`),
/// then file order. Only an approach that ENDS on this runway or at a MAP on
/// straight legs qualifies: LOWI's RNAV 08 V and 26 E are circling procedures
/// with no runway fix, 08 Z turns onto the runway on RF arcs, and picked by
/// rank alone the circling one won and its missed approach was flown as the
/// approach (2026-09-24). The geometry of a visual segment is the caller's to
/// judge once the fixes are resolved (`visual_segment`).
pub fn approach_candidates(cifp_dir: &str, icao: &str, runway: &str) -> Vec<String> {
    let rank = |id: &str| match id.chars().next() {
        Some('I') => 0,
        Some('X') | Some('R') | Some('Q') => 1,
        Some('L') => 2,
        Some('D') | Some('S') | Some('V') => 3,
        _ => 4,
    };
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let Ok(text) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    let mut out: Vec<(u8, u8, String)> = list_procedures(cifp_dir, icao, ProcKind::Appch)
        .into_iter()
        .filter(|id| approach_runway(id).is_some_and(|r| runway_matches(&r, runway)))
        .filter_map(|id| {
            let legs = parse_proc_legs(text.lines(), ProcKind::Appch, &id, None);
            if straight_in_legs(legs.clone(), runway).is_some() {
                Some((rank(&id), 0, id))
            } else if map_final_legs(legs).is_some() {
                Some((rank(&id), 1, id))
            } else {
                None
            }
        })
        .collect();
    out.sort_by_key(|(r, m, _)| (*r, *m));
    out.into_iter().map(|(_, _, id)| id).collect()
}

/// An approach that ends at a missed approach point SHORT of the runway (no
/// runway fix before it), legs as far as the MAP, with no RF arc before it.
/// LSZS's RNAV 03 and 21: a 4.5 degree final to a MAP about two miles out,
/// then a visual segment. `None` otherwise.
pub fn map_final_legs(legs: Vec<ProcLeg>) -> Option<Vec<ProcLeg>> {
    let at = legs.iter().position(|l| l.is_map || l.fix_ident.starts_with("RW"))?;
    if !legs[at].is_map || legs[..=at].iter().any(|l| l.leg_type == "RF") {
        return None;
    }
    let mut legs = legs;
    legs.truncate(at + 1);
    Some(legs)
}

/// The visual segment from a MAP to the threshold: how far, how far off the
/// extended centreline, how far the final course is from the runway's, and
/// the angle down to the threshold.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VisualSegment {
    pub dist_nm: f64,
    pub lateral_nm: f64,
    pub course_diff_deg: f64,
    pub angle_deg: f64,
}

/// A visual segment the landing can fly from: MAP within this of the threshold...
pub const VISUAL_MAX_NM: f64 = 3.0;
/// ...this close to the extended centreline...
pub const VISUAL_MAX_LATERAL_NM: f64 = 0.4;
/// ...on a final course within this of the runway's (straight-in criteria)...
pub const VISUAL_MAX_COURSE_DEG: f64 = 30.0;
/// ...and no steeper than this down to the threshold.
pub const VISUAL_MAX_ANGLE_DEG: f64 = 5.0;

/// Judge the visual segment of resolved `legs` (ending at the MAP) onto the
/// threshold at `thr`, runway course `course_deg` true, elevation `thr_ft`.
pub fn visual_segment(
    legs: &[ProcLeg],
    thr: (f64, f64),
    course_deg: f64,
    thr_ft: f64,
) -> Option<VisualSegment> {
    let map = legs.last()?;
    let mf = map.fix.as_ref()?;
    let map_ft = map.alt1_ft?;
    let prev = legs[..legs.len() - 1].iter().rev().find_map(|l| l.fix.as_ref())?;
    let final_course = bearing_deg(prev.lat, prev.lon, mf.lat, mf.lon);
    let dist_nm = haversine_nm(mf.lat, mf.lon, thr.0, thr.1);
    // Position relative to the runway: the MAP lies BEFORE the threshold on
    // the approach side, so from the threshold it bears the reciprocal.
    let from_thr = bearing_deg(thr.0, thr.1, mf.lat, mf.lon);
    let off = sim::angles::signed_diff_deg(from_thr, (course_deg + 180.0).rem_euclid(360.0));
    let lateral_nm = dist_nm * off.to_radians().sin().abs();
    let height_ft = map_ft - thr_ft;
    Some(VisualSegment {
        dist_nm,
        lateral_nm,
        course_diff_deg: sim::angles::abs_diff_deg(final_course, course_deg),
        angle_deg: (height_ft / (dist_nm * 6076.12)).atan().to_degrees(),
    })
}

impl VisualSegment {
    /// Flyable as a straight-in visual segment by the landing.
    pub fn flyable(&self) -> bool {
        self.dist_nm <= VISUAL_MAX_NM
            && self.lateral_nm <= VISUAL_MAX_LATERAL_NM
            && self.course_diff_deg <= VISUAL_MAX_COURSE_DEG
            && (2.5..=VISUAL_MAX_ANGLE_DEG).contains(&self.angle_deg)
    }
}

/// The named transitions (route type A) of approach `ident`, file order.
pub fn approach_transitions(cifp_dir: &str, icao: &str, ident: &str) -> Vec<String> {
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let Ok(text) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    let mut out: Vec<String> = Vec::new();
    for line in text.lines() {
        let Some(body) = line.strip_prefix("APPCH:") else { continue };
        let f: Vec<&str> = body.split(',').collect();
        if cifp_field(&f, 1) == "A" && cifp_field(&f, 2) == ident {
            let t = cifp_field(&f, 3).to_string();
            if !t.is_empty() && !out.contains(&t) {
                out.push(t);
            }
        }
    }
    out
}

/// An approach's legs as far as the runway threshold, when it is a straight-in
/// the follower can fly: a runway fix for THIS runway (exact designator, so a
/// grass "26R" never inherits a "26" approach), and no RF (constant-radius
/// arc) leg before it, which the follower would chord across. The missed
/// approach after the threshold is dropped: it is never the approach.
/// `None` for a circling procedure or an arc.
pub fn straight_in_legs(legs: Vec<ProcLeg>, runway: &str) -> Option<Vec<ProcLeg>> {
    let at = legs.iter().position(|l| {
        l.fix_ident
            .strip_prefix("RW")
            .is_some_and(|r| r.eq_ignore_ascii_case(runway) || (runway_number(r) == runway_number(runway) && side_letter(r) == side_letter(runway)))
    })?;
    if legs[..at].iter().any(|l| l.leg_type == "RF") {
        return None;
    }
    let mut legs = legs;
    legs.truncate(at + 1);
    Some(legs)
}

pub(super) fn side_letter(rwy: &str) -> Option<char> {
    rwy.chars().rev().find(|c| matches!(c, 'L' | 'R' | 'C'))
}
