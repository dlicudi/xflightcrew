//! The recovery planner: from ANY airborne state, a safe path to the plan's
//! destination.
//!
//! LEMG 737, 2026-10-05: after a restart at 2,900 ft, "fly the flight plan"
//! was refused three times in two minutes ("No plan is loaded", "Climb to
//! 9,000 feet first ... outside the 3 mile rejoin corridor", "not abeam any
//! leg") while the aircraft descended toward the hills, and "land the
//! aircraft" self-picked Gibraltar over Malaga, the destination, with the
//! plan not loaded. Getting to the destination was a set of narrow gates
//! that each named a reason to do nothing.
//!
//! # Design
//!
//! One question, one answer, for every way of asking ("fly the flight plan"
//! in the air, "land the aircraft", the pilot's hand-over): WHERE does the
//! safest short way to the destination join, and HOW HIGH must we be to fly
//! it? The plan is the shelf's, else fetched now (the SimBrief plan IS the
//! destination); only a request for the nearest field, or an emergency that
//! makes the destination physically impossible, lands anywhere else.
//!
//! **The join** ([`choose_join`], pure), in order:
//! 1. on the plan's final: the arrival now (`OnFinal`);
//! 2. abeam a leg inside the rejoin corridor: that leg (`Abeam`);
//! 3. near the origin: the first fix (`Departure`);
//! 4. inside the arrival radius of the destination ([`arrival_radius_nm`]):
//!    its arrival or approach (`Arrival`);
//! 5. else DIRECT to the fix whose way on to the destination is shortest:
//!    the distance to it, plus the route from it, plus the turn
//!    ([`TURN_NM_PER_DEG`]) (`Direct`), unless inside autoland's reach that
//!    way is a detour of more than [`ARRIVAL_DETOUR_NM`] over the arrival
//!    direct. Off the route, behind it, beside it: there is always a way in.
//!
//! **The height** ([`climb_first`]): the highest ground on the way to the
//! join (probed, else the grid MORA) plus `procedure::TERRAIN_CLEARANCE_FT`.
//! Below it, the first leg is a climb on a heading whose ground the climb
//! clears ([`climb_leg`], a CIFP "VA" leg the follower already flies),
//! then the join. The follower's own floor ahead watches the rest in flight.
//!
//! **Refused only for the physically impossible** ([`impossible`]), each
//! with its number: the safe altitude over the airframe's ceiling, fuel
//! short of the destination, the destination runway short of the landing
//! distance. For a landing request those, an engine out, or no plan to be
//! had are the emergency: the nearest suitable runway instead, said with why
//! (`tools::full_auto::ResumeArm::Nearest`).

use crate::nav::ProcLeg;
use crate::simbrief::Waypoint;

/// What the pilot asked for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Purpose {
    /// "Fly the flight plan" in the air.
    Resume,
    /// "Land the aircraft".
    Land,
}

/// A resume this near the destination is its arrival (the old rule's 25 nm,
/// `tools::full_auto::ARRIVAL_REJOIN_NM`).
pub const RESUME_ARRIVAL_NM: f64 = 25.0;
/// A landing request this near the destination flies its approach directly:
/// autoland's own search radius.
pub const LAND_ARRIVAL_NM: f64 = 40.0;
/// Inside [`LAND_ARRIVAL_NM`], a route join longer than the direct arrival
/// by more than this is a detour, nm.
pub const ARRIVAL_DETOUR_NM: f64 = 10.0;
/// A resume this near the origin, abeam no leg, joins the first fix.
pub const DEPARTURE_REJOIN_NM: f64 = 15.0;
/// A degree of turn to a direct fix costs this many miles (the self-pick's
/// figure: 180 degrees is six miles).
pub const TURN_NM_PER_DEG: f64 = 1.0 / 30.0;
/// The fuel kept in hand at the destination for the approach, minutes.
pub const FUEL_MARGIN_MIN: f64 = 10.0;
/// The terrain probe looks this far along the join path; beyond, the grid.
pub const PATH_PROBE_NM: f64 = 40.0;
/// Points along the path, one every this many nm.
pub const PATH_SPACING_NM: f64 = 0.25;

/// The arrival radius for a purpose.
pub fn arrival_radius_nm(p: Purpose) -> f64 {
    match p {
        Purpose::Resume => RESUME_ARRIVAL_NM,
        Purpose::Land => LAND_ARRIVAL_NM,
    }
}

/// Where the way to the destination joins.
#[derive(Debug, Clone, PartialEq)]
pub enum Join {
    /// On the plan's final, `along_nm` from the threshold.
    OnFinal { dist_nm: f64, along_nm: f64 },
    /// Abeam the leg ending at this index, inside the corridor.
    Abeam(usize),
    /// Near the origin: the first fix.
    Departure,
    /// The destination's arrival, `dist_nm` away.
    Arrival { dist_nm: f64 },
    /// Direct to the fix at this index: `dist_nm` to it, `via_nm` to the
    /// destination through it.
    Direct {
        index: usize,
        dist_nm: f64,
        via_nm: f64,
    },
}

impl Join {
    pub fn word(&self) -> &'static str {
        match self {
            Join::OnFinal { .. } => "on_final",
            Join::Abeam(_) => "abeam",
            Join::Departure => "departure",
            Join::Arrival { .. } => "arrival",
            Join::Direct { .. } => "direct",
        }
    }

    /// The index of the leg the follower starts at, for a route join.
    pub fn leg_index(&self) -> Option<usize> {
        match self {
            Join::Abeam(i) | Join::Direct { index: i, .. } => Some(*i),
            Join::Departure => Some(0),
            Join::OnFinal { .. } | Join::Arrival { .. } => None,
        }
    }
}

/// The plan's destination waypoint, when the navlog ends at it.
pub fn destination_of<'a>(waypoints: &'a [Waypoint], destination: &str) -> Option<&'a Waypoint> {
    waypoints
        .last()
        .filter(|d| d.ident.eq_ignore_ascii_case(destination))
}

/// The route still to fly from the fix of leg `i` to the destination, nm:
/// fix to fix, then the last fix to the field. Legs without a fix are
/// skipped (a to-altitude leg has no distance of its own). Pure.
pub fn remaining_nm(legs: &[ProcLeg], i: usize, dest: Option<&Waypoint>) -> f64 {
    let fixes: Vec<(f64, f64)> = legs
        .iter()
        .skip(i)
        .filter_map(|l| l.fix.as_ref().map(|f| (f.lat, f.lon)))
        .collect();
    let mut nm = 0.0;
    for w in fixes.windows(2) {
        nm += crate::nav::haversine_nm(w[0].0, w[0].1, w[1].0, w[1].1);
    }
    if let (Some(last), Some(d)) = (fixes.last(), dest) {
        nm += crate::nav::haversine_nm(last.0, last.1, d.lat, d.lon);
    }
    nm
}

/// The direct fix whose way on to the destination is shortest: the distance
/// to it, the route from it and the turn to it. Pure.
pub fn best_direct(
    legs: &[ProcLeg],
    dest: Option<&Waypoint>,
    lat: f64,
    lon: f64,
    track_true: Option<f64>,
) -> Option<Join> {
    legs.iter()
        .enumerate()
        .filter_map(|(i, l)| {
            let f = l.fix.as_ref()?;
            let d = crate::nav::haversine_nm(lat, lon, f.lat, f.lon);
            let turn = track_true
                .filter(|t| t.is_finite())
                .map(|t| {
                    crate::angles::abs_diff_deg(t, crate::nav::bearing_deg(lat, lon, f.lat, f.lon))
                })
                .unwrap_or(0.0);
            let via = d + remaining_nm(legs, i, dest);
            Some((via + turn * TURN_NM_PER_DEG, i, d, via))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, index, dist_nm, via_nm)| Join::Direct {
            index,
            dist_nm,
            via_nm,
        })
}

/// Where the way to the destination joins, from anywhere ([`Join`]'s order).
/// `None` only with no route and no destination to fly to. Pure.
#[allow(clippy::too_many_arguments)]
pub fn choose_join(
    legs: &[ProcLeg],
    waypoints: &[Waypoint],
    destination: &str,
    lat: f64,
    lon: f64,
    mag_var: Option<f64>,
    final_course: Option<&crate::tools::full_auto::FinalCourse>,
    track_true: Option<f64>,
    purpose: Purpose,
) -> Option<Join> {
    if !lat.is_finite() || !lon.is_finite() {
        return None;
    }
    let dest = destination_of(waypoints, destination);
    let to_dest = dest.map(|d| crate::nav::haversine_nm(lat, lon, d.lat, d.lon));
    if let (Some(fc), Some(trk), Some(dist)) = (final_course, track_true, to_dest) {
        if let Some(f) = crate::tools::full_auto::on_final(fc, lat, lon, trk) {
            return Some(Join::OnFinal {
                dist_nm: dist,
                along_nm: f.along_nm,
            });
        }
    }
    if let Ok(i) = crate::interaction::rejoin_index(legs, lat, lon, mag_var) {
        return Some(Join::Abeam(i));
    }
    if !legs.is_empty()
        && waypoints.first().is_some_and(|o| {
            crate::nav::haversine_nm(lat, lon, o.lat, o.lon) <= DEPARTURE_REJOIN_NM
        })
    {
        return Some(Join::Departure);
    }
    if let Some(dist) = to_dest.filter(|d| *d <= arrival_radius_nm(purpose)) {
        return Some(Join::Arrival { dist_nm: dist });
    }
    let direct = best_direct(legs, dest, lat, lon, track_true);
    // Inside autoland's reach, a route that only comes back from behind is
    // a detour: the arrival direct (LIPK from 29 nm past the route, its
    // fixes 40 nm back).
    if let (Some(dist), Some(Join::Direct { via_nm, .. })) = (to_dest, direct.as_ref()) {
        if dist <= LAND_ARRIVAL_NM && dist + ARRIVAL_DETOUR_NM < *via_nm {
            return Some(Join::Arrival { dist_nm: dist });
        }
    }
    direct.or_else(|| to_dest.map(|dist_nm| Join::Arrival { dist_nm }))
}

/// The route still to fly through a join, nm (the fuel's distance).
pub fn to_go_nm(join: &Join, legs: &[ProcLeg], dest: Option<&Waypoint>, lat: f64, lon: f64) -> f64 {
    match join {
        Join::OnFinal { dist_nm, .. } | Join::Arrival { dist_nm } => *dist_nm,
        Join::Direct { via_nm, .. } => *via_nm,
        Join::Abeam(i) => leg_via(legs, *i, dest, lat, lon),
        Join::Departure => leg_via(legs, 0, dest, lat, lon),
    }
}

fn leg_via(legs: &[ProcLeg], i: usize, dest: Option<&Waypoint>, lat: f64, lon: f64) -> f64 {
    let to = legs
        .get(i)
        .and_then(|l| l.fix.as_ref())
        .map(|f| crate::nav::haversine_nm(lat, lon, f.lat, f.lon))
        .unwrap_or(0.0);
    to + remaining_nm(legs, i, dest)
}

/// The point the join's first leg flies to: its fix, or the destination.
pub fn join_point(join: &Join, legs: &[ProcLeg], dest: Option<&Waypoint>) -> Option<(f64, f64)> {
    match join.leg_index() {
        Some(i) => legs
            .iter()
            .skip(i)
            .find_map(|l| l.fix.as_ref().map(|f| (f.lat, f.lon))),
        None => dest.map(|d| (d.lat, d.lon)),
    }
}

/// The points along the way to the join that the terrain probe asks, one
/// every [`PATH_SPACING_NM`] out to [`PATH_PROBE_NM`]. Pure.
pub fn path_points(lat: f64, lon: f64, to_lat: f64, to_lon: f64) -> Vec<(f64, f64)> {
    let dist = crate::nav::haversine_nm(lat, lon, to_lat, to_lon).min(PATH_PROBE_NM);
    let brg = crate::nav::bearing_deg(lat, lon, to_lat, to_lon);
    let n = ((dist / PATH_SPACING_NM).ceil() as usize).max(1);
    crate::safe_hold::ray_points(lat, lon, brg, dist, n)
        .into_iter()
        .map(|(_, la, lo)| (la, lo))
        .collect()
}

/// The altitude to climb to before the join, indicated feet, a round
/// hundred: `None` when the aircraft is already within a hundred feet of
/// the path's floor or above it. Pure.
pub fn climb_first(alt_ft: f64, floor_ind_ft: f64) -> Option<f64> {
    (alt_ft < floor_ind_ft - crate::safe_hold::FLOOR_TOL_FT)
        .then(|| crate::safe_hold::up_100(floor_ind_ft))
}

/// The climb before the join, as the follower flies it: a heading to an
/// altitude (CIFP "VA": no fix, done at or above the altitude).
pub fn climb_leg(course_mag: f64, alt_ft: f64) -> ProcLeg {
    ProcLeg {
        leg_type: "VA".into(),
        fix_ident: String::new(),
        course_mag: Some(course_mag.rem_euclid(360.0)),
        alt_desc: '+',
        alt1_ft: Some(alt_ft),
        ..Default::default()
    }
}

/// What cannot be done, with its number: the safe altitude over the
/// ceiling, fuel short of the destination, a runway short of the landing.
/// `None`: the way is flyable. Pure.
#[allow(clippy::too_many_arguments)]
pub fn impossible(
    destination: &str,
    safe_ft: f64,
    ceiling_ft: f64,
    endurance_min: Option<f64>,
    to_go_nm: f64,
    gs_kts: f64,
    runway: Option<(&str, f64, f64)>,
) -> Option<String> {
    if safe_ft.is_finite() && safe_ft > ceiling_ft {
        return Some(format!(
            "The safe altitude on the way to {destination} is {}, above this aircraft's {} ceiling.",
            crate::flightplan::spoken_feet(safe_ft),
            crate::flightplan::spoken_feet(ceiling_ft)
        ));
    }
    if let Some(e) = endurance_min.filter(|e| e.is_finite()) {
        // The planners' groundspeed floor (`procedure`'s 60 kt): a slow
        // aircraft at 100 kt was promised a third more range than it has.
        let need = to_go_nm / gs_kts.max(60.0) * 60.0;
        if e < need + FUEL_MARGIN_MIN {
            return Some(format!(
                "We have about {e:.0} minutes of fuel, and {destination} is {need:.0} minutes away with {FUEL_MARGIN_MIN:.0} for the approach."
            ));
        }
    }
    if let Some((rwy, have_m, need_m)) = runway {
        if have_m.is_finite() && need_m.is_finite() && have_m < need_m {
            return Some(format!(
                "{destination} runway {} is {have_m:.0} metres; we need {need_m:.0} to land.",
                atc::atis::spoken_runway(rwy)
            ));
        }
    }
    None
}

#[cfg(test)]
mod tests;
