//! The Map tab's route picture, airborne away from a field: the plan's legs
//! (flown ones marked), the active fix, our position as the monitor's tick
//! read it, and the top of descent where the follower or the plan puts it.
//! Pure; the page draws it (no tiles, no library). On the ground or near
//! the drawn field the Map tab is the airport diagram instead (`mode`).

use serde::Serialize;

use crate::efb::airport::AirportOut;
use crate::efb::state::{airborne, Facts};

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct MapFix {
    pub(crate) id: String,
    pub(crate) lat: f64,
    pub(crate) lon: f64,
    /// Behind us.
    pub(crate) done: bool,
    /// The fix the follower steers at.
    pub(crate) active: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct MapPoint {
    pub(crate) lat: f64,
    pub(crate) lon: f64,
    /// Track true, for the aircraft symbol.
    pub(crate) track: Option<f64>,
    pub(crate) label: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct MapOut {
    /// "airport" (the diagram) or "route".
    pub(crate) mode: &'static str,
    pub(crate) fixes: Vec<MapFix>,
    pub(crate) own: Option<MapPoint>,
    pub(crate) tod: Option<MapPoint>,
}

/// A local flat frame around a point, nautical miles east and north.
fn flat(origin: (f64, f64), p: (f64, f64)) -> (f64, f64) {
    let k = origin.0.to_radians().cos();
    ((p.1 - origin.1) * 60.0 * k, (p.0 - origin.0) * 60.0)
}

/// The segment of a (lat, lon) polyline nearest a point, and how far along
/// it the foot falls (0..1). Pure.
pub(crate) fn nearest_segment(path: &[(f64, f64)], p: (f64, f64)) -> Option<(usize, f64)> {
    if path.len() < 2 {
        return None;
    }
    let mut best: Option<(f64, usize, f64)> = None;
    for i in 0..path.len() - 1 {
        let a = flat(p, path[i]);
        let b = flat(p, path[i + 1]);
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let len2 = dx * dx + dy * dy;
        let t = if len2 > 0.0 {
            ((-a.0) * dx + (-a.1) * dy) / len2
        } else {
            0.0
        }
        .clamp(0.0, 1.0);
        let (fx, fy) = (a.0 + t * dx, a.1 + t * dy);
        let d = fx.hypot(fy);
        if best.is_none_or(|b| d < b.0 - 1e-9) {
            best = Some((d, i, t));
        }
    }
    best.map(|(_, i, t)| (i, t))
}

fn lerp(a: (f64, f64), b: (f64, f64), t: f64) -> (f64, f64) {
    (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t)
}

/// The point `nm` along a polyline from segment `i` at fraction `t`, or
/// the end when the line runs out first. Pure.
pub(crate) fn point_along(path: &[(f64, f64)], i: usize, t: f64, nm: f64) -> Option<(f64, f64)> {
    if path.len() < 2 || i + 1 >= path.len() {
        return path.last().copied();
    }
    let mut left = nm.max(0.0);
    let mut from = lerp(path[i], path[i + 1], t.clamp(0.0, 1.0));
    for k in i..path.len() - 1 {
        let to = path[k + 1];
        let d = crate::nav::haversine_nm(from.0, from.1, to.0, to.1);
        if d >= left && d > 0.0 {
            return Some(lerp(from, to, left / d));
        }
        left -= d;
        from = to;
    }
    path.last().copied()
}

/// The map, from the facts and the airport page already built. Pure.
pub(crate) fn build(f: &Facts, a: &AirportOut) -> MapOut {
    let up = airborne(f);
    let near_field = a.own.is_some() && a.status == "ready";
    let plan = f.plan.as_ref();
    let has_route = plan.is_some_and(|p| p.waypoints.len() >= 2);
    let mode = if !up || (near_field && !has_route) || (near_field && a.why == Some("destination"))
    {
        "airport"
    } else if has_route {
        "route"
    } else {
        "airport"
    };
    let path: Vec<(f64, f64)> = plan
        .map(|p| p.waypoints.iter().map(|w| (w.1, w.2)).collect())
        .unwrap_or_default();
    let here = f.tick.as_ref().map(|t| (t.lat, t.lon));
    let active = crate::efb::flight_tab::active_leg_index(f).or_else(|| {
        // No follower: the leg we are on, by where we are.
        here.filter(|_| up)
            .and_then(|p| nearest_segment(&path, p))
            .map(|(i, _)| i + 1)
    });
    let fixes = plan
        .map(|p| {
            p.waypoints
                .iter()
                .enumerate()
                .map(|(i, w)| MapFix {
                    id: w.0.clone(),
                    lat: w.1,
                    lon: w.2,
                    done: active.is_some_and(|a| i < a),
                    active: active == Some(i),
                })
                .collect()
        })
        .unwrap_or_default();
    let own = f.tick.as_ref().map(|t| MapPoint {
        lat: t.lat,
        lon: t.lon,
        track: t.track_deg.or(Some(t.heading_deg)),
        label: None,
    });
    // The follower's own top of descent, ahead of us along the route; else
    // the plan's ring, measured back from the destination.
    let tod = match (f.tod.as_ref(), here) {
        (Some(t), Some(p)) if !t.started && t.held_ft.is_none() && t.to_tod_nm > 0.0 => {
            nearest_segment(&path, p).and_then(|(i, frac)| point_along(&path, i, frac, t.to_tod_nm))
        }
        (None, _) => plan.filter(|p| p.tod_to_dest_nm > 0.0).and_then(|p| {
            let back: Vec<(f64, f64)> = path.iter().rev().copied().collect();
            point_along(&back, 0, 0.0, p.tod_to_dest_nm)
        }),
        _ => None,
    }
    .map(|(lat, lon)| MapPoint {
        lat,
        lon,
        track: None,
        label: Some("TOD".into()),
    });
    MapOut {
        mode,
        fixes,
        own,
        tod,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::efb::state::tests::cruise;

    #[test]
    fn a_point_projects_and_walks_along_the_route() {
        let path = [(0.0, 0.0), (0.0, 1.0), (1.0, 1.0)];
        let (i, t) = nearest_segment(&path, (0.1, 0.5)).unwrap();
        assert_eq!(i, 0);
        assert!((t - 0.5).abs() < 0.01);
        // 30 nm from the middle of the first leg (60 nm long) is its end.
        let p = point_along(&path, 0, 0.5, 30.0).unwrap();
        assert!(p.0.abs() < 1e-6 && (p.1 - 1.0).abs() < 0.01, "{p:?}");
        // Past the end: the end.
        assert_eq!(point_along(&path, 0, 0.0, 999.0), Some((1.0, 1.0)));
    }

    /// In the cruise, away from any field: the route, NORMI active, BELEN
    /// flown, the follower's top of descent ahead of us.
    #[test]
    fn the_cruise_map_is_the_route() {
        let f = cruise();
        let a = crate::efb::airport::page(&f.airport, f.tick.as_ref(), f.plan.as_ref(), true);
        let m = build(&f, &a);
        assert_eq!(m.mode, "route");
        assert!(m.fixes[0].done && !m.fixes[0].active);
        assert!(m.fixes[1].active);
        assert!(!m.fixes[2].done);
        let tod = m.tod.expect("the follower's top of descent");
        assert!(
            tod.lat > 45.0,
            "ahead of the aircraft, north along the route"
        );
        // On the ground: the airport diagram.
        let mut g = cruise();
        g.tick.as_mut().unwrap().on_ground = true;
        let a = crate::efb::airport::page(&g.airport, g.tick.as_ref(), g.plan.as_ref(), false);
        assert_eq!(build(&g, &a).mode, "airport");
    }
}
