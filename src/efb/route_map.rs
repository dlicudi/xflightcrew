//! The Map tab's route picture, airborne away from a field: the plan's legs
//! (flown ones marked), the active fix, our position as the monitor's tick
//! read it, the top of descent where the follower or the plan puts it, the
//! plan's fields, and the view to fit ([`fit`]). Pure; the page draws it
//! over the water of X-Plane's own map data (`efb::basemap`). On the ground
//! or low near the drawn destination the Map tab is the airport diagram
//! instead ([`mode`]).

use serde::Serialize;

use crate::efb::airport::AirportOut;
use crate::efb::state::{airborne, Facts};
use crate::flight::Phase;

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

/// A plan's field, labelled on the map whatever the zoom.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct MapField {
    pub(crate) icao: String,
    /// "origin", "destination" or "alternate".
    pub(crate) role: &'static str,
    pub(crate) lat: f64,
    pub(crate) lon: f64,
}

/// What the route picture fits: a lat/lon box.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct View {
    pub(crate) lat0: f64,
    pub(crate) lon0: f64,
    pub(crate) lat1: f64,
    pub(crate) lon1: f64,
    /// "terminal" (the aircraft and the destination), "ahead" (the aircraft,
    /// the next fix and the legs to fly) or "route" (the whole plan).
    pub(crate) kind: &'static str,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct MapOut {
    /// "airport" (the diagram) or "route".
    pub(crate) mode: &'static str,
    pub(crate) fixes: Vec<MapFix>,
    pub(crate) own: Option<MapPoint>,
    pub(crate) tod: Option<MapPoint>,
    pub(crate) fields: Vec<MapField>,
    pub(crate) view: Option<View>,
}

/// Near the destination: the route map fits the aircraft and the field
/// (a terminal view) within this, ...
pub(crate) const TERMINAL_NM: f64 = 40.0;
/// ... or within this once below [`TERMINAL_LOW_FT`] (a descent into it).
pub(crate) const TERMINAL_LOW_NM: f64 = 60.0;
pub(crate) const TERMINAL_LOW_FT: f64 = 10_000.0;
/// No view is narrower than this, either way.
pub(crate) const MIN_SPAN_NM: f64 = 15.0;
/// The Map tab turns to the destination's diagram below this, inside
/// [`AIRPORT_NM`] of it: the terminal radius where the diagram still draws
/// the aircraft (`airport::OWN_MAX_M`, 30 km).
pub(crate) const AIRPORT_AGL_FT: f64 = 3_000.0;
pub(crate) const AIRPORT_NM: f64 = 15.0;

/// The box round some points, widened to [`MIN_SPAN_NM`] each way. Pure.
fn bbox(pts: &[(f64, f64)], kind: &'static str) -> Option<View> {
    let first = *pts.first()?;
    let (mut a0, mut a1, mut o0, mut o1) = (first.0, first.0, first.1, first.1);
    for &(la, lo) in pts {
        a0 = a0.min(la);
        a1 = a1.max(la);
        o0 = o0.min(lo);
        o1 = o1.max(lo);
    }
    let mid = (a0 + a1) / 2.0;
    let k = mid.to_radians().cos().max(0.05);
    let min_lat = MIN_SPAN_NM / 60.0;
    let min_lon = MIN_SPAN_NM / 60.0 / k;
    if a1 - a0 < min_lat {
        let c = (a0 + a1) / 2.0;
        a0 = c - min_lat / 2.0;
        a1 = c + min_lat / 2.0;
    }
    if o1 - o0 < min_lon {
        let c = (o0 + o1) / 2.0;
        o0 = c - min_lon / 2.0;
        o1 = c + min_lon / 2.0;
    }
    Some(View {
        lat0: a0,
        lon0: o0,
        lat1: a1,
        lon1: o1,
        kind,
    })
}

/// What the route picture fits. Airborne near the destination (within
/// [`TERMINAL_NM`], or [`TERMINAL_LOW_NM`] below [`TERMINAL_LOW_FT`]): the
/// aircraft and the field, never the flown route. Airborne elsewhere: the
/// aircraft, the next fix and the legs still to fly; flown legs are drawn
/// but never drive the fit. On the ground, or with no aircraft: the whole
/// plan. At least [`MIN_SPAN_NM`] across. Pure.
pub(crate) fn fit(
    fixes: &[MapFix],
    own: Option<(f64, f64)>,
    airborne: bool,
    dest: Option<(f64, f64)>,
    alt_ft: Option<f64>,
) -> Option<View> {
    if let (true, Some(here)) = (airborne, own) {
        if let Some(d) = dest {
            let nm = crate::nav::haversine_nm(here.0, here.1, d.0, d.1);
            let low = alt_ft.is_some_and(|a| a < TERMINAL_LOW_FT);
            if nm <= TERMINAL_NM || (low && nm <= TERMINAL_LOW_NM) {
                return bbox(&[here, d], "terminal");
            }
        }
        let mut pts = vec![here];
        pts.extend(fixes.iter().filter(|f| !f.done).map(|f| (f.lat, f.lon)));
        if pts.len() == 1 {
            pts.extend(dest.or(fixes.last().map(|f| (f.lat, f.lon))));
        }
        return bbox(&pts, "ahead");
    }
    let mut pts: Vec<(f64, f64)> = fixes.iter().map(|f| (f.lat, f.lon)).collect();
    pts.extend(own);
    bbox(&pts, "route")
}

/// The destination where the drawn diagram puts it, else the field
/// directory's, else the plan's last waypoint of that name.
fn destination_at(f: &Facts) -> Option<(f64, f64)> {
    let plan = f.plan.as_ref()?;
    let icao = plan.destination.trim();
    if icao.is_empty() {
        return None;
    }
    f.airport
        .diagram
        .as_ref()
        .filter(|d| d.icao.eq_ignore_ascii_case(icao))
        .map(|d| (d.lat0, d.lon0))
        .or_else(|| {
            f.fields
                .iter()
                .find(|x| x.0 == "destination")
                .map(|x| (x.2, x.3))
        })
        .or_else(|| {
            plan.waypoints
                .iter()
                .rev()
                .find(|w| w.0.eq_ignore_ascii_case(icao))
                .map(|w| (w.1, w.2))
        })
}

/// The Map tab's own choice. On the ground: the airport. Airborne: the
/// destination's diagram inside [`AIRPORT_NM`] of it below
/// [`AIRPORT_AGL_FT`] (or in the landing, on the runway), the route
/// otherwise; with no route to draw, the airport. Pure.
pub(crate) fn mode(f: &Facts, a: &AirportOut) -> &'static str {
    if !airborne(f) {
        return "airport";
    }
    let has_route = f.plan.as_ref().is_some_and(|p| p.waypoints.len() >= 2);
    if !has_route {
        return "airport";
    }
    let drawn = a.why == Some("destination") && matches!(a.status, "ready" | "loading");
    let near = match (f.tick.as_ref(), destination_at(f)) {
        (Some(t), Some(d)) => crate::nav::haversine_nm(t.lat, t.lon, d.0, d.1) <= AIRPORT_NM,
        _ => false,
    };
    let low = f.tick.as_ref().is_some_and(|t| t.agl_ft < AIRPORT_AGL_FT);
    let landing = f
        .flight
        .as_ref()
        .is_some_and(|s| matches!(s.phase, Phase::Landing | Phase::Rollout));
    if drawn && near && (low || landing) {
        "airport"
    } else {
        "route"
    }
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
    let plan = f.plan.as_ref();
    let mode = mode(f, a);
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
    let fixes: Vec<MapFix> = plan
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
    let view = fit(
        &fixes,
        here,
        up,
        destination_at(f),
        f.tick.as_ref().map(|t| t.alt_ft),
    );
    let fields = f
        .fields
        .iter()
        .map(|(role, icao, lat, lon)| MapField {
            icao: icao.clone(),
            role,
            lat: *lat,
            lon: *lon,
        })
        .collect();
    MapOut {
        mode,
        fixes,
        own,
        tod,
        fields,
        view,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::efb::state::tests::{approach, cruise};

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

    fn page_of(f: &Facts) -> AirportOut {
        crate::efb::airport::page(
            &f.airport,
            f.tick.as_ref(),
            f.plan.as_ref(),
            crate::efb::state::airborne(f),
        )
    }

    fn contains(v: &View, p: (f64, f64)) -> bool {
        p.0 >= v.lat0 && p.0 <= v.lat1 && p.1 >= v.lon0 && p.1 <= v.lon1
    }

    fn span_nm(v: &View) -> (f64, f64) {
        let k = ((v.lat0 + v.lat1) / 2.0).to_radians().cos();
        ((v.lat1 - v.lat0) * 60.0, (v.lon1 - v.lon0) * 60.0 * k)
    }

    /// The cruise: the aircraft, the next fix and the legs to fly; the flown
    /// BELEN is drawn but stays out of the fit.
    #[test]
    fn the_cruise_fits_what_is_ahead() {
        let f = cruise();
        let m = build(&f, &page_of(&f));
        let v = m.view.expect("a view");
        assert_eq!(v.kind, "ahead");
        assert!(contains(&v, (45.0, -1.0)), "the aircraft");
        assert!(contains(&v, (46.0, -0.8)), "NORMI, the next fix");
        assert!(contains(&v, (51.15, -0.19)), "the destination");
        assert!(!contains(&v, (37.5, -4.5)), "BELEN is flown: {v:?}");
        assert_eq!(m.fields.len(), 3);
        assert!(m
            .fields
            .iter()
            .any(|x| x.role == "alternate" && x.icao == "EGLL"));
    }

    /// 8 nm out of EGKK at 2,040 ft, only the destination left (the
    /// pilot's screenshot fitted the whole 1,000 nm route): a terminal view,
    /// the aircraft and the field, never the flown route.
    #[test]
    fn the_last_fix_near_the_field_fits_the_terminal() {
        let f = approach();
        let m = build(&f, &page_of(&f));
        let v = m.view.expect("a view");
        assert_eq!(v.kind, "terminal");
        assert!(contains(&v, (51.12, -0.40)) && contains(&v, (51.148, -0.19)));
        let (h, w) = span_nm(&v);
        assert!(
            h >= MIN_SPAN_NM - 0.01 && w >= MIN_SPAN_NM - 0.01,
            "{h} x {w}"
        );
        assert!(h < 40.0 && w < 40.0, "{h} x {w}");
        assert!(!contains(&v, (50.6, -0.5)), "BOGNA is flown");
        // Descending through 9,000 ft 55 nm out is the terminal too; at
        // 20,000 ft it is still what is ahead.
        let mut g = approach();
        let t = g.tick.as_mut().unwrap();
        t.lat = 51.15;
        t.lon = -1.65;
        t.alt_ft = 9_000.0;
        assert_eq!(build(&g, &page_of(&g)).view.unwrap().kind, "terminal");
        g.tick.as_mut().unwrap().alt_ft = 20_000.0;
        assert_eq!(build(&g, &page_of(&g)).view.unwrap().kind, "ahead");
    }

    /// On the ground: the whole plan.
    #[test]
    fn the_ground_fits_the_whole_plan() {
        let mut f = cruise();
        let t = f.tick.as_mut().unwrap();
        t.on_ground = true;
        t.lat = 36.675;
        t.lon = -4.499;
        f.active_fix = None;
        let v = build(&f, &page_of(&f)).view.unwrap();
        assert_eq!(v.kind, "route");
        assert!(contains(&v, (37.5, -4.5)) && contains(&v, (51.15, -0.19)));
    }

    /// The Map tab's own choice: the diagram on the ground and low near the
    /// destination, the route above or away.
    #[test]
    fn the_map_turns_to_the_field_low_near_it() {
        // 8 nm out at 1,840 ft above the field: the diagram.
        let f = approach();
        assert_eq!(mode(&f, &page_of(&f)), "airport");
        // The same place at 6,000 ft: the route.
        let mut g = approach();
        g.tick.as_mut().unwrap().agl_ft = 6_000.0;
        assert_eq!(mode(&g, &page_of(&g)), "route");
        // In the landing, whatever the height reads: the diagram.
        g.flight.as_mut().unwrap().phase = Phase::Landing;
        assert_eq!(mode(&g, &page_of(&g)), "airport");
        // Low but 25 nm away (past where the diagram draws us), or 50: the
        // route.
        let mut far = approach();
        far.tick.as_mut().unwrap().lon = -0.85;
        assert_eq!(mode(&far, &page_of(&far)), "route");
        far.tick.as_mut().unwrap().lon = -1.5;
        assert_eq!(mode(&far, &page_of(&far)), "route");
        // On the ground anywhere: the airport.
        let mut gnd = cruise();
        gnd.tick.as_mut().unwrap().on_ground = true;
        assert_eq!(mode(&gnd, &page_of(&gnd)), "airport");
        // Airborne with no route to draw: the airport.
        let mut none = approach();
        none.plan = None;
        assert_eq!(mode(&none, &page_of(&none)), "airport");
        // The cruise: the route.
        let c = cruise();
        assert_eq!(mode(&c, &page_of(&c)), "route");
    }
}
