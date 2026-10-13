//! The Map tab's live picture in lat/lon, for one continuous world map
//! (`map.js`, Web Mercator): the plan's legs (flown ones marked), the active
//! fix, our aircraft as the monitor's tick read it, our traffic, every
//! field's stands free or taken, the top of descent where the follower or
//! the plan puts it, the plan's fields, and the overview to fit ([`fit`]).
//! Pure; the page draws it over the water of X-Plane's own map data
//! (`efb::basemap`) and any field's diagram (`efb::airport`) once close.

use serde::Serialize;

use crate::efb::airport::{AirportOut, StandsOut, TrafficOut};
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
    pub(crate) label: Option<String>,
}

/// Our aircraft, as the monitor's tick read it.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct OwnOut {
    pub(crate) lat: f64,
    pub(crate) lon: f64,
    /// True heading (the monitor's `psi`, best-effort: 0 on a failed read).
    pub(crate) heading: f64,
    /// Ground track, true, when the read held.
    pub(crate) track: Option<f64>,
    pub(crate) gs_kts: f64,
    pub(crate) on_ground: bool,
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

/// The overview the map's button (and its first airborne view) fits: a
/// lat/lon box.
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
    pub(crate) fixes: Vec<MapFix>,
    pub(crate) own: Option<OwnOut>,
    pub(crate) tod: Option<MapPoint>,
    pub(crate) fields: Vec<MapField>,
    pub(crate) view: Option<View>,
    /// Our traffic, every simulated field's (`airport::traffic_out`).
    pub(crate) traffic: Vec<TrafficOut>,
    /// The stands of every field anybody knows of, free or taken
    /// (`airport::stands_by_field`).
    pub(crate) stands: Vec<StandsOut>,
}

/// Near the destination: the route map fits the aircraft and the field
/// (a terminal view) within this, ...
pub(crate) const TERMINAL_NM: f64 = 40.0;
/// ... or within this once below [`TERMINAL_LOW_FT`] (a descent into it).
pub(crate) const TERMINAL_LOW_NM: f64 = 60.0;
pub(crate) const TERMINAL_LOW_FT: f64 = 10_000.0;
/// No view is narrower than this, either way.
pub(crate) const MIN_SPAN_NM: f64 = 15.0;

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
    let r1 = |v: f64| (v * 10.0).round() / 10.0;
    let own = f.tick.as_ref().map(|t| OwnOut {
        lat: (t.lat * 1e7).round() / 1e7,
        lon: (t.lon * 1e7).round() / 1e7,
        heading: r1(t.heading_deg.rem_euclid(360.0)),
        track: t.track_deg.map(|v| r1(v.rem_euclid(360.0))),
        gs_kts: r1(t.gs_kts.abs()),
        on_ground: t.on_ground,
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
    let traffic = f.airport.traffic.as_deref();
    // The wanted field's stands take our own aircraft standing on one.
    let here = f
        .airport
        .diagram
        .as_ref()
        .filter(|d| a.status == "ready" && a.icao.as_deref() == Some(d.icao.as_str()))
        .map(|d| (d, a.own_on_ground));
    MapOut {
        fixes,
        own,
        tod,
        fields,
        view,
        traffic: traffic
            .map(crate::efb::airport::traffic_out)
            .unwrap_or_default(),
        stands: crate::efb::airport::stands_by_field(traffic, here),
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

    /// In the cruise, away from any field: NORMI active, BELEN flown, the
    /// follower's top of descent ahead of us, the aircraft in lat/lon.
    #[test]
    fn the_cruise_map_marks_the_route() {
        let f = cruise();
        let a = crate::efb::airport::page(&f.airport, f.tick.as_ref(), f.plan.as_ref(), true);
        let m = build(&f, &a);
        let own = m.own.as_ref().expect("the aircraft");
        assert!(!own.on_ground && (own.lat - 45.0).abs() < 1e-6);
        assert!(m.fixes[0].done && !m.fixes[0].active);
        assert!(m.fixes[1].active);
        assert!(!m.fixes[2].done);
        let tod = m.tod.expect("the follower's top of descent");
        assert!(
            tod.lat > 45.0,
            "ahead of the aircraft, north along the route"
        );
        assert!(
            m.traffic.is_empty() && m.stands.is_empty(),
            "no traffic world"
        );
    }

    fn traffic_ac(id: u32, lat: f64, lon: f64) -> crate::efb::feed::TrafficAc {
        crate::efb::feed::TrafficAc {
            id,
            callsign: format!("EZY{id}"),
            spoken: format!("Easy {id}"),
            airline: "easyJet".into(),
            icao_type: "A320".into(),
            from: "LEMG".into(),
            to: "EGKK".into(),
            phase: "inbound",
            stand: None,
            sched: None,
            est: None,
            lat,
            lon,
            hdg: 10.0,
            alt_ft: 9_000.0,
            on_ground: false,
            gs_kts: 280.0,
        }
    }

    /// One world: our traffic is on the map wherever it flies (a field 600
    /// nm away too), and every simulated field's stands are known, the
    /// wanted field's with our own aircraft on one.
    #[test]
    fn traffic_and_stands_ride_the_map_wherever_they_are() {
        let mut f = approach();
        f.airport.traffic = Some(vec![
            crate::efb::feed::TrafficField {
                icao: "EGKK".into(),
                aircraft: vec![traffic_ac(1, 51.2, -0.1)],
                taken: vec![crate::efb::feed::StandTaken {
                    name: "12".into(),
                    by: "EZY1".into(),
                    how: "due",
                }],
            },
            crate::efb::feed::TrafficField {
                icao: "LEMG".into(),
                aircraft: vec![traffic_ac(2, 36.7, -4.5)],
                taken: Vec::new(),
            },
        ]);
        let m = build(&f, &page_of(&f));
        assert_eq!(m.traffic.len(), 2, "both fields', however far");
        assert!((m.traffic[1].lat - 36.7).abs() < 1e-9);
        let icaos: Vec<&str> = m.stands.iter().map(|s| s.icao.as_str()).collect();
        assert_eq!(icaos, vec!["LEMG", "EGKK"]);
        assert!(m.stands.iter().all(|s| s.known));
        // On the ground on Gatwick's stand 12's spot: ours, never listed twice.
        let t = f.tick.as_mut().unwrap();
        t.on_ground = true;
        t.lat = 51.148;
        t.lon = -0.19;
        f.airport.diagram.as_mut().unwrap().stands = vec![("12".into(), 1.0, 1.0)];
        let mut a = page_of(&f);
        a.icao = Some("EGKK".into());
        let m = build(&f, &a);
        let kk = m.stands.iter().find(|s| s.icao == "EGKK").unwrap();
        assert_eq!(kk.taken.len(), 1);
        assert_eq!(kk.taken[0].by, "us");
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

    /// The page's projection (`map.js`, run under node where it is
    /// installed, else skipped): Web Mercator round-trips; a field's frame
    /// placed with one scale lands within a centimetre of where the
    /// server's `Frame` puts the point, 4 km out at 60 degrees north; the
    /// basemap box keeps within the server's tile limit, and a continent
    /// asks for none.
    #[test]
    fn the_page_places_a_field_where_the_server_measured_it() {
        let js = concat!(env!("CARGO_MANIFEST_DIR"), "/src/efb/map.js");
        let script = format!(
            r#"global.window = {{}};
            eval(require('fs').readFileSync({js:?}, 'utf8'));
            const M = window.XvlMapMath, out = {{}};
            out.lat = M.latOf(M.my(51.148));
            out.lon = M.lonOf(M.mx(-0.19));
            const lat0 = 60.3, lon0 = 5.2, x = 3000, y = -2800;
            const lat = lat0 + y / M.M_PER_DEG;
            const lon = lon0 + x / (M.M_PER_DEG * Math.cos(lat0 * Math.PI / 180));
            const f = M.fieldFrame(lat0, lon0);
            const k = M.kAt(lat);
            const X = f.X0 + x * f.k, Y = f.Y0 - f.v(y) * f.k;
            out.err_m = Math.hypot(X - M.mx(lon), Y - M.my(lat)) / k;
            out.flat_m = Math.abs(f.Y0 - y * f.k - M.my(lat)) / k;
            const b = M.baseBox(50.9, -0.7, 51.4, 0.3);
            out.tiles = M.tileCount(b[0], b[1], b[2], b[3]);
            out.wide = M.baseBox(40, -14, 56, 12);
            out.wide_tiles = out.wide ? M.tileCount(out.wide[0], out.wide[1], out.wide[2], out.wide[3]) : 0;
            out.continent = M.baseBox(-60, -170, 60, 170);
            console.log(JSON.stringify(out));"#
        );
        let Ok(run) = std::process::Command::new("node")
            .arg("-e")
            .arg(&script)
            .output()
        else {
            eprintln!("node is not installed: the page's projection is not checked here");
            return;
        };
        assert!(
            run.status.success(),
            "{}",
            String::from_utf8_lossy(&run.stderr)
        );
        let v: serde_json::Value = serde_json::from_slice(&run.stdout).unwrap();
        let num = |k: &str| v[k].as_f64().unwrap_or(f64::NAN);
        assert!((num("lat") - 51.148).abs() < 1e-9 && (num("lon") + 0.19).abs() < 1e-9);
        assert!(num("err_m") < 0.01, "placed {} m off", num("err_m"));
        assert!(
            num("flat_m") > 1.0,
            "without the north re-measure: {} m",
            num("flat_m")
        );
        assert!(num("tiles") <= crate::efb::basemap::MAX_TILES as f64);
        assert_eq!(
            num("tiles"),
            4.0,
            "a margin round the view: 50.75 to 51.55, -1.0 to 0.6"
        );
        assert!(
            num("wide_tiles") <= crate::efb::basemap::MAX_TILES as f64 && num("wide_tiles") > 459.0
        );
        assert!(v["continent"].is_null());
    }
}
