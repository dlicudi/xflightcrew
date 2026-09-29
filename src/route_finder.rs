//! The local planner's LATERAL route: which way the line from the origin to
//! the destination actually goes.
//!
//! Two rules, both over X-Plane's own data and nothing invented:
//!
//! - **Airways for an IFR plan** (#75): `earth_awy.dat` segments, their ends
//!   placed from `earth_fix.dat` / `earth_nav.dat`, searched with Dijkstra
//!   from an entry fix near the origin to an exit fix near the destination.
//!   Only segments whose published band holds the planned level are used, so
//!   a jet at FL350 flies the upper routes and a turboprop at 9,000 the lower
//!   ones. The result is taken only when it is a real airway route (at least
//!   [`AIRWAY_MIN_SHARE`] of the direct distance on airways) and not much
//!   longer than direct ([`AIRWAY_MAX_DETOUR`]); else the plan stays direct.
//! - **Dog-legs round no-go airspace for a direct plan** (#74): the OpenAir
//!   file the sim ships (`airspace.txt`, the one the circuit already reads,
//!   P/R/Q only), checked along the line at the altitude the flight will be
//!   at there. A volume in the way is flown round on the shortest path past
//!   its convex hull, turning [`DOGLEG_MARGIN_NM`] clear of it and about
//!   [`CHORD_CLEAR_FRACTION`] of that clear between turns, within
//!   [`DOGLEG_MAX_DETOUR`] of direct. A volume the bound will not allow is
//!   flown through and NAMED in the brief; a volume holding either field
//!   (the flight starts or ends in it) is set aside and blackboxed.
//!
//! Pure below the file readers, so every rule is a plain `cargo test`.

use std::collections::{BinaryHeap, HashMap, HashSet};

use crate::airspace::Airspace;
use crate::nav;
use crate::simbrief::Waypoint;

/// An airway route may be at most this much longer than the direct line
/// (entry and exit legs included). Typical airway inefficiency is 5 to 15 %.
pub const AIRWAY_MAX_DETOUR: f64 = 1.20;
/// Below this the hop is flown direct: the joins would be the whole flight.
pub const AIRWAY_MIN_DIRECT_NM: f64 = 60.0;
/// ...and at least this share of the direct distance must be ON airways, or
/// it is a direct plan with a fix in it.
pub const AIRWAY_MIN_SHARE: f64 = 0.5;
/// The entry fix within this of the origin, the exit within this of the
/// destination.
pub const JOIN_MAX_NM: f64 = 60.0;
/// The exit fix at least this far out: the arm's final alignment fix sits
/// 16 nm out on the runway's line, and an exit inside it turns back.
pub const EXIT_MIN_NM: f64 = 25.0;
/// A direct leg costs this much more than an airway leg in the search, so a
/// route JOINS the airways near the ends instead of cutting across to the
/// one segment nearest the straight line. The real distance is what the
/// bounds judge.
const DCT_WEIGHT: f64 = 2.0;

/// A dog-leg route may be at most this much longer than direct.
pub const DOGLEG_MAX_DETOUR: f64 = 1.30;
/// Lateral clearance kept from a volume's hull, nm.
pub const DOGLEG_MARGIN_NM: f64 = 2.0;
/// Volumes routed round at once, and the lazy loop's passes.
const DOGLEG_MAX_OBSTACLES: usize = 12;
const DOGLEG_MAX_PASSES: usize = 24;
/// The conflict check's step along the path, nm. A mile stepped over the
/// small ones: the #74 sweep over the real `airspace.txt`
/// (`real_install_dog_legs_go_round_or_name_it`) found direct lines flown
/// silently through R 63 and R 118 and dog-legs grazing parachute and
/// aerobatic boxes a mile step never sampled.
const PATH_STEP_NM: f64 = 0.25;
/// How much of [`DOGLEG_MARGIN_NM`] a leg between two corners keeps from the
/// hull it passes (the corners keep all of it).
const CHORD_CLEAR_FRACTION: f64 = 0.75;
/// Named turn points: XVLD1..XVLD9 (five characters, like the arm's XVLCR).
const DOGLEG_MAX_TURNS: usize = 9;

/// How the lateral route was made.
#[derive(Debug, Clone, Default, PartialEq)]
pub enum Via {
    #[default]
    Direct,
    /// Joined at `entry`, flown along `airways` in order.
    Airways { entry: String, airways: Vec<String> },
    /// Round these volumes (spoken names).
    AroundAirspace { avoided: Vec<String> },
}

impl Via {
    pub fn label(&self) -> &'static str {
        match self {
            Via::Direct => "direct",
            Via::Airways { .. } => "airways",
            Via::AroundAirspace { .. } => "dogleg",
        }
    }
}

/// The lateral route between the two fields: the points between them, the
/// route string, the flown distance, and the no-go volumes it still crosses
/// (said in the brief, never hidden).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Lateral {
    pub via: Via,
    /// Enroute points, origin and destination excluded, `alt` 0 (cruise).
    pub mids: Vec<Waypoint>,
    /// "direct", or the airway string ("BIBAX UN866 LUMAS UL620 GIPNO").
    pub route: String,
    /// Origin to destination along the points, nm.
    pub distance_nm: f64,
    /// No-go volumes the route still passes through (spoken names).
    pub crossed: Vec<String>,
    /// Volumes over either field (blackboxed, not spoken; see [`DogLeg`]).
    pub at_fields: Vec<String>,
}

impl Lateral {
    pub fn direct(o: (f64, f64), d: (f64, f64)) -> Lateral {
        Lateral {
            via: Via::Direct,
            mids: Vec::new(),
            route: "direct".to_string(),
            distance_nm: nav::haversine_nm(o.0, o.1, d.0, d.1),
            crossed: Vec::new(),
            at_fields: Vec::new(),
        }
    }
}

/// Length of a lat/lon polyline, nm.
pub fn path_nm(pts: &[(f64, f64)]) -> f64 {
    pts.windows(2)
        .map(|w| nav::haversine_nm(w[0].0, w[0].1, w[1].0, w[1].1))
        .sum()
}

/// `n` points spread evenly along a polyline by distance, both ends included
/// (the terrain probe's samples for a route that is not one leg).
pub fn path_samples(pts: &[(f64, f64)], n: usize) -> Vec<(f64, f64)> {
    let total = path_nm(pts);
    if pts.len() < 2 || n < 2 || total <= 0.0 {
        return pts.first().map(|p| vec![*p; n.max(1)]).unwrap_or_default();
    }
    (0..n)
        .map(|i| point_along(pts, total * i as f64 / (n - 1) as f64))
        .collect()
}

/// The point `s` nm along a polyline (linear in lat/lon within a leg).
pub(crate) fn point_along(pts: &[(f64, f64)], s: f64) -> (f64, f64) {
    let mut left = s.max(0.0);
    for w in pts.windows(2) {
        let d = nav::haversine_nm(w[0].0, w[0].1, w[1].0, w[1].1);
        if left <= d && d > 0.0 {
            let f = left / d;
            return (
                w[0].0 + (w[1].0 - w[0].0) * f,
                w[0].1 + (w[1].1 - w[0].1) * f,
            );
        }
        left -= d;
    }
    *pts.last().unwrap_or(&(0.0, 0.0))
}

// ===========================================================================
// Airways
// ===========================================================================

/// A navaid or fix key as `earth_awy.dat` names it: ident, ICAO region, and
/// the type code (11 fix, 3 VOR, 2 NDB).
type NavKey = (String, String, u8);

/// Positions of every fix, VOR and NDB that `keep` accepts. An enroute fix
/// outranks a terminal one of the same ident and region (airways name the
/// enroute one).
pub fn read_positions(
    fix_lines: impl Iterator<Item = String>,
    nav_lines: impl Iterator<Item = String>,
    keep: impl Fn(f64, f64) -> bool,
) -> HashMap<NavKey, (f64, f64)> {
    let mut out: HashMap<NavKey, (f64, f64)> = HashMap::new();
    for line in fix_lines {
        // `lat lon IDENT area REGION ...`
        let t: Vec<&str> = line.split_whitespace().collect();
        if t.len() < 5 {
            continue;
        }
        let (Ok(lat), Ok(lon)) = (t[0].parse::<f64>(), t[1].parse::<f64>()) else {
            continue;
        };
        if !keep(lat, lon) {
            continue;
        }
        let key = (t[2].to_string(), t[4].to_string(), 11u8);
        if t[3] == "ENRT" || !out.contains_key(&key) {
            out.insert(key, (lat, lon));
        }
    }
    for line in nav_lines {
        // `row lat lon elev freq range var IDENT area REGION name...`
        let t: Vec<&str> = line.split_whitespace().collect();
        if t.len() < 10 {
            continue;
        }
        let code: u8 = match t[0] {
            "2" => 2,
            "3" => 3,
            _ => continue,
        };
        let (Ok(lat), Ok(lon)) = (t[1].parse::<f64>(), t[2].parse::<f64>()) else {
            continue;
        };
        if !keep(lat, lon) {
            continue;
        }
        let key = (t[7].to_string(), t[9].to_string(), code);
        if t[8] == "ENRT" || !out.contains_key(&key) {
            out.insert(key, (lat, lon));
        }
    }
    out
}

#[derive(Debug, Clone, PartialEq)]
pub struct AwyNode {
    pub ident: String,
    pub lat: f64,
    pub lon: f64,
}

#[derive(Debug, Clone)]
struct AwyEdge {
    to: usize,
    nm: f64,
    base_ft: f64,
    top_ft: f64,
    names: Vec<String>,
}

/// The airway network (the part of it that was kept), as a directed graph.
#[derive(Debug, Default)]
pub struct AirwayGraph {
    nodes: Vec<AwyNode>,
    index: HashMap<NavKey, usize>,
    adj: Vec<Vec<AwyEdge>>,
}

/// One airway route found.
#[derive(Debug, Clone, PartialEq)]
pub struct AirwayRoute {
    pub fixes: Vec<AwyNode>,
    /// The airway flown on each leg between consecutive fixes.
    pub leg_airways: Vec<String>,
    /// Origin to destination, joins included, nm.
    pub distance_nm: f64,
    /// The part on airways, nm.
    pub airway_nm: f64,
}

impl AirwayRoute {
    /// The airways in the order flown, each once per run.
    pub fn airways(&self) -> Vec<String> {
        let mut out: Vec<String> = Vec::new();
        for a in &self.leg_airways {
            if out.last() != Some(a) {
                out.push(a.clone());
            }
        }
        out
    }

    /// The ICAO-style route string: the entry, then each airway and the fix
    /// it is left at ("BIBAX UN866 LUMAS UL620 GIPNO").
    pub fn route_string(&self) -> String {
        let Some(first) = self.fixes.first() else {
            return "direct".to_string();
        };
        let mut s = vec![first.ident.clone()];
        for (i, a) in self.leg_airways.iter().enumerate() {
            let leaves = self.leg_airways.get(i + 1) != Some(a);
            if leaves {
                s.push(a.clone());
                s.push(self.fixes[i + 1].ident.clone());
            }
        }
        s.join(" ")
    }
}

#[derive(Clone, Copy, PartialEq)]
struct Queued(f64, usize);
impl Eq for Queued {}
impl Ord for Queued {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        o.0.total_cmp(&self.0).then(o.1.cmp(&self.1))
    }
}
impl PartialOrd for Queued {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}

impl AirwayGraph {
    /// Build from `earth_awy.dat` lines: `FIX1 REG1 T1 FIX2 REG2 T2 DIR LEVEL
    /// BASE TOP NAMES`, BASE/TOP in hundreds of feet, DIR `N` both ways, `F`
    /// first to second only, `B` second to first only, NAMES hyphen-joined.
    /// A segment with an end `positions` lacks is left out.
    pub fn parse(
        awy_lines: impl Iterator<Item = String>,
        positions: &HashMap<NavKey, (f64, f64)>,
    ) -> AirwayGraph {
        let mut g = AirwayGraph::default();
        for line in awy_lines {
            let t: Vec<&str> = line.split_whitespace().collect();
            if t.len() < 11 {
                continue;
            }
            let (Ok(t1), Ok(t2)) = (t[2].parse::<u8>(), t[5].parse::<u8>()) else {
                continue;
            };
            let (Ok(base), Ok(top)) = (t[8].parse::<f64>(), t[9].parse::<f64>()) else {
                continue;
            };
            let k1 = (t[0].to_string(), t[1].to_string(), t1);
            let k2 = (t[3].to_string(), t[4].to_string(), t2);
            let (Some(p1), Some(p2)) = (positions.get(&k1), positions.get(&k2)) else {
                continue;
            };
            let a = g.node(k1, *p1);
            let b = g.node(k2, *p2);
            let nm = nav::haversine_nm(p1.0, p1.1, p2.0, p2.1);
            let names: Vec<String> = t[10].split('-').map(str::to_string).collect();
            let edge = |to| AwyEdge {
                to,
                nm,
                base_ft: base * 100.0,
                top_ft: top * 100.0,
                names: names.clone(),
            };
            match t[6] {
                "F" => g.adj[a].push(edge(b)),
                "B" => g.adj[b].push(edge(a)),
                _ => {
                    g.adj[a].push(edge(b));
                    g.adj[b].push(edge(a));
                }
            }
        }
        g
    }

    fn node(&mut self, key: NavKey, p: (f64, f64)) -> usize {
        if let Some(&i) = self.index.get(&key) {
            return i;
        }
        let i = self.nodes.len();
        self.nodes.push(AwyNode {
            ident: key.0.clone(),
            lat: p.0,
            lon: p.1,
        });
        self.adj.push(Vec::new());
        self.index.insert(key, i);
        i
    }

    #[cfg(test)]
    pub fn len(&self) -> usize {
        self.nodes.len()
    }

    /// The airway route from `o` to `d` at `cruise_ft`, or `None` when there
    /// is none worth flying (see the module doc for the bounds).
    pub fn route(&self, o: (f64, f64), d: (f64, f64), cruise_ft: f64) -> Option<AirwayRoute> {
        let direct = nav::haversine_nm(o.0, o.1, d.0, d.1);
        if direct < AIRWAY_MIN_DIRECT_NM || self.nodes.is_empty() {
            return None;
        }
        let n = self.nodes.len();
        let to_o: Vec<f64> = self
            .nodes
            .iter()
            .map(|p| nav::haversine_nm(o.0, o.1, p.lat, p.lon))
            .collect();
        let to_d: Vec<f64> = self
            .nodes
            .iter()
            .map(|p| nav::haversine_nm(p.lat, p.lon, d.0, d.1))
            .collect();
        // Two layers: state i (just joined at i) and n + i (flown at least
        // one airway leg to i). Only the second may leave for the destination.
        let mut cost = vec![f64::INFINITY; 2 * n];
        let mut prev: Vec<Option<(usize, String)>> = vec![None; 2 * n];
        let mut heap = BinaryHeap::new();
        for i in 0..n {
            if to_o[i] <= JOIN_MAX_NM {
                cost[i] = to_o[i] * DCT_WEIGHT;
                heap.push(Queued(cost[i], i));
            }
        }
        while let Some(Queued(c, s)) = heap.pop() {
            if c > cost[s] {
                continue;
            }
            let u = s % n;
            for e in &self.adj[u] {
                if cruise_ft < e.base_ft || cruise_ft > e.top_ft {
                    continue;
                }
                let t = n + e.to;
                let nc = c + e.nm;
                if nc < cost[t] {
                    cost[t] = nc;
                    // The name continuing the airway we came in on, if it can.
                    let came = prev[s].as_ref().map(|(_, a)| a.clone());
                    let name = came
                        .filter(|a| e.names.contains(a))
                        .unwrap_or_else(|| e.names[0].clone());
                    prev[t] = Some((s, name));
                    heap.push(Queued(nc, t));
                }
            }
        }
        let exit = (0..n)
            .filter(|&i| cost[n + i].is_finite() && (EXIT_MIN_NM..=JOIN_MAX_NM).contains(&to_d[i]))
            .min_by(|&a, &b| {
                (cost[n + a] + to_d[a] * DCT_WEIGHT)
                    .total_cmp(&(cost[n + b] + to_d[b] * DCT_WEIGHT))
            })?;
        let mut states = vec![n + exit];
        let mut names = Vec::new();
        let mut s = n + exit;
        while let Some((p, name)) = prev[s].clone() {
            names.push(name);
            states.push(p);
            s = p;
        }
        states.reverse();
        names.reverse();
        let fixes: Vec<AwyNode> = states.iter().map(|s| self.nodes[s % n].clone()).collect();
        let airway_nm: f64 = fixes
            .windows(2)
            .map(|w| nav::haversine_nm(w[0].lat, w[0].lon, w[1].lat, w[1].lon))
            .sum();
        let entry = states[0] % n;
        let distance_nm = to_o[entry] + airway_nm + to_d[exit];
        (distance_nm <= direct * AIRWAY_MAX_DETOUR && airway_nm >= direct * AIRWAY_MIN_SHARE)
            .then_some(AirwayRoute {
                fixes,
                leg_airways: names,
                distance_nm,
                airway_nm,
            })
    }
}

/// Load the airway graph a route from `o` to `d` could use (every point
/// inside the detour ellipse) and search it. `None` on any missing file or
/// no route worth flying.
pub fn airway_route_from_files(
    awy_path: &str,
    fix_path: &str,
    nav_path: &str,
    o: (f64, f64),
    d: (f64, f64),
    cruise_ft: f64,
) -> Option<AirwayRoute> {
    use std::io::{BufRead, BufReader};
    let lines = |p: &str| {
        std::fs::File::open(p)
            .ok()
            .map(|f| BufReader::new(f).lines().map_while(Result::ok))
    };
    let direct = nav::haversine_nm(o.0, o.1, d.0, d.1);
    if direct < AIRWAY_MIN_DIRECT_NM {
        return None;
    }
    let reach = direct * AIRWAY_MAX_DETOUR;
    let keep = |lat: f64, lon: f64| {
        nav::haversine_nm(o.0, o.1, lat, lon) + nav::haversine_nm(lat, lon, d.0, d.1) <= reach
    };
    let positions = read_positions(lines(fix_path)?, lines(nav_path)?, keep);
    let graph = AirwayGraph::parse(lines(awy_path)?, &positions);
    graph.route(o, d, cruise_ft)
}

/// The route as the plan's lateral: fixes as waypoints, the airway string.
pub fn airway_lateral(r: &AirwayRoute) -> Lateral {
    Lateral {
        via: Via::Airways {
            entry: r.fixes.first().map(|f| f.ident.clone()).unwrap_or_default(),
            airways: r.airways(),
        },
        mids: r
            .fixes
            .iter()
            .map(|f| Waypoint {
                ident: f.ident.clone(),
                lat: f.lat,
                lon: f.lon,
                alt: 0.0,
            })
            .collect(),
        route: r.route_string(),
        distance_nm: r.distance_nm,
        crossed: Vec::new(),
        at_fields: Vec::new(),
    }
}

// ===========================================================================
// Dog-legs round no-go airspace
// ===========================================================================

/// Where the flight will be vertically along the route, for deciding which
/// volumes it would actually meet. With both gradients known, one altitude
/// at each point (climbing from the origin, descending to the destination,
/// never above cruise); without them, anything from the lower field up to
/// cruise, which is the conservative band, not a guess.
#[derive(Debug, Clone, Copy)]
pub struct Profile {
    pub origin_elev_ft: f64,
    pub dest_elev_ft: f64,
    pub cruise_ft: f64,
    pub climb_ft_per_nm: Option<f64>,
    pub descent_ft_per_nm: Option<f64>,
}

impl Profile {
    /// The altitude band at `s` nm along a route of `total` nm.
    pub fn band(&self, s: f64, total: f64) -> (f64, f64) {
        match (self.climb_ft_per_nm, self.descent_ft_per_nm) {
            (Some(c), Some(dsc)) if c > 0.0 && dsc > 0.0 => {
                let a = (self.origin_elev_ft + c * s)
                    .min(self.dest_elev_ft + dsc * (total - s).max(0.0))
                    .min(self.cruise_ft);
                (a, a)
            }
            _ => (self.origin_elev_ft.min(self.dest_elev_ft), self.cruise_ft),
        }
    }
}

/// Does `a` block a flight in the band `(lo, hi)` over `p`? The volume's own
/// 100 ft tolerance, as [`Airspace::blocks`].
fn blocks_band(a: &Airspace, p: (f64, f64), band: (f64, f64)) -> bool {
    band.1 >= a.floor_ft - 100.0 && band.0 <= a.ceiling_ft + 100.0 && a.contains(p.0, p.1)
}

/// A local flat projection in nautical miles about a reference point.
#[derive(Debug, Clone, Copy)]
struct Proj {
    lat0: f64,
    lon0: f64,
    k: f64,
}

impl Proj {
    fn new(lat0: f64, lon0: f64) -> Proj {
        Proj {
            lat0,
            lon0,
            k: lat0.to_radians().cos().max(1e-6),
        }
    }
    fn xy(&self, p: (f64, f64)) -> (f64, f64) {
        ((p.1 - self.lon0) * 60.0 * self.k, (p.0 - self.lat0) * 60.0)
    }
    fn ll(&self, q: (f64, f64)) -> (f64, f64) {
        (self.lat0 + q.1 / 60.0, self.lon0 + q.0 / (60.0 * self.k))
    }
}

fn cross(o: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0)
}

/// Convex hull, counter-clockwise, no repeated point (Andrew's chain).
fn convex_hull(mut pts: Vec<(f64, f64)>) -> Vec<(f64, f64)> {
    pts.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
    pts.dedup();
    if pts.len() < 3 {
        return pts;
    }
    let mut lower: Vec<(f64, f64)> = Vec::new();
    for p in &pts {
        while lower.len() >= 2 && cross(lower[lower.len() - 2], lower[lower.len() - 1], *p) <= 0.0 {
            lower.pop();
        }
        lower.push(*p);
    }
    let mut upper: Vec<(f64, f64)> = Vec::new();
    for p in pts.iter().rev() {
        while upper.len() >= 2 && cross(upper[upper.len() - 2], upper[upper.len() - 1], *p) <= 0.0 {
            upper.pop();
        }
        upper.push(*p);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

/// The hull pushed out by `m`: each vertex moved out along both adjacent
/// edges' normals, hulled again. Every edge of the result is at least
/// `m cos(half the turn)` clear of the original hull, strictly outside it.
fn offset_hull(h: &[(f64, f64)], m: f64) -> Vec<(f64, f64)> {
    let n = h.len();
    let normal = |a: (f64, f64), b: (f64, f64)| {
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let l = (dx * dx + dy * dy).sqrt().max(1e-9);
        (dy / l, -dx / l)
    };
    let mut pts = Vec::with_capacity(2 * n);
    for i in 0..n {
        let (p, v, q) = (h[(i + n - 1) % n], h[i], h[(i + 1) % n]);
        for nn in [normal(p, v), normal(v, q)] {
            pts.push((v.0 + m * nn.0, v.1 + m * nn.1));
        }
    }
    convex_hull(pts)
}

/// Strictly inside a counter-clockwise convex polygon.
fn inside_convex(h: &[(f64, f64)], p: (f64, f64)) -> bool {
    h.len() >= 3 && (0..h.len()).all(|i| cross(h[i], h[(i + 1) % h.len()], p) > 1e-9)
}

/// Does the segment `a`-`b` pass through the interior of the convex hull
/// (more than a graze)? Cyrus-Beck clipping.
fn segment_crosses(h: &[(f64, f64)], a: (f64, f64), b: (f64, f64)) -> bool {
    if h.len() < 3 {
        return false;
    }
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len = (dx * dx + dy * dy).sqrt();
    let (mut t0, mut t1) = (0.0_f64, 1.0_f64);
    for i in 0..h.len() {
        let (p, q) = (h[i], h[(i + 1) % h.len()]);
        // Outward normal of a CCW edge.
        let (nx, ny) = (q.1 - p.1, -(q.0 - p.0));
        let side = nx * (a.0 - p.0) + ny * (a.1 - p.1);
        let den = nx * dx + ny * dy;
        if den.abs() < 1e-12 {
            if side > 0.0 {
                return false;
            }
            continue;
        }
        let t = -side / den;
        if den > 0.0 {
            t1 = t1.min(t);
        } else {
            t0 = t0.max(t);
        }
        if t1 <= t0 {
            return false;
        }
    }
    (t1 - t0) * len > 0.01
}

/// The shortest path from `o` to `d` (projected) past every hull, through
/// the offset hulls' corners. `None` when there is none.
fn visibility_path(
    o: (f64, f64),
    d: (f64, f64),
    hulls: &[Vec<(f64, f64)>],
    margin: f64,
) -> Option<Vec<(f64, f64)>> {
    let mut nodes = vec![o, d];
    for h in hulls {
        for v in offset_hull(h, margin) {
            if !hulls.iter().any(|g| inside_convex(g, v)) {
                nodes.push(v);
            }
        }
    }
    // A leg between two offset corners is a CHORD of the offset ring and
    // cuts back toward the hull: tested against the bare hull, EBAW-LFYG
    // (#74 sweep) passed R 77's edge at 0.05 nm between two corners 2 nm
    // out. Legs are tested against the hull pushed out by
    // `CHORD_CLEAR_FRACTION` of the margin, so every leg keeps at least that
    // (the corners keep the whole margin); the bare hull where a field sits
    // inside the wider one (the flight starts or ends that close).
    let walls: Vec<Vec<(f64, f64)>> = hulls
        .iter()
        .map(|h| {
            let w = offset_hull(h, margin * CHORD_CLEAR_FRACTION);
            if inside_convex(&w, o) || inside_convex(&w, d) {
                h.clone()
            } else {
                w
            }
        })
        .collect();
    let n = nodes.len();
    let visible =
        |i: usize, j: usize| !walls.iter().any(|h| segment_crosses(h, nodes[i], nodes[j]));
    let len = |i: usize, j: usize| {
        let (dx, dy) = (nodes[j].0 - nodes[i].0, nodes[j].1 - nodes[i].1);
        (dx * dx + dy * dy).sqrt()
    };
    let mut dist = vec![f64::INFINITY; n];
    let mut prev = vec![usize::MAX; n];
    let mut done = vec![false; n];
    dist[0] = 0.0;
    loop {
        let u = (0..n)
            .filter(|&i| !done[i] && dist[i].is_finite())
            .min_by(|&a, &b| dist[a].total_cmp(&dist[b]))?;
        if u == 1 {
            break;
        }
        done[u] = true;
        for v in 0..n {
            if done[v] || v == u {
                continue;
            }
            let c = dist[u] + len(u, v);
            if c < dist[v] && visible(u, v) {
                dist[v] = c;
                prev[v] = u;
            }
        }
    }
    let mut path = vec![nodes[1]];
    let mut k = 1;
    while prev[k] != usize::MAX {
        k = prev[k];
        path.push(nodes[k]);
    }
    path.reverse();
    Some(path)
}

/// What the dog-leg search found.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct DogLeg {
    /// Turn points between the fields (lat, lon); empty = direct.
    pub turns: Vec<(f64, f64)>,
    /// Spoken names of the volumes flown round.
    pub avoided: Vec<String>,
    /// Spoken names of the volumes the route still enters that a detour
    /// within the bound could not avoid.
    pub crossed: Vec<String>,
    /// Volumes over either field: the flight starts or ends inside them, so
    /// they are no dog-leg's business. Blackboxed, not spoken (LFPN sits
    /// inside eight, from R 324 to a no-supersonic zone).
    pub at_fields: Vec<String>,
    pub distance_nm: f64,
}

/// Every no-go volume the lat/lon path enters with the flight at its
/// profile's altitude there, in the order met, each once; `skip` excluded.
fn conflicts(
    path: &[(f64, f64)],
    spaces: &[Airspace],
    boxes: &[(f64, f64, f64, f64)],
    profile: &Profile,
    skip: &HashSet<usize>,
) -> Vec<usize> {
    let total = path_nm(path);
    let steps = (total / PATH_STEP_NM).ceil().max(1.0) as usize;
    let mut out: Vec<usize> = Vec::new();
    for i in 0..=steps {
        let s = total * i as f64 / steps as f64;
        let p = point_along(path, s);
        let band = profile.band(s, total);
        for (k, a) in spaces.iter().enumerate() {
            let b = boxes[k];
            if skip.contains(&k)
                || out.contains(&k)
                || p.0 < b.0
                || p.0 > b.1
                || p.1 < b.2
                || p.1 > b.3
            {
                continue;
            }
            if blocks_band(a, p, band) {
                out.push(k);
            }
        }
    }
    out
}

/// Fly round the no-go volumes between `o` and `d` (see the module doc).
pub fn dog_leg(o: (f64, f64), d: (f64, f64), spaces: &[Airspace], profile: &Profile) -> DogLeg {
    let direct = nav::haversine_nm(o.0, o.1, d.0, d.1);
    let proj = Proj::new((o.0 + d.0) / 2.0, (o.1 + d.1) / 2.0);
    let boxes: Vec<(f64, f64, f64, f64)> = spaces
        .iter()
        .map(|a| {
            a.poly
                .iter()
                .fold((f64::MAX, f64::MIN, f64::MAX, f64::MIN), |b, (la, lo)| {
                    (b.0.min(*la), b.1.max(*la), b.2.min(*lo), b.3.max(*lo))
                })
        })
        .collect();
    let hull = |k: usize| convex_hull(spaces[k].poly.iter().map(|p| proj.xy(*p)).collect());
    let (oxy, dxy) = (proj.xy(o), proj.xy(d));
    let mut skip: HashSet<usize> = HashSet::new();
    let mut at_fields: Vec<String> = Vec::new();
    let mut active: Vec<usize> = Vec::new();
    let mut path = vec![o, d];
    // A volume over either field: the flight starts or ends inside it.
    let over_field = |k: usize, h: &[(f64, f64)]| {
        h.len() < 3
            || inside_convex(h, oxy)
            || inside_convex(h, dxy)
            || spaces[k].contains(o.0, o.1)
            || spaces[k].contains(d.0, d.1)
    };
    let give_up = |skip: &HashSet<usize>, mut at_fields: Vec<String>| {
        // Direct, naming everything on the direct line but the fields' own,
        // including a field's volume the search never reached: EDDG-EDVK
        // (#74 sweep) named Kassel-Calden's own parachute zone as a volume
        // "the route still enters".
        let mut named: Vec<String> = Vec::new();
        for k in conflicts(&[o, d], spaces, &boxes, profile, skip) {
            let s = spaces[k].spoken_name();
            if over_field(k, &hull(k)) {
                if !at_fields.contains(&s) {
                    at_fields.push(s);
                }
            } else if !named.contains(&s) {
                named.push(s);
            }
        }
        DogLeg {
            turns: Vec::new(),
            avoided: Vec::new(),
            crossed: named,
            at_fields,
            distance_nm: direct,
        }
    };
    for _ in 0..DOGLEG_MAX_PASSES {
        let Some(&k) = conflicts(&path, spaces, &boxes, profile, &skip).first() else {
            let turns: Vec<(f64, f64)> = path[1..path.len() - 1].to_vec();
            if turns.len() > DOGLEG_MAX_TURNS {
                return give_up(&skip, at_fields);
            }
            let mut avoided: Vec<String> = Vec::new();
            for k in &active {
                let s = spaces[*k].spoken_name();
                if !avoided.contains(&s) {
                    avoided.push(s);
                }
            }
            return DogLeg {
                distance_nm: path_nm(&path),
                turns,
                avoided,
                crossed: Vec::new(),
                at_fields,
            };
        };
        let h = hull(k);
        // A volume over either field cannot be flown round: set aside.
        if over_field(k, &h) {
            skip.insert(k);
            let s = spaces[k].spoken_name();
            if !at_fields.contains(&s) {
                at_fields.push(s);
            }
            continue;
        }
        if active.contains(&k) || active.len() >= DOGLEG_MAX_OBSTACLES {
            return give_up(&skip, at_fields);
        }
        active.push(k);
        let hulls: Vec<Vec<(f64, f64)>> = active.iter().map(|k| hull(*k)).collect();
        let Some(p) = visibility_path(oxy, dxy, &hulls, DOGLEG_MARGIN_NM) else {
            return give_up(&skip, at_fields);
        };
        path = p.into_iter().map(|q| proj.ll(q)).collect();
        // The ends exactly as given (the projection round trip is not).
        path[0] = o;
        let last = path.len() - 1;
        path[last] = d;
        if path_nm(&path) > direct * DOGLEG_MAX_DETOUR {
            return give_up(&skip, at_fields);
        }
    }
    give_up(&skip, at_fields)
}

/// The dog-leg as the plan's lateral.
pub fn dog_leg_lateral(o: (f64, f64), d: (f64, f64), dl: &DogLeg) -> Lateral {
    if dl.turns.is_empty() {
        return Lateral {
            crossed: dl.crossed.clone(),
            at_fields: dl.at_fields.clone(),
            ..Lateral::direct(o, d)
        };
    }
    let mids: Vec<Waypoint> = dl
        .turns
        .iter()
        .enumerate()
        .map(|(i, p)| Waypoint {
            ident: format!("XVLD{}", i + 1),
            lat: p.0,
            lon: p.1,
            alt: 0.0,
        })
        .collect();
    Lateral {
        via: Via::AroundAirspace {
            avoided: dl.avoided.clone(),
        },
        route: mids
            .iter()
            .map(|w| w.ident.clone())
            .collect::<Vec<_>>()
            .join(" "),
        mids,
        distance_nm: dl.distance_nm,
        crossed: dl.crossed.clone(),
        at_fields: dl.at_fields.clone(),
    }
}

/// The no-go volumes a route from `o` to `d` could meet: the box round the
/// dog-leg detour ellipse. Empty when the file is missing.
pub fn load_spaces(path: &str, o: (f64, f64), d: (f64, f64)) -> Vec<Airspace> {
    let direct = nav::haversine_nm(o.0, o.1, d.0, d.1);
    // The ellipse's half-width, plus a margin, in degrees.
    let half = direct * DOGLEG_MAX_DETOUR / 2.0 / 60.0 + 0.2;
    let lat_lo = o.0.min(d.0) - half;
    let lat_hi = o.0.max(d.0) + half;
    let k = ((o.0 + d.0) / 2.0).to_radians().cos().max(0.2);
    let lon_lo = o.1.min(d.1) - half / k;
    let lon_hi = o.1.max(d.1) + half / k;
    crate::airspace::load_box(path, lat_lo, lat_hi, lon_lo, lon_hi)
}

/// The planner's whole lateral decision, from the files: airways for an IFR
/// plan when a route is worth flying, else direct, dog-legged round no-go
/// volumes. Blocking; call it off the async thread.
pub struct LateralFiles<'a> {
    pub awy: &'a str,
    pub fix: &'a str,
    pub nav: &'a str,
    pub airspace: &'a str,
}

pub fn plan_lateral(
    files: &LateralFiles,
    o: (f64, f64),
    d: (f64, f64),
    ifr: bool,
    profile: &Profile,
) -> Lateral {
    if ifr {
        if let Some(r) =
            airway_route_from_files(files.awy, files.fix, files.nav, o, d, profile.cruise_ft)
        {
            return airway_lateral(&r);
        }
    }
    let spaces = load_spaces(files.airspace, o, d);
    dog_leg_lateral(o, d, &dog_leg(o, d, &spaces, profile))
}

/// The file beside `sibling` named `name`, else X-Plane's default copy
/// (`earth_awy.dat` next to the fix file in use, like `earth_mora.dat`).
pub fn data_file_beside(sibling: &str, name: &str) -> String {
    let beside = std::path::Path::new(sibling).with_file_name(name);
    if beside.exists() {
        beside.to_string_lossy().into_owned()
    } else {
        format!(
            "{}/Resources/default data/{name}",
            crate::config::xplane_root()
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lines(s: &str) -> impl Iterator<Item = String> + '_ {
        s.lines().map(str::to_string)
    }

    /// A square of airways west to east along 40N from 0E to 4E, a high
    /// route along it and a low one beside it, and a stray segment.
    const FIX: &str = "I\n1200 Version\n\n\
 40.000000000    0.500000000  AAAAA ENRT LE 2 AAAAA\n\
 40.000000000    1.500000000  BBBBB ENRT LE 2 BBBBB\n\
 40.000000000    2.500000000  CCCCC ENRT LE 2 CCCCC\n\
 40.000000000    3.500000000  DDDDD ENRT LE 2 DDDDD\n\
 40.500000000    2.000000000  HIGHX ENRT LE 2 HIGHX\n\
 40.000000000    2.500000000  CCCCC LEMD LE 4 CCCCC\n\
99\n";
    const NAV: &str = "I\n1200 Version\n\n\
 3  40.000000000    2.000000000     1000    11500   130     0.000  VVV ENRT LE VICTOR VOR\n\
99\n";
    const AWY: &str = "I\n1100 Version\n\n\
AAAAA LE 11 BBBBB LE 11 N 2 245 460 UN1\n\
BBBBB LE 11 VVV LE 3 N 2 245 460 UN1-UL2\n\
VVV LE 3 CCCCC LE 11 N 2 245 460 UL2\n\
CCCCC LE 11 DDDDD LE 11 F 2 245 460 UL2\n\
BBBBB LE 11 HIGHX LE 11 N 1 50 200 B3\n\
HIGHX LE 11 CCCCC LE 11 N 1 50 200 B3\n\
MISSING LE 11 CCCCC LE 11 N 1 50 200 Z9\n\
99\n";

    fn graph() -> AirwayGraph {
        let pos = read_positions(lines(FIX), lines(NAV), |_, _| true);
        AirwayGraph::parse(lines(AWY), &pos)
    }

    #[test]
    fn airways_parse_from_the_three_files_and_skip_what_has_no_position() {
        let pos = read_positions(lines(FIX), lines(NAV), |_, _| true);
        // The enroute CCCCC, not the terminal namesake.
        assert_eq!(pos[&("CCCCC".into(), "LE".into(), 11)], (40.0, 2.5));
        assert_eq!(pos[&("VVV".into(), "LE".into(), 3)], (40.0, 2.0));
        let g = graph();
        assert_eq!(g.len(), 6, "MISSING never became a node");
    }

    #[test]
    fn an_ifr_route_flies_the_airways_at_its_level_and_one_way_segments_one_way() {
        let g = graph();
        let o = (40.0, 0.0);
        let d = (40.0, 4.1);
        // FL350: the upper route, joined at AAAAA, left at DDDDD.
        let r = g.route(o, d, 35_000.0).expect("an upper route");
        let ids: Vec<&str> = r.fixes.iter().map(|f| f.ident.as_str()).collect();
        assert_eq!(ids, ["AAAAA", "BBBBB", "VVV", "CCCCC", "DDDDD"]);
        assert_eq!(r.route_string(), "AAAAA UN1 VVV UL2 DDDDD");
        assert_eq!(r.airways(), ["UN1", "UL2"]);
        let direct = nav::haversine_nm(o.0, o.1, d.0, d.1);
        assert!(
            r.distance_nm <= direct * AIRWAY_MAX_DETOUR,
            "{} vs {direct}",
            r.distance_nm
        );
        let lat = airway_lateral(&r);
        assert_eq!(lat.via.label(), "airways");
        assert_eq!(lat.mids.len(), 5);
        // Eastbound only: CCCCC to DDDDD is `F`, so westbound has no route
        // that far (the exit would be CCCCC, inside the join band).
        let back = g.route(d, o, 35_000.0);
        assert!(back.is_none_or(|b| !b.fixes.iter().any(|f| f.ident == "DDDDD")));
        // 9,000 ft: under the upper airways; the low B3 alone is too short a
        // share of the flight, so direct.
        assert!(g.route(o, d, 9_000.0).is_none());
    }

    #[test]
    fn a_short_hop_or_a_long_detour_stays_direct() {
        let g = graph();
        // 40 nm: the joins would be the flight.
        assert!(g.route((40.0, 1.0), (40.0, 1.8), 35_000.0).is_none());
        // Due north across the airways: every route is a detour.
        assert!(g.route((38.5, 2.0), (41.5, 2.0), 35_000.0).is_none());
    }

    /// A restricted square 10 nm a side straddling the line from (40N, 0E)
    /// to (40N, 2E), surface to 5,000 ft.
    fn square(floor: &str, ceiling: &str, lat: f64, lon: f64, half_nm: f64) -> Airspace {
        let (dl, dn) = (half_nm / 60.0, half_nm / 60.0 / (lat.to_radians().cos()));
        let text = format!(
            "AC R\nAN R-77 (TEST)\nAL {floor}\nAH {ceiling}\nDP {}\nDP {}\nDP {}\nDP {}\n",
            dms(lat - dl, lon - dn),
            dms(lat - dl, lon + dn),
            dms(lat + dl, lon + dn),
            dms(lat + dl, lon - dn)
        );
        crate::airspace::parse_no_go(text.lines().map(str::to_string)).remove(0)
    }

    fn dms(lat: f64, lon: f64) -> String {
        let f = |v: f64| {
            let a = v.abs();
            let d = a.floor();
            let m = ((a - d) * 60.0).floor();
            let s = ((a - d) * 60.0 - m) * 60.0;
            (d, m, s)
        };
        let (a, b, c) = f(lat);
        let (x, y, z) = f(lon);
        format!(
            "{:02}:{:02}:{:05.2} {} {:03}:{:02}:{:05.2} {}",
            a,
            b,
            c,
            if lat >= 0.0 { "N" } else { "S" },
            x,
            y,
            z,
            if lon >= 0.0 { "E" } else { "W" }
        )
    }

    fn vfr(cruise: f64) -> Profile {
        Profile {
            origin_elev_ft: 0.0,
            dest_elev_ft: 0.0,
            cruise_ft: cruise,
            // 700 fpm at 90 kt, 500 fpm at 120 kt.
            climb_ft_per_nm: Some(700.0 / 1.5),
            descent_ft_per_nm: Some(250.0),
        }
    }

    #[test]
    fn a_restricted_area_on_the_line_is_flown_round_clear_of_its_edge() {
        let o = (40.0, 0.0);
        let d = (40.0, 2.0);
        let r = square("GND", "5000 MSL", 40.0, 1.0, 5.0);
        let spaces = vec![r];
        let dl = dog_leg(o, d, &spaces, &vfr(4500.0));
        assert!(!dl.turns.is_empty(), "{dl:?}");
        assert_eq!(dl.avoided, ["restricted area R 77"]);
        assert!(dl.crossed.is_empty());
        let direct = nav::haversine_nm(o.0, o.1, d.0, d.1);
        assert!(
            dl.distance_nm > direct && dl.distance_nm <= direct * DOGLEG_MAX_DETOUR,
            "{}",
            dl.distance_nm
        );
        // The whole route, sampled finely, never enters it.
        let mut path = vec![o];
        path.extend(dl.turns.iter().copied());
        path.push(d);
        for p in path_samples(&path, 400) {
            assert!(!spaces[0].contains(p.0, p.1), "{p:?}");
        }
        // Every turn point is at least the margin (less the chamfer) off it.
        let lat = dog_leg_lateral(o, d, &dl);
        assert_eq!(lat.via.label(), "dogleg");
        assert!(lat.mids.iter().all(|w| w.ident.starts_with("XVLD")));
    }

    #[test]
    fn a_volume_under_the_cruise_or_over_a_field_is_not_a_dog_leg() {
        let o = (40.0, 0.0);
        let d = (40.0, 2.0);
        // Surface to 2,000 ft mid-route: the flight is at 4,500 there.
        let low = vec![square("GND", "2000 MSL", 40.0, 1.0, 5.0)];
        let dl = dog_leg(o, d, &low, &vfr(4500.0));
        assert!(dl.turns.is_empty() && dl.crossed.is_empty(), "{dl:?}");
        // Without climb figures, the band is the whole flight: flown round.
        let blind = Profile {
            climb_ft_per_nm: None,
            ..vfr(4500.0)
        };
        assert!(!dog_leg(o, d, &low, &blind).turns.is_empty());
        // Over the destination: the flight ends inside it, so it is set
        // aside (blackboxed), neither dodged nor spoken.
        let over = vec![square("GND", "5000 MSL", 40.0, 2.0, 3.0)];
        let dl = dog_leg(o, d, &over, &vfr(4500.0));
        assert!(dl.turns.is_empty() && dl.crossed.is_empty(), "{dl:?}");
        assert_eq!(dl.at_fields, ["restricted area R 77"]);
    }

    #[test]
    fn a_wall_too_wide_to_go_round_is_direct_and_named() {
        let o = (40.0, 0.0);
        let d = (40.0, 2.0);
        // 60 nm tall across a 92 nm leg: round it is far past the bound.
        let wall = vec![square("GND", "FL100", 40.0, 1.0, 30.0)];
        let dl = dog_leg(o, d, &wall, &vfr(4500.0));
        assert!(dl.turns.is_empty());
        assert_eq!(dl.crossed, ["restricted area R 77"]);
        let lat = dog_leg_lateral(o, d, &dl);
        assert_eq!(lat.via, Via::Direct);
        assert_eq!(lat.crossed, ["restricted area R 77"]);
    }

    #[test]
    fn two_volumes_in_a_row_are_both_flown_round() {
        let o = (40.0, 0.0);
        let d = (40.0, 3.0);
        let spaces = vec![
            square("GND", "5000 MSL", 40.0, 1.0, 4.0),
            square("GND", "5000 MSL", 40.05, 2.0, 4.0),
        ];
        let dl = dog_leg(o, d, &spaces, &vfr(4500.0));
        assert!(!dl.turns.is_empty(), "{dl:?}");
        let mut path = vec![o];
        path.extend(dl.turns.iter().copied());
        path.push(d);
        for p in path_samples(&path, 600) {
            assert!(!spaces.iter().any(|s| s.contains(p.0, p.1)), "{p:?}");
        }
    }

    #[test]
    fn samples_follow_the_path_not_the_chord() {
        // Out and back: the chord is nothing, the path 120 nm.
        let pts = [(40.0, 0.0), (41.0, 0.0), (40.0, 0.0)];
        let s = path_samples(&pts, 5);
        assert_eq!(s.len(), 5);
        assert!(
            (s[2].0 - 41.0).abs() < 0.02 && s[2].1.abs() < 0.02,
            "{:?}",
            s[2]
        );
        assert!((s[1].0 - 40.5).abs() < 0.02, "{:?}", s[1]);
        assert_eq!(s[4], (40.0, 0.0));
    }

    /// A real airway route over the installed nav data, printed.
    /// `cargo test -- --ignored real_install_airway_route --nocapture`
    #[test]
    #[ignore]
    fn real_install_airway_route() {
        let c = crate::config::Config::from_env().expect("config");
        let awy = data_file_beside(&c.fix_data_path, "earth_awy.dat");
        for (name, o, d, fl) in [
            ("EGKK-EGPH", (51.148, -0.190), (55.950, -3.363), 35_000.0),
            ("EGKK-EGPH", (51.148, -0.190), (55.950, -3.363), 21_000.0),
            ("EGKK-EGPH", (51.148, -0.190), (55.950, -3.363), 11_000.0),
            (
                "KSFO-KLAS",
                (37.619, -122.375),
                (36.080, -115.152),
                35_000.0,
            ),
            ("LEMD-LEBL", (40.472, -3.561), (41.297, 2.078), 33_000.0),
            ("KSFO-KLAS", (37.619, -122.375), (36.080, -115.152), 9_000.0),
        ] {
            let t = std::time::Instant::now();
            let r = airway_route_from_files(&awy, &c.fix_data_path, &c.nav_data_path, o, d, fl);
            let direct = nav::haversine_nm(o.0, o.1, d.0, d.1);
            println!(
                "{name} FL{}: direct {direct:.0} nm, {:?} in {:?}",
                fl / 100.0,
                r.as_ref()
                    .map(|r| (r.route_string(), r.distance_nm.round(), r.airway_nm.round())),
                t.elapsed()
            );
        }
        for (name, o, d, alt) in [
            ("LXGB-LEMG", (36.151, -5.349), (36.675, -4.499), 3500.0),
            ("KGAI-KESN", (39.168, -77.166), (38.804, -76.069), 4500.0),
            ("LFPN-LFAT", (48.751, 2.106), (50.517, 1.620), 4500.0),
            ("LFPN-LFOB", (48.751, 2.106), (49.454, 2.113), 2500.0),
        ] {
            let t = std::time::Instant::now();
            let spaces = load_spaces(&c.airspace_path, o, d);
            let dl = dog_leg(o, d, &spaces, &vfr(alt));
            println!(
                "{name} at {alt}: {} volumes near, direct {:.0} nm, {dl:?} in {:?}",
                spaces.len(),
                nav::haversine_nm(o.0, o.1, d.0, d.1),
                t.elapsed()
            );
        }
    }
    /// #74 over the real data: field pairs from apt.dat (Europe, hard
    /// runway of 800 m or more, 25 to 100 nm apart) whose DIRECT line meets
    /// a P/R/Q volume from the sim's `airspace.txt` at the altitude the
    /// flight is at there (VFR, 4,500 ft cruise), not counting a volume over
    /// either field. Each is planned by `dog_leg` and must either go round
    /// (no blocking volume anywhere along the flown legs, within 1.30x
    /// direct; the passes from the volumes gone round are printed) or fly direct and NAME
    /// what it enters (every name one the direct line really meets); never
    /// direct through a volume in silence. `XVL_DOGLEG_PAIRS` caps the pairs
    /// (default 400).
    /// `cargo test --release -- --ignored real_install_dog_legs --nocapture`
    #[test]
    #[ignore] // needs the real X-Plane install; run with --ignored
    fn real_install_dog_legs_go_round_or_name_it() {
        let c = crate::config::Config::from_env().expect("config");
        if !std::path::Path::new(&c.airspace_path).exists()
            || !std::path::Path::new(&c.apt_data_path).exists()
        {
            return;
        }
        let cap: usize = std::env::var("XVL_DOGLEG_PAIRS")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(400);
        let (la0, la1, lo0, lo1) = (36.0, 58.0, -10.0, 25.0);
        let mut fields: Vec<(String, (f64, f64))> = Vec::new();
        nav::scan_airports(&c.apt_data_path, |icao, a| {
            let Some(p) = a.position() else { return };
            let hard = a.runways.iter().any(|r| r.is_hard() && r.length_m() >= 800.0);
            if hard
                && icao.len() == 4
                && icao.chars().all(|ch| ch.is_ascii_uppercase())
                && (la0..=la1).contains(&p.0)
                && (lo0..=lo1).contains(&p.1)
            {
                fields.push((icao.to_string(), p));
            }
        })
        .unwrap();
        fields.sort_by(|a, b| a.0.cmp(&b.0));
        let all = crate::airspace::load_box(&c.airspace_path, la0 - 2.0, la1 + 2.0, lo0 - 3.0, lo1 + 3.0);
        let bbox: Vec<(f64, f64, f64, f64)> = all
            .iter()
            .map(|a| {
                a.poly.iter().fold((f64::MAX, f64::MIN, f64::MAX, f64::MIN), |b, (la, lo)| {
                    (b.0.min(*la), b.1.max(*la), b.2.min(*lo), b.3.max(*lo))
                })
            })
            .collect();
        println!("{} fields, {} P/R/Q volumes", fields.len(), all.len());
        let profile = Profile {
            origin_elev_ft: 0.0,
            dest_elev_ft: 0.0,
            cruise_ft: 4500.0,
            climb_ft_per_nm: Some(700.0 / 1.5),
            descent_ft_per_nm: Some(250.0),
        };
        // Planar nm from p to the polygon's nearest edge.
        let edge_nm = |a: &Airspace, p: (f64, f64)| {
            let k = p.0.to_radians().cos();
            let xy = |q: (f64, f64)| ((q.1 - p.1) * 60.0 * k, (q.0 - p.0) * 60.0);
            let n = a.poly.len();
            (0..n)
                .map(|i| {
                    let (ax, ay) = xy(a.poly[i]);
                    let (bx, by) = xy(a.poly[(i + 1) % n]);
                    let (dx, dy) = (bx - ax, by - ay);
                    let l2 = dx * dx + dy * dy;
                    let t = if l2 > 0.0 { (-(ax * dx + ay * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
                    (ax + t * dx).hypot(ay + t * dy)
                })
                .fold(f64::INFINITY, f64::min)
        };
        let (mut pairs, mut round, mut named) = (0usize, 0usize, 0usize);
        let (mut detours, mut clear_min, mut bad) = (Vec::new(), f64::INFINITY, Vec::new());
        let (mut clear_at, mut near_fields, mut vertical) = (String::new(), 0usize, 0usize);
        let mut clears: Vec<f64> = Vec::new();
        let mut examples: Vec<String> = Vec::new();
        'outer: for (i, (oi, o)) in fields.iter().enumerate() {
            let o = *o;
            // A few partners each, spread through the list.
            for (di, d) in fields.iter().skip(i + 1).step_by(7).map(|(n, p)| (n, *p)) {
                let direct = nav::haversine_nm(o.0, o.1, d.0, d.1);
                if !(25.0..=100.0).contains(&direct) {
                    continue;
                }
                // `load_spaces`'s box, over the preloaded file.
                let half = direct * DOGLEG_MAX_DETOUR / 2.0 / 60.0 + 0.2;
                let k = ((o.0 + d.0) / 2.0).to_radians().cos().max(0.2);
                let (bl0, bl1) = (o.0.min(d.0) - half, o.0.max(d.0) + half);
                let (bo0, bo1) = (o.1.min(d.1) - half / k, o.1.max(d.1) + half / k);
                let spaces: Vec<Airspace> = all
                    .iter()
                    .zip(&bbox)
                    .filter(|(_, b)| b.0 <= bl1 && b.1 >= bl0 && b.2 <= bo1 && b.3 >= bo0)
                    .map(|(a, _)| a.clone())
                    .collect();
                // Volumes over a field are no dog-leg's business.
                let over_field = |a: &Airspace| a.contains(o.0, o.1) || a.contains(d.0, d.1);
                let meets = |path: &[(f64, f64)]| -> Vec<usize> {
                    let total = path_nm(path);
                    let steps = (total / 0.25).ceil().max(1.0) as usize;
                    let mut out = Vec::new();
                    for s in 0..=steps {
                        let at = total * s as f64 / steps as f64;
                        let p = point_along(path, at);
                        let band = profile.band(at, total);
                        for (k, a) in spaces.iter().enumerate() {
                            if !out.contains(&k) && !over_field(a) && blocks_band(a, p, band) {
                                out.push(k);
                            }
                        }
                    }
                    out
                };
                let on_direct = meets(&[o, d]);
                if on_direct.is_empty() {
                    continue;
                }
                pairs += 1;
                let dl = dog_leg(o, d, &spaces, &profile);
                let at_fields: HashSet<&String> = dl.at_fields.iter().collect();
                let name = |k: usize| spaces[k].spoken_name();
                if dl.turns.is_empty() {
                    // Direct: everything it meets is named (or over a field
                    // by the planner's own hull test).
                    let silent: Vec<String> = on_direct
                        .iter()
                        .map(|k| name(*k))
                        .filter(|n| !dl.crossed.contains(n) && !at_fields.contains(n))
                        .collect();
                    let unmet: Vec<&String> = dl
                        .crossed
                        .iter()
                        .filter(|n| !on_direct.iter().any(|k| name(*k) == **n))
                        .collect();
                    if !silent.is_empty() || !unmet.is_empty() {
                        bad.push(format!("{oi}-{di}: direct, silent through {silent:?}, named but not met {unmet:?}"));
                    } else {
                        named += 1;
                        if examples.len() < 6 && !dl.crossed.is_empty() {
                            examples.push(format!("{oi}-{di} {direct:.0} nm: direct, names {:?}", dl.crossed));
                        }
                    }
                    continue;
                }
                round += 1;
                let mut path = vec![o];
                path.extend(dl.turns.iter().copied());
                path.push(d);
                let flown = path_nm(&path);
                detours.push(flown / direct);
                let still: Vec<String> = meets(&path)
                    .into_iter()
                    .map(name)
                    .filter(|n| !at_fields.contains(n))
                    .collect();
                if !dl.crossed.is_empty() || !still.is_empty() || flown > direct * DOGLEG_MAX_DETOUR + 0.01 {
                    bad.push(format!(
                        "{oi}-{di}: round {:?} {flown:.1}/{direct:.1} nm, still enters {still:?}, crossed {:?}",
                        dl.avoided, dl.crossed
                    ));
                    continue;
                }
                // Clearance from each volume flown round, along the legs;
                // one within the margin of a field cannot be passed with the
                // margin, so it is counted apart.
                let total = flown;
                let steps = (total / 0.1).ceil() as usize;
                for k in on_direct.iter().filter(|k| dl.avoided.contains(&name(**k))) {
                    let a = &spaces[*k];
                    let near_field =
                        edge_nm(a, o).min(edge_nm(a, d)) < DOGLEG_MARGIN_NM;
                    let (mut c, mut under) = (f64::INFINITY, false);
                    for s in 0..=steps {
                        let p = point_along(&path, total * s as f64 / steps as f64);
                        c = c.min(edge_nm(a, p));
                        under |= a.contains(p.0, p.1);
                    }
                    if near_field {
                        near_fields += 1;
                    } else if under {
                        // Inside its outline, outside its band (checked
                        // above): passed over or under, not round.
                        vertical += 1;
                    } else {
                        clears.push(c);
                        if c < clear_min {
                            clear_min = c;
                            clear_at = format!("{oi}-{di} {}", name(*k));
                        }
                    }
                }
                if examples.len() < 12 {
                    examples.push(format!(
                        "{oi}-{di} {direct:.0} nm: round {:?}, {} turns, {flown:.0} nm ({:.2}x)",
                        dl.avoided,
                        dl.turns.len(),
                        flown / direct
                    ));
                }
                if pairs >= cap {
                    break 'outer;
                }
            }
        }
        detours.sort_by(f64::total_cmp);
        println!(
            "{pairs} pairs meet a volume direct: {round} flown round (detour median {:.2}x, max {:.2}x, \
             nearest pass {clear_min:.2} nm at {clear_at}, {near_fields} volumes within the margin \
             of a field not measured, {vertical} passed over or under), {named} direct and named, {} wrong",
            detours.get(detours.len() / 2).copied().unwrap_or(f64::NAN),
            detours.last().copied().unwrap_or(f64::NAN),
            bad.len()
        );
        // How far the legs pass from the named volumes they were flown round
        // (a same-named volume can be one the final legs no longer meet in
        // its band, passed at any distance): reported, not asserted.
        println!(
            "passes from volumes named as flown round: {} measured, {} under 1.5 nm, {} under 1.9 nm",
            clears.len(),
            clears.iter().filter(|c| **c < 1.5).count(),
            clears.iter().filter(|c| **c < 1.9).count()
        );
        for e in &examples {
            println!("  {e}");
        }
        for b in &bad {
            println!("  BAD {b}");
        }
        assert!(round > 0, "no pair was flown round");
        assert!(bad.is_empty(), "{} pairs", bad.len());
    }
}
