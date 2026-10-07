//! The Airport page: the field that matters now as a diagram, and the live
//! picture over it (the aircraft, the taxi route held, the turn card, the
//! cleared runway).
//!
//! Two halves, both readers of what the app already holds (CLAUDE.md "EFB",
//! no new sim reads):
//!
//! - The GEOMETRY ([`Diagram`], `GET /airport`): apt.dat as the nav crate
//!   already parses it (`nav::find_airport` for the runways,
//!   `nav::find_taxi_net` for the 110 pavement, the 120 centrelines, the
//!   1201/1202 network with its 1204 zones, the 1300 stands), put in a LOCAL
//!   metric frame (east and north metres from the field's reference point,
//!   [`Frame`]) so the page does no geodesy. Built once per field off the
//!   runtime's thread and kept ([`CACHE`]); a version (a hash of the
//!   document) lets the page fetch it only when the field changes.
//! - The OVERLAY ([`page`], pure, in the page state): the monitor's tick
//!   position, the cleared or crew's own taxi route (`party_line`), the
//!   taxi guide's turn card as the panel draws it (`taxi_guide::TurnCard`),
//!   the cleared runway, projected into the same frame.
//!
//! Headings are TRUE, distances metres. Route lengths are magnitudes; the
//! card's cross-track keeps its sign (positive right of the route).

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde::Serialize;

use crate::efb::feed::Tick;
use crate::efb::state::PlanFacts;
use crate::nav::{AirportInfo, TaxiNet};

/// Metres per degree on the nav crate's haversine sphere (3,440.065 nm),
/// so a length on the diagram is the length `Runway::length_m` says.
const M_PER_DEG: f64 = 3440.065 * 1852.0 * std::f64::consts::PI / 180.0;

/// On the ground within this of a field (its reference point), we are at it.
pub(crate) const ON_FIELD_NM: f64 = 5.0;

/// The aircraft is drawn on the diagram only within this of the reference
/// point: further out the dot is off the chart anyway.
pub(crate) const OWN_MAX_M: f64 = 30_000.0;

/// A route is drawn only when it starts within this of the reference point:
/// a route held at another field is not this field's.
pub(crate) const ROUTE_MAX_M: f64 = 15_000.0;

/// Douglas-Peucker tolerance for pavement rings and centrelines: well under
/// a pixel at any zoom a pilot reads a diagram at.
const SIMPLIFY_M: f64 = 0.4;

/// Nominal width of a taxiway drawn from the network where the field draws
/// no pavement (1202 rows carry no width).
pub(crate) const NETWORK_WIDTH_M: f64 = 20.0;

/// Runway width when the 100 row says nothing.
const RUNWAY_WIDTH_DEFAULT_M: f64 = 30.0;

// ---------------------------------------------------------------- frame

/// The local frame: east (x) and north (y) metres from a reference point.
/// Equirectangular about the reference latitude: across an airport (a few
/// km) the error is centimetres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Frame {
    pub(crate) lat0: f64,
    pub(crate) lon0: f64,
    k_lon: f64,
}

impl Frame {
    pub(crate) fn new(lat0: f64, lon0: f64) -> Frame {
        Frame {
            lat0,
            lon0,
            k_lon: M_PER_DEG * lat0.to_radians().cos(),
        }
    }

    /// (east, north) metres of a point.
    pub(crate) fn xy(&self, lat: f64, lon: f64) -> (f64, f64) {
        (
            (lon - self.lon0) * self.k_lon,
            (lat - self.lat0) * M_PER_DEG,
        )
    }

    /// The field's reference point: the middle of its runways' extent (apt.dat
    /// has no single airport position row). `None` with no runway.
    pub(crate) fn for_airport(apt: &AirportInfo) -> Option<Frame> {
        let mut lat = (f64::INFINITY, f64::NEG_INFINITY);
        let mut lon = (f64::INFINITY, f64::NEG_INFINITY);
        for r in &apt.runways {
            for (a, o) in [(r.lat_a, r.lon_a), (r.lat_b, r.lon_b)] {
                lat = (lat.0.min(a), lat.1.max(a));
                lon = (lon.0.min(o), lon.1.max(o));
            }
        }
        lat.0
            .is_finite()
            .then(|| Frame::new((lat.0 + lat.1) / 2.0, (lon.0 + lon.1) / 2.0))
    }
}

/// One decimal: a decimetre is finer than any line on the page.
fn r1(v: f64) -> f64 {
    (v * 10.0).round() / 10.0
}

/// A polyline as the wire carries it: [x0, y0, x1, y1, ...], decimetres.
fn flat(pts: &[(f64, f64)]) -> Vec<f64> {
    pts.iter().flat_map(|&(x, y)| [r1(x), r1(y)]).collect()
}

/// Distance from `p` to the segment `a`-`b`, and the parameter of the foot.
fn seg_foot(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 0.0 {
        (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let (fx, fy) = (a.0 + t * dx, a.1 + t * dy);
    (((p.0 - fx).powi(2) + (p.1 - fy).powi(2)).sqrt(), t)
}

/// Douglas-Peucker: the fewest points within `tol` of the line.
pub(crate) fn simplify(pts: &[(f64, f64)], tol: f64) -> Vec<(f64, f64)> {
    if pts.len() < 3 {
        return pts.to_vec();
    }
    let mut keep = vec![false; pts.len()];
    keep[0] = true;
    keep[pts.len() - 1] = true;
    let mut stack = vec![(0usize, pts.len() - 1)];
    while let Some((a, b)) = stack.pop() {
        if b <= a + 1 {
            continue;
        }
        let (mut worst, mut at) = (0.0, a);
        for (i, &p) in pts.iter().enumerate().take(b).skip(a + 1) {
            let (d, _) = seg_foot(p, pts[a], pts[b]);
            if d > worst {
                worst = d;
                at = i;
            }
        }
        if worst > tol {
            keep[at] = true;
            stack.push((a, at));
            stack.push((at, b));
        }
    }
    pts.iter()
        .zip(keep)
        .filter(|(_, k)| *k)
        .map(|(p, _)| *p)
        .collect()
}

// ---------------------------------------------------------------- geometry

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct End {
    pub(crate) id: String,
    pub(crate) x: f64,
    pub(crate) y: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct RunwayGeo {
    pub(crate) a: End,
    pub(crate) b: End,
    pub(crate) width_m: f64,
    pub(crate) length_m: f64,
    /// Asphalt or concrete (grass, dirt and water draw differently).
    pub(crate) hard: bool,
}

/// A taxi-network edge, drawn as the pavement where the field draws none.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct EdgeGeo {
    pub(crate) name: String,
    pub(crate) pts: Vec<f64>,
}

/// A taxiway name and where to print it: on one of its edges, along it.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Label {
    pub(crate) name: String,
    pub(crate) x: f64,
    pub(crate) y: f64,
    /// The edge's true bearing, folded into 0..180.
    pub(crate) brg: f64,
}

/// A runway holding position: where an active zone (apt.dat 1204) meets the
/// taxiway outside it.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct HoldGeo {
    pub(crate) x: f64,
    pub(crate) y: f64,
    /// The runway(s) held short of, "13/31".
    pub(crate) rwy: String,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct StandGeo {
    pub(crate) name: String,
    pub(crate) x: f64,
    pub(crate) y: f64,
    /// True heading the aircraft parks on.
    pub(crate) hdg: f64,
    /// apt.dat location type: gate, hangar, tie_down, misc.
    pub(crate) kind: String,
}

/// Where each layer came from, said on the page.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Sources {
    pub(crate) pavement: &'static str,
    pub(crate) lines: &'static str,
}

/// One field's diagram, `GET /airport`.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Diagram {
    pub(crate) icao: String,
    pub(crate) name: String,
    /// A hash of the rest of the document (the page's ETag).
    pub(crate) version: String,
    pub(crate) ref_lat: f64,
    pub(crate) ref_lon: f64,
    pub(crate) elev_ft: f64,
    pub(crate) sources: Sources,
    /// [min x, min y, max x, max y], metres.
    pub(crate) bounds: [f64; 4],
    pub(crate) runways: Vec<RunwayGeo>,
    /// Polygons, each its rings (the outline, then holes), flat.
    pub(crate) pavement: Vec<Vec<Vec<f64>>>,
    /// The network, only where the field draws no pavement.
    pub(crate) network: Vec<EdgeGeo>,
    /// The width the network is drawn at, metres.
    pub(crate) network_width_m: f64,
    /// The drawn taxi centrelines (apt.dat 120, line types 1 and 51).
    pub(crate) lines: Vec<Vec<f64>>,
    pub(crate) taxiways: Vec<Label>,
    pub(crate) holds: Vec<HoldGeo>,
    pub(crate) stands: Vec<StandGeo>,
}

/// The holding positions: network nodes where an edge inside a runway's
/// active zone meets a taxiway edge outside every zone. Runway edges are
/// the strip itself and never count.
pub(crate) fn holds(net: &TaxiNet, frame: &Frame) -> Vec<HoldGeo> {
    use std::collections::HashMap;
    let mut zoned: HashMap<i64, Vec<String>> = HashMap::new();
    let mut free: std::collections::HashSet<i64> = Default::default();
    for e in net.edges.iter().filter(|e| !e.runway) {
        for n in [e.a, e.b] {
            if e.zone_of.is_empty() {
                free.insert(n);
            } else {
                let z = zoned.entry(n).or_default();
                for r in &e.zone_of {
                    if !z.contains(r) {
                        z.push(r.clone());
                    }
                }
            }
        }
    }
    let mut ids: Vec<&i64> = zoned.keys().filter(|n| free.contains(n)).collect();
    ids.sort();
    let mut out: Vec<HoldGeo> = Vec::new();
    for id in ids {
        let Some(&(lat, lon)) = net.nodes.get(id) else {
            continue;
        };
        let (x, y) = frame.xy(lat, lon);
        if out.iter().any(|h| (h.x - x).hypot(h.y - y) < 15.0) {
            continue;
        }
        let mut rwys = zoned[id].clone();
        rwys.sort_by_key(|r| crate::nav::runway_number(r).unwrap_or(99));
        out.push(HoldGeo {
            x: r1(x),
            y: r1(y),
            rwy: rwys.join("/"),
        });
    }
    out
}

/// Where to print each taxiway's name: on its longest edges first, no two
/// of one name within 350 m, no two labels within 40 m, at most eight of a
/// name.
pub(crate) fn taxiway_labels(net: &TaxiNet, frame: &Frame) -> Vec<Label> {
    let mut edges: Vec<(String, (f64, f64), (f64, f64), f64)> = net
        .edges
        .iter()
        .filter(|e| !e.runway && !e.name.trim().is_empty())
        .filter_map(|e| {
            let a = net.nodes.get(&e.a)?;
            let b = net.nodes.get(&e.b)?;
            let (pa, pb) = (frame.xy(a.0, a.1), frame.xy(b.0, b.1));
            let len = (pb.0 - pa.0).hypot(pb.1 - pa.1);
            (len >= 25.0).then(|| (e.name.trim().to_string(), pa, pb, len))
        })
        .collect();
    edges.sort_by(|a, b| b.3.total_cmp(&a.3).then_with(|| a.0.cmp(&b.0)));
    let mut out: Vec<Label> = Vec::new();
    for (name, pa, pb, _) in edges {
        let (x, y) = ((pa.0 + pb.0) / 2.0, (pa.1 + pb.1) / 2.0);
        let mine = out.iter().filter(|l| l.name == name).count();
        if mine >= 8
            || out
                .iter()
                .any(|l| l.name == name && (l.x - x).hypot(l.y - y) < 350.0)
            || out.iter().any(|l| (l.x - x).hypot(l.y - y) < 40.0)
        {
            continue;
        }
        let brg = (pb.0 - pa.0)
            .atan2(pb.1 - pa.1)
            .to_degrees()
            .rem_euclid(180.0);
        out.push(Label {
            name,
            x: r1(x),
            y: r1(y),
            brg: r1(brg),
        });
    }
    out
}

/// The diagram of a field, from what the nav crate parsed. Pure.
pub(crate) fn build_diagram(
    icao: &str,
    apt: &AirportInfo,
    net: Option<&TaxiNet>,
) -> Option<Diagram> {
    let frame = Frame::for_airport(apt)?;
    let mut bounds = [
        f64::INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NEG_INFINITY,
    ];
    let mut grow = |x: f64, y: f64| {
        bounds[0] = bounds[0].min(x);
        bounds[1] = bounds[1].min(y);
        bounds[2] = bounds[2].max(x);
        bounds[3] = bounds[3].max(y);
    };
    let runways: Vec<RunwayGeo> = apt
        .runways
        .iter()
        .map(|r| {
            let (ax, ay) = frame.xy(r.lat_a, r.lon_a);
            let (bx, by) = frame.xy(r.lat_b, r.lon_b);
            grow(ax, ay);
            grow(bx, by);
            RunwayGeo {
                a: End {
                    id: r.ident_a.clone(),
                    x: r1(ax),
                    y: r1(ay),
                },
                b: End {
                    id: r.ident_b.clone(),
                    x: r1(bx),
                    y: r1(by),
                },
                width_m: if r.width_m > 0.0 {
                    r1(r.width_m)
                } else {
                    RUNWAY_WIDTH_DEFAULT_M
                },
                length_m: r.length_m().round(),
                hard: r.is_hard(),
            }
        })
        .collect();
    let mut pavement = Vec::new();
    let mut network = Vec::new();
    let mut lines = Vec::new();
    let (mut taxiways, mut holds_out, mut stands) = (Vec::new(), Vec::new(), Vec::new());
    let mut sources = Sources {
        pavement: "none: the field has no taxi network in apt.dat",
        lines: "none",
    };
    if let Some(net) = net {
        let to_xy = |pts: &[(f64, f64)]| -> Vec<(f64, f64)> {
            pts.iter().map(|&(a, o)| frame.xy(a, o)).collect()
        };
        for poly in &net.pavement {
            let rings: Vec<Vec<f64>> = poly
                .iter()
                .map(|r| simplify(&to_xy(r), SIMPLIFY_M))
                .filter(|r| r.len() >= 3)
                .map(|r| {
                    for &(x, y) in &r {
                        grow(x, y);
                    }
                    flat(&r)
                })
                .collect();
            if !rings.is_empty() {
                pavement.push(rings);
            }
        }
        if pavement.is_empty() {
            sources.pavement =
                "apt.dat 1202 taxi network, drawn at a nominal width (the field draws no 110 pavement)";
            for e in net.edges.iter().filter(|e| !e.runway) {
                let (Some(a), Some(b)) = (net.nodes.get(&e.a), net.nodes.get(&e.b)) else {
                    continue;
                };
                let pts = to_xy(&[*a, *b]);
                for &(x, y) in &pts {
                    grow(x, y);
                }
                network.push(EdgeGeo {
                    name: e.name.trim().to_string(),
                    pts: flat(&pts),
                });
            }
        } else {
            sources.pavement = "apt.dat 110 pavement polygons";
        }
        for l in &net.paint {
            let s = simplify(&to_xy(l), SIMPLIFY_M);
            if s.len() >= 2 {
                lines.push(flat(&s));
            }
        }
        sources.lines = if lines.is_empty() {
            "none: the field draws no taxi centrelines"
        } else {
            "apt.dat 120 taxi centrelines"
        };
        taxiways = taxiway_labels(net, &frame);
        holds_out = holds(net, &frame);
        stands = net
            .stands
            .iter()
            .map(|s| {
                let (x, y) = frame.xy(s.lat, s.lon);
                StandGeo {
                    name: s.name.clone(),
                    x: r1(x),
                    y: r1(y),
                    hdg: r1(s.heading.rem_euclid(360.0)),
                    kind: s.kind.clone(),
                }
            })
            .collect();
        for s in &stands {
            grow(s.x, s.y);
        }
    }
    let mut d = Diagram {
        icao: icao.to_ascii_uppercase(),
        name: apt.name.clone(),
        version: String::new(),
        ref_lat: frame.lat0,
        ref_lon: frame.lon0,
        elev_ft: apt.elev_ft,
        sources,
        bounds: bounds.map(r1),
        runways,
        pavement,
        network,
        network_width_m: NETWORK_WIDTH_M,
        lines,
        taxiways,
        holds: holds_out,
        stands,
    };
    d.version = version_of(&serde_json::to_string(&d).unwrap_or_default());
    Some(d)
}

/// FNV-1a over the document: the page's ETag.
pub(crate) fn version_of(s: &str) -> String {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in s.bytes() {
        h ^= b as u64;
        h = h.wrapping_mul(0x0100_0000_01b3);
    }
    format!("{h:016x}")
}

// ---------------------------------------------------------------- cache

/// What the page state needs of a cached diagram: which field, which
/// version, and its frame.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct DiagramRef {
    pub(crate) icao: String,
    pub(crate) name: String,
    pub(crate) version: String,
    pub(crate) lat0: f64,
    pub(crate) lon0: f64,
    /// Every runway end of the field, as drawn (the Flight tab's available
    /// length and the wind's components).
    pub(crate) runways: Vec<RunwayEnd>,
}

/// One runway end of a drawn field: its ident, the strip's length and the
/// true heading flown along it from this end.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct RunwayEnd {
    pub(crate) ident: String,
    pub(crate) length_m: f64,
    pub(crate) heading_true: f64,
}

/// Both ends of every runway in a diagram. Pure.
pub(crate) fn runway_ends(d: &Diagram) -> Vec<RunwayEnd> {
    let mut out = Vec::new();
    for r in &d.runways {
        let ab = (r.b.x - r.a.x)
            .atan2(r.b.y - r.a.y)
            .to_degrees()
            .rem_euclid(360.0);
        out.push(RunwayEnd {
            ident: r.a.id.clone(),
            length_m: r.length_m,
            heading_true: ab,
        });
        out.push(RunwayEnd {
            ident: r.b.id.clone(),
            length_m: r.length_m,
            heading_true: (ab + 180.0).rem_euclid(360.0),
        });
    }
    out
}

/// A runway end by ident ("08R", "RW08R", "8R" all name 08R).
pub(crate) fn runway_end<'a>(ends: &'a [RunwayEnd], ident: &str) -> Option<&'a RunwayEnd> {
    let norm = |s: &str| {
        let s = s.trim().to_ascii_uppercase();
        let s = s.strip_prefix("RW").unwrap_or(&s).to_string();
        s.trim_start_matches('0').to_string()
    };
    let want = norm(ident);
    (!want.is_empty())
        .then(|| ends.iter().find(|e| norm(&e.ident) == want))
        .flatten()
}

pub(crate) struct Ready {
    pub(crate) reference: DiagramRef,
    pub(crate) json: String,
}

/// The diagrams built, newest last, the one being built, and the fields
/// that could not be (with when, to retry later).
struct Cache {
    ready: Vec<Arc<Ready>>,
    loading: Option<(String, Instant)>,
    failed: Vec<(String, Instant)>,
}

/// Fields kept: the departure, the destination and one spare.
const CACHE_FIELDS: usize = 3;

/// A field that could not be built is tried again after this.
const RETRY: Duration = Duration::from_secs(120);

/// A build still running after this is given up on (its thread died).
const BUILD_MAX: Duration = Duration::from_secs(60);

/// Built diagrams by ICAO (a memo of a file read: apt.dat geometry holds no
/// flight fact, so it outlives every flight).
static CACHE: Mutex<Cache> = Mutex::new(Cache {
    ready: Vec::new(),
    loading: None,
    failed: Vec::new(),
});

fn cache() -> std::sync::MutexGuard<'static, Cache> {
    CACHE.lock().unwrap_or_else(|e| e.into_inner())
}

/// The built diagram of a field, if it is.
pub(crate) fn lookup(icao: &str) -> Option<Arc<Ready>> {
    cache()
        .ready
        .iter()
        .find(|r| r.reference.icao == icao)
        .cloned()
}

/// The field could not be built (no apt.dat entry, no runway), lately.
pub(crate) fn unavailable(icao: &str) -> bool {
    cache()
        .failed
        .iter()
        .any(|(i, at)| i == icao && at.elapsed() < RETRY)
}

fn store(icao: &str, built: Option<Diagram>) {
    let mut c = cache();
    if c.loading.as_ref().is_some_and(|(i, _)| i == icao) {
        c.loading = None;
    }
    match built {
        Some(d) => {
            let reference = DiagramRef {
                icao: d.icao.clone(),
                name: d.name.clone(),
                version: d.version.clone(),
                lat0: d.ref_lat,
                lon0: d.ref_lon,
                runways: runway_ends(&d),
            };
            let json = serde_json::to_string(&d).unwrap_or_default();
            c.ready.retain(|r| r.reference.icao != icao);
            c.ready.push(Arc::new(Ready { reference, json }));
            while c.ready.len() > CACHE_FIELDS {
                c.ready.remove(0);
            }
            c.failed.retain(|(i, _)| i != icao);
        }
        None => {
            c.failed.retain(|(i, _)| i != icao);
            c.failed.push((icao.to_string(), Instant::now()));
        }
    }
}

/// Two apt.dat scans (the runways, then the network): a disk read, never
/// the sim, on a blocking thread.
pub(crate) fn load(apt_path: &str, icao: &str) -> Option<Diagram> {
    let apt = crate::nav::find_airport(apt_path, icao).ok().flatten()?;
    let net = crate::nav::find_taxi_net(apt_path, icao).ok().flatten();
    build_diagram(icao, &apt, net.as_ref())
}

/// Build the field's diagram unless it is built, building, or failed lately.
/// Never waits: the page shows "loading" until it lands.
pub(crate) fn ensure(icao: &str) {
    {
        let mut c = cache();
        if c.ready.iter().any(|r| r.reference.icao == icao)
            || c.loading
                .as_ref()
                .is_some_and(|(_, at)| at.elapsed() < BUILD_MAX)
            || c.failed
                .iter()
                .any(|(i, at)| i == icao && at.elapsed() < RETRY)
        {
            return;
        }
        c.loading = Some((icao.to_string(), Instant::now()));
    }
    let icao = icao.to_string();
    let path = crate::settings::live().apt_data_path.clone();
    let started = Instant::now();
    tokio::task::spawn_blocking(move || {
        let built = load(&path, &icao);
        tracing::info!(
            icao = %icao,
            ok = built.is_some(),
            ms = started.elapsed().as_millis() as u64,
            "EFB airport diagram built"
        );
        crate::blackbox::record(
            "efb",
            serde_json::json!({
                "event": "diagram", "icao": &icao, "ok": built.is_some(),
                "ms": started.elapsed().as_millis() as u64,
                "pavement": built.as_ref().map(|d| d.sources.pavement),
            }),
        );
        store(&icao, built);
    });
}

// ---------------------------------------------------------------- facts

/// The taxi route the crew is driving or was cleared, as the party line
/// holds it (`party_line::with_taxi_route`).
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct RouteFacts {
    /// "cleared" (a controller's) or "crew" (the crew's own at an
    /// uncontrolled field).
    pub(crate) source: &'static str,
    pub(crate) path: Vec<(f64, f64)>,
    /// `seg[i]` names the leg `path[i] -> path[i+1]` (name, is a runway).
    pub(crate) seg: Vec<(String, bool)>,
    pub(crate) via: String,
    pub(crate) holding_point: String,
    /// The route ends at a stand, not a runway hold.
    pub(crate) to_stand: bool,
}

/// The guide's turn card as the monitor computed it for the panel.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct CardFacts {
    pub(crate) kind: char,
    pub(crate) dist_m: f64,
    pub(crate) label: String,
    pub(crate) aim_true_deg: f64,
    pub(crate) xtk_m: f64,
    pub(crate) turn_deg: f64,
    pub(crate) remaining_m: f64,
}

impl CardFacts {
    pub(crate) fn of(c: &crate::taxi_guide::TurnCard) -> CardFacts {
        CardFacts {
            kind: c.kind,
            dist_m: c.dist_m,
            label: c.label.clone(),
            aim_true_deg: c.aim_true_deg,
            xtk_m: c.xtk_m,
            turn_deg: c.turn_deg,
            remaining_m: c.remaining_m,
        }
    }
}

/// Everything the Airport page reads beyond the tick and the plan.
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct AirportFacts {
    /// The active station's field: ICAO, lat, lon (`crate::party_line::active_icao`).
    pub(crate) station: Option<(String, f64, f64)>,
    /// The diagram built for the field the page wants, when it is.
    pub(crate) diagram: Option<DiagramRef>,
    /// The wanted field could not be built.
    pub(crate) unavailable: bool,
    pub(crate) route: Option<RouteFacts>,
    pub(crate) card: Option<CardFacts>,
    /// The runway END of the standing clearance (`party_line::clearance_runway`).
    pub(crate) cleared_runway: Option<String>,
    /// Full auto's arrival runway (`full_auto::arrival_runway`).
    pub(crate) arrival_runway: Option<String>,
}

// ---------------------------------------------------------------- choice

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct FieldChoice {
    pub(crate) icao: String,
    /// "departure", "destination" or "here" (the field we are at, or the
    /// station's).
    pub(crate) why: &'static str,
}

fn plan_field(p: &PlanFacts, icao: &str) -> Option<(f64, f64)> {
    p.waypoints
        .iter()
        .find(|w| w.0.eq_ignore_ascii_case(icao))
        .map(|w| (w.1, w.2))
}

fn clean(icao: &str) -> Option<String> {
    let i = icao.trim().to_ascii_uppercase();
    (!i.is_empty()).then_some(i)
}

/// The field the page draws. Airborne: the plan's destination, else the
/// station's field. On the ground: the field we are at (the plan's origin
/// or destination, or the station's, nearest within [`ON_FIELD_NM`]), else
/// the departure, else the station's. Pure.
pub(crate) fn wanted_field(
    a: &AirportFacts,
    tick: Option<&Tick>,
    plan: Option<&PlanFacts>,
    airborne: bool,
) -> Option<FieldChoice> {
    let station = a
        .station
        .as_ref()
        .and_then(|(i, la, lo)| clean(i).map(|i| (i, *la, *lo)));
    let dest = plan.and_then(|p| clean(&p.destination));
    let origin = plan.and_then(|p| clean(&p.origin));
    if airborne {
        return dest
            .map(|icao| FieldChoice {
                icao,
                why: "destination",
            })
            .or(station.map(|(icao, _, _)| FieldChoice { icao, why: "here" }));
    }
    if let Some(t) = tick {
        let mut near: Vec<(f64, FieldChoice)> = Vec::new();
        let mut consider = |icao: &str, pos: Option<(f64, f64)>, why: &'static str| {
            if let Some((la, lo)) = pos {
                let d = crate::nav::haversine_nm(t.lat, t.lon, la, lo);
                if d <= ON_FIELD_NM {
                    near.push((
                        d,
                        FieldChoice {
                            icao: icao.to_string(),
                            why,
                        },
                    ));
                }
            }
        };
        if let (Some(p), Some(o)) = (plan, origin.as_deref()) {
            consider(o, plan_field(p, o), "departure");
        }
        if let (Some(p), Some(d)) = (plan, dest.as_deref()) {
            consider(d, plan_field(p, d), "destination");
        }
        // The station's field is "here" only when the plan does not name it.
        if let Some((i, la, lo)) = &station {
            if origin.as_deref() != Some(i.as_str()) && dest.as_deref() != Some(i.as_str()) {
                consider(i, Some((*la, *lo)), "here");
            } else if plan.is_some_and(|p| plan_field(p, i).is_none()) {
                let why = if origin.as_deref() == Some(i.as_str()) {
                    "departure"
                } else {
                    "destination"
                };
                consider(i, Some((*la, *lo)), why);
            }
        }
        if let Some((_, c)) = near.into_iter().min_by(|a, b| a.0.total_cmp(&b.0)) {
            return Some(c);
        }
    }
    origin
        .map(|icao| FieldChoice {
            icao,
            why: "departure",
        })
        .or(station.map(|(icao, _, _)| FieldChoice { icao, why: "here" }))
}

// ---------------------------------------------------------------- overlay

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Own {
    pub(crate) x: f64,
    pub(crate) y: f64,
    /// True heading (the monitor's `psi`, best-effort: 0 on a failed read).
    pub(crate) heading: f64,
    /// Ground track, true, when the read held.
    pub(crate) track: Option<f64>,
    pub(crate) gs_kts: f64,
    pub(crate) on_ground: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Mark {
    pub(crate) x: f64,
    pub(crate) y: f64,
    pub(crate) label: String,
    /// "hold" (a runway crossing's or the route's holding point) or "stand".
    pub(crate) kind: &'static str,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct RouteOut {
    pub(crate) source: &'static str,
    pub(crate) via: String,
    /// The part already driven (start to the aircraft), flat.
    pub(crate) driven: Vec<f64>,
    /// The part still to drive (the aircraft to the end), flat.
    pub(crate) remaining: Vec<f64>,
    pub(crate) remaining_m: f64,
    /// Hold short of each runway the route crosses.
    pub(crate) holds: Vec<Mark>,
    pub(crate) dest: Option<Mark>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct CardOut {
    /// T turn, C runway crossing, N new taxiway, H holding point or stand,
    /// X off the route.
    pub(crate) kind: String,
    pub(crate) text: String,
    pub(crate) dist_m: f64,
    pub(crate) label: String,
    pub(crate) aim_true_deg: f64,
    /// Signed, metres, positive right of the route.
    pub(crate) xtk_m: f64,
    /// Signed, degrees, positive right.
    pub(crate) turn_deg: f64,
    pub(crate) remaining_m: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct RunwayMark {
    pub(crate) ident: String,
    /// "cleared", "arrival" or "the plan's".
    pub(crate) why: &'static str,
}

/// The Airport page's live state.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct AirportOut {
    /// The field the page wants drawn.
    pub(crate) icao: Option<String>,
    pub(crate) why: Option<&'static str>,
    pub(crate) name: Option<String>,
    /// The diagram's version to fetch (`GET /airport?icao=`), when built.
    pub(crate) version: Option<String>,
    /// "ready", "loading", "unavailable" or "none".
    pub(crate) status: &'static str,
    pub(crate) own: Option<Own>,
    pub(crate) route: Option<RouteOut>,
    pub(crate) card: Option<CardOut>,
    pub(crate) runway: Option<RunwayMark>,
}

/// A polyline cut where the aircraft projects onto it: (driven, remaining,
/// metres remaining). The foot is the nearest point on the line, the first
/// of equals. With no aircraft, nothing is driven. Pure.
pub(crate) fn split_route(
    pts: &[(f64, f64)],
    at: Option<(f64, f64)>,
) -> (Vec<(f64, f64)>, Vec<(f64, f64)>, f64) {
    let length = |p: &[(f64, f64)]| {
        p.windows(2)
            .map(|w| (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1))
            .sum::<f64>()
    };
    let (Some(p), true) = (at, pts.len() >= 2) else {
        return (Vec::new(), pts.to_vec(), length(pts));
    };
    let mut best = (f64::INFINITY, 0usize, 0.0);
    for i in 0..pts.len() - 1 {
        let (d, t) = seg_foot(p, pts[i], pts[i + 1]);
        if d < best.0 - 1e-9 {
            best = (d, i, t);
        }
    }
    let (_, i, t) = best;
    let (a, b) = (pts[i], pts[i + 1]);
    let foot = (a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1));
    let mut driven: Vec<(f64, f64)> = pts[..=i].to_vec();
    driven.push(foot);
    let mut remaining = vec![foot];
    remaining.extend_from_slice(&pts[i + 1..]);
    let rem = length(&remaining);
    (driven, remaining, rem)
}

/// A distance as a pilot reads it on a card: tens of metres, then km.
pub(crate) fn distance_words(m: f64) -> String {
    let m = m.max(0.0);
    if m < 1000.0 {
        format!("{} m", ((m / 10.0).round() * 10.0) as i64)
    } else {
        format!("{:.1} km", m / 1000.0)
    }
}

/// The card's banner: "Left onto B in 120 m", "Off the route".
pub(crate) fn card_text(c: &CardFacts) -> String {
    if c.kind == 'X' {
        return "Off the route".into();
    }
    let mut chars = c.label.trim().chars();
    let label = match chars.next() {
        Some(f) => f.to_uppercase().collect::<String>() + chars.as_str(),
        None => "Next".into(),
    };
    if c.dist_m < 15.0 {
        format!("{label} now")
    } else {
        format!("{label} in {}", distance_words(c.dist_m))
    }
}

fn route_out(r: &RouteFacts, frame: &Frame, at: Option<(f64, f64)>) -> Option<RouteOut> {
    let pts: Vec<(f64, f64)> = r.path.iter().map(|&(la, lo)| frame.xy(la, lo)).collect();
    let first = *pts.first()?;
    if first.0.hypot(first.1) > ROUTE_MAX_M || pts.len() < 2 {
        return None;
    }
    let (driven, remaining, rem) = split_route(&pts, at);
    // Hold short where a runway leg begins (a crossing), never at the start.
    let holds = r
        .seg
        .iter()
        .enumerate()
        .filter(|(i, (_, rwy))| *rwy && *i > 0 && !r.seg[*i - 1].1)
        .filter_map(|(i, (name, _))| {
            let p = pts.get(i)?;
            Some(Mark {
                x: r1(p.0),
                y: r1(p.1),
                label: name.clone(),
                kind: "hold",
            })
        })
        .collect();
    let last = *pts.last()?;
    let dest = Some(Mark {
        x: r1(last.0),
        y: r1(last.1),
        label: r.holding_point.clone(),
        kind: if r.to_stand { "stand" } else { "hold" },
    });
    Some(RouteOut {
        source: r.source,
        via: r.via.clone(),
        driven: flat(&driven),
        remaining: flat(&remaining),
        remaining_m: rem.round(),
        holds,
        dest,
    })
}

/// The Airport page, from the facts. Pure.
pub(crate) fn page(
    a: &AirportFacts,
    tick: Option<&Tick>,
    plan: Option<&PlanFacts>,
    airborne: bool,
) -> AirportOut {
    let choice = wanted_field(a, tick, plan, airborne);
    let mut out = AirportOut {
        icao: choice.as_ref().map(|c| c.icao.clone()),
        why: choice.as_ref().map(|c| c.why),
        name: None,
        version: None,
        status: "none",
        own: None,
        route: None,
        card: None,
        runway: None,
    };
    let Some(choice) = choice else {
        return out;
    };
    let Some(d) = a.diagram.as_ref().filter(|d| d.icao == choice.icao) else {
        out.status = if a.unavailable {
            "unavailable"
        } else {
            "loading"
        };
        return out;
    };
    out.status = "ready";
    out.name = Some(d.name.clone());
    out.version = Some(d.version.clone());
    let frame = Frame::new(d.lat0, d.lon0);
    let at = tick.map(|t| frame.xy(t.lat, t.lon));
    if let (Some(t), Some((x, y))) = (tick, at) {
        if x.hypot(y) <= OWN_MAX_M {
            out.own = Some(Own {
                x: r1(x),
                y: r1(y),
                heading: r1(t.heading_deg.rem_euclid(360.0)),
                track: t.track_deg.map(|v| r1(v.rem_euclid(360.0))),
                gs_kts: r1(t.gs_kts.abs()),
                on_ground: t.on_ground,
            });
        }
    }
    let own_at = out.own.as_ref().map(|o| (o.x, o.y));
    out.route = a.route.as_ref().and_then(|r| route_out(r, &frame, own_at));
    if out.route.is_some() {
        out.card = a.card.as_ref().map(|c| CardOut {
            kind: c.kind.to_string(),
            text: card_text(c),
            dist_m: c.dist_m.max(0.0).round(),
            label: c.label.clone(),
            aim_true_deg: r1(c.aim_true_deg.rem_euclid(360.0)),
            xtk_m: r1(c.xtk_m),
            turn_deg: c.turn_deg.round(),
            remaining_m: c.remaining_m.max(0.0).round(),
        });
    }
    // The clearance is the station's, so it marks the station's field only.
    let at_station = a
        .station
        .as_ref()
        .is_some_and(|(i, _, _)| i.eq_ignore_ascii_case(&choice.icao));
    let plan_rwy = plan.and_then(|p| {
        if p.destination.eq_ignore_ascii_case(&choice.icao) {
            p.dest_rwy.clone()
        } else if p.origin.eq_ignore_ascii_case(&choice.icao) {
            p.origin_rwy.clone()
        } else {
            None
        }
    });
    let arrival = plan
        .is_some_and(|p| p.destination.eq_ignore_ascii_case(&choice.icao))
        .then(|| a.arrival_runway.clone())
        .flatten();
    out.runway = a
        .cleared_runway
        .clone()
        .filter(|_| at_station)
        .map(|r| (r, "cleared"))
        .or(arrival.map(|r| (r, "arrival")))
        .or(plan_rwy.map(|r| (r, "the plan's")))
        .filter(|(r, _)| !r.trim().is_empty())
        .map(|(ident, why)| RunwayMark { ident, why });
    out
}

#[cfg(test)]
mod tests;
