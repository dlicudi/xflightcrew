//! The map's basemap: water and coastlines from X-Plane's OWN map
//! data, `Resources/map data/water/<+LL+LLL>/<+LL+LLL>.shp`, one ESRI
//! Shapefile of water polygons per 1x1 degree tile (named by its south-west
//! corner), grouped in 10x10 degree folders. An installed file like apt.dat,
//! never a sim read. 355 folders, 664 MB: only the tiles under the view are
//! read, on a blocking thread, and kept in small caches.
//!
//! A tile with no file is all water or all land (open sea mostly: whole
//! oceans have no folder). Its neighbours say which: a neighbour whose
//! shared edge is water makes it water, a land edge land; a tile among
//! missing tiles takes its missing neighbours' answer; none at all is sea.
//!
//! Served as `GET /basemap?bbox=lat0,lon0,lat1,lon1&px=<view width>`:
//! rings simplified (Douglas-Peucker) to about one pixel of the view, flat
//! `[lat, lon, ...]` arrays at 4 decimals, under a size cap (coarser until
//! it fits), and the fields in the box when zoomed in enough (with or
//! without the map data: the field directory is apt.dat's).

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

use serde::Serialize;

/// The water tiles under the X-Plane install.
pub(crate) const WATER_DIR: &str = "Resources/map data/water";

/// The payload's ceiling; a view over it is redone coarser.
pub(crate) const MAX_BYTES: usize = 400_000;

/// A view over more tiles than this is not drawn (a continent); the page
/// never asks for one (`map.js` `MAX_TILES`, pinned by
/// `the_page_asks_no_wider_than_the_server_draws`).
pub(crate) const MAX_TILES: usize = 1_200;

/// Fields are sent once the view is this narrow (degrees of latitude
/// across): towered ones under [`TOWERED_SPAN_DEG`], every land field
/// under [`ALL_FIELDS_SPAN_DEG`].
pub(crate) const TOWERED_SPAN_DEG: f64 = 6.0;
pub(crate) const ALL_FIELDS_SPAN_DEG: f64 = 1.5;
const FIELDS_MAX: usize = 300;

pub(crate) fn water_dir() -> PathBuf {
    PathBuf::from(crate::config::xplane_path(WATER_DIR))
}

/// "+51-001": a tile's (or folder's) south-west corner.
pub(crate) fn tile_name(lat: i32, lon: i32) -> String {
    format!("{lat:+03}{lon:+04}")
}

/// `<dir>/+50-010/+51-001.shp`.
pub(crate) fn tile_path(dir: &Path, lat: i32, lon: i32) -> PathBuf {
    let folder = tile_name(lat.div_euclid(10) * 10, lon.div_euclid(10) * 10);
    dir.join(folder)
        .join(format!("{}.shp", tile_name(lat, lon)))
}

// ---------------------------------------------------------------- reader

/// A ring as (x = lon, y = lat), closed as the file stores it.
pub(crate) type Ring = Vec<(f64, f64)>;

fn be_i32(b: &[u8], at: usize) -> Option<i32> {
    Some(i32::from_be_bytes(b.get(at..at + 4)?.try_into().ok()?))
}
fn le_i32(b: &[u8], at: usize) -> Option<i32> {
    Some(i32::from_le_bytes(b.get(at..at + 4)?.try_into().ok()?))
}
fn le_f64(b: &[u8], at: usize) -> Option<f64> {
    Some(f64::from_le_bytes(b.get(at..at + 8)?.try_into().ok()?))
}

/// The rings of an ESRI Shapefile's polygons (shape type 5, or 15 and 25
/// whose Z and M tails are skipped). The 100-byte header (file code 9994,
/// big-endian; version 1000, little-endian), then records: an 8-byte
/// big-endian header (number, content length in 16-bit words) and a
/// little-endian content (type, bbox, part and point counts, the parts'
/// first indices, the points). `None` for a file that is not one; a
/// damaged record ends the read with what came before it.
pub(crate) fn parse_shp(b: &[u8]) -> Option<Vec<Ring>> {
    if be_i32(b, 0)? != 9994 || le_i32(b, 28)? != 1000 {
        return None;
    }
    let file_len = (be_i32(b, 24)?.max(0) as usize * 2).min(b.len());
    let mut rings = Vec::new();
    let mut at = 100usize;
    while at + 8 <= file_len {
        let Some(words) = be_i32(b, at + 4) else {
            break;
        };
        let content = at + 8;
        let end = content + words.max(0) as usize * 2;
        if end > file_len {
            break;
        }
        at = end;
        let Some(kind) = le_i32(b, content) else {
            break;
        };
        if !matches!(kind, 5 | 15 | 25) {
            continue;
        }
        let (Some(np), Some(npt)) = (le_i32(b, content + 36), le_i32(b, content + 40)) else {
            break;
        };
        let (np, npt) = (np.max(0) as usize, npt.max(0) as usize);
        let parts_at = content + 44;
        let pts_at = parts_at + 4 * np;
        if pts_at + 16 * npt > end {
            break;
        }
        let mut starts: Vec<usize> = (0..np)
            .filter_map(|i| le_i32(b, parts_at + 4 * i))
            .map(|s| (s.max(0) as usize).min(npt))
            .collect();
        starts.push(npt);
        for w in starts.windows(2) {
            let (s, e) = (w[0], w[1]);
            if e <= s {
                continue;
            }
            let ring: Ring = (s..e)
                .filter_map(|i| {
                    let p = pts_at + 16 * i;
                    Some((le_f64(b, p)?, le_f64(b, p + 8)?))
                })
                .filter(|(x, y)| x.is_finite() && y.is_finite())
                .collect();
            if ring.len() >= 4 {
                rings.push(ring);
            }
        }
    }
    Some(rings)
}

/// Even-odd: is the point inside the rings (water)?
pub(crate) fn inside(rings: &[Ring], x: f64, y: f64) -> bool {
    let mut odd = false;
    for r in rings {
        for w in r.windows(2) {
            let ((x0, y0), (x1, y1)) = (w[0], w[1]);
            if (y0 > y) != (y1 > y) && x < x0 + (y - y0) * (x1 - x0) / (y1 - y0) {
                odd = !odd;
            }
        }
    }
    odd
}

// ---------------------------------------------------------------- tiles

/// A tile as read: its rings, and how much of each edge is water (south,
/// north, west, east; 0..1), for its missing neighbours.
pub(crate) struct Tile {
    pub(crate) rings: Vec<Ring>,
    pub(crate) edges: [f32; 4],
}

const SOUTH: usize = 0;
const NORTH: usize = 1;
const WEST: usize = 2;
const EAST: usize = 3;

pub(crate) fn tile_from_rings(lat: i32, lon: i32, rings: Vec<Ring>) -> Tile {
    let (y0, x0) = (lat as f64, lon as f64);
    let inset = 0.002;
    let n = 9;
    let frac = |pts: Vec<(f64, f64)>| {
        let hits = pts.iter().filter(|(x, y)| inside(&rings, *x, *y)).count();
        hits as f32 / pts.len() as f32
    };
    let along = |i: usize| (i as f64 + 0.5) / n as f64;
    let edges = [
        frac((0..n).map(|i| (x0 + along(i), y0 + inset)).collect()),
        frac((0..n).map(|i| (x0 + along(i), y0 + 1.0 - inset)).collect()),
        frac((0..n).map(|i| (x0 + inset, y0 + along(i))).collect()),
        frac((0..n).map(|i| (x0 + 1.0 - inset, y0 + along(i))).collect()),
    ];
    Tile { rings, edges }
}

/// A small least-recently-used list (the newest last).
struct Lru<K, V> {
    cap: usize,
    items: Vec<(K, V)>,
}

impl<K: PartialEq, V: Clone> Lru<K, V> {
    const fn new(cap: usize) -> Self {
        Lru {
            cap,
            items: Vec::new(),
        }
    }
    fn get(&mut self, k: &K) -> Option<V> {
        let i = self.items.iter().position(|(x, _)| x == k)?;
        let it = self.items.remove(i);
        let v = it.1.clone();
        self.items.push(it);
        Some(v)
    }
    fn put(&mut self, k: K, v: V) {
        self.items.retain(|(x, _)| x != &k);
        self.items.push((k, v));
        while self.items.len() > self.cap {
            self.items.remove(0);
        }
    }
}

/// Tiles as read (a memo of a file read: map data holds no flight fact).
static TILES: Mutex<Lru<(i32, i32), Option<Arc<Tile>>>> = Mutex::new(Lru::new(96));
/// Edges of every tile looked at (tiny), `None` for a tile with no file.
static EDGES: Mutex<Option<HashMap<(i32, i32), Option<[f32; 4]>>>> = Mutex::new(None);
/// Tiles simplified at a zoom bucket: flat rounded rings.
type Flat = Arc<Vec<Vec<f64>>>;
static SIMPLE: Mutex<Lru<(i32, i32, i32), Flat>> = Mutex::new(Lru::new(1_024));
/// Whole answers by (covered tiles, bucket).
static ANSWERS: Mutex<Lru<(i32, i32, i32, i32, i32), Arc<Answer>>> = Mutex::new(Lru::new(16));

fn lock<T>(m: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

/// The tile, read once while it stays in the cache; `None` with no file.
fn tile(dir: &Path, lat: i32, lon: i32) -> Option<Arc<Tile>> {
    if let Some(t) = lock(&TILES).get(&(lat, lon)) {
        return t;
    }
    let t = std::fs::read(tile_path(dir, lat, lon))
        .ok()
        .and_then(|b| parse_shp(&b))
        .map(|rings| Arc::new(tile_from_rings(lat, lon, rings)));
    lock(&TILES).put((lat, lon), t.clone());
    lock(&EDGES)
        .get_or_insert_with(HashMap::new)
        .insert((lat, lon), t.as_ref().map(|t| t.edges));
    t
}

fn edges(dir: &Path, lat: i32, lon: i32) -> Option<[f32; 4]> {
    if let Some(e) = lock(&EDGES).as_ref().and_then(|m| m.get(&(lat, lon))) {
        return *e;
    }
    tile(dir, lat, lon).map(|t| t.edges)
}

/// Which missing tiles of the box are water. `present(lat, lon)` says a tile
/// has a file; `edge(lat, lon, side)` the water fraction of a present
/// tile's side. Pure.
pub(crate) fn water_tiles(
    lat: (i32, i32),
    lon: (i32, i32),
    present: &dyn Fn(i32, i32) -> bool,
    edge: &dyn Fn(i32, i32, usize) -> f32,
) -> Vec<(i32, i32)> {
    let (la0, la1, lo0, lo1) = (lat.0, lat.1, lon.0, lon.1);
    let mut class: HashMap<(i32, i32), bool> = HashMap::new();
    // The neighbour, and the side of IT that faces us.
    let around = |a: i32, o: i32| {
        [
            (a - 1, o, NORTH),
            (a + 1, o, SOUTH),
            (a, o - 1, EAST),
            (a, o + 1, WEST),
        ]
    };
    for a in la0..=la1 {
        for o in lo0..=lo1 {
            if present(a, o) {
                continue;
            }
            let (mut wet, mut dry) = (0, 0);
            for (na, no, side) in around(a, o) {
                if present(na, no) {
                    if edge(na, no, side) >= 0.5 {
                        wet += 1;
                    } else {
                        dry += 1;
                    }
                }
            }
            if wet + dry > 0 {
                class.insert((a, o), wet >= dry);
            }
        }
    }
    // Missing among missing: the answer spreads from those that have one.
    loop {
        let mut grew = false;
        for a in la0..=la1 {
            for o in lo0..=lo1 {
                if present(a, o) || class.contains_key(&(a, o)) {
                    continue;
                }
                let votes: Vec<bool> = around(a, o)
                    .iter()
                    .filter_map(|(na, no, _)| class.get(&(*na, *no)).copied())
                    .collect();
                if !votes.is_empty() {
                    let wet = votes.iter().filter(|w| **w).count();
                    class.insert((a, o), wet * 2 >= votes.len());
                    grew = true;
                }
            }
        }
        if !grew {
            break;
        }
    }
    let mut out = Vec::new();
    for a in la0..=la1 {
        for o in lo0..=lo1 {
            if !present(a, o) && class.get(&(a, o)).copied().unwrap_or(true) {
                out.push((a, o));
            }
        }
    }
    out
}

/// Is a point on the sides of the tile whose south-west corner is `sw`
/// (lat, lon)?
fn on_tile_side(p: (f64, f64), sw: (f64, f64)) -> bool {
    let e = 1e-6;
    let (x, y) = p;
    (x - sw.1).abs() < e
        || (x - sw.1 - 1.0).abs() < e
        || (y - sw.0).abs() < e
        || (y - sw.0 - 1.0).abs() < e
}

/// A ring at a tolerance (degrees, x scaled by the tile's cos(lat)), as the
/// wire carries it: `[lat, lon, ...]` at 4 decimals. `None` once it is
/// smaller than the tolerance. Every vertex on the sides of the tile `sw`
/// is kept (the simplifier runs between them), so neighbouring tiles still
/// meet edge to edge and the page can leave the tiles' own sides unstroked.
pub(crate) fn wire_ring(r: &Ring, tol: f64, k: f64, sw: Option<(f64, f64)>) -> Option<Vec<f64>> {
    let (mut x0, mut x1, mut y0, mut y1) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for &(x, y) in r {
        x0 = x0.min(x);
        x1 = x1.max(x);
        y0 = y0.min(y);
        y1 = y1.max(y);
    }
    if ((x1 - x0) * k).max(y1 - y0) < tol {
        return None;
    }
    let scaled: Vec<(f64, f64)> = r.iter().map(|&(x, y)| (x * k, y)).collect();
    let mut anchors: Vec<usize> = match sw {
        Some(sw) => (0..r.len()).filter(|&i| on_tile_side(r[i], sw)).collect(),
        None => Vec::new(),
    };
    let s = if anchors.is_empty() {
        crate::efb::airport::simplify(&scaled, tol)
    } else {
        // The spans between the side vertices, each simplified on its own;
        // the ring closes back through its first point.
        if anchors[0] != 0 {
            anchors.insert(0, 0);
        }
        if *anchors.last().unwrap_or(&0) != r.len() - 1 {
            anchors.push(r.len() - 1);
        }
        let mut s: Vec<(f64, f64)> = vec![scaled[0]];
        for w in anchors.windows(2) {
            let part = crate::efb::airport::simplify(&scaled[w[0]..=w[1]], tol);
            s.extend(part.into_iter().skip(1));
        }
        s
    };
    let mut out: Vec<f64> = Vec::with_capacity(s.len() * 2);
    let mut last: Option<(i64, i64)> = None;
    for (x, y) in s {
        let q = ((y * 1e4).round() as i64, (x / k * 1e4).round() as i64);
        if last != Some(q) {
            out.push(q.0 as f64 / 1e4);
            out.push(q.1 as f64 / 1e4);
            last = Some(q);
        }
    }
    (out.len() >= 8).then_some(out)
}

fn simplified(dir: &Path, lat: i32, lon: i32, bucket: i32) -> Option<Flat> {
    if let Some(f) = lock(&SIMPLE).get(&(lat, lon, bucket)) {
        return Some(f);
    }
    let t = tile(dir, lat, lon)?;
    let tol = 2f64.powi(bucket);
    let k = ((lat as f64 + 0.5).to_radians().cos()).max(0.05);
    let f: Flat = Arc::new(
        t.rings
            .iter()
            .filter_map(|r| wire_ring(r, tol, k, Some((lat as f64, lon as f64))))
            .collect(),
    );
    lock(&SIMPLE).put((lat, lon, bucket), f.clone());
    Some(f)
}

// ---------------------------------------------------------------- answer

/// One answer: what `GET /basemap` returns.
#[derive(Debug, Serialize)]
pub(crate) struct Out {
    /// The tiles covered, lat0, lon0, lat1, lon1 (the page asks again only
    /// once its view leaves them).
    pub(crate) bbox: [i32; 4],
    /// log2 of the tolerance in degrees, and the tolerance.
    pub(crate) bucket: i32,
    pub(crate) tol: f64,
    /// Water: rings, each `[lat, lon, ...]`, filled even-odd in one path.
    pub(crate) water: Vec<Vec<f64>>,
    /// Fields in the box: [icao, lat, lon, towered (1 or 0)].
    pub(crate) fields: Vec<(String, f64, f64, u8)>,
}

pub(crate) struct Answer {
    pub(crate) etag: String,
    pub(crate) json: String,
    pub(crate) rings: usize,
}

/// The zoom bucket for a view `px` pixels wide: log2 of its degrees (of
/// latitude) per pixel.
pub(crate) fn bucket_for(lat0: f64, lon0: f64, lat1: f64, lon1: f64, px: f64) -> i32 {
    let k = ((lat0 + lat1) / 2.0).to_radians().cos().max(0.05);
    let span = (lon1 - lon0).abs() * k;
    let dpp = (span / px.clamp(50.0, 8_000.0)).max(1e-6);
    dpp.log2().round() as i32
}

/// The covered tiles of a box: south-west corners, inclusive.
pub(crate) fn tile_range(lat0: f64, lon0: f64, lat1: f64, lon1: f64) -> (i32, i32, i32, i32) {
    let (la0, la1) = (lat0.min(lat1).max(-90.0), lat0.max(lat1).min(89.999));
    let (lo0, lo1) = (lon0.min(lon1).max(-180.0), lon0.max(lon1).min(179.999));
    (
        la0.floor() as i32,
        lo0.floor() as i32,
        la1.floor() as i32,
        lo1.floor() as i32,
    )
}

/// Build (or recall) the answer for a view. Blocking: file reads. `None`
/// when the view is too wide; no water (the fields only) when the map data
/// is not installed.
pub(crate) fn answer(dir: &Path, bbox: [f64; 4], px: f64) -> Option<Arc<Answer>> {
    let have_water = dir.is_dir();
    let [lat0, lon0, lat1, lon1] = bbox;
    if !bbox.iter().all(|v| v.is_finite()) {
        return None;
    }
    let (a0, o0, a1, o1) = tile_range(lat0, lon0, lat1, lon1);
    let n = ((a1 - a0 + 1) * (o1 - o0 + 1)).max(0) as usize;
    if n == 0 || n > MAX_TILES {
        return None;
    }
    let mut bucket = bucket_for(lat0, lon0, lat1, lon1, px).max(-16);
    let key = (a0, o0, a1, o1, bucket);
    if let Some(a) = lock(&ANSWERS).get(&key) {
        return Some(a);
    }
    let regions_ready = crate::party_line::region_loaded();
    let present = |a: i32, o: i32| tile_path(dir, a, o).is_file();
    let edge = |a: i32, o: i32, side: usize| edges(dir, a, o).map_or(0.0, |e| e[side]);
    let wet = if have_water {
        water_tiles((a0, a1), (o0, o1), &present, &edge)
    } else {
        Vec::new()
    };
    let span = (a1 - a0 + 1) as f64;
    let fields: Vec<(String, f64, f64, u8)> = if span <= TOWERED_SPAN_DEG {
        crate::party_line::fields_in_box(
            a0 as f64,
            o0 as f64,
            a1 as f64 + 1.0,
            o1 as f64 + 1.0,
            span <= ALL_FIELDS_SPAN_DEG,
            FIELDS_MAX,
        )
        .into_iter()
        .map(|f| {
            let r4 = |v: f64| (v * 1e4).round() / 1e4;
            (f.icao, r4(f.lat), r4(f.lon), f.towered as u8)
        })
        .collect()
    } else {
        Vec::new()
    };
    let mut out;
    loop {
        let mut water: Vec<Vec<f64>> = Vec::new();
        for a in a0..=a1 {
            for o in (o0..=o1).filter(|_| have_water) {
                if let Some(f) = simplified(dir, a, o, bucket) {
                    water.extend(f.iter().cloned());
                }
            }
        }
        // A tile of open water: its square, clockwise like the file's outers.
        for (a, o) in &wet {
            let (y0, x0, y1, x1) = (*a as f64, *o as f64, *a as f64 + 1.0, *o as f64 + 1.0);
            water.push(vec![y0, x0, y1, x0, y1, x1, y0, x1, y0, x0]);
        }
        out = Out {
            bbox: [a0, o0, a1, o1],
            bucket,
            tol: 2f64.powi(bucket),
            water,
            fields: fields.clone(),
        };
        let json = serde_json::to_string(&out).unwrap_or_default();
        if json.len() <= MAX_BYTES || bucket >= 4 {
            let a = Arc::new(Answer {
                etag: crate::efb::airport::version_of(&json),
                rings: out.water.len(),
                json,
            });
            if regions_ready {
                lock(&ANSWERS).put(key, a.clone());
            }
            return Some(a);
        }
        bucket += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A one-record Polygon shapefile: an outer square with a square hole.
    fn synthetic() -> Vec<u8> {
        let outer = [
            (0.0, 51.0),
            (0.0, 52.0),
            (1.0, 52.0),
            (1.0, 51.0),
            (0.0, 51.0),
        ];
        let hole = [
            (0.25, 51.25),
            (0.75, 51.25),
            (0.75, 51.75),
            (0.25, 51.75),
            (0.25, 51.25),
        ];
        let pts: Vec<(f64, f64)> = outer.iter().chain(hole.iter()).copied().collect();
        let mut c: Vec<u8> = Vec::new();
        c.extend(5i32.to_le_bytes());
        for v in [0.0f64, 51.0, 1.0, 52.0] {
            c.extend(v.to_le_bytes());
        }
        c.extend(2i32.to_le_bytes());
        c.extend((pts.len() as i32).to_le_bytes());
        c.extend(0i32.to_le_bytes());
        c.extend(5i32.to_le_bytes());
        for (x, y) in &pts {
            c.extend(x.to_le_bytes());
            c.extend(y.to_le_bytes());
        }
        let mut f: Vec<u8> = Vec::new();
        f.extend(9994i32.to_be_bytes());
        f.extend([0u8; 20]);
        let total = 100 + 8 + c.len();
        f.extend(((total / 2) as i32).to_be_bytes());
        f.extend(1000i32.to_le_bytes());
        f.extend(5i32.to_le_bytes());
        f.extend([0u8; 64]);
        assert_eq!(f.len(), 100);
        f.extend(1i32.to_be_bytes());
        f.extend(((c.len() / 2) as i32).to_be_bytes());
        f.extend(c);
        f
    }

    #[test]
    fn a_polygon_shapefile_reads_its_rings() {
        let rings = parse_shp(&synthetic()).expect("a shapefile");
        assert_eq!(rings.len(), 2, "the outer and the hole");
        assert_eq!(rings[0].len(), 5);
        assert_eq!(rings[1][0], (0.25, 51.25));
        // Even-odd: water round the hole, land in it.
        assert!(inside(&rings, 0.1, 51.1));
        assert!(!inside(&rings, 0.5, 51.5));
        let t = tile_from_rings(51, 0, rings);
        assert!(t.edges.iter().all(|e| *e > 0.99), "{:?}", t.edges);
        // Not a shapefile, or cut short: no rings, no panic.
        assert!(parse_shp(b"hello").is_none());
        let mut cut = synthetic();
        cut.truncate(150);
        assert_eq!(parse_shp(&cut).map(|r| r.len()), Some(0));
    }

    #[test]
    fn tiles_are_named_by_their_south_west_corner() {
        let d = Path::new("/w");
        assert_eq!(
            tile_path(d, 51, -1),
            PathBuf::from("/w/+50-010/+51-001.shp")
        );
        assert_eq!(
            tile_path(d, 36, -6),
            PathBuf::from("/w/+30-010/+36-006.shp")
        );
        assert_eq!(tile_path(d, 0, 0), PathBuf::from("/w/+00+000/+00+000.shp"));
        assert_eq!(
            tile_path(d, -34, 151),
            PathBuf::from("/w/-40+150/-34+151.shp")
        );
        assert_eq!(tile_range(50.2, -1.5, 51.9, 0.3), (50, -2, 51, 0));
    }

    /// A wiggly coast thins to the view's pixel; a lake under it goes.
    #[test]
    fn rings_thin_to_the_pixel_and_specks_go() {
        let mut ring: Ring = (0..=100)
            .map(|i| {
                let x = i as f64 / 100.0;
                (x, 51.5 + 0.0005 * ((i % 2) as f64))
            })
            .collect();
        ring.push((1.0, 51.0));
        ring.push((0.0, 51.0));
        ring.push(ring[0]);
        let fine = wire_ring(&ring, 0.0001, 1.0, None).unwrap();
        let coarse = wire_ring(&ring, 0.01, 1.0, None).unwrap();
        assert!(fine.len() > 150, "{}", fine.len());
        assert!(coarse.len() <= 12, "{coarse:?}");
        // Lat first, at 4 decimals.
        assert_eq!(&coarse[..2], &[51.5, 0.0]);
        // On its tile, every vertex on the tile's sides stays: the
        // neighbours meet edge to edge.
        let mut sided = ring.clone();
        sided.insert(50, (0.5, 51.0));
        let anchored = wire_ring(&sided, 0.01, 1.0, Some((51.0, 0.0))).unwrap();
        let pairs: Vec<(f64, f64)> = anchored.chunks(2).map(|c| (c[0], c[1])).collect();
        for side in [
            (51.5, 0.0),
            (51.0, 0.5),
            (51.5, 1.0),
            (51.0, 1.0),
            (51.0, 0.0),
        ] {
            assert!(pairs.contains(&side), "{side:?} in {pairs:?}");
        }
        let speck: Ring = vec![(0.0, 51.0), (0.0, 51.001), (0.001, 51.001), (0.0, 51.0)];
        assert!(wire_ring(&speck, 0.01, 1.0, None).is_none());
    }

    /// A missing tile beside a sea edge is sea, beside a land edge land; a
    /// missing tile among missing ones takes their answer; none is sea.
    #[test]
    fn a_missing_tile_is_water_or_land_by_its_neighbours() {
        // Present: (0,0) wet on its east edge; (2,0) dry everywhere.
        let present = |a: i32, o: i32| (a, o) == (0, 0) || (a, o) == (2, 0);
        let edge = |a: i32, _o: i32, side: usize| {
            if a == 0 && side == EAST {
                1.0
            } else {
                0.0
            }
        };
        let w = water_tiles((0, 2), (0, 2), &present, &edge);
        assert!(w.contains(&(0, 1)), "east of a sea edge: {w:?}");
        assert!(!w.contains(&(1, 0)), "between two dry edges: land");
        assert!(!w.contains(&(3, 0)));
        assert!(w.contains(&(0, 2)), "next to sea, missing: sea");
        // Nothing to go on: open sea.
        let none = water_tiles((10, 11), (10, 11), &|_, _| false, &|_, _, _| 0.0);
        assert_eq!(none.len(), 4);
    }

    /// No map data installed: still an answer (the fields, no water); a
    /// continent: none.
    #[test]
    fn no_map_data_is_no_water_and_a_continent_is_no_answer() {
        let none = Path::new("/nonexistent/map data/water");
        let a = answer(none, [-12.5, -150.5, -11.5, -149.5], 800.0).expect("an answer");
        let v: serde_json::Value = serde_json::from_str(&a.json).unwrap();
        assert_eq!(v["water"], serde_json::json!([]));
        assert!(answer(none, [-60.0, -170.0, 60.0, 170.0], 800.0).is_none());
    }

    #[test]
    fn the_bucket_is_about_a_pixel() {
        // 2 degrees of latitude-equivalent over 1,000 px: 0.002, 2^-9.
        let b = bucket_for(50.0, -1.0, 52.0, 2.11, 1000.0);
        assert_eq!(b, -9);
        assert!(bucket_for(30.0, -10.0, 52.0, 5.0, 1000.0) > b);
    }

    /// The basemap answers for the fixture pages (`state::tests::
    /// efb_fixture_json`), to look at the Map tab without a sim:
    /// `EFB_FIXTURE_OUT=target/efb cargo test --release -- --ignored
    /// real_install_basemap_fixtures` writes `basemap_cruise.json` and
    /// `basemap_approach.json` (fields included: the region index is read).
    #[test]
    #[ignore]
    fn real_install_basemap_fixtures() {
        let out = std::env::var("EFB_FIXTURE_OUT").expect("EFB_FIXTURE_OUT");
        std::fs::create_dir_all(&out).unwrap();
        crate::party_line::load_region(&crate::config::xplane_path(
            "Global Scenery/Global Airports/Earth nav data/apt.dat",
        ));
        let dir = water_dir();
        for (name, bbox, px) in [
            ("cruise", [40.0, -14.0, 56.0, 12.0], 1400.0),
            ("approach", [50.5, -1.6, 51.8, 1.1], 1400.0),
        ] {
            let a = answer(&dir, bbox, px).expect("an answer");
            println!("{name}: {} bytes, {} rings", a.json.len(), a.rings);
            std::fs::write(format!("{out}/basemap_{name}.json"), &a.json).unwrap();
        }
    }

    /// The real install: Gatwick's tile and Gibraltar's hold water.
    /// `cargo test --release -- --ignored real_install_basemap --nocapture`
    #[test]
    #[ignore]
    fn real_install_basemap() {
        let dir = water_dir();
        for (a, o) in [(51, -1), (36, -6)] {
            let b = std::fs::read(tile_path(&dir, a, o)).expect("the tile");
            let rings = parse_shp(&b).expect("a shapefile");
            let pts: usize = rings.iter().map(|r| r.len()).sum();
            println!("{} rings {} points {}", tile_name(a, o), rings.len(), pts);
            assert!(!rings.is_empty(), "{}", tile_name(a, o));
        }
        // Gibraltar's strait: water in it, the Rock dry.
        let t = tile(&dir, 36, -6).unwrap();
        assert!(inside(&t.rings, -5.6, 36.0), "the strait");
        assert!(!inside(&t.rings, -5.35, 36.14), "the Rock");
        // The EGKK approach view and a cruise view over Spain and France.
        for (name, bbox, px) in [
            ("EGKK terminal", [50.9, -0.7, 51.4, 0.3], 1400.0),
            ("EGKK phone", [50.9, -0.7, 51.4, 0.3], 390.0),
            ("cruise", [37.0, -6.0, 51.5, 1.0], 1400.0),
        ] {
            let t0 = std::time::Instant::now();
            let a = answer(&dir, bbox, px).expect("an answer");
            println!(
                "{name}: {} bytes, {} rings, {} ms",
                a.json.len(),
                a.rings,
                t0.elapsed().as_millis()
            );
            assert!(a.json.len() <= MAX_BYTES);
            assert!(a.rings > 0);
        }
    }
}
