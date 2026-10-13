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
    if la1 < la0 || lo1 < lo0 {
        return Vec::new();
    }
    // The box as a grid (the whole world's is 64,800 cells): present, and
    // each missing tile's answer once it has one.
    let cols = (lo1 - lo0 + 1) as usize;
    let at = |a: i32, o: i32| (a - la0) as usize * cols + (o - lo0) as usize;
    let inside_box = |a: i32, o: i32| (la0..=la1).contains(&a) && (lo0..=lo1).contains(&o);
    let mut here = Vec::with_capacity(cols * (la1 - la0 + 1) as usize);
    for a in la0..=la1 {
        for o in lo0..=lo1 {
            here.push(present(a, o));
        }
    }
    let mut class: Vec<Option<bool>> = vec![None; here.len()];
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
            if here[at(a, o)] {
                continue;
            }
            let (mut wet, mut dry) = (0, 0);
            for (na, no, side) in around(a, o) {
                let there = if inside_box(na, no) {
                    here[at(na, no)]
                } else {
                    present(na, no)
                };
                if there {
                    if edge(na, no, side) >= 0.5 {
                        wet += 1;
                    } else {
                        dry += 1;
                    }
                }
            }
            if wet + dry > 0 {
                class[at(a, o)] = Some(wet >= dry);
            }
        }
    }
    // Missing among missing: the answer spreads from those that have one.
    loop {
        let mut grew = false;
        for a in la0..=la1 {
            for o in lo0..=lo1 {
                let i = at(a, o);
                if here[i] || class[i].is_some() {
                    continue;
                }
                let (mut wet, mut votes) = (0, 0);
                for (na, no, _) in around(a, o) {
                    if let Some(w) = inside_box(na, no).then(|| class[at(na, no)]).flatten() {
                        votes += 1;
                        wet += w as usize;
                    }
                }
                if votes > 0 {
                    class[i] = Some(wet * 2 >= votes);
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
            let i = at(a, o);
            if !here[i] && class[i].unwrap_or(true) {
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

/// Douglas-Peucker over a ring, every `anchors` vertex kept: the spans
/// between them are simplified each on its own, the ring closing back
/// through its first point.
fn thin(pts: &[(f64, f64)], tol: f64, mut anchors: Vec<usize>) -> Vec<(f64, f64)> {
    if anchors.is_empty() || pts.is_empty() {
        return crate::efb::airport::simplify(pts, tol);
    }
    if anchors[0] != 0 {
        anchors.insert(0, 0);
    }
    if *anchors.last().unwrap_or(&0) != pts.len() - 1 {
        anchors.push(pts.len() - 1);
    }
    let mut s: Vec<(f64, f64)> = vec![pts[0]];
    for w in anchors.windows(2) {
        let part = crate::efb::airport::simplify(&pts[w[0]..=w[1]], tol);
        s.extend(part.into_iter().skip(1));
    }
    s
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
    let anchors: Vec<usize> = match sw {
        Some(sw) => (0..r.len()).filter(|&i| on_tile_side(r[i], sw)).collect(),
        None => Vec::new(),
    };
    let s = thin(&scaled, tol, anchors);
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

/// Why a view has no answer.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum Miss {
    /// Not a box.
    BadView,
    /// Wider than [`MAX_TILES`] while the world layer is not built yet.
    NotReady,
}

/// Build (or recall) the answer for a view. Blocking: file reads. A view
/// wider than [`MAX_TILES`] is the world layer's ([`world_answer`]); no
/// water (the fields only) when the map data is not installed.
pub(crate) fn answer(dir: &Path, bbox: [f64; 4], px: f64) -> Result<Arc<Answer>, Miss> {
    answer_with(dir, world().as_deref(), bbox, px)
}

pub(crate) fn answer_with(
    dir: &Path,
    world: Option<&World>,
    bbox: [f64; 4],
    px: f64,
) -> Result<Arc<Answer>, Miss> {
    let have_water = dir.is_dir();
    let [lat0, lon0, lat1, lon1] = bbox;
    if !bbox.iter().all(|v| v.is_finite()) {
        return Err(Miss::BadView);
    }
    let (a0, o0, a1, o1) = tile_range(lat0, lon0, lat1, lon1);
    let n = ((a1 - a0 + 1) * (o1 - o0 + 1)).max(0) as usize;
    if n == 0 {
        return Err(Miss::BadView);
    }
    if n > MAX_TILES {
        let w = world.ok_or(Miss::NotReady)?;
        return Ok(world_answer(w, bbox, px));
    }
    let mut bucket = bucket_for(lat0, lon0, lat1, lon1, px).max(-16);
    let key = (a0, o0, a1, o1, bucket);
    if let Some(a) = lock(&ANSWERS).get(&key) {
        return Ok(a);
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
            return Ok(a);
        }
        bucket += 1;
    }
}

// ---------------------------------------------------------------- the world
//
// Views wider than MAX_TILES. A whole world of one-degree tiles is 17,848
// little rings, each with its tile's sides: megabytes at any tolerance. So
// each 10x10 degree block is DISSOLVED once: every tile's sides cancel
// against its neighbour's (an even-odd fill: a side drawn twice is no
// side), the open sea's tiles join in, and what is left is the coast, the
// block's own sides and the lakes, chained into rings and thinned on
// Mercator (the page's projection: a tolerance there is a pixel anywhere).

/// The world layer's tolerance, log2 of degrees of longitude (Mercator's
/// unit): 2^-5, about a pixel of the widest view the tiles still serve
/// (1,200 tiles across 1,400 px), a quarter of one of the whole world on
/// 2,560 px. Rings smaller than it are dropped.
pub(crate) const WORLD_BUCKET: i32 = -5;
/// Each tile is thinned to this (its sides kept) before its block is
/// dissolved: the dissolve then sees the coast, not the file's every point.
const WORLD_READ_BUCKET: i32 = -7;
/// A block: the 10x10 degree folder.
const BLOCK: i32 = 10;
/// A point at the wire's 4 decimals: (lat, lon) in 1e-4 degrees, exact, so
/// the tiles' shared sides meet.
type Q = (i32, i32);
const QD: i32 = 10_000;

/// Every installed tile dissolved per block, for views wider than
/// [`MAX_TILES`]: built in the background ([`start_world`]), held for the
/// run (a memo of the installed files, no flight fact).
pub(crate) struct World {
    /// Each block (its south-west corner) with rings, sorted: flat
    /// `[lat, lon, ...]` in 1e-4 degrees, closed, at [`WORLD_BUCKET`].
    blocks: Vec<((i32, i32), Vec<Vec<i32>>)>,
    pub(crate) tiles_read: usize,
    pub(crate) rings: usize,
    /// What the layer holds in memory, about.
    pub(crate) bytes: usize,
    answers: Mutex<Lru<(i32, i32, i32, i32, i32), Arc<Answer>>>,
}

/// `None` until the EFB first serves; `Building` on the build's thread.
static WORLD: Mutex<Option<WorldState>> = Mutex::new(None);

enum WorldState {
    Building,
    Ready(Arc<World>),
}

/// The world layer, once built.
pub(crate) fn world() -> Option<Arc<World>> {
    match &*lock(&WORLD) {
        Some(WorldState::Ready(w)) => Some(w.clone()),
        _ => None,
    }
}

/// "+51-001.shp" back to (51, -1).
fn tile_of_name(s: &str) -> Option<(i32, i32)> {
    let s = s.strip_suffix(".shp")?;
    if s.len() != 7 || !s.is_ascii() {
        return None;
    }
    Some((s[..3].parse().ok()?, s[3..].parse().ok()?))
}

/// Mercator's north, in degrees of longitude, within the page's MAX_LAT.
fn merc(lat: f64) -> f64 {
    let l = lat.clamp(-85.05, 85.05).to_radians();
    (l / 2.0 + std::f64::consts::FRAC_PI_4)
        .tan()
        .ln()
        .to_degrees()
}

/// The sides cancelled, the rest chained: the even-odd boundary of rings
/// whose shared sides lie on whole degrees. A run along a whole degree
/// is kept where an odd number of rings draw it; every other edge as is.
fn dissolve(rings: &[Vec<Q>]) -> Vec<Vec<Q>> {
    let mut lines: HashMap<(bool, i32), Vec<i32>> = HashMap::new();
    let mut edges: Vec<(Q, Q)> = Vec::new();
    for r in rings {
        for w in r.windows(2) {
            let (p, q) = (w[0], w[1]);
            if p == q {
                continue;
            }
            if p.0 == q.0 && p.0 % QD == 0 {
                lines.entry((true, p.0)).or_default().extend([p.1, q.1]);
            } else if p.1 == q.1 && p.1 % QD == 0 {
                lines.entry((false, p.1)).or_default().extend([p.0, q.0]);
            } else {
                edges.push((p, q));
            }
        }
    }
    for ((along_lat, c), mut ends) in lines {
        // Each run toggles the parity at both its ends.
        ends.sort_unstable();
        let mut odd = false;
        let mut i = 0;
        while i < ends.len() {
            let x = ends[i];
            let n = ends[i..].iter().take_while(|e| **e == x).count();
            i += n;
            odd ^= n % 2 == 1;
            if odd && i < ends.len() {
                let pt = |v: i32| if along_lat { (c, v) } else { (v, c) };
                edges.push((pt(x), pt(ends[i])));
            }
        }
    }
    // Every vertex of an even-odd boundary has an even degree: a walk from
    // any edge comes back to where it began.
    let mut at: HashMap<Q, Vec<usize>> = HashMap::new();
    for (i, (p, q)) in edges.iter().enumerate() {
        at.entry(*p).or_default().push(i);
        at.entry(*q).or_default().push(i);
    }
    let mut used = vec![false; edges.len()];
    let mut out = Vec::new();
    for s in 0..edges.len() {
        if used[s] {
            continue;
        }
        used[s] = true;
        let start = edges[s].0;
        let (mut ring, mut cur) = (vec![start, edges[s].1], edges[s].1);
        while cur != start {
            let next = at[&cur].iter().copied().find(|i| !used[*i]);
            let Some(i) = next else { break };
            used[i] = true;
            cur = if edges[i].0 == cur {
                edges[i].1
            } else {
                edges[i].0
            };
            ring.push(cur);
        }
        if cur != start {
            ring.push(start);
        }
        if ring.len() >= 4 {
            out.push(ring);
        }
    }
    out
}

/// A block's ring thinned on Mercator to `tol`, the vertices on the
/// block's sides kept (neighbouring blocks meet); flat `[lat, lon, ...]` in
/// 1e-4 degrees, `None` once smaller than `tol`.
fn world_ring(r: &[Q], tol: f64, block: (i32, i32)) -> Option<Vec<i32>> {
    let pts: Vec<(f64, f64)> = r
        .iter()
        .map(|&(a, o)| (o as f64 / QD as f64, merc(a as f64 / QD as f64)))
        .collect();
    let (mut x0, mut x1, mut y0, mut y1) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for &(x, y) in &pts {
        (x0, x1, y0, y1) = (x0.min(x), x1.max(x), y0.min(y), y1.max(y));
    }
    if (x1 - x0).max(y1 - y0) < tol {
        return None;
    }
    // Which of the block's four sides a point is on (a corner: two).
    let (s, w) = (block.0 * QD, block.1 * QD);
    let sides = |&(a, o): &Q| {
        (a == s) as u8
            | ((a == s + BLOCK * QD) as u8) << 1
            | ((o == w) as u8) << 2
            | ((o == w + BLOCK * QD) as u8) << 3
    };
    // The ends of each run along a side are kept; between them the side is
    // straight and thins to nothing.
    let n = r.len();
    let anchors = (0..n)
        .filter(|&i| {
            let (prev, next) = (r[if i == 0 { n - 2 } else { i - 1 }], r[(i + 1) % n]);
            sides(&r[i]) != 0 && sides(&prev) & sides(&r[i]) & sides(&next) == 0
        })
        .collect();
    // The thinned points are the ring's own, in order: back to their Q.
    let mut out = Vec::new();
    let mut j = 0;
    for p in thin(&pts, tol, anchors) {
        while j + 1 < pts.len() && pts[j] != p {
            j += 1;
        }
        out.extend([r[j].0, r[j].1]);
    }
    (out.len() >= 8).then_some(out)
}

/// Read every installed tile, dissolve each block and thin it to
/// [`WORLD_BUCKET`]. Blocking and long (the whole map data): `go_on` is
/// asked between tiles and blocks and a `false` ends it with `None`;
/// `pace` sleeps as long as each tile took (half a core at most, the sim's
/// frames first). No map data: an empty world, as the tiles' own answer.
pub(crate) fn build_world(dir: &Path, go_on: &dyn Fn() -> bool, pace: bool) -> Option<World> {
    let mut names: Vec<(i32, i32, PathBuf)> = Vec::new();
    for folder in std::fs::read_dir(dir).into_iter().flatten().flatten() {
        for f in std::fs::read_dir(folder.path())
            .into_iter()
            .flatten()
            .flatten()
        {
            if let Some((a, o)) = f.file_name().to_str().and_then(tile_of_name) {
                if (-90..90).contains(&a) && (-180..180).contains(&o) {
                    names.push((a, o, f.path()));
                }
            }
        }
    }
    names.sort_by_key(|n| (n.0, n.1));
    let block_of = |a: i32, o: i32| (a.div_euclid(BLOCK) * BLOCK, o.div_euclid(BLOCK) * BLOCK);
    let pre = 2f64.powi(WORLD_READ_BUCKET);
    let mut blocks: HashMap<(i32, i32), Vec<Vec<Q>>> = HashMap::new();
    let mut edges: HashMap<(i32, i32), [f32; 4]> = HashMap::new();
    for (i, (a, o, path)) in names.iter().enumerate() {
        if i % 16 == 0 && !go_on() {
            return None;
        }
        let t0 = std::time::Instant::now();
        let Some(rings) = std::fs::read(path).ok().and_then(|b| parse_shp(&b)) else {
            // Present but unreadable: dry, as the tiles' own answer has it.
            edges.insert((*a, *o), [0.0; 4]);
            continue;
        };
        let t = tile_from_rings(*a, *o, rings);
        edges.insert((*a, *o), t.edges);
        let k = ((*a as f64 + 0.5).to_radians().cos()).max(0.05);
        let into = blocks.entry(block_of(*a, *o)).or_default();
        for r in &t.rings {
            if let Some(f) = wire_ring(r, pre, k, Some((*a as f64, *o as f64))) {
                let q = |v: f64| (v * QD as f64).round() as i32;
                into.push(f.chunks(2).map(|p| (q(p[0]), q(p[1]))).collect());
            }
        }
        if pace {
            std::thread::sleep(t0.elapsed().min(std::time::Duration::from_millis(50)));
        }
    }
    if !names.is_empty() {
        // The open sea: the missing tiles that are water, as squares.
        let wet = water_tiles(
            (-90, 89),
            (-180, 179),
            &|a, o| edges.contains_key(&(a, o)),
            &|a, o, side| edges.get(&(a, o)).map_or(0.0, |e| e[side]),
        );
        for (a, o) in wet {
            let (s, w, n, e) = (a * QD, o * QD, (a + 1) * QD, (o + 1) * QD);
            let sq = vec![(s, w), (n, w), (n, e), (s, e), (s, w)];
            blocks.entry(block_of(a, o)).or_default().push(sq);
        }
    }
    let tol = 2f64.powi(WORLD_BUCKET);
    let mut keys: Vec<(i32, i32)> = blocks.keys().copied().collect();
    keys.sort_unstable();
    let (mut out, mut rings, mut bytes) = (Vec::new(), 0usize, 0usize);
    for key in keys {
        if !go_on() {
            return None;
        }
        let t0 = std::time::Instant::now();
        let thin: Vec<Vec<i32>> = dissolve(&blocks.remove(&key).unwrap_or_default())
            .iter()
            .filter_map(|r| world_ring(r, tol, key))
            .collect();
        rings += thin.len();
        bytes += thin.iter().map(|r| r.len() * 4 + 24).sum::<usize>() + 40;
        out.push((key, thin));
        if pace {
            std::thread::sleep(t0.elapsed().min(std::time::Duration::from_millis(50)));
        }
    }
    Some(World {
        blocks: out,
        tiles_read: names.len(),
        rings,
        bytes,
        answers: Mutex::new(Lru::new(16)),
    })
}

/// Build the world layer once per run, on its own thread, in the
/// background; called when the EFB starts serving (never while it is off).
/// The EFB turned off mid-build cancels it (the next start builds again).
/// Logs one line and a blackbox `efb` `basemap_world` record when built.
pub(crate) fn start_world(dir: PathBuf) {
    {
        let mut w = lock(&WORLD);
        if w.is_some() {
            return;
        }
        *w = Some(WorldState::Building);
    }
    let spawned = std::thread::Builder::new()
        .name("efb-world".into())
        .spawn(move || {
            let t0 = std::time::Instant::now();
            let built = build_world(&dir, &|| crate::settings::live().efb, true);
            let Some(w) = built else {
                *lock(&WORLD) = None;
                tracing::info!("EFB world basemap: cancelled, the EFB turned off");
                return;
            };
            let ms = t0.elapsed().as_millis() as u64;
            tracing::info!(
                ms,
                tiles = w.tiles_read,
                rings = w.rings,
                bytes = w.bytes,
                "EFB world basemap built"
            );
            crate::blackbox::record(
                "efb",
                serde_json::json!({ "event": "basemap_world", "ms": ms, "tiles": w.tiles_read,
                                    "rings": w.rings, "bytes": w.bytes }),
            );
            *lock(&WORLD) = Some(WorldState::Ready(Arc::new(w)));
        });
    if spawned.is_err() {
        *lock(&WORLD) = None;
    }
}

/// A wide view from the world layer: the blocks under it (thinned again
/// when the view is coarser than the layer), under [`MAX_BYTES`] like any
/// answer; `bbox` the blocks' tiles. No fields: a view this wide is past
/// [`TOWERED_SPAN_DEG`].
pub(crate) fn world_answer(w: &World, bbox: [f64; 4], px: f64) -> Arc<Answer> {
    let [lat0, lon0, lat1, lon1] = bbox;
    let (a0, o0, a1, o1) = tile_range(lat0, lon0, lat1, lon1);
    let (b0, c0) = (a0.div_euclid(BLOCK) * BLOCK, o0.div_euclid(BLOCK) * BLOCK);
    let (b1, c1) = (a1.div_euclid(BLOCK) * BLOCK, o1.div_euclid(BLOCK) * BLOCK);
    // Mercator's unit is a degree of longitude: the pixel is the span's.
    let dpp = (lon1 - lon0).abs().max(1e-6) / px.clamp(50.0, 8_000.0);
    let mut bucket = (dpp.log2().round() as i32).max(WORLD_BUCKET);
    let key = (b0, c0, b1, c1, bucket);
    if let Some(a) = lock(&w.answers).get(&key) {
        return a;
    }
    let in_view = |(a, o): (i32, i32)| (b0..=b1).contains(&a) && (c0..=c1).contains(&o);
    loop {
        let tol = 2f64.powi(bucket);
        // Decimals for an eighth of the tolerance (2 to 4): a whole degree
        // stays whole, so the page still leaves the blocks' sides unstroked.
        let d = 10f64.powi((-(tol / 8.0).log10()).ceil().clamp(2.0, 4.0) as i32);
        let deg = |r: &[i32]| -> Vec<f64> {
            r.iter()
                .map(|v| (*v as f64 / QD as f64 * d).round() / d)
                .collect()
        };
        let mut water: Vec<Vec<f64>> = Vec::new();
        for (block, rings) in w.blocks.iter().filter(|(b, _)| in_view(*b)) {
            for r in rings {
                if bucket == WORLD_BUCKET {
                    water.push(deg(r));
                } else {
                    let q: Vec<Q> = r.chunks(2).map(|p| (p[0], p[1])).collect();
                    water.extend(world_ring(&q, tol, *block).map(|f| deg(&f)));
                }
            }
        }
        let out = Out {
            bbox: [b0, c0, (b1 + BLOCK - 1).min(89), (c1 + BLOCK - 1).min(179)],
            bucket,
            tol,
            water,
            fields: Vec::new(),
        };
        let json = serde_json::to_string(&out).unwrap_or_default();
        if json.len() <= MAX_BYTES || bucket >= 4 {
            let a = Arc::new(Answer {
                etag: crate::efb::airport::version_of(&json),
                rings: out.water.len(),
                json,
            });
            lock(&w.answers).put(key, a.clone());
            return a;
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
        shapefile(&[&outer, &hole])
    }

    /// A one-record Polygon shapefile of these rings, (x = lon, y = lat).
    fn shapefile(parts: &[&[(f64, f64)]]) -> Vec<u8> {
        let pts: Vec<(f64, f64)> = parts.iter().flat_map(|p| p.iter().copied()).collect();
        let mut c: Vec<u8> = Vec::new();
        c.extend(5i32.to_le_bytes());
        for v in [0.0f64; 4] {
            c.extend(v.to_le_bytes());
        }
        c.extend((parts.len() as i32).to_le_bytes());
        c.extend((pts.len() as i32).to_le_bytes());
        let mut start = 0i32;
        for p in parts {
            c.extend(start.to_le_bytes());
            start += p.len() as i32;
        }
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
    /// continent before the world layer is built: not ready; not a box: a
    /// bad view.
    #[test]
    fn no_map_data_is_no_water_and_a_continent_waits_for_the_world() {
        let none = Path::new("/nonexistent/map data/water");
        let a = answer_with(none, None, [-12.5, -150.5, -11.5, -149.5], 800.0).expect("an answer");
        let v: serde_json::Value = serde_json::from_str(&a.json).unwrap();
        assert_eq!(v["water"], serde_json::json!([]));
        let wide = [-60.0, -170.0, 60.0, 170.0];
        assert_eq!(
            answer_with(none, None, wide, 800.0).err(),
            Some(Miss::NotReady)
        );
        let nan = [f64::NAN, 0.0, 1.0, 1.0];
        assert_eq!(
            answer_with(none, None, nan, 800.0).err(),
            Some(Miss::BadView)
        );
        // No map data at all: the world is empty, and a continent no water.
        let w = build_world(none, &|| true, false).expect("a world");
        assert_eq!((w.tiles_read, w.rings), (0, 0));
        let a = answer_with(none, Some(&w), wide, 800.0).expect("an answer");
        let v: serde_json::Value = serde_json::from_str(&a.json).unwrap();
        assert_eq!(v["water"], serde_json::json!([]));
    }

    #[test]
    fn the_bucket_is_about_a_pixel() {
        // 2 degrees of latitude-equivalent over 1,000 px: 0.002, 2^-9.
        let b = bucket_for(50.0, -1.0, 52.0, 2.11, 1000.0);
        assert_eq!(b, -9);
        assert!(bucket_for(30.0, -10.0, 52.0, 5.0, 1000.0) > b);
    }

    /// The world layer over a fixture install of two tiles south of New
    /// Zealand (a sea tile with an island; a coast with a wiggle and a lake
    /// too small to see) and one damaged file: read, thinned, the open sea
    /// classed; a whole-world view and a clipped one served from it, under
    /// the cap; a narrow view still the tiles' own; cancelled, nothing.
    #[test]
    fn the_world_layer_serves_wide_views_from_every_tile() {
        let dir = std::env::temp_dir().join(format!("xvl-basemap-world-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let put = |a: i32, o: i32, b: &[u8]| {
            let p = tile_path(&dir, a, o);
            std::fs::create_dir_all(p.parent().unwrap()).unwrap();
            std::fs::write(p, b).unwrap();
        };
        let sea = [
            (166.0, -47.0),
            (166.0, -46.0),
            (167.0, -46.0),
            (167.0, -47.0),
            (166.0, -47.0),
        ];
        let island = [
            (166.25, -46.75),
            (166.75, -46.75),
            (166.75, -46.25),
            (166.25, -46.25),
            (166.25, -46.75),
        ];
        put(-47, 166, &shapefile(&[&sea, &island]));
        let mut coast: Vec<(f64, f64)> = vec![(167.0, -47.0), (167.0, -46.5)];
        coast.extend((1..100).map(|i| (167.0 + i as f64 / 100.0, -46.5 + 0.002 * (i % 2) as f64)));
        coast.extend([(168.0, -46.5), (168.0, -47.0), (167.0, -47.0)]);
        let lake = [
            (167.5, -46.2),
            (167.5, -46.199),
            (167.501, -46.199),
            (167.5, -46.2),
        ];
        put(-47, 167, &shapefile(&[&coast, &lake]));
        put(10, 10, b"not a shapefile");
        std::fs::write(dir.join("readme.txt"), b"x").unwrap();

        assert!(build_world(&dir, &|| false, false).is_none(), "cancelled");
        let w = build_world(&dir, &|| true, false).expect("a world");
        assert_eq!(w.tiles_read, 3);
        // Dissolved: the sides between water tiles are gone (the corner
        // all four share), the coast thinned, the lake dropped.
        let block = &w
            .blocks
            .iter()
            .find(|b| b.0 == (-50, 160))
            .expect("the block")
            .1;
        let pts: Vec<(i32, i32)> = block
            .iter()
            .flat_map(|r| r.chunks(2).map(|p| (p[0], p[1])))
            .collect();
        assert!(pts.len() < 80, "{} points: {block:?}", pts.len());
        assert!(!pts.contains(&(-470_000, 1_660_000)), "a shared corner");
        assert!(pts.contains(&(-462_500, 1_662_500)), "the island");
        assert!(!pts.iter().any(|p| (p.0 + 462_000).abs() < 50), "the lake");

        // The whole world, as the page asks for it zoomed right out.
        let rings_of = |a: &Answer| -> (serde_json::Value, Vec<Vec<f64>>) {
            let v: serde_json::Value = serde_json::from_str(&a.json).unwrap();
            let r = serde_json::from_value(v["water"].clone()).unwrap();
            (v, r)
        };
        let all = answer_with(&dir, Some(&w), [-85.05, -180.0, 85.05, 179.999], 2560.0).unwrap();
        assert!(all.json.len() <= MAX_BYTES);
        let (v, water) = rings_of(&all);
        assert!(v["bucket"].as_i64().unwrap() >= WORLD_BUCKET as i64);
        assert_eq!(v["fields"], serde_json::json!([]));
        let has = |rings: &[Vec<f64>], lat: f64, lon: f64| {
            rings
                .iter()
                .any(|r| r.chunks(2).any(|p| p[0] == lat && p[1] == lon))
        };
        assert!(has(&water, -46.25, 166.25), "the island");
        let wet = |rings: &[Vec<f64>], lat: f64, lon: f64| {
            let r: Vec<Ring> = rings
                .iter()
                .map(|r| r.chunks(2).map(|p| (p[1], p[0])).collect())
                .collect();
            inside(&r, lon, lat)
        };
        assert!(wet(&water, -47.5, 166.5), "the open sea south of it");
        assert!(
            wet(&water, -46.9, 167.5) && !wet(&water, -46.1, 167.5),
            "the coast"
        );
        assert!(!wet(&water, -46.5, 166.5), "the island");

        // Clipped to the blocks under the view: the antimeridian is an edge
        // of the world, never crossed.
        let east = answer_with(&dir, Some(&w), [-60.0, 100.0, -5.0, 179.999], 1400.0).unwrap();
        let (v, water) = rings_of(&east);
        assert_eq!(v["bbox"], serde_json::json!([-60, 100, -1, 179]));
        assert!(has(&water, -46.25, 166.25));
        for p in water.iter().flat_map(|r| r.chunks(2)) {
            assert!(
                (-60.0..=0.0).contains(&p[0]) && (100.0..=180.0).contains(&p[1]),
                "{p:?}"
            );
        }
        let west = answer_with(&dir, Some(&w), [20.0, -100.0, 70.0, -10.0], 1400.0).unwrap();
        let (_, water) = rings_of(&west);
        assert!(!has(&water, -46.25, 166.25));
        assert!(water
            .iter()
            .flat_map(|r| r.chunks(2))
            .all(|p| (-100.0..=0.0).contains(&p[1])));

        // A narrow view is the tiles' own answer, world or not.
        let narrow = [-47.5, 166.2, -46.3, 167.8];
        let a = answer_with(&dir, Some(&w), narrow, 1000.0).unwrap();
        let b = answer_with(&dir, None, narrow, 1000.0).unwrap();
        assert_eq!(a.json, b.json);
        assert!(has(&rings_of(&a).1, -46.25, 166.25));
        let _ = std::fs::remove_dir_all(&dir);
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

    /// The world layer on the real install: its build time and size, and
    /// the wide views it serves.
    /// `cargo test --release -- --ignored real_install_world_basemap --nocapture`
    #[test]
    #[ignore]
    fn real_install_world_basemap() {
        let dir = water_dir();
        let t0 = std::time::Instant::now();
        let w = build_world(&dir, &|| true, true).expect("a world");
        println!(
            "world: {} ms paced as live, {} tiles, {} rings, {} bytes held",
            t0.elapsed().as_millis(),
            w.tiles_read,
            w.rings,
            w.bytes
        );
        for (name, bbox, px) in [
            ("world 2560", [-85.05, -180.0, 85.05, 179.999], 2560.0),
            ("world 1400", [-85.05, -180.0, 85.05, 179.999], 1400.0),
            ("world phone", [-85.05, -180.0, 85.05, 179.999], 390.0),
            ("europe 1400", [30.0, -25.0, 72.0, 45.0], 1400.0),
            ("pacific 1400", [-50.0, 140.0, 10.0, 179.999], 1400.0),
        ] {
            let t0 = std::time::Instant::now();
            let a = answer_with(&dir, Some(&w), bbox, px).expect("an answer");
            let v: serde_json::Value = serde_json::from_str(&a.json).unwrap();
            println!(
                "{name}: {} bytes, {} rings, bucket {}, {} ms",
                a.json.len(),
                a.rings,
                v["bucket"],
                t0.elapsed().as_millis()
            );
            assert!(a.json.len() <= MAX_BYTES && a.rings > 0);
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
