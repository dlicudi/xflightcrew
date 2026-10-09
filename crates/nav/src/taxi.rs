//! The taxiway network (apt.dat 1201/1202 rows): routes, vacates, stands.

use super::*;

// --- Taxiway network (apt.dat 1201/1202 rows) --------------------------------

/// One taxi-network edge. Runway edges are routable — crossing a runway is a
/// real taxi instruction — but heavily penalized so a route never taxis
/// ALONG one when a taxiway exists.
#[derive(Debug, Clone)]
pub struct TaxiEdge {
    pub a: i64,
    pub b: i64,
    /// apt.dat "oneway": traversable a→b only.
    pub oneway: bool,
    pub runway: bool,
    /// Taxiway name ("A", "B2"); runway edges carry the designator pair
    /// ("09L/27R"). May be empty (unnamed links).
    pub name: String,
    /// Runway ends whose ACTIVE ZONE this edge lies inside (apt.dat 1204
    /// arrival/departure rows) — the authored equivalent of the hold-short
    /// lines: the zone boundary is where an aircraft holds. `ils` rows
    /// (the CAT/LVP holds, further out) are deliberately not collected
    /// until low-visibility ops exist.
    pub zone_of: Vec<String>,
}

/// A parking position — apt.dat row 1300 (ramp start location). The endpoint
/// an arrival taxi routes to.
#[derive(Debug, Clone)]
pub struct Stand {
    pub lat: f64,
    pub lon: f64,
    /// True heading the aircraft parks on (nose direction).
    pub heading: f64,
    /// The stand name as painted ("Stand 12", "A7", "GA North").
    pub name: String,
    /// apt.dat location type: gate / hangar / tie_down / misc. A hangar is not
    /// a suitable parking stand to send an arrival to.
    pub kind: String,
    /// Aircraft categories the stand accepts (apt.dat field: heavy / jets /
    /// turboprops / props / helos / fighters), lowercased. Empty = unspecified.
    pub accepts: Vec<String>,
    /// The biggest aircraft the stand takes, as its ICAO wingspan code
    /// ('A' under 15 m to 'F' under 80 m): the 1301 row under the 1300.
    /// `None` where the file gives none.
    pub width: Option<char>,
}

impl Stand {
    /// A stand an arrival can actually park at — anything but a hangar.
    pub fn is_suitable(&self) -> bool {
        !self.kind.eq_ignore_ascii_case("hangar")
    }
}

/// The taxi routing network of one airport.
#[derive(Debug, Clone, Default)]
pub struct TaxiNet {
    pub nodes: std::collections::HashMap<i64, (f64, f64)>,
    pub edges: Vec<TaxiEdge>,
    /// Parking stands (apt.dat 1300), for arrival taxi-to-parking.
    pub stands: Vec<Stand>,
    /// The PAINT: every drawn taxi centreline at the field, as polylines with
    /// their beziers already sampled (apt.dat 120 linear features carrying
    /// line type 1 or 51). Empty at a field that draws none, or when the
    /// caller built the net without them.
    ///
    /// The network and the paint answer two different questions and the
    /// network is only authoritative for one of them. 1201/1202 give
    /// TOPOLOGY — what connects to what, and what it is called — and they are
    /// authored by hand as an approximation of the pavement, so their
    /// geometry can sit metres from the line the pilot is looking at. At
    /// Malaga the Baron tracked its planned route to within 0.12 m for a
    /// kilometre and taxied the whole way 1.7 to 2.8 m to the side of the
    /// drawn line, up to 6 m through the taxiway E junction (2026-08-05).
    /// It is the same lesson the runway lead-on already carries: the entry
    /// onto the strip is DRAWN, and inventing it is how an aircraft ends up
    /// in the grass. See [`TaxiNet::ride_paint`].
    pub paint: Vec<Vec<(f64, f64)>>,
    /// The PAVEMENT: every apt.dat 110 polygon at the field, each as its
    /// rings (the first the outline, the rest holes), beziers sampled. Empty
    /// where the field draws none. A tug's pattern is the sim's geometry,
    /// not ours, and at LIPK it ran 98 m straight and left a 172 on the
    /// grass, where the taxi could not move it (2026-09-26): the push stops
    /// at the pavement's edge ([`TaxiNet::on_pavement`]).
    pub pavement: Vec<Vec<Vec<(f64, f64)>>>,
    /// The stands' LEAD-IN lines (apt.dat line type 60, the painted curve
    /// from a taxiway into a stand), beziers sampled. Read backwards from the
    /// stand it is the way a push should go ([`TaxiNet::push_lane`]); the
    /// push used to aim at the route's first node instead, and swung a 737's
    /// tail into the buildings beside LIBP's stand 1 (2026-09-26). Type 60
    /// ends at 650 stands across fourteen fields surveyed that day.
    pub stand_lines: Vec<Vec<(f64, f64)>>,
}

/// How far a drawn centreline may be from the planned route and still be the
/// line that route MEANT. Sized off the error being corrected: Malaga's
/// network sits 1.7 to 2.8 m off its paint on the straights and about 6 m
/// through a junction, and the next parallel line at a normal field is a
/// taxiway width away. Wider than this is not a mis-drawn network, it is a
/// different taxiway, and moving a route there would be inventing one.
pub(super) const PAINT_SNAP_M: f64 = 8.0;

/// How far off the leg's own direction a drawn line may run and still be the
/// same line. This is what stops a route being dragged onto the taxiway it is
/// crossing, or onto a hold bar drawn square across it: those run at an angle
/// no continuation of this leg does.
pub(super) const PAINT_PARALLEL_DEG: f64 = 25.0;

/// Spacing of the samples a leg is broken into before snapping. The drawn
/// line curves where the network draws a corner, so a leg has to be sampled
/// to follow it at all — and the samples ARE the new route, so this is also
/// how finely the drive can be asked to trace a fillet.
pub(super) const PAINT_SAMPLE_M: f64 = 5.0;

/// How far past the network's last node a stand may sit and still be driven
/// to. The routing network stops at the taxiway; the apron lead-in into a
/// stand is usually not modelled at all, so without this the drive finishes
/// at the nearest node and parks ON THE TAXIWAY — 66 m from stand W5 at
/// Malaga, across the lane the tower had just cleared somebody else down
/// (2026-08-05). Bounded because past this the "stand" is not the one this
/// route serves and driving blind across an apron is worse than stopping.
pub(super) const STAND_LEAD_IN_MAX_M: f64 = 120.0;

/// ...and inside this the route already ends at the stand; anything shorter
/// is inside the tolerance of where the aircraft parks anyway.
pub(super) const STAND_LEAD_IN_MIN_M: f64 = 6.0;

/// A stand is entered along its parking line from this far back.
pub(super) const STAND_ALIGN_M: f64 = 25.0;

/// ...when the turn at that point is no more than this.
pub(super) const STAND_ALIGN_MAX_TURN_DEG: f64 = 90.0;

/// ...unless the straight lead-in already arrives within this of the stand's
/// heading.
pub(super) const STAND_ALIGN_OK_DEG: f64 = 30.0;

/// The lead-in's points from the route's last node `end` to the stand: the
/// straight line when it already arrives on the stand's heading, else via a
/// point [`STAND_ALIGN_M`] back along the parking line.
pub(super) fn stand_lead_in(end: (f64, f64), stand: &Stand, m_per_deg_lon: f64) -> Vec<(f64, f64)> {
    let spot = (stand.lat, stand.lon);
    if !stand.heading.is_finite() || m_per_deg_lon <= 0.0 {
        return vec![end, spot];
    }
    let straight = bearing_deg(end.0, end.1, spot.0, spot.1);
    if sim::angles::abs_diff_deg(straight, stand.heading) <= STAND_ALIGN_OK_DEG {
        return vec![end, spot];
    }
    let h = stand.heading.to_radians();
    let back = (
        spot.0 - STAND_ALIGN_M * h.cos() / 111_320.0,
        spot.1 - STAND_ALIGN_M * h.sin() / m_per_deg_lon,
    );
    // Only a turn the aircraft can make: the leg to the back point must not
    // double back on the stand line. LOWI (2026-09-24) put the back point
    // BEHIND the route's end, the drive needed a 140-170 degree reversal at
    // walking pace, circled, and stopped "off the cleared route" 25 m out.
    let to_back = bearing_deg(end.0, end.1, back.0, back.1);
    if sim::angles::abs_diff_deg(to_back, stand.heading) > STAND_ALIGN_MAX_TURN_DEG {
        return vec![end, spot];
    }
    vec![end, back, spot]
}

/// The default hold-short standoff: how far back from the runway-junction
/// node a planned route ends. Clears the painted line at GA fields
/// (30-50 m from the centreline); callers pass more for instrument
/// runways, whose holding positions sit ~100 m out.
pub const HOLD_SHORT_STANDOFF_M: f64 = 40.0;

/// How far past the runway's zone boundary a vacate drives before it is
/// finished. The mirror of [`HOLD_SHORT_STANDOFF_M`]: a departure stops that
/// far SHORT of the line, an arrival ends that far PAST it, and for the same
/// reason — the aircraft has length, and the nose being over the line is not
/// the tail being over it.
pub const VACATE_CLEAR_M: f64 = 60.0;

/// An exit more than this off the rollout heading is BEHIND us: reaching it
/// would mean taxiing back down the runway, which is worse than the longer
/// exit ahead. Abeam counts as ahead — we are stopped, so the drive's own
/// join owns the turn.
pub(super) const AHEAD_OF_ABEAM_DEG: f64 = 100.0;

/// Hard stop on the vacate walk. A field whose active zones swallow half the
/// apron would otherwise send us to the terminal: this is the first hundred
/// metres off the strip, not the taxi in — the taxi-in clearance owns the
/// rest, and it is requested from wherever this ends.
pub(super) const MAX_VACATE_STEPS: usize = 8;

/// How far from an authored runway edge still counts as ON the strip. A
/// vacate planned from anywhere else is a drive TOWARD a runway, not off one:
/// the caller's gate is the standing landing clearance, which holds from
/// touchdown until the taxi-in request — so a "taxi to parking" retried from
/// a taxiway after a give-up re-entered here and aimed the aircraft back at
/// the runway entrance (audit, 2026-08-02). Only judgeable where runway
/// edges are authored; thin data keeps the old behaviour, as everywhere else
/// in the vacate.
pub(super) const VACATE_ON_RUNWAY_M: f64 = 60.0;

/// The most a vacate may turn off the rollout heading to leave the runway.
/// A right-angle turn-off is standard and a rapid exit is 30 to 45 degrees;
/// past this the link is one built for the OTHER end of the strip and taking
/// it means reversing on the runway (EGLL's N4W, 169 degrees off a 27L
/// rollout, 2026-09-17). But a turn-off a little past abeam is an ordinary
/// turn at taxi pace: EDLP's B leaves 24 at 108 degrees, 406 m ahead of an
/// SF50 rolling at 9 kt, and at 100 it was refused for C, 970 m further
/// down; the vacate drove 1,217 m at turn pace, ran out of time on the
/// runway, and the taxi-in began with a backtrack (2026-09-24).
pub(super) const VACATE_TURN_MAX_DEG: f64 = 120.0;

/// The exit snap's distance bound — the same 3 km every other snap in this
/// file caps at (`snap_within`, `runway_entrance_near`). Without it a stale
/// clearance at one field planned a "vacate" toward a junction kilometres
/// away at another.
pub(super) const MAX_EXIT_SNAP_M: f64 = 3_000.0;

/// A planned exit from the runway we have just landed on: the polyline to
/// drive, and the taxiway it puts us on.
///
/// Deliberately NOT a [`TaxiRoutePlan`]. There is no clearance here, no
/// holding point and no via list to read back — vacating is the completion of
/// the landing clearance we already hold, not a new instruction, and dressing
/// it as a cleared route would invite a readback for something nobody issued.
#[derive(Debug, Clone)]
pub struct VacatePlan {
    /// Starts AT THE AIRCRAFT, ends clear of the runway's protected zone.
    pub path: Vec<(f64, f64)>,
    /// Per-segment `(name, is_runway)`, aligned with `path` exactly as
    /// [`TaxiRoutePlan::seg`] is — the drive and the guide read both the same
    /// way.
    pub seg: Vec<(String, bool)>,
    /// The taxiway we end up on, for the crew line and the "runway vacated"
    /// report. May be empty on an unnamed link.
    pub exit_name: String,
}

/// A stand route that begins ON the runway: the drive off it, forward, then
/// the route to the stand from where that drive ends.
///
/// Planned from the runway itself, a stand route snaps to the nearest node
/// whatever the heading, and that node is as likely behind as ahead: EDLP 24
/// (SF50, 2026-09-24) the vacate timed out 714 m short of exit C, the stand
/// route was planned from 51.61388 N 8.61568 E heading 236, and it started
/// back up the runway (the drive joined 179 degrees off) with a clearance
/// that said "cross runway zero six" for the strip we were already on. The
/// vacate knows the way off ([`TaxiNet::plan_vacate`]: exits ahead only,
/// toward the apron); the stand route starts where it ends.
///
/// `rest` is the stand route from the vacate's end, its crossings already
/// marked: the vacate's own runway legs are the landing clearance's, never
/// a crossing. The vacate's exit leads the `via`.
pub fn after_vacate(vacate: VacatePlan, rest: TaxiRoutePlan) -> TaxiRoutePlan {
    let mut path = vacate.path;
    let mut seg = vacate.seg;
    let mut tail_path = rest.path.into_iter();
    // The stand route starts where the vacate ends (its first point is the
    // position it was planned from): the duplicate goes, and the stand
    // route's first leg then runs from the vacate's end. Apart, a connecting
    // leg joins them.
    match (path.last(), tail_path.as_slice().first()) {
        (Some(&end), Some(first)) => {
            let m_per_deg_lon = 111_320.0 * end.0.to_radians().cos();
            let gap = ((first.1 - end.1) * m_per_deg_lon).hypot((first.0 - end.0) * 111_320.0);
            if gap < 1.0 {
                tail_path.next();
            } else {
                seg.push((String::new(), false));
            }
        }
        // No vacate to lead with: the stand route whole.
        (None, _) => seg.clear(),
        (Some(_), None) => {}
    }
    path.extend(tail_path);
    seg.extend(rest.seg);
    let mut via = Vec::with_capacity(rest.via.len() + 1);
    if !vacate.exit_name.is_empty() && rest.via.first() != Some(&vacate.exit_name) {
        via.push(vacate.exit_name);
    }
    via.extend(rest.via);
    TaxiRoutePlan {
        via,
        crossings: rest.crossings,
        holding_point: rest.holding_point,
        path,
        seg,
        lead_on: Vec::new(),
    }
}

/// A planned taxi route, ready to phrase: ordered taxiway names, runways
/// crossed, the holding-point name, and the node polyline for the
/// off-route compliance watch. Names are RAW apt.dat strings — the station
/// speaks them.
#[derive(Debug, Clone, Default)]
pub struct TaxiRoutePlan {
    pub via: Vec<String>,
    pub crossings: Vec<String>,
    pub holding_point: String,
    pub path: Vec<(f64, f64)>,
    /// Per-segment (name, is_runway) aligned with `path`: `seg[i]` describes
    /// the leg `path[i] -> path[i+1]` (so `seg.len() == path.len() - 1`).
    /// This is what lets the copilot's taxi guide say "left onto Bravo"
    /// instead of just "left": the planner knew the names all along.
    pub seg: Vec<(String, bool)>,
    /// The tail the hold-short trim REMOVED: the authored lead-on from the
    /// route's end (the holding point) onto the runway-centreline junction —
    /// the painted yellow line from the hold onto the strip. Empty when the
    /// route wasn't trimmed (it ends at a genuine off-runway node). The
    /// line-up drives THIS instead of inventing a diagonal (the aircraft cut
    /// the Echo stub's curve and stopped crooked, LXGB 2026-07-24).
    pub lead_on: Vec<(f64, f64)>,
}

/// Parse `icao`'s taxi network from apt.dat — same streaming block scan as
/// [`find_airport`], collecting 1201 (node) and 1202 (edge) rows, plus the
/// drawn taxi centrelines (120 linear features) the routing geometry is only
/// an approximation of. `None` when the field has no network (small strips
/// mostly don't) — the paint alone is not a network, since it carries no
/// connectivity and no names.
pub fn find_taxi_net(path: &str, icao: &str) -> Result<Option<TaxiNet>> {
    let file = std::fs::File::open(path).with_context(|| format!("open {path}"))?;
    let mut reader = BufReader::with_capacity(1 << 20, file);
    let mut net = TaxiNet::default();
    let mut in_airport = false;
    let mut buf = Vec::with_capacity(256);
    // The linear feature being read. Nodes only count as PAINT inside a 120
    // block: the same 111-116 rows draw pavement boundaries (110) and the
    // airport perimeter (130), and a boundary is not a line to taxi down.
    let mut in_feature = false;
    let mut feature: Vec<PaintNode> = Vec::new();
    // The same 120 feature, flagged by LEAD-IN segments (type 60).
    let mut leadin: Vec<PaintNode> = Vec::new();
    // The pavement polygon being read (a 110 block), and its current ring.
    let mut in_pavement = false;
    let mut ring: Vec<PaintNode> = Vec::new();
    // The row before was a 1300 that made a stand: a 1301 there is its size.
    let mut stand_open = false;
    loop {
        buf.clear();
        if reader.read_until(b'\n', &mut buf).unwrap_or(0) == 0 {
            break;
        }
        if buf.first() != Some(&b'1') {
            continue;
        }
        let Ok(line) = std::str::from_utf8(&buf) else {
            continue;
        };
        let mut f = line.split_whitespace();
        let row = f.next();
        let after_stand = std::mem::replace(&mut stand_open, row == Some("1300"));
        // Any row that opens a new block ends the feature being read.
        if !matches!(row, Some("111" | "112" | "113" | "114" | "115" | "116")) {
            flush_paint_feature(&mut feature, &mut net.paint);
            flush_paint_feature(&mut leadin, &mut net.stand_lines);
            in_feature = in_airport && row == Some("120");
            ring.clear();
            in_pavement = in_airport && row == Some("110");
            if in_pavement {
                net.pavement.push(Vec::new());
            }
        }
        match row {
            Some("111" | "112" | "113" | "114" | "115" | "116") if in_pavement => {
                let (Some(lat), Some(lon)) = (
                    f.next().and_then(|s| s.parse::<f64>().ok()),
                    f.next().and_then(|s| s.parse::<f64>().ok()),
                ) else {
                    continue;
                };
                let ctrl = if matches!(row, Some("112" | "114" | "116")) {
                    match (
                        f.next().and_then(|s| s.parse::<f64>().ok()),
                        f.next().and_then(|s| s.parse::<f64>().ok()),
                    ) {
                        (Some(a), Some(b)) => Some((a, b)),
                        _ => None,
                    }
                } else {
                    None
                };
                ring.push(PaintNode {
                    p: (lat, lon),
                    ctrl,
                    centre: true,
                });
                // 113/114 close a ring; the next ring of the same polygon
                // is a hole.
                if matches!(row, Some("113" | "114")) {
                    if let Some(&first) = ring.first() {
                        ring.push(first);
                    }
                    let mut sampled: Vec<Vec<(f64, f64)>> = Vec::new();
                    flush_paint_feature(&mut ring, &mut sampled);
                    if let (Some(r), Some(poly)) = (sampled.into_iter().next(), net.pavement.last_mut()) {
                        poly.push(r);
                    }
                    ring.clear();
                }
            }
            Some("111" | "112" | "113" | "114" | "115" | "116") if in_feature => {
                let (Some(lat), Some(lon)) = (
                    f.next().and_then(|s| s.parse::<f64>().ok()),
                    f.next().and_then(|s| s.parse::<f64>().ok()),
                ) else {
                    continue;
                };
                // 112/114/116 carry the bezier control point before the
                // attributes; the plain rows go straight to them.
                let ctrl = if matches!(row, Some("112" | "114" | "116")) {
                    match (
                        f.next().and_then(|s| s.parse::<f64>().ok()),
                        f.next().and_then(|s| s.parse::<f64>().ok()),
                    ) {
                        (Some(a), Some(b)) => Some((a, b)),
                        _ => None,
                    }
                } else {
                    None
                };
                let attrs: Vec<&str> = f.collect();
                let centre = attrs.iter().any(|a| PAINT_CENTRELINE_TYPES.contains(a));
                let lead = attrs.contains(&LEADIN_TYPE);
                feature.push(PaintNode {
                    p: (lat, lon),
                    ctrl,
                    centre,
                });
                leadin.push(PaintNode {
                    p: (lat, lon),
                    ctrl,
                    centre: lead,
                });
                // 113/114 close a loop: the drawn line returns to its start.
                if matches!(row, Some("113" | "114")) {
                    if let Some(&first) = feature.first() {
                        feature.push(first);
                    }
                    if let Some(&first) = leadin.first() {
                        leadin.push(first);
                    }
                    flush_paint_feature(&mut feature, &mut net.paint);
                    flush_paint_feature(&mut leadin, &mut net.stand_lines);
                    in_feature = false;
                } else if matches!(row, Some("115" | "116")) {
                    flush_paint_feature(&mut feature, &mut net.paint);
                    flush_paint_feature(&mut leadin, &mut net.stand_lines);
                    in_feature = false;
                }
            }
            Some("1") | Some("16") | Some("17") => {
                if in_airport {
                    break;
                }
                in_airport = f.nth(3) == Some(icao);
            }
            Some("1201") if in_airport => {
                // lat lon usage id [name...]
                let (lat, lon) = match (
                    f.next().and_then(|s| s.parse::<f64>().ok()),
                    f.next().and_then(|s| s.parse::<f64>().ok()),
                ) {
                    (Some(a), Some(b)) => (a, b),
                    _ => continue,
                };
                let _usage = f.next();
                if let Some(id) = f.next().and_then(|s| s.parse::<i64>().ok()) {
                    net.nodes.insert(id, (lat, lon));
                }
            }
            Some("1202") if in_airport => {
                // a b twoway|oneway runway|taxiway [name...]
                let (a, b) = match (
                    f.next().and_then(|s| s.parse::<i64>().ok()),
                    f.next().and_then(|s| s.parse::<i64>().ok()),
                ) {
                    (Some(a), Some(b)) => (a, b),
                    _ => continue,
                };
                let oneway = f.next() == Some("oneway");
                let kind = f.next().unwrap_or("");
                net.edges.push(TaxiEdge {
                    a,
                    b,
                    oneway,
                    runway: kind == "runway",
                    name: f.collect::<Vec<_>>().join(" ").trim().to_string(),
                    zone_of: Vec::new(),
                });
            }
            Some("1300") if in_airport => {
                // lat lon heading loc_type plane_types name...
                // e.g. `1300 52.30 -1.20 90.0 gate jets|turboprops Stand 12`
                let (lat, lon, heading) = match (
                    f.next().and_then(|s| s.parse::<f64>().ok()),
                    f.next().and_then(|s| s.parse::<f64>().ok()),
                    f.next().and_then(|s| s.parse::<f64>().ok()),
                ) {
                    (Some(a), Some(b), Some(h)) => (a, b, h),
                    _ => {
                        stand_open = false;
                        continue;
                    }
                };
                let kind = f.next().unwrap_or("misc").to_string();
                let accepts = f
                    .next()
                    .unwrap_or("")
                    .split('|')
                    .map(|s| s.trim().to_ascii_lowercase())
                    .filter(|s| !s.is_empty())
                    .collect();
                let name = f.collect::<Vec<_>>().join(" ").trim().to_string();
                net.stands.push(Stand {
                    lat,
                    lon,
                    heading,
                    name,
                    kind,
                    accepts,
                    width: None,
                });
            }
            Some("1301") if in_airport && after_stand => {
                // width_code operation_type [airlines...]
                // e.g. `1301 C airline baw afr klm`
                let width = f
                    .next()
                    .and_then(|w| w.chars().next())
                    .map(|c| c.to_ascii_uppercase())
                    .filter(|c| ('A'..='F').contains(c));
                if let Some(s) = net.stands.last_mut() {
                    s.width = width;
                }
            }
            Some("1204") if in_airport => {
                // classification runway-list — attaches to the 1202 row
                // above it. arrival/departure/both zones bound the runway
                // holding points; `ils` zones are the LVP holds, skipped.
                let class = f.next().unwrap_or("");
                if class == "arrival" || class == "departure" || class == "both" {
                    if let (Some(list), Some(e)) = (f.next(), net.edges.last_mut()) {
                        for ident in list.split(',').map(str::trim).filter(|s| !s.is_empty()) {
                            if !e.zone_of.iter().any(|z| z == ident) {
                                e.zone_of.push(ident.to_string());
                            }
                        }
                    }
                }
            }
            _ => {}
        }
    }
    net.stand_lines = stitch_lines(std::mem::take(&mut net.stand_lines), 1.0);
    Ok((!net.edges.is_empty()).then_some(net))
}

/// Join polylines whose ends meet (within `tol_m`) into continuous lines.
/// A lead-in is often drawn as several short pieces (LIBP stand 1: a 2-point
/// straight into the stand, then a 9-point curve, end to end), and a piece
/// shorter than the direction probe says nothing about where it goes.
pub fn stitch_lines(mut lines: Vec<Vec<(f64, f64)>>, tol_m: f64) -> Vec<Vec<(f64, f64)>> {
    let near = |a: (f64, f64), b: (f64, f64)| {
        let k = 111_320.0 * a.0.to_radians().cos();
        (((a.1 - b.1) * k).powi(2) + ((a.0 - b.0) * 111_320.0).powi(2)).sqrt() <= tol_m
    };
    let mut out: Vec<Vec<(f64, f64)>> = Vec::new();
    while let Some(mut cur) = lines.pop() {
        loop {
            let (head, tail) = (cur[0], cur[cur.len() - 1]);
            let Some(i) = lines.iter().position(|l| {
                let (a, b) = (l[0], l[l.len() - 1]);
                near(tail, a) || near(tail, b) || near(head, a) || near(head, b)
            }) else {
                break;
            };
            let mut l = lines.swap_remove(i);
            let (a, b) = (l[0], l[l.len() - 1]);
            if near(tail, a) {
                cur.extend(l.drain(1..));
            } else if near(tail, b) {
                l.reverse();
                cur.extend(l.drain(1..));
            } else if near(head, b) {
                l.extend(cur.drain(1..));
                cur = l;
            } else {
                l.reverse();
                l.extend(cur.drain(1..));
                cur = l;
            }
        }
        out.push(cur);
    }
    out
}

impl TaxiNet {
    /// The way a push from this spot should go: the stand's painted lead-in
    /// read BACKWARDS, from its end at the aircraft out to the taxiway. A
    /// lead-in (or, lacking one, a painted centreline) counts when one of its
    /// ends is within [`LANE_END_M`] of the aircraft and runs INTO that end
    /// along the aircraft's heading (a nose-in stand's line points at the
    /// terminal, as the nose does). `None` where nothing is drawn: the push
    /// then plans off the cleared route as before.
    pub fn push_lane(&self, lat: f64, lon: f64, psi_deg: f64) -> Option<Vec<(f64, f64)>> {
        let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
        let xy = |p: (f64, f64)| ((p.1 - lon) * m_per_deg_lon, (p.0 - lat) * 111_320.0);
        let diff = |a: f64, b: f64| sim::angles::abs_diff_deg(a, b);
        let mut best: Option<(f64, Vec<(f64, f64)>)> = None;
        for line in self.stand_lines.iter().chain(self.paint.iter()) {
            let lead = self.stand_lines.contains(line);
            for i in 0..line.len().saturating_sub(1) {
                let (a, b) = (xy(line[i]), xy(line[i + 1]));
                let (dx, dy) = (b.0 - a.0, b.1 - a.1);
                let len2 = dx * dx + dy * dy;
                if len2 < 0.25 {
                    continue;
                }
                let t = (((-a.0) * dx + (-a.1) * dy) / len2).clamp(0.0, 1.0);
                let (px, py) = (a.0 + t * dx, a.1 + t * dy);
                let d = (px * px + py * py).sqrt();
                if d > LANE_END_M {
                    continue;
                }
                let brg = dx.atan2(dy).to_degrees().rem_euclid(360.0);
                // Along the nose: the way out is back down the line; against
                // it: forward down the line.
                let back = if diff(brg, psi_deg) <= LANE_ALIGN_DEG {
                    true
                } else if diff(brg + 180.0, psi_deg) <= LANE_ALIGN_DEG {
                    false
                } else {
                    continue;
                };
                let p = (lat + py / 111_320.0, lon + px / m_per_deg_lon);
                let mut lane = vec![p];
                if back {
                    lane.extend(line[..=i].iter().rev().copied());
                } else {
                    lane.extend(line[i + 1..].iter().copied());
                }
                // A line drawn on past the stop point toward the terminal is
                // not a way out: only a lane long enough to push along.
                let length: f64 = lane
                    .windows(2)
                    .map(|w| haversine_nm(w[0].0, w[0].1, w[1].0, w[1].1) * 1852.0)
                    .sum();
                if length < LANE_MIN_M {
                    continue;
                }
                let score = d + if lead { 0.0 } else { 5.0 };
                if best.as_ref().is_none_or(|(s, _)| score < *s) {
                    best = Some((score, lane));
                }
            }
        }
        best.map(|(_, lane)| lane)
    }


    /// Is this point on the field's drawn pavement? `None` when the field
    /// draws none, so a caller never reads "no data" as "grass".
    pub fn on_pavement(&self, lat: f64, lon: f64) -> Option<bool> {
        if self.pavement.iter().all(|p| p.is_empty()) {
            return None;
        }
        Some(self.pavement.iter().any(|rings| {
            rings
                .first()
                .is_some_and(|outer| ring_contains(outer, lat, lon))
                && !rings[1..].iter().any(|hole| ring_contains(hole, lat, lon))
        }))
    }

    /// The nearest SUITABLE parking stand (excludes hangars) by straight-line
    /// distance — the destination an arrival taxi routes to. `None` when the
    /// field publishes no stands.
    pub fn nearest_stand(&self, lat: f64, lon: f64) -> Option<&Stand> {
        let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
        let d2 = |s: &Stand| {
            let dx = (s.lon - lon) * m_per_deg_lon;
            let dy = (s.lat - lat) * 111_320.0;
            dx * dx + dy * dy
        };
        self.stands
            .iter()
            .filter(|s| s.is_suitable())
            .min_by(|a, b| d2(a).total_cmp(&d2(b)))
    }

    /// [`plan`](Self::plan) to a STAND — the arrival mirror of
    /// [`plan_to_runway`](Self::plan_to_runway), and it exists for the same
    /// reason: the network's nearest node is not the destination.
    ///
    /// A departure's route ends short of the runway ON PURPOSE and the drawn
    /// lead-on carries it the rest of the way. An arrival's ended short by
    /// ACCIDENT, because apt.dat routes taxiways and mostly does not model
    /// the apron: the plain snap put the last point at whatever node lay
    /// nearest the stand and the drive stopped there, which at Malaga was
    /// 66 m short of stand W5 and parked across a live taxiway — twice, the
    /// tower having named the stand both times (2026-08-05).
    ///
    /// So the route is continued from that node to the stand itself, and the
    /// continuation rides the paint like everything else: the lead-in to a
    /// stand IS a drawn line at any field that paints one, and where none is
    /// drawn the straight leg is what the pavement is for. Bounded by
    /// [`STAND_LEAD_IN_MAX_M`] — a stand further out than that is not the one
    /// this route serves, and the old stop-at-the-node behaviour is better
    /// than inventing a drive across an unseen apron.
    pub fn plan_to_stand(&self, from_lat: f64, from_lon: f64, stand: &Stand) -> Option<TaxiRoutePlan> {
        let mut plan = self.plan(from_lat, from_lon, stand.lat, stand.lon, 0.0, &[])?;
        trim_snap_spur(&mut plan);
        let end = *plan.path.last()?;
        let m_per_deg_lon = 111_320.0 * stand.lat.to_radians().cos();
        let gap = {
            let (dx, dy) = (
                (stand.lon - end.1) * m_per_deg_lon,
                (stand.lat - end.0) * 111_320.0,
            );
            (dx * dx + dy * dy).sqrt()
        };
        if !(STAND_LEAD_IN_MIN_M..=STAND_LEAD_IN_MAX_M).contains(&gap) {
            return Some(plan);
        }
        // Sampled and snapped exactly like a planned leg, so a drawn lead-in
        // is followed where there is one. The leg carries no NAME: an apron
        // lead-in is not a taxiway, and the guide would otherwise invent a
        // turn "onto" something the pilot cannot see on the chart.
        //
        // And the lead-in ARRIVES on the stand's own heading: the stand is
        // entered along its parking line through a point [`STAND_ALIGN_M`]
        // back from it. Straight from the last taxiway node, EDLP 7C was
        // reached 2 m from its spot and 147 degrees across it, so the
        // securing check (nose within 60 degrees) said "stopped short of the
        // stand" (2026-09-24).
        let pts = stand_lead_in(end, stand, m_per_deg_lon);
        let segs = vec![(String::new(), false); pts.len() - 1];
        let (lead, lead_seg) = ride_paint(&self.paint, &pts, &segs);
        plan.path.extend(lead.into_iter().skip(1));
        plan.seg.extend(lead_seg);
        Some(plan)
    }

    /// Route from a position to a position (a stand to a runway threshold)
    /// and extract the phrasable plan. `None` when either end doesn't snap
    /// to the network (within 3 km) or no path exists.
    ///
    /// The tail is ended at the destination runway's HOLD-SHORT position,
    /// best data first: `hold_zone` names the runway ends whose authored
    /// active zone (apt.dat 1204 rows) the route must stop at the edge of —
    /// the per-stub truth when the airport carries it. Without zone data,
    /// `hold_short_m` is the geometric fallback: how far back from a
    /// runway-junction tail the route ends ([`HOLD_SHORT_STANDOFF_M`]
    /// unless the caller knows the runway's class says otherwise —
    /// instrument holds sit further out).
    pub fn plan(
        &self,
        from_lat: f64,
        from_lon: f64,
        to_lat: f64,
        to_lon: f64,
        hold_short_m: f64,
        hold_zone: &[String],
    ) -> Option<TaxiRoutePlan> {
        self.plan_inner(
            from_lat,
            from_lon,
            to_lat,
            to_lon,
            hold_short_m,
            hold_zone,
            false,
            None,
        )
    }

    /// [`plan`](Self::plan) to a RUNWAY — the destination is a threshold, and
    /// the route must end somewhere the aircraft can get onto the strip from.
    ///
    /// Split from `plan` rather than inferred from the geometry because the
    /// callers genuinely differ and only they know: the departure planner is
    /// handed a threshold, taxi-in is handed a stand, and the off-route replan
    /// is handed the standing route's already-trimmed holding point — which
    /// sits a hold-short standoff from the runway and must NOT be dragged back
    /// onto the junction the trim just removed.
    ///
    /// `strip` is the destination runway's own pavement, and it is what keeps
    /// the entrance search on the right strip at a field whose thresholds sit
    /// on top of each other (see [`Self::runway_entrance_near`]). `None`
    /// falls back to nearest-entrance-anywhere.
    #[allow(clippy::too_many_arguments)]
    pub fn plan_to_runway(
        &self,
        from_lat: f64,
        from_lon: f64,
        to_lat: f64,
        to_lon: f64,
        hold_short_m: f64,
        hold_zone: &[String],
        strip: Option<&Runway>,
    ) -> Option<TaxiRoutePlan> {
        self.plan_inner(
            from_lat,
            from_lon,
            to_lat,
            to_lon,
            hold_short_m,
            hold_zone,
            true,
            strip,
        )
    }

    /// Whether a departure from `ident` (an end of `strip`) can get onto
    /// the strip on its threshold's half. `true` when the network cannot
    /// say (no runway nodes of its own): thin data never vetoes a runway.
    /// LEMD 18R (2026-10-05): its only ways on are at the 36L end.
    pub fn departure_end_enterable(&self, strip: &Runway, ident: &str) -> bool {
        let (lat, lon) = if strip.ident_a == ident {
            (strip.lat_a, strip.lon_a)
        } else {
            (strip.lat_b, strip.lon_b)
        };
        let taxiway_nodes: std::collections::HashSet<i64> = self
            .edges
            .iter()
            .filter(|e| !e.runway)
            .flat_map(|e| [e.a, e.b])
            .collect();
        !matches!(
            self.runway_entrance_near(lat, lon, &taxiway_nodes, Some(strip)),
            Entrance::Backtrack(_)
        )
    }

    /// Plan the drive OFF the runway we have just landed on.
    ///
    /// The arrival mirror of `plan_to_runway`, and it reuses that work rather
    /// than inventing a second idea of where a runway meets a taxiway: an
    /// ENTRANCE and an EXIT are the same node, decided by the same
    /// connectivity relation (on a runway edge, or one hop from one) — only
    /// the direction of travel differs. And the zone walk below is the
    /// hold-short trim run backwards: the trim DROPS the trailing edges inside
    /// the runway's protected zone so a departure stops short of the line;
    /// this KEEPS going until it is `clear_m` past the same boundary, because
    /// the tail has to be over it too.
    ///
    /// Why this exists at all: the taxi-in used to transmit its request from
    /// wherever autoland stopped and wait up to sixty seconds for a route,
    /// sitting on the strip the whole time — and on the give-up path it handed
    /// back "your aircraft" still on the runway. Vacating needs no clearance
    /// from anybody. It is the completion of the landing clearance we already
    /// hold, so it happens first and the request comes from the taxiway, which
    /// is the order a real crew flies.
    ///
    /// `psi_true` is the rollout direction: exits BEHIND us are refused, since
    /// reaching one means taxiing back down the runway, which is worse than
    /// the longer exit ahead. `zone` names the runway ends whose active zone
    /// we want to be clear of.
    ///
    /// Never refuses for want of data — a field with no authored runway edges
    /// (776 ends worldwide) falls back to the nearest taxiway node ahead, and
    /// only a field with no network at all returns `None`.
    pub fn plan_vacate(
        &self,
        lat: f64,
        lon: f64,
        psi_true: f64,
        zone: &[String],
        clear_m: f64,
    ) -> Option<VacatePlan> {
        let m_per_deg_lat = 111_320.0;
        let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
        let dist_m = |a: (f64, f64), b: (f64, f64)| -> f64 {
            let dx = (a.1 - b.1) * m_per_deg_lon;
            let dy = (a.0 - b.0) * m_per_deg_lat;
            (dx * dx + dy * dy).sqrt()
        };
        let here = (lat, lon);
        // Ahead of abeam, measured from the rollout heading. We are stopped, so
        // the drive's own join owns the turn; this only rules out the exits
        // that would send us back down the strip.
        let ahead = |p: (f64, f64)| -> bool {
            let brg = bearing_deg(lat, lon, p.0, p.1);
            sim::angles::abs_diff_deg(brg, psi_true) <= AHEAD_OF_ABEAM_DEG
        };
        // Which side of the rollout a point lies on, metres, right-positive.
        // The rollout heading IS the runway direction to within a few degrees,
        // so this needs no strip geometry passed in.
        let side_of = |p: (f64, f64)| -> f64 {
            let dx = (p.1 - lon) * m_per_deg_lon;
            let dy = (p.0 - lat) * m_per_deg_lat;
            let (sin, cos) = psi_true.to_radians().sin_cos();
            dx * cos - dy * sin
        };
        // ...and which side the PARKING is on, because a vacate turns TOWARD
        // the apron. At Liege (2026-09-22) a 737 landed 04R and took C3, the
        // link to the parallel 04L: 35 m nearer than S3 and the only test was
        // distance. It ended 148 m out on the wrong side of 04R, inside 04L's
        // authored zone, with every stand and every other exit behind it - so
        // the taxi-in clearance came back "via Charlie three, Sierra three,
        // cross runway zero four right" and the drive was asked for a 180 on a
        // taxiway a 737 cannot turn on. The nearest suitable stand is the
        // proxy for the apron: at a field with parking on both sides that
        // picks the near one, which is the answer anyway. No stands in the
        // data leaves the old distance-only rule.
        let apron_side = self
            .nearest_stand(lat, lon)
            .map(|st| side_of((st.lat, st.lon)))
            .filter(|s| s.abs() > 1.0)
            .map(f64::signum);

        let mut taxiway_nodes: std::collections::HashSet<i64> = std::collections::HashSet::new();
        for e in self.edges.iter().filter(|e| !e.runway) {
            taxiway_nodes.insert(e.a);
            taxiway_nodes.insert(e.b);
        }
        if taxiway_nodes.is_empty() {
            return None;
        }
        // Only plan a vacate FROM the strip. The caller's gate is the standing
        // landing clearance, which holds from touchdown to the taxi-in
        // request — position is what says the vacate already happened. Where
        // runway edges are authored, an aircraft nowhere near one has nothing
        // to vacate, and the nearest-junction-ahead snap below would aim it
        // BACK at the runway entrance. Thin data (no runway edges) cannot
        // judge this and keeps the old behaviour.
        let seg_dist_m = |p: (f64, f64), a: (f64, f64), b: (f64, f64)| -> f64 {
            let (px, py) = ((p.1 - a.1) * m_per_deg_lon, (p.0 - a.0) * m_per_deg_lat);
            let (bx, by) = ((b.1 - a.1) * m_per_deg_lon, (b.0 - a.0) * m_per_deg_lat);
            let len2 = bx * bx + by * by;
            let t = if len2 > 0.0 {
                ((px * bx + py * by) / len2).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let (dx, dy) = (px - t * bx, py - t * by);
            (dx * dx + dy * dy).sqrt()
        };
        let mut has_runway_edges = false;
        let mut on_strip = false;
        for e in self.edges.iter().filter(|e| e.runway) {
            has_runway_edges = true;
            if let (Some(&a), Some(&b)) = (self.nodes.get(&e.a), self.nodes.get(&e.b)) {
                if seg_dist_m(here, a, b) <= VACATE_ON_RUNWAY_M {
                    on_strip = true;
                    break;
                }
            }
        }
        if has_runway_edges && !on_strip {
            return None;
        }
        // Candidate exits: runway nodes that a taxiway also touches — the
        // JUNCTION where the two meet.
        //
        // This is where an exit and an entrance stop being the same thing.
        // `runway_entrance_near` accepts a node one hop OFF the runway,
        // because a departure holds short there and the drawn lead-on carries
        // it the rest of the way. An arrival cannot use that node as its
        // decision point: measuring "is this exit ahead of me" at the far end
        // of a link says nothing about where the link MEETS the runway, and a
        // junction 22 m behind the aircraft reads as an exit 150 m off to the
        // side. Southbound on the test net that picked the turn-off we had
        // just rolled past. The junction is the exit; the walk below carries
        // us onto the taxiway from there.
        //
        // When the field authors no runway edges at all there is nothing to
        // filter on, so every taxiway node is a candidate and the nearest one
        // ahead wins — the same degradation `plan_to_runway` makes.
        let mut runway_nodes: std::collections::HashSet<i64> = std::collections::HashSet::new();
        for e in self.edges.iter().filter(|e| e.runway) {
            runway_nodes.insert(e.a);
            runway_nodes.insert(e.b);
        }
        // How a link is ranked against the zones it lies in. Shared by the
        // turn-off test and the walk so both read the pavement the same way.
        let zone_rank = |e: &TaxiEdge| -> u8 {
            if e.zone_of.is_empty() {
                0
            } else if e.zone_of.iter().all(|z| zone.contains(z)) {
                1
            } else {
                2
            }
        };
        // The turn-off a junction actually offers this rollout: the best
        // traversable taxiway leaving it that we can TURN ONTO, or `None`.
        //
        // A junction being ahead is not enough. A rapid-exit taxiway is
        // angled toward the traffic it serves, so the ones built for the
        // OTHER end of the strip leave the same junction pointing backwards.
        // EGLL's N4W meets 27L eighty-four metres ahead and runs back
        // east-north-east, 169 degrees off the landing direction: the vacate
        // took it, drove the 737 up the runway, turned it around ON the strip
        // and drove it back past its own touchdown point — 624 m of route and
        // three minutes of occupancy with traffic inside a mile (2026-09-17).
        // The turn-off that works is 220 m further on (N5W, 54 degrees off).
        // More distance down a runway we are already rolling along is cheap;
        // a reversal on it is not.
        let turn_off = |id: i64, p: (f64, f64)| -> Option<(i64, (f64, f64), String, u8)> {
            self.edges
                .iter()
                .filter(|e| !e.runway)
                .filter_map(|e| {
                    let nxt = if e.a == id {
                        e.b
                    } else if e.b == id && !e.oneway {
                        e.a
                    } else {
                        return None;
                    };
                    let np = *self.nodes.get(&nxt)?;
                    let brg = bearing_deg(p.0, p.1, np.0, np.1);
                    if sim::angles::abs_diff_deg(brg, psi_true) > VACATE_TURN_MAX_DEG {
                        return None;
                    }
                    // Never onto ANOTHER runway's protected pavement. Ranking
                    // it last was not enough: at Liege it was the only link C3
                    // offered past its midpoint, so the walk took it and
                    // stopped 62 m from runway 04L's centreline, inside its
                    // departure AND arrival zones. Vacating one runway is
                    // covered by the landing clearance we hold; entering
                    // another is not covered by anything.
                    if zone_rank(e) == 2 {
                        return None;
                    }
                    Some((nxt, np, e.name.clone(), zone_rank(e)))
                })
                .min_by(|x, y| {
                    x.3.cmp(&y.3)
                        .then_with(|| dist_m(here, y.1).total_cmp(&dist_m(here, x.1)))
                })
        };
        let (exit, first) = taxiway_nodes
            .iter()
            .filter(|id| runway_nodes.is_empty() || runway_nodes.contains(id))
            .filter_map(|id| self.nodes.get(id).map(|&p| (*id, p)))
            .filter(|(_, p)| ahead(*p))
            // Bounded like every other snap in this file: a junction
            // kilometres away is another airport's, not our exit.
            .filter(|(_, p)| dist_m(here, *p) <= MAX_EXIT_SNAP_M)
            // Only a junction we can leave. A field whose every turn-off is
            // authored backwards yields no plan at all, and the caller falls
            // through to the taxi-in from the runway — the same degradation
            // the dead-end case makes, and better than the U-turn.
            .filter_map(|(id, p)| turn_off(id, p).map(|s| (id, p, s)))
            .min_by(|x, y| {
                // Toward the apron first, then nearest. A turn-off that leads
                // away from the parking is a longer taxi at best and, where
                // the pavement between two parallels is the thing it leads
                // onto, a drive that cannot be completed at all.
                type TurnOffCandidate = (i64, (f64, f64), (i64, (f64, f64), String, u8));
                let toward = |c: &TurnOffCandidate| -> bool {
                    apron_side.is_none_or(|a| side_of(c.2 .1) * a >= 0.0)
                };
                toward(y)
                    .cmp(&toward(x))
                    .then_with(|| dist_m(here, x.1).total_cmp(&dist_m(here, y.1)))
            })
            .map(|(id, _, step)| (id, step))?;

        // Walk outward until `clear_m` past the exit, never along a runway
        // edge and never back through a node we have used. "Outward" is
        // measured from the AIRCRAFT, which is on the runway — so maximising
        // it is literally "get away from the runway", which is the whole job.
        let mut path_ids = vec![exit];
        let mut seen: std::collections::HashSet<i64> = std::collections::HashSet::new();
        seen.insert(exit);
        let mut names: Vec<String> = Vec::new();
        // Distance travelled OUTSIDE the zone. Metres covered while still
        // inside it earn no credit, which is the whole point: the link from
        // the runway onto the parallel taxiway is itself a zone edge, so
        // counting its length would finish the drive at the far end of it —
        // and by the hold-short trim's own logic that far node IS the holding
        // point, i.e. exactly the line we are trying to get the tail over.
        let mut clear_gone = 0.0;
        // A field whose zones swallow the apron would otherwise walk us to the
        // terminal; the vacate is the first hundred metres, not the taxi in.
        let mut left_zone = zone.is_empty();
        // The first step is the turn-off this exit was CHOSEN for, not
        // whatever lies furthest from the aircraft: at a junction offering
        // both, "furthest" is the link angled back down the strip.
        {
            let (nxt, p, name, rank) = first;
            let cur_p = *self.nodes.get(&exit)?;
            if rank == 0 {
                left_zone = true;
            }
            if left_zone {
                clear_gone += dist_m(cur_p, p);
            }
            path_ids.push(nxt);
            seen.insert(nxt);
            names.push(name);
        }
        while clear_gone < clear_m && names.len() < MAX_VACATE_STEPS {
            let cur = *path_ids.last()?;
            let cur_p = *self.nodes.get(&cur)?;
            let step = self
                .edges
                .iter()
                .filter(|e| !e.runway)
                .filter_map(|e| {
                    let nxt = if e.a == cur {
                        e.b
                    } else if e.b == cur && !e.oneway {
                        e.a
                    } else {
                        return None;
                    };
                    if seen.contains(&nxt) {
                        return None;
                    }
                    // Leaving protected pavement is the point: rank clean
                    // edges first, then our own runway's zone, then — last of
                    // all — an edge inside ANOTHER runway's zone. `zone` only
                    // carries the runway we landed on, so a plain
                    // "in-our-zone" test read a link crossing an intersecting
                    // runway as a way OUT and actively preferred driving
                    // across the other strip, with the metres counted as
                    // clearance (audit, 2026-08-02).
                    // ...and the walk refuses it too, for the same reason.
                    if zone_rank(e) == 2 {
                        return None;
                    }
                    self.nodes
                        .get(&nxt)
                        .map(|&p| (nxt, p, e.name.clone(), zone_rank(e)))
                })
                .min_by(|x, y| {
                    // Cleanest pavement first, then whichever gets us furthest
                    // from the runway we are standing on.
                    x.3.cmp(&y.3)
                        .then_with(|| dist_m(here, y.1).total_cmp(&dist_m(here, x.1)))
                });
            let Some((nxt, p, name, zone_rank)) = step else {
                break;
            };
            if zone_rank == 0 {
                left_zone = true;
            }
            if left_zone {
                clear_gone += dist_m(cur_p, p);
            }
            path_ids.push(nxt);
            seen.insert(nxt);
            names.push(name);
        }

        // A walk that never moved found no traversable taxiway edge at the
        // junction — a one-way stub authored toward the runway is enough. The
        // "plan" would be a single leg from the aircraft to a node ON the
        // centreline: a drive down the runway reported as a vacate, with
        // "runway vacated" already owed from the strip. No plan is strictly
        // better — the caller falls through to the taxi-in from the runway,
        // exactly as it behaved before the vacate existed (audit, 2026-08-02,
        // demonstrated by the dead-end test below).
        if names.is_empty() {
            return None;
        }
        // The path starts AT THE AIRCRAFT — the same rule the stand route
        // learned the hard way, because a drive whose route begins 200 m away
        // reads as already-arrived on its first tick.
        let mut path = vec![here];
        for id in &path_ids {
            path.push(*self.nodes.get(id)?);
        }
        // The first leg is the one off the runway itself; it has no authored
        // name, and calling it by the runway's would be a lie.
        let mut seg = vec![(String::new(), true)];
        for n in &names {
            seg.push((n.clone(), false));
        }
        seg.truncate(path.len().saturating_sub(1));
        // Where we END UP, not the link we turned onto. The walk usually
        // crosses a short stub inside the zone before reaching the parallel
        // taxiway, and it is the latter the crew reports being clear on and
        // the taxi-in plans from.
        let exit_name = names
            .iter()
            .rev()
            .find(|n| !n.is_empty())
            .cloned()
            .unwrap_or_default();
        // Onto the paint, like the departure route. The first leg is flagged
        // as the runway one, so the snap leaves it — and with it the start
        // point, which is the aircraft's own position and the one point in
        // any route that must never be moved.
        let (path, seg) = ride_paint(&self.paint, &path, &seg);
        (path.len() >= 2).then_some(VacatePlan {
            path,
            seg,
            exit_name,
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn plan_inner(
        &self,
        from_lat: f64,
        from_lon: f64,
        to_lat: f64,
        to_lon: f64,
        hold_short_m: f64,
        hold_zone: &[String],
        to_runway: bool,
        strip: Option<&Runway>,
    ) -> Option<TaxiRoutePlan> {
        let m_per_deg_lat = 111_320.0;
        let m_per_deg_lon = 111_320.0 * from_lat.to_radians().cos();
        let dist_m = |a: (f64, f64), b: (f64, f64)| -> f64 {
            let dx = (a.1 - b.1) * m_per_deg_lon;
            let dy = (a.0 - b.0) * m_per_deg_lat;
            (dx * dx + dy * dy).sqrt()
        };
        // Snap both ends to the nearest node that touches a TAXIWAY edge —
        // never a runway-only node (the aircraft taxis on taxiways).
        let mut taxiway_nodes: std::collections::HashSet<i64> = std::collections::HashSet::new();
        for e in self.edges.iter().filter(|e| !e.runway) {
            taxiway_nodes.insert(e.a);
            taxiway_nodes.insert(e.b);
        }
        let snap_within = |lat: f64, lon: f64, from: &std::collections::HashSet<i64>| {
            from.iter()
                .filter_map(|id| self.nodes.get(id).map(|&p| (*id, dist_m((lat, lon), p))))
                .min_by(|x, y| x.1.total_cmp(&y.1))
                .filter(|(_, d)| *d < 3000.0)
                .map(|(id, _)| id)
        };
        let start = snap_within(from_lat, from_lon, &taxiway_nodes)?;
        // A runway destination must snap to a node that can actually REACH the
        // runway, not merely to the taxiway node that happens to lie nearest
        // the threshold. Nearest-anything is how EGHI's runway 20 became a
        // clearance to nowhere (2026-07-30): the north end of the parallel
        // taxiway A5 is a dead-end stub 172 m abeam the strip, and being the
        // closest taxiway node to the threshold (631 m, against 1047 m for the
        // nearest real entrance) it won the snap. The tower cleared us to
        // "holding point Alpha five runway two zero", the drive stopped at a
        // stub connected to nothing, and because the goal touched no runway
        // edge the hold-short trim never ran and `lead_on` came out empty —
        // leaving the line-up to invent an entrance, which is the trap the
        // drawn lead-on exists to close. Across the whole apt.dat this is a
        // class, not a field: 3,498 of 21,656 runway ends snap to a
        // non-entrance, 551 of them badly (EFHK 04R, ENBO 07, ESNN 34).
        let goal = match to_runway
            .then(|| self.runway_entrance_near(to_lat, to_lon, &taxiway_nodes, strip))
        {
            Some(Entrance::Node(n) | Entrance::Backtrack(n)) => Some(n),
            _ => None,
        }
        .or_else(|| snap_within(to_lat, to_lon, &taxiway_nodes))?;
        if start == goal {
            // For a RUNWAY destination this is genuinely nothing to do: we are
            // already at the entrance the clearance would send us to.
            if to_runway {
                return None;
            }
            // For a STAND it is not. A sparse network puts both ends on one
            // node whenever the parking is close, and returning None there
            // means the tower answers "taxi to parking, this frequency" with
            // no route attached — the crew's taxi-in then waits out its whole
            // 60 s for a route that was never coming and gives up with "no
            // taxi-in clearance came back", which is a lie: the clearance
            // came, the ROUTE did not (EGHI 2026-07-30, aircraft 224 m from
            // node 44 and Stand 14 another 69 m past it, on a 24-edge
            // network).
            //
            // The honest answer is the short taxi it actually is: out to the
            // node, then in to the stand. No named legs, because none were
            // traversed.
            // The path must START AT THE AIRCRAFT. A normal route begins at
            // its snapped node and the drive JOINS to it, which is fine when
            // the aircraft is beside that node — but here the whole point is
            // that one node serves both ends, and the aircraft can be a long
            // way from it. First cut began at the node: at EGHI that put the
            // route's start 224 m from an aircraft that was stopped, so the
            // drive's arrival test ("within a metre and under 1.5 kt") was
            // true on the first tick and taxi_in went from "driving" to
            // "secured" in two seconds without moving.
            let node = *self.nodes.get(&start)?;
            let here = (from_lat, from_lon);
            let dest = (to_lat, to_lon);
            // Route VIA the node when it is roughly on the way — it sits on
            // the taxiway, which is where we would rather be — but not when
            // doubling back to it would be the long way round.
            let via_node =
                dist_m(here, node) + dist_m(node, dest) <= 1.6 * dist_m(here, dest).max(1.0);
            let path = if via_node {
                vec![here, node, dest]
            } else {
                vec![here, dest]
            };
            let seg = vec![(String::new(), false); path.len() - 1];
            return Some(TaxiRoutePlan {
                via: Vec::new(),
                crossings: Vec::new(),
                holding_point: String::new(),
                seg,
                path,
                lead_on: Vec::new(),
            });
        }
        // Dijkstra; runway edges cost ×25 so a route only touches one to
        // CROSS it, never to taxi along it.
        let mut adj: std::collections::HashMap<i64, Vec<usize>> = std::collections::HashMap::new();
        for (i, e) in self.edges.iter().enumerate() {
            adj.entry(e.a).or_default().push(i);
            if !e.oneway {
                adj.entry(e.b).or_default().push(i);
            }
        }
        use std::cmp::Reverse;
        use std::collections::BinaryHeap;
        let mut best: std::collections::HashMap<i64, (f64, Option<usize>)> =
            std::collections::HashMap::new();
        let mut heap: BinaryHeap<(Reverse<u64>, i64)> = BinaryHeap::new();
        best.insert(start, (0.0, None));
        heap.push((Reverse(0), start));
        while let Some((Reverse(cost_mm), node)) = heap.pop() {
            let cost = cost_mm as f64 / 1000.0;
            if node == goal {
                break;
            }
            if best.get(&node).is_none_or(|(c, _)| cost > *c + 0.5) {
                continue; // stale heap entry
            }
            for &ei in adj.get(&node).map(|v| v.as_slice()).unwrap_or(&[]) {
                let e = &self.edges[ei];
                let next = if e.a == node { e.b } else { e.a };
                let (Some(&pa), Some(&pb)) = (self.nodes.get(&e.a), self.nodes.get(&e.b)) else {
                    continue;
                };
                let mut w = dist_m(pa, pb);
                if e.runway {
                    w *= 25.0;
                }
                let nc = cost + w;
                if best.get(&next).is_none_or(|(c, _)| nc < *c) {
                    best.insert(next, (nc, Some(ei)));
                    heap.push((Reverse((nc * 1000.0) as u64), next));
                }
            }
        }
        // Walk back goal→start collecting edges, then flip.
        let mut edges_rev: Vec<usize> = Vec::new();
        let mut node = goal;
        let mut hops = 0;
        while node != start {
            let (_, Some(ei)) = *best.get(&node)? else {
                return None;
            };
            edges_rev.push(ei);
            let e = &self.edges[ei];
            node = if e.a == node { e.b } else { e.a };
            hops += 1;
            if hops > 10_000 {
                return None; // cycle guard
            }
        }
        edges_rev.reverse();
        let mut via: Vec<String> = Vec::new();
        let mut crossings: Vec<String> = Vec::new();
        let mut holding_point = String::new();
        let mut path: Vec<(f64, f64)> = Vec::new();
        let mut seg: Vec<(String, bool)> = Vec::new();
        let mut walk = start;
        path.extend(self.nodes.get(&walk).copied());
        for &ei in &edges_rev {
            let e = &self.edges[ei];
            walk = if e.a == walk { e.b } else { e.a };
            // Keep `seg` aligned with `path` segments: a seg entry only when
            // this edge actually appends a point after an existing one.
            if let Some(&p) = self.nodes.get(&walk) {
                if !path.is_empty() {
                    seg.push((e.name.clone(), e.runway));
                }
                path.push(p);
            }
            if e.name.is_empty() {
                continue;
            }
            if e.runway {
                if !crossings.contains(&e.name) {
                    crossings.push(e.name.clone());
                }
            } else {
                if via.last() != Some(&e.name) {
                    via.push(e.name.clone());
                }
                holding_point = e.name.clone();
            }
        }
        via.dedup();
        via.truncate(6);
        // End the route at the destination's hold-short position — the goal
        // node is usually the taxiway/runway JUNCTION (apt.dat networks
        // join taxiways to the runway centreline), and a route ending there
        // put the auto-taxi stop (and the guide's "holding point ahead,
        // that's us") across the hold line (2026-07-20).
        //
        // Best data first: when the airport authors ACTIVE ZONES (1204
        // rows) for the destination runway, drop the trailing edges inside
        // its zone — the boundary node IS the authored holding point,
        // per stub (EGLL carries these for every entry; the geometric 100 m
        // guess was still wrong stub-to-stub).
        // The untrimmed path, kept so the trimmed tail — the authored lead-on
        // from the holding point onto the runway centreline — survives on the
        // plan for the line-up to drive.
        let full = path.clone();
        // The aircraft's own position is put in front of the route below
        // when it is off the first node: then the trims may cut the network
        // part down to that one node and the route still has two points.
        // UEMH (2026-10-08, the traffic sweep): stand 13 snaps to the node
        // ONE edge from the runway, the two-node route could not be trimmed
        // by either rule and ended on the centreline junction, so the taxi
        // clearance drove onto runway zero niner.
        let fronted = path
            .first()
            .is_some_and(|f| dist_m((from_lat, from_lon), *f) > 1.0);
        let keep_nodes = if fronted { 1 } else { 2 };
        let zone_edges = if hold_zone.is_empty() {
            0
        } else {
            edges_rev
                .iter()
                .rev()
                .take_while(|&&ei| self.edges[ei].zone_of.iter().any(|z| hold_zone.contains(z)))
                .count()
                .min(path.len().saturating_sub(keep_nodes))
        };
        if zone_edges > 0 {
            path.truncate(path.len() - zone_edges);
            seg.truncate(seg.len() - zone_edges);
        }
        // Geometric fallback: when the last node touches a runway edge,
        // pull the tail back by a hold-short standoff so the route ends
        // where an aircraft should STOP. A network whose tail ends at a
        // genuine holding-point node off the runway keeps its geometry.
        // The distance is the caller's to judge (instrument runways paint
        // their lines much further out than GA fields) — an approximation
        // either way, which is why authored zones win above.
        let ends_on_runway = self
            .edges
            .iter()
            .any(|e| e.runway && (e.a == goal || e.b == goal));
        let total: f64 = path.windows(2).map(|w| dist_m(w[0], w[1])).sum();
        if zone_edges == 0 && ends_on_runway && total > hold_short_m + 30.0 {
            let mut cut = 0.0;
            while path.len() >= 3 {
                let n = path.len();
                let leg = dist_m(path[n - 2], path[n - 1]);
                if cut + leg > hold_short_m {
                    // The stop point lands inside this leg: move the end
                    // node back along it (or drop it when only a sliver
                    // would remain).
                    let keep = 1.0 - (hold_short_m - cut) / leg;
                    if keep > 0.05 {
                        let (a, b) = (path[n - 2], path[n - 1]);
                        path[n - 1] = (a.0 + (b.0 - a.0) * keep, a.1 + (b.1 - a.1) * keep);
                    } else {
                        path.pop();
                        seg.pop();
                    }
                    break;
                }
                cut += leg;
                path.pop();
                seg.pop();
            }
        }
        // The lead-on: whatever the trims removed, from the (possibly moved)
        // hold point out to the runway-junction node. It STARTS at the
        // trimmed route's own end, always: the zone trim and the sliver-pop
        // truncate at a node, which leaves the trimmed path a strict prefix
        // of the full one — the old longest-common-prefix arithmetic then
        // skipped the holding point itself, and a one-edge zone tail (the
        // per-stub authoring EGLL uses) degenerated the lead-on to a single
        // point. Then the original tail follows from the first point the
        // trims touched. Empty when nothing was trimmed.
        let trimmed = full.len() != path.len() || full.last() != path.last();
        let mut lead_on: Vec<(f64, f64)> = Vec::new();
        if trimmed {
            lead_on.extend(path.last().copied());
            let prefix = full
                .iter()
                .zip(path.iter())
                .take_while(|(a, b)| a == b)
                .count();
            lead_on.extend_from_slice(&full[prefix..]);
        }
        // Onto the paint, now the route is trimmed and the lead-on split off:
        // the network chose the taxiways, the drawn line says where they run.
        // Both halves ride it — the lead-on IS a painted line (the yellow one
        // from the holding point onto the strip), and the line-up drives it.
        (path, seg) = ride_paint(&self.paint, &path, &seg);
        let lead_seg = vec![(String::new(), false); lead_on.len().saturating_sub(1)];
        lead_on = ride_paint(&self.paint, &lead_on, &lead_seg).0;
        // ...and the route JOINS its first taxiway ahead of the aircraft,
        // never at a snapped node behind it (`join_first_edge_ahead`).
        join_first_edge_ahead(&mut path, &mut seg, (from_lat, from_lon));
        // The route starts where the AIRCRAFT is, not at the node it snapped
        // to: the apron leg from the stand to the network is part of the
        // cleared route, so the off-route watch and the taxi guide measure
        // from brake release (a Malaga stand exit was berated as off-route
        // 2 s after rolling, 100 m from a route it had no way to be on yet,
        // 2026-07-13).
        if let Some(&first) = path.first() {
            if dist_m((from_lat, from_lon), first) > 1.0 {
                path.insert(0, (from_lat, from_lon));
                seg.insert(0, (String::new(), false));
            }
        }
        Some(TaxiRoutePlan {
            via,
            crossings,
            holding_point,
            path,
            seg,
            lead_on,
        })
    }

    /// The taxiway node nearest `(lat, lon)` that can actually get onto a
    /// runway from there. `None` when the field's network models no entrance at
    /// all, which leaves [`plan_to_runway`](Self::plan_to_runway) on the plain
    /// nearest-node snap rather than refusing a route the old code would fly.
    ///
    /// "Entrance" is decided by CONNECTIVITY, never by edge names: apt.dat
    /// authors leave runway edges unnamed often enough that
    /// the station's `mark_runway_crossings` already refuses to trust the
    /// flags for the same reason. A node qualifies if it sits on a runway edge,
    /// or one hop from a node that does — which is exactly how a network draws
    /// the stub from a holding point out to the centreline.
    ///
    /// ...and it must be an entrance to THIS strip, decided by geometry like
    /// every other "is that pavement ours" question here: a runway node counts
    /// only when it lies on `strip`'s own rectangle. Nearest-entrance-anywhere
    /// was the rule, on the reasoning that a parallel field's other strip sits
    /// a runway separation away and loses on distance. Le Bourget breaks it
    /// end-on: 07 and 09 start 18 m apart and diverge, so 09's entrance (node
    /// 201, 43 m from 07's threshold) beat 07's own (node 134, 67 m). The
    /// tower cleared "holding point Alpha runway zero seven", the route was
    /// planned to 09's hold, the active-zone trim cut its last three edges,
    /// and the drive stopped 169 m from runway 07's centreline — 19 m outside
    /// the crew's `LINE_UP_MAX_LATERAL_M` as the line-up sees it. The
    /// lead-on ended 34 m off the 07 centreline too, so the authored entry
    /// was refused as well, and full auto said "I couldn't get us onto the
    /// runway" with the aircraft correctly parked where it had been sent
    /// (2026-09-22). Across the whole apt.dat this is a class like the dead-end
    /// snap above: 5,228 of 75,812 runway ends start within 300 m of a
    /// different strip's end on a different course, at 1,815 fields.
    /// `None` for the strip, or a strip no runway node sits on, keeps the old
    /// nearest-anywhere rule — thin data degrades, never refuses.
    fn runway_entrance_near(
        &self,
        lat: f64,
        lon: f64,
        taxiway_nodes: &std::collections::HashSet<i64>,
        strip: Option<&Runway>,
    ) -> Entrance {
        let m_per_deg_lat = 111_320.0;
        let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
        let mut runway_nodes: std::collections::HashSet<i64> = std::collections::HashSet::new();
        for e in self.edges.iter().filter(|e| e.runway) {
            runway_nodes.insert(e.a);
            runway_nodes.insert(e.b);
        }
        if runway_nodes.is_empty() {
            return Entrance::Unknown;
        }
        // How far from the threshold an entrance may be. With the strip
        // known, anywhere along it: its own entrance, however far up, beats
        // a neighbour's beside the numbers (KEDW 33, #36 sweep: 15/33 is
        // 8.9 km of lakebed with no entrance within 3 km of the end, and the
        // plain snap sent the route to 07C/25C).
        let mut reach_m: f64 = 3000.0;
        let mut departure_half: Option<std::collections::HashSet<i64>> = None;
        // Ours, by geometry. The margin is the pavement's own half-width plus
        // a shoulder for an author whose node sits off the painted edge; the
        // along-track window is the strip's length plus the same, so a node
        // just off the numbers still counts.
        if let Some(r) = strip {
            let half = (r.width_m / 2.0).max(15.0) + STRIP_NODE_SHOULDER_M;
            let course = r.heading_a_deg();
            let length = haversine_nm(r.lat_a, r.lon_a, r.lat_b, r.lon_b) * 1852.0;
            let on_strip = |(plat, plon): (f64, f64)| {
                let d = haversine_nm(r.lat_a, r.lon_a, plat, plon) * 1852.0;
                let brg = bearing_deg(r.lat_a, r.lon_a, plat, plon);
                let ang = sim::angles::signed_diff_deg(brg, course).to_radians();
                let along = d * ang.cos();
                (d * ang.sin()).abs() <= half
                    && along >= -STRIP_NODE_SHOULDER_M
                    && along <= length + STRIP_NODE_SHOULDER_M
            };
            // A node is ours when a runway edge ALONG the strip holds it:
            // the edge runs the strip's course and its middle is on the
            // rectangle. A node alone inside the shoulder is not enough:
            // KEND 31 (#36 sweep) ends at node 54, the last node of
            // 17L/35R's edge, 24 m off 13/31's centreline and so "on" it,
            // and the route ended there; a crossing runway's edge can lie
            // wholly inside our rectangle where it crosses (CYXX 01: 07/25's
            // edge 167-168 over 01's numbers).
            let ours: std::collections::HashSet<i64> = self
                .edges
                .iter()
                .filter(|e| e.runway)
                .filter(|e| {
                    let (Some(&pa), Some(&pb)) = (self.nodes.get(&e.a), self.nodes.get(&e.b))
                    else {
                        return false;
                    };
                    let off = sim::angles::abs_diff_deg(bearing_deg(pa.0, pa.1, pb.0, pb.1), course);
                    off.min(180.0 - off) <= STRIP_EDGE_ALIGN_DEG
                        && on_strip(((pa.0 + pb.0) / 2.0, (pa.1 + pb.1) / 2.0))
                })
                .flat_map(|e| [e.a, e.b])
                .collect();
            if !ours.is_empty() {
                // The threshold's half: a node no further along the strip
                // from the threshold than its midpoint.
                let along = |p: (f64, f64)| {
                    let d = haversine_nm(r.lat_a, r.lon_a, p.0, p.1) * 1852.0;
                    let brg = bearing_deg(r.lat_a, r.lon_a, p.0, p.1);
                    d * sim::angles::signed_diff_deg(brg, course).to_radians().cos()
                };
                let thr = along((lat, lon));
                departure_half = Some(
                    ours.iter()
                        .copied()
                        .filter(|id| {
                            self.nodes
                                .get(id)
                                .is_some_and(|&p| (along(p) - thr).abs() <= length / 2.0)
                        })
                        .collect(),
                );
                runway_nodes = ours;
                reach_m = reach_m.max(length + STRIP_NODE_SHOULDER_M);
            }
        }
        // One hop over a TAXIWAY edge: the stub. A hop over a RUNWAY edge
        // from our strip lands on runway pavement, and once `runway_nodes` is
        // our strip's own that is the NEIGHBOUR's: KGJT 29 (issue #36, the
        // real apt.dat sweep `real_install_converging_thresholds`) ends
        // where 04/22's edge 38-66 runs onto it, node 38 on 04/22's
        // centreline won as "one hop from runway 29", the route was planned
        // to 04/22's entrance and its lead-on ended 122 m off 11/29.
        let dist_to = |id: &i64| {
            self.nodes.get(id).map(|&p| {
                let dx = (p.1 - lon) * m_per_deg_lon;
                let dy = (p.0 - lat) * m_per_deg_lat;
                dx.hypot(dy)
            })
        };
        // The entrance among `entry` (runway nodes) nearest the threshold.
        // Only a way ON counts: a oneway taxiway edge leading off the strip
        // is an exit, and a node reached only by one is reached only along
        // the runway. LEMD 18R (2026-10-05): every stub at the 18R end is
        // oneway outbound, the threshold node won, the route rode 3.7 km of
        // runway from Z6 and the zone trim left the hold at the 36L end.
        let pick = |entry: &std::collections::HashSet<i64>| -> Option<i64> {
            let mut onto: std::collections::HashSet<i64> = std::collections::HashSet::new();
            let mut adjacent: std::collections::HashSet<i64> = std::collections::HashSet::new();
            for e in self.edges.iter().filter(|e| !e.runway) {
                if entry.contains(&e.b) {
                    onto.insert(e.b);
                    adjacent.insert(e.a);
                }
                if entry.contains(&e.a) && !e.oneway {
                    onto.insert(e.a);
                    adjacent.insert(e.b);
                }
            }
            let entrance = taxiway_nodes
                .iter()
                .filter(|id| onto.contains(id) || adjacent.contains(id))
                .filter_map(|id| dist_to(id).map(|d| (*id, d)))
                .min_by(|x, y| x.1.total_cmp(&y.1))
                .filter(|(_, d)| *d < reach_m)
                .map(|(id, _)| id)?;
            // A node one hop OFF the strip is where the stub starts, not
            // where it meets the runway: the route goes on to the runway node
            // it joins, so the hold-short trim finds the runway and the
            // lead-on it leaves behind reaches the centreline. Ending at the
            // hop, the lead-on stopped where the stub began. LIBP 04
            // (2026-09-26): taxiway D's first node beside the threshold (68 m
            // off the centreline, 92 m from the numbers) beat the node where
            // D meets the runway (on the centreline, 101 m up it), the
            // lead-on ran from the hold to 68 m off, the line-up refused it
            // as `junction_off_the_runway`, and two 737s drove the synthetic
            // 135-degree corner instead, 6.7 and 7.5 m past the centreline.
            if runway_nodes.contains(&entrance) {
                return Some(entrance);
            }
            let joins = self
                .edges
                .iter()
                .filter(|e| !e.runway)
                .filter_map(|e| {
                    if e.a == entrance {
                        Some(e.b)
                    } else if e.b == entrance && !e.oneway {
                        Some(e.a)
                    } else {
                        None
                    }
                })
                .filter(|n| entry.contains(n))
                .filter_map(|n| dist_to(&n).map(|d| (n, d)))
                .min_by(|x, y| x.1.total_cmp(&y.1))
                .map(|(n, _)| n);
            Some(joins.unwrap_or(entrance))
        };
        // A departure enters on its threshold's half of the strip; with no
        // way on there, the nearest way on anywhere is a BACKTRACK from it,
        // never a ride along the runway to the numbers.
        match departure_half {
            Some(half) => match pick(&half) {
                Some(n) => Entrance::Node(n),
                None => pick(&runway_nodes).map_or(Entrance::Unknown, Entrance::Backtrack),
            },
            None => pick(&runway_nodes).map_or(Entrance::Unknown, Entrance::Node),
        }
    }
}

/// What [`TaxiNet::runway_entrance_near`] found for a runway destination.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Entrance {
    /// A way on at the threshold's half of the strip (or anywhere, with the
    /// strip unknown).
    Node(i64),
    /// The only way on is on the far half: the departure backtracks from it.
    Backtrack(i64),
    /// No way on modelled: the plain nearest-node snap.
    Unknown,
}

/// How far off a strip's pavement a 1201 node may sit and still be read as a
/// node ON that strip. Authors place the centreline nodes by hand, and the
/// runway rectangle is the only geometry we have to judge them against.
pub(super) const STRIP_NODE_SHOULDER_M: f64 = 15.0;

/// How far off the strip's course a runway edge may run and still be one of
/// the strip's own (authors' centreline nodes wander a degree or two; a
/// crossing runway is tens of degrees off).
pub(super) const STRIP_EDGE_ALIGN_DEG: f64 = 15.0;

/// How far into a route a snap spur is looked for, points.
const SPUR_SCAN_PTS: usize = 30;

/// The route back out of its first point counts as a spur only when it
/// comes back this much nearer the start than the snap took it, metres.
const SPUR_MARGIN_M: f64 = 5.0;

/// A route that opens by snapping BACK to a node behind the aircraft and
/// retracing the line it is already on: cut from the start straight to the
/// point where the route passes nearest it. UUYY 18 (world sweep,
/// 2026-10-09): a vacate stopped clear of the runway end behind a
/// departure, and its stand route snapped 40 m back onto the runway corner
/// and came straight out again, a phantom "hold short runway one eight"
/// the arrival waited at, ON the runway, for six minutes.
fn trim_snap_spur(plan: &mut TaxiRoutePlan) {
    if plan.path.len() < 3 {
        return;
    }
    let start = plan.path[0];
    let m_per_deg_lon = 111_320.0 * start.0.to_radians().cos();
    let d = |p: (f64, f64)| {
        let (dx, dy) = ((p.1 - start.1) * m_per_deg_lon, (p.0 - start.0) * 111_320.0);
        (dx * dx + dy * dy).sqrt()
    };
    let first = d(plan.path[1]);
    let Some((m, dm)) = plan
        .path
        .iter()
        .enumerate()
        .skip(2)
        .take(SPUR_SCAN_PTS)
        .map(|(i, p)| (i, d(*p)))
        .min_by(|a, b| a.1.total_cmp(&b.1))
    else {
        return;
    };
    if dm + SPUR_MARGIN_M >= first {
        return;
    }
    let lead = plan.seg.get(m - 1).cloned().unwrap_or_default();
    let mut path = vec![start];
    path.extend_from_slice(&plan.path[m..]);
    let mut seg = vec![lead];
    seg.extend_from_slice(plan.seg.get(m..).unwrap_or(&[]));
    plan.path = path;
    plan.seg = seg;
}

#[cfg(test)]
mod spur_tests {
    use super::*;

    /// UUYY 18 (2026-10-09): a route that snaps 40 m back and retraces the
    /// line it starts on is cut from the start to where it passes nearest;
    /// a route that simply leads away is left alone.
    #[test]
    fn a_snap_spur_back_onto_the_runway_corner_is_cut() {
        let m = 1.0 / 111_320.0;
        let start = (60.0, 50.0);
        let back = (60.0 - 40.0 * m, 50.0);
        let mut plan = TaxiRoutePlan {
            path: vec![start, back, (60.0 - 20.0 * m, 50.0), (60.0 + 2.0 * m, 50.0), (60.0 + 50.0 * m, 50.0)],
            seg: vec![("RWY".into(), true), ("RWY".into(), true), ("A".into(), false), ("A".into(), false)],
            ..Default::default()
        };
        trim_snap_spur(&mut plan);
        assert_eq!(plan.path, vec![start, (60.0 + 2.0 * m, 50.0), (60.0 + 50.0 * m, 50.0)]);
        assert_eq!(plan.seg.len(), plan.path.len() - 1);
        assert_eq!(plan.seg[0].0, "A");
        let away = vec![start, (60.0 + 10.0 * m, 50.0), (60.0 + 30.0 * m, 50.0)];
        let mut plain = TaxiRoutePlan { path: away.clone(), seg: vec![Default::default(); 2], ..Default::default() };
        trim_snap_spur(&mut plain);
        assert_eq!(plain.path, away);
    }
}
