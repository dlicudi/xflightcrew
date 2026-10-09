//! One field as traffic sees it: nav's airport (runways, frequencies,
//! circuits, pattern sides) and its taxi network (stands, routes,
//! vacates), with the runway-end geometry the phase machine flies.

use crate::kin::{bearing, dist_m, project_m, LatLon};
use nav::{AirportInfo, Runway, Stand, TaxiNet, TaxiRoutePlan, VacatePlan};

/// The field: what apt.dat says, loaded once per installed station.
#[derive(Debug, Clone)]
pub struct FieldModel {
    pub icao: String,
    pub airport: AirportInfo,
    /// Empty (default) where the field has no network: nothing taxis.
    pub net: TaxiNet,
    /// The field's published procedures by name (empty without CIFP).
    pub procs: Procedures,
    /// Stands nothing taxis to or from, only parks on: the way between the
    /// stand and the network runs through another stand
    /// ([`FieldModel::joins_through_a_stand`]).
    pub parked_only: std::collections::HashSet<String>,
}

/// The terminal procedures a field publishes, by ident: what the
/// schedule needs to know IFR traffic can come and go.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Procedures {
    pub sids: Vec<String>,
    pub stars: Vec<String>,
    pub approaches: Vec<String>,
}

impl Procedures {
    /// Read from the CIFP folder (`<cifp_dir>/<ICAO>.dat`); empty when the
    /// field has none.
    pub fn load(cifp_dir: &str, icao: &str) -> Procedures {
        Procedures {
            sids: nav::list_procedures(cifp_dir, icao, nav::ProcKind::Sid),
            stars: nav::list_procedures(cifp_dir, icao, nav::ProcKind::Star),
            approaches: nav::list_procedures(cifp_dir, icao, nav::ProcKind::Appch),
        }
    }

    /// An approach that serves runway `ident`.
    pub fn approach_for(&self, ident: &str) -> Option<&str> {
        self.approaches
            .iter()
            .find(|a| nav::approach_runway(a).is_some_and(|r| nav::same_runway_end(&r, ident)))
            .map(String::as_str)
    }
}

/// One runway END, ready to fly: its threshold, the far end, the course,
/// the length and the published pattern side.
#[derive(Debug, Clone, PartialEq)]
pub struct RunwayEnd {
    pub ident: String,
    /// The other end's designator.
    pub reciprocal: String,
    pub thr: LatLon,
    pub far: LatLon,
    /// Degrees true, the direction flown landing or taking off on it.
    pub course_deg: f64,
    pub length_m: f64,
    /// Feet MSL (the field's elevation: apt.dat carries no per-end figure).
    pub elev_ft: f64,
    /// The circuit is flown to the left (the published `1101` side, else
    /// left, the standard).
    pub left: bool,
}

impl RunwayEnd {
    /// A point in the runway frame: `along_m` from the threshold toward
    /// the far end, `right_m` off the centreline to the right of the course.
    pub fn at(&self, along_m: f64, right_m: f64) -> LatLon {
        let p = project_m(self.thr, self.course_deg, along_m);
        project_m(p, self.course_deg + 90.0, right_m)
    }

    /// `p` in the runway frame: (along from the threshold, right of the
    /// centreline), metres.
    pub fn frame(&self, p: LatLon) -> (f64, f64) {
        let d = dist_m(self.thr, p);
        if d < 1e-6 {
            return (0.0, 0.0);
        }
        let rel = (bearing(self.thr, p) - self.course_deg).to_radians();
        (d * rel.cos(), d * rel.sin())
    }

    /// The circuit side as a sign on `right_m`: -1 left, +1 right.
    pub fn side(&self) -> f64 {
        if self.left {
            -1.0
        } else {
            1.0
        }
    }
}

impl FieldModel {
    /// From parsed parts (tests and callers that already hold them). Every
    /// stand's name is made the field's own ([`unique_stand_names`]).
    pub fn new(icao: &str, airport: AirportInfo, net: Option<TaxiNet>) -> FieldModel {
        let mut net = net.unwrap_or_default();
        unique_stand_names(&mut net.stands);
        let mut f = FieldModel {
            icao: icao.to_string(),
            airport,
            net,
            procs: Procedures::default(),
            parked_only: Default::default(),
        };
        if f.net.stands.is_empty() {
            f.net.stands = f.apron_spots();
        }
        f.parked_only = f
            .net
            .stands
            .iter()
            .filter(|s| f.joins_through_a_stand(s))
            .map(|s| s.name.clone())
            .collect();
        f
    }

    /// The way onto the network from `stand` runs through another stand: the
    /// straight join to the nearest taxiway node (where nav's routes start
    /// and end, [`nav::TaxiNet::plan`]) passes within half its wingspan code
    /// of a stand beside it. ULKK (the hour sweep, 2026-10-09): a row of
    /// eight stands the network never reaches, so the Cirrus on the fourth
    /// joined the taxiway straight down the row and through the ATR parked
    /// on the ninth, 8 m from its nose.
    pub fn joins_through_a_stand(&self, stand: &Stand) -> bool {
        let at = (stand.lat, stand.lon);
        let Some(node) = self.join_node(at) else {
            return false;
        };
        self.net.stands.iter().any(|o| {
            let p = (o.lat, o.lon);
            dist_m(p, at) >= STAND_TAKEN_M
                && crate::kin::cross_track_m(&[at, node], p) < max_span_m(stand_width(o)) / 2.0
        })
    }

    /// The taxiway node nearest `p`: where a route from `p` starts.
    fn join_node(&self, p: LatLon) -> Option<LatLon> {
        self.net
            .edges
            .iter()
            .filter(|e| !e.runway)
            .flat_map(|e| [e.a, e.b])
            .filter_map(|id| self.net.nodes.get(&id).copied())
            .min_by(|a, b| dist_m(p, *a).total_cmp(&dist_m(p, *b)))
    }

    /// `stand` takes an `icao_type` that taxis to or from it: it fits
    /// ([`stand_fits`]), stands clear of the runways
    /// ([`FieldModel::beside_runway`]) and its way onto the network is its
    /// own ([`FieldModel::parked_only`]).
    pub fn takes(&self, stand: &Stand, icao_type: &str) -> bool {
        stand_fits(stand, icao_type)
            && !self.parked_only.contains(&stand.name)
            && !self.beside_runway((stand.lat, stand.lon))
    }

    /// Parking for a field that publishes no stands (apt.dat 1300): the
    /// dead ends of its taxi network clear of every runway, where a real
    /// apron's lines end. LXGB (2026-10-09): Gibraltar draws its taxiways
    /// and no stands, every movement needs a stand, and the pilot sat at
    /// the field with no traffic at all. Facing the way the line runs in,
    /// at most [`APRON_SPOTS_MAX`], none within [`APRON_SPOT_GAP_M`] of
    /// another. Empty with no network.
    fn apron_spots(&self) -> Vec<Stand> {
        let mut degree: std::collections::HashMap<i64, Vec<i64>> = Default::default();
        for e in self.net.edges.iter().filter(|e| !e.runway) {
            degree.entry(e.a).or_default().push(e.b);
            degree.entry(e.b).or_default().push(e.a);
        }
        let clear = |p: LatLon| {
            self.airport.runways.iter().all(|r| {
                self.end(&r.ident_a).is_none_or(|e| {
                    let (along, right) = e.frame(p);
                    right.abs() > APRON_CLEAR_M
                        || along < -APRON_CLEAR_M
                        || along > e.length_m + APRON_CLEAR_M
                })
            })
        };
        let mut ends: Vec<(i64, i64)> = degree
            .iter()
            .filter(|(_, n)| n.len() == 1)
            .map(|(id, n)| (*id, n[0]))
            .collect();
        ends.sort();
        let mut out: Vec<Stand> = Vec::new();
        for (id, from) in ends {
            let (Some(&p), Some(&q)) = (self.net.nodes.get(&id), self.net.nodes.get(&from)) else {
                continue;
            };
            if !clear(p) || out.iter().any(|s| dist_m((s.lat, s.lon), p) < APRON_SPOT_GAP_M) {
                continue;
            }
            out.push(Stand {
                lat: p.0,
                lon: p.1,
                heading: bearing(q, p),
                name: format!("Apron {}", out.len() + 1),
                kind: "tie_down".into(),
                accepts: Vec::new(),
                width: None,
            });
            if out.len() >= APRON_SPOTS_MAX {
                break;
            }
        }
        // Each spot's size is the room it has: half the way to the next
        // spot each side, and its wingtip clear of every lane but its own
        // lead-in; never more than [`APRON_SPOT_WIDTH`].
        let sizes: Vec<char> = out
            .iter()
            .map(|s| {
                let p = (s.lat, s.lon);
                let neighbour = out
                    .iter()
                    .map(|o| dist_m(p, (o.lat, o.lon)))
                    .filter(|d| *d > 0.0)
                    .fold(f64::INFINITY, f64::min);
                let room = f64::min(
                    neighbour - STAND_WINGTIP_GAP_M,
                    2.0 * (self.lane_clearance_m(p) - LANE_WINGTIP_GAP_M),
                );
                widest_code_within(room).min(APRON_SPOT_WIDTH)
            })
            .collect();
        for (s, w) in out.iter_mut().zip(sizes) {
            s.width = Some(w);
        }
        out
    }

    /// The field with no taxi edge passing within `clear_m` of `avoid`:
    /// where the ground spacing's guard plans a way round an aircraft that
    /// will not move off a route ([`crate::spacing::apply`]). The runways'
    /// own edges stay: a vacate is planned from them, and with the one under
    /// it gone there was no vacate at all (ULMM, 2026-10-09: a departure
    /// held short at the exit, and every other exit refused).
    pub fn without_edges_near(&self, avoid: LatLon, clear_m: f64) -> FieldModel {
        let mut f = self.clone();
        let nodes = &self.net.nodes;
        f.net.edges.retain(|e| {
            e.runway
                || match (nodes.get(&e.a), nodes.get(&e.b)) {
                    (Some(&a), Some(&b)) => crate::kin::cross_track_m(&[a, b], avoid) > clear_m,
                    _ => true,
                }
        });
        f
    }

    /// With the procedures from the CIFP folder.
    pub fn with_cifp(mut self, cifp_dir: &str) -> FieldModel {
        self.procs = Procedures::load(cifp_dir, &self.icao);
        self
    }

    /// Read `icao` from an apt.dat. `None` when the file has no such field.
    pub fn load(apt_dat: &str, icao: &str) -> anyhow::Result<Option<FieldModel>> {
        let Some(airport) = nav::find_airport(apt_dat, icao)? else {
            return Ok(None);
        };
        let net = nav::find_taxi_net(apt_dat, icao)?;
        Ok(Some(FieldModel::new(icao, airport, net)))
    }

    pub fn elev_ft(&self) -> f64 {
        self.airport.elev_ft
    }

    /// The field's reference point (the first threshold).
    pub fn reference(&self) -> Option<LatLon> {
        self.airport.position()
    }

    /// The physical runway that has end `ident`.
    pub fn strip(&self, ident: &str) -> Option<&Runway> {
        self.airport.runways.iter().find(|r| {
            nav::same_runway_end(&r.ident_a, ident) || nav::same_runway_end(&r.ident_b, ident)
        })
    }

    /// Runway end `ident`, flyable.
    pub fn end(&self, ident: &str) -> Option<RunwayEnd> {
        let r = self.strip(ident)?;
        let a_end = nav::same_runway_end(&r.ident_a, ident);
        let (ident, reciprocal, thr, far) = if a_end {
            (
                &r.ident_a,
                &r.ident_b,
                (r.lat_a, r.lon_a),
                (r.lat_b, r.lon_b),
            )
        } else {
            (
                &r.ident_b,
                &r.ident_a,
                (r.lat_b, r.lon_b),
                (r.lat_a, r.lon_a),
            )
        };
        Some(RunwayEnd {
            ident: ident.clone(),
            reciprocal: reciprocal.clone(),
            thr,
            far,
            course_deg: bearing(thr, far),
            length_m: dist_m(thr, far),
            elev_ft: self.airport.elev_ft,
            left: self.airport.pattern_left(ident).unwrap_or(true),
        })
    }

    /// Every runway end at the field, in file order.
    pub fn ends(&self) -> Vec<RunwayEnd> {
        self.airport
            .runways
            .iter()
            .flat_map(|r| [r.ident_a.clone(), r.ident_b.clone()])
            .filter_map(|id| self.end(&id))
            .collect()
    }

    /// The taxi route from `from` to the holding point of `end`.
    pub fn taxi_to_runway(&self, from: LatLon, end: &RunwayEnd) -> Option<TaxiRoutePlan> {
        let zone = [end.ident.clone(), end.reciprocal.clone()];
        self.net.plan_to_runway(
            from.0,
            from.1,
            end.thr.0,
            end.thr.1,
            nav::HOLD_SHORT_STANDOFF_M,
            &zone,
            self.strip(&end.ident),
        )
    }

    /// The drive off `end`'s strip from a rollout at `pos` heading `hdg`:
    /// forward to an exit, or, where the parking is behind, turned round
    /// and BACKTRACKED to the exit nearest it. BIEG 03 (world sweep,
    /// 2026-10-08): the one exit to the apron is behind a landing, so the
    /// forward vacate rolled 1.2 km to the far end, waited there for a
    /// crossing and was cleared 1,850 m back down the whole runway: the
    /// strip held for minutes, 22 go-arounds in the hour. A real crew turns
    /// round and backtracks as it slows. The backtrack is taken when its
    /// drive to the stand is under [`BACKTRACK_SAVES`] of the forward one,
    /// or when there is no exit ahead at all. The parking is the one an
    /// `icao_type` takes ([`FieldModel::parking_near`]).
    ///
    /// It never ends where somebody stands (`standing`, the aircraft
    /// stopped on the ground): another way off round them, else cut back
    /// short of them while that is still off the runway's side, else as
    /// planned (the spacing stops it behind them). BIGH (the runway-slot
    /// sweep, 2026-10-09, #249): a vacate ended exactly where the last
    /// arrival had ended its own and stood, held short of a crossing of the
    /// same runway, the two inside each other.
    pub fn vacate(
        &self,
        pos: LatLon,
        hdg: f64,
        end: &RunwayEnd,
        icao_type: &str,
        standing: &[LatLon],
    ) -> Option<VacatePlan> {
        let v = self.vacate_open(pos, hdg, end, icao_type)?;
        let near = |p: LatLon| -> Vec<LatLon> {
            standing
                .iter()
                .copied()
                .filter(|s| dist_m(p, *s) < VACATE_TAKEN_M)
                .collect()
        };
        let taken = |p: &LatLon| !near(*p).is_empty();
        let Some(&off) = v.path.last().filter(|p| taken(p)) else {
            return Some(v);
        };
        let open = near(off)
            .into_iter()
            .fold(self.clone(), |f, s| f.without_edges_near(s, VACATE_TAKEN_M));
        let round = open
            .vacate_open(pos, hdg, end, icao_type)
            .filter(|r| r.path.last().is_some_and(|p| !taken(p)));
        if round.is_some() {
            return round;
        }
        let keep = v
            .path
            .iter()
            .rposition(|p| !taken(p) && !self.beside_runway(*p))
            .filter(|i| *i >= 1);
        let mut v = v;
        if let Some(i) = keep {
            v.path.truncate(i + 1);
            v.seg.truncate(i);
        }
        Some(v)
    }

    /// [`FieldModel::vacate`] with nobody in the way.
    fn vacate_open(
        &self,
        pos: LatLon,
        hdg: f64,
        end: &RunwayEnd,
        icao_type: &str,
    ) -> Option<VacatePlan> {
        let zone = [end.ident.clone(), end.reciprocal.clone()];
        let plan = |h: f64| {
            self.net
                .plan_vacate(pos.0, pos.1, h, &zone, nav::VACATE_CLEAR_M)
                .filter(|v| v.path.len() >= 2)
                .map(|v| self.short_of_the_stands(v))
        };
        let ahead = plan(hdg).or_else(|| self.drive_off(pos, hdg));
        let Some(stand) = self.parking_near(pos, icao_type) else {
            return ahead;
        };
        let behind = plan((hdg + 180.0).rem_euclid(360.0));
        let ahead = ahead.map(|v| self.off_the_strip(v, stand));
        let behind = behind.map(|v| self.off_the_strip(v, stand));
        // The whole drive to the parking: the vacate, then the stand route.
        let drive_m = |v: &VacatePlan| -> Option<f64> {
            let off = *v.path.last()?;
            let route = self.net.plan_to_stand(off.0, off.1, stand)?;
            Some(path_m(&v.path) + path_m(&route.path))
        };
        match (ahead, behind) {
            (Some(a), Some(b)) => match (drive_m(&a), drive_m(&b)) {
                (Some(da), Some(db)) if db < BACKTRACK_SAVES * da => Some(b),
                _ => Some(a),
            },
            (a, b) => a.or(b),
        }
    }

    /// `v` ended short of the stands it would end among: cut back to its
    /// last point [`VACATE_STAND_CLEAR_M`] from every stand and off the
    /// runway's side ([`FieldModel::beside_runway`]); left whole when no
    /// point is. BIRK 13 (the hour sweep, 2026-10-09): exit B runs on into
    /// a dead end ringed by GA tie-downs 11 m round its last node, the
    /// vacate drove to that node and stopped there, wingtips in the parked
    /// Cessnas, and the taxi-in turned round through them.
    fn short_of_the_stands(&self, mut v: VacatePlan) -> VacatePlan {
        let among = |p: LatLon| {
            self.net
                .stands
                .iter()
                .any(|s| dist_m(p, (s.lat, s.lon)) < VACATE_STAND_CLEAR_M)
        };
        if !v.path.last().is_some_and(|p| among(*p)) {
            return v;
        }
        let keep = v
            .path
            .iter()
            .rposition(|p| !among(*p) && !self.beside_runway(*p));
        if let Some(i) = keep.filter(|i| *i >= 1) {
            v.path.truncate(i + 1);
            v.seg.truncate(i);
        }
        v
    }

    /// The vacate where the data draws none: straight to the nearest taxi
    /// node off every runway, ahead of abeam and within [`DRIVE_OFF_M`].
    /// PAWS 4 (world sweep, 2026-10-09): the runway is not in the field's
    /// network at all (its runway edges are the parallel gravel strip's),
    /// so no vacate was ever planned, every arrival rolled to the end and
    /// stood there, and the next drove into it.
    fn drive_off(&self, pos: LatLon, hdg: f64) -> Option<VacatePlan> {
        let off: std::collections::HashSet<i64> = self
            .net
            .edges
            .iter()
            .filter(|e| !e.runway)
            .flat_map(|e| [e.a, e.b])
            .collect();
        let to = off
            .iter()
            .filter_map(|id| self.net.nodes.get(id).copied())
            .filter(|p| !self.beside_runway(*p))
            .filter(|p| {
                let d = (bearing(pos, *p) - hdg + 540.0).rem_euclid(360.0) - 180.0;
                d.abs() <= 100.0
            })
            .filter(|p| dist_m(pos, *p) <= DRIVE_OFF_M)
            .min_by(|a, b| dist_m(pos, *a).total_cmp(&dist_m(pos, *b)))?;
        Some(VacatePlan {
            path: vec![pos, to],
            seg: vec![Default::default()],
            exit_name: String::new(),
        })
    }

    /// `v` carried on along the stand route until it is OFF the runway's
    /// pavement. UNSS 15 (world sweep, 2026-10-09): the taxiways there are
    /// drawn down the runway itself, so both vacates ended on the centreline;
    /// the arrival said "runway vacated" standing on it and the tower called
    /// it on the runway without clearance. A vacate already off, or with no
    /// route on, is left as it is.
    fn off_the_strip(&self, mut v: VacatePlan, stand: &Stand) -> VacatePlan {
        let Some(&last) = v.path.last() else {
            return v;
        };
        if !self.on_runway(last) {
            return v;
        }
        let Some(route) = self.net.plan_to_stand(last.0, last.1, stand) else {
            // No way drawn off (BIGH, a gravel strip whose network is its
            // runway): straight to a stand that close, as a crew would.
            if dist_m(last, (stand.lat, stand.lon)) <= STRAIGHT_TO_STAND_M {
                v.path.push((stand.lat, stand.lon));
                v.seg.push(Default::default());
            }
            return v;
        };
        let start = route
            .path
            .iter()
            .position(|p| dist_m(*p, last) >= 1.0)
            .unwrap_or(route.path.len());
        for (k, p) in route.path.iter().enumerate().skip(start) {
            v.path.push(*p);
            v.seg.push(
                route
                    .seg
                    .get(k.saturating_sub(1))
                    .cloned()
                    .unwrap_or_default(),
            );
            if !self.on_runway(*p) {
                break;
            }
        }
        v
    }

    /// `p` is on a runway strip or within [`STAND_RUNWAY_GAP_M`] of its
    /// edge, alongside it: no place for a stand. UEMH 27 (world sweep,
    /// 2026-10-09): a stand against the runway edge put a departure on the
    /// runway within 15 m of starting to taxi.
    pub fn beside_runway(&self, p: LatLon) -> bool {
        self.on_runway(p)
            || self.airport.runways.iter().any(|r| {
                self.end(&r.ident_a).is_some_and(|e| {
                    let (along, right) = e.frame(p);
                    (0.0..=e.length_m).contains(&along)
                        && right.abs() <= r.width_m.max(30.0) / 2.0 + STAND_RUNWAY_GAP_M
                })
            })
    }

    /// How far `p` is from the nearest lane traffic taxis along, metres:
    /// every taxiway edge but a dead-end spur ending within
    /// [`SPUR_END_M`] of `p` (a stand's own lead-in). Infinite with no
    /// network.
    pub fn lane_clearance_m(&self, p: LatLon) -> f64 {
        let mut degree: std::collections::HashMap<i64, usize> = Default::default();
        for e in self.net.edges.iter().filter(|e| !e.runway) {
            *degree.entry(e.a).or_default() += 1;
            *degree.entry(e.b).or_default() += 1;
        }
        let m_lon = 111_320.0 * p.0.to_radians().cos();
        let xy = |q: LatLon| ((q.1 - p.1) * m_lon, (q.0 - p.0) * 111_320.0);
        self.net
            .edges
            .iter()
            .filter(|e| !e.runway)
            .filter_map(|e| Some((e, *self.net.nodes.get(&e.a)?, *self.net.nodes.get(&e.b)?)))
            .filter(|(e, a, b)| {
                let spur = |id: i64, q: LatLon| degree.get(&id) == Some(&1) && dist_m(p, q) <= SPUR_END_M;
                !spur(e.a, *a) && !spur(e.b, *b)
            })
            .map(|(_, a, b)| {
                let ((ax, ay), (bx, by)) = (xy(a), xy(b));
                let (dx, dy) = (bx - ax, by - ay);
                let len2 = dx * dx + dy * dy;
                let t = if len2 > 0.0 { ((-ax * dx - ay * dy) / len2).clamp(0.0, 1.0) } else { 0.0 };
                let (cx, cy) = (ax + t * dx, ay + t * dy);
                (cx * cx + cy * cy).sqrt()
            })
            .fold(f64::INFINITY, f64::min)
    }

    /// The stand an `icao_type` arrival parks on, nearest `p`: one that
    /// takes it ([`stand_fits`]), off every runway strip, of the tightest
    /// size there is.
    pub fn parking_near(&self, p: LatLon, icao_type: &str) -> Option<&Stand> {
        nearest_tightest(
            self.net.stands.iter().filter(|s| self.takes(s, icao_type)),
            icao_type,
            p,
        )
    }

    /// The taxi route from `from` onto `stand`.
    pub fn taxi_to_stand(&self, from: LatLon, stand: &Stand) -> Option<TaxiRoutePlan> {
        self.net.plan_to_stand(from.0, from.1, stand)
    }

    /// The stand called `name` (exact, then case-insensitive).
    pub fn stand(&self, name: &str) -> Option<&Stand> {
        self.net.stands.iter().find(|s| s.name == name).or_else(|| {
            self.net
                .stands
                .iter()
                .find(|s| s.name.eq_ignore_ascii_case(name))
        })
    }

    /// The stand nearest `from` that takes an `icao_type` ([`stand_fits`])
    /// and nobody in `occupied` stands on (within [`STAND_TAKEN_M`]), of the
    /// tightest size free: a bigger stand only when none of its own size is.
    /// Of those, one with no neighbour in `occupied` ([`neighbour_taken`])
    /// first: a quiet apron is not filled stand by stand.
    pub fn free_stand(&self, from: LatLon, occupied: &[LatLon], icao_type: &str) -> Option<&Stand> {
        let free = || {
            self.net
                .stands
                .iter()
                .filter(|s| self.takes(s, icao_type) && !stand_taken(s, occupied))
        };
        nearest_tightest(
            free().filter(|s| !neighbour_taken(s, occupied)),
            icao_type,
            from,
        )
        .filter(|s| {
            // Lonely, but not of a bigger size than one free beside somebody.
            nearest_tightest(free(), icao_type, from)
                .is_none_or(|t| stand_width(t) == stand_width(s))
        })
        .or_else(|| nearest_tightest(free(), icao_type, from))
    }
}

/// A taxi stops this far along its route short of where the route comes
/// onto a runway's pavement (nav's [`nav::on_strip`], the tower's own
/// test): the holding position of a crossing.
pub const CROSSING_STOP_M: f64 = 20.0;

/// The route is sampled this often for the crossings.
const CROSSING_SAMPLE_M: f64 = 2.0;

/// A taxi route cut at the runways it crosses, by the strips' GEOMETRY
/// (never the network's flags): `pieces[i]` ends at the stop short of
/// `crossings[i]` (the end nearer where the route comes onto it); the last
/// piece runs on to the route's end. A route that starts on a strip (a
/// vacate's end) does not stop for it; a route that ENDS on a strip stops
/// short of it and goes no further.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CutRoute {
    pub pieces: Vec<Vec<LatLon>>,
    pub crossings: Vec<String>,
}

impl FieldModel {
    /// The strip under `p`, by index, and the nearer end's ident.
    fn strip_at(&self, p: LatLon) -> Option<(usize, String)> {
        self.airport
            .runways
            .iter()
            .enumerate()
            .find_map(|(i, r)| nav::on_strip(r, p.0, p.1).map(|end| (i, end)))
    }

    /// `p` is on a runway's strip (the geometry the crossings are cut by).
    pub fn on_runway(&self, p: LatLon) -> bool {
        self.strip_at(p).is_some()
    }

    /// `path` cut at every runway it crosses ([`CutRoute`]).
    pub fn cut_at_crossings(&self, path: &[LatLon]) -> CutRoute {
        if path.len() < 2 {
            return CutRoute {
                pieces: vec![path.to_vec()],
                crossings: Vec::new(),
            };
        }
        // Each sample: metres along, the point, the strip under it.
        let mut samples: Vec<(f64, LatLon, Option<(usize, String)>)> = Vec::new();
        let mut along = 0.0;
        for (i, w) in path.windows(2).enumerate() {
            let len = dist_m(w[0], w[1]);
            let n = ((len / CROSSING_SAMPLE_M).ceil() as usize).max(1);
            for k in usize::from(i > 0)..=n {
                let f = k as f64 / n as f64;
                let p = (
                    w[0].0 + (w[1].0 - w[0].0) * f,
                    w[0].1 + (w[1].1 - w[0].1) * f,
                );
                samples.push((along + len * f, p, self.strip_at(p)));
            }
            along += len;
        }
        // The entries: where the route comes onto a strip it was not on, the
        // first sample excepted (already on it), and whether it leaves again.
        let mut stops: Vec<(f64, String, bool)> = Vec::new();
        let mut prev: Option<usize> = samples.first().and_then(|s| s.2.as_ref().map(|x| x.0));
        for s in samples.iter().skip(1) {
            let here = s.2.as_ref().map(|x| x.0);
            if here.is_some() && prev.is_none() {
                let (_, end) = s.2.clone().unwrap_or_default();
                stops.push((s.0, end, false));
            }
            if here.is_none() && prev.is_some() {
                if let Some(last) = stops.last_mut() {
                    last.2 = true;
                }
            }
            prev = here;
        }
        let mut out = CutRoute::default();
        let mut from = 0.0;
        for (enter_m, end, leaves) in stops {
            let stop = (enter_m - CROSSING_STOP_M).max(from);
            out.pieces.push(slice_m(path, from, stop));
            if !leaves {
                // It ends on the strip: short of it is where it stops.
                return out;
            }
            out.crossings.push(end);
            from = stop;
        }
        out.pieces.push(slice_m(path, from, f64::INFINITY));
        out
    }
}

/// The part of `path` from `a_m` to `b_m` metres along it.
fn slice_m(path: &[LatLon], a_m: f64, b_m: f64) -> Vec<LatLon> {
    let at = |m: f64| -> LatLon {
        let mut along = 0.0;
        for w in path.windows(2) {
            let len = dist_m(w[0], w[1]);
            if along + len >= m && len > 0.0 {
                let f = ((m - along) / len).clamp(0.0, 1.0);
                return (
                    w[0].0 + (w[1].0 - w[0].0) * f,
                    w[0].1 + (w[1].1 - w[0].1) * f,
                );
            }
            along += len;
        }
        *path.last().unwrap_or(&(0.0, 0.0))
    };
    let mut out = vec![at(a_m)];
    let mut along = 0.0;
    for w in path.windows(2) {
        along += dist_m(w[0], w[1]);
        if along > a_m + 0.5 && along < b_m - 0.5 {
            out.push(w[1]);
        }
    }
    if b_m.is_finite() {
        out.push(at(b_m));
    } else if let Some(&last) = path.last() {
        if out.last() != Some(&last) {
            out.push(last);
        }
    }
    out.dedup();
    out
}

/// Every stand named once: a second stand of the same name gets a number
/// ("GA Tie-Down 2"). Stands are found BY NAME ([`FieldModel::stand`]), and
/// at a field that names every tie-down alike each one resolved to the
/// first: CYJP's four "GA Tie-Down" and EFMI's eight "Apron Stand" put two
/// and three aircraft on one stand in the hour sweep (2026-10-08).
fn unique_stand_names(stands: &mut [Stand]) {
    let mut used: std::collections::HashSet<String> =
        stands.iter().map(|s| s.name.clone()).collect();
    let mut seen: std::collections::HashSet<String> = std::collections::HashSet::new();
    for s in stands.iter_mut() {
        if seen.insert(s.name.clone()) {
            continue;
        }
        let name = (2..)
            .map(|k| format!("{} {k}", s.name))
            .find(|n| !used.contains(n))
            .unwrap_or_default();
        used.insert(name.clone());
        seen.insert(name.clone());
        s.name = name;
    }
}

/// An aircraft this close to a stand is on it.
pub const STAND_TAKEN_M: f64 = 20.0;

/// A dead-end spur ending this close to a stand is its own lead-in, metres.
const SPUR_END_M: f64 = 40.0;

/// A stand is at least this far beyond a runway's edge, metres.
const STAND_RUNWAY_GAP_M: f64 = 30.0;

/// A vacate the data draws none for drives off to a taxi node this close,
/// metres.
const DRIVE_OFF_M: f64 = 200.0;

/// A vacate ends at least this far from every stand, metres
/// ([`FieldModel::vacate`]).
const VACATE_STAND_CLEAR_M: f64 = 25.0;

/// A vacate ends at least this far from an aircraft standing on the
/// ground, metres, centre to centre: two light aircraft's taxi spacing.
const VACATE_TAKEN_M: f64 = 40.0;

/// A vacate with no route off drives straight to a stand this close, metres.
const STRAIGHT_TO_STAND_M: f64 = 800.0;

/// An apron spot is at least this far from every runway, metres.
const APRON_CLEAR_M: f64 = 120.0;
/// ...and this far from the next one.
const APRON_SPOT_GAP_M: f64 = 40.0;
/// A field without stands gets at most this many.
const APRON_SPOTS_MAX: usize = 12;

/// A backtrack vacate is taken when its drive to the parking is under this
/// share of the forward one's ([`FieldModel::vacate`]): a turn round on the
/// runway is paid for only by a real saving.
pub const BACKTRACK_SAVES: f64 = 0.6;

/// The length of a polyline, metres.
fn path_m(path: &[LatLon]) -> f64 {
    path.windows(2).map(|w| dist_m(w[0], w[1])).sum()
}

/// Somebody in `occupied` is on `stand`.
pub fn stand_taken(stand: &Stand, occupied: &[LatLon]) -> bool {
    occupied
        .iter()
        .any(|p| dist_m(*p, (stand.lat, stand.lon)) < STAND_TAKEN_M)
}

/// Somebody in `taken` stands on a stand beside `stand`: within its
/// wingspan code's widest span and [`NEIGHBOUR_GAP_M`], never on it
/// ([`STAND_TAKEN_M`]). An aircraft leaving `stand` passes them on its way
/// out. The hour sweep (2026-10-09): sized to fit, ENVA's airliners all
/// took its eleven code C gates, 40 m apart, and every departure left past
/// the one parked beside it.
pub fn neighbour_taken(stand: &Stand, taken: &[LatLon]) -> bool {
    let reach = max_span_m(stand_width(stand)) + NEIGHBOUR_GAP_M;
    taken.iter().any(|p| {
        let d = dist_m(*p, (stand.lat, stand.lon));
        (STAND_TAKEN_M..reach).contains(&d)
    })
}

/// A stand is a neighbour's within its span and this, metres.
const NEIGHBOUR_GAP_M: f64 = 10.0;

/// A field's own apron spot is at most this size (a narrowbody): a dead end
/// in a network is no widebody's stand.
const APRON_SPOT_WIDTH: char = 'C';
/// Two aircraft parked side by side keep this between their wingtips,
/// metres (ICAO Annex 14's stand clearance for code C is 4.5 m).
const STAND_WINGTIP_GAP_M: f64 = 4.5;
/// A parked wingtip keeps this from the centreline of a lane traffic taxis
/// along, metres.
const LANE_WINGTIP_GAP_M: f64 = 10.0;

/// The widest wingspan a code letter takes, metres.
fn max_span_m(code: char) -> f64 {
    match code {
        'A' => 15.0,
        'B' => 24.0,
        'C' => 36.0,
        'D' => 52.0,
        'E' => 65.0,
        _ => 80.0,
    }
}

/// The biggest wingspan code whose every aircraft spans `room_m` or less
/// ('A' when even that one does not fit: a spot is never sized to nothing).
fn widest_code_within(room_m: f64) -> char {
    ['F', 'E', 'D', 'C', 'B']
        .into_iter()
        .find(|c| max_span_m(*c) <= room_m)
        .unwrap_or('A')
}

/// apt.dat's stand category for `icao_type`'s class: heavy, jets,
/// turboprops or props.
pub fn stand_category(icao_type: &str) -> &'static str {
    let (perf, _) = crate::perf::class_for_type(icao_type);
    // By name: a `const` table's address is not one address.
    if perf.name == crate::perf::JET_HEAVY.name {
        "heavy"
    } else if perf.name == crate::perf::JET_MED.name {
        "jets"
    } else if perf.name == crate::perf::TURBOPROP.name {
        "turboprops"
    } else {
        "props"
    }
}

/// The biggest wingspan code `stand` takes: its 1301 row's, else what the
/// categories it names imply (a heavy's stand E, a jet's or a turboprop's
/// C, anything else B), else its kind (a gate C, anything else B).
pub fn stand_width(stand: &Stand) -> char {
    if let Some(w) = stand.width {
        return w;
    }
    let takes = |c: &str| stand.accepts.iter().any(|a| a == c);
    if takes("heavy") {
        'E'
    } else if takes("jets") || takes("turboprops") {
        'C'
    } else if stand.accepts.is_empty() && stand.kind.eq_ignore_ascii_case("gate") {
        'C'
    } else {
        'B'
    }
}

/// `stand` takes an `icao_type`: a suitable stand ([`Stand::is_suitable`])
/// that names the type's category among its own (a jet's stand takes a
/// heavy's too, and a heavy's a jet), or names none, and is as wide as the
/// type's wingspan code ([`crate::perf::wingspan_code`]). The hour sweep
/// (2026-10-09): airliners on light aircraft's stands, packed 20 m apart,
/// left their stands through the neighbours parked beside them.
pub fn stand_fits(stand: &Stand, icao_type: &str) -> bool {
    let cat = stand_category(icao_type);
    let named = stand.accepts.is_empty()
        || stand.accepts.iter().any(|a| {
            a == cat || (cat == "jets" && a == "heavy") || (cat == "heavy" && a == "jets")
        });
    stand.is_suitable() && named && stand_width(stand) >= crate::perf::wingspan_code(icao_type)
}

/// `stands` that take an `icao_type` ([`stand_fits`]), the tightest first:
/// every stand of the smallest size that fits, then the next size up, each
/// size in the order given. A big stand is kept for a big aircraft.
pub fn tightest_first<'a>(
    stands: impl IntoIterator<Item = &'a Stand>,
    icao_type: &str,
) -> Vec<&'a Stand> {
    let mut out: Vec<&Stand> = stands
        .into_iter()
        .filter(|s| stand_fits(s, icao_type))
        .collect();
    out.sort_by_key(|s| stand_width(s));
    out
}

/// Of `stands`, the one nearest `p` among those of the tightest size that
/// takes an `icao_type` ([`tightest_first`]).
fn nearest_tightest<'a>(
    stands: impl IntoIterator<Item = &'a Stand>,
    icao_type: &str,
    p: LatLon,
) -> Option<&'a Stand> {
    let fits = tightest_first(stands, icao_type);
    let size = stand_width(fits.first()?);
    fits.into_iter()
        .take_while(|s| stand_width(s) == size)
        .min_by(|a, b| dist_m(p, (a.lat, a.lon)).total_cmp(&dist_m(p, (b.lat, b.lon))))
}

#[cfg(test)]
pub(crate) mod fixtures {
    use super::*;

    /// A fixture under `crates/traffic/fixtures/`.
    pub fn path(name: &str) -> String {
        format!("{}/fixtures/{name}", env!("CARGO_MANIFEST_DIR"))
    }

    /// XTST: one 1,500 m runway 09/27, a parallel taxiway, an apron with
    /// four stands (see `fixtures/xtst.dat`).
    pub fn xtst() -> FieldModel {
        FieldModel::load(&path("xtst.dat"), "XTST")
            .unwrap()
            .expect("XTST in the fixture")
    }

    /// XTST with its procedures (a SID and an approach for 27).
    pub fn xtst_ifr() -> FieldModel {
        xtst().with_cifp(&path("cifp"))
    }

    /// XNOC: no circuits (`allows_circuits 0`), a 2,800 m runway 12/30,
    /// three stands (two for jets), SIDs, a STAR and an ILS 12.
    pub fn xnoc() -> FieldModel {
        FieldModel::load(&path("xnoc.dat"), "XNOC")
            .unwrap()
            .expect("XNOC in the fixture")
            .with_cifp(&path("cifp"))
    }

    /// XBAR: an 800 m grass strip with no network, no stands, no
    /// procedures.
    pub fn xbar() -> FieldModel {
        FieldModel::load(&path("xbar.dat"), "XBAR")
            .unwrap()
            .expect("XBAR in the fixture")
            .with_cifp(&path("cifp"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// BIEG 03 (2026-10-08): the apron's one exit is near the 03
    /// threshold, behind a landing. A rollout 1.2 km down 03 turns round
    /// and backtracks to it, never on to the far end and the whole runway
    /// back; on XTST, a parallel taxiway with exits ahead, the vacate is
    /// forward as before.
    #[test]
    fn a_rollout_past_the_only_exit_to_the_apron_backtracks_to_it() {
        let f = FieldModel::load(&fixtures::path("bieg.dat"), "BIEG")
            .unwrap()
            .expect("BIEG in the fixture");
        let e03 = f.end("03").unwrap();
        let here = (65.28663668654765, -14.397736800949273);
        let v = f
            .vacate(here, e03.course_deg, &e03, "C172", &[])
            .expect("a vacate");
        let off = *v.path.last().unwrap();
        assert!(
            e03.frame(off).0 < e03.frame(here).0,
            "turned round: off at {:.0} m, rollout at {:.0} m",
            e03.frame(off).0,
            e03.frame(here).0
        );
        let stand = f.net.nearest_stand(here.0, here.1).unwrap();
        let route = f.net.plan_to_stand(off.0, off.1, stand).unwrap();
        assert!(
            path_m(&route.path) < 600.0,
            "then a short way in: {:.0} m",
            path_m(&route.path)
        );

        let x = fixtures::xtst();
        let e09 = x.end("09").unwrap();
        let roll = e09.at(500.0, 0.0);
        let v = x
            .vacate(roll, e09.course_deg, &e09, "C172", &[])
            .expect("a vacate");
        assert!(
            e09.frame(*v.path.last().unwrap()).0 > 500.0,
            "forward to the next exit"
        );
    }

    #[test]
    fn the_fixture_field_reads_with_its_geometry() {
        let f = fixtures::xtst();
        let e09 = f.end("09").unwrap();
        assert_eq!(e09.reciprocal, "27");
        assert!((e09.course_deg - 90.0).abs() < 0.5, "{}", e09.course_deg);
        assert!((e09.length_m - 1500.0).abs() < 5.0, "{}", e09.length_m);
        assert!(e09.left, "unpublished is left");
        assert!(!f.end("27").unwrap().left, "27 publishes right");
        let (along, right) = e09.frame(e09.at(300.0, -100.0));
        assert!((along - 300.0).abs() < 0.5 && (right + 100.0).abs() < 0.5);
        assert!(f.net.stands.len() >= 4);
        let route = f
            .taxi_to_runway((36.0026, -5.0003), &e09)
            .expect("a route to 09");
        assert!(!route.lead_on.is_empty(), "the hold is short of the strip");
    }

    /// A route across a runway is cut short of its PAVEMENT (the strip's
    /// geometry, as the tower reads it), the crossing named by the nearer
    /// end; a route that starts on a strip is not stopped for it; one that
    /// ends on a strip stops short and goes no further.
    #[test]
    fn a_route_is_cut_short_of_every_runway_it_crosses() {
        let f = fixtures::xtst();
        let e09 = f.end("09").unwrap();
        // North to south across 09/27 (30 m wide), a quarter of the way along.
        let across = vec![(36.0015, -5.004), (35.9985, -5.004)];
        let cut = f.cut_at_crossings(&across);
        assert_eq!(cut.crossings, vec!["09".to_string()]);
        assert_eq!(cut.pieces.len(), 2);
        let stop = *cut.pieces[0].last().unwrap();
        let off = e09.frame(stop).1;
        // The pavement test reaches 15 + 10 m off the centreline; the stop
        // is CROSSING_STOP_M before it, on the near (north, left) side.
        assert!((-47.0..=-43.0).contains(&off), "stop {off:.1} m off");
        assert_eq!(
            cut.pieces[1].first(),
            Some(&stop),
            "the next piece starts there"
        );
        assert_eq!(cut.pieces[1].last(), across.last(), "and runs to the end");
        // Starting on the strip (a vacate's end): nothing to hold short of.
        let off_it = vec![(36.0, -5.004), (35.9985, -5.004)];
        let cut = f.cut_at_crossings(&off_it);
        assert!(cut.crossings.is_empty() && cut.pieces.len() == 1);
        // Ending on it: short of it, and no further.
        let onto = vec![(36.0015, -5.004), (36.0, -5.004)];
        let cut = f.cut_at_crossings(&onto);
        assert!(cut.crossings.is_empty());
        assert_eq!(cut.pieces.len(), 1);
        let end = *cut.pieces[0].last().unwrap();
        assert!(e09.frame(end).1 < -40.0, "stopped short of the pavement");
        // Along the parallel taxiway: no crossing at all.
        let parallel = vec![(36.0008, -5.008), (36.0008, -4.992)];
        let cut = f.cut_at_crossings(&parallel);
        assert!(cut.crossings.is_empty());
        assert_eq!(cut.pieces, vec![parallel]);
    }
}
#[cfg(test)]
mod apron_tests {
    use super::*;

    /// A field with no stands gets its own apron spots at the dead ends of
    /// its network clear of the runway (LXGB, 2026-10-09); the stand
    /// filter never offers one beside the runway.
    #[test]
    fn a_field_with_no_stands_parks_at_its_network_dead_ends() {
        let x = fixtures::xtst();
        let mut net = x.net.clone();
        net.stands.clear();
        let f = FieldModel::new("XTST", x.airport.clone(), Some(net));
        assert!(!f.net.stands.is_empty(), "apron spots made");
        for s in &f.net.stands {
            assert!(!f.beside_runway((s.lat, s.lon)), "{} beside the runway", s.name);
            assert!(s.name.starts_with("Apron "));
        }
        let e = f.end("09").unwrap();
        assert!(f.beside_runway(e.at(300.0, 30.0)), "30 m off the centreline is beside it");
        assert!(!f.beside_runway(e.at(300.0, 200.0)));
        // Each is sized by the room it has, never past a narrowbody's.
        for s in &f.net.stands {
            let w = s.width.expect("an apron spot has a size");
            assert!(('A'..=APRON_SPOT_WIDTH).contains(&w), "{} is {w}", s.name);
        }
    }

    fn stand(name: &str, at: LatLon, accepts: &[&str], width: Option<char>) -> Stand {
        Stand {
            lat: at.0,
            lon: at.1,
            heading: 180.0,
            name: name.into(),
            kind: "tie_down".into(),
            accepts: accepts.iter().map(|a| a.to_string()).collect(),
            width,
        }
    }

    /// XTST with `stands` for its own.
    fn xtst_with(stands: Vec<Stand>) -> FieldModel {
        let x = fixtures::xtst();
        let mut net = x.net.clone();
        net.stands = stands;
        FieldModel::new("XTST", x.airport.clone(), Some(net))
    }

    /// The hour sweep (2026-10-09): airliners on light aircraft's stands
    /// left them through the aircraft parked beside them. A stand takes a
    /// type of its category and no wider than its 1301 size; of those the
    /// tightest size is taken first, a bigger one only when none of its own
    /// size is free, and nothing when none is wide enough.
    #[test]
    fn an_airliner_is_never_put_on_a_light_aircrafts_stand() {
        let small = stand("Small", (36.0026, -5.0006), &["jets", "props"], Some('B'));
        let c = stand("C", (36.0026, -4.9994), &["jets"], Some('C'));
        let e = stand("E", (36.0026, -5.0), &["heavy", "jets"], Some('E'));
        let props = stand("Props", (36.0026, -5.0012), &["props"], Some('C'));
        assert!(
            !stand_fits(&small, "A320"),
            "a B stand is too small for an A320"
        );
        assert!(stand_fits(&c, "A320") && stand_fits(&e, "B738"));
        assert!(stand_fits(&e, "B77W") && !stand_fits(&c, "B77W"));
        assert!(
            !stand_fits(&props, "A320"),
            "a props stand takes no jet, however wide"
        );
        assert!(stand_fits(&small, "C172") && stand_fits(&props, "C172"));
        let order: Vec<&str> = tightest_first([&e, &small, &c], "A319")
            .into_iter()
            .map(|s| s.name.as_str())
            .collect();
        assert_eq!(order, vec!["C", "E"]);

        let f = xtst_with(vec![small.clone(), c.clone(), e.clone()]);
        let near_e = (e.lat, e.lon);
        assert_eq!(
            f.free_stand(near_e, &[], "A320").map(|s| s.name.as_str()),
            Some("C"),
            "the code C stand, though the E is nearer"
        );
        assert_eq!(
            f.free_stand(near_e, &[(c.lat, c.lon)], "A320")
                .map(|s| s.name.as_str()),
            Some("E"),
            "the next size up when it is taken"
        );
        assert_eq!(
            f.free_stand(near_e, &[(c.lat, c.lon), (e.lat, e.lon)], "A320")
                .map(|s| s.name.as_str()),
            None,
            "never the B stand"
        );
        assert_eq!(
            f.parking_near(near_e, "A320").map(|s| s.name.as_str()),
            Some("C")
        );
        // No 1301 row: judged by what it takes.
        assert_eq!(stand_width(&stand("x", (0.0, 0.0), &["props"], None)), 'B');
        assert_eq!(stand_width(&stand("x", (0.0, 0.0), &["jets"], None)), 'C');
        assert_eq!(stand_width(&stand("x", (0.0, 0.0), &["heavy"], None)), 'E');
    }

    /// ULKK (the hour sweep, 2026-10-09): a row of stands the network never
    /// reaches; the way from each onto it runs straight down the row through
    /// the others, so they are parked on and never taxied to or from. The
    /// one nearest the network keeps its own way.
    #[test]
    fn a_stand_whose_way_out_runs_through_its_neighbours_is_only_parked_on() {
        // Straight north of the apron node (36.0022, -5.0), 33 m apart.
        let row: Vec<Stand> = (0..4)
            .map(|k| {
                let at = (36.0040 - 0.0003 * k as f64, -5.0);
                stand(&format!("Row {k}"), at, &["props"], Some('A'))
            })
            .collect();
        let f = xtst_with(row);
        for k in 0..3 {
            let name = format!("Row {k}");
            assert!(
                f.parked_only.contains(&name),
                "{name} joins through the row"
            );
            assert!(!f.takes(f.stand(&name).unwrap(), "C172"));
        }
        assert!(
            !f.parked_only.contains("Row 3"),
            "the nearest has its own way"
        );
        assert_eq!(
            f.free_stand((36.0040, -5.0), &[], "C172")
                .map(|s| s.name.as_str()),
            Some("Row 3")
        );
    }

    /// BIRK 13 (the hour sweep, 2026-10-09): an exit running on into a dead
    /// end ringed by tie-downs. The vacate stops short of them, on its own
    /// path, off the runway's side.
    #[test]
    fn a_vacate_stops_short_of_the_stands_its_exit_runs_into() {
        let x = fixtures::xtst();
        let e09 = x.end("09").unwrap();
        let roll = e09.at(500.0, 0.0);
        let open = xtst_with(Vec::new());
        let whole = open
            .vacate(roll, e09.course_deg, &e09, "C172", &[])
            .expect("a vacate");
        let end = *whole.path.last().unwrap();
        let ring: Vec<Stand> = [(0.0001, 0.0), (0.0, 0.00012), (0.0, -0.00012)]
            .iter()
            .enumerate()
            .map(|(k, d)| {
                stand(
                    &format!("Ring {k}"),
                    (end.0 + d.0, end.1 + d.1),
                    &["props"],
                    Some('A'),
                )
            })
            .collect();
        let f = xtst_with(ring);
        let v = f
            .vacate(roll, e09.course_deg, &e09, "C172", &[])
            .expect("a vacate");
        let off = *v.path.last().unwrap();
        assert!(v.path.len() < whole.path.len(), "cut short");
        assert_eq!(v.seg.len(), v.path.len() - 1);
        assert!(whole.path.starts_with(&v.path), "along the same way");
        for s in &f.net.stands {
            assert!(
                dist_m(off, (s.lat, s.lon)) >= VACATE_STAND_CLEAR_M,
                "{}",
                s.name
            );
        }
        assert!(!f.beside_runway(off), "and off the runway's side");
    }

    /// ENVA (the hour sweep, 2026-10-09): sized to fit, the airliners all
    /// took its eleven code C gates and every departure left past the one
    /// beside it. An arrival takes a stand with nobody beside it while one
    /// of its size is free.
    #[test]
    fn an_arrival_parks_away_from_the_aircraft_already_parked() {
        let gates: Vec<Stand> = (0..3)
            .map(|k| {
                let at = (36.0026, -5.0012 + 0.00045 * k as f64);
                stand(&format!("Gate {k}"), at, &["jets"], Some('C'))
            })
            .collect();
        let f = xtst_with(gates.clone());
        let parked = [(gates[0].lat, gates[0].lon)];
        assert!(
            neighbour_taken(&gates[1], &parked),
            "40 m apart is beside it"
        );
        assert!(!neighbour_taken(&gates[2], &parked));
        let from = (gates[1].lat, gates[1].lon);
        assert_eq!(
            f.free_stand(from, &parked, "A320").map(|s| s.name.as_str()),
            Some("Gate 2"),
            "not the nearer one beside the parked aircraft"
        );
        let both = [parked[0], (gates[2].lat, gates[2].lon)];
        assert_eq!(
            f.free_stand(from, &both, "A320").map(|s| s.name.as_str()),
            Some("Gate 1"),
            "beside somebody when that is all there is"
        );
    }
}
