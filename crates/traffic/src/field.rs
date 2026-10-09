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
    /// From parsed parts (tests and callers that already hold them).
    pub fn new(icao: &str, airport: AirportInfo, net: Option<TaxiNet>) -> FieldModel {
        FieldModel {
            icao: icao.to_string(),
            airport,
            net: net.unwrap_or_default(),
            procs: Procedures::default(),
        }
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
    /// or when there is no exit ahead at all.
    pub fn vacate(&self, pos: LatLon, hdg: f64, end: &RunwayEnd) -> Option<VacatePlan> {
        let zone = [end.ident.clone(), end.reciprocal.clone()];
        let plan = |h: f64| {
            self.net
                .plan_vacate(pos.0, pos.1, h, &zone, nav::VACATE_CLEAR_M)
                .filter(|v| v.path.len() >= 2)
        };
        let ahead = plan(hdg);
        let Some(stand) = self.net.nearest_stand(pos.0, pos.1) else {
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

    /// The nearest suitable stand to `from` that nobody in `occupied`
    /// stands on (within [`STAND_TAKEN_M`]).
    pub fn free_stand(&self, from: LatLon, occupied: &[LatLon]) -> Option<&Stand> {
        self.net
            .stands
            .iter()
            .filter(|s| s.is_suitable())
            .filter(|s| !stand_taken(s, occupied))
            .min_by(|a, b| dist_m(from, (a.lat, a.lon)).total_cmp(&dist_m(from, (b.lat, b.lon))))
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

/// An aircraft this close to a stand is on it.
pub const STAND_TAKEN_M: f64 = 20.0;

/// Somebody in `occupied` is on `stand`.
/// A backtrack vacate is taken when its drive to the parking is under this
/// share of the forward one's ([`FieldModel::vacate`]): a turn round on the
/// runway is paid for only by a real saving.
pub const BACKTRACK_SAVES: f64 = 0.6;

/// The length of a polyline, metres.
fn path_m(path: &[LatLon]) -> f64 {
    path.windows(2).map(|w| dist_m(w[0], w[1])).sum()
}

pub fn stand_taken(stand: &Stand, occupied: &[LatLon]) -> bool {
    occupied
        .iter()
        .any(|p| dist_m(*p, (stand.lat, stand.lon)) < STAND_TAKEN_M)
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
        let v = f.vacate(here, e03.course_deg, &e03).expect("a vacate");
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
        let v = x.vacate(roll, e09.course_deg, &e09).expect("a vacate");
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
