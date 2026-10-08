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
        }
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

    /// The drive off `end`'s strip from a rollout at `pos` heading `hdg`.
    pub fn vacate(&self, pos: LatLon, hdg: f64, end: &RunwayEnd) -> Option<VacatePlan> {
        let zone = [end.ident.clone(), end.reciprocal.clone()];
        self.net
            .plan_vacate(pos.0, pos.1, hdg, &zone, nav::VACATE_CLEAR_M)
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

/// An aircraft this close to a stand is on it.
pub const STAND_TAKEN_M: f64 = 20.0;

/// Somebody in `occupied` is on `stand`.
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
}

#[cfg(test)]
mod tests {
    use super::*;

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
}
