//! The arrival sequence (2026-10-14, `docs/tasks/2026-10-14-arrival-sequencing.md`):
//! our own arrival as the station has armed it, for the traffic link to put
//! our slot in the traffic's picture.

use super::*;

/// Our arrival as the station has it armed: the runway END we land on,
/// where the vectors (or approach's own-navigation clearance) join the
/// final, and whether the tower has cleared us to land. Read by the traffic
/// link ([`crate::party_line::own_arrival_plan`]), which works out our slot
/// at the runway from it.
#[derive(Debug, Clone, PartialEq)]
pub struct OwnArrivalPlan {
    /// The landing END ("13").
    pub runway_ident: String,
    /// Where the final is joined, nm from the threshold, while a radar
    /// controller is bringing us onto it.
    pub join_nm: Option<f64>,
    /// Cleared to land.
    pub cleared: bool,
}

/// The runway end whose threshold is at `(lat, lon)`, of the installed
/// field's runways (a vectors plan and an approach watch carry the
/// threshold, not the ident).
fn end_at(runways: &[nav::Runway], lat: f64, lon: f64) -> Option<String> {
    runways.iter().find_map(|r| {
        if nav::haversine_nm(lat, lon, r.lat_a, r.lon_a) < 0.1 {
            Some(r.ident_a.clone())
        } else if nav::haversine_nm(lat, lon, r.lat_b, r.lon_b) < 0.1 {
            Some(r.ident_b.clone())
        } else {
            None
        }
    })
}

/// Our arrival as armed on our strip: cleared to land, under vectors,
/// watched by approach on its own navigation, a landing clearance withheld,
/// or Inbound to the tower for a runway. `None` with none of them (a crew
/// on a final nobody armed is found by the link's own final wedge).
pub fn own_arrival_plan() -> Option<OwnArrivalPlan> {
    let runways = ACTIVE_RUNWAYS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone();
    let clearance = OWN_STRIP
        .clearance
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|c| (c.state, c.runway_ident.clone()));
    if let Some((tower::State::ClearedToLand, ident)) = &clearance {
        if !ident.is_empty() {
            return Some(OwnArrivalPlan {
                runway_ident: ident.clone(),
                join_nm: None,
                cleared: true,
            });
        }
    }
    let vectored = OWN_STRIP
        .vectors
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .and_then(|p| Some((end_at(&runways, p.thr_lat, p.thr_lon)?, p.join_nm)));
    let watched = || {
        OWN_STRIP
            .approach_watch
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .and_then(|w| Some((end_at(&runways, w.thr_lat, w.thr_lon)?, w.join_nm)))
    };
    if let Some((ident, join)) = vectored.or_else(watched) {
        return Some(OwnArrivalPlan {
            runway_ident: ident,
            join_nm: Some(join),
            cleared: false,
        });
    }
    let held = OWN_STRIP
        .landing_hold
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|h| h.runway_ident.clone());
    let inbound = match clearance {
        Some((tower::State::Inbound, ident)) => Some(ident),
        _ => None,
    };
    held.or(inbound)
        .filter(|i| !i.is_empty())
        .map(|ident| OwnArrivalPlan {
            runway_ident: ident,
            join_nm: None,
            cleared: false,
        })
}
