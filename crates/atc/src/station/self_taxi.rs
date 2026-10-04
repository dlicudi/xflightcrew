//! Uncontrolled departures: the crew's own taxi.

use super::*;

/// A planned runway is kept for an uncontrolled departure unless the wind
/// puts more than this on its tail.
pub const SELF_TAXI_TAILWIND_KTS: f64 = 5.0;

/// The field the aircraft is on: the nearest by pavement within
/// [`AT_FIELD_NM`], any field (not only towered ones). A full apt.dat pass.
pub(super) fn field_at(apt_path: &str, lat: f64, lon: f64) -> Option<(String, nav::AirportInfo)> {
    let mut best: Option<(f64, String, nav::AirportInfo)> = None;
    let _ = nav::scan_airports(apt_path, |icao, apt| {
        let Some(d) = apt.ground_distance_nm(lat, lon) else {
            return;
        };
        if d <= AT_FIELD_NM && best.as_ref().is_none_or(|(b, ..)| d < *b) {
            best = Some((d, icao.to_string(), apt));
        }
    });
    best.map(|(_, icao, apt)| (icao, apt))
}

/// The published [`TAXI_ROUTE`] is the crew's own (an uncontrolled field),
/// not a controller's, to this runway end. Cleared by any controller's
/// route. It stands where a controller's cleared runway would for the
/// line-up: with none, the line-up took the nearest end and lined the
/// Evolution up on LILB 21 for a route planned to 03 (2026-09-25).
pub(super) static SELF_TAXI: Mutex<Option<String>> = Mutex::new(None);

/// The runway of the crew's own route, when there is one.
pub(crate) fn self_taxi_runway() -> Option<String> {
    SELF_TAXI.lock().unwrap_or_else(|e| e.into_inner()).clone()
}

/// A name a pilot could say: a taxiway designator ("A", "B2", "TL").
/// An uncontrolled field's apt.dat names are whatever its author typed
/// ("Taxy", "21_hold" at LILB), and no controller turns them into words.
pub(super) fn designator(name: &str) -> bool {
    let n = name.trim();
    let letters = n.chars().take_while(|c| c.is_ascii_alphabetic()).count();
    let digits = n.chars().skip(letters).take_while(|c| c.is_ascii_digit()).count();
    (1..=2).contains(&letters) && digits <= 2 && letters + digits == n.len()
}

/// An uncontrolled departure's route, as the crew announces it.
#[derive(Debug, Clone, PartialEq)]
pub struct SelfTaxi {
    pub field: String,
    pub runway: String,
    /// Spoken taxiway names, in order ("Alpha, Bravo").
    pub via: String,
    /// Runways crossed on the way, spoken idents.
    pub crossings: Vec<String>,
}

/// The crew's own route to parking at an uncontrolled arrival field.
#[derive(Debug, Clone, PartialEq)]
pub struct SelfTaxiStand {
    pub field: String,
    pub stand: String,
    pub via: String,
}

/// Plan and publish a stand route without a controller. This is the arrival
/// mirror of [`plan_self_taxi`]: the field is resolved from the pavement under
/// the aircraft, and the route is published as taxi-to-stand so the ordinary
/// taxi-in drive and securing checks consume it unchanged.
pub fn plan_self_taxi_to_stand(
    apt_path: &str,
    lat: f64,
    lon: f64,
    psi_true: Option<f64>,
) -> Result<SelfTaxiStand, String> {
    let (icao, apt) = field_at(apt_path, lat, lon).ok_or_else(|| {
        "I can't tell which field we're at, so the taxi is yours.".to_string()
    })?;
    let net = nav::find_taxi_net(apt_path, &icao)
        .ok()
        .flatten()
        .ok_or_else(|| format!("{icao} has no taxiways drawn, so the taxi is yours."))?;
    let mut plan = stand_route_on(&net, &apt.runways, lat, lon, psi_true)
        .filter(|p| p.path.len() >= 2)
        .ok_or_else(|| format!("I can't find a taxi route to parking at {icao}."))?;
    let stand = plan.holding_point.clone();
    let raw_via = plan.via.join(", ");
    let named: Vec<String> = plan.via.iter().filter(|n| designator(n)).cloned().collect();
    let via = spoken_via(&named).join(", ");
    plan.seg = plan
        .seg
        .into_iter()
        .map(|(n, rwy)| (if rwy || designator(&n) { n } else { String::new() }, rwy))
        .collect();
    *TAXI_ROUTE.lock().unwrap_or_else(|e| e.into_inner()) = Some(TaxiWatch {
        path: plan.path,
        via: via.clone(),
        seg: plan.seg,
        holding_point: stand.clone(),
        lead_on: plan.lead_on,
    });
    *TAXI_TO_STAND.lock().unwrap_or_else(|e| e.into_inner()) = true;
    *SELF_TAXI.lock().unwrap_or_else(|e| e.into_inner()) = None;
    sim::blackbox::record(
        "atc",
        serde_json::json!({
            "route": "self_taxi_in", "field": icao, "stand": stand, "via": raw_via,
        }),
    );
    Ok(SelfTaxiStand { field: apt.name, stand, via })
}

/// Plan the crew's own taxi at an uncontrolled field (no tower, or none in
/// reach): `icao`'s taxi network from `apt_path` (no ICAO = the field we
/// are at, found by a scan), to the holding point of `runway` (the plan's
/// departure runway) unless the wind puts more than [`SELF_TAXI_TAILWIND_KTS`]
/// on its tail, else the end the wind favours, else the longest main runway. Published to the party line like a tower's
/// route, so the taxi drive, the line-up's lead-on and the guide read one
/// route. `Err` says why, in words the crew can say. A full apt.dat pass:
/// call it off the realtime loop.
pub fn plan_self_taxi(
    apt_path: &str,
    icao: Option<&str>,
    lat: f64,
    lon: f64,
    runway: Option<&str>,
    wind: Option<(f64, f64)>,
) -> Result<SelfTaxi, String> {
    let found = match icao {
        Some(icao) => nav::find_airport(apt_path, icao)
            .ok()
            .flatten()
            .map(|a| (icao.to_string(), a))
            .ok_or_else(|| format!("I can't find {icao} in the scenery data."))?,
        None => field_at(apt_path, lat, lon).ok_or_else(|| {
            "I can't tell which field we're at, so the taxi is yours.".to_string()
        })?,
    };
    let (icao, apt) = (found.0.as_str(), found.1);
    let dist = apt
        .ground_distance_nm(lat, lon)
        .or_else(|| apt.position().map(|(a, b)| crate::nav::haversine_nm(lat, lon, a, b)))
        .unwrap_or(f64::MAX);
    if dist > AT_FIELD_NM {
        return Err(format!("We're not at {icao}, so I can't plan its taxi."));
    }
    let net = nav::find_taxi_net(apt_path, icao)
        .ok()
        .flatten()
        .ok_or_else(|| format!("{icao} has no taxiways drawn, so the taxi is yours."))?;
    let tail_ok = |r: &str| {
        wind.and_then(|(from, kts)| nav::headwind_for(&apt.runways, r, from, kts))
            .is_none_or(|hw| hw >= -SELF_TAXI_TAILWIND_KTS)
    };
    let asked = runway
        .filter(|r| strip(&apt.runways, r).is_some() && tail_ok(r))
        .map(str::to_string);
    let ident = asked
        .or_else(|| {
            wind.filter(|(_, kts)| *kts >= 1.0)
                .and_then(|(from, kts)| nav::best_runway_for_wind(&apt.runways, from, kts))
                .map(|(id, _, _)| id)
        })
        .or_else(|| {
            nav::assignable_runways(&apt.runways)
                .max_by(|a, b| a.length_m().total_cmp(&b.length_m()))
                .map(|r| r.ident_a.clone())
        })
        .ok_or_else(|| format!("{icao} has no runway I can pick."))?;
    let r = strip(&apt.runways, &ident).ok_or_else(|| format!("No runway {ident} at {icao}."))?;
    let (tlat, tlon) = if r.ident_a == ident {
        (r.lat_a, r.lon_a)
    } else {
        (r.lat_b, r.lon_b)
    };
    let standoff = (r.width_m / 2.0 + 30.0).max(nav::HOLD_SHORT_STANDOFF_M);
    let zone = [r.ident_a.clone(), r.ident_b.clone()];
    let mut plan = net
        .plan_to_runway(lat, lon, tlat, tlon, standoff, &zone, Some(r))
        .filter(|p| p.path.len() >= 2)
        .ok_or_else(|| {
            format!(
                "I can't find a taxi route to runway {} from here.",
                atis::spoken_runway(&ident)
            )
        })?;
    mark_runway_crossings(&mut plan, &apt.runways);
    // Only names a pilot could say ride the route: the drive, the guide
    // and the announcements read them.
    let raw_via = plan.via.join(", ");
    let named: Vec<String> = plan.via.iter().filter(|n| designator(n)).cloned().collect();
    let via = spoken_via(&named).join(", ");
    let seg = plan
        .seg
        .iter()
        .map(|(n, rwy)| (if *rwy || designator(n) { n.clone() } else { String::new() }, *rwy))
        .collect();
    let holding_point = if designator(&plan.holding_point) {
        plan.holding_point.clone()
    } else {
        String::new()
    };
    *TAXI_ROUTE.lock().unwrap_or_else(|e| e.into_inner()) = Some(TaxiWatch {
        path: plan.path.clone(),
        via: via.clone(),
        seg,
        holding_point,
        lead_on: plan.lead_on.clone(),
    });
    *TAXI_TO_STAND.lock().unwrap_or_else(|e| e.into_inner()) = false;
    *SELF_TAXI.lock().unwrap_or_else(|e| e.into_inner()) = Some(ident.clone());
    sim::blackbox::record(
        "atc",
        serde_json::json!({ "route": "self_taxi", "field": icao, "runway": ident, "via": raw_via }),
    );
    Ok(SelfTaxi {
        field: apt.name.clone(),
        runway: ident,
        via,
        crossings: plan.crossings.iter().map(|c| atis::spoken_runway(c)).collect(),
    })
}
