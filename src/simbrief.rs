//! SimBrief flight-plan (OFP) fetch and summary.
//!
//! This is the one outbound network call besides X-Plane: on demand it pulls the
//! user's *latest generated* operational flight plan from SimBrief by their
//! numeric ID, and distils it to the few figures a copilot would brief — route,
//! cruise altitude, distance, time enroute, and planned fuel. Best-effort: if the
//! user hasn't generated a plan, or is offline, the caller reports that and the
//! copilot carries on.
//!
//! Fuel is normalised to US gallons (the GA unit the rest of the app speaks), from
//! whatever unit the plan was generated in (lbs or kg).

use anyhow::{anyhow, Context, Result};
use serde_json::Value;

const AVGAS_LB_PER_GAL: f64 = 6.0;
const AVGAS_KG_PER_GAL: f64 = 2.72;

/// The briefed essentials of a SimBrief OFP. Fuel figures are in US gallons.
pub struct FlightPlan {
    pub origin: String,      // ICAO
    pub destination: String, // ICAO
    pub alternate: String,   // ICAO (may be empty)
    pub aircraft: String,    // ICAO type
    pub cruise_alt_ft: f64,
    pub distance_nm: f64,
    pub ete_min: f64,
    pub block_fuel_gal: f64, // plan_ramp (total fuel loaded)
    pub trip_fuel_gal: f64,  // enroute burn
    pub reserve_gal: f64,
    pub route: String,
    /// Average wind component, knots: negative = headwind, positive = tailwind.
    pub wind_comp_kts: f64,
    pub wind_dir: f64,
    pub wind_spd_kts: f64,
    /// Distance from top-of-descent to destination, nm (0 if not in the navlog).
    pub tod_to_dest_nm: f64,
    /// Enroute waypoints (named fixes + destination), for loading into the GPS/FMS.
    pub waypoints: Vec<Waypoint>,
}

/// A route waypoint for the FMS: an ident plus position and planned altitude.
#[derive(Clone)]
pub struct Waypoint {
    pub ident: String,
    pub lat: f64,
    pub lon: f64,
    pub alt: f64,
}

/// Fetch and parse the user's latest SimBrief OFP. `Err` if the id is empty, the
/// request fails, or SimBrief has no plan for the user.
pub async fn fetch(user_id: &str) -> Result<FlightPlan> {
    let user_id = user_id.trim();
    if user_id.is_empty() {
        return Err(anyhow!("no SimBrief id set"));
    }
    let url = format!("https://www.simbrief.com/api/xml.fetcher.php?userid={user_id}&json=1");
    let v: Value = reqwest::Client::new()
        .get(&url)
        .send()
        .await
        .with_context(|| format!("GET {url}"))?
        .error_for_status()?
        .json()
        .await
        .context("decoding SimBrief JSON")?;

    // The fetcher reports a status; anything but Success means no usable plan.
    if let Some(status) = v.pointer("/fetch/status").and_then(Value::as_str) {
        if !status.eq_ignore_ascii_case("Success") {
            return Err(anyhow!("SimBrief returned: {status}"));
        }
    }

    // Fuel values are in the plan's unit (lbs by default, sometimes kgs).
    let unit = v.pointer("/params/units").and_then(Value::as_str).unwrap_or("lbs").to_ascii_lowercase();
    let to_gal = move |raw: f64| {
        if unit.starts_with("kg") { raw / AVGAS_KG_PER_GAL } else { raw / AVGAS_LB_PER_GAL }
    };
    let s = |path: &str| v.pointer(path).and_then(Value::as_str).unwrap_or("").to_string();
    let n = |path: &str| v.pointer(path).and_then(num_of).unwrap_or(0.0);

    let route_dist = n("/general/route_distance");
    let origin = s("/origin/icao_code");
    let destination = s("/destination/icao_code");

    // Enroute fixes, then bookended by the airports so the FMS plan reads
    // origin → ... → destination. The navlog usually ends at the destination
    // airport already; the origin is missing (the route starts at the first SID
    // fix), so prepend it.
    let mut waypoints = parse_waypoints(&v);
    let o_lat = n("/origin/pos_lat");
    let o_lon = n("/origin/pos_long");
    if !origin.is_empty() && (o_lat != 0.0 || o_lon != 0.0) {
        waypoints.insert(0, Waypoint { ident: origin.clone(), lat: o_lat, lon: o_lon, alt: 0.0 });
    }
    let d_lat = n("/destination/pos_lat");
    let d_lon = n("/destination/pos_long");
    if !destination.is_empty()
        && waypoints.last().map(|w| w.ident.as_str()) != Some(destination.as_str())
        && (d_lat != 0.0 || d_lon != 0.0)
    {
        waypoints.push(Waypoint { ident: destination.clone(), lat: d_lat, lon: d_lon, alt: 0.0 });
    }

    Ok(FlightPlan {
        origin,
        destination,
        alternate: s("/alternate/icao_code"),
        aircraft: s("/aircraft/icaocode"),
        cruise_alt_ft: n("/general/initial_altitude"),
        distance_nm: if route_dist > 0.0 { route_dist } else { n("/general/air_distance") },
        ete_min: n("/times/est_time_enroute") / 60.0,
        block_fuel_gal: to_gal(n("/fuel/plan_ramp")),
        trip_fuel_gal: to_gal(n("/fuel/enroute_burn")),
        reserve_gal: to_gal(n("/fuel/reserve")),
        route: s("/general/route"),
        wind_comp_kts: n("/general/avg_wind_comp"),
        wind_dir: n("/general/avg_wind_dir"),
        wind_spd_kts: n("/general/avg_wind_spd"),
        tod_to_dest_nm: navlog_tod_to_dest(&v),
        waypoints,
    })
}

impl FlightPlan {
    /// A spoken-friendly one-line brief.
    pub fn summary(&self) -> String {
        let h = (self.ete_min / 60.0).floor() as i32;
        let m = (self.ete_min % 60.0).round() as i32;
        let altn = if self.alternate.is_empty() {
            String::new()
        } else {
            format!(", alternate {}", self.alternate)
        };
        let acft = if self.aircraft.is_empty() { String::new() } else { format!("{}, ", self.aircraft) };
        format!(
            "Flight plan: {}{} to {}{}. Cruise {:.0} feet, {:.0} nautical miles, about {} hours {} minutes enroute{}. \
             Block fuel {:.0} gallons, {:.0} for the trip, {:.0} reserve.",
            acft, self.origin, self.destination, altn, self.cruise_alt_ft, self.distance_nm,
            h, m, self.wind_phrase(), self.block_fuel_gal, self.trip_fuel_gal, self.reserve_gal,
        )
    }

    /// ", wind 184 at 13, 10 knot headwind" / "" when calm.
    pub fn wind_phrase(&self) -> String {
        if self.wind_spd_kts < 1.0 {
            return String::new();
        }
        let comp = if self.wind_comp_kts <= -3.0 {
            format!(", {:.0} knot headwind", self.wind_comp_kts.abs())
        } else if self.wind_comp_kts >= 3.0 {
            format!(", {:.0} knot tailwind", self.wind_comp_kts)
        } else {
            String::new()
        };
        format!(", wind {:03.0} at {:.0}{}", self.wind_dir, self.wind_spd_kts, comp)
    }
}

/// Real route waypoints from the navlog (named fixes + the destination airport),
/// skipping the pseudo points (top of climb/descent). Capped so the serialised
/// route stays within the plugin's fixed buffer — ample for a GA route.
fn parse_waypoints(v: &Value) -> Vec<Waypoint> {
    const MAX_WAYPOINTS: usize = 14;
    let Some(fixes) = v.pointer("/navlog/fix").and_then(Value::as_array) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for f in fixes {
        let ident = f.get("ident").and_then(Value::as_str).unwrap_or("");
        if ident.is_empty() || ident == "TOC" || ident == "TOD" {
            continue;
        }
        let (Some(lat), Some(lon)) =
            (f.get("pos_lat").and_then(num_of), f.get("pos_long").and_then(num_of))
        else {
            continue;
        };
        out.push(Waypoint {
            ident: ident.to_string(),
            lat,
            lon,
            alt: f.get("altitude_feet").and_then(num_of).unwrap_or(0.0),
        });
        if out.len() >= MAX_WAYPOINTS {
            break;
        }
    }
    out
}

/// Distance (nm) from the top-of-descent fix to the destination: the sum of the
/// leg distances *after* the TOD fix in the navlog. 0 if there's no TOD fix.
fn navlog_tod_to_dest(v: &Value) -> f64 {
    let Some(fixes) = v.pointer("/navlog/fix").and_then(Value::as_array) else {
        return 0.0;
    };
    let mut after_tod = false;
    let mut sum = 0.0;
    for f in fixes {
        if after_tod {
            sum += f.get("distance").and_then(num_of).unwrap_or(0.0);
        } else if f.get("ident").and_then(Value::as_str) == Some("TOD") {
            after_tod = true;
        }
    }
    sum
}

/// A JSON value as `f64`, whether a number or a numeric string (SimBrief sends
/// most figures as strings).
fn num_of(v: &Value) -> Option<f64> {
    v.as_f64().or_else(|| v.as_str().and_then(|s| s.trim().parse().ok()))
}
