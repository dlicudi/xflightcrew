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
    Ok(FlightPlan {
        origin: s("/origin/icao_code"),
        destination: s("/destination/icao_code"),
        alternate: s("/alternate/icao_code"),
        aircraft: s("/aircraft/icaocode"),
        cruise_alt_ft: n("/general/initial_altitude"),
        distance_nm: if route_dist > 0.0 { route_dist } else { n("/general/air_distance") },
        ete_min: n("/times/est_time_enroute") / 60.0,
        block_fuel_gal: to_gal(n("/fuel/plan_ramp")),
        trip_fuel_gal: to_gal(n("/fuel/enroute_burn")),
        reserve_gal: to_gal(n("/fuel/reserve")),
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
            "Flight plan: {}{} to {}{}. Cruise {:.0} feet, {:.0} nautical miles, about {} hours {} minutes enroute. \
             Block fuel {:.0} gallons, {:.0} for the trip, {:.0} reserve.",
            acft, self.origin, self.destination, altn, self.cruise_alt_ft, self.distance_nm,
            h, m, self.block_fuel_gal, self.trip_fuel_gal, self.reserve_gal,
        )
    }
}

/// A JSON value as `f64`, whether a number or a numeric string (SimBrief sends
/// most figures as strings).
fn num_of(v: &Value) -> Option<f64> {
    v.as_f64().or_else(|| v.as_str().and_then(|s| s.trim().parse().ok()))
}
