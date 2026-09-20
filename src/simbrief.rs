//! SimBrief flight-plan (OFP) fetch and summary.
//!
//! This is the one outbound network call besides X-Plane: on demand it pulls the
//! user's *latest generated* operational flight plan from SimBrief by their
//! numeric ID, and distils it to the few figures a copilot would brief — route,
//! cruise altitude, distance, time enroute, and planned fuel. Best-effort: if the
//! user hasn't generated a plan, or is offline, the caller reports that and the
//! copilot carries on.
//!
//! Fuel is normalised to KILOGRAMS — the unit the sim's tanks are read in and
//! the only one both ends of a comparison can share — from whatever unit the
//! plan was generated in (lbs or kg). It is converted to the AIRFRAME's unit
//! at the moment it is spoken ([`crate::aircraft::say_fuel_kg`]).
//!
//! It used to be normalised to US gallons, "the GA unit the rest of the app
//! speaks", at AVGAS density. On a 737 that briefed kerosene 11.8% light and
//! in a unit no airline crew uses: 3,126 kg in the tanks read back as "1,149
//! gallons" while full auto's own preflight called the same fuel 6,891
//! pounds (BIKF-EGPH, 2026-09-20).

use anyhow::{anyhow, Context, Result};
use serde_json::Value;

const LB_PER_KG: f64 = 2.204_62;

/// Where a plan came from. A LOCAL plan (the direct-to planner) is the
/// pilot's own decision and never silently replaced by a SimBrief refetch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlanSource {
    SimBrief,
    Local,
}

/// The speech loop publishes its shelf here for tool-triggered full auto.
/// Only a local plan bypasses the explicit SimBrief fetch at the arm.
static SHELF: std::sync::Mutex<Option<FlightPlan>> = std::sync::Mutex::new(None);
pub fn publish_plan(plan: &FlightPlan) {
    *SHELF.lock().unwrap() = Some(plan.clone());
}
pub fn loaded_plan() -> Option<FlightPlan> {
    SHELF.lock().unwrap().clone()
}
pub fn local_plan() -> Option<FlightPlan> {
    SHELF
        .lock()
        .unwrap()
        .as_ref()
        .filter(|p| p.source == PlanSource::Local)
        .cloned()
}

/// The briefed essentials of a SimBrief OFP. Fuel figures are in KILOGRAMS;
/// speak them through [`crate::aircraft::say_fuel_kg`].
#[derive(Clone)]
pub struct FlightPlan {
    pub source: PlanSource,
    pub origin: String,      // ICAO
    pub destination: String, // ICAO
    pub alternate: String,   // ICAO (may be empty)
    pub aircraft: String,    // ICAO type
    pub cruise_alt_ft: f64,
    pub distance_nm: f64,
    pub ete_min: f64,
    pub trip_fuel_kg: f64, // enroute burn
    pub reserve_kg: f64,
    pub route: String,
    /// Average wind component, knots: negative = headwind, positive = tailwind.
    pub wind_comp_kts: f64,
    pub wind_dir: f64,
    pub wind_spd_kts: f64,
    /// Distance from top-of-descent to destination, nm (0 if not in the navlog).
    pub tod_to_dest_nm: f64,
    /// Enroute waypoints (named fixes + destination), for loading into the GPS/FMS.
    pub waypoints: Vec<Waypoint>,
    /// Block fuel in KILOGRAMS, the sim's own unit, straight from the plan's
    /// figure and never through a fuel density. It used to be the ONLY honest
    /// fuel figure here — the spoken brief had its own set in avgas gallons,
    /// wrong by 11.8% on a jet and in a unit no airline crew uses. They are
    /// all kilos now and the airframe decides how they are said. 0 = unknown.
    pub block_fuel_kg: f64,
    /// The plan's MINIMUM takeoff fuel (trip + reserves, no extra), kg. Below
    /// this the departure is refused; between this and block it is a caution.
    pub min_takeoff_fuel_kg: f64,
    /// The load sheet, when the plan carries one (a local plan does not).
    pub weights: Option<PlanWeights>,
}

/// The OFP's load sheet, in kilograms. SimBrief plans in the user's unit
/// (`params/units`, lbs or kgs); everything here is converted on the way in so
/// it compares against the sim's `weight/m_*` datarefs without a unit in the
/// way, and `spoken_lbs` remembers which unit the pilot planned in.
#[derive(Clone, Debug, PartialEq)]
pub struct PlanWeights {
    pub pax: u32,
    pub bags: u32,
    pub cargo_kg: f64,
    /// Passengers + bags + cargo: what the sim's `m_fixed` should hold.
    pub payload_kg: f64,
    pub est_tow_kg: f64,
    pub max_tow_kg: f64,
    pub est_ldw_kg: f64,
    pub max_ldw_kg: f64,
    /// The plan was generated in pounds; speak weights in pounds.
    pub spoken_lbs: bool,
}

impl PlanWeights {
    /// A weight in the plan's own unit, rounded to the nearest ten.
    pub fn spoken(&self, kg: f64) -> String {
        let (v, unit) = if self.spoken_lbs {
            (kg * LB_PER_KG, "pounds")
        } else {
            (kg, "kilos")
        };
        spoken_weight(v, unit)
    }
}

/// "2,450 pounds": rounded to ten, grouped, so the TTS says two thousand four
/// hundred and fifty and not twenty-four fifty.
pub fn spoken_weight(v: f64, unit: &str) -> String {
    let n = ((v / 10.0).round() * 10.0) as i64;
    let digits = n.abs().to_string();
    let bytes = digits.as_bytes();
    let mut out = String::new();
    for (i, c) in bytes.iter().enumerate() {
        if i > 0 && (bytes.len() - i).is_multiple_of(3) {
            out.push(',');
        }
        out.push(*c as char);
    }
    format!("{out} {unit}")
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
    let unit = v
        .pointer("/params/units")
        .and_then(Value::as_str)
        .unwrap_or("lbs")
        .to_ascii_lowercase();
    let in_kg = unit.starts_with("kg");
    let to_kg = move |raw: f64| if in_kg { raw } else { raw / LB_PER_KG };
    let s = |path: &str| {
        v.pointer(path)
            .and_then(Value::as_str)
            .unwrap_or("")
            .to_string()
    };
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
        waypoints.insert(
            0,
            Waypoint {
                ident: origin.clone(),
                lat: o_lat,
                lon: o_lon,
                alt: 0.0,
            },
        );
    }
    let d_lat = n("/destination/pos_lat");
    let d_lon = n("/destination/pos_long");
    if !destination.is_empty()
        && waypoints.last().map(|w| w.ident.as_str()) != Some(destination.as_str())
        && (d_lat != 0.0 || d_lon != 0.0)
    {
        waypoints.push(Waypoint {
            ident: destination.clone(),
            lat: d_lat,
            lon: d_lon,
            alt: 0.0,
        });
    }

    Ok(FlightPlan {
        source: PlanSource::SimBrief,
        origin,
        destination,
        alternate: s("/alternate/icao_code"),
        aircraft: s("/aircraft/icaocode"),
        cruise_alt_ft: n("/general/initial_altitude"),
        distance_nm: if route_dist > 0.0 {
            route_dist
        } else {
            n("/general/air_distance")
        },
        ete_min: n("/times/est_time_enroute") / 60.0,
        trip_fuel_kg: to_kg(n("/fuel/enroute_burn")),
        reserve_kg: to_kg(n("/fuel/reserve")),
        route: s("/general/route"),
        wind_comp_kts: n("/general/avg_wind_comp"),
        wind_dir: n("/general/avg_wind_dir"),
        wind_spd_kts: n("/general/avg_wind_spd"),
        tod_to_dest_nm: navlog_tod_to_dest(&v),
        waypoints,
        block_fuel_kg: to_kg(n("/fuel/plan_ramp")),
        min_takeoff_fuel_kg: to_kg(n("/fuel/min_takeoff")),
        weights: parse_weights(&v, in_kg),
    })
}

/// The `weights` block, when the plan has one with a takeoff weight in it.
fn parse_weights(v: &Value, in_kg: bool) -> Option<PlanWeights> {
    let to_kg = |raw: f64| if in_kg { raw } else { raw / LB_PER_KG };
    let n = |k: &str| v.pointer(&format!("/weights/{k}")).and_then(num_of);
    let est_tow_kg = to_kg(n("est_tow")?);
    if est_tow_kg <= 0.0 {
        return None;
    }
    Some(PlanWeights {
        pax: n("pax_count_actual")
            .or_else(|| n("pax_count"))
            .unwrap_or(0.0)
            .max(0.0) as u32,
        bags: n("bag_count_actual")
            .or_else(|| n("bag_count"))
            .unwrap_or(0.0)
            .max(0.0) as u32,
        cargo_kg: to_kg(n("cargo").unwrap_or(0.0)),
        payload_kg: to_kg(n("payload").unwrap_or(0.0)),
        est_tow_kg,
        max_tow_kg: to_kg(n("max_tow").unwrap_or(0.0)),
        est_ldw_kg: to_kg(n("est_ldw").unwrap_or(0.0)),
        max_ldw_kg: to_kg(n("max_ldw").unwrap_or(0.0)),
        spoken_lbs: !in_kg,
    })
}

impl FlightPlan {
    /// A spoken-friendly one-line brief.
    pub fn summary(&self) -> String {
        let h = (self.ete_min / 60.0).floor() as i32;
        let m = (self.ete_min % 60.0).round() as i32;
        // "0 hours 24 minutes" is a printf, not a brief (2026-09-15).
        // ...and "1 hours 48 minutes" is a printf too (2026-09-20).
        let ete = match h {
            0 => format!("{m} minutes"),
            1 => format!("1 hour {m} minutes"),
            _ => format!("{h} hours {m} minutes"),
        };
        let altn = if self.alternate.is_empty() {
            String::new()
        } else {
            format!(", alternate {}", self.alternate)
        };
        let acft = if self.aircraft.is_empty() {
            String::new()
        } else {
            format!("{}, ", self.aircraft)
        };
        format!(
            "Flight plan: {}{} to {}{}. Cruise {:.0} feet, {:.0} nautical miles, about {} enroute{}. \
             Block fuel {}, {} for the trip, {} reserve.",
            acft,
            self.origin,
            self.destination,
            altn,
            self.cruise_alt_ft,
            self.distance_nm,
            ete,
            self.wind_phrase(),
            crate::aircraft::say_fuel_kg(self.block_fuel_kg),
            crate::aircraft::fuel_figure_kg(self.trip_fuel_kg),
            crate::aircraft::fuel_figure_kg(self.reserve_kg),
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
        format!(
            ", wind {:03.0} at {:.0}{}",
            self.wind_dir, self.wind_spd_kts, comp
        )
    }
}

/// Real route waypoints from the navlog (named fixes + the destination airport),
/// skipping the pseudo points (top of climb/descent).
///
/// EVERY fix, to the end of the navlog. This was capped at the first 14 so the
/// serialised route would fit the plugin's 512-byte mailbox, and a cap on the
/// front of a list is an amputation of its tail: EGLL-EGPH, 2026-09-17, the
/// B738's plan stopped at RIBEL in the Lake District and the STAR, the arrival
/// and EGPH itself were never in it. The follower flew the 14 fixes it was
/// given, the arrival armed the ILS 24 from 121 nm out, and what the plan did
/// not cover the approach had to. The plugin's buffer is the PAYLOAD's problem
/// and [`fms_route_payload`] owns it; the plan the crew flies is not shaped by
/// the size of a dataref.
fn parse_waypoints(v: &Value) -> Vec<Waypoint> {
    let Some(fixes) = v.pointer("/navlog/fix").and_then(Value::as_array) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for f in fixes {
        let ident = f.get("ident").and_then(Value::as_str).unwrap_or("");
        if ident.is_empty() || ident == "TOC" || ident == "TOD" {
            continue;
        }
        let (Some(lat), Some(lon)) = (
            f.get("pos_lat").and_then(num_of),
            f.get("pos_long").and_then(num_of),
        ) else {
            continue;
        };
        out.push(Waypoint {
            ident: ident.to_string(),
            lat,
            lon,
            alt: f.get("altitude_feet").and_then(num_of).unwrap_or(0.0),
        });
    }
    out
}

/// The plugin's `xcopilot/fms/route` mailbox is a `TextRef`: 512 bytes, and a
/// longer write is truncated at 511 — mid-entry, silently. So the GPS route is
/// thinned to fit, and thinned in the MIDDLE: the origin and the destination
/// are the two points that must survive, because they are what fills the CDU's
/// ORIGIN/DEST and what the whole-plan load needs to read as a flight. Dropping
/// from the end instead is what cost the EGPH arrival its last hundred miles.
///
/// A thinned GPS route costs the crew nothing: the follower flies the PLAN's
/// own fixes, and `xcopilot/fms/goto` moves the pointer for the idents the
/// loaded plan does hold — an ident it lacks changes nothing on the plugin side.
pub fn fms_route_payload(waypoints: &[Waypoint]) -> String {
    const FMS_ROUTE_MAX_BYTES: usize = 511;
    let entry = |w: &Waypoint| format!("{},{:.4},{:.4},{}", w.ident, w.lat, w.lon, w.alt as i64);
    let join = |ws: &[&Waypoint]| ws.iter().map(|w| entry(w)).collect::<Vec<_>>().join("|");
    let mut kept: Vec<&Waypoint> = waypoints.iter().collect();
    while kept.len() > 2 && join(&kept).len() > FMS_ROUTE_MAX_BYTES {
        // Drop the fix nearest the middle, so what goes is enroute padding
        // rather than either end of the flight.
        kept.remove(kept.len() / 2);
    }
    let payload = join(&kept);
    // Two points that still do not fit (absurd idents): the tail is the half
    // that matters, so keep the destination.
    if payload.len() > FMS_ROUTE_MAX_BYTES {
        return kept
            .last()
            .map(|w| entry(w))
            .filter(|p| p.len() <= FMS_ROUTE_MAX_BYTES)
            .unwrap_or_default();
    }
    payload
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
    v.as_f64()
        .or_else(|| v.as_str().and_then(|s| s.trim().parse().ok()))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The load sheet as the fetcher sends it (LXGB-LEAX in the 172,
    /// 2026-09-15: three aboard, 67 lb of cargo, planned at max takeoff
    /// weight). Everything is a string, the unit is the pilot's (lbs), and
    /// the kilo figures must come out right for the sim's `weight/m_*`.
    fn ofp(units: &str) -> Value {
        serde_json::json!({
            "params": { "units": units },
            "fuel": { "plan_ramp": "240", "min_takeoff": "227", "enroute_burn": "64", "reserve": "34" },
            "weights": {
                "oew": "1631", "pax_count": "3", "bag_count": "3",
                "pax_count_actual": "3", "bag_count_actual": "1",
                "cargo": "67", "payload": "592",
                "est_zfw": "2223", "max_zfw": "2450",
                "est_tow": "2450", "max_tow": "2450",
                "est_ldw": "2386", "max_ldw": "2450", "est_ramp": "2463"
            }
        })
    }

    #[test]
    fn the_load_sheet_parses_in_kilos_whatever_the_plan_was_made_in() {
        let w = parse_weights(&ofp("lbs"), false).expect("a load sheet");
        assert_eq!((w.pax, w.bags), (3, 1));
        assert!(
            (w.cargo_kg - 67.0 / LB_PER_KG).abs() < 0.01,
            "{}",
            w.cargo_kg
        );
        assert!((w.payload_kg - 268.5).abs() < 0.5, "{}", w.payload_kg);
        assert!((w.max_tow_kg - 1111.3).abs() < 0.5, "{}", w.max_tow_kg);
        assert!(w.spoken_lbs);
        assert_eq!(w.spoken(w.payload_kg), "590 pounds");

        let w = parse_weights(&ofp("kgs"), true).expect("a load sheet");
        assert_eq!(w.max_tow_kg, 2450.0);
        assert!(!w.spoken_lbs);
        assert_eq!(w.spoken(w.cargo_kg), "70 kilos");

        // No takeoff weight = no sheet, never a sheet of zeros.
        assert!(
            parse_weights(&serde_json::json!({"weights": {"pax_count": "3"}}), false).is_none()
        );
        assert!(parse_weights(&serde_json::json!({}), false).is_none());
    }

    /// An airline navlog, EGLL-EGPH shaped: a SID, the airway fixes, TOC and
    /// TOD in the middle of it, a STAR and the destination at the end.
    fn navlog(n: usize) -> Value {
        let mut fixes = vec![serde_json::json!({
            "ident": "TOC", "pos_lat": "51.9", "pos_long": "-0.6", "altitude_feet": "32000"
        })];
        for i in 0..n {
            fixes.push(serde_json::json!({
                "ident": format!("FIX{i:02}"),
                "pos_lat": format!("{:.4}", 51.5 + i as f64 * 0.2),
                "pos_long": "-2.5000",
                "altitude_feet": "32000",
            }));
        }
        fixes.push(serde_json::json!({
            "ident": "TOD", "pos_lat": "55.2", "pos_long": "-3.0", "altitude_feet": "32000"
        }));
        fixes.push(serde_json::json!({
            "ident": "EGPH", "pos_lat": "55.9500", "pos_long": "-3.3725", "altitude_feet": "0"
        }));
        serde_json::json!({ "navlog": { "fix": fixes } })
    }

    /// The EGPH sortie: the navlog ran well past the fourteenth fix and the
    /// plan stopped there, so the route ended 110 nm short of the field and
    /// the arrival had to cover it. Every fix, and the destination last.
    #[test]
    fn the_navlog_is_parsed_to_its_end() {
        let ws = parse_waypoints(&navlog(22));
        assert_eq!(ws.len(), 23, "22 fixes and the destination, no TOC/TOD");
        assert_eq!(ws.last().map(|w| w.ident.as_str()), Some("EGPH"));
        assert!(!ws.iter().any(|w| w.ident == "TOC" || w.ident == "TOD"));
    }

    /// The plugin's mailbox is 512 bytes and truncates at 511 mid-entry. The
    /// route is thinned to fit — from the MIDDLE, so both ends of the flight
    /// survive, which is the whole point of loading a plan at all.
    #[test]
    fn a_long_route_is_thinned_in_the_middle_to_fit_the_mailbox() {
        let ws = parse_waypoints(&navlog(40));
        let payload = fms_route_payload(&ws);
        assert!(payload.len() <= 511, "{} bytes", payload.len());
        let entries: Vec<&str> = payload.split('|').collect();
        assert!(entries.len() > 2, "not thinned to nothing: {entries:?}");
        assert!(entries[0].starts_with("FIX00,"), "{}", entries[0]);
        assert!(
            entries.last().unwrap().starts_with("EGPH,"),
            "the destination survives: {}",
            entries.last().unwrap()
        );
    }

    /// Nothing in the tree builds an FMS route entry by hand. The mailbox
    /// rule — thin from the middle, cap at 511 — lives in
    /// [`fms_route_payload`] and nowhere else, so a caller cannot reach the
    /// plugin with a payload that has never met it.
    ///
    /// The resume path did exactly that: a byte-identical copy of the entry
    /// format with no thinning and no cap, which overwrote full auto's fitted
    /// payload the moment the pilot resumed (EIDW-BIKF, 2026-09-20 — 528
    /// bytes cut to `BIKF,6`, and X-Plane refused the composed plan for an
    /// "invalid destination airport: ELVUM"). One copy is a bug; the second
    /// copy is this test's job.
    #[test]
    fn the_entry_format_lives_only_in_the_payload_builder() {
        const ENTRY_FMT: &str = r#"{},{:.4},{:.4},{}"#;
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        let mut files = Vec::new();
        fn walk(dir: &std::path::Path, out: &mut Vec<std::path::PathBuf>) {
            for e in std::fs::read_dir(dir).unwrap().flatten() {
                let p = e.path();
                if p.is_dir() {
                    walk(&p, out);
                } else if p.extension().is_some_and(|x| x == "rs") {
                    out.push(p);
                }
            }
        }
        walk(&root, &mut files);
        let mut offenders = Vec::new();
        for f in &files {
            if f.file_name().is_some_and(|n| n == "simbrief.rs") {
                continue;
            }
            let text = std::fs::read_to_string(f).unwrap();
            for (i, line) in text.lines().enumerate() {
                if line.contains(ENTRY_FMT) {
                    offenders.push(format!("{}:{}", f.display(), i + 1));
                }
            }
        }
        assert!(
            offenders.is_empty(),
            "build the route payload with fms_route_payload, not by hand: {offenders:?}"
        );
        assert!(files.len() > 10, "the walk found the tree: {}", files.len());
    }

    /// A route that already fits is sent whole — every fix, in order.
    #[test]
    fn a_short_route_keeps_every_fix() {
        let ws = parse_waypoints(&navlog(8));
        let payload = fms_route_payload(&ws);
        assert!(payload.len() <= 511, "{} bytes", payload.len());
        assert_eq!(payload.split('|').count(), ws.len());
        assert!(
            payload.starts_with("FIX00,51.5000,-2.5000,32000|"),
            "{payload}"
        );
    }
}
