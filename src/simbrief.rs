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
    *SHELF_AT.lock().unwrap_or_else(|e| e.into_inner()) = Some(std::time::SystemTime::now());
}
/// When the plan on the shelf was put there (the EFB's "fetched at"): set
/// with it, so it lives exactly as long as the shelf's plan.
static SHELF_AT: std::sync::Mutex<Option<std::time::SystemTime>> = std::sync::Mutex::new(None);
pub fn plan_shelved_at() -> Option<std::time::SystemTime> {
    *SHELF_AT.lock().unwrap_or_else(|e| e.into_inner())
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

/// The Route tab's load sheet feed (`xcopilot/status/loadsheet`): the plan's
/// side only, the plugin reads the aircraft's side itself every frame.
/// "ORIGIN|DEST|block_kg|payload_kg|max_tow_kg|pax|unit", kilos throughout,
/// `unit` the plan's spoken one (lbs or kgs); a figure the plan lacks is 0.
/// Empty with no plan on the shelf.
pub fn loadsheet_payload(plan: Option<&FlightPlan>) -> String {
    let Some(fp) = plan else {
        return String::new();
    };
    let w = fp.weights.as_ref();
    format!(
        "{}|{}|{:.0}|{:.0}|{:.0}|{}|{}",
        fp.origin,
        fp.destination,
        fp.block_fuel_kg,
        w.map_or(0.0, |w| w.payload_kg),
        w.map_or(0.0, |w| w.max_tow_kg),
        w.map_or(0, |w| w.pax),
        if w.is_some_and(|w| w.spoken_lbs) {
            "lbs"
        } else {
            "kgs"
        },
    )
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
    /// The ORIGIN's transition altitude and the DESTINATION's transition
    /// level, ft, when the plan carries them (`origin/trans_alt`,
    /// `destination/trans_level`). The profile's figure is an airframe's
    /// guess at a fact about airspace: the 737's US 18,000 flew Edinburgh to
    /// Exeter at "one seven thousand" on QNH, the monitor resetting the knob
    /// to each new QNH all the way down England (2026-09-21).
    pub transition_altitude_ft: Option<f64>,
    pub transition_level_ft: Option<f64>,
    /// The departure and arrival procedures and runways the plan was built
    /// for (OFP `general/sid_ident`, `general/star_ident`, `origin/plan_rwy`,
    /// `destination/plan_rwy`). Empty on a local plan.
    pub procedures: PlanProcedures,
    /// The OFP's ATC callsign ("BAW306"; `atc/callsign`, else
    /// `general/icao_airline` + `general/flight_number`), the tower's and the
    /// crew's name for us while this plan is loaded and fits the airframe
    /// (`atc::callsign::resolve`). `None` on a local plan.
    pub callsign: Option<String>,
    /// The OFP's speed schedule and planned step climbs (navlog `stage`,
    /// `ind_airspeed`, `mach_thousandths`; `general/costindex`,
    /// `general/stepclimb_string`). Empty on a local plan: the follower then
    /// flies the profile's `reference.cruise_ias_kts` as before.
    pub schedule: CruiseSchedule,
}

/// Where a navlog fix sits on the plan's profile (the OFP's `stage`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stage {
    Climb,
    Cruise,
    Descent,
}

/// One navlog fix's planned speeds: indicated airspeed and Mach at that fix,
/// at its planned altitude, as the cost index made them.
#[derive(Clone, Debug, PartialEq)]
pub struct NavlogSpeed {
    pub ident: String,
    pub lat: f64,
    pub lon: f64,
    pub stage: Stage,
    pub alt_ft: f64,
    pub ias_kts: Option<f64>,
    pub mach: Option<f64>,
}

/// A planned level from `general/stepclimb_string` ("EGKK/0180",
/// "EIDW/0340 ERNAN/0360"): from `fix` on, fly `alt_ft`. The first entry is
/// the origin and the initial cruise level; every later one is a step.
#[derive(Clone, Debug, PartialEq)]
pub struct StepClimb {
    pub fix: String,
    pub alt_ft: f64,
}

/// The OFP's cruise schedule: the per-fix speeds its cost index planned and
/// the levels it planned to step through. Before this the follower flew the
/// airframe's one `cruise_ias_kts` at every level (the 737's 255, chosen to
/// stay under Mmo at FL360, is 45 kt under the OFP's 300 at FL180) and
/// climbed straight to the plan's HIGHEST level from the runway, so a
/// planned step was flown as the initial cruise (#101).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CruiseSchedule {
    /// `general/costindex`, for the record (the speeds already carry it).
    pub cost_index: Option<f64>,
    /// `general/cruise_profile` ("CI 77", "LRC", "M78").
    pub profile: Option<String>,
    /// Every navlog fix with a stage, TOC and TOD excluded.
    pub fixes: Vec<NavlogSpeed>,
    /// The planned levels, the initial one first. One entry = no step.
    pub steps: Vec<StepClimb>,
    /// The navlog's own top of descent (its `TOD` entry), lat and lon: where
    /// the plan leaves its cruise. `tod_to_dest_nm` is the navlog's miles
    /// from it along the ARRIVAL as flown (EGKK-EGBB: 61 against 42 of
    /// direct legs), so a point before it is measured from its position.
    pub tod: Option<(f64, f64)>,
}

/// The navlog fixes' stages and planned speeds. A fix without a stage is
/// left out (an old or foreign OFP): no schedule, the airframe's speed.
fn parse_schedule_fixes(v: &Value) -> Vec<NavlogSpeed> {
    let Some(fixes) = v.pointer("/navlog/fix").and_then(Value::as_array) else {
        return Vec::new();
    };
    fixes
        .iter()
        .filter_map(|f| {
            let ident = f.get("ident").and_then(Value::as_str).unwrap_or("");
            if ident.is_empty() || ident == "TOC" || ident == "TOD" {
                return None;
            }
            let stage = match f.get("stage").and_then(Value::as_str)?.trim() {
                "CLB" => Stage::Climb,
                "CRZ" => Stage::Cruise,
                "DSC" | "DES" => Stage::Descent,
                _ => return None,
            };
            let lat = f.get("pos_lat").and_then(num_of)?;
            let lon = f.get("pos_long").and_then(num_of)?;
            let ias_kts = f
                .get("ind_airspeed")
                .and_then(num_of)
                .filter(|k| (40.0..=450.0).contains(k));
            let mach = f
                .get("mach_thousandths")
                .and_then(num_of)
                .or_else(|| f.get("mach").and_then(num_of))
                .filter(|m| (0.05..=0.99).contains(m));
            Some(NavlogSpeed {
                ident: ident.to_string(),
                lat,
                lon,
                stage,
                alt_ft: f.get("altitude_feet").and_then(num_of).unwrap_or(0.0),
                ias_kts,
                mach,
            })
        })
        .collect()
}

/// `general/stepclimb_string`: "IDENT/LLLL" tokens, the level in hundreds of
/// feet ("EGKK/0180" = FL180). A token that does not read is dropped; a
/// level out of any airliner's range too.
pub fn parse_step_climbs(s: &str) -> Vec<StepClimb> {
    s.split_whitespace()
        .filter_map(|tok| {
            let mut parts = tok.split('/');
            let fix = parts.next()?.trim().to_ascii_uppercase();
            let lvl = parts.next()?.trim();
            if fix.is_empty() || !fix.chars().all(|c| c.is_ascii_alphanumeric()) {
                return None;
            }
            let digits: String = lvl.chars().take_while(|c| c.is_ascii_digit()).collect();
            if digits.len() < 3 {
                return None;
            }
            let alt_ft = digits.parse::<f64>().ok()? * 100.0;
            (1_000.0..=60_000.0).contains(&alt_ft).then_some(StepClimb { fix, alt_ft })
        })
        .collect()
}

/// The whole schedule from an OFP.
pub fn parse_schedule(v: &Value) -> CruiseSchedule {
    let text = |p: &str| {
        v.pointer(p)
            .and_then(Value::as_str)
            .map(str::trim)
            .filter(|s| !s.is_empty() && *s != "{}")
            .map(str::to_string)
    };
    CruiseSchedule {
        cost_index: v.pointer("/general/costindex").and_then(num_of),
        profile: text("/general/cruise_profile"),
        fixes: parse_schedule_fixes(v),
        steps: text("/general/stepclimb_string")
            .map(|s| parse_step_climbs(&s))
            .unwrap_or_default(),
        tod: navlog_tod(v),
    }
}

/// See [`FlightPlan::procedures`].
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PlanProcedures {
    pub sid: Option<String>,
    pub star: Option<String>,
    pub origin_rwy: Option<String>,
    pub dest_rwy: Option<String>,
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
#[derive(Clone, Debug, PartialEq)]
pub struct Waypoint {
    pub ident: String,
    pub lat: f64,
    pub lon: f64,
    pub alt: f64,
}

/// The ICAO lat/lon points of a route string, in order, at `alt_ft`:
/// "4610N00944E" (degrees and minutes) or "46N009E" (whole degrees). Any
/// other token (an airway, a named fix, DCT) is skipped: without nav data it
/// has no position.
pub fn latlon_waypoints(route: &str, alt_ft: f64) -> Vec<Waypoint> {
    route
        .split_whitespace()
        .filter_map(|tok| {
            let b = tok.as_bytes();
            let (lat_len, lon_len, minutes) = match b.len() {
                11 => (4, 5, true),
                7 => (2, 3, false),
                _ => return None,
            };
            let (ns, ew) = (b[lat_len] as char, b[lat_len + 1 + lon_len] as char);
            if !matches!(ns, 'N' | 'S') || !matches!(ew, 'E' | 'W') {
                return None;
            }
            let lat_s = &tok[..lat_len];
            let lon_s = &tok[lat_len + 1..lat_len + 1 + lon_len];
            if !lat_s.bytes().chain(lon_s.bytes()).all(|c| c.is_ascii_digit()) {
                return None;
            }
            let deg = |s: &str, d: usize| -> f64 {
                let whole: f64 = s[..d].parse().unwrap_or(0.0);
                let mins: f64 = if minutes { s[d..].parse().unwrap_or(0.0) } else { 0.0 };
                whole + mins / 60.0
            };
            let mut lat = deg(lat_s, 2);
            let mut lon = deg(lon_s, 3);
            if ns == 'S' {
                lat = -lat;
            }
            if ew == 'W' {
                lon = -lon;
            }
            (lat.abs() <= 90.0 && lon.abs() <= 180.0).then(|| Waypoint {
                ident: tok.to_string(),
                lat,
                lon,
                alt: alt_ft,
            })
        })
        .collect()
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

    plan_from_ofp(&v)
}

/// A SimBrief OFP (the fetcher's JSON) made the plan. Pure: the enroute
/// setup's placement is tested on a trimmed real OFP through it.
pub fn plan_from_ofp(v: &Value) -> Result<FlightPlan> {
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
    // SimBrief sometimes keeps a route string it never built into a navlog:
    // LILO-LILB's fourteen lat/lon points came back with an empty navlog and
    // the direct distance, and the helicopter was checked on the direct line
    // over a ridge (2026-09-24). A lat/lon point needs no nav data, so the
    // route is read from the string itself.
    if waypoints.is_empty() {
        waypoints = latlon_waypoints(&s("/general/route"), n("/general/initial_altitude"));
        if !waypoints.is_empty() {
            tracing::warn!(
                n = waypoints.len(),
                "simbrief navlog empty; route read from its lat/lon points"
            );
        }
    }
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
        transition_altitude_ft: Some(n("/origin/trans_alt")).filter(|v| *v >= 1000.0),
        transition_level_ft: Some(n("/destination/trans_level")).filter(|v| *v >= 1000.0),
        procedures: PlanProcedures {
            sid: ident_of(s("/general/sid_ident")),
            star: ident_of(s("/general/star_ident")),
            origin_rwy: ident_of(s("/origin/plan_rwy")),
            dest_rwy: ident_of(s("/destination/plan_rwy")),
        },
        source: PlanSource::SimBrief,
        callsign: plan_callsign(&v),
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
        schedule: parse_schedule(&v),
    })
}

/// The OFP's ATC callsign: `atc/callsign` ("WAT2516" on the pilot's live
/// LFBD-EGKK OFP, 2026-09-27, beside `general/icao_airline` "WAT" and
/// `general/flight_number` "2516"), else airline + flight number. It is the
/// field the pilot's "ATC callsign" box on SimBrief fills, which may differ
/// from the flight number (BA545 flies as BAW5GU). SimBrief writes an empty
/// field as `{}`, which reads as absent here.
fn plan_callsign(v: &Value) -> Option<String> {
    let s = |path: &str| {
        v.pointer(path)
            .and_then(Value::as_str)
            .map(|s| s.trim().to_ascii_uppercase())
            .filter(|s| !s.is_empty())
    };
    s("/atc/callsign").or_else(|| {
        let airline = s("/general/icao_airline")?;
        let number = s("/general/flight_number")?;
        Some(format!("{airline}{number}"))
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

/// The plugin's `xcopilot/fms/route` mailbox is a `TextRef`: 2,048 bytes
/// since 2026-09-25 (`plugin/src/common.h`), and a longer write is truncated
/// at 2,047, mid-entry, silently. So a route too long for it is thinned to
/// fit, and thinned in the MIDDLE: the origin and the destination
/// are the two points that must survive, because they are what fills the CDU's
/// ORIGIN/DEST and what the whole-plan load needs to read as a flight. Dropping
/// from the end instead is what cost the EGPH arrival its last hundred miles.
///
/// A thinned route is NOT free once LNAV flies the FMS: the app kept the old
/// 511-byte cap after the plugin grew, and a 28-fix LXGB-EGKK plan lost
/// YURZI, ERIGA and ARTIV, so the 737's LNAV flew NENEM direct REVTU and
/// `xcopilot/fms/goto` could not find them (probed 2026-10-06). The cap is
/// the mailbox's own now; about 80 fixes fit before anything is thinned.
/// The longest route payload the plugin's mailbox holds whole.
pub const FMS_ROUTE_MAILBOX_BYTES: usize = 2047;

pub fn fms_route_payload(waypoints: &[Waypoint]) -> String {
    const FMS_ROUTE_MAX_BYTES: usize = FMS_ROUTE_MAILBOX_BYTES;
    let entry =
        |w: &Waypoint| format!("{},{:.4},{:.4},{}", fms_ident(&w.ident), w.lat, w.lon, w.alt as i64);
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

/// An ident as the FMS plan file can carry it: uppercase letters and digits
/// only. The plan line is space-separated, so the follower's synthetic
/// "LEMG arrival" leg shifted every field and X-Plane read the altitude as
/// the latitude ("Latitude must be between -Pi/2 and Pi/2", the whole plan
/// refused, LFRG-LEMG 2026-10-05); a comma or pipe would break our own
/// payload the same way. `WPT` when nothing is left. Pure.
pub fn fms_ident(ident: &str) -> String {
    let s: String = ident
        .chars()
        .filter(char::is_ascii_alphanumeric)
        .map(|c| c.to_ascii_uppercase())
        .collect();
    if s.is_empty() {
        "WPT".to_string()
    } else {
        s
    }
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

/// The navlog's `TOD` entry's position.
fn navlog_tod(v: &Value) -> Option<(f64, f64)> {
    let f = v
        .pointer("/navlog/fix")?
        .as_array()?
        .iter()
        .find(|f| f.get("ident").and_then(Value::as_str) == Some("TOD"))?;
    let lat = f.get("pos_lat").and_then(num_of)?;
    let lon = f.get("pos_long").and_then(num_of)?;
    (lat.abs() <= 90.0 && lon.abs() <= 180.0).then_some((lat, lon))
}

/// A JSON value as `f64`, whether a number or a numeric string (SimBrief sends
/// most figures as strings).
fn num_of(v: &Value) -> Option<f64> {
    v.as_f64()
        .or_else(|| v.as_str().and_then(|s| s.trim().parse().ok()))
}

#[cfg(test)]
mod tests {
    /// Every ident reaches the plan file as one token: the follower's
    /// "LEMG arrival" leg made X-Plane refuse the LFRG-LEMG plan
    /// (2026-10-05), its altitude read as the latitude.
    #[test]
    fn an_fms_ident_is_one_plan_file_token() {
        use super::*;
        assert_eq!(fms_ident("LEMG arrival"), "LEMGARRIVAL");
        assert_eq!(fms_ident("mg529"), "MG529");
        assert_eq!(fms_ident("A,B|C"), "ABC");
        assert_eq!(fms_ident("  "), "WPT");
        let w = |ident: &str, lat, lon, alt| Waypoint { ident: ident.into(), lat, lon, alt };
        let p = fms_route_payload(&[
            w("LFRG", 49.3633, 0.16, 0.0),
            w("LEMG arrival", 36.8453, -4.6299, 9600.0),
            w("LEMG", 36.675, -4.4992, 2600.0),
        ]);
        assert_eq!(p, "LFRG,49.3633,0.1600,0|LEMGARRIVAL,36.8453,-4.6299,9600|LEMG,36.6750,-4.4992,2600");
        for e in p.split('|') {
            assert!(!e.contains(' '), "{e}");
            let lat: f64 = e.split(',').nth(1).unwrap().parse().unwrap();
            assert!(lat.abs() <= 90.0, "{e}");
        }
    }

    /// The OFP's ATC callsign, from the shape of the pilot's live OFP
    /// (LFBD-EGKK, 2026-09-27: `atc.callsign` "WAT2516", `general`
    /// "WAT" + "2516"); airline + flight number when the ATC box is empty,
    /// which SimBrief writes as `{}`; none when neither is there.
    #[test]
    fn the_ofp_callsign_is_the_atc_field_else_airline_and_number() {
        let full = serde_json::json!({
            "atc": {"callsign": "BAW5GU", "flight_rules": "I"},
            "general": {"icao_airline": "BAW", "flight_number": "545"},
        });
        assert_eq!(super::plan_callsign(&full).as_deref(), Some("BAW5GU"));
        let live = serde_json::json!({
            "atc": {"callsign": "WAT2516"},
            "general": {"icao_airline": "WAT", "flight_number": "2516"},
        });
        assert_eq!(super::plan_callsign(&live).as_deref(), Some("WAT2516"));
        for no_atc in [
            serde_json::json!({"atc": {"callsign": {}}, "general": {"icao_airline": "BAW", "flight_number": "306"}}),
            serde_json::json!({"atc": {"callsign": " "}, "general": {"icao_airline": "baw", "flight_number": "306"}}),
            serde_json::json!({"general": {"icao_airline": "BAW", "flight_number": "306"}}),
        ] {
            assert_eq!(
                super::plan_callsign(&no_atc).as_deref(),
                Some("BAW306"),
                "{no_atc}"
            );
        }
        for none in [
            serde_json::json!({}),
            serde_json::json!({"general": {"icao_airline": {}, "flight_number": "306"}}),
            serde_json::json!({"general": {"icao_airline": "BAW", "flight_number": ""}}),
        ] {
            assert_eq!(super::plan_callsign(&none), None, "{none}");
        }
    }

    #[test]
    fn a_route_simbrief_never_built_is_read_from_its_lat_lon_points() {
        let w = super::latlon_waypoints(
            "DCT 4610N00944E 4608N00934E UL9 BIG 46N009E 3330S07040W",
            4000.0,
        );
        let got: Vec<(&str, f64, f64)> = w.iter().map(|w| (w.ident.as_str(), w.lat, w.lon)).collect();
        assert_eq!(got.len(), 4);
        assert!((got[0].1 - (46.0 + 10.0 / 60.0)).abs() < 1e-9 && (got[0].2 - (9.0 + 44.0 / 60.0)).abs() < 1e-9);
        assert_eq!((got[2].1, got[2].2), (46.0, 9.0));
        assert!(got[3].1 < -33.4 && got[3].2 < -70.6, "south and west");
        assert!(w.iter().all(|w| w.alt == 4000.0));
    }

    use super::*;

    /// The pilot's real OFP, trimmed (EGKK-EGBB, B738, 2026-09-28): the
    /// cruise schedule is its CI, the CRZ fixes (TOD left out) with their
    /// airspeed and Mach, and a step string of just the initial level.
    #[test]
    fn the_ofp_cruise_schedule_is_read_from_the_navlog() {
        let v: Value =
            serde_json::from_str(include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/ofp/egkk-egbb-738.trimmed.json"))).unwrap();
        let s = parse_schedule(&v);
        assert_eq!(s.cost_index, Some(77.0));
        assert_eq!(s.profile.as_deref(), Some("CI 77"));
        assert_eq!(s.steps, vec![StepClimb { fix: "EGKK".into(), alt_ft: 18_000.0 }]);
        let cruise: Vec<&str> = s
            .fixes
            .iter()
            .filter(|f| f.stage == Stage::Cruise)
            .map(|f| f.ident.as_str())
            .collect();
        assert_eq!(cruise, ["D156O", "D156J", "LAM", "HEMEL", "BUZAD", "WELIN"]);
        let hemel = s.fixes.iter().find(|f| f.ident == "HEMEL").unwrap();
        assert_eq!((hemel.ias_kts, hemel.mach, hemel.alt_ft), (Some(300.0), Some(0.628), 18_000.0));
        assert!(!s.fixes.iter().any(|f| f.ident == "TOC" || f.ident == "TOD"));
        assert_eq!(s.fixes.iter().find(|f| f.ident == "HON").unwrap().stage, Stage::Descent);
        assert_eq!(s.fixes.iter().find(|f| f.ident == "ACORN").unwrap().stage, Stage::Climb);
        // The waypoints the follower flies are the same fixes.
        assert_eq!(parse_waypoints(&v).len(), s.fixes.len());
        // An OFP without the fields: an empty schedule, never a guess.
        assert_eq!(parse_schedule(&serde_json::json!({})), CruiseSchedule::default());
    }

    #[test]
    fn a_step_climb_string_reads_fix_and_level() {
        assert_eq!(
            parse_step_climbs("EIDW/0340 ERNAN/0360  5715N/0380"),
            vec![
                StepClimb { fix: "EIDW".into(), alt_ft: 34_000.0 },
                StepClimb { fix: "ERNAN".into(), alt_ft: 36_000.0 },
                StepClimb { fix: "5715N".into(), alt_ft: 38_000.0 },
            ]
        );
        // Anything that does not read is dropped, not guessed.
        assert_eq!(
            parse_step_climbs("EGKK/0180 BAD 12/ /0350 X/99 HON/0090"),
            vec![
                StepClimb { fix: "EGKK".into(), alt_ft: 18_000.0 },
                StepClimb { fix: "HON".into(), alt_ft: 9_000.0 },
            ]
        );
    }

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

    /// The plugin's mailbox truncates at 2,047 bytes mid-entry. The
    /// route is thinned to fit — from the MIDDLE, so both ends of the flight
    /// survive, which is the whole point of loading a plan at all.
    #[test]
    fn a_long_route_is_thinned_in_the_middle_to_fit_the_mailbox() {
        let ws = parse_waypoints(&navlog(120));
        let payload = fms_route_payload(&ws);
        assert!(payload.len() <= FMS_ROUTE_MAILBOX_BYTES, "{} bytes", payload.len());
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
    /// rule — thin from the middle, cap at the mailbox — lives in
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
            if f.file_name().is_some_and(|n| n == "simbrief.rs")
                || f.components().any(|c| c.as_os_str() == "simbrief")
            {
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

    /// LXGB-EGKK, 2026-10-06: 28 fixes lost three in the middle to the old
    /// 511-byte cap; a route that size, and up to 60 fixes, now goes whole.
    #[test]
    fn a_continental_route_is_sent_whole() {
        for n in [28, 60] {
            let ws = parse_waypoints(&navlog(n));
            let payload = fms_route_payload(&ws);
            assert_eq!(payload.split('|').count(), ws.len(), "{n} fixes");
        }
    }

    /// A route that already fits is sent whole — every fix, in order.
    #[test]
    fn a_short_route_keeps_every_fix() {
        let ws = parse_waypoints(&navlog(8));
        let payload = fms_route_payload(&ws);
        assert!(payload.len() <= FMS_ROUTE_MAILBOX_BYTES, "{} bytes", payload.len());
        assert_eq!(payload.split('|').count(), ws.len());
        assert!(
            payload.starts_with("FIX00,51.5000,-2.5000,32000|"),
            "{payload}"
        );
    }
}

/// An OFP identifier field, or `None` when it is empty or a placeholder
/// (SimBrief writes `{}` for "none": LOWI-LSZS has no STAR).
fn ident_of(raw: String) -> Option<String> {
    let t = raw.trim();
    (!t.is_empty() && t != "{}" && t.chars().all(|c| c.is_ascii_alphanumeric()))
        .then(|| t.to_ascii_uppercase())
}
