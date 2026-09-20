//! Local flight planning from a spoken destination: "plan a flight to Malaga",
//! "divert to Gibraltar", "take us to lima echo mike golf".
//!
//! Stage one of the local planner: a DIRECT route from the field we are at (or
//! nearest to, when airborne) to the named destination, with a cruise altitude
//! picked by rule, a fuel figure from the profile's cruise burn, and a runway
//! sanity check at the far end. The result is the same [`FlightPlan`] shape
//! SimBrief produces, so everything downstream (the GPS load, top of descent,
//! ETA, runway advice, the monitor's destination) works unchanged.
//!
//! Deterministic throughout: no model, no network. The nav data is X-Plane's
//! own apt.dat; terrain comes from the sim through the plugin's probe. Every
//! rule here is a HYPOTHESIS about what a careful pilot would pick, written so
//! the numbers are testable in isolation.
//!
//! This module never actuates. Loading the plan into the FMS is the pilot's
//! separate word ("load it"), and flying it is the automation's own request
//! grammar; a plan on the shelf authorises nothing.

use anyhow::Result;

use crate::nav;
use crate::phrase::digit_value;
use crate::simbrief::{FlightPlan, PlanSource, Waypoint};

const M_TO_FT: f64 = 3.280_84;

/// Feet of cruise altitude (above the higher field) the leg length "earns":
/// the climb and the descent both have to fit inside the distance. 80 ft/nm
/// puts a 60 nm hop at about 4,500 and a 150 nm leg at 12,000.
const FT_PER_NM: f64 = 80.0;
/// Never plan lower than this above the higher of the two fields.
const MIN_AGL_FT: f64 = 1500.0;
/// Clearance over the highest ground under the leg.
const TERRAIN_CLEARANCE_FT: f64 = 2000.0;
/// Without a terrain answer, assume this much above the fields is clear.
const NO_TERRAIN_MARGIN_FT: f64 = 3000.0;
/// Below the service ceiling by this much.
const CEILING_MARGIN_FT: f64 = 1000.0;
/// Unpressurised cap (the oxygen rule); terrain can push past it.
const OXYGEN_CAP_FT: f64 = 12_500.0;
/// Climb, taxi and pattern allowance on top of the still-air cruise time.
const CLIMB_ALLOWANCE_MIN: f64 = 10.0;
/// Reserve carried on every plan, minutes at cruise burn.
const RESERVE_MIN: f64 = 45.0;
/// Terrain probe points along the leg (the plugin answers at most 24).
const TERRAIN_SAMPLES: usize = 24;

/// Where the pilot wants to go, as spoken. `codes` are ICAO candidates
/// (spelled phonetically, or a single short token); `name` is the words to
/// match against apt.dat airport names. Either may be empty, never both.
#[derive(Debug, Clone, PartialEq)]
pub struct Destination {
    pub codes: Vec<String>,
    pub name: String,
}

impl Destination {
    /// How the pilot said it, for "I don't know an airport called ...".
    pub fn spoken(&self) -> String {
        if !self.name.is_empty() {
            self.name.clone()
        } else {
            self.codes.first().cloned().unwrap_or_default()
        }
    }
}

/// A parsed planning request. `dest` is `None` for a bare "plan a flight" /
/// "divert" (the copilot asks "to where?"); `load` means the pilot also asked
/// for the plan to go into the GPS in the same breath.
#[derive(Debug, Clone, PartialEq)]
pub struct PlanRequest {
    pub dest: Option<Destination>,
    pub load: bool,
}

/// Request shapes. Each ends in "to" so the tail is the destination. Nothing
/// here overlaps the mission grammar ("fly my flight plan" has no "to") or the
/// GPS load ("load the flight plan").
const TRIGGERS: &[&str] = &[
    "plan a flight to",
    "plan the flight to",
    "plan me a flight to",
    "plan us a flight to",
    "plan a route to",
    "plan a trip to",
    "plan flight to",
    "plan for",
    "set up a flight to",
    "set up a route to",
    "setup a flight to",
    "program a flight to",
    "program a route to",
    "make a plan to",
    "make a flight plan to",
    "flight plan to",
    "take us to",
    "take me to",
    "get us to",
    "get me to",
    "route us to",
    "fly us to",
    "fly me to",
    "fly to",
    "lets go to",
    "let's go to",
    "divert to",
    "diversion to",
    "divert us to",
];

/// Bare requests with no destination: worth a "to where?".
const BARE: &[&str] = &[
    "plan a flight",
    "plan the flight",
    "divert",
    "diversion",
    "plan a route",
];

/// Words that mark the end of the destination in the tail.
const TAIL_STOPS: &[&str] = &[
    "and", "then", "please", "for", "in", "into", "thanks", "thank",
];

/// A tail with any of these is a number/command, never an airport.
const NOT_A_PLACE: &[&str] = &[
    "feet",
    "foot",
    "knots",
    "degrees",
    "thousand",
    "hundred",
    "heading",
    "altitude",
    "flight level",
    "runway",
    "waypoint",
    "fix",
    "next",
];

fn load_wanted(t: &str) -> bool {
    has(t, "load") || has(t, "program") || has(t, "send") || t.contains("put it in")
}

fn has(t: &str, w: &str) -> bool {
    t.split_whitespace().any(|x| x == w)
}

/// Lowercase, punctuation to spaces (apostrophes kept), single-spaced.
fn clean(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for c in text.to_lowercase().chars() {
        if c.is_alphanumeric() || c == '\'' {
            out.push(fold(c));
        } else {
            out.push(' ');
        }
    }
    out.split_whitespace().collect::<Vec<_>>().join(" ")
}

/// Fold the accented vowels whisper writes for Spanish/French names to the
/// ASCII apt.dat carries ("Málaga" → "malaga").
fn fold(c: char) -> char {
    match c {
        'á' | 'à' | 'â' | 'ä' | 'ã' | 'å' => 'a',
        'é' | 'è' | 'ê' | 'ë' => 'e',
        'í' | 'ì' | 'î' | 'ï' => 'i',
        'ó' | 'ò' | 'ô' | 'ö' | 'õ' => 'o',
        'ú' | 'ù' | 'û' | 'ü' => 'u',
        'ñ' => 'n',
        'ç' => 'c',
        c => c,
    }
}

/// Parse a planning request out of a transcript. `None` when the words are
/// not one (they fall through to the normal command/question routing).
pub fn parse_request(text: &str) -> Option<PlanRequest> {
    let t = clean(text);
    if t.contains("flight plan") && !TRIGGERS.iter().any(|tr| t.contains(tr)) {
        // "load/fly/brief the flight plan" is the existing SimBrief surface.
        return None;
    }
    let mut best: Option<(usize, &str)> = None;
    for tr in TRIGGERS {
        if let Some(i) = t.find(tr) {
            // Whole-word at both ends: "plan for" must not fire inside "plan format".
            let end = i + tr.len();
            let ok_start = i == 0 || t.as_bytes()[i - 1] == b' ';
            let ok_end = end == t.len() || t.as_bytes()[end] == b' ';
            if ok_start && ok_end && best.is_none_or(|(bi, _)| i < bi) {
                best = Some((i, tr));
            }
        }
    }
    let Some((i, tr)) = best else {
        // Bare "divert" / "plan a flight": ask where.
        let bare = BARE
            .iter()
            .any(|b| t == *b || t.starts_with(&format!("{b} ")));
        return bare.then(|| PlanRequest {
            dest: None,
            load: load_wanted(&t),
        });
    };
    let tail = t[i + tr.len()..].trim();
    let dest = parse_destination(tail)?;
    Some(PlanRequest {
        dest: Some(dest),
        load: load_wanted(&t),
    })
}

/// The destination words alone ("gibraltar", "lima echo mike golf", "LEMG"),
/// as the answer to "to where?". `None` when they are not a place.
pub fn parse_destination(text: &str) -> Option<Destination> {
    let t = clean(text);
    let words: Vec<&str> = t
        .split_whitespace()
        .take_while(|w| !TAIL_STOPS.contains(w))
        .filter(|w| {
            !matches!(
                *w,
                "the" | "a" | "airport" | "airfield" | "field" | "international"
            )
        })
        .collect();
    if words.is_empty() || words.len() > 5 {
        return None;
    }
    if NOT_A_PLACE.iter().any(|n| t.contains(n)) {
        return None;
    }
    // Spelled: every word a phonetic letter, a single letter, or a digit word.
    if let Some(code) = spelled_code(&words) {
        return Some(Destination {
            codes: code_candidates(&code),
            name: String::new(),
        });
    }
    // Digits, written or spoken ("ten"): a value, never a place.
    if words
        .iter()
        .any(|w| w.chars().any(|c| c.is_ascii_digit()) || digit_value(w).is_some() || *w == "ten")
    {
        return None;
    }
    // A single short token is an ICAO written out ("lemg") OR a name ("nice"):
    // try both; the resolver prefers the exact code.
    let name = words.join(" ");
    let codes = if words.len() == 1 && (3..=4).contains(&words[0].len()) {
        code_candidates(&words[0].to_ascii_uppercase())
    } else {
        Vec::new()
    };
    Some(Destination { codes, name })
}

/// ICAO candidates for a spelled code: a 4-character code as is; a 3-letter
/// one also with the US "K" prefix (the FAA identifier a US pilot spells).
fn code_candidates(code: &str) -> Vec<String> {
    match code.len() {
        4 => vec![code.to_string()],
        3 => vec![code.to_string(), format!("K{code}")],
        _ => Vec::new(),
    }
}

/// The spelling alphabet as whisper writes it, plus single letters and digit
/// words. `None` unless EVERY word is one of those (a name never is).
fn spelled_code(words: &[&str]) -> Option<String> {
    let mut code = String::new();
    let mut skip_ray = false;
    for w in words {
        if skip_ray && *w == "ray" {
            skip_ray = false;
            continue;
        }
        skip_ray = false;
        let c = match *w {
            "alpha" | "alfa" => 'A',
            "bravo" => 'B',
            "charlie" => 'C',
            "delta" => 'D',
            "echo" => 'E',
            "foxtrot" => 'F',
            "golf" => 'G',
            "hotel" => 'H',
            "india" => 'I',
            "juliet" | "juliett" => 'J',
            "kilo" => 'K',
            "lima" => 'L',
            "mike" => 'M',
            "november" => 'N',
            "oscar" => 'O',
            "papa" => 'P',
            "quebec" => 'Q',
            "romeo" => 'R',
            "sierra" => 'S',
            "tango" => 'T',
            "uniform" => 'U',
            "victor" => 'V',
            "whiskey" | "whisky" => 'W',
            "xray" => 'X',
            "x" => {
                skip_ray = true;
                'X'
            }
            "yankee" => 'Y',
            "zulu" => 'Z',
            w if w.len() == 1 => w.chars().next()?.to_ascii_uppercase(),
            w => char::from_digit(digit_value(w)? as u32, 10)?,
        };
        code.push(c);
    }
    (3..=4).contains(&code.len()).then_some(code)
}

/// An airport as the planner sees it.
#[derive(Debug, Clone, PartialEq)]
pub struct Airport {
    pub icao: String,
    pub name: String,
    pub lat: f64,
    pub lon: f64,
    pub elev_ft: f64,
    pub longest_runway_ft: f64,
    /// Has communication rows: a real aerodrome, not a farm strip.
    pub towered: bool,
}

impl Airport {
    fn from_info(icao: &str, a: &nav::AirportInfo) -> Option<Airport> {
        let (lat, lon) = a.position()?;
        let longest = longest_runway_ft(a);
        Some(Airport {
            icao: icao.to_string(),
            name: a.name.trim().to_string(),
            lat,
            lon,
            elev_ft: a.elev_ft,
            longest_runway_ft: longest,
            towered: !a.freqs.is_empty(),
        })
    }
}

/// What one apt.dat pass found: the destination (best match) and the field
/// nearest `here` (the origin). `dest_matches` counts the name matches, so
/// the brief can say the code it picked when several fields share a name.
#[derive(Debug, Default)]
pub struct Resolved {
    pub dest: Option<Airport>,
    pub dest_matches: usize,
    pub origin: Option<Airport>,
}

/// Does `name` contain every query word as a whole word?
fn name_matches(query: &str, name: &str) -> bool {
    let name = clean(name);
    let words: Vec<&str> = name.split_whitespace().collect();
    !query.is_empty() && query.split_whitespace().all(|q| words.contains(&q))
}

/// One streaming scan of apt.dat (the ~400 MB file, a moment; call it from a
/// blocking task, never a tick). Land runways only: heliports and seaplane
/// bases have none and are never a destination here.
pub fn resolve(apt_path: &str, dest: &Destination, here: (f64, f64)) -> Result<Resolved> {
    let mut out = Resolved::default();
    let mut best_dist = f64::MAX;
    let mut best_code: Option<Airport> = None;
    let mut best_name: Option<(Airport, (bool, f64))> = None;
    nav::scan_airports(apt_path, |icao, info| {
        let Some(apt) = Airport::from_info(icao, &info) else {
            return;
        };
        let d = nav::haversine_nm(here.0, here.1, apt.lat, apt.lon);
        if d < best_dist {
            best_dist = d;
            out.origin = Some(apt.clone());
        }
        if dest.codes.iter().any(|c| c.eq_ignore_ascii_case(icao)) {
            // Candidates are ordered by preference; the first-listed wins.
            let rank = |a: &Airport| {
                dest.codes
                    .iter()
                    .position(|c| c.eq_ignore_ascii_case(&a.icao))
            };
            if best_code.as_ref().is_none_or(|b| rank(&apt) < rank(b)) {
                best_code = Some(apt.clone());
            }
        }
        if name_matches(&dest.name, &apt.name) {
            out.dest_matches += 1;
            let score = (apt.towered, apt.longest_runway_ft);
            if best_name.as_ref().is_none_or(|(_, s)| score > *s) {
                best_name = Some((apt, score));
            }
        }
    })?;
    out.dest = best_code.or(best_name.map(|(a, _)| a));
    Ok(out)
}

/// Inputs to the cruise-altitude rule.
#[derive(Debug, Clone)]
pub struct AltInputs {
    pub bearing_deg: f64,
    pub dist_nm: f64,
    pub origin_elev_ft: f64,
    pub dest_elev_ft: f64,
    /// Highest ground under the leg, feet MSL; `None` = no terrain data.
    pub max_terrain_ft: Option<f64>,
    pub service_ceiling_ft: Option<f64>,
    /// Pressurised (jets, turboprops): IFR whole-thousand levels, no oxygen cap.
    pub pressurised: bool,
    /// The winds aloft, when the sim could be asked. `None` keeps the rule
    /// exactly as it was — a plan made without a wind profile is still a plan.
    pub winds: Option<crate::weather::WindsAloft>,
}

/// How far from the distance-earned level the wind is allowed to move the
/// pick, and the gain it has to be worth.
///
/// The band is deliberately narrow, and the reason is that the obvious
/// formulation is wrong: "pick the level with the least headwind" would have
/// flown EIDW-BIKF at FL236, where the headwind is 45 kt lighter — and a
/// turbofan down there burns far more than the 45 kt buys back. Real practice
/// is to cruise near the efficient level and take the best wind WITHIN a step
/// or two, which is what this does. Two thousand-foot levels either side.
const WIND_SEARCH_FT: f64 = 4000.0;
/// ...and only for a real gain, so a plan does not move a level for a knot.
const WIND_WORTH_KTS: f64 = 10.0;

/// The semicircular cruising level for `bearing`: odd thousands eastbound
/// (000-179), even westbound, plus 500 for VFR piston flying.
fn level_for(k: u32, eastbound: bool, vfr: bool) -> Option<f64> {
    let odd = k % 2 == 1;
    (odd == eastbound).then_some(k as f64 * 1000.0 + if vfr { 500.0 } else { 0.0 })
}

/// The cruise altitude a careful pilot would pick for a direct leg: as high as
/// the distance earns (climb and descent must fit), capped by the ceiling and
/// the oxygen rule, never below the terrain floor, snapped to the semicircular
/// level for the direction of flight.
pub fn cruise_altitude_ft(i: &AltInputs) -> f64 {
    let fields = i.origin_elev_ft.max(i.dest_elev_ft);
    let floor = match i.max_terrain_ft {
        Some(t) => t + TERRAIN_CLEARANCE_FT,
        None => fields + NO_TERRAIN_MARGIN_FT,
    }
    .max(fields + MIN_AGL_FT);
    let mut target = fields + (i.dist_nm * FT_PER_NM).max(MIN_AGL_FT);
    if let Some(c) = i.service_ceiling_ft {
        target = target.min(c - CEILING_MARGIN_FT);
    }
    if !i.pressurised {
        target = target.min(OXYGEN_CAP_FT);
    }
    let eastbound = i.bearing_deg.rem_euclid(360.0) < 180.0;
    let vfr = !i.pressurised;
    let levels = (1..=60).filter_map(|k| level_for(k, eastbound, vfr));
    // Highest level within the target, but never below the floor.
    let mut pick = None;
    for l in levels {
        if l < floor {
            continue;
        }
        if pick.is_none() || l <= target {
            pick = Some(l);
        }
        if l > target {
            break;
        }
    }
    let pick = pick.unwrap_or(floor);
    wind_adjusted(pick, floor, target, i).unwrap_or(pick)
}

/// Among the levels within [`WIND_SEARCH_FT`] of `pick` that the rule would
/// also allow, the one with the least headwind on the route bearing — if it
/// beats `pick` by [`WIND_WORTH_KTS`].
///
/// Deliberately a nudge to the existing answer rather than a rule of its own:
/// every constraint that produced `pick` (terrain floor, ceiling, oxygen,
/// semicircular direction) still binds, and the search only reorders what was
/// already legal. `None` when there are no winds or nothing worth moving for.
fn wind_adjusted(pick: f64, floor: f64, target: f64, i: &AltInputs) -> Option<f64> {
    let winds = i.winds.as_ref()?;
    let eastbound = i.bearing_deg.rem_euclid(360.0) < 180.0;
    let vfr = !i.pressurised;
    let here = winds.headwind_at(pick, i.bearing_deg)?;
    let mut best = (pick, here);
    for l in (1..=60).filter_map(|k| level_for(k, eastbound, vfr)) {
        // The band, and every constraint that shaped the original pick. The
        // ceiling and the oxygen cap live in `target`, so staying at or under
        // it keeps them — this can only ever move DOWN from a capped level.
        if (l - pick).abs() > WIND_SEARCH_FT || l < floor || l > target.max(pick) {
            continue;
        }
        let Some(hw) = winds.headwind_at(l, i.bearing_deg) else {
            continue;
        };
        if hw < best.1 {
            best = (l, hw);
        }
    }
    (best.0 != pick && here - best.1 >= WIND_WORTH_KTS).then_some(best.0)
}

/// Fuel and time for the leg, from the profile's cruise figures. KILOGRAMS,
/// because that is the unit the burn arrives in and the unit the sim's tanks
/// are read in — a density belongs at the moment of SPEAKING, once, and
/// nowhere else. Going through gallons here meant converting kg/hr down at
/// avgas density and back up again, on airframes that burn kerosene.
#[derive(Debug, Clone, PartialEq)]
pub struct FuelPlan {
    pub ete_min: f64,
    pub trip_kg: f64,
    pub reserve_kg: f64,
    pub block_kg: f64,
}

/// `None` when the profile lacks a cruise speed or burn: the copilot then
/// says nothing about fuel rather than inventing a number.
pub fn fuel_plan(dist_nm: f64, cruise_kts: Option<f64>, burn_kg_hr: f64) -> Option<FuelPlan> {
    let cruise = cruise_kts.filter(|k| *k > 0.0)?;
    if burn_kg_hr <= 0.0 {
        return None;
    }
    let ete_min = dist_nm / cruise * 60.0 + CLIMB_ALLOWANCE_MIN;
    let trip_kg = burn_kg_hr * ete_min / 60.0;
    let reserve_kg = burn_kg_hr * RESERVE_MIN / 60.0;
    Some(FuelPlan {
        ete_min,
        trip_kg,
        reserve_kg,
        block_kg: trip_kg + reserve_kg,
    })
}

/// Points along the leg for the terrain probe, origin to destination.
pub fn route_samples(o: (f64, f64), d: (f64, f64)) -> Vec<(f64, f64)> {
    let dist = nav::haversine_nm(o.0, o.1, d.0, d.1);
    let brg = nav::bearing_deg(o.0, o.1, d.0, d.1);
    (0..TERRAIN_SAMPLES)
        .map(|i| {
            let f = i as f64 / (TERRAIN_SAMPLES - 1) as f64;
            nav::project_latlon(o.0, o.1, brg, dist * f)
        })
        .collect()
}

/// The longest runway a field has, feet (0 with no runway rows).
pub fn longest_runway_ft(a: &nav::AirportInfo) -> f64 {
    a.runways
        .iter()
        .map(|r| nav::haversine_nm(r.lat_a, r.lon_a, r.lat_b, r.lon_b) * 1852.0 * M_TO_FT)
        .fold(0.0, f64::max)
}

/// The runway the destination must offer, feet: the profile's landing
/// distance, else its takeoff distance (what every GA POH gives), else no check.
pub fn runway_needed_ft(landing_ft: Option<f64>, takeoff_ft: Option<f64>) -> Option<f64> {
    landing_ft.or(takeoff_ft).filter(|f| *f > 0.0)
}

/// A local flight joins an eight-mile final through a sixteen-mile alignment
/// fix. Cruise and descent remain ordinary procedure legs, with no new laws.
pub struct LocalRoute {
    pub runway: String,
    pub waypoints: Vec<Waypoint>,
    pub distance_nm: f64,
    /// Probe points paired with the lowest altitude the route may fly there.
    pub terrain: Vec<((f64, f64), f64)>,
}

pub fn local_route(
    fp: &FlightPlan,
    origin: &nav::AirportInfo,
    dest: &nav::AirportInfo,
    cruise_kts: f64,
    climb_fpm: f64,
    runway_needed_ft: f64,
    wind: Option<(f64, f64)>,
) -> Result<LocalRoute, String> {
    let o = fp
        .waypoints
        .first()
        .ok_or("The local plan has no origin.")?;
    let cruise = fp.cruise_alt_ft;
    if ![cruise, cruise_kts, climb_fpm, origin.elev_ft, dest.elev_ft]
        .iter()
        .all(|v| v.is_finite())
        || cruise_kts <= 0.0
        || climb_fpm <= 0.0
        || cruise < dest.elev_ft + 2500.0
    {
        return Err("The local flight needs usable climb, cruise and arrival figures.".into());
    }
    let mut ends = Vec::new();
    for r in &dest.runways {
        let length = nav::haversine_nm(r.lat_a, r.lon_a, r.lat_b, r.lon_b) * 1852.0 * M_TO_FT;
        if length < runway_needed_ft {
            continue;
        }
        for (id, lat, lon, heading) in [
            (&r.ident_a, r.lat_a, r.lon_a, r.heading_a_deg()),
            (&r.ident_b, r.lat_b, r.lon_b, r.heading_b_deg()),
        ] {
            let target = wind
                .filter(|(_, k)| *k >= 4.0)
                .map(|(d, _)| d)
                .unwrap_or_else(|| nav::bearing_deg(o.lat, o.lon, lat, lon));
            let score = ((heading - target + 540.0).rem_euclid(360.0) - 180.0).abs();
            ends.push((score, id.clone(), lat, lon, heading));
        }
    }
    ends.sort_by(|a, b| a.0.total_cmp(&b.0));
    let (_, runway, lat, lon, heading) = ends
        .into_iter()
        .next()
        .ok_or("The destination has no runway long enough for this local flight.")?;
    let outer = nav::project_latlon(lat, lon, heading + 180.0, 16.0);
    let final_fix = nav::project_latlon(lat, lon, heading + 180.0, 8.0);
    let dist = nav::haversine_nm(o.lat, o.lon, outer.0, outer.1);
    let outer_alt = cruise.min(dest.elev_ft + 4900.0);
    // Reserve descent at 250 ft/nm, gentler than the follower's nominal path,
    // and five miles for the turn onto final.
    let descent_nm = ((cruise - outer_alt) / 250.0).max(0.0) + 5.0;
    let climb_nm = ((cruise - origin.elev_ft - 500.0).max(0.0) / (climb_fpm * 0.7))
        * (cruise_kts + 30.0)
        / 60.0
        + 3.0;
    if dist < climb_nm + descent_nm {
        return Err("This local route has too little room for the climb, descent and final turn. Choose a farther destination or a lower cruise altitude.".into());
    }
    let top = nav::project_latlon(
        o.lat,
        o.lon,
        nav::bearing_deg(o.lat, o.lon, outer.0, outer.1),
        dist - descent_nm,
    );
    let wp = |id: &str, p: (f64, f64), alt| Waypoint {
        ident: id.into(),
        lat: p.0,
        lon: p.1,
        alt,
    };
    let waypoints = vec![
        o.clone(),
        wp("XVLCR", top, cruise),
        wp("XVLIF", outer, outer_alt),
        wp("XVLFA", final_fix, cruise.min(dest.elev_ft + 2500.0)),
        wp(&fp.destination, (lat, lon), dest.elev_ft),
    ];
    let mut terrain = Vec::new();
    let mut distance_nm = 0.0;
    for (leg, pair) in waypoints.windows(2).enumerate() {
        let a = &pair[0];
        let b = &pair[1];
        let d = nav::haversine_nm(a.lat, a.lon, b.lat, b.lon);
        distance_nm += d;
        // Final eight miles belong to the proven autoland path; verify that
        // corridor too, with a 3-degree path and a 50-foot threshold crossing.
        let n = (d * 1852.0 / 150.0).ceil().max(1.0) as usize;
        for i in 0..=n {
            let f = i as f64 / n as f64;
            let p = nav::project_latlon(
                a.lat,
                a.lon,
                nav::bearing_deg(a.lat, a.lon, b.lat, b.lon),
                d * f,
            );
            let floor = if leg == 0 {
                // Conservative reachable climb; the takeoff supplies the
                // first 500 feet, before the follower owns any lateral turn.
                (origin.elev_ft
                    + 500.0
                    + (d * f - 3.0).max(0.0) / (cruise_kts + 30.0) * 60.0 * climb_fpm * 0.7)
                    .min(cruise)
                    - 500.0
            } else if leg == 3 {
                dest.elev_ft + 50.0 + d * (1.0 - f) * 300.0
            } else {
                // Lowest of a continuous descent and its end constraint.
                b.alt.min(a.alt) - 1000.0
            };
            terrain.push((p, floor));
        }
    }
    Ok(LocalRoute {
        runway,
        waypoints,
        distance_nm,
        terrain,
    })
}

/// Every point must be measured, finite, and below the route's clearance floor.
pub fn local_terrain_clear(route: &LocalRoute, heights: &[f64]) -> bool {
    heights.len() == route.terrain.len()
        && heights
            .iter()
            .zip(&route.terrain)
            .all(|(h, (_, floor))| h.is_finite() && *h <= *floor)
}

/// Everything the brief and the plan are built from.
#[derive(Debug, Clone)]
pub struct Planned {
    pub origin: Airport,
    pub dest: Airport,
    pub dest_matches: usize,
    pub dist_nm: f64,
    pub bearing_deg: f64,
    pub cruise_alt_ft: f64,
    pub max_terrain_ft: Option<f64>,
    pub fuel: Option<FuelPlan>,
    pub cruise_kts: Option<f64>,
    /// Fuel in the tanks now, gallons; `None` = unreadable.
    /// What is actually in the tanks, KILOGRAMS — spoken in the airframe's
    /// own unit like every other fuel figure.
    pub aboard_kg: Option<f64>,
    pub aircraft: String,
}

/// Thousands separators for an already-converted fuel figure.
fn commas_f(n: f64) -> String {
    crate::auto_status::commas(n.round() as i64)
}

impl Planned {
    /// The plan in SimBrief's shape, so every downstream consumer is shared.
    pub fn to_flight_plan(&self) -> FlightPlan {
        let f = self.fuel.as_ref();
        FlightPlan {
            source: PlanSource::Local,
            origin: self.origin.icao.clone(),
            destination: self.dest.icao.clone(),
            alternate: String::new(),
            aircraft: self.aircraft.clone(),
            cruise_alt_ft: self.cruise_alt_ft,
            distance_nm: self.dist_nm,
            ete_min: f.map(|f| f.ete_min).unwrap_or(0.0),
            trip_fuel_kg: f.map(|f| f.trip_kg).unwrap_or(0.0),
            reserve_kg: f.map(|f| f.reserve_kg).unwrap_or(0.0),
            block_fuel_kg: f.map(|f| f.block_kg).unwrap_or(0.0),
            min_takeoff_fuel_kg: f.map(|f| f.trip_kg + f.reserve_kg).unwrap_or(0.0),
            weights: None,
            route: "direct".to_string(),
            wind_comp_kts: 0.0,
            wind_dir: 0.0,
            wind_spd_kts: 0.0,
            tod_to_dest_nm: 0.0,
            waypoints: vec![
                Waypoint {
                    ident: self.origin.icao.clone(),
                    lat: self.origin.lat,
                    lon: self.origin.lon,
                    alt: 0.0,
                },
                Waypoint {
                    ident: self.dest.icao.clone(),
                    lat: self.dest.lat,
                    lon: self.dest.lon,
                    alt: 0.0,
                },
            ],
        }
    }

    /// The spoken brief: where, how far, how high, how long, fuel against the
    /// tanks, the ground on the way, the runway at the far end. Ends by
    /// telling the pilot the next word unless the plan is already loading.
    pub fn brief(&self, loading: bool) -> String {
        let mut s = format!(
            "Direct to {}, {}, {:.0} miles on a heading of {:03.0}.",
            self.dest.name,
            spell_code(&self.dest.icao),
            self.dist_nm,
            self.bearing_deg
        );
        if self.dest_matches > 1 {
            s.push_str(&format!(
                " There are {} fields by that name; I took the biggest.",
                self.dest_matches
            ));
        }
        s.push_str(&format!(" Cruise {}.", spoken_feet(self.cruise_alt_ft)));
        match (&self.fuel, self.cruise_kts) {
            (Some(f), Some(k)) => {
                s.push_str(&format!(
                    " About {} at {k:.0} knots. Trip fuel {}, {} with a {} minute reserve",
                    spoken_minutes(f.ete_min),
                    crate::aircraft::say_fuel_kg(f.trip_kg),
                    crate::aircraft::fuel_figure_kg(f.block_kg),
                    RESERVE_MIN
                ));
                // Rounded into the airframe's unit BEFORE the subtraction, so
                // the figures the pilot hears agree with each other.
                match self.aboard_kg {
                    Some(kg) => {
                        let a = crate::aircraft::fuel_in_unit_kg(kg).round();
                        let need = crate::aircraft::fuel_in_unit_kg(f.block_kg).round();
                        if a >= need {
                            s.push_str(&format!("; {} aboard, enough.", commas_f(a)));
                        } else {
                            s.push_str(&format!(
                                "; {} aboard, short by {}. Fuel before we go.",
                                commas_f(a),
                                commas_f(need - a)
                            ));
                        }
                    }
                    None => s.push('.'),
                }
            }
            _ => s.push_str(" No cruise figures in the profile, so no time or fuel estimate."),
        }
        if let Some(t) = self.max_terrain_ft {
            if t > self.origin.elev_ft.max(self.dest.elev_ft) + 1000.0 {
                s.push_str(&format!(" High ground to {} on the way.", spoken_feet(t)));
            }
        }
        if self.dest.longest_runway_ft > 0.0 {
            s.push_str(&format!(
                " Longest runway there {}.",
                spoken_feet(self.dest.longest_runway_ft)
            ));
        }
        if !loading {
            s.push_str(" Say load it and I'll put it in the GPS.");
        }
        s
    }
}

/// The refusal for a field the aircraft cannot land at. `None` = fine.
pub fn runway_refusal(dest: &Airport, needed_ft: Option<f64>) -> Option<String> {
    let need = needed_ft?;
    if dest.longest_runway_ft <= 0.0 || dest.longest_runway_ft >= need {
        return None;
    }
    Some(format!(
        "Negative. {}'s longest runway is {}, and we need {} to land. Not planning that.",
        dest.name,
        spoken_feet(dest.longest_runway_ft),
        spoken_feet(need)
    ))
}

/// ICAO codes read letter by letter ("L E M G"): TTS says a bare "LEMG" as
/// a word.
pub fn spell_code(code: &str) -> String {
    code.chars()
        .map(|c| c.to_ascii_uppercase().to_string())
        .collect::<Vec<_>>()
        .join(" ")
}

pub fn spoken_feet(ft: f64) -> String {
    let n = ft.round() as i64;
    let digits = n.abs().to_string();
    let bytes = digits.as_bytes();
    let mut out = String::new();
    for (i, c) in bytes.iter().enumerate() {
        if i > 0 && (bytes.len() - i).is_multiple_of(3) {
            out.push(',');
        }
        out.push(*c as char);
    }
    format!("{out} feet")
}

fn spoken_minutes(min: f64) -> String {
    let m = min.round() as i64;
    if m >= 60 {
        let (h, r) = (m / 60, m % 60);
        if r == 0 {
            format!("{h} hour{}", if h == 1 { "" } else { "s" })
        } else {
            format!("{h} hour{} {r} minutes", if h == 1 { "" } else { "s" })
        }
    } else {
        format!("{m} minutes")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dest(t: &str) -> Destination {
        parse_request(t).unwrap().dest.unwrap()
    }

    #[test]
    fn request_shapes_parse_and_the_rest_fall_through() {
        assert_eq!(dest("plan a flight to Malaga").name, "malaga");
        assert_eq!(dest("Divert to Gibraltar.").name, "gibraltar");
        assert_eq!(dest("could you take us to Jerez please").name, "jerez");
        assert_eq!(dest("let's go to Málaga airport").name, "malaga");
        let d = dest("plan a flight to lima echo mike golf");
        assert_eq!(d.codes, vec!["LEMG"]);
        assert!(d.name.is_empty());
        assert_eq!(dest("take us to L E M G").codes, vec!["LEMG"]);
        assert_eq!(dest("fly to x-ray alpha bravo charlie").codes, vec!["XABC"]);
        assert_eq!(
            dest("divert to sierra foxtrot oscar").codes,
            vec!["SFO", "KSFO"]
        );
        // A single short token is both a code and a name.
        let d = dest("plan a flight to LEMG");
        assert_eq!(d.codes, vec!["LEMG"]);
        assert_eq!(d.name, "lemg");
        assert_eq!(dest("plan a flight to Nice").name, "nice");
        // Load in the same breath.
        let r = parse_request("plan a flight to Malaga and load it").unwrap();
        assert!(r.load);
        assert_eq!(r.dest.unwrap().name, "malaga");
        // Bare: ask where.
        let r = parse_request("divert").unwrap();
        assert!(r.dest.is_none());
        assert!(parse_request("plan a flight").unwrap().dest.is_none());
        // Not planning requests.
        for t in [
            "load the flight plan",
            "fly my flight plan",
            "brief the flight plan",
            "take us to 5000 feet",
            "take us to flight level one two zero",
            "fly to heading 270",
            "flaps ten",
            "what's our altitude",
            "navigate to the next waypoint",
            "plan format",
        ] {
            assert!(parse_request(t).is_none(), "{t}");
        }
    }

    #[test]
    fn destination_answers_to_where() {
        assert_eq!(parse_destination("Gibraltar").unwrap().name, "gibraltar");
        assert_eq!(
            parse_destination("lima x-ray golf bravo").unwrap().codes,
            vec!["LXGB"]
        );
        assert!(parse_destination("flaps ten").is_none());
        assert!(parse_destination("").is_none());
    }

    /// The EIDW-BIKF wind profile as the sim published it (2026-09-20), in
    /// the sim's own units: thirteen layers, meters and meters per second.
    fn atlantic_jetstream() -> crate::weather::WindsAloft {
        crate::weather::WindsAloft::from_layers(
            &[
                0.0, 540.1, 988.5, 1948.3, 3010.8, 4206.5, 5572.0, 7182.3, 9160.2, 10362.9,
                11887.2, 13594.1, 16179.4,
            ],
            &[
                4.70, 5.17, 5.15, 6.15, 9.23, 13.36, 19.49, 31.21, 50.88, 61.16, 57.40, 39.84,
                19.07,
            ],
            &[
                274.7, 274.7, 263.4, 283.8, 277.7, 268.4, 268.6, 279.6, 280.9, 278.9, 284.9, 282.9,
                279.6,
            ],
        )
        .expect("thirteen good layers")
    }

    /// The wind moves the level, but only within the band and only for a real
    /// gain — and it never reaches for the calm air far below, which is the
    /// mistake the narrow band exists to prevent.
    #[test]
    fn the_cruise_level_takes_the_best_wind_it_can_afford() {
        let jet = AltInputs {
            bearing_deg: 317.0,
            dist_nm: 900.0,
            origin_elev_ft: 200.0,
            dest_elev_ft: 171.0,
            max_terrain_ft: Some(3000.0),
            service_ceiling_ft: Some(41_000.0),
            pressurised: true,
            winds: None,
        };
        let still = cruise_altitude_ft(&jet);
        let windy = cruise_altitude_ft(&AltInputs {
            winds: Some(atlantic_jetstream()),
            ..jet.clone()
        });
        assert!(
            (windy - still).abs() <= WIND_SEARCH_FT,
            "the wind nudges the rule, it does not replace it: {still} -> {windy}"
        );
        // FL236 is 45 kt calmer than FL340 and 10,000 ft below the band. A
        // "least headwind" rule would have gone there and burned more.
        assert!(
            windy > 28_000.0,
            "never drops out of efficient air for wind alone: {windy}"
        );
        // And the level it picks really is no worse on wind than the one the
        // distance alone earned.
        let w = atlantic_jetstream();
        let (a, b) = (
            w.headwind_at(still, 317.0).unwrap(),
            w.headwind_at(windy, 317.0).unwrap(),
        );
        assert!(b <= a, "the pick is never a worse wind: {a:.0} -> {b:.0}");
    }

    /// The jet stream this flight actually met: a core across FL300-FL390
    /// where climbing buys nothing, and calm air far below where descending
    /// would cost more than it saves. Both readings come out of the profile.
    #[test]
    fn the_wind_profile_reads_the_core_and_the_calm_beneath_it() {
        let w = atlantic_jetstream();
        let hw = |ft: f64| w.headwind_at(ft, 317.3).unwrap();
        assert!(
            hw(34_000.0) > 90.0,
            "FL340 is in the core: {:.0}",
            hw(34_000.0)
        );
        assert!(hw(39_000.0) > 90.0, "so is FL390: {:.0}", hw(39_000.0));
        assert!(
            (hw(39_000.0) - hw(34_000.0)).abs() < 5.0,
            "climbing through the core buys nothing: {:.0} vs {:.0}",
            hw(34_000.0),
            hw(39_000.0)
        );
        assert!(hw(23_600.0) < 55.0, "calm beneath: {:.0}", hw(23_600.0));
        // A tailwind is negative, so the sign survives the round trip.
        assert!(w.headwind_at(34_000.0, 99.0).unwrap() < -90.0, "reciprocal");
    }

    #[test]
    fn altitude_rule_is_semicircular_terrain_floored_and_capped() {
        let base = AltInputs {
            bearing_deg: 85.0,
            dist_nm: 60.0,
            origin_elev_ft: 15.0,
            dest_elev_ft: 52.0,
            max_terrain_ft: Some(1400.0),
            service_ceiling_ft: Some(14_000.0),
            pressurised: false,
            // No winds: every assertion below is the rule exactly as it was.
            winds: None,
        };
        // 60 nm earns ~4,800: eastbound VFR level under that is 3,500 (floor 3,400).
        assert_eq!(cruise_altitude_ft(&base), 3500.0);
        // Westbound: even + 500.
        let w = AltInputs {
            bearing_deg: 265.0,
            ..base.clone()
        };
        assert_eq!(cruise_altitude_ft(&w), 4500.0);
        // Terrain pushes the floor above what the distance earns.
        let t = AltInputs {
            max_terrain_ft: Some(6200.0),
            ..base.clone()
        };
        assert_eq!(
            cruise_altitude_ft(&t),
            9500.0,
            "floor 8,200: first eastbound level above"
        );
        // Long leg on a piston: oxygen cap.
        let long = AltInputs {
            dist_nm: 400.0,
            ..base.clone()
        };
        assert_eq!(cruise_altitude_ft(&long), 11_500.0);
        // Ceiling cap beats distance.
        let low = AltInputs {
            dist_nm: 400.0,
            service_ceiling_ft: Some(9000.0),
            ..base.clone()
        };
        assert_eq!(cruise_altitude_ft(&low), 7500.0);
        // Pressurised: whole thousands, past 12,500.
        let jet = AltInputs {
            dist_nm: 400.0,
            service_ceiling_ft: Some(51_000.0),
            pressurised: true,
            ..base.clone()
        };
        assert_eq!(cruise_altitude_ft(&jet), 31_000.0);
        // No terrain data: fields + 3,000 floor, never below.
        let blind = AltInputs {
            max_terrain_ft: None,
            dist_nm: 10.0,
            ..base.clone()
        };
        assert_eq!(cruise_altitude_ft(&blind), 3500.0);
    }

    #[test]
    fn fuel_from_profile_or_nothing() {
        // 24 kg/hr for 40 minutes, plus a 45 minute reserve — in KILOS, the
        // unit the burn arrives in. The old assertions were the same fuel put
        // through avgas density and back.
        let f = fuel_plan(62.0, Some(124.0), 24.0).unwrap();
        assert!((f.ete_min - 40.0).abs() < 0.1);
        assert!((f.trip_kg - 16.0).abs() < 0.1, "{}", f.trip_kg);
        assert!((f.reserve_kg - 18.0).abs() < 0.1, "{}", f.reserve_kg);
        assert!((f.block_kg - 34.0).abs() < 0.1, "{}", f.block_kg);
        assert!(fuel_plan(62.0, None, 24.0).is_none());
        assert!(fuel_plan(62.0, Some(124.0), 0.0).is_none());
    }

    const APT: &str = "I\n1200 Generated\n\n\
1 15 0 0 LXGB Gibraltar\n\
100 45.11 1 0 0.25 0 2 0 09 36.1500 -5.3550 0 0 2 0 0 1 27 36.1520 -5.3360 0 0 2 0 0 1\n\
1054 118500 TWR Gibraltar Tower\n\
1 52 0 0 LEMG Malaga Costa del Sol\n\
100 45.11 1 0 0.25 0 2 0 13 36.6850 -4.5100 0 0 2 0 0 1 31 36.6620 -4.4780 0 0 2 0 0 1\n\
1054 118150 TWR Malaga Tower\n\
1 1200 0 0 XX01 Malaga Farm Strip\n\
100 10.00 1 0 0.25 0 2 0 09 37.0000 -4.0000 0 0 2 0 0 1 27 37.0010 -3.9950 0 0 2 0 0 1\n\
17 100 0 0 XH01 Malaga Heliport\n\
102 H1 36.7 -4.4 0 20 20 1 0 0 0 0 0\n\
1 20 0 0 KSFO San Francisco Intl\n\
100 60.00 1 0 0.25 0 2 0 10L 37.6280 -122.3930 0 0 2 0 0 1 28R 37.6130 -122.3570 0 0 2 0 0 1\n\
99\n";

    fn fixture() -> String {
        let p = std::env::temp_dir().join(format!("xvl_apt_{}.dat", std::process::id()));
        std::fs::write(&p, APT).unwrap();
        p.to_string_lossy().into_owned()
    }

    #[test]
    fn resolves_names_codes_and_the_nearest_origin() {
        let p = fixture();
        let here = (36.15, -5.35); // on the Rock
        let r = resolve(&p, &dest("plan a flight to Malaga"), here).unwrap();
        let d = r.dest.unwrap();
        assert_eq!(
            d.icao, "LEMG",
            "towered field with the long runway wins the name"
        );
        assert_eq!(
            r.dest_matches, 2,
            "the farm strip matched too; the heliport never"
        );
        assert_eq!(r.origin.unwrap().icao, "LXGB");
        assert!(
            d.longest_runway_ft > 9000.0 && d.longest_runway_ft < 14_000.0,
            "{}",
            d.longest_runway_ft
        );
        // Spelled code, exact.
        let r = resolve(&p, &dest("divert to lima echo mike golf"), here).unwrap();
        assert_eq!(r.dest.unwrap().icao, "LEMG");
        // Three letters: the K-prefixed US field.
        let r = resolve(&p, &dest("take us to sierra foxtrot oscar"), here).unwrap();
        assert_eq!(r.dest.unwrap().icao, "KSFO");
        // Unknown.
        let r = resolve(&p, &dest("plan a flight to Narnia"), here).unwrap();
        assert!(r.dest.is_none());
        assert!(r.origin.is_some());
        let _ = std::fs::remove_file(&p);
    }

    #[test]
    fn local_route_has_flyable_legs_and_refuses_missing_or_obstructed_terrain() {
        let mut fp = planned().to_flight_plan();
        fp.cruise_alt_ft = 3500.0;
        fp.waypoints[0].lat = 0.0;
        fp.waypoints[0].lon = 0.0;
        fp.waypoints[1].lat = 1.2;
        fp.waypoints[1].lon = 0.0;
        let airport = nav::AirportInfo {
            name: "Flat field".into(),
            elev_ft: 0.0,
            freqs: vec![],
            runways: vec![nav::Runway {
                ident_a: "36".into(),
                ident_b: "18".into(),
                lat_a: 1.2,
                lon_a: 0.0,
                lat_b: 1.23,
                lon_b: 0.0,
                width_m: 30.0,
            }],
        };
        let route = local_route(&fp, &airport, &airport, 100.0, 700.0, 2000.0, None).unwrap();
        assert_eq!(route.runway, "36");
        let legs = crate::procedure::enroute_legs(
            &route.waypoints,
            &fp.origin,
            &fp.destination,
            fp.cruise_alt_ft,
        );
        assert_eq!(legs.len(), 3);
        assert!(legs.iter().all(|l| l.fix.is_some()));
        assert_eq!(legs.last().unwrap().alt1_ft, Some(2500.0));
        let mut heights = vec![0.0; route.terrain.len()];
        assert!(local_terrain_clear(&route, &heights));
        assert!(!local_terrain_clear(&route, &heights[..heights.len() - 1]));
        heights[10] = route.terrain[10].1 + 1.0;
        assert!(!local_terrain_clear(&route, &heights));
        heights[10] = f64::NAN;
        assert!(!local_terrain_clear(&route, &heights));
        assert!(local_route(&fp, &airport, &airport, 100.0, 20.0, 2000.0, None).is_err());
        assert!(local_route(&fp, &airport, &airport, 100.0, 700.0, 100000.0, None).is_err());
        let opposite = local_route(
            &fp,
            &airport,
            &airport,
            100.0,
            700.0,
            2000.0,
            Some((180.0, 10.0)),
        )
        .unwrap();
        assert_eq!(opposite.runway, "18");
        assert!(opposite.distance_nm > route.distance_nm);
    }

    #[test]
    fn local_shelf_survives_until_an_explicit_replacement_is_published() {
        let mut fp = planned().to_flight_plan();
        crate::simbrief::publish_plan(&fp);
        assert_eq!(
            crate::simbrief::local_plan().unwrap().destination,
            fp.destination
        );
        fp.source = PlanSource::SimBrief;
        crate::simbrief::publish_plan(&fp);
        assert!(crate::simbrief::local_plan().is_none());
    }

    fn planned() -> Planned {
        Planned {
            origin: Airport {
                icao: "LXGB".into(),
                name: "Gibraltar".into(),
                lat: 36.15,
                lon: -5.35,
                elev_ft: 15.0,
                longest_runway_ft: 5500.0,
                towered: true,
            },
            dest: Airport {
                icao: "LEMG".into(),
                name: "Malaga Costa del Sol".into(),
                lat: 36.67,
                lon: -4.49,
                elev_ft: 52.0,
                longest_runway_ft: 10_500.0,
                towered: true,
            },
            dest_matches: 1,
            dist_nm: 62.0,
            bearing_deg: 52.0,
            cruise_alt_ft: 3500.0,
            max_terrain_ft: Some(2800.0),
            fuel: fuel_plan(62.0, Some(124.0), 25.0),
            cruise_kts: Some(124.0),
            aboard_kg: Some(40.0 * 2.72),
            aircraft: "Cessna 172SP".into(),
        }
    }

    #[test]
    fn brief_reads_the_numbers_and_the_shortfall() {
        let p = planned();
        let b = p.brief(false);
        assert!(
            b.starts_with("Direct to Malaga Costa del Sol, L E M G, 62 miles on a heading of 052."),
            "{b}"
        );
        assert!(b.contains("Cruise 3,500 feet."), "{b}");
        assert!(b.contains("About 40 minutes at 124 knots."), "{b}");
        assert!(b.contains("40 aboard, enough."), "{b}");
        assert!(b.contains("High ground to 2,800 feet"), "{b}");
        assert!(b.contains("Longest runway there 10,500 feet."), "{b}");
        assert!(
            b.ends_with("Say load it and I'll put it in the GPS."),
            "{b}"
        );
        let short = Planned {
            aboard_kg: Some(8.0 * 2.72),
            ..planned()
        };
        let b = short.brief(true);
        assert!(
            b.contains("8 aboard, short by 5. Fuel before we go."),
            "{b}"
        );
        assert!(!b.contains("Say load it"));
        let blind = Planned {
            fuel: None,
            cruise_kts: None,
            ..planned()
        };
        assert!(blind.brief(false).contains("no time or fuel estimate"));
        assert!(!blind.brief(false).contains("gallons"));
    }

    #[test]
    fn plan_shape_matches_simbriefs_and_the_runway_gate_refuses() {
        let fp = planned().to_flight_plan();
        assert_eq!(fp.source, PlanSource::Local);
        assert_eq!(fp.waypoints.len(), 2);
        assert_eq!(fp.waypoints[0].ident, "LXGB");
        assert_eq!(fp.waypoints[1].ident, "LEMG");
        assert_eq!(fp.route, "direct");
        assert!(fp.summary().contains("LXGB to LEMG"));
        let d = planned().dest;
        assert!(runway_refusal(&d, Some(1335.0)).is_none());
        let strip = Airport {
            longest_runway_ft: 900.0,
            ..d.clone()
        };
        let r = runway_refusal(&strip, Some(1335.0)).unwrap();
        assert!(r.starts_with("Negative."), "{r}");
        assert!(r.contains("900 feet") && r.contains("1,335 feet"), "{r}");
        assert!(runway_refusal(&strip, None).is_none(), "no figure, no gate");
        assert_eq!(runway_needed_ft(None, Some(1630.0)), Some(1630.0));
        assert_eq!(runway_needed_ft(Some(1335.0), Some(1630.0)), Some(1335.0));
    }

    #[test]
    fn samples_span_the_leg() {
        let pts = route_samples((36.15, -5.35), (36.67, -4.49));
        assert_eq!(pts.len(), TERRAIN_SAMPLES);
        assert!((pts[0].0 - 36.15).abs() < 1e-6);
        let last = pts[TERRAIN_SAMPLES - 1];
        assert!((last.0 - 36.67).abs() < 0.01 && (last.1 + 4.49).abs() < 0.01);
    }
}
